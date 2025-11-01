/**
 * @file takeoff.cpp
 * @brief High-level takeoff control logic for ArduCopter flight modes
 * 
 * @details This file implements the takeoff state machine and control algorithms
 *          for both pilot-initiated and automated takeoffs in various flight modes
 *          (Loiter, PosHold, AltHold, Sport, Auto, Guided).
 * 
 *          Key functionality includes:
 *          - MAVLink NAV_TAKEOFF command handling
 *          - Takeoff state management and sequencing
 *          - Smooth climb rate ramping for gentle takeoffs
 *          - Pilot override detection during automated takeoffs
 *          - Takeoff completion detection based on altitude achievement
 *          - Safety checks for motor arming, interlock, and ground detection
 * 
 *          The takeoff process uses a state machine that transitions through:
 *          1. Pre-takeoff (land_complete=true, motors spooling up)
 *          2. Liftoff detection (monitoring throttle, acceleration, velocity)
 *          3. Climb phase (position controller active, ramping to target altitude)
 *          4. Completion (within altitude tolerance and velocity threshold)
 * 
 * @note This implementation is called at the main loop rate (typically 400Hz)
 * @warning Modifications to takeoff logic can affect vehicle stability during
 *          the critical liftoff phase. All changes must be thoroughly tested
 *          in SITL and on actual hardware.
 * 
 * @see Mode::_TakeOff, _AutoTakeoff
 * 
 * Source: ArduCopter/takeoff.cpp
 */

#include "Copter.h"

Mode::_TakeOff Mode::takeoff;
_AutoTakeoff Mode::auto_takeoff;

/**
 * @brief Initiate the user-commanded takeoff sequence
 * 
 * @details This function starts the takeoff state machine by calling the
 *          internal takeoff.start() method with the specified altitude.
 *          It is called after all pre-takeoff validation checks have passed.
 * 
 * @param[in] takeoff_alt_cm Target takeoff altitude in centimeters relative
 *                           to current position (not absolute altitude)
 * 
 * @return true always (takeoff initiation successful)
 * 
 * @note This is a simple wrapper function that can be overridden by specific
 *       flight modes if custom takeoff initialization is required
 * 
 * @see do_user_takeoff(), _TakeOff::start()
 */
bool Mode::do_user_takeoff_start(float takeoff_alt_cm)
{
    copter.flightmode->takeoff.start(takeoff_alt_cm);
    return true;
}

/**
 * @brief Initiate user-commanded takeoff with comprehensive safety validation
 * 
 * @details This function is called when a MAVLink NAV_TAKEOFF command is received
 *          from the ground control station. It performs extensive pre-flight safety
 *          checks before initiating the takeoff sequence.
 * 
 *          Safety checks performed (in order):
 *          1. Motors must be armed
 *          2. Vehicle must be on the ground (land_complete=true)
 *          3. Current flight mode must support user takeoff
 *          4. Target altitude must be above current altitude (no downward takeoffs)
 *          5. Motor interlock must be enabled (for helicopters and interlock-equipped vehicles)
 * 
 *          If all checks pass, the takeoff state machine is started and the vehicle
 *          is set to auto-armed state to enable motor control.
 * 
 * @param[in] takeoff_alt_cm Absolute target altitude in centimeters (relative to home)
 *                           for the takeoff maneuver
 * @param[in] must_navigate  If true, the flight mode must support navigation during
 *                           takeoff (e.g., GPS-based modes). If false, altitude-only
 *                           modes (like AltHold) are acceptable.
 * 
 * @return true if takeoff successfully initiated, false if any safety check fails
 * 
 * @note This function does NOT command throttle or begin the climb - it only
 *       initializes the takeoff state machine. The actual takeoff control is
 *       performed by do_pilot_takeoff() or auto_takeoff.run() in subsequent loops.
 * 
 * @warning Attempting takeoff with motor interlock disabled on helicopters will
 *          fail this check and return false to prevent unsafe operation.
 * 
 * @see do_user_takeoff_start(), has_user_takeoff(), _TakeOff::start()
 */
bool Mode::do_user_takeoff(float takeoff_alt_cm, bool must_navigate)
{
    // Safety check 1: Motors must be armed before attempting takeoff
    if (!copter.motors->armed()) {
        return false;
    }
    
    // Safety check 2: Vehicle must be on the ground to initiate takeoff
    if (!copter.ap.land_complete) {
        // can't takeoff again!
        return false;
    }
    
    // Safety check 3: Verify current flight mode supports user-initiated takeoff
    if (!has_user_takeoff(must_navigate)) {
        // this mode doesn't support user takeoff
        return false;
    }
    
    // Safety check 4: Validate takeoff altitude is above current position
    if (takeoff_alt_cm <= copter.current_loc.alt) {
        // can't takeoff downwards...
        return false;
    }

    // Safety check 5: For vehicles with motor interlock, verify it is enabled
    // Vehicles using motor interlock should return false if motor interlock is disabled.
    // Interlock must be enabled to allow the controller to spool up the motor(s) for takeoff.
    if (!motors->get_interlock() && copter.ap.using_interlock) {
        return false;
    }

    // All safety checks passed - initiate takeoff sequence
    if (!do_user_takeoff_start(takeoff_alt_cm)) {
        return false;
    }

    // Enable auto-armed state to allow motor control during takeoff
    copter.set_auto_armed(true);
    return true;
}

/**
 * @brief Initialize the takeoff state machine with target altitude
 * 
 * @details This function sets up the takeoff state by recording the starting
 *          altitude and calculating the target completion altitude. The takeoff
 *          is considered a relative climb from the current desired position.
 * 
 *          State initialization:
 *          - Sets _running flag to true (enables takeoff processing)
 *          - Records starting altitude from position controller
 *          - Calculates completion altitude (start + climb distance)
 * 
 *          The starting altitude is obtained from the position controller's
 *          desired position rather than measured position to ensure smooth
 *          transitions if the controller is already active.
 * 
 * @param[in] alt_cm Relative altitude change in centimeters from current position.
 *                   This is added to the current desired altitude to determine
 *                   the takeoff target altitude.
 * 
 * @note This function only initializes state variables - it does not command
 *       throttle or begin the climb. The actual takeoff control loop is in
 *       do_pilot_takeoff().
 * 
 * @note Called at takeoff initiation, not at the main loop rate
 * 
 * @see stop(), do_pilot_takeoff()
 */
void Mode::_TakeOff::start(float alt_cm)
{
    // initialise takeoff state
    _running = true;
    take_off_start_alt = copter.pos_control->get_pos_desired_U_cm();
    take_off_complete_alt  = take_off_start_alt + alt_cm;
}

/**
 * @brief Terminate the takeoff sequence and update ground detection state
 * 
 * @details This function stops the active takeoff by clearing the _running flag
 *          and intelligently manages the land_complete status based on throttle level.
 * 
 *          Takeoff termination logic:
 *          1. Clear _running flag to disable takeoff processing
 *          2. Check if throttle indicates vehicle may have lifted off
 *          3. If throttle > non-takeoff threshold, mark vehicle as airborne
 * 
 *          This prevents a race condition where takeoff is stopped (e.g., by pilot
 *          commanding negative climb rate) after the vehicle has begun to lift off
 *          but before the climb detection logic has triggered. Without this check,
 *          the vehicle might incorrectly remain in land_complete state despite
 *          being airborne.
 * 
 * @note This function can be called at any point during the takeoff sequence,
 *       either by pilot override (negative climb rate) or by reaching target altitude
 * 
 * @note Called during takeoff execution when termination conditions are met
 * 
 * @see start(), do_pilot_takeoff(), get_non_takeoff_throttle()
 */
void Mode::_TakeOff::stop()
{
    _running = false;
    // Check if we have progressed far enough through the takeoff process that the
    // aircraft may have left the ground but not yet detected the climb.
    if (copter.attitude_control->get_throttle_in() > copter.get_non_takeoff_throttle()) {
        copter.set_land_complete(false);
    }
}

/**
 * @brief Execute pilot-controlled takeoff with smooth climb rate ramping
 * 
 * @details This function implements the main takeoff control loop for pilot-initiated
 *          takeoffs in modes like Loiter, PosHold, and AltHold. It manages a two-phase
 *          takeoff sequence with intelligent liftoff detection and pilot override capability.
 * 
 *          TAKEOFF STATE MACHINE - Two distinct phases:
 * 
 *          PHASE 1: Pre-Liftoff (land_complete = true)
 *          ---------------------------------------------
 *          - Gradual throttle ramping from current throttle to target
 *          - Ramp rate controlled by TKOFF_SLEW_TIME parameter (default 2 seconds)
 *          - Throttle applied directly via attitude controller (bypasses position controller)
 *          - Position controller vertical axis continuously reset to prevent I-term windup
 * 
 *          Liftoff Detection Criteria (ANY condition triggers transition to Phase 2):
 *          1. Throttle reaches 90% (or TKOFF_THR_MAX if lower)
 *          2. Vertical acceleration exceeds 50% of maximum capability
 *          3. Vertical velocity reaches 10-50% of maximum climb rate (pilot dependent)
 *          4. Altitude climbed exceeds 50% of target altitude
 * 
 *          PHASE 2: Active Climb (land_complete = false)
 *          ----------------------------------------------
 *          - Position controller takes over vertical axis control
 *          - Target altitude = take_off_complete_alt
 *          - Climb rate = pilot's commanded climb rate (pilot_climb_rate_cm)
 *          - Smooth blending of pilot input with altitude target
 * 
 *          Takeoff Completion Detection:
 *          - Completes when altitude reaches within 0.1% of target
 *          - Also completes if pilot commands negative climb rate (override/cancel)
 * 
 *          PILOT OVERRIDE CAPABILITY:
 *          - Pilot retains full control of climb rate throughout takeoff
 *          - Negative climb rate immediately terminates takeoff
 *          - Allows pilot to slow, pause, or cancel takeoff at any time
 * 
 * @param[in,out] pilot_climb_rate_cm Pilot's commanded vertical climb rate in cm/s
 *                                    (typically from throttle stick input).
 *                                    Function uses this to control climb speed and
 *                                    detect cancellation (negative = cancel takeoff).
 *                                    Parameter is not modified by this function.
 * 
 * @note This function is called at main loop rate (typically 400Hz) while takeoff
 *       is active (_running = true). Returns immediately if takeoff is inactive.
 * 
 * @note The gradual throttle ramp (Phase 1) prevents sudden motor acceleration that
 *       could cause instability on takeoff, especially important for heavy vehicles
 *       or those with low-authority controllers.
 * 
 * @warning The liftoff detection criteria are safety-critical. False negatives
 *          (failing to detect liftoff) can cause position controller instability.
 *          False positives (detecting liftoff while still grounded) can cause
 *          excessive throttle and unstable liftoff.
 * 
 * @see start(), stop(), TKOFF_SLEW_TIME parameter, TKOFF_THR_MAX parameter
 */
void Mode::_TakeOff::do_pilot_takeoff(float& pilot_climb_rate_cm)
{
    // return pilot_climb_rate if take-off inactive
    if (!_running) {
        return;
    }

    // PHASE 1: PRE-LIFTOFF - Gradual throttle ramp until liftoff detected
    if (copter.ap.land_complete) {
        // Ramp throttle gradually to provide smooth, controlled liftoff
        // Slew rate determined by TKOFF_SLEW_TIME parameter (default 2.0 seconds)
        // send throttle to attitude controller with angle boost
        float throttle = constrain_float(copter.attitude_control->get_throttle_in() + copter.G_Dt / copter.g2.takeoff_throttle_slew_time, 0.0, 1.0);
        copter.attitude_control->set_throttle_out(throttle, true, 0.0);
        
        // Reset position controller to prevent I-term windup during throttle ramp
        // tell position controller to reset alt target and reset I terms
        copter.pos_control->init_U_controller();
        
        // LIFTOFF DETECTION: Monitor multiple indicators to detect when vehicle has left ground
        // Once any criterion is met, transition to Phase 2 (active position control)
        if (throttle >= MIN(copter.g2.takeoff_throttle_max, 0.9) || 
            (copter.pos_control->get_measured_accel_U_cmss() >= 0.5 * copter.pos_control->get_max_accel_U_cmss()) ||
            (copter.pos_control->get_vel_desired_NEU_cms().z >= constrain_float(pilot_climb_rate_cm, copter.pos_control->get_max_speed_up_cms() * 0.1, copter.pos_control->get_max_speed_up_cms() * 0.5)) || 
            (is_positive(take_off_complete_alt - take_off_start_alt) && copter.pos_control->get_pos_desired_U_cm() - take_off_start_alt > 0.5 * (take_off_complete_alt - take_off_start_alt))) {
            // throttle > 90%
            // acceleration > 50% maximum acceleration
            // velocity > 10% maximum velocity && commanded climb rate
            // velocity > 50% maximum velocity
            // altitude change greater than half complete alt from start off alt
            copter.set_land_complete(false);
        }
    } else {
        // PHASE 2: ACTIVE CLIMB - Position controller manages altitude with pilot input
        float pos_z = take_off_complete_alt;
        float vel_z = pilot_climb_rate_cm;

        // Command position controller with target altitude and pilot's desired climb rate
        // This creates smooth blending: approaches target altitude while respecting pilot input
        // command the aircraft to the take off altitude and current pilot climb rate
        copter.pos_control->input_pos_vel_accel_U_cm(pos_z, vel_z, 0);

        // TAKEOFF COMPLETION/CANCELLATION DETECTION
        // Complete if altitude target reached (within 0.1% tolerance)
        // Cancel if pilot commands negative climb rate (pilot override)
        // stop take off early and return if negative climb rate is commanded or we are within 0.1% of our take off altitude
        if (is_negative(pilot_climb_rate_cm) ||
            (take_off_complete_alt  - take_off_start_alt) * 0.999f < copter.pos_control->get_pos_desired_U_cm() - take_off_start_alt) {
            stop();
        }
    }
}

/**
 * @brief Execute automated takeoff sequence for Auto and Guided flight modes
 * 
 * @details This function implements a sophisticated three-phase automated takeoff
 *          specifically designed for autonomous missions and guided mode operations.
 *          Unlike pilot-controlled takeoff (do_pilot_takeoff), this provides fully
 *          automated vertical and horizontal control with navigation capability.
 * 
 *          AUTOMATED TAKEOFF STATE MACHINE - Three distinct phases:
 * 
 *          PHASE 1: Motor Spool-Up (spool_state != THROTTLE_UNLIMITED)
 *          -----------------------------------------------------------
 *          - Motors ramping up to full operational speed
 *          - All controllers held in relaxed state (zero output)
 *          - Position/velocity controllers continuously reset
 *          - Attitude controller I-terms reset to prevent windup
 *          - No navigation active (horizontal position relaxed)
 *          - Updates no_nav_alt_cm for Phase 2 navigation threshold
 * 
 *          PHASE 2: Pre-Liftoff Ground Phase (land_complete = true)
 *          ---------------------------------------------------------
 *          - Gradual throttle ramping controlled by TKOFF_SLEW_TIME
 *          - Horizontal navigation disabled (velocity controller relaxed)
 *          - Vertical position controller continuously reset
 *          - Attitude hold with no rate commands
 * 
 *          Liftoff Detection Criteria (ANY condition triggers Phase 3):
 *          1. Throttle reaches 90% (or TKOFF_THR_MAX if lower)
 *          2. Vertical acceleration exceeds 50% of maximum capability
 *          3. Vertical velocity exceeds 10% of maximum climb rate
 *          4. Altitude exceeds no_nav_alt_cm threshold (if no-nav mode active)
 * 
 *          PHASE 3: Active Flight (land_complete = false)
 *          -----------------------------------------------
 *          - Full position controller active (vertical axis to target altitude)
 *          - Horizontal navigation enabled once altitude > no_nav_alt_cm
 *          - Auto-yaw control active (heading from waypoint or ROI)
 *          - Terrain following if terrain_alt flag set
 * 
 *          Navigation Altitude Threshold (WP_NAVALT_MIN):
 *          - Below threshold: Horizontal navigation disabled (vertical only)
 *          - Above threshold: Full 3D navigation enabled
 *          - Prevents GPS drift from causing horizontal movement near ground
 *          - Critical for safe operation in areas with poor GPS reception
 * 
 *          Takeoff Completion Criteria (BOTH must be satisfied):
 *          1. Altitude within stopping distance of target (calculated from velocity/acceleration)
 *          2. Climb rate below 10% of maximum climb rate
 * 
 *          Completion position saved for smooth transition to next waypoint
 * 
 * @note This function is called at main loop rate (typically 400Hz) during
 *       automated takeoff operations in Auto, Guided, and similar modes
 * 
 * @note The no-nav altitude threshold prevents potentially dangerous horizontal
 *       movement close to the ground where GPS accuracy may be reduced and
 *       obstacle clearance is critical
 * 
 * @warning If terrain data is required (terrain_alt=true) but unavailable,
 *          this function triggers a terrain failsafe and returns immediately
 * 
 * @warning Motor interlock must be enabled before this function is called,
 *          or the vehicle will remain grounded with motors in ground-idle
 * 
 * @see _AutoTakeoff::start(), get_completion_pos(), WP_NAVALT_MIN parameter
 */
void _AutoTakeoff::run()
{
    const auto &g2 = copter.g2;
    const auto &wp_nav = copter.wp_nav;
    auto *motors = copter.motors;
    auto *pos_control = copter.pos_control;
    auto *attitude_control = copter.attitude_control;

    // SAFETY CHECK: Abort takeoff if motors not armed or auto-arming incomplete
    // if not armed set throttle to zero and exit immediately
    if (!motors->armed() || !copter.ap.auto_armed) {
        // do not spool down tradheli when on the ground with motor interlock enabled
        copter.flightmode->make_safe_ground_handling(copter.is_tradheli() && motors->get_interlock());
        // Update navigation altitude threshold for next iteration
        // update auto_takeoff_no_nav_alt_cm
        no_nav_alt_cm = pos_control->get_pos_estimate_NEU_cm().z + g2.wp_navalt_min * 100;
        return;
    }

    // TERRAIN FOLLOWING: Retrieve terrain altitude offset if terrain-relative takeoff requested
    // get terrain offset
    float terr_offset = 0.0f;
    if (terrain_alt && !wp_nav->get_terrain_offset_cm(terr_offset)) {
        // Terrain data required but unavailable - trigger failsafe for safety
        // trigger terrain failsafe
        copter.failsafe_terrain_on_event();
        return;
    }

    // Request full motor throttle range for takeoff (allows motors to reach maximum power)
    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // PHASE 1: MOTOR SPOOL-UP - Wait for motors to reach operational speed before liftoff
    // aircraft stays in landed state until rotor speed run up has finished
    if (motors->get_spool_state() != AP_Motors::SpoolState::THROTTLE_UNLIMITED) {
        // Motors still spooling up - maintain zero control output to prevent premature movement
        // motors have not completed spool up yet so relax navigation and position controllers
        pos_control->relax_velocity_controller_NE();
        pos_control->update_NE_controller();
        pos_control->relax_U_controller(0.0f);   // forces throttle output to decay to zero
        pos_control->update_U_controller();
        attitude_control->reset_yaw_target_and_rate();
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->input_thrust_vector_rate_heading_rads(pos_control->get_thrust_vector(), 0.0);
        // Calculate navigation altitude threshold for when full navigation will be enabled
        // update auto_takeoff_no_nav_alt_cm
        no_nav_alt_cm = pos_control->get_pos_estimate_NEU_cm().z + g2.wp_navalt_min * 100;
        return;
    }
    
    // PHASE 2: PRE-LIFTOFF GROUND PHASE - Gradual throttle ramp until liftoff detected
    // aircraft stays in landed state until vertical movement is detected or 90% throttle is reached
    if (copter.ap.land_complete) {
        // Ramp throttle gradually using TKOFF_SLEW_TIME parameter for smooth acceleration
        // send throttle to attitude controller with angle boost
        float throttle = constrain_float(copter.attitude_control->get_throttle_in() + copter.G_Dt / copter.g2.takeoff_throttle_slew_time, 0.0, 1.0);
        copter.attitude_control->set_throttle_out(throttle, true, 0.0);
        
        // Reset position controller to prevent I-term accumulation during ramp
        // tell position controller to reset alt target and reset I terms
        copter.pos_control->init_U_controller();
        
        // Keep horizontal position relaxed (no navigation commands during ground phase)
        pos_control->relax_velocity_controller_NE();
        pos_control->update_NE_controller();
        
        // Maintain zero attitude rates and reset I-terms for smooth transition to flight
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->input_thrust_vector_rate_heading_rads(pos_control->get_thrust_vector(), 0.0);
        
        // LIFTOFF DETECTION: Check multiple indicators to determine when vehicle has left ground
        if (throttle >= MIN(copter.g2.takeoff_throttle_max, 0.9) || 
            (copter.pos_control->get_measured_accel_U_cmss() >= 0.5 * copter.pos_control->get_max_accel_U_cmss()) ||
            (copter.pos_control->get_vel_desired_NEU_cms().z >= 0.1 * copter.pos_control->get_max_speed_up_cms()) || 
            ( no_nav_active && (pos_control->get_pos_estimate_NEU_cm().z >= no_nav_alt_cm))) {
            // throttle > 90%
            // acceleration > 50% maximum acceleration
            // velocity > 10% maximum velocity
            // altitude change greater than half auto_takeoff_no_nav_alt_cm
            copter.set_land_complete(false);
        }
        return;
    }

    // PHASE 3: ACTIVE FLIGHT - Full navigation control active
    
    // HORIZONTAL NAVIGATION CONTROL: Enable navigation only after reaching safe altitude
    // check if we are not navigating because of low altitude
    if (no_nav_active) {
        // Below navigation threshold altitude - disable horizontal navigation for safety
        // check if vehicle has reached no_nav_alt threshold
        if (pos_control->get_pos_estimate_NEU_cm().z >= no_nav_alt_cm) {
            // Reached safe altitude - enable horizontal navigation
            no_nav_active = false;
        }
        // Keep horizontal velocity relaxed (zero target) while below threshold
        pos_control->relax_velocity_controller_NE();
    } else {
        // Above navigation threshold - enable horizontal position/velocity control
        // Zero velocity/acceleration commands (maintain position during vertical climb)
        Vector2f vel;
        Vector2f accel;
        pos_control->input_vel_accel_NE_cm(vel, accel);
    }
    pos_control->update_NE_controller();

    // VERTICAL POSITION CONTROL: Command climb to target altitude
    // command the aircraft to the take off altitude
    float pos_z = complete_alt_cm + terr_offset;
    float vel_z = 0.0;
    copter.pos_control->input_pos_vel_accel_U_cm(pos_z, vel_z, 0.0);
    
    // run the vertical position controller and set output throttle
    pos_control->update_U_controller();

    // ATTITUDE CONTROL: Apply thrust vector with automated yaw control
    // call attitude controller with auto yaw
    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), copter.flightmode->auto_yaw.get_heading());

    // TAKEOFF COMPLETION DETECTION: Check if target altitude and velocity criteria met
    // takeoff complete when we are less than 1% of the stopping distance from the target altitude
    // and 10% our maximum climb rate
    const float vel_threshold_fraction = 0.1;
    // Calculate stopping distance based on deceleration from threshold velocity
    // stopping distance from vel_threshold_fraction * max velocity
    const float stop_distance = 0.5 * sq(vel_threshold_fraction * copter.pos_control->get_max_speed_up_cms()) / copter.pos_control->get_max_accel_U_cmss();
    bool reached_altitude = copter.pos_control->get_pos_desired_U_cm() >= pos_z - stop_distance;
    bool reached_climb_rate = copter.pos_control->get_vel_desired_NEU_cms().z < copter.pos_control->get_max_speed_up_cms() * vel_threshold_fraction;
    complete = reached_altitude && reached_climb_rate;

    // Save completion position for smooth transition to waypoint navigation
    // calculate completion for location in case it is needed for a smooth transition to wp_nav
    if (complete) {
        const Vector3p& _complete_pos = copter.pos_control->get_pos_desired_NEU_cm();
        complete_pos = Vector3p{_complete_pos.x, _complete_pos.y, pos_z};
    }
}

/**
 * @brief Initialize automated takeoff state for Auto/Guided mode operation
 * 
 * @details This function configures the automated takeoff state machine with
 *          target altitude and navigation parameters. It determines whether
 *          horizontal navigation should be delayed until reaching a safe altitude
 *          based on the WP_NAVALT_MIN parameter and current flight state.
 * 
 *          Navigation Delay Logic (no_nav_active):
 *          - If WP_NAVALT_MIN > 0 AND (disarmed OR landed OR interlock disabled):
 *            Enable no-nav mode (climb vertically without horizontal navigation
 *            until reaching current_altitude + WP_NAVALT_MIN)
 *          - Otherwise: Full 3D navigation enabled immediately
 * 
 *          This prevents GPS drift from causing horizontal movement near the
 *          ground where clearance is critical and GPS accuracy may be reduced.
 * 
 * @param[in] _complete_alt_cm Target takeoff altitude in centimeters relative
 *                             to EKF origin (typically home altitude)
 * @param[in] _terrain_alt     If true, target altitude is terrain-relative and
 *                             terrain data will be used to adjust target during
 *                             climb. If false, altitude is relative to EKF origin.
 * 
 * @note This function only initializes state - it does not command motors or
 *       begin the climb. The actual takeoff execution is performed by run().
 * 
 * @note Called once at takeoff initiation, not at main loop rate
 * 
 * @see run(), WP_NAVALT_MIN parameter, get_completion_pos()
 */
void _AutoTakeoff::start(float _complete_alt_cm, bool _terrain_alt)
{
    // auto_takeoff_complete_alt_cm is a problem if equal to auto_takeoff_start_alt_cm
    complete_alt_cm = _complete_alt_cm;
    terrain_alt = _terrain_alt;
    complete = false;
    
    // Calculate the altitude threshold for enabling horizontal navigation
    // initialise auto_takeoff_no_nav_alt_cm
    const auto &g2 = copter.g2;
    no_nav_alt_cm = copter.pos_control->get_pos_estimate_NEU_cm().z + g2.wp_navalt_min * 100;
    
    // Determine if horizontal navigation should be delayed until reaching safe altitude
    if ((g2.wp_navalt_min > 0) && (copter.flightmode->is_disarmed_or_landed() || !copter.motors->get_interlock())) {
        // Starting from ground - delay horizontal navigation until reaching no_nav_alt_cm
        // we are not flying, climb with no navigation to current alt-above-ekf-origin + wp_navalt_min
        no_nav_active = true;
    } else {
        // Already airborne or navigation threshold disabled - enable full navigation immediately
        no_nav_active = false;
    }
}

/**
 * @brief Retrieve the final position where automated takeoff completed
 * 
 * @details This function provides the precise 3D position (in NEU frame relative
 *          to EKF origin) where the automated takeoff sequence reached completion.
 *          This position is used for smooth transition to waypoint navigation,
 *          ensuring the vehicle doesn't suddenly jump to a new target position.
 * 
 *          The completion position is calculated when both altitude and velocity
 *          completion criteria are met in the run() function. It represents the
 *          commanded position (not measured position) at the moment of completion.
 * 
 * @param[out] pos_neu_cm 3D position vector in NEU frame (North-East-Up) in
 *                        centimeters relative to EKF origin. Only valid if
 *                        function returns true.
 * 
 * @return true if takeoff is complete and position is available, false if
 *         takeoff still in progress or not started
 * 
 * @note NEU frame: X=North, Y=East, Z=Up (all in cm from EKF origin)
 * @note This is the commanded position, not the measured position, to ensure
 *       smooth controller handoff to the next navigation mode
 * 
 * @see run(), complete flag
 */
bool _AutoTakeoff::get_completion_pos(Vector3p& pos_neu_cm)
{
    // only provide location if takeoff has completed
    if (!complete) {
        return false;
    }

    pos_neu_cm = complete_pos;
    return true;
}

/**
 * @brief Query whether user-initiated takeoff is currently active
 * 
 * @details This function checks if a pilot-controlled takeoff sequence is
 *          currently in progress. It verifies both that the current flight mode
 *          supports user takeoff and that the takeoff state machine is running.
 * 
 *          This is used by other systems to determine if special takeoff handling
 *          is needed (e.g., modified failsafe behavior, logging triggers).
 * 
 * @return true if user takeoff is active, false otherwise
 * 
 * @note This only detects user-initiated takeoffs (via MAVLink NAV_TAKEOFF command),
 *       not automated takeoffs in Auto/Guided modes (use auto_takeoff for those)
 * 
 * @note Returns false for flight modes that don't support user takeoff
 *       (e.g., Stabilize, Acro, manual throttle modes)
 * 
 * @see has_user_takeoff(), _TakeOff::running(), auto_takeoff
 */
bool Mode::is_taking_off() const
{
    if (!has_user_takeoff(false)) {
        return false;
    }
    return takeoff.running();
}
