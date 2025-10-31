/**
 * @file mode_land.cpp
 * @brief Land mode implementation for autonomous landing operations
 * 
 * @details This file implements Land mode, which autonomously descends the vehicle
 *          and lands at the current location (GPS-enabled) or descends vertically
 *          (non-GPS). Land mode is the primary autonomous landing method used during
 *          normal operations, failsafes, and mission completion.
 * 
 * Key Features:
 * - GPS-based landing with horizontal position hold
 * - Non-GPS landing with pilot roll/pitch control
 * - Precision landing integration when available
 * - Rangefinder-aware descent rate control
 * - Landing detection and automatic disarm
 * - Optional landing delay for failsafe scenarios
 * 
 * @note This is a safety-critical flight mode - modifications require extensive testing
 * @warning Improper landing parameters can result in hard landings or vehicle damage
 */

#include "Copter.h"

/**
 * @brief Initialize Land mode controller and configure descent parameters
 * 
 * @details Initializes the Land mode by:
 *          1. Determining if GPS position control is available (control_position flag)
 *          2. Configuring horizontal position controller limits (if GPS available)
 *          3. Configuring vertical descent rate and acceleration limits
 *          4. Initializing position controllers (NE and U axes)
 *          5. Setting up yaw hold mode
 *          6. Deploying landing gear (if equipped)
 *          7. Initializing precision landing state machine (if enabled)
 * 
 *          The initialization determines landing strategy based on GPS availability:
 *          - With GPS: Uses position controller to hold horizontal position during descent
 *          - Without GPS: Allows pilot to control horizontal movement via roll/pitch
 * 
 * @param[in] ignore_checks Not used in Land mode - included for mode interface consistency
 * 
 * @return true Always returns true - Land mode can always be entered for safety
 * 
 * @note Called at mode entry (typically 100Hz main loop rate)
 * @note Sets land_start_time for delay calculations in failsafe scenarios
 * @note Resets land_repo_active and prec_land_active flags
 * 
 * @warning GPS loss during init will result in non-GPS landing mode
 * 
 * @see gps_run() for GPS-enabled landing implementation
 * @see nogps_run() for non-GPS landing implementation
 * @see Copter::position_ok() for GPS availability determination
 */
bool ModeLand::init(bool ignore_checks)
{
    // Determine landing strategy based on GPS availability
    // control_position = true: GPS-based landing with horizontal position hold
    // control_position = false: Non-GPS landing with pilot roll/pitch control
    // This check uses EKF position accuracy and GPS lock status
    control_position = copter.position_ok();

    // Configure horizontal (North-East) position controller limits
    // Uses waypoint navigation default speeds for smooth, predictable landing approach
    // Max speed: Limits how fast vehicle can reposition during descent
    // Acceleration: Prevents aggressive movements that could destabilize landing
    pos_control->set_max_speed_accel_NE_cm(wp_nav->get_default_speed_NE_cms(), wp_nav->get_wp_acceleration_cmss());
    pos_control->set_correction_speed_accel_NE_cm(wp_nav->get_default_speed_NE_cms(), wp_nav->get_wp_acceleration_cmss());

    // Initialize horizontal position controller if GPS is available and controller not already active
    // The NE (North-East) controller maintains horizontal position during descent
    // Only activated for GPS-based landing to prevent drift from landing point
    if (control_position && !pos_control->is_active_NE()) {
        pos_control->init_NE_controller();
    }

    // Configure vertical (Up) descent rate and acceleration limits
    // Down speed: Typically faster than up speed for efficient landing
    // Up speed: Used if pilot overrides or landing is aborted
    // Acceleration: Controls how quickly descent rate changes (smoother = safer)
    // Note: Actual descent rate will be ramped based on altitude and rangefinder data
    pos_control->set_max_speed_accel_U_cm(wp_nav->get_default_speed_down_cms(), wp_nav->get_default_speed_up_cms(), wp_nav->get_accel_U_cmss());
    pos_control->set_correction_speed_accel_U_cmss(wp_nav->get_default_speed_down_cms(), wp_nav->get_default_speed_up_cms(), wp_nav->get_accel_U_cmss());

    // Initialize vertical position controller if not already active
    // The U (Up) controller manages descent rate throughout landing sequence
    // Always initialized regardless of GPS availability - vertical control is essential
    if (!pos_control->is_active_U()) {
        pos_control->init_U_controller();
    }

    // Record landing start time for delay calculations
    // Used to implement LAND_WITH_DELAY_MS pause in failsafe scenarios
    // This delay prevents immediate descent, giving time for situation assessment
    land_start_time = millis();
    land_pause = false;

    // Reset flag indicating if pilot has applied roll or pitch inputs during landing
    // When pilot provides stick input during GPS landing, land_repo_active is set
    // This allows limited repositioning while maintaining autonomous descent
    copter.ap.land_repo_active = false;

    // Reset precision landing active flag
    // This will be set true if precision landing successfully acquires target
    // Precision landing uses visual targets for highly accurate touchdown
    copter.ap.prec_land_active = false;

    // Initialize yaw control to hold current heading
    // Prevents vehicle rotation during landing for stable descent
    // Pilot can still override yaw with rudder stick if needed
    auto_yaw.set_mode(AutoYaw::Mode::HOLD);

#if AP_LANDINGGEAR_ENABLED
    // Deploy landing gear if vehicle is equipped
    // Ensures landing gear is extended before touchdown
    // Deployment typically takes 1-2 seconds, done early in landing sequence
    copter.landinggear.deploy_for_landing();
#endif

#if AC_PRECLAND_ENABLED
    // Initialize precision landing state machine
    // Precision landing uses visual targets (IR beacons, AprilTags, etc.)
    // Provides centimeter-level accuracy for landing on precise targets
    // State machine handles: target acquisition, tracking, descent, and final approach
    copter.precland_statemachine.init();
#endif

    return true;
}

/**
 * @brief Main Land mode controller - dispatches to GPS or non-GPS implementation
 * 
 * @details This is the main entry point for Land mode control logic, called every
 *          iteration of the main control loop. It routes execution to either GPS-based
 *          or non-GPS landing based on the control_position flag set during init().
 * 
 *          Landing Strategy Selection:
 *          - control_position = true: GPS-based landing (gps_run)
 *            - Maintains horizontal position during descent
 *            - Uses position controller for drift correction
 *            - Supports precision landing if target available
 *          
 *          - control_position = false: Non-GPS landing (nogps_run)
 *            - Pilot controls horizontal position via roll/pitch
 *            - Vehicle descends vertically at current location
 *            - Useful for GPS-denied environments or GPS failure
 * 
 * @note Called at main loop rate (typically 100-400Hz depending on vehicle configuration)
 * @note control_position flag can be changed mid-flight via do_not_use_GPS() during failsafe
 * 
 * @warning GPS loss during landing will NOT automatically switch modes - vehicle continues
 *          with last known position until landing complete or mode changed
 * 
 * @see gps_run() for GPS-enabled landing implementation
 * @see nogps_run() for non-GPS landing implementation
 * @see do_not_use_GPS() for forced non-GPS mode during GPS failsafe
 */
void ModeLand::run()
{
    // Route to appropriate landing controller based on GPS availability
    // This determination is made at mode init and can be overridden by GPS failsafe
    if (control_position) {
        gps_run();
    } else {
        nogps_run();
    }
}

/**
 * @brief GPS-enabled landing controller with horizontal position hold
 * 
 * @details Implements autonomous landing with GPS-based horizontal position control.
 *          This method maintains the vehicle's horizontal position (loiter) while
 *          executing a controlled vertical descent to the ground.
 * 
 *          Landing Sequence (GPS Mode):
 *          1. Optional delay period (LAND_WITH_DELAY_MS) if triggered by failsafe
 *          2. Maintain horizontal position using position controller
 *          3. Controlled descent with ramped rate (faster high, slower near ground)
 *          4. Rangefinder integration for precise final approach altitude
 *          5. Precision landing guidance if target acquired
 *          6. Landing detection monitors for ground contact
 *          7. Automatic motor disarm once landing confirmed
 * 
 *          Descent Rate Strategy:
 *          - Initial descent: Uses LAND_SPEED parameter (typically 50cm/s)
 *          - Intermediate: Gradual rate reduction as altitude decreases
 *          - Final approach: Slower rate (LAND_SPEED_HIGH_CM) for soft touchdown
 *          - Rangefinder-adjusted: More accurate near-ground speed control
 * 
 *          Landing Detection Integration:
 *          - Monitors throttle output, barometer, and accelerometer
 *          - Detects ground contact via land_detector.cpp algorithms
 *          - Sets ap.land_complete flag when landing confirmed
 *          - Waits for motors to spool down to GROUND_IDLE before disarm
 * 
 * @note Called at main loop rate (typically 100-400Hz)
 * @note Horizontal position hold uses same controller as Loiter mode
 * @note Precision landing takes precedence over normal landing if target visible
 * 
 * @warning Landing detection must confirm ground contact before disarm
 * @warning Motor disarm timing critical - premature disarm causes hard landing
 * 
 * @see land_run_normal_or_precland() for descent rate control implementation
 * @see land_detector.cpp for landing detection algorithm details
 * @see AC_PrecLand for precision landing implementation
 */
void ModeLand::gps_run()
{
    // Automatic disarm sequence after confirmed landing
    // Requires BOTH conditions to prevent premature disarm:
    // 1. land_complete: Landing detector confirms ground contact (throttle low, no movement)
    // 2. GROUND_IDLE: Motors have spooled down to minimum, vehicle is stable
    // This two-stage check ensures safe disarm without bounce or instability
    if (copter.ap.land_complete && motors->get_spool_state() == AP_Motors::SpoolState::GROUND_IDLE) {
        copter.arming.disarm(AP_Arming::Method::LANDED);
    }

    // Land State Machine - two primary states:
    // 1. Disarmed or landed: Safe ground handling mode
    // 2. In-flight: Active descent with position control
    if (is_disarmed_or_landed()) {
        // Ground handling: Zero throttle, disable position control, prep for takeoff
        // Entered when disarmed OR when landing is complete but not yet disarmed
        make_safe_ground_handling();
    } else {
        // Active landing descent - vehicle is in the air and descending
        
        // Enable full motor range for descent control
        // THROTTLE_UNLIMITED allows controller full authority for smooth descent
        // Essential for maintaining stable descent rate and handling wind
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // Implement optional landing delay for failsafe scenarios
        // land_pause is set true when Land mode triggered by failsafe
        // Provides LAND_WITH_DELAY_MS (default 4 seconds) before starting descent
        // Allows pilot time to assess situation and take manual control if needed
        if (land_pause && millis()-land_start_time >= LAND_WITH_DELAY_MS) {
            land_pause = false;
        }

        // Execute landing descent with position hold
        // Runs either normal landing or precision landing based on target availability
        // - Normal landing: Maintains horizontal position, controlled vertical descent
        // - Precision landing: Tracks visual target for accurate touchdown location
        // land_pause parameter prevents descent during delay period
        land_run_normal_or_precland(land_pause);
    }
}

/**
 * @brief Non-GPS landing controller with pilot horizontal control
 * 
 * @details Implements autonomous descent without GPS position hold. The pilot
 *          controls roll and pitch to manage horizontal position while the flight
 *          controller manages vertical descent automatically.
 * 
 *          Landing Sequence (Non-GPS Mode):
 *          1. Optional delay period if triggered by failsafe
 *          2. Pilot controls horizontal movement via roll/pitch stick inputs
 *          3. Automatic vertical descent rate control (same as GPS mode)
 *          4. Rangefinder integration for final approach (if available)
 *          5. Landing detection monitors for ground contact
 *          6. Automatic motor disarm once landing confirmed
 * 
 *          Non-GPS Landing Strategy:
 *          - No horizontal position hold - vehicle drifts with wind unless pilot corrects
 *          - Pilot has full roll/pitch control authority (within lean angle limits)
 *          - Altitude hold disabled - continuous descent to ground
 *          - Useful for: GPS failure, indoor flight, GPS-denied environments
 * 
 *          Pilot Control Features:
 *          - Roll/pitch: Reposition vehicle during descent (if LAND_REPOSITION enabled)
 *          - Yaw: Rotate vehicle during descent (yaw rate control)
 *          - Throttle: Can cancel landing if throttle raised above threshold
 *          - SIMPLE mode: Optional simplified control reference frame
 * 
 *          Safety Features:
 *          - Throttle-based landing cancel: High throttle exits to ALT_HOLD
 *          - Lean angle limiting: Prevents excessive tilt during descent
 *          - Automatic descent rate ramping: Slower near ground
 *          - Landing detection: Prevents premature disarm
 * 
 * @note Called at main loop rate (typically 100-400Hz)
 * @note Descent rate control identical to GPS mode - only horizontal control differs
 * @note Pilot repositioning can be disabled via LAND_REPOSITION parameter
 * 
 * @warning Without GPS, vehicle may drift significantly in wind during descent
 * @warning Pilot must maintain awareness of horizontal position to avoid obstacles
 * @warning Landing on slopes more challenging without position hold
 * 
 * @see land_run_vertical_control() for descent rate control
 * @see get_pilot_desired_lean_angles_rad() for pilot input processing
 * @see land_detector.cpp for landing detection algorithm
 */
void ModeLand::nogps_run()
{
    // Initialize target attitude angles for pilot control
    // Default to level (0,0) if no pilot input provided
    float target_roll_rad = 0.0f, target_pitch_rad = 0.0f;

    // Process pilot RC inputs for horizontal control and landing cancellation
    if (rc().has_valid_input()) {
        // Optional landing cancellation via high throttle input
        // If THR_BEHAVE_HIGH_THROTTLE_CANCELS_LAND flag is set:
        // - Pilot can abort landing by raising throttle above LAND_CANCEL_TRIGGER_THR
        // - Vehicle switches to ALT_HOLD mode, maintaining current altitude
        // - Useful if pilot needs to abort landing due to obstacle or unsafe conditions
        if ((g.throttle_behavior & THR_BEHAVE_HIGH_THROTTLE_CANCELS_LAND) != 0 && copter.rc_throttle_control_in_filter.get() > LAND_CANCEL_TRIGGER_THR){
            LOGGER_WRITE_EVENT(LogEvent::LAND_CANCELLED_BY_PILOT);
            // Exit land mode to altitude hold, allowing pilot to reposition
            copter.set_mode(Mode::Number::ALT_HOLD, ModeReason::THROTTLE_LAND_ESCAPE);
        }

        // Process pilot roll/pitch inputs for horizontal repositioning during descent
        // Only enabled if LAND_REPOSITION parameter is true
        // Allows pilot to make fine adjustments to landing location
        if (g.land_repositioning) {
            // Apply SIMPLE mode transformation if enabled
            // SIMPLE mode references control inputs to initial heading, easier for beginners
            update_simple_mode();

            // Convert pilot stick inputs to desired lean angles
            // Respects both normal lean angle max and altitude-hold specific limit
            // Alt-hold limit typically more conservative for safer low-altitude maneuvering
            get_pilot_desired_lean_angles_rad(target_roll_rad, target_pitch_rad, attitude_control->lean_angle_max_rad(), attitude_control->get_althold_lean_angle_max_rad());
        }
    }

    // Automatic disarm sequence after confirmed landing
    // Identical logic to GPS mode - requires both landing confirmation and motor spool down
    // Prevents disarm during momentary ground contact (bounces) or unstable touchdown
    if (copter.ap.land_complete && motors->get_spool_state() == AP_Motors::SpoolState::GROUND_IDLE) {
        copter.arming.disarm(AP_Arming::Method::LANDED);
    }

    // Land State Machine - same structure as GPS mode
    if (is_disarmed_or_landed()) {
        // Ground handling: Zero throttle, safe state for disarmed vehicle
        make_safe_ground_handling();
    } else {
        // Active landing descent - vehicle descending under control
        
        // Enable full motor authority for descent control
        // Full throttle range needed to maintain stable descent and respond to pilot input
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // Implement optional landing delay (typically used in failsafe scenarios)
        // Gives pilot time to assess situation before descent begins
        // Same delay mechanism as GPS mode
        if (land_pause && millis()-land_start_time >= LAND_WITH_DELAY_MS) {
            land_pause = false;
        }

        // Execute vertical descent control
        // Manages descent rate, rangefinder integration, landing detection
        // Horizontal position is NOT controlled - pilot manages via roll/pitch
        land_run_vertical_control(land_pause);
    }

    // Apply attitude control based on pilot inputs and yaw mode
    // target_roll_rad, target_pitch_rad: From pilot stick or zero if no input
    // yaw_rate_rads: From auto_yaw controller (typically hold current heading)
    // This gives pilot horizontal control while automation manages descent
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_rad(target_roll_rad, target_pitch_rad, auto_yaw.get_heading().yaw_rate_rads);
}

/**
 * @brief Force Land mode to switch from GPS to non-GPS landing strategy
 * 
 * @details Called by GPS failsafe handler when GPS quality degrades below safe thresholds
 *          during an active landing. Disables horizontal position control and switches to
 *          pilot-controlled horizontal movement while maintaining automatic vertical descent.
 * 
 *          This function provides graceful degradation when GPS is lost mid-landing:
 *          - Switches from gps_run() to nogps_run() execution path
 *          - Disables horizontal position hold immediately
 *          - Pilot gains roll/pitch control to manage drift
 *          - Vertical descent continues normally
 *          - Landing detection remains active
 * 
 *          Failsafe Scenario:
 *          1. Vehicle landing with GPS position hold (gps_run active)
 *          2. GPS signal degrades (lost satellites, interference, multipath)
 *          3. GPS failsafe triggers, calls do_not_use_GPS()
 *          4. control_position flag set false
 *          5. Next run() call routes to nogps_run() instead of gps_run()
 *          6. Pilot must manage horizontal drift manually
 * 
 * @note Only affects Land mode - has no effect if vehicle is in other modes
 * @note Does not change flight mode, only modifies Land mode behavior
 * @note Change takes effect on next run() iteration (within 2.5-10ms typically)
 * 
 * @warning Pilot must be ready to control horizontal position if GPS fails during landing
 * @warning Vehicle may drift significantly in wind after GPS loss without pilot input
 * @warning Transition from position hold to drift can be abrupt - requires pilot awareness
 * 
 * @see gps_run() for GPS-based landing implementation
 * @see nogps_run() for non-GPS landing implementation
 * @see Copter::failsafe_gps_check() for GPS failsafe trigger conditions
 */
void ModeLand::do_not_use_GPS()
{
    // Disable GPS-based horizontal position control
    // Next run() call will route to nogps_run() instead of gps_run()
    // This provides immediate failsafe response to GPS degradation
    control_position = false;
}

/**
 * @brief Initiate Land mode with delayed descent for failsafe scenarios
 * 
 * @details Sets the vehicle to Land mode with an initial pause period before descent begins.
 *          This function is specifically designed for failsafe scenarios where immediate
 *          descent may be undesirable, giving the pilot time to assess the situation and
 *          potentially take manual control.
 * 
 *          Failsafe Landing Sequence:
 *          1. Mode switches to LAND
 *          2. land_pause flag set true
 *          3. Visual/audio notification alerts pilot
 *          4. Vehicle holds current altitude for LAND_WITH_DELAY_MS (default 4 seconds)
 *          5. After delay, normal landing descent begins
 * 
 *          Common Failsafe Triggers:
 *          - Battery failsafe (low voltage or capacity)
 *          - Radio failsafe (loss of RC link)
 *          - GCS failsafe (loss of telemetry link)
 *          - Geofence breach (altitude or horizontal boundary violation)
 *          - EKF failsafe (navigation system failure)
 * 
 *          The delay allows pilot to:
 *          - Regain RC control before landing starts
 *          - Switch to manual mode if appropriate
 *          - Assess wind, obstacles, and landing suitability
 *          - Reposition vehicle if needed (in GPS mode)
 * 
 * @param[in] reason ModeReason enum indicating which failsafe triggered landing
 *                   Logged for post-flight analysis and debugging
 * 
 * @note Always triggers pilot notification (LEDs, buzzer, GCS message)
 * @note Delay period configurable via LAND_WITH_DELAY_MS (typically 4000ms)
 * @note If pilot takes manual control during delay, landing is cancelled
 * 
 * @warning 4-second delay means vehicle continues current flight during this period
 * @warning In critical battery failsafe, even delayed landing may not prevent brownout
 * 
 * @see ModeLand::init() for landing initialization
 * @see ModeLand::gps_run() for delay handling in GPS mode
 * @see ModeLand::nogps_run() for delay handling in non-GPS mode
 */
void Copter::set_mode_land_with_pause(ModeReason reason)
{
    // Switch to Land mode with specified reason for logging
    set_mode(Mode::Number::LAND, reason);
    
    // Enable landing delay - vehicle will hold altitude for LAND_WITH_DELAY_MS
    // Gives pilot time to react to failsafe before descent begins
    mode_land.set_land_pause(true);

    // Trigger pilot notification system
    // Activates LEDs, buzzer, and GCS message to alert pilot of mode change
    // Critical for pilot awareness during autonomous failsafe landing
    AP_Notify::events.failsafe_mode_change = 1;
}

/**
 * @brief Check if vehicle is currently landing with GPS position control
 * 
 * @details Determines if the vehicle is executing a GPS-based autonomous landing,
 *          which implies horizontal position hold during descent. This is used by
 *          various subsystems to adjust behavior based on landing state.
 * 
 *          Returns true when ALL of the following conditions are met:
 *          1. Vehicle is in LAND flight mode
 *          2. Land mode is using GPS position control (control_position = true)
 * 
 *          Used by subsystems including:
 *          - Precision landing: Determines if position corrections should be applied
 *          - Compass: May adjust compass priority during GPS landing
 *          - EKF: Landing-specific state estimation adjustments
 *          - Logging: Identifies GPS landing events in logs
 *          - Geofence: Modified boundary checking during landing
 * 
 * @return true if vehicle is in Land mode with GPS position control active
 * @return false if in any other mode, or in Land mode without GPS (non-GPS landing)
 * 
 * @note Does not indicate landing progress - only landing strategy type
 * @note Returns false immediately if GPS position control lost (failsafe)
 * @note Can transition false mid-landing if do_not_use_GPS() called
 * 
 * @see ModeLand::gps_run() for GPS landing implementation
 * @see ModeLand::controlling_position() for control_position flag accessor
 * @see ModeLand::do_not_use_GPS() for GPS failsafe handling
 */
bool Copter::landing_with_GPS()
{
    // Check both conditions required for GPS landing:
    // 1. Currently in LAND mode (not any other flight mode)
    // 2. Land mode using position control (GPS available and control_position = true)
    return (flightmode->mode_number() == Mode::Number::LAND &&
            mode_land.controlling_position());
}
