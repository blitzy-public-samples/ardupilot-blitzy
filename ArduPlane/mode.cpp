/**
 * @file mode.cpp
 * @brief Base Mode class implementation for ArduPlane flight modes
 * 
 * @details This file implements the Mode base class, which provides common
 *          functionality and lifecycle management for all ArduPlane flight modes.
 *          Each specific mode (AUTO, MANUAL, FBWA, etc.) inherits from this base
 *          class and overrides virtual methods to implement mode-specific behavior.
 *          
 *          The Mode base class handles:
 *          - Mode initialization and state reset in enter() method
 *          - Mode cleanup in exit() method
 *          - Common stabilization through run() method
 *          - Controller reset via reset_controllers() method
 *          - Shared helper methods for throttle, rudder, and steering output
 *          - Capability checking (throttle limits, battery compensation, etc.)
 *          - Target altitude updates for navigation modes
 *          - Pre-arm checks for mode-specific arming requirements
 *          
 *          All flight modes in ArduPlane follow a consistent lifecycle:
 *          1. enter() - Initialize mode state, reset controllers, configure systems
 *          2. run() - Called at scheduler loop rate to perform mode control
 *          3. exit() - Clean up mode state when switching to another mode
 *          
 * @note This is safety-critical flight control code. Any modifications must be
 *       thoroughly tested in SITL before hardware deployment.
 * 
 * @see Mode.h for the Mode base class definition
 * @see Plane.h for the main vehicle class
 * 
 * Source: ArduPlane/mode.cpp
 */

#include "Plane.h"

/**
 * @brief Mode base class constructor
 * 
 * @details Initializes the Mode base class with references to key subsystems.
 *          For quadplane-enabled builds, initializes references to VTOL control
 *          systems (position control, attitude control, loiter navigation).
 *          All modes share a reference to the AHRS (Attitude and Heading Reference
 *          System) for vehicle state estimation.
 *          
 *          The unused_integer member exists for compatibility reasons and is not
 *          used in current implementation.
 * 
 * @note This constructor is called by all derived mode classes during their
 *       instantiation. The references are bound to the main Plane object's
 *       subsystems.
 * 
 * @warning Constructor must not perform any flight control operations or state
 *          changes. All mode initialization happens in enter() method.
 */
Mode::Mode() :
    unused_integer{17},
#if HAL_QUADPLANE_ENABLED
    pos_control(plane.quadplane.pos_control),
    attitude_control(plane.quadplane.attitude_control),
    loiter_nav(plane.quadplane.loiter_nav),
    quadplane(plane.quadplane),
    poscontrol(plane.quadplane.poscontrol),
#endif
    ahrs(plane.ahrs)
{
}

/**
 * @brief Exit the current flight mode
 * 
 * @details Called when switching away from this mode to another flight mode.
 *          This method performs cleanup operations to ensure a clean transition.
 *          The base implementation:
 *          1. Calls the derived class's _exit() method for mode-specific cleanup
 *          2. Stops autotuning if leaving any mode other than AUTOTUNE mode
 *          
 *          Derived classes should override _exit() to implement mode-specific
 *          cleanup while this base method handles common cleanup tasks.
 * 
 * @note This is called automatically by the mode change system and should not
 *       be called directly from user code.
 * 
 * @warning Failing to properly clean up mode state can lead to unexpected
 *          behavior in the next mode, potentially affecting flight safety.
 * 
 * @see enter() for mode initialization
 * @see _exit() for mode-specific cleanup implementation
 */
void Mode::exit()
{
    // call sub-classes exit
    _exit();
    // stop autotuning if not AUTOTUNE mode
    if (plane.control_mode != &plane.mode_autotune){
        plane.autotune_restore();
    }

}

/**
 * @brief Enter this flight mode from another mode
 * 
 * @details Called when switching into this mode from another flight mode.
 *          This method performs comprehensive initialization to ensure the mode
 *          starts in a safe, known state. The initialization sequence:
 *          
 *          1. Reset scripting, inverted flight, and special state flags
 *          2. Clear mission and navigation state (cross-track, autoland checks)
 *          3. Reset steering and crash detection state
 *          4. Clear external guidance commands (forced RPY, throttle, offboard guided)
 *          5. Configure camera auto mode state
 *          6. Initialize altitude and takeoff state tracking
 *          7. Reset loiter start time and record mode change timestamp
 *          8. Set VTOL mode flags for quadplane transitions
 *          9. Call derived class's _enter() method for mode-specific initialization
 *          10. If _enter() succeeds, configure throttle suppression, ADSB, navigation
 *          11. Reset steering integrator and update failsafe state
 *          12. Handle fence breach recovery and mission reset if appropriate
 *          13. Update flight stage and landing state
 *          14. Configure quadplane assisted flight if enabled
 * 
 * @return true if mode entry successful, false if mode cannot be entered
 *         (failure typically indicates mode-specific requirements not met)
 * 
 * @note This is called automatically by the mode change system. The extensive
 *       state reset ensures no residual state from the previous mode affects
 *       the new mode's behavior.
 * 
 * @warning This is safety-critical initialization code. All vehicle state must
 *          be properly initialized to prevent unexpected behavior. Missing
 *          initialization can cause control instability or loss of vehicle.
 * 
 * @see exit() for mode cleanup
 * @see _enter() for mode-specific initialization implementation
 */
bool Mode::enter()
{
#if AP_SCRIPTING_ENABLED
    // reset nav_scripting.enabled
    plane.nav_scripting.enabled = false;
#endif

    // cancel inverted flight
    plane.auto_state.inverted_flight = false;
    
    // cancel waiting for rudder neutral
    plane.takeoff_state.waiting_for_rudder_neutral = false;

    // don't cross-track when starting a mission
    plane.auto_state.next_wp_crosstrack = false;

    // reset landing check
    plane.auto_state.checked_for_autoland = false;

    // zero locked course
    plane.steer_state.locked_course_err = 0;
    plane.steer_state.locked_course = false;

    // reset crash detection
    plane.crash_state.is_crashed = false;
    plane.crash_state.impact_detected = false;

    // reset external attitude guidance
    plane.guided_state.last_forced_rpy_ms.zero();
    plane.guided_state.last_forced_throttle_ms = 0;

#if AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED
    plane.guided_state.target_heading = -4; // radians here are in range -3.14 to 3.14, so a default value needs to be outside that range
    plane.guided_state.target_heading_type = GUIDED_HEADING_NONE;
    plane.guided_state.target_airspeed_cm = -1; // same as above, although an airspeed of -1 is rare on plane.
    plane.guided_state.target_alt_time_ms = 0;
    plane.guided_state.target_location.set_alt_cm(-1, Location::AltFrame::ABSOLUTE); 
#endif

#if AP_CAMERA_ENABLED
    plane.camera.set_is_auto_mode(this == &plane.mode_auto);
#endif

    // zero initial pitch and highest airspeed on mode change
    plane.auto_state.highest_airspeed = 0;
    plane.auto_state.initial_pitch_cd = ahrs.pitch_sensor;

    // disable taildrag takeoff on mode change
    plane.auto_state.fbwa_tdrag_takeoff_mode = false;

    // wipe the takeoff rotation complete state
    plane.auto_state.rotation_complete = false;

    // start with previous WP at current location
    plane.prev_WP_loc = plane.current_loc;

    // new mode means new loiter
    plane.loiter.start_time_ms = 0;

    // record time of mode change
    plane.last_mode_change_ms = AP_HAL::millis();

    // set VTOL auto state
    plane.auto_state.vtol_mode = is_vtol_mode();
    plane.auto_state.vtol_loiter = false;

    // initialize speed variable used in AUTO and GUIDED for DO_CHANGE_SPEED commands
    plane.new_airspeed_cm = -1;
    
    // clear postponed long failsafe if mode change (from GCS) occurs before recall of long failsafe
    plane.long_failsafe_pending = false;

#if HAL_QUADPLANE_ENABLED
    quadplane.mode_enter();
#endif

#if AP_TERRAIN_AVAILABLE
    plane.target_altitude.terrain_following_pending = false;
#endif

#if AP_PLANE_SYSTEMID_ENABLED
    plane.g2.systemid.stop();
#endif

    // disable auto mode servo idle during altitude wait command
    plane.auto_state.idle_mode = false;

    bool enter_result = _enter();

    if (enter_result) {
        // -------------------
        // these must be done AFTER _enter() because they use the results to set more flags

        // start with throttle suppressed in auto_throttle modes
        plane.throttle_suppressed = does_auto_throttle();
#if HAL_ADSB_ENABLED
        plane.adsb.set_is_auto_mode(does_auto_navigation());
#endif

        // set the nav controller stale AFTER _enter() so that we can check if we're currently in a loiter during the mode change
        plane.nav_controller->set_data_is_stale();

        // reset steering integrator on mode change
        plane.steerController.reset_I();

        // update RC failsafe, as mode change may have necessitated changing the failsafe throttle
        plane.control_failsafe();

#if AP_FENCE_ENABLED
        // pilot requested flight mode change during a fence breach indicates pilot is attempting to manually recover
        // this flight mode change could be automatic (i.e. fence, battery, GPS or GCS failsafe)
        // but it should be harmless to disable the fence temporarily in these situations as well
        plane.fence.manual_recovery_start();
#endif
        //reset mission if in landing sequence, disarmed, not flying, and have changed to a non-autothrottle mode to clear prearm
        if (plane.mission.get_in_landing_sequence_flag() &&
            !plane.is_flying() && !plane.arming.is_armed_and_safety_off() &&
            !plane.control_mode->does_auto_navigation()) {
           GCS_SEND_TEXT(MAV_SEVERITY_INFO, "In landing sequence: mission reset");
           plane.mission.reset();
        }

        // Make sure the flight stage is correct for the new mode
        plane.update_flight_stage();
        
        // reset landing state
        plane.landing.reset();


#if HAL_QUADPLANE_ENABLED
        if (quadplane.enabled()) {
            float aspeed;
            bool have_airspeed = quadplane.ahrs.airspeed_estimate(aspeed);
            quadplane.assisted_flight = quadplane.assist.should_assist(aspeed, have_airspeed);
        }

        if (is_vtol_mode() && !quadplane.tailsitter.enabled()) {
            // if flying inverted and entering a VTOL mode cancel
            // inverted flight
            plane.inverted_flight = false;
        }
#endif
    }

    return enter_result;
}

/**
 * @brief Check if manual throttle control applies to VTOL motors in this mode
 * 
 * @details Determines whether the pilot has direct manual control of VTOL
 *          vertical thrust motors in the current mode. This is primarily used
 *          for tailsitter aircraft in Q-assisted forward flight.
 *          
 *          Special case: When a tailsitter has fully transitioned to Q-assisted
 *          forward flight, the forward throttle stick directly drives the vertical
 *          throttle, so the vertical throttle state must match the forward throttle
 *          control state. Note the inverted logic: forward throttle uses
 *          'does_auto_throttle' while vertical uses 'is_vtol_man_throttle'.
 * 
 * @return true if pilot has direct manual control of VTOL vertical throttle
 * @return false if VTOL vertical throttle is under automatic control or not applicable
 * 
 * @note This method is only relevant for quadplane builds with VTOL capabilities.
 *       For conventional fixed-wing builds, always returns false.
 * 
 * @see does_auto_throttle() for forward throttle control state
 * @see is_vtol_mode() for VTOL mode detection
 */
bool Mode::is_vtol_man_throttle() const
{
#if HAL_QUADPLANE_ENABLED
    if (plane.quadplane.tailsitter.is_in_fw_flight() &&
        plane.quadplane.assisted_flight) {
        // We are a tailsitter that has fully transitioned to Q-assisted forward flight.
        // In this case the forward throttle directly drives the vertical throttle so
        // set vertical throttle state to match the forward throttle state. Confusingly the booleans are inverted,
        // forward throttle uses 'does_auto_throttle' whereas vertical uses 'is_vtol_man_throttle'.
        return !does_auto_throttle();
    }
#endif
    return false;
}

/**
 * @brief Update the target altitude for TECS (Total Energy Control System)
 * 
 * @details Calculates and sets the target altitude based on the current flight phase
 *          and conditions. This method handles multiple special cases to provide
 *          smooth altitude tracking throughout the flight:
 *          
 *          Priority order:
 *          1. Landing flare: Use TECS_LAND_SINK as target sink rate
 *          2. Landing approach: Setup glide slope with optional rangefinder adjustment
 *          3. Landing phase with target location: Use landing-specific altitude
 *          4. Soaring with throttle suppressed: Lock to current altitude
 *          5. Reached loiter target: Lock to final waypoint altitude
 *          6. Terrain-following: Use terrain-relative altitude proportioning
 *          7. Climb/descent with offset: Proportional altitude between waypoints
 *          8. Default: Use next waypoint altitude as target
 * 
 * @note This method is called regularly during navigation modes to update the
 *       altitude target that TECS uses for energy management and climb/descent
 *       control. The target altitude directly affects aircraft vertical trajectory.
 * 
 * @warning Incorrect altitude targeting can cause dangerous flight behavior
 *          including terrain impact or excessive climb rates. All altitude
 *          calculations must account for terrain, obstacles, and aircraft limits.
 * 
 * @see Plane::set_target_altitude_location() for target altitude setting
 * @see AP_TECS for Total Energy Control System implementation
 */
void Mode::update_target_altitude()
{
    Location target_location;

    if (plane.landing.is_flaring()) {
        // during a landing flare, use TECS_LAND_SINK as a target sink
        // rate, and ignores the target altitude
        plane.set_target_altitude_location(plane.next_WP_loc);
    } else if (plane.landing.is_on_approach()) {
        plane.landing.setup_landing_glide_slope(plane.prev_WP_loc, plane.next_WP_loc, plane.current_loc, plane.target_altitude.offset_cm);
#if AP_RANGEFINDER_ENABLED
        plane.landing.adjust_landing_slope_for_rangefinder_bump(plane.rangefinder_state, plane.prev_WP_loc, plane.next_WP_loc, plane.current_loc, plane.auto_state.wp_distance, plane.target_altitude.offset_cm);
#endif
    } else if (plane.landing.get_target_altitude_location(target_location)) {
        plane.set_target_altitude_location(target_location);
#if HAL_SOARING_ENABLED
    } else if (plane.g2.soaring_controller.is_active() && plane.g2.soaring_controller.get_throttle_suppressed()) {
        // Reset target alt to current alt, to prevent large altitude errors when gliding.
        plane.set_target_altitude_location(plane.current_loc);
        plane.reset_offset_altitude();
#endif
    } else if (plane.reached_loiter_target()) {
        // once we reach a loiter target then lock to the final
        // altitude target
        plane.set_target_altitude_location(plane.next_WP_loc);
#if AP_TERRAIN_AVAILABLE
    } else if (plane.next_WP_loc.terrain_alt &&
               plane.set_target_altitude_proportion_terrain()) {
        // special case for target as terrain relative handled inside
        // set_target_altitude_proportion_terrain
#endif
    } else if (plane.target_altitude.offset_cm != 0 &&
               !plane.current_loc.past_interval_finish_line(plane.prev_WP_loc, plane.next_WP_loc)) {
        // control climb/descent rate
        plane.set_target_altitude_proportion(plane.next_WP_loc, 1.0f-plane.auto_state.wp_proportion);

        // stay within the range of the start and end locations in altitude
        plane.constrain_target_altitude_location(plane.next_WP_loc, plane.prev_WP_loc);
    } else {
        plane.set_target_altitude_location(plane.next_WP_loc);
    }
}

/**
 * @brief Perform pre-arm safety checks for this flight mode
 * 
 * @details Validates whether the vehicle can be safely armed in this mode.
 *          This method calls the mode-specific _pre_arm_checks() and provides
 *          a generic error message if the check fails without providing a
 *          specific reason. Pre-arm checks prevent arming in unsafe configurations
 *          that could lead to loss of control or unexpected behavior.
 * 
 * @param[in]  buflen Size of the error message buffer in bytes
 * @param[out] buffer Character buffer to receive error message if check fails
 * 
 * @return true if vehicle can be armed in this mode
 * @return false if arming should be prevented (buffer contains reason)
 * 
 * @note This is part of the arming safety system. Failed pre-arm checks will
 *       prevent the vehicle from arming until the issue is resolved.
 * 
 * @warning These checks are critical for flight safety. Bypassing or weakening
 *          pre-arm checks can lead to dangerous flight conditions.
 * 
 * @see _pre_arm_checks() for mode-specific check implementation
 * @see AP_Arming for overall arming system
 */
bool Mode::pre_arm_checks(size_t buflen, char *buffer) const
{
    if (!_pre_arm_checks(buflen, buffer)) {
        if (strlen(buffer) == 0) {
            // If no message is provided add a generic one
            hal.util->snprintf(buffer, buflen, "mode not armable");
        }
        return false;
    }

    return true;
}

/**
 * @brief Mode-specific pre-arm checks implementation
 * 
 * @details Implements mode-specific arming requirements. The base implementation
 *          checks for quadplane ONLY_ARM_IN_QMODE_OR_AUTO option, which restricts
 *          arming to Q modes or AUTO mode when enabled. This prevents accidental
 *          fixed-wing arming when VTOL-only operation is desired.
 *          
 *          Note: AUTO and Guided modes override this method to bypass the Q-mode
 *          check since they are explicitly allowed by the option name.
 * 
 * @param[in]  buflen Size of the error message buffer in bytes
 * @param[out] buffer Character buffer to receive error message if check fails
 * 
 * @return true if mode-specific arming requirements are met
 * @return false if mode cannot be armed (buffer contains reason)
 * 
 * @note Derived classes can override this method to implement mode-specific
 *       arming restrictions based on vehicle state or configuration.
 * 
 * @see pre_arm_checks() for the public interface
 */
bool Mode::_pre_arm_checks(size_t buflen, char *buffer) const
{
#if HAL_QUADPLANE_ENABLED
    if (plane.quadplane.enabled() && !is_vtol_mode() &&
            plane.quadplane.option_is_set(QuadPlane::OPTION::ONLY_ARM_IN_QMODE_OR_AUTO)) {
        hal.util->snprintf(buffer, buflen, "not Q mode");
        return false;
    }
#endif
    return true;
}

/**
 * @brief Execute mode control loop and stabilization
 * 
 * @details Called at the main scheduler loop rate to perform flight stabilization.
 *          This base implementation handles:
 *          1. Stick mixing (if configured): Blends pilot inputs with mode commands
 *          2. Roll stabilization: Maintains desired roll attitude
 *          3. Pitch stabilization: Maintains desired pitch attitude
 *          4. Yaw stabilization: Coordinates turns and maintains heading
 *          
 *          Stick mixing behavior depends on STICK_MIXING parameter:
 *          - NONE: No stick mixing, mode has full control
 *          - FBW: Fly-by-wire mixing of pilot inputs
 *          - FBW_NO_PITCH: FBW mixing but excluding pitch axis
 *          - VTOL_YAW: Yaw mixing only for VTOL modes
 *          - DIRECT_REMOVED: Legacy option, now maps to FBW mixing
 *          
 *          Most modes override this method to implement mode-specific control
 *          before or instead of calling the base stabilization.
 * 
 * @note This method is called at high frequency (typically 50-400 Hz depending
 *       on scheduler configuration). Performance is critical.
 * 
 * @warning This is the core control loop for fixed-wing stabilization. Changes
 *          to this method or the stabilization functions it calls can directly
 *          affect flight stability and safety.
 * 
 * @see Plane::stabilize_roll() for roll control implementation
 * @see Plane::stabilize_pitch() for pitch control implementation
 * @see Plane::stabilize_yaw() for yaw control implementation
 * @see Plane::stabilize_stick_mixing_fbw() for stick mixing implementation
 */
void Mode::run()
{
    // Direct stick mixing functionality has been removed, so as not to remove all stick mixing from the user completely
    // the old direct option is now used to enable fbw mixing, this is easier than doing a param conversion.
    switch ((StickMixing)plane.g.stick_mixing) {
        case StickMixing::FBW:
        case StickMixing::FBW_NO_PITCH:
        case StickMixing::DIRECT_REMOVED:
            plane.stabilize_stick_mixing_fbw();
            break;
        case StickMixing::NONE:
        case StickMixing::VTOL_YAW:
            break;
    }
    plane.stabilize_roll();
    plane.stabilize_pitch();
    plane.stabilize_yaw();
}

/**
 * @brief Reset all flight control integrators and steering state
 * 
 * @details Resets control system state to prevent integrator windup and unwanted
 *          control responses when starting a new control sequence. This method:
 *          1. Resets roll controller integrator to zero
 *          2. Resets pitch controller integrator to zero
 *          3. Resets yaw controller integrator to zero
 *          4. Clears steering locked course mode and accumulated error
 *          5. Resets TECS (Total Energy Control System) internal state
 *          
 *          Resetting integrators is important when:
 *          - Starting a new mode that changes control objectives
 *          - Beginning a new navigation segment (waypoint, landing, etc.)
 *          - Recovering from a control saturation condition
 *          - Transitioning between significantly different flight phases
 * 
 * @note This is typically called during mode transitions or at the start of
 *       new navigation segments to ensure clean initial conditions.
 * 
 * @warning Failing to reset integrators when appropriate can cause control
 *          overshoot, oscillation, or instability when control objectives change.
 *          However, unnecessary resets can reduce tracking performance.
 * 
 * @see AP_RollController::reset_I() for roll integrator reset
 * @see AP_PitchController::reset_I() for pitch integrator reset  
 * @see AP_YawController::reset_I() for yaw integrator reset
 * @see AP_TECS::reset() for TECS state reset
 */
void Mode::reset_controllers()
{
    // reset integrators
    plane.rollController.reset_I();
    plane.pitchController.reset_I();
    plane.yawController.reset_I();

    // reset steering controls
    plane.steer_state.locked_course = false;
    plane.steer_state.locked_course_err = 0;

    // reset TECS
    plane.TECS_controller.reset();
}

/**
 * @brief Check if vehicle is currently in takeoff flight stage
 * 
 * @details Simple query method that checks if the vehicle's flight stage is
 *          currently set to TAKEOFF. The flight stage is managed by the main
 *          flight code and transitions through TAKEOFF, NORMAL, LAND, and other
 *          stages during flight.
 * 
 * @return true if flight_stage is TAKEOFF
 * @return false if in any other flight stage
 * 
 * @note Flight stage affects control behavior, particularly for altitude
 *       management and automatic mode sequencing.
 * 
 * @see AP_FixedWing::FlightStage for flight stage definitions
 * @see Plane::update_flight_stage() for flight stage management
 */
bool Mode::is_taking_off() const
{
    return (plane.flight_stage == AP_FixedWing::FlightStage::TAKEOFF);
}

/**
 * @brief Output command value to both rudder and steering servo channels
 * 
 * @details Helper function that simultaneously sets the same output value to
 *          both the rudder servo function and the steering servo function.
 *          This is used for ground steering control where the rudder also
 *          controls the nose wheel or tail wheel steering. By setting both
 *          channels to the same value, the aircraft can coordinate rudder
 *          and ground steering for directional control.
 * 
 * @param[in] val Output value in scaled servo units (typically -4500 to +4500)
 * 
 * @note This is commonly used during ground operations (taxi, takeoff roll,
 *       landing rollout) where both air rudder and ground steering need to
 *       work together for directional control.
 * 
 * @see SRV_Channel::k_rudder for rudder servo function
 * @see SRV_Channel::k_steering for steering servo function
 * @see SRV_Channels::set_output_scaled() for servo output control
 */
void Mode::output_rudder_and_steering(float val)
{
    SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, val);
    SRV_Channels::set_output_scaled(SRV_Channel::k_steering, val);
}

/**
 * @brief Output pilot's throttle stick input to throttle servo channel
 * 
 * @details Processes and outputs the pilot's throttle stick position to the
 *          throttle servo channel. This is used in stabilized modes where the
 *          pilot has direct manual control of throttle (not auto-throttle modes).
 *          
 *          Two throttle mapping modes are supported:
 *          1. Direct passthrough (THR_PASS_STAB=1): Pilot stick directly maps
 *             to throttle output with no modification
 *          2. Adjusted mapping (THR_PASS_STAB=0): Applies trim correction curve
 *             if configured via flight options, centering throttle stick around
 *             TRIM_THROTTLE parameter value
 * 
 * @note This method is typically called by manual throttle modes (MANUAL,
 *       STABILIZE, FBWA, TRAINING, ACRO, AUTOTUNE) where pilot directly
 *       controls engine power.
 * 
 * @see Plane::get_throttle_input() for direct throttle reading
 * @see Plane::get_adjusted_throttle_input() for trim-adjusted throttle
 * @see SRV_Channel::k_throttle for throttle servo function
 */
void Mode::output_pilot_throttle()
{
    if (plane.g.throttle_passthru_stabilize) {
        // THR_PASS_STAB set, direct mapping
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, plane.get_throttle_input(true));
        return;
    }

    // get throttle, but adjust center to output TRIM_THROTTLE if flight option set
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, plane.get_adjusted_throttle_input(true));
}

/**
 * @brief Check if throttle min/max limits should be applied in this mode
 * 
 * @details Determines whether the throttle output should be constrained to
 *          configured minimum and maximum values (THR_MIN and THR_MAX parameters).
 *          
 *          Throttle limits are NOT applied when:
 *          - Navigation scripting is active (script has full control)
 *          - Manual throttle modes with direct passthrough enabled (STABILIZE,
 *            TRAINING, ACRO, FBWA, AUTOTUNE with THR_PASS_STAB=1)
 *          - GUIDED mode with throttle passthrough enabled
 *          - VTOL mode where forward throttle limiting is disabled
 *          
 *          Throttle limits ARE applied in automatic throttle modes to prevent
 *          excessive power or throttle cutoff during automated flight.
 * 
 * @return true if throttle should be limited to THR_MIN/THR_MAX range
 * @return false if throttle should pass through without min/max limiting
 * 
 * @note Throttle limiting is a safety feature for auto-throttle modes to
 *       prevent runaway throttle conditions or complete throttle cutoff.
 * 
 * @warning Disabling throttle limits in automatic modes can allow throttle
 *          to reach 0% (engine off) or 100% (full power) which may not be
 *          safe in all flight conditions.
 * 
 * @see Plane::g.throttle_passthru_stabilize (THR_PASS_STAB parameter)
 * @see does_auto_throttle() for auto-throttle mode detection
 */
bool Mode::use_throttle_limits() const
{
#if AP_SCRIPTING_ENABLED
    if (plane.nav_scripting_active()) {
        return false;
    }
#endif

    if (this == &plane.mode_stabilize ||
        this == &plane.mode_training ||
        this == &plane.mode_acro ||
        this == &plane.mode_fbwa ||
        this == &plane.mode_autotune) {
        // a manual throttle mode
        return !plane.g.throttle_passthru_stabilize;
    }

    if (is_guided_mode() && plane.guided_throttle_passthru) {
        // manual pass through of throttle while in GUIDED
        return false;
    }

#if HAL_QUADPLANE_ENABLED
    if (quadplane.in_vtol_mode()) {
        return quadplane.allow_forward_throttle_in_vtol_mode();
    }
#endif

    return true;
}

/**
 * @brief Check if battery voltage compensation should be applied to throttle
 * 
 * @details Determines whether throttle output should be compensated for battery
 *          voltage sag. Battery compensation increases throttle as battery voltage
 *          decreases to maintain consistent power output throughout the flight.
 *          This helps maintain consistent flight performance as the battery
 *          discharges.
 *          
 *          Battery compensation is NOT applied when:
 *          - Navigation scripting is active (script controls power directly)
 *          - Manual throttle modes (STABILIZE, TRAINING, ACRO, FBWA, AUTOTUNE)
 *            where pilot directly controls throttle
 *          - GUIDED mode with manual throttle passthrough enabled
 *          - Any VTOL mode (quadplane handles compensation separately)
 *          
 *          Battery compensation IS applied in automatic throttle modes to
 *          maintain consistent airspeed and climb performance throughout flight.
 * 
 * @return true if throttle should be compensated for battery voltage
 * @return false if throttle should be used without voltage compensation
 * 
 * @note Battery compensation is important for maintaining consistent performance
 *       in automatic modes as battery voltage decreases during flight. Without
 *       compensation, aircraft speed and climb rate would decrease as battery
 *       voltage drops.
 * 
 * @warning Battery compensation is disabled in manual modes to give pilot
 *          direct control feel. Enabling it in manual modes would cause throttle
 *          response to change as battery voltage changes.
 * 
 * @see AP_BattMonitor for battery voltage monitoring
 * @see does_auto_throttle() for auto-throttle mode detection
 */
bool Mode::use_battery_compensation() const
{
#if AP_SCRIPTING_ENABLED
    if (plane.nav_scripting_active()) {
        return false;
    }
#endif

    if (this == &plane.mode_stabilize ||
        this == &plane.mode_training ||
        this == &plane.mode_acro ||
        this == &plane.mode_fbwa ||
        this == &plane.mode_autotune) {
        // a manual throttle mode
        return false;
    }

    if (is_guided_mode() && plane.guided_throttle_passthru) {
        // manual pass through of throttle while in GUIDED
        return false;
    }

#if HAL_QUADPLANE_ENABLED
    if (quadplane.in_vtol_mode()) {
        return false;
    }
#endif

    return true;
}
