/**
 * @file mode_auto.cpp
 * @brief Implements AUTO flight mode for ArduPlane
 * 
 * @details This file implements AUTO mode which executes waypoint missions defined
 *          in the mission planner. AUTO mode supports:
 *          - Fixed-wing waypoint navigation using L1 controller
 *          - VTOL/QuadPlane operations when Q_ENABLE is configured
 *          - Automatic takeoff and landing sequences
 *          - Mission resume after watchdog recovery/reboot
 *          - Integration with soaring controller for autonomous thermal detection
 *          - Scripting integration for custom navigation behaviors
 * 
 *          AUTO mode processes MAV_CMD_NAV_* mission commands including:
 *          - MAV_CMD_NAV_WAYPOINT: Standard waypoint navigation
 *          - MAV_CMD_NAV_LAND: Automated landing approach and flare
 *          - MAV_CMD_NAV_TAKEOFF: Automated takeoff sequence
 *          - MAV_CMD_NAV_LOITER_*: Various loiter patterns
 *          - MAV_CMD_NAV_ALTITUDE_WAIT: Altitude hold with control relaxation
 *          - MAV_CMD_NAV_SCRIPT_TIME: Scripting-controlled navigation
 * 
 * @note For QuadPlane configurations, VTOL-specific commands are handled by the
 *       quadplane subsystem through quadplane.control_auto()
 * 
 * @warning AUTO mode requires a valid mission. If the mission stops unexpectedly,
 *          the aircraft will automatically switch to RTL mode for safety.
 * 
 * Source: ArduPlane/mode_auto.cpp:1-202
 */

#include "mode.h"
#include "Plane.h"

/**
 * @brief Initialize AUTO mode on entry
 * 
 * @details Performs comprehensive AUTO mode initialization sequence:
 *          1. QuadPlane Takeoff Validation: If transitioning from GUIDED mode with
 *             guided_wait_takeoff state, verifies that mission starts with a takeoff
 *             command. Refuses AUTO entry if takeoff waypoint is missing.
 *          2. VTOL Mode Determination: Sets vtol_mode based on Q_ENABLE parameter:
 *             - Q_ENABLE == 2: Forces VTOL mode for all AUTO operations
 *             - Otherwise: VTOL mode determined dynamically by mission commands
 *          3. Waypoint Initialization: Initializes next_WP_loc and prev_WP_loc to
 *             current position for navigation controller startup
 *          4. Mission Start/Resume: Calls mission.start_or_resume() which respects
 *             MIS_AUTORESET parameter to either start from beginning or resume
 *          5. Watchdog Recovery: If system rebooted due to watchdog, resumes mission
 *             at the waypoint number stored in persistent memory
 *          6. Soaring Initialization: If soaring is enabled, initializes cruising mode
 *             for autonomous thermal detection
 * 
 * @return true if AUTO mode entry successful
 * @return false if takeoff waypoint required but missing (QuadPlane safety check)
 * 
 * @note Watchdog recovery allows mission continuation after unexpected reboot,
 *       improving mission reliability in field operations
 * 
 * @warning Refusing AUTO mode entry without takeoff waypoint prevents unsafe
 *          QuadPlane operations where VTOL takeoff capability is required
 * 
 * Source: ArduPlane/mode_auto.cpp:4-43
 */
bool ModeAuto::_enter()
{
#if HAL_QUADPLANE_ENABLED
    // check if we should refuse auto mode due to a missing takeoff in
    // guided_wait_takeoff state
    if (plane.previous_mode == &plane.mode_guided &&
        quadplane.guided_wait_takeoff_on_mode_enter) {
        if (!plane.mission.starts_with_takeoff_cmd()) {
            gcs().send_text(MAV_SEVERITY_ERROR,"Takeoff waypoint required");
            quadplane.guided_wait_takeoff = true;
            return false;
        }
    }
    
    if (plane.quadplane.available() && plane.quadplane.enable == 2) {
        plane.auto_state.vtol_mode = true;
    } else {
        plane.auto_state.vtol_mode = false;
    }
#else
    plane.auto_state.vtol_mode = false;
#endif
    plane.next_WP_loc = plane.prev_WP_loc = plane.current_loc;
    // start or resume the mission, based on MIS_AUTORESET
    plane.mission.start_or_resume();

    if (hal.util->was_watchdog_armed()) {
        if (hal.util->persistent_data.waypoint_num != 0) {
            gcs().send_text(MAV_SEVERITY_INFO, "Watchdog: resume WP %u", hal.util->persistent_data.waypoint_num);
            plane.mission.set_current_cmd(hal.util->persistent_data.waypoint_num);
            hal.util->persistent_data.waypoint_num = 0;
        }
    }

#if HAL_SOARING_ENABLED
    plane.g2.soaring_controller.init_cruising();
#endif

    return true;
}

/**
 * @brief Cleanup when exiting AUTO mode
 * 
 * @details Performs safe shutdown of AUTO mode operations:
 *          1. Mission Stop: If mission is running, stops mission execution
 *          2. Landing Sequence Handling: For fixed-wing NAV_LAND operations,
 *             restarts the landing sequence to allow smooth transition to other
 *             modes during approach without losing landing waypoint state
 *          3. VTOL Landing: For QuadPlane VTOL landings, does NOT restart the
 *             landing sequence as VTOL land logic is handled differently
 *          4. State Cleanup: Clears auto_state.started_flying_in_auto_ms timestamp
 * 
 * @note Landing sequence restart allows safe mode transitions during approach
 *       phase without requiring mission replay from start
 * 
 * @note VTOL landings are excluded from restart logic because QuadPlane landing
 *       uses different state management through quadplane subsystem
 * 
 * Source: ArduPlane/mode_auto.cpp:45-61
 */
void ModeAuto::_exit()
{
    if (plane.mission.state() == AP_Mission::MISSION_RUNNING) {
        plane.mission.stop();

        bool restart = plane.mission.get_current_nav_cmd().id == MAV_CMD_NAV_LAND;
#if HAL_QUADPLANE_ENABLED
        if (plane.quadplane.is_vtol_land(plane.mission.get_current_nav_cmd().id)) {
            restart = false;
        }
#endif
        if (restart) {
            plane.landing.restart_landing_sequence();
        }
    }
    plane.auto_state.started_flying_in_auto_ms = 0;
}

/**
 * @brief Update navigation targets and control calculations for AUTO mode
 * 
 * @details Called at navigation rate to compute desired attitude and throttle for
 *          current mission command. Update sequence:
 *          
 *          1. Mission State Verification: Checks mission is running. If stopped
 *             unexpectedly (e.g., AP_Landing::restart_landing_sequence() failed),
 *             switches to RTL mode for safety.
 *          
 *          2. Command Dispatch: Gets current navigation command ID and delegates
 *             control based on vehicle state and command type:
 *             
 *             - VTOL Auto Mode: If quadplane.in_vtol_auto() is true, delegates
 *               all control to quadplane.control_auto() and returns early
 *             
 *             - Glider Pullup: If glider pullup maneuver is active, returns early
 *               to let pullup controller handle attitude
 *             
 *             - MAV_CMD_NAV_TAKEOFF: Calls takeoff-specific calculators for roll,
 *               pitch, and throttle with takeoff climb constraints
 *             
 *             - MAV_CMD_NAV_LAND or ABORT_LANDING: Calls navigation calculators
 *               for roll/pitch, constrains roll to level limits during flare,
 *               and handles throttle suppression when landing is complete
 *             
 *             - MAV_CMD_NAV_SCRIPT_TIME: For scripting control, maintains current
 *               roll/pitch sensor values (scripting controls rates directly)
 *             
 *             - All Other Commands: Standard navigation using L1 controller for
 *               lateral guidance and TECS for vertical guidance. Clears hold_course
 *               for normal waypoint tracking.
 * 
 * @note Called at navigation rate (typically 10-50Hz depending on scheduler config)
 * 
 * @note For VTOL operations, this function returns early after delegating to
 *       quadplane subsystem, which handles multirotor position control
 * 
 * @warning Mission stopping unexpectedly triggers automatic RTL for safety.
 *          This prevents the aircraft from continuing uncontrolled flight.
 * 
 * Source: ArduPlane/mode_auto.cpp:63-122
 */
void ModeAuto::update()
{
    if (plane.mission.state() != AP_Mission::MISSION_RUNNING) {
        // this could happen if AP_Landing::restart_landing_sequence() returns false which would only happen if:
        // restart_landing_sequence() is called when not executing a NAV_LAND or there is no previous nav point
        plane.set_mode(plane.mode_rtl, ModeReason::MISSION_END);
        gcs().send_text(MAV_SEVERITY_INFO, "Aircraft in auto without a running mission");
        return;
    }

    uint16_t nav_cmd_id = plane.mission.get_current_nav_cmd().id;

#if HAL_QUADPLANE_ENABLED
    if (plane.quadplane.in_vtol_auto()) {
        plane.quadplane.control_auto();
        return;
    }
#endif

#if AP_PLANE_GLIDER_PULLUP_ENABLED
    if (pullup.in_pullup()) {
        return;
    }
#endif

    if (nav_cmd_id == MAV_CMD_NAV_TAKEOFF ||
        (nav_cmd_id == MAV_CMD_NAV_LAND && plane.flight_stage == AP_FixedWing::FlightStage::ABORT_LANDING)) {
        plane.takeoff_calc_roll();
        plane.takeoff_calc_pitch();
        plane.takeoff_calc_throttle();
    } else if (nav_cmd_id == MAV_CMD_NAV_LAND) {
        plane.calc_nav_roll();
        plane.calc_nav_pitch();

        // allow landing to restrict the roll limits
        plane.nav_roll_cd = plane.landing.constrain_roll(plane.nav_roll_cd, plane.g.level_roll_limit*100UL);

        if (plane.landing.is_throttle_suppressed()) {
            // if landing is considered complete throttle is never allowed, regardless of landing type
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0.0);
        } else {
            plane.calc_throttle();
        }
#if AP_SCRIPTING_ENABLED
    } else if (nav_cmd_id == MAV_CMD_NAV_SCRIPT_TIME) {
        // NAV_SCRIPTING has a desired roll and pitch rate and desired throttle
        plane.nav_roll_cd = ahrs.roll_sensor;
        plane.nav_pitch_cd = ahrs.pitch_sensor;
#endif
    } else {
        // we are doing normal AUTO flight, the special cases
        // are for takeoff and landing
        if (nav_cmd_id != MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT) {
            plane.steer_state.hold_course_cd = -1;
        }
        plane.calc_nav_roll();
        plane.calc_nav_pitch();
        plane.calc_throttle();
    }
}

/**
 * @brief Advance mission to next waypoint when appropriate
 * 
 * @details Updates mission state machine to progress through mission commands.
 *          Only updates mission if home position is set (required for distance
 *          and bearing calculations to mission waypoints).
 *          
 *          The mission.update() call handles:
 *          - Checking if current waypoint has been reached
 *          - Advancing to next mission command
 *          - Processing DO_ commands (immediate actions)
 *          - Managing mission completion and looping
 * 
 * @note Called at slower rate than update() - typically 5-10Hz
 * 
 * @note Home position is required for mission navigation because many mission
 *       commands use home-relative coordinates or require distance calculations
 * 
 * Source: ArduPlane/mode_auto.cpp:124-129
 */
void ModeAuto::navigate()
{
    if (AP::ahrs().home_is_set()) {
        plane.mission.update();
    }
}


/**
 * @brief Check if mode is performing autonomous navigation
 * 
 * @details Determines whether AUTO mode is actively controlling navigation or if
 *          navigation control has been delegated to scripting.
 * 
 * @return true if AUTO mode is controlling navigation (normal operation)
 * @return false if navigation is delegated to active scripting (MAV_CMD_NAV_SCRIPT_TIME)
 * 
 * @note When scripting is active (nav_scripting_active() returns true), this returns
 *       false to indicate that navigation calculations should not override script control
 * 
 * @note Used by base mode logic to determine if autonomous navigation calculations
 *       should be performed
 * 
 * Source: ArduPlane/mode_auto.cpp:132-138
 */
bool ModeAuto::does_auto_navigation() const
{
#if AP_SCRIPTING_ENABLED
   return (!plane.nav_scripting_active());
#endif
   return true;
}

/**
 * @brief Check if mode is performing autonomous throttle control
 * 
 * @details Determines whether AUTO mode is actively controlling throttle or if
 *          throttle control has been delegated to scripting.
 * 
 * @return true if AUTO mode is controlling throttle (normal operation)
 * @return false if throttle is delegated to active scripting (MAV_CMD_NAV_SCRIPT_TIME)
 * 
 * @note When scripting is active (nav_scripting_active() returns true), this returns
 *       false to indicate that throttle calculations should not override script control
 * 
 * @note Used by base mode logic to determine if autonomous throttle calculations
 *       should be performed
 * 
 * Source: ArduPlane/mode_auto.cpp:140-146
 */
bool ModeAuto::does_auto_throttle() const
{
#if AP_SCRIPTING_ENABLED
   return (!plane.nav_scripting_active());
#endif
   return true;
}

/**
 * @brief Perform pre-arm safety checks specific to AUTO mode
 * 
 * @details Validates mission configuration before allowing arming in AUTO mode.
 *          Performs QuadPlane-specific checks when applicable:
 *          
 *          1. VTOL Takeoff Requirement: If QuadPlane option ONLY_ARM_IN_QMODE_OR_AUTO
 *             is set, verifies that the current mission command is a VTOL takeoff.
 *             Prevents arming if starting on non-VTOL command which could be unsafe.
 *          
 *          2. Takeoff Waypoint: For QuadPlane configurations, ensures mission starts
 *             with a takeoff command. Prevents arming without proper takeoff sequence
 *             which could result in immediate landing detection or unsafe behavior.
 * 
 * @param[in] buflen Size of error message buffer
 * @param[out] buffer Error message written here if check fails
 * 
 * @return true if pre-arm checks pass and vehicle can be armed in AUTO mode
 * @return false if checks fail (error message written to buffer)
 * 
 * @note This implementation bypasses base class checks and only performs AUTO-specific
 *       QuadPlane validation. Other general pre-arm checks are performed elsewhere.
 * 
 * @warning For QuadPlane, missing takeoff waypoint is a critical safety issue.
 *          Immediate post-arm behavior without takeoff command can be unpredictable.
 * 
 * Source: ArduPlane/mode_auto.cpp:149-166
 */
bool ModeAuto::_pre_arm_checks(size_t buflen, char *buffer) const
{
#if HAL_QUADPLANE_ENABLED
    if (plane.quadplane.enabled()) {
        if (plane.quadplane.option_is_set(QuadPlane::OPTION::ONLY_ARM_IN_QMODE_OR_AUTO) &&
                !plane.quadplane.is_vtol_takeoff(plane.mission.get_current_nav_cmd().id)) {
            hal.util->snprintf(buffer, buflen, "not in VTOL takeoff");
            return false;
        }
        if (!plane.mission.starts_with_takeoff_cmd()) {
            hal.util->snprintf(buffer, buflen, "missing takeoff waypoint");
            return false;
        }
    }
#endif
    // Note that this bypasses the base class checks
    return true;
}

/**
 * @brief Check if aircraft is currently in landing phase
 * 
 * @details Queries the flight stage to determine if aircraft is executing landing.
 *          Returns true when flight stage is LAND, indicating aircraft is in final
 *          approach, flare, or touchdown phase.
 * 
 * @return true if flight_stage is AP_FixedWing::FlightStage::LAND
 * @return false otherwise
 * 
 * @note Flight stage LAND is set by landing sequence controller when aircraft
 *       transitions from approach to final landing phases
 * 
 * @note Used by other systems to modify behavior during landing (e.g., limiting
 *       roll authority, suppressing throttle, disabling certain failsafes)
 * 
 * Source: ArduPlane/mode_auto.cpp:168-171
 */
bool ModeAuto::is_landing() const
{
    return (plane.flight_stage == AP_FixedWing::FlightStage::LAND);
}

/**
 * @brief Main control loop execution for AUTO mode
 * 
 * @details Implements the primary control execution for AUTO mode, called at main
 *          loop rate (typically 50-400Hz depending on board and configuration).
 *          
 *          Control Flow:
 *          1. Glider Pullup: If pullup maneuver is active, runs pullup stabilization
 *             controller and returns early (pullup overrides normal AUTO control)
 *          
 *          2. Altitude Wait: If current mission command is MAV_CMD_NAV_ALTITUDE_WAIT:
 *             - Wiggles servos to keep them exercised during long waits
 *             - Sets throttle outputs to 0.0 for all throttle channels
 *             - Sets throttle outputs to trim values (typically mid-range PWM)
 *             - Resets attitude controllers to prevent integrator wind-up
 *             - This creates a safe holding state with minimal control activity
 *          
 *          3. Normal Flight: For all other commands, delegates to Mode::run() which
 *             executes standard control pipeline (stabilization, output mixing, etc.)
 * 
 * @note Called at main loop rate - typically 50Hz for most boards, up to 400Hz for
 *       high-performance flight controllers
 * 
 * @note Altitude wait is used for scenarios like waiting for balloon release or
 *       parachute deployment where aircraft should minimize control inputs
 * 
 * @note Servo wiggling during altitude wait prevents servo binding and keeps
 *       mechanical linkages exercised during extended holds
 * 
 * Source: ArduPlane/mode_auto.cpp:173-202
 */
void ModeAuto::run()
{
#if AP_PLANE_GLIDER_PULLUP_ENABLED
    if (pullup.in_pullup()) {
        pullup.stabilize_pullup();
        return;
    }
#endif
    
    if (plane.mission.get_current_nav_cmd().id == MAV_CMD_NAV_ALTITUDE_WAIT) {

        wiggle_servos();

        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0.0);
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttleLeft, 0.0);
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttleRight, 0.0);

        SRV_Channels::set_output_to_trim(SRV_Channel::k_throttle);
        SRV_Channels::set_output_to_trim(SRV_Channel::k_throttleLeft);
        SRV_Channels::set_output_to_trim(SRV_Channel::k_throttleRight);

        // Relax attitude control
        reset_controllers();

    } else {
        // Normal flight, run base class
        Mode::run();

    }
}
