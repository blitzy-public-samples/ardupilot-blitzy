/**
 * @file commands_logic.cpp
 * @brief Mission command execution and verification logic for ArduPlane
 * 
 * @details This file implements the core mission command handling system for fixed-wing
 *          aircraft in ArduPilot. It provides two primary functions:
 *          
 *          1. Command Initiation (start_command): Configures vehicle state and control
 *             parameters when beginning execution of a mission command
 *          
 *          2. Command Verification (verify_command): Monitors ongoing mission commands
 *             and determines when completion criteria have been met
 *          
 *          The file handles three categories of mission commands:
 *          - NAV commands: Navigation waypoints, takeoff, landing, loiter patterns
 *          - CONDITION commands: Time delays, distance triggers
 *          - DO commands: Immediate actions like speed changes, camera control
 *          
 *          Mission Command State Tracking:
 *          State is maintained in auto_state structure including:
 *          - Waypoint locations (prev_WP_loc, next_WP_loc)
 *          - Loiter state (start time, turn count, direction)
 *          - Takeoff/landing phases (completion flags, altitude targets)
 *          - Flight stage transitions
 *          - Condition values for multi-phase commands
 *          
 *          Verification Logic:
 *          Each NAV command type has specific completion criteria:
 *          - Waypoints: Distance threshold or finish line crossed
 *          - Loiter: Time elapsed, turns completed, or altitude reached
 *          - Takeoff: Target altitude achieved
 *          - Landing: Flare completion and touchdown detection
 *          
 *          Safety Considerations:
 *          - All commands validated before execution
 *          - Quadplane VTOL transitions handled with appropriate checks
 *          - Terrain following integrated where applicable
 *          - Failsafe mechanisms for command timeouts
 * 
 * @note This file is vehicle-specific for ArduPlane and integrates with the
 *       vehicle-agnostic AP_Mission library for command sequencing
 * 
 * @see AP_Mission for mission management and command sequencing
 * @see Plane::auto_state for mission execution state variables
 */

#include "Plane.h"

/********************************************************************************/
// Command Event Handlers
/********************************************************************************/

/**
 * @brief Initialize and begin execution of a mission command
 * 
 * @details This function is called when the mission library begins a new command.
 *          It performs command-specific initialization including:
 *          - Setting navigation targets and waypoint locations
 *          - Configuring control parameters (speed, altitude, heading)
 *          - Initializing state tracking variables
 *          - Starting timers for time-based commands
 *          - Handling vehicle-specific setup (fixed-wing vs VTOL)
 *          
 *          For NAV commands, special handling includes:
 *          - Resetting navigation controller staleness flag
 *          - Clearing loiter start time for new loiter commands
 *          - Looking ahead to detect land approach waypoints
 *          - Disabling idle mode to enable active control
 *          
 *          Command categories handled:
 *          - Navigation: TAKEOFF, WAYPOINT, LAND, LOITER variants, RTL
 *          - Conditional: DELAY, DISTANCE
 *          - Do: CHANGE_SPEED, SET_HOME, INVERTED_FLIGHT, VTOL_TRANSITION
 *          
 * @param[in] cmd Mission command to execute from AP_Mission library
 * 
 * @return true if command was recognized and initialization succeeded
 * @return false if command is not supported (mission advances to next command)
 * 
 * @note Called from AP_Mission via start_command_callback at command start
 * @note Only executes when control_mode == mode_auto
 * 
 * @see verify_command() for command completion checking
 * @see AP_Mission::Mission_Command for command structure definition
 */
bool Plane::start_command(const AP_Mission::Mission_Command& cmd)
{
    // default to non-VTOL loiter
    auto_state.vtol_loiter = false;

#if AP_TERRAIN_AVAILABLE
    plane.target_altitude.terrain_following_pending = false;
#endif

    // special handling for nav vs non-nav commands
    if (AP_Mission::is_nav_cmd(cmd)) {
        // set takeoff_complete to true so we don't add extra elevator
        // except in a takeoff
        auto_state.takeoff_complete = true;

        nav_controller->set_data_is_stale();

        // start non-idle
        auto_state.idle_mode = false;
        
        // reset loiter start time. New command is a new loiter
        loiter.start_time_ms = 0;

        // Mission lookahead is only valid in auto
        if (control_mode == &mode_auto) {
            AP_Mission::Mission_Command next_nav_cmd;
            const uint16_t next_index = mission.get_current_nav_index() + 1;
            const bool have_next_cmd = mission.get_next_nav_cmd(next_index, next_nav_cmd);
            auto_state.wp_is_land_approach = have_next_cmd && (next_nav_cmd.id == MAV_CMD_NAV_LAND);
#if HAL_QUADPLANE_ENABLED
            if (have_next_cmd && quadplane.is_vtol_land(next_nav_cmd.id)) {
                auto_state.wp_is_land_approach = false;
            }
#endif
        }
    }

    switch(cmd.id) {

    case MAV_CMD_NAV_TAKEOFF:
        crash_state.is_crashed = false;
#if HAL_QUADPLANE_ENABLED
        if (quadplane.is_vtol_takeoff(cmd.id)) {
            return quadplane.do_vtol_takeoff(cmd);
        }
#endif
        do_takeoff(cmd);
        break;

    case MAV_CMD_NAV_WAYPOINT:                  // Navigate to Waypoint
        do_nav_wp(cmd);
        break;

    case MAV_CMD_NAV_LAND:              // LAND to Waypoint
#if HAL_QUADPLANE_ENABLED
        if (quadplane.is_vtol_land(cmd.id)) {
            crash_state.is_crashed = false;
            return quadplane.do_vtol_land(cmd);            
        }
#endif
        do_land(cmd);
        break;

    case MAV_CMD_NAV_LOITER_UNLIM:              // Loiter indefinitely
        do_loiter_unlimited(cmd);
        break;

    case MAV_CMD_NAV_LOITER_TURNS:              // Loiter N Times
        do_loiter_turns(cmd);
        break;

    case MAV_CMD_NAV_LOITER_TIME:
        do_loiter_time(cmd);
        break;

    case MAV_CMD_NAV_LOITER_TO_ALT:
        do_loiter_to_alt(cmd);
        break;

    case MAV_CMD_NAV_RETURN_TO_LAUNCH:
        set_mode(mode_rtl, ModeReason::MISSION_CMD);
        break;

    case MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT:
        do_continue_and_change_alt(cmd);
        break;

    case MAV_CMD_NAV_ALTITUDE_WAIT:
        do_altitude_wait(cmd);
        break;

#if HAL_QUADPLANE_ENABLED
    case MAV_CMD_NAV_VTOL_TAKEOFF:
        crash_state.is_crashed = false;
        return quadplane.do_vtol_takeoff(cmd);

    case MAV_CMD_NAV_VTOL_LAND:
    case MAV_CMD_NAV_PAYLOAD_PLACE:
        if (quadplane.landing_with_fixed_wing_spiral_approach()) {
            // the user wants to approach the landing in a fixed wing flight mode
            // the waypoint will be used as a loiter_to_alt
            // after which point the plane will compute the optimal into the wind direction
            // and fly in on that direction towards the landing waypoint
            // it will then transition to VTOL and do a normal quadplane landing
            do_landing_vtol_approach(cmd);
            break;
        } else {
            return quadplane.do_vtol_land(cmd);
        }
#endif

    // Conditional commands

    case MAV_CMD_CONDITION_DELAY:
        do_wait_delay(cmd);
        break;

    case MAV_CMD_CONDITION_DISTANCE:
        do_within_distance(cmd);
        break;

    // Do commands

    case MAV_CMD_DO_CHANGE_SPEED:
        do_change_speed(cmd);
        break;

    case MAV_CMD_DO_SET_HOME:
        do_set_home(cmd);
        break;

    case MAV_CMD_DO_INVERTED_FLIGHT:
        if (cmd.p1 == 0 || cmd.p1 == 1) {
            auto_state.inverted_flight = (bool)cmd.p1;
            gcs().send_text(MAV_SEVERITY_INFO, "Set inverted %u", cmd.p1);
        }
        break;

    case MAV_CMD_DO_RETURN_PATH_START:
    case MAV_CMD_DO_LAND_START:
        break;

    case MAV_CMD_DO_AUTOTUNE_ENABLE:
        autotune_enable(cmd.p1);
        break;

#if HAL_MOUNT_ENABLED
    // Sets the region of interest (ROI) for a sensor set or the
    // vehicle itself. This can then be used by the vehicles control
    // system to control the vehicle attitude and the attitude of various
    // devices such as cameras.
    //    |Region of interest mode. (see MAV_ROI enum)| Waypoint index/ target ID. (see MAV_ROI enum)| ROI index (allows a vehicle to manage multiple cameras etc.)| Empty| x the location of the fixed ROI (see MAV_FRAME)| y| z|
    // ROI_NONE can be handled by the regular ROI handler because lat, lon, alt are always zero
    case MAV_CMD_DO_SET_ROI_LOCATION:
    case MAV_CMD_DO_SET_ROI_NONE:
    case MAV_CMD_DO_SET_ROI:
        if (cmd.content.location.alt == 0 && cmd.content.location.lat == 0 && cmd.content.location.lng == 0) {
            // switch off the camera tracking if enabled
            if (camera_mount.get_mode() == MAV_MOUNT_MODE_GPS_POINT) {
                camera_mount.set_mode_to_default();
            }
        } else {
            // set mount's target location
            camera_mount.set_roi_target(cmd.content.location);
        }
        break;

    case MAV_CMD_DO_MOUNT_CONTROL:          // 205
        // point the camera to a specified angle
        camera_mount.set_angle_target(cmd.content.mount_control.roll, cmd.content.mount_control.pitch, cmd.content.mount_control.yaw, false);
        break;
#endif

#if HAL_QUADPLANE_ENABLED
    case MAV_CMD_DO_VTOL_TRANSITION:
        plane.quadplane.handle_do_vtol_transition((enum MAV_VTOL_STATE)cmd.content.do_vtol_transition.target_state);
        break;
#endif

#if AP_ICENGINE_ENABLED
    case MAV_CMD_DO_ENGINE_CONTROL:
        plane.g2.ice_control.engine_control(cmd.content.do_engine_control.start_control,
                                            cmd.content.do_engine_control.cold_start,
                                            cmd.content.do_engine_control.height_delay_cm*0.01f,
                                            cmd.content.do_engine_control.allow_disarmed_start);
        break;
#endif

#if AP_SCRIPTING_ENABLED
    case MAV_CMD_NAV_SCRIPT_TIME:
        do_nav_script_time(cmd);
        break;
#endif

    case MAV_CMD_NAV_DELAY:
        mode_auto.do_nav_delay(cmd);
        break;
        
    default:
        // unable to use the command, allow the vehicle to try the next command
        return false;
    }

    return true;
}

/*******************************************************************************
Verify command Handlers

Each type of mission element has a "verify" operation. The verify
operation returns true when the mission element has completed and we
should move onto the next mission element.
Return true if we do not recognize the command so that we move on to the next command
*******************************************************************************/

/**
 * @brief Check if the currently executing mission command has completed
 * 
 * @details This function is called repeatedly (typically at 10Hz or higher) to monitor
 *          the progress of the active mission command. Each command type has specific
 *          completion criteria that are evaluated:
 *          
 *          Navigation Command Completion Criteria:
 *          - TAKEOFF: Target altitude reached or timeout
 *          - WAYPOINT: Within acceptance radius or past finish line
 *          - LAND: Touchdown detected or abort condition met
 *          - LOITER_TIME: Time elapsed and heading aligned
 *          - LOITER_TURNS: Turn count reached and heading aligned
 *          - LOITER_TO_ALT: Altitude and heading targets achieved
 *          
 *          Conditional Command Completion:
 *          - DELAY: Timer expired
 *          - DISTANCE: Within specified distance to waypoint
 *          
 *          DO Command Completion:
 *          - Most DO commands complete immediately (return true)
 *          - State changes take effect and mission continues
 *          
 *          Multi-Phase Commands:
 *          Some commands have primary and secondary completion phases tracked
 *          via condition_value flag:
 *          - LOITER_TIME: First wait for time, then align heading
 *          - LOITER_TURNS: First complete turns, then align heading
 *          - LOITER_TO_ALT: First reach altitude, then align heading
 *          
 * @param[in] cmd Currently active mission command to verify
 * 
 * @return true if command has completed and mission should advance
 * @return false if command is still in progress
 * 
 * @note Called from AP_Mission via verify_command_callback at scheduler rate
 * @note Only executes when control_mode == mode_auto
 * @note Unrecognized commands return true to prevent mission stall
 * 
 * @see start_command() for command initialization
 * @see verify_nav_wp() for waypoint completion logic
 * @see verify_takeoff() for takeoff completion logic
 * @see verify_loiter_time(), verify_loiter_turns(), verify_loiter_to_alt()
 */
bool Plane::verify_command(const AP_Mission::Mission_Command& cmd)        // Returns true if command complete
{
    switch(cmd.id) {

    case MAV_CMD_NAV_TAKEOFF:
#if HAL_QUADPLANE_ENABLED
        if (quadplane.is_vtol_takeoff(cmd.id)) {
            return quadplane.verify_vtol_takeoff(cmd);
        }
#endif
        return verify_takeoff();

    case MAV_CMD_NAV_WAYPOINT:
        return verify_nav_wp(cmd);

    case MAV_CMD_NAV_LAND:
#if HAL_QUADPLANE_ENABLED
        if (quadplane.is_vtol_land(cmd.id)) {
            return quadplane.verify_vtol_land();
        }
#endif
        if (flight_stage == AP_FixedWing::FlightStage::ABORT_LANDING) {
            return landing.verify_abort_landing(prev_WP_loc, next_WP_loc, current_loc, auto_state.takeoff_altitude_rel_cm, throttle_suppressed);

        } else {
            // use rangefinder to correct if possible
            bool rangefinder_active = false;
            float height = plane.get_landing_height(rangefinder_active);

            // for flare calculations we don't want to use the terrain
            // correction as otherwise we will flare early on rising
            // ground
            height -= auto_state.terrain_correction;
            return landing.verify_land(prev_WP_loc, next_WP_loc, current_loc,
                                       height, auto_state.sink_rate, auto_state.wp_proportion, auto_state.last_flying_ms, arming.is_armed(), is_flying(),
                                       rangefinder_active);
        }

    case MAV_CMD_NAV_LOITER_UNLIM:
        return verify_loiter_unlim(cmd);

    case MAV_CMD_NAV_LOITER_TURNS:
        return verify_loiter_turns(cmd);

    case MAV_CMD_NAV_LOITER_TIME:
        return verify_loiter_time();

    case MAV_CMD_NAV_LOITER_TO_ALT:
        return verify_loiter_to_alt(cmd);


    case MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT:
        return verify_continue_and_change_alt();

    case MAV_CMD_NAV_ALTITUDE_WAIT:
        return mode_auto.verify_altitude_wait(cmd);

#if HAL_QUADPLANE_ENABLED
    case MAV_CMD_NAV_VTOL_TAKEOFF:
        return quadplane.verify_vtol_takeoff(cmd);
    case MAV_CMD_NAV_VTOL_LAND:
    case MAV_CMD_NAV_PAYLOAD_PLACE:
        if (quadplane.landing_with_fixed_wing_spiral_approach() && !verify_landing_vtol_approach(cmd)) {
            // verify_landing_vtol_approach will return true once we have completed the approach,
            // in which case we fall over to normal vtol landing code
            return false;
        } else {
            return quadplane.verify_vtol_land();
        }
#endif  // HAL_QUADPLANE_ENABLED

    // Conditional commands

    case MAV_CMD_CONDITION_DELAY:
        return verify_wait_delay();

    case MAV_CMD_CONDITION_DISTANCE:
        return verify_within_distance();

#if AP_SCRIPTING_ENABLED
    case MAV_CMD_NAV_SCRIPT_TIME:
        return verify_nav_script_time(cmd);
#endif

     case MAV_CMD_NAV_DELAY:
         return mode_auto.verify_nav_delay(cmd);

    // do commands (always return true)
    case MAV_CMD_DO_CHANGE_SPEED:
    case MAV_CMD_DO_SET_HOME:
    case MAV_CMD_DO_INVERTED_FLIGHT:
    case MAV_CMD_DO_RETURN_PATH_START:
    case MAV_CMD_DO_LAND_START:
    case MAV_CMD_DO_FENCE_ENABLE:
    case MAV_CMD_DO_AUTOTUNE_ENABLE:
    case MAV_CMD_DO_SET_CAM_TRIGG_DIST:
    case MAV_CMD_DO_SET_ROI_LOCATION:
    case MAV_CMD_DO_SET_ROI_NONE:
    case MAV_CMD_DO_SET_ROI:
    case MAV_CMD_DO_MOUNT_CONTROL:
    case MAV_CMD_DO_VTOL_TRANSITION:
    case MAV_CMD_DO_ENGINE_CONTROL:
        return true;

    default:
        // error message
        gcs().send_text(MAV_SEVERITY_WARNING,"Skipping invalid cmd #%i",cmd.id);
        // return true if we do not recognize the command so that we move on to the next command
        return true;
    }
}

/********************************************************************************/
//  Nav (Must) commands
/********************************************************************************/

/**
 * @brief Initialize Return To Launch navigation
 * 
 * @details Configures the aircraft to navigate to home or nearest rally point at
 *          specified altitude. This is the core RTL initialization used by RTL mode
 *          and the MAV_CMD_NAV_RETURN_TO_LAUNCH mission command.
 *          
 *          Navigation Setup:
 *          1. Disable crosstrack navigation for direct path
 *          2. Set previous waypoint to current location
 *          3. Calculate best rally point or home as destination
 *          4. Apply terrain following corrections if enabled
 *          5. Set target altitude for climb/descent
 *          6. Configure loiter direction from aparm.loiter_radius sign
 *          7. Setup glide slope and turn angle calculations
 *          
 *          Destination Selection:
 *          - Chooses nearest rally point if configured and enabled
 *          - Falls back to home location if no rally points
 *          - Uses provided rtl_altitude_AMSL_cm as target altitude
 *          
 *          Altitude Handling:
 *          - rtl_altitude_AMSL_cm: Absolute altitude in centimeters MSL
 *          - Applied to destination location before navigation
 *          - Aircraft climbs or descends to reach target altitude
 *          - Integrates with terrain following if enabled
 *          
 *          Crosstrack Disabled:
 *          - Direct path to destination without line following
 *          - Prevents wide turns when RTL activated mid-mission
 *          - Ensures most direct route to safety
 * 
 * @param[in] rtl_altitude_AMSL_cm Target altitude in centimeters Above Mean Sea Level
 * 
 * @note Called by RTL mode initialization and NAV_RETURN_TO_LAUNCH command
 * @note Rally point selection requires HAL_RALLY_ENABLED
 * @note Terrain corrections applied if AP_TERRAIN_AVAILABLE
 * 
 * @see calc_best_rally_or_home_location() for destination selection
 * @see setup_terrain_target_alt() for terrain following integration
 * @see setup_alt_slope() for altitude profile calculation
 * @see setup_turn_angle() for turn radius computation
 */
void Plane::do_RTL(int32_t rtl_altitude_AMSL_cm)
{
    auto_state.next_wp_crosstrack = false;
    auto_state.crosstrack = false;
    prev_WP_loc = current_loc;
    next_WP_loc = calc_best_rally_or_home_location(current_loc, rtl_altitude_AMSL_cm);

    fix_terrain_WP(next_WP_loc, __LINE__);

    setup_terrain_target_alt(next_WP_loc);
    set_target_altitude_location(next_WP_loc);

    if (aparm.loiter_radius < 0) {
        loiter.direction = -1;
    } else {
        loiter.direction = 1;
    }

    setup_alt_slope();
    setup_turn_angle();
}

/**
 * @brief Calculate optimal rally point or home location for RTL
 * 
 * @details Determines the best destination for return-to-launch based on current
 *          position and available rally points. Chooses nearest suitable location
 *          and applies specified altitude.
 *          
 *          Selection Logic:
 *          If HAL_RALLY_ENABLED (rally point support compiled in):
 *          - Delegates to rally.calc_best_rally_or_home_location()
 *          - Considers all configured rally points
 *          - Returns nearest rally point to current location
 *          - Falls back to home if no rally points configured
 *          - Respects rally point enable/disable state
 *          
 *          If rally support not compiled:
 *          - Always returns home location
 *          - Applies rtl_home_alt_amsl_cm as absolute altitude
 *          
 *          Altitude Application:
 *          - Sets altitude in ABSOLUTE frame (MSL)
 *          - Overrides stored altitude of selected location
 *          - Ensures consistent RTL altitude regardless of destination
 *          
 *          Usage Context:
 *          - Called during RTL initialization
 *          - Used by failsafe RTL
 *          - Destination for mission RTL commands
 * 
 * @param[in] _current_loc Current aircraft location for distance comparison
 * @param[in] rtl_home_alt_amsl_cm Target altitude in centimeters AMSL
 * 
 * @return Location object with nearest rally point or home, at specified altitude
 * 
 * @note Rally point selection requires HAL_RALLY_ENABLED build option
 * @note Returns home location if no rally points configured
 * @note Altitude always set in ABSOLUTE frame (MSL)
 * 
 * @see plane.rally.calc_best_rally_or_home_location() for rally selection logic
 * @see plane.home for home location coordinates
 */
Location Plane::calc_best_rally_or_home_location(const Location &_current_loc, float rtl_home_alt_amsl_cm) const
{
#if HAL_RALLY_ENABLED
    return plane.rally.calc_best_rally_or_home_location(_current_loc, rtl_home_alt_amsl_cm);
#else
    Location destination = plane.home;
    destination.set_alt_cm(rtl_home_alt_amsl_cm, Location::AltFrame::ABSOLUTE);
    return destination;
#endif
}

/**
 * @brief Initialize fixed-wing takeoff command
 * 
 * @details Configures aircraft for automatic takeoff sequence. Sets waypoint targets,
 *          pitch attitude, and initializes takeoff state tracking. Handles transition
 *          from ground roll through rotation to climb-out.
 *          
 *          Waypoint Configuration:
 *          - prev_WP_loc set to current location (takeoff start point)
 *          - next_WP_loc initially set from command location altitude
 *          - Lat/lon artificially offset from home to create heading reference
 *          - Actual navigation uses GPS ground course, not waypoint bearing
 *          
 *          Pitch Management:
 *          - takeoff_pitch_cd from cmd.p1 in degrees * 100
 *          - Defaults to 4 degrees (400 centidegrees) if not specified
 *          - Maintains nose-up attitude during initial climb
 *          - Transitions to level flight at target altitude
 *          
 *          Altitude Tracking:
 *          - takeoff_altitude_rel_cm: Target altitude relative to home
 *          - baro_takeoff_alt: Barometric altitude at start (for fallback)
 *          - height_below_takeoff_to_level_off_cm: Pitch level-off tracking
 *          
 *          State Flags:
 *          - takeoff_complete = false: Enables special takeoff handling
 *          - rotation_complete = false: Ground roll vs airborne transition
 *          - Uses GPS ground course for heading during ground roll
 *          - IMU yaw drift correction active until rotation
 *          
 *          Steering Initialization:
 *          - locked_course_err = 0: Clear any previous course errors
 *          - hold_course_cd = -1: Signals to compute course when GPS ready
 *          - Course locked once GPS speed exceeds threshold
 *          
 *          Typical Takeoff Sequence:
 *          1. Ground roll with locked heading from GPS
 *          2. Pitch to takeoff_pitch_cd as speed increases
 *          3. Monitor altitude for rotation detection
 *          4. Climb to target altitude
 *          5. Level off and transition to normal navigation
 * 
 * @param[in] cmd Mission command containing takeoff parameters
 *                cmd.p1: Pitch angle in degrees (0 = default 4 degrees)
 *                cmd.content.location: Target altitude and optional lat/lon
 * 
 * @note For VTOL takeoff, use quadplane.do_vtol_takeoff() instead
 * @note Takeoff navigation uses GPS ground course, not waypoint bearing
 * @note Throttle override active until rotation_complete
 * 
 * @see verify_takeoff() for completion checking
 * @see auto_state.takeoff_pitch_cd for pitch target during climb
 * @see auto_state.takeoff_altitude_rel_cm for target altitude
 */
void Plane::do_takeoff(const AP_Mission::Mission_Command& cmd)
{
    prev_WP_loc = current_loc;
    set_next_WP(cmd.content.location);
    // pitch in deg, airspeed  m/s, throttle %, track WP 1 or 0
    auto_state.takeoff_pitch_cd        = (int16_t)cmd.p1 * 100;
    if (auto_state.takeoff_pitch_cd <= 0) {
        // if the mission doesn't specify a pitch use 4 degrees
        auto_state.takeoff_pitch_cd = 400;
    }
    auto_state.takeoff_altitude_rel_cm = next_WP_loc.alt - home.alt;
    next_WP_loc.lat = home.lat + 10;
    next_WP_loc.lng = home.lng + 10;
    auto_state.takeoff_complete = false; // set flag to use gps ground course during TO. IMU will be doing yaw drift correction.
    auto_state.rotation_complete = false;
    auto_state.height_below_takeoff_to_level_off_cm = 0;
    // Flag also used to override "on the ground" throttle disable

    // zero locked course
    steer_state.locked_course_err = 0;
    steer_state.hold_course_cd = -1;
    auto_state.baro_takeoff_alt = barometer.get_altitude();
}

/**
 * @brief Initialize navigation to waypoint command
 * 
 * @details Simplest NAV command - configures aircraft to fly to specified waypoint.
 *          Sets navigation target and lets the L1 controller handle path following.
 *          
 *          Navigation Setup:
 *          - Calls set_next_WP() which configures next_WP_loc
 *          - Preserves previous waypoint for crosstrack calculation
 *          - L1 controller computes lateral guidance
 *          - TECS handles altitude tracking
 *          
 *          Command Parameters (via cmd.p1):
 *          - HIGHBYTE: Pass-by distance in meters (fly past waypoint)
 *          - LOWBYTE: Acceptance radius in meters (custom completion distance)
 *          - See verify_nav_wp() for completion criteria using these values
 *          
 *          Altitude Handling:
 *          - Altitude from cmd.content.location applied by set_next_WP()
 *          - Supports multiple altitude frames (MSL, AGL, terrain-relative)
 *          - Terrain following active if location has terrain altitude
 *          
 *          Typical Usage:
 *          - Standard waypoint navigation in AUTO missions
 *          - Survey patterns and route following
 *          - Approach paths to landing or other operations
 * 
 * @param[in] cmd Mission command with target location in cmd.content.location
 * 
 * @note Completion verified by verify_nav_wp() checking distance and pass-through
 * @note Crosstrack navigation enabled for accurate path following
 * @note Works with standard and terrain-following altitudes
 * 
 * @see verify_nav_wp() for completion criteria
 * @see set_next_WP() for waypoint configuration
 * @see nav_controller->update_waypoint() for path guidance
 */
void Plane::do_nav_wp(const AP_Mission::Mission_Command& cmd)
{
    set_next_WP(cmd.content.location);
}

/**
 * @brief Initialize fixed-wing landing command
 * 
 * @details Configures aircraft for automatic landing approach and touchdown. Sets
 *          landing target, abort parameters, and initializes landing state machine.
 *          Handles transition through pre-flare, flare, and touchdown phases.
 *          
 *          Waypoint Configuration:
 *          - Landing target set via set_next_WP(cmd.content.location)
 *          - Aircraft navigates toward touchdown point
 *          - Final approach path computed by landing library
 *          
 *          Abort Altitude Configuration:
 *          - Altitude for go-around if landing aborted
 *          - Priority: 1) cmd.p1 if >0, 2) last takeoff altitude, 3) 30m default
 *          - Stored in takeoff_altitude_rel_cm (reuses takeoff field)
 *          - Used if pilot or automation triggers abort
 *          
 *          Abort Pitch Configuration:
 *          - Pitch angle for climb during go-around
 *          - Uses takeoff_pitch_cd from previous takeoff
 *          - Defaults to conservative 10 degrees if never set
 *          - Ensures safe climb gradient during abort
 *          
 *          Rangefinder Initialization:
 *          - Clears rangefinder_state to start fresh measurement
 *          - Accumulates altitude samples during approach
 *          - Used for flare timing and touchdown detection
 *          - Critical for accurate height-above-ground measurement
 *          
 *          Landing Library Delegation:
 *          - landing.do_land() handles detailed landing logic
 *          - Configures flare parameters, approach slope
 *          - Manages landing state machine transitions
 *          - Monitors altitude, speed, sink rate
 *          
 *          Abort State Handling:
 *          - If currently in ABORT_LANDING state, transition to LAND
 *          - ABORT_LANDING is "sticky" and must be explicitly cleared
 *          - Allows new landing attempt after aborted landing
 *          
 *          Landing Phases:
 *          1. Approach: Navigate to landing point at glide slope
 *          2. Pre-flare: Setup for flare at configured altitude
 *          3. Flare: Pitch up, reduce descent rate
 *          4. Touchdown: Ground contact detection and disarm
 * 
 * @param[in] cmd Mission command containing landing parameters
 *                cmd.content.location: Landing touchdown point
 *                cmd.p1: Abort altitude in meters (0 = use default)
 * 
 * @note For VTOL landing, use quadplane.do_vtol_land() instead
 * @note Rangefinder critical for accurate flare timing
 * @note Landing can be aborted to configured altitude
 * 
 * @see landing.do_land() for detailed landing state machine
 * @see verify_land() for landing phase monitoring
 * @see landing.verify_land() for completion criteria
 */
void Plane::do_land(const AP_Mission::Mission_Command& cmd)
{
    set_next_WP(cmd.content.location);

    // configure abort altitude and pitch
    // if NAV_LAND has an abort altitude then use it, else use last takeoff, else use 30m
    if (cmd.p1 > 0) {
        auto_state.takeoff_altitude_rel_cm = (int16_t)cmd.p1 * 100;
    } else if (auto_state.takeoff_altitude_rel_cm <= 0) {
        auto_state.takeoff_altitude_rel_cm = 3000;
    }

    if (auto_state.takeoff_pitch_cd <= 0) {
        // If no takeoff command has ever been used, default to a conservative 10deg
        auto_state.takeoff_pitch_cd = 1000;
    }

#if AP_RANGEFINDER_ENABLED
    // zero rangefinder state, start to accumulate good samples now
    memset(&rangefinder_state, 0, sizeof(rangefinder_state));
#endif

    landing.do_land(cmd, relative_altitude);

    if (flight_stage == AP_FixedWing::FlightStage::ABORT_LANDING) {
        // if we were in an abort we need to explicitly move out of the abort state, as it's sticky
        set_flight_stage(AP_FixedWing::FlightStage::LAND);
    }
}

#if HAL_QUADPLANE_ENABLED
/**
 * @brief Initialize fixed-wing approach for VTOL landing
 * 
 * @details Configures quadplane to perform fixed-wing spiral approach before
 *          transitioning to VTOL landing. Used when fw_land_approach_radius
 *          parameter is configured for fixed-wing landing approach.
 *          
 *          Approach Strategy:
 *          - Fly in fixed-wing mode to landing area
 *          - Loiter down to VTOL transition altitude
 *          - Line up into wind for final approach
 *          - Transition to VTOL and land vertically
 *          
 *          Initial Configuration:
 *          - Sanitizes command location with current_loc (validates coordinates)
 *          - Sets navigation target to landing point
 *          - Initializes approach_stage to LOITER_TO_ALT
 *          
 *          Multi-Phase Approach Sequence:
 *          1. LOITER_TO_ALT: Descend in loiter to VTOL transition altitude
 *          2. ENSURE_RADIUS: Position at correct distance from landing point
 *          3. WAIT_FOR_BREAKOUT: Time loiter exit to face into wind
 *          4. APPROACH_LINE: Fly straight approach path to landing point
 *          5. VTOL_LANDING: Transition to VTOL and execute vertical landing
 *          
 *          Wind Compensation:
 *          - Determines approach direction based on wind estimate
 *          - Aligns final approach path into wind
 *          - Minimizes ground speed at VTOL transition
 *          - Improves hover stability and landing accuracy
 *          
 *          Altitude Management:
 *          - Descends to quadplane.qrtl_alt before VTOL transition
 *          - Maintains safe altitude margin throughout approach
 *          - Monitors altitude achievement for phase transitions
 *          
 *          Usage Context:
 *          - Triggered by NAV_VTOL_LAND or NAV_PAYLOAD_PLACE commands
 *          - Requires quadplane.fw_land_approach_radius > 0
 *          - Alternative to direct VTOL descent from mission altitude
 * 
 * @param[in] cmd Mission command with landing target location
 *                cmd.content.location: VTOL landing touchdown point
 * 
 * @note Only compiled with HAL_QUADPLANE_ENABLED
 * @note Requires fw_land_approach_radius parameter configured
 * @note Approach stages verified by verify_landing_vtol_approach()
 * 
 * @see verify_landing_vtol_approach() for phase progression monitoring
 * @see quadplane.fw_land_approach_radius for approach radius configuration
 * @see vtol_approach_s for approach state tracking
 */
void Plane::do_landing_vtol_approach(const AP_Mission::Mission_Command& cmd)
{
    //set target alt
    Location loc = cmd.content.location;
    loc.sanitize(current_loc);
    set_next_WP(loc);

    vtol_approach_s.approach_stage = VTOLApproach::Stage::LOITER_TO_ALT;
}
#endif

/**
 * @brief Set loiter direction from waypoint command
 * 
 * @details Extracts loiter direction (clockwise or counter-clockwise) from mission
 *          command location flags and applies to loiter state. Direction affects
 *          L1 controller guidance and turn coordination.
 *          
 *          Direction Encoding:
 *          - loiter_ccw flag in Location object controls direction
 *          - false (default): Clockwise loiter, direction = 1
 *          - true: Counter-clockwise loiter, direction = -1
 *          
 *          Direction Sign Convention:
 *          - Positive (+1): Right turn / clockwise
 *          - Negative (-1): Left turn / counter-clockwise
 *          - Matches L1 controller convention
 *          - Consistent with loiter_radius sign
 *          
 *          Impact on Navigation:
 *          - L1 controller uses direction for turn calculations
 *          - Affects bank angle commands
 *          - Determines which side of loiter circle to approach
 *          
 *          Usage:
 *          - Called during initialization of loiter commands
 *          - Applied to LOITER_UNLIM, LOITER_TURNS, LOITER_TIME, LOITER_TO_ALT
 *          - Overrides any previous loiter direction
 * 
 * @param[in] cmd Mission command containing location with loiter_ccw flag
 * 
 * @note Direction can also be set by sign of aparm.loiter_radius parameter
 * @note Counter-clockwise preferred in some regions by local regulations
 * 
 * @see loiter.direction for stored direction state
 * @see nav_controller->update_loiter() for direction application
 */
void Plane::loiter_set_direction_wp(const AP_Mission::Mission_Command& cmd)
{
    if (cmd.content.location.loiter_ccw) {
        loiter.direction = -1;
    } else {
        loiter.direction = 1;
    }
}

/**
 * @brief Initialize unlimited duration loiter command
 * 
 * @details Configures aircraft to orbit specified location indefinitely. Loiter
 *          continues until mode change or mission intervention. Used for holding
 *          patterns, station keeping, or awaiting further commands.
 *          
 *          Configuration Steps:
 *          1. Extract location from command
 *          2. Sanitize coordinates (validate lat/lon against current position)
 *          3. Set as navigation target waypoint
 *          4. Apply loiter direction from command flags
 *          
 *          Location Sanitization:
 *          - Validates latitude/longitude are reasonable
 *          - Uses current_loc as reference for validation
 *          - Prevents navigation to invalid coordinates
 *          - Preserves altitude and other location attributes
 *          
 *          Loiter Characteristics:
 *          - No time limit (loiter.time_max_ms = 0)
 *          - No turn count limit (loiter.total_cd = 0)
 *          - Continues indefinitely until external intervention
 *          - Uses standard loiter radius from parameters
 *          
 *          Direction:
 *          - Set from cmd.content.location.loiter_ccw flag
 *          - Or uses sign of aparm.loiter_radius if flag not set
 *          - Determines clockwise vs counter-clockwise orbit
 *          
 *          Completion:
 *          - verify_loiter_unlim() always returns false
 *          - Never completes autonomously
 *          - Requires mode change or mission reset to exit
 *          
 *          Typical Usage:
 *          - Holding patterns for airspace coordination
 *          - Waiting for external event (payload operation, clearance)
 *          - Station keeping over area of interest
 *          - Default behavior when mission completes with no RTL
 * 
 * @param[in] cmd Mission command containing loiter center location
 *                cmd.content.location: Center point of loiter circle
 *                cmd.content.location.loiter_ccw: Direction flag
 * 
 * @note Never completes autonomously - continues until mode change
 * @note Uses standard loiter radius from aparm.loiter_radius
 * @note Location sanitized to prevent invalid coordinates
 * 
 * @see verify_loiter_unlim() for verification (always false)
 * @see loiter_set_direction_wp() for direction configuration
 */
void Plane::do_loiter_unlimited(const AP_Mission::Mission_Command& cmd)
{
    Location cmdloc = cmd.content.location;
    cmdloc.sanitize(current_loc);
    set_next_WP(cmdloc);
    loiter_set_direction_wp(cmd);
}

/**
 * @brief Initialize loiter with specified number of turns command
 * 
 * @details Configures aircraft to complete specified number of orbits around
 *          location before proceeding. Tracks cumulative heading change to
 *          determine turn completion.
 *          
 *          Configuration Steps:
 *          1. Sanitize and set loiter center location
 *          2. Configure loiter direction (CW/CCW)
 *          3. Calculate total heading change target in centidegrees
 *          4. Initialize condition_value for phase tracking
 *          
 *          Turn Counting:
 *          - turns parameter extracted via cmd.get_loiter_turns()
 *          - Converted to centidegrees: turns * 36000
 *          - Example: 2.5 turns = 90000 centidegrees
 *          - Tracks cumulative heading change in loiter.sum_cd
 *          
 *          Radius Parameter:
 *          - Extracted from HIGHBYTE(cmd.p1)
 *          - If type_specific_bits & (1<<0): multiply radius by 10
 *          - Allows larger loiter radii via special encoding
 *          - Applied during verification, not initialization
 *          
 *          Two-Phase Completion:
 *          - Primary goal: Complete specified number of turns
 *          - Secondary goal: Align heading toward next waypoint
 *          - condition_value tracks phase (1=turns, 0=heading)
 *          - Ensures smooth transition to next mission item
 *          
 *          Turn Measurement:
 *          - loiter.sum_cd accumulates heading changes
 *          - Increments regardless of direction
 *          - Must reach loiter target before counting starts
 *          - Accounts for wind drift and position corrections
 *          
 *          Usage Scenarios:
 *          - Thermal soaring (gain altitude in rising air)
 *          - Photography (multiple passes over target)
 *          - Delay with predictable duration
 *          - Coordination with other aircraft
 * 
 * @param[in] cmd Mission command containing loiter parameters
 *                cmd.content.location: Loiter center point
 *                cmd.get_loiter_turns(): Number of turns (supports decimals)
 *                HIGHBYTE(cmd.p1): Optional custom radius
 * 
 * @note Completion requires reaching target AND completing turns
 * @note Heading alignment phase follows turn completion
 * @note VTOL loiter not supported (auto_state.vtol_loiter = false)
 * 
 * @see verify_loiter_turns() for completion tracking
 * @see loiter.sum_cd for cumulative heading change
 * @see loiter.total_cd for target heading change
 */
void Plane::do_loiter_turns(const AP_Mission::Mission_Command& cmd)
{
    Location cmdloc = cmd.content.location;
    cmdloc.sanitize(current_loc);
    set_next_WP(cmdloc);
    loiter_set_direction_wp(cmd);
    const float turns = cmd.get_loiter_turns();

    loiter.total_cd = (uint32_t)(turns * 36000UL);
    condition_value = 1; // used to signify primary turns goal not yet met
}

/**
 * @brief Initialize time-limited loiter command
 * 
 * @details Configures aircraft to orbit location for specified duration. Timer
 *          starts when loiter target is reached, not when command initiated.
 *          Ensures accurate loiter time regardless of approach distance.
 *          
 *          Configuration Steps:
 *          1. Sanitize and set loiter center location
 *          2. Configure loiter direction (CW/CCW)
 *          3. Store loiter duration (seconds to milliseconds)
 *          4. Initialize condition_value for phase tracking
 *          
 *          Timing Behavior:
 *          - loiter.start_time_ms initially 0 (timer not started)
 *          - Timer starts when reached_loiter_target() returns true
 *          - Requires loiter.sum_cd > 1 to prevent false triggers
 *          - Duration from cmd.p1 (seconds), converted to milliseconds
 *          
 *          Two-Phase Completion:
 *          1. Primary: Loiter for specified time duration
 *          2. Secondary: Align heading toward next waypoint
 *          - condition_value = 1 during time phase
 *          - condition_value = 0 during heading alignment
 *          - Smooth transition prevents overshoot of next waypoint
 *          
 *          Target Reaching Detection:
 *          - Must be within loiter radius of target
 *          - Must have completed some heading change (sum_cd > 1)
 *          - Prevents premature timer start during approach
 *          - Accounts for wind drift and position corrections
 *          
 *          Loiter Radius:
 *          - Uses aparm.loiter_radius parameter
 *          - No custom radius support in this command
 *          - Consistent orbit size throughout loiter
 *          
 *          Usage Scenarios:
 *          - Timed observation of area
 *          - Coordination delays in multi-aircraft operations
 *          - Predictable duration holding patterns
 *          - Photography with time-based coverage
 *          - Waiting for payload operation to complete
 * 
 * @param[in] cmd Mission command containing loiter parameters
 *                cmd.content.location: Loiter center point
 *                cmd.p1: Loiter duration in seconds
 *                cmd.content.location.loiter_ccw: Direction flag
 * 
 * @note Timer starts when loiter target reached, not immediately
 * @note Heading alignment phase follows time completion
 * @note Uses standard loiter radius from parameters
 * 
 * @see verify_loiter_time() for timer management and completion
 * @see reached_loiter_target() for target reaching detection
 * @see verify_loiter_heading() for secondary heading alignment
 */
void Plane::do_loiter_time(const AP_Mission::Mission_Command& cmd)
{
    Location cmdloc = cmd.content.location;
    cmdloc.sanitize(current_loc);
    set_next_WP(cmdloc);
    loiter_set_direction_wp(cmd);

    // we set start_time_ms when we reach the waypoint
    loiter.time_max_ms = cmd.p1 * (uint32_t)1000;     // convert sec to ms
    condition_value = 1; // used to signify primary time goal not yet met
}

/**
 * @brief Initialize continue-current-heading and change altitude command
 * 
 * @details Maintains current flight direction while climbing or descending to new
 *          altitude. Used for altitude adjustments without changing lateral path,
 *          common after mode changes or aborted operations.
 *          
 *          Heading Selection Priority (three methods):
 *          
 *          Method 1: Waypoint-Based Bearing (preferred)
 *          - If prev_WP_loc and next_WP_loc differ: Use existing waypoint track
 *          - Most accurate for normal mission sequence
 *          - Maintains established navigation path
 *          - steer_state.hold_course_cd = -1 enables waypoint navigation
 *          
 *          Method 2: GPS Ground Course Bearing
 *          - If waypoints same AND GPS has 2D+ fix
 *          - Uses current ground track from GPS
 *          - Projects 1km waypoint ahead on current course
 *          - Handles post-mode-change scenarios (e.g., aborted landing)
 *          - steer_state.hold_course_cd = -1 uses projected waypoint
 *          
 *          Method 3: AHRS Yaw Bearing (fallback)
 *          - If waypoints same AND no GPS fix
 *          - Uses current aircraft heading from AHRS
 *          - Projects 1km waypoint ahead on current heading
 *          - Least accurate but works without GPS
 *          - steer_state.hold_course_cd holds fixed heading in centidegrees
 *          
 *          Altitude Change Handling:
 *          - ABOVE_TERRAIN frame: Apply terrain-relative altitude directly
 *          - Other frames: Convert to ABSOLUTE frame if possible
 *          - If conversion fails: Maintain current altitude (safety)
 *          - Altitude applied to next_WP_loc for tracking
 *          
 *          Direction Indicator (condition_value from cmd.p1):
 *          - 1: Climbing to target altitude
 *          - 2: Descending to target altitude
 *          - 0: Altitude change direction not specified
 *          - Used by verify function for completion criteria
 *          
 *          Altitude Offset Reset:
 *          - Clears any accumulated altitude offset
 *          - Ensures clean altitude tracking for new target
 *          - Prevents interference from previous altitude adjustments
 *          
 *          Typical Usage:
 *          - Altitude adjustment after aborted landing
 *          - Climb/descend after mode change
 *          - Altitude change without lateral path disruption
 *          - Recovery from off-nominal situations
 * 
 * @param[in] cmd Mission command with altitude change parameters
 *                cmd.content.location: Target altitude (various frames supported)
 *                cmd.p1: Direction indicator (1=climb, 2=descend, 0=either)
 * 
 * @note Heading method automatically selected based on available data
 * @note Terrain-following supported if altitude frame is ABOVE_TERRAIN
 * @note Completion when within 5m of target altitude
 * 
 * @see verify_continue_and_change_alt() for completion criteria
 * @see reset_offset_altitude() for altitude offset clearing
 */
void Plane::do_continue_and_change_alt(const AP_Mission::Mission_Command& cmd)
{
    // select heading method. Either mission, gps bearing projection or yaw based
    // If prev_WP_loc and next_WP_loc are different then an accurate wp based bearing can
    // be computed. However, if we had just changed modes before this, such as an aborted landing
    // via mode change, the prev and next wps are the same.
    float bearing;
    if (!prev_WP_loc.same_latlon_as(next_WP_loc)) {
        // use waypoint based bearing, this is the usual case
        steer_state.hold_course_cd = -1;
    } else if (AP::gps().status() >= AP_GPS::GPS_OK_FIX_2D) {
        // use gps ground course based bearing hold
        steer_state.hold_course_cd = -1;
        bearing = AP::gps().ground_course();
        next_WP_loc.offset_bearing(bearing, 1000); // push it out 1km
    } else {
        // use yaw based bearing hold
        steer_state.hold_course_cd = wrap_360_cd(ahrs.yaw_sensor);
        bearing = ahrs.get_yaw_deg();
        next_WP_loc.offset_bearing(bearing, 1000); // push it out 1km
    }

    if (cmd.content.location.get_alt_frame() == Location::AltFrame::ABOVE_TERRAIN) {
        next_WP_loc.set_alt_cm(cmd.content.location.alt,
                               Location::AltFrame::ABOVE_TERRAIN);
    } else {
        int32_t alt_abs_cm;
        // if this fails we don't change alt
        if (cmd.content.location.get_alt_cm(Location::AltFrame::ABSOLUTE, alt_abs_cm)) {
            next_WP_loc.set_alt_cm(alt_abs_cm,
                                   Location::AltFrame::ABSOLUTE);
        }
    }
    condition_value = cmd.p1;
    reset_offset_altitude();
}

/**
 * @brief Initialize altitude wait command for glider/soaring operations
 * 
 * @details Enters idle mode where control surfaces go to trim positions while
 *          waiting for altitude gain (thermal soaring) or descent rate criteria.
 *          Used in glider operations to minimize drag and allow efficient altitude
 *          gain or waiting for specific sink rate.
 *          
 *          Idle Mode Activation:
 *          - Sets auto_state.idle_mode = true
 *          - All servos commanded to trim positions
 *          - Minimizes control surface drag
 *          - Allows clean aerodynamics for soaring
 *          - Throttle typically at idle or off
 *          
 *          Command Completion Criteria (either):
 *          1. Altitude Target: Reach or exceed specified altitude (AMSL)
 *          2. Descent Rate: Achieve specified sink rate (m/s)
 *          - First criterion met triggers completion
 *          - Altitude in cmd.content.altitude_wait.altitude (AMSL)
 *          - Sink rate in cmd.content.altitude_wait.descent_rate
 *          
 *          Optional Pullup Maneuver:
 *          - If AP_PLANE_GLIDER_PULLUP_ENABLED and configured
 *          - Pullup state reset at command start
 *          - Can perform energy-management pullup before release
 *          - Converts speed to altitude before payload drop
 *          - Extends glide range after release
 *          
 *          Optional Servo Wiggle:
 *          - Periodic servo movement to prevent sticking
 *          - Configured via cmd.content.altitude_wait.wiggle_time
 *          - Only wiggles when not near release altitude
 *          - Prevents servo binding during long waits
 *          - See verify_altitude_wait() for wiggle control
 *          
 *          Typical Usage Scenarios:
 *          - Thermal soaring: Wait for altitude gain
 *          - Glider release: Wait for descent rate before drop
 *          - Energy management: Pullup before payload release
 *          - Towed glider: Wait for release altitude
 *          
 *          Safety Considerations:
 *          - Aircraft not actively controlled in idle mode
 *          - Relies on inherent stability and trim settings
 *          - Monitor altitude loss rate during wait
 *          - Ensure safe minimum altitude margins
 * 
 * @param[in] cmd Mission command with altitude wait parameters
 *                cmd.content.altitude_wait.altitude: Target altitude AMSL (m)
 *                cmd.content.altitude_wait.descent_rate: Target sink rate (m/s)
 *                cmd.content.altitude_wait.wiggle_time: Servo wiggle period (s)
 * 
 * @note Control surfaces at trim - minimal active control
 * @note Completion on EITHER altitude OR descent rate achieved
 * @note Pullup feature requires AP_PLANE_GLIDER_PULLUP_ENABLED
 * 
 * @see verify_altitude_wait() for completion monitoring
 * @see auto_state.idle_mode for servo trim control
 */
void Plane::do_altitude_wait(const AP_Mission::Mission_Command& cmd)
{
    // set all servos to trim until we reach altitude or descent speed
    auto_state.idle_mode = true;
#if AP_PLANE_GLIDER_PULLUP_ENABLED
    mode_auto.pullup.reset();
#endif
}

/**
 * @brief Initialize loiter while climbing/descending to altitude command
 * 
 * @details Configures aircraft to circle at specified location while changing
 *          altitude. Command completes when both altitude target and heading
 *          alignment achieved. Commonly used for altitude gain/loss at specific
 *          location before proceeding to next waypoint.
 *          
 *          Location Setup:
 *          - Sanitizes command location with current_loc (validates coordinates)
 *          - Sets navigation target to loiter center point
 *          - Configures loiter direction (CW/CCW) via loiter_set_direction_wp()
 *          - Loiter radius from cmd.p1 (0 = use aparm.loiter_radius)
 *          
 *          Two-Phase Completion Criteria:
 *          
 *          Phase 1: Altitude Achievement (condition_value = 0)
 *          - Primary goal: Reach target altitude
 *          - Altitude from cmd.content.location (various frames supported)
 *          - Monitors loiter.reached_target_alt flag
 *          - Also checks loiter.unable_to_acheive_target_alt timeout
 *          - Must have loiter.sum_cd > 1 (completed >1° of circle)
 *          - Transitions to Phase 2 when altitude reached
 *          
 *          Phase 2: Heading Alignment (condition_value = 1)
 *          - Secondary goal: Align heading toward next waypoint
 *          - Positions aircraft for smooth transition to next command
 *          - Uses verify_loiter_heading() for alignment check
 *          - Ensures efficient path to next navigation target
 *          
 *          Altitude Monitoring:
 *          - Tracks vertical progress during loiter
 *          - Detects if altitude unachievable (wind, performance limits)
 *          - Sets loiter.unable_to_acheive_target_alt if stuck
 *          - Allows progression if altitude cannot be reached
 *          
 *          Loiter Direction:
 *          - Extracted from cmd.content.location.loiter_ccw flag
 *          - CW (clockwise): loiter.direction = 1
 *          - CCW (counter-clockwise): loiter.direction = -1
 *          - Matches loiter_radius sign convention
 *          
 *          Condition Value State Machine:
 *          - 0: Waiting for altitude achievement (initial state)
 *          - 1: Altitude reached, waiting for heading alignment
 *          - State tracked by verify_loiter_to_alt()
 *          
 *          Typical Usage:
 *          - Gain altitude at specific location before long transit
 *          - Descend to approach altitude before landing pattern
 *          - Position aircraft at correct altitude and heading
 *          - Thermal soaring altitude gain at specific location
 * 
 * @param[in] cmd Mission command with loiter and altitude parameters
 *                cmd.content.location: Loiter center point with target altitude
 *                cmd.p1: Loiter radius in meters (0 = use parameter)
 *                loiter_ccw flag in location determines direction
 * 
 * @note Command completes when BOTH altitude AND heading criteria met
 * @note Altitude failure timeout allows progression if unreachable
 * @note Heading alignment ensures smooth transition to next waypoint
 * 
 * @see verify_loiter_to_alt() for completion monitoring
 * @see loiter_set_direction_wp() for direction configuration
 * @see update_loiter() for navigation control
 */
void Plane::do_loiter_to_alt(const AP_Mission::Mission_Command& cmd)
{
    //set target alt  
    Location loc = cmd.content.location;
    loc.sanitize(current_loc);
    set_next_WP(loc);
    loiter_set_direction_wp(cmd);

    // init to 0, set to 1 when altitude is reached
    condition_value = 0;
}

/**
 * @brief Initialize navigation delay command
 * 
 * @details Pauses mission progression for specified duration or until specific UTC time.
 *          Allows timed coordination with external events, optimal lighting conditions,
 *          or synchronized operations. Aircraft maintains position during delay.
 *          
 *          Two Delay Modes:
 *          
 *          Relative Delay Mode (cmd.content.nav_delay.seconds > 0):
 *          - Delays for specified number of seconds
 *          - nav_delay.time_max_ms = seconds * 1000 (convert to milliseconds)
 *          - Countdown starts from command initiation
 *          - Simple time-based delay
 *          
 *          Absolute UTC Time Mode (cmd.content.nav_delay.seconds <= 0):
 *          - Delays until specific UTC time reached
 *          - Requires AP_RTC_ENABLED and valid RTC
 *          - Time specified in hour_utc, min_utc, sec_utc
 *          - Useful for dawn/dusk coordination, scheduled operations
 *          - If RTC not available: nav_delay.time_max_ms = 0 (immediate)
 *          
 *          Time Tracking:
 *          - nav_delay.time_start_ms captures millis() at command start
 *          - nav_delay.time_max_ms stores delay duration or UTC target
 *          - verify_nav_delay() compares elapsed time against target
 *          
 *          Safety Override:
 *          - If aircraft armed and safety off: Delay skipped
 *          - Prevents delays during active flight operations
 *          - Ensures nav controller keeps running when armed
 *          - See verify_nav_delay() for safety check
 *          
 *          Aircraft Behavior During Delay:
 *          - Holds current position/altitude
 *          - Loiter pattern if appropriate for mode
 *          - All systems remain active
 *          - GCS receives delay status message
 *          
 *          GCS Notification:
 *          - Sends MAV_SEVERITY_INFO message with delay duration
 *          - Helps operator understand mission pause
 *          - Duration displayed in seconds
 *          
 *          Typical Usage:
 *          - Wait for optimal lighting (photography missions)
 *          - Coordinate with ground operations
 *          - Timed payload operations
 *          - Synchronize with external events
 *          - Dawn/dusk mission timing
 * 
 * @param[in] cmd Mission command with delay parameters
 *                cmd.content.nav_delay.seconds: Relative delay (>0) or UTC mode (<=0)
 *                cmd.content.nav_delay.hour_utc: Target hour UTC (absolute mode)
 *                cmd.content.nav_delay.min_utc: Target minute UTC (absolute mode)
 *                cmd.content.nav_delay.sec_utc: Target second UTC (absolute mode)
 * 
 * @note Delay skipped if armed with safety off (safety override)
 * @note Absolute time mode requires AP_RTC_ENABLED
 * @note Aircraft maintains position during delay
 * 
 * @see verify_nav_delay() for completion monitoring and safety checks
 * @see AP::rtc().get_time_utc() for UTC time conversion
 */
void ModeAuto::do_nav_delay(const AP_Mission::Mission_Command& cmd)
{
    nav_delay.time_start_ms = millis();

    if (cmd.content.nav_delay.seconds > 0) {
        // relative delay
        nav_delay.time_max_ms = cmd.content.nav_delay.seconds * 1000; // convert seconds to milliseconds
    } else {
        // absolute delay to utc time
#if AP_RTC_ENABLED
        nav_delay.time_max_ms = AP::rtc().get_time_utc(cmd.content.nav_delay.hour_utc, cmd.content.nav_delay.min_utc, cmd.content.nav_delay.sec_utc, 0);
#else
        nav_delay.time_max_ms = 0;
#endif
    }
    gcs().send_text(MAV_SEVERITY_INFO, "Delaying %u sec", (unsigned)(nav_delay.time_max_ms/1000));
}

/********************************************************************************/
//  Verify Nav (Must) commands
/********************************************************************************/

/**
 * @brief Verify takeoff command completion by checking altitude achievement
 * 
 * @details Monitors the takeoff phase and determines when the aircraft has successfully
 *          climbed to the target altitude. The takeoff sequence involves:
 *          
 *          Phase 1 - Ground Roll (hold_course_cd == -1):
 *          - Wait for GPS speed > GPS_GND_CRS_MIN_SPD
 *          - Capture ground course corrected for yaw drift
 *          - Lock heading hold for directional control
 *          
 *          Phase 2 - Rotation and Climb (hold_course_cd set):
 *          - Maintain locked heading during initial climb
 *          - Apply takeoff pitch angle (auto_state.takeoff_pitch_cd)
 *          - Monitor altitude gain
 *          
 *          Phase 3 - Completion:
 *          - Altitude reaches auto_state.takeoff_altitude_rel_cm
 *          - OR timeout occurs (check_takeoff_timeout_level_off)
 *          - Release heading hold, enable fence, disable crosstrack
 *          
 *          Navigation Control:
 *          - Before heading locked: update_level_flight()
 *          - After heading locked: update_heading_hold(steer_state.hold_course_cd)
 *          
 *          Safety Features:
 *          - Checks for optional takeoff timeout to prevent stuck state
 *          - Requires AHRS initialization before capturing heading
 *          - Requires safety switch armed before locking course
 *          - Automatically enables geofence after takeoff completion
 * 
 * @return true if takeoff altitude reached or timeout occurred
 * @return false if still climbing to target altitude
 * 
 * @note Called repeatedly at scheduler rate during TAKEOFF command
 * @note Updates nav_controller for heading/level control
 * @note Sets auto_state.takeoff_complete flag on completion
 * 
 * @see do_takeoff() for takeoff command initialization
 * @see auto_state.takeoff_altitude_rel_cm for target altitude
 * @see steer_state.hold_course_cd for locked heading (-1 = not locked)
 */
bool Plane::verify_takeoff()
{
    bool trust_ahrs_yaw = AP::ahrs().initialised();
#if AP_AHRS_DCM_ENABLED
    trust_ahrs_yaw |= ahrs.dcm_yaw_initialised();
#endif
    if (trust_ahrs_yaw && steer_state.hold_course_cd == -1) {
        // once we reach sufficient speed for good GPS course
        // estimation we save our current GPS ground course
        // corrected for summed yaw to set the take off
        // course. This keeps wings level until we are ready to
        // rotate, and also allows us to cope with arbitrary
        // compass errors for auto takeoff
        if (gps.status() >= AP_GPS::GPS_OK_FIX_3D && 
            gps.ground_speed() > GPS_GND_CRS_MIN_SPD &&
            hal.util->safety_switch_state() != AP_HAL::Util::SAFETY_DISARMED) {
            float takeoff_course = wrap_PI(radians(gps.ground_course())) - steer_state.locked_course_err;
            takeoff_course = wrap_PI(takeoff_course);
            steer_state.hold_course_cd = wrap_360_cd(degrees(takeoff_course)*100);
            gcs().send_text(MAV_SEVERITY_INFO, "Holding course %d at %.1fm/s (%.1f)",
                              (int)steer_state.hold_course_cd,
                              (double)gps.ground_speed(),
                              (double)degrees(steer_state.locked_course_err));
        }
    }

    if (steer_state.hold_course_cd != -1) {
        // call navigation controller for heading hold
        nav_controller->update_heading_hold(steer_state.hold_course_cd);
    } else {
        nav_controller->update_level_flight();        
    }

    // check for optional takeoff timeout
    if (plane.check_takeoff_timeout()) {
        mission.reset();
    }

    // see if we have reached takeoff altitude
    int32_t relative_alt_cm = adjusted_relative_altitude_cm();
    if (
        relative_alt_cm > auto_state.takeoff_altitude_rel_cm || // altitude reached
        plane.check_takeoff_timeout_level_off() // pitch level-off maneuver has timed out
        ) {
        gcs().send_text(MAV_SEVERITY_INFO, "Takeoff complete at %.2fm",
                          (double)(relative_alt_cm*0.01f));
        steer_state.hold_course_cd = -1;
        auto_state.takeoff_complete = true;
        next_WP_loc = prev_WP_loc = current_loc;

#if AP_FENCE_ENABLED
        plane.fence.auto_enable_fence_after_takeoff();
#endif

        // don't cross-track on completion of takeoff, as otherwise we
        // can end up doing too sharp a turn
        auto_state.next_wp_crosstrack = false;
        return true;
    } else {
        return false;
    }
}

/**
 * @brief Verify waypoint arrival by checking distance and position criteria
 * 
 * @details Determines when a navigation waypoint has been reached using multiple
 *          completion criteria. Handles both standard waypoint arrival and special
 *          "pass-by" waypoint behavior where the aircraft flies past the waypoint
 *          for a specified distance.
 *          
 *          Waypoint Completion Criteria (evaluated in order):
 *          
 *          1. Maximum Radius Override (g.waypoint_max_radius > 0):
 *             - If distance > max_radius AND past finish line, complete
 *             - Prevents indefinite pursuit of distant waypoints
 *          
 *          2. Acceptance Distance (within acceptance_distance_m):
 *             - Standard: turn_distance based on turn radius and next turn angle
 *             - User override: cmd.p1 low byte specifies custom radius
 *             - Pass-by mode: acceptance at waypoint location (0m)
 *          
 *          3. Finish Line Crossed:
 *             - Uses past_interval_finish_line() to detect overflight
 *             - Prevents circling back to missed waypoints
 *          
 *          Pass-By Waypoint Mode (cmd.p1 high byte > 0):
 *          - Projects waypoint forward along track by pass-by distance
 *          - Aircraft flies past original waypoint position
 *          - Useful for maintaining speed through waypoint
 *          
 *          Navigation Controller Updates:
 *          - If crosstrack enabled: update_waypoint(prev_WP, flex_next_WP)
 *          - If crosstrack disabled: update_waypoint(current, flex_next_WP)
 *          
 *          Command Parameter Encoding (cmd.p1):
 *          - Low byte (LOWBYTE): Acceptance distance override in meters
 *          - High byte (HIGHBYTE): Pass-by distance in meters
 * 
 * @param[in] cmd Waypoint mission command with location and parameters
 * 
 * @return true if waypoint reached via any completion criterion
 * @return false if aircraft still en route to waypoint
 * 
 * @note Called repeatedly at scheduler rate during WAYPOINT command
 * @note Updates nav_controller with waypoint pair for crosstrack calculation
 * @note Sends GCS message on completion with distance and waypoint index
 * 
 * @see do_nav_wp() for waypoint command initialization
 * @see nav_controller->update_waypoint() for crosstrack navigation
 * @see auto_state.crosstrack flag for crosstrack enable/disable
 * @see g.waypoint_max_radius for maximum pursuit distance parameter
 */
bool Plane::verify_nav_wp(const AP_Mission::Mission_Command& cmd)
{
    steer_state.hold_course_cd = -1;

    // depending on the pass by flag either go to waypoint in regular manner or
    // fly past it for set distance along the line of waypoints
    Location flex_next_WP_loc = next_WP_loc;

    uint8_t cmd_passby = HIGHBYTE(cmd.p1); // distance in meters to pass beyond the wp
    uint8_t cmd_acceptance_distance = LOWBYTE(cmd.p1); // radius in meters to accept reaching the wp

    if (cmd_passby > 0) {
        const float dist = prev_WP_loc.get_distance(flex_next_WP_loc);
        const float bearing_deg = degrees(prev_WP_loc.get_bearing(flex_next_WP_loc));

        if (is_positive(dist)) {
            flex_next_WP_loc.offset_bearing(bearing_deg, cmd_passby);
        }
    }

    if (auto_state.crosstrack) {
        nav_controller->update_waypoint(prev_WP_loc, flex_next_WP_loc);
    } else {
        nav_controller->update_waypoint(current_loc, flex_next_WP_loc);
    }

    // see if the user has specified a maximum distance to waypoint
    // If override with p3 - then this is not used as it will overfly badly
    if (g.waypoint_max_radius > 0 &&
        auto_state.wp_distance > (uint16_t)g.waypoint_max_radius) {
        if (current_loc.past_interval_finish_line(prev_WP_loc, flex_next_WP_loc)) {
            // this is needed to ensure completion of the waypoint
            if (cmd_passby == 0) {
                prev_WP_loc = current_loc;
            }
        }
        return false;
    }

    float acceptance_distance_m = 0; // default to: if overflown - let it fly up to the point
    if (cmd_acceptance_distance > 0) {
        // allow user to override acceptance radius
        acceptance_distance_m = cmd_acceptance_distance;
    } else if (cmd_passby == 0) {
        acceptance_distance_m = nav_controller->turn_distance(get_wp_radius(), auto_state.next_turn_angle);
    }
    const float wp_dist = current_loc.get_distance(flex_next_WP_loc);
    if (wp_dist <= acceptance_distance_m) {
        gcs().send_text(MAV_SEVERITY_INFO, "Reached waypoint #%i dist %um",
                          (unsigned)mission.get_current_nav_cmd().index,
                          (unsigned)current_loc.get_distance(flex_next_WP_loc));
        return true;
	}

    // have we flown past the waypoint?
    if (current_loc.past_interval_finish_line(prev_WP_loc, flex_next_WP_loc)) {
        gcs().send_text(MAV_SEVERITY_INFO, "Passed waypoint #%i dist %um",
                          (unsigned)mission.get_current_nav_cmd().index,
                          (unsigned)current_loc.get_distance(flex_next_WP_loc));
        return true;
    }

    return false;
}

/**
 * @brief Verify unlimited loiter command (never completes)
 * 
 * @details Maintains indefinite circular loitering at the commanded location.
 *          This command does not have completion criteria and will continue
 *          until manually interrupted by mode change or mission reset.
 *          
 *          Updates navigation controller with loiter center and radius each cycle
 *          to maintain the circular flight pattern.
 *          
 *          Loiter radius priority:
 *          - cmd.p1 if specified (mission-specific radius)
 *          - aparm.loiter_radius otherwise (default parameter)
 * 
 * @param[in] cmd Loiter command with optional radius in p1 parameter
 * 
 * @return false always (unlimited loiter never completes)
 * 
 * @note Called repeatedly at scheduler rate during LOITER_UNLIM command
 * @note Typically used for holding patterns or aerial survey orbits
 * 
 * @see update_loiter() for navigation controller updates
 * @see verify_loiter_time() for time-limited loiter
 * @see verify_loiter_turns() for turn-limited loiter
 */
bool Plane::verify_loiter_unlim(const AP_Mission::Mission_Command &cmd)
{
    // else use mission radius
    update_loiter(cmd.p1);
    return false;
}

/**
 * @brief Verify time-limited loiter command with two-phase completion
 * 
 * @details Monitors a timed loiter operation with primary and secondary goals:
 *          
 *          Phase 1 - Loiter Time (condition_value == 1):
 *          - Wait for aircraft to reach loiter target (reached_loiter_target)
 *          - Start timer when target reached and loiter.sum_cd > 1
 *          - Monitor elapsed time against loiter.time_max_ms
 *          - When time expires, transition to Phase 2
 *          
 *          Phase 2 - Heading Alignment (condition_value == 0):
 *          - Align aircraft heading with next waypoint bearing
 *          - Uses verify_loiter_heading() to check alignment
 *          - Completes when heading lined up with next mission leg
 *          
 *          Timer Behavior:
 *          - loiter.start_time_ms initially 0 (timer not started)
 *          - Set to millis() when reaching loiter circle
 *          - Requires loiter.sum_cd > 1 to ensure valid loiter
 *          
 *          Navigation:
 *          - Always uses aparm.loiter_radius (cmd radius = 0)
 *          - Continuously updates loiter navigation controller
 * 
 * @return true when both time elapsed AND heading aligned with next waypoint
 * @return false while waiting for time or heading alignment
 * 
 * @note Called repeatedly at scheduler rate during LOITER_TIME command
 * @note loiter.time_max_ms set by do_loiter_time() from cmd.p1 (seconds)
 * @note Clears auto_state.vtol_loiter flag on completion
 * @note Sends "Loiter time complete" GCS message on completion
 * 
 * @see do_loiter_time() for command initialization
 * @see verify_loiter_heading() for heading alignment verification
 * @see reached_loiter_target() for loiter circle arrival detection
 * @see loiter.start_time_ms for timer state
 * @see condition_value for phase tracking (1=time phase, 0=heading phase)
 */
bool Plane::verify_loiter_time()
{
    bool result = false;
    // mission radius is always aparm.loiter_radius
    update_loiter(0);

    if (loiter.start_time_ms == 0) {
        if (reached_loiter_target() && loiter.sum_cd > 1) {
            // we've reached the target, start the timer
            loiter.start_time_ms = millis();
        }
    } else if (condition_value != 0) {
        // primary goal, loiter time
        if ((millis() - loiter.start_time_ms) > loiter.time_max_ms) {
            // primary goal completed, initialize secondary heading goal
            condition_value = 0;
            result = verify_loiter_heading(true);
        }
    } else {
        // secondary goal, loiter to heading
        result = verify_loiter_heading(false);
    }

    if (result) {
        gcs().send_text(MAV_SEVERITY_INFO,"Loiter time complete");
        auto_state.vtol_loiter = false;
    }
    return result;
}

/**
 * @brief Verify turn-counted loiter command with two-phase completion
 * 
 * @details Monitors a loiter operation that completes after a specified number of
 *          360-degree turns, followed by heading alignment to the next waypoint.
 *          
 *          Phase 1 - Loiter Turns (condition_value == 1):
 *          - Wait for aircraft to reach loiter target
 *          - Accumulate turn angle in loiter.sum_cd (centidegrees)
 *          - Compare against loiter.total_cd (target turns * 36000)
 *          - Requires loiter.sum_cd > 1 to ensure valid loiter started
 *          - When turn count reached, transition to Phase 2
 *          
 *          Phase 2 - Heading Alignment (condition_value == 0):
 *          - Align aircraft heading with next waypoint bearing
 *          - Uses verify_loiter_heading() to check alignment
 *          - Completes when heading lined up with next mission leg
 *          
 *          Radius Encoding in cmd.p1:
 *          - High byte (HIGHBYTE): Base radius value
 *          - If type_specific_bits[0] set: radius *= 10 for larger values
 *          - Allows encoding radii up to 2550 meters
 *          
 *          Turn Counting:
 *          - loiter.sum_cd accumulates signed angle (CW or CCW)
 *          - Direction set by loiter.direction (+1 or -1)
 *          - loiter.total_cd set by do_loiter_turns() from turn count
 * 
 * @param[in] cmd Loiter turns command with encoded radius and turn count
 * 
 * @return true when both turn count reached AND heading aligned
 * @return false while completing turns or aligning heading
 * 
 * @note Called repeatedly at scheduler rate during LOITER_TURNS command
 * @note VTOL loiter disabled (auto_state.vtol_loiter = false)
 * @note Sends "Loiter orbits complete" GCS message on completion
 * 
 * @see do_loiter_turns() for command initialization and total_cd calculation
 * @see verify_loiter_heading() for heading alignment verification
 * @see reached_loiter_target() for loiter circle arrival detection
 * @see loiter.sum_cd for accumulated turn angle
 * @see condition_value for phase tracking (1=turns phase, 0=heading phase)
 */
bool Plane::verify_loiter_turns(const AP_Mission::Mission_Command &cmd)
{
    bool result = false;
    uint16_t radius = HIGHBYTE(cmd.p1);
    if (cmd.type_specific_bits & (1U<<0)) {
        // special storage handling allows for larger radii
        radius *= 10;
    }
    update_loiter(radius);

    // LOITER_TURNS makes no sense as VTOL
    auto_state.vtol_loiter = false;

    if (!reached_loiter_target()) {
        result = false;
    } else if (condition_value != 0) {
        // primary goal, loiter turns
        if (loiter.sum_cd > loiter.total_cd && loiter.sum_cd > 1) {
            // primary goal completed, initialize secondary heading goal
            condition_value = 0;
            result = verify_loiter_heading(true);
        }
    } else {
        // secondary goal, loiter to heading
        result = verify_loiter_heading(false);
    }

    if (result) {
        gcs().send_text(MAV_SEVERITY_INFO,"Loiter orbits complete");
    }
    return result;
}

/**
 * @brief Verify loiter-to-altitude command with two-phase completion
 * 
 * @details Monitors a loiter operation that continues until reaching a target altitude,
 *          then aligns heading with the next waypoint before completing. Unlike timed
 *          or turn-counted loiters, this command is altitude-driven.
 *          
 *          Phase 1 - Climb/Descend to Altitude (condition_value == 0):
 *          - Loiter while climbing or descending to target altitude
 *          - Monitor loiter.reached_target_alt flag from altitude controller
 *          - Also monitor loiter.unable_to_acheive_target_alt for stuck condition
 *          - Requires loiter.sum_cd > 1 to ensure loiter pattern established
 *          - When altitude reached (or stuck), transition to Phase 2
 *          
 *          Phase 2 - Heading Alignment (condition_value == 1):
 *          - Align aircraft heading with next waypoint bearing
 *          - Uses verify_loiter_heading() to check alignment
 *          - Completes when heading lined up with next mission leg
 *          
 *          Stuck Altitude Handling:
 *          - If unable_to_acheive_target_alt becomes true, proceed anyway
 *          - Sends GCS warning message with current altitude
 *          - Prevents mission stall due to wind or performance limits
 *          
 *          Loiter Radius:
 *          - Uses cmd.p1 if specified (mission-specific radius)
 *          - Uses aparm.loiter_radius if cmd.p1 == 0
 *          
 *          The desired altitude only needs to be reached once - if the aircraft
 *          drifts away from target altitude during heading alignment, the command
 *          will still complete.
 * 
 * @param[in] cmd Loiter to altitude command with target location and radius
 * 
 * @return true when both altitude reached AND heading aligned
 * @return false while climbing/descending or aligning heading
 * 
 * @note Called repeatedly at scheduler rate during LOITER_TO_ALT command
 * @note Sends "Loiter to alt complete" GCS message on completion
 * @note Sends warning if altitude unreachable but continuing
 * 
 * @see do_loiter_to_alt() for command initialization
 * @see verify_loiter_heading() for heading alignment verification
 * @see loiter.reached_target_alt flag set by altitude controller
 * @see loiter.unable_to_acheive_target_alt for stuck detection
 * @see condition_value for phase tracking (0=altitude phase, 1=heading phase)
 */
bool Plane::verify_loiter_to_alt(const AP_Mission::Mission_Command &cmd)
{
    bool result = false;

    update_loiter(cmd.p1);

    // condition_value == 0 means alt has never been reached
    if (condition_value == 0) {
        // primary goal, loiter to alt
        if (labs(loiter.sum_cd) > 1 && (loiter.reached_target_alt || loiter.unable_to_acheive_target_alt)) {
            // primary goal completed, initialize secondary heading goal
            if (loiter.unable_to_acheive_target_alt) {
                gcs().send_text(MAV_SEVERITY_INFO,"Loiter to alt was stuck at %d", int(current_loc.alt/100));
            }

            condition_value = 1;
            result = verify_loiter_heading(true);
        }
    } else {
        // secondary goal, loiter to heading
        result = verify_loiter_heading(false);
    }

    if (result) {
        gcs().send_text(MAV_SEVERITY_INFO,"Loiter to alt complete");
    }
    return result;
}


bool Plane::verify_continue_and_change_alt()
{
    // is waypoint info not available and heading hold is?
    if (prev_WP_loc.same_latlon_as(next_WP_loc) &&
        steer_state.hold_course_cd != -1) {
        //keep flying the same course with fixed steering heading computed at start if cmd
        nav_controller->update_heading_hold(steer_state.hold_course_cd);
    }
    else {
        // Is the next_WP less than 200 m away?
        if (current_loc.get_distance(next_WP_loc) < 200.0f) {
            //push another 300 m down the line
            int32_t next_wp_bearing_cd = prev_WP_loc.get_bearing_to(next_WP_loc);
            next_WP_loc.offset_bearing(next_wp_bearing_cd * 0.01f, 300.0f);
        }

        //keep flying the same course
        nav_controller->update_waypoint(prev_WP_loc, next_WP_loc);
    }

    //climbing?
    if (condition_value == 1 && adjusted_altitude_cm() >= next_WP_loc.alt) {
        return true;
    }
    //descending?
    else if (condition_value == 2 &&
             adjusted_altitude_cm() <= next_WP_loc.alt) {
        return true;
    }    
    //don't care if we're climbing or descending
    else if (labs(adjusted_altitude_cm() - next_WP_loc.alt) <= 500) {
        return true;
    }

    return false;
}

/*
  see if we have reached altitude or descent speed
 */
bool ModeAuto::verify_altitude_wait(const AP_Mission::Mission_Command &cmd)
{
#if AP_PLANE_GLIDER_PULLUP_ENABLED
    if (pullup.in_pullup()) {
        return pullup.verify_pullup();
    }
#endif

    /*
      the target altitude in param1 is always AMSL
     */
    const float alt_diff = plane.current_loc.alt*0.01 - cmd.content.altitude_wait.altitude;
    bool completed = false;
    if (alt_diff > 0) {
        gcs().send_text(MAV_SEVERITY_INFO,"Reached altitude");
        completed = true;
    } else if (cmd.content.altitude_wait.descent_rate > 0 &&
        plane.auto_state.sink_rate > cmd.content.altitude_wait.descent_rate) {
        gcs().send_text(MAV_SEVERITY_INFO, "Reached descent rate %.1f m/s", (double)plane.auto_state.sink_rate);
        completed = true;
    }

    if (completed) {
#if AP_PLANE_GLIDER_PULLUP_ENABLED
        if (pullup.pullup_start()) {
            // we are doing a pullup, ALTITUDE_WAIT not complete until pullup is done
            return false;
        }
#endif
        return true;
    }

    const float time_to_alt = alt_diff / MIN(plane.auto_state.sink_rate, -0.01);

    /*
      if requested, wiggle servos

      we don't start a wiggle if we expect to release soon as we don't
      want the servos to be off trim at the time of release
    */
    if (cmd.content.altitude_wait.wiggle_time != 0 &&
        (plane.auto_state.sink_rate > 0 || time_to_alt > cmd.content.altitude_wait.wiggle_time*5)) {
        if (wiggle.stage == 0 &&
            AP_HAL::millis() - wiggle.last_ms > cmd.content.altitude_wait.wiggle_time*1000) {
            wiggle.stage = 1;
            wiggle.last_ms = AP_HAL::millis();
            // idle_wiggle_stage is updated in wiggle_servos()
        }
    }

    return false;
}

// verify_nav_delay - check if we have waited long enough
bool ModeAuto::verify_nav_delay(const AP_Mission::Mission_Command& cmd)
{
    if (AP::arming().is_armed_and_safety_off()) {
        // don't delay while armed, we need a nav controller running
        return true;
    }
    if (millis() - nav_delay.time_start_ms > nav_delay.time_max_ms) {
        nav_delay.time_max_ms = 0;
        return true;
    }
    return false;
}

/********************************************************************************/
//  Condition (May) commands
/********************************************************************************/

/**
 * @brief Initialize time delay condition command
 * 
 * @details Sets up a CONDITION_DELAY that gates subsequent DO commands within the
 *          same mission item. The delay is non-blocking and allows navigation to
 *          continue while waiting.
 *          
 *          Initialization:
 *          - Records current time in condition_start
 *          - Converts delay from seconds to milliseconds
 *          - Stores duration in condition_value
 *          
 *          Typical Usage:
 *          - Delay camera trigger after waypoint
 *          - Wait before deploying payload
 *          - Coordinate multiple timed actions
 * 
 * @param[in] cmd Mission command with delay duration in cmd.content.delay.seconds
 * 
 * @note Called once when CONDITION_DELAY command starts
 * @note verify_wait_delay() monitors completion
 * @note Does not block mission execution or navigation
 * 
 * @see verify_wait_delay() for completion checking
 * @see condition_start timestamp for delay start
 * @see condition_value for delay duration in milliseconds
 */
void Plane::do_wait_delay(const AP_Mission::Mission_Command& cmd)
{
    condition_start = millis();
    condition_value  = cmd.content.delay.seconds * 1000;    // convert seconds to milliseconds
}

/**
 * @brief Initialize distance-to-waypoint condition command
 * 
 * @details Sets up a CONDITION_DISTANCE that gates subsequent DO commands until
 *          the aircraft is within specified distance of the current waypoint.
 *          
 *          Stores target distance threshold in condition_value where it will be
 *          continuously compared against auto_state.wp_distance.
 *          
 *          Typical Usage:
 *          - Trigger camera at specific distance before waypoint
 *          - Deploy landing gear when approaching runway
 *          - Activate payload systems near target
 * 
 * @param[in] cmd Mission command with distance in cmd.content.distance.meters
 * 
 * @note Called once when CONDITION_DISTANCE command starts
 * @note verify_within_distance() monitors completion
 * @note Distance continuously updated by navigation controller
 * 
 * @see verify_within_distance() for completion checking
 * @see auto_state.wp_distance for current distance to waypoint
 * @see condition_value for distance threshold in meters
 */
void Plane::do_within_distance(const AP_Mission::Mission_Command& cmd)
{
    condition_value  = cmd.content.distance.meters;
}

/********************************************************************************/
// Verify Condition (May) commands
/********************************************************************************/

/**
 * @brief Verify time delay condition has elapsed
 * 
 * @details Checks if a CONDITION_DELAY has completed by comparing elapsed time
 *          against the specified delay duration. This is a "may" command that
 *          controls when subsequent mission commands in the same mission item execute.
 *          
 *          Timing:
 *          - condition_start: Set by do_wait_delay() to millis() when delay begins
 *          - condition_value: Delay duration in milliseconds
 *          - Uses unsigned arithmetic to handle millis() rollover safely
 *          
 *          Completion Behavior:
 *          - Clears condition_value to 0 when delay complete
 *          - Returns true to allow mission progression
 * 
 * @return true if delay duration has elapsed
 * @return false if still waiting for delay completion
 * 
 * @note Called repeatedly at scheduler rate during CONDITION_DELAY
 * @note Non-blocking delay - does not pause mission execution
 * 
 * @see do_wait_delay() for delay initialization from cmd.content.delay.seconds
 * @see condition_start for delay start timestamp (milliseconds)
 * @see condition_value for delay duration (milliseconds)
 */
bool Plane::verify_wait_delay()
{
    if ((unsigned)(millis() - condition_start) > (unsigned)condition_value) {
        condition_value         = 0;
        return true;
    }
    return false;
}

/**
 * @brief Verify distance-to-waypoint condition has been met
 * 
 * @details Checks if aircraft is within specified distance of the current waypoint.
 *          This is a "may" command that gates execution of subsequent DO commands
 *          within the same mission item until the distance criterion is satisfied.
 *          
 *          Distance Comparison:
 *          - auto_state.wp_distance: Continuously updated distance to next_WP_loc
 *          - condition_value: Target distance in meters (from cmd.content.distance.meters)
 *          - Uses MAX(condition_value, 0) to prevent negative distance comparison
 *          
 *          Typical Usage:
 *          - Trigger camera at specific distance before waypoint
 *          - Deploy payload when within approach distance
 *          - Activate landing gear at defined distance from runway
 *          
 *          Completion Behavior:
 *          - Clears condition_value to 0 when distance reached
 *          - Returns true to enable gated DO commands
 * 
 * @return true if within condition_value meters of waypoint
 * @return false if still beyond target distance
 * 
 * @note Called repeatedly at scheduler rate during CONDITION_DISTANCE
 * @note Distance continuously monitored as aircraft approaches waypoint
 * 
 * @see do_within_distance() for condition initialization
 * @see auto_state.wp_distance for current distance to waypoint
 * @see condition_value for target distance threshold (meters)
 */
bool Plane::verify_within_distance()
{
    if (auto_state.wp_distance < MAX(condition_value,0)) {
        condition_value = 0;
        return true;
    }
    return false;
}

/********************************************************************************/
//  Do (Now) commands
/********************************************************************************/

/**
 * @brief Initiate loiter at current location
 * 
 * @details Configures the aircraft to begin loitering at its current position.
 *          This is typically called when switching to loiter mode or when a
 *          mission command requires immediate loitering behavior.
 *          
 *          Configuration Steps:
 *          1. Set loiter direction based on aparm.loiter_radius sign:
 *             - Positive radius: Clockwise (direction = 1)
 *             - Negative radius: Counter-clockwise (direction = -1)
 *          2. Set loiter center to current aircraft position
 *          
 *          Navigation Integration:
 *          - next_WP_loc becomes loiter center point
 *          - Navigation controller will maintain circular path around this point
 *          - Radius magnitude determined by aparm.loiter_radius parameter
 * 
 * @note Called when immediate loiter is required at current position
 * @note Does not set loiter altitude - uses current altitude
 * @note Loiter radius and direction from aparm.loiter_radius parameter
 * 
 * @see aparm.loiter_radius for loiter radius magnitude and direction sign
 * @see loiter.direction for turn direction (-1=CCW, 1=CW)
 * @see next_WP_loc for loiter center coordinates
 */
void Plane::do_loiter_at_location()
{
    if (aparm.loiter_radius < 0) {
        loiter.direction = -1;
    } else {
        loiter.direction = 1;
    }
    next_WP_loc = current_loc;
}

/**
 * @brief Execute DO_CHANGE_SPEED mission command
 * 
 * @details Wrapper function that extracts speed change parameters from a mission
 *          command and delegates to the core implementation. This allows mission
 *          commands to modify airspeed, groundspeed, or throttle settings.
 *          
 *          Delegates to overloaded do_change_speed() with extracted parameters:
 *          - speed_type: AIRSPEED or GROUNDSPEED
 *          - target_ms: Target speed in meters/second
 *          - throttle_pct: Throttle percentage (0-100)
 * 
 * @param[in] cmd Mission command containing speed change parameters
 * 
 * @return true if speed change successfully applied
 * @return false if parameters invalid or out of range
 * 
 * @note Speed changes persist until next speed command or mode change
 * @note Airspeed changes constrained by aparm.airspeed_min and aparm.airspeed_max
 * 
 * @see do_change_speed(SPEED_TYPE, float, float) for implementation details
 */
bool Plane::do_change_speed(const AP_Mission::Mission_Command& cmd)
{
    return do_change_speed(
        (SPEED_TYPE)cmd.content.speed.speed_type,
        cmd.content.speed.target_ms,
         cmd.content.speed.throttle_pct
        );
}

/**
 * @brief Change aircraft speed or throttle settings
 * 
 * @details Implements speed and throttle changes for mission execution. Supports
 *          three types of speed modifications with validation and range checking:
 *          
 *          AIRSPEED Mode:
 *          - Special value -2.0: Return to default airspeed (clears override)
 *          - Valid range: aparm.airspeed_min to aparm.airspeed_max
 *          - Sets new_airspeed_cm for AUTO/GUIDED modes
 *          - Affects TECS speed controller target
 *          
 *          GROUNDSPEED Mode:
 *          - Sets minimum groundspeed target
 *          - Updates aparm.min_groundspeed parameter
 *          - Used for wind compensation
 *          
 *          Throttle Override:
 *          - Valid range: 0-100 percent
 *          - Updates aparm.throttle_cruise
 *          - Can be combined with speed type change
 *          - Affects throttle controller baseline
 *          
 *          Parameter Persistence:
 *          - Changes stored in aparm parameters
 *          - Persist across waypoints within mission
 *          - Reset on mode change or mission end
 * 
 * @param[in] speedtype Speed type to modify (AIRSPEED or GROUNDSPEED)
 * @param[in] speed_target_ms Target speed in meters per second
 * @param[in] throttle_pct Throttle percentage (0-100), or 0 for no change
 * 
 * @return true if at least one parameter successfully changed
 * @return false if all parameters invalid or out of range
 * 
 * @note GCS notification sent for each successful change
 * @note Invalid speed requests silently ignored (returns false)
 * @note Throttle can be changed independently of speed type
 * 
 * @see aparm.airspeed_min minimum allowed airspeed
 * @see aparm.airspeed_max maximum allowed airspeed
 * @see aparm.min_groundspeed minimum groundspeed target
 * @see aparm.throttle_cruise cruise throttle percentage
 * @see new_airspeed_cm target airspeed in cm/s (-1 for default)
 */
bool Plane::do_change_speed(SPEED_TYPE speedtype, float speed_target_ms, float throttle_pct)
{
    switch (speedtype) {
    case SPEED_TYPE_AIRSPEED:
        if (is_equal(speed_target_ms, -2.0f)) {
            new_airspeed_cm = -1; // return to default airspeed
            return true;
        } else if ((speed_target_ms >= aparm.airspeed_min.get()) &&
                   (speed_target_ms <= aparm.airspeed_max.get()))  {
            new_airspeed_cm = speed_target_ms * 100; //new airspeed target for AUTO or GUIDED modes
            gcs().send_text(MAV_SEVERITY_INFO, "Set airspeed %u m/s", (unsigned)speed_target_ms);
            return true;
        }
        break;
    case SPEED_TYPE_GROUNDSPEED:
        gcs().send_text(MAV_SEVERITY_INFO, "Set groundspeed %u", (unsigned)speed_target_ms);
        aparm.min_groundspeed.set(speed_target_ms);
        return true;

    case SPEED_TYPE_CLIMB_SPEED:
    case SPEED_TYPE_DESCENT_SPEED:
    case SPEED_TYPE_ENUM_END:
        break;
    }

    if (throttle_pct > 0 && throttle_pct <= 100) {
        gcs().send_text(MAV_SEVERITY_INFO, "Set throttle %u", (unsigned)throttle_pct);
        aparm.throttle_cruise.set(throttle_pct);
        return true;
    }

    return false;
}

/**
 * @brief Execute DO_SET_HOME mission command
 * 
 * @details Sets the home location for return-to-launch and failsafe operations.
 *          Supports two modes: use current GPS location or use specified coordinates.
 *          
 *          Mode Selection (cmd.p1):
 *          - p1 == 1: Use current GPS position as home
 *            * Requires GPS_OK_FIX_3D or better
 *            * Calls set_home_persistently() to save to EEPROM
 *            * Persists across reboots
 *          
 *          - p1 != 1: Use coordinates from cmd.content.location
 *            * Uses mission-specified lat/lon/alt
 *            * Calls AP::ahrs().set_home()
 *            * RAM only, not persisted
 *          
 *          Home Location Usage:
 *          - RTL target when no rally points configured
 *          - Origin for relative altitude calculations
 *          - Failsafe return destination
 *          - Reference for distance-to-home telemetry
 *          
 *          Safety Considerations:
 *          - Validates GPS fix quality before using current location
 *          - Errors silently ignored to prevent mission abort
 *          - Invalid coordinates rejected by AHRS
 * 
 * @param[in] cmd Mission command with home mode (p1) and location coordinates
 * 
 * @note Home changes affect RTL behavior immediately
 * @note Persistent home saved to EEPROM only when p1==1
 * @note Errors silently ignored to maintain mission flow
 * 
 * @see set_home_persistently() for EEPROM storage
 * @see AP::ahrs().set_home() for non-persistent home update
 * @see gps.location() for current GPS coordinates
 */
void Plane::do_set_home(const AP_Mission::Mission_Command& cmd)
{
    if (cmd.p1 == 1 && gps.status() >= AP_GPS::GPS_OK_FIX_3D) {
        if (!set_home_persistently(gps.location())) {
            // silently ignore error
        }
    } else {
        if (!AP::ahrs().set_home(cmd.content.location)) {
            // silently ignore failure
        }
    }
}

/**
 * @brief Callback from AP_Mission when starting a new mission command
 * 
 * @details Safety wrapper around start_command() that validates flight mode before
 *          allowing mission command execution. This prevents the mission library from
 *          inadvertently triggering actions when not in AUTO mode.
 *          
 *          Mode Validation:
 *          - Only executes if control_mode == &mode_auto
 *          - Returns true in other modes to acknowledge command without execution
 *          - Protects against race conditions during mode changes
 *          
 *          Callback Registration:
 *          - Registered with AP_Mission library during initialization
 *          - Called once per mission item when starting NAV commands
 *          - Also called for DO commands that need setup
 * 
 * @param[in] cmd Mission command to start executing
 * 
 * @return true if command started successfully or mode not AUTO
 * @return false if command initialization failed (rare)
 * 
 * @note Called by AP_Mission library, not directly by vehicle code
 * @note Mode check prevents actions when switching out of AUTO
 * 
 * @see start_command() for actual command initialization logic
 * @see AP_Mission::set_start_fn() for callback registration
 */
bool Plane::start_command_callback(const AP_Mission::Mission_Command &cmd)
{
    if (control_mode == &mode_auto) {
        return start_command(cmd);
    }
    return true;
}

/**
 * @brief Callback from AP_Mission to check command completion
 * 
 * @details Safety wrapper around verify_command() that validates flight mode and
 *          sends telemetry when commands complete. Called repeatedly by the mission
 *          library while a command is active.
 *          
 *          Mode Validation:
 *          - Only verifies if control_mode == &mode_auto
 *          - Returns false in other modes to prevent mission progression
 *          - Ensures mission doesn't advance when mode switched away
 *          
 *          Completion Handling:
 *          - Checks command-specific completion criteria
 *          - Sends MAVLink MISSION_ITEM_REACHED message to GCS
 *          - Returns status to mission library for sequencing
 *          
 *          Call Frequency:
 *          - Called at 10Hz or higher during command execution
 *          - Frequency depends on scheduler task rate
 *          - Must execute quickly to not impact control loop
 * 
 * @param[in] cmd Mission command being verified
 * 
 * @return true if command complete and can advance to next command
 * @return false if still executing or not in AUTO mode
 * 
 * @note Called repeatedly by AP_Mission library during command execution
 * @note GCS notification sent only once when command completes
 * @note Mode check prevents mission advancement when not in AUTO
 * 
 * @see verify_command() for command-specific completion logic
 * @see gcs().send_mission_item_reached_message() for telemetry
 * @see AP_Mission::set_verify_fn() for callback registration
 */
bool Plane::verify_command_callback(const AP_Mission::Mission_Command& cmd)
{
    if (control_mode == &mode_auto) {
        bool cmd_complete = verify_command(cmd);

        // send message to GCS
        if (cmd_complete) {
            gcs().send_mission_item_reached_message(cmd.index);
        }

        return cmd_complete;
    }
    return false;
}

/**
 * @brief Callback from AP_Mission when mission completes
 * 
 * @details Called by the mission library when the last mission item has been
 *          executed. Automatically transitions vehicle to RTL mode for safe
 *          return to launch location.
 *          
 *          Mode Validation:
 *          - Only acts if control_mode == &mode_auto
 *          - Prevents unwanted RTL when mission library clears during mode change
 *          - Ensures clean mode transitions
 *          
 *          Completion Actions:
 *          1. Switch to RTL mode with MISSION_END reason
 *          2. Send telemetry notification to GCS
 *          3. RTL mode handles navigation to home/rally point
 *          
 *          Safety Behavior:
 *          - RTL ensures vehicle returns safely after mission
 *          - Altitude maintained or climbed to RTL_ALTITUDE
 *          - Loiter at home if further action not taken
 * 
 * @note Called by AP_Mission library when mission sequence completes
 * @note Automatic RTL can be disabled via parameter settings
 * @note Pilot can override with mode change at any time
 * 
 * @see set_mode() for mode transition to RTL
 * @see mode_rtl for return-to-launch behavior
 * @see AP_Mission::set_exit_fn() for callback registration
 */
void Plane::exit_mission_callback()
{
    if (control_mode == &mode_auto) {
        set_mode(mode_rtl, ModeReason::MISSION_END);
        gcs().send_text(MAV_SEVERITY_INFO, "Mission complete, changing mode to RTL");
    }
}

#if HAL_QUADPLANE_ENABLED
/**
 * @brief Verify fixed-wing VTOL landing approach progression
 * 
 * @details Multi-stage state machine managing fixed-wing spiral approach before
 *          VTOL landing. Handles loiter descent, wind-aligned breakout, straight-in
 *          approach, and VTOL transition. Returns true when ready for VTOL landing.
 *          
 *          Approach Radius Configuration:
 *          - Uses quadplane.fw_land_approach_radius if configured
 *          - Falls back to aparm.loiter_radius if fw_land_approach_radius is zero
 *          - Direction: radius sign determines CW (+) or CCW (-) loiter
 *          - abs_radius used for distance calculations
 *          
 *          Five-Stage Approach Sequence:
 *          
 *          Stage 1: RTL (Return to Launch)
 *          - Fly home and loiter at RTL altitude
 *          - update_loiter() maintains circular pattern
 *          - Waits for reached_loiter_target()
 *          - Transitions: Calls do_RTL(qrtl_alt) and moves to LOITER_TO_ALT
 *          
 *          Stage 2: LOITER_TO_ALT (Spiral Descent)
 *          - Descend in loiter to VTOL transition altitude
 *          - Monitors loiter.reached_target_alt or unable_to_acheive_target_alt
 *          - Requires loiter.sum_cd > 1 (completed some loiter angle)
 *          - Estimates wind: ahrs.wind_estimate()
 *          - Computes approach_direction_deg: Into wind (atan2f(-wind.y, -wind.x))
 *          - GCS notification: "Selected an approach path of X.X"
 *          - Transitions to ENSURE_RADIUS
 *          
 *          Stage 3: ENSURE_RADIUS (Position Validation)
 *          - Validates aircraft at correct distance from landing point
 *          - Checks: Distance from loiter center within 5m of abs_radius
 *          - Also requires: loiter.sum_cd >= 2 centidegrees
 *          - Special case: 1 centidegree has special meaning, require >=2
 *          - If not positioned correctly: Continues loitering
 *          - When positioned: Transitions to WAIT_FOR_BREAKOUT (fallthrough)
 *          
 *          Stage 4: WAIT_FOR_BREAKOUT (Timing Exit)
 *          - Times loiter exit to face correct direction
 *          - Computes breakout_direction_rad: 90° or 270° from approach path
 *          - Direction depends on loiter direction (CW/CCW)
 *          - Waits for heading within 5° of breakout direction
 *          - When aligned: GCS "Starting VTOL land approach path"
 *          - set_next_WP(landing location)
 *          - Transitions to APPROACH_LINE (fallthrough)
 *          
 *          Stage 5: APPROACH_LINE (Straight-In Approach)
 *          - Projects 1km approach path on both sides of landing point
 *          - start: 1km behind landing point (approach_direction + 180°)
 *          - end: 1km ahead of landing point (approach_direction)
 *          - update_waypoint(start, end) for straight navigation
 *          
 *          Completion Criteria (must satisfy past_finish_line AND (lined_up OR half_radius)):
 *          1. past_finish_line: Crossed threshold (landing point - stopping_distance)
 *          2. lined_up: Velocity vector within 30° of target direction
 *          3. half_radius: Passed 50% distance from breakout to landing
 *          
 *          When complete:
 *          - Transitions to VTOL_LANDING stage
 *          - Calls quadplane.do_vtol_land(cmd)
 *          - Returns true to signal VTOL landing handover
 *          
 *          Wind Compensation Strategy:
 *          - Determines approach direction from AHRS wind estimate
 *          - Approach into wind minimizes ground speed at transition
 *          - Reduces VTOL hover drift after transition
 *          - Improves landing accuracy and stability
 *          
 *          Safety Margins:
 *          - Stopping distance accounts for transition deceleration
 *          - Breakout threshold includes stopping margin
 *          - Lined-up check ensures good approach alignment
 *          - Half-radius fallback if alignment difficult to achieve
 * 
 * @param[in] cmd Mission command with landing location
 *                cmd.content.location: VTOL landing touchdown point
 * 
 * @return true when ready for VTOL landing (stage VTOL_LANDING reached)
 * @return false while still executing approach stages
 * 
 * @note Only compiled with HAL_QUADPLANE_ENABLED
 * @note Approach direction optimized for wind conditions
 * @note Multiple completion criteria ensure safe transition
 * 
 * @see do_landing_vtol_approach() for initialization
 * @see vtol_approach_s.approach_stage for current stage
 * @see quadplane.fw_land_approach_radius for radius configuration
 * @see quadplane.stopping_distance() for transition deceleration
 */
bool Plane::verify_landing_vtol_approach(const AP_Mission::Mission_Command &cmd)
{
    const float radius = is_zero(quadplane.fw_land_approach_radius)? aparm.loiter_radius : quadplane.fw_land_approach_radius;
    const int8_t direction = is_negative(radius) ? -1 : 1;
    const float abs_radius = fabsf(radius);

    loiter.direction = direction;

    switch (vtol_approach_s.approach_stage) {
        case VTOLApproach::Stage::RTL:
            {
                // fly home and loiter at RTL alt
                nav_controller->update_loiter(cmd.content.location, abs_radius, direction);
                if (plane.reached_loiter_target()) {
                    // descend to Q RTL alt
                    plane.do_RTL(plane.home.alt + plane.quadplane.qrtl_alt*100UL);
                    plane.loiter_angle_reset();
                    vtol_approach_s.approach_stage = VTOLApproach::Stage::LOITER_TO_ALT;
                }
                break;
            }
        case VTOLApproach::Stage::LOITER_TO_ALT:
            {
                nav_controller->update_loiter(cmd.content.location, abs_radius, direction);

                if (labs(loiter.sum_cd) > 1 && (loiter.reached_target_alt || loiter.unable_to_acheive_target_alt)) {
                    Vector3f wind = ahrs.wind_estimate();
                    vtol_approach_s.approach_direction_deg = degrees(atan2f(-wind.y, -wind.x));
                    gcs().send_text(MAV_SEVERITY_INFO, "Selected an approach path of %.1f", (double)vtol_approach_s.approach_direction_deg);
                    vtol_approach_s.approach_stage = VTOLApproach::Stage::ENSURE_RADIUS;
                }
                break;
            }
        case VTOLApproach::Stage::ENSURE_RADIUS:
            {
                // validate that the vehicle is at least the expected distance away from the loiter point
                // require an angle total of at least 2 centidegrees, due to special casing of 1 centidegree
                if (((fabsF(cmd.content.location.get_distance(current_loc) - abs_radius) > 5.0f) &&
                      (cmd.content.location.get_distance(current_loc) < abs_radius)) ||
                    (labs(loiter.sum_cd) < 2)) {
                    nav_controller->update_loiter(cmd.content.location, abs_radius, direction);
                    break;
                }
                vtol_approach_s.approach_stage = VTOLApproach::Stage::WAIT_FOR_BREAKOUT;
                FALLTHROUGH;
            }
        case VTOLApproach::Stage::WAIT_FOR_BREAKOUT:
            {
                nav_controller->update_loiter(cmd.content.location, radius, direction);

                const float breakout_direction_rad = radians(vtol_approach_s.approach_direction_deg + (direction > 0 ? 270 : 90));

                // breakout when within 5 degrees of the opposite direction
                if (fabsF(wrap_PI(ahrs.get_yaw_rad() - breakout_direction_rad)) < radians(5.0f)) {
                    gcs().send_text(MAV_SEVERITY_INFO, "Starting VTOL land approach path");
                    vtol_approach_s.approach_stage = VTOLApproach::Stage::APPROACH_LINE;
                    set_next_WP(cmd.content.location);
                    // fallthrough
                } else {
                    break;
                }
                FALLTHROUGH;
            }
        case VTOLApproach::Stage::APPROACH_LINE:
            {
                // project an approach path
                Location start = cmd.content.location;
                Location end = cmd.content.location;

                // project a 1km waypoint to either side of the landing location
                start.offset_bearing(vtol_approach_s.approach_direction_deg + 180, 1000);
                end.offset_bearing(vtol_approach_s.approach_direction_deg, 1000);

                nav_controller->update_waypoint(start, end);

                // check if we should move on to the next waypoint
                Location breakout_stopping_loc = cmd.content.location;
                breakout_stopping_loc.offset_bearing(vtol_approach_s.approach_direction_deg + 180, quadplane.stopping_distance());
                const bool past_finish_line = current_loc.past_interval_finish_line(start, breakout_stopping_loc);

                Location breakout_loc = cmd.content.location;
                breakout_loc.offset_bearing(vtol_approach_s.approach_direction_deg + 180, abs_radius);
                const bool half_radius = current_loc.line_path_proportion(breakout_loc, cmd.content.location) > 0.5;
                bool lined_up = true;
                Vector3f vel_NED;
                if (ahrs.get_velocity_NED(vel_NED)) {
                    const Vector2f target_vec = current_loc.get_distance_NE(cmd.content.location);
                    const float angle_err = fabsf(wrap_180(degrees(vel_NED.xy().angle(target_vec))));
                    lined_up = (angle_err < 30);
                }

                if (past_finish_line && (lined_up || half_radius)) {
                    vtol_approach_s.approach_stage = VTOLApproach::Stage::VTOL_LANDING;
                    quadplane.do_vtol_land(cmd);
                    // fallthrough
                } else {
                    break;
                }
                FALLTHROUGH;
            }
        case VTOLApproach::Stage::VTOL_LANDING:
            // nothing to do here, we should be into the quadplane landing code
            return true;
    }

    return false;
}
#endif // HAL_QUADPLANE_ENABLED

/**
 * @brief Verify aircraft heading is aligned with next waypoint bearing
 * 
 * @details Secondary completion criterion for loiter commands that require the
 *          aircraft to exit the loiter aligned with the next mission leg. This
 *          prevents sharp turns when transitioning from loiter to the next waypoint.
 *          
 *          Alignment Check Process:
 *          1. Retrieve next NAV waypoint from mission
 *          2. Calculate bearing from current waypoint to next waypoint
 *          3. Compare aircraft heading to required bearing
 *          4. Use mode_loiter.isHeadingLinedUp() for threshold check
 *          
 *          Special Cases:
 *          - No next waypoint: Return true immediately (no alignment needed)
 *          - VTOL auto mode: Skip heading check, return true
 *          - Autoland mode: Use autoland's own lineup criterion
 *          
 *          Initialization Mode:
 *          - If init == true: Reset loiter.sum_cd to 0
 *          - Used when transitioning from primary goal (time/turns/alt) to heading
 *          - Allows fresh heading alignment after primary completion
 *          
 *          Heading Alignment Tolerance:
 *          - Threshold defined in mode_loiter.isHeadingLinedUp()
 *          - Typically allows smooth exit from loiter circle
 *          - Prevents overshoot or sharp course corrections
 * 
 * @param[in] init If true, reset loiter angle accumulator for fresh start
 * 
 * @return true if heading aligned with next waypoint or no next waypoint exists
 * @return false if heading alignment in progress
 * 
 * @note Called as secondary phase of loiter completion verification
 * @note Requires valid next NAV command in mission
 * @note VTOL and autoland modes bypass this check
 * 
 * @see verify_loiter_time() for time-based loiter that uses this
 * @see verify_loiter_turns() for turn-based loiter that uses this
 * @see verify_loiter_to_alt() for altitude-based loiter that uses this
 * @see mode_loiter.isHeadingLinedUp() for actual alignment threshold check
 */
/**
 * @brief Verify aircraft heading alignment for loiter exit
 * 
 * @details Secondary completion criterion for loiter commands ensuring aircraft
 *          exits loiter on proper heading toward next waypoint. Prevents sharp
 *          turns after loiter completion, maintains smooth flight path, and
 *          optimizes approach angle for next navigation segment.
 *          
 *          Loiter commands with secondary heading goal use two-phase completion:
 *          1. Primary Goal: Time elapsed, turns completed, or altitude reached
 *          2. Secondary Goal: This function - heading aligned to next waypoint
 *          
 *          This function handles the secondary (heading) goal only.
 *          
 *          Early Exit Conditions (returns true immediately):
 *          
 *          VTOL Auto Mode:
 *          - quadplane.in_vtol_auto() returns true
 *          - VTOL doesn't need fixed-wing heading alignment
 *          - Multirotors can point any direction, then turn in place
 *          - Fixed-wing heading logic not applicable
 *          
 *          Autoland Mode:
 *          - control_mode == &mode_autoland
 *          - Delegates to mode_autoland.landing_lined_up()
 *          - Autoland has specialized lineup criteria
 *          - Uses runway alignment logic instead of waypoint heading
 *          
 *          No Next Waypoint:
 *          - mission.get_next_nav_cmd() fails to retrieve next waypoint
 *          - Last waypoint in mission or mission structure issue
 *          - No heading target available - exit loiter immediately
 *          - Prevents indefinite loiter at mission end
 *          
 *          Normal Heading Verification:
 *          
 *          Initialization (init == true):
 *          - Called when transitioning from primary to secondary goal
 *          - Resets loiter.sum_cd = 0
 *          - Clears accumulated loiter angle from primary phase
 *          - Starts fresh heading alignment tracking
 *          
 *          Heading Alignment Check (init == false):
 *          - Retrieves next_nav_cmd: Next navigation waypoint after current loiter
 *          - Calls mode_loiter.isHeadingLinedUp(current_wp, next_wp)
 *          - Checks if aircraft heading within acceptable angle of desired track
 *          - Returns true when aligned, false when still turning to align
 *          
 *          Typical Usage Pattern:
 *          1. Primary loiter goal completes (time/turns/altitude)
 *          2. condition_value = 0 signals switch to secondary goal
 *          3. verify_loiter_heading(true) called to initialize
 *          4. Subsequent calls verify_loiter_heading(false) check alignment
 *          5. Returns true when heading acceptable for loiter exit
 *          6. Mission proceeds to next waypoint with good approach angle
 *          
 *          Heading Alignment Tolerance:
 *          - Defined by mode_loiter.isHeadingLinedUp() implementation
 *          - Balances mission progress vs trajectory smoothness
 *          - Typically allows some heading error to prevent excessive loiter
 *          - May consider turn radius and aircraft dynamics
 *          
 *          Flight Path Optimization:
 *          - Smooth exit reduces passenger discomfort
 *          - Prevents excessive bank angles after loiter
 *          - Maintains efficient flight path
 *          - Important for energy management (TECS)
 *          - Critical for camera/sensor pointing stability
 * 
 * @param[in] init true to initialize heading check (reset loiter angle tracking),
 *                 false to verify current heading alignment
 * 
 * @return true when heading aligned for loiter exit (or special mode/condition)
 * @return false when still waiting for heading alignment
 * 
 * @note Called as secondary completion criterion after primary loiter goal met
 * @note VTOL and autoland modes have specialized handling
 * @note Returns true immediately if no next waypoint available
 * 
 * @see verify_loiter_time() for primary time-based loiter goal
 * @see verify_loiter_turns() for primary turns-based loiter goal
 * @see verify_loiter_to_alt() for primary altitude-based loiter goal
 * @see mode_loiter.isHeadingLinedUp() for alignment logic
 */
bool Plane::verify_loiter_heading(bool init)
{
#if HAL_QUADPLANE_ENABLED
    if (quadplane.in_vtol_auto()) {
        // skip heading verify if in VTOL auto
        return true;
    }
#endif

#if MODE_AUTOLAND_ENABLED
    if (control_mode == &mode_autoland) {
        // autoland mode has its own lineup criterion
        return mode_autoland.landing_lined_up();
    }
#endif

    //Get the lat/lon of next Nav waypoint after this one:
    AP_Mission::Mission_Command next_nav_cmd;
    if (! mission.get_next_nav_cmd(mission.get_current_nav_index() + 1,
                                   next_nav_cmd)) {
        //no next waypoint to shoot for -- go ahead and break out of loiter
        return true;
    }

    if (init) {
        loiter.sum_cd = 0;
    }

    return plane.mode_loiter.isHeadingLinedUp(next_WP_loc, next_nav_cmd.content.location);
}

/**
 * @brief Get waypoint acceptance radius for current flight mode
 * 
 * @details Returns appropriate waypoint acceptance radius based on vehicle
 *          mode (VTOL vs fixed-wing). VTOL modes use tighter radius from
 *          position controller, fixed-wing uses configured waypoint_radius.
 *          
 *          Mode-Specific Radius Selection:
 *          
 *          VTOL Mode (quadplane.in_vtol_mode() == true):
 *          - Uses quadplane.wp_nav->get_wp_radius_cm()
 *          - Position controller defines tight radius for hover precision
 *          - Typically smaller radius (e.g., 2-5 meters)
 *          - Converted from centimeters to meters (* 0.01)
 *          - Appropriate for vertical landing, precise hover positioning
 *          
 *          Fixed-Wing Mode:
 *          - Uses g.waypoint_radius parameter
 *          - Larger radius accounts for turn dynamics
 *          - Typically 30-100 meters depending on speed/bank limits
 *          - Prevents excessive maneuvering to hit precise point
 *          - Already in meters, no conversion needed
 *          
 *          Waypoint Acceptance Radius Purpose:
 *          - Determines "close enough" distance to waypoint
 *          - When within radius: Waypoint considered reached
 *          - Too small: Excessive maneuvering, may never reach
 *          - Too large: Imprecise navigation, cuts corners
 *          
 *          Usage in Turn Distance Calculations:
 *          - nav_controller->turn_distance(get_wp_radius(), next_turn_angle)
 *          - Computes when to start turn for next waypoint
 *          - Larger radius: Earlier turn initiation
 *          - Smoother flight path with appropriate radius
 *          
 *          Fixed-Wing Considerations:
 *          - Bank angle limits constrain turn radius
 *          - Higher speeds require larger acceptance radius
 *          - Wind affects achievable accuracy
 *          - Waypoint radius should exceed minimum turn radius
 *          
 *          VTOL Considerations:
 *          - Can hover and translate precisely
 *          - Smaller radius appropriate for precision tasks
 *          - Landing, payload delivery benefit from tight tolerance
 *          - Position controller actively maintains setpoint
 * 
 * @return Waypoint acceptance radius in meters (mode-dependent)
 * 
 * @note VTOL modes use position controller radius (typically 2-5m)
 * @note Fixed-wing uses configured waypoint_radius parameter (typically 30-100m)
 * @note Const function - no side effects
 * 
 * @see verify_nav_wp() for waypoint acceptance logic
 * @see nav_controller->turn_distance() for turn initiation calculation
 * @see g.waypoint_radius for fixed-wing radius parameter
 * @see quadplane.wp_nav->get_wp_radius_cm() for VTOL radius
 */
float Plane::get_wp_radius() const
{
#if HAL_QUADPLANE_ENABLED
    if (plane.quadplane.in_vtol_mode()) {
        return plane.quadplane.wp_nav->get_wp_radius_cm() * 0.01;
    }
#endif
    return g.waypoint_radius;
}

#if AP_SCRIPTING_ENABLED
/**
 * @brief Initialize NAV_SCRIPT_TIME mission command for Lua script control
 * 
 * @details Enables scripted navigation allowing Lua scripts to directly command
 *          aircraft attitude rates and throttle for aerobatic maneuvers and custom
 *          flight behaviors. Initializes nav_scripting state with current control
 *          targets to ensure smooth transition from autopilot to script control.
 *          
 *          Scripted Navigation Concept:
 *          - Lua scripts take over low-level flight control
 *          - Scripts command roll/pitch/yaw rates and throttle directly
 *          - Enables aerobatics: loops, rolls, hammerheads, knife-edge, etc.
 *          - Supports custom flight profiles not available in standard modes
 *          - Mission command integration: Scripts run as mission items
 *          
 *          Initialization Sequence:
 *          
 *          1. Enable Scripting Control:
 *          - nav_scripting.enabled = true
 *          - Signals scripting system is active
 *          - Checked by nav_scripting_active() and control loops
 *          
 *          2. Command Instance ID:
 *          - nav_scripting.id++
 *          - Unique identifier for this script command instance
 *          - Scripts use ID to confirm they're controlling current command
 *          - Prevents stale script commands from affecting new instances
 *          
 *          3. Timestamp Initialization:
 *          - nav_scripting.start_ms = AP_HAL::millis()
 *          - Records command start time for timeout tracking
 *          - nav_scripting.current_ms = start_ms
 *          - Tracks last script update, ensures scripts remain active
 *          
 *          4. Smooth Transition - Initialize Control Targets:
 *          - nav_scripting.roll_rate_dps: Current roll controller target (deg/s)
 *          - nav_scripting.pitch_rate_dps: Current pitch controller target (deg/s)
 *          - nav_scripting.yaw_rate_dps: Current gyro yaw rate (deg/s)
 *          - nav_scripting.throttle_pct: Current throttle output (%)
 *          
 *          Starting from current targets ensures:
 *          - No sudden control discontinuity
 *          - Aircraft doesn't jerk when script takes over
 *          - Script can smoothly transition to desired maneuver
 *          - Safe handoff from autopilot to script
 *          
 *          Control Rate Extraction:
 *          - Roll: rollController.get_pid_info().target (desired roll rate)
 *          - Pitch: pitchController.get_pid_info().target (desired pitch rate)
 *          - Yaw: degrees(ahrs.get_gyro().z) (current yaw rate from gyro)
 *          - Throttle: SRV_Channels::get_output_scaled(k_throttle) (current output)
 *          
 *          Script Interaction:
 *          - Script calls nav_script_time() to retrieve command parameters
 *          - Script calls set_target_throttle_rate_rpy() to command aircraft
 *          - Script calls nav_script_time_done() when maneuver complete
 *          - Timeout: cmd.content.nav_script_time.timeout_s enforces time limit
 *          
 *          Safety Mechanisms:
 *          - Timeout protection: verify_nav_script_time() monitors timeout
 *          - Watchdog: 1000ms without script update disables nav_scripting
 *          - Rate limits: set_target_throttle_rate_rpy() constrains commands
 *          - Manual override: RC mode change exits script control
 *          
 *          Typical Use Cases:
 *          - Aerobatic sequences (loops, rolls, immelmann, split-s)
 *          - Survey patterns requiring precise maneuvers
 *          - Dynamic soaring scripts for autonomous glider flight
 *          - Precision landing approaches with custom glide slopes
 *          - Photo missions with specific bank angles and rates
 * 
 * @param[in] cmd Mission command with script parameters
 *                cmd.content.nav_script_time.command: Script command ID
 *                cmd.content.nav_script_time.timeout_s: Maximum execution time
 *                cmd.content.nav_script_time.arg1-4: Script-specific arguments
 * 
 * @note Only compiled with AP_SCRIPTING_ENABLED
 * @note Initializes with current control targets for smooth transition
 * @note Scripts must call set_target_throttle_rate_rpy() within 1000ms
 * 
 * @see verify_nav_script_time() for timeout monitoring and completion
 * @see nav_script_time() for script parameter retrieval
 * @see set_target_throttle_rate_rpy() for script control interface
 * @see nav_script_time_done() for script completion signaling
 */
void Plane::do_nav_script_time(const AP_Mission::Mission_Command& cmd)
{
    nav_scripting.enabled = true;
    nav_scripting.id++;
    nav_scripting.start_ms = AP_HAL::millis();
    nav_scripting.current_ms = nav_scripting.start_ms;

    // start with current roll rate, pitch rate and throttle
    nav_scripting.roll_rate_dps = plane.rollController.get_pid_info().target;
    nav_scripting.pitch_rate_dps = plane.pitchController.get_pid_info().target;
    nav_scripting.yaw_rate_dps = degrees(ahrs.get_gyro().z);
    nav_scripting.throttle_pct = SRV_Channels::get_output_scaled(SRV_Channel::k_throttle);
}

/**
 * @brief Verify NAV_SCRIPT_TIME command completion or timeout
 * 
 * @details Monitors scripted navigation command for completion or timeout,
 *          returns true when script signals done or timeout expires. Ensures
 *          mission doesn't hang if script fails or takes too long.
 *          
 *          Completion Criteria:
 *          - Script calls nav_script_time_done(id) → nav_scripting.enabled = false
 *          - Function returns !nav_scripting.enabled (true when disabled)
 *          - Mission progresses to next command
 *          
 *          Timeout Protection (cmd.content.nav_script_time.timeout_s > 0):
 *          
 *          Purpose:
 *          - Prevents indefinite script execution
 *          - Ensures mission progress even if script stalls
 *          - Safety net for script errors or infinite loops
 *          - Allows controlled script runtime limits
 *          
 *          Timeout Check:
 *          - now = AP_HAL::millis()
 *          - Elapsed time: now - nav_scripting.start_ms
 *          - Timeout threshold: timeout_s * 1000 (convert seconds to ms)
 *          - If elapsed > threshold: Timeout triggered
 *          
 *          Timeout Actions:
 *          1. GCS Notification:
 *             - gcs().send_text(MAV_SEVERITY_INFO, "NavScriptTime timed out")
 *             - Alerts operator script didn't complete normally
 *             - Important for post-flight analysis
 *          
 *          2. Disable Scripting Control:
 *             - nav_scripting.enabled = false
 *             - Returns control to autopilot
 *             - Scripts can no longer command aircraft
 *          
 *          3. Reset Rudder Offset:
 *             - nav_scripting.rudder_offset_pct = 0
 *             - Clears any rudder bias set by script
 *             - Prevents stuck rudder affecting subsequent flight
 *          
 *          4. Re-enable Yaw Rate Controller:
 *             - nav_scripting.run_yaw_rate_controller = true
 *             - Restores normal yaw damping
 *             - Scripts can disable controller for aerobatics
 *             - Must re-enable for normal flight
 *          
 *          No Timeout Configuration (timeout_s == 0 or timeout_s < 0):
 *          - Timeout check skipped
 *          - Script must explicitly call nav_script_time_done()
 *          - Useful for scripts with variable completion time
 *          - Risk: Mission hangs if script fails
 *          - Watchdog: nav_scripting_active() has 1000ms update timeout
 *          
 *          Called Frequency:
 *          - Called at autopilot update rate (typically 50-400 Hz)
 *          - Frequent timeout checking ensures timely response
 *          - Low overhead: Simple time comparison
 *          
 *          Interaction with nav_scripting_active():
 *          - nav_scripting_active() has separate 1000ms watchdog
 *          - Disables scripting if no set_target_throttle_rate_rpy() calls
 *          - Catches script crashes or communication failures
 *          - Timeout here is for script runtime limit
 * 
 * @param[in] cmd Mission command with timeout parameter
 *                cmd.content.nav_script_time.timeout_s: Maximum script runtime (seconds)
 *                                                        0 or negative = no timeout
 * 
 * @return true when script completes or times out (mission proceeds)
 * @return false while script still running (continue waiting)
 * 
 * @note Timeout_s > 0 enables timeout protection
 * @note Timeout_s <= 0 disables timeout (script must call nav_script_time_done)
 * @note Resets rudder offset and yaw controller on timeout
 * @note Only compiled with AP_SCRIPTING_ENABLED
 * 
 * @see do_nav_script_time() for initialization
 * @see nav_script_time_done() for script completion signaling
 * @see nav_scripting_active() for 1000ms update watchdog
 */
bool Plane::verify_nav_script_time(const AP_Mission::Mission_Command& cmd)
{
    if (cmd.content.nav_script_time.timeout_s > 0) {
        const uint32_t now = AP_HAL::millis();
        if (now - nav_scripting.start_ms > cmd.content.nav_script_time.timeout_s*1000U) {
            gcs().send_text(MAV_SEVERITY_INFO, "NavScriptTime timed out");
            nav_scripting.enabled = false;
            nav_scripting.rudder_offset_pct = 0;
            nav_scripting.run_yaw_rate_controller = true;
        }
    }
    return !nav_scripting.enabled;
}

/**
 * @brief Check if scripted navigation is currently active and monitor watchdog
 * 
 * @details Returns whether nav_scripting is enabled and enforces watchdog timeout
 *          to detect script failures. Disables scripting if script stops updating
 *          control commands or if no longer in NAV_SCRIPT_TIME mission command.
 *          
 *          Watchdog Timeout Protection (1000ms):
 *          
 *          Purpose:
 *          - Detect script crashes or communication failures
 *          - Prevent runaway aircraft if script stops responding
 *          - Independent from command timeout in verify_nav_script_time()
 *          - Ensures timely handoff back to autopilot
 *          
 *          Watchdog Mechanism:
 *          - nav_scripting.current_ms updated by set_target_throttle_rate_rpy()
 *          - Elapsed time: AP_HAL::millis() - nav_scripting.current_ms
 *          - Timeout threshold: 1000ms (1 second)
 *          - Script must update controls at least once per second
 *          
 *          Timeout Actions (identical to verify_nav_script_time timeout):
 *          1. nav_scripting.enabled = false (disable scripting)
 *          2. nav_scripting.current_ms = 0 (reset timestamp)
 *          3. nav_scripting.rudder_offset_pct = 0 (clear rudder bias)
 *          4. nav_scripting.run_yaw_rate_controller = true (restore yaw control)
 *          5. GCS notification: "NavScript time out"
 *          
 *          Typical Script Update Rate:
 *          - Scripts typically run at 10-50 Hz
 *          - Call set_target_throttle_rate_rpy() each iteration
 *          - 1000ms timeout allows for script processing delays
 *          - Catches script hangs, not just slow updates
 *          
 *          Mission Command Validation:
 *          - Checks control_mode == &mode_auto
 *          - Checks mission.get_current_nav_cmd().id == MAV_CMD_NAV_SCRIPT_TIME
 *          - If not in AUTO or not in NAV_SCRIPT_TIME: Disable scripting
 *          - Prevents scripting control outside of mission context
 *          - Mode changes automatically terminate script control
 *          
 *          Usage in Flight Control:
 *          - Called by control loops to check if using script commands
 *          - If true: Use nav_scripting.roll_rate_dps, pitch_rate_dps, etc.
 *          - If false: Use normal autopilot control
 *          - Ensures clean handoff between script and autopilot
 *          
 *          Differences from verify_nav_script_time():
 *          - verify_nav_script_time(): Mission command timeout (configurable)
 *          - nav_scripting_active(): Script update watchdog (fixed 1000ms)
 *          - Both can disable scripting, serve different purposes
 *          - verify checks overall command completion
 *          - nav_scripting_active checks script is still alive
 *          
 *          Safety Philosophy:
 *          - Multiple layers of protection
 *          - Watchdog catches communication/crash failures
 *          - Command timeout limits overall runtime
 *          - Mode check prevents inappropriate activation
 *          - All ensure prompt return to autopilot control
 * 
 * @return true if nav_scripting is enabled and script is active
 * @return false if scripting disabled, timed out, or wrong mode/command
 * 
 * @note Script must call set_target_throttle_rate_rpy() within 1000ms
 * @note Automatically disabled if not in AUTO mode with NAV_SCRIPT_TIME
 * @note Only compiled with AP_SCRIPTING_ENABLED
 * 
 * @see set_target_throttle_rate_rpy() to update control and refresh watchdog
 * @see verify_nav_script_time() for command completion timeout
 * @see do_nav_script_time() for initialization
 */
bool Plane::nav_scripting_active(void)
{
    if (nav_scripting.enabled && AP_HAL::millis() - nav_scripting.current_ms > 1000) {
        // set_target_throttle_rate_rpy has not been called from script in last 1000ms
        nav_scripting.enabled = false;
        nav_scripting.current_ms = 0;
        nav_scripting.rudder_offset_pct = 0;
        nav_scripting.run_yaw_rate_controller = true;
        gcs().send_text(MAV_SEVERITY_INFO, "NavScript time out");
    }
    if (control_mode == &mode_auto &&
        mission.get_current_nav_cmd().id != MAV_CMD_NAV_SCRIPT_TIME) {
        nav_scripting.enabled = false;
    }
    return nav_scripting.enabled;
}

/**
 * @brief Retrieve current NAV_SCRIPT_TIME mission command parameters for scripts
 * 
 * @details Provides Lua scripts with access to mission command parameters when
 *          NAV_SCRIPT_TIME is active. Returns command ID and arguments allowing
 *          scripts to execute mission-specific behaviors. Scripts poll this
 *          function to determine what maneuver or action to perform.
 *          
 *          Script Interaction Pattern:
 *          1. Mission loads NAV_SCRIPT_TIME command
 *          2. do_nav_script_time() initializes nav_scripting
 *          3. Script calls nav_script_time() to get command parameters
 *          4. Script decodes command and arguments
 *          5. Script executes appropriate maneuver
 *          6. Script calls nav_script_time_done() when complete
 *          
 *          Parameter Retrieval:
 *          - Checks nav_scripting_active() for validation
 *          - If not active: Returns false (no command available)
 *          - If active: Extracts parameters from mission command
 *          
 *          Mission Command Structure:
 *          - mission.get_current_nav_cmd(): Current navigation command
 *          - .content.nav_script_time: NAV_SCRIPT_TIME specific fields
 *          - .command: Script command ID (identifies maneuver type)
 *          - .arg1, .arg2: Float arguments (speeds, angles, distances)
 *          - .arg3, .arg4: Integer arguments (counts, flags, modes)
 *          
 *          Output Parameters (passed by reference):
 *          
 *          @param[out] id: nav_scripting.id
 *          - Unique identifier for this command instance
 *          - Script stores this to confirm commands match
 *          - Passed to nav_script_time_done(id) on completion
 *          - Prevents scripts from completing wrong command
 *          
 *          @param[out] cmd: c.command
 *          - Script command ID (user-defined meaning)
 *          - Example: 1 = loop, 2 = roll, 3 = knife-edge, etc.
 *          - Scripts use switch/case on this value
 *          - Determines which maneuver to execute
 *          
 *          @param[out] arg1: c.arg1.get()
 *          - First float argument (mission planner field p1)
 *          - Typical uses: Target speed, roll angle, radius
 *          - Units defined by script convention
 *          - .get() extracts float from AP_Float type
 *          
 *          @param[out] arg2: c.arg2.get()
 *          - Second float argument (mission planner field p2)
 *          - Typical uses: Duration, height change, turn rate
 *          - Units defined by script convention
 *          - .get() extracts float from AP_Float type
 *          
 *          @param[out] arg3: c.arg3
 *          - First integer argument (16-bit signed)
 *          - Typical uses: Repeat count, direction flag, mode
 *          - Faster for simple counters and flags
 *          
 *          @param[out] arg4: c.arg4
 *          - Second integer argument (16-bit signed)
 *          - Typical uses: Additional flags, sub-modes
 *          - Often used for option bitmasks
 *          
 *          Typical Script Usage Pattern:
 *          ```lua
 *          function update()
 *              local id, cmd, arg1, arg2, arg3, arg4 = plane:nav_script_time()
 *              if not id then
 *                  return -- Not in scripting mode
 *              end
 *              
 *              if cmd == 1 then -- Loop
 *                  local radius = arg1
 *                  local g_force = arg2
 *                  perform_loop(radius, g_force)
 *              elseif cmd == 2 then -- Roll
 *                  local roll_rate = arg1
 *                  perform_roll(roll_rate)
 *              end
 *              
 *              if maneuver_complete then
 *                  plane:nav_script_time_done(id)
 *              end
 *          end
 *          ```
 *          
 *          Safety Validation:
 *          - nav_scripting_active() checks mode and watchdog
 *          - Returns false if scripting disabled or timed out
 *          - Prevents scripts from accessing stale parameters
 *          - Ensures script only executes in appropriate context
 * 
 * @param[out] id       Command instance identifier from nav_scripting.id
 * @param[out] cmd      Script command ID from mission command field
 * @param[out] arg1     First float argument (user-defined meaning)
 * @param[out] arg2     Second float argument (user-defined meaning)
 * @param[out] arg3     First integer argument (user-defined meaning)
 * @param[out] arg4     Second integer argument (user-defined meaning)
 * 
 * @return true if NAV_SCRIPT_TIME active and parameters retrieved
 * @return false if not in scripting mode (parameters not valid)
 * 
 * @note Only returns true when nav_scripting_active()
 * @note Scripts must store id for nav_script_time_done(id)
 * @note Parameter meanings are script-defined conventions
 * @note Only compiled with AP_SCRIPTING_ENABLED
 * 
 * @see do_nav_script_time() for command initialization
 * @see nav_scripting_active() for validation and watchdog
 * @see nav_script_time_done() for signaling completion
 */
bool Plane::nav_script_time(uint16_t &id, uint8_t &cmd, float &arg1, float &arg2, int16_t &arg3, int16_t &arg4)
{
    if (!nav_scripting_active()) {
        // not in NAV_SCRIPT_TIME
        return false;
    }
    const auto &c = mission.get_current_nav_cmd().content.nav_script_time;
    id = nav_scripting.id;
    cmd = c.command;
    arg1 = c.arg1.get();
    arg2 = c.arg2.get();
    arg3 = c.arg3;
    arg4 = c.arg4;
    return true;
}

/**
 * @brief Signal NAV_SCRIPT_TIME command completion from Lua script
 * 
 * @details Called by Lua scripts to indicate scripted maneuver is complete and
 *          mission should proceed to next command. Validates command ID to ensure
 *          script is completing the correct command instance, preventing race
 *          conditions and stale completions.
 *          
 *          Script Completion Flow:
 *          1. Script retrieves id from nav_script_time()
 *          2. Script executes maneuver/behavior
 *          3. Script determines maneuver complete (position reached, time elapsed, etc.)
 *          4. Script calls nav_script_time_done(id)
 *          5. This function validates id and disables nav_scripting
 *          6. verify_nav_script_time() returns true (mission proceeds)
 *          
 *          Command ID Validation:
 *          - Compares provided id with nav_scripting.id
 *          - nav_scripting.id incremented each time new NAV_SCRIPT_TIME starts
 *          - Ensures script completing correct command instance
 *          - Prevents stale script from completing wrong command
 *          
 *          Why ID Validation is Critical:
 *          
 *          Race Condition Prevention:
 *          - Script may run asynchronously from mission execution
 *          - Mission might advance to new NAV_SCRIPT_TIME while script finishing
 *          - Without ID check: Old script could complete new command
 *          - With ID check: Only matching ID completes command
 *          
 *          Example Scenario:
 *          1. Mission command A starts (id=5)
 *          2. Script A begins executing
 *          3. Mission times out command A (id=5), proceeds to command B (id=6)
 *          4. Script A finishes, calls nav_script_time_done(5)
 *          5. ID mismatch (5 != 6): Command B not affected
 *          6. Script B must independently complete command B
 *          
 *          Completion Effect:
 *          - nav_scripting.enabled = false
 *          - Signals scripting control should end
 *          - verify_nav_script_time() checks !nav_scripting.enabled
 *          - Mission system advances to next command
 *          - Autopilot resumes normal control
 *          
 *          ID Mismatch Behavior:
 *          - id != nav_scripting.id: No action taken
 *          - nav_scripting.enabled remains true
 *          - Mission continues waiting for timeout or correct completion
 *          - Script's completion call ignored silently
 *          - No error message (prevents log spam from old scripts)
 *          
 *          Typical Script Usage:
 *          ```lua
 *          local script_id, cmd, arg1, arg2, arg3, arg4 = plane:nav_script_time()
 *          
 *          -- Execute maneuver
 *          perform_aerobatic_sequence(cmd, arg1, arg2)
 *          
 *          -- Check completion
 *          if sequence_finished() then
 *              plane:nav_script_time_done(script_id)  -- Signal complete
 *          end
 *          ```
 *          
 *          Interaction with Verify Function:
 *          - verify_nav_script_time() checks !nav_scripting.enabled
 *          - Returns true when nav_scripting.enabled becomes false
 *          - Mission advances immediately on completion
 *          - No delay waiting for next verify cycle
 *          
 *          Alternative Completion Methods:
 *          - Timeout: verify_nav_script_time() times out after timeout_s
 *          - Mode Change: nav_scripting_active() disables if mode changes
 *          - Watchdog: nav_scripting_active() disables if no updates for 1000ms
 *          - Normal: This function for script-signaled completion
 * 
 * @param[in] id Command instance identifier from nav_script_time()
 *               Must match nav_scripting.id for completion to be accepted
 * 
 * @note Silently ignores completion if ID mismatch (prevents race conditions)
 * @note Only disables scripting if id matches current nav_scripting.id
 * @note Scripts must use id retrieved from nav_script_time()
 * @note Only compiled with AP_SCRIPTING_ENABLED
 * 
 * @see nav_script_time() to retrieve command ID
 * @see verify_nav_script_time() for completion detection
 * @see do_nav_script_time() where nav_scripting.id is incremented
 */
void Plane::nav_script_time_done(uint16_t id)
{
    if (id == nav_scripting.id) {
        nav_scripting.enabled = false;
    }
}

/**
 * @brief Set target throttle and roll/pitch/yaw rates for scripted flight control
 * 
 * @details Allows Lua scripts to command desired throttle and body-axis rotation
 *          rates during NAV_SCRIPT_TIME or other scripting-enabled modes. This is
 *          the primary control interface for aerobatic scripts, providing direct
 *          rate control for precise maneuver execution.
 *          
 *          Control Authority:
 *          - Scripts command desired throttle percentage
 *          - Scripts command desired body-frame rotation rates
 *          - Autopilot rate controllers track commanded rates
 *          - Control surfaces/motors automatically commanded
 *          
 *          Supported Modes:
 *          - AUTO with NAV_SCRIPT_TIME: Primary use case
 *          - CIRCLE, STABILIZE, ACRO: If nav_scripting_enable() called
 *          - FLY_BY_WIRE_A, FLY_BY_WIRE_B: If nav_scripting_enable() called
 *          - CRUISE, LOITER: If nav_scripting_enable() called
 *          
 *          Rate Control Frame:
 *          - Body frame rates (not earth frame)
 *          - Roll rate: Rotation about longitudinal (nose) axis (deg/s)
 *          - Pitch rate: Rotation about lateral (wing) axis (deg/s)
 *          - Yaw rate: Rotation about vertical axis (deg/s)
 *          - Positive directions follow right-hand rule
 *          
 *          Parameter Storage:
 *          - Stored in nav_scripting structure (auto_state member)
 *          - Values persist until next script update
 *          - Attitude controller reads from nav_scripting
 *          - Applied every control loop while scripting active
 *          
 *          Rate Limiting:
 *          - Roll rate constrained to ±g.acro_roll_rate
 *          - Pitch rate constrained to ±g.acro_pitch_rate
 *          - Yaw rate constrained to ±g.acro_yaw_rate
 *          - Throttle constrained to [aparm.throttle_min, aparm.throttle_max]
 *          - Prevents scripts from commanding unsafe rates
 *          
 *          ACRO Rate Parameters:
 *          - g.acro_roll_rate: Maximum roll rate (default 180 deg/s)
 *          - g.acro_pitch_rate: Maximum pitch rate (default 180 deg/s)
 *          - g.acro_yaw_rate: Maximum yaw rate (default 60 deg/s)
 *          - User-configurable via ground station
 *          - Conservative defaults for safety
 *          
 *          Throttle Percentage:
 *          - 0-100 scale (percentage of throttle range)
 *          - aparm.throttle_min: Minimum allowed (typically 0%)
 *          - aparm.throttle_max: Maximum allowed (typically 100%)
 *          - Applied to motor/ESC outputs via SRV_Channels
 *          
 *          Watchdog Update:
 *          - nav_scripting.current_ms updated to AP_HAL::millis()
 *          - Resets watchdog timeout in nav_scripting_active()
 *          - Must be called at least every 1000ms
 *          - If not called: scripting disabled automatically
 *          - Prevents runaway if script crashes or stalls
 *          
 *          Control Loop Integration:
 *          - Attitude controller checks nav_scripting_active()
 *          - If active: Uses nav_scripting.roll/pitch/yaw_rate_dps
 *          - Rate PID controllers track these target rates
 *          - Control surface deflections computed to achieve rates
 *          - Throttle output set directly from nav_scripting.throttle_pct
 *          
 *          Typical Script Usage Pattern:
 *          ```lua
 *          function update()
 *              if not nav_scripting_active() then
 *                  return
 *              end
 *              
 *              -- Calculate desired rates for current maneuver phase
 *              local throttle = 70  -- 70% throttle
 *              local roll_rate = 180  -- Full roll rate
 *              local pitch_rate = 0  -- Hold pitch
 *              local yaw_rate = 0  -- Coordinated
 *              
 *              -- Command rates (updates watchdog)
 *              plane:set_target_throttle_rate_rpy(throttle, roll_rate, pitch_rate, yaw_rate)
 *          end
 *          ```
 *          
 *          Aerobatic Maneuver Examples:
 *          
 *          Aileron Roll:
 *          - throttle_pct: 50-70 (maintain energy)
 *          - roll_rate_dps: ±180 (full rate)
 *          - pitch_rate_dps: 0 (minimize pitch change)
 *          - yaw_rate_dps: 0 (coordinated)
 *          
 *          Loop:
 *          - throttle_pct: 100 initially, reduce at top
 *          - roll_rate_dps: 0 (wings level)
 *          - pitch_rate_dps: 45-90 (pull rate)
 *          - yaw_rate_dps: 0 (coordinated)
 *          
 *          Knife Edge:
 *          - throttle_pct: Variable (maintain altitude)
 *          - roll_rate_dps: 0 (hold 90° bank)
 *          - pitch_rate_dps: 0 (maintain heading)
 *          - yaw_rate_dps: Variable (rudder for altitude)
 *          
 *          Safety Considerations:
 *          - Rate limits prevent structural damage
 *          - Throttle limits prevent motor/ESC damage
 *          - Watchdog prevents runaway if script fails
 *          - Mode restrictions prevent inadvertent activation
 *          - Scripts should implement altitude/attitude limits
 *          
 *          Coordination with Other Functions:
 *          - set_rudder_offset(): Direct rudder control override
 *          - nav_scripting_enable(): Enable in non-AUTO modes
 *          - nav_scripting_active(): Validates scripting still active
 * 
 * @param[in] throttle_pct   Desired throttle 0-100%, constrained to [throttle_min, throttle_max]
 * @param[in] roll_rate_dps  Desired roll rate in deg/s, constrained to ±acro_roll_rate
 * @param[in] pitch_rate_dps Desired pitch rate in deg/s, constrained to ±acro_pitch_rate
 * @param[in] yaw_rate_dps   Desired yaw rate in deg/s, constrained to ±acro_yaw_rate
 * 
 * @note Updates watchdog timer - must be called regularly (< 1000ms)
 * @note Rates are body-frame (roll about nose, pitch about wing, yaw about vertical)
 * @note Values automatically constrained to configured ACRO rate limits
 * @note Only compiled with AP_SCRIPTING_ENABLED
 * 
 * @see set_rudder_offset() for direct rudder control
 * @see nav_scripting_active() for watchdog validation
 * @see nav_scripting_enable() to enable in non-AUTO modes
 */
void Plane::set_target_throttle_rate_rpy(float throttle_pct, float roll_rate_dps, float pitch_rate_dps, float yaw_rate_dps)
{
    nav_scripting.roll_rate_dps = constrain_float(roll_rate_dps, -g.acro_roll_rate, g.acro_roll_rate);
    nav_scripting.pitch_rate_dps = constrain_float(pitch_rate_dps, -g.acro_pitch_rate, g.acro_pitch_rate);
    nav_scripting.yaw_rate_dps = constrain_float(yaw_rate_dps, -g.acro_yaw_rate, g.acro_yaw_rate);
    nav_scripting.throttle_pct = constrain_float(throttle_pct, aparm.throttle_min, aparm.throttle_max);
    nav_scripting.current_ms = AP_HAL::millis();
}

/**
 * @brief Set direct rudder control override for aerobatic scripting
 * 
 * @details Provides scripts with direct rudder control, optionally bypassing the
 *          yaw rate controller. This is critical for advanced aerobatic maneuvers
 *          requiring precise rudder coordination, such as knife-edge flight, spins,
 *          and snap rolls where direct rudder authority is needed.
 *          
 *          Control Modes:
 *          
 *          Direct Rudder Control (run_yaw_rate_controller = false):
 *          - Rudder commanded directly from rudder_pct
 *          - Yaw rate controller bypassed completely
 *          - Script has full rudder authority
 *          - Used for: Knife-edge, spins, snap rolls, sideslips
 *          - Script responsible for coordination
 *          
 *          Augmented Control (run_yaw_rate_controller = true):
 *          - Yaw rate controller still runs
 *          - rudder_pct added as offset to controller output
 *          - Combines automatic coordination with manual trim
 *          - Used for: Fine-tuning during rolls, loops
 *          - Autopilot assists with coordination
 *          
 *          Rudder Percentage Scale:
 *          - Range: -100 to +100
 *          - Positive: Right rudder deflection
 *          - Negative: Left rudder deflection
 *          - 0: Neutral position
 *          - Unconstrained (script must limit)
 *          
 *          Aerobatic Use Cases:
 *          
 *          Knife-Edge Flight:
 *          - run_yaw_rate_controller = false
 *          - rudder_pct: Variable (counters gravity with rudder)
 *          - Roll: 90° bank, maintain with ailerons
 *          - Pitch: Maintain with elevator
 *          - Rudder: Acts as elevator substitute to maintain altitude
 *          - Script calculates rudder from altitude error
 *          
 *          Spin:
 *          - run_yaw_rate_controller = false
 *          - rudder_pct: +100 or -100 (full pro-spin rudder)
 *          - Roll: Ailerons neutral or pro-spin
 *          - Pitch: Full aft stick (stall)
 *          - Yaw controller would fight spin (must bypass)
 *          
 *          Snap Roll:
 *          - run_yaw_rate_controller = false
 *          - rudder_pct: Full deflection direction of snap
 *          - Simultaneous full rudder, aileron, elevator
 *          - Induces rapid autorotation
 *          - Requires precise rudder timing
 *          
 *          Coordinated Roll:
 *          - run_yaw_rate_controller = true
 *          - rudder_pct: Small offset (±10-20%)
 *          - Yaw controller maintains coordination
 *          - Offset provides fine-tuning for asymmetry
 *          - Useful for perfect tracking in rolls
 *          
 *          Integration with Rate Control:
 *          - set_target_throttle_rate_rpy() commands roll/pitch/yaw rates
 *          - If run_yaw_rate_controller = true:
 *            * Yaw rate controller calculates rudder for yaw_rate_dps
 *            * rudder_offset_pct added to controller output
 *            * Final rudder = controller_output + offset
 *          - If run_yaw_rate_controller = false:
 *            * Yaw rate controller skipped
 *            * Rudder output directly from rudder_offset_pct
 *            * yaw_rate_dps target ignored
 *          
 *          Control Surface Mixing:
 *          - Rudder output applied to SRV_Channel::k_rudder
 *          - May also affect differential spoilers if configured
 *          - May affect differential thrust if configured
 *          - Respects SRV_Channel min/max/trim settings
 *          
 *          Typical Script Usage:
 *          ```lua
 *          -- Knife-edge flight example
 *          function knife_edge_right()
 *              local altitude_error = target_alt - current_alt
 *              local rudder = pid_controller:update(altitude_error)
 *              
 *              -- Direct rudder control, no yaw rate controller
 *              plane:set_rudder_offset(rudder, false)
 *              
 *              -- Hold 90° bank with rate control
 *              plane:set_target_throttle_rate_rpy(70, 0, 0, 0)
 *          end
 *          
 *          -- Coordinated roll with trim
 *          function perfect_roll()
 *              -- Roll with full rate
 *              plane:set_target_throttle_rate_rpy(60, 180, 0, 0)
 *              
 *              -- Add slight right rudder for asymmetry compensation
 *              -- Yaw controller still runs (coordinated)
 *              plane:set_rudder_offset(5, true)
 *          end
 *          ```
 *          
 *          Safety Considerations:
 *          - No automatic limiting of rudder_pct (script responsible)
 *          - Excessive rudder can cause sideslip or spin entry
 *          - run_yaw_rate_controller=false removes coordination safety
 *          - Scripts must implement their own limits and safety checks
 *          - Consider airspeed and G-loading when commanding rudder
 *          
 *          State Persistence:
 *          - nav_scripting.rudder_offset_pct stored until changed
 *          - nav_scripting.run_yaw_rate_controller stored until changed
 *          - Values persist across multiple control loop cycles
 *          - Reset when nav_scripting disabled
 *          
 *          Clearing Rudder Override:
 *          - Call set_rudder_offset(0, true) to return to normal
 *          - Automatic when nav_scripting_active() becomes false
 *          - Automatic when mode changes away from scripting
 *          - Automatic on watchdog timeout
 *          
 *          Interaction with Yaw Damper:
 *          - run_yaw_rate_controller = true: Yaw damper active
 *          - run_yaw_rate_controller = false: Yaw damper bypassed
 *          - May affect dutch roll damping and spiral stability
 *          - Scripts should consider stability augmentation needs
 * 
 * @param[in] rudder_pct             Rudder deflection percentage -100 to +100
 *                                    (positive = right rudder, unconstrained)
 * @param[in] run_yaw_rate_controller true: Add offset to yaw rate controller output
 *                                     false: Direct rudder control (bypass controller)
 * 
 * @note No automatic limiting - script must constrain rudder_pct
 * @note run_yaw_rate_controller=false disables yaw coordination
 * @note Values persist until changed or scripting disabled
 * @note Only compiled with AP_SCRIPTING_ENABLED
 * 
 * @warning Direct rudder control can induce spins or sideslips
 * @warning Scripts must implement safety checks for rudder limits
 * 
 * @see set_target_throttle_rate_rpy() for rate control
 * @see nav_scripting_active() for scripting state validation
 */
void Plane::set_rudder_offset(float rudder_pct, bool run_yaw_rate_controller)
{
    nav_scripting.rudder_offset_pct = rudder_pct;
    nav_scripting.run_yaw_rate_controller = run_yaw_rate_controller;
}

/**
 * @brief Enable scripted control takeover in non-AUTO flight modes
 * 
 * @details Allows Lua scripts to take control of the aircraft in specific flight
 *          modes outside of AUTO mode NAV_SCRIPT_TIME commands. This enables
 *          aerobatic scripts to be executed on-demand in manual or assisted flight
 *          modes while maintaining mode-specific safety constraints.
 *          
 *          Normal Scripting Context:
 *          - AUTO mode: NAV_SCRIPT_TIME mission command enables scripting
 *          - do_nav_script_time() sets nav_scripting.enabled = true
 *          - Mission-based activation, automatic progression
 *          
 *          Extended Scripting Context:
 *          - Other modes: Script calls nav_scripting_enable() explicitly
 *          - Enables aerobatics in STABILIZE, ACRO, LOITER, etc.
 *          - Script-initiated activation, script-controlled duration
 *          - Allows training, practice, or manual aerobatic execution
 *          
 *          Mode Validation:
 *          - Checks if current_control_mode matches requested mode
 *          - Only enables if mode match confirmed
 *          - Prevents scripts from assuming wrong mode active
 *          - Returns current enabled state
 *          
 *          Allowed Modes for Scripting Takeover:
 *          
 *          Mode::Number::CIRCLE:
 *          - Normally circles around point
 *          - Script can take over for custom circling patterns
 *          - Example: Helical climb, figure-8 around point
 *          
 *          Mode::Number::STABILIZE:
 *          - Normally stabilizes attitude based on pilot input
 *          - Script can automate maneuvers while pilot maintains override
 *          - Example: Auto-level after aerobatic maneuver
 *          
 *          Mode::Number::ACRO:
 *          - Normally rate control from pilot stick
 *          - Script can execute programmed aerobatic sequences
 *          - Example: Competition aerobatic sequence on command
 *          
 *          Mode::Number::FLY_BY_WIRE_A:
 *          - Normally stabilized altitude/attitude
 *          - Script can automate maneuvers with safety limits
 *          - Example: Controlled aerobatics with altitude floor
 *          
 *          Mode::Number::FLY_BY_WIRE_B:
 *          - Normally altitude/speed hold
 *          - Script can perform altitude-aware maneuvers
 *          - Example: Terrain-following aerobatics
 *          
 *          Mode::Number::CRUISE:
 *          - Normally maintains heading and altitude
 *          - Script can execute gentle maneuvers
 *          - Example: Automated photo survey patterns
 *          
 *          Mode::Number::LOITER:
 *          - Normally loiters at fixed point
 *          - Script can perform custom loiter variations
 *          - Example: Spiral climb/descent patterns
 *          
 *          Prohibited Modes:
 *          - AUTO: Use NAV_SCRIPT_TIME mission command instead
 *          - RTL, QRTL: Safety-critical return modes (no scripting)
 *          - QLAND, QLOITER: Quadplane landing/loiter (no scripting)
 *          - GUIDED: Dedicated command interface (conflicts with scripting)
 *          - MANUAL: No stabilization (too dangerous for scripting)
 *          - TRAINING: Learning mode (should not be automated)
 *          - INITIALISING: System startup (not ready for scripting)
 *          
 *          Mode Number Validation:
 *          - current_control_mode: Active flight mode number
 *          - mode parameter: Requested mode number for scripting
 *          - Must match exactly for security
 *          - Prevents script from enabling in wrong mode
 *          
 *          Why Mode Matching is Required:
 *          - Script may assume specific mode behaviors
 *          - Different modes have different safety characteristics
 *          - Mode change during script execution could be dangerous
 *          - Prevents inadvertent activation during mode transitions
 *          
 *          Typical Script Usage Pattern:
 *          ```lua
 *          -- Check if in ACRO mode and enable scripting
 *          local ACRO = 4
 *          if plane:nav_scripting_enable(ACRO) then
 *              -- Now in scripting control
 *              plane:set_target_throttle_rate_rpy(70, 180, 0, 0)
 *          else
 *              -- Not in ACRO or scripting not allowed
 *              -- Return control to pilot/autopilot
 *          end
 *          ```
 *          
 *          Activation Procedure:
 *          1. Script checks current mode number
 *          2. Script calls nav_scripting_enable(mode_number)
 *          3. Function validates mode is current and allowed
 *          4. If valid: Sets nav_scripting.enabled = true
 *          5. Updates watchdog timer (nav_scripting.current_ms)
 *          6. Returns true (scripting now active)
 *          7. Script can now use set_target_throttle_rate_rpy()
 *          
 *          Deactivation:
 *          - Mode change: Automatically disables scripting
 *          - Call nav_scripting_enable() with different mode: Returns false
 *          - Watchdog timeout: 1000ms without set_target_throttle_rate_rpy()
 *          - Script can set nav_scripting.enabled = false directly
 *          
 *          Safety Considerations:
 *          - Mode restrictions limit potential for misuse
 *          - Mode validation prevents wrong-mode activation
 *          - Watchdog timeout protects against script hang
 *          - Pilot can override by switching modes
 *          - Some modes (RTL, QLAND) protected from scripting
 *          
 *          Interaction with Mission Scripting:
 *          - AUTO mode: Use NAV_SCRIPT_TIME, not this function
 *          - This function is for non-mission scripting only
 *          - Allows aerobatics independent of mission execution
 *          - Different use cases, different activation methods
 *          
 *          Watchdog Timer Initialization:
 *          - nav_scripting.current_ms = AP_HAL::millis()
 *          - Starts watchdog countdown
 *          - Script must call set_target_throttle_rate_rpy() regularly
 *          - If not updated within 1000ms: nav_scripting_active() returns false
 * 
 * @param[in] mode Requested flight mode number (Mode::Number enum)
 *                 Must match current mode for activation
 * 
 * @return true if scripting enabled in requested mode
 * @return false if mode mismatch, mode not allowed, or validation failed
 * 
 * @note Only enables in whitelisted modes (CIRCLE, STABILIZE, ACRO, FBWA, FBWB, CRUISE, LOITER)
 * @note Mode must match current flight mode exactly
 * @note Initializes watchdog timer on successful enable
 * @note Only compiled with AP_SCRIPTING_ENABLED
 * 
 * @see nav_scripting_active() for validation and watchdog
 * @see set_target_throttle_rate_rpy() for commanding control
 * @see do_nav_script_time() for mission-based scripting (AUTO mode)
 */
bool Plane::nav_scripting_enable(uint8_t mode)
{
   uint8_t current_control_mode = control_mode->mode_number();
   if (current_control_mode == mode) {
       switch (current_control_mode) {
       case Mode::Number::CIRCLE:
       case Mode::Number::STABILIZE:
       case Mode::Number::ACRO:
       case Mode::Number::FLY_BY_WIRE_A:
       case Mode::Number::FLY_BY_WIRE_B:
       case Mode::Number::CRUISE:
       case Mode::Number::LOITER:
           nav_scripting.enabled = true;
           nav_scripting.current_ms = AP_HAL::millis();
           break;
       default:
           nav_scripting.enabled = false;
       }
   } else {
       nav_scripting.enabled = false;
   }
   return nav_scripting.enabled;
}
#endif // AP_SCRIPTING_ENABLED

/**
 * @brief Check if mission command is a landing command
 * 
 * @details Determines whether a MAVLink mission command ID represents a landing
 *          operation. This is used throughout the codebase to apply landing-specific
 *          logic such as approach configuration, descent profiles, and safety checks.
 *          
 *          Landing Command Types:
 *          
 *          MAV_CMD_NAV_VTOL_LAND:
 *          - Quadplane vertical landing
 *          - Transitions to VTOL mode if not already
 *          - Descends vertically to ground
 *          - Primary landing method for quadplanes
 *          
 *          MAV_CMD_NAV_LAND:
 *          - Fixed-wing landing approach and flare
 *          - Can also trigger quadplane VTOL landing (if configured)
 *          - Follows approach path to runway/landing point
 *          - Executes landing flare near ground
 *          
 *          MAV_CMD_NAV_PAYLOAD_PLACE:
 *          - Quadplane landing with payload deployment
 *          - Lands like VTOL_LAND but with payload release
 *          - Considered landing for flight logic purposes
 *          - Follows same descent and safety profiles
 *          
 *          Why PAYLOAD_PLACE is Considered Landing:
 *          - Uses identical descent and approach logic
 *          - Same safety considerations (low altitude, ground proximity)
 *          - Requires same pre-landing checks and configuration
 *          - Pilot monitoring and abort procedures same as landing
 *          - Ground contact expected (even if brief for drop-and-go)
 *          
 *          Usage Throughout Codebase:
 *          - Landing gear deployment logic
 *          - Approach speed configuration
 *          - Descent rate monitoring
 *          - Fence disable during landing approach
 *          - Flare and ground effect compensation
 *          - Touch-down detection
 *          - Disarm after landing logic
 *          
 *          Example Usage Patterns:
 *          ```cpp
 *          if (plane.is_land_command(mission.get_current_nav_cmd().id)) {
 *              // Apply landing configuration
 *              deploy_landing_gear();
 *              reduce_approach_speed();
 *              arm_landing_flare();
 *          }
 *          ```
 *          
 *          Mission Planning Implications:
 *          - These commands should be final or near-final in mission
 *          - May trigger automatic fence disable
 *          - May enable special telemetry reporting
 *          - Landing checks applied before command execution
 *          
 *          Not Considered Landing:
 *          - MAV_CMD_NAV_LOITER_TO_ALT: Can descend but not landing
 *          - MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT: Altitude change only
 *          - Altitude wait commands: May descend but intentional landing
 *          
 *          Safety Considerations:
 *          - Landing commands have different abort criteria
 *          - May have reduced fence protection
 *          - Pilot override more critical during landing
 *          - Terrain collision avoidance may be modified
 * 
 * @param[in] command MAVLink mission command ID to check
 * 
 * @return true if command is MAV_CMD_NAV_VTOL_LAND, MAV_CMD_NAV_LAND, or MAV_CMD_NAV_PAYLOAD_PLACE
 * @return false for all other command IDs
 * 
 * @note PAYLOAD_PLACE included because it follows landing logic for quadplanes
 * @note This is const - does not modify vehicle state
 * 
 * @see do_land() for fixed-wing landing initialization
 * @see quadplane.do_vtol_land() for VTOL landing initialization
 * @see in_auto_mission_id() to check if currently executing specific command
 */
bool Plane::is_land_command(uint16_t command) const
{
    return
        command == MAV_CMD_NAV_VTOL_LAND ||
        command == MAV_CMD_NAV_LAND ||
        command == MAV_CMD_NAV_PAYLOAD_PLACE;
}

/**
 * @brief Check if currently executing a specific AUTO mission command
 * 
 * @details Determines whether the aircraft is in AUTO mode AND currently executing
 *          a navigation command with the specified MAVLink command ID. This provides
 *          a convenient single check for mode-and-command-specific logic throughout
 *          the codebase.
 *          
 *          Two Conditions Must Be Met:
 *          1. control_mode == &mode_auto (in AUTO flight mode)
 *          2. mission.get_current_nav_id() == command (executing specified nav command)
 *          
 *          Why Both Checks Are Required:
 *          - Mission system can be running in other modes (e.g., GUIDED with mission)
 *          - Command-specific behavior should only apply in AUTO mode
 *          - Prevents unintended activation during mode transitions
 *          - Other modes may use mission system for different purposes
 *          
 *          Navigation Command vs DO Command:
 *          - Uses get_current_nav_id() not get_current_do_id()
 *          - Only checks NAV commands (waypoints, takeoff, land, loiter, etc.)
 *          - DO commands (immediate actions) not checked by this function
 *          - NAV commands define where/how aircraft moves
 *          - DO commands trigger auxiliary actions (camera, servo, etc.)
 *          
 *          Common Usage Patterns:
 *          
 *          Check if Landing:
 *          ```cpp
 *          if (plane.in_auto_mission_id(MAV_CMD_NAV_LAND)) {
 *              // Apply landing-specific logic only in AUTO landing
 *              configure_landing_parameters();
 *          }
 *          ```
 *          
 *          Check if in Specific Loiter:
 *          ```cpp
 *          if (plane.in_auto_mission_id(MAV_CMD_NAV_LOITER_TIME)) {
 *              // Loiter time specific behavior
 *              update_loiter_timer();
 *          }
 *          ```
 *          
 *          Mode-Specific Behavior:
 *          ```cpp
 *          if (plane.in_auto_mission_id(MAV_CMD_NAV_VTOL_LAND)) {
 *              // Only in AUTO VTOL landing, not manual QLAND
 *              mission_vtol_land_telemetry();
 *          }
 *          ```
 *          
 *          Why Not Just Check Mission Command ID:
 *          - Mission system active in multiple modes:
 *            * AUTO: Full mission execution (this is what we want)
 *            * GUIDED: May reference mission for rally points
 *            * RTL: Uses mission system for rally point selection
 *          - Checking only command ID would trigger in wrong modes
 *          - Control mode check ensures AUTO-specific behavior
 *          
 *          Typical Use Cases:
 *          
 *          Landing Gear Timing:
 *          - Deploy gear only during AUTO landing
 *          - Not during manual QLAND or RTL landing
 *          - Ensures mission-specified gear deployment
 *          
 *          Telemetry Reporting:
 *          - Mission progress reporting
 *          - Command-specific telemetry streams
 *          - Ground station mission monitoring
 *          
 *          Safety Checks:
 *          - Apply command-specific safety limits
 *          - Enable/disable features for specific mission phases
 *          - Example: Allow aggressive maneuvering in NAV_SCRIPT_TIME
 *          
 *          Camera/Payload Control:
 *          - Trigger camera at specific mission command
 *          - Coordinate payload actions with mission progress
 *          - Ensure actions occur at right mission point
 *          
 *          Alternative to This Function:
 *          - Could check both conditions separately each time
 *          - This function provides cleaner, more readable code
 *          - Reduces chance of forgetting one of the checks
 *          - Centralizes the logic for consistency
 *          
 *          Example Without This Function:
 *          ```cpp
 *          // More verbose, easier to get wrong
 *          if (control_mode == &mode_auto && 
 *              mission.get_current_nav_id() == MAV_CMD_NAV_LAND) {
 *              // Do something
 *          }
 *          ```
 *          
 *          Example With This Function:
 *          ```cpp
 *          // Cleaner, harder to get wrong
 *          if (plane.in_auto_mission_id(MAV_CMD_NAV_LAND)) {
 *              // Do something
 *          }
 *          ```
 *          
 *          Related Mode Checks:
 *          - control_mode == &mode_auto: In AUTO mode (any mission command)
 *          - mission.get_current_nav_id(): Current nav command (any mode)
 *          - in_auto_mission_id(): Both AUTO mode AND specific command
 *          
 *          Performance Considerations:
 *          - Very lightweight check (two pointer/value comparisons)
 *          - No computation or state changes
 *          - Safe to call frequently (every loop cycle if needed)
 *          - Marked const (does not modify state)
 *          
 *          Mission State Machine Context:
 *          - Mission has current nav command (ongoing navigation)
 *          - Mission may have current do command (immediate action)
 *          - This checks nav command only
 *          - Nav command changes when waypoint reached/command complete
 *          - DO commands execute instantly and don't affect this check
 * 
 * @param[in] command MAVLink NAV command ID to check against current mission command
 * 
 * @return true if in AUTO mode AND current navigation command matches specified command
 * @return false if not in AUTO mode OR current nav command is different
 * 
 * @note Only checks NAV commands, not DO commands
 * @note Both AUTO mode and command match required for true result
 * @note This is const - does not modify vehicle state
 * @note Very lightweight check, safe to call frequently
 * 
 * @see mission.get_current_nav_id() for current navigation command ID
 * @see mission.get_current_nav_cmd() for full navigation command structure
 * @see is_land_command() to check if command is any type of landing
 */
bool Plane::in_auto_mission_id(uint16_t command) const
{
    return control_mode == &mode_auto && mission.get_current_nav_id() == command;
}

