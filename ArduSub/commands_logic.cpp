/**
 * @file commands_logic.cpp
 * @brief Mission command execution and verification logic for ArduSub underwater vehicle
 * 
 * @details This file implements the command processing pipeline for autonomous mission
 *          execution in ArduSub. It handles the initiation, execution, and verification
 *          of MAVLink mission commands, adapting standard autopilot behaviors for the
 *          underwater environment.
 *          
 *          Key responsibilities:
 *          - Command validation and pre-flight checks specific to underwater operations
 *          - Command execution (do_* functions) that initiate requested behaviors
 *          - Command verification (verify_* functions) that check completion status
 *          - State management for complex multi-step commands (e.g., surface procedure)
 *          - Underwater-specific adaptations (depth vs altitude, buoyancy considerations)
 *          
 *          Command Processing Flow:
 *          1. AP_Mission library calls start_command() with new command
 *          2. start_command() validates command and dispatches to appropriate do_* function
 *          3. do_* function initiates the requested behavior (waypoint, loiter, surface, etc.)
 *          4. AP_Mission repeatedly calls verify_command_callback() to check progress
 *          5. verify_command() dispatches to appropriate verify_* function
 *          6. verify_* function returns true when command objective is achieved
 *          7. exit_mission() handles cleanup when entire mission completes
 *          
 *          Underwater Adaptations:
 *          - Altitude frame validation: ABOVE_HOME must be negative (below surface)
 *          - ABOVE_TERRAIN must be positive (above seafloor)
 *          - Surface command implements multi-stage ascent procedure
 *          - Depth hold mode used for mission end instead of loiter
 *          - Coordinate system: NED frame (North-East-Down), depth is positive down
 *          
 * @note This file is vehicle-specific. See ArduCopter/commands_logic.cpp for aerial
 *       vehicle implementation and Rover/commands_logic.cpp for ground vehicle version.
 * 
 * @warning Incorrect command execution can lead to loss of vehicle. All navigation
 *          commands must respect depth limits and consider buoyancy effects.
 * 
 * Source: ArduSub/commands_logic.cpp:1-702
 */

#include "Sub.h"

#include <AP_RTC/AP_RTC.h>

/// @brief State machine for multi-stage surface procedure
static enum AutoSurfaceState auto_surface_state = AUTO_SURFACE_STATE_GO_TO_LOCATION;

/**
 * @brief Initiates execution of a new mission command
 * 
 * @details Called by the AP_Mission library when beginning a new command during
 *          autonomous mission execution. This function performs underwater-specific
 *          validation checks and dispatches to the appropriate command handler.
 *          
 *          Validation Process:
 *          1. Check altitude frame type (ABOVE_HOME, ABOVE_TERRAIN, or other)
 *          2. For ABOVE_HOME: Verify altitude is negative (below surface)
 *          3. For ABOVE_TERRAIN: Verify altitude is positive (above seafloor)
 *          4. Reject invalid altitude frames
 *          5. Dispatch to command-specific handler based on cmd.id
 *          
 *          Supported Command Categories:
 *          - Navigation (NAV): Waypoint, Land/Surface, RTL, Loiter, Circle, Guided, Delay
 *          - Conditional (CONDITION): Delay, Distance, Yaw
 *          - Immediate (DO): Speed, Home, ROI, Mount, Guided Limits
 *          
 *          Underwater-Specific Behavior:
 *          - NAV_LAND command triggers surface procedure (ascend to zero depth)
 *          - Altitude validation ensures vehicle doesn't try to fly above surface
 *          - Depth is managed through negative ABOVE_HOME altitudes
 * 
 * @param[in] cmd Mission command structure containing command ID, parameters, and location
 * 
 * @return true if command was accepted and handler initiated successfully
 * @return false if command validation failed or command ID not recognized
 * 
 * @note This function is called from AP_Mission::update() during AUTO mode
 * @note Unrecognized commands are logged and skipped (returns false)
 * 
 * @warning Altitude frame validation is critical for underwater safety. Attempting
 *          to command positive altitude in ABOVE_HOME frame could cause vehicle to
 *          breach surface unexpectedly.
 * 
 * Source: ArduSub/commands_logic.cpp:8-120
 */
bool Sub::start_command(const AP_Mission::Mission_Command& cmd)
{
    const Location &target_loc = cmd.content.location;
    auto alt_frame = target_loc.get_alt_frame();

    if (alt_frame == Location::AltFrame::ABOVE_HOME) {
        if (target_loc.alt > 0) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Alt above home must be negative");
            return false;
        }
    } else if (alt_frame == Location::AltFrame::ABOVE_TERRAIN) {
        if (target_loc.alt < 0) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Alt above terrain must be positive");
            return false;
        }
    } else {
        gcs().send_text(MAV_SEVERITY_WARNING, "Bad alt frame");
        return false;
    }

    switch (cmd.id) {

        ///
        /// navigation commands
        ///
    case MAV_CMD_NAV_WAYPOINT:                  // 16  Navigate to Waypoint
        do_nav_wp(cmd);
        break;

    case MAV_CMD_NAV_LAND:              // 21 LAND to Waypoint
        do_surface(cmd);
        break;

    case MAV_CMD_NAV_RETURN_TO_LAUNCH:
        do_RTL();
        break;

    case MAV_CMD_NAV_LOITER_UNLIM:              // 17 Loiter indefinitely
        do_loiter_unlimited(cmd);
        break;

    case MAV_CMD_NAV_LOITER_TURNS:              //18 Loiter N Times
        do_circle(cmd);
        break;

    case MAV_CMD_NAV_LOITER_TIME:              // 19
        do_loiter_time(cmd);
        break;

#if NAV_GUIDED
    case MAV_CMD_NAV_GUIDED_ENABLE:             // 92  accept navigation commands from external nav computer
        do_nav_guided_enable(cmd);
        break;
#endif

    case MAV_CMD_NAV_DELAY:                    // 93 Delay the next navigation command
        do_nav_delay(cmd);
        break;

        //
        // conditional commands
        //
    case MAV_CMD_CONDITION_DELAY:             // 112
        do_wait_delay(cmd);
        break;

    case MAV_CMD_CONDITION_DISTANCE:             // 114
        do_within_distance(cmd);
        break;

    case MAV_CMD_CONDITION_YAW:             // 115
        do_yaw(cmd);
        break;

        ///
        /// do commands
        ///
    case MAV_CMD_DO_CHANGE_SPEED:             // 178
        do_change_speed(cmd);
        break;

    case MAV_CMD_DO_SET_HOME:             // 179
        do_set_home(cmd);
        break;

    case MAV_CMD_DO_SET_ROI_LOCATION:       // 195
    case MAV_CMD_DO_SET_ROI_NONE:           // 197
    case MAV_CMD_DO_SET_ROI:                // 201
        // point the vehicle and camera at a region of interest (ROI)
        // ROI_NONE can be handled by the regular ROI handler because lat, lon, alt are always zero
        do_roi(cmd);
        break;

    case MAV_CMD_DO_MOUNT_CONTROL:          // 205
        // point the camera to a specified angle
        do_mount_control(cmd);
        break;

#if NAV_GUIDED
    case MAV_CMD_DO_GUIDED_LIMITS:                      // 222  accept guided mode limits
        do_guided_limits(cmd);
        break;
#endif

    default:
        // unable to use the command, allow the vehicle to try the next command
        gcs().send_text(MAV_SEVERITY_WARNING, "Ignoring command %d", cmd.id);
        return false;
    }

    // always return success
    return true;
}

/********************************************************************************/
// Verify Command Handlers
/********************************************************************************/

/**
 * @brief Callback function to check if current mission command has completed
 * 
 * @details Called repeatedly by AP_Mission::update() to poll command completion status.
 *          This wrapper function ensures verification only occurs in AUTO mode and
 *          sends telemetry notification when commands complete.
 *          
 *          Execution Flow:
 *          1. Check vehicle is in AUTO mode (only mode that executes missions)
 *          2. Call verify_command() to check actual command completion status
 *          3. If command complete, send MAVLink MISSION_ITEM_REACHED message to GCS
 *          4. Return completion status to AP_Mission library
 *          
 *          Verification Frequency:
 *          - Called at main loop rate (typically 50Hz for Sub)
 *          - High frequency allows precise detection of completion conditions
 *          - Each command type implements its own completion criteria
 * 
 * @param[in] cmd Mission command being verified
 * 
 * @return true if command has completed successfully (allows mission to advance)
 * @return false if command still in progress or not in AUTO mode
 * 
 * @note This is the entry point called by AP_Mission library, not called directly
 *       by vehicle code. Internal verification logic is in verify_command().
 * 
 * Source: ArduSub/commands_logic.cpp:128-141
 */
bool Sub::verify_command_callback(const AP_Mission::Mission_Command& cmd)
{
    if (control_mode == Mode::Number::AUTO) {
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
 * @brief Dispatches command verification to appropriate handler function
 * 
 * @details Main verification dispatcher that routes command completion checking to
 *          command-specific verify_* functions. Different command types have different
 *          completion criteria (position reached, time elapsed, condition met, etc.).
 *          
 *          Verification Categories:
 *          
 *          Navigation Commands (must complete before mission advances):
 *          - NAV_WAYPOINT: Position reached and loiter time elapsed
 *          - NAV_LAND: Surface procedure completed (at zero depth)
 *          - NAV_RETURN_TO_LAUNCH: Returned to home position
 *          - NAV_LOITER_UNLIM: Never completes (infinite loiter)
 *          - NAV_LOITER_TURNS: Specified number of circles completed
 *          - NAV_LOITER_TIME: Loiter duration elapsed
 *          - NAV_GUIDED_ENABLE: Guided mode limits not breached
 *          - NAV_DELAY: Delay time elapsed or absolute time reached
 *          
 *          Conditional Commands (gate execution of subsequent commands):
 *          - CONDITION_DELAY: Wait time elapsed
 *          - CONDITION_DISTANCE: Within specified distance to target
 *          - CONDITION_YAW: Desired heading achieved
 *          
 *          DO Commands (immediate actions):
 *          - Always return true immediately (execute and continue)
 *          - Examples: DO_CHANGE_SPEED, DO_SET_HOME, DO_SET_ROI, DO_MOUNT_CONTROL
 *          
 *          Error Handling:
 *          - Unrecognized commands: Log warning and return true (skip to next command)
 *          - Invalid commands: Better to continue mission than abort
 * 
 * @param[in] cmd Mission command being verified
 * 
 * @return true if command has achieved its objective and mission can advance
 * @return false if command still in progress (keep executing current command)
 * 
 * @note Called at high frequency (50Hz) from verify_command_callback()
 * @note DO commands return true immediately as they execute instantly
 * @note NAV_LOITER_UNLIM returns false indefinitely (infinite loiter)
 * 
 * @warning Returning true prematurely will cause mission to skip to next command
 *          before current command objective is achieved
 * 
 * Source: ArduSub/commands_logic.cpp:145-206
 */
bool Sub::verify_command(const AP_Mission::Mission_Command& cmd)
{
    switch (cmd.id) {
        //
        // navigation commands
        //
    case MAV_CMD_NAV_WAYPOINT:
        return verify_nav_wp(cmd);

    case MAV_CMD_NAV_LAND:
        return verify_surface(cmd);

    case MAV_CMD_NAV_RETURN_TO_LAUNCH:
        return verify_RTL();

    case MAV_CMD_NAV_LOITER_UNLIM:
        return verify_loiter_unlimited();

    case MAV_CMD_NAV_LOITER_TURNS:
        return verify_circle(cmd);

    case MAV_CMD_NAV_LOITER_TIME:
        return verify_loiter_time();

#if NAV_GUIDED
    case MAV_CMD_NAV_GUIDED_ENABLE:
        return verify_nav_guided_enable(cmd);
#endif

    case MAV_CMD_NAV_DELAY:
        return verify_nav_delay(cmd);

        ///
        /// conditional commands
        ///
    case MAV_CMD_CONDITION_DELAY:
        return verify_wait_delay();

    case MAV_CMD_CONDITION_DISTANCE:
        return verify_within_distance();

    case MAV_CMD_CONDITION_YAW:
        return verify_yaw();

        // do commands (always return true)
    case MAV_CMD_DO_CHANGE_SPEED:
    case MAV_CMD_DO_SET_HOME:
    case MAV_CMD_DO_SET_ROI_LOCATION:
    case MAV_CMD_DO_SET_ROI_NONE:
    case MAV_CMD_DO_SET_ROI:
    case MAV_CMD_DO_MOUNT_CONTROL:
    case MAV_CMD_DO_SET_CAM_TRIGG_DIST:
    case MAV_CMD_DO_GUIDED_LIMITS:
        return true;

    default:
        // error message
        gcs().send_text(MAV_SEVERITY_WARNING,"Skipping invalid cmd #%i",cmd.id);
        // return true if we do not recognize the command so that we move on to the next command
        return true;
    }
}

/**
 * @brief Handles vehicle state when autonomous mission completes
 * 
 * @details Called by AP_Mission library when the last command in the mission completes
 *          or when the mission is stopped. Implements safe fallback behavior to prevent
 *          vehicle from drifting after mission end.
 *          
 *          Mission End Sequence:
 *          1. Trigger mission complete notification (tone/LED)
 *          2. Attempt to enter AUTO Loiter mode (maintain position)
 *          3. If loiter start fails, fall back to ALT_HOLD (depth hold)
 *          
 *          Underwater-Specific Behavior:
 *          - Loiter mode maintains horizontal position and depth
 *          - ALT_HOLD fallback maintains depth but allows horizontal drift
 *          - Depth hold is safer than ascending for underwater operations
 *          - Prevents vehicle from floating to surface on mission end
 *          
 *          Safety Considerations:
 *          - Vehicle remains underwater at last commanded depth
 *          - Operator intervention required to return to surface
 *          - Prevents uncontrolled ascent that could occur with aerial vehicles
 * 
 * @note Called automatically by AP_Mission when mission completes
 * @note Mode transition logged with ModeReason::MISSION_END
 * @note AP_Notify::events.mission_complete triggers audible/visual notification
 * 
 * @warning Vehicle will remain at depth after mission completion. Operator must
 *          manually command surface or RTL if retrieval needed.
 * 
 * Source: ArduSub/commands_logic.cpp:209-218
 */
void Sub::exit_mission()
{
    // play a tone
    AP_Notify::events.mission_complete = 1;

    // Try to enter loiter, if that fails, go to depth hold
    if (!mode_auto.auto_loiter_start()) {
        set_mode(Mode::Number::ALT_HOLD, ModeReason::MISSION_END);
    }
}

/********************************************************************************/
//  Nav (Must) Commands
/********************************************************************************/

/**
 * @brief Initiates navigation to a waypoint location
 * 
 * @details Processes MAV_CMD_NAV_WAYPOINT command to navigate the vehicle to a specified
 *          3D position (latitude, longitude, depth). Handles default values and altitude
 *          frame conversions for underwater operation.
 *          
 *          Waypoint Processing Algorithm:
 *          1. Copy command location to target_loc
 *          2. If lat/lon are zero, use current horizontal position (vertical move only)
 *          3. If altitude is zero, use current depth in command's altitude frame
 *          4. Set loiter time from cmd.p1 (time to hold position at waypoint)
 *          5. Initialize waypoint navigation via mode_auto.auto_wp_start()
 *          
 *          Default Value Handling:
 *          - Zero lat/lon: Stay at current horizontal position (depth change only)
 *          - Zero altitude: Maintain current depth
 *          - Frame conversion: Ensures depth interpreted correctly regardless of frame
 *          
 *          Loiter Behavior:
 *          - loiter_time_max set from cmd.p1 (seconds to hold at waypoint)
 *          - If loiter_time_max is 0, waypoint completes immediately upon arrival
 *          - If loiter_time_max > 0, must hold position for specified duration
 *          
 *          Coordinate System:
 *          - Horizontal: Latitude/longitude (WGS84)
 *          - Vertical: Depth in NED frame (down is positive)
 *          - Altitude frames: ABOVE_HOME (negative for underwater) or ABOVE_TERRAIN
 * 
 * @param[in] cmd Mission command containing target location and loiter time (p1)
 * 
 * @note Called from start_command() when MAV_CMD_NAV_WAYPOINT is received
 * @note Waypoint is "accepted" when within acceptance radius (configurable parameter)
 * @note Completion verified by verify_nav_wp() checking position and loiter time
 * 
 * @see verify_nav_wp() for completion criteria
 * @see mode_auto.auto_wp_start() for navigation controller initialization
 * 
 * Source: ArduSub/commands_logic.cpp:225-252
 */
void Sub::do_nav_wp(const AP_Mission::Mission_Command& cmd)
{
    Location target_loc(cmd.content.location);
    // use current lat, lon if zero
    if (target_loc.lat == 0 && target_loc.lng == 0) {
        target_loc.lat = current_loc.lat;
        target_loc.lng = current_loc.lng;
    }
    // use current altitude if not provided
    if (target_loc.alt == 0) {
        // set to current altitude but in command's alt frame
        int32_t curr_alt;
        if (current_loc.get_alt_cm(target_loc.get_alt_frame(),curr_alt)) {
            target_loc.set_alt_cm(curr_alt, target_loc.get_alt_frame());
        } else {
            // default to current altitude as alt-above-home
            target_loc.set_alt_cm(current_loc.alt, current_loc.get_alt_frame());
        }
    }

    // this will be used to remember the time in millis after we reach or pass the WP.
    loiter_time = 0;
    // this is the delay, stored in seconds
    loiter_time_max = cmd.p1;

    // Set wp navigation target
    mode_auto.auto_wp_start(target_loc);
}

/**
 * @brief Initiates multi-stage procedure to ascend vehicle to the surface
 * 
 * @details Implements MAV_CMD_NAV_LAND for underwater vehicles, adapted to perform
 *          controlled ascent to zero depth (water surface). Supports both direct
 *          ascent and lateral movement followed by ascent.
 *          
 *          Surface Procedure Variants:
 *          
 *          Variant 1 - Surface with Lateral Movement:
 *          If lat/lon specified in command:
 *          - State: AUTO_SURFACE_STATE_GO_TO_LOCATION
 *          - Step 1: Navigate horizontally to target lat/lon at current depth
 *          - Step 2: Once position reached, ascend vertically to zero depth
 *          - Use case: Surface at specific GPS coordinates
 *          
 *          Variant 2 - Direct Ascent:
 *          If lat/lon are zero (not specified):
 *          - State: AUTO_SURFACE_STATE_ASCEND
 *          - Ascend directly from current position to zero depth
 *          - Use case: Emergency surface or simple ascent
 *          
 *          Altitude Frame Handling:
 *          - Attempts terrain-following if ABOVE_TERRAIN available
 *          - Falls back to ABOVE_HOME if terrain data unavailable
 *          - During lateral movement, maintains current depth to avoid obstacles
 *          
 *          State Machine:
 *          - auto_surface_state tracks current stage of procedure
 *          - State persists across verify_surface() calls
 *          - State advances when each stage completes
 *          
 *          Safety Considerations:
 *          - Two-stage approach prevents collision during lateral movement
 *          - Maintains depth during horizontal translation
 *          - Only ascends once positioned or if no lateral movement needed
 * 
 * @param[in] cmd Mission command containing optional target surface location
 * 
 * @note MAV_CMD_NAV_LAND repurposed for underwater ascent (no land equivalent)
 * @note Zero depth interpreted as water surface (ABOVE_HOME frame)
 * @note Completion verified by verify_surface() checking state machine progress
 * 
 * @warning Ascending through zero depth will cause vehicle to breach surface.
 *          Ensure positive buoyancy compensation configured if needed.
 * 
 * @see verify_surface() for state machine verification logic
 * @see AutoSurfaceState enum for state definitions
 * 
 * Source: ArduSub/commands_logic.cpp:255-288
 */
void Sub::do_surface(const AP_Mission::Mission_Command& cmd)
{
    Location target_location;

    // if location provided we fly to that location at current altitude
    if (cmd.content.location.lat != 0 || cmd.content.location.lng != 0) {
        // set state to go to location
        auto_surface_state = AUTO_SURFACE_STATE_GO_TO_LOCATION;

        // calculate and set desired location below surface target
        // convert to location class
        target_location = Location(cmd.content.location);

        // decide if we will use terrain following
        int32_t curr_terr_alt_cm, target_terr_alt_cm;
        if (current_loc.get_alt_cm(Location::AltFrame::ABOVE_TERRAIN, curr_terr_alt_cm) &&
                target_location.get_alt_cm(Location::AltFrame::ABOVE_TERRAIN, target_terr_alt_cm)) {
            // if using terrain, set target altitude to current altitude above terrain
            target_location.set_alt_cm(curr_terr_alt_cm, Location::AltFrame::ABOVE_TERRAIN);
        } else {
            // set target altitude to current altitude above home
            target_location.set_alt_cm(current_loc.alt, Location::AltFrame::ABOVE_HOME);
        }
    } else {
        // set surface state to ascend
        auto_surface_state = AUTO_SURFACE_STATE_ASCEND;

        // Set waypoint destination to current location at zero depth
        target_location = Location(current_loc.lat, current_loc.lng, 0, Location::AltFrame::ABOVE_HOME);
    }

    // Go to wp location
    mode_auto.auto_wp_start(target_location);
}

/**
 * @brief Initiates return to launch (home) location
 * 
 * @details Implements MAV_CMD_NAV_RETURN_TO_LAUNCH by commanding navigation to the
 *          home position. For underwater vehicles, returns to home at home's depth
 *          (typically zero/surface) rather than ascending first.
 *          
 *          RTL Behavior:
 *          - Navigates directly to home position (lat, lon, alt)
 *          - Home altitude typically zero (surface level)
 *          - No pre-ascent like aerial vehicles
 *          - Simple 3D navigation to home point
 *          
 * @note Home position set during arming or via DO_SET_HOME command
 * @note For underwater use, consider if home depth is appropriate
 * @note Completion verified by verify_RTL() checking wp_nav.reached_wp_destination()
 * 
 * @see verify_RTL() for completion criteria
 * @see set_home() for home position configuration
 * 
 * Source: ArduSub/commands_logic.cpp:290-293
 */
void Sub::do_RTL()
{
    mode_auto.auto_wp_start(ahrs.get_home());
}

/**
 * @brief Initiates indefinite loiter at specified location
 * 
 * @details Implements MAV_CMD_NAV_LOITER_UNLIM to hold position at a specified 3D
 *          location indefinitely. Mission will not advance past this command until
 *          manually interrupted or mission stopped.
 *          
 *          Position Processing:
 *          1. If lat/lon zero: Use current horizontal position (depth change only)
 *          2. If altitude zero: Maintain current depth
 *          3. Convert altitude to specified frame if needed
 *          4. Start waypoint navigation to target (hold position once reached)
 *          
 *          Altitude Frame Handling:
 *          - ABOVE_HOME (1): Altitude relative to home (negative for underwater)
 *          - ABOVE_ORIGIN (2): Relative to EKF origin
 *          - ABOVE_TERRAIN (3): Relative to seafloor (requires terrain data)
 *          - Frame conversion ensures depth interpreted correctly
 *          
 *          Use Cases:
 *          - Station keeping for underwater observation
 *          - Holding pattern waiting for external trigger
 *          - End of mission steady state
 *          
 * @param[in] cmd Mission command containing loiter position
 * 
 * @note Yaw mode should be configured by caller before this function
 * @note This command never completes on its own (verify_loiter_unlimited returns false)
 * @note Mission must be stopped manually or via MAVLink command to exit
 * @note If lat/lon zero, uses wp_nav stopping point for current horizontal position
 * 
 * @see verify_loiter_unlimited() which always returns false (infinite loiter)
 * 
 * Source: ArduSub/commands_logic.cpp:297-332
 */
void Sub::do_loiter_unlimited(const AP_Mission::Mission_Command& cmd)
{
    // convert back to location
    Location target_loc(cmd.content.location);

    // use current location if not provided
    if (target_loc.lat == 0 && target_loc.lng == 0) {
        // To-Do: make this simpler
        Vector3f temp_pos;
        wp_nav.get_wp_stopping_point_NE_cm(temp_pos.xy());
        const Location temp_loc(temp_pos, Location::AltFrame::ABOVE_ORIGIN);
        target_loc.lat = temp_loc.lat;
        target_loc.lng = temp_loc.lng;
    }

    // In mavproxy misseditor: Abs = 0 = ALT_FRAME_ABSOLUTE
    //                         Rel = 1 = ALT_FRAME_ABOVE_HOME
    //                         AGL = 3 = ALT_FRAME_ABOVE_TERRAIN
    //    2 = ALT_FRAME_ABOVE_ORIGIN, not an option in mavproxy misseditor

    // use current altitude if not provided
    // To-Do: use z-axis stopping point instead of current alt
    if (target_loc.alt == 0) {
        // set to current altitude but in command's alt frame
        int32_t curr_alt;
        if (current_loc.get_alt_cm(target_loc.get_alt_frame(),curr_alt)) {
            target_loc.set_alt_cm(curr_alt, target_loc.get_alt_frame());
        } else {
            // default to current altitude as alt-above-home
            target_loc.set_alt_cm(current_loc.alt, current_loc.get_alt_frame());
        }
    }

    // start way point navigator and provide it the desired location
    mode_auto.auto_wp_start(target_loc);
}

/**
 * @brief Initiates circular flight pattern around a center point
 * 
 * @details Implements MAV_CMD_NAV_LOITER_TURNS to fly in circles around a specified
 *          center point for a given number of turns. Useful for surveys, inspection,
 *          or establishing circular search patterns underwater.
 *          
 *          Circle Parameter Extraction:
 *          - Center: cmd.content.location (lat, lon, alt)
 *          - Radius: HIGHBYTE(cmd.p1) in meters (high byte of p1)
 *          - Radius multiplier: If bit 0 of type_specific_bits set, multiply radius by 10
 *          - Direction: cmd.content.location.loiter_ccw (true = counter-clockwise)
 *          - Turns: Stored in cmd.p1, extracted by get_loiter_turns()
 *          
 *          Default Value Handling:
 *          - Zero lat/lon: Use current position as center
 *          - Zero altitude: Use current depth
 *          - Altitude frame conversion applied for depth interpretation
 *          
 *          Execution Sequence:
 *          1. Parse center point, radius, direction from command
 *          2. Apply defaults for unspecified values
 *          3. Start "move to edge" mode (auto_circle_movetoedge_start)
 *          4. Navigate to circle edge (verify_circle monitors progress)
 *          5. Once at edge, begin circling (verify_circle starts circle)
 *          6. Count completed turns, finish when target turns reached
 *          
 *          Underwater Applications:
 *          - Circular inspection of underwater structures
 *          - Survey patterns for mapping
 *          - Search patterns around points of interest
 * 
 * @param[in] cmd Mission command containing circle center, radius, and turns
 * 
 * @note Circle starts by moving to the edge, then begins rotation
 * @note Radius encoded in high byte of p1 parameter (MAVLink convention)
 * @note Direction controlled by loiter_ccw flag in location
 * @note Completion verified by verify_circle() counting turns
 * 
 * @see verify_circle() for state machine and turn counting
 * @see auto_circle_movetoedge_start() for edge navigation
 * @see circle_nav.get_angle_total_rad() for turn counting
 * 
 * Source: ArduSub/commands_logic.cpp:335-371
 */
void Sub::do_circle(const AP_Mission::Mission_Command& cmd)
{
    Location circle_center(cmd.content.location);

    // default lat/lon to current position if not provided
    // To-Do: use stopping point or position_controller's target instead of current location to avoid jerk?
    if (circle_center.lat == 0 && circle_center.lng == 0) {
        circle_center.lat = current_loc.lat;
        circle_center.lng = current_loc.lng;
    }

    // default target altitude to current altitude if not provided
    if (circle_center.alt == 0) {
        int32_t curr_alt;
        if (current_loc.get_alt_cm(circle_center.get_alt_frame(),curr_alt)) {
            // circle altitude uses frame from command
            circle_center.set_alt_cm(curr_alt,circle_center.get_alt_frame());
        } else {
            // default to current altitude above origin
            circle_center.set_alt_cm(current_loc.alt, current_loc.get_alt_frame());
            LOGGER_WRITE_ERROR(LogErrorSubsystem::TERRAIN, LogErrorCode::MISSING_TERRAIN_DATA);
        }
    }

    // calculate radius
    uint16_t circle_radius_m = HIGHBYTE(cmd.p1); // circle radius held in high byte of p1
    if (cmd.type_specific_bits & (1U << 0)) {
        circle_radius_m *= 10;
    }


    // true if circle should be ccw
    const bool circle_direction_ccw = cmd.content.location.loiter_ccw;

    // move to edge of circle (verify_circle) will ensure we begin circling once we reach the edge
    mode_auto.auto_circle_movetoedge_start(circle_center, circle_radius_m, circle_direction_ccw);
}

/**
 * @brief Initiates loiter at a location for a specified time duration
 * 
 * @details Implements MAV_CMD_NAV_LOITER_TIME to hold position at a specified 3D
 *          location for a fixed duration before continuing mission. Uses same position
 *          logic as unlimited loiter but adds time-based completion condition.
 *          
 *          Timing Mechanism:
 *          - loiter_time_max set from cmd.p1 (duration in seconds)
 *          - Timer starts when position reached (in verify_loiter_time)
 *          - Command completes when duration elapsed
 *          
 *          Position Handling:
 *          - Delegates to do_loiter_unlimited() for position logic
 *          - Same default value handling (zero lat/lon/alt)
 *          - Same altitude frame conversion
 *          
 *          Use Cases:
 *          - Timed observation at specific locations
 *          - Delay at waypoint before continuing
 *          - Station keeping for sensor data collection
 * 
 * @param[in] cmd Mission command containing loiter position and duration (p1 in seconds)
 * 
 * @note Yaw mode should be configured by caller before this function
 * @note Timer does not start until vehicle reaches target position
 * @note Completion verified by verify_loiter_time() checking elapsed time
 * 
 * @see do_loiter_unlimited() for position logic
 * @see verify_loiter_time() for timing and completion logic
 * 
 * Source: ArduSub/commands_logic.cpp:374-383
 */
void Sub::do_loiter_time(const AP_Mission::Mission_Command& cmd)
{
    // re-use loiter unlimited
    do_loiter_unlimited(cmd);

    // setup loiter timer
    loiter_time     = 0;
    loiter_time_max = cmd.p1;     // units are (seconds)
}

#if NAV_GUIDED
/**
 * @brief Enables guided mode to accept external navigation commands
 * 
 * @details Implements MAV_CMD_NAV_GUIDED_ENABLE to allow mission to accept navigation
 *          commands from external computer or GCS. When enabled, vehicle suspends
 *          mission execution and responds to GUIDED mode targets until limits breached
 *          or guided disabled.
 *          
 *          Guided Mode Integration:
 *          - If cmd.p1 > 0: Enable guided mode within mission
 *          - Initialize time and position limits for safety
 *          - Start accepting external navigation targets
 *          - Mission resumes when guided disabled or limits exceeded
 *          
 *          Safety Limits:
 *          - Time limit: Maximum duration in guided mode
 *          - Position limit: Maximum distance from entry point
 *          - Altitude limit: Min/max altitude bounds
 *          - Limits prevent vehicle from straying during external control
 *          
 * @param[in] cmd Mission command with p1 = 1 to enable, 0 to disable
 * 
 * @note Only compiled if NAV_GUIDED feature enabled
 * @note Limits configured via DO_GUIDED_LIMITS command
 * @note Completion verified by verify_nav_guided_enable() checking limit status
 * 
 * @see do_guided_limits() for limit configuration
 * @see verify_nav_guided_enable() for limit monitoring
 * 
 * Source: ArduSub/commands_logic.cpp:387-396
 */
void Sub::do_nav_guided_enable(const AP_Mission::Mission_Command& cmd)
{
    if (cmd.p1 > 0) {
        // initialise guided limits
        mode_auto.guided_limit_init_time_and_pos();

        // set navigation target
        mode_auto.auto_nav_guided_start();
    }
}
#endif  // NAV_GUIDED

/**
 * @brief Delays mission execution for specified duration or until absolute time
 * 
 * @details Implements MAV_CMD_NAV_DELAY to pause mission progression either for a
 *          relative time period or until a specific UTC time is reached. Vehicle
 *          holds current position during delay.
 *          
 *          Delay Modes:
 *          
 *          Relative Delay (cmd.content.nav_delay.seconds > 0):
 *          - Delay for specified number of seconds from now
 *          - nav_delay_time_max_ms = seconds * 1000
 *          - Common use case: timed pauses in mission
 *          
 *          Absolute Delay (cmd.content.nav_delay.seconds <= 0):
 *          - Wait until specified UTC time (hour, minute, second)
 *          - Requires RTC (Real-Time Clock) support
 *          - nav_delay_time_max_ms = UTC timestamp from RTC
 *          - Use case: synchronized operations, time-specific events
 *          
 *          Timing Implementation:
 *          - nav_delay_time_start_ms captures current time
 *          - verify_nav_delay() checks if delay elapsed
 *          - Vehicle maintains position during delay
 * 
 * @param[in] cmd Mission command containing delay parameters
 * 
 * @note Absolute time delay requires AP_RTC_ENABLED
 * @note If RTC not available, absolute delay behaves as no delay
 * @note Completion verified by verify_nav_delay() comparing timestamps
 * 
 * @see verify_nav_delay() for delay completion checking
 * @see AP::rtc().get_time_utc() for absolute time calculation
 * 
 * Source: ArduSub/commands_logic.cpp:400-416
 */
void Sub::do_nav_delay(const AP_Mission::Mission_Command& cmd)
{
    nav_delay_time_start_ms = AP_HAL::millis();

    if (cmd.content.nav_delay.seconds > 0) {
        // relative delay
        nav_delay_time_max_ms = cmd.content.nav_delay.seconds * 1000; // convert seconds to milliseconds
    } else {
        // absolute delay to utc time
#if AP_RTC_ENABLED
        nav_delay_time_max_ms = AP::rtc().get_time_utc(cmd.content.nav_delay.hour_utc, cmd.content.nav_delay.min_utc, cmd.content.nav_delay.sec_utc, 0);
#else
        nav_delay_time_max_ms = 0;
#endif
    }
    gcs().send_text(MAV_SEVERITY_INFO, "Delaying %u sec", (unsigned)(nav_delay_time_max_ms/1000));
}

#if NAV_GUIDED
/**
 * @brief Configures safety limits for guided mode operation
 * 
 * @details Implements MAV_CMD_DO_GUIDED_LIMITS to set constraints on external navigation
 *          commands during guided mode. Prevents vehicle from being commanded outside
 *          safe operating envelope when under external control.
 *          
 *          Limit Parameters:
 *          - Time limit: cmd.p1 (seconds) - Maximum duration in guided mode
 *          - Altitude min: cmd.content.guided_limits.alt_min (meters) - Minimum depth
 *          - Altitude max: cmd.content.guided_limits.alt_max (meters) - Maximum depth
 *          - Horizontal: cmd.content.guided_limits.horiz_max (meters) - Max radius from entry
 *          
 *          Unit Conversions:
 *          - Time: seconds → milliseconds (×1000)
 *          - Altitude: meters → centimeters (×100)
 *          - Horizontal: meters → centimeters (×100)
 *          
 *          Safety Behavior:
 *          - Limits checked by verify_nav_guided_enable()
 *          - If any limit breached, guided mode exits and mission resumes
 *          - Prevents runaway during external control
 * 
 * @param[in] cmd Mission command containing limit values in meters and seconds
 * 
 * @note Only compiled if NAV_GUIDED feature enabled
 * @note Limits apply to subsequent NAV_GUIDED_ENABLE commands
 * @note This is a DO command (executes immediately, no verification)
 * 
 * @see do_nav_guided_enable() which uses these limits
 * @see mode_guided.guided_limit_set() for limit enforcement
 * 
 * Source: ArduSub/commands_logic.cpp:420-426
 */
void Sub::do_guided_limits(const AP_Mission::Mission_Command& cmd)
{
    mode_guided.guided_limit_set(cmd.p1 * 1000, // convert seconds to ms
                     cmd.content.guided_limits.alt_min * 100.0f,    // convert meters to cm
                     cmd.content.guided_limits.alt_max * 100.0f,    // convert meters to cm
                     cmd.content.guided_limits.horiz_max * 100.0f); // convert meters to cm
}
#endif

/********************************************************************************/
//  Verify Nav (Must) Commands
/********************************************************************************/

/**
 * @brief Verifies waypoint navigation command has completed
 * 
 * @details Checks if vehicle has reached waypoint position and completed any required
 *          loiter time. Two-stage completion: position reached, then timer elapsed.
 *          
 *          Completion Criteria:
 *          1. Position Check: wp_nav.reached_wp_destination() returns true
 *             - Horizontal distance within acceptance radius
 *             - Vertical distance within altitude tolerance
 *          2. Loiter Timer: If loiter_time_max > 0, must hold position for duration
 *             - Timer starts when position first reached
 *             - Command completes when timer expires
 *          
 *          State Management:
 *          - loiter_time initialized to 0 in do_nav_wp()
 *          - Set to current time when position first reached
 *          - Duration checked against loiter_time_max
 *          
 *          Event Notifications:
 *          - waypoint_complete event triggers notification (LED/tone)
 *          - GCS message sent when command fully completes
 * 
 * @param[in] cmd Mission command being verified (for logging command index)
 * 
 * @return true if position reached and loiter time elapsed (if required)
 * @return false if still navigating to waypoint or loiter time incomplete
 * 
 * @note Called at main loop rate (50Hz) until returns true
 * @note If loiter_time_max is 0, completes immediately upon position reached
 * @note Acceptance radius configured by WPNAV_RADIUS parameter
 * 
 * @see do_nav_wp() for command initiation and timer setup
 * @see wp_nav.reached_wp_destination() for position acceptance logic
 * 
 * Source: ArduSub/commands_logic.cpp:434-456
 */
bool Sub::verify_nav_wp(const AP_Mission::Mission_Command& cmd)
{
    // check if we have reached the waypoint
    if (!wp_nav.reached_wp_destination()) {
        return false;
    }

    // play a tone
    AP_Notify::events.waypoint_complete = 1;

    // start timer if necessary
    if (loiter_time == 0) {
        loiter_time = AP_HAL::millis();
    }

    // check if timer has run out
    if (((AP_HAL::millis() - loiter_time) / 1000) >= loiter_time_max) {
        gcs().send_text(MAV_SEVERITY_INFO, "Reached command #%i",cmd.index);
        return true;
    }

    return false;
}

/**
 * @brief Verifies multi-stage surface procedure has completed
 * 
 * @details Implements state machine to verify the surface command's two-stage ascent
 *          procedure: optional lateral movement followed by vertical ascent to zero depth.
 *          
 *          State Machine:
 *          
 *          State: AUTO_SURFACE_STATE_GO_TO_LOCATION
 *          - Verify: wp_nav.reached_wp_destination() at target lat/lon
 *          - Action on complete: Set new target to current xy, zero depth
 *          - Transition: Advance to AUTO_SURFACE_STATE_ASCEND
 *          - Purpose: Complete horizontal movement before ascending
 *          
 *          State: AUTO_SURFACE_STATE_ASCEND
 *          - Verify: wp_nav.reached_wp_destination() at zero depth
 *          - Action on complete: Return true (surface command done)
 *          - Purpose: Verify vehicle has reached surface
 *          
 *          State: Default (should never occur)
 *          - Invalid state, return true to continue mission
 *          - Log error if possible (TODO in code)
 *          
 *          Safety Considerations:
 *          - Two-stage approach prevents collision during lateral movement
 *          - Horizontal movement completes at depth before ascent
 *          - Ascent is vertical from final horizontal position
 *          
 *          Underwater Behavior:
 *          - Zero depth = surface (ABOVE_HOME frame)
 *          - Vehicle will breach surface at completion
 *          - Positive buoyancy may cause further ascent
 * 
 * @param[in] cmd Mission command containing target surface location
 * 
 * @return true if vehicle has reached surface (zero depth) at target location
 * @return false if still in GO_TO_LOCATION or ASCEND state
 * 
 * @note State variable auto_surface_state persists across calls
 * @note Called at main loop rate (50Hz) until returns true
 * @note TODO: Add error logging for invalid state
 * 
 * @warning Vehicle will breach surface on completion. Ensure safe retrieval plan.
 * 
 * @see do_surface() for state initialization and command setup
 * @see AutoSurfaceState enum for state definitions
 * 
 * Source: ArduSub/commands_logic.cpp:459-493
 */
bool Sub::verify_surface(const AP_Mission::Mission_Command& cmd)
{
    bool retval = false;

    switch (auto_surface_state) {
        case AUTO_SURFACE_STATE_GO_TO_LOCATION:
            // check if we've reached the location
            if (wp_nav.reached_wp_destination()) {
                // Set target to current xy and zero depth
                // TODO get xy target from current wp destination, because current location may be acceptance-radius away from original destination
                Location target_location(cmd.content.location.lat, cmd.content.location.lng, 0, Location::AltFrame::ABOVE_HOME);

                mode_auto.auto_wp_start(target_location);

                // advance to next state
                auto_surface_state = AUTO_SURFACE_STATE_ASCEND;
            }
            break;

        case AUTO_SURFACE_STATE_ASCEND:
            if (wp_nav.reached_wp_destination()) {
                retval = true;
            }
            break;

        default:
            // this should never happen
            // TO-DO: log an error
            retval = true;
            break;
    }

    // true is returned if we've successfully surfaced
    return retval;
}

/**
 * @brief Verifies return to launch command has completed
 * 
 * @details Simple verification that vehicle has reached home position. No loiter
 *          time required for RTL command.
 * 
 * @return true if vehicle within acceptance radius of home position
 * @return false if still navigating to home
 * 
 * @note Home position includes altitude, so vehicle navigates to home depth
 * @note Acceptance radius configured by WPNAV_RADIUS parameter
 * 
 * @see do_RTL() for command initiation
 * 
 * Source: ArduSub/commands_logic.cpp:495-497
 */
bool Sub::verify_RTL() {
    return wp_nav.reached_wp_destination();
}

/**
 * @brief Verifies unlimited loiter command (never completes)
 * 
 * @details Always returns false to maintain loiter indefinitely. Mission will not
 *          advance past this command without manual intervention.
 * 
 * @return false Always returns false (infinite loiter, never completes)
 * 
 * @note Mission must be stopped manually or via MAVLink command to exit
 * @note Vehicle maintains position at loiter target location
 * 
 * @see do_loiter_unlimited() for command initiation
 * 
 * Source: ArduSub/commands_logic.cpp:499-502
 */
bool Sub::verify_loiter_unlimited()
{
    return false;
}

/**
 * @brief Verifies timed loiter command has completed position hold and duration
 * 
 * @details Two-stage verification: reach position, then wait for timer to expire.
 *          Timer only starts counting once vehicle reaches target position.
 *          
 *          Completion Sequence:
 *          1. Check position: wp_nav.reached_wp_destination()
 *          2. If not reached, return false (keep navigating)
 *          3. If reached and timer not started (loiter_time == 0), start timer
 *          4. If timer started, check if duration (loiter_time_max) elapsed
 *          5. Return true if duration complete, false otherwise
 *          
 *          Timer Mechanism:
 *          - loiter_time: Timestamp when position reached (milliseconds)
 *          - loiter_time_max: Required hold duration (seconds)
 *          - Comparison: (current_time - loiter_time) / 1000 >= loiter_time_max
 * 
 * @return true if position reached and loiter duration elapsed
 * @return false if still navigating or holding position during timer
 * 
 * @note Timer initialized to 0 in do_loiter_time()
 * @note Timer starts when position first reached, not when command starts
 * @note Called at main loop rate (50Hz) until returns true
 * 
 * @see do_loiter_time() for command initiation and timer setup
 * 
 * Source: ArduSub/commands_logic.cpp:505-519
 */
bool Sub::verify_loiter_time()
{
    // return immediately if we haven't reached our destination
    if (!wp_nav.reached_wp_destination()) {
        return false;
    }

    // start our loiter timer
    if (loiter_time == 0) {
        loiter_time = AP_HAL::millis();
    }

    // check if loiter timer has run out
    return (((AP_HAL::millis() - loiter_time) / 1000) >= loiter_time_max);
}

/**
 * @brief Verifies circle command has completed required number of turns
 * 
 * @details Two-phase verification: navigate to circle edge, then count completed turns.
 *          State transitions from move-to-edge to circling mode when edge reached.
 *          
 *          Phase 1 - Move to Edge (auto_mode == Auto_CircleMoveToEdge):
 *          - Check if edge position reached via wp_nav.reached_wp_destination()
 *          - Once reached, parse circle center from command
 *          - Apply defaults: zero altitude → current depth, zero lat/lon → current position
 *          - Start circling via auto_circle_start()
 *          - Return false (circling just started, not complete)
 *          
 *          Phase 2 - Circling:
 *          - Extract required turns from command via get_loiter_turns()
 *          - Monitor circle_nav.get_angle_total_rad() for accumulated rotation
 *          - Convert radians to revolutions: angle / (2π)
 *          - Compare absolute revolutions to required turns
 *          - Return true when turns complete
 *          
 *          Turn Counting:
 *          - circle_nav tracks cumulative angle rotated
 *          - Positive or negative rotation counted (absolute value)
 *          - Sub-turn precision (can specify fractional turns)
 *          
 *          Default Handling in Phase 1:
 *          - Zero altitude: Use current z position from inertial_nav
 *          - Zero lat/lon: Use current xy position from inertial_nav
 *          - Ensures smooth transition to circling if parameters unspecified
 * 
 * @param[in] cmd Mission command containing circle parameters and turn count
 * 
 * @return true if required number of turns completed
 * @return false if still moving to edge or circling in progress
 * 
 * @note auto_mode state variable tracks current auto flight sub-mode
 * @note Called at main loop rate (50Hz) until returns true
 * @note Turn count extracted via cmd.get_loiter_turns()
 * 
 * @see do_circle() for command initiation and parameter parsing
 * @see auto_circle_movetoedge_start() for edge navigation
 * @see auto_circle_start() for circle mode activation
 * @see circle_nav.get_angle_total_rad() for rotation tracking
 * 
 * Source: ArduSub/commands_logic.cpp:522-549
 */
bool Sub::verify_circle(const AP_Mission::Mission_Command& cmd)
{
    // check if we've reached the edge
    if (auto_mode == Auto_CircleMoveToEdge) {
        if (wp_nav.reached_wp_destination()) {
            Vector3f circle_center;
            UNUSED_RESULT(cmd.content.location.get_vector_from_origin_NEU_cm(circle_center));

            // set target altitude if not provided
            if (is_zero(circle_center.z)) {
                circle_center.z = inertial_nav.get_position_z_up_cm();
            }

            // set lat/lon position if not provided
            if (cmd.content.location.lat == 0 && cmd.content.location.lng == 0) {
                circle_center.xy() = inertial_nav.get_position_xy_cm();
            }

            // start circling
            mode_auto.auto_circle_start();
        }
        return false;
    }
    const float turns = cmd.get_loiter_turns();

    // check if we have completed circling
    return fabsf(sub.circle_nav.get_angle_total_rad()/M_2PI) >= turns;
}

#if NAV_GUIDED
/**
 * @brief Verifies guided mode enable command by checking safety limits
 * 
 * @details Monitors guided mode operation and returns true when guided should end,
 *          either by explicit disable or by breaching safety limits.
 *          
 *          Verification Logic:
 *          
 *          Disable Request (cmd.p1 == 0):
 *          - Return true immediately
 *          - Exits guided mode and resumes mission
 *          
 *          Enable Mode (cmd.p1 > 0):
 *          - Check time limit: Has maximum guided duration elapsed?
 *          - Check position limit: Has vehicle exceeded horizontal radius?
 *          - Check altitude limits: Is vehicle within min/max depth bounds?
 *          - Return true if ANY limit breached (safety exit)
 *          - Return false if all limits satisfied (continue guided)
 *          
 *          Safety Mechanism:
 *          - Limits prevent vehicle straying during external control
 *          - Automatic exit returns control to mission
 *          - Conservative approach: breach any limit → exit guided
 * 
 * @param[in] cmd Mission command with p1 indicating enable (>0) or disable (0)
 * 
 * @return true if guided mode should end (disabled or limits breached)
 * @return false if guided mode should continue (limits not breached)
 * 
 * @note Only compiled if NAV_GUIDED feature enabled
 * @note Limits configured by prior DO_GUIDED_LIMITS command
 * @note Called at main loop rate (50Hz) while in guided mode
 * 
 * @see do_nav_guided_enable() for guided mode initialization
 * @see do_guided_limits() for limit configuration
 * @see mode_auto.guided_limit_check() for limit enforcement logic
 * 
 * Source: ArduSub/commands_logic.cpp:553-562
 */
bool Sub::verify_nav_guided_enable(const AP_Mission::Mission_Command& cmd)
{
    // if disabling guided mode then immediately return true so we move to next command
    if (cmd.p1 == 0) {
        return true;
    }

    // check time and position limits
    return mode_auto.guided_limit_check();
}
#endif  // NAV_GUIDED

/**
 * @brief Verifies navigation delay command has completed
 * 
 * @details Checks if required delay duration has elapsed by comparing current time
 *          to start time plus delay duration. Handles both relative and absolute delays.
 *          
 *          Timing Check:
 *          - Start time: nav_delay_time_start_ms (set in do_nav_delay)
 *          - Delay duration: nav_delay_time_max_ms (milliseconds)
 *          - Current time: AP_HAL::millis()
 *          - Complete when: (current_time - start_time) > duration
 *          
 *          Cleanup:
 *          - Reset nav_delay_time_max_ms to 0 when complete
 *          - Prepares for potential future delay commands
 * 
 * @param[in] cmd Mission command (not used, delay parameters already stored)
 * 
 * @return true if delay duration has elapsed
 * @return false if still waiting for delay to complete
 * 
 * @note Works for both relative delays (seconds) and absolute delays (UTC time)
 * @note Called at main loop rate (50Hz) during delay period
 * @note Vehicle maintains position during delay
 * 
 * @see do_nav_delay() for delay initialization and parameter processing
 * 
 * Source: ArduSub/commands_logic.cpp:566-573
 */
bool Sub::verify_nav_delay(const AP_Mission::Mission_Command& cmd)
{
    if (AP_HAL::millis() - nav_delay_time_start_ms > nav_delay_time_max_ms) {
        nav_delay_time_max_ms = 0;
        return true;
    }
    return false;
}

/********************************************************************************/
//  Condition (May) Commands
/********************************************************************************/

/**
 * @brief Initiates conditional delay timer
 * 
 * @details Implements MAV_CMD_CONDITION_DELAY to pause subsequent DO commands until
 *          a time delay expires. Unlike NAV_DELAY, this is a conditional command that
 *          gates DO command execution without affecting NAV command navigation.
 *          
 *          Timing Setup:
 *          - condition_start: Current time in milliseconds
 *          - condition_value: Delay duration converted to milliseconds
 *          - Conversion: seconds × 1000 → milliseconds
 *          
 *          Conditional Command Behavior:
 *          - NAV commands continue executing normally
 *          - DO commands after this condition wait for verification
 *          - Allows timed sequencing of actions during navigation
 * 
 * @param[in] cmd Mission command containing delay duration in seconds
 * 
 * @note This is a condition command, not a navigation command
 * @note Does not affect vehicle navigation or position holding
 * @note Completion verified by verify_wait_delay()
 * 
 * @see verify_wait_delay() for completion checking
 * 
 * Source: ArduSub/commands_logic.cpp:579-583
 */
void Sub::do_wait_delay(const AP_Mission::Mission_Command& cmd)
{
    condition_start = AP_HAL::millis();
    condition_value = cmd.content.delay.seconds * 1000;     // convert seconds to milliseconds
}

/**
 * @brief Initiates conditional distance check
 * 
 * @details Implements MAV_CMD_CONDITION_DISTANCE to gate DO command execution until
 *          vehicle comes within specified distance of current waypoint target.
 *          
 *          Distance Setup:
 *          - condition_value: Distance threshold in meters
 *          - Compared against wp_nav distance to destination
 *          - Condition met when within threshold
 *          
 *          Use Case:
 *          - Trigger camera at specific distance before waypoint
 *          - Start data collection when approaching target
 *          - Sequence actions based on proximity
 * 
 * @param[in] cmd Mission command containing distance threshold in meters
 * 
 * @note This is a condition command, not a navigation command
 * @note Does not affect vehicle navigation
 * @note Completion verified by verify_within_distance()
 * 
 * @see verify_within_distance() for distance checking
 * 
 * Source: ArduSub/commands_logic.cpp:585-588
 */
void Sub::do_within_distance(const AP_Mission::Mission_Command& cmd)
{
    condition_value  = cmd.content.distance.meters;
}

/**
 * @brief Initiates yaw (heading) command for vehicle orientation
 * 
 * @details Implements MAV_CMD_CONDITION_YAW to command vehicle to turn to a specified
 *          heading at a specified turn rate. Can be absolute or relative heading.
 *          
 *          Yaw Command Parameters:
 *          - angle_deg: Target heading in degrees
 *          - turn_rate_dps: Turn rate in degrees per second
 *          - direction: Rotation direction (shortest, CW, CCW)
 *          - relative_angle: If true, angle is relative to current heading
 *          
 *          Yaw Control:
 *          - Delegates to mode_auto.set_auto_yaw_look_at_heading()
 *          - Yaw controller manages rotation at specified rate
 *          - Completion checked by verify_yaw()
 *          
 *          Underwater Application:
 *          - Orient vehicle for sensor pointing
 *          - Camera framing for inspection
 *          - Vehicle orientation for hydrodynamics
 * 
 * @param[in] cmd Mission command containing yaw parameters
 * 
 * @note Can be used as condition or as immediate DO command in guided mode
 * @note Completion verified by verify_yaw() checking heading achieved
 * 
 * @see verify_yaw() for completion checking (within 2 degrees)
 * @see mode_auto.set_auto_yaw_look_at_heading() for yaw control
 * 
 * Source: ArduSub/commands_logic.cpp:590-597
 */
void Sub::do_yaw(const AP_Mission::Mission_Command& cmd)
{
    sub.mode_auto.set_auto_yaw_look_at_heading(
        cmd.content.yaw.angle_deg,
        cmd.content.yaw.turn_rate_dps,
        cmd.content.yaw.direction,
        cmd.content.yaw.relative_angle);
}


/********************************************************************************/
// Verify Condition (May) Commands
/********************************************************************************/

/**
 * @brief Verifies conditional delay has elapsed
 * 
 * @details Checks if condition timer has expired by comparing elapsed time to
 *          required delay duration. Uses MAX to ensure non-negative comparison.
 *          
 *          Timing Check:
 *          - Elapsed time: AP_HAL::millis() - condition_start
 *          - Required delay: condition_value (milliseconds)
 *          - Complete when: elapsed > required
 *          
 *          Cleanup:
 *          - Reset condition_value to 0 when complete
 *          - Prepares for potential future condition commands
 * 
 * @return true if delay has elapsed, allowing subsequent DO commands
 * @return false if still waiting for delay to complete
 * 
 * @note Called at main loop rate (50Hz) during condition delay
 * @note MAX macro prevents negative value issues
 * 
 * @see do_wait_delay() for condition initialization
 * 
 * Source: ArduSub/commands_logic.cpp:604-611
 */
bool Sub::verify_wait_delay()
{
    if (AP_HAL::millis() - condition_start > (uint32_t)MAX(condition_value, 0)) {
        condition_value = 0;
        return true;
    }
    return false;
}

/**
 * @brief Verifies vehicle is within specified distance of waypoint
 * 
 * @details Checks if current distance to active waypoint destination is less than
 *          the condition threshold. Uses centimeter comparison from wp_nav.
 *          
 *          Distance Check:
 *          - Current distance: wp_nav.get_wp_distance_to_destination_cm()
 *          - Threshold: condition_value (meters, but compared in cm context)
 *          - Complete when: distance < threshold
 *          
 *          Note on Units:
 *          - wp_nav returns distance in centimeters
 *          - condition_value stored as meters from command
 *          - Comparison appears to assume meters interpretation
 *          
 *          Cleanup:
 *          - Reset condition_value to 0 when complete
 *          - Prepares for potential future condition commands
 * 
 * @return true if within distance threshold, allowing subsequent DO commands
 * @return false if still beyond distance threshold
 * 
 * @note Called at main loop rate (50Hz) while approaching waypoint
 * @note MAX macro prevents negative value issues
 * @note Only meaningful when navigating to a waypoint
 * 
 * @see do_within_distance() for condition initialization
 * @see wp_nav.get_wp_distance_to_destination_cm() for distance calculation
 * 
 * Source: ArduSub/commands_logic.cpp:613-620
 */
bool Sub::verify_within_distance()
{
    if (wp_nav.get_wp_distance_to_destination_cm() < (uint32_t)MAX(condition_value,0)) {
        condition_value = 0;
        return true;
    }
    return false;
}

/**
 * @brief Verifies vehicle has achieved commanded yaw (heading)
 * 
 * @details Checks if vehicle heading is within tolerance of target heading.
 *          Also ensures auto yaw mode is set correctly for heading control.
 *          
 *          Verification Steps:
 *          1. Check if auto_yaw_mode is LOOK_AT_HEADING
 *          2. If not, reset to LOOK_AT_HEADING mode
 *             (waypoint controller may have changed yaw mode)
 *          3. Compare current heading to target heading
 *          4. Return true if within 2 degrees (200 centidegrees)
 *          
 *          Heading Comparison:
 *          - Current: ahrs.yaw_sensor (centidegrees)
 *          - Target: yaw_look_at_heading (centidegrees)
 *          - Tolerance: ±200 centidegrees (±2 degrees)
 *          - Wrapping: wrap_180_cd() handles 0°/360° boundary
 *          
 *          Yaw Mode Management:
 *          - Waypoint controller may change yaw mode during navigation
 *          - This function restores LOOK_AT_HEADING if needed
 *          - Ensures yaw command maintains priority
 * 
 * @return true if heading within 2 degrees of target
 * @return false if still rotating to target heading
 * 
 * @note Called at main loop rate (50Hz) during yaw maneuver
 * @note Tolerance of 2 degrees provides reasonable completion detection
 * @note Centidegrees used for precision (1 centidegree = 0.01 degrees)
 * 
 * @see do_yaw() for yaw command initiation
 * @see wrap_180_cd() for heading wrapping (-180 to +180)
 * @see mode_auto.set_auto_yaw_mode() for yaw mode control
 * 
 * Source: ArduSub/commands_logic.cpp:623-632
 */
bool Sub::verify_yaw()
{
    // set yaw mode if it has been changed (the waypoint controller often retakes control of yaw as it executes a new waypoint command)
    if (auto_yaw_mode != AUTO_YAW_LOOK_AT_HEADING) {
        sub.mode_auto.set_auto_yaw_mode(AUTO_YAW_LOOK_AT_HEADING);
    }

    // check if we are within 2 degrees of the target heading
    return (abs(wrap_180_cd(ahrs.yaw_sensor-yaw_look_at_heading)) <= 200);
}

/********************************************************************************/
//  Do (Now) Commands
/********************************************************************************/

/**
 * @brief Processes guided mode command during mission or guided mode
 * 
 * @details Handles immediate guided commands that can be sent during mission execution
 *          or while in guided mode. Allows external control to override certain behaviors
 *          without fully exiting autonomous operation.
 *          
 *          Mode Requirements:
 *          - Must be in GUIDED mode, OR
 *          - Must be in AUTO mode with NavGuided sub-mode active
 *          - Otherwise, command rejected (return false)
 *          
 *          Supported Commands:
 *          
 *          MAV_CMD_NAV_WAYPOINT:
 *          - Set new guided destination waypoint
 *          - Vehicle navigates to new location immediately
 *          - Return value from guided_set_destination (success/fail)
 *          
 *          MAV_CMD_CONDITION_YAW:
 *          - Change vehicle heading while guided
 *          - Delegates to do_yaw() for yaw control
 *          - Always returns true (yaw command accepted)
 *          
 *          Default:
 *          - Unrecognized commands rejected
 *          - Returns false (command not handled)
 *          
 *          Use Cases:
 *          - External navigation during mission guided segments
 *          - Dynamic waypoint adjustment from GCS
 *          - Heading changes during guided operations
 * 
 * @param[in] cmd Guided command to execute
 * 
 * @return true if command was accepted and executed successfully
 * @return false if not in guided mode or command unrecognized
 * 
 * @note This is a DO command (executes immediately, no verification needed)
 * @note Only processes commands when in appropriate mode
 * @note Waypoint destination subject to guided mode limits
 * 
 * @see do_yaw() for yaw command processing
 * @see mode_guided.guided_set_destination() for waypoint setting
 * 
 * Source: ArduSub/commands_logic.cpp:639-664
 */
bool Sub::do_guided(const AP_Mission::Mission_Command& cmd)
{
    // only process guided waypoint if we are in guided mode
    if (control_mode != Mode::Number::GUIDED && !(control_mode == Mode::Number::AUTO && auto_mode == Auto_NavGuided)) {
        return false;
    }

    // switch to handle different commands
    switch (cmd.id) {

    case MAV_CMD_NAV_WAYPOINT: {
        // set wp_nav's destination
        return sub.mode_guided.guided_set_destination(cmd.content.location);
    }

    case MAV_CMD_CONDITION_YAW:
        do_yaw(cmd);
        return true;

    default:
        // reject unrecognised command
        return false;
    }

    return true;
}

/**
 * @brief Changes horizontal navigation speed during mission
 * 
 * @details Implements MAV_CMD_DO_CHANGE_SPEED to adjust vehicle navigation speed
 *          dynamically during mission execution. Affects waypoint navigation rate.
 *          
 *          Speed Processing:
 *          - Target speed: cmd.content.speed.target_ms (meters/second)
 *          - Unit conversion: m/s × 100 → cm/s
 *          - Applied via wp_nav.set_speed_NE_cms()
 *          
 *          Validation:
 *          - Only accepts positive speeds (target_ms > 0)
 *          - Zero or negative speeds ignored (no change)
 *          - Prevents invalid speed configurations
 *          
 *          Coordinate System:
 *          - NE: North-East (horizontal plane in NED frame)
 *          - Does not affect vertical (depth) speed
 *          - Applies to xy waypoint navigation
 *          
 *          Underwater Considerations:
 *          - Speed limited by vehicle hydrodynamics
 *          - Higher speeds increase power consumption
 *          - Consider current effects on navigation
 * 
 * @param[in] cmd Mission command containing target speed in m/s
 * 
 * @note This is a DO command (executes immediately, no verification needed)
 * @note Speed persists for subsequent waypoints until changed again
 * @note Zero/negative speeds are ignored (no error, no change)
 * @note Vertical speed unaffected (controlled separately)
 * 
 * @see wp_nav.set_speed_NE_cms() for navigation controller speed setting
 * 
 * Source: ArduSub/commands_logic.cpp:666-671
 */
void Sub::do_change_speed(const AP_Mission::Mission_Command& cmd)
{
    if (cmd.content.speed.target_ms > 0) {
        wp_nav.set_speed_NE_cms(cmd.content.speed.target_ms * 100.0f);
    }
}

/**
 * @brief Sets home position during mission execution
 * 
 * @details Implements MAV_CMD_DO_SET_HOME to update the home position, which is used
 *          for RTL (Return To Launch) and as reference for relative navigation.
 *          
 *          Home Position Modes:
 *          
 *          Mode 1 - Use Current Location (cmd.p1 == 1):
 *          - Set home to vehicle's current position
 *          - Captures current lat, lon, alt from inertial nav
 *          - Use case: Update home during mission
 *          
 *          Mode 2 - Zero Location Check (all lat/lon/alt are 0):
 *          - Treat as request for current location
 *          - Behaves same as Mode 1
 *          - Convenient command format
 *          
 *          Mode 3 - Specified Location (default):
 *          - Set home to specified lat, lon, alt
 *          - Use case: Set home to known safe location
 *          - Can be used for mission start point
 *          
 *          Error Handling:
 *          - Failures silently ignored (noted in comment)
 *          - Mission continues regardless of set_home result
 *          - Conservative approach: don't abort mission on home update failure
 *          
 *          Underwater Implications:
 *          - Home altitude typically zero (surface)
 *          - RTL will navigate to home depth
 *          - Consider if home depth is safe for RTL
 * 
 * @param[in] cmd Mission command with p1 flag and optional location
 * 
 * @note This is a DO command (executes immediately, no verification needed)
 * @note Set home failures are silently ignored (mission continues)
 * @note Home position affects RTL behavior and relative navigation
 * @note Parameter false in set_home calls disables home altitude save
 * 
 * @see set_home_to_current_location() for current position capture
 * @see set_home() for explicit location setting
 * @see do_RTL() which navigates to home position
 * 
 * Source: ArduSub/commands_logic.cpp:673-684
 */
void Sub::do_set_home(const AP_Mission::Mission_Command& cmd)
{
    if (cmd.p1 == 1 || (cmd.content.location.lat == 0 && cmd.content.location.lng == 0 && cmd.content.location.alt == 0)) {
        if (!set_home_to_current_location(false)) {
            // silently ignore this failure
        }
    } else {
        if (!set_home(cmd.content.location, false)) {
            // silently ignore this failure
        }
    }
}

/**
 * @brief Commands vehicle and/or camera to point at Region of Interest (ROI)
 * 
 * @details Implements MAV_CMD_NAV_ROI, MAV_CMD_DO_SET_ROI, MAV_CMD_DO_SET_ROI_LOCATION,
 *          and MAV_CMD_DO_SET_ROI_NONE to direct vehicle heading and camera pointing
 *          toward a specified location.
 *          
 *          ROI Behavior:
 *          - Camera: Points camera gimbal at ROI location (if mount available)
 *          - Vehicle: Rotates vehicle to face ROI (if camera mount lacks yaw)
 *          - Combined: Both camera and vehicle orient toward ROI
 *          
 *          Implementation:
 *          - Delegates to set_auto_yaw_roi() for yaw control
 *          - Auto yaw mode set to track ROI location
 *          - Vehicle continuously adjusts heading to face ROI
 *          - ROI tracking persists until cleared or overridden
 *          
 *          ROI Location:
 *          - cmd.content.location: Target point (lat, lon, alt)
 *          - Vehicle calculates bearing to ROI
 *          - Yaw controller maintains pointing
 *          
 *          Clearing ROI:
 *          - MAV_CMD_DO_SET_ROI_NONE clears ROI (location zeros)
 *          - Yaw control returns to normal (waypoint following)
 *          
 *          Underwater Applications:
 *          - Point camera at inspection target
 *          - Orient vehicle for sensor pointing
 *          - Track moving targets or features
 * 
 * @param[in] cmd Mission command containing ROI location
 * 
 * @note This is a DO command (executes immediately, no verification needed)
 * @note ROI tracking is continuous (vehicle constantly adjusts heading)
 * @note Camera mount gimbal control handled separately by mount driver
 * @note TODO: Add support for waypoint-relative ROI (future enhancement)
 * 
 * @warning If vehicle mount lacks yaw axis, vehicle body will rotate to point ROI,
 *          affecting vehicle orientation and hydrodynamics
 * 
 * @see mode_auto.set_auto_yaw_roi() for ROI yaw control implementation
 * @see do_mount_control() for explicit camera gimbal control
 * 
 * Source: ArduSub/commands_logic.cpp:686-693
 */
void Sub::do_roi(const AP_Mission::Mission_Command& cmd)
{
    sub.mode_auto.set_auto_yaw_roi(cmd.content.location);
}

/**
 * @brief Commands camera gimbal mount to specific angle
 * 
 * @details Implements MAV_CMD_DO_MOUNT_CONTROL to position camera gimbal to specified
 *          roll, pitch, and yaw angles. Provides precise camera pointing independent
 *          of vehicle orientation.
 *          
 *          Mount Control:
 *          - Roll: cmd.content.mount_control.roll (degrees)
 *          - Pitch: cmd.content.mount_control.pitch (degrees)
 *          - Yaw: cmd.content.mount_control.yaw (degrees)
 *          - Angles are absolute in vehicle body frame
 *          
 *          Gimbal Operation:
 *          - 3-axis stabilized camera mount
 *          - Compensates for vehicle motion
 *          - Maintains camera pointing regardless of vehicle attitude
 *          
 *          Angle Interpretation:
 *          - Body frame relative to vehicle
 *          - Roll: Right wing down positive
 *          - Pitch: Nose up positive  
 *          - Yaw: Right turn positive
 *          
 *          Underwater Applications:
 *          - Inspection camera positioning
 *          - Survey camera control
 *          - Target tracking and observation
 *          - Independent camera pointing during navigation
 *          
 *          Compilation:
 *          - Only compiled if HAL_MOUNT_ENABLED
 *          - Requires camera mount hardware support
 *          - No-op if mount support not compiled in
 * 
 * @param[in] cmd Mission command containing target gimbal angles in degrees
 * 
 * @note This is a DO command (executes immediately, no verification needed)
 * @note Only compiled if HAL_MOUNT_ENABLED is true
 * @note Fourth parameter (false) indicates angles are not stabilized (unused)
 * @note Actual gimbal movement handled asynchronously by mount driver
 * 
 * @see camera_mount.set_angle_target() for gimbal angle control
 * @see AP_Mount library for mount driver implementation
 * 
 * Source: ArduSub/commands_logic.cpp:695-701
 */
void Sub::do_mount_control(const AP_Mission::Mission_Command& cmd)
{
#if HAL_MOUNT_ENABLED
    camera_mount.set_angle_target(cmd.content.mount_control.roll, cmd.content.mount_control.pitch, cmd.content.mount_control.yaw, false);
#endif
}
