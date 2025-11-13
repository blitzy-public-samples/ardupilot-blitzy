/**
 * @file mode_auto.cpp
 * @brief Implementation of Auto mode for autonomous mission execution
 * 
 * @details This file implements the ModeAuto class which executes uploaded mission
 *          waypoints with autonomous navigation for ground vehicles (Rover). Auto mode
 *          enables the vehicle to follow a pre-programmed mission consisting of waypoints,
 *          navigation commands, and conditional commands uploaded via MAVLink.
 * 
 *          Key Features:
 *          - Mission waypoint navigation with automatic path following
 *          - Support for various mission command types (NAV_WAYPOINT, NAV_LOITER, NAV_RETURN_TO_LAUNCH, etc.)
 *          - Auto-kickstart feature to delay throttle until vehicle is aligned with target
 *          - Mission resume capability based on MIS_RESTART parameter
 *          - Integration with external navigation systems via NAV_GUIDED_ENABLE
 *          - Support for Lua scripting via NAV_SCRIPT_TIME commands
 *          - Configurable mission completion behavior (hold, loiter, switch to manual/acro)
 * 
 *          State Machine:
 *          The Auto mode operates through submodes that handle different navigation states:
 *          - WP: Waypoint navigation using path following controller
 *          - HeadingAndSpeed: Turn to heading and achieve target speed
 *          - RTL: Return to launch
 *          - Loiter: Station keeping at current location
 *          - Guided: External navigation control via MAVLink
 *          - Stop: Vehicle stopped
 *          - NavScriptTime: Lua script control
 *          - Circle: Circular loiter pattern
 * 
 * @note Missions are uploaded/downloaded via MAVLink mission protocol
 * @note Mission resumption behavior controlled by MIS_RESTART parameter
 * @note Auto-kickstart (AUTO_KICKSTART parameter) delays throttle until heading aligned
 * 
 * @see AP_Mission for mission storage and command execution
 * @see AR_WPNav for waypoint navigation controller
 * 
 * Source: Rover/mode_auto.cpp
 */

#include "Rover.h"

// Maximum time between sending position targets to external navigation controller (milliseconds)
#define AUTO_GUIDED_SEND_TARGET_MS 1000

/**
 * @brief Enter Auto mode and initialize mission execution
 * 
 * @details Initializes the Auto mode state machine and prepares for mission execution.
 *          This function is called when the vehicle transitions into Auto mode.
 * 
 *          Initialization sequence:
 *          1. Verify mission is present (uploaded and valid)
 *          2. Initialize waypoint navigation controller (AR_WPNav)
 *          3. Reset auto-kickstart trigger state
 *          4. Clear any previous guided mode limits
 *          5. Set initial submode based on vehicle type:
 *             - Boats: Start in Loiter submode (station keeping)
 *             - Land vehicles: Start in Stop submode (stationary)
 *          6. Set waiting_to_start flag (mission will begin once origin is set)
 * 
 *          Mission Start Behavior:
 *          The mission does not immediately start upon entering Auto mode. The actual
 *          mission execution begins in update() once an origin is available from AHRS.
 *          This ensures the vehicle has a valid navigation reference frame before
 *          attempting autonomous navigation.
 * 
 *          Mission Resume:
 *          Mission resumption is handled by mission.start_or_resume() in update(),
 *          which respects the MIS_RESTART parameter to either:
 *          - Resume from last commanded waypoint (MIS_RESTART=0)
 *          - Restart mission from beginning (MIS_RESTART=1)
 * 
 * @return true if successfully entered Auto mode, false if no mission present
 * 
 * @note Returns false if no mission is present, preventing mode change
 * @note Boats prefer loiter over stop to maintain position against currents
 * @note Auto-kickstart (if configured) prevents throttle until trigger condition met
 * 
 * @see update() for mission start sequence
 * @see mission.start_or_resume() for mission resumption logic
 */
bool ModeAuto::_enter()
{
    // Fail to enter auto if no mission commands
    // Mission must be uploaded via MAVLink before Auto mode can be used
    if (!mission.present()) {
        GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "No Mission. Can't set AUTO.");
        return false;
    }

    // Initialize waypoint navigation library
    // Resets path following controller state and navigation parameters
    g2.wp_nav.init();

    // Reset auto-kickstart trigger
    // If AUTO_KICKSTART is configured, vehicle won't apply throttle until triggered
    auto_triggered = false;

    // Clear guided limits from any previous guided operations
    // Ensures fresh state if mission contains NAV_GUIDED_ENABLE commands
    rover.mode_guided.limit_clear();

    // Initialize submode to stop or loiter based on vehicle type
    // Boats benefit from active station-keeping to counteract currents
    // Land vehicles simply stop and wait for mission to start
    if (rover.is_boat()) {
        if (!start_loiter()) {
            start_stop();
        }
    } else {
        start_stop();
    }

    // Set flag to start mission once origin is available
    // Mission execution begins in update() after AHRS origin is established
    waiting_to_start = true;

    return true;
}

/**
 * @brief Exit Auto mode and stop mission execution
 * 
 * @details Called when the vehicle transitions out of Auto mode to another flight mode.
 *          Stops the mission if currently running, preserving the current mission index
 *          for potential resumption if Auto mode is re-entered.
 * 
 * @note Mission progress is preserved - re-entering Auto respects MIS_RESTART parameter
 * @note Does not clear the mission from memory, only stops execution
 */
void ModeAuto::_exit()
{
    // Stop running the mission
    // Mission index is preserved for potential resumption
    if (mission.state() == AP_Mission::MISSION_RUNNING) {
        mission.stop();
    }
}

/**
 * @brief Main update loop for Auto mode - executes mission commands and navigates autonomously
 * 
 * @details Called at main loop rate (typically 50Hz) to execute the current mission,
 *          navigate to waypoints, and handle mission state changes.
 * 
 *          Update Sequence:
 *          1. Safety check: Stop vehicle if disarmed and mission cleared
 *          2. Mission start: Wait for AHRS origin before starting mission execution
 *          3. Mission change detection: Restart current waypoint if mission modified
 *          4. Mission update: Process current command and advance to next when complete
 *          5. Submode execution: Run appropriate navigation controller for current submode
 * 
 *          Mission Start Logic:
 *          On first entry to Auto mode, waiting_to_start flag is set. Mission execution
 *          is delayed until AHRS has a valid origin (navigation reference frame). Once
 *          origin is available, mission.start_or_resume() begins mission execution based
 *          on MIS_RESTART parameter.
 * 
 *          Mission Change Handling:
 *          If mission is modified while running (e.g., new mission uploaded via MAVLink),
 *          the change is detected and the current navigation command is restarted if it's
 *          a waypoint command. This ensures the vehicle navigates to the new waypoint
 *          location rather than continuing to the old one.
 * 
 *          Submode Navigation:
 *          - WP: Follow path to waypoint using L1 controller, boats loiter at waypoint
 *          - HeadingAndSpeed: Turn to desired heading while maintaining target speed
 *          - RTL: Delegate to RTL mode for return-to-launch behavior
 *          - Loiter: Delegate to Loiter mode for station keeping
 *          - Guided: External navigation control via MAVLink position targets
 *          - Stop: Hold vehicle stationary
 *          - NavScriptTime: Lua script control of vehicle
 *          - Circle: Circular loiter pattern
 * 
 * @note Called at main loop rate (typically 50Hz)
 * @note Mission must have valid origin before execution begins
 * @note Boats automatically loiter at waypoints instead of stopping
 * 
 * @see mission.update() for mission command processing
 * @see navigate_to_waypoint() for waypoint following controller
 */
void ModeAuto::update()
{
    // Safety check: If mission has been cleared while disarmed in AUTO mode,
    // stop the vehicle. Mission must be reloaded and mode cycled to run again.
    if (!hal.util->get_soft_armed() && !mission.present()) {
        start_stop();
    }

    // Mission start sequence - wait for valid navigation origin
    if (waiting_to_start) {
        // Don't start the mission until we have an origin from AHRS
        // Origin provides the navigation reference frame for lat/lon/alt conversions
        Location loc;
        if (ahrs.get_origin(loc)) {
            // Start or resume the mission based on MIS_RESTART parameter:
            // MIS_RESTART=0: Resume from last commanded waypoint
            // MIS_RESTART=1: Restart mission from beginning
            mission.start_or_resume();
            waiting_to_start = false;

            // Initialize mission change detection to baseline current mission state
            IGNORE_RETURN(mis_change_detector.check_for_mission_change());
        }
    } else {
        // Check for mission changes (new mission uploaded via MAVLink)
        if (mis_change_detector.check_for_mission_change()) {
            // If mission is running and we're navigating to a waypoint,
            // restart the current command to navigate to the new waypoint location
            if ((mission.state() == AP_Mission::MISSION_RUNNING) && (_submode == SubMode::WP)) {
                if (mission.restart_current_nav_cmd()) {
                    GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Auto mission changed, restarted command");
                } else {
                    // Failed to restart mission for some reason
                    GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Auto mission changed but failed to restart command");
                }
            }
        }

        // Update mission state machine: execute current command, check for completion,
        // advance to next command when current completes
        mission.update();
    }

    // Execute navigation controller for current submode
    switch (_submode) {
        case SubMode::WP:
        {
            // Waypoint navigation submode - follow path to target waypoint
            // Boats loiter once the waypoint is reached for station keeping
            bool keep_navigating = true;
            if (rover.is_boat() && g2.wp_nav.reached_destination() && !g2.wp_nav.is_fast_waypoint()) {
                keep_navigating = !start_loiter();
            }

            // Update waypoint navigation controller (L1 path following)
            if (keep_navigating) {
                navigate_to_waypoint();
            }
            break;
        }

        case SubMode::HeadingAndSpeed:
        {
            // Turn to desired heading while achieving target speed
            // Used by NAV_SET_YAW_SPEED command
            if (!_reached_heading) {
                // Run steering and throttle controllers to turn to target heading
                // Steering: Calculate steering output to achieve _desired_yaw_cd
                calc_steering_to_heading(_desired_yaw_cd);
                // Throttle: Maintain _desired_speed with avoidance enabled
                calc_throttle(calc_speed_nudge(_desired_speed, is_negative(_desired_speed)), true);
                // Check if we have reached within 5 degrees (500 centidegrees) of target heading
                _reached_heading = (fabsf(_desired_yaw_cd - ahrs.yaw_sensor) < 500);
            } else {
                // We have reached the destination heading, stop or loiter
                if (rover.is_boat()) {
                    if (!start_loiter()) {
                        stop_vehicle();
                    }
                } else {
                    stop_vehicle();
                }
            }
            break;
        }

        case SubMode::RTL:
            // Return to launch - delegate to RTL mode
            rover.mode_rtl.update();
            break;

        case SubMode::Loiter:
            // Station keeping - delegate to Loiter mode
            rover.mode_loiter.update();
            break;

        case SubMode::Guided:
        {
            // External navigation control via MAVLink
            // Send location target to offboard navigation system at 1Hz
            send_guided_position_target();
            rover.mode_guided.update();
            break;
        }

        case SubMode::Stop:
            // Hold vehicle stationary
            stop_vehicle();
            break;

        case SubMode::NavScriptTime:
            // Lua script control - delegate to Guided mode for vehicle control
            rover.mode_guided.update();
            break;

        case SubMode::Circle:
            // Circular loiter pattern - delegate to Circle mode
            g2.mode_circle.update();
            break;
    }
}

/**
 * @brief Calculate throttle output with auto-kickstart feature
 * 
 * @details Overrides base Mode::calc_throttle() to implement auto-kickstart functionality.
 *          If auto-kickstart is configured (AUTO_KICKSTART parameter or trigger pin),
 *          throttle is held at zero until the trigger condition is met. This prevents
 *          the vehicle from moving until intentionally started, even in Auto mode.
 * 
 *          Auto-Kickstart Trigger Conditions:
 *          - AUTO_KICKSTART parameter: Vehicle acceleration exceeds threshold (m/s²)
 *          - Auto trigger pin: Digital pin pulled low
 *          - Both disabled: Throttle immediately enabled
 * 
 *          Once triggered, auto_triggered flag is set true and throttle control
 *          passes to normal Mode::calc_throttle() for speed controller operation.
 * 
 * @param[in] target_speed Target speed in m/s (positive forward, negative reverse)
 * @param[in] avoidance_enabled True to enable object avoidance speed limits
 * 
 * @note Called at main loop rate (typically 50Hz) when navigating
 * @note Prevents vehicle motion until trigger condition met
 * @note Trigger pin (if configured) can also stop vehicle by going high
 * 
 * @see check_trigger() for auto-kickstart logic
 * @see Mode::calc_throttle() for base throttle controller
 */
void ModeAuto::calc_throttle(float target_speed, bool avoidance_enabled)
{
    // If not autostarting, set the throttle to minimum (vehicle stopped)
    // check_trigger() implements auto-kickstart feature
    if (!check_trigger()) {
        stop_vehicle();
        return;
    }
    // Auto-kickstart triggered, use normal throttle controller
    Mode::calc_throttle(target_speed, avoidance_enabled);
}

// return heading (in degrees) to target destination (aka waypoint)
float ModeAuto::wp_bearing() const
{
    switch (_submode) {
    case SubMode::WP:
        return g2.wp_nav.wp_bearing_cd() * 0.01f;
    case SubMode::HeadingAndSpeed:
    case SubMode::Stop:
        return 0.0f;
    case SubMode::RTL:
        return rover.mode_rtl.wp_bearing();
    case SubMode::Loiter:
        return rover.mode_loiter.wp_bearing();
    case SubMode::Guided:
    case SubMode::NavScriptTime:
        return rover.mode_guided.wp_bearing();
    case SubMode::Circle:
        return g2.mode_circle.wp_bearing();
    }

    // this line should never be reached
    return 0.0f;
}

// return short-term target heading in degrees (i.e. target heading back to line between waypoints)
float ModeAuto::nav_bearing() const
{
    switch (_submode) {
    case SubMode::WP:
        return g2.wp_nav.nav_bearing_cd() * 0.01f;
    case SubMode::HeadingAndSpeed:
    case SubMode::Stop:
        return 0.0f;
    case SubMode::RTL:
        return rover.mode_rtl.nav_bearing();
    case SubMode::Loiter:
        return rover.mode_loiter.nav_bearing();
    case SubMode::Guided:
    case SubMode::NavScriptTime:
        return rover.mode_guided.nav_bearing();
    case SubMode::Circle:
        return g2.mode_circle.nav_bearing();
    }

    // this line should never be reached
    return 0.0f;
}

// return cross track error (i.e. vehicle's distance from the line between waypoints)
float ModeAuto::crosstrack_error() const
{
    switch (_submode) {
    case SubMode::WP:
        return g2.wp_nav.crosstrack_error();
    case SubMode::HeadingAndSpeed:
    case SubMode::Stop:
        return 0.0f;
    case SubMode::RTL:
        return rover.mode_rtl.crosstrack_error();
    case SubMode::Loiter:
        return rover.mode_loiter.crosstrack_error();
    case SubMode::Guided:
    case SubMode::NavScriptTime:
        return rover.mode_guided.crosstrack_error();
    case SubMode::Circle:
        return g2.mode_circle.crosstrack_error();
    }

    // this line should never be reached
    return 0.0f;
}

// return desired lateral acceleration
float ModeAuto::get_desired_lat_accel() const
{
    switch (_submode) {
    case SubMode::WP:
        return g2.wp_nav.get_lat_accel();
    case SubMode::HeadingAndSpeed:
    case SubMode::Stop:
        return 0.0f;
    case SubMode::RTL:
        return rover.mode_rtl.get_desired_lat_accel();
    case SubMode::Loiter:
        return rover.mode_loiter.get_desired_lat_accel();
    case SubMode::Guided:
    case SubMode::NavScriptTime:
        return rover.mode_guided.get_desired_lat_accel();
    case SubMode::Circle:
        return g2.mode_circle.get_desired_lat_accel();
    }

    // this line should never be reached
    return 0.0f;
}

// return distance (in meters) to destination
float ModeAuto::get_distance_to_destination() const
{
    switch (_submode) {
    case SubMode::WP:
        return _distance_to_destination;
    case SubMode::HeadingAndSpeed:
    case SubMode::Stop:
        // no valid distance so return zero
        return 0.0f;
    case SubMode::RTL:
        return rover.mode_rtl.get_distance_to_destination();
    case SubMode::Loiter:
        return rover.mode_loiter.get_distance_to_destination();
    case SubMode::Guided:
    case SubMode::NavScriptTime:
        return rover.mode_guided.get_distance_to_destination();
    case SubMode::Circle:
        return g2.mode_circle.get_distance_to_destination();
    }

    // this line should never be reached
    return 0.0f;
}

// get desired location
bool ModeAuto::get_desired_location(Location& destination) const
{
    switch (_submode) {
    case SubMode::WP:
        if (g2.wp_nav.is_destination_valid()) {
            destination = g2.wp_nav.get_oa_destination();
            return true;
        }
        return false;
    case SubMode::HeadingAndSpeed:
    case SubMode::Stop:
        // no desired location for this submode
        return false;
    case SubMode::RTL:
        return rover.mode_rtl.get_desired_location(destination);
    case SubMode::Loiter:
        return rover.mode_loiter.get_desired_location(destination);
    case SubMode::Guided:
    case SubMode::NavScriptTime:
        return rover.mode_guided.get_desired_location(destination);
    case SubMode::Circle:
        return g2.mode_circle.get_desired_location(destination);
    }

    // we should never reach here but just in case
    return false;
}

// set desired location to drive to
bool ModeAuto::set_desired_location(const Location &destination, Location next_destination)
{
    // call parent
    if (!Mode::set_desired_location(destination, next_destination)) {
        return false;
    }

    _submode = SubMode::WP;

    return true;
}

// return true if vehicle has reached or even passed destination
bool ModeAuto::reached_destination() const
{
    switch (_submode) {
    case SubMode::WP:
        return g2.wp_nav.reached_destination();
        break;
    case SubMode::HeadingAndSpeed:
    case SubMode::Stop:
        // always return true because this is the safer option to allow missions to continue
        return true;
        break;
    case SubMode::RTL:
        return rover.mode_rtl.reached_destination();
        break;
    case SubMode::Loiter:
        return rover.mode_loiter.reached_destination();
        break;
    case SubMode::Guided:
    case SubMode::NavScriptTime:
        return rover.mode_guided.reached_destination();
    case SubMode::Circle:
        return g2.mode_circle.reached_destination();
    }

    // we should never reach here but just in case, return true to allow missions to continue
    return true;
}

// set desired speed in m/s
bool ModeAuto::set_desired_speed(float speed)
{
    switch (_submode) {
    case SubMode::WP:
    case SubMode::Stop:
        return g2.wp_nav.set_speed_max(speed);
    case SubMode::HeadingAndSpeed:
        _desired_speed = speed;
        return true;
    case SubMode::RTL:
        return rover.mode_rtl.set_desired_speed(speed);
    case SubMode::Loiter:
        return rover.mode_loiter.set_desired_speed(speed);
    case SubMode::Guided:
    case SubMode::NavScriptTime:
        return rover.mode_guided.set_desired_speed(speed);
    case SubMode::Circle:
        return g2.mode_circle.set_desired_speed(speed);
    }
    return false;
}

// start RTL (within auto)
void ModeAuto::start_RTL()
{
    if (rover.mode_rtl.enter()) {
        _submode = SubMode::RTL;
    }
}

// lua scripts use this to retrieve the contents of the active command
bool ModeAuto::nav_script_time(uint16_t &id, uint8_t &cmd, float &arg1, float &arg2, int16_t &arg3, int16_t &arg4)
{
#if AP_SCRIPTING_ENABLED
    if (_submode == SubMode::NavScriptTime) {
        id = nav_scripting.id;
        cmd = nav_scripting.command;
        arg1 = nav_scripting.arg1;
        arg2 = nav_scripting.arg2;
        arg3 = nav_scripting.arg3;
        arg4 = nav_scripting.arg4;
        return true;
    }
#endif
    return false;
}

// lua scripts use this to indicate when they have complete the command
void ModeAuto::nav_script_time_done(uint16_t id)
{
#if AP_SCRIPTING_ENABLED
    if ((_submode == SubMode::NavScriptTime) && (id == nav_scripting.id)) {
        nav_scripting.done = true;
    }
#endif
}

/**
 * @brief Check for auto-kickstart trigger conditions
 * 
 * @details Implements the auto-kickstart feature that delays throttle application in Auto
 *          mode until the vehicle is intentionally started by the user. This prevents
 *          unintended vehicle motion when entering Auto mode.
 * 
 *          Trigger Mechanisms:
 *          1. Digital Pin Trigger (AUTO_TRIGGER_PIN parameter):
 *             - Pin pulled low (0): Auto triggered, vehicle can move
 *             - Pin pulled high (1): Auto disabled, vehicle stopped
 *             - Pin = -1: Trigger pin disabled
 * 
 *          2. Acceleration Kickstart (AUTO_KICKSTART parameter):
 *             - Vehicle pushed/kicked producing X-axis acceleration > AUTO_KICKSTART (m/s²)
 *             - Useful for hand-launching or push-starting vehicles
 *             - AUTO_KICKSTART = 0: Kickstart disabled
 * 
 *          3. No Trigger Configured:
 *             - If both AUTO_TRIGGER_PIN=-1 and AUTO_KICKSTART=0
 *             - Auto immediately triggered on mode entry
 * 
 *          Once triggered, auto_triggered flag is latched true and remains true until:
 *          - Mode is exited and re-entered (resets in _enter())
 *          - Trigger pin goes high (stops vehicle)
 * 
 *          Typical Use Case:
 *          Set AUTO_KICKSTART to 2.0 m/s² to require a push to start the vehicle,
 *          providing a safety interlock that prevents motion until user confirms
 *          the vehicle is ready to navigate autonomously.
 * 
 * @return true if auto is triggered and throttle should be applied, false to hold throttle at zero
 * 
 * @note Called every time calc_throttle() runs (main loop rate, ~50Hz)
 * @note Trigger state latches true once triggered (doesn't require holding)
 * @note Trigger pin can stop vehicle by going high even after initial trigger
 * @note Uses X-axis acceleration (forward/backward) for kickstart detection
 * 
 * @warning Ensure AUTO_KICKSTART threshold is achievable but not too sensitive to prevent false triggers
 * 
 * @see calc_throttle() for throttle control integration
 * @see _enter() for auto_triggered reset
 */
bool ModeAuto::check_trigger(void)
{
    // Check for user pressing the auto trigger pin to off (pin goes high)
    // This can stop the vehicle even after initial trigger
    if (auto_triggered && g.auto_trigger_pin != -1 && rover.check_digital_pin(g.auto_trigger_pin) == 1) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "AUTO triggered off");
        auto_triggered = false;
        return false;
    }

    // If already triggered, return true (latched state)
    // User doesn't need to hold the switch down or maintain acceleration
    if (auto_triggered) {
        return true;
    }

    // Return true if auto trigger and kickstart are both disabled
    // No trigger configured - let's go immediately!
    if (g.auto_trigger_pin == -1 && is_zero(g.auto_kickstart)) {
        auto_triggered = true;
        return true;
    }

    // Check if trigger pin has been pushed (pin pulled low)
    if (g.auto_trigger_pin != -1 && rover.check_digital_pin(g.auto_trigger_pin) == 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Triggered AUTO with pin");
        auto_triggered = true;
        return true;
    }

    // Check if mission is started by giving vehicle a kick with acceleration > AUTO_KICKSTART
    // Uses X-axis (forward/backward) acceleration from IMU
    if (!is_zero(g.auto_kickstart)) {
        const float xaccel = rover.ins.get_accel().x;
        if (xaccel >= g.auto_kickstart) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Triggered AUTO xaccel=%.1f", static_cast<double>(xaccel));
            auto_triggered = true;
            return true;
        }
    }

    // No trigger condition met yet - keep throttle at zero
    return false;
}

bool ModeAuto::start_loiter()
{
    if (rover.mode_loiter.enter()) {
        _submode = SubMode::Loiter;
        return true;
    }
    return false;
}

// hand over control to external navigation controller in AUTO mode
void ModeAuto::start_guided(const Location& loc)
{
    if (rover.mode_guided.enter()) {
        _submode = SubMode::Guided;

        // initialise guided start time and position as reference for limit checking
        rover.mode_guided.limit_init_time_and_location();

        // sanity check target location
        if ((loc.lat != 0) || (loc.lng != 0)) {
            guided_target.loc = loc;
            guided_target.loc.sanitize(rover.current_loc);
            guided_target.valid = true;
        } else {
            guided_target.valid = false;
        }
    }
}

// start stopping vehicle as quickly as possible
void ModeAuto::start_stop()
{
    _submode = SubMode::Stop;
}

// send latest position target to offboard navigation system
/**
 * @brief Send position target to external navigation controller during NAV_GUIDED_ENABLE
 * 
 * @details Sends SET_POSITION_TARGET_GLOBAL_INT MAVLink messages to an offboard
 *          navigation system at 1Hz when in Guided submode. This allows an external
 *          computer (e.g., companion computer) to know where the vehicle is trying
 *          to navigate while it provides control inputs via MAVLink guided commands.
 * 
 *          Use Case:
 *          NAV_GUIDED_ENABLE missions hand control to an external navigation system,
 *          which sends position/velocity/attitude targets via MAVLink. This function
 *          informs the external system of the mission's intended destination so it
 *          can make informed navigation decisions.
 * 
 *          Rate Limiting:
 *          Position targets are sent at maximum 1Hz (every 1000ms) to avoid flooding
 *          the MAVLink channel. The target location remains constant until the mission
 *          provides a new target via set_desired_location().
 * 
 *          Target Selection:
 *          Messages are sent only to MAV_TYPE_ONBOARD_CONTROLLER systems, which are
 *          typically companion computers running navigation software. Ground control
 *          stations are not sent these messages.
 * 
 * @note Only sends when guided_target.valid is true (target location set)
 * @note Sends at maximum 1Hz (AUTO_GUIDED_SEND_TARGET_MS = 1000ms)
 * @note Only sends to MAV_TYPE_ONBOARD_CONTROLLER (companion computers)
 * @note Called from update() when _submode == SubMode::Guided
 * 
 * @see do_nav_guided_enable() for entering guided submode
 * @see ModeGuided for processing external navigation commands
 */
void ModeAuto::send_guided_position_target()
{
    // Only send if we have a valid target location
    if (!guided_target.valid) {
        return;
    }

    // Rate limit to maximum of 1Hz to avoid flooding MAVLink channel
    const uint32_t now_ms = AP_HAL::millis();
    if ((guided_target.last_sent_ms == 0) || (now_ms - guided_target.last_sent_ms > AUTO_GUIDED_SEND_TARGET_MS)) {
        guided_target.last_sent_ms = now_ms;

        // Get system ID and component ID of offboard navigation controller
        // Only send to MAV_TYPE_ONBOARD_CONTROLLER (companion computers)
        uint8_t sysid;
        uint8_t compid;
        mavlink_channel_t chan;
        if (GCS_MAVLINK::find_by_mavtype(MAV_TYPE_ONBOARD_CONTROLLER, sysid, compid, chan)) {
            // Send SET_POSITION_TARGET_GLOBAL_INT message with target location
            gcs().chan(chan-MAVLINK_COMM_0)->send_set_position_target_global_int(sysid, compid, guided_target.loc);
        }
    }

}

/********************************************************************************/
// Command Event Handlers
/********************************************************************************/

/**
 * @brief Start execution of a mission command
 * 
 * @details Called by AP_Mission when a new mission command should be started.
 *          Routes the command to the appropriate handler based on command ID.
 *          This function implements the mission command dispatch logic for Auto mode.
 * 
 *          Supported Navigation Commands (Must):
 *          - MAV_CMD_NAV_WAYPOINT: Navigate to waypoint with optional loiter time
 *          - MAV_CMD_NAV_RETURN_TO_LAUNCH: Return to launch location
 *          - MAV_CMD_NAV_LOITER_UNLIM: Loiter indefinitely at location
 *          - MAV_CMD_NAV_LOITER_TIME: Loiter for specified duration
 *          - MAV_CMD_NAV_LOITER_TURNS: Circle loiter pattern
 *          - MAV_CMD_NAV_GUIDED_ENABLE: Hand control to external navigation system
 *          - MAV_CMD_NAV_SET_YAW_SPEED: Turn to heading and maintain speed
 *          - MAV_CMD_NAV_DELAY: Delay next navigation command
 *          - MAV_CMD_NAV_SCRIPT_TIME: Lua script control (if scripting enabled)
 * 
 *          Supported Conditional Commands (May):
 *          - MAV_CMD_CONDITION_DELAY: Wait for specified time
 *          - MAV_CMD_CONDITION_DISTANCE: Wait until within distance of target
 * 
 *          Supported Do Commands (Now):
 *          - MAV_CMD_DO_CHANGE_SPEED: Change target speed
 *          - MAV_CMD_DO_SET_HOME: Set home location
 *          - MAV_CMD_DO_SET_ROI*: Set region of interest for camera/mount
 *          - MAV_CMD_DO_SET_REVERSE: Enable/disable reverse driving
 *          - MAV_CMD_DO_GUIDED_LIMITS: Set guided mode timeout and position limits
 * 
 *          Command Types:
 *          - Nav commands: Block mission progression until complete (verified by verify_command)
 *          - Condition commands: Gate execution of subsequent commands until condition met
 *          - Do commands: Execute immediately, don't block mission progression
 * 
 * @param[in] cmd Mission command structure containing command ID and parameters
 * 
 * @return true if command was handled successfully, false if command not recognized
 * 
 * @note Called by AP_Mission when advancing to new command
 * @note Nav commands transition to appropriate submode (WP, RTL, Guided, etc.)
 * @note Do commands execute immediately without blocking mission progression
 * @note Unrecognized commands return false to allow mission to continue
 * 
 * @see verify_command() for checking command completion
 * @see AP_Mission::update() for mission state machine
 */
bool ModeAuto::start_command(const AP_Mission::Mission_Command& cmd)
{
    switch (cmd.id) {
    // Navigation Commands (Must) - These block mission progression until complete
    
    case MAV_CMD_NAV_WAYPOINT:  // Navigate to waypoint with path following
        return do_nav_wp(cmd, false);

    case MAV_CMD_NAV_RETURN_TO_LAUNCH:  // Return to launch location
        do_RTL();
        break;

    case MAV_CMD_NAV_LOITER_UNLIM:  // Loiter indefinitely at location
    case MAV_CMD_NAV_LOITER_TIME:   // Loiter for specified time (param1 = seconds)
        return do_nav_wp(cmd, true);

    case MAV_CMD_NAV_LOITER_TURNS:  // Circle loiter pattern (param1 = turns, param2 = radius)
        return do_circle(cmd);

    case MAV_CMD_NAV_GUIDED_ENABLE:  // Hand control to external navigation computer via MAVLink
        do_nav_guided_enable(cmd);
        break;

    case MAV_CMD_NAV_SET_YAW_SPEED:  // Turn to heading (param1 = angle deg) at speed (param2 = m/s)
        do_nav_set_yaw_speed(cmd);
        break;

    case MAV_CMD_NAV_DELAY:  // Delay next navigation command for time or until absolute time
        do_nav_delay(cmd);
        break;

#if AP_SCRIPTING_ENABLED
    case MAV_CMD_NAV_SCRIPT_TIME:  // Lua script control with timeout
        do_nav_script_time(cmd);
        break;
#endif

    // Conditional Commands (May) - Gate subsequent commands until condition met
    
    case MAV_CMD_CONDITION_DELAY:  // Delay for specified time (param1 = seconds)
        do_wait_delay(cmd);
        break;

    case MAV_CMD_CONDITION_DISTANCE:  // Wait until within distance of target (param1 = meters)
        do_within_distance(cmd);
        break;

    // Do Commands (Now) - Execute immediately without blocking mission
    
    case MAV_CMD_DO_CHANGE_SPEED:  // Change target speed (param1 = m/s)
        do_change_speed(cmd);
        break;

    case MAV_CMD_DO_SET_HOME:  // Set home location (param1=1: current location, else: command location)
        do_set_home(cmd);
        break;

#if HAL_MOUNT_ENABLED
    // Region of Interest commands - Point camera/mount at location
    // Sets the region of interest (ROI) for camera mount control
    // If location is (0,0,0), disables ROI tracking
    case MAV_CMD_DO_SET_ROI_LOCATION:
    case MAV_CMD_DO_SET_ROI_NONE:
    case MAV_CMD_DO_SET_ROI:
        if (cmd.content.location.alt == 0 && cmd.content.location.lat == 0 && cmd.content.location.lng == 0) {
            // Zero location: switch off camera tracking if enabled
            if (rover.camera_mount.get_mode() == MAV_MOUNT_MODE_GPS_POINT) {
                rover.camera_mount.set_mode_to_default();
            }
        } else {
            // Non-zero location: point camera/mount at target
            rover.camera_mount.set_roi_target(cmd.content.location);
        }
        break;
#endif

    case MAV_CMD_DO_SET_REVERSE:  // Enable/disable reverse driving (param1=1: reverse, 0: forward)
        do_set_reverse(cmd);
        break;

    case MAV_CMD_DO_GUIDED_LIMITS:  // Set guided mode timeout (param1=seconds) and distance limit (param2=meters)
        do_guided_limits(cmd);
        break;

    default:
        // Return false for unhandled commands to allow mission to continue
        return false;
    }

    // If we got this far we must have been successful
    return true;
}

// exit_mission - callback function called from ap-mission when the mission has completed
void ModeAuto::exit_mission()
{
    // play a tone
    AP_Notify::events.mission_complete = 1;
    // send message
    GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Mission Complete");

    switch ((DoneBehaviour)g2.mis_done_behave) {
    case DoneBehaviour::HOLD:
        // the default "start_stop" behaviour is used
        break;
    case DoneBehaviour::LOITER:
        if (start_loiter()) {
            return;
        }
        break;
    case DoneBehaviour::ACRO:
        if (rover.set_mode(rover.mode_acro, ModeReason::MISSION_END)) {
            return;
        }
        break;
    case DoneBehaviour::MANUAL:
        if (rover.set_mode(rover.mode_manual, ModeReason::MISSION_END)) {
            return;
        }
        break;
    }

    start_stop();
}

// verify_command_callback - callback function called from ap-mission at 10hz or higher when a command is being run
//      we double check that the flight mode is AUTO to avoid the possibility of ap-mission triggering actions while we're not in AUTO mode
bool ModeAuto::verify_command_callback(const AP_Mission::Mission_Command& cmd)
{
    const bool cmd_complete = verify_command(cmd);

    // send message to GCS
    if (cmd_complete) {
        gcs().send_mission_item_reached_message(cmd.index);
    }

    return cmd_complete;
}

/*******************************************************************************
 * Verify Command Handlers
 * 
 * Each type of mission element has a "verify" operation that checks whether
 * the command has completed. The verify operation returns true when the mission
 * element has completed and the mission should advance to the next element.
 * 
 * Verification Logic:
 * - Nav commands: Check if waypoint reached, loiter time elapsed, etc.
 * - Condition commands: Check if condition is met (delay expired, within distance)
 * - Do commands: Always return true (executed immediately, no verification needed)
 * 
 * Unknown commands return true to allow mission progression rather than blocking.
 *******************************************************************************/

/**
 * @brief Verify if current mission command has completed
 * 
 * @details Routes the verification to the appropriate handler based on command ID.
 *          Called by AP_Mission at main loop rate to check command completion status.
 * 
 *          Nav Command Verification:
 *          - MAV_CMD_NAV_WAYPOINT: Waypoint reached within acceptance radius
 *          - MAV_CMD_NAV_RETURN_TO_LAUNCH: RTL complete (home reached)
 *          - MAV_CMD_NAV_LOITER_UNLIM: Never completes (requires mode change)
 *          - MAV_CMD_NAV_LOITER_TURNS: Required turns completed
 *          - MAV_CMD_NAV_LOITER_TIME: Loiter time expired
 *          - MAV_CMD_NAV_GUIDED_ENABLE: Guided mode timeout or disabled
 *          - MAV_CMD_NAV_DELAY: Delay time expired or absolute time reached
 *          - MAV_CMD_NAV_SCRIPT_TIME: Script timeout or script signals complete
 *          - MAV_CMD_NAV_SET_YAW_SPEED: Target heading reached
 * 
 *          Condition Command Verification:
 *          - MAV_CMD_CONDITION_DELAY: Delay time expired
 *          - MAV_CMD_CONDITION_DISTANCE: Within specified distance of target
 * 
 *          Do Command Verification:
 *          - All DO commands: Always return true (execute immediately, no waiting)
 * 
 *          Mission Progression:
 *          When verify_command returns true, AP_Mission advances to next command.
 *          Nav commands block progression until complete, while Do commands
 *          execute immediately without blocking.
 * 
 * @param[in] cmd Mission command structure to verify
 * 
 * @return true if command has completed, false if still executing
 * 
 * @note Called at main loop rate (typically 50Hz) for active command
 * @note Unknown commands return true to prevent mission blocking
 * @note Do commands always return true (immediate execution)
 * 
 * @see start_command() for command initialization
 * @see AP_Mission::update() for mission state machine
 */
bool ModeAuto::verify_command(const AP_Mission::Mission_Command& cmd)
{
    switch (cmd.id) {
    // Nav Command Verification - These block until complete
    
    case MAV_CMD_NAV_WAYPOINT:  // Waypoint reached within acceptance radius?
        return verify_nav_wp(cmd);

    case MAV_CMD_NAV_RETURN_TO_LAUNCH:  // RTL complete (home reached)?
        return verify_RTL();

    case MAV_CMD_NAV_LOITER_UNLIM:  // Never completes (infinite loiter)
        return verify_loiter_unlimited(cmd);

    case MAV_CMD_NAV_LOITER_TURNS:  // Required turns completed?
        return verify_circle(cmd);

    case MAV_CMD_NAV_LOITER_TIME:  // Loiter time expired?
        return verify_loiter_time(cmd);

    case MAV_CMD_NAV_GUIDED_ENABLE:  // Guided timeout or disabled?
        return verify_nav_guided_enable(cmd);

    case MAV_CMD_NAV_DELAY:  // Delay time expired or absolute time reached?
        return verify_nav_delay(cmd);

#if AP_SCRIPTING_ENABLED
    case MAV_CMD_NAV_SCRIPT_TIME:  // Script timeout or completion?
        return verify_nav_script_time();
#endif

    case MAV_CMD_NAV_SET_YAW_SPEED:  // Target heading reached?
        return verify_nav_set_yaw_speed();

    // Condition Command Verification - Gate subsequent commands
    
    case MAV_CMD_CONDITION_DELAY:  // Delay time expired?
        return verify_wait_delay();

    case MAV_CMD_CONDITION_DISTANCE:  // Within distance of target?
        return verify_within_distance();

    // Do Commands - Always complete immediately (no verification needed)
    case MAV_CMD_DO_CHANGE_SPEED:
    case MAV_CMD_DO_SET_HOME:
    case MAV_CMD_DO_SET_CAM_TRIGG_DIST:
    case MAV_CMD_DO_SET_ROI_LOCATION:
    case MAV_CMD_DO_SET_ROI_NONE:
    case MAV_CMD_DO_SET_ROI:
    case MAV_CMD_DO_SET_REVERSE:
    case MAV_CMD_DO_FENCE_ENABLE:
    case MAV_CMD_DO_GUIDED_LIMITS:
        return true;

    default:
        // Unknown command - report warning and return true to continue mission
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Skipping invalid cmd #%i", cmd.id);
        // Return true to move on to next command rather than blocking mission
        return true;
    }
}

/********************************************************************************/
//  Nav (Must) commands
/********************************************************************************/

void ModeAuto::do_RTL(void)
{
    // start rtl in auto mode
    start_RTL();
}

/**
 * @brief Start navigation to waypoint
 * 
 * @details Initializes navigation to a target waypoint using L1 path following controller.
 *          Handles both single waypoint navigation and corner cutting optimization when
 *          multiple sequential waypoints exist.
 * 
 *          Waypoint Processing:
 *          1. Sanitize target location (ensure valid altitude, wrap longitude)
 *          2. Extract loiter duration from p1 parameter (0 = no loiter)
 *          3. Determine navigation strategy:
 *             - Corner cutting: If next waypoint exists and no loiter required
 *             - Stop at waypoint: If loiter required or last waypoint in mission
 *          4. Configure waypoint navigation controller with target(s)
 *          5. Reset waypoint reached flag
 * 
 *          Corner Cutting Optimization:
 *          When navigating between multiple waypoints without loitering, the controller
 *          is given both the current target and next target. This enables the L1
 *          controller to smooth the path and begin turning toward the next waypoint
 *          before fully reaching the current one, improving navigation efficiency.
 * 
 *          Loiter Behavior:
 *          If param1 > 0, vehicle will loiter at waypoint for specified seconds before
 *          advancing to next command. Loiter time is enforced by verify_nav_wp().
 * 
 * @param[in] cmd Mission command containing waypoint location and loiter time
 * @param[in] always_stop_at_destination True to disable corner cutting (for loiter commands)
 * 
 * @return true if waypoint navigation started successfully, false if location invalid
 * 
 * @note MAV_CMD_NAV_WAYPOINT: param1 = loiter time (seconds), location = target
 * @note MAV_CMD_NAV_LOITER_TIME: Always stops at destination for timed loiter
 * @note Corner cutting only enabled if no loiter time and next nav command exists
 * @note Location sanitization ensures altitude is valid and longitude wrapped to ±180°
 * 
 * @see set_desired_location() for navigation controller setup
 * @see verify_nav_wp() for waypoint reached detection
 * @see AR_WPNav for L1 path following controller
 */
bool ModeAuto::do_nav_wp(const AP_Mission::Mission_Command& cmd, bool always_stop_at_destination)
{
    // Retrieve target location and sanitize to ensure valid altitude and wrapped longitude
    Location cmdloc = cmd.content.location;
    cmdloc.sanitize(rover.current_loc);

    // Extract loiter duration from param1 (seconds), ensure non-negative
    loiter_duration = ((int16_t) cmd.p1 < 0) ? 0 : cmd.p1;
    loiter_start_time = 0;
    if (loiter_duration > 0) {
        // If loiter time specified, must stop at waypoint (disable corner cutting)
        always_stop_at_destination = true;
    }

    // Determine navigation strategy: corner cutting vs stop at waypoint
    AP_Mission::Mission_Command next_cmd;
    if (always_stop_at_destination || !mission.get_next_nav_cmd(cmd.index+1, next_cmd)) {
        // Single destination mode: Stop at this waypoint
        // Used when: loiter required, last waypoint, or explicit stop requested
        if (!set_desired_location(cmdloc)) {
            return false;
        }
    } else {
        // Corner cutting mode: Provide current AND next waypoint to L1 controller
        // Controller will smooth path and begin turn before reaching current waypoint
        // Retrieve and sanitize next destination location
        Location next_cmdloc = next_cmd.content.location;
        next_cmdloc.sanitize(cmdloc);
        if (!set_desired_location(cmdloc, next_cmdloc)) {
            return false;
        }
    }

    // Reset waypoint reached flag - just starting navigation to this waypoint
    previously_reached_wp = false;

    return true;
}

// do_nav_delay - Delay the next navigation command
void ModeAuto::do_nav_delay(const AP_Mission::Mission_Command& cmd)
{
    nav_delay_time_start_ms = millis();

    // boats loiter, cars and balancebots stop
    if (rover.is_boat()) {
        if (!start_loiter()) {
            start_stop();
        }
    } else {
        start_stop();
    }

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
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Delaying %u sec", (unsigned)(nav_delay_time_max_ms/1000));
}

// start guided within auto to allow external navigation system to control vehicle
void ModeAuto::do_nav_guided_enable(const AP_Mission::Mission_Command& cmd)
{
    if (cmd.p1 > 0) {
        start_guided(cmd.content.location);
    }
}

// do_set_yaw_speed - turn to a specified heading and achieve a given speed
void ModeAuto::do_nav_set_yaw_speed(const AP_Mission::Mission_Command& cmd)
{
    float desired_heading_cd;

    // get final angle, 1 = Relative, 0 = Absolute
    if (cmd.content.set_yaw_speed.relative_angle > 0) {
        // relative angle
        desired_heading_cd = wrap_180_cd(ahrs.yaw_sensor + cmd.content.set_yaw_speed.angle_deg * 100.0f);
    } else {
        // absolute angle
        desired_heading_cd = cmd.content.set_yaw_speed.angle_deg * 100.0f;
    }

    // set targets
    const float speed_max = g2.wp_nav.get_default_speed();
    _desired_speed = constrain_float(cmd.content.set_yaw_speed.speed, -speed_max, speed_max);
    _desired_yaw_cd = desired_heading_cd;
    _reached_heading = false;
    _submode = SubMode::HeadingAndSpeed;
}

/********************************************************************************/
//  Verify Nav (Must) commands
/********************************************************************************/
bool ModeAuto::verify_nav_wp(const AP_Mission::Mission_Command& cmd)
{
    // exit immediately if we haven't reached the destination
    if (!reached_destination()) {
        return false;
    }

    // Check if this is the first time we have noticed reaching the waypoint
    if (!previously_reached_wp) {
        previously_reached_wp = true;

        // check if we are loitering at this waypoint - the message sent to the GCS is different
        if (loiter_duration > 0) {
            // send message including loiter time
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Reached waypoint #%u. Loiter for %u seconds",
                            (unsigned int)cmd.index,
                            (unsigned int)loiter_duration);
            // record the current time i.e. start timer
            loiter_start_time = millis();
        } else {
            // send simpler message to GCS
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Reached waypoint #%u", (unsigned int)cmd.index);
        }
    }

    // Check if we have loitered long enough
    if (loiter_duration == 0) {
        return true;
    } else {
        return (((millis() - loiter_start_time) / 1000) >= loiter_duration);
    }
}

// verify_nav_delay - check if we have waited long enough
bool ModeAuto::verify_nav_delay(const AP_Mission::Mission_Command& cmd)
{
    if (millis() - nav_delay_time_start_ms > nav_delay_time_max_ms) {
        nav_delay_time_max_ms = 0;
        return true;
    }

    return false;
}

bool ModeAuto::verify_RTL() const
{
    return reached_destination();
}

bool ModeAuto::verify_loiter_unlimited(const AP_Mission::Mission_Command& cmd)
{
    verify_nav_wp(cmd);
    return false;
}

// verify_loiter_time - check if we have loitered long enough
bool ModeAuto::verify_loiter_time(const AP_Mission::Mission_Command& cmd)
{
    const bool result = verify_nav_wp(cmd);
    if (result) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Finished active loiter");
    }
    return result;
}

// check if guided has completed
bool ModeAuto::verify_nav_guided_enable(const AP_Mission::Mission_Command& cmd)
{
    // if we failed to enter guided or this command disables guided
    // return true so we move to next command
    if (_submode != SubMode::Guided || cmd.p1 == 0) {
        return true;
    }

    // if a location target was set, return true once vehicle is close
    if (guided_target.valid) {
        if (rover.current_loc.get_distance(guided_target.loc) <= g2.wp_nav.get_radius()) {
            return true;
        }
    }

    // guided command complete once a limit is breached
    return rover.mode_guided.limit_breached();
}

// verify_yaw - return true if we have reached the desired heading
bool ModeAuto::verify_nav_set_yaw_speed()
{
    if (_submode == SubMode::HeadingAndSpeed) {
        return _reached_heading;
    }
    // we should never reach here but just in case, return true to allow missions to continue
    return true;
}

bool ModeAuto::do_circle(const AP_Mission::Mission_Command& cmd)
{
    // retrieve and sanitize target location
    Location circle_center = cmd.content.location;
    circle_center.sanitize(rover.current_loc);

    // calculate radius
    uint16_t circle_radius_m = HIGHBYTE(cmd.p1); // circle radius held in high byte of p1
    if (cmd.id == MAV_CMD_NAV_LOITER_TURNS &&
        cmd.type_specific_bits & (1U << 0)) {
        // special storage handling allows for larger radii
        circle_radius_m *= 10;
    }

    // initialise circle mode
    if (g2.mode_circle.set_center(circle_center, circle_radius_m, cmd.content.location.loiter_ccw)) {
        _submode = SubMode::Circle;
        return true;
    }
    return false;
}

bool ModeAuto::verify_circle(const AP_Mission::Mission_Command& cmd)
{
    const float turns = cmd.get_loiter_turns();
    // check if we have completed circling
    return ((g2.mode_circle.get_angle_total_rad() / M_2PI) >= turns);
}

/********************************************************************************/
//  Condition (May) commands
/********************************************************************************/

void ModeAuto::do_wait_delay(const AP_Mission::Mission_Command& cmd)
{
    condition_start = millis();
    condition_value = static_cast<int32_t>(cmd.content.delay.seconds * 1000);  // convert seconds to milliseconds
}

void ModeAuto::do_within_distance(const AP_Mission::Mission_Command& cmd)
{
    condition_value = cmd.content.distance.meters;
}

/********************************************************************************/
// Verify Condition (May) commands
/********************************************************************************/

bool ModeAuto::verify_wait_delay()
{
    if (static_cast<uint32_t>(millis() - condition_start) > static_cast<uint32_t>(condition_value)) {
        condition_value = 0;
        return true;
    }
    return false;
}

bool ModeAuto::verify_within_distance()
{
    if (get_distance_to_destination() < condition_value) {
        condition_value = 0;
        return true;
    }
    return false;
}


/********************************************************************************/
//  Do (Now) commands
/********************************************************************************/

void ModeAuto::do_change_speed(const AP_Mission::Mission_Command& cmd)
{
    // set speed for active mode
    if (set_desired_speed(cmd.content.speed.target_ms)) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "speed: %.1f m/s", static_cast<double>(cmd.content.speed.target_ms));
    }
}

void ModeAuto::do_set_home(const AP_Mission::Mission_Command& cmd)
{
    if (cmd.p1 == 1 && rover.have_position) {
        if (!rover.set_home_to_current_location(false)) {
            // ignored...
        }
    } else {
        if (!rover.set_home(cmd.content.location, false)) {
            // ignored...
        }
    }
}

void ModeAuto::do_set_reverse(const AP_Mission::Mission_Command& cmd)
{
    set_reversed(cmd.p1 == 1);
}

// set timeout and position limits for guided within auto
void ModeAuto::do_guided_limits(const AP_Mission::Mission_Command& cmd)
{
    rover.mode_guided.limit_set(
        cmd.p1 * 1000, // convert seconds to ms
        cmd.content.guided_limits.horiz_max);
}

#if AP_SCRIPTING_ENABLED
// start accepting position, velocity and acceleration targets from lua scripts
void ModeAuto::do_nav_script_time(const AP_Mission::Mission_Command& cmd)
{
    // call regular guided flight mode initialisation
    if (rover.mode_guided.enter()) {
        _submode = SubMode::NavScriptTime;
        nav_scripting.done = false;
        nav_scripting.id++;
        nav_scripting.start_ms = millis();
        nav_scripting.command = cmd.content.nav_script_time.command;
        nav_scripting.timeout_s = cmd.content.nav_script_time.timeout_s;
        nav_scripting.arg1 = cmd.content.nav_script_time.arg1.get();
        nav_scripting.arg2 = cmd.content.nav_script_time.arg2.get();
        nav_scripting.arg3 = cmd.content.nav_script_time.arg3;
        nav_scripting.arg4 = cmd.content.nav_script_time.arg4;
    } else {
        // for safety we set nav_scripting to done to protect against the mission getting stuck
        nav_scripting.done = true;
    }
}

// check if verify_nav_script_time command has completed
bool ModeAuto::verify_nav_script_time()
{
    // if done or timeout then return true
    if (nav_scripting.done ||
        ((nav_scripting.timeout_s > 0) &&
         (AP_HAL::millis() - nav_scripting.start_ms) > (nav_scripting.timeout_s * 1000))) {
        return true;
    }
    return false;
}
#endif
