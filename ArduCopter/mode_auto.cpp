#include "Copter.h"

#if MODE_AUTO_ENABLED

/**
 * @file mode_auto.cpp
 * @brief Auto flight mode implementation for ArduCopter
 * 
 * @details This file implements autonomous mission execution (Auto mode) for multicopter vehicles.
 *          Auto mode is the primary flight mode for executing pre-programmed missions stored in
 *          the AP_Mission library. The mode handles waypoint navigation, takeoff, landing,
 *          circles/loiter patterns, and various conditional/immediate commands.
 * 
 *          Auto mode integrates with:
 *          - AP_Mission library: Mission command storage, sequencing, and state management
 *          - AC_WPNav: Waypoint navigation controller for position control
 *          - AC_PosControl: Low-level position and velocity control
 *          - AC_AttitudeControl: Attitude stabilization and thrust vectoring
 * 
 *          Supported MAVLink mission commands include:
 *          Navigation commands:
 *          - NAV_WAYPOINT: Fly to waypoint with optional delay
 *          - NAV_SPLINE_WAYPOINT: Smooth spline path through waypoint
 *          - NAV_TAKEOFF/NAV_VTOL_TAKEOFF: Vertical takeoff to altitude
 *          - NAV_LAND/NAV_VTOL_LAND: Autonomous landing with optional precision landing
 *          - NAV_RETURN_TO_LAUNCH: Return to home position
 *          - NAV_LOITER_UNLIM: Loiter indefinitely at location
 *          - NAV_LOITER_TIME: Loiter for specified duration
 *          - NAV_LOITER_TURNS: Fly circles around point
 *          - NAV_LOITER_TO_ALT: Loiter while climbing/descending to altitude
 *          - NAV_GUIDED_ENABLE: Accept external navigation commands
 *          - NAV_DELAY: Delay next navigation command
 *          - NAV_PAYLOAD_PLACE: Autonomous payload placement
 *          - NAV_SCRIPT_TIME: Lua script-controlled navigation
 *          - NAV_ATTITUDE_TIME: Maintain attitude for duration
 * 
 *          Conditional commands (execute when condition met):
 *          - CONDITION_DELAY: Wait for time period
 *          - CONDITION_DISTANCE: Wait until within distance of target
 *          - CONDITION_YAW: Rotate to specified heading
 * 
 *          DO commands (execute immediately):
 *          - DO_CHANGE_SPEED: Modify horizontal/vertical speeds
 *          - DO_SET_HOME: Update home position
 *          - DO_SET_ROI: Point camera/vehicle at region of interest
 *          - DO_MOUNT_CONTROL: Position camera gimbal
 *          - DO_GUIDED_LIMITS: Set limits for guided mode
 *          - DO_LAND_START: Mark start of landing sequence
 *          - DO_RETURN_PATH_START: Mark start of return path
 * 
 *          Mission Resume and Interruption:
 *          - Mission can be interrupted and resumed based on MIS_RESTART parameter
 *          - Mission change detection allows safe mission updates during flight
 *          - Auto RTL mode allows programmatic return-to-launch within Auto mode
 * 
 *          Terrain Following:
 *          - Waypoints can be specified as terrain-relative altitudes
 *          - Uses terrain database or rangefinder for terrain altitude
 *          - Automatic failsafe if terrain data unavailable
 * 
 *          Safety Features:
 *          - Pre-arm checks prevent arming with invalid missions
 *          - Takeoff command required when armed on ground
 *          - Mission change detection prevents unexpected behavior
 *          - Terrain failsafe handling for altitude references
 *          - Precision landing support with IR-LOCK or similar sensors
 * 
 * @note Auto mode requires valid position estimate (GPS or external nav)
 * @warning Mission commands execute autonomously - validate missions thoroughly before flight
 * 
 * @see AP_Mission for mission command storage and sequencing
 * @see AC_WPNav for waypoint navigation algorithms
 * @see mode.h for base Mode class and common mode functionality
 */

/**
 * @brief Initialize Auto flight mode
 * 
 * @details Initializes the Auto mode flight controller when switching from another mode.
 *          Performs safety checks to ensure mission is ready for execution and prevents
 *          unsafe mode transitions (e.g. armed on ground without takeoff command).
 * 
 *          Initialization sequence:
 *          1. Verify mission is present (unless ignore_checks=true)
 *          2. Check for takeoff command if armed and landed
 *          3. Initialize waypoint and spline navigation controllers
 *          4. Reset ROI (Region of Interest) to prevent carryover
 *          5. Clear speed overrides from previous mode
 *          6. Set flag to start/resume mission on next run() call
 *          7. Initialize mission change detector
 *          8. Clear guided mode limits
 *          9. Reset landing repositioning flag
 *          10. Initialize precision landing state machine (if enabled)
 * 
 *          Safety checks prevent mode change if:
 *          - No mission is loaded (and ignore_checks=false)
 *          - Armed and landed without NAV_TAKEOFF as first command (prevents flip-over)
 * 
 *          The mode starts in LOITER submode and waits for EKF origin before
 *          starting mission execution in run().
 * 
 * @param[in] ignore_checks If true, bypasses mission presence check (used for failsafe RTL)
 * 
 * @return true if initialization successful and mode change allowed
 * @return false if initialization failed (no mission or unsafe conditions)
 * 
 * @note Rejects mode change if armed on ground without takeoff command to prevent vehicle flip
 * @note Mission execution doesn't start until EKF origin is available (checked in run())
 * @warning Mode initialization clears pilot speed overrides - they must be re-applied if needed
 * 
 * @see run() for mission start/resume once origin is available
 * @see mission.starts_with_takeoff_cmd() for takeoff command detection
 * @see mission.start_or_resume() for mission sequencing control
 */
bool ModeAuto::init(bool ignore_checks)
{
    auto_RTL = false;
    if (mission.present() || ignore_checks) {
        // reject switching to auto mode if landed with motors armed but first command is not a takeoff (reduce chance of flips)
        if (motors->armed() && copter.ap.land_complete && !mission.starts_with_takeoff_cmd()) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Auto: Missing Takeoff Cmd");
            return false;
        }

        _mode = SubMode::LOITER;

        // stop ROI from carrying over from previous runs of the mission
        // To-Do: reset the yaw as part of auto_wp_start when the previous command was not a wp command to remove the need for this special ROI check
        if (auto_yaw.mode() == AutoYaw::Mode::ROI) {
            auto_yaw.set_mode(AutoYaw::Mode::HOLD);
        }

        // initialise waypoint and spline controller
        wp_nav->wp_and_spline_init_cm();

        // initialise desired speed overrides
        desired_speed_override = {0, 0, 0};

        // set flag to start mission
        waiting_to_start = true;

        // initialise mission change check (ignore results)
        IGNORE_RETURN(mis_change_detector.check_for_mission_change());

        // clear guided limits
        copter.mode_guided.limit_clear();

        // reset flag indicating if pilot has applied roll or pitch inputs during landing
        copter.ap.land_repo_active = false;

#if AC_PRECLAND_ENABLED
        // initialise precland state machine
        copter.precland_statemachine.init();
#endif

        return true;
    } else {
        return false;
    }
}

/**
 * @brief Clean up and reset state when exiting Auto flight mode
 * 
 * @details This function performs necessary cleanup operations when the vehicle transitions
 *          out of Auto mode into another flight mode (Loiter, Land, RTL, Manual, etc.).
 *          Ensures mission execution stops cleanly and peripheral devices reset to safe
 *          default configurations.
 * 
 *          Exit Cleanup Operations:
 *          
 *          1. Mission Execution Halt:
 *             - Checks if mission currently running (MISSION_RUNNING state)
 *             - Calls mission.stop() to pause mission command execution
 *             - Mission index preserved (can resume later if returning to Auto)
 *             - Prevents mission commands executing in non-Auto modes
 *             - Stops verify functions and command progression
 * 
 *          2. Camera Mount Reset (if enabled):
 *             - Resets gimbal/mount to default control mode
 *             - Typically returns mount to RC control or neutral position
 *             - Clears any ROI (Region of Interest) targeting from mission
 *             - Ensures mount doesn't remain locked to mission waypoint
 *             - Only compiled if HAL_MOUNT_ENABLED (hardware-dependent)
 * 
 *          3. Auto RTL Flag Clear:
 *             - Resets auto_RTL pseudo-mode flag to false
 *             - Auto RTL is special Auto mode variant that emulates RTL behavior
 *             - Ensures clean state if re-entering Auto mode later
 *             - Prevents Auto RTL behavior persisting to next Auto entry
 * 
 *          Mission Stop Behavior:
 *          - Mission execution paused at current command index
 *          - Mission NOT reset to beginning (index preserved)
 *          - Allows resume from same point if returning to Auto
 *          - MIS_RESTART parameter controls behavior on next Auto entry
 *          - Mission state changes from MISSION_RUNNING to MISSION_STOPPED
 * 
 *          Mode Transition Scenarios:
 *          
 *          Scenario A: Auto → Loiter (pilot takeover)
 *          - Mission stops immediately
 *          - Vehicle holds current position in Loiter mode
 *          - Mount returns to pilot control
 *          - Mission can resume later from same command
 *          
 *          Scenario B: Auto → Land (emergency or mission end)
 *          - Mission stops
 *          - Landing mode takes control
 *          - Mount reset to default
 *          - No mission resume expected
 *          
 *          Scenario C: Auto → RTL (failsafe or pilot command)
 *          - Mission stops
 *          - RTL mode navigates home independently
 *          - Mount reset
 *          - Mission may resume if RTL transitions back to Auto
 *          
 *          Scenario D: Auto → Manual (stabilize, alt hold, etc.)
 *          - Mission stops immediately
 *          - Pilot assumes manual control
 *          - All autonomous navigation halts
 *          - Mount returns to pilot control
 * 
 *          Why Mission Stop is Necessary:
 *          - Mission commands (NAV_WAYPOINT, DO_SET_ROI, etc.) only valid in Auto
 *          - Prevents navigation commands executing in manual modes
 *          - Stops verify functions checking for mission command completion
 *          - Prevents unexpected behavior from mission command side effects
 *          - Clean separation between autonomous and manual control
 * 
 *          Mount/Gimbal Reset Rationale:
 *          - Mission may have set ROI pointing mount at specific location
 *          - Other modes expect mount under pilot or default control
 *          - Prevents mount staying locked to waypoint after mode change
 *          - Avoids confusion for pilot (camera pointing wrong direction)
 *          - set_mode_to_default() typically means RC control or rate control
 * 
 *          Auto RTL Flag Management:
 *          - Auto RTL is pseudo-mode where Auto executes RTL-like behavior
 *          - Flag tracks when Auto is emulating RTL via DO_LAND_START or DO_RETURN_PATH_START
 *          - Must clear on exit to prevent state pollution
 *          - Separate from actual RTL mode (different mode number)
 * 
 *          State Preservation:
 *          - Mission index preserved (not reset to 0)
 *          - Mission waypoint list unchanged
 *          - Parameters unchanged
 *          - Vehicle position/velocity/attitude unchanged
 *          - Only mission execution state and peripheral configs reset
 * 
 *          Call Sequence on Mode Change:
 *          1. New mode's init() called first
 *          2. Old mode's exit() called second
 *          3. Ensures smooth transition (new mode active before cleanup)
 *          4. Prevents gap where no mode controlling vehicle
 * 
 *          Safety Considerations:
 *          - Clean exit prevents mission commands affecting other modes
 *          - Mount reset prevents unexpected camera movements
 *          - Mission stop is immediate (no delayed stopping)
 *          - Exit always succeeds (no failure conditions)
 * 
 * @note Called automatically by mode change logic when leaving Auto mode
 * @note Does NOT reset mission to beginning (preserves current mission index)
 * @note Does NOT disarm motors or change vehicle position
 * @note Mount reset only compiled if HAL_MOUNT_ENABLED defined
 * @note Safe to call multiple times (idempotent operations)
 * 
 * @see init() - Complementary initialization when entering Auto mode
 * @see mission.stop() - Halts mission command execution
 * @see AP_Mount::set_mode_to_default() - Resets gimbal control mode
 */
void ModeAuto::exit()
{
    if (copter.mode_auto.mission.state() == AP_Mission::MISSION_RUNNING) {
        copter.mode_auto.mission.stop();
    }
#if HAL_MOUNT_ENABLED
    copter.camera_mount.set_mode_to_default();
#endif  // HAL_MOUNT_ENABLED

    auto_RTL = false;
}

/**
 * @brief Main Auto mode controller - executes mission commands
 * 
 * @details Called at 100Hz or higher from the main loop. Manages mission sequencing through
 *          the AP_Mission library and dispatches to appropriate sub-controllers based on
 *          current navigation command type.
 * 
 *          Mission Start/Resume Logic:
 *          - Waits for EKF origin before starting mission (prevents navigation without reference)
 *          - Calls mission.start_or_resume() which respects MIS_RESTART parameter
 *          - MIS_RESTART=0: Resume from last command
 *          - MIS_RESTART=1: Restart from beginning
 * 
 *          Mission Change Detection:
 *          - Monitors mission for changes during flight using mis_change_detector
 *          - If waypoint command is running and mission changes, restarts current command
 *          - Provides GCS notification of mission change and restart status
 *          - Prevents unexpected behavior from mission updates during flight
 * 
 *          Mission Update:
 *          - Calls mission.update() to advance through mission commands
 *          - Mission library handles start_command() and verify_command() callbacks
 *          - Automatically sequences through NAV, CONDITION, and DO commands
 * 
 *          Submode Dispatch:
 *          Dispatches to specialized controllers based on current navigation command:
 *          - TAKEOFF: Vertical climb to altitude with auto_takeoff controller
 *          - WP / CIRCLE_MOVE_TO_EDGE: Waypoint navigation with wp_nav
 *          - LAND: Autonomous landing with optional precision landing
 *          - RTL: Return-to-launch sequence (delegates to RTL mode)
 *          - CIRCLE: Circular loiter pattern at constant radius
 *          - NAVGUIDED: External navigation control (MAVLink or companion computer)
 *          - NAV_SCRIPT_TIME: Lua script-controlled navigation
 *          - LOITER: Position hold at waypoint
 *          - LOITER_TO_ALT: Loiter while climbing/descending
 *          - NAV_PAYLOAD_PLACE: Autonomous payload placement
 *          - NAV_ATTITUDE_TIME: Fixed attitude flight
 * 
 *          Auto RTL Mode:
 *          - Tracks auto_RTL flag for programmatic return-to-launch within Auto
 *          - Auto RTL considered active while in landing sequence or return path
 *          - Logs mode transitions for AUTO_RTL entry/exit
 * 
 * @note Called at main loop rate (typically 400Hz for attitude, 100Hz+ for this function)
 * @note Mission execution blocked until AHRS origin is set and available
 * @warning Mission changes during flight restart current waypoint - may cause unexpected navigation
 * 
 * @see mission.update() for mission sequencing and command advancement
 * @see start_command() for mission command initialization
 * @see verify_command() for mission command completion checking
 * @see wp_run(), takeoff_run(), land_run(), etc. for submode controllers
 */
void ModeAuto::run()
{
    // start or update mission
    if (waiting_to_start) {
        // don't start the mission until we have an origin
        Location loc;
        if (copter.ahrs.get_origin(loc)) {
            // start/resume the mission (based on MIS_RESTART parameter)
            mission.start_or_resume();
            waiting_to_start = false;

            // initialise mission change check (ignore results)
            IGNORE_RETURN(mis_change_detector.check_for_mission_change());
        }
    } else {
        // check for mission changes
        if (mis_change_detector.check_for_mission_change()) {
            // if mission is running restart the current command if it is a waypoint or spline command
            if ((mission.state() == AP_Mission::MISSION_RUNNING) && (_mode == SubMode::WP)) {
                if (mission.restart_current_nav_cmd()) {
                    gcs().send_text(MAV_SEVERITY_CRITICAL, "Auto mission changed, restarted command");
                } else {
                    // failed to restart mission for some reason
                    gcs().send_text(MAV_SEVERITY_CRITICAL, "Auto mission changed but failed to restart command");
                }
            }
        }

        mission.update();
    }

    // call the correct auto controller
    switch (_mode) {

    case SubMode::TAKEOFF:
        takeoff_run();
        break;

    case SubMode::WP:
    case SubMode::CIRCLE_MOVE_TO_EDGE:
        wp_run();
        break;

    case SubMode::LAND:
        land_run();
        break;

    case SubMode::RTL:
        rtl_run();
        break;

    case SubMode::CIRCLE:
        circle_run();
        break;

    case SubMode::NAVGUIDED:
    case SubMode::NAV_SCRIPT_TIME:
#if AC_NAV_GUIDED || AP_SCRIPTING_ENABLED
        nav_guided_run();
#endif
        break;

    case SubMode::LOITER:
        loiter_run();
        break;

    case SubMode::LOITER_TO_ALT:
        loiter_to_alt_run();
        break;

#if AP_MISSION_NAV_PAYLOAD_PLACE_ENABLED && AC_PAYLOAD_PLACE_ENABLED
    case SubMode::NAV_PAYLOAD_PLACE:
        payload_place.run();
        break;
#endif

    case SubMode::NAV_ATTITUDE_TIME:
        nav_attitude_time_run();
        break;
    }

    // only pretend to be in auto RTL so long as mission still thinks its in a landing sequence or the mission has completed
    const bool auto_rtl_active = mission.get_in_landing_sequence_flag() || mission.get_in_return_path_flag() || mission.state() == AP_Mission::mission_state::MISSION_COMPLETE;
    if (auto_RTL && !auto_rtl_active) {
        auto_RTL = false;
        // log exit from Auto RTL
#if HAL_LOGGING_ENABLED
        copter.logger.Write_Mode((uint8_t)copter.flightmode->mode_number(), ModeReason::AUTO_RTL_EXIT);
#endif
    }
}

/**
 * @brief Determine if GPS/position estimate required for current Auto submode
 * 
 * @details This function indicates whether the current Auto submode requires a valid position
 *          estimate from GPS, visual odometry, or other positioning system. Used by the EKF
 *          failsafe system to determine if position loss should trigger a failsafe action.
 * 
 *          Position Requirement by Submode:
 *          
 *          Submodes REQUIRING Position Estimate:
 *          - SubMode::WP - Waypoint navigation needs position for path following
 *          - SubMode::CIRCLE - Circle navigation needs center point and radius
 *          - SubMode::LAND - Landing needs position hold and descent tracking
 *          - SubMode::RTL - Return to Launch needs position for navigation home
 *          - SubMode::LOITER - Loitering needs position hold
 *          - SubMode::LOITER_TO_ALT - Position hold while climbing/descending
 *          - SubMode::TAKEOFF - Takeoff needs position hold during climb
 *          - SubMode::NAVGUIDED - Guided navigation requires position control
 *          - SubMode::NAV_SCRIPT_TIME - Scripting control usually needs position
 *          - SubMode::NAV_PAYLOAD_PLACE - Payload placement needs precise position
 *          - SubMode::CIRCLE_MOVE_TO_EDGE - Moving to circle edge needs position
 * 
 *          Submode NOT REQUIRING Position Estimate:
 *          - SubMode::NAV_ATTITUDE_TIME - Pure attitude/throttle control mode
 *            * Commands specific roll, pitch, yaw angles
 *            * Commands vertical climb rate
 *            * No horizontal position control active
 *            * Can fly without GPS (similar to Stabilize mode with altitude hold)
 *            * Used for aerobatics or GPS-denied flight segments
 * 
 *          EKF Failsafe Integration:
 *          - EKF failsafe checks this function when position estimate degrades
 *          - If requires_GPS() returns true and position lost: trigger failsafe
 *          - If requires_GPS() returns false: allow continued operation without position
 *          - Prevents unnecessary failsafe during GPS-independent submodes
 * 
 *          Position Estimate Sources:
 *          - GPS (primary source for outdoor flight)
 *          - Visual odometry (T265 camera, external vision system)
 *          - Optical flow + rangefinder (indoor hover)
 *          - Wheel encoders (ground vehicles)
 *          - External positioning systems (motion capture, UWB beacons)
 * 
 *          Failsafe Behavior Examples:
 *          
 *          Example A: GPS Loss During Waypoint Navigation
 *          - _mode = SubMode::WP (waypoint navigation)
 *          - requires_GPS() returns true
 *          - GPS signal lost (EKF position estimate fails)
 *          - EKF failsafe triggered → vehicle switches to Land mode
 *          - Prevents flyaway due to no position control
 *          
 *          Example B: GPS Loss During NAV_ATTITUDE_TIME
 *          - _mode = SubMode::NAV_ATTITUDE_TIME (attitude control)
 *          - requires_GPS() returns false
 *          - GPS signal lost (EKF position estimate fails)
 *          - EKF failsafe NOT triggered (position not required)
 *          - Vehicle continues executing attitude commands
 *          - Pilot or mission can recover when GPS returns
 * 
 *          Mission Command Usage:
 *          - NAV_ATTITUDE_TIME mission command enables GPS-independent flight
 *          - Useful for aerobatic maneuvers where position drift acceptable
 *          - Useful for flying through GPS-denied areas (tunnels, under bridges)
 *          - Must carefully plan entry/exit to ensure GPS available before/after
 * 
 *          set_submode() Integration:
 *          - When submode changes via set_submode(), may re-trigger EKF failsafe check
 *          - Changing from NAV_ATTITUDE_TIME to WP re-evaluates position requirement
 *          - If position not available when entering position-required submode: failsafe
 * 
 *          Safety Considerations:
 *          - NAV_ATTITUDE_TIME is only GPS-independent submode (use carefully)
 *          - All navigation submodes require position for safe operation
 *          - Position loss during navigation → immediate failsafe to Land
 *          - Prevents uncontrolled drift or flyaway
 * 
 *          Const Correctness:
 *          - Function is const (does not modify mode state)
 *          - Can be safely called from failsafe checking code
 *          - No side effects or state changes
 * 
 * @return true if current submode requires GPS/position estimate for safe operation
 * @return false if current submode can operate without position estimate (NAV_ATTITUDE_TIME only)
 * 
 * @note Only NAV_ATTITUDE_TIME submode returns false (all others require position)
 * @note Used by EKF failsafe to determine if position loss should trigger failsafe
 * @note Returning false does NOT disable failsafe entirely (barometer still required)
 * @note Position requirement checked continuously during flight
 * 
 * @warning Flying without position estimate is dangerous - use NAV_ATTITUDE_TIME carefully
 * @warning Vehicle will drift with wind during GPS-independent flight
 * 
 * @see set_submode() - Changes submode and may re-trigger EKF failsafe check
 * @see copter.failsafe_ekf_recheck() - Re-evaluates failsafe when requirements change
 */
bool ModeAuto::requires_GPS() const
{
    // position estimate is required in all sub modes except attitude control
    return _mode != SubMode::NAV_ATTITUDE_TIME;
}

/**
 * @brief Change Auto mode submode with EKF failsafe re-evaluation
 * 
 * @details This function transitions Auto mode between different submodes (waypoint navigation,
 *          circle, land, takeoff, etc.) and critically handles EKF failsafe re-triggering when
 *          transitioning from GPS-independent to GPS-dependent submodes.
 * 
 *          Submode Transition Process:
 *          
 *          1. Check for No-Op:
 *             - If new_submode == current _mode: return immediately
 *             - Prevents unnecessary state changes and failsafe checks
 *             - Idempotent (safe to call with same submode multiple times)
 * 
 *          2. Backup Old Submode:
 *             - Store current _mode in old_submode
 *             - Needed to determine if EKF failsafe recheck required
 *             - Specifically checking if leaving NAV_ATTITUDE_TIME
 * 
 *          3. Update Submode:
 *             - Set _mode = new_submode
 *             - Submode immediately active (no delayed activation)
 *             - Subsequent run() calls execute new submode's logic
 * 
 *          4. EKF Failsafe Re-Evaluation (Critical):
 *             - If OLD submode was NAV_ATTITUDE_TIME: recheck EKF failsafe
 *             - NAV_ATTITUDE_TIME is only submode not requiring position
 *             - Transitioning away means position now required
 *             - If position estimate invalid: failsafe triggers immediately
 * 
 *          Why EKF Failsafe Recheck Needed:
 *          
 *          Scenario: GPS Lost During NAV_ATTITUDE_TIME
 *          - Vehicle executing NAV_ATTITUDE_TIME command (no position required)
 *          - GPS signal lost or EKF position estimate fails
 *          - EKF failsafe NOT triggered (position not required in this submode)
 *          - Mission progresses to next command: NAV_WAYPOINT
 *          - set_submode(SubMode::WP) called
 *          - Waypoint navigation REQUIRES position estimate
 *          - Must recheck EKF failsafe status immediately
 *          - If position still unavailable: trigger failsafe → Land mode
 * 
 *          Without Failsafe Recheck:
 *          - Vehicle could enter waypoint navigation without valid position
 *          - Would attempt position control with no position estimate
 *          - Likely flyaway or uncontrolled behavior
 *          - Safety hazard (vehicle drifting without position hold)
 * 
 *          With Failsafe Recheck:
 *          - Position requirement detected immediately on submode change
 *          - If position invalid: mode change to Land triggered
 *          - Vehicle safely descends and lands
 *          - Prevents attempt to navigate without position
 * 
 *          Failsafe Recheck Direction:
 *          - Only checks when LEAVING NAV_ATTITUDE_TIME (old_submode check)
 *          - Not needed when ENTERING NAV_ATTITUDE_TIME (reducing requirements)
 *          - Not needed between other submodes (all require position)
 *          - Specifically handles GPS-independent → GPS-dependent transition
 * 
 *          copter.failsafe_ekf_recheck() Behavior:
 *          - Re-evaluates current EKF health vs current mode requirements
 *          - Calls requires_GPS() on current mode to get requirements
 *          - If position required but unavailable: triggers EKF failsafe
 *          - May cause immediate mode change (Auto → Land)
 *          - Logs failsafe trigger event
 *          - Notifies ground station of failsafe activation
 * 
 *          Potential Mode Change:
 *          - failsafe_ekf_recheck() can change flight mode
 *          - set_submode() caller must handle mode being different after call
 *          - After failsafe: copter.flightmode != &copter.mode_auto
 *          - Mission execution stops (verify_command checks mode)
 * 
 *          Submode Transitions in Normal Operation:
 *          - TAKEOFF → WP (after takeoff complete)
 *          - WP → WP (waypoint to waypoint)
 *          - WP → CIRCLE (entering loiter turns command)
 *          - CIRCLE_MOVE_TO_EDGE → CIRCLE (reached circle edge)
 *          - WP → LAND (beginning landing sequence)
 *          - WP → RTL (executing NAV_RETURN_TO_LAUNCH)
 *          - WP → LOITER (loiter unlimited command)
 *          - WP → NAV_ATTITUDE_TIME (entering attitude control)
 *          - NAV_ATTITUDE_TIME → WP (exiting attitude control - FAILSAFE CHECK)
 * 
 *          Call Sites:
 *          - takeoff_start() → set_submode(SubMode::TAKEOFF)
 *          - wp_start() → set_submode(SubMode::WP)
 *          - land_start() → set_submode(SubMode::LAND)
 *          - rtl_start() → set_submode(SubMode::RTL)
 *          - circle_start() → set_submode(SubMode::CIRCLE)
 *          - circle_movetoedge_start() → set_submode(SubMode::CIRCLE_MOVE_TO_EDGE)
 *          - nav_guided_start() → set_submode(SubMode::NAVGUIDED)
 *          - loiter_start() → set_submode(SubMode::LOITER)
 *          - do_loiter_to_alt() → set_submode(SubMode::LOITER_TO_ALT)
 *          - do_nav_script_time() → set_submode(SubMode::NAV_SCRIPT_TIME)
 *          - do_nav_attitude_time() → set_submode(SubMode::NAV_ATTITUDE_TIME)
 *          - do_payload_place() → set_submode(SubMode::NAV_PAYLOAD_PLACE)
 * 
 *          Thread Safety:
 *          - Called from main thread only (400Hz scheduler task)
 *          - No mutex needed (_mode not shared across threads)
 *          - Failsafe recheck synchronous (completes before return)
 * 
 * @param[in] new_submode The Auto submode to transition to (WP, CIRCLE, LAND, etc.)
 * 
 * @note No-op if new_submode equals current submode (safe to call redundantly)
 * @note May trigger EKF failsafe if transitioning from NAV_ATTITUDE_TIME and position invalid
 * @note May cause immediate mode change to Land if failsafe triggered
 * @note Caller should check copter.flightmode after call (mode may have changed)
 * 
 * @warning Flight mode may change during this function due to failsafe
 * @warning Position loss transitioning from NAV_ATTITUDE_TIME → other submodes triggers Land
 * 
 * @see requires_GPS() - Determines if current submode requires position estimate
 * @see copter.failsafe_ekf_recheck() - Re-evaluates EKF failsafe based on current requirements
 */
void ModeAuto::set_submode(SubMode new_submode)
{
    // return immediately if the submode has not been changed
    if (new_submode == _mode) {
        return;
    }

    // backup old mode
    SubMode old_submode = _mode;

    // set mode
    _mode = new_submode;

    // if changing out of the nav-attitude-time submode, recheck the EKF failsafe
    // this may trigger a flight mode change if the EKF failsafe is active
    if (old_submode == SubMode::NAV_ATTITUDE_TIME) {
        copter.failsafe_ekf_recheck();
    }
}

/**
 * @brief Check if specific Auto mode option flag is enabled
 * 
 * @details This function tests whether a particular Auto mode configuration option is enabled
 *          by checking the AUTO_OPTIONS bitmask parameter. AUTO_OPTIONS controls various
 *          optional behaviors in Auto mode through individual bit flags.
 * 
 *          Implementation:
 *          - Retrieves AUTO_OPTIONS parameter value from copter.g2.auto_options
 *          - Casts option enum to uint32_t to get bit position
 *          - Performs bitwise AND with parameter value
 *          - Returns true if specified bit is set (option enabled)
 * 
 *          Available Auto Mode Options (enum Option):
 *          
 *          Option::AllowArming (bit 0):
 *          - Allow arming in Auto mode
 *          - Normally cannot arm in Auto (safety feature)
 *          - Enable for auto-takeoff applications
 *          - Used by allows_arming() method
 * 
 *          Option::AllowTakeOffWithoutRaisingThrottle (bit 1):
 *          - Allow takeoff without pilot raising throttle
 *          - Normally require throttle input to arm and takeoff
 *          - Enable for fully autonomous takeoff
 *          - Bypasses disarm check during takeoff
 *          - Used by takeoff_run() method
 * 
 *          Option::IgnorePilotYaw (bit 2):
 *          - Ignore pilot yaw stick input during Auto
 *          - Normally pilot can override yaw in Auto
 *          - Enable for strict autonomous yaw control
 *          - Used by use_pilot_yaw() method
 *          - Applies to waypoints and loiter
 * 
 *          Option::AllowWeatherVaning (bit 3):
 *          - Allow weathervaning (nose into wind) in Auto
 *          - Weathervaning yaws aircraft to minimize drift
 *          - Useful for fixed-wing or hybrid aircraft
 *          - Used by allows_weathervaning() method
 *          - Only compiled if WEATHERVANE_ENABLED
 * 
 *          Parameter Storage:
 *          - Parameter: AUTO_OPTIONS (g2.auto_options)
 *          - Type: Bitmask (uint32_t)
 *          - Location: Parameters g2 group (newer parameters)
 *          - Persistent: Stored in EEPROM/parameter storage
 *          - Configurable: Via ground station parameter editor
 * 
 *          Bitwise Operation Example:
 *          - AUTO_OPTIONS = 0b00000101 (bits 0 and 2 set)
 *          - option = Option::AllowArming (value 0, bit 0)
 *          - (uint32_t)option = 0x00000001
 *          - auto_options & option = 0b00000101 & 0b00000001 = 0b00000001 (non-zero)
 *          - Returns true (bit 0 is set)
 * 
 *          Usage Pattern:
 *          ```cpp
 *          if (option_is_enabled(Option::AllowArming)) {
 *              // Allow arming in Auto mode
 *          }
 *          
 *          if (option_is_enabled(Option::IgnorePilotYaw)) {
 *              // Don't check pilot yaw input
 *          }
 *          ```
 * 
 *          Call Sites:
 *          - allows_arming() - Check if arming permitted in Auto
 *          - allows_weathervaning() - Check if weathervaning allowed
 *          - use_pilot_yaw() - Check if pilot yaw input should be used
 *          - takeoff_run() - Check if throttle raise required for takeoff
 * 
 *          Const Correctness:
 *          - Function is const (does not modify state)
 *          - Can be called from const methods
 *          - Safe to call during mode evaluation
 * 
 *          Ground Station Configuration:
 *          - Parameter visible in ground station parameter list
 *          - Bitmask shown as decimal or checkbox list
 *          - Changes take effect immediately (no reboot required)
 *          - Can be changed during flight (use caution)
 * 
 *          Safety Implications:
 *          - AllowArming: Enables arming in Auto (use for auto-takeoff only)
 *          - AllowTakeOffWithoutRaisingThrottle: Bypasses safety check
 *          - IgnorePilotYaw: Pilot cannot override yaw (reduces manual control)
 *          - Options should be enabled only when behavior is desired
 * 
 * @param[in] option The Auto mode option to check (AllowArming, IgnorePilotYaw, etc.)
 * 
 * @return true if the specified option bit is set in AUTO_OPTIONS parameter
 * @return false if the specified option bit is not set
 * 
 * @note Parameter AUTO_OPTIONS is bitmask (multiple options can be enabled simultaneously)
 * @note Changes to AUTO_OPTIONS parameter take effect immediately
 * @note Function is const (safe to call from any context)
 * 
 * @see allows_arming() - Uses AllowArming option
 * @see allows_weathervaning() - Uses AllowWeatherVaning option
 * @see use_pilot_yaw() - Uses IgnorePilotYaw option
 * @see takeoff_run() - Uses AllowTakeOffWithoutRaisingThrottle option
 */
bool ModeAuto::option_is_enabled(Option option) const
{
    return ((copter.g2.auto_options & (uint32_t)option) != 0);
}

/**
 * @brief Determine if arming is permitted in Auto mode based on configuration and state
 * 
 * @details This function evaluates whether the vehicle can be armed while in Auto flight mode.
 *          By default, arming in Auto is prohibited for safety (prevents accidental autonomous
 *          flight). However, specific configurations allow arming for auto-takeoff scenarios.
 * 
 *          Arming Permission Logic:
 *          
 *          1. Auto RTL Check (Highest Priority):
 *             - If auto_RTL flag is true: arming PROHIBITED
 *             - Auto RTL is pseudo-mode emulating RTL behavior within Auto
 *             - Should never allow arming during RTL-like behavior
 *             - Returns false immediately (no further checks)
 * 
 *          2. Auto Options Check:
 *             - If AUTO_OPTIONS includes AllowArming bit: arming PERMITTED
 *             - Controlled by AUTO_OPTIONS parameter (bitmask)
 *             - Must be explicitly enabled by user
 *             - Default is disabled (arming not allowed)
 * 
 *          Why Arming Usually Prohibited in Auto:
 *          - Auto mode is fully autonomous (executes mission commands)
 *          - Arming in Auto could immediately start mission execution
 *          - Risk of unexpected takeoff or motor start
 *          - Safer to arm in manual mode (Stabilize, Loiter) then switch to Auto
 *          - Prevents pilot confusion about vehicle state
 * 
 *          When to Enable AllowArming Option:
 *          - Automated flight systems (no pilot present)
 *          - Auto-takeoff sequences from ground
 *          - Fully scripted mission execution
 *          - Competition or demonstration flights
 *          - Systems with external mission start trigger
 * 
 *          Auto RTL Prohibition Rationale:
 *          - Auto RTL means vehicle is returning home autonomously
 *          - Arming during return would be nonsensical (already in flight)
 *          - May indicate Auto RTL entered from failsafe
 *          - Should never re-arm during autonomous return
 *          - Safety check prevents configuration errors
 * 
 *          Typical Arming Sequence (AllowArming Disabled):
 *          1. Pilot arms in Stabilize or Loiter mode
 *          2. Pilot verifies vehicle stable and responsive
 *          3. Pilot switches to Auto mode
 *          4. Mission execution begins
 *          5. Provides opportunity to abort before autonomous flight
 * 
 *          Typical Arming Sequence (AllowArming Enabled):
 *          1. Vehicle on ground in Auto mode
 *          2. Mission loaded with NAV_TAKEOFF as first command
 *          3. External system or pilot sends arm command
 *          4. Vehicle arms directly in Auto mode
 *          5. Pilot raises throttle (or auto-takeoff begins)
 *          6. Takeoff executes autonomously
 * 
 *          Interaction with Mission Commands:
 *          - Mission execution does NOT begin until armed
 *          - First mission command should be NAV_TAKEOFF if arming in Auto
 *          - init() checks for takeoff command when armed on ground
 *          - Prevents flip risk (armed but not taking off)
 * 
 *          Arming Method Parameter:
 *          - method parameter indicates how arming requested (RC, GCS, scripting)
 *          - Currently not used in decision logic (all methods treated equally)
 *          - Could enable method-specific arming restrictions
 *          - Passed to maintain API consistency with other modes
 * 
 *          Call Sequence:
 *          - AP_Arming checks mode's allows_arming() before permitting arm
 *          - If returns false: arming request denied
 *          - If returns true: arming proceeds (pre-arm checks still apply)
 *          - Logged and reported to ground station
 * 
 *          Safety Checks Still Apply:
 *          - Even if allows_arming() returns true, standard pre-arm checks run
 *          - GPS lock, sensor health, calibration, etc. still required
 *          - This function only controls mode-specific arming permission
 *          - Does not bypass critical safety checks
 * 
 *          Ground Station Behavior:
 *          - If arming denied: GCS shows "Arming not allowed in current mode"
 *          - Pilot must switch modes or enable AUTO_OPTIONS AllowArming
 *          - Some GCS automatically switch to Stabilize for arming
 * 
 * @param[in] method Arming method (RC, GCS, scripting, etc.) - currently unused
 * 
 * @return false if in Auto RTL pseudo-mode (always prohibited)
 * @return true if AUTO_OPTIONS AllowArming bit enabled (arming permitted)
 * @return false if AUTO_OPTIONS AllowArming bit disabled (default behavior)
 * 
 * @note Default behavior: arming prohibited in Auto mode for safety
 * @note AUTO_OPTIONS parameter bit 0 (AllowArming) must be set to allow arming
 * @note Always returns false during Auto RTL (safety requirement)
 * @note Pre-arm checks still apply even if this returns true
 * 
 * @warning Enabling arming in Auto mode requires careful mission planning
 * @warning First mission command should be NAV_TAKEOFF when arming in Auto
 * @warning Vehicle may start autonomous flight immediately upon arming
 * 
 * @see option_is_enabled() - Checks AUTO_OPTIONS parameter bits
 * @see init() - Checks for valid takeoff command when armed in Auto
 * @see AP_Arming::arm() - Main arming logic that calls this function
 */
bool ModeAuto::allows_arming(AP_Arming::Method method) const
{
    if (auto_RTL) {
        return false;
    }
    return option_is_enabled(Option::AllowArming);
}

/**
 * @brief Check if weathervaning (nose-into-wind yaw control) is permitted in Auto mode
 * 
 * @details Weathervaning is an advanced flight control feature that automatically yaws the
 *          vehicle to point into the wind during position-hold and waypoint navigation. This
 *          minimizes drift caused by crosswinds and improves position accuracy. In Auto mode,
 *          weathervaning must be explicitly enabled through the AUTO_OPTIONS parameter.
 * 
 *          Weathervaning Concept:
 *          - Automatically rotate vehicle to face into wind direction
 *          - Reduces horizontal drift caused by crosswinds
 *          - Minimizes lateral surface area exposed to wind
 *          - Similar to weather vane rotating on building rooftop
 *          - Most effective for aircraft with asymmetric drag (fixed-wing, hybrid VTOL)
 * 
 *          Why Weathervaning Control Needed:
 *          - For multirotors: usually not beneficial (symmetric drag)
 *          - For quadplanes/VTOL: very beneficial (fuselage creates drag)
 *          - For tailsitters: essential for position control
 *          - Can conflict with yaw commands in mission
 *          - May interfere with camera/sensor pointing
 *          - Should be disabled when specific yaw orientation required
 * 
 *          Enabling Weathervaning in Auto:
 *          - Set AUTO_OPTIONS bit 3 (AllowWeatherVaning)
 *          - Weathervane controller must also be enabled globally
 *          - Weathervane gains must be configured (WVANE_GAIN, etc.)
 *          - Wind estimation must be available (EKF wind state)
 *          - Both conditions required for weathervaning to activate
 * 
 *          When Weathervaning Applied:
 *          - During waypoint navigation (SubMode::WP)
 *          - During loiter operations
 *          - During position hold
 *          - NOT during landing (uses landing yaw control)
 *          - NOT during takeoff (uses fixed yaw)
 *          - NOT when ROI (Region of Interest) active
 * 
 *          Interaction with Mission Yaw Commands:
 *          - Weathervaning conflicts with DO_SET_ROI commands
 *          - Weathervaning conflicts with CONDITION_YAW commands
 *          - Weathervaning controller respects yaw command priority
 *          - If yaw explicitly commanded: weathervane suspended
 *          - When yaw command completes: weathervane can resume
 * 
 *          Compilation Conditional:
 *          - Only compiled if WEATHERVANE_ENABLED defined
 *          - WEATHERVANE_ENABLED controlled by build configuration
 *          - Disabled on memory-constrained boards
 *          - Reduces code size when feature not needed
 *          - Check HAL_WEATHERVANE_ENABLED in hwdef files
 * 
 *          Integration with Attitude Controller:
 *          - Weathervane controller calculates desired yaw rate
 *          - Yaw rate blended with mission yaw commands
 *          - Uses PID controller for smooth yaw control
 *          - Respects vehicle yaw rate limits
 *          - Coordinated with position controller
 * 
 *          Wind Estimation Requirements:
 *          - EKF must provide wind velocity estimate
 *          - Requires GPS velocity and airspeed sensor OR
 *          - Requires GPS velocity and sufficient flight time for learning
 *          - Wind estimate quality affects weathervane performance
 *          - Poor wind estimate may cause oscillation
 * 
 *          Tuning Parameters:
 *          - WVANE_GAIN: Main weathervane controller gain
 *          - WVANE_MINROLL: Minimum roll angle before engaging
 *          - WVANE_PCT_MAX: Maximum yaw rate authority
 *          - WVANE_OPTIONS: Additional weathervane options
 *          - Parameters documented in AP_WeatherVane library
 * 
 *          Performance Considerations:
 *          - Reduces position error in crosswinds (especially hybrid VTOL)
 *          - May increase mission completion time (extra yaw movement)
 *          - Can improve stability in gusty conditions
 *          - May distract payload sensors (cameras, LIDAR)
 *          - Trade-off between accuracy and efficiency
 * 
 *          Use Cases:
 *          - Quadplane surveying in windy conditions
 *          - Tailsitter VTOL position hold
 *          - Long-range autonomous waypoint missions
 *          - Precision agriculture with crosswinds
 *          - Any mission where position accuracy > yaw control
 * 
 *          When to Disable:
 *          - Multirotor missions (little benefit)
 *          - Missions with specific yaw requirements
 *          - Camera/gimbal missions needing fixed orientation
 *          - Tight indoor navigation
 *          - Missions with frequent yaw commands
 * 
 * @return true if AUTO_OPTIONS AllowWeatherVaning bit is set (weathervaning allowed)
 * @return false if AUTO_OPTIONS AllowWeatherVaning bit is clear (default, weathervaning disabled)
 * 
 * @note Only compiled if WEATHERVANE_ENABLED feature flag set
 * @note Weathervane feature must also be globally enabled (WVANE_GAIN > 0)
 * @note Most useful for hybrid VTOL and tailsitter aircraft
 * @note May conflict with ROI and yaw mission commands
 * @note Requires EKF wind estimation to function
 * 
 * @see option_is_enabled() - Checks AUTO_OPTIONS parameter bits
 * @see AP_WeatherVane - Global weathervane controller implementation
 * @see auto_yaw - Auto mode yaw controller that integrates weathervane commands
 */
#if WEATHERVANE_ENABLED
bool ModeAuto::allows_weathervaning() const
{
    return option_is_enabled(Option::AllowWeatherVaning);
}
#endif

/**
 * @brief Jump directly to mission landing sequence, entering Auto RTL pseudo-mode
 * 
 * @details This function implements one of three Auto RTL entry methods. It searches the loaded
 *          mission for a DO_LAND_START command and jumps directly to that point, beginning the
 *          landing sequence immediately. This provides a "mission-based RTL" that follows the
 *          mission's predefined landing approach rather than the standard RTL behavior.
 * 
 *          Auto RTL Concept:
 *          - Alternative to standard RTL mode
 *          - Uses mission commands for return and landing
 *          - Leverages DO_LAND_START, DO_RETURN_PATH_START markers
 *          - Allows complex return paths (not just straight line home)
 *          - Supports precision landing, survey patterns before landing
 *          - More flexible than standard RTL fixed behavior
 * 
 *          Mission Landing Sequence:
 *          - Mission must contain DO_LAND_START command
 *          - DO_LAND_START marks beginning of landing sequence
 *          - All commands after DO_LAND_START are part of landing
 *          - Typical sequence: approach waypoint → loiter → land
 *          - May include precision landing, parachute deploy, etc.
 * 
 *          Jump Logic:
 *          1. Calculate vehicle's current stopping point
 *          2. Search mission for DO_LAND_START command
 *          3. Find closest mission leg to stopping point after DO_LAND_START
 *          4. Set mission index to that command
 *          5. Mission resumes from landing sequence start
 *          6. Enter Auto RTL pseudo-mode
 * 
 *          Stopping Point Calculation:
 *          - get_stopping_point() returns position where vehicle would stop
 *          - Accounts for current velocity and deceleration
 *          - Used to find closest mission leg to rejoin
 *          - Prevents backtracking if already near landing sequence
 *          - Minimizes flight time to landing
 * 
 *          Success Path:
 *          - mission.jump_to_landing_sequence() returns true
 *          - Calls enter_auto_rtl() to activate Auto RTL pseudo-mode
 *          - Sets auto_RTL flag (vehicle now in "Auto RTL" state)
 *          - Mission continues from landing sequence
 *          - Returns true to indicate success
 * 
 *          Failure Path:
 *          - No DO_LAND_START found in mission
 *          - Mission empty or invalid
 *          - Landing sequence unreachable
 *          - Logs AUTO_RTL error to dataflash
 *          - Triggers user notification (sad tone)
 *          - Sends GCS message "No landing sequence found"
 *          - Returns false to indicate failure
 * 
 *          When This Method Used:
 *          - Triggered by RTL_ALT_TYPE = 3 (Auto RTL via land start)
 *          - Failsafe options configured to use Auto RTL
 *          - RC failsafe → Auto RTL
 *          - Battery failsafe → Auto RTL
 *          - GCS failsafe → Auto RTL
 *          - Smart RTL failure fallback → Auto RTL
 * 
 *          Comparison to Other Auto RTL Methods:
 *          - jump_to_landing_sequence_auto_RTL(): Go directly to landing (this function)
 *          - return_path_start_auto_RTL(): Follow return path, then land
 *          - return_path_or_jump_to_landing_sequence_auto_RTL(): Try return path, fallback to landing
 * 
 *          Mission Planning Requirements:
 *          - Mission MUST contain DO_LAND_START command
 *          - DO_LAND_START typically placed before final landing waypoints
 *          - Landing sequence should be accessible from any mission point
 *          - Consider multiple approach paths for different wind conditions
 *          - Test Auto RTL behavior before flight
 * 
 *          Error Handling:
 *          - Logs error code to dataflash for post-flight analysis
 *          - Triggers AP_Notify "user_mode_change_failed" event
 *          - Sad tone alerts pilot to failure
 *          - GCS text message provides specific failure reason
 *          - Mode change aborted (vehicle stays in current mode)
 * 
 *          Mode Reason Parameter:
 *          - reason logged to explain why Auto RTL triggered
 *          - Examples: ModeReason::RC_FAILSAFE, BATTERY_FAILSAFE, GCS_FAILSAFE
 *          - Recorded in dataflash MODE message
 *          - Helpful for post-flight analysis
 *          - Passed to enter_auto_rtl() for logging
 * 
 *          Auto RTL Flag:
 *          - auto_RTL flag set by enter_auto_rtl()
 *          - Distinguishes Auto RTL from normal Auto mode
 *          - Prevents certain operations (e.g., arming)
 *          - Used for telemetry reporting
 *          - Cleared when landing complete or mission ends
 * 
 *          Use Cases:
 *          - Complex landing patterns (e.g., circling descent)
 *          - Precision landing on moving target
 *          - Survey pattern before landing
 *          - Parachute deployment sequence
 *          - Cargo drop before landing
 * 
 * @param[in] reason Why Auto RTL triggered (RC failsafe, battery, GCS, etc.) - logged
 * 
 * @return true if landing sequence found and Auto RTL mode entered successfully
 * @return false if no DO_LAND_START found in mission or jump failed
 * 
 * @note Mission must contain DO_LAND_START command for this to succeed
 * @note Jumps directly to landing (does not follow return path)
 * @note Calculates vehicle stopping point to minimize backtracking
 * @note Failure triggers error log, notification, and GCS message
 * @note On failure, vehicle remains in current flight mode
 * 
 * @warning Mission without DO_LAND_START will cause Auto RTL to fail
 * @warning Always test Auto RTL behavior before flight
 * 
 * @see return_path_start_auto_RTL() - Alternative: follow return path before landing
 * @see return_path_or_jump_to_landing_sequence_auto_RTL() - Try return path, fallback to landing
 * @see enter_auto_rtl() - Activates Auto RTL pseudo-mode
 * @see mission.jump_to_landing_sequence() - AP_Mission method to find and jump to landing
 * @see get_stopping_point() - Calculate where vehicle would stop given current state
 */
// Go straight to landing sequence via DO_LAND_START, if succeeds pretend to be Auto RTL mode
bool ModeAuto::jump_to_landing_sequence_auto_RTL(ModeReason reason)
{
    if (!mission.jump_to_landing_sequence(get_stopping_point())) {
        LOGGER_WRITE_ERROR(LogErrorSubsystem::FLIGHT_MODE, LogErrorCode(Number::AUTO_RTL));
        // make sad noise
        if (copter.ap.initialised) {
            AP_Notify::events.user_mode_change_failed = 1;
        }
        gcs().send_text(MAV_SEVERITY_WARNING, "Mode change to AUTO RTL failed: No landing sequence found");
        return false;
    }

    return enter_auto_rtl(reason);
}

/**
 * @brief Join mission return path, entering Auto RTL pseudo-mode
 * 
 * @details This function implements one of three Auto RTL entry methods. It searches the loaded
 *          mission for a return path (marked by DO_RETURN_PATH_START) and intelligently rejoins
 *          the mission at the closest point on that path. This provides a "mission-based RTL"
 *          that follows predefined return waypoints rather than flying straight home.
 * 
 *          Return Path Concept:
 *          - DO_RETURN_PATH_START command marks beginning of return path in mission
 *          - Return path: series of waypoints from mission area back to landing
 *          - May include safety corridors, obstacle avoidance, restricted airspace
 *          - More sophisticated than straight-line RTL
 *          - Allows mission-specific return procedures
 * 
 *          Mission Structure with Return Path:
 *          ```
 *          1. NAV_TAKEOFF
 *          2-10. Survey waypoints (outbound mission)
 *          11. DO_RETURN_PATH_START ← Marks start of return path
 *          12-15. Return waypoints (safe path home)
 *          16. DO_LAND_START ← Marks start of landing sequence
 *          17-20. Landing approach waypoints
 *          21. NAV_LAND
 *          ```
 * 
 *          Rejoin Logic:
 *          1. Calculate vehicle's current stopping point
 *          2. Search mission for DO_RETURN_PATH_START command
 *          3. Find all mission legs after DO_RETURN_PATH_START
 *          4. Calculate distance from stopping point to each leg
 *          5. Jump to closest mission leg (minimize backtracking)
 *          6. Resume mission from that waypoint
 *          7. Enter Auto RTL pseudo-mode
 * 
 *          Stopping Point Calculation:
 *          - get_stopping_point() returns position where vehicle would stop
 *          - Accounts for current velocity and deceleration limits
 *          - Used to find optimal rejoin point on return path
 *          - Prevents overshooting or excessive maneuvering
 *          - Minimizes time and distance to landing
 * 
 *          Closest Leg Algorithm:
 *          - mission.jump_to_closest_mission_leg() finds best rejoin point
 *          - Considers all waypoint-to-waypoint legs after DO_RETURN_PATH_START
 *          - Calculates perpendicular distance to each leg
 *          - Chooses leg with minimum distance
 *          - May skip waypoints if already past them on return path
 * 
 *          Success Path:
 *          - mission.jump_to_closest_mission_leg() returns true
 *          - Vehicle jumps to closest point on return path
 *          - Calls enter_auto_rtl() to activate Auto RTL pseudo-mode
 *          - Sets auto_RTL flag (now in "Auto RTL" state)
 *          - Mission execution continues along return path
 *          - Eventually reaches landing sequence
 *          - Returns true to indicate success
 * 
 *          Failure Path:
 *          - No DO_RETURN_PATH_START found in mission
 *          - Mission empty or invalid
 *          - Return path too far from current position
 *          - Logs AUTO_RTL error to dataflash
 *          - Triggers user notification (sad tone)
 *          - Sends GCS message "No return path found"
 *          - Returns false to indicate failure
 * 
 *          When This Method Used:
 *          - Triggered by RTL_ALT_TYPE = 4 (Auto RTL via return path)
 *          - Failsafe configured to use return path RTL
 *          - Useful for beyond visual line of sight (BVLOS) missions
 *          - Required when direct home path blocked by obstacles/airspace
 *          - Professional/commercial operations with flight corridors
 * 
 *          Comparison to Other Auto RTL Methods:
 *          - jump_to_landing_sequence_auto_RTL(): Skip return path, go directly to landing
 *          - return_path_start_auto_RTL(): Follow return path (this function)
 *          - return_path_or_jump_to_landing_sequence_auto_RTL(): Try return path, fallback to direct landing
 * 
 *          Mission Planning Requirements:
 *          - Mission MUST contain DO_RETURN_PATH_START command
 *          - Return path should be accessible from any mission location
 *          - Return waypoints should form safe corridor back to landing
 *          - Consider wind, obstacles, airspace restrictions
 *          - Include altitude changes if needed for safety/regulations
 *          - DO_LAND_START typically follows return path
 * 
 *          Advantages Over Standard RTL:
 *          - Follows predefined safe corridors
 *          - Respects airspace restrictions
 *          - Avoids known obstacles (terrain, buildings)
 *          - Consistent behavior for mission planning
 *          - Can include altitude changes, speed changes
 *          - Supports complex return procedures
 * 
 *          Error Handling:
 *          - Logs error code to dataflash for analysis
 *          - Triggers AP_Notify "user_mode_change_failed" event
 *          - Sad tone alerts pilot to failure
 *          - GCS text message indicates "No return path found"
 *          - Mode change aborted (vehicle stays in current mode)
 *          - Pilot should fall back to standard RTL or manual control
 * 
 *          Mode Reason Parameter:
 *          - reason logged to explain why Auto RTL triggered
 *          - Examples: ModeReason::RC_FAILSAFE, BATTERY_FAILSAFE
 *          - Recorded in dataflash MODE message
 *          - Useful for post-flight analysis
 *          - Passed to enter_auto_rtl() for logging
 * 
 *          Use Cases:
 *          - Long-range missions with specific return corridors
 *          - BVLOS operations with air traffic coordination
 *          - Missions in controlled airspace
 *          - Operations near obstacles (mountains, buildings)
 *          - Professional surveys with flight plan requirements
 *          - Multi-leg missions requiring specific routing
 * 
 * @param[in] reason Why Auto RTL triggered (RC failsafe, battery, GCS, etc.) - logged
 * 
 * @return true if return path found and Auto RTL mode entered successfully
 * @return false if no DO_RETURN_PATH_START found or rejoin failed
 * 
 * @note Mission must contain DO_RETURN_PATH_START command for this to succeed
 * @note Rejoins return path at closest point to minimize maneuvering
 * @note Calculates vehicle stopping point to prevent overshoot
 * @note Failure triggers error log, notification, and GCS message
 * @note On failure, vehicle remains in current flight mode
 * 
 * @warning Mission without DO_RETURN_PATH_START will cause Auto RTL to fail
 * @warning Return path must be reachable from mission area
 * @warning Always test Auto RTL behavior before flight
 * 
 * @see jump_to_landing_sequence_auto_RTL() - Alternative: skip return path, go directly to landing
 * @see return_path_or_jump_to_landing_sequence_auto_RTL() - Try return path, fallback to direct landing
 * @see enter_auto_rtl() - Activates Auto RTL pseudo-mode
 * @see mission.jump_to_closest_mission_leg() - AP_Mission method to find closest rejoin point
 * @see get_stopping_point() - Calculate where vehicle would stop given current state
 */
// Join mission after DO_RETURN_PATH_START waypoint, if succeeds pretend to be Auto RTL mode
bool ModeAuto::return_path_start_auto_RTL(ModeReason reason)
{
    if (!mission.jump_to_closest_mission_leg(get_stopping_point())) {
        LOGGER_WRITE_ERROR(LogErrorSubsystem::FLIGHT_MODE, LogErrorCode(Number::AUTO_RTL));
        // make sad noise
        if (copter.ap.initialised) {
            AP_Notify::events.user_mode_change_failed = 1;
        }
        gcs().send_text(MAV_SEVERITY_WARNING, "Mode change to AUTO RTL failed: No return path found");
        return false;
    }

    return enter_auto_rtl(reason);
}

/**
 * @brief Try return path first, fallback to direct landing, entering Auto RTL pseudo-mode
 * 
 * @details This function implements a hybrid Auto RTL strategy, providing maximum mission
 *          flexibility and reliability. It first attempts to rejoin the mission's return path
 *          (DO_RETURN_PATH_START), and if that fails, falls back to jumping directly to the
 *          landing sequence (DO_LAND_START). This "best effort" approach maximizes the chance
 *          of successful mission-based RTL.
 * 
 *          Two-Stage Attempt Strategy:
 *          Stage 1: Try to join return path (preferred)
 *          - Search for DO_RETURN_PATH_START in mission
 *          - Calculate closest mission leg to current position
 *          - Rejoin return path if found and accessible
 *          - Provides full return corridor + landing sequence
 *          
 *          Stage 2: Fallback to direct landing (if Stage 1 fails)
 *          - Search for DO_LAND_START in mission
 *          - Jump directly to landing sequence
 *          - Skip return path, go straight to landing approach
 *          - Shorter but still mission-based
 * 
 *          Stopping Point Calculation:
 *          - Calculated once: const Location stopping_point = get_stopping_point()
 *          - Represents where vehicle would stop given current velocity
 *          - Used for both return path and landing sequence search
 *          - Accounts for deceleration to choose optimal rejoin point
 *          - Single calculation improves performance and consistency
 * 
 *          Stage 1: Return Path Attempt:
 *          - mission.jump_to_closest_mission_leg(stopping_point) called first
 *          - Searches for DO_RETURN_PATH_START marker
 *          - Finds closest waypoint leg after marker
 *          - Returns true if return path found and accessible
 *          - If successful, enter_auto_rtl() called and function returns true
 * 
 *          Stage 2: Landing Sequence Fallback:
 *          - Only attempted if Stage 1 fails (returns false)
 *          - mission.jump_to_landing_sequence(stopping_point) called
 *          - Searches for DO_LAND_START marker
 *          - Finds landing approach closest to current position
 *          - Returns true if landing sequence found
 *          - If successful, enter_auto_rtl() called and function returns true
 * 
 *          Complete Failure Path:
 *          - Both attempts fail (neither marker found)
 *          - Mission invalid, empty, or missing required markers
 *          - Logs AUTO_RTL error to dataflash
 *          - Triggers user notification (sad tone)
 *          - Sends GCS message "No return path or landing sequence found"
 *          - Returns false to indicate failure
 * 
 *          Success Paths (in order of preference):
 *          Path A (Best): Return path found
 *          - Vehicle rejoins at DO_RETURN_PATH_START
 *          - Follows return waypoints back to landing area
 *          - Proceeds through full landing sequence
 *          - Maximum safety and predictability
 *          
 *          Path B (Good): Landing sequence found
 *          - Vehicle skips return path entirely
 *          - Jumps directly to DO_LAND_START
 *          - Follows landing approach waypoints
 *          - Faster but skips return corridor
 * 
 *          When This Method Used:
 *          - Triggered by RTL_ALT_TYPE = 5 (Auto RTL with fallback)
 *          - Most robust Auto RTL configuration
 *          - Recommended for missions with both markers
 *          - Provides maximum flexibility
 *          - Handles partial mission uploads gracefully
 * 
 *          Comparison to Other Auto RTL Methods:
 *          - jump_to_landing_sequence_auto_RTL(): Only tries direct landing (no return path)
 *          - return_path_start_auto_RTL(): Only tries return path (no fallback)
 *          - return_path_or_jump_to_landing_sequence_auto_RTL(): Tries both (this function - most robust)
 * 
 *          Mission Planning Best Practices:
 *          - Include BOTH DO_RETURN_PATH_START and DO_LAND_START
 *          - DO_RETURN_PATH_START for normal operations
 *          - DO_LAND_START provides safety fallback
 *          - Return path should be reachable from mission area
 *          - Landing sequence should be reachable from anywhere
 *          - Test both paths before flight
 * 
 *          Mission Structure Example:
 *          ```
 *          1. NAV_TAKEOFF
 *          2-8. Mission waypoints
 *          9. DO_RETURN_PATH_START ← Stage 1 tries to reach this
 *          10-13. Return corridor waypoints
 *          14. DO_LAND_START ← Stage 2 falls back to this
 *          15-17. Landing approach
 *          18. NAV_LAND
 *          ```
 * 
 *          Advantages:
 *          - Maximum robustness (two chances to succeed)
 *          - Graceful degradation (return path → direct landing)
 *          - Handles incomplete missions
 *          - Works if vehicle very far from return path
 *          - Reduces Auto RTL failure rate
 *          - Preferred for operational missions
 * 
 *          Error Handling:
 *          - Only fails if BOTH stages fail
 *          - Logs single error (not two separate errors)
 *          - Error message indicates both attempts failed
 *          - Clear GCS message helps pilot understand situation
 *          - Pilot should fall back to standard RTL or manual
 * 
 *          Performance Considerations:
 *          - Stopping point calculated once (efficient)
 *          - Return path attempted first (preferred path)
 *          - Landing sequence only attempted if return path fails
 *          - Short-circuit evaluation (if first succeeds, second not attempted)
 *          - Minimal computational overhead
 * 
 *          Mode Reason Parameter:
 *          - reason logged to explain why Auto RTL triggered
 *          - Recorded in dataflash MODE message
 *          - Useful for post-flight analysis
 *          - Helps identify failsafe patterns
 *          - Passed to enter_auto_rtl() for logging
 * 
 *          Use Cases:
 *          - Default Auto RTL configuration (most robust)
 *          - Long-range missions where return path may be far
 *          - Missions with partial uploads
 *          - Testing and development (handles errors gracefully)
 *          - Commercial operations requiring high reliability
 * 
 * @param[in] reason Why Auto RTL triggered (RC failsafe, battery, GCS, etc.) - logged
 * 
 * @return true if either return path OR landing sequence found and Auto RTL entered
 * @return false only if BOTH return path and landing sequence missing from mission
 * 
 * @note Most robust Auto RTL method - tries two strategies
 * @note Prefers return path (Stage 1), falls back to direct landing (Stage 2)
 * @note Calculates stopping point once for efficiency
 * @note Only fails if mission missing both DO_RETURN_PATH_START and DO_LAND_START
 * @note Recommended configuration for operational missions
 * 
 * @warning Mission should contain at least one marker (preferably both)
 * @warning Complete failure only if mission lacks both return path and landing sequence
 * @warning Always test Auto RTL behavior with actual mission before flight
 * 
 * @see jump_to_landing_sequence_auto_RTL() - Direct landing only (no return path attempt)
 * @see return_path_start_auto_RTL() - Return path only (no landing fallback)
 * @see enter_auto_rtl() - Activates Auto RTL pseudo-mode
 * @see mission.jump_to_closest_mission_leg() - Stage 1: Find return path
 * @see mission.jump_to_landing_sequence() - Stage 2: Find landing sequence
 * @see get_stopping_point() - Calculate vehicle stopping position
 */
// Try join return path else do land start
bool ModeAuto::return_path_or_jump_to_landing_sequence_auto_RTL(ModeReason reason)
{
    const Location stopping_point = get_stopping_point();
    if (!mission.jump_to_closest_mission_leg(stopping_point) && !mission.jump_to_landing_sequence(stopping_point)) {
        LOGGER_WRITE_ERROR(LogErrorSubsystem::FLIGHT_MODE, LogErrorCode(Number::AUTO_RTL));
        // make sad noise
        if (copter.ap.initialised) {
            AP_Notify::events.user_mode_change_failed = 1;
        }
        gcs().send_text(MAV_SEVERITY_WARNING, "Mode change to AUTO RTL failed: No return path or landing sequence found");
        return false;
    }

    return enter_auto_rtl(reason);
}

/**
 * @brief Activate Auto RTL pseudo-mode within Auto mode
 * 
 * @details This function completes the Auto RTL entry process by switching to Auto mode
 *          (if not already in it), setting the auto_RTL flag to distinguish it from normal
 *          Auto mode, and configuring the mission to force resume. This creates a "pseudo-mode"
 *          where the vehicle is technically in Auto mode but behaving as Auto RTL.
 * 
 *          Auto RTL Pseudo-Mode Concept:
 *          - Not a separate flight mode (uses Auto mode infrastructure)
 *          - Distinguished by auto_RTL flag being true
 *          - Appears as "AUTO RTL" in telemetry and logs
 *          - Prevents certain operations (e.g., arming)
 *          - Uses mission commands for return and landing
 *          - More flexible than standard RTL mode
 * 
 *          Force Resume Configuration:
 *          - mission.set_force_resume(true) called first
 *          - Forces mission to resume even if normally wouldn't
 *          - Overrides MIS_RESTART parameter
 *          - Ensures mission continues from Auto RTL entry point
 *          - Prevents mission restarting from beginning
 *          - Critical for Auto RTL to work correctly
 * 
 *          Mode Switch Logic:
 *          - Check if already in Auto mode: (copter.flightmode == this)
 *          - If yes: Already in Auto, just set flag and proceed
 *          - If no: Call set_mode(Mode::Number::AUTO, reason)
 *          - set_mode() handles full mode transition
 *          - Initializes Auto mode if coming from different mode
 * 
 *          Success Path:
 *          1. Force resume set to true
 *          2. Mode switch successful (or already in Auto)
 *          3. Set auto_RTL flag = true
 *          4. Log mode entry with reason
 *          5. Trigger happy notification (success tone)
 *          6. Return true
 * 
 *          Failure Path (Mode Switch Failed):
 *          1. set_mode() returns false
 *          2. Revert force resume to false
 *          3. Log AUTO_RTL error
 *          4. Trigger sad notification (failure tone)
 *          5. Return false
 * 
 *          auto_RTL Flag Effects:
 *          - Distinguishes Auto RTL from normal Auto
 *          - Checked by allows_arming() → prevents arming in Auto RTL
 *          - Used in run() to determine when to clear flag
 *          - Affects telemetry reporting to GCS
 *          - Logged separately in dataflash
 *          - Helps pilots understand vehicle state
 * 
 *          Mode Logging:
 *          - Write_Mode() logs mode change to dataflash
 *          - Records mode number (AUTO = 3)
 *          - Records reason for change (failsafe type)
 *          - Separate entry logged specifically for Auto RTL
 *          - Distinguishable from normal Auto in logs
 *          - Essential for post-flight analysis
 * 
 *          Notification Events:
 *          Success:
 *          - AP_Notify::events.user_mode_change = 1
 *          - Triggers happy tone
 *          - LED pattern indicates mode change
 *          - Confirms Auto RTL activation to pilot
 *          
 *          Failure:
 *          - AP_Notify::events.user_mode_change_failed = 1
 *          - Triggers sad tone
 *          - Alerts pilot to failure
 *          - Pilot should take manual control
 * 
 *          Why Mode Switch Might Fail:
 *          - GPS not available (Auto requires GPS)
 *          - No valid position estimate
 *          - EKF not initialized
 *          - Vehicle disarmed (shouldn't happen in failsafe)
 *          - Internal error in Auto mode initialization
 * 
 *          Force Resume Purpose:
 *          - MIS_RESTART parameter controls mission behavior
 *          - MIS_RESTART = 0: Resume from last command
 *          - MIS_RESTART = 1: Restart from beginning
 *          - Auto RTL always needs to resume (not restart)
 *          - set_force_resume(true) overrides MIS_RESTART
 *          - Ensures mission continues from Auto RTL jump point
 * 
 *          Cleanup on Failure:
 *          - If mode switch fails, force resume must be cleared
 *          - mission.set_force_resume(false) reverts setting
 *          - Prevents force resume affecting next mission operation
 *          - Maintains mission state machine integrity
 *          - Important for mission system reliability
 * 
 *          Called By:
 *          - jump_to_landing_sequence_auto_RTL()
 *          - return_path_start_auto_RTL()
 *          - return_path_or_jump_to_landing_sequence_auto_RTL()
 *          - All three Auto RTL entry methods
 * 
 *          Subsequent Behavior:
 *          - Vehicle now in Auto mode with auto_RTL flag set
 *          - Mission executes from jump point
 *          - run() method called at 100Hz+
 *          - Mission commands processed normally
 *          - auto_RTL flag cleared when landing complete or mission ends
 * 
 *          Telemetry Reporting:
 *          - GCS sees mode as "AUTO RTL" (not just "AUTO")
 *          - Special handling in GCS_MAVLink for Auto RTL
 *          - Pilots can distinguish from normal Auto
 *          - Important for situational awareness
 * 
 *          Arming Prevention:
 *          - allows_arming() returns false if auto_RTL is true
 *          - Prevents arming while in Auto RTL pseudo-mode
 *          - Safety feature: Auto RTL is for returning/landing
 *          - Must exit Auto RTL before arming again
 * 
 *          Auto RTL Exit:
 *          - auto_RTL flag cleared in run() when landing complete
 *          - Also cleared in exit() when leaving Auto mode
 *          - Mission completes normally
 *          - Vehicle disarms on landing (if configured)
 * 
 * @param[in] reason Why Auto RTL triggered (RC failsafe, battery, GCS, etc.) - logged
 * 
 * @return true if Auto mode entered successfully and Auto RTL activated
 * @return false if mode switch failed (no GPS, no position, EKF not ready)
 * 
 * @note Sets mission to force resume regardless of MIS_RESTART parameter
 * @note Sets auto_RTL flag to distinguish from normal Auto mode
 * @note Logs mode entry separately for Auto RTL
 * @note Triggers notification events (happy tone on success, sad on failure)
 * @note Prevents arming while in Auto RTL pseudo-mode
 * 
 * @warning Mode switch can fail if GPS/position not available
 * @warning On failure, force resume is reverted to maintain mission integrity
 * @warning Called after mission jump has already succeeded
 * 
 * @see jump_to_landing_sequence_auto_RTL() - Calls this after jumping to landing
 * @see return_path_start_auto_RTL() - Calls this after joining return path
 * @see return_path_or_jump_to_landing_sequence_auto_RTL() - Calls this after either jump
 * @see allows_arming() - Checks auto_RTL flag to prevent arming
 * @see run() - Clears auto_RTL flag when landing complete
 * @see exit() - Clears auto_RTL flag when leaving Auto mode
 */
// Enter auto rtl pseudo mode
bool ModeAuto::enter_auto_rtl(ModeReason reason) 
{
    mission.set_force_resume(true);

    // if not already in auto switch to auto
    if ((copter.flightmode == this) || set_mode(Mode::Number::AUTO, reason)) {
        auto_RTL = true;
#if HAL_LOGGING_ENABLED
        // log entry into AUTO RTL
        copter.logger.Write_Mode((uint8_t)copter.flightmode->mode_number(), reason);
#endif

        // make happy noise
        if (copter.ap.initialised) {
            AP_Notify::events.user_mode_change = 1;
        }
        return true;
    }

    // mode change failed, revert force resume flag
    mission.set_force_resume(false);

    LOGGER_WRITE_ERROR(LogErrorSubsystem::FLIGHT_MODE, LogErrorCode(Number::AUTO_RTL));
    // make sad noise
    if (copter.ap.initialised) {
        AP_Notify::events.user_mode_change_failed = 1;
    }
    return false;
}

/**
 * @brief Retrieve active NAV_SCRIPT_TIME command parameters for Lua script execution
 * 
 * @details This function provides the interface for Lua scripts to access the parameters
 *          of an active NAV_SCRIPT_TIME mission command. When the mission executes a
 *          NAV_SCRIPT_TIME command, it transitions to the NAV_SCRIPT_TIME submode and
 *          stores the command parameters. Lua scripts call this function to retrieve those
 *          parameters and execute custom navigation logic.
 * 
 *          NAV_SCRIPT_TIME Command Concept:
 *          - MAV_CMD_NAV_SCRIPT_TIME (mission command ID 42001)
 *          - Hands control over to Lua scripting for specified time
 *          - Script receives command parameters via this function
 *          - Script controls vehicle position/velocity/attitude
 *          - Allows custom navigation behaviors in missions
 *          - Script signals completion via nav_script_time_done()
 * 
 *          Submode Check:
 *          - Only returns data if _mode == SubMode::NAV_SCRIPT_TIME
 *          - Ensures script only gets data when command is active
 *          - Prevents stale data from previous commands
 *          - Returns false if in any other submode
 * 
 *          Parameter Structure (nav_scripting):
 *          - id: Unique identifier incremented each command execution
 *          - command: Script command number (user-defined meaning)
 *          - arg1: Float argument 1 (meters, seconds, etc.)
 *          - arg2: Float argument 2
 *          - arg3: Integer argument 3
 *          - arg4: Integer argument 4
 * 
 *          Parameter Usage Patterns:
 *          Example 1 - Custom Circle:
 *          - command = 1 (user defines as "circle")
 *          - arg1 = radius in meters
 *          - arg2 = rate in degrees/second
 *          - arg3 = direction (0=CW, 1=CCW)
 *          - arg4 = altitude change in cm
 *          
 *          Example 2 - Figure-8:
 *          - command = 2 (user defines as "figure-8")
 *          - arg1 = lobe size in meters
 *          - arg2 = completion time in seconds
 *          - arg3 = orientation in degrees
 *          - arg4 = unused
 * 
 *          ID Parameter Purpose:
 *          - Unique identifier for each NAV_SCRIPT_TIME execution
 *          - Incremented in do_nav_script_time()
 *          - Used to match completion signal in nav_script_time_done()
 *          - Prevents race conditions if script runs multiple times
 *          - Ensures script completes correct command instance
 * 
 *          Script Workflow:
 *          1. Mission reaches NAV_SCRIPT_TIME command
 *          2. do_nav_script_time() stores parameters and switches submode
 *          3. Script calls nav_script_time() to retrieve parameters
 *          4. Script executes custom navigation logic
 *          5. Script calls nav_script_time_done(id) to signal completion
 *          6. Mission advances to next command
 * 
 *          Scripting API Usage:
 *          ```lua
 *          function update()
 *              local id, cmd, arg1, arg2, arg3, arg4 = vehicle:nav_script_time()
 *              if id then
 *                  if cmd == 1 then
 *                      -- Execute custom circle
 *                      do_custom_circle(arg1, arg2)
 *                  elseif cmd == 2 then
 *                      -- Execute figure-8
 *                      do_figure_8(arg1, arg2)
 *                  end
 *                  
 *                  if completed then
 *                      vehicle:nav_script_time_done(id)
 *                  end
 *              end
 *              return update, 100 -- call at 10Hz
 *          end
 *          ```
 * 
 *          Output Parameters:
 *          - All parameters passed by reference (modified by function)
 *          - Only valid if function returns true
 *          - Undefined if function returns false
 *          - Script must check return value before using parameters
 * 
 *          Return Value Logic:
 *          - Returns true: NAV_SCRIPT_TIME active, parameters valid
 *          - Returns false: Not in NAV_SCRIPT_TIME mode, parameters invalid
 *          - Script should handle both cases gracefully
 *          - False return means script should not take action
 * 
 *          Compilation Configuration:
 *          - Wrapped in #if AP_SCRIPTING_ENABLED
 *          - Only compiled if scripting support enabled
 *          - Returns false if scripting not enabled
 *          - Zero overhead when scripting disabled
 * 
 *          Timeout Handling:
 *          - NAV_SCRIPT_TIME includes timeout parameter
 *          - Stored in nav_scripting.timeout_s
 *          - Verified in verify_nav_script_time()
 *          - If timeout expires, mission continues automatically
 *          - Prevents mission getting stuck if script fails
 * 
 *          Script Responsibility:
 *          - Script must call copter.mode_guided functions to control vehicle
 *          - Script must monitor position/velocity/attitude
 *          - Script must detect completion conditions
 *          - Script must call nav_script_time_done() when complete
 *          - Script must handle errors gracefully
 * 
 *          Safety Considerations:
 *          - Script has full control during NAV_SCRIPT_TIME
 *          - Script must respect safety limits (altitude, speed, etc.)
 *          - Timeout provides safety fallback
 *          - Pilot can override with mode switch
 *          - Failsafes still active during script execution
 * 
 *          Use Cases:
 *          - Custom survey patterns
 *          - Complex inspection maneuvers
 *          - Search patterns
 *          - Photography sequences
 *          - Research/experimental navigation
 *          - Precision positioning tasks
 * 
 * @param[out] id Unique command instance identifier - use with nav_script_time_done()
 * @param[out] cmd Script command number (user-defined meaning)
 * @param[out] arg1 Float argument 1 (meters, seconds, or user-defined)
 * @param[out] arg2 Float argument 2 (rate, distance, or user-defined)
 * @param[out] arg3 Integer argument 3 (flags, mode, or user-defined)
 * @param[out] arg4 Integer argument 4 (additional parameter or user-defined)
 * 
 * @return true if NAV_SCRIPT_TIME command active and parameters valid
 * @return false if not in NAV_SCRIPT_TIME submode (parameters undefined)
 * 
 * @note Only returns true when in NAV_SCRIPT_TIME submode
 * @note Output parameters only valid if function returns true
 * @note ID used to match completion signal in nav_script_time_done()
 * @note Command and argument meanings are user-defined
 * @note Called by Lua scripts via scripting API binding
 * 
 * @warning Parameters undefined if function returns false
 * @warning Script must check return value before using parameters
 * @warning Only compiled if AP_SCRIPTING_ENABLED
 * 
 * @see nav_script_time_done() - Script calls this to signal command completion
 * @see do_nav_script_time() - Stores parameters when NAV_SCRIPT_TIME command starts
 * @see verify_nav_script_time() - Checks for completion or timeout
 * @see nav_guided_run() - Executes guided mode control during script execution
 */
// lua scripts use this to retrieve the contents of the active command
bool ModeAuto::nav_script_time(uint16_t &id, uint8_t &cmd, float &arg1, float &arg2, int16_t &arg3, int16_t &arg4)
{
#if AP_SCRIPTING_ENABLED
    if (_mode == SubMode::NAV_SCRIPT_TIME) {
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

/**
 * @brief Signal NAV_SCRIPT_TIME command completion from Lua script
 * 
 * @details This function allows Lua scripts to signal that they have completed the custom
 *          navigation task specified by a NAV_SCRIPT_TIME mission command. When the script
 *          calls this function with the correct ID, the mission system will advance to the
 *          next command. This provides the "script-to-mission" communication necessary for
 *          mission-integrated scripted navigation.
 * 
 *          Completion Signaling Purpose:
 *          - Mission waits for script to signal completion
 *          - Script determines when navigation task is complete
 *          - Allows flexible completion conditions
 *          - Mission can't advance until signal received (or timeout)
 *          - Provides synchronization between script and mission
 * 
 *          ID Matching Requirement:
 *          - Function checks if id == nav_scripting.id
 *          - ID must match current command instance
 *          - Prevents stale signals from previous commands
 *          - Prevents race conditions in multi-command scenarios
 *          - Only sets done flag if IDs match
 * 
 *          Submode Verification:
 *          - Only processes signal if _mode == SubMode::NAV_SCRIPT_TIME
 *          - Ensures command is still active
 *          - Ignores signals if mission already moved on
 *          - Prevents incorrect state transitions
 * 
 *          Done Flag Effect:
 *          - Sets nav_scripting.done = true
 *          - verify_nav_script_time() checks this flag
 *          - When true, verify function returns true
 *          - Mission advances to next command
 *          - Flag reset when next NAV_SCRIPT_TIME starts
 * 
 *          Script Workflow:
 *          1. Script retrieves parameters via nav_script_time()
 *          2. Script executes custom navigation logic
 *          3. Script monitors completion conditions
 *          4. When complete, script calls nav_script_time_done(id)
 *          5. done flag set to true
 *          6. verify_nav_script_time() returns true
 *          7. Mission advances to next command
 * 
 *          Completion Condition Examples:
 *          - Reached target position (custom tolerance)
 *          - Completed specified number of iterations
 *          - Elapsed specified amount of time
 *          - Detected target object in camera
 *          - Received external command
 *          - User-defined criteria met
 * 
 *          Scripting API Usage:
 *          ```lua
 *          function update()
 *              local id, cmd, arg1, arg2, arg3, arg4 = vehicle:nav_script_time()
 *              if id then
 *                  -- Execute navigation task
 *                  local completed = do_navigation_task(cmd, arg1, arg2)
 *                  
 *                  if completed then
 *                      -- Signal completion with matching ID
 *                      vehicle:nav_script_time_done(id)
 *                  end
 *              end
 *              return update, 100 -- 10Hz
 *          end
 *          ```
 * 
 *          Why ID Parameter Required:
 *          - Prevents stale signals from affecting new commands
 *          - Each NAV_SCRIPT_TIME execution has unique ID
 *          - ID incremented in do_nav_script_time()
 *          - Script must pass ID back to match command instance
 *          - Ensures script completes correct command
 * 
 *          Multiple NAV_SCRIPT_TIME Commands:
 *          - Mission can contain multiple NAV_SCRIPT_TIME commands
 *          - Each gets unique ID (incremented)
 *          - Script must track current ID
 *          - Can't complete command with wrong ID
 *          - Prevents cross-command interference
 * 
 *          Timeout Behavior:
 *          - NAV_SCRIPT_TIME includes timeout parameter
 *          - If timeout expires before done signal, mission continues
 *          - Timeout checked in verify_nav_script_time()
 *          - Provides safety if script fails to signal completion
 *          - Script should complete before timeout
 * 
 *          Safety Considerations:
 *          - Script must actually complete task before signaling
 *          - Premature signaling could skip navigation steps
 *          - Script should verify completion conditions thoroughly
 *          - Timeout provides failsafe if script hangs
 *          - Mission won't advance without done or timeout
 * 
 *          Verification Process:
 *          - verify_nav_script_time() called at 10Hz+ by mission system
 *          - Checks nav_scripting.done flag
 *          - Also checks for timeout
 *          - Returns true if either condition met
 *          - Mission advances when verify returns true
 * 
 *          Ignored Signals:
 *          - Signal ignored if not in NAV_SCRIPT_TIME submode
 *          - Signal ignored if ID doesn't match
 *          - No error generated for ignored signals
 *          - Script can call safely even if not active
 * 
 *          Compilation Configuration:
 *          - Wrapped in #if AP_SCRIPTING_ENABLED
 *          - Only compiled if scripting support enabled
 *          - No-op if scripting not enabled
 *          - Zero overhead when scripting disabled
 * 
 *          Script Responsibility:
 *          - Script must detect its own completion conditions
 *          - Script must call this function when complete
 *          - Script must pass correct ID
 *          - Script should handle timeout gracefully
 *          - Script should not signal prematurely
 * 
 *          Use Cases:
 *          - Custom survey pattern completion
 *          - Inspection task finished
 *          - Search pattern exhausted
 *          - Photography sequence complete
 *          - User-defined task accomplished
 * 
 * @param[in] id Command instance identifier from nav_script_time() - must match current command
 * 
 * @note Only sets done flag if in NAV_SCRIPT_TIME submode and ID matches
 * @note Called by Lua scripts via scripting API binding
 * @note Mission advances to next command when done flag set
 * @note Ignored if ID doesn't match current command
 * @note Provides synchronization between script and mission system
 * 
 * @warning Script must verify task actually complete before calling
 * @warning Premature completion signal can skip navigation steps
 * @warning Only compiled if AP_SCRIPTING_ENABLED
 * 
 * @see nav_script_time() - Script calls this to retrieve command parameters and ID
 * @see verify_nav_script_time() - Checks done flag to determine if mission should advance
 * @see do_nav_script_time() - Initializes NAV_SCRIPT_TIME command and generates unique ID
 */
// lua scripts use this to indicate when they have complete the command
void ModeAuto::nav_script_time_done(uint16_t id)
{
#if AP_SCRIPTING_ENABLED
    if ((_mode == SubMode::NAV_SCRIPT_TIME) && (id == nav_scripting.id)) {
        nav_scripting.done = true;
    }
#endif
}

/**
 * @brief Initialize loiter mode within Auto flight mode
 * 
 * @details This function transitions Auto mode into a loiter (hold position) submode, calculating
 *          the vehicle's stopping point and commanding the waypoint controller to maintain that
 *          position. This is used when the mission ends, when a loiter command is executed, or
 *          as a fallback when landing is not possible.
 * 
 *          Loiter Purpose in Auto:
 *          - Hold position when mission completes
 *          - Execute NAV_LOITER_UNLIM or NAV_LOITER_TIME commands
 *          - Fallback when landing not possible (terrain data missing)
 *          - Safe state to await pilot commands
 *          - Maintains altitude and horizontal position
 * 
 *          Position Estimate Requirement:
 *          - Checks copter.position_ok() before proceeding
 *          - position_ok() requires valid GPS and EKF position estimate
 *          - Returns false if position not available
 *          - Prevents loiter without position knowledge
 *          - Caller must handle failure appropriately
 * 
 *          Failure Scenarios:
 *          - GPS not available or unhealthy
 *          - EKF position estimate not valid
 *          - Position accuracy insufficient
 *          - In these cases, function returns false
 *          - Caller (e.g., exit_mission) tries landing instead
 * 
 *          Stopping Point Calculation:
 *          - wp_nav->get_wp_stopping_point_NEU_cm() calculates where vehicle would stop
 *          - Accounts for current velocity
 *          - Applies deceleration limits
 *          - Returns position in NEU frame (North-East-Up, cm)
 *          - NEU frame: X=North, Y=East, Z=Up from EKF origin
 * 
 *          NEU Coordinate Frame:
 *          - North-East-Up coordinate system
 *          - Origin at EKF origin (typically home position)
 *          - X axis: North (positive = north)
 *          - Y axis: East (positive = east)
 *          - Z axis: Up (positive = up, altitude increase)
 *          - Units: centimeters
 * 
 *          Waypoint Controller Configuration:
 *          - wp_nav->set_wp_destination_NEU_cm(stopping_point)
 *          - Sets destination to calculated stopping point
 *          - Waypoint controller will fly to that point and hold
 *          - Uses position control to maintain position against wind
 *          - Altitude component maintains current altitude
 * 
 *          Submode Transition:
 *          - _mode = SubMode::LOITER
 *          - Auto mode's run() will call loiter_run()
 *          - loiter_run() updates waypoint navigation at 100Hz+
 *          - Position held until mission command changes or mode exit
 * 
 *          Yaw Behavior:
 *          - auto_yaw.set_mode(AutoYaw::Mode::HOLD)
 *          - Holds current heading
 *          - Vehicle doesn't rotate while loitering
 *          - Can be overridden by pilot yaw input (if allowed)
 *          - Different from CIRCLE yaw (which points at center)
 * 
 *          Called By:
 *          - exit_mission() - When mission completes
 *          - do_loiter_unlimited() - NAV_LOITER_UNLIM command
 *          - do_loiter_time() - NAV_LOITER_TIME command
 *          - do_loiter_to_alt() - NAV_LOITER_TO_ALT command
 *          - Fallback when landing not possible
 * 
 *          exit_mission() Usage:
 *          - Mission complete, vehicle not landed
 *          - Tries loiter_start() first
 *          - If loiter fails (no position), tries landing instead
 *          - Provides safe fallback behavior
 * 
 *          Loiter vs Land Decision:
 *          - exit_mission() prefers loiter over landing
 *          - Loiter keeps vehicle airborne (safer in many scenarios)
 *          - Pilot can take control or plan next action
 *          - Landing is automatic and may not be desired
 *          - Loiter requires position; landing can work without
 * 
 *          Success Conditions:
 *          - Valid position estimate available
 *          - Stopping point calculated successfully
 *          - Waypoint destination set successfully
 *          - Submode changed to LOITER
 *          - Yaw mode set to HOLD
 *          - Returns true
 * 
 *          Failure Handling:
 *          - Returns false immediately if position not available
 *          - No state changes on failure
 *          - Caller must handle failure (try alternative action)
 *          - Typical fallback: try landing (doesn't require precise position)
 * 
 *          Loiter Execution:
 *          - loiter_run() called at 100Hz+ by Auto::run()
 *          - Updates waypoint navigation
 *          - Maintains position against wind/disturbances
 *          - Continues until mission advances or mode change
 * 
 *          Wind Compensation:
 *          - Waypoint controller actively maintains position
 *          - Adjusts attitude to compensate for wind
 *          - Uses position feedback from EKF
 *          - Can hold position in moderate winds
 * 
 *          Altitude Maintenance:
 *          - Stopping point includes current altitude
 *          - Altitude held at stopping point Z coordinate
 *          - Vertical position controller maintains altitude
 *          - Compensates for barometric drift
 * 
 *          Use Cases:
 *          - End-of-mission hold
 *          - Timed loiter during mission
 *          - Indefinite loiter until pilot action
 *          - Safe state while awaiting instructions
 * 
 * @return true if loiter initialized successfully (position available)
 * @return false if position estimate not available (GPS/EKF not ready)
 * 
 * @note Returns false if GPS or EKF position not available
 * @note Calculates stopping point accounting for current velocity
 * @note Sets waypoint destination to stopping point
 * @note Holds current heading (yaw)
 * @note Called by exit_mission() when mission completes
 * @note Called by loiter mission commands
 * 
 * @warning Requires valid position estimate to succeed
 * @warning Caller must handle failure (typically try landing instead)
 * 
 * @see loiter_run() - Executes loiter control at 100Hz+
 * @see exit_mission() - Calls this when mission ends
 * @see do_loiter_unlimited() - NAV_LOITER_UNLIM command handler
 * @see do_loiter_time() - NAV_LOITER_TIME command handler
 * @see copter.position_ok() - Checks if position estimate valid
 */
/**
 * @brief Initialize loitering at current position in AUTO mode
 * 
 * @details This function transitions the vehicle into a loiter state within AUTO mode,
 *          establishing a hold position for the waypoint controller. The vehicle will
 *          maintain its current horizontal position while holding altitude.
 *          
 *          Key operations:
 *          - Verifies GPS position quality before initiating loiter
 *          - Calculates stopping point based on current velocity and deceleration limits
 *          - Initializes waypoint controller with the stopping point as target
 *          - Sets yaw mode to hold current heading
 *          
 *          This function is called both during normal mission execution and by exit_mission()
 *          when the mission completes but the vehicle is not yet landed.
 * 
 * @return true if loiter successfully initialized
 * @return false if position estimate is unavailable (GPS failure or EKF not ready)
 * 
 * @note This is a navigation command initialization function, not the run loop
 * @note Called at mission command rate (approximately 10Hz)
 * 
 * @see loiter_run() for the corresponding execution function
 * @see exit_mission() for mission completion behavior
 */
bool ModeAuto::loiter_start()
{
    // return failure if GPS is bad
    if (!copter.position_ok()) {
        return false;
    }
    _mode = SubMode::LOITER;

    // calculate stopping point
    Vector3f stopping_point;
    wp_nav->get_wp_stopping_point_NEU_cm(stopping_point);

    // initialise waypoint controller target to stopping point
    wp_nav->set_wp_destination_NEU_cm(stopping_point);

    // hold yaw at current heading
    auto_yaw.set_mode(AutoYaw::Mode::HOLD);

    return true;
}

/**
 * @brief Initialize RTL (Return to Launch) as a submode within Auto flight mode
 * 
 * @details This function transitions Auto mode into an RTL submode, delegating all RTL navigation
 *          logic to the dedicated RTL flight mode controller. This is used when a NAV_RTL mission
 *          command is encountered, allowing a mission to include a "return home" step. The function
 *          bypasses RTL's normal safety checks since the mission system has already validated
 *          that RTL execution is appropriate.
 * 
 *          NAV_RTL Mission Command:
 *          - MAV_CMD_NAV_RETURN_TO_LAUNCH (command ID 20)
 *          - Mission command to return to launch position
 *          - Often used as failsafe or mission end step
 *          - Can be part of planned mission sequence
 *          - Triggers this function via do_RTL() -> rtl_start()
 * 
 *          RTL Mode Delegation:
 *          - Auto mode doesn't reimplement RTL logic
 *          - Delegates to copter.mode_rtl controller
 *          - RTL mode handles all navigation phases
 *          - Auto mode just wraps RTL execution
 *          - Maintains mission system control
 * 
 *          Ignore Checks Parameter:
 *          - Calls copter.mode_rtl.init(true)
 *          - true = ignore normal pre-flight checks
 *          - RTL normally checks if position available
 *          - Mission context assumes checks already done
 *          - Prevents redundant validation
 * 
 *          Why Ignore Checks:
 *          - Mission system validates commands before execution
 *          - Vehicle already in flight (Auto mode active)
 *          - Position must be available (Auto requires GPS)
 *          - Safety checks already passed to start Auto
 *          - Mission wouldn't execute if not safe
 * 
 *          RTL init(true) Behavior:
 *          - With ignore_checks=true, RTL init always succeeds
 *          - Sets up RTL navigation state machine
 *          - Configures climb, return, and descent phases
 *          - Returns true (guaranteed when ignoring checks)
 * 
 *          Submode Transition:
 *          - set_submode(SubMode::RTL)
 *          - Changes Auto's internal state to RTL
 *          - Auto::run() will call rtl_run()
 *          - rtl_run() delegates to copter.mode_rtl.run(false)
 * 
 *          RTL Execution Phases:
 *          1. Climb to RTL altitude (if below)
 *          2. Return to launch location horizontally
 *          3. Descend to landing altitude
 *          4. Land
 *          - All phases managed by RTL mode controller
 * 
 *          rtl_run() Integration:
 *          - Called by Auto::run() when _mode == SubMode::RTL
 *          - rtl_run() simply calls copter.mode_rtl.run(false)
 *          - false parameter indicates not standalone RTL mode
 *          - RTL controller updates at 100Hz+
 * 
 *          verify_RTL() Completion:
 *          - Mission system calls verify_RTL() to check completion
 *          - Checks copter.mode_rtl.state_complete()
 *          - RTL complete when landed and motors at ground idle
 *          - Mission advances to next command when complete
 * 
 *          Internal Error Handling:
 *          - If init returns false, logs INTERNAL_ERROR
 *          - Should never happen (init(true) always succeeds)
 *          - Indicates programming logic error
 *          - Defensive programming for unexpected cases
 * 
 *          Error Type: flow_of_control:
 *          - Indicates unexpected code path executed
 *          - Used for "should never happen" scenarios
 *          - Helps identify bugs during testing
 *          - Logged to AP_InternalError system
 * 
 *          Difference from Direct RTL Mode:
 *          - Direct RTL: Pilot switches to RTL mode
 *          - Mission RTL: NAV_RTL command in mission
 *          - Direct RTL: Full mode switch, disarms on land
 *          - Mission RTL: Submode, mission continues after land
 *          - Both use same RTL controller internally
 * 
 *          Mission Context Advantages:
 *          - RTL can be part of planned mission
 *          - Mission continues after RTL completes
 *          - Next command can be takeoff, waypoint, etc.
 *          - Allows complex mission patterns
 *          - Example: fly to waypoint, RTL, takeoff, continue
 * 
 *          Auto RTL vs Mission RTL:
 *          - Auto RTL: Failsafe-triggered return (jump_to_landing_sequence)
 *          - Mission RTL: Planned NAV_RTL command
 *          - Both use rtl_start() internally
 *          - Auto RTL may disarm after landing
 *          - Mission RTL continues mission
 * 
 *          RTL Parameters Applied:
 *          - RTL_ALT (return altitude)
 *          - RTL_CLIMB_MIN (minimum climb before return)
 *          - RTL_SPEED (horizontal return speed)
 *          - RTL_CONE_SLOPE (approach cone)
 *          - All standard RTL parameters used
 * 
 *          Home Position:
 *          - RTL returns to home position
 *          - Home set at arming or via DO_SET_HOME command
 *          - Mission can change home location
 *          - RTL uses current home position
 * 
 *          Landing Behavior:
 *          - RTL descends and lands at home
 *          - Uses standard landing controller
 *          - Pilot can adjust landing with stick inputs
 *          - Motors disarm after landing (if mission complete)
 * 
 *          Use Cases:
 *          - Mission step to return home
 *          - Safety procedure in mission
 *          - Return before next mission segment
 *          - Planned battery change point
 *          - Weather check return
 * 
 * @note Delegates to RTL mode controller (copter.mode_rtl)
 * @note Bypasses normal RTL safety checks (assumes mission validation)
 * @note Sets Auto submode to RTL
 * @note RTL init with ignore_checks=true always succeeds
 * @note Called by do_RTL() when NAV_RTL command executed
 * 
 * @warning Should never fail (logs internal error if RTL init fails)
 * @warning Internal error indicates programming logic bug
 * 
 * @see rtl_run() - Executes RTL navigation by delegating to RTL mode
 * @see verify_RTL() - Checks if RTL navigation complete
 * @see do_RTL() - Mission command handler that calls this function
 * @see ModeRTL::init() - RTL mode initialization
 * @see ModeRTL::run() - RTL mode execution loop
 */
/**
 * @brief Initialize Return-to-Launch (RTL) behavior within AUTO mode
 * 
 * @details This function delegates RTL initialization to the dedicated RTL flight mode
 *          while keeping the vehicle in AUTO mode. This allows mission commands like
 *          MAV_CMD_NAV_RETURN_TO_LAUNCH to be executed within an AUTO mission.
 *          
 *          The function calls ModeRTL::init() with ignore_checks=true to bypass
 *          normal RTL entry requirements, then switches the AUTO submode to RTL.
 *          
 *          RTL behavior includes:
 *          - Climb to RTL altitude if below it
 *          - Navigate to home position
 *          - Descend and land at home
 * 
 * @note Passes ignore_checks=true to RTL mode to ensure initialization always succeeds
 * @note If RTL init fails (should never happen with ignore_checks=true), triggers internal error
 * @note The vehicle remains in AUTO mode; only the submode changes to RTL
 * 
 * @see do_RTL() for the mission command handler
 * @see rtl_run() for the corresponding execution function
 * @see ModeRTL::init() for the RTL mode initialization details
 */
void ModeAuto::rtl_start()
{
    // call regular rtl flight mode initialisation and ask it to ignore checks
    if (copter.mode_rtl.init(true)) {
        set_submode(SubMode::RTL);
    } else {
        // this should never happen because RTL never fails init if argument is true
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
    }
}

/**
 * @brief Initialize takeoff navigation in Auto flight mode
 * 
 * @details This function configures the takeoff controller to climb to a specified altitude,
 *          handling terrain-relative and absolute altitude targets. It's called by do_takeoff()
 *          when a NAV_TAKEOFF or NAV_VTOL_TAKEOFF mission command is encountered. The function
 *          sets up altitude targets, initializes controllers, and transitions to the TAKEOFF
 *          submode for execution.
 * 
 *          NAV_TAKEOFF Command:
 *          - MAV_CMD_NAV_TAKEOFF (ID 22) for standard takeoff
 *          - MAV_CMD_NAV_VTOL_TAKEOFF for VTOL aircraft
 *          - Specifies target altitude and altitude frame
 *          - Mission command executed during flight or from ground
 *          - Typically first command after arming
 * 
 *          Location Initialization Check:
 *          - Verifies copter.current_loc.initialised()
 *          - Current location requires valid AHRS/EKF origin
 *          - Origin set during initialization before mission starts
 *          - If not initialized, logs INTERNAL_ERROR (should never happen)
 *          - Returns early if location invalid
 * 
 *          Why Location Must Be Initialized:
 *          - Need current altitude for relative calculations
 *          - Terrain-relative takeoff requires position
 *          - Altitude frame conversions require origin
 *          - Mission commands wait for EKF origin
 *          - Safety check for unexpected conditions
 * 
 *          Altitude Frame Handling:
 *          - Destination can specify multiple altitude frames
 *          - ABOVE_TERRAIN: Altitude relative to terrain below
 *          - ABOVE_ORIGIN: Altitude relative to EKF origin
 *          - ABOVE_HOME: Altitude relative to home position
 *          - Function converts to appropriate frame
 * 
 *          Terrain-Relative Takeoff Path:
 *          - If dest_loc.get_alt_frame() == ABOVE_TERRAIN
 *          - AND wp_nav->get_terrain_offset_cm() succeeds
 *          - Uses terrain database or rangefinder
 *          - Subtracts terrain offset from current altitude
 *          - Sets alt_target_cm to dest_loc.alt (terrain-relative)
 *          - Sets alt_target_terrain = true flag
 * 
 *          Terrain Offset Explained:
 *          - Terrain offset = terrain altitude above EKF origin
 *          - EKF origin typically set at arming location
 *          - Terrain can be above or below origin
 *          - Offset allows altitude frame conversions
 *          - Retrieved from terrain database or rangefinder
 * 
 *          Terrain Database:
 *          - Global terrain elevation data
 *          - 100m resolution tiles downloaded via MAVLink
 *          - Provides ground elevation for any lat/lon
 *          - Used when rangefinder not available
 *          - May not be available in all regions
 * 
 *          Absolute Altitude Path (Non-Terrain):
 *          - If terrain not available or not requested
 *          - Creates temp Location with current lat/lon
 *          - Calls dest.get_alt_cm(ABOVE_ORIGIN, alt_target_cm)
 *          - Converts destination altitude to EKF origin frame
 *          - Falls back to relative if conversion fails
 * 
 *          Altitude Conversion Failure:
 *          - get_alt_cm() can fail if terrain data missing
 *          - Occurs if takeoff alt specified as ABOVE_TERRAIN
 *          - No terrain database or rangefinder available
 *          - Logs MISSING_TERRAIN_DATA error
 *          - Falls back: alt_target_cm = current_alt_cm + dest.alt
 * 
 *          Fallback Calculation:
 *          - When terrain conversion fails
 *          - Interprets dest.alt as relative climb
 *          - Adds to current altitude
 *          - Simple relative takeoff
 *          - Ensures takeoff proceeds despite missing data
 * 
 *          Minimum Altitude Sanity Check:
 *          - alt_target_min_cm = current_alt_cm + (land_complete ? 100 : 0)
 *          - If on ground (land_complete), require at least 1m climb
 *          - If already airborne, use current altitude as minimum
 *          - alt_target_cm = MAX(alt_target_cm, alt_target_min_cm)
 *          - Prevents descending during takeoff
 * 
 *          Why 100cm Minimum:
 *          - Ensures vehicle actually takes off from ground
 *          - 1 meter minimum climb
 *          - Prevents zero or negative altitude targets
 *          - Safety margin above ground
 *          - Detects and lands if target too low
 * 
 *          Already Airborne Case:
 *          - If not land_complete, vehicle already flying
 *          - No 100cm minimum added
 *          - Uses current altitude as minimum
 *          - Can climb higher or maintain altitude
 *          - Won't descend during "takeoff"
 * 
 *          Yaw Initialization:
 *          - auto_yaw.set_mode(AutoYaw::Mode::HOLD)
 *          - Holds current heading during takeoff
 *          - Vehicle doesn't rotate while climbing
 *          - Pilot yaw input still effective (if allowed)
 *          - Different from waypoint yaw (which points at target)
 * 
 *          Vertical Controller Initialization:
 *          - pos_control->init_U_controller()
 *          - Resets vertical position controller integrator
 *          - "U" = Up axis in NEU coordinate frame
 *          - Clears accumulated integral term
 *          - Prevents wind-up from previous state
 * 
 *          Why Clear Integrator:
 *          - Previous flight state may have I term built up
 *          - Landing has negative I term (fighting gravity)
 *          - Takeoff needs fresh start
 *          - Prevents overshoot or oscillation
 *          - Clean controller state for climb
 * 
 *          Takeoff Controller Start:
 *          - auto_takeoff.start(alt_target_cm, alt_target_terrain)
 *          - Dedicated takeoff helper object
 *          - Manages climb profile and completion detection
 *          - Handles WP_NAVALT_MIN parameter
 *          - Tracks takeoff progress
 * 
 *          WP_NAVALT_MIN Parameter:
 *          - Minimum altitude for waypoint navigation
 *          - During takeoff, enforces minimum altitude before next waypoint
 *          - Ensures vehicle climbs sufficiently before horizontal movement
 *          - auto_takeoff tracks this requirement
 *          - Prevents premature waypoint transition
 * 
 *          Submode Transition:
 *          - set_submode(SubMode::TAKEOFF)
 *          - Auto::run() will call takeoff_run()
 *          - takeoff_run() executes auto_takeoff.run()
 *          - Runs at 100Hz+ until takeoff complete
 * 
 *          takeoff_run() Execution:
 *          - Called by Auto::run() when _mode == SubMode::TAKEOFF
 *          - Delegates to auto_takeoff.run()
 *          - Climbs at configured rate
 *          - Monitors altitude progress
 *          - Completes when target altitude reached
 * 
 *          verify_takeoff() Completion:
 *          - Mission system calls verify_takeoff()
 *          - Checks auto_takeoff.complete flag
 *          - Returns true when altitude target reached
 *          - Mission advances to next command
 *          - May retract landing gear on completion
 * 
 *          Altitude Target Units:
 *          - All altitudes in centimeters
 *          - current_alt_cm: cm above EKF origin or terrain
 *          - alt_target_cm: cm in appropriate frame
 *          - Internal calculations all in cm
 *          - Converted from Location meters (x100)
 * 
 *          NEU Coordinate Frame:
 *          - N = North, E = East, U = Up
 *          - Z axis is Up (altitude increase)
 *          - pos_control uses NEU internally
 *          - get_pos_estimate_NEU_cm().z is altitude
 *          - Origin at EKF origin (typically home)
 * 
 *          Error Handling Summary:
 *          - Uninitialized location: INTERNAL_ERROR, return early
 *          - Missing terrain data: Log error, use fallback altitude
 *          - Target too low: Enforce minimum altitude
 *          - All errors handled gracefully
 * 
 *          Takeoff from Ground vs Airborne:
 *          - From ground: Enforces 100cm minimum climb
 *          - Already airborne: Uses current altitude as min
 *          - Handles both scenarios safely
 *          - Mission can include mid-flight "takeoff" commands
 * 
 *          Use Cases:
 *          - Initial takeoff at mission start
 *          - Takeoff after automatic landing
 *          - Mid-mission altitude gain
 *          - Terrain-following takeoff in hilly areas
 * 
 * @param[in] dest_loc Takeoff destination location with target altitude and frame
 * 
 * @note Handles both terrain-relative and absolute altitude targets
 * @note Enforces minimum 1m climb if on ground
 * @note Resets vertical position controller integrator
 * @note Holds current heading during takeoff
 * @note Falls back to relative altitude if terrain data missing
 * @note Target altitude in cm (converted from Location meters)
 * 
 * @warning Logs INTERNAL_ERROR if current location not initialized
 * @warning Logs TERRAIN error if terrain data missing for ABOVE_TERRAIN target
 * @warning Minimum altitude enforced to prevent descending during takeoff
 * 
 * @see takeoff_run() - Executes takeoff climb at 100Hz+
 * @see verify_takeoff() - Checks if takeoff complete
 * @see do_takeoff() - Mission command handler that calls this function
 * @see auto_takeoff.start() - Initializes takeoff controller
 * @see auto_takeoff.run() - Executes takeoff control
 */
/**
 * @brief Initialize autonomous takeoff sequence
 * 
 * @details This function sets up the waypoint controller to execute a vertical takeoff
 *          to a specified altitude. The function handles both absolute and terrain-relative
 *          altitude targets, with appropriate fallbacks for missing terrain data.
 *          
 *          Takeoff sequence initialization:
 *          1. Verify current location is initialized (should always be true in AUTO)
 *          2. Calculate target altitude in appropriate frame (above-origin or above-terrain)
 *          3. Apply terrain offset correction if using terrain-relative altitude
 *          4. Enforce minimum altitude safety margin (100cm above current if landed)
 *          5. Initialize yaw to hold current heading during vertical ascent
 *          6. Reset integral terms in position controller for clean start
 *          7. Configure auto_takeoff helper with target altitude
 *          
 *          Altitude frame handling:
 *          - ABOVE_TERRAIN: Uses terrain database or rangefinder for ground reference
 *          - Other frames: Converted to altitude above EKF origin
 *          
 *          Safety features:
 *          - Minimum 100cm climb if starting from landed state
 *          - Fallback to alt-above-current if terrain data unavailable
 *          - Error logging for missing terrain data
 * 
 * @param[in] dest_loc Target location for takeoff (altitude used, lat/lon typically ignored)
 * 
 * @note This is called at mission command rate (approximately 10Hz)
 * @note Resets position controller integral terms to avoid accumulated errors
 * @note Takeoff completion altitude is used for WP_NAVALT_MIN logic
 * 
 * @warning Requires valid current_loc; logs internal error if unavailable
 * 
 * @see takeoff_run() for the corresponding execution function
 * @see do_takeoff() for the mission command handler
 * @see AC_AutoTakeoff for the takeoff implementation details
 */
void ModeAuto::takeoff_start(const Location& dest_loc)
{
    if (!copter.current_loc.initialised()) {
        // this should never happen because mission commands are not executed until
        // the AHRS/EKF origin is set by which time current_loc should also have been set
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        return;
    }

    // calculate current and target altitudes
    // by default current_alt_cm and alt_target_cm are alt-above-EKF-origin
    int32_t alt_target_cm;
    bool alt_target_terrain = false;
    float current_alt_cm = pos_control->get_pos_estimate_NEU_cm().z;
    float terrain_offset;   // terrain's altitude in cm above the ekf origin
    if ((dest_loc.get_alt_frame() == Location::AltFrame::ABOVE_TERRAIN) && wp_nav->get_terrain_offset_cm(terrain_offset)) {
        // subtract terrain offset to convert vehicle's alt-above-ekf-origin to alt-above-terrain
        current_alt_cm -= terrain_offset;

        // specify alt_target_cm as alt-above-terrain
        alt_target_cm = dest_loc.alt;
        alt_target_terrain = true;
    } else {
        // set horizontal target
        Location dest(dest_loc);
        dest.lat = copter.current_loc.lat;
        dest.lng = copter.current_loc.lng;

        // get altitude target above EKF origin
        if (!dest.get_alt_cm(Location::AltFrame::ABOVE_ORIGIN, alt_target_cm)) {
            // this failure could only happen if take-off alt was specified as an alt-above terrain and we have no terrain data
            LOGGER_WRITE_ERROR(LogErrorSubsystem::TERRAIN, LogErrorCode::MISSING_TERRAIN_DATA);
            // fall back to altitude above current altitude
            alt_target_cm = current_alt_cm + dest.alt;
        }
    }

    // sanity check target
    int32_t alt_target_min_cm = current_alt_cm + (copter.ap.land_complete ? 100 : 0);
    alt_target_cm = MAX(alt_target_cm, alt_target_min_cm);

    // initialise yaw
    auto_yaw.set_mode(AutoYaw::Mode::HOLD);

    // clear i term when we're taking off
    pos_control->init_U_controller();

    // initialise alt for WP_NAVALT_MIN and set completion alt
    auto_takeoff.start(alt_target_cm, alt_target_terrain);

    // set submode
    set_submode(SubMode::TAKEOFF);
}

/**
 * @brief Initialize waypoint navigation to fly to a specified destination location
 * 
 * @details This function configures the waypoint navigation controller (AC_WPNav) to fly to a target
 *          location, handling the transition from previous submodes and applying speed overrides from
 *          DO_CHANGE_SPEED commands. It's called by do_nav_wp() and do_loiter_unlimited() to start
 *          horizontal and vertical navigation to waypoint destinations.
 * 
 *          Called By:
 *          - do_nav_wp(): NAV_WAYPOINT mission command
 *          - do_loiter_unlimited(): NAV_LOITER_UNLIM command
 *          - do_land(): NAV_LAND with lat/lon specified
 *          - do_payload_place(): NAV_PAYLOAD_PLACE with location
 *          - Used for most autonomous position navigation
 * 
 *          Waypoint Navigator Initialization:
 *          - Checks if wp_nav->is_active()
 *          - If not active, needs full initialization
 *          - Occurs when transitioning from non-WP submode
 *          - Or when starting Auto mode fresh
 *          - Active state persists between consecutive waypoints
 * 
 *          Why Check is_active():
 *          - Consecutive waypoints don't need reinitialization
 *          - Preserves navigation state between waypoints
 *          - Only reinit when changing from different submode
 *          - Improves path smoothness
 *          - Reduces computational overhead
 * 
 *          Takeoff Transition Handling:
 *          - Special case: if _mode == SubMode::TAKEOFF
 *          - Gets takeoff completion position
 *          - auto_takeoff.get_completion_pos() provides expected position
 *          - Uses as stopping_point for smooth transition
 *          - Prevents jump in navigation origin
 * 
 *          Takeoff Completion Position:
 *          - Predicted position when takeoff completes
 *          - Accounts for vehicle drift during climb
 *          - Provides smooth transition to horizontal navigation
 *          - If not available, stopping_point defaults to zero
 *          - Zero means current position used
 * 
 *          Speed Override Application:
 *          - desired_speed_override set by DO_CHANGE_SPEED command
 *          - Persists across multiple waypoints
 *          - Override values in m/s, converted to cm/s (*100)
 *          - Applied to horizontal, up, and down speeds
 *          - Zero or negative means use defaults
 * 
 *          Horizontal Speed Override:
 *          - is_positive(desired_speed_override.xy) checks if override set
 *          - If set, converts m/s to cm/s and passes to wp_and_spline_init_cm
 *          - If not set, passes 0 (controller uses default WP_NAV_SPEED)
 *          - Applied to NE horizontal movement
 * 
 *          wp_and_spline_init_cm():
 *          - Initializes both waypoint and spline navigation
 *          - Sets origin position (from stopping_point or current pos)
 *          - Configures speed limits
 *          - Resets path following state
 *          - Called once per navigation sequence
 * 
 *          Vertical Speed Overrides:
 *          - Separate overrides for up and down speeds
 *          - Up speed: climb rate (cm/s positive up)
 *          - Down speed: descent rate (cm/s positive down)
 *          - set_speed_up_cms() and set_speed_down_cms()
 *          - Override defaults from WP_NAV_SPEED_UP/DN parameters
 * 
 *          Why Separate Up/Down:
 *          - Descent often slower for safety
 *          - Climb limited by motor power
 *          - Asymmetric speed profiles common
 *          - Mission may need different rates
 *          - DO_CHANGE_SPEED supports separate rates
 * 
 *          Destination Setting:
 *          - wp_nav->set_wp_destination_loc(dest_loc)
 *          - Configures target waypoint location
 *          - Returns false if terrain data missing (for terrain alt)
 *          - Starts path planning and trajectory generation
 *          - Can fail with terrain-relative waypoints
 * 
 *          Terrain Data Failure:
 *          - If dest_loc altitude frame is ABOVE_TERRAIN
 *          - And terrain database/rangefinder unavailable
 *          - set_wp_destination_loc() returns false
 *          - Function returns false to caller
 *          - Caller triggers terrain failsafe
 * 
 *          Yaw Mode Configuration:
 *          - Determines how vehicle orients during waypoint navigation
 *          - Multiple modes: look at waypoint, ROI, fixed, hold
 *          - Complex logic preserves ROI and special FIXED modes
 *          - set_mode_to_default(false) typically points at waypoint
 * 
 *          ROI Mode Preservation:
 *          - If auto_yaw.mode() == AutoYaw::Mode::ROI
 *          - DO_SET_ROI command previously set ROI target
 *          - Vehicle continues pointing at ROI
 *          - Don't reset to default waypoint-pointing
 *          - Allows camera to track ROI while moving
 * 
 *          Fixed Yaw with WP_YAW_BEHAVIOR_NONE:
 *          - If auto_yaw.mode() == AutoYaw::Mode::FIXED
 *          - AND copter.g.wp_yaw_behavior == WP_YAW_BEHAVIOR_NONE
 *          - Parameter says "don't change yaw for waypoints"
 *          - Preserve fixed yaw heading
 *          - Vehicle flies sideways/backwards if needed
 * 
 *          WP_YAW_BEHAVIOR Parameter:
 *          - 0 = Never change yaw (WP_YAW_BEHAVIOR_NONE)
 *          - 1 = Face next waypoint (default)
 *          - 2 = Face next waypoint except RTL
 *          - 3 = Face along GPS course
 *          - Controls default yaw behavior
 * 
 *          set_mode_to_default(false) Behavior:
 *          - false parameter means "don't reset if already set"
 *          - Typically sets AutoYaw::Mode::LOOK_AT_NEXT_WP
 *          - Vehicle rotates to point at destination
 *          - Smooth yaw transition during flight
 *          - Depends on WP_YAW_BEHAVIOR parameter
 * 
 *          Submode Transition:
 *          - set_submode(SubMode::WP)
 *          - Auto::run() will call wp_run()
 *          - wp_run() executes waypoint following
 *          - Runs at 100Hz+ until waypoint reached
 * 
 *          wp_run() Execution:
 *          - Called by Auto::run() when _mode == SubMode::WP
 *          - Calls wp_nav->update_wpnav()
 *          - Updates position controller targets
 *          - Runs attitude controller
 *          - Continues until waypoint reached
 * 
 *          Waypoint Following:
 *          - wp_nav calculates path from current position to destination
 *          - Generates velocity and position targets
 *          - Respects speed and acceleration limits
 *          - Handles altitude changes during horizontal movement
 *          - Coordinates all 3 axes smoothly
 * 
 *          Corner Cutting:
 *          - If next waypoint set via set_next_wp()
 *          - wp_nav blends waypoints for smooth path
 *          - Vehicle doesn't stop at each waypoint
 *          - Cuts corner within acceptable radius
 *          - Reduces mission time
 * 
 *          Return Value:
 *          - true: Waypoint navigation successfully started
 *          - false: Failed to set destination (terrain data missing)
 *          - Caller checks return and handles failure
 *          - Typically triggers terrain failsafe on false
 * 
 *          Success Path:
 *          - wp_nav initialized (if needed)
 *          - Speed overrides applied
 *          - Destination set successfully
 *          - Yaw mode configured
 *          - Submode set to WP
 *          - Returns true
 * 
 *          Consecutive Waypoint Optimization:
 *          - If wp_nav already active, skips initialization
 *          - Only sets new destination
 *          - Preserves navigation state
 *          - Faster execution
 *          - Smoother path
 * 
 *          Speed Override Persistence:
 *          - DO_CHANGE_SPEED sets desired_speed_override
 *          - Remains set until another DO_CHANGE_SPEED
 *          - Applied to all subsequent waypoints
 *          - Checked every wp_start() call
 *          - Mission can have speed zones
 * 
 *          Use Cases:
 *          - NAV_WAYPOINT mission command
 *          - NAV_LOITER_UNLIM (fly to loiter position)
 *          - NAV_LAND (fly to landing position first)
 *          - NAV_PAYLOAD_PLACE (fly to place position)
 *          - Any mission command requiring position navigation
 * 
 * @param[in] dest_loc Target waypoint location (lat, lon, alt with frame)
 * 
 * @return true if waypoint navigation successfully started, false if terrain data missing
 * 
 * @note Handles smooth transition from takeoff submode
 * @note Applies speed overrides from DO_CHANGE_SPEED commands
 * @note Preserves ROI yaw mode if active
 * @note Respects WP_YAW_BEHAVIOR parameter
 * @note Only reinitializes wp_nav when not already active
 * @note Speed values in m/s, converted to cm/s internally
 * 
 * @warning Returns false if terrain data unavailable for terrain-relative waypoint
 * @warning Caller must handle false return (typically terrain failsafe)
 * 
 * @see wp_run() - Executes waypoint navigation at 100Hz+
 * @see do_nav_wp() - NAV_WAYPOINT command handler
 * @see do_change_speed() - Sets speed overrides
 * @see set_next_wp() - Configures next waypoint for corner cutting
 * @see AC_WPNav::set_wp_destination_loc() - Waypoint controller destination setting
 */
/**
 * @brief Initialize waypoint navigation to a target location
 * 
 * @details This function configures the waypoint controller (AC_WPNav) to navigate to
 *          a specified destination location. It handles both initial waypoint setup and
 *          transitions from other submodes (particularly takeoff).
 *          
 *          Initialization sequence:
 *          1. Check if waypoint controller is already active
 *          2. If transitioning from takeoff, use takeoff completion position as origin
 *          3. Initialize waypoint and spline controller with appropriate speed overrides
 *          4. Apply any speed overrides from DO_CHANGE_SPEED commands
 *          5. Set waypoint destination in AC_WPNav
 *          6. Configure yaw behavior (ROI, fixed, or default)
 *          
 *          Speed override handling:
 *          - Horizontal (xy): Applied if positive value from desired_speed_override
 *          - Up: Applied if positive value, overrides default climb speed
 *          - Down: Applied if positive value, overrides default descent speed
 *          
 *          Yaw behavior:
 *          - Preserves ROI (Region of Interest) if active
 *          - Preserves FIXED yaw with WP_YAW_BEHAVIOR_NONE setting
 *          - Otherwise reverts to default yaw behavior (typically points at waypoint)
 * 
 * @param[in] dest_loc Target waypoint location (lat, lon, alt in specified frame)
 * 
 * @return true if waypoint destination set successfully
 * @return false if destination invalid (typically due to missing terrain data)
 * 
 * @note Called at mission command rate (approximately 10Hz)
 * @note Speed overrides persist until explicitly changed or mission restart
 * 
 * @see wp_run() for the corresponding execution function
 * @see do_nav_wp() for the mission command handler
 * @see AC_WPNav::set_wp_destination_loc() for waypoint setting details
 */
bool ModeAuto::wp_start(const Location& dest_loc)
{
    // init wpnav and set origin if transitioning from takeoff
    if (!wp_nav->is_active()) {
        Vector3f stopping_point;
        if (_mode == SubMode::TAKEOFF) {
            Vector3p takeoff_complete_pos;
            if (auto_takeoff.get_completion_pos(takeoff_complete_pos)) {
                stopping_point = takeoff_complete_pos.tofloat();
            }
        }
        float des_speed_xy_cm = is_positive(desired_speed_override.xy) ? (desired_speed_override.xy * 100) : 0;
        wp_nav->wp_and_spline_init_cm(des_speed_xy_cm, stopping_point);

        // override speeds up and down if necessary
        if (is_positive(desired_speed_override.up)) {
            wp_nav->set_speed_up_cms(desired_speed_override.up * 100.0);
        }
        if (is_positive(desired_speed_override.down)) {
            wp_nav->set_speed_down_cms(desired_speed_override.down * 100.0);
        }
    }

    if (!wp_nav->set_wp_destination_loc(dest_loc)) {
        return false;
    }

    // initialise yaw
    // To-Do: reset the yaw only when the previous navigation command is not a WP.  this would allow removing the special check for ROI
    if (auto_yaw.mode() != AutoYaw::Mode::ROI && !(auto_yaw.mode() == AutoYaw::Mode::FIXED && copter.g.wp_yaw_behavior == WP_YAW_BEHAVIOR_NONE)) {
        auto_yaw.set_mode_to_default(false);
    }

    // set submode
    set_submode(SubMode::WP);

    return true;
}

/**
 * @brief Initialize autonomous landing sequence in Auto flight mode
 * 
 * @details This function configures position controllers for descent to landing, handling both
 *          direct landing at current position and landing after flying to a specified location.
 *          It's called by do_land() when NAV_LAND or NAV_VTOL_LAND mission command is encountered,
 *          setting up horizontal position hold, vertical descent control, landing gear deployment,
 *          and precision landing initialization.
 * 
 *          Called By:
 *          - do_land(): After reaching land location (if specified) or immediately
 *          - verify_land(): When transitioning from FlyToLocation to Descending state
 *          - NAV_LAND (ID 21) or NAV_VTOL_LAND mission commands
 * 
 *          Landing Sequence Overview:
 *          1. Fly to landing location (if lat/lon specified in command)
 *          2. land_start() called to initialize descent
 *          3. land_run() executes descent at configured rate
 *          4. Descent slows near ground (LAND_SPEED_HIGH -> LAND_SPEED)
 *          5. Motor disarm when ground detected
 * 
 *          Horizontal Position Control Setup:
 *          - Uses pos_control for NE (North-East) horizontal control
 *          - Holds position during descent (no drift)
 *          - Speed limits set from wp_nav defaults
 *          - Correction speeds allow small position adjustments
 * 
 *          Horizontal Speed Limits:
 *          - set_max_speed_accel_NE_cm() sets maximum horizontal speed/accel
 *          - Uses wp_nav->get_default_speed_NE_cms() (typically WP_NAV_SPEED param)
 *          - Uses wp_nav->get_wp_acceleration_cmss() (typically WP_NAV_ACCEL param)
 *          - Speed in cm/s, acceleration in cm/s²
 *          - Limits drift correction during descent
 * 
 *          Correction Speed Limits:
 *          - set_correction_speed_accel_NE_cm() separate from max limits
 *          - Used when correcting position errors
 *          - Typically same as max limits for landing
 *          - Allows position hold during wind/drift
 *          - Prevents excessive horizontal movement while descending
 * 
 *          NE Controller Initialization:
 *          - Checks if pos_control->is_active_NE()
 *          - If not active, calls init_NE_controller()
 *          - Resets controller state (integrators, targets)
 *          - Necessary when transitioning from non-position-control mode
 *          - Skipped if already active (consecutive land commands)
 * 
 *          Why Check is_active_NE():
 *          - Controller may already be running from waypoint navigation
 *          - No need to reset if already position-controlling
 *          - Preserves smooth transition
 *          - Only init when necessary
 *          - Reduces control discontinuities
 * 
 *          Vertical Speed Limits:
 *          - set_max_speed_accel_U_cm() for vertical axis (U = Up in NEU frame)
 *          - Down speed: wp_nav->get_default_speed_down_cms() (WP_NAV_SPEED_DN param)
 *          - Up speed: wp_nav->get_default_speed_up_cms() (WP_NAV_SPEED_UP param)
 *          - Vertical accel: wp_nav->get_accel_U_cmss()
 *          - Used for initial descent phase
 * 
 *          Vertical Correction Limits:
 *          - set_correction_speed_accel_U_cmss() for altitude corrections
 *          - Same values as max limits for landing
 *          - Applied when correcting altitude errors
 *          - Allows smooth descent profile
 * 
 *          U Controller Initialization:
 *          - Checks if pos_control->is_active_U()
 *          - If not active, calls init_U_controller()
 *          - Resets vertical position controller state
 *          - Clears integrator from previous modes
 *          - Necessary for clean landing start
 * 
 *          Why Init U Controller:
 *          - Previous mode may have different altitude target
 *          - Landing needs fresh controller state
 *          - Prevents integrator wind-up from affecting landing
 *          - Ensures smooth descent start
 *          - Clean slate for landing control
 * 
 *          Yaw Hold Mode:
 *          - auto_yaw.set_mode(AutoYaw::Mode::HOLD)
 *          - Maintains current heading during descent
 *          - Vehicle doesn't rotate while landing
 *          - Pilot yaw input typically allowed (configurable)
 *          - Stable platform for precision landing sensors
 * 
 *          Why Hold Yaw During Landing:
 *          - Consistent orientation for landing cameras
 *          - Reduces disturbance during descent
 *          - Easier for pilot to monitor
 *          - Precision landing sensors need stable heading
 *          - Landing spot orientation maintained
 * 
 *          Landing Gear Deployment:
 *          - Conditional compilation: #if AP_LANDINGGEAR_ENABLED
 *          - copter.landinggear.deploy_for_landing()
 *          - Extends landing gear (if installed)
 *          - Servo/motor command sent
 *          - Typically extends in 1-3 seconds
 * 
 *          Landing Gear Types:
 *          - Servo-actuated retractable gear
 *          - Electric motor-driven gear
 *          - Pneumatic/hydraulic systems
 *          - Parameter LGR_DEPLOY_ALT can trigger earlier
 *          - Deployed before touchdown
 * 
 *          Land Reposition Flag Reset:
 *          - copter.ap.land_repo_active = false
 *          - Tracks if pilot used stick input during landing
 *          - "Reposition" = pilot adjusting landing position
 *          - Reset at start of each landing
 *          - Set true if pilot inputs detected during descent
 * 
 *          Why Track Pilot Input:
 *          - Affects landing detector sensitivity
 *          - May delay motor disarm
 *          - Safety feature for pilot intervention
 *          - Logged for post-flight analysis
 *          - Used in verify_land() logic
 * 
 *          Precision Landing Flag Reset:
 *          - copter.ap.prec_land_active = false
 *          - Indicates if precision landing is actively guiding
 *          - Will be set true if precision landing takes over
 *          - Reset at start of each landing
 *          - Updated by land_run_normal_or_precland()
 * 
 *          Precision Landing System:
 *          - Uses IR-LOCK, optical flow, or vision system
 *          - Detects landing target (e.g., IR beacon)
 *          - Adjusts horizontal position during descent
 *          - Achieves cm-level landing accuracy
 *          - Requires AP_PRECLAND_ENABLED and hardware
 * 
 *          Submode Transition:
 *          - set_submode(SubMode::LAND)
 *          - Auto::run() will call land_run()
 *          - land_run() executes descent control
 *          - Runs at 100Hz+ until landing complete
 * 
 *          land_run() Execution:
 *          - Called by Auto::run() when _mode == SubMode::LAND
 *          - Calls land_run_normal_or_precland()
 *          - Manages descent rate (faster high, slower low)
 *          - Monitors for ground contact
 *          - Disarms when landing detected
 * 
 *          Landing Descent Profile:
 *          - Initial descent at LAND_SPEED_HIGH (default 500 cm/s)
 *          - Slows to LAND_SPEED below LAND_ALT_LOW (default 1000 cm)
 *          - Final approach at LAND_SPEED (default 50 cm/s)
 *          - Gentle touchdown
 *          - Configurable via parameters
 * 
 *          Landing Detection:
 *          - Monitors throttle output
 *          - Checks for low climb rate
 *          - Verifies consistent ground contact
 *          - Sets copter.ap.land_complete flag
 *          - Triggers motor disarm
 * 
 *          verify_land() Completion:
 *          - Mission system calls verify_land()
 *          - Checks copter.ap.land_complete
 *          - Checks motors->get_spool_state() == GROUND_IDLE
 *          - Returns true when safely on ground
 *          - Mission advances or disarms
 * 
 *          Position Hold During Descent:
 *          - NE controller maintains horizontal position
 *          - Corrects for wind drift
 *          - Precision landing overrides if active
 *          - Keeps vehicle over intended landing spot
 *          - Uses GPS and/or optical sensors
 * 
 *          Safety Features:
 *          - Controlled descent rate
 *          - Position hold prevents drift
 *          - Pilot can intervene with sticks
 *          - Landing gear deployed early
 *          - Gentle final approach
 * 
 *          Use Cases:
 *          - Mission commanded landing
 *          - Battery failsafe landing
 *          - Auto RTL landing
 *          - Precision landing on moving platform (e.g., ship)
 *          - Autonomous mission completion
 * 
 * @note Sets up both horizontal position hold and vertical descent control
 * @note Deploys landing gear if feature enabled
 * @note Holds current heading during descent
 * @note Resets pilot input and precision landing flags
 * @note Uses waypoint navigation speed limits
 * @note Controller initialization only if not already active
 * 
 * @warning Must have valid position estimate (GPS or optical flow)
 * @warning Precision landing requires additional hardware and configuration
 * 
 * @see land_run() - Executes landing descent control
 * @see land_run_normal_or_precland() - Handles normal or precision landing
 * @see do_land() - Mission command handler that calls this function
 * @see verify_land() - Checks if landing complete
 */
/**
 * @brief Initialize autonomous landing sequence
 * 
 * @details This function configures the position controller for a controlled descent to
 *          landing. It sets up both horizontal (NE) and vertical (U) position control
 *          with appropriate speed and acceleration limits for safe landing.
 *          
 *          Landing controller setup:
 *          1. Configure horizontal control with waypoint navigation speeds
 *          2. Initialize NE (North-East) controller if not already active
 *          3. Configure vertical control with descent speeds
 *          4. Initialize U (Up) controller if not already active
 *          5. Set yaw to hold current heading during descent
 *          6. Deploy landing gear if equipped (AP_LANDINGGEAR_ENABLED)
 *          7. Reset pilot repositioning flag
 *          8. Clear precision landing flag (set later if prec land activates)
 *          
 *          Speed configuration:
 *          - Horizontal: Uses default waypoint navigation speed and acceleration
 *          - Vertical: Uses configured descent speed (typically slower than normal flight)
 *          - Correction speeds match max speeds for consistent behavior
 *          
 *          Landing gear deployment:
 *          - Automatically deploys landing gear if system is equipped
 *          - Triggered via AP_LandingGear::deploy_for_landing()
 *          
 *          State flags initialized:
 *          - land_repo_active: Tracks if pilot has attempted to reposition during landing
 *          - prec_land_active: Indicates precision landing system engagement
 * 
 * @note This is the landing initialization function; actual descent occurs in land_run()
 * @note Called at mission command rate (approximately 10Hz)
 * @note Yaw is held constant unless pilot input overrides (if enabled)
 * 
 * @see land_run() for the corresponding execution function
 * @see do_land() for the mission command handler
 * @see land_run_normal_or_precland() for precision landing integration
 */
void ModeAuto::land_start()
{
    // set horizontal speed and acceleration limits
    pos_control->set_max_speed_accel_NE_cm(wp_nav->get_default_speed_NE_cms(), wp_nav->get_wp_acceleration_cmss());
    pos_control->set_correction_speed_accel_NE_cm(wp_nav->get_default_speed_NE_cms(), wp_nav->get_wp_acceleration_cmss());

    // initialise the vertical position controller
    if (!pos_control->is_active_NE()) {
        pos_control->init_NE_controller();
    }

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_U_cm(wp_nav->get_default_speed_down_cms(), wp_nav->get_default_speed_up_cms(), wp_nav->get_accel_U_cmss());
    pos_control->set_correction_speed_accel_U_cmss(wp_nav->get_default_speed_down_cms(), wp_nav->get_default_speed_up_cms(), wp_nav->get_accel_U_cmss());

    // initialise the vertical position controller
    if (!pos_control->is_active_U()) {
        pos_control->init_U_controller();
    }

    // initialise yaw
    auto_yaw.set_mode(AutoYaw::Mode::HOLD);

#if AP_LANDINGGEAR_ENABLED
    // optionally deploy landing gear
    copter.landinggear.deploy_for_landing();
#endif

    // reset flag indicating if pilot has applied roll or pitch inputs during landing
    copter.ap.land_repo_active = false;

    // this will be set true if prec land is later active
    copter.ap.prec_land_active = false;

    // set submode
    set_submode(SubMode::LAND);
}

/**
 * @brief Initialize waypoint navigation to fly to the edge of a circle before starting to circle
 * 
 * @details This function is called by do_circle() when handling NAV_LOITER_TURNS mission commands.
 *          It determines if the vehicle is close enough to the circle perimeter to start circling
 *          immediately, or if it needs to first fly to the nearest point on the circle. This two-phase
 *          approach ensures smooth entry into circular flight paths.
 * 
 *          Called By:
 *          - do_circle(): NAV_LOITER_TURNS mission command handler
 *          - Sets up circular loiter pattern around specified center point
 * 
 *          Circle Navigation Overview:
 *          1. Vehicle receives NAV_LOITER_TURNS command
 *          2. circle_movetoedge_start() called with center, radius, direction
 *          3. If far from circle edge (>3m), fly to edge first
 *          4. If close to edge (≤3m), start circling immediately
 *          5. circle_start() begins circular flight
 *          6. Complete specified number of turns
 * 
 *          Circle Center Configuration:
 *          - copter.circle_nav->set_center(circle_center)
 *          - Sets the lat/lon/alt center point of circular path
 *          - AC_Circle navigation controller stores this
 *          - All circle calculations reference this center
 *          - Can be absolute or terrain-relative altitude
 * 
 *          Circle Radius Configuration:
 *          - Parameter radius_m in meters from mission command
 *          - Converted to cm: radius_m * 100.0f
 *          - Only set if non-zero (is_zero() check)
 *          - Zero means use previous/default radius
 *          - set_radius_cm() stores in circle controller
 * 
 *          Circle Radius Parameter:
 *          - NAV_LOITER_TURNS command stores radius in high byte of p1
 *          - HIGHBYTE(cmd.p1) extracts radius
 *          - Can be multiplied by 10 for larger radii (special bit set)
 *          - Typical range: 1-250m
 *          - Larger radii useful for survey patterns
 * 
 *          Circle Direction Configuration:
 *          - ccw_turn parameter: true = counter-clockwise, false = clockwise
 *          - From mission command location.loiter_ccw field
 *          - get_rate_degs() retrieves current angular rate setting
 *          - Makes rate negative for CCW (left turn)
 *          - Makes rate positive for CW (right turn)
 * 
 *          Angular Rate Sign Convention:
 *          - Positive rate = clockwise rotation (right turn)
 *          - Negative rate = counter-clockwise rotation (left turn)
 *          - fabsf() gets absolute value
 *          - Apply sign based on ccw_turn parameter
 *          - set_rate_degs() stores signed rate
 * 
 *          Angular Rate Value:
 *          - Typically from CIRCLE_RATE parameter (default 20 deg/s)
 *          - Determines how fast vehicle circles
 *          - Higher rate = tighter turns, higher bank angle
 *          - Lower rate = gentler turns, lower bank angle
 *          - Rate adjusted for radius to maintain speed
 * 
 *          Distance to Edge Calculation:
 *          - get_closest_point_on_circle_NEU_cm() finds nearest point on circle
 *          - Returns circle_edge_neu (NEU coordinates in cm)
 *          - Returns dist_to_edge (straight-line distance in cm)
 *          - NEU frame = North-East-Up relative to EKF origin
 *          - Calculation in horizontal plane only
 * 
 *          3-Meter Threshold Decision:
 *          - if (dist_to_edge > 300.0f) = if more than 3 meters from edge
 *          - 3m threshold allows smooth transition to circling
 *          - Too large: unnecessary waypoint navigation
 *          - Too small: may overshoot and need correction
 *          - 300.0f is in cm units
 * 
 *          Why 3-Meter Threshold:
 *          - Close enough to start circling without overshoot
 *          - Far enough that waypoint navigation useful
 *          - Tested value that works well in practice
 *          - Balances efficiency vs smoothness
 *          - Prevents back-and-forth oscillation
 * 
 *          Fly-to-Edge Path (dist > 3m):
 *          - Create waypoint at nearest circle edge point
 *          - Use waypoint controller to fly there
 *          - Submode set to CIRCLE_MOVE_TO_EDGE
 *          - verify_circle() monitors progress
 *          - When reached, circle_start() called
 * 
 *          Circle Edge Location Construction:
 *          - circle_edge_neu is in NEU cm coordinates
 *          - Convert to Location(lat/lon/alt) object
 *          - Location::AltFrame::ABOVE_ORIGIN for NEU conversion
 *          - Then convert altitude to match circle_center frame
 *          - Ensures altitude consistency
 * 
 *          Altitude Frame Conversion:
 *          - circle_edge initially in ABOVE_ORIGIN (EKF origin)
 *          - Circle center may be terrain-relative or absolute
 *          - set_alt_cm() converts to circle_center's frame
 *          - Maintains altitude consistency
 *          - Prevents altitude changes when circling
 * 
 *          Waypoint Destination Setting:
 *          - wp_nav->set_wp_destination_loc(circle_edge)
 *          - Programs waypoint controller to fly to edge
 *          - Can fail if terrain data missing (terrain alt)
 *          - Failure triggers terrain failsafe
 *          - Success starts navigation to edge
 * 
 *          Terrain Failsafe Trigger:
 *          - If circle altitude is terrain-relative
 *          - And terrain database unavailable
 *          - set_wp_destination_loc() returns false
 *          - copter.failsafe_terrain_on_event() called
 *          - Prevents flying with unknown terrain clearance
 * 
 *          Yaw Mode Selection Logic:
 *          - If outside circle (dist_to_center > radius):
 *            * Point towards edge (set_mode_to_default)
 *            * Allows seeing where flying to
 *            * Standard waypoint-pointing behavior
 *          - If inside circle (dist_to_center ≤ radius):
 *            * Hold current yaw (AutoYaw::Mode::HOLD)
 *            * Prevents spinning while moving to edge
 *            * More stable visual reference
 * 
 *          Distance to Center Calculation:
 *          - get_horizontal_distance() helper function
 *          - pos_control->get_pos_estimate_NEU_cm().xy() = current position
 *          - copter.circle_nav->get_center_NEU_cm().xy() = circle center
 *          - 2D horizontal distance only
 *          - Used to determine inside vs outside circle
 * 
 *          Outside Circle Yaw:
 *          - dist_to_center > radius AND > 500cm
 *          - 500cm (5m) additional threshold prevents jitter
 *          - set_mode_to_default(false) points at target
 *          - Vehicle flies forward towards circle edge
 *          - Visual cue of intended path
 * 
 *          Inside Circle Yaw:
 *          - Vehicle already within circle boundary
 *          - Moving to edge is short distance
 *          - HOLD mode prevents disorienting rotation
 *          - Maintains spatial awareness
 *          - Smoother transition to circling
 * 
 *          ROI Mode Preservation:
 *          - if (auto_yaw.mode() != AutoYaw::Mode::ROI)
 *          - If DO_SET_ROI previously set
 *          - Don't override with circle yaw logic
 *          - Continue pointing at ROI target
 *          - Camera tracking priority
 * 
 *          CIRCLE_MOVE_TO_EDGE Submode:
 *          - set_submode(SubMode::CIRCLE_MOVE_TO_EDGE)
 *          - Auto::run() calls wp_run() for this submode
 *          - Same execution as WP submode
 *          - Distinction for verify_circle() logic
 *          - Transition to CIRCLE when edge reached
 * 
 *          Immediate Circle Start (dist ≤ 3m):
 *          - circle_start() called directly
 *          - No waypoint navigation needed
 *          - Begins circular flight immediately
 *          - Smoother, more efficient
 *          - Typical when consecutive circles or nearby start
 * 
 *          verify_circle() Monitoring:
 *          - Mission system calls verify_circle()
 *          - If submode == CIRCLE_MOVE_TO_EDGE:
 *            * Checks wp_nav->reached_wp_destination()
 *            * When reached, calls circle_start()
 *            * Transitions to actual circling
 *          - If submode == CIRCLE:
 *            * Monitors turns completed
 *            * Returns true when required turns done
 * 
 *          Circle Start Transition:
 *          - circle_start() initializes AC_Circle controller
 *          - Sets angular rate, radius, center
 *          - Begins circular motion
 *          - Submode changes to CIRCLE
 *          - circle_run() takes over control
 * 
 *          Use Cases:
 *          - Survey/mapping circular patterns
 *          - Multiple orbits around point of interest
 *          - Camera work around subject
 *          - Waiting patterns in missions
 *          - Wind estimation maneuvers
 * 
 * @param[in] circle_center Center point of circle (lat, lon, alt)
 * @param[in] radius_m Circle radius in meters
 * @param[in] ccw_turn True for counter-clockwise, false for clockwise rotation
 * 
 * @note Assumes caller verified GPS position OK
 * @note 3-meter threshold determines if waypoint navigation needed
 * @note Preserves ROI yaw mode if active
 * @note Yaw behavior depends on position relative to circle
 * @note Calls circle_start() directly if close enough to edge
 * 
 * @warning Triggers terrain failsafe if terrain data unavailable
 * 
 * @see circle_start() - Begins circular flight when at edge
 * @see do_circle() - Mission command handler that calls this
 * @see verify_circle() - Monitors progress and completion
 * @see circle_run() - Executes circular flight control
 */
/**
 * @brief Initialize movement to the edge of a circular loiter pattern
 * 
 * @details This function prepares the vehicle to fly to the edge of a circle before
 *          beginning circular flight. If the vehicle is already close to the edge
 *          (within 3 meters), it immediately starts circling. Otherwise, it calculates
 *          the closest point on the circle and navigates there first.
 *          
 *          Circle setup sequence:
 *          1. Configure circle center location in circle_nav controller
 *          2. Set circle radius (or use existing if radius_m is zero)
 *          3. Set circle direction (CCW negative rate, CW positive rate)
 *          4. Calculate closest point on circle from current position
 *          5. If distance to edge > 3m, navigate to edge using waypoint controller
 *          6. If distance to edge <= 3m, immediately begin circling
 *          
 *          Yaw behavior during approach:
 *          - If ROI (Region of Interest) is active, yaw control is preserved
 *          - If outside circle (>5m from center), point at circle edge
 *          - If inside circle, hold current yaw to avoid spinning during approach
 *          
 *          Altitude handling:
 *          - Circle edge waypoint altitude matches circle_center altitude and frame
 *          - Altitude frame (ABOVE_HOME, ABOVE_TERRAIN, etc.) is preserved
 * 
 * @param[in] circle_center Center location of the circular pattern (lat, lon, alt)
 * @param[in] radius_m Circle radius in meters (if zero, uses current circle_nav radius)
 * @param[in] ccw_turn true for counter-clockwise rotation, false for clockwise
 * 
 * @note Caller must verify GPS and position estimate are valid before calling
 * @note 3-meter threshold balances smooth entry with excessive approach time
 * @note Failure to set waypoint destination triggers terrain failsafe
 * 
 * @see circle_start() for immediate circle initiation
 * @see do_circle() for the mission command handler
 * @see verify_circle() for circle completion detection
 */
void ModeAuto::circle_movetoedge_start(const Location &circle_center, float radius_m, bool ccw_turn)
{
    // set circle center
    copter.circle_nav->set_center(circle_center);

    // set circle radius
    if (!is_zero(radius_m)) {
        copter.circle_nav->set_radius_cm(radius_m * 100.0f);
    }

    // set circle direction by using rate
    float current_rate = copter.circle_nav->get_rate_degs();
    current_rate = ccw_turn ? -fabsf(current_rate) : fabsf(current_rate);
    copter.circle_nav->set_rate_degs(current_rate);

    // check our distance from edge of circle
    Vector3f circle_edge_neu;
    float dist_to_edge;
    copter.circle_nav->get_closest_point_on_circle_NEU_cm(circle_edge_neu, dist_to_edge);

    // if more than 3m then fly to edge
    if (dist_to_edge > 300.0f) {
        // convert circle_edge_neu to Location
        Location circle_edge(circle_edge_neu, Location::AltFrame::ABOVE_ORIGIN);

        // convert altitude to same as command
        circle_edge.set_alt_cm(circle_center.alt, circle_center.get_alt_frame());

        // initialise wpnav to move to edge of circle
        if (!wp_nav->set_wp_destination_loc(circle_edge)) {
            // failure to set destination can only be because of missing terrain data
            copter.failsafe_terrain_on_event();
        }

        // if we are outside the circle, point at the edge, otherwise hold yaw
        const float dist_to_center = get_horizontal_distance(pos_control->get_pos_estimate_NEU_cm().xy().tofloat(), copter.circle_nav->get_center_NEU_cm().xy().tofloat());
        // initialise yaw
        // To-Do: reset the yaw only when the previous navigation command is not a WP.  this would allow removing the special check for ROI
        if (auto_yaw.mode() != AutoYaw::Mode::ROI) {
            if (dist_to_center > copter.circle_nav->get_radius_cm() && dist_to_center > 500) {
                auto_yaw.set_mode_to_default(false);
            } else {
                // vehicle is within circle so hold yaw to avoid spinning as we move to edge of circle
                auto_yaw.set_mode(AutoYaw::Mode::HOLD);
            }
        }

        // set the submode to move to the edge of the circle
        set_submode(SubMode::CIRCLE_MOVE_TO_EDGE);
    } else {
        circle_start();
    }
}

/**
 * @brief Initialize circular flight pattern in Auto mode
 * 
 * @details This function starts the actual circular flight after the vehicle has either reached
 *          the circle edge (via circle_movetoedge_start) or determined it's already close enough.
 *          It initializes the AC_Circle navigation controller and sets up yaw control to rotate
 *          with the circular motion, creating smooth orbits around the center point.
 * 
 *          Prerequisite: circle_nav must already be configured with:
 *          - Circle center (lat/lon/alt)
 *          - Circle radius (meters converted to cm)
 *          - Angular rate (degrees/second, signed for direction)
 * 
 *          Called By:
 *          - circle_movetoedge_start(): When already close to edge (≤3m)
 *          - verify_circle(): When vehicle reaches circle edge after flying there
 *          - Transition from CIRCLE_MOVE_TO_EDGE to CIRCLE submode
 * 
 *          Circle Controller Initialization:
 *          - init_NEU_cm() starts the AC_Circle navigation
 *          - Uses existing circle_nav configuration
 *          - Begin generating circular trajectory
 *          - Position/velocity targets updated continuously
 *          - Runs at 100Hz+ in circle_run()
 * 
 *          init_NEU_cm() Parameters:
 *          - Center: copter.circle_nav->get_center_NEU_cm()
 *            * Returns circle center in NEU coordinates (cm)
 *            * NEU = North-East-Up frame relative to EKF origin
 *            * Previously set by circle_movetoedge_start()
 *          - Terrain flag: copter.circle_nav->center_is_terrain_alt()
 *            * True if circle altitude is terrain-relative
 *            * False if altitude is absolute or above-home
 *            * Affects altitude control during circling
 *          - Rate: copter.circle_nav->get_rate_degs()
 *            * Angular rate in degrees/second
 *            * Negative = counter-clockwise (CCW)
 *            * Positive = clockwise (CW)
 *            * Typically 10-30 deg/s
 * 
 *          Terrain-Relative Altitude:
 *          - If center_is_terrain_alt() returns true
 *          - Circle maintains constant height above terrain
 *          - Uses terrain database or rangefinder
 *          - Altitude adjusts as terrain varies
 *          - Useful for following ground contours
 * 
 *          Absolute Altitude:
 *          - If center_is_terrain_alt() returns false
 *          - Circle maintains constant altitude above sea level or home
 *          - Does not adjust for terrain variations
 *          - Simpler, no terrain data needed
 *          - Standard for most missions
 * 
 *          Yaw Mode Selection:
 *          - Default: AutoYaw::Mode::CIRCLE
 *          - Vehicle rotates to face direction of travel
 *          - Creates smooth rotation during orbit
 *          - Forward camera points tangent to circle
 *          - "Carousel" view of center point
 * 
 *          CIRCLE Yaw Mode Behavior:
 *          - Yaw angle advances with circular position
 *          - Always pointing in direction of travel
 *          - Clockwise circle: yaw rotates clockwise
 *          - Counter-clockwise: yaw rotates counter-clockwise
 *          - Complete 360° yaw rotation per orbit
 * 
 *          ROI Yaw Mode Preservation:
 *          - if (auto_yaw.mode() != AutoYaw::Mode::ROI)
 *          - If DO_SET_ROI previously active
 *          - Vehicle continues pointing at ROI
 *          - Camera tracks ROI throughout circle
 *          - Common for filming/surveying specific target
 * 
 *          Why Check ROI Mode:
 *          - DO_SET_ROI sets region of interest
 *          - User explicitly commanded yaw behavior
 *          - Should take priority over default CIRCLE mode
 *          - Allows orbiting while camera points at target
 *          - Cinematography use case
 * 
 *          ROI During Circling:
 *          - Vehicle flies circular path
 *          - Yaw continuously adjusts to point at ROI
 *          - If ROI at circle center: vehicle rotates smoothly
 *          - If ROI offset: vehicle yaw varies around circle
 *          - Camera/gimbal may also track ROI
 * 
 *          Submode Transition:
 *          - set_submode(SubMode::CIRCLE)
 *          - Auto::run() will call circle_run()
 *          - circle_run() executes circular flight control
 *          - Runs at 100Hz+ until required turns complete
 * 
 *          circle_run() Execution:
 *          - Called by Auto::run() when _mode == SubMode::CIRCLE
 *          - Calls copter.circle_nav->update_cms()
 *          - Updates position controller targets
 *          - Monitors terrain (if terrain-relative)
 *          - Runs attitude controller with yaw
 * 
 *          Circular Trajectory Generation:
 *          - AC_Circle calculates position on circle
 *          - Advances angular position based on rate
 *          - Generates smooth velocity vector tangent to circle
 *          - Maintains constant radius
 *          - Adjusts for wind/drift
 * 
 *          Bank Angle in Circles:
 *          - Vehicle banks inward toward center
 *          - Bank angle depends on speed and radius
 *          - Higher rate or smaller radius = more bank
 *          - Attitude controller handles bank automatically
 *          - Limited by ANGLE_MAX parameter
 * 
 *          Speed Calculation:
 *          - Linear speed = radius × angular_rate (converted to rad/s)
 *          - Example: 20m radius, 20°/s = ~7 m/s
 *          - Larger radius = higher speed for same rate
 *          - Speed limited by WP_NAV_SPEED parameter
 *          - May reduce angular rate if speed limit hit
 * 
 *          Turn Counting:
 *          - AC_Circle tracks total angle traversed
 *          - get_angle_total_rad() returns cumulative angle
 *          - Divide by 2π to get number of complete circles
 *          - verify_circle() monitors turn count
 *          - Mission specifies required number of turns
 * 
 *          Circle Completion:
 *          - verify_circle() checks num_circles_completed
 *          - Compares to cmd.get_loiter_turns()
 *          - When required turns reached, returns true
 *          - Mission advances to next command
 *          - May exit to next waypoint or land
 * 
 *          Altitude Control During Circle:
 *          - Maintains commanded altitude
 *          - Terrain-relative: follows terrain profile
 *          - Absolute: constant altitude
 *          - U controller handles vertical position
 *          - Independent of horizontal circular motion
 * 
 *          Wind Compensation:
 *          - Position controller compensates for wind
 *          - Adjusts ground velocity to maintain track
 *          - Circle appears perfect relative to ground
 *          - May vary airspeed around circle
 *          - GPS-based feedback loop
 * 
 *          Use Cases:
 *          - Survey/mapping patterns
 *          - Multiple orbits for photography
 *          - Structure inspection
 *          - Search patterns
 *          - Wind estimation
 *          - Waiting patterns
 * 
 * @note Assumes circle_nav already configured with center, radius, rate
 * @note Sets yaw to CIRCLE mode unless ROI active
 * @note Vehicle will rotate while circling in CIRCLE yaw mode
 * @note ROI mode allows tracking target while orbiting
 * 
 * @see circle_movetoedge_start() - Moves to circle edge before calling this
 * @see circle_run() - Executes circular flight control at 100Hz+
 * @see verify_circle() - Monitors turn completion
 * @see do_circle() - Mission command handler that initiates circle
 */
/**
 * @brief Begin circular loiter pattern in AUTO mode
 * 
 * @details This function initiates active circular flight after the circle parameters
 *          (center, radius, rate) have been configured. It initializes the circle
 *          navigation controller and sets up appropriate yaw behavior.
 *          
 *          Circle flight initialization:
 *          1. Initialize circle_nav controller with current center position
 *          2. Configure terrain altitude following if center uses terrain frame
 *          3. Set rotation rate (degrees per second, sign determines direction)
 *          4. Configure yaw to track circle tangent (unless ROI active)
 *          5. Change AUTO submode to CIRCLE
 *          
 *          Yaw behavior:
 *          - Default: AutoYaw::Mode::CIRCLE - vehicle yaw follows circle tangent
 *          - If ROI active: Yaw mode preserved to point at region of interest
 *          
 *          The circle controller maintains:
 *          - Horizontal position on the circle circumference
 *          - Constant altitude (or terrain-relative altitude)
 *          - Smooth velocity profile with centripetal acceleration
 * 
 * @note Assumes circle_nav has been configured with center, radius, and rate
 * @note Called by circle_movetoedge_start() when vehicle reaches circle edge
 * @note Called by verify_circle() when transitioning from CIRCLE_MOVE_TO_EDGE submode
 * 
 * @see circle_movetoedge_start() for the approach phase
 * @see circle_run() for the corresponding execution function
 * @see AC_Circle for the circle navigation controller implementation
 */
void ModeAuto::circle_start()
{
    // initialise circle controller
    copter.circle_nav->init_NEU_cm(copter.circle_nav->get_center_NEU_cm(), copter.circle_nav->center_is_terrain_alt(), copter.circle_nav->get_rate_degs());

    if (auto_yaw.mode() != AutoYaw::Mode::ROI) {
        auto_yaw.set_mode(AutoYaw::Mode::CIRCLE);
    }

    // set submode to circle
    set_submode(SubMode::CIRCLE);
}

#if AC_NAV_GUIDED
/**
 * @brief Hand over control to external navigation controller (Guided mode) within Auto mission
 * 
 * @details This function implements NAV_GUIDED_ENABLE mission command functionality, allowing
 *          external navigation commands (from companion computer, GCS, or scripts) to control
 *          vehicle position while Auto mode continues running. This creates a "Guided within Auto"
 *          state where external position/velocity targets override the mission, with optional
 *          limits to ensure the vehicle stays within safe bounds.
 * 
 *          Called By:
 *          - do_nav_guided_enable(): When NAV_GUIDED_ENABLE command with p1=1 encountered
 *          - Enables external navigation control mid-mission
 * 
 *          NAV_GUIDED_ENABLE Mission Command:
 *          - MAV_CMD_NAV_GUIDED_ENABLE (ID 92)
 *          - p1=1: Enable guided within auto
 *          - p1=0: Disable guided, resume mission
 *          - Allows dynamic mission modification
 *          - External computer can inject waypoints
 * 
 *          Use Cases:
 *          - Precision tasks requiring external vision/sensors
 *          - Real-time path planning by companion computer
 *          - Human-in-the-loop mission modifications
 *          - Object tracking/following during mission
 *          - Emergency path updates from GCS
 * 
 *          Guided Mode Initialization:
 *          - copter.mode_guided.init(true)
 *          - true parameter = ignore pre-arm checks
 *          - Sets up guided mode controllers
 *          - Initializes position/velocity targets
 *          - Configures yaw mode
 * 
 *          Why ignore_checks = true:
 *          - Already in Auto mode (passed checks)
 *          - Vehicle already flying
 *          - Don't need to re-check arming
 *          - Don't need to re-check position estimate
 *          - Just taking control, not mode change
 * 
 *          Should Never Fail Logic:
 *          - guided.init(true) always succeeds
 *          - ignore_checks=true bypasses failure conditions
 *          - If fails, INTERNAL_ERROR logged
 *          - Indicates software bug, not user error
 *          - Early return prevents further execution
 * 
 *          INTERNAL_ERROR Reporting:
 *          - AP_InternalError::error_t::flow_of_control
 *          - Logs error for developer debugging
 *          - Indicates unexpected code path
 *          - Should be investigated if occurs
 *          - Not user-actionable
 * 
 *          Guided Limit Initialization:
 *          - copter.mode_guided.limit_init_time_and_pos()
 *          - Records current time as start time
 *          - Records current position as reference
 *          - Used for enforcing DO_GUIDED_LIMITS
 *          - Prevents runaway external control
 * 
 *          DO_GUIDED_LIMITS Command:
 *          - MAV_CMD_DO_GUIDED_LIMITS (ID 222)
 *          - Sets timeout in seconds
 *          - Sets altitude min/max limits (meters)
 *          - Sets horizontal distance limit (meters)
 *          - Applied before NAV_GUIDED_ENABLE
 * 
 *          Limit Checking:
 *          - verify_nav_guided_enable() calls limit_check()
 *          - Returns true if limits exceeded
 *          - Mission then advances to next command
 *          - Prevents external control going too far
 *          - Safety mechanism for autonomous operation
 * 
 *          Time Limit:
 *          - Maximum duration external control allowed
 *          - Measured from limit_init_time_and_pos() call
 *          - If exceeded, guided mode exits
 *          - Mission resumes normal execution
 *          - Prevents indefinite external control
 * 
 *          Altitude Limits:
 *          - Minimum and maximum altitude bounds
 *          - Relative to reference position
 *          - External commands cannot exceed
 *          - If violated, guided mode exits
 *          - Prevents altitude excursions
 * 
 *          Horizontal Distance Limit:
 *          - Maximum distance from reference position
 *          - 2D horizontal constraint
 *          - Circular boundary around start point
 *          - If exceeded, guided mode exits
 *          - Prevents vehicle flying away
 * 
 *          External Control Methods:
 *          - SET_POSITION_TARGET_LOCAL_NED (MAVLink)
 *          - SET_POSITION_TARGET_GLOBAL_INT (MAVLink)
 *          - SET_ATTITUDE_TARGET (MAVLink)
 *          - Guided mode API (scripting)
 *          - All accepted during NAVGUIDED state
 * 
 *          Position Target Example:
 *          - Companion computer sends waypoints
 *          - Vehicle navigates to each target
 *          - Smooth path following
 *          - Can chain multiple targets
 *          - Real-time path planning
 * 
 *          Velocity Target Example:
 *          - External controller sends velocity commands
 *          - Direct velocity control
 *          - High-rate updates (10-50Hz typical)
 *          - Precise trajectory following
 *          - Used for dynamic obstacles
 * 
 *          Submode Transition:
 *          - set_submode(SubMode::NAVGUIDED)
 *          - Auto::run() will call nav_guided_run()
 *          - nav_guided_run() delegates to mode_guided.run()
 *          - Standard guided mode control active
 *          - External commands processed
 * 
 *          nav_guided_run() Execution:
 *          - Called by Auto::run() when _mode == SubMode::NAVGUIDED
 *          - Calls copter.mode_guided.run()
 *          - Processes external navigation commands
 *          - Updates position/velocity targets
 *          - Runs attitude controller
 * 
 *          Mission Resume:
 *          - NAV_GUIDED_ENABLE with p1=0 disables guided
 *          - verify_nav_guided_enable() returns true
 *          - Mission advances to next command
 *          - Normal mission execution resumes
 *          - Clean handoff from external control
 * 
 *          Limit Violation Resume:
 *          - verify_nav_guided_enable() detects violation
 *          - Returns true (command complete)
 *          - Mission automatically advances
 *          - Safety limits prevent problematic behavior
 *          - Logged for post-flight review
 * 
 *          Yaw Control:
 *          - Guided mode handles yaw
 *          - Can be commanded externally
 *          - Or auto-calculated (point at target)
 *          - Depends on external command type
 *          - MAVLink SET_*_TARGET messages specify yaw
 * 
 *          Typical Mission Sequence:
 *          1. Normal mission waypoints
 *          2. DO_GUIDED_LIMITS (optional, set safety bounds)
 *          3. NAV_GUIDED_ENABLE (p1=1, start external control)
 *          4. External computer sends position/velocity targets
 *          5. NAV_GUIDED_ENABLE (p1=0, end external control)
 *          6. Resume normal mission waypoints
 * 
 *          Companion Computer Integration:
 *          - ROS/DDS integration (AP_DDS)
 *          - MAVLink over serial/UDP/TCP
 *          - DroneKit or MAVSDK APIs
 *          - Custom companion computer code
 *          - Real-time sensor fusion
 * 
 *          Safety Considerations:
 *          - Always set DO_GUIDED_LIMITS before enabling
 *          - Timeout prevents loss of external control
 *          - Altitude limits prevent terrain collision
 *          - Distance limit prevents flyaway
 *          - Mission can always resume
 * 
 * @note Requires AC_NAV_GUIDED feature compiled in
 * @note Guided mode init with ignore_checks=true should never fail
 * @note Limit checking reference time/position recorded at start
 * @note External control via MAVLink or scripting API
 * @note DO_GUIDED_LIMITS should be set before this command
 * 
 * @warning Logs INTERNAL_ERROR if guided.init() fails unexpectedly
 * 
 * @see nav_guided_run() - Executes guided control within Auto
 * @see verify_nav_guided_enable() - Checks limits and completion
 * @see do_nav_guided_enable() - Command handler that calls this
 * @see do_guided_limits() - Sets safety limits for guided control
 */
/**
 * @brief Hand over navigation control to external controller during AUTO mission
 * 
 * @details This function enables the MAV_CMD_NAV_GUIDED_ENABLE mission command,
 *          allowing an external navigation computer (companion computer, ground
 *          station) to send position/velocity/attitude targets while the mission
 *          remains in AUTO mode. This provides a way to integrate external path
 *          planning or obstacle avoidance into autonomous missions.
 *          
 *          Initialization sequence:
 *          1. Initialize guided mode controller (reuses Mode::Guided implementation)
 *          2. Initialize time and position limits for guided operation
 *          3. Change AUTO submode to NAVGUIDED
 *          
 *          While in NAVGUIDED submode:
 *          - External controller sends SET_POSITION_TARGET_LOCAL_NED or similar
 *          - Guided limits (time, altitude, horizontal distance) are enforced
 *          - Mission advances when limits are exceeded or explicitly disabled
 *          
 *          Typical use cases:
 *          - Vision-based precision navigation to mission waypoint
 *          - Dynamic obstacle avoidance during mission execution
 *          - Integration with offboard path planning algorithms
 *          - Companion computer control within mission framework
 *          
 *          Safety considerations:
 *          - Guided limits prevent unlimited external control duration
 *          - Position limits constrain allowable deviation from mission path
 *          - Mission can override guided control at any time
 *          - Failsafe mechanisms remain active during guided navigation
 * 
 * @note Requires AC_NAV_GUIDED feature enabled at compile time
 * @note Guided mode initialization cannot fail when ignore_checks=true
 * @note External controller must send targets at >2Hz to maintain control
 * @warning Loss of external control commands will trigger guided timeout
 * 
 * @see do_nav_guided_enable() for the mission command handler
 * @see do_guided_limits() for setting operational constraints
 * @see nav_guided_run() for the execution function
 * @see verify_nav_guided_enable() for completion detection
 */
void ModeAuto::nav_guided_start()
{
    // call regular guided flight mode initialisation
    if (!copter.mode_guided.init(true)) {
        // this should never happen because guided mode never fails to init
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        return;
    }

    // initialise guided start time and position as reference for limit checking
    copter.mode_guided.limit_init_time_and_pos();

    // set submode
    set_submode(SubMode::NAVGUIDED);
}
#endif //AC_NAV_GUIDED

/**
 * @brief Check if vehicle is currently executing a landing sequence in Auto mode
 * 
 * @details This function determines if the vehicle is actively landing, which is important for:
 *          - Landing detection logic (throttle behavior, motor output)
 *          - Disarming decisions (when safe to disarm)
 *          - Pilot input handling (allow repositioning during landing)
 *          - GCS status reporting (flight phase indication)
 *          - Log analysis (identify landing portions of flight)
 * 
 *          Checked By:
 *          - Parent Mode class for flight phase determination
 *          - Land detector for landing-specific behavior
 *          - Pilot input processors for land repositioning
 *          - Disarm logic for safe shutdown decisions
 * 
 *          Landing Submodes:
 *          - SubMode::LAND: Direct landing command (NAV_LAND, NAV_VTOL_LAND)
 *          - SubMode::RTL: Return-to-launch may include landing phase
 *          - Other submodes: Not considered landing
 * 
 *          LAND Submode:
 *          - Triggered by NAV_LAND mission command
 *          - Triggered by NAV_VTOL_LAND mission command
 *          - May fly to location first, then descend
 *          - Always returns true when in this submode
 *          - Indicates committed landing sequence
 * 
 *          RTL Submode:
 *          - Auto mode can execute RTL as mission command (NAV_RETURN_TO_LAUNCH)
 *          - RTL has multiple phases: climb, return, loiter, land
 *          - Only final phases constitute "landing"
 *          - Delegates to copter.mode_rtl.is_landing()
 *          - True only in RTL's descent/landing phases
 * 
 *          mode_rtl.is_landing() Behavior:
 *          - Returns true if RTL in FINAL_DESCENT or LAND phase
 *          - FINAL_DESCENT: Final altitude descent before land
 *          - LAND: Actual landing (throttle decrease to ground)
 *          - Earlier RTL phases: climbing, returning, loitering
 *          - Not considered landing until final descent
 * 
 *          Why Check RTL Separately:
 *          - RTL is multi-phase operation
 *          - Most of RTL is navigation, not landing
 *          - Only final phases need landing behavior
 *          - Allows appropriate handling in each phase
 *          - Pilot can reposition only during actual landing
 * 
 *          Use Case - Pilot Repositioning:
 *          - During landing, pilot can use roll/pitch
 *          - Allows last-minute adjustments
 *          - Prevents landing on obstacle
 *          - Fine-tune landing position
 *          - Only enabled when is_landing() returns true
 * 
 *          Use Case - Landing Detection:
 *          - Monitors throttle level
 *          - Watches for reduced motor output
 *          - Detects ground contact
 *          - Sets ap.land_complete flag
 *          - Different thresholds during landing vs flight
 * 
 *          Use Case - Disarming:
 *          - After landing completes
 *          - verify_land() checks motors at GROUND_IDLE
 *          - Can auto-disarm if mission ends
 *          - Safe to power down
 *          - is_landing() helps determine context
 * 
 *          Non-Landing Submodes:
 *          - WP: Waypoint navigation
 *          - CIRCLE: Circular flight
 *          - TAKEOFF: Taking off
 *          - LOITER: Hovering at position
 *          - NAVGUIDED: External control
 *          - NAV_SCRIPT_TIME: Script control
 *          - NAV_ATTITUDE_TIME: Attitude hold
 *          - NAV_PAYLOAD_PLACE: Payload descent (not landing)
 *          - CIRCLE_MOVE_TO_EDGE: Moving to circle start
 *          - LOITER_TO_ALT: Loitering to altitude
 * 
 *          Payload Place vs Landing:
 *          - NAV_PAYLOAD_PLACE descends but isn't "landing"
 *          - Releases payload and ascends
 *          - Vehicle doesn't remain on ground
 *          - Different behavior needed
 *          - Not considered landing sequence
 * 
 *          Return Value Meaning:
 *          - true: Vehicle actively executing landing
 *          - false: Normal flight operations
 *          - Affects multiple flight systems
 *          - Changes control input handling
 *          - Modifies safety thresholds
 * 
 *          Redundant Return Statement:
 *          - Final "return false;" is unreachable
 *          - All cases already return
 *          - Kept for defensive programming
 *          - Compiler may optimize away
 *          - No functional impact
 * 
 *          Flight Phase Context:
 *          - Takeoff phase: is_taking_off() returns true
 *          - Cruise phase: Both return false
 *          - Landing phase: is_landing() returns true
 *          - Allows phase-appropriate behavior
 * 
 * @return true if vehicle is actively landing (LAND submode or RTL landing phase)
 * @return false if vehicle in other flight phases
 * 
 * @note Payload place (descent/release/ascent) is not considered landing
 * @note RTL only considered landing in final descent/land phases
 * @note Used for landing detection, disarming, and pilot input handling
 * 
 * @see is_taking_off() - Companion function for takeoff detection
 * @see land_run() - Landing control execution
 * @see rtl_run() - RTL control execution (delegates to mode_rtl)
 * @see verify_land() - Landing completion check
 */
bool ModeAuto::is_landing() const
{
    switch(_mode) {
    case SubMode::LAND:
        return true;
    case SubMode::RTL:
        return copter.mode_rtl.is_landing();
    default:
        return false;
    }
    return false;
}

/**
 * @brief Check if vehicle is currently executing a takeoff sequence in Auto mode
 * 
 * @details This function determines if the vehicle is actively taking off, which affects:
 *          - Motor arming behavior (auto-arm during takeoff)
 *          - Landing gear retraction timing
 *          - Altitude controller initialization
 *          - Flight phase reporting to GCS
 *          - Log analysis for takeoff identification
 * 
 *          Checked By:
 *          - Parent Mode class for flight phase determination
 *          - Takeoff controller for phase-specific logic
 *          - Landing gear system for retraction timing
 *          - Motor/throttle controllers for arming
 * 
 *          Takeoff Conditions (Both Must Be True):
 *          1. _mode == SubMode::TAKEOFF: In takeoff submode
 *          2. !auto_takeoff.complete: Takeoff not yet finished
 * 
 *          SubMode::TAKEOFF:
 *          - Set by do_takeoff() mission command handler
 *          - Triggered by NAV_TAKEOFF (ID 22)
 *          - Triggered by NAV_VTOL_TAKEOFF (ID 84)
 *          - Remains active throughout takeoff
 *          - Transitions away when complete
 * 
 *          auto_takeoff.complete Flag:
 *          - Managed by AC_Loiter::AutoTakeoff class
 *          - Set to false in auto_takeoff.start()
 *          - Set to true when target altitude reached
 *          - Checked by verify_takeoff()
 *          - Controls takeoff completion
 * 
 *          Takeoff Altitude Target:
 *          - Specified in NAV_TAKEOFF command
 *          - Can be absolute, relative, or terrain-relative
 *          - auto_takeoff.start(alt_target_cm, alt_target_terrain)
 *          - Minimum 1m above current position if landed
 *          - Ensures safe altitude gain
 * 
 *          Why Both Conditions:
 *          - In TAKEOFF mode: Commanded to take off
 *          - !complete: Still climbing to target
 *          - If complete: Physically taken off but may still be in submode
 *          - verify_takeoff() will advance mission when complete
 *          - Brief period where in TAKEOFF but complete=true
 * 
 *          Auto-Arm During Takeoff:
 *          - takeoff_run() can enable auto-arming
 *          - If ALLOW_TAKEOFF_WITHOUT_RAISING_THROTTLE option set
 *          - copter.set_auto_armed(true)
 *          - Bypasses throttle stick check
 *          - Allows automated takeoff
 * 
 *          Landing Gear Retraction:
 *          - verify_takeoff() retracts gear when complete
 *          - copter.landinggear.retract_after_takeoff()
 *          - Only after altitude reached
 *          - Prevents premature retraction
 *          - is_taking_off() helps gate this
 * 
 *          Altitude Controller Initialization:
 *          - auto_takeoff.start() calls pos_control->init_U_controller()
 *          - Clears vertical position integrator
 *          - Fresh start for climb
 *          - Prevents previous flight affecting takeoff
 *          - Clean initial state
 * 
 *          Takeoff Run Execution:
 *          - Auto::run() calls takeoff_run() when _mode == TAKEOFF
 *          - takeoff_run() calls auto_takeoff.run()
 *          - AutoTakeoff handles climb profile
 *          - Manages WP_NAVALT_MIN parameter
 *          - Smooth altitude gain
 * 
 *          WP_NAVALT_MIN Parameter:
 *          - Minimum altitude for waypoint navigation
 *          - AutoTakeoff climbs to this before horizontal movement
 *          - Ensures obstacle clearance
 *          - After reached, can navigate horizontally
 *          - Safety feature for low takeoffs
 * 
 *          Takeoff Completion Detection:
 *          - AutoTakeoff monitors altitude error
 *          - Sets complete=true near target altitude
 *          - verify_takeoff() returns true
 *          - Mission advances to next command
 *          - Typically first waypoint or loiter
 * 
 *          Transition from Takeoff:
 *          - When complete=true, verify_takeoff() returns true
 *          - Mission command completes
 *          - Next navigation command starts
 *          - Usually do_nav_wp() or other nav command
 *          - Submode changes from TAKEOFF
 * 
 *          Comparison with is_landing():
 *          - Complementary flight phase detection
 *          - is_landing(): Final phase (descent)
 *          - is_taking_off(): Initial phase (climb)
 *          - Cruise: Both return false
 *          - Different behaviors in each phase
 * 
 *          Use in Motor Controller:
 *          - During takeoff, motor limits may differ
 *          - Spin-when-armed behavior
 *          - Throttle ramp rates
 *          - Prevents aggressive movements
 *          - Smooth power application
 * 
 *          SITL Testing:
 *          - Can test auto takeoff in simulation
 *          - Mission starts on ground
 *          - NAV_TAKEOFF first command
 *          - Vehicle climbs to altitude
 *          - Proceeds with mission
 * 
 *          Return Value Meaning:
 *          - true: Vehicle actively climbing during takeoff
 *          - false: Takeoff complete or not in takeoff phase
 *          - Used for phase-specific behavior
 *          - Affects multiple subsystems
 * 
 *          Edge Case - Takeoff Interrupted:
 *          - If mode switched during takeoff
 *          - is_taking_off() returns false
 *          - AutoTakeoff state reset
 *          - Different mode takes control
 *          - Takeoff incomplete
 * 
 * @return true if vehicle is actively taking off (in TAKEOFF submode and not yet complete)
 * @return false if takeoff complete or not in takeoff phase
 * 
 * @note Both conditions must be met: TAKEOFF submode AND !complete flag
 * @note auto_takeoff.complete set true when target altitude reached
 * @note Used for auto-arming, landing gear retraction, and flight phase reporting
 * 
 * @see is_landing() - Companion function for landing detection
 * @see takeoff_run() - Takeoff control execution
 * @see verify_takeoff() - Takeoff completion check
 * @see do_takeoff() - Takeoff mission command handler
 */
bool ModeAuto::is_taking_off() const
{
    return ((_mode == SubMode::TAKEOFF) && !auto_takeoff.complete);
}

#if AC_PAYLOAD_PLACE_ENABLED
/**
 * @brief Initialize payload place descent phase (controlled descent to place payload)
 * 
 * @details This function prepares the vehicle for the descent phase of the payload place sequence,
 *          which involves slowly descending while monitoring motor thrust to detect payload touchdown,
 *          then releasing the payload via gripper, and ascending back to the starting altitude.
 *          This is distinct from landing because the vehicle doesn't remain on the ground.
 * 
 *          Called By:
 *          - do_payload_place(): When NAV_PAYLOAD_PLACE lat/lng are zero (descend in place)
 *          - PayloadPlace::run(): When state machine transitions from FlyToLocation after reaching target
 * 
 *          NAV_PAYLOAD_PLACE Mission Command:
 *          - MAV_CMD_NAV_PAYLOAD_PLACE (ID 94)
 *          - p1: Maximum descent distance in cm
 *          - lat/lng: Target location (if non-zero, fly there first)
 *          - alt: Not used for this command
 *          - Automated payload delivery
 * 
 *          PayloadPlace State Machine:
 *          1. FlyToLocation: Navigate to target coordinates (if provided)
 *          2. Descent_Start: Initialize descent (this function)
 *          3. Descent: Controlled descent while monitoring thrust
 *          4. Release: Trigger gripper release
 *          5. Releasing: Wait for gripper to open
 *          6. Delay: Post-release delay (PLDP_DELAY_S parameter)
 *          7. Ascent: Climb back to start altitude
 *          8. Done: Sequence complete, mission advances
 * 
 *          Position Controller Setup:
 *          - Configures both horizontal (NE) and vertical (U) controllers
 *          - Sets speed and acceleration limits from wp_nav defaults
 *          - Initializes controllers if not already active
 *          - Prepares for smooth, controlled descent
 * 
 *          Horizontal (NE) Controller:
 *          - set_max_speed_accel_NE_cm(): Maximum horizontal speed/accel
 *          - set_correction_speed_accel_NE_cm(): Wind correction limits
 *          - Uses wp_nav defaults (typically 500-1500 cm/s)
 *          - Maintains horizontal position during descent
 * 
 *          Why Set NE Limits:
 *          - Payload place requires precise position hold
 *          - Wind can push vehicle during slow descent
 *          - Position controller fights drift
 *          - Must stay over payload drop point
 *          - Safety: Don't swing payload
 * 
 *          NE Controller Initialization:
 *          - if (!pos_control->is_active_NE()): Check if running
 *          - pos_control->init_NE_controller(): Fresh start
 *          - Clears integrator terms
 *          - Sets current position as target
 *          - Prevents sudden movements
 * 
 *          Vertical (U) Controller:
 *          - set_max_speed_accel_U_cm(): Up/down speed and accel limits
 *          - set_correction_speed_accel_U_cmss(): Altitude correction limits
 *          - Uses wp_nav defaults or PLDP_DESCENT_SPEED_MS parameter
 *          - Controls descent rate precisely
 * 
 *          PLDP_DESCENT_SPEED_MS Parameter:
 *          - Desired descent speed in m/s
 *          - Set in PayloadPlace::run() state machine
 *          - Converted to cm/s: * 100.0
 *          - Defaults to LAND_SPEED if not set
 *          - Slow descent for gentle payload contact
 * 
 *          U Controller Initialization:
 *          - if (!pos_control->is_active_U()): Check if running
 *          - pos_control->init_U_controller(): Fresh start
 *          - Clears vertical integrator
 *          - Sets current altitude as starting point
 *          - Smooth descent initiation
 * 
 *          Why Initialize Controllers:
 *          - May be transitioning from FlyToLocation
 *          - May be starting from different control mode
 *          - Clears old state/integrators
 *          - Prevents jerks or oscillations
 *          - Clean state for descent
 * 
 *          Yaw Behavior:
 *          - copter.flightmode->auto_yaw.set_mode(Mode::AutoYaw::Mode::HOLD)
 *          - Locks yaw at current heading
 *          - Prevents rotation during descent
 *          - Maintains payload orientation
 *          - Simplifies position control
 * 
 *          Why HOLD Yaw:
 *          - Payload hanging below vehicle
 *          - Rotation causes pendulum swing
 *          - Want minimal disturbance
 *          - Maintain approach heading
 *          - Safer for precise placement
 * 
 *          State Transition:
 *          - state = PayloadPlace::State::Descent_Start
 *          - Next loop: PayloadPlace::run() processes Descent_Start
 *          - Initializes thrust calibration variables
 *          - Transitions immediately to Descent state
 *          - Actual descent begins
 * 
 *          Descent_Start Processing (Next Loop):
 *          - Records descent_start_altitude_cm
 *          - Sets descent_speed_cms from parameter
 *          - Initializes descent_thrust_level = 1.0
 *          - Sets descent_established_time_ms
 *          - Immediately transitions to Descent state
 * 
 *          Thrust-Based Touchdown Detection:
 *          - During descent, monitors attitude_control->get_throttle_in()
 *          - Calibrates thrust needed for steady descent
 *          - Detects reduction when payload touches ground
 *          - More reliable than altitude for soft surfaces
 *          - Works on uneven terrain
 * 
 *          Thrust Calibration Period:
 *          - 2 seconds (descent_thrust_cal_duration_ms = 2000)
 *          - Measures minimum thrust during steady descent
 *          - Establishes baseline: descent_thrust_level
 *          - Touchdown = thrust < PLDP_THRUST_PLACED_FRACTION * baseline
 *          - Accounts for vehicle weight variations
 * 
 *          PLDP_THRUST_PLACED_FRACTION Parameter:
 *          - Fraction of descent thrust indicating touchdown
 *          - Typically 0.7-0.9
 *          - Lower = more sensitive detection
 *          - Higher = less false positives
 *          - Tuned per vehicle/payload
 * 
 *          Maximum Descent Distance:
 *          - p1 parameter from mission command
 *          - descent_max_cm: Maximum descent from start altitude
 *          - Safety limit to prevent excessive descent
 *          - If exceeded without touchdown, abort
 *          - Return to start altitude
 * 
 *          Rangefinder Integration:
 *          - PLDP_RNG_MAX parameter: Maximum rangefinder reading
 *          - If rangefinder shows > max distance, not placed yet
 *          - Provides secondary confirmation
 *          - Prevents false positives from thrust variations
 *          - Enhanced reliability
 * 
 *          Gripper Integration:
 *          - AP::gripper() interface
 *          - Release command when touchdown detected
 *          - Wait for gripper to fully open
 *          - Manual release monitoring (pilot override)
 *          - State machine handles timing
 * 
 *          Ascent After Placement:
 *          - Returns to descent_start_altitude_cm
 *          - Same horizontal position
 *          - Clears payload from drop zone
 *          - Mission can continue
 *          - Safe for next operations
 * 
 *          Use Case - Delivery Missions:
 *          - Automated package delivery
 *          - Precision agriculture (seed/sample placement)
 *          - Search and rescue (supply drops)
 *          - Inspection (sensor placement)
 *          - Construction (material delivery)
 * 
 *          Safety Considerations:
 *          - Maximum descent limit prevents ground collision
 *          - Thrust monitoring detects contact gently
 *          - Abort if rangefinder unavailable (if required)
 *          - Manual gripper release allowed
 *          - Automatic ascent after placement
 * 
 * @note Requires AC_PAYLOAD_PLACE_ENABLED feature compiled in
 * @note Position controllers initialized for precise position hold
 * @note Yaw locked to prevent payload swing
 * @note State machine immediately transitions to Descent after this
 * 
 * @see PayloadPlace::run() - Main state machine execution
 * @see do_payload_place() - Mission command handler
 * @see verify() - Checks if payload place complete
 */
// auto_payload_place_start - initialises controller to implement a placing
void PayloadPlace::start_descent()
{
    auto *pos_control = copter.pos_control;
    auto *wp_nav = copter.wp_nav;

    // set horizontal speed and acceleration limits
    pos_control->set_max_speed_accel_NE_cm(wp_nav->get_default_speed_NE_cms(), wp_nav->get_wp_acceleration_cmss());
    pos_control->set_correction_speed_accel_NE_cm(wp_nav->get_default_speed_NE_cms(), wp_nav->get_wp_acceleration_cmss());

    // initialise the vertical position controller
    if (!pos_control->is_active_NE()) {
        pos_control->init_NE_controller();
    }

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_U_cm(wp_nav->get_default_speed_down_cms(), wp_nav->get_default_speed_up_cms(), wp_nav->get_accel_U_cmss());
    pos_control->set_correction_speed_accel_U_cmss(wp_nav->get_default_speed_down_cms(), wp_nav->get_default_speed_up_cms(), wp_nav->get_accel_U_cmss());

    // initialise the vertical position controller
    if (!pos_control->is_active_U()) {
        pos_control->init_U_controller();
    }

    // initialise yaw
    copter.flightmode->auto_yaw.set_mode(Mode::AutoYaw::Mode::HOLD);

    state = PayloadPlace::State::Descent_Start;
}
#endif

/**
 * @brief Determine if pilot's yaw stick input should control vehicle heading during Auto mode
 * 
 * @details In Auto mode, yaw is typically controlled automatically (point at waypoint, ROI, or hold heading).
 *          However, in certain situations pilot yaw input is allowed for manual override, particularly
 *          during landing when fine-tuning heading is important, or when AUTO_OPTIONS parameter permits it.
 *          This function determines when pilot has manual yaw authority during autonomous flight.
 * 
 *          Called By:
 *          - Mode base class yaw input processing
 *          - Auto yaw controller to determine control source
 *          - Attitude controller input mixing
 *          - Used to gate pilot yaw stick effects
 * 
 *          Three Conditions Allow Pilot Yaw (OR logic):
 *          1. AUTO_OPTIONS parameter allows pilot yaw (IgnorePilotYaw NOT set)
 *          2. In RTL submode AND mode_rtl allows pilot yaw
 *          3. In LAND submode (always allow during landing)
 * 
 *          AUTO_OPTIONS Parameter:
 *          - Bitmask parameter controlling Auto mode behavior
 *          - Bit for Option::IgnorePilotYaw
 *          - If set: Pilot yaw input completely ignored
 *          - If clear: Pilot yaw input allowed (default)
 *          - User preference for automation level
 * 
 *          Option::IgnorePilotYaw Bit:
 *          - When enabled, enforce full autonomous yaw control
 *          - Pilot cannot override yaw with stick
 *          - Useful for precision missions
 *          - Prevents accidental pilot interference
 *          - Competition/commercial use case
 * 
 *          allow_yaw_option Logic:
 *          - !option_is_enabled(Option::IgnorePilotYaw)
 *          - Returns true if IgnorePilotYaw is NOT set
 *          - Default behavior: Allow pilot yaw
 *          - User must explicitly disable
 *          - Conservative default (more control)
 * 
 *          RTL Submode Yaw Control:
 *          - Auto mode can execute RTL (NAV_RETURN_TO_LAUNCH command)
 *          - RTL has its own phases and yaw rules
 *          - Delegates to copter.mode_rtl.use_pilot_yaw()
 *          - RTL typically allows yaw during descent/landing
 *          - Consistent behavior between Auto-RTL and direct RTL mode
 * 
 *          mode_rtl.use_pilot_yaw() Behavior:
 *          - Returns true during RTL landing phase
 *          - Allows pilot to adjust heading before touchdown
 *          - Important for landing orientation
 *          - False during climb/return phases
 *          - Phase-appropriate control authority
 * 
 *          Landing Submode Always Allows Yaw:
 *          - _mode == SubMode::LAND
 *          - NAV_LAND or NAV_VTOL_LAND mission commands
 *          - Pilot can adjust heading during entire landing sequence
 *          - Critical for final landing position/orientation
 *          - May need to avoid obstacles
 * 
 *          Why Always Allow Yaw During Landing:
 *          - Final approach may need adjustment
 *          - Wind may require heading correction
 *          - Obstacle avoidance (visual by pilot)
 *          - Land into wind for stability
 *          - Safety-critical manual override
 * 
 *          OR Logic (Any Condition True):
 *          - return allow_yaw_option || rtl_allow_yaw || landing
 *          - If ANY condition true, pilot yaw enabled
 *          - Multiple paths to enable pilot control
 *          - Safety: Defaults to allowing control
 *          - Must actively disable via parameter
 * 
 *          Use Case - Precision Missions:
 *          - Enable IgnorePilotYaw option
 *          - Fully autonomous yaw control
 *          - ROI or waypoint heading maintained
 *          - No pilot interference
 *          - Repeatable heading for inspection
 * 
 *          Use Case - Landing Flexibility:
 *          - Even with IgnorePilotYaw set
 *          - Pilot can still adjust yaw during landing
 *          - Landing phase too critical to disable
 *          - Safety override preserved
 *          - Final control authority to pilot
 * 
 *          Use Case - Manual Camera Pointing:
 *          - During Auto mission
 *          - Pilot adjusts yaw to frame shot
 *          - Vehicle continues waypoint navigation
 *          - Horizontal position autonomous
 *          - Yaw under manual control
 * 
 *          Yaw Input Processing:
 *          - If this returns true, pilot yaw stick active
 *          - Input mixed into attitude controller
 *          - May override auto_yaw commands
 *          - Smooth blending of auto/manual
 *          - Rate-limited for safety
 * 
 *          Auto Yaw Modes (When Pilot Disabled):
 *          - AutoYaw::Mode::HOLD: Hold current heading
 *          - AutoYaw::Mode::LOOK_AT_NEXT_WP: Point at waypoint
 *          - AutoYaw::Mode::ROI: Point at region of interest
 *          - AutoYaw::Mode::FIXED: Fixed heading (CONDITION_YAW)
 *          - AutoYaw::Mode::LOOK_AHEAD: Follow path tangent
 *          - AutoYaw::Mode::CIRCLE: Circling yaw rate
 * 
 *          Pilot Yaw Rate:
 *          - Pilot input controls yaw rate (deg/s)
 *          - Not direct heading control
 *          - Expo curves applied to stick input
 *          - Rate limits enforced (ACRO_YAW_P)
 *          - Smooth response
 * 
 *          Return to Auto Yaw:
 *          - When pilot releases yaw stick (neutral)
 *          - Auto yaw mode may resume
 *          - Depends on auto_yaw mode setting
 *          - Some modes hold, others resume tracking
 *          - Seamless transition
 * 
 *          Parameter Interaction:
 *          - AUTO_OPTIONS bitmask
 *          - RTL_OPTIONS bitmask (for RTL submode)
 *          - WP_YAW_BEHAVIOR (how vehicle points at waypoints)
 *          - All affect final yaw behavior
 *          - Complex interaction matrix
 * 
 *          Flight Testing:
 *          - Test with IgnorePilotYaw disabled (default)
 *          - Verify yaw stick controls heading
 *          - Test with IgnorePilotYaw enabled
 *          - Verify yaw stick ignored except landing
 *          - Confirm smooth transitions
 * 
 * @return true if pilot yaw input should be accepted
 * @return false if pilot yaw input should be ignored (fully autonomous yaw)
 * 
 * @note Landing phase (LAND submode) always allows pilot yaw regardless of options
 * @note RTL submode delegates to mode_rtl.use_pilot_yaw() for phase-appropriate control
 * @note Default behavior is to allow pilot yaw unless IgnorePilotYaw option set
 * 
 * @see option_is_enabled() - Checks AUTO_OPTIONS parameter bits
 * @see auto_yaw - Auto yaw controller managing autonomous heading
 * @see land_run() - Landing execution where pilot yaw useful
 */
// returns true if pilot's yaw input should be used to adjust vehicle's heading
bool ModeAuto::use_pilot_yaw(void) const
{
    const bool allow_yaw_option = !option_is_enabled(Option::IgnorePilotYaw);
    const bool rtl_allow_yaw = (_mode == SubMode::RTL) && copter.mode_rtl.use_pilot_yaw();
    const bool landing = _mode == SubMode::LAND;
    return allow_yaw_option || rtl_allow_yaw || landing;
}

/**
 * @brief Set horizontal speed override for Auto mode navigation (external command)
 * 
 * @details Allows external systems (GCS, companion computer, scripts) to override the default horizontal
 *          navigation speed for the current Auto mode operation. This affects waypoint navigation and
 *          other horizontal movement commands. The override persists until mode exit or next wp_start().
 * 
 *          Called By:
 *          - GCS MAVLink DO_CHANGE_SPEED command handler
 *          - Lua scripts via set_speed_xy() binding
 *          - Companion computer via MAVLink
 *          - Mission command processing
 * 
 *          Speed Application:
 *          - copter.wp_nav->set_speed_NE_cms(speed_xy_cms)
 *          - Sets horizontal (North-East plane) speed limit
 *          - Applies to current and future waypoint navigation
 *          - Affects WP, SPLINE_WP, LOITER movements
 *          - Immediate effect on active navigation
 * 
 *          desired_speed_override Storage:
 *          - desired_speed_override.xy = speed_xy_cms * 0.01
 *          - Converts cm/s to m/s for storage
 *          - Remembered for mission resume or next wp_start()
 *          - Applied when initializing waypoint navigation
 *          - Survives between waypoints in same Auto session
 * 
 *          Why Store Override Separately:
 *          - wp_start() re-initializes wp_nav
 *          - Need to reapply speed at each waypoint
 *          - Check: if (is_positive(desired_speed_override.xy))
 *          - If set, apply to new waypoint
 *          - Maintains speed across mission segments
 * 
 *          Speed Units:
 *          - Input: speed_xy_cms in centimeters/second
 *          - wp_nav->set_speed_NE_cms() expects cm/s
 *          - stored as meters/second (* 0.01)
 *          - Consistent with other speed parameters
 * 
 *          Use Case - Slow for Precision:
 *          - Mission requires careful navigation
 *          - GCS sends DO_CHANGE_SPEED(0.5 m/s)
 *          - Vehicle slows to 50 cm/s horizontal
 *          - Precise positioning for inspection
 *          - Later resume normal speed
 * 
 *          Use Case - Speed Up for Transit:
 *          - Long transit between mission areas
 *          - Increase speed to 15 m/s (1500 cm/s)
 *          - Reduce mission time
 *          - Slow down again at target area
 * 
 * @param[in] speed_xy_cms Desired horizontal speed in centimeters per second
 * 
 * @return true (always succeeds)
 * 
 * @note Override persists between waypoints in same Auto mode session
 * @note Cleared on mode exit or explicit reset
 * @note Applied immediately to active navigation
 * 
 * @see set_speed_up_cms() - Vertical climb speed override
 * @see set_speed_down_cms() - Vertical descent speed override
 * @see do_change_speed() - Mission command handler that calls this
 * @see wp_start() - Reapplies override when starting new waypoint
 */
bool ModeAuto::set_speed_xy_cms(float speed_xy_cms)
{
    copter.wp_nav->set_speed_NE_cms(speed_xy_cms);
    desired_speed_override.xy = speed_xy_cms * 0.01;
    return true;
}

/**
 * @brief Set vertical climb speed override for Auto mode (external command)
 * 
 * @details Allows external systems to override the default climb speed for Auto mode vertical movements.
 *          Affects all upward motion during waypoint navigation, takeoff, and altitude changes.
 *          The override persists until mode exit or next wp_start() re-initialization.
 * 
 *          Called By:
 *          - GCS MAVLink DO_CHANGE_SPEED command (SPEED_TYPE_CLIMB_SPEED)
 *          - Lua scripts via set_speed_up() binding
 *          - do_change_speed() mission command handler
 *          - External navigation controllers
 * 
 *          Speed Application:
 *          - copter.wp_nav->set_speed_up_cms(speed_up_cms)
 *          - Sets maximum upward velocity limit
 *          - Affects climb to higher waypoints
 *          - Limits altitude gain rate
 *          - Applied to position controller
 * 
 *          desired_speed_override Storage:
 *          - desired_speed_override.up = speed_up_cms * 0.01
 *          - Converts cm/s to m/s for storage
 *          - Reapplied in wp_start() if is_positive()
 *          - Survives between waypoints
 *          - Persists during mission segments
 * 
 *          Why Override Climb Speed:
 *          - Battery conservation (slow climb)
 *          - Passenger comfort in manned vehicles
 *          - Prevent vortex ring state (too fast descent)
 *          - Precise altitude transitions
 *          - Match operational requirements
 * 
 *          Use Case - Gentle Climb:
 *          - Carrying fragile payload
 *          - Set climb speed to 1 m/s (100 cm/s)
 *          - Smooth, gentle altitude changes
 *          - Reduce stress on payload
 * 
 *          Use Case - Fast Climb:
 *          - Obstacle clearance needed quickly
 *          - Set climb speed to 5 m/s (500 cm/s)
 *          - Rapid altitude gain
 *          - Clear obstacles or terrain
 * 
 * @param[in] speed_up_cms Desired climb speed in centimeters per second
 * 
 * @return true (always succeeds)
 * 
 * @note Override persists between waypoints in same Auto mode session
 * @note Applied immediately to active altitude changes
 * @note Reapplied in wp_start() for each new waypoint
 * 
 * @see set_speed_xy_cms() - Horizontal speed override
 * @see set_speed_down_cms() - Descent speed override
 * @see do_change_speed() - Mission command handler
 */
bool ModeAuto::set_speed_up_cms(float speed_up_cms)
{
    copter.wp_nav->set_speed_up_cms(speed_up_cms);
    desired_speed_override.up = speed_up_cms * 0.01;
    return true;
}

/**
 * @brief Set vertical descent speed override for Auto mode (external command)
 * 
 * @details Allows external systems to override the default descent speed for Auto mode downward movements.
 *          Affects descents during waypoint navigation, landing approaches, and altitude decreases.
 *          Particularly important for landing safety and obstacle clearance during descent.
 * 
 *          Called By:
 *          - GCS MAVLink DO_CHANGE_SPEED command (SPEED_TYPE_DESCENT_SPEED)
 *          - Lua scripts via set_speed_down() binding
 *          - do_change_speed() mission command handler
 *          - Automated landing procedures
 * 
 *          Speed Application:
 *          - copter.wp_nav->set_speed_down_cms(speed_down_cms)
 *          - Sets maximum downward velocity limit
 *          - Affects descent to lower waypoints
 *          - Controls descent rate
 *          - Safety-critical for landing
 * 
 *          desired_speed_override Storage:
 *          - desired_speed_override.down = speed_down_cms * 0.01
 *          - Converts cm/s to m/s for storage
 *          - Reapplied in wp_start() if is_positive()
 *          - Persists between waypoints
 *          - Maintains descent control
 * 
 *          Why Override Descent Speed:
 *          - Slow landing for precision
 *          - Avoid vortex ring state (too fast descent)
 *          - Terrain following at safe rate
 *          - Battery optimization (slower = more efficient)
 *          - Passenger comfort
 * 
 *          Vortex Ring State Avoidance:
 *          - Excessive descent rate causes dangerous condition
 *          - Vehicle descends into its own rotor downwash
 *          - Loss of lift, potential crash
 *          - Limiting descent speed prevents this
 *          - Critical safety feature
 * 
 *          Use Case - Careful Landing Approach:
 *          - Landing in confined area
 *          - Set descent speed to 0.3 m/s (30 cm/s)
 *          - Slow, controlled descent
 *          - Time to assess landing zone
 *          - Abort if unsafe
 * 
 *          Use Case - Rapid Descent:
 *          - Clear airspace quickly
 *          - Set descent to 3 m/s (300 cm/s)
 *          - Fast altitude loss
 *          - Still within safe limits
 *          - Monitor for VRS
 * 
 *          Landing Speed Interaction:
 *          - LAND_SPEED parameter is default landing descent
 *          - This override can slow it further
 *          - Applied during NAV_LAND execution
 *          - Affects entire descent profile
 *          - Landing detector still functions normally
 * 
 * @param[in] speed_down_cms Desired descent speed in centimeters per second (positive value)
 * 
 * @return true (always succeeds)
 * 
 * @note Override persists between waypoints in same Auto mode session
 * @note Critical for landing safety and vortex ring state avoidance
 * @note Applied immediately to active descents
 * @note Reapplied in wp_start() for each new waypoint
 * 
 * @see set_speed_xy_cms() - Horizontal speed override
 * @see set_speed_up_cms() - Climb speed override
 * @see do_change_speed() - Mission command handler
 * @see land_run() - Landing execution affected by descent speed
 */
bool ModeAuto::set_speed_down_cms(float speed_down_cms)
{
    copter.wp_nav->set_speed_down_cms(speed_down_cms);
    desired_speed_override.down = speed_down_cms * 0.01;
    return true;
}

/**
 * @brief Dispatch and initiate execution of a mission command
 * 
 * @details Called by AP_Mission library when starting a new mission command. Routes command
 *          to appropriate handler based on MAVLink command ID. Supports navigation commands
 *          (NAV_*), conditional commands (CONDITION_*), and immediate commands (DO_*).
 * 
 *          Mission Command Categories:
 * 
 *          Navigation Commands (NAV_*): Define vehicle movement and waypoints
 *          - MAV_CMD_NAV_WAYPOINT (16): Fly to waypoint with optional delay
 *          - MAV_CMD_NAV_LOITER_UNLIM (17): Loiter indefinitely at location
 *          - MAV_CMD_NAV_LOITER_TURNS (18): Circle location N times
 *          - MAV_CMD_NAV_LOITER_TIME (19): Loiter for specified duration
 *          - MAV_CMD_NAV_RETURN_TO_LAUNCH (20): Execute RTL sequence
 *          - MAV_CMD_NAV_LAND (21): Land at location or current position
 *          - MAV_CMD_NAV_TAKEOFF (22): Takeoff to specified altitude
 *          - MAV_CMD_NAV_SPLINE_WAYPOINT (82): Fly smooth spline path through waypoint
 *          - MAV_CMD_NAV_GUIDED_ENABLE (92): Enable external guidance control
 *          - MAV_CMD_NAV_DELAY (93): Delay before next navigation command
 *          - MAV_CMD_NAV_PAYLOAD_PLACE (94): Automated payload placement
 *          - MAV_CMD_NAV_LOITER_TO_ALT: Loiter while climbing/descending to altitude
 *          - MAV_CMD_NAV_VTOL_TAKEOFF: VTOL-specific takeoff
 *          - MAV_CMD_NAV_VTOL_LAND: VTOL-specific landing
 *          - MAV_CMD_NAV_SCRIPT_TIME: Lua scripting control integration
 *          - MAV_CMD_NAV_ATTITUDE_TIME: Hold attitude for duration
 * 
 *          Conditional Commands (CONDITION_*): Must complete before next NAV command
 *          - MAV_CMD_CONDITION_DELAY (112): Wait specified seconds
 *          - MAV_CMD_CONDITION_DISTANCE (114): Wait until within distance of waypoint
 *          - MAV_CMD_CONDITION_YAW (115): Rotate to specified heading
 * 
 *          Do Commands (DO_*): Execute immediately without blocking mission progression
 *          - MAV_CMD_DO_CHANGE_SPEED (178): Change speed parameters
 *          - MAV_CMD_DO_SET_HOME (179): Set home position
 *          - MAV_CMD_DO_SET_ROI (201): Point camera at region of interest
 *          - MAV_CMD_DO_SET_ROI_LOCATION (195): ROI at specific location
 *          - MAV_CMD_DO_SET_ROI_NONE (197): Clear ROI
 *          - MAV_CMD_DO_MOUNT_CONTROL (205): Direct camera mount control
 *          - MAV_CMD_DO_GUIDED_LIMITS (220): Set guided mode limits
 *          - MAV_CMD_DO_WINCH: Control winch operations
 *          - MAV_CMD_DO_LAND_START: Mark start of landing sequence (for auto-RTL)
 *          - MAV_CMD_DO_RETURN_PATH_START: Mark start of return path (for auto-RTL)
 * 
 *          Command Execution Flow:
 *          1. AP_Mission library calls this function with next command
 *          2. Switch statement routes to appropriate do_* handler function
 *          3. Handler initializes state/controllers for command type
 *          4. Function returns true if command accepted, false to skip
 *          5. Mission library begins calling verify_command() to check completion
 * 
 *          Handler Functions (examples):
 *          - do_nav_wp(): Initialize waypoint navigation
 *          - do_takeoff(): Setup takeoff controller
 *          - do_land(): Initialize landing sequence
 *          - do_circle(): Configure circle navigation
 *          - do_spline_wp(): Setup spline path following
 *          - do_change_speed(): Update speed parameters
 *          - do_set_home(): Update home position
 *          - do_roi(): Configure region of interest tracking
 * 
 *          Feature Flags:
 *          - Some commands conditional on feature compilation (AC_NAV_GUIDED, AP_SCRIPTING_ENABLED, etc.)
 *          - Feature-disabled commands will fall through to default (skip)
 * 
 *          Error Handling:
 *          - Unsupported/unrecognized commands return false
 *          - Mission library will skip to next command
 *          - Terrain data failures trigger failsafe_terrain_on_event()
 * 
 * @param[in] cmd Mission command structure from AP_Mission library containing:
 *                - id: MAVLink command ID (MAV_CMD_*)
 *                - content: Command-specific parameters (location, speed, etc.)
 *                - p1: Multi-purpose parameter 1 (usage varies by command)
 *                - index: Command index in mission for logging/reporting
 * 
 * @return true if command was recognized and initialized successfully
 * @return false if command not supported (mission will try next command)
 * 
 * @note Called by AP_Mission::update() at mission execution rate
 * @note NAV commands are sequential - only one active at a time
 * @note CONDITION commands gate progression to next NAV command
 * @note DO commands execute immediately and don't block mission
 * @note Command completion checked by verify_command() callback
 * 
 * @see verify_command() for command completion detection
 * @see AP_Mission::Mission_Command for command structure details
 * @see MAVLink command definitions in common.xml
 */
bool ModeAuto::start_command(const AP_Mission::Mission_Command& cmd)
{
    switch(cmd.id) {

    ///
    /// navigation commands
    ///
    case MAV_CMD_NAV_VTOL_TAKEOFF:
    case MAV_CMD_NAV_TAKEOFF:                   // 22
        do_takeoff(cmd);
        break;

    case MAV_CMD_NAV_WAYPOINT:                  // 16  Navigate to Waypoint
        do_nav_wp(cmd);
        break;

    case MAV_CMD_NAV_VTOL_LAND:
    case MAV_CMD_NAV_LAND:              // 21 LAND to Waypoint
        do_land(cmd);
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

    case MAV_CMD_NAV_LOITER_TO_ALT:
        do_loiter_to_alt(cmd);
        break;

    case MAV_CMD_NAV_RETURN_TO_LAUNCH:             //20
        do_RTL();
        break;

    case MAV_CMD_NAV_SPLINE_WAYPOINT:           // 82  Navigate to Waypoint using spline
        do_spline_wp(cmd);
        break;

#if AC_NAV_GUIDED
    case MAV_CMD_NAV_GUIDED_ENABLE:             // 92  accept navigation commands from external nav computer
        do_nav_guided_enable(cmd);
        break;
#endif

    case MAV_CMD_NAV_DELAY:                    // 93 Delay the next navigation command
        do_nav_delay(cmd);
        break;

#if AP_MISSION_NAV_PAYLOAD_PLACE_ENABLED && AC_PAYLOAD_PLACE_ENABLED
    case MAV_CMD_NAV_PAYLOAD_PLACE:              // 94 place at Waypoint
        do_payload_place(cmd);
        break;
#endif

#if AP_SCRIPTING_ENABLED
    case MAV_CMD_NAV_SCRIPT_TIME:
        do_nav_script_time(cmd);
        break;
#endif

    case MAV_CMD_NAV_ATTITUDE_TIME:
        do_nav_attitude_time(cmd);
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
        // point the copter and camera at a region of interest (ROI)
        // ROI_NONE can be handled by the regular ROI handler because lat, lon, alt are always zero
        do_roi(cmd);
        break;

#if HAL_MOUNT_ENABLED
    case MAV_CMD_DO_MOUNT_CONTROL:          // 205
        // point the camera to a specified angle
        do_mount_control(cmd);
        break;
#endif

#if AC_NAV_GUIDED
    case MAV_CMD_DO_GUIDED_LIMITS:                      // 220  accept guided mode limits
        do_guided_limits(cmd);
        break;
#endif

#if AP_WINCH_ENABLED
    case MAV_CMD_DO_WINCH:                             // Mission command to control winch
        do_winch(cmd);
        break;
#endif

    case MAV_CMD_DO_RETURN_PATH_START:
    case MAV_CMD_DO_LAND_START:
        break;

    default:
        // unable to use the command, allow the vehicle to try the next command
        return false;
    }

    // always return success
    return true;
}

/**
 * @brief Handle mission completion - transition to safe end state
 * 
 * @details Called by AP_Mission library when the mission reaches its final command or encounters
 *          a condition that terminates mission execution. Responsible for transitioning vehicle
 *          to a safe state: either loiter in air, land, or disarm if already landed. Provides
 *          notification of mission completion and ensures vehicle doesn't fall out of the sky.
 * 
 *          Called By:
 *          - AP_Mission::update() when mission reaches end
 *          - Mission abort conditions
 *          - Mission complete (all commands executed)
 *          - Automatic on last command verify completion
 * 
 *          Mission Complete Notification:
 *          - AP_Notify::events.mission_complete = 1
 *          - Triggers notification system
 *          - LED pattern for mission complete
 *          - Audible tone on buzzer
 *          - GCS notification sent
 * 
 *          Two Execution Paths:
 *          1. Vehicle In Air (!copter.ap.land_complete)
 *             - Try to enter LOITER mode
 *             - If loiter fails, enter LAND mode
 *             - Keeps vehicle safe in air
 *          2. Vehicle On Ground (copter.ap.land_complete)
 *             - Automatically disarm motors
 *             - Safe to disarm when on ground
 *             - Completes mission sequence
 * 
 *          Why Try Loiter First:
 *          - Keep vehicle at current position
 *          - Allows pilot to take over control
 *          - Better than automatic landing if position is bad
 *          - Pilot can assess situation
 *          - Common for aerial photography missions
 * 
 *          Loiter Failure Conditions:
 *          - !copter.position_ok(): No valid position estimate
 *          - EKF not healthy
 *          - GPS lost
 *          - Terrain database unavailable (terrain mode)
 *          - Position error too high
 * 
 *          loiter_start() Return Value:
 *          - true: Successfully initialized loiter
 *          - false: Cannot loiter (position bad)
 *          - If false, must land immediately
 * 
 *          Fallback to LAND Mode:
 *          - If loiter fails, LAND is safer than nothing
 *          - set_mode(Mode::Number::LAND, ModeReason::MISSION_END)
 *          - LAND mode can handle degraded position
 *          - Uses barometer, rangefinder, land detector
 *          - Descends at LAND_SPEED
 * 
 *          LAND Mode Safety:
 *          - Does not require position estimate
 *          - Descends vertically from current position
 *          - Uses land detector to sense touchdown
 *          - Automatically disarms after landing
 *          - More robust than loiter in degraded state
 * 
 *          On-Ground Disarm:
 *          - copter.arming.disarm(AP_Arming::Method::MISSIONEXIT)
 *          - Automatic disarm after mission
 *          - AP_Arming::Method::MISSIONEXIT reason logged
 *          - Motors stop spinning
 *          - Completes mission lifecycle
 * 
 *          Why Automatic Disarm:
 *          - Mission complete, no further action needed
 *          - Safe on ground (land_complete verified)
 *          - Prevents accidental takeoff
 *          - Saves battery
 *          - Clean mission completion
 * 
 *          land_complete Flag:
 *          - copter.ap.land_complete
 *          - Set by land detector
 *          - true when vehicle on ground, settled
 *          - Motors at low throttle for several seconds
 *          - Reliable ground detection
 * 
 *          Use Case - Aerial Survey:
 *          - Mission completes last waypoint
 *          - Vehicle still in air
 *          - Enters LOITER at final position
 *          - Pilot can manually land or take control
 *          - Provides opportunity for visual check
 * 
 *          Use Case - Delivery Mission:
 *          - Mission includes landing at destination
 *          - Last command is NAV_LAND
 *          - Vehicle already on ground when mission completes
 *          - Automatically disarms
 *          - Ready for next mission load
 * 
 *          Use Case - Position Loss:
 *          - Mission completes but GPS degraded
 *          - loiter_start() fails (no position)
 *          - Automatically enters LAND mode
 *          - Descends safely at current location
 *          - Better than crash
 * 
 *          ModeReason::MISSION_END:
 *          - Logged reason for mode change
 *          - Distinguishes from other mode changes
 *          - Appears in logs
 *          - Helps with post-flight analysis
 * 
 *          Flight Log Entry:
 *          - Mode change logged if entering LAND
 *          - Disarm event logged if on ground
 *          - Mission complete event
 *          - Helps with mission analysis
 * 
 *          Parameter Interaction:
 *          - MIS_DONE_BEHAVE: Can override behavior
 *          - If set, may loiter or land based on parameter
 *          - This function implements default behavior
 * 
 * @note Always provides audible/visual mission complete notification
 * @note Tries loiter first if in air, falls back to land if position bad
 * @note Automatically disarms if vehicle is already on ground
 * @note Mission complete tone/LED pattern helps pilot recognize completion
 * 
 * @see loiter_start() - Attempts to enter loiter mode
 * @see copter.ap.land_complete - Ground detection flag
 * @see copter.arming.disarm() - Motor disarm function
 */
// exit_mission - function that is called once the mission completes
void ModeAuto::exit_mission()
{
    // play a tone
    AP_Notify::events.mission_complete = 1;
    // if we are not on the ground switch to loiter or land
    if (!copter.ap.land_complete) {
        // try to enter loiter but if that fails land
        if (!loiter_start()) {
            set_mode(Mode::Number::LAND, ModeReason::MISSION_END);
        }
    } else {
        // if we've landed it's safe to disarm
        copter.arming.disarm(AP_Arming::Method::MISSIONEXIT);
    }
}

/**
 * @brief Process guided mode waypoint commands when already in Guided mode
 * 
 * @details This function handles mission commands that should be executed when the vehicle is in
 *          Guided mode (not Auto). It's called by the mission library when the vehicle is being
 *          externally controlled but mission commands may still be sent. Primary use is for
 *          ground stations to send waypoints while in Guided mode.
 * 
 *          Context:
 *          - Vehicle currently in Guided flight mode
 *          - Mission library attempting to execute command
 *          - Command forwarded to Guided mode handler
 *          - Not normal Auto mode execution path
 * 
 *          Supported Commands:
 *          1. MAV_CMD_NAV_WAYPOINT: Navigate to waypoint
 *          2. MAV_CMD_CONDITION_YAW: Change heading
 *          
 *          Rejected Commands:
 *          - All other mission commands return false
 *          - Must be in guided mode to process
 *          - Safety check prevents unexpected execution
 * 
 *          Waypoint Handling (MAV_CMD_NAV_WAYPOINT):
 *          - Extract location from command
 *          - Forward to copter.mode_guided.set_destination()
 *          - Guided mode performs actual navigation
 *          - Return guided mode's acceptance status
 * 
 *          Yaw Handling (MAV_CMD_CONDITION_YAW):
 *          - Call do_yaw(cmd) directly
 *          - Sets yaw target for auto_yaw controller
 *          - Works in guided mode
 *          - Always returns true (command accepted)
 * 
 *          Why Check in_guided_mode():
 *          - Safety gate: Only process in correct mode
 *          - Prevents accidental execution in Auto
 *          - Clear separation of mode responsibilities
 *          - Predictable behavior
 * 
 * @param[in] cmd Mission command to process
 * 
 * @return true if command was accepted and processed
 * @return false if not in guided mode or command not supported
 * 
 * @note Only processes commands when vehicle is in Guided mode
 * @note Limited subset of commands supported compared to Auto mode
 * 
 * @see copter.mode_guided.set_destination() - Guided waypoint navigation
 * @see do_yaw() - Yaw command processing
 */
// do_guided - start guided mode
bool ModeAuto::do_guided(const AP_Mission::Mission_Command& cmd)
{
    // only process guided waypoint if we are in guided mode
    if (!copter.flightmode->in_guided_mode()) {
        return false;
    }

    // switch to handle different commands
    switch (cmd.id) {

        case MAV_CMD_NAV_WAYPOINT:
        {
            // set wp_nav's destination
            Location dest(cmd.content.location);
            return copter.mode_guided.set_destination(dest);
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
 * @brief Get horizontal distance to current Auto mode navigation target in meters
 * 
 * @details Returns the horizontal (2D) distance from the vehicle's current position to the
 *          active navigation target, regardless of Auto submode. Used for GCS telemetry,
 *          mission monitoring, conditional command checking, and pilot situational awareness.
 * 
 *          Distance Source by Submode:
 *          - SubMode::CIRCLE: copter.circle_nav->get_distance_to_target_cm()
 *            Distance to circle center point
 *          - SubMode::WP: wp_nav->get_wp_distance_to_destination_cm()
 *            Distance to waypoint destination
 *          - SubMode::CIRCLE_MOVE_TO_EDGE: wp_nav distance (moving to circle edge)
 *          - Default (all other): wp_nav distance
 * 
 *          Circle Mode Special Case:
 *          - While circling, "target" is circle center
 *          - Distance shows how far from center
 *          - Useful for monitoring circle accuracy
 *          - Different from "completion" distance
 * 
 *          Waypoint Navigation:
 *          - Most common case
 *          - Straight-line distance to destination
 *          - 2D horizontal distance (ignores altitude difference)
 *          - Updates continuously as vehicle moves
 * 
 *          Distance Calculation:
 *          - Computed from current position estimate
 *          - Uses EKF position in NE frame
 *          - Pythagorean distance: sqrt(dN^2 + dE^2)
 *          - Ignores vertical component
 * 
 *          Unit Conversion:
 *          - Internal: centimeters (cm)
 *          - Returned: meters (m)
 *          - Multiply by 0.01f for conversion
 *          - Consistent with MAVLink units
 * 
 *          Used For:
 *          - GCS distance-to-waypoint display
 *          - MAVLink NAV_CONTROLLER_OUTPUT message
 *          - CONDITION_DISTANCE mission command checking
 *          - Pilot OSD distance display
 *          - Mission progress monitoring
 * 
 *          CONDITION_DISTANCE Command:
 *          - "Wait until within X meters of target"
 *          - Calls this function to check distance
 *          - Mission pauses until condition met
 *          - verify_within_distance() uses this
 * 
 *          GCS Telemetry:
 *          - Sent in NAV_CONTROLLER_OUTPUT
 *          - Updates at 10 Hz typically
 *          - Shows pilot mission progress
 *          - Helps with manual intervention decisions
 * 
 * @return Horizontal distance to navigation target in meters
 * 
 * @note Returns 2D horizontal distance only (altitude not included)
 * @note Distance is to circle center when in CIRCLE submode
 * @note Distance is to active waypoint in most other submodes
 * 
 * @see wp_bearing_deg() - Bearing to same target
 * @see verify_within_distance() - Uses this for CONDITION_DISTANCE
 * @see get_wp() - Get target location coordinates
 */
float ModeAuto::wp_distance_m() const
{
    switch (_mode) {
    case SubMode::CIRCLE:
        return copter.circle_nav->get_distance_to_target_cm() * 0.01f;
    case SubMode::WP:
    case SubMode::CIRCLE_MOVE_TO_EDGE:
    default:
        return wp_nav->get_wp_distance_to_destination_cm() * 0.01f;
    }
}

/**
 * @brief Get bearing to current Auto mode navigation target in degrees
 * 
 * @details Returns the horizontal bearing from the vehicle's current position to the active
 *          navigation target. Bearing is compass heading (0-360°) that the vehicle should fly
 *          to reach the target. Used for GCS display, navigation monitoring, and pilot awareness.
 * 
 *          Bearing Source by Submode:
 *          - SubMode::CIRCLE: copter.circle_nav->get_bearing_to_target_rad()
 *            Bearing to circle center point
 *          - SubMode::WP: wp_nav->get_wp_bearing_to_destination_rad()
 *            Bearing to waypoint destination
 *          - SubMode::CIRCLE_MOVE_TO_EDGE: wp_nav bearing (moving to circle edge)
 *          - Default (all other): wp_nav bearing
 * 
 *          Bearing Definition:
 *          - North = 0° (or 360°)
 *          - East = 90°
 *          - South = 180°
 *          - West = 270°
 *          - Clockwise from North
 *          - True bearing (not magnetic)
 * 
 *          Calculation:
 *          - atan2(dE, dN) in radians
 *          - Converted to degrees
 *          - Based on NED (North-East-Down) frame
 *          - Uses EKF position estimate
 * 
 *          Unit Conversion:
 *          - Internal: radians
 *          - Returned: degrees
 *          - degrees() macro converts
 *          - Range: 0-360 degrees
 * 
 *          Circle Mode Bearing:
 *          - Points to circle center
 *          - While circling, constantly changing
 *          - Useful for monitoring circle centering
 *          - Not the flight direction (that's tangential)
 * 
 *          Waypoint Mode Bearing:
 *          - Straight-line bearing to destination
 *          - Ideal heading to fly
 *          - May differ from actual heading due to wind
 *          - L1 controller follows curved path, not direct
 * 
 *          Used For:
 *          - GCS compass rose display (target direction)
 *          - MAVLink NAV_CONTROLLER_OUTPUT message
 *          - Pilot OSD target bearing
 *          - Crosstrack error calculation
 *          - Navigation quality monitoring
 * 
 *          Bearing vs Heading:
 *          - Bearing: Direction TO target (this function)
 *          - Heading: Direction vehicle is pointing
 *          - May differ due to:
 *            * Wind drift
 *            * Yaw mode (ROI, hold, etc.)
 *            * Path following curvature
 * 
 *          GCS Display:
 *          - Shows arrow pointing to target
 *          - Helps pilot understand flight path
 *          - Combined with distance shows progress
 *          - Critical for manual intervention
 * 
 * @return Bearing to navigation target in degrees (0-360°, North=0°, clockwise)
 * 
 * @note Bearing is true (not magnetic) in NED frame
 * @note Bearing to circle center when in CIRCLE submode
 * @note Bearing constantly updates as vehicle and/or target moves
 * 
 * @see wp_distance_m() - Distance to same target
 * @see get_wp() - Get target location coordinates
 */
float ModeAuto::wp_bearing_deg() const
{
    switch (_mode) {
    case SubMode::CIRCLE:
        return degrees(copter.circle_nav->get_bearing_to_target_rad());
    case SubMode::WP:
    case SubMode::CIRCLE_MOVE_TO_EDGE:
    default:
        return degrees(wp_nav->get_wp_bearing_to_destination_rad());
    }
}

/**
 * @brief Retrieve current Auto mode navigation destination as Location object
 * 
 * @details Populates the provided Location reference with the current navigation target based on
 *          the active Auto submode. Used by GCS for mission monitoring, by obstacle avoidance
 *          to predict path, and by failsafe systems to determine safe alternatives.
 * 
 *          Location Retrieval by Submode:
 *          - SubMode::NAVGUIDED: copter.mode_guided.get_wp(destination)
 *            External navigation controller's target
 *          - SubMode::WP: wp_nav->get_oa_wp_destination(destination)
 *            Waypoint target (may be adjusted by obstacle avoidance)
 *          - SubMode::RTL: copter.mode_rtl.get_wp(destination)
 *            RTL mode's current phase target
 *          - All other submodes: return false (no specific target location)
 * 
 *          SubMode::NAVGUIDED Target:
 *          - Auto mode delegating to external navigation
 *          - Target from MAVLink, companion computer, or script
 *          - Dynamically updated by external system
 *          - Guided mode manages actual target
 * 
 *          SubMode::WP Target:
 *          - Standard waypoint navigation
 *          - wp_nav->get_oa_wp_destination() provides target
 *          - OA = Obstacle Avoidance adjusted if active
 *          - May differ from raw mission waypoint
 * 
 *          get_oa_wp_destination() Details:
 *          - Returns obstacle-avoidance-adjusted target
 *          - If OA inactive, returns raw waypoint
 *          - If OA active, returns avoidance target
 *          - Transparent to caller
 * 
 *          SubMode::RTL Target:
 *          - RTL has multiple phases (climb, return, descend, land)
 *          - Each phase has different target
 *          - Delegates to mode_rtl.get_wp()
 *          - Returns current phase target
 * 
 *          RTL Phase Targets:
 *          - Climb phase: Home location at RTL altitude
 *          - Return phase: Home horizontal position
 *          - Descend/Land phase: Home location at ground
 *          - Provides consistent interface
 * 
 *          Return Value Meaning:
 *          - true: destination populated with valid target
 *          - false: No specific target location for current submode
 * 
 *          Submodes Returning False:
 *          - LOITER: Already at target (loitering)
 *          - CIRCLE: Orbiting center, not going to it
 *          - LAND: Descending at current position
 *          - TAKEOFF: Climbing from current position
 *          - NAV_ATTITUDE_TIME: Maintaining attitude, no location target
 * 
 *          Location Object Contents:
 *          - Latitude (int32_t, degrees * 1e7)
 *          - Longitude (int32_t, degrees * 1e7)
 *          - Altitude (int32_t, centimeters or meters depending on frame)
 *          - AltFrame: ABOVE_HOME, ABOVE_ORIGIN, ABOVE_TERRAIN
 * 
 *          Used By:
 *          - GCS for mission path preview
 *          - Obstacle avoidance path planning
 *          - Failsafe terrain checking
 *          - Mission monitoring systems
 *          - Data logging and analysis
 * 
 *          Obstacle Avoidance Integration:
 *          - get_oa_wp_destination() critical
 *          - Returns adjusted target if OA active
 *          - Allows GCS to see actual flight path
 *          - Shows OA behavior transparently
 * 
 * @param[out] destination Location object to populate with target coordinates
 * 
 * @return true if destination was populated with valid target location
 * @return false if current submode has no specific destination location
 * 
 * @note NAVGUIDED delegates to Guided mode's target
 * @note WP submode returns obstacle-avoidance-adjusted target if OA active
 * @note RTL submode returns phase-specific target (home, climb point, etc.)
 * @note Returns false for submodes without specific location targets (LOITER, CIRCLE, LAND, etc.)
 * 
 * @see wp_distance_m() - Distance to target returned by this function
 * @see wp_bearing_deg() - Bearing to target returned by this function
 */
bool ModeAuto::get_wp(Location& destination) const
{
    switch (_mode) {
    case SubMode::NAVGUIDED:
        return copter.mode_guided.get_wp(destination);
    case SubMode::WP:
        return wp_nav->get_oa_wp_destination(destination);
    case SubMode::RTL:
        return copter.mode_rtl.get_wp(destination);
    default:
        return false;
    }
}

/*******************************************************************************
Verify command Handlers

Each type of mission element has a "verify" operation. The verify
operation returns true when the mission element has completed and we
should move onto the next mission element.
Return true if we do not recognize the command so that we move on to the next command
*******************************************************************************/

/**
 * @brief Check if currently executing mission command has completed
 * 
 * @details Called repeatedly by AP_Mission library at 10Hz or higher to monitor command
 *          completion status. Routes to appropriate verify_* function based on command ID.
 *          When returns true, mission advances to next command.
 * 
 *          Verification Pattern:
 *          1. Safety check: Return false if not in Auto mode (prevents unintended actions)
 *          2. Route to command-specific verify_* function via switch statement
 *          3. verify_* function checks completion criteria (position reached, time elapsed, etc.)
 *          4. If complete, send mission item reached message to GCS and return true
 *          5. Mission library advances to next command when true returned
 * 
 *          Navigation Command Verification:
 *          - verify_takeoff(): Check if reached target altitude and auto_takeoff.complete flag
 *          - verify_nav_wp(): Check if reached waypoint and delay timer expired
 *          - verify_land(): Check if landed (land_complete && motors at GROUND_IDLE)
 *          - verify_RTL(): Check if RTL state machine completed and landed
 *          - verify_circle(): Check if completed specified number of turns
 *          - verify_spline_wp(): Check if reached spline waypoint destination
 *          - verify_loiter_time(): Check if loitered for required duration
 *          - verify_loiter_unlimited(): Always false (never completes)
 *          - verify_loiter_to_alt(): Check if reached target altitude at loiter position
 *          - verify_nav_guided_enable(): Check time/position limits or if guidance disabled
 *          - verify_nav_delay(): Check if delay duration elapsed
 *          - verify_nav_script_time(): Check if Lua script signaled completion or timeout
 *          - verify_nav_attitude_time(): Check if attitude hold duration elapsed
 * 
 *          Conditional Command Verification:
 *          - verify_wait_delay(): Check if delay timer expired
 *          - verify_within_distance(): Check if within specified distance of waypoint
 *          - verify_yaw(): Check if reached target heading
 * 
 *          Do Command Verification:
 *          - All DO commands return true immediately (execute once, don't block mission)
 *          - MAV_CMD_DO_CHANGE_SPEED, DO_SET_HOME, DO_SET_ROI, DO_MOUNT_CONTROL, etc.
 *          - These commands take effect immediately and don't require completion monitoring
 * 
 *          Command Completion Criteria Examples:
 *          - Position-based: wp_nav->reached_wp_destination()
 *          - Time-based: (millis() - start_time) >= duration
 *          - State-based: mode_rtl.state_complete()
 *          - Sensor-based: land_complete flag from land detector
 *          - External: nav_scripting.done flag from Lua script
 * 
 *          GCS Notification:
 *          - When command completes, sends MISSION_ITEM_REACHED message to ground station
 *          - Provides mission progress feedback to operator
 *          - gcs().send_mission_item_reached_message(cmd.index)
 * 
 *          Unknown Command Handling:
 *          - Unrecognized commands return true (skip and advance)
 *          - Warning message sent to GCS: "Skipping invalid cmd #N"
 *          - Prevents mission from getting stuck on unsupported commands
 * 
 *          Mode Check Safety:
 *          - Always verifies copter.flightmode == &copter.mode_auto before processing
 *          - Prevents AP_Mission from triggering actions if pilot switched modes
 *          - Returns false if not in Auto mode (command not verified)
 * 
 * @param[in] cmd Mission command being verified (from AP_Mission current command)
 *                Contains command ID, parameters, and index for status reporting
 * 
 * @return true if command has completed and mission should advance to next command
 * @return false if command still executing or not in Auto mode
 * 
 * @note Called at 10Hz or higher by AP_Mission::update() during command execution
 * @note Must return true for mission to progress to next command
 * @note NAV commands typically take time to complete (moving to position)
 * @note CONDITION commands gate next NAV command until verified true
 * @note DO commands return true immediately (single execution)
 * @warning Returning false when not in Auto mode prevents unintended mission actions
 * 
 * @see start_command() for command initialization
 * @see AP_Mission::update() for mission state machine
 * @see Individual verify_* functions for specific completion criteria
 */
bool ModeAuto::verify_command(const AP_Mission::Mission_Command& cmd)
{
    if (copter.flightmode != &copter.mode_auto) {
        return false;
    }

    bool cmd_complete = false;

    switch (cmd.id) {
    //
    // navigation commands
    //
    case MAV_CMD_NAV_VTOL_TAKEOFF:
    case MAV_CMD_NAV_TAKEOFF:
        cmd_complete = verify_takeoff();
        break;

    case MAV_CMD_NAV_WAYPOINT:
        cmd_complete = verify_nav_wp(cmd);
        break;

    case MAV_CMD_NAV_VTOL_LAND:
    case MAV_CMD_NAV_LAND:
        cmd_complete = verify_land();
        break;

#if AP_MISSION_NAV_PAYLOAD_PLACE_ENABLED && AC_PAYLOAD_PLACE_ENABLED
    case MAV_CMD_NAV_PAYLOAD_PLACE:
        cmd_complete = payload_place.verify();
        break;
#endif

    case MAV_CMD_NAV_LOITER_UNLIM:
        cmd_complete = verify_loiter_unlimited();
        break;

    case MAV_CMD_NAV_LOITER_TURNS:
        cmd_complete = verify_circle(cmd);
        break;

    case MAV_CMD_NAV_LOITER_TIME:
        cmd_complete = verify_loiter_time(cmd);
        break;

    case MAV_CMD_NAV_LOITER_TO_ALT:
        return verify_loiter_to_alt();

    case MAV_CMD_NAV_RETURN_TO_LAUNCH:
        cmd_complete = verify_RTL();
        break;

    case MAV_CMD_NAV_SPLINE_WAYPOINT:
        cmd_complete = verify_spline_wp(cmd);
        break;

#if AC_NAV_GUIDED
    case MAV_CMD_NAV_GUIDED_ENABLE:
        cmd_complete = verify_nav_guided_enable(cmd);
        break;
#endif

     case MAV_CMD_NAV_DELAY:
        cmd_complete = verify_nav_delay(cmd);
        break;

#if AP_SCRIPTING_ENABLED
    case MAV_CMD_NAV_SCRIPT_TIME:
        cmd_complete = verify_nav_script_time();
        break;
#endif

    case MAV_CMD_NAV_ATTITUDE_TIME:
        cmd_complete = verify_nav_attitude_time(cmd);
        break;

    ///
    /// conditional commands
    ///
    case MAV_CMD_CONDITION_DELAY:
        cmd_complete = verify_wait_delay();
        break;

    case MAV_CMD_CONDITION_DISTANCE:
        cmd_complete = verify_within_distance();
        break;

    case MAV_CMD_CONDITION_YAW:
        cmd_complete = verify_yaw();
        break;

    // do commands (always return true)
    case MAV_CMD_DO_CHANGE_SPEED:
    case MAV_CMD_DO_SET_HOME:
    case MAV_CMD_DO_SET_ROI_LOCATION:
    case MAV_CMD_DO_SET_ROI_NONE:
    case MAV_CMD_DO_SET_ROI:
#if HAL_MOUNT_ENABLED
    case MAV_CMD_DO_MOUNT_CONTROL:
#endif
#if AC_NAV_GUIDED
    case MAV_CMD_DO_GUIDED_LIMITS:
#endif
#if AP_FENCE_ENABLED
    case MAV_CMD_DO_FENCE_ENABLE:
#endif
#if AP_WINCH_ENABLED
    case MAV_CMD_DO_WINCH:
#endif
    case MAV_CMD_DO_RETURN_PATH_START:
    case MAV_CMD_DO_LAND_START:
        cmd_complete = true;
        break;

    default:
        // error message
        gcs().send_text(MAV_SEVERITY_WARNING,"Skipping invalid cmd #%i",cmd.id);
        // return true if we do not recognize the command so that we move on to the next command
        cmd_complete = true;
        break;
    }


    // send message to GCS
    if (cmd_complete) {
        gcs().send_mission_item_reached_message(cmd.index);
    }

    return cmd_complete;
}

/**
 * @brief Execute takeoff sequence in Auto mode
 * 
 * @details Runs vertical takeoff to target altitude specified in NAV_TAKEOFF mission command.
 *          Delegates actual takeoff control to the auto_takeoff helper object which manages
 *          altitude ramping, minimum altitude enforcement, and completion detection.
 * 
 *          Takeoff Sequence:
 *          1. Optionally auto-arms throttle if ALLOW_TAKEOFF_WITHOUT_RAISING_THROTTLE option enabled
 *          2. Calls auto_takeoff.run() which handles:
 *             - Vertical position control to target altitude
 *             - Enforcement of WP_NAVALT_MIN parameter (minimum altitude for waypoint nav)
 *             - Terrain-relative altitude support if target frame is ABOVE_TERRAIN
 *             - Completion detection when altitude reached
 * 
 *          The takeoff runs until auto_takeoff.complete flag is set by the takeoff controller,
 *          which is checked by verify_takeoff(). Once complete, mission advances to next command.
 * 
 *          Auto-Arming Feature:
 *          - If AUTO_OPTIONS bit for AllowTakeOffWithoutRaisingThrottle is set, automatically
 *            sets armed state without requiring pilot to raise throttle
 *          - Useful for automated missions but bypasses normal disarm-on-ground safety check
 *          - Use with caution - removes pilot confirmation step before takeoff
 * 
 * @note Called at 100Hz or higher from run() when in TAKEOFF submode
 * @note Yaw is held at heading from init() - typically current heading
 * @note Position controller I-term cleared at takeoff start for clean altitude control
 * 
 * @see takeoff_start() for takeoff initialization and altitude target calculation
 * @see verify_takeoff() for completion detection
 * @see auto_takeoff.run() in AP_AutoTakeoff for detailed takeoff control
 */
void ModeAuto::takeoff_run()
{
    // if the user doesn't want to raise the throttle we can set it automatically
    // note that this can defeat the disarm check on takeoff
    if (option_is_enabled(Option::AllowTakeOffWithoutRaisingThrottle)) {
        copter.set_auto_armed(true);
    }
    auto_takeoff.run();
}

/**
 * @brief Execute waypoint navigation in Auto mode
 * 
 * @details Runs waypoint navigation controller to fly vehicle to target location specified
 *          in NAV_WAYPOINT or NAV_SPLINE_WAYPOINT mission commands. Handles both straight-line
 *          waypoint navigation and smooth spline paths. Also used for CIRCLE_MOVE_TO_EDGE
 *          submode when moving to the edge of a circle before starting circular loiter.
 * 
 *          Control Flow:
 *          1. Safety check: If disarmed or landed, safe ground handling and exit
 *          2. Set motor spool state to THROTTLE_UNLIMITED for full control authority
 *          3. Update waypoint navigation controller (wp_nav->update_wpnav())
 *             - Calculates desired position, velocity, and acceleration
 *             - Handles obstacle avoidance if enabled
 *             - Manages speed profiles and corner cutting
 *             - Supports terrain following with altitude adjustments
 *          4. Update vertical position controller (pos_control->update_U_controller())
 *             - Converts altitude targets to throttle commands
 *             - Applies feed-forward from velocity/acceleration targets
 *          5. Command attitude controller with thrust vector and auto yaw
 *             - Converts position controller output to attitude targets
 *             - Applies automatic yaw control (ROI, heading, or look-ahead)
 * 
 *          Waypoint Navigation Features:
 *          - Straight-line navigation with configurable speed and acceleration
 *          - Spline path following for smooth curved trajectories
 *          - Corner cutting/anticipation for faster mission execution
 *          - Speed ramping at start/end of segments
 *          - Terrain following using terrain database or rangefinder
 *          - Obstacle avoidance integration (if enabled)
 * 
 *          Terrain Handling:
 *          - wp_nav->update_wpnav() returns terrain failsafe status
 *          - If terrain data unavailable for terrain-relative waypoint, triggers failsafe
 *          - copter.failsafe_terrain_set_status() logs terrain state
 * 
 *          Auto Yaw Modes:
 *          - HOLD: Maintain current heading
 *          - LOOK_AT_NEXT_WP: Point toward next waypoint (default)
 *          - ROI: Point toward region of interest
 *          - FIXED: Hold specified heading
 *          - LOOK_AHEAD: Point along velocity vector
 *          - CIRCLE: Rotate with circular motion (for circle mode)
 * 
 * @note Called at 100Hz or higher from run() when in WP or CIRCLE_MOVE_TO_EDGE submode
 * @note Navigation continues until wp_nav->reached_wp_destination() returns true
 * @note If motors disarmed mid-flight, makes ground handling safe and exits
 * @warning Terrain failsafe may trigger if terrain database unavailable for terrain waypoints
 * 
 * @see wp_start() for waypoint initialization
 * @see verify_nav_wp() for waypoint completion detection
 * @see AC_WPNav for waypoint navigation algorithms
 * @see auto_yaw for yaw control in Auto mode
 */
void ModeAuto::wp_run()
{
    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // run waypoint controller
    copter.failsafe_terrain_set_status(wp_nav->update_wpnav());

    // WP_Nav has set the vertical position control targets
    // run the vertical position controller and set output throttle
    pos_control->update_U_controller();

    // call attitude controller with auto yaw
    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.get_heading());
}

/**
 * @brief Execute autonomous landing sequence in Auto mode
 * 
 * @details Performs controlled descent and landing at current location or mission-specified
 *          landing point. Supports precision landing with IR-LOCK or other position sensors.
 *          Triggered by NAV_LAND or NAV_VTOL_LAND mission commands.
 * 
 *          Landing Sequence:
 *          1. If landing location specified in command, first flies to location (FlyToLocation state)
 *          2. Once at location (or if no location specified), begins descent (Descending state)
 *          3. Descends at configured rate (LAND_SPEED parameter)
 *          4. Detects ground contact via motor output/IMU
 *          5. Disarms motors after landing complete (unless MIS_OPTIONS continues mission)
 * 
 *          Control Flow:
 *          1. Safety check: If disarmed or landed, safe ground handling and exit
 *          2. Set motor spool state to THROTTLE_UNLIMITED for controlled descent
 *          3. Call land_run_normal_or_precland() which handles:
 *             - Normal landing: Vertical descent with horizontal position hold
 *             - Precision landing: Sensor-guided landing on target (if enabled/detected)
 *             - Ground contact detection via thrust reduction
 *             - Landing complete detection
 * 
 *          Precision Landing:
 *          - If AC_PRECLAND_ENABLED and target detected, uses precision landing
 *          - Guides vehicle to land precisely on IR-LOCK beacon or similar target
 *          - Falls back to normal landing if target lost
 *          - copter.ap.prec_land_active flag indicates precision landing active
 * 
 *          Landing Speed:
 *          - Vertical descent rate from LAND_SPEED parameter (default 50 cm/s)
 *          - Automatically reduces speed near ground for soft touchdown
 *          - LAND_SPEED_HIGH parameter for initial fast descent (if enabled)
 * 
 *          Ground Contact Detection:
 *          - Monitors motor output reduction when vehicle weight transfers to ground
 *          - Uses IMU Z-axis acceleration to detect touchdown
 *          - Sets copter.ap.land_complete flag when landing detected
 *          - Requires motors at GROUND_IDLE spool state for verify_land() completion
 * 
 *          Horizontal Position Control:
 *          - Maintains position hold during descent
 *          - Pilot can reposition with stick inputs if LAND_REPOSITION enabled
 *          - copter.ap.land_repo_active tracks repositioning state
 * 
 *          Landing Gear:
 *          - Automatically deploys landing gear at land_start() (if equipped)
 *          - Provides mechanical protection during touchdown
 * 
 *          Mission Continuation:
 *          - Normal behavior: Disarms after landing via verify_land()
 *          - MIS_OPTIONS bit can allow mission continue after landing
 *          - Useful for multi-leg missions with intermediate landings
 * 
 * @note Called at 100Hz or higher from run() when in LAND submode
 * @note Landing complete flag set by land_detector in main code
 * @note Yaw held at current heading during descent (AutoYaw::Mode::HOLD)
 * @warning Vehicle will disarm automatically after landing unless MIS_OPTIONS configured otherwise
 * @warning Precision landing requires target visibility - ensure lighting and contrast adequate
 * 
 * @see land_start() for landing initialization
 * @see verify_land() for landing completion detection
 * @see land_run_normal_or_precland() for landing control algorithms
 * @see do_land() for mission command processing
 */
void ModeAuto::land_run()
{

    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // run normal landing or precision landing (if enabled)
    land_run_normal_or_precland();
}

/**
 * @brief Execute Return-to-Launch sequence within Auto mode
 * 
 * @details Delegates RTL behavior to the dedicated RTL flight mode controller when
 *          NAV_RETURN_TO_LAUNCH mission command is active. This allows missions to
 *          include an RTL sequence without switching out of Auto mode.
 * 
 *          RTL Delegation:
 *          - Calls copter.mode_rtl.run(false) to execute RTL logic
 *          - false parameter indicates running as sub-mode (not primary mode)
 *          - Uses all RTL mode configuration (RTL_ALT, RTL_SPEED, etc.)
 *          - Follows standard RTL state machine: climb, return, loiter, land
 * 
 *          RTL Sequence Phases:
 *          1. InitialClimb: Climb to RTL_ALT if below it
 *          2. Starting: Initialize return navigation
 *          3. Returning: Fly toward home position at RTL_SPEED
 *          4. LoiteringAtHome: Arrive at home, loiter briefly
 *          5. FinalDescent: Descend to land
 *          6. Land: Execute landing sequence
 * 
 *          Home Position:
 *          - Returns to position set by DO_SET_HOME or initial arming location
 *          - RTL_ALT parameter determines return altitude (cm above home)
 *          - RTL_ALT_FINAL determines final loiter altitude before landing
 * 
 *          Speed Control:
 *          - RTL_SPEED parameter controls horizontal return speed (cm/s)
 *          - Uses standard climb/descent speeds from WPNAV parameters
 *          - Can be overridden by DO_CHANGE_SPEED commands before NAV_RETURN_TO_LAUNCH
 * 
 *          Completion Detection:
 *          - verify_RTL() checks mode_rtl.state_complete()
 *          - Also verifies in FINAL_DESCENT or LAND state
 *          - And motors at GROUND_IDLE spool state
 *          - Mission advances to next command once landed
 * 
 *          Integration with Auto RTL:
 *          - Different from auto_RTL feature (which uses DO_LAND_START/DO_RETURN_PATH_START)
 *          - This is explicit NAV_RETURN_TO_LAUNCH mission command
 *          - Both use RTL mode controller but triggered differently
 * 
 *          Failsafe Interaction:
 *          - If failsafe triggers during mission RTL, may switch to true RTL mode
 *          - Behavior depends on failsafe configuration (FS_THR_ENABLE, etc.)
 * 
 * @note Called at 100Hz or higher from run() when in RTL submode
 * @note Uses RTL mode parameters and configuration (RTL_ALT, RTL_SPEED, RTL_CONE_SLOPE, etc.)
 * @note rtl_start() must be called first to initialize RTL mode state
 * @warning Ensure RTL_ALT is sufficient to clear obstacles between current position and home
 * @warning Home position must be set before RTL can execute (automatically set at arming)
 * 
 * @see rtl_start() for RTL mode initialization in Auto
 * @see verify_RTL() for RTL completion detection
 * @see ModeRTL for detailed RTL flight mode implementation
 * @see do_RTL() for NAV_RETURN_TO_LAUNCH command processing
 */
void ModeAuto::rtl_run()
{
    // call regular rtl flight mode run function
    copter.mode_rtl.run(false);
}

/**
 * @brief Execute circular loiter pattern in Auto mode
 * 
 * @details Flies vehicle in circular pattern around specified center point at constant radius.
 *          Triggered by NAV_LOITER_TURNS mission command which specifies circle center, radius,
 *          number of turns, and direction (clockwise/counter-clockwise).
 * 
 *          Circle Navigation:
 *          - Maintains constant distance (radius) from center point
 *          - Flies at constant angular rate (configurable via CIRCLE_RATE parameter)
 *          - Supports both clockwise and counter-clockwise rotation
 *          - Altitude can be relative to home, absolute, or terrain-relative
 *          - Yaw can track circle (tangent to path) or point at ROI
 * 
 *          Control Flow:
 *          1. Update circle navigation controller (circle_nav->update_cms())
 *             - Calculates position on circle based on current angle
 *             - Manages angular velocity and acceleration
 *             - Returns terrain failsafe status for terrain-relative circles
 *          2. Update vertical position controller (pos_control->update_U_controller())
 *             - Maintains target altitude while circling
 *          3. Command attitude controller with thrust vector and yaw
 *             - Yaw follows circle tangent (CIRCLE mode) or ROI if set
 * 
 *          Circle Parameters:
 *          - Radius: Specified in NAV_LOITER_TURNS command (meters)
 *          - Rate: Angular velocity from CIRCLE_RATE parameter (deg/s)
 *          - Direction: Clockwise (positive rate) or CCW (negative rate)
 *          - Turns: Number of complete circles before advancing to next command
 * 
 *          Entry Method:
 *          - Vehicle first moves to edge of circle via CIRCLE_MOVE_TO_EDGE submode
 *          - Once at edge (within 3m), switches to CIRCLE submode
 *          - Prevents spiral-in trajectory by starting from correct radius
 * 
 *          Terrain Following:
 *          - If circle center altitude is terrain-relative, maintains height above terrain
 *          - Terrain failsafe triggered if terrain data unavailable
 *          - copter.failsafe_terrain_set_status() monitors terrain status
 * 
 *          Completion Detection:
 *          - verify_circle() checks total angle traveled (circle_nav->get_angle_total_rad())
 *          - Mission advances when angle >= (turns * 2*PI)
 *          - GCS notification sent for each completed turn
 * 
 * @note Called at 100Hz or higher from run() when in CIRCLE submode
 * @note Circle center and radius set by circle_movetoedge_start() before entering this mode
 * @note Auto yaw typically set to CIRCLE mode to maintain tangent heading
 * @warning Terrain failsafe may trigger for terrain-relative circles without terrain data
 * 
 * @see circle_start() for circle initialization
 * @see circle_movetoedge_start() for moving to circle edge
 * @see verify_circle() for completion detection
 * @see AC_Circle for circular navigation algorithms
 */
void ModeAuto::circle_run()
{
    // call circle controller
    copter.failsafe_terrain_set_status(copter.circle_nav->update_cms());

    // WP_Nav has set the vertical position control targets
    // run the vertical position controller and set output throttle
    pos_control->update_U_controller();

    // call attitude controller with auto yaw
    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.get_heading());
}

#if AC_NAV_GUIDED || AP_SCRIPTING_ENABLED
/**
 * @brief Execute externally-guided navigation in AUTO mode
 * 
 * @details Implements NAV_GUIDED mission command execution, allowing external navigation
 *          controllers (GCS or companion computer) to command position/velocity/attitude
 *          targets during an AUTO mission. This function is called by run() at 100Hz or
 *          higher when in NAVGUIDED or NAV_SCRIPT_TIME submode.
 *          
 *          External guidance integration:
 *          - Delegates control to Guided flight mode controller
 *          - Accepts MAVLink SET_POSITION_TARGET_* messages from external sources
 *          - Supports position, velocity, and acceleration control modes
 *          - Enforces safety limits configured via DO_GUIDED_LIMITS command
 *          - Returns control to mission when NAV_GUIDED_ENABLE(0) command received
 *          
 *          Typical use cases:
 *          - Precision landing with external vision system guidance
 *          - Object tracking or following using companion computer
 *          - Dynamic obstacle avoidance with external path planning
 *          - Custom navigation behaviors via Lua scripting (NAV_SCRIPT_TIME)
 *          
 *          Safety mechanisms:
 *          - Time limits prevent indefinite external control
 *          - Horizontal and vertical position limits enforced
 *          - Failsafe triggers if limits exceeded or timeout occurs
 *          - Mission resumes at next command when guided period ends
 * 
 * @note Called at 100Hz or higher when _mode == SubMode::NAVGUIDED or SubMode::NAV_SCRIPT_TIME
 * @note Requires valid position estimate (GPS or non-GPS navigation)
 * @note Only available if AC_NAV_GUIDED or AP_SCRIPTING_ENABLED is defined
 * 
 * @see ModeAuto::nav_guided_start() for initialization
 * @see ModeAuto::do_nav_guided_enable() for mission command handling
 * @see ModeAuto::verify_nav_guided_enable() for completion checking
 * @see ModeGuided::run() for actual control implementation
 * @see MAV_CMD_NAV_GUIDED_ENABLE mission command documentation
 */
void ModeAuto::nav_guided_run()
{
    // call regular guided flight mode run function
    copter.mode_guided.run();
}
#endif  // AC_NAV_GUIDED || AP_SCRIPTING_ENABLED

/**
 * @brief Execute loiter (position hold) behavior in AUTO mode
 * 
 * @details Maintains position at a loiter target during AUTO mission execution. This function
 *          is called by run() at 100Hz or higher when in LOITER submode, implementing station-
 *          keeping at a commanded waypoint location with automatic heading control.
 *          
 *          Loiter mode mission commands:
 *          - NAV_LOITER_UNLIM: Indefinite position hold at waypoint
 *          - NAV_LOITER_TIME: Time-limited position hold (duration in cmd.p1)
 *          - Post-mission loiter: Default behavior after mission completion
 *          - Mission abort recovery: Safe holding pattern if mission cannot continue
 *          
 *          Position hold control implementation:
 *          - WP_Nav controller maintains horizontal position at loiter target
 *          - Position controller maintains vertical position (altitude hold)
 *          - Auto-yaw controller manages heading (HOLD, ROI, or custom behavior)
 *          - Terrain following supported if loiter target uses ABOVE_TERRAIN frame
 *          
 *          Control loop execution (100Hz typical):
 *          1. Safety check: Verify vehicle armed and airborne
 *          2. Motor spool-up: Enable full throttle range for position control
 *          3. Horizontal control: wp_nav->update_wpnav() computes position corrections
 *          4. Vertical control: pos_control->update_U_controller() maintains altitude
 *          5. Attitude control: Convert position demands to motor thrust commands
 *          
 *          Loiter behavior characteristics:
 *          - Precision: ±50cm horizontal accuracy (GPS-dependent)
 *          - Wind resistance: Active compensation via position controller
 *          - Pilot override: Yaw input allowed unless IgnorePilotYaw option set
 *          - Battery monitoring: Loiter continues during low battery warnings
 *          - Terrain tracking: Uses rangefinder or terrain database if configured
 * 
 * @note Called at 100Hz or higher when _mode == SubMode::LOITER
 * @note Requires valid position estimate from GPS or non-GPS navigation source
 * @note If vehicle is disarmed or landed, safely shuts down motors and exits
 * @note Loiter position set by loiter_start() or from NAV_LOITER_* mission commands
 * 
 * @warning Mission will not progress while in LOITER submode unless time expires
 *          (NAV_LOITER_TIME) or external mode change occurs
 * 
 * @see ModeAuto::loiter_start() for loiter target initialization
 * @see ModeAuto::do_loiter_unlimited() for NAV_LOITER_UNLIM command handling
 * @see ModeAuto::do_loiter_time() for NAV_LOITER_TIME command handling
 * @see ModeAuto::verify_loiter_time() for time-based completion checking
 * @see AC_WPNav::update_wpnav() for horizontal position control implementation
 */
void ModeAuto::loiter_run()
{
    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // run waypoint and z-axis position controller
    copter.failsafe_terrain_set_status(wp_nav->update_wpnav());

    pos_control->update_U_controller();

    // call attitude controller with auto yaw
    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.get_heading());
}

/**
 * @brief Execute loiter-to-altitude behavior in AUTO mode
 * 
 * @details Implements NAV_LOITER_TO_ALT mission command execution, combining horizontal
 *          position hold (loiter) with simultaneous altitude change to reach a target
 *          altitude. This function is called by run() at 100Hz or higher when in
 *          LOITER_TO_ALT submode, providing a smooth coordinated climb or descent while
 *          maintaining horizontal position.
 *          
 *          NAV_LOITER_TO_ALT mission command behavior:
 *          - Flies to specified horizontal position (lat/lon in command)
 *          - Climbs or descends to specified altitude (alt in command)
 *          - Horizontal and vertical movements occur simultaneously
 *          - Mission completes when BOTH position AND altitude targets achieved
 *          - Useful for altitude changes at waypoints without stopping vertical motion
 *          
 *          Two-phase execution sequence:
 *          
 *          Phase 1 - Waypoint Navigation (until horizontal destination reached):
 *          - Delegates to wp_run() for combined horizontal and vertical navigation
 *          - Flies toward target horizontal position at full navigation speed
 *          - Monitors wp_nav->reached_wp_destination_NE() for arrival
 *          - Transitions to Phase 2 when horizontal position achieved
 *          
 *          Phase 2 - Loiter with Altitude Adjustment (after horizontal arrival):
 *          - Initializes horizontal position hold controller (one-time setup)
 *          - Maintains precise loiter at target horizontal position
 *          - Applies sqrt_controller for smooth altitude approach with natural deceleration
 *          - Continuously monitors altitude error and crossing detection
 *          - Completes when altitude within 5cm tolerance OR crosses target altitude
 *          
 *          Altitude control implementation:
 *          - Target altitude set by do_loiter_to_alt() in loiter_to_alt.alt (ABOVE_HOME frame)
 *          - Altitude error: current_alt - target_alt (positive = above target)
 *          - Square-root controller provides natural deceleration near target altitude
 *          - Climb rate constrained by configured speed limits (up/down)
 *          - Crossing detection: if sign of altitude error changes, target achieved
 *          - 5cm hysteresis tolerance prevents oscillation at target altitude
 *          
 *          Horizontal control during Phase 2:
 *          - land_run_horizontal_control() provides precise position hold
 *          - Uses same control logic as landing for smooth, stable positioning
 *          - Supports pilot yaw input override (unless disabled by AUTO_OPTIONS)
 *          - Wind compensation via position controller
 *          
 *          Safety and obstacle avoidance:
 *          - Motor interlock check prevents uncommanded altitude changes
 *          - Obstacle avoidance adjusts climb rate if obstacles detected above/below
 *          - Surface tracking (rangefinder) maintains ground clearance if enabled
 *          - Disarm safety: immediately cuts throttle if motors disarmed
 *          
 *          Completion criteria (both must be true):
 *          - loiter_to_alt.reached_destination_xy == true (horizontal position achieved)
 *          - loiter_to_alt.reached_alt == true (altitude within tolerance or crossed)
 *          
 *          Typical use cases:
 *          - Survey missions: Climb to survey altitude at first waypoint
 *          - Landing approach: Descend to safe altitude before final approach
 *          - Terrain following: Adjust altitude while maintaining horizontal track
 *          - Multi-level operations: Change altitude between work areas
 * 
 * @note Called at 100Hz or higher when _mode == SubMode::LOITER_TO_ALT
 * @note Requires valid position estimate and motor interlock enabled
 * @note Altitude tolerance is 5cm, optimized for stable completion without oscillation
 * @note If horizontal destination not specified (0,0), loiters at current position
 * @note Square-root controller provides time-optimal altitude changes with smooth deceleration
 * 
 * @warning Motor interlock must be enabled; if disabled, immediately disarms for safety
 * @warning Altitude crossing detection means overshooting target counts as completion
 *          (prevents infinite oscillation if target not achievable)
 * 
 * @see ModeAuto::do_loiter_to_alt() for target altitude and position initialization
 * @see ModeAuto::verify_loiter_to_alt() for completion checking
 * @see ModeAuto::wp_run() for Phase 1 waypoint navigation implementation
 * @see ModeAuto::land_run_horizontal_control() for Phase 2 horizontal position hold
 * @see sqrt_controller() for altitude approach velocity calculation
 * @see MAV_CMD_NAV_LOITER_TO_ALT mission command documentation
 */
void ModeAuto::loiter_to_alt_run()
{
    // if not auto armed or motor interlock not enabled set throttle to zero and exit immediately
    if (is_disarmed_or_landed() || !motors->get_interlock()) {
        make_safe_ground_handling();
        return;
    }

    // possibly just run the waypoint controller:
    if (!loiter_to_alt.reached_destination_xy) {
        loiter_to_alt.reached_destination_xy = wp_nav->reached_wp_destination_NE();
        if (!loiter_to_alt.reached_destination_xy) {
            wp_run();
            return;
        }
    }

    if (!loiter_to_alt.loiter_start_done) {
        // set horizontal speed and acceleration limits
        pos_control->set_max_speed_accel_NE_cm(wp_nav->get_default_speed_NE_cms(), wp_nav->get_wp_acceleration_cmss());
        pos_control->set_correction_speed_accel_NE_cm(wp_nav->get_default_speed_NE_cms(), wp_nav->get_wp_acceleration_cmss());

        if (!pos_control->is_active_NE()) {
            pos_control->init_NE_controller();
        }

        loiter_to_alt.loiter_start_done = true;
    }
    const float alt_error_cm = copter.current_loc.alt - loiter_to_alt.alt;
    if (fabsf(alt_error_cm) < 5.0) { // random numbers R US
        loiter_to_alt.reached_alt = true;
    } else if (alt_error_cm * loiter_to_alt.alt_error_cm < 0) {
        // we were above and are now below, or vice-versa
        loiter_to_alt.reached_alt = true;
    }
    loiter_to_alt.alt_error_cm = alt_error_cm;

    // loiter...

    land_run_horizontal_control();

    // Compute a vertical velocity demand such that the vehicle
    // approaches the desired altitude.
    float target_climb_rate_cms = sqrt_controller(
        -alt_error_cm,
        pos_control->get_pos_U_p().kP(),
        pos_control->get_max_accel_U_cmss(),
        G_Dt);
    target_climb_rate_cms = constrain_float(target_climb_rate_cms, pos_control->get_max_speed_down_cms(), pos_control->get_max_speed_up_cms());

    // get avoidance adjusted climb rate
    target_climb_rate_cms = get_avoidance_adjusted_climbrate_cms(target_climb_rate_cms);

#if AP_RANGEFINDER_ENABLED
    // update the vertical offset based on the surface measurement
    copter.surface_tracking.update_surface_offset();
#endif

    // Send the commanded climb rate to the position controller
    pos_control->set_pos_target_U_from_climb_rate_cm(target_climb_rate_cms);

    pos_control->update_U_controller();
}

/**
 * @brief Execute fixed attitude hold for specified duration in AUTO mode
 * 
 * @details Implements NAV_ATTITUDE_TIME mission command execution, maintaining a commanded
 *          attitude (roll, pitch, yaw angles) with specified climb rate for a fixed time
 *          duration. This function is called by run() at 100Hz or higher when in
 *          NAV_ATTITUDE_TIME submode, providing direct attitude control during AUTO missions.
 *          
 *          NAV_ATTITUDE_TIME mission command purpose:
 *          - Holds specific vehicle attitude (Euler angles) for testing or special maneuvers
 *          - Allows controlled tilting while maintaining altitude or climbing/descending
 *          - Bypasses normal position control for direct attitude authority
 *          - Useful for sensor calibration, aerodynamic testing, or acrobatic maneuvers
 *          - Time duration specified in command; mission continues when time expires
 *          
 *          Attitude control implementation:
 *          - Roll angle: nav_attitude_time.roll_deg (degrees, positive = right roll)
 *          - Pitch angle: nav_attitude_time.pitch_deg (degrees, positive = nose up)
 *          - Yaw angle: nav_attitude_time.yaw_deg (degrees, absolute heading in NED frame)
 *          - Climb rate: nav_attitude_time.climb_rate (m/s, positive = climbing)
 *          - All targets set by do_nav_attitude_time() from mission command parameters
 *          
 *          Safety angle limiting:
 *          - Maximum lean angle is the lesser of:
 *            a) Configured maximum lean angle (attitude_control->lean_angle_max_rad())
 *            b) Altitude hold lean angle limit (for vertical control stability)
 *            c) Hard minimum limit of 10 degrees (safety floor)
 *          - Roll and pitch magnitude constrained as vector (preserves direction)
 *          - Prevents excessive tilting that could cause loss of control or crashes
 *          - Yaw angle not limited (full 360° rotation allowed)
 *          
 *          Climb rate control:
 *          - Input climb rate converted from m/s to cm/s (×100)
 *          - Constrained to configured speed_up and speed_down limits
 *          - Obstacle avoidance can adjust climb rate if obstacles detected
 *          - Zero climb rate maintains current altitude (altitude hold)
 *          - Positive values climb, negative values descend
 *          
 *          Control loop execution (100Hz typical):
 *          1. Safety checks: Verify armed and motor interlock enabled
 *          2. Climb rate processing: Convert units, constrain limits, apply avoidance
 *          3. Attitude angle limiting: Compute vector magnitude limit, constrain roll/pitch
 *          4. Attitude command: Send Euler angles to attitude controller
 *          5. Vertical command: Send climb rate to position controller
 *          6. Motor mixing: Attitude and position controllers compute motor outputs
 *          
 *          Differences from normal AUTO flight:
 *          - NO horizontal position control (vehicle will drift with wind)
 *          - NO GPS-based navigation (attitude-only control)
 *          - NO waypoint tracking or path following
 *          - DOES maintain vertical position control (altitude hold or climb)
 *          - Does NOT require position estimate (can operate without GPS)
 *          
 *          Typical use cases:
 *          - In-flight IMU calibration (specific attitudes for accel calibration)
 *          - Aerodynamic data collection (maintain test attitude during data logging)
 *          - Camera pointing (hold specific angle for sensor or camera operations)
 *          - Custom flight testing (evaluate vehicle response at various attitudes)
 *          - Acrobatic maneuvers in AUTO mode (advanced users only)
 *          
 *          Completion and safety:
 *          - Command completes when time expires (checked by verify_nav_attitude_time())
 *          - Start time: nav_attitude_time.start_ms (set by do_nav_attitude_time())
 *          - Duration: cmd.content.nav_attitude_time.time_sec (from mission command)
 *          - Motor interlock required: immediately disarms if interlock disabled
 *          - EKF failsafe disabled during this command (does not require position estimate)
 * 
 * @note Called at 100Hz or higher when _mode == SubMode::NAV_ATTITUDE_TIME
 * @note Does NOT require position estimate - only attitude control mode in AUTO
 * @note Vehicle will drift horizontally due to wind (no position control active)
 * @note Angle limits prioritize safety over commanded attitude if limits exceeded
 * @note Motor interlock must remain enabled throughout maneuver
 * 
 * @warning NO horizontal position control - vehicle can drift significantly with wind
 * @warning Excessive commanded angles are automatically limited to safe values
 * @warning If motor interlock disabled mid-maneuver, immediately disarms for safety
 * @warning EKF position estimate not required, but attitude estimate must be valid
 * 
 * @see ModeAuto::do_nav_attitude_time() for attitude target initialization from mission command
 * @see ModeAuto::verify_nav_attitude_time() for time-based completion checking
 * @see ModeAuto::requires_GPS() for GPS requirement logic (returns false for this submode)
 * @see AC_AttitudeControl::input_euler_angle_roll_pitch_yaw_rad() for attitude command processing
 * @see MAV_CMD_NAV_ATTITUDE_TIME mission command documentation
 */
void ModeAuto::nav_attitude_time_run()
{
    // if not auto armed or motor interlock not enabled set throttle to zero and exit immediately
    if (is_disarmed_or_landed() || !motors->get_interlock()) {
        make_safe_ground_handling();
        return;
    }

    // constrain climb rate
    float target_climb_rate_cms = constrain_float(nav_attitude_time.climb_rate * 100.0, pos_control->get_max_speed_down_cms(), pos_control->get_max_speed_up_cms());

    // get avoidance adjusted climb rate
    target_climb_rate_cms = get_avoidance_adjusted_climbrate_cms(target_climb_rate_cms);

    // limit and scale lean angles
    // todo: change euler magnitiude limit to lean angle limit
    const float angle_limit_rad = MAX(radians(10.0f), MIN(attitude_control->lean_angle_max_rad(), attitude_control->get_althold_lean_angle_max_rad()));
    Vector2f target_rp_rad(radians(nav_attitude_time.roll_deg), radians(nav_attitude_time.pitch_deg));
    target_rp_rad.limit_length(angle_limit_rad);

    // send targets to attitude controller
    attitude_control->input_euler_angle_roll_pitch_yaw_rad(target_rp_rad.x, target_rp_rad.y, radians(nav_attitude_time.yaw_deg), true);

    // Send the commanded climb rate to the position controller
    pos_control->set_pos_target_U_from_climb_rate_cm(target_climb_rate_cms);

    pos_control->update_U_controller();
}

#if AC_PAYLOAD_PLACE_ENABLED
/**
 * @brief Execute autonomous payload placement maneuver in AUTO mode
 * 
 * @details Implements NAV_PAYLOAD_PLACE mission command execution, autonomously delivering
 *          a payload to a specified location using intelligent touchdown detection and
 *          controlled release. This function is called by ModeAuto::run() at 100Hz or higher
 *          when in NAV_PAYLOAD_PLACE submode, managing an 8-state state machine for safe
 *          payload delivery operations.
 *          
 *          NAV_PAYLOAD_PLACE mission command capabilities:
 *          - Autonomous cargo delivery to GPS coordinates
 *          - Intelligent payload touchdown detection via thrust monitoring
 *          - Controlled gripper release after confirmed ground contact
 *          - Automatic return to pre-descent altitude after release
 *          - Configurable descent speed, detection thresholds, and delays
 *          - Optional rangefinder-based altitude confirmation for added safety
 *          
 *          Eight-state state machine sequence:
 *          
 *          1. **FlyToLocation**: Navigate to horizontal drop position
 *             - Uses wp_run() for waypoint navigation to target lat/lon
 *             - Maintains current altitude during horizontal flight
 *             - Transitions to Descent_Start when wp_nav->reached_wp_destination()
 *             - If no location specified (0,0), immediately starts descent at current position
 *          
 *          2. **Descent_Start**: Initialize descent parameters (single iteration)
 *             - Records starting altitude (descent_start_altitude_cm) for return climb
 *             - Configures descent speed from PLDP_DESCENT_SPEED_MS or falls back to LAND_SPEED
 *             - Initializes thrust calibration baseline (descent_thrust_level = 1.0)
 *             - Resets descent timing counters for thrust measurement phase
 *             - Immediately transitions to Descent state (FALLTHROUGH)
 *          
 *          3. **Descent**: Controlled descent with touchdown detection
 *             - Descends at constant rate (descent_speed_cms)
 *             - Monitors descent_max_cm limit; aborts to Ascent_Start if exceeded
 *             - **Thrust calibration phase** (first 2 seconds after steady descent):
 *               * Waits for descent velocity to stabilize (95% of target speed)
 *               * Records minimum thrust over 2-second calibration window
 *               * Establishes baseline thrust_level required for steady descent
 *             - **Touchdown detection phase** (after calibration):
 *               * Continuously compares current thrust to baseline
 *               * Touchdown detected when: thrust < PLDP_THRUST_PLACED_FRAC × baseline
 *               * Typical threshold: 90% of descent thrust (10% reduction = payload on ground)
 *               * Optional rangefinder check: altitude must drop below PLDP_RNG_MAX
 *               * Requires sustained detection for 500ms to prevent false positives
 *             - Transitions to Release when touchdown confirmed for 500ms
 *          
 *          4. **Release**: Initiate gripper release sequence
 *             - Reinitializes vertical position controller to handle weight discontinuity
 *             - Commands gripper to release payload (AP::gripper().release())
 *             - Transitions to Releasing to monitor gripper actuation
 *             - If no gripper configured, directly transitions to Delay state
 *          
 *          5. **Releasing**: Wait for gripper release confirmation
 *             - Monitors AP::gripper().released() status
 *             - Holds zero climb rate to maintain touchdown position
 *             - Transitions to Delay when gripper confirms full release
 *             - Timeout protection if gripper fails to confirm (transitions after delay)
 *          
 *          6. **Delay**: Post-release stabilization period
 *             - Maintains horizontal position and zero vertical velocity
 *             - Allows payload to fully separate before climb begins
 *             - Duration: PLDP_DELAY_S parameter (default 0.5s)
 *             - Purpose: Prevents snagging tether or premature liftoff
 *             - Transitions to Ascent_Start when delay expires
 *          
 *          7. **Ascent_Start**: Begin return climb (single iteration)
 *             - No parameter changes, just state transition marker
 *             - Immediately transitions to Ascent state (FALLTHROUGH)
 *          
 *          8. **Ascent**: Return to pre-descent altitude
 *             - Climbs back to descent_start_altitude_cm (original altitude)
 *             - Uses standard position controller climb
 *             - Completion criteria: Within 10% of stopping distance from target altitude
 *             - Stopping distance: 0.5 × (0.1 × max_speed_up)² / max_accel_up
 *             - Transitions to Done when ascent complete
 *          
 *          9. **Done**: Maneuver complete, ready for next mission command
 *          
 *          Touchdown detection algorithm details:
 *          
 *          The intelligent touchdown detection uses thrust monitoring to identify when the
 *          payload makes ground contact. As the vehicle descends with a suspended payload,
 *          the motors provide thrust to counteract both vehicle and payload weight. When
 *          the payload touches the ground, it transfers weight to the surface, immediately
 *          reducing the thrust required to maintain descent rate.
 *          
 *          Detection sequence:
 *          1. Establish descent at steady rate (descent_speed_cms)
 *          2. Wait for velocity to stabilize (>95% of target descent speed)
 *          3. Measure baseline thrust over 2-second calibration window
 *          4. Continuously monitor thrust; compare to baseline × threshold fraction
 *          5. Touchdown indicated when thrust drops below threshold
 *          6. Confirm sustained detection for 500ms to reject transients
 *          7. Optional: Cross-check rangefinder altitude below configured maximum
 *          
 *          Configurable parameters (in g2 parameter group):
 *          - PLDP_DESCENT_SPEED_MS: Descent speed in m/s (0 = use LAND_SPEED)
 *          - PLDP_THRUST_PLACED_FRAC: Thrust fraction threshold (0.0-1.0, typically 0.9)
 *          - PLDP_DELAY_S: Post-release delay in seconds
 *          - PLDP_RNG_MAX: Maximum rangefinder altitude for touchdown (meters, 0 = disabled)
 *          - descent_max_cm: Maximum descent distance from mission command (cm, 0 = unlimited)
 *          
 *          Safety mechanisms and abort conditions:
 *          
 *          1. **Manual abort**: Pilot manually opens gripper before descent
 *             - Detected in FlyToLocation or Descent_Start states
 *             - Immediately transitions to Done without descent
 *             - Logs "Abort: Gripper Open" telemetry message
 *          
 *          2. **Manual release**: Pilot manually opens gripper during descent
 *             - Skips touchdown detection and proceeds to Release state
 *             - Logs "Manual release" telemetry message
 *             - Continues through normal ascent sequence
 *          
 *          3. **Maximum descent limit**: Vehicle descends past configured limit
 *             - Checks descent_start_altitude_cm - current_altitude > descent_max_cm
 *             - Aborts descent, immediately begins ascent to starting altitude
 *             - Logs "Reached maximum descent" warning
 *             - Prevents ground impact if touchdown not detected
 *          
 *          4. **Rangefinder validation**: If PLDP_RNG_MAX set but rangefinder unavailable
 *             - Aborts if rangefinder not enabled when parameter requires it
 *             - Logs "PLDP_RNG_MAX set and rangefinder not enabled" warning
 *             - Prevents descent without required safety sensor
 *          
 *          5. **Landing detection**: Vehicle fully lands before reaching drop zone
 *             - Monitors copter.ap.land_complete and land_complete_maybe flags
 *             - Immediately releases payload if landed during descent
 *             - Softens position target to accommodate ground contact
 *             - Logs "landed" telemetry message
 *          
 *          6. **Disarm safety**: Vehicle disarmed or motors disabled mid-maneuver
 *             - Immediately calls make_safe_ground_handling() and exits
 *             - Cuts throttle to zero for safety
 *             - State preserved for potential re-arm and continuation
 *          
 *          Horizontal position control during maneuver:
 *          - FlyToLocation: Uses wp_run() for full waypoint navigation
 *          - All other states: Uses land_run_horizontal_control() for precision hold
 *          - Maintains GPS position throughout descent, release, and ascent
 *          - Wind compensation active to prevent drift during vertical maneuvers
 *          
 *          Vertical control implementation by state:
 *          - Descent_Start/Descent: land_at_climb_rate_cm(-descent_speed_cms, true)
 *          - Release/Releasing/Delay/Ascent_Start: land_at_climb_rate_cm(0.0, false)
 *          - Ascent/Done: input_pos_vel_accel_U_cm(descent_start_altitude_cm, 0, 0)
 *          
 *          Typical mission use cases:
 *          - Emergency supply delivery (medical supplies, rescue equipment)
 *          - Agricultural operations (sensor package deployment, sample collection)
 *          - Construction site material delivery
 *          - Search and rescue equipment drops
 *          - Scientific payload deployment (sensors, markers, instruments)
 *          
 *          Gripper integration:
 *          - Requires AP_GRIPPER_ENABLED and valid gripper hardware
 *          - Gripper must be closed (holding payload) before mission starts
 *          - Release command: AP::gripper().release()
 *          - Confirmation check: AP::gripper().released()
 *          - If no gripper configured, maneuver still executes (positioning only)
 * 
 * @note Called at 100Hz or higher when ModeAuto::_mode == SubMode::NAV_PAYLOAD_PLACE
 * @note Requires valid GPS position estimate for horizontal position hold
 * @note Optional rangefinder provides additional touchdown confirmation if configured
 * @note State machine preserves progress across calls; state persists in PayloadPlace::state
 * @note Thrust calibration requires 2 seconds of steady descent for accurate baseline
 * @note Touchdown detection threshold (90% default) may need tuning for different payload weights
 * 
 * @warning Ensure PLDP_THRUST_PLACED_FRAC tuned for payload weight (heavier = lower threshold)
 * @warning descent_max_cm parameter must be set to prevent uncontrolled descent to ground
 * @warning Rangefinder glitches can cause false touchdown detection; uses glitch_count filter
 * @warning Manual gripper release aborts touchdown detection; ascent proceeds immediately
 * @warning If gripper fails to release, vehicle holds position indefinitely (no timeout)
 * 
 * @see PayloadPlace::start_descent() for Descent_Start initialization
 * @see PayloadPlace::verify() for mission completion checking
 * @see ModeAuto::do_payload_place() for mission command parsing and state setup
 * @see ModeAuto::land_run_horizontal_control() for horizontal position hold implementation
 * @see AC_PosControl::land_at_climb_rate_cm() for descent control with ground detection
 * @see MAV_CMD_NAV_PAYLOAD_PLACE mission command documentation
 * @see PLDP_* parameter documentation for configuration details
 */
void PayloadPlace::run()
{
    const char* prefix_str = "PayloadPlace:";

    if (copter.flightmode->is_disarmed_or_landed()) {
        copter.flightmode->make_safe_ground_handling();
        return;
    }

    // set motors to full range
    copter.motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    const uint32_t descent_thrust_cal_duration_ms = 2000; // milliseconds
    const uint32_t placed_check_duration_ms = 500; // how long we have to be below a throttle threshold before considering placed

    auto &g2 = copter.g2;
    const auto &g = copter.g;
    auto *attitude_control = copter.attitude_control;
    const auto &pos_control = copter.pos_control;
    const auto &wp_nav = copter.wp_nav;

    // Vertical thrust is taken from the attitude controller before angle boost is added
    const float thrust_level = attitude_control->get_throttle_in();
    const uint32_t now_ms = AP_HAL::millis();

    // relax position target if we might be landed
    // if we discover we've landed then immediately release the load:
    if (copter.ap.land_complete || copter.ap.land_complete_maybe) {
        pos_control->soften_for_landing_NE();
        switch (state) {
        case State::FlyToLocation:
            // this is handled in wp_run()
            break;
        case State::Descent_Start:
            // do nothing on this loop
            break;
        case State::Descent:
            gcs().send_text(MAV_SEVERITY_INFO, "%s landed", prefix_str);
            state = State::Release;
            break;
        case State::Release:
        case State::Releasing:
        case State::Delay:
        case State::Ascent_Start:
        case State::Ascent:
        case State::Done:
            break;
        }
    }

#if AP_GRIPPER_ENABLED
    // if pilot releases load manually:
    if (AP::gripper().valid() && AP::gripper().released()) {
        switch (state) {
        case State::FlyToLocation:
        case State::Descent_Start:
            gcs().send_text(MAV_SEVERITY_INFO, "%s Abort: Gripper Open", prefix_str);
            // Descent_Start has not run so we must also initialise descent_start_altitude_cm
            descent_start_altitude_cm = pos_control->get_pos_desired_U_cm();
            state = State::Done;
            break;
        case State::Descent:
            gcs().send_text(MAV_SEVERITY_INFO, "%s Manual release", prefix_str);
            state = State::Release;
            break;
        case State::Release:
        case State::Releasing:
        case State::Delay:
        case State::Ascent_Start:
        case State::Ascent:
        case State::Done:
            break;
        }
    }
#endif

    switch (state) {
    case State::FlyToLocation:
        if (copter.wp_nav->reached_wp_destination()) {
            start_descent();
        }
        break;

    case State::Descent_Start:
        descent_established_time_ms = now_ms;
        descent_start_altitude_cm = pos_control->get_pos_desired_U_cm();
        // limiting the decent rate to the limit set in wp_nav is not necessary but done for safety
        descent_speed_cms = MIN((is_positive(g2.pldp_descent_speed_ms)) ? g2.pldp_descent_speed_ms * 100.0 : abs(g.land_speed), wp_nav->get_default_speed_down_cms());
        descent_thrust_level = 1.0;
        state = State::Descent;
        FALLTHROUGH;

    case State::Descent:
        // check maximum decent distance
        if (!is_zero(descent_max_cm) &&
            descent_start_altitude_cm - pos_control->get_pos_desired_U_cm() > descent_max_cm) {
            state = State::Ascent_Start;
            gcs().send_text(MAV_SEVERITY_WARNING, "%s Reached maximum descent", prefix_str);
            break;
        }
        // calibrate the decent thrust after aircraft has reached constant decent rate and release if threshold is reached
        if (pos_control->get_vel_desired_NEU_cms().z > -0.95 * descent_speed_cms) {
            // decent rate has not reached descent_speed_cms
            descent_established_time_ms = now_ms;
            break;
        } else if (now_ms - descent_established_time_ms < descent_thrust_cal_duration_ms) {
            // record minimum thrust for descent_thrust_cal_duration_ms
            descent_thrust_level = MIN(descent_thrust_level, thrust_level);
            place_start_time_ms = now_ms;
            break;
        } else if (thrust_level > g2.pldp_thrust_placed_fraction * descent_thrust_level) {
            // thrust is above minimum threshold
            place_start_time_ms = now_ms;
            break;
        } else if (is_positive(g2.pldp_range_finder_maximum_m)) {
            if (!copter.rangefinder_state.enabled) {
                // abort payload place because rangefinder is not enabled
                state = State::Ascent_Start;
                gcs().send_text(MAV_SEVERITY_WARNING, "%s PLDP_RNG_MAX set and rangefinder not enabled", prefix_str);
                break;
            } else if (copter.rangefinder_alt_ok() && (copter.rangefinder_state.glitch_count == 0) && (copter.rangefinder_state.alt_cm > g2.pldp_range_finder_maximum_m * 100.0)) {
                // range finder altitude is above maximum
                place_start_time_ms = now_ms;
                break;
            }
        }

        // If we get here:
        // 1. we have reached decent velocity
        // 2. measured the thrust level required for decent
        // 3. detected that our thrust requirements have reduced
        // 4. rangefinder range has dropped below minimum if set
        // 5. place_start_time_ms has been initialised

        // payload touchdown must be detected for 0.5 seconds

        if (now_ms - place_start_time_ms > placed_check_duration_ms) {
            state = State::Release;
            gcs().send_text(MAV_SEVERITY_INFO, "%s payload release thrust threshold: %f", prefix_str, static_cast<double>(g2.pldp_thrust_placed_fraction * descent_thrust_level));
        }
        break;

    case State::Release:
        // Reinitialise vertical position controller to remove discontinuity due to touch down of payload
        pos_control->init_U_controller_no_descent();
#if AP_GRIPPER_ENABLED
        if (AP::gripper().valid()) {
            gcs().send_text(MAV_SEVERITY_INFO, "%s Releasing the gripper", prefix_str);
            AP::gripper().release();
            state = State::Releasing;
        } else {
            state = State::Delay;
        }
#else
        state = State::Delay;
#endif
        break;

    case State::Releasing:
#if AP_GRIPPER_ENABLED
        if (AP::gripper().valid() && !AP::gripper().released()) {
            break;
        }
#endif
        state = State::Delay;
        FALLTHROUGH;

    case State::Delay:
        // If we get here we have finished releasing the gripper
        if (now_ms - place_start_time_ms < placed_check_duration_ms + g2.pldp_delay_s * 1000.0) {
            break;
        }
        FALLTHROUGH;

    case State::Ascent_Start:
        state = State::Ascent;
        FALLTHROUGH;

    case State::Ascent: {
        // Ascent complete when we are less than 10% of the stopping
        // distance from the target altitude stopping distance from
        // vel_threshold_fraction * max velocity
        const float vel_threshold_fraction = 0.1;
        const float stop_distance = 0.5 * sq(vel_threshold_fraction * copter.pos_control->get_max_speed_up_cms()) / copter.pos_control->get_max_accel_U_cmss();
        bool reached_altitude = pos_control->get_pos_desired_U_cm() >= descent_start_altitude_cm - stop_distance;
        if (reached_altitude) {
            state = State::Done;
        }
        break;
    }
    case State::Done:
        break;
    default:
        // this should never happen
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        break;
    }

    switch (state) {
    case State::FlyToLocation:
        // this should never happen
        return copter.mode_auto.wp_run();
    case State::Descent_Start:
    case State::Descent:
        copter.flightmode->land_run_horizontal_control();
        // update altitude target and call position controller
        pos_control->land_at_climb_rate_cm(-descent_speed_cms, true);
        break;
    case State::Release:
    case State::Releasing:
    case State::Delay:
    case State::Ascent_Start:
        copter.flightmode->land_run_horizontal_control();
        // update altitude target and call position controller
        pos_control->land_at_climb_rate_cm(0.0, false);
        break;
    case State::Ascent:
    case State::Done:
        float vel = 0.0;
        copter.flightmode->land_run_horizontal_control();
        pos_control->input_pos_vel_accel_U_cm(descent_start_altitude_cm, vel, 0.0);
        break;
    }
    pos_control->update_U_controller();
}
#endif

/**
 * @brief Replace target location's altitude with vehicle's current altitude, preserving altitude frame
 * 
 * @details Modifies target_loc to use current vehicle altitude while keeping target's horizontal
 *          position and altitude frame unchanged. Critical for commands like "fly to waypoint X,Y
 *          at current altitude" or "land at location X,Y starting from current altitude". Handles
 *          both terrain-relative and other altitude frames with proper sensor selection and frame
 *          conversion. Position controller offsets are subtracted to account for precision landing
 *          or obstacle avoidance adjustments.
 * 
 *          Purpose - Current Altitude Preservation:
 *          - Fly to horizontal position at current altitude
 *          - Start landing descent from current altitude
 *          - Move horizontally without altitude change
 *          - Common for NAV_LAND with specified lat/lon
 * 
 *          Input: target_loc
 *          - Has desired horizontal position (lat, lon)
 *          - Has altitude frame (ABOVE_HOME, ABOVE_TERRAIN, etc.)
 *          - Altitude value will be replaced
 *          - Frame preserved
 * 
 *          Output: Modified target_loc
 *          - Same horizontal position (lat, lon unchanged)
 *          - Same altitude frame
 *          - Altitude = current vehicle altitude in that frame
 *          - Position offsets subtracted
 * 
 *          Terrain Altitude Special Case:
 *          - if target_loc frame is ABOVE_TERRAIN
 *          - AND wp_nav uses rangefinder for terrain
 *          - Use rangefinder-based current altitude
 *          - More accurate for low-altitude terrain following
 * 
 *          Why Rangefinder for Terrain:
 *          - Rangefinder provides direct ground distance
 *          - More accurate than terrain database at low altitude
 *          - Real-time measurement vs database lookup
 *          - Preferred when available and configured
 * 
 *          Rangefinder Altitude Retrieval:
 *          - copter.get_rangefinder_height_interpolated_cm(curr_rngfnd_alt_cm)
 *          - Interpolated for smoother readings
 *          - Returns false if rangefinder unavailable/unhealthy
 *          - Returns altitude above ground in centimeters
 * 
 *          Position Offset Subtraction:
 *          - curr_rngfnd_alt_cm -= pos_control->get_pos_offset_U_cm()
 *          - Position controller may add offsets for:
 *            * Precision landing corrections
 *            * Obstacle avoidance adjustments
 *            * Wind compensation
 *          - Subtract to get true desired altitude
 *          - Ensures consistency
 * 
 *          Why Subtract Position Offset:
 *          - pos_control tracks "desired position" + "offset"
 *          - Offset used for temporary adjustments
 *          - We want the base desired altitude
 *          - Not the offset-adjusted altitude
 *          - Prevents accumulation of offsets
 * 
 *          Minimum Altitude Limit:
 *          - MAX(curr_rngfnd_alt_cm, 200)
 *          - Minimum 2 meters (200cm) altitude
 *          - Safety margin above ground
 *          - Prevents ground collision
 *          - Only for rangefinder-based terrain altitude
 * 
 *          Rangefinder Failure Handling:
 *          - If get_rangefinder_height_interpolated_cm() fails
 *          - Return false immediately
 *          - Caller must handle failure (typically triggers failsafe)
 *          - Cannot proceed without altitude information
 * 
 *          Non-Terrain Altitude Handling:
 *          - Take copy of current location: currloc = copter.current_loc
 *          - Current location has vehicle's current altitude
 *          - May be in different frame than target_loc
 *          - Need to convert to target's frame
 * 
 *          Altitude Frame Conversion:
 *          - currloc.change_alt_frame(target_loc.get_alt_frame())
 *          - Converts currloc to target's altitude frame
 *          - Example: ABOVE_HOME → ABOVE_TERRAIN
 *          - May require terrain database lookup
 * 
 *          Frame Conversion Examples:
 *          - ABOVE_HOME → ABOVE_ORIGIN: Simple offset (home vs origin)
 *          - ABOVE_HOME → ABOVE_TERRAIN: Requires terrain database
 *          - ABOVE_TERRAIN → ABOVE_HOME: Requires terrain database
 *          - ABSOLUTE → ABOVE_HOME: Requires home altitude MSL
 * 
 *          Why Frame Conversion May Fail:
 *          - Terrain database unavailable
 *          - No terrain data at current location
 *          - EKF origin not set
 *          - Home not set
 *          - Return false on failure
 * 
 *          Final Altitude Assignment:
 *          - target_loc.set_alt_cm(currloc.alt - pos_offset, currloc.get_alt_frame())
 *          - currloc.alt: Now in target's frame after conversion
 *          - Subtract position offset
 *          - Set target_loc to this altitude
 *          - Frame already matches (from conversion)
 * 
 *          Use Case - Land Command with Location:
 *          - NAV_LAND at lat=X, lon=Y, alt=0, frame=ABOVE_HOME
 *          - Want to fly to X,Y at current altitude, then descend
 *          - target_loc = X,Y in ABOVE_HOME frame
 *          - shift_alt_to_current_alt() sets altitude to current
 *          - Vehicle flies horizontally to X,Y at current altitude
 *          - Then land_start() initiates descent
 * 
 *          Use Case - Terrain Relative Landing:
 *          - NAV_LAND at lat=X, lon=Y, frame=ABOVE_TERRAIN
 *          - Rangefinder active for terrain following
 *          - target_loc has ABOVE_TERRAIN frame
 *          - shift_alt_to_current_alt() uses rangefinder
 *          - Gets current altitude above ground
 *          - Sets target to same altitude above terrain at X,Y
 *          - Smooth terrain-relative transition
 * 
 *          Use Case - Failed Terrain Conversion:
 *          - Target frame is ABOVE_TERRAIN
 *          - Terrain database has no data at current location
 *          - change_alt_frame() fails
 *          - Function returns false
 *          - Caller logs TERRAIN error
 *          - Falls back to alt-above-home
 * 
 *          Caller Responsibility:
 *          - Check return value
 *          - If false, altitude not set correctly
 *          - Must handle error (log, failsafe, fallback)
 *          - Common: Use ABOVE_HOME as fallback
 * 
 * @param[in,out] target_loc Location to modify; altitude replaced with current altitude
 * 
 * @return true if altitude successfully set to current altitude, false if altitude unavailable
 *              (rangefinder failure or terrain conversion failure)
 * 
 * @note For ABOVE_TERRAIN with rangefinder: Uses rangefinder, minimum 200cm altitude
 * @note For other frames: Converts current altitude to target's frame
 * @note Position controller offsets are subtracted from altitude
 * @note Horizontal position (lat, lon) and altitude frame are preserved
 * @note Failure indicates terrain database or rangefinder unavailable
 * 
 * @see do_land() - Uses this to set approach altitude for landing with location
 * @see do_payload_place() - Uses this for payload place altitude
 * @see subtract_pos_offsets() - Related function for position offset handling
 */
// sets the target_loc's alt to the vehicle's current alt but does not change target_loc's frame
// in the case of terrain altitudes either the terrain database or the rangefinder may be used
// returns true on success, false on failure
bool ModeAuto::shift_alt_to_current_alt(Location& target_loc) const
{
    // if terrain alt using rangefinder is being used then set alt to current rangefinder altitude
    if ((target_loc.get_alt_frame() == Location::AltFrame::ABOVE_TERRAIN) &&
        (wp_nav->get_terrain_source() == AC_WPNav::TerrainSource::TERRAIN_FROM_RANGEFINDER)) {
        int32_t curr_rngfnd_alt_cm;
        if (copter.get_rangefinder_height_interpolated_cm(curr_rngfnd_alt_cm)) {
            // subtract position offset (if any)
            curr_rngfnd_alt_cm -= pos_control->get_pos_offset_U_cm();
            // wp_nav is using rangefinder so use current rangefinder alt
            target_loc.set_alt_cm(MAX(curr_rngfnd_alt_cm, 200), Location::AltFrame::ABOVE_TERRAIN);
            return true;
        }
        return false;
    }

    // take copy of current location and change frame to match target
    Location currloc = copter.current_loc;
    if (!currloc.change_alt_frame(target_loc.get_alt_frame())) {
        // this could fail due missing terrain database alt
        return false;
    }

    // set target_loc's alt minus position offset (if any)
    target_loc.set_alt_cm(currloc.alt - pos_control->get_pos_offset_U_cm(), currloc.get_alt_frame());
    return true;
}

/**
 * @brief Subtract position controller offsets from target location for consistency
 * 
 * @details Removes position controller offsets from target_loc before sending to position controller.
 *          Position controller maintains offsets for precision landing corrections, obstacle avoidance,
 *          and wind compensation. When constructing new targets, must subtract these offsets to avoid
 *          double-application and accumulated errors. Critical for accurate waypoint navigation and
 *          preventing position drift during missions.
 * 
 *          Why Position Offsets Exist:
 *          - Precision landing: Camera offset corrections
 *          - Obstacle avoidance: Temporary position adjustments
 *          - Wind compensation: Drift corrections
 *          - Position controller tracks: desired_position + offset
 *          - Actual vehicle position includes offset
 * 
 *          Problem - Double Offset:
 *          - Vehicle at position P (includes offset O)
 *          - Create target from current position: T = P
 *          - If we send T to pos_control, it adds O again
 *          - Result: Vehicle moves by O (wrong!)
 *          - Must subtract O before sending
 * 
 *          Solution:
 *          - Get current offsets from position controller
 *          - Subtract from target location
 *          - When pos_control receives target, it adds offset back
 *          - Net effect: Target is where we want it
 * 
 *          When to Call This Function:
 *          - Creating target from current vehicle position
 *          - Using previous waypoint as default location
 *          - Any time location includes position offsets
 *          - Before sending location to waypoint controller
 * 
 *          Position Offset Retrieval:
 *          - pos_ofs_neu_cm = pos_control->get_pos_offset_NEU_cm()
 *          - NEU coordinate frame (North-East-Up)
 *          - In centimeters
 *          - Returns Vector3p (precise position vector)
 * 
 *          NEU vs NED Coordinate Frames:
 *          - Position controller uses NEU (North-East-Up)
 *          - Location class uses NED (North-East-Down)
 *          - Z axis inverted between frames
 *          - Must convert when applying offset
 * 
 *          Unit Conversion:
 *          - Position offsets in centimeters
 *          - Location.offset() expects meters
 *          - Multiply by 0.01 to convert cm → m
 *          - pos_ofs_neu_cm.x * 0.01 = north offset in meters
 * 
 *          Coordinate Frame Conversion:
 *          - NEU: {north_cm, east_cm, up_cm}
 *          - NED: {north_m, east_m, down_m}
 *          - North: Same in both (but cm → m)
 *          - East: Same in both (but cm → m)
 *          - Up → Down: Negate Z (-pos_ofs_neu_cm.z)
 * 
 *          Vector3p Type:
 *          - Precise position vector
 *          - Higher precision than Vector3f
 *          - Used for global positions
 *          - Prevents floating point precision loss
 * 
 *          Offset Application:
 *          - target_loc.offset(-pos_ofs_ned_m)
 *          - Location.offset() adds vector to location
 *          - We subtract offsets, so negate: -pos_ofs_ned_m
 *          - Modifies lat, lon, alt based on offset vector
 * 
 *          How Location.offset() Works:
 *          - Takes NED offset in meters
 *          - Converts to lat/lon/alt changes
 *          - Adjusts location accordingly
 *          - Accounts for earth curvature
 *          - Precise geodetic calculation
 * 
 *          Sign Convention:
 *          - Positive offset means "vehicle is offset in that direction"
 *          - To get base position, subtract offset
 *          - offset(-pos_ofs) removes the offset
 *          - Result is underlying desired position
 * 
 *          Use Case - Current Position as Default:
 *          - Mission command: Loiter at current position
 *          - default_loc = copter.current_loc (includes offsets)
 *          - subtract_pos_offsets(default_loc)
 *          - Now default_loc is true desired position
 *          - Can use for waypoint calculation
 * 
 *          Use Case - Previous Waypoint Default:
 *          - Previous waypoint at 100m north
 *          - Precision landing added 2m east offset
 *          - Get previous waypoint destination: includes offset
 *          - subtract_pos_offsets() removes 2m east
 *          - Get true waypoint position for next calculation
 * 
 *          Example - Precision Landing Offset:
 *          - Landing target detected 3m east of planned position
 *          - pos_control adds offset: {0, 3m, 0}
 *          - Current position shows as 3m east
 *          - Create next waypoint from current position
 *          - Must subtract {0, 3m, 0} first
 *          - Otherwise vehicle would move another 3m east (wrong)
 * 
 *          Example - Obstacle Avoidance:
 *          - Obstacle detected, vehicle moved 5m north
 *          - Offset: {5m, 0, 0}
 *          - Mission continues to next waypoint
 *          - Use current position as starting point
 *          - Subtract {5m, 0, 0} offset
 *          - Waypoint calculation uses pre-avoidance position
 *          - Smooth return to track
 * 
 *          Called From:
 *          - do_nav_wp(): Before creating waypoint target
 *          - do_loiter_unlimited(): Before creating loiter target
 *          - do_circle(): Before creating circle center
 *          - Any function using current position as base
 * 
 *          Relationship to shift_alt_to_current_alt():
 *          - shift_alt_to_current_alt() also subtracts offsets
 *          - But only for altitude (U component)
 *          - This function handles all 3 axes (NEU/NED)
 *          - Both ensure consistent position handling
 * 
 *          Offset Accumulation Prevention:
 *          - Without this: Offsets accumulate over mission
 *          - Vehicle drifts from intended track
 *          - Each waypoint adds previous offset again
 *          - Critical for mission accuracy
 * 
 * @param[in,out] target_loc Location to adjust; offsets subtracted in-place
 * 
 * @note Position controller offsets include precision landing, obstacle avoidance, wind compensation
 * @note Offsets converted from NEU centimeters to NED meters
 * @note Z-axis inverted (Up → Down) during frame conversion
 * @note Called before using location as waypoint controller target
 * @note Prevents double-application of offsets and position drift
 * 
 * @see do_nav_wp() - Calls this before setting waypoint target
 * @see do_loiter_unlimited() - Calls this for loiter position
 * @see shift_alt_to_current_alt() - Related altitude offset handling
 */
// subtract position controller offsets from target location
// should be used when the location will be used as a target for the position controller
void ModeAuto::subtract_pos_offsets(Location& target_loc) const
{
    // subtract position controller offsets from target location
    const Vector3p& pos_ofs_neu_cm = pos_control->get_pos_offset_NEU_cm();
    Vector3p pos_ofs_ned_m = Vector3p{pos_ofs_neu_cm.x * 0.01, pos_ofs_neu_cm.y * 0.01, -pos_ofs_neu_cm.z * 0.01};
    target_loc.offset(-pos_ofs_ned_m);
}

/********************************************************************************/
// Nav (Must) commands
/********************************************************************************/

/**
 * @brief Process NAV_TAKEOFF or NAV_VTOL_TAKEOFF mission command
 * 
 * @details Initiates vertical takeoff to altitude specified in mission command. Used at
 *          mission start or after landing to get vehicle airborne before navigating.
 * 
 *          Takeoff Behavior:
 *          - Vehicle climbs vertically from current position
 *          - Maintains horizontal position (no lateral movement)
 *          - Climbs to altitude specified in cmd.content.location.alt
 *          - Yaw held at current heading (AutoYaw::Mode::HOLD)
 *          - Completes when target altitude reached (verified by verify_takeoff())
 * 
 *          Altitude Frame Support:
 *          - ABOVE_HOME: Altitude relative to home position
 *          - ABOVE_ORIGIN: Altitude relative to EKF origin
 *          - ABOVE_TERRAIN: Altitude above ground level (requires terrain data)
 * 
 *          Delegates to takeoff_start() which handles:
 *          - Altitude frame conversions
 *          - Minimum takeoff altitude enforcement (10cm if landed)
 *          - Position controller initialization
 *          - Auto takeoff controller setup
 *          - WP_NAVALT_MIN altitude tracking
 * 
 * @param[in] cmd Mission command containing takeoff altitude in location.alt field
 *                Altitude frame determined by location.get_alt_frame()
 * 
 * @note NAV_TAKEOFF (22) and NAV_VTOL_TAKEOFF commands handled identically for multicopters
 * @note Should be first command in mission if starting from ground
 * @note Mission initialization rejects switching to Auto if armed on ground without takeoff cmd
 * @see takeoff_start() for detailed takeoff initialization
 * @see verify_takeoff() for completion detection
 * @see takeoff_run() for takeoff execution loop
 */
void ModeAuto::do_takeoff(const AP_Mission::Mission_Command& cmd)
{
    // Set wp navigation target to safe altitude above current position
    takeoff_start(cmd.content.location);
}

/**
 * @brief Extract Location from mission command, filling in zero fields with defaults
 * 
 * @details Mission commands can specify lat/lon/alt as zero to indicate "use current position"
 *          or "use previous waypoint position". This function constructs a complete Location
 *          object from a mission command, substituting values from default_loc when the command
 *          specifies zero. Critical for mission flexibility and relative positioning.
 * 
 *          Zero Field Meaning in Mission Commands:
 *          - lat=0, lon=0: Use default location's horizontal position
 *          - alt=0: Use default location's altitude
 *          - Allows mission commands to be position-relative
 *          - Common for loiter, altitude changes at current position
 * 
 *          default_loc Parameter:
 *          - Usually current vehicle position
 *          - Sometimes previous waypoint position
 *          - Context-dependent based on command sequence
 *          - Caller determines appropriate default
 * 
 *          Latitude/Longitude Handling:
 *          - if (ret.lat == 0 && ret.lng == 0)
 *          - Both must be zero to use default
 *          - Prevents accidental equator (0,0) position
 *          - Copies both lat and lon from default_loc
 * 
 *          Why Both Must Be Zero:
 *          - Lat=0 alone could be equator
 *          - Lon=0 alone could be prime meridian
 *          - Require both zero for "use default" semantic
 *          - Reduces mission planning errors
 * 
 *          Altitude Handling (Complex):
 *          - if (ret.alt == 0): Use default altitude
 *          - Must respect command's altitude frame
 *          - default_loc may be in different frame
 *          - Requires frame conversion
 * 
 *          Altitude Frame Types:
 *          - ABOVE_HOME: Relative to home position
 *          - ABOVE_ORIGIN: Relative to EKF origin
 *          - ABOVE_TERRAIN: Relative to terrain (requires terrain database)
 *          - ABSOLUTE: Mean sea level altitude
 * 
 *          Altitude Conversion Process:
 *          1. Get command's altitude frame: ret.get_alt_frame()
 *          2. Try to convert default_loc to that frame
 *          3. default_loc.get_alt_cm(ret.get_alt_frame(), default_alt)
 *          4. If successful, use converted altitude
 *          5. If fails, fall back to default_loc's native altitude/frame
 * 
 *          Why Conversion May Fail:
 *          - Terrain database unavailable (for ABOVE_TERRAIN)
 *          - No terrain data at location
 *          - EKF origin not set
 *          - Frame conversion not possible in current state
 * 
 *          Fallback Altitude Handling:
 *          - If conversion fails: Use default_loc as-is
 *          - ret.set_alt_cm(default_loc.alt, default_loc.get_alt_frame())
 *          - Ignores command's requested frame
 *          - Uses default_loc's frame and altitude
 *          - Logs error elsewhere for terrain failures
 * 
 *          Use Case - Loiter at Current Position:
 *          - NAV_LOITER_UNLIM with lat=0, lon=0, alt=1000cm ABOVE_HOME
 *          - default_loc = current vehicle position
 *          - Result: Loiter at current lat/lon, altitude 1000cm above home
 *          - Horizontal position from vehicle, altitude from command
 * 
 *          Use Case - Climb at Current Position:
 *          - NAV_WAYPOINT with lat=0, lon=0, alt=5000cm
 *          - default_loc = current position at 2000cm
 *          - Result: Climb to 5000cm at current horizontal position
 * 
 *          Use Case - Land at Next Waypoint:
 *          - NAV_LAND with lat=0, lon=0, alt=0
 *          - default_loc = previous waypoint position
 *          - Result: Land at previous waypoint location
 *          - Complete position from previous waypoint
 * 
 *          Use Case - Terrain Relative Mission:
 *          - NAV_WAYPOINT with lat=X, lon=Y, alt=0, frame=ABOVE_TERRAIN
 *          - default_loc = previous waypoint at 50m above terrain
 *          - Tries to convert 50m terrain alt to ABOVE_TERRAIN at new position
 *          - May fail if terrain data missing
 * 
 *          Mission Planning Workflow:
 *          - Mission planner sets lat=0,lon=0 for "here"
 *          - Or sets lat=0,lon=0,alt=0 for "here at this alt"
 *          - Or lat/lon specified, alt=0 for "there at current alt"
 *          - Flexible mission command construction
 * 
 *          Terrain Database Interaction:
 *          - ABOVE_TERRAIN frame requires terrain data
 *          - get_alt_cm() queries terrain database
 *          - May fail in areas without SRTM data
 *          - Fallback provides safe behavior
 * 
 * @param[in] cmd Mission command containing location (may have zero fields)
 * @param[in] default_loc Location to use for zero fields (typically current or previous position)
 * 
 * @return Complete Location object with no zero fields
 * 
 * @note Both lat and lon must be zero to use default horizontal position
 * @note Altitude conversion respects command's altitude frame
 * @note Falls back to default_loc's native frame if conversion fails
 * @note Terrain frame conversions may fail if terrain database unavailable
 * 
 * @see do_nav_wp() - Calls this to process waypoint commands
 * @see do_loiter_unlimited() - Uses this for loiter location
 * @see shift_alt_to_current_alt() - Related altitude adjustment function
 */
// return the Location portion of a command.  If the command's lat and lon and/or alt are zero the default_loc's lat,lon and/or alt are returned instead
Location ModeAuto::loc_from_cmd(const AP_Mission::Mission_Command& cmd, const Location& default_loc) const
{
    Location ret(cmd.content.location);

    // use current lat, lon if zero
    if (ret.lat == 0 && ret.lng == 0) {
        ret.lat = default_loc.lat;
        ret.lng = default_loc.lng;
    }
    // use default altitude if not provided in cmd
    if (ret.alt == 0) {
        // set to default_loc's altitude but in command's alt frame
        // note that this may use the terrain database
        int32_t default_alt;
        if (default_loc.get_alt_cm(ret.get_alt_frame(), default_alt)) {
            ret.set_alt_cm(default_alt, ret.get_alt_frame());
        } else {
            // default to default_loc's altitude and frame
            ret.set_alt_cm(default_loc.alt, default_loc.get_alt_frame());
        }
    }
    return ret;
}

/**
 * @brief Process NAV_WAYPOINT mission command to fly to specified location
 * 
 * @details Initiates autonomous flight to waypoint coordinates using AC_WPNav controller.
 *          Supports straight-line navigation with optional loiter delay at destination.
 *          Can use terrain-relative altitudes and handles position offsets.
 * 
 *          Navigation Flow:
 *          1. Calculate default location (current position with offsets subtracted)
 *          2. If coming from previous waypoint and reached, use that as default
 *          3. Extract waypoint location from command using loc_from_cmd()
 *          4. Initialize waypoint navigation via wp_start()
 *          5. Set loiter delay timer from cmd.p1 (seconds to loiter at waypoint)
 *          6. Set next destination for path smoothing (if no delay and next cmd is waypoint/spline)
 * 
 *          Zero Coordinate Handling:
 *          - If cmd lat/lon are zero, uses current position (fly to same lat/lon, change alt)
 *          - If cmd altitude is zero, uses current altitude in command's altitude frame
 *          - loc_from_cmd() handles these substitutions with default_loc parameter
 * 
 *          Position Offsets:
 *          - Subtracts position controller offsets from default location
 *          - Ensures waypoint coordinates account for any controller adjustments
 *          - subtract_pos_offsets() modifies location by NEU offset
 * 
 *          Waypoint Delay (Loiter):
 *          - cmd.p1 parameter specifies loiter time in seconds at destination
 *          - loiter_time tracks when waypoint reached (millis())
 *          - loiter_time_max stores required delay duration
 *          - verify_nav_wp() checks if delay completed before advancing mission
 *          - If delay > 0, vehicle stops at waypoint (next waypoint not pre-loaded)
 * 
 *          Path Smoothing (Corner Cutting):
 *          - set_next_wp() pre-loads next waypoint destination into wp_nav
 *          - Allows smooth path through waypoint if no delay
 *          - Works with both straight waypoints and spline waypoints
 *          - Only if next command is compatible (NAV_WAYPOINT, NAV_SPLINE_WAYPOINT, etc.)
 *          - Does not smooth if delay, loiter, land, RTL, or takeoff follows
 * 
 *          Altitude Frame Support:
 *          - ABOVE_HOME: Relative to home position
 *          - ABOVE_ORIGIN: Relative to EKF origin  
 *          - ABOVE_TERRAIN: Terrain-relative (requires terrain data or rangefinder)
 * 
 *          Terrain Handling:
 *          - If terrain altitude requested but data unavailable, triggers failsafe
 *          - copter.failsafe_terrain_on_event() called on terrain data failure
 *          - Ensures mission doesn't proceed with incorrect altitude reference
 * 
 *          Speed Overrides:
 *          - Uses desired_speed_override if set by previous DO_CHANGE_SPEED command
 *          - wp_start() applies speed overrides to wp_nav controller
 *          - Overrides persist until changed by another DO_CHANGE_SPEED
 * 
 *          Yaw Control:
 *          - Normally sets yaw to default mode (point toward waypoint)
 *          - Preserves ROI mode if active (continues pointing at region of interest)
 *          - Preserves FIXED yaw if WP_YAW_BEHAVIOR set to none
 *          - auto_yaw.set_mode_to_default(false) called by wp_start()
 * 
 * @param[in] cmd Mission command with waypoint coordinates in content.location
 *                - location.lat, location.lng: Target coordinates (centidegrees)
 *                - location.alt: Target altitude in specified frame
 *                - location.get_alt_frame(): Altitude reference frame
 *                - p1: Loiter time at waypoint in seconds (0 = no delay)
 * 
 * @note Called by start_command() when MAV_CMD_NAV_WAYPOINT (16) encountered
 * @note verify_nav_wp() checks completion (reached + delay elapsed)
 * @note wp_run() executes waypoint navigation at 100Hz+
 * @warning Terrain data failure triggers failsafe - mission will not advance
 * 
 * @see wp_start() for waypoint navigation initialization
 * @see loc_from_cmd() for coordinate processing with defaults
 * @see set_next_wp() for path smoothing to next waypoint
 * @see verify_nav_wp() for waypoint completion detection
 * @see wp_run() for waypoint navigation execution loop
 */
void ModeAuto::do_nav_wp(const AP_Mission::Mission_Command& cmd)
{
    // calculate default location used when lat, lon or alt is zero
    Location default_loc = copter.current_loc;

    // subtract position offsets
    subtract_pos_offsets(default_loc);

    if (wp_nav->is_active() && wp_nav->reached_wp_destination()) {
        if (!wp_nav->get_wp_destination_loc(default_loc)) {
            // this should never happen
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        }
    }

    // get waypoint's location from command and send to wp_nav
    const Location target_loc = loc_from_cmd(cmd, default_loc);

    if (!wp_start(target_loc)) {
        // failure to set next destination can only be because of missing terrain data
        copter.failsafe_terrain_on_event();
        return;
    }

    // this will be used to remember the time in millis after we reach or pass the WP.
    loiter_time = 0;
    // this is the delay, stored in seconds
    loiter_time_max = cmd.p1;

    // set next destination if necessary
    if (!set_next_wp(cmd, target_loc)) {
        // failure to set next destination can only be because of missing terrain data
        copter.failsafe_terrain_on_event();
        return;
    }
}

/**
 * @brief Pre-load next waypoint for smooth cornering and trajectory planning
 * 
 * @details Called after setting current waypoint destination to optimize vehicle path by
 *          informing waypoint controller of the next navigation target. Enables smooth
 *          cornering, speed profiling, and spline path generation. Vehicle can plan ahead
 *          to reduce stopping at waypoints and create smoother mission execution.
 * 
 *          Purpose - Smooth Trajectory Generation:
 *          - AC_WPNav can plan smoother paths if it knows next waypoint
 *          - Allows corner cutting and speed profiling
 *          - Reduces stops at intermediate waypoints
 *          - Creates more efficient flight paths
 *          - Better for aerial photography and survey
 * 
 *          When NOT to Add Next Waypoint:
 *          1. Current command has delay (current_cmd.p1 > 0)
 *             - Vehicle must stop at waypoint to wait
 *             - No smooth transition allowed
 *             - Return true immediately (success, no next wp needed)
 *          2. No more navigation commands in mission
 *             - mission.get_next_nav_cmd() returns false
 *             - This is the last waypoint
 *             - Return true (success, end of mission)
 * 
 *          Delay Parameter (current_cmd.p1):
 *          - p1 field in mission command
 *          - Seconds to loiter at waypoint
 *          - If > 0, vehicle stops completely
 *          - Next waypoint not relevant until delay completes
 *          - Smooth cornering disabled
 * 
 *          Navigation Command Types:
 *          - Only NAV commands matter (NAV_WAYPOINT, NAV_LAND, etc.)
 *          - DO and CONDITION commands ignored by get_next_nav_cmd()
 *          - Only consider commands that move vehicle
 *          - Ensures proper trajectory planning
 * 
 *          Commands That Allow Next Waypoint:
 *          - MAV_CMD_NAV_WAYPOINT: Standard waypoint, smooth transition
 *          - MAV_CMD_NAV_LOITER_UNLIM: Loiter position, can pre-plan
 *          - MAV_CMD_NAV_LOITER_TIME: Timed loiter, can pre-plan
 *          - MAV_CMD_NAV_PAYLOAD_PLACE: Payload place, can pre-plan
 *          - MAV_CMD_NAV_SPLINE_WAYPOINT: Spline path, requires next point
 * 
 *          Waypoint Next Destination:
 *          - dest_loc = loc_from_cmd(current_cmd, default_loc)
 *          - Current waypoint destination (corrected for zeros)
 *          - next_dest_loc = loc_from_cmd(next_cmd, dest_loc)
 *          - Next waypoint uses current as default
 *          - wp_nav->set_wp_destination_next_loc(next_dest_loc)
 *          - Informs controller of next target
 * 
 *          Spline Waypoint Special Handling:
 *          - Spline requires current, next, and next-next points
 *          - Enables smooth curve generation
 *          - get_spline_from_cmd() extracts all needed points
 *          - next_next_dest_loc_is_spline: Is next-next also spline?
 *          - wp_nav->set_spline_destination_next_loc()
 *          - More complex than straight waypoints
 * 
 *          Commands That Force Stop:
 *          - MAV_CMD_NAV_LAND: Must stop before landing
 *          - MAV_CMD_NAV_VTOL_LAND: VTOL landing, must stop
 *          - MAV_CMD_NAV_LOITER_TURNS: Circle loiter, must stop
 *          - MAV_CMD_NAV_RETURN_TO_LAUNCH: RTL, must stop
 *          - MAV_CMD_NAV_TAKEOFF: Takeoff, must stop
 *          - MAV_CMD_NAV_VTOL_TAKEOFF: VTOL takeoff, must stop
 * 
 *          Why Stop Before Landing:
 *          - Altitude frame may change (ABOVE_HOME → ABOVE_TERRAIN)
 *          - Landing requires precise positioning
 *          - No smooth cornering into landing
 *          - Safety-critical to stop and assess
 * 
 *          Why Stop Before RTL:
 *          - RTL may change altitude significantly
 *          - Return path calculation needed
 *          - Rally point selection
 *          - Different control logic
 * 
 *          Why Stop Before Takeoff:
 *          - Takeoff from ground requires full stop
 *          - Cannot corner into takeoff
 *          - Must be armed and ready
 * 
 *          Default Case - Unknown Commands:
 *          - For unsupported commands it is safer to stop
 *          - Unknown behavior, don't pre-plan
 *          - Conservative approach
 *          - Future command additions won't break behavior
 * 
 *          Return Value:
 *          - true: Success (next wp set, or not needed, or no next cmd)
 *          - false: Failure (terrain data unavailable)
 *          - Failure only from terrain database access
 * 
 *          Terrain Data Failure:
 *          - wp_nav->set_wp_destination_next_loc() may fail
 *          - Happens if next waypoint is ABOVE_TERRAIN
 *          - Terrain database has no data at location
 *          - Caller triggers terrain failsafe
 * 
 *          Corner Cutting Behavior:
 *          - If next wp known, AC_WPNav reduces deceleration
 *          - Doesn't stop completely at waypoint
 *          - Flies smooth arc through waypoint
 *          - Maintains higher speed
 *          - Much more efficient
 * 
 *          Speed Profiling:
 *          - Knows distance to next waypoint
 *          - Can calculate optimal speed through current waypoint
 *          - Smooth acceleration/deceleration
 *          - Better battery efficiency
 * 
 *          Use Case - Survey Mission:
 *          - 50 waypoints in grid pattern
 *          - No delays between waypoints
 *          - Each waypoint pre-loads next
 *          - Smooth flight, no stops
 *          - Faster mission completion
 * 
 *          Use Case - Waypoint with Delay:
 *          - Waypoint with p1=10 (10 second delay)
 *          - Next waypoint NOT pre-loaded
 *          - Vehicle stops completely
 *          - Waits 10 seconds
 *          - Then proceeds to next
 * 
 *          Use Case - Landing Approach:
 *          - Current: NAV_WAYPOINT (approach point)
 *          - Next: NAV_LAND (landing point)
 *          - Next wp NOT pre-loaded (landing must stop)
 *          - Vehicle stops at approach point
 *          - Assesses situation
 *          - Then initiates landing
 * 
 * @param[in] current_cmd Current mission command being executed
 * @param[in] default_loc Destination from current_cmd (corrected for zero lat/lon/alt)
 * 
 * @return true if next waypoint set successfully or not needed, false if terrain data unavailable
 * 
 * @note Returns true (success) if current command has delay - no next wp needed
 * @note Returns true (success) if no more nav commands - end of mission
 * @note Returns false only if terrain database unavailable for next waypoint
 * @note Vehicle stops at waypoint if delay specified or next command requires stop
 * 
 * @see loc_from_cmd() - Extracts location from command
 * @see get_spline_from_cmd() - Gets spline waypoint data
 * @see AC_WPNav::set_wp_destination_next_loc() - Sets next waypoint in controller
 */
// checks the next mission command and adds it as a destination if necessary
// supports both straight line and spline waypoints
// cmd should be the current command
// default_loc should be the destination from the current_cmd but corrected for cases where user set lat, lon or alt to zero
// returns true on success, false on failure which should only happen due to a failure to retrieve terrain data
bool ModeAuto::set_next_wp(const AP_Mission::Mission_Command& current_cmd, const Location &default_loc)
{
    // do not add next wp if current command has a delay meaning the vehicle will stop at the destination
    if (current_cmd.p1 > 0) {
        return true;
    }

    // do not add next wp if there are no more navigation commands
    AP_Mission::Mission_Command next_cmd;
    if (!mission.get_next_nav_cmd(current_cmd.index+1, next_cmd)) {
        return true;
    }

    // whether vehicle should stop at the target position depends upon the next command
    switch (next_cmd.id) {
    case MAV_CMD_NAV_WAYPOINT:
    case MAV_CMD_NAV_LOITER_UNLIM:
#if AP_MISSION_NAV_PAYLOAD_PLACE_ENABLED
    case MAV_CMD_NAV_PAYLOAD_PLACE:
#endif
    case MAV_CMD_NAV_LOITER_TIME: {
        const Location dest_loc = loc_from_cmd(current_cmd, default_loc);
        const Location next_dest_loc = loc_from_cmd(next_cmd, dest_loc);
        return wp_nav->set_wp_destination_next_loc(next_dest_loc);
    }
    case MAV_CMD_NAV_SPLINE_WAYPOINT: {
        // get spline's location and next location from command and send to wp_nav
        Location next_dest_loc, next_next_dest_loc;
        bool next_next_dest_loc_is_spline;
        get_spline_from_cmd(next_cmd, default_loc, next_dest_loc, next_next_dest_loc, next_next_dest_loc_is_spline);
        return wp_nav->set_spline_destination_next_loc(next_dest_loc, next_next_dest_loc, next_next_dest_loc_is_spline);
    }
    case MAV_CMD_NAV_VTOL_LAND:
    case MAV_CMD_NAV_LAND:
        // stop because we may change between rel,abs and terrain alt types
    case MAV_CMD_NAV_LOITER_TURNS:
    case MAV_CMD_NAV_RETURN_TO_LAUNCH:
    case MAV_CMD_NAV_VTOL_TAKEOFF:
    case MAV_CMD_NAV_TAKEOFF:
        // always stop for RTL and takeoff commands
    default:
        // for unsupported commands it is safer to stop
        break;
    }

    return true;
}

/**
 * @brief Process NAV_LAND or NAV_VTOL_LAND mission command
 * 
 * @details Initiates autonomous landing sequence. If landing coordinates provided, first
 *          flies to location at current altitude, then descends. If no coordinates provided,
 *          lands immediately at current position.
 * 
 *          Landing Modes:
 * 
 *          Two-Phase Landing (if lat/lon specified in command):
 *          1. FlyToLocation State:
 *             - Flies horizontally to landing coordinates
 *             - Maintains current altitude during transit
 *             - Uses shift_alt_to_current_alt() to keep altitude constant
 *             - verify_land() transitions to Descending when location reached
 *          2. Descending State:
 *             - land_start() initializes descent controllers
 *             - Vertical descent at LAND_SPEED
 *             - Ground detection and motor disarm
 * 
 *          Direct Landing (if lat=0 and lon=0):
 *          - Skips FlyToLocation phase
 *          - Immediately enters Descending state
 *          - Lands at current position
 *          - land_start() called directly
 * 
 *          Altitude Handling for FlyToLocation:
 *          - shift_alt_to_current_alt() converts target location to current altitude
 *          - Preserves altitude frame of command (ABOVE_HOME, ABOVE_TERRAIN, etc.)
 *          - For terrain altitudes, uses terrain database or rangefinder
 *          - If terrain data unavailable, falls back to alt-above-home with error log
 * 
 *          Landing Controller Initialization (land_start()):
 *          - Sets horizontal position control limits (speed/accel from wp_nav)
 *          - Initializes NE (horizontal) position controller if not active
 *          - Sets vertical position control limits (descent rate from wp_nav)
 *          - Initializes U (vertical) position controller if not active
 *          - Sets yaw mode to HOLD (maintain current heading)
 *          - Deploys landing gear (if equipped and AP_LANDINGGEAR_ENABLED)
 *          - Resets land repositioning flag (copter.ap.land_repo_active)
 *          - Clears precision landing flag (copter.ap.prec_land_active)
 * 
 *          Landing Execution (land_run()):
 *          - Called at 100Hz when in LAND submode
 *          - Maintains horizontal position hold
 *          - Descends at LAND_SPEED (default 50 cm/s)
 *          - Detects ground contact via thrust/IMU
 *          - Sets copter.ap.land_complete when touchdown detected
 * 
 *          Precision Landing Support:
 *          - If target detected, uses precision landing (IR-LOCK or similar)
 *          - land_run() calls land_run_normal_or_precland()
 *          - Guides vehicle to land on target beacon
 *          - Falls back to normal landing if target lost
 * 
 *          Landing Complete Detection (verify_land()):
 *          - Checks copter.ap.land_complete flag (from land detector)
 *          - Verifies motors at GROUND_IDLE spool state
 *          - Returns true when both conditions met
 *          - Mission advances to next command or disarms
 * 
 *          Mission Continuation vs Disarm:
 *          - Default: Disarms after landing (via AP_Arming::Method::LANDED)
 *          - MIS_OPTIONS can allow mission continuation after landing
 *          - mission.continue_after_land_check_for_takeoff() checks configuration
 *          - If continuing, verify_land() returns false to keep mission active
 * 
 *          Terrain Data Failure Handling:
 *          - If terrain altitude requested but unavailable:
 *            * Logs TERRAIN/MISSING_TERRAIN_DATA error
 *            * Sends critical GCS message
 *            * Falls back to ABOVE_HOME altitude frame
 *            * For FlyToLocation failure: triggers failsafe_terrain_on_event()
 *          - Ensures safe landing even with terrain data issues
 * 
 *          State Tracking:
 *          - state variable (enum State) tracks landing phase
 *          - State::FlyToLocation: Flying to landing coordinates
 *          - State::Descending: Descending to ground
 *          - verify_land() uses state to determine completion logic
 * 
 * @param[in] cmd Mission command containing optional landing location
 *                - content.location.lat: Landing latitude (0 = current position)
 *                - content.location.lng: Landing longitude (0 = current position)
 *                - content.location.alt: Not used (lands at ground level)
 *                - Location altitude frame affects FlyToLocation altitude reference
 * 
 * @note Called by start_command() when NAV_LAND (21) or NAV_VTOL_LAND encountered
 * @note NAV_LAND and NAV_VTOL_LAND handled identically for multicopters
 * @note verify_land() checks landing completion
 * @note land_run() executes landing control at 100Hz+
 * @warning Vehicle will disarm automatically after landing unless MIS_OPTIONS configured
 * @warning Terrain data failure on FlyToLocation triggers terrain failsafe
 * 
 * @see land_start() for landing controller initialization
 * @see land_run() for landing execution loop
 * @see verify_land() for landing completion detection
 * @see shift_alt_to_current_alt() for altitude frame handling
 */
void ModeAuto::do_land(const AP_Mission::Mission_Command& cmd)
{
    // To-Do: check if we have already landed

    // if location provided we fly to that location at current altitude
    if (cmd.content.location.lat != 0 || cmd.content.location.lng != 0) {
        // set state to fly to location
        state = State::FlyToLocation;

        // convert cmd to location class
        Location target_loc(cmd.content.location);
        if (!shift_alt_to_current_alt(target_loc)) {
            // this can only fail due to missing terrain database alt or rangefinder alt
            // use current alt-above-home and report error
            target_loc.set_alt_cm(copter.current_loc.alt, Location::AltFrame::ABOVE_HOME);
            LOGGER_WRITE_ERROR(LogErrorSubsystem::TERRAIN, LogErrorCode::MISSING_TERRAIN_DATA);
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Land: no terrain data, using alt-above-home");
        }

        if (!wp_start(target_loc)) {
            // failure to set next destination can only be because of missing terrain data
            copter.failsafe_terrain_on_event();
            return;
        }
    } else {
        // set landing state
        state = State::Descending;

        // initialise landing controller
        land_start();
    }
}

/**
 * @brief Process NAV_LOITER_UNLIM mission command to loiter indefinitely
 * 
 * @details Initiates position hold at specified location with no time limit or end condition.
 *          Mission will not advance past this command unless manually commanded or mission
 *          is stopped/restarted. Useful for indefinite station-keeping or waiting for
 *          external intervention.
 * 
 *          Loiter Behavior:
 *          - Flies to target location if not already there
 *          - Holds position indefinitely at target altitude
 *          - Maintains heading (yaw mode typically set by mission context)
 *          - verify_loiter_unlimited() always returns false (never completes)
 *          - Mission pauses at this command until external action taken
 * 
 *          Location Determination:
 *          1. Calculate default location (current position with offsets subtracted)
 *          2. If transitioning from previous waypoint that was reached, use that location
 *          3. Extract target location from command using loc_from_cmd()
 *          4. Zero coordinates (lat=0, lon=0, alt=0) replaced with defaults
 * 
 *          Zero Coordinate Substitution:
 *          - lat/lon = 0: Uses current horizontal position
 *          - alt = 0: Uses current altitude in command's altitude frame
 *          - Allows loitering at current position with "stay here" command
 * 
 *          Position Controller Initialization:
 *          - wp_start() initializes AC_WPNav waypoint controller
 *          - Sets target location as waypoint destination
 *          - Once reached, maintains position hold
 *          - loiter_run() executes position hold at 100Hz
 * 
 *          Altitude Frame Support:
 *          - ABOVE_HOME: Relative to home position
 *          - ABOVE_ORIGIN: Relative to EKF origin
 *          - ABOVE_TERRAIN: Terrain-relative (requires terrain data)
 * 
 *          Mission Flow:
 *          - Mission advances TO this command normally
 *          - Mission STOPS at this command (never advances past)
 *          - Ways to exit:
 *            * Manual mode change by pilot
 *            * Mission stop/restart command
 *            * DO_JUMP command earlier in mission
 *            * GCS command to change mission or mode
 *            * Failsafe event (battery, geofence, etc.)
 * 
 *          Yaw Control:
 *          - Yaw mode NOT set by this function (note in original comment)
 *          - Typically inherits from previous command or mission context
 *          - May point toward target during approach
 *          - Holds current heading once position reached
 * 
 *          Typical Use Cases:
 *          - Survey loiter points requiring manual data collection
 *          - Waiting for external event before proceeding
 *          - Emergency mission abort to safe holding position
 *          - Development/testing: pause mission for inspection
 * 
 *          Terrain Failure Handling:
 *          - If terrain altitude requested but data unavailable
 *          - copter.failsafe_terrain_on_event() triggered
 *          - Prevents mission from continuing with incorrect altitude
 * 
 * @param[in] cmd Mission command with loiter location in content.location
 *                - location.lat, location.lng: Target coordinates (0 = current position)
 *                - location.alt: Target altitude (0 = current altitude in cmd frame)
 *                - location.get_alt_frame(): Altitude reference frame
 * 
 * @return void (no return value, but sets up indefinite loiter state)
 * 
 * @note Called by start_command() when NAV_LOITER_UNLIM (17) encountered
 * @note verify_loiter_unlimited() always returns false (command never completes)
 * @note Mission will not progress past this command without external intervention
 * @note loiter_run() executes position hold control at 100Hz
 * @warning Mission effectively stops at this command - use with caution
 * @warning No automatic timeout - vehicle will loiter until mode changed
 * 
 * @see loc_from_cmd() for coordinate processing with zero substitution
 * @see wp_start() for position hold initialization
 * @see verify_loiter_unlimited() for verification (always false)
 * @see loiter_run() for position hold execution loop
 * @see do_loiter_time() for time-limited loiter variant
 */
void ModeAuto::do_loiter_unlimited(const AP_Mission::Mission_Command& cmd)
{
    // calculate default location used when lat, lon or alt is zero
    Location default_loc = copter.current_loc;

    // subtract position offsets
    subtract_pos_offsets(default_loc);

    // use previous waypoint destination as default if available
    if (wp_nav->is_active() && wp_nav->reached_wp_destination()) {
        if (!wp_nav->get_wp_destination_loc(default_loc)) {
            // this should never happen
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        }
    }

    // get waypoint's location from command and send to wp_nav
    const Location target_loc = loc_from_cmd(cmd, default_loc);

    // start way point navigator and provide it the desired location
    if (!wp_start(target_loc)) {
        // failure to set next destination can only be because of missing terrain data
        copter.failsafe_terrain_on_event();
        return;
    }
}

/**
 * @brief Process NAV_LOITER_TURNS mission command to circle a location
 * 
 * @details Initiates circular flight pattern around specified center point for a defined
 *          number of complete rotations. Supports both clockwise (CW) and counter-clockwise
 *          (CCW) directions with configurable radius.
 * 
 *          Circle Navigation Phases:
 *          1. CIRCLE_MOVE_TO_EDGE submode: If more than 3m from circle edge
 *             - Flies to nearest point on circle perimeter
 *             - wp_run() controls waypoint navigation to edge
 *             - Yaw points toward target (if outside circle) or held (if inside)
 *             - verify_circle() monitors edge arrival
 *          2. CIRCLE submode: Once at edge (within 3m)
 *             - Begins circular motion around center
 *             - circle_run() executes circular trajectory
 *             - Yaw follows circle tangent (AutoYaw::Mode::CIRCLE)
 *             - Counts complete rotations for completion detection
 * 
 *          Circle Parameters:
 *          - Center: Specified in cmd.content.location (lat, lon, alt)
 *          - Radius: Encoded in high byte of cmd.p1 parameter (meters)
 *            * Special handling: If type_specific_bits bit 0 set, radius *= 10
 *            * Allows larger circles (up to 2550m) with extended encoding
 *          - Direction: cmd.content.location.loiter_ccw flag
 *            * true = counter-clockwise (CCW)
 *            * false = clockwise (CW)
 *          - Turns: Number of complete circles before advancing (from cmd.get_loiter_turns())
 * 
 *          Radius Calculation:
 *          - Standard: HIGHBYTE(cmd.p1) gives radius in meters (0-255m)
 *          - Extended: If type_specific_bits & (1U << 0), multiply by 10 (0-2550m)
 *          - Zero radius: Uses default from CIRCLE_RADIUS parameter
 * 
 *          Direction Control:
 *          - Circle rate parameter (CIRCLE_RATE degrees/second)
 *          - Positive rate = clockwise
 *          - Negative rate = counter-clockwise
 *          - circle_direction_ccw flag sets sign of rate
 * 
 *          Edge Detection (3m threshold):
 *          - If vehicle is >3m from circle edge: Move to edge first
 *          - If vehicle is ≤3m from edge: Start circling immediately
 *          - Prevents abrupt transitions when already near circle
 * 
 *          Circle Initialization (circle_movetoedge_start()):
 *          - Sets circle_nav center to target location
 *          - Configures circle radius (or uses default if zero)
 *          - Sets direction by adjusting rate sign
 *          - Calculates closest point on circle edge
 *          - Determines if move-to-edge phase needed
 * 
 *          Move-to-Edge Navigation:
 *          - Converts circle edge point to Location
 *          - Uses wp_nav to fly straight line to edge
 *          - Matches altitude frame to circle center
 *          - Yaw behavior depends on position:
 *            * Outside circle (>radius): Point toward edge (AutoYaw default)
 *            * Inside circle: Hold yaw to avoid spinning
 *            * Near circle (within 5m of radius): Hold yaw
 * 
 *          Circle Execution (circle_start()):
 *          - Initializes circle_nav controller with center and rate
 *          - Sets yaw mode to CIRCLE (follows tangent direction)
 *          - Unless ROI active (then maintains ROI pointing)
 *          - Transitions to CIRCLE submode
 *          - circle_run() takes over at 100Hz
 * 
 *          Turn Counting:
 *          - circle_last_num_complete tracks progress
 *          - verify_circle() monitors circle_nav->get_angle_total_rad()
 *          - Completes when angle_total >= turns * 2π radians
 *          - GCS message sent at start of each new rotation
 * 
 *          Altitude Control:
 *          - Maintains altitude specified in circle center location
 *          - Altitude frame from command (ABOVE_HOME, ABOVE_TERRAIN, etc.)
 *          - Vertical position controller holds altitude during circle
 * 
 *          Default Location Handling:
 *          - Uses current position minus offsets as default
 *          - Zero coordinates (lat/lon) replaced with current position
 *          - loc_from_cmd() performs substitution
 * 
 *          Circle Rate Parameter:
 *          - CIRCLE_RATE: Rotation speed in degrees/second
 *          - Higher rate = faster circles, tighter turns
 *          - Limited by attitude controller lean angle limits
 *          - Typical values: 10-45 deg/s
 * 
 * @param[in] cmd Mission command with circle parameters
 *                - p1: High byte contains radius (0-255m, or 0-2550m if extended)
 *                - type_specific_bits bit 0: If set, multiply radius by 10
 *                - content.location: Circle center (lat, lon, alt)
 *                - content.location.loiter_ccw: Direction (true=CCW, false=CW)
 *                - get_loiter_turns(): Number of complete circles
 * 
 * @return void (initializes circle state, execution via run())
 * 
 * @note Called by start_command() when NAV_LOITER_TURNS (18) encountered
 * @note 3m threshold determines if move-to-edge phase needed
 * @note verify_circle() checks completion based on total angle rotated
 * @note circle_run() executes circular trajectory at 100Hz
 * @warning Circle radius must be appropriate for vehicle speed and turn capability
 * @warning Altitude must clear terrain around entire circle perimeter
 * 
 * @see circle_movetoedge_start() for edge approach initialization
 * @see circle_start() for circle controller initialization
 * @see circle_run() for circular trajectory execution
 * @see verify_circle() for turn completion detection
 */
void ModeAuto::do_circle(const AP_Mission::Mission_Command& cmd)
{
    // calculate default location used when lat, lon or alt is zero
    Location default_loc = copter.current_loc;

    // subtract position offsets
    subtract_pos_offsets(default_loc);

    const Location circle_center = loc_from_cmd(cmd, default_loc);

    // calculate radius
    uint16_t circle_radius_m = HIGHBYTE(cmd.p1); // circle radius held in high byte of p1
    if (cmd.id == MAV_CMD_NAV_LOITER_TURNS &&
        cmd.type_specific_bits & (1U << 0)) {
        // special storage handling allows for larger radii
        circle_radius_m *= 10;
    }

    // true if circle should be ccw
    const bool circle_direction_ccw = cmd.content.location.loiter_ccw;

    // move to edge of circle (verify_circle) will ensure we begin circling once we reach the edge
    circle_movetoedge_start(circle_center, circle_radius_m, circle_direction_ccw);

    circle_last_num_complete = -1;
}

/**
 * @brief Process NAV_LOITER_TIME mission command to loiter for specified duration
 * 
 * @details Initiates position hold at target location for a specific time period, then
 *          automatically advances to next mission command. Time-limited variant of
 *          NAV_LOITER_UNLIM.
 * 
 *          Loiter Time Behavior:
 *          - Flies to target location if not already there
 *          - Holds position at target altitude
 *          - Starts timer when waypoint destination reached
 *          - Advances to next command after time expires
 *          - Maintains heading (yaw mode typically set by mission context)
 * 
 *          Implementation Strategy:
 *          - Reuses do_loiter_unlimited() for initial setup
 *          - Sets loiter_time = 0 (timer not started yet)
 *          - Sets loiter_time_max = cmd.p1 (duration in seconds)
 *          - verify_loiter_time() starts timer and checks expiration
 * 
 *          Timer Mechanism:
 *          1. loiter_time initialized to 0 (timer inactive)
 *          2. verify_loiter_time() called at 10Hz during mission execution
 *          3. When wp_nav->reached_wp_destination() true, timer starts
 *          4. loiter_time set to millis() (current time in milliseconds)
 *          5. Each verify call checks: (millis() - loiter_time) >= loiter_time_max
 *          6. Returns true when time expires, advancing mission
 * 
 *          Location Determination (via do_loiter_unlimited):
 *          - Calculate default location (current position with offsets subtracted)
 *          - If transitioning from previous waypoint, use that location
 *          - Extract target location from command using loc_from_cmd()
 *          - Zero coordinates replaced with current position/altitude
 * 
 *          Zero Coordinate Substitution:
 *          - lat/lon = 0: Uses current horizontal position
 *          - alt = 0: Uses current altitude in command's altitude frame
 *          - Allows "loiter here for X seconds" command
 * 
 *          Time Parameter:
 *          - cmd.p1: Loiter duration in seconds (integer)
 *          - Typical range: 1-3600 seconds (1 second to 1 hour)
 *          - Zero time: Immediately advances (no loiter)
 *          - Timer doesn't start until position reached
 * 
 *          Altitude Frame Support:
 *          - ABOVE_HOME: Relative to home position
 *          - ABOVE_ORIGIN: Relative to EKF origin
 *          - ABOVE_TERRAIN: Terrain-relative (requires terrain data)
 * 
 *          Mission Flow:
 *          - Approaches target location using wp_run()
 *          - Switches to loiter_run() when destination reached
 *          - Starts countdown timer
 *          - verify_loiter_time() returns true when time expires
 *          - Mission automatically advances to next command
 * 
 *          Waypoint Completion Notification:
 *          - No tone played when arriving at loiter point
 *          - Waypoint completion tone played when time expires
 *          - GCS message sent: "Reached command #X"
 *          - Keeps pilot informed of mission progress
 * 
 *          Yaw Control:
 *          - Yaw mode NOT set by this function (note in original comment)
 *          - Typically inherits from previous command or mission context
 *          - May point toward target during approach
 *          - Holds current heading during loiter
 * 
 *          Typical Use Cases:
 *          - Observation point with timed data collection
 *          - Waypoint with required dwell time (regulations)
 *          - Allowing settling time for sensor readings
 *          - Coordinated timing with other vehicles
 *          - Photo capture at specific locations with stabilization delay
 * 
 *          Difference from NAV_LOITER_UNLIM:
 *          - NAV_LOITER_UNLIM: Never advances (indefinite)
 *          - NAV_LOITER_TIME: Advances after timeout (time-limited)
 *          - Both use same loiter position control
 * 
 *          Early Termination:
 *          - Manual mode change by pilot
 *          - Mission stop/restart command
 *          - Failsafe event (battery, geofence, etc.)
 *          - GCS command to skip to next waypoint
 * 
 *          Terrain Failure Handling:
 *          - If terrain altitude requested but data unavailable
 *          - copter.failsafe_terrain_on_event() triggered
 *          - Mission halts to prevent incorrect altitude
 * 
 * @param[in] cmd Mission command with loiter location and duration
 *                - p1: Loiter duration in seconds
 *                - content.location: Target position (lat, lon, alt)
 *                - Zero coordinates use current position/altitude
 * 
 * @return void (sets up timed loiter state, timer starts when position reached)
 * 
 * @note Called by start_command() when NAV_LOITER_TIME (19) encountered
 * @note Timer starts only after reaching waypoint destination
 * @note verify_loiter_time() checks timer and returns true when expired
 * @note loiter_run() executes position hold control at 100Hz
 * @note Yaw mode not modified by this function (set by mission context)
 * 
 * @see do_loiter_unlimited() for initial loiter setup
 * @see verify_loiter_time() for timer checking and completion detection
 * @see loiter_run() for position hold execution loop
 * @see loc_from_cmd() for coordinate processing with zero substitution
 * @see wp_start() for position hold initialization
 */
void ModeAuto::do_loiter_time(const AP_Mission::Mission_Command& cmd)
{
    // re-use loiter unlimited
    do_loiter_unlimited(cmd);

    // setup loiter timer
    loiter_time     = 0;
    loiter_time_max = cmd.p1;     // units are (seconds)
}

/**
 * @brief Process NAV_LOITER_TO_ALT mission command to loiter while climbing/descending to target altitude
 * 
 * @details Flies to horizontal position, then loiters while changing altitude to reach target.
 *          Command completes when both horizontal position AND altitude target are achieved.
 *          Useful for altitude changes at specific locations.
 * 
 *          Two-Phase Operation:
 *          1. Horizontal Phase: Navigate to target lat/lon position
 *             - Uses wp_run() for waypoint navigation
 *             - loiter_to_alt.reached_destination_xy initially false
 *             - Transitions to Phase 2 when wp_nav->reached_wp_destination_NE()
 *          2. Vertical Phase: Loiter while changing altitude
 *             - Uses loiter_to_alt_run() for altitude change at position
 *             - Monitors altitude error convergence
 *             - Completes when altitude target reached
 * 
 *          Altitude Target Conversion:
 *          - Target location extracted from cmd.content.location
 *          - If lat/lon zero, uses current position
 *          - Altitude converted to ABOVE_HOME frame for comparison
 *          - Stored in loiter_to_alt.alt (centimeters)
 * 
 *          Altitude Completion Detection (loiter_to_alt_run):
 *          1. Calculate altitude error: current_loc.alt - target_alt
 *          2. Within 5cm tolerance: Mark reached
 *          3. Error sign change (crossing target): Mark reached
 *          4. Uses hysteresis to prevent oscillation
 * 
 *          State Variables (loiter_to_alt structure):
 *          - reached_destination_xy: Horizontal position reached flag
 *          - loiter_start_done: Position controller initialized flag
 *          - reached_alt: Altitude target achieved flag
 *          - alt: Target altitude in cm (ABOVE_HOME frame)
 *          - alt_error_cm: Previous altitude error for sign detection
 * 
 *          Vertical Speed Control:
 *          - Uses sqrt_controller for smooth altitude approach
 *          - Speed increases with distance from target (within limits)
 *          - Automatically slows as target approached
 *          - Respects pos_control max speeds (up and down)
 * 
 *          Position Controller Initialization:
 *          - First calls do_loiter_unlimited() for horizontal setup
 *          - Sets vertical speed/acceleration limits
 *            * Speed up: wp_nav->get_default_speed_up_cms()
 *            * Speed down: wp_nav->get_default_speed_down_cms()
 *            * Acceleration: wp_nav->get_accel_U_cmss()
 *          - Configures correction speeds for position hold
 *          - Sets submode to LOITER_TO_ALT
 * 
 *          Execution (loiter_to_alt_run):
 *          1. If horizontal position not reached: Execute wp_run()
 *          2. Once horizontal reached:
 *             - Initialize NE position controller (if needed)
 *             - Calculate vertical velocity demand using sqrt_controller
 *             - Apply avoidance adjustments
 *             - Update surface tracking (rangefinder offset)
 *             - Execute position hold with climb rate
 * 
 *          Altitude Error Tolerance:
 *          - 5cm threshold for completion (tight tolerance)
 *          - Also completes on error sign change
 *          - Prevents waiting for exact precision
 * 
 *          Location Handling:
 *          - Zero lat/lon: Loiter at current horizontal position
 *          - Non-zero lat/lon: Navigate to specified position first
 *          - Alt frame from command (ABOVE_HOME, ABOVE_TERRAIN, etc.)
 * 
 *          Altitude Frame Conversion:
 *          - Command may specify any altitude frame
 *          - Internally converted to ABOVE_HOME for tracking
 *          - get_alt_cm() performs frame conversion
 *          - Failure case: Returns error message, marks complete
 * 
 *          Error Handling:
 *          - If altitude frame conversion fails:
 *            * Sets reached_destination_xy = true
 *            * Sets reached_alt = true
 *            * GCS message: "bad do_loiter_to_alt"
 *            * Command immediately completes (failsafe)
 * 
 *          Sqrt Controller Benefits:
 *          - Smooth velocity profile (not bang-bang)
 *          - Faster when far from target
 *          - Gentle approach to target
 *          - Reduces overshoot and oscillation
 * 
 *          Yaw Control:
 *          - Yaw mode NOT set by this function (note in original comment)
 *          - Typically inherits from mission context
 *          - Usually holds heading during loiter
 * 
 *          Typical Use Cases:
 *          - Altitude change at observation point
 *          - Descend at specific location before landing
 *          - Climb at waypoint for obstacle clearance
 *          - Survey altitude changes at defined positions
 * 
 *          Terrain Following:
 *          - Supports ABOVE_TERRAIN altitude frame
 *          - Rangefinder surface tracking active if enabled
 *          - Surface offset continuously updated
 * 
 *          Completion Verification:
 *          - verify_loiter_to_alt() checks both flags:
 *            * loiter_to_alt.reached_destination_xy
 *            * loiter_to_alt.reached_alt
 *          - Returns true only when BOTH conditions met
 * 
 * @param[in] cmd Mission command with loiter position and target altitude
 *                - content.location.lat: Target latitude (0 = current)
 *                - content.location.lng: Target longitude (0 = current)
 *                - content.location.alt: Target altitude (any frame)
 *                - content.location.get_alt_frame(): Altitude reference frame
 * 
 * @return void (initializes loiter-to-altitude state)
 * 
 * @note Called by start_command() when NAV_LOITER_TO_ALT (31) encountered
 * @note Completes when both horizontal position AND altitude reached
 * @note loiter_to_alt_run() executes altitude change at position
 * @note verify_loiter_to_alt() checks completion (both xy and alt)
 * @note 5cm altitude tolerance for completion detection
 * @note Yaw mode not modified by this function (set by mission context)
 * 
 * @see do_loiter_unlimited() for horizontal loiter setup
 * @see loiter_to_alt_run() for altitude change execution
 * @see verify_loiter_to_alt() for completion detection
 */
void ModeAuto::do_loiter_to_alt(const AP_Mission::Mission_Command& cmd)
{
    // re-use loiter unlimited
    do_loiter_unlimited(cmd);

    // if we aren't navigating to a location then we have to adjust
    // altitude for current location
    Location target_loc(cmd.content.location);
    if (target_loc.lat == 0 && target_loc.lng == 0) {
        target_loc.lat = copter.current_loc.lat;
        target_loc.lng = copter.current_loc.lng;
    }

    if (!target_loc.get_alt_cm(Location::AltFrame::ABOVE_HOME, loiter_to_alt.alt)) {
        loiter_to_alt.reached_destination_xy = true;
        loiter_to_alt.reached_alt = true;
        gcs().send_text(MAV_SEVERITY_INFO, "bad do_loiter_to_alt");
        return;
    }
    loiter_to_alt.reached_destination_xy = false;
    loiter_to_alt.loiter_start_done = false;
    loiter_to_alt.reached_alt = false;
    loiter_to_alt.alt_error_cm = 0;

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_U_cm(wp_nav->get_default_speed_down_cms(), wp_nav->get_default_speed_up_cms(), wp_nav->get_accel_U_cmss());
    pos_control->set_correction_speed_accel_U_cmss(wp_nav->get_default_speed_down_cms(), wp_nav->get_default_speed_up_cms(), wp_nav->get_accel_U_cmss());

    // set submode
    set_submode(SubMode::LOITER_TO_ALT);
}

/**
 * @brief Process NAV_SPLINE_WAYPOINT mission command for smooth curved path following
 * 
 * @details Flies vehicle along smooth spline curve through waypoint instead of straight line.
 *          Creates fluid, cinematically smooth paths by computing Hermite spline curves
 *          between waypoints. Reduces mechanical stress and provides smoother video footage.
 * 
 *          Spline vs Straight Waypoint:
 *          - Regular NAV_WAYPOINT: Straight line, sharp corners at waypoints
 *          - NAV_SPLINE_WAYPOINT: Curved path, smooth transitions through waypoints
 *          - Spline maintains continuous velocity through waypoint (no stop)
 *          - Better for racing, cinematography, and mechanical efficiency
 * 
 *          Spline Curve Calculation:
 *          - Uses Hermite spline algorithm in AC_WPNav
 *          - Requires current waypoint, next waypoint, and waypoint after next
 *          - Three points define curve tangents for smooth transitions
 *          - Curve passes through waypoint position at specified altitude
 * 
 *          Three-Point Spline Construction:
 *          1. Current destination (dest_loc): This waypoint
 *          2. Next destination (next_dest_loc): Following waypoint
 *          3. Next-next destination: Waypoint after next (if available)
 *          - get_spline_from_cmd() extracts these from mission
 *          - If next or next-next unavailable, uses current as default
 * 
 *          Next Waypoint Detection:
 *          - get_spline_from_cmd() retrieves following waypoint
 *          - If cmd.p1 > 0 (delay at waypoint): No next waypoint needed
 *          - mission.get_next_nav_cmd() fetches subsequent command
 *          - Checks if next is also spline or regular waypoint
 *          - next_dest_loc_is_spline flag indicates continuity
 * 
 *          Spline Continuity:
 *          - If next waypoint also NAV_SPLINE_WAYPOINT: Continuous curve
 *          - If next is regular waypoint: Spline ends, transitions to straight
 *          - Maintains smooth velocity profile across spline segments
 * 
 *          Location Calculation:
 *          - Default location: Current position minus offsets
 *          - If transitioning from previous waypoint: Use that destination
 *          - get_spline_from_cmd() processes command into locations
 *          - loc_from_cmd() handles zero coordinate substitution
 * 
 *          Zero Coordinate Substitution:
 *          - lat/lon = 0: Uses current or previous waypoint position
 *          - alt = 0: Uses current altitude in command's altitude frame
 *          - Allows relative positioning in spline paths
 * 
 *          Spline Destination Setting:
 *          - wp_nav->set_spline_destination_loc() configures controller
 *          - Provides current dest, next dest, and spline continuity flag
 *          - AC_WPNav calculates Hermite spline control points
 *          - Generates smooth velocity profile along curve
 * 
 *          Next Waypoint Lookahead (set_next_wp):
 *          - If no delay (p1 = 0): Configure next destination
 *          - Allows velocity to smoothly continue through waypoint
 *          - set_next_wp() handles next spline or straight segment
 *          - Creates seamless transitions between mission legs
 * 
 *          Delay Parameter (cmd.p1):
 *          - If p1 > 0: Vehicle stops at this waypoint for p1 seconds
 *          - loiter_time_max set to delay duration
 *          - Breaks spline continuity (stop and wait)
 *          - Used when specific timing required at waypoint
 * 
 *          Yaw Control:
 *          - Default: Yaw follows flight path direction
 *          - Exception 1: ROI active (auto_yaw.mode() == ROI)
 *          - Exception 2: FIXED yaw with WP_YAW_BEHAVIOR_NONE
 *          - Otherwise: auto_yaw.set_mode_to_default(false)
 * 
 *          Spline Algorithm Details:
 *          - Hermite spline ensures C1 continuity (smooth velocity)
 *          - Position and velocity continuous at waypoint
 *          - Acceleration may have discontinuity
 *          - Curve guaranteed to pass through waypoint position
 * 
 *          Altitude Profile:
 *          - Altitude smoothly interpolated along spline
 *          - Altitude frame from command (ABOVE_HOME, ABOVE_TERRAIN, etc.)
 *          - Vertical velocity matched to horizontal progress
 * 
 *          Speed Control:
 *          - Maintains constant ground speed along spline (if possible)
 *          - Attitude controller limits constrain achievable curvature
 *          - Tighter curves may cause speed reduction
 *          - Speed parameters from wp_nav (WP_SPEED, WP_SPEED_UP, WP_SPEED_DN)
 * 
 *          Terrain Following:
 *          - Supports ABOVE_TERRAIN altitude frame
 *          - Spline altitude adjusted for terrain height
 *          - Terrain data required for entire spline segment
 *          - copter.failsafe_terrain_on_event() on missing data
 * 
 *          Typical Use Cases:
 *          - Aerial photography/videography (smooth camera motion)
 *          - Racing and time trials (minimize time with smooth corners)
 *          - Inspection paths (fluid motion along structures)
 *          - Reducing mechanical stress on vehicle
 * 
 *          Mission Constraints:
 *          - Best with 3+ consecutive spline waypoints
 *          - Single spline behaves similar to straight waypoint
 *          - Mixing spline and regular waypoints works but less smooth
 * 
 *          Execution:
 *          - Submode set to WP (same as regular waypoint)
 *          - wp_run() executes spline following at 100Hz
 *          - AC_WPNav handles spline trajectory generation
 *          - verify_spline_wp() checks completion
 * 
 * @param[in] cmd Mission command with spline waypoint parameters
 *                - content.location: Waypoint position (lat, lon, alt)
 *                - p1: Loiter time at waypoint in seconds (0 = no stop)
 *                - Zero coordinates use current or previous position
 * 
 * @return void (initializes spline navigation, execution via wp_run)
 * 
 * @note Called by start_command() when NAV_SPLINE_WAYPOINT (82) encountered
 * @note Requires next waypoint for proper spline calculation
 * @note get_spline_from_cmd() extracts 3-point spline configuration
 * @note set_next_wp() configures lookahead for smooth velocity
 * @note verify_spline_wp() checks completion (same as regular waypoint)
 * @warning Tight curves may exceed attitude limits, causing path deviation
 * @warning Requires terrain data if ABOVE_TERRAIN altitude frame used
 * 
 * @see get_spline_from_cmd() for spline point extraction
 * @see set_next_wp() for next destination configuration
 * @see verify_spline_wp() for completion detection
 * @see wp_run() for spline trajectory execution
 */
void ModeAuto::do_spline_wp(const AP_Mission::Mission_Command& cmd)
{
    // calculate default location used when lat, lon or alt is zero
    Location default_loc = copter.current_loc;

    // subtract position offsets
    subtract_pos_offsets(default_loc);

    if (wp_nav->is_active() && wp_nav->reached_wp_destination()) {
        if (!wp_nav->get_wp_destination_loc(default_loc)) {
            // this should never happen
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        }
    }

    // get spline's location and next location from command and send to wp_nav
    Location dest_loc, next_dest_loc;
    bool next_dest_loc_is_spline;
    get_spline_from_cmd(cmd, default_loc, dest_loc, next_dest_loc, next_dest_loc_is_spline);
    if (!wp_nav->set_spline_destination_loc(dest_loc, next_dest_loc, next_dest_loc_is_spline)) {
        // failure to set destination can only be because of missing terrain data
        copter.failsafe_terrain_on_event();
        return;
    }

    // this will be used to remember the time in millis after we reach or pass the WP.
    loiter_time = 0;
    // this is the delay, stored in seconds
    loiter_time_max = cmd.p1;

    // set next destination if necessary
    if (!set_next_wp(cmd, dest_loc)) {
        // failure to set next destination can only be because of missing terrain data
        copter.failsafe_terrain_on_event();
        return;
    }

    // initialise yaw
    // To-Do: reset the yaw only when the previous navigation command is not a WP.  this would allow removing the special check for ROI
    if (auto_yaw.mode() != AutoYaw::Mode::ROI && !(auto_yaw.mode() == AutoYaw::Mode::FIXED && copter.g.wp_yaw_behavior == WP_YAW_BEHAVIOR_NONE)) {
        auto_yaw.set_mode_to_default(false);
    }

    // set submode
    set_submode(SubMode::WP);
}

/**
 * @brief Extract spline waypoint data including current and lookahead destinations
 * 
 * @details Parses spline waypoint mission command to extract destinations required for smooth
 *          curve generation. Spline navigation requires current destination and next destination
 *          to calculate Bezier curve tangent vectors. Also determines if next segment is also
 *          spline (for continuous curves) or straight line (for curve termination). Enables
 *          smooth, flowing flight paths through multiple waypoints with continuous curvature.
 * 
 *          What is Spline Navigation:
 *          - Smooth curved path through waypoints
 *          - Uses Bezier curves or similar splines
 *          - Continuous path (no corners)
 *          - Tangent continuity between segments
 *          - More natural flight path than straight lines
 * 
 *          Why Splines Need Lookahead:
 *          - Spline curve requires entry and exit tangents
 *          - Exit tangent points toward next destination
 *          - Cannot generate curve without knowing next point
 *          - Must look ahead in mission to get next destination
 *          - Similar to how corner cutting needs next waypoint
 * 
 *          Bezier Curve Basics:
 *          - Defined by control points
 *          - Curve doesn't necessarily pass through control points
 *          - Tangent at start point toward first control point
 *          - Tangent at end point from last control point
 *          - Smooth, continuous curvature
 * 
 *          Spline Waypoint Implementation:
 *          - Waypoint positions are on the curve
 *          - Control points calculated from previous/next waypoints
 *          - Creates smooth path through all waypoints
 *          - No sharp corners
 *          - Better for photography, inspection
 * 
 *          Output Parameters:
 *          - dest_loc: Current spline waypoint destination
 *          - next_dest_loc: Next navigation destination (for tangent)
 *          - next_dest_loc_is_spline: Is next segment also spline?
 * 
 *          Current Destination Extraction:
 *          - dest_loc = loc_from_cmd(cmd, default_loc)
 *          - Standard location extraction
 *          - Handles zero lat/lon/alt using default_loc
 *          - This is the waypoint vehicle flies through
 * 
 *          Delay Check (cmd.p1):
 *          - if cmd.p1 == 0: No delay at waypoint
 *          - if cmd.p1 > 0: Vehicle stops at waypoint
 *          - Spline continues only if no delay
 *          - Delay breaks spline continuity
 * 
 *          Why Delay Breaks Spline:
 *          - Vehicle must stop at waypoint
 *          - Cannot maintain continuous curve
 *          - Stopping requires deceleration to zero
 *          - Then wait, then accelerate
 *          - Incompatible with smooth spline
 * 
 *          Next Navigation Command Retrieval:
 *          - mission.get_next_nav_cmd(cmd.index+1, temp_cmd)
 *          - Get next navigation command in mission
 *          - Skips DO and CONDITION commands
 *          - Only NAV commands matter for trajectory
 * 
 *          Next Destination Extraction:
 *          - next_dest_loc = loc_from_cmd(temp_cmd, dest_loc)
 *          - Extract location from next command
 *          - Use current dest_loc as default
 *          - Next waypoint relative to current if needed
 * 
 *          Next Segment Type Check:
 *          - next_dest_loc_is_spline = (temp_cmd.id == MAV_CMD_NAV_SPLINE_WAYPOINT)
 *          - Is next segment also spline?
 *          - If yes: Maintain curvature continuity
 *          - If no: Terminate spline at current waypoint
 * 
 *          Continuous Spline Path:
 *          - Multiple consecutive spline waypoints
 *          - Each segment knows next is also spline
 *          - Creates one continuous curve through all
 *          - Tangent continuity maintained
 *          - Very smooth flight
 * 
 *          Spline to Straight Transition:
 *          - Current: MAV_CMD_NAV_SPLINE_WAYPOINT
 *          - Next: MAV_CMD_NAV_WAYPOINT (straight)
 *          - next_dest_loc_is_spline = false
 *          - Spline terminates at current waypoint
 *          - Vehicle transitions to straight line
 * 
 *          No Next Command Case:
 *          - Last waypoint in mission
 *          - OR next command has delay (breaks continuity)
 *          - next_dest_loc = dest_loc (same as current)
 *          - next_dest_loc_is_spline = false
 *          - Spline terminates at current waypoint
 * 
 *          Why Set next_dest_loc = dest_loc:
 *          - AC_WPNav still needs a "next" destination
 *          - Setting to same point means "stop here"
 *          - Exit tangent points at waypoint (zero tangent)
 *          - Vehicle decelerates to stop at waypoint
 *          - Safe default behavior
 * 
 *          Use Case - 3 Spline Waypoints:
 *          - WP1: SPLINE (100, 0, 50m)
 *          - WP2: SPLINE (200, 0, 50m)  
 *          - WP3: SPLINE (300, 0, 50m)
 *          - At WP1: next_dest = WP2, is_spline = true
 *          - Curve from WP1 → WP2 with continuous tangent
 *          - At WP2: next_dest = WP3, is_spline = true  
 *          - One smooth curve through all three points
 * 
 *          Use Case - Spline to Straight:
 *          - WP1: SPLINE (100, 0, 50m)
 *          - WP2: SPLINE (200, 0, 50m)
 *          - WP3: WAYPOINT (300, 0, 50m)
 *          - At WP2: next_dest = WP3, is_spline = false
 *          - Spline terminates at WP2
 *          - Straight line from WP2 → WP3
 * 
 *          Use Case - Spline with Delay:
 *          - WP1: SPLINE (100, 0, 50m), p1=0
 *          - WP2: SPLINE (200, 0, 50m), p1=5 (5 second delay)
 *          - WP3: SPLINE (300, 0, 50m), p1=0
 *          - At WP1: next_dest = WP2, is_spline = true, smooth curve
 *          - At WP2: next_dest = WP2 (same), is_spline = false
 *          - Vehicle stops at WP2, waits 5 seconds
 *          - Spline broken at WP2
 * 
 *          Relationship to set_next_wp():
 *          - set_next_wp() also looks ahead for corner cutting
 *          - get_spline_from_cmd() specifically for spline curves
 *          - Both optimize trajectory planning
 *          - Spline requires more information (is next also spline?)
 * 
 *          Called From:
 *          - do_spline_wp(): Initial spline setup
 *          - set_next_wp(): When next command is spline
 * 
 *          AC_WPNav Spline Controller:
 *          - Receives dest_loc, next_dest_loc, next_dest_loc_is_spline
 *          - Calculates Bezier control points
 *          - Generates smooth curve
 *          - Maintains speed through curves
 *          - Adjusts for altitude changes
 * 
 *          Altitude in Splines:
 *          - Spline path includes altitude curve
 *          - Smooth altitude transitions
 *          - Not just horizontal spline
 *          - 3D curved path
 * 
 * @param[in]  cmd Mission command (MAV_CMD_NAV_SPLINE_WAYPOINT)
 * @param[in]  default_loc Default location for zero fields in cmd
 * @param[out] dest_loc Current spline waypoint destination (corrected for zeros)
 * @param[out] next_dest_loc Next navigation destination (for tangent calculation)
 * @param[out] next_dest_loc_is_spline True if next segment also spline, false otherwise
 * 
 * @note If cmd has delay (p1 > 0), next_dest_loc set to dest_loc and next_dest_loc_is_spline = false
 * @note If no next nav command, next_dest_loc set to dest_loc and next_dest_loc_is_spline = false
 * @note Both cases terminate spline at current waypoint
 * @note Spline continuity maintained only if next command is also spline with no delay
 * 
 * @see do_spline_wp() - Calls this to set up spline waypoint
 * @see set_next_wp() - Similar lookahead for straight waypoints
 * @see loc_from_cmd() - Extracts location from command
 */
// calculate locations required to build a spline curve from a mission command
// dest_loc is populated from cmd's location using default_loc in cases where the lat and lon or altitude is zero
// next_dest_loc and nest_dest_loc_is_spline is filled in with the following navigation command's location if it exists.  If it does not exist it is set to the dest_loc and false
void ModeAuto::get_spline_from_cmd(const AP_Mission::Mission_Command& cmd, const Location& default_loc, Location& dest_loc, Location& next_dest_loc, bool& next_dest_loc_is_spline)
{
    dest_loc = loc_from_cmd(cmd, default_loc);

    // if there is no delay at the end of this segment get next nav command
    AP_Mission::Mission_Command temp_cmd;
    if (cmd.p1 == 0 && mission.get_next_nav_cmd(cmd.index+1, temp_cmd)) {
        next_dest_loc = loc_from_cmd(temp_cmd, dest_loc);
        next_dest_loc_is_spline = temp_cmd.id == MAV_CMD_NAV_SPLINE_WAYPOINT;
    } else {
        next_dest_loc = dest_loc;
        next_dest_loc_is_spline = false;
    }
}

#if AC_NAV_GUIDED
// do_nav_guided_enable - initiate accepting commands from external nav computer
void ModeAuto::do_nav_guided_enable(const AP_Mission::Mission_Command& cmd)
{
    if (cmd.p1 > 0) {
        // start guided within auto
        nav_guided_start();
    }
}

// do_guided_limits - pass guided limits to guided controller
void ModeAuto::do_guided_limits(const AP_Mission::Mission_Command& cmd)
{
    copter.mode_guided.limit_set(
        cmd.p1 * 1000, // convert seconds to ms
        cmd.content.guided_limits.alt_min * 100.0f,    // convert meters to cm
        cmd.content.guided_limits.alt_max * 100.0f,    // convert meters to cm
        cmd.content.guided_limits.horiz_max * 100.0f); // convert meters to cm
}
#endif  // AC_NAV_GUIDED

// do_nav_delay - Delay the next navigation command
void ModeAuto::do_nav_delay(const AP_Mission::Mission_Command& cmd)
{
    nav_delay_time_start_ms = millis();

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

#if AP_SCRIPTING_ENABLED
// start accepting position, velocity and acceleration targets from lua scripts
void ModeAuto::do_nav_script_time(const AP_Mission::Mission_Command& cmd)
{
    // call regular guided flight mode initialisation
    if (copter.mode_guided.init(true)) {
        nav_scripting.done = false;
        nav_scripting.id++;
        nav_scripting.start_ms = millis();
        nav_scripting.command = cmd.content.nav_script_time.command;
        nav_scripting.timeout_s = cmd.content.nav_script_time.timeout_s;
        nav_scripting.arg1 = cmd.content.nav_script_time.arg1.get();
        nav_scripting.arg2 = cmd.content.nav_script_time.arg2.get();
        nav_scripting.arg3 = cmd.content.nav_script_time.arg3;
        nav_scripting.arg4 = cmd.content.nav_script_time.arg4;
        set_submode(SubMode::NAV_SCRIPT_TIME);
    } else {
        // for safety we set nav_scripting to done to protect against the mission getting stuck
        nav_scripting.done = true;
    }
}
#endif

// start maintaining an attitude for a specified time
void ModeAuto::do_nav_attitude_time(const AP_Mission::Mission_Command& cmd)
{
    // copy command arguments into local structure
    nav_attitude_time.roll_deg = cmd.content.nav_attitude_time.roll_deg;
    nav_attitude_time.pitch_deg = cmd.content.nav_attitude_time.pitch_deg;
    nav_attitude_time.yaw_deg = cmd.content.nav_attitude_time.yaw_deg;
    nav_attitude_time.climb_rate = cmd.content.nav_attitude_time.climb_rate;
    nav_attitude_time.start_ms = AP_HAL::millis();
    set_submode(SubMode::NAV_ATTITUDE_TIME);
}

/********************************************************************************/
// Condition (May) commands
/********************************************************************************/

/**
 * @brief Process CONDITION_DELAY to pause mission progression for specified time
 * 
 * @details Implements timed delay condition - mission waits for specified duration
 *          before advancing to next command. Used to synchronize mission timing,
 *          allow sensor stabilization, or coordinate with external events.
 * 
 *          Condition Command Behavior:
 *          - Executes immediately when encountered in mission
 *          - Blocks advancement to next NAV command
 *          - Does NOT affect current navigation (vehicle continues current action)
 *          - verify_wait_delay() called repeatedly to check completion
 * 
 *          Timing Mechanism:
 *          - condition_start: Stores millis() when delay begins
 *          - condition_value: Delay duration in milliseconds
 *          - cmd.content.delay.seconds: Command parameter in seconds
 *          - Converted to ms by multiplying by 1000
 * 
 *          Delay Duration:
 *          - cmd.content.delay.seconds: Integer seconds to wait
 *          - Minimum 0 seconds (immediate completion)
 *          - Maximum 2^31 ms (~24 days practical limit)
 *          - Actual precision: ~10ms (mission update rate)
 * 
 *          Execution Flow:
 *          1. start_command() calls do_wait_delay()
 *          2. condition_start = millis() (current time)
 *          3. condition_value = seconds * 1000 (convert to ms)
 *          4. verify_wait_delay() called at 10Hz by mission library
 *          5. When (millis() - condition_start) > condition_value: Complete
 * 
 *          Verification (verify_wait_delay):
 *          - Called repeatedly by mission.update()
 *          - Checks: millis() - condition_start > MAX(condition_value, 0)
 *          - MAX ensures negative values treated as 0
 *          - Returns true when delay elapsed
 *          - Sets condition_value = 0 on completion
 * 
 *          Impact on Navigation:
 *          - Current NAV command continues executing
 *          - Vehicle maintains position/track during delay
 *          - If at waypoint: Loiters during delay
 *          - If in transit: Continues to destination, then delays
 * 
 *          Difference from Waypoint Loiter Time:
 *          - NAV_WAYPOINT p1: Delays at specific waypoint location
 *          - CONDITION_DELAY: Delays wherever vehicle currently is
 *          - Condition more flexible for non-waypoint delays
 * 
 *          Mission Structure Example:
 *          ```
 *          CMD 1: NAV_WAYPOINT (lat, lon, alt) p1=0  // Fly to waypoint
 *          CMD 2: CONDITION_DELAY 10s                // Wait 10 seconds
 *          CMD 3: DO_SET_SERVO                       // Then activate servo
 *          CMD 4: NAV_WAYPOINT (next position)       // Continue mission
 *          ```
 * 
 *          Timing Accuracy:
 *          - Mission update rate: 10Hz (every 100ms)
 *          - Delay checked every 100ms
 *          - Actual delay: Requested + 0-100ms
 *          - For precise timing, account for 100ms quantization
 * 
 *          Zero Delay:
 *          - delay.seconds = 0: Completes immediately
 *          - Still processes through condition mechanism
 *          - Useful for conditional logic placeholders
 * 
 *          Condition vs Do Commands:
 *          - CONDITION commands: Block mission progression (Must complete)
 *          - DO commands: Execute immediately and continue (May execute)
 *          - CONDITION_DELAY is a blocking delay
 * 
 *          Multiple Conditions:
 *          - Only one condition active at a time
 *          - condition_start/condition_value shared across condition types
 *          - Subsequent condition overwrites previous
 * 
 *          Use Cases:
 *          - Sensor stabilization (wait after rapid maneuver)
 *          - Coordinated timing (surveying, data collection)
 *          - Waiting for external event (manual intervention window)
 *          - Synchronized multi-vehicle operations
 *          - Gimbal stabilization after movement
 * 
 *          Timeout Considerations:
 *          - Uses uint32_t millis() (wraps every 49.7 days)
 *          - Delay calculation handles millis() rollover correctly
 *          - Subtraction arithmetic naturally handles wraparound
 * 
 *          Interaction with DO Commands:
 *          - DO commands between NAV commands execute during delay
 *          - Example: NAV_WP, CONDITION_DELAY, DO_SET_SERVO all execute
 *          - Servo activates after delay completes
 * 
 *          Pilot Override:
 *          - Pilot can switch flight mode during delay (aborts mission)
 *          - Cannot skip delay while remaining in Auto
 *          - Delay enforced for mission integrity
 * 
 * @param[in] cmd Mission command with delay parameters
 *                - content.delay.seconds: Duration to wait (0-2147483)
 * 
 * @return void (sets condition_start and condition_value)
 * 
 * @note Called by start_command() when CONDITION_DELAY (112) encountered
 * @note verify_wait_delay() called repeatedly to check completion
 * @note Uses shared condition_start and condition_value variables
 * @note Delay does not affect current navigation behavior
 * @note Mission update rate limits timing precision to ~100ms
 * 
 * @see verify_wait_delay() for completion detection
 * @see do_within_distance() for distance-based conditions
 * @see do_yaw() for yaw-based conditions
 */
void ModeAuto::do_wait_delay(const AP_Mission::Mission_Command& cmd)
{
    condition_start = millis();
    condition_value = cmd.content.delay.seconds * 1000;     // convert seconds to milliseconds
}

/**
 * @brief Process CONDITION_DISTANCE to wait until within specified distance of target
 * 
 * @details Implements distance-based condition - mission waits until vehicle is within
 *          specified distance of current navigation target before advancing. Used to
 *          trigger actions at specific distances from waypoints or targets.
 * 
 *          Condition Purpose:
 *          - Triggers when distance to target <= threshold
 *          - More precise than waypoint arrival detection
 *          - Allows actions before reaching waypoint center
 *          - Useful for camera triggers, payload drops, approach notifications
 * 
 *          Distance Measurement:
 *          - condition_value: Distance threshold in meters
 *          - cmd.content.distance.meters: Commanded threshold
 *          - wp_distance_m(): Current distance to target
 *          - Measured as 2D horizontal distance (ignores altitude)
 * 
 *          Verification (verify_within_distance):
 *          - Called repeatedly by mission.update() at 10Hz
 *          - Checks: wp_distance_m() < MAX(condition_value, 0)
 *          - MAX ensures negative thresholds treated as 0
 *          - Returns true when within threshold
 *          - Sets condition_value = 0 on completion
 * 
 *          Distance Calculation (wp_distance_m):
 *          - SubMode::WP or CIRCLE_MOVE_TO_EDGE: wp_nav->get_wp_distance_to_destination_cm()
 *          - SubMode::CIRCLE: circle_nav->get_distance_to_target_cm()
 *          - Converted from cm to m (divide by 100)
 *          - Horizontal distance only (NE plane)
 * 
 *          Execution Flow:
 *          1. start_command() calls do_within_distance()
 *          2. condition_value = meters (threshold)
 *          3. verify_within_distance() checks distance repeatedly
 *          4. When wp_distance_m() < condition_value: Completes
 *          5. Mission advances to next command
 * 
 *          Typical Mission Structure:
 *          ```
 *          CMD 1: NAV_WAYPOINT (lat, lon, alt)       // Navigate to waypoint
 *          CMD 2: CONDITION_DISTANCE 50m              // When within 50m...
 *          CMD 3: DO_DIGICAM_CONTROL                  // Trigger camera
 *          CMD 4: NAV_WAYPOINT (next position)        // Continue
 *          ```
 * 
 *          Distance Threshold Selection:
 *          - Smaller values: More precise trigger, risk of overshoot
 *          - Larger values: Earlier trigger, more margin
 *          - Typical: 20-100m depending on speed and application
 *          - Consider vehicle speed and stopping distance
 * 
 *          Minimum Distance:
 *          - 0 meters: Triggers only when exactly at target (rare)
 *          - Negative values: Treated as 0 (MAX function)
 *          - Practical minimum: 1-5m (position accuracy limits)
 * 
 *          Position Accuracy:
 *          - GPS accuracy typically 1-5m
 *          - DGPS/RTK: <1m accuracy
 *          - Distance threshold should exceed position uncertainty
 *          - Indoor/optical flow: Variable accuracy
 * 
 *          Waypoint Radius vs Distance Condition:
 *          - WPNAV_RADIUS: Default waypoint completion radius
 *          - CONDITION_DISTANCE: Explicit action trigger distance
 *          - Can be larger or smaller than WPNAV_RADIUS
 *          - Provides finer control than waypoint completion
 * 
 *          Circle Mode Behavior:
 *          - Distance to circle center (not current position on circle)
 *          - Useful for triggering when approaching circle start
 *          - circle_nav->get_distance_to_target_cm() measures to center
 * 
 *          Use Cases:
 *          - Camera triggers at specific distances
 *          - Payload release approach warnings
 *          - Precision approach notifications
 *          - Early action triggers before waypoint arrival
 *          - Graduated actions at multiple distances
 * 
 *          Multiple Distance Conditions:
 *          - Can stack multiple CONDITION_DISTANCE commands
 *          - Each with different threshold and associated DO command
 *          - Example: 100m warning, 50m prepare, 10m trigger
 * 
 *          Speed Considerations:
 *          - High speed may overshoot small thresholds
 *          - Mission updates at 10Hz (100ms intervals)
 *          - At 20m/s, vehicle travels 2m between checks
 *          - Use threshold > speed * 0.1s for reliability
 * 
 *          Altitude Independence:
 *          - Distance measured horizontally only (NE plane)
 *          - Altitude not considered in threshold
 *          - Useful for maintaining altitude separation
 * 
 *          Interaction with Navigation:
 *          - Does not affect waypoint navigation
 *          - Vehicle continues normal flight
 *          - Simply triggers when threshold crossed
 * 
 *          Edge Cases:
 *          - Starting within threshold: Completes immediately
 *          - Moving away from target: Never completes
 *          - Ensure mission logic approaches target
 * 
 * @param[in] cmd Mission command with distance threshold
 *                - content.distance.meters: Distance threshold in meters (0-65535)
 * 
 * @return void (sets condition_value to distance threshold)
 * 
 * @note Called by start_command() when CONDITION_DISTANCE (114) encountered
 * @note verify_within_distance() checks completion repeatedly
 * @note Uses wp_distance_m() for distance calculation
 * @note Measures horizontal distance only (altitude independent)
 * @note Consider vehicle speed when setting threshold
 * @warning Small thresholds may be overshot at high speeds
 * 
 * @see verify_within_distance() for completion detection
 * @see wp_distance_m() for distance measurement
 * @see do_wait_delay() for time-based conditions
 */
void ModeAuto::do_within_distance(const AP_Mission::Mission_Command& cmd)
{
    condition_value  = cmd.content.distance.meters;
}

/**
 * @brief Process CONDITION_YAW command to set vehicle heading
 * 
 * @details Implements yaw angle condition that rotates vehicle to specified heading.
 *          Can specify absolute compass heading or relative angle from current heading.
 *          Controls turn direction and rate. Mission continues when target heading reached.
 * 
 *          Yaw Command Types:
 *          - Absolute: Specific compass heading (0-360°)
 *          - Relative: Angle from current heading (+/- degrees)
 *          - Configurable turn direction (shortest, CW, CCW)
 *          - Configurable turn rate (deg/s)
 * 
 *          Command Parameters:
 *          - angle_deg: Target angle in degrees
 *          - turn_rate_dps: Turn rate in degrees per second (0 = use default)
 *          - direction: -1=CCW, 0=shortest, 1=CW
 *          - relative_angle: 0=absolute compass heading, 1=relative to current
 * 
 *          Absolute vs Relative:
 *          - Absolute (relative_angle = 0): North=0°, East=90°, South=180°, West=270°
 *          - Relative (relative_angle > 0): +90° = turn right, -90° = turn left
 *          - Relative useful for vehicle-centric commands
 *          - Absolute useful for fixed compass directions
 * 
 *          Turn Direction Options:
 *          - direction = 0 (shortest): Chooses shorter rotation path
 *          - direction = 1 (clockwise): Always turns right/clockwise
 *          - direction = -1 (counter-clockwise): Always turns left/CCW
 *          - Shortest typically most efficient
 * 
 *          Turn Rate Control:
 *          - turn_rate_dps: Commanded turn rate (deg/s)
 *          - 0 = use default yaw rate from parameters
 *          - Typical values: 10-90 deg/s
 *          - Slower rates: Smoother, better for cameras
 *          - Faster rates: Quicker response, more aggressive
 * 
 *          AutoYaw Controller:
 *          - auto_yaw.set_fixed_yaw_rad() configures target
 *          - Converts degrees to radians for internal use
 *          - Sets AutoYaw::Mode::FIXED
 *          - Attitude controller executes yaw change
 * 
 *          Execution Process:
 *          1. do_yaw() called by start_command()
 *          2. auto_yaw.set_fixed_yaw_rad() sets target and parameters
 *          3. AutoYaw mode set to FIXED
 *          4. Attitude controller rotates vehicle at specified rate
 *          5. verify_yaw() checks completion repeatedly
 *          6. Mission advances when target reached
 * 
 *          Verification (verify_yaw):
 *          - Resets auto_yaw.set_mode(FIXED) (prevent override)
 *          - Calls auto_yaw.reached_fixed_yaw_target()
 *          - Returns true when within tolerance
 *          - Yaw tolerance typically 2-5 degrees
 * 
 *          Yaw During Navigation:
 *          - Can execute during waypoint navigation
 *          - Vehicle continues moving while rotating
 *          - Rotation independent of translation
 *          - Useful for camera pointing while flying
 * 
 *          Typical Mission Sequence:
 *          ```
 *          CMD 1: NAV_WAYPOINT (position)        // Fly to location
 *          CMD 2: CONDITION_YAW 45° CW           // Rotate to 45° heading
 *          CMD 3: DO_DIGICAM_CONTROL             // Take photo facing 45°
 *          CMD 4: NAV_WAYPOINT (next position)   // Continue mission
 *          ```
 * 
 *          ROI Interaction:
 *          - CONDITION_YAW overrides ROI (Region of Interest)
 *          - If ROI active, yaw command takes precedence
 *          - ROI must be re-established after yaw command
 *          - verify_yaw() resets to FIXED to maintain control
 * 
 *          Camera Applications:
 *          - Point camera at specific compass direction
 *          - Panorama capture (multiple yaw commands)
 *          - Building inspection (face perpendicular walls)
 *          - Oblique photography (controlled angles)
 * 
 *          Relative Yaw Examples:
 *          - +90°: Turn right 90 degrees from current heading
 *          - -180°: Turn around (reverse direction)
 *          - +45°: Slight right turn
 *          - Multiple relative commands cumulative
 * 
 *          Absolute Yaw Examples:
 *          - 0°: Face North
 *          - 90°: Face East
 *          - 180°: Face South
 *          - 270°: Face West
 * 
 *          Yaw Rate Considerations:
 *          - Too fast: Oscillation, instability
 *          - Too slow: Delayed mission, battery consumption
 *          - Camera gimbal: Use slower rates (10-30 deg/s)
 *          - Racing: Use faster rates (60-90 deg/s)
 * 
 *          Turn Direction Use Cases:
 *          - Shortest: Most efficient, unpredictable direction
 *          - Clockwise: Guaranteed right turn for cameras
 *          - Counter-clockwise: Guaranteed left turn
 *          - Specific direction for repeatable behaviors
 * 
 *          Coordinate with DO Commands:
 *          - CONDITION_YAW blocks until complete
 *          - DO commands after yaw execute when reached
 *          - Common: Yaw then trigger camera/servo
 * 
 *          Magnetic Declination:
 *          - Absolute angles relative to magnetic north
 *          - System automatically applies declination
 *          - Result points to true north when angle=0
 * 
 *          Multiple Yaw Commands:
 *          - Can chain multiple yaw commands
 *          - Each must complete before next starts
 *          - Useful for panoramas or inspection patterns
 * 
 *          Guided Mode Compatibility:
 *          - do_guided() also processes CONDITION_YAW
 *          - Same behavior in Auto and Guided modes
 *          - Allows external yaw control
 * 
 * @param[in] cmd Mission command with yaw parameters
 *                - content.yaw.angle_deg: Target angle (0-360 or relative)
 *                - content.yaw.turn_rate_dps: Turn rate deg/s (0=default)
 *                - content.yaw.direction: -1=CCW, 0=shortest, 1=CW
 *                - content.yaw.relative_angle: 0=absolute, 1=relative
 * 
 * @return void (configures AutoYaw controller)
 * 
 * @note Called by start_command() when CONDITION_YAW (115) encountered
 * @note Also called by do_guided() for external yaw commands
 * @note verify_yaw() checks completion and prevents mode override
 * @note Overrides ROI and other yaw control modes
 * @note Vehicle continues navigation during yaw rotation
 * 
 * @see verify_yaw() for completion detection
 * @see AutoYaw::set_fixed_yaw_rad() for yaw controller
 * @see do_roi() for alternative pointing control
 */
void ModeAuto::do_yaw(const AP_Mission::Mission_Command& cmd)
{
    auto_yaw.set_fixed_yaw_rad(
        radians(cmd.content.yaw.angle_deg),
        radians(cmd.content.yaw.turn_rate_dps),
        cmd.content.yaw.direction,
        cmd.content.yaw.relative_angle > 0);
}

/********************************************************************************/
// Do (Now) commands
/********************************************************************************/

/**
 * @brief Process DO_CHANGE_SPEED command to modify navigation speeds mid-mission
 * 
 * @details Changes vehicle speed parameters during mission execution. Modifies horizontal,
 *          climb, or descent speeds immediately. Useful for adjusting speeds based on
 *          mission phase, terrain, or operational requirements. Changes persist until
 *          another speed command or mission completion.
 * 
 *          Speed Types (cmd.content.speed.speed_type):
 *          - SPEED_TYPE_AIRSPEED: Horizontal speed (multirotors treat as groundspeed)
 *          - SPEED_TYPE_GROUNDSPEED: Horizontal ground speed
 *          - SPEED_TYPE_CLIMB_SPEED: Vertical ascent speed
 *          - SPEED_TYPE_DESCENT_SPEED: Vertical descent speed
 * 
 *          Horizontal Speed (AIRSPEED/GROUNDSPEED):
 *          - Sets wp_nav->set_speed_NE_cms() (North-East speed)
 *          - Applies to waypoint navigation, loiter positioning
 *          - Affects auto, guided, and RTL submodes
 *          - Stored in desired_speed_override.xy (m/s)
 * 
 *          Climb Speed (SPEED_TYPE_CLIMB_SPEED):
 *          - Sets wp_nav->set_speed_up_cms() (upward speed)
 *          - Controls climb rate during waypoint ascents
 *          - Affects takeoff, climb phases
 *          - Stored in desired_speed_override.up (m/s)
 * 
 *          Descent Speed (SPEED_TYPE_DESCENT_SPEED):
 *          - Sets wp_nav->set_speed_down_cms() (downward speed)
 *          - Controls descent rate during waypoint descents
 *          - Affects landing approaches (not final landing)
 *          - Stored in desired_speed_override.down (m/s)
 * 
 *          Speed Parameter:
 *          - cmd.content.speed.target_ms: Target speed in meters/second
 *          - Must be > 0 to take effect (0 or negative ignored)
 *          - Converted to cm/s for internal use (* 100.0f)
 *          - Practical range: 0.5-25 m/s depending on vehicle
 * 
 *          Speed Override Tracking:
 *          - desired_speed_override structure stores current overrides
 *          - .xy: Horizontal speed (m/s)
 *          - .up: Climb speed (m/s)
 *          - .down: Descent speed (m/s)
 *          - Used when initializing waypoint navigation
 * 
 *          Override Persistence:
 *          - Speed change persists for remainder of mission
 *          - Survives submode transitions (WP→LOITER→WP)
 *          - Reset only by: Another DO_CHANGE_SPEED, mode exit, reboot
 *          - wp_start() reapplies overrides when starting new waypoint
 * 
 *          Speed Application in wp_start():
 *          - Checks is_positive(desired_speed_override.xy/up/down)
 *          - If override set, applies before starting waypoint
 *          - Ensures speed persists across waypoint transitions
 *          - Allows mission-wide speed changes
 * 
 *          Unit Conversion:
 *          - Command parameter: m/s (meters per second)
 *          - Internal wp_nav: cm/s (centimeters per second)
 *          - Conversion: target_ms * 100.0f = cm/s
 *          - Override storage: m/s for external interface
 * 
 *          Speed Limits:
 *          - Cannot exceed vehicle's physical capabilities
 *          - Attitude control limits constrain achievable speeds
 *          - High speeds may degrade position accuracy
 *          - Battery efficiency varies with speed
 * 
 *          Typical Mission Usage:
 *          ```
 *          CMD 1: DO_CHANGE_SPEED 15 m/s horizontal  // Fast transit
 *          CMD 2: NAV_WAYPOINT (distant location)
 *          CMD 3: DO_CHANGE_SPEED 3 m/s horizontal   // Slow for precision
 *          CMD 4: NAV_WAYPOINT (survey area)
 *          CMD 5: DO_CHANGE_SPEED 8 m/s horizontal   // Medium speed return
 *          ```
 * 
 *          Climb Speed Applications:
 *          - Slower climbs: Better altitude control, lower noise
 *          - Faster climbs: Reach altitude quickly, battery efficient
 *          - Terrain following: Match terrain ascent rates
 *          - Takeoff: May want slower for safety
 * 
 *          Descent Speed Applications:
 *          - Slower descents: Precise altitude, better for landing approaches
 *          - Faster descents: Efficient return to lower altitudes
 *          - Does NOT affect final landing rate (separate parameter)
 * 
 *          Airspeed vs Groundspeed:
 *          - Fixed-wing: AIRSPEED controls indicated airspeed
 *          - Multirotor: Both treated as groundspeed (no airflow sensor)
 *          - Wind affects groundspeed achieved with airspeed commands
 * 
 *          Speed Command Placement:
 *          - DO commands execute immediately when encountered
 *          - Place BEFORE waypoint to affect that waypoint
 *          - Can place between waypoints to change during transit
 *          - Multiple speed commands allowed in mission
 * 
 *          Parameter Interaction:
 *          - Overrides default WPNAV_SPEED parameter
 *          - Overrides WPNAV_SPEED_UP parameter
 *          - Overrides WPNAV_SPEED_DN parameter
 *          - Parameters remain as fallback defaults
 * 
 *          Performance Considerations:
 *          - Higher speeds: Faster mission, less precision, more battery
 *          - Lower speeds: Better precision, more time, camera stability
 *          - Match speed to mission phase requirements
 * 
 *          Safety Considerations:
 *          - Very high speeds may exceed control authority
 *          - Consider obstacle avoidance reaction time
 *          - High descent speeds near ground dangerous
 *          - Test speed changes in simulation first
 * 
 *          Zero/Negative Speed:
 *          - target_ms <= 0: Command ignored
 *          - No change to current speed settings
 *          - Prevents accidental stops
 * 
 * @param[in] cmd Mission command with speed change parameters
 *                - content.speed.speed_type: Which speed to change
 *                - content.speed.target_ms: Target speed in m/s (must be > 0)
 * 
 * @return void (modifies wp_nav speeds and desired_speed_override)
 * 
 * @note Called by start_command() when DO_CHANGE_SPEED (178) encountered
 * @note Changes persist until another speed change or mission end
 * @note Zero or negative speeds ignored (no effect)
 * @note Speed overrides reapplied in wp_start() for each waypoint
 * @note Does not affect final landing descent rate
 * 
 * @see wp_start() for override application
 * @see set_speed_xy_cms(), set_speed_up_cms(), set_speed_down_cms()
 */
void ModeAuto::do_change_speed(const AP_Mission::Mission_Command& cmd)
{
    if (cmd.content.speed.target_ms > 0) {
        switch (cmd.content.speed.speed_type) {
        case SPEED_TYPE_CLIMB_SPEED:
            copter.wp_nav->set_speed_up_cms(cmd.content.speed.target_ms * 100.0f);
            desired_speed_override.up = cmd.content.speed.target_ms;
            break;
        case SPEED_TYPE_DESCENT_SPEED:
            copter.wp_nav->set_speed_down_cms(cmd.content.speed.target_ms * 100.0f);
            desired_speed_override.down = cmd.content.speed.target_ms;
            break;
        case SPEED_TYPE_AIRSPEED:
        case SPEED_TYPE_GROUNDSPEED:
            copter.wp_nav->set_speed_NE_cms(cmd.content.speed.target_ms * 100.0f);
            desired_speed_override.xy = cmd.content.speed.target_ms;
            break;
        }
    }
}

/**
 * @brief Process DO_SET_HOME command to change home/rally point location
 * 
 * @details Changes the home position to either current location or specified coordinates.
 *          Home position used for RTL (Return to Launch), altitude references, and
 *          failsafe return point. Critical for mission safety and navigation reference.
 * 
 *          Home Position Importance:
 *          - RTL returns to home when triggered
 *          - Altitude-Above-Home references home altitude
 *          - Failsafe actions use home as safe point
 *          - EKF origin typically set at first home
 *          - Distance-to-home calculations
 * 
 *          Set Home Options:
 *          1. Current Location (p1 == 1): Sets home to vehicle's current position
 *          2. Specified Location (p1 == 0): Sets home to provided lat/lon/alt
 *          3. Zero Location: Treated as current location (safety fallback)
 * 
 *          Current Location Method (p1 == 1):
 *          - Calls copter.set_home_to_current_location(false)
 *          - Uses current GPS position as new home
 *          - Useful for moving base scenarios
 *          - Updates home marker on GCS
 * 
 *          Specified Location Method:
 *          - Calls copter.set_home(cmd.content.location, false)
 *          - Uses provided latitude, longitude, altitude
 *          - Allows pre-planned home locations
 *          - Can set home before reaching location
 * 
 *          Zero Location Check:
 *          - If lat=0, lng=0, alt=0: Treated as current location
 *          - Safety feature prevents invalid (0,0,0) home
 *          - Condition: (lat == 0 && lng == 0 && alt == 0)
 *          - Results in set_home_to_current_location() call
 * 
 *          Lock Parameter (false):
 *          - Second parameter to set_home functions: false
 *          - false: Allows future SET_HOME commands
 *          - true: Would lock home (prevent changes)
 *          - Mission typically allows flexibility
 * 
 *          Failure Handling:
 *          - Both methods return bool (success/failure)
 *          - Failures ignored (commented: "ignore failure")
 *          - Mission continues regardless of outcome
 *          - Reasons for failure: Invalid GPS, invalid coordinates
 * 
 *          EKF Origin vs Home:
 *          - EKF origin: Set once at initialization, rarely changed
 *          - Home position: Can be changed anytime with DO_SET_HOME
 *          - Different purposes and constraints
 *          - Changing home does NOT move EKF origin
 * 
 *          RTL Behavior After Home Change:
 *          - RTL commands after DO_SET_HOME return to new home
 *          - Immediate effect on failsafe actions
 *          - Rally points not affected (separate system)
 *          - Distance-to-home calculations updated
 * 
 *          Typical Mission Usage:
 *          ```
 *          CMD 1: NAV_TAKEOFF                    // Takeoff from original home
 *          CMD 2: NAV_WAYPOINT (distant location)
 *          CMD 3: DO_SET_HOME (p1=1)             // Set home to current location
 *          CMD 4: NAV_WAYPOINT (survey area)     // Continue mission
 *          CMD 5: NAV_RTL                        // Return to new home
 *          ```
 * 
 *          Moving Base Scenario:
 *          - Vehicle takeoff from moving platform (boat, vehicle)
 *          - Platform moves during mission
 *          - Periodic DO_SET_HOME to platform's new location
 *          - Ensures RTL returns to current platform position
 * 
 *          Pre-Planned Home:
 *          - Mission starts at location A
 *          - DO_SET_HOME specifies location B
 *          - RTL and failsafes use location B
 *          - Useful for multi-staging operations
 * 
 *          GCS Updates:
 *          - Home position change sent to ground station
 *          - Home icon moves on map
 *          - Distance-to-home updates
 *          - Altitude references recalculated
 * 
 *          Altitude Reference Impact:
 *          - Altitude-Above-Home commands reference new home altitude
 *          - Existing waypoints maintain their altitude frames
 *          - May cause altitude shifts if not considered
 *          - Plan missions carefully when changing home
 * 
 *          Safety Considerations:
 *          - New home should be safe landing location
 *          - Consider failsafe consequences
 *          - Verify GPS accuracy before setting
 *          - Test in simulation first
 * 
 *          GPS Requirements:
 *          - Valid GPS fix required for current location
 *          - 3D fix minimum (horizontal + altitude)
 *          - Poor GPS may set invalid home
 *          - Function may fail with bad GPS
 * 
 *          Coordinate Frames:
 *          - Location uses standard Location class
 *          - Supports multiple altitude frames
 *          - Lat/lon in degrees * 1e7 (internal representation)
 *          - Altitude handling depends on frame
 * 
 *          Parameter p1 Interpretation:
 *          - p1 == 1: Use current location (ignore provided location)
 *          - p1 == 0 (or other): Use provided location
 *          - MAVLink standard parameter
 * 
 *          Mission Planner Usage:
 *          - "Set Home" command in mission planner
 *          - Option: "Current location" sets p1=1
 *          - Option: "Specified location" provides coordinates
 * 
 *          Multiple SET_HOME Commands:
 *          - Can have multiple in single mission
 *          - Each changes home to new location
 *          - Last command before RTL determines return point
 * 
 *          Logging:
 *          - Home changes logged to dataflash
 *          - GPS position logged at time of change
 *          - Useful for post-flight analysis
 * 
 * @param[in] cmd Mission command with home location parameters
 *                - p1: 1 = use current location, 0 = use specified
 *                - content.location: New home coordinates (if p1 != 1)
 *                - Coordinates: lat, lon, alt in standard Location format
 * 
 * @return void (attempts to set home, ignores failures)
 * 
 * @note Called by start_command() when DO_SET_HOME (179) encountered
 * @note Failures ignored - mission continues regardless
 * @note Zero coordinates (0,0,0) treated as current location
 * @note Does not lock home (allows future changes)
 * @note Immediately affects RTL and failsafe behavior
 * 
 * @see copter.set_home_to_current_location()
 * @see copter.set_home()
 */
void ModeAuto::do_set_home(const AP_Mission::Mission_Command& cmd)
{
    if (cmd.p1 == 1 || (cmd.content.location.lat == 0 && cmd.content.location.lng == 0 && cmd.content.location.alt == 0)) {
        if (!copter.set_home_to_current_location(false)) {
            // ignore failure
        }
    } else {
        if (!copter.set_home(cmd.content.location, false)) {
            // ignore failure
        }
    }
}

/**
 * @brief Process DO_SET_ROI command to point vehicle/camera at Region of Interest
 * 
 * @details Points camera mount and/or vehicle heading at specified location. Maintains
 *          tracking as vehicle moves. ROI (Region of Interest) used for automated camera
 *          pointing, building inspection, target tracking. Integrates with camera mounts
 *          and auto_yaw controller.
 * 
 *          ROI Command Types:
 *          - MAV_CMD_DO_SET_ROI_LOCATION (195): Point at specific coordinates
 *          - MAV_CMD_DO_SET_ROI (201): Legacy ROI command
 *          - MAV_CMD_DO_SET_ROI_NONE (197): Cancel ROI (handled as zero location)
 * 
 *          ROI Behavior:
 *          - Camera mount: Points camera at ROI continuously
 *          - No mount or mount without pan: Vehicle yaws to point at ROI
 *          - Mount with pan: Camera tracks, vehicle heading independent
 *          - Maintains pointing as vehicle moves along mission
 * 
 *          AutoYaw ROI Mode:
 *          - auto_yaw.set_roi(location) activates ROI mode
 *          - Sets AutoYaw::Mode::ROI
 *          - Continuously calculates bearing to ROI
 *          - Updates vehicle yaw each control loop
 * 
 *          ROI Tracking:
 *          - Vehicle position updated 400 Hz (fast loop)
 *          - Bearing to ROI recalculated each loop
 *          - Yaw command updated to track ROI
 *          - Smooth tracking as vehicle moves
 * 
 *          Camera Mount Integration:
 *          - If mount present: Camera points at ROI
 *          - Mount handles pan/tilt/roll independently
 *          - Vehicle may still yaw if mount lacks pan control
 *          - Best results with 3-axis gimbals
 * 
 *          No Mount Behavior:
 *          - Vehicle yaws to point at ROI location
 *          - Camera fixed to vehicle body frame
 *          - Vehicle heading continuously adjusted
 *          - May conflict with desired flight path
 * 
 *          Mount Without Pan:
 *          - Camera mount controls pitch (tilt)
 *          - Vehicle yaws to provide pan
 *          - Combined mount + vehicle pointing
 *          - Common with 1-axis or 2-axis gimbals
 * 
 *          ROI Location:
 *          - cmd.content.location: Target coordinates
 *          - Latitude, longitude in degrees * 1e7
 *          - Altitude in centimeters (various frames)
 *          - 3D point in space
 * 
 *          Altitude Frames:
 *          - ABOVE_HOME: Altitude above home position
 *          - ABOVE_ORIGIN: Altitude above EKF origin
 *          - ABOVE_TERRAIN: Altitude above terrain (if data available)
 *          - Affects vertical pointing angle
 * 
 *          ROI Coordinate Calculation:
 *          - Bearing: Calculated from vehicle to ROI (horizontal)
 *          - Pitch: Calculated based on altitude difference (vertical)
 *          - Distance: 3D distance vehicle to ROI
 *          - Continuous recalculation as vehicle moves
 * 
 *          ROI Persistence:
 *          - Remains active until explicitly cancelled
 *          - Cancellation: DO_SET_ROI_NONE or mode change
 *          - Survives waypoint transitions
 *          - Mission continues with ROI tracking
 * 
 *          Cancel ROI:
 *          - MAV_CMD_DO_SET_ROI_NONE: Clears ROI
 *          - Location (0,0,0) treated as cancel
 *          - Reverts to default yaw behavior
 *          - Camera mount returns to default
 * 
 *          Typical Mission Usage:
 *          ```
 *          CMD 1: NAV_TAKEOFF
 *          CMD 2: DO_SET_ROI (building coordinates)  // Start tracking building
 *          CMD 3: NAV_WAYPOINT (orbit point 1)       // Fly around while tracking
 *          CMD 4: NAV_WAYPOINT (orbit point 2)
 *          CMD 5: NAV_WAYPOINT (orbit point 3)
 *          CMD 6: DO_SET_ROI_NONE                    // Stop tracking
 *          ```
 * 
 *          Building Inspection Pattern:
 *          - Set ROI to building center
 *          - Fly waypoints around building perimeter
 *          - Camera/vehicle tracks building throughout
 *          - Automated inspection with continuous pointing
 * 
 *          Tower/Structure Inspection:
 *          - ROI at structure base or center
 *          - Vertical waypoint climb around structure
 *          - Maintains camera pointing at structure
 *          - Captures all angles during ascent/descent
 * 
 *          Target Tracking:
 *          - ROI set to ground target location
 *          - Vehicle flies surveillance pattern
 *          - Camera tracks target throughout
 *          - Useful for monitoring applications
 * 
 *          Yaw Override Priority:
 *          - CONDITION_YAW command overrides ROI
 *          - After yaw command completes, may revert to previous yaw mode
 *          - Check verify_yaw() for mode management
 *          - Plan mission carefully with mixed commands
 * 
 *          ROI vs CONDITION_YAW:
 *          - ROI: Continuous dynamic tracking
 *          - CONDITION_YAW: Fixed heading command
 *          - ROI survives waypoint changes
 *          - CONDITION_YAW single heading
 * 
 *          Mount Control Interaction:
 *          - DO_MOUNT_CONTROL: Manual camera angle override
 *          - May conflict with ROI tracking
 *          - Last command typically takes precedence
 *          - Test interaction in simulation
 * 
 *          Camera Triggering with ROI:
 *          - ROI ensures camera points at target
 *          - DO_DIGICAM_CONTROL triggers capture
 *          - Combine for automated target photography
 *          - Useful for surveys and inspections
 * 
 *          Advantages:
 *          - Automated camera pointing
 *          - No manual pilot input required
 *          - Consistent results
 *          - Frees pilot for navigation monitoring
 * 
 *          Limitations:
 *          - Fixed target location (not moving targets)
 *          - May conflict with optimal flight path
 *          - Aggressive yaw may affect stability
 *          - Mount limitations (pan range, speed)
 * 
 *          Future Enhancements (TO-DO):
 *          - Point at moving waypoint
 *          - Point at another vehicle
 *          - Track moving target
 *          - ROI with offset
 * 
 *          Implementation Note:
 *          - Simple passthrough to auto_yaw.set_roi()
 *          - AutoYaw class handles all tracking logic
 *          - Camera mount handled separately
 *          - Clean separation of concerns
 * 
 * @param[in] cmd Mission command with ROI location
 *                - content.location: Target ROI coordinates (lat, lon, alt)
 *                - Location with standard altitude frame
 *                - Zero location (0,0,0) cancels ROI
 * 
 * @return void (configures AutoYaw ROI tracking)
 * 
 * @note Called by start_command() for MAV_CMD_DO_SET_ROI (201),
 *       MAV_CMD_DO_SET_ROI_LOCATION (195), MAV_CMD_DO_SET_ROI_NONE (197)
 * @note ROI persists across waypoints until explicitly cancelled
 * @note Vehicle yaw behavior depends on camera mount capabilities
 * @note TO-DO: Add support for waypoint-relative ROI
 * @note CONDITION_YAW command can override ROI
 * 
 * @see auto_yaw.set_roi()
 * @see do_mount_control() for manual camera control
 * @see do_yaw() for fixed heading commands
 */
void ModeAuto::do_roi(const AP_Mission::Mission_Command& cmd)
{
    auto_yaw.set_roi(cmd.content.location);
}

#if HAL_MOUNT_ENABLED
/**
 * @brief Process DO_MOUNT_CONTROL command to position camera gimbal to specific angles
 * 
 * @details Commands camera mount to specific roll, pitch, yaw angles. Used for precise
 *          camera positioning independent of vehicle attitude. Supports gimbals with
 *          1, 2, or 3 axes. Handles mounts without pan by yawing vehicle instead.
 * 
 *          Mount Types Supported:
 *          - 3-axis gimbals: Full roll, pitch, yaw control
 *          - 2-axis gimbals: Typically pitch + roll or pitch + yaw
 *          - 1-axis gimbals: Typically pitch (tilt) only
 *          - Servo mounts: Direct servo angle control
 * 
 *          Angle Parameters:
 *          - roll: Camera roll angle in degrees (-180 to +180)
 *          - pitch: Camera pitch/tilt angle in degrees (-90 to +90)
 *          - yaw: Camera yaw/pan angle in degrees (-180 to +180)
 *          - Angles in body frame (relative to vehicle)
 * 
 *          Body Frame Angles:
 *          - Roll: Left negative, right positive
 *          - Pitch: Down negative, up positive  
 *          - Yaw: Left negative, right positive
 *          - Relative to vehicle forward direction
 * 
 *          No-Pan Workaround:
 *          - If mount lacks pan control: Vehicle yaws instead
 *          - Check: has_pan_control() returns false
 *          - auto_yaw.set_yaw_angle_offset_deg() yaws vehicle
 *          - Yaw angle treated as body-frame offset
 *          - Combines vehicle yaw + camera tilt
 * 
 *          Has Pan Control Check:
 *          - copter.camera_mount.has_pan_control()
 *          - true: Mount handles yaw independently
 *          - false: Vehicle must yaw for pan effect
 *          - Determined by mount configuration
 * 
 *          Vehicle Yaw Compensation:
 *          - Yaw angle in body frame (per AP_Mount handler)
 *          - Equivalent to offset from current yaw demand
 *          - auto_yaw.set_yaw_angle_offset_deg() applies offset
 *          - Vehicle yaws by specified angle
 *          - Camera mount handles pitch/roll
 * 
 *          Mount Angle Command:
 *          - copter.camera_mount.set_angle_target()
 *          - Parameters: (roll, pitch, yaw, is_earth_frame)
 *          - is_earth_frame = false (body frame angles)
 *          - Mount hardware executes positioning
 * 
 *          Execution Flow:
 *          1. Check if mount present and lacks pan
 *          2. If no pan: Command vehicle yaw offset
 *          3. Send angles to mount hardware
 *          4. Mount positions camera to angles
 *          5. Mission continues immediately (DO command)
 * 
 *          DO Command Nature:
 *          - Executes immediately
 *          - No verification needed (fire-and-forget)
 *          - Mission continues to next command
 *          - Mount tracks to target in background
 * 
 *          Typical Mission Usage:
 *          ```
 *          CMD 1: NAV_WAYPOINT
 *          CMD 2: DO_MOUNT_CONTROL (pitch=-45°)   // Look down 45°
 *          CMD 3: DO_DIGICAM_CONTROL              // Take photo
 *          CMD 4: DO_MOUNT_CONTROL (pitch=-90°)   // Look straight down
 *          CMD 5: DO_DIGICAM_CONTROL              // Take nadir photo
 *          ```
 * 
 *          Oblique Photography:
 *          - Set pitch to angle (e.g., -30° to -60°)
 *          - Captures building facades
 *          - Better perspective than nadir
 *          - Useful for 3D reconstruction
 * 
 *          Nadir Photography (straight down):
 *          - pitch = -90°
 *          - roll = 0°
 *          - yaw = 0° (or any if mount has pan)
 *          - Standard mapping/survey configuration
 * 
 *          Horizon Scan:
 *          - pitch = 0° (level)
 *          - Yaw through 360° with multiple commands
 *          - Captures panoramic horizon
 *          - Useful for surveillance
 * 
 *          Mount Neutral Position:
 *          - roll = 0°, pitch = 0°, yaw = 0°
 *          - Camera aligned with vehicle body
 *          - Forward-facing level
 *          - Default/stowed position
 * 
 *          Angle Limits:
 *          - Mount hardware has mechanical limits
 *          - Commanded angles clamped to limits
 *          - Typical pitch: -90° to +30°
 *          - Typical roll: -45° to +45° (if supported)
 *          - Yaw: 0° to 360° or limited range
 * 
 *          Mount Configuration Parameters:
 *          - MNT_TYPE: Mount type (servo, gimbal, etc.)
 *          - MNT_NEUTRAL_X/Y/Z: Neutral position angles
 *          - MNT_ANGLE_MIN/MAX: Angle limits
 *          - MNT_LEAD_RLL/PTCH: Lead compensation
 * 
 *          Coordinate with ROI:
 *          - DO_MOUNT_CONTROL: Manual angle command
 *          - DO_SET_ROI: Automatic tracking
 *          - May conflict if both used
 *          - Last command typically wins
 * 
 *          Coordinate with Camera:
 *          - Mount positions camera
 *          - DO_DIGICAM_CONTROL triggers capture
 *          - DO_SET_CAM_TRIGG_DIST for interval
 *          - Coordinated for automated photography
 * 
 *          Mount Stabilization:
 *          - Most gimbals provide stabilization
 *          - Maintains camera angle despite vehicle motion
 *          - Commanded angle maintained through maneuvers
 *          - Improves image quality
 * 
 *          Mount Retract:
 *          - Some mounts support retract
 *          - Protects camera during aggressive flight
 *          - DO_MOUNT_CONTROL can command retract position
 *          - Check mount documentation
 * 
 *          Multiple Mount Commands:
 *          - Can issue multiple DO_MOUNT_CONTROL in mission
 *          - Each changes camera angle
 *          - Smooth transitions between angles
 *          - Rate limits prevent jerky motion
 * 
 *          Mount vs Vehicle Attitude:
 *          - Mount angles independent of vehicle
 *          - Vehicle can bank/pitch while camera stable
 *          - Gimbal compensates for vehicle motion
 *          - Key advantage of gimbal systems
 * 
 *          No Mount Installed:
 *          - If mount_type == None: Command ignored
 *          - Mission continues normally
 *          - No error generated
 *          - Safe for missions on unmounted vehicles
 * 
 *          Mount Slew Rate:
 *          - Mounts move at configured slew rate
 *          - Prevents jerky motion
 *          - Configured in mount parameters
 *          - Large angle changes take time
 * 
 *          Testing:
 *          - Test mount commands in simulation first
 *          - Verify angle limits and behavior
 *          - Check mechanical clearances
 *          - Ensure camera framing correct
 * 
 * @param[in] cmd Mission command with mount angle targets
 *                - content.mount_control.roll: Roll angle in degrees
 *                - content.mount_control.pitch: Pitch angle in degrees
 *                - content.mount_control.yaw: Yaw angle in degrees (body frame)
 * 
 * @return void (commands camera mount hardware)
 * 
 * @note Only compiled if HAL_MOUNT_ENABLED
 * @note Called by start_command() for MAV_CMD_DO_MOUNT_CONTROL (205)
 * @note Executes immediately, no verification needed (DO command)
 * @note If mount lacks pan, vehicle yaws instead
 * @note Yaw angle in body frame (offset from current heading)
 * @note Mount hardware applies stabilization automatically
 * 
 * @see do_roi() for automatic target tracking
 * @see auto_yaw.set_yaw_angle_offset_deg() for vehicle yaw compensation
 * @see copter.camera_mount.set_angle_target()
 */
void ModeAuto::do_mount_control(const AP_Mission::Mission_Command& cmd)
{
    // if vehicle has a camera mount but it doesn't do pan control then yaw the entire vehicle instead
    if ((copter.camera_mount.get_mount_type() != AP_Mount::Type::None) &&
        !copter.camera_mount.has_pan_control()) {
        // Per the handler in AP_Mount, DO_MOUNT_CONTROL yaw angle is in body frame, which is
        // equivalent to an offset to the current yaw demand.
        auto_yaw.set_yaw_angle_offset_deg(cmd.content.mount_control.yaw);
    }
    // pass the target angles to the camera mount
    copter.camera_mount.set_angle_target(cmd.content.mount_control.roll, cmd.content.mount_control.pitch, cmd.content.mount_control.yaw, false);
}
#endif  // HAL_MOUNT_ENABLED

#if AP_WINCH_ENABLED
/**
 * @brief Process DO_WINCH command to control winch/tether for payload deployment
 * 
 * @details Controls winch motor to extend/retract cable for lowering/raising payloads.
 *          Supports relaxed (free-spool), length control, and rate control modes.
 *          Used for payload delivery, sampling, or tethered operations. Executes
 *          as DO command (fire-and-forget, no verification).
 * 
 *          Winch Actions Supported:
 *          - WINCH_RELAXED: Disengage motor, allow free-spool
 *          - WINCH_RELATIVE_LENGTH_CONTROL: Extend/retract specific length
 *          - WINCH_RATE_CONTROL: Continuous extend/retract at rate
 * 
 *          WINCH_RELAXED Mode (0):
 *          - Disengages winch motor
 *          - Cable free to spool out
 *          - Gravity pulls payload down
 *          - No active control
 *          - Used for initial descent or emergency
 * 
 *          Relaxed Use Cases:
 *          - Emergency release
 *          - Passive payload lowering
 *          - Reduce motor load
 *          - Allow manual cable adjustment
 * 
 *          WINCH_RELATIVE_LENGTH_CONTROL Mode (1):
 *          - Extends or retracts cable by specified length
 *          - release_length: Length in meters (+ extends, - retracts)
 *          - Motor stops when length reached
 *          - Closed-loop position control
 * 
 *          Length Control Use Cases:
 *          - Lower payload X meters
 *          - Raise payload X meters  
 *          - Precise positioning
 *          - Multi-stage deployment
 * 
 *          Length Control Examples:
 *          - release_length = 10.0: Extend 10 meters
 *          - release_length = -5.0: Retract 5 meters
 *          - Relative to current position
 *          - Motor automatically stops at target
 * 
 *          WINCH_RATE_CONTROL Mode (2):
 *          - Extends or retracts at specified rate
 *          - release_rate: Rate in m/s (+ extends, - retracts)
 *          - Continuous until stopped by another command
 *          - Open-loop velocity control
 * 
 *          Rate Control Use Cases:
 *          - Continuous lowering during flight
 *          - Controlled ascent
 *          - Dynamic operations
 *          - Time-based deployment
 * 
 *          Rate Control Examples:
 *          - release_rate = 0.5: Extend at 0.5 m/s
 *          - release_rate = -0.3: Retract at 0.3 m/s
 *          - Must issue stop command to halt
 *          - Rate control until mode change
 * 
 *          Command Parameters:
 *          - action: Winch action type (RELAXED, LENGTH, RATE)
 *          - release_length: Length in meters (for LENGTH mode)
 *          - release_rate: Rate in m/s (for RATE mode)
 *          - gripper_num: Gripper ID (currently ignored, only 1 supported)
 * 
 *          Winch Hardware:
 *          - Brushless motor with encoder
 *          - Cable drum with length tracking
 *          - Clutch/brake mechanism
 *          - Load sensor (optional)
 * 
 *          Winch Configuration:
 *          - WINCH_TYPE: Winch hardware type
 *          - WINCH_RATE_MAX: Maximum spool rate (m/s)
 *          - WINCH_POS_MIN/MAX: Cable length limits (m)
 *          - WINCH_RELAX: Relaxed brake behavior
 * 
 *          Length Tracking:
 *          - Encoder measures cable extension
 *          - Zero reference at initialization
 *          - Tracks total extended length
 *          - Resets with calibration
 * 
 *          Safety Limits:
 *          - Maximum extension limit (WINCH_POS_MAX)
 *          - Maximum rate limit (WINCH_RATE_MAX)
 *          - Commanded values clamped to limits
 *          - Prevents cable overrun
 * 
 *          Typical Mission Sequence:
 *          ```
 *          CMD 1: NAV_WAYPOINT (delivery location)
 *          CMD 2: NAV_LOITER_TIME (30 seconds)
 *          CMD 3: DO_WINCH (RATE, 0.5 m/s extend)     // Start lowering
 *          CMD 4: CONDITION_DELAY (20 seconds)        // Lower for 20s = 10m
 *          CMD 5: DO_WINCH (RATE, 0.0 m/s)            // Stop lowering
 *          CMD 6: DO_GRIPPER (RELEASE)                // Release payload
 *          CMD 7: DO_WINCH (RATE, -0.5 m/s)           // Retract cable
 *          CMD 8: CONDITION_DELAY (20 seconds)        // Retract
 *          CMD 9: DO_WINCH (RATE, 0.0)                // Stop retract
 *          ```
 * 
 *          Payload Delivery Pattern:
 *          1. Fly to delivery location
 *          2. Hover or loiter
 *          3. Extend cable to lower payload
 *          4. Detect ground contact (load sensor)
 *          5. Release gripper
 *          6. Retract cable
 *          7. Resume mission
 * 
 *          Sampling Pattern:
 *          1. Lower sampling device on cable
 *          2. Contact surface (water, ground)
 *          3. Dwell for sample collection
 *          4. Retract with sample
 *          5. Continue to next location
 * 
 *          Load Sensing:
 *          - Optional load cell on winch
 *          - Detects cable tension
 *          - Can detect payload touchdown
 *          - Useful for automated delivery
 * 
 *          Gripper Integration:
 *          - Winch lowers payload
 *          - Gripper holds payload during lowering
 *          - DO_GRIPPER releases at bottom
 *          - Coordinate winch + gripper commands
 * 
 *          Cable Management:
 *          - Prevent cable slack
 *          - Maintain tension during flight
 *          - Avoid cable tangling
 *          - Retract cable when not in use
 * 
 *          Vehicle Stability:
 *          - Extending cable changes center of gravity
 *          - Heavy payloads affect stability
 *          - Slow, controlled operations
 *          - Monitor vehicle attitude
 * 
 *          Wind Effects:
 *          - Cable swings in wind
 *          - Payload pendulum motion
 *          - Affects vehicle position hold
 *          - Reduce speeds in windy conditions
 * 
 *          Emergency Procedures:
 *          - WINCH_RELAXED: Quick release
 *          - Cut cable if necessary
 *          - Land with extended cable if safe
 *          - Practice emergency scenarios
 * 
 *          Multi-Gripper Note:
 *          - gripper_num parameter currently ignored
 *          - Code comment: "only support one gripper"
 *          - Future: May support multiple winches
 *          - Current: Single g2.winch instance
 * 
 *          DO Command Behavior:
 *          - Executes immediately
 *          - No verification (fire-and-forget)
 *          - Mission continues to next command
 *          - Winch operates in background
 * 
 *          Stop Winch:
 *          - Send RATE_CONTROL with rate = 0.0
 *          - Or send LENGTH_CONTROL with length = 0.0
 *          - Holds current position
 *          - Motor brake engaged
 * 
 *          Unknown Actions:
 *          - Default case: Do nothing
 *          - No error message
 *          - Mission continues
 *          - Safe fallback behavior
 * 
 *          Testing:
 *          - Test winch in simulation first
 *          - Verify length tracking accuracy
 *          - Test emergency stop procedures
 *          - Check payload weight limits
 * 
 * @param[in] cmd Mission command with winch control parameters
 *                - content.winch.action: RELAXED (0), LENGTH (1), or RATE (2)
 *                - content.winch.release_length: Length in meters (LENGTH mode)
 *                - content.winch.release_rate: Rate in m/s (RATE mode)
 * 
 * @return void (commands winch hardware)
 * 
 * @note Only compiled if AP_WINCH_ENABLED
 * @note Called by start_command() for MAV_CMD_DO_WINCH
 * @note gripper_num parameter ignored (only one winch supported)
 * @note Executes immediately, no verification (DO command)
 * @note Unknown actions ignored (default case)
 * 
 * @warning Extended cable affects vehicle CG and stability
 * @warning Heavy payloads may exceed motor limits
 * @warning Monitor cable length to prevent overrun
 * 
 * @see g2.winch.relax() for relaxed mode
 * @see g2.winch.release_length() for length control
 * @see g2.winch.set_desired_rate() for rate control
 * @see do_payload_place() for automated placement
 */
void ModeAuto::do_winch(const AP_Mission::Mission_Command& cmd)
{
    // Note: we ignore the gripper num parameter because we only support one gripper
    switch (cmd.content.winch.action) {
        case WINCH_RELAXED:
            g2.winch.relax();
            break;
        case WINCH_RELATIVE_LENGTH_CONTROL:
            g2.winch.release_length(cmd.content.winch.release_length);
            break;
        case WINCH_RATE_CONTROL:
            g2.winch.set_desired_rate(cmd.content.winch.release_rate);
            break;
        default:
            // do nothing
            break;
    }
}
#endif

#if AP_MISSION_NAV_PAYLOAD_PLACE_ENABLED && AC_PAYLOAD_PLACE_ENABLED
/**
 * @brief Process NAV_PAYLOAD_PLACE command to automatically detect and place payload
 * 
 * @details Intelligent payload placement using thrust sensing to detect touchdown.
 *          Descends until payload contacts ground (detected by reduced thrust),
 *          releases gripper, then ascends. Safer than fixed-altitude landing for
 *          uneven terrain. Optional horizontal flight to placement location first.
 * 
 *          Payload Place Sequence:
 *          1. Fly to location (if coordinates provided)
 *          2. Descend at controlled rate
 *          3. Monitor thrust levels during descent
 *          4. Detect payload touchdown (thrust drops)
 *          5. Release gripper
 *          6. Delay for release confirmation
 *          7. Ascend back to starting altitude
 *          8. Continue mission
 * 
 *          Location Handling:
 *          - If lat/lon provided: Fly to location first (State::FlyToLocation)
 *          - If lat/lon zero: Start descent immediately at current position
 *          - Altitude set to current altitude (maintains altitude during approach)
 *          - Target location in cmd.content.location
 * 
 *          Fly to Location Mode:
 *          - payload_place.state = PayloadPlace::State::FlyToLocation
 *          - wp_start() navigates to target coordinates
 *          - Maintains current altitude during approach
 *          - When destination reached, transitions to descent
 * 
 *          Immediate Descent Mode:
 *          - No horizontal navigation
 *          - payload_place.start_descent() begins lowering
 *          - Descends from current position
 *          - Used when already at placement location
 * 
 *          Altitude Frame Handling:
 *          - shift_alt_to_current_alt() sets target altitude
 *          - Converts to current altitude in appropriate frame
 *          - Handles terrain-relative altitudes
 *          - Falls back to ABOVE_HOME if terrain data missing
 * 
 *          Terrain Data Failure:
 *          - If terrain data missing: Use ABOVE_HOME frame
 *          - Log error: LogErrorSubsystem::TERRAIN
 *          - GCS message: "no terrain data, using alt-above-home"
 *          - MAV_SEVERITY_CRITICAL message
 *          - Mission continues with fallback
 * 
 *          Waypoint Navigation Failure:
 *          - wp_start() failure indicates missing terrain data
 *          - copter.failsafe_terrain_on_event() triggers terrain failsafe
 *          - May abort mission or trigger failsafe action
 *          - Function returns early
 * 
 *          Maximum Descent Distance:
 *          - payload_place.descent_max_cm = cmd.p1
 *          - p1 parameter: Maximum descent in centimeters
 *          - Prevents excessive descent if ground not detected
 *          - Safety limit for unexpected terrain
 * 
 *          Descent Max Safety:
 *          - If ground not detected within max distance
 *          - Abort placement and ascend
 *          - Prevents flying into obstacles below
 *          - Typical values: 500-1000 cm (5-10 meters)
 * 
 *          Thrust Sensing Algorithm:
 *          - Measures throttle during descent
 *          - Descends at constant rate (PLDP_DESCENT_SPEED_MS)
 *          - Calibrates thrust for descent (2 seconds)
 *          - Detects thrust drop below threshold (PLDP_THRUST_PLACED_FRACTION)
 *          - Indicates payload touchdown
 * 
 *          Thrust Detection Process:
 *          1. Descend at PLDP_DESCENT_SPEED_MS (e.g., 0.5 m/s)
 *          2. Wait for steady descent (descent_thrust_cal_duration_ms = 2000ms)
 *          3. Record minimum descent thrust
 *          4. Monitor for thrust < (PLDP_THRUST_PLACED_FRACTION * descent_thrust)
 *          5. When threshold crossed: Payload detected on ground
 * 
 *          Rangefinder Integration:
 *          - Optional: PLDP_RANGE_FINDER_MAXIMUM_M parameter
 *          - If set: Also check rangefinder altitude
 *          - Payload place only if rangefinder < threshold
 *          - Adds confidence to thrust detection
 *          - Useful for detecting false positives
 * 
 *          Gripper Release:
 *          - When touchdown detected: Release gripper
 *          - AP::gripper().release() opens gripper
 *          - Payload drops from gripper
 *          - Delay ensures complete release
 * 
 *          Post-Release Delay:
 *          - PLDP_DELAY_S parameter (seconds)
 *          - Wait after gripper release
 *          - Ensures payload fully released
 *          - Prevents re-grabbing during ascent
 * 
 *          Ascent Phase:
 *          - Ascend back to descent_start_altitude_cm
 *          - Returns to altitude before descent
 *          - Controlled ascent rate
 *          - Clears payload area
 * 
 *          State Machine:
 *          - FlyToLocation: Navigate to coordinates
 *          - Descent_Start: Initialize descent parameters
 *          - Descent: Controlled descent, monitor thrust
 *          - Release: Gripper release command
 *          - Releasing: Wait for gripper confirmation
 *          - Delay: Post-release dwell
 *          - Ascent_Start: Begin ascent
 *          - Ascent: Return to original altitude
 *          - Done: Complete, mission continues
 * 
 *          Abort Conditions:
 *          - Maximum descent reached without touchdown
 *          - Rangefinder check fails
 *          - Gripper manually released (pilot override)
 *          - Terrain failsafe triggered
 * 
 *          Manual Gripper Override:
 *          - Pilot can release gripper manually
 *          - Detected during descent or approach
 *          - Aborts placement sequence
 *          - Transitions to Done state
 * 
 *          Land Complete Detection:
 *          - copter.ap.land_complete or land_complete_maybe
 *          - Indicates vehicle on ground
 *          - Triggers immediate gripper release
 *          - Safety feature for unintended landings
 * 
 *          SubMode Assignment:
 *          - set_submode(SubMode::NAV_PAYLOAD_PLACE)
 *          - Activates payload place controller
 *          - PayloadPlace::run() called each loop
 *          - Handles all placement state machine
 * 
 *          Configuration Parameters:
 *          - PLDP_DESCENT_SPEED_MS: Descent rate (m/s)
 *          - PLDP_THRUST_PLACED_FRACTION: Thrust threshold (0-1)
 *          - PLDP_DELAY_S: Post-release delay (seconds)
 *          - PLDP_RANGE_FINDER_MAXIMUM_M: Rangefinder max (meters)
 * 
 *          Typical Mission Usage:
 *          ```
 *          CMD 1: NAV_WAYPOINT (approach waypoint)
 *          CMD 2: NAV_PAYLOAD_PLACE (placement coordinates, max_descent=1000cm)
 *          CMD 3: NAV_WAYPOINT (departure waypoint)
 *          ```
 * 
 *          Advantages vs Fixed Landing:
 *          - Adapts to terrain variations
 *          - No precise altitude knowledge needed
 *          - Gentle payload touchdown
 *          - Automated release detection
 *          - No manual timing required
 * 
 *          Typical Parameters:
 *          - Descent speed: 0.3-0.8 m/s
 *          - Thrust fraction: 0.7-0.9
 *          - Max descent: 5-10 meters
 *          - Post-release delay: 1-3 seconds
 * 
 *          Testing Recommendations:
 *          - Test in simulation first
 *          - Verify thrust detection threshold
 *          - Practice with test payloads
 *          - Start with high altitude safety margins
 *          - Monitor thrust levels in logs
 * 
 * @param[in] cmd Mission command with placement parameters
 *                - content.location: Target placement coordinates (lat, lon)
 *                  * If (0,0): Place at current location
 *                  * If coordinates: Fly to location first
 *                - p1: Maximum descent distance in centimeters
 *                  * Safety limit to prevent excessive descent
 * 
 * @return void (configures payload placement state machine)
 * 
 * @note Only compiled if AP_MISSION_NAV_PAYLOAD_PLACE_ENABLED && AC_PAYLOAD_PLACE_ENABLED
 * @note Called by start_command() for MAV_CMD_NAV_PAYLOAD_PLACE (94)
 * @note This is a NAV command (requires verification, see verify_payload_place())
 * @note Requires gripper hardware for payload release
 * @note Thrust sensing requires stable descent for accurate detection
 * @note Optional rangefinder enhances detection reliability
 * 
 * @warning Ensure sufficient altitude for max descent safety margin
 * @warning Test thrust threshold with actual payload weight
 * @warning Avoid obstacles below placement altitude
 * 
 * @see PayloadPlace::run() for state machine execution
 * @see PayloadPlace::start_descent() for descent initialization
 * @see verify_payload_place() for completion detection
 * @see do_winch() for alternative payload lowering method
 */
void ModeAuto::do_payload_place(const AP_Mission::Mission_Command& cmd)
{
    // if location provided we fly to that location at current altitude
    if (cmd.content.location.lat != 0 || cmd.content.location.lng != 0) {
        // set state to fly to location
        payload_place.state = PayloadPlace::State::FlyToLocation;

        // convert cmd to location class
        Location target_loc(cmd.content.location);
        if (!shift_alt_to_current_alt(target_loc)) {
            // this can only fail due to missing terrain database alt or rangefinder alt
            // use current alt-above-home and report error
            target_loc.set_alt_cm(copter.current_loc.alt, Location::AltFrame::ABOVE_HOME);
            LOGGER_WRITE_ERROR(LogErrorSubsystem::TERRAIN, LogErrorCode::MISSING_TERRAIN_DATA);
            gcs().send_text(MAV_SEVERITY_CRITICAL, "PayloadPlace: no terrain data, using alt-above-home");
        }
        if (!wp_start(target_loc)) {
            // failure to set next destination can only be because of missing terrain data
            copter.failsafe_terrain_on_event();
            return;
        }
    } else {
        // initialise placing controller
        payload_place.start_descent();
    }
    payload_place.descent_max_cm = cmd.p1;

    // set submode
    set_submode(SubMode::NAV_PAYLOAD_PLACE);
}
#endif  // AP_MISSION_NAV_PAYLOAD_PLACE_ENABLED

/**
 * @brief Process NAV_RETURN_TO_LAUNCH mission command to return home within Auto mode
 * 
 * @details Executes RTL (Return To Launch) as a mission command within Auto mode.
 *          Delegates to ModeRTL for actual RTL behavior while remaining in Auto mode.
 *          Used to return home as part of automated mission sequence.
 * 
 *          RTL Within Auto Mode:
 *          - Stays in Auto mode (doesn't switch to RTL mode)
 *          - Uses RTL flight mode logic for navigation
 *          - SubMode set to RTL for internal tracking
 *          - Mission continues after RTL completes (if configured)
 * 
 *          Initialization:
 *          - Calls rtl_start() to configure RTL submode
 *          - rtl_start() invokes copter.mode_rtl.init(true)
 *          - ignore_checks=true: RTL init cannot fail
 *          - Sets set_submode(SubMode::RTL)
 * 
 *          RTL Execution Flow:
 *          1. rtl_start() initializes RTL navigation
 *          2. SubMode::RTL set in mode state machine
 *          3. auto_run() dispatches to rtl_run()
 *          4. rtl_run() calls copter.mode_rtl.run(false)
 *          5. verify_RTL() checks completion status
 * 
 *          RTL Phases (from ModeRTL):
 *          1. Initial Climb: Climb to RTL_ALT (if below)
 *          2. Return: Navigate horizontally to home location
 *          3. Descent: Descend to final altitude
 *          4. Land: Execute landing sequence
 * 
 *          RTL Completion Detection (verify_RTL):
 *          - Checks mode_rtl.state_complete()
 *          - Must be in FINAL_DESCENT or LAND state
 *          - Motors must reach GROUND_IDLE spool state
 *          - All three conditions required for completion
 * 
 *          Home Location:
 *          - Returns to position stored in AP_AHRS home
 *          - Typically set at arming or via DO_SET_HOME
 *          - If no home set, RTL behavior undefined (may loiter)
 * 
 *          Altitude Behavior:
 *          - RTL_ALT parameter: Altitude for return flight
 *          - RTL_ALT_FINAL parameter: Final altitude before landing
 *          - If current altitude > RTL_ALT: Maintains current altitude
 *          - If current altitude < RTL_ALT: Climbs to RTL_ALT first
 * 
 *          RTL_CLIMB_MIN:
 *          - Minimum altitude gain before lateral movement
 *          - Ensures clearance of nearby obstacles
 *          - Applied even if already above RTL_ALT
 * 
 *          Difference from Mode Switch:
 *          - Mode RTL: Separate flight mode, mission stops
 *          - Mission NAV_RTL: Remains in Auto, mission continues
 *          - Mission RTL useful for return-and-continue sequences
 * 
 *          Mission Continuation:
 *          - After RTL completes, mission advances to next command
 *          - Allows automated return, land, then continue
 *          - Or return home, then execute post-RTL commands
 * 
 *          Terrain Following:
 *          - RTL respects terrain following settings
 *          - Can return at terrain-relative altitude
 *          - Depends on RTL_ALT frame configuration
 * 
 *          Failsafe Interaction:
 *          - This is NOT a failsafe RTL (that's mode switch)
 *          - Normal mission command execution
 *          - Pilot can still override with mode switch
 * 
 *          Rally Point Support:
 *          - If rally points configured, may return to nearest rally
 *          - Depends on RTL_ALT_TYPE parameter
 *          - Rally point provides alternate return destination
 * 
 *          Yaw Behavior:
 *          - RTL mode controls yaw (typically points toward home)
 *          - use_pilot_yaw() returns true during RTL submode
 *          - Pilot yaw input accepted during return
 * 
 *          Landing Behavior:
 *          - Uses standard landing logic from ModeRTL
 *          - Precision landing if configured and detected
 *          - Disarms after landing if MIS_OPTIONS allows
 * 
 *          Auto RTL Feature:
 *          - Different from DO_LAND_START auto RTL
 *          - This is explicit mission command RTL
 *          - auto_RTL flag not set for mission NAV_RTL
 * 
 *          Parameter Dependencies:
 *          - RTL_ALT: Return altitude (cm)
 *          - RTL_ALT_FINAL: Final altitude before land (cm)
 *          - RTL_CLIMB_MIN: Minimum initial climb (cm)
 *          - RTL_SPEED: Horizontal return speed (cm/s)
 *          - RTL_CONE_SLOPE: Cone approach slope
 *          - WPNAV speeds: Control ascent/descent rates
 * 
 *          Typical Use Cases:
 *          - Return home mid-mission for battery/data offload
 *          - Survey mission: Return between coverage areas
 *          - Conditional return based on mission logic
 *          - Safety return before continuing mission
 * 
 * @param void (no parameters, uses current position and home location)
 * 
 * @return void (initializes RTL submode, execution via rtl_run)
 * 
 * @note Called by start_command() when NAV_RETURN_TO_LAUNCH (20) encountered
 * @note Remains in Auto mode (doesn't switch to RTL flight mode)
 * @note rtl_run() delegates to copter.mode_rtl.run(false)
 * @note verify_RTL() checks completion for mission advancement
 * @note Mission continues to next command after RTL completes
 * @note Home location must be set for RTL to function properly
 * 
 * @see rtl_start() for RTL submode initialization
 * @see rtl_run() for RTL execution within Auto
 * @see verify_RTL() for completion detection
 * @see ModeRTL for complete RTL behavior description
 */
void ModeAuto::do_RTL(void)
{
    // start rtl in auto flight mode
    rtl_start();
}

/********************************************************************************/
// Verify Nav (Must) commands
/********************************************************************************/

/**
 * @brief Verify NAV_TAKEOFF command completion
 * 
 * @details Checks if takeoff has reached target altitude and triggers landing
 *          gear retraction when complete. Called repeatedly by mission library
 *          at 10Hz+ until returns true. Takeoff complete when vehicle reaches
 *          target altitude with sufficient vertical velocity margin.
 * 
 *          Completion Criteria (in auto_takeoff):
 *          - Reached target altitude (within WP_NAVALT_MIN tolerance)
 *          - Vertical velocity below threshold
 *          - auto_takeoff.complete flag set to true
 *          - Position controller stable
 * 
 *          Landing Gear Retraction:
 *          - If AP_LANDINGGEAR_ENABLED compiled
 *          - Triggers when auto_takeoff.complete becomes true
 *          - copter.landinggear.retract_after_takeoff() retracts gear
 *          - Reduces drag for forward flight
 *          - Improves aerodynamics and efficiency
 * 
 *          Retraction Timing:
 *          - Waits until takeoff complete
 *          - Ensures sufficient altitude before retraction
 *          - Safe distance from ground
 *          - Prevents gear damage on retraction
 * 
 *          Auto Takeoff Controller:
 *          - auto_takeoff object manages climb
 *          - Handles altitude ramping
 *          - WP_NAVALT_MIN parameter: Completion threshold
 *          - Smooth transition to next waypoint
 * 
 *          Altitude Completion Logic:
 *          - Within WP_NAVALT_MIN of target altitude
 *          - Vertical velocity near zero
 *          - Position hold stable
 *          - Ready for horizontal navigation
 * 
 *          Mission Flow:
 *          - While verify returns false: Continue climbing
 *          - When verify returns true: Advance to next command
 *          - Landing gear retracts on completion
 *          - Typical next: NAV_WAYPOINT
 * 
 *          Repeated Calls:
 *          - Mission library calls this 10Hz or faster
 *          - Each call checks auto_takeoff.complete
 *          - Returns same value until complete
 *          - Landing gear retraction triggered once
 * 
 *          Altitude Frames:
 *          - Takeoff can be terrain-relative or absolute
 *          - auto_takeoff handles frame conversions
 *          - Completion detected in vehicle frame
 *          - See do_takeoff() for frame setup
 * 
 *          Safety Considerations:
 *          - Don't advance mission until stable at altitude
 *          - Ensure clearance before horizontal movement
 *          - Landing gear retracted safely above ground
 *          - Sufficient altitude for any failsafes
 * 
 * @return true if takeoff complete and mission can advance
 * @return false if still climbing to target altitude
 * 
 * @note Called by verify_command() for MAV_CMD_NAV_TAKEOFF/NAV_VTOL_TAKEOFF
 * @note Landing gear retraction only if AP_LANDINGGEAR_ENABLED
 * @note Completion criteria defined by auto_takeoff controller
 * 
 * @see do_takeoff() for takeoff initialization
 * @see takeoff_run() for climb control execution
 * @see auto_takeoff.complete for completion flag
 */
bool ModeAuto::verify_takeoff()
{
#if AP_LANDINGGEAR_ENABLED
    // if we have reached our destination
    if (auto_takeoff.complete) {
        // retract the landing gear
        copter.landinggear.retract_after_takeoff();
    }
#endif

    return auto_takeoff.complete;
}

/**
 * @brief Verify NAV_LAND mission command completion
 * 
 * @details This function verifies the completion of a MAV_CMD_NAV_LAND or MAV_CMD_NAV_VTOL_LAND
 *          command during Auto mode mission execution. The landing process follows a two-state
 *          sequence:
 * 
 *          State 1 - FlyToLocation: If the land command includes non-zero lat/lon coordinates,
 *          the vehicle first flies to that location at current altitude. Once reached, transitions
 *          to Descending state and initializes the landing controller.
 * 
 *          State 2 - Descending: Executes the actual landing descent. Landing is considered
 *          complete when both land_complete flag is set AND motors have spooled down to
 *          GROUND_IDLE state. This ensures the vehicle has touched down and settled.
 * 
 *          Mission Continuation Behavior: After landing, if mission.continue_after_land_check_for_takeoff()
 *          returns false and motors are still armed, the vehicle will automatically disarm and
 *          return false to keep the mission state machine on the current NAV_LAND item. This
 *          prevents unintended mission progression after landing. After disarm, the mission resets.
 * 
 *          Integration: Works in conjunction with land_run() which executes the landing control
 *          loop, and do_land() which initializes the landing sequence.
 * 
 * @return true if landing has been completed successfully (vehicle on ground, motors idle)
 * @return false if landing is still in progress or if vehicle was disarmed to stop mission progression
 * 
 * @note This is called at 10Hz+ by the mission library verify_command() function
 * @note The verify function only checks completion status; actual landing control is handled by land_run()
 * @note For precision landing, the land_complete flag is set by the precision landing state machine
 * 
 * @see do_land() - Initializes the landing sequence
 * @see land_run() - Executes landing control loop (called at 100Hz+)
 * @see land_start() - Initializes landing controller parameters
 */
bool ModeAuto::verify_land()
{
    bool retval = false;

    switch (state) {
        case State::FlyToLocation:
            // check if we've reached the location
            if (copter.wp_nav->reached_wp_destination()) {
                // initialise landing controller
                land_start();

                // advance to next state
                state = State::Descending;
            }
            break;

        case State::Descending:
            // rely on THROTTLE_LAND mode to correctly update landing status
            retval = copter.ap.land_complete && (motors->get_spool_state() == AP_Motors::SpoolState::GROUND_IDLE);
            if (retval && !mission.continue_after_land_check_for_takeoff() && copter.motors->armed()) {
                /*
                  we want to stop mission processing on land
                  completion. Disarm now, then return false. This
                  leaves mission state machine in the current NAV_LAND
                  mission item. After disarming the mission will reset
                */
                copter.arming.disarm(AP_Arming::Method::LANDED);
                retval = false;
            }
            break;

        default:
            // this should never happen
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
            retval = true;
            break;
    }

    // true is returned if we've successfully landed
    return retval;
}

#if AC_PAYLOAD_PLACE_ENABLED
// verify_payload_place - returns true if placing has been completed
bool PayloadPlace::verify()
{
    switch (state) {
    case State::FlyToLocation:
    case State::Descent_Start:
    case State::Descent:
    case State::Release:
    case State::Releasing:
    case State::Delay:
    case State::Ascent_Start:
    case State::Ascent:
        return false;
    case State::Done:
        return true;
    }
    // should never get here
    return true;
}
#endif

/**
 * @brief Verify NAV_LOITER_UNLIM mission command completion
 * 
 * @details This function verifies the MAV_CMD_NAV_LOITER_UNLIM command, which commands
 *          the vehicle to loiter at a specified location indefinitely. By design, this
 *          command never completes on its own - it will continue loitering until the
 *          mission is manually advanced, a mode change occurs, or the mission is stopped.
 * 
 *          This is used in mission plans where the vehicle should hold position at a
 *          waypoint and wait for external intervention (pilot input, ground control
 *          station command, or scripting command).
 * 
 *          The loiter position is established by do_loiter_unlimited() which initializes
 *          the waypoint navigation controller with the target location. The actual loiter
 *          control is executed by loiter_run() at 100Hz+.
 * 
 * @return Always returns false, indicating the command never completes autonomously
 * 
 * @note This is called at 10Hz+ by the mission library verify_command() function
 * @note Mission progression must be manually triggered (mode change, mission skip, etc.)
 * @note Vehicle will maintain position using waypoint controller horizontal and vertical control
 * 
 * @see do_loiter_unlimited() - Initializes unlimited loiter at specified location
 * @see loiter_run() - Executes loiter control loop (called at 100Hz+)
 * @see verify_loiter_time() - Similar but with a time limit for completion
 */
bool ModeAuto::verify_loiter_unlimited()
{
    return false;
}

/**
 * @brief Verify NAV_LOITER_TIME mission command completion
 * 
 * @details This function verifies the MAV_CMD_NAV_LOITER_TIME command, which commands
 *          the vehicle to loiter at a specified location for a defined duration. The
 *          verification process follows a two-phase sequence:
 * 
 *          Phase 1 - Transit to Location: Vehicle navigates to the loiter waypoint.
 *          During this phase, wp_nav->reached_wp_destination() returns false, and the
 *          timer has not started. Returns false to continue mission execution.
 * 
 *          Phase 2 - Timed Loiter: Once the destination is reached, a timer starts
 *          (loiter_time set to current milliseconds). Vehicle holds position while
 *          timer counts up. Command completes when elapsed time reaches loiter_time_max
 *          (specified in mission command p1 parameter in seconds).
 * 
 *          The loiter timer only starts after the vehicle has reached the waypoint,
 *          ensuring the full loiter duration is spent at the target location rather
 *          than including transit time.
 * 
 * @param[in] cmd Mission command containing loiter duration in p1 field (seconds)
 * 
 * @return true if vehicle has reached waypoint AND loitered for specified duration
 * @return false if still transiting to waypoint or loiter time not yet elapsed
 * 
 * @note This is called at 10Hz+ by the mission library verify_command() function
 * @note Loiter duration is set by do_loiter_time() from cmd.p1 parameter
 * @note Timer resolution is in seconds, converted from milliseconds
 * @note GCS notification sent when command completes
 * 
 * @see do_loiter_time() - Initializes timed loiter at specified location
 * @see loiter_run() - Executes loiter control loop (called at 100Hz+)
 * @see verify_loiter_unlimited() - Similar but without time limit
 */
bool ModeAuto::verify_loiter_time(const AP_Mission::Mission_Command& cmd)
{
    // return immediately if we haven't reached our destination
    if (!copter.wp_nav->reached_wp_destination()) {
        return false;
    }

    // start our loiter timer
    if ( loiter_time == 0 ) {
        loiter_time = millis();
    }

    // check if loiter timer has run out
    if (((millis() - loiter_time) / 1000) >= loiter_time_max) {
        gcs().send_text(MAV_SEVERITY_INFO, "Reached command #%i",cmd.index);
        return true;
    }

    return false;
}

/**
 * @brief Verify NAV_LOITER_TO_ALT mission command completion
 * 
 * @details This function verifies the MAV_CMD_NAV_LOITER_TO_ALT command, which commands
 *          the vehicle to navigate to a specified location and then climb or descend to
 *          a target altitude while loitering. The command requires two conditions to be
 *          met for completion:
 * 
 *          Condition 1 - Horizontal Position: Vehicle must reach the target waypoint in
 *          the horizontal plane (NE coordinates). This is tracked by loiter_to_alt.reached_destination_xy
 *          flag, set when wp_nav->reached_wp_destination_NE() returns true.
 * 
 *          Condition 2 - Vertical Position: Vehicle must reach the commanded altitude.
 *          This is tracked by loiter_to_alt.reached_alt flag, set when altitude error
 *          falls below threshold (5cm) or when vehicle crosses the target altitude
 *          (detected by altitude error sign change). The altitude tracking uses a sqrt
 *          controller to smoothly approach the target.
 * 
 *          Both conditions must be satisfied simultaneously for command completion. The
 *          vehicle first flies to the waypoint, then transitions to loiter mode while
 *          climbing/descending to the target altitude. This allows altitude changes at
 *          a specific location rather than during transit.
 * 
 * @return true if vehicle has reached target location (XY) AND target altitude (Z)
 * @return false if either horizontal or vertical target not yet achieved
 * 
 * @note This is called at 10Hz+ by the mission library verify_command() function
 * @note Altitude target is specified in mission command and converted to ABOVE_HOME frame
 * @note Uses separate flags for XY and Z achievement to handle decoupled horizontal/vertical control
 * @note const function - does not modify object state
 * 
 * @see do_loiter_to_alt() - Initializes loiter-to-altitude sequence
 * @see loiter_to_alt_run() - Executes loiter-to-alt control loop (called at 100Hz+)
 */
bool ModeAuto::verify_loiter_to_alt() const
{
    if (loiter_to_alt.reached_destination_xy &&
        loiter_to_alt.reached_alt) {
        return true;
    }
    return false;
}

/**
 * @brief Verify NAV_RETURN_TO_LAUNCH mission command completion
 * 
 * @details This function verifies the MAV_CMD_NAV_RETURN_TO_LAUNCH command executed within
 *          an Auto mode mission. RTL within Auto delegates to the dedicated ModeRTL
 *          flight mode for the actual return-to-launch logic, while this verify function
 *          monitors its progress to determine mission command completion.
 * 
 *          The RTL command is considered complete when ALL of the following conditions
 *          are satisfied:
 * 
 *          1. RTL State Machine Complete: copter.mode_rtl.state_complete() returns true,
 *             indicating the RTL state machine has finished its current state.
 * 
 *          2. Landing Phase Active: The RTL state must be either FINAL_DESCENT or LAND,
 *             ensuring the vehicle has completed horizontal navigation and is in the
 *             vertical landing sequence.
 * 
 *          3. Motors at Ground Idle: motors->get_spool_state() == GROUND_IDLE confirms
 *             the vehicle has fully landed and motors have spooled down, preventing
 *             premature mission continuation while still airborne.
 * 
 *          This multi-condition check ensures the vehicle has completed the full RTL
 *          sequence (climb, return, descend, land) before advancing to the next mission
 *          command.
 * 
 * @return true if RTL has fully completed including landing and motor spool down
 * @return false if RTL still in progress (any phase: climb, return, descend, land)
 * 
 * @note This is called at 10Hz+ by the mission library verify_command() function
 * @note do_RTL() must be called first via rtl_start() to initialize the RTL mode
 * @note Delegates actual RTL control to ModeRTL flight mode
 * @note RTL behavior (altitude, return path) controlled by RTL mode parameters
 * 
 * @see do_RTL() - Initializes RTL within Auto mode mission
 * @see rtl_run() - Executes RTL control by calling ModeRTL::run()
 * @see ModeRTL - Dedicated RTL flight mode handling all RTL logic
 */
bool ModeAuto::verify_RTL()
{
    return (copter.mode_rtl.state_complete() && 
            (copter.mode_rtl.state() == ModeRTL::SubMode::FINAL_DESCENT || copter.mode_rtl.state() == ModeRTL::SubMode::LAND) &&
            (motors->get_spool_state() == AP_Motors::SpoolState::GROUND_IDLE));
}

/********************************************************************************/
// Verify Condition (May) commands
/********************************************************************************/

/**
 * @brief Verify CONDITION_DELAY command completion
 * 
 * @details This function verifies the MAV_CMD_CONDITION_DELAY conditional command, which
 *          pauses mission progression for a specified duration. Unlike navigation commands,
 *          conditional commands don't control vehicle movement - they gate the execution
 *          of subsequent DO commands until the condition is satisfied.
 * 
 *          The delay timer starts when do_wait_delay() is called, storing the start time
 *          in condition_start (milliseconds). The delay duration is stored in condition_value
 *          (milliseconds). This verify function checks if sufficient time has elapsed.
 * 
 *          Elapsed Time Calculation: (millis() - condition_start) computes milliseconds
 *          since delay started. When this exceeds condition_value, the delay completes.
 * 
 *          The MAX(condition_value, 0) guards against negative values, ensuring timer
 *          completion if an invalid delay was specified. Upon completion, condition_value
 *          is reset to 0 for the next conditional command.
 * 
 * @return true if delay duration has elapsed
 * @return false if still waiting for delay to complete
 * 
 * @note This is called at 10Hz+ by the mission library verify_command() function
 * @note Delay duration set by do_wait_delay() from mission command seconds parameter
 * @note Conditional commands don't affect vehicle control, only mission flow logic
 * @note Timer uses millis() which wraps every ~49.7 days (handled by unsigned arithmetic)
 * 
 * @see do_wait_delay() - Initializes delay timer with duration from mission command
 */
bool ModeAuto::verify_wait_delay()
{
    if (millis() - condition_start > (uint32_t)MAX(condition_value,0)) {
        condition_value = 0;
        return true;
    }
    return false;
}

/**
 * @brief Verify CONDITION_DISTANCE command completion
 * 
 * @details This function verifies the MAV_CMD_CONDITION_DISTANCE conditional command, which
 *          gates mission progression until the vehicle reaches within a specified distance
 *          of the current navigation target waypoint. This allows DO commands to be triggered
 *          at specific approach distances rather than only at waypoint arrival.
 * 
 *          Distance Threshold: condition_value stores the distance threshold in meters,
 *          set by do_within_distance() from the mission command. The command completes
 *          when the vehicle comes within this distance of the active navigation target.
 * 
 *          Distance Measurement: wp_distance_m() returns current distance to target in
 *          meters. This function varies by active sub-mode:
 *          - WP/CIRCLE_MOVE_TO_EDGE: Distance to waypoint destination
 *          - CIRCLE: Distance to circle target point
 *          - Other modes: Appropriate distance calculation
 * 
 *          The MAX(condition_value, 0) guards against negative distances, ensuring immediate
 *          completion if an invalid distance was specified. Upon completion, condition_value
 *          is reset to 0 for the next conditional command.
 * 
 *          Example Use Case: Trigger camera 50m before reaching waypoint using
 *          CONDITION_DISTANCE followed by DO_DIGICAM_CONTROL.
 * 
 * @return true if vehicle is within specified distance of navigation target
 * @return false if still approaching target and outside threshold distance
 * 
 * @note This is called at 10Hz+ by the mission library verify_command() function
 * @note Distance threshold set by do_within_distance() from mission command meters parameter
 * @note Distance measured horizontally in meters (does not include altitude difference)
 * @note Conditional commands enable precise event triggering during waypoint approaches
 * 
 * @see do_within_distance() - Initializes distance threshold from mission command
 * @see wp_distance_m() - Returns current distance to navigation target
 */
bool ModeAuto::verify_within_distance()
{
    if (wp_distance_m() < MAX(condition_value,0)) {
        condition_value = 0;
        return true;
    }
    return false;
}

/**
 * @brief Verify CONDITION_YAW command completion
 * 
 * @details This function verifies the MAV_CMD_CONDITION_YAW conditional command, which
 *          gates mission progression until the vehicle reaches a specified heading. This
 *          allows precise yaw control independent of navigation waypoints, enabling DO
 *          commands to trigger at specific orientations.
 * 
 *          Yaw Target Management: The target heading is set by do_yaw() which configures:
 *          - Absolute or relative heading angle (degrees)
 *          - Turn direction (shortest, clockwise, counter-clockwise)
 *          - Turn rate (deg/s) for controlled rotation speed
 * 
 *          Mode Enforcement: The waypoint controller often overrides yaw control when
 *          executing navigation commands (pointing toward next waypoint). This verify
 *          function re-asserts FIXED yaw mode on each call to maintain heading control
 *          until the conditional completes, preventing waypoint navigation from interfering.
 * 
 *          Completion Check: reached_fixed_yaw_target() in auto_yaw returns true when:
 *          - Current heading is within tolerance of target (typically 2-5 degrees)
 *          - Yaw rate has settled (vehicle not still rotating significantly)
 * 
 *          Example Use Case: Point camera at 90 degrees before taking photo using
 *          CONDITION_YAW followed by DO_DIGICAM_CONTROL.
 * 
 * @return true if vehicle has reached target heading within tolerance
 * @return false if still rotating toward target heading
 * 
 * @note This is called at 10Hz+ by the mission library verify_command() function
 * @note Yaw target and rotation parameters set by do_yaw() from mission command
 * @note Repeatedly re-enables FIXED yaw mode to prevent navigation commands from overriding
 * @note Heading tolerance typically 2-5 degrees (configured in auto_yaw controller)
 * @note Turn rate limiting prevents aggressive yaw changes that could destabilize vehicle
 * 
 * @see do_yaw() - Sets target heading, turn rate, and direction from mission command
 * @see AutoYaw::Mode::FIXED - Maintains fixed heading regardless of navigation
 */
bool ModeAuto::verify_yaw()
{
    // make sure still in fixed yaw mode, the waypoint controller often retakes control of yaw as it executes a new waypoint command
    auto_yaw.set_mode(AutoYaw::Mode::FIXED);

    // check if we have reached the target heading
    return auto_yaw.reached_fixed_yaw_target();
}

/**
 * @brief Verify NAV_WAYPOINT command completion
 * 
 * @details This function verifies the MAV_CMD_NAV_WAYPOINT navigation command, which is
 *          the fundamental waypoint navigation command in Auto mode. It checks both position
 *          arrival at the waypoint and optional delay completion before advancing to the
 *          next mission command.
 * 
 *          Two-Phase Completion:
 *          1. Position Arrival: reached_wp_destination() returns true when vehicle is within
 *             acceptance radius of target waypoint (typically 2m horizontally, 1m vertically)
 *          2. Delay Completion: If mission command specifies delay (cmd.p1 > 0), vehicle
 *             loiters at waypoint for specified seconds before proceeding
 * 
 *          Delay Timer Management:
 *          - loiter_time initialized to 0 by do_nav_wp()
 *          - On first arrival, loiter_time set to current time (millis())
 *          - Timer checked against loiter_time_max (delay duration in seconds from cmd.p1)
 *          - Zero delay (default) causes immediate completion on arrival
 * 
 *          Waypoint Reached Notification:
 *          - If delay specified: Tone plays immediately on arrival, mission completes after delay
 *          - If no delay: Tone plays when function returns true
 *          - GCS text message sent with waypoint index when command completes
 * 
 *          Continuous Loiter: While waiting for delay, vehicle actively holds position at
 *          waypoint using wp_nav position controller. This differs from dedicated loiter
 *          commands which provide more sophisticated loiter control.
 * 
 * @param[in] cmd Current mission command being verified (contains index for reporting)
 * 
 * @return true if waypoint reached and any specified delay has elapsed
 * @return false if still navigating to waypoint or waiting for delay completion
 * 
 * @note This is called at 10Hz+ by the mission library verify_command() function
 * @note Waypoint position and acceptance radius configured by do_nav_wp() and wp_nav
 * @note Delay duration (loiter_time_max) set from mission command param1 (seconds)
 * @note Zero delay is default behavior for continuous waypoint-to-waypoint navigation
 * @note Vehicle continues to run wp_run() during delay, maintaining position hold
 * @note Waypoint index reported to GCS for mission progress tracking
 * 
 * @see do_nav_wp() - Initializes waypoint navigation and delay timer
 * @see wp_run() - Executes position hold during waypoint delay
 * @see AC_WPNav::reached_wp_destination() - Checks position arrival within acceptance radius
 */
bool ModeAuto::verify_nav_wp(const AP_Mission::Mission_Command& cmd)
{
    // check if we have reached the waypoint
    if ( !copter.wp_nav->reached_wp_destination() ) {
        return false;
    }

    // start timer if necessary
    if (loiter_time == 0) {
        loiter_time = millis();
        if (loiter_time_max > 0) {
            // play a tone
            AP_Notify::events.waypoint_complete = 1;
        }
    }

    // check if timer has run out
    if (((millis() - loiter_time) / 1000) >= loiter_time_max) {
        if (loiter_time_max == 0) {
            // play a tone
            AP_Notify::events.waypoint_complete = 1;
        }
        gcs().send_text(MAV_SEVERITY_INFO, "Reached command #%i",cmd.index);
        return true;
    }
    return false;
}

/**
 * @brief Verify NAV_LOITER_TURNS (circle) command completion
 * 
 * @details This function verifies the MAV_CMD_NAV_LOITER_TURNS navigation command, which
 *          commands the vehicle to fly circles around a specified center point for a given
 *          number of turns. This is used for orbiting points of interest, creating circular
 *          flight patterns, or establishing holding patterns.
 * 
 *          Two-Phase Execution:
 *          1. Move to Circle Edge (SubMode::CIRCLE_MOVE_TO_EDGE): If vehicle is more than
 *             3m from circle edge when command starts, it first flies direct to the edge
 *             using waypoint navigation (wp_run()). Verification waits in this phase.
 *          2. Circle Execution (SubMode::CIRCLE): Once at edge, circle_start() activates
 *             circle_nav controller which flies smooth circular path at specified radius.
 * 
 *          Circle Progress Tracking:
 *          - circle_nav->get_angle_total_rad() accumulates total rotation angle (radians)
 *          - Total angle divided by 2π gives number of complete circles
 *          - Absolute value used because circles can be clockwise or counter-clockwise
 *          - Progress messages sent to GCS each time integer circle count increments
 * 
 *          Circle Parameters (from mission command):
 *          - Center point: cmd.content.location (lat, lon, alt)
 *          - Radius: HIGHBYTE(cmd.p1) in meters
 *          - Turns: cmd.get_loiter_turns() - number of complete circles required
 *          - Direction: cmd.content.location.loiter_ccw (counter-clockwise if true)
 * 
 *          Completion: Command completes when vehicle has flown the specified number of
 *          complete 360-degree rotations around the center point. Partial final turn
 *          counts toward completion (>= comparison used).
 * 
 * @param[in] cmd Current mission command being verified (contains turn count and index)
 * 
 * @return true if vehicle has completed specified number of circles
 * @return false if still moving to edge or circling
 * 
 * @note This is called at 10Hz+ by the mission library verify_command() function
 * @note Circle parameters set by do_circle() which calls circle_movetoedge_start()
 * @note Progress messages sent to GCS each time a new circle begins
 * @note circle_last_num_complete tracks last reported circle to prevent duplicate messages
 * @note Circle direction (CW/CCW) affects yaw behavior but not completion tracking
 * @note Fractional turns supported (e.g., 2.5 turns flies 2.5 complete circles)
 * 
 * @see do_circle() - Initializes circle center, radius, direction, and turn count
 * @see circle_movetoedge_start() - Initiates movement to circle edge if necessary
 * @see circle_start() - Activates circle_nav controller for circular flight
 * @see circle_run() - Executes circular flight control (called at 100Hz+)
 */
bool ModeAuto::verify_circle(const AP_Mission::Mission_Command& cmd)
{
    // check if we've reached the edge
    if (_mode == SubMode::CIRCLE_MOVE_TO_EDGE) {
        if (copter.wp_nav->reached_wp_destination()) {
            // start circling
            circle_start();
        }
        return false;
    }

    const float turns = cmd.get_loiter_turns();

    const auto num_circles_completed = fabsf(copter.circle_nav->get_angle_total_rad()/float(M_2PI));
    if (int(num_circles_completed) != int(circle_last_num_complete)) {
        circle_last_num_complete = num_circles_completed;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Mission: starting circle %u/%u", unsigned(num_circles_completed)+1, unsigned(turns));
    }

    // check if we have completed circling
    return num_circles_completed >= turns;
}

/**
 * @brief Verify NAV_SPLINE_WAYPOINT command completion
 * 
 * @details This function verifies the MAV_CMD_NAV_SPLINE_WAYPOINT navigation command, which
 *          commands the vehicle to fly smooth spline curves through waypoints rather than
 *          straight line segments. Spline waypoints create curved flight paths that are
 *          more natural and efficient, reducing sharp corners and enabling higher speeds.
 * 
 *          Spline Path Generation:
 *          - do_spline_wp() initializes spline using current waypoint and next waypoint
 *          - AC_WPNav::set_spline_destination_loc() generates smooth Hermite spline curve
 *          - Curve passes through waypoint with tangent determined by adjacent waypoints
 *          - Multiple consecutive spline waypoints create continuous smooth path
 * 
 *          Verification Behavior (identical to regular waypoints):
 *          1. Position Arrival: reached_wp_destination() checks if within acceptance radius
 *          2. Delay Completion: If cmd.p1 > 0, vehicle loiters for specified seconds
 *          3. Command Complete: Returns true when both position and delay satisfied
 * 
 *          Spline vs Straight Waypoints:
 *          - Spline: Curved approach, tangent-continuous path, higher cruise speeds possible
 *          - Straight: Direct line, sharp corners, may require slowdown for corner angles
 *          - Verification logic identical - only path generation differs
 * 
 *          Delay Timer Management (same as regular waypoints):
 *          - loiter_time initialized to 0 by do_spline_wp()
 *          - Timer starts (millis()) on first arrival at waypoint
 *          - Delay duration stored in loiter_time_max from cmd.p1 (seconds)
 *          - Zero delay causes immediate completion on arrival
 * 
 * @param[in] cmd Current mission command being verified (contains index for reporting)
 * 
 * @return true if waypoint reached and any specified delay has elapsed
 * @return false if still navigating to waypoint or waiting for delay completion
 * 
 * @note This is called at 10Hz+ by the mission library verify_command() function
 * @note Spline path calculated by do_spline_wp() using get_spline_from_cmd()
 * @note Smooth curves reduce vehicle jerk and allow higher navigation speeds
 * @note Spline quality depends on waypoint spacing and adjacent waypoint positions
 * @note If next waypoint is not spline type, curve transitions to straight line
 * @note Delay behavior identical to NAV_WAYPOINT - vehicle holds position at waypoint
 * 
 * @see do_spline_wp() - Initializes spline path and delay timer
 * @see get_spline_from_cmd() - Extracts current and next waypoint for curve generation
 * @see AC_WPNav::set_spline_destination_loc() - Generates Hermite spline curve
 * @see verify_nav_wp() - Equivalent verification for straight-line waypoints
 */
bool ModeAuto::verify_spline_wp(const AP_Mission::Mission_Command& cmd)
{
    // check if we have reached the waypoint
    if ( !copter.wp_nav->reached_wp_destination() ) {
        return false;
    }

    // start timer if necessary
    if (loiter_time == 0) {
        loiter_time = millis();
    }

    // check if timer has run out
    if (((millis() - loiter_time) / 1000) >= loiter_time_max) {
        gcs().send_text(MAV_SEVERITY_INFO, "Reached command #%i",cmd.index);
        return true;
    }
    return false;
}

#if AC_NAV_GUIDED
/**
 * @brief Verify NAV_GUIDED_ENABLE command completion
 * 
 * @details This function verifies the MAV_CMD_NAV_GUIDED_ENABLE navigation command, which
 *          enables or disables external navigation control within Auto mode. When enabled,
 *          this allows companion computers or ground control stations to temporarily take
 *          control of navigation via MAVLink GUIDED messages while remaining in Auto mode.
 * 
 *          Two Operational Modes:
 *          1. Enable Guided (cmd.p1 = 1): Activates SubMode::NAVGUIDED, vehicle accepts
 *             external position/velocity targets via MAVLink. Command verification waits
 *             for time or position limits to be reached (see DO_GUIDED_LIMITS command).
 *          2. Disable Guided (cmd.p1 = 0): Immediately completes and returns to normal
 *             Auto mission execution. Used to return control to mission script.
 * 
 *          Guided Limits Enforcement (when enabled):
 *          - Time Limit: Maximum duration (seconds) for external control
 *          - Altitude Limits: Min/max altitude bounds (meters)
 *          - Horizontal Limit: Maximum horizontal distance from entry point (meters)
 *          - Limits configured by preceding DO_GUIDED_LIMITS command
 *          - If any limit breached, verification returns true and mission advances
 * 
 *          Integration with Companion Computers:
 *          - Enables advanced path planning or obstacle avoidance off-board
 *          - Companion can update target position/velocity at high rate via MAVLink
 *          - Mission maintains overall sequence control and safety limits
 *          - Automatic fallback to next mission command if limits exceeded
 * 
 *          Safety Behavior:
 *          - If companion loses connection, time limit ensures mission progression
 *          - Position limits prevent vehicle from straying too far from mission path
 *          - Altitude limits enforce operational envelope during external control
 *          - Mission always retains ultimate authority via limit enforcement
 * 
 * @param[in] cmd Current mission command being verified (p1: 0=disable, 1=enable)
 * 
 * @return true if guided mode disabled (cmd.p1=0) or limits breached while enabled
 * @return false if guided mode active and within all configured limits
 * 
 * @note This is called at 10Hz+ by the mission library verify_command() function
 * @note External control targets sent via MAVLink SET_POSITION_TARGET_* messages
 * @note Limits set by preceding DO_GUIDED_LIMITS mission command
 * @note If no limits set, guided mode continues indefinitely until disabled
 * @note do_nav_guided_enable() initializes guided mode and limit tracking
 * 
 * @see do_nav_guided_enable() - Initializes NAV_GUIDED submode and limit tracking
 * @see do_guided_limits() - Configures time, altitude, and horizontal limits
 * @see nav_guided_run() - Executes external navigation control (calls mode_guided.run())
 * @see ModeGuided::limit_check() - Checks if any configured limit has been breached
 */
bool ModeAuto::verify_nav_guided_enable(const AP_Mission::Mission_Command& cmd)
{
    // if disabling guided mode then immediately return true so we move to next command
    if (cmd.p1 == 0) {
        return true;
    }

    // check time and position limits
    return copter.mode_guided.limit_check();
}
#endif  // AC_NAV_GUIDED

/**
 * @brief Verify NAV_DELAY command completion
 * 
 * @details This function verifies the MAV_CMD_NAV_DELAY navigation command, which delays
 *          the execution of subsequent navigation commands by a specified duration. The
 *          vehicle holds its current position (loiter) during the delay period.
 * 
 *          Two Delay Modes:
 *          1. Relative Delay (cmd.content.nav_delay.seconds > 0): Wait for specified
 *             number of seconds from when delay command started executing
 *          2. Absolute Delay (cmd.content.nav_delay.seconds = 0): Wait until specific
 *             UTC time reached (hour, minute, second specified in command parameters)
 * 
 *          Relative Delay Operation:
 *          - do_nav_delay() sets nav_delay_time_start_ms = millis() when command starts
 *          - nav_delay_time_max_ms = seconds * 1000 (convert to milliseconds)
 *          - Verification checks if elapsed time exceeds specified duration
 *          - Vehicle loiters at current position via auto_loiter submode
 * 
 *          Absolute Delay Operation (requires RTC):
 *          - do_nav_delay() calculates target UTC time from hour:min:sec parameters
 *          - nav_delay_time_max_ms set to absolute UTC timestamp (milliseconds)
 *          - Verification waits until system UTC time reaches target time
 *          - Useful for coordinated multi-vehicle operations or time-synchronized events
 * 
 *          Use Cases:
 *          - Mission sequencing: Allow other vehicles time to complete their tasks
 *          - Observation missions: Loiter at waypoint collecting data for fixed duration
 *          - Coordinated operations: Start next phase at specific UTC time
 *          - Camera operations: Wait for camera to finish capturing panorama
 * 
 *          Delay Timer Cleanup:
 *          - nav_delay_time_max_ms reset to 0 when delay completes
 *          - Prevents delay state from affecting subsequent mission commands
 *          - Allows reuse of nav_delay variables for multiple delay commands in mission
 * 
 * @param[in] cmd Current mission command being verified (unused, delay uses state variables)
 * 
 * @return true if delay period has elapsed (relative or absolute time reached)
 * @return false if still waiting for delay completion
 * 
 * @note This is called at 10Hz+ by the mission library verify_command() function
 * @note During delay, vehicle loiters at position where delay command started
 * @note Absolute delay requires RTC (Real-Time Clock) support enabled (AP_RTC_ENABLED)
 * @note If RTC not enabled, absolute delays silently fail (nav_delay_time_max_ms = 0)
 * @note Delay does not account for GPS week rollover in absolute time calculations
 * 
 * @see do_nav_delay() - Initializes delay timer and determines relative vs absolute mode
 * @see loiter_run() - Maintains position hold during delay period
 */
bool ModeAuto::verify_nav_delay(const AP_Mission::Mission_Command& cmd)
{
    if (millis() - nav_delay_time_start_ms > nav_delay_time_max_ms) {
        nav_delay_time_max_ms = 0;
        return true;
    }
    return false;
}

#if AP_SCRIPTING_ENABLED
/**
 * @brief Verify NAV_SCRIPT_TIME command completion
 * 
 * @details This function verifies the MAV_CMD_NAV_SCRIPT_TIME navigation command, which
 *          hands over vehicle navigation control to a Lua script for a specified duration.
 *          This enables sophisticated autonomous behaviors implemented in user scripts
 *          while maintaining mission-level safety and sequencing.
 * 
 *          Script Control Mechanism:
 *          - Mission command contains: command ID, timeout, and 4 custom arguments
 *          - do_nav_script_time() initializes guided mode and populates nav_scripting struct
 *          - Lua script retrieves command parameters via nav_script_time() method
 *          - Script sends position/velocity/acceleration targets via guided mode interface
 *          - Script calls nav_script_time_done(id) when behavior complete
 * 
 *          Completion Conditions (either triggers command completion):
 *          1. Script Completion: Lua script calls nav_script_time_done(id) with matching ID
 *             - Sets nav_scripting.done = true
 *             - Script explicitly signals it has achieved its objective
 *             - Normal completion path for successful script execution
 * 
 *          2. Timeout: Specified timeout period expires without script completion
 *             - timeout_s configured in mission command (0 = no timeout)
 *             - Elapsed time calculated: (millis() - start_ms) > (timeout_s * 1000)
 *             - Safety mechanism prevents mission from stalling if script fails
 *             - Allows mission to continue even if script malfunctions
 * 
 *          Script Parameter Passing:
 *          - nav_scripting.command: User-defined command identifier (0-255)
 *          - nav_scripting.arg1, arg2: Float arguments for script logic
 *          - nav_scripting.arg3, arg4: Integer arguments for script logic
 *          - nav_scripting.id: Unique incrementing ID to prevent race conditions
 * 
 *          Safety Guarantees:
 *          - Timeout ensures mission progression even if script hangs or fails
 *          - ID matching prevents old script callbacks from affecting new commands
 *          - Mission retains ultimate control via timeout mechanism
 *          - Guided mode limits can still be enforced during script execution
 * 
 *          Use Cases:
 *          - Complex inspection patterns (orbits, spirals, custom paths)
 *          - Dynamic obstacle avoidance with real-time path replanning
 *          - Precision landing with visual servoing
 *          - Custom survey patterns based on terrain or mission requirements
 *          - Cooperative multi-vehicle behaviors with inter-vehicle communication
 * 
 * @return true if script signaled completion (done=true) or timeout expired
 * @return false if script still executing and within timeout period
 * 
 * @note This is called at 10Hz+ by the mission library verify_command() function
 * @note Script executes in SubMode::NAV_SCRIPT_TIME, using guided mode infrastructure
 * @note Zero timeout (timeout_s=0) means script must explicitly signal completion
 * @note Script failures without timeout will stall mission (design carefully)
 * @note Unique ID prevents race conditions if mission restarts same command
 * 
 * @see do_nav_script_time() - Initializes script control and populates parameters
 * @see nav_script_time() - Lua script calls this to retrieve command parameters
 * @see nav_script_time_done() - Lua script calls this to signal completion
 * @see nav_guided_run() - Executes guided mode control while script is active
 */
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

/**
 * @brief Verify NAV_ATTITUDE_TIME command completion
 * 
 * @details This function verifies the MAV_CMD_NAV_ATTITUDE_TIME navigation command, which
 *          commands the vehicle to maintain a specified attitude (roll, pitch, yaw) and
 *          climb rate for a fixed duration. This is a time-based, open-loop control mode
 *          that does not require GPS or position estimation.
 * 
 *          Command Operation:
 *          - Vehicle holds fixed roll, pitch, yaw angles specified in command
 *          - Maintains constant climb rate (m/s) during entire duration
 *          - Attitude angles limited by lean angle limits for safety
 *          - Time duration specified in command (seconds)
 * 
 *          GPS-Independent Operation:
 *          - Does NOT require position estimate or GPS lock (requires_GPS() returns false)
 *          - Useful for GPS-denied environments or GPS failure scenarios
 *          - Allows basic aircraft control when navigation systems unavailable
 *          - EKF failsafe NOT triggered during attitude_time execution
 * 
 *          Completion Criterion:
 *          - Simple time-based: Elapsed time > specified duration
 *          - do_nav_attitude_time() sets nav_attitude_time.start_ms = millis()
 *          - Verification checks: (current_time - start_time) > (duration * 1000ms)
 *          - No position or velocity targets - purely time-driven completion
 * 
 *          Safety Limits Applied:
 *          - Roll/pitch angles clamped to attitude_control lean_angle_max
 *          - Climb rate limited by position controller speed limits (max_speed_up/down)
 *          - Minimum 10 degree lean angle limit enforced
 *          - Yaw rate unlimited (follows commanded yaw angle)
 * 
 *          Submode Behavior (SubMode::NAV_ATTITUDE_TIME):
 *          - nav_attitude_time_run() sends attitude targets to attitude controller
 *          - Position controller maintains vertical rate control for climb
 *          - Horizontal position drifts with wind/momentum (no position hold)
 *          - Vehicle does NOT loiter or maintain GPS position
 * 
 *          Use Cases:
 *          - GPS-denied flight segments (tunnels, under bridges, indoor)
 *          - Emergency control when navigation systems fail
 *          - Testing attitude controllers independently of position control
 *          - Aerobatic maneuvers with fixed attitudes (e.g., sustained bank turns)
 *          - Transition maneuvers where position hold not desired
 * 
 * @param[in] cmd Current mission command containing time_sec duration parameter
 * 
 * @return true if specified duration has elapsed since command started
 * @return false if command still executing (within time duration)
 * 
 * @note This is called at 10Hz+ by the mission library verify_command() function
 * @note Vehicle position will drift during execution - not suitable for precision navigation
 * @note Attitude limits prevent dangerous maneuvers but allow significant lean angles
 * @note This command does NOT trigger EKF failsafe checking (position not required)
 * @note No obstacle avoidance or terrain following during attitude_time execution
 * 
 * @see do_nav_attitude_time() - Initializes attitude and climb rate parameters
 * @see nav_attitude_time_run() - Executes attitude control during command
 * @see requires_GPS() - Returns false for NAV_ATTITUDE_TIME submode
 */
bool ModeAuto::verify_nav_attitude_time(const AP_Mission::Mission_Command& cmd)
{
    return ((AP_HAL::millis() - nav_attitude_time.start_ms) > (cmd.content.nav_attitude_time.time_sec * 1000));
}

/**
 * @brief Pause autonomous mission waypoint navigation
 * 
 * @details This function pauses the vehicle's forward progress toward waypoint destinations
 *          during Auto mode execution. When paused, the vehicle holds its current position
 *          along the path trajectory rather than continuing toward the next waypoint.
 * 
 *          Pause Mechanism:
 *          - Freezes wp_nav trajectory advancement at current position along path
 *          - Vehicle maintains altitude and loiters at pause point in space
 *          - Yaw control continues as configured (ROI, heading control, etc.)
 *          - Mission command sequence does NOT advance (stays on current nav command)
 * 
 *          Pause Conditions and Restrictions:
 *          1. Only Applicable in Waypoint Submode:
 *             - _mode must equal SubMode::WP (straight waypoint navigation)
 *             - Cannot pause in CIRCLE, LAND, RTL, LOITER, or other submodes
 *             - Spline waypoints treated as SubMode::WP, so pause works
 * 
 *          2. Destination Not Yet Reached:
 *             - Pause rejected if wp_nav->reached_wp_destination() returns true
 *             - No point pausing if already at destination (would just loiter)
 *             - Pause intended for mid-trajectory position hold
 * 
 *          Pause vs. Mission Pause:
 *          - This is waypoint trajectory pause (position hold along path)
 *          - Different from mission.stop() which halts command execution
 *          - Mission continues "executing" current nav command, just frozen in space
 *          - Verify functions still called, but position target frozen
 * 
 *          Resume Behavior:
 *          - Call resume() to un-freeze trajectory and continue to waypoint
 *          - Vehicle smoothly continues from paused position to destination
 *          - No discontinuity or jump in trajectory
 * 
 *          External Control Use Cases:
 *          - Ground operator pauses mission to assess situation
 *          - Companion computer pauses for sensor data collection
 *          - Collision avoidance system pauses for obstacle clearance
 *          - Inspection pause to capture high-resolution imagery
 *          - Emergency pause before entering restricted airspace
 * 
 *          Implementation Detail:
 *          - wp_nav->set_pause() freezes position target at current location
 *          - Position controller continues active (maintains current position)
 *          - Attitude controller continues normal operation
 *          - No mode change or submode change occurs
 * 
 * @return true if pause successfully engaged (was in WP submode, not at destination)
 * @return false if pause rejected (wrong submode or already at destination)
 * 
 * @note Can be called via MAVLink SET_POSITION_TARGET_LOCAL_NED or GCS commands
 * @note Pause state persists until resume() called or mode changes
 * @note Mission timer (loiter_time) continues running during pause
 * @note Obstacle avoidance and terrain following remain active during pause
 * 
 * @see resume() - Clears pause state and resumes waypoint navigation
 * @see paused() - Checks if waypoint navigation currently paused
 * @see wp_run() - Continues executing position control even when paused
 */
bool ModeAuto::pause()
{
    // do not pause if not in the WP sub mode or already reached to the destination
    if (_mode != SubMode::WP || wp_nav->reached_wp_destination()) {
        return false;
    }

    wp_nav->set_pause();
    return true;
}

/**
 * @brief Resume autonomous mission waypoint navigation after pause
 * 
 * @details This function resumes waypoint trajectory advancement after a previous pause()
 *          call. The vehicle smoothly continues along its path from the paused position
 *          toward the waypoint destination without discontinuities or trajectory jumps.
 * 
 *          Resume Mechanism:
 *          - Clears pause flag in wp_nav controller
 *          - Trajectory generation resumes from current position along path
 *          - Vehicle accelerates smoothly toward waypoint using configured acceleration limits
 *          - No position jump or velocity discontinuity introduced
 * 
 *          Unconditional Resume:
 *          - Always returns true (no failure conditions)
 *          - Safe to call even if not currently paused (no-op if not paused)
 *          - Safe to call in any Auto submode (only affects wp_nav behavior)
 *          - No validation of current vehicle state required
 * 
 *          Trajectory Continuity:
 *          - wp_nav maintains smooth S-curve trajectory from pause point to destination
 *          - Current velocity and acceleration respected when resuming
 *          - No sudden changes in thrust or attitude commanded
 *          - Position controller smoothly blends from hold to navigation
 * 
 *          Resume After Position Drift:
 *          - If vehicle drifted during pause (wind, manual control), resume handles gracefully
 *          - New trajectory calculated from actual current position to destination
 *          - May result in slightly different path than original if drift occurred
 *          - Speed and acceleration limits prevent aggressive correction
 * 
 *          Mission Command Impact:
 *          - Mission command execution resumes (verify functions continue)
 *          - If loiter timer was running, continues counting
 *          - Waypoint reached checks resume normal operation
 *          - Mission state machine continues from where paused
 * 
 *          Use Cases:
 *          - Ground operator resumes after inspection pause
 *          - Companion computer resumes after obstacle clearance
 *          - Automated resume after timed hold for sensor stabilization
 *          - Resume after manual pilot adjustment of position during pause
 * 
 *          Pause/Resume vs. Mode Changes:
 *          - Pause/resume does NOT change flight mode (stays in Auto)
 *          - Faster and smoother than switching modes
 *          - Mission state preserved through pause/resume cycle
 *          - Preferred method for temporary navigation hold
 * 
 *          Safety Considerations:
 *          - Vehicle must be in stable hover before resume for smooth transition
 *          - Large position errors at resume may cause aggressive maneuvering
 *          - Operator should verify clear path before resuming
 *          - Terrain and obstacle avoidance active during resume trajectory
 * 
 * @return true always (resume command accepted unconditionally)
 * 
 * @note Can be called via MAVLink commands or GCS resume button
 * @note Safe to call multiple times or when not paused (idempotent)
 * @note No mode change or logging event triggered by resume
 * @note Yaw control and altitude control unaffected by resume (already active during pause)
 * 
 * @see pause() - Pauses waypoint navigation at current position along path
 * @see paused() - Checks if waypoint navigation currently paused
 * @see wp_run() - Executes waypoint navigation (with or without pause)
 */
bool ModeAuto::resume()
{
    wp_nav->set_resume();
    return true;
}

/**
 * @brief Check if autonomous waypoint navigation is currently paused
 * 
 * @details This function queries the wp_nav controller to determine if waypoint trajectory
 *          advancement is currently frozen due to a pause() call. Returns the real-time
 *          pause state for mission monitoring and decision-making.
 * 
 *          Pause State Query:
 *          - Checks wp_nav internal pause flag
 *          - Returns true if trajectory advancement frozen
 *          - Returns false if trajectory actively progressing toward waypoint
 *          - Returns false if wp_nav not initialized (nullptr safety check)
 * 
 *          Null Pointer Safety:
 *          - Checks wp_nav != nullptr before accessing pause state
 *          - Prevents crash if called before wp_nav initialization
 *          - Returns false (not paused) if wp_nav is null
 *          - Defensive programming for startup/shutdown edge cases
 * 
 *          Use Cases:
 *          1. GCS Status Display:
 *             - Ground station shows "PAUSED" indicator when true
 *             - Mission progress display reflects pause state
 *             - Operator aware of navigation hold condition
 * 
 *          2. Companion Computer Decision Making:
 *             - Check before sending new waypoint commands
 *             - Coordinate obstacle avoidance pause/resume cycles
 *             - Verify navigation active before expecting progress
 * 
 *          3. Telemetry Logging:
 *             - Log pause events for mission analysis
 *             - Correlate pause periods with sensor data
 *             - Debug unexpected mission delays
 * 
 *          4. Mission State Monitoring:
 *             - Distinguish between paused and waypoint reached
 *             - Understand why vehicle not advancing
 *             - Validate resume command effectiveness
 * 
 *          Pause State Persistence:
 *          - Pause state persists across loop iterations until resume() called
 *          - Survives position controller updates (pause at wp_nav level)
 *          - Cleared automatically if Auto mode exited
 *          - Reset if wp_nav reinitialized for new navigation command
 * 
 *          Pause State vs. Submode:
 *          - Pause state only relevant in SubMode::WP
 *          - Other submodes (CIRCLE, LAND, RTL) have their own hold mechanisms
 *          - Paused() returns false in non-WP submodes even if wp_nav internally paused
 *          - Pause state decoupled from mission command state machine
 * 
 *          Performance:
 *          - Lightweight query (simple flag check in wp_nav)
 *          - Safe to call at high frequency (400Hz main loop)
 *          - No side effects or state changes
 *          - Const correctness enforced
 * 
 * @return true if waypoint navigation currently paused (trajectory frozen)
 * @return false if navigation active or wp_nav not initialized
 * 
 * @note Const method - does not modify Auto mode state
 * @note Thread-safe (reads single atomic flag in wp_nav)
 * @note Returns false if called before wp_nav initialization
 * @note Pause state specific to waypoint navigation, not other Auto submodes
 * 
 * @see pause() - Pauses waypoint navigation at current position
 * @see resume() - Resumes waypoint navigation after pause
 * @see AC_WPNav::paused() - Underlying wp_nav pause state query
 */
bool ModeAuto::paused() const
{
    return (wp_nav != nullptr) && wp_nav->paused();
}

/**
 * @brief Get height above ground estimate during Auto mode landing
 * 
 * @details This function provides enhanced altitude-above-ground estimation specifically
 *          optimized for Auto mode landing operations. It prioritizes rangefinder data
 *          when available and falls back to mission-specified landing location height
 *          calculations, improving landing precision and safety.
 * 
 *          Altitude Estimation Priority (during LAND submode):
 *          1. Rangefinder (highest priority - direct ground measurement)
 *          2. Height above mission-specified landing location
 *          3. Base class default method (terrain database or barometer)
 * 
 *          Rangefinder Priority Rationale:
 *          - Direct measurement of actual ground distance (most accurate)
 *          - Updates at high frequency (typically 10-50Hz)
 *          - Accounts for obstacles, slopes, and terrain features
 *          - Essential for precision landing and obstacle avoidance
 *          - Interpolated to smooth noisy measurements
 * 
 *          Mission Landing Location Method:
 *          - Used when rangefinder unavailable or out of range
 *          - Calculates height above specified landing waypoint
 *          - Accounts for terrain altitude at landing coordinates
 *          - Valid only if landing command has non-zero lat/lon
 *          - Returns height difference between current position and landing target
 * 
 *          LAND Submode Optimization:
 *          - Enhanced altitude estimation ONLY active during SubMode::LAND
 *          - Other submodes (WP, CIRCLE, RTL, etc.) use base class default method
 *          - Landing-specific logic avoids interfering with other navigation modes
 *          - Ensures consistent behavior across Auto mode operations
 * 
 *          Altitude Source Selection Logic:
 *          
 *          Step 1: Check if in LAND submode
 *          - If not, immediately delegate to base class Mode::get_alt_above_ground_cm()
 *          - Base class uses terrain database, GPS altitude, or barometer
 *          - Standard altitude estimation for non-landing operations
 *          
 *          Step 2: Try rangefinder (if in LAND submode)
 *          - Query copter.get_rangefinder_height_interpolated_cm()
 *          - Returns interpolated rangefinder altitude if sensor healthy and in range
 *          - Interpolation smooths noisy measurements for stable descent control
 *          - If successful, return immediately (highest priority)
 *          
 *          Step 3: Try mission landing location (if rangefinder failed)
 *          - Retrieve current navigation command from mission
 *          - Verify command is MAV_CMD_NAV_LAND or MAV_CMD_NAV_VTOL_LAND
 *          - Check landing location has valid coordinates (non-zero lat/lon)
 *          - Calculate height above landing location using copter.current_loc.get_height_above()
 *          - Converts meters to centimeters for consistent units
 *          - If successful, return height above landing target
 *          
 *          Step 4: Fallback to base class (if all methods failed)
 *          - Use Mode::get_alt_above_ground_cm() as last resort
 *          - Base class method uses terrain database or relative altitude
 *          - Ensures altitude estimate always available for landing control
 * 
 *          Landing Scenarios and Altitude Sources:
 *          
 *          Scenario A: Precision Landing with Rangefinder
 *          - Vehicle descending to land at current location
 *          - Rangefinder detecting ground at 5.2m
 *          - Returns: 520cm (rangefinder measurement)
 *          - Result: Smooth precision landing using direct ground measurement
 *          
 *          Scenario B: Landing at Mission Waypoint (no rangefinder)
 *          - Mission specifies landing at coordinates with terrain data
 *          - Rangefinder out of range (>25m altitude)
 *          - Current altitude 50m, landing target at 30m terrain elevation
 *          - Returns: 2000cm height above landing location (20m difference)
 *          - Result: Controlled descent toward specified landing point
 *          
 *          Scenario C: Emergency Landing (no location, no rangefinder)
 *          - Landing at current location (lat/lon = 0 in command)
 *          - No rangefinder available
 *          - Returns: Mode::get_alt_above_ground_cm() (terrain DB or barometer)
 *          - Result: Best-effort landing with available altitude reference
 * 
 *          Units and Coordinate Frames:
 *          - Return value in centimeters (cm) for consistency with position controller
 *          - Positive values indicate height above ground
 *          - Zero or negative values indicate at or below ground level
 *          - Measurement relative to ground surface, not EKF origin
 * 
 *          Rangefinder Interpolation:
 *          - get_rangefinder_height_interpolated_cm() provides smoothed measurements
 *          - Reduces impact of sensor noise on landing control
 *          - Averages multiple samples for stable altitude estimate
 *          - Handles brief signal loss gracefully
 * 
 *          Error Handling:
 *          - Missing terrain data: Falls through to base class method
 *          - Rangefinder out of range: Falls back to mission location method
 *          - Invalid landing coordinates: Falls back to base class method
 *          - Each fallback ensures altitude estimate always available
 * 
 *          Safety Considerations:
 *          - Rangefinder priority ensures accurate ground detection during final descent
 *          - Mission location fallback prevents blind landing at high altitude
 *          - Base class fallback ensures landing proceeds even with sensor failures
 *          - Altitude estimate critical for landing throttle control and touchdown detection
 * 
 * @return int32_t Estimated height above ground in centimeters
 *         - Positive value: Height above ground surface
 *         - Zero: At ground level
 *         - Negative: Below ground reference (unusual, indicates estimation error)
 * 
 * @note Only provides enhanced estimation during SubMode::LAND
 * @note Rangefinder measurements prioritized for precision landing
 * @note Falls back to mission landing location if rangefinder unavailable
 * @note Always returns valid altitude estimate (guaranteed by fallback chain)
 * @note Called by landing controller at high frequency (100Hz+) during descent
 * 
 * @warning Inaccurate altitude estimate can cause hard landing or premature touchdown
 * @warning Rangefinder must be properly configured and calibrated for safe landing
 * 
 * @see land_run() - Uses altitude estimate for landing descent control
 * @see copter.get_rangefinder_height_interpolated_cm() - Smoothed rangefinder altitude
 * @see Mode::get_alt_above_ground_cm() - Base class default altitude estimation
 */
int32_t ModeAuto::get_alt_above_ground_cm() const
{
    // Only override if in landing submode
    if (_mode == SubMode::LAND) {
        // Rangefinder takes priority
        int32_t alt_above_ground_cm;
        if (copter.get_rangefinder_height_interpolated_cm(alt_above_ground_cm)) {
            return alt_above_ground_cm;
        }

        // Take land altitude from command
        const AP_Mission::Mission_Command& cmd = mission.get_current_nav_cmd();
        switch (cmd.id) {
        case MAV_CMD_NAV_VTOL_LAND:
        case MAV_CMD_NAV_LAND: {
            if (cmd.content.location.lat != 0 || cmd.content.location.lng != 0) {
                // If land location is valid return height above it
                ftype dist;
                if (copter.current_loc.get_height_above(cmd.content.location, dist)) {
                    return dist * 100.0;
                }
            }
            break;
        }

        default:
            // Really should not end up here as were in SubMode land
            break;
        }
    }

    // Use default method
    return Mode::get_alt_above_ground_cm();
}

#endif
