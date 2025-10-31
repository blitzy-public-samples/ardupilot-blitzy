/**
 * @file mode_smart_rtl.cpp
 * @brief Smart Return to Launch (SmartRTL) flight mode implementation
 * 
 * @details SmartRTL mode provides an intelligent return-to-home capability that retraces
 *          the vehicle's actual outbound flight path rather than flying a direct line home.
 *          This approach offers several safety advantages:
 *          - Returns through known-good airspace that was successfully traversed outbound
 *          - Automatically avoids obstacles that were encountered and avoided on the outbound flight
 *          - Useful in areas with complex terrain, urban environments, or obstacle-rich spaces
 *          
 *          Algorithm Overview:
 *          1. Path Recording: During normal flight, the AP_SmartRTL library continuously records
 *             the vehicle's position at regular intervals, storing waypoints in a memory buffer
 *          2. Path Simplification: Points are simplified using Ramer-Douglas-Peucker algorithm
 *             to reduce memory usage while preserving path shape (removes redundant points on
 *             straight segments)
 *          3. Path Cleanup: When SmartRTL is triggered, the path is "cleaned" by removing loops
 *             and optimizing the return route
 *          4. Path Following: Vehicle retraces the simplified path in reverse, one waypoint at
 *             a time, using the WPNav controller for smooth navigation
 *          5. Final Approach: Upon reaching the origin, executes standard RTL descent and landing
 *          
 *          Memory Limitations:
 *          - Path length is limited by available memory (configured via SRTL_POINTS parameter)
 *          - Typical configurations support 200-500 points depending on board memory
 *          - If memory fills during flight, oldest points are discarded (FIFO behavior)
 *          - Path recording stops if memory is exhausted
 *          
 *          Fallback Behavior:
 *          - If no valid path is available (e.g., armed without prior flight), mode will fail
 *             to initialize and vehicle will fall back to regular RTL mode
 *          - If path is lost or corrupted during return, vehicle transitions to standard RTL
 *          
 *          Integration:
 *          - Uses g2.smart_rtl (AP_SmartRTL library) for path storage and retrieval
 *          - Uses wp_nav (AC_WPNav) for waypoint navigation and trajectory generation
 *          - Uses pos_control (AC_PosControl) for position hold and altitude control
 *          - Reuses descent_run() and land_run() from standard RTL mode for final landing
 *          
 * @note Path is recorded whenever vehicle is armed and NOT in SmartRTL mode
 * @warning Path is stored in RAM only and is lost on reboot/power cycle
 * 
 * @see AP_SmartRTL - Path recording and management library
 * @see AC_WPNav - Waypoint navigation controller
 * @see mode_rtl.cpp - Standard RTL mode (fallback and descent/land logic)
 */

#include "Copter.h"

#if MODE_SMARTRTL_ENABLED

/**
 * @brief Initialize Smart RTL flight mode
 * 
 * @details This function is called when the vehicle transitions into SmartRTL mode.
 *          It performs initialization checks and sets up the navigation controllers
 *          for path following.
 *          
 *          Initialization sequence:
 *          1. Verify that a valid SmartRTL path exists in memory (is_active check)
 *          2. Initialize waypoint and spline navigation controllers
 *          3. Calculate and set a smooth stopping point as initial target (prevents aggressive
 *             maneuvers when mode is entered at high speed)
 *          4. Configure yaw behavior according to SRTL_OPTIONS parameter
 *          5. Transition to WAIT_FOR_PATH_CLEANUP state to allow path optimization
 *          
 *          The stopping point calculation ensures the vehicle smoothly decelerates to the
 *          current position rather than attempting an immediate turn toward the return path,
 *          which could cause aggressive banking or overshoot.
 * 
 * @param[in] ignore_checks Currently unused for SmartRTL mode (standard mode interface parameter)
 * 
 * @return true if mode successfully initialized (valid path available), false otherwise
 * 
 * @note If this function returns false, the flight mode controller will typically fall back
 *       to regular RTL mode or the previous flight mode depending on configuration
 * @note Path availability depends on prior flight - if vehicle was just armed without flying,
 *       no path will be available and initialization will fail
 * 
 * @see g2.smart_rtl.is_active() - Checks if valid path data exists
 * @see wait_cleanup_run() - Next state after initialization
 */
bool ModeSmartRTL::init(bool ignore_checks)
{
    if (g2.smart_rtl.is_active()) {
        // initialise waypoint and spline controller
        wp_nav->wp_and_spline_init_cm();

        // set current target to a reasonable stopping point
        Vector3p stopping_point;
        pos_control->get_stopping_point_NE_cm(stopping_point.xy());
        pos_control->get_stopping_point_U_cm(stopping_point.z);
        wp_nav->set_wp_destination_NEU_cm(stopping_point.tofloat());

        // initialise yaw to obey user parameter
        auto_yaw.set_mode_to_default(true);

        // wait for cleanup of return path
        smart_rtl_state = SubMode::WAIT_FOR_PATH_CLEANUP;
        return true;
    }

    return false;
}

/**
 * @brief Perform cleanup when exiting SmartRTL mode
 * 
 * @details This function is called when the vehicle transitions out of SmartRTL mode
 *          (e.g., pilot switches to another mode, failsafe triggers, etc.). It ensures
 *          the path data remains valid for potential future SmartRTL attempts.
 *          
 *          Cleanup operations:
 *          1. Restore Unreached Waypoint: If the vehicle was following a path point that
 *             wasn't reached yet, restore it to the path so it's available if SmartRTL
 *             is re-entered later. This prevents losing a segment of the return path.
 *          2. Clear Backup: Zero out the destination backup to prevent stale data
 *          3. Cancel Path Cleanup: Stop any ongoing path optimization operations in the
 *             background task to free up processing resources
 * 
 * @note The backup/restore mechanism ensures path integrity if SmartRTL is interrupted
 *       and later resumed
 * @warning If add_point fails due to memory constraints, a warning is sent to GCS but
 *          operation continues (point is lost but not critical to safety)
 * 
 * @see g2.smart_rtl.add_point() - Restores point to path
 * @see dest_NED_backup - Stores current target for restoration
 */
void ModeSmartRTL::exit()
{
    // restore last point if we hadn't reached it
    if (smart_rtl_state == SubMode::PATH_FOLLOW && !dest_NED_backup.is_zero()) {
        if (!g2.smart_rtl.add_point(dest_NED_backup)) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "SmartRTL: lost one point");
        }
    }
    dest_NED_backup.zero();

    g2.smart_rtl.cancel_request_for_thorough_cleanup();
}

/**
 * @brief Main run function for SmartRTL mode, called at scheduler rate (typically 400Hz)
 * 
 * @details Implements the SmartRTL state machine by dispatching to appropriate sub-mode
 *          handlers. The state machine progresses through the following sequence:
 *          
 *          State Machine Progression:
 *          1. WAIT_FOR_PATH_CLEANUP - Initial state after mode entry, waits for background
 *             path optimization (loop removal, simplification) to complete before beginning
 *             return journey
 *          2. PATH_FOLLOW - Main return phase, vehicle retraces recorded path waypoint by
 *             waypoint in reverse order until reaching the origin point
 *          3. PRELAND_POSITION - Brief positioning phase at 2m above home point to align
 *             for final descent
 *          4. DESCEND - Controlled descent to RTL_ALT_FINAL altitude (if configured)
 *          5. LAND - Final landing sequence with precision position hold
 *          
 *          Each state handler manages:
 *          - Motor throttle state configuration
 *          - Position/velocity controller updates
 *          - Attitude controller commands with appropriate yaw behavior
 *          - State transition logic when objectives are met
 * 
 * @note Called at main loop rate (typically 400Hz) - handlers must execute efficiently
 * @note States DESCEND and LAND reuse logic from standard RTL mode for consistency
 * 
 * @see wait_cleanup_run() - Handles WAIT_FOR_PATH_CLEANUP state
 * @see path_follow_run() - Handles PATH_FOLLOW state
 * @see pre_land_position_run() - Handles PRELAND_POSITION state
 * @see descent_run() - Inherited from ModeRTL for DESCEND state
 * @see land_run() - Inherited from ModeRTL for LAND state
 */
void ModeSmartRTL::run()
{
    switch (smart_rtl_state) {
        case SubMode::WAIT_FOR_PATH_CLEANUP:
            wait_cleanup_run();
            break;
        case SubMode::PATH_FOLLOW:
            path_follow_run();
            break;
        case SubMode::PRELAND_POSITION:
            pre_land_position_run();
            break;
        case SubMode::DESCEND:
            descent_run(); // Re-using the descend method from normal rtl mode.
            break;
        case SubMode::LAND:
            land_run(true); // Re-using the land method from normal rtl mode.
            break;
    }
}

/**
 * @brief Query if vehicle is in landing phase of SmartRTL
 * 
 * @details Returns true only when in the final LAND state, indicating active landing
 *          with disarm-on-ground logic enabled. Used by the autopilot to determine
 *          when landing detectors and auto-disarm should be active.
 * 
 * @return true if in LAND state, false for all other states including descent
 * 
 * @note DESCEND state returns false since vehicle is not yet landing (still descending)
 */
bool ModeSmartRTL::is_landing() const
{
    return smart_rtl_state == SubMode::LAND;
}

/**
 * @brief Wait for path cleanup/optimization before beginning return journey
 * 
 * @details This function runs in the initial WAIT_FOR_PATH_CLEANUP state immediately
 *          after SmartRTL mode is entered. It maintains a stable hover while the
 *          AP_SmartRTL library performs "thorough cleanup" of the recorded path.
 *          
 *          Path Cleanup Process (runs in background IO task):
 *          - Loop Removal: Detects and removes loops in the path (e.g., if vehicle
 *            circled back over previous positions)
 *          - Path Simplification: Further optimizes the path by removing redundant
 *            points while preserving overall path shape
 *          - Point Ordering: Ensures points are properly ordered for reverse traversal
 *          
 *          This cleanup is critical because:
 *          - Raw recorded path may contain inefficiencies (loops, redundant points)
 *          - Cleaned path is shorter and faster to follow
 *          - Removes potentially unsafe maneuvers (e.g., sudden reversals)
 *          
 *          During this state:
 *          - Vehicle holds current position with full throttle authority
 *          - Waypoint controller maintains target at stopping point from init()
 *          - Altitude controller maintains current altitude
 *          - Yaw follows configured behavior (pilot control or auto)
 *          
 *          Transition: Once cleanup completes (request_thorough_cleanup returns true),
 *          transitions to PATH_FOLLOW state to begin retracing the optimized path.
 * 
 * @note Cleanup typically completes within a few scheduler cycles but may take longer
 *       for complex paths or on resource-constrained boards
 * @note If cleanup fails or path becomes invalid, mode may need to transition to RTL
 * 
 * @see g2.smart_rtl.request_thorough_cleanup() - Requests and checks cleanup completion
 * @see path_follow_run() - Next state after cleanup completes
 */
void ModeSmartRTL::wait_cleanup_run()
{
    // hover at current target position
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    wp_nav->update_wpnav();
    pos_control->update_U_controller();
    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.get_heading());

    // check if return path is computed and if yes, begin journey home
    if (g2.smart_rtl.request_thorough_cleanup()) {
        path_follow_last_pop_fail_ms = 0;
        smart_rtl_state = SubMode::PATH_FOLLOW;
    }
}

/**
 * @brief Follow the SmartRTL path home, navigating waypoint by waypoint
 * 
 * @details This is the core SmartRTL function that executes the return journey by
 *          sequentially navigating through the recorded path points in reverse order
 *          (LIFO - Last In First Out from the path stack).
 *          
 *          Path Following Algorithm:
 *          1. Check Waypoint Reached: When wp_nav reports current waypoint reached,
 *             fetch the next point from the path
 *          2. Pop Point: Retrieve next path point from AP_SmartRTL library (LIFO order)
 *          3. Backup Point: Store popped point in dest_NED_backup for restoration if
 *             mode is exited before reaching it
 *          4. Look-Ahead: Peek at the following point to enable smooth trajectory
 *             generation with the spline controller (prevents jerky turns)
 *          5. Final Point Handling: When last point is reached, add 2m altitude offset
 *             and transition to pre-land positioning state
 *          
 *          Thread-Safety Considerations:
 *          - Path operations (pop_point, peek_point) use semaphores for thread safety
 *          - AP_SmartRTL library runs cleanup in background IO task
 *          - pop_point/peek_point may fail if IO task holds semaphore
 *          - Failures are tolerated with timeout monitoring (10 second limit)
 *          
 *          Smooth Navigation Strategy:
 *          - wp_nav uses current point + next point to generate smooth spline trajectories
 *          - Vehicle anticipates turns by knowing upcoming waypoint
 *          - Reduces aggressive maneuvering compared to reaching each point before turning
 *          - If peek fails (semaphore contention), sends point anyway - vehicle will slow
 *            at waypoint but continue safely
 *          
 *          Error Handling:
 *          - Semaphore Contention: Tolerated up to 10 seconds before declaring error
 *          - Path Exhausted Unexpectedly: Should never occur, triggers internal error
 *          - Persistent Failures: After 10s of pop failures, transitions to pre-land state
 *            as safety fallback
 *          
 *          During this state:
 *          - Motors at full throttle authority for responsive control
 *          - Position and altitude controllers updated every cycle
 *          - Yaw behavior follows SRTL_OPTIONS parameter configuration
 * 
 * @note Called at 400Hz - must be efficient and handle semaphore failures gracefully
 * @note NED coordinate frame: North-East-Down, altitude is negative up
 * @note The 2m altitude offset at final point provides clearance for pre-land positioning
 * 
 * @warning Semaphore failures are expected occasionally due to concurrent access but
 *          persistent failures indicate a serious problem requiring failsafe transition
 * 
 * @see g2.smart_rtl.pop_point() - Retrieves next path point (LIFO)
 * @see g2.smart_rtl.peek_point() - Look-ahead for next point without removing
 * @see wp_nav->set_wp_destination_NED_m() - Sets current waypoint target
 * @see wp_nav->set_wp_destination_next_NED_m() - Sets look-ahead point for smooth trajectory
 * @see pre_land_position_run() - Next state after reaching origin
 */
void ModeSmartRTL::path_follow_run()
{
    // if we are close to current target point, switch the next point to be our target.
    if (wp_nav->reached_wp_destination()) {

        // clear destination backup so that it cannot be restored
        dest_NED_backup.zero();

        // this pop_point can fail if the IO task currently has the
        // path semaphore.
        Vector3f dest_NED;
        if (g2.smart_rtl.pop_point(dest_NED)) {
            // backup destination in case we exit smart_rtl mode and need to restore it to the path
            dest_NED_backup = dest_NED;
            path_follow_last_pop_fail_ms = 0;
            if (g2.smart_rtl.get_num_points() == 0) {
                // this is the very last point, add 2m to the target alt and move to pre-land state
                dest_NED.z -= 2.0f;
                smart_rtl_state = SubMode::PRELAND_POSITION;
                wp_nav->set_wp_destination_NED_m(dest_NED);
            } else {
                // peek at the next point.  this can fail if the IO task currently has the path semaphore
                Vector3f next_dest_NED;
                if (g2.smart_rtl.peek_point(next_dest_NED)) {
                    wp_nav->set_wp_destination_NED_m(dest_NED);
                    if (g2.smart_rtl.get_num_points() == 1) {
                        // this is the very last point, add 2m to the target alt
                        next_dest_NED.z -= 2.0f;
                    }
                    wp_nav->set_wp_destination_next_NED_m(next_dest_NED);
                } else {
                    // this can only happen if peek failed to take the semaphore
                    // send next point anyway which will cause the vehicle to slow at the next point
                    wp_nav->set_wp_destination_NED_m(dest_NED);
                    INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
                }
            }
        } else if (g2.smart_rtl.get_num_points() == 0) {
            // We should never get here; should always have at least
            // two points and the "zero points left" is handled above.
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
            smart_rtl_state = SubMode::PRELAND_POSITION;
        } else if (path_follow_last_pop_fail_ms == 0) {
            // first time we've failed to pop off (ever, or after a success)
            path_follow_last_pop_fail_ms = AP_HAL::millis();
        } else if (AP_HAL::millis() - path_follow_last_pop_fail_ms > 10000) {
            // we failed to pop a point off for 10 seconds.  This is
            // almost certainly a bug.
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
            smart_rtl_state = SubMode::PRELAND_POSITION;
        }
    }

    // update controllers
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    wp_nav->update_wpnav();
    pos_control->update_U_controller();

    // call attitude controller with auto yaw
    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.get_heading());
}

/**
 * @brief Pre-landing positioning at 2m above home point
 * 
 * @details This state provides a brief positioning phase 2 meters above the return
 *          origin (typically launch location) before beginning final descent and landing.
 *          The altitude offset provides clearance for precise positioning and allows
 *          the vehicle to settle any oscillations before landing.
 *          
 *          Transition Logic:
 *          Once positioned at the target (2m above origin):
 *          - If RTL_ALT_FINAL > 0: Transition to DESCEND state and descend to the
 *            configured final altitude before landing (allows pilot inspection time)
 *          - If RTL_ALT_FINAL <= 0: Skip descent phase and transition directly to LAND
 *          - If radio failsafe active: Skip descent and land immediately for safety
 *          
 *          RTL_ALT_FINAL Parameter:
 *          - Value > 0: Vehicle will descend to this altitude (in cm) and hold,
 *            requiring pilot confirmation to land (via mode change or throttle stick)
 *          - Value = 0: Automatic landing without pause
 *          - Value < 0: Automatic landing without pause
 *          
 *          Failsafe Override:
 *          - If radio failsafe is active, skips descent-and-hold phase and proceeds
 *            directly to landing regardless of RTL_ALT_FINAL setting (safety priority)
 * 
 * @note The 2m offset was set in path_follow_run() when the last waypoint was processed
 * @note This state reuses descent_start() and land_start() from standard RTL mode
 * 
 * @see descent_run() - Next state if RTL_ALT_FINAL > 0
 * @see land_run() - Next state if RTL_ALT_FINAL <= 0 or failsafe active
 * @see g.rtl_alt_final - Final altitude parameter (centimeters)
 */
void ModeSmartRTL::pre_land_position_run()
{
    // if we are close to 2m above start point, we are ready to land.
    if (wp_nav->reached_wp_destination()) {
        // choose descend and hold, or land based on user parameter rtl_alt_final
        if (g.rtl_alt_final <= 0 || copter.failsafe.radio) {
            land_start();
            smart_rtl_state = SubMode::LAND;
        } else {
            set_descent_target_alt(copter.g.rtl_alt_final);
            descent_start();
            smart_rtl_state = SubMode::DESCEND;
        }
    }

    // update controllers
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    wp_nav->update_wpnav();
    pos_control->update_U_controller();
    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.get_heading());
}

/**
 * @brief Save current vehicle position to the SmartRTL path (called periodically during flight)
 * 
 * @details This function is called at regular intervals during normal flight operations to
 *          record the vehicle's position for later use by SmartRTL mode. It implements the
 *          path recording logic that enables SmartRTL to retrace the outbound journey.
 *          
 *          Recording Logic:
 *          - Only records when vehicle is armed AND not currently in SmartRTL mode
 *          - Recording happens continuously during all other flight modes (Loiter, Auto,
 *            Guided, PosHold, etc.)
 *          - Path recording paused during SmartRTL mode itself (prevents recording the
 *            return journey)
 *          
 *          Path Storage (handled by AP_SmartRTL library):
 *          - Positions stored in 3D (NED frame): North, East, Down coordinates
 *          - Points automatically simplified using Ramer-Douglas-Peucker algorithm to
 *            save memory while preserving path shape
 *          - Straight-line segments compressed (intermediate points removed)
 *          - Path limited by SRTL_POINTS parameter (200-500 typical)
 *          - Oldest points discarded when memory fills (FIFO with simplification)
 *          
 *          Position Validity:
 *          - Only records if position_ok() returns true (GPS lock, EKF healthy)
 *          - Invalid positions ignored to prevent corrupted path data
 *          - Ensures only reliable position estimates are stored
 *          
 *          Recording Frequency:
 *          - Actual storage rate managed by AP_SmartRTL (typically 0.2-3 Hz depending
 *            on vehicle movement and path complexity)
 *          - update() call may not store every position (intelligent sampling)
 * 
 * @note This function is typically called from the main scheduler at 1-10 Hz rate
 * @note Path is stored in RAM only and lost on power cycle/reboot
 * @warning Recording disabled during SmartRTL mode prevents infinite path growth
 * 
 * @see copter.g2.smart_rtl.update() - AP_SmartRTL library path recording function
 * @see copter.position_ok() - Position validity check (GPS + EKF health)
 */
void ModeSmartRTL::save_position()
{
    const bool should_save_position = motors->armed() && (copter.flightmode->mode_number() != Mode::Number::SMART_RTL);

    copter.g2.smart_rtl.update(copter.position_ok(), should_save_position);
}

/**
 * @brief Get current target waypoint location for telemetry and ground station display
 * 
 * @details Provides the current navigation target to the ground control station for
 *          display on the map and for telemetry logging. Different states have different
 *          target reporting:
 *          
 *          States with valid waypoint targets:
 *          - WAIT_FOR_PATH_CLEANUP: Reports stopping point from initialization
 *          - PATH_FOLLOW: Reports current path point being navigated toward
 *          - PRELAND_POSITION: Reports 2m above origin position
 *          - DESCEND: Reports descent target altitude position
 *          
 *          State without waypoint target:
 *          - LAND: Returns false as landing uses vertical-only control, no horizontal target
 * 
 * @param[out] destination Location structure to populate with current target waypoint
 * 
 * @return true if valid waypoint available (WAIT_FOR_PATH_CLEANUP through DESCEND states)
 * @return false if no waypoint target (LAND state)
 * 
 * @note Used by GCS_MAVLink for NAV_CONTROLLER_OUTPUT message
 * @see wp_nav->get_wp_destination_loc() - Retrieves current waypoint from navigation controller
 */
bool ModeSmartRTL::get_wp(Location& destination) const
{
    // provide target in states which use wp_nav
    switch (smart_rtl_state) {
    case SubMode::WAIT_FOR_PATH_CLEANUP:
    case SubMode::PATH_FOLLOW:
    case SubMode::PRELAND_POSITION:
    case SubMode::DESCEND:
        return wp_nav->get_wp_destination_loc(destination);
    case SubMode::LAND:
        return false;
    }

    // we should never get here but just in case
    return false;
}

/**
 * @brief Get horizontal distance to current waypoint target in meters
 * 
 * @details Provides the straight-line horizontal distance from vehicle current position
 *          to the active waypoint target. Used for telemetry, logging, and ground station
 *          display of progress toward target.
 * 
 * @return Horizontal distance to waypoint in meters (2D distance, altitude ignored)
 * 
 * @note Returns distance in meters (converted from internal centimeter representation)
 * @see get_wp() - Retrieves the waypoint location this distance refers to
 */
float ModeSmartRTL::wp_distance_m() const
{
    return wp_nav->get_wp_distance_to_destination_cm() * 0.01f;
}

/**
 * @brief Get bearing to current waypoint target in degrees
 * 
 * @details Provides the compass bearing (direction) from vehicle current position to
 *          the active waypoint target. Used for telemetry, navigation display, and
 *          ground station directional indicators.
 *          
 *          Bearing Convention:
 *          - 0° = North
 *          - 90° = East  
 *          - 180° = South
 *          - 270° = West
 *          - Range: 0-360 degrees
 * 
 * @return Bearing to waypoint in degrees (0-360°, true north reference)
 * 
 * @note Converts from radians (internal representation) to degrees for external interface
 * @see get_wp() - Retrieves the waypoint location this bearing refers to
 */
float ModeSmartRTL::wp_bearing_deg() const
{
    return degrees(wp_nav->get_wp_bearing_to_destination_rad());
}

/**
 * @brief Determine if pilot yaw input should override automatic yaw control
 * 
 * @details Controls whether pilot's rudder/yaw stick input is honored or ignored during
 *          SmartRTL operation. Yaw control behavior varies by flight phase and configuration.
 *          
 *          Pilot Yaw Enabled When:
 *          1. SRTL_OPTIONS parameter enables pilot yaw control (bit flag configuration)
 *          2. In DESCEND state with LAND_REPOSITION enabled (allows pilot to adjust heading
 *             during descent for better landing orientation)
 *          3. In LAND state (allows final heading adjustments during touchdown)
 *          
 *          Pilot Yaw Disabled (Automatic) When:
 *          - SRTL_OPTIONS disables pilot control AND
 *          - Not in descent-with-repositioning phase AND
 *          - Not in final landing
 *          
 *          Typical automatic yaw behaviors (when pilot control disabled):
 *          - Point in direction of travel (most common)
 *          - Hold initial heading
 *          - Point toward home
 *          - Face Region of Interest (ROI)
 *          
 *          Landing Phase Special Cases:
 *          - DESCEND with LAND_REPOSITION=1: Pilot yaw enabled for repositioning
 *          - LAND: Always pilot yaw enabled for final touchdown adjustments
 * 
 * @return true if pilot yaw input should be used
 * @return false if automatic yaw control should be used
 * 
 * @note SRTL_OPTIONS is a bitmask parameter controlling various SmartRTL behaviors
 * @note LAND_REPOSITION allows pilot to adjust position during descent for precision landing
 * 
 * @see g2.smart_rtl.use_pilot_yaw() - Checks SRTL_OPTIONS parameter
 * @see g.land_repositioning - Landing repositioning enable flag
 * @see auto_yaw.set_mode_to_default() - Sets automatic yaw behavior in init()
 */
bool ModeSmartRTL::use_pilot_yaw() const
{
    const bool land_repositioning = g.land_repositioning && (smart_rtl_state == SubMode::DESCEND);
    const bool final_landing = smart_rtl_state == SubMode::LAND;
    return g2.smart_rtl.use_pilot_yaw() || land_repositioning || final_landing;
}

#endif
