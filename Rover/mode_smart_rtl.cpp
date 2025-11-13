/**
 * @file mode_smart_rtl.cpp
 * @brief Implementation of SmartRTL mode for Rover
 * 
 * @details SmartRTL (Smart Return To Launch) mode implements an intelligent return
 *          to home by replaying the vehicle's recorded path in reverse. Unlike
 *          traditional RTL which returns via a straight line, SmartRTL follows
 *          the actual path taken during the outbound journey, automatically avoiding
 *          obstacles that were encountered along the way.
 * 
 *          Key Features:
 *          - Records breadcrumb trail of positions during normal operation
 *          - Uses AP_SmartRTL library for path recording and simplification
 *          - Replays recorded path in reverse to return home
 *          - Path simplification algorithm reduces unnecessary waypoints
 *          - Falls back to standard behavior if SmartRTL path unavailable
 * 
 *          Algorithm Overview:
 *          1. During normal operation, positions are continuously recorded
 *          2. When SmartRTL is activated, path cleanup/simplification occurs
 *          3. Simplified path is replayed in reverse order
 *          4. Each waypoint is navigated to using the waypoint navigation library
 *          5. If path is exhausted or fails, vehicle stops at destination
 * 
 * @note SmartRTL advantages over standard RTL:
 *       - Avoids obstacles encountered on outbound journey
 *       - More efficient return path through complex terrain
 *       - Reduces backtracking and unnecessary detours
 *       - Particularly useful for rovers navigating around obstacles
 * 
 * @warning SmartRTL requires EKF (Extended Kalman Filter) to be active, not DCM.
 *          Mode entry will fail if EKF origin is not set or if no path has been recorded.
 * 
 * Source: Rover/mode_smart_rtl.cpp
 */
#include "Rover.h"

/**
 * @brief Enter SmartRTL mode
 * 
 * @details This method initializes the SmartRTL mode and validates all prerequisites
 *          are met before allowing mode entry. SmartRTL has strict requirements:
 *          - EKF (Extended Kalman Filter) must be active with valid origin
 *          - AP_SmartRTL library must have recorded a valid path
 *          - Waypoint navigation must initialize successfully
 * 
 *          Initialization sequence:
 *          1. Verify EKF origin is set (SmartRTL requires EKF, not DCM)
 *          2. Verify AP_SmartRTL has recorded a valid return path
 *          3. Initialize waypoint navigation library with RTL speed
 *          4. Set initial desired location to safe stopping point
 *          5. Initialize state machine to WaitForPathCleanup
 * 
 * @return true if mode entry successful and all prerequisites met
 * @return false if EKF origin invalid, no recorded path, or navigation init fails
 * 
 * @note Mode entry failure is common if vehicle hasn't moved since boot, as
 *       SmartRTL requires a minimum path to be recorded before it can be used
 * 
 * @see AP_SmartRTL::is_active() for path validity check
 * @see AR_WPNav::init() for waypoint navigation initialization
 */
bool ModeSmartRTL::_enter()
{
    // SmartRTL requires EKF (not DCM) - EKF provides the origin needed for NED coordinates
    Location ekf_origin;
    if (!ahrs.get_origin(ekf_origin)) {
        return false;
    }

    // Refuse to enter SmartRTL if smart RTL's home has not been set
    // This means no path has been recorded yet or the path is invalid
    if (!g2.smart_rtl.is_active()) {
        return false;
    }

    // Initialize waypoint navigation library with configured RTL speed
    // Speed is clamped to non-negative values for safety
    g2.wp_nav.init(MAX(0, g2.rtl_speed));

    // Set desired location to reasonable stopping point based on current velocity
    // This ensures smooth deceleration when mode is first entered
    if (!g2.wp_nav.set_desired_location_to_stopping_location()) {
        return false;
    }

    // Initialize state machine to wait for path cleanup/simplification
    // Path cleanup removes redundant waypoints for more efficient return
    smart_rtl_state = SmartRTLState::WaitForPathCleanup;
    _loitering = false;

    return true;
}

/**
 * @brief Main update function for SmartRTL mode
 * 
 * @details Implements the SmartRTL state machine that manages the return journey.
 *          Called at the main loop rate (typically 50Hz for rovers) to execute
 *          the appropriate behavior for the current state.
 * 
 *          State Machine:
 *          - WaitForPathCleanup: Initial state where path simplification occurs
 *          - PathFollow: Active navigation following the recorded path in reverse
 *          - StopAtHome: Final state when destination is reached successfully
 *          - Failure: Error state when path following fails
 * 
 *          Path Simplification Algorithm:
 *          The AP_SmartRTL library performs "thorough cleanup" which:
 *          - Removes redundant waypoints that lie on straight segments
 *          - Simplifies the path while maintaining obstacle avoidance
 *          - Reduces memory usage and improves navigation efficiency
 *          - Typically completes in one or two update cycles
 * 
 * @note This function is called at main loop rate, so operations must be efficient
 * @warning Do not block in this function - all operations must be non-blocking
 * 
 * @see AP_SmartRTL::request_thorough_cleanup() for path simplification
 * @see AR_WPNav for waypoint navigation implementation
 */
void ModeSmartRTL::update()
{
    switch (smart_rtl_state) {
        case SmartRTLState::WaitForPathCleanup:
            // Wait for path simplification to complete before starting return journey
            // Path cleanup removes redundant waypoints for efficient navigation
            if (g2.smart_rtl.request_thorough_cleanup()) {
                // Path cleanup complete - transition to following the simplified path
                smart_rtl_state = SmartRTLState::PathFollow;
                _load_point = true;
            }
            // Keep vehicle stationary during path cleanup
            // Note: This may lead to an unnecessary 20ms slowdown of the vehicle (but it is unlikely)
            stop_vehicle();
            break;

        case SmartRTLState::PathFollow:
            // Path following state - navigate through recorded waypoints in reverse order
            // This is the core of SmartRTL where the vehicle retraces its path back home
            
            // Load next waypoint from the path if needed
            if (_load_point) {
                Vector3f dest_NED;
                
                // Pop the next point from the SmartRTL path (LIFO - Last In First Out)
                // This retrieves waypoints in reverse order of recording
                if (!g2.smart_rtl.pop_point(dest_NED)) {
                    // No more points remaining - path is exhausted, we've reached home
                    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Reached destination");
                    smart_rtl_state = SmartRTLState::StopAtHome;
                    break;
                } else {
                    // Successfully retrieved next waypoint - now try to look ahead
                    // Peeking at the next point allows for smoother path following
                    // by letting the navigation controller anticipate upcoming turns
                    Vector3f next_dest_NED;
                    if (g2.smart_rtl.peek_point(next_dest_NED)) {
                        // Look-ahead successful - set both current and next point
                        // This enables smoother cornering and velocity planning
                        if (!g2.wp_nav.set_desired_location_NED(dest_NED, next_dest_NED)) {
                            // Failure to set destination should never occur here
                            // EKF origin was validated during mode entry
                            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SmartRTL: failed to set destination");
                            smart_rtl_state = SmartRTLState::Failure;
                            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
                        }
                    } else {
                        // Peek failed (IO task may have path semaphore) or this is the last point
                        // Fall back to setting only the immediate destination
                        if (!g2.wp_nav.set_desired_location_NED(dest_NED)) {
                            // Failure to set destination should never occur here
                            // EKF origin was validated during mode entry
                            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SmartRTL: failed to set destination");
                            smart_rtl_state = SmartRTLState::Failure;
                            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
                        }
                    }
                }
                _load_point = false;
            }
            
            // Execute waypoint navigation controller to follow the path
            // This updates steering and throttle to navigate toward current waypoint
            navigate_to_waypoint();

            // Check if current waypoint has been reached
            // When reached, flag that the next point should be loaded on next update
            if (g2.wp_nav.reached_destination()) {
                _load_point = true;
            }
            break;

        case SmartRTLState::StopAtHome:
        case SmartRTLState::Failure:
            // Terminal states - either successful completion or failure
            // StopAtHome: Successfully completed SmartRTL path
            // Failure: Error occurred during path following (e.g., navigation failure)
            
            _reached_destination = true;
            
            // Vehicle behavior at destination depends on vehicle type
            // Ground rovers stop in place, boats loiter to maintain position
            if (!rover.is_boat()) {
                // Ground rovers: Stop all motors and hold position
               stop_vehicle();
            } else {
                // Boats: Loiter to maintain position against current/wind
                // Loitering uses active thrust to stay near the target location
                
                // if not loitering yet, start loitering
                if (!_loitering) {
                    // First time in this state - attempt to enter loiter mode
                    _loitering = rover.mode_loiter.enter();
                }
                if (_loitering) {
                    // Loiter mode active - update loiter controller
                    rover.mode_loiter.update();
                } else {
                    // Loiter entry failed - fall back to stopping
                    stop_vehicle();
               }
            }
            break;
    }
}

/**
 * @brief Get the current desired destination location
 * 
 * @details Retrieves the active navigation target for SmartRTL mode. This is used
 *          by other systems (e.g., GCS display, obstacle avoidance) to understand
 *          where the vehicle is trying to go. The destination is only valid during
 *          the PathFollow state when actively navigating toward a waypoint.
 * 
 * @param[out] destination The current target location in global coordinates
 * 
 * @return true if a valid destination is available (PathFollow state)
 * @return false if no active destination (WaitForPathCleanup, StopAtHome, or Failure states)
 * 
 * @note This is a const method and does not modify mode state
 * @note Returns false when not actively following path (e.g., during cleanup or after completion)
 * 
 * @see AR_WPNav::get_destination() for the underlying navigation destination
 */
bool ModeSmartRTL::get_desired_location(Location& destination) const
{
    switch (smart_rtl_state) {
    case SmartRTLState::WaitForPathCleanup:
        // During path cleanup, no active navigation target exists yet
        return false;
    case SmartRTLState::PathFollow:
        // Active path following - return current waypoint destination
        if (g2.wp_nav.is_destination_valid()) {
            destination = g2.wp_nav.get_destination();
            return true;
        }
        return false;
    case SmartRTLState::StopAtHome:
    case SmartRTLState::Failure:
        // Terminal states - no longer navigating to any destination
        return false;
    }
    // Should never reach here but defensive programming for safety
    return false;
}

/**
 * @brief Set the desired maximum speed for SmartRTL navigation
 * 
 * @details Allows external systems to override the maximum speed used during
 *          SmartRTL path following. This is used for dynamic speed control
 *          based on conditions like obstacle proximity or battery level.
 * 
 * @param[in] speed Desired maximum speed in m/s (meters per second)
 * 
 * @return true if speed was set successfully
 * @return false if speed setting failed (invalid waypoint navigation state)
 * 
 * @note The speed parameter is not validated here - the waypoint navigation
 *       library (AR_WPNav) handles speed limiting and validation
 * @note This affects the speed of the return journey but not the recorded path itself
 * 
 * @see AR_WPNav::set_speed_max() for underlying speed control
 */
bool ModeSmartRTL::set_desired_speed(float speed)
{
    return g2.wp_nav.set_speed_max(speed);
}

/**
 * @brief Save current position to the SmartRTL path breadcrumb trail
 * 
 * @details This method is called regularly from the main scheduler to record the
 *          vehicle's position as it moves. These positions form a breadcrumb trail
 *          that can be followed in reverse to return home via SmartRTL.
 * 
 *          Path Recording Strategy:
 *          - Positions are recorded continuously during normal operation
 *          - Recording is disabled when already in SmartRTL mode (no recording during return)
 *          - AP_SmartRTL library handles position storage and path simplification
 *          - Oldest positions are pruned when memory limit is reached (FIFO buffer)
 * 
 *          Breadcrumb Trail Management:
 *          The AP_SmartRTL library implements intelligent position recording:
 *          - Only records positions when vehicle has moved significantly
 *          - Automatically simplifies path to reduce memory usage
 *          - Maintains a balance between path accuracy and memory efficiency
 *          - Typically stores several hundred waypoints depending on configuration
 * 
 * @note Called at regular intervals by the scheduler (typically 3Hz)
 * @note Does not record positions when already in SmartRTL mode to avoid
 *       recording the return path which would create a loop
 * @note Path recording requires valid EKF position estimate
 * 
 * @see AP_SmartRTL::update() for the underlying path recording implementation
 * @see AP_SmartRTL for breadcrumb trail storage and management
 */
void ModeSmartRTL::save_position()
{
    // Only save position when NOT currently in SmartRTL mode
    // This prevents recording the return path which would create an infinite loop
    const bool save_pos = (rover.control_mode != &rover.mode_smartrtl);
    
    // Update AP_SmartRTL library with current time and save flag
    // The library handles actual position recording and path management
    g2.smart_rtl.update(true, save_pos);
}
