/**
 * @file mode_rtl.cpp
 * @brief Implementation of RTL (Return To Launch) mode for Rover
 * 
 * @details RTL mode autonomously navigates the rover back to its home position or
 *          the nearest rally point (if rally points are configured). The mode uses
 *          the waypoint navigation controller to follow a direct path to the target
 *          location.
 *          
 *          RTL Behavior:
 *          - On entry: Calculates best return destination (home or nearest rally point)
 *          - During navigation: Uses waypoint controller to navigate to destination
 *          - On arrival: Rovers stop, boats enter loiter mode
 *          
 *          Rally Point Integration:
 *          If rally points are enabled (HAL_RALLY_ENABLED), RTL will navigate to the
 *          closest rally point instead of home if it's closer to the current position.
 *          
 *          RTL Completion Behavior:
 *          - Ground rovers: Stop at destination (stop_vehicle)
 *          - Boats: Enter loiter mode to maintain position
 *          
 *          Configuration:
 *          - RTL_SPEED: Speed to travel during RTL (m/s)
 *          
 * @note RTL mode can be triggered automatically by failsafes when FS_ACTION
 *       parameter is set to RTL (typically value 2)
 * @note RTL will refuse to enter if home position has not been set
 * 
 * @see ModeRTL class in mode.h
 * @see AR_WPNav for waypoint navigation controller
 * 
 * Source: Rover/mode_rtl.cpp
 */

#include "Rover.h"

/**
 * @brief Enter RTL mode and configure return navigation
 * 
 * @details Initializes RTL mode by:
 *          1. Verifying home position is set (required for RTL)
 *          2. Initializing waypoint navigation with RTL_SPEED parameter
 *          3. Selecting return destination (rally point or home)
 *          4. Configuring navigation controller to navigate to destination
 * 
 * @return true if RTL mode successfully entered, false if home not set or destination invalid
 * 
 * @note RTL will fail to enter if home position has not been set via GPS or MAVLink
 */
bool ModeRTL::_enter()
{
    // refuse RTL if home has not been set
    // Home must be set before RTL can be used, either via GPS lock or MAVLink command
    if (!AP::ahrs().home_is_set()) {
        return false;
    }

    // initialise waypoint navigation library with RTL_SPEED parameter
    // Uses maximum of 0.0 and configured RTL speed to prevent negative speeds
    g2.wp_nav.init(MAX(0.0f, g2.rtl_speed));

    // Rally Point Integration: Set target to the closest rally point or home
    // If rally points are configured, calc_best_rally_or_home_location() will
    // calculate the distance to each rally point and home, then return the
    // closest one. This minimizes return distance and battery usage.
#if HAL_RALLY_ENABLED
    if (!g2.wp_nav.set_desired_location(g2.rally.calc_best_rally_or_home_location(rover.current_loc, ahrs.get_home().alt))) {
        return false;
    }
#else
    // Rally points disabled: set destination directly to home position
    if (!g2.wp_nav.set_desired_location(ahrs.get_home())) {
        return false;
    }
#endif

    // Prepare for RTL navigation
    send_notification = true;  // Flag to send "Reached destination" notification when arriving
    _loitering = false;        // Clear loiter state for boats
    return true;
}

/**
 * @brief Main update loop for RTL mode, called at scheduler rate (typically 50Hz)
 * 
 * @details RTL Navigation Algorithm:
 *          
 *          Phase 1 - Active Navigation (until destination reached):
 *          - Check if destination has been reached using waypoint controller
 *          - If not reached: Call navigate_to_waypoint() to update steering/throttle
 *          - navigate_to_waypoint() uses L1 controller for path following
 *          
 *          Phase 2 - Destination Arrival (when reached_destination() returns true):
 *          - Send "Reached destination" notification to ground station (once)
 *          - Execute completion behavior based on vehicle type:
 *            * Ground rovers: Stop vehicle (zero throttle, hold position)
 *            * Boats: Enter loiter mode to maintain position against currents
 *          
 *          RTL Completion Behavior:
 *          The behavior upon reaching home/rally point depends on vehicle configuration:
 *          - Rovers (is_boat() == false): Call stop_vehicle() to halt and hold position
 *          - Boats (is_boat() == true): Transition to loiter mode for active position hold
 *          
 *          If loiter mode entry fails for boats, fallback to stop_vehicle() for safety.
 *          
 *          Distance tracking continues even after arrival for telemetry purposes.
 * 
 * @note This is called at scheduler rate, typically 50Hz for rovers
 * @note navigate_to_waypoint() calculates desired steering and throttle outputs
 * @note The waypoint controller (g2.wp_nav) handles obstacle avoidance if enabled
 * 
 * @see navigate_to_waypoint() for steering/throttle calculation
 * @see AR_WPNav::reached_destination() for arrival detection logic
 */
void ModeRTL::update()
{
    // RTL Navigation Algorithm: Navigate to waypoint or handle arrival
    
    // Phase 1: Active Navigation - determine if we should keep navigating
    if (!g2.wp_nav.reached_destination()) {
        // Still navigating to destination - update navigation controller
        // navigate_to_waypoint() calculates desired steering and throttle based on:
        // - Current position and heading
        // - Target waypoint (home or rally point)
        // - Configured RTL speed
        // - L1 controller guidance algorithm
        navigate_to_waypoint();
    } else {
        // Phase 2: Destination Arrival - we have reached home or rally point
        
        // Send one-time notification to ground control station
        if (send_notification) {
            send_notification = false;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Reached destination");
        }

        // RTL Completion Behavior: Execute vehicle-type-specific arrival behavior
        // Ground rovers stop and hold position, boats actively loiter
        if (!rover.is_boat()) {
            // Ground rover completion: Stop vehicle (zero throttle, engaged brake if available)
            stop_vehicle();
        } else {
            // Boat completion: Enter loiter mode for active position maintenance
            // Boats need active loiter to counter water currents and maintain position
            
            // if not loitering yet, attempt to start loitering
            if (!_loitering) {
                _loitering = rover.mode_loiter.enter();
            }
            
            // Execute loiter behavior or stop as fallback
            if (_loitering) {
                // Loiter mode successfully entered - update loiter controller
                rover.mode_loiter.update();
            } else {
                // Loiter mode entry failed - fallback to stopping for safety
                stop_vehicle();
            }
        }

        // Update distance to destination for telemetry reporting
        // This continues even after arrival for ground station display
        _distance_to_destination = rover.current_loc.get_distance(g2.wp_nav.get_destination());
    }
}

/**
 * @brief Get the current RTL destination location (home or rally point)
 * 
 * @details Returns the active RTL destination including any obstacle avoidance
 *          adjustments. The destination is set during _enter() and may be modified
 *          by the object avoidance system if enabled.
 *          
 *          Returns obstacle-avoidance-adjusted destination (get_oa_destination) rather
 *          than raw destination to reflect actual navigation target.
 * 
 * @param[out] destination Location object to be filled with RTL target coordinates
 * 
 * @return true if destination is valid and was copied to parameter, false if no valid destination
 * 
 * @note This returns the OA (obstacle avoidance) destination which may differ from
 *       the original home/rally point if obstacles are detected
 * 
 * @see AR_WPNav::get_oa_destination() for obstacle avoidance details
 */
bool ModeRTL::get_desired_location(Location& destination) const
{
    if (g2.wp_nav.is_destination_valid()) {
        // Return obstacle-avoidance-adjusted destination
        destination = g2.wp_nav.get_oa_destination();
        return true;
    }
    return false;
}

/**
 * @brief Check if rover has reached the RTL destination (home or rally point)
 * 
 * @details Queries the waypoint navigation controller to determine if the vehicle
 *          has arrived at the RTL destination. The waypoint controller considers
 *          the destination "reached" when the vehicle is within the configured
 *          waypoint radius (WP_RADIUS parameter).
 * 
 * @return true if within waypoint radius of destination, false if still navigating
 * 
 * @note Waypoint radius is configurable via WP_RADIUS parameter (default 2 meters)
 * @note This is used by update() to switch from navigation to completion behavior
 * 
 * @see AR_WPNav::reached_destination() for arrival detection algorithm
 */
bool ModeRTL::reached_destination() const
{
    return g2.wp_nav.reached_destination();
}

/**
 * @brief Set the desired RTL navigation speed
 * 
 * @details Updates the maximum speed used during RTL navigation. This allows
 *          dynamic speed changes during RTL, though typically RTL uses the
 *          configured RTL_SPEED parameter set during _enter().
 *          
 *          Speed changes take effect immediately and affect the waypoint
 *          controller's throttle output calculations.
 * 
 * @param[in] speed Desired RTL speed in meters/second (m/s)
 * 
 * @return true if speed was successfully set, false if speed invalid
 * 
 * @note Speed must be non-negative (>= 0.0 m/s)
 * @note Maximum speed may be limited by other parameters (e.g., CRUISE_SPEED, SPEED_MAX)
 * @note Typically RTL uses RTL_SPEED parameter, but this allows external override
 * 
 * @see AR_WPNav::set_speed_max() for speed limiting and validation
 */
bool ModeRTL::set_desired_speed(float speed)
{
    return g2.wp_nav.set_speed_max(speed);
}
