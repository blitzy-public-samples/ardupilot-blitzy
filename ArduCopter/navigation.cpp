/**
 * @file navigation.cpp
 * @brief Navigation helper functions for position and home location calculations
 * 
 * @details This file implements fundamental navigation calculations used throughout
 *          the ArduCopter flight control system:
 *          
 *          - Home position distance and bearing calculations with caching
 *          - Super simple mode bearing updates for pilot orientation reference
 *          - Position estimation validation and error checking
 *          
 *          These functions integrate with:
 *          - AP_AHRS: Provides attitude, position, and home location data
 *          - Location class: Geographic coordinate calculations in NED frame
 *          - Position estimation: EKF/GPS-based position validity checking
 *          
 *          All distance calculations use the NED (North-East-Down) coordinate frame.
 *          
 * @note Navigation calculations are cached to avoid redundant computation across
 *       multiple calls per control loop iteration.
 * 
 * @warning These calculations depend on valid position estimates. Always verify
 *          position_ok() returns true before relying on distance/bearing values.
 */

#include "Copter.h"

/**
 * @brief Top-level navigation update for autopilot calculations
 * 
 * @details This function serves as the primary entry point for navigation-related
 *          updates that must be executed before autopilot decision-making. Currently
 *          responsible for updating the super simple mode bearing reference.
 *          
 *          Super simple mode provides a simplified flight experience where pilot
 *          stick inputs are translated relative to the vehicle's takeoff orientation
 *          rather than the current heading. This function updates the bearing
 *          reference used for that translation.
 *          
 *          Called by: Main scheduler at navigation update rate (typically 50Hz)
 *          
 *          Integration points:
 *          - Scheduler: Registered as periodic task in scheduler_tasks
 *          - Flight modes: All modes that support super simple mode
 *          
 * @note Function name and location are marked for refactoring (see To-Do below).
 *       Consider renaming to reflect super simple mode specificity.
 * 
 * @note This function is called at main loop rate before mode-specific updates
 *       to ensure navigation state is current for all flight mode calculations.
 * 
 * @todo Rename and move this function to better reflect its purpose and improve
 *       code organization. Consider moving to dedicated super_simple.cpp or
 *       integrating into mode-specific update paths.
 * 
 * @see update_super_simple_bearing() for bearing calculation implementation
 * @see Copter::scheduler_tasks for call frequency and timing
 */
void Copter::run_nav_updates(void)
{
    // Update super simple mode bearing reference for pilot-oriented control
    // Parameter 'false' indicates this is not a forced update
    update_super_simple_bearing(false);
}

/**
 * @brief Calculate and cache distance from current position to home location
 * 
 * @details Computes the horizontal distance between the vehicle's current position
 *          and the home location in the NED (North-East-Down) reference frame.
 *          
 *          Distance calculation process:
 *          1. Verify position estimate is valid via position_ok() check
 *          2. Retrieve current vehicle location from EKF/GPS (current_loc)
 *          3. Retrieve home location from AHRS (ahrs.get_home())
 *          4. Calculate great circle distance using Location::get_distance()
 *          5. Convert from meters to centimeters and cache result
 *          
 *          Caching strategy:
 *          - Distance is recalculated only when position_ok() returns true
 *          - Cached value (_home_distance) is returned when position invalid
 *          - This prevents returning zero during temporary GPS glitches
 *          - Cache provides last known good distance for display/logic
 *          
 *          Position validity requirements (position_ok() checks):
 *          - EKF healthy and providing position estimates
 *          - GPS fix available or alternative position source active
 *          - Position estimate uncertainty within acceptable thresholds
 *          
 *          Coordinate frame:
 *          - Calculations performed in earth-fixed NED frame
 *          - Altitude difference not included (horizontal distance only)
 *          - Uses WGS-84 ellipsoid for geographic calculations
 *          
 *          Used by:
 *          - RTL mode: Distance monitoring for return to launch
 *          - Failsafe logic: Trigger geofence and distance-based failsafes
 *          - GCS telemetry: Display home distance to pilot
 *          - Simple mode: Reference for pilot-oriented control
 *          
 * @return Distance to home location in centimeters
 * 
 * @note Returns cached value if position estimate is invalid, providing
 *       last known good distance rather than zero or invalid data
 * 
 * @note Distance is horizontal only (2D) - does not include altitude difference.
 *       For 3D distance, use get_distance_NED() with vertical component.
 * 
 * @note Unit is centimeters for consistency with ArduCopter's internal position
 *       representations. GCS telemetry converts to meters for display.
 * 
 * @warning Cached value may be stale during GPS outages or EKF failures.
 *          Check position_ok() separately for position validity before using
 *          this value for navigation or failsafe decisions.
 * 
 * @see home_bearing() for direction to home location
 * @see position_ok() for position estimate validity checking
 * @see Location::get_distance() for distance calculation algorithm
 * @see AP_AHRS::get_home() for home location source
 */
uint32_t Copter::home_distance()
{
    // Only recalculate distance when position estimate is valid
    // This prevents returning zero during temporary GPS glitches
    if (position_ok()) {
        // Get great circle distance between current position and home
        // Location::get_distance() returns distance in meters
        // Multiply by 100 to convert meters to centimeters for internal use
        _home_distance = current_loc.get_distance(ahrs.get_home()) * 100;
    }
    
    // Return cached distance (either newly calculated or previous value)
    // Returning cached value during position loss provides last known distance
    // rather than zero, which is better for display and some logic
    return _home_distance;
}

/**
 * @brief Calculate and cache bearing from current position to home location
 * 
 * @details Computes the compass bearing (azimuth) from the vehicle's current
 *          position to the home location in the NED (North-East-Down) frame.
 *          
 *          Bearing calculation process:
 *          1. Verify position estimate is valid via position_ok() check
 *          2. Retrieve current vehicle location from EKF/GPS (current_loc)
 *          3. Retrieve home location from AHRS (ahrs.get_home())
 *          4. Calculate forward azimuth using Location::get_bearing_to()
 *          5. Cache result in centidegrees for internal use
 *          
 *          Bearing convention:
 *          - 0° = North (0 centidegrees)
 *          - 90° = East (9000 centidegrees)
 *          - 180° = South (18000 centidegrees)
 *          - 270° = West (27000 centidegrees)
 *          - Range: 0 to 35999 centidegrees (0° to 359.99°)
 *          - Increases clockwise when viewed from above (NED convention)
 *          
 *          Caching strategy:
 *          - Bearing is recalculated only when position_ok() returns true
 *          - Cached value (_home_bearing) is returned when position invalid
 *          - This prevents returning zero during temporary GPS glitches
 *          - Cache provides last known good bearing for display/logic
 *          
 *          Position validity requirements (position_ok() checks):
 *          - EKF healthy and providing position estimates
 *          - GPS fix available or alternative position source active
 *          - Position estimate uncertainty within acceptable thresholds
 *          
 *          Coordinate frame:
 *          - Bearing calculated in earth-fixed NED frame
 *          - True north reference (not magnetic north)
 *          - Forward azimuth from current position to home
 *          - Uses geodetic calculations for curved earth surface
 *          
 *          Used by:
 *          - RTL mode: Navigation heading for return to launch
 *          - Simple mode: Pilot-oriented control frame reference
 *          - GCS telemetry: Display home direction to pilot
 *          - Heads-up displays: Home arrow/indicator pointing
 *          - Yaw control: Automatic yaw alignment during RTL
 *          
 * @return Bearing to home location in centidegrees (0-35999, 0° to 359.99°)
 * 
 * @note Returns cached value if position estimate is invalid, providing
 *       last known good bearing rather than zero or invalid data
 * 
 * @note Bearing is true north referenced, not magnetic north. Add/subtract
 *       declination if magnetic heading is needed for compass displays.
 * 
 * @note Unit is centidegrees (degrees * 100) for precision and consistency
 *       with ArduCopter's internal angle representations. GCS telemetry
 *       converts to degrees for display.
 * 
 * @note Bearing calculation accounts for earth curvature using geodetic math,
 *       making it accurate for long distances. For very short distances
 *       (< 1 meter), bearing may be noisy due to GPS precision limits.
 * 
 * @warning Cached value may be stale during GPS outages or EKF failures.
 *          Check position_ok() separately for position validity before using
 *          this value for navigation or autopilot heading control.
 * 
 * @warning Bearing becomes undefined when vehicle is directly over home
 *          location (zero horizontal distance). Implementation returns the
 *          last valid bearing in this case.
 * 
 * @see home_distance() for distance to home location
 * @see position_ok() for position estimate validity checking
 * @see Location::get_bearing_to() for bearing calculation algorithm
 * @see AP_AHRS::get_home() for home location source
 */
int32_t Copter::home_bearing()
{
    // Only recalculate bearing when position estimate is valid
    // This prevents returning zero during temporary GPS glitches
    if (position_ok()) {
        // Get forward azimuth from current position to home location
        // Location::get_bearing_to() returns bearing in centidegrees (0-35999)
        // Bearing is true north referenced in NED frame (0° = North, clockwise)
        _home_bearing = current_loc.get_bearing_to(ahrs.get_home());
    }
    
    // Return cached bearing (either newly calculated or previous value)
    // Returning cached value during position loss provides last known bearing
    // rather than zero, which is better for display and some logic
    return _home_bearing;
}
