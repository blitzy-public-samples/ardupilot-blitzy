/**
 * @file commands.cpp
 * @brief Mission command initialization and home position management for ArduSub
 * 
 * @details This file handles mission command setup and home position management
 *          for underwater vehicles. It provides functions to initialize and update
 *          the home position used for Return-to-Launch (RTL) operations.
 *          
 *          Key underwater-specific behaviors:
 *          - Home position is always set at the water surface, not at depth
 *          - Allows disarming and arming at any depth without losing surface reference
 *          - Ensures mission items with relative altitude are relative to water surface
 *          - Works in both high-elevation lakes and at sea level
 *          
 *          Integration with AP_Mission:
 *          - Home position serves as reference point for mission commands
 *          - Mission altitude frames (relative/absolute) are processed relative to home
 *          - Home position must be set before mission execution can begin
 *          
 * @note This implementation is specific to underwater vehicles and differs from
 *       other ArduPilot vehicle types in how altitude/depth is handled
 * 
 * @see AP_Mission for mission command processing
 * @see Sub::set_home() for the core home setting implementation
 * 
 * Source: ArduSub/commands.cpp:1-76
 */

#include "Sub.h"

/**
 * @brief Checks if home position should be updated from EKF and performs update
 * 
 * @details This function is called periodically to establish the home position
 *          if it hasn't been set yet. It uses the Extended Kalman Filter (EKF)
 *          position estimate as the source for the home location.
 *          
 *          The function implements different logic depending on whether the vehicle
 *          is armed (in operation) or disarmed (on surface):
 *          - If disarmed: Sets home to current EKF location at water surface
 *          - If armed: Uses special in-flight logic to preserve depth reference
 *          
 *          This is typically called from the main scheduler loop during initialization
 *          or when GPS lock is first acquired.
 * 
 * @note Returns immediately if home is already set - only sets home once
 * @note Does not return status; failures are silently ignored to prevent blocking
 * 
 * @see Sub::set_home_to_current_location() for disarmed home setting
 * @see Sub::set_home_to_current_location_inflight() for armed home setting
 * @see ahrs.home_is_set() for home status checking
 * 
 * Source: ArduSub/commands.cpp:4-20
 */
void Sub::update_home_from_EKF()
{
    // Exit immediately if home already set - only initialize home once
    // This prevents overwriting an intentionally set home position
    if (ahrs.home_is_set()) {
        return;
    }

    // Use different home setting logic based on vehicle state:
    // - Armed (in operation): Preserve depth reference using in-flight method
    // - Disarmed (on surface): Set home to surface position using standard method
    if (motors.armed()) {
        // Vehicle is armed (motors running) - use special in-flight logic
        // This preserves the EKF origin altitude to maintain depth reference
        set_home_to_current_location_inflight();
    } else {
        // Vehicle is disarmed - set home to current EKF location at water surface
        // This will set home_state to HOME_SET in the AHRS
        // Pass false for lock parameter to allow future home updates if needed
        if (!set_home_to_current_location(false)) {
            // Home setting failed (no GPS fix or EKF not ready)
            // Failure is silently ignored - will retry on next call
            // This prevents blocking vehicle initialization if GPS is not yet available
        }
    }
}

/**
 * @brief Set home to current GPS location (horizontally) and EKF origin vertically
 * 
 * @details This function is specifically designed for setting home position while
 *          the vehicle is already armed and potentially submerged. It preserves
 *          the EKF origin altitude to maintain consistent depth reference.
 *          
 *          The function combines:
 *          - Horizontal position (lat/lon) from current EKF location
 *          - Vertical position (alt) from EKF origin (typically surface)
 *          
 *          This prevents the home altitude from being set to the current depth,
 *          which would cause mission altitude references to be incorrect.
 *          
 *          Called by: update_home_from_EKF() when motors are armed
 * 
 * @note This function does not return status; failures are silently ignored
 * @note Only called when vehicle is armed (motors running)
 * 
 * @warning Do not use this function when disarmed - use set_home_to_current_location() instead
 * 
 * @see Sub::set_home() for the underlying home setting mechanism
 * @see Sub::update_home_from_EKF() for caller logic
 * 
 * Source: ArduSub/commands.cpp:23-34
 */
void Sub::set_home_to_current_location_inflight()
{
    // Get current horizontal position from EKF
    Location temp_loc;
    Location ekf_origin;
    
    // Retrieve both current location and EKF origin
    // Both must be available for this operation to proceed
    if (ahrs.get_location(temp_loc) && ahrs.get_origin(ekf_origin)) {
        // Override current altitude with EKF origin altitude
        // This ensures home altitude is at the original surface reference,
        // not at the current depth where the vehicle may be operating
        // Preserves consistent altitude reference for mission items and RTL
        temp_loc.alt = ekf_origin.alt;
        
        // Set home with the modified location (surface altitude, current horizontal position)
        // Pass false for lock parameter to allow future home updates if needed
        if (!set_home(temp_loc, false)) {
            // Home setting failed - silently ignore
            // Failure is non-critical during armed operation
            // Most likely cause: AHRS not accepting location for some reason
        }
    }
    // If location or origin unavailable, silently return without setting home
    // This is not an error condition - simply means position not yet available
}

/**
 * @brief Set home position to current GPS location at water surface
 * 
 * @details This function sets the home position using the current EKF location,
 *          but adjusts the altitude to always be at the water surface, regardless
 *          of the vehicle's current depth.
 *          
 *          Underwater-Specific Altitude Handling:
 *          The function subtracts the current barometric altitude (depth) from
 *          the EKF altitude to calculate the surface position:
 *          
 *          surface_alt = ekf_alt - (barometer_alt * 100.0)
 *          
 *          where barometer_alt is in meters and converted to centimeters.
 *          
 *          This ensures:
 *          - Vehicle can disarm and rearm at depth without changing home altitude
 *          - Mission waypoints with relative altitude are always relative to surface
 *          - Consistent behavior in lakes at elevation or at sea level
 *          - RTL operations return to surface, not to depth where home was set
 *          
 *          Typical Usage:
 *          - Called during initialization when vehicle is on surface
 *          - Called from update_home_from_EKF() when disarmed
 *          - Called when user commands home position reset
 * 
 * @param[in] lock If true, prevents further changes to home position until unlock
 *                 If false, home can be updated by subsequent calls
 * 
 * @return true if home location set successfully
 * @return false if EKF position unavailable or home setting failed
 * 
 * @note Home altitude is in centimeters in ArduPilot convention
 * @note Barometer altitude is negative when submerged (below surface)
 * 
 * @see Sub::set_home() for the core home setting implementation
 * @see barometer.get_altitude() returns depth in meters (negative = submerged)
 * 
 * Source: ArduSub/commands.cpp:37-51
 */
bool Sub::set_home_to_current_location(bool lock)
{
    // Get current location from EKF (Extended Kalman Filter)
    // This provides the best estimate of vehicle position
    Location temp_loc;
    if (ahrs.get_location(temp_loc)) {

        // Make home always at the water's surface.
        // This allows disarming and arming again at depth.
        // This also ensures that mission items with relative altitude frame, are always
        // relative to the water's surface, whether in a high elevation lake, or at sea level.
        
        // Adjust altitude to surface level by subtracting current depth
        // barometer.get_altitude() returns altitude in meters (negative when submerged)
        // Multiply by 100.0 to convert meters to centimeters (ArduPilot altitude convention)
        // Subtracting negative depth effectively adds to altitude, placing home at surface
        temp_loc.alt -= barometer.get_altitude() * 100.0f;
        
        // Set the adjusted location as home with optional lock parameter
        return set_home(temp_loc, lock);
    }
    
    // Return false if EKF location is unavailable (no GPS fix or EKF not initialized)
    return false;
}

/**
 * @brief Set AHRS home position to specified location
 * 
 * @details This is the core home setting function used by all other home position
 *          functions in ArduSub. It sets the AHRS (Attitude and Heading Reference
 *          System) home position which serves as the reference point for:
 *          - Return-to-Launch (RTL) operations
 *          - Mission waypoints with relative altitude frame
 *          - Position hold operations
 *          - Distance calculations for geofencing
 *          
 *          The function performs validation to ensure the EKF origin has been
 *          established before allowing home to be set. This prevents setting
 *          home with an invalid reference frame.
 *          
 *          Home Lock Feature:
 *          If the lock parameter is true, the home position is locked and cannot
 *          be changed until explicitly unlocked. This prevents accidental home
 *          position changes during critical operations.
 *          
 *          Integration with AP_AHRS:
 *          This function delegates to the AHRS subsystem for the actual home
 *          position storage and management. The AHRS provides the home position
 *          to navigation and mission systems throughout the codebase.
 * 
 * @param[in] loc  The location to set as home position
 *                 - loc.lat: Latitude in degrees * 1e7
 *                 - loc.lng: Longitude in degrees * 1e7
 *                 - loc.alt: Altitude in centimeters above reference
 * @param[in] lock If true, locks home position to prevent further changes
 *                 If false, home can be updated by subsequent calls
 * 
 * @return true if home location set successfully
 * @return false if EKF origin not set or AHRS home setting failed
 * 
 * @note This function requires EKF origin to be initialized before use
 * @note Home position is stored in AHRS and accessible throughout ArduPilot
 * @note Locked home prevents accidental changes during mission execution
 * 
 * @warning Always ensure loc.alt is correctly set for underwater operations
 *          (typically at water surface, not at depth)
 * 
 * @see Sub::set_home_to_current_location() for automatic home setting
 * @see ahrs.set_home() for underlying AHRS implementation
 * @see ahrs.lock_home() for home position locking mechanism
 * 
 * Source: ArduSub/commands.cpp:170-190
 */
bool Sub::set_home(const Location& loc, bool lock)
{
    // Check if EKF origin has been established
    // The EKF origin is the reference point for the local coordinate frame
    // and must be set before home position can be meaningfully defined
    Location ekf_origin;
    if (!ahrs.get_origin(ekf_origin)) {
        // EKF origin not yet initialized - cannot set home without valid reference
        return false;
    }

    // Set AHRS home position (used for RTL and mission planning)
    // This stores the home location in the AHRS subsystem where it's
    // accessible to all navigation and control systems
    if (!ahrs.set_home(loc)) {
        // Home setting failed - AHRS rejected the location
        // This can occur if the location is invalid or AHRS is not ready
        return false;
    }

    // Lock home position if requested to prevent accidental changes
    // Locking is useful during mission execution or critical operations
    // where home position must remain stable
    if (lock) {
        ahrs.lock_home();
    }

    // Home successfully set (and optionally locked)
    return true;
}
