/**
 * @file commands.cpp
 * @brief Home position management and mission command handling for ArduCopter
 * 
 * This file contains functions for setting and updating the vehicle's home position,
 * which is used as the return point for RTL (Return To Launch) operations. The home
 * position represents the takeoff location and is distinct from the EKF origin, which
 * serves as the mathematical reference point for the Extended Kalman Filter's coordinate
 * frame.
 * 
 * Key Concepts:
 * - **Home Position**: The physical location where the vehicle took off or was armed,
 *   stored in AHRS. This is where RTL will return the vehicle to.
 * - **EKF Origin**: The mathematical origin of the EKF's local NED (North-East-Down)
 *   coordinate frame, established when the EKF initializes. This provides a stable
 *   reference for all position estimates.
 * - **Home vs EKF Origin**: While often co-located, these serve different purposes.
 *   Home can be moved/updated by the user, while EKF origin is typically fixed once set.
 * 
 * Safety Considerations:
 * - Home position should not be changed while armed without careful consideration
 * - GPS lock quality affects home position accuracy
 * - EKF origin must be established before home can be set
 * - In-flight home updates use special logic to maintain consistency
 * 
 * @note All position operations require valid EKF origin and GPS lock
 * @warning Improper home position updates can cause RTL failures or unsafe navigation
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#include "Copter.h"

/**
 * @brief Check if AHRS home position should be updated from current EKF location
 * 
 * @details This function is called periodically to initialize the home position when
 *          it hasn't been set yet. The home position is the point where the vehicle
 *          will return during RTL (Return To Launch) operations.
 * 
 *          The function distinguishes between two scenarios:
 *          1. **Disarmed**: Sets home to current location (horizontal and vertical from EKF)
 *          2. **Armed (in-flight)**: Uses special in-flight logic that sets horizontal
 *             position from current GPS location but takes altitude from EKF origin to
 *             avoid sudden altitude changes mid-flight
 * 
 *          This function exits immediately if home is already set, preventing unintended
 *          home position changes during normal operation.
 * 
 * @note Called from the scheduler at regular intervals (typically main loop rate)
 * @note Requires valid EKF origin to be established before home can be set
 * @warning Only updates home when not yet set; does not override user-specified home
 * 
 * @see set_home_to_current_location()
 * @see set_home_to_current_location_inflight()
 * @see set_home()
 */
void Copter::update_home_from_EKF()
{
    // Exit immediately if home already set - prevents unintended home position changes
    // during flight or after user has manually set home. Home persistence ensures RTL
    // returns to the original takeoff/arming location.
    if (ahrs.home_is_set()) {
        return;
    }

    // Special logic if home is being set while armed (in-flight home initialization)
    // This can occur if the vehicle was armed before GPS lock was sufficient for
    // home initialization. Uses altitude from EKF origin rather than current altitude
    // to avoid sudden altitude target changes.
    if (motors->armed()) {
        set_home_to_current_location_inflight();
    } else {
        // Standard home initialization while disarmed
        // Sets home to current EKF location (horizontal and vertical position)
        // The 'false' parameter means home is not locked and can be updated by user commands
        // This will transition home_state to HOME_SET once successful
        if (!set_home_to_current_location(false)) {
            // Silently ignore failure - will retry on next scheduler call
            // Failure typically occurs when EKF doesn't have valid position estimate yet
        }
    }
}

/**
 * @brief Set home position during flight using current GPS horizontally and EKF origin vertically
 * 
 * @details This specialized function handles the case where home needs to be initialized
 *          while the vehicle is already armed and airborne. This scenario can occur when:
 *          - Vehicle was armed before GPS lock was sufficient
 *          - EKF initialized after takeoff
 *          - GPS signal was lost and then recovered
 * 
 *          **Key Behavior**: To prevent sudden altitude target changes that could destabilize
 *          the vehicle, this function:
 *          1. Takes the horizontal position (latitude/longitude) from current EKF location
 *          2. Takes the vertical position (altitude) from the EKF origin instead of current altitude
 *          3. Sets home to this composite location
 * 
 *          **EKF Origin vs Current Altitude**: The EKF origin altitude represents the
 *          reference altitude where the EKF coordinate frame was initialized. Using this
 *          instead of current altitude ensures that if RTL is triggered immediately after
 *          home is set, the vehicle won't attempt to climb or descend to match the altitude
 *          difference.
 * 
 *          If successful, also initializes SmartRTL home position for intelligent return
 *          path planning.
 * 
 * @note Only called when motors are armed (vehicle is in flight)
 * @note Requires both valid EKF position estimate and EKF origin
 * @warning Failure to use EKF origin altitude could cause sudden altitude changes
 * 
 * @see set_home()
 * @see set_home_to_current_location()
 * @see update_home_from_EKF()
 */
void Copter::set_home_to_current_location_inflight() {
    // Get current location from EKF (latitude, longitude, and current altitude)
    Location temp_loc;
    // Get EKF origin (the reference point for the local NED coordinate frame)
    Location ekf_origin;
    
    // Both position estimate and EKF origin must be valid to proceed
    // get_location() returns false if EKF doesn't have valid position estimate
    // get_origin() returns false if EKF origin hasn't been established yet
    if (ahrs.get_location(temp_loc) && ahrs.get_origin(ekf_origin)) {
        // CRITICAL: Replace current altitude with EKF origin altitude
        // This prevents the vehicle from trying to climb/descend to match a new home altitude
        // if RTL is triggered immediately after home initialization. The horizontal position
        // (lat/lon) remains the current GPS location.
        temp_loc.copy_alt_from(ekf_origin);
        
        // Set AHRS home position with the composite location
        // The 'false' parameter means home is not locked and can be updated by user commands
        if (!set_home(temp_loc, false)) {
            // Set home failed - typically because EKF origin not established
            // Return without setting SmartRTL home
            return;
        }
        
        // Successfully set AHRS home position, now initialize SmartRTL home
        // SmartRTL tracks the path taken and plans an intelligent return route
#if MODE_SMARTRTL_ENABLED
        g2.smart_rtl.set_home(true);
#endif
    }
}

/**
 * @brief Set home position to current GPS location (typically called while disarmed)
 * 
 * @details This is the standard method for initializing the home position before flight.
 *          It sets home to the complete current EKF location estimate (latitude, longitude,
 *          and altitude). This function is typically called:
 *          - During pre-arm initialization
 *          - When the vehicle first obtains GPS lock
 *          - When the pilot manually commands "set home to current location"
 * 
 *          Unlike set_home_to_current_location_inflight(), this function uses the full
 *          current altitude rather than EKF origin altitude, which is appropriate when
 *          the vehicle is on the ground.
 * 
 *          **Home Locking**: The lock parameter determines whether the home position
 *          can be changed by subsequent MAVLink commands or user input:
 *          - lock=false: Home can be updated by user (typical for automatic initialization)
 *          - lock=true: Home is locked and cannot be changed (used when pilot explicitly
 *            sets home)
 * 
 *          If successful, also initializes SmartRTL home position for intelligent return
 *          path planning.
 * 
 * @param[in] lock If true, prevents home from being changed by subsequent commands;
 *                 if false, allows home updates via MAVLink or other methods
 * 
 * @return true if home position was successfully set, false if failed
 * @retval false Typically indicates EKF doesn't have valid position estimate yet
 * 
 * @note Typically called while disarmed during pre-flight initialization
 * @note Requires valid EKF position estimate and established EKF origin
 * @warning Setting home without GPS lock may result in inaccurate RTL position
 * 
 * @see set_home()
 * @see set_home_to_current_location_inflight()
 * @see update_home_from_EKF()
 */
bool Copter::set_home_to_current_location(bool lock) {
    // Get complete current location from EKF (latitude, longitude, altitude)
    // This includes the full current altitude, which is appropriate when vehicle
    // is on the ground. Uses EKF position estimate which fuses GPS, barometer,
    // and other sensors.
    Location temp_loc;
    
    // Verify EKF has valid position estimate before proceeding
    // Returns false if EKF hasn't converged or doesn't have sufficient sensor data
    if (ahrs.get_location(temp_loc)) {
        // Attempt to set AHRS home position with the current location
        // This will also verify that EKF origin has been established
        if (!set_home(temp_loc, lock)) {
            // Home set failed - typically because EKF origin not established yet
            return false;
        }
        
        // Successfully set AHRS home position, now initialize SmartRTL home
        // SmartRTL records the flight path and uses it to plan an intelligent
        // return route that follows a safe known path instead of a direct line
#if MODE_SMARTRTL_ENABLED
        g2.smart_rtl.set_home(true);
#endif
        return true;
    }
    
    // EKF position estimate not available - cannot set home yet
    // This is normal during early initialization before GPS lock
    return false;
}

/**
 * @brief Set AHRS home position to a specified location
 * 
 * @details This is the low-level function that actually sets the home position in AHRS
 *          (Attitude and Heading Reference System). It is called by the higher-level
 *          home initialization functions after they prepare an appropriate location.
 * 
 *          **Critical Requirement**: The EKF origin MUST be established before home can
 *          be set. The EKF origin serves as the mathematical reference point for the local
 *          NED (North-East-Down) coordinate frame used by the Extended Kalman Filter. This
 *          origin is typically set when the EKF first obtains sufficient GPS data.
 * 
 *          **Home vs EKF Origin Distinction**:
 *          - **EKF Origin**: Mathematical reference point for position estimation. Fixed
 *            once established. Provides a stable local coordinate system for all navigation.
 *            Usually set automatically when EKF initializes with good GPS data.
 *          - **Home Position**: Operational reference point stored in AHRS. This is where
 *            RTL (Return To Launch) will return the vehicle. Can be updated by pilot or
 *            during flight initialization. May differ from EKF origin.
 * 
 *          **Home Locking**: When locked, the home position cannot be changed by MAVLink
 *          DO_SET_HOME commands or other user inputs. This prevents accidental home changes
 *          that could cause the vehicle to RTL to an incorrect location.
 * 
 * @param[in] loc  The location to set as home (latitude, longitude, altitude in Location struct)
 * @param[in] lock If true, locks home position preventing further changes via MAVLink;
 *                 if false, allows home to be updated by user commands
 * 
 * @return true if home was successfully set in AHRS, false if operation failed
 * @retval false Typically indicates EKF origin has not been established yet
 * 
 * @note This is the core function called by all home-setting operations
 * @note Does not set SmartRTL home - caller's responsibility if needed
 * @warning Home position is critical for RTL safety - incorrect home can cause flyaways
 * @warning EKF origin must be valid before calling this function
 * 
 * @see set_home_to_current_location()
 * @see set_home_to_current_location_inflight()
 * @see ahrs.set_home()
 * @see ahrs.get_origin()
 * @see ahrs.lock_home()
 */
bool Copter::set_home(const Location& loc, bool lock)
{
    // CRITICAL PRE-CHECK: Verify EKF origin has been established
    // The EKF origin is the mathematical reference point for the local NED coordinate
    // frame used by the Extended Kalman Filter. It must be set before home position
    // can be initialized. Without a valid EKF origin, the navigation system cannot
    // properly relate GPS coordinates to local positions.
    Location ekf_origin;
    if (!ahrs.get_origin(ekf_origin)) {
        // EKF origin not established yet - typically occurs during early boot before
        // GPS lock is obtained. Home initialization will be retried automatically.
        return false;
    }

    // Set the AHRS home position to the specified location
    // This home position is stored in AHRS and used as the target for RTL (Return To Launch)
    // operations. AHRS maintains this separately from the EKF origin to allow operational
    // flexibility (e.g., pilot can set home to a different location than takeoff point).
    if (!ahrs.set_home(loc)) {
        // Home set failed in AHRS - rare failure case, possibly due to invalid location
        return false;
    }

    // Optionally lock the home position to prevent changes
    // When locked, MAVLink DO_SET_HOME commands and other user inputs cannot change
    // the home position. This is used when the pilot explicitly sets a home location
    // and wants to ensure it won't be accidentally changed during flight.
    if (lock) {
        ahrs.lock_home();
    }

    // Home successfully set in AHRS
    // Note: Caller is responsible for setting SmartRTL home if needed
    return true;
}
