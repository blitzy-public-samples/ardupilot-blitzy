/**
 * @file commands.cpp
 * @brief Home position management and initialization for Blimp vehicle
 * 
 * @details This file implements home position setup and management for the Blimp autopilot.
 *          The home position serves as the reference point for Return-to-Launch (RTL) operations
 *          and establishes the origin for the NED (North-East-Down) coordinate frame used by
 *          the Extended Kalman Filter (EKF) for navigation.
 * 
 *          Key responsibilities:
 *          - Initialize home position from EKF location (update_home_from_EKF)
 *          - Set home position manually with validation (set_home)
 *          - Handle in-flight home position updates (set_home_to_current_location_inflight)
 *          - Manage NED frame origin relationship to home position
 * 
 *          The home position can be set from either GPS data or EKF estimates, with special
 *          handling for armed vs disarmed states. The EKF origin must be established before
 *          setting home position to ensure proper coordinate frame initialization.
 * 
 *          Home position is critical for:
 *          - RTL (Return to Launch) flight mode navigation
 *          - NED coordinate frame reference for position estimation
 *          - Failsafe return point in case of communication loss
 *          - Initial position reference for mission planning
 * 
 * @note Home position is typically set on first GPS lock or can be updated manually via GCS
 * @warning Incorrect home position will cause RTL to navigate to wrong location
 * 
 * Source: Blimp/commands.cpp
 */

#include "Blimp.h"

/**
 * @brief Check and update AHRS home position from EKF estimate
 * 
 * @details This function attempts to initialize the home position from the current EKF
 *          position estimate if home has not already been set. It is typically called
 *          during initialization and periodically during pre-arm checks to establish
 *          the home position as soon as a valid position estimate is available.
 * 
 *          Behavior varies based on armed state:
 *          - Disarmed: Sets home to current EKF location (both horizontal and vertical)
 *          - Armed: Sets home to current GPS location horizontally, EKF origin altitude
 * 
 *          The function exits immediately if home is already set to avoid inadvertent
 *          home position changes during flight operations.
 * 
 *          This establishes the origin for the NED (North-East-Down) coordinate frame
 *          used throughout the navigation system. The EKF must have a valid origin
 *          before home can be set.
 * 
 * @note Called periodically during pre-arm checks and initialization sequence
 * @note Home position is only set once unless explicitly commanded to update
 * @note RTL mode relies on this home position for return navigation
 * 
 * @see set_home_to_current_location()
 * @see set_home_to_current_location_inflight()
 * @see set_home()
 * 
 * Source: Blimp/commands.cpp:4-20
 */
void Blimp::update_home_from_EKF()
{
    // exit immediately if home already set
    if (ahrs.home_is_set()) {
        return;
    }

    // special logic if home is set in-flight
    if (motors->armed()) {
        set_home_to_current_location_inflight();
    } else {
        // move home to current ekf location (this will set home_state to HOME_SET)
        if (!set_home_to_current_location(false)) {
            // ignore failure
        }
    }
}

/**
 * @brief Set home position to current location during flight (armed state)
 * 
 * @details This function sets the home position while the vehicle is armed and motors are
 *          running. It uses a hybrid approach to ensure safe home position setting:
 *          - Horizontal position (latitude/longitude): Current EKF location estimate
 *          - Vertical position (altitude): EKF origin altitude (not current altitude)
 * 
 *          The altitude is intentionally set to the EKF origin rather than current altitude
 *          to prevent RTL from attempting to climb/descend to the in-flight altitude where
 *          home was set. This ensures RTL returns to a ground-level reference point.
 * 
 *          This function is typically called when:
 *          - Pilot commands home position update via GCS during flight
 *          - Automatic home update is triggered by mission commands
 *          - Special flight modes require home position relocation
 * 
 *          The function requires both valid EKF location and origin before proceeding.
 *          If either is unavailable, the home position is not updated.
 * 
 * @note Only called when motors are armed (in-flight)
 * @note Uses EKF origin altitude to avoid setting home at flight altitude
 * @note Silently returns if location or origin data is unavailable
 * 
 * @warning In-flight home updates affect RTL return point - use with caution
 * @warning Does not lock home position - home can still be updated later
 * 
 * @see update_home_from_EKF()
 * @see set_home()
 * @see AP_AHRS::get_location()
 * @see AP_AHRS::get_origin()
 * 
 * Source: Blimp/commands.cpp:23-34
 */
void Blimp::set_home_to_current_location_inflight()
{
    // get current location from EKF
    Location temp_loc;
    Location ekf_origin;
    if (ahrs.get_location(temp_loc) && ahrs.get_origin(ekf_origin)) {
        temp_loc.alt = ekf_origin.alt;
        if (!set_home(temp_loc, false)) {
            return;
        }
    }
}

/**
 * @brief Set home position to current EKF location estimate
 * 
 * @details This function sets the home position to the current location as estimated by
 *          the EKF (Extended Kalman Filter). It retrieves the current position estimate
 *          including latitude, longitude, and altitude, then attempts to set it as the
 *          home position through the core set_home() function.
 * 
 *          This is the standard method for initializing home position when the vehicle
 *          is disarmed. It uses the complete EKF location estimate (all three dimensions)
 *          unlike the in-flight version which uses EKF origin altitude.
 * 
 *          The function will fail and return false if:
 *          - EKF does not have a valid location estimate
 *          - EKF origin has not been established
 *          - AHRS home setting fails for any reason
 * 
 *          Typical usage scenarios:
 *          - Initial home position setup on GPS lock
 *          - Manual home position update via GCS command
 *          - Home position reset after moving vehicle on ground
 * 
 * @param[in] lock If true, prevents further home position updates until explicitly unlocked.
 *                 If false, allows home position to be updated again.
 * 
 * @return true if home position was successfully set, false otherwise
 * 
 * @note Typically called when vehicle is disarmed
 * @note Requires valid EKF location estimate and established EKF origin
 * @note Lock parameter determines if home position can be changed later
 * 
 * @see update_home_from_EKF()
 * @see set_home()
 * @see AP_AHRS::get_location()
 * 
 * Source: Blimp/commands.cpp:37-48
 */
bool Blimp::set_home_to_current_location(bool lock)
{
    // get current location from EKF
    Location temp_loc;
    if (ahrs.get_location(temp_loc)) {
        if (!set_home(temp_loc, lock)) {
            return false;
        }
        return true;
    }
    return false;
}

/**
 * @brief Set AHRS home position to specified location with validation
 * 
 * @details This is the core home position setting function that establishes the home position
 *          used for Return-to-Launch (RTL) operations and other navigation functions. It
 *          performs validation checks before setting the home position to ensure the EKF
 *          coordinate frame is properly initialized.
 * 
 *          The function validates that:
 *          1. EKF origin has been established (required for NED frame reference)
 *          2. AHRS successfully accepts the home position
 * 
 *          If validation fails, the function returns false and home position is not changed.
 *          
 *          NED Frame Relationship:
 *          The home position is related to the EKF origin which defines the origin of the
 *          NED (North-East-Down) coordinate frame. The EKF origin is typically set when
 *          the first GPS fix is obtained. Home position can be at or near this origin,
 *          but they are distinct:
 *          - EKF origin: Fixed reference point for NED coordinate frame (set once)
 *          - Home position: RTL return point (can be updated if not locked)
 * 
 *          GPS vs EKF Sources:
 *          The location parameter can come from either:
 *          - Direct GPS measurement (raw GPS position)
 *          - EKF position estimate (fused sensor data including GPS)
 *          The EKF estimate is generally preferred as it incorporates multiple sensors
 *          and provides better accuracy than raw GPS alone.
 * 
 *          Locking Behavior:
 *          When locked, the home position cannot be changed without explicitly unlocking.
 *          This prevents inadvertent home position changes during flight operations.
 * 
 * @param[in] loc Location to set as home position (WGS84 lat/lon/alt)
 * @param[in] lock If true, locks home position to prevent further updates.
 *                 If false, allows home position to be updated again later.
 * 
 * @return true if home position was successfully set and validated, false if:
 *         - EKF origin has not been established
 *         - AHRS rejects the home position
 *         - Location is invalid or out of range
 * 
 * @note EKF origin MUST be set before calling this function
 * @note This function is called by other home-setting convenience functions
 * @note Home position is critical for RTL mode - incorrect home causes navigation errors
 * @note First call may initialize inertial navigation and compass systems
 * 
 * @warning Setting incorrect home position will cause RTL to navigate to wrong location
 * @warning Ensure EKF has converged and GPS has good accuracy before setting home
 * @warning Locked home position cannot be updated without explicit unlock command
 * 
 * @see update_home_from_EKF()
 * @see set_home_to_current_location()
 * @see set_home_to_current_location_inflight()
 * @see AP_AHRS::set_home()
 * @see AP_AHRS::get_origin()
 * @see AP_AHRS::lock_home()
 * 
 * Source: Blimp/commands.cpp:53-73
 */
bool Blimp::set_home(const Location& loc, bool lock)
{
    // check EKF origin has been set
    Location ekf_origin;
    if (!ahrs.get_origin(ekf_origin)) {
        return false;
    }

    // set ahrs home (used for RTL)
    if (!ahrs.set_home(loc)) {
        return false;
    }

    // lock home position
    if (lock) {
        ahrs.lock_home();
    }

    // return success
    return true;
}
