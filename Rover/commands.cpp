/**
 * @file commands.cpp
 * @brief Home position management for Rover vehicle
 * 
 * @details This file implements home position setting and updating functionality
 *          for ground vehicles. The home position serves as:
 *          - Return-to-launch (RTL) destination
 *          - Reference point for relative navigation
 *          - Origin for local coordinate frame transformations
 *          - Starting point for SmartRTL path recording
 *          
 *          Home position can be set explicitly via MAVLink commands or
 *          automatically updated while disarmed based on GPS position.
 *          The home altitude is updated with barometer calibration.
 *          
 *          Coordinate system: WGS84 geodetic (latitude/longitude in degrees * 1e7,
 *          altitude in centimeters above mean sea level)
 * 
 * @note Home position updates are suppressed when position is explicitly locked
 * @warning Incorrect home position affects RTL behavior and could cause
 *          the vehicle to drive to an unexpected location
 * 
 * @see AP_Mission for mission command execution
 * @see mode_rtl.cpp for return-to-launch implementation
 * @see AP_AHRS for attitude and home reference system
 */

#include "Rover.h"

/**
 * @brief Set AHRS home position to current vehicle location
 * 
 * @details Retrieves current position from inertial navigation system and sets
 *          it as the AHRS home position. Also initializes SmartRTL path recording
 *          origin. This is typically called during arming or when explicitly
 *          commanded via MAVLink DO_SET_HOME.
 *          
 *          The function uses inertial navigation if available (EKF-based position),
 *          providing more accurate home positioning than raw GPS.
 *          
 *          Sequence:
 *          1. Check if inertial nav is available
 *          2. Get current location from AHRS
 *          3. Set as home position (optionally locking it)
 *          4. Initialize SmartRTL home for path recording
 * 
 * @param[in] lock  If true, prevents automatic home position updates while disarmed.
 *                  Used when pilot explicitly sets home via ground station command.
 * 
 * @return true if home position successfully set, false if no valid position available
 * 
 * @note Requires valid inertial navigation solution (EKF converged)
 * @note SmartRTL home is always set when this function succeeds
 * 
 * @see set_home() for the underlying home setting implementation
 * @see update_home() for automatic home position updates
 */
bool Rover::set_home_to_current_location(bool lock)
{
    Location temp_loc;
    
    // Verify inertial navigation is available and retrieve current position
    // Inertial nav provides EKF-fused position (more accurate than raw GPS)
    if (ahrs.have_inertial_nav() && ahrs.get_location(temp_loc)) {
        // Set retrieved location as home position
        if (!set_home(temp_loc, lock)) {
            return false;
        }
        
        // Initialize SmartRTL home position for intelligent return path recording
        // SmartRTL builds a breadcrumb trail from home for efficient return
        g2.smart_rtl.set_home(true);
        return true;
    }
    
    // No valid inertial navigation solution available
    return false;
}

/**
 * @brief Set AHRS home position to specified location
 * 
 * @details Sets the home position to an arbitrary location, typically provided
 *          via MAVLink command (DO_SET_HOME) or loaded from parameters on boot.
 *          Optionally locks the home position to prevent automatic updates.
 *          
 *          The home position is stored in AHRS and used by:
 *          - RTL mode for return destination
 *          - Fence system for home-relative boundaries
 *          - Navigation system for local frame origin
 *          - Telemetry for distance-to-home calculations
 *          
 *          Ground station notification is sent via MAVLink SYS_STATUS text message.
 * 
 * @param[in] loc   Location structure containing:
 *                  - lat: Latitude in degrees * 1e7 (WGS84)
 *                  - lng: Longitude in degrees * 1e7 (WGS84)
 *                  - alt: Altitude in centimeters above mean sea level
 * @param[in] lock  If true, home position is locked and will not be automatically
 *                  updated by update_home(). If false, home can drift with GPS
 *                  while disarmed.
 * 
 * @return true if home successfully set in AHRS, false if AHRS rejected location
 * 
 * @note Ground control stations receive MAV_SEVERITY_INFO notification with
 *       human-readable coordinates (decimal degrees and meters)
 * @note Locked home persists across disarm/arm cycles until explicitly changed
 * 
 * @see set_home_to_current_location() for setting home to vehicle position
 * @see update_home() for automatic home updates while disarmed
 */
bool Rover::set_home(const Location& loc, bool lock)
{
    // Attempt to set home position in AHRS (attitude and home reference system)
    // AHRS validates the location and manages home-relative transformations
    if (!ahrs.set_home(loc)) {
        return false;  // Location rejected by AHRS (invalid coordinates or AHRS not initialized)
    }

    // Lock home position if requested to prevent automatic updates
    // Locked home is typically used when pilot explicitly sets home via GCS
    if (lock) {
        ahrs.lock_home();
    }

    // Notify all connected ground control stations of new home position
    // Coordinates converted from internal format to human-readable decimal degrees and meters
    // lat/lng: 1e7 scaling factor converts to decimal degrees
    // alt: 0.01 scaling factor converts centimeters to meters
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Set HOME to %.6f %.6f at %.2fm",
            static_cast<double>(loc.lat * 1.0e-7f),
            static_cast<double>(loc.lng * 1.0e-7f),
            static_cast<double>(loc.alt * 0.01f));

    return true;
}

/**
 * @brief Automatically update home position to current vehicle location while disarmed
 * 
 * @details Called periodically by the main scheduler while vehicle is disarmed.
 *          Gradually updates home position to track GPS drift or intentional
 *          vehicle repositioning before flight. This ensures RTL returns to the
 *          actual launch location even if the vehicle was moved after initial GPS lock.
 *          
 *          Update logic:
 *          1. Skip if home is explicitly locked (pilot-commanded home)
 *          2. Skip if no valid position available (GPS not fixed)
 *          3. Update barometer calibration (home altitude reference)
 *          4. Skip if vehicle hasn't moved significantly (< DISTANCE_HOME_MINCHANGE)
 *          5. Update home to current position
 *          
 *          This automatic updating prevents stale home positions from GPS drift
 *          but is suppressed during flight or when home is explicitly set.
 * 
 * @note Only operates while vehicle is disarmed
 * @note Suppressed when home position is locked via DO_SET_HOME command
 * @note Barometer calibration updates on every call regardless of position change
 * @note Home updates require minimum movement threshold (DISTANCE_HOME_MINCHANGE)
 *       to prevent excessive EEPROM wear and spurious updates from GPS noise
 * 
 * @warning Must not be called while armed - could cause RTL destination to move
 *          during flight
 * 
 * @see DISTANCE_HOME_MINCHANGE in defines.h for minimum movement threshold
 * @see set_home() for the underlying home update implementation
 * @see set_home_to_current_location() for explicit home setting
 */
void Rover::update_home()
{
    // Check if home position is locked (explicitly set by pilot/GCS)
    // Locked home takes precedence over automatic updates
    if (ahrs.home_is_locked()) {
        // Home was explicitly set via DO_SET_HOME or similar command
        // Respect pilot intent and do not automatically update
        return;
    }

    // Retrieve current vehicle position from AHRS
    Location loc{};
    if (!ahrs.get_location(loc)) {
        // No valid position available (GPS not fixed or EKF not converged)
        return;
    }

    // Update barometer calibration using current altitude as reference
    // This adjusts barometer zero-point to match GPS altitude at home position
    // Called on every update_home() invocation to track pressure changes
    barometer.update_calibration();

    // Check if vehicle has moved significantly from current home position
    // Prevents excessive updates from GPS noise/drift and reduces EEPROM wear
    if (ahrs.home_is_set() &&
        loc.get_distance(ahrs.get_home()) < DISTANCE_HOME_MINCHANGE) {
        // Vehicle hasn't moved enough to warrant home position update
        // DISTANCE_HOME_MINCHANGE typically 0.5-2 meters
        return;
    }

    // Update home position to current location
    // Return value intentionally ignored - if update fails, we'll retry next iteration
    // Failure is rare and typically indicates AHRS not fully initialized
    IGNORE_RETURN(ahrs.set_home(loc));
}
