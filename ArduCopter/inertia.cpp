/**
 * @file inertia.cpp
 * @brief Inertial navigation state estimation and position tracking for ArduCopter
 * 
 * @details This file manages the integration of inertial navigation data from the
 *          Extended Kalman Filter (EKF) and Attitude Heading Reference System (AHRS)
 *          into the vehicle's position control and navigation systems.
 *          
 *          The term "inertia" in this context refers to inertial navigation - the
 *          process of calculating position and velocity by integrating accelerometer
 *          measurements over time, corrected with other sensors (GPS, barometer, etc.)
 *          through the EKF.
 *          
 *          Key responsibilities:
 *          - Update position control estimates from EKF/AHRS
 *          - Manage altitude coordinate frame conversions
 *          - Track current vehicle location for navigation
 *          - Handle high vibration scenarios affecting position estimates
 *          - Maintain follow mode position tracking
 *          
 *          Relationship to vehicle control:
 *          The position estimates updated here are used by:
 *          - Position controller (AC_PosControl) for maintaining setpoints
 *          - Navigation waypoint controller for mission execution
 *          - Flight modes requiring position hold or navigation
 *          - Altitude controller for vertical position management
 *          
 *          Coordinate frames:
 *          - EKF origin: The reference point where EKF was initialized
 *          - Home position: The location where vehicle was armed
 *          - NED frame: North-East-Down coordinate system used internally
 *          - Altitude frames: ABOVE_ORIGIN vs ABOVE_HOME (converted in this file)
 * 
 * @note This file does NOT manage physical inertia properties (mass, center of
 *       gravity, moment of inertia tensor). Those parameters are handled by the
 *       EKF and attitude controller libraries when configured.
 * 
 * @see AC_PosControl for position controller using these estimates
 * @see AP_AHRS for attitude and heading reference system
 * @see AP_NavEKF3 for Extended Kalman Filter state estimation
 * @see AP_InertialNav for inertial navigation wrapper
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#include "Copter.h"

/**
 * @brief Update inertial navigation estimates and current position from EKF/AHRS
 * 
 * @details This function serves as the main integration point between the vehicle's
 *          navigation filter (EKF) and the position control system. It performs
 *          three critical tasks:
 *          
 *          1. Position Control Estimate Updates:
 *             Updates the position controller with the latest position and velocity
 *             estimates from the EKF. During high vibration conditions, the position
 *             controller may adjust its trust in accelerometer-derived estimates and
 *             rely more heavily on barometric altitude data to prevent instability.
 *          
 *          2. Location Tracking:
 *             Retrieves the current latitude, longitude, and altitude from AHRS and
 *             stores them in current_loc for use throughout the vehicle code. This
 *             location is used for navigation, geofencing, mission execution, and
 *             telemetry reporting.
 *          
 *          3. Altitude Frame Conversion:
 *             Converts altitude from the EKF's native ABOVE_ORIGIN frame to the
 *             ABOVE_HOME frame used by most vehicle systems. This conversion is
 *             necessary because:
 *             - EKF origin is set at initialization (may not be home location)
 *             - Home position is set at arming (user's reference point)
 *             - Flight modes and missions typically reference home altitude
 *             - Fallback: If home not set, treats origin as home
 *          
 *          High Vibration Handling:
 *          When vibration_check.high_vibes is true, the position controller adjusts
 *          its fusion weights to reduce reliance on accelerometer integration, which
 *          becomes unreliable during high vibration. This prevents altitude loss or
 *          position drift during periods of mechanical vibration (e.g., aggressive
 *          maneuvers, structural resonance, motor imbalance).
 *          
 *          Coordinate Frame Details:
 *          - NED frame: North-East-Down, right-handed coordinate system
 *          - pos_d_m: Down position in meters (negative means above origin)
 *          - Conversion: pos_d_m * -100.0 → altitude in centimeters above origin
 *          - ABOVE_ORIGIN: Altitude relative to EKF initialization point
 *          - ABOVE_HOME: Altitude relative to arming location (home)
 *          
 *          Follow Mode Integration:
 *          If MODE_FOLLOW_ENABLED is configured, this function also updates position
 *          estimates for follow mode, which tracks and maintains position relative
 *          to a moving target vehicle.
 *          
 *          Early Exit Condition:
 *          If AHRS cannot provide a valid altitude estimate (e.g., EKF not initialized,
 *          no GPS fix, insufficient sensor data), the function exits early to prevent
 *          using invalid position data. In this case, current_loc retains only the
 *          lat/lon from AHRS but altitude is not updated.
 *          
 *          Relationship to Control Loop:
 *          This function is called at the main loop rate (typically 400Hz) to provide
 *          the freshest possible position estimates to the control system. The updated
 *          estimates feed into:
 *          - Altitude controller: Maintains vertical position setpoints
 *          - Position controller: Maintains horizontal position setpoints
 *          - Waypoint navigator: Tracks progress toward navigation targets
 *          - Geofence: Monitors boundary violations
 *          - Telemetry: Reports current position to ground station
 * 
 * @note Called at main loop rate (typically 400Hz for ArduCopter)
 * @note Function name is historical - "inertia" refers to inertial navigation,
 *       not physical mass or moment of inertia properties
 * 
 * @warning If this function fails to update estimates due to missing AHRS data,
 *          the vehicle will continue using stale position estimates which can lead
 *          to navigation errors, position drift, or altitude hold failures. The EKF
 *          health should be monitored via pre-arm checks and in-flight failsafes.
 * 
 * @see AC_PosControl::update_estimates() for how estimates are used in control
 * @see AP_AHRS::get_location() for location data source
 * @see AP_AHRS::get_relative_position_D_origin_float() for altitude estimation
 * @see Location::change_alt_frame() for coordinate frame conversion details
 * @see Copter::vibration_check for vibration detection system
 * 
 * Source: ArduCopter/inertia.cpp:4-31
 */
void Copter::read_inertia()
{
    // Update position controller with latest EKF position and velocity estimates.
    // During high vibrations (vibration_check.high_vibes = true), the position
    // controller adjusts fusion weights to rely more on barometric altitude and
    // less on accelerometer integration, preventing altitude drift caused by
    // vibration-induced accelerometer noise. This is critical for maintaining
    // stable altitude hold during aggressive maneuvers or mechanical resonance.
    // The position controller (AC_PosControl) uses these estimates for:
    // - Horizontal position control in LOITER, PosHold, AUTO, GUIDED modes
    // - Vertical position control in AltHold and position-controlled modes
    // - Velocity control for smooth waypoint navigation
    pos_control->update_estimates(vibration_check.high_vibes);
    
#if MODE_FOLLOW_ENABLED
    // Update position estimates for follow mode if compiled in.
    // Follow mode tracks a moving target vehicle (e.g., another copter, rover, or
    // boat) and maintains a relative position offset. The follow system needs
    // updated position estimates to calculate the relative position error and
    // generate appropriate velocity commands to maintain formation.
    g2.follow.update_estimates();
#endif

    // Retrieve current vehicle location (latitude/longitude) from AHRS.
    // The AHRS system fuses GPS, compass, and inertial data to provide the
    // best estimate of the vehicle's position. This location is used throughout
    // the vehicle code for:
    // - Navigation: Distance and bearing to waypoints
    // - Geofencing: Boundary violation detection
    // - Mission execution: Progress tracking and next waypoint selection
    // - Telemetry: Position reporting to ground control station
    // - Safety features: Return-to-launch position calculations
    // Note: We only copy lat/lon here; altitude is handled separately below
    // to perform necessary coordinate frame conversions.
    Location loc;
    ahrs.get_location(loc);
    current_loc.lat = loc.lat;
    current_loc.lng = loc.lng;

    // Attempt to retrieve altitude estimate from AHRS/EKF.
    // pos_d_m = Down position in meters in NED frame, relative to EKF origin
    // In NED frame: Negative values indicate above origin, positive below origin
    // If altitude estimate is unavailable (EKF not initialized, no GPS fix,
    // insufficient sensors), exit immediately to avoid using invalid altitude data.
    // This can occur during:
    // - Initial boot before EKF converges
    // - GPS signal loss with no other altitude sources
    // - EKF failsafe conditions
    // When this fails, current_loc retains only lat/lon; altitude is not updated.
    float pos_d_m;
    if (!AP::ahrs().get_relative_position_D_origin_float(pos_d_m)) {
        return;
    }

    // Convert altitude from NED frame Down coordinate to altitude above EKF origin.
    // Coordinate frame transformation:
    // - pos_d_m: Down position in meters (NED frame, negative = above origin)
    // - Multiply by -1: Convert Down to Up direction
    // - Multiply by 100: Convert meters to centimeters (ArduPilot internal unit)
    // Result: Altitude in centimeters above the EKF origin point
    // 
    // Example: If vehicle is 10m above EKF origin:
    //   pos_d_m = -10.0 (negative in NED Down)
    //   alt_above_origin_cm = -(-10.0) * 100 = 1000 cm above origin
    const int32_t alt_above_origin_cm = -pos_d_m * 100.0;
    
    // Store altitude in ABOVE_ORIGIN frame initially
    current_loc.set_alt_cm(alt_above_origin_cm, Location::AltFrame::ABOVE_ORIGIN);
    
    // Convert from ABOVE_ORIGIN frame to ABOVE_HOME frame for user reference.
    // This conversion is necessary because:
    // - EKF origin: Set at EKF initialization (first good position fix)
    // - Home position: Set when vehicle arms (user's takeoff location)
    // - These two points may differ if vehicle moved between boot and arming
    // - Most flight modes, missions, and user displays reference home altitude
    // 
    // The change_alt_frame() function performs the conversion by:
    // 1. Calculating the altitude difference between home and origin
    // 2. Adjusting the altitude value to reference home instead of origin
    // 3. Accounting for terrain altitude changes if using terrain following
    // 
    // Fallback behavior:
    // If home is not set (pre-arm) OR conversion fails (shouldn't happen but
    // defensive programming), treat the origin altitude as home altitude.
    // This provides valid altitude data even before arming, with the understanding
    // that "home" isn't established yet. Once home is set at arming, subsequent
    // calls will use proper ABOVE_HOME frame.
    if (!ahrs.home_is_set() || !current_loc.change_alt_frame(Location::AltFrame::ABOVE_HOME)) {
        // Pre-arming state: Home position not yet established, or conversion failed.
        // Use origin-relative altitude and label it as home-relative.
        // This allows altitude displays and controllers to function before arming,
        // treating the current EKF origin as a temporary "home" reference.
        // Once vehicle arms and home is set, future calls will use true ABOVE_HOME.
        current_loc.set_alt_cm(alt_above_origin_cm, Location::AltFrame::ABOVE_HOME);
    }
    
    // At this point, current_loc contains:
    // - lat/lon: Current GPS position from AHRS
    // - alt: Current altitude in ABOVE_HOME frame (user's reference)
    // This location is now available to all vehicle systems for navigation,
    // control, geofencing, telemetry, and safety functions.
}
