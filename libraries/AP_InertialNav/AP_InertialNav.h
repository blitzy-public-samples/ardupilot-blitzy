/**
 * @file AP_InertialNav.h
 * @brief Inertial navigation wrapper providing normalized position and velocity outputs
 * 
 * This file defines the AP_InertialNav class, which serves as a wrapper around the
 * AP_AHRS and NavEKF systems to provide normalized inertial navigation outputs in
 * consistent coordinate frames and units. The class handles coordinate frame transformations
 * (NED to NEU) and unit conversions (meters to centimeters) for position and velocity
 * estimates derived from the Extended Kalman Filter (EKF).
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */
#pragma once

#include <AP_NavEKF/AP_Nav_Common.h>              // definitions shared by inertial and ekf nav filters

/**
 * @class AP_InertialNav
 * @brief Inertial navigation wrapper providing normalized navigation outputs from AHRS/EKF
 * 
 * @details AP_InertialNav serves as a lightweight wrapper around the AP_AHRS and NavEKF
 *          systems, providing vehicle position and velocity estimates in a consistent
 *          coordinate frame and unit system.
 * 
 *          Key responsibilities:
 *          - Wraps AP_AHRS/EKF position and velocity outputs
 *          - Converts coordinate frames from NED (North-East-Down) to NEU (North-East-Up)
 *          - Converts units from meters to centimeters for position
 *          - Converts units from m/s to cm/s for velocity
 *          - Provides fallback behavior during high vibration or EKF failures
 * 
 *          Coordinate Frame Convention:
 *          All outputs use NEU (North-East-Up) frame:
 *          - .x : North direction (positive = north)
 *          - .y : East direction (positive = east)
 *          - .z : Up direction (positive = up, away from Earth center)
 * 
 *          Integration with AHRS/EKF:
 *          The class queries AP_AHRS for EKF-derived position and velocity estimates,
 *          which are internally computed in NED frame and meters. This wrapper performs
 *          the necessary transformations for consistent usage throughout the flight stack.
 * 
 * @note This class does not perform state estimation itself; it delegates to AP_AHRS/EKF
 * @warning Position and velocity accuracy depends entirely on EKF health and sensor quality
 * 
 * @see AP_AHRS
 * @see AP_NavEKF2
 * @see AP_NavEKF3
 */
class AP_InertialNav
{
public:
    /**
     * @brief Constructor for AP_InertialNav
     * 
     * Initializes the inertial navigation wrapper with a reference to the
     * AP_AHRS system, which provides access to EKF-derived position and
     * velocity estimates.
     * 
     * @param[in] ahrs Reference to the AP_AHRS instance providing EKF estimates
     * 
     * @note The AHRS reference is stored for the lifetime of this object
     */
    AP_InertialNav(class AP_AHRS &ahrs) :
        _ahrs_ekf(ahrs)
        {}

    /**
     * @brief Update internal position and velocity estimates from AHRS/EKF
     * 
     * @details This method queries AP_AHRS for the latest EKF-derived position and
     *          velocity estimates and updates the internal state with appropriate
     *          coordinate frame and unit conversions.
     * 
     *          Update sequence:
     *          1. Query AP_AHRS for relative position NE (North-East) in meters
     *          2. Query AP_AHRS for relative position D (Down) in meters
     *          3. Convert position from meters to centimeters
     *          4. Convert position from NED frame to NEU frame (negate z-axis)
     *          5. Query AP_AHRS for velocity NED in m/s
     *          6. Convert velocity from m/s to cm/s
     *          7. Convert velocity from NED frame to NEU frame (negate z-axis)
     * 
     *          Fallback behavior:
     *          - During EKF velocity failures: Horizontal velocity is frozen at last
     *            good value; vertical velocity uses fallback estimate from get_vert_pos_rate_D()
     *          - During high vibration events: Vertical velocity uses fallback estimate
     *            while horizontal velocity continues to use EKF
     * 
     * @param[in] high_vibes Set to true during high vibration events to trigger
     *                       fallback vertical velocity estimation. Default is false.
     * 
     * @note This method is typically called at main navigation loop rate (e.g., 50Hz for Copter)
     * @note All position outputs are relative to the EKF origin set during initialization
     * 
     * @warning Accuracy depends entirely on EKF health and sensor quality
     * @warning During EKF failures, horizontal velocity will be frozen (not updated)
     * 
     * @see AP_AHRS::get_relative_position_NE_origin_float()
     * @see AP_AHRS::get_relative_position_D_origin_float()
     * @see AP_AHRS::get_velocity_NED()
     * @see AP_AHRS::get_vert_pos_rate_D()
     */
    void        update(bool high_vibes = false);

    /**
     * @brief Get current 3D position relative to EKF origin in NEU frame
     * 
     * Returns the current position estimate relative to the EKF origin in
     * North-East-Up coordinate frame with centimeter units.
     *
     * @return Position vector in NEU frame in centimeters:
     *         - .x : North position in cm (positive = north of origin)
     *         - .y : East position in cm (positive = east of origin)
     *         - .z : Up position in cm (positive = above origin)
     * 
     * @note Coordinate frame is NEU (North-East-Up), not NED
     * @note Updated by calling update() method
     */
    const Vector3f&    get_position_neu_cm() const;

    /**
     * @brief Get current horizontal position relative to EKF origin in NE frame
     * 
     * Returns the current horizontal (North-East) position estimate relative to
     * the EKF origin in centimeters, excluding the vertical component.
     *
     * @return Horizontal position vector in NE frame in centimeters:
     *         - .x : North position in cm (positive = north of origin)
     *         - .y : East position in cm (positive = east of origin)
     * 
     * @note Coordinate frame is North-East (horizontal plane only)
     * @note Updated by calling update() method
     */
    const Vector2f&    get_position_xy_cm() const;

    /**
     * @brief Get current vertical position relative to EKF origin (z-up convention)
     * 
     * Returns the current vertical position estimate relative to the EKF origin
     * using z-up convention (positive = above origin) in centimeters.
     *
     * @return Vertical position in cm with z-up convention (positive = up/above origin)
     * 
     * @note Uses z-up convention: positive values indicate altitude above EKF origin
     * @note Updated by calling update() method
     */
    float              get_position_z_up_cm() const;

    /**
     * @brief Get current 3D velocity in NEU frame
     * 
     * Returns the current velocity estimate in North-East-Up coordinate frame
     * with centimeters per second units.
     *
     * @return Velocity vector in NEU frame in cm/s:
     *         - .x : North velocity in cm/s (positive = moving north)
     *         - .y : East velocity in cm/s (positive = moving east)
     *         - .z : Up velocity in cm/s (positive = climbing/ascending)
     * 
     * @note Coordinate frame is NEU (North-East-Up), not NED
     * @note Units are cm/s (centimeters per second)
     * @note Updated by calling update() method
     * @warning During EKF failures, horizontal velocity (.x and .y) will be frozen at last good value
     */
    const Vector3f&    get_velocity_neu_cms() const;

    /**
     * @brief Get current horizontal velocity in NE frame
     * 
     * Returns the current horizontal (North-East) velocity estimate in
     * centimeters per second, excluding the vertical component.
     *
     * @return Horizontal velocity vector in NE frame in cm/s:
     *         - .x : North velocity in cm/s (positive = moving north)
     *         - .y : East velocity in cm/s (positive = moving east)
     * 
     * @note Coordinate frame is North-East (horizontal plane only)
     * @note Units are cm/s (centimeters per second)
     * @note Updated by calling update() method
     * @warning During EKF failures, this value will be frozen at last good estimate
     */
    const Vector2f& get_velocity_xy_cms() const;

    /**
     * @brief Get current horizontal ground speed (magnitude of horizontal velocity)
     * 
     * Returns the magnitude of the horizontal velocity vector (ground speed),
     * calculated as the length of the North-East velocity vector.
     *
     * @return Horizontal ground speed in cm/s (always positive, magnitude only)
     * 
     * @note This is the magnitude (length) of the horizontal velocity vector
     * @note Calculated as: sqrt(velocity_north^2 + velocity_east^2)
     * @note Units are cm/s (centimeters per second)
     * @note Updated by calling update() method
     * @warning During EKF failures, this value will reflect the frozen horizontal velocity
     */
    float        get_speed_xy_cms() const;

    /**
     * @brief Get current vertical velocity (z-up convention)
     * 
     * Returns the current vertical velocity estimate using z-up convention
     * (positive = climbing) in centimeters per second.
     *
     * @return Vertical velocity in cm/s with z-up convention:
     *         - Positive values indicate climbing/ascending
     *         - Negative values indicate descending
     * 
     * @note Uses z-up convention: positive = up/climbing, negative = down/descending
     * @note Units are cm/s (centimeters per second)
     * @note Updated by calling update() method
     * @note During high vibration or EKF failures, uses fallback estimate from get_vert_pos_rate_D()
     */
    float       get_velocity_z_up_cms() const;

private:
    /// Current position estimate relative to EKF origin in NEU frame, centimeters
    /// (.x = North in cm, .y = East in cm, .z = Up in cm)
    Vector3f _relpos_cm;
    
    /// Current velocity estimate in NEU frame, centimeters per second
    /// (.x = North velocity in cm/s, .y = East velocity in cm/s, .z = Up velocity in cm/s)
    Vector3f _velocity_cm;
    
    /// Reference to AP_AHRS instance providing EKF-derived position and velocity estimates
    AP_AHRS &_ahrs_ekf;
};
