/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * @file AP_Beacon_Backend.h
 * @brief Abstract backend interface for beacon positioning systems
 * 
 * This file defines the base class interface that all beacon positioning
 * system backends must implement. Beacon systems provide vehicle position
 * estimates based on distances to multiple fixed beacons with known positions.
 * 
 * Backends implement hardware-specific communication and data processing for
 * systems like Marvelmind, Pozyx, and other indoor positioning solutions.
 */

#pragma once

#include "AP_Beacon.h"

#if AP_BEACON_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>

/**
 * @class AP_Beacon_Backend
 * @brief Abstract base class for beacon positioning system backends
 * 
 * @details This class defines the interface contract that all beacon positioning
 *          system backends must implement. Beacon systems provide 3D position
 *          estimates by measuring distances to multiple fixed beacons with known
 *          positions, enabling indoor or GPS-denied positioning.
 * 
 *          Backend Lifecycle:
 *          1. Construction: Backend is instantiated with reference to frontend
 *          2. Update Loop: update() called periodically (typically 10-50 Hz)
 *          3. Position Reporting: Backend calls set_vehicle_position() or
 *             set_beacon_distance() when new measurements are available
 *          4. Health Monitoring: healthy() indicates data reception status
 * 
 *          Coordinate System Requirements:
 *          All positions must be reported in NED (North-East-Down) frame relative
 *          to the beacon system's local origin. Backends are responsible for
 *          coordinate transformations from hardware-specific frames.
 * 
 *          Supported beacon systems include:
 *          - Marvelmind indoor positioning
 *          - Pozyx UWB positioning
 *          - Custom UART-based beacon systems
 * 
 * @warning All position data MUST be in NED frame with distances in meters.
 *          Incorrect coordinate frames will cause position estimation failures.
 * 
 * @note Backends typically communicate via UART with external beacon hardware.
 */
class AP_Beacon_Backend
{
public:
    /**
     * @brief Construct a new beacon backend
     * 
     * @details Constructor initializes the backend and establishes the connection
     *          to the frontend AP_Beacon instance. Backends should perform minimal
     *          initialization here and defer hardware communication to update().
     * 
     * @param[in] frontend Reference to AP_Beacon frontend for data reporting
     */
    AP_Beacon_Backend(AP_Beacon &frontend);

    /**
     * @brief Check if beacon system is receiving valid data
     * 
     * @details Pure virtual method that backends must implement to report
     *          their operational health. Returns true if the backend is
     *          actively receiving position or distance data from the beacon
     *          hardware within expected timeframes.
     * 
     *          Typical healthy criteria:
     *          - Data received within last 1-2 seconds
     *          - Communication link established
     *          - Beacon hardware initialized successfully
     *          - Position/distance data passes validity checks
     * 
     * @return true if beacon system is operational and receiving data
     * @return false if no recent data, communication failure, or initialization incomplete
     * 
     * @note This method is called by the frontend to determine whether to use
     *       beacon data for position estimation
     */
    virtual bool healthy() = 0;

    /**
     * @brief Update beacon backend state and process incoming data
     * 
     * @details Pure virtual method called periodically (typically 10-50 Hz) by
     *          the scheduler. Backends should:
     *          - Read data from UART or other communication interface
     *          - Parse beacon protocol messages
     *          - Validate incoming position/distance measurements
     *          - Call set_vehicle_position() or set_beacon_distance() with new data
     *          - Update health status for healthy() check
     *          - Apply yaw orientation correction if needed
     * 
     *          This method should be non-blocking and return quickly to avoid
     *          impacting scheduler timing.
     * 
     * @note Called at main loop rate, typically 50 Hz
     * @see set_vehicle_position(), set_beacon_distance()
     */
    virtual void update() = 0;

    /**
     * @brief Report vehicle position from beacon system measurements
     * 
     * @details Backends call this method when a complete 3D position solution
     *          is available from the beacon system. The position is reported
     *          relative to the beacon system's local origin and integrated with
     *          other position sources by the Extended Kalman Filter (EKF).
     * 
     *          Position Coordinate System:
     *          - Frame: NED (North-East-Down) relative to beacon origin
     *          - Units: meters
     *          - Origin: Defined by beacon system configuration
     * 
     *          The accuracy estimate influences sensor fusion weighting in the EKF.
     *          Lower values indicate higher confidence and will be weighted more
     *          heavily in the position estimate.
     * 
     * @param[in] pos Vehicle position in meters in NED frame from beacon origin
     *                    - pos.x: North position in meters
     *                    - pos.y: East position in meters  
     *                    - pos.z: Down position in meters (positive = below origin)
     * @param[in] accuracy_estimate Position uncertainty in meters (1-sigma standard deviation)
     * 
     * @warning pos MUST be in NED frame with distances in meters. Incorrect
     *          coordinate frames will cause navigation failures.
     * 
     * @note This is typically called from update() when beacon system provides
     *       a position solution
     * @see set_beacon_distance() for range-only measurements
     */
    void set_vehicle_position(const Vector3f& pos, float accuracy_estimate);

    /**
     * @brief Report distance measurement to a specific beacon
     * 
     * @details Backends call this method to report individual range measurements
     *          to beacons with known positions. This allows the EKF to compute
     *          vehicle position through multilateration from multiple beacon
     *          distance measurements.
     * 
     *          This method is used when the beacon hardware provides raw distance
     *          measurements rather than a computed position solution. The frontend
     *          will use these distances along with beacon positions to estimate
     *          vehicle location.
     * 
     * @param[in] beacon_instance Zero-based index identifying which beacon (0-9)
     * @param[in] distance Straight-line distance from vehicle to beacon in meters
     * 
     * @note Beacon positions must be configured via parameters or set_beacon_position()
     *       before distance measurements can be used for positioning
     * @note Distance measurements are typically more robust than full position
     *       solutions for certain beacon hardware
     * @see set_beacon_position(), set_vehicle_position()
     */
    void set_beacon_distance(uint8_t beacon_instance, float distance);

    /**
     * @brief Set the known position of a beacon in the beacon coordinate system
     * 
     * @details Backends call this method to register the fixed positions of
     *          beacons in the positioning system. Beacon positions may be provided
     *          by the beacon hardware during initialization or configured via
     *          parameters. These positions are required for multilateration when
     *          using distance-only measurements.
     * 
     *          Beacon Position Coordinate System:
     *          - Frame: NED (North-East-Down) relative to beacon system origin
     *          - Units: meters
     *          - Origin: Common reference point for all beacons in the system
     * 
     *          Beacon positions are typically surveyed during installation or
     *          auto-calibrated by some beacon systems.
     * 
     * @param[in] beacon_instance Zero-based index identifying which beacon (0-9)
     * @param[in] pos Beacon position in meters in NED frame from beacon origin
     *                    - pos.x: North position in meters
     *                    - pos.y: East position in meters
     *                    - pos.z: Down position in meters (positive = below origin)
     * 
     * @warning pos MUST be in NED frame with distances in meters for correct
     *          position calculation from distance measurements
     * 
     * @note Some beacon systems transmit beacon positions automatically, others
     *       require manual configuration
     * @see set_beacon_distance()
     */
    void set_beacon_position(uint8_t beacon_instance, const Vector3f& pos);

    /**
     * @brief Get latitude of beacon system origin
     * 
     * @details Returns the GPS latitude of the beacon coordinate system origin.
     *          This allows integration of beacon positioning (local NED frame)
     *          with global GPS coordinates. The origin is typically set via
     *          parameters or beacon system configuration.
     * 
     * @return Latitude of beacon origin in degrees
     * 
     * @note Origin must be configured for beacon position to integrate with EKF
     */
    float get_beacon_origin_lat(void) const { return _frontend.origin_lat; }

    /**
     * @brief Get longitude of beacon system origin
     * 
     * @details Returns the GPS longitude of the beacon coordinate system origin.
     *          This allows integration of beacon positioning (local NED frame)
     *          with global GPS coordinates. The origin is typically set via
     *          parameters or beacon system configuration.
     * 
     * @return Longitude of beacon origin in degrees
     * 
     * @note Origin must be configured for beacon position to integrate with EKF
     */
    float get_beacon_origin_lon(void) const { return _frontend.origin_lon; }

    /**
     * @brief Get altitude of beacon system origin
     * 
     * @details Returns the altitude (AMSL - Above Mean Sea Level) of the beacon
     *          coordinate system origin. This allows integration of beacon
     *          positioning (local NED frame) with global altitude references.
     *          The origin is typically set via parameters or beacon system
     *          configuration.
     * 
     * @return Altitude of beacon origin in meters AMSL
     * 
     * @note Origin must be configured for beacon position to integrate with EKF
     */
    float get_beacon_origin_alt(void) const { return _frontend.origin_alt; }

protected:

    /**
     * @brief Reference to AP_Beacon frontend instance
     * 
     * @details Provides access to frontend data and methods for reporting
     *          position measurements, accessing parameters, and retrieving
     *          beacon origin coordinates. Backends use this to communicate
     *          measurement data back to the main AP_Beacon subsystem.
     */
    AP_Beacon &_frontend;

    /**
     * @brief Cached yaw orientation correction angle
     * 
     * @details Stores the beacon system orientation yaw parameter in degrees.
     *          This cached value avoids repeated parameter lookups. The yaw
     *          correction allows beacon systems to be mounted at arbitrary
     *          orientations relative to the vehicle's forward direction.
     * 
     *          Units: degrees
     *          Range: -180 to 180 degrees
     */
    int16_t orient_yaw_deg;

    /**
     * @brief Cached cosine of yaw orientation angle
     * 
     * @details Pre-computed cosine of the yaw orientation correction angle.
     *          This cached value optimizes coordinate rotation calculations
     *          in correct_for_orient_yaw() by avoiding repeated trigonometric
     *          function calls.
     * 
     * @see orient_yaw_deg, correct_for_orient_yaw()
     */
    float orient_cos_yaw = 0.0f;

    /**
     * @brief Cached sine of yaw orientation angle
     * 
     * @details Pre-computed sine of the yaw orientation correction angle.
     *          This cached value optimizes coordinate rotation calculations
     *          in correct_for_orient_yaw() by avoiding repeated trigonometric
     *          function calls.
     * 
     * @see orient_yaw_deg, correct_for_orient_yaw()
     */
    float orient_sin_yaw = 1.0f;

    /**
     * @brief Apply yaw orientation correction to a position vector
     * 
     * @details Rotates a position vector to correct for beacon system mounting
     *          orientation. If the beacon system is mounted with a yaw offset
     *          relative to the vehicle's forward direction, this method applies
     *          a rotation transformation to align the beacon coordinate frame
     *          with the vehicle frame.
     * 
     *          Rotation is applied in the horizontal plane (North-East) while
     *          preserving the Down component.
     * 
     *          Typical use case: Beacon system mounted sideways or backwards
     *          on vehicle requires coordinate transformation before reporting
     *          position to EKF.
     * 
     * @param[in] vector Position vector in beacon frame to be rotated
     * @return Position vector rotated to vehicle NED frame
     * 
     * @note Uses cached orient_cos_yaw and orient_sin_yaw for efficiency
     * @see orient_yaw_deg, orient_cos_yaw, orient_sin_yaw
     */
    Vector3f correct_for_orient_yaw(const Vector3f &vector);

    /**
     * @brief UART serial interface for beacon hardware communication
     * 
     * @details Pointer to UART driver instance used for serial communication
     *          with beacon positioning hardware. Backends use this to read
     *          incoming data packets and send configuration commands to the
     *          beacon system.
     * 
     *          Typical usage:
     *          - Read incoming position/distance messages
     *          - Parse hardware-specific protocols
     *          - Send initialization and configuration commands
     * 
     * @note UART parameters (baud rate, etc.) are typically configured via
     *       the SERIAL port parameters
     * @note May be null if backend uses alternative communication method
     */
    AP_HAL::UARTDriver *uart;
};

#endif  // AP_BEACON_ENABLED
