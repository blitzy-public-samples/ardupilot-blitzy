#pragma once

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
 * @file AP_AHRS_External.h
 * @brief External AHRS (Attitude Heading Reference System) backend for ArduPilot
 * 
 * @details This file implements the External AHRS backend which allows ArduPilot
 *          to use attitude, heading, position, and velocity estimates from an
 *          external navigation system rather than computing them internally.
 *          
 *          The external source provides pre-computed state estimates via MAVLink
 *          or other communication protocols. This backend acts as a passthrough,
 *          forwarding external estimates to the ArduPilot flight control system.
 *          
 *          Common use cases:
 *          - Integration with external INS/GPS systems
 *          - Vision-based navigation systems
 *          - Research platforms with custom state estimators
 *          
 * @note This backend depends entirely on the external system for state estimation.
 *       If the external system fails or provides invalid data, the vehicle will
 *       have no fallback unless another AHRS backend is available.
 * 
 * @warning External AHRS is safety-critical. The external system must provide
 *          accurate, timely data or vehicle control will be compromised.
 */

#include "AP_AHRS_config.h"

#if AP_AHRS_EXTERNAL_ENABLED

#include "AP_AHRS_Backend.h"

/**
 * @class AP_AHRS_External
 * @brief AHRS backend that uses attitude and position estimates from an external source
 * 
 * @details AP_AHRS_External implements the AHRS backend interface for external navigation
 *          systems. Instead of computing attitude, heading, position, and velocity from
 *          sensor data, this backend receives pre-computed state estimates from an
 *          external source (typically via MAVLink VISION_POSITION_ESTIMATE or similar).
 *          
 *          Key characteristics:
 *          - Passive backend: Does not perform sensor fusion or state estimation
 *          - Depends on external system for all navigation data
 *          - Supports position, velocity, and attitude from external source
 *          - Used for integration with external INS, vision systems, or motion capture
 *          
 *          Coordinate frame conventions:
 *          - Position/velocity in NED (North-East-Down) earth-fixed frame
 *          - Attitude as quaternion rotation from NED to body frame (Forward-Right-Down)
 *          - Body frame: X-Forward, Y-Right, Z-Down
 *          
 *          Units:
 *          - Position: meters
 *          - Velocity: m/s
 *          - Attitude: quaternion (unit magnitude)
 * 
 * @warning This backend provides no redundancy. If external data is lost or invalid,
 *          the vehicle will lose navigation capability unless a fallback backend exists.
 * 
 * @see AP_AHRS_Backend
 * @see AP_AHRS
 */
class AP_AHRS_External : public AP_AHRS_Backend {
public:

    using AP_AHRS_Backend::AP_AHRS_Backend;

    /* Do not allow copies */
    CLASS_NO_COPY(AP_AHRS_External);

    /**
     * @brief Reset the current gyro drift estimate (no-op for external AHRS)
     * 
     * @details This method is called when gyro offsets are recalculated. For external
     *          AHRS, gyro drift estimation is handled by the external system, so this
     *          is a no-op implementation.
     * 
     * @note This override satisfies the AP_AHRS_Backend interface but performs no action
     *       since the external system manages its own gyro bias estimation.
     */
    void reset_gyro_drift() override {}

    /**
     * @brief Check if the external AHRS backend has been initialized
     * 
     * @details Returns true if the external AHRS source is active and providing data.
     *          Initialization is considered complete when valid external state estimates
     *          have been received.
     * 
     * @return true if external AHRS is initialized and receiving data
     * @return false if no external data has been received or system is not ready
     */
    bool            initialised() const override;

    /**
     * @brief Update the AHRS state from external source
     * 
     * @details Called periodically (typically at main loop rate) to update internal state
     *          from the external navigation system. This method processes incoming external
     *          attitude, position, and velocity data and makes it available to the flight
     *          control system.
     * 
     * @note This method should be called at regular intervals to maintain current state.
     *       The external system is responsible for providing timely updates.
     */
    void            update() override;

    /**
     * @brief Populate the Estimates structure with current external AHRS data
     * 
     * @details Fills the provided Estimates structure with the latest attitude, position,
     *          and velocity data received from the external source. All data is in NED
     *          (North-East-Down) earth-fixed frame for position/velocity, with attitude
     *          represented as rotation from NED to body frame.
     * 
     * @param[out] results Estimates structure to populate with current state data
     * 
     * @note Position and velocity are in NED frame with units of meters and m/s respectively
     * @note Attitude quaternion represents rotation from NED earth frame to body frame (XYZ)
     */
    void            get_results(Estimates &results) override;

    /**
     * @brief Reset the AHRS state (no-op for external AHRS)
     * 
     * @details For external AHRS, state reset is managed by the external system.
     *          This no-op implementation satisfies the backend interface.
     * 
     * @note State management is handled externally; this method performs no action
     */
    void            reset() override {}

    /**
     * @brief Get wind estimation vector (not supported by external AHRS)
     * 
     * @details External AHRS backend does not currently support wind estimation.
     *          Wind estimation must be performed by another subsystem if required.
     * 
     * @param[out] ret Wind vector in NED frame (unused, always returns false)
     * 
     * @return false Wind estimation not available from external source
     * 
     * @note Units would be m/s if implemented
     * @note This could be extended if external system provides wind estimates
     */
    bool wind_estimate(Vector3f &ret) const override {
        return false;
    }

    /**
     * @brief Get ground velocity vector estimate from external source
     * 
     * @details Returns the horizontal ground velocity as a 2D vector in North/East order.
     *          Ground velocity is the vehicle's velocity relative to the ground, derived
     *          from the external navigation system's velocity estimate.
     * 
     * @return Vector2f Ground velocity in North/East order, units: m/s
     * 
     * @note Coordinate order is North (x), East (y)
     * @note Positive North is increasing latitude, positive East is increasing longitude
     */
    Vector2f groundspeed_vector() override;

    /**
     * @brief Check if compass should be used for heading (returns true for compatibility)
     * 
     * @details This method indicates whether compass data should be used for heading
     *          estimation. For external AHRS, heading comes from the external source,
     *          not from compass. However, this returns true for compatibility.
     * 
     * @return true Indicates compass usage flag (legacy compatibility)
     * 
     * @note This is actually never called at the moment; DCM backend's return value is used
     * @note External AHRS uses externally-provided heading, not compass data
     */
    bool            use_compass() override {
        // this is actually never called at the moment; we use dcm's
        // return value.
        return true;
    }

    /**
     * @brief Get the attitude quaternion from external AHRS source
     * 
     * @details Returns the quaternion representing vehicle attitude as provided by the
     *          external navigation system. The quaternion defines the rotation from the
     *          NED (North-East-Down) earth-fixed frame to the body frame (XYZ: Forward-Right-Down).
     *          
     *          Coordinate frames:
     *          - NED frame: North (X), East (Y), Down (Z) earth-fixed
     *          - Body frame: Forward (X), Right (Y), Down (Z) vehicle-fixed
     * 
     * @param[out] quat Quaternion representing rotation from NED to body frame
     * 
     * @return true if valid quaternion is available from external source
     * @return false if no valid attitude data is available
     * 
     * @note Quaternion should have unit magnitude
     * @note Body frame convention: X-Forward, Y-Right, Z-Down
     */
    bool get_quaternion(Quaternion &quat) const override WARN_IF_UNUSED;

    /**
     * @brief Run wind estimation algorithm (placeholder for external AHRS)
     * 
     * @details This method is called to update wind estimation. For external AHRS,
     *          wind estimation is typically not performed as the external system
     *          provides state estimates but not environmental parameters.
     * 
     * @note Currently a placeholder; external systems do not typically provide wind data
     * @note Could be extended to estimate wind from groundspeed vs airspeed if available
     */
    void estimate_wind(void);

    /**
     * @brief Check if the external AHRS subsystem is healthy
     * 
     * @details Returns true if the external AHRS backend is receiving valid data and
     *          operating normally. Health checks include data freshness, validity of
     *          attitude/position estimates, and communication status with external source.
     * 
     * @return true if external AHRS is healthy and providing valid data
     * @return false if data is stale, invalid, or external source is not responding
     * 
     * @note A healthy status indicates the external system is providing timely, valid estimates
     * @warning Unhealthy status may trigger failsafe actions depending on vehicle configuration
     */
    bool healthy() const override;

    /**
     * @brief Get velocity vector in NED frame from external source
     * 
     * @details Returns the 3D velocity vector in the NED (North-East-Down) earth-fixed
     *          reference frame as provided by the external navigation system.
     *          
     *          Vector components:
     *          - vec.x: North velocity (positive = moving north)
     *          - vec.y: East velocity (positive = moving east)  
     *          - vec.z: Down velocity (positive = descending)
     * 
     * @param[out] vec Velocity vector in NED frame, units: m/s
     * 
     * @return true if valid velocity data is available from external source
     * @return false if no velocity data available or data is invalid
     * 
     * @note NED frame convention: North (X), East (Y), Down (Z)
     * @note Units: meters per second (m/s) for all components
     */
    bool get_velocity_NED(Vector3f &vec) const override;

    /**
     * @brief Get vertical velocity (Down) kinematically consistent with position
     * 
     * @details Returns the derivative of vertical position in m/s (Down positive).
     *          This vertical velocity is kinematically consistent with the vertical
     *          position estimate, as required by some control loops.
     *          
     *          This differs from typical EKF vertical velocity which may not be
     *          consistent with position due to error corrections being applied.
     *          For external AHRS, position and velocity come from the same source
     *          so consistency is maintained.
     * 
     * @param[out] velocity Vertical velocity in Down direction, units: m/s (positive = descending)
     * 
     * @return true if valid vertical velocity is available
     * @return false if no velocity data available
     * 
     * @note Positive values indicate downward motion (descending)
     * @note Kinematic consistency means velocity matches position derivative
     * @note Units: meters per second (m/s)
     */
    bool get_vert_pos_rate_D(float &velocity) const override;

    /**
     * @brief Perform pre-arm checks for external AHRS backend
     * 
     * @details Validates that external AHRS is ready for flight by checking:
     *          - External source is providing data
     *          - Attitude estimates are valid
     *          - Position estimates are valid (if requires_position is true)
     *          - Data is fresh and not stale
     *          - System health status is good
     *          
     *          If checks fail, a descriptive failure message is written to the buffer.
     * 
     * @param[in] requires_position true if horizontal position should be checked, false to skip position check
     * @param[out] failure_msg Buffer to receive failure message if checks fail
     * @param[in] failure_msg_len Length of failure_msg buffer in bytes
     * 
     * @return true if all pre-arm checks pass, vehicle is ready to arm
     * @return false if any check fails, failure_msg contains description
     * 
     * @note This is a critical safety check - vehicle should not arm if this returns false
     * @warning Bypassing failed pre-arm checks can result in loss of vehicle control
     */
    bool pre_arm_check(bool requires_position, char *failure_msg, uint8_t failure_msg_len) const override;

    /**
     * @brief Get the origin location for relative position calculations
     * 
     * @details Returns the geographic origin (reference point) used for relative position
     *          estimates. The origin is typically set when the external system initializes
     *          or when the vehicle first achieves a position fix. All relative positions
     *          are computed with respect to this origin.
     * 
     * @param[out] ret Location structure containing origin latitude, longitude, and altitude
     * 
     * @return true if origin has been established and is available
     * @return false if no origin has been set or external data unavailable
     * 
     * @note Origin is used as reference for AP_InertialNav fallback calculations
     * @note Origin remains fixed unless explicitly reset by external system
     */
    bool get_origin(Location &ret) const override;

    /**
     * @brief Get 3D position relative to origin in NED frame
     * 
     * @details Returns the vehicle's current position relative to the established origin
     *          as a 3D vector in the NED (North-East-Down) frame.
     *          
     *          Vector components:
     *          - vec.x: North offset from origin in meters (positive = north of origin)
     *          - vec.y: East offset from origin in meters (positive = east of origin)
     *          - vec.z: Down offset from origin in meters (positive = below origin)
     * 
     * @param[out] vec 3D position vector relative to origin in NED frame, units: meters
     * 
     * @return true if valid relative position is available
     * @return false if no position data or origin not established
     * 
     * @note Used for AP_InertialNav fallback when EKF is unavailable
     * @note NED frame: North (X), East (Y), Down (Z), all in meters
     */
    bool get_relative_position_NED_origin(Vector3p &vec) const override;

    /**
     * @brief Get 2D horizontal position relative to origin (North/East only)
     * 
     * @details Returns the vehicle's horizontal position relative to origin as a 2D vector
     *          containing only North and East components (Down component omitted).
     *          
     *          Vector components:
     *          - posNE.x: North offset from origin in meters
     *          - posNE.y: East offset from origin in meters
     * 
     * @param[out] posNE 2D position vector (North, East) relative to origin, units: meters
     * 
     * @return true if valid horizontal position is available
     * @return false if no position data or origin not established
     * 
     * @note Used when only horizontal position is needed
     * @note Units: meters for both North and East components
     */
    bool get_relative_position_NE_origin(Vector2p &posNE) const override;

    /**
     * @brief Get Down position relative to origin (altitude/depth only)
     * 
     * @details Returns only the Down component of position relative to origin.
     *          Positive values indicate the vehicle is below the origin (descending).
     *          
     *          For aerial vehicles: Positive = lower altitude than origin
     *          For underwater vehicles: Positive = deeper than origin
     * 
     * @param[out] posD Down position relative to origin, units: meters (positive = below origin)
     * 
     * @return true if valid vertical position is available
     * @return false if no position data or origin not established
     * 
     * @note Positive Down = below origin, negative Down = above origin
     * @note Units: meters
     * @note Kinematically consistent with get_vert_pos_rate_D() velocity
     */
    bool get_relative_position_D_origin(postype_t &posD) const override;

    /**
     * @brief Get the navigation filter status from external AHRS
     * 
     * @details Populates the nav_filter_status structure with the current operational
     *          status of the external navigation system. This includes flags for:
     *          - Attitude validity
     *          - Horizontal/vertical velocity validity
     *          - Horizontal/vertical position validity
     *          - Initialization state
     *          
     *          Status information is derived from the external source's health and
     *          data validity indicators.
     * 
     * @param[out] status Navigation filter status structure to populate
     * 
     * @return true if status information is available
     * @return false if unable to determine external system status
     * 
     * @note Status reflects external system's reported state, not internal filtering
     * @see nav_filter_status for complete structure definition
     */
    bool get_filter_status(nav_filter_status &status) const override;

    /**
     * @brief Send EKF status report via MAVLink to ground control station
     * 
     * @details Transmits navigation system status information to the ground control
     *          station via MAVLink protocol. For external AHRS, this reports the
     *          status of the external navigation source rather than internal EKF.
     *          
     *          Transmitted information includes:
     *          - System health flags
     *          - Position/velocity validity
     *          - Error estimates (if provided by external system)
     * 
     * @param[in] link MAVLink communication link to use for transmission
     * 
     * @note Message format follows standard EKF status reporting for GCS compatibility
     * @note External AHRS status is mapped to EKF status message format
     */
    void send_ekf_status_report(class GCS_MAVLINK &link) const override;

    /**
     * @brief Get control limits for navigation filter
     * 
     * @details Returns control system limits derived from the external navigation system:
     *          - ekfGndSpdLimit: Maximum safe ground speed for current conditions
     *          - controlScaleXY: Horizontal control scaling factor (0.0 to 1.0)
     *          
     *          Control scale is used to reduce control authority when navigation accuracy
     *          degrades. A scale of 1.0 indicates full control authority, lower values
     *          indicate reduced confidence and reduced control gains.
     * 
     * @param[out] ekfGndSpdLimit Maximum ground speed limit, units: m/s
     * @param[out] controlScaleXY Horizontal control scale factor (0.0 = no control, 1.0 = full control)
     * 
     * @note Ground speed limit helps prevent excessive speeds during poor navigation
     * @note Control scale factor is multiplied with position control gains
     * @note For external AHRS, limits may be derived from external system's confidence metrics
     */
    void get_control_limits(float &ekfGndSpdLimit, float &controlScaleXY) const override;
};

#endif
