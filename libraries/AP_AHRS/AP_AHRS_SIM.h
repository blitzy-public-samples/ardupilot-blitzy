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
 * @file AP_AHRS_SIM.h
 * @brief SITL simulation AHRS backend providing perfect attitude knowledge for testing
 * 
 * @details This file implements the AP_AHRS_SIM class, which serves as a Software-In-The-Loop
 *          (SITL) simulation backend for the Attitude Heading Reference System. Unlike real AHRS
 *          backends that fuse noisy sensor data, this backend provides perfect attitude, position,
 *          and velocity information directly from the SITL physics simulation for algorithm testing
 *          and validation.
 *          
 *          The SITL backend bypasses actual sensor fusion and instead returns ideal state estimates
 *          from SITL::sitl_fdm (Flight Dynamics Model), enabling developers to test control algorithms
 *          and navigation logic without sensor noise, delays, or failures.
 * 
 * @note This backend is only available when AP_AHRS_SIM_ENABLED is defined (SITL builds)
 * @warning SITL perfect knowledge does not represent real-world sensor characteristics, timing delays,
 *          or failure modes. Algorithms must still be validated on actual hardware with real sensors.
 */

#include "AP_AHRS_config.h"

#if AP_AHRS_SIM_ENABLED

#include "AP_AHRS_Backend.h"

#include <GCS_MAVLink/GCS.h>
#include <SITL/SITL.h>

#if HAL_NAVEKF3_AVAILABLE
#include <AP_NavEKF3/AP_NavEKF3.h>
#endif

/**
 * @class AP_AHRS_SIM
 * @brief SITL simulation backend for AHRS providing perfect knowledge for algorithm testing
 * 
 * @details AP_AHRS_SIM is a specialized AHRS backend used exclusively in Software-In-The-Loop (SITL)
 *          simulation environments. Instead of fusing real sensor data, it provides perfect attitude,
 *          position, velocity, and wind information directly from the SITL physics simulation
 *          (SITL::sitl_fdm). This enables deterministic testing of flight control algorithms, navigation
 *          logic, and mission planning without sensor noise, bias, or failures.
 *          
 *          Key characteristics:
 *          - Perfect attitude estimation (no gyro drift, no accelerometer bias)
 *          - Instantaneous updates with no latency or timing jitter
 *          - Always reports healthy status for testing nominal code paths
 *          - Optionally integrates with EKF3 for testing body-frame odometry injection
 *          - All coordinate frames use NED (North-East-Down) convention
 *          - Position and velocity values have zero error relative to simulation truth
 * 
 * @note This backend is only compiled when AP_AHRS_SIM_ENABLED is true (SITL builds only)
 * @warning SITL perfect knowledge does not model real sensor characteristics including:
 *          noise, quantization, temperature drift, vibration sensitivity, magnetic interference,
 *          GPS multipath, timing delays, or sensor failures. Always validate algorithms on hardware.
 */
class AP_AHRS_SIM : public AP_AHRS_Backend {
public:

#if HAL_NAVEKF3_AVAILABLE
    /**
     * @brief Constructor with optional EKF3 integration for body-frame odometry testing
     * 
     * @details Constructs AP_AHRS_SIM backend with reference to NavEKF3 instance. This allows
     *          the SITL backend to inject body-frame odometry data into EKF3 for testing
     *          visual odometry or wheel encoder fusion algorithms.
     * 
     * @param[in] _EKF3 Reference to NavEKF3 instance for odometry injection
     * 
     * @note This constructor is only available when HAL_NAVEKF3_AVAILABLE is defined
     */
    AP_AHRS_SIM(NavEKF3 &_EKF3) :
        AP_AHRS_Backend(),
        EKF3(_EKF3)
        { }
    
    /**
     * @brief Destructor for SITL AHRS backend
     */
    ~AP_AHRS_SIM() {}
#else
    /**
     * @brief Constructor for SITL AHRS backend without EKF3 integration
     * 
     * @details Constructs AP_AHRS_SIM backend without EKF3 reference. This version is used
     *          when NavEKF3 is not available in the build configuration.
     * 
     * @note This constructor is used when HAL_NAVEKF3_AVAILABLE is not defined
     */
    AP_AHRS_SIM() : AP_AHRS_Backend() { }
#endif

    CLASS_NO_COPY(AP_AHRS_SIM);

    /**
     * @brief Reset gyro drift estimate (no-op for SITL)
     * 
     * @details In real AHRS backends, this resets the accumulated gyro bias estimate when
     *          gyro offsets are recalibrated. For SITL simulation, gyros are perfect with
     *          zero drift, so this operation is a no-op.
     * 
     * @note SITL gyros have no drift or bias - this method exists for interface compatibility
     */
    void reset_gyro_drift() override {};

    /**
     * @brief Update AHRS state estimate (no-op for SITL)
     * 
     * @details In real AHRS backends, this performs sensor fusion and state prediction.
     *          For SITL simulation, state data comes directly from SITL::sitl_fdm and is
     *          accessed on-demand via get_results(), so no periodic update is required.
     * 
     * @note SITL state is pulled from simulation, not pushed via update cycles
     */
    void            update() override { }
    
    /**
     * @brief Populate Estimates structure with perfect SITL simulation data
     * 
     * @details Retrieves attitude (quaternion), angular rates (gyro), position (NED frame),
     *          velocity (NED frame), and origin location directly from SITL physics simulation.
     *          All values represent perfect ground truth with zero error, enabling deterministic
     *          testing of control algorithms.
     * 
     * @param[out] results Estimates structure populated with SITL simulation state including:
     *                     - attitude: quaternion rotation from NED to body frame
     *                     - gyro_estimate: body-frame angular rates in rad/s
     *                     - gyro_drift: zero (no drift in simulation)
     *                     - location: current position as Location
     *                     - velocity_NED: velocity in m/s (North-East-Down frame)
     * 
     * @note All data has zero latency and perfect accuracy - not representative of real sensors
     * @warning Coordinate frames: NED for earth-relative, body frame for vehicle-relative
     */
    void            get_results(Estimates &results) override;
    
    /**
     * @brief Reset AHRS state (no-op for SITL)
     * 
     * @details In real AHRS backends, this reinitializes state estimates. For SITL simulation,
     *          state is always synchronized with physics simulation, so reset is not applicable.
     * 
     * @note SITL backend tracks simulation state automatically - no reset needed
     */
    void            reset() override { return; }

    /**
     * @brief Get height above ground level from SITL simulation
     * 
     * @details Returns the simulated height above ground level in meters. This represents
     *          perfect rangefinder or terrain-following altitude from the SITL physics model.
     * 
     * @param[out] hagl Height above ground level in meters (positive upward)
     * 
     * @return true if HAGL data is available from simulation, false otherwise
     * 
     * @note Units: meters (positive altitude above ground)
     */
    bool get_hagl(float &hagl) const override WARN_IF_UNUSED;

    /**
     * @brief Get simulated wind velocity vector
     * 
     * @details Returns the wind vector configured in SITL simulation parameters. Wind is
     *          specified in NED (North-East-Down) frame and represents steady-state wind
     *          without gusts or turbulence (unless SITL turbulence is enabled).
     * 
     * @param[out] wind Wind velocity vector in m/s (NED frame: North, East, Down components)
     * 
     * @return true if wind estimate is available (always true for SITL)
     * 
     * @note Coordinate frame: NED (North-East-Down)
     * @note Units: m/s for all three components
     */
    bool wind_estimate(Vector3f &wind) const override;

    /**
     * @brief Get equivalent airspeed estimate from SITL simulation
     * 
     * @details Returns simulated equivalent airspeed (EAS) from the SITL flight dynamics model.
     *          This represents the airspeed that would be measured by a pitot tube, accounting
     *          for air density.
     * 
     * @param[out] airspeed_ret Equivalent airspeed in m/s
     * 
     * @return true if airspeed estimate is available from simulation
     * 
     * @note Units: m/s (meters per second)
     * @note EAS accounts for air density variations with altitude
     */
    bool airspeed_EAS(float &airspeed_ret) const override;

    /**
     * @brief Get equivalent airspeed estimate from specific sensor index
     * 
     * @details Returns simulated equivalent airspeed for a specific airspeed sensor index.
     *          In SITL, all sensor indices return the same simulated airspeed value.
     * 
     * @param[in]  airspeed_index Airspeed sensor index (0-based)
     * @param[out] airspeed_ret   Equivalent airspeed in m/s
     * 
     * @return true if airspeed estimate is available for specified sensor index
     * 
     * @note Units: m/s (meters per second)
     * @note All sensor indices return same SITL airspeed value
     */
    bool airspeed_EAS(uint8_t airspeed_index, float &airspeed_ret) const override;

    /**
     * @brief Get 2D ground speed vector from SITL simulation
     * 
     * @details Returns horizontal ground velocity vector from SITL physics simulation in
     *          North-East order. This represents vehicle velocity over ground in the
     *          horizontal plane.
     * 
     * @return Vector2f containing ground speed in m/s with components:
     *         - x: North velocity (m/s)
     *         - y: East velocity (m/s)
     * 
     * @note Coordinate frame: NED horizontal plane (North-East components only)
     * @note Units: m/s (meters per second)
     */
    Vector2f groundspeed_vector() override;

    /**
     * @brief Check if compass should be used (always true for SITL)
     * 
     * @details In real AHRS backends, this indicates whether compass data is being used for
     *          heading estimation. For SITL simulation, compass is always considered available
     *          and reliable.
     * 
     * @return true always - SITL compass is perfect and always used
     * 
     * @note SITL compass has zero error and no interference
     */
    bool            use_compass() override { return true; }

    /**
     * @brief Get attitude quaternion from SITL simulation
     * 
     * @details Returns the quaternion representing vehicle attitude from SITL flight dynamics.
     *          The quaternion defines the rotation from NED (North-East-Down) earth frame to
     *          body frame (X-forward, Y-right, Z-down).
     * 
     * @param[out] quat Quaternion rotation from NED frame to body frame (X-Y-Z axes)
     * 
     * @return true if quaternion is available (always true for SITL)
     * 
     * @note Coordinate frames: NED earth frame to body frame (X-forward, Y-right, Z-down)
     * @note Quaternion is normalized with perfect accuracy
     * @warning This is perfect attitude with no gyro drift, accelerometer bias, or magnetic interference
     */
    bool get_quaternion(Quaternion &quat) const override WARN_IF_UNUSED;

    /**
     * @brief Check AHRS subsystem health status
     * 
     * @details In real AHRS backends, this checks sensor health, fusion convergence, and
     *          innovation consistency. For SITL simulation, AHRS is always healthy to enable
     *          testing of nominal flight code paths.
     * 
     * @return true always - SITL AHRS never fails for deterministic testing
     * 
     * @note SITL backend is always healthy - use this for testing nominal code paths only
     * @warning Does not simulate AHRS failure modes or degraded states
     */
    bool healthy() const override { return true; }

    /**
     * @brief Get 3D velocity vector in NED frame from SITL simulation
     * 
     * @details Returns vehicle velocity vector from SITL physics simulation in NED
     *          (North-East-Down) frame. All three components represent inertial velocity
     *          with perfect accuracy.
     * 
     * @param[out] vec Velocity vector in m/s with NED components:
     *                 - x: North velocity (m/s)
     *                 - y: East velocity (m/s)
     *                 - z: Down velocity (m/s, positive downward)
     * 
     * @return true if velocity data is available (always true for SITL)
     * 
     * @note Coordinate frame: NED (North-East-Down)
     * @note Units: m/s (meters per second)
     * @note Down component is positive when descending
     */
    bool get_velocity_NED(Vector3f &vec) const override;

    /**
     * @brief Get vertical velocity kinematically consistent with position
     * 
     * @details Returns vertical velocity in Down direction that is kinematically consistent
     *          with the vertical position derivative. This is required by some control loops
     *          that need position and velocity to be synchronized. In SITL, this is always
     *          perfectly consistent since both come from the same physics simulation.
     * 
     * @param[out] velocity Vertical velocity in m/s (Down positive, consistent with NED frame)
     * 
     * @return true if vertical velocity is available (always true for SITL)
     * 
     * @note Units: m/s (positive when descending)
     * @note Coordinate frame: NED Down component (positive downward)
     * @note SITL provides perfect kinematic consistency between position and velocity
     */
    bool get_vert_pos_rate_D(float &velocity) const override;

    /**
     * @brief Perform pre-arm checks for SITL AHRS (always passes)
     * 
     * @details In real AHRS backends, this validates sensor health, fusion convergence, GPS lock,
     *          and position estimation quality before allowing vehicle arming. For SITL simulation,
     *          all checks always pass to enable testing without setup requirements.
     * 
     * @param[in]  requires_position Flag indicating if horizontal position check is required (not used in SITL)
     * @param[out] failure_msg       Buffer for failure message (not populated in SITL)
     * @param[in]  failure_msg_len   Length of failure message buffer
     * 
     * @return true always - SITL AHRS always passes pre-arm checks for testing convenience
     * 
     * @note SITL always passes pre-arm to enable rapid testing without sensor setup
     * @warning Does not validate real pre-arm conditions - hardware validation required
     */
    bool pre_arm_check(bool requires_position, char *failure_msg, uint8_t failure_msg_len) const override { return true; }

    /**
     * @brief Get navigation filter status flags for SITL
     * 
     * @details Returns idealized filter status indicating perfect position, velocity, and
     *          attitude estimation. All flags indicate optimal conditions for testing nominal
     *          code paths. Status includes solution validity, sensor health, and estimation modes.
     * 
     * @param[out] status nav_filter_status structure populated with ideal flags:
     *                    - All position/velocity/attitude flags set to valid
     *                    - All sensor health flags set to healthy
     *                    - No error or warning flags set
     * 
     * @return true if filter status is available (always true for SITL)
     * 
     * @note SITL provides idealized status flags for deterministic testing
     * @warning Does not represent real filter convergence or sensor degradation states
     */
    bool get_filter_status(nav_filter_status &status) const override;

    /**
     * @brief Get simulated navigation origin location
     * 
     * @details Returns the origin Location used as reference for relative position calculations
     *          in SITL simulation. This typically corresponds to the vehicle's initial position
     *          when SITL was started.
     * 
     * @param[out] ret Origin location (latitude, longitude, altitude)
     * 
     * @return true if origin is available (always true for SITL)
     * 
     * @note Used as reference point for relative position calculations in AP_InertialNav
     */
    bool get_origin(Location &ret) const override;
    
    /**
     * @brief Get 3D position relative to origin in NED frame
     * 
     * @details Returns vehicle position relative to navigation origin in NED frame from SITL
     *          simulation. Position is expressed in meters North, East, and Down from origin.
     * 
     * @param[out] vec Relative position in meters (NED frame):
     *                 - x: North offset from origin (m)
     *                 - y: East offset from origin (m)
     *                 - z: Down offset from origin (m, positive downward)
     * 
     * @return true if relative position is available (always true for SITL)
     * 
     * @note Coordinate frame: NED (North-East-Down) relative to origin
     * @note Units: meters
     */
    bool get_relative_position_NED_origin(Vector3p &vec) const override;
    
    /**
     * @brief Get 2D horizontal position relative to origin
     * 
     * @details Returns vehicle horizontal position relative to navigation origin from SITL
     *          simulation. Position is expressed in meters North and East from origin.
     * 
     * @param[out] posNE Relative horizontal position in meters:
     *                   - x: North offset from origin (m)
     *                   - y: East offset from origin (m)
     * 
     * @return true if horizontal position is available (always true for SITL)
     * 
     * @note Coordinate frame: NED horizontal plane (North-East only)
     * @note Units: meters
     */
    bool get_relative_position_NE_origin(Vector2p &posNE) const override;
    
    /**
     * @brief Get vertical position relative to origin (Down component)
     * 
     * @details Returns vehicle Down position relative to navigation origin from SITL simulation.
     *          Positive values indicate vehicle is below the origin altitude.
     * 
     * @param[out] posD Down position relative to origin in meters (positive when below origin)
     * 
     * @return true if Down position is available (always true for SITL)
     * 
     * @note Coordinate frame: NED Down component (positive downward)
     * @note Units: meters (positive when vehicle is below origin altitude)
     */
    bool get_relative_position_D_origin(postype_t &posD) const override;

    /**
     * @brief Send idealized EKF status telemetry to ground control station
     * 
     * @details Sends MAVLink EKF_STATUS_REPORT message with idealized flags and zero errors
     *          for SITL simulation. All innovation tests show passing, all sensors healthy,
     *          and all variances at nominal levels for testing GCS displays.
     * 
     * @param[in] link Reference to GCS_MAVLINK instance for sending telemetry message
     * 
     * @note SITL provides perfect EKF status for testing telemetry displays and GCS logic
     * @warning Does not represent real EKF convergence, innovation failures, or sensor issues
     */
    void send_ekf_status_report(class GCS_MAVLINK &link) const override;

    /**
     * @brief Get control limits for SITL navigation
     * 
     * @details Returns control limits used by position controllers and navigation logic.
     *          In SITL, these represent ideal conditions without EKF uncertainty limits.
     * 
     * @param[out] ekfGndSpdLimit  EKF-derived ground speed limit in m/s (typically large for SITL)
     * @param[out] controlScaleXY  Horizontal control scaling factor (typically 1.0 for SITL)
     * 
     * @note Units: m/s for ekfGndSpdLimit, dimensionless (0.0-1.0) for controlScaleXY
     * @note SITL provides nominal limits without uncertainty-based restrictions
     */
    void get_control_limits(float &ekfGndSpdLimit, float &controlScaleXY) const override;
    
    /**
     * @brief Get simulated EKF innovation vectors for testing
     * 
     * @details Returns innovation vectors (measurement - prediction differences) from simulated
     *          EKF processing. In SITL, innovations are typically near zero or simulated values
     *          for testing innovation monitoring logic and fault detection algorithms.
     * 
     * @param[out] velInnov Velocity innovation vector in m/s (NED frame)
     * @param[out] posInnov Position innovation vector in meters (NED frame)
     * @param[out] magInnov Magnetometer innovation vector in milligauss
     * @param[out] tasInnov True airspeed innovation in m/s
     * @param[out] yawInnov Yaw angle innovation in radians
     * 
     * @return true if innovation data is available
     * 
     * @note Units: m/s for velocities, meters for positions, milligauss for mag, radians for yaw
     * @note SITL innovations are simulated - may be zero or test values
     */
    bool get_innovations(Vector3f &velInnov, Vector3f &posInnov, Vector3f &magInnov, float &tasInnov, float &yawInnov) const override;
    
    /**
     * @brief Get simulated EKF variance estimates for testing
     * 
     * @details Returns variance estimates from simulated EKF covariance matrix. In SITL, variances
     *          represent idealized uncertainty estimates for testing control logic that adapts
     *          to estimation uncertainty.
     * 
     * @param[out] velVar Velocity estimate variance in (m/s)^2
     * @param[out] posVar Horizontal position estimate variance in m^2
     * @param[out] hgtVar Vertical position estimate variance in m^2
     * @param[out] magVar Magnetometer variance vector in (milligauss)^2
     * @param[out] tasVar True airspeed estimate variance in (m/s)^2
     * 
     * @return true if variance data is available
     * 
     * @note Units: squared units of corresponding states (m^2, (m/s)^2, etc.)
     * @note SITL variances are simulated - typically small values representing perfect estimation
     */
    bool get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar) const override;

private:

    // dead-reckoning support
    bool get_location(Location &loc) const;

#if HAL_NAVEKF3_AVAILABLE
    // a reference to the EKF3 backend that we can use to send in
    // body-frame-odometry data into the EKF.  Rightfully there should
    // be something over in the SITL directory doing this.
    NavEKF3 &EKF3;
#endif

    class SITL::SIM *_sitl;
    uint32_t _last_body_odm_update_ms;
};

#endif  // AP_AHRS_SIM_ENABLED
