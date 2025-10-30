/**
 * @file AP_AHRS_Backend.h
 * @brief AHRS (Attitude Heading Reference System) backend interface for ArduPilot
 * 
 * @details This file defines the abstract base class for all AHRS backend implementations
 *          in ArduPilot. The AHRS system is responsible for providing vehicle attitude,
 *          heading, position, and velocity estimates by fusing data from multiple sensors
 *          including IMUs, GPS, compass, barometer, and airspeed sensors.
 * 
 *          Backend implementations include:
 *          - DCM (Direction Cosine Matrix): Legacy complementary filter implementation
 *          - EKF2/EKF3: Extended Kalman Filter implementations for full navigation
 *          - External: Interface for external navigation systems
 *          - SITL: Software-in-the-loop simulation backend
 * 
 *          All attitude angles use NED (North-East-Down) earth-fixed frame convention,
 *          and body frame follows XYZ (Forward-Right-Down) convention. Quaternions and
 *          DCM matrices represent rotation from NED frame to body frame.
 * 
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 * @license GNU General Public License v3.0 or later
 */

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

#include <AP_Math/AP_Math.h>
#include <inttypes.h>
#include <AP_Airspeed/AP_Airspeed.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_Common/Location.h>
#include <AP_NavEKF/AP_NavEKF_Source.h>

/// Maximum trim angle in degrees - limits IMU board orientation correction
#define AP_AHRS_TRIM_LIMIT 10.0f

/// Minimum value for AHRS_RP_P parameter (roll/pitch correction gain) - prevents too slow attitude correction
#define AP_AHRS_RP_P_MIN   0.05f

/// Minimum value for AHRS_YAW_P parameter (yaw correction gain) - prevents too slow heading correction
#define AP_AHRS_YAW_P_MIN  0.05f

/**
 * @enum GPSUse
 * @brief GPS usage configuration for AHRS altitude estimation
 * 
 * @details Controls how GPS altitude data is incorporated into the AHRS solution.
 *          This affects both horizontal position and vertical position estimates.
 */
enum class GPSUse : uint8_t {
    Disable = 0,            ///< GPS altitude not used, rely on barometer only
    Enable  = 1,            ///< GPS altitude used for horizontal position only
    EnableWithHeight = 2,   ///< GPS altitude used for both horizontal and vertical position
};

/**
 * @class AP_AHRS_Backend
 * @brief Abstract base class for all AHRS backend implementations
 * 
 * @details This class defines the interface that all AHRS backend implementations must provide.
 *          It abstracts the differences between various attitude and position estimation algorithms,
 *          allowing the main AP_AHRS class to work with any backend implementation.
 * 
 *          Concrete implementations include:
 *          - AP_AHRS_DCM: Complementary filter using Direction Cosine Matrix representation
 *          - AP_AHRS_NavEKF: Extended Kalman Filter implementations (EKF2/EKF3) for full navigation
 *          - AP_AHRS_External: Interface for external navigation systems (e.g., motion capture)
 *          - AP_AHRS_SIM: Simulation backend for software-in-the-loop testing
 * 
 *          The backend is responsible for:
 *          - Fusing IMU, GPS, compass, barometer, and airspeed sensor data
 *          - Providing attitude estimates (roll, pitch, yaw) in NED earth frame
 *          - Providing position and velocity estimates when GPS or other navigation sources available
 *          - Detecting and handling sensor failures and inconsistencies
 *          - Supporting pre-arm safety checks for flight controller
 * 
 *          Coordinate frame conventions:
 *          - NED frame: North-East-Down earth-fixed reference frame
 *          - Body frame (XYZ): Forward-Right-Down vehicle-fixed frame
 *          - Rotations: Quaternions and DCM matrices represent rotation from NED to body frame
 * 
 * @note This is an abstract base class - all pure virtual methods must be implemented by derived classes
 * @warning Implementations must be thread-safe for access from multiple scheduler tasks
 */
class AP_AHRS_Backend
{
public:

    // Constructor
    AP_AHRS_Backend() {}

    // empty virtual destructor
    virtual ~AP_AHRS_Backend() {}

    CLASS_NO_COPY(AP_AHRS_Backend);

    /**
     * @struct Estimates
     * @brief Structure containing complete state estimates from AHRS backend
     * 
     * @details This structure is populated by the get_results() method and contains
     *          all attitude, rate, acceleration, and position estimates from the backend.
     *          All values are in standard ArduPilot units and coordinate frames.
     */
    struct Estimates {
        float roll_rad;              ///< Roll angle in radians (rotation about North axis, positive right wing down)
        float pitch_rad;             ///< Pitch angle in radians (rotation about East axis, positive nose up)
        float yaw_rad;               ///< Yaw angle in radians (rotation about Down axis, positive clockwise viewed from above)
        Matrix3f dcm_matrix;         ///< Direction Cosine Matrix - rotation from NED frame to body frame
        Vector3f gyro_estimate;      ///< Estimated gyro rates in rad/s (body frame: roll, pitch, yaw rates)
        Vector3f gyro_drift;         ///< Estimated gyro bias/drift in rad/s (body frame)
        Vector3f accel_ef;           ///< Acceleration in earth frame (NED) in m/s² with gravity removed
        Vector3f accel_bias;         ///< Estimated accelerometer bias in m/s² (body frame)

        Location location;           ///< Estimated position (latitude in degrees, longitude in degrees, altitude in centimeters AMSL)
        bool location_valid;         ///< True if location estimate is valid and should be used

        /**
         * @brief Get current location estimate if valid
         * @param[out] loc Location structure to be populated
         * @return true if location is valid, false otherwise
         */
        bool get_location(Location &loc) const {
            loc = location;
            return location_valid;
        };
    };

    /**
     * @brief Initialize the AHRS backend
     * 
     * @details Sets up the backend including INS (Inertial Navigation System) board orientation
     *          from parameters. This is called once during system startup after parameters are loaded.
     *          Derived classes should call this base implementation and then perform their own
     *          initialization (e.g., EKF memory allocation, filter state initialization).
     * 
     * @note Called from main thread during vehicle initialization
     */
    virtual void init();

    /**
     * @brief Get the index of the current primary gyro sensor
     * 
     * @details Returns the IMU gyro index that is currently being used as the primary source
     *          for attitude estimation. This is used for logging, diagnostics, and to ensure
     *          consistency between AHRS and other systems using gyro data.
     * 
     * @return uint8_t Index of active gyro sensor (0-based), or 0 if no gyro available
     * 
     * @note This may change during flight if IMU failover occurs
     */
    virtual uint8_t get_primary_gyro_index(void) const {
#if AP_INERTIALSENSOR_ENABLED
        return AP::ins().get_first_usable_gyro();
#else
        return 0;
#endif
    }

    /**
     * @brief Update AHRS state estimates from latest sensor data
     * 
     * @details This is the main processing function called at the vehicle's main loop rate
     *          (typically 400Hz for multicopters, 50-400Hz for other vehicles). It reads the
     *          latest IMU samples and other sensor data, runs the estimation algorithm, and
     *          updates internal state. After this call, get_results() will return updated estimates.
     * 
     * @warning This is a pure virtual method that must be implemented by all concrete backends
     * @note Called from main scheduler task at loop rate - must complete within timing budget
     * @note Implementation must be deterministic and avoid blocking operations
     */
    virtual void update() = 0;

    /**
     * @brief Retrieve current state estimates from the backend
     * 
     * @details Populates the provided Estimates structure with the most recent attitude, rate,
     *          acceleration, and position estimates. This should be called after update() to
     *          get the latest state. Values are valid immediately after update() completes.
     * 
     * @param[out] results Estimates structure to be populated with current state
     * 
     * @warning This is a pure virtual method that must be implemented by all concrete backends
     * @note Thread-safe for reading after update() completes
     */
    virtual void get_results(Estimates &results) = 0;

    /**
     * @brief Perform pre-arm safety checks on AHRS state
     * 
     * @details Validates that the AHRS system is in a safe state for arming and flight.
     *          Checks may include: attitude estimate validity, sensor health, position accuracy,
     *          innovation consistency, and other backend-specific safety criteria. If any check
     *          fails, a descriptive error message is written to the failure message buffer.
     * 
     * @param[in]  requires_position If true, horizontal position estimation must be available and accurate
     * @param[out] failure_msg       Buffer to receive failure description if check fails
     * @param[in]  failure_msg_len   Size of failure_msg buffer in bytes
     * 
     * @return true if all pre-arm checks pass and vehicle is safe to arm, false otherwise
     * 
     * @warning This is a pure virtual method that must be implemented by all concrete backends
     * @note Called during arming sequence and periodically during pre-arm state
     */
    virtual bool pre_arm_check(bool requires_position, char *failure_msg, uint8_t failure_msg_len) const = 0;

    /**
     * @brief Check attitude consistency across multiple EKF cores/IMUs
     * 
     * @details For backends supporting multiple independent estimators (e.g., EKF with multiple cores),
     *          verifies that all cores are providing consistent attitude estimates. Large divergence
     *          between cores indicates sensor failure or misconfiguration and prevents arming.
     * 
     * @param[out] failure_msg     Buffer to receive failure description if inconsistent
     * @param[in]  failure_msg_len Size of failure_msg buffer in bytes
     * 
     * @return true if attitudes are consistent or only one core active, false if divergence detected
     * 
     * @note Default implementation returns true (no check) - override for multi-core backends
     */
    virtual bool attitudes_consistent(char *failure_msg, const uint8_t failure_msg_len) const { return true; }

    /**
     * @brief Attempt EKF lane switching to avoid imminent failsafe
     * 
     * @details For multi-core EKF backends, attempts to switch to a healthier EKF lane if the
     *          current primary lane is experiencing issues. This can prevent unnecessary EKF
     *          failsafes when another lane has a better solution. Called when innovation checks
     *          indicate the current lane may be diverging.
     * 
     * @note Default implementation does nothing - override for multi-lane EKF backends
     * @note May cause brief discontinuity in position/velocity estimates during switch
     */
    virtual void check_lane_switch(void) {}

    /**
     * @brief Check if non-compass sensor is providing yaw estimate
     * 
     * @details Returns true if yaw estimate is coming from a source other than magnetometer,
     *          such as GPS heading, external navigation system, or GSF (Gaussian Sum Filter).
     *          This allows compass pre-arm checks to be bypassed when not relying on compass.
     * 
     * @return true if using non-compass yaw source (GPS, external nav, etc.), false if using compass
     * 
     * @note Affects whether compass calibration and health checks are required for arming
     */
    virtual bool using_noncompass_for_yaw(void) const { return false; }

    /**
     * @brief Check if external navigation system is providing yaw
     * 
     * @details Returns true if yaw estimate is provided by an external navigation source
     *          such as motion capture system, visual odometry, or RTK GPS with dual antenna.
     * 
     * @return true if external navigation is active and providing yaw, false otherwise
     */
    virtual bool using_extnav_for_yaw(void) const { return false; }

    /**
     * @brief Request EKF to reset yaw to avoid failsafe or lane switch
     * 
     * @details Requests the backend to reset its yaw estimate, typically to match GPS heading
     *          or another trusted source. Used when yaw innovations are large but resetting
     *          yaw would resolve the issue, avoiding EKF failsafe or lane switch.
     * 
     * @note Default implementation does nothing - override for EKF backends supporting yaw reset
     * @note May cause brief heading discontinuity but preferable to full EKF failsafe
     */
    virtual void request_yaw_reset(void) {}

    /**
     * @brief Set position, velocity, and yaw source set selection
     * 
     * @details Switches between primary, secondary, and tertiary sensor source sets for
     *          position, velocity, and yaw estimation. Used to manually select GPS instances,
     *          switch between GPS and non-GPS navigation, or recover from sensor failures.
     * 
     * @param[in] source_set_idx Source set to activate: 0=primary, 1=secondary, 2=tertiary
     * 
     * @note Default implementation does nothing - override for backends supporting source switching
     * @note Typically used with EKF3 multi-source configuration
     */
    virtual void set_posvelyaw_source_set(AP_NavEKF_Source::SourceSetSelection source_set_idx) {}

    /**
     * @brief Reset gyro drift/bias estimate
     * 
     * @details Clears the accumulated gyro bias estimate and restarts bias learning.
     *          Should be called after IMU calibration or when gyro offsets are recalculated
     *          to ensure old bias estimates don't contaminate new measurements.
     * 
     * @warning This is a pure virtual method that must be implemented by all concrete backends
     * @note Causes brief increase in attitude estimation error until bias re-converges
     * @note Typically called during ground calibration, not during flight
     */
    virtual void reset_gyro_drift(void) = 0;

    /**
     * @brief Reset attitude estimate
     * 
     * @details Resets the attitude estimate to a new initial condition, typically after
     *          IMU calibration or when the vehicle has been moved while disarmed. The backend
     *          will reinitialize attitude from accelerometer and compass readings.
     * 
     * @warning This is a pure virtual method that must be implemented by all concrete backends
     * @warning Do NOT call during flight - only safe when vehicle is stationary on ground
     * @note For EKF backends, may also reset velocity and position estimates
     */
    virtual void reset() = 0;

    /**
     * @brief Get height above ground level estimate
     * 
     * @details Returns the estimated height above the terrain or landing surface in meters.
     *          This is distinct from altitude above mean sea level - it uses rangefinder,
     *          terrain database, or other sensors to estimate clearance above ground.
     * 
     * @param[out] height Height above ground level in meters (positive up)
     * 
     * @return true if HAGL estimate is available and valid, false otherwise
     * 
     * @note Used for terrain following, landing detection, and obstacle avoidance
     * @note Accuracy depends on rangefinder availability and terrain database coverage
     */
    virtual bool get_hagl(float &height) const WARN_IF_UNUSED { return false; }

    /**
     * @brief Get wind velocity estimate
     * 
     * @details Returns the estimated wind velocity vector in NED (North-East-Down) frame.
     *          Wind is estimated from the difference between airspeed and ground speed
     *          (for vehicles with airspeed sensor) or from accelerometer during coordinated turns.
     * 
     * @param[out] wind Wind velocity vector in m/s (North, East, Down components)
     * 
     * @return true if wind estimate is available, false otherwise
     * 
     * @warning This is a pure virtual method that must be implemented by all concrete backends
     * @note Primarily used by fixed-wing aircraft for navigation and control
     * @note Down component typically near zero for horizontal wind
     */
    virtual bool wind_estimate(Vector3f &wind) const = 0;

    /**
     * @brief Get equivalent airspeed (EAS) estimate
     * 
     * @details Returns the equivalent airspeed, which is the airspeed corrected to sea level
     *          standard atmospheric conditions. This is the speed that produces the same
     *          dynamic pressure as the actual flight condition. For low-altitude flight,
     *          EAS is approximately equal to indicated airspeed (IAS).
     * 
     * @param[out] airspeed_ret Equivalent airspeed in m/s
     * 
     * @return true if airspeed estimate is available, false otherwise
     * 
     * @note Requires airspeed sensor or synthetic airspeed estimation from accelerometer
     * @note Used primarily by fixed-wing aircraft for speed control
     */
    virtual bool airspeed_EAS(float &airspeed_ret) const WARN_IF_UNUSED { return false; }

    /**
     * @brief Get equivalent airspeed from specific airspeed sensor
     * 
     * @details Returns equivalent airspeed from a specific airspeed sensor index.
     *          Used when multiple airspeed sensors are installed for redundancy.
     * 
     * @param[in]  airspeed_index Index of airspeed sensor (0-based)
     * @param[out] airspeed_ret   Equivalent airspeed in m/s
     * 
     * @return true if specified airspeed sensor is available, false otherwise
     */
    virtual bool airspeed_EAS(uint8_t airspeed_index, float &airspeed_ret) const { return false; }

    /**
     * @brief Get true airspeed (TAS) estimate
     * 
     * @details Returns the true airspeed by applying EAS2TAS correction factor to equivalent
     *          airspeed. True airspeed is the actual speed of the aircraft through the air mass,
     *          accounting for altitude and temperature effects on air density. This is the
     *          airspeed used for navigation calculations.
     * 
     * @param[out] airspeed_ret True airspeed in m/s
     * 
     * @return true if airspeed estimate is available, false otherwise
     * 
     * @note Automatically applies EAS2TAS correction based on current altitude and temperature
     * @note TAS is higher than EAS at altitude due to lower air density
     */
    bool airspeed_TAS(float &airspeed_ret) const WARN_IF_UNUSED {
        if (!airspeed_EAS(airspeed_ret)) {
            return false;
        }
        airspeed_ret *= get_EAS2TAS();
        return true;
    }

    /**
     * @brief Get equivalent-to-true airspeed correction ratio
     * 
     * @details Returns the ratio used to convert equivalent airspeed (EAS) to true airspeed (TAS).
     *          This ratio accounts for air density variation with altitude and temperature.
     *          At sea level standard conditions, ratio is 1.0. Ratio increases with altitude.
     * 
     * @return Ratio to multiply EAS by to get TAS (dimensionless, typically 1.0 to 1.5)
     * 
     * @note Based on current barometric altitude and temperature estimate
     */
    static float get_EAS2TAS(void);

    /**
     * @brief Get true-to-equivalent airspeed correction ratio
     * 
     * @details Returns the ratio used to convert true airspeed (TAS) to equivalent airspeed (EAS).
     *          This is simply the inverse of get_EAS2TAS().
     * 
     * @return Ratio to multiply TAS by to get EAS (dimensionless, typically 0.67 to 1.0)
     */
    static float get_TAS2EAS(void) { return 1.0/get_EAS2TAS(); }

    /**
     * @brief Check if airspeed sensor is enabled and healthy
     * 
     * @details Returns true if airspeed is coming from a physical airspeed sensor
     *          (pitot tube) rather than synthetic estimate from accelerometer. Checks
     *          that sensor is configured for use and currently healthy.
     * 
     * @return true if airspeed sensor is enabled and providing valid data, false otherwise
     * 
     * @note Returns false if airspeed is disabled or using synthetic estimate
     * @note Returns false if sensor is unhealthy or not connected
     */
    static bool airspeed_sensor_enabled(void) {
    #if AP_AIRSPEED_ENABLED
        const AP_Airspeed *_airspeed = AP::airspeed();
        return _airspeed != nullptr && _airspeed->use() && _airspeed->healthy();
    #else
        return false;
    #endif
    }

    /**
     * @brief Check if specific airspeed sensor is enabled and healthy
     * 
     * @details Returns true if a specific airspeed sensor (by index) is configured,
     *          enabled, and currently providing healthy data.
     * 
     * @param[in] airspeed_index Index of airspeed sensor to check (0-based)
     * 
     * @return true if specified sensor is enabled and healthy, false otherwise
     * 
     * @note Used for multi-sensor airspeed configurations with redundancy
     */
    static bool airspeed_sensor_enabled(uint8_t airspeed_index) {
    #if AP_AIRSPEED_ENABLED
        const AP_Airspeed *_airspeed = AP::airspeed();
        return _airspeed != nullptr && _airspeed->use(airspeed_index) && _airspeed->healthy(airspeed_index);
    #else
        return false;
    #endif
    }

    /**
     * @brief Get ground velocity vector in horizontal plane
     * 
     * @details Returns the 2D ground velocity vector in the horizontal plane (NED frame).
     *          Ground velocity is the vehicle's velocity relative to the earth, derived from
     *          GPS or other position sources combined with IMU integration.
     * 
     * @return Vector2f Ground velocity in m/s (North and East components only)
     * 
     * @warning This is a pure virtual method that must be implemented by all concrete backends
     * @note Order is North, East (not East, North)
     * @note Used by navigation controllers for waypoint tracking
     */
    virtual Vector2f groundspeed_vector(void) = 0;

    /**
     * @brief Get 3D velocity in NED frame
     * 
     * @details Returns the complete 3D velocity vector in NED (North-East-Down) frame.
     *          This estimate combines GPS velocity with IMU acceleration integration for
     *          high-rate, low-latency velocity. Only accurate when inertial navigation is active.
     * 
     * @param[out] vec Velocity vector in m/s (North, East, Down components)
     * 
     * @return true if velocity estimate is available and valid, false otherwise
     * 
     * @note Requires GPS or other position source for accuracy
     * @note Down component positive means descending
     * @note Update rate matches main loop rate (faster than GPS alone)
     */
    virtual bool get_velocity_NED(Vector3f &vec) const WARN_IF_UNUSED {
        return false;
    }

    /**
     * @brief Get vertical velocity kinematically consistent with position
     * 
     * @details Returns the time derivative of vertical position (Down component) that is
     *          kinematically consistent with the position estimate. This is required by some
     *          control loops to avoid control instability. Note this is different from the
     *          vertical velocity from EKF, which may contain corrections for sensor errors.
     * 
     * @param[out] velocity Vertical velocity in m/s (positive Down)
     * 
     * @return true if estimate is available, false otherwise
     * 
     * @warning This is a pure virtual method that must be implemented by all concrete backends
     * @note Used by altitude hold and vertical position controllers
     * @note Matches position derivative exactly to prevent control loop oscillation
     */
    virtual bool get_vert_pos_rate_D(float &velocity) const = 0;

    /**
     * @brief Set the EKF origin for position estimates
     * 
     * @details Sets the origin (reference point) for all relative position estimates.
     *          The origin is typically set to the first GPS lock location or takeoff location.
     *          All relative position estimates are measured from this origin.
     * 
     * @param[in] loc Location to use as origin (latitude, longitude, altitude)
     * 
     * @return true if origin was set successfully, false if not supported or failed
     * 
     * @note Only works with EKF backends - DCM returns false
     * @note Origin should be set once and not changed during flight
     */
    virtual bool set_origin(const Location &loc) {
        return false;
    }

    /**
     * @brief Get the current EKF origin
     * 
     * @details Retrieves the origin location used for all relative position estimates.
     *          This is the reference point from which NED position offsets are measured.
     * 
     * @param[out] ret Location structure to receive origin (latitude, longitude, altitude AMSL)
     * 
     * @return true if origin is set and valid, false otherwise
     * 
     * @warning This is a pure virtual method that must be implemented by all concrete backends
     * @note Returns false if origin has not been set yet (no GPS lock)
     */
    virtual bool get_origin(Location &ret) const = 0;

    /**
     * @brief Get 3D position relative to origin in NED frame
     * 
     * @details Returns the vehicle position relative to the EKF origin in meters, expressed
     *          in NED (North-East-Down) frame. This provides high-rate position with EKF
     *          smoothing. Only accurate when inertial navigation is active (GPS available).
     * 
     * @param[out] vec Position vector in meters (North, East, Down from origin)
     * 
     * @return true if position estimate is available and valid, false otherwise
     * 
     * @note Requires EKF backend with GPS or other position source
     * @note Origin must be set before relative positions are valid
     * @note Down component positive means below origin
     */
    virtual bool get_relative_position_NED_origin(Vector3p &vec) const WARN_IF_UNUSED {
        return false;
    }

    /**
     * @brief Get 2D horizontal position relative to origin
     * 
     * @details Returns the horizontal position relative to origin in meters (North and East only).
     *          Used when only horizontal position is needed (saves computation).
     * 
     * @param[out] vecNE Position vector in meters (North and East components only)
     * 
     * @return true if horizontal position estimate is valid, false otherwise
     * 
     * @note More efficient than get_relative_position_NED_origin() when altitude not needed
     */
    virtual bool get_relative_position_NE_origin(Vector2p &vecNE) const WARN_IF_UNUSED {
        return false;
    }

    /**
     * @brief Get vertical position relative to origin
     * 
     * @details Returns the Down position component relative to origin in meters.
     *          Used when only altitude/vertical position is needed.
     * 
     * @param[out] posD Vertical position in meters (positive Down from origin)
     * 
     * @return true if vertical position estimate is valid, false otherwise
     * 
     * @note Positive value means below origin altitude
     * @note More efficient than get_relative_position_NED_origin() when horizontal not needed
     */
    virtual bool get_relative_position_D_origin(postype_t &posD) const WARN_IF_UNUSED {
        return false;
    }

    /**
     * @brief Get scalar ground speed magnitude
     * 
     * @details Returns the magnitude of the horizontal ground velocity vector (2D speed).
     *          This is the vehicle's speed over ground, useful for ground vehicles and
     *          for monitoring aircraft ground track speed.
     * 
     * @return Ground speed in m/s (always positive)
     * 
     * @note Calculated from groundspeed_vector() magnitude
     * @note Used by ground vehicle speed controllers
     */
    float groundspeed(void) {
        return groundspeed_vector().length();
    }

    /**
     * @brief Check if compass is being used for yaw estimation
     * 
     * @details Returns true if the magnetometer (compass) is currently being used as a yaw
     *          (heading) source. May return false if using GPS heading, external navigation,
     *          or if compass has been disabled or failed health checks.
     * 
     * @return true if compass is active yaw source, false otherwise
     * 
     * @warning This is a pure virtual method that must be implemented by all concrete backends
     * @note Affects whether compass calibration is required for arming
     * @note May change during flight if compass fails or GPS heading becomes available
     */
    virtual bool use_compass(void) = 0;

    /**
     * @brief Get attitude as a quaternion
     * 
     * @details Returns the current attitude estimate as a quaternion representing the rotation
     *          from NED (North-East-Down) earth frame to XYZ (Forward-Right-Down) body frame.
     *          Quaternions avoid gimbal lock and are preferred for 3D rotations.
     * 
     * @param[out] quat Quaternion representing NED-to-body rotation
     * 
     * @return true if quaternion is valid, false otherwise
     * 
     * @warning This is a pure virtual method that must be implemented by all concrete backends
     * @note Quaternion is normalized (magnitude = 1.0)
     * @note Can be converted to Euler angles or DCM matrix as needed
     */
    virtual bool get_quaternion(Quaternion &quat) const WARN_IF_UNUSED = 0;

    /**
     * @brief Check overall AHRS health status
     * 
     * @details Returns true if the AHRS system is operating normally with valid sensor inputs
     *          and consistent estimates. Returns false if sensors have failed, estimates have
     *          diverged, or the backend is otherwise not providing reliable state information.
     * 
     * @return true if AHRS is healthy and estimates are reliable, false otherwise
     * 
     * @warning This is a pure virtual method that must be implemented by all concrete backends
     * @note Used for pre-arm checks and in-flight failsafe detection
     * @note May trigger EKF failsafe if returns false during flight
     */
    virtual bool healthy(void) const = 0;

    /**
     * @brief Check if AHRS initialization is complete
     * 
     * @details Returns true if the backend has completed its initialization sequence and is
     *          ready to provide valid estimates. Initialization typically includes waiting for
     *          sensor data, computing initial attitude, and converging filter state.
     * 
     * @return true if initialization complete, false if still initializing
     * 
     * @note Default implementation returns true immediately
     * @note EKF backends may take several seconds to initialize after GPS lock
     */
    virtual bool initialised(void) const {
        return true;
    };

    /**
     * @brief Check if AHRS has started operation
     * 
     * @details Synonym for initialised() - returns true when backend has started and is
     *          providing estimates. Exists for API compatibility.
     * 
     * @return true if AHRS has started, false otherwise
     */
    virtual bool started(void) const {
        return initialised();
    };

    /**
     * @brief Get yaw angle change from last reset
     * 
     * @details Returns the amount of yaw angle discontinuity introduced by the last yaw reset.
     *          Yaw resets can occur when switching to GPS heading, resolving yaw ambiguity,
     *          or recovering from compass failure. Control systems use this to avoid integrator
     *          windup during the reset.
     * 
     * @param[out] yawAng Yaw angle change in radians (positive = clockwise when viewed from above)
     * 
     * @return Timestamp in milliseconds when reset occurred, or 0 if no reset has ever occurred
     * 
     * @note Controllers should reset integrators when a yaw reset is detected
     * @note Large yaw resets may cause brief attitude control transients
     */
    virtual uint32_t getLastYawResetAngle(float &yawAng) {
        return 0;
    };

    /**
     * @brief Get horizontal position change from last reset
     * 
     * @details Returns the North-East position discontinuity introduced by the last position reset.
     *          Position resets occur when switching GPS sources, initializing EKF origin, or
     *          recovering from position estimate divergence. Controllers use this to prevent
     *          large transient errors.
     * 
     * @param[out] pos Position change vector in meters (North and East components)
     * 
     * @return Timestamp in milliseconds when reset occurred, or 0 if no reset has ever occurred
     * 
     * @note Position controllers should account for reset to avoid sudden velocity spikes
     * @note Used by position hold and waypoint navigation modes
     */
    virtual uint32_t getLastPosNorthEastReset(Vector2f &pos) WARN_IF_UNUSED {
        return 0;
    };

    /**
     * @brief Get horizontal velocity change from last reset
     * 
     * @details Returns the North-East velocity discontinuity from the last velocity reset.
     *          Velocity resets can occur when switching velocity sources or recovering from
     *          velocity divergence. Used to prevent integrator windup in velocity controllers.
     * 
     * @param[out] vel Velocity change vector in m/s (North and East components)
     * 
     * @return Timestamp in milliseconds when reset occurred, or 0 if no reset has ever occurred
     * 
     * @note Velocity controllers should reset integrators after velocity reset
     */
    virtual uint32_t getLastVelNorthEastReset(Vector2f &vel) const WARN_IF_UNUSED {
        return 0;
    };

    /**
     * @brief Get vertical position change from last reset
     * 
     * @details Returns the Down position discontinuity from the last altitude reset.
     *          Altitude resets occur when changing height references, initializing origin
     *          altitude, or recovering from altitude estimate divergence.
     * 
     * @param[out] posDelta Vertical position change in meters (positive Down)
     * 
     * @return Timestamp in milliseconds when reset occurred, or 0 if no reset has ever occurred
     * 
     * @note Altitude controllers should account for reset to avoid sudden climb/descent rates
     */
    virtual uint32_t getLastPosDownReset(float &posDelta) WARN_IF_UNUSED {
        return 0;
    };

    /**
     * @brief Reset barometer height datum to current altitude
     * 
     * @details Resets the barometer so it reads zero at the current height, resets EKF height
     *          to zero, and adjusts the EKF origin height so total altitude (EKF height + origin)
     *          remains unchanged. Used to eliminate accumulated barometer drift by establishing
     *          current position as new height reference.
     * 
     * @return true if height datum reset was performed successfully, false if not supported or failed
     * 
     * @note If using rangefinder for height, no reset is performed and returns false
     * @note Used during ground operations to correct for barometer drift
     * @warning Do not use during flight - may cause altitude transients
     */
    virtual bool resetHeightDatum(void) WARN_IF_UNUSED {
        return false;
    }

    /**
     * @brief Get EKF innovation vectors for diagnostics
     * 
     * @details Returns the innovation (measurement minus prediction) for various sensor measurements.
     *          Innovations indicate how well sensor measurements agree with the current state estimate.
     *          Large innovations suggest sensor problems or state estimate divergence. Used for
     *          diagnostics, monitoring, and failsafe logic.
     * 
     * @param[out] velInnov  Velocity innovations in m/s (NED frame)
     * @param[out] posInnov  Position innovations in meters (NED frame)
     * @param[out] magInnov  Magnetometer innovations in Gauss (body frame)
     * @param[out] tasInnov  True airspeed innovation in m/s
     * @param[out] yawInnov  Yaw angle innovation in radians
     * 
     * @return true if innovations are available, false if not supported by backend
     * 
     * @note Only available with EKF backends - DCM returns false
     * @note Innovations near zero indicate good sensor/estimate agreement
     */
    virtual bool get_innovations(Vector3f &velInnov, Vector3f &posInnov, Vector3f &magInnov, float &tasInnov, float &yawInnov) const {
        return false;
    }

    /**
     * @brief Get EKF filter status flags
     * 
     * @details Returns the nav_filter_status structure containing various status flags about
     *          EKF health, sensor usage, and operational mode. Used for monitoring, pre-arm checks,
     *          and in-flight failsafe decisions.
     * 
     * @param[out] status nav_filter_status structure to be populated with status flags
     * 
     * @return true if status is available, false if not supported
     * 
     * @note Contains flags for GPS usage, height source, attitude validity, position validity, etc.
     * @note Only available with EKF backends
     */
    virtual bool get_filter_status(union nav_filter_status &status) const {
        return false;
    }

    /**
     * @brief Get normalized innovation variances
     * 
     * @details Returns innovation variances normalized so that:
     *          - Value of 0 indicates perfect consistency (measurement exactly matches prediction)
     *          - Value of 1 is the maximum inconsistency the filter will accept
     *          - Values above 1 indicate measurement is being rejected or heavily downweighted
     * 
     *          Used for health monitoring and to detect sensor failures or estimate divergence.
     * 
     * @param[out] velVar Normalized velocity innovation variance (dimensionless)
     * @param[out] posVar Normalized position innovation variance (dimensionless)
     * @param[out] hgtVar Normalized height innovation variance (dimensionless)
     * @param[out] magVar Normalized magnetometer innovation variances (X, Y, Z body frame)
     * @param[out] tasVar Normalized true airspeed innovation variance (dimensionless)
     * 
     * @return true if variances available, false if not supported
     * 
     * @note Only available with EKF backends
     * @note Sustained high variances indicate sensor or estimate problems
     */
    virtual bool get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar) const {
        return false;
    }

    /**
     * @brief Get velocity innovations for a specific source
     * 
     * @details Returns velocity innovations and variances for a specific position/velocity source
     *          (e.g., GPS1, GPS2, external nav). Used to monitor health of individual sources in
     *          multi-source EKF configurations and to decide when to switch sources.
     * 
     * @param[in]  source      Source index from 0 to 7 (see AP_NavEKF_Source::SourceXY enum)
     * @param[out] innovations Velocity innovations in m/s (NED frame)
     * @param[out] variances   Normalized innovation variances (dimensionless)
     * 
     * @return true on success, false if source invalid or not supported
     * 
     * @note Only available with EKF3 multi-source configuration
     * @note Used for source selection and failover logic
     */
    virtual bool get_vel_innovations_and_variances_for_source(uint8_t source, Vector3f &innovations, Vector3f &variances) const WARN_IF_UNUSED {
        return false;
    }

    /**
     * @brief Send EKF status telemetry via MAVLink
     * 
     * @details Formats and sends EKF status information via MAVLink to ground control station.
     *          Includes filter health, variances, innovations, and operational status. Used for
     *          real-time monitoring and post-flight analysis.
     * 
     * @param[in] link Reference to MAVLink communications link for sending message
     * 
     * @warning This is a pure virtual method that must be implemented by all concrete backends
     * @note Called periodically by telemetry scheduler (typically 1-5 Hz)
     * @note Message format depends on backend type (EKF_STATUS_REPORT for EKF)
     */
    virtual void send_ekf_status_report(class GCS_MAVLINK &link) const = 0;

    /**
     * @brief Get maximum height limit for control loops
     * 
     * @details Returns the maximum observable height that should be used by control loops,
     *          in meters. This limit is based on EKF covariance and sensor range, preventing
     *          controllers from commanding maneuvers beyond the observable state space.
     * 
     * @param[out] limit Maximum height for control in meters (AMSL or AGL depending on height reference)
     * 
     * @return true if limiting is active and limit is valid, false if no limiting required
     * 
     * @note Used by altitude and position controllers to prevent control divergence
     * @note Returns false when position estimate is good enough that no limiting needed
     */
    virtual bool get_hgt_ctrl_limit(float &limit) const WARN_IF_UNUSED { return false; };

    /**
     * @brief Set terrain stability flag for height reference
     * 
     * @details Informs the backend whether the terrain underneath the vehicle is stable enough
     *          to be used as a height reference (e.g., for rangefinder-based height control).
     *          This is NOT related to terrain following - it's about using ground as a stable
     *          height datum instead of barometric altitude.
     * 
     * @param[in] stable true if terrain is stable (solid ground), false if unstable (water, moving platform)
     * 
     * @note Default implementation ignores flag - override for backends supporting terrain height datum
     * @note Used to switch between barometric and rangefinder height references
     */
    virtual void set_terrain_hgt_stable(bool stable) {}

    /**
     * @brief Get EKF-based control limits
     * 
     * @details Returns control limits based on EKF covariance and innovation consistency.
     *          Used to scale down control authority when position/velocity estimates are uncertain
     *          or when EKF health is degraded, preventing control divergence or instability.
     * 
     * @param[out] ekfGndSpdLimit Maximum ground speed command in m/s based on EKF health
     * @param[out] controlScaleXY Horizontal control scaling factor (0.0 to 1.0, where 1.0 = full authority)
     * 
     * @warning This is a pure virtual method that must be implemented by all concrete backends
     * @note Controllers multiply their outputs by controlScaleXY when EKF health is marginal
     * @note Prevents aggressive maneuvering when position estimates are uncertain
     */
    virtual void get_control_limits(float &ekfGndSpdLimit, float &controlScaleXY) const = 0;
};
