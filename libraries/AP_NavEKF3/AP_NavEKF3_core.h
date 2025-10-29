/**
 * @file AP_NavEKF3_core.h
 * @brief Extended Kalman Filter (EKF3) core computational backend implementation
 * 
 * @details This file defines the NavEKF3_core class, which implements a 24-state 
 *          Extended Kalman Filter for inertial navigation. The filter fuses data from
 *          multiple sensors including IMU, GPS, magnetometer, barometer, airspeed,
 *          optical flow, rangefinder, and other sources to provide robust estimates
 *          of vehicle attitude, velocity, and position.
 * 
 *          The EKF3 implementation is based on the derivation from PX4 ECL:
 *          https://github.com/PX4/ecl/blob/master/matlab/scripts/Inertial%20Nav%20EKF/GenerateNavFilterEquations.m
 *          Originally converted from Matlab to C++ by Paul Riseborough.
 * 
 * **24-State Vector Composition:**
 * - **Quaternion (4)**: Vehicle attitude representation (body frame to NED frame)
 * - **Velocity NED (3)**: North, East, Down velocity in m/s (NED navigation frame)
 * - **Position NED (3)**: North, East, Down position in m (NED navigation frame)
 * - **Gyro Bias (3)**: X, Y, Z gyroscope bias in rad/s (body frame)
 * - **Accel Bias (3)**: X, Y, Z accelerometer bias in m/s² (body frame)
 * - **Earth Magnetic Field NED (3)**: North, East, Down magnetic field in gauss (NED frame)
 * - **Body Magnetic Field XYZ (3)**: X, Y, Z magnetic field bias in gauss (body frame)
 * - **Wind Velocity NE (2)**: North, East wind velocity in m/s (NED frame)
 * 
 * **Coordinate Frame Conventions:**
 * - **NED (North-East-Down)**: Navigation frame fixed to Earth's surface
 * - **Body Frame**: Vehicle-fixed frame (X forward, Y right, Z down)
 * - **Autopilot Frame**: Sensor mounting frame (may differ from body frame by rotation)
 * 
 * **Multi-Core Architecture:**
 * Each NavEKF3_core instance represents a computational backend tied to a specific IMU.
 * The frontend NavEKF3 class manages multiple cores and selects the healthiest one for
 * navigation output. This architecture provides redundancy and robustness against
 * sensor failures.
 * 
 * **Safety-Critical Considerations:**
 * This filter is flight-critical. Errors in state estimation can lead to vehicle crashes.
 * All modifications must be thoroughly tested in Software-In-The-Loop (SITL) simulation
 * and on test vehicles before deployment.
 * 
 * @warning This is safety-critical code. Modifications require extensive validation.
 * 
 * @note This program is free software: you can redistribute it and/or modify
 *       it under the terms of the GNU General Public License as published by
 *       the Free Software Foundation, either version 3 of the License, or
 *       (at your option) any later version.
 * 
 * @copyright Copyright (c) 2016-2025 ArduPilot.org
 * @author Paul Riseborough
 */
#pragma once


#if !defined(HAL_DEBUG_BUILD) || !HAL_DEBUG_BUILD
    #pragma GCC optimize("O2")
#endif

#include "AP_NavEKF3_feature.h"
#include <AP_Common/Location.h>
#include <AP_Math/AP_Math.h>
#include <AP_Math/vectorN.h>
#include <AP_NavEKF/AP_NavEKF_core_common.h>
#include <AP_NavEKF/AP_NavEKF_Source.h>
#include <AP_NavEKF/EKF_Buffer.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_RangeFinder/AP_RangeFinder.h>

#include "AP_NavEKF/EKFGSF_yaw.h"

// GPS pre-flight check bit locations
#define MASK_GPS_NSATS      (1<<0)
#define MASK_GPS_HDOP       (1<<1)
#define MASK_GPS_SPD_ERR    (1<<2)
#define MASK_GPS_POS_ERR    (1<<3)
#define MASK_GPS_YAW_ERR 	(1<<4)
#define MASK_GPS_POS_DRIFT  (1<<5)
#define MASK_GPS_VERT_SPD   (1<<6)
#define MASK_GPS_HORIZ_SPD  (1<<7)

#define earthRate 0.000072921f // earth rotation rate (rad/sec)

// maximum allowed gyro bias (rad/sec)
#define GYRO_BIAS_LIMIT 0.5f

// initial accel bias uncertainty as a fraction of the state limit
#define ACCEL_BIAS_LIM_SCALER 0.2f

// target update time for the EKF in msec and sec
#define EKF_TARGET_DT_MS 12
#define EKF_TARGET_DT    0.012f

// mag fusion final reset altitude (using NED frame so altitude is negative)
#define EKF3_MAG_FINAL_RESET_ALT 2.5f
#define EKF3_MAG_FINAL_RESET_ALT_SUB 0.5f

// learning rate for mag biases when using GPS yaw
#define EK3_GPS_MAG_LEARN_RATE 0.005f

// learning limit for mag biases when using GPS yaw (Gauss)
#define EK3_GPS_MAG_LEARN_LIMIT 0.02f

// maximum number of yaw resets due to detected magnetic anomaly allowed per flight
#define MAG_ANOMALY_RESET_MAX 2

// number of seconds a request to reset the yaw to the GSF estimate is active before it times out
#define YAW_RESET_TO_GSF_TIMEOUT_MS 5000

// accuracy threshold applied to GSF yaw estimate use
#define GSF_YAW_ACCURACY_THRESHOLD_DEG 15.0f

// number of continuous valid GSF yaw estimates required to confirm valid hostory
#define GSF_YAW_VALID_HISTORY_THRESHOLD 5

// minimum variances allowed for velocity and position states
#define VEL_STATE_MIN_VARIANCE 1E-4
#define POS_STATE_MIN_VARIANCE 1E-4

// maximum number of times the vertical velocity variance can hit the lower limit before the
// associated states, variances and covariances are reset
#define EKF_TARGET_RATE_HZ uint32_t(1.0 / EKF_TARGET_DT)
#define VERT_VEL_VAR_CLIP_COUNT_LIM (5 * EKF_TARGET_RATE_HZ)

// limit on horizontal position states
#if HAL_WITH_EKF_DOUBLE
#define EK3_POSXY_STATE_LIMIT 50.0e6
#else
#define EK3_POSXY_STATE_LIMIT 1.0e6
#endif

// IMU acceleration process noise in m/s/s used when bad vibration affected IMU accel is detected
#define BAD_IMU_DATA_ACC_P_NSE 5.0f

// Number of milliseconds of bad IMU data before a reset to vertical position and velocity height sources is performed
#define BAD_IMU_DATA_TIMEOUT_MS 1000

// number of milliseconds the bad IMU data response settings will be held after the last bad IMU data is detected
#define BAD_IMU_DATA_HOLD_MS 10000

// wind state variance limits
#define WIND_VEL_VARIANCE_MAX 400.0f
#define WIND_VEL_VARIANCE_MIN 0.25f

// maximum number of downward facing rangefinder instances available
#if EK3_FEATURE_RANGEFINDER_MEASUREMENTS
#if RANGEFINDER_MAX_INSTANCES > 1
#define DOWNWARD_RANGEFINDER_MAX_INSTANCES 2
#else
#define DOWNWARD_RANGEFINDER_MAX_INSTANCES 1
#endif
#endif

// number of continuous valid GPS velocity samples required to reset yaw
#define GPS_VEL_YAW_ALIGN_COUNT_THRESHOLD 5

// maximum GPs ground course uncertainty allowed for yaw alignment (deg)
#define GPS_VEL_YAW_ALIGN_MAX_ANG_ERR 15.0F

/**
 * @class NavEKF3_core
 * @brief 24-state Extended Kalman Filter computational backend for inertial navigation
 * 
 * @details NavEKF3_core implements a per-IMU computational backend for the EKF3 navigation filter.
 *          Each core instance processes data from a specific IMU and fuses it with other available
 *          sensors to produce state estimates. The frontend NavEKF3 class manages multiple cores
 *          and selects the healthiest one for navigation output, providing sensor redundancy.
 * 
 * **Architecture:**
 * - **Multi-Core Design**: Each core is tied to one IMU (up to 3 cores supported)
 * - **Frontend Selection**: NavEKF3 frontend chooses the healthiest core for output
 * - **Independent Operation**: Each core runs its own prediction and fusion cycles
 * - **Fault Tolerance**: Unhealthy cores are automatically excluded from use
 * 
 * **State Vector (24 states):**
 * 1. Quaternion (4): Attitude representation from body to NED frame
 * 2. Velocity NED (3): North, East, Down velocity in m/s
 * 3. Position NED (3): North, East, Down position in m
 * 4. Gyro Bias (3): X, Y, Z gyroscope bias in rad/s
 * 5. Accel Bias (3): X, Y, Z accelerometer bias in m/s²
 * 6. Earth Mag Field NED (3): North, East, Down magnetic field in gauss
 * 7. Body Mag Field XYZ (3): X, Y, Z body frame magnetic field bias in gauss
 * 8. Wind Velocity NE (2): North, East wind velocity in m/s
 * 
 * **Sensor Fusion Capabilities:**
 * - IMU (gyroscope + accelerometer): Primary inertial measurements
 * - GPS: Position, velocity, and optionally yaw (dual-antenna systems)
 * - Magnetometer: Heading reference and magnetic field anomaly detection
 * - Barometer: Altitude reference
 * - Airspeed: True airspeed for fixed-wing vehicles
 * - Optical Flow: Visual velocity measurement for GPS-denied navigation
 * - Rangefinder: Height above ground for terrain following
 * - External Navigation: Integration with external positioning systems (e.g., visual-inertial odometry)
 * - Body Frame Odometry: Wheel encoders or visual odometry in body frame
 * - Range Beacons: Position triangulation from fixed beacons
 * 
 * **Coordinate Frames:**
 * - **NED Frame**: North-East-Down navigation frame (earth-fixed, local tangent plane)
 * - **Body Frame**: Vehicle-fixed frame (X forward, Y right, Z down)
 * - **Autopilot Frame**: IMU sensor mounting frame (may differ from body by rotation offset)
 * 
 * **Filter Operation Cycle:**
 * 1. **Prediction Step**: Integrate IMU measurements to predict state evolution (typically 400 Hz)
 * 2. **Covariance Prediction**: Update state covariance using process noise model
 * 3. **Sensor Fusion**: Fuse available sensor measurements using Kalman update equations
 * 4. **Health Monitoring**: Track innovation consistency and numerical health
 * 5. **Output**: Provide attitude, velocity, position estimates to flight control
 * 
 * **Health Monitoring:**
 * - Innovation consistency checks: Verify sensor measurements align with predictions
 * - Numerical health: Monitor for covariance divergence or numerical instability
 * - Sensor validity: Track timeout and quality metrics for each sensor
 * - Error scoring: Compute consolidated health score for frontend selection
 * 
 * **Compile-Time Feature Gating:**
 * Various features can be disabled via #if EK3_FEATURE_* directives to reduce
 * code size for platforms with limited flash memory:
 * - EK3_FEATURE_BEACON_FUSION: Range beacon support
 * - EK3_FEATURE_DRAG_FUSION: Multicopter wind estimation via drag model
 * - EK3_FEATURE_BODY_ODOM: Body frame odometry support
 * - EK3_FEATURE_EXTERNAL_NAV: External navigation system integration
 * - EK3_FEATURE_RANGEFINDER_MEASUREMENTS: Rangefinder terrain following
 * 
 * **Thread Safety:**
 * This class is NOT inherently thread-safe. It is designed to be called from the
 * scheduler at a fixed rate. Access from multiple threads requires external synchronization.
 * 
 * **Performance Characteristics:**
 * - Prediction Rate: 400 Hz (EKF_TARGET_DT = 12 ms)
 * - CPU Load: ~2-3% on typical autopilot hardware (depends on enabled features)
 * - Memory: ~20-30 KB per core (depends on buffer sizes and enabled features)
 * 
 * @warning This is flight-critical code. State estimation errors can cause crashes.
 *          All changes require thorough testing in SITL and on test vehicles.
 * 
 * @note Inherits from NavEKF_core_common which provides shared functionality across
 *       EKF2 and EKF3 implementations.
 * 
 * @see NavEKF3 Frontend class managing multiple cores
 * @see state_elements Definition of 24-state vector structure
 * 
 * Source: libraries/AP_NavEKF3/AP_NavEKF3_core.h
 */
class NavEKF3_core : public NavEKF_core_common
{
public:
    /**
     * @brief Constructor for NavEKF3_core backend
     * 
     * @param[in] _frontend Pointer to the NavEKF3 frontend managing this core
     * @param[in] dal Reference to Data Abstraction Layer for sensor access
     * 
     * @note The constructor only initializes pointers. Actual filter setup occurs in setup_core().
     */
    NavEKF3_core(class NavEKF3 *_frontend, class AP_DAL &dal);

    /**
     * @brief Set up this core backend and bind it to a specific IMU
     * 
     * @details Initializes the core's data buffers, state variables, and sensor associations.
     *          Must be called before the filter can be used. Each core is associated with
     *          a specific IMU index for redundancy.
     * 
     * @param[in] _imu_index Index of the IMU this core will use (0-2)
     * @param[in] _core_index Index of this core instance (0-2)
     * 
     * @return true if setup successful, false if setup failed (e.g., invalid indices)
     * 
     * @note Should be called once during system initialization before flight
     */
    bool setup_core(uint8_t _imu_index, uint8_t _core_index);
    
    /**
     * @brief Initialize filter states from accelerometer and magnetometer measurements
     * 
     * @details Performs initial alignment by using gravity vector from accelerometers to
     *          determine roll and pitch, and magnetic field vector to determine yaw heading.
     *          This bootstrap initialization can only be performed when the vehicle is
     *          stationary on the ground.
     * 
     *          Initialization sequence:
     *          1. Average accelerometer readings to determine gravity direction
     *          2. Compute roll and pitch from gravity vector
     *          3. Average magnetometer readings to determine magnetic field direction
     *          4. Compute yaw from magnetic field (accounting for roll/pitch)
     *          5. Initialize quaternion from computed Euler angles
     *          6. Set initial state covariances based on measurement uncertainties
     * 
     * @return true if initialization successful, false if insufficient sensor data
     * 
     * @warning Vehicle MUST be stationary during initialization. Movement will cause
     *          incorrect initial attitude estimates and subsequent filter divergence.
     * 
     * @note Alternative initialization methods exist for specific scenarios (e.g., in-flight restart)
     */
    bool InitialiseFilterBootstrap(void);

    /**
     * @brief Update filter states with new IMU data (main filter processing function)
     * 
     * @details This is the main entry point for the filter's processing cycle. Should be
     *          called every time new IMU data is available (typically 400 Hz). Performs:
     *          1. State prediction using IMU measurements (gyro + accel integration)
     *          2. Covariance prediction with process noise
     *          3. Sensor data fusion when measurements are available
     *          4. State and covariance constraint enforcement
     *          5. Health monitoring and diagnostics
     * 
     *          The filter operates in two phases:
     *          - **Prediction**: Integrate IMU to predict state evolution
     *          - **Update**: Fuse sensor measurements to correct predictions
     * 
     * @param[in] predict true when a new prediction cycle should be started,
     *                    false to skip prediction and only process sensor updates
     * 
     * @note This function is flight-critical and must execute within timing constraints.
     *       Typical execution time: 0.5-2 ms depending on enabled features and active sensors.
     * 
     * @warning Excessive execution time can cause scheduler overruns and flight instability.
     *          Monitor EKF timing in logs if adding computational load.
     */
    void UpdateFilter(bool predict);

    /**
     * @brief Check basic filter health metrics and return consolidated health status
     * 
     * @details Evaluates multiple health criteria to determine if the filter is operating
     *          correctly and can be used for flight control. Health checks include:
     *          - Innovation consistency: Sensor measurements align with predictions
     *          - Numerical stability: Covariances remain positive definite
     *          - Sensor timeouts: Required sensors are providing data
     *          - State bounds: States remain within reasonable physical limits
     *          - Filter convergence: Filter has achieved sufficient accuracy
     * 
     * @return true if filter is healthy and suitable for flight control, false otherwise
     * 
     * @warning Flight control should NOT use estimates from an unhealthy filter.
     *          Doing so can lead to erratic behavior or crashes.
     * 
     * @note Frontend uses this to select between multiple cores. An unhealthy core
     *       will not be selected as the primary navigation source.
     */
    bool healthy(void) const;

    /**
     * @brief Return consolidated error score where higher numbers indicate worse health
     * 
     * @details Computes a numerical health score by combining multiple error metrics:
     *          - Innovation test ratios for each sensor
     *          - State variance magnitudes
     *          - Time since last successful sensor fusion
     *          - Gyro bias magnitude (indicates IMU quality)
     *          - Accel bias magnitude
     * 
     *          The frontend uses this score to select the healthiest core when multiple
     *          cores are available. The core with the LOWEST error score is selected.
     * 
     * @return float Error score (0.0 = perfect health, higher values = worse health)
     * 
     * @note Typical error scores range from 0.0-1.0 for healthy operation.
     *       Scores > 2.0 usually indicate significant problems.
     */
    float errorScore(void) const;

    /**
     * @brief Get last calculated North-East position relative to EKF origin
     * 
     * @details Returns the horizontal position estimate in the NED navigation frame.
     *          Position is relative to the EKF origin which is set when the filter
     *          is first initialized (typically vehicle power-on location or first GPS fix).
     * 
     * @param[out] posNE North and East position components in meters (NED frame)
     * 
     * @return true if calculated solution available and suitable for flight control,
     *         false if using fallback data (do NOT use for flight control)
     * 
     * @warning If false is returned, the output data should not be used for flight control
     *          as it may be stale, inaccurate, or based on dead reckoning.
     * 
     * @note Units: meters
     * @note Coordinate frame: NED navigation frame
     */
    bool getPosNE(Vector2p &posNE) const;

    /**
     * @brief Get Down position from local (EKF) origin
     * 
     * @param[out] posD Down position component in meters (NED frame, negative = up)
     * 
     * @return true if calculated solution available, false if using fallback
     * 
     * @note Units: meters (negative values indicate altitude above origin)
     * @note Coordinate frame: NED navigation frame (Down is positive downward)
     */
    bool getPosD_local(postype_t &posD) const;

    /**
     * @brief Get Down position relative to public (common) origin
     * 
     * @details Returns altitude relative to the public origin shared across all cores.
     *          The public origin may differ from the local EKF origin if the filter
     *          was initialized at a different location or time.
     * 
     * @param[out] posD Down position in meters relative to public origin (NED frame)
     * 
     * @return true if calculated solution available and suitable for flight control,
     *         false if using fallback data (do NOT use for flight control)
     * 
     * @warning If false is returned, do not use output for flight control
     * 
     * @note Units: meters (negative = above origin, positive = below origin)
     * @note Coordinate frame: NED navigation frame
     */
    bool getPosD(postype_t &posD) const;

    /**
     * @brief Return velocity estimate in NED navigation frame
     * 
     * @details Provides the current velocity estimate with components in the North,
     *          East, and Down directions. Always returns a velocity estimate even
     *          if the filter is unhealthy (for logging/monitoring purposes).
     * 
     * @param[out] vel Velocity vector with North, East, Down components in m/s
     * 
     * @note Units: m/s
     * @note Coordinate frame: NED navigation frame
     * @note This method always succeeds (no return value check needed)
     */
    void getVelNED(Vector3f &vel) const;

    /**
     * @brief Return estimated true airspeed vector in body frame
     * 
     * @details Computes the vehicle's velocity relative to the surrounding airmass
     *          by subtracting estimated wind velocity from ground velocity. The result
     *          is rotated into the body frame for use by airspeed controllers.
     * 
     *          Calculation: airspeed_body = Rotation_NED_to_body * (velocity_NED - wind_NED)
     * 
     * @param[out] vel True airspeed vector in body frame (X forward, Y right, Z down) in m/s
     * 
     * @return true if airspeed estimate is available and valid, false otherwise
     * 
     * @note Units: m/s
     * @note Coordinate frame: Body frame (vehicle-fixed)
     * @note Requires valid wind estimate (typically from airspeed sensor fusion)
     */
    bool getAirSpdVec(Vector3f &vel) const;

    /**
     * @brief Return True Airspeed (TAS) innovation health data
     * 
     * @details Provides diagnostic information about the last processed airspeed measurement.
     *          Innovation is the difference between measured and predicted airspeed.
     *          Large innovations indicate sensor problems or filter divergence.
     * 
     * @param[out] innovation Airspeed innovation in m/s (measurement - prediction)
     * @param[out] innovationVariance Innovation variance in (m/s)² (expected innovation magnitude)
     * @param[out] age_ms Age of last airspeed measurement in milliseconds
     * 
     * @return true if airspeed data is available, false otherwise
     * 
     * @note Units: m/s for innovation, (m/s)² for variance, milliseconds for age
     */
    bool getAirSpdHealthData(float &innovation, float &innovationVariance, uint32_t &age_ms) const;

    /**
     * @brief Return rate of change of Down position (vertical velocity derivative)
     * 
     * @details Returns dPosD/dt which may differ from the Down component of velocity state
     *          due to height corrections and error dynamics. However, this derivative is
     *          always kinematically consistent with the Down position state, making it
     *          useful for height rate control that must track position commands.
     * 
     * @return float Vertical velocity in m/s (positive = downward, negative = upward)
     * 
     * @note Units: m/s
     * @note Sign convention: Positive = descending, Negative = climbing (NED frame)
     */
    float getPosDownDerivative(void) const;

    /**
     * @brief Return estimated gyroscope bias in body frame
     * 
     * @details Provides the current estimate of IMU gyroscope bias. These biases are
     *          subtracted from raw gyro measurements before integration. Bias estimation
     *          occurs continuously during flight and adapts to temperature changes and
     *          sensor drift.
     * 
     * @param[out] gyroBias Gyro bias vector for X, Y, Z axes in rad/s (body frame)
     * 
     * @note Units: rad/s
     * @note Coordinate frame: Body frame (X forward, Y right, Z down)
     * @note Typical bias magnitudes: 0.001-0.01 rad/s (0.06-0.6 deg/s)
     */
    void getGyroBias(Vector3f &gyroBias) const;

    /**
     * @brief Return estimated accelerometer bias in body frame
     * 
     * @details Provides the current estimate of IMU accelerometer bias. These biases are
     *          subtracted from raw accelerometer measurements. Bias estimation adapts to
     *          sensor thermal effects and systematic errors during flight.
     * 
     * @param[out] accelBias Accelerometer bias vector for X, Y, Z axes in m/s² (body frame)
     * 
     * @note Units: m/s²
     * @note Coordinate frame: Body frame (X forward, Y right, Z down)
     * @note Typical bias magnitudes: 0.1-0.5 m/s²
     */
    void getAccelBias(Vector3f &accelBias) const;

    /**
     * @brief Reset gyroscope bias estimates to zero
     * 
     * @details Forces gyro bias states to zero and resets their covariances to initial values.
     *          Should only be used when IMU has been recalibrated or replaced. Incorrect
     *          use can cause attitude estimation errors.
     * 
     * @warning Use with caution. Resetting biases during flight can cause temporary
     *          attitude errors until biases reconverge.
     * 
     * @note The filter will re-estimate biases after reset, but this takes time
     */
    void resetGyroBias(void);

    /**
     * @brief Reset height datum to current altitude
     * 
     * @details Resets the barometer and EKF height states so that current altitude becomes
     *          the new zero reference. The EKF origin height is adjusted to maintain global
     *          position consistency (EKF height + origin height remains unchanged).
     * 
     *          This is useful for:
     *          - Zeroing altitude at takeoff location
     *          - Compensating for barometric drift during long flights
     *          - Resetting after changing altitude reference source
     * 
     * @return true if height datum reset was performed, false if reset was not possible
     *         (e.g., when using rangefinder as primary height source)
     * 
     * @note No reset is performed when rangefinder is the primary height source
     * @note This operation is transparent to users - reported altitude remains consistent
     */
    bool resetHeightDatum(void);

    /**
     * @brief Return EKF control limits when using optical flow
     * 
     * @details When optical flow is the primary navigation sensor, velocity noise increases
     *          with altitude due to the fixed angular resolution of the flow sensor.
     *          This function returns:
     *          1. Maximum safe ground speed for reliable flow tracking
     *          2. Gain scaling factor to reduce velocity controller gains at altitude
     * 
     * @param[out] ekfGndSpdLimit Maximum horizontal speed limit in m/s for optical flow
     * @param[out] ekfNavVelGainScaler Velocity gain scale factor (0.0-1.0) to apply at current height
     * 
     * @note Units: m/s for speed limit, dimensionless (0-1) for gain scaler
     * @note Gain scaler decreases with increasing height when using optical flow
     */
    void getEkfControlLimits(float &ekfGndSpdLimit, float &ekfNavVelGainScaler) const;

    /**
     * @brief Return estimated wind velocity in NED frame
     * 
     * @details Provides the estimated wind velocity vector. Wind estimation uses airspeed
     *          measurements (if available) or drag-based estimation for multirotors.
     *          Accurate wind estimates improve position hold and waypoint tracking.
     * 
     * @param[out] wind Wind velocity vector: North, East components in m/s (Down is zero)
     * 
     * @return true if wind state estimation is active and estimates are valid, false otherwise
     * 
     * @note Units: m/s
     * @note Coordinate frame: NED navigation frame (only NE components, Down is always zero)
     * @note Sign convention: Positive = air moving in positive axis direction
     */
    bool getWind(Vector3f &wind) const;

    /**
     * @brief Return estimated Earth magnetic field in NED frame
     * 
     * @details Returns the estimated Earth's magnetic field vector in the NED navigation frame.
     *          These estimates adapt over time to account for local magnetic anomalies and
     *          improve heading estimation accuracy.
     * 
     * @param[out] magNED Magnetic field vector: North, East, Down components in gauss
     * 
     * @note Units: gauss (measurement units / 1000)
     * @note Coordinate frame: NED navigation frame
     * @note Typical Earth field magnitude: 0.25-0.65 gauss depending on location
     */
    void getMagNED(Vector3f &magNED) const;

    /**
     * @brief Return estimated body frame magnetic field bias
     * 
     * @details Returns the estimated magnetic field bias in the vehicle body frame.
     *          Body frame biases account for magnetic interference from vehicle electronics,
     *          motors, and ferromagnetic materials. These estimates improve heading accuracy
     *          by removing vehicle-induced magnetic distortions.
     * 
     * @param[out] magXYZ Magnetic field bias vector: X, Y, Z components in gauss (body frame)
     * 
     * @note Units: gauss (measurement units / 1000)
     * @note Coordinate frame: Body frame (X forward, Y right, Z down)
     * @note These biases are added to measured field to compensate for vehicle interference
     */
    void getMagXYZ(Vector3f &magXYZ) const;

    /**
     * @brief Return index of active airspeed sensor
     * 
     * @return uint8_t Index of currently active airspeed sensor (0-based)
     * 
     * @note Used when multiple airspeed sensors are available for redundancy
     */
    uint8_t getActiveAirspeed() const;

    /**
     * @brief Return estimated magnetometer offset calibration values
     * 
     * @details Provides the estimated hard-iron offset corrections for a specified magnetometer.
     *          These offsets represent constant magnetic biases in the sensor measurements and
     *          are subtracted to improve heading accuracy.
     * 
     * @param[in] mag_idx Index of magnetometer (0-based)
     * @param[out] magOffsets Magnetometer offset vector in gauss (sensor frame)
     * 
     * @return true if magnetometer offsets are valid and available, false otherwise
     * 
     * @note Units: gauss
     * @note Coordinate frame: Magnetometer sensor frame
     */
    bool getMagOffsets(uint8_t mag_idx, Vector3f &magOffsets) const;

    /**
     * @brief Return last calculated position in WGS-84 coordinates (latitude, longitude, altitude)
     * 
     * @details Returns the vehicle's global position. If a calculated EKF solution is not
     *          available (e.g., filter not yet converged), falls back to raw GPS measurement.
     *          
     * @param[out] loc Location structure containing latitude, longitude, altitude (WGS-84)
     * 
     * @return true if any position data is available (calculated or raw), false otherwise
     * 
     * @warning For flight control use, MUST check getFilterStatus() to verify solution quality.
     *          This function returns true even when using fallback GPS data which may be inaccurate.
     * 
     * @note Altitude is AMSL (Above Mean Sea Level) in centimeters
     * @note Latitude/longitude in degrees * 1e7
     */
    bool getLLH(Location &loc) const;

    /**
     * @brief Return EKF NED origin in WGS-84 coordinates
     * 
     * @details The NED origin is the reference point for all NED position calculations.
     *          It is typically set at vehicle power-on or first GPS lock. All relative
     *          positions (North, East, Down) are measured from this origin.
     * 
     * @param[out] loc Origin location containing latitude, longitude, altitude (WGS-84)
     * 
     * @return true if origin has been set, false if origin is not yet established
     * 
     * @note The origin remains fixed unless explicitly changed or filter is reset
     * @note Used to convert between NED (local) and LLH (global) coordinates
     */
    bool getOriginLLH(Location &loc) const;

    /**
     * @brief Set the EKF NED origin to a specified WGS-84 location
     * 
     * @details Establishes the reference point for NED position calculations. Once set,
     *          all position estimates will be relative to this origin. This function can
     *          only succeed if the origin has not been previously set.
     * 
     * @param[in] loc Desired origin location (latitude, longitude, altitude in WGS-84)
     * 
     * @return true if origin was successfully set, false if origin was already established
     * 
     * @warning Cannot change origin after it has been set. This prevents coordinate frame
     *          discontinuities that could disrupt flight control.
     * 
     * @note Typically called automatically on first GPS lock or during filter initialization
     */
    bool setOriginLLH(const Location &loc);

    /**
     * @brief Set EKF horizontal position states from WGS-84 location and accuracy estimate
     * 
     * @details Directly sets the North-East position states and their variances based on
     *          an external position measurement (e.g., GPS, external navigation system).
     *          The altitude component is ignored. Useful for initializing position or
     *          recovering from position divergence.
     * 
     * @param[in] loc WGS-84 location (latitude/longitude used, altitude ignored)
     * @param[in] posAccuracy Estimated 1-sigma horizontal position uncertainty in meters
     * @param[in] timestamp_ms System time of the position measurement in milliseconds
     * 
     * @return true if position was successfully set, false if operation failed
     * 
     * @warning Use with caution. Forcing position states can cause temporary filter
     *          inconsistencies. Ensure posAccuracy reflects true measurement uncertainty.
     * 
     * @note Units: meters for posAccuracy, milliseconds for timestamp
     */
    bool setLatLng(const Location &loc, float posAccuracy, uint32_t timestamp_ms);

    /**
     * @brief Set expected Earth magnetic field from World Magnetic Model (WMM)
     * 
     * @details Populates the Earth magnetic field estimate using WMM tables based on
     *          geographic location. Provides the filter with expected field magnitude
     *          and direction to aid magnetometer fusion and anomaly detection.
     * 
     * @param[in] loc WGS-84 location to query WMM (latitude, longitude, altitude)
     * 
     * @note WMM provides expected field based on global magnetic model
     * @note Helps detect local magnetic anomalies that differ from expected field
     */
    void setEarthFieldFromLocation(const Location &loc);

    /**
     * @brief Return estimated Height Above Ground Level (HAGL)
     * 
     * @details Provides height above the local terrain surface, useful for terrain following,
     *          landing, and obstacle avoidance. Estimated using rangefinder measurements
     *          and/or terrain database when available.
     * 
     * @param[out] HAGL Height above ground level in meters
     * 
     * @return true if HAGL is being estimated and valid, false if not available
     * 
     * @note Units: meters
     * @note Requires rangefinder or terrain database for estimation
     * @note Returns false if ground height is not being tracked
     */
    bool getHAGL(float &HAGL) const;

    /**
     * @brief Return vehicle attitude as Euler angles (roll, pitch, yaw)
     * 
     * @details Converts the internal quaternion attitude representation to Euler angles
     *          in radians. Uses aerospace sequence (ZYX: yaw-pitch-roll).
     * 
     * @param[out] eulers Euler angle vector [roll, pitch, yaw] in radians
     * 
     * @note Units: radians
     * @note Euler angle range: roll ±π, pitch ±π/2, yaw ±π
     * @note Coordinate frame: roll/pitch about body axes, yaw relative to North
     * @note Subject to gimbal lock at pitch = ±90°
     */
    void getEulerAngles(Vector3f &eulers) const;

    /**
     * @brief Return rotation matrix from body frame to NED frame
     * 
     * @details Provides the Direction Cosine Matrix (DCM) that transforms vectors from
     *          body coordinates to NED navigation coordinates. Useful for rotating
     *          body-frame measurements (e.g., airspeed) to navigation frame.
     * 
     *          Usage: vector_NED = mat * vector_body
     * 
     * @param[out] mat 3x3 rotation matrix (body to NED transformation)
     * 
     * @note Rotation direction: Body frame → NED frame
     * @note Transpose gives NED → Body transformation
     */
    void getRotationBodyToNED(Matrix3f &mat) const;

    /**
     * @brief Return attitude quaternion (rotation from NED to body frame)
     * 
     * @details Provides the quaternion representation of vehicle attitude. Quaternions
     *          avoid gimbal lock and are computationally efficient for attitude propagation.
     *          Represents rotation from NED navigation frame to body frame.
     * 
     * @param[out] quat Attitude quaternion [q0, q1, q2, q3] (NED to body rotation)
     * 
     * @note Quaternion convention: q = [q0, q1, q2, q3] = [cos(θ/2), sin(θ/2)*axis]
     * @note Unit quaternion: q0² + q1² + q2² + q3² = 1
     * @note Rotation direction: NED frame → Body frame
     */
    void getQuaternion(Quaternion &quat) const;

    /**
     * @brief Return measurement innovations for position, velocity, magnetometer, and airspeed
     * 
     * @details Innovations represent the difference between sensor measurements and filter predictions.
     *          Large innovations indicate either sensor problems or filter divergence. Used for
     *          health monitoring and diagnostics.
     * 
     *          Innovation = Measurement - Prediction
     * 
     * @param[out] velInnov Velocity innovation vector [N, E, D] in m/s (NED frame)
     * @param[out] posInnov Position innovation vector [N, E, D] in m (NED frame)
     * @param[out] magInnov Magnetometer innovation vector [X, Y, Z] in gauss (body frame)
     * @param[out] tasInnov True airspeed innovation in m/s
     * @param[out] yawInnov Yaw angle innovation in radians
     * 
     * @return true if innovation data is available, false otherwise
     * 
     * @note Units: m/s for velocity, m for position, gauss for mag, m/s for airspeed, rad for yaw
     * @note Used for innovation consistency checks and sensor health monitoring
     */
    bool getInnovations(Vector3f &velInnov, Vector3f &posInnov, Vector3f &magInnov, float &tasInnov, float &yawInnov) const;

    /**
     * @brief Return synthetic air data innovations (drag and sideslip)
     * 
     * @details For multicopters, drag and sideslip models can be used to estimate wind.
     *          This function returns the innovations for these synthetic measurements.
     * 
     * @param[out] dragInnov Drag force innovations [X, Y] in m/s² (body frame)
     * @param[out] betaInnov Sideslip angle innovation in radians
     * 
     * @note Units: m/s² for drag, radians for sideslip
     * @note Only relevant for multicopter wind estimation
     */
    void getSynthAirDataInnovations(Vector2f &dragInnov, float &betaInnov) const;

    /**
     * @brief Return innovation variance test ratios for filter health monitoring
     * 
     * @details Provides innovation variance test ratios that indicate sensor health and
     *          filter consistency. Test ratio = (innovation² / innovation_variance).
     *          Values significantly above 1.0 indicate potential sensor faults or filter problems.
     * 
     * @param[out] velVar Velocity innovation variance test ratio (dimensionless)
     * @param[out] posVar Position innovation variance test ratio (dimensionless)
     * @param[out] hgtVar Height innovation variance test ratio (dimensionless)
     * @param[out] magVar Magnetometer innovation variance test ratios [X, Y, Z] (dimensionless)
     * @param[out] tasVar True airspeed innovation variance test ratio (dimensionless)
     * @param[out] offset Position measurement offset in m (for diagnostics)
     * 
     * @return true if variance data is available, false otherwise
     * 
     * @note Test ratios < 1.0 indicate good consistency (measurement within expected noise)
     * @note Test ratios > 5.0 typically trigger innovation rejection
     */
    bool getVariances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar, Vector2f &offset) const;

    /**
     * @brief Get velocity innovations and variances for a specific position source
     * 
     * @details Returns innovation data for a particular positioning source (GPS, external nav, etc.)
     *          to enable source-specific health monitoring and fault detection.
     * 
     * @param[in] source Position source identifier (GPS1, GPS2, ExternalNav, etc.)
     * @param[out] innovations Velocity innovations [N, E, D] in m/s
     * @param[out] variances Innovation variances [N, E, D] in (m/s)²
     * 
     * @return true on success with valid data, false if source not active or data unavailable
     * 
     * @note Units: m/s for innovations, (m/s)² for variances
     */
    bool getVelInnovationsAndVariancesForSource(AP_NavEKF_Source::SourceXY source, Vector3f &innovations, Vector3f &variances) const WARN_IF_UNUSED;

    /**
     * @brief Check if compass should be used for heading estimation
     * 
     * @details Determines whether magnetometer data is suitable for yaw estimation based on:
     *          - Compass calibration status
     *          - Magnetic interference levels
     *          - Alternative yaw sources availability (GPS yaw, external nav)
     *          - Vehicle type (some vehicles don't require compass)
     * 
     * @return true if compass should be used, false if compass should not be used
     * 
     * @note Public method to allow AHRS layer to report compass usage status
     * @note Used by ahrs.use_compass() for GCS reporting
     */
    bool use_compass(void) const;

    /**
     * @brief Write raw optical flow measurements to the EKF for fusion
     * 
     * @details Optical flow sensors measure apparent angular motion of the ground, which can be
     *          converted to linear velocity when combined with altitude. Used for GPS-denied
     *          navigation. This function writes raw sensor data for EKF processing.
     * 
     * **Sign Convention:**
     * Right-hand physical rotation about an axis produces positive flow and gyro rates.
     * 
     * @param[in] rawFlowQuality Quality metric 0-255 (255 = best quality, 0 = invalid)
     * @param[in] rawFlowRates Optical flow rates [X, Y] in rad/s (sensor frame)
     * @param[in] rawGyroRates Sensor's internal gyro rates [X, Y] in rad/s for motion compensation
     * @param[in] msecFlowMeas Scheduler timestamp when flow data received (milliseconds)
     * @param[in] posOffset Flow sensor position offset [X, Y, Z] from IMU in m (body frame)
     * @param[in] heightOverride Fixed height above ground in m for rovers (0 = use EKF height estimate)
     * 
     * @note Units: rad/s for flow rates, milliseconds for timestamp, meters for offsets
     * @note Flow measurements are delayed; EKF compensates using timestamp
     * @note Quality threshold for fusion typically 50-100 depending on conditions
     */
    void writeOptFlowMeas(const uint8_t rawFlowQuality, const Vector2f &rawFlowRates, const Vector2f &rawGyroRates, const uint32_t msecFlowMeas, const Vector3f &posOffset, float heightOverride);

    /**
     * @brief Retrieve latest corrected optical flow sample (for calibration purposes)
     * 
     * @details Returns flow measurements after EKF corrections have been applied. Used for
     *          flow sensor calibration and diagnostic purposes.
     * 
     * @param[out] timeStamp_ms Timestamp of flow sample in milliseconds
     * @param[out] flowRate Corrected flow rate [X, Y] in rad/s
     * @param[out] bodyRate Vehicle body rotation rate [X, Y] in rad/s
     * @param[out] losPred Predicted line-of-sight rate [X, Y] in rad/s
     * 
     * @return true if flow sample is available, false otherwise
     * 
     * @note Units: milliseconds for timestamp, rad/s for rates
     */
    bool getOptFlowSample(uint32_t& timeStamp_ms, Vector2f& flowRate, Vector2f& bodyRate, Vector2f& losPred) const;

    /**
     * @brief Write body frame odometry measurements from visual odometry sensor
     * 
     * @details Processes delta position and delta angle measurements from visual odometry systems
     *          (e.g., visual-inertial odometry, T265 camera). Measurements are in body frame
     *          and relative to an inertial reference. Used for GPS-denied navigation.
     * 
     *          Integration with EKF:
     *          - Delta position fused as velocity measurement
     *          - Delta angle cross-checked with IMU gyro integration
     *          - Quality metric used to weight measurements
     * 
     * @param[in] quality Normalized confidence value 0-100 (100 = highest confidence)
     * @param[in] delPos Change in position [X, Y, Z] over delTime in m (body frame)
     * @param[in] delAng Angular rotation [roll, pitch, yaw] over delTime in rad (body frame)
     * @param[in] delTime Time interval for delPos and delAng measurements in seconds
     * @param[in] timeStamp_ms Timestamp of last image/measurement in milliseconds
     * @param[in] delay_ms Average measurement delay relative to IMU in milliseconds
     * @param[in] posOffset Camera/sensor position [X, Y, Z] from IMU in m (body frame)
     * 
     * @note Units: meters for position, radians for angles, seconds/milliseconds for time
     * @note Coordinate frame: Body frame (X forward, Y right, Z down)
     * @note Quality < 50 typically indicates poor tracking conditions
     * 
     * @warning Sensor delay must be accurately characterized for proper fusion
     */
    void writeBodyFrameOdom(float quality, const Vector3f &delPos, const Vector3f &delAng, float delTime, uint32_t timeStamp_ms, uint16_t delay_ms, const Vector3f &posOffset);

    /**
     * @brief Write wheel encoder odometry data for ground vehicles
     * 
     * @details Processes wheel rotation measurements to estimate vehicle velocity. Used primarily
     *          for rovers and ground vehicles. Assumes wheel rotation axis is parallel to vehicle
     *          body Y axis (lateral axis).
     * 
     *          Velocity calculation: velocity = (delAng * radius) / delTime
     * 
     * @param[in] delAng Change in wheel angular position in radians (positive = forward motion)
     * @param[in] delTime Time interval for delAng measurement in seconds
     * @param[in] timeStamp_ms Timestamp of measurement in milliseconds
     * @param[in] posOffset Wheel hub position [X, Y, Z] from IMU in m (body frame)
     * @param[in] radius Effective rolling radius of wheel in meters
     * 
     * @note Units: radians for angle, seconds/milliseconds for time, meters for position/radius
     * @note Assumes wheel rotation axis parallel to body Y axis
     * @note Wheel slip degrades velocity estimate accuracy
     */
    void writeWheelOdom(float delAng, float delTime, uint32_t timeStamp_ms, const Vector3f &posOffset, float radius);

    /**
     * @brief Return body frame odometry fusion debug data
     * 
     * @details Provides diagnostic information for debugging body frame odometry (visual odometry,
     *          wheel encoders) fusion performance.
     * 
     * @param[out] velInnov Velocity innovations [X, Y, Z] in m/s (body frame)
     * @param[out] velInnovVar Velocity innovation variances [X, Y, Z] in (m/s)² (body frame)
     * 
     * @return uint32_t Timestamp of last odometry fusion update in milliseconds
     * 
     * @note Units: m/s for innovations, (m/s)² for variances, milliseconds for timestamp
     * @note Coordinate frame: Body frame
     */
    uint32_t getBodyFrameOdomDebug(Vector3f &velInnov, Vector3f &velInnovVar);

    /**
     * @brief Write yaw angle measurement from external sensor
     * 
     * @details Processes direct yaw angle measurements from sensors like GPS compasses (dual antenna)
     *          or external heading sensors. Provides absolute heading reference independent of
     *          magnetometer.
     * 
     * **Rotation Convention:**
     * Positive yaw angle = right-hand rotation about Z body axis (clockwise viewed from above)
     * 
     * **Euler Rotation Sequences:**
     * - Type 1: 312 (ZXY) - Yaw first, then Roll, then Pitch
     * - Type 2: 321 (ZYX) - Yaw first, then Pitch, then Roll (most common)
     * 
     * @param[in] yawAngle Yaw angle relative to true north in radians
     * @param[in] yawAngleErr 1-sigma yaw measurement uncertainty in radians
     * @param[in] timeStamp_ms System time of measurement including all delays (milliseconds)
     * @param[in] type Euler rotation order: 1 = 312 (ZXY), 2 = 321 (ZYX)
     * 
     * @note Units: radians for angle and error, milliseconds for timestamp
     * @note Timestamp must include all measurement lag and transmission delays
     * @note Type 2 (321/ZYX) is the standard aerospace sequence
     */
    void writeEulerYawAngle(float yawAngle, float yawAngleErr, uint32_t timeStamp_ms, uint8_t type);

    /**
     * @brief Write position and attitude data from external navigation system
     * 
     * @details Integrates external navigation sources such as motion capture systems, external SLAM,
     *          or visual-inertial odometry systems that provide full 6-DOF pose (position + orientation).
     * 
     * @param[in] pos Position [N, E, D] or [X, Y, Z] in navigation frame (m)
     * @param[in] quat Attitude quaternion (navigation frame to body frame rotation)
     * @param[in] posErr 1-sigma spherical position uncertainty in meters
     * @param[in] angErr 1-sigma spherical angular uncertainty in radians
     * @param[in] timeStamp_ms System time measurement was taken (not received) in milliseconds
     * @param[in] delay_ms Average measurement delay relative to IMU in milliseconds
     * @param[in] resetTime_ms System time of last position reset request in milliseconds
     * 
     * @note Units: meters for position, radians for angles, milliseconds for time
     * @note Navigation frame assumed to be NED-aligned
     * @note Quaternion represents rotation from navigation frame to body frame
     * 
     * @warning Measurement delay must be accurately characterized for proper fusion
     */
    void writeExtNavData(const Vector3f &pos, const Quaternion &quat, float posErr, float angErr, uint32_t timeStamp_ms, uint16_t delay_ms, uint32_t resetTime_ms);

    /**
     * @brief Write velocity data from external navigation system
     * 
     * @details Processes velocity measurements from external navigation sources. Useful when
     *          external system provides velocity estimates (e.g., visual odometry, Doppler radar).
     * 
     * @param[in] vel Velocity [N, E, D] in m/s (NED frame)
     * @param[in] err 1-sigma velocity uncertainty in m/s
     * @param[in] timeStamp_ms System time measurement was taken (not received) in milliseconds
     * @param[in] delay_ms Average measurement delay relative to IMU in milliseconds
     * 
     * @note Units: m/s for velocity and error, milliseconds for time
     * @note Coordinate frame: NED navigation frame
     */
    void writeExtNavVelData(const Vector3f &vel, float err, uint32_t timeStamp_ms, uint16_t delay_ms);

    /**
     * @brief Set whether terrain underneath is stable for height reference
     * 
     * @details Controls whether rangefinder measurements should be used as height reference.
     *          Set false when flying over water, moving obstacles, or unstable terrain to prevent
     *          rangefinder-induced height errors. Used in conjunction with EK3_RNG_USE_HGT and
     *          EK3_RNG_USE_SPD parameters.
     * 
     * @param[in] val true if terrain is stable (use rangefinder), false if unstable (ignore rangefinder)
     * 
     * @note Prevents rangefinder fusion when terrain unsuitable for height reference
     * @note Useful for transitioning between stable and unstable terrain
     */
    void setTerrainHgtStable(bool val);

    /**
     * @brief Return filter fault status as bitmask
     * 
     * @details Provides detailed fault information for diagnosing filter problems. Each bit
     *          represents a specific fault condition. Multiple faults can occur simultaneously.
     * 
     * **Fault Bit Definitions:**
     * - Bit 0: Quaternions are NaN (catastrophic attitude failure)
     * - Bit 1: Velocities are NaN (catastrophic velocity failure)
     * - Bit 2: Badly conditioned X magnetometer fusion
     * - Bit 3: Badly conditioned Y magnetometer fusion
     * - Bit 4: Badly conditioned Z magnetometer fusion
     * - Bit 5: Badly conditioned airspeed fusion
     * - Bit 6: Badly conditioned synthetic sideslip fusion
     * - Bit 7: Filter is not initialized
     * 
     * @param[out] faults Bitmask of active faults (0 = no faults)
     * 
     * @warning Any non-zero fault status indicates problems requiring investigation
     * @note Badly conditioned fusion indicates numerical instability or sensor malfunction
     */
    void getFilterFaults(uint16_t &faults) const;

    /**
     * @brief Return comprehensive filter status information
     * 
     * @details Provides detailed status of filter operation including output validity,
     *          flight phase detection, and sensor usage. Used by flight control and GCS
     *          to determine filter health and operational mode.
     * 
     * **Status Information Includes:**
     * - Which state outputs are valid (attitude, velocity, position)
     * - Takeoff detection status
     * - Ground effect compensation active
     * - GPS data usage status
     * - Compass usage status
     * - Flight phase (pre-arm, armed, flying)
     * 
     * @param[out] status Structure containing comprehensive filter status flags
     * 
     * @note Essential for flight control to determine which EKF outputs can be used
     * @see nav_filter_status structure definition for all available fields
     */
    void getFilterStatus(nav_filter_status &status) const;

    /**
     * @brief Send EKF status report message to ground control station
     * 
     * @details Transmits comprehensive EKF health and performance data via MAVLink for
     *          real-time monitoring and post-flight analysis.
     * 
     * @param[in] link MAVLink communication channel to send message on
     * 
     * @note Message type: EKF_STATUS_REPORT
     * @note Includes innovation test ratios, variances, and health metrics
     */
    void send_status_report(class GCS_MAVLINK &link) const;

    /**
     * @brief Provide height control limit when using optical flow navigation
     * 
     * @details When optical flow is the primary navigation sensor, imposes maximum altitude
     *          limit to maintain acceptable velocity estimation accuracy. Flow velocity
     *          measurement quality degrades with increasing altitude due to fixed angular
     *          resolution.
     * 
     * @param[out] height Maximum altitude limit in meters (negative in NED frame)
     * 
     * @return true if height limiting is required, false if no limit needed
     * 
     * @note Units: meters (NED frame - negative = above ground)
     * @note Only applicable when using optical flow for primary navigation
     * @note Flight controller should enforce this limit to prevent navigation degradation
     */
    bool getHeightControlLimit(float &height) const;

    /**
     * @brief Return yaw angle change from last reset and reset timestamp
     * 
     * @details When the filter performs a yaw reset (e.g., due to magnetic anomaly or GPS yaw
     *          initialization), this function reports the magnitude of the reset for flight
     *          controller compensation.
     * 
     * @param[out] yawAng Yaw angle change in radians (applied to correct heading)
     * 
     * @return uint32_t Timestamp of last yaw reset in milliseconds (0 if never reset)
     * 
     * @note Units: radians for angle, milliseconds for timestamp
     * @note Flight controller uses this to adjust heading-dependent calculations
     * @note Returns 0 timestamp if no reset has occurred since filter initialization
     */
    uint32_t getLastYawResetAngle(float &yawAng) const;

    /**
     * @brief Return North-East position change from last reset and reset timestamp
     * 
     * @details When the filter performs a position reset (e.g., GPS glitch recovery, external
     *          navigation initialization), this function reports the position discontinuity
     *          for flight controller compensation.
     * 
     * @param[out] pos Position change [North, East] in meters
     * 
     * @return uint32_t Timestamp of last position reset in milliseconds (0 if never reset)
     * 
     * @note Units: meters for position, milliseconds for timestamp
     * @note Flight controller uses this to adjust position-dependent waypoint tracking
     * @note Returns 0 timestamp if no reset has occurred since filter initialization
     */
    uint32_t getLastPosNorthEastReset(Vector2f &pos) const;

    /**
     * @brief Return Down position change from last reset and reset timestamp
     * 
     * @details When the filter performs a vertical position reset (e.g., barometer glitch recovery,
     *          terrain reference change), this function reports the altitude discontinuity for
     *          flight controller compensation.
     * 
     * @param[out] posD Position change in Down direction in meters (NED frame)
     * 
     * @return uint32_t Timestamp of last down position reset in milliseconds (0 if never reset)
     * 
     * @note Units: meters for position (positive = down in NED), milliseconds for timestamp
     * @note Flight controller uses this to adjust altitude-dependent control loops
     * @note Returns 0 timestamp if no reset has occurred since filter initialization
     */
    uint32_t getLastPosDownReset(float &posD) const;

    /**
     * @brief Return North-East velocity change from last reset and reset timestamp
     * 
     * @details When the filter performs a velocity reset (e.g., GPS velocity jump recovery),
     *          this function reports the velocity discontinuity for flight controller
     *          compensation and acceleration limiting.
     * 
     * @param[out] vel Velocity change [North, East] in m/s
     * 
     * @return uint32_t Timestamp of last velocity reset in milliseconds (0 if never reset)
     * 
     * @note Units: m/s for velocity, milliseconds for timestamp
     * @note Flight controller uses this to prevent false acceleration spikes
     * @note Returns 0 timestamp if no reset has occurred since filter initialization
     */
    uint32_t getLastVelNorthEastReset(Vector2f &vel) const;

    /**
     * @brief Report reason why EKF backend is refusing to initialize (pre-arm check)
     * 
     * @details During pre-arm checks, if the filter cannot initialize, this provides a
     *          human-readable explanation. Common reasons include insufficient GPS quality,
     *          excessive gyro drift, or magnetometer calibration problems.
     * 
     * @return const char* Pointer to failure reason string, or nullptr if no failure
     * 
     * @note Used by arming checks to provide user feedback
     * @note String remains valid until next call or filter state change
     */
    const char *prearm_failure_reason(void) const;

    /**
     * @brief Report number of frames since last state prediction
     * 
     * @details Used for load balancing across multiple EKF instances. Instances that haven't
     *          updated recently can be prioritized to maintain balanced computational load
     *          across all active cores.
     * 
     * @return uint8_t Number of scheduler frames elapsed since last prediction update
     * 
     * @note Used by frontend for multi-core load balancing
     * @note Lower values indicate more recently updated instance
     */
    uint8_t getFramesSincePredict(void) const;

    /**
     * @brief Get the IMU index associated with this EKF core
     * 
     * @details Each EKF core instance is bound to a specific IMU. This returns the index
     *          of the gyro (and associated IMU) being used. Critical for subsystems that
     *          need to know which sensor set is driving which filter.
     * 
     * @return uint8_t Active gyro/IMU index (0-based)
     * 
     * @note Returns gyro index as it is most critical for other subsystems
     * @note Used for sensor selection and failover management
     */
    uint8_t getIMUIndex(void) const { return gyro_index_active; }

    /**
     * @enum MagCal
     * @brief Magnetometer calibration modes for EK3_MAG_CAL parameter
     * 
     * @details Controls when in-flight magnetometer calibration (learning) is enabled.
     *          Determines strategy for updating magnetometer bias estimates during flight.
     */
    enum class MagCal {
        WHEN_FLYING = 0,        ///< Learn mag biases only when vehicle is flying
        WHEN_MANOEUVRING = 1,   ///< Learn mag biases during manoeuvres (more aggressive)
        NEVER = 2,              ///< Never learn mag biases in flight (use ground cal only)
        AFTER_FIRST_CLIMB = 3,  ///< Start learning after first climb (conservative)
        ALWAYS = 4              ///< Always learn mag biases (most aggressive)
        // 5 was EXTERNAL_YAW (deprecated - do not use)
        // 6 was EXTERNAL_YAW_FALLBACK (deprecated - do not use)
    };

    /**
     * @enum MagFuseSel
     * @brief Magnetometer fusion mode selection
     * 
     * @details Indicates current magnetometer fusion strategy. Filter can use full 3-axis
     *          magnetometer data, only yaw component, or no magnetometer at all.
     */
    enum class MagFuseSel {
        NOT_FUSING = 0,  ///< Not fusing magnetometer data (using alternative yaw source)
        FUSE_YAW = 1,    ///< Fusing only yaw component of magnetometer
        FUSE_MAG = 2     ///< Fusing full 3-axis magnetometer measurements
    };

    /**
     * @brief Check if using non-compass yaw source
     * 
     * @details Returns true if filter is using an alternative yaw source instead of compass
     *          (e.g., GPS yaw, external navigation yaw, GSF yaw estimator).
     * 
     * @return true if using non-compass yaw source, false if using compass
     * 
     * @note Important for determining heading source reliability
     * @note Used to report yaw source to GCS and logging
     */
    bool using_noncompass_for_yaw(void) const;

    /**
     * @brief Check if using external navigation for yaw
     * 
     * @details Returns true if filter is currently fusing yaw measurements from external
     *          navigation system (e.g., motion capture, external SLAM).
     * 
     * @return true if fusing external nav yaw, false otherwise
     * 
     * @note Subset of non-compass yaw sources
     */
    bool using_extnav_for_yaw() const;

    /**
     * @brief Write default airspeed to use when measured airspeed unavailable
     * 
     * @details Provides fallback airspeed estimate for fixed-wing vehicles when pitot tube
     *          is not available or has failed. Used in airspeed-dependent calculations like
     *          drag modeling and wind estimation.
     * 
     * @param[in] airspeed Default equivalent airspeed in m/s
     * @param[in] uncertainty 1-sigma airspeed uncertainty in m/s
     * 
     * @note Units: m/s for both airspeed and uncertainty
     * @note Typically derived from throttle setting and vehicle performance model
     * @note Only used when actual airspeed sensor unavailable
     */
    void writeDefaultAirSpeed(float airspeed, float uncertainty);

    /**
     * @brief Request yaw reset to EKF-GSF (Gaussian Sum Filter) estimate
     * 
     * @details Triggers a yaw reset using the GSF yaw estimator, which is robust to
     *          magnetometer anomalies. Used for recovering from magnetic interference
     *          or poor compass calibration.
     * 
     * @note GSF runs in parallel and provides backup yaw estimate
     * @note Request times out after YAW_RESET_TO_GSF_TIMEOUT_MS (5 seconds)
     * @note Only applied if GSF estimate has sufficient accuracy
     * 
     * @warning Can cause temporary attitude disturbance during reset
     */
    void EKFGSF_requestYawReset();

    /**
     * @brief Check if tilt alignment is complete
     * 
     * @details During filter initialization, roll and pitch estimates must converge before
     *          full navigation can begin. Returns true once horizontal attitude is sufficiently
     *          aligned using accelerometer gravity vector.
     * 
     * @return true if tilt (roll/pitch) alignment complete, false if still initializing
     * 
     * @note Tilt alignment typically completes within 1-2 seconds of stationary IMU data
     * @note Prerequisite for yaw alignment
     */
    bool have_aligned_tilt(void) const {
        return tiltAlignComplete;
    }

    /**
     * @brief Check if yaw alignment is complete
     * 
     * @details After tilt alignment, yaw must be initialized using magnetometer, GPS, or
     *          other heading source. Returns true once heading estimate has converged.
     * 
     * @return true if yaw alignment complete, false if still initializing
     * 
     * @note Yaw alignment requires valid heading source (compass, GPS yaw, etc.)
     * @note Full 6-DOF navigation only available after both tilt and yaw aligned
     */
    bool have_aligned_yaw(void) const {
        return yawAlignComplete;
    }

    /**
     * @brief Write EKF state and diagnostic data to onboard log
     * 
     * @details Records filter state vector, covariances, innovations, and health metrics
     *          to dataflash log for post-flight analysis and debugging.
     * 
     * @param[in] time_us System timestamp in microseconds
     * 
     * @note Units: microseconds for timestamp
     * @note Creates XKF1-XKF5 and multiple NKF log messages
     * @note Essential for post-flight EKF performance analysis
     */
    void Log_Write(uint64_t time_us);

    /**
     * @brief Check if state estimates are degraded by vibration
     * 
     * @details High vibration levels corrupt IMU measurements and can cause filter divergence.
     *          This function detects when vibration-induced errors are significantly affecting
     *          state estimation accuracy.
     * 
     * @return true if vibration is significantly degrading estimates, false if acceptable
     * 
     * @note Triggers increased process noise to handle bad IMU data
     * @note Flight controller may limit maneuvers when vibration detected
     * @note Logged for post-flight vibration analysis
     */
    bool isVibrationAffected() const { return badIMUdata; }

    /**
     * @brief Get pointer to GSF (Gaussian Sum Filter) yaw estimator instance
     * 
     * @details Provides access to the parallel GSF yaw estimator that runs alongside the
     *          main EKF. GSF provides robust yaw backup independent of magnetometer, useful
     *          for detecting compass failures and recovering from magnetic anomalies.
     * 
     * @return const EKFGSF_yaw* Pointer to GSF yaw estimator instance, or nullptr if not initialized
     * 
     * @note GSF uses multiple parallel filters with different initial yaw hypotheses
     * @note Provides backup yaw when compass unreliable
     */
    const EKFGSF_yaw *get_yawEstimator(void) const { return yawEstimator; }

    /**
     * @brief Perform per-core pre-arm safety checks
     * 
     * @details Validates that this EKF core is ready for flight. Checks sensor health,
     *          filter convergence, initialization status, and position accuracy. Prevents
     *          arming if critical issues detected.
     * 
     * **Checks Performed:**
     * - Filter initialized and aligned
     * - IMU data quality acceptable
     * - Position accuracy within limits (if required)
     * - Velocity estimates valid
     * - Attitude estimates valid
     * - Innovation test ratios within bounds
     * 
     * @param[in] requires_position true to enforce horizontal position accuracy checks
     * @param[out] failure_msg Buffer to receive failure description if check fails
     * @param[in] failure_msg_len Size of failure_msg buffer in bytes
     * 
     * @return true if all pre-arm checks pass (safe to arm), false if checks fail
     * 
     * @note Failure message provides user-friendly explanation for arming prevention
     * @note Position checks may be disabled for indoor/optical flow flight modes
     * 
     * @warning Vehicle must not arm if this returns false - unsafe to fly
     */
    bool pre_arm_check(bool requires_position, char *failure_msg, uint8_t failure_msg_len) const;
    
private:
    EKFGSF_yaw *yawEstimator;
    AP_DAL &dal;

    // Reference to the global EKF frontend for parameters
    class NavEKF3 *frontend;
    uint8_t imu_index; // preferred IMU index
    uint8_t gyro_index_active; // active gyro index (in case preferred fails)
    uint8_t accel_index_active; // active accel index (in case preferred fails)
    uint8_t core_index;
    uint8_t imu_buffer_length;
    uint8_t obs_buffer_length;

#if MATH_CHECK_INDEXES
    class Vector9 : public VectorN<ftype, 9> {
    public:
        Vector9(ftype p0, ftype p1, ftype p2,
                ftype p3, ftype p4, ftype p5,
                ftype p6, ftype p7, ftype p8) {
            _v[0] = p0;   _v[1] = p1;  _v[2] = p2;
            _v[3] = p3;   _v[4] = p4;  _v[5] = p5;
            _v[6] = p6;   _v[7] = p7;  _v[8] = p8;
        }
    };
    class Vector5 : public VectorN<ftype, 5> {
    public:
        Vector5(ftype p0, ftype p1, ftype p2,
                ftype p3, ftype p4) {
            _v[0] = p0;   _v[1] = p1;  _v[2] = p2;
            _v[3] = p3;   _v[4] = p4;
        }
    };

    typedef VectorN<ftype,2> Vector2;
    typedef VectorN<ftype,3> Vector3;
    typedef VectorN<ftype,4> Vector4;
    typedef VectorN<ftype,6> Vector6;
    typedef VectorN<ftype,7> Vector7;
    typedef VectorN<ftype,8> Vector8;
    typedef VectorN<ftype,10> Vector10;
    typedef VectorN<ftype,11> Vector11;
    typedef VectorN<ftype,13> Vector13;
    typedef VectorN<ftype,14> Vector14;
    typedef VectorN<ftype,15> Vector15;
    typedef VectorN<ftype,21> Vector21;
    typedef VectorN<ftype,22> Vector22;
    typedef VectorN<ftype,23> Vector23;
    typedef VectorN<ftype,24> Vector24;
    typedef VectorN<ftype,25> Vector25;
    typedef VectorN<ftype,31> Vector31;
    typedef VectorN<VectorN<ftype,3>,3> Matrix3;
    typedef VectorN<VectorN<ftype,24>,24> Matrix24;
    typedef VectorN<VectorN<ftype,34>,50> Matrix34_50;
    typedef VectorN<uint32_t,50> Vector_u32_50;
#else
    typedef ftype Vector2[2];
    typedef ftype Vector3[3];
    typedef ftype Vector4[4];
    typedef ftype Vector5[5];
    typedef ftype Vector6[6];
    typedef ftype Vector7[7];
    typedef ftype Vector8[8];
    typedef ftype Vector9[9];
    typedef ftype Vector10[10];
    typedef ftype Vector11[11];
    typedef ftype Vector13[13];
    typedef ftype Vector14[14];
    typedef ftype Vector15[15];
    typedef ftype Vector21[21];
    typedef ftype Vector22[22];
    typedef ftype Vector23[23];
    typedef ftype Vector24[24];
    typedef ftype Vector25[25];
    typedef ftype Matrix3[3][3];
    typedef ftype Matrix24[24][24];
    typedef ftype Matrix34_50[34][50];
    typedef uint32_t Vector_u32_50[50];
#endif

    /**
     * @struct state_elements
     * @brief The 24-state EKF3 state vector broken down by element
     * 
     * @details Defines the complete Extended Kalman Filter state vector for EKF3. This structure
     *          provides named access to state elements, while the union with statesArray allows
     *          vector math operations on the full 24-element state.
     * 
     * **State Vector Composition (24 states total):**
     * - States 0-3:   Attitude quaternion (4 states)
     * - States 4-6:   Velocity NED (3 states)
     * - States 7-9:   Position NED (3 states)
     * - States 10-12: Gyro biases (3 states)
     * - States 13-15: Accel biases (3 states)
     * - States 16-18: Earth magnetic field (3 states)
     * - States 19-21: Body magnetic field (3 states)
     * - States 22-23: Wind velocity NE (2 states)
     * 
     * **Design Note:**
     * State vector available in two equivalent forms sharing same memory:
     * 1. Vector24 statesArray - for matrix math operations
     * 2. struct state_elements - for named field access
     * 
     * **Coordinate Frames:**
     * - Attitude: NED to Body frame rotation
     * - Velocity/Position: NED navigation frame
     * - Gyro/Accel biases: Body frame
     * - Earth magfield: NED navigation frame
     * - Body magfield: Body frame
     * - Wind: NED navigation frame (horizontal only)
     * 
     * @note All elements use floating-point type (ftype) for precision
     * @note Quaternion normalized to unit length during propagation
     */
    struct state_elements {
        QuaternionF quat;           ///< Attitude quaternion (NED to body frame) - States 0..3
        Vector3F    velocity;       ///< Velocity [N,E,D] in m/s (NED frame) - States 4..6
        Vector3F    position;       ///< Position [N,E,D] in m (NED frame) - States 7..9
        Vector3F    gyro_bias;      ///< Gyro bias [X,Y,Z] in rad (body frame) - States 10..12
        Vector3F    accel_bias;     ///< Accel bias [X,Y,Z] in m/s (body frame) - States 13..15
        Vector3F    earth_magfield; ///< Earth mag field [N,E,D] in gauss (NED frame) - States 16..18
        Vector3F    body_magfield;  ///< Body mag field [X,Y,Z] in gauss (body frame) - States 19..21
        Vector2F    wind_vel;       ///< Wind velocity [N,E] in m/s (NED frame, horizontal only) - States 22..23
    };

    /**
     * @brief Union providing dual access to state vector
     * 
     * @details Allows states to be accessed either as a 24-element array for matrix operations
     *          or as named structure members for readability. Both views share the same memory.
     * 
     * - statesArray: For covariance propagation, Kalman updates, and vector math
     * - stateStruct: For intuitive named access to individual state components
     */
    union {
        Vector24 statesArray;              ///< Array form for matrix operations
        struct state_elements stateStruct; ///< Struct form for named access
    };

    /**
     * @struct output_elements
     * @brief Delayed and corrected state outputs for flight control
     * 
     * @details EKF outputs are buffered and corrected for IMU-to-body frame offsets before
     *          being provided to the flight controller. This structure holds the output state
     *          estimates at the body frame origin rather than the IMU location.
     * 
     * **Key Differences from Internal States:**
     * - Time delayed to match sensor fusion delays
     * - Position/velocity corrected for IMU lever arm offset
     * - Represents body frame origin, not IMU location
     * 
     * @note Units: quaternion (dimensionless), velocity (m/s), position (m)
     * @note Coordinate frame: NED navigation frame for position/velocity
     */
    struct output_elements {
        QuaternionF quat;     ///< Attitude quaternion at body frame origin (NED to body)
        Vector3F    velocity; ///< Velocity [N,E,D] at body frame origin in m/s (NED frame)
        Vector3F    position; ///< Position [N,E,D] of body frame origin in m (NED frame)
    };

    /**
     * @struct imu_elements
     * @brief Buffered IMU measurement data for state propagation
     * 
     * @details Stores incremental IMU measurements (delta angles and delta velocities) rather
     *          than instantaneous rates. Allows exact integration regardless of sample rate
     *          variations. Buffered for time-delayed fusion with slower sensors.
     * 
     * @note Units: radians for delAng, m/s for delVel, seconds for DT, milliseconds for timestamp
     * @note Coordinate frame: Body frame (IMU sensor frame)
     */
    struct imu_elements {
        Vector3F    delAng;      ///< Delta angle [X,Y,Z] over sample period in rad (body frame)
        Vector3F    delVel;      ///< Delta velocity [X,Y,Z] over sample period in m/s (body frame)
        ftype       delAngDT;    ///< Time interval for delAng measurement in seconds
        ftype       delVelDT;    ///< Time interval for delVel measurement in seconds
        uint32_t    time_ms;     ///< Measurement timestamp in milliseconds
        uint8_t     gyro_index;  ///< Gyro sensor index (for multi-IMU systems)
        uint8_t     accel_index; ///< Accelerometer sensor index (for multi-IMU systems)
    };

    /**
     * @struct gps_elements
     * @brief Buffered GPS measurement data for position/velocity fusion
     * 
     * @details Stores GPS position (lat/lon/alt) and velocity measurements for fusion into EKF.
     *          Inherits from EKF_obs_element_t for timestamp and buffering support.
     * 
     * @note Units: 1e-7 degrees for lat/lon, meters for height/position, m/s for velocity
     * @note Coordinate frame: WGS-84 for lat/lon, NED for height and velocity
     * @note Measurements corrected for GPS antenna offset from IMU
     */
    struct gps_elements : EKF_obs_element_t {
        int32_t     lat, lng;    ///< Latitude/longitude in 1e-7 degrees (WGS-84)
        ftype       hgt;         ///< GPS antenna height in m (NED frame, negative = above datum)
        Vector3F    vel;         ///< GPS velocity [N,E,D] in m/s (NED frame)
        uint8_t     sensor_idx;  ///< Unique GPS sensor identifier (for multi-GPS systems)
        bool        corrected;   ///< true if corrected for antenna offset from IMU
        bool        have_vz;     ///< true if vertical velocity (Down component) is valid
    };

    /**
     * @struct mag_elements
     * @brief Buffered magnetometer measurement data for heading fusion
     * 
     * @details Stores 3-axis magnetometer measurements for fusion into EKF. Used for yaw
     *          estimation and magnetic field state updates.
     * 
     * @note Units: gauss
     * @note Coordinate frame: Body frame (sensor frame)
     */
    struct mag_elements : EKF_obs_element_t {
        Vector3F    mag;  ///< Magnetic field [X,Y,Z] in gauss (body frame)
    };

    /**
     * @struct baro_elements
     * @brief Buffered barometer measurement data for altitude fusion
     * 
     * @details Stores barometric altitude measurements for vertical position estimation.
     *          Primary source of altitude when GPS unavailable or unreliable.
     * 
     * @note Units: meters (NED frame)
     * @note Coordinate frame: NED (negative = above datum)
     */
    struct baro_elements : EKF_obs_element_t {
        ftype       hgt;  ///< Barometric height in m (NED frame, negative = above datum)
    };

    /**
     * @struct range_elements
     * @brief Buffered rangefinder measurement data for terrain following
     * 
     * @details Stores distance-to-ground measurements from rangefinder (lidar, sonar, etc.)
     *          for terrain-relative altitude estimation and precision landing.
     * 
     * @note Units: meters (always positive, measured downward)
     * @note Up to two rangefinders supported simultaneously
     */
    struct range_elements : EKF_obs_element_t {
        ftype       rng;         ///< Range measurement in meters (distance to ground)
        uint8_t     sensor_idx;  ///< Sensor identifier: 0 or 1 (for dual rangefinder systems)
    };

    /**
     * @struct rng_bcn_elements
     * @brief Buffered range beacon measurement for indoor positioning
     * 
     * @details Stores range measurements to fixed beacons at known positions for GPS-denied
     *          navigation. Multiple beacons used for trilateration-based position estimation.
     * 
     * @note Units: meters for range and position
     * @note Coordinate frame: NED for beacon position
     */
    struct rng_bcn_elements : EKF_obs_element_t {
        ftype       rng;            ///< Range to beacon in meters
        Vector3F    beacon_posNED;  ///< Beacon position [N,E,D] in m (NED frame)
        ftype       rngErr;         ///< 1-sigma range measurement error in meters
        uint8_t     beacon_ID;      ///< Unique beacon identification number
    };

    /**
     * @struct tas_elements
     * @brief Buffered true airspeed measurement for wind estimation
     * 
     * @details Stores true airspeed measurements from pitot-static system for fixed-wing
     *          aircraft. Fused with GPS ground speed to estimate wind velocity.
     * 
     * @note Units: m/s for airspeed, (m/s)² for variance
     * @note True airspeed = indicated airspeed corrected for altitude and temperature
     */
    struct tas_elements : EKF_obs_element_t {
        ftype       tas;          ///< True airspeed measurement in m/s
        ftype       tasVariance;  ///< Measurement variance in (m/s)²
        bool        allowFusion;  ///< true if measurement allowed to update states
    };

    /**
     * @struct of_elements
     * @brief Buffered optical flow measurement data for velocity estimation
     * 
     * @details Stores optical flow sensor measurements for GPS-denied navigation. Flow provides
     *          angular velocity of features in the sensor field of view, which combined with
     *          height yields ground velocity estimate. Critical for indoor/GPS-denied navigation.
     * 
     * @note Units: rad/s for flow rates, m for offsets, m for heightOverride
     * @note Coordinate frame: Body frame for all measurements and offsets
     * @note Motion compensation: flowRadXYcomp has vehicle rotation removed from raw flowRadXY
     */
    struct of_elements : EKF_obs_element_t {
        Vector2F    flowRadXY;      ///< Raw optical flow angular rates [X,Y] in rad/s (body frame)
        Vector2F    flowRadXYcomp;  ///< Motion-compensated flow rates [X,Y] in rad/s (body frame)
        Vector3F    bodyRadXYZ;     ///< Body angular rates [X,Y,Z] averaged over flow interval in rad/s
        Vector3F    body_offset;    ///< Flow sensor position [X,Y,Z] from IMU in m (body frame)
        float       heightOverride; ///< Fixed sensor height above ground in m (rovers only, 0 if unused)
    };

    /**
     * @struct vel_odm_elements
     * @brief Buffered body-frame velocity odometry measurement data
     * 
     * @details Stores velocity measurements from external odometry sources (e.g., visual-inertial
     *          odometry, wheel odometry fusion systems). Provides body-frame velocity for aiding
     *          EKF in GPS-denied environments.
     * 
     * @note Units: m/s for velocity and error, m for offset, rad/s for angular rate
     * @note Coordinate frame: Body frame for all measurements
     * @note Lever arm correction: body_offset allows correcting velocity for sensor location
     */
    struct vel_odm_elements : EKF_obs_element_t {
        Vector3F        vel;        ///< Velocity [X,Y,Z] in body frame in m/s
        ftype           velErr;     ///< 1-sigma velocity measurement uncertainty in m/s
        Vector3F        body_offset;///< Odometry sensor position [X,Y,Z] from IMU in m (body frame)
        Vector3F        angRate;    ///< Angular rate [X,Y,Z] from odometry in rad/s (body frame)
    };

    /**
     * @struct wheel_odm_elements
     * @brief Buffered wheel odometry measurement data for ground vehicles
     * 
     * @details Stores wheel encoder measurements for rover/ground vehicle velocity estimation.
     *          Provides highly accurate forward velocity when wheels maintain traction. Essential
     *          for GPS-denied ground vehicle navigation.
     * 
     * @note Units: radians for wheel angle, meters for radius and offset, seconds for time
     * @note Coordinate frame: Body frame (positive delAng = forward vehicle motion)
     * @note Assumes no wheel slip - reduce weight if slip detected
     */
    struct wheel_odm_elements : EKF_obs_element_t {
        ftype           delAng;     ///< Incremental wheel rotation angle in rad (positive = forward)
        ftype           radius;     ///< Wheel radius in meters
        Vector3F        hub_offset; ///< Wheel hub position [X,Y,Z] from IMU in m (body frame)
        ftype           delTime;    ///< Measurement accumulation interval in seconds
    };
        
    /**
     * @enum rotationOrder
     * @brief Specifies rotation sequence for Euler angle conversions
     * 
     * @details Defines the axis rotation order when converting between quaternions/DCM and
     *          Euler angles. Different sensors and systems use different conventions.
     * 
     * - TAIT_BRYAN_321 (ZYX): Yaw-Pitch-Roll sequence (most common in aerospace)
     * - TAIT_BRYAN_312 (ZXY): Yaw-Roll-Pitch sequence (alternative convention)
     */
    enum class rotationOrder {
        TAIT_BRYAN_321=0, ///< ZYX rotation order (yaw-pitch-roll)
        TAIT_BRYAN_312=1  ///< ZXY rotation order (yaw-roll-pitch)
    };

    /**
     * @struct yaw_elements
     * @brief Buffered yaw angle measurement data
     * 
     * @details Stores direct yaw measurements from sources like GPS heading, external compass,
     *          or vision systems. Provides absolute heading reference without magnetometer.
     * 
     * @note Units: radians for angle and uncertainty
     * @note Coordinate frame: Yaw relative to North (navigation frame)
     * @note Rotation order must match source convention for correct fusion
     */
    struct yaw_elements : EKF_obs_element_t {
        ftype         yawAng;         ///< Yaw angle measurement in radians (CW from North)
        ftype         yawAngErr;      ///< 1-sigma yaw measurement uncertainty in radians
        rotationOrder order;          ///< Euler rotation sequence used (321=ZYX, 312=ZXY)
    };

    /**
     * @struct ext_nav_elements
     * @brief Buffered external navigation position measurement data
     * 
     * @details Stores position measurements from external navigation systems (e.g., motion capture,
     *          RTK GPS, SLAM). Provides absolute position reference in right-handed navigation frame.
     * 
     * @note Units: meters for position and error
     * @note Coordinate frame: Right-handed navigation frame (typically NED or ENU)
     * @note Reset flag: posReset indicates position reference frame change requiring filter adaptation
     */
    struct ext_nav_elements : EKF_obs_element_t {
        Vector3F        pos;        ///< Position [X,Y,Z] in navigation frame in meters
        ftype           posErr;     ///< Spherical 1-sigma position uncertainty in meters
        bool            posReset;   ///< true if position reference has been reset
        bool            corrected;  ///< true if corrected for sensor offset from IMU
    };

    /**
     * @struct ext_nav_vel_elements
     * @brief Buffered external navigation velocity measurement data
     * 
     * @details Stores velocity measurements from external navigation systems. Complements
     *          position measurements for improved dynamic response and position estimation.
     * 
     * @note Units: m/s for velocity and error
     * @note Coordinate frame: NED navigation frame
     * @note Typically fused alongside ext_nav_elements position
     */
    struct ext_nav_vel_elements : EKF_obs_element_t {
        Vector3F vel;               ///< Velocity [N,E,D] in m/s (NED frame)
        ftype err;                  ///< 1-sigma velocity measurement uncertainty in m/s
        bool corrected;             ///< true if corrected for sensor offset from IMU
    };

    /**
     * @struct drag_elements
     * @brief Buffered aerodynamic drag measurement data
     * 
     * @details Stores specific force measurements along body X and Y axes for drag-based
     *          airspeed estimation. Uses vehicle drag model to infer airspeed from
     *          accelerometer measurements. Useful when airspeed sensor unavailable.
     * 
     * @note Units: m/s² (specific force, not mass-dependent)
     * @note Coordinate frame: Body frame X-Y axes
     * @note Requires accurate drag coefficient model for reliable airspeed estimate
     */
    struct drag_elements : EKF_obs_element_t {
        Vector2f accelXY;       ///< Specific force [X,Y] in m/s² (body frame)
    };

    /**
     * @brief Bias estimates for inactive IMUs in multi-IMU systems
     * 
     * @details Tracks gyro and accelerometer biases for IMUs that are present but not
     *          currently selected as primary by this EKF core. Allows seamless IMU switching
     *          without bias convergence delays.
     * 
     * @note Units: radians for gyro_bias, m/s for accel_bias
     * @note Coordinate frame: Body frame (IMU sensor frame)
     * @note Array sized for maximum number of IMU instances supported
     */
    struct {
        Vector3F gyro_bias;  ///< Gyro bias [X,Y,Z] in radians (body frame)
        Vector3F accel_bias; ///< Accel bias [X,Y,Z] in m/s (body frame)
    } inactiveBias[INS_MAX_INSTANCES];

    /**
     * @enum resetDataSource
     * @brief Specifies data source for partial state reset operations
     * 
     * @details When the filter needs to reset specific states (e.g., position after GPS glitch,
     *          yaw after magnetic anomaly), this enum specifies which sensor to use as reference.
     * 
     * **Reset Source Options:**
     * - DEFAULT: Filter selects best available source automatically
     * - GPS: Use GPS position/velocity
     * - RNGBCN: Use range beacon triangulation
     * - FLOW: Use optical flow velocity
     * - BARO: Use barometer altitude
     * - MAG: Use magnetometer heading
     * - RNGFND: Use rangefinder height
     * - EXTNAV: Use external navigation system
     * 
     * @warning Caller must verify data source availability and quality before requesting reset
     */
    enum class resetDataSource {
        DEFAULT=0,      ///< Use filter's internal source selection logic
        GPS=1,          ///< Use GPS measurements
        RNGBCN=2,       ///< Use range beacon measurements
        FLOW=3,         ///< Use optical flow measurements
        BARO=4,         ///< Use barometer measurements
        MAG=5,          ///< Use magnetometer measurements
        RNGFND=6,       ///< Use rangefinder measurements
        EXTNAV=7        ///< Use external navigation measurements
    };

    /**
     * @enum yawFusionMethod
     * @brief Specifies active method for yaw (heading) estimation
     * 
     * @details EKF3 supports multiple yaw estimation methods depending on sensor availability
     *          and environmental conditions. This enum tracks which method is currently active.
     * 
     * **Yaw Estimation Methods:**
     * - MAGNETOMETER: Traditional compass-based heading (default)
     * - GPS: GPS velocity vector or dual-antenna GPS heading
     * - GSF: Gaussian Sum Filter multi-hypothesis yaw estimation
     * - STATIC: Static yaw (no updates, for stationary operation)
     * - PREDICTED: Yaw propagated from gyros only (degraded mode)
     * - EXTNAV: External navigation system provides heading
     * 
     * @note Method selection impacts heading accuracy and magnetic interference sensitivity
     * @note GSF useful for yaw initialization when magnetometer unreliable
     */
    enum class yawFusionMethod {
	    MAGNETOMETER=0, ///< Magnetometer-based heading estimation
	    GPS=1,          ///< GPS velocity or dual-antenna heading
        GSF=2,          ///< Gaussian Sum Filter multi-hypothesis estimation
        STATIC=3,       ///< Static yaw (no updates)
        PREDICTED=4,    ///< Gyro-only propagation (no corrections)
        EXTNAV=5,       ///< External navigation system heading
    };

    // update the navigation filter status
    void updateFilterStatus(void);

    // update the quaternion, velocity and position states using IMU measurements
    void UpdateStrapdownEquationsNED();

    // calculate the predicted state covariance matrix
    // Argument rotVarVecPtr is pointer to a vector defining the earth frame uncertainty variance of the quaternion states
    // used to perform a reset of the quaternion state covariances only. Set to null for normal operation.
    void CovariancePrediction(Vector3F *rotVarVecPtr);

    // force symmetry on the state covariance matrix
    void ForceSymmetry();

    // constrain variances (diagonal terms) in the state covariance matrix
    void ConstrainVariances();

    // constrain states
    void ConstrainStates();

    // constrain earth field using WMM tables
    void MagTableConstrain(void);

    // fuse selected position, velocity and height measurements
    void FuseVelPosNED();

    // fuse body frame velocity measurements
    void FuseBodyVel();

#if EK3_FEATURE_BEACON_FUSION
    // fuse range beacon measurements
    void FuseRngBcn();
#endif

    // use range beacon measurements to calculate a static position
    void FuseRngBcnStatic();

    // calculate the offset from EKF vertical position datum to the range beacon system datum
    void CalcRangeBeaconPosDownOffset(ftype obsVar, Vector3F &vehiclePosNED, bool aligning);

    // fuse magnetometer measurements
    void FuseMagnetometer();

    // fuse true airspeed measurements
    void FuseAirspeed();

    // fuse synthetic sideslip measurement of zero
    void FuseSideslip();

    // zero specified range of rows in the state covariance matrix
    void zeroRows(Matrix24 &covMat, uint8_t first, uint8_t last);

    // zero specified range of columns in the state covariance matrix
    void zeroCols(Matrix24 &covMat, uint8_t first, uint8_t last);

    // Reset the stored output history to current data
    void StoreOutputReset(void);

    // Reset the stored output quaternion history to current EKF state
    void StoreQuatReset(void);

    // Rotate the stored output quaternion history through a quaternion rotation
    void StoreQuatRotate(const QuaternionF &deltaQuat);

    // calculate the NED earth spin vector in rad/sec
    void calcEarthRateNED(Vector3F &omega, int32_t latitude) const;

    // initialise the covariance matrix
    void CovarianceInit();

    // helper functions for readIMUData
    bool readDeltaVelocity(uint8_t ins_index, Vector3F &dVel, ftype &dVel_dt);
    bool readDeltaAngle(uint8_t ins_index, Vector3F &dAng, ftype &dAng_dt);

    // helper functions for correcting IMU data
    void correctDeltaAngle(Vector3F &delAng, ftype delAngDT, uint8_t gyro_index);
    void correctDeltaVelocity(Vector3F &delVel, ftype delVelDT, uint8_t accel_index);

    // update IMU delta angle and delta velocity measurements
    void readIMUData(bool startPredictEnabled);

    // update estimate of inactive bias states
    void learnInactiveBiases();

    // check for new valid GPS data and update stored measurement if available
    void readGpsData();

    // check for new valid GPS yaw data
    void readGpsYawData();

    // check for new altitude measurement data and update stored measurement if available
    void readBaroData();

    // check for new magnetometer data and update store measurements if available
    void readMagData();

    // try changing compasses on compass failure or timeout
    void tryChangeCompass(void);

    // try changing to a specific compass index
    void tryChangeCompass(uint8_t compass_index);

    // check for new airspeed data and update stored measurements if available
    void readAirSpdData();

#if EK3_FEATURE_BEACON_FUSION
    // check for new range beacon data and update stored measurements if available
    void readRngBcnData();
#endif

    // determine when to perform fusion of GPS position and  velocity measurements
    void SelectVelPosFusion();

#if EK3_FEATURE_BEACON_FUSION
    // determine when to perform fusion of range measurements take relative to a beacon at a known NED position
    void SelectRngBcnFusion();
#endif

    // determine when to perform fusion of magnetometer measurements
    void SelectMagFusion();

    // determine when to perform fusion of true airspeed measurements
    void SelectTasFusion();

    // determine when to perform fusion of drag or synthetic sideslip measurements
    void SelectBetaDragFusion();

    // force alignment of the yaw angle using GPS velocity data
    void realignYawGPS(bool emergency_reset);

    // initialise the earth magnetic field states using declination and current attitude and magnetometer measurements

    // align the yaw angle for the quaternion states to the given yaw angle which should be at the fusion horizon
    void alignYawAngle(const yaw_elements &yawAngData);

    // update mag field states and associated variances using magnetomer and declination data
    void resetMagFieldStates();

    // reset yaw based on magnetic field sample
    void setYawFromMag();

    // zero stored variables
    void InitialiseVariables();

    // zero stored variables related to mag
    void InitialiseVariablesMag();

    // reset the horizontal position states uing the last GPS measurement
    void ResetPosition(resetDataSource posResetSource);

    // reset the stateStruct's NE position to the specified position
    void ResetPositionNE(ftype posN, ftype posE);

    // reset the stateStruct's D position
    void ResetPositionD(ftype posD);

    // reset velocity states using the last GPS measurement
    void ResetVelocity(resetDataSource velResetSource);

    // reset the vertical position state using the last height measurement
    void ResetHeight(void);

    // return true if we should use the airspeed sensor
    bool useAirspeed(void) const;

    // return true if the vehicle code has requested the filter to be ready for flight
    bool readyToUseGPS(void) const;

    // return true if the filter to be ready to use the beacon range measurements
    bool readyToUseRangeBeacon(void) const;

    // Check for filter divergence
    void checkDivergence(void);

    // Calculate weighting that is applied to IMU1 accel data to blend data from IMU's 1 and 2
    void calcIMU_Weighting(ftype K1, ftype K2);

#if EK3_FEATURE_OPTFLOW_FUSION
    // return true if the filter is ready to start using optical flow measurements for position and velocity estimation
    bool readyToUseOptFlow(void) const;
#endif

    // return true if the filter is ready to start using body frame odometry measurements
    bool readyToUseBodyOdm(void) const;

    // return true if the filter to be ready to use external nav data
    bool readyToUseExtNav(void) const;

    // return true if we should use the range finder sensor
    bool useRngFinder(void) const;

#if EK3_FEATURE_OPTFLOW_FUSION
    // determine when to perform fusion of optical flow measurements
    void SelectFlowFusion();
#endif

    // determine when to perform fusion of body frame odometry measurements
    void SelectBodyOdomFusion();

    // Estimate terrain offset using a single state EKF
    void EstimateTerrainOffset(const of_elements &ofDataDelayed);

#if EK3_FEATURE_OPTFLOW_FUSION
    // fuse optical flow measurements into the main filter
    // really_fuse should be true to actually fuse into the main filter, false to only calculate variances
    void FuseOptFlow(const of_elements &ofDataDelayed, bool really_fuse);
#endif

    // Control filter mode changes
    void controlFilterModes();

    // Determine if we are flying or on the ground
    void detectFlight();

    // set the default yaw source
    void setYawSource();

    // Set inertial navigation aiding mode
    void setAidingMode();

    // Determine if learning of wind and magnetic field will be enabled and set corresponding indexing limits to
    // avoid unnecessary operations
    void setWindMagStateLearningMode();

    // Check the alignmnent status of the tilt attitude
    // Used during initial bootstrap alignment of the filter
    void checkAttitudeAlignmentStatus();

    // Control reset of yaw and magnetic field states
    void controlMagYawReset();

    // set the latitude and longitude and height used to set the NED origin
    // All NED positions calculated by the filter will be relative to this location
    // returns false if the origin has already been set
    bool setOrigin(const Location &loc);

    // Assess GPS data quality and set gpsGoodToAlign
    void calcGpsGoodToAlign(void);

    // set the class variable true if the delta angle bias variances are sufficiently small
    void checkGyroCalStatus(void);

    // update inflight calculaton that determines if GPS data is good enough for reliable navigation
    void calcGpsGoodForFlight(void);

    // Read the range finder and take new measurements if available
    // Apply a median filter to range finder data
    void readRangeFinder();

#if EK3_FEATURE_OPTFLOW_FUSION
    // check if the vehicle has taken off during optical flow navigation by looking at inertial and range finder data
    void detectOptFlowTakeoff(void);
#endif

    // align the NE earth magnetic field states with the published declination
    void alignMagStateDeclination();

    // Fuse compass measurements using a direct yaw angle observation (doesn't require magnetic field states)
    // Returns true if the fusion was successful
    bool fuseEulerYaw(yawFusionMethod method);

    // return the best Tait-Bryan rotation order to use
    void bestRotationOrder(rotationOrder &order);

    // Fuse declination angle to keep earth field declination from changing when we don't have earth relative observations.
    // Input is 1-sigma uncertainty in published declination
    void FuseDeclination(ftype declErr);

    // return magnetic declination in radians
    ftype MagDeclination(void) const;

    // Propagate PVA solution forward from the fusion time horizon to the current time horizon
    // using a simple observer
    void calcOutputStates();

    // calculate a filtered offset between baro height measurement and EKF height estimate
    void calcFiltBaroOffset();

    // correct the height of the EKF origin to be consistent with GPS Data using a Bayes filter.
    void correctEkfOriginHeight();

    // Select height data to be fused from the available baro, range finder and GPS sources
    void selectHeightForFusion();

    // zero attitude state covariances, but preserve variances
    void zeroAttCovOnly();

    // record all requested yaw resets completed
    void recordYawResetsCompleted();

    // record a magnetic field state reset event
    void recordMagReset();

    // effective value of MAG_CAL
    MagCal effective_magCal(void) const;

    // calculate the tilt error variance
    void calcTiltErrorVariance(void);
    
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    // calculate the tilt error variance using an alternative numerical difference technique
    // and log with value generated by NavEKF3_core::calcTiltErrorVariance()
    void verifyTiltErrorVariance();
#endif

    // update timing statistics structure
    void updateTimingStatistics(void);

    // Update the state index limit based on which states are active
    void updateStateIndexLim(void);

    // correct GPS data for antenna position
    void CorrectGPSForAntennaOffset(gps_elements &gps_data) const;

    // correct external navigation earth-frame position using sensor body-frame offset
    void CorrectExtNavForSensorOffset(ext_nav_elements &ext_nav_data);

    // correct external navigation earth-frame velocity using sensor body-frame offset
    void CorrectExtNavVelForSensorOffset(ext_nav_vel_elements &ext_nav_vel_data) const;

    // calculate velocity variances and innovations
    // Scale factor applied to NE velocity measurement variance due to manoeuvre acceleration
    // Scale factor applied to vertical velocity measurement variance due to manoeuvre acceleration
    // variances argument is updated with variances for each axis
    void CalculateVelInnovationsAndVariances(const Vector3F &velocity, ftype noise, ftype accel_scale, Vector3F &innovations, Vector3F &variances) const;

    // Runs the IMU prediction step for an independent GSF yaw estimator algorithm
    // that uses IMU, GPS horizontal velocity and optionally true airspeed data.
    void runYawEstimatorPrediction(void);

    // Run the GPS velocity correction step for the GSF yaw estimator and use the
    // yaw estimate to reset the main EKF yaw if requested
    void runYawEstimatorCorrection(void);

    // reset the quaternion states using the supplied yaw angle, maintaining the previous roll and pitch
    // also reset the body to nav frame rotation matrix
    // reset the quaternion state covariances using the supplied yaw variance
    // yaw          : new yaw angle (rad)
    // yaw_variance : variance of new yaw angle (rad^2)
    // order : enum defining Tait-Bryan rotation order used in calculation of the yaw angle
    void resetQuatStateYawOnly(ftype yaw, ftype yawVariance, rotationOrder order);

    // attempt to reset the yaw to the EKF-GSF value
    // emergency_reset should be true if this reset is triggered by the loss of the yaw estimate
    // returns false if unsuccessful
    bool EKFGSF_resetMainFilterYaw(bool emergency_reset);

    // returns true on success and populates yaw (in radians) and yawVariance (rad^2)
    bool EKFGSF_getYaw(ftype &yaw, ftype &yawVariance) const;

    // Fusion of body frame X and Y axis drag specific forces for multi-rotor wind estimation
    void FuseDragForces();
    void SelectDragFusion();
    void SampleDragData(const imu_elements &imu);

    bool getGPSLLH(Location &loc) const;

    // Variables
    bool statesInitialised;         // boolean true when filter states have been initialised
    bool magHealth;                 // boolean true if magnetometer has passed innovation consistency check
    bool velTimeout;                // boolean true if velocity measurements have failed innovation consistency check and timed out
    bool posTimeout;                // boolean true if position measurements have failed innovation consistency check and timed out
    bool hgtTimeout;                // boolean true if height measurements have failed innovation consistency check and timed out
    bool magTimeout;                // boolean true if magnetometer measurements have failed for too long and have timed out
    bool tasTimeout;                // boolean true if true airspeed measurements have failed for too long and have timed out
    bool dragTimeout;               // boolean true if drag measurements have failed for too long and have timed out
    bool badIMUdata;                // boolean true if the bad IMU data is detected
    bool velAiding;                 // boolean true if the velocity drift is constrained by observations
    bool waitingForGpsChecks;       // boolean true if the EKF should write GPS data to the buffer until quality checks have passed
    uint32_t badIMUdata_ms;         // time stamp bad IMU data was last detected
    uint32_t goodIMUdata_ms;        // time stamp good IMU data was last detected
    uint32_t vertVelVarClipCounter; // counter used to control reset of vertical velocity variance following collapse against the lower limit

    ftype gpsNoiseScaler;           // Used to scale the  GPS measurement noise and consistency gates to compensate for operation with small satellite counts
    Matrix24 P;                     // covariance matrix
    EKF_IMU_buffer_t<imu_elements> storedIMU;      // IMU data buffer
    EKF_obs_buffer_t<gps_elements> storedGPS;      // GPS data buffer
    EKF_obs_buffer_t<mag_elements> storedMag;      // Magnetometer data buffer
    EKF_obs_buffer_t<baro_elements> storedBaro;    // Baro data buffer
    EKF_obs_buffer_t<tas_elements> storedTAS;      // TAS data buffer
#if EK3_FEATURE_RANGEFINDER_MEASUREMENTS
    EKF_obs_buffer_t<range_elements> storedRange;  // Range finder data buffer
#endif
    EKF_IMU_buffer_t<output_elements> storedOutput;// output state buffer
    Matrix3F prevTnb;               // previous nav to body transformation used for INS earth rotation compensation
    ftype accNavMag;                // magnitude of navigation accel - used to adjust GPS obs variance (m/s^2)
    ftype accNavMagHoriz;           // magnitude of navigation accel in horizontal plane (m/s^2)
    Vector3F earthRateNED;          // earths angular rate vector in NED (rad/s)
    ftype dtIMUavg;                 // expected time between IMU measurements (sec)
    ftype dtEkfAvg;                 // expected time between EKF updates (sec)
    ftype dt;                       // time lapsed since the last covariance prediction (sec)
    ftype hgtRate;                  // state for rate of change of height filter
    bool onGround;                  // true when the flight vehicle is definitely on the ground
    bool prevOnGround;              // value of onGround from previous frame - used to detect transition
    bool inFlight;                  // true when the vehicle is definitely flying
    bool prevInFlight;              // value inFlight from previous frame - used to detect transition
    bool manoeuvring;               // boolean true when the flight vehicle is performing horizontal changes in velocity
    Vector6 innovVelPos;            // innovation output for a group of measurements
    Vector6 varInnovVelPos;         // innovation variance output for a group of measurements
    Vector6 velPosObs;              // observations for combined velocity and positon group of measurements (3x1 m , 3x1 m/s)
    bool fuseVelData;               // this boolean causes the velNE measurements to be fused
    bool fuseVelVertData;           // this boolean causes the velD measurement to be fused
    bool fusePosData;               // this boolean causes the posNE measurements to be fused
    bool fuseHgtData;               // this boolean causes the hgtMea measurements to be fused
    Vector3F innovMag;              // innovation output from fusion of X,Y,Z compass measurements
    Vector3F varInnovMag;           // innovation variance output from fusion of X,Y,Z compass measurements
    ftype innovVtas;                // innovation output from fusion of airspeed measurements
    ftype varInnovVtas;             // innovation variance output from fusion of airspeed measurements
    ftype defaultAirSpeed;          // default equivalent airspeed in m/s to be used if the measurement is unavailable. Do not use if not positive.
    ftype defaultAirSpeedVariance;  // default equivalent airspeed variance in (m/s)**2 to be used when defaultAirSpeed is specified. 
    bool magFusePerformed;          // boolean set to true when magnetometer fusion has been perfomred in that time step
    MagCal effectiveMagCal;         // the actual mag calibration being used as the default
    uint32_t prevTasStep_ms;        // time stamp of last TAS fusion step
    uint32_t prevBetaDragStep_ms;   // time stamp of last synthetic sideslip fusion step
    ftype innovBeta;                // synthetic sideslip innovation (rad)
    uint32_t lastMagUpdate_us;      // last time compass was updated in usec
    uint32_t lastMagRead_ms;        // last time compass data was successfully read
    Vector3F velDotNED;             // rate of change of velocity in NED frame
    Vector3F velDotNEDfilt;         // low pass filtered velDotNED
    uint32_t imuSampleTime_ms;      // time that the last IMU value was taken
    bool tasDataToFuse;             // true when new airspeed data is waiting to be fused
    uint32_t lastBaroReceived_ms;   // time last time we received baro height data
    uint16_t hgtRetryTime_ms;       // time allowed without use of height measurements before a height timeout is declared
    uint32_t lastVelPassTime_ms;    // time stamp when GPS velocity measurement last passed innovation consistency check (msec)
    uint32_t lastGpsPosPassTime_ms;    // time stamp when GPS position measurement last passed innovation consistency check (msec)
    uint32_t lastHgtPassTime_ms;    // time stamp when height measurement last passed innovation consistency check (msec)
    uint32_t lastTasPassTime_ms;    // time stamp when airspeed measurement last passed innovation consistency check (msec)
    uint32_t lastTasFailTime_ms;    // time stamp when airspeed measurement last failed innovation consistency check (msec)
    uint32_t lastTimeGpsReceived_ms;// last time we received GPS data
    uint32_t timeAtLastAuxEKF_ms;   // last time the auxiliary filter was run to fuse range or optical flow measurements
    uint32_t lastHealthyMagTime_ms; // time the magnetometer was last declared healthy
    bool allMagSensorsFailed;       // true if all magnetometer sensors have timed out on this flight and we are no longer using magnetometer data
    uint32_t lastSynthYawTime_ms;   // time stamp when yaw observation was last fused (msec)
    uint32_t ekfStartTime_ms;       // time the EKF was started (msec)
    Vector2F lastKnownPositionNE;   // last known position
    float lastKnownPositionD;       // last known height
    uint32_t lastLaunchAccelTime_ms;
    ftype velTestRatio;             // sum of squares of GPS velocity innovation divided by fail threshold
    ftype posTestRatio;             // sum of squares of GPS position innovation divided by fail threshold
    ftype hgtTestRatio;             // sum of squares of baro height innovation divided by fail threshold
    Vector3F magTestRatio;          // sum of squares of magnetometer innovations divided by fail threshold
    ftype tasTestRatio;             // sum of squares of true airspeed innovation divided by fail threshold
    bool inhibitWindStates;         // true when wind states and covariances should not be used
    ftype lastAirspeedEstimate;     // last true airspeed estimate (m/s)
    bool lastAspdEstIsValid;        // true when the last true airspeed estimate is valid (m/s)
    bool windStateIsObservable;     // true when wind states are observable from measurements.
    bool treatWindStatesAsTruth;    // true when wind states should be used as a truth reference
    bool windStatesAligned;         // true when wind states have been aligned
    bool inhibitMagStates;          // true when magnetic field states are inactive
    bool lastInhibitMagStates;      // previous inhibitMagStates
    bool needMagBodyVarReset;       // we need to reset mag body variances at next CovariancePrediction
    bool needEarthBodyVarReset;     // we need to reset mag earth variances at next CovariancePrediction
    bool inhibitDelAngBiasStates;   // true when IMU delta angle bias states are inactive
    bool gpsIsInUse;                // bool true when GPS data is being used to correct states estimates
    Location EKF_origin;     // LLH origin of the NED axis system, internal only
    Location &public_origin; // LLH origin of the NED axis system, public functions
    bool validOrigin;               // true when the EKF origin is valid
    ftype gpsSpdAccuracy;           // estimated speed accuracy in m/s returned by the GPS receiver
    ftype gpsPosAccuracy;           // estimated position accuracy in m returned by the GPS receiver
    ftype gpsHgtAccuracy;           // estimated height accuracy in m returned by the GPS receiver
    uint32_t lastGpsVelFail_ms;     // time of last GPS vertical velocity consistency check fail
    uint32_t lastGpsVelPass_ms;     // time of last GPS vertical velocity consistency check pass
    uint32_t lastGpsAidBadTime_ms;  // time in msec gps aiding was last detected to be bad
    ftype posDownAtTakeoff;         // flight vehicle vertical position sampled at transition from on-ground to in-air and used as a reference (m)
    bool useGpsVertVel;             // true if GPS vertical velocity should be used
    ftype yawResetAngle;            // Change in yaw angle due to last in-flight yaw reset in radians. A positive value means the yaw angle has increased.
    uint32_t lastYawReset_ms;       // System time at which the last yaw reset occurred. Returned by getLastYawResetAngle
    bool tiltAlignComplete;         // true when tilt alignment is complete
    bool yawAlignComplete;          // true when yaw alignment is complete
    uint8_t yawAlignGpsValidCount;  // number of continuous good GPS velocity samples used for in flight yaw alignment
    bool magStateInitComplete;      // true when the magnetic field states have been initialised
    uint8_t stateIndexLim;          // Max state index used during matrix and array operations
    imu_elements imuDataDelayed;    // IMU data at the fusion time horizon
    imu_elements imuDataNew;        // IMU data at the current time horizon
    imu_elements imuDataDownSampledNew; // IMU data at the current time horizon that has been downsampled to a 100Hz rate
    QuaternionF imuQuatDownSampleNew; // Quaternion obtained by rotating through the IMU delta angles since the start of the current down sampled frame
    baro_elements baroDataNew;      // Baro data at the current time horizon
    baro_elements baroDataDelayed;  // Baro data at the fusion time horizon
    range_elements rangeDataNew;    // Range finder data at the current time horizon
    range_elements rangeDataDelayed;// Range finder data at the fusion time horizon
    tas_elements tasDataNew;        // TAS data at the current time horizon
    tas_elements tasDataDelayed;    // TAS data at the fusion time horizon
    mag_elements magDataDelayed;    // Magnetometer data at the fusion time horizon
    gps_elements gpsDataNew;        // GPS data at the current time horizon
    gps_elements gpsDataDelayed;    // GPS data at the fusion time horizon
    uint8_t last_gps_idx;           // sensor ID of the GPS receiver used for the last fusion or reset
    output_elements outputDataNew;  // output state data at the current time step
    output_elements outputDataDelayed; // output state data at the current time step
    Vector3F delAngCorrection;      // correction applied to delta angles used by output observer to track the EKF
    Vector3F velErrintegral;        // integral of output predictor NED velocity tracking error (m)
    Vector3F posErrintegral;        // integral of output predictor NED position tracking error (m.sec)
    ftype badImuVelErrIntegral;     // integral of output predictor D velocity tracking error when bad IMU data is detected (m)
    ftype innovYaw;                 // compass yaw angle innovation (rad)
    uint32_t timeTasReceived_ms;    // time last TAS data was received (msec)
    bool gpsGoodToAlign;            // true when the GPS quality can be used to initialise the navigation system
    uint32_t magYawResetTimer_ms;   // timer in msec used to track how long good magnetometer data is failing innovation consistency checks
    bool consistentMagData;         // true when the magnetometers are passing consistency checks
    bool motorsArmed;               // true when the motors have been armed
    bool prevMotorsArmed;           // value of motorsArmed from previous frame
    bool posVelFusionDelayed;       // true when the position and velocity fusion has been delayed
#if EK3_FEATURE_OPTFLOW_FUSION
    bool optFlowFusionDelayed;      // true when the optical flow fusion has been delayed
#endif
    bool airSpdFusionDelayed;       // true when the air speed fusion has been delayed
    bool sideSlipFusionDelayed;     // true when the sideslip fusion has been delayed
    bool airDataFusionWindOnly;     // true when  sideslip and airspeed fusion is only allowed to modify the wind states
    Vector3F lastMagOffsets;        // Last magnetometer offsets from COMPASS_ parameters. Used to detect parameter changes.
    bool lastMagOffsetsValid;       // True when lastMagOffsets has been initialized
    Vector2F posResetNE;            // Change in North/East position due to last in-flight reset in metres. Returned by getLastPosNorthEastReset
    uint32_t lastPosReset_ms;       // System time at which the last position reset occurred. Returned by getLastPosNorthEastReset
    Vector2F velResetNE;            // Change in North/East velocity due to last in-flight reset in metres/sec. Returned by getLastVelNorthEastReset
    uint32_t lastVelReset_ms;       // System time at which the last velocity reset occurred. Returned by getLastVelNorthEastReset
    ftype posResetD;                // Change in Down position due to last in-flight reset in metres. Returned by getLastPosDowntReset
    uint32_t lastPosResetD_ms;      // System time at which the last position reset occurred. Returned by getLastPosDownReset
    ftype yawTestRatio;             // square of magnetometer yaw angle innovation divided by fail threshold
    QuaternionF prevQuatMagReset;    // Quaternion from the last time the magnetic field state reset condition test was performed
    ftype hgtInnovFiltState;        // state used for fitering of the height innovations used for pre-flight checks
    uint8_t magSelectIndex;         // Index of the magnetometer that is being used by the EKF
    bool runUpdates;                // boolean true when the EKF updates can be run
    uint32_t framesSincePredict;    // number of frames lapsed since EKF instance did a state prediction
    uint8_t localFilterTimeStep_ms; // average number of msec between filter updates
    ftype posDownObsNoise;          // observation noise variance on the vertical position used by the state and covariance update step (m^2)
    Vector3F delAngCorrected;       // corrected IMU delta angle vector at the EKF time horizon (rad)
    Vector3F delVelCorrected;       // corrected IMU delta velocity vector at the EKF time horizon (m/s)
    bool magFieldLearned;           // true when the magnetic field has been learned
    uint32_t wasLearningCompass_ms; // time when we were last waiting for compass learn to complete
    Vector3F earthMagFieldVar;      // NED earth mag field variances for last learned field (mGauss^2)
    Vector3F bodyMagFieldVar;       // XYZ body mag field variances for last learned field (mGauss^2)
    bool delAngBiasLearned;         // true when the gyro bias has been learned
    nav_filter_status filterStatus; // contains the status of various filter outputs
    ftype ekfOriginHgtVar;          // Variance of the EKF WGS-84 origin height estimate (m^2)
    double ekfGpsRefHgt;            // floating point representation of the WGS-84 reference height used to convert GPS height to local height (m)
    uint32_t lastOriginHgtTime_ms;  // last time the ekf's WGS-84 origin height was corrected
    Vector3F outputTrackError;      // attitude (rad), velocity (m/s) and position (m) tracking error magnitudes from the output observer
    Vector3F velOffsetNED;          // This adds to the earth frame velocity estimate at the IMU to give the velocity at the body origin (m/s)
    Vector3F posOffsetNED;          // This adds to the earth frame position estimate at the IMU to give the position at the body origin (m)
    uint32_t firstInitTime_ms;      // First time the initialise function was called (msec)
    uint32_t lastInitFailReport_ms; // Last time the buffer initialisation failure report was sent (msec)
    ftype tiltErrorVariance;        // variance of the angular uncertainty measured perpendicular to the vertical (rad^2)

    // variables used to calculate a vertical velocity that is kinematically consistent with the vertical position
    struct {
        ftype pos;
        ftype vel;
        ftype acc;
    } vertCompFiltState;

    // variables used by the pre-initialisation GPS checks
    Location gpsloc_prev;    // LLH location of previous GPS measurement
    uint32_t lastPreAlignGpsCheckTime_ms;   // last time in msec the GPS quality was checked during pre alignment checks
    ftype gpsDriftNE;               // amount of drift detected in the GPS position during pre-flight GPs checks
    ftype gpsVertVelFilt;           // amount of filtered vertical GPS velocity detected during pre-flight GPS checks
    ftype gpsHorizVelFilt;          // amount of filtered horizontal GPS velocity detected during pre-flight GPS checks

    // variable used by the in-flight GPS quality check
    bool gpsSpdAccPass;             // true when reported GPS speed accuracy passes in-flight checks
    bool gpsVertAccPass;            // true when reported GPS vertical accuracy passes in-flight checks
    bool ekfInnovationsPass;        // true when GPS innovations pass in-flight checks
    ftype sAccFilterState1;         // state variable for LPF applied to reported GPS speed accuracy
    ftype sAccFilterState2;         // state variable for peak hold filter applied to reported GPS speed
    uint32_t lastGpsCheckTime_ms;   // last time in msec the GPS quality was checked
    uint32_t lastGpsInnovPassTime_ms;  // last time in msec the GPS innovations passed
    uint32_t lastGpsInnovFailTime_ms;  // last time in msec the GPS innovations failed
    uint32_t lastGpsVertAccPassTime_ms;  // last time in msec the GPS vertical accuracy test passed
    uint32_t lastGpsVertAccFailTime_ms;  // last time in msec the GPS vertical accuracy test failed
    bool gpsAccuracyGood;           // true when the GPS accuracy is considered to be good enough for safe flight.
    bool gpsAccuracyGoodForAltitude; // true when the GPS accuracy is considered to be good enough to use it as an altitude source.
    Vector3F gpsVelInnov;           // gps velocity innovations
    Vector3F gpsVelVarInnov;        // gps velocity innovation variances
    uint32_t gpsRetrieveTime_ms;    // system time that GPS data was retrieved from the buffer (to detect timeouts)

    // variables added for optical flow fusion
    EKF_obs_buffer_t<of_elements> storedOF;    // OF data buffer
    bool flowDataValid;             // true while optical flow data is still fresh
    Vector2F auxFlowObsInnov;       // optical flow rate innovation from 1-state terrain offset estimator
    uint32_t flowValidMeaTime_ms;   // time stamp from latest valid flow measurement (msec)
    uint32_t rngValidMeaTime_ms;    // time stamp from latest valid range measurement (msec)
    uint32_t flowMeaTime_ms;        // time stamp from latest flow measurement (msec)
    uint32_t gndHgtValidTime_ms;    // time stamp from last terrain offset state update (msec)
    Vector2 flowVarInnov;           // optical flow innovations variances (rad/sec)^2
    Vector2 flowInnov;              // optical flow LOS innovations (rad/sec)
    uint32_t flowInnovTime_ms;      // system time that optical flow innovations and variances were recorded (to detect timeouts)
#if EK3_FEATURE_OPTFLOW_FUSION
    ftype Popt;                     // Optical flow terrain height state covariance (m^2)
#endif
    ftype terrainState;             // terrain position state (m)
    ftype prevPosN;                 // north position at last measurement
    ftype prevPosE;                 // east position at last measurement
    ftype varInnovRng;              // range finder observation innovation variance (m^2)
    ftype innovRng;                 // range finder observation innovation (m)
    struct {
        uint32_t timestamp_ms;      // system timestamp of last correct optical flow sample (used for calibration)
        Vector2f flowRate;          // latest corrected optical flow flow rate (used for calibration)
        Vector2f bodyRate;          // latest corrected optical flow body rate (used for calibration)
        Vector2f losPred;           // EKF estimated component of flowRate that comes from vehicle movement (not rotation)
    } flowCalSample;

    ftype hgtMea;                   // height measurement derived from either baro, gps or range finder data (m)
    bool inhibitGndState;           // true when the terrain position state is to remain constant
    uint32_t prevFlowFuseTime_ms;   // time both flow measurement components passed their innovation consistency checks
    Vector2 flowTestRatio;          // square of optical flow innovations divided by fail threshold used by main filter where >1.0 is a fail
    Vector2F auxFlowTestRatio;      // sum of squares of optical flow innovation divided by fail threshold used by 1-state terrain offset estimator
    ftype R_LOS;                    // variance of optical flow rate measurements (rad/sec)^2
    ftype auxRngTestRatio;          // square of range finder innovations divided by fail threshold used by main filter where >1.0 is a fail
    Vector2F flowGyroBias;          // bias error of optical flow sensor gyro output
    bool rangeDataToFuse;           // true when valid range finder height data has arrived at the fusion time horizon.
    bool baroDataToFuse;            // true when valid baro height finder data has arrived at the fusion time horizon.
    bool gpsDataToFuse;             // true when valid GPS data has arrived at the fusion time horizon.
    bool magDataToFuse;             // true when valid magnetometer data has arrived at the fusion time horizon
    enum AidingMode {
        AID_ABSOLUTE=0,    // GPS or some other form of absolute position reference aiding is being used (optical flow may also be used in parallel) so position estimates are absolute.
        AID_NONE=1,       // no aiding is being used so only attitude and height estimates are available. Either constVelMode or constPosMode must be used to constrain tilt drift.
        AID_RELATIVE=2,    // only optical flow aiding is being used so position estimates will be relative
    };
    AidingMode PV_AidingMode;       // Defines the preferred mode for aiding of velocity and position estimates from the INS
    AidingMode PV_AidingModePrev;   // Value of PV_AidingMode from the previous frame - used to detect transitions
    bool gndOffsetValid;            // true when the ground offset state can still be considered valid
    Vector3F delAngBodyOF;          // bias corrected delta angle of the vehicle IMU measured summed across the time since the last OF measurement
    ftype delTimeOF;                // time that delAngBodyOF is summed across
    bool flowFusionActive;          // true when optical flow fusion is active

    Vector3F accelPosOffset;        // position of IMU accelerometer unit in body frame (m)

    // Range finder
    ftype baroHgtOffset;                    // offset applied when when switching to use of Baro height
    ftype rngOnGnd;                         // Expected range finder reading in metres when vehicle is on ground
    uint32_t lastRngMeasTime_ms;            // Timestamp of last range measurement
    bool terrainHgtStable;                  // true when the terrain height is stable enough to be used as a height reference
#if EK3_FEATURE_RANGEFINDER_MEASUREMENTS
    ftype storedRngMeas[DOWNWARD_RANGEFINDER_MAX_INSTANCES][3];              // Ringbuffer of stored range measurements for dual range sensors
    uint32_t storedRngMeasTime_ms[DOWNWARD_RANGEFINDER_MAX_INSTANCES][3];    // Ringbuffers of stored range measurement times for dual range sensors
    uint8_t rngMeasIndex[DOWNWARD_RANGEFINDER_MAX_INSTANCES];                // Current range measurement ringbuffer index for dual range sensors
#endif

    // body frame odometry fusion
#if EK3_FEATURE_BODY_ODOM
    EKF_obs_buffer_t<vel_odm_elements> storedBodyOdm;    // body velocity data buffer
    vel_odm_elements bodyOdmDataNew;       // Body frame odometry data at the current time horizon
    vel_odm_elements bodyOdmDataDelayed;  // Body  frame odometry data at the fusion time horizon
#endif
    uint32_t lastbodyVelPassTime_ms;    // time stamp when the body velocity measurement last passed innovation consistency checks (msec)
    Vector3 bodyVelTestRatio;           // Innovation test ratios for body velocity XYZ measurements
    Vector3 varInnovBodyVel;            // Body velocity XYZ innovation variances (m/sec)^2
    Vector3 innovBodyVel;               // Body velocity XYZ innovations (m/sec)
    uint32_t prevBodyVelFuseTime_ms;    // previous time all body velocity measurement components passed their innovation consistency checks (msec)
    uint32_t bodyOdmMeasTime_ms;        // time body velocity measurements were accepted for input to the data buffer (msec)
    bool bodyVelFusionDelayed;          // true when body frame velocity fusion has been delayed
    bool bodyVelFusionActive;           // true when body frame velocity fusion is active

#if EK3_FEATURE_BODY_ODOM
    // wheel sensor fusion
    EKF_obs_buffer_t<wheel_odm_elements> storedWheelOdm;    // body velocity data buffer
    wheel_odm_elements wheelOdmDataDelayed;   // Body  frame odometry data at the fusion time horizon
#endif

    // GPS yaw sensor fusion
    uint32_t yawMeasTime_ms;            // system time GPS yaw angle was last input to the data buffer
    EKF_obs_buffer_t<yaw_elements> storedYawAng;    // GPS yaw angle buffer
    yaw_elements yawAngDataNew;         // GPS yaw angle at the current time horizon
    yaw_elements yawAngDataDelayed;     // GPS yaw angle at the fusion time horizon
    yaw_elements yawAngDataStatic;      // yaw angle (regardless of yaw source) when the vehicle was last on ground and not moving

    // Range Beacon Sensor Fusion
#if EK3_FEATURE_BEACON_FUSION
    class BeaconFusion {
    public:
        BeaconFusion(AP_DAL &_dal) :
            dal{_dal}
            {}

        void InitialiseVariables();

        EKF_obs_buffer_t<rng_bcn_elements> storedRange; // Beacon range buffer
        rng_bcn_elements dataDelayed; // Range beacon data at the fusion time horizon
        uint32_t lastPassTime_ms;     // time stamp when the range beacon measurement last passed innovation consistency checks (msec)
        ftype testRatio;              // Innovation test ratio for range beacon measurements
        bool health;                  // boolean true if range beacon measurements have passed innovation consistency check
        ftype varInnov;               // range beacon observation innovation variance (m^2)
        ftype innov;                  // range beacon observation innovation (m)
        uint32_t lastTime_ms[4];      // last time we received a range beacon measurement (msec)
        bool dataToFuse;              // true when there is new range beacon data to fuse
        Vector3F vehiclePosNED;       // NED position estimate from the beacon system (NED)
        ftype vehiclePosErr;          // estimated position error from the beacon system (m)
        uint32_t last3DmeasTime_ms;   // last time the beacon system returned a 3D fix (msec)
        bool goodToAlign;             // true when the range beacon systems 3D fix can be used to align the filter
        uint8_t lastChecked;          // index of the last range beacon checked for data
        Vector3F receiverPos;               // receiver NED position (m) - alignment 3 state filter
        ftype receiverPosCov[3][3];         // Receiver position covariance (m^2) - alignment 3 state filter (
        bool alignmentStarted;        // True when the initial position alignment using range measurements has started
        bool alignmentCompleted;      // True when the initial position alignment using range measurements has finished
        uint8_t lastIndex;            // Range beacon index last read -  used during initialisation of the 3-state filter
        Vector3F posSum;              // Sum of range beacon NED position (m) - used during initialisation of the 3-state filter
        uint8_t numMeas;                 // Number of beacon measurements - used during initialisation of the 3-state filter
        ftype sum;                       // Sum of range measurements (m) - used during initialisation of the 3-state filter
        uint8_t N;                  // Number of range beacons in use
        ftype maxPosD;                   // maximum position of all beacons in the down direction (m)
        ftype minPosD;                   // minimum position of all beacons in the down direction (m)
        bool usingMinHypothesis;            // true when the min beacon constellation offset hypothesis is being used

        ftype posDownOffsetMax;          // Vertical position offset of the beacon constellation origin relative to the EKF origin (m)
        ftype posOffsetMaxVar;           // Variance of the PosDownOffsetMax state (m)
        ftype maxOffsetStateChangeFilt;     // Filtered magnitude of the change in PosOffsetHigh

        ftype posDownOffsetMin;          // Vertical position offset of the beacon constellation origin relative to the EKF origin (m)
        ftype posOffsetMinVar;           // Variance of the PosDownOffsetMin state (m)
        ftype minOffsetStateChangeFilt;     // Filtered magnitude of the change in PosOffsetLow

        Vector3F posOffsetNED;           // NED position of the beacon origin in earth frame (m)
        bool originEstInit;              // True when the beacon origin has been initialised

        // Range Beacon Fusion Debug Reporting
        uint8_t fuseDataReportIndex;// index of range beacon fusion data last reported
        struct FusionReport {
            ftype rng;          // measured range to beacon (m)
            ftype innov;        // range innovation (m)
            ftype innovVar;     // innovation variance (m^2)
            ftype testRatio;    // innovation consistency test ratio
            Vector3F beaconPosNED; // beacon NED position
        } *fusionReport;
        uint8_t numFusionReports;

        AP_DAL &dal;
    } rngBcn{dal};
#endif  // if EK3_FEATURE_BEACON_FUSION

#if EK3_FEATURE_DRAG_FUSION
    // drag fusion for multicopter wind estimation
    EKF_obs_buffer_t<drag_elements> storedDrag;
    drag_elements dragSampleDelayed;
    drag_elements dragDownSampled;	    // down sampled from filter prediction rate to observation rate
    uint8_t dragSampleCount;	        // number of drag specific force samples accumulated at the filter prediction rate
    ftype dragSampleTimeDelta;	        // time integral across all samples used to form _drag_down_sampled (sec)
    Vector2F innovDrag;		            // multirotor drag measurement innovation (m/sec**2)
	Vector2F innovDragVar;	            // multirotor drag measurement innovation variance ((m/sec**2)**2)
	Vector2F dragTestRatio;		        // drag innovation consistency check ratio
#endif
	uint32_t lastDragPassTime_ms;       // system time that drag samples were last successfully fused
    bool dragFusionEnabled;

    // height source selection logic
    AP_NavEKF_Source::SourceZ activeHgtSource;  // active height source
    AP_NavEKF_Source::SourceZ prevHgtSource;    // previous height source used to detect changes in source

    // Movement detector
    bool takeOffDetected;           // true when takeoff for optical flow navigation has been detected
    ftype rngAtStartOfFlight;       // range finder measurement at start of flight
    uint32_t timeAtArming_ms;       // time in msec that the vehicle armed

    // baro ground effect
    ftype meaHgtAtTakeOff;            // height measured at commencement of takeoff

    // control of post takeoff magnetic field and heading resets
    bool finalInflightYawInit;      // true when the final post takeoff initialisation of yaw angle has been performed
    uint8_t magYawAnomallyCount;    // Number of times the yaw has been reset due to a magnetic anomaly during initial ascent
    bool finalInflightMagInit;      // true when the final post takeoff initialisation of magnetic field states been performed
    bool magStateResetRequest;      // true if magnetic field states need to be reset using the magnetomter measurements
    bool magYawResetRequest;        // true if the vehicle yaw and magnetic field states need to be reset using the magnetometer measurements
    bool gpsYawResetRequest;        // true if the vehicle yaw needs to be reset to the GPS course
    ftype posDownAtLastMagReset;    // vertical position last time the mag states were reset (m)
    ftype yawInnovAtLastMagReset;   // magnetic yaw innovation last time the yaw and mag field states were reset (rad)
    QuaternionF quatAtLastMagReset;  // quaternion states last time the mag states were reset
    MagFuseSel magFusionSel;        // magnetometer fusion selection

    // Used by on ground movement check required when operating on ground without a yaw reference
    ftype gyro_diff;                    // filtered gyro difference (rad/s)
    ftype accel_diff;                   // filtered acceerometer difference (m/s/s)
    Vector3F gyro_prev;                 // gyro vector from previous time step (rad/s)
    Vector3F accel_prev;                // accelerometer vector from previous time step (m/s/s)
    bool onGroundNotMoving;             // true when on the ground and not moving
    uint32_t lastMoveCheckLogTime_ms;   // last time the movement check data was logged (msec)

	// variables used to inhibit accel bias learning
    bool inhibitDelVelBiasStates;       // true when all IMU delta velocity bias states are de-activated
    bool dvelBiasAxisInhibit[3] {};		// true when IMU delta velocity bias states for a specific axis is de-activated
	Vector3F dvelBiasAxisVarPrev;		// saved delta velocity XYZ bias variances (m/sec)**2

#if EK3_FEATURE_EXTERNAL_NAV
    // external navigation fusion
    EKF_obs_buffer_t<ext_nav_elements> storedExtNav; // external navigation data buffer
    ext_nav_elements extNavDataDelayed; // External nav at the fusion time horizon
    uint32_t extNavMeasTime_ms;         // time external measurements were accepted for input to the data buffer (msec)
    uint32_t extNavLastPosResetTime_ms; // last time the external nav systen performed a position reset (msec)
    bool extNavDataToFuse;              // true when there is new external nav data to fuse
    bool extNavUsedForPos;              // true when the external nav data is being used as a position reference.
    EKF_obs_buffer_t<ext_nav_vel_elements> storedExtNavVel;    // external navigation velocity data buffer
    ext_nav_vel_elements extNavVelDelayed;  // external navigation velocity data at the fusion time horizon.  Already corrected for sensor position
    uint32_t extNavVelMeasTime_ms;      // time external navigation velocity measurements were accepted for input to the data buffer (msec)
    bool extNavVelToFuse;               // true when there is new external navigation velocity to fuse
    Vector3F extNavVelInnov;            // external nav velocity innovations
    Vector3F extNavVelVarInnov;         // external nav velocity innovation variances
    uint32_t extNavVelInnovTime_ms;     // system time that external nav velocity innovations were recorded (to detect timeouts)
    EKF_obs_buffer_t<yaw_elements> storedExtNavYawAng;  // external navigation yaw angle buffer
    yaw_elements extNavYawAngDataDelayed;   // external navigation yaw angle at the fusion time horizon
    uint32_t last_extnav_yaw_fusion_ms; // system time that external nav yaw was last fused
#endif // EK3_FEATURE_EXTERNAL_NAV
    bool useExtNavVel;                  // true if external nav velocity should be used

    // flags indicating severe numerical errors in innovation variance calculation for different fusion operations
    struct {
        bool bad_xmag:1;
        bool bad_ymag:1;
        bool bad_zmag:1;
        bool bad_airspeed:1;
        bool bad_sideslip:1;
        bool bad_nvel:1;
        bool bad_evel:1;
        bool bad_dvel:1;
        bool bad_npos:1;
        bool bad_epos:1;
        bool bad_dpos:1;
        bool bad_yaw:1;
        bool bad_decl:1;
        bool bad_xflow:1;
        bool bad_yflow:1;
        bool bad_rngbcn:1;
        bool bad_xvel:1;
        bool bad_yvel:1;
        bool bad_zvel:1;
    } faultStatus;

    // flags indicating which GPS quality checks are failing
    union {
        struct {
            bool bad_sAcc:1;
            bool bad_hAcc:1;
            bool bad_vAcc:1;
            bool bad_yaw:1;
            bool bad_sats:1;
            bool bad_VZ:1;
            bool bad_horiz_drift:1;
            bool bad_hdop:1;
            bool bad_vert_vel:1;
            bool bad_fix:1;
            bool bad_horiz_vel:1;
        };
        uint16_t value;
    } gpsCheckStatus;

    // string representing last reason for prearm failure
    char prearm_fail_string[40];

    // earth field from WMM tables
    bool have_table_earth_field;   // true when we have initialised table_earth_field_ga
    Vector3F table_earth_field_ga; // earth field from WMM tables
    ftype table_declination;       // declination in radians from the tables

    // 1Hz update
    uint32_t last_oneHz_ms;
    void oneHzUpdate(void);

    // move EKF origin at 1Hz
    void moveEKFOrigin(void);

    // handle earth field updates
    void getEarthFieldTable(const Location &loc);
    void checkUpdateEarthField(void);
    
    // timing statistics
    struct ekf_timing timing;

    // when was attitude filter status last non-zero?
    uint32_t last_filter_ok_ms;
    
    // should we assume zero sideslip?
    bool assume_zero_sideslip(void) const;

    // vehicle specific initial gyro bias uncertainty
    ftype InitialGyroBiasUncertainty(void) const;

    /*
      learn magnetometer biases from GPS yaw. Return true if the
      resulting mag vector is close enough to the one predicted by GPS
      yaw to use it for fallback
    */
    bool learnMagBiasFromGPS(void);

    uint32_t last_gps_yaw_ms; // last time the EKF attempted to use the GPS yaw
    uint32_t last_gps_yaw_fuse_ms; // last time the EKF successfully fused the GPS yaw
    bool gps_yaw_mag_fallback_ok;
    bool gps_yaw_mag_fallback_active;
    uint8_t gps_yaw_fallback_good_counter;

    /*
    Update the on ground and not moving check.
    Should be called once per IMU update.
    Only updates when on ground and when operating with an external yaw sensor
    */
    void updateMovementCheck(void);

    // The following declarations are used to control when the main navigation filter resets it's yaw to the estimate provided by the GSF
    uint32_t EKFGSF_yaw_reset_ms;           // timestamp of last emergency yaw reset (uSec)
    uint32_t EKFGSF_yaw_reset_request_ms;   // timestamp of last emergency yaw reset request (uSec)
    uint8_t EKFGSF_yaw_reset_count;         // number of emergency yaw resets performed
    bool EKFGSF_run_filterbank;             // true when the filter bank is active
    uint8_t EKFGSF_yaw_valid_count;         // number of updates since the last invalid yaw estimate

    // logging timestamps
    uint32_t lastLogTime_ms;
    uint32_t lastUpdateTime_ms;
    uint32_t lastEkfStateVarLogTime_ms;
    uint32_t lastTimingLogTime_ms;

    // bits in EK3_AFFINITY
    enum ekf_affinity {
        EKF_AFFINITY_GPS  = (1U<<0),
        EKF_AFFINITY_BARO = (1U<<1),
        EKF_AFFINITY_MAG  = (1U<<2),
        EKF_AFFINITY_ARSP = (1U<<3),
    };

    // update selected_sensors for this core
    void update_sensor_selection(void);
    void update_gps_selection(void);
    void update_mag_selection(void);
    void update_baro_selection(void);
    void update_airspeed_selection(void);

    // selected and preferred sensor instances. We separate selected
    // from preferred so that calcGpsGoodToAlign() can ensure the
    // preferred sensor is ready. Note that magSelectIndex is used for
    // compass selection
    uint8_t selected_gps;
    uint8_t preferred_gps;
    uint8_t selected_baro;
    uint8_t selected_airspeed;

    // source reset handling
    AP_NavEKF_Source::SourceXY posxy_source_last;   // horizontal position source on previous iteration (used to detect a changes)
    bool posxy_source_reset;                        // true when the horizontal position source has changed but the position has not yet been reset
    AP_NavEKF_Source::SourceYaw yaw_source_last;    // yaw source on previous iteration (used to detect a change)
    bool yaw_source_reset;                          // true when the yaw source has changed but the yaw has not yet been reset

    // logging functions shared by cores:
    void Log_Write_XKF1(uint64_t time_us) const;
    void Log_Write_XKF2(uint64_t time_us) const;
    void Log_Write_XKF3(uint64_t time_us) const;
    void Log_Write_XKF4(uint64_t time_us) const;
    void Log_Write_XKF5(uint64_t time_us) const;
    void Log_Write_XKFS(uint64_t time_us) const;
    void Log_Write_Quaternion(uint64_t time_us) const;
    void Log_Write_Beacon(uint64_t time_us);
    void Log_Write_BodyOdom(uint64_t time_us);
    void Log_Write_State_Variances(uint64_t time_us);
    void Log_Write_Timing(uint64_t time_us);
    void Log_Write_GSF(uint64_t time_us);
};
