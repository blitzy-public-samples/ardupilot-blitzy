/**
 * @file AP_NavEKF2_core.h
 * @brief NavEKF2 Core Backend - 24-State Extended Kalman Filter for Inertial Navigation
 * 
 * @details This file implements the computational backend for the second-generation
 *          ArduPilot navigation Extended Kalman Filter (EKF2). Each instance of this
 *          class represents one "core" or "lane" in a multi-core EKF architecture
 *          that provides redundancy and robustness through parallel filtering with
 *          independent IMU sources.
 * 
 * Mathematical Derivation:
 * Based on the derivation documented at:
 * https://github.com/priseborough/InertialNav/blob/master/derivations/
 * RotationVectorAttitudeParameterisation/GenerateNavFilterEquations.m
 * Converted from Matlab to C++ by Paul Riseborough
 * 
 * State Vector Composition (24 states + 4 quaternion = 28 total):
 * - States 0-2:   Rotation error vector (rad) - rotation vector parameterization
 * - States 3-5:   Velocity NED (m/s) - navigation frame
 * - States 6-8:   Position NED (m) - relative to EKF origin
 * - States 9-11:  Gyro bias (rad/s) - body frame
 * - States 12-14: Gyro scale factor error (dimensionless) - body frame  
 * - State 15:     Z-axis accelerometer bias (m/s²) - body frame
 * - States 16-18: Earth magnetic field NED (gauss/1000) - navigation frame
 * - States 19-21: Body magnetic field XYZ (gauss/1000) - body frame offsets
 * - States 22-23: Wind velocity NE (m/s) - horizontal navigation frame
 * - States 24-27: Attitude quaternion (NED to body frame rotation)
 * 
 * Coordinate Frames:
 * - NED (North-East-Down): Right-handed earth-fixed navigation frame
 * - Body Frame: Right-handed frame fixed to vehicle, X forward, Y right, Z down
 * - All position and velocity states are in NED frame
 * - All IMU measurements and biases are in body frame
 * 
 * Units:
 * - Position: meters (m)
 * - Velocity: meters per second (m/s)
 * - Angles: radians (rad)
 * - Angular rates: radians per second (rad/s)
 * - Acceleration: meters per second squared (m/s²)
 * - Magnetic field: gauss/1000 (milligauss)
 * 
 * Filter Operation:
 * - Target update rate: 100 Hz (EKF_TARGET_DT = 0.01s)
 * - Prediction step: IMU-driven state propagation and covariance prediction
 * - Update step: Sequential fusion of position, velocity, magnetometer, airspeed measurements
 * - Buffer-based: Measurements buffered and fused at appropriate time horizon
 * 
 * @warning This is computationally intensive code optimized with GCC O2 pragmas.
 *          Modifications to filter equations or covariance updates must be
 *          carefully validated to maintain numerical stability and real-time performance.
 * 
 * @note Filter stability and accuracy depend on:
 *       - Proper sensor calibration (IMU, magnetometer, barometer)
 *       - Sufficient GPS quality and availability during alignment
 *       - Appropriate tuning of process noise and measurement noise parameters
 *       - Numerical conditioning of covariance matrix (maintained via ForceSymmetry)
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h
 */
#pragma once

#if !defined(HAL_DEBUG_BUILD) || !HAL_DEBUG_BUILD
    #pragma GCC optimize("O2")
#endif

#include <AP_Common/Location.h>
#include <AP_Math/AP_Math.h>
#include <AP_Math/vectorN.h>
#include <AP_NavEKF/AP_NavEKF_core_common.h>
#include <AP_NavEKF/EKF_Buffer.h>
#include <AP_RangeFinder/AP_RangeFinder.h>
#include <AP_Beacon/AP_Beacon_config.h>

#include "AP_NavEKF/EKFGSF_yaw.h"

/**
 * @defgroup GPS_Preflight_Masks GPS Pre-flight Check Bitmask Flags
 * @brief Bitmask flags used in GPS quality pre-flight checks
 * @details These flags identify which specific GPS quality metrics have failed
 *          during pre-flight validation. Multiple flags can be set simultaneously.
 * @{
 */

/** @def MASK_GPS_NSATS
 * @brief GPS satellite count check failed
 * @details Set when number of satellites is below minimum threshold for reliable navigation
 */
#define MASK_GPS_NSATS      (1<<0)

/** @def MASK_GPS_HDOP
 * @brief GPS horizontal dilution of precision (HDOP) check failed
 * @details Set when HDOP exceeds maximum threshold indicating poor satellite geometry
 */
#define MASK_GPS_HDOP       (1<<1)

/** @def MASK_GPS_SPD_ERR
 * @brief GPS speed accuracy check failed
 * @details Set when reported speed accuracy exceeds acceptable threshold
 */
#define MASK_GPS_SPD_ERR    (1<<2)

/** @def MASK_GPS_POS_ERR
 * @brief GPS position accuracy check failed
 * @details Set when reported horizontal position accuracy exceeds acceptable threshold
 */
#define MASK_GPS_POS_ERR    (1<<3)

/** @def MASK_GPS_YAW_ERR
 * @brief GPS yaw/heading accuracy check failed
 * @details Set when GPS-derived heading accuracy is insufficient (dual GPS systems)
 */
#define MASK_GPS_YAW_ERR    (1<<4)

/** @def MASK_GPS_POS_DRIFT
 * @brief GPS position drift check failed
 * @details Set when stationary GPS position measurements show excessive drift
 */
#define MASK_GPS_POS_DRIFT  (1<<5)

/** @def MASK_GPS_VERT_SPD
 * @brief GPS vertical speed check failed
 * @details Set when GPS vertical velocity is inconsistent with stationary state
 */
#define MASK_GPS_VERT_SPD   (1<<6)

/** @def MASK_GPS_HORIZ_SPD
 * @brief GPS horizontal speed check failed
 * @details Set when GPS horizontal velocity exceeds threshold for stationary vehicle
 */
#define MASK_GPS_HORIZ_SPD  (1<<7)

/** @} */ // end of GPS_Preflight_Masks group

/**
 * @defgroup Height_Sources Height Data Source Selection
 * @brief Constants defining active height measurement source for EKF
 * @details The EKF can fuse height measurements from multiple sources.
 *          These constants identify which sensor is currently being used
 *          as the primary height reference.
 * @{
 */

/** @def HGT_SOURCE_BARO
 * @brief Barometric altimeter as height source
 * @details Most common height source, provides smooth altitude relative to QNH/QFE
 */
#define HGT_SOURCE_BARO     0

/** @def HGT_SOURCE_RNG
 * @brief Range finder (lidar/sonar) as height source
 * @details Used for terrain following and precision landing, height above ground level
 */
#define HGT_SOURCE_RNG      1

/** @def HGT_SOURCE_GPS
 * @brief GPS altitude as height source
 * @details WGS-84 ellipsoid height, used when baro unavailable or for absolute altitude
 */
#define HGT_SOURCE_GPS      2

/** @def HGT_SOURCE_BCN
 * @brief Range beacon system as height source
 * @details Height derived from range beacon triangulation
 */
#define HGT_SOURCE_BCN      3

/** @def HGT_SOURCE_EXTNAV
 * @brief External navigation system as height source
 * @details Height from external nav system (e.g., motion capture, visual odometry)
 */
#define HGT_SOURCE_EXTNAV   4

/** @} */ // end of Height_Sources group

/** @def EKF_TARGET_DT
 * @brief Target EKF update time step in seconds
 * @details Nominal time between EKF prediction updates, corresponding to 100 Hz update rate.
 *          Actual update rate may vary slightly based on IMU timing.
 * @note Value: 0.01s (100 Hz)
 */
#define EKF_TARGET_DT 0.01f

/** @def EKF2_MAG_FINAL_RESET_ALT
 * @brief Altitude threshold for final magnetometer reset during ascent
 * @details After takeoff, the EKF performs a final magnetometer field reset at this
 *          altitude (meters AGL) to eliminate ground magnetic interference effects.
 * @note Value: 2.5 meters above ground level
 * @warning Critical for yaw accuracy - vehicle should clear ground magnetic distortions
 */
#define EKF2_MAG_FINAL_RESET_ALT 2.5f

/** @def MAG_ANOMALY_RESET_MAX
 * @brief Maximum yaw resets allowed per flight due to magnetic anomalies
 * @details Limits the number of emergency yaw resets triggered by detected magnetic
 *          field anomalies during a single flight. Prevents repeated resets in
 *          magnetically disturbed environments.
 * @note Value: 2 resets per flight
 * @warning Exceeding this limit indicates persistent magnetic interference requiring investigation
 */
#define MAG_ANOMALY_RESET_MAX 2

/** @def YAW_RESET_TO_GSF_TIMEOUT_MS
 * @brief Timeout for GSF yaw reset request in milliseconds
 * @details Duration that a request to reset EKF yaw to the GSF (Gaussian Sum Filter)
 *          yaw estimate remains active before timing out.
 * @note Value: 5000 ms (5 seconds)
 * @see EKFGSF_requestYawReset()
 */
#define YAW_RESET_TO_GSF_TIMEOUT_MS 5000

/** @def EK2_POSXY_STATE_LIMIT
 * @brief Maximum allowed horizontal position state magnitude in meters
 * @details Limits horizontal position states to prevent filter divergence and
 *          numerical overflow. Different limits for single vs double precision builds.
 * @note Single precision: 1.0e6 m (1000 km)
 * @note Double precision: 50.0e6 m (50,000 km)
 * @warning Exceeding this limit triggers position reset to prevent numerical issues
 */
#if HAL_WITH_EKF_DOUBLE
#define EK2_POSXY_STATE_LIMIT 50.0e6
#else
#define EK2_POSXY_STATE_LIMIT 1.0e6
#endif

// maximum number of downward facing rangefinder instances available
#if AP_RANGEFINDER_ENABLED
#if RANGEFINDER_MAX_INSTANCES > 1
#define DOWNWARD_RANGEFINDER_MAX_INSTANCES 2
#else
#define DOWNWARD_RANGEFINDER_MAX_INSTANCES 1
#endif
#endif

class AP_AHRS;

/**
 * @class NavEKF2_core
 * @brief EKF2 Computational Backend - Core Navigation Filter Implementation
 * 
 * @details This class implements a single computational "core" or "lane" of the
 *          NavEKF2 Extended Kalman Filter. The EKF2 system typically runs multiple
 *          cores in parallel (one per available IMU) to provide redundancy and
 *          robustness through independent state estimation with different sensor sources.
 * 
 * Filter Architecture:
 * - 24-state Extended Kalman Filter with quaternion attitude representation (28 total states)
 * - Prediction-update cycle architecture with buffered measurements
 * - Sequential measurement fusion (one observation at a time)
 * - Time-delayed fusion using ring buffers to align measurements with IMU data
 * 
 * State Estimation:
 * - Attitude: Full 3D orientation via quaternion + rotation error vector
 * - Velocity: 3D velocity in NED frame (m/s)
 * - Position: 3D position relative to EKF origin in NED frame (m)
 * - IMU Biases: Gyro bias, gyro scale, and Z-accel bias
 * - Earth/Body Magnetic Field: 3D field estimates for magnetometer calibration
 * - Wind: 2D horizontal wind velocity estimate
 * 
 * Coordinate Frames:
 * - NED (North-East-Down): Earth-fixed navigation frame, right-handed
 * - Body Frame: Vehicle-fixed frame, X forward, Y right, Z down, right-handed
 * - All states in NED unless specified as body frame (biases, body magnetic field)
 * 
 * Unit Conventions:
 * - Position: meters (m)
 * - Velocity: meters per second (m/s)  
 * - Angles: radians (rad)
 * - Angular rates: radians per second (rad/s)
 * - Acceleration: meters per second squared (m/s²)
 * - Magnetic field: gauss/1000 (milligauss)
 * - Time: seconds (s) for intervals, milliseconds (ms) for timestamps
 * 
 * Measurement Fusion:
 * - IMU: Delta angles and velocities at high rate (typically 400 Hz or higher)
 * - GPS: Position and velocity at 5-10 Hz
 * - Magnetometer: 3-axis magnetic field at 10-100 Hz (sequential fusion per axis)
 * - Barometer: Altitude at 10-50 Hz
 * - Airspeed: True airspeed for fixed-wing at 5-10 Hz
 * - Optical Flow: Angular rates for height/velocity when GPS unavailable
 * - Range Finder: Height above ground for terrain following
 * - Range Beacons: Ranges to fixed beacons for indoor positioning
 * - External Nav: Position/attitude from external systems (e.g., motion capture)
 * 
 * Core Selection:
 * - Multiple cores run in parallel with different IMUs
 * - Front-end selects "primary" core based on errorScore()
 * - Lane switching occurs when primary core health degrades
 * - Provides fault tolerance for IMU failures
 * 
 * Initialization Requirements:
 * - Vehicle must be stationary for initial alignment
 * - Requires valid IMU data (accelerometer for tilt, gyro for bias learning)
 * - Magnetometer needed for yaw alignment (or GPS velocity for in-flight init)
 * - GPS quality must meet pre-flight check thresholds for position reference
 * 
 * Real-Time Performance:
 * - Target update rate: 100 Hz (EKF_TARGET_DT = 0.01s)
 * - Optimized with GCC O2 pragmas for computational efficiency
 * - Covariance prediction is most expensive operation (~70% of CPU time)
 * - Sequential fusion reduces per-update computational load
 * 
 * Numerical Stability:
 * - Covariance matrix forced symmetric after prediction
 * - Diagonal variance terms constrained to prevent negative values
 * - State limits enforced to prevent divergence
 * - Innovation consistency checks detect measurement outliers
 * - Condition number monitoring for numerical health
 * 
 * @note Inherits from NavEKF_core_common which provides common functionality
 *       shared between EKF2 and EKF3 implementations.
 * 
 * @warning This is safety-critical navigation code. Modifications must be:
 *          - Mathematically validated against filter derivation
 *          - Tested extensively in SITL and hardware
 *          - Verified for numerical stability across all flight conditions
 *          - Profiled for real-time performance constraints
 * 
 * Thread Safety:
 * - Not thread-safe - designed to be called from single EKF update thread
 * - Front-end provides inter-core synchronization
 * - Semaphore protection required for state queries from other threads
 * 
 * Memory Usage:
 * - Dominated by 24x24 covariance matrix (24*24*sizeof(ftype) = 2.3 KB single, 4.6 KB double)
 * - Ring buffers for IMU and observations (~1-2 KB depending on buffer depths)
 * - Total per-core: approximately 5-10 KB depending on precision and buffer sizes
 * 
 * @see NavEKF2 for front-end that manages multiple cores
 * @see AP_NavEKF_core_common for common base functionality
 * @see https://github.com/priseborough/InertialNav for mathematical derivation
 * 
 * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:84-1215
 */
class NavEKF2_core : public NavEKF_core_common
{
public:
    /**
     * @brief Constructor for NavEKF2 core backend
     * 
     * @param[in] _frontend Pointer to NavEKF2 front-end managing this core
     * 
     * @details Initializes a new EKF2 core instance. Each core operates independently
     *          with its own state vector, covariance matrix, and measurement buffers.
     *          The front-end manages multiple cores and selects the primary based on
     *          health metrics.
     * 
     * @note Does not initialize filter states - call InitialiseFilterBootstrap() for that
     * @note Multiple cores typically created, one per available IMU
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:88
     */
    NavEKF2_core(class NavEKF2 *_frontend);

    /**
     * @brief Setup and configure this core backend
     * 
     * @param[in] _imu_index Preferred IMU sensor index to use for this core
     * @param[in] _core_index Index identifying this core in the core array
     * 
     * @return true if setup successful, false on failure
     * 
     * @details Configures the core with IMU assignment and allocates required resources.
     *          Called during EKF system initialization before filter bootstrap.
     * 
     * Setup Operations:
     * - Assigns preferred IMU sensor index
     * - Sets core identification number
     * - Allocates measurement buffers
     * - Initializes EKFGSF yaw estimator if enabled
     * 
     * @note Must be called before InitialiseFilterBootstrap()
     * @note Core index used for identification in logging and lane selection
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:91
     */
    bool setup_core(uint8_t _imu_index, uint8_t _core_index);
    
    /**
     * @brief Initialize filter states from accelerometer and magnetometer data
     * 
     * @return true if initialization successful, false if conditions not met
     * 
     * @details Performs initial alignment and state initialization using static
     *          sensor measurements. This is the bootstrap process that establishes
     *          initial attitude, position reference, and sensor biases.
     * 
     * Initialization Sequence:
     * 1. Align tilt (roll/pitch) from accelerometer gravity vector
     * 2. Align yaw from magnetometer (or leave unaligned for GPS yaw init)
     * 3. Initialize position from GPS (if available) or set to origin
     * 4. Initialize velocity to zero (static assumption)
     * 5. Initialize covariance matrix with appropriate uncertainties
     * 6. Initialize magnetic field states from magnetometer or WMM
     * 
     * Prerequisites:
     * - Vehicle must be stationary (critical assumption)
     * - Valid IMU data (accelerometer for tilt, gyro for bias)
     * - Magnetometer data recommended (for yaw alignment)
     * - GPS quality checks passed if using GPS for position
     * 
     * Failure Conditions:
     * - Excessive IMU noise or vibration
     * - Accelerometer not reading approximately 1g
     * - Magnetometer data invalid or inconsistent
     * - GPS quality insufficient for position reference
     * 
     * @warning Vehicle MUST be stationary during this process
     * @warning Large initial attitude errors will cause immediate divergence
     * @note Can be called repeatedly if initial attempt fails
     * @note Alternative: in-flight initialization using GPS velocity for yaw
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:95
     */
    bool InitialiseFilterBootstrap(void);

    /**
     * @brief Update filter states with new IMU data
     * 
     * @param[in] predict Flag indicating if new prediction cycle should start
     * 
     * @details Main EKF update method called at IMU rate (typically 400 Hz).
     *          Performs prediction step when IMU data buffered and fusion step
     *          when measurements available at fusion time horizon.
     * 
     * Update Cycle:
     * 1. Buffer new IMU measurements (delta angles and velocities)
     * 2. If predict==true: Run prediction step
     *    - Update states using IMU measurements (UpdateStrapdownEquationsNED)
     *    - Propagate covariance forward (CovariancePrediction)
     *    - Enforce constraints (ConstrainStates, ConstrainVariances)
     * 3. Check for available measurements at fusion time horizon
     * 4. Fuse measurements sequentially:
     *    - GPS position and velocity (SelectVelPosFusion)
     *    - Magnetometer (SelectMagFusion)
     *    - Barometer/rangefinder height (selectHeightForFusion)
     *    - Airspeed (SelectTasFusion)
     *    - Optical flow (SelectFlowFusion)
     *    - Range beacons (SelectRngBcnFusion)
     *    - External navigation (fusion logic in update methods)
     * 5. Update filter status and health metrics
     * 6. Calculate output states for current time
     * 
     * Prediction vs Update:
     * - Prediction: IMU-driven state propagation (high rate, ~400 Hz)
     * - Update: Measurement fusion (varies by sensor, 1-100 Hz)
     * - Predict flag allows multiple IMU samples before prediction step
     * 
     * @note Called at IMU data rate (typically 400 Hz)
     * @note Prediction step much more expensive than simple IMU buffering
     * @note Filter remains initialized through this method's calls
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:99
     */
    void UpdateFilter(bool predict);

    /**
     * @brief Check filter health status
     * 
     * @return true if filter is healthy and suitable for navigation, false otherwise
     * 
     * @details Performs comprehensive health assessment by checking multiple
     *          indicators of filter validity and numerical stability.
     * 
     * Health Checks:
     * - States initialized (statesInitialised flag)
     * - No NaN values in state vector or covariance
     * - Innovation consistency checks passing
     * - Position and velocity estimates valid
     * - Aiding sources available and healthy
     * - Filter not diverged (position within limits)
     * - Sufficient time since initialization
     * 
     * Unhealthy Conditions:
     * - States contain NaN (numerical error)
     * - All aiding sources timed out (GPS, mag, etc.)
     * - Innovation test ratios exceed thresholds
     * - Position states exceed EK2_POSXY_STATE_LIMIT
     * - Filter recently reset or initialized
     * 
     * @note Used by front-end for core selection and lane switching
     * @note Also used by flight control to determine if EKF solution trustworthy
     * @note Health can transition false→true (recovery) or true→false (degradation)
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:102
     */
    bool healthy(void) const;

    /**
     * @brief Calculate consolidated error score for core selection
     * 
     * @return Error score where lower is better, 0 is perfect
     * 
     * @details Computes a single scalar metric representing overall filter error
     *          and health. Used by front-end to select primary core when multiple
     *          cores are running.
     * 
     * Error Score Components:
     * - Innovation test ratios (velocity, position, height, mag, airspeed)
     * - Covariance matrix diagonal terms (uncertainty)
     * - Time since last successful measurement fusion
     * - Filter initialization status
     * - Numerical health indicators
     * 
     * Score Interpretation:
     * - 0.0: Perfect, no errors (rarely achieved)
     * - 0.0-0.5: Excellent, primary core candidate
     * - 0.5-1.0: Good, acceptable backup core
     * - 1.0-2.0: Marginal, aiding issues or high uncertainty
     * - >2.0: Poor, not suitable for navigation
     * 
     * Lane Selection:
     * - Front-end compares errorScore() across all cores
     * - Core with lowest score becomes primary
     * - Hysteresis applied to prevent rapid switching
     * - Score difference threshold required for switch
     * 
     * @note Lower score indicates better filter performance
     * @note Used continuously by front-end for lane selection
     * @note Score can change rapidly with measurement availability
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:106
     */
    ftype errorScore(void) const;

    /**
     * @brief Get North-East horizontal position estimate
     * 
     * @param[out] posNE North and East position components (m)
     * 
     * @return true if solution valid for flight control, false otherwise
     * 
     * @details Returns the latest calculated horizontal position in the NE plane
     *          relative to the EKF origin. Position propagated to current time
     *          using output observer.
     * 
     * Position Convention:
     * - posNE[0]: North position (m), positive north of origin
     * - posNE[1]: East position (m), positive east of origin
     * - Relative to EKF origin set by setOriginLLH() or first GPS fix
     * 
     * Return Value Interpretation:
     * - true: Position estimate valid, suitable for flight control
     * - false: Position degraded or invalid, DO NOT use for control
     * 
     * Invalid Conditions:
     * - Filter not initialized
     * - Position aiding unavailable (no GPS, beacons, or external nav)
     * - Position timeout (aiding lost for too long)
     * - States contain NaN or exceed limits
     * - Filter unhealthy
     * 
     * When false returned:
     * - Output contains last known position OR dead-reckoned position
     * - Value provided for telemetry/logging only
     * - Flight controller should enter failsafe mode
     * - Do NOT use for waypoint navigation or position hold
     * 
     * @warning If false returned, DO NOT use output for flight control
     * @note Position accuracy depends on aiding source quality (GPS, beacons, etc.)
     * @note Updated at IMU rate via output observer interpolation
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:111
     */
    bool getPosNE(Vector2p &posNE) const;

    /**
     * @brief Get Down position estimate (height)
     * 
     * @param[out] posD Down position component (m, positive is down/below origin)
     * 
     * @return true if solution valid for flight control, false otherwise
     * 
     * @details Returns the latest calculated vertical position (height) relative
     *          to the EKF origin. Position propagated to current time using
     *          output observer.
     * 
     * Position Convention:
     * - posD: Down position (m), positive below origin, negative above
     * - NED frame: Down is positive downward direction
     * - Relative to EKF origin height datum
     * 
     * Height Sources:
     * - Barometer (most common, smooth but drifts)
     * - GPS altitude (absolute but noisy)
     * - Range finder (AGL, limited range)
     * - Range beacons (if configured)
     * - External navigation system
     * 
     * Return Value Interpretation:
     * - true: Height valid for altitude hold, terrain following, landing
     * - false: Height degraded, DO NOT use for control
     * 
     * Invalid Conditions:
     * - All height sources timed out
     * - Filter not initialized
     * - States contain NaN
     * - Recent height reset without convergence
     * 
     * @warning If false returned, DO NOT use output for flight control
     * @note Height datum can shift during flight (baro drift, origin corrections)
     * @note More stable than horizontal position (always has some height reference)
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:116
     */
    bool getPosD(postype_t &posD) const;

    /**
     * @brief Get NED velocity estimate
     * 
     * @param[out] vel 3D velocity vector NED (m/s)
     * 
     * @details Returns the latest velocity estimate in North-East-Down frame.
     *          Velocity propagated to current time using output observer.
     * 
     * Velocity Convention:
     * - vel[0]: North velocity (m/s), positive moving north
     * - vel[1]: East velocity (m/s), positive moving east
     * - vel[2]: Down velocity (m/s), positive descending
     * 
     * Velocity Sources:
     * - GPS velocity (primary for horizontal)
     * - IMU integration (short-term accuracy)
     * - Optical flow (GPS-denied)
     * - External navigation system
     * 
     * Characteristics:
     * - Updated at IMU rate (high rate, low latency)
     * - Accuracy depends on aiding availability
     * - Can remain accurate briefly after GPS loss (IMU propagation)
     * - Drift increases without external aiding
     * 
     * @note Always returns current best estimate (never returns false)
     * @note Check healthy() or velocity timeout flags for validity
     * @note Velocity typically more reliable than position in GPS-denied scenarios
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:119
     */
    void getVelNED(Vector3f &vel) const;

    /**
     * @brief Get true airspeed vector estimate in body frame
     * 
     * @param[out] vel Airspeed vector XYZ body frame (m/s)
     * 
     * @return true if airspeed estimate available, false otherwise
     * 
     * @details Calculates true airspeed vector in body frame by subtracting
     *          estimated wind from ground velocity and rotating to body frame.
     * 
     * Airspeed Vector Convention:
     * - vel[0]: X airspeed (m/s), positive = air flowing from front
     * - vel[1]: Y airspeed (m/s), positive = air flowing from right
     * - vel[2]: Z airspeed (m/s), positive = air flowing from below
     * - Body frame: X forward, Y right, Z down
     * 
     * Calculation:
     * 1. Get ground velocity in NED frame
     * 2. Subtract wind velocity estimate
     * 3. Rotate resulting air-relative velocity to body frame
     * 4. Return as body-frame airspeed vector
     * 
     * Requirements:
     * - Valid velocity estimate
     * - Wind states estimated (requires airspeed sensor or maneuvers)
     * - Valid attitude quaternion for rotation
     * 
     * Returns false when:
     * - Filter not initialized
     * - Wind states not observable
     * - Velocity estimate invalid
     * 
     * Use Cases:
     * - Airspeed indication for pilots
     * - Energy management calculations
     * - Angle of attack / sideslip estimation
     * - Stall prevention systems
     * 
     * @note Requires wind estimation (airspeed sensor or dynamic flight)
     * @note Fixed-wing applications primarily
     * @note Accuracy depends on wind estimate quality
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:124
     */
    bool getAirSpdVec(Vector3f &vel) const;

    /**
     * @brief Get vertical velocity (rate of change of height)
     * 
     * @return Vertical velocity in down direction (m/s), positive = descending
     * 
     * @details Returns the rate of change of vertical position (dPosD/dt).
     *          This is kinematically consistent with position but may differ
     *          from the EKF velocity state due to height corrections.
     * 
     * Difference from vel[2]:
     * - This method: Derivative of position (includes height corrections)
     * - getVelNED()[2]: EKF velocity state (smoother, delayed corrections)
     * - This fluctuates more but maintains position consistency
     * - Velocity state is smoother but can lag position updates
     * 
     * Calculation:
     * - Computed from complementary filter tracking position error
     * - Includes height innovations and corrections
     * - Kinematically consistent with getPosD()
     * 
     * Use Cases:
     * - Vertical speed indication
     * - Climb/descent rate control
     * - Terrain following
     * - Altitude hold loop
     * 
     * Convention:
     * - Positive: Moving down (descending)
     * - Negative: Moving up (climbing)
     * - Units: meters per second (m/s)
     * 
     * @note Includes height measurement corrections (can be noisy)
     * @note Guaranteed consistent with position output
     * @note May show transients during height resets
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:128
     */
    float getPosDownDerivative(void) const;

    /**
     * @brief Get gyro bias estimates
     * 
     * @param[out] gyroBias Gyro bias XYZ body frame (rad/s)
     * 
     * @details Returns the current gyroscope bias estimates learned by the EKF.
     *          These are the constant offsets subtracted from gyro measurements
     *          during IMU data correction.
     * 
     * Bias Convention:
     * - gyroBias[0]: X-axis gyro bias (rad/s), roll rate bias
     * - gyroBias[1]: Y-axis gyro bias (rad/s), pitch rate bias
     * - gyroBias[2]: Z-axis gyro bias (rad/s), yaw rate bias
     * - Body frame: bias in sensor frame
     * 
     * Bias Learning:
     * - Estimated continuously by EKF during flight
     * - Converges during steady flight with good aiding
     * - Temperature-dependent (can vary with IMU temperature)
     * - Improves attitude estimation accuracy
     * 
     * Typical Values:
     * - Consumer IMUs: 0.001 to 0.01 rad/s (0.05 to 0.5 deg/s)
     * - Well-calibrated: < 0.002 rad/s (< 0.1 deg/s)
     * - Large values indicate calibration or IMU issues
     * 
     * Use Cases:
     * - IMU health monitoring
     * - Pre-flight bias learning status
     * - Post-flight bias logging/analysis
     * - Bias initialization for other cores
     * 
     * @note Biases updated by filter prediction step
     * @note Takes time to converge (typically 30-60 seconds of flight)
     * @note Check delAngBiasLearned flag for convergence status
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:131
     */
    void getGyroBias(Vector3f &gyroBias) const;

    /**
     * @brief Get gyro scale factor errors as percentages
     * 
     * @param[out] gyroScale Scale factor errors XYZ (percentage, e.g., 1.0 = 1%)
     * 
     * @details Returns the gyroscope scale factor errors learned by the EKF.
     *          These are multiplicative corrections applied to gyro measurements.
     * 
     * Scale Factor Convention:
     * - gyroScale[0]: X-axis scale error (%), applied to roll rate
     * - gyroScale[1]: Y-axis scale error (%), applied to pitch rate
     * - gyroScale[2]: Z-axis scale error (%), applied to yaw rate
     * - Positive = gyro reading too high, negative = too low
     * 
     * Correction Application:
     * - Corrected = Measured * (1 + scale/100)
     * - Example: scale = -2.0 means gyro reads 2% low
     * - Applied during delta angle correction
     * 
     * Scale Error Sources:
     * - Manufacturing tolerances
     * - Temperature effects
     * - Non-linearity in sensor response
     * - Cross-axis sensitivity
     * 
     * Typical Values:
     * - Most IMUs: < 1% (< 1.0)
     * - Good calibration: < 0.5%
     * - Values > 5% indicate IMU problems
     * 
     * @note Scale factor learning requires dynamic flight
     * @note Convergence slower than bias estimation
     * @note Less critical than bias for navigation accuracy
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:134
     */
    void getGyroScaleErrorPercentage(Vector3f &gyroScale) const;

    /**
     * @brief Reset gyro bias estimates to zero
     * 
     * @details Forces gyro bias states to zero and resets their covariances
     *          to initial uncertainty values. Used when bias estimates become
     *          invalid or for testing.
     * 
     * Reset Operations:
     * - Sets gyro_bias states to [0, 0, 0]
     * - Resets bias state covariances to InitialGyroBiasUncertainty()
     * - Clears delAngBiasLearned flag
     * - Forces filter to re-learn biases from scratch
     * 
     * When to Use:
     * - IMU temperature changed significantly
     * - Suspected invalid bias estimates
     * - After IMU sensor replacement
     * - Testing filter convergence
     * 
     * Consequences:
     * - Temporary attitude estimation degradation
     * - Requires time to re-converge (30-60 seconds)
     * - May cause brief attitude errors if in flight
     * 
     * @warning Use with caution in flight - causes transient attitude errors
     * @note Filter will re-learn biases automatically
     * @note Prefer to let filter converge naturally during pre-flight
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:137
     */
    void resetGyroBias(void);

    /**
     * @brief Reset height datum to current altitude
     * 
     * @return true if reset performed, false if using range finder
     * 
     * @details Resets the barometer and EKF so the current altitude reads zero,
     *          while maintaining consistent absolute altitude by adjusting the
     *          EKF origin height. Used for relative altitude operations.
     * 
     * Reset Operations:
     * 1. Note current EKF height (posD) and origin height
     * 2. Reset barometer datum to current pressure
     * 3. Set EKF posD state to zero
     * 4. Adjust EKF origin height = old_origin + old_posD
     * 5. Result: New posD = 0, but absolute altitude unchanged
     * 
     * Use Cases:
     * - Relative altitude operations (e.g., competition altitude zero)
     * - Compensating for baro drift during long flights
     * - Setting takeoff point as height reference
     * 
     * Limitations:
     * - Only works when using barometer as height source
     * - Returns false if using range finder (no reset performed)
     * - Does not affect GPS altitude (WGS-84 ellipsoid)
     * 
     * Side Effects:
     * - All logged height values relative to new datum
     * - Terrain database offsets adjusted
     * - Rally point heights remain in old datum
     * 
     * @note Does NOT work with range finder height source
     * @note Maintains continuity of absolute altitude
     * @note Affects only barometric altitude reference
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:144
     */
    bool resetHeightDatum(void);

    /**
     * @brief Get optical flow navigation control limits
     * 
     * @param[out] ekfGndSpdLimit Maximum ground speed for optical flow (m/s)
     * @param[out] ekfNavVelGainScaler Velocity gain scale factor (dimensionless)
     * 
     * @details Returns control limits needed when using optical flow for
     *          navigation. These limits compensate for optical flow sensor
     *          constraints and noise characteristics.
     * 
     * Ground Speed Limit:
     * - Maximum safe horizontal speed for optical flow navigation
     * - Based on optical flow maximum measurable rate
     * - Based on vehicle height above ground
     * - Higher altitude = lower speed limit (same angular rate)
     * 
     * Calculation: max_speed = max_flow_rate * height / focal_length
     * 
     * Velocity Gain Scaler:
     * - Factor to scale position/velocity controller gains
     * - Compensates for increased noise with height
     * - Reduces gains at high altitude (worse flow quality)
     * - Maintains stable control across height range
     * 
     * Application in Flight Control:
     * - Limit commanded velocities to ekfGndSpdLimit
     * - Multiply velocity controller gains by ekfNavVelGainScaler
     * - Prevents flow rate saturation
     * - Maintains stability with degraded flow quality
     * 
     * @note Only relevant when using optical flow for position hold
     * @note Both outputs can change with height (altitude-dependent)
     * @note Critical for preventing loss of control at high altitudes
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:148
     */
    void getEkfControlLimits(float &ekfGndSpdLimit, float &ekfNavVelGainScaler) const;

    /**
     * @brief Get Z-axis accelerometer bias estimate
     * 
     * @param[out] zbias Z-axis accel bias (m/s², positive = accel reads high)
     * 
     * @details Returns the vertical accelerometer bias learned by the EKF.
     *          This is the constant offset subtracted from Z-axis accelerometer
     *          measurements.
     * 
     * Bias Convention:
     * - Positive: Accelerometer reads high (measures more downward accel)
     * - Negative: Accelerometer reads low
     * - Body frame Z-axis (down in body frame)
     * - Units: m/s²
     * 
     * Bias Learning:
     * - Estimated from height measurements (baro, GPS)
     * - Converges during steady altitude or known vertical motion
     * - Improves vertical velocity estimation
     * - Critical for accurate altitude hold
     * 
     * Typical Values:
     * - Well-calibrated: < 0.1 m/s²
     * - Consumer IMUs: 0.05 to 0.3 m/s²
     * - Large values (> 0.5 m/s²) indicate calibration issues
     * 
     * Impact on Flight:
     * - Affects altitude hold performance
     * - Impacts terrain following accuracy
     * - Influences landing flare execution
     * - Critical for vertical velocity estimation
     * 
     * @note Only Z-axis bias estimated (X,Y biases not observable)
     * @note Converges faster than gyro biases
     * @note Temperature-dependent like gyro biases
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:151
     */
    void getAccelZBias(float &zbias) const;

    /**
     * @brief Get wind velocity estimate
     * 
     * @param[out] wind 3D wind vector NED (m/s)
     * 
     * @details Returns the current wind velocity estimate in North-East-Down frame.
     *          Wind is defined as air mass movement relative to ground.
     * 
     * Wind Convention:
     * - wind[0]: North wind component (m/s), positive = wind from south
     * - wind[1]: East wind component (m/s), positive = wind from west
     * - wind[2]: Down wind component (m/s), typically zero (no vertical wind)
     * - Positive value = air moving IN that axis direction
     * 
     * Example: wind = [5, 0, 0] means 5 m/s wind FROM the south
     * 
     * Wind Estimation:
     * - Requires airspeed sensor OR dynamic flight (turns, altitude changes)
     * - Converges faster with airspeed sensor
     * - Can estimate without airspeed but requires maneuvering
     * - Vertical wind (wind[2]) typically constrained to zero
     * 
     * Use Cases:
     * - True airspeed calculation from ground speed
     * - Compensating for wind drift
     * - Energy management (headwind/tailwind awareness)
     * - Optimal routing for fixed-wing
     * - Soaring (thermal detection)
     * 
     * Accuracy:
     * - With airspeed: typically ±1 m/s after convergence
     * - Without airspeed: ±2-3 m/s, requires dynamic flight
     * - Convergence time: 10-30 seconds with airspeed, longer without
     * 
     * @note wind[2] usually zero (vertical wind not estimated)
     * @note Check inhibitWindStates flag - wind estimation may be disabled
     * @note Returns [0,0,0] if wind states not observable
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:154
     */
    void getWind(Vector3f &wind) const;

    /**
     * @brief Get earth magnetic field estimate in NED frame
     * 
     * @param[out] magNED Earth field components NED (gauss/1000 = milligauss)
     * 
     * @details Returns the learned earth magnetic field vector in navigation frame.
     *          This is the expected magnetic field at the current location in
     *          NED coordinates.
     * 
     * Magnetic Field Convention:
     * - magNED[0]: North component (mGauss), horizontal field north
     * - magNED[1]: East component (mGauss), horizontal field east (typically ~0)
     * - magNED[2]: Down component (mGauss), vertical field downward (positive down)
     * - Units: measurement units / 1000 = milligauss
     * - NED frame: relative to true north
     * 
     * Field Learning:
     * - Initialized from World Magnetic Model (WMM) tables
     * - Refined during flight using magnetometer measurements
     * - Adapts to local magnetic anomalies
     * - Improves yaw estimation accuracy
     * 
     * Typical Values (mid-latitudes):
     * - Total field: 400-600 mGauss
     * - North component: 150-400 mGauss
     * - East component: ~0 mGauss (aligned with true north)
     * - Down component: 300-500 mGauss (positive downward)
     * 
     * Use Cases:
     * - Magnetic declination calculation
     * - Compass calibration validation
     * - Magnetic anomaly detection
     * - Yaw observability assessment
     * 
     * @note Units are gauss/1000 (milligauss)
     * @note Check magFieldLearned flag for convergence
     * @note May differ from WMM due to local anomalies
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:157
     */
    void getMagNED(Vector3f &magNED) const;

    /**
     * @brief Get body magnetic field offsets
     * 
     * @param[out] magXYZ Body field offsets XYZ (gauss/1000 = milligauss)
     * 
     * @details Returns the learned body-fixed magnetic field offsets (hard iron).
     *          These are constant magnetic fields in the vehicle body frame caused
     *          by ferromagnetic materials and current-carrying wires.
     * 
     * Body Field Convention:
     * - magXYZ[0]: X-axis offset (mGauss), forward direction
     * - magXYZ[1]: Y-axis offset (mGauss), right direction
     * - magXYZ[2]: Z-axis offset (mGauss), down direction
     * - Body frame: fixed to vehicle structure
     * - These are ADDED to measured field to get true field
     * 
     * Hard Iron Effects:
     * - Constant offsets from ferromagnetic materials
     * - Steel in vehicle structure
     * - Permanent magnets (motors, speakers)
     * - Magnetic latches, fasteners
     * 
     * Soft Iron Effects:
     * - NOT modeled by these offsets (scale factor not estimated here)
     * - Requires separate compass calibration
     * 
     * Field Learning:
     * - Estimated during flight with good GPS/velocity aiding
     * - Requires vehicle rotation in 3D (pitch, roll, yaw)
     * - Converges over time if learning enabled
     * - Can vary with motor current (dynamic offset)
     * 
     * Typical Values:
     * - Small vehicles: 10-100 mGauss per axis
     * - Large metal vehicles: 100-300 mGauss per axis
     * - Values > 500 mGauss indicate strong interference
     * 
     * @note Units are gauss/1000 (milligauss)
     * @note These are hard iron offsets only
     * @note Check inhibitMagStates flag - learning may be disabled
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:160
     */
    void getMagXYZ(Vector3f &magXYZ) const;

    /**
     * @brief Get estimated magnetometer offsets
     * 
     * @param[in]  mag_idx    Magnetometer sensor index
     * @param[out] magOffsets Estimated offsets XYZ (gauss/1000)
     * 
     * @return true if offsets valid and available, false otherwise
     * 
     * @details Returns the learned magnetometer offsets for a specific compass
     *          sensor. These offsets can be used to update compass calibration
     *          parameters.
     * 
     * Difference from getMagXYZ():
     * - getMagXYZ(): Body-fixed offsets from EKF state vector
     * - This method: Sensor-specific offsets for compass calibration
     * - This accounts for sensor mounting and individual sensor biases
     * 
     * Offset Validity:
     * - Returns false if: filter not initialized, mag states not learned,
     *   mag_idx invalid, insufficient flight time
     * - Returns true if: offsets converged and reliable
     * 
     * Use Cases:
     * - In-flight compass calibration
     * - Validating pre-flight compass calibration
     * - Detecting compass degradation over time
     * - Automatic compass parameter updates
     * 
     * Offset Application:
     * - Offsets can be written to COMPASS_OFS parameters
     * - Improves heading accuracy without re-calibration
     * - Particularly useful for aging compasses
     * 
     * @param mag_idx Which magnetometer sensor (0, 1, 2, etc.)
     * @note Check return value before using offsets
     * @note Offsets require significant flight time to converge
     * @note Only valid if magnetic field states have been learned
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:164
     */
    bool getMagOffsets(uint8_t mag_idx, Vector3f &magOffsets) const;

    /**
     * @brief Get current latitude, longitude, and height
     * 
     * @param[out] loc Location structure (lat, lon, alt)
     * 
     * @return true if location available, false if no data
     * 
     * @details Returns the vehicle's calculated position in WGS-84 coordinates.
     *          If EKF position invalid, returns last raw GPS measurement instead.
     * 
     * Location Convention:
     * - loc.lat: Latitude in degrees * 1e7 (int32_t)
     * - loc.lng: Longitude in degrees * 1e7 (int32_t)
     * - loc.alt: Altitude in cm above MSL or ellipsoid
     * - WGS-84 datum
     * 
     * Position Source Priority:
     * 1. EKF NED position + origin (preferred, filtered)
     * 2. Last GPS measurement (fallback if EKF invalid)
     * 3. No data (returns false)
     * 
     * Return Value Interpretation:
     * - true: Some location available (EKF or GPS)
     * - false: No location data available at all
     * 
     * CRITICAL - Check getFilterStatus():
     * - Return value only indicates data availability
     * - Does NOT indicate if safe for flight control
     * - MUST call getFilterStatus() for control validity
     * - Use getLLH() for logging/telemetry only
     * 
     * Flight Control Usage:
     * - Check getFilterStatus().flags.horiz_pos_abs = true
     * - Check healthy() = true
     * - Check getPosNE() = true
     * - Only then use for navigation
     * 
     * Altitude Reference:
     * - Depends on EKF origin height setting
     * - Can be MSL (barometer) or ellipsoid (GPS)
     * - Check origin setting for interpretation
     * 
     * @warning Return true does NOT mean safe for flight control
     * @note Always verify with getFilterStatus() before control use
     * @note May return GPS position even when EKF diverged
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:170
     */
    bool getLLH(Location &loc) const;

    /**
     * @brief Get EKF origin location
     * 
     * @param[out] loc Origin location (lat, lon, alt)
     * 
     * @return true if origin has been set, false otherwise
     * 
     * @details Returns the latitude, longitude, and height of the EKF NED origin.
     *          All NED position estimates are relative to this reference point.
     * 
     * Origin Purpose:
     * - Defines zero point for NED navigation frame
     * - All position states relative to this location
     * - North = 0° true north from this point
     * - East = 90° true from this point
     * - Down = 0 at this altitude
     * 
     * Origin Setting:
     * - Set automatically at first GPS fix (if origin not pre-set)
     * - Can be set manually via setOriginLLH()
     * - Once set, typically remains fixed for flight
     * - Height can be refined during flight (correctEkfOriginHeight)
     * 
     * Origin Location:
     * - loc.lat: Origin latitude (degrees * 1e7)
     * - loc.lng: Origin longitude (degrees * 1e7)  
     * - loc.alt: Origin altitude (cm, MSL or ellipsoid)
     * 
     * Use Cases:
     * - Converting NED positions to lat/lon
     * - Logging EKF reference frame
     * - Multi-vehicle coordination
     * - Fence/rally point offset calculation
     * 
     * Returns false when:
     * - Filter not initialized
     * - No GPS fix received yet
     * - Origin not manually set
     * 
     * @note Origin typically set at first GPS lock
     * @note Height component can drift with baro (correctEkfOriginHeight)
     * @note Critical for position interpretation
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:175
     */
    bool getOriginLLH(Location &loc) const;

    /**
     * @brief Set EKF origin location manually
     * 
     * @param[in] loc Desired origin location (lat, lon, alt)
     * 
     * @return true if origin set successfully, false if rejected
     * 
     * @details Manually sets the NED frame origin to a specified location.
     *          All subsequent position estimates will be relative to this point.
     * 
     * Success Conditions:
     * - Origin not already set (validOrigin = false)
     * - Not using GPS absolute positioning yet
     * - Filter not already aligned with different origin
     * 
     * Failure Conditions (returns false):
     * - Origin already set and locked
     * - GPS aiding active with different origin
     * - Filter already initialized with position
     * - Attempting to change mid-flight
     * 
     * Use Cases:
     * - Setting known takeoff location before GPS lock
     * - Indoor/GPS-denied operations with known starting point
     * - Multi-vehicle coordination (shared origin)
     * - Surveyed launch point for precision applications
     * 
     * Consequences of Setting Origin:
     * - All position states become relative to this location
     * - NED frame fixed at this point
     * - Fence/rally points interpreted relative to origin
     * - Cannot be changed once GPS aiding begins
     * 
     * Altitude Datum:
     * - loc.alt should match intended height reference
     * - Typically MSL for barometer, ellipsoid for GPS
     * - Affects height estimates throughout flight
     * 
     * @warning Cannot change origin after GPS aiding starts
     * @warning Incorrect origin causes position errors
     * @note Prefer automatic origin from first GPS fix when possible
     * @note Only use when starting location precisely known
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:180
     */
    bool setOriginLLH(const Location &loc);

    /**
     * @brief Get height above ground level estimate
     * 
     * @param[out] HAGL Height above ground level (m, positive = above ground)
     * 
     * @return true if ground height being estimated, false otherwise
     * 
     * @details Returns the estimated height above the ground surface beneath
     *          the vehicle. Requires range finder or terrain database.
     * 
     * HAGL Estimation Methods:
     * 1. Range finder: Direct measurement (most accurate when available)
     * 2. Terrain estimator: EKF terrain state from optical flow
     * 3. Terrain database + position: Database lookup (if available)
     * 
     * Returns false when:
     * - No range finder available
     * - Range finder out of range or failed
     * - Terrain estimator not converged
     * - No terrain database available
     * - Ground height unknown
     * 
     * Use Cases:
     * - Terrain following flight
     * - Automatic landing flare
     * - Obstacle avoidance vertical margins
     * - Low altitude warnings
     * - Range finder health validation
     * 
     * Accuracy:
     * - With range finder: ±0.05-0.5 m (depends on sensor)
     * - Terrain estimator: ±1-3 m (depends on conditions)
     * - Terrain database: ±10-30 m (database resolution)
     * 
     * Important Notes:
     * - HAGL not available over water (no terrain return)
     * - Terrain estimator requires optical flow and movement
     * - Can be unreliable over vegetation (radar altitude)
     * - Different from barometric altitude
     * 
     * @note Returns false if no ground height estimate available
     * @note Check return value before using HAGL
     * @note More accurate with range finder than terrain database
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:184
     */
    bool getHAGL(float &HAGL) const;

    /**
     * @brief Get Euler angle attitude
     * 
     * @param[out] eulers Euler angles roll, pitch, yaw (radians)
     * 
     * @details Returns vehicle attitude as Euler angles in radians.
     *          Extracted from quaternion state with proper singularity handling.
     * 
     * Euler Angle Convention:
     * - eulers[0]: Roll (rad), rotation about X-axis (forward)
     * - eulers[1]: Pitch (rad), rotation about Y-axis (right)
     * - eulers[2]: Yaw (rad), rotation about Z-axis (down)
     * - Rotation sequence: 3-2-1 (yaw-pitch-roll)
     * - Right-hand rule, NED frame
     * 
     * Angle Ranges:
     * - Roll: -π to +π (-180° to +180°)
     * - Pitch: -π/2 to +π/2 (-90° to +90°)
     * - Yaw: 0 to 2π (0° to 360°)
     * 
     * Sign Convention:
     * - Roll: Positive = right wing down
     * - Pitch: Positive = nose up
     * - Yaw: Positive = clockwise from north (viewed from above)
     * 
     * Gimbal Lock:
     * - Occurs at pitch = ±90° (straight up/down)
     * - Roll and yaw become ambiguous
     * - Use getQuaternion() if attitude near vertical
     * 
     * Use Cases:
     * - Pilot displays (more intuitive than quaternions)
     * - Simple control loops
     * - Logging and telemetry
     * - Attitude limits checking
     * 
     * @note Extracted from quaternion state (no separate Euler storage)
     * @note Gimbal lock at pitch = ±90°
     * @note Prefer getQuaternion() for 3D math to avoid singularities
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:187
     */
    void getEulerAngles(Vector3f &eulers) const;

    /**
     * @brief Get rotation matrix from body frame to NED frame
     * 
     * @param[out] mat 3x3 rotation matrix (direction cosine matrix)
     * 
     * @details Returns the direction cosine matrix (DCM) that transforms vectors
     *          from body frame to NED navigation frame. Extracted from quaternion.
     * 
     * Matrix Application:
     * - vecNED = mat * vecBody
     * - Transforms body-frame vector to NED frame
     * - Example: NED_velocity = mat * body_velocity
     * 
     * Matrix Properties:
     * - Orthonormal (rows/columns are unit vectors)
     * - Determinant = +1 (proper rotation)
     * - Transpose = Inverse (mat^T = mat^-1)
     * - Each column is body axis direction in NED frame
     * 
     * Matrix Elements:
     * - mat[0][0..2]: Body X-axis (forward) in NED components
     * - mat[1][0..2]: Body Y-axis (right) in NED components
     * - mat[2][0..2]: Body Z-axis (down) in NED components
     * 
     * Use Cases:
     * - Transforming sensor measurements to navigation frame
     * - Gravity compensation in body frame
     * - Control vector transformation
     * - Coordinate frame conversions
     * 
     * Inverse Transformation (NED to body):
     * - vecBody = mat^T * vecNED (transpose of matrix)
     * - Or use getRotationBodyToNED() then transpose
     * 
     * @note Extracted from quaternion state (no separate DCM storage)
     * @note Computationally more expensive than quaternion
     * @note Matrix always orthonormal (guaranteed by quaternion)
     * @see getQuaternion() for more efficient 3D rotations
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:190
     */
    void getRotationBodyToNED(Matrix3f &mat) const;

    /**
     * @brief Get attitude quaternion
     * 
     * @param[out] quat Quaternion representing rotation from NED to body frame
     * 
     * @details Returns the attitude quaternion from the EKF state vector.
     *          This is the preferred attitude representation avoiding gimbal lock.
     * 
     * Quaternion Convention:
     * - quat[0] (w): Scalar component
     * - quat[1] (x): X vector component
     * - quat[2] (y): Y vector component
     * - quat[3] (z): Z vector component
     * - Unit quaternion: w² + x² + y² + z² = 1
     * - Represents rotation FROM NED TO body frame
     * 
     * Quaternion Application:
     * - vecBody = quat * vecNED * quat_conjugate
     * - Rotates vector from NED to body frame
     * - Use quaternion math library functions
     * 
     * Advantages over Euler:
     * - No gimbal lock at any attitude
     * - Computationally efficient for rotations
     * - No trigonometric functions needed
     * - Numerically stable
     * - Smooth interpolation possible
     * 
     * Quaternion Properties:
     * - Always normalized (unit magnitude)
     * - Represents any 3D rotation uniquely
     * - Inverse = conjugate (for unit quaternions)
     * - Identity: [1, 0, 0, 0] (no rotation)
     * 
     * Use Cases:
     * - Attitude control (VTOL, aerobatics)
     * - 3D graphics and simulation
     * - Sensor fusion math
     * - Avoiding gimbal lock in all attitudes
     * 
     * @note This is the native EKF attitude representation
     * @note Preferred over Euler angles for computation
     * @note Quaternion normalized automatically by filter
     * @see getEulerAngles() for human-readable representation
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:193
     */
    void getQuaternion(Quaternion &quat) const;

    /**
     * @brief Get measurement innovations for various sensors
     * 
     * @param[out] velInnov Velocity innovations NED (m/s)
     * @param[out] posInnov Position innovations NED (m)
     * @param[out] magInnov Magnetometer innovations XYZ (gauss/1000)
     * @param[out] tasInnov True airspeed innovation (m/s)
     * @param[out] yawInnov Yaw innovation (rad)
     * 
     * @return true if innovations available, false otherwise
     * 
     * @details Returns the most recent measurement innovations (residuals) from
     *          the Kalman filter update steps. Innovations are the difference
     *          between predicted and actual measurements.
     * 
     * Innovation Definition:
     * - Innovation = Measurement - Prediction
     * - Positive = measurement > prediction
     * - Should be zero-mean white noise if filter tuned correctly
     * - Large innovations indicate model errors or bad measurements
     * 
     * Innovation Components:
     * - velInnov: [N, E, D] velocity (m/s)
     * - posInnov: [N, E, D] position (m)
     * - magInnov: [X, Y, Z] magnetometer body frame (mGauss)
     * - tasInnov: True airspeed scalar (m/s)
     * - yawInnov: Yaw angle (rad)
     * 
     * Innovation Interpretation:
     * - Small (~0): Filter predictions match measurements well
     * - Large: Model errors, bad measurements, or poor tuning
     * - Consistent bias: Systematic error (calibration, model)
     * - Random spikes: Measurement outliers
     * 
     * Use Cases:
     * - Filter health monitoring
     * - Sensor fault detection
     * - Tuning validation (should be zero-mean)
     * - Innovation consistency test calculation
     * - Measurement quality assessment
     * 
     * Returns false when:
     * - No measurements fused yet
     * - Filter not initialized
     * - Specific sensor not being used
     * 
     * Typical Magnitudes (well-tuned filter):
     * - Velocity: < 1 m/s
     * - Position: < 5 m
     * - Magnetometer: < 100 mGauss
     * - Airspeed: < 2 m/s
     * - Yaw: < 0.1 rad (< 6°)
     * 
     * @note Innovations updated each time measurement fused
     * @note Large persistent innovations indicate tuning issues
     * @note Used internally for consistency checks
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:196
     */
    bool getInnovations(Vector3f &velInnov, Vector3f &posInnov, Vector3f &magInnov, float &tasInnov, float &yawInnov) const;

    /**
     * @brief Get innovation consistency test ratios and variances
     * 
     * @param[out] velVar Velocity test ratio (dimensionless)
     * @param[out] posVar Position test ratio (dimensionless)
     * @param[out] hgtVar Height test ratio (dimensionless)
     * @param[out] magVar Magnetometer test ratios XYZ (dimensionless)
     * @param[out] tasVar True airspeed test ratio (dimensionless)
     * @param[out] offset Position offset for optical flow (m)
     * 
     * @return true if variances available, false otherwise
     * 
     * @details Returns innovation consistency test ratios used to detect
     *          measurement faults and assess filter health. Test ratio is
     *          innovation squared divided by innovation variance.
     * 
     * Test Ratio Definition:
     * - TestRatio = innovation² / innovation_variance
     * - Chi-squared distributed if filter correct
     * - Dimensionless (normalized innovation)
     * - > 1.0 indicates inconsistency
     * 
     * Test Ratio Interpretation:
     * - < 0.3: Excellent consistency, conservative tuning
     * - 0.3-1.0: Good consistency, well tuned
     * - 1.0-2.0: Marginal, approaching threshold
     * - > 2.0: Poor, measurement likely rejected
     * 
     * Consistency Test Usage:
     * - Compare ratio to threshold (typically 5.0-10.0)
     * - Reject measurements exceeding threshold
     * - Accumulated ratios indicate sustained problems
     * - Used for sensor timeout logic
     * 
     * Variance Components:
     * - velVar: Combined NED velocity test ratio
     * - posVar: Combined NE position test ratio
     * - hgtVar: Vertical position test ratio
     * - magVar: [X, Y, Z] magnetometer axis test ratios
     * - tasVar: Airspeed test ratio
     * 
     * Offset Parameter:
     * - offset: [N, E] position offset for optical flow terrain estimator
     * - Used for optical flow fusion health assessment
     * 
     * Use Cases:
     * - Sensor failure detection
     * - Automatic sensor switching
     * - Filter health reporting
     * - Tuning validation
     * - Lane selection weighting
     * 
     * Failure Indicators:
     * - Sustained high ratios: Sensor drift or failure
     * - Sudden spikes: Measurement outliers
     * - All ratios high: Filter divergence
     * 
     * @note Test ratios are primary health metric
     * @note Used for automatic measurement rejection
     * @note Critical for multi-sensor redundancy management
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:199
     */
    bool getVariances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar, Vector2f &offset) const;

    /**
     * @brief Check if compass should be used
     * 
     * @return true if compass being used for yaw, false otherwise
     * 
     * @details Indicates whether the magnetometer (compass) is currently being
     *          used by the EKF for yaw estimation. Public for AHRS reporting.
     * 
     * Compass Usage Conditions:
     * - Magnetometer data available and healthy
     * - Magnetic field learned or initializable
     * - Not inhibited by user parameters
     * - Innovation consistency checks passing
     * - Sufficient yaw observability
     * 
     * Compass NOT Used When:
     * - No magnetometer available
     * - Magnetometer failed health checks
     * - Magnetic interference detected
     * - Using GPS yaw (dual GPS heading)
     * - Using external yaw source
     * - Compass explicitly disabled in parameters
     * 
     * Yaw Alternative Sources:
     * - GPS course over ground (requires movement)
     * - GPS heading (dual GPS systems)
     * - External navigation system yaw
     * - GSF yaw estimator (emergency backup)
     * 
     * Use Cases:
     * - AHRS.use_compass() reporting to GCS
     * - Pre-flight compass health validation
     * - Sensor configuration diagnostics
     * - Determining yaw reference source
     * 
     * Implications:
     * - true: Yaw referenced to magnetic north
     * - false: Yaw from GPS or other source
     * - Affects heading accuracy and response
     * - Critical for navigation in GPS-denied environments
     * 
     * @note Made public for AHRS reporting interface
     * @note Compass use can change during flight (fallback logic)
     * @note Does not indicate compass health, only current usage
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:203
     */
    bool use_compass(void) const;

    /**
     * @brief Write raw optical flow measurements to EKF buffer
     * 
     * @param[in] rawFlowQuality Flow quality metric (0-255, 255 = best)
     * @param[in] rawFlowRates Flow rates about sensor X,Y axes (rad/s)
     * @param[in] rawGyroRates Sensor internal gyro rates X,Y (rad/s)
     * @param[in] msecFlowMeas Measurement timestamp (ms, scheduler time)
     * @param[in] posOffset Sensor position in body frame XYZ (m)
     * @param[in] heightOverride Fixed height override for rovers (m, 0 = not used)
     * 
     * @details Buffers optical flow sensor data for subsequent fusion into the EKF.
     *          Optical flow provides velocity measurements for GPS-denied navigation.
     * 
     * Optical Flow Measurement:
     * - Measures apparent motion of ground features
     * - Returns angular rates about sensor axes
     * - Requires known height above ground
     * - Velocity = flow_rate * height
     * 
     * Quality Metric:
     * - rawFlowQuality: 0 = no usable data, 255 = perfect
     * - Based on feature tracking confidence
     * - Sensor-specific interpretation
     * - Low quality measurements may be rejected
     * 
     * Flow Rate Convention:
     * - rawFlowRates[0]: X-axis flow (rad/s), forward motion produces positive
     * - rawFlowRates[1]: Y-axis flow (rad/s), rightward motion produces positive
     * - Right-hand rule: clockwise rotation = positive
     * - Sensor frame (may differ from body frame)
     * 
     * Gyro Compensation:
     * - rawGyroRates: Sensor's internal gyro measurement
     * - Used to compensate for vehicle rotation
     * - Flow from ground motion = raw flow - gyro rate
     * - Critical for accurate velocity estimation
     * 
     * Sensor Position:
     * - posOffset: [X, Y, Z] in body frame (m)
     * - X forward, Y right, Z down
     * - Accounts for lever arm effects
     * - Important for accurate velocity transformation
     * 
     * Height Override:
     * - heightOverride: Fixed height for ground vehicles (m)
     * - 0 = use EKF height estimate (flying vehicles)
     * - Non-zero = fixed terrain clearance (rovers)
     * - Rovers on flat ground can assume constant height
     * 
     * Fusion Process:
     * 1. Data buffered with timestamp
     * 2. Retrieved at fusion time horizon
     * 3. Compensated for vehicle rotation
     * 4. Converted to velocity using height
     * 5. Fused into EKF velocity states
     * 
     * Use Cases:
     * - GPS-denied position hold (multicopters)
     * - Indoor navigation
     * - Precision landing
     * - Velocity aiding when GPS unavailable
     * 
     * @note Flow quality threshold typically 50-150 for fusion
     * @note Requires valid height estimate (baro, rangefinder)
     * @note Critical for GPS-denied operations
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:213
     */
    void  writeOptFlowMeas(const uint8_t rawFlowQuality, const Vector2f &rawFlowRates, const Vector2f &rawGyroRates, const uint32_t msecFlowMeas, const Vector3f &posOffset, float heightOverride);

    /**
     * @brief Get range beacon fusion debug data
     * 
     * @param[out] ID Beacon identifier number
     * @param[out] rng Measured range to beacon (m)
     * @param[out] innov Range innovation (m)
     * @param[out] innovVar Innovation variance (m²)
     * @param[out] testRatio Innovation consistency test ratio (dimensionless)
     * @param[out] beaconPosNED Beacon position NED (m)
     * @param[out] offsetHigh High altitude offset estimate (m)
     * @param[out] offsetLow Low altitude offset estimate (m)
     * 
     * @return true if beacon data available, false otherwise
     * 
     * @details Returns detailed diagnostics for range beacon fusion, used for
     *          debugging beacon positioning system integration and performance.
     * 
     * Range Beacon System:
     * - Multiple fixed beacons at known positions
     * - Measure range from vehicle to each beacon
     * - Triangulation provides 3D position
     * - Alternative to GPS for indoor/constrained environments
     * 
     * Output Parameters:
     * - ID: Which beacon this data is for (0-9)
     * - rng: Raw range measurement from time-of-flight (m)
     * - innov: Predicted - measured range (m)
     * - innovVar: Expected innovation variance (m²)
     * - testRatio: innov² / innovVar (dimensionless)
     * - beaconPosNED: Beacon location in NED frame (m)
     * - offsetHigh/Low: Altitude offset hypotheses (m)
     * 
     * Innovation Interpretation:
     * - innov near zero: Good agreement prediction/measurement
     * - Large positive innov: Vehicle closer than predicted
     * - Large negative innov: Vehicle farther than predicted
     * - Systematic bias: Beacon position error or altitude offset
     * 
     * Test Ratio Interpretation:
     * - < 1.0: Consistent measurement, likely fused
     * - 1.0-5.0: Marginal consistency
     * - > 5.0: Inconsistent, likely rejected
     * 
     * Altitude Offset:
     * - offsetHigh/Low: Vertical offset beacon constellation to EKF origin
     * - Resolves ambiguity in beacon height datum
     * - Filter tests multiple hypotheses
     * - Converges to correct offset over time
     * 
     * Use Cases:
     * - Debugging beacon position configuration
     * - Validating range measurements
     * - Tuning beacon fusion parameters
     * - Troubleshooting position jumps
     * - Assessing beacon geometry quality
     * 
     * Common Issues Diagnosed:
     * - Incorrect beacon positions (large innovations)
     * - Altitude datum mismatch (offset errors)
     * - Beacon range errors (outliers)
     * - Poor beacon geometry (high test ratios)
     * 
     * @note Call repeatedly to get data for different beacons
     * @note Returns false if no beacon fusion active
     * @note Beacon ID cycles through available beacons
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:224
     */
    bool getRangeBeaconDebug(uint8_t &ID, float &rng, float &innov, float &innovVar, float &testRatio, Vector3f &beaconPosNED, float &offsetHigh, float &offsetLow);

    /**
     * @brief Set terrain height stability flag for range finder fusion
     * 
     * @param[in] val true if terrain stable, false if unstable
     * 
     * @details Controls whether range finder measurements are used as a height
     *          reference based on terrain characteristics. Prevents range finder
     *          use over unsuitable terrain.
     * 
     * Terrain Stability Assessment:
     * - Stable terrain: Flat, solid surface (concrete, asphalt, calm water)
     * - Unstable terrain: Vegetation, rough ground, waves, moving objects
     * - Vehicle code assesses terrain from vehicle type and mode
     * 
     * Effect on Range Finder Usage:
     * - val = true: Range finder can be used per EK2_RNG_AID_HGT parameter
     * - val = false: Range finder disabled regardless of parameters
     * - Overrides automatic range finder selection logic
     * 
     * Parameter Interaction:
     * - EK2_RNG_AID_HGT: Controls range finder height aiding mode
     * - EK2_RNG_USE_SPD: Speed threshold for range finder use
     * - setTerrainHgtStable(false) disables regardless of parameters
     * 
     * Use Cases:
     * - Disable range finder over vegetation (plane takeoff/landing)
     * - Disable over water with waves (boats, seaplanes)
     * - Enable over flat surfaces (precision landing pads)
     * - Mode-dependent terrain assessment (auto vs manual)
     * 
     * Common Usage Patterns:
     * - Copters: Usually true (stable terrain assumption)
     * - Planes: Set false during cruise, true during landing approach
     * - Rovers: Usually true (ground contact)
     * - Boats: Set based on wave conditions
     * 
     * Effects When Disabled:
     * - Range finder data not fused into height estimate
     * - Barometer becomes primary height source
     * - Terrain height estimator may still run (optical flow)
     * - Prevents erroneous height corrections from bad range data
     * 
     * Safety Considerations:
     * - Incorrect range fused over unstable terrain causes altitude errors
     * - Altitude errors can trigger failsafes or crashes
     * - Conservative: disable if terrain questionable
     * 
     * @note Call from vehicle code based on flight mode and terrain type
     * @note Does not affect optical flow terrain estimator
     * @note Can be changed in-flight based on conditions
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:230
     */
    void setTerrainHgtStable(bool val);

    /**
     * @brief Get filter fault status as bitmask
     * 
     * @param[out] faults Bitmask of fault conditions (see details)
     * 
     * @details Returns bitmask indicating severe filter faults that compromise
     *          navigation solution validity. Used for fault detection and failsafe logic.
     * 
     * Fault Bitmask Bits:
     * - Bit 0 (0x01): Quaternions are NaN (attitude invalid)
     * - Bit 1 (0x02): Velocities are NaN (velocity invalid)
     * - Bit 2 (0x04): X magnetometer fusion badly conditioned
     * - Bit 3 (0x08): Y magnetometer fusion badly conditioned
     * - Bit 4 (0x10): Z magnetometer fusion badly conditioned
     * - Bit 5 (0x20): Airspeed fusion badly conditioned
     * - Bit 6 (0x40): Synthetic sideslip fusion badly conditioned
     * - Bit 7 (0x80): Filter not initialized
     * 
     * NaN Faults (Bits 0-1):
     * - Indicates numerical instability or divergence
     * - Complete loss of attitude or velocity solution
     * - Requires filter reset or lane switch
     * - CRITICAL: vehicle cannot navigate
     * 
     * Badly Conditioned Fusion (Bits 2-6):
     * - Innovation variance calculation failed
     * - Near-zero variance (numerical conditioning issue)
     * - Measurement rejected to prevent covariance corruption
     * - Can indicate sensor failure or poor geometry
     * 
     * Not Initialized (Bit 7):
     * - Filter has not completed alignment
     * - States not yet set from measurements
     * - Normal during pre-flight or immediately after reset
     * - Should clear within seconds if sensors good
     * 
     * Fault Interpretation:
     * - faults = 0: No faults, filter healthy
     * - Any NaN bits set: CRITICAL, switch lanes
     * - Conditioning faults: Sensor issue, continue with caution
     * - Multiple faults: Likely filter divergence
     * 
     * Use Cases:
     * - EKF lane selection (prefer fault-free lanes)
     * - Triggering EKF failsafe
     * - Pre-flight health checks
     * - Post-flight log analysis
     * - Sensor failure detection
     * 
     * Common Fault Causes:
     * - NaN: Covariance corruption, extreme conditions
     * - Mag conditioning: Magnetic interference, poor calibration
     * - Airspeed: Pitot tube blockage, extreme winds
     * - Not initialized: Insufficient GPS, IMU calibration
     * 
     * @warning NaN faults require immediate filter reset or lane switch
     * @note Conditioning faults may be transient
     * @note Check faults regularly during flight
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:243
     */
    void  getFilterFaults(uint16_t &faults) const;

    /**
     * @brief Get GPS quality check status
     * 
     * @param[out] status Structure containing GPS check results
     * 
     * @details Returns detailed GPS quality assessment from pre-flight and
     *          in-flight checks. Used to determine GPS suitability for navigation.
     * 
     * GPS Quality Checks:
     * - Satellite count (nsats >= threshold)
     * - Horizontal dilution of precision (HDOP)
     * - Speed accuracy (sAcc)
     * - Position accuracy (hAcc, vAcc)
     * - Yaw accuracy (dual GPS heading)
     * - Horizontal position drift (stationary check)
     * - Vertical velocity consistency
     * - Horizontal velocity consistency
     * 
     * Check Categories:
     * - Pre-flight: Performed before arming
     * - In-flight: Continuous monitoring during flight
     * - Both use same thresholds but different logic
     * 
     * Status Structure (nav_gps_status):
     * - Boolean flags indicating which checks failed
     * - Each check has pass/fail status
     * - Used to gate GPS usage in EKF
     * 
     * Pre-flight GPS Checks:
     * - Must pass before allowing GPS-based initialization
     * - Ensure sufficient accuracy for takeoff
     * - Typical: 6+ satellites, HDOP < 2.5, hAcc < 5m
     * 
     * In-flight GPS Checks:
     * - Monitor GPS health during flight
     * - Detect GPS failures or degradation
     * - Trigger fallback to non-GPS navigation if failed
     * 
     * Check Failure Implications:
     * - bad_sats: Insufficient satellite count
     * - bad_hdop: Poor satellite geometry
     * - bad_hAcc/vAcc: Reported accuracy too poor
     * - bad_horiz_drift: Excessive position noise (stationary)
     * - bad_vert_vel/horiz_vel: Velocity consistency failed
     * 
     * Use Cases:
     * - Pre-arm GPS quality validation
     * - Preventing takeoff with bad GPS
     * - In-flight GPS health monitoring
     * - Triggering GPS failsafe
     * - Diagnostic reporting to GCS
     * 
     * Common Failure Reasons:
     * - Insufficient satellites (urban canyon, indoors)
     * - Poor satellite geometry (high HDOP)
     * - GPS receiver fault
     * - Antenna obstruction
     * - RF interference
     * 
     * @note Check before relying on GPS for navigation
     * @note Failures during pre-flight prevent arming
     * @note In-flight failures may trigger position hold or RTL
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:248
     */
    void  getFilterGpsStatus(nav_gps_status &status) const;

    /**
     * @brief Get comprehensive filter status
     * 
     * @param[out] status Structure containing filter state indicators
     * 
     * @details Returns comprehensive filter status including output validity,
     *          flight phase detection, and aiding source status. Primary interface
     *          for assessing EKF health and capability.
     * 
     * Status Indicators (nav_filter_status):
     * - Output validity flags (attitude, velocity, position)
     * - Flight phase flags (in_air, taking_off, ground_effect)
     * - Aiding source flags (GPS, optical_flow, compass, airspeed)
     * - Height source indicator (baro, GPS, rangefinder)
     * - Solution mode (3D, 2D, altitude_only)
     * 
     * Output Validity:
     * - attitude_valid: Attitude quaternion usable
     * - horiz_vel_valid: NE velocity valid for flight control
     * - vert_vel_valid: D velocity valid for flight control
     * - horiz_pos_valid: NE position valid for navigation
     * - vert_pos_valid: D position valid for altitude hold
     * 
     * Flight Phase Detection:
     * - in_air: Vehicle airborne (flying)
     * - taking_off: Takeoff transition in progress
     * - ground_effect: Hovering near ground (affects barometer)
     * - Used to adapt filter behavior to flight conditions
     * 
     * Aiding Sources Active:
     * - using_gps: GPS measurements being fused
     * - using_optflow: Optical flow being fused
     * - using_compass: Magnetometer being fused for yaw
     * - using_airspeed: Airspeed sensor being fused
     * 
     * Ground Effect Mode:
     * - Activated when hovering close to ground
     * - Mitigates barometer errors from rotor downwash
     * - Increases barometer measurement noise
     * - Reduces reliance on height for short period
     * 
     * Use Cases:
     * - Pre-flight: Check if filter ready for takeoff
     * - Flight control: Determine if position hold possible
     * - Failsafe logic: Assess navigation capability
     * - Mode switching: Verify prerequisites for GPS modes
     * - GCS reporting: Display EKF status to operator
     * 
     * Validity Interpretation:
     * - attitude_valid required for stabilize mode
     * - horiz_vel_valid required for loiter/position hold
     * - horiz_pos_valid required for auto/guided modes
     * - All valid: Full navigation capability
     * 
     * Common Status Patterns:
     * - Pre-arm: attitude valid, position invalid (not initialized)
     * - GPS-denied: attitude/velocity valid, position invalid
     * - Full navigation: All valid flags set
     * - Failsafe: Position invalid, attitude still valid
     * 
     * @note Must check before using EKF outputs for flight control
     * @note Output validity can change during flight (GPS loss)
     * @note Primary health check for mode switching logic
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:257
     */
    void  getFilterStatus(nav_filter_status &status) const;

    /**
     * @brief Send EKF status report to ground control station
     * 
     * @param[in] link MAVLink connection to send on
     * 
     * @details Sends EKF_STATUS_REPORT MAVLink message containing comprehensive
     *          filter health and performance metrics for ground station display.
     * 
     * Message Contents:
     * - Innovation test ratios for all sensors
     * - Output validity flags
     * - Fault status bitmask
     * - GPS quality metrics
     * - Terrain estimator status
     * - Filter solution mode
     * 
     * Reported Metrics:
     * - Velocity innovation test ratio
     * - Position innovation test ratio
     * - Height innovation test ratio
     * - Magnetometer innovation test ratios (X,Y,Z)
     * - Airspeed innovation test ratio
     * - Output tracking errors
     * 
     * Use Cases:
     * - Real-time EKF health monitoring in GCS
     * - Identifying sensor issues during flight
     * - Tuning validation (innovation ratios)
     * - Pre-flight health assessment
     * - Post-flight analysis preparation
     * 
     * Message Rate:
     * - Typically sent at 1-5 Hz to GCS
     * - Rate controlled by MAVLink stream configuration
     * - Lower rate than internal filter update (100 Hz)
     * 
     * @note Called by GCS_MAVLINK streaming system
     * @note Message format defined in MAVLink protocol
     * @note Lightweight summary, detailed logs in dataflash
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:260
     */
    void send_status_report(class GCS_MAVLINK &link) const;

    /**
     * @brief Get height control limit for optical flow navigation
     * 
     * @param[out] height Maximum altitude limit (m)
     * 
     * @return true if height limit active, false if no limit
     * 
     * @details Provides altitude ceiling when using optical flow navigation
     *          to prevent vehicle from flying too high for flow sensor to work.
     * 
     * Optical Flow Height Limitation:
     * - Optical flow requires visible ground features
     * - Effectiveness degrades with altitude (feature size)
     * - Maximum effective altitude typically 10-50m
     * - Height limit prevents flying beyond useful range
     * 
     * Limit Calculation:
     * - Based on optical flow maximum range parameter
     * - Considers sensor field of view
     * - May include safety margin
     * - Returns false if not using optical flow
     * 
     * Height Limit Enforcement:
     * - Control loops constrain altitude commands
     * - Prevents pilot commanding excessive altitude
     * - Gentle ceiling (soft limit, not hard stop)
     * - May allow brief excursions for safety
     * 
     * When Limit Active:
     * - Optical flow is primary position reference
     * - GPS unavailable or not being used
     * - Range finder providing height reference
     * - AID_RELATIVE mode (optical flow aiding)
     * 
     * When No Limit (returns false):
     * - GPS-based navigation (AID_ABSOLUTE mode)
     * - Not using optical flow for position
     * - Height from barometer or GPS
     * - No need to constrain altitude
     * 
     * Typical Limits:
     * - Indoor: 5-10m (ceiling height)
     * - Outdoor low altitude: 10-30m (flow effectiveness)
     * - Parameter configurable: EK2_MAX_FLOW_HGT or similar
     * 
     * Use Cases:
     * - GPS-denied indoor flight
     * - Precision hover over landing pad
     * - Preventing loss of optical flow lock
     * - Mode-specific altitude limits
     * 
     * Effects on Flight Modes:
     * - Alt Hold: Ceiling applied
     * - Loiter: Position and altitude constrained
     * - Auto: Waypoint altitudes clamped
     * - Guided: Command altitudes limited
     * 
     * @note Returns false when GPS navigation available
     * @note Limit only active during optical flow navigation
     * @note Control loops must respect this limit
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:265
     */
    bool getHeightControlLimit(float &height) const;

    /**
     * @brief Get last yaw angle reset magnitude and time
     * 
     * @param[out] yawAng Yaw angle change from reset (rad)
     * 
     * @return Timestamp of last reset (ms), or 0 if never reset
     * 
     * @details Returns the yaw angle change applied during the most recent
     *          in-flight yaw reset event. Used by controllers to compensate.
     * 
     * Yaw Reset Events:
     * - Magnetic anomaly detected (large innovation)
     * - GPS yaw initialization (dual GPS heading available)
     * - External navigation yaw reset
     * - GSF yaw estimator reset (emergency)
     * - Manual yaw reset request
     * 
     * Reset Magnitude:
     * - yawAng: Change in yaw angle (rad)
     * - Positive = yaw increased (turned right)
     * - Negative = yaw decreased (turned left)
     * - Typically < 180° (π rad), may be large if magnetic anomaly
     * 
     * Controller Compensation:
     * - Controllers track last reset time
     * - Apply opposite correction to heading targets
     * - Prevents sudden heading changes commanded to vehicle
     * - Gradual absorption of yaw reset
     * 
     * Yaw Reset Causes:
     * - Magnetic interference (large sudden change)
     * - Initial yaw alignment after takeoff
     * - Switching yaw reference source
     * - EKF lane switch with yaw difference
     * - Manual reset command from vehicle code
     * 
     * Use Cases:
     * - Attitude controller yaw target adjustment
     * - Navigation controller heading compensation
     * - GCS notification of yaw discontinuity
     * - Post-flight analysis of yaw events
     * - Validating yaw stability
     * 
     * Reset Frequency:
     * - Normal operation: 0-2 per flight (initial alignment)
     * - Magnetic anomaly: May reset multiple times
     * - Excessive resets indicate problem (interference, tuning)
     * 
     * Time Interpretation:
     * - Returns 0 if no reset since filter initialization
     * - Returns scheduler time_ms of most recent reset
     * - Controllers compare to current time
     * - Only compensate recent resets (< few seconds)
     * 
     * @note Controllers must poll regularly to detect resets
     * @note Large unexpected resets may indicate sensor failure
     * @note Resets are normal during alignment phase
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:269
     */
    uint32_t getLastYawResetAngle(float &yawAng) const;

    /**
     * @brief Get last North-East position reset magnitude and time
     * 
     * @param[out] pos NE position change vector (m)
     * 
     * @return Timestamp of last reset (ms), or 0 if never reset
     * 
     * @details Returns horizontal position change from most recent position
     *          reset event. Used by controllers and navigation to compensate.
     * 
     * Position Reset Events:
     * - GPS acquisition after GPS-denied navigation
     * - Large GPS position innovation (GPS glitch recovery)
     * - External navigation position reset
     * - EKF lane switch with position difference
     * - Origin reset or relocation
     * 
     * Reset Vector:
     * - pos.x: North position change (m)
     * - pos.y: East position change (m)
     * - Positive N = moved north, positive E = moved east
     * - Magnitude can be large (meters to kilometers)
     * 
     * Common Reset Scenarios:
     * - GPS reacquisition: Optical flow drift correction
     * - GPS glitch: Reverting bad position jump
     * - External nav: Vision system position update
     * - Origin change: Coordinate frame shift
     * 
     * Controller Compensation:
     * - Position controller adjusts targets by reset amount
     * - Prevents commanding sudden velocity to absorb reset
     * - Waypoint targets shifted to maintain relative position
     * - Gradual blending to new position reference
     * 
     * Use Cases:
     * - Position controller target adjustment
     * - Waypoint mission compensation
     * - Preventing position hold oscillation after reset
     * - GCS notification of position discontinuity
     * - Validating position stability
     * 
     * Reset Magnitude Interpretation:
     * - < 1m: Minor correction, normal
     * - 1-10m: Moderate, possibly optical flow drift
     * - > 10m: Large reset, GPS glitch or long GPS-denied period
     * - > 100m: Very large, investigate cause
     * 
     * Time Interpretation:
     * - Returns 0 if no reset since filter initialization
     * - Returns scheduler time_ms of most recent reset
     * - Controllers poll regularly and check timestamp
     * - Compensation applied for recent resets only
     * 
     * Reset Frequency:
     * - Normal: 0-1 per flight (GPS reacquisition)
     * - GPS-denied operations: 1-2 (entering/exiting)
     * - Frequent resets indicate problem (GPS quality, tuning)
     * 
     * @note Large resets may cause momentary position hold overshoot
     * @note Controllers should limit reset compensation rate
     * @note Frequent resets indicate GPS or filter issues
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:273
     */
    uint32_t getLastPosNorthEastReset(Vector2f &pos) const;

    /**
     * @brief Get last Down position reset magnitude and time
     * 
     * @param[out] posD Down position change (m)
     * 
     * @return Timestamp of last reset (ms), or 0 if never reset
     * 
     * @details Returns vertical position change from most recent height
     *          reset event. Used by altitude controller to compensate.
     * 
     * Height Reset Events:
     * - Switching height reference source (baro ↔ GPS ↔ rangefinder)
     * - Large height innovation (barometer glitch recovery)
     * - Ground level reset (zeroing height datum)
     * - EKF lane switch with altitude difference
     * - Origin height correction
     * 
     * Reset Value:
     * - posD: Down position change (m)
     * - Positive = moved down (altitude decreased)
     * - Negative = moved up (altitude increased)
     * - NED frame convention (down is positive)
     * 
     * Common Reset Scenarios:
     * - Height source switch: Aligning to new reference
     * - Ground effect: Barometer correction after takeoff
     * - Baro glitch: Reverting erroneous altitude change
     * - Range finder transition: Ground to baro altitude
     * 
     * Controller Compensation:
     * - Altitude controller adjusts altitude target
     * - Prevents commanding large climb/descent rate
     * - Smooth absorption of altitude discontinuity
     * - Rate-limited to avoid instability
     * 
     * Use Cases:
     * - Altitude hold target adjustment
     * - Climb rate controller compensation
     * - Terrain following altitude correction
     * - GCS altitude display adjustment
     * - Validating altitude stability
     * 
     * Reset Magnitude Interpretation:
     * - < 0.5m: Minor correction, normal
     * - 0.5-2m: Moderate, height source alignment
     * - 2-5m: Large, investigate barometer or GPS
     * - > 5m: Very large, likely sensor fault
     * 
     * Height Source Switching:
     * - Baro → GPS: May see several meter correction
     * - GPS → Rangefinder: Terrain following engagement
     * - Rangefinder → Baro: Leaving terrain following
     * - Each switch may cause reset
     * 
     * Time Interpretation:
     * - Returns 0 if no reset since initialization
     * - Returns scheduler time_ms of most recent reset
     * - Altitude controller polls and compensates
     * - Short-term compensation only (< 5 seconds)
     * 
     * Reset Frequency:
     * - Normal: 1-3 per flight (takeoff, landing, source switches)
     * - Frequent resets indicate barometer issue
     * - Ground effect causes one reset after takeoff
     * 
     * @note NED frame: positive posD = altitude decreased
     * @note Large resets may cause momentary altitude hold overshoot
     * @note Height resets normal during takeoff/landing transitions
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:277
     */
    uint32_t getLastPosDownReset(float &posD) const;

    /**
     * @brief Get last North-East velocity reset magnitude and time
     * 
     * @param[out] vel NE velocity change vector (m/s)
     * 
     * @return Timestamp of last reset (ms), or 0 if never reset
     * 
     * @details Returns horizontal velocity change from most recent velocity
     *          reset event. Used by controllers to prevent sudden response.
     * 
     * Velocity Reset Events:
     * - GPS reacquisition after GPS-denied period
     * - Large velocity innovation (GPS glitch)
     * - Switching velocity reference (GPS ↔ optical flow)
     * - External navigation velocity reset
     * - EKF lane switch with velocity difference
     * 
     * Reset Vector:
     * - vel.x: North velocity change (m/s)
     * - vel.y: East velocity change (m/s)
     * - Positive N/E = velocity increased in that direction
     * - Can be positive or negative
     * 
     * Common Reset Scenarios:
     * - GPS reacquisition: Optical flow velocity drift
     * - GPS glitch recovery: Reverting bad velocity
     * - Optical flow to GPS: Drift correction
     * - Zero velocity update: Confirming stationary state
     * 
     * Controller Compensation:
     * - Velocity controller adjusts velocity targets
     * - Rate controller compensates for velocity jump
     * - Prevents sudden acceleration commands
     * - Smooth blending to new velocity reference
     * 
     * Use Cases:
     * - Velocity controller feed-forward adjustment
     * - Rate controller compensation
     * - Position controller velocity target correction
     * - Preventing oscillation after velocity reset
     * - Validating velocity estimate stability
     * 
     * Reset Magnitude Interpretation:
     * - < 0.5 m/s: Minor correction, normal
     * - 0.5-2 m/s: Moderate, optical flow drift
     * - 2-5 m/s: Large, significant drift or glitch
     * - > 5 m/s: Very large, investigate sensor issues
     * 
     * Time Interpretation:
     * - Returns 0 if no reset since initialization
     * - Returns scheduler time_ms of most recent reset
     * - Controllers poll and check timestamp
     * - Compensation decays over few seconds
     * 
     * Reset Frequency:
     * - Normal: 0-2 per flight (GPS loss/reacquisition)
     * - GPS-denied: More frequent (optical flow drift)
     * - Excessive resets indicate sensor or tuning issue
     * 
     * Velocity Drift:
     * - Optical flow accumulates velocity drift over time
     * - GPS reacquisition reveals accumulated error
     * - Reset magnitude indicates drift magnitude
     * - Used to assess optical flow performance
     * 
     * @note Large velocity resets cause momentary control disturbance
     * @note Controllers should rate-limit compensation
     * @note Frequent resets indicate poor velocity estimation
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:281
     */
    uint32_t getLastVelNorthEastReset(Vector2f &vel) const;

    /**
     * @brief Get pre-arm failure reason string
     * 
     * @return C-string describing why EKF refusing to initialize, or NULL if OK
     * 
     * @details Returns human-readable explanation of why this EKF core is
     *          refusing to initialize, preventing vehicle arming.
     * 
     * Common Failure Reasons:
     * - "Waiting for GPS" - Insufficient GPS quality
     * - "Waiting for gyro cal" - Gyro bias not converged
     * - "Waiting for ACC cal" - Accelerometer bias not stable
     * - "Waiting for compass" - Magnetometer not healthy
     * - "GPS speed accuracy" - GPS sAcc too high
     * - "GPS position accuracy" - GPS hAcc too high
     * - "GPS vertical velocity" - GPS vertical velocity inconsistent
     * - "GPS horizontal velocity" - GPS horizontal velocity excessive
     * - "GPS position drift" - Excessive position noise while stationary
     * - "GPS HDOP" - Horizontal dilution of precision too high
     * - "Compass not calibrated" - Compass offsets not learned
     * 
     * GPS Quality Requirements:
     * - Minimum satellites (typically 6)
     * - Maximum HDOP (typically 2.5)
     * - Position accuracy threshold (hAcc < 5m)
     * - Speed accuracy threshold (sAcc < 1.5 m/s)
     * - Velocity consistency checks
     * 
     * IMU Requirements:
     * - Gyro bias learned and stable
     * - Accelerometer bias within limits
     * - No excessive vibration
     * - IMU temperature stable
     * 
     * Magnetometer Requirements:
     * - Compass healthy (not timing out)
     * - Compass calibrated (offsets learned)
     * - Magnetic field reasonable magnitude
     * - Low magnetic interference
     * 
     * Use Cases:
     * - Pre-arm check reporting to GCS
     * - Displaying reason to pilot
     * - Troubleshooting initialization failures
     * - Determining which sensor to address
     * - Pre-flight validation
     * 
     * Return Values:
     * - NULL: EKF ready to initialize, no issues
     * - Non-NULL: Points to static string with reason
     * - String buffer max 40 characters
     * - String valid until next call to this function
     * 
     * Troubleshooting:
     * - GPS failures: Wait, move to clear sky view
     * - Gyro cal: Keep vehicle stationary longer
     * - Compass: Perform compass calibration
     * - Accelerometer: Check vibration levels
     * 
     * Multi-Core Behavior:
     * - Each core may have different failure reason
     * - Vehicle arms if ANY core is ready
     * - Check all cores to diagnose systemic issues
     * 
     * @note String is static buffer, contents may change
     * @note NULL indicates core ready to initialize
     * @note Used for pre-arm check reporting
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:284
     */
    const char *prearm_failure_reason(void) const;

    /**
     * @brief Get frames elapsed since last prediction step
     * 
     * @return Number of frames since last prediction (0-255)
     * 
     * @details Returns count of frames lapsed since this core last performed
     *          a state prediction step. Used for computational load balancing.
     * 
     * Frame Definition:
     * - Frame = One scheduler loop iteration
     * - Typically 400 Hz or 1000 Hz depending on vehicle
     * - EKF prediction target 100 Hz (every 4-10 frames)
     * - Counter increments each frame, resets on prediction
     * 
     * Load Balancing Purpose:
     * - Multiple EKF cores run in parallel
     * - Prediction step is computationally expensive
     * - Spread predictions across different frames
     * - Prevents multiple cores predicting same frame
     * - Distributes CPU load over time
     * 
     * Scheduling Algorithm:
     * - Core with largest framesSincePredict goes next
     * - Ensures fair distribution of computation
     * - Maintains target prediction rate for all cores
     * - Prevents starvation of any core
     * 
     * Typical Values:
     * - 0: Just performed prediction this frame
     * - 1-9: Normal, waiting for next prediction interval
     * - > 10: Overdue for prediction (CPU overload)
     * - > 20: Severe delay, may indicate CPU saturation
     * 
     * Use Cases:
     * - Frontend scheduler selecting which core updates
     * - Load balancing across multiple EKF instances
     * - Detecting CPU overload conditions
     * - Performance monitoring and optimization
     * - Diagnosing EKF timing issues
     * 
     * Multi-Core Architecture:
     * - 2-3 cores typically run (redundancy)
     * - Each core has independent counter
     * - Frontend polls all cores
     * - Selects core with highest count
     * - That core performs next prediction
     * 
     * Performance Implications:
     * - High values indicate insufficient CPU
     * - Prediction delays affect filter accuracy
     * - May need to reduce sensor fusion rate
     * - Could indicate need for faster processor
     * 
     * @note Used by NavEKF2 frontend for core scheduling
     * @note Part of computational load management system
     * @note Does not indicate filter health, only scheduling
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:287
     */
    uint8_t getFramesSincePredict(void) const;

    /**
     * @brief Get active IMU index for this core
     * 
     * @return Gyro index currently being used (0-2)
     * 
     * @details Returns the gyro sensor index this EKF core is currently using.
     *          Gyro index returned as it's most critical for other subsystems.
     * 
     * IMU Selection:
     * - Each EKF core can use different IMU
     * - Provides sensor redundancy and fault tolerance
     * - Core may switch IMUs if primary fails
     * - gyro_index_active updated on IMU selection
     * 
     * Multi-IMU Architecture:
     * - Modern autopilots have 2-3 IMUs
     * - EKF cores distributed across IMUs
     * - Core 0 typically uses IMU 0
     * - Core 1 typically uses IMU 1
     * - Provides cross-checking and redundancy
     * 
     * Use Cases:
     * - Identifying which IMU a core is using
     * - Associating EKF lanes with sensors
     * - IMU failure diagnosis
     * - Logging and telemetry
     * - Cross-validation between cores
     * 
     * @note Returns gyro index, not accelerometer index
     * @note May differ from configured preferred IMU
     * @note Can change during flight if IMU fails
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:292
     */
    uint8_t getIMUIndex(void) const { return gyro_index_active; }

    /**
     * @brief Write external navigation position and attitude data
     * 
     * @param[in] pos Position in NED navigation frame (m)
     * @param[in] quat Quaternion navigation to body frame rotation
     * @param[in] posErr 1-sigma spherical position uncertainty (m)
     * @param[in] angErr 1-sigma spherical angle uncertainty (rad)
     * @param[in] timeStamp_ms Measurement time (ms, not receipt time)
     * @param[in] delay_ms Average delay of ext nav relative to IMU (ms)
     * @param[in] resetTime_ms Time of last position reset from ext nav (ms)
     * 
     * @details Buffers position and attitude data from an external navigation
     *          system (e.g., visual odometry, motion capture) for EKF fusion.
     * 
     * External Navigation Sources:
     * - Visual odometry (camera-based SLAM)
     * - Motion capture systems (Vicon, OptiTrack)
     * - Visual-inertial odometry (VIO)
     * - Intel RealSense T265 tracking camera
     * - External position estimators
     * 
     * Coordinate Frame:
     * - pos: [North, East, Down] in navigation frame (m)
     * - Right-handed NED convention
     * - Origin at EKF origin or system-specific
     * - Consistent with EKF navigation frame
     * 
     * Quaternion Convention:
     * - quat: Rotation from navigation frame to body frame
     * - Hamilton convention: [w, x, y, z]
     * - Passive rotation (transforms vectors nav → body)
     * - Must be normalized
     * 
     * Uncertainty Parameters:
     * - posErr: 1-sigma position uncertainty, spherical (m)
     * - angErr: 1-sigma attitude uncertainty, spherical (rad)
     * - Used to weight measurements in fusion
     * - Larger uncertainty = less weight in filter
     * 
     * Timing Parameters:
     * - timeStamp_ms: When measurement was taken (not received)
     * - delay_ms: Average processing/transmission delay
     * - resetTime_ms: Last reset event from external system
     * - Critical for time-alignment with IMU
     * 
     * Sensor Offset:
     * - Body frame offset from IMU to sensor
     * - Automatically retrieved from AP_VisualOdom library
     * - Accounts for lever arm effects
     * - Important for accurate velocity transformation
     * 
     * Fusion Process:
     * 1. Data buffered with timestamp
     * 2. Delayed to match IMU fusion horizon
     * 3. Position and quaternion fused independently
     * 4. May be used as yaw observation
     * 5. Can replace GPS as position reference
     * 
     * Reset Handling:
     * - resetTime_ms indicates external system reset
     * - EKF detects and handles position discontinuities
     * - Prevents large position jumps
     * - Coordinates reset compensation
     * 
     * Use Cases:
     * - GPS-denied indoor navigation
     * - Precision positioning (motion capture)
     * - Visual-inertial navigation
     * - Sensor fusion with external estimator
     * 
     * Requirements:
     * - Consistent coordinate frame
     * - Accurate timing information
     * - Reasonable update rate (> 10 Hz)
     * - Known sensor offset
     * 
     * @note Position and attitude can be fused independently
     * @note May be used as primary position reference
     * @note Requires accurate time synchronization
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:308
     */
    void writeExtNavData(const Vector3f &pos, const Quaternion &quat, float posErr, float angErr, uint32_t timeStamp_ms, uint16_t delay_ms, uint32_t resetTime_ms);

    /**
     * @brief Write external navigation velocity data
     * 
     * @param[in] vel Velocity in NED frame (m/s)
     * @param[in] err 1-sigma velocity uncertainty (m/s)
     * @param[in] timeStamp_ms Measurement time (ms, not receipt time)
     * @param[in] delay_ms Average delay relative to IMU (ms)
     * 
     * @details Buffers velocity data from external navigation system for
     *          EKF fusion. Provides velocity aiding independent of position.
     * 
     * Velocity Measurement:
     * - vel: [North, East, Down] velocity in NED frame (m/s)
     * - Earth-referenced velocity (not ground speed)
     * - Consistent with EKF velocity states
     * - Positive down convention
     * 
     * External Velocity Sources:
     * - Visual odometry velocity estimation
     * - Motion capture system derivatives
     * - Optical flow with external height
     * - Doppler radar velocity
     * - External estimator velocity output
     * 
     * Uncertainty:
     * - err: 1-sigma velocity uncertainty (m/s)
     * - Spherical uncertainty (same all directions)
     * - Used to weight measurement in fusion
     * - Typical values: 0.1-1.0 m/s
     * 
     * Timing:
     * - timeStamp_ms: When velocity measured (not received)
     * - delay_ms: Processing and transmission delay
     * - Critical for proper time-alignment
     * - Must account for external system latency
     * 
     * Fusion Benefits:
     * - Improves velocity estimation accuracy
     * - Reduces velocity drift in GPS-denied
     * - Faster velocity convergence
     * - Better dynamic response
     * - Complements position measurements
     * 
     * Independent of Position:
     * - Can fuse velocity without position
     * - Useful when position reference unreliable
     * - Constrains velocity drift
     * - Does not affect position directly
     * 
     * Use Cases:
     * - Visual odometry velocity aiding
     * - Motion capture velocity fusion
     * - GPS-denied velocity estimation
     * - Complementing position measurements
     * 
     * Requirements:
     * - NED frame velocity
     * - Accurate timestamps
     * - Reasonable uncertainty estimates
     * - Update rate > 10 Hz desirable
     * 
     * @note Velocity fused independently of position
     * @note Requires accurate timing for proper fusion
     * @note Complements but doesn't require position data
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:317
     */
    void writeExtNavVelData(const Vector3f &vel, float err, uint32_t timeStamp_ms, uint16_t delay_ms);

    /**
     * @brief Check if external nav providing yaw observations
     * 
     * @return true if external nav yaw being fused, false otherwise
     * 
     * @details Indicates whether external navigation system attitude is
     *          being used as a yaw observation in addition to position.
     * 
     * Yaw Observation Modes:
     * - Position only: External nav provides position, not yaw
     * - Position + Yaw: External nav provides both
     * - This function returns true only for position + yaw mode
     * 
     * When External Yaw Used:
     * - Visual odometry with good yaw observability
     * - Motion capture system providing orientation
     * - VIO system with reliable yaw estimate
     * - External nav yaw quality sufficient
     * 
     * When External Yaw NOT Used:
     * - External nav position-only mode
     * - External nav yaw uncertainty too high
     * - Magnetometer preferred for yaw
     * - Visual odometry yaw drifting
     * 
     * Yaw Source Selection:
     * - Magnetometer: Magnetic north reference
     * - GPS: Course over ground when moving
     * - External nav: Alternative yaw reference
     * - GSF: Emergency yaw estimator
     * 
     * Implications:
     * - true: Yaw from external system, not compass
     * - Compass may be disabled or low weight
     * - Yaw referenced to external nav frame
     * - Less susceptible to magnetic interference
     * 
     * Use Cases:
     * - Determining active yaw reference
     * - Validating external nav fusion mode
     * - Diagnostics and logging
     * - Compass health assessment
     * 
     * Visual Odometry Yaw:
     * - Scale ambiguity in monocular VIO
     * - Yaw drift in feature-poor environments
     * - May not always be reliable
     * - EKF assesses quality before using
     * 
     * @note External nav may provide position without yaw
     * @note Yaw usage independent of position usage
     * @note Determined by external nav quality and parameters
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:320
     */
    bool isExtNavUsedForYaw(void) const;

    /**
     * @brief Set default airspeed for forward flight
     * 
     * @param[in] airspeed Default equivalent airspeed (m/s)
     * 
     * @details Sets default airspeed to use when airspeed measurement
     *          required but not available. Used as fallback for planes.
     * 
     * Default Airspeed Usage:
     * - Used when airspeed sensor unavailable/failed
     * - Provides approximate airspeed for EKF
     * - Enables EKF to continue functioning
     * - Prevents complete loss of airspeed information
     * 
     * When Applied:
     * - Airspeed sensor not installed
     * - Airspeed sensor failed health checks
     * - Airspeed sensor disconnected
     * - Sensor reading obviously wrong
     * - Only in forward flight (planes)
     * 
     * Airspeed Parameter:
     * - airspeed: Equivalent airspeed (EAS) in m/s
     * - Should be typical cruise airspeed
     * - Positive value required for use
     * - Zero or negative disables default airspeed
     * 
     * Typical Values:
     * - Small plane: 15-20 m/s (30-40 kt)
     * - Medium plane: 20-30 m/s (40-60 kt)
     * - Large plane: 25-40 m/s (50-80 kt)
     * - Set to expected cruise speed
     * 
     * EKF Airspeed Fusion:
     * - Airspeed aids in estimating wind
     * - Improves velocity estimation in wind
     * - Helps with synthetic sideslip (beta) fusion
     * - Critical for accurate navigation in wind
     * 
     * Default vs Measured:
     * - Default: Constant, doesn't adapt to conditions
     * - Measured: Varies with speed, wind, altitude
     * - Default less accurate but better than nothing
     * - Performance degraded vs. measured airspeed
     * 
     * Use Cases:
     * - Backup for failed airspeed sensor
     * - Aircraft without airspeed sensor
     * - Testing/simulation with no airspeed
     * - Graceful degradation on sensor failure
     * 
     * Limitations:
     * - Doesn't account for speed variations
     * - Wind estimation less accurate
     * - Sideslip fusion less effective
     * - Position accuracy reduced in wind
     * 
     * Not Used For:
     * - Multicopters (don't need airspeed)
     * - Ground vehicles
     * - Hovering flight
     * - Vertical flight
     * 
     * @note Only used when actual airspeed unavailable
     * @note Must be positive to be used (0 = disabled)
     * @note Set to typical cruise speed for best results
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:323
     */
    void writeDefaultAirSpeed(float airspeed);

    /**
     * @brief Request yaw reset to GSF estimate
     * 
     * @details Requests that the main EKF reset its yaw to the estimate
     *          provided by the Gaussian Sum Filter (GSF) yaw estimator.
     * 
     * GSF Yaw Estimator:
     * - Gaussian Sum Filter = Multiple parallel EKF hypothesis
     * - Runs continuously in background
     * - Uses IMU + GPS velocity (+ optional airspeed)
     * - Does NOT use magnetometer
     * - Resolves yaw ambiguity through GPS heading
     * 
     * When to Request Reset:
     * - Detected magnetic anomaly (large compass error)
     * - Compass complete failure
     * - Inconsistent magnetometer readings
     * - Manual reset commanded by vehicle code
     * - Emergency yaw recovery
     * 
     * Reset Request Process:
     * - Request made via this function
     * - Request remains active for YAW_RESET_TO_GSF_TIMEOUT_MS
     * - Main filter checks GSF estimate validity
     * - If valid, performs yaw reset
     * - If timeout, request expires without reset
     * 
     * GSF Estimate Validity:
     * - GSF must be initialized (vehicle moved sufficiently)
     * - GPS velocity quality adequate
     * - Multiple hypotheses converged
     * - Estimate uncertainty below threshold
     * - Recent GPS updates available
     * 
     * Yaw Reset Execution:
     * - Main EKF quaternion reset to GSF yaw
     * - Roll/pitch preserved from main filter
     * - Yaw covariance reset appropriately
     * - Magnetic field states may be reset
     * - Reset logged and reported
     * 
     * Emergency Backup:
     * - GSF provides yaw when magnetometer fails
     * - Critical for continued safe navigation
     * - Requires vehicle movement for initialization
     * - Cannot initialize while stationary
     * 
     * Use Cases:
     * - Magnetic anomaly detected in flight
     * - Compass failure recovery
     * - Flying through magnetic disturbance
     * - Manual yaw reset command
     * - Testing yaw estimation robustness
     * 
     * Limitations:
     * - Requires GPS and vehicle motion
     * - Cannot work stationary or GPS-denied
     * - Slower convergence than magnetometer
     * - Less accurate than good compass
     * 
     * Request Timeout:
     * - Request active for YAW_RESET_TO_GSF_TIMEOUT_MS (5000 ms)
     * - Prevents stale requests
     * - Allows time for GSF to converge
     * - Expires if conditions not met
     * 
     * @warning Requires GPS and vehicle movement to work
     * @note Does not use magnetometer (magnetic anomaly recovery)
     * @note Request may not execute if GSF not ready
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:326
     */
    void EKFGSF_requestYawReset();

    /**
     * @brief Check if tilt alignment complete
     * 
     * @return true if roll/pitch aligned, false if still aligning
     * 
     * @details Indicates whether initial tilt (roll/pitch) alignment has
     *          completed. First step of EKF initialization process.
     * 
     * Tilt Alignment Process:
     * - Uses accelerometer gravity vector
     * - Estimates roll and pitch angles
     * - Requires vehicle stationary
     * - Typically completes in 1-3 seconds
     * - Prerequisite for yaw alignment
     * 
     * Alignment Requirements:
     * - Vehicle must be stationary
     * - Accelerometer measuring only gravity
     * - Low vibration for clean measurement
     * - Gyro bias relatively stable
     * - No external accelerations
     * 
     * What Gets Aligned:
     * - Roll angle (bank)
     * - Pitch angle (elevation)
     * - Attitude covariance initialized
     * - Does NOT include yaw (heading)
     * 
     * Alignment Quality:
     * - Good: Vehicle level, no vibration
     * - Poor: Tilted surface, high vibration
     * - Affects initial attitude accuracy
     * - Critical for subsequent yaw alignment
     * 
     * Use Cases:
     * - Pre-flight initialization status
     * - Determining if ready for yaw alignment
     * - Diagnosing slow initialization
     * - Pre-arm check conditions
     * - Logging initialization progress
     * 
     * Initialization Sequence:
     * 1. Tilt alignment (this flag)
     * 2. Yaw alignment (see have_aligned_yaw)
     * 3. Position/velocity initialization
     * 4. Full filter operation
     * 
     * Common Issues:
     * - false: Vehicle moving or high vibration
     * - Prolonged false: Accelerometer fault
     * - Quick true: Good conditions, clean sensors
     * 
     * @note Must be true before yaw alignment can start
     * @note Requires stationary vehicle
     * @note Typically completes in 1-3 seconds
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:329
     */
    bool have_aligned_tilt(void) const {
        return tiltAlignComplete;
    }

    /**
     * @brief Check if yaw alignment complete
     * 
     * @return true if yaw aligned, false if still aligning
     * 
     * @details Indicates whether initial yaw (heading) alignment has
     *          completed. Second step of EKF initialization after tilt.
     * 
     * Yaw Alignment Process:
     * - Uses magnetometer OR GPS velocity
     * - Establishes heading reference
     * - Requires tilt alignment first
     * - May take several seconds
     * - Necessary for full navigation
     * 
     * Yaw Alignment Methods:
     * - Magnetometer: Immediate if calibrated and healthy
     * - GPS velocity: Requires movement, uses GPS course
     * - External nav: Immediate if providing yaw
     * - GSF: Background estimator, slower
     * 
     * Magnetometer Alignment:
     * - Fastest method (< 1 second)
     * - Requires calibrated compass
     * - Uses earth magnetic field
     * - Affected by local magnetic interference
     * - Preferred when available
     * 
     * GPS Velocity Alignment:
     * - Fallback if no magnetometer
     * - Requires vehicle movement (> 2-3 m/s)
     * - Uses GPS course over ground
     * - Takes longer (5-10 seconds of motion)
     * - Less accurate than good magnetometer
     * 
     * Alignment Quality Factors:
     * - Good compass calibration
     * - Low magnetic interference
     * - Sufficient GPS velocity (if using GPS)
     * - Time allowed for convergence
     * - Stable attitude (tilt aligned)
     * 
     * Use Cases:
     * - Pre-flight initialization status
     * - Determining if ready for GPS navigation
     * - Pre-arm check conditions
     * - Logging initialization progress
     * - Troubleshooting heading issues
     * 
     * Initialization Sequence:
     * 1. Tilt alignment (roll/pitch)
     * 2. Yaw alignment (heading) ← this flag
     * 3. Position/velocity initialization
     * 4. Full navigation ready
     * 
     * Common Issues:
     * - false: No magnetometer and stationary (can't align)
     * - false: Poor compass calibration
     * - false: High magnetic interference
     * - Prolonged false: Compass failure or no movement
     * 
     * Flight Implications:
     * - false: GPS modes unavailable
     * - false: May prevent arming
     * - true: Full navigation capability
     * - true: GPS modes available
     * 
     * @note Requires tilt alignment complete first
     * @note May require vehicle movement if no compass
     * @note Critical for GPS-based navigation modes
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:334
     */
    bool have_aligned_yaw(void) const {
        return yawAlignComplete;
    }

    /**
     * @brief Write EKF data to dataflash log
     * 
     * @param[in] time_us Current time in microseconds
     * 
     * @details Logs comprehensive EKF state, covariance, and diagnostic data
     *          to dataflash for post-flight analysis and debugging.
     * 
     * Log Messages Written:
     * - NKF1: State vector and velocity innovations
     * - NKF2: Position, magnetometer innovations
     * - NKF3: Gyro bias, magnetic field estimates
     * - NKF4: Airspeed, height source, terrain state
     * - NKF5: Innovation test ratios
     * - Quaternion: Attitude quaternion
     * - Beacon: Range beacon fusion data (if active)
     * - Timing: Performance timing statistics
     * - GSF: Yaw estimator data
     * 
     * NKF1 Message Contents:
     * - Attitude quaternion states
     * - Velocity NED states
     * - Velocity innovations
     * - Innovation test ratios
     * - Core index and IMU index
     * 
     * NKF2 Message Contents:
     * - Position NED states
     * - Position innovations
     * - Magnetometer innovations
     * - Height measurement and innovation
     * - GPS quality metrics
     * 
     * NKF3 Message Contents:
     * - Gyro bias estimates X,Y,Z
     * - Gyro scale factors
     * - Accelerometer Z bias
     * - Earth magnetic field NED
     * - Body magnetic field XYZ
     * 
     * NKF4 Message Contents:
     * - Airspeed innovation and test ratio
     * - Height source indicator
     * - Terrain state estimate
     * - Terrain state variance
     * - Optical flow quality
     * 
     * NKF5 Message Contents:
     * - Normalized innovation tests
     * - Velocity, position, height test ratios
     * - Magnetometer test ratios X,Y,Z
     * - Airspeed test ratio
     * 
     * Timing Message:
     * - Prediction timing statistics
     * - Fusion step timings
     * - CPU load metrics
     * - Helps diagnose performance issues
     * 
     * GSF Message:
     * - Yaw estimator states
     * - Hypothesis weights
     * - Yaw estimate and variance
     * - Used for yaw recovery debugging
     * 
     * Logging Rate:
     * - Called at EKF update rate (typically 10-50 Hz)
     * - Rate controlled by LOG_RATE parameter
     * - Messages decimated to reduce log size
     * - Critical data logged more frequently
     * 
     * Use Cases:
     * - Post-flight analysis
     * - Diagnosing filter issues
     * - Validating tuning
     * - Troubleshooting sensor problems
     * - Performance optimization
     * 
     * Log Analysis:
     * - Plot innovations to check tuning
     * - Monitor test ratios for sensor health
     * - Track bias estimates over time
     * - Identify filter divergence
     * - Validate magnetic field learning
     * 
     * @note Called regularly during EKF operation
     * @note Multiple messages per call
     * @note Essential for post-flight debugging
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:338
     */
    void Log_Write(uint64_t time_us);

    /**
     * @brief Get GSF yaw estimator instance
     * 
     * @return Pointer to EKFGSF_yaw object, or NULL if not available
     * 
     * @details Returns pointer to the Gaussian Sum Filter yaw estimator
     *          running in parallel with the main EKF.
     * 
     * GSF Yaw Estimator:
     * - Independent yaw estimation system
     * - Gaussian Sum Filter with multiple hypothesis
     * - Runs continuously in background
     * - Does not use magnetometer
     * - Provides emergency yaw backup
     * 
     * Estimator Architecture:
     * - Multiple parallel EKF instances (5-9 typical)
     * - Each with different initial yaw hypothesis
     * - Uses IMU and GPS velocity
     * - Optionally uses airspeed
     * - Weighted average provides yaw estimate
     * 
     * Use Cases:
     * - Accessing yaw estimate directly
     * - Monitoring GSF convergence
     * - Debugging yaw reset logic
     * - Logging GSF state
     * - Research and development
     * 
     * GSF Yaw Estimator Data:
     * - Current yaw estimate (rad)
     * - Yaw estimate variance
     * - Hypothesis weights
     * - Convergence status
     * - Validity flags
     * 
     * When Available:
     * - Allocated during core initialization
     * - Present if sufficient memory
     * - Active when vehicle has moved sufficiently
     * - Updated at IMU rate
     * 
     * When NULL:
     * - Memory allocation failed
     * - Feature disabled by parameter
     * - Not supported on platform
     * - Very unlikely in normal operation
     * 
     * Accessing GSF Data:
     * - Check pointer non-NULL before use
     * - Call EKFGSF_yaw methods for data
     * - getYawData() for estimate
     * - getYawVariance() for uncertainty
     * 
     * @note May return NULL if not allocated
     * @note GSF provides magnetometer-independent yaw
     * @note Primarily for emergency yaw recovery
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:341
     */
    const EKFGSF_yaw *get_yawEstimator(void) const { return yawEstimator; }
    
private:
    EKFGSF_yaw *yawEstimator;
    class AP_DAL &dal;

    // Reference to the global EKF frontend for parameters
    class NavEKF2 *frontend;
    uint8_t imu_index; // preferred IMU index
    uint8_t gyro_index_active; // active gyro index (in case preferred fails)
    uint8_t accel_index_active; // active accel index (in case preferred fails)
    uint8_t core_index;
    uint8_t imu_buffer_length;

#if MATH_CHECK_INDEXES
    typedef VectorN<ftype,2> Vector2;
    typedef VectorN<ftype,3> Vector3;
    typedef VectorN<ftype,4> Vector4;
    typedef VectorN<ftype,5> Vector5;
    typedef VectorN<ftype,6> Vector6;
    typedef VectorN<ftype,7> Vector7;
    typedef VectorN<ftype,8> Vector8;
    typedef VectorN<ftype,9> Vector9;
    typedef VectorN<ftype,10> Vector10;
    typedef VectorN<ftype,11> Vector11;
    typedef VectorN<ftype,13> Vector13;
    typedef VectorN<ftype,14> Vector14;
    typedef VectorN<ftype,15> Vector15;
    typedef VectorN<ftype,22> Vector22;
    typedef VectorN<ftype,23> Vector23;
    typedef VectorN<ftype,24> Vector24;
    typedef VectorN<ftype,25> Vector25;
    typedef VectorN<ftype,31> Vector31;
    typedef VectorN<VectorN<ftype,3>,3> Matrix3;
    
    /**
     * @typedef Matrix24
     * @brief 24x24 covariance matrix type with bounds checking
     * 
     * @details Type used for the EKF state covariance matrix P. This typedef provides
     *          a checked vector implementation that validates array indices at runtime
     *          when MATH_CHECK_INDEXES is enabled for debugging.
     * 
     * Matrix Structure:
     * - Dimensions: 24x24 elements (576 total elements)
     * - Element type: ftype (float or double depending on HAL_WITH_EKF_DOUBLE)
     * - Memory: 2.3 KB (single precision) or 4.6 KB (double precision)
     * - Symmetry: Maintained symmetric through ForceSymmetry() operations
     * 
     * Covariance Matrix P Usage:
     * - Diagonal elements [i][i]: Variances of state i (squared units)
     * - Off-diagonal elements [i][j]: Covariances between states i and j
     * - Must remain positive definite for filter stability
     * - Numerical conditioning critical for accurate innovation calculations
     * 
     * State Indexing (0-23):
     * - 0-2: Rotation error (rad²)
     * - 3-5: Velocity NED (m²/s²)
     * - 6-8: Position NED (m²)
     * - 9-11: Gyro bias (rad²/s²)
     * - 12-14: Gyro scale factor (dimensionless²)
     * - 15: Accel Z bias (m²/s⁴)
     * - 16-18: Earth magnetic field NED (gauss²/10⁶)
     * - 19-21: Body magnetic field XYZ (gauss²/10⁶)
     * - 22-23: Wind velocity NE (m²/s²)
     * 
     * @note VectorN implementation provides runtime bounds checking when enabled
     * @note Checked version used during development and testing
     * @note See unchecked typedef for production builds
     * 
     * @see P for the actual covariance matrix instance
     * @see CovariancePrediction() for covariance propagation
     * @see ForceSymmetry() for symmetry enforcement
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:375
     */
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
    typedef ftype Vector22[22];
    typedef ftype Vector23[23];
    typedef ftype Vector24[24];
    typedef ftype Vector25[25];
    typedef ftype Matrix3[3][3];
    
    /**
     * @typedef Matrix24
     * @brief 24x24 covariance matrix type using unchecked C arrays for production
     * 
     * @details Type used for the EKF state covariance matrix P. This typedef uses
     *          standard C arrays without bounds checking for optimal performance in
     *          production builds (MATH_CHECK_INDEXES disabled).
     * 
     * Matrix Structure:
     * - Dimensions: 24x24 elements (576 total elements)
     * - Element type: ftype (float or double depending on HAL_WITH_EKF_DOUBLE)
     * - Memory layout: Contiguous row-major C array
     * - Memory: 2.3 KB (single precision) or 4.6 KB (double precision)
     * - Symmetry: Maintained symmetric through ForceSymmetry() operations
     * 
     * Covariance Matrix P Usage:
     * - Diagonal elements [i][i]: Variances of state i (squared units)
     * - Off-diagonal elements [i][j]: Covariances between states i and j
     * - Must remain positive definite for filter stability
     * - Numerical conditioning critical for accurate innovation calculations
     * 
     * State Indexing (0-23):
     * - 0-2: Rotation error (rad²)
     * - 3-5: Velocity NED (m²/s²)
     * - 6-8: Position NED (m²)
     * - 9-11: Gyro bias (rad²/s²)
     * - 12-14: Gyro scale factor (dimensionless²)
     * - 15: Accel Z bias (m²/s⁴)
     * - 16-18: Earth magnetic field NED (gauss²/10⁶)
     * - 19-21: Body magnetic field XYZ (gauss²/10⁶)
     * - 22-23: Wind velocity NE (m²/s²)
     * 
     * Performance Characteristics:
     * - No runtime bounds checking (faster but requires careful index management)
     * - Optimal cache locality with contiguous memory
     * - Direct memory access without indirection overhead
     * - Used in all production flight software builds
     * 
     * @warning No bounds checking - invalid indices cause undefined behavior
     * @note Production build typedef (MATH_CHECK_INDEXES not defined)
     * @note Array indexing: P[row][col] where row and col range from 0 to 23
     * 
     * @see P for the actual covariance matrix instance
     * @see CovariancePrediction() for covariance propagation
     * @see ForceSymmetry() for symmetry enforcement
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:397
     */
    typedef ftype Matrix24[24][24];
    typedef ftype Matrix34_50[34][50];
    typedef uint32_t Vector_u32_50[50];
#endif

    /**
     * @struct state_elements
     * @brief EKF2 24-state vector broken down by component groups
     * 
     * @details This structure provides named access to the 24-state EKF state vector.
     *          The state vector uses a rotation vector parameterization for attitude
     *          error combined with a full quaternion representation for the attitude
     *          itself (28 total elements when including quaternion).
     * 
     * State Vector Organization:
     * - The states are available in two forms: Vector28 array or this struct
     * - Both representations occupy the same memory via union
     * - Allows both indexed access (for math operations) and named access (for clarity)
     * 
     * Coordinate Frame Conventions:
     * - angErr, velocity, position: NED (North-East-Down) navigation frame
     * - gyro_bias, gyro_scale, accel_zbias, body_magfield: Body frame
     * - earth_magfield: NED navigation frame
     * - wind_vel: NE horizontal plane of navigation frame
     * - quat: Rotation from NED frame to body frame
     * 
     * @note State indices 0-23 for error states, 24-27 for quaternion (total 28)
     * @note Attitude error uses rotation vector (small angle approximation during updates)
     * @note Full quaternion provides complete 3D orientation without gimbal lock
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:405-416
     */
    struct state_elements {
        /**
         * @brief Rotation error vector (states 0-2)
         * @details 3D rotation error in radians using rotation vector parameterization.
         *          During covariance updates, attitude errors are represented as small
         *          rotation vectors about NED axes. These errors are applied to the
         *          quaternion after each update and then reset to zero.
         * @note Units: radians (rad)
         * @note Frame: NED navigation frame
         * @warning Must remain small (< 0.1 rad) for linearization validity
         */
        Vector3F    angErr;         // 0..2
        
        /**
         * @brief Velocity vector in NED frame (states 3-5)
         * @details 3D velocity with components:
         *          - velocity[0]: North velocity (positive = moving north)
         *          - velocity[1]: East velocity (positive = moving east)  
         *          - velocity[2]: Down velocity (positive = moving down/descending)
         * @note Units: meters per second (m/s)
         * @note Frame: NED navigation frame
         * @note Typical range: 0-50 m/s for multirotors, 0-150 m/s for fixed-wing
         */
        Vector3F    velocity;       // 3..5
        
        /**
         * @brief Position vector in NED frame relative to EKF origin (states 6-8)
         * @details 3D position with components:
         *          - position[0]: North position from origin (positive = north of origin)
         *          - position[1]: East position from origin (positive = east of origin)
         *          - position[2]: Down position from origin (positive = below origin)
         * @note Units: meters (m)
         * @note Frame: NED navigation frame
         * @note Origin set by setOriginLLH() or first GPS fix
         * @note Limited to ±EK2_POSXY_STATE_LIMIT to prevent overflow
         */
        Vector3F    position;       // 6..8
        
        /**
         * @brief Gyroscope bias estimate in body frame (states 9-11)
         * @details 3D gyro bias with components for body X, Y, Z axes.
         *          These biases are subtracted from raw gyro measurements.
         *          Learned during initial alignment and continuously updated in flight.
         * @note Units: radians per second (rad/s)
         * @note Frame: Body frame (X forward, Y right, Z down)
         * @note Typical values: ±0.01 rad/s (±0.57 deg/s)
         * @warning Critical for attitude accuracy - poor gyro bias degrades orientation
         */
        Vector3F    gyro_bias;      // 9..11
        
        /**
         * @brief Gyroscope scale factor error (states 12-14)
         * @details Dimensionless scale factor errors for body X, Y, Z gyros.
         *          Applied as: gyro_corrected = gyro_raw * (1 + gyro_scale).
         *          Typically very small and not always observable in flight.
         * @note Units: dimensionless (fraction)
         * @note Frame: Body frame
         * @note Typical values: ±0.01 (±1% scale error)
         * @note Often held constant unless long-duration flights with persistent rotation
         */
        Vector3F    gyro_scale;     // 12..14
        
        /**
         * @brief Z-axis accelerometer bias (state 15)
         * @details Bias on the body Z-axis (down) accelerometer in m/s².
         *          Only Z-axis bias estimated as it's most observable from vertical motion.
         *          Subtracted from Z-accel measurement before use.
         * @note Units: meters per second squared (m/s²)
         * @note Frame: Body frame Z-axis (down in body frame)
         * @note Typical values: ±0.5 m/s²
         * @note X and Y accel biases not estimated (less observable in normal flight)
         */
        ftype       accel_zbias;    // 15
        
        /**
         * @brief Earth magnetic field in NED frame (states 16-18)
         * @details 3D earth magnetic field vector with components:
         *          - earth_magfield[0]: North magnetic field component
         *          - earth_magfield[1]: East magnetic field component
         *          - earth_magfield[2]: Down magnetic field component (typically dominant)
         * @note Units: gauss/1000 (milligauss)
         * @note Frame: NED navigation frame
         * @note Learned in-flight or initialized from WMM (World Magnetic Model) tables
         * @note Magnitude typically 0.2-0.6 gauss depending on latitude
         * @note Provides yaw reference when GPS velocity unavailable
         */
        Vector3F    earth_magfield; // 16..18
        
        /**
         * @brief Body-fixed magnetic field offsets (states 19-21)
         * @details 3D magnetic field offsets in body frame, representing:
         *          - Hard iron effects (permanent magnetization)
         *          - Soft iron effects (approximately, combined with earth field)
         *          - Vehicle-induced magnetic interference
         * @note Units: gauss/1000 (milligauss)
         * @note Frame: Body frame (X forward, Y right, Z down)
         * @note Learned during flight, especially during maneuvers
         * @note Used to calibrate magnetometer readings: mag_calibrated = mag_raw - body_magfield
         */
        Vector3F    body_magfield;  // 19..21
        
        /**
         * @brief Wind velocity in NE horizontal plane (states 22-23)
         * @details 2D horizontal wind velocity with components:
         *          - wind_vel[0]: North wind component (positive = wind from south)
         *          - wind_vel[1]: East wind component (positive = wind from west)
         * @note Units: meters per second (m/s)
         * @note Frame: NE horizontal plane of NED frame
         * @note Convention: Positive values mean air moving in that axis direction
         * @note Only observable with airspeed sensor or significant maneuvering
         * @note Used for fixed-wing navigation and energy management
         */
        Vector2F    wind_vel;       // 22..23
        
        /**
         * @brief Attitude quaternion (states 24-27)
         * @details Quaternion representing rotation from NED navigation frame to body frame.
         *          Four components: q0 (scalar/real), q1, q2, q3 (vector/imaginary).
         *          Maintained as unit quaternion (magnitude = 1.0).
         * @note Units: dimensionless (unit quaternion)
         * @note Rotation: NED frame → Body frame
         * @note Updated by applying angErr after covariance updates, then angErr reset
         * @note Normalized periodically to maintain unit constraint
         * @note Provides gimbal-lock-free 3D orientation representation
         */
        QuaternionF quat;           // 24..27
    };

    union {
        Vector28 statesArray;
        struct state_elements stateStruct;
    };

    /**
     * @struct output_elements
     * @brief Output state buffer element for predictor-corrector observer
     * 
     * @details Buffered output states used to propagate EKF solution forward from
     *          fusion time horizon to current time using complementary filter observer.
     *          Provides smooth output at IMU rate even when measurements update slowly.
     * 
     * @note Units: quat (dimensionless), velocity (m/s), position (m)
     * @note Frame: NED navigation frame for velocity and position
     * @note Buffered at IMU rate for output interpolation
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:423-427
     */
    struct output_elements {
        QuaternionF quat;           // 0..3  - Attitude quaternion (NED to body)
        Vector3F    velocity;       // 4..6  - Velocity NED (m/s)
        Vector3F    position;       // 7..9  - Position NED (m)
    };

    /**
     * @struct imu_elements
     * @brief IMU measurement buffer element
     * 
     * @details Stores one IMU sample with delta angles and delta velocities
     *          integrated over the IMU sample period. Used for both buffering
     *          raw measurements and time-delayed fusion alignment.
     * 
     * Delta Angles and Velocities:
     * - delAng: Integrated gyro measurements over sample period (rad)
     * - delVel: Integrated accelerometer measurements over sample period (m/s)
     * - Computed by IMU driver, allows variable sample rates
     * 
     * Timing Information:
     * - delAngDT: Time interval for delta angle integration (s)
     * - delVelDT: Time interval for delta velocity integration (s)  
     * - time_ms: System timestamp when measurement received (ms)
     * - Typically delAngDT ≈ delVelDT ≈ 0.0025s (400 Hz IMU)
     * 
     * @note Units: delAng (rad), delVel (m/s), delAngDT/delVelDT (s)
     * @note Frame: Body frame (X forward, Y right, Z down)
     * @note gyro_index/accel_index: Identifies which IMU hardware unit
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:429-437
     */
    struct imu_elements {
        Vector3F    delAng;         // 0..2  - Delta angles over sample period (rad, body frame)
        Vector3F    delVel;         // 3..5  - Delta velocities over sample period (m/s, body frame)
        ftype       delAngDT;       // 6     - Delta angle integration time (s)
        ftype       delVelDT;       // 7     - Delta velocity integration time (s)
        uint32_t    time_ms;        // 8     - System timestamp of measurement (ms)
        uint8_t     gyro_index;     //       - Gyro sensor index (0, 1, 2...)
        uint8_t     accel_index;    //       - Accelerometer sensor index (0, 1, 2...)
    };

    /**
     * @struct gps_elements
     * @brief GPS measurement buffer element
     * 
     * @details Stores one GPS measurement sample including horizontal position,
     *          height, and 3D velocity. Inherits timestamp from EKF_obs_element_t.
     * 
     * Position Convention:
     * - pos: North/East position (m) relative to EKF origin
     * - hgt: Height (m) - typically WGS-84 ellipsoid height from GPS
     * - Converted from GPS lat/lon/alt to local NED at reception
     * 
     * Velocity Convention:
     * - vel: 3D velocity in NED frame (m/s)
     * - Includes North, East, Down velocity components
     * - Down velocity used for vertical velocity aiding
     * 
     * @note Units: pos (m), hgt (m), vel (m/s)
     * @note Frame: NED navigation frame relative to EKF origin
     * @note sensor_idx: Identifies which GPS receiver (0, 1 for dual GPS)
     * @note Buffered at GPS update rate (typically 5-10 Hz)
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:439-444
     */
    struct gps_elements : EKF_obs_element_t {
        Vector2F    pos;            // Horizontal position NE (m, relative to EKF origin)
        ftype       hgt;            // Height (m, WGS-84 ellipsoid height)
        Vector3F    vel;            // Velocity NED (m/s)
        uint8_t     sensor_idx;     // GPS receiver index (0, 1, 2...)
    };

    /**
     * @struct mag_elements
     * @brief Magnetometer measurement buffer element
     * 
     * @details Stores one 3-axis magnetometer measurement sample.
     *          Measurements are in body frame and corrected for hard/soft iron
     *          effects by the compass library before reaching the EKF.
     * 
     * Measurement Convention:
     * - mag[0]: X-axis magnetic field (body forward)
     * - mag[1]: Y-axis magnetic field (body right)
     * - mag[2]: Z-axis magnetic field (body down)
     * 
     * Fusion Approach:
     * - Fused sequentially (one axis per time step) to reduce computational load
     * - Provides yaw observability and magnetic field state updates
     * - Can be degraded or disabled in magnetically disturbed environments
     * 
     * @note Units: gauss or milligauss (depends on compass library scaling)
     * @note Frame: Body frame (X forward, Y right, Z down)
     * @note Buffered at magnetometer update rate (typically 10-100 Hz)
     * @note Corrected for hard/soft iron effects before buffering
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:446-448
     */
    struct mag_elements : EKF_obs_element_t {
        Vector3F    mag;            // Magnetic field XYZ (body frame, gauss or mgauss)
    };

    /**
     * @struct baro_elements
     * @brief Barometer measurement buffer element
     * 
     * @details Stores one barometric altitude measurement. Height is typically
     *          converted from pressure to altitude using standard atmosphere model.
     * 
     * Height Convention:
     * - hgt: Altitude relative to reference datum (typically MSL or ground level)
     * - Sign: Positive up (opposite of NED down convention)
     * - Converted to NED down position (negative) when fused
     * 
     * Characteristics:
     * - Smooth but drifts with pressure changes
     * - Affected by weather patterns and ground effect during takeoff/landing
     * - Most common height reference for multirotors
     * 
     * @note Units: meters (m)
     * @note Frame: Up positive (converted to NED down negative for fusion)
     * @note Buffered at barometer update rate (typically 10-50 Hz)
     * @note Subject to ground effect (static pressure errors) in ground proximity
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:450-452
     */
    struct baro_elements : EKF_obs_element_t {
        ftype       hgt;            // Barometric height (m, up positive)
    };

    /**
     * @struct range_elements
     * @brief Range finder measurement buffer element
     * 
     * @details Stores one distance measurement from downward-facing range finder
     *          (lidar, sonar, or radar). Provides height above ground level.
     * 
     * Range Convention:
     * - rng: Distance from sensor to ground (m)
     * - Corrected for sensor mounting position and vehicle attitude
     * - Used for terrain following and precision landing
     * 
     * Fusion Conditions:
     * - Only used when in valid range (typically 0.2m to 40m depending on sensor)
     * - Requires reasonably level attitude for accurate height interpretation
     * - Can be primary height source when enabled (EK2_RNG_AID_HGT)
     * 
     * @note Units: meters (m)
     * @note Frame: Body down direction (distance positive)
     * @note sensor_idx: Range finder instance (0, 1 for dual rangefinders)
     * @note Buffered at range finder update rate (typically 10-50 Hz)
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:454-457
     */
    struct range_elements : EKF_obs_element_t {
        ftype       rng;            // Range to ground (m, positive down)
        uint8_t     sensor_idx;     // Range finder sensor index (0, 1, 2...)
    };

    /**
     * @struct rng_bcn_elements
     * @brief Range beacon measurement buffer element
     * 
     * @details Stores one range measurement to a beacon at a known position.
     *          Multiple beacons enable 3D position estimation (similar to GPS
     *          but using distances instead of pseudoranges).
     * 
     * Beacon System:
     * - Beacons at fixed, surveyed NED positions
     * - Vehicle measures range (distance) to each beacon
     * - Triangulation provides position estimate
     * - Useful for indoor/GPS-denied environments
     * 
     * Error Model:
     * - rngErr: 1-sigma measurement uncertainty (m)
     * - Used to weight measurements in Kalman gain calculation
     * - Typically 0.1-1.0m depending on beacon system
     * 
     * @note Units: rng (m), beacon_posNED (m), rngErr (m)
     * @note Frame: beacon_posNED in NED navigation frame
     * @note beacon_ID: Unique identifier for each beacon
     * @note Requires 4+ beacons for full 3D position solution
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:459-464
     */
    struct rng_bcn_elements : EKF_obs_element_t {
        ftype       rng;                // Range measurement to beacon (m)
        Vector3F    beacon_posNED;      // NED position of beacon (m, relative to EKF origin)
        ftype       rngErr;             // Range measurement error 1-sigma (m)
        uint8_t     beacon_ID;          // Beacon identification number
    };

    /**
     * @struct tas_elements
     * @brief True airspeed measurement buffer element
     * 
     * @details Stores one true airspeed measurement from pitot-static system
     *          or estimated from other sources. Used primarily by fixed-wing
     *          for wind estimation and navigation.
     * 
     * Airspeed Convention:
     * - tas: True airspeed (TAS) magnitude (m/s)
     * - Scalar magnitude, no direction (direction inferred from velocity + wind)
     * - Corrected for air density (altitude and temperature)
     * - Typical conversion: IAS → EAS → TAS
     * 
     * Fusion Benefits:
     * - Enables wind state estimation
     * - Provides forward velocity observability for fixed-wing
     * - Improves navigation accuracy in wind
     * - Assists with GPS velocity validation
     * 
     * @note Units: meters per second (m/s)
     * @note Scalar magnitude (no direction)
     * @note Only meaningful for fixed-wing or high-speed forward flight
     * @note Buffered at airspeed sensor update rate (typically 5-10 Hz)
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:466-468
     */
    struct tas_elements : EKF_obs_element_t {
        ftype       tas;            // True airspeed magnitude (m/s)
    };

    /**
     * @struct of_elements
     * @brief Optical flow measurement buffer element
     * 
     * @details Stores optical flow sensor data including raw flow rates,
     *          compensated flow rates, and sensor motion. Used for velocity
     *          estimation when GPS unavailable (indoor flight).
     * 
     * Flow Rate Convention:
     * - flowRadXY: Raw optical flow rates (rad/s) about sensor X and Y axes
     * - flowRadXYcomp: Gyro-compensated flow rates (rad/s)
     * - bodyRadXYZ: Sensor angular rates (rad/s) from internal gyro
     * - Positive flow = apparent ground motion in positive axis direction
     * 
     * Sensor Mounting:
     * - body_offset: XYZ position of flow sensor in body frame (m)
     * - Accounts for sensor offset from IMU for velocity calculation
     * - Typically mounted on bottom of vehicle pointing down
     * 
     * Height Override:
     * - heightOverride: Fixed height for rovers (m), 0 if not used
     * - Rovers have constant height, simplifies flow-to-velocity conversion
     * 
     * @note Units: flowRadXY/flowRadXYcomp/bodyRadXYZ (rad/s), body_offset (m), heightOverride (m)
     * @note Frame: Sensor frame (typically X forward, Y right)
     * @note Requires valid height estimate for velocity calculation
     * @note Quality depends on surface texture and lighting
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:470-476
     */
    struct of_elements : EKF_obs_element_t {
        Vector2F    flowRadXY;      // Raw optical flow rates XY (rad/s, sensor frame)
        Vector2F    flowRadXYcomp;  // Gyro-compensated flow rates XY (rad/s, sensor frame)
        Vector3F    bodyRadXYZ;     // Sensor angular rates XYZ (rad/s, body frame)
        Vector3F    body_offset;    // Sensor position offset XYZ (m, body frame)
        float       heightOverride; // Fixed height for rovers (m), 0 if not used
    };

    /**
     * @struct ext_nav_elements
     * @brief External navigation system measurement buffer element
     * 
     * @details Stores position and attitude measurements from external navigation
     *          systems such as motion capture (Vicon, OptiTrack), visual-inertial
     *          odometry (T265), or other external localization sources.
     * 
     * Position Measurement:
     * - pos: 3D position in right-handed navigation frame (m)
     * - Frame assumed to be NED or compatible coordinate system
     * - Converted to EKF local NED frame if needed
     * - posErr: 1-sigma spherical position uncertainty (m)
     * 
     * Attitude Measurement:
     * - quat: Quaternion rotation from navigation frame to body frame
     * - Can be used for yaw aiding when enabled
     * - angErr: 1-sigma spherical angular uncertainty (rad)
     * 
     * Reset Handling:
     * - posReset: Flag indicating external system performed position reset
     * - Triggers EKF position reset to maintain alignment
     * - Critical for systems that reset origin periodically
     * 
     * Use Cases:
     * - Indoor flight with motion capture systems
     * - Visual odometry for GPS-denied navigation
     * - Integration with external SLAM systems
     * - Fusion with other absolute position references
     * 
     * @note Units: pos (m), posErr (m), angErr (rad)
     * @note Frame: Right-handed navigation frame (assumed NED-compatible)
     * @note Requires sensor offset configuration via AP_VisualOdom library
     * @note Quality depends on external system accuracy and update rate
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:478-484
     */
    struct ext_nav_elements : EKF_obs_element_t {
        Vector3F        pos;        // XYZ position in navigation frame (m)
        QuaternionF     quat;       // Quaternion rotation navigation→body frame
        ftype           posErr;     // Spherical position measurement error 1-sigma (m)
        ftype           angErr;     // Spherical angular measurement error 1-sigma (rad)
        bool            posReset;   // True when external system reset its position reference
    };

    /**
     * @struct inactiveBias
     * @brief Bias estimates for inactive IMUs
     * 
     * @details Stores learned bias estimates for IMU sensors that are available
     *          but not currently being used by this EKF core. Allows rapid
     *          convergence when switching to a different IMU.
     * 
     * Bias Learning:
     * - Biases learned from active core propagated to inactive cores
     * - Reduces initialization time if IMU switch occurs
     * - Maintains estimates even when IMU not in use
     * 
     * Storage:
     * - Array indexed by IMU instance number
     * - One entry per possible IMU (up to INS_MAX_INSTANCES)
     * - Only entries for detected IMUs are updated
     * 
     * @note Units: gyro_bias (rad/s), gyro_scale (dimensionless), accel_zbias (m/s²)
     * @note Frame: Body frame
     * @note Updated via learnInactiveBiases() method
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:486-492
     */
    struct {
        Vector3F gyro_bias;         // Gyro bias XYZ (rad/s, body frame)
        Vector3F gyro_scale;        // Gyro scale factor error XYZ (dimensionless)
        ftype accel_zbias;          // Z-axis accel bias (m/s², body frame)
    } inactiveBias[INS_MAX_INSTANCES];

    /**
     * @struct ext_nav_vel_elements
     * @brief External navigation velocity measurement buffer element
     * 
     * @details Stores velocity measurements from external navigation systems.
     *          Complements position/attitude measurements from ext_nav_elements.
     * 
     * Velocity Measurement:
     * - vel: 3D velocity vector in NED frame (m/s)
     * - Provides additional observability for velocity states
     * - Can be fused independently of position
     * 
     * Error Model:
     * - err: 1-sigma velocity measurement uncertainty (m/s)
     * - Scalar error applied to all velocity components
     * - Used to weight measurements in Kalman update
     * 
     * @note Units: vel (m/s), err (m/s)
     * @note Frame: NED navigation frame
     * @note Complements ext_nav position measurements
     * @note Useful when external system provides reliable velocity
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:494-497
     */
    struct ext_nav_vel_elements : EKF_obs_element_t {
        Vector3F vel;               // Velocity NED (m/s)
        ftype err;                  // Velocity measurement error 1-sigma (m/s)
    };

    /**
     * @brief Update the navigation filter status structure
     * 
     * @details Updates the nav_filter_status structure that tracks filter health,
     *          aiding mode, and output validity. This status is queried by the vehicle
     *          code to determine if EKF estimates are safe for flight control.
     * 
     * Status Flags Updated:
     * - Attitude validity (roll, pitch, yaw)
     * - Velocity validity (NED components)
     * - Position validity (horizontal and vertical)
     * - Height source being used
     * - Takeoff detection
     * - GPS usage status
     * - Aiding mode (absolute, relative, none)
     * 
     * @note Called after every measurement update cycle
     * @note Flight control should not use outputs when status indicates invalid
     * 
     * @see getFilterStatus() for external access to status
     * @see nav_filter_status structure definition
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:500
     */
    void  updateFilterStatus(void);

    /**
     * @brief Update quaternion, velocity and position states using IMU measurements
     * 
     * @details Implements the strapdown inertial navigation equations to propagate
     *          the navigation state forward using delta angles and delta velocities
     *          from the IMU. This is the prediction step of the EKF.
     * 
     * State Updates:
     * - Quaternion: Updated by rotating through corrected delta angles
     * - Velocity: Updated by transforming corrected delta velocity to NED frame
     * - Position: Updated by integrating velocity
     * - Biases: Remain constant (updated only by measurement corrections)
     * 
     * Process:
     * 1. Apply gyro bias and scale factor corrections to delta angles
     * 2. Rotate quaternion through corrected delta angles
     * 3. Apply accel Z bias correction to delta velocity
     * 4. Transform delta velocity from body to NED frame
     * 5. Add coriolis and gravity corrections
     * 6. Update velocity and position states
     * 
     * Coordinate Frames:
     * - Input: Delta angles and velocities in body frame
     * - Processing: Transformation using quaternion (body → NED)
     * - Output: Updated velocity and position in NED frame
     * 
     * @note Called at IMU rate (typically 400Hz for prediction)
     * @note Uses imuDataDelayed from the IMU buffer at fusion time horizon
     * @note Earth rotation compensation applied using earthRateNED
     * 
     * @warning Quaternion must be normalized before/after rotation
     * @warning Delta angle and velocity corrections must be bias-compensated
     * 
     * @see readIMUData() for IMU data acquisition
     * @see correctDeltaAngle() for gyro bias/scale compensation
     * @see correctDeltaVelocity() for accel bias compensation
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:503
     */
    void UpdateStrapdownEquationsNED();

    /**
     * @brief Calculate the predicted state covariance matrix
     * 
     * @details Propagates the covariance matrix P forward in time using the EKF
     *          prediction equations. This accounts for how uncertainty grows due to
     *          process noise (IMU errors, environmental disturbances).
     * 
     * Prediction Equation:
     * P(k+1|k) = F * P(k|k) * F' + Q
     * 
     * Where:
     * - F: State transition matrix (Jacobian of state equations)
     * - P: State covariance matrix (24x24)
     * - Q: Process noise covariance matrix
     * - ': Matrix transpose
     * 
     * Process Noise Sources (Q matrix):
     * - IMU gyro noise and bias random walk
     * - IMU accel noise and bias random walk
     * - Gyro scale factor random walk
     * - Magnetic field random walk
     * - Wind velocity random walk
     * 
     * Implementation:
     * - Matlab-generated symbolic equations for computational efficiency
     * - Optimized to reduce matrix multiplications
     * - Selective state updates based on stateIndexLim (wind/mag learning)
     * - Uses ftype (float/double) for numerical precision
     * 
     * Numerical Stability:
     * - Symmetry enforced after prediction (ForceSymmetry)
     * - Variance limits applied (ConstrainVariances)
     * - Covariance numerical corrections (CopyCovariances)
     * 
     * @param dt Time step for prediction (typically EKF_TARGET_DT = 0.01s)
     * 
     * @note Called once per EKF update cycle (typically 100Hz)
     * @note Computationally intensive - uses GCC O2 optimization
     * @note Derived from symbolic differentiation in Matlab (see file header reference)
     * 
     * @warning Process noise tuning affects filter convergence and stability
     * @warning Excessive process noise causes filter divergence
     * @warning Insufficient process noise causes filter overconfidence
     * 
     * @see ForceSymmetry() for symmetry enforcement
     * @see ConstrainVariances() for variance limiting
     * @see CopyCovariances() for numerical correction
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:506
     */
    void CovariancePrediction();

    /**
     * @brief Force symmetry on the state covariance matrix
     * 
     * @details The covariance matrix P must be symmetric (P[i][j] = P[j][i]) by definition,
     *          but numerical errors during matrix operations can introduce small asymmetries.
     *          This function enforces symmetry by averaging corresponding off-diagonal elements.
     * 
     * Algorithm:
     * For all i,j where i != j:
     *   P[i][j] = P[j][i] = (P[i][j] + P[j][i]) / 2
     * 
     * Why Symmetry Matters:
     * - Required for valid covariance matrix (mathematical definition)
     * - Ensures consistent innovation variance calculations
     * - Prevents numerical instability in Kalman gain computation
     * - Maintains positive definiteness property
     * 
     * When Called:
     * - After every CovariancePrediction() step
     * - After measurement updates that modify P
     * - During covariance resets
     * 
     * Performance:
     * - O(n²) operation for n=24 states (276 element pairs)
     * - Minimal computational cost compared to full prediction
     * - Essential for long-term numerical stability
     * 
     * @note Only operates on lower triangle, then mirrors to upper
     * @note Diagonal elements are not modified (already symmetric)
     * @note Critical for preventing accumulation of numerical errors
     * 
     * @warning Skipping this step leads to filter divergence over time
     * 
     * @see CovariancePrediction() which calls this after prediction
     * @see Matrix24 covariance matrix type definition
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:509
     */
    void ForceSymmetry();

    /**
     * @brief Copy covariances across from prediction calculation and fix numerical errors
     * 
     * @details After the covariance prediction, this function copies the computed covariances
     *          and applies numerical corrections to prevent issues like negative variances or
     *          NaN values that can arise from floating-point arithmetic errors.
     * 
     * Numerical Corrections Applied:
     * - Checks for NaN values in diagonal elements (variances)
     * - Ensures all diagonal elements remain positive
     * - Applies lower bounds to prevent variances approaching zero
     * - Corrects propagation errors from single/double precision limits
     * 
     * Error Detection:
     * - NaN detection: if (P[i][i] != P[i][i]) → indicates NaN
     * - Negative variance detection: if (P[i][i] < 0) → numerical error
     * - Near-zero variance: if (P[i][i] < minimum) → may cause division by zero
     * 
     * Fault Recovery:
     * - Sets faultStatus flags for severe errors
     * - May trigger filter reinitialization if errors are critical
     * - Logs numerical issues for post-flight analysis
     * 
     * @note Called immediately after CovariancePrediction()
     * @note Works in conjunction with ConstrainVariances() for bounds
     * @note Essential for robust operation with limited precision arithmetic
     * 
     * @warning Numerical errors indicate potential filter divergence
     * @warning Repeated corrections may signal tuning issues
     * 
     * @see CovariancePrediction() for prediction equations
     * @see ConstrainVariances() for variance limit enforcement
     * @see faultStatus structure for error flags
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:512
     */
    void CopyCovariances();

    /**
     * @brief Constrain variances (diagonal terms) in the state covariance matrix
     * 
     * @details Applies upper and lower bounds to diagonal elements of P to prevent
     *          filter divergence or overconfidence. Variances that grow too large indicate
     *          divergence; variances that become too small indicate overconfidence.
     * 
     * Variance Constraints Applied:
     * - Position: Limited to prevent runaway position uncertainty
     * - Velocity: Limited based on expected vehicle dynamics
     * - Attitude: Limited to maintain valid attitude uncertainty
     * - Gyro bias: Constrained to realistic bias drift rates
     * - Magnetic field: Limited to valid field strength ranges
     * - Wind: Constrained to plausible wind speeds
     * 
     * Upper Limits (prevent divergence):
     * - Position: EK2_POSXY_STATE_LIMIT (1e6 m² or 50e6 m² for double)
     * - Velocity: Based on maximum vehicle speed squared
     * - Attitude: Limited to prevent attitude loss
     * 
     * Lower Limits (prevent overconfidence):
     * - All states: Minimum variance based on sensor noise floors
     * - Prevents division by near-zero in innovation calculations
     * - Maintains realistic uncertainty estimates
     * 
     * State-Specific Constraints:
     * - States 0-2 (rotation error): Angular variance limits
     * - States 3-5 (velocity): Speed-based limits
     * - States 6-8 (position): Position limits (different X/Y vs Z)
     * - States 9-14 (gyro bias/scale): Bias stability limits
     * - States 15 (accel Z bias): Accel bias limits
     * - States 16-21 (magnetic field): Field strength limits
     * - States 22-23 (wind): Wind speed limits
     * 
     * @note Called after CovariancePrediction() and measurement updates
     * @note Limits are tuned based on extensive flight testing
     * @note Different limits for single vs double precision (HAL_WITH_EKF_DOUBLE)
     * 
     * @warning Overly tight limits prevent filter adaptation to real changes
     * @warning Overly loose limits allow filter divergence
     * 
     * @see CovariancePrediction() for covariance propagation
     * @see EK2_POSXY_STATE_LIMIT for position limit definition
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:515
     */
    void ConstrainVariances();

    /**
     * @brief Constrain states to valid ranges
     * 
     * @details Applies physical and numerical limits to the state vector to prevent
     *          invalid values that could cause filter instability or unrealistic outputs.
     *          Complements ConstrainVariances() which limits uncertainty.
     * 
     * State Constraints:
     * - Position states: Limited to prevent numerical overflow
     * - Velocity states: Limited to maximum plausible vehicle speeds
     * - Quaternion: Normalized to maintain unit length
     * - Gyro bias: Limited to maximum expected bias values
     * - Gyro scale: Limited to realistic scale factor errors
     * - Accel Z bias: Limited to expected accelerometer bias range
     * - Magnetic field: Constrained to valid field strengths
     * - Wind velocity: Limited to maximum expected wind speeds
     * 
     * Position State Limits:
     * - Horizontal (N/E): ±EK2_POSXY_STATE_LIMIT (prevents overflow)
     * - Vertical (D): Based on vehicle operating altitude range
     * - Triggered by divergence or GPS jumps
     * 
     * Quaternion Normalization:
     * - Maintains unit quaternion constraint (q₀² + q₁² + q₂² + q₃² = 1)
     * - Prevents attitude representation singularities
     * - Called whenever quaternion is updated
     * 
     * Bias Constraints:
     * - Gyro bias: Typically ±0.5 rad/s (prevents unrealistic bias estimates)
     * - Accel Z bias: Typically ±1.0 m/s² (realistic accelerometer bias)
     * - Prevents bias states from compensating for modeling errors
     * 
     * @note Called after every state update (prediction and correction)
     * @note Critical for numerical stability with single-precision floats
     * @note Quaternion normalization called separately as needed
     * 
     * @warning Constraint violations indicate potential filter problems
     * @warning Position constraints prevent loss of GPS lock recovery
     * 
     * @see ConstrainVariances() for uncertainty limits
     * @see stateStruct for state vector definition
     * @see EK2_POSXY_STATE_LIMIT for position bounds
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:518
     */
    void ConstrainStates();

    /**
     * @brief Constrain earth magnetic field using WMM tables
     * 
     * @details Uses World Magnetic Model (WMM) lookup tables to constrain the estimated
     *          earth magnetic field states to expected values based on global position.
     *          Prevents magnetic field states from diverging to unrealistic values.
     * 
     * WMM Constraints:
     * - Field strength: Constrained to WMM predicted magnitude ±50%
     * - Field inclination: Constrained to WMM predicted dip angle ±20°
     * - Field declination: Constrained to WMM predicted declination ±20°
     * - Prevents anomalous magnetic interference from corrupting field estimates
     * 
     * When Applied:
     * - During magnetic field state initialization
     * - After magnetic field resets
     * - Periodically during flight if divergence detected
     * - When large deviations from WMM model are detected
     * 
     * WMM Table Usage:
     * - Requires valid GPS position (latitude/longitude)
     * - Interpolates WMM coefficients for current location
     * - Calculates expected field vector in NED frame
     * - Applies constraints to states 16-18 (earth_magfield)
     * 
     * Geographic Considerations:
     * - More aggressive near magnetic poles (large field inclination)
     * - Looser constraints in equatorial regions (weaker vertical component)
     * - Accounts for secular variation (annual field changes)
     * 
     * @note Only applied when have_table_earth_field flag is true
     * @note Requires valid GPS position for WMM lookup
     * @note Uses table_earth_field_ga and table_declination variables
     * 
     * @warning Overly aggressive constraints prevent adaptation to local anomalies
     * @warning Requires updated WMM tables (typically updated every 5 years)
     * 
     * @see table_earth_field_ga for WMM predicted field
     * @see table_declination for WMM predicted declination
     * @see alignMagStateDeclination() for declination alignment
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:521
     */
    void MagTableConstrain(void);

    /**
     * @brief Fuse selected position, velocity and height measurements
     * 
     * @details Implements the measurement update step of the EKF for GPS position and velocity.
     *          Uses sequential fusion (one measurement component at a time) to reduce
     *          computational cost while maintaining optimal Kalman gain calculation.
     * 
     * Measurements Fused:
     * - Velocity North (m/s) - if fuseVelData flag set
     * - Velocity East (m/s) - if fuseVelData flag set  
     * - Velocity Down (m/s) - if fuseVelData flag set
     * - Position North (m) - if fusePosData flag set
     * - Position East (m) - if fusePosData flag set
     * - Position/Height Down (m) - if fuseHgtData flag set
     * 
     * Sequential Fusion Process (per measurement):
     * 1. Calculate observation Jacobian H (sensitivity of measurement to states)
     * 2. Calculate innovation: y = z - h(x) (measurement - predicted)
     * 3. Calculate innovation variance: S = H*P*H' + R
     * 4. Calculate Kalman gain: K = P*H' / S
     * 5. Update states: x = x + K*y
     * 6. Update covariance: P = (I - K*H)*P
     * 7. Store innovation and variance for consistency checks
     * 
     * Innovation Consistency Checks:
     * - Normalized innovation squared (NIS) test: y²/S < threshold
     * - Failed checks increment timeout counters
     * - Repeated failures trigger velTimeout, posTimeout, hgtTimeout flags
     * 
     * Measurement Sources:
     * - GPS position/velocity (primary)
     * - External navigation system (alternative)
     * - Optical flow (velocity only, relative)
     * 
     * State Updates:
     * - Direct: Velocity (3-5), Position (6-8)
     * - Indirect: Attitude error (0-2) from velocity innovations
     * - Coupled: Gyro/accel biases, wind, magnetic field (if observable)
     * 
     * @note Called when GPS or external nav data available at fusion horizon
     * @note Innovation vectors stored in innovVelPos and varInnovVelPos
     * @note Fusion controlled by fuseVelData, fusePosData, fuseHgtData flags
     * 
     * @warning Innovation consistency check failures indicate GPS/measurement issues
     * @warning Excessive innovations trigger failsafe actions
     * 
     * @see SelectVelPosFusion() which sets fusion control flags
     * @see gpsDataDelayed for measurement data at fusion horizon
     * @see innovVelPos for innovation vector (6x1)
     * @see varInnovVelPos for innovation variance (6x1)
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:524
     */
    void FuseVelPosNED();

    /**
     * @brief Fuse range beacon measurements
     * 
     * @details Fuses range measurements from fixed beacons at known positions to estimate
     *          vehicle position. Supports 3D positioning when 4+ beacons are available,
     *          typically used indoors or in GPS-denied environments.
     * 
     * Range Beacon System:
     * - Beacons at known NED positions relative to origin
     * - Vehicle measures range (distance) to each beacon
     * - Triangulation provides 3D position estimate
     * - Requires minimum 4 beacons for 3D fix (3 for 2D)
     * 
     * Measurement Model:
     * - Predicted range: r_pred = ||beacon_pos - vehicle_pos||
     * - Innovation: y = r_measured - r_pred
     * - Nonlinear measurement (range involves square root)
     * 
     * Fusion Process:
     * 1. Select beacon with new measurement from storedRangeBeacon buffer
     * 2. Calculate predicted range to beacon from current state
     * 3. Calculate innovation (measured - predicted range)
     * 4. Calculate observation Jacobian ∂r/∂x (linearization)
     * 5. Sequential fusion using standard Kalman update
     * 6. Update position states and coupled states
     * 
     * State Updates:
     * - Primary: Position NED (states 6-8)
     * - Secondary: Velocity NED (states 3-5) if moving
     * - Coupled: Attitude error, biases (weak observability)
     * 
     * Vertical Datum Offset:
     * - Beacon constellation may have different vertical reference
     * - bcnPosOffset tracks offset between EKF and beacon datums
     * - Estimated using CalcRangeBeaconPosDownOffset()
     * 
     * Innovation Checks:
     * - Range innovation squared / variance < threshold
     * - Failed checks increment rngBcnTimeout counter
     * - rngBcnHealth flag indicates measurement validity
     * 
     * @note Only fuses when rngBcnDataToFuse flag is true
     * @note Beacon data buffered in storedRangeBeacon with timestamp
     * @note Requires rngBcnAlignmentCompleted flag for full fusion
     * 
     * @warning Range measurements corrupted by multipath indoors
     * @warning Poor beacon geometry (all beacons coplanar) weakens observability
     * 
     * @see FuseRngBcnStatic() for initial position solving
     * @see CalcRangeBeaconPosDownOffset() for vertical datum offset
     * @see rngBcnDataDelayed for measurement at fusion horizon
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:527
     */
    void FuseRngBcn();

    /**
     * @brief Use range beacon measurements to calculate a static position
     * 
     * @details Solves for initial vehicle position using range measurements to beacons
     *          at known positions. Used during filter initialization before full EKF fusion.
     *          Assumes vehicle is stationary during alignment.
     * 
     * Static Position Solving:
     * - Collects range measurements from multiple beacons
     * - Solves nonlinear least-squares problem for position
     * - Uses iterative Gauss-Newton or Levenberg-Marquardt method
     * - Requires minimum 4 range measurements for 3D position
     * 
     * Initialization Process:
     * 1. Accumulate range measurements from all visible beacons
     * 2. Initialize position estimate (center of beacon constellation)
     * 3. Iteratively refine position to minimize range residuals
     * 4. Calculate position covariance from residual statistics
     * 5. Set initial EKF position states and covariance
     * 
     * Least-Squares Cost Function:
     * Minimize: Σ(r_measured_i - ||beacon_pos_i - vehicle_pos||)²
     * 
     * Solution Method:
     * - Jacobian: ∂r_i/∂pos = -(beacon_i - pos) / ||beacon_i - pos||
     * - Update: pos = pos + (J'*J)^(-1) * J' * residuals
     * - Iterate until convergence or max iterations
     * 
     * Convergence Criteria:
     * - Position change < threshold (e.g., 0.01 m)
     * - RMS residual < threshold (e.g., 0.1 m)
     * - Maximum iterations reached (typically 10)
     * 
     * Alignment Completion:
     * - Sets rngBcnAlignmentCompleted flag when solved
     * - Stores solution in receiverPos variable
     * - Initializes receiverPosCov covariance (3x3)
     * - Resets EKF position states to solved position
     * 
     * @note Called during pre-flight beacon alignment phase
     * @note Requires rngBcnAlignmentStarted flag set
     * @note Vehicle must be stationary for accurate solution
     * 
     * @warning Poor beacon geometry causes ill-conditioned solve
     * @warning Moving vehicle during alignment produces incorrect position
     * @warning Requires at least 4 beacons for 3D position
     * 
     * @see FuseRngBcn() for dynamic fusion after alignment
     * @see receiverPos for solved position storage
     * @see rngBcnAlignmentCompleted flag
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:530
     */
    void FuseRngBcnStatic();

    /**
     * @brief Calculate the offset from EKF vertical position datum to the range beacon system datum
     * 
     * @details Estimates the vertical offset between the EKF's origin and the beacon constellation's
     *          reference frame. Necessary because beacon positions and EKF position may use
     *          different vertical datums (e.g., beacon floor vs GPS/baro origin).
     * 
     * @param[in] obsVar Observation variance for the offset estimation (m²)
     * @param[in] vehiclePosNED Current vehicle position in NED frame relative to EKF origin (m)
     * @param[in] aligning True during initial alignment phase, false during normal operation
     * 
     * Offset Estimation:
     * - Estimates single scalar offset: bcnPosOffset (m)
     * - Uses range innovations to infer vertical datum difference
     * - Maintains offset variance: bcnPosOffsetVar (m²)
     * - Updates using simple 1-state Kalman filter
     * 
     * Dual Hypothesis Tracking:
     * - bcnPosOffsetMax: Upper bound hypothesis
     * - bcnPosOffsetMin: Lower bound hypothesis  
     * - OffsetMaxInnovFilt: Filtered innovation for max hypothesis
     * - OffsetMinInnovFilt: Filtered innovation for min hypothesis
     * - Selects hypothesis with smallest innovation
     * 
     * Update Process:
     * 1. Calculate range predictions using both offset hypotheses
     * 2. Compare innovation magnitudes for each hypothesis
     * 3. Update selected hypothesis using Kalman filter
     * 4. Apply offset to beacon positions in range calculations
     * 
     * Alignment Mode (aligning = true):
     * - More aggressive learning rate (larger process noise)
     * - Faster convergence during initial alignment
     * - Relaxed innovation gating
     * 
     * Normal Mode (aligning = false):
     * - Conservative learning rate (smaller process noise)
     * - Prevents offset drift during flight
     * - Strict innovation gating
     * 
     * @note Offset required when beacon origin differs from EKF origin
     * @note Example: Beacons on building floor, EKF origin at GPS altitude
     * @note Maintains separate variances for min/max hypotheses
     * 
     * @warning Incorrect offset causes systematic range residuals
     * @warning Offset estimation requires vehicle height changes
     * 
     * @see FuseRngBcn() which calls this during beacon fusion
     * @see bcnPosOffset for estimated offset value
     * @see bcnPosOffsetVar for offset uncertainty
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:533
     */
    void CalcRangeBeaconPosDownOffset(ftype obsVar, Vector3F &vehiclePosNED, bool aligning);

    /**
     * @brief Fuse magnetometer measurements
     * 
     * @details Fuses 3-axis magnetometer measurements to constrain yaw angle and estimate
     *          magnetic field states. Critical for yaw observability in absence of GPS velocity.
     *          Uses sequential fusion (X, Y, Z components fused separately across time steps).
     * 
     * Magnetometer Measurement Model:
     * - Body frame measurement: [mag_x, mag_y, mag_z] in body frame (gauss/1000)
     * - Predicted: mag_body = DCM' * (earth_field + body_field)
     * - DCM: Direction cosine matrix from quaternion (NED to body rotation)
     * 
     * Sequential Fusion Strategy:
     * - Iteration 1: Fuse X-axis component
     * - Iteration 2: Fuse Y-axis component  
     * - Iteration 3: Fuse Z-axis component
     * - Spreads computational load across 3 time steps
     * - Maintains magnetometer state variables in mag_state structure
     * 
     * State Updates:
     * - Primary: Attitude quaternion (yaw component)
     * - Primary: Earth magnetic field NED (states 16-18)
     * - Primary: Body magnetic field offsets (states 19-21)
     * - Secondary: Coupled states (position, velocity weakly)
     * 
     * Yaw Observability:
     * - Magnetometer is primary yaw reference when GPS velocity unavailable
     * - Provides absolute heading reference (subject to declination)
     * - Essential for hover, low-speed flight, pre-takeoff alignment
     * 
     * Magnetic Field Learning:
     * - Earth field: Estimated from magnetometer during flight
     * - Body field: Estimated to compensate motor/battery interference
     * - Declination: Constrained to WMM table values
     * - Learning enabled when inhibitMagStates = false
     * 
     * Innovation Consistency:
     * - Per-axis innovation tests: mag_innov² / variance < threshold
     * - Failed tests set faultStatus.bad_xmag/ymag/zmag flags
     * - Repeated failures trigger magTimeout
     * - magHealth flag indicates overall magnetometer health
     * 
     * Declination Handling:
     * - Earth field constrained to match WMM declination
     * - FuseDeclination() provides soft constraint
     * - Prevents magnetic field states drifting from true north
     * 
     * @note Called when magDataToFuse flag is true
     * @note Fusion may be inhibited if GPS velocity provides yaw observability
     * @note Magnetic anomalies can corrupt yaw estimate
     * 
     * @warning Motor magnetic interference degrades yaw accuracy
     * @warning Hard iron disturbances require body field compensation
     * @warning Soft iron effects not modeled (can cause heading errors)
     * 
     * @see mag_state structure for fusion state variables
     * @see magDataDelayed for measurement at fusion horizon
     * @see FuseDeclination() for declination constraint
     * @see alignMagStateDeclination() for declination alignment
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:536
     */
    void FuseMagnetometer();

    /**
     * @brief Fuse true airspeed measurements
     * 
     * @details Fuses true airspeed (TAS) measurements from pitot tube to estimate wind velocity
     *          and improve velocity/position estimates. Particularly valuable for fixed-wing
     *          aircraft and provides observability of wind states.
     * 
     * @param[in] tasDataDelayed True airspeed measurement at fusion time horizon (m/s)
     * 
     * Airspeed Measurement Model:
     * - TAS = ||velocity_air|| where velocity_air = velocity_ground - wind
     * - Predicted TAS: sqrt((vN-wN)² + (vE-wE)² + vD²)
     * - Innovation: y = TAS_measured - TAS_predicted
     * - Nonlinear measurement (involves square root)
     * 
     * State Observability:
     * - Primary: Wind velocity NE (states 22-23)
     * - Secondary: Ground velocity NED (states 3-5)
     * - Tertiary: Position NED (states 6-8) indirectly
     * - Requires vehicle motion for wind observability
     * 
     * Wind Estimation:
     * - TAS + GPS velocity → wind velocity estimate
     * - Requires persistent excitation (varying flight direction)
     * - Wind states have weak observability (require time to converge)
     * - inhibitWindStates flag can freeze wind estimates
     * 
     * Sideslip Constraint:
     * - FuseSideslip() provides additional constraint
     * - Assumes zero sideslip for symmetric aircraft
     * - Improves wind estimation accuracy
     * - Complementary to airspeed fusion
     * 
     * Fusion Process:
     * 1. Calculate predicted TAS from state vector
     * 2. Compute innovation (measured - predicted)
     * 3. Linearize measurement model (Jacobian calculation)
     * 4. Calculate innovation variance
     * 5. Apply innovation consistency check
     * 6. Calculate Kalman gain and update states
     * 
     * Innovation Checks:
     * - Normalized innovation squared: innov² / variance < threshold
     * - Failed checks set faultStatus.bad_airspeed flag
     * - Timeout counter increments on repeated failures
     * - tasTimeout flag set if failures exceed threshold
     * 
     * EAS to TAS Conversion:
     * - Measurement may be EAS (equivalent airspeed)
     * - Converted to TAS using air density (altitude/temperature dependent)
     * - TAS = EAS * sqrt(ρ₀/ρ) where ρ₀ is sea level density
     * 
     * @note Only fuses when tasDataToFuse flag is true
     * @note useAirspeed() checks parameter settings and sensor health
     * @note Wind learning requires sufficient vehicle motion
     * 
     * @warning Airspeed sensor failures cause wind estimate divergence
     * @warning Blocked pitot tube produces large innovations
     * @warning Wind estimation slow to converge (requires varied flight paths)
     * 
     * @see FuseSideslip() for complementary synthetic sideslip constraint
     * @see tasDataDelayed for measurement at fusion horizon
     * @see useAirspeed() for sensor availability check
     * @see innovVtas and varInnovVtas for innovation storage
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:539
     */
    void FuseAirspeed();

    /**
     * @brief Fuse synthetic sideslip measurement of zero
     * 
     * @details Fuses a synthetic (virtual) measurement that assumes zero sideslip angle for
     *          the vehicle. Valid assumption for symmetric aircraft without significant
     *          sideslip. Provides additional constraint for wind velocity estimation.
     * 
     * Sideslip Measurement Model:
     * - Sideslip angle: β = atan2(v_body_y, v_body_x)
     * - Assumption: β ≈ 0 for coordinated flight
     * - Synthetic measurement: β_measured = 0 (perfect coordination)
     * - Body velocity from: v_body = DCM * (v_ned - wind_ned)
     * 
     * When Assumption Valid:
     * - Fixed-wing aircraft in coordinated flight (ball centered)
     * - Multicopters with symmetric frame
     * - Vehicle not experiencing significant crosswind
     * - No asymmetric drag or thrust
     * 
     * When Assumption Invalid:
     * - Aggressive maneuvering with sideslip
     * - Asymmetric vehicle configuration
     * - Significant crosswind relative to airspeed
     * - Damaged aircraft with asymmetric drag
     * 
     * State Updates:
     * - Primary: Wind velocity NE (states 22-23)
     * - Secondary: Velocity NED (states 3-5)
     * - Provides orthogonal information to airspeed fusion
     * 
     * Fusion Process:
     * 1. Calculate body-frame velocity from state
     * 2. Compute predicted sideslip (should be near zero)
     * 3. Innovation = 0 - predicted_sideslip
     * 4. Linearize measurement model (Jacobian)
     * 5. Apply Kalman update with synthetic measurement noise
     * 
     * Measurement Noise:
     * - Assigned based on expected sideslip variation
     * - Larger noise for less symmetric vehicles
     * - Smaller noise for highly symmetric aircraft
     * - Tuned to prevent overconstraining wind estimates
     * 
     * Innovation Checks:
     * - Normalized innovation squared: innov² / variance < threshold
     * - Failed checks set faultStatus.bad_sideslip flag
     * - Large innovations indicate assumption violation
     * 
     * Complementary to Airspeed:
     * - Airspeed measures total air-relative velocity magnitude
     * - Sideslip constrains lateral wind component
     * - Together provide full wind velocity observability
     * - Improves wind estimation convergence time
     * 
     * @note Only applied when assume_zero_sideslip() returns true
     * @note Multicopters typically use this constraint
     * @note Fixed-wing depends on vehicle symmetry and flight condition
     * 
     * @warning Invalid for vehicles with intentional sideslip
     * @warning Can degrade estimates if assumption violated
     * @warning Disable for asymmetric vehicles or aggressive maneuvering
     * 
     * @see FuseAirspeed() for complementary TAS fusion
     * @see assume_zero_sideslip() for applicability check
     * @see faultStatus.bad_sideslip for consistency check failures
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:542
     */
    void FuseSideslip();

    /**
     * @brief Zero specified range of rows in the state covariance matrix
     * 
     * @details Sets a range of rows in the covariance matrix to zero. Used during state
     *          resets to remove correlations between reset states and other states,
     *          preventing reset from affecting uncorrelated state estimates.
     * 
     * @param[in,out] covMat Covariance matrix to modify (typically P)
     * @param[in] first First row index to zero (0-23, inclusive)
     * @param[in] last Last row index to zero (0-23, inclusive)
     * 
     * Operation:
     * For i = first to last:
     *   For j = 0 to 23:
     *     covMat[i][j] = 0
     * 
     * Use Cases:
     * - State resets: Zero rows for reset states to remove covariances
     * - Gyro bias reset: Zero rows 9-11
     * - Magnetic field reset: Zero rows 16-21
     * - Wind reset: Zero rows 22-23
     * - Combined with zeroCols() for symmetric reset
     * 
     * Typical Pattern:
     * 1. zeroRows(P, first, last)  // Zero row range
     * 2. zeroCols(P, first, last)  // Zero corresponding columns
     * 3. Set diagonal P[i][i] to reset variance for i in [first, last]
     * 4. Result: Reset states decorrelated, diagonal variance preserved
     * 
     * @note Always paired with zeroCols() to maintain symmetry
     * @note Diagonal elements typically set after zeroing rows/cols
     * @note Used during magnetic field resets, bias resets, wind resets
     * 
     * @warning Zeroing rows without zeroing columns breaks symmetry
     * @warning Must restore diagonal variances after zeroing
     * 
     * @see zeroCols() for column zeroing (symmetric operation)
     * @see ForceSymmetry() to restore symmetry if needed
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:545
     */
    void zeroRows(Matrix24 &covMat, uint8_t first, uint8_t last);

    /**
     * @brief Zero specified range of columns in the state covariance matrix
     * 
     * @details Sets a range of columns in the covariance matrix to zero. Used in combination
     *          with zeroRows() during state resets to completely decorrelate reset states
     *          from the rest of the state vector.
     * 
     * @param[in,out] covMat Covariance matrix to modify (typically P)
     * @param[in] first First column index to zero (0-23, inclusive)
     * @param[in] last Last column index to zero (0-23, inclusive)
     * 
     * Operation:
     * For i = 0 to 23:
     *   For j = first to last:
     *     covMat[i][j] = 0
     * 
     * Use Cases:
     * - State resets: Zero columns for reset states to remove covariances
     * - Always paired with zeroRows() for symmetric reset
     * - Gyro bias reset: Zero columns 9-11
     * - Magnetic field reset: Zero columns 16-21
     * - Wind reset: Zero columns 22-23
     * 
     * Symmetric Reset Pattern:
     * ```
     * // Example: Reset magnetic field states (16-21)
     * zeroRows(P, 16, 21);    // Zero rows 16-21
     * zeroCols(P, 16, 21);    // Zero columns 16-21
     * // Now restore diagonal variances
     * for (uint8_t i=16; i<=21; i++) {
     *     P[i][i] = initial_variance;
     * }
     * ```
     * Result: Magnetic field states decorrelated from all other states
     * 
     * Why Symmetric Reset Needed:
     * - Covariance must be symmetric by definition
     * - Rows represent covariance of state i with all states
     * - Columns represent covariance of all states with state j
     * - Both must be zeroed to fully decorrelate
     * 
     * @note Always call after zeroRows() with same indices
     * @note Maintains matrix symmetry when paired with zeroRows()
     * @note Diagonal elements typically restored after zeroing
     * 
     * @warning Zeroing columns without zeroing rows breaks symmetry
     * @warning Must restore diagonal variances after zeroing
     * 
     * @see zeroRows() for row zeroing (symmetric operation)
     * @see ForceSymmetry() to enforce symmetry
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:548
     */
    void zeroCols(Matrix24 &covMat, uint8_t first, uint8_t last);

    /**
     * @brief Reset the stored output history to current data
     * 
     * @details Resets the output state buffer to current EKF state estimates. Used after
     *          state resets or when output observer tracking error becomes excessive.
     *          Prevents output discontinuities by reinitializing the complementary filter.
     * 
     * Output Observer Architecture:
     * - EKF runs at fusion time horizon (delayed for buffering)
     * - Output observer extrapolates EKF states to current time
     * - Uses IMU data from fusion horizon to present
     * - Maintains smooth output despite EKF delays
     * 
     * What Gets Reset:
     * - storedOutput buffer: Clears output history
     * - outputDataNew: Set to current state (quaternion, velocity, position)
     * - outputDataDelayed: Set to current state at fusion horizon
     * - Output tracking errors: velErrintegral, posErrintegral zeroed
     * - delAngCorrection: Delta angle correction term zeroed
     * 
     * When Called:
     * - After large position resets (GPS glitch recovery)
     * - After yaw resets (magnetometer anomaly recovery)
     * - When output tracking error exceeds thresholds
     * - During filter reinitialization
     * - After extended GPS outages
     * 
     * Output Observer Operation:
     * Normal: EKF state → complementary filter → smooth output
     * After reset: Current state → output buffer → immediate valid output
     * 
     * Impact on Vehicle:
     * - Prevents output jumps that could destabilize control
     * - Ensures smooth state estimate transitions
     * - Maintains control system stability through resets
     * 
     * @note Part of output complementary filter reset sequence
     * @note May be combined with StoreQuatReset() for attitude resets
     * @note Critical for smooth position/velocity output during resets
     * 
     * @warning Causes brief loss of output smoothing
     * @warning Should only be called during necessary state resets
     * 
     * @see StoreQuatReset() for quaternion-only reset
     * @see calcOutputStates() for normal output observer operation
     * @see storedOutput buffer for output history
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:551
     */
    void StoreOutputReset(void);

    /**
     * @brief Reset the stored output quaternion history to current EKF state
     * 
     * @details Resets only the quaternion portion of the output buffer to the current EKF
     *          attitude estimate. Used after yaw resets to prevent attitude output jumps
     *          while preserving position/velocity output continuity.
     * 
     * Quaternion-Only Reset:
     * - Resets: outputDataNew.quat and outputDataDelayed.quat
     * - Preserves: outputDataNew.velocity, position (unchanged)
     * - Clears: delAngCorrection delta angle tracking
     * - Maintains: Position/velocity output observer state
     * 
     * Use Cases:
     * - Yaw resets from magnetometer (magnetic anomaly recovery)
     * - Yaw resets from GPS course (GPS velocity yaw alignment)
     * - Yaw resets from EKFGSF (emergency yaw recovery)
     * - Compass failure detection and switch
     * 
     * Why Quaternion-Only:
     * - Yaw resets don't affect position/velocity states
     * - Position/velocity output observer can continue smoothly
     * - Only attitude output needs reinitialization
     * - Minimizes output disturbance
     * 
     * Reset Sequence:
     * 1. EKF quaternion reset: resetQuatStateYawOnly()
     * 2. Output quaternion reset: StoreQuatReset()
     * 3. Record reset: recordYawReset() for external tracking
     * 4. Result: Smooth attitude transition, continuous position/velocity
     * 
     * Output Observer Impact:
     * - Attitude output immediately reflects new yaw
     * - No attitude tracking error accumulation after reset
     * - Delta angle correction cleared (fresh start)
     * - Position/velocity observer unaffected
     * 
     * @note Called during yaw-only state resets
     * @note More selective than StoreOutputReset() (full reset)
     * @note Preserves position/velocity output smoothness
     * 
     * @warning Causes brief attitude output discontinuity (by design)
     * @warning Should match timing of EKF quaternion state reset
     * 
     * @see StoreOutputReset() for full output reset
     * @see StoreQuatRotate() for incremental quaternion rotation
     * @see resetQuatStateYawOnly() for EKF state yaw reset
     * @see recordYawReset() for reset angle recording
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:554
     */
    void StoreQuatReset(void);

    /**
     * @brief Rotate the stored output quaternion history through a quaternion rotation
     * 
     * @details Applies an incremental quaternion rotation to the output quaternion buffer.
     *          Used to smoothly adjust output attitude when EKF applies small attitude
     *          corrections without full reset, maintaining output observer continuity.
     * 
     * @param[in] deltaQuat Incremental rotation quaternion to apply
     * 
     * Quaternion Rotation:
     * - Updates: q_new = q_old ⊗ deltaQuat (quaternion multiplication)
     * - Applied to: All quaternions in storedOutput buffer
     * - Applied to: outputDataNew.quat and outputDataDelayed.quat
     * - Preserves: Quaternion normalization (unit length)
     * 
     * Use Cases:
     * - Small attitude corrections from measurements
     * - Incremental yaw updates (< 5° typically)
     * - Attitude adjustments without full reset
     * - Smooth integration of external attitude references
     * 
     * Quaternion Multiplication:
     * - deltaQuat represents rotation: (axis, angle)
     * - Compound rotation: q_new = q_old ⊗ deltaQuat
     * - Order matters: right-multiplication for body-frame rotation
     * - Preserves normalization if both inputs normalized
     * 
     * When Used vs StoreQuatReset:
     * - StoreQuatRotate: Small corrections, smooth transition
     * - StoreQuatReset: Large corrections, immediate reset
     * - Threshold: Typically ~5° determines which method
     * 
     * Output Observer Continuity:
     * - Smooth attitude transition (no jump)
     * - Delta angle correction remains valid
     * - Position/velocity observer unaffected
     * - Control system sees smooth attitude evolution
     * 
     * Typical Workflow:
     * 1. EKF detects small attitude error from measurement
     * 2. Apply correction to EKF state quaternion
     * 3. Calculate deltaQuat = correction rotation
     * 4. Apply to output: StoreQuatRotate(deltaQuat)
     * 5. Result: EKF and output observer stay synchronized
     * 
     * @note Used for small attitude corrections (< few degrees)
     * @note Maintains output smoothness better than reset
     * @note deltaQuat should be normalized (unit quaternion)
     * 
     * @warning Large rotations should use StoreQuatReset instead
     * @warning Accumulated rotations can introduce numerical error
     * 
     * @see StoreQuatReset() for large attitude resets
     * @see QuaternionF for quaternion type and operations
     * @see storedOutput buffer for output history
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:557
     */
    void StoreQuatRotate(const QuaternionF &deltaQuat);

    /**
     * @brief Calculate the NED earth spin vector in rad/sec
     * 
     * @details Calculates Earth's rotation rate vector expressed in the local NED frame.
     *          Required for compensating Coriolis effect in inertial navigation equations.
     *          Earth rotation affects gyro measurements and velocity integration.
     * 
     * @param[out] omega Earth rotation rate vector in NED frame (rad/s)
     * @param[in] latitude Vehicle latitude in degrees * 1e7 (GPS format)
     * 
     * Earth Rotation Rate:
     * - Magnitude: ωₑ = 7.2921150e-5 rad/s (constant)
     * - Direction: Along Earth's spin axis (North pole)
     * - Impact: Small but measurable with high-accuracy IMUs
     * 
     * NED Frame Components:
     * - omega.x (North): ωₑ * cos(latitude) [rad/s]
     * - omega.y (East): 0 [rad/s]
     * - omega.z (Down): -ωₑ * sin(latitude) [rad/s]
     * 
     * Latitude Effects:
     * - Equator (lat=0°): Maximum North component, zero Down
     * - Poles (lat=±90°): Zero North component, maximum Down
     * - Northern hemisphere: Positive Down component
     * - Southern hemisphere: Negative Down component
     * 
     * Coriolis Compensation:
     * - Transport rate: Accounts for frame rotation
     * - Applied in UpdateStrapdownEquationsNED()
     * - Corrects velocity integration for rotating reference frame
     * - Critical for long-duration missions (> 1 hour)
     * 
     * Typical Values:
     * - Latitude 0° (Equator): [7.29e-5, 0, 0] rad/s
     * - Latitude 45°: [5.16e-5, 0, -5.16e-5] rad/s
     * - Latitude 90° (North Pole): [0, 0, -7.29e-5] rad/s
     * 
     * @note Called during initialization and when latitude changes significantly
     * @note Stored in earthRateNED member variable
     * @note Effect is small but accumulates over time
     * 
     * @warning Neglecting Earth rate causes position drift (~1 m per hour)
     * @warning Important for high-accuracy navigation (survey, mapping)
     * 
     * @see UpdateStrapdownEquationsNED() where Earth rate is applied
     * @see earthRateNED member variable for storage
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:560
     */
    void calcEarthRateNED(Vector3F &omega, int32_t latitude) const;

    /**
     * @brief Initialize the covariance matrix
     * 
     * @details Sets initial values for the state covariance matrix P based on expected
     *          sensor accuracies, initial alignment uncertainty, and parameter settings.
     *          Determines filter confidence in initial state estimates.
     * 
     * Initial Covariance Structure:
     * - Diagonal elements: Initial state variances (uncertainty squared)
     * - Off-diagonal elements: Zero (states initially uncorrelated)
     * - Positive definite by construction
     * - Units: Squared state units
     * 
     * Initial Variance Values:
     * - Rotation error (0-2): Based on alignment method and sensor quality
     *   * Tilt: 0.1 rad² typical (accelerometer accuracy)
     *   * Yaw: 1.0 rad² typical (magnetometer/GPS velocity dependent)
     * - Velocity (3-5): Based on GPS accuracy or initial assumption
     *   * 10 m²/s² typical for GPS velocity
     * - Position (6-8): Based on GPS accuracy or initialization method
     *   * Horizontal: 25 m² typical (5m GPS accuracy)
     *   * Vertical: 25 m² typical (5m GPS/baro accuracy)
     * - Gyro bias (9-11): InitialGyroBiasUncertainty()²
     *   * Vehicle-specific, typically 0.01 rad²/s²
     * - Gyro scale (12-14): 1e-6 (0.1% scale factor error)
     * - Accel Z bias (15): 1.0 m²/s⁴ typical
     * - Earth mag field (16-18): 0.01 gauss²/1e6 (if learning)
     * - Body mag field (19-21): 0.01 gauss²/1e6 (if learning)
     * - Wind velocity (22-23): 25 m²/s² (5 m/s initial uncertainty)
     * 
     * Initialization Dependencies:
     * - Sensor availability (GPS, magnetometer, baro)
     * - Alignment mode (bootstrap vs in-flight)
     * - Parameter settings (sensor noise, process noise)
     * - Vehicle state (on ground vs in flight)
     * 
     * Impact on Filter Behavior:
     * - Large initial variance: Fast convergence, accepts measurements quickly
     * - Small initial variance: Slow convergence, rejects outliers better
     * - Trade-off: Convergence speed vs initial stability
     * 
     * Special Cases:
     * - In-flight initialization: Larger variances for unknown states
     * - Compass calibration active: Larger magnetic field variances
     * - GPS not available: Larger position/velocity variances
     * 
     * @note Called during InitialiseFilterBootstrap()
     * @note Different initialization for ground vs in-flight start
     * @note Tuned based on extensive flight testing
     * 
     * @warning Overly optimistic (small) values cause filter overconfidence
     * @warning Overly pessimistic (large) values cause slow convergence
     * 
     * @see InitialiseFilterBootstrap() which calls this during initialization
     * @see P covariance matrix member variable
     * @see InitialGyroBiasUncertainty() for gyro bias initial variance
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:563
     */
    void CovarianceInit();

    /**
     * @brief Helper function to read delta velocity from INS
     * 
     * @details Retrieves accumulated delta velocity (integrated acceleration) from the
     *          inertial navigation system for the specified IMU index. Returns false
     *          if data is unavailable or invalid.
     * 
     * @param[in] ins_index IMU/accelerometer index (0 to INS_MAX_INSTANCES-1)
     * @param[out] dVel Delta velocity vector in body frame (m/s)
     * @param[out] dVel_dt Time interval over which delta velocity accumulated (s)
     * @return true if delta velocity data successfully retrieved, false otherwise
     * 
     * Delta Velocity:
     * - Integrated accelerometer output over sample period
     * - Units: m/s (velocity change, not acceleration)
     * - Body frame: X=forward, Y=right, Z=down
     * - Includes gravity component (not removed at sensor)
     * 
     * Typical Sample Rates:
     * - High-rate IMUs: 1000-2000 Hz raw, accumulated to ~100-400 Hz
     * - Delta time: 0.0025s to 0.01s typical
     * - Accumulation reduces quantization noise
     * 
     * Data Validation:
     * - Checks INS data availability
     * - Verifies delta time is positive and reasonable
     * - Detects IMU failures or data gaps
     * - Returns false for invalid data
     * 
     * Multi-IMU Support:
     * - ins_index selects which IMU to read
     * - Supports primary/backup IMU redundancy
     * - EKF can switch IMUs if primary fails
     * - Inactive IMU biases tracked separately
     * 
     * Usage in EKF:
     * 1. Called by readIMUData() to acquire measurements
     * 2. Bias correction applied: correctDeltaVelocity()
     * 3. Used in UpdateStrapdownEquationsNED() for velocity integration
     * 4. Transformed from body frame to NED frame
     * 
     * @note Part of IMU data acquisition pipeline
     * @note Wrapper around AP_InertialSensor delta velocity API
     * @note Handles multi-IMU systems transparently
     * 
     * @warning Returns false on IMU failure - must check return value
     * @warning Delta time must be reasonable (0.001s to 0.1s typical)
     * 
     * @see readDeltaAngle() for companion delta angle reading
     * @see correctDeltaVelocity() for bias correction
     * @see readIMUData() which calls this function
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:566
     */
    bool readDeltaVelocity(uint8_t ins_index, Vector3F &dVel, ftype &dVel_dt);

    /**
     * @brief Helper function to read delta angle from INS
     * 
     * @details Retrieves accumulated delta angle (integrated gyro rate) from the
     *          inertial navigation system for the specified IMU index. Returns false
     *          if data is unavailable or invalid.
     * 
     * @param[in] ins_index IMU/gyro index (0 to INS_MAX_INSTANCES-1)
     * @param[out] dAng Delta angle vector in body frame (rad)
     * @param[out] dAng_dt Time interval over which delta angle accumulated (s)
     * @return true if delta angle data successfully retrieved, false otherwise
     * 
     * Delta Angle:
     * - Integrated gyroscope output over sample period
     * - Units: radians (angle change, not rate)
     * - Body frame: X=roll axis, Y=pitch axis, Z=yaw axis
     * - Right-hand rotation convention
     * 
     * Typical Sample Rates:
     * - High-rate gyros: 1000-2000 Hz raw, accumulated to ~100-400 Hz
     * - Delta time: 0.0025s to 0.01s typical
     * - Accumulation improves integration accuracy
     * 
     * Data Validation:
     * - Checks INS data availability for specified gyro
     * - Verifies delta time is positive and reasonable
     * - Detects gyro failures or data gaps
     * - Returns false for invalid/missing data
     * 
     * Multi-IMU/Gyro Support:
     * - ins_index selects which gyro to read
     * - Supports primary/backup gyro redundancy
     * - EKF can switch gyros if primary fails
     * - Inactive gyro biases tracked in inactiveBias[]
     * 
     * Usage in EKF:
     * 1. Called by readIMUData() to acquire measurements
     * 2. Bias/scale correction applied: correctDeltaAngle()
     * 3. Used in UpdateStrapdownEquationsNED() for attitude integration
     * 4. Quaternion rotated by corrected delta angle
     * 
     * Coning Correction:
     * - INS may apply coning compensation internally
     * - Improves attitude accuracy during rotation
     * - Accounts for rotation-induced cross-coupling
     * 
     * @note Part of IMU data acquisition pipeline
     * @note Wrapper around AP_InertialSensor delta angle API
     * @note Critical for attitude estimation accuracy
     * 
     * @warning Returns false on gyro failure - must check return value
     * @warning Delta time must be reasonable (0.001s to 0.1s typical)
     * 
     * @see readDeltaVelocity() for companion delta velocity reading
     * @see correctDeltaAngle() for bias and scale factor correction
     * @see readIMUData() which calls this function
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:567
     */
    bool readDeltaAngle(uint8_t ins_index, Vector3F &dAng, ftype &dAng_dt);

    /**
     * @brief Helper function to correct delta angle for bias and scale errors
     * 
     * @details Applies gyro bias and scale factor corrections to delta angle measurements.
     *          Essential for accurate attitude estimation as even small gyro errors
     *          accumulate rapidly in attitude integration.
     * 
     * @param[in,out] delAng Delta angle vector to correct, modified in place (rad)
     * @param[in] delAngDT Time interval for delta angle (s)
     * @param[in] gyro_index Index of gyro providing this measurement
     * 
     * Gyro Error Model:
     * - ω_true = (ω_measured - bias) / (1 + scale_error)
     * - bias: Additive offset (rad/s)
     * - scale_error: Multiplicative scale factor error (dimensionless, typically < 0.01)
     * 
     * Correction Process:
     * 1. Convert delta angle to rate: ω = delAng / delAngDT
     * 2. Apply bias correction: ω_corrected = ω - gyro_bias
     * 3. Apply scale correction: ω_corrected = ω_corrected / (1 + gyro_scale)
     * 4. Convert back to delta angle: delAng = ω_corrected * delAngDT
     * 
     * Bias Correction:
     * - Uses estimated gyro_bias from EKF state (states 9-11)
     * - If gyro_index matches active gyro: Use state estimate
     * - If gyro_index is inactive: Use inactiveBias[gyro_index].gyro_bias
     * - Critical for long-term attitude accuracy
     * 
     * Scale Factor Correction:
     * - Uses estimated gyro_scale from EKF state (states 12-14)
     * - Corrects for gain errors in gyro amplifier
     * - Typically small (< 1%) but observable over time
     * - Important for high rotation rates
     * 
     * Active vs Inactive Gyro:
     * - Active gyro: Corrections from EKF state (states 9-14)
     * - Inactive gyro: Corrections from inactiveBias[] tracking
     * - Allows seamless IMU switching
     * 
     * Impact on Attitude:
     * - Uncorrected bias: Attitude drift ~0.01 deg/s typical
     * - Uncorrected scale: Errors proportional to rotation rate
     * - Corrections reduce drift to sensor noise levels
     * 
     * @note Called by readIMUData() before attitude propagation
     * @note Corrections updated by EKF measurement updates
     * @note Essential for preventing attitude drift
     * 
     * @warning Incorrect bias causes attitude drift
     * @warning Scale errors become significant at high rotation rates
     * 
     * @see gyro_bias state elements (states 9-11)
     * @see gyro_scale state elements (states 12-14)
     * @see inactiveBias for inactive IMU bias tracking
     * @see readIMUData() which calls this function
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:570
     */
    void correctDeltaAngle(Vector3F &delAng, ftype delAngDT, uint8_t gyro_index);

    /**
     * @brief Helper function to correct delta velocity for bias errors
     * 
     * @details Applies accelerometer Z-axis bias correction to delta velocity measurements.
     *          EKF2 estimates only Z-axis (vertical) accelerometer bias as it's most
     *          observable and impacts altitude estimation significantly.
     * 
     * @param[in,out] delVel Delta velocity vector to correct, modified in place (m/s)
     * @param[in] delVelDT Time interval for delta velocity (s)
     * @param[in] accel_index Index of accelerometer providing this measurement
     * 
     * Accelerometer Error Model:
     * - a_true = a_measured - bias
     * - Only Z-axis bias estimated in EKF2 (state 15)
     * - X/Y biases unobservable without external velocity reference
     * 
     * Correction Process:
     * 1. Convert delta velocity to acceleration: a = delVel / delVelDT
     * 2. Apply Z-axis bias correction: a.z = a.z - accel_zbias
     * 3. Convert back to delta velocity: delVel.z = a.z * delVelDT
     * 4. X/Y components unchanged (biases not estimated)
     * 
     * Z-Axis Bias Correction:
     * - Uses estimated accel_zbias from EKF state (state 15)
     * - If accel_index matches active: Use state estimate
     * - If accel_index is inactive: Use inactiveBias[accel_index].accel_zbias
     * - Critical for altitude accuracy
     * 
     * Why Only Z-Axis:
     * - Z-axis observable from barometer height measurements
     * - X/Y biases compensated by GPS velocity corrections
     * - Estimating unobservable states causes filter issues
     * - Vertical bias has direct impact on altitude drift
     * 
     * Active vs Inactive Accelerometer:
     * - Active accel: Correction from EKF state (state 15)
     * - Inactive accel: Correction from inactiveBias[] tracking
     * - Enables IMU switching without bias discontinuity
     * 
     * Impact on Navigation:
     * - Uncorrected Z bias: Altitude drift ~0.1 m/s² typical
     * - 10 seconds integration: ~5 m altitude error
     * - Correction reduces drift to barometer noise levels
     * 
     * Typical Bias Values:
     * - Well-calibrated: < 0.05 m/s²
     * - Temperature-dependent: 0.1-0.5 m/s² variation
     * - Aging: Slow drift over months
     * 
     * @note Called by readIMUData() before velocity propagation
     * @note Only Z-axis corrected (X/Y unobservable)
     * @note Critical for altitude hold and landing
     * 
     * @warning Uncorrected Z bias causes altitude drift
     * @warning X/Y biases not estimated (use GPS corrections)
     * 
     * @see accel_zbias state element (state 15)
     * @see inactiveBias for inactive IMU bias tracking
     * @see readIMUData() which calls this function
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:571
     */
    void correctDeltaVelocity(Vector3F &delVel, ftype delVelDT, uint8_t accel_index);

    /**
     * @brief Update IMU delta angle and delta velocity measurements
     * 
     * @details Main IMU data acquisition function. Reads delta angles and delta velocities
     *          from the selected IMU, applies bias/scale corrections, and stores corrected
     *          measurements in the IMU buffer for fusion time horizon processing.
     * 
     * Data Acquisition Flow:
     * 1. Read delta angle from gyro: readDeltaAngle()
     * 2. Read delta velocity from accel: readDeltaVelocity()
     * 3. Apply gyro corrections: correctDeltaAngle()
     * 4. Apply accel corrections: correctDeltaVelocity()
     * 5. Store corrected data in storedIMU buffer
     * 6. Update imuDataNew with latest measurements
     * 
     * IMU Selection:
     * - Uses gyro_index_active for gyro selection
     * - Uses accel_index_active for accelerometer selection
     * - Supports mixed IMU configurations (gyro from IMU1, accel from IMU2)
     * - Automatically switches on IMU failure detection
     * 
     * Data Buffering:
     * - Stores measurements in storedIMU ring buffer
     * - Buffer length: imu_buffer_length samples
     * - Enables time-delayed fusion (fusion time horizon)
     * - Allows measurement reordering and interpolation
     * 
     * Downsampling:
     * - IMU may run faster than EKF rate (e.g., 400 Hz IMU, 100 Hz EKF)
     * - Accumulates IMU samples into imuDataDownSampledNew
     * - Reduces computational load while preserving accuracy
     * - Target rate: EKF_TARGET_DT = 0.01s (100 Hz)
     * 
     * Bad IMU Detection:
     * - Checks for NaN values in delta angles/velocities
     * - Checks for unreasonable delta times
     * - Sets badIMUdata flag if problems detected
     * - Triggers IMU switching or filter safeguards
     * 
     * Timing Management:
     * - Records imuSampleTime_ms timestamp
     * - Tracks delAngDT and delVelDT intervals
     * - Ensures consistent timing for integration
     * - Detects IMU data gaps
     * 
     * Inactive IMU Tracking:
     * - Calls learnInactiveBiases() to maintain bias estimates
     * - Ensures smooth transition when switching IMUs
     * - Tracks gyro bias, gyro scale, accel Z bias for all IMUs
     * 
     * @note Called every time new IMU data is available (typically 100-400 Hz)
     * @note Critical path for EKF - must be efficient
     * @note Corrected data used by UpdateStrapdownEquationsNED()
     * 
     * @warning Bad IMU data causes filter divergence
     * @warning Must handle IMU failures gracefully
     * @warning Timing consistency essential for accurate integration
     * 
     * @see readDeltaAngle() for gyro data acquisition
     * @see readDeltaVelocity() for accel data acquisition
     * @see correctDeltaAngle() for gyro corrections
     * @see correctDeltaVelocity() for accel corrections
     * @see storedIMU buffer for measurement storage
     * @see learnInactiveBiases() for inactive IMU tracking
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:574
     */
    void readIMUData();

    /**
     * @brief Update estimate of inactive bias states
     * 
     * @details Maintains bias estimates for IMUs that are not currently being used by this
     *          EKF core. Enables smooth IMU switching by keeping all IMU biases up-to-date
     *          even when not actively fusing their data.
     * 
     * Inactive IMU Tracking:
     * - Tracks gyro bias for inactive gyros
     * - Tracks gyro scale factors for inactive gyros
     * - Tracks accel Z bias for inactive accelerometers
     * - Stored in inactiveBias[INS_MAX_INSTANCES] array
     * 
     * Why Track Inactive IMUs:
     * - Enables instant IMU switching without bias discontinuity
     * - IMU failure → switch → use pre-tracked biases immediately
     * - Prevents attitude/position jumps during IMU failover
     * - Maintains redundant sensor readiness
     * 
     * Learning Algorithm:
     * - Slowly converges inactive biases toward active bias estimates
     * - Time constant: Slow enough to prevent noise amplification
     * - Fast enough to track temperature-dependent drift
     * - Weighted average: inactive += alpha * (active - inactive)
     * 
     * Bias Propagation:
     * 1. Read active IMU bias estimates from EKF states
     * 2. For each inactive IMU:
     *    a. Apply low-pass filter toward active bias
     *    b. Account for temperature drift model
     *    c. Limit rate of change
     * 3. Store updated biases in inactiveBias[] array
     * 
     * Multi-Core Coordination:
     * - Different EKF cores may use different IMUs
     * - Each core tracks biases for IMUs it's not using
     * - Shared bias information across cores possible
     * - Enables best IMU selection per core
     * 
     * Temperature Compensation:
     * - IMU biases drift with temperature
     * - Learning tracks temperature-induced changes
     * - Maintains accuracy across flight envelope
     * - Critical for long flights with large temperature swings
     * 
     * Update Rate:
     * - Called every EKF update cycle
     * - Slow time constant (seconds to minutes)
     * - Prevents rapid bias changes from transients
     * 
     * @note Called by readIMUData() each cycle
     * @note Minimal computational cost (simple averaging)
     * @note Essential for IMU redundancy
     * 
     * @warning Inactive bias tracking disabled → IMU switching causes jumps
     * @warning Too fast convergence → noise coupling between IMUs
     * @warning Too slow convergence → outdated biases on IMU switch
     * 
     * @see inactiveBias array for storage
     * @see gyro_bias state (states 9-11) for active bias
     * @see accel_zbias state (state 15) for active accel bias
     * @see readIMUData() which calls this function
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:577
     */
    void learnInactiveBiases();

    /**
     * @brief Check for new valid GPS data and update stored measurement if available
     * 
     * @details Polls the GPS system for new measurements, validates data quality, performs
     *          GPS quality checks, and stores valid measurements in the GPS buffer for
     *          time-delayed fusion at the fusion time horizon.
     * 
     * GPS Data Acquisition:
     * - Reads position (latitude, longitude, height)
     * - Reads velocity (North, East, Down in NED frame)
     * - Reads accuracy estimates (position, velocity, height)
     * - Reads number of satellites and HDOP
     * - Records timestamp for fusion time alignment
     * 
     * Data Validation:
     * - Checks GPS fix type (3D fix required for position)
     * - Validates number of satellites (> minimum threshold)
     * - Validates HDOP (< maximum threshold)
     * - Checks accuracy estimates against limits
     * - Detects GPS glitches and discontinuities
     * 
     * GPS Quality Checks:
     * - Pre-flight checks: calcGpsGoodToAlign()
     * - In-flight checks: calcGpsGoodForFlight()
     * - Horizontal position drift check
     * - Vertical velocity consistency check
     * - Speed accuracy check
     * 
     * Antenna Position Correction:
     * - Calls CorrectGPSForAntennaOffset()
     * - Accounts for GPS antenna offset from vehicle CG
     * - Corrects for vehicle attitude (rotation)
     * - Critical for accurate position when banking/pitching
     * 
     * GPS Buffering:
     * - Stores measurements in storedGPS ring buffer
     * - Buffer length: OBS_BUFFER_LENGTH samples
     * - Enables fusion time horizon processing
     * - Handles GPS update rate (typically 5-10 Hz)
     * 
     * Multi-GPS Support:
     * - Reads from selected GPS instance (sensor_idx)
     * - Can switch GPS on quality degradation
     * - Tracks last_gps_idx for GPS switching detection
     * - Blends multiple GPS if configured
     * 
     * GPS Data Flow:
     * 1. Check for new GPS data availability
     * 2. Read raw GPS position and velocity
     * 3. Validate fix type and accuracy
     * 4. Correct for antenna offset
     * 5. Perform quality checks
     * 6. Store in buffer with timestamp
     * 7. Set gpsDataToFuse flag if valid
     * 
     * Timing Management:
     * - Records lastTimeGpsReceived_ms
     * - Detects GPS timeout conditions
     * - Handles GPS data gaps gracefully
     * 
     * @note Called every EKF update cycle (100 Hz typical)
     * @note GPS typically updates at 5-10 Hz
     * @note Essential for absolute position reference
     * 
     * @warning Poor GPS quality causes position drift
     * @warning GPS glitches must be detected and rejected
     * @warning Antenna offset errors cause position errors in turns
     * 
     * @see storedGPS buffer for measurement storage
     * @see CorrectGPSForAntennaOffset() for antenna correction
     * @see calcGpsGoodToAlign() for pre-flight quality check
     * @see calcGpsGoodForFlight() for in-flight quality check
     * @see SelectVelPosFusion() which uses GPS data
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:580
     */
    void readGpsData();

    /**
     * @brief Check for new altitude measurement data and update stored measurement if available
     * 
     * @details Reads barometric pressure sensor data, converts to altitude, and stores
     *          measurements in the barometer buffer for height fusion. Barometer provides
     *          primary altitude reference for most flight phases.
     * 
     * Barometer Data Acquisition:
     * - Reads barometric pressure from AP_Baro
     * - Converts pressure to altitude (pressure altitude)
     * - Applies temperature compensation
     * - Records timestamp for fusion alignment
     * 
     * Pressure to Altitude Conversion:
     * - Uses standard atmosphere model
     * - Altitude relative to reference pressure
     * - Accounts for temperature lapse rate
     * - Typical accuracy: 0.5-1.0 m in stable conditions
     * 
     * Barometer Buffering:
     * - Stores measurements in storedBaro ring buffer
     * - Buffer length: OBS_BUFFER_LENGTH samples
     * - Enables fusion time horizon processing
     * - Handles baro update rate (typically 20-50 Hz)
     * 
     * Barometer Selection:
     * - Supports multiple barometer instances
     * - Selects primary barometer based on health
     * - Can switch barometers on failure detection
     * - Uses barometer closest to vehicle CG if available
     * 
     * Data Validation:
     * - Checks for reasonable altitude values
     * - Detects pressure sensor failures
     * - Validates rate of change (climb rate limits)
     * - Records lastBaroReceived_ms for timeout detection
     * 
     * Ground Effect Compensation:
     * - Barometer affected by rotor downwash near ground
     * - Pressure disturbance causes altitude errors
     * - Filter applies ground effect compensation
     * - meaHgtAtTakeOff recorded for reference
     * 
     * Barometer Noise Characteristics:
     * - Short-term noise: ~0.1-0.3 m RMS
     * - Long-term drift: Weather-dependent, ~1 m/hour
     * - Turbulence effects: Increased noise in rough air
     * - Temperature effects: Compensation applied
     * 
     * Height Fusion Priority:
     * - Barometer: Primary altitude source
     * - Range finder: Close to ground (< 10 m typically)
     * - GPS: Backup when baro unreliable
     * - selectHeightForFusion() determines source
     * 
     * Baro Data Flow:
     * 1. Check for new barometer data
     * 2. Read pressure measurement
     * 3. Convert to altitude
     * 4. Validate measurement
     * 5. Store in buffer with timestamp
     * 6. Set baroDataToFuse flag if valid
     * 
     * @note Called every EKF update cycle (100 Hz typical)
     * @note Barometer updates at 20-50 Hz typically
     * @note Primary altitude reference for navigation
     * 
     * @warning Barometer affected by weather (pressure changes)
     * @warning Ground effect causes altitude errors near ground
     * @warning Temperature changes cause drift
     * 
     * @see storedBaro buffer for measurement storage
     * @see selectHeightForFusion() for height source selection
     * @see FuseVelPosNED() which fuses barometer height
     * @see meaHgtAtTakeOff for ground effect reference
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:583
     */
    void readBaroData();

    /**
     * @brief Check for new magnetometer data and update store measurements if available
     * 
     * @details Reads 3-axis magnetometer (compass) measurements, validates data quality,
     *          and stores valid measurements in the magnetometer buffer for yaw fusion
     *          and magnetic field state updates.
     * 
     * Magnetometer Data Acquisition:
     * - Reads magnetic field vector in body frame (X, Y, Z)
     * - Units: milligauss (mGauss)
     * - Applies calibration (offsets, scale, rotation matrix)
     * - Records timestamp for fusion alignment
     * 
     * Data Validation:
     * - Checks for reasonable field magnitude
     * - Validates against expected Earth field (~500 mGauss)
     * - Detects magnetic anomalies (ferrous objects nearby)
     * - Checks consistency across multiple measurements
     * 
     * Magnetometer Selection:
     * - Supports multiple magnetometer instances
     * - magSelectIndex determines active compass
     * - Can switch compasses on failure: tryChangeCompass()
     * - Prioritizes external compasses (less interference)
     * 
     * Magnetometer Buffering:
     * - Stores measurements in storedMag ring buffer
     * - Buffer length: OBS_BUFFER_LENGTH samples
     * - Enables fusion time horizon processing
     * - Handles mag update rate (typically 50-100 Hz)
     * 
     * Compass Calibration:
     * - Offsets: Hard iron compensation (body_magfield states)
     * - Scale factors: Soft iron compensation
     * - Rotation matrix: Alignment to body frame
     * - Motor compensation: Electric current effects
     * 
     * Magnetic Interference Detection:
     * - Monitors field magnitude consistency
     * - Detects motor/ESC interference patterns
     * - Identifies ferrous metal nearby
     * - Sets consistentMagData flag
     * 
     * Yaw Observability:
     * - Magnetometer primary yaw reference
     * - Fused with GPS velocity yaw when available
     * - Critical for yaw accuracy without GPS
     * - Magnetic declination applied (true north correction)
     * 
     * Compass Health Monitoring:
     * - Tracks lastMagRead_ms for timeout detection
     * - Innovation consistency checking
     * - Sets magHealth flag
     * - Triggers compass switching if needed
     * 
     * Mag Data Flow:
     * 1. Check for new magnetometer data
     * 2. Read 3-axis magnetic field
     * 3. Validate field magnitude and consistency
     * 4. Check for magnetic interference
     * 5. Store in buffer with timestamp
     * 6. Set magDataToFuse flag if valid
     * 7. Update compass health status
     * 
     * Magnetic Anomaly Handling:
     * - Detects large field deviations
     * - May trigger yaw reset if persistent
     * - magYawAnomallyCount tracks resets
     * - Limited resets per flight (MAG_ANOMALY_RESET_MAX)
     * 
     * @note Called every EKF update cycle (100 Hz typical)
     * @note Magnetometer updates at 50-100 Hz typically
     * @note Primary yaw reference, critical for heading
     * 
     * @warning Magnetic interference causes yaw errors
     * @warning Metal structures distort magnetic field
     * @warning Motor/ESC currents create interference
     * @warning Compass must be calibrated regularly
     * 
     * @see storedMag buffer for measurement storage
     * @see tryChangeCompass() for compass switching
     * @see SelectMagFusion() which schedules magnetometer fusion
     * @see FuseMagnetometer() which fuses mag measurements
     * @see magSelectIndex for active compass selection
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:586
     */
    void readMagData();

    /**
     * @brief Try changing compasses on compass failure or timeout
     * 
     * @details Attempts to switch to an alternative magnetometer when the current compass
     *          fails health checks or times out. Implements compass redundancy for
     *          continued yaw estimation during compass failures.
     * 
     * Compass Failure Detection:
     * - Innovation consistency test failures
     * - Magnetometer timeout (no new data)
     * - Excessive magnetic interference detected
     * - Field magnitude out of range
     * - Persistent yaw innovation errors
     * 
     * Switching Criteria:
     * - Current compass unhealthy for extended period
     * - Alternative compass available and healthy
     * - Alternative compass passes initial validation
     * - System allows compass switching (not inhibited)
     * 
     * Switching Process:
     * 1. Detect current compass failure condition
     * 2. Identify available alternative compasses
     * 3. Check health of alternative compasses
     * 4. Select best alternative based on:
     *    - Recent innovation test performance
     *    - External vs internal priority
     *    - Calibration status
     * 5. Update magSelectIndex to new compass
     * 6. Reset magnetic field states if necessary
     * 7. Log compass switch event
     * 
     * Compass Priority:
     * - External compasses preferred (less interference)
     * - Recently calibrated compasses preferred
     * - Compasses with better recent health preferred
     * - Can be configured via parameters
     * 
     * State Impact:
     * - May trigger magnetic field state reset
     * - Body magnetic field offsets may change
     * - Yaw covariance may be increased temporarily
     * - allMagSensorsFailed flag if no good compass
     * 
     * Switching Limitations:
     * - Limited switches per flight (prevent oscillation)
     * - Minimum time between switches (hysteresis)
     * - Won't switch during critical flight phases
     * - Requires alternative compass to be significantly better
     * 
     * Fallback Behavior:
     * - If all compasses fail: allMagSensorsFailed = true
     * - Filter continues with GPS velocity yaw only
     * - Reduced yaw accuracy without compass
     * - May inhibit certain flight modes
     * 
     * @note Called by readMagData() when compass health degrades
     * @note Provides compass redundancy for safety
     * @note Prevents single compass failure from causing incident
     * 
     * @warning Excessive compass switching indicates interference problem
     * @warning All compass failure requires GPS for any yaw reference
     * @warning Compass switching may cause brief yaw transients
     * 
     * @see readMagData() which calls this function
     * @see magSelectIndex for current compass selection
     * @see magHealth for compass health status
     * @see allMagSensorsFailed flag
     * @see consistentMagData for magnetic consistency status
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:589
     */
    void tryChangeCompass(void);

    /**
     * @brief Check for new airspeed data and update stored measurements if available
     * 
     * @details Reads airspeed sensor (pitot tube) measurements, validates data, and stores
     *          in the airspeed buffer for true airspeed fusion. Airspeed fusion improves
     *          wind estimation and enables synthetic sideslip fusion for fixed-wing aircraft.
     * 
     * Airspeed Data Acquisition:
     * - Reads differential pressure from pitot tube
     * - Converts to indicated airspeed (IAS)
     * - Converts to true airspeed (TAS) using air density
     * - Records timestamp for fusion alignment
     * 
     * Airspeed Measurement:
     * - Differential pressure: Dynamic pressure from pitot tube
     * - Temperature correction: Air density compensation
     * - Altitude correction: Pressure altitude factor
     * - Units: m/s (true airspeed in body X-axis direction)
     * 
     * Data Validation:
     * - Checks for reasonable airspeed values (> 0, < max)
     * - Validates rate of change (acceleration limits)
     * - Detects pitot tube blockage (zero or stuck reading)
     * - Checks sensor health status
     * - Records timeTasReceived_ms for timeout detection
     * 
     * Airspeed Buffering:
     * - Stores measurements in storedTAS ring buffer
     * - Buffer length: OBS_BUFFER_LENGTH samples
     * - Enables fusion time horizon processing
     * - Handles airspeed update rate (typically 10-50 Hz)
     * 
     * Use Airspeed Decision:
     * - useAirspeed() determines if sensor should be used
     * - Requires minimum airspeed threshold
     * - Disabled if sensor unhealthy
     * - Typically enabled only for fixed-wing
     * - Parameter configured: ARSPD_USE
     * 
     * True Airspeed vs Ground Speed:
     * - TAS: Speed relative to surrounding air
     * - Ground speed: GPS velocity magnitude
     * - Difference: Wind velocity
     * - TAS fusion improves wind state estimation
     * 
     * Wind Estimation:
     * - Airspeed + attitude = air velocity vector (body frame)
     * - GPS velocity = ground velocity vector (NED frame)
     * - Difference = wind velocity (states 22-23)
     * - Enables accurate wind compensation
     * 
     * Synthetic Sideslip:
     * - Assumes zero sideslip (coordinated flight)
     * - Airspeed + sideslip constraint = improved wind estimate
     * - Reduces lateral velocity errors
     * - Important for fixed-wing accuracy
     * 
     * Airspeed Data Flow:
     * 1. Check for new airspeed data
     * 2. Read differential pressure
     * 3. Convert to TAS (temperature/altitude corrected)
     * 4. Validate measurement
     * 5. Store in buffer with timestamp
     * 6. Set tasDataToFuse flag if valid
     * 
     * @note Called every EKF update cycle (100 Hz typical)
     * @note Airspeed updates at 10-50 Hz typically
     * @note Important for fixed-wing wind estimation
     * @note Optional for multicopters (usually not used)
     * 
     * @warning Pitot tube blockage causes airspeed errors
     * @warning Ice formation blocks pitot tube in cold conditions
     * @warning Turbulence causes airspeed fluctuations
     * 
     * @see storedTAS buffer for measurement storage
     * @see useAirspeed() for airspeed usage decision
     * @see SelectTasFusion() which schedules airspeed fusion
     * @see FuseAirspeed() which fuses airspeed measurements
     * @see wind_vel states (states 22-23)
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:592
     */
    void readAirSpdData();

    /**
     * @brief Check for new range beacon data and update stored measurements if available
     * 
     * @details Reads range measurements from positioning beacons at known locations, validates
     *          data quality, and stores measurements for beacon-based position estimation.
     *          Enables GPS-independent positioning indoors or in GPS-denied environments.
     * 
     * Range Beacon System:
     * - Multiple beacons at surveyed locations
     * - Each beacon transmits unique ID
     * - Measures range (distance) to vehicle
     * - Typical accuracy: 0.1-0.5 m per range
     * - Position from trilateration (3+ beacons)
     * 
     * Beacon Data Acquisition:
     * - Reads range measurement to each beacon
     * - Reads beacon ID and NED position
     * - Reads range measurement error estimate
     * - Records timestamp for fusion alignment
     * 
     * Data Validation:
     * - Checks beacon ID is valid and configured
     * - Validates range is within reasonable limits
     * - Checks for multi-path errors
     * - Validates beacon geometry (not coplanar)
     * - Detects beacon signal loss
     * 
     * Beacon Buffering:
     * - Stores measurements in storedRangeBeacon buffer
     * - Buffer length: OBS_BUFFER_LENGTH samples
     * - One measurement per beacon per cycle
     * - Tracks lastTimeRngBcn_ms[beacon_ID] per beacon
     * 
     * Beacon System Initialization:
     * - Requires 3+ beacons for 3D position
     * - Initial position from beacon trilateration
     * - Alignment uses FuseRngBcnStatic()
     * - rngBcnAlignmentStarted/Completed flags track progress
     * 
     * Range Beacon Positioning:
     * - Beacon positions in NED frame (beacon_posNED)
     * - Vehicle position estimated from ranges
     * - Vertical offset: bcnPosOffset
     * - Handles beacon constellation geometry
     * 
     * Beacon Fusion Modes:
     * - Static alignment: Vehicle stationary, learn position
     * - Dynamic fusion: Vehicle moving, range updates
     * - 3D vs 2D: Depends on beacon geometry
     * - Altitude offset estimation: bcnPosOffset
     * 
     * Beacon Health Monitoring:
     * - Tracks innovation test performance
     * - rngBcnHealth flag indicates beacon system health
     * - rngBcnTimeout if beacons lost too long
     * - rngBcnGoodToAlign for initialization quality
     * 
     * Beacon Data Flow:
     * 1. Check for new range beacon data
     * 2. Read range and beacon ID
     * 3. Look up beacon position from configuration
     * 4. Validate range measurement
     * 5. Store in buffer with timestamp
     * 6. Set rngBcnDataToFuse flag if valid
     * 7. Update beacon health status
     * 
     * Use Cases:
     * - Indoor navigation (GPS unavailable)
     * - Underground mining operations
     * - Tunnels and urban canyons
     * - Precise positioning in warehouses
     * - GPS-denied or GPS-jammed environments
     * 
     * @note Called every EKF update cycle (100 Hz typical)
     * @note Beacon updates at 1-10 Hz per beacon typically
     * @note Requires AP_Beacon library support
     * @note Conditional compilation: AP_BEACON_ENABLED
     * 
     * @warning Requires accurate beacon position survey
     * @warning Multi-path errors in reflective environments
     * @warning Poor geometry causes poor position accuracy
     * @warning Minimum 3 beacons required for 3D position
     * 
     * @see storedRangeBeacon buffer for measurement storage
     * @see SelectRngBcnFusion() which schedules beacon fusion
     * @see FuseRngBcn() which fuses beacon ranges
     * @see FuseRngBcnStatic() for static initialization
     * @see rngBcnGoodToAlign for alignment readiness
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:595
     */
    void readRngBcnData();

    /**
     * @brief Determine when to perform fusion of GPS position and velocity measurements
     * 
     * @details Manages the scheduling and triggering of GPS position and velocity fusion
     *          based on data availability, innovation consistency, and filter state.
     *          Controls when GPS measurements update the filter's position and velocity states.
     * 
     * Fusion Decision Criteria:
     * - GPS data available at fusion time horizon (gpsDataToFuse)
     * - GPS data passed quality checks (readyToUseGPS())
     * - Innovation consistency checks passed recently
     * - Not in GPS-denied mode
     * - Filter sufficiently initialized
     * 
     * Fusion Timing:
     * - GPS typically updates at 5-10 Hz
     * - EKF runs at 100 Hz (EKF_TARGET_DT)
     * - Fusion time horizon: Delayed by buffer length
     * - Synchronizes GPS time to IMU time
     * 
     * Selective Fusion Flags:
     * - fuseVelData: Enable velocity (NED) fusion this cycle
     * - fusePosData: Enable position (NE) fusion this cycle
     * - fuseHgtData: Enable height (D) fusion this cycle
     * - Can fuse velocity without position (better dynamics)
     * - Can fuse position without velocity (static positioning)
     * 
     * GPS Quality Gating:
     * - Checks gpsGoodToAlign before first fusion
     * - Monitors innovation consistency (velTestRatio, posTestRatio)
     * - Tracks time since last successful fusion
     * - Triggers GPS timeout if innovations fail too long
     * 
     * Timeout Handling:
     * - velTimeout: Velocity innovations failing, stop velocity fusion
     * - posTimeout: Position innovations failing, stop position fusion
     * - Allows filter to coast on IMU during GPS glitches
     * - Resets when GPS quality improves
     * 
     * Position Reset Detection:
     * - Large innovation after GPS timeout suggests position jump
     * - May trigger position reset to GPS
     * - ResetPosition() called if reset warranted
     * - Prevents filter divergence after GPS recovery
     * 
     * Height Source Selection:
     * - GPS height can be fusion source (activeHgtSource == HGT_SOURCE_GPS)
     * - Typically barometer preferred for height
     * - GPS height used as backup or when baro unavailable
     * - selectHeightForFusion() determines source
     * 
     * GPS Switching:
     * - Detects GPS receiver changes (sensor_idx)
     * - May reset position/velocity on GPS switch
     * - Handles multi-GPS configurations
     * - Ensures continuity across GPS changes
     * 
     * Fusion Rate Control:
     * - Don't fuse every GPS sample (over-weighting)
     * - Fusion rate matched to GPS update rate
     * - Prevents numerical issues from repeated fusion
     * - Maintains proper Kalman gain calculation
     * 
     * Innovation Monitoring:
     * - Records lastVelPassTime_ms, lastPosPassTime_ms
     * - Tracks innovation test ratios
     * - Updates filter status flags
     * - Logs innovation data for analysis
     * 
     * @note Called every EKF update cycle (100 Hz)
     * @note GPS fusion typically occurs at 5-10 Hz
     * @note Critical for absolute position/velocity accuracy
     * 
     * @warning GPS glitches must be detected and rejected
     * @warning Too aggressive fusion causes filter instability
     * @warning Too conservative fusion causes GPS position lag
     * 
     * @see FuseVelPosNED() which performs the actual fusion
     * @see readGpsData() which acquires GPS measurements
     * @see readyToUseGPS() for GPS readiness check
     * @see velTimeout, posTimeout, hgtTimeout flags
     * @see fuseVelData, fusePosData, fuseHgtData flags
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:598
     */
    void SelectVelPosFusion();

    /**
     * @brief Determine when to perform fusion of range measurements to beacons at known NED positions
     * 
     * @details Manages scheduling of range beacon fusion based on data availability, beacon
     *          geometry, and innovation consistency. Enables GPS-independent position estimation
     *          using trilateration from multiple ranging beacons.
     * 
     * Fusion Decision Criteria:
     * - Beacon range data available (rngBcnDataToFuse)
     * - Beacon system ready (readyToUseRangeBeacon())
     * - Sufficient beacons in view (>= 3 for 3D)
     * - Beacon geometry adequate (not coplanar)
     * - Innovation checks passed recently
     * 
     * Fusion Modes:
     * - Static alignment: Vehicle stationary, initial position
     *   * Uses FuseRngBcnStatic()
     *   * Builds initial position from multiple measurements
     *   * Completes when rngBcnAlignmentCompleted = true
     * - Dynamic fusion: Vehicle moving, continuous updates
     *   * Uses FuseRngBcn()
     *   * Sequential range fusion per beacon
     *   * Updates position states continuously
     * 
     * Beacon Selection:
     * - Processes one beacon per cycle (computational load)
     * - Cycles through available beacons
     * - lastRngBcnChecked tracks current beacon
     * - Prioritizes beacons with recent measurements
     * 
     * Beacon Geometry Validation:
     * - Checks beacon distribution (3D vs 2D geometry)
     * - Avoids coplanar beacons (poor vertical accuracy)
     * - Validates beacon height spread (maxBcnPosD - minBcnPosD)
     * - Requires adequate geometric dilution of precision
     * 
     * Initialization Sequence:
     * 1. Wait for vehicle stationary
     * 2. Collect ranges from multiple beacons
     * 3. Solve for initial position: FuseRngBcnStatic()
     * 4. Estimate vertical offset: CalcRangeBeaconPosDownOffset()
     * 5. Set rngBcnAlignmentCompleted = true
     * 6. Switch to dynamic fusion mode
     * 
     * Vertical Offset Handling:
     * - bcnPosOffset: Vertical offset between EKF origin and beacon datum
     * - bcnPosOffsetMax/Min: Dual hypothesis tracking
     * - Resolves vertical ambiguity in 2D beacon geometry
     * - Converges to correct offset over time
     * 
     * Innovation Gating:
     * - rngBcnTestRatio: Innovation consistency test
     * - Rejects outlier measurements
     * - Tracks beacon health per beacon
     * - rngBcnTimeout if all beacons fail too long
     * 
     * Fusion Timing:
     * - Beacon updates typically 1-10 Hz per beacon
     * - One beacon fused per EKF cycle
     * - Rotation through beacons for load distribution
     * - Maintains fresh beacon contributions
     * 
     * Beacon System Health:
     * - rngBcnHealth: Overall beacon system health
     * - Tracks lastRngBcnPassTime_ms for timeout
     * - Monitors innovation consistency
     * - Falls back to GPS/dead reckoning if beacons fail
     * 
     * @note Called every EKF update cycle (100 Hz)
     * @note One beacon fused per cycle (load distribution)
     * @note Requires AP_Beacon library support
     * @note Alternative to GPS for indoor/GPS-denied navigation
     * 
     * @warning Requires accurate beacon position survey (< 0.1 m error)
     * @warning Poor geometry causes poor position accuracy
     * @warning Multi-path errors in reflective environments
     * @warning Minimum 3 beacons required for 3D position
     * 
     * @see FuseRngBcn() for dynamic beacon fusion
     * @see FuseRngBcnStatic() for static alignment
     * @see CalcRangeBeaconPosDownOffset() for vertical offset
     * @see readRngBcnData() which acquires beacon measurements
     * @see readyToUseRangeBeacon() for beacon system readiness
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:601
     */
    void SelectRngBcnFusion();

    /**
     * @brief Determine when to perform fusion of magnetometer measurements
     * 
     * @details Manages scheduling of magnetometer fusion for yaw estimation and magnetic
     *          field state learning. Magnetometer provides primary yaw reference and enables
     *          accurate heading estimation without GPS velocity.
     * 
     * Fusion Decision Criteria:
     * - Magnetometer data available (magDataToFuse)
     * - Compass use enabled (use_compass())
     * - Not inhibiting magnetic field states (inhibitMagStates)
     * - Filter sufficiently initialized (tilt alignment complete)
     * - Innovation consistency checks passed
     * 
     * Fusion Timing:
     * - Magnetometer updates at 50-100 Hz typically
     * - Fusion performed at lower rate (computational efficiency)
     * - Sequential fusion: X, Y, Z axes on separate cycles
     * - Full 3-axis fusion every 3 cycles
     * - Spreads computational load evenly
     * 
     * Sequential Magnetometer Fusion:
     * - Cycle 1: Fuse X-axis magnetometer
     * - Cycle 2: Fuse Y-axis magnetometer
     * - Cycle 3: Fuse Z-axis magnetometer
     * - Reduces peak computation per cycle
     * - Maintains fusion rate for yaw observability
     * - mag_state structure holds intermediate calculations
     * 
     * Compass Usage Decision:
     * - use_compass() checks parameter settings
     * - Can disable compass if GPS velocity available
     * - Automatically disabled if all compasses failed
     * - Inhibited during magnetic anomalies
     * 
     * Magnetic Field State Learning:
     * - Earth magnetic field (states 16-18): NED components
     * - Body magnetic field (states 19-21): Offsets from interference
     * - Wind states (22-23): Often learned with magnetic states
     * - inhibitMagStates flag controls learning
     * 
     * Compass Alignment Sequence:
     * 1. Tilt alignment from accelerometer (tiltAlignComplete)
     * 2. Initial yaw from magnetometer + declination
     * 3. Magnetic field state initialization
     * 4. Continuous field learning and yaw updates
     * 5. Final in-flight alignment (finalInflightMagInit)
     * 
     * Magnetic Anomaly Handling:
     * - Detects large innovation residuals
     * - magYawResetTimer_ms tracks anomaly duration
     * - May trigger yaw reset if persistent
     * - magYawResetRequest flag indicates reset needed
     * - Limited anomaly resets per flight
     * 
     * Innovation Gating:
     * - magTestRatio: Per-axis innovation tests
     * - yawTestRatio: Yaw-specific innovation test
     * - Rejects outlier measurements
     * - magTimeout if innovations fail too long
     * 
     * Compass Health Monitoring:
     * - magHealth flag indicates compass health
     * - consistentMagData flag for consistency
     * - lastHealthyMagTime_ms tracks last good measurement
     * - May trigger compass switch: tryChangeCompass()
     * 
     * Magnetic Field Reset Conditions:
     * - Large magnetic anomaly detected
     * - Persistent yaw innovation failures
     * - User-requested reset (magStateResetRequest)
     * - After takeoff if needed (finalInflightMagInit)
     * - controlMagYawReset() manages reset logic
     * 
     * @note Called every EKF update cycle (100 Hz)
     * @note 3-axis fusion spread across 3 cycles
     * @note Primary yaw reference for navigation
     * @note Critical for accurate heading
     * 
     * @warning Magnetic interference causes yaw errors
     * @warning Hard/soft iron requires calibration
     * @warning Metal structures distort magnetic field
     * @warning Motor/ESC currents create time-varying interference
     * 
     * @see FuseMagnetometer() which performs the fusion
     * @see readMagData() which acquires compass measurements
     * @see use_compass() for compass usage decision
     * @see controlMagYawReset() for reset management
     * @see earth_magfield states (16-18), body_magfield states (19-21)
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:604
     */
    void SelectMagFusion();

    /**
     * @brief Determine when to perform fusion of true airspeed measurements
     * 
     * @details Manages scheduling of airspeed fusion for wind estimation and synthetic
     *          sideslip constraint. Airspeed fusion improves navigation accuracy for
     *          fixed-wing aircraft by providing air-relative velocity measurements.
     * 
     * Fusion Decision Criteria:
     * - Airspeed data available (tasDataToFuse)
     * - Airspeed sensor enabled (useAirspeed())
     * - Vehicle airborne (inFlight flag)
     * - Airspeed above minimum threshold
     * - Innovation consistency checks passed
     * 
     * Fusion Timing:
     * - Airspeed updates at 10-50 Hz typically
     * - Fusion rate limited to avoid over-weighting
     * - Minimum time between fusions: ~100 ms typical
     * - prevTasStep_ms tracks last fusion time
     * - Prevents excessive fusion rate
     * 
     * Airspeed Usage Decision:
     * - useAirspeed() checks configuration and health
     * - Requires minimum airspeed (typically > 5 m/s)
     * - Disabled on ground (airspeed unreliable)
     * - Typically only for fixed-wing vehicles
     * - Parameter controlled: ARSPD_USE
     * 
     * Wind State Estimation:
     * - Airspeed provides air-relative velocity
     * - GPS provides ground-relative velocity
     * - Difference enables wind estimation
     * - Wind states: wind_vel (states 22-23, 2D NE)
     * - Assumes horizontal wind (no vertical wind component)
     * 
     * Synthetic Sideslip:
     * - Coordinated flight assumption: zero sideslip
     * - Airspeed + sideslip constraint = tighter bounds
     * - SelectBetaFusion() schedules sideslip fusion
     * - Typically fused after airspeed fusion
     * - Improves lateral velocity accuracy
     * 
     * Innovation Gating:
     * - tasTestRatio: Airspeed innovation consistency
     * - Rejects outlier measurements
     * - tasTimeout if innovations fail too long
     * - Tracks lastTasPassTime_ms, lastTasFailTime_ms
     * 
     * Fusion Delay Management:
     * - airSpdFusionDelayed flag if fusion postponed
     * - Fusion time horizon alignment
     * - Handles airspeed buffer delays
     * - Synchronizes with IMU time
     * 
     * Wind Learning:
     * - Continuous wind state updates
     * - Wind magnitude and direction estimated
     * - Used for wind compensation in guidance
     * - Logged for analysis and reporting
     * 
     * Airspeed Sensor Health:
     * - Monitors innovation consistency
     * - Detects pitot tube blockage
     * - Checks for reasonable airspeed values
     * - May disable fusion if sensor fails
     * 
     * Fixed-Wing Specific:
     * - Airspeed critical for stall prevention
     * - Wind affects landing approach
     * - Enables accurate groundspeed prediction
     * - Improves waypoint arrival accuracy
     * 
     * @note Called every EKF update cycle (100 Hz)
     * @note Airspeed fusion typically 10-20 Hz
     * @note Important for fixed-wing wind estimation
     * @note Usually not used for multirotors
     * 
     * @warning Pitot tube blockage causes bad wind estimates
     * @warning Requires minimum airspeed for accuracy
     * @warning Turbulence increases airspeed noise
     * @warning Ice can block pitot tube (cold weather)
     * 
     * @see FuseAirspeed() which performs the fusion
     * @see readAirSpdData() which acquires airspeed measurements
     * @see useAirspeed() for airspeed usage decision
     * @see SelectBetaFusion() for synthetic sideslip
     * @see wind_vel states (22-23) for wind estimation
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:607
     */
    void SelectTasFusion();

    /**
     * @brief Determine when to perform fusion of synthetic sideslip measurements
     * 
     * @details Manages scheduling of synthetic sideslip constraint fusion. Assumes zero
     *          sideslip (coordinated flight) to improve lateral velocity and wind estimation
     *          for fixed-wing aircraft. This constraint tightens velocity bounds significantly.
     * 
     * Fusion Decision Criteria:
     * - Vehicle is fixed-wing or suitable for sideslip constraint
     * - Vehicle airborne (inFlight flag)
     * - Not performing aggressive maneuvers (manoeuvring flag)
     * - Airspeed available and reliable
     * - Sufficient time since last beta fusion
     * 
     * Sideslip Constraint:
     * - Assumes coordinated flight: sideslip angle β ≈ 0
     * - β = arctan(v_y / v_x) in body frame
     * - For coordinated flight: lateral velocity v_y ≈ 0
     * - Provides synthetic "measurement" of zero lateral velocity
     * 
     * Fusion Timing:
     * - Typically fused after airspeed fusion
     * - Rate limited: Minimum time between fusions
     * - prevBetaStep_ms tracks last fusion time
     * - Typical fusion rate: 5-10 Hz
     * - Balance: Constraint benefit vs over-constraint risk
     * 
     * Zero Sideslip Assumption:
     * - Valid for: Coordinated turns (proper rudder use)
     * - Invalid for: Slips, skids, crosswind landings
     * - assume_zero_sideslip() checks vehicle type
     * - Automatically disabled during aggressive maneuvers
     * - Parameter controlled or automatic
     * 
     * Wind Estimation Impact:
     * - Sideslip constraint improves wind estimates
     * - Couples airspeed + GPS + sideslip for wind
     * - Reduces wind estimation uncertainty
     * - Particularly helps with crosswind estimation
     * 
     * Lateral Velocity Constraint:
     * - Body-frame lateral velocity (v_y) constrained near zero
     * - Reduces lateral velocity drift errors
     * - Improves track-following accuracy
     * - Reduces position errors in turns
     * 
     * When to Disable:
     * - Aggressive maneuvers (high roll rates)
     * - Intentional slips (e.g., crosswind landing)
     * - Aerobatic flight
     * - Multicopters (no coordinated flight assumption)
     * - Thrust vectoring vehicles
     * 
     * Fusion Delay Management:
     * - sideSlipFusionDelayed flag if fusion postponed
     * - Ensures proper time alignment
     * - Handles computational load distribution
     * 
     * Innovation Monitoring:
     * - Monitors synthetic sideslip innovations
     * - Large innovations indicate uncoordinated flight
     * - May disable fusion if consistently violated
     * - Logs for analysis
     * 
     * Fixed-Wing Benefits:
     * - Tighter velocity bounds
     * - Better wind estimation
     * - Improved position accuracy
     * - Reduced drift in GPS-denied segments
     * 
     * Typical Scenarios:
     * - Cruise flight: Beta fusion active
     * - Coordinated turns: Beta fusion active
     * - Aggressive maneuvers: Beta fusion disabled
     * - Aerobatics: Beta fusion disabled
     * 
     * @note Called every EKF update cycle (100 Hz)
     * @note Fusion rate typically 5-10 Hz
     * @note Fixed-wing specific optimization
     * @note Not applicable to multirotors
     * 
     * @warning Invalid during uncoordinated flight
     * @warning Over-constrains during intentional sideslips
     * @warning Disable for crosswind landings if slipping
     * @warning Not valid for all vehicle types
     * 
     * @see FuseSideslip() which performs the fusion
     * @see assume_zero_sideslip() for applicability check
     * @see SelectTasFusion() for airspeed fusion (often paired)
     * @see wind_vel states (22-23) improved by this constraint
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:610
     */
    void SelectBetaFusion();

    /**
     * @brief Force alignment of the yaw angle using GPS velocity data
     * 
     * @details Resets the vehicle yaw angle to match the GPS ground track direction.
     *          Used when magnetometer is unavailable, unreliable, or as emergency yaw
     *          recovery during magnetic anomalies. Requires sufficient GPS velocity.
     * 
     * Yaw from GPS Velocity:
     * - GPS velocity vector: V_NE = [V_north, V_east]
     * - Yaw angle: ψ = atan2(V_east, V_north)
     * - Requires minimum velocity: Typically > 2 m/s
     * - Velocity direction = vehicle heading (for forward motion)
     * 
     * Alignment Conditions:
     * - GPS velocity magnitude sufficient (> threshold)
     * - GPS velocity data valid and recent
     * - Magnetometer unavailable or unreliable
     * - User requested GPS yaw reset (gpsYawResetRequest)
     * - Magnetic anomaly detected requiring yaw reset
     * 
     * Reset Process:
     * 1. Calculate yaw from GPS velocity direction
     * 2. Preserve current roll and pitch estimates
     * 3. Reset quaternion with new yaw: resetQuatStateYawOnly()
     * 4. Reset yaw covariance (increased uncertainty)
     * 5. Record yaw reset event: recordYawReset()
     * 6. Log yaw reset for analysis
     * 
     * Yaw Covariance Reset:
     * - Increase yaw uncertainty (P[0-2][0-2])
     * - Reflects reduced confidence in yaw
     * - Allows magnetometer to refine yaw subsequently
     * - Prevents over-confidence in GPS-derived yaw
     * 
     * Limitations:
     * - Requires vehicle motion (velocity > threshold)
     * - Ambiguous when stationary (no velocity direction)
     * - Velocity direction ≠ heading for sideways motion
     * - GPS velocity noise causes yaw jitter at low speeds
     * - Multipath errors affect GPS velocity accuracy
     * 
     * Use Cases:
     * - Initial yaw alignment without magnetometer
     * - Magnetometer failure recovery
     * - Magnetic anomaly correction
     * - Indoor navigation (no compass)
     * - Areas with magnetic disturbances
     * 
     * Vehicle Type Considerations:
     * - Fixed-wing: Velocity direction = heading (forward flight)
     * - Multicopter: May move sideways, velocity ≠ heading
     * - Ground vehicle: Generally forward motion, OK
     * - Requires careful application for non-fixed-wing
     * 
     * Magnetic Field State Impact:
     * - May reset magnetic field states
     * - Earth magnetic field states unchanged
     * - Body magnetic field offsets may be reset
     * - Allows magnetometer re-learning after reset
     * 
     * Reset Event Recording:
     * - yawResetAngle: Change in yaw (radians)
     * - lastYawReset_ms: Time of reset
     * - Reported to vehicle code for compensation
     * - Position controller adjusts for yaw jump
     * 
     * @note Called when GPS yaw reset requested or needed
     * @note Requires sufficient GPS velocity (> 2 m/s typical)
     * @note Emergency yaw recovery mechanism
     * @note Alternative to magnetometer for yaw
     * 
     * @warning Velocity direction ≠ heading for sideways motion
     * @warning Unreliable at low speeds (< 2 m/s)
     * @warning GPS velocity noise causes yaw errors
     * @warning Not suitable when stationary
     * 
     * @see resetQuatStateYawOnly() for quaternion reset with yaw
     * @see recordYawReset() for reset event recording
     * @see gpsYawResetRequest flag
     * @see yawResetAngle for reset magnitude
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:613
     */
    void realignYawGPS();

    /**
     * @brief Initialize earth magnetic field states using declination, attitude, and magnetometer
     * 
     * @details Calculates initial attitude quaternion and magnetic field states from
     *          accelerometer (roll, pitch) and magnetometer measurements. Critical for
     *          establishing accurate initial yaw and magnetic field references.
     * 
     * @param[in] roll Initial roll angle estimate in radians (from accelerometer)
     * @param[in] pitch Initial pitch angle estimate in radians (from accelerometer)
     * @return Quaternion representing initial attitude (NED to body frame rotation)
     * 
     * Initialization Inputs:
     * - Roll: From accelerometer gravity vector (tilt)
     * - Pitch: From accelerometer gravity vector (tilt)
     * - Magnetometer: 3-axis magnetic field in body frame
     * - Declination: Magnetic declination from world model
     * 
     * Calculation Process:
     * 1. Create quaternion from roll and pitch (zero yaw)
     * 2. Read current magnetometer measurement
     * 3. Rotate magnetometer to NED frame using attitude
     * 4. Apply declination correction for true north
     * 5. Calculate yaw from NED magnetic field
     * 6. Update quaternion with yaw component
     * 7. Initialize earth_magfield states (16-18)
     * 8. Initialize body_magfield states (19-21) to zero
     * 
     * Earth Magnetic Field States:
     * - earth_magfield (states 16-18): NED frame components
     * - Magnitude: Local earth field strength (~500 mGauss)
     * - Direction: Points to magnetic north (with declination)
     * - Down component: Variable with latitude
     * 
     * Body Magnetic Field States:
     * - body_magfield (states 19-21): Body frame offsets
     * - Initially zero (no learned interference)
     * - Will be learned during flight
     * - Compensates for hard iron (fixed offsets)
     * 
     * Declination Application:
     * - Declination: Angle between magnetic north and true north
     * - Applied to align magnetic field with true north
     * - Critical for accurate yaw initialization
     * - Source: MagDeclination() from world magnetic model
     * 
     * Yaw Calculation:
     * - From NED magnetic field: ψ = atan2(-mag_E, mag_N)
     * - Negative East component convention
     * - Result: True north referenced yaw
     * - Accuracy: Limited by magnetic disturbances
     * 
     * Tilt-Compensated Yaw:
     * - Magnetometer rotated to NED frame first
     * - Compensates for vehicle tilt
     * - Accurate yaw regardless of roll/pitch
     * - Critical for accurate initialization
     * 
     * Use Cases:
     * - Initial filter alignment on ground
     * - Bootstrap initialization
     * - Magnetic field state reset
     * - After compass calibration
     * 
     * Requirements:
     * - Vehicle stationary or slow moving
     * - Valid magnetometer calibration
     * - Minimal magnetic interference
     * - Accurate roll/pitch from accelerometer
     * 
     * Returned Quaternion:
     * - Full attitude: Roll, pitch, yaw
     * - NED to body frame rotation
     * - Used to initialize stateStruct.quat
     * - Starting point for attitude estimation
     * 
     * @note Called during InitialiseFilterBootstrap()
     * @note Critical for accurate yaw initialization
     * @note Requires valid magnetometer and accelerometer
     * @note Sets initial magnetic field references
     * 
     * @warning Magnetic interference causes yaw errors
     * @warning Requires vehicle stationary for accuracy
     * @warning Accelerometer must provide accurate tilt
     * @warning Declination must be correct for location
     * 
     * @see InitialiseFilterBootstrap() which calls this
     * @see earth_magfield states (16-18)
     * @see body_magfield states (19-21)
     * @see MagDeclination() for declination lookup
     * @see stateStruct.quat for quaternion storage
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:616-617
     */
    QuaternionF calcQuatAndFieldStates(ftype roll, ftype pitch);

    /**
     * @brief Zero stored variables to initial/safe values
     * 
     * @details Initializes or resets all EKF member variables to known safe states.
     *          Called during filter construction, reset, or re-initialization.
     *          Ensures clean state before beginning navigation.
     * 
     * Variable Categories Initialized:
     * 
     * 1. State and Covariance:
     *    - State vector (statesArray): Zeroed
     *    - Covariance matrix (P): Will be set by CovarianceInit()
     *    - Output states: Reset to default
     * 
     * 2. Timing Variables:
     *    - imuSampleTime_ms: Current time
     *    - ekfStartTime_ms: Current time
     *    - Last measurement times: Zeroed
     *    - Timeout timers: Reset
     * 
     * 3. Health Flags:
     *    - magHealth, velTimeout, posTimeout, etc.: false
     *    - All sensor health flags: Reset to unknown
     *    - allMagSensorsFailed: false
     * 
     * 4. Measurement Buffers:
     *    - storedIMU: Cleared
     *    - storedGPS, storedMag, storedBaro: Cleared
     *    - storedTAS, storedRange, storedOF: Cleared
     *    - storedOutput: Cleared
     * 
     * 5. Innovation Variables:
     *    - innovVelPos, innovMag: Zeroed
     *    - varInnovVelPos, varInnovMag: Zeroed
     *    - All test ratios: Reset
     * 
     * 6. Fusion Flags:
     *    - fuseVelData, fusePosData, fuseHgtData: false
     *    - tasDataToFuse, flowDataToFuse, etc.: false
     *    - Fusion delayed flags: Reset
     * 
     * 7. Alignment Flags:
     *    - tiltAlignComplete: false
     *    - yawAlignComplete: false
     *    - magStateInitComplete: false
     *    - statesInitialised: false
     * 
     * 8. Position/Velocity Tracking:
     *    - lastKnownPositionNE: Zeroed
     *    - velOffsetNED, posOffsetNED: Zeroed
     *    - Output tracking errors: Reset
     * 
     * 9. Range Finder Variables:
     *    - terrainState: Zero
     *    - Popt: Large initial uncertainty
     *    - Range measurements: Cleared
     * 
     * 10. Optical Flow Variables:
     *     - Flow data: Reset
     *     - Flow test ratios: Zeroed
     *     - Terrain offset: Reset
     * 
     * 11. Beacon Variables:
     *     - Beacon positions: Cleared
     *     - Beacon health: Reset
     *     - Alignment flags: false
     * 
     * 12. External Nav Variables:
     *     - External nav data: Cleared
     *     - External nav flags: Reset
     * 
     * 13. Wind and Airspeed:
     *     - defaultAirSpeed: Zero
     *     - Wind states will be initialized
     * 
     * 14. Fault Status:
     *     - faultStatus: All bits cleared
     *     - gpsCheckStatus: All bits cleared
     * 
     * Purpose of Initialization:
     * - Prevent undefined behavior from uninitialized variables
     * - Establish known starting state
     * - Clear stale data from previous flight/reset
     * - Ensure consistent behavior across resets
     * 
     * When Called:
     * - During NavEKF2_core construction
     * - During setup_core()
     * - Before InitialiseFilterBootstrap()
     * - After major filter resets
     * 
     * Separate Initialization:
     * - Magnetic variables: InitialiseVariablesMag()
     * - Covariance matrix: CovarianceInit()
     * - States: Set during alignment
     * 
     * @note Called before filter begins operation
     * @note Essential for deterministic behavior
     * @note Separates construction from initialization
     * @note Enables clean filter resets
     * 
     * @warning Must be called before using filter
     * @warning Incomplete initialization causes undefined behavior
     * 
     * @see InitialiseVariablesMag() for magnetic-specific variables
     * @see CovarianceInit() for covariance initialization
     * @see InitialiseFilterBootstrap() for full filter init
     * @see setup_core() which calls initialization functions
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:620
     */
    void InitialiseVariables();

    /**
     * @brief Initialize magnetometer-specific variables
     * 
     * @details Initializes variables specifically related to magnetometer fusion and
     *          magnetic field state management. Called separately from general variable
     *          initialization to allow magnetic system reset without full filter reset.
     * 
     * Magnetic Variables Initialized:
     * 
     * 1. Compass Selection:
     *    - magSelectIndex: Selected compass (default: 0)
     *    - lastMagOffsets: Cleared
     *    - lastMagOffsetsValid: false
     * 
     * 2. Magnetic Health Status:
     *    - magHealth: false (unknown health)
     *    - magTimeout: false (no timeout yet)
     *    - consistentMagData: false (not yet verified)
     *    - allMagSensorsFailed: false
     *    - lastHealthyMagTime_ms: Current time
     * 
     * 3. Magnetic Fusion Timing:
     *    - lastMagUpdate_us: Zero
     *    - lastMagRead_ms: Zero
     *    - lastYawTime_ms: Zero
     * 
     * 4. Magnetic Innovations:
     *    - innovMag: [0, 0, 0]
     *    - varInnovMag: [0, 0, 0]
     *    - magTestRatio: [0, 0, 0]
     *    - yawTestRatio: Zero
     * 
     * 5. Magnetic Fusion State:
     *    - mag_state: All fields cleared
     *    - magFusePerformed: false
     * 
     * 6. Magnetic Field Learning:
     *    - inhibitMagStates: Initially false (allow learning)
     *    - lastInhibitMagStates: false
     *    - magFieldLearned: false
     *    - wasLearningCompass_ms: Zero
     *    - earthMagFieldVar: Initial uncertainty values
     *    - bodyMagFieldVar: Initial uncertainty values
     * 
     * 7. Yaw Reset Variables:
     *    - magYawResetTimer_ms: Zero
     *    - magStateResetRequest: false
     *    - magYawResetRequest: false
     *    - yawInnovAtLastMagReset: Zero
     *    - posDownAtLastMagReset: Zero
     *    - quatAtLastMagReset: Identity quaternion
     *    - prevQuatMagReset: Identity quaternion
     * 
     * 8. Magnetic Anomaly Tracking:
     *    - magYawAnomallyCount: Zero
     *    - Allows limited yaw resets due to anomalies
     * 
     * 9. In-Flight Magnetic Initialization:
     *    - finalInflightYawInit: false
     *    - finalInflightMagInit: false
     *    - Flags for post-takeoff magnetic alignment
     * 
     * 10. Declination Handling:
     *     - table_declination: From world magnetic model
     *     - have_table_earth_field: false initially
     *     - table_earth_field_ga: Will be set from WMM
     * 
     * Purpose of Separate Initialization:
     * - Enables compass switching without full filter reset
     * - Allows magnetic field reset without losing navigation
     * - Supports compass calibration during flight
     * - Facilitates compass failure recovery
     * 
     * When Called:
     * - During InitialiseVariables()
     * - When switching compasses
     * - After compass calibration
     * - During magnetic anomaly recovery
     * - Before magnetic field state reset
     * 
     * Reset Scenarios:
     * - Compass calibration completed
     * - Magnetic anomaly detected
     * - Compass switching
     * - User-requested magnetic reset
     * 
     * Impact on Navigation:
     * - Brief yaw uncertainty increase
     * - Position/velocity maintained
     * - Magnetic field states re-learned
     * - Yaw converges to magnetometer over time
     * 
     * @note Called by InitialiseVariables() during full init
     * @note Can be called independently for magnetic reset
     * @note Preserves non-magnetic navigation states
     * @note Essential for compass failure recovery
     * 
     * @warning Frequent resets indicate compass problems
     * @warning Yaw accuracy reduced until field re-learned
     * 
     * @see InitialiseVariables() for general initialization
     * @see controlMagYawReset() for reset logic
     * @see tryChangeCompass() for compass switching
     * @see magSelectIndex for compass selection
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:622
     */
    void InitialiseVariablesMag();

    /**
     * @brief Reset horizontal position states using the last GPS measurement
     * 
     * @details Resets the North and East position states to match the most recent GPS
     *          position measurement. Used to recover from position divergence, large
     *          innovation failures, or when switching position references.
     * 
     * Reset Process:
     * 1. Retrieve last GPS position from buffer (gpsDataDelayed)
     * 2. Convert GPS lat/lon to NED relative to EKF origin
     * 3. Update position states: stateStruct.position[0-1]
     * 4. Reset position covariance: P[6-7][6-7]
     * 5. Reset output buffer positions to match
     * 6. Record reset event for reporting
     * 
     * Position Covariance Reset:
     * - Increase position uncertainty: Reflects reset
     * - P[6][6] (North position variance): GPS accuracy
     * - P[7][7] (East position variance): GPS accuracy
     * - Correlations with other states: Preserved or reduced
     * - Prevents over-confidence after reset
     * 
     * When Reset Triggered:
     * - Large position innovation failures (posTimeout)
     * - Position divergence detected (checkDivergence())
     * - After long GPS outage recovery
     * - GPS receiver switch
     * - Initial GPS acquisition after GPS-denied period
     * 
     * Output Buffer Impact:
     * - storedOutput buffer: All positions updated
     * - Ensures output consistency with state
     * - Prevents position jumps in output
     * - Smooth transition to reset position
     * 
     * Position Error Integral:
     * - posErrintegral: Reset to zero
     * - Output predictor tracking error cleared
     * - Fresh start for complementary filter
     * 
     * Reset Event Recording:
     * - posResetNE: Position change [ΔN, ΔE] in meters
     * - lastPosReset_ms: Time of reset
     * - Reported to vehicle code for compensation
     * - Position controller adjusts for jump
     * 
     * GPS Origin Requirement:
     * - Requires valid EKF origin (validOrigin)
     * - GPS position converted relative to origin
     * - Origin typically set on first GPS fix
     * 
     * Impact on Navigation:
     * - Immediate position jump in NED frame
     * - Velocity states unchanged (continuity)
     * - Attitude states unchanged
     * - Vehicle controllers compensate for jump
     * 
     * Use Cases:
     * - Position divergence correction
     * - GPS recovery after outage
     * - Switching between GPS receivers
     * - Correcting accumulated drift
     * - Emergency position recovery
     * 
     * Alternative to Reset:
     * - Sometimes GPS velocity fusion continues
     * - Position gradually converges without reset
     * - Reset is last resort for large errors
     * 
     * Height Handling:
     * - This resets only horizontal (NE) position
     * - Vertical (D) position reset: ResetHeight()
     * - Height often has independent reference (baro)
     * 
     * @note Triggered by large position innovation failures
     * @note Creates discrete position jump (reset event)
     * @note Vehicle code must handle position discontinuity
     * @note Position covariance increased to reflect uncertainty
     * 
     * @warning Causes immediate position jump
     * @warning May disrupt mission execution temporarily
     * @warning Should be rare during normal operation
     * @warning Frequent resets indicate filter problems
     * 
     * @see ResetPositionNE() for direct position reset
     * @see ResetHeight() for vertical position reset
     * @see ResetVelocity() for velocity reset
     * @see posResetNE, lastPosReset_ms for reset tracking
     * @see gpsDataDelayed for GPS measurement source
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:625
     */
    void ResetPosition(void);

    /**
     * @brief Reset the stateStruct's NE position to the specified position
     * 
     * @details Directly sets the North and East position states to specified values.
     *          Lower-level reset function called by ResetPosition() or for external
     *          position references (external nav, range beacons, etc.).
     * 
     * @param[in] posN North position in meters (NED frame, relative to EKF origin)
     * @param[in] posE East position in meters (NED frame, relative to EKF origin)
     * 
     * Reset Actions:
     * 1. Set stateStruct.position[0] = posN (North)
     * 2. Set stateStruct.position[1] = posE (East)
     * 3. Update output buffer positions
     * 4. Reset position error integrals
     * 5. Record reset event
     * 
     * Direct State Assignment:
     * - Bypasses fusion process
     * - Immediate state update
     * - No innovation calculation
     * - No Kalman gain computation
     * 
     * Output Buffer Update:
     * - All entries in storedOutput updated
     * - Maintains consistency between state and output
     * - Prevents output prediction errors
     * - Smooth output after reset
     * 
     * Position Error Integral Reset:
     * - posErrintegral[0] = 0 (North)
     * - posErrintegral[1] = 0 (East)
     * - Clears accumulated tracking error
     * - Fresh start for output predictor
     * 
     * Reset Event Recording:
     * - posResetNE[0] = posN - old_posN (North change)
     * - posResetNE[1] = posE - old_posE (East change)
     * - lastPosReset_ms = current time
     * - Allows vehicle code to compensate
     * 
     * Use Cases:
     * - Called by ResetPosition() with GPS position
     * - External navigation position reset
     * - Range beacon position reset
     * - Manual position initialization
     * - Test/simulation scenarios
     * 
     * Coordinate Frame:
     * - NED frame relative to EKF origin
     * - Origin set by setOrigin() or auto-set from GPS
     * - North: Positive northward (m)
     * - East: Positive eastward (m)
     * 
     * Covariance Handling:
     * - Position covariance NOT automatically reset here
     * - Caller responsible for covariance reset
     * - Typically increase P[6][6] and P[7][7]
     * - Reflects uncertainty in new position
     * 
     * State Vector Indexing:
     * - stateStruct.position[0]: North position (state 6)
     * - stateStruct.position[1]: East position (state 7)
     * - stateStruct.position[2]: Down position (state 8, unchanged)
     * 
     * No Validation:
     * - Does not check position reasonableness
     * - Does not validate against limits
     * - Caller responsible for valid inputs
     * - Trusts provided position values
     * 
     * Impact on Filter:
     * - Position states discontinuity
     * - Velocity states unchanged (continuous)
     * - Attitude states unchanged
     * - May affect magnetic field state convergence
     * 
     * @note Low-level reset function, use with care
     * @note Caller must handle covariance reset separately
     * @note Records reset event for vehicle code
     * @note Does not perform validation checks
     * 
     * @warning No safety checks on input values
     * @warning Caller must ensure reasonable position
     * @warning Must reset covariance separately
     * @warning Can cause filter instability if misused
     * 
     * @see ResetPosition() for GPS-based reset (higher level)
     * @see ResetPositionD() for vertical position reset
     * @see posResetNE for reset magnitude tracking
     * @see stateStruct.position for position states
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:628
     */
    void ResetPositionNE(ftype posN, ftype posE);

    /**
     * @brief Reset velocity states using the last GPS measurement
     * 
     * @details Resets the NED velocity states to match the most recent GPS velocity
     *          measurement. Used to recover from velocity divergence or innovation
     *          failures. Less disruptive than position reset.
     * 
     * Reset Process:
     * 1. Retrieve last GPS velocity from buffer (gpsDataDelayed)
     * 2. Update velocity states: stateStruct.velocity[0-2]
     * 3. Reset velocity covariance: P[3-5][3-5]
     * 4. Reset output buffer velocities
     * 5. Record reset event
     * 
     * Velocity Covariance Reset:
     * - Increase velocity uncertainty
     * - P[3][3] (North velocity variance): GPS accuracy
     * - P[4][4] (East velocity variance): GPS accuracy
     * - P[5][5] (Down velocity variance): GPS accuracy
     * - Reflects reduced confidence after reset
     * 
     * When Reset Triggered:
     * - Velocity innovation failures (velTimeout)
     * - Large velocity test ratio (velTestRatio > threshold)
     * - After GPS outage recovery
     * - IMU accelerometer bias errors detected
     * - Velocity divergence in GPS-denied recovery
     * 
     * Output Buffer Impact:
     * - storedOutput buffer: All velocities updated
     * - outputDataNew.velocity: Reset to GPS velocity
     * - outputDataDelayed.velocity: Reset
     * - Ensures output consistency
     * 
     * Velocity Error Integral:
     * - velErrintegral: Reset to [0, 0, 0]
     * - Output predictor tracking error cleared
     * - Complementary filter restarted
     * 
     * Reset Event Recording:
     * - velResetNE: Velocity change [ΔVn, ΔVe] m/s
     * - lastVelReset_ms: Time of reset
     * - Reported for velocity controller compensation
     * - Smoother than position reset impact
     * 
     * Impact on Navigation:
     * - Velocity states discontinuity
     * - Position states continuous (unchanged)
     * - Attitude states unchanged
     * - Less disruptive than position reset
     * 
     * Accelerometer Bias Impact:
     * - Velocity reset doesn't fix accel bias
     * - Bias will cause velocity to drift again
     * - May indicate accelerometer calibration issue
     * - Repeated resets suggest bias problem
     * 
     * Use Cases:
     * - Velocity divergence correction
     * - GPS velocity recovery after outage
     * - Accelerometer step bias compensation
     * - Emergency velocity recovery
     * - Pre-flight velocity initialization
     * 
     * GPS Velocity Quality:
     * - Uses GPS velocity, not derived from position
     * - GPS velocity often more accurate short-term
     * - Less affected by multipath than position
     * - Good for dynamic motion
     * 
     * Height Velocity:
     * - Resets vertical (Down) velocity component
     * - May differ from barometer-derived rate
     * - GPS vertical velocity less accurate
     * - Consider if baro rate is better
     * 
     * @note Less disruptive than position reset
     * @note Velocity typically converges faster than position
     * @note May indicate accelerometer issues if frequent
     * @note Records event for vehicle code compensation
     * 
     * @warning Doesn't fix underlying accelerometer bias
     * @warning Frequent resets indicate IMU problems
     * @warning May briefly affect velocity control loops
     * @warning Check GPS velocity quality before reset
     * 
     * @see ResetPosition() for position reset
     * @see ResetHeight() for height reset
     * @see velResetNE, lastVelReset_ms for reset tracking
     * @see gpsDataDelayed for GPS velocity source
     * @see stateStruct.velocity for velocity states (3-5)
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:631
     */
    void ResetVelocity(void);

    /**
     * @brief Reset the vertical position state using the last height measurement
     * 
     * @details Resets the Down position state to match the most recent height measurement
     *          from the currently active height source (barometer, GPS, rangefinder, or
     *          external nav). Corrects vertical position drift or reference changes.
     * 
     * Reset Process:
     * 1. Identify active height source (activeHgtSource)
     * 2. Retrieve height measurement from appropriate buffer
     * 3. Update Down position state: stateStruct.position[2]
     * 4. Reset vertical position covariance: P[8][8]
     * 5. Reset output buffer Down positions
     * 6. Record reset event
     * 
     * Active Height Sources:
     * - HGT_SOURCE_BARO (0): Barometric altitude
     *   * Most common, stable, smooth
     *   * Affected by weather pressure changes
     * - HGT_SOURCE_RNG (1): Rangefinder
     *   * Terrain-relative height
     *   * Limited range (typically < 100 m)
     * - HGT_SOURCE_GPS (2): GPS altitude
     *   * WGS-84 ellipsoid height
     *   * Less accurate vertically than horizontally
     * - HGT_SOURCE_BCN (3): Range beacons
     *   * Trilateration vertical component
     * - HGT_SOURCE_EXTNAV (4): External navigation
     *   * External position system
     * 
     * Height Measurement Selection:
     * - selectHeightForFusion() determines source
     * - May switch sources during flight
     * - Reset triggered when switching sources
     * - Different sources have different references
     * 
     * Vertical Position Covariance:
     * - P[8][8]: Down position variance
     * - Reset to measurement uncertainty
     * - Barometer: ~1 m² typical
     * - GPS: ~4 m² typical
     * - Rangefinder: ~0.1 m² over good terrain
     * 
     * When Reset Triggered:
     * - Height innovation failures (hgtTimeout)
     * - Height source switching
     * - Large barometric pressure step
     * - GPS altitude recovery
     * - Rangefinder terrain reference change
     * 
     * Output Buffer Update:
     * - storedOutput Down positions updated
     * - Ensures smooth output transition
     * - Prevents altitude jump in output
     * 
     * Reset Event Recording:
     * - posResetD: Down position change (m)
     * - lastPosResetD_ms: Time of reset
     * - Reported to altitude controller
     * - Controller compensates for altitude jump
     * 
     * Height Reference Consistency:
     * - EKF origin height: ekfGpsRefHgt (WGS-84)
     * - Barometer: Relative to origin height
     * - GPS: WGS-84, converted to NED Down
     * - Rangefinder: Terrain-relative
     * - Must maintain consistent reference frame
     * 
     * Impact on Navigation:
     * - Vertical position discontinuity
     * - Vertical velocity continuous (unchanged)
     * - Horizontal states unchanged
     * - Altitude controller sees step
     * 
     * Use Cases:
     * - Barometer pressure step correction
     * - Height source switching
     * - GPS altitude recovery
     * - Rangefinder reference change
     * - Initial height alignment
     * 
     * Barometer Special Handling:
     * - Barometer can have QNH offset
     * - Pressure changes with weather
     * - May need origin height adjustment
     * - resetHeightDatum() available for this
     * 
     * @note Vertical position only (Down state)
     * @note Height source determines reference frame
     * @note Less frequent than horizontal resets
     * @note Records event for altitude controller
     * 
     * @warning Causes altitude jump
     * @warning Must handle in altitude controller
     * @warning Different sources have different references
     * @warning Frequent resets indicate height sensor issues
     * 
     * @see ResetPositionD() for direct Down position reset
     * @see ResetPosition() for horizontal position reset
     * @see selectHeightForFusion() for source selection
     * @see activeHgtSource for current height source
     * @see posResetD, lastPosResetD_ms for reset tracking
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:634
     */
    void ResetHeight(void);

    /**
     * @brief Reset the stateStruct's D (Down) position to specified value
     * 
     * @details Directly sets the Down position state to a specified value. Lower-level
     *          reset function for vertical position, called by ResetHeight() or for
     *          direct height initialization.
     * 
     * @param[in] posD Down position in meters (NED frame, positive downward from origin)
     * 
     * Reset Actions:
     * 1. Set stateStruct.position[2] = posD (Down)
     * 2. Update output buffer Down positions
     * 3. Reset vertical position error integral
     * 4. Record reset event
     * 
     * Down Position Convention:
     * - NED frame: Down is positive downward
     * - Origin: EKF origin altitude (ekfGpsRefHgt)
     * - posD = 0: At origin altitude
     * - posD > 0: Below origin altitude
     * - posD < 0: Above origin altitude
     * 
     * Direct State Assignment:
     * - Bypasses measurement fusion
     * - Immediate state update
     * - No innovation, no Kalman gain
     * - Simple assignment: stateStruct.position[2] = posD
     * 
     * Output Buffer Update:
     * - All storedOutput Down positions updated
     * - Maintains state-output consistency
     * - Prevents altitude jump in output predictor
     * - Smooth altitude output after reset
     * 
     * Vertical Error Integral Reset:
     * - posErrintegral[2] = 0 (Down component)
     * - Clears accumulated vertical tracking error
     * - Output predictor complementary filter reset
     * - Fresh start for altitude output
     * 
     * Reset Event Recording:
     * - posResetD = posD - old_posD (altitude change)
     * - lastPosResetD_ms = current time
     * - Vehicle altitude controller compensates
     * - Logged for analysis
     * 
     * Use Cases:
     * - Called by ResetHeight() with measurement
     * - Direct height initialization
     * - External nav altitude reset
     * - Beacon altitude reset
     * - Manual altitude setting (test/simulation)
     * 
     * Covariance Handling:
     * - Vertical position covariance NOT reset here
     * - Caller must reset P[8][8] separately
     * - Typically increase Down position variance
     * - Reflects uncertainty in new altitude
     * 
     * No Validation:
     * - Does not check altitude reasonableness
     * - Does not enforce limits
     * - Caller responsible for valid input
     * - Trusts provided altitude value
     * 
     * State Vector Indexing:
     * - stateStruct.position[2]: Down position (state 8)
     * - stateStruct.position[0-1]: North/East (unchanged)
     * 
     * Height Rate Consistency:
     * - Vertical velocity (state 5) unchanged
     * - hgtRate filter may need adjustment
     * - vertCompFiltState may need update
     * - Ensures kinematic consistency
     * 
     * Impact on Filter:
     * - Down position discontinuity
     * - Vertical velocity continuous
     * - Horizontal states unchanged
     * - Barometer offset may be affected
     * 
     * @note Low-level reset function
     * @note Caller must handle covariance reset
     * @note Records reset event for vehicle code
     * @note No validation of input value
     * 
     * @warning No safety checks on input
     * @warning Caller ensures reasonable altitude
     * @warning Must reset covariance separately
     * @warning Can cause instability if misused
     * 
     * @see ResetHeight() for measurement-based reset (higher level)
     * @see ResetPositionNE() for horizontal position reset
     * @see posResetD for reset magnitude tracking
     * @see stateStruct.position[2] for Down state
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:637
     */
    void ResetPositionD(ftype posD);

    /**
     * @brief Return true if we should use the airspeed sensor
     * 
     * @details Determines whether airspeed sensor data should be used for EKF fusion
     *          based on sensor availability, quality, vehicle type, and configuration.
     * 
     * @return true if airspeed should be used, false otherwise
     * 
     * Airspeed Usage Conditions:
     * All must be true for airspeed use:
     * 1. Airspeed sensor configured and enabled
     * 2. Vehicle is fixed-wing or relevant type
     * 3. Sufficient airspeed for accuracy (typically > 5 m/s)
     * 4. Sensor health good (no timeouts)
     * 5. Innovation consistency acceptable
     * 6. Not failed pre-flight checks
     * 
     * Vehicle Type Considerations:
     * - Fixed-wing aircraft: Airspeed critical
     *   * Stall prevention requires airspeed
     *   * Coordinated turns need airspeed
     * - Multicopters: Generally don't use airspeed
     *   * No forward flight assumption
     *   * Wind velocity estimated differently
     * - Quadplanes: Use airspeed in forward flight
     *   * Transitions between modes
     * - Ground vehicles: Don't use airspeed
     * 
     * Airspeed Sensor Types:
     * - Analog differential pressure
     * - I2C digital sensors (MS4525, MS5525, etc.)
     * - Calibration required for accuracy
     * - Temperature compensation important
     * 
     * Minimum Airspeed Threshold:
     * - Low airspeeds unreliable (sensor noise)
     * - Typically require > 5 m/s for fusion
     * - Below threshold: Use synthetic airspeed
     * - Prevents fusion of noisy low-speed data
     * 
     * Airspeed Innovation Checks:
     * - tasTestRatio: Innovation consistency metric
     * - If tasTestRatio > threshold: Don't use
     * - Large innovations indicate sensor problem
     * - Or indicates actual sideslip/wind change
     * 
     * Airspeed Timeout:
     * - tasTimeout flag: No valid data recently
     * - Set if no airspeed for ~10 seconds
     * - Indicates sensor failure or disconnection
     * - Filter continues without airspeed
     * 
     * Impact of Airspeed Fusion:
     * - Enables wind velocity estimation (states 22-23)
     * - Improves GPS velocity accuracy in wind
     * - Enables synthetic sideslip constraint
     * - Critical for coordinated flight in wind
     * 
     * Synthetic Airspeed:
     * - If !useAirspeed(): May use defaultAirSpeed
     * - Approximate airspeed for wind estimation
     * - Less accurate than measured
     * - writeDefaultAirSpeed() sets this value
     * 
     * Wind State Learning:
     * - Requires airspeed or GPS ground track
     * - Airspeed + GPS velocity → wind velocity
     * - Without airspeed: Wind states inhibited or poor
     * - setWindMagStateLearningMode() handles this
     * 
     * Configuration Parameters:
     * - EK2_ARSPEED_USE: Enable/disable airspeed fusion
     * - Sensor selection in AP_Airspeed library
     * - Calibration parameters critical
     * 
     * Pre-Flight Checks:
     * - Airspeed sensor health checked
     * - Reasonable airspeed on ground (near zero)
     * - Sensor offset calibration validated
     * 
     * @note Fixed-wing aircraft typically need airspeed
     * @note Multirotors generally don't use airspeed
     * @note Low airspeeds (< 5 m/s) not fused
     * @note Enables accurate wind estimation
     * 
     * @warning Airspeed sensor calibration critical
     * @warning Uncalibrated sensor causes errors
     * @warning Blocked pitot tube causes failure
     * 
     * @see FuseAirspeed() for airspeed fusion
     * @see SelectTasFusion() for fusion scheduling
     * @see tasTimeout for sensor health
     * @see tasTestRatio for innovation check
     * @see stateStruct.wind_vel for wind states (22-23)
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:640
     */
    bool useAirspeed(void) const;

    /**
     * @brief Return true if vehicle code has requested filter ready for flight
     * 
     * @details Checks if the vehicle code has indicated that GPS data quality is
     *          sufficient and the filter should be ready to use GPS for navigation.
     *          Part of the pre-arm and flight readiness checks.
     * 
     * @return true if filter should prepare for GPS-based flight, false otherwise
     * 
     * Readiness Conditions:
     * - Vehicle arming system requesting GPS readiness
     * - GPS quality meets minimum standards
     * - GPS data is being received
     * - Filter has completed initial alignment
     * - System is ready for autonomous flight
     * 
     * Pre-Arm Checks:
     * - Called during pre-arm check sequence
     * - Prevents arming with inadequate GPS
     * - Ensures GPS meets safety standards
     * - Part of comprehensive pre-flight validation
     * 
     * GPS Quality Requirements:
     * - Sufficient satellites (typically ≥ 6)
     * - Adequate HDOP (typically < 2.0)
     * - GPS velocity accuracy acceptable
     * - GPS position accuracy acceptable
     * - GPS altitude accuracy acceptable (if used)
     * 
     * Filter Alignment Status:
     * - Tilt alignment complete
     * - Yaw alignment complete (GPS or compass)
     * - Position initialized
     * - Velocity tracking GPS
     * 
     * Use in Filter Logic:
     * - Enables GPS data fusion
     * - Allows autonomous mode transitions
     * - Required for GPS-based missions
     * - Required for position hold modes
     * 
     * Vehicle State Considerations:
     * - On ground vs in flight
     * - Manual vs autonomous mode
     * - GPS-denied fallback available
     * - Safety pilot vs autonomous operation
     * 
     * Configuration Impact:
     * - EK2_CHECK_SCALE: Affects thresholds
     * - EK2_NOAID_M_NSE: Non-GPS mode noise
     * - May allow non-GPS flight modes
     * 
     * Timeout Handling:
     * - If GPS lost after ready: May continue briefly
     * - Eventually transitions to non-GPS mode
     * - Depends on available backup sensors
     * 
     * External Factors:
     * - Geofence requirements
     * - Mission requirements
     * - Regulatory requirements (some require GPS)
     * - Operator safety procedures
     * 
     * @note Does not guarantee GPS currently valid
     * @note Indicates vehicle requesting GPS readiness
     * @note Part of comprehensive pre-arm checks
     * @note May be false even if GPS available
     * 
     * @warning Vehicle won't arm if returns false when GPS required
     * @warning Does not check current GPS health (just readiness)
     * 
     * @see calcGpsGoodToAlign() for GPS quality assessment
     * @see gpsGoodToAlign for GPS alignment status
     * @see readGpsData() for GPS data reading
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:643
     */
    bool readyToUseGPS(void) const;

    /**
     * @brief Return true if the filter is ready to use range beacon measurements
     * 
     * @details Checks if range beacon system has sufficient quality and the filter
     *          is ready to use range beacon measurements for position estimation.
     * 
     * @return true if ready to use range beacons, false otherwise
     * 
     * Range Beacon System:
     * - Multiple beacons at known locations
     * - Measure range to vehicle
     * - Trilateration determines vehicle position
     * - Alternative to GPS for positioning
     * 
     * Readiness Requirements:
     * All must be true:
     * 1. Range beacon system enabled (AP_Beacon)
     * 2. Minimum beacon count (typically ≥ 3)
     * 3. Beacons at known positions
     * 4. Recent valid range measurements
     * 5. Beacon geometry adequate (not collinear)
     * 6. Range measurement quality acceptable
     * 
     * Beacon Configuration:
     * - N_beacons: Number of configured beacons
     * - Beacon positions: beaconPosNED[] in NED frame
     * - Beacon health: Individual beacon status
     * - Minimum 3 beacons for 3D position
     * 
     * Quality Checks:
     * - Range innovation consistency
     * - rngBcnTestRatio < threshold
     * - Recent range measurements (not timed out)
     * - Adequate signal strength (if available)
     * 
     * Geometric Considerations:
     * - Beacons must not be collinear
     * - Good geometric dilution of precision (GDOP)
     * - Vertical distribution improves altitude
     * - Horizontal spread improves horizontal position
     * 
     * Alignment Process:
     * - rngBcnAlignmentStarted: Initial position estimation
     * - rngBcnAlignmentCompleted: Alignment converged
     * - Uses FuseRngBcnStatic() for static alignment
     * - Transitions to dynamic fusion after alignment
     * 
     * Use Cases:
     * - Indoor navigation (no GPS)
     * - GPS-denied environments
     * - Warehouses, mines, tunnels
     * - Precision positioning in small areas
     * - Alternative to GPS
     * 
     * Vertical Position:
     * - bcnPosOffset: Beacon constellation vertical offset
     * - Vertical position less accurate than horizontal
     * - Depends on beacon vertical distribution
     * 
     * Fusion Mode:
     * - Static: Vehicle stationary during alignment
     * - Dynamic: Vehicle moving, continuous fusion
     * - FuseRngBcn() for dynamic fusion
     * - SelectRngBcnFusion() schedules fusion
     * 
     * Impact on Navigation:
     * - Provides absolute position reference
     * - Alternative to GPS
     * - Can be more accurate in small volumes
     * - Limited to beacon coverage area
     * 
     * Configuration:
     * - BCN_TYPE: Beacon system type
     * - BCN_LATITUDE, BCN_LONGITUDE, BCN_ALT: Origin
     * - BCN_ORIENT_YAW: Beacon frame orientation
     * - Individual beacon positions configured
     * 
     * @note Alternative to GPS for positioning
     * @note Requires ≥ 3 beacons for 3D position
     * @note Beacon geometry affects accuracy
     * @note Limited to beacon coverage area
     * 
     * @warning Beacon positions must be accurately surveyed
     * @warning Poor geometry causes poor position accuracy
     * @warning Multipath affects range measurements
     * 
     * @see FuseRngBcn() for range beacon fusion
     * @see FuseRngBcnStatic() for static alignment
     * @see SelectRngBcnFusion() for fusion scheduling
     * @see rngBcnGoodToAlign for alignment readiness
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:646
     */
    bool readyToUseRangeBeacon(void) const;

    /**
     * @brief Return true if the filter is ready to use external navigation data
     * 
     * @details Checks if external navigation system data quality is sufficient and
     *          the filter is ready to use external nav measurements for position/velocity
     *          estimation (e.g., motion capture, external vision, RTK GPS).
     * 
     * @return true if ready to use external navigation, false otherwise
     * 
     * External Navigation Sources:
     * - Motion capture systems (Vicon, OptiTrack, etc.)
     * - Visual-Inertial Odometry (VIO)
     * - External vision systems (Intel T265, etc.)
     * - RTK GPS corrections
     * - External positioning systems
     * - Marker-based tracking
     * 
     * Readiness Requirements:
     * All must be true:
     * 1. External nav system enabled (AP_VisualOdom)
     * 2. Recent valid external nav data received
     * 3. External nav data quality acceptable
     * 4. Position and orientation provided
     * 5. Data rate sufficient (typically ≥ 10 Hz)
     * 6. Coordinate frame alignment configured
     * 
     * Data Quality Checks:
     * - posErr: Position error estimate < threshold
     * - angErr: Angular error estimate < threshold
     * - Recent data: Not timed out
     * - Innovation consistency: extNavPassTime recent
     * 
     * External Nav Data Elements:
     * - Position: 3D position in NED frame
     * - Orientation: Quaternion (navigation to body)
     * - Position error: 1-sigma uncertainty (m)
     * - Angular error: 1-sigma uncertainty (rad)
     * - Timestamp: Measurement time
     * - Delay: System latency
     * 
     * Coordinate Frame Alignment:
     * - External system frame → NED frame
     * - Sensor body offset: CorrectExtNavForSensorOffset()
     * - Rotation alignment critical
     * - Translation offset handled
     * 
     * Use Cases:
     * - Indoor flight (motion capture)
     * - GPS-denied navigation
     * - High-precision positioning
     * - Vision-based navigation
     * - Research and development
     * - Warehouse automation
     * 
     * Fusion Options:
     * - Position only: extNavUsedForPos
     * - Orientation/yaw: extNavUsedForYaw
     * - Velocity: External nav velocity (if provided)
     * - Selective fusion based on quality
     * 
     * Yaw from External Nav:
     * - isExtNavUsedForYaw(): Check yaw fusion
     * - Alternative to magnetometer
     * - More accurate in some environments
     * - Requires good orientation estimate
     * 
     * Position Reset Handling:
     * - External system may reset position
     * - extNavLastPosResetTime_ms: Tracks resets
     * - EKF must handle discontinuities
     * - Communicated via extNavDataNew.posReset
     * 
     * Data Rate Requirements:
     * - Minimum ~10 Hz for good performance
     * - Higher rate (30-100 Hz) better
     * - Latency critical (delay_ms parameter)
     * - Old data may be rejected
     * 
     * Configuration:
     * - VISO_TYPE: External nav system type
     * - VISO_POS_*: Position offset to sensor
     * - VISO_DELAY_MS: System latency
     * - EK2_EXTNAV_DELAY: EKF-specific delay
     * 
     * Impact on Navigation:
     * - Provides absolute position reference
     * - May provide yaw reference
     * - Enables GPS-denied flight
     * - Often more accurate than GPS short-term
     * 
     * @note Alternative to GPS for positioning
     * @note Requires external positioning system
     * @note Critical for indoor autonomous flight
     * @note Latency compensation important
     * 
     * @warning External system failures cause nav loss
     * @warning Coordinate frame misalignment causes errors
     * @warning System latency must be configured correctly
     * @warning Limited to external system coverage area
     * 
     * @see writeExtNavData() for external nav input
     * @see writeExtNavVelData() for velocity input
     * @see isExtNavUsedForYaw() for yaw usage check
     * @see extNavUsedForPos for position usage flag
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:649
     */
    bool readyToUseExtNav(void) const;

    /**
     * @brief Check for filter divergence and take corrective action
     * 
     * @details Monitors key filter health metrics to detect divergence (filter estimates
     *          drifting away from reality). When divergence detected, triggers resets or
     *          other recovery actions to restore filter health.
     * 
     * Divergence Indicators:
     * 
     * 1. Innovation Test Ratios:
     *    - velTestRatio > threshold: Velocity divergence
     *    - posTestRatio > threshold: Position divergence
     *    - hgtTestRatio > threshold: Height divergence
     *    - Sustained high ratios indicate problem
     * 
     * 2. State Magnitude Checks:
     *    - Position states exceeding limits
     *    - EK2_POSXY_STATE_LIMIT for horizontal
     *    - Velocity states unreasonably large
     *    - Indicates runaway estimation
     * 
     * 3. Covariance Checks:
     *    - Diagonal elements (variances) too large
     *    - Indicates low confidence / high uncertainty
     *    - Or covariances becoming negative (numerical error)
     * 
     * 4. Innovation Persistence:
     *    - Innovations consistently in one direction
     *    - Indicates systematic bias or model error
     *    - Or sensor calibration problem
     * 
     * 5. Timeout Flags:
     *    - velTimeout, posTimeout, hgtTimeout
     *    - Extended periods without valid fusion
     *    - Indicates measurement rejection
     * 
     * Divergence Detection Logic:
     * - Multiple checks must fail simultaneously
     * - Single metric spike may be false alarm
     * - Sustained problems trigger divergence flag
     * - Threshold typically 3-5x normal values
     * 
     * Corrective Actions:
     * 
     * 1. Position Reset:
     *    - ResetPosition() to GPS position
     *    - Corrects large position errors
     *    - Last resort for horizontal divergence
     * 
     * 2. Velocity Reset:
     *    - ResetVelocity() to GPS velocity
     *    - Corrects velocity drift
     *    - May indicate accelerometer bias
     * 
     * 3. Height Reset:
     *    - ResetHeight() to height sensor
     *    - Corrects vertical divergence
     *    - Switches height source if needed
     * 
     * 4. Covariance Reset:
     *    - Increase relevant covariance terms
     *    - Reflects increased uncertainty
     *    - Allows measurements to correct state
     * 
     * 5. Measurement Gating:
     *    - Temporarily widen innovation gates
     *    - Allow measurements to be accepted
     *    - Helps recovery from divergence
     * 
     * Causes of Divergence:
     * - Sensor calibration errors (IMU, compass, etc.)
     * - Sensor failures or large errors
     * - Model mismatch (incorrect assumptions)
     * - Numerical issues in covariance propagation
     * - Excessive vehicle dynamics
     * - Prolonged GPS outage
     * 
     * Prevention Strategies:
     * - Regular sensor calibration
     * - Conservative process noise tuning
     * - Adequate measurement updates
     * - Covariance limiting (ConstrainVariances)
     * - State limiting (ConstrainStates)
     * 
     * Impact of Divergence:
     * - Unreliable position/velocity estimates
     * - Vehicle may not fly correctly
     * - Autonomous modes may behave erratically
     * - Safety risk if not detected
     * 
     * Recovery Time:
     * - Immediate reset: Instantaneous position jump
     * - Gradual recovery: Seconds to minutes
     * - Depends on measurement availability
     * - May require landing and re-initialization
     * 
     * Logging:
     * - Divergence events logged
     * - Innovation test ratios logged
     * - Reset events logged
     * - Critical for post-flight analysis
     * 
     * @note Called periodically during UpdateFilter()
     * @note Critical safety function
     * @note Multiple checks prevent false triggers
     * @note Resets are last resort
     * 
     * @warning Divergence indicates serious problem
     * @warning May require landing and sensor check
     * @warning Frequent divergence needs investigation
     * @warning Review logs after divergence event
     * 
     * @see ResetPosition() for position reset
     * @see ResetVelocity() for velocity reset
     * @see ResetHeight() for height reset
     * @see velTestRatio, posTestRatio, hgtTestRatio
     * @see ConstrainStates() for state limiting
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:652
     */
    void checkDivergence(void);

    /**
     * @brief Calculate weighting applied to IMU1 accel data to blend IMU1 and IMU2
     * 
     * @details Computes the blend weight for accelerometer data when using multiple IMUs.
     *          Allows graceful degradation when one IMU becomes less healthy, by blending
     *          IMU1 and IMU2 accelerometer data with health-based weighting.
     * 
     * @param[in] K1 Health metric for IMU1 (higher = healthier, 0.0 to 1.0)
     * @param[in] K2 Health metric for IMU2 (higher = healthier, 0.0 to 1.0)
     * 
     * Blend Weight Calculation:
     * - Weight for IMU1 = K1 / (K1 + K2)
     * - Weight for IMU2 = K2 / (K1 + K2)
     * - Weights sum to 1.0 (normalized)
     * - Stored for use during IMU data reading
     * 
     * Health Metrics (K1, K2):
     * - Based on innovation consistency
     * - Based on sensor self-test results
     * - Based on sensor calibration status
     * - Based on detected sensor errors
     * - Range: 0.0 (failed) to 1.0 (perfect)
     * 
     * Blending Strategy:
     * - Smooth transition between IMUs
     * - Avoids sudden IMU switches
     * - Reduces impact of transient errors
     * - Maintains filter stability during IMU issues
     * 
     * Use Cases:
     * - One IMU developing errors
     * - IMU vibration affecting one sensor
     * - Temperature effects on one IMU
     * - Gradual IMU failure
     * - IMU startup transients
     * 
     * Delta Velocity Blending:
     * - Blended_delVel = W1 × IMU1_delVel + W2 × IMU2_delVel
     * - Applied during readIMUData()
     * - Weights (W1, W2) from this calculation
     * - Gyro data: Primary IMU selected, not blended
     * 
     * Gyro Handling:
     * - Gyros NOT blended (accel only)
     * - Gyro: Use healthiest IMU exclusively
     * - gyro_index_active: Selected gyro
     * - Blending gyros causes attitude errors
     * 
     * IMU Selection vs Blending:
     * - Selection: Switch entirely to better IMU
     * - Blending: Gradual weight transition
     * - Blending smoother than hard switch
     * - Avoids transients from switching
     * 
     * Weight Limits:
     * - Minimum weight: Prevents complete IMU exclusion
     * - Typically min weight ~0.1 even if unhealthy
     * - Allows recovery if IMU health improves
     * - Prevents numerical issues with zero weight
     * 
     * Configuration:
     * - Automatic based on health monitoring
     * - No user-configurable parameters for blending
     * - Driven by internal health assessments
     * 
     * Impact on Filter:
     * - Smoother response to IMU degradation
     * - Maintains state estimation continuity
     * - Reduces innovation spikes from IMU switch
     * - Better than hard IMU failover
     * 
     * Gyro Bias Estimation:
     * - Bias estimates per IMU maintained
     * - inactiveBias[] for non-primary IMUs
     * - Allows seamless IMU switching
     * - learnInactiveBiases() updates inactive IMU biases
     * 
     * Multiple IMU Systems:
     * - Some flight controllers have 3+ IMUs
     * - This implementation: 2-IMU blending
     * - Extension to 3+ IMUs possible
     * - Complexity increases with IMU count
     * 
     * @note Only accelerometer data blended
     * @note Gyro data uses single selected IMU
     * @note Weights based on health metrics
     * @note Smooth transition prevents transients
     * 
     * @warning K1 and K2 must be non-negative
     * @warning If both K1=0 and K2=0, default to equal weights
     * @warning Gyro must not be blended (causes attitude errors)
     * 
     * @see readIMUData() for blend weight application
     * @see learnInactiveBiases() for inactive IMU bias tracking
     * @see gyro_index_active for active gyro selection
     * @see accel_index_active for active accel (before blending)
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:655
     */
    void calcIMU_Weighting(ftype K1, ftype K2);

    /**
     * @brief Return true if optical flow data is available
     * 
     * @details Checks if valid optical flow measurements have been received recently
     *          and are available for fusion into the navigation filter.
     * 
     * @return true if optical flow data is present and valid, false otherwise
     * 
     * Optical Flow System:
     * - Optical flow sensor (PX4Flow, Cheerson, etc.)
     * - Measures apparent motion across ground
     * - Flow rates in rad/s about sensor X/Y axes
     * - Requires downward-facing camera/sensor
     * - Used for velocity estimation and position hold
     * 
     * Availability Checks:
     * - flowDataValid flag: Data recent and not timed out
     * - flowValidMeaTime_ms: Time of last valid measurement
     * - Timeout: Typically 1-2 seconds without data
     * - Quality threshold: Sufficient image features
     * 
     * Flow Data Elements:
     * - flowRadXY: Raw flow rates (rad/s) about sensor axes
     * - flowRadXYcomp: Gyro-compensated flow rates
     * - bodyRadXYZ: Gyro rates during flow measurement
     * - Quality indicator (0-255, higher = better)
     * 
     * Quality Requirements:
     * - Minimum quality threshold (e.g., > 50/255)
     * - Sufficient image texture/features
     * - Adequate lighting conditions
     * - Not over-saturated (flow too fast)
     * - Height above ground in sensor range
     * 
     * Height Requirements:
     * - Rangefinder providing height above ground
     * - Or known terrain altitude
     * - Flow rate → ground velocity requires height
     * - velocity = height × flow_rate × Tbn
     * 
     * Use Cases:
     * - GPS-denied position hold (indoor)
     * - Low-altitude flight over textured ground
     * - Precision landing
     * - Hover stabilization without GPS
     * - Velocity estimation backup to GPS
     * 
     * Sensor Mounting:
     * - Typically downward-facing
     * - body_offset: Sensor position in body frame
     * - Mounting angle compensation if not vertical
     * - Field of view considerations
     * 
     * Lighting Conditions:
     * - Requires adequate lighting
     * - Some sensors have onboard illumination
     * - Poor lighting → low quality or no data
     * - Outdoor: Affected by sun angle, shadows
     * 
     * Surface Requirements:
     * - Textured surface with features
     * - Featureless surfaces (carpet, water) poor
     * - High contrast features best
     * - Motion blur at high speeds
     * 
     * Height Above Ground:
     * - Too low: Field of view too small
     * - Too high: Features too small, flow too slow
     * - Optimal: 1-10 m depending on sensor
     * - Must be within rangefinder range
     * 
     * Data Rate:
     * - Typically 10-30 Hz
     * - Higher rate better for dynamic flight
     * - Latency affects performance
     * - Timestamped for proper fusion timing
     * 
     * Impact on Navigation:
     * - Enables GPS-denied velocity estimation
     * - Enables position hold without GPS
     * - Complements GPS when available
     * - Critical for indoor autonomous flight
     * 
     * @note Requires downward-facing sensor
     * @note Requires height above ground measurement
     * @note Quality depends on surface texture and lighting
     * @note Timeout if no data ~1-2 seconds
     * 
     * @warning Featureless surfaces cause failure
     * @warning Poor lighting degrades quality
     * @warning High altitude reduces accuracy
     * @warning Fast motion causes blur/saturation
     * 
     * @see FuseOptFlow() for optical flow fusion
     * @see SelectFlowFusion() for fusion scheduling
     * @see writeOptFlowMeas() for flow data input
     * @see flowDataValid for validity flag
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:658
     */
    bool optFlowDataPresent(void) const;

    /**
     * @brief Return true if we should use the range finder sensor
     * 
     * @details Determines whether rangefinder data should be used for height estimation
     *          based on sensor availability, configuration, flight phase, and terrain.
     * 
     * @return true if rangefinder should be used, false otherwise
     * 
     * Rangefinder Usage Conditions:
     * All must be true for rangefinder use:
     * 1. Rangefinder sensor enabled and configured
     * 2. Valid range measurements being received
     * 3. In range: Within min/max distance limits
     * 4. Terrain stable (if terrain stability required)
     * 5. Flight phase appropriate (e.g., low altitude)
     * 6. Innovation consistency acceptable
     * 
     * Rangefinder Types:
     * - Lidar (laser rangefinder)
     * - Sonar (ultrasonic)
     * - Radar (mmWave)
     * - Optical ToF (time-of-flight)
     * - Each has different characteristics
     * 
     * Range Limits:
     * - Minimum range: Typically 0.1-0.3 m
     * - Maximum range: 5-100 m depending on sensor
     * - Outside limits: Data not used
     * - Sensor-specific limits from AP_RangeFinder
     * 
     * Terrain Stability:
     * - terrainHgtStable flag: Terrain usable as reference
     * - Set by setTerrainHgtStable()
     * - False for moving over trees, vehicles, people
     * - True for stable ground, building floors
     * 
     * Flight Phase Considerations:
     * - Takeoff: May inhibit rangefinder initially
     * - Landing: Rangefinder critical
     * - High altitude: Out of range, use baro
     * - Low altitude: Rangefinder more accurate than baro
     * 
     * Height Source Selection:
     * - activeHgtSource determines primary height source
     * - May be HGT_SOURCE_RNG (rangefinder)
     * - selectHeightForFusion() chooses source
     * - Can switch sources during flight
     * 
     * Configuration Parameters:
     * - EK2_RNG_USE_HGT: Rangefinder use altitude (m)
     *   * Below this altitude, use rangefinder
     *   * Above this altitude, switch to baro
     * - EK2_RNG_USE_SPD: Rangefinder use speed (m/s)
     *   * Below this speed, use rangefinder
     *   * Prevents use during fast forward flight
     * 
     * Use Cases:
     * - Precision landing
     * - Low-altitude position hold
     * - Terrain following
     * - Obstacle avoidance (height component)
     * - Indoor flight (more accurate than baro)
     * 
     * Advantages Over Barometer:
     * - Terrain-relative (not pressure-relative)
     * - Fast response, no lag
     * - Not affected by weather pressure changes
     * - More accurate at low altitudes
     * 
     * Disadvantages:
     * - Limited range
     * - Terrain-relative (not absolute altitude)
     * - Affected by terrain features
     * - May give false readings (vegetation, etc.)
     * 
     * Sensor Quality Checks:
     * - Innovation consistency: Range innovation within limits
     * - Range rate: Consistent with velocity estimate
     * - Not detecting false targets
     * - Signal quality adequate (sensor-specific)
     * 
     * Impact on Height Estimation:
     * - Provides terrain-relative height
     * - More accurate for low-altitude ops
     * - Enables precision altitude control
     * - Critical for landing flare
     * 
     * Terrain Offset Estimation:
     * - EstimateTerrainOffset() estimates ground level
     * - terrainState: Estimated terrain altitude
     * - Allows terrain-following
     * - Separates vehicle altitude from terrain changes
     * 
     * Multiple Rangefinders:
     * - Some systems have multiple rangefinders
     * - Forward-looking: Obstacle detection
     * - Downward: Height measurement
     * - Only downward used for height
     * 
     * @note Terrain-relative, not absolute altitude
     * @note Limited range (typically < 100 m)
     * @note Affected by terrain features
     * @note More accurate than baro at low altitude
     * 
     * @warning Not usable at high altitudes (out of range)
     * @warning Requires stable terrain for accuracy
     * @warning May give false readings over vegetation
     * @warning Fast forward flight may affect accuracy
     * 
     * @see selectHeightForFusion() for height source selection
     * @see EstimateTerrainOffset() for terrain estimation
     * @see readRangeFinder() for range data reading
     * @see terrainHgtStable for terrain stability flag
     * @see activeHgtSource for current height source
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:661
     */
    bool useRngFinder(void) const;

    /**
     * @brief Determine when to perform fusion of optical flow measurements
     * 
     * @details Schedules optical flow measurement fusion into the EKF based on data
     *          availability, innovation consistency, and filter state. Manages the
     *          timing and conditions for flow fusion.
     * 
     * Fusion Scheduling Logic:
     * 1. Check if flow data available (optFlowDataPresent)
     * 2. Check if terrain offset estimate valid
     * 3. Verify innovation consistency acceptable
     * 4. Check minimum time since last fusion
     * 5. Set flowDataToFuse flag if conditions met
     * 
     * Optical Flow Requirements:
     * - Valid flow measurements available
     * - Height above ground known (rangefinder)
     * - Sufficient image quality/features
     * - Not timed out (data recent)
     * - Terrain offset estimate converged
     * 
     * Terrain Offset Requirement:
     * - EstimateTerrainOffset() must have converged
     * - terrainState: Estimated ground level
     * - Popt: Terrain covariance acceptable
     * - Without terrain estimate, can't fuse flow
     * 
     * Innovation Consistency:
     * - flowTestRatio: Flow innovation test ratio
     * - If flowTestRatio > threshold: Reject fusion
     * - Large innovations indicate:
     *   * Terrain change
     *   * Sensor error
     *   * Filter divergence
     *   * Wind gust
     * 
     * Fusion Rate Limiting:
     * - prevFlowFuseTime_ms: Last fusion time
     * - Minimum interval between fusions
     * - Prevents excessive computation
     * - Typically fuse at flow data rate (10-30 Hz)
     * 
     * Height Above Ground Check:
     * - Requires valid height AGL
     * - From rangefinder or terrain database
     * - Flow rate → velocity = height × flow × Tbn
     * - Invalid height → can't convert flow to velocity
     * 
     * Gyro Compensation:
     * - flowRadXYcomp: Gyro-compensated flow
     * - Removes vehicle rotation from flow
     * - bodyRadXYZ: Gyro rates during measurement
     * - Compensation critical for accuracy
     * 
     * Fusion Inhibit Conditions:
     * - Excessive tilt angle (flow unreliable)
     * - Height above ground too high (out of range)
     * - Terrain offset not converged
     * - Poor flow quality indicator
     * - Recent flow sensor reset/switch
     * 
     * Optical Flow Use Cases:
     * - GPS-denied velocity estimation
     * - Position hold without GPS (indoor)
     * - Velocity backup to GPS
     * - Low-altitude precision control
     * 
     * Impact on Filter States:
     * - Updates velocity states (3-5)
     * - Updates position states (6-8) indirectly
     * - Enables position hold without GPS
     * - Reduces GPS velocity reliance
     * 
     * Fusion Process:
     * - FuseOptFlow() performs actual fusion
     * - Sequential fusion: X-axis then Y-axis
     * - Kalman update for each axis
     * - Innovation, gain, covariance update
     * 
     * Auxiliary Terrain Estimator:
     * - Single-state EKF for terrain offset
     * - Runs in parallel with main filter
     * - EstimateTerrainOffset() updates terrain
     * - Provides height AGL for flow fusion
     * 
     * Coordinate Transformations:
     * - Flow in sensor frame (X, Y axes)
     * - Tbn_flow: Body to NED rotation during flow
     * - Transform flow to NED velocity
     * - Account for sensor body offset
     * 
     * @note Requires height above ground measurement
     * @note Gyro compensation critical
     * @note Terrain offset must converge first
     * @note Enables GPS-denied flight
     * 
     * @warning Large tilt angles degrade accuracy
     * @warning Requires textured surface
     * @warning High altitude reduces effectiveness
     * @warning Poor lighting affects quality
     * 
     * @see FuseOptFlow() for optical flow fusion
     * @see EstimateTerrainOffset() for terrain estimation
     * @see optFlowDataPresent() for data availability
     * @see flowDataToFuse flag for fusion trigger
     * @see flowTestRatio for innovation consistency
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:664
     */
    void SelectFlowFusion();

    /**
     * @brief Estimate terrain offset using a single state EKF
     * 
     * @details Implements a simple 1-state Extended Kalman Filter to estimate the
     *          terrain altitude (ground level) beneath the vehicle. This terrain
     *          estimate is used to convert optical flow measurements to velocities
     *          and to provide terrain-relative altitude.
     * 
     * Single-State Terrain Filter:
     * - State: terrainState (terrain altitude in NED Down, meters)
     * - Covariance: Popt (terrain altitude variance, m²)
     * - Measurements: Rangefinder + vehicle altitude
     * - Simple enough to run at high rate
     * 
     * Terrain State Definition:
     * - terrainState: Down position of ground in NED frame (m)
     * - Positive down (NED convention)
     * - Relative to EKF origin
     * - Updated by rangefinder measurements
     * 
     * Measurement Model:
     * - Rangefinder measures: height above ground
     * - Vehicle Down position: stateStruct.position[2]
     * - Expected range = terrainState - vehicle_Down
     * - Innovation = measured_range - expected_range
     * 
     * Filter Update Process:
     * 1. Predict: terrainState unchanged (stationary terrain)
     * 2. Predict covariance: Popt += process_noise
     * 3. Measurement: Rangefinder reading
     * 4. Innovation: measured_hgt - estimated_hgt
     * 5. Kalman gain: Popt / (Popt + R_rng)
     * 6. Update state: terrainState += gain × innovation
     * 7. Update covariance: Popt = (1 - gain) × Popt
     * 
     * Process Noise:
     * - Accounts for slowly changing terrain
     * - Allows terrain estimate to adapt
     * - Small for flat terrain
     * - Larger if terrain may change
     * 
     * Measurement Noise:
     * - R_LOS: Rangefinder measurement variance
     * - Includes sensor noise
     * - Includes terrain roughness
     * - Typically 0.1-1.0 m²
     * 
     * Terrain Offset Use:
     * - Optical flow fusion: Height AGL = vehicle_alt - terrainState
     * - Flow velocity = height_AGL × flow_rate × Tbn
     * - Terrain following: Maintain constant height AGL
     * - Obstacle detection: Ground reference
     * 
     * Initialization:
     * - First rangefinder reading sets initial terrain
     * - terrainState = vehicle_Down + range_reading
     * - Popt = large initial uncertainty
     * - Converges over several measurements
     * 
     * Convergence:
     * - Popt decreases with measurements
     * - Typically converges in 1-2 seconds
     * - Convergence required before flow fusion
     * - gndHgtValidTime_ms: Last valid terrain update
     * 
     * Terrain Changes:
     * - Filter adapts to changing terrain
     * - Process noise allows tracking
     * - Rapid changes may cause temporary errors
     * - Flying over buildings, trees, etc.
     * 
     * Validity Checks:
     * - gndOffsetValid: Terrain estimate valid
     * - Recent rangefinder data required
     * - Covariance within acceptable limits
     * - Not diverged from vehicle altitude
     * 
     * Inhibit Conditions:
     * - inhibitGndState: Hold terrain constant
     * - Set when terrain unstable
     * - Set when rangefinder out of range
     * - Set when flying over moving objects
     * 
     * Use Cases:
     * - Terrain-relative altitude hold
     * - Optical flow velocity estimation
     * - Terrain following
     * - Precision landing (terrain gradient)
     * - Obstacle detection reference
     * 
     * Advantages of Simple Filter:
     * - Computationally cheap
     * - Runs at rangefinder rate (high Hz)
     * - Robust and stable
     * - Independent of main filter
     * 
     * Limitations:
     * - Assumes terrain stationary
     * - Single measurement source (rangefinder)
     * - No horizontal terrain model
     * - Limited to directly beneath vehicle
     * 
     * Integration with Main Filter:
     * - Terrain estimate used by main filter
     * - But computed independently
     * - Prevents main filter divergence from terrain errors
     * - Auxiliary filter isolates terrain uncertainty
     * 
     * @note Simple 1-state EKF for terrain altitude
     * @note Requires rangefinder measurements
     * @note Converges in 1-2 seconds typically
     * @note Used for optical flow height AGL
     * 
     * @warning Assumes terrain stationary
     * @warning Rapid terrain changes cause temporary errors
     * @warning Flying over moving objects invalid
     * @warning Requires valid rangefinder data
     * 
     * @see FuseOptFlow() for optical flow fusion using terrain
     * @see useRngFinder() for rangefinder usage check
     * @see terrainState for terrain altitude state
     * @see Popt for terrain covariance
     * @see gndOffsetValid for validity flag
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:667
     */
    void EstimateTerrainOffset();

    /**
     * @brief Fuse optical flow measurements into the main filter
     * 
     * @details Performs the Kalman filter update step using optical flow measurements.
     *          Fuses gyro-compensated flow rates to update velocity and position states.
     *          Uses terrain offset estimate to convert flow rates to velocities.
     * 
     * Optical Flow Fusion Process:
     * 1. Retrieve flow measurement: ofDataDelayed
     * 2. Get terrain offset: terrainState
     * 3. Calculate height above ground: vehicle_alt - terrain
     * 4. Predict flow rates: LOS rates from state estimate
     * 5. Compute innovations: measured_flow - predicted_flow
     * 6. Calculate innovation variance (S = H × P × H' + R)
     * 7. Compute Kalman gain (K = P × H' / S)
     * 8. Update states: state += K × innovation
     * 9. Update covariance: P = (I - K × H) × P
     * 
     * Measurement Model:
     * - Flow rate (rad/s) = velocity (m/s) / height_AGL (m)
     * - Transformed to body frame using Tbn_flow
     * - LOS (Line of Sight) rates about sensor X/Y axes
     * - Gyro compensation already applied
     * 
     * Height Above Ground:
     * - From terrain offset estimator
     * - height_AGL = stateStruct.position[2] - terrainState
     * - Critical for flow-to-velocity conversion
     * - Invalid height → can't fuse flow
     * 
     * Innovation Calculation:
     * - X-axis: innovOptFlow[0] = measured_flowX - predicted_flowX
     * - Y-axis: innovOptFlow[1] = measured_flowY - predicted_flowY
     * - Sequential fusion: X then Y
     * - Independent updates for each axis
     * 
     * Innovation Variance:
     * - varInnovOptFlow[]: Innovation variance for each axis
     * - Includes measurement noise (R_LOS)
     * - Includes state uncertainty (H × P × H')
     * - Used for innovation consistency check
     * 
     * Innovation Consistency:
     * - flowTestRatio: Normalized innovation magnitude
     * - flowTestRatio[i] = innov[i]² / varInnov[i]
     * - Threshold typically 3.0-5.0 (chi-squared)
     * - If exceeded: Don't fuse this axis
     * 
     * Kalman Gain:
     * - Computed for each measurement axis
     * - Determines state update magnitude
     * - Higher gain → trust measurement more
     * - Lower gain → trust state estimate more
     * 
     * State Updates:
     * - Primary: Velocity states (3-5) NED velocity
     * - Secondary: Position states (6-8) NED position
     * - Also updates: Attitude error, gyro bias, wind
     * - Full state update, not just velocity
     * 
     * Covariance Update:
     * - Joseph form for numerical stability
     * - Maintains positive definite covariance
     * - Updates full P matrix
     * - Accounts for state correlations
     * 
     * Body Frame Offset:
     * - ofDataDelayed.body_offset: Sensor position in body
     * - Velocity at sensor ≠ velocity at IMU
     * - Account for lever arm effect
     * - Important for large offsets
     * 
     * Tbn_flow Matrix:
     * - Body to NED rotation at flow measurement time
     * - Stored when flow received
     * - Accounts for vehicle attitude during flow
     * - Critical for accurate velocity estimation
     * 
     * Gyro Compensation:
     * - flowRadXYcomp: Already gyro-compensated
     * - Vehicle rotation removed from flow
     * - Uses bodyRadXYZ gyro measurements
     * - Prevents attitude motion affecting flow
     * 
     * Sequential Fusion:
     * - X-axis fusion first
     * - Y-axis fusion second
     * - Spreads computational load
     * - Maintains filter stability
     * 
     * Fusion Frequency:
     * - Typically 10-30 Hz (flow sensor rate)
     * - Matched to flow measurement availability
     * - SelectFlowFusion() schedules fusion
     * 
     * Impact on Navigation:
     * - Updates velocity estimate
     * - Enables position hold without GPS
     * - Provides velocity backup to GPS
     * - Critical for GPS-denied flight
     * 
     * Auxiliary Innovations:
     * - auxFlowObsInnov: Terrain estimator innovations
     * - Used by EstimateTerrainOffset()
     * - Separate from main filter innovations
     * 
     * @note Requires valid terrain offset estimate
     * @note Sequential fusion (X then Y axis)
     * @note Updates full state vector
     * @note Enables GPS-denied velocity estimation
     * 
     * @warning Large tilt angles degrade accuracy
     * @warning Invalid terrain estimate causes errors
     * @warning Body offset must be correct
     * @warning Gyro compensation critical
     * 
     * @see SelectFlowFusion() for fusion scheduling
     * @see EstimateTerrainOffset() for terrain estimation
     * @see writeOptFlowMeas() for flow data input
     * @see innovOptFlow for flow innovations
     * @see flowTestRatio for consistency check
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:670
     */
    void FuseOptFlow();

    /**
     * @brief Control filter mode changes
     * 
     * @details High-level filter mode state machine that manages transitions between
     *          different navigation modes based on sensor availability, vehicle state,
     *          and configuration. Coordinates multiple subsystems for mode changes.
     * 
     * Filter Mode Categories:
     * 
     * 1. Initialization Modes:
     *    - Pre-initialization: Waiting for sensors
     *    - Bootstrap: Initial alignment from static
     *    - Initialization complete: Ready for flight
     * 
     * 2. Aiding Modes (PV_AidingMode):
     *    - AID_ABSOLUTE: GPS or absolute position aiding
     *    - AID_RELATIVE: Optical flow only (relative position)
     *    - AID_NONE: No position aiding (attitude only)
     * 
     * 3. Height Source Modes:
     *    - Baro: Barometer for height
     *    - Rangefinder: Range sensor for height
     *    - GPS: GPS altitude
     *    - Beacon: Beacon system altitude
     *    - External Nav: External navigation altitude
     * 
     * Mode Control Flow:
     * 1. Check vehicle state (detectFlight)
     * 2. Evaluate sensor availability
     * 3. Check transition conditions
     * 4. Execute mode transition if needed
     * 5. Update filter configuration for new mode
     * 6. Log mode changes
     * 
     * Aiding Mode Transitions:
     * - GPS available → AID_ABSOLUTE
     * - GPS lost, flow available → AID_RELATIVE
     * - All aiding lost → AID_NONE
     * - Mode changes affect state learning
     * 
     * Height Source Transitions:
     * - selectHeightForFusion() chooses source
     * - activeHgtSource updated
     * - May require height reset on switch
     * - Hysteresis prevents rapid switching
     * 
     * Transition Triggers:
     * - Sensor timeout (GPS, flow, etc.)
     * - Sensor recovery (was lost, now valid)
     * - Configuration changes
     * - Flight state changes (takeoff, landing)
     * - Innovation failures
     * - Filter divergence
     * 
     * On-Ground vs In-Flight:
     * - detectFlight() determines flight state
     * - onGround flag: Vehicle on ground
     * - inFlight flag: Vehicle airborne
     * - Different modes/behaviors for each
     * 
     * Initialization Protection:
     * - Prevent mode changes during startup
     * - Wait for filter convergence
     * - tiltAlignComplete: Attitude aligned
     * - yawAlignComplete: Heading aligned
     * 
     * Wind/Mag State Learning:
     * - setWindMagStateLearningMode() controls
     * - Enable/disable based on aiding mode
     * - inhibitWindStates, inhibitMagStates flags
     * - Prevents divergence without aiding
     * 
     * Yaw and Mag Field Control:
     * - controlMagYawReset() manages resets
     * - Post-takeoff mag field initialization
     * - Yaw resets when needed
     * - Magnetic anomaly detection
     * 
     * Optical Flow Transitions:
     * - Takeoff detection: detectOptFlowTakeoff()
     * - Enable flow fusion after takeoff
     * - Requires stable terrain initially
     * - Height AGL convergence
     * 
     * External Nav Transitions:
     * - External nav available/lost
     * - May use for position and/or yaw
     * - Coordinate frame alignment
     * - Position reset on external nav reset
     * 
     * Configuration Changes:
     * - Parameter changes during operation
     * - Sensor enable/disable via parameters
     * - Fusion source selection updates
     * - May trigger filter reset
     * 
     * Mode Change Actions:
     * - Reset states if large mode change
     * - Adjust covariances for new mode
     * - Enable/disable state learning
     * - Update state indexing limits
     * - Log mode transition event
     * 
     * Hysteresis:
     * - Prevent rapid mode switching
     * - Require sustained conditions for change
     * - Timeouts before reverting
     * - Prevents oscillation between modes
     * 
     * Safety Considerations:
     * - Smooth transitions critical
     * - Avoid step changes in estimates
     * - Maintain attitude estimate always
     * - Graceful degradation on sensor loss
     * 
     * Impact on Navigation:
     * - Determines which sensors fused
     * - Affects state estimation accuracy
     * - Enables appropriate flight modes
     * - Critical for GPS-denied capability
     * 
     * @note Called every filter update cycle
     * @note Coordinates multiple subsystems
     * @note Smooth transitions critical
     * @note Maintains filter health through mode changes
     * 
     * @warning Improper mode transitions cause divergence
     * @warning Must maintain attitude estimate in all modes
     * @warning Mode changes can cause transient errors
     * 
     * @see detectFlight() for flight state detection
     * @see setAidingMode() for aiding mode selection
     * @see setWindMagStateLearningMode() for state learning
     * @see controlMagYawReset() for yaw/mag management
     * @see selectHeightForFusion() for height source selection
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:673
     */
    void controlFilterModes();

    /**
     * @brief Determine if we are flying or on the ground
     * 
     * @details Flight state detection using IMU, barometer, GPS, and other sensor data
     *          to determine if vehicle is on ground or airborne. Used to adapt filter
     *          behavior and enable/disable flight-specific features.
     * 
     * Flight Detection Criteria:
     * 
     * Takeoff Detection (Ground → Flight):
     * - Positive vertical velocity (climbing)
     * - Sustained vertical acceleration
     * - Airspeed above threshold (fixed-wing)
     * - GPS horizontal velocity above threshold
     * - Time since motors armed
     * - All criteria must persist for confidence
     * 
     * Landing Detection (Flight → Ground):
     * - Near-zero vertical velocity
     * - Low horizontal velocity
     * - Low vertical acceleration variance
     * - Airspeed near zero (fixed-wing)
     * - All criteria sustained for duration
     * 
     * Flight State Flags:
     * - onGround: true when definitely on ground
     * - inFlight: true when definitely airborne
     * - prevOnGround, prevInFlight: Previous states
     * - Used to detect state transitions
     * 
     * Vertical Velocity Check:
     * - Climb rate from velocity state (stateStruct.velocity[2])
     * - Threshold: Typically > 0.5 m/s sustained
     * - Must persist to avoid false takeoff detection
     * - Accounts for measurement noise
     * 
     * Horizontal Velocity Check:
     * - GPS or flow-based velocity
     * - Threshold: Typically > 2 m/s for flight
     * - Important for fixed-wing
     * - Less critical for multirotors
     * 
     * Airspeed Check (Fixed-Wing):
     * - True airspeed above threshold
     * - Typically > 5-10 m/s for flight
     * - Indicates forward flight
     * - Not applicable to multirotors
     * 
     * Barometer Check:
     * - Altitude change from takeoff position
     * - posDownAtTakeoff: Initial altitude
     * - Threshold: Typically > 2 m altitude gain
     * - Subject to baro drift and weather
     * 
     * Time Since Arming:
     * - timeAtArming_ms: Time motors armed
     * - Prevent premature takeoff detection
     * - Allow IMU/sensors to settle
     * - Typically wait ~2 seconds after arming
     * 
     * Confidence Filtering:
     * - Multiple criteria must agree
     * - Sustained conditions required (not transient)
     * - Hysteresis: Different thresholds for takeoff vs landing
     * - Prevents rapid state oscillation
     * 
     * State Transitions:
     * - onGround → inFlight: Takeoff detected
     *   * Store takeoff altitude: posDownAtTakeoff
     *   * Enable flight-specific features
     *   * Update airborneDetectTime_ms
     * - inFlight → onGround: Landing detected
     *   * Disable flight features
     *   * May reset certain states
     * 
     * Use of Flight State:
     * 
     * When onGround:
     * - Disable wind state learning (no motion)
     * - Tighter constraints on states
     * - Pre-flight alignment allowed
     * - May inhibit some sensor fusion
     * - GPS position resets allowed
     * 
     * When inFlight:
     * - Enable wind state learning
     * - Looser state constraints
     * - No position resets (unsafe)
     * - Full sensor fusion enabled
     * - Post-takeoff initializations
     * 
     * Manoeuvring Detection:
     * - manoeuvring flag: Horizontal acceleration
     * - accNavMagHoriz: Horizontal accel magnitude
     * - Affects measurement noise scaling
     * - Different thresholds when manoeuvring
     * 
     * Takeoff-Specific Actions:
     * - Post-takeoff mag field initialization
     * - Post-takeoff yaw reset (if needed)
     * - Enable optical flow (detectOptFlowTakeoff)
     * - Reset/initialize certain states
     * 
     * Landing-Specific Actions:
     * - May reset height to ground
     * - Disable optical flow
     * - Prepare for next takeoff
     * - Log landing event
     * 
     * Vehicle Type Differences:
     * - Multirotor: Vertical takeoff/landing
     * - Fixed-wing: Requires horizontal velocity
     * - VTOL: Complex transition logic
     * - Ground vehicle: Always onGround=true
     * 
     * @note Called every filter update cycle
     * @note Multiple sensors used for robustness
     * @note Hysteresis prevents false detections
     * @note Critical for mode transitions
     * 
     * @warning False detection causes incorrect behavior
     * @warning Affects safety-critical features
     * @warning Must be reliable and robust
     * @warning Vehicle-type specific tuning needed
     * 
     * @see onGround flag for ground state
     * @see inFlight flag for flight state
     * @see prevOnGround, prevInFlight for transition detection
     * @see controlFilterModes() for mode management
     * @see detectOptFlowTakeoff() for optical flow takeoff
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:676
     */
    void detectFlight();

    /**
     * @brief Set inertial navigation aiding mode
     * 
     * @details Determines and sets the primary aiding mode (absolute, relative, or none)
     *          based on available sensors and their health. Updates PV_AidingMode to
     *          reflect the current navigation capability and fusion sources.
     * 
     * @note PV = Position and Velocity
     * 
     * Aiding Modes:
     * 
     * 1. AID_ABSOLUTE (GPS or equivalent):
     *    - GPS providing position and velocity
     *    - Or external navigation system (motion capture)
     *    - Or range beacon system
     *    - Position estimates are absolute (earth-referenced)
     *    - Highest accuracy and capability
     * 
     * 2. AID_RELATIVE (Optical flow only):
     *    - Optical flow providing velocity
     *    - No absolute position reference
     *    - Position drifts over time
     *    - Suitable for position hold (short term)
     *    - Not suitable for waypoint navigation
     * 
     * 3. AID_NONE (No position aiding):
     *    - Only IMU and magnetometer
     *    - Attitude and heading only
     *    - No velocity or position estimation
     *    - Or velocity/position highly unreliable
     *    - Emergency degraded mode
     * 
     * Mode Selection Logic:
     * 
     * Priority 1: Absolute Aiding
     * - If GPS healthy: → AID_ABSOLUTE
     * - Or external nav healthy: → AID_ABSOLUTE
     * - Or range beacons healthy: → AID_ABSOLUTE
     * 
     * Priority 2: Relative Aiding
     * - If optical flow healthy: → AID_RELATIVE
     * - Requires rangefinder for height
     * - No absolute position, but velocity OK
     * 
     * Priority 3: No Aiding
     * - All aiding sources failed: → AID_NONE
     * - Fallback emergency mode
     * - Attitude only
     * 
     * GPS Health Criteria:
     * - Recent GPS data received
     * - GPS quality acceptable (gpsGoodToAlign)
     * - Not timed out (velTimeout, posTimeout)
     * - Innovation consistency passing
     * 
     * Optical Flow Health:
     * - Flow data available (optFlowDataPresent)
     * - Terrain offset converged
     * - Innovation consistency acceptable
     * - Rangefinder providing height
     * 
     * External Nav Health:
     * - External nav data recent
     * - Position and orientation valid
     * - Innovation consistency acceptable
     * - Not timed out
     * 
     * Range Beacon Health:
     * - Sufficient beacons (≥3)
     * - Recent range measurements
     * - Innovation consistency acceptable
     * - Alignment complete
     * 
     * Mode Transition Actions:
     * 
     * Entering AID_ABSOLUTE:
     * - Enable wind state learning
     * - Enable magnetic field learning
     * - Enable position/velocity updates
     * - Full navigation capability
     * 
     * Entering AID_RELATIVE:
     * - May inhibit magnetic field learning
     * - Wind learning limited
     * - Position drifts (relative only)
     * - Suitable for loiter, not waypoints
     * 
     * Entering AID_NONE:
     * - Inhibit wind state learning
     * - Inhibit magnetic field learning
     * - May inhibit position/velocity states
     * - Attitude and heading only
     * - Or constVelMode / constPosMode
     * 
     * State Learning Impact:
     * - setWindMagStateLearningMode() called
     * - Determines which states to learn
     * - stateIndexLim: Limit state updates
     * - Prevents divergence without aiding
     * 
     * Constrained Modes:
     * - constVelMode: Assume zero velocity
     * - constPosMode: Assume fixed position
     * - Used in AID_NONE to constrain drift
     * - Prevents velocity/position divergence
     * 
     * Mode Persistence:
     * - PV_AidingModePrev: Previous mode
     * - Detect transitions
     * - Apply transition logic
     * - Log mode change events
     * 
     * Flight Mode Impact:
     * - AID_ABSOLUTE: All modes available
     *   * Auto missions, guided, loiter, RTL
     * - AID_RELATIVE: Limited modes
     *   * Loiter possible (short term)
     *   * No waypoint missions
     * - AID_NONE: Minimal modes
     *   * Stabilize, altitude hold
     *   * No autonomous position control
     * 
     * Configuration Influence:
     * - EK2_GPS_TYPE: GPS usage mode
     * - Fusion source enable flags
     * - May allow/prevent certain aiding modes
     * 
     * Safety Considerations:
     * - Mode degradation must be graceful
     * - Vehicle should remain controllable
     * - Notify pilot of mode changes
     * - May trigger failsafe actions
     * 
     * @note Updates PV_AidingMode based on sensor availability
     * @note Highest quality aiding source selected
     * @note Affects state learning and fusion
     * @note Critical for navigation capability
     * 
     * @warning AID_NONE provides minimal navigation
     * @warning AID_RELATIVE position drifts over time
     * @warning Mode changes may cause transients
     * @warning Pilot must be aware of current mode
     * 
     * @see PV_AidingMode for current aiding mode
     * @see PV_AidingModePrev for previous mode
     * @see setWindMagStateLearningMode() for state learning
     * @see controlFilterModes() for mode management
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:679
     */
    void setAidingMode();

    /**
     * @brief Determine if learning of wind and magnetic field states will be enabled
     * 
     * @details Controls whether wind velocity states (22-23) and magnetic field states
     *          (16-21) are updated or held constant. Sets stateIndexLim to avoid
     *          unnecessary operations on inhibited states. Prevents divergence of
     *          these states when insufficient observability.
     * 
     * State Learning Control:
     * - inhibitWindStates: If true, hold wind states constant
     * - inhibitMagStates: If true, hold mag field states constant
     * - stateIndexLim: Maximum state index to update
     * - Optimization: Don't update inhibited states
     * 
     * Wind State Learning (States 22-23):
     * 
     * Enable Wind Learning When:
     * - Airspeed sensor available and healthy
     * - Or GPS available with reasonable velocity
     * - Vehicle in flight (wind affects flight)
     * - Aiding mode: AID_ABSOLUTE or sometimes AID_RELATIVE
     * 
     * Disable Wind Learning When:
     * - No airspeed sensor
     * - GPS not available
     * - Vehicle on ground (no wind effect)
     * - Aiding mode: AID_NONE
     * - Insufficient observability
     * 
     * Wind State Observability:
     * - Airspeed + GPS velocity → wind velocity
     * - Without both: Wind unobservable
     * - GPS ground track + airspeed → wind
     * - In hover: Wind less observable
     * 
     * Magnetic Field Learning (States 16-21):
     * 
     * Enable Mag Learning When:
     * - Magnetometer available and healthy
     * - Absolute aiding available (GPS, etc.)
     * - Vehicle moving (better observability)
     * - Sufficient excitation for learning
     * 
     * Disable Mag Learning When:
     * - Magnetometer not available
     * - No absolute aiding (AID_NONE or AID_RELATIVE)
     * - Vehicle static (poor observability)
     * - Magnetic anomaly detected
     * - Compass learning disabled by config
     * 
     * State Index Limit (stateIndexLim):
     * 
     * Purpose: Optimize covariance operations
     * - Skip updates for inhibited states
     * - Reduces computation in CovariancePrediction
     * - Reduces computation in state updates
     * 
     * Typical Values:
     * - 23: Update all states (full 24-state filter)
     * - 21: Inhibit wind states (states 22-23)
     * - 15: Inhibit mag and wind (states 16-23)
     * - Optimization for inhibited state ranges
     * 
     * State Groups and Inhibition:
     * - States 0-15: Always updated (core states)
     *   * Attitude error, velocity, position
     *   * Gyro bias, gyro scale, accel Z bias
     * - States 16-21: Mag field (inhibitable)
     *   * Earth mag field (16-18)
     *   * Body mag field (19-21)
     * - States 22-23: Wind velocity (inhibitable)
     * - State 24-27: Quaternion (always updated)
     * 
     * Aiding Mode Impact:
     * 
     * AID_ABSOLUTE:
     * - Enable wind learning (if airspeed available)
     * - Enable mag learning
     * - stateIndexLim = 23 (full learning)
     * 
     * AID_RELATIVE:
     * - May inhibit mag learning
     * - May inhibit wind learning
     * - stateIndexLim reduced
     * 
     * AID_NONE:
     * - Inhibit both wind and mag learning
     * - stateIndexLim = 15
     * - Prevent divergence without aiding
     * 
     * Effective Mag Calibration:
     * - effective_magCal() determines calibration mode
     * - Affects mag state learning enable
     * - Configuration parameter: MAG_CAL
     * 
     * Compass Learning Parameters:
     * - COMPASS_LEARN: Enable/disable compass learning
     * - Affects inhibitMagStates
     * - User can disable mag learning
     * 
     * Performance Optimization:
     * - Inhibited states: No covariance updates
     * - Saves computation in prediction step
     * - Saves computation in measurement updates
     * - Significant for embedded processors
     * 
     * Impact on Navigation:
     * - Wind states: Affect airspeed fusion, wind estimation
     * - Mag states: Affect heading accuracy
     * - Inhibition: Prevents divergence but limits learning
     * - Trade-off: Observability vs stability
     * 
     * State Reset Flags:
     * - lastInhibitMagStates: Previous mag inhibit state
     * - needMagBodyVarReset: Reset mag variances on transition
     * - Tracks state learning enable changes
     * 
     * @note Called by controlFilterModes() each update
     * @note Prevents divergence without observability
     * @note Optimizes computation for inhibited states
     * @note Critical for filter stability
     * 
     * @warning Inhibiting states prevents learning
     * @warning But necessary without observability
     * @warning stateIndexLim must match inhibition
     * @warning Improper inhibition causes errors
     * 
     * @see inhibitWindStates flag for wind inhibition
     * @see inhibitMagStates flag for mag inhibition
     * @see stateIndexLim for state update limit
     * @see setAidingMode() for aiding mode selection
     * @see PV_AidingMode for current aiding mode
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:682-684
     */
    void setWindMagStateLearningMode();

    /**
     * @brief Check the alignment status of the tilt attitude
     * 
     * @details Monitors convergence of roll and pitch estimates during initial
     *          bootstrap alignment. Sets tiltAlignComplete flag when tilt has
     *          converged sufficiently for filter to proceed to full operation.
     * 
     * Tilt Alignment Purpose:
     * - Roll and pitch must converge before full filter operation
     * - Uses accelerometer gravity vector for tilt
     * - Critical for correct velocity and position estimation
     * - Prerequisite for yaw alignment
     * 
     * Alignment Process:
     * 
     * Initial Tilt Estimate:
     * - From accelerometer: Measures gravity vector
     * - Roll and pitch from gravity direction
     * - Assumes vehicle stationary
     * - InitialiseFilterBootstrap() provides initial estimate
     * 
     * Convergence Monitoring:
     * - tiltErrFilt: Filtered tilt error magnitude
     * - Computed from attitude error states (angErr[0:2])
     * - Low-pass filtered for stability
     * - Threshold: Typically < 1-2 degrees
     * 
     * Convergence Criteria:
     * - tiltErrFilt < threshold (e.g., 0.02 rad ≈ 1.1°)
     * - Sustained for duration (not transient)
     * - Innovation consistency acceptable
     * - State covariance converged
     * 
     * Tilt Error Calculation:
     * - tiltErrVec: Attitude error from vel/pos fusion
     * - Magnitude: sqrt(roll_err² + pitch_err²)
     * - Filtered: tiltErrFilt (low-pass)
     * - Reflects alignment quality
     * 
     * Use of Velocity/Position Updates:
     * - GPS velocity constrains attitude
     * - Horizontal velocity perpendicular to gravity
     * - Velocity errors reveal tilt errors
     * - FuseVelPosNED() provides attitude correction
     * 
     * Alignment Flags:
     * - tiltAlignComplete: true when aligned
     * - yawAlignComplete: true when heading aligned
     * - Both required for full filter operation
     * - magStateInitComplete: Mag field initialized
     * 
     * Bootstrap Sequence:
     * 1. Initialize states (InitialiseFilterBootstrap)
     * 2. Wait for tilt convergence (this function)
     * 3. Initialize yaw and mag field
     * 4. Full filter operation
     * 
     * Required Conditions:
     * - Vehicle stationary (on ground)
     * - Level surface (or handle tilt properly)
     * - Accelerometer healthy
     * - GPS providing velocity (for alignment check)
     * 
     * Time to Align:
     * - Typically 5-10 seconds
     * - Depends on filter tuning
     * - Depends on IMU noise
     * - Depends on GPS quality
     * 
     * Alignment on Non-Level Surface:
     * - Tilt reflects actual vehicle attitude
     * - Not necessarily level
     * - Alignment converges to actual tilt
     * - GPS velocity must be zero (stationary)
     * 
     * Impact on Filter:
     * - Until aligned: Limited filter operation
     * - After aligned: Proceed to yaw alignment
     * - Full operation: After both tilt and yaw aligned
     * - Pre-arm checks require alignment
     * 
     * Failure to Align:
     * - Excessive vibration
     * - Vehicle not stationary
     * - Accelerometer errors
     * - Poor GPS quality
     * - Prevents filter initialization
     * 
     * Alignment Verification:
     * - Log tiltErrFilt value
     * - Check tiltAlignComplete flag
     * - Monitor time to convergence
     * - Compare to expected performance
     * 
     * @note Called during initial alignment phase
     * @note Vehicle must be stationary
     * @note Prerequisite for yaw alignment
     * @note Typically completes in 5-10 seconds
     * 
     * @warning Alignment requires stationary vehicle
     * @warning Movement during alignment causes errors
     * @warning Excessive vibration prevents alignment
     * @warning Must complete before flight
     * 
     * @see tiltAlignComplete for alignment status
     * @see tiltErrFilt for filtered tilt error
     * @see tiltErrVec for tilt error vector
     * @see InitialiseFilterBootstrap() for initial alignment
     * @see yawAlignComplete for yaw alignment status
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:686-688
     */
    void checkAttitudeAlignmentStatus();

    /**
     * @brief Control reset of yaw and magnetic field states
     * 
     * @details Manages yaw angle and magnetic field state resets during filter operation.
     *          Handles post-takeoff mag initialization, magnetic anomaly detection,
     *          yaw reset requests, and compass failure recovery.
     * 
     * Yaw/Mag Reset Scenarios:
     * 
     * 1. Post-Takeoff Initialization:
     *    - finalInflightYawInit: Final yaw initialization in flight
     *    - finalInflightMagInit: Final mag field initialization in flight
     *    - Altitude threshold: EKF2_MAG_FINAL_RESET_ALT (2.5 m)
     *    - Ensures good yaw/mag before high-altitude flight
     * 
     * 2. Magnetic Anomaly Detection:
     *    - magYawResetTimer_ms: Time since mag consistent
     *    - Yaw innovations diverging from mag
     *    - Indicates mag field disturbance
     *    - May trigger yaw reset to GPS course
     * 
     * 3. Manual Yaw Reset Request:
     *    - magYawResetRequest: Yaw reset requested
     *    - gpsYawResetRequest: GPS course reset requested
     *    - External request from vehicle code
     *    - EKFGSF_requestYawReset(): GSF yaw estimator
     * 
     * 4. Compass Failure Recovery:
     *    - allMagSensorsFailed: All compasses failed
     *    - Timeout or innovation failures
     *    - Fallback to GPS course or GSF yaw
     *    - Disable mag fusion
     * 
     * Post-Takeoff Initialization:
     * 
     * Purpose:
     * - Ground mag field may have interference
     * - In-flight mag field cleaner
     * - Reset after reaching safe altitude
     * - One-time operation per flight
     * 
     * Process:
     * 1. Check altitude > EKF2_MAG_FINAL_RESET_ALT
     * 2. Check not already done (finalInflightMagInit)
     * 3. Calculate new mag field states from measurements
     * 4. Reset yaw to match new mag field
     * 5. Set finalInflightMagInit = true
     * 6. Set finalInflightYawInit = true
     * 7. Log reset event
     * 
     * Magnetic Anomaly Detection:
     * 
     * Detection Criteria:
     * - Sustained mag innovation failures
     * - yawInnovation large and persistent
     * - consistentMagData = false
     * - magYawResetTimer_ms exceeded threshold
     * 
     * Response:
     * - Count anomalies: magYawAnomallyCount
     * - Limit resets: MAG_ANOMALY_RESET_MAX (2 per flight)
     * - Reset yaw to GPS course (if available)
     * - Or reset yaw to GSF estimate
     * - Log magnetic anomaly event
     * 
     * GPS Course Yaw Reset:
     * 
     * When Used:
     * - Compass failed or anomaly detected
     * - GPS velocity sufficient for course determination
     * - Typically requires > 3 m/s ground speed
     * - realignYawGPS() performs reset
     * 
     * Process:
     * - Calculate yaw from GPS velocity vector
     * - Reset quaternion yaw: resetQuatStateYawOnly()
     * - Reset yaw covariance
     * - Update magnetic field states to match new yaw
     * - Log yaw reset event
     * 
     * GSF Yaw Estimator Reset:
     * 
     * Purpose:
     * - Gaussian Sum Filter yaw estimator
     * - Independent yaw estimate from IMU + GPS
     * - Backup when compass unreliable
     * - EKFGSF_resetMainFilterYaw() performs reset
     * 
     * When Used:
     * - EKFGSF_requestYawReset() called
     * - Compass failed
     * - Yaw uncertainty high
     * - GSF estimate confident
     * 
     * Mag Field State Reset:
     * 
     * When Triggered:
     * - magStateResetRequest flag set
     * - After yaw reset (mag must match yaw)
     * - Magnetic anomaly detected
     * - Compass calibration changed
     * 
     * Process:
     * - Calculate new earth mag field (states 16-18)
     * - Calculate new body mag field offsets (states 19-21)
     * - From current magnetometer measurements
     * - Reset associated covariances
     * - alignMagStateDeclination() aligns to declination
     * 
     * Yaw Reset Methods:
     * - resetQuatStateYawOnly(): Reset quaternion yaw only
     * - Preserves roll and pitch
     * - Updates yaw covariance
     * - isDeltaYaw: Add or set absolute
     * 
     * Reset Rate Limiting:
     * - MAG_ANOMALY_RESET_MAX: Max yaw resets per flight (2)
     * - Prevent excessive resetting
     * - After limit: Disable mag fusion, use GPS course
     * - Indicates persistent mag problem
     * 
     * Altitude Gating:
     * - Post-takeoff reset: Altitude > 2.5 m
     * - Ensures away from ground interference
     * - posDownAtLastMagReset: Altitude at last reset
     * - Prevents premature reset on ground
     * 
     * State Tracking:
     * - posDownAtLastMagReset: Altitude at last reset
     * - yawInnovAtLastMagReset: Yaw innovation at reset
     * - quatAtLastMagReset: Quaternion at last reset
     * - Used for anomaly detection
     * 
     * Impact on Navigation:
     * - Yaw reset: Momentary heading discontinuity
     * - Mag reset: Affects future heading accuracy
     * - Vehicle control may see transient
     * - Logged for post-flight analysis
     * 
     * @note Called each filter update cycle
     * @note Manages multiple reset scenarios
     * @note Limited resets per flight
     * @note Critical for heading accuracy
     * 
     * @warning Yaw resets cause heading discontinuity
     * @warning Multiple resets indicate mag problem
     * @warning May need compass calibration
     * @warning Affects vehicle control momentarily
     * 
     * @see magYawResetRequest for reset request flag
     * @see gpsYawResetRequest for GPS course reset
     * @see resetQuatStateYawOnly() for yaw reset method
     * @see realignYawGPS() for GPS course alignment
     * @see EKFGSF_resetMainFilterYaw() for GSF yaw reset
     * @see magYawAnomallyCount for anomaly tracking
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:690
     */
    void controlMagYawReset();

    /**
     * @brief Set the latitude and longitude and height used to set the NED origin
     * 
     * @details Internal method to set the EKF's NED coordinate frame origin. All NED
     *          positions calculated by the filter are relative to this location. This
     *          method performs the actual origin setting with validation checks.
     * 
     * @param[in] loc Location for NED origin (latitude, longitude, altitude)
     * 
     * @return true if origin set successfully, false if origin already set or invalid
     * 
     * NED Origin Concept:
     * - NED: North-East-Down coordinate frame
     * - Origin: Reference point for all NED positions
     * - EKF position states (6-8): NED relative to origin
     * - Origin typically set at first GPS fix or startup location
     * 
     * Origin Initialization:
     * - Called during filter initialization
     * - First valid GPS fix location
     * - Or external navigation system origin
     * - Or manually set via setOriginLLH()
     * 
     * Validation Checks:
     * - Origin not already set (validOrigin == false)
     * - Location parameters valid:
     *   * Latitude: -90° to +90°
     *   * Longitude: -180° to +180°
     *   * Altitude: Reasonable value (e.g., -1000 to 10000 m)
     * - If checks fail: Return false
     * 
     * Origin Setting Process:
     * 1. Validate input location
     * 2. Check origin not already set
     * 3. Store location: EKF_origin
     * 4. Set validOrigin = true
     * 5. Initialize ekfGpsRefHgt (WGS-84 reference height)
     * 6. Log origin setting event
     * 7. Return true
     * 
     * EKF_origin Storage:
     * - Type: Location structure
     * - Contains: Latitude, longitude, altitude
     * - Used for LLH ↔ NED conversions
     * - Persistent for filter lifetime (or until reset)
     * 
     * WGS-84 Reference Height:
     * - ekfGpsRefHgt: Double precision altitude (m)
     * - WGS-84 ellipsoid reference
     * - Used to convert GPS altitude to local height
     * - local_height = gps_alt - ekfGpsRefHgt
     * 
     * Origin Lock:
     * - Once set, origin cannot be changed (normally)
     * - validOrigin flag prevents re-setting
     * - Ensures position consistency
     * - Reset only via filter re-initialization
     * 
     * Position State Interpretation:
     * - stateStruct.position[0]: North from origin (m)
     * - stateStruct.position[1]: East from origin (m)
     * - stateStruct.position[2]: Down from origin (m)
     * - All relative to EKF_origin
     * 
     * LLH Conversions:
     * - getPosNE(), getPosD(): Return NED position
     * - getLLH(): Convert NED to lat/lon/alt
     * - Uses EKF_origin as reference
     * - Earth curvature accounted for
     * 
     * Origin Altitude:
     * - Typically GPS altitude at initialization
     * - Or barometer altitude + GPS altitude offset
     * - ekfOriginHgtVar: Origin height variance
     * - correctEkfOriginHeight(): Corrects over time
     * 
     * Use Cases:
     * - Initial GPS fix sets origin
     * - Indoor flight: Manual origin setting
     * - Multi-vehicle: Common origin for coordination
     * - Simulation: Known origin location
     * 
     * External vs Internal Setting:
     * - External: setOriginLLH() (public method)
     * - Internal: setOrigin() (this private method)
     * - setOriginLLH() calls setOrigin() with checks
     * - This method does actual work
     * 
     * Origin Reset Conditions:
     * - Normally origin never reset during flight
     * - Filter re-initialization may reset origin
     * - Large position discontinuities handled differently
     * - Origin stability critical for navigation
     * 
     * Impact on Filter:
     * - All position estimates relative to origin
     * - GPS position converted to NED via origin
     * - Position resets use origin as reference
     * - Mission waypoints use origin for conversion
     * 
     * Multi-Vehicle Coordination:
     * - Multiple vehicles can share origin
     * - Enables relative positioning
     * - Formation flight applications
     * - Swarm coordination
     * 
     * @note Origin can only be set once (unless filter reset)
     * @note All NED positions relative to this origin
     * @note Critical for position estimation accuracy
     * @note Earth curvature effects for large distances
     * 
     * @warning Origin cannot be changed after setting
     * @warning Invalid origin causes navigation errors
     * @warning Large distances from origin reduce accuracy
     * @warning Ensure origin location is valid
     * 
     * @see EKF_origin for stored origin location
     * @see validOrigin for origin set status
     * @see setOriginLLH() for public origin setting
     * @see getOriginLLH() for origin retrieval
     * @see ekfGpsRefHgt for WGS-84 reference height
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:692-694
     */
    bool setOrigin(const Location &loc);

    /**
     * @brief Assess GPS data quality and set gpsGoodToAlign flag
     * 
     * @details Evaluates GPS quality metrics to determine if GPS data is of sufficient
     *          quality to align the EKF during initialization. Checks satellite count,
     *          accuracy estimates, and position stability. Sets gpsGoodToAlign flag.
     * 
     * GPS Quality Checks:
     * 
     * 1. Satellite Count (MASK_GPS_NSATS):
     *    - Minimum satellites: Typically ≥ 6
     *    - More satellites → better geometry
     *    - Fewer satellites → poor accuracy
     *    - Check: numSats >= threshold
     * 
     * 2. Horizontal Dilution of Precision (MASK_GPS_HDOP):
     *    - HDOP: Geometric quality factor
     *    - Lower HDOP → better geometry
     *    - Threshold: Typically < 2.0
     *    - Check: hdop < threshold
     * 
     * 3. Speed Accuracy (MASK_GPS_SPD_ERR):
     *    - GPS-reported speed accuracy
     *    - gpsSpdAccuracy: Estimated velocity error (m/s)
     *    - Threshold: Typically < 1.0 m/s
     *    - Check: sAcc < threshold
     * 
     * 4. Position Accuracy (MASK_GPS_POS_ERR):
     *    - GPS-reported position accuracy
     *    - gpsPosAccuracy: Estimated horizontal error (m)
     *    - Threshold: Typically < 10 m
     *    - Check: hAcc < threshold
     * 
     * 5. Altitude Accuracy (if used):
     *    - gpsHgtAccuracy: Vertical position error (m)
     *    - Threshold: Typically < 20 m
     *    - Check: vAcc < threshold
     * 
     * 6. Position Drift (MASK_GPS_POS_DRIFT):
     *    - gpsDriftNE: Measured position drift while stationary
     *    - Vehicle should be stationary for alignment
     *    - Threshold: Typically < 3 m
     *    - Indicates GPS not settled or vehicle moving
     * 
     * 7. Vertical Speed (MASK_GPS_VERT_SPD):
     *    - gpsVertVelFilt: Filtered vertical velocity
     *    - Should be near zero when stationary
     *    - Threshold: Typically < 0.3 m/s
     *    - Check: |vel_down| < threshold
     * 
     * 8. Horizontal Speed (MASK_GPS_HORIZ_SPD):
     *    - gpsHorizVelFilt: Filtered horizontal velocity
     *    - Should be near zero when stationary
     *    - Threshold: Typically < 0.3 m/s
     *    - Check: sqrt(vel_n² + vel_e²) < threshold
     * 
     * gpsCheckStatus Flags:
     * - Bitfield indicating which checks failed
     * - bad_sats, bad_hdop, bad_sAcc, bad_hAcc, bad_vAcc
     * - bad_horiz_drift, bad_vert_vel, bad_horiz_vel
     * - Used for pre-arm failure reporting
     * 
     * Alignment Readiness:
     * - gpsGoodToAlign = true: GPS quality sufficient
     * - All checks must pass (logical AND)
     * - Single failure → gpsGoodToAlign = false
     * - Critical for successful filter initialization
     * 
     * Pre-Alignment Monitoring:
     * - Called repeatedly before alignment
     * - lastPreAlignGpsCheckTime_ms: Last check time
     * - Check interval: Typically 1 Hz
     * - Waits for GPS to settle and meet criteria
     * 
     * Position Drift Calculation:
     * - gpsloc_prev: Previous GPS position
     * - Current position: Latest GPS reading
     * - Drift: Distance between consecutive readings
     * - Accumulated over check period
     * - Should be minimal when stationary
     * 
     * Velocity Filtering:
     * - gpsVertVelFilt: Low-pass filtered vertical velocity
     * - gpsHorizVelFilt: Low-pass filtered horizontal velocity
     * - Removes transient GPS noise
     * - Requires sustained low velocity
     * 
     * Configuration Parameters:
     * - EK2_CHECK_SCALE: Scales check thresholds
     * - Allows relaxing checks if needed
     * - Scale > 1.0: More lenient
     * - Scale < 1.0: More strict
     * 
     * Use Cases:
     * - Pre-arm checks: Prevent arming with poor GPS
     * - Filter initialization: Wait for good GPS
     * - Origin setting: Ensure accurate origin
     * - Post-GPS-outage recovery
     * 
     * Failure Reasons:
     * - Insufficient satellites (obstructed sky)
     * - Poor satellite geometry (high HDOP)
     * - GPS not locked/settled
     * - Vehicle moving (not stationary)
     * - GPS sensor malfunction
     * - Multipath interference
     * 
     * Impact on Initialization:
     * - InitialiseFilterBootstrap() requires gpsGoodToAlign
     * - Without good GPS: Can't initialize position
     * - May initialize with alternative sensors
     * - Or wait for GPS quality improvement
     * 
     * Pre-Arm Reporting:
     * - prearm_failure_reason() uses gpsCheckStatus
     * - Reports which specific checks failed
     * - Helps pilot identify GPS issues
     * - Guides troubleshooting
     * 
     * @note Called during pre-flight initialization
     * @note All checks must pass for alignment
     * @note Vehicle should be stationary
     * @note May take 10-30 seconds for GPS to settle
     * 
     * @warning Poor GPS quality prevents alignment
     * @warning Moving vehicle fails drift check
     * @warning Obstructed sky reduces satellite count
     * @warning Multipath causes position instability
     * 
     * @see gpsGoodToAlign for alignment readiness flag
     * @see gpsCheckStatus for detailed failure reasons
     * @see InitialiseFilterBootstrap() for filter initialization
     * @see prearm_failure_reason() for failure reporting
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:697
     */
    void calcGpsGoodToAlign(void);

    /**
     * @brief Return true and set flag if delta angle bias (gyro bias) has been learned
     * 
     * @details Checks if the gyro bias estimates have converged sufficiently to be
     *          considered learned. Returns true when gyro bias covariance is below
     *          threshold, indicating bias is well-determined. Sets delAngBiasLearned.
     * 
     * Gyro Bias Learning:
     * 
     * Purpose:
     * - Gyroscopes have slowly varying bias
     * - Bias must be estimated for accurate attitude
     * - Learning: Convergence of bias estimate
     * - Critical for navigation accuracy
     * 
     * Bias States (9-11):
     * - stateStruct.gyro_bias[0]: X-axis gyro bias (rad/s)
     * - stateStruct.gyro_bias[1]: Y-axis gyro bias (rad/s)
     * - stateStruct.gyro_bias[2]: Z-axis gyro bias (rad/s)
     * - Estimated by EKF along with attitude
     * 
     * Convergence Criteria:
     * - Gyro bias covariances (P[9:11, 9:11]) small
     * - Threshold: Typically < (0.1 deg/s)²
     * - All three axes must converge
     * - Sustained convergence (not transient)
     * 
     * Covariance Check:
     * - P[9][9]: X-axis bias variance
     * - P[10][10]: Y-axis bias variance
     * - P[11][11]: Z-axis bias variance
     * - Compare to InitialGyroBiasUncertainty()
     * - If all below threshold → learned
     * 
     * Learning Process:
     * - Filter updates bias during operation
     * - GPS velocity provides observability
     * - Accelerometer provides observability
     * - Magnetometer (yaw) provides observability
     * - Converges over 30-60 seconds typically
     * 
     * delAngBiasLearned Flag:
     * - Set to true when bias learned
     * - Used in pre-arm checks
     * - Used to enable certain features
     * - Indicates filter ready for flight
     * 
     * Initial Bias Uncertainty:
     * - InitialGyroBiasUncertainty(): Vehicle-specific
     * - Larger for vehicles with more dynamics
     * - Smaller for static platforms
     * - Defines convergence threshold
     * 
     * Use in Pre-Arm Checks:
     * - Prevents arming with unconverged bias
     * - Ensures attitude accuracy
     * - Part of comprehensive pre-flight checks
     * - May require 30-60 seconds stationary
     * 
     * Impact on Attitude:
     * - Unconverged bias → attitude drift
     * - Learned bias → stable attitude
     * - Bias errors cause navigation errors
     * - Particularly affects yaw accuracy
     * 
     * Gyro Scale Factor:
     * - States 12-14: gyro_scale
     * - Scale factor also estimated
     * - Less critical than bias
     * - Typically converges slower
     * 
     * Learning Requirements:
     * - Vehicle stationary initially
     * - Sufficient sensor data (GPS, mag, etc.)
     * - Adequate time (30-60 seconds)
     * - Low vibration environment
     * 
     * Bias Stability:
     * - Gyro bias drifts slowly with temperature
     * - Filter tracks slow changes
     * - Rapid changes may indicate sensor issue
     * - Bias logged for analysis
     * 
     * Multiple IMUs:
     * - Each IMU has separate bias estimate
     * - inactiveBias[]: Biases for non-primary IMUs
     * - learnInactiveBiases() updates
     * - Enables seamless IMU switching
     * 
     * Return Value:
     * - true: Bias learned, sets delAngBiasLearned
     * - false: Bias not yet converged
     * - Function has side effect (sets flag)
     * 
     * @note Requires 30-60 seconds to learn typically
     * @note Vehicle should be stationary initially
     * @note Critical for attitude accuracy
     * @note Part of pre-arm checks
     * 
     * @warning Insufficient learning time causes drift
     * @warning Movement prevents bias learning
     * @warning High vibration degrades learning
     * @warning Poor sensor data prevents convergence
     * 
     * @see delAngBiasLearned for learned status flag
     * @see stateStruct.gyro_bias for bias states (9-11)
     * @see InitialGyroBiasUncertainty() for threshold
     * @see learnInactiveBiases() for inactive IMU biases
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:700
     */
    bool checkGyroCalStatus(void);

    /**
     * @brief Update in-flight calculation that determines if GPS is good for reliable navigation
     * 
     * @details Continuously monitors GPS quality during flight to determine if GPS data
     *          is reliable enough for safe navigation. Checks GPS-reported accuracy,
     *          innovation consistency, and speed accuracy. Sets gpsAccuracyGood flag.
     * 
     * In-Flight GPS Monitoring:
     * 
     * Purpose:
     * - GPS quality can degrade during flight
     * - Detect GPS issues before they cause problems
     * - Determine if GPS reliable for autonomous flight
     * - Different from pre-flight checks (more stringent)
     * 
     * Quality Checks:
     * 
     * 1. Speed Accuracy (gpsSpdAccPass):
     *    - GPS-reported speed accuracy (sAcc)
     *    - Filtered: sAccFilterState1, sAccFilterState2
     *    - Threshold: Typically < 1.5 m/s
     *    - Low-pass and peak-hold filtering
     * 
     * 2. Innovation Consistency (ekfInnovationsPass):
     *    - velTestRatio: Velocity innovation ratio
     *    - posTestRatio: Position innovation ratio
     *    - Thresholds: Typically < 5.0 (chi-squared)
     *    - Indicates GPS consistent with EKF estimate
     * 
     * 3. GPS-Reported Accuracy:
     *    - gpsPosAccuracy: Horizontal position accuracy (m)
     *    - gpsHgtAccuracy: Vertical accuracy (m)
     *    - gpsSpdAccuracy: Speed accuracy (m/s)
     *    - Must be within acceptable limits
     * 
     * Speed Accuracy Filtering:
     * - sAccFilterState1: Low-pass filter state
     *   * Time constant: ~10 seconds
     *   * Smooths transient accuracy changes
     * - sAccFilterState2: Peak-hold filter state
     *   * Captures worst-case accuracy
     *   * Decays slowly over time
     * - Both must be below threshold
     * 
     * Innovation Consistency:
     * - EKF estimate vs GPS measurement
     * - Large innovations → GPS or EKF problem
     * - Sustained large innovations fail check
     * - lastInnovPassTime_ms: Last pass time
     * - lastInnovFailTime_ms: Last fail time
     * 
     * Combined Assessment:
     * - gpsAccuracyGood = gpsSpdAccPass AND ekfInnovationsPass
     * - Both checks must pass
     * - Failure of either → GPS not reliable
     * - Used for flight mode restrictions
     * 
     * Check Timing:
     * - lastGpsCheckTime_ms: Last check time
     * - Check rate: Typically 1 Hz
     * - Continuous monitoring during flight
     * - Faster response than pre-flight checks
     * 
     * Hysteresis:
     * - Different thresholds for pass → fail vs fail → pass
     * - Prevents rapid state oscillation
     * - More lenient to recover than to fail
     * - Stabilizes GPS quality assessment
     * 
     * Impact on Flight Modes:
     * - gpsAccuracyGood = true: All modes available
     *   * Auto missions, guided, loiter, RTL
     * - gpsAccuracyGood = false: Restricted modes
     *   * May inhibit auto missions
     *   * May trigger failsafe
     *   * Pilot notification
     * 
     * GPS Quality Degradation Causes:
     * - Reduced satellite visibility
     * - Poor satellite geometry
     * - Ionospheric disturbances
     * - Multipath interference
     * - GPS jamming or interference
     * - Satellite clock errors
     * 
     * EKF Innovation Failures:
     * - May indicate GPS error
     * - Or may indicate EKF divergence
     * - Or sensor calibration problem
     * - Context used to determine cause
     * 
     * Failsafe Integration:
     * - gpsAccuracyGood used in failsafe logic
     * - May trigger GPS failsafe
     * - May trigger RTL or land
     * - Depends on vehicle configuration
     * 
     * Configuration Parameters:
     * - EK2_CHECK_SCALE: Scales check thresholds
     * - GPS_*: GPS configuration affecting quality
     * - FS_EKF_THRESH: EKF failsafe threshold
     * 
     * Logging:
     * - GPS quality metrics logged
     * - Innovation test ratios logged
     * - State transitions logged
     * - Post-flight analysis available
     * 
     * Recovery:
     * - GPS quality may improve
     * - Filter tracks quality continuously
     * - Automatic recovery when quality restored
     * - No manual intervention needed
     * 
     * @note Monitors GPS quality continuously in flight
     * @note Different from pre-flight GPS checks
     * @note Used for flight mode restrictions
     * @note May trigger failsafe actions
     * 
     * @warning GPS quality can degrade suddenly
     * @warning Poor GPS may cause navigation errors
     * @warning Innovation failures may indicate EKF problem
     * @warning Monitor GPS quality indicators during flight
     * 
     * @see gpsAccuracyGood for GPS quality flag
     * @see gpsSpdAccPass for speed accuracy check
     * @see ekfInnovationsPass for innovation check
     * @see velTestRatio, posTestRatio for innovation ratios
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:703
     */
    void calcGpsGoodForFlight(void);

#if AP_RANGEFINDER_ENABLED
    /**
     * @brief Read the range finder and take new measurements if available
     * 
     * @details Reads rangefinder sensor data, applies median filtering to remove outliers,
     *          and stores filtered measurements in the rangefinder buffer for fusion.
     *          Handles multiple downward-facing rangefinder instances if available.
     * 
     * Rangefinder Data Reading:
     * 
     * Data Retrieval:
     * - Query AP_RangeFinder library
     * - Get latest range measurement (meters)
     * - Get measurement timestamp
     * - Get sensor health status
     * - Get sensor instance index
     * 
     * Multiple Rangefinders:
     * - DOWNWARD_RANGEFINDER_MAX_INSTANCES: Typically 1-2
     * - Each rangefinder processed independently
     * - Best quality measurement selected
     * - Redundancy for reliability
     * 
     * Median Filtering:
     * 
     * Purpose:
     * - Remove outliers and false detections
     * - Vegetation, water, moving objects cause spikes
     * - Median filter rejects single-point outliers
     * - More robust than simple averaging
     * 
     * Filter Implementation:
     * - storedRngMeas[instance][3]: 3-sample buffer per sensor
     * - storedRngMeasTime_ms[instance][3]: Timestamps
     * - rngMeasIndex[instance]: Current buffer index
     * - Circular buffer: Oldest sample replaced
     * 
     * Median Calculation:
     * - Sort 3 samples
     * - Select middle value
     * - Reject min and max (outliers)
     * - Median is robust central tendency
     * 
     * Data Validation:
     * 
     * Range Limits:
     * - Minimum range: Sensor-specific (e.g., 0.2 m)
     * - Maximum range: Sensor-specific (e.g., 40 m)
     * - Out-of-range: Reject measurement
     * - Invalid readings: Typically 0 or max range
     * 
     * Sensor Health:
     * - Check sensor status from AP_RangeFinder
     * - Timeout: No data recently
     * - Out of range: Beyond sensor capability
     * - Hardware error: Sensor malfunction
     * 
     * Quality Checks:
     * - Signal strength (if available)
     * - Measurement variance
     * - Consistency with previous readings
     * - Rate of change limits
     * 
     * Buffer Management:
     * - rangeDataNew: Latest filtered measurement
     * - time_ms: Measurement timestamp
     * - rng: Filtered range value (meters)
     * - sensor_idx: Sensor instance index
     * 
     * Fusion Scheduling:
     * - rangeDataToFuse flag: Set when new data ready
     * - Data added to storedRange buffer
     * - Buffer maintains history for delayed fusion
     * - Fusion time horizon compensation
     * 
     * Timestamp Management:
     * - lastRngMeasTime_ms: Most recent measurement time
     * - Used for timeout detection
     * - Used for data staleness check
     * - Critical for proper fusion timing
     * 
     * Sensor Instance Selection:
     * - If multiple downward sensors available
     * - Select best quality measurement
     * - Shortest valid range typically preferred
     * - Or highest signal strength
     * 
     * Terrain Detection:
     * - Detect ground surface
     * - Distinguish from false targets
     * - Vegetation penetration (for some sensors)
     * - Water surface detection
     * 
     * False Target Rejection:
     * - Birds, dust, precipitation
     * - Transient objects
     * - Median filter helps
     * - Innovation consistency check in fusion
     * 
     * Height Above Ground:
     * - Range measurement = height AGL
     * - Used for terrain-relative altitude
     * - Used for optical flow fusion
     * - Used for precision landing
     * 
     * Integration with EKF:
     * - Measurements stored in buffer
     * - Delayed fusion at fusion time horizon
     * - selectHeightForFusion() chooses source
     * - May switch between baro and rangefinder
     * 
     * Configuration:
     * - RNGFND_TYPE: Sensor type
     * - RNGFND_ORIENT: Sensor orientation (downward)
     * - RNGFND_MIN_CM, RNGFND_MAX_CM: Range limits
     * - RNGFND_GAIN: Sensor-specific gain
     * 
     * @note Only compiled if AP_RANGEFINDER_ENABLED
     * @note Median filter requires 3 samples
     * @note Handles multiple rangefinder instances
     * @note Critical for terrain-relative navigation
     * 
     * @warning Outliers can affect navigation
     * @warning Median filter adds latency (3 samples)
     * @warning Vegetation causes false readings
     * @warning Water surface may give false readings
     * 
     * @see storedRngMeas for measurement buffer
     * @see rangeDataNew for latest filtered data
     * @see rangeDataToFuse for fusion trigger
     * @see useRngFinder() for rangefinder usage check
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:706-710
     */
    void readRangeFinder();
#endif

    /**
     * @brief Check if vehicle has taken off during optical flow navigation
     * 
     * @details Detects takeoff when using optical flow for navigation by monitoring
     *          rangefinder and IMU data. Sets takeOffDetected flag to enable optical
     *          flow fusion after takeoff is confirmed. Prevents false flow fusion on ground.
     * 
     * Takeoff Detection for Optical Flow:
     * 
     * Purpose:
     * - Optical flow unreliable on ground
     * - Ground surface may move or be unstable
     * - Detect liftoff to enable flow fusion
     * - Prevent ground effect errors
     * 
     * Detection Criteria:
     * 
     * 1. Range Change:
     *    - rngAtStartOfFlight: Range at arming
     *    - Current range: Latest rangefinder reading
     *    - Altitude gain: current_rng - rngAtStartOfFlight
     *    - Threshold: Typically > 0.5-1.0 m
     * 
     * 2. Vertical Acceleration:
     *    - IMU Z-axis acceleration
     *    - Sustained upward acceleration
     *    - Indicates climb
     *    - Threshold: > 0.5 m/s² sustained
     * 
     * 3. Time Since Arming:
     *    - timeAtArming_ms: Motor arming time
     *    - Elapsed time: millis() - timeAtArming_ms
     *    - Minimum: Typically > 2 seconds
     *    - Prevents premature detection
     * 
     * 4. Rangefinder Validity:
     *    - Valid rangefinder data available
     *    - Not out of range
     *    - Not timed out
     *    - Quality acceptable
     * 
     * Takeoff Sequence:
     * 
     * 1. Motors Armed:
     *    - timeAtArming_ms recorded
     *    - rngAtStartOfFlight captured
     *    - takeOffDetected = false initially
     *    - Wait for liftoff
     * 
     * 2. Altitude Gain Detected:
     *    - Range increases from rngAtStartOfFlight
     *    - Sustained increase (not transient)
     *    - Above altitude threshold
     * 
     * 3. Takeoff Confirmed:
     *    - Set takeOffDetected = true
     *    - Enable optical flow fusion
     *    - Enable terrain offset estimation
     *    - Log takeoff event
     * 
     * Impact on Optical Flow:
     * - Before takeoff: Flow fusion disabled
     *   * Ground surface may be unstable
     *   * Propwash affects ground
     *   * False motion detected
     * - After takeoff: Flow fusion enabled
     *   * Stable flight conditions
     *   * Reliable velocity estimation
     *   * Position hold capability
     * 
     * Ground Effect Handling:
     * - meaHgtAtTakeOff: Measured height at takeoff
     * - Ground effect causes baro errors
     * - Detect and compensate
     * - Important for multirotors
     * 
     * Terrain Offset Initialization:
     * - EstimateTerrainOffset() starts after takeoff
     * - terrainState initialized
     * - Converges during initial climb
     * - Required for flow fusion
     * 
     * False Takeoff Prevention:
     * - Multiple criteria must be met
     * - Sustained conditions required
     * - Time delay after arming
     * - Prevents false detections from:
     *   * Vehicle vibration
     *   * Prop wash on ground
     *   * Sensor noise
     * 
     * Use Cases:
     * - Indoor flight with optical flow
     * - GPS-denied navigation
     * - Precision hover and landing
     * - Requires rangefinder + flow sensor
     * 
     * Landing Detection:
     * - Reverse logic for landing
     * - Range decreases to near rngOnGnd
     * - Low vertical velocity
     * - Disable flow fusion on landing
     * 
     * Configuration Impact:
     * - EK2_FLOW_DELAY: Flow sensor latency
     * - EK2_FLOW_M_NSE: Flow measurement noise
     * - Takeoff detection thresholds not user-configurable
     * 
     * Multirotor vs Fixed-Wing:
     * - Multirotors: Vertical takeoff, clear transition
     * - Fixed-wing: May not have vertical takeoff
     *   * Takeoff detection less clear
     *   * May use airspeed instead
     * 
     * @note Specific to optical flow navigation
     * @note Prevents ground effect errors
     * @note Requires rangefinder for altitude
     * @note Typically detects takeoff in 2-5 seconds
     * 
     * @warning Premature flow fusion causes errors
     * @warning Ground surface motion affects flow
     * @warning Prop wash disturbs ground surface
     * @warning Requires stable rangefinder readings
     * 
     * @see takeOffDetected for takeoff status flag
     * @see rngAtStartOfFlight for initial range
     * @see EstimateTerrainOffset() for terrain initialization
     * @see FuseOptFlow() for optical flow fusion
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:713
     */
    void detectOptFlowTakeoff(void);

    /**
     * @brief Align the NE earth magnetic field states with the published declination
     * 
     * @details Rotates the earth magnetic field states (16-18) to align with the
     *          published magnetic declination. Ensures earth field declination matches
     *          world magnetic model, preventing gradual declination drift.
     * 
     * Magnetic Declination:
     * - Angle between true north and magnetic north
     * - Varies by location (World Magnetic Model)
     * - MagDeclination(): Returns declination (radians)
     * - Typically obtained from tables or online calculation
     * 
     * Earth Magnetic Field States (16-18):
     * - stateStruct.earth_magfield[0]: North component (mGauss)
     * - stateStruct.earth_magfield[1]: East component (mGauss)
     * - stateStruct.earth_magfield[2]: Down component (mGauss)
     * - NED frame representation
     * 
     * Declination Alignment Purpose:
     * 
     * Problem:
     * - EKF learns earth field from mag measurements
     * - Learned field may drift from true declination
     * - Causes yaw errors over time
     * - Especially without GPS or other yaw reference
     * 
     * Solution:
     * - Periodically align earth field to declination
     * - Rotate NE components to match published declination
     * - Keep total field magnitude constant
     * - Prevents declination drift
     * 
     * Alignment Process:
     * 
     * 1. Get Current Earth Field:
     *    - magN = stateStruct.earth_magfield[0]
     *    - magE = stateStruct.earth_magfield[1]
     *    - magD = stateStruct.earth_magfield[2] (unchanged)
     * 
     * 2. Calculate Current Declination:
     *    - current_decl = atan2(magE, magN)
     *    - Angle from true north to mag field
     * 
     * 3. Get Published Declination:
     *    - published_decl = MagDeclination()
     *    - From World Magnetic Model (WMM)
     *    - Or manual configuration
     * 
     * 4. Calculate Rotation Angle:
     *    - delta_decl = published_decl - current_decl
     *    - Rotation needed to align
     * 
     * 5. Rotate NE Components:
     *    - magN_new = magN × cos(delta) - magE × sin(delta)
     *    - magE_new = magN × sin(delta) + magE × cos(delta)
     *    - magD unchanged (rotation in horizontal plane)
     * 
     * 6. Update States:
     *    - stateStruct.earth_magfield[0] = magN_new
     *    - stateStruct.earth_magfield[1] = magE_new
     *    - Total magnitude preserved
     * 
     * When Alignment Performed:
     * - During magnetic field initialization
     * - After magnetic field reset
     * - Periodically if large declination error
     * - When switching to different location
     * 
     * Covariance Handling:
     * - Earth field covariances (P[16:18, 16:18]) updated
     * - Rotation matrix applied to covariance
     * - Maintains covariance consistency
     * - earthMagFieldVar updated
     * 
     * Declination Sources:
     * - World Magnetic Model (WMM) tables
     * - table_declination: From WMM lookup
     * - Manual setting: COMPASS_DEC parameter
     * - Or auto-calibration from GPS
     * 
     * Impact on Yaw:
     * - Declination change affects yaw estimate
     * - Small alignment: Negligible yaw change
     * - Large alignment: Noticeable yaw correction
     * - Logged for post-flight analysis
     * 
     * Magnetic Field Magnitude:
     * - Total magnitude preserved
     * - Only direction (declination) changed
     * - field_magnitude = sqrt(magN² + magE² + magD²)
     * - Magnitude learned from measurements
     * 
     * Use Cases:
     * - Post-initialization alignment
     * - After flying to new location
     * - After magnetic field reset
     * - Preventing long-term declination drift
     * 
     * Alternative: Declination Fusion:
     * - FuseDeclination(): Soft constraint
     * - This method: Hard constraint (reset)
     * - Fusion: Gradual correction
     * - Alignment: Immediate correction
     * 
     * @note Aligns earth field to published declination
     * @note Preserves total field magnitude
     * @note Affects yaw estimate
     * @note Called after mag field initialization
     * 
     * @warning Large declination errors cause yaw jump
     * @warning Incorrect declination causes heading error
     * @warning Must have valid declination source
     * @warning Affects vehicle heading
     * 
     * @see stateStruct.earth_magfield for earth mag states (16-18)
     * @see MagDeclination() for published declination
     * @see FuseDeclination() for soft declination constraint
     * @see table_declination for WMM declination
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:716
     */
    void alignMagStateDeclination();

    /**
     * @brief Fuse compass measurements using simple declination observation
     * 
     * @details Alternative yaw fusion method that uses compass measurements with
     *          declination constraint but doesn't require full magnetic field states.
     *          Simpler than full magnetometer fusion, useful during initialization
     *          or when magnetic field states are not fully learned.
     * 
     * @note Also known as "Euler yaw fusion" (fusion of yaw angle directly)
     * 
     * Simple Declination Observation:
     * 
     * Concept:
     * - Don't estimate full earth/body mag fields
     * - Only estimate yaw angle
     * - Use compass + declination for yaw
     * - Simpler than full mag fusion
     * 
     * Measurement Model:
     * - Rotate compass reading to earth frame
     * - Expected declination from WMM
     * - Compare compass-derived yaw to expected
     * - Innovation = measured_yaw - estimated_yaw
     * 
     * Yaw Calculation from Compass:
     * 1. Get compass reading in body frame
     * 2. Rotate to earth frame using Tbn
     * 3. Calculate yaw: atan2(magE, magN)
     * 4. Compare to expected declination
     * 5. Innovation = difference
     * 
     * Advantages Over Full Mag Fusion:
     * - Simpler computation
     * - Doesn't require mag field states (16-21)
     * - Good for initialization
     * - Faster convergence for yaw
     * - Less sensitive to mag field errors
     * 
     * Disadvantages:
     * - Doesn't learn earth/body mag fields
     * - No compensation for body mag offsets
     * - Less accurate long-term
     * - Requires valid declination
     * 
     * When Used:
     * - During initial alignment
     * - When mag states not initialized
     * - Backup when full mag fusion unavailable
     * - Simple vehicles without full mag calibration
     * 
     * Fusion Process:
     * 1. Get latest compass measurement
     * 2. Compensate for body offset (if known)
     * 3. Rotate to earth frame
     * 4. Calculate yaw innovation
     * 5. Compute innovation variance
     * 6. Calculate Kalman gain
     * 7. Update quaternion states (yaw only)
     * 8. Update covariance
     * 
     * State Updates:
     * - Primary: Quaternion yaw component
     * - Secondary: Attitude error states
     * - Gyro bias: Partially observable
     * - Wind states: May be updated
     * 
     * Innovation Variance:
     * - Compass measurement noise
     * - Declination uncertainty
     * - Attitude uncertainty contribution
     * - Total: S = H × P × H' + R
     * 
     * Innovation Consistency:
     * - innovYaw: Yaw innovation (rad)
     * - Compare to threshold
     * - If excessive: Don't fuse
     * - Indicates compass or EKF problem
     * 
     * Declination Uncertainty:
     * - Published declination has uncertainty
     * - Typically ±1-2 degrees
     * - Included in measurement noise
     * - Larger uncertainty → lower gain
     * 
     * Comparison to Full Mag Fusion:
     * 
     * Full Mag Fusion (FuseMagnetometer):
     * - Estimates earth field (16-18)
     * - Estimates body field (19-21)
     * - Learns mag calibration
     * - More accurate long-term
     * 
     * Euler Yaw Fusion (this method):
     * - Only yaw angle
     * - Uses known declination
     * - Simpler, faster
     * - Good for initialization
     * 
     * Use Cases:
     * - Rapid yaw initialization
     * - Backup yaw source
     * - Simple vehicles
     * - When mag field states unreliable
     * 
     * Configuration:
     * - COMPASS_DEC: Manual declination
     * - Or auto from WMM tables
     * - Compass calibration still needed
     * - Body offsets if available
     * 
     * @note Simpler than full magnetometer fusion
     * @note Doesn't learn magnetic field states
     * @note Uses declination for yaw reference
     * @note Good for initialization and backup
     * 
     * @warning Requires valid declination
     * @warning Doesn't compensate body mag offsets
     * @warning Less accurate than full mag fusion
     * @warning Not suitable for long-term navigation
     * 
     * @see FuseMagnetometer() for full mag fusion
     * @see FuseDeclination() for declination constraint
     * @see MagDeclination() for published declination
     * @see innovYaw for yaw innovation
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:719
     */
    void fuseEulerYaw();

    /**
     * @brief Fuse declination angle to prevent earth field declination drift
     * 
     * @details Applies a soft constraint on the earth magnetic field declination to
     *          prevent gradual drift away from the published declination. Uses Kalman
     *          update with specified declination uncertainty. Maintains declination
     *          accuracy without hard resets.
     * 
     * @param[in] declErr 1-sigma uncertainty in published declination (radians)
     * 
     * Declination Constraint Purpose:
     * 
     * Problem:
     * - Earth mag field states drift slowly
     * - Declination may drift from published value
     * - Causes gradual yaw drift
     * - Especially without GPS heading reference
     * 
     * Solution:
     * - Soft Kalman constraint on declination
     * - Pulls declination toward published value
     * - Gradual correction, not sudden reset
     * - Weighted by specified uncertainty
     * 
     * Measurement Model:
     * - Measured declination: atan2(magE, magN)
     * - Expected declination: MagDeclination()
     * - Innovation: measured_decl - expected_decl
     * - Simple 1D measurement
     * 
     * Innovation Calculation:
     * - magN = stateStruct.earth_magfield[0]
     * - magE = stateStruct.earth_magfield[1]
     * - current_decl = atan2(magE, magN)
     * - published_decl = MagDeclination()
     * - innov = current_decl - published_decl
     * 
     * Innovation Variance:
     * - R = declErr² (measurement noise)
     * - H: Jacobian of declination w.r.t. states
     * - S = H × P × H' + R
     * - Includes state uncertainty contribution
     * 
     * Kalman Gain:
     * - K = P × H' / S
     * - Weighted by uncertainty
     * - Low declErr (confident) → high gain
     * - High declErr (uncertain) → low gain
     * 
     * State Updates:
     * - Earth mag field states (16-18) updated
     * - Declination pulled toward published value
     * - Magnitude mostly preserved
     * - Gradual correction over time
     * 
     * Covariance Update:
     * - P = (I - K × H) × P
     * - Reduces earth field uncertainty
     * - Maintains positive definite covariance
     * - Joseph form for stability
     * 
     * Declination Uncertainty (declErr):
     * 
     * Typical Values:
     * - High confidence: 0.02 rad (≈ 1.1°)
     * - Medium confidence: 0.05 rad (≈ 2.9°)
     * - Low confidence: 0.1 rad (≈ 5.7°)
     * - Very uncertain: 0.2 rad (≈ 11.5°)
     * 
     * Confidence Sources:
     * - WMM tables: High confidence (≈ 1°)
     * - Manual setting: Medium (≈ 3°)
     * - Outdated WMM: Lower (≈ 5°)
     * - Unknown location: Very low (≈ 10°)
     * 
     * When Fusion Applied:
     * - Periodically during flight
     * - When lacking GPS heading reference
     * - When earth field states active
     * - Not when mag states inhibited
     * 
     * Fusion Frequency:
     * - Not every update cycle
     * - Periodic: Every few seconds
     * - Sufficient to prevent drift
     * - Avoid excessive computation
     * 
     * Impact on Navigation:
     * - Prevents long-term heading drift
     * - Maintains declination accuracy
     * - Small gradual corrections
     * - No sudden heading jumps
     * 
     * Comparison to alignMagStateDeclination():
     * 
     * alignMagStateDeclination():
     * - Hard constraint (reset)
     * - Immediate correction
     * - Used during initialization
     * - Or after large errors
     * 
     * FuseDeclination() (this method):
     * - Soft constraint (Kalman update)
     * - Gradual correction
     * - Used during operation
     * - Prevents slow drift
     * 
     * Use Cases:
     * - Prevent declination drift during long flights
     * - Maintain heading accuracy without GPS
     * - Indoor flight (no GPS heading)
     * - Optical flow navigation
     * 
     * Earth Relative Observations:
     * - GPS course provides yaw reference
     * - Optical flow provides heading reference (weak)
     * - Without earth reference: Declination drift
     * - This fusion compensates
     * 
     * Configuration:
     * - COMPASS_DEC: Manual declination
     * - Or WMM tables
     * - declErr parameter: How much to trust
     * - Typically not user-configurable
     * 
     * @note Soft constraint, not hard reset
     * @note Gradual declination correction
     * @note Prevents long-term drift
     * @note Weighted by specified uncertainty
     * 
     * @warning Incorrect declination causes heading error
     * @warning Too small declErr over-constrains
     * @warning Too large declErr ineffective
     * @warning Requires valid declination source
     * 
     * @see alignMagStateDeclination() for hard alignment
     * @see MagDeclination() for published declination
     * @see stateStruct.earth_magfield for earth mag states (16-18)
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:722-724
     */
    void FuseDeclination(ftype declErr);

    /**
     * @brief Return magnetic declination in radians
     * 
     * @details Retrieves the magnetic declination (angle between true north and
     *          magnetic north) for the current location. Used for magnetic field
     *          alignment and fusion. Sources from WMM tables or manual configuration.
     * 
     * @return Magnetic declination in radians (positive = magnetic north east of true north)
     * 
     * Magnetic Declination Definition:
     * - Angle from true north to magnetic north
     * - Positive: Magnetic north east of true north
     * - Negative: Magnetic north west of true north
     * - Varies by location and time
     * - Typically -30° to +30° globally
     * 
     * Declination Sources:
     * 
     * 1. World Magnetic Model (WMM):
     *    - table_declination: From WMM lookup
     *    - Based on latitude, longitude, altitude
     *    - Updated periodically (new WMM every 5 years)
     *    - Most accurate source
     * 
     * 2. Manual Configuration:
     *    - COMPASS_DEC parameter
     *    - User-specified declination
     *    - Used if WMM not available
     *    - Or to override WMM
     * 
     * 3. Auto-Calibration (optional):
     *    - Learn from GPS course vs compass
     *    - Requires GPS and motion
     *    - Updates COMPASS_DEC automatically
     *    - COMPASS_AUTODEC parameter enables
     * 
     * WMM Table Lookup:
     * - have_table_earth_field: WMM data available
     * - Lookup based on current location
     * - Interpolation for smooth values
     * - Altitude correction included
     * 
     * Return Value:
     * - Radians: Standard EKF units
     * - Range: Typically -π to +π
     * - Example: 0.1 rad ≈ 5.7°
     * - Sign convention: East positive
     * 
     * Use in EKF:
     * 
     * 1. Magnetic Field Alignment:
     *    - alignMagStateDeclination() uses this
     *    - Aligns earth field to declination
     *    - Prevents declination drift
     * 
     * 2. Declination Fusion:
     *    - FuseDeclination() uses this
     *    - Soft constraint on declination
     *    - Gradual correction
     * 
     * 3. Yaw Initialization:
     *    - fuseEulerYaw() uses this
     *    - Initial yaw from compass + declination
     *    - calcQuatAndFieldStates() uses this
     * 
     * 4. Compass Calibration:
     *    - Declination needed for accurate heading
     *    - Separates true north from magnetic north
     *    - Critical for navigation accuracy
     * 
     * Declination Changes:
     * - Secular variation: ~0.1-0.5° per year
     * - Location change: Immediate change
     * - WMM update: Every 5 years (2020, 2025, etc.)
     * - Filter adapts to new declination
     * 
     * Accuracy:
     * - WMM: Typically ±1° accuracy
     * - Manual setting: User-dependent
     * - Auto-calibration: Requires good GPS/compass
     * - Errors cause heading errors
     * 
     * Special Cases:
     * - Magnetic poles: Declination undefined
     * - Equator: Declination typically small
     * - High latitudes: Large declinations possible
     * - Indoor: May use default or last known
     * 
     * Configuration:
     * - COMPASS_DEC: Manual declination (degrees)
     * - COMPASS_AUTODEC: Auto-learn enable
     * - WMM tables: Built into firmware
     * - Location from GPS or manual setting
     * 
     * Impact of Incorrect Declination:
     * - Heading error = declination error
     * - Navigation: Consistent offset
     * - 10° declination error = 10° heading error
     * - Critical for waypoint navigation
     * 
     * @note Declination varies by location
     * @note Positive = magnetic north east of true north
     * @note Typically from WMM tables
     * @note Critical for heading accuracy
     * 
     * @warning Incorrect declination causes heading error
     * @warning Must update for new locations
     * @warning WMM valid for ±5 years from epoch
     * @warning Manual setting less accurate than WMM
     * 
     * @see table_declination for WMM-derived value
     * @see alignMagStateDeclination() for alignment
     * @see FuseDeclination() for declination fusion
     * @see calcQuatAndFieldStates() for initialization use
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:726
     */
    ftype MagDeclination(void) const;
    
    /**
     * @brief Propagate PVA solution forward from fusion time horizon to current time
     * 
     * @details Uses a simple complementary observer to propagate Position, Velocity, and
     *          Attitude (PVA) estimates from the delayed fusion time horizon to the current
     *          time, compensating for sensor delays. Provides low-latency output for control.
     * 
     * Output Prediction Problem:
     * 
     * Time Horizons:
     * - Fusion time horizon: When measurements fused (delayed)
     * - Current time horizon: Now (IMU data latest)
     * - Delay: Sensor latency + processing time
     * - Typically 100-200 ms delay
     * 
     * Solution:
     * - EKF state at fusion time (delayed)
     * - Propagate forward using IMU
     * - Complementary observer for corrections
     * - Output: Current time estimate
     * 
     * Output State Buffer:
     * - storedOutput: Ring buffer of output states
     * - outputDataNew: Latest output at current time
     * - outputDataDelayed: Output at fusion time
     * - Buffer spans delay period
     * 
     * Output State Elements:
     * - quat: Attitude quaternion
     * - velocity: NED velocity (m/s)
     * - position: NED position (m)
     * - Total: 10 states (4 + 3 + 3)
     * 
     * Propagation Method:
     * 
     * 1. Start from Fusion Time:
     *    - outputDataDelayed: EKF state at fusion time
     *    - This is the "truth" reference
     *    - Based on fused measurements
     * 
     * 2. IMU Integration:
     *    - Integrate IMU from fusion to current time
     *    - Dead reckoning propagation
     *    - Accumulates errors rapidly
     * 
     * 3. Complementary Observer:
     *    - Tracking error: output vs EKF state
     *    - Error feedback to correct drift
     *    - PID-like correction
     *    - Smooth convergence
     * 
     * Tracking Error Calculation:
     * 
     * Attitude Error:
     * - EKF quaternion vs output quaternion
     * - Convert to rotation vector
     * - delAngCorrection: Correction to apply
     * - Fed back to attitude integration
     * 
     * Velocity Error:
     * - velErrintegral: Integrated velocity error
     * - EKF velocity - output velocity
     * - Proportional + integral feedback
     * - Corrects velocity drift
     * 
     * Position Error:
     * - posErrintegral: Integrated position error
     * - EKF position - output position
     * - Integral feedback only (for smooth tracking)
     * - Prevents position jumps
     * 
     * Observer Gains:
     * - Attitude: Fast correction (high bandwidth)
     * - Velocity: Medium correction
     * - Position: Slow correction (low bandwidth)
     * - Trade-off: Responsiveness vs smoothness
     * 
     * IMU Correction:
     * - delAngCorrected: IMU delta angle + correction
     * - delVelCorrected: IMU delta velocity + correction
     * - Corrected IMU drives output propagation
     * - Closes feedback loop
     * 
     * Propagation Steps:
     * 
     * 1. Calculate tracking errors:
     *    - Attitude error
     *    - Velocity error
     *    - Position error
     * 
     * 2. Update error integrals:
     *    - velErrintegral += vel_error × dt
     *    - posErrintegral += pos_error × dt
     * 
     * 3. Calculate corrections:
     *    - delAngCorrection from attitude error
     *    - Velocity correction from vel error + integral
     *    - Position correction from pos integral
     * 
     * 4. Correct IMU data:
     *    - delAngCorrected = delAng + delAngCorrection
     *    - delVelCorrected = delVel + vel_correction
     * 
     * 5. Propagate output states:
     *    - Attitude: Quaternion integration with corrected delAng
     *    - Velocity: vel += delVelCorrected
     *    - Position: pos += vel × dt + pos_correction
     * 
     * 6. Store in output buffer:
     *    - outputDataNew updated
     *    - Buffer index advanced
     * 
     * Body Frame Offsets:
     * - velOffsetNED: IMU to body origin velocity offset
     * - posOffsetNED: IMU to body origin position offset
     * - Accounts for IMU not at vehicle CG
     * - Applied to output states
     * 
     * Vertical Position Filtering:
     * - vertCompFiltState: Kinematic consistency filter
     * - Ensures vertical velocity consistent with position
     * - Prevents discontinuities
     * - Used by getPosDownDerivative()
     * 
     * Output Tracking Error:
     * - outputTrackError: Magnitude of tracking errors
     * - [0]: Attitude error (rad)
     * - [1]: Velocity error (m/s)
     * - [2]: Position error (m)
     * - Logged for performance analysis
     * 
     * Use Cases:
     * - Flight control: Needs current state, not delayed
     * - Low latency critical for stability
     * - Compensates sensor delays
     * - Smooth state estimates
     * 
     * Benefits:
     * - Low latency output for controllers
     * - Smooth state transitions
     * - Handles measurement delays
     * - Reduces control lag
     * 
     * Challenges:
     * - Observer tuning (gains)
     * - Too slow: Lag persists
     * - Too fast: Oscillations
     * - Balance needed
     * 
     * @note Called every IMU update
     * @note Provides low-latency output for control
     * @note Compensates for sensor delays
     * @note Uses complementary observer feedback
     * 
     * @warning Observer gains critical for performance
     * @warning High gains cause oscillations
     * @warning Low gains cause lag
     * @warning Requires tuning for vehicle dynamics
     * 
     * @see outputDataNew for current time output
     * @see outputDataDelayed for fusion time output
     * @see storedOutput for output state buffer
     * @see velErrintegral, posErrintegral for error integrals
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:728-730
     */
    void calcOutputStates();

    /**
     * @brief Calculate a filtered offset between baro height and EKF height estimate
     * 
     * @details Computes and low-pass filters the difference between barometer height
     *          measurements and the EKF's height estimate. Used to detect barometer
     *          drift, calibration errors, or ground effect. Enables smooth transitions
     *          when switching height sources.
     * 
     * Baro Offset Purpose:
     * 
     * Why Offset Exists:
     * - Barometer drift over time
     * - Temperature effects on baro
     * - Ground effect during takeoff/landing
     * - Calibration errors
     * - Baro vs GPS altitude reference differences
     * 
     * Offset Calculation:
     * - baro_height: Latest barometer reading
     * - ekf_height: stateStruct.position[2] (down)
     * - offset = baro_height - ekf_height
     * - Positive: Baro reads higher than EKF
     * 
     * Filtering:
     * - Low-pass filter on offset
     * - Time constant: Several seconds
     * - Removes transient variations
     * - Tracks slowly varying bias
     * - baroHgtOffset: Filtered result
     * 
     * Filter Implementation:
     * - First-order IIR filter
     * - Exponential smoothing
     * - baroHgtOffset += (measured_offset - baroHgtOffset) × alpha
     * - alpha: Filter coefficient (time constant)
     * 
     * Uses of Filtered Offset:
     * 
     * 1. Height Source Switching:
     *    - When switching from GPS to baro
     *    - Apply offset for continuity
     *    - Prevents height jumps
     *    - Smooth transition
     * 
     * 2. Baro Drift Detection:
     *    - Large offset indicates drift
     *    - May trigger height source change
     *    - Or baro recalibration
     * 
     * 3. Ground Effect Compensation:
     *    - Prop wash affects baro near ground
     *    - Offset grows during takeoff
     *    - meaHgtAtTakeOff: Reference height
     *    - Compensate for ground effect
     * 
     * Ground Effect:
     * - Multirotor prop wash affects baro
     * - Creates low pressure near ground
     * - Baro reads higher altitude than actual
     * - Effect diminishes with altitude
     * - Typical: 0.5-2 m error near ground
     * 
     * Height Source Transitions:
     * 
     * GPS to Baro:
     * - Apply baroHgtOffset
     * - Ensures continuity
     * - Prevents control disturbance
     * 
     * Baro to Rangefinder:
     * - Similar offset compensation
     * - Smooth transition
     * 
     * Rangefinder to Baro:
     * - Reverse compensation
     * 
     * Update Timing:
     * - Called regularly during flight
     * - Typically at baro update rate (10-50 Hz)
     * - Continuous tracking
     * - Adapts to changing conditions
     * 
     * Baro Calibration:
     * - resetHeightDatum() resets baro
     * - Sets current altitude as zero
     * - Adjusts EKF origin height
     * - Maintains global height consistency
     * 
     * Temperature Effects:
     * - Barometer sensitive to temperature
     * - Slow temperature changes cause drift
     * - Offset tracking compensates
     * - Better than raw baro
     * 
     * Altitude Reference:
     * - Baro: Relative to pressure datum
     * - GPS: WGS-84 ellipsoid height
     * - Difference: Geoid undulation + baro setting
     * - Offset accounts for reference difference
     * 
     * Configuration:
     * - Filter time constant: Fixed in code
     * - Not user-configurable
     * - Tuned for typical flight conditions
     * 
     * Logging:
     * - baroHgtOffset logged
     * - Post-flight analysis
     * - Identify baro issues
     * - Validate height estimation
     * 
     * Failure Modes:
     * - Large offset: Baro malfunction
     * - Rapidly changing: Pressure system issue
     * - Stuck offset: Filter not updating
     * - Monitor for anomalies
     * 
     * @note Tracks slowly varying baro offset
     * @note Low-pass filtered for stability
     * @note Used for smooth height source transitions
     * @note Compensates ground effect and drift
     * 
     * @warning Large offset indicates baro problem
     * @warning Ground effect significant near ground
     * @warning Temperature affects baro accuracy
     * @warning Offset must be tracked continuously
     * 
     * @see baroHgtOffset for filtered offset value
     * @see meaHgtAtTakeOff for ground effect reference
     * @see selectHeightForFusion() for height source selection
     * @see resetHeightDatum() for baro reset
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:733
     */
    void calcFiltBaroOffset();

    /**
     * @brief Correct the EKF origin height to be consistent with GPS data
     * 
     * @details Uses a Bayes filter to gradually correct the EKF's WGS-84 origin height
     *          to align with GPS altitude measurements. Ensures long-term consistency
     *          between EKF height estimate and GPS altitude, accounting for barometer
     *          drift and initial calibration errors.
     * 
     * Origin Height Correction Problem:
     * 
     * Height References:
     * - EKF position[2]: Height down from origin (meters)
     * - EKF origin: LLH location, includes altitude
     * - ekfGpsRefHgt: WGS-84 origin altitude (meters)
     * - GPS altitude: WGS-84 ellipsoid height
     * 
     * Why Correction Needed:
     * - Initial origin height may be inaccurate
     * - Barometer drift causes accumulated error
     * - GPS altitude more stable long-term
     * - Origin correction maintains GPS consistency
     * 
     * Without Correction:
     * - EKF height drifts from GPS
     * - getLLH() returns incorrect altitude
     * - Waypoint altitude errors
     * - Terrain following errors
     * 
     * Bayes Filter Approach:
     * 
     * State:
     * - ekfGpsRefHgt: EKF origin WGS-84 altitude
     * - ekfOriginHgtVar: Variance of origin altitude
     * - Single-state estimator
     * 
     * Measurement:
     * - GPS altitude: WGS-84 height
     * - EKF height: stateStruct.position[2]
     * - Predicted GPS alt: ekfGpsRefHgt - EKF_height
     * - Innovation: GPS_alt - predicted_alt
     * 
     * Prediction:
     * - Origin height doesn't change (static)
     * - Variance may grow slightly (process noise)
     * - Accounts for modeling uncertainty
     * 
     * Update:
     * - Kalman gain based on variances
     * - K = ekfOriginHgtVar / (ekfOriginHgtVar + GPS_var + EKF_var)
     * - Correction: ekfGpsRefHgt += K × innovation
     * - Variance: ekfOriginHgtVar = (1 - K) × ekfOriginHgtVar
     * 
     * Update Conditions:
     * - GPS altitude available and healthy
     * - GPS accuracy acceptable (gpsHgtAccuracy < threshold)
     * - EKF height estimate healthy
     * - Time since last update > minimum interval
     * - lastOriginHgtTime_ms: Last update time
     * 
     * Update Rate:
     * - Slow: Typically every 10-30 seconds
     * - Gradual correction
     * - Avoids sudden changes
     * - Allows convergence over minutes
     * 
     * Innovation Validation:
     * - Check innovation magnitude
     * - Reject if excessive (> ~10 m)
     * - Indicates GPS or EKF problem
     * - Prevents bad correction
     * 
     * Convergence:
     * - Initial: Large uncertainty, fast correction
     * - Over time: Uncertainty decreases
     * - Steady state: Small corrections
     * - Typical convergence: 5-10 minutes
     * 
     * Impact on Height Estimate:
     * - EKF position[2] not directly affected
     * - Origin altitude adjusted
     * - getLLH() returns corrected altitude
     * - Maintains internal consistency
     * 
     * GPS Altitude Uncertainty:
     * - Vertical GPS less accurate than horizontal
     * - gpsHgtAccuracy: Reported uncertainty
     * - Typically 3-10 m (1-sigma)
     * - Larger than horizontal (1-2 m)
     * 
     * Barometer vs GPS:
     * - Baro: Short-term accurate, long-term drift
     * - GPS: Short-term noisy, long-term stable
     * - Origin correction: Anchors to GPS
     * - EKF: Uses both optimally
     * 
     * Geoid vs Ellipsoid:
     * - GPS: WGS-84 ellipsoid height
     * - Geoid: Mean sea level (MSL)
     * - Difference: Geoid undulation (-100 to +100 m)
     * - EKF uses ellipsoid (same as GPS)
     * 
     * Use Cases:
     * - Long-duration flights
     * - Barometer drift compensation
     * - Accurate terrain following
     * - Waypoint altitude accuracy
     * 
     * Configuration:
     * - Update rate: Fixed in code
     * - Innovation threshold: Fixed
     * - Not user-configurable
     * - Automatic correction
     * 
     * Logging:
     * - ekfGpsRefHgt logged
     * - ekfOriginHgtVar logged
     * - Innovation logged
     * - Post-flight analysis of corrections
     * 
     * Failure Modes:
     * - GPS altitude jumps: Reject update
     * - Large innovations: GPS or EKF problem
     * - Non-convergence: Systematic error
     * 
     * @note Corrects origin altitude gradually
     * @note Uses Bayes filter for optimal estimation
     * @note Maintains GPS altitude consistency
     * @note Compensates barometer drift
     * 
     * @warning GPS altitude less accurate than horizontal
     * @warning Large innovations indicate problems
     * @warning Convergence requires several minutes
     * @warning Critical for terrain following accuracy
     * 
     * @see ekfGpsRefHgt for origin WGS-84 altitude
     * @see ekfOriginHgtVar for origin altitude variance
     * @see lastOriginHgtTime_ms for last update time
     * @see getLLH() for altitude output
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:736
     */
    void correctEkfOriginHeight();

    /**
     * @brief Select height data source to fuse from available sensors
     * 
     * @details Intelligently selects which height measurement source (barometer, GPS,
     *          rangefinder, beacon, external nav) to use for EKF fusion based on
     *          availability, quality, flight phase, and configuration. Sets flags
     *          for fusion and manages smooth transitions between sources.
     * 
     * Height Source Options:
     * 
     * 1. Barometer (HGT_SOURCE_BARO = 0):
     *    - Default primary source
     *    - High update rate (10-50 Hz)
     *    - Low noise
     *    - Subject to drift
     *    - Ground effect near surface
     * 
     * 2. Rangefinder (HGT_SOURCE_RNG = 1):
     *    - Terrain-relative altitude
     *    - Used when available and enabled
     *    - Limited range (0.2-40 m typically)
     *    - Good for low altitude
     *    - Precision landing
     * 
     * 3. GPS (HGT_SOURCE_GPS = 2):
     *    - WGS-84 ellipsoid height
     *    - Lower accuracy than horizontal
     *    - No drift (long-term stable)
     *    - Backup when baro fails
     *    - Slow update rate (5-10 Hz)
     * 
     * 4. Range Beacon (HGT_SOURCE_BCN = 3):
     *    - 3D position from beacon system
     *    - Vertical component used
     *    - Indoor positioning
     *    - Requires beacon infrastructure
     * 
     * 5. External Nav (HGT_SOURCE_EXTNAV = 4):
     *    - External navigation system
     *    - Motion capture, external EKF
     *    - Typically high accuracy
     *    - Requires external source
     * 
     * Selection Logic:
     * 
     * Primary Source Selection:
     * - Check configuration parameters
     * - Check sensor availability
     * - Check sensor health
     * - Check flight phase
     * - activeHgtSource: Selected source
     * 
     * Rangefinder Priority:
     * - If enabled: EK2_RNG_AID_HGT
     * - If altitude < max range
     * - If quality acceptable
     * - If terrain stable: terrainHgtStable
     * - If speed < threshold: EK2_RNG_USE_SPD
     * - Then use rangefinder
     * 
     * Speed Threshold:
     * - EK2_RNG_USE_SPD: Max speed for rangefinder
     * - High speed: Rangefinder unreliable
     * - Switches to baro at high speed
     * - Prevents rangefinder errors
     * 
     * Terrain Stability:
     * - terrainHgtStable flag
     * - Set by setTerrainHgtStable()
     * - Stable: Use rangefinder
     * - Unstable: Use baro
     * - Prevents errors over water, vegetation
     * 
     * Fallback Hierarchy:
     * 1. Configured primary source
     * 2. Rangefinder (if conditions met)
     * 3. Barometer (default)
     * 4. GPS (if baro failed)
     * 5. Last known height (emergency)
     * 
     * Source Transition:
     * - Detect source change
     * - Apply offset for continuity
     * - baroHgtOffset used for baro transitions
     * - Smooth transition prevents jumps
     * 
     * Fusion Flags Set:
     * - fuseHgtData: Fuse height this cycle
     * - rangeDataToFuse: Rangefinder data ready
     * - baroDataToFuse: Baro data ready
     * - gpsDataToFuse: GPS data ready (height component)
     * 
     * Height Measurement Preparation:
     * - hgtMea: Selected height measurement
     * - From chosen source
     * - Properly referenced
     * - Ready for fusion
     * 
     * Observation Noise:
     * - posDownObsNoise: Height measurement variance
     * - Varies by source:
     *   * Baro: Low noise (~0.5 m std)
     *   * GPS: Higher noise (~3-10 m std)
     *   * Rangefinder: Very low (~0.1 m std)
     *   * Beacon: Medium (~1 m std)
     * 
     * Flight Phase Considerations:
     * - On ground: Use rangefinder if available
     * - Takeoff: Transition from rangefinder to baro
     * - Flight: Baro or GPS
     * - Landing: Transition to rangefinder
     * - Precision landing: Rangefinder critical
     * 
     * Altitude Limits:
     * - Rangefinder: Limited range
     * - Above max range: Switch to baro
     * - Below min range: May switch to baro
     * - Hysteresis prevents oscillation
     * 
     * Quality Checks:
     * - Sensor health status
     * - Innovation consistency
     * - Update rate
     * - Data freshness
     * - Timeout detection
     * 
     * Configuration Parameters:
     * - EK2_ALT_SOURCE: Primary altitude source
     * - EK2_RNG_AID_HGT: Rangefinder aiding enable
     * - EK2_RNG_USE_HGT: Rangefinder max altitude
     * - EK2_RNG_USE_SPD: Rangefinder max speed
     * 
     * Use Cases:
     * - Precision landing: Rangefinder
     * - High altitude cruise: Barometer
     * - Indoor flight: Rangefinder or beacon
     * - Baro failure: GPS fallback
     * 
     * @note Called every update cycle
     * @note Intelligently switches between sources
     * @note Smooth transitions prevent jumps
     * @note Flight phase aware
     * 
     * @warning Source transitions can cause transients
     * @warning Rangefinder limited range
     * @warning GPS altitude less accurate than horizontal
     * @warning Baro subject to drift
     * 
     * @see activeHgtSource for current height source
     * @see hgtMea for selected height measurement
     * @see fuseHgtData for fusion trigger flag
     * @see posDownObsNoise for measurement variance
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:739
     */
    void selectHeightForFusion();

    /**
     * @brief Zero attitude state covariances, but preserve diagonal variances
     * 
     * @details Zeroes the off-diagonal elements of the attitude error covariance matrix
     *          (correlations between attitude and other states) while preserving the
     *          diagonal variance terms. Used after attitude resets to remove invalid
     *          correlation information while maintaining uncertainty estimates.
     * 
     * Attitude Covariance Structure:
     * 
     * Attitude States (0-2):
     * - angErr[0]: Roll error (rotation vector)
     * - angErr[1]: Pitch error
     * - angErr[2]: Yaw error
     * - Covariance: P[0:2, 0:2]
     * 
     * Why Zero Covariances:
     * - After attitude reset (e.g., yaw reset)
     * - Old correlations invalid
     * - Reset changes attitude suddenly
     * - Correlations with other states wrong
     * - Must be rebuilt from scratch
     * 
     * Diagonal Preservation:
     * - P[0][0]: Roll variance (preserved)
     * - P[1][1]: Pitch variance (preserved)
     * - P[2][2]: Yaw variance (preserved)
     * - Individual uncertainties still valid
     * 
     * Off-Diagonal Zeroing:
     * - P[0:2, 3:23]: Attitude-other correlations
     * - P[3:23, 0:2]: Other-attitude correlations
     * - All set to zero
     * - Removes invalid correlation information
     * 
     * Covariance Matrix Sections Affected:
     * \n
     * Attitude rows/columns (0-2):\n
     * - vs Velocity (3-5): Zeroed\n
     * - vs Position (6-8): Zeroed\n
     * - vs Gyro bias (9-11): Zeroed\n
     * - vs Gyro scale (12-14): Zeroed\n
     * - vs Accel Z bias (15): Zeroed\n
     * - vs Earth mag (16-18): Zeroed\n
     * - vs Body mag (19-21): Zeroed\n
     * - vs Wind (22-23): Zeroed\n
     * 
     * Reset Scenarios:
     * 
     * 1. Yaw Reset:
     *    - resetQuatStateYawOnly() called
     *    - Yaw changed significantly
     *    - Yaw-other correlations invalid
     *    - Zero yaw covariances
     * 
     * 2. Magnetic Field Reset:
     *    - Mag field states reset
     *    - Attitude-mag correlations invalid
     *    - Zero attitude covariances
     * 
     * 3. GPS Yaw Alignment:
     *    - realignYawGPS() called
     *    - Yaw aligned to GPS course
     *    - Old yaw correlations wrong
     * 
     * 4. EKFGSF Yaw Reset:
     *    - EKFGSF_resetMainFilterYaw()
     *    - Emergency yaw reset
     *    - All yaw correlations invalid
     * 
     * Covariance Rebuilding:
     * - After zeroing, correlations rebuild naturally
     * - Prediction step couples states
     * - Measurement updates create correlations
     * - Converges over time (seconds to minutes)
     * 
     * Implementation:
     * - zeroRows(P, 0, 2): Zero rows 0-2
     * - zeroCols(P, 0, 2): Zero columns 0-2
     * - Preserve P[0][0], P[1][1], P[2][2]
     * - Symmetry maintained
     * 
     * Matrix Symmetry:
     * - P must remain symmetric
     * - Zero rows and columns together
     * - P[i][j] = P[j][i] after zeroing
     * - Critical for filter stability
     * 
     * Variance Magnitude:
     * - Diagonal preserved at current values
     * - May be small (well-determined)
     * - Or large (uncertain)
     * - Depends on state before reset
     * 
     * Impact on Filter:
     * - Short-term: Reduced state coupling
     * - Medium-term: Correlations rebuild
     * - Long-term: Normal operation
     * - Temporary effect only
     * 
     * Alternative Approach:
     * - Could increase diagonal variances
     * - Indicates increased uncertainty
     * - This method: Keep uncertainty, lose correlation
     * - Trade-off decision
     * 
     * Use in Reset Sequence:
     * 1. Compute new attitude (yaw)
     * 2. Update quaternion states
     * 3. Update attitude variances if needed
     * 4. Zero attitude covariances (this method)
     * 5. Record reset event
     * 6. Continue operation
     * 
     * @note Preserves diagonal variances
     * @note Zeros off-diagonal covariances
     * @note Used after attitude resets
     * @note Correlations rebuild naturally
     * 
     * @warning Invalid correlations cause filter errors
     * @warning Must zero after significant resets
     * @warning Symmetry must be maintained
     * @warning Temporary reduction in state coupling
     * 
     * @see zeroRows() for row zeroing
     * @see zeroCols() for column zeroing
     * @see resetQuatStateYawOnly() for yaw reset
     * @see recordYawReset() for reset logging
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:742
     */
    void zeroAttCovOnly();

    /**
     * @brief Record a yaw reset event for reporting to user
     * 
     * @details Records the magnitude and time of a yaw angle reset so that
     *          external systems can be notified. Stores reset angle and
     *          timestamp for retrieval by getLastYawResetAngle().
     * 
     * Yaw Reset Events:
     * - Large corrections to yaw angle
     * - Discontinuous change in heading
     * - External systems need notification
     * - Controllers must adapt
     * 
     * Information Recorded:
     * - yawResetAngle: Change in yaw (radians)
     * - lastYawReset_ms: System time of reset
     * - Sign convention: Positive = yaw increased
     * 
     * Yaw Reset Sources:
     * 
     * 1. GPS Course Alignment:
     *    - realignYawGPS() called
     *    - Align yaw to GPS ground track
     *    - Typically at initialization
     *    - Or after magnetic anomaly
     * 
     * 2. EKFGSF Yaw Reset:
     *    - EKFGSF_resetMainFilterYaw()
     *    - Emergency yaw estimator
     *    - Used when mag fusion fails
     *    - Provides backup yaw estimate
     * 
     * 3. External Nav Yaw:
     *    - External navigation system
     *    - Motion capture, T265, etc.
     *    - Provides direct yaw observation
     * 
     * 4. Magnetic Anomaly Recovery:
     *    - Detected large mag disturbance
     *    - Reset yaw to last known good
     *    - Or to GPS course
     *    - magYawAnomallyCount incremented
     * 
     * Reset Angle Calculation:
     * - old_yaw: Before reset
     * - new_yaw: After reset
     * - yawResetAngle = new_yaw - old_yaw
     * - Wrapped to ±π
     * 
     * Timestamp:
     * - lastYawReset_ms = AP_HAL::millis()
     * - Used to identify most recent reset
     * - Retrieved by getLastYawResetAngle()
     * - Allows time-based filtering
     * 
     * User Notification:
     * - getLastYawResetAngle() returns:
     *   * Reset angle (radians)
     *   * Reset timestamp (ms)
     * - Controllers can compensate
     * - Rate limiter adjustments
     * - Feed-forward corrections
     * 
     * Impact on Vehicle:
     * - Heading changes suddenly
     * - Rate controller sees step input
     * - Position controller affected
     * - Waypoint tracking adjusts
     * 
     * Controller Adaptation:
     * - Attitude controller notified
     * - Rate feed-forward adjusted
     * - Prevents overshoot
     * - Smoother response
     * 
     * Multiple Resets:
     * - Each reset overwrites previous
     * - Only most recent stored
     * - Count of resets tracked separately
     * - EKFGSF_yaw_reset_count
     * 
     * Logging:
     * - Reset events logged
     * - Angle magnitude logged
     * - Timestamp logged
     * - Post-flight analysis
     * 
     * Use Cases:
     * - Magnetic anomaly recovery
     * - Initialization yaw acquisition
     * - GPS-denied navigation transitions
     * - Indoor/outdoor transitions
     * 
     * @note Records reset angle and timestamp
     * @note Only most recent reset stored
     * @note Retrieved by getLastYawResetAngle()
     * @note Controllers use for adaptation
     * 
     * @warning Yaw resets disturb flight control
     * @warning Controllers must be notified
     * @warning Frequent resets indicate problems
     * @warning Monitor reset count
     * 
     * @see yawResetAngle for reset angle storage
     * @see lastYawReset_ms for reset timestamp
     * @see getLastYawResetAngle() for retrieval
     * @see EKFGSF_yaw_reset_count for reset count
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:745
     */
    void recordYawReset();

    /**
     * @brief Record a magnetic field state reset event for analysis
     * 
     * @details Records the quaternion state and yaw innovation at the time of a
     *          magnetic field reset. Used to track magnetic anomalies and assess
     *          reset decisions. Helps detect magnetic interference patterns.
     * 
     * Magnetic Field Reset:
     * - Earth mag field states (16-18) reset
     * - Body mag field states (19-21) reset
     * - Usually triggered by innovation failures
     * - Or magnetic anomaly detection
     * 
     * Information Recorded:
     * - quatAtLastMagReset: Quaternion at reset
     * - posDownAtLastMagReset: Altitude at reset
     * - yawInnovAtLastMagReset: Yaw innovation magnitude
     * - Used for anomaly analysis
     * 
     * Why Record Reset Events:
     * \n
     * 1. Anomaly Detection:\n
     *    - Magnetic interference patterns\n
     *    - Power line interference\n
     *    - Ferrous material near compass\n
     *    - Vehicle magnetic signature changes\n
     * \n
     * 2. Reset Decision Validation:\n
     *    - Was reset justified?\n
     *    - Innovation magnitude check\n
     *    - Altitude correlation (ground effect)\n
     *    - Attitude at reset time\n
     * \n
     * 3. Flight Phase Analysis:\n
     *    - Resets during takeoff expected\n
     *    - Resets during cruise unusual\n
     *    - Altitude correlation\n
     *    - Pattern identification\n
     * 
     * Magnetic Anomaly Tracking:
     * - magYawAnomallyCount: Number of resets
     * - MAG_ANOMALY_RESET_MAX: Maximum allowed (2)
     * - Excessive resets: Disable mag fusion
     * - Safety mechanism
     * 
     * Reset Triggers:
     * \n
     * 1. Large Yaw Innovation:\n
     *    - innovYaw: Yaw innovation (rad)\n
     *    - Exceeds threshold (typically >0.5 rad)\n
     *    - Sustained over time\n
     *    - magYawResetTimer_ms tracks duration\n
     * \n
     * 2. Magnetic Field Inconsistency:\n
     *    - Measured field vs estimated field\n
     *    - Large magnitude error\n
     *    - Declination error\n
     *    - Total field magnitude error\n
     * \n
     * 3. Post-Takeoff Reset:\n
     *    - finalInflightMagInit flag\n
     *    - Reset after leaving ground effect\n
     *    - Altitude: EK2_MAG_FINAL_RESET_ALT (2.5 m)\n
     *    - One-time event\n
     * 
     * Altitude Correlation:
     * - posDownAtLastMagReset stored
     * - Compare to current altitude
     * - Ground-related resets: Low altitude
     * - In-flight anomalies: Any altitude
     * - Pattern analysis
     * 
     * Quaternion Storage:
     * - quatAtLastMagReset: Full attitude
     * - Compare to current attitude
     * - Detect if attitude changed significantly
     * - May indicate magnetic disturbance source
     * 
     * Yaw Innovation:
     * - yawInnovAtLastMagReset: Innovation magnitude
     * - Indicates severity of disturbance
     * - Large innovation: Major anomaly
     * - Threshold validation
     * 
     * Reset Decision Logic:
     * - controlMagYawReset() uses these records
     * - Determines if reset needed
     * - Checks anomaly count
     * - Altitude-based gating
     * - Time since last reset
     * 
     * Anomaly Recovery:
     * - First reset: Try to recover
     * - Second reset: Try again
     * - Third reset: Give up, disable mag
     * - Prevents repeated failures
     * 
     * Magnetic Field Re-Learning:
     * - After reset, field states re-learned
     * - Converges over 30-60 seconds
     * - earthMagFieldVar, bodyMagFieldVar updated
     * - magFieldLearned flag set when converged
     * 
     * Logging:
     * - Reset events logged
     * - Quaternion logged
     * - Altitude logged
     * - Innovation logged
     * - Post-flight analysis
     * 
     * Use Cases:
     * - Magnetic interference debugging
     * - Anomaly pattern identification
     * - Flight phase correlation
     * - Reset decision validation
     * 
     * @note Records quaternion, altitude, innovation
     * @note Used for anomaly tracking
     * @note Validates reset decisions
     * @note Helps identify interference sources
     * 
     * @warning Frequent resets indicate problems
     * @warning Exceeding reset limit disables mag
     * @warning Monitor reset patterns
     * @warning Investigate repeated resets
     * 
     * @see quatAtLastMagReset for quaternion storage
     * @see posDownAtLastMagReset for altitude storage
     * @see yawInnovAtLastMagReset for innovation storage
     * @see magYawAnomallyCount for reset counter
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:748
     */
    void recordMagReset();

    /**
     * @brief Return effective value of MAG_CAL parameter
     * 
     * @details Determines the effective magnetometer calibration mode to use,
     *          considering vehicle type, configuration, and flight conditions.
     *          Returns calibration mode for compass offset learning behavior.
     * 
     * @return Effective MAG_CAL value (0-5)
     * \n
     * 0 = Never learn offsets\n
     * 1 = Learn offsets when disarmed\n
     * 2 = Learn offsets all the time\n
     * 3 = Learn body field offsets\n
     * 4 = Learn both earth and body fields\n
     * 5 = Advanced calibration mode\n
     * 
     * MAG_CAL Parameter:
     * - User-configured magnetometer calibration mode
     * - Controls when/how compass offsets learned
     * - Trade-off: Accuracy vs adaptation
     * 
     * Vehicle Type Considerations:
     * - Fixed-wing: Different than multirotor
     * - Multirotor: More dynamics, more learning
     * - Rover: Ground interference
     * - Sub: Minimal magnetic disturbance
     * 
     * Configuration Override:
     * - EK2_MAG_CAL parameter
     * - If set: Use this value
     * - If not set: Use vehicle default
     * - Allows per-vehicle tuning
     * 
     * Default Calibration Modes:
     * \n
     * Multirotor:\n
     * - Default: MAG_CAL = 4\n
     * - Learn earth and body fields\n
     * - Adapts to motor magnetic fields\n
     * - Critical for small vehicles\n
     * \n
     * Fixed-Wing:\n
     * - Default: MAG_CAL = 1 or 2\n
     * - Learn offsets when safe\n
     * - Less motor interference\n
     * - More stable fields\n
     * \n
     * Rover:\n
     * - Default: MAG_CAL = 1\n
     * - Learn when disarmed\n
     * - Avoid ground interference\n
     * 
     * Calibration Behavior:
     * \n
     * MAG_CAL = 0 (Never):\n
     * - No offset learning\n
     * - Use pre-calibrated offsets\n
     * - Most conservative\n
     * - For stable magnetic environments\n
     * \n
     * MAG_CAL = 1 (Disarmed):\n
     * - Learn only when disarmed\n
     * - Safe, controlled learning\n
     * - Avoids in-flight errors\n
     * - Good for most vehicles\n
     * \n
     * MAG_CAL = 2 (Always):\n
     * - Continuous learning\n
     * - Adapts to changing fields\n
     * - Risk of learning bad data\n
     * - Use with caution\n
     * \n
     * MAG_CAL = 3 (Body field):\n
     * - Learn body magnetic field\n
     * - Compensates motor fields\n
     * - States 19-21: body_magfield\n
     * - Good for vehicles with motors\n
     * \n
     * MAG_CAL = 4 (Earth and body):\n
     * - Learn both fields\n
     * - Most comprehensive\n
     * - States 16-21 all learned\n
     * - Best for dynamic vehicles\n
     * 
     * Impact on EKF:
     * - inhibitMagStates flag\n
     * - MAG_CAL = 0: inhibitMagStates = true\n
     * - MAG_CAL > 0: Allow learning\n
     * - stateIndexLim affected\n
     * 
     * Learning Conditions:
     * - Motors armed/disarmed check
     * - Sufficient excitation for learning
     * - Innovation consistency
     * - Time to converge
     * 
     * Use Cases:
     * - Multirotor: MAG_CAL = 4 (full learning)
     * - Fixed-wing: MAG_CAL = 1 or 2
     * - Indoor flight: MAG_CAL = 0 (don't trust mag)
     * - Magnetic environment: MAG_CAL = 0
     * 
     * @note Returns effective calibration mode
     * @note Considers vehicle type and configuration
     * @note Controls compass offset learning
     * @note Critical for magnetic field estimation
     * 
     * @warning Incorrect setting causes heading errors
     * @warning MAG_CAL = 2 risky (continuous learning)
     * @warning MAG_CAL = 0 requires good calibration
     * @warning Vehicle-specific tuning important
     * 
     * @see inhibitMagStates for learning control flag
     * @see stateStruct.earth_magfield for earth field (16-18)
     * @see stateStruct.body_magfield for body field (19-21)
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:751
     */
    uint8_t effective_magCal(void) const;

    /**
     * @brief Update timing statistics structure for performance monitoring
     * 
     * @details Records timing information for EKF update cycles, including time
     *          since last prediction, IMU delta time, and EKF average time step.
     *          Used for performance analysis, loop rate monitoring, and debugging.
     * 
     * Timing Statistics Structure:
     * - struct ekf_timing timing
     * - Contains timing metrics
     * - Logged for post-flight analysis
     * - Performance monitoring
     * 
     * Metrics Recorded:
     * \n
     * 1. Time Since Last Predict:\n
     *    - Time since last prediction cycle\n
     *    - Should match IMU sample rate\n
     *    - Typically 2.5 ms (400 Hz) for copters\n
     *    - Or 10 ms (100 Hz) for planes\n
     * \n
     * 2. IMU Delta Time:\n
     *    - dtIMUavg: Average IMU sample period\n
     *    - Calculated from delAngDT, delVelDT\n
     *    - Should be stable\n
     *    - Variations indicate timing issues\n
     * \n
     * 3. EKF Delta Time:\n
     *    - dtEkfAvg: Average EKF update period\n
     *    - May differ from IMU rate\n
     *    - Down-sampling for computational load\n
     *    - Target: EKF_TARGET_DT (0.01s = 100Hz)\n
     * \n
     * 4. Current Delta Time:\n
     *    - dt: Time since last covariance prediction\n
     *    - Varies slightly cycle to cycle\n
     *    - Used in filter equations\n
     *    - Critical for correct integration\n
     * 
     * Timing Consistency Checks:
     * - Detect IMU sample timing jitter
     * - Detect EKF update timing jitter
     * - Identify computational overload
     * - Trigger warnings if excessive
     * 
     * Performance Metrics:
     * - framesSincePredict: Frames since last prediction
     * - Load balancing across EKF cores
     * - Prevents all cores running simultaneously
     * - Distributes computational load
     * 
     * Local Filter Time Step:
     * - localFilterTimeStep_ms: Average update interval
     * - Measured over multiple cycles
     * - Should match target rate
     * - Deviations indicate problems
     * 
     * Use Cases:
     * \n
     * 1. Performance Monitoring:\n
     *    - Ensure EKF running at target rate\n
     *    - Detect computational issues\n
     *    - Identify timing problems\n
     * \n
     * 2. Debug Timing Issues:\n
     *    - IMU sample rate verification\n
     *    - EKF update rate verification\n
     *    - Jitter analysis\n
     *    - Load balancing check\n
     * \n
     * 3. Post-Flight Analysis:\n
     *    - Log timing statistics\n
     *    - Correlate with flight events\n
     *    - Identify performance degradation\n
     *    - Validate timing assumptions\n
     * 
     * Timing Logging:
     * - Log_Write_Timing() uses this data
     * - lastTimingLogTime_ms: Last log time
     * - Periodic logging (not every cycle)
     * - Reduces log data volume
     * 
     * Expected Values:
     * \n
     * Multirotor:\n
     * - IMU rate: 400-1000 Hz\n
     * - EKF rate: 100-400 Hz\n
     * - dtIMUavg: 1-2.5 ms\n
     * - dtEkfAvg: 2.5-10 ms\n
     * \n
     * Fixed-Wing:\n
     * - IMU rate: 100-400 Hz\n
     * - EKF rate: 50-100 Hz\n
     * - dtIMUavg: 2.5-10 ms\n
     * - dtEkfAvg: 10-20 ms\n
     * 
     * Timing Issues Detection:
     * - Large dtIMUavg: Slow IMU sampling
     * - Variable dt: Jitter or scheduling issues
     * - High framesSincePredict: Computational overload
     * - dtEkfAvg >> target: Can't keep up
     * 
     * Computational Load:
     * - EKF computationally intensive
     * - Multiple cores share CPU time
     * - Timing statistics verify CPU adequate
     * - May need to reduce EKF rate if overloaded
     * 
     * @note Records EKF timing performance metrics
     * @note Used for performance monitoring
     * @note Logged for post-flight analysis
     * @note Critical for debugging timing issues
     * 
     * @warning Timing jitter affects filter accuracy
     * @warning Computational overload causes delays
     * @warning Must maintain target update rate
     * @warning Monitor timing statistics regularly
     * 
     * @see timing for timing statistics structure
     * @see dtIMUavg for average IMU delta time
     * @see dtEkfAvg for average EKF delta time
     * @see framesSincePredict for load balancing
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:754
     */
    void updateTimingStatistics(void);

    /**
     * @brief Correct GPS data for antenna position offset from IMU
     * 
     * @details Adjusts GPS position and velocity measurements to account for the
     *          GPS antenna's physical offset from the IMU/EKF origin in the vehicle
     *          body frame. Ensures GPS data correctly represents the EKF reference
     *          point, improving accuracy especially during turns and maneuvers.
     * 
     * @param[in,out] gps_data GPS measurement to correct (position, velocity updated)
     * 
     * Antenna Offset Problem:
     * - GPS antenna not at IMU location
     * - Typically offset: 10-30 cm
     * - During turns: Antenna traces larger circle
     * - Velocity at antenna ≠ velocity at IMU
     * - Position offset causes errors
     * 
     * Body Frame Offset:
     * - GPS_POS_* parameters: Antenna position
     * - Body frame: Forward (X), Right (Y), Down (Z)
     * - Relative to IMU location
     * - Example: GPS_POS_X = 0.2 m (20 cm forward)
     * 
     * Velocity Correction:
     * \n
     * Angular Velocity Effect:\n
     * - Vehicle rotating: Angular velocity ω\n
     * - Antenna offset: r (body frame)\n
     * - Velocity offset: v_offset = ω × r\n
     * - Must subtract offset from GPS velocity\n
     * \n
     * Correction Equation:\n
     * - ω: Angular velocity (body frame, rad/s)\n
     * - r: Antenna offset (body frame, m)\n
     * - v_offset = ω × r (cross product)\n
     * - v_corrected = v_gps - v_offset\n
     * \n
     * Earth Frame:\n
     * - GPS velocity in NED frame\n
     * - Rotate offset to NED: Tbn × (ω × r)\n
     * - Subtract from GPS velocity\n
     * - Result: Velocity at IMU location\n
     * 
     * Position Correction:
     * \n
     * Static Offset:\n
     * - Antenna offset in body frame\n
     * - Rotate to earth frame: Tbn × r\n
     * - Subtract from GPS position\n
     * - Result: Position of IMU\n
     * \n
     * Correction Equation:\n
     * - r: Antenna offset (body frame)\n
     * - Tbn: Body to NED rotation matrix\n
     * - offset_NED = Tbn × r\n
     * - pos_corrected = pos_gps - offset_NED\n
     * 
     * Correction Magnitude:
     * \n
     * Example:\n
     * - Antenna 20 cm forward of IMU\n
     * - Vehicle pitch 10°\n
     * - Altitude error: ~3.5 cm\n
     * - Turn rate 30°/s, 20 cm offset\n
     * - Velocity error: ~10 cm/s\n
     * 
     * When Correction Important:
     * - Fast turns: Large angular velocity
     * - Large offsets: > 10 cm
     * - Precision positioning required
     * - During aggressive maneuvers
     * 
     * When Correction Negligible:
     * - Slow flight or hover
     * - Small offsets: < 5 cm
     * - Low precision requirements
     * - Straight and level flight
     * 
     * Configuration:
     * - GPS_POS_X: Forward offset (m)
     * - GPS_POS_Y: Right offset (m)
     * - GPS_POS_Z: Down offset (m)
     * - Should be measured accurately
     * 
     * Measurement:
     * - Measure from IMU to GPS antenna
     * - Body frame coordinates
     * - Positive: Forward, right, down
     * - Accuracy: ± 1 cm recommended
     * 
     * Multiple GPS:
     * - Each GPS has own offset
     * - Correction per GPS instance
     * - sensor_idx identifies which GPS
     * 
     * IMU Location:
     * - EKF origin at primary IMU
     * - All measurements referenced to IMU
     * - Consistent reference frame
     * - Body offsets account for physical separation
     * 
     * Attitude Dependency:
     * - Position correction depends on attitude
     * - Uses current quaternion estimate
     * - Attitude errors affect correction
     * - Usually negligible error
     * 
     * Angular Velocity Source:
     * - From IMU gyroscopes
     * - Bias-corrected
     * - Body frame rates
     * - delAng / dt for current rates
     * 
     * @note Corrects for antenna offset from IMU
     * @note Important during turns and maneuvers
     * @note Uses GPS_POS_* parameters
     * @note Requires accurate offset measurement
     * 
     * @warning Incorrect offset causes systematic errors
     * @warning Large offsets increase correction importance
     * @warning Attitude errors affect position correction
     * @warning Must configure offsets accurately
     * 
     * @see GPS_POS_X, GPS_POS_Y, GPS_POS_Z for offset parameters
     * @see CorrectExtNavForSensorOffset() for similar external nav correction
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:757
     */
    void CorrectGPSForAntennaOffset(gps_elements &gps_data) const;

    /**
     * @brief Correct external navigation position for sensor body-frame offset
     * 
     * @details Adjusts external navigation position measurement to account for the
     *          external nav sensor's physical offset from the IMU in the vehicle body
     *          frame. Similar to GPS antenna correction but for external nav sensors
     *          like motion capture systems, T265 cameras, or external EKFs.
     * 
     * @param[in,out] ext_position External nav position in NED frame (meters), corrected in place
     * 
     * External Navigation Sensors:
     * - Motion capture systems (Vicon, OptiTrack)
     * - Intel RealSense T265 camera
     * - External EKF/SLAM systems
     * - Lighthouse tracking
     * - AprilTag positioning
     * 
     * Sensor Offset Problem:
     * - External nav sensor not at IMU
     * - Camera/tracker at different location
     * - Typically offset: 5-20 cm
     * - Position reported at sensor, not IMU
     * - Must correct to IMU location
     * 
     * Body Frame Offset:
     * - Sensor position relative to IMU
     * - From AP_VisualOdom library
     * - Body frame: Forward (X), Right (Y), Down (Z)
     * - Example: Camera 10 cm forward, 5 cm up
     * 
     * Position Correction:
     * \n
     * Static Offset:\n
     * - Sensor offset in body frame: r\n
     * - Rotate to earth frame: offset_NED = Tbn × r\n
     * - Subtract from sensor position\n
     * - pos_IMU = pos_sensor - offset_NED\n
     * \n
     * Rotation Matrix:\n
     * - Tbn: Current body to NED rotation\n
     * - From EKF quaternion estimate\n
     * - Transforms body frame offset to NED\n
     * - Attitude-dependent correction\n
     * 
     * Correction Importance:
     * - Critical for accuracy
     * - External nav often high precision
     * - Small errors matter
     * - Typical offsets 10-20 cm significant
     * 
     * Example Magnitudes:
     * - Sensor 15 cm forward of IMU
     * - Vehicle pitch 20°
     * - Vertical error: ~5 cm (important!)
     * - Horizontal error: ~14 cm
     * 
     * Configuration:
     * - VISO_POS_* parameters in AP_VisualOdom
     * - VISO_POS_X: Forward offset (m)
     * - VISO_POS_Y: Right offset (m)
     * - VISO_POS_Z: Down offset (m)
     * - Pulled from AP_VisualOdom library
     * 
     * Measurement:
     * - Measure from IMU to sensor
     * - Body frame coordinates
     * - Positive: Forward, right, down
     * - Accuracy important (± 1 cm)
     * 
     * T265 Camera Example:
     * - Camera typically forward and up
     * - VISO_POS_X: +0.10 m (10 cm forward)
     * - VISO_POS_Y: 0.0 m (centerline)
     * - VISO_POS_Z: -0.05 m (5 cm up)
     * 
     * Motion Capture:
     * - Marker position tracked
     * - Marker not at IMU
     * - Offset correction critical
     * - High precision system needs accuracy
     * 
     * Attitude Dependency:
     * - Correction depends on vehicle attitude
     * - Uses EKF quaternion estimate
     * - Initial attitude uncertainty affects correction
     * - Converges as attitude improves
     * 
     * Comparison to GPS Correction:
     * - Same principle as GPS antenna
     * - CorrectGPSForAntennaOffset() analogous
     * - Both transform body offset to earth frame
     * - Both subtract from measurement
     * 
     * Velocity Correction:
     * - Separate function: CorrectExtNavVelForSensorOffset()
     * - Accounts for angular velocity
     * - Cross product: ω × r
     * - Similar to GPS velocity correction
     * 
     * Use Cases:
     * - Indoor positioning (motion capture)
     * - GPS-denied navigation
     * - Precision positioning
     * - Visual-inertial odometry
     * 
     * Impact on Accuracy:
     * - Uncorrected: Systematic position error
     * - Error magnitude = offset × sin(attitude)
     * - Can be 5-20 cm easily
     * - Correction restores sub-cm accuracy
     * 
     * @note Corrects for sensor offset from IMU
     * @note Critical for external nav accuracy
     * @note Uses VISO_POS_* parameters
     * @note Attitude-dependent correction
     * 
     * @warning Incorrect offset causes systematic errors
     * @warning High precision systems need accurate offset
     * @warning Attitude errors affect correction
     * @warning Must configure offsets accurately
     * 
     * @see VISO_POS_X, VISO_POS_Y, VISO_POS_Z for offset parameters
     * @see CorrectExtNavVelForSensorOffset() for velocity correction
     * @see CorrectGPSForAntennaOffset() for similar GPS correction
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:760
     */
    void CorrectExtNavForSensorOffset(Vector3F &ext_position) const;

    /**
     * @brief Correct external navigation velocity for sensor body-frame offset
     * 
     * @details Adjusts external navigation velocity measurement to account for the
     *          sensor's offset from the IMU combined with vehicle angular velocity.
     *          Corrects for the additional velocity component that the offset sensor
     *          experiences during vehicle rotation.
     * 
     * @param[in,out] ext_velocity External nav velocity in NED frame (m/s), corrected in place
     * 
     * Velocity Offset Problem:
     * - External nav sensor offset from IMU
     * - During rotation: Sensor has additional velocity
     * - v_offset = ω × r (angular velocity × offset)
     * - Must correct to IMU velocity
     * 
     * Angular Velocity Effect:
     * - Vehicle rotating: Angular velocity ω (rad/s)
     * - Sensor offset: r (meters, body frame)
     * - Additional velocity: v = ω × r (m/s)
     * - Cross product in body frame
     * 
     * Correction Equation:
     * \n
     * Body Frame:\n
     * - ω: Angular velocity (body frame, rad/s)\n
     * - r: Sensor offset (body frame, m)\n
     * - v_offset_body = ω × r\n
     * \n
     * Earth Frame:\n
     * - Transform to NED: v_offset_NED = Tbn × v_offset_body\n
     * - Subtract from sensor velocity\n
     * - v_IMU = v_sensor - v_offset_NED\n
     * 
     * Cross Product:
     * - ω = [ωx, ωy, ωz] (body frame)
     * - r = [rx, ry, rz] (body frame)
     * - ω × r = [ωy×rz - ωz×ry, ωz×rx - ωx×rz, ωx×ry - ωy×rx]
     * 
     * Example Magnitude:
     * - Turn rate: 30°/s = 0.52 rad/s
     * - Sensor offset: 15 cm forward
     * - Velocity offset: ~8 cm/s (significant!)
     * 
     * When Correction Important:
     * - Fast rotations (turns, maneuvers)
     * - Large sensor offsets (> 10 cm)
     * - Precision velocity required
     * - Aggressive flight
     * 
     * When Correction Negligible:
     * - Slow flight or hover
     * - Small offsets (< 5 cm)
     * - Low angular rates
     * - Coarse velocity acceptable
     * 
     * Angular Velocity Source:
     * - From IMU gyroscopes
     * - Bias-corrected: gyro_bias states (9-11)
     * - Body frame rates
     * - Latest IMU measurement
     * 
     * Sensor Offset Source:
     * - AP_VisualOdom library
     * - VISO_POS_* parameters
     * - Same offset as position correction
     * - Body frame coordinates
     * 
     * Rotation Matrix:
     * - Tbn: Body to NED transformation
     * - From EKF quaternion
     * - Transforms body velocity offset to NED
     * - Current attitude used
     * 
     * T265 Example:
     * - Camera 10 cm forward, 5 cm up
     * - Yaw rate 45°/s (0.79 rad/s)
     * - Lateral velocity offset: ~8 cm/s
     * - Must correct for accuracy
     * 
     * Motion Capture:
     * - Marker offset from IMU
     * - System tracks marker velocity
     * - High precision requires correction
     * - Even small offsets matter
     * 
     * Comparison to GPS:
     * - Same principle as GPS antenna correction
     * - CorrectGPSForAntennaOffset() does similar
     * - Both use ω × r
     * - Both transform to earth frame
     * 
     * Position vs Velocity Correction:
     * - Position: Static offset (attitude-dependent)
     * - Velocity: Dynamic offset (angular velocity-dependent)
     * - Both needed for complete correction
     * - Independent corrections
     * 
     * Use Cases:
     * - Visual-inertial odometry
     * - Motion capture systems
     * - Indoor navigation
     * - High-precision positioning
     * 
     * Impact on Accuracy:
     * - Uncorrected: Velocity errors during turns
     * - Error = angular_rate × offset
     * - Can be 5-15 cm/s
     * - Correction critical for precision
     * 
     * Multiple Sensors:
     * - Each sensor has own offset
     * - Correction per sensor
     * - Use appropriate offset parameters
     * 
     * @note Corrects for angular velocity effect on offset sensor
     * @note Uses cross product: ω × r
     * @note Important during rotations
     * @note Requires accurate sensor offset
     * 
     * @warning Incorrect offset causes velocity errors
     * @warning Large offsets amplify errors
     * @warning High angular rates increase error
     * @warning Critical for precision navigation
     * 
     * @see VISO_POS_* for sensor offset parameters
     * @see CorrectExtNavForSensorOffset() for position correction
     * @see CorrectGPSForAntennaOffset() for similar GPS correction
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:763
     */
    void CorrectExtNavVelForSensorOffset(Vector3F &ext_velocity) const;

    /**
     * @brief Run IMU prediction step for independent GSF yaw estimator
     * 
     * @details Runs the prediction step for the Gaussian Sum Filter (GSF) yaw estimator,
     *          an independent backup yaw estimation algorithm. Uses IMU delta angles and
     *          optional airspeed data. Provides emergency yaw estimate when magnetometer
     *          fusion fails or magnetic anomalies detected.
     * 
     * GSF Yaw Estimator:
     * - EKFGSF_yaw: Independent yaw estimator
     * - Gaussian Sum Filter: Multiple hypothesis tracker
     * - Runs in parallel with main EKF
     * - Backup for magnetometer failures
     * 
     * Why GSF Yaw Estimator:
     * \n
     * Magnetometer Problems:\n
     * - Magnetic anomalies common\n
     * - Power lines, metal structures\n
     * - Vehicle magnetic interference\n
     * - Compass calibration errors\n
     * \n
     * Backup Yaw Source:\n
     * - Independent of magnetometer\n
     * - Uses GPS velocity for yaw\n
     * - Multiple hypotheses (robust)\n
     * - Emergency yaw reset capability\n
     * 
     * Gaussian Sum Filter Concept:
     * - Multiple parallel EKF filters (typically 5-7)
     * - Each with different initial yaw hypothesis
     * - All run simultaneously
     * - Best hypothesis emerges over time
     * - Weights track probability
     * 
     * Prediction Step:
     * \n
     * IMU Integration:\n
     * - Each GSF hypothesis predicts using IMU\n
     * - Delta angles: Gyro integration\n
     * - Bias-corrected gyro rates\n
     * - Independent yaw propagation\n
     * \n
     * Process Noise:\n
     * - Gyro bias uncertainty\n
     * - Gyro measurement noise\n
     * - Model uncertainty\n
     * - Covariance prediction\n
     * \n
     * Multiple Hypotheses:\n
     * - Each filter predicts independently\n
     * - Different yaw states\n
     * - Same IMU data\n
     * - Parallel processing\n
     * 
     * Airspeed Integration:
     * - Optional: If airspeed available
     * - Provides velocity magnitude constraint
     * - Helps with GPS velocity ambiguity
     * - Improves convergence
     * 
     * State Vector (per hypothesis):
     * - Velocity North (m/s)
     * - Velocity East (m/s)
     * - Yaw angle (rad)
     * - Gyro Z bias (rad/s)
     * - Simple 4-state filter
     * 
     * Prediction Equations:
     * - Velocity: v_k+1 = v_k (no acceleration model)
     * - Yaw: yaw_k+1 = yaw_k + (gyro_z - bias) × dt
     * - Bias: bias_k+1 = bias_k (random walk)
     * - Covariance: P_k+1 = F × P_k × F' + Q
     * 
     * When Activated:
     * - EKFGSF_run_filterbank flag set
     * - Activated conditions:
     *   * Mag fusion consistently failing
     *   * Yaw innovation excessive
     *   * Magnetic anomaly detected
     *   * GPS available for correction
     * 
     * Activation Logic:
     * - controlMagYawReset() decides
     * - After mag problems persist
     * - Provides backup yaw source
     * - Runs continuously once started
     * 
     * Computational Cost:
     * - Multiple filters running
     * - More CPU than single filter
     * - Worth it for robustness
     * - Only runs when needed
     * 
     * Integration with Main EKF:
     * - Runs independently
     * - Main EKF may query GSF yaw
     * - runYawEstimatorCorrection() gets yaw
     * - Main EKF can reset to GSF yaw
     * 
     * IMU Data Source:
     * - Same IMU as main EKF
     * - imuDataNew: Latest IMU sample
     * - delAng: Delta angles
     * - delAngDT: Time interval
     * 
     * @note Independent backup yaw estimator
     * @note Gaussian Sum Filter (multiple hypotheses)
     * @note Prediction step using IMU
     * @note Runs when magnetometer unreliable
     * 
     * @warning Computational overhead when active
     * @warning Requires GPS velocity for correction
     * @warning Not standalone (needs correction step)
     * @warning Convergence takes time (10-30 seconds)
     * 
     * @see yawEstimator for EKFGSF_yaw instance
     * @see runYawEstimatorCorrection() for correction step
     * @see EKFGSF_run_filterbank for activation flag
     * @see EKFGSF_resetMainFilterYaw() for yaw reset
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:766-768
     */
    void runYawEstimatorPrediction(void);

    /**
     * @brief Run GPS velocity correction step for GSF yaw estimator
     * 
     * @details Runs the measurement update (correction) step for the GSF yaw estimator
     *          using GPS horizontal velocity. Each hypothesis is corrected based on how
     *          well its predicted velocity matches GPS. Hypothesis weights updated.
     *          If GSF converged and reset requested, resets main EKF yaw to GSF estimate.
     * 
     * GPS Velocity Correction:
     * \n
     * Measurement Model:\n
     * - GPS provides velocity North and East\n
     * - Each GSF hypothesis predicts velocity\n
     * - Innovation: GPS_vel - predicted_vel\n
     * - Depends on yaw hypothesis\n
     * \n
     * Velocity Prediction:\n
     * - Ground speed magnitude (from GPS or airspeed)\n
     * - Direction from yaw hypothesis\n
     * - v_N = speed × cos(yaw)\n
     * - v_E = speed × sin(yaw)\n
     * \n
     * Innovation:\n
     * - For each hypothesis:\n
     *   * innov_N = GPS_vN - predicted_vN\n
     *   * innov_E = GPS_vE - predicted_vE\n
     * - Different yaw → different innovation\n
     * 
     * Kalman Update:
     * - Each hypothesis updated independently
     * - Kalman gain computed
     * - State correction applied
     * - Covariance updated
     * - Standard EKF update
     * 
     * Hypothesis Weighting:
     * \n
     * Weight Update:\n
     * - Based on innovation likelihood\n
     * - Gaussian probability of innovation\n
     * - weight_i ∝ exp(-0.5 × innov' × S^-1 × innov)\n
     * - Normalize: Σ weight_i = 1.0\n
     * \n
     * Hypothesis Selection:\n
     * - Highest weight = best hypothesis\n
     * - Weights evolve over time\n
     * - Correct yaw emerges\n
     * - Convergence indicator\n
     * 
     * Convergence Detection:
     * - One weight dominates (e.g., > 0.95)
     * - Yaw uncertainty small
     * - Sustained over time
     * - Indicates reliable yaw estimate
     * 
     * Yaw Estimate Extraction:
     * - Weighted average of hypotheses
     * - Or single best hypothesis
     * - yaw_est = Σ (weight_i × yaw_i)
     * - Uncertainty from weight distribution
     * 
     * Main EKF Yaw Reset:
     * \n
     * Reset Conditions:\n
     * - GSF converged (high confidence)\n
     * - Reset requested: EKFGSF_yaw_reset_request_ms set\n
     * - Yaw uncertainty below threshold\n
     * - Time since request < timeout (5 seconds)\n
     * \n
     * Reset Process:\n
     * - Call EKFGSF_resetMainFilterYaw()\n
     * - Extract best yaw estimate\n
     * - Reset main EKF quaternion\n
     * - Update covariance\n
     * - Record reset event\n
     * 
     * Reset Request Sources:
     * - EKFGSF_requestYawReset() called externally
     * - Magnetic anomaly detected
     * - Yaw innovation excessive
     * - Operator command
     * 
     * Airspeed Integration:
     * - If available: Constrains speed magnitude
     * - Reduces velocity ambiguity
     * - Faster convergence
     * - Better yaw observability
     * 
     * GPS Requirements:
     * - Horizontal velocity available
     * - Sufficient velocity magnitude (> 2-3 m/s)
     * - Good GPS quality
     * - Update rate adequate (5-10 Hz)
     * 
     * Low Velocity Problem:
     * - Near hover: Yaw not observable from GPS
     * - Velocity magnitude small
     * - Direction ambiguous
     * - GSF may not converge
     * - Need motion for yaw
     * 
     * Convergence Time:
     * - Typical: 10-30 seconds
     * - Depends on maneuvers
     * - Turns help convergence
     * - Straight flight harder
     * 
     * Multiple Hypotheses Benefits:
     * - Robust to initial yaw error
     * - Handles large yaw uncertainty
     * - No single bad initialization
     * - Correct hypothesis emerges
     * 
     * Integration with Main EKF:
     * - Main EKF uses magnetometer (if available)
     * - GSF provides backup
     * - Can reset main EKF if needed
     * - Seamless transition
     * 
     * Reset Decision:
     * - controlMagYawReset() orchestrates
     * - Decides when to request reset
     * - Monitors GSF convergence
     * - Triggers reset if appropriate
     * 
     * Reset Logging:
     * - EKFGSF_yaw_reset_ms: Reset time
     * - EKFGSF_yaw_reset_count: Number of resets
     * - Logged for analysis
     * - Monitor reset frequency
     * 
     * @note Correction step using GPS velocity
     * @note Updates hypothesis weights
     * @note Can trigger main EKF yaw reset
     * @note Requires vehicle motion for convergence
     * 
     * @warning Requires sufficient GPS velocity
     * @warning Low speed reduces yaw observability
     * @warning Convergence takes 10-30 seconds
     * @warning Monitor GSF convergence status
     * 
     * @see runYawEstimatorPrediction() for prediction step
     * @see EKFGSF_resetMainFilterYaw() for yaw reset
     * @see EKFGSF_requestYawReset() for reset request
     * @see yawEstimator for EKFGSF_yaw instance
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:770-772
     */
    void runYawEstimatorCorrection(void);

    /**
     * @brief Reset quaternion states using supplied yaw angle
     * 
     * @details Resets the vehicle yaw component of the quaternion state while preserving
     *          roll and pitch. Updates body-to-nav rotation matrix and resets yaw-related
     *          covariances. Used for yaw resets from various sources (GPS, GSF, mag, etc.).
     * 
     * @param[in] yaw New yaw angle in radians (or delta yaw if isDeltaYaw = true)
     * @param[in] yawVariance Variance (uncertainty squared) of new yaw angle (rad²)
     * @param[in] isDeltaYaw If true, yaw is added to existing; if false, yaw is absolute
     * 
     * Quaternion Yaw Reset:
     * \n
     * Preserve Roll and Pitch:\n
     * - Extract current roll and pitch from quaternion\n
     * - Keep these components unchanged\n
     * - Only modify yaw component\n
     * - Reconstruct quaternion with new yaw\n
     * \n
     * Reset Modes:\n
     * - Absolute (isDeltaYaw = false): Set yaw to specific value\n
     * - Delta (isDeltaYaw = true): Add yaw offset to current\n
     * - Different use cases for each\n
     * 
     * Absolute Yaw Reset:
     * - New yaw is target value
     * - Example: Align to GPS course (90°)
     * - Example: Set yaw from compass (45°)
     * - Direct replacement
     * 
     * Delta Yaw Reset:
     * - New yaw is correction amount
     * - Example: +10° correction
     * - yaw_new = yaw_old + delta
     * - Incremental adjustment
     * 
     * Quaternion Reconstruction:
     * \n
     * 1. Extract Euler Angles:\n
     *    - Convert quaternion to Euler (roll, pitch, yaw)\n
     *    - Keep roll and pitch\n
     * \n
     * 2. Apply Yaw:\n
     *    - If absolute: yaw_new = yaw (input)\n
     *    - If delta: yaw_new = yaw_old + yaw (input)\n
     *    - Wrap to ±π\n
     * \n
     * 3. Quaternion from Euler:\n
     *    - quat = Euler_to_Quat(roll, pitch, yaw_new)\n
     *    - Update stateStruct.quat\n
     * \n
     * 4. Update Output States:\n
     *    - outputDataNew.quat = stateStruct.quat\n
     *    - outputDataDelayed.quat updated\n
     *    - StoreQuatReset() updates history\n
     * 
     * Rotation Matrix Update:
     * - Body-to-NED: Tbn matrix
     * - Recompute from new quaternion
     * - quat_to_rotation_matrix(quat, Tbn)
     * - Used in all transformations
     * 
     * Covariance Reset:
     * \n
     * Yaw Variance:\n
     * - P[2][2] = yawVariance\n
     * - Sets yaw uncertainty\n
     * - Large variance: Uncertain reset\n
     * - Small variance: Confident reset\n
     * \n
     * Zero Yaw Covariances:\n
     * - zeroAttCovOnly() called\n
     * - Yaw-other correlations invalid\n
     * - P[2, 3:23] = 0\n
     * - P[3:23, 2] = 0\n
     * - Preserve P[2][2] (variance)\n
     * 
     * Yaw Variance Examples:
     * - GPS course: 0.05 rad² ≈ 13° std
     * - Compass: 0.02 rad² ≈ 8° std  
     * - EKFGSF: 0.01 rad² ≈ 6° std
     * - Depends on source quality
     * 
     * Reset Event Recording:
     * - recordYawReset() called
     * - yawResetAngle = yaw_new - yaw_old
     * - lastYawReset_ms = current time
     * - For external notification
     * 
     * Output Buffer Management:
     * - StoreQuatReset() updates output buffer
     * - All stored quaternions updated
     * - Maintains output consistency
     * - Prevents output jumps
     * 
     * Use Cases:
     * \n
     * 1. GPS Yaw Alignment:\n
     *    - realignYawGPS() calls this\n
     *    - Align to GPS ground track\n
     *    - Absolute yaw\n
     * \n
     * 2. EKFGSF Yaw Reset:\n
     *    - EKFGSF_resetMainFilterYaw() calls this\n
     *    - Emergency yaw from GSF\n
     *    - Absolute yaw\n
     * \n
     * 3. Magnetic Anomaly Recovery:\n
     *    - controlMagYawReset() triggers\n
     *    - Reset to last known good\n
     *    - Or GPS course\n
     * \n
     * 4. External Nav Yaw:\n
     *    - Direct yaw observation\n
     *    - High precision\n
     *    - Absolute yaw\n
     * 
     * Roll and Pitch Preservation:
     * - Critical for flight control
     * - Attitude control depends on accurate roll/pitch
     * - Only yaw modified
     * - Smooth transition
     * 
     * Impact on Navigation:
     * - Heading changes immediately
     * - Position estimate affected
     * - Wind estimate affected
     * - Mag field states may be affected
     * 
     * Controller Notification:
     * - getLastYawResetAngle() retrieves reset
     * - Attitude controller compensates
     * - Rate feed-forward adjustment
     * - Smoother response
     * 
     * @note Preserves roll and pitch, only resets yaw
     * @note Supports absolute and delta modes
     * @note Updates quaternion and covariances
     * @note Records reset for external notification
     * 
     * @warning Large yaw resets disturb control
     * @warning Covariances must be reset properly
     * @warning Output buffer must be updated
     * @warning Controllers should be notified
     * 
     * @see recordYawReset() for reset event logging
     * @see zeroAttCovOnly() for covariance reset
     * @see StoreQuatReset() for output buffer update
     * @see getLastYawResetAngle() for reset retrieval
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:774-780
     */
    void resetQuatStateYawOnly(ftype yaw, ftype yawVariance, bool isDeltaYaw);

    /**
     * @brief Attempt to reset main EKF yaw to EKF-GSF estimate
     * 
     * @details Tries to reset the main EKF's yaw angle to the Gaussian Sum Filter
     *          yaw estimate. Checks GSF convergence and validity before resetting.
     *          Provides emergency yaw recovery when magnetometer fusion fails.
     * 
     * @return true if yaw reset successful, false if GSF not ready or reset failed
     * 
     * EKFGSF Yaw Reset Purpose:
     * - Emergency yaw recovery mechanism
     * - When magnetometer fusion fails
     * - Magnetic anomalies detected
     * - Yaw divergence correction
     * - Backup to compass
     * 
     * Reset Conditions (all must be met):
     * \n
     * 1. GSF Filter Running:\n
     *    - EKFGSF_run_filterbank = true\n
     *    - GSF activated due to mag problems\n
     *    - Filter bank operational\n
     * \n
     * 2. GSF Converged:\n
     *    - Yaw estimate confidence high\n
     *    - Hypothesis weights converged\n
     *    - Yaw uncertainty below threshold\n
     *    - Typically after 10-30 seconds\n
     * \n
     * 3. Reset Requested:\n
     *    - EKFGSF_yaw_reset_request_ms set\n
     *    - Request not timed out\n
     *    - Within YAW_RESET_TO_GSF_TIMEOUT_MS (5 sec)\n
     * \n
     * 4. GPS Available:\n
     *    - GPS velocity used by GSF\n
     *    - GPS quality acceptable\n
     *    - Sufficient vehicle motion\n
     * 
     * GSF Convergence Check:
     * - Query yawEstimator->getYawData()
     * - Returns yaw estimate and variance
     * - Check variance < threshold
     * - Check weight distribution
     * - Ensure stable estimate
     * 
     * Reset Process:
     * \n
     * 1. Get GSF Yaw Estimate:\n
     *    - yaw_est: Best yaw from GSF (rad)\n
     *    - yaw_var: Yaw variance (rad²)\n
     *    - From converged GSF\n
     * \n
     * 2. Validate Estimate:\n
     *    - Check yaw in valid range [-π, π]\n
     *    - Check variance reasonable (< 0.1 rad²)\n
     *    - Ensure not NaN or invalid\n
     * \n
     * 3. Reset Main EKF:\n
     *    - Call resetQuatStateYawOnly()\n
     *    - yaw = yaw_est (absolute)\n
     *    - yawVariance = yaw_var\n
     *    - isDeltaYaw = false\n
     * \n
     * 4. Record Reset:\n
     *    - EKFGSF_yaw_reset_ms = current time\n
     *    - EKFGSF_yaw_reset_count++\n
     *    - Log reset event\n
     * \n
     * 5. Clear Request:\n
     *    - EKFGSF_yaw_reset_request_ms = 0\n
     *    - Reset completed\n
     * 
     * Failure Conditions:
     * - GSF not converged: Return false
     * - GPS not available: Return false
     * - Yaw variance too large: Return false
     * - Invalid yaw estimate: Return false
     * - Timeout expired: Return false
     * 
     * Reset Count Tracking:
     * - EKFGSF_yaw_reset_count incremented
     * - Monitor number of resets
     * - Excessive resets indicate problems
     * - Logged for analysis
     * 
     * Timeout Mechanism:
     * - YAW_RESET_TO_GSF_TIMEOUT_MS: 5 seconds
     * - Request expires if not acted on
     * - Prevents stale reset requests
     * - Must re-request if timeout
     * 
     * Impact on Main EKF:
     * - Yaw state changes immediately
     * - Quaternion updated
     * - Covariances reset
     * - May affect position, velocity estimates
     * - Wind estimate may change
     * 
     * Use Cases:
     * \n
     * 1. Magnetic Anomaly:\n
     *    - Detect large mag disturbance\n
     *    - Activate GSF\n
     *    - Wait for convergence\n
     *    - Reset yaw from GSF\n
     * \n
     * 2. Compass Failure:\n
     *    - Magnetometer timeout\n
     *    - Switch to GSF yaw\n
     *    - Emergency backup\n
     * \n
     * 3. Initialization Issues:\n
     *    - Poor initial yaw\n
     *    - Compass cal problems\n
     *    - GSF provides correction\n
     * 
     * Triggering Reset:
     * - Call EKFGSF_requestYawReset()
     * - Sets request timestamp
     * - controlMagYawReset() monitors
     * - This function executes reset
     * 
     * Reset Frequency Limits:
     * - Should be rare event
     * - Frequent resets indicate problems
     * - Monitor EKFGSF_yaw_reset_count
     * - Investigate if > 2-3 per flight
     * 
     * GPS Requirements:
     * - Horizontal velocity for GSF
     * - Sufficient motion (> 2-3 m/s)
     * - Good GPS quality
     * - GSF needs GPS to converge
     * 
     * Convergence Time:
     * - Typical: 10-30 seconds
     * - Depends on maneuvers
     * - Turns accelerate convergence
     * - Straight flight slower
     * 
     * @note Emergency yaw recovery mechanism
     * @note Checks GSF convergence before reset
     * @note Returns false if not ready
     * @note Limits reset frequency
     * 
     * @warning Requires GSF convergence (10-30 sec)
     * @warning Needs GPS velocity and motion
     * @warning Disturbs flight control temporarily
     * @warning Monitor reset frequency
     * 
     * @see EKFGSF_requestYawReset() for reset request
     * @see resetQuatStateYawOnly() for yaw reset
     * @see runYawEstimatorCorrection() for GSF update
     * @see EKFGSF_yaw_reset_count for reset counter
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:782-784
     */
    bool EKFGSF_resetMainFilterYaw();

    // Length of FIFO buffers used for non-IMU sensor data.
    // Must be larger than the time period defined by IMU_BUFFER_LENGTH
    static const uint32_t OBS_BUFFER_LENGTH = 5;
    static const uint32_t FLOW_BUFFER_LENGTH = 15;
    static const uint32_t EXTNAV_BUFFER_LENGTH = 15;

    // Variables
    bool statesInitialised;         // boolean true when filter states have been initialised
    bool magHealth;                 // boolean true if magnetometer has passed innovation consistency check
    bool velTimeout;                // boolean true if velocity measurements have failed innovation consistency check and timed out
    bool posTimeout;                // boolean true if position measurements have failed innovation consistency check and timed out
    bool hgtTimeout;                // boolean true if height measurements have failed innovation consistency check and timed out
    bool magTimeout;                // boolean true if magnetometer measurements have failed for too long and have timed out
    bool tasTimeout;                // boolean true if true airspeed measurements have failed for too long and have timed out
    bool badIMUdata;                // boolean true if the bad IMU data is detected

    ftype gpsNoiseScaler;           // Used to scale the  GPS measurement noise and consistency gates to compensate for operation with small satellite counts
    
    /**
     * @brief State estimation error covariance matrix
     * 
     * @details The P matrix is the heart of the Kalman filter, representing the uncertainty
     *          in the estimated state vector. This 24x24 symmetric positive definite matrix
     *          is continuously updated through prediction and measurement update cycles.
     * 
     * Matrix Properties:
     * - Type: Matrix24 (24x24 covariance matrix)
     * - Symmetry: Maintained symmetric (P[i][j] == P[j][i]) via ForceSymmetry()
     * - Positive Definite: All eigenvalues must be positive for filter stability
     * - Units: Diagonal elements have squared state units, off-diagonals are covariances
     * - Size: 2.3 KB (float) or 4.6 KB (double precision)
     * 
     * State Covariances (diagonal elements):
     * - P[0][0] to P[2][2]: Rotation error variance (rad²)
     * - P[3][3] to P[5][5]: Velocity NED variance (m²/s²)
     * - P[6][6] to P[8][8]: Position NED variance (m²)
     * - P[9][9] to P[11][11]: Gyro bias variance (rad²/s²)
     * - P[12][12] to P[14][14]: Gyro scale factor variance (dimensionless²)
     * - P[15][15]: Accel Z bias variance (m²/s⁴)
     * - P[16][16] to P[18][18]: Earth magnetic field NED variance (gauss²/10⁶)
     * - P[19][19] to P[21][21]: Body magnetic field XYZ variance (gauss²/10⁶)
     * - P[22][22] to P[23][23]: Wind velocity NE variance (m²/s²)
     * 
     * Update Cycle:
     * 1. Prediction: CovariancePrediction() propagates P forward using process noise
     * 2. Measurement Update: Fusion functions update P using Kalman gain
     * 3. Symmetry: ForceSymmetry() enforces P[i][j] = P[j][i]
     * 4. Constraints: ConstrainVariances() limits diagonal elements
     * 
     * Numerical Considerations:
     * - Conditioning: Poor conditioning leads to innovation variance calculation errors
     * - Symmetry: Enforced after every prediction to prevent numerical asymmetry
     * - Variance Limits: Diagonal elements constrained to prevent divergence
     * - Positive Definiteness: Monitored through innovation consistency checks
     * 
     * Innovation Variance Calculation:
     * - Measurement innovation variance: S = H * P * H' + R
     * - Used for Kalman gain: K = P * H' * inv(S)
     * - Used for consistency checks: innovation² / S < threshold
     * 
     * Common Operations:
     * - Read variance: variance = P[i][i]
     * - Read covariance: covariance = P[i][j]
     * - Check uncertainty: if (P[i][i] > limit) { ... }
     * 
     * @warning Numerical conditioning of P is critical for filter stability
     * @warning Large diagonal elements indicate filter divergence
     * @warning Asymmetry in P indicates numerical errors requiring investigation
     * 
     * @note Initialized in CovarianceInit() with diagonal variances based on sensor accuracy
     * @note Continuously updated at EKF_TARGET_DT = 0.01s (100Hz) rate
     * @note Matrix operations are computationally expensive (O(n³) for some updates)
     * 
     * @see Matrix24 for typedef definition and memory layout
     * @see CovariancePrediction() for time update equations
     * @see ForceSymmetry() for symmetry enforcement
     * @see ConstrainVariances() for variance limiting
     * @see CopyCovariances() for numerical error correction
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:802
     */
    Matrix24 P;                     // covariance matrix
    EKF_IMU_buffer_t<imu_elements> storedIMU;      // IMU data buffer
    EKF_obs_buffer_t<gps_elements> storedGPS;      // GPS data buffer
    EKF_obs_buffer_t<mag_elements> storedMag;      // Magnetometer data buffer
    EKF_obs_buffer_t<baro_elements> storedBaro;    // Baro data buffer
    EKF_obs_buffer_t<tas_elements> storedTAS;      // TAS data buffer
    EKF_obs_buffer_t<range_elements> storedRange;  // Range finder data buffer
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
    uint32_t airborneDetectTime_ms; // last time flight movement was detected
    Vector6 innovVelPos;            // innovation output for a group of measurements
    Vector6 varInnovVelPos;         // innovation variance output for a group of measurements
    Vector6 velPosObs;              // observations for combined velocity and positon group of measurements (3x1 m , 3x1 m/s)
    bool fuseVelData;               // this boolean causes the velNED measurements to be fused
    bool fusePosData;               // this boolean causes the posNE measurements to be fused
    bool fuseHgtData;               // this boolean causes the hgtMea measurements to be fused
    Vector3F innovMag;              // innovation output from fusion of X,Y,Z compass measurements
    Vector3F varInnovMag;           // innovation variance output from fusion of X,Y,Z compass measurements
    ftype innovVtas;                // innovation output from fusion of airspeed measurements
    ftype varInnovVtas;             // innovation variance output from fusion of airspeed measurements
    bool magFusePerformed;          // boolean set to true when magnetometer fusion has been performed in that time step
    uint32_t prevTasStep_ms;        // time stamp of last TAS fusion step
    uint32_t prevBetaStep_ms;       // time stamp of last synthetic sideslip fusion step
    uint32_t lastMagUpdate_us;      // last time compass was updated in usec
    uint32_t lastMagRead_ms;        // last time compass data was successfully read
    Vector3F velDotNED;             // rate of change of velocity in NED frame
    Vector3F velDotNEDfilt;         // low pass filtered velDotNED
    uint32_t imuSampleTime_ms;      // time that the last IMU value was taken
    bool tasDataToFuse;             // true when new airspeed data is waiting to be fused
    uint32_t lastBaroReceived_ms;   // time last time we received baro height data
    uint16_t hgtRetryTime_ms;       // time allowed without use of height measurements before a height timeout is declared
    uint32_t lastVelPassTime_ms;    // time stamp when GPS velocity measurement last passed innovation consistency check (msec)
    uint32_t lastPosPassTime_ms;    // time stamp when GPS position measurement last passed innovation consistency check (msec)
    uint32_t lastHgtPassTime_ms;    // time stamp when height measurement last passed innovation consistency check (msec)
    uint32_t lastTasPassTime_ms;    // time stamp when airspeed measurement last passed innovation consistency check (msec)
    uint32_t lastTasFailTime_ms;    // time stamp when airspeed measurement last failed innovation consistency check (msec)
    uint32_t lastTimeGpsReceived_ms;// last time we received GPS data
    uint32_t timeAtLastAuxEKF_ms;   // last time the auxiliary filter was run to fuse range or optical flow measurements
    uint32_t lastHealthyMagTime_ms; // time the magnetometer was last declared healthy
    bool allMagSensorsFailed;       // true if all magnetometer sensors have timed out on this flight and we are no longer using magnetometer data
    uint32_t lastYawTime_ms;        // time stamp when yaw observation was last fused (msec)
    uint32_t ekfStartTime_ms;       // time the EKF was started (msec)
    Vector2F lastKnownPositionNE;   // last known position
    ftype velTestRatio;             // sum of squares of GPS velocity innovation divided by fail threshold
    ftype posTestRatio;             // sum of squares of GPS position innovation divided by fail threshold
    ftype hgtTestRatio;             // sum of squares of baro height innovation divided by fail threshold
    Vector3F magTestRatio;          // sum of squares of magnetometer innovations divided by fail threshold
    ftype tasTestRatio;             // sum of squares of true airspeed innovation divided by fail threshold
    ftype defaultAirSpeed;          // default equivalent airspeed in m/s to be used if the measurement is unavailable. Do not use if not positive.
    bool inhibitWindStates;         // true when wind states and covariances are to remain constant
    bool inhibitMagStates;          // true when magnetic field states and covariances are to remain constant
    bool lastInhibitMagStates;      // previous inhibitMagStates
    bool needMagBodyVarReset;       // we need to reset mag body variances at next CovariancePrediction
    bool gpsNotAvailable;           // bool true when valid GPS data is not available
    uint8_t last_gps_idx;           // sensor ID of the GPS receiver used for the last fusion or reset
    Location EKF_origin;     // LLH origin of the NED axis system
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
    Vector3F tiltErrVec;            // Vector of most recent attitude error correction from Vel,Pos fusion
    ftype tiltErrFilt;              // Filtered tilt error metric
    bool tiltAlignComplete;         // true when tilt alignment is complete
    bool yawAlignComplete;          // true when yaw alignment is complete
    bool magStateInitComplete;      // true when the magnetic field sttes have been initialised
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
    output_elements outputDataNew;  // output state data at the current time step
    output_elements outputDataDelayed; // output state data at the current time step
    Vector3F delAngCorrection;      // correction applied to delta angles used by output observer to track the EKF
    Vector3F velErrintegral;        // integral of output predictor NED velocity tracking error (m)
    Vector3F posErrintegral;        // integral of output predictor NED position tracking error (m.sec)
    ftype innovYaw;                 // compass yaw angle innovation (rad)
    uint32_t timeTasReceived_ms;    // time last TAS data was received (msec)
    bool gpsGoodToAlign;            // true when the GPS quality can be used to initialise the navigation system
    uint32_t magYawResetTimer_ms;   // timer in msec used to track how long good magnetometer data is failing innovation consistency checks
    bool consistentMagData;         // true when the magnetometers are passing consistency checks
    bool motorsArmed;               // true when the motors have been armed
    bool prevMotorsArmed;           // value of motorsArmed from previous frame
    bool posVelFusionDelayed;       // true when the position and velocity fusion has been delayed
    bool optFlowFusionDelayed;      // true when the optical flow fusion has been delayed
    bool airSpdFusionDelayed;       // true when the air speed fusion has been delayed
    bool sideSlipFusionDelayed;     // true when the sideslip fusion has been delayed
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
    bool startPredictEnabled;       // boolean true when the frontend has given permission to start a new state prediciton cycele
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

    // variables used to calculate a vertical velocity that is kinematically consistent with the verical position
    struct {
        ftype pos;
        ftype vel;
        ftype acc;
    } vertCompFiltState;

    // variables used by the pre-initialisation GPS checks
    Location gpsloc_prev;    // LLH location of previous GPS measurement
    uint32_t lastPreAlignGpsCheckTime_ms;   // last time in msec the GPS quality was checked during pre alignment checks
    ftype gpsDriftNE;               // amount of drift detected in the GPS position during pre-flight GPs checks
    ftype gpsVertVelFilt;           // amount of filterred vertical GPS velocity detected durng pre-flight GPS checks
    ftype gpsHorizVelFilt;          // amount of filtered horizontal GPS velocity detected during pre-flight GPS checks

    // variable used by the in-flight GPS quality check
    bool gpsSpdAccPass;             // true when reported GPS speed accuracy passes in-flight checks
    bool ekfInnovationsPass;        // true when GPS innovations pass in-flight checks
    ftype sAccFilterState1;         // state variable for LPF applid to reported GPS speed accuracy
    ftype sAccFilterState2;         // state variable for peak hold filter applied to reported GPS speed
    uint32_t lastGpsCheckTime_ms;   // last time in msec the GPS quality was checked
    uint32_t lastInnovPassTime_ms;  // last time in msec the GPS innovations passed
    uint32_t lastInnovFailTime_ms;  // last time in msec the GPS innovations failed
    bool gpsAccuracyGood;           // true when the GPS accuracy is considered to be good enough for safe flight.

    // variables added for optical flow fusion
    EKF_obs_buffer_t<of_elements> storedOF;    // OF data buffer
    of_elements ofDataNew;          // OF data at the current time horizon
    of_elements ofDataDelayed;      // OF data at the fusion time horizon
    bool flowDataToFuse;            // true when optical flow data is ready for fusion
    bool flowDataValid;             // true while optical flow data is still fresh
    Vector2F auxFlowObsInnov;       // optical flow rate innovation from 1-state terrain offset estimator
    uint32_t flowValidMeaTime_ms;   // time stamp from latest valid flow measurement (msec)
    uint32_t rngValidMeaTime_ms;    // time stamp from latest valid range measurement (msec)
    uint32_t flowMeaTime_ms;        // time stamp from latest flow measurement (msec)
    uint32_t gndHgtValidTime_ms;    // time stamp from last terrain offset state update (msec)
    Matrix3F Tbn_flow;              // transformation matrix from body to nav axes at the middle of the optical flow sample period
    Vector2 varInnovOptFlow;        // optical flow innovations variances (rad/sec)^2
    Vector2 innovOptFlow;           // optical flow LOS innovations (rad/sec)
    ftype Popt;                     // Optical flow terrain height state covariance (m^2)
    ftype terrainState;             // terrain position state (m)
    ftype prevPosN;                 // north position at last measurement
    ftype prevPosE;                 // east position at last measurement
    ftype varInnovRng;              // range finder observation innovation variance (m^2)
    ftype innovRng;                 // range finder observation innovation (m)
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
    Vector3F accelPosOffset;        // position of IMU accelerometer unit in body frame (m)


    // Range finder
    ftype baroHgtOffset;                    // offset applied when when switching to use of Baro height
    ftype rngOnGnd;                         // Expected range finder reading in metres when vehicle is on ground
    uint32_t lastRngMeasTime_ms;            // Timestamp of last range measurement
    bool terrainHgtStable;                  // true when the terrain height is stable enough to be used as a height reference
#if AP_RANGEFINDER_ENABLED
    ftype storedRngMeas[DOWNWARD_RANGEFINDER_MAX_INSTANCES][3];              // Ringbuffer of stored range measurements for dual range sensors
    uint32_t storedRngMeasTime_ms[DOWNWARD_RANGEFINDER_MAX_INSTANCES][3];    // Ringbuffers of stored range measurement times for dual range sensors
    uint8_t rngMeasIndex[DOWNWARD_RANGEFINDER_MAX_INSTANCES];                // Current range measurement ringbuffer index for dual range sensors
#endif

    // Range Beacon Sensor Fusion
    EKF_obs_buffer_t<rng_bcn_elements> storedRangeBeacon; // Beacon range buffer
    rng_bcn_elements rngBcnDataNew;     // Range beacon data at the current time horizon
    rng_bcn_elements rngBcnDataDelayed; // Range beacon data at the fusion time horizon
    uint32_t lastRngBcnPassTime_ms;     // time stamp when the range beacon measurement last passed innvovation consistency checks (msec)
    ftype rngBcnTestRatio;              // Innovation test ratio for range beacon measurements
    bool rngBcnHealth;                  // boolean true if range beacon measurements have passed innovation consistency check
    bool rngBcnTimeout;                 // boolean true if range beacon measurements have faled innovation consistency checks for too long
    ftype varInnovRngBcn;               // range beacon observation innovation variance (m^2)
    ftype innovRngBcn;                  // range beacon observation innovation (m)
    uint32_t lastTimeRngBcn_ms[10];     // last time we received a range beacon measurement (msec)
#if AP_BEACON_ENABLED
    bool rngBcnDataToFuse;              // true when there is new range beacon data to fuse
#else
    const bool rngBcnDataToFuse = false;              // true when there is new range beacon data to fuse
#endif
    Vector3F beaconVehiclePosNED;       // NED position estimate from the beacon system (NED)
    ftype beaconVehiclePosErr;          // estimated position error from the beacon system (m)
    uint32_t rngBcnLast3DmeasTime_ms;   // last time the beacon system returned a 3D fix (msec)
    bool rngBcnGoodToAlign;             // true when the range beacon systems 3D fix can be used to align the filter
    uint8_t lastRngBcnChecked;          // index of the last range beacon checked for data
    Vector3F receiverPos;               // receiver NED position (m) - alignment 3 state filter
    ftype receiverPosCov[3][3];         // Receiver position covariance (m^2) - alignment 3 state filter (
    bool rngBcnAlignmentStarted;        // True when the initial position alignment using range measurements has started
    bool rngBcnAlignmentCompleted;      // True when the initial position alignment using range measurements has finished
    uint8_t lastBeaconIndex;            // Range beacon index last read -  used during initialisation of the 3-state filter
    Vector3F rngBcnPosSum;              // Sum of range beacon NED position (m) - used during initialisation of the 3-state filter
    uint8_t numBcnMeas;                 // Number of beacon measurements - used during initialisation of the 3-state filter
    ftype rngSum;                       // Sum of range measurements (m) - used during initialisation of the 3-state filter
    uint8_t N_beacons;                  // Number of range beacons in use
    ftype maxBcnPosD;                   // maximum position of all beacons in the down direction (m)
    ftype minBcnPosD;                   // minimum position of all beacons in the down direction (m)
    ftype bcnPosOffset;                 // Vertical position offset of the beacon constellation origin relative to the EKF origin (m)

    ftype bcnPosOffsetMax;             // Vertical position offset of the beacon constellation origin relative to the EKF origin (m)
    ftype bcnPosOffsetMaxVar;          // Variance of the bcnPosOffsetHigh state (m)
    ftype OffsetMaxInnovFilt;          // Filtered magnitude of the range innovations using bcnPosOffsetHigh

    ftype bcnPosOffsetMin;              // Vertical position offset of the beacon constellation origin relative to the EKF origin (m)
    ftype bcnPosOffsetMinVar;           // Variance of the bcnPosoffset state (m)
    ftype OffsetMinInnovFilt;           // Filtered magnitude of the range innovations using bcnPosOffsetLow

    // Range Beacon Fusion Debug Reporting
    uint8_t rngBcnFuseDataReportIndex;// index of range beacon fusion data last reported
    struct rngBcnFusionReport_t {
        ftype rng;          // measured range to beacon (m)
        ftype innov;        // range innovation (m)
        ftype innovVar;     // innovation variance (m^2)
        ftype testRatio;    // innovation consistency test ratio
        Vector3F beaconPosNED; // beacon NED position
    } rngBcnFusionReport[10];

    // height source selection logic
    uint8_t activeHgtSource;    // integer defining active height source

    // Movement detector
    bool takeOffDetected;           // true when takeoff for optical flow navigation has been detected
    ftype rngAtStartOfFlight;       // range finder measurement at start of flight
    uint32_t timeAtArming_ms;       // time in msec that the vehicle armed

    // baro ground effect
    ftype meaHgtAtTakeOff;            // height measured at commencement of takeoff

    // control of post takeoff magnetic field and heading resets
    bool finalInflightYawInit;      // true when the final post takeoff initialisation of yaw angle has been performed
    bool finalInflightMagInit;      // true when the final post takeoff initialisation of magnetic field states been performed
    bool magStateResetRequest;      // true if magnetic field states need to be reset using the magneteomter measurements
    bool magYawResetRequest;        // true if the vehicle yaw and magnetic field states need to be reset using the magnetometer measurements
    bool gpsYawResetRequest;        // true if the vehicle yaw needs to be reset to the GPS course
    ftype posDownAtLastMagReset;    // vertical position last time the mag states were reset (m)
    ftype yawInnovAtLastMagReset;   // magnetic yaw innovation last time the yaw and mag field states were reset (rad)
    QuaternionF quatAtLastMagReset;  // quaternion states last time the mag states were reset
    uint8_t magYawAnomallyCount;    // Number of times the yaw has been reset due to a magnetic anomaly during initial ascent

    // external navigation fusion
    EKF_obs_buffer_t<ext_nav_elements> storedExtNav; // external navigation data buffer
    ext_nav_elements extNavDataNew;     // External nav data at the current time horizon
    ext_nav_elements extNavDataDelayed; // External nav at the fusion time horizon
    uint32_t extNavMeasTime_ms;         // time external measurements were accepted for input to the data buffer (msec)
    uint32_t extNavLastPosResetTime_ms; // last time the external nav systen performed a position reset (msec)
    uint32_t lastExtNavPassTime_ms;     // time stamp when external nav position measurement last passed innovation consistency check (msec)
    bool extNavDataToFuse;              // true when there is new external nav data to fuse
    bool extNavUsedForYaw;              // true when the external nav data is also being used as a yaw observation
    bool extNavUsedForPos;              // true when the external nav data is being used as a position reference.
    bool extNavYawResetRequest;         // true when a reset of vehicle yaw using the external nav data is requested

    EKF_obs_buffer_t<ext_nav_vel_elements> storedExtNavVel; // external navigation velocity data buffer
    ext_nav_vel_elements extNavVelNew;                       // external navigation velocity data at the current time horizon
    ext_nav_vel_elements extNavVelDelayed;                   // external navigation velocity data at the fusion time horizon
    uint32_t extNavVelMeasTime_ms;                           // time external navigation velocity measurements were accepted for input to the data buffer (msec)
    bool extNavVelToFuse;                                    // true when there is new external navigation velocity to fuse
    bool useExtNavVel;                                       // true external navigation velocity should be used

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
    } faultStatus;

    // flags indicating which GPS quality checks are failing
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
    } gpsCheckStatus;

    // states held by magnetomter fusion across time steps
    // magnetometer X,Y,Z measurements are fused across three time steps
    // to level computational load as this is an expensive operation
    struct {
        ftype q0;
        ftype q1;
        ftype q2;
        ftype q3;
        ftype magN;
        ftype magE;
        ftype magD;
        ftype magXbias;
        ftype magYbias;
        ftype magZbias;
        Matrix3F DCM;
        Vector3F MagPred;
        ftype R_MAG;
        Vector9 SH_MAG;
    } mag_state;

    // string representing last reason for prearm failure
    char prearm_fail_string[41];

    // earth field from WMM tables
    bool have_table_earth_field;   // true when we have initialised table_earth_field_ga
    Vector3F table_earth_field_ga; // earth field from WMM tables
    ftype table_declination;       // declination in radians from the tables

    // timing statistics
    struct ekf_timing timing;

    // when was attitude filter status last non-zero?
    uint32_t last_filter_ok_ms;
    
    /**
     * @brief Determine if zero sideslip assumption should be used
     * 
     * @details Checks vehicle configuration to determine if zero sideslip constraint
     *          should be applied. Used for synthetic sideslip fusion (beta fusion).
     *          Different vehicle types have different aerodynamic characteristics.
     * 
     * @return true if zero sideslip should be assumed, false otherwise
     * 
     * Zero Sideslip Assumption:
     * - Sideslip angle (beta): Angle between velocity and body X axis
     * - Zero sideslip: Vehicle points in direction of travel
     * - Constraint: Lateral velocity = 0 in body frame
     * - Aerodynamic assumption
     * 
     * Vehicle-Specific Behavior:
     * \n
     * Fixed-Wing Aircraft:\n
     * - Assume zero sideslip: true\n
     * - Coordinated flight assumption\n
     * - Aerodynamic constraint\n
     * - Helps constrain yaw\n
     * \n
     * Multicopters:\n
     * - Assume zero sideslip: false\n
     * - Can move laterally\n
     * - No aerodynamic constraint\n
     * - Not applicable\n
     * \n
     * Rovers:\n
     * - Depends on steering type\n
     * - Ackermann: May assume zero\n
     * - Skid-steer: No assumption\n
     * \n
     * Underwater:\n
     * - Assume zero sideslip: false\n
     * - Can translate laterally\n
     * - Different dynamics\n
     * 
     * Implementation:
     * - Queries vehicle type from frontend
     * - Checks configuration parameters
     * - Returns vehicle-specific policy
     * - Used by FuseSideslip()
     * 
     * Sideslip Fusion (Beta Fusion):
     * - Synthetic measurement: beta = 0
     * - Lateral velocity = 0 (body frame)
     * - Measurement update step
     * - Constrains yaw drift
     * 
     * Benefits When Applied:
     * - Improves yaw observability
     * - Reduces yaw drift
     * - Better heading accuracy
     * - Especially without magnetometer
     * 
     * When NOT to Apply:
     * - Vehicle can move laterally
     * - Uncoordinated flight
     * - Aggressive maneuvers
     * - Sideslip intentional
     * 
     * Use Cases:
     * - Fixed-wing coordinated flight
     * - Airspeed-based navigation
     * - Magnetometer degraded
     * - Yaw drift mitigation
     * 
     * @note Vehicle-specific aerodynamic assumption
     * @note Enables synthetic sideslip fusion
     * @note Fixed-wing: typically true
     * @note Multicopter: typically false
     * 
     * @see FuseSideslip() for sideslip fusion
     * @see SelectBetaFusion() for fusion scheduling
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:1191
     */
    bool assume_zero_sideslip(void) const;

    /**
     * @brief Get vehicle-specific initial gyro bias uncertainty
     * 
     * @details Returns the initial gyro bias covariance based on vehicle type.
     *          Different vehicles have different gyro bias characteristics and
     *          initialization requirements. Sets P[9][9], P[10][10], P[11][11].
     * 
     * @return Initial gyro bias 1-sigma uncertainty in rad/s
     * 
     * Gyro Bias Uncertainty:
     * - Initial uncertainty: Variance in P matrix
     * - P[9][9], P[10][10], P[11][11]: Gyro bias variances
     * - Units: (rad/s)²
     * - Return value: Square root (1-sigma, rad/s)
     * 
     * Vehicle-Specific Values:
     * \n
     * Fixed-Wing:\n
     * - Larger initial uncertainty\n
     * - Longer warm-up time\n
     * - Higher bias variability\n
     * - Typical: 0.02 rad/s (1-sigma)\n
     * \n
     * Multicopter:\n
     * - Smaller initial uncertainty\n
     * - Faster initialization\n
     * - More stable gyros (higher rate)\n
     * - Typical: 0.01 rad/s (1-sigma)\n
     * \n
     * Rover:\n
     * - Moderate uncertainty\n
     * - Similar to fixed-wing\n
     * - Depends on quality\n
     * \n
     * Underwater:\n
     * - Similar to multicopter\n
     * - Stable environment\n
     * 
     * Initialization Impact:
     * - Larger uncertainty: Slower convergence
     * - Smaller uncertainty: Faster, but risk of divergence
     * - Must match actual bias variability
     * - Trade-off: Speed vs robustness
     * 
     * Gyro Bias Learning:
     * - EKF estimates bias over time
     * - Bias states: stateStruct.gyro_bias (9-11)
     * - Converges as data accumulates
     * - Initial uncertainty sets starting point
     * 
     * Covariance Initialization:
     * - CovarianceInit() uses this value
     * - P[9][9] = uncertainty²
     * - P[10][10] = uncertainty²
     * - P[11][11] = uncertainty²
     * 
     * Convergence Time:
     * - Higher uncertainty → slower
     * - Lower uncertainty → faster (if correct)
     * - Typical convergence: 30-60 seconds
     * - Depends on motion/maneuvers
     * 
     * Pre-Flight Calibration:
     * - If gyros pre-calibrated: Lower uncertainty
     * - If cold start: Higher uncertainty
     * - Gyro warm-up affects bias
     * - Temperature stabilization important
     * 
     * Use Cases:
     * - Filter initialization
     * - Cold start scenarios
     * - Post-reset recovery
     * - Conservative vs aggressive tuning
     * 
     * Tuning Considerations:
     * - Match actual gyro performance
     * - Consider environment (temperature)
     * - Vehicle dynamics matter
     * - Field testing validates choice
     * 
     * @note Vehicle-specific gyro bias initialization
     * @note Affects convergence speed
     * @note Trade-off: Speed vs robustness
     * @note Used in CovarianceInit()
     * 
     * @see CovarianceInit() for covariance initialization
     * @see stateStruct.gyro_bias (9-11) for bias states
     * @see P for covariance matrix
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:1194
     */
    ftype InitialGyroBiasUncertainty(void) const;

    // The following declarations are used to control when the main navigation filter resets it's yaw to the estimate provided by the GSF
    uint32_t EKFGSF_yaw_reset_ms;           // timestamp of last emergency yaw reset (uSec)
    uint32_t EKFGSF_yaw_reset_request_ms;   // timestamp of last emergency yaw reset request (uSec)
    uint8_t EKFGSF_yaw_reset_count;         // number of emergency yaw resets performed
    bool EKFGSF_run_filterbank;             // true when the filter bank is active

    // logging timestamps
    uint32_t lastTimingLogTime_ms;

    /**
     * @brief Logging functions shared by EKF cores
     * 
     * @details These methods write EKF state and diagnostic information to the
     *          binary log system (AP_Logger). Each function logs specific subsets
     *          of EKF data for post-flight analysis, debugging, and validation.
     *          Called from Log_Write() which coordinates all logging.
     */

    /**
     * @brief Write NKF1 log message: Innovations and variances
     * 
     * @details Logs velocity and position innovations (measurement - prediction)
     *          and their variances. Primary data for assessing measurement consistency
     *          and filter health. Critical for debugging GPS and sensor fusion issues.
     * 
     * @param[in] time_us Timestamp in microseconds (AP_HAL::micros64())
     * 
     * NKF1 Message Contents:
     * - TimeUS: Timestamp (µs)
     * - Roll, Pitch, Yaw: Euler angles (degrees)
     * - VN, VE, VD: Velocity innovations NED (m/s)
     * - PN, PE, PD: Position innovations NED (m)
     * - GX, GY, GZ: Gyro bias estimates (rad/s)
     * 
     * Innovations Logged:
     * \n
     * Velocity Innovations:\n
     * - innovVelPos[0]: North velocity innovation\n
     * - innovVelPos[1]: East velocity innovation\n
     * - innovVelPos[2]: Down velocity innovation\n
     * - Units: m/s\n
     * - Ideally small (< 1 m/s)\n
     * \n
     * Position Innovations:\n
     * - innovVelPos[3]: North position innovation\n
     * - innovVelPos[4]: East position innovation\n
     * - innovVelPos[5]: Down position innovation\n
     * - Units: meters\n
     * - Ideally small (< 5 m)\n
     * 
     * Innovation Interpretation:
     * - Small innovations: Good agreement
     * - Large innovations: Measurement mismatch
     * - Sustained large: Filter divergence
     * - Sudden spike: Sensor fault
     * 
     * Use Cases:
     * - GPS quality assessment
     * - Filter convergence monitoring
     * - Sensor fault detection
     * - Debugging navigation issues
     * 
     * @note Core innovation and variance data
     * @note Primary filter health indicator
     * @note Essential for debugging
     * 
     * @see innovVelPos for innovation storage
     * @see varInnovVelPos for variance storage
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:1206
     */
    void Log_Write_NKF1(uint64_t time_us) const;

    /**
     * @brief Write NKF2 log message: Primary states and velocities
     * 
     * @details Logs the main navigation states: attitude (yaw), velocity, position,
     *          and gyro biases. Core EKF state vector data for trajectory reconstruction
     *          and performance analysis.
     * 
     * @param[in] time_us Timestamp in microseconds
     * 
     * NKF2 Message Contents:
     * - TimeUS: Timestamp (µs)
     * - AZbias: Accelerometer Z bias (m/s²)
     * - IVN, IVE, IVD: Innovation velocities (m/s)
     * - IPN, IPE, IPD: Innovation positions (m)
     * - IVT: True airspeed innovation (m/s)
     * - Yaw: Yaw angle innovation (rad)
     * 
     * States Logged:
     * - Velocity: NED frame (m/s)
     * - Position: NED frame (m)
     * - Gyro bias: Body frame (rad/s)
     * - Accel Z bias: Body Z axis (m/s²)
     * 
     * Use Cases:
     * - Trajectory reconstruction
     * - State estimation validation
     * - Bias tracking
     * - Performance analysis
     * 
     * @note Main navigation states
     * @note Essential for trajectory analysis
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:1207
     */
    void Log_Write_NKF2(uint64_t time_us) const;

    /**
     * @brief Write NKF3 log message: Magnetic field states and innovations
     * 
     * @details Logs magnetometer innovations, magnetic field state estimates (earth
     *          and body), and compass-related diagnostic data. Critical for debugging
     *          magnetic interference and compass calibration issues.
     * 
     * @param[in] time_us Timestamp in microseconds
     * 
     * NKF3 Message Contents:
     * - TimeUS: Timestamp (µs)
     * - IVX, IVY, IVZ: Magnetometer innovations (gauss)
     * - VarX, VarY, VarZ: Mag innovation variances (gauss²)
     * - MN, ME, MD: Earth magnetic field NED (gauss)
     * - MX, MY, MZ: Body magnetic field XYZ (gauss)
     * 
     * Magnetic States:
     * \n
     * Earth Field (16-18):\n
     * - North component\n
     * - East component\n
     * - Down component\n
     * - Should match WMM model\n
     * \n
     * Body Field (19-21):\n
     * - X component (forward)\n
     * - Y component (right)\n
     * - Z component (down)\n
     * - Vehicle magnetic signature\n
     * 
     * Use Cases:
     * - Compass calibration validation
     * - Magnetic interference detection
     * - Heading accuracy assessment
     * - Anomaly investigation
     * 
     * @note Magnetometer fusion diagnostics
     * @note Critical for compass debugging
     * 
     * @see innovMag for mag innovations
     * @see stateStruct.earth_magfield (16-18)
     * @see stateStruct.body_magfield (19-21)
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:1208
     */
    void Log_Write_NKF3(uint64_t time_us) const;

    /**
     * @brief Write NKF4 log message: Wind, airspeed, and range innovations
     * 
     * @details Logs wind velocity estimates, airspeed data, and range finder
     *          measurements. Important for fixed-wing navigation and terrain
     *          following analysis.
     * 
     * @param[in] time_us Timestamp in microseconds
     * 
     * NKF4 Message Contents:
     * - TimeUS: Timestamp (µs)
     * - SV: Airspeed measurement variance (m/s)²
     * - SP: Sideslip angle (rad)
     * - SH: Sensor height AGL (m)
     * - WN, WE: Wind velocity NE (m/s)
     * - Wvar: Wind velocity variance (m/s)²
     * - Hgt: Terrain height estimate (m)
     * 
     * Wind States (22-23):
     * - Wind North: 2D wind velocity
     * - Wind East: 2D wind velocity
     * - Positive = air moving in axis direction
     * - Used for airspeed correction
     * 
     * Use Cases:
     * - Wind estimation validation
     * - Airspeed sensor debugging
     * - Terrain following analysis
     * - Fixed-wing navigation
     * 
     * @note Wind, airspeed, and terrain data
     * @note Important for fixed-wing
     * 
     * @see stateStruct.wind_vel (22-23)
     * @see terrainState for terrain height
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:1209
     */
    void Log_Write_NKF4(uint64_t time_us) const;

    /**
     * @brief Write NKF5 log message: Auxiliary innovations and filter status
     * 
     * @details Logs optical flow, range beacon, and other auxiliary sensor
     *          innovations plus overall filter status flags. Provides comprehensive
     *          view of all sensor fusion performance.
     * 
     * @param[in] time_us Timestamp in microseconds
     * 
     * NKF5 Message Contents:
     * - TimeUS: Timestamp (µs)
     * - FIX, FIY: Optical flow innovations X,Y (rad/s)
     * - FVX, FVY: Flow innovation variances (rad/s)²
     * - RI: Range beacon innovation (m)
     * - RIV: Range beacon innovation variance (m²)
     * - FH: Filter health flags
     * - FS: Filter status flags
     * 
     * Status Flags:
     * - Filter initialized
     * - Attitude valid
     * - Position valid
     * - Velocity valid
     * - GPS fusion active
     * - Optical flow fusion active
     * 
     * Use Cases:
     * - Optical flow debugging
     * - Range beacon validation
     * - Filter status monitoring
     * - Multi-sensor fusion analysis
     * 
     * @note Auxiliary sensors and status
     * @note Filter health indicators
     * 
     * @see filterStatus for status flags
     * @see innovOptFlow for flow innovations
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:1210
     */
    void Log_Write_NKF5(uint64_t time_us) const;

    /**
     * @brief Write quaternion log message for attitude
     * 
     * @details Logs the full quaternion state representing vehicle attitude.
     *          Provides complete attitude information without gimbal lock issues
     *          of Euler angles. Essential for high-rate attitude analysis.
     * 
     * @param[in] time_us Timestamp in microseconds
     * 
     * Quaternion Message Contents:
     * - TimeUS: Timestamp (µs)
     * - Q1: Quaternion component 1 (scalar w)
     * - Q2: Quaternion component 2 (vector x)
     * - Q3: Quaternion component 3 (vector y)
     * - Q4: Quaternion component 4 (vector z)
     * 
     * Quaternion Benefits:
     * - No gimbal lock
     * - Continuous representation
     * - Efficient computations
     * - Smooth interpolation
     * 
     * State Source:
     * - stateStruct.quat (24-27)
     * - Normalized quaternion
     * - NED to body frame rotation
     * - Unit length (w² + x² + y² + z² = 1)
     * 
     * Use Cases:
     * - High-rate attitude reconstruction
     * - Gimbal lock avoidance
     * - Smooth trajectory analysis
     * - Aerobatic flight logging
     * 
     * @note Full quaternion attitude state
     * @note No gimbal lock issues
     * @note High precision attitude
     * 
     * @see stateStruct.quat (24-27)
     * @see getQuaternion() for retrieval
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:1211
     */
    void Log_Write_Quaternion(uint64_t time_us) const;

    /**
     * @brief Write range beacon fusion diagnostic data
     * 
     * @details Logs range beacon measurements, innovations, and diagnostic
     *          information. Used for debugging indoor positioning and range
     *          beacon-based navigation systems.
     * 
     * @param[in] time_us Timestamp in microseconds
     * 
     * Beacon Message Contents:
     * - TimeUS: Timestamp (µs)
     * - ID: Beacon identifier
     * - Rng: Measured range (m)
     * - Innov: Range innovation (m)
     * - InnovVar: Innovation variance (m²)
     * - TestRatio: Consistency test ratio
     * - BcnN, BcnE, BcnD: Beacon position NED (m)
     * 
     * Range Beacon System:
     * - Indoor positioning
     * - Multiple beacons at known locations
     * - Measure range to each beacon
     * - Triangulate vehicle position
     * 
     * Diagnostic Data:
     * - Per-beacon innovations
     * - Consistency checks
     * - Beacon positions
     * - Fusion performance
     * 
     * Use Cases:
     * - Indoor positioning debugging
     * - Beacon placement validation
     * - Range accuracy assessment
     * - Triangulation analysis
     * 
     * @note Range beacon fusion diagnostics
     * @note Indoor positioning systems
     * @note Not const (updates report index)
     * 
     * @see rngBcnFusionReport for data storage
     * @see FuseRngBcn() for beacon fusion
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:1212
     */
    void Log_Write_Beacon(uint64_t time_us);

    /**
     * @brief Write timing statistics for performance monitoring
     * 
     * @details Logs EKF timing information including update rates, computational
     *          load, and timing consistency. Critical for detecting performance
     *          issues and validating real-time execution.
     * 
     * @param[in] time_us Timestamp in microseconds
     * 
     * Timing Message Contents:
     * - TimeUS: Timestamp (µs)
     * - IMUdt: IMU sample period (ms)
     * - EKFdt: EKF update period (ms)
     * - FramesSincePredict: Frames since last prediction
     * - LocalFilterTimeStep: Average filter time step (ms)
     * 
     * Timing Metrics:
     * - IMU sample rate
     * - EKF update rate
     * - Computational jitter
     * - Load balancing effectiveness
     * 
     * Performance Indicators:
     * - Consistent update rates: Good
     * - Variable rates: Jitter/overload
     * - High framesSincePredict: CPU overload
     * - Timing within targets: Nominal
     * 
     * Use Cases:
     * - Performance monitoring
     * - Computational overload detection
     * - Timing jitter analysis
     * - Real-time validation
     * 
     * @note EKF timing performance data
     * @note Critical for real-time validation
     * 
     * @see timing for timing statistics
     * @see updateTimingStatistics() for updates
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:1213
     */
    void Log_Write_Timing(uint64_t time_us);

    /**
     * @brief Write GSF yaw estimator diagnostic data
     * 
     * @details Logs Gaussian Sum Filter yaw estimator state, including yaw estimate,
     *          uncertainty, hypothesis weights, and convergence status. Essential for
     *          debugging magnetometer backup systems and emergency yaw recovery.
     * 
     * @param[in] time_us Timestamp in microseconds
     * 
     * GSF Message Contents:
     * - TimeUS: Timestamp (µs)
     * - YawEst: GSF yaw estimate (rad)
     * - YawVar: Yaw estimate variance (rad²)
     * - MaxWeight: Maximum hypothesis weight (0-1)
     * - NumHyp: Number of active hypotheses
     * - Converged: Convergence flag (boolean)
     * 
     * GSF Diagnostics:
     * - Yaw estimate quality
     * - Convergence status
     * - Hypothesis distribution
     * - Filter confidence
     * 
     * Convergence Indicators:
     * - MaxWeight > 0.95: Well converged
     * - YawVar < 0.01: High confidence
     * - Converged flag set
     * - Ready for yaw reset
     * 
     * Use Cases:
     * - GSF convergence monitoring
     * - Magnetometer backup validation
     * - Emergency yaw recovery debugging
     * - Yaw reset decision analysis
     * 
     * @note GSF yaw estimator diagnostics
     * @note Magnetometer backup system
     * @note Emergency yaw recovery
     * 
     * @see yawEstimator for EKFGSF_yaw instance
     * @see EKFGSF_resetMainFilterYaw() for yaw reset
     * 
     * Source: libraries/AP_NavEKF2/AP_NavEKF2_core.h:1214
     */
    void Log_Write_GSF(uint64_t time_us) const;
};
