/**
 * @file AP_NavEKF2.h
 * @brief Extended Kalman Filter Version 2 (NavEKF2) - Public Front-End API
 * 
 * This file defines the NavEKF2 class, which provides the public interface for
 * ArduPilot's second-generation Extended Kalman Filter for inertial navigation.
 * 
 * NavEKF2 is a 24-state Extended Kalman Filter that fuses IMU, GPS, barometer,
 * magnetometer, airspeed, optical flow, range finder, and range beacon measurements
 * to provide high-accuracy position, velocity, and attitude estimates for autopilot
 * flight control.
 * 
 * Key Features:
 * - Multi-core/lane architecture supporting multiple EKF instances for IMU redundancy
 * - Primary core selection based on innovation consistency and error scores
 * - Lane switching with anti-churn timer to prevent rapid oscillation
 * - Continuous-time state prediction with discrete measurement updates
 * - Adaptive aiding mode management (GPS, non-GPS, dead-reckoning)
 * - EKF-GSF yaw estimator integration for magnetic anomaly recovery
 * 
 * Architecture:
 * - Front-end (this file): Parameter management, core selection, API delegation
 * - Backend (NavEKF2_core): State prediction, covariance propagation, measurement fusion
 * - Typically runs 2 cores (one per IMU) for redundancy and fault tolerance
 * - Primary core selected dynamically based on innovation consistency test ratios
 * - Lane switching prevented within 5 seconds (anti-churn timer)
 * 
 * State Vector (24 states + quaternion):
 * - Position NED (3): North, East, Down position relative to EKF origin (m)
 * - Velocity NED (3): Velocity in NED frame (m/s)
 * - Attitude (3 rotation error + 4 quaternion): Attitude as quaternion with error parameterization
 * - Gyro Bias (3): Gyroscope bias estimates (rad/s)
 * - Gyro Scale (3): Gyroscope scale factor errors (dimensionless)
 * - Accel Z Bias (1): Z-axis accelerometer bias (m/s²)
 * - Earth Magnetic Field (3): NED earth magnetic field (gauss/1000)
 * - Body Magnetic Field (3): Body frame magnetic offsets for hard iron compensation (gauss/1000)
 * - Wind Velocity (2): NE wind velocity (m/s)
 * 
 * Measurement Sources:
 * - GPS: Position and velocity in NED frame
 * - Barometer: Pressure altitude
 * - Magnetometer: 3-axis magnetic field for heading
 * - Airspeed: True airspeed for wind estimation
 * - Optical Flow: 2D velocity from flow sensor
 * - Range Finder: Height above ground
 * - Range Beacons: Ranges to fixed beacons for positioning
 * - External Navigation: Visual odometry or motion capture position/velocity
 * 
 * Mathematical Foundation:
 * Full derivation available at:
 * https://github.com/priseborough/InertialNav/blob/master/derivations/RotationVectorAttitudeParameterisation/GenerateNavFilterEquations.m
 * 
 * @note All navigation outputs are in NED (North-East-Down) coordinate frame
 * @note Coordinate frame origin set at filter initialization or via setOriginLLH()
 * 
 * @warning EKF tuning parameters critically affect navigation accuracy and vehicle safety
 * @warning Incorrect tuning can cause navigation failures, crashes, or flyaways
 * @warning Innovation gate parameters must balance outlier rejection vs measurement acceptance
 * 
 * @author Paul Riseborough (Matlab to C++ conversion)
 * @author Tom Cauchois (EKF tuning parameter refactoring)
 * 
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
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
 * Source: libraries/AP_NavEKF2/AP_NavEKF2.h:1-478
 */
#pragma once

#include <AP_Common/Location.h>
#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>
#include <AP_NavEKF/AP_Nav_Common.h>

class NavEKF2_core;
class EKFGSF_yaw;

/**
 * @class NavEKF2
 * @brief Extended Kalman Filter Version 2 for inertial navigation with multi-IMU redundancy
 * 
 * @details NavEKF2 implements a 24-state Extended Kalman Filter that provides high-accuracy
 * position, velocity, and attitude estimates for ArduPilot flight control. The filter fuses
 * measurements from multiple sensor sources to maintain robust navigation even in challenging
 * environments or sensor failure scenarios.
 * 
 * Multi-Core/Lane Architecture:
 * - Supports multiple independent EKF instances (cores/lanes) running in parallel
 * - Typically runs 2 cores, each using a different IMU for redundancy
 * - Primary core dynamically selected based on innovation consistency and error scores
 * - Lane switching with 5-second anti-churn timer prevents rapid oscillation
 * - Enables fault-tolerant operation if one IMU fails or has high noise
 * 
 * Core Selection Algorithm:
 * - Each core computes an error score from innovation consistency test ratios
 * - Core with lowest error score becomes the primary candidate
 * - Switch occurs if candidate core has significantly better score than current primary
 * - Anti-churn timer (lastLaneSwitch_ms) enforces minimum 5000ms between switches
 * - Reset data structures track discontinuities (yaw, position) for control compensation
 * 
 * State Vector Composition (24 states + 4 quaternion elements):
 * | States | Component           | Units      | Description                                    |
 * |--------|---------------------|------------|------------------------------------------------|
 * | 0-2    | Rotation Error      | rad        | 3D rotation error vector (minimal param)       |
 * | 3-5    | Velocity NED        | m/s        | Velocity in North-East-Down frame              |
 * | 6-8    | Position NED        | m          | Position relative to EKF origin (NED frame)    |
 * | 9-11   | Gyro Bias           | rad/s      | Gyroscope bias estimates (body frame)          |
 * | 12-14  | Gyro Scale          | -          | Gyroscope scale factor errors (dimensionless)  |
 * | 15     | Accel Z Bias        | m/s²       | Z-axis accelerometer bias (body frame)         |
 * | 16-18  | Earth Mag Field     | gauss/1000 | Earth magnetic field (NED frame, learned)      |
 * | 19-21  | Body Mag Field      | gauss/1000 | Body magnetic offsets (hard iron, learned)     |
 * | 22-23  | Wind Velocity       | m/s        | Wind velocity in NE plane                      |
 * | 24-27  | Quaternion          | -          | Attitude quaternion (NED to body rotation)     |
 * 
 * Sensor Fusion Capabilities:
 * - GPS: Position and velocity measurements (configurable fusion modes)
 * - Barometer: Pressure altitude with ground effect compensation
 * - Magnetometer: 3-axis magnetic field for yaw observability
 * - Airspeed: True airspeed for wind estimation and sideslip constraint
 * - Optical Flow: 2D flow rates for velocity estimation (non-GPS navigation)
 * - Range Finder: Height above ground for terrain following
 * - Range Beacons: Ranges to known beacon positions for indoor navigation
 * - External Navigation: Visual odometry or motion capture (position/velocity/attitude)
 * 
 * Aiding Mode Management:
 * - GPS Aiding: Full position and velocity from GPS
 * - Non-GPS Aiding: Optical flow, external nav, or range beacons provide velocity/position
 * - Dead-Reckoning: Inertial-only mode (accumulates drift without aiding)
 * - Automatic transition between modes based on sensor availability and quality
 * - Timeout and retry logic for each aiding source
 * 
 * EKF-GSF Yaw Estimator Integration:
 * - Separate Gaussian Sum Filter estimates yaw from IMU and GPS velocity
 * - Provides yaw reference for emergency yaw resets during magnetic anomalies
 * - Limited to _gsfResetMaxCount resets per flight to prevent repeated failures
 * - Yaw reset triggers when magnetometer innovations consistently exceed gates
 * 
 * Coordinate Frames:
 * - NED Frame: North-East-Down navigation frame, origin at EKF initialization
 * - Body Frame: Forward-Right-Down aircraft body frame (rotates with vehicle)
 * - Sensor Frames: Individual sensor frames with known offsets from body frame
 * 
 * @note All position, velocity, and magnetic field outputs are in NED coordinate frame
 * @note Attitude is provided as quaternion (NED to body rotation) or Euler angles
 * @note Units are always specified explicitly: meters (m), m/s, rad, rad/s, gauss, etc.
 * 
 * @warning EKF tuning parameters critically affect navigation accuracy and must be adjusted carefully
 * @warning Process noise parameters control filter convergence speed vs steady-state noise
 * @warning Measurement noise parameters control sensor weight vs outlier rejection
 * @warning Innovation gates must balance accepting valid data vs rejecting outliers
 * @warning GPS glitch protection (_gpsGlitchRadiusMax) prevents position jumps but may reject valid data during rapid maneuvers
 * @warning Magnetometer fusion is critical for yaw observability; mag failures cause yaw drift and position errors
 * @warning Incorrect EKF tuning can cause navigation failures, altitude loss, crashes, or flyaways
 * 
 * Usage Pattern:
 * @code
 * // Initialize EKF2
 * NavEKF2 ekf;
 * if (!ekf.InitialiseFilter()) {
 *     // Handle initialization failure (insufficient memory, no IMUs, etc.)
 * }
 * 
 * // Main loop (called at IMU rate, typically 400Hz)
 * ekf.UpdateFilter();
 * 
 * // Read navigation outputs
 * if (ekf.healthy()) {
 *     Vector2f posNE;
 *     ekf.getPosNE(posNE);  // Get NE position
 *     
 *     Vector3f velNED;
 *     ekf.getVelNED(velNED);  // Get NED velocity
 *     
 *     Quaternion quat;
 *     ekf.getQuaternion(quat);  // Get attitude quaternion
 * }
 * 
 * // Pre-arm safety check
 * char failure_msg[50];
 * if (!ekf.pre_arm_check(failure_msg, sizeof(failure_msg))) {
 *     // Arming not allowed, failure_msg contains reason
 * }
 * @endcode
 * 
 * @see NavEKF2_core for backend implementation details
 * @see AP_AHRS for attitude and heading reference system integration
 * @see https://github.com/priseborough/InertialNav for mathematical derivation
 */
class NavEKF2 {
    friend class NavEKF2_core;

public:
    NavEKF2();

    /* Do not allow copies */
    CLASS_NO_COPY(NavEKF2);

    static const struct AP_Param::GroupInfo var_info[];

    /**
     * @brief Get number of active EKF cores
     * @return Number of allocated and running cores (typically 2 for dual-IMU systems)
     * @details Used by logging system to determine how many cores to log data from
     */
    uint8_t activeCores(void) const {
        return num_cores;
    }

    /**
     * @brief Initialize EKF cores and allocate memory
     * @return true if initialization successful, false on failure
     * 
     * @details Initialization sequence:
     * - Checks EK2_ENABLE parameter (must be 1)
     * - Detects available IMUs using AP::dal().ins()
     * - Allocates memory for NavEKF2_core array based on EK2_IMU_MASK
     * - Uses placement new with AP::dal().malloc_type() for memory allocation
     * - Prefers FAST memory regions (DTCM on STM32) for performance
     * - Initializes each core with its assigned IMU index
     * - Sets up EKF-GSF yaw estimator if enabled
     * 
     * @note Must be called during vehicle initialization before first UpdateFilter() call
     * @note Typically called from AHRS during system startup
     * 
     * @warning Requires sufficient memory in MAIN_RAM or FAST memory regions
     * @warning Returns false if EK2_ENABLE=0, no IMUs available, or memory allocation fails
     * @warning Vehicle should not arm if initialization fails
     * 
     * Memory Requirements:
     * - Each core requires ~8-12 KB depending on configuration
     * - EKF-GSF yaw estimator requires additional ~2 KB per enabled core
     * - Allocation failure sets core_malloc_failed flag
     */
    bool InitialiseFilter(void);

    /**
     * @brief Main filter update called at IMU sample rate
     * 
     * @details Update sequence executed each call:
     * 1. Run prediction step for each core using latest IMU delta angles/velocities
     * 2. Schedule measurement updates based on sensor availability and timing
     * 3. Execute measurement fusion (GPS, baro, mag, airspeed, flow, etc.)
     * 4. Perform innovation consistency checks and update error scores
     * 5. Execute core selection algorithm and lane switching if beneficial
     * 6. Update output states with time-aligned delayed horizon values
     * 
     * Timing Characteristics:
     * - Called at IMU rate: typically 400Hz for copter, 100-400Hz for plane
     * - Prediction step runs every call
     * - Covariance prediction limited to fusionTimeStep_ms (default 10ms)
     * - Measurement updates scheduled based on sensor sample rates and delays
     * - Execution time typically 200-500 microseconds per core on STM32H7
     * 
     * @note Must be called at consistent rate matching IMU sample rate
     * @note Prediction uses IMU delta angles and velocities since last call
     * @note Longer time between calls (>100ms) may degrade filter performance
     * 
     * @warning Do not call faster than IMU rate (will process duplicate IMU samples)
     * @warning Large time gaps (>1 second) may cause filter divergence
     * @warning CPU overload can cause delayed execution and increased estimation errors
     */
    void UpdateFilter(void);

    /**
     * @brief Consolidated health check across all cores
     * @return true if primary core is healthy and navigation solution is valid
     * 
     * @details Health checks performed on primary core:
     * - Filter is initialized (InitialiseFilter completed successfully)
     * - No NaN values in position, velocity, or quaternion states
     * - At least one aiding source active (GPS, optical flow, external nav, beacons)
     * - Innovation checks not consistently failing
     * - Covariance matrix remains positive definite and well-conditioned
     * - No critical filter faults in getFilterFaults() bitmask
     * 
     * Unhealthy Conditions:
     * - Filter not initialized or initialization failed
     * - Primary core index is -1 (no primary selected)
     * - Core has entered dead-reckoning mode with no aiding
     * - Position or velocity states contain NaN or Inf
     * - Covariance has grown unbounded indicating divergence
     * 
     * @note Health status can change rapidly during sensor transitions or failures
     * @note Vehicle code should implement hysteresis before triggering failsafe
     * @note Logging NKF4 messages contain detailed health and fault information
     * 
     * @warning Returning false indicates navigation solution is not suitable for flight control
     * @warning Vehicle should trigger EKF failsafe if healthy() remains false for extended period
     * @warning Do not use position/velocity outputs for control when healthy() returns false
     */
    bool healthy(void) const;

    /**
     * @brief Pre-arm safety checks before allowing vehicle arming
     * @param[out] failure_msg Buffer to receive failure message string
     * @param[in] failure_msg_len Size of failure_msg buffer in bytes
     * @return false if any check fails (arming should be prevented), true if all checks pass
     * 
     * @details Pre-arm checks performed:
     * 
     * GPS Checks (if using GPS aiding):
     * - Minimum satellite count: 6 for 3D fix, 9 if using GPS yaw
     * - HDOP within acceptable range (typically <2.0)
     * - GPS horizontal accuracy within threshold (scaled by EK2_CHECK_SCALE)
     * - GPS vertical accuracy within threshold
     * - GPS speed accuracy within threshold
     * - Position drift rate acceptable
     * - Vertical velocity within reasonable bounds
     * 
     * Compass Checks:
     * - Magnetometer healthy and providing valid data
     * - Compass orientation configured correctly
     * - Magnetic field magnitude within expected range (0.2-0.6 gauss typically)
     * - Compass offsets learned if in-flight calibration enabled
     * 
     * Filter Initialization Checks:
     * - Filter successfully initialized (InitialiseFilter returned true)
     * - Primary core selected and healthy
     * - Attitude alignment completed (not in INIT_NO_AIDING state)
     * - Position initialized if using GPS or external nav
     * 
     * Innovation Checks:
     * - GPS innovations within acceptable bounds
     * - Magnetometer innovations not excessive
     * - Height innovations reasonable
     * 
     * @note EK2_CHECK_SCALE parameter scales thresholds: 100=normal, 200=relaxed, 50=strict
     * @note Some checks can be disabled via EK2_OPTIONS or vehicle-specific parameters
     * @note Failure message limited to failure_msg_len characters (typically 50)
     * 
     * @warning Vehicle MUST NOT arm if this returns false
     * @warning Arming with failed pre-arm checks risks immediate crash or flyaway
     * @warning GPS checks are critical - poor GPS can cause position errors and crashes
     * 
     * Example failure messages:
     * - "EKF2 not initialized"
     * - "GPS horiz accuracy too high"
     * - "GPS speed accuracy too high"
     * - "Mag field strength out of range"
     * - "Large GPS velocity innovations"
     */
    bool pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const;

    /**
     * @brief Get index of currently active primary core
     * @return Core index (0 to num_cores-1), or -1 if no primary core selected
     * 
     * @details Primary core is the EKF instance currently providing navigation outputs.
     * Core selection based on:
     * - Innovation consistency test ratios (lower is better)
     * - Error score computed from weighted innovation metrics
     * - Alignment status (must have valid attitude, position, velocity)
     * - Lane switching only occurs if new core significantly better than current
     * - 5-second anti-churn timer prevents rapid switching
     * 
     * @note Used by logging system to mark primary core data
     * @note Used by vehicle code to query which IMU is being used for navigation
     * @note Returns -1 before InitialiseFilter() or if all cores unhealthy
     * 
     * @see getPrimaryCoreIMUIndex() to get the IMU index used by primary core
     * @see checkLaneSwitch() to request lane switch attempt
     */
    int8_t getPrimaryCoreIndex(void) const;

    /**
     * @brief Get IMU index used by primary core
     * @return IMU index (0 to number of IMUs - 1), or -1 if no primary core
     * 
     * @details Each EKF core is assigned a specific IMU based on EK2_IMU_MASK.
     * This function returns which physical IMU the current primary core is using.
     * 
     * Typical configurations:
     * - Single IMU: Core 0 uses IMU 0, returns 0
     * - Dual IMU: Core 0 uses IMU 0, Core 1 uses IMU 1, returns 0 or 1 depending on primary
     * - Triple IMU: Cores assigned to IMU 0, 1, 2 based on mask
     * 
     * @note Multiple cores can use the same IMU if EK2_IMU_MASK configured that way
     * @note Returns -1 before initialization or if no primary selected
     * @note Used for diagnostics and IMU health monitoring
     * 
     * @see getPrimaryCoreIndex() to get core index
     */
    int8_t getPrimaryCoreIMUIndex(void) const;
    
    /**
     * @brief Get NED position (North-East components) relative to EKF origin
     * @param[out] posNE Position vector (m) in NED frame, North and East components only
     * @return true if solution valid for flight control, false if solution degraded
     * 
     * @details Returns the horizontal position estimate from the primary EKF core.
     * Position is relative to the EKF origin, which is set at:
     * - GPS: First GPS lock position (or manually set via SET_GPS_GLOBAL_ORIGIN)
     * - Non-GPS: Initial position when filter initialized
     * - External nav: Origin provided by external navigation system
     * 
     * Return value indicates solution quality:
     * - true: Position estimate valid, suitable for flight control (normal operation)
     * - false: Position degraded (dead-reckoning, initialization, or failed aiding)
     * 
     * Position always populated even when returning false, but may drift rapidly
     * without aiding (GPS, optical flow, external nav, or beacons).
     * 
     * Coordinate Frame:
     * - NED (North-East-Down) navigation frame
     * - posNE.x = North displacement in meters (positive = north)
     * - posNE.y = East displacement in meters (positive = east)
     * - Origin altitude preserved across resetHeightDatum() calls
     * 
     * @note Position type is Vector2p which may be float or double depending on HAL_WITH_POSTYPE_DOUBLE
     * @note Filtered output with time delay compensation (delayed horizon state propagation)
     * @note Typical accuracy: GPS aiding 1-5m, optical flow 0.5-2m, dead-reckoning >10m and growing
     * 
     * @warning If false returned, DO NOT use for flight control - risk of position divergence
     * @warning Vehicle should trigger failsafe if consistently returns false during flight
     * @warning Position reference frame changes if resetHeightDatum() or origin changed
     * 
     * @see getPosD() for down position component
     * @see getVelNED() for velocity estimate
     * @see healthy() for overall filter health check
     */
    bool getPosNE(Vector2p &posNE) const;

    /**
     * @brief Get NED position (Down component) relative to EKF origin
     * @param[out] posD Down position (m) in NED frame, positive = down from origin
     * @return true if solution valid for flight control, false if solution degraded
     * 
     * @details Returns the vertical position estimate from the primary EKF core.
     * Down position measured positive downward from EKF origin altitude.
     * 
     * Height sources (selected by EK2_ALT_SOURCE and runtime conditions):
     * - Barometer (default): Pressure altitude relative to initialization or reset
     * - GPS: GPS altitude (MSL or ellipsoid depending on GPS receiver)
     * - Rangefinder: Height above ground when EK2_RNG_USE_HGT conditions met
     * - Range beacon: Triangulated altitude from beacon ranges
     * - External nav: Altitude from visual odometry or motion capture system
     * 
     * Return value indicates solution quality:
     * - true: Height estimate valid for flight control
     * - false: Height degraded or not converged
     * 
     * Coordinate Frame:
     * - NED Down axis (positive = below origin)
     * - posD = 0 at EKF origin altitude
     * - posD = 10.0 means 10 meters below origin
     * - posD = -5.0 means 5 meters above origin
     * 
     * @note Position type is postype_t which may be float or double depending on HAL_WITH_POSTYPE_DOUBLE
     * @note resetHeightDatum() sets posD to zero while preserving absolute altitude
     * @note Typical accuracy: Baro 1-3m, GPS 3-10m, rangefinder 0.1-0.5m
     * 
     * @warning If false returned, DO NOT use for flight control
     * @warning Barometer subject to pressure changes from weather and ground effect
     * @warning GPS altitude less accurate than horizontal position
     * 
     * @see getPosNE() for horizontal position components
     * @see getPosDownDerivative() for vertical velocity
     * @see resetHeightDatum() to reset height datum to current altitude
     */
    bool getPosD(postype_t &posD) const;

    /**
     * @brief Get NED velocity estimate
     * @param[out] vel Velocity vector (m/s) in NED frame
     * 
     * @details Returns the velocity estimate from the primary EKF core.
     * Velocity always populated even if filter is unhealthy, but accuracy degrades without aiding.
     * 
     * Velocity sources:
     * - GPS velocity (primary when available): Doppler-based, typically accurate to 0.1-0.3 m/s
     * - Optical flow: Ground-relative velocity, requires valid height above ground
     * - Airspeed + wind estimate: For fixed-wing without GPS
     * - IMU integration: Dead-reckoning when no aiding available (drifts rapidly)
     * 
     * Coordinate Frame:
     * - NED (North-East-Down) navigation frame
     * - vel.x = North velocity in m/s (positive = moving north)
     * - vel.y = East velocity in m/s (positive = moving east)
     * - vel.z = Down velocity in m/s (positive = descending)
     * 
     * Velocity Characteristics:
     * - Updated at IMU rate (typically 400Hz) with prediction
     * - Corrected by measurements when available
     * - Includes wind compensation for airspeed-based estimates
     * - Does NOT include wind - this is ground-relative velocity
     * 
     * @note Always returns data, no validity flag (use healthy() to check overall state)
     * @note Velocity typically more accurate than position, especially short-term
     * @note Fixed-wing: Airspeed + wind gives reasonable velocity even with poor GPS
     * @note Multicopter: Relies heavily on GPS or optical flow for velocity accuracy
     * 
     * @warning Dead-reckoning velocity drifts due to accelerometer bias and scale errors
     * @warning Optical flow velocity only valid when flow sensor has valid data
     * @warning Velocity estimate lags actual velocity by sensor delays (GPS ~200ms, flow ~10ms)
     * 
     * @see getAirSpdVec() for true airspeed vector in body frame
     * @see getWind() for wind velocity estimate
     * @see getPosDownDerivative() for kinematically consistent vertical velocity
     */
    void getVelNED(Vector3f &vel) const;

    /**
     * @brief Get true airspeed vector estimate in body frame
     * @param[out] vel True airspeed (TAS) vector (m/s) in body frame (forward-right-down)
     * @return true if airspeed estimate available, false if unavailable
     * 
     * @details Returns body-frame true airspeed vector computed from:
     * - Airspeed sensor fusion (when sensor installed and healthy)
     * - Wind estimate subtracted from ground velocity
     * - Synthetic airspeed based on throttle and vehicle model (fallback)
     * 
     * True airspeed vector represents velocity of vehicle relative to airmass:
     * - TAS = groundspeed_vector - wind_vector (rotated to body frame)
     * - Accounts for air density via EAS to TAS conversion
     * - Includes wind estimate learned by filter during flight
     * 
     * Coordinate Frame (Body Frame):
     * - vel.x = Forward airspeed (m/s, positive = forward through air)
     * - vel.y = Right airspeed (m/s, positive = right through air, ideally ~0 in coordinated flight)
     * - vel.z = Down airspeed (m/s, positive = descending through air)
     * 
     * Return Conditions:
     * - true: Airspeed sensor fused OR wind estimate valid with GPS
     * - false: No airspeed sensor AND (no GPS OR wind not observable)
     * 
     * Typical Usage:
     * - Fixed-wing: Critical for stall prevention, TECS energy management
     * - Multicopter: Used for wind estimation and forward flight optimization
     * - Airspeed sensor: Directly measured, accurate to ~0.5-1.0 m/s
     * - Synthetic airspeed: Estimated from GPS and wind, less accurate
     * 
     * @note Wind estimate requires GPS aiding and sufficient vehicle velocity to be observable
     * @note Sideslip (vel.y) should be near zero for coordinated flight
     * @note TAS affected by air density - higher TAS than IAS at altitude
     * 
     * @warning Returns false if wind not observable (low speed, no GPS, no airspeed sensor)
     * @warning Synthetic airspeed less accurate than sensor-based measurement
     * @warning Body-frame airspeed rotates with vehicle - convert to NED if needed
     * 
     * @see getWind() to get wind velocity estimate in NED frame
     * @see getVelNED() to get ground-relative velocity
     */
    bool getAirSpdVec(Vector3f &vel) const;

    /**
     * @brief Get rate of change of vertical position (vertical velocity)
     * @return Vertical velocity (m/s) in down direction, positive = descending
     * 
     * @details Returns the time derivative of down position (dPosD/dt), which represents
     * vertical velocity kinematically consistent with the EKF down position state.
     * 
     * This differs from vel.z returned by getVelNED() because:
     * - getPosDownDerivative(): Kinematically consistent with position state
     * - getVelNED().z: Velocity state which may temporarily differ during corrections
     * 
     * Kinematic consistency means:
     * - Integrating this derivative reproduces the EKF position
     * - Accounts for position corrections and resets
     * - No discontinuities when height datum reset or lane switch occurs
     * 
     * The difference arises because:
     * - EKF maintains separate position and velocity states
     * - Measurement updates can correct position more than velocity
     * - Height corrections cause temporary mismatch
     * - This function provides the position-derived velocity
     * 
     * Typical Use Cases:
     * - Altitude hold controllers preferring position-consistent velocity
     * - Variometer displays (rate of climb indicators)
     * - Terrain following where position tracking is critical
     * - Applications requiring strict kinematic consistency
     * 
     * Sign Convention:
     * - Positive = descending (moving down, increasing down position)
     * - Negative = climbing (moving up, decreasing down position)
     * - Zero = holding altitude
     * 
     * @note Matches NED convention: positive down velocity
     * @note Updated at EKF update rate (typically 100-400 Hz)
     * @note Includes effects of height resets and corrections
     * 
     * @warning May have slightly more noise than getVelNED().z velocity state
     * @warning Fluctuates when height corrections applied by filter
     * 
     * @see getPosD() for down position
     * @see getVelNED() for velocity state (may differ slightly)
     */
    float getPosDownDerivative() const;

    /**
     * @brief Get gyro bias estimates in body frame
     * @param[out] gyroBias Gyro bias vector (rad/s) in body frame
     * 
     * @details Returns the estimated gyroscope bias from the primary EKF core.
     * Gyro bias represents the constant offset error in gyro measurements.
     * 
     * Bias Estimation:
     * - Learned by EKF during flight through comparison with attitude from other sensors
     * - Magnetometer provides yaw reference for yaw bias observability
     * - GPS velocity provides roll/pitch reference for roll/pitch bias observability
     * - Convergence typically requires 30-60 seconds of flight with good aiding
     * 
     * Coordinate Frame (Body Frame):
     * - gyroBias.x = Roll rate bias (rad/s) about body X-axis (forward)
     * - gyroBias.y = Pitch rate bias (rad/s) about body Y-axis (right)
     * - gyroBias.z = Yaw rate bias (rad/s) about body Z-axis (down)
     * 
     * Bias Characteristics:
     * - Magnitude typically 0.001-0.05 rad/s (0.05-3 deg/s) for consumer IMUs
     * - Varies with temperature, aging, and vibration
     * - More observable when vehicle maneuvering vs hovering/straight flight
     * - Uncertainty increases when dead-reckoning (no aiding)
     * 
     * Typical Values by IMU Quality:
     * - Consumer IMU (MPU6000): 0.01-0.05 rad/s (0.5-3 deg/s)
     * - Mid-grade IMU (ICM20689): 0.005-0.02 rad/s (0.3-1 deg/s)
     * - Tactical IMU (ADIS16xxx): 0.001-0.005 rad/s (0.05-0.3 deg/s)
     * 
     * @note Bias estimates saved between flights if EK2_GBIAS_P_NSE properly tuned
     * @note resetGyroBias() zeroes estimates (used after IMU calibration)
     * @note Bias observability requires sufficient maneuvering and aiding
     * @note Process noise EK2_GBIAS_P_NSE controls learning rate (default 1e-6 rad/s)
     * 
     * @warning Large bias (>0.1 rad/s) may indicate IMU failure or miscalibration
     * @warning Bias estimates less accurate during dead-reckoning or poor aiding
     * @warning Do not directly apply these biases - EKF already compensates internally
     * 
     * @see resetGyroBias() to zero bias estimates
     * @see getGyroBiasVariances() to check estimation uncertainty
     */
    void getGyroBias(Vector3f &gyroBias) const;

    /**
     * @brief Reset gyro bias estimates to zero
     * 
     * @details Zeros the gyro bias states in all EKF cores and resets bias uncertainties
     * to initial values. This forces the filter to re-learn biases from scratch.
     * 
     * When to use:
     * - After performing IMU factory calibration (ACCEL_CAL, gyro calibration)
     * - When bias estimates appear incorrect (sudden large bias changes)
     * - After IMU replacement or hardware change
     * - When transitioning from one flight environment to drastically different one
     * 
     * Effects:
     * - All core gyro bias states set to zero
     * - Bias covariances reset to initial uncertainty (typically 0.1-1 rad²/s²)
     * - Filter will re-converge over next 30-60 seconds of flight
     * - Temporary increase in attitude estimation error until biases re-learned
     * 
     * Not recommended during:
     * - Active flight (causes temporary attitude errors)
     * - When current biases are reasonable and stable
     * - Immediately before arming (wait for re-convergence)
     * 
     * @note Should only be called on ground or in stable flight
     * @note Consider resetGyroBias() as part of IMU calibration workflow
     * @note Filter will automatically re-learn biases given sufficient maneuvering and aiding
     * 
     * @warning Causes temporary degradation of attitude estimate (5-30 seconds)
     * @warning Do not call during aggressive flight - may cause control issues
     * @warning Not necessary during normal operation - filter adapts to slow bias changes
     * 
     * @see getGyroBias() to read current bias estimates
     */
    void resetGyroBias(void);

    /**
     * @brief Reset height datum to current altitude
     * @return true if reset performed successfully, false if reset not allowed
     * 
     * @details Resets the EKF down position to zero while preserving absolute altitude.
     * Achieved by adjusting the EKF origin height so that (EKF height + origin height)
     * remains constant.
     * 
     * Reset Sequence:
     * 1. Read current EKF down position: posD_old
     * 2. Set EKF down position state to zero: posD = 0
     * 3. Adjust origin height: origin_height += posD_old
     * 4. Result: Absolute altitude unchanged, but EKF relative position reset
     * 
     * Use Cases:
     * - Reset accumulated barometer drift on long flights
     * - Set current altitude as new reference point
     * - Clear large down position values for numerical precision
     * - Synchronize with new altitude reference
     * 
     * Restrictions (returns false if):
     * - Currently using rangefinder as primary height source (EK2_ALT_SOURCE=1 or
     *   EK2_RNG_USE_HGT active) - rangefinder height is relative, reset breaks this
     * - Filter not initialized or unhealthy
     * - No valid height estimate available
     * 
     * Effects:
     * - Down position getPosD() changes to ~0
     * - Absolute altitude (MSL, AGL) unchanged
     * - Height variance may be reset to reflect new reference
     * - Logged position data will show discontinuity
     * 
     * @note Does NOT affect horizontal position (posNE unchanged)
     * @note Does NOT affect vehicle's actual altitude - only internal reference
     * @note Barometer offset adjusted to maintain consistency
     * @note Controllers see position reset via pos_down_reset_data structure
     * 
     * @warning Returns false when rangefinder is primary height source
     * @warning Causes logged position discontinuity (visible in plots)
     * @warning Should only be called when vehicle is stable (not during aggressive maneuvering)
     * 
     * @see getPosD() to read current down position
     * @see getLastPosDownReset() to query reset details for control compensation
     */
    bool resetHeightDatum(void);

    /**
     * @brief Get optical flow velocity control limits
     * @param[out] ekfGndSpdLimit Horizontal speed limit (m/s) imposed by optical flow
     * @param[out] ekfNavVelGainScaler Velocity gain scale factor (0-1) for height-dependent noise
     * 
     * @details When using optical flow for velocity aiding, this function returns control
     * constraints to account for flow sensor limitations and height-dependent measurement noise.
     * 
     * Ground Speed Limit (ekfGndSpdLimit):
     * - Maximum safe horizontal velocity when relying on optical flow
     * - Computed from: flow sensor max rate × height above ground
     * - Typical flow sensors: max 5-10 rad/s
     * - Example: At 2m height with 10 rad/s sensor: limit = 20 m/s
     * - Used by velocity controllers to limit commanded velocity
     * - Prevents exceeding flow sensor field-of-view or rate limits
     * 
     * Navigation Velocity Gain Scaler (ekfNavVelGainScaler):
     * - Scale factor applied to velocity control gains (range 0.0 to 1.0)
     * - Compensates for increased velocity noise at higher altitudes with optical flow
     * - At low altitude (good flow): scaler ≈ 1.0 (full gains)
     * - At high altitude (poor flow): scaler < 1.0 (reduced gains)
     * - Reduces control aggressiveness as flow measurement quality degrades
     * - Computed from: sqrt(height_variance / reference_height_variance)
     * 
     * Typical Usage:
     * ```cpp
     * float spd_limit, gain_scaler;
     * ekf.getEkfControlLimits(spd_limit, gain_scaler);
     * 
     * // Limit commanded velocity
     * desired_vel_ne.limit_length(spd_limit);
     * 
     * // Scale velocity controller gains
     * pos_control.set_vel_gain_scaler(gain_scaler);
     * ```
     * 
     * @note Only applies when optical flow is being used for velocity aiding
     * @note Returns conservative limits (1.0 m/s, 1.0) if flow not active or data invalid
     * @note Height from terrain estimator or rangefinder, not barometric height
     * @note Multicopters especially benefit from gain scaling at varying heights
     * 
     * @warning Exceeding speed limit may cause flow sensor saturation and loss of velocity estimate
     * @warning Ignoring gain scaler may cause oscillations or loss of control at high altitude
     * @warning Limits only valid when optical flow is primary velocity source
     * 
     * @see writeOptFlowMeas() for optical flow data input
     * @see EK2_FLOW_USE parameter to enable flow fusion
     */
    void getEkfControlLimits(float &ekfGndSpdLimit, float &ekfNavVelGainScaler) const;

    /**
     * @brief Get Z-axis accelerometer bias estimate
     * @param[out] zbias Accelerometer Z-axis bias (m/s²) in body frame (down axis)
     * 
     * @details Returns the estimated accelerometer Z-axis bias from the primary EKF core.
     * Z-axis bias represents constant offset error in vertical accelerometer measurement.
     * 
     * Why Only Z-axis:
     * - EKF2 estimates only Z-axis accel bias (simplified model vs full 3-axis)
     * - Z-axis most critical for altitude hold and vertical velocity
     * - Horizontal accel biases less observable and less impact on navigation
     * - Full 3-axis bias estimation available in EKF3
     * 
     * Bias Estimation:
     * - Learned through comparison of IMU vertical accel with height rate of change
     * - Requires good barometer or GPS height aiding for observability
     * - Convergence typically requires 30-120 seconds of varied vertical motion
     * - More observable during altitude changes than level flight
     * 
     * Coordinate Frame:
     * - Body frame down axis (Z-axis, positive = down)
     * - zbias positive = accelerometer reading too high (measures excess downward accel)
     * - zbias negative = accelerometer reading too low (measures insufficient downward accel)
     * 
     * Typical Values:
     * - Consumer IMU: 0.05-0.5 m/s² (0.005-0.05 g)
     * - Mid-grade IMU: 0.01-0.1 m/s² (0.001-0.01 g)
     * - Varies with temperature, vibration, and IMU orientation
     * 
     * @note Bias already compensated internally by EKF - do not directly apply
     * @note Process noise EK2_ABIAS_P_NSE controls learning rate (default 5e-3 m/s²)
     * @note Horizontal accel biases (X, Y) not estimated by EKF2 (use EKF3 for full 3-axis)
     * @note Used primarily for diagnostics and IMU health monitoring
     * 
     * @warning Large bias (>1 m/s²) may indicate IMU failure, mounting issue, or calibration error
     * @warning Bias estimate less accurate during dead-reckoning without height aiding
     * 
     * @see getGyroBias() for gyro bias estimates
     */
    void getAccelZBias(float &zbias) const;

    /**
     * @brief Get wind velocity estimate in NED frame
     * @param[out] wind Wind velocity vector (m/s) in NED frame
     * 
     * @details Returns the estimated wind velocity from the primary EKF core.
     * Wind represents the velocity of the airmass relative to the ground.
     * 
     * Wind Estimation:
     * - Learned by comparing airspeed (if available) with GPS ground velocity
     * - Requires GPS aiding and either airspeed sensor or sufficient vehicle motion
     * - Convergence typically requires 30-60 seconds of flight with varied headings
     * - More observable during maneuvering (turns, speed changes) than straight flight
     * 
     * Coordinate Frame (NED):
     * - wind.x = North wind component (m/s, positive = air moving north)
     * - wind.y = East wind component (m/s, positive = air moving east)
     * - wind.z = Down wind component (m/s, typically ~0 for horizontal wind)
     * 
     * Wind Convention:
     * - Positive wind component = air moving in positive axis direction
     * - Example: wind.x = 5.0 means 5 m/s wind from south (air moving north)
     * - Example: wind.y = -3.0 means 3 m/s wind from east (air moving west)
     * - Meteorological convention (wind FROM direction) is opposite
     * 
     * Wind Observability:
     * - Best: Airspeed sensor + GPS + varied headings
     * - Good: GPS + high-speed flight + varied headings (infers wind from groundspeed variation)
     * - Poor: Low speed, straight flight, no airspeed sensor
     * - None: No GPS (dead-reckoning), hover without airspeed sensor
     * 
     * Typical Wind Estimates:
     * - Accuracy: 0.5-2 m/s with airspeed sensor, 1-5 m/s without
     * - Convergence time: 30-60 seconds with varied flight
     * - Range: -50 to +50 m/s practical limits
     * - Down component (wind.z) typically near zero unless significant updraft/downdraft
     * 
     * Applications:
     * - Fixed-wing: Wind compensation for navigation and energy management
     * - Multicopter: Forward flight efficiency, weathervane compensation
     * - Mission planning: Expected flight time, energy consumption
     * - Ground station: Wind display, drift compensation
     * 
     * @note Wind state not estimated without GPS or external navigation aiding
     * @note Process noise EK2_WIND_P_NSE controls adaptation rate (default 0.1 m/s²)
     * @note EK2_WIND_PSCL parameter scales wind process noise with height rate
     * @note Vertical wind (wind.z) estimated but typically small and less reliable
     * 
     * @warning Wind estimate degrades without GPS or during prolonged straight flight
     * @warning Requires varied headings for wind direction observability
     * @warning Sudden large wind changes may take 10-30 seconds to converge
     * 
     * @see getAirSpdVec() for true airspeed in body frame
     * @see getVelNED() for ground velocity (includes wind effect)
     */
    void getWind(Vector3f &wind) const;

    /**
     * @brief Get earth magnetic field estimate in NED frame
     * @param[out] magNED Earth magnetic field vector (gauss/1000) in NED frame
     * 
     * @details Returns the estimated earth magnetic field from the primary EKF core.
     * Earth field represents the geomagnetic field at the vehicle's location in NED frame.
     * 
     * Earth Magnetic Field Components:
     * - magNED.x = North component (gauss/1000), typically largest component in NH
     * - magNED.y = East component (gauss/1000), small unless near magnetic poles
     * - magNED.z = Down component (gauss/1000), positive = downward field (NH), negative (SH)
     * 
     * Field Characteristics:
     * - Total magnitude typically 0.2-0.6 gauss (200-600 milligauss) depending on latitude
     * - Inclination angle: steep downward in Northern Hemisphere, steep upward in Southern Hemisphere
     * - Declination: Offset between true north and magnetic north (depends on location)
     * 
     * Field Learning:
     * - Initialized from World Magnetic Model (WMM) based on GPS position and date
     * - Refined during flight through magnetometer fusion
     * - Learns local deviations from WMM (magnetic anomalies, iron deposits, etc.)
     * - Controlled by in-flight mag calibration mode (EK2_MAG_CAL parameter)
     * 
     * Learning Modes (EK2_MAG_CAL):
     * - 0: Never learn (use WMM only)
     * - 1: Learn when flying
     * - 2: Learn when moving
     * - 3: Always learn
     * - 4: Learn after yaw alignment
     * - 5: Learn during initial yaw alignment
     * 
     * Field Variance Limiting:
     * - EK2_MAG_EF_LIM parameter limits learned field deviation from WMM
     * - Prevents learning incorrect fields due to local magnetic interference
     * - Typical limit: 50 milligauss deviation from WMM
     * 
     * Applications:
     * - Validate magnetometer calibration (compare measured vs learned earth field)
     * - Detect magnetic anomalies (sudden changes in learned earth field)
     * - Check compass health (earth field should be stable except during magnetic storms)
     * - Declination verification (compare learned vs WMM declination)
     * 
     * Units Conversion:
     * - Function returns gauss/1000, so multiply by 1000 to get milligauss
     * - Example: magNED.x = 0.25 means 250 milligauss north component
     * - Total field magnitude: sqrt(magNED.x² + magNED.y² + magNED.z²)
     * 
     * @note Earth field is location-dependent, changes as vehicle moves large distances
     * @note Declination computed from: atan2(magNED.y, magNED.x)
     * @note Inclination computed from: atan2(magNED.z, sqrt(magNED.x² + magNED.y²))
     * @note Field learning controlled by EK2_MAG_CAL and EK2_MAG_EF_LIM parameters
     * 
     * @warning Large deviation from WMM may indicate magnetic interference or anomaly
     * @warning Rapid changes in learned earth field may indicate moving magnetic sources nearby
     * @warning Do not use learned field for compass calibration - calibrate magnetometer separately
     * 
     * @see getMagXYZ() for body magnetic field offsets (hard iron)
     * @see EK2_MAG_CAL parameter for in-flight calibration mode
     * @see EK2_MAG_EF_LIM parameter for field learning limits
     */
    void getMagNED(Vector3f &magNED) const;

    /**
     * @brief Get body magnetic field offset estimates in body frame
     * @param[out] magXYZ Body magnetic field offset vector (gauss/1000) in body frame
     * 
     * @details Returns the estimated body magnetic field offsets (hard iron compensation)
     * from the primary EKF core. Body field represents magnetic disturbances fixed to
     * the vehicle frame.
     * 
     * Hard Iron vs Soft Iron:
     * - Hard iron: Constant magnetic field offsets in body frame (estimated by EKF)
     * - Soft iron: Field distortions depending on orientation (NOT estimated by EKF2)
     * - EKF2 estimates only hard iron offsets (constant 3D offset vector)
     * - Pre-flight compass calibration typically handles both hard and soft iron
     * 
     * Coordinate Frame (Body Frame):
     * - magXYZ.x = Forward axis offset (gauss/1000)
     * - magXYZ.y = Right axis offset (gauss/1000)
     * - magXYZ.z = Down axis offset (gauss/1000)
     * 
     * Sources of Body Magnetic Field:
     * - Vehicle ferromagnetic materials (steel frame, motors, ESCs)
     * - DC currents in power wiring (create magnetic fields)
     * - Permanent magnets in motors, speakers, actuators
     * - Battery current (varies with throttle, accounted for by motor compensation)
     * 
     * Body Field Learning:
     * - Learned in-flight when EK2_MAG_CAL parameter enables learning
     * - Requires varied vehicle orientations for full 3D offset observability
     * - Typically converges in first 30-60 seconds of flight with maneuvering
     * - Complements pre-flight compass calibration
     * 
     * Learning Modes (EK2_MAG_CAL):
     * - 0: Never learn (use pre-flight calibration only)
     * - 1: Learn when flying (recommended for most vehicles)
     * - 2: Learn when moving (rovers, ground vehicles)
     * - 3: Always learn (can learn on ground, but risks interference)
     * - 4: Learn after yaw alignment
     * - 5: Learn during initial yaw alignment
     * 
     * Typical Offset Magnitudes:
     * - Well-calibrated vehicle: 10-100 milligauss (0.01-0.1 gauss)
     * - Poorly calibrated or high interference: 100-500 milligauss (0.1-0.5 gauss)
     * - Severe interference: >500 milligauss (>0.5 gauss) - check vehicle magnetic environment
     * 
     * Applications:
     * - Monitor magnetic interference levels
     * - Validate compass calibration quality
     * - Detect changes in vehicle magnetic environment (new payloads, wiring changes)
     * - Complement pre-flight compass calibration with in-flight refinement
     * 
     * Units Conversion:
     * - Function returns gauss/1000, multiply by 1000 for milligauss
     * - Example: magXYZ.x = 0.15 means 150 milligauss forward offset
     * - Total offset magnitude: sqrt(magXYZ.x² + magXYZ.y² + magXYZ.z²)
     * 
     * @note Pre-flight compass calibration should handle bulk of hard iron compensation
     * @note In-flight learning provides refinement and adapts to configuration changes
     * @note Body field estimates are vehicle-frame specific, change if sensors moved
     * @note Process noise EK2_MAG_B_NSE controls learning rate (default 1e-4 gauss/s)
     * 
     * @warning Large body field offsets (>0.5 gauss) indicate strong magnetic interference
     * @warning Rapidly changing body field may indicate electrical interference or loose sensor
     * @warning Do not rely solely on in-flight learning - always perform pre-flight calibration
     * @warning High interference degrades compass accuracy and yaw estimation
     * 
     * @see getMagNED() for earth magnetic field estimate
     * @see EK2_MAG_CAL parameter for in-flight calibration mode
     * @see Pre-arm compass calibration (COMPASS_CAL) for initial hard/soft iron compensation
     */
    void getMagXYZ(Vector3f &magXYZ) const;

    /**
     * @brief Get estimated magnetometer offsets for a specific compass
     * @param[in] mag_idx Magnetometer index (0, 1, 2, etc.)
     * @param[out] magOffsets Estimated magnetometer offsets (gauss/1000) in sensor frame
     * @return true if offsets are valid and available, false otherwise
     * 
     * @details Returns the learned body magnetic field offsets (hard iron compensation)
     * for a specific compass sensor. Unlike getMagXYZ() which returns the primary core's
     * body field estimate, this function allows querying offsets for individual compasses.
     * 
     * Offset Sources:
     * - Learned by EKF during flight when in-flight mag calibration enabled (EK2_MAG_CAL)
     * - Represents hard iron offsets in the sensor's own reference frame
     * - Different from getMagXYZ() which is in vehicle body frame
     * - Can be used to update compass offsets or validate calibration
     * 
     * Validity Conditions (returns false if):
     * - Specified mag_idx not available or out of range
     * - Filter not initialized or not learning magnetometer offsets
     * - Insufficient flight time to learn offsets reliably
     * - In-flight calibration disabled (EK2_MAG_CAL = 0)
     * 
     * @note Requires EK2_MAG_CAL parameter to be non-zero for offsets to be learned
     * @note Offsets in sensor frame, not body frame (different if compass externally mounted)
     * @note Used primarily for compass calibration validation and diagnostics
     * 
     * @warning Returns false if compass not being used by EKF2
     * @warning Learned offsets may not be accurate if insufficient maneuvering during flight
     * 
     * @see getMagXYZ() for primary core body magnetic field estimate
     * @see EK2_MAG_CAL parameter for in-flight calibration mode
     */
    bool getMagOffsets(uint8_t mag_idx, Vector3f &magOffsets) const;

    /**
     * @brief Get vehicle position in latitude, longitude, altitude (WGS-84)
     * @param[out] loc Location structure with lat/lon/alt populated
     * @return true if position available (calculated or raw GPS), false if unavailable
     * 
     * @details Returns the vehicle's global position in WGS-84 coordinate system.
     * Combines EKF NED position relative to origin with origin lat/lon/alt to produce
     * absolute global position.
     * 
     * Position Sources (priority order):
     * 1. EKF calculated position: NED position + origin → lat/lon/alt
     * 2. Raw GPS measurement: If EKF position not valid, returns last GPS fix
     * 3. No position: Returns false if neither available
     * 
     * Coordinate System:
     * - Latitude: Decimal degrees, positive = north of equator, negative = south
     * - Longitude: Decimal degrees, positive = east of prime meridian, negative = west
     * - Altitude: Meters above WGS-84 ellipsoid (or MSL depending on GPS)
     * 
     * Data Quality:
     * - Return value indicates availability, NOT quality
     * - Always check getFilterStatus() for data quality before using for flight control
     * - Raw GPS fallback may have degraded accuracy (no EKF filtering)
     * 
     * Usage Scenarios:
     * - Ground station display: Show vehicle position on map
     * - Telemetry logging: Record flight path
     * - Mission execution: Check if at target waypoint
     * - Geofencing: Verify vehicle within allowed area
     * 
     * Location Structure Fields:
     * - loc.lat: Latitude in degrees × 1e7 (int32_t, ArduPilot standard scaling)
     * - loc.lng: Longitude in degrees × 1e7 (int32_t)
     * - loc.alt: Altitude in centimeters relative to origin or MSL (int32_t)
     * - loc.flags: Altitude reference frame flags
     * 
     * @note Returns raw GPS if EKF position not converged or healthy
     * @note Altitude reference frame may be MSL, ellipsoid, or relative depending on GPS
     * @note Position derived from getPosNE/D() + origin from getOriginLLH()
     * @note Does NOT guarantee position is valid for flight control - check getFilterStatus()
     * 
     * @warning Return true does NOT mean position is suitable for flight control
     * @warning Must call getFilterStatus() to check nav_filter_status flags before using for control
     * @warning Raw GPS fallback has no filtering - may have large errors
     * @warning Origin must be set (via GPS lock or setOriginLLH) for calculated position
     * 
     * @see getOriginLLH() to get NED origin position
     * @see getPosNE() and getPosD() for NED position relative to origin
     * @see getFilterStatus() for detailed position validity assessment
     */
    bool getLLH(Location &loc) const;

    /**
     * @brief Get the NED origin position in latitude, longitude, altitude
     * @param[out] loc Location structure with origin lat/lon/alt
     * @return true if origin has been set, false if not yet initialized
     * 
     * @details Returns the reference position (origin) used by the EKF for NED frame.
     * All NED positions from getPosNE() and getPosD() are relative to this origin.
     * 
     * Origin Setting Methods:
     * 1. Automatic: Set at first GPS lock position (most common)
     * 2. Manual: Via setOriginLLH() or MAVLink SET_GPS_GLOBAL_ORIGIN command
     * 3. External nav: Provided by visual odometry or motion capture system
     * 
     * Origin Characteristics:
     * - Defines NED frame orientation: North and East axes aligned with WGS-84 N/E at origin
     * - Defines altitude reference: Height datum for EKF down position
     * - Typically constant throughout flight (but can be changed if vehicle disarmed)
     * - Preserved across resetHeightDatum() calls (altitude reference adjusted internally)
     * 
     * NED Frame Definition:
     * - North axis: Points toward true north (not magnetic north) at origin location
     * - East axis: Points toward true east at origin location
     * - Down axis: Points toward earth center (perpendicular to WGS-84 ellipsoid)
     * - Origin defines zero position: getPosNE() = (0,0) and getPosD() = 0 at origin
     * 
     * Location Structure Fields:
     * - loc.lat: Origin latitude in degrees × 1e7 (int32_t)
     * - loc.lng: Origin longitude in degrees × 1e7 (int32_t)
     * - loc.alt: Origin altitude in centimeters MSL or ellipsoid (int32_t)
     * - loc.flags: Altitude reference frame flags
     * 
     * Typical Usage:
     * - Convert NED position to global lat/lon/alt for telemetry
     * - Initialize external navigation system with EKF origin
     * - Verify origin matches expected mission start location
     * - Geofence setup relative to launch location
     * 
     * Example Conversion NED → Global:
     * ```cpp
     * Location origin;
     * Vector2p posNE;
     * postype_t posD;
     * if (getOriginLLH(origin) && getPosNE(posNE) && getPosD(posD)) {
     *     Location vehicle_loc = origin;
     *     vehicle_loc.offset(posNE.x, posNE.y);  // Offset north and east
     *     vehicle_loc.alt += -posD * 100;        // Adjust altitude (posD in m, alt in cm)
     * }
     * ```
     * 
     * @note Returns false before first GPS lock or before setOriginLLH() called
     * @note Origin can only be changed when vehicle disarmed (safety feature)
     * @note Origin altitude updated by resetHeightDatum() to maintain absolute altitude
     * @note All EKF position outputs are relative to this origin
     * 
     * @warning Origin changes during flight will cause discontinuity in logged data
     * @warning Changing origin requires vehicle to be disarmed for safety
     * @warning Do not confuse with home position (they may differ if origin set manually)
     * 
     * @see setOriginLLH() to manually set origin
     * @see getPosNE() and getPosD() for NED position relative to this origin
     * @see getLLH() to get current vehicle global position
     */
    bool getOriginLLH(Location &loc) const;

    /**
     * @brief Set the NED origin position manually
     * @param[in] loc Location structure with desired origin lat/lon/alt
     * @return true if origin set successfully, false if rejected
     * 
     * @details Manually sets the EKF NED frame origin to specified global position.
     * All subsequent NED position calculations will be relative to this origin.
     * 
     * Setting Origin:
     * - Defines the reference point for EKF NED coordinate frame
     * - North and East axes aligned with true north/east at this location
     * - Down axis perpendicular to WGS-84 ellipsoid
     * - EKF position states initialized relative to this origin
     * 
     * Rejection Conditions (returns false if):
     * - Vehicle is armed or in flight mode (safety lockout)
     * - Filter is actively running with existing origin (prevents mid-flight changes)
     * - Specified location is invalid (lat/lon out of range)
     * - EKF not initialized or in error state
     * 
     * Common Use Cases:
     * - Set known precise survey point as origin before flight
     * - Initialize indoor navigation with motion capture origin
     * - Synchronize multiple vehicles to common origin
     * - Resume mission from specific location after restart
     * - Testing with specific origin position for repeatability
     * 
     * Alternative Methods:
     * - Automatic: Origin set at first GPS lock (most common, no action needed)
     * - MAVLink: Send SET_GPS_GLOBAL_ORIGIN message from ground station
     * - External nav: Origin provided by visual odometry system
     * 
     * Location Structure Fields:
     * - loc.lat: Desired origin latitude in degrees × 1e7 (int32_t)
     * - loc.lng: Desired origin longitude in degrees × 1e7 (int32_t)
     * - loc.alt: Desired origin altitude in centimeters MSL or ellipsoid (int32_t)
     * - loc.flags: Altitude reference frame flags (MSL vs ellipsoid)
     * 
     * Effects of Setting Origin:
     * - Current EKF NED position recomputed relative to new origin
     * - May cause discontinuity in logged position data
     * - Home position may differ from origin if set separately
     * - Geofence calculations will use new origin as reference
     * 
     * Example Usage:
     * ```cpp
     * Location survey_point;
     * survey_point.lat = 37.7749 * 1e7;    // San Francisco
     * survey_point.lng = -122.4194 * 1e7;
     * survey_point.alt = 5000;              // 50 meters MSL in cm
     * 
     * if (!hal.util->get_soft_armed()) {   // Only when disarmed
     *     if (!setOriginLLH(survey_point)) {
     *         GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Failed to set EKF origin");
     *     }
     * }
     * ```
     * 
     * @note Origin can only be set when vehicle disarmed (safety requirement)
     * @note If origin already set, must disarm to change (prevents accidental changes)
     * @note Origin persists across EKF resets but not across reboot
     * @note MAVLink SET_GPS_GLOBAL_ORIGIN provides same functionality via GCS
     * 
     * @warning Do NOT call during flight - will be rejected for safety
     * @warning Setting incorrect origin will cause navigation errors
     * @warning All waypoints and fence positions are relative to origin - setting origin affects mission
     * @warning Changing origin will cause discontinuity in position telemetry and logs
     * 
     * @see getOriginLLH() to read current origin
     * @see getPosNE() and getPosD() for NED position relative to origin
     */
    bool setOriginLLH(const Location &loc);

    /**
     * @brief Get estimated height above ground level (AGL)
     * @param[out] HAGL Height above ground level (meters), positive = above ground
     * @return true if HAGL estimate available, false if not being estimated
     * 
     * @details Returns the estimated height above the terrain or ground surface.
     * Requires terrain estimation from rangefinder or optical flow terrain estimator.
     * 
     * HAGL Sources:
     * - Rangefinder: Direct downward distance measurement corrected for tilt
     * - Optical flow terrain estimator: Single-state EKF estimating terrain offset
     * - Terrain database: Pre-loaded terrain model (if available and enabled)
     * - Combined: Fusion of available sources
     * 
     * HAGL vs Altitude:
     * - HAGL: Height above local terrain/obstacles (useful for low-altitude flight)
     * - Altitude (getPosD): Height above EKF origin datum (baro/GPS reference)
     * - HAGL accounts for terrain variation under vehicle
     * - Altitude is relative to fixed datum, not terrain
     * 
     * Return Conditions:
     * - true: Terrain estimation active and HAGL available
     * - false: No terrain estimation (no rangefinder, no flow terrain estimator)
     * 
     * Terrain Estimation Methods:
     * 1. Rangefinder: Most accurate, typical range 0.2-50m depending on sensor
     * 2. Optical flow: Terrain offset estimated from flow velocity and HAGL relationship
     * 3. Terrain database: Pre-loaded SRTM data, requires GPS, typical accuracy 10-30m
     * 
     * Typical Accuracy:
     * - Rangefinder: 0.05-0.5m (best, limited range)
     * - Optical flow terrain est: 0.5-2m (good, requires valid flow)
     * - Terrain database: 10-30m (approximate, depends on database resolution)
     * 
     * Applications:
     * - Terrain following: Maintain constant AGL for mapping, spraying
     * - Obstacle avoidance: Ensure minimum clearance above ground
     * - Auto-land: Final approach and flare guidance
     * - Low-altitude warning: Alert if too close to terrain
     * - Rangefinder status: Verify sensor providing valid data
     * 
     * Limitations:
     * - Rangefinder: Limited max range, affected by terrain reflectivity
     * - Optical flow: Requires valid flow data and reasonable tilt
     * - Terrain database: Requires GPS, limited resolution, outdated in some regions
     * - Over water: Difficult for rangefinders (poor reflection)
     * 
     * @note Rangefinder HAGL preferred when available and in range
     * @note HAGL continuously updated as vehicle flies over varying terrain
     * @note Optical flow terrain estimator provides backup when rangefinder out of range
     * @note HAGL critical for terrain following and low-altitude operations
     * 
     * @warning Returns false if no terrain estimation method active
     * @warning Rangefinder may give spurious readings over water or reflective surfaces
     * @warning Optical flow terrain estimate degrades at high altitude or poor lighting
     * @warning HAGL over moving objects (vehicles, water) may be unreliable
     * 
     * @see EK2_RNG_USE_HGT parameter to enable rangefinder for height
     * @see EK2_FLOW_USE parameter to enable optical flow terrain estimator
     */
    bool getHAGL(float &HAGL) const;

    /**
     * @brief Get Euler angles (roll, pitch, yaw) in radians
     * @param[out] eulers Euler angle vector (radians): eulers.x=roll, eulers.y=pitch, eulers.z=yaw
     * 
     * @details Returns the vehicle attitude as Euler angles from the primary EKF core.
     * Converted from the EKF's internal quaternion representation.
     * 
     * Euler Angle Definitions:
     * - Roll (eulers.x): Rotation about body X-axis (forward), positive = right wing down
     * - Pitch (eulers.y): Rotation about body Y-axis (right), positive = nose up
     * - Yaw (eulers.z): Rotation about body Z-axis (down), positive = nose right (clockwise from above)
     * 
     * Rotation Sequence:
     * - 3-2-1 Euler sequence (yaw, pitch, roll) applied in that order
     * - Also known as Tait-Bryan angles with Z-Y-X sequence
     * - Transforms from NED frame to body frame via: R = Rz(yaw) * Ry(pitch) * Rx(roll)
     * 
     * Angle Ranges and Conventions:
     * - Roll: -π to +π radians (-180° to +180°)
     * - Pitch: -π/2 to +π/2 radians (-90° to +90°) - gimbal lock at ±90°
     * - Yaw: 0 to 2π radians (0° to 360°), 0 = north, π/2 = east, π = south, 3π/2 = west
     * 
     * Gimbal Lock Warning:
     * - Euler angles have singularity (gimbal lock) at pitch = ±90°
     * - At gimbal lock, roll and yaw become ambiguous and cannot be distinguished
     * - For aerobatics or when pitch near ±90°, use getQuaternion() instead
     * - Controllers should avoid pitch near ±90° or use quaternion-based control
     * 
     * Coordinate Frame:
     * - Angles represent rotation from NED frame to body frame
     * - Yaw measured from true north (not magnetic north)
     * - Zero yaw = vehicle pointing north, zero pitch = level, zero roll = wings level
     * 
     * Typical Uses:
     * - Attitude display on ground station (artificial horizon)
     * - Simple attitude controllers (when pitch not near ±90°)
     * - Logging and telemetry (human-readable attitude)
     * - Stabilization modes (horizon, stabilize)
     * 
     * Conversion to Degrees:
     * ```cpp
     * Vector3f eulers;
     * getEulerAngles(eulers);
     * float roll_deg = degrees(eulers.x);
     * float pitch_deg = degrees(eulers.y);
     * float yaw_deg = degrees(eulers.z);
     * ```
     * 
     * @note Euler angles always populated even if filter unhealthy
     * @note Yaw from magnetometer fusion (if enabled) or GPS velocity direction
     * @note For aerobatics or high pitch angles, use getQuaternion() to avoid gimbal lock
     * @note Preferred for display and simple control, quaternions preferred for advanced control
     * 
     * @warning Gimbal lock at pitch = ±90° makes roll and yaw ambiguous
     * @warning Do not use for aerobatic control or when pitch may exceed ±85°
     * @warning Yaw is true heading (from north), not magnetic heading
     * @warning Discontinuity in yaw at 0/360° boundary - use quaternions for smooth interpolation
     * 
     * @see getQuaternion() for gimbal-lock-free attitude representation
     * @see getRotationBodyToNED() for direction cosine matrix (DCM)
     */
    void getEulerAngles(Vector3f &eulers) const;

    /**
     * @brief Get rotation matrix from body frame to NED frame
     * @param[out] mat 3×3 direction cosine matrix (DCM) representing rotation
     * 
     * @details Returns the direction cosine matrix (DCM) that transforms vectors
     * from body frame to NED frame. This is the transpose of the body-to-NED
     * rotation represented by the EKF attitude quaternion.
     * 
     * Rotation Matrix Properties:
     * - 3×3 orthogonal matrix (columns are orthonormal basis vectors)
     * - Determinant = +1 (proper rotation, no reflection)
     * - Transpose equals inverse: mat^T = mat^(-1)
     * - Columns represent body axes expressed in NED frame
     * - Rows represent NED axes expressed in body frame
     * 
     * Matrix Interpretation:
     * - mat.column(0) = body X-axis (forward) direction in NED frame
     * - mat.column(1) = body Y-axis (right) direction in NED frame
     * - mat.column(2) = body Z-axis (down) direction in NED frame
     * - mat.row(0) = NED North axis direction in body frame
     * - mat.row(1) = NED East axis direction in body frame
     * - mat.row(2) = NED Down axis direction in body frame
     * 
     * Vector Transformation:
     * - NED vector = mat * body_vector (transform body to NED)
     * - body_vector = mat^T * NED_vector (transform NED to body)
     * 
     * Typical Applications:
     * - Transform body-frame velocities/accelerations to NED frame
     * - Rotate sensor measurements to navigation frame
     * - Compute gravity vector in body frame: g_body = mat^T * [0, 0, g]
     * - Attitude control: Compute attitude error between desired and actual
     * 
     * Example Usage:
     * ```cpp
     * Matrix3f dcm;
     * getRotationBodyToNED(dcm);
     * 
     * // Transform body velocity to NED
     * Vector3f vel_body(10, 0, 0);  // 10 m/s forward
     * Vector3f vel_ned = dcm * vel_body;
     * 
     * // Get gravity in body frame
     * Vector3f g_ned(0, 0, GRAVITY_MSS);  // Down = positive
     * Vector3f g_body = dcm.transposed() * g_ned;
     * ```
     * 
     * Relationship to Euler Angles:
     * - DCM computed from quaternion (no gimbal lock)
     * - Can extract Euler angles: roll = atan2(mat[2][1], mat[2][2])
     * - Pitch = -asin(mat[2][0]), yaw = atan2(mat[1][0], mat[0][0])
     * 
     * @note DCM preferred over Euler angles for mathematical operations (no gimbal lock)
     * @note Matrix always valid even at ±90° pitch (unlike Euler angles)
     * @note Slightly more computation than quaternion but more intuitive for some operations
     * @note DCM = quaternion converted to matrix form
     * 
     * @warning Matrix orthogonality maintained by EKF quaternion normalization
     * @warning If using for repeated transformations, cache result to avoid recomputation
     * 
     * @see getQuaternion() for quaternion representation (most compact)
     * @see getEulerAngles() for human-readable angles
     */
    void getRotationBodyToNED(Matrix3f &mat) const;

    /**
     * @brief Get attitude quaternion for specified core instance
     * @param[in] instance Core instance index (0 to num_cores-1), or -1 for primary
     * @param[out] quat Quaternion representing rotation from NED to body frame
     * 
     * @details Returns the attitude quaternion from specified EKF core.
     * Unlike getQuaternion() which returns primary core only, this allows
     * querying attitude from any active core for diagnostics or comparison.
     * 
     * Quaternion Convention:
     * - quat.q1, quat.q2, quat.q3, quat.q4 = scalar-first format: [w, x, y, z]
     * - Unit quaternion: q1² + q2² + q3² + q4² = 1
     * - Represents rotation from NED frame to body frame
     * - Inverse rotation (body to NED): conjugate quaternion [q1, -q2, -q3, -q4]
     * 
     * Instance Selection:
     * - instance = 0, 1, 2, ...: Specific core index
     * - instance = -1: Primary core (same as getQuaternion())
     * - Out of range: Returns primary core quaternion
     * 
     * Applications:
     * - Compare attitudes between multiple cores for lane switching diagnostics
     * - Monitor core divergence (quaternion difference between cores)
     * - Validate core alignment before lane switch
     * - Debugging multi-IMU redundancy issues
     * 
     * Quaternion Distance Metric:
     * ```cpp
     * Quaternion q1, q2;
     * getQuaternionBodyToNED(0, q1);
     * getQuaternionBodyToNED(1, q2);
     * float angle_diff = 2.0 * acosf(fabsf(q1.q1*q2.q1 + q1.q2*q2.q2 + 
     *                                        q1.q3*q2.q3 + q1.q4*q2.q4));
     * // angle_diff is attitude difference in radians
     * ```
     * 
     * @note Instance parameter allows querying non-primary cores
     * @note Returns primary core if instance out of range
     * @note Quaternion automatically normalized by EKF (unit norm maintained)
     * @note Prefer getQuaternion() for primary core - simpler interface
     * 
     * @see getQuaternion() for primary core attitude (simpler, more commonly used)
     * @see getPrimaryCoreIndex() to determine which core is primary
     */
    void getQuaternionBodyToNED(int8_t instance, Quaternion &quat) const;

    /**
     * @brief Get attitude quaternion from primary core
     * @param[out] quat Quaternion representing rotation from NED to body frame
     * 
     * @details Returns the primary EKF core's attitude quaternion. This is the
     * preferred method for retrieving vehicle attitude - no gimbal lock and
     * compact representation.
     * 
     * Quaternion Format:
     * - Four components: quat[0]=w, quat[1]=x, quat[2]=y, quat[3]=z (scalar-first)
     * - Unit quaternion: w² + x² + y² + z² = 1 (normalized by EKF)
     * - Represents rotation from NED frame to body frame
     * - Empty quaternion [1, 0, 0, 0] = zero rotation (NED aligned with body)
     * 
     * Quaternion Advantages:
     * - No gimbal lock singularity (valid at all attitudes)
     * - Smooth interpolation (SLERP) for trajectory planning
     * - Compact representation (4 numbers vs 9 for DCM, 3 for Euler with singularity)
     * - Efficient composition of rotations
     * - Numerical stability in integration and updates
     * 
     * Rotating Vectors with Quaternion:
     * ```cpp
     * Quaternion q;
     * getQuaternion(q);
     * Vector3f body_vec(1, 0, 0);  // Forward in body frame
     * 
     * // Rotate body vector to NED (inverse rotation)
     * Matrix3f dcm;
     * q.rotation_matrix(dcm);
     * Vector3f ned_vec = dcm * body_vec;
     * 
     * // Or use quaternion rotate method
     * q.earth_to_body(ned_vec, body_vec);  // NED to body
     * q.body_to_earth(body_vec, ned_vec);  // Body to NED
     * ```
     * 
     * Converting to Euler Angles:
     * ```cpp
     * Quaternion q;
     * getQuaternion(q);
     * float roll, pitch, yaw;
     * q.to_euler(roll, pitch, yaw);
     * // roll, pitch, yaw in radians
     * ```
     * 
     * Applications:
     * - Advanced attitude control (model predictive control, quaternion feedback)
     * - Trajectory planning with smooth attitude interpolation
     * - Aerobatic flight control (no gimbal lock)
     * - Sensor fusion and estimation
     * - Attitude logging for later analysis
     * 
     * @note Quaternion automatically normalized by EKF (always unit norm)
     * @note Preferred over Euler angles for control and mathematical operations
     * @note Same result as getQuaternionBodyToNED(-1, quat)
     * @note Always returns valid quaternion even if filter unhealthy
     * 
     * @warning Quaternion sign ambiguity: q and -q represent same rotation
     * @warning When comparing quaternions, account for sign ambiguity (use absolute value of dot product)
     * 
     * @see getEulerAngles() for Euler angle representation (display/simple control)
     * @see getRotationBodyToNED() for DCM representation (matrix operations)
     * @see getQuaternionBodyToNED() to query specific core instance
     */
    void getQuaternion(Quaternion &quat) const;

    /**
     * @brief Get measurement innovations from primary core
     * @param[out] velInnov Velocity innovation vector (m/s) in NED frame
     * @param[out] posInnov Position innovation vector (m) in NED frame (N, E, Down)
     * @param[out] magInnov Magnetometer innovation vector (gauss/1000) in body frame
     * @param[out] tasInnov True airspeed innovation (m/s)
     * @param[out] yawInnov Yaw innovation (radians)
     * @return true if data available, false if filter not initialized
     * 
     * @details Returns the current measurement innovations (residuals) from the
     * primary EKF core. Innovations are the difference between actual sensor
     * measurements and EKF predicted measurements: innovation = measurement - prediction.
     * 
     * Innovation Definition:
     * - Innovation (residual) = z - h(x) where z = measurement, h(x) = predicted measurement
     * - Small innovations = measurements agree with predictions (good)
     * - Large innovations = measurement/prediction disagreement (potential fault or model error)
     * - Innovation magnitude compared to innovation variance for consistency checks
     * 
     * Velocity Innovation (velInnov):
     * - NED frame velocity residual (m/s)
     * - velInnov.x = GPS_vel_north - EKF_predicted_vel_north
     * - velInnov.y = GPS_vel_east - EKF_predicted_vel_east
     * - velInnov.z = GPS_vel_down - EKF_predicted_vel_down
     * - Typical magnitude: <0.5 m/s for healthy GPS, >2 m/s may indicate GPS issue
     * 
     * Position Innovation (posInnov):
     * - NED frame position residual (m)
     * - posInnov.x = GPS_pos_north - EKF_predicted_pos_north
     * - posInnov.y = GPS_pos_east - EKF_predicted_pos_east
     * - posInnov.z = Height_meas - EKF_predicted_height (baro or GPS)
     * - Typical magnitude: <5m for healthy GPS, >20m may indicate glitch
     * 
     * Magnetometer Innovation (magInnov):
     * - Body frame magnetic field residual (gauss/1000)
     * - magInnov.x = Mag_body_X - EKF_predicted_mag_body_X
     * - magInnov.y = Mag_body_Y - EKF_predicted_mag_body_Y
     * - magInnov.z = Mag_body_Z - EKF_predicted_mag_body_Z
     * - Typical magnitude: <0.05 gauss for healthy compass, >0.2 may indicate interference
     * 
     * Airspeed Innovation (tasInnov):
     * - True airspeed residual (m/s)
     * - tasInnov = Measured_TAS - EKF_predicted_TAS
     * - EKF predicts TAS from |velocity - wind|
     * - Typical magnitude: <2 m/s, large values indicate airspeed sensor or wind estimate error
     * 
     * Yaw Innovation (yawInnov):
     * - Yaw angle residual (radians)
     * - From magnetometer or GPS velocity yaw measurements
     * - Typical magnitude: <0.1 rad (5°), >0.3 rad (17°) may indicate mag anomaly or GPS issue
     * 
     * Using Innovations for Fault Detection:
     * ```cpp
     * Vector3f vel_innov, pos_innov, mag_innov;
     * float tas_innov, yaw_innov;
     * if (getInnovations(vel_innov, pos_innov, mag_innov, tas_innov, yaw_innov)) {
     *     // Check GPS health
     *     if (vel_innov.length() > 2.0) {
     *         GCS_SEND_TEXT(MAV_SEVERITY_WARNING, \"Large GPS velocity innovation\");
     *     }
     *     // Check compass health
     *     if (mag_innov.length() > 0.3) {
     *         GCS_SEND_TEXT(MAV_SEVERITY_WARNING, \"Large compass innovation - interference?\");
     *     }
     * }
     * ```
     * 
     * Relationship to Innovation Gates:
     * - Innovations normalized by innovation variance for gating: test_ratio = innov² / variance
     * - test_ratio compared to gates (EK2_VELNE_I_GATE, EK2_POSNE_I_GATE, etc.)
     * - Measurements rejected if test_ratio > gate²
     * 
     * @note Innovations logged in NKF3 message for detailed analysis
     * @note Persistently large innovations indicate sensor fault, calibration error, or model mismatch
     * @note Innovation magnitude depends on measurement noise and filter tuning
     * @note Returns false before filter initialization
     * 
     * @warning Large innovations may precede EKF failsafe or lane switch
     * @warning Sudden innovation spikes during maneuvers are normal (adaptive noise scaling compensates)
     * @warning Check innovation consistency over time, not just instantaneous values
     * 
     * @see getVariances() for innovation variances (expected innovation² magnitudes)
     * @see getFilterFaults() for detected fault conditions based on innovations
     * @see EK2_*_I_GATE parameters for innovation consistency test gates
     */
    bool getInnovations(Vector3f &velInnov, Vector3f &posInnov, Vector3f &magInnov, float &tasInnov, float &yawInnov) const;

    /**
     * @brief Get innovation variances and test ratios from primary core
     * @param[out] velVar Velocity innovation variance (m²/s²)
     * @param[out] posVar Position innovation variance (m²)
     * @param[out] hgtVar Height innovation variance (m²)
     * @param[out] magVar Magnetometer innovation variance vector (gauss²/1000²) per axis
     * @param[out] tasVar True airspeed innovation variance (m²/s²)
     * @param[out] offset Terrain altitude offset estimate (m) [for optical flow]
     * @return true if data available, false if filter not initialized
     * 
     * @details Returns the innovation variances from the primary EKF core.
     * Innovation variance S = H*P*H' + R is the expected squared magnitude of
     * innovations, used for consistency checks and measurement rejection.
     * 
     * Innovation Variance Definition:
     * - S = H * P * H' + R where:
     *   - H = measurement Jacobian (∂h/∂x)
     *   - P = state covariance matrix
     *   - R = measurement noise covariance
     * - Represents expected innovation² given current uncertainty
     * - Used to normalize innovations for chi-squared consistency test
     * 
     * Innovation Consistency Test:
     * - Test ratio τ = innovation² / variance
     * - τ follows chi-squared distribution if model correct
     * - Gate threshold typically 3-5 sigma: τ < gate²
     * - τ > gate² indicates outlier → reject measurement
     * 
     * Velocity Variance (velVar):
     * - Scalar representing typical velocity innovation variance (m²/s²)
     * - Combined from north, east, down components
     * - Larger during maneuvers (adaptive measurement noise scaling)
     * - Typical value: 0.05-0.5 m²/s² depending on GPS quality and maneuvers
     * 
     * Position Variance (posVar):
     * - Scalar representing typical horizontal position innovation variance (m²)
     * - Combined from north and east components
     * - Increases during GPS glitches or poor satellite geometry
     * - Typical value: 10-100 m² depending on GPS accuracy
     * 
     * Height Variance (hgtVar):
     * - Vertical position innovation variance (m²)
     * - From barometer, GPS altitude, or rangefinder
     * - Larger for GPS altitude than barometer
     * - Typical value: 1-25 m² depending on height source
     * 
     * Magnetometer Variance (magVar):
     * - Per-axis innovation variance (gauss²/1000²)
     * - magVar.x, magVar.y, magVar.z for body X, Y, Z axes
     * - Increases near magnetic anomalies or interference
     * - Typical value: 0.0025-0.01 gauss²/1000² per axis
     * 
     * Airspeed Variance (tasVar):
     * - True airspeed innovation variance (m²/s²)
     * - Scales with air density (EAS noise * EAS2TAS)²
     * - Larger at high altitude (lower air density)
     * - Typical value: 1-4 m²/s²
     * 
     * Terrain Offset (offset):
     * - Terrain altitude offset estimate (m) for optical flow
     * - offset.x = terrain offset relative to EKF origin down position
     * - offset.y = terrain offset variance (uncertainty)
     * - Used by optical flow terrain estimator for HAGL
     * 
     * Example Usage - Detecting Sensor Issues:
     * ```cpp
     * float vel_var, pos_var, hgt_var, tas_var;
     * Vector3f mag_var;
     * Vector2f terrain_offset;
     * 
     * if (getVariances(vel_var, pos_var, hgt_var, mag_var, tas_var, terrain_offset)) {
     *     // Check if variance growing (indicates divergence or poor observability)
     *     if (pos_var > 10000.0f) {  // > 100m standard deviation
     *         GCS_SEND_TEXT(MAV_SEVERITY_ERROR, \"EKF position variance very high\");
     *     }
     *     
     *     // Check mag variance for interference
     *     if (mag_var.length() > 0.05f) {
     *         GCS_SEND_TEXT(MAV_SEVERITY_WARNING, \"High mag variance - interference?\");
     *     }
     * }
     * ```
     * 
     * Variance Growth:
     * - Variances grow during dead-reckoning (no aiding)
     * - Variances reduce when measurements fused
     * - Unbounded growth indicates filter divergence
     * - Monitoring variance trends critical for fault detection
     * 
     * Relationship to Innovation Gates:
     * - Gate test: innovation² / variance < gate²
     * - Example: If velVar = 0.25 m²/s², gate = 500 (5 sigma)
     * - Accept if innovation² < 0.25 * (5)² = 6.25 m²/s²
     * - Reject if innovation > 2.5 m/s
     * 
     * @note Variances logged in NKF4 message for detailed analysis
     * @note Growing variances indicate filter divergence or loss of aiding
     * @note Variances used internally for Kalman gain calculation
     * @note Returns false before filter initialization
     * 
     * @warning Rapidly growing variance indicates filter divergence - may precede EKF failsafe
     * @warning Very small variance (<1e-6) may indicate numerical conditioning issues
     * @warning Variance alone doesn't indicate health - check innovations and filter status too
     * 
     * @see getInnovations() for actual innovation values
     * @see getFilterStatus() for comprehensive filter health assessment
     * @see EK2_*_I_GATE parameters for innovation gate thresholds
     */
    bool getVariances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar, Vector2f &offset) const;

    /**
     * @brief Check if compass should be used for navigation
     * @return true if compass is being used, false if compass disabled or not healthy
     * 
     * @details Returns whether the EKF is currently using magnetometer (compass)
     * measurements for yaw estimation. Public function to support ahrs.use_compass()
     * reporting to GCS and vehicle code.
     * 
     * Compass Usage Conditions (returns true if):
     * - Compass sensor available and healthy
     * - Compass enabled in parameters (COMPASS_ENABLE, COMPASS_USE)
     * - EKF not inhibiting compass due to detected faults
     * - Compass innovations passing consistency checks
     * - Not in compass-free flight mode
     * 
     * Compass Disabled Conditions (returns false if):
     * - No compass sensor configured (COMPASS_ENABLE = 0)
     * - Compass use disabled (COMPASS_USE = 0)
     * - Compass health check failed (no data, wrong orientation, etc.)
     * - Persistent large magnetometer innovations (magnetic anomaly detected)
     * - EKF switched to GPS velocity yaw or EKF-GSF yaw estimator
     * - Vehicle in compass-inhibited flight mode (COMPASS_USE = 0 during flight)
     * 
     * Yaw Sources When Compass Not Used:
     * - GPS velocity yaw: Yaw from GPS ground track (requires movement, GPS velocity >3 m/s typically)
     * - EKF-GSF yaw estimator: Emergency yaw estimate from IMU and GPS (magnetic anomaly recovery)
     * - Dead-reckoning: Gyro integration only (yaw drifts without external reference)
     * 
     * Applications:
     * - GCS display: Show pilot whether compass is being used
     * - Pre-flight checks: Verify compass operational before arming
     * - In-flight monitoring: Detect if compass disabled due to interference
     * - Failsafe logic: Trigger warnings if compass lost during flight
     * 
     * Example Usage:
     * ```cpp
     * if (!use_compass()) {
     *     if (hal.util->get_soft_armed()) {
     *         // Compass lost during flight - may indicate magnetic anomaly
     *         GCS_SEND_TEXT(MAV_SEVERITY_WARNING, \"Compass not used - relying on GPS yaw\");
     *     } else {
     *         // Pre-flight: compass not available
     *         GCS_SEND_TEXT(MAV_SEVERITY_ERROR, \"Compass not available for arming\");
     *     }
     * }
     * ```
     * 
     * Compass Health vs Usage:
     * - Compass may be healthy but not used (e.g., GPS yaw preferred in some modes)
     * - Compass may be used intermittently (e.g., only when vehicle stationary)
     * - use_compass() returns current usage status, not just health
     * 
     * @note This is queried by AHRS for ahrs.use_compass() reporting
     * @note Compass usage can change during flight (magnetic anomaly, mode changes)
     * @note GPS yaw requires sufficient velocity - compass needed when hovering or stationary
     * @note May return false even if compass sensor healthy (if not being fused)
     * 
     * @warning Compass critical for yaw observability when stationary or low speed
     * @warning Flying without compass in non-GPS modes risks yaw drift and position errors
     * @warning If compass disabled during flight, ensure GPS velocity sufficient for yaw
     * 
     * @see getMagNED() and getMagXYZ() for magnetometer field estimates
     * @see getInnovations() to check magnetometer innovations
     * @see EK2_MAG_CAL parameter for in-flight compass calibration settings
     */
    bool use_compass(void) const;

    // write the raw optical flow measurements
    // rawFlowQuality is a measured of quality between 0 and 255, with 255 being the best quality
    // rawFlowRates are the optical flow rates in rad/sec about the X and Y sensor axes.
    // rawGyroRates are the sensor rotation rates in rad/sec measured by the sensors internal gyro
    // The sign convention is that a RH physical rotation of the sensor about an axis produces both a positive flow and gyro rate
    // msecFlowMeas is the scheduler time in msec when the optical flow data was received from the sensor.
    // posOffset is the XYZ flow sensor position in the body frame in m
    // heightOverride is the fixed height of the sensor above ground in m, when on rover vehicles. 0 if not used
    void  writeOptFlowMeas(const uint8_t rawFlowQuality, const Vector2f &rawFlowRates, const Vector2f &rawGyroRates, const uint32_t msecFlowMeas, const Vector3f &posOffset, float heightOverride);

    // Set to true if the terrain underneath is stable enough to be used as a height reference
    // in combination with a range finder. Set to false if the terrain underneath the vehicle
    // cannot be used as a height reference. Use to prevent range finder operation otherwise
    // enabled by the combination of EK2_RNG_AID_HGT and EK2_RNG_USE_SPD parameters.
    void setTerrainHgtStable(bool val);

    /*
    return the filter fault status as a bitmasked integer for the specified instance
    An out of range instance (eg -1) returns data for the primary instance
     0 = quaternions are NaN
     1 = velocities are NaN
     2 = badly conditioned X magnetometer fusion
     3 = badly conditioned Y magnetometer fusion
     4 = badly conditioned Z magnetometer fusion
     5 = badly conditioned airspeed fusion
     6 = badly conditioned synthetic sideslip fusion
     7 = filter is not initialised
    */
    void  getFilterFaults(uint16_t &faults) const;

    /*
    return filter gps quality check status for the specified instance
    An out of range instance (eg -1) returns data for the primary instance
    */
    void  getFilterGpsStatus(nav_gps_status &faults) const;

    /*
    return filter status flags for the specified instance
    An out of range instance (eg -1) returns data for the primary instance
    */
    void  getFilterStatus(nav_filter_status &status) const;

    // send an EKF_STATUS_REPORT message to GCS
    void send_status_report(class GCS_MAVLINK &link) const;

    // provides the height limit to be observed by the control loops
    // returns false if no height limiting is required
    // this is needed to ensure the vehicle does not fly too high when using optical flow navigation
    bool getHeightControlLimit(float &height) const;

    // return the amount of yaw angle change (in radians) due to the last yaw angle reset or core selection switch
    // returns the time of the last yaw angle reset or 0 if no reset has ever occurred
    uint32_t getLastYawResetAngle(float &yawAngDelta);

    // return the amount of NE position change due to the last position reset in metres
    // returns the time of the last reset or 0 if no reset has ever occurred
    uint32_t getLastPosNorthEastReset(Vector2f &posDelta);

    // return the amount of NE velocity change due to the last velocity reset in metres/sec
    // returns the time of the last reset or 0 if no reset has ever occurred
    uint32_t getLastVelNorthEastReset(Vector2f &vel) const;

    // return the amount of vertical position change due to the last reset in metres
    // returns the time of the last reset or 0 if no reset has ever occurred
    uint32_t getLastPosDownReset(float &posDelta);

    // set and save the _baroAltNoise parameter
    void set_baro_alt_noise(float noise) { _baroAltNoise.set_and_save(noise); };

    // allow the enable flag to be set by Replay
    void set_enable(bool enable) { _enable.set_enable(enable); }

    // get the enable parameter
    bool get_enable(void) const { return bool(_enable.get()); }
    
    /*
     * Write position and quaternion data from an external navigation system
     *
     * pos        : position in the RH navigation frame. Frame is assumed to be NED (m)
     * quat       : quaternion desribing the rotation from navigation frame to body frame
     * posErr     : 1-sigma spherical position error (m)
     * angErr     : 1-sigma spherical angle error (rad)
     * timeStamp_ms : system time the measurement was taken, not the time it was received (mSec)
     * delay_ms   : average delay of external nav system measurements relative to inertial measurements
     * resetTime_ms : system time of the last position reset request (mSec)
     *
     * Sensor offsets are pulled directly from the AP_VisualOdom library
     *
    */
    void writeExtNavData(const Vector3f &pos, const Quaternion &quat, float posErr, float angErr, uint32_t timeStamp_ms, uint16_t delay_ms, uint32_t resetTime_ms);

    /*
     * Write velocity data from an external navigation system
     * vel : velocity in NED (m)
     * err : velocity error (m/s)
     * timeStamp_ms : system time the measurement was taken, not the time it was received (mSec)
     * delay_ms   : average delay of external nav system measurements relative to inertial measurements
     */
    void writeExtNavVelData(const Vector3f &vel, float err, uint32_t timeStamp_ms, uint16_t delay_ms);

    /*
      check if switching lanes will reduce the normalised
      innovations. This is called when the vehicle code is about to
      trigger an EKF failsafe, and it would like to avoid that by
      using a different EKF lane
     */
    void checkLaneSwitch(void);

    /*
      Request a reset of the EKF yaw. This is called when the vehicle code is about to
      trigger an EKF failsafe, and it would like to avoid that.
     */
    void requestYawReset(void);

    // write EKF information to on-board logs
    void Log_Write();

    // check if external navigation is being used for yaw observation
    bool isExtNavUsedForYaw(void) const;

    // check if configured to use GPS for horizontal position estimation
    bool configuredToUseGPSForPosXY(void) const;

    // Writes the default equivalent airspeed in m/s to be used in forward flight if a measured airspeed is required and not available.
    void writeDefaultAirSpeed(float airspeed);

    // get a yaw estimator instance
    const EKFGSF_yaw *get_yawEstimator(void) const;
    
private:
    uint8_t num_cores; // number of allocated cores
    uint8_t primary;   // current primary core
    NavEKF2_core *core = nullptr;
    bool core_malloc_failed;

    uint32_t _frameTimeUsec;        // time per IMU frame
    uint8_t  _framesPerPrediction;  // expected number of IMU frames per prediction

    // EKF Mavlink Tuneable Parameters
    AP_Int8  _enable;               // zero to disable EKF2
    AP_Float _gpsHorizVelNoise;     // GPS horizontal velocity measurement noise : m/s
    AP_Float _gpsVertVelNoise;      // GPS vertical velocity measurement noise : m/s
    AP_Float _gpsHorizPosNoise;     // GPS horizontal position measurement noise m
    AP_Float _baroAltNoise;         // Baro height measurement noise : m
    AP_Float _magNoise;             // magnetometer measurement noise : gauss
    AP_Float _easNoise;             // equivalent airspeed measurement noise : m/s
    AP_Float _windVelProcessNoise;  // wind velocity state process noise : m/s^2
    AP_Float _wndVarHgtRateScale;   // scale factor applied to wind process noise due to height rate
    AP_Float _magEarthProcessNoise; // Earth magnetic field process noise : gauss/sec
    AP_Float _magBodyProcessNoise;  // Body magnetic field process noise : gauss/sec
    AP_Float _gyrNoise;             // gyro process noise : rad/s
    AP_Float _accNoise;             // accelerometer process noise : m/s^2
    AP_Float _gyroBiasProcessNoise; // gyro bias state process noise : rad/s
    AP_Float _accelBiasProcessNoise;// accel bias state process noise : m/s^2
    AP_Int16 _hgtDelay_ms;          // effective average delay of Height measurements relative to inertial measurements (msec)
    AP_Int8  _fusionModeGPS;        // 0 = use 3D velocity, 1 = use 2D velocity, 2 = use no velocity, 3 = do not use GPS
    AP_Int16  _gpsVelInnovGate;     // Percentage number of standard deviations applied to GPS velocity innovation consistency check
    AP_Int16  _gpsPosInnovGate;     // Percentage number of standard deviations applied to GPS position innovation consistency check
    AP_Int16  _hgtInnovGate;        // Percentage number of standard deviations applied to height innovation consistency check
    AP_Int16  _magInnovGate;        // Percentage number of standard deviations applied to magnetometer innovation consistency check
    AP_Int16  _tasInnovGate;        // Percentage number of standard deviations applied to true airspeed innovation consistency check
    AP_Int8  _magCal;               // Sets activation condition for in-flight magnetometer calibration
    AP_Int8 _gpsGlitchRadiusMax;    // Maximum allowed discrepancy between inertial and GPS Horizontal position before GPS glitch is declared : m
    AP_Float _flowNoise;            // optical flow rate measurement noise
    AP_Int16  _flowInnovGate;       // Percentage number of standard deviations applied to optical flow innovation consistency check
    AP_Int8  _flowDelay_ms;         // effective average delay of optical flow measurements rel to IMU (msec)
    AP_Int16  _rngInnovGate;        // Percentage number of standard deviations applied to range finder innovation consistency check
    AP_Float _maxFlowRate;          // Maximum flow rate magnitude that will be accepted by the filter
    AP_Int8 _altSource;             // Primary alt source during optical flow navigation. 0 = use Baro, 1 = use range finder, 2 = use GPS, 3 = use Range Beacon
    AP_Float _gyroScaleProcessNoise;// gyro scale factor state process noise : 1/s
    AP_Float _rngNoise;             // Range finder noise : m
    AP_Int8 _gpsCheck;              // Bitmask controlling which preflight GPS checks are bypassed
    AP_Int8 _imuMask;               // Bitmask of IMUs to instantiate EKF2 for
    AP_Int16 _gpsCheckScaler;       // Percentage increase to be applied to GPS pre-flight accuracy and drift thresholds
    AP_Float _noaidHorizNoise;      // horizontal position measurement noise assumed when synthesised zero position measurements are used to constrain attitude drift : m
    AP_Float _yawNoise;             // magnetic yaw measurement noise : rad
    AP_Int16 _yawInnovGate;         // Percentage number of standard deviations applied to magnetic yaw innovation consistency check
    AP_Int8 _tauVelPosOutput;       // Time constant of output complementary filter : csec (centi-seconds)
    AP_Int8 _useRngSwHgt;           // Maximum valid range of the range finder as a percentage of the maximum range specified by the sensor driver
    AP_Float _terrGradMax;          // Maximum terrain gradient below the vehicle
    AP_Float _rngBcnNoise;          // Range beacon measurement noise (m)
    AP_Int16 _rngBcnInnovGate;      // Percentage number of standard deviations applied to range beacon innovation consistency check
    AP_Int8  _rngBcnDelay_ms;       // effective average delay of range beacon measurements rel to IMU (msec)
    AP_Float _useRngSwSpd;          // Maximum horizontal ground speed to use range finder as the primary height source (m/s)
    AP_Int8 _magMask;               // Bitmask forcng specific EKF core instances to use simple heading magnetometer fusion.
    AP_Int8 _originHgtMode;         // Bitmask controlling post alignment correction and reporting of the EKF origin height.
    AP_Int8 _flowUse;               // Controls if the optical flow data is fused into the main navigation estimator and/or the terrain estimator.
    AP_Int16 _mag_ef_limit;         // limit on difference between WMM tables and learned earth field.
    AP_Float _hrt_filt_freq;        // frequency of output observer height rate complementary filter in Hz
    AP_Int8 _gsfRunMask;            // mask controlling which EKF2 instances run a separate EKF-GSF yaw estimator
    AP_Int8 _gsfUseMask;            // mask controlling which EKF2 instances will use EKF-GSF yaw estimator data to assit with yaw resets
    AP_Int8 _gsfResetMaxCount;      // maximum number of times the EKF2 is allowed to reset it's yaw to the EKF-GSF estimate
    AP_Int32 _options;              // optional behaviour bitmask

    // enum for processing options
    enum class Option {
        DisableExternalNav     = (1U<<0),
    };

    // return true if an option is set
    bool option_is_set(Option option) const {
        return (uint32_t(option) & uint32_t(_options)) != 0;
    }
    
// Possible values for _flowUse
#define FLOW_USE_NONE    0
#define FLOW_USE_NAV     1
#define FLOW_USE_TERRAIN 2

    // Tuning parameters
    const float gpsNEVelVarAccScale = 0.05f;       // Scale factor applied to NE velocity measurement variance due to manoeuvre acceleration
    const float gpsDVelVarAccScale = 0.07f;        // Scale factor applied to vertical velocity measurement variance due to manoeuvre acceleration
    const float extNavVelVarAccScale = 0.05f;      // Scale factor applied to ext nav velocity measurement variance due to manoeuvre acceleration
    const float gpsPosVarAccScale = 0.05f;         // Scale factor applied to horizontal position measurement variance due to manoeuvre acceleration
    const uint8_t magDelay_ms = 60;               // Magnetometer measurement delay (msec)
    const uint8_t tasDelay_ms = 240;              // Airspeed measurement delay (msec)
    const uint16_t tiltDriftTimeMax_ms = 15000;    // Maximum number of ms allowed without any form of tilt aiding (GPS, flow, TAS, etc)
    const uint16_t posRetryTimeUseVel_ms = 10000;  // Position aiding retry time with velocity measurements (msec)
    const uint16_t posRetryTimeNoVel_ms = 7000;    // Position aiding retry time without velocity measurements (msec)
    const uint16_t hgtRetryTimeMode0_ms = 10000;   // Height retry time with vertical velocity measurement (msec)
    const uint16_t hgtRetryTimeMode12_ms = 5000;   // Height retry time without vertical velocity measurement (msec)
    const uint16_t tasRetryTime_ms = 5000;         // True airspeed timeout and retry interval (msec)
    const uint16_t magFailTimeLimit_ms = 10000;    // number of msec before a magnetometer failing innovation consistency checks is declared failed (msec)
    const float magVarRateScale = 0.005f;          // scale factor applied to magnetometer variance due to angular rate
    const float gyroBiasNoiseScaler = 2.0f;        // scale factor applied to gyro bias state process noise when on ground
    const uint8_t hgtAvg_ms = 100;                // average number of msec between height measurements
    const uint8_t betaAvg_ms = 100;               // average number of msec between synthetic sideslip measurements
    const float covTimeStepMax = 0.1f;             // maximum time (sec) between covariance prediction updates
    const float covDelAngMax = 0.05f;              // maximum delta angle between covariance prediction updates
    const float DCM33FlowMin = 0.71f;              // If Tbn(3,3) is less than this number, optical flow measurements will not be fused as tilt is too high.
    const float fScaleFactorPnoise = 1e-10f;       // Process noise added to focal length scale factor state variance at each time step
    const uint8_t flowTimeDeltaAvg_ms = 100;       // average interval between optical flow measurements (msec)
    const uint8_t flowIntervalMax_ms = 100;       // maximum allowable time between flow fusion events
    const float gndEffectBaroScaler = 4.0f;        // scaler applied to the barometer observation variance when ground effect mode is active
    const uint8_t fusionTimeStep_ms = 10;          // The minimum time interval between covariance predictions and measurement fusions in msec
    const float maxYawEstVelInnov = 2.0f;          // Maximum acceptable length of the velocity innovation returned by the EKF-GSF yaw estimator (m/s)

    // origin set by one of the cores
    Location common_EKF_origin;
    bool common_origin_valid;

    // time at start of current filter update
    uint64_t imuSampleTime_us;

    // last time of Log_Write
    uint64_t lastLogWrite_us;
    
    struct {
        uint32_t last_function_call;  // last time getLastYawResetAngle was called
        bool core_changed;            // true when a core change happened and hasn't been consumed, false otherwise
        uint32_t last_primary_change; // last time a primary has changed
        float core_delta;             // the amount of yaw change between cores when a change happened
    } yaw_reset_data;

    struct {
        uint32_t last_function_call;  // last time getLastPosNorthEastReset was called
        bool core_changed;            // true when a core change happened and hasn't been consumed, false otherwise
        uint32_t last_primary_change; // last time a primary has changed
        Vector2f core_delta;          // the amount of NE position change between cores when a change happened
    } pos_reset_data;

    struct {
        uint32_t last_function_call;  // last time getLastPosDownReset was called
        bool core_changed;            // true when a core change happened and hasn't been consumed, false otherwise
        uint32_t last_primary_change; // last time a primary has changed
        float core_delta;             // the amount of D position change between cores when a change happened
    } pos_down_reset_data;

    bool runCoreSelection; // true when the primary core has stabilised and the core selection logic can be started

    // time of last lane switch
    uint32_t lastLaneSwitch_ms;

    enum class InitFailures {
        UNKNOWN,
        NO_ENABLE, 
        NO_IMUS, 
        NO_MASK, 
        NO_MEM, 
        NO_SETUP,
        NUM_INIT_FAILURES
    };
    InitFailures initFailure;

    // update the yaw reset data to capture changes due to a lane switch
    // new_primary - index of the ekf instance that we are about to switch to as the primary
    // old_primary - index of the ekf instance that we are currently using as the primary
    void updateLaneSwitchYawResetData(uint8_t new_primary, uint8_t old_primary);

    // update the position reset data to capture changes due to a lane switch
    // new_primary - index of the ekf instance that we are about to switch to as the primary
    // old_primary - index of the ekf instance that we are currently using as the primary
    void updateLaneSwitchPosResetData(uint8_t new_primary, uint8_t old_primary);

    // update the position down reset data to capture changes due to a lane switch
    // new_primary - index of the ekf instance that we are about to switch to as the primary
    // old_primary - index of the ekf instance that we are currently using as the primary
    void updateLaneSwitchPosDownResetData(uint8_t new_primary, uint8_t old_primary);

    // return true if a new core has a better score than an existing core, including
    // checks for alignment
    bool coreBetterScore(uint8_t new_core, uint8_t current_core) const;
};
