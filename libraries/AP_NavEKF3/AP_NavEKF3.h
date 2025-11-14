/**
 * @file AP_NavEKF3.h
 * @brief 24-state Extended Kalman Filter for navigation state estimation (EKF3)
 * 
 * @details This file defines the NavEKF3 frontend interface for ArduPilot's third-generation
 *          Extended Kalman Filter. NavEKF3 provides high-accuracy attitude, position, and
 *          velocity estimation by fusing data from multiple sensors including IMUs, GPS,
 *          magnetometers, barometers, rangefinders, optical flow, and external navigation systems.
 *
 *          EKF3 Architecture:
 *          - Multi-core design: Runs parallel EKF instances (one per IMU) for redundancy
 *          - Lane management: Automatically selects the healthiest EKF core as primary
 *          - Sensor fusion: Combines inertial, GNSS, magnetic, barometric, and visual data
 *          - State vector: 24 states including position, velocity, orientation, gyro/accel biases, wind, and magnetic field
 *
 *          Improvements over EKF2:
 *          - Enhanced multi-IMU support with better lane selection algorithms
 *          - GSF (Gaussian Sum Filter) yaw estimator for improved yaw initialization
 *          - Better fault detection and isolation across sensor inputs
 *          - Improved handling of sensor dropouts and GPS glitches
 *          - Enhanced visual odometry and external navigation system integration
 *
 *          Coordinate Frame Conventions:
 *          - NED frame: North-East-Down navigation frame (right-handed, Z-axis points down)
 *          - Body frame: X-forward, Y-right, Z-down relative to vehicle body
 *          - Autopilot frame: May differ from body frame by a fixed rotation
 *
 *          Mathematical Foundation:
 *          Based on the derivation in https://github.com/PX4/ecl/
 *          blob/master/matlab/scripts/Inertial%20Nav%20EKF/GenerateNavFilterEquations.m
 *          Converted from Matlab to C++ by Paul Riseborough
 *
 * @note This is a frontend manager class that coordinates multiple NavEKF3_core instances.
 *       The actual EKF algorithms are implemented in NavEKF3_core.
 *
 * @warning Incorrect parameter configuration (especially innovation gates and sensor priorities)
 *          can lead to poor state estimation or loss of navigation capability. Test thoroughly
 *          in SITL before deploying parameter changes on actual vehicles.
 *
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 *            This program is free software: you can redistribute it and/or modify
 *            it under the terms of the GNU General Public License as published by
 *            the Free Software Foundation, either version 3 of the License, or
 *            (at your option) any later version.
 */
#pragma once

#include <AP_Common/Location.h>
#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>
#include <AP_NavEKF/AP_Nav_Common.h>
#include <AP_NavEKF/AP_NavEKF_Source.h>

class NavEKF3_core;
class EKFGSF_yaw;

/**
 * @class NavEKF3
 * @brief Frontend manager for the third-generation Extended Kalman Filter navigation system
 *
 * @details NavEKF3 is the main interface class for ArduPilot's EKF3 navigation state estimator.
 *          It manages multiple NavEKF3_core instances (one per IMU) and implements a lane
 *          selection algorithm to choose the healthiest core as the primary state source.
 *
 *          Multi-Core Architecture:
 *          - Creates one EKF core instance per available IMU (up to MAX_EKF_CORES)
 *          - Each core runs independently with its own state estimates
 *          - Continuously monitors health and innovation consistency of all cores
 *          - Automatically switches to a healthier core if primary becomes unreliable
 *          - Lane switching happens seamlessly without disrupting vehicle control
 *
 *          Lane Selection Algorithm:
 *          - Computes normalized innovation test ratios for each core
 *          - Tracks error scores based on innovation consistency
 *          - Calculates relative error between cores
 *          - Switches lanes when an alternate core is consistently better
 *          - Includes hysteresis to prevent excessive switching
 *
 *          State Estimation:
 *          The 24-state vector includes:
 *          - Position: North, East, Down in NED frame (m)
 *          - Velocity: VN, VE, VD in NED frame (m/s)
 *          - Attitude: Quaternion representing rotation from body to NED frame
 *          - Gyro bias: 3-axis gyro bias estimates (rad/s)
 *          - Accel bias: 3-axis Z-accel bias estimate (m/s²)
 *          - Wind: North and East wind velocity (m/s)
 *          - Magnetic field: NED earth magnetic field (gauss)
 *          - Magnetic field: XYZ body magnetic field (gauss)
 *
 *          Sensor Fusion Capabilities:
 *          - IMU: Gyroscopes and accelerometers (continuous prediction)
 *          - GNSS: Position and velocity (GPS, GLONASS, Galileo, BeiDou)
 *          - Magnetometer: Heading reference via earth magnetic field
 *          - Barometer: Altitude reference
 *          - Rangefinder: Terrain-relative height
 *          - Airspeed: True airspeed for fixed-wing
 *          - Optical flow: Visual velocity for low-altitude flight
 *          - Visual odometry: Camera-based position/attitude
 *          - Wheel odometry: Ground vehicle wheel encoders
 *          - External navigation: Integration with external positioning systems
 *          - Beacon positioning: Triangulation from fixed beacons
 *
 *          Initialization and Startup:
 *          - Filter initializes when IMU data becomes available
 *          - Attitude initialized from accelerometer (roll, pitch) and magnetometer (yaw)
 *          - GSF yaw estimator can provide initial yaw when magnetometer unreliable
 *          - Position initialized from first GPS fix or external navigation
 *          - Velocity initialized from GPS or external navigation
 *
 *          Health Monitoring:
 *          - Tracks innovation consistency for all sensor inputs
 *          - Monitors state covariance growth (divergence detection)
 *          - Checks for numerical errors (NaN/Inf in states)
 *          - Provides detailed filter status and fault reporting
 *          - Implements pre-arm checks to prevent arming with poor state estimates
 *
 *          Failsafe and Recovery:
 *          - Automatic lane switching to healthier cores
 *          - Yaw reset capability when magnetometer anomalies detected
 *          - Position and velocity resets after prolonged sensor loss
 *          - Graceful degradation when sensors become unavailable
 *
 * @note This class is a frontend that delegates actual filter operations to NavEKF3_core instances.
 *       Public methods typically operate on the primary core or aggregate data from all cores.
 *
 * @warning The EKF requires continuous IMU data at high rate (typically 400Hz for copter, 50-400Hz for plane).
 *          IMU data dropouts or timing jitter can degrade state estimates or cause filter divergence.
 *
 * @warning Parameter changes should be tested extensively in SITL before flight use. Incorrect
 *          innovation gates, process noise, or sensor weights can cause poor estimation or instability.
 *
 * @see NavEKF3_core for the actual EKF implementation
 * @see EKFGSF_yaw for the Gaussian Sum Filter yaw estimator
 * @see AP_AHRS for the attitude and heading reference system that uses NavEKF3
 */
class NavEKF3 {
    friend class NavEKF3_core;

public:
    /**
     * @brief Constructor for NavEKF3 frontend
     * 
     * @details Initializes the EKF frontend manager. Does not create core instances yet;
     *          cores are created in InitialiseFilter() based on available IMUs.
     */
    NavEKF3();

    /* Do not allow copies */
    CLASS_NO_COPY(NavEKF3);

    /**
     * @brief Parameter table for EKF3 configuration (primary parameters)
     * 
     * @details Defines AP_Param group structure for EKF3 tunable parameters including
     *          sensor noise values, innovation gate thresholds, process noise parameters,
     *          and feature enable flags. Used by AP_Param system for EEPROM storage and
     *          ground station parameter access.
     */
    static const struct AP_Param::GroupInfo var_info[];
    
    /**
     * @brief Parameter table for EKF3 configuration (secondary parameters)
     * 
     * @details Additional EKF3 parameters that didn't fit in the primary var_info table.
     *          Includes extended configuration options added in later releases.
     */
    static const struct AP_Param::GroupInfo var_info2[];

    /**
     * @brief Get the number of active EKF cores
     * 
     * @details Returns the number of NavEKF3_core instances currently running.
     *          Typically equals the number of available IMUs (up to MAX_EKF_CORES).
     *          Used by logging and diagnostics to determine how many lanes are active.
     * 
     * @return Number of active cores (1 to MAX_EKF_CORES)
     */
    uint8_t activeCores(void) const {
        return num_cores;
    }

    /**
     * @brief Initialize the EKF filter cores
     * 
     * @details Creates and initializes NavEKF3_core instances based on available IMUs
     *          and the EK3_IMU_MASK parameter. Each enabled IMU gets its own EKF core.
     *          Must be called before UpdateFilter() can be used.
     *
     *          Initialization sequence:
     *          1. Determines number of cores from available IMUs and EK3_IMU_MASK
     *          2. Allocates memory for core instances
     *          3. Initializes each core with IMU index and shared parameters
     *          4. Sets initial primary core based on EK3_PRIMARY parameter
     *          5. Initializes GSF yaw estimator if enabled
     * 
     * @return true if initialization successful, false on failure
     * 
     * @note Should be called once during system startup after IMUs are initialized
     * @warning Returns false if memory allocation fails or no valid IMUs available
     */
    bool InitialiseFilter(void);

    /**
     * @brief Update all EKF filter cores with latest sensor data
     * 
     * @details This is the main EKF update function, called at the IMU sample rate
     *          (typically 400Hz for multicopters, 50-400Hz for planes). Each call:
     *          1. Retrieves latest IMU data from each core's assigned IMU
     *          2. Performs EKF prediction step (state and covariance propagation)
     *          3. Fuses available sensor measurements (GPS, mag, baro, etc.)
     *          4. Updates core health scores and innovation statistics
     *          5. Performs lane selection if needed
     *          6. Updates output complementary filter for smooth state estimates
     *
     * @note Must be called whenever new IMU data is available
     * @note This function should complete within the IMU sample period to avoid timing issues
     * @warning Timing jitter or delayed calls can degrade state estimation quality
     * 
     * @see InitialiseFilter() must be called successfully before first UpdateFilter() call
     */
    void UpdateFilter(void);

    /**
     * @brief Check consolidated EKF health status
     * 
     * @details Evaluates the health of the primary EKF core by checking:
     *          - Filter initialization status
     *          - Innovation consistency (measurements match predictions)
     *          - State covariance bounds (no divergence)
     *          - Numerical stability (no NaN or Inf values)
     *          - Recent measurement updates (not in dead reckoning)
     *
     * @return true if primary core is healthy and suitable for flight control, false otherwise
     * 
     * @note This is a high-level health check. Use getFilterStatus() for detailed diagnostics
     * @warning A false return indicates the EKF should not be used for flight control
     */
    bool healthy(void) const;

    /**
     * @brief Perform pre-arm checks for EKF readiness
     * 
     * @details Validates that the EKF is properly initialized and accurate enough for flight.
     *          Checks include:
     *          - Filter initialization complete
     *          - GPS quality sufficient (if GPS required)
     *          - Position accuracy within limits
     *          - Attitude initialization complete (especially yaw alignment)
     *          - Innovation consistency acceptable
     *          - No critical faults detected
     *
     * @param[in] requires_position true if horizontal position estimation must be available
     * @param[out] failure_msg buffer to store failure description if check fails
     * @param[in] failure_msg_len size of failure_msg buffer in bytes
     * 
     * @return true if all pre-arm checks pass, false if any check fails
     * 
     * @note Populates failure_msg with human-readable error description on failure
     * @warning Vehicle should not arm if this returns false
     */
    bool pre_arm_check(bool requires_position, char *failure_msg, uint8_t failure_msg_len) const;

    /**
     * @brief Get the index of the primary (currently active) EKF core
     * 
     * @details Returns which NavEKF3_core instance is currently being used as the
     *          primary state source for vehicle control. The primary is selected
     *          based on health scoring and innovation consistency.
     * 
     * @return Index of primary core (0 to num_cores-1), or -1 if no core selected
     * 
     * @note Core index corresponds to array position in internal core array
     */
    int8_t getPrimaryCoreIndex(void) const;

    /**
     * @brief Get the IMU index used by the primary EKF core
     * 
     * @details Returns which IMU sensor is being used by the currently active
     *          primary EKF core. Useful for diagnostics and logging to determine
     *          which physical IMU is providing the navigation solution.
     * 
     * @return IMU index (0, 1, 2, etc.) of primary core's IMU, or -1 if no primary selected
     * 
     * @note IMU index corresponds to AP_InertialSensor instance numbers
     */
    int8_t getPrimaryCoreIMUIndex(void) const;

    /**
     * @brief Get the North-East position relative to the EKF origin
     * 
     * @details Returns the last estimated horizontal position in the NED navigation frame.
     *          Position is relative to the EKF origin set by setOriginLLH() or the first
     *          GPS fix location. If a high-quality solution is not available, returns the
     *          best available estimate but indicates this via the return value.
     *
     * @param[out] posNE North and East position components in meters (NED frame)
     * 
     * @return true if solution is suitable for flight control, false if degraded/unavailable
     * 
     * @note Units: meters in NED frame (North positive, East positive)
     * @warning If false returned, do not use for flight control - state estimate is degraded
     */
    bool getPosNE(Vector2p &posNE) const;

    /**
     * @brief Get the Down position relative to the EKF origin
     * 
     * @details Returns the last estimated vertical position in the NED navigation frame.
     *          Positive Down means below the origin. Origin altitude is typically set to
     *          home altitude or first GPS altitude. If high-quality solution unavailable,
     *          returns best estimate but indicates this via return value.
     *
     * @param[out] posD Down position component in meters (NED frame, positive down)
     * 
     * @return true if solution is suitable for flight control, false if degraded/unavailable
     * 
     * @note Units: meters in NED frame (Down positive = below origin)
     * @warning If false returned, do not use for flight control - state estimate is degraded
     */
    bool getPosD(postype_t &posD) const;

    /**
     * @brief Get the velocity vector in NED frame
     * 
     * @details Returns the current estimated velocity in North-East-Down frame.
     *          Velocity is with respect to the earth (not airspeed).
     *
     * @param[out] vel Velocity vector [VN, VE, VD] in m/s (NED frame)
     * 
     * @note Units: m/s in NED frame (North positive, East positive, Down positive)
     * @note This is ground velocity (earth-relative), not airspeed
     */
    void getVelNED(Vector3f &vel) const;

    /**
     * @brief Get the true airspeed vector in body frame
     * 
     * @details Returns estimated airspeed (velocity relative to air mass) in body frame.
     *          Calculated from ground velocity and estimated wind. Only available when
     *          wind estimation is active (typically requires airspeed sensor or sufficient
     *          maneuvering for fixed-wing aircraft).
     *
     * @param[out] vel Airspeed vector [Vx, Vy, Vz] in m/s (body frame)
     * 
     * @return true if airspeed estimate available, false if unavailable
     * 
     * @note Units: m/s in body frame (X-forward, Y-right, Z-down)
     * @note Requires wind estimation to be active
     */
    bool getAirSpdVec(Vector3f &vel) const;

    /**
     * @brief Get true airspeed sensor health and innovation data
     * 
     * @details Returns diagnostic information about the most recent true airspeed (TAS)
     *          measurement fusion. Innovation is the difference between measured and
     *          predicted airspeed. Large innovations indicate sensor error or state
     *          estimation problems.
     *
     * @param[out] innovation Airspeed innovation (measured - predicted) in m/s
     * @param[out] innovationVariance Innovation variance (uncertainty squared) in (m/s)²
     * @param[out] age_ms Time since last airspeed measurement processed in milliseconds
     * 
     * @return true if data available, false if no airspeed data processed yet
     * 
     * @note Units: innovation in m/s, variance in (m/s)², age in milliseconds
     * @note Large innovation (> ~5 m/s) suggests airspeed sensor error or calibration issue
     */
    bool getAirSpdHealthData(float &innovation, float &innovationVariance, uint32_t &age_ms) const;

    /**
     * @brief Get the vertical velocity (rate of altitude change)
     * 
     * @details Returns the instantaneous rate of change of the Down position (dPosD/dt).
     *          This may differ slightly from the VD component of the velocity state because
     *          it includes effects of vertical position corrections, but it is always
     *          kinematically consistent with the vertical position estimate.
     *
     * @return Vertical velocity in m/s (positive = descending, negative = climbing)
     * 
     * @note Units: m/s in NED frame (positive Down = descending)
     * @note More responsive to altitude corrections than velocity state VD component
     */
    float getPosDownDerivative() const;

    /**
     * @brief Get gyroscope bias estimates
     * 
     * @details Returns the estimated gyro bias for a specific EKF core or the primary core.
     *          Gyro bias is the constant offset error in gyroscope measurements, estimated
     *          by the EKF and subtracted from raw gyro readings during state prediction.
     *
     * @param[in] instance EKF core index (0 to num_cores-1), or -1 for primary core
     * @param[out] gyroBias Gyro bias vector [bias_x, bias_y, bias_z] in rad/s (body frame)
     * 
     * @note Units: rad/s in body frame
     * @note Bias is subtracted from raw gyro measurements: corrected = raw - bias
     * @note Typical values: 0.001 to 0.01 rad/s for consumer-grade IMUs
     */
    void getGyroBias(int8_t instance, Vector3f &gyroBias) const;

    /**
     * @brief Get accelerometer bias estimates
     * 
     * @details Returns the estimated accelerometer bias for a specific EKF core or primary.
     *          Currently only Z-axis accel bias is estimated. Accel bias is the constant
     *          offset error in accelerometer measurements, estimated by the EKF.
     *
     * @param[in] instance EKF core index (0 to num_cores-1), or -1 for primary core  
     * @param[out] accelBias Accel bias vector [bias_x, bias_y, bias_z] in m/s² (body frame)
     * 
     * @note Units: m/s² in body frame
     * @note Currently only Z-axis bias is estimated; X and Y biases are typically zero
     * @note Typical Z-axis values: 0.01 to 0.5 m/s² for consumer-grade IMUs
     */
    void getAccelBias(int8_t instance, Vector3f &accelBias) const;

    /**
     * @brief Get the active position/velocity/yaw source set index
     * 
     * @details Returns which source set is currently being used for position, velocity,
     *          and yaw estimation. Source sets allow configuration of primary, secondary,
     *          and tertiary sensor priorities (e.g., GPS primary, optical flow secondary).
     *
     * @return Active source set index (0=primary, 1=secondary, 2=tertiary)
     * 
     * @see setPosVelYawSourceSet() to change the active source set
     */
    uint8_t get_active_source_set() const;

    /**
     * @brief Reset gyroscope bias estimates to zero
     * 
     * @details Resets the gyro bias states in all EKF cores to zero. Used when gyro
     *          calibration is re-performed or when bias estimates have diverged incorrectly.
     *
     * @warning Only use when vehicle is stationary and a fresh gyro calibration is needed
     * @warning Will cause temporary attitude estimation errors until bias re-converges
     */
    void resetGyroBias(void);

    /**
     * @brief Reset the barometric altitude reference (height datum)
     * 
     * @details Resets the barometer to read zero at the current height and adjusts the
     *          EKF origin altitude accordingly, so the global position (origin + EKF height)
     *          remains unchanged. Used to prevent barometer drift from accumulating as a
     *          DC offset. Not performed if using rangefinder as primary height source.
     *
     *          Reset sequence:
     *          1. Current baro reading becomes new zero reference
     *          2. EKF height state reset to zero
     *          3. Origin altitude adjusted: new_origin_alt = old_origin_alt + old_EKF_height
     *          4. Global altitude (WGS-84) remains continuous
     *
     * @return true if height datum reset was performed, false if not (e.g., using rangefinder)
     * 
     * @note Does not affect global position - only changes the reference frame
     * @note Typically called on landing or during long flights to compensate for baro drift
     * @warning Not performed when rangefinder is primary height source
     */
    bool resetHeightDatum(void);

    /**
     * @brief Get EKF control limits when using optical flow
     * 
     * @details When using optical flow for velocity estimation, returns the maximum
     *          ground speed limit imposed by flow sensor field-of-view and altitude,
     *          and a gain scaling factor to compensate for increased velocity noise
     *          at higher altitudes.
     *
     * @param[out] ekfGndSpdLimit Maximum horizontal ground speed in m/s before flow saturates
     * @param[out] ekfNavVelGainScaler Gain scale factor (0-1) for velocity controller
     * 
     * @note ekfGndSpdLimit decreases with altitude due to flow sensor angular resolution
     * @note ekfNavVelGainScaler should multiply velocity controller gains to maintain stability
     * @note Only relevant when optical flow is active for velocity estimation
     */
    void getEkfControlLimits(float &ekfGndSpdLimit, float &ekfNavVelGainScaler) const;

    /**
     * @brief Get wind velocity estimates in NED frame
     * 
     * @details Returns the estimated wind velocity in North-East-Down frame. Wind is
     *          the velocity of the air mass relative to the ground. Only available when
     *          wind estimation is active (requires airspeed sensor or sufficient vehicle
     *          maneuvering for fixed-wing).
     *
     * @param[out] wind Wind velocity vector [WN, WE, WD] in m/s (NED frame)
     * 
     * @return true if wind state estimation is active, false otherwise
     * 
     * @note Units: m/s in NED frame (positive = air moving North/East/Down)
     * @note Vertical wind (WD) is typically small and may not be estimated
     * @note Wind estimation improves with airspeed sensor or dynamic maneuvering
     */
    bool getWind(Vector3f &wind) const;

    /**
     * @brief Get earth magnetic field estimates in NED frame
     * 
     * @details Returns the estimated earth magnetic field vector in NED frame. This is
     *          the local geomagnetic field including declination, inclination, and intensity.
     *          EKF learns this field to provide heading reference even if World Magnetic
     *          Model (WMM) is inaccurate.
     *
     * @param[out] magNED Earth magnetic field [MagN, MagE, MagD] in gauss/1000 (NED frame)
     * 
     * @note Units: gauss/1000 (milligauss) in NED frame
     * @note Typical total field: 250-650 milligauss depending on location
     * @note Field slowly drifts as vehicle moves or when near magnetic anomalies
     */
    void getMagNED(Vector3f &magNED) const;

    /**
     * @brief Get body magnetic field estimates in body frame
     * 
     * @details Returns the estimated magnetic field in vehicle body frame. This includes
     *          both the earth field rotated into body frame and any fixed magnetic
     *          interference from vehicle components (hard-iron bias). EKF estimates these
     *          biases during magnetometer calibration.
     *
     * @param[out] magXYZ Body magnetic field [MagX, MagY, MagZ] in gauss/1000 (body frame)
     * 
     * @note Units: gauss/1000 (milligauss) in body frame (X-forward, Y-right, Z-down)
     * @note Includes hard-iron magnetometer bias from vehicle electronics/motors
     * @note Should remain constant in body frame (earth field rotates as vehicle rotates)
     */
    void getMagXYZ(Vector3f &magXYZ) const;

    /**
     * @brief Get the index of the active airspeed sensor
     * 
     * @details Returns which airspeed sensor (if multiple available) is currently being
     *          used by the primary EKF core for true airspeed measurements.
     *
     * @return Airspeed sensor index (0, 1, 2, etc.)
     * 
     * @note Corresponds to AP_Airspeed sensor instance numbers
     */
    uint8_t getActiveAirspeed() const;

    /**
     * @brief Get estimated magnetometer hard-iron offsets
     * 
     * @details Returns the estimated constant magnetic field offsets (hard-iron bias)
     *          for a specific magnetometer. These offsets are learned during magnetometer
     *          calibration or in-flight compass learning and subtracted from raw readings.
     *
     * @param[in] mag_idx Magnetometer index (0, 1, 2, etc.)
     * @param[out] magOffsets Hard-iron offset vector [Ox, Oy, Oz] in gauss/1000 (body frame)
     * 
     * @return true if offsets are valid and available, false otherwise
     * 
     * @note Units: gauss/1000 (milligauss) in body frame
     * @note Offsets are subtracted: corrected_mag = raw_mag - offset
     * @note Offsets vary by vehicle due to different motor/ESC/wiring configurations
     */
    bool getMagOffsets(uint8_t mag_idx, Vector3f &magOffsets) const;

    /**
     * @brief Get current global position in WGS-84 coordinates
     * 
     * @details Returns the estimated global position (latitude, longitude, altitude) by
     *          combining the EKF origin with the NED position estimates. If a high-quality
     *          EKF solution is not available, returns the last raw GPS position instead.
     *
     * @param[out] loc Location structure with latitude, longitude (degrees), altitude (meters AMSL)
     * 
     * @return true if position data available (EKF or raw GPS), false if unavailable
     * 
     * @note Altitude is above mean sea level (AMSL) in WGS-84 ellipsoid
     * @note Use getFilterStatus() for detailed health - this only indicates data availability
     * @warning For flight control, always check getFilterStatus() to ensure estimate quality
     */
    bool getLLH(Location &loc) const;

    /**
     * @brief Get the EKF origin location in WGS-84 coordinates
     * 
     * @details Returns the latitude, longitude, and altitude that define the origin of the
     *          NED navigation frame. All EKF position estimates (North, East, Down) are
     *          relative to this point. Origin is typically set to home location or first
     *          GPS fix position.
     *
     * @param[out] loc Origin location with latitude, longitude (degrees), altitude (meters AMSL)
     * 
     * @return true if origin has been set, false if not yet initialized
     * 
     * @note All getPosNE() and getPosD() values are relative to this origin
     * @note Origin should not change during flight
     * @see setOriginLLH() to manually set the origin
     */
    bool getOriginLLH(Location &loc) const;

    /**
     * @brief Set the EKF origin location in WGS-84 coordinates
     * 
     * @details Manually sets the latitude, longitude, and altitude that define the origin
     *          of the NED navigation frame. All subsequent position estimates will be relative
     *          to this location. Origin can only be set before arming or during initialization.
     *
     * @param[in] loc Desired origin location with latitude, longitude (degrees), altitude (meters AMSL)
     * 
     * @return true if origin was successfully set, false if rejected
     * 
     * @note Cannot change origin while vehicle is armed (safety restriction)
     * @note All EKF cores share a common origin
     * @warning Changing origin during flight would cause discontinuous position jumps
     * @see getOriginLLH() to retrieve current origin
     */
    bool setOriginLLH(const Location &loc);

    /**
     * @brief Set horizontal position from GPS coordinates with uncertainty
     * 
     * @details Directly sets the EKF's North-East position states and covariances from
     *          a supplied GPS location and accuracy estimate. Used for position initialization
     *          or correction. The altitude component is ignored (use height sensors instead).
     *
     * @param[in] loc Location with latitude and longitude (degrees); altitude not used
     * @param[in] posErr 1-sigma horizontal position uncertainty in meters
     * @param[in] timestamp_ms System time when position measurement was taken (milliseconds)
     * 
     * @return true if position set successfully, false on failure
     * 
     * @note Only horizontal position (NE) is set; vertical position (D) is unaffected
     * @note Position error initializes the state covariance appropriately
     * @warning Large position jumps can cause temporary estimation issues
     */
    bool setLatLng(const Location &loc, float posErr, uint32_t timestamp_ms);

    /**
     * @brief Get height above ground level (AGL)
     * 
     * @details Returns the estimated height above the terrain directly below the vehicle.
     *          Calculated using rangefinder measurements and/or terrain database. Only
     *          available when terrain height estimation is active.
     *
     * @param[out] HAGL Height above ground level in meters
     * 
     * @return true if terrain height is being estimated, false otherwise
     * 
     * @note Units: meters (positive = above ground)
     * @note Requires rangefinder or terrain database to be active
     * @note Terrain height is maintained in a separate estimator within each core
     */
    bool getHAGL(float &HAGL) const;

    /**
     * @brief Get vehicle attitude as Euler angles
     * 
     * @details Returns the estimated vehicle orientation as roll, pitch, and yaw angles
     *          in radians. Euler angles use the aerospace 3-2-1 (yaw-pitch-roll) sequence.
     *          Derived from the EKF attitude quaternion.
     *
     * @param[out] eulers Euler angles [roll, pitch, yaw] in radians
     * 
     * @note Units: radians
     * @note Roll: rotation about X-axis (right wing down = positive)
     * @note Pitch: rotation about Y-axis (nose up = positive)
     * @note Yaw: rotation about Z-axis (nose right = positive, 0 = North)
     * @note Euler angles have gimbal lock at pitch = ±90 degrees
     * @see getQuaternion() for singularity-free attitude representation
     */
    void getEulerAngles(Vector3f &eulers) const;

    /**
     * @brief Get rotation matrix from body frame to NED frame
     * 
     * @details Returns the 3x3 Direction Cosine Matrix (DCM) that transforms vectors from
     *          body frame to NED frame. Used for coordinate transformations and control
     *          calculations.
     *
     * @param[out] mat 3x3 rotation matrix (body to NED transformation)
     * 
     * @note To transform body vector to NED: v_NED = mat * v_body
     * @note To transform NED vector to body: v_body = mat^T * v_NED
     * @note Body frame: X-forward, Y-right, Z-down
     * @note NED frame: X-North, Y-East, Z-Down
     */
    void getRotationBodyToNED(Matrix3f &mat) const;

    /**
     * @brief Get attitude quaternion from body frame to NED frame
     * 
     * @details Returns the quaternion representing vehicle orientation for a specific
     *          EKF core or the primary core. Quaternions avoid gimbal lock and are the
     *          internal attitude representation used by the EKF.
     *
     * @param[in] instance EKF core index (0 to num_cores-1), or -1 for primary core
     * @param[out] quat Quaternion representing rotation from body frame to NED frame
     * 
     * @note Quaternion convention: q = [q0, q1, q2, q3] = [w, x, y, z]
     * @note Quaternion normalized: q0² + q1² + q2² + q3² = 1
     * @note Rotation: v_NED = quat * v_body * quat^(-1)
     */
    void getQuaternionBodyToNED(int8_t instance, Quaternion &quat) const;

    /**
     * @brief Get attitude quaternion from NED frame to autopilot frame
     * 
     * @details Returns the quaternion for the primary core representing rotation from NED
     *          to the autopilot frame. The autopilot frame may differ from the body frame
     *          by a fixed rotation (board orientation offset).
     *
     * @param[out] quat Quaternion representing rotation from NED frame to autopilot frame
     * 
     * @note Autopilot frame accounts for AHRS_ORIENTATION (board mounting angle)
     * @note For standard mounting, autopilot frame = body frame
     * @note This is the inverse rotation of getQuaternionBodyToNED()
     */
    void getQuaternion(Quaternion &quat) const;

    /**
     * @brief Get sensor measurement innovations
     * 
     * @details Returns the innovations (measurement residuals) for key sensors. Innovation
     *          is the difference between the measured value and the EKF predicted value:
     *          innovation = measurement - prediction. Large innovations indicate sensor
     *          errors, state estimation problems, or unexpected vehicle dynamics.
     *
     * @param[out] velInnov Velocity innovation [VN, VE, VD] in m/s (NED frame)
     * @param[out] posInnov Position innovation [PN, PE, PD] in meters (NED frame)
     * @param[out] magInnov Magnetometer innovation [MagX, MagY, MagZ] in gauss/1000 (body frame)
     * @param[out] tasInnov True airspeed innovation in m/s
     * @param[out] yawInnov Yaw angle innovation in radians
     * 
     * @return true if innovations available, false otherwise
     * 
     * @note Units: velocity in m/s, position in meters, mag in gauss/1000, TAS in m/s, yaw in radians
     * @note Innovations should typically be small (within a few standard deviations)
     * @note Large persistent innovations suggest sensor calibration or EKF tuning issues
     */
    bool getInnovations(Vector3f &velInnov, Vector3f &posInnov, Vector3f &magInnov, float &tasInnov, float &yawInnov) const;

    /**
     * @brief Get innovation test ratios (normalized innovations)
     * 
     * @details Returns the innovation variances and test ratios used for sensor consistency
     *          checking. Test ratio = innovation² / (innovation_variance * gate_threshold).
     *          Values > 1 indicate the innovation exceeds the consistency gate and the
     *          measurement may be rejected.
     *
     * @param[out] velVar Velocity innovation test ratio (dimensionless)
     * @param[out] posVar Position innovation test ratio (dimensionless)
     * @param[out] hgtVar Height innovation test ratio (dimensionless)
     * @param[out] magVar Magnetometer innovation test ratios [X, Y, Z] (dimensionless)
     * @param[out] tasVar True airspeed innovation test ratio (dimensionless)
     * @param[out] offset Optical flow innovation test ratios [X, Y] (dimensionless)
     * 
     * @return true if variance data available, false otherwise
     * 
     * @note Test ratios are normalized: < 1 = passed gate, > 1 = failed gate
     * @note Gates controlled by EK3_*_INNOV_GATE parameters (default typically 300-500%)
     * @note Consistent gate failures indicate sensor problems or poor EKF tuning
     */
    bool getVariances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar, Vector2f &offset) const;

    /**
     * @brief Get velocity innovations for a specific position source
     * 
     * @details Returns the velocity innovations and variances for a specified position
     *          source (GPS, external nav, optical flow, etc.). Useful for diagnostics
     *          when multiple position sources are available.
     *
     * @param[in] source Position source type (GPS, ExtNav, OptFlow, etc.)
     * @param[out] innovations Velocity innovations [VN, VE, VD] in m/s
     * @param[out] variances Innovation variances [VarN, VarE, VarD] in (m/s)²
     * 
     * @return true if data available for this source, false otherwise
     * 
     * @note Units: innovations in m/s, variances in (m/s)²
     * @note Only returns data for sources currently being fused
     */
    bool getVelInnovationsAndVariancesForSource(AP_NavEKF_Source::SourceXY source, Vector3f &innovations, Vector3f &variances) const WARN_IF_UNUSED;

    /**
     * @brief Check if compass (magnetometer) is being used for yaw
     * 
     * @details Returns whether the compass is currently being fused for yaw estimation
     *          in the primary EKF core. Compass may be disabled due to configuration,
     *          excessive interference, or use of alternative yaw sources (GPS, external nav).
     *
     * @return true if compass is being used for yaw, false otherwise
     * 
     * @note Public method for AHRS reporting via use_compass()
     * @note Compass may be temporarily disabled during in-flight calibration
     * @see using_noncompass_for_yaw() for non-magnetic yaw sources
     */
    bool use_compass(void) const;

    /**
     * @brief Write optical flow sensor measurements to EKF
     * 
     * @details Provides raw optical flow measurements to the EKF for velocity estimation.
     *          Optical flow measures apparent angular motion of the ground texture, which
     *          combined with height gives ground velocity. Used for velocity estimation
     *          when GPS unavailable (indoor flight, GPS-denied environments).
     *
     * @param[in] rawFlowQuality Flow measurement quality (0-255, 255 = best quality)
     * @param[in] rawFlowRates Angular flow rates [rateX, rateY] in rad/s (sensor frame)
     * @param[in] rawGyroRates Gyro rates [gyroX, gyroY] in rad/s from flow sensor's internal gyro
     * @param[in] msecFlowMeas System time when flow measurement was taken (milliseconds)
     * @param[in] posOffset Flow sensor position [X, Y, Z] in body frame (meters)
     * @param[in] heightOverride Fixed height above ground for rovers (meters), 0 if not used
     * 
     * @note Sign convention: Right-hand rotation about axis produces positive flow and gyro
     * @note Units: flow rates in rad/s, position offset in meters, height in meters
     * @note Quality < ~100 may cause fusion to be rejected
     * @note Flow rate = (ground_velocity / height) + gyro_compensation
     * @warning Optical flow is unreliable above ~5m altitude for typical sensors
     * @warning Requires textured surface - doesn't work over uniform surfaces
     */
    void writeOptFlowMeas(const uint8_t rawFlowQuality, const Vector2f &rawFlowRates, const Vector2f &rawGyroRates, const uint32_t msecFlowMeas, const Vector3f &posOffset, float heightOverride);

    /**
     * @brief Retrieve latest corrected optical flow sample
     * 
     * @details Gets the most recent optical flow measurement after EKF corrections
     *          (gyro compensation, body rate removal). Used for optical flow sensor
     *          calibration and diagnostics.
     *
     * @param[out] timeStamp_ms Time of flow measurement (milliseconds)
     * @param[out] flowRate Corrected flow rates [rateX, rateY] in rad/s
     * @param[out] bodyRate Vehicle body rates [rateX, rateY] in rad/s
     * @param[out] losPred Predicted line-of-sight rates [predX, predY] in rad/s
     * 
     * @return true if flow sample available, false otherwise
     * 
     * @note flowRate has body rates removed: flowRate = raw_flow - bodyRate
     * @note losPred is EKF prediction based on velocity and height estimates
     * @note Innovation = flowRate - losPred
     */
    bool getOptFlowSample(uint32_t& timeStamp_ms, Vector2f& flowRate, Vector2f& bodyRate, Vector2f& losPred) const;

    /**
     * @brief Write visual odometry body frame displacement measurements to EKF
     *
     * @details Provides linear and angular displacement measurements from a visual odometry
     *          sensor (e.g., camera-based tracking like T265). Measurements are in body frame
     *          and represent change since last measurement. Used for position and attitude
     *          estimation when GPS unavailable.
     *
     * @param[in] quality Normalized confidence value (0-100, 100 = highest confidence)
     * @param[in] delPos Linear displacement [ΔX, ΔY, ΔZ] in body frame (meters)
     * @param[in] delAng Angular displacement [Δroll, Δpitch, Δyaw] in body frame (radians)
     * @param[in] delTime Time interval for displacement measurements (seconds)
     * @param[in] timeStamp_ms System time of last image used for measurement (milliseconds)
     * @param[in] delay_ms Average measurement delay relative to IMU (milliseconds)
     * @param[in] posOffset Camera focal point position [X, Y, Z] in body frame (meters)
     * 
     * @note Displacements are relative to inertial reference at timeStamp_ms
     * @note Units: delPos in meters, delAng in radians, delTime in seconds
     * @note Quality < 50 may cause measurement rejection
     * @note Typical update rate: 30-90 Hz depending on camera
     * @warning Camera position offset (posOffset) must be accurate for proper fusion
     */
    void writeBodyFrameOdom(float quality, const Vector3f &delPos, const Vector3f &delAng, float delTime, uint32_t timeStamp_ms, uint16_t delay_ms, const Vector3f &posOffset);

    /**
     * @brief Write wheel odometry measurements to EKF
     *
     * @details Provides wheel encoder measurements for ground vehicle velocity estimation.
     *          Measures wheel rotation to calculate vehicle forward motion. Axis of rotation
     *          assumed parallel to vehicle body Y-axis (perpendicular to forward direction).
     *
     * @param[in] delAng Change in wheel angular position since last measurement (radians)
     * @param[in] delTime Time interval for angular measurement (seconds)
     * @param[in] timeStamp_ms System time when rotation was measured (milliseconds)
     * @param[in] posOffset Wheel hub position [X, Y, Z] in body frame (meters)
     * @param[in] radius Effective rolling radius of wheel (meters)
     * 
     * @note Positive rotation produced by forward vehicle motion
     * @note Linear velocity = delAng * radius / delTime
     * @note Should not be called faster than EKF update rate (typically 50-100 Hz)
     * @note Units: delAng in radians, delTime in seconds, radius in meters
     * @warning Wheel slip and skidding can cause velocity errors
     * @warning Radius should account for tire deformation under load
     */
    void writeWheelOdom(float delAng, float delTime, uint32_t timeStamp_ms, const Vector3f &posOffset, float radius);

    /**
     * @brief Write yaw angle measurement from external sensor to EKF
     *
     * @details Provides absolute yaw angle measurement from external yaw sensor (e.g.,
     *          dual GPS heading, external compass, vision system). Yaw is relative to
     *          true north. Useful for improved heading estimation or when magnetometer
     *          is unavailable/unreliable.
     *
     * @param[in] yawAngle Vehicle yaw relative to true north (radians)
     * @param[in] yawAngleErr 1-sigma measurement uncertainty (radians)
     * @param[in] timeStamp_ms System time of measurement including all delays (milliseconds)
     * @param[in] type Euler rotation order: 1 = 312 (ZXY), 2 = 321 (ZYX)
     * 
     * @note Sign convention: Positive = right-hand rotation about Z body axis (clockwise from above)
     * @note Yaw is first rotation in Euler sequence (applied before pitch and roll)
     * @note Units: yawAngle and yawAngleErr in radians
     * @note timeStamp_ms must include measurement lag and transmission delays
     * @note Type determines how yaw combines with roll/pitch in full orientation
     * @warning Ensure yaw is referenced to true north, not magnetic north
     * @warning Measurement delay (timeStamp_ms) must be accurate for proper fusion
     */
    void writeEulerYawAngle(float yawAngle, float yawAngleErr, uint32_t timeStamp_ms, uint8_t type);

    /**
     * @brief Write position and attitude data from external navigation system
     *
     * @details Provides absolute position and orientation from an external navigation source
     *          (e.g., motion capture system, UWB positioning, visual SLAM, VICON). Used for
     *          precise indoor positioning or GPS-denied navigation. Data can be in NED frame
     *          or other right-handed navigation frame.
     *
     * @param[in] pos Position [North/X, East/Y, Down/Z] in navigation frame (meters)
     * @param[in] quat Quaternion for rotation from navigation frame to body frame
     * @param[in] posErr 1-sigma spherical position uncertainty (meters)
     * @param[in] angErr 1-sigma spherical attitude uncertainty (radians)
     * @param[in] timeStamp_ms System time measurement was taken, not received (milliseconds)
     * @param[in] delay_ms Average measurement delay relative to IMU (milliseconds)
     * @param[in] resetTime_ms System time of last position reset request (milliseconds)
     * 
     * @note Frame assumed to be NED if configured; otherwise generic right-handed frame
     * @note Units: pos in meters, posErr in meters, angErr in radians
     * @note timeStamp_ms should be measurement time, not reception time (critical for delay compensation)
     * @note resetTime_ms used to detect when external system has reset its reference frame
     * @warning Incorrect delay_ms causes position/velocity inconsistency
     * @warning Position jumps after external system resets require EKF to reinitialize
     */
    void writeExtNavData(const Vector3f &pos, const Quaternion &quat, float posErr, float angErr, uint32_t timeStamp_ms, uint16_t delay_ms, uint32_t resetTime_ms);

    /**
     * @brief Write velocity data from external navigation system
     *
     * @details Provides velocity measurements from an external navigation source to supplement
     *          or replace GPS velocity. Used with external positioning systems that provide
     *          velocity estimates (e.g., visual odometry with motion tracking).
     *
     * @param[in] vel Velocity [VN, VE, VD] in NED frame (m/s)
     * @param[in] err 1-sigma velocity uncertainty (m/s)
     * @param[in] timeStamp_ms System time measurement was taken, not received (milliseconds)
     * @param[in] delay_ms Average measurement delay relative to IMU (milliseconds)
     * 
     * @note Units: vel in m/s (NED frame), err in m/s
     * @note Velocity in NED frame (earth-relative, not body-relative)
     * @note timeStamp_ms must be measurement time for proper delay compensation
     * @warning Ensure velocity is earth-relative (NED), not body-relative
     */
    void writeExtNavVelData(const Vector3f &vel, float err, uint32_t timeStamp_ms, uint16_t delay_ms);

    /**
     * @brief Set terrain height stability flag for rangefinder usage
     *
     * @details Controls whether the terrain underneath is considered stable enough to use
     *          a rangefinder as a height reference. Set to false over moving terrain (e.g.,
     *          landing on a moving platform) or unstable surfaces to prevent rangefinder
     *          from corrupting height estimates.
     *
     * @param[in] val true if terrain stable (enable rangefinder height), false if unstable
     * 
     * @note Overrides rangefinder usage otherwise enabled by EK3_RNG_USE_HGT and EK3_RNG_USE_SPD
     * @note Should be set false when landing on moving platforms (ships, trailers)
     * @note Should be set false over water or tall grass where range may be unreliable
     * @warning Incorrect setting can cause height estimate corruption during landing
     */
    void setTerrainHgtStable(bool val);

    /**
     * @brief Get filter fault status as bitmask
     *
     * @details Returns a bitmask indicating detected filter faults. Used for diagnostics
     *          and failsafe triggering. Each bit represents a specific fault condition.
     *
     * @param[out] faults Bitmask of fault conditions (see bit definitions below)
     * 
     * Bit definitions:
     * - Bit 0: Quaternions are NaN (attitude state corrupted)
     * - Bit 1: Velocities are NaN (velocity state corrupted)
     * - Bit 2: Badly conditioned X magnetometer fusion (numerical issues)
     * - Bit 3: Badly conditioned Y magnetometer fusion (numerical issues)
     * - Bit 4: Badly conditioned Z magnetometer fusion (numerical issues)
     * - Bit 5: Badly conditioned airspeed fusion (numerical issues)
     * - Bit 6: Badly conditioned synthetic sideslip fusion (numerical issues)
     * - Bit 7: Filter is not initialized (not ready for use)
     *
     * @note Faults are aggregated from the primary EKF core
     * @note Non-zero faults indicate degraded estimation or failure
     * @warning Bits 0-1 (NaN states) are critical faults requiring immediate action
     * @warning Bit 7 (not initialized) means position/velocity should not be used
     */
    void getFilterFaults(uint16_t &faults) const;

    /**
     * @brief Get detailed filter status flags
     *
     * @details Returns comprehensive filter status including sensor usage, estimation quality,
     *          and operational mode. Provides detailed health information beyond simple faults.
     *
     * @param[out] status Structure containing detailed status flags
     * 
     * Status includes:
     * - Attitude estimation quality (roll/pitch/yaw validity)
     * - Horizontal/vertical velocity estimation status
     * - Horizontal/vertical position estimation status
     * - Sensor usage flags (GPS, magnetometer, airspeed, optical flow, etc.)
     * - Estimation mode (absolute position, dead reckoning, etc.)
     * - Innovation test status
     *
     * @note Status aggregated from primary EKF core
     * @note Use for detailed diagnostics and sensor health monitoring
     * @see nav_filter_status structure definition for complete field list
     */
    void getFilterStatus(nav_filter_status &status) const;

    /**
     * @brief Send EKF status telemetry to ground control station
     *
     * @details Sends EKF_STATUS_REPORT MAVLink message containing filter health metrics,
     *          innovation test ratios, and velocity/position variances for GCS display
     *          and logging.
     *
     * @param[in] link MAVLink communication link to send message on
     * 
     * @note Called by telemetry system at configured rate (typically 1-5 Hz)
     * @note Message includes data from primary core
     */
    void send_status_report(class GCS_MAVLINK &link) const;

    /**
     * @brief Get height control limit for optical flow navigation
     *
     * @details When using optical flow for navigation, provides a height limit that control
     *          loops should observe. Flow-based velocity estimates degrade with altitude,
     *          so this enforces a maximum height to maintain acceptable accuracy.
     *
     * @param[out] height Maximum height limit (meters above ground)
     * @return true if height limiting is required, false otherwise
     * 
     * @note Limit based on optical flow sensor specifications and configured parameters
     * @note Prevents vehicle from flying too high when using flow for position hold
     * @note Returns false when not using optical flow or when no limit needed
     * @warning Exceeding this limit can cause position control instability with optical flow
     */
    bool getHeightControlLimit(float &height) const;

    /**
     * @brief Get yaw angle change from last reset or lane switch
     *
     * @details Returns the magnitude of yaw angle discontinuity that occurred during the last
     *          yaw reset or core selection switch. Flight controllers use this to compensate
     *          heading-dependent control states and prevent control transients.
     *
     * @param[out] yawAngDelta Yaw angle change (radians, positive = clockwise from above)
     * @return System time of last reset (milliseconds), or 0 if no reset has occurred
     * 
     * @note Returns non-zero only on first call after a reset; subsequent calls return 0
     * @note Yaw resets can occur due to: magnetometer anomalies, lane switches, EKF-GSF resets
     * @note Units: yawAngDelta in radians
     * @warning Flight controllers must compensate for yaw resets to maintain smooth control
     */
    uint32_t getLastYawResetAngle(float &yawAngDelta);

    /**
     * @brief Get horizontal position change from last reset
     *
     * @details Returns the North-East position discontinuity from the last position reset.
     *          Position resets occur when GPS regained after loss, lane switches, or when
     *          switching between different position sources.
     *
     * @param[out] posDelta Position change [ΔN, ΔE] in NED frame (meters)
     * @return System time of last reset (milliseconds), or 0 if no reset has occurred
     * 
     * @note Returns non-zero only on first call after a reset; subsequent calls return 0
     * @note Position resets occur when: GPS reacquired, lane switch, position source change
     * @note Units: posDelta in meters (NED frame)
     * @warning Large position resets can cause control transients; monitor and compensate
     */
    uint32_t getLastPosNorthEastReset(Vector2f &posDelta);

    /**
     * @brief Get horizontal velocity change from last reset
     *
     * @details Returns the North-East velocity discontinuity from the last velocity reset.
     *          Velocity resets occur when measurements resume after prolonged loss or during
     *          lane switches.
     *
     * @param[out] vel Velocity change [ΔVN, ΔVE] in NED frame (m/s)
     * @return System time of last reset (milliseconds), or 0 if no reset has occurred
     * 
     * @note Returns non-zero only on first call after a reset; subsequent calls return 0
     * @note Velocity resets less common than position resets but can occur with sensor recovery
     * @note Units: vel in m/s (NED frame)
     */
    uint32_t getLastVelNorthEastReset(Vector2f &vel) const;

    /**
     * @brief Get vertical position change from last reset
     *
     * @details Returns the Down position discontinuity from the last height reset. Height
     *          resets occur when barometer rezeroed, switching height sources, or during
     *          lane switches.
     *
     * @param[out] posDelta Vertical position change ΔD in NED frame (meters, positive = down)
     * @return System time of last reset (milliseconds), or 0 if no reset has occurred
     * 
     * @note Returns non-zero only on first call after a reset; subsequent calls return 0
     * @note Height resets occur when: baro reset, rangefinder transitions, height source changes
     * @note Units: posDelta in meters (positive = downward in NED frame)
     * @warning Height resets can affect altitude hold and landing sequences
     */
    uint32_t getLastPosDownReset(float &posDelta);

    /**
     * @brief Set and save barometric altitude noise parameter
     *
     * @details Updates the EK3_ALT_NOISE parameter which controls barometer measurement
     *          noise assumption. Higher values reduce trust in barometer, lower values
     *          increase trust. Saved to EEPROM for persistence.
     *
     * @param[in] noise Barometric altitude measurement noise (meters)
     * 
     * @note Typical values: 0.5-2.0 meters depending on barometer quality
     * @note Affects how quickly EKF responds to barometer changes
     * @note Saved to non-volatile memory; survives reboots
     */
    void set_baro_alt_noise(float noise) { _baroAltNoise.set_and_save(noise); };

    /**
     * @brief Set EKF enable flag (for log replay)
     *
     * @details Allows log replay system to enable/disable EKF3 for testing and analysis.
     *          Not typically used during normal flight operations.
     *
     * @param[in] enable true to enable EKF3, false to disable
     * 
     * @note Used by Replay tool for log analysis
     * @note Not saved to EEPROM; temporary setting
     */
    void set_enable(bool enable) { _enable.set_enable(enable); }

    /**
     * @brief Get EKF enable parameter value
     *
     * @details Returns whether EKF3 is enabled via EK3_ENABLE parameter.
     *
     * @return true if EKF3 enabled, false otherwise
     * 
     * @note Reads EK3_ENABLE parameter (0 = disabled, 1 = enabled)
     */
    bool get_enable(void) const { return bool(_enable.get()); }
    
    /**
     * @brief Check if lane switch would reduce innovation errors
     *
     * @details Evaluates whether switching to an alternate EKF core (lane) would improve
     *          innovation consistency. Called when vehicle code is about to trigger an
     *          EKF failsafe; provides opportunity to avoid failsafe by switching to a
     *          healthier lane.
     *
     * @note Compares normalized innovation test ratios across all cores
     * @note Will trigger switchLane() if better core found
     * @note Part of EKF failsafe recovery strategy
     * @warning Should only be called when primary core showing signs of failure
     */
    void checkLaneSwitch(void);

    /**
     * @brief Switch to a different EKF lane (core)
     *
     * @details Forces a switch to the specified EKF core. Updates primary core index and
     *          records position, velocity, and yaw discontinuities for control compensation.
     *          Used for manual lane switching or automated failover.
     *
     * @param[in] new_lane_index Index of core to switch to (0 to num_cores-1)
     * 
     * @note Switching causes discontinuities in state estimates
     * @note Reset tracking updated so controllers can compensate for jumps
     * @note Manual switching available via EK3_OPTIONS bit 1
     * @warning Excessive lane switching indicates systemic issues requiring investigation
     * @warning new_lane_index must be valid (< num_cores) or behavior undefined
     */
    void switchLane(uint8_t new_lane_index);

    /**
     * @brief Request EKF yaw reset to recover from heading errors
     *
     * @details Requests that the EKF reset its yaw estimate, typically using the EKF-GSF
     *          (Gaussian Sum Filter) yaw estimator or magnetometer. Called when vehicle
     *          code about to trigger EKF failsafe due to heading issues; attempts to
     *          recover without full failsafe.
     *
     * @note Uses EKF-GSF yaw estimate if available and reliable
     * @note Yaw reset causes heading discontinuity; controllers must compensate
     * @note Subject to maximum reset count (EK3_GSF_RST_MAX) to prevent repeated resets
     * @note Part of automatic yaw recovery strategy
     * @warning Repeated yaw resets indicate magnetometer or yaw estimation problems
     */
    void requestYawReset(void);

    /**
     * @brief Set active position, velocity, and yaw source set
     *
     * @details Switches between primary, secondary, and tertiary sensor source sets for
     *          position, velocity, and yaw measurements. Allows runtime switching between
     *          GPS, optical flow, external navigation, etc.
     *
     * @param[in] source_set_idx Source set: 0 = primary, 1 = secondary, 2 = tertiary
     * 
     * @note Source sets configured via EK3_SRCn_* parameters
     * @note Switching sources may cause position/velocity/yaw discontinuities
     * @note Used for adaptive sensor selection based on environment
     * @warning Ensure target source set is properly configured before switching
     */
    void setPosVelYawSourceSet(uint8_t source_set_idx);

    /**
     * @brief Write EKF information to onboard dataflash logs
     *
     * @details Writes comprehensive EKF state, innovation, and health data to dataflash
     *          for post-flight analysis. Logs include XKF0-XKF4, XKQ, NKQ, XKY messages.
     *
     * @note Called by logging system at configured rate (typically 10-50 Hz)
     * @note Log detail level controlled by EK3_LOG_LEVEL parameter
     * @note Essential for post-flight analysis and debugging
     * @see EK3_LOG_LEVEL parameter (0=all, 1=XKF4 only, 2=XKF4+GSF, 3=none)
     */
    void Log_Write();

    /**
     * @brief Check if using non-compass yaw source
     *
     * @details Returns whether EKF is fusing yaw from a source other than magnetometer
     *          (e.g., GPS heading, external nav, optical flow).
     *
     * @return true if fusing non-compass yaw, false if using magnetometer
     * 
     * @note Used for compass health monitoring and declination handling
     * @note Non-compass sources include: GPS heading, external nav, GSF yaw estimator
     */
    bool using_noncompass_for_yaw() const;

    /**
     * @brief Check if using external navigation for yaw
     *
     * @details Returns whether EKF is fusing yaw/heading from external navigation system
     *          (e.g., motion capture, vision-based pose estimation).
     *
     * @return true if fusing external nav yaw, false otherwise
     * 
     * @note Indicates reliance on external heading source
     * @note External nav typically more accurate than magnetometer indoors
     */
    bool using_extnav_for_yaw() const;

    /**
     * @brief Check if configured to use GPS for horizontal position
     *
     * @details Returns whether EKF is configured to use GPS as a horizontal position source
     *          based on EK3_SRC_OPTIONS settings.
     *
     * @return true if GPS configured for horizontal position, false otherwise
     * 
     * @note Configuration check only; does not indicate GPS currently in use
     * @note Used for pre-flight checks and mode restrictions
     */
    bool configuredToUseGPSForPosXY(void) const;
    
    /**
     * @brief Write default airspeed for use when sensor unavailable
     *
     * @details Provides a fallback equivalent airspeed and uncertainty for use when
     *          airspeed sensor required but not available or unhealthy. Used in fixed-wing
     *          forward flight to maintain reasonable airspeed estimates.
     *
     * @param[in] airspeed Default equivalent airspeed (m/s)
     * @param[in] uncertainty 1-sigma airspeed uncertainty (m/s)
     * 
     * @note Only used when airspeed required but sensor unavailable
     * @note Typical defaults: 15-25 m/s depending on aircraft
     * @note High uncertainty reduces reliance on default value
     * @warning Fixed-wing aircraft should always have functioning airspeed sensor
     */
    void writeDefaultAirSpeed(float airspeed, float uncertainty);

    /**
     * @brief Convert legacy parameters to current format
     *
     * @details Converts parameters from older EKF versions or parameter formats to current
     *          EK3 format. Called during initialization to maintain backwards compatibility.
     *
     * @note Automatically called during initialization
     * @note Handles migration from EKF2 parameters
     * @note Saves converted parameters to EEPROM
     */
    void convert_parameters();

    /**
     * @brief Check if yaw alignment completed
     *
     * @details Returns whether initial yaw alignment has completed. Yaw must be aligned
     *          before position estimates are reliable. Alignment uses magnetometer and/or
     *          GPS velocity direction.
     *
     * @return true if yaw aligned, false if still aligning
     * 
     * @note Yaw alignment required before GPS position fusion can begin
     * @note Typically completes within 5-10 seconds of initialization
     * @note Alignment failure prevents arming
     * @warning Vehicle should not arm until yaw alignment complete
     */
    bool yawAlignmentComplete(void) const;

    /**
     * @brief Check if vibration is degrading state estimates
     *
     * @details Returns whether excessive vibration is significantly affecting EKF state
     *          estimates. High vibration causes accelerometer/gyro noise, degrading
     *          navigation performance.
     *
     * @return true if vibration affecting estimates, false if acceptable
     * 
     * @note Based on IMU delta velocity and delta angle consistency
     * @note High vibration can cause attitude and position drift
     * @note Indicates mechanical issues requiring attention
     * @warning Persistent vibration warnings indicate need for mechanical inspection
     * @warning Flight performance degraded when vibration affecting EKF
     */
    bool isVibrationAffected() const;

    /**
     * @brief Get pointer to EKF-GSF yaw estimator instance
     *
     * @details Returns pointer to the Gaussian Sum Filter yaw estimator used for
     *          improved yaw initialization and recovery. GSF runs independently of
     *          main EKF and provides yaw estimates when magnetometer unreliable.
     *
     * @return Pointer to EKFGSF_yaw instance, or nullptr if not available
     * 
     * @note GSF yaw estimator runs when enabled via EK3_GSF_RUN_MASK
     * @note Provides robust yaw estimates using IMU and velocity measurements
     * @note Used for automatic yaw resets when main EKF yaw diverges
     * @see EKFGSF_yaw class for GSF implementation details
     */
    const EKFGSF_yaw *get_yawEstimator(void) const;

private:
    class AP_DAL &dal;

    uint8_t num_cores; // number of allocated cores
    uint8_t primary;   // current primary core
    NavEKF3_core *core = nullptr;

    uint32_t _frameTimeUsec;        // time per IMU frame
    uint8_t  _framesPerPrediction;  // expected number of IMU frames per prediction
  
    /**
     * @brief EKF3 log verbosity levels
     * 
     * @details Controls amount of EKF3 debug data written to onboard logs
     */
    enum class LogLevel {
        ALL = 0,        ///< Log all EKF3 messages
        XKF4 = 1,       ///< Log only XKF4 messages
        XKF4_GSF = 2,   ///< Log XKF4 and GSF messages
        NONE = 3        ///< Disable EKF3 logging
    };

    // ========================================================================
    // EKF3 Tunable Parameters (AP_Param)
    // ========================================================================
    // These parameters are exposed to ground control stations and can be
    // adjusted for different vehicle configurations and operating conditions.
    // All parameters prefixed with EK3_ in parameter lists.
    // Units and ranges documented inline for each parameter.
    // ========================================================================

    /** @brief Enable/disable EKF3 (0=disabled, 1=enabled) */
    AP_Int8  _enable;               // zero to disable EKF3

    // ---- Sensor Measurement Noise Parameters ----
    // These parameters define the expected 1-sigma measurement noise for each sensor type.
    // Larger values make EKF trust sensor less, smaller values trust sensor more.
    // Should be set based on sensor datasheet specifications and observed performance.

    /** @brief GPS horizontal velocity measurement noise (m/s, 1-sigma) */
    AP_Float _gpsHorizVelNoise;     // GPS horizontal velocity measurement noise : m/s
    
    /** @brief GPS vertical velocity measurement noise (m/s, 1-sigma) */
    AP_Float _gpsVertVelNoise;      // GPS vertical velocity measurement noise : m/s
    
    /** @brief GPS horizontal position measurement noise (m, 1-sigma) */
    AP_Float _gpsHorizPosNoise;     // GPS horizontal position measurement noise m
    
    /** @brief Barometric altitude measurement noise (m, 1-sigma) */
    AP_Float _baroAltNoise;         // Baro height measurement noise : m
    
    /** @brief Magnetometer measurement noise (gauss, 1-sigma) */
    AP_Float _magNoise;             // magnetometer measurement noise : gauss
    
    /** @brief Airspeed sensor measurement noise (m/s, 1-sigma) */
    AP_Float _easNoise;             // equivalent airspeed measurement noise : m/s

    // ---- Process Noise Parameters ----
    // These parameters define the expected rate of change (process noise) for EKF states.
    // Higher values allow states to change more quickly but may cause instability.
    // Lower values provide smoother estimates but slower adaptation to changes.

    /** @brief Wind velocity state process noise (m/s^2, 1-sigma rate of change) */
    AP_Float _windVelProcessNoise;  // wind velocity state process noise : m/s^2
    
    /** @brief Wind process noise height rate scaling factor (dimensionless) */
    AP_Float _wndVarHgtRateScale;   // scale factor applied to wind process noise due to height rate
    
    /** @brief Earth magnetic field process noise (gauss/s, 1-sigma rate of change) */
    AP_Float _magEarthProcessNoise; // Earth magnetic field process noise : gauss/sec
    
    /** @brief Body magnetic field process noise (gauss/s, 1-sigma rate of change) */
    AP_Float _magBodyProcessNoise;  // Body magnetic field process noise : gauss/sec
    
    /** @brief Gyroscope process noise (rad/s, 1-sigma) */
    AP_Float _gyrNoise;             // gyro process noise : rad/s
    
    /** @brief Accelerometer process noise (m/s^2, 1-sigma) */
    AP_Float _accNoise;             // accelerometer process noise : m/s^2
    
    /** @brief Gyro bias state process noise (rad/s, 1-sigma rate of change) */
    AP_Float _gyroBiasProcessNoise; // gyro bias state process noise : rad/s
    
    /** @brief Accelerometer bias state process noise (m/s^2, 1-sigma rate of change) */
    AP_Float _accelBiasProcessNoise;// accel bias state process noise : m/s^2

    // ---- Measurement Timing Delays ----
    
    /** @brief Height measurement delay relative to IMU (milliseconds) */
    AP_Int16 _hgtDelay_ms;          // effective average delay of Height measurements relative to inertial measurements (msec)

    // ---- Innovation Gate Parameters ----
    // Innovation gates reject measurements with innovations (measurement - predicted)
    // larger than the specified number of standard deviations. Prevents bad measurements
    // from corrupting state estimates. Values are percentages of sigma (e.g., 300 = 3-sigma).

    /** @brief GPS velocity innovation gate (percentage of sigma, e.g., 300 = 3-sigma) */
    AP_Int16  _gpsVelInnovGate;     // Percentage number of standard deviations applied to GPS velocity innovation consistency check
    
    /** @brief GPS position innovation gate (percentage of sigma, e.g., 300 = 3-sigma) */
    AP_Int16  _gpsPosInnovGate;     // Percentage number of standard deviations applied to GPS position innovation consistency check
    
    /** @brief Height innovation gate (percentage of sigma, e.g., 300 = 3-sigma) */
    AP_Int16  _hgtInnovGate;        // Percentage number of standard deviations applied to height innovation consistency check
    
    /** @brief Magnetometer innovation gate (percentage of sigma, e.g., 300 = 3-sigma) */
    AP_Int16  _magInnovGate;        // Percentage number of standard deviations applied to magnetometer innovation consistency check
    
    /** @brief Airspeed innovation gate (percentage of sigma, e.g., 300 = 3-sigma) */
    AP_Int16  _tasInnovGate;        // Percentage number of standard deviations applied to true airspeed innovation consistency check

    // ---- Magnetometer and GPS Configuration ----
    
    /** @brief In-flight magnetometer calibration activation (bitmask) */
    AP_Int8  _magCal;               // Sets activation condition for in-flight magnetometer calibration
    
    /** @brief GPS glitch detection radius threshold (m) */
    AP_Int8 _gpsGlitchRadiusMax;    // Maximum allowed discrepancy between inertial and GPS Horizontal position before GPS glitch is declared : m

    // ---- Optical Flow Parameters ----
    
    /** @brief Optical flow measurement noise (rad/s, 1-sigma) */
    AP_Float _flowNoise;            // optical flow rate measurement noise
    
    /** @brief Optical flow innovation gate (percentage of sigma, e.g., 300 = 3-sigma) */
    AP_Int16  _flowInnovGate;       // Percentage number of standard deviations applied to optical flow innovation consistency check
    
    /** @brief Optical flow measurement delay relative to IMU (milliseconds) */
    AP_Int8  _flowDelay_ms;         // effective average delay of optical flow measurements rel to IMU (msec)
    
    /** @brief Maximum optical flow rate accepted by filter (rad/s) */
    AP_Float _maxFlowRate;          // Maximum flow rate magnitude that will be accepted by the filter

    // ---- Rangefinder Parameters ----
    
    /** @brief Rangefinder innovation gate (percentage of sigma, e.g., 300 = 3-sigma) */
    AP_Int16  _rngInnovGate;        // Percentage number of standard deviations applied to range finder innovation consistency check
    
    /** @brief Rangefinder measurement noise (m, 1-sigma) */
    AP_Float _rngNoise;             // Range finder noise : m

    // ---- GPS and IMU Selection ----
    
    /** @brief GPS preflight check bypass bitmask */
    AP_Int8 _gpsCheck;              // Bitmask controlling which preflight GPS checks are bypassed
    
    /** @brief IMU instance selection bitmask for EKF3 cores */
    AP_Int8 _imuMask;               // Bitmask of IMUs to instantiate EKF3 for
    
    /** @brief GPS check threshold scaler (percentage, e.g., 100 = 1x, 150 = 1.5x) */
    AP_Int16 _gpsCheckScaler;       // Percentage increase to be applied to GPS pre-flight accuracy and drift thresholds

    // ---- Non-Aiding and Yaw Parameters ----
    
    /** @brief Horizontal position noise for non-aiding mode (m, 1-sigma) */
    AP_Float _noaidHorizNoise;      // horizontal position measurement noise assumed when synthesised zero position measurements are used to constrain attitude drift : m
    
    /** @brief Magnetic yaw measurement noise (rad, 1-sigma) */
    AP_Float _yawNoise;             // magnetic yaw measurement noise : rad
    
    /** @brief Yaw innovation gate (percentage of sigma, e.g., 300 = 3-sigma) */
    AP_Int16 _yawInnovGate;         // Percentage number of standard deviations applied to magnetic yaw innovation consistency check

    // ---- Output Filtering ----
    
    /** @brief Output complementary filter time constant (centiseconds, e.g., 25 = 0.25s) */
    AP_Int8 _tauVelPosOutput;       // Time constant of output complementary filter : csec (centi-seconds)

    // ---- Rangefinder Height Source Parameters ----
    
    /** @brief Rangefinder max range usage threshold (% of sensor max range, e.g., 70 = 70%) */
    AP_Int8 _useRngSwHgt;           // Maximum valid range of the range finder as a percentage of the maximum range specified by the sensor driver

    // ---- Terrain Parameters ----
    
    /** @brief Maximum terrain gradient (dimensionless, e.g., 0.2 = 20% slope) */
    AP_Float _terrGradMax;          // Maximum terrain gradient below the vehicle

    // ---- Range Beacon Parameters ----
    
    /** @brief Range beacon measurement noise (m, 1-sigma) */
    AP_Float _rngBcnNoise;          // Range beacon measurement noise (m)
    
    /** @brief Range beacon innovation gate (percentage of sigma, e.g., 300 = 3-sigma) */
    AP_Int16 _rngBcnInnovGate;      // Percentage number of standard deviations applied to range beacon innovation consistency check
    
    /** @brief Range beacon measurement delay relative to IMU (milliseconds) */
    AP_Int8  _rngBcnDelay_ms;       // effective average delay of range beacon measurements rel to IMU (msec)

    // ---- Speed Threshold Parameters ----
    
    /** @brief Maximum horizontal speed for rangefinder as primary height (m/s) */
    AP_Float _useRngSwSpd;          // Maximum horizontal ground speed to use range finder as the primary height source (m/s)

    // ---- Accelerometer Bias Limits ----
    
    /** @brief Maximum allowed accelerometer bias magnitude (m/s^2) */
    AP_Float _accBiasLim;           // Accelerometer bias limit (m/s/s)

    // ---- Core-Specific Configuration ----
    
    /** @brief Magnetometer fusion mode bitmask (forces simple heading fusion for specified cores) */
    AP_Int8 _magMask;               // Bitmask forcing specific EKF core instances to use simple heading magnetometer fusion.
    
    /** @brief Origin height reporting mode bitmask */
    AP_Int8 _originHgtMode;         // Bitmask controlling post alignment correction and reporting of the EKF origin height.

    // ---- Visual and Wheel Odometry Parameters ----
    
    /** @brief Visual odometry velocity error at minimum quality (m/s, 1-sigma) */
    AP_Float _visOdmVelErrMax;      // Observation 1-STD velocity error assumed for visual odometry sensor at lowest reported quality (m/s)
    
    /** @brief Visual odometry velocity error at maximum quality (m/s, 1-sigma) */
    AP_Float _visOdmVelErrMin;      // Observation 1-STD velocity error assumed for visual odometry sensor at highest reported quality (m/s)
    
    /** @brief Wheel odometry velocity error (m/s, 1-sigma) */
    AP_Float _wencOdmVelErr;        // Observation 1-STD velocity error assumed for wheel odometry sensor (m/s)

    // ---- Flow Usage Configuration ----
    
    /** @brief Optical flow usage mode (0=none, 1=navigation, 2=terrain) */
    AP_Int8  _flowUse;              // Controls if the optical flow data is fused into the main navigation estimator and/or the terrain estimator.

    // ---- Output Observer Filter Parameters ----
    
    /** @brief Height rate complementary filter frequency (Hz) */
    AP_Float _hrt_filt_freq;        // frequency of output observer height rate complementary filter in Hz

    // ---- Magnetometer Earth Field Parameters ----
    
    /** @brief Maximum deviation from WMM earth field model (milligauss) */
    AP_Int16 _mag_ef_limit;         // limit on difference between WMM tables and learned earth field.

    // ---- EKF-GSF (Gaussian Sum Filter) Yaw Estimator Parameters ----
    
    /** @brief EKF-GSF run mask (bitmask of cores that run GSF yaw estimator) */
    AP_Int8 _gsfRunMask;            // mask controlling which EKF3 instances run a separate EKF-GSF yaw estimator
    
    /** @brief EKF-GSF use mask (bitmask of cores that use GSF for yaw resets) */
    AP_Int8 _gsfUseMask;            // mask controlling which EKF3 instances will use EKF-GSF yaw estimator data to assit with yaw resets
    
    /** @brief Maximum EKF-GSF yaw resets allowed per core */
    AP_Int8 _gsfResetMaxCount;      // maximum number of times the EKF3 is allowed to reset it's yaw to the EKF-GSF estimate

    // ---- Multi-Core Lane Selection Parameters ----
    
    /** @brief Lane switch error threshold (dimensionless relative error) */
    AP_Float _err_thresh;           // lanes have to be consistently better than the primary by at least this threshold to reduce their overall relativeCoreError

    // ---- Sensor Affinity and Drag Parameters ----
    
    /** @brief Sensor affinity options bitmask */
    AP_Int32 _affinity;             // bitmask of sensor affinity options
    
    /** @brief Drag specific force observation noise ((m/s^2)^2) */
    AP_Float _dragObsNoise;         // drag specific force observatoin noise (m/s/s)**2
    
    /** @brief Ballistic coefficient for X body axis drag (kg/m^2) */
    AP_Float _ballisticCoef_x;      // ballistic coefficient measured for flow in X body frame directions
    
    /** @brief Ballistic coefficient for Y body axis drag (kg/m^2) */
    AP_Float _ballisticCoef_y;      // ballistic coefficient measured for flow in Y body frame directions
    
    /** @brief Rotor momentum drag coefficient (1/s) */
    AP_Float _momentumDragCoef;     // lift rotor momentum drag coefficient

    // ---- Sideslip Fusion Parameters ----
    
    /** @brief Sideslip fusion control bitmask */
    AP_Int8 _betaMask;              // Bitmask controlling when sideslip angle fusion is used to estimate non wind states

    // ---- Ground Detection Parameters ----
    
    /** @brief On-ground-not-moving test threshold scale factor (dimensionless) */
    AP_Float _ognmTestScaleFactor;  // Scale factor applied to the thresholds used by the on ground not moving test
    
    /** @brief Barometer ground effect dead zone (m) */
    AP_Float _baroGndEffectDeadZone;// Dead zone applied to positive baro height innovations when in ground effect (m)

    // ---- Core Selection and Logging ----
    
    /** @brief Initial primary core selection (0-2, or -1 for automatic) */
    AP_Int8 _primary_core;          // initial core number
    
    /** @brief EKF3 logging verbosity level */
    AP_Enum<LogLevel> _log_level;   // log verbosity level

    // ---- GPS Altitude Source Configuration ----
    
    /** @brief GPS vertical accuracy threshold for altitude usage (m) */
    AP_Float _gpsVAccThreshold;     // vertical accuracy threshold to use GPS as an altitude source

    // ---- Processing Options ----
    
    /** @brief EKF3 processing options bitmask */
    AP_Int32 _options;              // bit mask of processing options

    /**
     * @brief EKF3 processing option flags
     * 
     * @details Bit flags for controlling special EKF3 processing modes
     */
    enum class Option {
        JammingExpected     = (1<<0),  ///< Expect GPS jamming, use more conservative checks
        ManualLaneSwitch   = (1<<1),   ///< Disable automatic lane switching
    };
    
    /**
     * @brief Check if a processing option is enabled
     * 
     * @param[in] option The Option flag to check
     * @return true if the option bit is set in _options, false otherwise
     */
    bool option_is_enabled(Option option) const {
        return (_options & (uint32_t)option) != 0;
    }

    // ========================================================================
    // Optical Flow Usage Mode Constants
    // ========================================================================
    
    /** @brief Optical flow disabled */
#define FLOW_USE_NONE    0
    /** @brief Optical flow used for navigation state estimation */
#define FLOW_USE_NAV     1
    /** @brief Optical flow used for terrain height estimation */
#define FLOW_USE_TERRAIN 2

    // ========================================================================
    // EKF3 Fixed Tuning Constants
    // ========================================================================
    // These are compile-time constants that define EKF behavior but are not
    // user-adjustable parameters. They represent carefully tuned values derived
    // from analysis and flight testing.
    // ========================================================================

    // ---- Measurement Variance Scaling ----
    
    /** @brief GPS NE velocity variance scale factor due to maneuver acceleration (dimensionless) */
    const float gpsNEVelVarAccScale = 0.05f;       // Scale factor applied to NE velocity measurement variance due to manoeuvre acceleration
    
    /** @brief GPS D velocity variance scale factor due to maneuver acceleration (dimensionless) */
    const float gpsDVelVarAccScale = 0.07f;        // Scale factor applied to vertical velocity measurement variance due to manoeuvre acceleration
    
    /** @brief GPS position variance scale factor due to maneuver acceleration (dimensionless) */
    const float gpsPosVarAccScale = 0.05f;         // Scale factor applied to horizontal position measurement variance due to manoeuvre acceleration
    
    /** @brief External nav velocity variance scale factor due to maneuver acceleration (dimensionless) */
    const float extNavVelVarAccScale = 0.05f;      // Scale factor applied to ext nav velocity measurement variance due to manoeuvre acceleration

    // ---- Fixed Measurement Delays ----
    
    /** @brief Magnetometer measurement delay (milliseconds) */
    const uint16_t magDelay_ms = 60;               // Magnetometer measurement delay (msec)
    
    /** @brief True airspeed measurement delay (milliseconds) */
    const uint16_t tasDelay_ms = 100;              // Airspeed measurement delay (msec)

    // ---- Aiding Timeout Thresholds ----
    
    /** @brief Maximum time without tilt aiding before declaring failure (milliseconds) */
    const uint16_t tiltDriftTimeMax_ms = 15000;    // Maximum number of ms allowed without any form of tilt aiding (GPS, flow, TAS, etc)
    
    /** @brief Position aiding retry time with velocity (milliseconds) */
    const uint16_t posRetryTimeUseVel_ms = 10000;  // Position aiding retry time with velocity measurements (msec)
    
    /** @brief Position aiding retry time without velocity (milliseconds) */
    const uint16_t posRetryTimeNoVel_ms = 7000;    // Position aiding retry time without velocity measurements (msec)
    
    /** @brief Height retry time with vertical velocity (milliseconds) */
    const uint16_t hgtRetryTimeMode0_ms = 10000;   // Height retry time with vertical velocity measurement (msec)
    
    /** @brief Height retry time without vertical velocity (milliseconds) */
    const uint16_t hgtRetryTimeMode12_ms = 5000;   // Height retry time without vertical velocity measurement (msec)
    
    /** @brief Airspeed timeout and retry interval (milliseconds) */
    const uint16_t tasRetryTime_ms = 5000;         // True airspeed timeout and retry interval (msec)
    
    /** @brief Drag measurement timeout (milliseconds) */
    const uint16_t dragFailTimeLimit_ms = 5000;    // Drag timeout (msec)
    
    /** @brief Magnetometer failure declaration timeout (milliseconds) */
    const uint32_t magFailTimeLimit_ms = 10000;    // number of msec before a magnetometer failing innovation consistency checks is declared failed (msec)

    // ---- Process Noise Scaling Factors ----
    
    /** @brief Magnetometer variance scaling due to angular rate (dimensionless) */
    const float magVarRateScale = 0.005f;          // scale factor applied to magnetometer variance due to angular rate
    
    /** @brief Gyro bias noise scaling factor when on ground (dimensionless) */
    const float gyroBiasNoiseScaler = 2.0f;        // scale factor applied to gyro bias state process noise when on ground

    // ---- Measurement Timing Constants ----
    
    /** @brief Average time between height measurements (milliseconds) */
    const uint16_t hgtAvg_ms = 100;                // average number of msec between height measurements
    
    /** @brief Average time between synthetic sideslip measurements (milliseconds) */
    const uint16_t betaAvg_ms = 100;               // average number of msec between synthetic sideslip measurements

    // ---- Covariance Prediction Limits ----
    
    /** @brief Maximum time step for covariance prediction (seconds) */
    const float covTimeStepMax = 0.1f;             // maximum time (sec) between covariance prediction updates
    
    /** @brief Maximum delta angle for covariance prediction (radians) */
    const float covDelAngMax = 0.05f;              // maximum delta angle between covariance prediction updates

    // ---- Optical Flow Constraints ----
    
    /** @brief Minimum DCM(3,3) value for optical flow fusion (cosine of max tilt angle) */
    const float DCM33FlowMin = 0.71f;              // If Tbn(3,3) is less than this number, optical flow measurements will not be fused as tilt is too high.
    
    /** @brief Focal length scale factor process noise (dimensionless variance) */
    const float fScaleFactorPnoise = 1e-10f;       // Process noise added to focal length scale factor state variance at each time step
    
    /** @brief Average optical flow measurement interval (milliseconds) */
    const uint8_t flowTimeDeltaAvg_ms = 100;       // average interval between optical flow measurements (msec)
    
    /** @brief Maximum time between optical flow fusion events (milliseconds) */
    const uint32_t flowIntervalMax_ms = 100;       // maximum allowable time between flow fusion events

    // ---- Ground Effect and Terrain ----
    
    /** @brief Barometer variance scaling factor during ground effect (dimensionless) */
    const float gndEffectBaroScaler = 4.0f;        // scaler applied to the barometer observation variance when ground effect mode is active
    
    /** @brief RMS terrain gradient for terrain estimation (percentage) */
    const uint8_t gndGradientSigma = 50;           // RMS terrain gradient percentage assumed by the terrain height estimation

    // ---- Fusion Timing Constraints ----
    
    /** @brief Minimum time between covariance predictions and fusions (milliseconds) */
    const uint16_t fusionTimeStep_ms = 10;         // The minimum time interval between covariance predictions and measurement fusions in msec
    
    /** @brief Minimum time between non-IMU sensor measurements (milliseconds) */
    const uint8_t sensorIntervalMin_ms = 50;       // The minimum allowed time between measurements from any non-IMU sensor (msec)
    
    /** @brief Minimum time between optical flow measurements (milliseconds) */
    const uint8_t flowIntervalMin_ms = 20;         // The minimum allowed time between measurements from optical flow sensors (msec)
    
    /** @brief Minimum time between external nav measurements (milliseconds) */
    const uint8_t extNavIntervalMin_ms = 20;       // The minimum allowed time between measurements from external navigation sensors (msec)

    // ---- EKF-GSF Yaw Estimator Thresholds ----
    
    /** @brief Maximum acceptable GSF yaw estimator velocity innovation (m/s) */
    const float maxYawEstVelInnov = 2.0f;          // Maximum acceptable length of the velocity innovation returned by the EKF-GSF yaw estimator (m/s)

    // ---- Dead Reckoning and GPS Alignment ----
    
    /** @brief Time to declare dead reckoning mode (milliseconds) */
    const uint16_t deadReckonDeclare_ms = 1000;    // Time without equivalent position or velocity observation to constrain drift before dead reckoning is declared (msec)
    
    /** @brief GPS fix timeout for alignment check reset (milliseconds) */
    const uint16_t gpsNoFixTimeout_ms = 2000;      // Time without a fix required to reset GPS alignment checks when EK3_OPTIONS bit 0 is set (msec)

    // ========================================================================
    // Private State Tracking Variables
    // ========================================================================

    /** @brief IMU sample time at start of current filter update (microseconds) */
    uint64_t imuSampleTime_us;

    /** @brief Time of last lane (core) switch (milliseconds) */
    uint32_t lastLaneSwitch_ms;

    /** @brief Last time telemetry logging was written (microseconds) */
    uint64_t lastLogWrite_us;

    /**
     * @brief Yaw angle reset tracking structure
     * 
     * @details Tracks yaw angle changes due to EKF resets or lane switches
     *          to enable external consumers to compensate for discontinuities
     */
    struct {
        uint32_t last_function_call;  ///< Last time getLastYawResetAngle() was called (msec)
        bool core_changed;            ///< True when a core change occurred and hasn't been consumed
        uint32_t last_primary_change; ///< Last time primary core changed (msec)
        float core_delta;             ///< Yaw change between cores when switch occurred (rad)
    } yaw_reset_data;

    /**
     * @brief North-East position reset tracking structure
     * 
     * @details Tracks NE position changes due to EKF resets or lane switches
     *          to enable external consumers to compensate for discontinuities
     */
    struct {
        uint32_t last_function_call;  ///< Last time getLastPosNorthEastReset() was called (msec)
        bool core_changed;            ///< True when a core change occurred and hasn't been consumed
        uint32_t last_primary_change; ///< Last time primary core changed (msec)
        Vector2f core_delta;          ///< NE position change between cores when switch occurred (m)
    } pos_reset_data;

    /**
     * @brief Down position reset tracking structure
     * 
     * @details Tracks D position changes due to EKF resets or lane switches
     *          to enable external consumers to compensate for discontinuities
     */
    struct {
        uint32_t last_function_call;  ///< Last time getLastPosDownReset() was called (msec)
        bool core_changed;            ///< True when a core change occurred and hasn't been consumed
        uint32_t last_primary_change; ///< Last time primary core changed (msec)
        float core_delta;             ///< D position change between cores when switch occurred (m)
    } pos_down_reset_data;

    // ========================================================================
    // Multi-Core Lane Selection Constants
    // ========================================================================
    
    /** @brief Relative error range limit for lane scoring (dimensionless, normalized to -1 to +1) */
#define CORE_ERR_LIM      1 // -LIM to LIM relative error range for a core
    
    /** @brief Minimum relative error improvement to consider lane switch (dimensionless) */
#define BETTER_THRESH   0.5 // a lane should have this much relative error difference to be considered for overriding a healthy primary core

    // ========================================================================
    // Multi-Core State Management
    // ========================================================================
    
    /** @brief Enable flag for core selection logic (true after primary stabilizes) */
    bool runCoreSelection;                          // true when the primary core has stabilised and the core selection logic can be started
    
    /** @brief Array indicating which cores need initialization */
    bool coreSetupRequired[MAX_EKF_CORES];          // true when this core index needs to be setup
    
    /** @brief IMU index assigned to each EKF core instance */
    uint8_t coreImuIndex[MAX_EKF_CORES];            // IMU index used by this core
    
    /** @brief Relative error scores for each core compared to primary (dimensionless) */
    float coreRelativeErrors[MAX_EKF_CORES];        // relative errors of cores with respect to primary
    
    /** @brief Instantaneous error scores for each core (dimensionless) */
    float coreErrorScores[MAX_EKF_CORES];           // the instance error values used to update relative core error
    
    /** @brief Last time each core was used as primary (microseconds) */
    uint64_t coreLastTimePrimary_us[MAX_EKF_CORES]; // last time we were using this core as primary

    // ========================================================================
    // Common EKF Origin
    // ========================================================================
    
    /** @brief Common NED origin location shared by all cores */
    Location common_EKF_origin;
    
    /** @brief Flag indicating if common origin has been set */
    bool common_origin_valid;

    // ========================================================================
    // Private Helper Methods - Lane Switch Reset Tracking
    // ========================================================================
    
    /**
     * @brief Update yaw reset tracking data after a lane switch
     * 
     * @param[in] new_primary Index of EKF core becoming the new primary (0-2)
     * @param[in] old_primary Index of EKF core currently serving as primary (0-2)
     */
    void updateLaneSwitchYawResetData(uint8_t new_primary, uint8_t old_primary);

    /**
     * @brief Update NE position reset tracking data after a lane switch
     * 
     * @param[in] new_primary Index of EKF core becoming the new primary (0-2)
     * @param[in] old_primary Index of EKF core currently serving as primary (0-2)
     */
    void updateLaneSwitchPosResetData(uint8_t new_primary, uint8_t old_primary);

    /**
     * @brief Update down position reset tracking data after a lane switch
     * 
     * @param[in] new_primary Index of EKF core becoming the new primary (0-2)
     * @param[in] old_primary Index of EKF core currently serving as primary (0-2)
     */
    void updateLaneSwitchPosDownResetData(uint8_t new_primary, uint8_t old_primary);

    // ========================================================================
    // Private Helper Methods - Multi-Core Error Scoring
    // ========================================================================

    /**
     * @brief Update instantaneous error scores for all active cores
     * 
     * @details Calculates normalized innovation-based error metrics for each
     *          running EKF core instance to support lane selection logic
     * 
     * @return Maximum error score among all cores (dimensionless)
     */
    float updateCoreErrorScores(void);

    /**
     * @brief Update relative error scores for non-primary cores
     * 
     * @details Computes relative performance of each lane compared to the
     *          current primary core, filtered over time for stability
     */
    void updateCoreRelativeErrors(void);

    /**
     * @brief Reset error scores for all cores
     * 
     * @details Clears accumulated error metrics, typically called after
     *          lane switches or major EKF resets
     */
    void resetCoreErrors(void);

    /**
     * @brief Evaluate if a candidate core has better performance than current core
     * 
     * @param[in] new_core Index of candidate core to evaluate (0-2)
     * @param[in] current_core Index of current core for comparison (0-2)
     * 
     * @return true if new_core is better and properly aligned, false otherwise
     * 
     * @details Compares error scores and verifies alignment status before
     *          recommending a lane switch to prevent switching to uninitialized cores
     */
    bool coreBetterScore(uint8_t new_core, uint8_t current_core) const;

    // ========================================================================
    // Source Selection Control
    // ========================================================================
    
    /** @brief Position, velocity, and yaw measurement source selection manager */
    AP_NavEKF_Source sources;
};
