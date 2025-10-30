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
 * @file AP_AHRS_DCM.h
 * @brief Direction Cosine Matrix (DCM) based AHRS backend for ArduPilot
 * 
 * @details This file implements a DCM-based Attitude Heading Reference System backend
 *          that estimates vehicle attitude using Direction Cosine Matrix representation
 *          without requiring Extended Kalman Filter (EKF) algorithms. The DCM approach
 *          integrates gyroscope data and corrects drift using accelerometer and compass/GPS
 *          measurements through complementary filtering.
 * 
 *          The DCM backend is suitable for:
 *          - GPS-denied environments where EKF convergence is challenging
 *          - Simple applications requiring lower computational overhead
 *          - Backup/fallback attitude estimation when EKF is unavailable
 *          - Platforms with limited processing power
 * 
 *          Key characteristics:
 *          - Attitude estimation using 3x3 rotation matrix (DCM)
 *          - PI controller for gyro drift correction
 *          - Dead-reckoning position estimation from last GPS fix
 *          - Wind estimation from GPS velocity and airspeed
 * 
 * @note DCM provides less accurate position and velocity estimates compared to EKF
 * @warning DCM drift increases significantly without GPS corrections
 */

#include "AP_AHRS_config.h"

#if AP_AHRS_DCM_ENABLED

#include "AP_AHRS_Backend.h"

/**
 * @class AP_AHRS_DCM
 * @brief Direction Cosine Matrix attitude estimation backend
 * 
 * @details Implements attitude estimation using Direction Cosine Matrix (DCM) algorithm,
 *          providing a lightweight alternative to EKF-based estimation. The DCM approach
 *          represents vehicle attitude as a 3x3 rotation matrix and maintains orthogonality
 *          through renormalization. Drift correction is achieved using PI controllers that
 *          integrate accelerometer, compass, and GPS measurements.
 * 
 *          Algorithm Overview:
 *          1. matrix_update(): Integrate gyro rates into DCM matrix
 *          2. normalize(): Renormalize matrix to maintain orthogonality
 *          3. drift_correction(): Apply PI control using accel/magnetometer/GPS
 * 
 *          Coordinate Frames:
 *          - NED (North-East-Down): Earth-fixed reference frame
 *          - Body Frame: Vehicle forward-right-down axes
 *          - DCM matrix represents rotation from NED to body frame
 * 
 * @note Suitable for applications without EKF or as backup attitude source
 * @warning Less accurate than EKF for position/velocity estimation
 * @warning Requires GPS or compass for yaw initialization
 * @warning Matrix can become ill-conditioned; watchdog recovery uses Euler backup
 */
class AP_AHRS_DCM : public AP_AHRS_Backend {
public:

    /**
     * @brief Construct DCM-based AHRS backend with tuning parameters
     * 
     * @details Initializes the DCM AHRS backend with references to tuning parameters
     *          that control drift correction behavior. The DCM matrix is initialized
     *          to identity (level attitude), and drift correction gains determine how
     *          quickly the algorithm responds to accelerometer and compass/GPS inputs.
     * 
     *          Tuning Parameter Roles:
     *          - kp_yaw: Proportional gain for yaw drift correction (typical: 0.1-0.5)
     *          - kp: Proportional gain for roll/pitch drift correction (typical: 0.2-0.5)
     *          - _gps_gain: Weight factor for GPS velocity corrections (typical: 1.0-2.0)
     *          - _beta: Accelerometer trust factor (typical: 0.1), higher = more accel influence
     *          - gps_use: GPS usage configuration (disabled/enabled/with compass fallback)
     *          - gps_minsats: Minimum GPS satellites required for position updates (typical: 6)
     * 
     *          Higher gains provide faster convergence but increase susceptibility to sensor noise.
     *          Lower gains provide smoother estimates but slower correction of drift.
     * 
     * @param[in] kp_yaw       Proportional gain for yaw drift correction (dimensionless)
     * @param[in] kp           Proportional gain for roll/pitch drift correction (dimensionless)
     * @param[in] _gps_gain    GPS velocity correction weight factor (dimensionless)
     * @param[in] _beta        Accelerometer trust factor for drift correction (dimensionless)
     * @param[in] gps_use      GPS usage policy configuration
     * @param[in] gps_minsats  Minimum satellite count for GPS usage
     * 
     * @note Integral gains (ki, ki_yaw) are fixed constants defined internally
     * @note DCM matrix starts as identity; attitude converges during first seconds of operation
     */
    AP_AHRS_DCM(AP_Float &kp_yaw,
                AP_Float &kp,
                AP_Float &_gps_gain,
                AP_Float &_beta,
                AP_Enum<GPSUse> &gps_use,
                AP_Int8 &gps_minsats)
        : AP_AHRS_Backend(),
          _kp_yaw(kp_yaw),
          _kp(kp),
          gps_gain(_gps_gain),
          beta(_beta),
          _gps_minsats(gps_minsats),
          _gps_use(gps_use)
    {
        _dcm_matrix.identity();
    }

    /* Do not allow copies */
    CLASS_NO_COPY(AP_AHRS_DCM);

    /**
     * @brief Reset the accumulated gyro drift estimate
     * 
     * @details Clears the integral term (_omega_I) of the gyro drift correction.
     *          This should be called when gyro offsets are recalculated during
     *          calibration or when the gyro bias estimate becomes invalid.
     *          The drift estimate will rebuild from zero using accelerometer
     *          and magnetometer/GPS corrections.
     * 
     * @note Called automatically during gyro calibration procedures
     * @note Drift correction will reconverge within several seconds
     */
    void reset_gyro_drift() override;

    /**
     * @brief Update DCM attitude estimate for current timestep
     * 
     * @details Performs one complete DCM update cycle consisting of:
     *          1. matrix_update(): Integrate gyro rates into DCM matrix
     *          2. normalize(): Renormalize matrix to maintain orthogonality
     *          3. drift_correction(): Apply PI corrections from accel/compass/GPS
     * 
     *          This method should be called at the main loop rate (typically 400Hz for copter,
     *          50-400Hz for other vehicles). The update uses the latest IMU data and applies
     *          complementary filtering to correct gyro drift.
     * 
     * @note Must be called regularly; typical rates: 50-400Hz depending on vehicle type
     * @note Update timing affects attitude accuracy and drift correction responsiveness
     */
    void            update() override;

    /**
     * @brief Populate Estimates structure with current DCM state
     * 
     * @details Extracts attitude, position, velocity, and other state estimates from
     *          the DCM algorithm and populates the provided Estimates structure.
     *          Attitude is derived from the DCM matrix, position from dead-reckoning
     *          since last GPS fix, and velocity from GPS integration.
     * 
     * @param[out] results Estimates structure to populate with current state
     *                     - Attitude: Quaternion derived from DCM matrix (NED to body rotation)
     *                     - Position: Dead-reckoned from last GPS fix (NED frame, meters)
     *                     - Velocity: From GPS (NED frame, m/s)
     *                     - Gyro bias: Integrated drift estimate (rad/s)
     * 
     * @note Position accuracy degrades with time since last GPS update
     * @note All outputs use NED (North-East-Down) coordinate frame
     */
    void            get_results(Estimates &results) override;

    /**
     * @brief Reset DCM attitude to current sensor-derived attitude
     * 
     * @details Resets the DCM matrix without recovering from Euler angle backup.
     *          This is the external interface; internal reset may optionally
     *          recover from Euler angles if the DCM matrix becomes ill-conditioned.
     * 
     * @note Does not reset position or velocity estimates
     * @note Gyro drift estimate is preserved
     */
    void            reset() override { reset(false); }

    /**
     * @brief Check if yaw has been initialized from reference source
     * 
     * @details Returns whether the DCM algorithm has obtained an initial yaw reference
     *          from compass or GPS heading. Until yaw is initialized, the attitude estimate
     *          will have arbitrary heading. Initialization occurs when:
     *          - Compass provides valid heading, OR
     *          - GPS velocity provides course-over-ground heading (requires movement)
     * 
     * @return true if yaw has been initialized, false if heading is arbitrary
     * 
     * @note Vehicle should not be armed until yaw is initialized
     * @note GPS heading requires sufficient ground speed (typically >2 m/s)
     */
    bool yaw_initialised(void) const {
        return have_initial_yaw;
    }

    /**
     * @brief Get roll/pitch error estimate
     * 
     * @details Returns the current roll and pitch drift error estimate in radians.
     *          This value represents the magnitude of attitude corrections being applied
     *          by the complementary filter from accelerometer measurements. Higher values
     *          indicate larger disagreement between gyro-integrated attitude and
     *          accelerometer-derived attitude (gravity vector).
     * 
     * @return Roll/pitch error magnitude in radians
     * 
     * @note Typical values: <0.1 rad when stable, >0.3 rad indicates problems
     * @note High values may indicate IMU calibration issues or excessive vibration
     */
    float           get_error_rp() const {
        return _error_rp;
    }

    /**
     * @brief Get yaw error estimate
     * 
     * @details Returns the current yaw drift error estimate in radians.
     *          This value represents corrections being applied from compass or GPS heading.
     *          Higher values indicate disagreement between gyro-integrated yaw and
     *          compass/GPS-derived heading.
     * 
     * @return Yaw error magnitude in radians
     * 
     * @note Typical values: <0.1 rad when stable, >0.5 rad indicates problems
     * @note High values may indicate compass interference or GPS issues
     */
    float           get_error_yaw() const {
        return _error_yaw;
    }

    /**
     * @brief Get estimated wind velocity vector
     * 
     * @details Returns the current wind estimate in NED frame derived from GPS ground
     *          velocity and airspeed measurements. Wind is estimated using a complementary
     *          filter that fuses GPS velocity with indicated airspeed to extract wind component.
     * 
     * @param[out] wind Wind velocity vector in NED frame (m/s)
     * 
     * @return true (wind estimate always available, may be zero if insufficient data)
     * 
     * @note Wind estimation requires both GPS and airspeed sensor
     * @note All components in m/s: North, East, Down (NED frame)
     * @note Accuracy depends on heading accuracy and airspeed sensor quality
     */
    bool wind_estimate(Vector3f &wind) const override {
        wind = _wind;
        return true;
    }

    /**
     * @brief Set wind estimate from external source
     * 
     * @details Allows external wind estimation (e.g., from weather station or external
     *          algorithm) to override the internal DCM wind estimate. Wind direction
     *          follows aviation convention: direction wind is coming FROM.
     * 
     * @param[in] speed     Wind speed in m/s (magnitude, always positive)
     * @param[in] direction Wind direction in radians (direction FROM, 0=North, clockwise)
     * 
     * @note Direction convention: 0 = North, π/2 = East, π = South, 3π/2 = West
     * @note Internally converted to NED velocity vector for use in airspeed calculations
     */
    void set_external_wind_estimate(float speed, float direction);

    /**
     * @brief Get equivalent airspeed (EAS) estimate
     * 
     * @details Returns equivalent airspeed from the primary airspeed sensor if available,
     *          otherwise falls back to synthetic airspeed estimated from GPS velocity and
     *          wind estimate. EAS is corrected for altitude and represents dynamic pressure.
     * 
     * @param[out] airspeed_ret Equivalent airspeed in m/s
     * 
     * @return true if estimate available (sensor or synthetic), false if using stale estimate
     * 
     * @note EAS (Equivalent Airspeed) accounts for air density variation with altitude
     * @note Falls back to GPS-derived airspeed when sensor unavailable
     */
    bool airspeed_EAS(float &airspeed_ret) const override;

    /**
     * @brief Get equivalent airspeed from specific sensor
     * 
     * @details Returns equivalent airspeed from a specific airspeed sensor index,
     *          with fallback to synthetic estimate if that sensor is unavailable.
     *          Useful for vehicles with multiple airspeed sensors.
     * 
     * @param[in]  airspeed_index Index of airspeed sensor (0-based)
     * @param[out] airspeed_ret   Equivalent airspeed in m/s
     * 
     * @return true if estimate available from requested sensor or synthetic, false otherwise
     * 
     * @note If specified sensor unavailable, falls back to GPS-derived estimate
     * @note Units: m/s (meters per second)
     */
    bool airspeed_EAS(uint8_t airspeed_index, float &airspeed_ret) const override;

    /**
     * @brief Get synthetic equivalent airspeed estimate
     * 
     * @details Returns synthetic airspeed derived from GPS ground velocity and wind estimate
     *          rather than direct airspeed sensor measurement. Calculated by subtracting
     *          wind velocity from GPS ground velocity to obtain air-relative velocity.
     * 
     * @param[out] ret Synthetic equivalent airspeed in m/s
     * 
     * @return true (synthetic estimate always available, may be inaccurate without GPS)
     * 
     * @note Does not use physical airspeed sensor - purely GPS and wind based
     * @note Accuracy depends on GPS velocity quality and wind estimate accuracy
     * @note Useful as backup when airspeed sensor fails
     * @note Units: m/s (meters per second)
     */
    bool synthetic_airspeed_EAS(float &ret) const WARN_IF_UNUSED {
        ret = _last_airspeed_TAS;
        return true;
    }

    /**
     * @brief Get ground velocity vector in horizontal plane
     * 
     * @details Returns GPS-derived ground velocity vector in North-East order.
     *          Uses complementary filter (high-pass + low-pass) to blend GPS velocity
     *          with attitude-corrected airspeed for smoother output.
     * 
     * @return Ground velocity vector (North, East) in m/s
     * 
     * @note Order: [0]=North component, [1]=East component
     * @note Filtered for smooth control loop input
     * @note Requires GPS for accurate output
     */
    Vector2f groundspeed_vector() override;

    /**
     * @brief Determine if compass should be used for yaw correction
     * 
     * @details Evaluates compass and GPS health to decide whether compass should be used
     *          for yaw drift correction. Returns true if compass is healthy and either:
     *          - GPS is unavailable/unhealthy, OR
     *          - GPS heading is unavailable (insufficient speed), OR
     *          - System is configured to use compass
     * 
     * @return true if compass should be used for yaw, false to use GPS heading only
     * 
     * @note Decision based on GPS health, compass health, and configuration
     * @note GPS heading requires sufficient ground speed (typically >2 m/s)
     */
    bool            use_compass() override;

    /**
     * @brief Get attitude quaternion from DCM matrix
     * 
     * @details Converts the DCM rotation matrix to quaternion representation.
     *          The quaternion represents the rotation from NED (North-East-Down) earth frame
     *          to XYZ (Forward-Right-Down) body frame.
     * 
     * @param[out] quat Quaternion representing NED to body frame rotation
     * 
     * @return true if quaternion successfully derived from DCM matrix
     * 
     * @note Coordinate convention: NED earth frame → body frame (forward-right-down)
     * @note Quaternion normalized to unit magnitude
     */
    bool get_quaternion(Quaternion &quat) const override WARN_IF_UNUSED;

    /**
     * @brief Update wind velocity estimate
     * 
     * @details Estimates wind velocity by comparing GPS ground velocity with air-relative
     *          velocity from airspeed sensor. Uses complementary filtering to smooth the
     *          estimate and reject transients. Called periodically during DCM updates.
     * 
     * @note Requires both GPS velocity and airspeed sensor for accurate estimation
     * @note Wind estimate stored internally in _wind member (NED frame, m/s)
     * @note Algorithm filters rapid changes to avoid control loop coupling
     */
    void estimate_wind(void);

    /**
     * @brief Check if DCM AHRS subsystem is healthy
     * 
     * @details Evaluates DCM algorithm health based on:
     *          - DCM matrix validity (no NaN values, proper orthogonality)
     *          - Recent successful updates
     *          - Sensor availability (IMU at minimum)
     *          - Attitude error within acceptable bounds
     * 
     * @return true if DCM is operating normally, false if degraded or failed
     * 
     * @note False return indicates vehicle should not arm or should execute failsafe
     * @note Check error_rp and error_yaw for specific problem indicators
     * @warning Do not fly if healthy() returns false
     */
    bool healthy() const override;

    /**
     * @brief Get velocity vector in NED frame
     * 
     * @details Returns velocity estimate in North-East-Down frame from GPS integration.
     *          DCM does not maintain a filtered velocity estimate like EKF; this returns
     *          GPS velocity directly with minimal processing.
     * 
     * @param[out] vec Velocity vector in NED frame (m/s)
     * 
     * @return true if velocity available (GPS lock), false otherwise
     * 
     * @note All components in m/s: North, East, Down (NED frame convention)
     * @note Accuracy and update rate depend on GPS performance
     * @note Returns false if GPS lock lost
     */
    bool get_velocity_NED(Vector3f &vec) const override;

    /**
     * @brief Get vertical velocity kinematically consistent with position
     * 
     * @details Returns vertical velocity (Down component) that is kinematically consistent
     *          with the vertical position estimate. This differs from raw GPS vertical velocity
     *          which may have discontinuities. Used by control loops requiring smooth position
     *          and velocity derivatives.
     * 
     * @param[out] velocity Vertical velocity in m/s (Down positive, NED convention)
     * 
     * @return true if velocity estimate available, false otherwise
     * 
     * @note Down positive: positive values indicate descent
     * @note Derived to be consistent with integrated position for control stability
     * @note Units: m/s (meters per second)
     */
    bool get_vert_pos_rate_D(float &velocity) const override;

    /**
     * @brief Perform pre-arm checks for DCM AHRS
     * 
     * @details Validates that DCM is ready for arming by checking:
     *          - IMU sensors are healthy and calibrated
     *          - Yaw has been initialized from compass or GPS
     *          - Attitude errors are within acceptable bounds
     *          - GPS available if position control required
     *          - Compass healthy if no GPS or GPS insufficient for heading
     * 
     * @param[in]  requires_position If true, checks position source availability (not used by DCM)
     * @param[out] failure_msg       Buffer to store failure reason if check fails
     * @param[in]  failure_msg_len   Size of failure_msg buffer
     * 
     * @return true if all checks pass, false if vehicle should not arm
     * 
     * @note failure_msg populated with human-readable reason if returning false
     * @note Most critical check: yaw initialization (requires compass or GPS movement)
     * @warning Do not arm vehicle if this returns false
     */
    bool pre_arm_check(bool requires_position, char *failure_msg, uint8_t failure_msg_len) const override;

    /**
     * @brief Get origin location for relative positioning
     * 
     * @details Returns the location of the last known GPS fix, which serves as the origin
     *          for dead-reckoning position estimates. DCM maintains position relative to
     *          this origin using dead-reckoning when GPS is unavailable.
     * 
     * @param[out] ret Origin location (latitude, longitude, altitude)
     * 
     * @return true if origin is valid (previous GPS lock obtained), false otherwise
     * 
     * @note Origin set at last GPS lock; position estimates degrade with time since lock
     * @note Used by AP_InertialNav when falling back from EKF to DCM
     */
    bool get_origin(Location &ret) const override;

    /**
     * @brief Get position relative to origin in NED frame
     * 
     * @details Returns dead-reckoned position offset from last GPS lock origin.
     *          Position accumulated by integrating velocity since last GPS fix.
     *          Accuracy degrades over time without GPS corrections.
     * 
     * @param[out] vec Position offset vector in NED frame (meters)
     * 
     * @return true if position estimate available, false if no origin established
     * 
     * @note All components in meters: North, East, Down offsets from origin
     * @note Accuracy decreases with time since last GPS lock
     * @note Dead-reckoning only - not Kalman filtered like EKF
     * @warning Position error accumulates rapidly without GPS
     */
    bool get_relative_position_NED_origin(Vector3p &vec) const override;

    /**
     * @brief Get horizontal position relative to origin
     * 
     * @details Returns North-East position components relative to last GPS lock origin.
     *          Horizontal position only (excludes altitude/Down component).
     * 
     * @param[out] posNE North-East position offset (meters)
     * 
     * @return true if position estimate available, false if no origin
     * 
     * @note Units: meters (North component, East component)
     * @note Dead-reckoned from last GPS fix
     */
    bool get_relative_position_NE_origin(Vector2p &posNE) const override;

    /**
     * @brief Get vertical position relative to origin
     * 
     * @details Returns Down position component relative to origin altitude.
     *          Vertical position estimate from barometer integration.
     * 
     * @param[out] posD Down position offset in meters (positive = below origin)
     * 
     * @return true if position estimate available, false if no origin
     * 
     * @note Down positive: positive values indicate below origin altitude
     * @note Units: meters
     */
    bool get_relative_position_D_origin(postype_t &posD) const override;

    /**
     * @brief Send DCM status report to ground control station
     * 
     * @details Formats DCM state information as an EKF-like status report for transmission
     *          to ground control station via MAVLink. Provides compatibility with GCS tools
     *          expecting EKF status messages.
     * 
     * @param[in] link MAVLink communication link to send status on
     * 
     * @note Status includes: attitude errors, sensor health, position validity
     * @note Format compatible with EKF status messages for GCS compatibility
     */
    void send_ekf_status_report(class GCS_MAVLINK &link) const override;

    /**
     * @brief Check if yaw source is available
     * 
     * @details Determines if DCM has a valid source for yaw initialization and correction.
     *          Returns true if either compass or GPS heading is available and healthy.
     * 
     * @return true if compass or GPS heading available, false if no yaw source
     * 
     * @note GPS heading requires sufficient ground speed (typically >2 m/s)
     * @note Without yaw source, heading will drift uncorrected
     * @warning Vehicle should not arm without yaw source
     */
    bool yaw_source_available(void) const;

    /**
     * @brief Get control scaling limits for DCM
     * 
     * @details Provides control system limits for DCM-based navigation. DCM does not impose
     *          velocity limits like EKF, so this returns conservative defaults suitable for
     *          dead-reckoning navigation without Kalman filtering.
     * 
     * @param[out] ekfGndSpdLimit   Ground speed limit in m/s (set to conservative default)
     * @param[out] controlScaleXY   Horizontal control scale factor (typically 1.0)
     * 
     * @note DCM does not enforce velocity constraints like EKF
     * @note Limits provided for control loop compatibility
     */
    void get_control_limits(float &ekfGndSpdLimit, float &controlScaleXY) const override;

private:

    // dead-reckoning support
    bool get_location(Location &loc) const;

    // Settable tuning parameters (references to AP_AHRS parameter storage)
    AP_Float &_kp_yaw;       ///< Proportional gain for yaw drift correction (dimensionless, typical: 0.1-0.5)
    AP_Float &_kp;           ///< Proportional gain for roll/pitch drift correction (dimensionless, typical: 0.2-0.5)
    AP_Float &gps_gain;      ///< GPS velocity correction weight factor (dimensionless, typical: 1.0-2.0)

    AP_Float &beta;          ///< Accelerometer trust factor (dimensionless, typical: 0.1)

    AP_Int8 &_gps_minsats;   ///< Minimum GPS satellites required for position updates (typical: 6)

    AP_Enum<GPSUse> &_gps_use; ///< GPS usage policy configuration

    // Integral gain constants (fixed, experimentally derived from simulation with large drift)
    static constexpr float _ki = 0.0087f;      ///< Integral gain for roll/pitch drift correction (rad/s per rad error)
    static constexpr float _ki_yaw = 0.01f;    ///< Integral gain for yaw drift correction (rad/s per rad error)

    // Accelerometer values in earth frame in m/s/s
    Vector3f        _accel_ef;  ///< Earth-frame acceleration vector (NED frame, m/s²)

    /**
     * @brief Integrate gyro rates into DCM matrix
     * 
     * @details Updates the DCM rotation matrix by integrating gyroscope measurements
     *          over the timestep. Uses corrected gyro rates (_omega) which include
     *          drift compensation from the PI controller. Integration uses first-order
     *          approximation suitable for high update rates (>50Hz).
     * 
     * @note Called at main loop rate (typically 50-400Hz)
     * @note Higher update rates improve integration accuracy
     */
    void            matrix_update(void);

    /**
     * @brief Renormalize DCM matrix to maintain orthogonality
     * 
     * @details Corrects DCM matrix to ensure it remains a valid rotation matrix
     *          (orthonormal). Numerical integration errors cause the matrix to drift
     *          from orthogonality; this applies Gram-Schmidt orthogonalization to
     *          restore the constraint that rows are unit vectors and mutually perpendicular.
     * 
     * @note Called after every matrix_update()
     * @note Essential for long-term stability of DCM algorithm
     * @warning Without normalization, matrix becomes ill-conditioned within seconds
     */
    void            normalize(void);

    void            check_matrix(void);
    bool            renorm(Vector3f const &a, Vector3f &result);

    /**
     * @brief Apply drift correction from accelerometer and GPS
     * 
     * @details Implements PI controller that corrects gyro drift using external references.
     *          Compares gravity vector from accelerometer with DCM-predicted gravity direction
     *          to generate roll/pitch corrections. GPS velocity provides additional corrections
     *          for centrifugal effects during turns.
     * 
     * @param[in] deltat Time step since last correction in seconds
     * 
     * @note Proportional gain: _kp, Integral gain: _ki
     * @note Accelerometer corrections weighted by beta parameter
     * @note GPS corrections enabled when available and velocity sufficient
     */
    void            drift_correction(float deltat);

    /**
     * @brief Apply yaw drift correction from compass or GPS
     * 
     * @details Corrects yaw drift using compass heading or GPS course-over-ground.
     *          Compares measured heading with DCM-predicted heading to generate yaw
     *          correction. Uses PI controller with separate yaw-specific gains.
     * 
     * @note Proportional gain: _kp_yaw, Integral gain: _ki_yaw
     * @note GPS heading requires sufficient ground speed (>2 m/s typically)
     * @note Compass corrections affected by magnetic interference
     */
    void            drift_correction_yaw(void);
    float           yaw_error_compass(class Compass &compass);
    bool            have_gps(void) const;
    bool            use_fast_gains(void) const;
    void            backup_attitude(void);

    // internal reset function.  Called externally, we never reset the
    // DCM matrix from the eulers.  Called internally we may.
    void            reset(bool recover_eulers);

    /**
     * @brief Get unconstrained airspeed estimate with fallback priority
     * 
     * @details Attempts to fill airspeed in priority order:
     *          1. Enabled airspeed sensor for specified index
     *          2. Synthetic estimate from GPS speed and wind estimation
     *          3. Previous airspeed estimate (stale data)
     * 
     * @param[in]  airspeed_index Index of airspeed sensor to query
     * @param[out] airspeed_ret   Filled with airspeed estimate in m/s
     * 
     * @return false if using previous (stale) airspeed estimate, true otherwise
     * 
     * @note Always fills airspeed_ret, even if data is stale
     */
    bool get_unconstrained_airspeed_EAS(uint8_t airspeed_index, float &airspeed_ret) const;

    // Primary DCM matrix representing sensor board attitude (rotation from NED to sensor frame)
    Matrix3f _dcm_matrix;

    // Vehicle body DCM matrix (rotation from NED to body frame, accounts for board mounting)
    Matrix3f _body_dcm_matrix;

    // Euler angle backup - used for recovery if DCM matrix becomes ill-conditioned and for watchdog storage
    float roll;   ///< Roll angle in radians (right-hand rotation about forward axis)
    float pitch;  ///< Pitch angle in radians (right-hand rotation about right axis)
    float yaw;    ///< Yaw angle in radians (right-hand rotation about down axis)

    // PI controller state for drift correction (all in rad/s)
    Vector3f _omega_P;         ///< Proportional correction from accelerometer (rad/s)
    Vector3f _omega_yaw_P;     ///< Proportional yaw correction from compass/GPS (rad/s)
    Vector3f _omega_I;         ///< Integral correction term - accumulated gyro bias estimate (rad/s)
    Vector3f _omega_I_sum;     ///< Integrator accumulator
    float _omega_I_sum_time;   ///< Time accumulator for integral term
    Vector3f _omega;           ///< Corrected gyro rates after drift compensation (rad/s)

    bool have_initial_yaw;     ///< true if yaw initialized from compass or GPS heading

    /**
     * @brief Delay GPS acceleration correction to match GPS lag
     * 
     * @param[in] instance IMU instance index
     * @param[in] ra       Acceleration correction vector
     * 
     * @return Delayed acceleration correction matching GPS measurement timing
     * 
     * @note GPS measurements have inherent lag (~200ms); this buffers corrections
     */
    Vector3f ra_delayed(uint8_t instance, const Vector3f &ra);
    Vector3f _ra_delay_buffer[INS_MAX_INSTANCES];  ///< Buffer for GPS correction delay compensation

    /**
     * @brief Calculate P gain as function of vehicle spin rate
     * 
     * @param[in] spin_rate Vehicle rotation rate in rad/s
     * 
     * @return Adjusted proportional gain (higher during aggressive maneuvering)
     * 
     * @note Increases gain during fast rotation to maintain attitude accuracy
     */
    float           _P_gain(float spin_rate);

    /**
     * @brief Calculate yaw P gain based on horizontal velocity change rate
     * 
     * @return Adjusted yaw proportional gain
     * 
     * @note Higher gain when velocity changing rapidly (turning)
     */
    float           _yaw_gain(void) const;

    /**
     * @brief Determine if centrifugal corrections should be applied
     * 
     * @return true if GPS velocity corrections should be used
     * 
     * @note Disabled for copters while disarmed to prevent HUD bobbing
     * @note Prevents attitude drift from arm movements while vehicle on ground
     */
    bool should_correct_centrifugal() const;

    // Status reporting state
    float _renorm_val_sum;       ///< Accumulated renormalization error for averaging
    uint16_t _renorm_val_count;  ///< Sample count for renormalization averaging
    float _error_rp{1.0f};       ///< Roll/pitch error estimate in radians
    float _error_yaw{1.0f};      ///< Yaw error estimate in radians

    // Sensor timing state
    uint32_t _compass_last_update;  ///< Time in microseconds of last compass update
    uint32_t _gps_last_update;      ///< Time in milliseconds when we last got GPS heading

    // Accelerometer drift correction state
    Vector3f _ra_sum[INS_MAX_INSTANCES];  ///< Accumulated GPS-accel comparison per IMU
    Vector3f _last_velocity;               ///< Previous GPS velocity for delta calculation (m/s)
    float _ra_deltat;                      ///< Time accumulator for drift correction interval
    uint32_t _ra_sum_start;                ///< Start time of current accumulation period

    uint8_t _active_accel_instance;  ///< Currently active accelerometer instance index

    // Earth magnetic field representation
    float _last_declination;     ///< Magnetic declination in radians (last used value)
    Vector2f _mag_earth{1, 0};   ///< Earth magnetic field unit vector (North, East components)

    // GPS position state
    bool _have_gps_lock;         ///< true if GPS currently has valid 3D fix

    // Dead-reckoning origin (last known GPS position)
    int32_t _last_lat;           ///< Latitude of last GPS lock (degrees * 1e7)
    int32_t _last_lng;           ///< Longitude of last GPS lock (degrees * 1e7)
    uint32_t _last_pos_ms;       ///< Timestamp of last position update (milliseconds)

    // Dead-reckoned position offset from GPS origin
    float _position_offset_north;  ///< North offset from last GPS lock (meters)
    float _position_offset_east;   ///< East offset from last GPS lock (meters)

    bool _have_position;  ///< true if position estimate available (GPS lock obtained at some point)

    // Wind estimation state
    Vector3f _last_fuse;               ///< Last velocity fusion input (m/s)
    Vector3f _last_vel;                ///< Previous velocity for wind calculation (m/s)
    uint32_t _last_wind_time;          ///< Timestamp of last wind update (milliseconds)
    float _last_airspeed_TAS;          ///< Last true airspeed estimate (m/s)
    uint32_t _last_consistent_heading; ///< Time of last heading consistency check (milliseconds)

    Vector3f _wind;  ///< Estimated wind velocity in NED frame (m/s)

    // Health monitoring
    uint32_t _last_failure_ms;  ///< Timestamp of last AHRS failure (milliseconds)
    uint32_t _last_startup_ms;  ///< Timestamp when DCM was last reset (milliseconds)

    Location last_origin;  ///< Last origin returned (for DCM fallback from EKF)

    // Complementary filter states for groundspeed_vector()
    Vector2f _lp;            ///< Ground vector low-pass filter state (m/s)
    Vector2f _hp;            ///< Ground vector high-pass filter state (m/s)
    Vector2f _lastGndVelADS; ///< Previous airspeed-derived ground velocity input (m/s)

    // Trigonometric cache for performance optimization
    float _sin_yaw;  ///< Cached sin(yaw) for coordinate transformations
    float _cos_yaw;  ///< Cached cos(yaw) for coordinate transformations

    uint32_t last_log_ms;  ///< Timestamp of last logging output (milliseconds)
};

#endif  // AP_AHRS_DCM_ENABLED
