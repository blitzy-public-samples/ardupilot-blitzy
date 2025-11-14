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
 * @file AP_AHRS.h
 * @brief AHRS (Attitude Heading Reference System) frontend interface for ArduPilot
 * 
 * @details This file defines the AP_AHRS class, which serves as the central singleton
 *          managing attitude, heading, position, and velocity estimation for all ArduPilot
 *          vehicle types. The AHRS frontend automatically selects and manages multiple
 *          backend estimators based on availability and health.
 *          
 *          Backend Selection Priority: EKF3 > EKF2 > DCM > External > SITL
 *          
 *          The AHRS integrates data from multiple sensors:
 *          - IMU (gyros and accelerometers) via AP_InertialSensor
 *          - GPS position and velocity via AP_GPS
 *          - Magnetometer (compass) via AP_Compass
 *          - Barometer altitude via AP_Baro
 *          - Airspeed sensor via AP_Airspeed (for fixed-wing)
 *          - Optical flow, rangefinders, and external navigation sources
 *          
 *          This provides unified vehicle state estimates including:
 *          - Attitude (roll, pitch, yaw) in multiple representations (Euler, quaternion, DCM)
 *          - Position (latitude, longitude, altitude) in NED frame
 *          - Velocity (North, East, Down) in m/s
 *          - Wind estimation for fixed-wing vehicles
 *          - Airspeed estimation (true and equivalent)
 * 
 * @note Access singleton via AP::ahrs() or AP_AHRS::get_singleton()
 * @note Thread-safe access via get_semaphore() for multi-threaded operations
 * @warning Must call update() in main vehicle loop (typically 50-400Hz depending on vehicle)
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#include "AP_AHRS_config.h"

#include <AP_HAL/Semaphores.h>

#include "AP_AHRS_Backend.h"
#include <AP_NavEKF2/AP_NavEKF2.h>
#include <AP_NavEKF3/AP_NavEKF3.h>
#include <AP_NavEKF/AP_Nav_Common.h>              // definitions shared by inertial and ekf nav filters

#include "AP_AHRS_DCM.h"
#include "AP_AHRS_SIM.h"
#include "AP_AHRS_External.h"
#include <AP_NavEKF/AP_Nav_Common.h>

// forward declare view class
class AP_AHRS_View;

#define AP_AHRS_NAVEKF_SETTLE_TIME_MS 20000     // time in milliseconds the ekf needs to settle after being started


// fwd declare GSF estimator
class EKFGSF_yaw;

/**
 * @class AP_AHRS
 * @brief Central AHRS frontend singleton managing attitude, heading, and position estimation
 * 
 * @details AP_AHRS is the primary interface for accessing vehicle state estimates in ArduPilot.
 *          It manages multiple estimation backends and automatically selects the best available
 *          estimator based on sensor availability, health, and configuration.
 *          
 *          **Backend Architecture:**
 *          The AHRS frontend can utilize multiple backend estimators:
 *          - **EKF3** (AP_NavEKF3): Primary Extended Kalman Filter (most accurate, modern)
 *          - **EKF2** (AP_NavEKF2): Legacy Extended Kalman Filter (fallback)
 *          - **DCM** (AP_AHRS_DCM): Direction Cosine Matrix estimator (fallback for attitude-only)
 *          - **External** (AP_AHRS_External): External navigation system (e.g., motion capture)
 *          - **SITL** (AP_AHRS_SIM): Software-in-the-loop simulation backend
 *          
 *          **Backend Selection Priority:** EKF3 > EKF2 > DCM > External > SITL
 *          The frontend automatically switches to the highest-priority healthy backend.
 *          
 *          **Coordinate Frames:**
 *          - **NED frame**: North-East-Down earth-fixed frame (origin at EKF origin or home location)
 *          - **Body frame**: Forward-Right-Down vehicle-fixed frame
 *          - **Earth frame**: NED-aligned earth-fixed frame (for accelerations)
 *          
 *          **Usage Pattern:**
 *          @code{.cpp}
 *          // Access singleton
 *          AP_AHRS &ahrs = AP::ahrs();
 *          
 *          // Thread-safe access
 *          {
 *              WITH_SEMAPHORE(ahrs.get_semaphore());
 *              Location current_loc;
 *              if (ahrs.get_location(current_loc)) {
 *                  // Use location
 *              }
 *          }
 *          
 *          // Get attitude
 *          float roll_rad = ahrs.get_roll_rad();
 *          float pitch_rad = ahrs.get_pitch_rad();
 *          float yaw_rad = ahrs.get_yaw_rad();
 *          @endcode
 * 
 * @note This is a singleton class - access via AP::ahrs() or get_singleton()
 * @note update() must be called at main loop rate (typically 50-400Hz depending on vehicle)
 * @note Use get_semaphore() for thread-safe access in multi-threaded contexts
 * @warning Backend selection affects accuracy - EKF provides position, DCM only provides attitude
 * @warning Some functions only valid when have_inertial_nav() returns true
 * 
 * @see AP_AHRS_Backend Base class for all backend estimators
 * @see AP_AHRS_DCM DCM attitude estimator backend
 * @see AP_NavEKF2 EKF2 navigation filter
 * @see AP_NavEKF3 EKF3 navigation filter (recommended)
 * @see AP_AHRS_View Rotated view of AHRS for multi-vehicle or sensor applications
 */
class AP_AHRS {
    friend class AP_AHRS_View;
public:

    // copy this into our namespace
    using Status = NavFilterStatusBit;

    /**
     * @enum Flags
     * @brief Configuration flags for AP_AHRS initialization
     */
    enum Flags {
        FLAG_ALWAYS_USE_EKF = 0x1,  ///< Force EKF usage even when not normally required
    };

    /**
     * @brief Check EKF status flags for health and capabilities
     * 
     * @details Returns information about the Extended Kalman Filter's operational
     *          status, including sensor health, initialization state, and available
     *          estimates. This provides detailed diagnostic information about the
     *          active estimation backend.
     * 
     * @param[in] status NavFilterStatusBit flag to check (e.g., ATTITUDE, HORIZ_VEL, HORIZ_POS_ABS)
     * @return true if the specified status bit is set (capability available and healthy)
     * @return false if status bit not set or backend doesn't support get_filter_status
     * 
     * @note Returns false for DCM backend as it doesn't provide filter status
     * @note Only valid when using EKF backends (EKF2 or EKF3)
     * 
     * @see NavFilterStatusBit for available status flags
     * @see get_filter_status() for complete status structure
     */
    bool has_status(Status status) const;

    /**
     * @brief Construct AP_AHRS singleton with configuration flags
     * 
     * @param[in] flags Initialization flags from Flags enum (default: 0)
     *                  Use FLAG_ALWAYS_USE_EKF to force EKF usage
     * 
     * @note This is a singleton - only one instance should exist
     * @note Constructor is typically called during vehicle initialization
     */
    AP_AHRS(uint8_t flags = 0);

    /**
     * @brief Initialize AHRS subsystem and backends
     * 
     * @details Performs full AHRS initialization including:
     *          - Loading parameters from storage
     *          - Initializing all available backends (DCM, EKF2, EKF3, External, SITL)
     *          - Setting up coordinate frame transformations
     *          - Configuring trim and board orientation
     *          - Establishing EKF origin if GPS available
     * 
     * @note Must be called once during vehicle startup after sensors initialized
     * @note Typically called from vehicle's init() method
     * 
     * @see update() for ongoing updates after initialization
     */
    void init(void);

    /* Do not allow copies */
    CLASS_NO_COPY(AP_AHRS);

    /**
     * @brief Get pointer to AHRS singleton instance
     * 
     * @return Pointer to the global AP_AHRS singleton instance
     * 
     * @note Prefer using AP::ahrs() accessor for cleaner syntax
     * @note Returns nullptr if AHRS not yet constructed
     * 
     * @see AP::ahrs() for alternative accessor
     */
    static AP_AHRS *get_singleton() {
        return _singleton;
    }

    /**
     * @brief Update board orientation from AHRS_ORIENTATION parameter
     * 
     * @details Periodically checks AHRS_ORIENTATION parameter and updates the
     *          coordinate frame transformation if it has changed. This allows
     *          runtime reconfiguration of board mounting orientation without
     *          requiring a reboot, simplifying initial setup and testing.
     * 
     * @note Called automatically from update() at reduced rate (not every loop)
     * @note Orientation changes trigger recalculation of trim rotation matrices
     * 
     * @see AHRS_ORIENTATION parameter for supported orientations
     */
    void update_orientation();

    /**
     * @brief Get semaphore for thread-safe AHRS access
     * 
     * @details Returns reference to semaphore that protects AHRS state during updates.
     *          Use WITH_SEMAPHORE() macro when accessing AHRS data from threads other
     *          than the main vehicle loop to prevent race conditions and ensure
     *          consistent state snapshots.
     * 
     * @return Reference to AHRS protection semaphore
     * 
     * @note Main vehicle loop holds semaphore during update()
     * @note Background threads must lock semaphore before accessing AHRS data
     * 
     * Example:
     * @code{.cpp}
     * // Thread-safe access pattern
     * {
     *     WITH_SEMAPHORE(ahrs.get_semaphore());
     *     Vector3f velocity;
     *     if (ahrs.get_velocity_NED(velocity)) {
     *         // Use velocity safely
     *     }
     * }
     * @endcode
     */
    HAL_Semaphore &get_semaphore(void) {
        return _rsem;
    }

    /**
     * @brief Get smoothed gyro vector corrected for drift
     * 
     * @details Returns the estimated angular velocity vector in body frame,
     *          corrected for gyro bias/drift. This is a filtered estimate
     *          combining raw gyro measurements with bias correction from
     *          the active backend estimator.
     * 
     * @return Gyro vector in rad/s, body frame (Forward-Right-Down)
     *         X: roll rate (rad/s)
     *         Y: pitch rate (rad/s)
     *         Z: yaw rate (rad/s)
     * 
     * @note Gyro drift correction improves over time as backend estimator converges
     * @note This is the recommended gyro estimate for control loops
     * 
     * @see get_gyro_latest() for uncached gyro with latest INS data
     * @see get_gyro_drift() for current bias estimate
     */
    const Vector3f &get_gyro(void) const { return state.gyro_estimate; }

    /**
     * @brief Get current gyro drift/bias estimate
     * 
     * @details Returns the estimated gyro bias in rad/s. This is the systematic
     *          offset being subtracted from raw gyro measurements to correct for
     *          sensor drift, temperature effects, and calibration errors.
     * 
     * @return Gyro drift vector in rad/s, body frame
     * 
     * @note Drift estimate improves over time as EKF converges
     * @note DCM backend also estimates and corrects gyro drift
     * 
     * @see reset_gyro_drift() to reset after gyro calibration
     */
    const Vector3f &get_gyro_drift(void) const { return state.gyro_drift; }

    /**
     * @brief Reset gyro drift estimate to zero
     * 
     * @details Clears the accumulated gyro bias estimate. Should be called
     *          after performing gyro calibration to prevent old bias estimates
     *          from affecting newly calibrated gyro offsets.
     * 
     * @note Call this after gyro calibration procedures
     * @note Backend estimators will re-converge on new bias estimate
     */
    void reset_gyro_drift();

    /**
     * @brief Main AHRS update - call at vehicle loop rate
     * 
     * @details Performs complete AHRS update cycle:
     *          - Updates INS (Inertial Navigation System) unless skip_ins_update=true
     *          - Updates all active backends (DCM, EKF2, EKF3, External, SITL)
     *          - Selects best backend based on health and priority
     *          - Copies estimates to canonical state structure
     *          - Updates derived values (trig functions, centidegree values)
     *          - Performs backend switching if necessary
     * 
     * @param[in] skip_ins_update If true, skip INS update (for use when INS updated separately)
     * 
     * @note Must be called at main loop rate (typically 50-400Hz depending on vehicle)
     * @note Copter: 400Hz, Plane: 50-100Hz, Rover: 50Hz
     * @warning Failure to call update() results in stale state estimates
     * 
     * @see init() for initialization before calling update()
     */
    void            update(bool skip_ins_update=false);
    
    /**
     * @brief Reset AHRS attitude estimate to current sensor values
     * 
     * @details Forces re-initialization of attitude estimate based on current
     *          accelerometer and magnetometer readings. Used for recovery from
     *          severe attitude errors or after significant external disturbances.
     * 
     * @warning Should only be called when vehicle is stationary and level
     * @note Primarily affects DCM backend; EKF backends have their own reset mechanisms
     */
    void            reset();

    /**
     * @brief Get current vehicle location estimate
     * 
     * @details Returns the estimated vehicle position as a Location structure
     *          containing latitude, longitude, and altitude. This is the primary
     *          method for obtaining vehicle position in WGS-84 coordinates.
     * 
     * @param[out] loc Location structure to populate with position estimate
     *                 lat: latitude in degrees * 1e7
     *                 lng: longitude in degrees * 1e7
     *                 alt: altitude in cm (AMSL - Above Mean Sea Level)
     * 
     * @return true if location estimate is valid and available
     * @return false if no valid position estimate (e.g., no GPS, EKF not initialized)
     * 
     * @note Requires EKF backend with GPS or external position source
     * @note DCM backend cannot provide position - will return false
     * @note Check return value before using location data
     * 
     * @see have_inertial_nav() to check if position estimation available
     * @see get_origin() for EKF origin location
     */
    bool get_location(Location &loc) const;

    /**
     * @brief Get height above ground level estimate
     * 
     * @details Returns the estimated height above the local ground surface in meters.
     *          This uses rangefinder data when available, otherwise falls back to
     *          terrain database or barometric altitude relative to origin.
     * 
     * @param[out] hagl Height above ground level in meters (positive up)
     * 
     * @return true if HAGL estimate is valid and available
     * @return false if no valid height estimate
     * 
     * @note Accuracy depends on terrain data or rangefinder availability
     * @note Useful for terrain following and automatic landing
     * 
     * @see AP_Terrain for terrain database functionality
     */
    bool get_hagl(float &hagl) const WARN_IF_UNUSED;

    /**
     * @brief Get estimated roll/pitch error magnitude
     * 
     * @details Returns the estimated uncertainty in roll and pitch attitude
     *          estimates in radians. Larger values indicate less confidence
     *          in the attitude solution.
     * 
     * @return Roll/pitch error estimate in radians
     * 
     * @note Only available from EKF backends
     * @note Returns 0.0 for DCM backend
     * @note Useful for determining when to trust attitude estimates
     * 
     * @see get_error_yaw() for yaw error estimate
     * @see healthy() for overall AHRS health status
     */
    float           get_error_rp() const;
    
    /**
     * @brief Get estimated yaw error magnitude
     * 
     * @details Returns the estimated uncertainty in yaw (heading) estimate in
     *          radians. Larger values indicate less confidence in the heading
     *          solution. Yaw error is typically larger than roll/pitch error,
     *          especially before GPS or compass lock.
     * 
     * @return Yaw error estimate in radians
     * 
     * @note Only available from EKF backends
     * @note Returns 0.0 for DCM backend
     * @note Yaw uncertainty is higher without GPS velocity or magnetometer
     * 
     * @see get_error_rp() for roll/pitch error estimate
     * @see dcm_yaw_initialised() to check if DCM yaw initialized
     */
    float           get_error_yaw() const;

    /*
     * wind estimation support
     */

    /**
     * @brief Enable or disable wind estimation
     * 
     * @details Controls whether the AHRS backends perform wind estimation.
     *          Wind estimation uses airspeed and GPS ground speed to compute
     *          wind velocity, which improves navigation accuracy for fixed-wing
     *          aircraft and aids in energy management.
     * 
     * @param[in] b true to enable wind estimation, false to disable
     * 
     * @note Primarily used for fixed-wing vehicles with airspeed sensors
     * @note Wind estimation requires both airspeed sensor and GPS
     * 
     * @see wind_estimate() to retrieve wind estimate
     * @see get_wind_estimation_enabled() to check current state
     */
    void set_wind_estimation_enabled(bool b) { wind_estimation_enabled = b; }

    /**
     * @brief Check if wind estimation is enabled
     * 
     * @return true if wind estimation is enabled
     * @return false if wind estimation is disabled
     * 
     * @see set_wind_estimation_enabled() to enable/disable
     */
    bool get_wind_estimation_enabled() const { return wind_estimation_enabled; }

    /**
     * @brief Get wind velocity estimate
     * 
     * @details Returns the estimated wind velocity vector in NED frame.
     *          Wind estimation combines airspeed and GPS ground speed
     *          measurements to compute ambient wind.
     * 
     * @return Wind velocity vector in m/s, NED frame
     *         North: wind from south (positive = northward wind)
     *         East: wind from west (positive = eastward wind)
     *         Down: typically zero for horizontal wind
     * 
     * @note Returns (0,0,0) if wind estimation not available or disabled
     * @note Requires airspeed sensor and GPS for accuracy
     * @note EKF and DCM backends both provide wind estimates
     * 
     * @see set_wind_estimation_enabled() to enable wind estimation
     * @see head_wind() for headwind component
     */
    const Vector3f &wind_estimate() const { return state.wind_estimate; }

    /**
     * @brief Get wind velocity estimate with validity check
     * 
     * @param[out] wind Wind velocity vector in m/s, NED frame
     * 
     * @return true if wind estimate is valid and available
     * @return false if no valid wind estimate
     * 
     * @note Prefer this version over wind_estimate() const when validity matters
     * 
     * @see wind_estimate() const for direct vector access
     */
    bool wind_estimate(Vector3f &wind) const;

    /**
     * @brief Calculate heading alignment with wind direction
     * 
     * @details Computes how aligned a given heading is with the wind direction.
     *          Useful for optimizing flight paths, soaring, and energy management.
     * 
     * @param[in] heading_deg Heading to test in degrees
     * 
     * @return +1.0 = perfectly aligned heading into wind (headwind)
     * @return -1.0 = perfectly aligned with wind (tailwind)
     * @return  0.0 = perfect cross-wind (perpendicular to wind)
     * 
     * @note No distinction between left and right cross-wind
     * @note Wind speed magnitude is ignored - only direction matters
     * 
     * @see wind_estimate() for wind vector
     * @see head_wind() for headwind component
     */
    float wind_alignment(const float heading_deg) const;

    /**
     * @brief Get headwind component along current heading
     * 
     * @details Returns the component of wind velocity along the vehicle's
     *          current heading direction. Positive values indicate headwind
     *          (wind opposing forward motion), negative indicates tailwind.
     * 
     * @return Headwind component in m/s
     *         Positive: headwind (reduces ground speed)
     *         Negative: tailwind (increases ground speed)
     * 
     * @note Uses current vehicle heading from get_yaw()
     * @note Critical for fixed-wing energy management and landing
     * 
     * @see wind_estimate() for complete wind vector
     * @see wind_alignment() for alignment calculation
     */
    float head_wind(void) const;

    /**
     * @brief Trigger DCM wind estimate update
     * 
     * @details Instructs the DCM backend to update its wind velocity estimate
     *          based on current airspeed and ground speed. This is called
     *          periodically when using DCM as the active backend.
     * 
     * @note Only affects DCM backend - no-op if DCM not compiled in
     * @note EKF backends update wind estimate automatically
     */
    void estimate_wind() {
#if AP_AHRS_DCM_ENABLED
        dcm.estimate_wind();
#endif
    }

#if AP_AHRS_EXTERNAL_WIND_ESTIMATE_ENABLED
    /**
     * @brief Set external wind estimate
     * 
     * @details Allows injection of wind estimate from external source (e.g.,
     *          weather station, ground control station, atmospheric model).
     * 
     * @param[in] speed Wind speed in m/s
     * @param[in] direction Wind direction in degrees
     * 
     * @note Only available if AP_AHRS_EXTERNAL_WIND_ESTIMATE_ENABLED
     * @note Passed to DCM backend for integration
     */
    void set_external_wind_estimate(float speed, float direction) {
        dcm.set_external_wind_estimate(speed, direction);
    }
#endif

    /**
     * @brief Get maximum expected wind parameter
     * 
     * @details Returns the AHRS_WIND_MAX parameter value, which represents
     *          the maximum expected wind speed. Used by some estimation
     *          algorithms to bound wind estimates and detect anomalies.
     * 
     * @return Maximum wind speed in m/s
     * 
     * @note Configured via AHRS_WIND_MAX parameter
     * @note Typical values: 10-30 m/s depending on operating environment
     */
    uint8_t get_max_wind() const {
        return _wind_max;
    }

    /*
     * airspeed support
     */

    /**
     * @brief Get equivalent-to-true airspeed ratio
     * 
     * @details Returns the ratio to convert Equivalent Airspeed (EAS) to
     *          True Airspeed (TAS). This ratio accounts for air density
     *          changes with altitude: TAS = EAS * ratio.
     * 
     * @return EAS to TAS conversion ratio (dimensionless, >= 1.0)
     * 
     * @note Ratio increases with altitude as air density decreases
     * @note At sea level: ratio ≈ 1.0, at 10,000ft: ratio ≈ 1.15
     * 
     * @see get_air_density_ratio() for density ratio
     * @see airspeed_estimate_true() for direct TAS estimate
     */
    float get_EAS2TAS(void) const;

    /**
     * @brief Get air density ratio relative to sea level
     * 
     * @details Returns the ratio of current air density to sea level standard
     *          density (1.225 kg/m³). Decreases with increasing altitude.
     * 
     * @return Air density ratio (dimensionless, 0.0 to 1.0)
     *         1.0 = sea level density
     *         0.7 = ~10,000 ft altitude
     * 
     * @note Used for aerodynamic calculations and airspeed conversions
     * @note Based on barometric altitude and temperature
     * 
     * @see get_EAS2TAS() for airspeed conversion ratio
     */
    float get_air_density_ratio(void) const;
    
    /**
     * @brief Get equivalent airspeed estimate
     * 
     * @details Returns an estimate of Equivalent Airspeed (EAS), which is
     *          calibrated airspeed corrected for compressibility. This is
     *          the primary airspeed estimate used for flight control.
     * 
     * @param[out] airspeed_ret Equivalent airspeed in m/s
     * 
     * @return true if airspeed estimate is valid and available
     * @return false if no airspeed estimate available
     * 
     * @note May come from physical airspeed sensor or synthetic estimate
     * @note For fixed-wing flight control and energy management
     * 
     * @see airspeed_estimate(float&, AirspeedEstimateType&) for source type
     * @see airspeed_estimate_true() for true airspeed
     * @see using_airspeed_sensor() to check if using physical sensor
     */
    bool airspeed_estimate(float &airspeed_ret) const;

    /**
     * @enum AirspeedEstimateType
     * @brief Source type for airspeed estimates
     */
    enum AirspeedEstimateType : uint8_t {
        NO_NEW_ESTIMATE = 0,    ///< No new estimate available
        AIRSPEED_SENSOR = 1,    ///< From physical airspeed sensor (pitot tube)
        DCM_SYNTHETIC = 2,      ///< Synthetic estimate from DCM backend
        EKF3_SYNTHETIC = 3,     ///< Synthetic estimate from EKF3 backend
        SIM = 4,                ///< From SITL simulation
    };

    /**
     * @brief Get equivalent airspeed estimate with source type
     * 
     * @details Returns airspeed estimate along with information about the
     *          source of the estimate (sensor vs synthetic). Useful for
     *          determining confidence in the estimate and for diagnostics.
     * 
     * @param[out] airspeed_ret Equivalent airspeed in m/s
     * @param[out] type Source type of the estimate (AirspeedEstimateType enum)
     * 
     * @return true if airspeed estimate is valid and available
     * @return false if no airspeed estimate available
     * 
     * @note Prefer sensor-based estimates over synthetic when available
     * @note Synthetic estimates less accurate, especially in wind
     * 
     * @see AirspeedEstimateType for source types
     * @see airspeed_estimate(float&) for simpler version without type
     */
    bool airspeed_estimate(float &airspeed_ret, AirspeedEstimateType &type) const;

    /**
     * @brief Check if using physical airspeed sensor
     * 
     * @details Returns true if the current airspeed estimate is derived
     *          directly from a physical airspeed sensor, as opposed to
     *          a synthetic estimate from GPS and attitude.
     * 
     * @return true if using physical airspeed sensor
     * @return false if using synthetic estimate or no estimate available
     * 
     * @note Critical for determining when to trust airspeed for flight control
     * @note Some flight modes require physical airspeed sensor
     * 
     * @see airspeed_sensor_enabled() to check if sensor configured
     * @see airspeed_estimate(float&, AirspeedEstimateType&) for detailed source
     */
    bool using_airspeed_sensor() const;

    /**
     * @brief Get true airspeed estimate
     * 
     * @details Returns an estimate of True Airspeed (TAS), which is the
     *          actual speed of the vehicle through the air mass. TAS is
     *          higher than EAS at altitude due to lower air density.
     * 
     * @param[out] airspeed_ret True airspeed in m/s
     * 
     * @return true if TAS estimate is valid and available
     * @return false if no TAS estimate available
     * 
     * @note TAS = EAS * get_EAS2TAS()
     * @note Used for wind estimation and navigation calculations
     * 
     * @see airspeed_estimate() for equivalent airspeed
     * @see get_EAS2TAS() for conversion ratio
     * @see airspeed_vector_true() for vector form in body frame
     */
    bool airspeed_estimate_true(float &airspeed_ret) const;

    /**
     * @brief Get true airspeed vector in body frame
     * 
     * @details Returns the true airspeed as a 3D vector in body frame,
     *          providing airflow velocity components along vehicle axes.
     *          Useful for advanced aerodynamic calculations.
     * 
     * @param[out] vec Airspeed vector in m/s, body frame (Forward-Right-Down)
     *                 X: forward airspeed component
     *                 Y: right (sideslip) component
     *                 Z: down component
     * 
     * @return true if airspeed vector estimate is valid
     * @return false if vector estimate unavailable
     * 
     * @note Primarily from EKF3 backend
     * @note Magnitude of vec is true airspeed
     * 
     * @see airspeed_estimate_true() for scalar TAS
     */
    bool airspeed_vector_true(Vector3f &vec) const;

    /**
     * @brief Get airspeed sensor health diagnostics
     * 
     * @details Returns detailed health information for the airspeed estimate,
     *          including innovation (difference between measured and predicted),
     *          innovation variance, and age of last measurement.
     * 
     * @param[out] innovation Measurement innovation in m/s (measured - predicted)
     * @param[out] innovationVariance Innovation variance in (m/s)²
     * @param[out] age_ms Age of last measurement in milliseconds
     * 
     * @return true if health data is available
     * @return false if data unavailable (no airspeed sensor or no EKF)
     * 
     * @note Only available with EKF backends
     * @note Large innovation indicates sensor fault or wind gust
     * @note Large variance indicates low confidence in estimate
     * 
     * @see airspeed_sensor_enabled() to check sensor availability
     */
    bool airspeed_health_data(float &innovation, float &innovationVariance, uint32_t &age_ms) const;

    /**
     * @brief Check if any airspeed sensor is enabled
     * 
     * @details Returns true if at least one airspeed sensor is configured
     *          and enabled in the system, regardless of health status.
     * 
     * @return true if airspeed sensor enabled
     * @return false if no airspeed sensor configured
     * 
     * @note Does not indicate sensor health, only configuration
     * 
     * @see airspeed_sensor_enabled(uint8_t) for specific sensor
     * @see using_airspeed_sensor() to check if sensor actually in use
     */
    bool airspeed_sensor_enabled(void) const {
        // FIXME: make this a method on the active backend
        return AP_AHRS_Backend::airspeed_sensor_enabled();
    }

    /**
     * @brief Check if specific airspeed sensor is enabled
     * 
     * @param[in] airspeed_index Index of airspeed sensor to check (0-based)
     * 
     * @return true if specified airspeed sensor is enabled
     * @return false if sensor not enabled or index invalid
     * 
     * @note Supports multiple airspeed sensors (primary, secondary, etc.)
     * 
     * @see airspeed_sensor_enabled() to check for any sensor
     * @see get_active_airspeed_index() for currently active sensor
     */
    bool airspeed_sensor_enabled(uint8_t airspeed_index) const {
        // FIXME: make this a method on the active backend
        return AP_AHRS_Backend::airspeed_sensor_enabled(airspeed_index);
    }

    /**
     * @brief Get synthetic airspeed estimate (without physical sensor)
     * 
     * @details Returns a synthetic airspeed estimate derived from GPS ground
     *          speed and wind estimate, without using a physical airspeed sensor.
     *          Less accurate than sensor-based airspeed, especially in wind.
     * 
     * @param[out] ret Synthetic equivalent airspeed in m/s
     * 
     * @return true if synthetic estimate is available
     * @return false if synthetic estimate unavailable
     * 
     * @note Requires GPS and valid wind estimate
     * @note Accuracy degraded in gusty or turbulent conditions
     * @note Used as fallback when airspeed sensor fails
     * 
     * @see airspeed_estimate() for primary airspeed (sensor or synthetic)
     * @see using_airspeed_sensor() to check source
     */
    bool synthetic_airspeed(float &ret) const WARN_IF_UNUSED;

    /**
     * @brief Check if compass (magnetometer) is being used for heading
     * 
     * @details Returns true if the active backend is using compass measurements
     *          for heading estimation. The AHRS may choose not to use compass
     *          if GPS velocity provides better heading, or if compass health
     *          is poor, or if non-compass yaw source is available.
     * 
     * @return true if compass is being used for yaw estimation
     * @return false if compass not used (GPS-based heading, external nav, etc.)
     * 
     * @note EKF may switch between compass and GPS heading dynamically
     * @note Useful for compass calibration and pre-arm checks
     * 
     * @see using_noncompass_for_yaw() to check for alternative yaw sources
     * @see dcm_yaw_initialised() for DCM yaw initialization status
     */
    bool use_compass();

    /**
     * @brief Get current attitude as quaternion
     * 
     * @details Returns the vehicle attitude as a quaternion representing the
     *          rotation from NED (North-East-Down) earth frame to XYZ body
     *          frame (Forward-Right-Down).
     * 
     * @param[out] quat Attitude quaternion (rotation from NED to body frame)
     * 
     * @return true if quaternion is valid and available
     * @return false if attitude estimate unavailable
     * 
     * @note Quaternion representation avoids gimbal lock issues
     * @note For Euler angles, use get_roll_rad(), get_pitch_rad(), get_yaw_rad()
     * @note Quaternion normalized: quat.length() == 1.0
     * 
     * @see get_rotation_body_to_ned() for DCM matrix representation
     * @see get_quat_body_to_ned() for alternative accessor
     */
    bool get_quaternion(Quaternion &quat) const WARN_IF_UNUSED;

    /**
     * @brief Get secondary attitude solution as Euler angles
     * 
     * @details Returns attitude estimate from a secondary backend (if available).
     *          Useful for redundancy checking and detecting attitude inconsistencies
     *          between multiple estimators.
     * 
     * @param[out] eulers Attitude as Euler angles in radians (roll, pitch, yaw)
     * 
     * @return true if secondary attitude solution is available
     * @return false if no secondary solution (only one backend active)
     * 
     * @note Typically EKF2 when EKF3 is primary, or vice versa
     * @note Used for pre-arm consistency checks
     * @note Large differences between primary and secondary indicate problems
     * 
     * @see get_secondary_quaternion() for quaternion form
     * @see attitudes_consistent() for consistency checking
     */
    bool get_secondary_attitude(Vector3f &eulers) const {
        eulers = state.secondary_attitude;
        return state.secondary_attitude_ok;
    }

    /**
     * @brief Get secondary attitude solution as quaternion
     * 
     * @details Returns quaternion attitude estimate from a secondary backend.
     *          Quaternion form is preferred over Euler angles for mathematical
     *          operations and to avoid gimbal lock.
     * 
     * @param[out] quat Attitude quaternion from secondary backend
     * 
     * @return true if secondary quaternion solution is available
     * @return false if no secondary solution available
     * 
     * @see get_secondary_attitude() for Euler angle form
     * @see get_quaternion() for primary attitude quaternion
     */
    bool get_secondary_quaternion(Quaternion &quat) const {
        quat = state.secondary_quat;
        return state.secondary_quat_ok;
    }

    /**
     * @brief Get secondary position solution
     * 
     * @details Returns position estimate from a secondary backend (if available).
     *          Used for redundancy and consistency checking between multiple
     *          position estimators.
     * 
     * @param[out] loc Location from secondary backend (lat/lon/alt)
     * 
     * @return true if secondary position solution is available
     * @return false if no secondary position available
     * 
     * @note Typically EKF2 when EKF3 is primary, or vice versa
     * @note Used for detecting position estimate divergence
     * @note Large differences indicate estimator problems
     * 
     * @see get_location() for primary position estimate
     * @see pre_arm_check() uses this for consistency validation
     */
    bool get_secondary_position(Location &loc) const {
        loc = state.secondary_pos;
        return state.secondary_pos_ok;
    }

    /**
     * @brief Get 2D ground speed vector
     * 
     * @details Returns horizontal ground velocity as a 2D vector in NED frame.
     *          This is the vehicle's velocity over the ground (North and East
     *          components only), useful for navigation and path following.
     * 
     * @return Ground speed vector in m/s, horizontal NED frame
     *         X: North velocity component (m/s)
     *         Y: East velocity component (m/s)
     * 
     * @note EKF backends provide more accurate ground speed than GPS alone
     * @note Magnitude is horizontal ground speed: sqrt(north^2 + east^2)
     * 
     * @see groundspeed() for scalar ground speed
     * @see get_velocity_NED() for full 3D velocity including vertical
     */
    const Vector2f &groundspeed_vector() const { return state.ground_speed_vec; }

    /**
     * @brief Get scalar ground speed
     * 
     * @details Returns the magnitude of horizontal ground velocity in m/s.
     *          This is the vehicle's speed over the ground, regardless of
     *          heading direction.
     * 
     * @return Ground speed in m/s (scalar, always positive)
     * 
     * @note Primarily used by ground vehicles for speed control
     * @note For aircraft, consider using airspeed for flight control
     * @note Calculated as: sqrt(velocity_north^2 + velocity_east^2)
     * 
     * @see groundspeed_vector() for vector form with direction
     * @see get_velocity_NED() for full 3D velocity
     */
    float groundspeed(void) const { return state.ground_speed; }

    /**
     * @brief Get acceleration in earth frame
     * 
     * @details Returns the vehicle's acceleration vector in NED earth frame.
     *          This is the inertial acceleration (corrected for gravity) and
     *          is useful for advanced control algorithms and disturbance
     *          observation.
     * 
     * @return Acceleration vector in m/s², NED earth frame
     *         North: acceleration in north direction
     *         East: acceleration in east direction
     *         Down: acceleration in down direction (positive down)
     * 
     * @note Gravity is removed - this is inertial acceleration only
     * @note Includes corrections for centripetal acceleration
     * 
     * @see get_accel() for body frame acceleration
     */
    const Vector3f &get_accel_ef() const {
        return state.accel_ef;
    }

    /**
     * @brief Get corrected delta velocity in NED frame
     * 
     * @details Retrieves the corrected velocity change (delta-V) vector and
     *          associated time interval used by the inertial navigation system.
     *          This is the integrated and corrected accelerometer output.
     * 
     * @param[out] ret Corrected delta velocity in m/s, NED frame
     * @param[out] dt Time interval for delta velocity in seconds
     * 
     * @note Used by position controllers for velocity feed-forward
     * @note Corrections include gyro and accelerometer bias compensation
     * @note Useful for replay and log analysis
     * 
     * @see get_velocity_NED() for absolute velocity
     * @see get_accel_ef() for instantaneous acceleration
     */
    void getCorrectedDeltaVelocityNED(Vector3f& ret, float& dt) const {
        ret = state.corrected_dv;
        dt = state.corrected_dv_dt;
    }

    /**
     * @brief Set the EKF origin location
     * 
     * @details Sets the origin (reference point) for the EKF's local NED coordinate
     *          frame. This should only be called when the EKF has no absolute position
     *          reference (e.g., no GPS) from which to decide the origin automatically.
     * 
     * @param[in] loc Origin location (lat/lon/alt in degrees/meters)
     * 
     * @return true if origin was set successfully
     * @return false if origin could not be set (already set with GPS, backend unavailable)
     * 
     * @warning Only call this when GPS is unavailable or unreliable
     * @warning Setting incorrect origin will cause position errors
     * @note Origin is typically set automatically when first GPS fix is acquired
     * @note All relative position estimates are referenced to this origin
     * 
     * @see get_origin() to retrieve current origin
     * @see have_inertial_nav() to check if inertial navigation is available
     */
    bool set_origin(const Location &loc) WARN_IF_UNUSED;

#if AP_AHRS_POSITION_RESET_ENABLED
    /**
     * @brief Update EKF horizontal position from external estimate
     * 
     * @details Sets the EKF's NE horizontal position states and their variances from
     *          an external WGS-84 location and uncertainty. Used when the EKF is
     *          dead reckoning to periodically correct position drift. If the EKF is
     *          actively using a position sensor (GPS), the position update is ignored.
     * 
     * @param[in] loc External position estimate (lat/lon), altitude ignored
     * @param[in] pos_accuracy 1-sigma horizontal position uncertainty in meters
     * @param[in] timestamp_ Timestamp of external measurement in milliseconds
     * 
     * @return true if position was set successfully
     * @return false if EKF is using GPS or update rejected
     * 
     * @note Only effective during dead reckoning (no GPS)
     * @note pos_accuracy should be realistic (typically 1-10m)
     * @warning Inaccurate position updates will degrade EKF performance
     * 
     * @see writeExtNavData() for full 6DOF external navigation
     */
    bool handle_external_position_estimate(const Location &loc, float pos_accuracy, uint32_t timestamp_);
#endif

    /**
     * @brief Get the inertial navigation origin
     * 
     * @details Returns the origin (reference point) of the local NED coordinate frame
     *          used by the EKF for position estimation. All relative position estimates
     *          are referenced to this origin.
     * 
     * @param[out] ret Origin location (lat/lon in degrees, alt in meters)
     * 
     * @return true if origin is available and valid
     * @return false if origin not yet set (no GPS, not initialized)
     * 
     * @note Origin is typically set when first GPS fix is acquired
     * @note Origin can be manually set with set_origin() if no GPS
     * @note Altitude is relative to home or MSL depending on configuration
     * 
     * @see set_origin() to manually set origin
     * @see get_relative_position_NED_origin() for position relative to origin
     */
    bool get_origin(Location &ret) const WARN_IF_UNUSED;

    /**
     * @brief Check if inertial navigation is available
     * 
     * @details Returns true if the AHRS is providing inertial navigation estimates
     *          (position and velocity). This requires an EKF to be running with
     *          sufficient sensor data.
     * 
     * @return true if position/velocity estimates are available
     * @return false if only attitude available (DCM-only mode, EKF not initialized)
     * 
     * @note Many position/velocity methods require this to be true
     * @note Returns false if only DCM backend is active
     * @note EKF requires GPS or other absolute position reference
     * 
     * @see get_velocity_NED() requires this to be true
     * @see get_relative_position_NED_origin() requires this to be true
     */
    bool have_inertial_nav() const;

    /**
     * @brief Get velocity in NED frame
     * 
     * @details Returns the vehicle's 3D velocity vector in North-East-Down frame.
     *          This is the inertial velocity estimate from the EKF, fusing GPS,
     *          accelerometers, and other sensors.
     * 
     * @param[out] vec Velocity vector in m/s, NED frame
     *                 North: velocity in north direction
     *                 East: velocity in east direction
     *                 Down: velocity in down direction (positive = descending)
     * 
     * @return true if velocity estimate is valid
     * @return false if velocity unavailable
     * 
     * @warning Must only be called if have_inertial_nav() returns true
     * @note More accurate than GPS velocity alone due to sensor fusion
     * 
     * @see have_inertial_nav() to check availability
     * @see groundspeed() for horizontal speed magnitude
     * @see get_velocity_D() for vertical velocity only
     */
    bool get_velocity_NED(Vector3f &vec) const WARN_IF_UNUSED;

    /**
     * @brief Get position relative to home in NED frame
     * 
     * @details Returns the vehicle's position relative to the home location
     *          in North-East-Down coordinates. Home is typically set at the
     *          arming location.
     * 
     * @param[out] vec Position offset from home in meters, NED frame
     *                 North: meters north of home (positive = north)
     *                 East: meters east of home (positive = east)
     *                 Down: meters below home (positive = below)
     * 
     * @return true if position estimate is valid
     * @return false if position unavailable
     * 
     * @note Requires have_inertial_nav() to return true
     * @note Home must be set with set_home()
     * 
     * @see get_relative_position_NED_origin() for position relative to EKF origin
     * @see set_home() to set home location
     */
    bool get_relative_position_NED_home(Vector3f &vec) const WARN_IF_UNUSED;
    
    /**
     * @brief Get position relative to EKF origin in NED frame (high precision)
     * 
     * @details Returns the vehicle's position relative to the EKF origin using
     *          high-precision postype_t representation (typically double precision).
     * 
     * @param[out] vec Position offset from origin in meters, NED frame
     * 
     * @return true if position estimate is valid
     * @return false if position unavailable
     * 
     * @note Uses high precision for long-distance flights
     * @see get_relative_position_NED_origin_float() for single precision
     * @see get_origin() to get the origin location
     */
    bool get_relative_position_NED_origin(Vector3p &vec) const WARN_IF_UNUSED;
    
    /**
     * @brief Get position relative to EKF origin in NED frame (float)
     * 
     * @details Returns the vehicle's position relative to the EKF origin using
     *          standard float precision. Suitable for most applications.
     * 
     * @param[out] vec Position offset from origin in meters, NED frame
     * 
     * @return true if position estimate is valid
     * @return false if position unavailable
     * 
     * @note Standard precision (float), sufficient for typical flight ranges
     * @see get_relative_position_NED_origin() for high precision version
     */
    bool get_relative_position_NED_origin_float(Vector3f &vec) const WARN_IF_UNUSED;

    /**
     * @brief Get 2D horizontal position relative to home
     * 
     * @details Returns the vehicle's horizontal position relative to home
     *          (North and East components only), ignoring altitude.
     * 
     * @param[out] posNE 2D position in meters (North, East)
     * 
     * @return true if position estimate is valid
     * @return false if position unavailable
     * 
     * @note Useful for ground vehicles that don't need altitude
     * @see get_relative_position_NED_home() for full 3D position
     */
    bool get_relative_position_NE_home(Vector2f &posNE) const WARN_IF_UNUSED;
    
    /**
     * @brief Get 2D horizontal position relative to origin (high precision)
     * 
     * @details Returns the vehicle's horizontal position relative to EKF origin
     *          using high-precision representation.
     * 
     * @param[out] posNE 2D position in meters (North, East)
     * 
     * @return true if position estimate is valid
     * @return false if position unavailable
     * 
     * @see get_relative_position_NE_origin_float() for standard precision
     */
    bool get_relative_position_NE_origin(Vector2p &posNE) const WARN_IF_UNUSED;
    
    /**
     * @brief Get 2D horizontal position relative to origin (float)
     * 
     * @details Returns the vehicle's horizontal position relative to EKF origin
     *          using standard float precision.
     * 
     * @param[out] posNE 2D position in meters (North, East)
     * 
     * @return true if position estimate is valid
     * @return false if position unavailable
     */
    bool get_relative_position_NE_origin_float(Vector2f &posNE) const WARN_IF_UNUSED;

    /**
     * @brief Get altitude relative to home (Down component)
     * 
     * @details Returns the vehicle's altitude relative to home as the Down
     *          component in NED frame (positive = below home). Uses barometer
     *          if EKF is unavailable.
     * 
     * @param[out] posD Altitude in meters, Down component (positive = below home)
     * 
     * @note Always succeeds (returns barometer altitude if EKF unavailable)
     * @note Positive value means below home, negative means above home
     * 
     * @see get_relative_position_D_origin() for altitude relative to origin
     */
    void get_relative_position_D_home(float &posD) const;
    
    /**
     * @brief Get altitude relative to origin (Down component, high precision)
     * 
     * @details Returns the vehicle's altitude relative to EKF origin using
     *          high-precision representation.
     * 
     * @param[out] posD Altitude in meters, Down component
     * 
     * @return true if EKF altitude estimate is valid
     * @return false if altitude unavailable
     * 
     * @see get_relative_position_D_origin_float() for standard precision
     */
    bool get_relative_position_D_origin(postype_t &posD) const WARN_IF_UNUSED;
    
    /**
     * @brief Get altitude relative to origin (Down component, float)
     * 
     * @details Returns the vehicle's altitude relative to EKF origin using
     *          standard float precision.
     * 
     * @param[out] posD Altitude in meters, Down component
     * 
     * @return true if EKF altitude estimate is valid
     * @return false if altitude unavailable
     */
    bool get_relative_position_D_origin_float(float &posD) const WARN_IF_UNUSED;

    /**
     * @brief Convert NED offset from origin to Location
     * 
     * @details Converts a position offset in meters (NED frame) relative to the
     *          EKF origin into an absolute Location (lat/lon/alt).
     * 
     * @param[out] loc Resulting location (lat/lon in degrees, alt in meters)
     * @param[in] offset_ned Position offset in meters, NED frame from origin
     * 
     * @return true if conversion successful
     * @return false if origin not set or conversion failed
     * 
     * @note Useful for converting waypoints or obstacles from local to global coordinates
     * @see get_origin() to get the reference origin
     * @see get_location_from_home_offset_NED() to convert from home offset
     */
    bool get_location_from_origin_offset_NED(Location &loc, const Vector3p &offset_ned) const WARN_IF_UNUSED;
    
    /**
     * @brief Convert NED offset from home to Location
     * 
     * @details Converts a position offset in meters (NED frame) relative to the
     *          home location into an absolute Location (lat/lon/alt).
     * 
     * @param[out] loc Resulting location (lat/lon in degrees, alt in meters)
     * @param[in] offset_ned Position offset in meters, NED frame from home
     * 
     * @return true if conversion successful
     * @return false if home not set or conversion failed
     * 
     * @see get_home() to get home location
     * @see get_location_from_origin_offset_NED() to convert from origin offset
     */
    bool get_location_from_home_offset_NED(Location &loc, const Vector3p &offset_ned) const WARN_IF_UNUSED;

    /**
     * @brief Get vertical velocity (Down component)
     * 
     * @details Returns the vehicle's vertical velocity in m/s (Down component of
     *          NED frame). Normally returns EKF velocity, but can fall back to
     *          position rate derivative if requested or EKF unavailable.
     * 
     * @param[out] velD Vertical velocity in m/s (positive = descending)
     * @param[in] high_vibes Set true to use position derivative during high vibration
     * 
     * @return true if velocity estimate is available
     * @return false if velocity unavailable
     * 
     * @note If high_vibes is true, uses get_vert_pos_rate_D() instead of EKF velocity
     * @note Positive value = descending, negative = climbing
     * 
     * @see get_velocity_NED() for full 3D velocity
     * @see get_vert_pos_rate_D() for kinematically consistent position derivative
     */
    bool get_velocity_D(float &velD, bool high_vibes = false) const WARN_IF_UNUSED;

    /**
     * @brief Get vertical position rate (kinematically consistent)
     * 
     * @details Returns a derivative of the vertical position in m/s that is
     *          kinematically consistent with the vertical position estimate.
     *          This differs from EKF velocity which may have transient
     *          inconsistencies during error correction.
     * 
     * @param[out] velocity Vertical position rate in m/s (positive = descending)
     * 
     * @return true if position rate is available
     * @return false if unavailable
     * 
     * @note Required by some control loops that need position-velocity consistency
     * @note May differ from get_velocity_NED().z during EKF corrections
     * @note Used during high vibration conditions
     * 
     * @see get_velocity_D() for standard vertical velocity
     */
    bool get_vert_pos_rate_D(float &velocity) const;

    /**
     * @brief Write optical flow measurements to EKF
     * 
     * @details Sends optical flow sensor data to the EKF for vision-based
     *          velocity estimation and position hold without GPS.
     * 
     * @param[in] rawFlowQuality Flow quality indicator (0-255, 255=best)
     * @param[in] rawFlowRates Flow rates in rad/s (X and Y body frame)
     * @param[in] rawGyroRates Gyro rates in rad/s at flow measurement time
     * @param[in] msecFlowMeas Timestamp of measurement in milliseconds
     * @param[in] posOffset Sensor position offset from IMU in body frame (meters)
     * @param[in] heightOverride Height above ground in meters (negative = use rangefinder)
     * 
     * @note Called by optical flow driver when new data available
     * @note EKF uses flow data when GPS unavailable or unreliable
     * @note Quality should reflect measurement confidence
     * 
     * @see getOptFlowSample() to retrieve corrected flow data
     */
    void writeOptFlowMeas(const uint8_t rawFlowQuality, const Vector2f &rawFlowRates, const Vector2f &rawGyroRates, const uint32_t msecFlowMeas, const Vector3f &posOffset, const float heightOverride);

    /**
     * @brief Retrieve latest corrected optical flow sample
     * 
     * @details Gets the most recent optical flow measurement corrected by the EKF.
     *          Used for optical flow sensor calibration and validation.
     * 
     * @param[out] timeStamp_ms Timestamp of flow sample in milliseconds
     * @param[out] flowRate Corrected flow rate in rad/s
     * @param[out] bodyRate Body rotation rate at measurement time in rad/s
     * @param[out] losPred Predicted line-of-sight rate in rad/s
     * 
     * @return true if valid flow sample available
     * @return false if no flow data or EKF not using flow
     * 
     * @note Used for sensor calibration and diagnostics
     * @see writeOptFlowMeas() to send flow measurements
     */
    bool getOptFlowSample(uint32_t& timeStamp_ms, Vector2f& flowRate, Vector2f& bodyRate, Vector2f& losPred) const;

    /**
     * @brief Write body frame odometry measurements to EKF
     * 
     * @details Sends visual or wheel odometry measurements to the EKF. These are
     *          delta position and rotation measurements in the vehicle's body frame.
     * 
     * @param[in] quality Measurement quality indicator (0.0-1.0, 1.0=best)
     * @param[in] delPos Delta position in body frame in meters (Forward-Right-Down)
     * @param[in] delAng Delta rotation angles in body frame in radians
     * @param[in] delTime Time interval for delta measurements in seconds
     * @param[in] timeStamp_ms Timestamp of measurement in milliseconds
     * @param[in] delay_ms Sensor delay from measurement to AHRS in milliseconds
     * @param[in] posOffset Sensor position offset from IMU in body frame (meters)
     * 
     * @note Used for visual odometry (e.g., Intel T265) or wheel encoders
     * @note Quality should reflect measurement confidence (0=invalid, 1=perfect)
     * @note delPos is in body frame: X=forward, Y=right, Z=down
     * 
     * @see writeExtNavData() for absolute position/attitude from external nav
     */
    void writeBodyFrameOdom(float quality, const Vector3f &delPos, const Vector3f &delAng, float delTime, uint32_t timeStamp_ms, uint16_t delay_ms, const Vector3f &posOffset);

    /**
     * @brief Write default airspeed for EKF
     * 
     * @details Sets the default equivalent airspeed and uncertainty to be used
     *          when no airspeed sensor is available but airspeed is needed for
     *          fixed-wing flight (e.g., for drag compensation).
     * 
     * @param[in] airspeed Default equivalent airspeed in m/s
     * @param[in] uncertainty 1-sigma airspeed uncertainty in m/s
     * 
     * @note Only used when airspeed sensor unavailable
     * @note Typically set to expected cruise airspeed
     * @note Uncertainty should be large (e.g., 5 m/s) to reflect lack of measurement
     */
    void writeDefaultAirSpeed(float airspeed, float uncertainty);

    /**
     * @brief Write external navigation position and attitude
     * 
     * @details Sends absolute position and attitude from an external navigation
     *          system (e.g., motion capture, RTK GPS+compass) to the EKF.
     * 
     * @param[in] pos Position in NED frame relative to EKF origin in meters
     * @param[in] quat Attitude quaternion (rotation from NED to body frame)
     * @param[in] posErr 1-sigma position uncertainty in meters
     * @param[in] angErr 1-sigma attitude uncertainty in radians
     * @param[in] timeStamp_ms Timestamp of measurement in milliseconds
     * @param[in] delay_ms Sensor delay from measurement to AHRS in milliseconds
     * @param[in] resetTime_ms Time of last external nav reset in milliseconds
     * 
     * @note Used with external navigation systems (Vicon, OptiTrack, etc.)
     * @note Position is in NED frame, origin must be aligned with EKF origin
     * @note Quaternion must be normalized
     * 
     * @see writeExtNavVelData() to also provide velocity data
     * @see writeBodyFrameOdom() for relative (delta) measurements
     */
    void writeExtNavData(const Vector3f &pos, const Quaternion &quat, float posErr, float angErr, uint32_t timeStamp_ms, uint16_t delay_ms, uint32_t resetTime_ms);

    /**
     * @brief Write external navigation velocity
     * 
     * @details Sends velocity from an external navigation system to the EKF.
     *          Used in conjunction with writeExtNavData() for improved velocity
     *          estimation.
     * 
     * @param[in] vel Velocity in NED frame in m/s
     * @param[in] err 1-sigma velocity uncertainty in m/s
     * @param[in] timeStamp_ms Timestamp of measurement in milliseconds
     * @param[in] delay_ms Sensor delay from measurement to AHRS in milliseconds
     * 
     * @note Complements position/attitude from writeExtNavData()
     * @note Velocity is in NED frame, same as position
     * 
     * @see writeExtNavData() for position and attitude
     */
    void writeExtNavVelData(const Vector3f &vel, float err, uint32_t timeStamp_ms, uint16_t delay_ms);

    /**
     * @brief Get EKF control limits
     * 
     * @details Retrieves control scaling factors from the EKF that indicate when
     *          position control should be limited due to poor navigation accuracy.
     * 
     * @param[out] ekfGndSpdLimit Maximum ground speed limit in m/s (0=no limit)
     * @param[out] controlScaleXY Horizontal control scaling factor (0.0-1.0)
     * 
     * @note controlScaleXY < 1.0 indicates position control should be reduced
     * @note Used by position controllers to prevent overshoot during poor GPS
     * @note ekfGndSpdLimit enforced by controllers when non-zero
     * 
     * @see getControlScaleZ() for vertical control scaling
     */
    void getControlLimits(float &ekfGndSpdLimit, float &controlScaleXY) const;
    
    /**
     * @brief Get vertical control scaling factor
     * 
     * @details Returns a scaling factor (0.0-1.0) indicating how much vertical
     *          position control should be reduced due to poor altitude accuracy.
     * 
     * @return Vertical control scale (0.0=no control, 1.0=full control)
     * 
     * @note < 1.0 indicates altitude control should be reduced
     * @note Used during optical flow navigation or poor baro conditions
     * 
     * @see getControlLimits() for horizontal control limits
     */
    float getControlScaleZ(void) const;

    /**
     * @brief Check if AHRS subsystem is healthy
     * 
     * @details Returns true if the AHRS is providing reliable attitude and
     *          position estimates. Considers backend health, sensor health,
     *          and innovation checks.
     * 
     * @return true if AHRS is healthy and estimates are reliable
     * @return false if AHRS unhealthy (sensor failure, high innovations, etc.)
     * 
     * @note Checked before critical operations (takeoff, auto missions)
     * @note Failure may trigger EKF failsafe
     * 
     * @see pre_arm_check() for detailed pre-arming validation
     * @see get_filter_status() for detailed status flags
     */
    bool healthy() const;

    /**
     * @brief Perform pre-arm AHRS checks
     * 
     * @details Validates that the AHRS is ready for arming and flight. Checks
     *          backend initialization, sensor health, attitude consistency,
     *          and optionally position availability.
     * 
     * @param[in] requires_position true if horizontal position must be available
     * @param[out] failure_msg Buffer to receive failure message if check fails
     * @param[in] failure_msg_len Size of failure_msg buffer
     * 
     * @return true if all pre-arm checks passed
     * @return false if arming should be prevented, failure_msg contains reason
     * 
     * @note failure_msg populated with human-readable failure reason
     * @note requires_position should be true for AUTO, GUIDED, LOITER modes
     * @note Checks attitude consistency between multiple backends
     * 
     * @see healthy() for runtime health check
     * @see attitudes_consistent() for multi-backend consistency
     */
    bool pre_arm_check(bool requires_position, char *failure_msg, uint8_t failure_msg_len) const;

    /**
     * @brief Check if AHRS has completed initialization
     * 
     * @details Returns true if the AHRS has finished initial startup and
     *          is providing valid estimates. May take several seconds after
     *          boot for full initialization.
     * 
     * @return true if AHRS initialized and providing estimates
     * @return false if still initializing
     * 
     * @note DCM initializes quickly (~1 second)
     * @note EKF may take 10-20 seconds to fully initialize
     * @note Should be true before attempting flight
     * 
     * @see healthy() to check ongoing health after initialization
     */
    bool initialised() const;

#if AP_AHRS_DCM_ENABLED
    /**
     * @brief Check if DCM yaw has been initialized
     * 
     * @details Returns true if the DCM backend has completed yaw initialization.
     *          DCM uses GPS velocity or compass to initialize yaw heading.
     * 
     * @return true if DCM yaw initialized
     * @return false if DCM yaw not yet initialized
     * 
     * @note Only available when DCM backend is compiled in
     * @note DCM requires GPS velocity or compass for yaw initialization
     */
    bool dcm_yaw_initialised(void) const {
        return dcm.yaw_initialised();
    }
#endif

    /**
     * @brief Get EKF filter status flags
     * 
     * @details Returns the navigation filter status as a series of bit flags
     *          indicating which sensors are being used and filter health.
     * 
     * @param[out] status Filter status structure with status flags
     * 
     * @return true if status retrieved successfully
     * @return false if no backend providing status
     * 
     * @note Status flags defined in nav_filter_status structure
     * @note Includes: attitude valid, horiz/vert velocity valid, position valid, etc.
     * 
     * @see has_status() to check individual status bits
     * @see healthy() for simplified health check
     */
    bool get_filter_status(nav_filter_status &status) const;

    /**
     * @brief Get compass offset estimates
     * 
     * @details Returns the EKF's current estimate of compass offsets (hard iron
     *          bias) for the specified magnetometer.
     * 
     * @param[in] mag_idx Magnetometer index (0 for primary)
     * @param[out] magOffsets Compass offset vector in milligauss, body frame
     * 
     * @return true if offsets are valid and available
     * @return false if offsets unavailable (EKF not learning, invalid mag_idx)
     * 
     * @note Offsets learned during flight to compensate for hard iron interference
     * @note Used for compass calibration and validation
     * 
     * @see get_mag_field_correction() for total magnetic field correction
     */
    bool getMagOffsets(uint8_t mag_idx, Vector3f &magOffsets) const;

    /**
     * @brief Get last yaw reset angle and time
     * 
     * @details Returns the amount of yaw angle change from the most recent EKF
     *          yaw reset and when it occurred. Used to correct attitude targets
     *          after reset.
     * 
     * @param[out] yawAng Yaw angle change in radians (positive = clockwise)
     * 
     * @return Time of last yaw reset in milliseconds
     * @return 0 if no reset has ever occurred
     * 
     * @note Controllers use this to adjust setpoints after EKF reset
     * @note Yaw resets occur when switching between yaw sources
     * 
     * @see request_yaw_reset() to trigger yaw reset
     */
    uint32_t getLastYawResetAngle(float &yawAng);

    /**
     * @brief Get last horizontal position reset and time
     * 
     * @details Returns the amount of North-East position change from the most
     *          recent EKF position reset and when it occurred.
     * 
     * @param[out] pos Position change in meters (North, East)
     * 
     * @return Time of last position reset in milliseconds
     * @return 0 if no reset has ever occurred
     * 
     * @note Position resets occur when switching GPS, using external nav, etc.
     * @note Controllers use this to adjust position targets after reset
     * 
     * @see getLastPosDownReset() for vertical position reset
     */
    uint32_t getLastPosNorthEastReset(Vector2f &pos);

    /**
     * @brief Get last horizontal velocity reset and time
     * 
     * @details Returns the amount of North-East velocity change from the most
     *          recent EKF velocity reset and when it occurred.
     * 
     * @param[out] vel Velocity change in m/s (North, East)
     * 
     * @return Time of last velocity reset in milliseconds
     * @return 0 if no reset has ever occurred
     * 
     * @note Velocity resets occur when switching velocity sources
     * @note Controllers use this to adjust velocity targets after reset
     */
    uint32_t getLastVelNorthEastReset(Vector2f &vel) const;

    /**
     * @brief Get last vertical position reset and time
     * 
     * @details Returns the amount of vertical position (Down) change from the
     *          most recent EKF altitude reset and when it occurred.
     * 
     * @param[out] posDelta Vertical position change in meters (Down component)
     * 
     * @return Time of last altitude reset in milliseconds
     * @return 0 if no reset has ever occurred
     * 
     * @note Altitude resets occur when switching barometer, using rangefinder, etc.
     * @note Controllers use this to adjust altitude targets after reset
     * 
     * @see getLastPosNorthEastReset() for horizontal position reset
     */
    uint32_t getLastPosDownReset(float &posDelta);

    /**
     * @brief Reset height datum to current altitude
     * 
     * @details Resets the barometer to read zero at the current height and
     *          adjusts the EKF origin altitude so that the absolute altitude
     *          (EKF height + origin height) remains unchanged.
     * 
     * @return true if height datum reset was performed
     * @return false if using rangefinder for height (no reset performed)
     * 
     * @note Used when landing or setting new altitude reference
     * @note Does not affect absolute altitude, only relative altitude reference
     * @note Not performed if rangefinder is primary height source
     * 
     * @warning Should not be called during flight
     */
    bool resetHeightDatum();

    /**
     * @brief Send EKF status report to ground station
     * 
     * @details Sends a MAVLink EKF_STATUS_REPORT message containing detailed
     *          EKF health and status information for the active EKF.
     * 
     * @param[in] link MAVLink communication channel to send message on
     * 
     * @note Called periodically by GCS telemetry streaming
     * @note Includes velocity/position variance, compass/airspeed health, etc.
     * 
     * @see get_filter_status() for programmatic status access
     */
    void send_ekf_status_report(class GCS_MAVLINK &link) const;

    /**
     * @brief Get height control limit for optical flow
     * 
     * @details Returns the maximum height that should be observed by control
     *          loops when using optical flow for navigation. Used to prevent
     *          altitude excursions beyond optical flow range.
     * 
     * @param[out] limit Maximum height limit in meters above ground
     * 
     * @return true if height limit is valid and should be enforced
     * @return false if no limiting is required
     * 
     * @note Only returns valid limit during optical flow navigation
     * @note Typically limited to rangefinder maximum range
     * @note Controllers clamp altitude setpoints to this limit
     * 
     * @see have_inertial_nav() to check if position control available
     */
    bool get_hgt_ctrl_limit(float &limit) const;

    /**
     * @brief Set terrain height stability flag
     * 
     * @details Informs the EKF whether the terrain underneath is stable enough
     *          to be used as a height reference (e.g., for rangefinder height).
     * 
     * @param[in] stable true if terrain is stable, false if unstable
     * 
     * @note Not related to terrain following (AP_Terrain)
     * @note Used to determine if rangefinder can be used for height
     * @note Set false when over water, tall grass, or moving objects
     * 
     * @see get_hgt_ctrl_limit() for height limiting during optical flow
     */
    void set_terrain_hgt_stable(bool stable);

    /**
     * @brief Get EKF innovations for all measurement types
     * 
     * @details Returns the innovation (measurement minus prediction) values for
     *          velocity, position, magnetometer, airspeed, and yaw measurements.
     *          Large innovations indicate sensor disagreement or EKF problems.
     * 
     * @param[out] velInnov Velocity innovation in m/s (North, East, Down)
     * @param[out] posInnov Position innovation in meters (North, East, Down)
     * @param[out] magInnov Magnetometer innovation in milligauss (X, Y, Z body frame)
     * @param[out] tasInnov True airspeed innovation in m/s
     * @param[out] yawInnov Yaw innovation in radians
     * 
     * @return true if innovations available
     * @return false if EKF not providing innovations
     * 
     * @note Large innovations (>3-5x variance) indicate sensor problems
     * @note Used for sensor health monitoring and pre-arming checks
     * @note Instance parameter of -1 returns primary EKF innovations
     * 
     * @see get_variances() for innovation variances
     * @see is_vibration_affected() for vibration-related degradation
     */
    bool get_innovations(Vector3f &velInnov, Vector3f &posInnov, Vector3f &magInnov, float &tasInnov, float &yawInnov) const;

    /**
     * @brief Check if vibration is affecting state estimates
     * 
     * @details Returns true when the EKF determines that vehicle vibration
     *          is significantly degrading navigation accuracy.
     * 
     * @return true if vibration is affecting estimates
     * @return false if vibration levels acceptable
     * 
     * @note High vibration detected through IMU clipping or accel variance
     * @note May trigger EKF_VIBRATION_AFFECTED message to GCS
     * @note Controllers may reduce gains or avoid aggressive maneuvers
     * 
     * @see get_vibration() for current vibration level
     */
    bool is_vibration_affected() const;

    /**
     * @brief Get normalized EKF innovation variances
     * 
     * @details Returns the innovation variances normalized by measurement variance.
     *          Value of 0 indicates perfect consistency, 1 is maximum inconsistency
     *          that will be accepted before measurement is rejected.
     * 
     * @param[out] velVar Normalized velocity variance (0-1)
     * @param[out] posVar Normalized position variance (0-1)
     * @param[out] hgtVar Normalized height variance (0-1)
     * @param[out] magVar Normalized magnetometer variance (X, Y, Z)
     * @param[out] tasVar Normalized true airspeed variance (0-1)
     * 
     * @return true if variances available
     * @return false if variances not available (EKF not running)
     * 
     * @note Values approaching 1.0 indicate sensor degradation
     * @note Used for EKF health monitoring and pre-arm checks
     * @note Normalized variance > 1.0 causes measurement rejection
     * 
     * @see get_innovations() for raw innovation values
     */
    bool get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar) const;

    /**
     * @brief Get velocity innovations for specific source
     * 
     * @details Returns velocity innovations and variances for a specific
     *          velocity source (GPS, external nav, optical flow, etc.).
     * 
     * @param[in] source Velocity source index
     * @param[out] innovations Velocity innovations in m/s (North, East, Down)
     * @param[out] variances Innovation variances in (m/s)²
     * 
     * @return true if innovations available for specified source
     * @return false if source invalid or innovations unavailable
     * 
     * @note Used for multi-source velocity fusion health monitoring
     * @note Allows checking individual GPS or external nav health
     * 
     * @see get_innovations() for general innovation retrieval
     */
    bool get_vel_innovations_and_variances_for_source(uint8_t source, Vector3f &innovations, Vector3f &variances) const WARN_IF_UNUSED;

    /**
     * @brief Get expected magnetic field in NED frame
     * 
     * @details Returns the Earth's magnetic field that the EKF expects at the
     *          current location, expressed in NED frame. Based on world magnetic
     *          model and learned declination.
     * 
     * @param[out] ret Expected magnetic field vector in milligauss (North, East, Down)
     * 
     * @return true if expected field available
     * @return false if not available (EKF not initialized)
     * 
     * @note Used for compass calibration and validation
     * @note Includes learned declination corrections
     * @note Magnitude typically ~450-650 milligauss depending on location
     * 
     * @see get_mag_field_correction() for body frame field correction
     */
    bool get_mag_field_NED(Vector3f& ret) const;

    /**
     * @brief Get estimated magnetic field correction in body frame
     * 
     * @details Returns the EKF's estimate of magnetic field offsets in body frame,
     *          representing the total correction for hard and soft iron effects.
     * 
     * @param[out] ret Magnetic field correction vector in milligauss (body frame)
     * 
     * @return true if correction estimate available
     * @return false if not available (EKF not learning field)
     * 
     * @note Learned during flight to compensate for vehicle magnetic interference
     * @note Used for compass calibration validation
     * 
     * @see getMagOffsets() for per-compass offset estimates
     * @see get_mag_field_NED() for expected field in NED frame
     */
    bool get_mag_field_correction(Vector3f &ret) const;

    /**
     * @brief Get active airspeed sensor index
     * 
     * @details Returns the index of the airspeed sensor currently being used
     *          by the active EKF lane. With multiple airspeed sensors and lane
     *          switching, the active sensor may differ from the primary.
     * 
     * @return Index of airspeed sensor in use by active EKF lane
     * 
     * @note Important after EKF lane switch due to airspeed sensor fault
     * @note With airspeed affinity, different lanes may use different sensors
     * @note Used by airspeed library to determine which sensor to report
     * 
     * @see airspeed_sensor_enabled() to check if airspeed sensor available
     * @see get_primary_core_index() for active EKF lane index
     */
    uint8_t get_active_airspeed_index() const;

    /**
     * @brief Get primary EKF core index
     * 
     * @details Returns the index of the currently active primary EKF core (lane).
     *          EKF3 can run multiple lanes in parallel and switch between them.
     * 
     * @return Index of primary EKF core (0-based)
     * @return -1 if no primary core selected
     * 
     * @note EKF3 typically runs 2-4 lanes with different tuning
     * @note Lane switching occurs when primary becomes unhealthy
     * @note Used for logging and debugging EKF behavior
     * 
     * @see check_lane_switch() to attempt lane switch
     */
    int8_t get_primary_core_index() const { return state.primary_core; }

    /**
     * @brief Get primary accelerometer sensor index
     * 
     * @details Returns the index of the accelerometer currently being used
     *          as the primary sensor by the AHRS/EKF.
     * 
     * @return Index of primary accelerometer (0-based)
     * 
     * @note May change if IMU sensor fails or degrades
     * @note Used by sensor libraries to identify active sensor
     * 
     * @see get_primary_gyro_index() for primary gyro sensor
     */
    uint8_t get_primary_accel_index(void) const { return state.primary_accel; }

    /**
     * @brief Get primary gyro sensor index
     * 
     * @details Returns the index of the gyroscope currently being used
     *          as the primary sensor by the AHRS/EKF.
     * 
     * @return Index of primary gyro (0-based)
     * 
     * @note May change if IMU sensor fails or degrades
     * @note Used by sensor libraries to identify active sensor
     * 
     * @see get_primary_accel_index() for primary accelerometer
     */
    uint8_t get_primary_gyro_index(void) const { return state.primary_gyro; }

    /**
     * @brief Attempt EKF lane switch to avoid failsafe
     * 
     * @details Checks if switching to a healthier EKF lane is possible and
     *          performs the switch if a better lane is available. Used to
     *          avoid EKF failsafe by proactively switching away from degrading lane.
     * 
     * @note Called when EKF health degrades but failsafe not yet triggered
     * @note EKF3 runs multiple lanes in parallel for redundancy
     * @note Switch occurs only if alternate lane is significantly healthier
     * 
     * @see request_yaw_reset() for yaw reset as alternative to lane switch
     * @see get_primary_core_index() to check which lane is active
     */
    void check_lane_switch(void);

    /**
     * @brief Request EKF yaw reset
     * 
     * @details Requests that the EKF reset its yaw estimate to try to recover
     *          from yaw errors without requiring a full EKF lane switch or failsafe.
     * 
     * @note Used when yaw innovations are high but other states healthy
     * @note EKF will reinitialize yaw from GPS velocity or compass
     * @note Less disruptive than lane switch or failsafe
     * 
     * @warning May cause brief attitude disturbance during reset
     * 
     * @see check_lane_switch() for lane switching alternative
     * @see getLastYawResetAngle() to detect when reset occurred
     */
    void request_yaw_reset(void);

    /**
     * @brief Set position/velocity/yaw source set
     * 
     * @details Selects which set of position, velocity, and yaw sources the
     *          EKF should use (primary, secondary, or tertiary configuration).
     * 
     * @param[in] source_set_idx Source set selection (0=primary, 1=secondary, 2=tertiary)
     * 
     * @note Used for switching between GPS/external nav/optical flow
     * @note Source sets configured via EKF3_SRCn_* parameters
     * @note Allows runtime switching of navigation sources
     * 
     * @see get_posvelyaw_source_set() to check active source set
     */
    void set_posvelyaw_source_set(AP_NavEKF_Source::SourceSetSelection source_set_idx);

    /**
     * @brief Get active position/velocity/yaw source set
     * 
     * @details Returns which source set (primary, secondary, or tertiary) is
     *          currently being used by the EKF for position, velocity, and yaw.
     * 
     * @return Active source set index (0=primary, 1=secondary, 2=tertiary)
     * 
     * @note Source sets allow switching between GPS/external nav/optical flow
     * @note Can be changed with set_posvelyaw_source_set()
     * 
     * @see set_posvelyaw_source_set() to change source set
     */
    uint8_t get_posvelyaw_source_set() const;

    /**
     * @brief Write AHRS log messages
     * 
     * @details Writes comprehensive AHRS telemetry to onboard dataflash log,
     *          including attitude, position, velocity, and EKF status.
     * 
     * @note Called periodically by main vehicle loop (typically 10-25Hz)
     * @note Logs multiple message types: AHRS, AHRS2, AHRS3, POS, etc.
     * @note Essential for post-flight analysis and debugging
     * 
     * @see Write_Attitude() for attitude-specific logging
     * @see Log_Write_Home_And_Origin() for home/origin logging
     */
    void Log_Write();

    /**
     * @brief Check if non-compass sensor is providing yaw
     * 
     * @details Returns true if yaw estimate is coming from a source other than
     *          magnetometer (e.g., GPS velocity, external nav, GSF estimator).
     * 
     * @return true if yaw from non-compass source
     * @return false if yaw from magnetometer
     * 
     * @note Allows compass pre-arm checks to be bypassed
     * @note GPS velocity can provide yaw when moving
     * @note External nav systems may provide yaw directly
     * 
     * @see using_extnav_for_yaw() specifically for external nav
     * @see use_compass() to check if compass is being used
     */
    bool using_noncompass_for_yaw(void) const;

    /**
     * @brief Check if external nav is providing yaw
     * 
     * @details Returns true if yaw estimate is specifically coming from an
     *          external navigation system (e.g., motion capture, VIO).
     * 
     * @return true if external nav providing yaw
     * @return false if yaw from other source
     * 
     * @note External nav via writeExtNavData() with quaternion
     * @note Allows bypassing compass requirements
     * 
     * @see using_noncompass_for_yaw() for any non-compass yaw source
     * @see writeExtNavData() for providing external nav data
     */
    bool using_extnav_for_yaw(void) const;

    /**
     * @brief Set altitude measurement noise parameter
     * 
     * @details Sets and saves the EKF altitude measurement noise (ALT_M_NSE)
     *          parameter, which controls how much the EKF trusts barometer altitude.
     * 
     * @param[in] noise Altitude measurement noise in meters
     * 
     * @note Higher values make EKF trust barometer less
     * @note Used for runtime tuning of altitude fusion
     * @note Saved to EEPROM for persistence
     * 
     * @warning Should be used carefully as it affects altitude control
     */
    void set_alt_measurement_noise(float noise);

    /**
     * @brief Get selected EKF type
     * 
     * @details Returns the currently selected EKF type from the EKF_TYPE parameter.
     *          Used for allocation decisions and configuration.
     * 
     * @return EKF type code (0=DCM, 2=EKF2, 3=EKF3, 10=SIM, 11=External)
     * 
     * @note Returns configured type, not necessarily active type
     * @note Used by vehicle code to allocate appropriate backends
     * 
     * @see active_EKF_type() for currently active backend
     * @see set_ekf_type() to change EKF selection
     */
    int8_t get_ekf_type(void) const {
        return _ekf_type;
    }

    /**
     * @enum EKFType
     * @brief EKF backend type selection
     * 
     * @details Defines available AHRS/EKF backend types. Not all types available
     *          on all platforms due to memory constraints.
     * 
     * @note DCM = Direction Cosine Matrix (legacy, lightweight)
     * @note TWO = NavEKF2 (Extended Kalman Filter version 2)
     * @note THREE = NavEKF3 (Extended Kalman Filter version 3, recommended)
     * @note SIM = SITL simulation backend
     * @note EXTERNAL = External AHRS via serial/CAN
     */
    enum class EKFType : uint8_t {
#if AP_AHRS_DCM_ENABLED
        DCM = 0,        ///< Direction Cosine Matrix (complementary filter)
#endif
#if HAL_NAVEKF3_AVAILABLE
        THREE = 3,      ///< NavEKF3 - Extended Kalman Filter version 3
#endif
#if HAL_NAVEKF2_AVAILABLE
        TWO = 2,        ///< NavEKF2 - Extended Kalman Filter version 2
#endif
#if AP_AHRS_SIM_ENABLED
        SIM = 10,       ///< SITL simulation backend
#endif
#if AP_AHRS_EXTERNAL_ENABLED
        EXTERNAL = 11,  ///< External AHRS (VectorNav, MicroStrain, etc.)
#endif
    };

    /**
     * @brief Set EKF type selection
     * 
     * @details Changes the active EKF backend type. Can be used for runtime
     *          switching via RC auxiliary function or GCS command.
     * 
     * @param[in] ahrs_type Desired EKF type from EKFType enum
     * 
     * @note Takes effect on next AHRS update cycle
     * @note Not all types available on all boards (compile-time configuration)
     * @note EKF3 (THREE) is recommended for most applications
     * 
     * @see get_ekf_type() to get configured type
     * @see EKFType for available backend types
     */
    void set_ekf_type(EKFType ahrs_type) {
        _ekf_type.set(ahrs_type);
    }
    
    // these are only out here so vehicles can reference them for parameters
#if HAL_NAVEKF2_AVAILABLE
    NavEKF2 EKF2;
#endif
#if HAL_NAVEKF3_AVAILABLE
    NavEKF3 EKF3;
#endif

    /**
     * @brief Parameter metadata for AP_AHRS
     * 
     * @details Table defining all configurable parameters for AHRS including
     *          board orientation, EKF selection, GPS usage, wind limits, etc.
     * 
     * @note Parameters prefixed with AHRS_* in ground station
     * @note See libraries/AP_AHRS/AP_AHRS.cpp for parameter definitions
     */
    static const struct AP_Param::GroupInfo var_info[];

    /**
     * @brief Create a rotated AHRS view
     * 
     * @details Creates an AP_AHRS_View object that provides attitude and position
     *          estimates from a rotated perspective (e.g., for gimbals, tilted cameras).
     * 
     * @param[in] rotation Rotation to apply to the view
     * @param[in] pitch_trim_deg Optional pitch trim in degrees (default 0)
     * 
     * @return Pointer to newly created AP_AHRS_View object
     * @return nullptr if view creation failed
     * 
     * @note Caller responsible for deleting returned view when done
     * @note Used for gimbal control, tilted cameras, or antenna tracking
     * @note View maintains rotated DCM matrix and position
     * 
     * @see AP_AHRS_View for view interface
     * @see get_view() to access current view
     */
    AP_AHRS_View *create_view(enum Rotation rotation, float pitch_trim_deg=0);

    /**
     * @brief Write angle of attack and sideslip angle to log
     * 
     * @details Logs the current AOA (Angle of Attack) and SSA (Sideslip Angle)
     *          estimates to dataflash for aerodynamic analysis.
     * 
     * @note AOA/SSA calculated from airspeed vector and attitude
     * @note Updated at 10Hz typically
     * @note Logged in AOA message type
     * 
     * @see getAOA() to retrieve current angle of attack
     * @see getSSA() to retrieve current sideslip angle
     */
    void Write_AOA_SSA(void) const;

    /**
     * @brief Get angle of attack
     * 
     * @details Returns the current estimated angle of attack (AOA) - the angle
     *          between the vehicle's longitudinal axis and the relative wind.
     * 
     * @return Angle of attack in radians (positive = nose up relative to airflow)
     * 
     * @note Calculated from airspeed vector and vehicle attitude
     * @note Only valid when airspeed estimate available
     * @note Used for stall prevention and aerodynamic analysis
     * 
     * @see getSSA() for sideslip angle
     * @see airspeed_vector_true() for airspeed vector
     */
    float getAOA(void) const { return _AOA; }

    /**
     * @brief Get sideslip angle
     * 
     * @details Returns the current estimated sideslip angle (SSA) - the angle
     *          between the vehicle's longitudinal axis and horizontal projection
     *          of relative wind in the body Y-Z plane.
     * 
     * @return Sideslip angle in radians (positive = wind from right)
     * 
     * @note Calculated from airspeed vector and vehicle attitude
     * @note Only valid when airspeed estimate available
     * @note Used for coordinated turn analysis and aerodynamic tuning
     * 
     * @see getAOA() for angle of attack
     * @see airspeed_vector_true() for airspeed vector
     */
    float getSSA(void) const { return _SSA; }

    /*
     * Trim-related functions
     * 
     * Trim compensates for CG offset, airframe asymmetry, or sensor mounting
     * errors. Stored as roll/pitch offsets applied between controller and vehicle.
     */

    /**
     * @brief Get current trim values
     * 
     * @details Returns the current roll/pitch/yaw trim offsets that compensate
     *          for vehicle asymmetry or sensor mounting errors.
     * 
     * @return Trim vector in radians (roll, pitch, yaw)
     * 
     * @note Trim offsets applied between autopilot body and vehicle body frames
     * @note Typically set during initial flight testing or auto-trim
     * @note Stored persistently in AHRS_TRIM_X/Y parameters
     * 
     * @see set_trim() to update trim values
     * @see add_trim() to incrementally adjust trim
     */
    const Vector3f &get_trim() const { return _trim.get(); }

    /**
     * @brief Set trim values
     * 
     * @details Sets the roll/pitch/yaw trim offsets to compensate for vehicle
     *          asymmetry or sensor mounting errors. Updates rotation matrices.
     * 
     * @param[in] new_trim Trim vector in radians (roll, pitch, yaw)
     * 
     * @note Trim offsets applied between autopilot body and vehicle body frames
     * @note Updates _rotation_autopilot_body_to_vehicle_body matrix
     * @note Should be saved to EEPROM for persistence
     * 
     * @see get_trim() to retrieve current trim
     * @see add_trim() for incremental trim adjustment
     */
    void set_trim(const Vector3f &new_trim);

    /**
     * @brief Add incremental trim adjustment
     * 
     * @details Adjusts the roll and pitch trim by the specified amounts, up to
     *          a total limit of 10 degrees. Used for in-flight trim adjustment.
     * 
     * @param[in] roll_in_radians Roll trim adjustment in radians
     * @param[in] pitch_in_radians Pitch trim adjustment in radians
     * @param[in] save_to_eeprom If true, save to EEPROM (default true)
     * 
     * @note Total trim limited to ±10 degrees for safety
     * @note Typically called by pilot trim switch or auto-trim function
     * @note Updates rotation matrices automatically
     * 
     * @warning Large trim values may indicate CG or airframe problems
     * 
     * @see set_trim() to set absolute trim values
     * @see get_trim() to retrieve current trim
     */
    void add_trim(float roll_in_radians, float pitch_in_radians, bool save_to_eeprom = true);

    /**
     * @brief Get rotation matrix from autopilot to vehicle body frame
     * 
     * @details Returns the rotation matrix that transforms vectors from the
     *          autopilot body frame to the vehicle body frame, incorporating trim.
     * 
     * @return Rotation matrix (autopilot body → vehicle body)
     * 
     * @note Includes roll/pitch/yaw trim offsets
     * @note Updated when trim values change
     * @note Used to transform control outputs to vehicle frame
     * 
     * @see get_rotation_vehicle_body_to_autopilot_body() for inverse rotation
     * @see get_trim() for raw trim values
     */
    const Matrix3f& get_rotation_autopilot_body_to_vehicle_body(void) const { return _rotation_autopilot_body_to_vehicle_body; }

    /**
     * @brief Get rotation matrix from vehicle to autopilot body frame
     * 
     * @details Returns the rotation matrix that transforms vectors from the
     *          vehicle body frame to the autopilot body frame (inverse of trim).
     * 
     * @return Rotation matrix (vehicle body → autopilot body)
     * 
     * @note Inverse of autopilot_body_to_vehicle_body rotation
     * @note Updated when trim values change
     * @note Used to transform sensor readings to autopilot frame
     * 
     * @see get_rotation_autopilot_body_to_vehicle_body() for forward rotation
     * @see get_trim() for raw trim values
     */
    const Matrix3f& get_rotation_vehicle_body_to_autopilot_body(void) const { return _rotation_vehicle_body_to_autopilot_body; }

    /*
     * Logging functions
     * 
     * Write AHRS state to onboard dataflash log for post-flight analysis
     */

    /**
     * @brief Write home and origin locations to log
     * 
     * @details Logs both the home location and EKF origin to dataflash for
     *          position reference during post-flight analysis.
     * 
     * @note Called when home set or origin initialized
     * @note Logs ORGN and HOME message types
     * @note Essential for understanding position estimates in logs
     * 
     * @see Write_Origin() for logging origin separately
     * @see set_home() for setting home location
     */
    void Log_Write_Home_And_Origin();

    /**
     * @brief Write attitude and targets to log
     * 
     * @details Logs current attitude (roll, pitch, yaw) along with controller
     *          attitude targets for control loop analysis.
     * 
     * @param[in] targets Target attitude vector in radians (roll, pitch, yaw_rate)
     * 
     * @note Called by vehicle attitude controllers
     * @note Logs ATT message type
     * @note Essential for tuning and control loop analysis
     * 
     * @see Log_Write() for comprehensive AHRS logging
     */
    void Write_Attitude(const Vector3f &targets) const;

    /**
     * @enum LogOriginType
     * @brief Type of origin being logged
     * 
     * @details Distinguishes between EKF origin (navigation reference) and
     *          AHRS home (takeoff location).
     */
    enum class LogOriginType {
        ekf_origin = 0,  ///< EKF navigation origin
        ahrs_home = 1    ///< AHRS home location (takeoff point)
    };

    /**
     * @brief Write origin location to log
     * 
     * @details Logs either the EKF origin or AHRS home location to dataflash
     *          with the specified origin type.
     * 
     * @param[in] origin_type Type of origin (EKF origin or AHRS home)
     * @param[in] loc Location to log (lat/lon/alt)
     * 
     * @note Logs ORGN message type
     * @note Called when origin changes or home is set
     * @note Essential for absolute position reconstruction from logs
     * 
     * @see Log_Write_Home_And_Origin() to log both
     * @see LogOriginType for origin type enumeration
     */
    void Write_Origin(LogOriginType origin_type, const Location &loc) const;

    /**
     * @brief Write video stabilization data to log
     * 
     * @details Logs attitude and rate information suitable for post-flight
     *          video stabilization processing.
     * 
     * @note Logs VSTB message type
     * @note Higher rate than standard attitude logging
     * @note Used by external video stabilization tools
     * 
     * @see Write_Attitude() for standard attitude logging
     */
    void write_video_stabilisation() const;

    /**
     * @brief Get latest smoothed gyro vector
     * 
     * @details Returns a smoothed and corrected gyro vector using the latest
     *          INS data, which may not have been consumed by the EKF yet.
     *          Provides lower latency than get_gyro() for rate control.
     * 
     * @return Gyro vector in rad/s, body frame (roll, pitch, yaw rates)
     * 
     * @note Uses latest INS data, may be ahead of EKF estimates
     * @note Lower latency than get_gyro() for high-rate control loops
     * @note Includes bias correction from gyro calibration
     * 
     * @see get_gyro() for EKF-integrated gyro estimate
     * @see get_gyro_drift() for gyro bias estimate
     */
    Vector3f get_gyro_latest(void) const;

    /**
     * @brief Get yaw rate in earth frame
     * 
     * @details Calculates yaw rate in earth-fixed (NED) frame from body frame
     *          gyro by projecting through DCM rotation matrix.
     * 
     * @return Yaw rate in rad/s in earth frame (positive = clockwise from above)
     * 
     * @note Uses current DCM rotation matrix
     * @note Earth frame yaw rate independent of vehicle roll/pitch
     * @note Called at main loop rate (typically 400Hz copter, 50Hz plane)
     * 
     * @see get_gyro() for body frame gyro vector
     * @see get_rotation_body_to_ned() for rotation matrix
     */
    float get_yaw_rate_earth(void) const {
        return get_gyro() * get_rotation_body_to_ned().c;
    }

    /*
     * Home-related functionality
     * 
     * Home is the vehicle's takeoff/launch location, used as reference point
     * for relative position calculations and return-to-launch navigation.
     */

    /**
     * @brief Get the home location
     * 
     * @details Returns the home location (takeoff point). Returned reference
     *          is const to prevent modification without AHRS notification.
     * 
     * @return Home location (lat/lon in degrees * 1e7, alt in cm)
     * 
     * @note Home typically set at arming or first GPS fix
     * @note Used as reference for RTL (Return To Launch) mode
     * @note Altitude is AMSL (Above Mean Sea Level) from GPS
     * 
     * @warning Check home_is_set() before using, may be uninitialized
     * 
     * @see set_home() to update home location
     * @see home_is_set() to check if home valid
     * @see home_is_locked() to check if home is locked
     */
    const Location &get_home(void) const {
        return _home;
    }

    /**
     * @brief Lock the home location
     * 
     * @details Prevents automatic home updates. Some vehicles allow GCS to
     *          lock in a specific home location, preventing automatic changes.
     * 
     * @note Once locked, set_home() will refuse updates unless unlocked
     * @note Used to manually set home at specific location
     * @note Persists until vehicle reboot or explicit unlock
     * 
     * @see home_is_locked() to check lock status
     * @see set_home() to update home (respects lock)
     */
    void lock_home() {
        _home_locked = true;
    }

    /**
     * @brief Check if home is locked
     * 
     * @details Returns true if home location has been locked, preventing
     *          automatic updates.
     * 
     * @return true if home is locked, false if updates allowed
     * 
     * @note Locked home prevents set_home() from updating
     * @note Used by GCS to maintain specific home location
     * 
     * @see lock_home() to lock home
     * @see set_home() which respects lock status
     */
    bool home_is_locked() const {
        return _home_locked;
    }

    /**
     * @brief Check if home is set
     * 
     * @details Returns true if home location has been initialized.
     * 
     * @return true if home is valid, false if uninitialized
     * 
     * @note Home typically set at arming or first GPS fix
     * @note Must be true before RTL or other home-relative navigation
     * @note Does not indicate GPS fix quality, only that home exists
     * 
     * @see set_home() to initialize home
     * @see get_home() to retrieve home location
     */
    bool home_is_set(void) const {
        return _home_is_set;
    }

    /**
     * @brief Set the home location
     * 
     * @details Sets the home location when the vehicle is at this position.
     *          Current barometer and GPS altitudes assumed to correspond.
     * 
     * @param[in] loc Home location (lat/lon in degrees * 1e7, alt in cm AMSL)
     * 
     * @return true if home set successfully, false if locked or invalid
     * 
     * @note Typically called at arming with current GPS position
     * @note Respects home lock status (fails if locked)
     * @note Altitude reference for relative altitude calculations
     * @note Triggers logging of home location
     * 
     * @warning Requires valid GPS fix for accurate home
     * @warning Setting home mid-flight affects relative altitude
     * 
     * @see get_home() to retrieve current home
     * @see home_is_locked() to check if updates allowed
     * @see lock_home() to prevent automatic updates
     */
    bool set_home(const Location &loc) WARN_IF_UNUSED;

    /*
     * Attitude-related public methods and attributes
     * 
     * Provides vehicle attitude in multiple formats (radians, degrees,
     * centidegrees) and pre-calculated trigonometric values for efficiency.
     */

#if AP_SCRIPTING_ENABLED
    /**
     * @brief Get roll angle (deprecated)
     * @deprecated Use get_roll_rad() instead. Will be removed in future release.
     * @return Roll angle in radians
     */
    float get_roll() const { return roll; }

    /**
     * @brief Get pitch angle (deprecated)
     * @deprecated Use get_pitch_rad() instead. Will be removed in future release.
     * @return Pitch angle in radians
     */
    float get_pitch() const { return pitch; }

    /**
     * @brief Get yaw angle (deprecated)
     * @deprecated Use get_yaw_rad() instead. Will be removed in future release.
     * @return Yaw angle in radians
     */
    float get_yaw() const { return yaw; }
#endif  // AP_SCRIPTING_ENABLED

    /**
     * @brief Get roll angle in radians
     * 
     * @details Returns current roll angle from active AHRS backend (EKF or DCM).
     * 
     * @return Roll angle in radians (right wing down = positive)
     * 
     * @note Updated at main loop rate by update() call
     * @note Right-hand rotation about forward (X) axis
     * @note Range: -π to +π radians
     * 
     * @see get_roll_deg() for degrees
     * @see get_quaternion() for full attitude representation
     */
    float get_roll_rad() const { return roll; }

    /**
     * @brief Get pitch angle in radians
     * 
     * @details Returns current pitch angle from active AHRS backend (EKF or DCM).
     * 
     * @return Pitch angle in radians (nose up = positive)
     * 
     * @note Updated at main loop rate by update() call
     * @note Right-hand rotation about right (Y) axis
     * @note Range: -π/2 to +π/2 radians (singularity at ±90°)
     * 
     * @see get_pitch_deg() for degrees
     * @see get_quaternion() for full attitude representation
     */
    float get_pitch_rad() const { return pitch; }

    /**
     * @brief Get yaw angle in radians
     * 
     * @details Returns current yaw angle from active AHRS backend (EKF or DCM).
     * 
     * @return Yaw angle in radians (0 = North, π/2 = East, positive clockwise)
     * 
     * @note Updated at main loop rate by update() call
     * @note Right-hand rotation about down (Z) axis
     * @note Range: -π to +π radians
     * @note Yaw = 0 when vehicle points North
     * 
     * @see get_yaw_deg() for degrees
     * @see get_quaternion() for full attitude representation
     */
    float get_yaw_rad() const { return yaw; }

    /**
     * @brief Get roll angle in degrees
     * 
     * @details Returns current roll angle in degrees.
     * 
     * @return Roll angle in degrees (right wing down = positive)
     * 
     * @note Updated at main loop rate by update() call
     * @note Range: -180° to +180°
     * 
     * @see get_roll_rad() for radians
     */
    float get_roll_deg() const { return rpy_deg[0]; }

    /**
     * @brief Get pitch angle in degrees
     * 
     * @details Returns current pitch angle in degrees.
     * 
     * @return Pitch angle in degrees (nose up = positive)
     * 
     * @note Updated at main loop rate by update() call
     * @note Range: -90° to +90°
     * 
     * @see get_pitch_rad() for radians
     */
    float get_pitch_deg() const { return rpy_deg[1]; }

    /**
     * @brief Get yaw angle in degrees
     * 
     * @details Returns current yaw angle in degrees.
     * 
     * @return Yaw angle in degrees (0° = North, 90° = East, positive clockwise)
     * 
     * @note Updated at main loop rate by update() call
     * @note Range: -180° to +180°
     * 
     * @see get_yaw_rad() for radians
     */
    float get_yaw_deg() const { return rpy_deg[2]; }

    /**
     * @brief Get cosine of roll angle
     * 
     * @details Returns pre-calculated cos(roll) for computational efficiency.
     * 
     * @return cos(roll)
     * 
     * @note Updated when attitude changes
     * @note Avoids repeated trigonometric calculations
     * @note Used frequently in coordinate transformations
     * 
     * @see sin_roll() for sine of roll
     */
    float cos_roll() const  {
        return _cos_roll;
    }

    /**
     * @brief Get cosine of pitch angle
     * 
     * @details Returns pre-calculated cos(pitch) for computational efficiency.
     * 
     * @return cos(pitch)
     * 
     * @note Updated when attitude changes
     * @note Avoids repeated trigonometric calculations
     * @note Used frequently in coordinate transformations
     * 
     * @see sin_pitch() for sine of pitch
     */
    float cos_pitch() const {
        return _cos_pitch;
    }

    /**
     * @brief Get cosine of yaw angle
     * 
     * @details Returns pre-calculated cos(yaw) for computational efficiency.
     * 
     * @return cos(yaw)
     * 
     * @note Updated when attitude changes
     * @note Avoids repeated trigonometric calculations
     * @note Used frequently in coordinate transformations
     * 
     * @see sin_yaw() for sine of yaw
     */
    float cos_yaw() const   {
        return _cos_yaw;
    }

    /**
     * @brief Get sine of roll angle
     * 
     * @details Returns pre-calculated sin(roll) for computational efficiency.
     * 
     * @return sin(roll)
     * 
     * @note Updated when attitude changes
     * @note Avoids repeated trigonometric calculations
     * @note Used frequently in coordinate transformations
     * 
     * @see cos_roll() for cosine of roll
     */
    float sin_roll() const  {
        return _sin_roll;
    }

    /**
     * @brief Get sine of pitch angle
     * 
     * @details Returns pre-calculated sin(pitch) for computational efficiency.
     * 
     * @return sin(pitch)
     * 
     * @note Updated when attitude changes
     * @note Avoids repeated trigonometric calculations
     * @note Used frequently in coordinate transformations
     * 
     * @see cos_pitch() for cosine of pitch
     */
    float sin_pitch() const {
        return _sin_pitch;
    }

    /**
     * @brief Get sine of yaw angle
     * 
     * @details Returns pre-calculated sin(yaw) for computational efficiency.
     * 
     * @return sin(yaw)
     * 
     * @note Updated when attitude changes
     * @note Avoids repeated trigonometric calculations
     * @note Used frequently in coordinate transformations
     * 
     * @see cos_yaw() for cosine of yaw
     */
    float sin_yaw() const   {
        return _sin_yaw;
    }

    /**
     * @brief Euler angles in degrees (public array)
     * 
     * @details Floating point Euler angles [roll, pitch, yaw] in degrees.
     * 
     * @note Updated by update() at main loop rate
     * @note Direct array access for legacy compatibility
     * @note Prefer get_roll_deg()/get_pitch_deg()/get_yaw_deg() for clarity
     */
    float rpy_deg[3];

    /**
     * @brief Euler angles in centidegrees (public values)
     * 
     * @details Integer Euler angles in centidegrees (degrees * 100) for
     *          compatibility with GCS protocols and legacy code.
     * 
     * @note roll_sensor: Roll in centidegrees
     * @note pitch_sensor: Pitch in centidegrees  
     * @note yaw_sensor: Yaw in centidegrees (0-36000)
     * @note Updated by update() at main loop rate
     * @note Used by MAVLink telemetry and some control loops
     * @note Integer format for efficient transmission/storage
     */
    int32_t roll_sensor;
    int32_t pitch_sensor;
    int32_t yaw_sensor;

    /**
     * @brief Get rotation matrix from body to NED frame
     * 
     * @details Returns the Direction Cosine Matrix (DCM) that rotates vectors
     *          from body frame to North-East-Down (NED) earth-fixed frame.
     * 
     * @return DCM rotation matrix (body → NED)
     * 
     * @note Body frame: X=forward, Y=right, Z=down
     * @note NED frame: X=North, Y=East, Z=Down (earth-fixed)
     * @note Updated by active backend (EKF or DCM) at main loop rate
     * @note DCM matrix orthonormal (transpose = inverse)
     * 
     * @see get_quaternion() for quaternion representation
     * @see body_to_earth() to transform a vector
     */
    const Matrix3f &get_rotation_body_to_ned(void) const { return state.dcm_matrix; }

    /**
     * @brief Get current attitude as quaternion
     * 
     * @details Returns a quaternion representing current attitude in NED frame.
     * 
     * @param[out] quat Quaternion representing rotation from NED to body frame
     * 
     * @note Quaternion convention: rotation from NED to body
     * @note Updated at main loop rate
     * @note Preferred over Euler angles for calculations (no singularities)
     * 
     * @see get_quaternion(Quaternion&) for bool return variant
     * @see get_rotation_body_to_ned() for DCM matrix
     */
    void get_quat_body_to_ned(Quaternion &quat) const;

#if AP_AHRS_DCM_ENABLED
    /**
     * @brief Get DCM backend rotation matrix
     * 
     * @details Returns rotation matrix specifically from DCM backend, regardless
     *          of active backend. Used for compass calibration.
     * 
     * @return DCM rotation matrix from body to NED (DCM backend only)
     * 
     * @note Always returns DCM estimate even if EKF is active
     * @note Used by compass calibrator for sensor calibration
     * @note May differ from primary estimate if EKF is active
     * 
     * @see get_rotation_body_to_ned() for active backend matrix
     */
    const Matrix3f &get_DCM_rotation_body_to_ned(void) const {
        return dcm_estimates.dcm_matrix;
    }
#endif

    /**
     * @brief Rotate 2D vector from earth to body frame
     * 
     * @details Rotates a 2D vector from earth-fixed frame to body frame using
     *          current yaw angle. Used for horizontal navigation.
     * 
     * @param[in] ef_vector Vector in earth frame (North, East)
     * 
     * @return Vector in body frame (forward=X, right=Y)
     * 
     * @note Only rotates in horizontal plane (yaw rotation)
     * @note Efficient 2D rotation for ground speed and wind
     * @note Body frame: X=forward, Y=right
     * @note Earth frame: X=North, Y=East
     * 
     * @see body_to_earth2D() for inverse transformation
     * @see earth_to_body() for full 3D transformation
     */
    Vector2f earth_to_body2D(const Vector2f &ef_vector) const;

    /**
     * @brief Rotate 2D vector from body to earth frame
     * 
     * @details Rotates a 2D vector from body frame to earth-fixed frame using
     *          current yaw angle. Used for horizontal navigation.
     * 
     * @param[in] bf Vector in body frame (forward, right)
     * 
     * @return Vector in earth frame (North=X, East=Y)
     * 
     * @note Only rotates in horizontal plane (yaw rotation)
     * @note Efficient 2D rotation for ground speed and wind
     * @note Body frame: X=forward, Y=right
     * @note Earth frame: X=North, Y=East
     * 
     * @see earth_to_body2D() for inverse transformation
     * @see body_to_earth() for full 3D transformation
     */
    Vector2f body_to_earth2D(const Vector2f &bf) const;

    /**
     * @brief Transform vector from body to earth frame
     * 
     * @details Transforms a 3D vector from body frame to earth-fixed NED frame
     *          using current DCM rotation matrix.
     * 
     * @param[in] v Vector in body frame (forward, right, down)
     * 
     * @return Vector in NED earth frame (North, East, Down)
     * 
     * @note Full 3D rotation using DCM matrix
     * @note Body frame: X=forward, Y=right, Z=down
     * @note NED frame: X=North, Y=East, Z=Down
     * @note Used for transforming accelerations, velocities
     * 
     * @see earth_to_body() for inverse transformation
     * @see get_rotation_body_to_ned() for rotation matrix
     */
    Vector3f body_to_earth(const Vector3f &v) const;

    /**
     * @brief Transform vector from earth to body frame
     * 
     * @details Transforms a 3D vector from earth-fixed NED frame to body frame
     *          using current DCM rotation matrix transpose.
     * 
     * @param[in] v Vector in NED earth frame (North, East, Down)
     * 
     * @return Vector in body frame (forward, right, down)
     * 
     * @note Full 3D rotation using DCM matrix transpose
     * @note Body frame: X=forward, Y=right, Z=down
     * @note NED frame: X=North, Y=East, Z=Down
     * @note Used for transforming wind, waypoint vectors
     * 
     * @see body_to_earth() for inverse transformation
     * @see get_rotation_body_to_ned() for rotation matrix
     */
    Vector3f earth_to_body(const Vector3f &v) const;

    /*
     * Methods for the benefit of LUA bindings
     * 
     * Convenience accessors for scripting and external interfaces
     */

    /**
     * @brief Get current vibration vector for primary IMU
     * 
     * @details Returns vibration levels detected on the primary IMU, useful
     *          for monitoring vehicle mechanical health.
     * 
     * @return Vibration vector in m/s² (X, Y, Z body frame)
     * 
     * @note High vibration affects EKF performance
     * @note Typical good values: <30 m/s²
     * @note Called by LUA scripts and telemetry
     * 
     * @see get_primary_accel_index() for which IMU is primary
     */
    Vector3f get_vibration(void) const;

    /**
     * @brief Get primary accelerometer reading
     * 
     * @details Returns raw acceleration from the primary IMU accelerometer.
     * 
     * @return Acceleration vector in m/s², body frame (forward, right, down)
     * 
     * @note Includes gravity component (1g down when stationary)
     * @note Raw reading, not corrected for bias
     * @note Updated at IMU rate (typically 1kHz)
     * 
     * @see get_accel_bias() for bias correction
     * @see get_accel_ef() for earth-frame acceleration
     * @see get_primary_accel_index() for which IMU is primary
     */
    const Vector3f &get_accel(void) const {
        return AP::ins().get_accel(_get_primary_accel_index());
    }

    /**
     * @brief Get primary accelerometer bias estimate
     * 
     * @details Returns the estimated accelerometer bias that should be
     *          subtracted from get_accel() for best body frame estimate.
     * 
     * @return Accel bias vector in m/s², body frame (forward, right, down)
     * 
     * @note Subtract this from get_accel() for corrected reading
     * @note Estimated by EKF from sensor fusion
     * @note Changes with temperature and calibration
     * 
     * @see get_accel() for raw accelerometer reading
     * @see get_accel_ef() for calibrated earth-frame acceleration
     */
    const Vector3f &get_accel_bias(void) const {
        return state.accel_bias;
    }
    
    /*
     * Vehicle state information
     * 
     * AHRS transports vehicle state flags (takeoff, landing, flight mode)
     * that affect estimation algorithms and ground effect compensation.
     */

    /**
     * @brief Set takeoff expected flag
     * 
     * @details Indicates vehicle is in a state where takeoff might be expected.
     *          Affects ground effect compensation and EKF tuning.
     * 
     * @param[in] b true if takeoff expected, false otherwise
     * 
     * @note Typically set when throttle high and on ground
     * @note Affects EKF barometer/GPS weighting
     * @note Ground effect may be in play during takeoff
     * 
     * @see get_takeoff_expected() to retrieve state
     * @see set_touchdown_expected() for landing state
     */
    void set_takeoff_expected(bool b);

    /**
     * @brief Get takeoff expected flag
     * 
     * @details Returns true if vehicle is in a state where takeoff might occur.
     * 
     * @return true if takeoff expected, false otherwise
     * 
     * @note Used by EKF for ground effect compensation
     * @note Affects sensor weighting during takeoff phase
     * 
     * @see set_takeoff_expected() to update state
     */
    bool get_takeoff_expected(void) const {
        return takeoff_expected;
    }

    /**
     * @brief Set touchdown expected flag
     * 
     * @details Indicates vehicle is in a state where landing might be expected.
     *          Affects ground effect compensation and EKF tuning.
     * 
     * @param[in] b true if touchdown expected, false otherwise
     * 
     * @note Typically set when descending with landing intent
     * @note Affects EKF rangefinder/barometer weighting
     * @note Ground effect may be in play during landing
     * 
     * @see get_touchdown_expected() to retrieve state
     * @see set_takeoff_expected() for takeoff state
     */
    void set_touchdown_expected(bool b);

    /**
     * @brief Get touchdown expected flag
     * 
     * @details Returns true if vehicle is in a state where landing might occur.
     * 
     * @return true if touchdown expected, false otherwise
     * 
     * @note Used by EKF for ground effect compensation
     * @note Affects sensor weighting during landing phase
     * 
     * @see set_touchdown_expected() to update state
     */
    bool get_touchdown_expected(void) const {
        return touchdown_expected;
    }

    /**
     * @brief Set fly-forward mode flag
     * 
     * @details Indicates vehicle should generally be moving in direction of
     *          heading. Provides additional information for improved estimates.
     * 
     * @param[in] b true if flying forward, false for omnidirectional
     * 
     * @note true for fixed-wing, false for copter in most modes
     * @note Affects wind estimation and synthetic airspeed
     * @note Enables forward-flight specific algorithms
     * 
     * @see get_fly_forward() to retrieve state
     * @see set_vehicle_class() for vehicle type
     */
    void set_fly_forward(bool b) {
        fly_forward = b;
    }

    /**
     * @brief Get fly-forward mode flag
     * 
     * @details Returns true if vehicle should be moving forward in heading direction.
     * 
     * @return true if fly-forward mode, false for omnidirectional
     * 
     * @note Used by wind estimation and synthetic airspeed
     * @note Typically true for planes, false for copters
     * 
     * @see set_fly_forward() to update state
     */
    bool get_fly_forward(void) const {
        return fly_forward;
    }

    /**
     * @enum VehicleClass
     * @brief Vehicle type classification
     * 
     * @details Categorizes vehicle type to allow AHRS and other libraries to
     *          adapt behavior appropriately.
     */
    enum class VehicleClass : uint8_t {
        UNKNOWN,      ///< Vehicle class not set
        GROUND,       ///< Ground vehicle (rover, boat)
        COPTER,       ///< Multicopter (quad, hex, octo, etc.)
        FIXED_WING,   ///< Fixed-wing aircraft (plane)
        SUBMARINE,    ///< Underwater vehicle (sub)
    };

    /**
     * @brief Get vehicle class
     * 
     * @details Returns the vehicle type classification.
     * 
     * @return Vehicle class enumeration value
     * 
     * @note Used by AHRS to adapt estimation algorithms
     * @note Affects EKF tuning and sensor selection
     * @note Other libraries query this for vehicle-specific behavior
     * 
     * @see set_vehicle_class() to configure vehicle type
     * @see VehicleClass for enumeration values
     */
    VehicleClass get_vehicle_class(void) const {
        return _vehicle_class;
    }

    /**
     * @brief Set vehicle class
     * 
     * @details Configures the vehicle type classification for appropriate
     *          algorithm selection and tuning.
     * 
     * @param[in] vclass Vehicle class enumeration value
     * 
     * @note Called by vehicle code at initialization
     * @note Affects EKF tuning parameters
     * @note Enables vehicle-specific estimation algorithms
     * 
     * @see get_vehicle_class() to retrieve current class
     * @see VehicleClass for enumeration values
     */
    void set_vehicle_class(VehicleClass vclass) {
        _vehicle_class = vclass;
    }

    /**
     * @brief Get AHRS view object
     * 
     * @details Returns pointer to an AP_AHRS_View object if one has been created,
     *          allowing rotated attitude views for non-standard orientations.
     * 
     * @return Pointer to AP_AHRS_View or nullptr if no view created
     * 
     * @note Views allow accessing attitude relative to rotated frame
     * @note Used for tailsitter transitions and unusual configurations
     * @note Must call create_view() first to create a view
     * 
     * @see create_view() to create a new view
     * @see AP_AHRS_View for rotated attitude interface
     */
    AP_AHRS_View *get_view(void) const { return _view; };

    /**
     * @brief Get EKF GSF yaw estimator
     * 
     * @details Returns pointer to the EKFGSF (Gaussian Sum Filter) yaw estimator
     *          used for emergency yaw recovery.
     * 
     * @return Pointer to EKFGSF_yaw estimator or nullptr if unavailable
     * 
     * @note GSF provides independent yaw estimate for EKF recovery
     * @note Used when compass fails or EKF yaw diverges
     * @note Multiple hypothesis filter for robust yaw estimation
     * 
     * @see request_yaw_reset() to trigger yaw reset from GSF
     */
    const EKFGSF_yaw *get_yaw_estimator(void) const;

private:

    // roll/pitch/yaw euler angles, all in radians
    float roll;
    float pitch;
    float yaw;

    // optional view class
    AP_AHRS_View *_view;

    static AP_AHRS *_singleton;

    /* we modify our behaviour based on what sort of vehicle the
     * vehicle code tells us we are.  This information is also pulled
     * from AP_AHRS by other libraries
     */
    VehicleClass _vehicle_class{VehicleClass::UNKNOWN};

    // multi-thread access support
    HAL_Semaphore _rsem;

    /*
     * Parameters
     */
    AP_Int8 _wind_max;
    AP_Int8 _board_orientation;
    AP_Enum<EKFType> _ekf_type;

    /*
     * DCM-backend parameters; it takes references to these
     */
    // settable parameters
    AP_Float _kp_yaw;
    AP_Float _kp;
    AP_Float gps_gain;

    AP_Float beta;

    AP_Enum<GPSUse> _gps_use;
    AP_Int8 _gps_minsats;

    EKFType active_EKF_type(void) const { return state.active_EKF; }

    bool always_use_EKF() const {
        return _ekf_flags & FLAG_ALWAYS_USE_EKF;
    }

    // check all cores providing consistent attitudes for prearm checks
    bool attitudes_consistent(char *failure_msg, const uint8_t failure_msg_len) const;
    // convenience method for setting error string:
    void set_failure_inconsistent_message(const char *estimator, const char *axis, float diff_rad, char *failure_msg, const uint8_t failure_msg_len) const;

    /*
     * Attitude-related private methods and attributes:
     */
    // calculate sin/cos of roll/pitch/yaw from rotation
    void calc_trig(const Matrix3f &rot,
                   float &cr, float &cp, float &cy,
                   float &sr, float &sp, float &sy) const;

    // update_trig - recalculates _cos_roll, _cos_pitch, etc based on latest attitude
    //      should be called after _dcm_matrix is updated
    void update_trig(void);

    // update roll_sensor, pitch_sensor and yaw_sensor
    void update_cd_values(void);

    // helper trig variables
    float _cos_roll{1.0f};
    float _cos_pitch{1.0f};
    float _cos_yaw{1.0f};
    float _sin_roll;
    float _sin_pitch;
    float _sin_yaw;

#if HAL_NAVEKF2_AVAILABLE
    void update_EKF2(void);
    bool _ekf2_started;
#endif
#if HAL_NAVEKF3_AVAILABLE
    bool _ekf3_started;
    void update_EKF3(void);
#endif

    const uint16_t startup_delay_ms = 1000;
    uint32_t start_time_ms;
    uint8_t _ekf_flags; // bitmask from Flags enumeration

    EKFType ekf_type(void) const;
    void update_DCM();

    /*
     * home-related state
     */
    void load_watchdog_home();
    bool _checked_watchdog_home;
    Location _home;
    bool _home_is_set :1;
    bool _home_locked :1;

    // avoid setting current state repeatedly across all cores on all EKFs:
    enum class TriState {
        False = 0,
        True = 1,
        UNKNOWN = 3,
    };

    TriState terrainHgtStableState = TriState::UNKNOWN;

    /*
     * private AOA and SSA-related state and methods
     */
    float _AOA, _SSA;
    uint32_t _last_AOA_update_ms;
    void update_AOA_SSA(void);

    EKFType last_active_ekf_type;

#if AP_AHRS_SIM_ENABLED
    void update_SITL(void);
#endif

#if AP_AHRS_EXTERNAL_ENABLED
    void update_external(void);
#endif    

    /*
     * trim-related state and private methods:
     */

    // a vector to capture the difference between the controller and body frames
    AP_Vector3f         _trim;

    // cached trim rotations
    Vector3f _last_trim;

    Matrix3f _rotation_autopilot_body_to_vehicle_body;
    Matrix3f _rotation_vehicle_body_to_autopilot_body;

    // last time orientation was updated from AHRS_ORIENTATION:
    uint32_t last_orientation_update_ms;

    // updates matrices responsible for rotating vectors from vehicle body
    // frame to autopilot body frame from _trim variables
    void update_trim_rotation_matrices();

    /*
     * AHRS is used as a transport for vehicle-takeoff-expected and
     * vehicle-landing-expected:
     */
    // update takeoff/touchdown flags
    void update_flags();
    bool takeoff_expected;    // true if the vehicle is in a state that takeoff might be expected.  Ground effect may be in play.
    uint32_t takeoff_expected_start_ms;
    bool touchdown_expected;    // true if the vehicle is in a state that touchdown might be expected.  Ground effect may be in play.
    uint32_t touchdown_expected_start_ms;

    /*
     * wind estimation support
     */
    bool wind_estimation_enabled;

    /*
     * fly_forward is set by the vehicles to indicate the vehicle
     * should generally be moving in the direction of its heading.
     * It is an additional piece of information that the backends can
     * use to provide additional and/or improved estimates.
     */
    bool fly_forward; // true if we can assume the vehicle will be flying forward on its X axis

    // poke AP_Notify based on values from status
    void update_notify_from_filter_status(const nav_filter_status &status);

    /*
     * copy results from a backend over AP_AHRS canonical results.
     * This updates member variables like roll and pitch, as well as
     * updating derived values like sin_roll and sin_pitch.
     */
    void copy_estimates_from_backend_estimates(const AP_AHRS_Backend::Estimates &results);

    // write out secondary estimates:
    void Write_AHRS2(void) const;
    // write POS (canonical vehicle position) message out:
    void Write_POS(void) const;

    // return an airspeed estimate if available. return true
    // if we have an estimate
    bool _airspeed_EAS(float &airspeed_ret, AirspeedEstimateType &status) const;

    // return secondary attitude solution if available, as eulers in radians
    bool _get_secondary_attitude(Vector3f &eulers) const;

    // return secondary attitude solution if available, as quaternion
    bool _get_secondary_quaternion(Quaternion &quat) const;

    // get ground speed 2D
    Vector2f _groundspeed_vector(void);

    // get active EKF type
    EKFType _active_EKF_type(void) const;

    // return a wind estimation vector, in m/s
    bool _wind_estimate(Vector3f &wind) const WARN_IF_UNUSED;

    // return a true airspeed estimate (navigation airspeed) if
    // available. return true if we have an estimate
    bool _airspeed_TAS(float &airspeed_ret) const;

    // return estimate of true airspeed vector in body frame in m/s
    // returns false if estimate is unavailable
    bool _airspeed_TAS(Vector3f &vec) const;

    // return the quaternion defining the rotation from NED to XYZ (body) axes
    bool _get_quaternion(Quaternion &quat) const WARN_IF_UNUSED;

    // return secondary position solution if available
    bool _get_secondary_position(Location &loc) const;

    // return ground speed estimate in meters/second. Used by ground vehicles.
    float _groundspeed(void);

    // Retrieves the corrected NED delta velocity in use by the inertial navigation
    void _getCorrectedDeltaVelocityNED(Vector3f& ret, float& dt) const;

    // returns the inertial navigation origin in lat/lon/alt
    bool _get_origin(Location &ret) const WARN_IF_UNUSED;

    // return origin for a specified EKF type
    bool _get_origin(EKFType type, Location &ret) const;

    // return a ground velocity in meters/second, North/East/Down
    // order. Must only be called if have_inertial_nav() is true
    bool _get_velocity_NED(Vector3f &vec) const WARN_IF_UNUSED;

    // get secondary EKF type.  returns false if no secondary (i.e. only using DCM)
    bool _get_secondary_EKF_type(EKFType &secondary_ekf_type) const;

    // return the index of the primary core or -1 if no primary core selected
    int8_t _get_primary_core_index() const;

    // get the index of the current primary accelerometer sensor
    uint8_t _get_primary_accel_index(void) const;

    // get the index of the current primary gyro sensor
    uint8_t _get_primary_gyro_index(void) const;

    // get the index of the current primary IMU
    uint8_t _get_primary_IMU_index(void) const;

    // get current location estimate
    bool _get_location(Location &loc) const;

    // return true if a airspeed sensor should be used for the AHRS airspeed estimate
    bool _should_use_airspeed_sensor(uint8_t airspeed_index) const;
    
    /*
      update state structure
     */
    void update_state(void);

    // returns an EKF type to be used as active if we decide the
    // primary is not good enough.
    EKFType fallback_active_EKF_type(void) const;

    /*
      state updated at the end of each update() call
     */
    struct {
        EKFType active_EKF;
        uint8_t primary_IMU;
        uint8_t primary_gyro;
        uint8_t primary_accel;
        uint8_t primary_core;
        Vector3f gyro_estimate;
        Matrix3f dcm_matrix;
        Vector3f gyro_drift;
        Vector3f accel_ef;
        Vector3f accel_bias;
        Vector3f wind_estimate;
        bool wind_estimate_ok;
        float EAS2TAS;
        bool airspeed_ok;
        float airspeed;
        AirspeedEstimateType airspeed_estimate_type;
        bool airspeed_true_ok;
        float airspeed_true;
        Vector3f airspeed_vec;
        bool airspeed_vec_ok;
        Quaternion quat;
        bool quat_ok;
        Vector3f secondary_attitude;
        bool secondary_attitude_ok;
        Quaternion secondary_quat;
        bool secondary_quat_ok;
        Location location;
        bool location_ok;
        Location secondary_pos;
        bool secondary_pos_ok;
        Vector2f ground_speed_vec;
        float ground_speed;
        Vector3f corrected_dv;
        float corrected_dv_dt;
        Location origin;
        bool origin_ok;
        Vector3f velocity_NED;
        bool velocity_NED_ok;
    } state;

    /*
     *  backends (and their results)
     */
#if AP_AHRS_DCM_ENABLED
    AP_AHRS_DCM dcm{_kp_yaw, _kp, gps_gain, beta, _gps_use, _gps_minsats};
    struct AP_AHRS_Backend::Estimates dcm_estimates;
#endif
#if AP_AHRS_SIM_ENABLED
#if HAL_NAVEKF3_AVAILABLE
    AP_AHRS_SIM sim{EKF3};
#else
    AP_AHRS_SIM sim;
#endif
    struct AP_AHRS_Backend::Estimates sim_estimates;
#endif

#if AP_AHRS_EXTERNAL_ENABLED
    AP_AHRS_External external;
    struct AP_AHRS_Backend::Estimates external_estimates;
#endif

    enum class Options : uint16_t {
        DISABLE_DCM_FALLBACK_FW=(1U<<0),
        DISABLE_DCM_FALLBACK_VTOL=(1U<<1),
        DISABLE_AIRSPEED_EKF_CHECK=(1U<<2),
    };
    AP_Int16 _options;
    
    bool option_set(Options option) const {
        return (_options & uint16_t(option)) != 0;
    }

    // true when we have completed the common origin setup
    bool done_common_origin;
};

/**
 * @namespace AP
 * @brief ArduPilot singleton accessor namespace
 */
namespace AP {
    /**
     * @brief Get AP_AHRS singleton instance
     * 
     * @details Returns reference to the global AP_AHRS singleton for accessing
     *          attitude, heading, and navigation estimates throughout ArduPilot.
     * 
     * @return Reference to AP_AHRS singleton
     * 
     * @note Preferred accessor method: AP::ahrs()
     * @note Thread-safe access requires locking: WITH_SEMAPHORE(AP::ahrs().get_semaphore())
     * @note Singleton must be initialized before use
     * 
     * Usage:
     * @code
     * // Get current attitude
     * Quaternion quat;
     * AP::ahrs().get_quaternion(quat);
     * 
     * // Thread-safe multi-step access
     * WITH_SEMAPHORE(AP::ahrs().get_semaphore());
     * Vector3f gyro = AP::ahrs().get_gyro();
     * float roll = AP::ahrs().get_roll_rad();
     * @endcode
     * 
     * @see AP_AHRS::get_singleton() for alternative accessor
     * @see AP_AHRS::get_semaphore() for thread safety
     */
    AP_AHRS &ahrs();
};
