/**
 * @file AP_Airspeed.h
 * @brief ArduPilot airspeed sensor subsystem main API
 * 
 * @details This file defines the airspeed sensor frontend for ArduPilot autopilot systems.
 * The airspeed subsystem provides critical velocity measurement for fixed-wing aircraft
 * using differential pressure sensors (pitot tubes) that measure dynamic pressure.
 * 
 * Architecture:
 * - Frontend-backend design supporting multiple simultaneous sensors
 * - Frontend (AP_Airspeed) manages sensor selection, calibration, health monitoring
 * - Backends (AP_Airspeed_Backend subclasses) handle hardware-specific communication
 * - Supports I2C, SPI, analog, CAN, and protocol-based (MSP, NMEA) sensors
 * 
 * Physics:
 * Airspeed calculation uses Bernoulli's equation for incompressible flow:
 *   V = sqrt(2 * ΔP / ρ)
 * where:
 *   V = airspeed (m/s)
 *   ΔP = differential pressure (Pascals) = total pressure - static pressure
 *   ρ = air density (kg/m³), compensated using barometric altitude and temperature
 * 
 * Critical Functions:
 * - Stall prevention: Provides airspeed limits for flight envelope protection
 * - Wind estimation: Combined with GPS ground speed for wind vector calculation
 * - Energy management: Required for TECS (Total Energy Control System) in fixed-wing
 * - Navigation: Improves dead reckoning when GPS degraded
 * 
 * Calibration:
 * - Offset calibration: Removes static pressure bias, performed on ground with zero wind
 * - Ratio calibration: Corrects for installation effects (pitot tube placement, ducting),
 *   can be performed automatically in flight using GPS ground speed comparison
 * 
 * @warning Airspeed accuracy directly affects stall protection and flight safety
 * @warning Incorrect calibration can cause premature or delayed stall warnings
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 * 
 * Source: libraries/AP_Airspeed/AP_Airspeed.h:1-383
 * Source: libraries/AP_Airspeed/AP_Airspeed.cpp (Bernoulli implementation)
 */

#pragma once

#include "AP_Airspeed_config.h"

#if AP_AIRSPEED_ENABLED

#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>

#if AP_AIRSPEED_MSP_ENABLED
#include <AP_MSP/msp.h>
#endif
#if AP_AIRSPEED_EXTERNAL_ENABLED
#include <AP_ExternalAHRS/AP_ExternalAHRS.h>
#endif

class AP_Airspeed_Backend;

/**
 * @class AP_Airspeed_Params
 * @brief Per-instance parameter storage for airspeed sensors
 * 
 * @details This class encapsulates all parameters associated with a single airspeed
 * sensor instance. ArduPilot supports multiple simultaneous airspeed sensors
 * (up to AIRSPEED_MAX_SENSORS), each with independent configuration.
 * 
 * Parameters are persisted to EEPROM/flash via the AP_Param system and survive
 * power cycles. All parameters are prefixed with ARSPD[n]_ where n is the sensor
 * index (e.g., ARSPD_TYPE, ARSPD2_TYPE for sensor 0 and 1).
 * 
 * @note Parameters are loaded during AP_Airspeed::init()
 * @see AP_Param for parameter persistence system
 */
class AP_Airspeed_Params {
public:
    /**
     * @brief Constructor - initializes parameter defaults
     */
    AP_Airspeed_Params(void);

    /**
     * @brief Device bus identifier for parameter association
     * @details Unique ID combining bus type, bus number, and device address.
     * Used to associate parameters with physical sensors across reboots,
     * even if sensor probing order changes.
     * @note Persisted across boots for sensor identification
     */
    AP_Int32 bus_id;
#ifndef HAL_BUILD_AP_PERIPH
    /**
     * @brief Zero-point pressure offset in Pascals
     * @details Differential pressure reading when airspeed is zero (vehicle stationary, no wind).
     * Removes static pressure bias inherent to sensor and installation.
     * Calibrated on ground via AP_Airspeed::calibrate().
     * Typical values: -5 to +5 Pa for well-installed pitot tubes.
     * Units: Pascals
     * @note Updated during offset calibration
     * @warning Must be calibrated with vehicle stationary and zero wind
     */
    AP_Float offset;
    
    /**
     * @brief Airspeed scaling factor (dimensionless)
     * @details Ratio to correct for installation effects on pitot tube sensitivity.
     * Accounts for pitot tube configuration, placement, ducting losses, and airframe effects.
     * Ideal value is 2.0 (from Bernoulli equation), but actual installation requires correction.
     * Can be calibrated automatically in flight using GPS ground speed comparison.
     * Typical values: 1.8 to 2.2
     * Units: dimensionless (1.0 = ideal, >1.0 = pitot undersensing, <1.0 = oversensing)
     * @note Updated via set_airspeed_ratio() or automatic calibration
     */
    AP_Float ratio;
#endif
    
    /**
     * @brief Sensor pressure range in PSI
     * @details Full-scale differential pressure range of the physical sensor.
     * Determines sensor sensitivity and maximum measurable airspeed.
     * Common values: 1 PSI (low speed, <50 m/s), 5 PSI (medium), 10 PSI (high speed)
     * Units: PSI (pounds per square inch)
     */
    AP_Float psi_range;
    
#ifndef HAL_BUILD_AP_PERIPH
    /**
     * @brief Enable sensor usage (0=disabled, 1=enabled, 2=enabled only when armed)
     * @details Controls whether this sensor is used for flight control.
     * 0 = Disabled - sensor ignored even if healthy
     * 1 = Enabled - sensor used if healthy
     * 2 = Enabled when armed - sensor only used after vehicle is armed
     * @note Can be overridden by AP_Airspeed::force_disable_use()
     */
    AP_Int8  use;
    
    /**
     * @brief Analog input pin number (for analog sensors only)
     * @details ADC pin for TYPE_ANALOG sensors. Ignored for digital sensors (I2C/SPI/CAN).
     * Valid range depends on board hardware (typically 0-15).
     * Units: pin number
     */
    AP_Int8  pin;

    /**
     * @enum SkipCalType
     * @brief Calibration skip behavior control
     * @details Controls whether offset calibration is performed during boot sequence.
     * Useful for installations with stable sensors or custom calibration workflows.
     */
    enum class SkipCalType : int8_t {
        /**
         * @brief Do not skip boot calibration (default, safest)
         * @details Performs offset calibration on every boot. Ensures fresh calibration
         * but requires vehicle to be stationary with zero wind during boot.
         */
        None = 0,

        /**
         * @brief Skip boot calibration, use saved offset
         * @details Uses previously saved offset value. No calibration required,
         * but manual calibration can still be triggered if needed.
         * Use when sensor/installation is known to be stable.
         */
        NoCalRequired = 1,

        /**
         * @brief Skip boot calibration, require manual calibration once per boot
         * @details Skips automatic calibration but requires manual trigger via GCS command.
         * Useful when boot conditions are unsuitable for calibration (e.g., windy).
         */
        SkipBootCal = 2,
    };
    
    /**
     * @brief Calibration skip mode parameter
     * @see SkipCalType
     */
    AP_Enum<SkipCalType> skip_cal;

    /**
     * @brief Pitot tube polarity configuration
     * @details Controls which pressure port is considered positive/negative.
     * Standard: positive port = total pressure, negative port = static pressure
     * Reversed: swapped connections
     * Auto: automatically detect correct polarity
     * @see pitot_tube_order enum
     */
    AP_Int8  tube_order;
#endif
    
    /**
     * @brief Sensor backend type selection
     * @details Selects which driver/protocol to use for this sensor instance.
     * 0 = None (disabled)
     * See airspeed_type enum for complete list (MS4525, MS5525, SDP3X, DLVR, etc.)
     * @see airspeed_type enum
     */
    AP_Int8  type;
    
    /**
     * @brief I2C/SPI bus number
     * @details Hardware bus index for digital sensors. Interpretation depends on type:
     * - I2C sensors: I2C bus number (0, 1, 2...)
     * - SPI sensors: SPI bus number
     * - Analog/CAN: ignored
     */
    AP_Int8  bus;
    
#if AP_AIRSPEED_AUTOCAL_ENABLE
    /**
     * @brief Enable automatic in-flight ratio calibration
     * @details 0 = disabled, 1 = enabled
     * When enabled, uses GPS ground speed comparison to continuously refine ratio parameter.
     * Requires steady flight conditions and accurate GPS for reliable calibration.
     * @note Only active during suitable flight conditions (see update_calibration)
     */
    AP_Int8  autocal;
#endif

    /**
     * @brief AP_Param metadata for parameter system
     * @details Defines parameter table structure for automatic parameter handling
     */
    static const struct AP_Param::GroupInfo var_info[];
};


/**
 * @class Airspeed_Calibration
 * @brief Kalman filter for in-flight airspeed ratio estimation
 * 
 * @details Implements a 3-state Extended Kalman Filter to estimate the airspeed ratio
 * parameter by comparing indicated airspeed (from pitot tube) to GPS-derived airspeed.
 * 
 * State vector: [airspeed_scale, wind_x, wind_y]
 * - airspeed_scale: ratio correction factor (what we're estimating)
 * - wind_x: wind velocity component in North direction (m/s)
 * - wind_y: wind velocity component in East direction (m/s)
 * 
 * Algorithm:
 * 1. Prediction step: propagate state and covariance (process noise)
 * 2. Measurement: compare indicated airspeed to |ground_velocity - wind_estimate|
 * 3. Innovation: compute measurement residual
 * 4. Update: correct state estimate using Kalman gain
 * 
 * The filter estimates both ratio and wind simultaneously, allowing calibration
 * even in windy conditions. Requires accurate GPS and relatively steady flight.
 * 
 * @note Calibration converges over several minutes of flight
 * @warning Requires accurate GPS (good HDOP) for reliable estimation
 * @see AP_Airspeed::update_calibration() for usage
 */
class Airspeed_Calibration {
public:
    friend class AP_Airspeed;
    
    /**
     * @brief Constructor - initializes filter parameters
     * @details Sets process noise constants Q0 and Q1
     */
    Airspeed_Calibration();

    /**
     * @brief Initialize Kalman filter with starting ratio
     * @param initial_ratio Starting airspeed scale factor (typically 1.0-2.0, dimensionless)
     * @details Resets state vector and covariance matrix. Call before beginning calibration.
     */
    void init(float initial_ratio);

    /**
     * @brief Kalman filter update step
     * @param airspeed Indicated airspeed from sensor (m/s)
     * @param vg GPS ground velocity vector in NED frame (m/s)
     * @param max_airspeed_allowed_during_cal Maximum valid airspeed threshold (m/s)
     * @return Updated airspeed scale ratio (dimensionless)
     * 
     * @details Performs one iteration of the Extended Kalman Filter:
     * - Compares indicated airspeed to GPS-derived airspeed magnitude
     * - Updates state estimate for ratio and wind components
     * - Returns refined ratio value
     * 
     * Measurement equation: indicated_airspeed * ratio ≈ |ground_velocity - wind|
     * 
     * @note Call repeatedly during steady flight for convergence
     * @note Rejects updates if airspeed exceeds max_airspeed_allowed_during_cal
     * @warning Requires good GPS (HDOP < 1.0) for accurate results
     */
    float update(float airspeed, const Vector3f &vg, int16_t max_airspeed_allowed_during_cal);

private:
    /**
     * @brief 3x3 covariance matrix tracking estimation uncertainty
     * @details Diagonal elements represent variance in [scale, wind_x, wind_y]
     * Off-diagonal elements represent state correlations
     * Units: varies by state (scale: dimensionless², wind: m²/s²)
     */
    Matrix3f P;
    
    /**
     * @brief Process noise constant for scale and wind_x states
     * @details Tuning parameter controlling filter responsiveness vs smoothness
     * Higher Q0 = faster convergence but noisier estimate
     */
    const float Q0;
    
    /**
     * @brief Process noise constant for wind_y state
     * @details Separate tuning for wind_y component
     */
    const float Q1;
    
    /**
     * @brief State vector: [airspeed_scale, wind_x, wind_y]
     * @details
     * - state.x = airspeed ratio (dimensionless, typically 1.0-2.0)
     * - state.y = wind velocity in North direction (m/s, NED frame)
     * - state.z = wind velocity in East direction (m/s, NED frame)
     */
    Vector3f state;
};

/**
 * @class AP_Airspeed
 * @brief Airspeed subsystem manager (frontend) for ArduPilot autopilot
 * 
 * @details This is the main airspeed sensor frontend class that manages multiple
 * airspeed sensor backends, handles sensor selection, calibration, health monitoring,
 * and provides a unified airspeed measurement interface to the vehicle code.
 * 
 * Architecture:
 * - Frontend-backend pattern: AP_Airspeed (frontend) manages AP_Airspeed_Backend instances
 * - Supports up to AIRSPEED_MAX_SENSORS simultaneous sensors
 * - Automatic primary sensor selection based on health and configuration
 * - Per-sensor state tracking: calibration, health, failure detection
 * 
 * Responsibilities:
 * - Parameter management: Load/save sensor parameters via AP_Param
 * - Backend management: Probe, initialize, and poll sensor backends
 * - Calibration: Coordinate offset and ratio calibration (ground and in-flight)
 * - Sensor selection: Choose primary sensor from multiple available sensors
 * - Health monitoring: Track sensor health, detect failures, trigger failsafes
 * - Data fusion: Apply filtering, offset/ratio corrections, air density compensation
 * - Logging: Record airspeed data to binary logs
 * - Protocol handlers: Process external airspeed data (MSP, External AHRS)
 * 
 * Airspeed Calculation:
 * 1. Read raw differential pressure from backend (Pascals)
 * 2. Apply offset correction: corrected_pressure = raw_pressure - offset
 * 3. Apply ratio and air density: airspeed = sqrt(2 * corrected_pressure * ratio / air_density)
 * 4. Apply low-pass filtering for noise reduction
 * 5. Validate against wind limits and EKF consistency checks
 * 
 * Singleton Access:
 * Access via AP::airspeed() or AP_Airspeed::get_singleton()
 * 
 * Typical Usage:
 * @code
 * AP_Airspeed *airspeed = AP::airspeed();
 * if (airspeed && airspeed->enabled() && airspeed->healthy()) {
 *     float speed_ms = airspeed->get_airspeed(); // Returns primary sensor airspeed in m/s
 * }
 * @endcode
 * 
 * @note Called at main loop rate (typically 50-400Hz via update())
 * @warning Critical for fixed-wing stall prevention and energy management
 * @see AP_Airspeed_Backend for backend implementation
 * @see AP_Airspeed_Params for per-sensor parameters
 * 
 * Source: libraries/AP_Airspeed/AP_Airspeed.h:80-376
 * Source: libraries/AP_Airspeed/AP_Airspeed.cpp (implementation)
 */
class AP_Airspeed
{
public:
    friend class AP_Airspeed_Backend;
    
    /**
     * @brief Constructor - initializes singleton instance
     * @details Sets up default values, registers parameter table, initializes state
     */
    AP_Airspeed();

    /**
     * @brief Link to fixed-wing parameter structure
     * @param _fixed_wing_parameters Pointer to AP_FixedWing parameter structure
     * @details Provides access to vehicle-specific parameters (e.g., stall speed limits)
     * used for airspeed validation and health checks
     */
    void set_fixedwing_parameters(const class AP_FixedWing *_fixed_wing_parameters);

    /**
     * @brief Initialize airspeed subsystem - probe and configure backends
     * @details Called during vehicle setup phase:
     * 1. Calls allocate() to create backend instances based on TYPE parameters
     * 2. Initializes each backend (I2C/SPI device setup, sensor configuration)
     * 3. Sets up logging
     * 4. Prepares calibration state
     * @note Call once during vehicle initialization, before first update()
     */
    void init(void);
    
    /**
     * @brief Create backend instances based on ARSPD_TYPE parameters
     * @details Iterates through configured sensor slots, creates appropriate backend
     * for each TYPE setting (MS4525, MS5525, SDP3X, etc.), and registers backends
     * with the frontend.
     * @note Called internally by init()
     */
    void allocate();

    /**
     * @brief Configure logging enable bit
     * @param log_bit Bit position in LOG_BITMASK parameter
     * @details Sets which bit in the vehicle's LOG_BITMASK parameter controls
     * airspeed logging. When the corresponding bit is set, airspeed data is
     * written to binary logs.
     */
    void set_log_bit(uint32_t log_bit) { _log_bit = log_bit; }

#if AP_AIRSPEED_AUTOCAL_ENABLE
    /**
     * @brief Enable or disable in-flight ratio calibration
     * @param enable true to enable automatic ratio calibration, false to disable
     * @details Controls whether the Kalman filter-based ratio calibration runs
     * during flight. When enabled, continuously refines ratio parameter using
     * GPS ground speed comparison.
     * @note Requires ARSPD_AUTOCAL parameter to be set per-sensor
     */
    void set_calibration_enabled(bool enable) {calibration_enabled = enable;}
#endif //AP_AIRSPEED_AUTOCAL_ENABLE

    /**
     * @brief Read sensors and update airspeed state - main update loop
     * @details Called at main loop rate (typically 50-400Hz). Performs:
     * 1. Poll all enabled backends for new pressure readings
     * 2. Apply offset and ratio corrections
     * 3. Convert pressure to airspeed using Bernoulli equation with air density compensation
     * 4. Apply low-pass filtering
     * 5. Update health status (freshness check, wind limit check, EKF consistency)
     * 6. Select primary sensor
     * 7. Log data (if logging enabled)
     * 8. Check for sensor failures
     * 
     * @note Must be called regularly to maintain sensor health status
     * @see calibrate() for offset calibration
     * @see update_calibration() for ratio calibration
     */
    void update(void);

    /**
     * @brief Perform offset calibration (zero-point calibration)
     * @param in_startup true if called during boot sequence, false for manual calibration
     * 
     * @details Calibrates the pressure offset by averaging sensor readings over ~5 seconds.
     * Removes static pressure bias to ensure zero airspeed reading when stationary.
     * 
     * Calibration process:
     * 1. Accumulates pressure readings (typically 50-500 samples)
     * 2. Computes average pressure
     * 3. Stores as offset parameter
     * 4. Updates calibration state
     * 
     * @warning Vehicle MUST be stationary with ZERO WIND during calibration
     * @warning Incorrect offset causes constant airspeed bias affecting stall protection
     * @note Can be skipped via ARSPD_SKIP_CAL parameter for stable installations
     * @note Manual calibration can be triggered via GCS command
     */
    void calibrate(bool in_startup);

    /**
     * @brief Get filtered airspeed from specified sensor
     * @param i Sensor index (0 to AIRSPEED_MAX_SENSORS-1)
     * @return Airspeed in meters per second (m/s)
     * 
     * @details Returns low-pass filtered airspeed computed using Bernoulli equation:
     *   airspeed = sqrt(2 * corrected_pressure * ratio / air_density)
     * where:
     *   corrected_pressure = raw_pressure - offset (Pascals)
     *   ratio = scaling factor from ARSPD_RATIO parameter (dimensionless)
     *   air_density = computed from barometric altitude and temperature (kg/m³)
     * 
     * Filtering reduces sensor noise and provides stable readings for flight control.
     * 
     * @note Returns 0.0 if sensor is disabled or unhealthy
     * Units: meters per second (m/s)
     */
    float get_airspeed(uint8_t i) const;
    
    /**
     * @brief Get filtered airspeed from primary sensor
     * @return Airspeed in m/s
     * @see get_airspeed(uint8_t i)
     */
    float get_airspeed(void) const { return get_airspeed(primary); }

    /**
     * @brief Get unfiltered (raw) airspeed from specified sensor
     * @param i Sensor index (0 to AIRSPEED_MAX_SENSORS-1)
     * @return Unfiltered airspeed in m/s
     * 
     * @details Returns instantaneous airspeed without low-pass filtering.
     * Useful for diagnostics, testing, and high-rate applications requiring
     * undelayed measurements. More noisy than get_airspeed().
     * 
     * @note Bypass low-pass filter for diagnostic purposes
     * Units: meters per second (m/s)
     */
    float get_raw_airspeed(uint8_t i) const;
    
    /**
     * @brief Get unfiltered airspeed from primary sensor
     * @return Unfiltered airspeed in m/s
     * @see get_raw_airspeed(uint8_t i)
     */
    float get_raw_airspeed(void) const { return get_raw_airspeed(primary); }

    /**
     * @brief Get airspeed ratio scaling factor for specified sensor
     * @param i Sensor index (0 to AIRSPEED_MAX_SENSORS-1)
     * @return Ratio (dimensionless)
     * 
     * @details Returns current ARSPD_RATIO parameter value. This factor corrects
     * for pitot tube installation effects (placement, ducting, airframe interference).
     * 
     * Typical values:
     * - 1.0 = ideal installation (rare)
     * - 2.0 = default starting value
     * - 1.8-2.2 = common range after calibration
     * - >1.0 = pitot tube undersensing (most common due to losses)
     * - <1.0 = pitot tube oversensing (uncommon, check installation)
     * 
     * @note Updated via set_airspeed_ratio() or automatic calibration
     * Units: dimensionless
     */
    float get_airspeed_ratio(uint8_t i) const {
#ifndef HAL_BUILD_AP_PERIPH
        return param[i].ratio;
#else
        return 0.0;
#endif
    }
    
    /**
     * @brief Get ratio for primary sensor
     * @return Ratio (dimensionless)
     * @see get_airspeed_ratio(uint8_t i)
     */
    float get_airspeed_ratio(void) const { return get_airspeed_ratio(primary); }

    /**
     * @brief Get sensor temperature if available
     * @param i Sensor index (0 to AIRSPEED_MAX_SENSORS-1)
     * @param[out] temperature Temperature in degrees Celsius
     * @return true if temperature reading available, false otherwise
     * 
     * @details Some airspeed sensors include temperature measurement.
     * Temperature can be used for additional air density compensation
     * and sensor diagnostics.
     * 
     * Units: degrees Celsius (°C)
     */
    bool get_temperature(uint8_t i, float &temperature);
    
    /**
     * @brief Get temperature from primary sensor
     * @param[out] temperature Temperature in °C
     * @return true if available
     */
    bool get_temperature(float &temperature) { return get_temperature(primary, temperature); }

#ifndef HAL_BUILD_AP_PERIPH
    /**
     * @brief Set airspeed ratio parameter for specified sensor
     * @param i Sensor index (0 to AIRSPEED_MAX_SENSORS-1)
     * @param ratio New ratio value (dimensionless, typically 1.8-2.2)
     * 
     * @details Updates ARSPD_RATIO parameter and persists to EEPROM.
     * Use after manual calibration or to apply known correction factor.
     * 
     * @note Changes take effect immediately on next update()
     * @warning Incorrect ratio affects airspeed accuracy and stall protection
     */
    void set_airspeed_ratio(uint8_t i, float ratio) {
        param[i].ratio.set(ratio);
    }
    
    /**
     * @brief Set ratio for primary sensor
     * @param ratio New ratio value
     */
    void set_airspeed_ratio(float ratio) { set_airspeed_ratio(primary, ratio); }
#endif

    /**
     * @brief Check if sensor is enabled and USE parameter is set
     * @param i Sensor index (0 to AIRSPEED_MAX_SENSORS-1)
     * @return true if sensor backend exists, healthy, and USE parameter allows usage
     * 
     * @details Returns true only if ALL conditions met:
     * 1. Sensor backend allocated (TYPE parameter configured)
     * 2. USE parameter is non-zero
     * 3. Not overridden by force_disable_use()
     * 4. Sensor is healthy
     * 
     * @note This is the key check before using airspeed for flight control
     */
    bool use(uint8_t i) const;
    
    /**
     * @brief Check if primary sensor is in use
     * @return true if primary sensor usable
     */
    bool use(void) const { return use(primary); }

    /**
     * @brief Force disable all airspeed sensors (emergency override)
     * @param value true to force disable all sensors, false to restore normal behavior
     * 
     * @details Overrides all USE parameters, immediately disabling airspeed usage.
     * Used for:
     * - Emergency failsafe when all sensors fail
     * - Testing vehicle behavior without airspeed
     * - Manual pilot override via GCS command
     * 
     * @warning Disabling airspeed removes stall protection on fixed-wing aircraft
     * @warning Vehicle must use synthetic airspeed (from throttle/attitude) after disabling
     */
    void force_disable_use(bool value) {
        _force_disable_use = value;
    }

    /**
     * @brief Check if sensor backend is configured/allocated
     * @param i Sensor index (0 to AIRSPEED_MAX_SENSORS-1)
     * @return true if backend instance exists (TYPE parameter is not None)
     * 
     * @details Returns true if sensor slot has been allocated with a backend driver.
     * Does NOT check health or USE status - only checks if backend exists.
     * 
     * @note Different from use() - enabled() only checks backend existence
     */
    bool enabled(uint8_t i) const;
    
    /**
     * @brief Check if primary sensor backend exists
     * @return true if primary backend allocated
     */
    bool enabled(void) const { return enabled(primary); }

    /**
     * @brief Get raw differential pressure from specified sensor
     * @param i Sensor index (0 to AIRSPEED_MAX_SENSORS-1)
     * @return Differential pressure in Pascals (Pa)
     * 
     * @details Returns most recent raw differential pressure reading from sensor.
     * This is total pressure minus static pressure, before any corrections.
     * 
     * Useful for:
     * - Sensor diagnostics
     * - Manual airspeed calculations
     * - Pressure-based algorithms
     * 
     * Units: Pascals (Pa)
     * @note No offset correction applied - use get_corrected_pressure() for corrected value
     */
    float get_differential_pressure(uint8_t i) const;
    
    /**
     * @brief Get raw differential pressure from primary sensor
     * @return Pressure in Pascals
     */
    float get_differential_pressure(void) const { return get_differential_pressure(primary); }

    /**
     * @brief Update in-flight ratio calibration for all sensors
     * @param vground GPS ground velocity vector in NED frame (m/s)
     * @param max_airspeed_allowed_during_cal Maximum valid airspeed limit (m/s)
     * 
     * @details Iterates through all enabled sensors with AUTOCAL=1 and updates
     * their ratio calibration Kalman filters using GPS ground speed comparison.
     * 
     * Calibration requirements:
     * - Steady flight (minimal maneuvering)
     * - Good GPS (HDOP < 1.0)
     * - Airspeed within valid range (< max_airspeed_allowed_during_cal)
     * - Sufficient air velocity (> min threshold)
     * 
     * @note Called automatically by vehicle code during suitable flight conditions
     * @note Typically converges over 5-10 minutes of flight
     * @see Airspeed_Calibration::update() for filter details
     */
    void update_calibration(const Vector3f &vground, int16_t max_airspeed_allowed_during_cal);

    /**
     * @brief Check sensor health status
     * @param i Sensor index (0 to AIRSPEED_MAX_SENSORS-1)
     * @return true if sensor is healthy and providing valid data
     * 
     * @details Sensor considered healthy if ALL conditions met:
     * 1. Fresh data (updated within timeout period, typically 500ms)
     * 2. Passing wind limit check (if enabled via OPTIONS)
     * 3. Passing EKF consistency check (if enabled via OPTIONS)
     * 4. Not flagged as failed by failure detection
     * 
     * Wind limit check:
     * - Compares indicated airspeed to EKF wind estimate
     * - Fails if implied wind exceeds ARSPD_WIND_MAX parameter
     * - Detects sensor failures and unrealistic readings
     * 
     * EKF consistency check:
     * - Compares sensor airspeed to EKF-estimated airspeed
     * - Fails if discrepancy exceeds threshold
     * - Uses innovation test with configurable gate (ARSPD_WIND_GATE)
     * 
     * @note Health status updated every update() call
     * @warning Unhealthy sensor should not be used for flight control
     */
    bool healthy(uint8_t i) const;
    
    /**
     * @brief Check primary sensor health
     * @return true if primary sensor healthy
     */
    bool healthy(void) const { return healthy(primary); }

    /**
     * @brief Check if all enabled sensors are healthy
     * @return true if ALL sensors with USE=1 are healthy
     * 
     * @details Useful for pre-arm checks and flight mode restrictions.
     * Returns true only if every enabled sensor passes health checks.
     * Returns true if no sensors are enabled (no airspeed requirement).
     * 
     * @note Stricter than healthy() - requires ALL enabled sensors pass
     */
    bool all_healthy(void) const;
    
    /**
     * @brief Get timestamp of last sensor update
     * @param i Sensor index (0 to AIRSPEED_MAX_SENSORS-1)
     * @return Time in milliseconds since boot (from AP_HAL::millis())
     * 
     * @details Returns timestamp of most recent successful sensor reading.
     * Used for freshness checking and timeout detection.
     * 
     * Units: milliseconds since system boot
     */
    uint32_t last_update_ms(uint8_t i) const { return state[i].last_update_ms; }
    
    /**
     * @brief Get timestamp for primary sensor
     * @return Milliseconds since boot
     */
    uint32_t last_update_ms(void) const { return last_update_ms(primary); }

#if AP_AIRSPEED_HYGROMETER_ENABLE
    /**
     * @brief Get hygrometer (humidity sensor) data if available
     * @param i Sensor index (0 to AIRSPEED_MAX_SENSORS-1)
     * @param[out] last_sample_ms Timestamp of measurement (ms since boot)
     * @param[out] temperature Temperature in degrees Celsius
     * @param[out] humidity Relative humidity in percent (0-100)
     * @return true if hygrometer data available, false otherwise
     * 
     * @details Some advanced airspeed sensors include humidity measurement.
     * Humidity data can be used for air density correction and weather monitoring.
     * 
     * Units:
     * - temperature: degrees Celsius (°C)
     * - humidity: percent (0-100%)
     * - last_sample_ms: milliseconds since boot
     */
    bool get_hygrometer(uint8_t i, uint32_t &last_sample_ms, float &temperature, float &humidity) const;
#endif

    /**
     * @brief AP_Param metadata for library-level parameters
     * @details Defines parameter table for ARSPD_* parameters (not per-sensor)
     */
    static const struct AP_Param::GroupInfo var_info[];

    /**
     * @enum pitot_tube_order
     * @brief Pitot tube pressure port polarity configuration
     * @details Specifies which pressure port on differential sensor is positive (total pressure)
     * and which is negative (static pressure). Standard installation has total pressure on
     * positive port. If tubes are reversed, select NEGATIVE or use AUTO detection.
     */
    enum pitot_tube_order {
        /**
         * @brief Standard polarity (default)
         * @details Positive port = total pressure (forward-facing pitot opening)
         *          Negative port = static pressure (side-facing static ports)
         */
        PITOT_TUBE_ORDER_POSITIVE = 0,
        
        /**
         * @brief Reversed polarity
         * @details Pressure tubes connected backwards.
         *          Positive port = static pressure
         *          Negative port = total pressure
         */
        PITOT_TUBE_ORDER_NEGATIVE = 1,
        
        /**
         * @brief Automatic polarity detection
         * @details System detects correct polarity based on pressure sign.
         *          If pressure is consistently negative, automatically reverses.
         */
        PITOT_TUBE_ORDER_AUTO     = 2
    };

    /**
     * @enum OptionsMask
     * @brief Bitmask options for airspeed library configuration (ARSPD_OPTIONS parameter)
     * @details Controls various airspeed subsystem behaviors via bitmask flags
     */
    enum OptionsMask {
        /**
         * @brief Enable wind-based failure detection
         * @details If set, compares indicated airspeed to EKF wind estimate.
         * Disables sensor if implied wind exceeds ARSPD_WIND_MAX threshold.
         * Detects sensor failures and unrealistic readings.
         */
        ON_FAILURE_AHRS_WIND_MAX_DO_DISABLE                   = (1<<0),
        
        /**
         * @brief Enable automatic sensor recovery after failure
         * @details If set, automatically re-enables sensor (sets USE=1) when
         * health checks pass again after previous failure. Allows recovery
         * from transient failures without manual intervention.
         */
        ON_FAILURE_AHRS_WIND_MAX_RECOVERY_DO_REENABLE         = (1<<1),
        
        /**
         * @brief Disable voltage-based airspeed correction
         * @details If set, disables compensation for analog sensor voltage variations.
         * Use when voltage correction causes issues with specific sensor types.
         */
        DISABLE_VOLTAGE_CORRECTION                            = (1<<2),
        
        /**
         * @brief Enable EKF consistency check
         * @details If set, compares sensor airspeed to EKF-estimated airspeed using
         * innovation test. Fails sensor if discrepancy exceeds ARSPD_WIND_GATE threshold.
         * Provides additional validation beyond wind limit check.
         * @warning Critical safety check for detecting sensor failures
         */
        USE_EKF_CONSISTENCY                                   = (1<<3),
        
        /**
         * @brief Report offset calibration to Ground Control Station
         * @details If set, sends offset calibration progress and results via MAVLink.
         * Useful for monitoring calibration status in ground station.
         */
        REPORT_OFFSET                                         = (1<<4),
    };

    /**
     * @enum airspeed_type
     * @brief Airspeed sensor backend types (ARSPD_TYPE parameter values)
     * @details Selects which hardware driver/protocol to use for airspeed sensor.
     * Each type corresponds to a specific sensor chip or communication protocol.
     * Availability depends on build-time feature flags (AP_AIRSPEED_*_ENABLED).
     */
    enum airspeed_type {
        /** @brief No sensor (disabled) */
        TYPE_NONE=0,
        
#if AP_AIRSPEED_MS4525_ENABLED
        /** @brief TE Connectivity MS4525DO I2C differential pressure sensor (common, ±1 PSI) */
        TYPE_I2C_MS4525=1,
#endif
        
#if AP_AIRSPEED_ANALOG_ENABLED
        /** @brief Analog voltage input airspeed sensor (MPXV7002DP or similar) */
        TYPE_ANALOG=2,
#endif
        
#if AP_AIRSPEED_MS5525_ENABLED
        /** @brief TE Connectivity MS5525DSO I2C sensor (higher accuracy, multiple addresses) */
        TYPE_I2C_MS5525=3,
        /** @brief MS5525 at alternate I2C address 1 */
        TYPE_I2C_MS5525_ADDRESS_1=4,
        /** @brief MS5525 at alternate I2C address 2 */
        TYPE_I2C_MS5525_ADDRESS_2=5,
#endif
        
#if AP_AIRSPEED_SDP3X_ENABLED
        /** @brief Sensirion SDP3x I2C differential pressure sensor (high accuracy, ±500 Pa) */
        TYPE_I2C_SDP3X=6,
#endif
        
#if AP_AIRSPEED_DLVR_ENABLED
        /** @brief AllSensors DLVR I2C sensor, 5 inch H2O range (~±12 mbar) */
        TYPE_I2C_DLVR_5IN=7,
#endif
        
#if AP_AIRSPEED_DRONECAN_ENABLED
        /** @brief DroneCAN/UAVCAN airspeed sensor (network-based, CAN bus) */
        TYPE_UAVCAN=8,
#endif
        
#if AP_AIRSPEED_DLVR_ENABLED
        /** @brief AllSensors DLVR, 10 inch H2O range (~±25 mbar) */
        TYPE_I2C_DLVR_10IN=9,
        /** @brief AllSensors DLVR, 20 inch H2O range (~±50 mbar) */
        TYPE_I2C_DLVR_20IN=10,
        /** @brief AllSensors DLVR, 30 inch H2O range (~±75 mbar) */
        TYPE_I2C_DLVR_30IN=11,
        /** @brief AllSensors DLVR, 60 inch H2O range (~±150 mbar, high-speed aircraft) */
        TYPE_I2C_DLVR_60IN=12,
#endif
        
#if AP_AIRSPEED_NMEA_ENABLED
        /** @brief NMEA water speed sensor (for water vehicles, RS232/UART) */
        TYPE_NMEA_WATER=13,
#endif
        
#if AP_AIRSPEED_MSP_ENABLED
        /** @brief MSP (MultiWii Serial Protocol) airspeed data */
        TYPE_MSP=14,
#endif
        
#if AP_AIRSPEED_ASP5033_ENABLED
        /** @brief TE Connectivity ASP5033 I2C pressure sensor */
        TYPE_I2C_ASP5033=15,
#endif
        
#if AP_AIRSPEED_EXTERNAL_ENABLED
        /** @brief External AHRS providing airspeed (VectorNav, LORD, etc.) */
        TYPE_EXTERNAL=16,
#endif
        
#if AP_AIRSPEED_AUAV_ENABLED
        /** @brief AUAV differential pressure sensor, 10 inch H2O range */
        TYPE_AUAV_10IN=17,
        /** @brief AUAV differential pressure sensor, 5 inch H2O range */
        TYPE_AUAV_5IN=18,
        /** @brief AUAV differential pressure sensor, 30 inch H2O range */
        TYPE_AUAV_30IN=19,
#endif
        
#if AP_AIRSPEED_SITL_ENABLED
        /** @brief Software-in-the-loop simulation airspeed (testing only) */
        TYPE_SITL=100,
#endif
    };

    /**
     * @brief Get current primary sensor index
     * @return Index of primary sensor (0 to AIRSPEED_MAX_SENSORS-1)
     * 
     * @details Returns index of sensor currently selected as primary for flight control.
     * Primary selection based on:
     * 1. ARSPD_PRIMARY parameter (if set to specific sensor)
     * 2. First healthy enabled sensor (if ARSPD_PRIMARY=-1 for automatic)
     * 3. Falls back through enabled sensors if primary fails
     * 
     * @note Primary can change dynamically if current primary becomes unhealthy
     */
    uint8_t get_primary(void) const { return primary; }

    /**
     * @brief Get total number of configured sensors
     * @return Count of allocated backend instances (0 to AIRSPEED_MAX_SENSORS)
     * 
     * @details Returns number of sensor slots with TYPE parameter configured (not TYPE_NONE).
     * Does not indicate how many are currently healthy.
     */
    uint8_t get_num_sensors(void) const { return num_sensors; }
    
    /**
     * @brief Get singleton instance pointer
     * @return Pointer to AP_Airspeed singleton, or nullptr if not instantiated
     * @note Prefer using AP::airspeed() for safer access
     */
    static AP_Airspeed *get_singleton() { return _singleton; }

    /**
     * @brief Get offset-corrected differential pressure
     * @param i Sensor index (0 to AIRSPEED_MAX_SENSORS-1)
     * @return Corrected pressure in Pascals
     * 
     * @details Returns differential pressure with offset calibration applied:
     *   corrected_pressure = raw_pressure - offset
     * 
     * This is the pressure value used in Bernoulli equation for airspeed calculation.
     * Public for AP_Periph (CAN peripheral) usage.
     * 
     * Units: Pascals (Pa)
     */
    float get_corrected_pressure(uint8_t i) const;
    
    /**
     * @brief Get corrected pressure from primary sensor
     * @return Pressure in Pascals
     */
    float get_corrected_pressure(void) const {
        return get_corrected_pressure(primary);
    }

#if AP_AIRSPEED_MSP_ENABLED
    /**
     * @brief Process MSP (MultiWii Serial Protocol) airspeed message
     * @param pkt MSP airspeed data packet
     * @details Handles airspeed data received via MSP protocol from external source
     * (e.g., OSD, external flight controller). Updates sensor with TYPE_MSP.
     */
    void handle_msp(const MSP::msp_airspeed_data_message_t &pkt);
#endif

#if AP_AIRSPEED_EXTERNAL_ENABLED
    /**
     * @brief Process external AHRS airspeed message
     * @param pkt External AHRS airspeed data packet
     * @details Handles airspeed data from external AHRS systems (VectorNav, LORD MicroStrain, etc.)
     * Updates sensor with TYPE_EXTERNAL.
     */
    void handle_external(const AP_ExternalAHRS::airspeed_data_message_t &pkt);
#endif

    /**
     * @enum CalibrationState
     * @brief Offset calibration state machine status
     * @details Tracks progress of ground-based offset calibration for each sensor
     */
    enum class CalibrationState {
        /** @brief Calibration not yet started or pending */
        NOT_STARTED,
        
        /** @brief Calibration not required - zero offset detected, sensor stable */
        NOT_REQUIRED_ZERO_OFFSET,
        
        /** @brief Calibration in progress - accumulating samples */
        IN_PROGRESS,
        
        /** @brief Calibration completed successfully - offset saved */
        SUCCESS,
        
        /** @brief Calibration failed - insufficient samples, excessive variation, or timeout */
        FAILED
    };

    /**
     * @brief Get aggregate calibration state for all sensors
     * @return Overall calibration status across all enabled sensors
     * 
     * @details Returns:
     * - SUCCESS if all enabled sensors successfully calibrated
     * - IN_PROGRESS if any sensor still calibrating
     * - FAILED if any sensor failed calibration
     * - NOT_STARTED if no calibration initiated
     * - NOT_REQUIRED_ZERO_OFFSET if all sensors have zero offset (no cal needed)
     * 
     * @note Used for pre-arm checks and GCS status reporting
     */
    CalibrationState get_calibration_state() const;

    /**
     * @brief Perform pre-flight arming safety checks
     * @param buflen Size of error message buffer
     * @param[out] buffer Error message string (populated if check fails)
     * @return false if arming should be prevented, true if checks pass
     * 
     * @details Validates airspeed subsystem is safe for flight:
     * 1. If airspeed required (USE=1), sensor must be healthy
     * 2. Calibration must be complete (not IN_PROGRESS or FAILED)
     * 3. Sensor readings must be reasonable (not frozen, not excessive)
     * 4. EKF consistency acceptable (if enabled)
     * 
     * Populates buffer with human-readable failure reason if check fails.
     * 
     * @warning CRITICAL SAFETY CHECK - prevents takeoff with failed airspeed
     * @note Called by vehicle arming checks before allowing motor arming
     * 
     * Example failure messages:
     * - "Airspeed sensor unhealthy"
     * - "Airspeed calibration incomplete"
     * - "Airspeed reading too high"
     */
    bool arming_checks(size_t buflen, char *buffer) const;

private:
    /**
     * @brief Singleton instance pointer
     * @note Set during construction, accessed via get_singleton() or AP::airspeed()
     */
    static AP_Airspeed *_singleton;

    /**
     * @brief Library enable parameter (ARSPD_ENABLE)
     * @details Master enable/disable for entire airspeed subsystem
     * 0 = disabled (no sensor probing or updates)
     * 1 = enabled
     */
    AP_Int8 _enable;
    
    /**
     * @brief Check if library is enabled
     * @return true if ARSPD_ENABLE=1
     */
    bool lib_enabled() const;

    /**
     * @brief Primary sensor selection (ARSPD_PRIMARY parameter)
     * @details -1 = automatic (first healthy), 0-N = force specific sensor
     */
    AP_Int8 primary_sensor;
    
    /**
     * @brief Maximum airspeed percentage warning (ARSPD_USE parameter)
     * @details Percentage of configured maximum airspeed for warnings
     */
    AP_Int8 max_speed_pcnt;
    
    /**
     * @brief Options bitmask (ARSPD_OPTIONS parameter)
     * @see OptionsMask enum for bit definitions
     */
    AP_Int32 _options;
    
    /**
     * @brief Maximum allowable wind magnitude (ARSPD_WIND_MAX parameter, m/s)
     * @details Used for wind-based failure detection. If implied wind exceeds
     * this value, sensor marked as failed (if ON_FAILURE_AHRS_WIND_MAX_DO_DISABLE set).
     * Typical value: 20 m/s
     * Units: meters per second (m/s)
     */
    AP_Float _wind_max;
    
    /**
     * @brief Wind magnitude warning threshold (ARSPD_WIND_WARN parameter, m/s)
     * @details Generates warning message to GCS if wind estimate exceeds threshold
     * Units: meters per second (m/s)
     */
    AP_Float _wind_warn;
    
    /**
     * @brief EKF consistency gate size (ARSPD_WIND_GATE parameter)
     * @details Standard deviations for innovation test when USE_EKF_CONSISTENCY enabled.
     * Lower = stricter check (more false positives), Higher = looser (more false negatives)
     * Typical value: 5.0
     * Units: standard deviations (dimensionless)
     */
    AP_Float _wind_gate;

    /**
     * @brief Per-sensor parameter instances
     * @details Array of parameter structures, one per sensor slot
     * @see AP_Airspeed_Params
     */
    AP_Airspeed_Params param[AIRSPEED_MAX_SENSORS];

    /**
     * @struct airspeed_state
     * @brief Runtime state tracking for a single airspeed sensor
     * @details Maintains current readings, calibration state, health status,
     * and failure detection data for one sensor instance
     */
    struct airspeed_state {
        /** @brief Unfiltered airspeed (m/s) */
        float   raw_airspeed;
        
        /** @brief Low-pass filtered airspeed (m/s) */
        float   airspeed;
        
        /** @brief Most recent differential pressure reading (Pa) */
        float	last_pressure;
        
        /** @brief Filtered differential pressure (Pa) */
        float   filtered_pressure;
        
        /** @brief Pressure with offset correction applied (Pa) */
        float	corrected_pressure;
        
        /** @brief Timestamp of last sensor update (ms since boot) */
        uint32_t last_update_ms;
        
        /** @brief Health status flag (freshness, wind check, EKF consistency) */
        bool	healthy;

        /**
         * @brief Pre-flight offset calibration state
         * @details Tracks progress of ground-based offset calibration
         */
        struct {
            /** @brief Calibration start time (ms) */
            uint32_t start_ms;
            
            /** @brief Accumulated pressure sum for averaging (Pa) */
            float    sum;
            
            /** @brief Number of valid samples accumulated */
            uint16_t count;
            
            /** @brief Total read attempts (for detecting sensor issues) */
            uint16_t read_count;
            
            /** @brief Current calibration state */
            CalibrationState state;
        } cal;

#if AP_AIRSPEED_AUTOCAL_ENABLE
        /**
         * @brief In-flight ratio calibration filter instance
         * @see Airspeed_Calibration
         */
        Airspeed_Calibration calibration;
        
        /** @brief Last ratio value saved to EEPROM (dimensionless) */
        float last_saved_ratio;
        
        /** @brief Update counter for periodic saving */
        uint8_t counter;
#endif

        /**
         * @brief Failure detection and health monitoring state
         */
        struct {
            /** @brief Last health check timestamp (ms) */
            uint32_t last_check_ms;
            
            /** @brief Health probability estimate (0.0-1.0) */
            float health_probability;
            
            /** @brief EKF consistency test ratio */
            float test_ratio;
            
            /** @brief Backup of USE parameter before auto-disable (-1=not backed up) */
            int8_t param_use_backup;
            
            /** @brief Last warning message timestamp (ms, for rate limiting) */
            uint32_t last_warn_ms;
        } failures;

#if AP_AIRSPEED_HYGROMETER_ENABLE
        /** @brief Last hygrometer data log timestamp (ms) */
        uint32_t last_hygrometer_log_ms;
#endif
    } state[AIRSPEED_MAX_SENSORS]; ///< State array, one entry per sensor slot

    /**
     * @brief Global in-flight calibration enable flag
     * @details Set via set_calibration_enabled(), controls whether any sensors
     * perform automatic ratio calibration
     */
    bool calibration_enabled;

    /**
     * @brief Emergency override flag to disable all airspeed sensors
     * @details Set via force_disable_use(), overrides all USE parameters
     * @warning Removes stall protection when active
     */
    bool _force_disable_use;

    /**
     * @brief Current primary sensor index (0 to AIRSPEED_MAX_SENSORS-1)
     * @details Updated each update() cycle based on health and configuration
     */
    uint8_t primary;
    
    /**
     * @brief Total number of allocated sensor backends
     */
    uint8_t num_sensors;

    /**
     * @brief LOG_BITMASK bit position for airspeed logging
     * @details When corresponding bit set in vehicle LOG_BITMASK, airspeed logged
     */
    uint32_t _log_bit = -1;

    /**
     * @brief Read and process data from backend sensor
     * @param i Sensor index
     * @details Polls backend for pressure reading, applies corrections,
     * calculates airspeed, updates state
     */
    void read(uint8_t i);

    /**
     * @brief Get sensor health probability estimate
     * @param i Sensor index
     * @return Health probability (0.0 = definitely failed, 1.0 = definitely healthy)
     * @details Statistical estimate based on wind consistency and EKF innovation
     */
    float get_health_probability(uint8_t i) const {
        return state[i].failures.health_probability;
    }
    
    /**
     * @brief Get health probability for primary sensor
     */
    float get_health_probability(void) const {
        return get_health_probability(primary);
    }

    /**
     * @brief Get EKF consistency test ratio
     * @param i Sensor index
     * @return Test ratio (innovation / gate threshold)
     * @details Values < 1.0 pass consistency check, > 1.0 fail
     */
    float get_test_ratio(uint8_t i) const {
        return state[i].failures.test_ratio;
    }
    
    /**
     * @brief Get test ratio for primary sensor
     */
    float get_test_ratio(void) const {
        return get_test_ratio(primary);
    }

    /**
     * @brief Update offset calibration with new pressure reading
     * @param i Sensor index
     * @param raw_pressure Differential pressure in Pascals
     * @details Accumulates samples during calibration period
     */
    void update_calibration(uint8_t i, float raw_pressure);
    
    /**
     * @brief Update in-flight ratio calibration for specific sensor
     * @param i Sensor index
     * @param vground GPS ground velocity vector (NED, m/s)
     * @param max_airspeed_allowed_during_cal Maximum valid airspeed (m/s)
     * @details Calls Kalman filter update if conditions suitable
     */
    void update_calibration(uint8_t i, const Vector3f &vground, int16_t max_airspeed_allowed_during_cal);
    
    /**
     * @brief Send calibration status to GCS via MAVLink
     * @param vg Ground velocity vector (m/s)
     */
    void send_airspeed_calibration(const Vector3f &vg);
    
    /**
     * @brief Get current offset parameter value
     * @param i Sensor index
     * @return Offset in Pascals
     */
    float get_offset(uint8_t i) const;
    
    /**
     * @brief Get offset for primary sensor
     */
    float get_offset(void) const { return get_offset(primary); }

    /**
     * @brief Check all sensors for failure conditions
     * @details Runs wind-based and EKF consistency checks, updates health status,
     * triggers auto-disable/re-enable based on OPTIONS configuration
     */
    void check_sensor_failures();
    
    /**
     * @brief Check specific sensor for wind-based failure
     * @param i Sensor index
     * @details Compares indicated airspeed to EKF wind estimate, fails if
     * implied wind exceeds ARSPD_WIND_MAX threshold
     */
    void check_sensor_ahrs_wind_max_failures(uint8_t i);

    /**
     * @brief Backend sensor driver instances
     * @details Array of pointers to AP_Airspeed_Backend subclass instances,
     * one per configured sensor slot. nullptr for unconfigured slots.
     */
    AP_Airspeed_Backend *sensor[AIRSPEED_MAX_SENSORS];

    /**
     * @brief Write airspeed data to binary log
     * @details Logs pressure, airspeed, ratio, health for all enabled sensors
     */
    void Log_Airspeed();

    /**
     * @brief Register backend sensor with frontend
     * @param backend Pointer to backend instance to add
     * @return true if successfully added, false if no slots available
     * @details Called during backend allocation to register each sensor
     */
    bool add_backend(AP_Airspeed_Backend *backend);
    
    /**
     * @brief Pointer to fixed-wing parameter structure
     * @details Provides access to vehicle-specific parameters (stall speeds, etc.)
     * Set via set_fixedwing_parameters()
     */
    const AP_FixedWing *fixed_wing_parameters;

    /**
     * @brief Convert legacy parameters to per-instance format
     * @details Migration function for parameter system updates
     */
    void convert_per_instance();

};

/**
 * @namespace AP
 * @brief ArduPilot library accessor namespace
 */
namespace AP {
    /**
     * @brief Get pointer to AP_Airspeed singleton instance
     * @return Pointer to airspeed subsystem singleton
     * 
     * @details Preferred method for accessing airspeed subsystem from vehicle code.
     * Returns nullptr if airspeed subsystem not initialized or disabled.
     * 
     * Example usage:
     * @code
     * AP_Airspeed *airspeed = AP::airspeed();
     * if (airspeed && airspeed->healthy()) {
     *     float speed = airspeed->get_airspeed();
     * }
     * @endcode
     * 
     * @see AP_Airspeed::get_singleton() for direct singleton access
     */
    AP_Airspeed *airspeed();
};

#endif  // AP_AIRSPEED_ENABLED
