/**
 * @file AP_Baro.h
 * @brief Barometer subsystem manager and frontend interface
 * 
 * @details This file defines the AP_Baro class which manages multiple barometer
 *          sensor instances and provides a unified interface for pressure and
 *          altitude measurement. The barometer subsystem is critical for altitude
 *          hold, terrain following, and vertical velocity estimation.
 * 
 *          Key responsibilities:
 *          - Probe and initialize barometer backend drivers (MS5611, BMP280, etc.)
 *          - Manage up to 3 simultaneous barometer sensor instances
 *          - Calculate altitude from pressure using atmospheric models
 *          - Provide ground pressure calibration for relative altitude
 *          - Monitor sensor health and select primary sensor
 *          - Calculate vertical velocity (climb rate) from pressure changes
 *          - Support wind compensation and thrust correction for pressure
 * 
 *          Coordinate System: Altitude is reported in meters, positive up
 *          Units: Pressure in Pascals (Pa), Temperature in Celsius (°C)
 * 
 * @note Thread-safe API access via HAL_Semaphore (_rsem)
 * @warning Barometer must be calibrated on ground before using altitude functions
 * 
 * Source: libraries/AP_Baro/AP_Baro.h
 */

#pragma once

#include "AP_Baro_config.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <Filter/DerivativeFilter.h>
#include <AP_MSP/msp.h>
#include <AP_ExternalAHRS/AP_ExternalAHRS.h>

/**
 * @brief Maximum number of barometer sensor instances
 * 
 * @details Supports up to 3 simultaneous barometer sensors. Multiple sensors
 *          provide redundancy and allow primary sensor selection based on health.
 *          Typical configurations: single internal MS5611, or internal + external I2C
 */
#ifndef BARO_MAX_INSTANCES
#define BARO_MAX_INSTANCES 3
#endif

/**
 * @brief Maximum number of barometer backend drivers
 * 
 * @details Note that a single driver can provide multiple sensor instances.
 *          For example, a DroneCAN driver may handle multiple CAN barometers.
 */
#define BARO_MAX_DRIVERS 3

/**
 * @brief Sensor health timeout in milliseconds
 * 
 * @details If no successful read within 500ms, sensor marked unhealthy.
 *          Used for SYS_STATUS health reporting and primary sensor selection.
 */
#define BARO_TIMEOUT_MS                 500

/**
 * @brief Data change timeout in milliseconds
 * 
 * @details If pressure or temperature hasn't changed in 2000ms, sensor
 *          considered stuck. Detects frozen sensors or communication failures.
 */
#define BARO_DATA_CHANGE_TIMEOUT_MS     2000

class AP_Baro_Backend;

/**
 * @class AP_Baro
 * @brief Barometer subsystem frontend and singleton manager
 * 
 * @details AP_Baro is the frontend class managing multiple barometer sensors and
 *          providing a unified interface for pressure and altitude measurement.
 *          This class is fundamental to altitude control, terrain following, and
 *          vertical navigation in all vehicle types.
 * 
 * **Architecture**:
 * - Frontend-backend pattern: AP_Baro (frontend) ↔ AP_Baro_Backend (drivers)
 * - Singleton accessible via AP::baro() or AP_Baro::get_singleton()
 * - Manages up to BARO_MAX_INSTANCES (3) sensor instances
 * - Supports up to BARO_MAX_DRIVERS (3) backend drivers
 * 
 * **Lifecycle**:
 * 1. Construction: AP_Baro() initializes parameters and data structures
 * 2. init() - Probes and loads backend drivers (MS5611, BMP280, MS5837, etc.)
 * 3. calibrate() - MUST be called on ground to set altitude reference
 * 4. update() - Called each main loop iteration, backends push new data
 * 5. get_*() - Retrieve pressure, temperature, altitude, climb rate
 * 
 * **Sensor Registration**:
 * - Backend drivers call register_sensor() during init to claim an instance slot
 * - Frontend tracks sensor health, last update time, and data validity
 * - Primary sensor auto-selected based on user preference and health status
 * 
 * **Altitude Calculation**:
 * - Uses atmospheric model to convert pressure to altitude
 * - Altitude is relative to calibrate() call unless using get_altitude_AMSL()
 * - Formula: alt = f(pressure, ground_pressure, ground_temperature)
 * - Two models: simple (fast) and extended (1976 standard atmosphere, accurate)
 * 
 * **Calibration**:
 * - calibrate() stores ground_pressure and ground_temperature for each sensor
 * - These ground values become the altitude zero reference
 * - update_calibration() allows incremental preflight calibration updates
 * - Field elevation parameter adds AMSL offset for absolute altitude
 * 
 * **Health Monitoring**:
 * - healthy(instance): true if sensor has recent data and valid altitude
 * - Timeout: BARO_TIMEOUT_MS (500ms) without update marks sensor unhealthy
 * - Data change timeout: BARO_DATA_CHANGE_TIMEOUT_MS (2000ms) detects stuck sensors
 * - all_healthy(): checks all instances for SYS_STATUS reporting
 * 
 * **Primary Sensor Selection Algorithm**:
 * 1. If user sets _primary_baro parameter, use that instance (if healthy)
 * 2. Otherwise, select lowest numbered healthy instance
 * 3. Primary selection affects default get_*() methods without instance parameter
 * 
 * **Climb Rate Calculation**:
 * - get_climb_rate() differentiates altitude via DerivativeFilterFloat_Size7
 * - Filter smooths noisy altitude to produce stable climb rate in m/s
 * - Positive = ascending, negative = descending
 * 
 * **Atmospheric Modeling**:
 * - Simple model: Exponential pressure-altitude relationship, constant temperature
 * - Extended model: 1976 International Standard Atmosphere with temperature layers
 * - Functions: get_altitude_difference(), get_EAS2TAS(), air density ratio
 * - Supports underwater operation for ROVs via _specific_gravity (1.0 fresh, 1.024 salt)
 * 
 * **Wind Compensation** (HAL_BARO_WIND_COMP_ENABLED):
 * - Corrects static pressure for dynamic pressure effects from airspeed
 * - WindCoeff parameters: xp, xn, yp, yn, zp, zn (per-axis pressure rise ratios)
 * - get_dynamic_pressure(): Returns wind-induced pressure correction vector
 * - get_corrected_pressure(): Pressure compensated for wind and thrust effects
 * - Requires AP_AHRS airspeed vector for correction calculation
 * 
 * **Thrust Compensation** (AP_BARO_THST_COMP_ENABLED):
 * - Corrects pressure for propeller downwash and airframe effects
 * - mot_scale parameter per sensor for thrust-based pressure scaling
 * 
 * **Multi-Sensor Handling**:
 * - Each sensor: independent health tracking, calibration, altitude calculation
 * - Outlier rejection: _filter_range parameter limits valid deviation from mean
 * - Instance selection: Use specific instance or default to primary
 * 
 * **External Interfaces**:
 * - MAVLink: Sends scaled pressure messages, handles parameter protocol
 * - MSP: handle_msp() for MSP OSD integration (AP_BARO_MSP_ENABLED)
 * - ExternalAHRS: handle_external() for external AHRS barometer data
 * - Logging: Write_Baro() logs pressure, temperature, altitude when enabled
 * 
 * **Thread Safety**:
 * - _rsem (HAL_Semaphore) protects sensors[] array and internal state
 * - get_semaphore() provides external lock for multi-threaded access
 * - Backend drivers lock _rsem when pushing data to frontend
 * 
 * **Sensor Types**:
 * - BARO_TYPE_AIR: Standard atmospheric pressure sensor (default)
 * - BARO_TYPE_WATER: Underwater pressure sensor for depth measurement (ROVs)
 * - set_type() / get_type() configure sensor interpretation
 * 
 * **Parameters**:
 * - GND_PRIMARY: User override for primary sensor selection
 * - GND_PROBE_EXT: Bitmask for external I2C barometer probing
 * - GND_EXT_BUS: Which I2C bus to probe for external sensors
 * - GND_ALT_OFFSET: Manual altitude offset for barometer drift compensation
 * - GND_SPEC_GRAV: Specific gravity for underwater pressure (1.0-1.024)
 * - GNDn_WND_* : Wind compensation coefficients (per instance)
 * - See var_info[] for complete parameter list
 * 
 * **Supported Barometer Types**:
 * - MS5611, MS5607, MS5637, MS5837 (TE Connectivity)
 * - BMP085, BMP280, BMP388, BMP581 (Bosch)
 * - LPS25H, LPS22H (ST Microelectronics)
 * - DPS280, DPS310 (Infineon)
 * - FBM320 (Formosa Measurement Technology)
 * - SPL06 (Goertek)
 * - Keller LD series
 * - DroneCAN/UAVCAN barometers
 * - MSP barometers
 * - External AHRS barometers
 * 
 * **Usage Example**:
 * @code
 * // Initialization (in vehicle setup)
 * AP::baro().init();
 * AP::baro().calibrate();  // MUST call on ground before flight
 * 
 * // Main loop update
 * AP::baro().update();
 * 
 * // Read altitude and climb rate
 * float altitude_m = AP::baro().get_altitude();  // meters relative to calibrate()
 * float climb_rate_ms = AP::baro().get_climb_rate();  // m/s, positive = up
 * 
 * // Read pressure and temperature
 * float pressure_pa = AP::baro().get_pressure();  // Pascals
 * float temp_c = AP::baro().get_temperature();  // Celsius
 * 
 * // Check health
 * if (AP::baro().healthy()) {
 *     // Use barometer data
 * }
 * @endcode
 * 
 * @note All units explicit: Pressure (Pa), Temperature (°C), Altitude (m), Time (ms)
 * @warning calibrate() MUST be called on ground before using altitude functions
 * @warning Altitude is relative to calibration point unless using get_altitude_AMSL()
 * @warning Changing ground_pressure parameter affects altitude zero reference
 * 
 * @see AP_Baro_Backend for backend driver interface
 * @see libraries/AP_Baro/AP_Baro_MS5611.cpp for example backend implementation
 */
class AP_Baro
{
    friend class AP_Baro_Backend;
    friend class AP_Baro_SITL; // for access to sensors[]
    friend class AP_Baro_DroneCAN; // for access to sensors[]

public:
    AP_Baro();

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Baro);

    /**
     * @brief Get singleton instance of AP_Baro
     * 
     * @return Pointer to the singleton AP_Baro instance, or nullptr if not initialized
     * 
     * @note Prefer using AP::baro() accessor from AP namespace
     */
    static AP_Baro *get_singleton(void) {
        return _singleton;
    }

    /**
     * @enum baro_type_t
     * @brief Barometer sensor type for pressure interpretation
     * 
     * @details Determines how pressure readings are interpreted. Air mode uses
     *          atmospheric pressure models for altitude. Water mode uses hydrostatic
     *          pressure for depth measurement in underwater vehicles (ROVs).
     */
    typedef enum {
        BARO_TYPE_AIR,      ///< Atmospheric pressure sensor for altitude measurement (default)
        BARO_TYPE_WATER     ///< Underwater pressure sensor for depth measurement (ROVs)
    } baro_type_t;

    /**
     * @brief Initialize barometer subsystem and probe backend drivers
     * 
     * @details Initializes the barometer subsystem by probing for supported
     *          barometer sensors on configured buses (SPI, I2C, CAN). Loads
     *          backend drivers for detected sensors and registers sensor instances.
     * 
     *          Initialization sequence:
     *          1. Probe internal SPI barometers (board-specific)
     *          2. Probe external I2C barometers based on GND_PROBE_EXT bitmask
     *          3. Initialize DroneCAN barometer discovery
     *          4. Register MSP and ExternalAHRS barometer handlers
     * 
     *          Supported probe methods:
     *          - Automatic board-specific internal sensors (via hwdef)
     *          - External I2C scanning on bus specified by GND_EXT_BUS parameter
     *          - DroneCAN/UAVCAN barometers via CAN bus
     *          - MSP barometers from OSD devices
     *          - External AHRS barometer data
     * 
     * @note Called once during vehicle initialization before scheduler starts
     * @note Must be called before calibrate() or update()
     * @warning Failure to detect any barometer may panic or disable altitude features
     * 
     * @see register_sensor() for backend sensor registration
     */
    __INITFUNC__ void init(void);

    /**
     * @brief Update all barometer sensors and calculate derived values
     * 
     * @details Updates all registered barometer backends, which push new pressure
     *          and temperature data to the frontend. Calculates altitude, climb rate,
     *          and applies corrections. Must be called regularly from main loop.
     * 
     *          Update sequence:
     *          1. Backends read hardware and push data via _copy_to_frontend()
     *          2. Update health status based on data freshness
     *          3. Calculate altitude from pressure using atmospheric model
     *          4. Update climb rate filter with new altitude
     *          5. Apply wind compensation if enabled
     *          6. Apply thrust compensation if enabled
     *          7. Select primary sensor if health status changed
     *          8. Log data if logging enabled
     * 
     * @note Typically called at main loop rate (50-400 Hz depending on vehicle)
     * @note Thread-safe via _rsem semaphore
     * @warning Sensor health affects EKF and altitude control - critical path
     * 
     * @see healthy() to check if data is valid after update
     */
    void update(void);

    /**
     * @brief Check if primary barometer sensor is healthy
     * 
     * @details Returns true if primary sensor has recent valid data and calculated
     *          altitude is valid. Health determined by update timeout and data change.
     * 
     * @return true if primary sensor healthy with valid altitude, false otherwise
     * 
     * @note Health checked against BARO_TIMEOUT_MS (500ms) since last update
     * @see healthy(uint8_t instance) for specific instance health check
     */
    bool healthy(void) const { return healthy(_primary); }

    /**
     * @brief Check if specific barometer sensor is healthy
     * 
     * @details A sensor is healthy if:
     *          - Last update within BARO_TIMEOUT_MS (500ms)
     *          - Pressure or temperature changed within BARO_DATA_CHANGE_TIMEOUT_MS (2000ms)
     *          - Calculated altitude is valid (alt_ok flag)
     *          - Sensor calibrated successfully
     * 
     * @param[in] instance Sensor instance number (0 to BARO_MAX_INSTANCES-1), default=primary
     * 
     * @return true if sensor healthy and altitude calculation valid, false otherwise
     * 
     * @note Used for primary sensor selection and EKF sensor fusion decisions
     * @warning Unhealthy sensor may trigger failsafe if no healthy alternatives
     */
    bool healthy(uint8_t instance) const;

    /**
     * @brief Check if all registered barometer sensors are healthy
     * 
     * @details Used for MAVLink SYS_STATUS reporting to indicate barometer subsystem health.
     *          Returns true only if ALL registered sensors are healthy.
     * 
     * @return true if all sensors healthy, false if any sensor unhealthy
     * 
     * @note Used for SYS_STATUS.onboard_control_sensors_health bitmask
     * @see healthy(uint8_t instance) for individual sensor health
     */
    bool all_healthy(void) const;

    /**
     * @brief Perform pre-arm checks for barometer subsystem
     * 
     * @details Validates barometer configuration and health before allowing arming.
     *          Checks performed:
     *          - At least one sensor healthy
     *          - Primary sensor calibrated
     *          - Altitude error within acceptable bounds
     *          - No excessive sensor disagreement
     * 
     * @param[in]  buflen Size of failure message buffer in bytes
     * @param[out] buffer Buffer to populate with failure message if check fails
     * 
     * @return false if arming should be blocked, true if checks pass
     * 
     * @note Failure message written to buffer explains specific check failure
     * @warning Arming blocked if barometer health insufficient for safe flight
     */
    bool arming_checks(size_t buflen, char *buffer) const;

    /**
     * @brief Get primary barometer sensor instance number
     * 
     * @details Returns the currently selected primary sensor. Selection algorithm:
     *          1. If GND_PRIMARY parameter set, use that instance (if healthy)
     *          2. Otherwise, use lowest numbered healthy instance
     *          3. Primary affects all get_*() methods without instance parameter
     * 
     * @return Primary sensor instance number (0 to BARO_MAX_INSTANCES-1)
     * 
     * @note Primary may change dynamically if selected sensor becomes unhealthy
     * @see set_primary_baro() to override primary selection
     */
    uint8_t get_primary(void) const { return _primary; }

    /**
     * @brief Get pressure from primary barometer sensor
     * 
     * @details Returns atmospheric or hydrostatic pressure from primary sensor.
     * 
     * @return Pressure in Pascals (Pa). Divide by 100 for millibars or hectopascals.
     * 
     * @note At sea level, standard atmospheric pressure is ~101325 Pa (1013.25 mbar)
     * @see get_pressure(uint8_t instance) for specific sensor
     */
    float get_pressure(void) const { return get_pressure(_primary); }

    /**
     * @brief Get pressure from specific barometer sensor
     * 
     * @details Returns raw pressure reading from specified sensor instance.
     *          For BARO_TYPE_AIR: atmospheric pressure for altitude calculation.
     *          For BARO_TYPE_WATER: hydrostatic pressure for depth measurement.
     * 
     * @param[in] instance Sensor instance number (0 to BARO_MAX_INSTANCES-1)
     * 
     * @return Pressure in Pascals (Pa). Divide by 100 for millibars or hectopascals.
     * 
     * @note Pressure NOT corrected for wind or thrust - use get_corrected_pressure() for corrections
     */
    float get_pressure(uint8_t instance) const { return sensors[instance].pressure; }

#if HAL_BARO_WIND_COMP_ENABLED
    /**
     * @brief Get dynamic pressure correction vector from wind effects
     * 
     * @details Returns calculated dynamic pressure induced by airspeed for wind
     *          compensation. Based on WindCoeff parameters (xp, xn, yp, yn, zp, zn)
     *          which define pressure rise ratios per axis.
     * 
     *          Correction calculated from:
     *          - Airspeed vector from AP_AHRS
     *          - Wind coefficient parameters per sensor
     *          - Dynamic pressure = 0.5 * rho * V^2 * coeff
     * 
     * @param[in] instance Sensor instance number (0 to BARO_MAX_INSTANCES-1)
     * 
     * @return Dynamic pressure correction vector in Pascals (Pa), body frame axes
     * 
     * @note Requires HAL_BARO_WIND_COMP_ENABLED and valid airspeed from AHRS
     * @warning Wind compensation accuracy depends on coefficient calibration
     * 
     * @see get_corrected_pressure() for pressure with wind correction applied
     */
    const Vector3f& get_dynamic_pressure(uint8_t instance) const { return sensors[instance].dynamic_pressure; }
#endif

#if (HAL_BARO_WIND_COMP_ENABLED || AP_BARO_THST_COMP_ENABLED)
    /**
     * @brief Get pressure corrected for wind and thrust effects
     * 
     * @details Returns pressure compensated for:
     *          - Dynamic pressure from airspeed (wind compensation)
     *          - Propeller downwash effects (thrust compensation)
     * 
     *          Corrected pressure more accurately reflects true static pressure
     *          for improved altitude estimation in dynamic flight conditions.
     * 
     * @param[in] instance Sensor instance number (0 to BARO_MAX_INSTANCES-1)
     * 
     * @return Corrected pressure in Pascals (Pa)
     * 
     * @note Requires HAL_BARO_WIND_COMP_ENABLED or AP_BARO_THST_COMP_ENABLED
     * @warning Correction quality depends on parameter calibration
     * 
     * @see get_dynamic_pressure() for wind-induced pressure component
     */
    float get_corrected_pressure(uint8_t instance) const { return sensors[instance].corrected_pressure; }
#endif

    /**
     * @brief Get temperature from primary barometer sensor
     * 
     * @return Temperature in degrees Celsius (°C)
     * 
     * @note Most barometers include integrated temperature sensor for pressure compensation
     * @see get_temperature(uint8_t instance) for specific sensor
     */
    float get_temperature(void) const { return get_temperature(_primary); }

    /**
     * @brief Get temperature from specific barometer sensor
     * 
     * @details Returns temperature reading from specified sensor. Temperature used
     *          internally for pressure-to-altitude conversion and sensor compensation.
     * 
     * @param[in] instance Sensor instance number (0 to BARO_MAX_INSTANCES-1)
     * 
     * @return Temperature in degrees Celsius (°C)
     * 
     * @note Temperature sensor integrated in barometer chip, not ambient air temperature
     */
    float get_temperature(uint8_t instance) const { return sensors[instance].temperature; }

    /**
     * @brief Get pressure correction from calibration for primary sensor
     * 
     * @details Returns pressure correction applied from AP_TempCalibration or other
     *          calibration sources. Correction compensates for temperature-dependent
     *          sensor errors.
     * 
     * @return Pressure correction in Pascals (Pa). Divide by 100 for millibars or hectopascals.
     * 
     * @note Correction added to raw pressure reading before altitude calculation
     * @see set_pressure_correction() to apply correction
     */
    float get_pressure_correction(void) const { return get_pressure_correction(_primary); }

    /**
     * @brief Get pressure correction from calibration for specific sensor
     * 
     * @details Returns pressure correction offset applied to specified sensor from
     *          external calibration (e.g., AP_TempCalibration temperature compensation).
     * 
     * @param[in] instance Sensor instance number (0 to BARO_MAX_INSTANCES-1)
     * 
     * @return Pressure correction in Pascals (Pa). Divide by 100 for millibars or hectopascals.
     * 
     * @note Positive correction increases reported pressure
     * @see set_pressure_correction() to apply calibration correction
     */
    float get_pressure_correction(uint8_t instance) const { return sensors[instance].p_correction; }

    /**
     * @brief Calibrate barometer on ground to establish altitude zero reference
     * 
     * @details CRITICAL: Must be called on ground before flight to set altitude reference.
     *          Calibration stores current ground_pressure and ground_temperature for each
     *          sensor, which become the zero altitude reference for get_altitude().
     * 
     *          Calibration process:
     *          1. Read current pressure and temperature from all sensors
     *          2. Store as ground_pressure and ground_temperature
     *          3. Optionally save ground_pressure to EEPROM parameters
     *          4. Reset altitude to zero (relative to current position)
     * 
     *          After calibration:
     *          - get_altitude() returns meters above calibration point
     *          - Positive altitude = above calibration level
     *          - Negative altitude = below calibration level
     * 
     * @param[in] save If true, save ground_pressure to EEPROM parameters (default=true).
     *                 Set false for temporary calibration not persisted across reboots.
     * 
     * @note Must be called when vehicle stationary on ground
     * @note Field elevation parameter (GND_ALT_OFFSET) adds AMSL offset if needed
     * @warning MUST call before using altitude functions or altitude will be incorrect
     * @warning Vehicle must be on ground, not in flight, for valid calibration
     * @warning Barometer drift over time may require recalibration
     * 
     * @see update_calibration() for incremental preflight calibration updates
     * @see get_altitude() for altitude relative to calibration
     * @see get_ground_pressure() to retrieve stored calibration pressure
     */
    void calibrate(bool save=true);

    /**
     * @brief Update barometer calibration to current pressure
     * 
     * @details Incrementally updates ground pressure calibration to current measured
     *          pressure. Useful for preflight calibration updates while on ground to
     *          account for changing weather conditions or barometer drift.
     * 
     *          Use case:
     *          - Vehicle sits on ground for extended period
     *          - Atmospheric pressure changes (weather front moves in)
     *          - Call update_calibration() to adjust altitude zero reference
     *          - Avoids altitude offset at takeoff
     * 
     * @note Vehicle must be stationary on ground when called
     * @note Does NOT save to EEPROM - temporary update for current flight
     * @warning Do not call in flight - will cause sudden altitude jump
     * 
     * @see calibrate() for full calibration with EEPROM save option
     */
    void update_calibration(void);

    /**
     * @brief Get altitude from primary sensor relative to calibration point
     * 
     * @details Returns altitude in meters relative to the altitude at the time of
     *          the last calibrate() call. Calculated from pressure using atmospheric
     *          model: alt = f(pressure, ground_pressure, ground_temperature).
     * 
     * @return Altitude in meters (m) above calibration point.
     *         Positive = above calibration level, Negative = below calibration level.
     * 
     * @note RELATIVE altitude, not absolute AMSL - see get_altitude_AMSL() for absolute
     * @warning Returns 0 before calibrate() is called
     * @warning Accuracy degrades with altitude (±10m at 10km altitude)
     * 
     * @see calibrate() to set altitude zero reference
     * @see get_altitude_AMSL() for altitude above mean sea level
     */
    float get_altitude(void) const { return get_altitude(_primary); }

    /**
     * @brief Get altitude from specific sensor relative to calibration point
     * 
     * @details Returns calculated altitude for specified sensor instance.
     *          Altitude computed from: alt = f(pressure, ground_pressure, ground_temperature)
     *          using atmospheric model (simple or extended based on configuration).
     * 
     * @param[in] instance Sensor instance number (0 to BARO_MAX_INSTANCES-1)
     * 
     * @return Altitude in meters (m) above calibration point
     * 
     * @note Each sensor maintains independent altitude calculation and calibration
     * @see get_altitude_difference() for altitude change calculation algorithm
     */
    float get_altitude(uint8_t instance) const { return sensors[instance].altitude; }

    /**
     * @brief Get altitude above mean sea level (AMSL) for specific sensor
     * 
     * @details Returns absolute altitude above mean sea level by adding field
     *          elevation offset to relative altitude. Uses GND_ALT_OFFSET parameter
     *          or EKF-derived field elevation.
     * 
     * @param[in] instance Sensor instance number (0 to BARO_MAX_INSTANCES-1)
     * 
     * @return Altitude in meters (m) above mean sea level (AMSL)
     * 
     * @note AMSL altitude = relative altitude + field elevation offset
     * @note Field elevation from GND_ALT_OFFSET parameter or EKF origin altitude
     * @warning Requires valid field elevation - otherwise returns relative altitude only
     * 
     * @see get_altitude() for relative altitude
     */
    float get_altitude_AMSL(uint8_t instance) const { return get_altitude(instance) + _field_elevation_active; }

    /**
     * @brief Get altitude above mean sea level (AMSL) from primary sensor
     * 
     * @details Returns absolute altitude above mean sea level from primary sensor.
     *          Adds field elevation offset to relative altitude for absolute reference.
     * 
     * @return Altitude in meters (m) above mean sea level (AMSL)
     * 
     * @note Used for terrain following, geofencing, and flight planning
     * @see get_altitude_AMSL(uint8_t instance) for specific sensor
     */
    float get_altitude_AMSL(void) const { return get_altitude_AMSL(_primary); }

    /**
     * @brief Get I2C bus number considered as external bus for barometer probing
     * 
     * @details Returns which I2C bus is designated as "external" for probing external
     *          barometer sensors. Configured via GND_EXT_BUS parameter.
     * 
     * @return I2C bus number (0-3) used for external sensor probing
     * 
     * @note Used during init() to probe for external I2C barometers
     * @see init() for barometer probing sequence
     */
    uint8_t external_bus() const { return _ext_bus; }

    /**
     * @brief Convert geometric altitude to geopotential altitude
     * 
     * @details Converts actual geometric altitude to geopotential altitude per 1976
     *          International Standard Atmosphere model. Geopotential altitude accounts
     *          for variation in gravity with altitude.
     * 
     *          Relationship: H = (r0 / (r0 + h)) * h
     *          where H = geopotential altitude, h = geometric altitude, r0 = Earth radius
     * 
     * @param[in] alt Geometric altitude in meters (m)
     * 
     * @return Geopotential altitude in meters (m)
     * 
     * @note Used internally for 1976 standard atmosphere pressure/temperature lookup
     * @see geopotential_alt_to_geometric() for inverse conversion
     */
    static float geometric_alt_to_geopotential(float alt);

    /**
     * @brief Convert geopotential altitude to geometric altitude
     * 
     * @details Inverse of geometric_alt_to_geopotential(). Converts geopotential
     *          altitude back to actual geometric altitude.
     * 
     * @param[in] alt Geopotential altitude in meters (m)
     * 
     * @return Geometric altitude in meters (m)
     * 
     * @see geometric_alt_to_geopotential() for forward conversion
     */
    static float geopotential_alt_to_geometric(float alt);

    /**
     * @brief Get expected temperature for given altitude
     * 
     * @details Looks up expected atmospheric temperature at specified altitude using
     *          atmospheric model (simple or 1976 standard atmosphere). Used for
     *          altitude calculation and EAS2TAS conversion.
     * 
     * @param[in] alt Altitude in meters (m) AMSL
     * 
     * @return Temperature in Kelvin (K)
     * 
     * @note Simple model: constant temperature. Extended model: temperature layers.
     * @see get_temperatureC_for_alt_amsl() for temperature in Celsius
     */
    float get_temperature_from_altitude(float alt) const;

    /**
     * @brief Calculate altitude from absolute pressure
     * 
     * @details Calculates altitude above mean sea level from absolute pressure using
     *          atmospheric model. Inverse of get_pressure_for_alt_amsl().
     * 
     *          Uses ground temperature for calculation. Algorithm depends on
     *          atmospheric model (simple exponential or 1976 standard atmosphere).
     * 
     * @param[in] pressure Absolute pressure in Pascals (Pa)
     * 
     * @return Altitude in meters (m) above mean sea level
     * 
     * @note Used for absolute altitude calculation from raw pressure
     * @warning Assumes standard atmosphere - actual conditions may differ
     * 
     * @see get_altitude_difference() for relative altitude between two pressures
     */
    float get_altitude_from_pressure(float pressure) const;

    /**
     * @brief Get Equivalent Airspeed to True Airspeed conversion factor
     * 
     * @details Returns scaling factor to convert Equivalent Airspeed (EAS) to True
     *          Airspeed (TAS) at specified altitude. Accounts for air density decrease
     *          with altitude: TAS = EAS * factor.
     * 
     *          Factor = sqrt(rho_sea_level / rho_altitude)
     * 
     * @param[in] alt_amsl Altitude above mean sea level in meters (m)
     * 
     * @return EAS to TAS scaling factor (dimensionless, typically 1.0 at sea level to ~1.6 at 5km)
     * 
     * @note Used by SITL backend and airspeed calculations
     * @note At sea level: factor ≈ 1.0. At 5000m: factor ≈ 1.26. At 10000m: factor ≈ 1.62
     * 
     * @see _get_EAS2TAS() for current altitude EAS2TAS from primary sensor
     */
    static float get_EAS2TAS_for_alt_amsl(float alt_amsl);

#if AP_BARO_1976_STANDARD_ATMOSPHERE_ENABLED
    /**
     * @brief Lookup pressure and temperature for altitude using 1976 standard atmosphere
     * 
     * @details Returns expected pressure and temperature at specified altitude per
     *          1976 International Standard Atmosphere model with temperature layers.
     *          Used by SITL backend for simulated barometer readings.
     * 
     * @param[in]  alt_amsl      Altitude above mean sea level in meters (m)
     * @param[out] pressure      Calculated pressure in Pascals (Pa)
     * @param[out] temperature_K Calculated temperature in Kelvin (K)
     * 
     * @note Only available if AP_BARO_1976_STANDARD_ATMOSPHERE_ENABLED
     * @note Accurate representation of standard atmosphere for simulation
     */
    static void get_pressure_temperature_for_alt_amsl(float alt_amsl, float &pressure, float &temperature_K);
#endif

    /**
     * @brief Lookup expected temperature for altitude in Celsius
     * 
     * @details Returns expected atmospheric temperature at specified altitude using
     *          atmospheric model. Used by SITL backend for temperature simulation.
     * 
     * @param[in] alt_amsl Altitude above mean sea level in meters (m)
     * 
     * @return Temperature in degrees Celsius (°C)
     * 
     * @note Simple model: constant temp. Extended model: temperature lapse rate.
     * @see get_temperature_from_altitude() for temperature in Kelvin
     */
    static float get_temperatureC_for_alt_amsl(const float alt_amsl);

    /**
     * @brief Lookup expected pressure for altitude
     * 
     * @details Returns expected atmospheric pressure at specified altitude using
     *          atmospheric model (simple or 1976 standard atmosphere). Used by SITL
     *          backend to generate simulated barometer pressure readings.
     * 
     * @param[in] alt_amsl Altitude above mean sea level in meters (m)
     * 
     * @return Pressure in Pascals (Pa)
     * 
     * @note Sea level standard pressure: 101325 Pa (1013.25 mbar)
     * @note At 1000m: ~89875 Pa. At 5000m: ~54048 Pa. At 10000m: ~26500 Pa.
     * 
     * @see get_altitude_from_pressure() for inverse calculation
     */
    static float get_pressure_for_alt_amsl(const float alt_amsl);
    
    /**
     * @brief Get air density ratio for altitude
     * 
     * @details Returns air density at altitude relative to sea level density.
     *          Used for EAS2TAS conversion and dynamic pressure calculations.
     *          Used by SITL for aerodynamic force simulation.
     * 
     *          Density ratio = rho(altitude) / rho(sea_level)
     * 
     * @param[in] alt_amsl Altitude above mean sea level in meters (m)
     * 
     * @return Air density ratio (dimensionless, 0.0 to 1.0)
     *         1.0 at sea level, ~0.74 at 3km, ~0.53 at 6km, ~0.34 at 10km
     * 
     * @note Used for airspeed and aerodynamic calculations
     * @see _get_air_density_ratio() for current altitude density ratio
     */
    static float get_air_density_for_alt_amsl(float alt_amsl);

    /**
     * @brief Calculate altitude difference from pressure difference
     * 
     * @details Calculates altitude change in meters between two pressure levels using
     *          atmospheric model. Useful for relative altitude calculations without
     *          needing absolute altitude reference.
     * 
     *          Algorithm: Uses barometric formula with ground temperature to convert
     *          pressure ratio to altitude difference. More accurate than simple
     *          proportional scaling.
     * 
     * @param[in] base_pressure Reference pressure in Pascals (Pa) at base altitude
     * @param[in] pressure      Current pressure in Pascals (Pa) at current altitude
     * 
     * @return Altitude difference in meters (m). Positive = above base, Negative = below base.
     * 
     * @note Used internally for get_altitude() calculation from ground_pressure
     * @note Accuracy better than ±1m for altitude changes up to 1000m
     * @warning Large altitude differences (>3km) may have reduced accuracy
     * 
     * @see get_altitude_from_pressure() for absolute altitude from pressure
     */
    float get_altitude_difference(float base_pressure, float pressure) const;

    /**
     * @brief Calculate sea level pressure from current pressure and altitude
     * 
     * @details Extrapolates sea level pressure from current pressure measurement and
     *          altitude using 1976 standard atmosphere model. Inverse operation of
     *          converting pressure to altitude.
     * 
     *          Used to set QNH for altimeter calibration and weather reporting.
     * 
     * @param[in] pressure Current atmospheric pressure in Pascals (Pa)
     * @param[in] altitude Current altitude above mean sea level in meters (m)
     * 
     * @return Calculated sea level pressure in Pascals (Pa)
     * 
     * @note QNH = sea level pressure adjusted to standard atmosphere
     * @note Standard sea level pressure: 101325 Pa (1013.25 mbar)
     * @warning Assumes standard atmosphere - non-standard conditions introduce error
     */
    float get_sealevel_pressure(float pressure, float altitude) const;

    /**
     * @brief Get EAS to TAS conversion factor for current altitude (internal use)
     * 
     * @details Returns Equivalent Airspeed to True Airspeed scaling factor for
     *          current barometric altitude. Used internally to update AHRS EAS2TAS value.
     * 
     *          DO NOT CALL DIRECTLY - use AP::ahrs().get_EAS2TAS() instead.
     *          This method should only be used once per loop to update AHRS.
     * 
     * @return EAS to TAS scaling factor (dimensionless)
     * 
     * @note Internal method - external code should use AP::ahrs().get_EAS2TAS()
     * @warning Calling directly bypasses AHRS caching and may cause inconsistency
     * 
     * @see get_EAS2TAS_for_alt_amsl() for EAS2TAS at specific altitude
     */
    float _get_EAS2TAS(void) const;

    /**
     * @brief Get air density ratio for current altitude (internal use)
     * 
     * @details Returns air density at current altitude relative to sea level density.
     *          Used internally to update AHRS air density value.
     * 
     *          DO NOT CALL DIRECTLY - use AP::ahrs().get_air_density_ratio() instead.
     *          This method should only be used once per loop to update AHRS.
     * 
     * @return Air density ratio (dimensionless, 0.0 to 1.0)
     * 
     * @note Internal method - external code should use AP::ahrs().get_air_density_ratio()
     * @warning Calling directly bypasses AHRS caching and may cause inconsistency
     * 
     * @see get_air_density_for_alt_amsl() for density ratio at specific altitude
     */
    float _get_air_density_ratio(void);

    /**
     * @brief Get current vertical velocity (climb rate)
     * 
     * @details Returns vertical velocity calculated by differentiating barometric
     *          altitude over time. Filtered with DerivativeFilterFloat_Size7 to reduce
     *          noise and provide smooth climb rate estimate.
     * 
     *          Positive value = ascending, Negative value = descending
     * 
     * @return Climb rate in meters per second (m/s)
     *         Positive = climbing, Negative = descending, Zero = level flight
     * 
     * @note Typical response time: ~1 second due to filtering
     * @note Accuracy: ±0.5 m/s in steady flight, may lag in dynamic maneuvers
     * @warning Requires regular update() calls for valid calculation
     * 
     * @see update() which updates altitude used for climb rate differentiation
     */
    float get_climb_rate(void);

    /**
     * @brief Get ground temperature from calibration
     * 
     * @details Returns ground temperature stored during last calibrate() call.
     *          Used as reference temperature for atmospheric model calculations.
     * 
     *          Ground values only valid after calibration - returns 0 before calibrate().
     * 
     * @return Ground temperature in degrees Celsius (°C)
     * 
     * @note Value stored during calibrate() and used for altitude calculations
     * @warning Returns 0 or invalid value before calibrate() is called
     * 
     * @see calibrate() to set ground temperature reference
     */
    float get_ground_temperature(void) const;

    /**
     * @brief Get ground pressure from calibration for primary sensor
     * 
     * @details Returns ground pressure stored during last calibrate() call for
     *          primary sensor. This is the pressure zero reference for altitude.
     * 
     * @return Ground pressure in Pascals (Pa)
     * 
     * @note Ground values only valid after calibration
     * @warning Returns parameter default before calibrate() is called
     * 
     * @see get_ground_pressure(uint8_t i) for specific sensor
     * @see calibrate() to set ground pressure reference
     */
    float get_ground_pressure(void) const { return get_ground_pressure(_primary); }

    /**
     * @brief Get ground pressure from calibration for specific sensor
     * 
     * @details Returns ground pressure reference stored during calibrate() for
     *          specified sensor instance. Each sensor maintains independent calibration.
     * 
     * @param[in] i Sensor instance number (0 to BARO_MAX_INSTANCES-1)
     * 
     * @return Ground pressure in Pascals (Pa)
     * 
     * @note Value retrieved from AP_Param ground_pressure parameter
     * @note Each sensor has independent ground pressure calibration
     */
    float get_ground_pressure(uint8_t i)  const { return sensors[i].ground_pressure.get(); }

    /**
     * @brief Set temperature from external source for altitude calibration
     * 
     * @details Allows external temperature source (e.g., digital airspeed sensor) to
     *          override barometer temperature for more accurate altitude calculations.
     *          External temperature may be more representative of ambient conditions.
     * 
     *          Use case:
     *          - Airspeed sensor with integrated temperature probe in airstream
     *          - External high-accuracy temperature sensor
     *          - Overrides internal barometer temperature sensor reading
     * 
     * @param[in] temperature External temperature in degrees Celsius (°C)
     * 
     * @note External temperature used for altitude calculation if recently updated
     * @note Times out after 2 seconds if not refreshed
     * 
     * @see get_external_temperature() to retrieve external temperature
     */
    void set_external_temperature(float temperature);

    /**
     * @brief Get time of last update for primary sensor
     * 
     * @details Returns system time in milliseconds when primary sensor was last
     *          updated with new data. Used for health monitoring and timeout detection.
     * 
     * @return Time of last update in milliseconds (ms) since system boot
     * 
     * @note Compare against AP_HAL::millis() to check data freshness
     * @see healthy() which uses this for timeout detection
     */
    uint32_t get_last_update(void) const { return get_last_update(_primary); }

    /**
     * @brief Get time of last update for specific sensor
     * 
     * @details Returns system time when specified sensor was last updated.
     *          Used for health monitoring, timeout detection, and sensor synchronization.
     * 
     * @param[in] instance Sensor instance number (0 to BARO_MAX_INSTANCES-1)
     * 
     * @return Time of last update in milliseconds (ms) since system boot
     * 
     * @note Used internally for BARO_TIMEOUT_MS health check
     */
    uint32_t get_last_update(uint8_t instance) const { return sensors[instance].last_update_ms; }

    // settable parameters
    static const struct AP_Param::GroupInfo var_info[];

    /**
     * @brief Get external temperature for primary sensor
     * 
     * @details Returns external temperature value if set via set_external_temperature().
     *          Falls back to barometer temperature if external not available or stale.
     * 
     * @return Temperature in degrees Celsius (°C)
     * 
     * @note Returns external temp if updated within 2 seconds, otherwise barometer temp
     * @see set_external_temperature() to provide external temperature
     */
    float get_external_temperature(void) const { return get_external_temperature(_primary); };

    /**
     * @brief Get external temperature for specific sensor
     * 
     * @details Returns external temperature for specified instance if available,
     *          otherwise returns sensor's internal temperature.
     * 
     * @param[in] instance Sensor instance number (0 to BARO_MAX_INSTANCES-1)
     * 
     * @return Temperature in degrees Celsius (°C)
     */
    float get_external_temperature(const uint8_t instance) const;

    /**
     * @brief Set primary barometer sensor override
     * 
     * @details Forces selection of specific sensor as primary, overriding automatic
     *          health-based selection. Saved to EEPROM parameter GND_PRIMARY.
     * 
     * @param[in] primary Sensor instance number (0 to BARO_MAX_INSTANCES-1) to use as primary
     * 
     * @note Automatic selection resumes if specified sensor becomes unhealthy
     * @warning Only use if specific sensor known to be more accurate
     * 
     * @see get_primary() to retrieve current primary sensor
     */
    void set_primary_baro(uint8_t primary) { _primary_baro.set_and_save(primary); };

    /**
     * @brief Set sensor type (air or water pressure)
     * 
     * @details Configures specified sensor as atmospheric (air) or hydrostatic (water)
     *          pressure sensor. Water type used for underwater vehicles (ROVs) for
     *          depth measurement using hydrostatic pressure.
     * 
     * @param[in] instance Sensor instance number (0 to BARO_MAX_INSTANCES-1)
     * @param[in] type     BARO_TYPE_AIR for atmospheric, BARO_TYPE_WATER for underwater
     * 
     * @note Default type is BARO_TYPE_AIR for all sensors
     * @note Water type applies specific gravity correction for freshwater/saltwater
     * 
     * @see baro_type_t for type definitions
     * @see get_type() to retrieve sensor type
     */
    void set_type(uint8_t instance, baro_type_t type) { sensors[instance].type = type; };

    /**
     * @brief Get sensor type (air or water pressure)
     * 
     * @details Returns whether specified sensor configured as atmospheric or
     *          hydrostatic pressure sensor.
     * 
     * @param[in] instance Sensor instance number (0 to BARO_MAX_INSTANCES-1)
     * 
     * @return BARO_TYPE_AIR for atmospheric, BARO_TYPE_WATER for underwater
     * 
     * @see set_type() to configure sensor type
     */
    baro_type_t get_type(uint8_t instance) { return sensors[instance].type; };

    /**
     * @brief Register new sensor instance from backend driver
     * 
     * @details Called by backend drivers during initialization to claim a sensor
     *          instance slot. Allocates next available instance from sensors[] array.
     *          If all BARO_MAX_INSTANCES slots full, triggers panic.
     * 
     *          Registration sequence:
     *          1. Backend detects physical sensor during init()
     *          2. Backend calls register_sensor() to claim slot
     *          3. Frontend returns instance number
     *          4. Backend stores instance number for subsequent data updates
     * 
     * @return Sensor instance number (0 to BARO_MAX_INSTANCES-1) assigned to backend
     * 
     * @note Called only during init() by backend drivers
     * @warning Panics if BARO_MAX_INSTANCES exceeded - increase limit if needed
     * 
     * @see init() for backend initialization sequence
     */
    uint8_t register_sensor(void);

    /**
     * @brief Get number of registered sensor instances
     * 
     * @details Returns count of barometer sensors successfully registered during
     *          init(). Used for sensor iteration and health reporting.
     * 
     * @return Number of registered sensors (0 to BARO_MAX_INSTANCES)
     * 
     * @note Returns 0 before init() or if no sensors detected
     * @note Typical configurations: 1-2 sensors
     */
    uint8_t num_instances(void) const { return _num_sensors; }

    /**
     * @brief Set barometer drift altitude offset
     * 
     * @details Sets manual altitude offset to compensate for barometer drift over time.
     *          Useful for long-duration flights where barometer calibration drifts due
     *          to temperature changes or sensor aging.
     * 
     * @param[in] alt Altitude offset in meters (m) to add to barometer reading
     *                Positive = increases reported altitude
     * 
     * @note Saves to GND_ALT_OFFSET parameter
     * @note Use sparingly - indicates sensor calibration drift
     * 
     * @see get_baro_drift_offset() to retrieve current offset
     */
    void set_baro_drift_altitude(float alt) { _alt_offset.set(alt); }

    /**
     * @brief Get current barometer drift altitude offset
     * 
     * @details Returns active altitude offset applied for barometer drift compensation.
     * 
     * @return Altitude offset in meters (m) added to barometer reading
     * 
     * @note Offset loaded from GND_ALT_OFFSET parameter during initialization
     * @see set_baro_drift_altitude() to set offset
     */
    float get_baro_drift_offset(void) const { return _alt_offset_active; }

    /**
     * @brief Simple underwater atmospheric model for ROV depth calculation
     * 
     * @details Calculates hydrostatic pressure parameters for underwater vehicles using
     *          simplified model. Returns density ratio, pressure ratio, and temperature
     *          ratio relative to surface conditions.
     * 
     *          Used for underwater barometer (BARO_TYPE_WATER) depth estimation.
     *          Applies specific gravity correction for freshwater (1.0) or saltwater (1.024).
     * 
     * @param[in]  alt   Depth below surface in meters (m), negative = underwater
     * @param[out] rho   Density ratio (dimensionless)
     * @param[out] delta Pressure ratio (dimensionless)
     * @param[out] theta Temperature ratio (dimensionless)
     * 
     * @note Used internally for BARO_TYPE_WATER sensors
     * @note Specific gravity configured via GND_SPEC_GRAV parameter
     */
    static void SimpleUnderWaterAtmosphere(float alt, float &rho, float &delta, float &theta);

    /**
     * @brief Set pressure correction from temperature calibration
     * 
     * @details Applies pressure correction offset from AP_TempCalibration subsystem.
     *          Compensates for temperature-dependent sensor errors discovered during
     *          temperature calibration procedure.
     * 
     * @param[in] instance     Sensor instance number (0 to BARO_MAX_INSTANCES-1)
     * @param[in] p_correction Pressure correction in Pascals (Pa) to add to reading
     * 
     * @note Correction applied before altitude calculation
     * @note Positive correction increases reported pressure
     * 
     * @see get_pressure_correction() to retrieve correction value
     */
    void set_pressure_correction(uint8_t instance, float p_correction);

    /**
     * @brief Get outlier rejection filter range parameter
     * 
     * @details Returns valid pressure deviation range from mean for multi-sensor
     *          outlier rejection. Sensors deviating beyond this range flagged as outliers.
     * 
     * @return Filter range parameter value
     * 
     * @note Used for multi-sensor consistency checking and outlier rejection
     */
    uint8_t get_filter_range() const { return _filter_range; }

    /**
     * @brief Set LOG_BITMASK bit indicating barometer logging enabled
     * 
     * @details Configures which bit in LOG_BITMASK parameter enables barometer logging.
     *          Called during vehicle initialization to link barometer logging to bitmask.
     * 
     * @param[in] bit LOG_BITMASK bit number for barometer logging enable
     * 
     * @note Called once during vehicle initialization
     * @see should_log() to check if logging currently enabled
     */
    void set_log_baro_bit(uint32_t bit) { _log_baro_bit = bit; }

    /**
     * @brief Check if barometer logging currently enabled
     * 
     * @details Returns true if barometer logging enabled in LOG_BITMASK parameter.
     *          Used internally to conditionally log barometer data.
     * 
     * @return true if logging enabled, false otherwise
     * 
     * @note Checks LOG_BITMASK bit configured via set_log_baro_bit()
     */
    bool should_log() const;

    /**
     * @brief Get semaphore for thread-safe barometer access
     * 
     * @details Returns HAL_Semaphore protecting sensors[] array and internal state.
     *          External threads must lock this semaphore before accessing barometer
     *          data to prevent race conditions with update() thread.
     * 
     *          Usage pattern:
     *          @code
     *          WITH_SEMAPHORE(AP::baro().get_semaphore());
     *          // Safe to access barometer data here
     *          float pressure = AP::baro().get_pressure();
     *          @endcode
     * 
     * @return Reference to HAL_Semaphore for barometer data protection
     * 
     * @note Backend drivers lock _rsem when pushing data to frontend
     * @warning Must lock semaphore for multi-threaded access to prevent corruption
     */
    HAL_Semaphore &get_semaphore(void) {
        return _rsem;
    }

#if AP_BARO_MSP_ENABLED
    /**
     * @brief Handle barometer data from MSP protocol
     * 
     * @details Processes barometer data received via MSP (MultiWii Serial Protocol)
     *          from MSP OSD devices or flight controllers. Updates virtual MSP
     *          barometer sensor instance with received pressure and temperature.
     * 
     * @param[in] pkt MSP barometer data packet containing pressure and temperature
     * 
     * @note Requires AP_BARO_MSP_ENABLED feature flag
     * @note Creates virtual sensor instance for MSP barometer data
     * @see AP_MSP for MSP protocol implementation
     */
    void handle_msp(const MSP::msp_baro_data_message_t &pkt);
#endif

#if AP_BARO_EXTERNALAHRS_ENABLED
    /**
     * @brief Handle barometer data from external AHRS system
     * 
     * @details Processes barometer data received from external AHRS systems
     *          (e.g., VectorNav, Lord Microstrain). Updates virtual sensor instance
     *          with external barometer measurements.
     * 
     * @param[in] pkt External AHRS barometer data packet
     * 
     * @note Requires AP_BARO_EXTERNALAHRS_ENABLED feature flag
     * @note Creates virtual sensor instance for external AHRS barometer
     * @see AP_ExternalAHRS for external AHRS integration
     */
    void handle_external(const AP_ExternalAHRS::baro_data_message_t &pkt);
#endif

    /**
     * @enum Options
     * @brief Barometer subsystem option flags
     * 
     * @details Bitmask options to configure barometer behavior. Configured via
     *          GND_OPTIONS parameter.
     */
    enum Options : uint16_t {
        TreatMS5611AsMS5607 = (1U << 0U),  ///< Treat MS5611 sensor as MS5607 for compatibility workaround
    };

    /**
     * @brief Check if specific option flag enabled
     * 
     * @details Tests whether specified option bit set in GND_OPTIONS parameter.
     * 
     * @param[in] option Option flag to test from Options enum
     * 
     * @return true if option enabled, false otherwise
     * 
     * @note Used internally to enable optional behaviors or workarounds
     * @see Options enum for available option flags
     */
    bool option_enabled(const Options option) const
    {
        return (uint16_t(_options.get()) & uint16_t(option)) != 0;
    }

private:
    /// Singleton instance pointer for AP_Baro
    static AP_Baro *_singleton;
    
    /// Number of registered backend drivers (up to BARO_MAX_DRIVERS)
    uint8_t _num_drivers;
    /// Array of backend driver pointers, each providing one or more sensor instances
    AP_Baro_Backend *drivers[BARO_MAX_DRIVERS];

    /// Number of registered sensor instances (up to BARO_MAX_INSTANCES)
    uint8_t _num_sensors;

    /// Current primary sensor index selected for flight control use
    uint8_t _primary;

    /// LOG_BITMASK bit number indicating barometer logging enabled
    uint32_t _log_baro_bit = -1;

    /// True if init() completed successfully
    bool init_done;

    /// Bitmask tracking which instances are MSP virtual sensors
    uint8_t msp_instance_mask;

    // bitmask values for GND_PROBE_EXT
    enum {
        PROBE_BMP085=(1<<0),
        PROBE_BMP280=(1<<1),
        PROBE_MS5611=(1<<2),
        PROBE_MS5607=(1<<3),
        PROBE_MS5637=(1<<4),
        PROBE_FBM320=(1<<5),
        PROBE_DPS280=(1<<6),
        PROBE_LPS25H=(1<<7),
        PROBE_KELLER=(1<<8),
        PROBE_MS5837=(1<<9),
        PROBE_BMP388=(1<<10),
        PROBE_SPL06 =(1<<11),
        PROBE_MSP   =(1<<12),
        PROBE_BMP581=(1<<13),
        PROBE_AUAV  =(1<<14),
    };
    
#if HAL_BARO_WIND_COMP_ENABLED
    /**
     * @class WindCoeff
     * @brief Wind compensation coefficients for barometer pressure correction
     * 
     * @details Stores per-axis dynamic pressure correction coefficients to compensate
     *          for wind-induced static pressure errors. Each axis has separate coefficients
     *          for positive and negative directions to handle asymmetric airflow effects.
     * 
     *          Coefficients define ratio of static pressure rise to dynamic pressure:
     *          - Positive direction (xp, yp, zp): forward, right, up
     *          - Negative direction (xn, yn, zn): backward, left, down
     * 
     *          Correction formula: ΔP = coeff * 0.5 * ρ * V²
     *          where ρ = air density, V = airspeed component on axis
     */
    class WindCoeff {
    public:
        static const struct AP_Param::GroupInfo var_info[];

        AP_Int8  enable; ///< Enable wind compensation for this barometer (0=disabled, 1=enabled)
        AP_Float xp;     ///< Static pressure rise ratio for forward flight (body +X axis)
        AP_Float xn;     ///< Static pressure rise ratio for backward flight (body -X axis)
        AP_Float yp;     ///< Static pressure rise ratio for rightward flight (body +Y axis)
        AP_Float yn;     ///< Static pressure rise ratio for leftward flight (body -Y axis)
        AP_Float zp;     ///< Static pressure rise ratio for upward flight (body +Z axis, down in NED)
        AP_Float zn;     ///< Static pressure rise ratio for downward flight (body -Z axis, up in NED)
    };
#endif

    /**
     * @struct sensor
     * @brief Per-instance barometer sensor data and state
     * 
     * @details Contains all data and state for a single barometer sensor instance.
     *          Each registered sensor gets one sensor struct in sensors[] array.
     */
    struct sensor {
        uint32_t last_update_ms;        ///< Last update time in milliseconds since boot
        uint32_t last_change_ms;        ///< Last update time in ms that included pressure or temperature change
        float pressure;                 ///< Current pressure in Pascals (Pa)
        float temperature;              ///< Current temperature in degrees Celsius (°C)
        float altitude;                 ///< Calculated altitude in meters (m) relative to calibration
        AP_Float ground_pressure;       ///< Calibration ground pressure in Pascals (Pa), EEPROM parameter
        float p_correction;             ///< Pressure correction from calibration in Pascals (Pa)
        baro_type_t type;               ///< Sensor type: BARO_TYPE_AIR (0) or BARO_TYPE_WATER (1)
        bool healthy;                   ///< True if sensor receiving valid updates within timeout
        bool alt_ok;                    ///< True if calculated altitude is valid
        bool calibrated;                ///< True if sensor calibrated successfully
        AP_Int32 bus_id;                ///< Hardware bus identifier (SPI/I2C/CAN) for sensor identification
#if HAL_BARO_WIND_COMP_ENABLED
        WindCoeff wind_coeff;           ///< Wind compensation coefficient parameters per sensor
        Vector3f dynamic_pressure;      ///< Calculated dynamic pressure correction vector in Pascals (Pa)
#endif
#if AP_BARO_THST_COMP_ENABLED
        AP_Float mot_scale;             ///< Thrust-based pressure scaling coefficient for propeller effects
#endif
#if (HAL_BARO_WIND_COMP_ENABLED || AP_BARO_THST_COMP_ENABLED)
        float corrected_pressure;       ///< Pressure after wind and/or thrust compensation in Pascals (Pa)
#endif
    } sensors[BARO_MAX_INSTANCES];      ///< Array of sensor instances, up to BARO_MAX_INSTANCES (3)

    /// Altitude offset for barometer drift compensation (GND_ALT_OFFSET parameter) in meters
    AP_Float                            _alt_offset;
    /// Active altitude offset applied to altitude calculations in meters
    float                               _alt_offset_active;
    /// Field elevation above mean sea level (GND_ABS_PRESS parameter) in meters
    AP_Float                            _field_elevation;
    /// Active field elevation used for AMSL altitude calculation in meters
    float                               _field_elevation_active;
    /// Last time field elevation was updated in milliseconds
    uint32_t                            _field_elevation_last_ms;
    /// User-specified primary sensor override (GND_PRIMARY parameter), 0=auto-select
    AP_Int8                             _primary_baro;
    /// I2C bus number designated as "external" for probing external sensors
    AP_Int8                             _ext_bus;
    /// Temperature from external source (e.g., airspeed sensor) in degrees Celsius
    float                               _external_temperature;
    /// Last time external temperature was updated in milliseconds
    uint32_t                            _last_external_temperature_ms;
    /// Derivative filter for climb rate calculation, 7-sample window
    DerivativeFilterFloat_Size7         _climb_rate_filter;
    /// Specific gravity of fluid for underwater ROV (GND_SPEC_GRAV): 1.0=freshwater, 1.024=saltwater
    AP_Float                            _specific_gravity;
    /// User override ground temperature for EAS2TAS calculation (GND_TEMP parameter) in °C
    AP_Float                            _user_ground_temperature;
    /// Best-estimate ground temperature from all available sources in °C
    float                               _guessed_ground_temperature;

    /// Last time GCS notified of ground pressure reference change in milliseconds
    uint32_t                            _last_notify_ms;

    /**
     * @brief Check if I2C driver already probed at bus/address
     * 
     * @param[in] bus_num I2C bus number
     * @param[in] address I2C device address
     * 
     * @return true if driver already probed, false otherwise
     * 
     * @note Prevents duplicate driver initialization
     */
    bool _have_i2c_driver(uint8_t bus_num, uint8_t address) const;
    
    /**
     * @brief Add backend driver to drivers array
     * 
     * @param[in] backend Pointer to backend driver to add
     * 
     * @return true if added successfully, false if drivers array full
     */
    bool _add_backend(AP_Baro_Backend *backend);
    
    /**
     * @brief Probe for I2C barometer sensors
     * 
     * @details Scans I2C buses for supported barometer sensors based on
     *          GND_PROBE_EXT and GND_EXT_BUS parameters. Creates backend
     *          drivers for detected sensors.
     */
    void _probe_i2c_barometers(void);
    
    /// Outlier rejection filter range (GND_FILTER_RANGE parameter)
    AP_Int8                            _filter_range;
    /// External sensor probe enable bitmask (GND_PROBE_EXT parameter)
    AP_Int32                           _baro_probe_ext;

#ifndef HAL_BUILD_AP_PERIPH
    /// Maximum altitude error for arming checks (GND_ALT_ERR_MAX parameter) in meters
    AP_Float                           _alt_error_max;
#endif

    /// Option flags bitmask (GND_OPTIONS parameter)
    AP_Int16                           _options;

    /// Semaphore protecting sensors[] array for thread-safe API access
    HAL_Semaphore                      _rsem;

#if HAL_BARO_WIND_COMP_ENABLED
    /**
     * @brief Calculate wind-induced pressure correction
     * 
     * @details Returns pressure correction based on wind compensation coefficients
     *          (GND_WCOEF_* parameters) and current airspeed vector from AHRS.
     *          Applies per-axis dynamic pressure corrections.
     * 
     * @param[in] instance Sensor instance number
     * 
     * @return Pressure correction in Pascals (Pa)
     * 
     * @note Requires valid airspeed from AHRS
     * @note Only active if wind_coeff.enable set for instance
     */
    float wind_pressure_correction(uint8_t instance);
#endif

#if AP_BARO_THST_COMP_ENABLED
    /**
     * @brief Calculate thrust-induced pressure correction
     * 
     * @details Returns pressure correction for propeller/motor-induced pressure
     *          changes based on motor thrust output.
     * 
     * @param[in] instance Sensor instance number
     * 
     * @return Pressure correction in Pascals (Pa)
     * 
     * @note Scales with motor output to compensate for propwash effects
     */
    float thrust_pressure_correction(uint8_t instance);
#endif

    /**
     * @brief Write barometer data to dataflash log
     * 
     * @details Logs all barometer instances to dataflash if logging enabled.
     *          Called from update() when should_log() returns true.
     * 
     * @note Generates BARO log messages for each instance
     */
    void Write_Baro(void);
    
    /**
     * @brief Write specific barometer instance to log
     * 
     * @param[in] time_us     Timestamp in microseconds
     * @param[in] baro_instance Sensor instance number to log
     * 
     * @note Helper function called by Write_Baro()
     */
    void Write_Baro_instance(uint64_t time_us, uint8_t baro_instance);

    /**
     * @brief Update active field elevation from parameter
     * 
     * @details Copies GND_ABS_PRESS parameter to active field elevation
     *          if parameter changed. Used for AMSL altitude calculation.
     * 
     * @note Called periodically from update()
     */
    void update_field_elevation();

    /**
     * @brief Calculate altitude difference using 1976 standard atmosphere (extended model)
     * 
     * @param[in] base_pressure Reference pressure in Pascals
     * @param[in] pressure      Current pressure in Pascals
     * 
     * @return Altitude difference in meters
     * 
     * @note Uses full 1976 standard atmosphere with multiple layers
     * @note More accurate than simple model, especially at high altitude
     */
    float get_altitude_difference_extended(float base_pressure, float pressure) const;
    
    /**
     * @brief Calculate EAS to TAS using extended atmosphere model
     * 
     * @param[in] pressure Current pressure in Pascals
     * 
     * @return EAS to TAS scaling factor (dimensionless)
     * 
     * @note Uses 1976 standard atmosphere model
     */
    float get_EAS2TAS_extended(float pressure) const;
    
    /**
     * @brief Get temperature for altitude layer in standard atmosphere
     * 
     * @param[in] alt Altitude in meters
     * @param[in] idx Atmosphere layer index
     * 
     * @return Temperature in Kelvin
     * 
     * @note Helper for extended atmosphere model calculations
     */
    static float get_temperature_by_altitude_layer(float alt, int8_t idx);

    /**
     * @brief Calculate altitude difference using simple atmosphere model
     * 
     * @param[in] base_pressure Reference pressure in Pascals
     * @param[in] pressure      Current pressure in Pascals
     * 
     * @return Altitude difference in meters
     * 
     * @note Simplified exponential model, faster but less accurate
     * @note Used when AP_BARO_1976_STANDARD_ATMOSPHERE_ENABLED not defined
     */
    float get_altitude_difference_simple(float base_pressure, float pressure) const;
    
    /**
     * @brief Calculate EAS to TAS using simple atmosphere model
     * 
     * @param[in] altitude Altitude in meters
     * @param[in] pressure Pressure in Pascals
     * 
     * @return EAS to TAS scaling factor (dimensionless)
     * 
     * @note Simplified model for computational efficiency
     */
    float get_EAS2TAS_simple(float altitude, float pressure) const;
};

/**
 * @namespace AP
 * @brief ArduPilot core singleton accessor namespace
 */
namespace AP {
    /**
     * @brief Get reference to barometer singleton instance
     * 
     * @details Provides global access to AP_Baro singleton for barometer operations.
     *          Preferred method for accessing barometer subsystem throughout codebase.
     * 
     *          Usage example:
     *          @code
     *          float altitude = AP::baro().get_altitude();
     *          float pressure = AP::baro().get_pressure();
     *          @endcode
     * 
     * @return Reference to AP_Baro singleton instance
     * 
     * @note Singleton initialized during vehicle startup
     * @see AP_Baro::get_singleton() for alternative singleton access
     */
    AP_Baro &baro();
};
