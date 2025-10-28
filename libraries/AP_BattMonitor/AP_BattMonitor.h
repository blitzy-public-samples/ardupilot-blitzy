/**
 * @file AP_BattMonitor.h
 * @brief ArduPilot battery monitoring frontend managing multiple battery monitor backend instances
 * 
 * @details This file defines the AP_BattMonitor singleton frontend class that manages up to 9
 *          battery monitor instances. Each instance uses a type-specific backend driver for
 *          voltage/current measurement via analog sensors, SMBus intelligent batteries,
 *          DroneCAN, scripting, or other interfaces. The frontend provides unified access,
 *          periodic polling (~10Hz), failsafe evaluation, and MAVLink/logging integration.
 * 
 *          Multi-instance architecture allows monitoring primary flight battery, backup battery,
 *          payload power, generator, solar panels, etc. Primary instance (index 0) is used for
 *          vehicle control decisions and failsafe triggers.
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

#include "AP_BattMonitor_config.h"

#if AP_BATTERY_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_TemperatureSensor/AP_TemperatureSensor_config.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include "AP_BattMonitor_Params.h"

/// Primary battery monitor instance index (first monitor is always primary)
#define AP_BATT_PRIMARY_INSTANCE            0

/// Default battery serial number when not set (-1 indicates unconfigured)
#define AP_BATT_SERIAL_NUMBER_DEFAULT       -1

/// Battery monitor communication timeout in milliseconds (5 seconds)
#define AP_BATT_MONITOR_TIMEOUT             5000

/// Battery internal resistance estimation time constant 1 (fast response, 0.5 seconds)
#define AP_BATT_MONITOR_RES_EST_TC_1        0.5f
/// Battery internal resistance estimation time constant 2 (slow response, 0.1 seconds for stability)
#define AP_BATT_MONITOR_RES_EST_TC_2        0.1f

/// Maximum number of battery cells supported (14 cells for boards with >1MB flash, 12 cells otherwise)
#if HAL_PROGRAM_SIZE_LIMIT_KB > 1024
#define AP_BATT_MONITOR_CELLS_MAX           14
#else
#define AP_BATT_MONITOR_CELLS_MAX           12
#endif

// Forward declarations of battery monitor backend driver classes
class AP_BattMonitor_Backend;
class AP_BattMonitor_Analog;
class AP_BattMonitor_SMBus;
class AP_BattMonitor_SMBus_Solo;
class AP_BattMonitor_SMBus_Generic;
class AP_BattMonitor_SMBus_Maxell;
class AP_BattMonitor_SMBus_Rotoye;
class AP_BattMonitor_DroneCAN;
class AP_BattMonitor_Generator;
class AP_BattMonitor_INA2XX;
class AP_BattMonitor_INA239;
class AP_BattMonitor_LTC2946;
class AP_BattMonitor_Torqeedo;
class AP_BattMonitor_FuelLevel_Analog;
class AP_BattMonitor_EFI;
class AP_BattMonitor_Scripting;

/**
 * @class AP_BattMonitor
 * @brief Battery monitoring frontend singleton managing multiple battery monitor backend instances
 * 
 * @details AP_BattMonitor is the main frontend class for battery monitoring in ArduPilot. It manages
 *          an array of up to 9 battery monitor instances (AP_BATT_MONITOR_MAX_INSTANCES), each with
 *          its own backend driver, state structure, and parameters.
 * 
 *          Architecture:
 *          - Frontend (this class): Singleton providing unified API, failsafe coordination, logging
 *          - Backend drivers: Type-specific implementations (Analog, SMBus, DroneCAN, INA2XX, etc.)
 *          - Per-instance state: Voltage, current, capacity, temperature, health status
 *          - Parameters: Stored per instance via AP_BattMonitor_Params
 * 
 *          Multi-Instance Usage:
 *          - Primary instance (index 0): Main flight battery, used for vehicle control decisions
 *          - Additional instances: Backup batteries, payload power, generators, solar panels
 *          - Each instance operates independently with separate health monitoring and failsafes
 * 
 *          Lifecycle:
 *          1. Construction with logging bit mask and failsafe callback
 *          2. init() - Detects and instantiates backend drivers based on parameters
 *          3. read() - Called periodically (~10Hz) from vehicle main loop to update all instances
 *          4. Failsafe checking on each read() with vehicle callback on state changes
 * 
 *          Failsafe System:
 *          - Hierarchical failsafe levels: None → Unhealthy → Low → Critical
 *          - Failsafes can only progress to higher severity (never backwards)
 *          - Vehicle-specific handler called with failsafe priority and instance
 *          - Tracks highest failsafe level reached across all instances
 * 
 *          Thread Safety:
 *          - read() and query methods called from main thread
 *          - Backend update methods may be called from scheduler callbacks
 *          - State updates are not explicitly protected (single-threaded access assumed)
 * 
 * @note The primary battery instance (index 0) should be the main flight battery
 * @note read() must be called at approximately 10Hz from the vehicle main loop
 * @warning Failsafe progression is unidirectional - system can only move to higher severity levels
 * @warning Many query methods have WARN_IF_UNUSED attribute - return values must be checked
 */
class AP_BattMonitor
{
    friend class AP_BattMonitor_Backend;
    friend class AP_BattMonitor_Analog;
    friend class AP_BattMonitor_SMBus;
    friend class AP_BattMonitor_SMBus_Solo;
    friend class AP_BattMonitor_SMBus_Generic;
    friend class AP_BattMonitor_SMBus_Maxell;
    friend class AP_BattMonitor_SMBus_Rotoye;
    friend class AP_BattMonitor_DroneCAN;
    friend class AP_BattMonitor_Sum;
    friend class AP_BattMonitor_FuelFlow;
    friend class AP_BattMonitor_FuelLevel_PWM;
    friend class AP_BattMonitor_Generator;
    friend class AP_BattMonitor_EFI;
    friend class AP_BattMonitor_INA2XX;
    friend class AP_BattMonitor_INA239;
    friend class AP_BattMonitor_LTC2946;
    friend class AP_BattMonitor_AD7091R5;
    friend class AP_BattMonitor_INA3221;

    friend class AP_BattMonitor_Torqeedo;
    friend class AP_BattMonitor_FuelLevel_Analog;
    friend class AP_BattMonitor_Synthetic_Current;
    friend class AP_BattMonitor_Scripting;

public:

    /**
     * @enum Failsafe
     * @brief Battery failsafe severity levels sorted by increasing severity
     * 
     * @details Failsafe levels are strictly ordered to ensure proper failsafe progression.
     *          The system can only transition to higher severity levels (e.g., Low → Critical)
     *          and never backwards. This prevents oscillation and ensures appropriate vehicle
     *          response to degrading battery conditions.
     * 
     *          Triggering conditions (configured via parameters):
     *          - Unhealthy: Communication timeout or sensor failure
     *          - Low: Voltage/capacity falls below warning threshold
     *          - Critical: Voltage/capacity reaches critical threshold requiring immediate action
     * 
     * @warning Failsafe levels are ordered by severity - do not reorder enum values
     */
    enum class Failsafe : uint8_t {
        None = 0,      ///< No failsafe condition, battery operating normally
        Unhealthy,     ///< Battery monitor communication failure or unhealthy sensor readings
        Low,           ///< Battery voltage or capacity below warning threshold
        Critical       ///< Battery voltage or capacity critically low, immediate action required
    };

    /// Battery monitor driver types (aliased from AP_BattMonitor_Params::Type for convenience)
    using Type = AP_BattMonitor_Params::Type;

    /**
     * @brief Function pointer typedef for vehicle-specific battery failsafe handler callback
     * 
     * @details This callback is invoked when a battery failsafe condition changes. The vehicle
     *          code implements this to take appropriate action (mode changes, warnings, landing, etc.)
     * 
     * @param message Human-readable failsafe description for logging/notification
     * @param failsafe_priority Failsafe severity level from vehicle's priority array
     */
    FUNCTOR_TYPEDEF(battery_failsafe_handler_fn_t, void, const char *, const int8_t);

    /**
     * @brief Construct AP_BattMonitor frontend singleton
     * 
     * @param[in] log_battery_bit Logger bitmask for battery data logging
     * @param[in] battery_failsafe_handler_fn Vehicle-specific callback for failsafe events
     * @param[in] failsafe_priorities Array of failsafe priority levels sorted highest to lowest,
     *                                 -1 sentinel indicates end of array
     * 
     * @note Constructor should be called once during vehicle initialization
     */
    AP_BattMonitor(uint32_t log_battery_bit, battery_failsafe_handler_fn_t battery_failsafe_handler_fn, const int8_t *failsafe_priorities);

    /* Do not allow copies */
    CLASS_NO_COPY(AP_BattMonitor);

    /**
     * @brief Get the AP_BattMonitor singleton instance
     * 
     * @return Pointer to the singleton AP_BattMonitor instance, or nullptr if not constructed
     * 
     * @note Singleton pattern provides global access to battery monitoring across vehicle code
     * @see AP::battery() for convenient namespace accessor
     */
    static AP_BattMonitor *get_singleton() {
        return _singleton;
    }

    /**
     * @struct cells
     * @brief Individual battery cell voltages array structure
     * 
     * @details Stores per-cell voltage measurements in millivolts for multi-cell battery packs.
     *          Cell count varies by battery chemistry and configuration. MAVLink battery status
     *          message supports up to 10 cells; this structure supports up to 14 cells on larger
     *          boards (12 cells on smaller boards due to flash constraints).
     * 
     * @note Cell voltages are in millivolts (mV), not volts
     * @note Unused cells contain zero or invalid values
     */
    struct cells {
        uint16_t cells[AP_BATT_MONITOR_CELLS_MAX];  ///< Array of cell voltages in millivolts (mV)
    };

    /**
     * @struct BattMonitor_State
     * @brief Comprehensive battery state maintained per instance by backend drivers
     * 
     * @details This structure contains all monitored battery parameters and health status for a single
     *          battery instance. Backend drivers populate this structure during their update cycle,
     *          and the frontend provides read access through query methods.
     * 
     *          Updated by backend drivers at varying rates depending on sensor type:
     *          - Analog sensors: Every read() call (~10Hz)
     *          - SMBus batteries: Periodic I2C transactions (typically 1-10Hz)
     *          - DroneCAN: Message arrival rate from CAN bus
     *          - Scripting: Script update frequency
     * 
     * @note All float values use standard SI units unless otherwise specified
     * @note Timestamps use system time (milliseconds or microseconds as specified)
     * @warning Not all backends provide all fields - check validity flags
     */
    struct BattMonitor_State {
        cells       cell_voltages;             ///< Individual battery cell voltages in millivolts (mV)
        float       voltage;                   ///< Total battery pack voltage in volts (V)
        float       current_amps;              ///< Instantaneous current draw in amperes (A), positive for discharge
        float       consumed_mah;              ///< Total current draw in milliamp-hours (mAh) since startup or reset
        float       consumed_wh;               ///< Total energy consumed in watt-hours (Wh) since startup or reset
        uint32_t    last_time_micros;          ///< Timestamp when voltage and current were last updated in microseconds (µs)
        uint32_t    low_voltage_start_ms;      ///< Timestamp when voltage first dropped below low threshold in milliseconds (ms)
        uint32_t    critical_voltage_start_ms; ///< Timestamp when voltage first dropped below critical threshold in milliseconds (ms)
        float       temperature;               ///< Battery temperature in degrees Celsius (°C) from internal sensor
#if AP_TEMPERATURE_SENSOR_ENABLED
        bool        temperature_external_use;  ///< True when using external temperature sensor instead of internal
        float       temperature_external;      ///< Battery temperature from external sensor in degrees Celsius (°C)
#endif
        uint32_t    temperature_time;          ///< Timestamp of last temperature update in milliseconds (ms)
        float       voltage_resting_estimate;  ///< Estimated open-circuit voltage in volts (V) with sag removed based on current and resistance
        float       resistance;                ///< Battery internal resistance in ohms (Ω), estimated from voltage sag under load
        Failsafe failsafe;                     ///< Current failsafe state for this battery instance
        bool        healthy;                   ///< True when battery monitor is communicating correctly and readings are valid
        uint32_t    last_healthy_ms;           ///< Timestamp when monitor was last healthy in milliseconds (ms)
        bool        is_powering_off;           ///< True when power button has commanded power off (MAVLink POWER_STATUS)
        bool        powerOffNotified;          ///< True after powering off notification sent (prevents repeated notifications)
        uint32_t    time_remaining;            ///< Estimated remaining battery time in seconds (s) based on current draw
        bool        has_time_remaining;        ///< True when time_remaining field contains valid data
        uint8_t     state_of_health_pct;       ///< Battery state of health (SoH) as percentage (0-100%) indicating capacity degradation
        bool        has_state_of_health_pct;   ///< True when state_of_health_pct field contains valid data
        uint8_t     instance;                  ///< Instance number (0-8) identifying this backend within the frontend array
        Type        type;                      ///< Backend driver type actually allocated (may differ from configured type if unavailable)
        const struct AP_Param::GroupInfo *var_info;  ///< Pointer to parameter metadata for this instance
    };

    /// Array of parameter metadata pointers for backend instances (shared across all instances of same type)
    static const struct AP_Param::GroupInfo *backend_var_info[AP_BATT_MONITOR_MAX_INSTANCES];

    /**
     * @brief Get the number of configured battery monitor instances
     * 
     * @return Number of battery instances (0-9) that have been successfully initialized
     * 
     * @note Returns the count of instances with configured types, not necessarily all healthy
     */
    uint8_t num_instances(void) const { return _num_instances; }

    /**
     * @brief Detect and initialize all configured battery monitor backend drivers
     * 
     * @details Iterates through parameter configuration and instantiates appropriate backend
     *          driver for each configured instance based on BATT*_MONITOR parameter. Backends
     *          are created for types including Analog, SMBus, DroneCAN, INA2xx, Scripting, etc.
     * 
     *          Called once during vehicle initialization before scheduler starts. Backends
     *          perform sensor detection and register periodic callbacks if needed.
     * 
     * @note Marked with __INITFUNC__ to place in initialization-only flash section on some platforms
     * @warning Must be called during vehicle init phase, before read() is called
     */
    __INITFUNC__ void init();

    /**
     * @brief Read battery voltage and current for all configured instances
     * 
     * @details Updates state for all battery instances by calling backend-specific read methods.
     *          Performs failsafe evaluation after each update and invokes vehicle failsafe callback
     *          if battery conditions have degraded. Should be called at approximately 10Hz from
     *          vehicle main loop for consistent monitoring and failsafe response.
     * 
     * @note Must be called at ~10Hz from vehicle main loop
     * @note Backends may use periodic callbacks for higher-rate sensor polling internally
     * @see check_failsafes()
     */
    void read();

    /**
     * @brief Check if a specific battery monitor instance is healthy
     * 
     * @param[in] instance Battery instance number (0-8)
     * @return True if instance is communicating correctly and providing valid readings
     * 
     * @note Healthy status indicates communication success, not battery charge state
     * @note Returns false for unconfigured instances
     */
    bool healthy(uint8_t instance) const;

    /**
     * @brief Check if all configured battery monitor instances are healthy
     * 
     * @return True only if all configured instances are healthy
     * 
     * @note Used for pre-arm checks to ensure all battery monitoring is functional
     */
    bool healthy() const;

    /**
     * @brief Get battery pack voltage for specified instance
     * 
     * @param[in] instance Battery instance number (0-8)
     * @return Battery voltage in volts (V), 0.0 if instance invalid or unhealthy
     * 
     * @note Returns instantaneous measured voltage including sag under load
     * @see voltage_resting_estimate() for sag-compensated voltage
     */
    float voltage(uint8_t instance) const;
    
    /**
     * @brief Get primary battery pack voltage
     * 
     * @return Primary battery (instance 0) voltage in volts (V)
     */
    float voltage() const { return voltage(AP_BATT_PRIMARY_INSTANCE); }

    /**
     * @brief Get battery voltage for GCS display, optionally resistance-compensated
     * 
     * @param[in] instance Battery instance number (0-8)
     * @return Battery voltage in volts (V), may be resistance-compensated based on parameters
     * 
     * @note Returned voltage may differ from voltage() if BATT_ESC_INDEX compensation is enabled
     */
    float gcs_voltage(uint8_t instance) const;
    
    /**
     * @brief Get primary battery voltage for GCS display
     * 
     * @return Primary battery voltage in volts (V), may be resistance-compensated
     */
    float gcs_voltage(void) const { return gcs_voltage(AP_BATT_PRIMARY_INSTANCE); }

    /**
     * @brief Get estimated open-circuit voltage with sag removed
     * 
     * @param[in] instance Battery instance number (0-8)
     * @return Estimated resting voltage in volts (V) based on current draw and resistance
     * 
     * @details Compensates for voltage sag under load using estimated internal resistance.
     *          Resting estimate = measured voltage + (current * resistance). Always greater
     *          than or equal to measured voltage since discharge current is positive.
     * 
     * @note Provides more stable voltage estimate during high current draw
     * @note Requires valid resistance estimate (updated during flight)
     */
    float voltage_resting_estimate(uint8_t instance) const;
    
    /**
     * @brief Get primary battery estimated open-circuit voltage
     * 
     * @return Primary battery resting voltage estimate in volts (V)
     */
    float voltage_resting_estimate() const { return voltage_resting_estimate(AP_BATT_PRIMARY_INSTANCE); }

    /**
     * @brief Get instantaneous battery current draw
     * 
     * @param[out] current Current draw in amperes (A), positive for discharge
     * @param[in]  instance Battery instance number (0-8), defaults to primary
     * @return True if current reading is valid and was written to output parameter
     * 
     * @note Current is positive during discharge, negative during charge
     * @warning Return value must be checked (WARN_IF_UNUSED) - output invalid if returns false
     */
    bool current_amps(float &current, const uint8_t instance = AP_BATT_PRIMARY_INSTANCE) const WARN_IF_UNUSED;

    /**
     * @brief Get total current consumed since startup or reset
     * 
     * @param[out] mah Total current draw in milliamp-hours (mAh)
     * @param[in]  instance Battery instance number (0-8), defaults to primary
     * @return True if consumed capacity is valid and was written to output parameter
     * 
     * @note Integrates current over time from startup or last reset_remaining() call
     * @warning Return value must be checked (WARN_IF_UNUSED) - output invalid if returns false
     * @see reset_remaining() to reset consumed capacity
     */
    bool consumed_mah(float &mah, const uint8_t instance = AP_BATT_PRIMARY_INSTANCE) const WARN_IF_UNUSED;

    /**
     * @brief Get total energy consumed since startup or reset
     * 
     * @param[out] wh Total energy consumption in watt-hours (Wh)
     * @param[in]  instance Battery instance number (0-8), defaults to primary
     * @return True if consumed energy is valid and was written to output parameter
     * 
     * @note Integrates power (voltage * current) over time
     * @warning Return value must be checked (WARN_IF_UNUSED) - output invalid if returns false
     */
    bool consumed_wh(float&wh, const uint8_t instance = AP_BATT_PRIMARY_INSTANCE) const WARN_IF_UNUSED;

    /**
     * @brief Get remaining battery capacity as percentage
     * 
     * @param[out] percentage Remaining capacity (0-100%)
     * @param[in]  instance Battery instance number (0-8)
     * @return True if percentage is valid and was written to output parameter
     * 
     * @details Calculates remaining capacity based on consumed mAh and configured pack capacity
     *          (BATT*_CAPACITY parameter). Returns false if capacity not configured or reading invalid.
     * 
     * @note 100% indicates full charge, 0% indicates empty
     * @warning Return value must be checked (WARN_IF_UNUSED) - output invalid if returns false
     */
    virtual bool capacity_remaining_pct(uint8_t &percentage, uint8_t instance) const WARN_IF_UNUSED;
    
    /**
     * @brief Get remaining capacity percentage for primary battery
     * 
     * @param[out] percentage Remaining capacity (0-100%)
     * @return True if percentage is valid
     * 
     * @warning Return value must be checked (WARN_IF_UNUSED)
     */
    bool capacity_remaining_pct(uint8_t &percentage) const WARN_IF_UNUSED { return capacity_remaining_pct(percentage, AP_BATT_PRIMARY_INSTANCE); }

    /**
     * @brief Get estimated remaining battery flight time
     * 
     * @param[out] seconds Estimated remaining time in seconds (s)
     * @param[in]  instance Battery instance number (0-8), defaults to primary
     * @return True if time estimate is valid and was written to output parameter
     * 
     * @details Estimates remaining time based on current draw rate and remaining capacity.
     *          Only available from intelligent batteries (SMBus) or when calculated by backend.
     * 
     * @note Estimate assumes current draw remains constant
     * @note Returns false for most battery types (only SMBus and some smart batteries provide this)
     * @warning Return value must be checked (WARN_IF_UNUSED) - output invalid if returns false
     */
    bool time_remaining(uint32_t &seconds, const uint8_t instance = AP_BATT_PRIMARY_INSTANCE) const WARN_IF_UNUSED;

    /**
     * @brief Get configured battery pack capacity
     * 
     * @param[in] instance Battery instance number (0-8)
     * @return Battery pack full capacity in milliamp-hours (mAh), from BATT*_CAPACITY parameter
     * 
     * @note Returns parameter value, not current remaining capacity
     * @see capacity_remaining_pct() for current charge level
     */
    int32_t pack_capacity_mah(uint8_t instance) const;
    
    /**
     * @brief Get primary battery pack configured capacity
     * 
     * @return Primary battery capacity in milliamp-hours (mAh)
     */
    int32_t pack_capacity_mah() const { return pack_capacity_mah(AP_BATT_PRIMARY_INSTANCE); }
 
    /**
     * @brief Check if any battery failsafe has ever been triggered
     * 
     * @return True if any battery instance has entered failsafe state since startup
     * 
     * @note Latches true on first failsafe and never resets (failsafe progression is unidirectional)
     * @see get_highest_failsafe_priority() for severity level reached
     */
    bool has_failsafed(void) const { return _has_triggered_failsafe; };

    /**
     * @brief Get the highest failsafe priority level reached
     * 
     * @return Highest failsafe priority value from vehicle's failsafe_priorities array
     * 
     * @details Returns the maximum severity failsafe that has been triggered across all battery
     *          instances since startup. Used by vehicle code to maintain appropriate failsafe response.
     * 
     * @note Priority values come from vehicle-specific failsafe_priorities array
     */
    int8_t get_highest_failsafe_priority(void) const { return _highest_failsafe_priority; };

    /**
     * @brief Get battery monitor type as configured in parameters
     * 
     * @param[in] instance Battery instance number (0-8)
     * @return Configured monitor type from BATT*_MONITOR parameter
     * 
     * @note Configured type may differ from allocated type if driver unavailable
     * @see allocated_type() for actual backend type instantiated
     */
    Type configured_type(uint8_t instance) const {
        return (Type)_params[instance]._type.get();
    }
    
    /**
     * @brief Get battery monitor backend type actually allocated
     * 
     * @param[in] instance Battery instance number (0-8)
     * @return Backend driver type that was successfully instantiated
     * 
     * @note May differ from configured_type() if requested driver unavailable
     */
    Type allocated_type(uint8_t instance) const {
        return state[instance].type;
    }

    /**
     * @brief Get battery serial number for primary battery
     * 
     * @return Battery serial number from BATT_SERIAL_NUM parameter, -1 if not configured
     * 
     * @note Used to identify specific battery packs, especially for SMBus intelligent batteries
     */
    int32_t get_serial_number() const { return get_serial_number(AP_BATT_PRIMARY_INSTANCE); }
    
    /**
     * @brief Get battery serial number for specified instance
     * 
     * @param[in] instance Battery instance number (0-8)
     * @return Battery serial number from BATT*_SERIAL_NUM parameter
     */
    int32_t get_serial_number(uint8_t instance) const {
        return _params[instance]._serial_number;
    }

    /**
     * @brief Check if primary battery power exceeds configured limit
     * 
     * @return True when (voltage * current) > BATT_WATT_MAX parameter
     * 
     * @note Used to detect overpower conditions that may damage battery or electronics
     */
    bool overpower_detected() const;
    
    /**
     * @brief Check if specified battery power exceeds configured limit
     * 
     * @param[in] instance Battery instance number (0-8)
     * @return True when power exceeds BATT*_WATT_MAX parameter
     */
    bool overpower_detected(uint8_t instance) const;

    /**
     * @brief Check if primary battery provides individual cell voltages
     * 
     * @return True if cell voltage data is available
     * 
     * @note Only some battery types provide cell voltages (SMBus, DroneCAN, scripting)
     */
    bool has_cell_voltages() const { return has_cell_voltages(AP_BATT_PRIMARY_INSTANCE); }
    
    /**
     * @brief Check if specified battery provides individual cell voltages
     * 
     * @param[in] instance Battery instance number (0-8)
     * @return True if cell voltage data is available for this instance
     */
    bool has_cell_voltages(const uint8_t instance) const;
    
    /**
     * @brief Get individual cell voltages for primary battery
     * 
     * @return Reference to cells structure containing cell voltages in millivolts (mV)
     * 
     * @note Check has_cell_voltages() first to verify data validity
     * @note Cell voltages are in millivolts, not volts
     */
    const cells &get_cell_voltages() const { return get_cell_voltages(AP_BATT_PRIMARY_INSTANCE); }
    
    /**
     * @brief Get individual cell voltages for specified battery
     * 
     * @param[in] instance Battery instance number (0-8)
     * @return Reference to cells structure containing cell voltages in millivolts (mV)
     */
    const cells &get_cell_voltages(const uint8_t instance) const;

    /**
     * @brief Get single cell voltage (primarily for Lua scripting access)
     * 
     * @param[in]  instance Battery instance number (0-8)
     * @param[in]  cell Cell index (0-based)
     * @param[out] voltage Cell voltage in volts (V)
     * @return True if cell voltage is valid and was written to output parameter
     * 
     * @note Output is in volts (V) unlike get_cell_voltages() which returns millivolts
     * @note Provided for scripting convenience to avoid array handling
     */
    bool get_cell_voltage(uint8_t instance, uint8_t cell, float &voltage) const;

    /**
     * @brief Get primary battery temperature
     * 
     * @param[out] temperature Battery temperature in degrees Celsius (°C)
     * @return True if temperature reading is valid and was written to output parameter
     * 
     * @note Temperature may come from internal sensor or external temperature sensor
     */
    bool get_temperature(float &temperature) const { return get_temperature(temperature, AP_BATT_PRIMARY_INSTANCE); }
    
    /**
     * @brief Get battery temperature for specified instance
     * 
     * @param[out] temperature Battery temperature in degrees Celsius (°C)
     * @param[in]  instance Battery instance number (0-8)
     * @return True if temperature reading is valid
     */
    bool get_temperature(float &temperature, const uint8_t instance) const;
    
#if AP_TEMPERATURE_SENSOR_ENABLED
    /**
     * @brief Set battery temperature from external temperature sensor
     * 
     * @param[in] temperature Battery temperature in degrees Celsius (°C)
     * @param[in] instance Battery instance number (0-8)
     * @return True if temperature was accepted and stored
     * 
     * @note Allows external temperature sensors to override battery's internal temperature
     * @note Requires AP_TEMPERATURE_SENSOR_ENABLED feature
     */
    bool set_temperature(const float temperature, const uint8_t instance);
    
    /**
     * @brief Set battery temperature by matching serial number
     * 
     * @param[in] temperature Battery temperature in degrees Celsius (°C)
     * @param[in] serial_number Battery serial number to match
     * @return True if matching battery found and temperature was set
     * 
     * @note Useful for external sensors that identify batteries by serial number
     */
    bool set_temperature_by_serial_number(const float temperature, const int32_t serial_number);
#endif

    /**
     * @brief Set powered state for all MPPT (solar panel) battery monitors
     * 
     * @param[in] power_on True to enable MPPT charging, false to disable
     * 
     * @note Only affects MPPT-type battery monitors, ignored by other types
     * @note Used to control solar panel power contribution to system
     */
    void MPPT_set_powered_state_to_all(const bool power_on);
    
    /**
     * @brief Set powered state for specific MPPT battery monitor
     * 
     * @param[in] instance Battery instance number (0-8)
     * @param[in] power_on True to enable MPPT charging, false to disable
     * 
     * @note Only affects MPPT-type battery monitors
     */
    void MPPT_set_powered_state(const uint8_t instance, const bool power_on);

    /**
     * @brief Check if a specific parameter option flag is set
     * 
     * @param[in] instance Battery instance number (0-8)
     * @param[in] option Option flag to check from AP_BattMonitor_Params::Options enum
     * @return True if the specified option is enabled in BATT*_OPTIONS parameter
     * 
     * @note Options control features like voltage compensation, current sensor type, etc.
     */
    bool option_is_set(uint8_t instance, AP_BattMonitor_Params::Options option) const;

    /**
     * @brief Get battery charge/discharge cycle count
     * 
     * @param[in]  instance Battery instance number (0-8)
     * @param[out] cycles Number of charge/discharge cycles
     * @return True if cycle count is available and was written to output parameter
     * 
     * @note Only available from intelligent batteries (SMBus) that track cycle count
     * @note Higher cycle count indicates battery degradation
     */
    bool get_cycle_count(uint8_t instance, uint16_t &cycles) const;

    /**
     * @brief Get primary battery internal resistance estimate
     * 
     * @return Battery internal resistance in ohms (Ω)
     * 
     * @note Resistance is estimated during flight by comparing resting vs loaded voltage
     * @note Valid resistance estimate requires flight with varying current draw
     */
    float get_resistance() const { return get_resistance(AP_BATT_PRIMARY_INSTANCE); }
    
    /**
     * @brief Get battery internal resistance estimate for specified instance
     * 
     * @param[in] instance Battery instance number (0-8)
     * @return Battery internal resistance in ohms (Ω)
     */
    float get_resistance(uint8_t instance) const { return state[instance].resistance; }

    /**
     * @brief Perform pre-arm checks for all battery monitors
     * 
     * @param[in]  buflen Size of failure message buffer
     * @param[out] buffer Buffer to receive failure message if check fails
     * @return False if any arming check fails, true if all checks pass
     * 
     * @details Validates:
     *          - All configured monitors are healthy
     *          - Battery voltage above minimum
     *          - Battery capacity sufficient
     *          - No existing failsafe conditions
     * 
     * @note Called by vehicle arming checks before allowing motor start
     * @warning Vehicle should not arm if this returns false
     */
    bool arming_checks(size_t buflen, char *buffer) const;

    /**
     * @brief Check for power button commanded shutdown and send MAVLink notifications
     * 
     * @details Monitors is_powering_off flag in battery state and sends MAVLink POWER_STATUS
     *          message to inform ground station of impending shutdown. Sets notification flag
     *          after first message to prevent repeated notifications.
     * 
     * @note Should be called periodically from main loop
     * @note Only relevant for systems with smart batteries supporting power button
     */
    void checkPoweringOff(void);

    /**
     * @brief Reset remaining capacity for multiple battery instances
     * 
     * @param[in] battery_mask Bitmask of battery instances to reset (bit 0 = instance 0, etc.)
     * @param[in] percentage New remaining capacity percentage (0-100%)
     * @return True if reset was successful for all specified instances
     * 
     * @note Used when replacing or recharging batteries to reset consumed capacity tracking
     * @note Typically set to 100% when installing fresh/charged battery
     */
    bool reset_remaining_mask(uint16_t battery_mask, float percentage);
    
    /**
     * @brief Reset remaining capacity for single battery instance
     * 
     * @param[in] instance Battery instance number (0-8)
     * @param[in] percentage New remaining capacity percentage (0-100%)
     * @return True if reset was successful
     */
    bool reset_remaining(uint8_t instance, float percentage) { return reset_remaining_mask(1U<<instance, percentage);}

    /**
     * @brief Get MAVLink battery charge state enum
     * 
     * @param[in] instance Battery instance number (0-8)
     * @return MAVLink MAV_BATTERY_CHARGE_STATE enum value
     * 
     * @note Returns charge state for MAVLink BATTERY_STATUS message
     * @note States include: UNDEFINED, OK, LOW, CRITICAL, EMERGENCY, FAILED, UNHEALTHY, CHARGING
     */
    MAV_BATTERY_CHARGE_STATE get_mavlink_charge_state(const uint8_t instance) const;

    /**
     * @brief Get MAVLink battery fault bitmask
     * 
     * @param[in] instance Battery instance number (0-8)
     * @return MAVLink MAV_BATTERY_FAULT bitmask indicating detected fault conditions
     * 
     * @note Returns fault flags for MAVLink BATTERY_STATUS message
     * @note Fault bits indicate: deep discharge, spikes, cell fail, overcurrent, over temperature, etc.
     */
    uint32_t get_mavlink_fault_bitmask(const uint8_t instance) const;

    /**
     * @brief Get battery state of health percentage
     * 
     * @param[in]  instance Battery instance number (0-8)
     * @param[out] soh_pct State of health percentage (0-100%), where 100% is new battery
     * @return True if state of health is available and was written to output parameter
     * 
     * @note Only available from intelligent batteries (SMBus) that track degradation
     * @note SoH indicates capacity degradation: 100% = full capacity, 80% = 20% capacity loss
     */
    bool get_state_of_health_pct(uint8_t instance, uint8_t &soh_pct) const;

    /// AP_Param parameter table metadata for frontend parameters
    static const struct AP_Param::GroupInfo var_info[];

#if AP_BATTERY_SCRIPTING_ENABLED
    /**
     * @brief Handle battery state updates from Lua scripting
     * 
     * @param[in] idx Battery instance index (0-8)
     * @param[in] state Battery state structure populated by Lua script
     * @return True if state was accepted and applied
     * 
     * @details Allows Lua scripts to inject battery monitoring data for custom battery types
     *          not supported by built-in backends. Script must update state at sufficient rate
     *          (~10Hz recommended) to maintain healthy status.
     * 
     * @note Requires AP_BATTERY_SCRIPTING_ENABLED and scripting monitor type configured
     * @note Script is responsible for providing all required state fields
     */
    bool handle_scripting(uint8_t idx, const struct BattMonitorScript_State &state);
#endif

protected:

    /// Per-instance parameter storage (up to 9 instances)
    AP_BattMonitor_Params _params[AP_BATT_MONITOR_MAX_INSTANCES];

private:
    static AP_BattMonitor *_singleton;  ///< Singleton instance pointer for global access

    BattMonitor_State state[AP_BATT_MONITOR_MAX_INSTANCES];        ///< Per-instance battery state maintained by backends
    AP_BattMonitor_Backend *drivers[AP_BATT_MONITOR_MAX_INSTANCES]; ///< Backend driver pointers (nullptr if instance not configured)
    uint32_t    _log_battery_bit;       ///< Logger bitmask for battery data logging
    uint8_t     _num_instances;         ///< Number of configured and initialized battery instances

    /**
     * @brief Evaluate failsafe condition for a single battery instance
     * 
     * @param[in] instance Battery instance number (0-8)
     * @return Current failsafe level for this instance
     * 
     * @details Checks battery voltage, capacity, and health against configured thresholds
     *          to determine appropriate failsafe level. Called by check_failsafes() for each instance.
     */
    Failsafe check_failsafe(const uint8_t instance);
    
    /**
     * @brief Check failsafe conditions for all battery instances
     * 
     * @details Iterates through all configured instances, evaluates failsafe state, and invokes
     *          vehicle callback if any instance has progressed to a higher failsafe level.
     *          Maintains highest failsafe priority reached across all instances.
     * 
     * @note Called automatically by read() after updating battery state
     */
    void check_failsafes(void);

    battery_failsafe_handler_fn_t _battery_failsafe_handler_fn; ///< Vehicle-specific failsafe callback function
    const int8_t *_failsafe_priorities; ///< Array of failsafe priorities sorted highest to lowest, -1 sentinel terminates

    int8_t      _highest_failsafe_priority; ///< Highest failsafe priority level triggered (tracks maximum severity reached)
    bool        _has_triggered_failsafe;    ///< True after any battery failsafe triggered for first time (latches true)

};

namespace AP {
    /**
     * @brief Get reference to AP_BattMonitor singleton
     * 
     * @return Reference to the global AP_BattMonitor instance
     * 
     * @note Convenient namespace accessor for global battery monitor access
     * @note Equivalent to AP_BattMonitor::get_singleton() but returns reference
     */
    AP_BattMonitor &battery();
};

#endif  // AP_BATTERY_ENABLED
