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
 * @file AP_BattMonitor_Backend.h
 * @brief Battery monitor backend abstract base class and interface contract
 * 
 * Defines the abstract interface that all battery monitor backend implementations
 * must implement. Provides common algorithms for capacity estimation, internal
 * resistance calculation, and failsafe determination that can be used by all
 * backend types.
 * 
 * Backend implementations support various battery monitoring hardware including
 * analog voltage/current sensors, SMBus/I2C fuel gauges, UAVCAN/DroneCAN smart
 * batteries, and ESC telemetry integration.
 */

#pragma once

#include "AP_BattMonitor_config.h"

#if AP_BATTERY_ENABLED

#include "AP_BattMonitor.h"

#include <AP_Common/AP_Common.h>

/**
 * @class AP_BattMonitor_Backend
 * @brief Abstract base class defining the interface for all battery monitor backends
 * 
 * @details This abstract base class defines the contract that all battery monitor
 *          backend implementations must fulfill. Backends represent different hardware
 *          interfaces for battery monitoring (analog sensors, SMBus, UAVCAN, etc.).
 *          
 *          The class provides:
 *          - Pure virtual read() method that backends must implement
 *          - Common algorithms for capacity estimation and resistance calculation
 *          - Failsafe evaluation logic
 *          - Logging functionality
 *          - Parameter option checking
 *          
 *          Typical backend lifecycle:
 *          1. Construction with references to frontend, state, and parameters
 *          2. Optional init() for backend-specific initialization
 *          3. Periodic read() calls at ~10Hz to update voltage/current
 *          4. Capability queries (has_current, has_temperature, etc.)
 *          5. Automatic resistance estimation and failsafe monitoring
 *          6. Binary logging of battery state
 * 
 * @note Backends must be thread-safe if accessing shared resources - use HAL semaphores
 * @warning This is safety-critical code affecting battery failsafe and capacity estimation
 */
class AP_BattMonitor_Backend
{
public:
    /**
     * @brief Construct a battery monitor backend instance
     * 
     * @param[in] mon Reference to the AP_BattMonitor frontend singleton
     * @param[in] mon_state Reference to this instance's state structure held in frontend
     * @param[in] params Reference to this instance's parameter set held in frontend
     * 
     * @note Constructor incorporates initialization - backend is ready for init() call after construction
     */
    AP_BattMonitor_Backend(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state, AP_BattMonitor_Params &params);

    /**
     * @brief Virtual destructor allowing backend-specific cleanup
     * 
     * @details Declared virtual so that battery monitor backends can override
     *          with custom destructors if they need to release hardware resources,
     *          close file handles, or perform other cleanup operations.
     */
    virtual ~AP_BattMonitor_Backend(void) {}

    /**
     * @brief Optional backend-specific initialization
     * 
     * @details Called after construction to perform any backend-specific setup such as
     *          configuring hardware registers, opening communication channels, or
     *          allocating resources. Default implementation does nothing.
     *          
     *          Backends should override this if they need initialization beyond construction.
     */
    virtual void init() {};

    /**
     * @brief Read and update battery voltage and current measurements
     * 
     * @details Pure virtual function that all backends must implement. Called periodically
     *          at approximately 10Hz by the scheduler to update battery state including
     *          voltage, current, consumed capacity, and other measurements.
     *          
     *          Implementations must:
     *          - Read voltage from hardware and update _state.voltage
     *          - Read current if available and update _state.current_amps
     *          - Update _state.consumed_mah using update_consumed() helper
     *          - Set _state.healthy based on successful reads
     *          - Update any backend-specific measurements (temperature, cell voltages, etc.)
     * 
     * @note Called at main loop rate, typically 10Hz - keep execution time minimal
     * @warning Must handle hardware read failures gracefully and set healthy flag appropriately
     */
    virtual void read() = 0;

    /**
     * @brief Query if backend provides estimated time remaining until battery empty
     * 
     * @return true if backend can provide time remaining estimate, false otherwise
     * 
     * @note Default implementation returns false - override if backend supports this capability
     */
    virtual bool has_time_remaining() const { return false; }

    /**
     * @brief Query if backend provides consumed energy information
     * 
     * @return true if backend tracks total energy consumed in watt-hours, false otherwise
     * 
     * @note Default implementation returns false - override if backend supports this capability
     */
    virtual bool has_consumed_energy() const { return false; }

    /**
     * @brief Query if backend provides current measurement
     * 
     * @return true if backend can measure battery current, false for voltage-only monitoring
     * 
     * @note Pure virtual - all backends must declare whether they support current sensing
     */
    virtual bool has_current() const = 0;

    /**
     * @brief Query if backend provides individual cell voltage measurements
     * 
     * @return true if backend can report per-cell voltages (e.g., SMBus batteries), false otherwise
     * 
     * @note Default implementation returns false - override if backend supports this capability
     */
    virtual bool has_cell_voltages() const { return false; }

    /**
     * @brief Query if backend provides temperature measurement
     * 
     * @return true if backend can measure battery temperature, false otherwise
     * 
     * @note Default implementation returns false - override if backend supports this capability
     */
    virtual bool has_temperature() const { return false; }

    /**
     * @brief Retrieve battery temperature measurement
     * 
     * @param[out] temperature Battery temperature in degrees Celsius
     * 
     * @return true if temperature retrieved successfully and written to parameter, false otherwise
     * 
     * @note Default implementation returns false - override if backend supports temperature sensing
     */
    virtual bool get_temperature(float &temperature) const;

    /**
     * @brief Calculate remaining battery capacity as a percentage
     * 
     * @param[out] percentage Remaining capacity as percentage (0-100)
     * 
     * @return true if percentage calculated and written to parameter, false on failure
     * 
     * @details Calculates remaining capacity based on consumed mAh and pack capacity parameter.
     *          Calculation: percentage = 100 * (1 - consumed_mah / pack_capacity)
     *          
     *          Failure conditions:
     *          - Battery unhealthy (communication lost)
     *          - No current monitoring capability (cannot track consumption)
     *          - Pack capacity parameter too small or not configured
     * 
     * @warning Returns false if battery is unhealthy - check health status before trusting values
     * @warning Requires pack capacity parameter to be configured correctly for accurate results
     * 
     * @note Result type is WARN_IF_UNUSED - callers should check return value
     */
    virtual bool capacity_remaining_pct(uint8_t &percentage) const WARN_IF_UNUSED;

    /**
     * @brief Retrieve battery charge/discharge cycle count
     * 
     * @param[out] cycles Number of charge/discharge cycles battery has experienced
     * 
     * @return true if cycle count available and written to parameter, false otherwise
     * 
     * @note Default implementation returns false - only smart batteries (SMBus, UAVCAN) provide this
     * @note Cycle count useful for estimating battery age and degradation
     */
    virtual bool get_cycle_count(uint16_t &cycles) const { return false; }

    /**
     * @brief Retrieve battery state of health as a percentage
     * 
     * @param[out] soh_pct State of health percentage (0-100, where 100 is new battery condition)
     * 
     * @return true if state of health available and written to parameter, false otherwise
     * 
     * @details State of health indicates battery degradation over its lifetime.
     *          100% = new battery, lower percentages indicate capacity loss and increased
     *          internal resistance due to aging, cycling, or abuse.
     * 
     * @note Only smart batteries with fuel gauge ICs typically provide SoH information
     */
    bool get_state_of_health_pct(uint8_t &soh_pct) const;

    /**
     * @brief Get estimated resting voltage with current-induced sag removed
     * 
     * @return Estimated battery voltage in volts without sag from current draw
     * 
     * @details Uses estimated internal resistance and measured current to calculate
     *          what battery voltage would be at zero load. This provides better
     *          estimation of true battery state than raw voltage under load.
     *          
     *          Calculation: V_rest = V_measured + (I_measured * R_internal)
     *          
     *          Resting voltage will always be greater than or equal to measured voltage.
     * 
     * @note Requires current monitoring and internal resistance estimation to be active
     * @see update_resistance_estimate()
     */
    float voltage_resting_estimate() const;

    /**
     * @brief Update internal resistance estimate and resting voltage calculation
     * 
     * @details Continuously updates estimate of battery internal resistance based on
     *          observed voltage sag during current draw. Uses filtered voltage and
     *          current measurements to calculate resistance.
     *          
     *          Algorithm tracks maximum current draw and corresponding voltage drop
     *          to estimate resistance: R = (V_rest_ref - V_sag) / I_max
     *          
     *          Updated periodically (default 10 seconds) to adapt to battery state changes.
     * 
     * @note Override if backend has direct resistance measurement capability
     * @note Resistance estimate improves over time as battery experiences load variations
     */
    virtual void update_resistance_estimate();

    /**
     * @brief Update failsafe state and return active failsafe conditions
     * 
     * @return Current failsafe state (None, Low, Critical, or combination)
     * 
     * @details Evaluates battery state against configured failsafe thresholds and updates
     *          failsafe timers. Checks for:
     *          - Low voltage condition (battery voltage below FS_VOLTSRC threshold)
     *          - Low capacity condition (remaining capacity below FS_CRT_MAH threshold)
     *          - Critical voltage condition (battery voltage below FS_VOLT_CRIT threshold)
     *          - Critical capacity condition (remaining capacity critically low)
     *          
     *          Failsafes include hysteresis to prevent oscillation at threshold boundaries.
     * 
     * @return Bitmask of active failsafes from AP_BattMonitor::Failsafe enum
     * 
     * @warning This is safety-critical code affecting vehicle failsafe behavior
     * @note Called periodically by frontend to monitor battery health
     */
    virtual AP_BattMonitor::Failsafe update_failsafes(void);

    /**
     * @brief Perform pre-arm safety checks for battery monitor
     * 
     * @param[out] buffer Character buffer to receive failure message if check fails
     * @param[in] buflen Length of provided buffer in bytes
     * 
     * @return true if all arming checks pass, false if vehicle should not be armed
     * 
     * @details Pre-arm checks validate:
     *          - Battery voltage is above minimum safe level
     *          - Battery capacity is sufficient for flight
     *          - Battery monitor is healthy and communicating
     *          - Battery parameters are properly configured
     *          
     *          If checks fail, buffer is populated with human-readable error message
     *          explaining why arming was rejected.
     * 
     * @warning Arming checks are safety-critical - failing checks prevents takeoff
     * @note Check failure messages displayed to pilot via ground station
     */
    bool arming_checks(char * buffer, size_t buflen) const;

    /**
     * @brief Reset remaining capacity to specified percentage
     * 
     * @param[in] percentage Target remaining capacity percentage (0-100)
     * 
     * @return true if reset successful, false if operation not supported or failed
     * 
     * @details Used when battery is replaced or recharged to reset capacity tracking.
     *          Adjusts consumed_mah calculation to reflect new capacity state.
     *          
     *          Typical usage: reset_remaining(100.0) after battery swap/recharge
     * 
     * @note Default implementation supports reset for backends with current monitoring
     * @warning Incorrect reset values will cause inaccurate capacity estimation
     */
    virtual bool reset_remaining(float percentage);

    /**
     * @brief Get MAVLink battery fault status bitmask
     * 
     * @return Bitmask of battery faults per MAV_BATTERY_FAULT enum specification
     * 
     * @details Returns bitmask indicating detected battery faults for MAVLink telemetry:
     *          - BATTERY_FAULT_DEEP_DISCHARGE (0x01)
     *          - BATTERY_FAULT_SPIKES (0x02)
     *          - BATTERY_FAULT_CELL_FAIL (0x04)
     *          - BATTERY_FAULT_OVER_CURRENT (0x08)
     *          - BATTERY_FAULT_OVER_TEMPERATURE (0x10)
     *          - BATTERY_FAULT_UNDER_TEMPERATURE (0x20)
     *          - BATTERY_FAULT_INCOMPATIBLE_VOLTAGE (0x40)
     *          - BATTERY_FAULT_INCOMPATIBLE_FIRMWARE (0x80)
     * 
     * @return 0 (no faults) for default implementation
     * 
     * @note Override if backend can detect and report specific battery fault conditions
     * @see MAV_BATTERY_FAULT enum in MAVLink common message set
     */
    virtual uint32_t get_mavlink_fault_bitmask() const { return 0; }

    /**
     * @brief Write BAT binary log message with battery state
     * 
     * @param[in] instance Battery monitor instance number (0-9)
     * @param[in] time_us Timestamp in microseconds since system boot
     * 
     * @details Generates BAT log message containing:
     *          - Voltage, current, consumed capacity
     *          - Remaining capacity percentage
     *          - Temperature (if available)
     *          - Failsafe state
     *          
     *          Called periodically to record battery state history for post-flight analysis.
     * 
     * @note Log messages enable detailed battery performance analysis and issue diagnosis
     */
    void Log_Write_BAT(const uint8_t instance, const uint64_t time_us) const;

    /**
     * @brief Write BCL binary log message with individual cell voltages
     * 
     * @param[in] instance Battery monitor instance number (0-9)
     * @param[in] time_us Timestamp in microseconds since system boot
     * 
     * @details Generates BCL log message containing individual cell voltages for
     *          multi-cell batteries. Only written if backend provides cell voltage data.
     *          
     *          Useful for diagnosing cell imbalance and identifying failing cells.
     * 
     * @note Only smart batteries (SMBus, UAVCAN) typically provide per-cell voltages
     */
    void Log_Write_BCL(const uint8_t instance, const uint64_t time_us) const;

    /**
     * @brief Control MPPT (Maximum Power Point Tracking) power state
     * 
     * @param[in] power_on true to enable MPPT, false to disable
     * 
     * @details Controls power state of solar MPPT charge controllers that can be
     *          remotely enabled/disabled. Default implementation does nothing.
     * 
     * @note Override if backend supports MPPT power control (e.g., Torqeedo solar integration)
     */
    virtual void mppt_set_powered_state(bool power_on) {};

    /**
     * @brief Update ESC telemetry with battery power information
     * 
     * @details Pushes battery voltage and current information to ESC telemetry system
     *          for integration with ESC power monitoring and logging. Enables correlation
     *          of battery state with motor power consumption.
     * 
     * @note Called periodically by backends that integrate with ESC telemetry
     */
    void update_esc_telem_outbound();

    /**
     * @brief Calculate milliamp-hours from current and time
     * 
     * @param[in] amps Current in amperes
     * @param[in] dt_us Time interval in microseconds
     * 
     * @return Charge transferred in milliamp-hours (mAh)
     * 
     * @details Performs integration of current over time to calculate consumed capacity:
     *          mAh = amps * dt_us * AUS_TO_MAH
     *          
     *          Where AUS_TO_MAH converts ampere-microseconds to milliamp-hours:
     *          AUS_TO_MAH = 1.0 / (3600 * 1000000) = 2.7778e-10
     * 
     * @note Static utility function used by backends to accumulate consumed capacity
     * @note AUS_TO_MAH constant converts ampere-microseconds to milliamp-hours
     */
    static float calculate_mah(float amps, float dt_us) { return (float) (amps * dt_us * AUS_TO_MAH); }

    /**
     * @brief Check if a specific parameter option flag is set
     * 
     * @param[in] option Option flag from AP_BattMonitor_Params::Options enum to check
     * 
     * @return true if specified option is enabled in parameters, false otherwise
     * 
     * @details Tests parameter bitmask to determine if specific option is enabled.
     *          Options control features like:
     *          - Ignore DroneCAN SoC (State of Charge)
     *          - MPPT power control
     *          - ESC telemetry integration
     * 
     * @note Used internally by backends to check configuration options
     */
    bool option_is_set(const AP_BattMonitor_Params::Options option) const {
        return (uint16_t(_params._options.get()) & uint16_t(option)) != 0;
    }
    
#if AP_BATTERY_SCRIPTING_ENABLED
    /**
     * @brief Handle battery state updates from Lua scripting interface
     * 
     * @param[in] battmon_state Battery state structure populated by Lua script
     * 
     * @return true if state update accepted and processed, false otherwise
     * 
     * @details Allows Lua scripts to provide battery monitoring data from custom
     *          hardware or protocols not natively supported. Scripts can update:
     *          - Voltage and current
     *          - Cell voltages and count
     *          - Consumed capacity and energy
     *          - Temperature
     *          - State of health and cycle count
     *          
     *          Default implementation returns false - only scripting backends support this.
     * 
     * @note Only available when AP_BATTERY_SCRIPTING_ENABLED is defined
     * @see BattMonitorScript_State for structure definition
     */
    virtual bool handle_scripting(const BattMonitorScript_State &battmon_state) { return false; }
#endif

protected:
    AP_BattMonitor                      &_mon;      ///< Reference to AP_BattMonitor frontend singleton
    AP_BattMonitor::BattMonitor_State   &_state;    ///< Reference to this instance's state structure (held in frontend)
    AP_BattMonitor_Params               &_params;   ///< Reference to this instance's parameters (held in frontend)

    /**
     * @brief Evaluate which failsafe conditions would be triggered by current battery state
     * 
     * @param[out] low_voltage Set to true if low voltage failsafe threshold exceeded
     * @param[out] low_capacity Set to true if low capacity failsafe threshold exceeded
     * @param[out] critical_voltage Set to true if critical voltage failsafe threshold exceeded
     * @param[out] critical_capacity Set to true if critical capacity failsafe threshold exceeded
     * 
     * @details Helper method that compares current battery state against configured
     *          failsafe thresholds and determines which failsafe types would be active.
     *          Used by update_failsafes() to evaluate failsafe state.
     * 
     * @note Does not update failsafe timers - only evaluates threshold conditions
     */
    void check_failsafe_types(bool &low_voltage, bool &low_capacity, bool &critical_voltage, bool &critical_capacity) const;

    /**
     * @brief Update consumed capacity tracking
     * 
     * @param[in,out] state Battery state structure to update
     * @param[in] dt_us Time since last update in microseconds
     * 
     * @details Integrates current measurement over time to track total consumed capacity
     *          in milliamp-hours. Updates state.consumed_mah based on state.current_amps
     *          and elapsed time.
     *          
     *          Called by backend read() implementations to maintain consumption tracking.
     * 
     * @note Requires accurate current measurement for correct capacity tracking
     */
    void update_consumed(AP_BattMonitor::BattMonitor_State &state, uint32_t dt_us);

private:
    // Internal resistance estimation state variables
    uint32_t    _resistance_timer_ms;    ///< System time (ms) of last resistance estimate update
    float       _voltage_filt;           ///< Filtered battery voltage (volts) for resistance calculation
    float       _current_max_amps;       ///< Maximum current (amperes) observed since startup
    float       _current_filt_amps;      ///< Filtered battery current (amperes) for resistance calculation
    float       _resistance_voltage_ref; ///< Reference voltage (volts) used for maximum resistance calculation
    float       _resistance_current_ref; ///< Reference current (amperes) used for maximum resistance calculation
};

#if AP_BATTERY_SCRIPTING_ENABLED
/**
 * @struct BattMonitorScript_State
 * @brief Battery state structure for Lua scripting interface
 * 
 * @details Structure used to pass battery monitoring data from Lua scripts to the
 *          battery monitor backend. Allows scripts to provide battery data from
 *          custom hardware or protocols not natively supported by ArduPilot.
 *          
 *          Scripts populate this structure and pass it to the battery monitor via
 *          the scripting bindings. The backend validates and integrates the data
 *          into the normal battery monitoring system.
 * 
 * @note Only available when AP_BATTERY_SCRIPTING_ENABLED is defined
 * @note All float values should be set to NaN (nanf("")) if unknown or unavailable
 * @note consumed_mah will auto-integrate from current_amps if set to NaN
 */
struct BattMonitorScript_State {
    float voltage;                                 ///< Battery voltage in volts
    bool healthy;                                  ///< true if battery communication is healthy, false on errors
    uint8_t cell_count;                            ///< Number of valid cell voltages in cell_voltages array
    uint8_t capacity_remaining_pct=UINT8_MAX;      ///< Remaining capacity percentage (0-100), UINT8_MAX (255) if unavailable
    uint8_t state_of_health_pct=UINT8_MAX;         ///< State of health percentage (0-100), UINT8_MAX (255) if unavailable
    uint16_t cell_voltages[32];                    ///< Individual cell voltages in millivolts (up to 32 cells, limited internally)
    uint16_t cycle_count=UINT16_MAX;               ///< Battery charge/discharge cycle count, UINT16_MAX (65535) if unavailable
    
    // Float fields should be set to NaN by script if unknown
    // consumed_mah will auto-integrate from current_amps if set to NaN
    float current_amps=nanf("");                   ///< Battery current in amperes, NaN if unknown (positive = discharge)
    float consumed_mah=nanf("");                   ///< Total capacity consumed since startup in milliamp-hours, NaN for auto-integration
    float consumed_wh=nanf("");                    ///< Total energy consumed since startup in watt-hours, NaN if unknown
    float temperature=nanf("");                    ///< Battery temperature in degrees Celsius, NaN if unknown
};
#endif // AP_BATTERY_SCRIPTING_ENABLED

#endif  // AP_BATTERY_ENABLED
