/**
 * @file AP_BattMonitor_Scripting.h
 * @brief Lua scripting interface for custom battery monitoring
 * 
 * This file defines the battery monitor backend that enables Lua scripts
 * to provide battery state information asynchronously, allowing custom
 * battery monitoring implementations without requiring C++ code changes.
 */

#pragma once

#include "AP_BattMonitor_config.h"

#if AP_BATTERY_SCRIPTING_ENABLED

#include "AP_BattMonitor_Backend.h"

/**
 * @class AP_BattMonitor_Scripting
 * @brief Battery monitor backend for Lua script-based custom battery drivers
 * 
 * @details This backend allows Lua scripts to provide battery state data
 *          asynchronously via the handle_scripting() method. This enables
 *          custom battery monitoring implementations for unsupported hardware
 *          or specialized battery management systems without modifying C++ code.
 *          
 *          The backend stores script-provided state in internal_state and
 *          validates data freshness using timestamps. Capability queries
 *          (current, energy, cell voltages, temperature) are determined
 *          dynamically based on the validity of script-provided data.
 *          
 *          Thread Safety: Script callbacks are protected by semaphore to
 *          ensure safe concurrent access between scheduler tasks and script
 *          execution context.
 * 
 * @note Lua scripting API allows custom battery drivers for unsupported hardware
 * @warning Script update rate requirements: Scripts should provide data at >=1Hz
 *          to avoid timeout. The read() method checks for stale data and will
 *          mark the backend unhealthy if updates stop.
 * 
 * @see AP_Scripting library and battery monitoring examples in libraries/AP_Scripting/examples/
 */
class AP_BattMonitor_Scripting : public AP_BattMonitor_Backend
{
public:
    /**
     * @brief Inherit constructor from AP_BattMonitor_Backend
     * 
     * Uses parent class constructor for standard backend initialization.
     */
    using AP_BattMonitor_Backend::AP_BattMonitor_Backend;

    /**
     * @brief Query whether current measurement capability is available
     * 
     * @return true if script has provided valid current data (non-NaN current_amps
     *         and at least one update received), false otherwise
     */
    bool has_current() const override { return last_update_us != 0 && !isnan(internal_state.current_amps); }
    
    /**
     * @brief Query whether consumed energy measurement capability is available
     * 
     * @return true if current measurement is available (consumed energy can be
     *         integrated from current), false otherwise
     */
    bool has_consumed_energy() const override { return has_current(); }
    
    /**
     * @brief Query whether individual cell voltage measurements are available
     * 
     * @return true if script has provided cell voltage data (cell_count > 0),
     *         false otherwise
     */
    bool has_cell_voltages() const override { return internal_state.cell_count > 0; }
    
    /**
     * @brief Query whether temperature measurement capability is available
     * 
     * @return true if script has provided valid temperature data (non-NaN temperature
     *         and at least one update received), false otherwise
     */
    bool has_temperature() const override { return last_update_us != 0 && !isnan(internal_state.temperature); }
    /**
     * @brief Get remaining battery capacity as percentage
     * 
     * Retrieves the battery capacity percentage from script-provided state data.
     * 
     * @param[out] percentage Battery capacity remaining as percentage (0-100)
     * @return true if valid percentage data available from script, false otherwise
     */
    bool capacity_remaining_pct(uint8_t &percentage) const override;
    
    /**
     * @brief Get battery charge/discharge cycle count
     * 
     * Retrieves the number of charge/discharge cycles from script-provided state.
     * Useful for battery health monitoring and maintenance scheduling.
     * 
     * @param[out] cycles Number of charge/discharge cycles the battery has experienced
     * @return true if valid cycle count data available from script, false otherwise
     */
    bool get_cycle_count(uint16_t &cycles) const override;

    /**
     * @brief Periodic state validation and health checking
     * 
     * @details Called at ~10Hz by the battery monitor framework. Validates that
     *          script-provided data is still fresh by checking timestamps.
     *          Marks backend as unhealthy if no updates received within timeout
     *          period, preventing use of stale battery data.
     *          
     *          This method does not actively read hardware - it relies on
     *          asynchronous updates from the Lua script via handle_scripting().
     */
    void read() override;

    /**
     * @brief Transfer battery state from Lua script to backend
     * 
     * @details Called by Lua scripting framework when script provides updated
     *          battery state. Copies script-provided state to internal_state
     *          with semaphore protection for thread safety. Updates timestamp
     *          for health monitoring by read() method.
     *          
     *          This is the primary interface through which Lua scripts populate
     *          battery voltage, current, capacity, cell voltages, temperature,
     *          and other battery parameters.
     * 
     * @param[in] battmon_state Battery state structure populated by Lua script
     *            containing voltage, current, capacity, temperature, cell voltages, etc.
     * 
     * @return true if state successfully stored, false on error (e.g., semaphore timeout)
     */
    bool handle_scripting(const BattMonitorScript_State &battmon_state) override;

protected:
    BattMonitorScript_State internal_state;  ///< Script-provided battery state data including voltage, current, capacity, temperature, and cell voltages
    uint32_t last_update_us;                 ///< Timestamp of last script update in microseconds for health monitoring and timeout detection
    
    HAL_Semaphore sem;                       ///< Semaphore for thread-safe access during script callback execution
};

#endif // AP_BATTERY_SCRIPTING_ENABLED
