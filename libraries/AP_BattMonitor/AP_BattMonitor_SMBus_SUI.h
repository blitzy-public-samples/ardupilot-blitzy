/**
 * @file AP_BattMonitor_SMBus_SUI.h
 * @brief Battery monitor backend for SUI (Smart Battery Systems / SBS) smart batteries
 * 
 * @details This file implements the SMBus battery monitor backend for SUI/SBS
 *          smart batteries. SUI batteries are SMBus-compliant smart batteries
 *          that require explicit cell count configuration and use custom register
 *          access without Packet Error Checking (PEC).
 */

#pragma once

#include "AP_BattMonitor_SMBus.h"

#if AP_BATTERY_SMBUS_SUI_ENABLED

/**
 * @class AP_BattMonitor_SMBus_SUI
 * @brief SMBus battery monitor backend for SUI/SBS smart batteries
 * 
 * @details This backend implements support for SUI (Smart Battery Systems / SBS)
 *          smart batteries over the SMBus interface. Key characteristics:
 *          - Fixed cell count configuration specified at construction
 *          - Custom register access without Packet Error Checking (PEC)
 *          - Individual cell voltage monitoring capability
 *          - Health status monitoring from battery internal diagnostics
 *          - Phased voltage reading (alternates between pack and cell voltages)
 * 
 * @note SUI batteries require explicit cell count configuration at initialization
 *       and do not support PEC for data integrity verification.
 * 
 * @warning Cell count mismatch between configured value and actual battery
 *          cell count will cause incorrect voltage reporting and potential
 *          battery damage due to improper charging/monitoring.
 */
class AP_BattMonitor_SMBus_SUI : public AP_BattMonitor_SMBus
{
public:

    /**
     * @brief Constructor for SUI smart battery monitor backend
     * 
     * @details Initializes the SMBus backend for SUI/SBS smart batteries with
     *          a fixed cell count. The cell count must match the physical battery
     *          configuration to ensure accurate voltage monitoring and reporting.
     * 
     * @param[in] mon           Reference to parent AP_BattMonitor object
     * @param[in] mon_state     Reference to battery monitor state structure
     * @param[in] params        Reference to battery monitor parameters
     * @param[in] cell_count    Number of cells in the battery (must match actual battery)
     * 
     * @warning Incorrect cell_count configuration will result in incorrect voltage
     *          readings and potential battery damage.
     */
    AP_BattMonitor_SMBus_SUI(AP_BattMonitor &mon,
                             AP_BattMonitor::BattMonitor_State &mon_state,
                             AP_BattMonitor_Params &params,
                             uint8_t cell_count
                            );

    /**
     * @brief Initialize the SUI battery monitor backend
     * 
     * @details Performs SUI-specific initialization including SMBus interface
     *          setup and initial battery parameter configuration. Called once
     *          during system startup.
     */
    void init(void) override;

private:
    /**
     * @brief Periodic timer callback for battery monitoring
     * 
     * @details Called at regular intervals to perform battery status updates
     *          including voltage readings, current measurements, and health
     *          status monitoring. Implements phased reading strategy to
     *          alternate between pack voltage and individual cell voltages.
     */
    void timer(void) override;
    
    /**
     * @brief Read individual cell voltages from the battery
     * 
     * @details Queries the battery for individual cell voltage measurements
     *          and updates the cell voltage array in the monitor state.
     *          Cell voltages are used for cell balancing and health monitoring.
     */
    void read_cell_voltages();
    
    /**
     * @brief Update battery health status from internal diagnostics
     * 
     * @details Reads battery health status information from the smart battery's
     *          internal diagnostics and updates the monitor state health flags.
     *          Health status may include temperature warnings, voltage imbalance,
     *          and other battery condition indicators.
     */
    void update_health();

    /**
     * @brief Read block data from SMBus register without PEC
     * 
     * @details Performs a block read from the specified SMBus register without
     *          Packet Error Checking. SUI batteries use this simplified protocol
     *          instead of the standard SMBus PEC mechanism.
     * 
     * @param[in]  reg   SMBus register address to read from
     * @param[out] data  Buffer to store read data
     * @param[in]  len   Number of bytes to read
     * 
     * @return true if read successful, false if communication error occurred
     * 
     * @note This method bypasses PEC validation used in standard SMBus transactions
     */
    bool read_block_bare(uint8_t reg, uint8_t* data, uint8_t len) const;

    const uint8_t cell_count;        ///< Fixed number of cells specified at construction (must match physical battery)
    bool phase_voltages;             ///< True when reading cell voltages, false when reading pack voltage (alternates each cycle)
    uint32_t last_volt_read_us;      ///< Timestamp of last voltage read in microseconds (used for read rate limiting)
};

#endif  // AP_BATTERY_SMBUS_SUI_ENABLED
