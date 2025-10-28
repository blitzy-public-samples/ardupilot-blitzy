/**
 * @file AP_BattMonitor_SMBus_Generic.h
 * @brief Generic SMBus smart battery monitor with auto-detection
 * 
 * @details This header defines a generic SMBus smart battery backend that implements
 *          automatic cell count detection and Packet Error Code (PEC) support detection.
 *          Supports batteries with up to 12 cells (14 cells in SITL) using the standard
 *          SMBus Smart Battery Data Specification protocol.
 * 
 * @note Part of the ArduPilot battery monitoring subsystem
 */

#pragma once

#include "AP_BattMonitor_SMBus.h"

#if AP_BATTERY_SMBUS_GENERIC_ENABLED

// Maximum number of battery cells supported - SITL allows more for testing
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#define BATTMONITOR_SMBUS_NUM_CELLS_MAX 14  ///< SITL supports up to 14 cells for testing
#else
#define BATTMONITOR_SMBUS_NUM_CELLS_MAX 12  ///< Hardware limited to 12 cells due to memory constraints
#endif

/**
 * @class AP_BattMonitor_SMBus_Generic
 * @brief Generic SMBus smart battery backend with automatic configuration
 * 
 * @details This backend implements a generic SMBus smart battery interface that:
 *          - Automatically detects the number of battery cells (1-12 cells, or 1-14 in SITL)
 *          - Auto-detects Packet Error Code (PEC) support via SpecificationInfo register
 *          - Polls individual cell voltages for health monitoring
 *          - Implements standard SMBus Smart Battery Data Specification commands
 * 
 *          Cell count detection works by sequentially polling CellVoltage registers
 *          (0x3F for cell 1, 0x3E for cell 2, etc.) until an invalid response is received,
 *          indicating the end of available cells.
 * 
 *          PEC support is detected by reading the SpecificationInfo register and checking
 *          for valid PEC responses. Once confirmed (multiple successful reads), PEC is
 *          enabled for all subsequent transactions to improve data integrity.
 * 
 * @note SITL allows up to 14 cells vs 12 on hardware due to memory constraints
 * @note Cell count auto-detection occurs on first connection and is cached
 * @warning Cell voltage health monitoring tracks last successful read time per cell
 *          to detect communication failures or degraded cells
 */
class AP_BattMonitor_SMBus_Generic : public AP_BattMonitor_SMBus
{
public:

    /**
     * @brief Construct a generic SMBus battery monitor backend
     * 
     * @param[in] mon Reference to parent AP_BattMonitor instance
     * @param[in] mon_state Reference to battery monitor state structure for this instance
     * @param[in] params Reference to battery monitor parameters for this instance
     * 
     * @note Constructor initializes cell count detection state machine and PEC confirmation counters
     */
    AP_BattMonitor_SMBus_Generic(AP_BattMonitor &mon,
                             AP_BattMonitor::BattMonitor_State &mon_state,
                             AP_BattMonitor_Params &params);

private:

    /**
     * @brief Periodic timer callback for battery register reads and cell voltage polling
     * 
     * @details Called periodically (typically 10Hz) to:
     *          - Read standard SMBus registers (voltage, current, capacity, etc.)
     *          - Poll individual cell voltages for all detected cells
     *          - Perform cell count auto-detection if not yet complete
     *          - Detect and enable PEC support if available
     *          - Update health monitoring timestamps for each cell
     * 
     * @note This override implements generic SMBus polling without battery-specific commands
     * @warning Cell voltage read failures update health monitoring state
     */
    void timer(void) override;

    /**
     * @brief Check if Packet Error Code (PEC) is supported by the battery
     * 
     * @details Attempts to detect PEC support by reading the SpecificationInfo register
     *          (0x1A) which returns the Smart Battery specification version. If multiple
     *          consecutive reads with PEC succeed, PEC support is confirmed and enabled
     *          for all subsequent transactions.
     * 
     * @return true once PEC support has been confirmed (either working or not working),
     *         false if detection is still in progress
     * 
     * @note PEC provides CRC-8 error detection for improved data integrity on noisy buses
     * @note Requires _pec_confirmed to reach threshold before returning true
     */
    bool check_pec_support();

    uint8_t _pec_confirmed;  ///< Counter for PEC working confirmation - incremented on successful PEC reads, used to confirm PEC support
    uint32_t _last_cell_update_us[BATTMONITOR_SMBUS_NUM_CELLS_MAX];  ///< System time (microseconds) of last successful read of each cell voltage for health monitoring
    uint32_t _cell_count_check_start_us;  ///< System time (microseconds) when cell count detection started, used to timeout detection
    uint8_t _cell_count;  ///< Detected number of cells returning valid voltages (1-12 or 1-14 in SITL)
    bool _cell_count_fixed;  ///< True when cell count auto-detection is complete and cell count is locked
};

#endif  // AP_BATTERY_SMBUS_GENERIC_ENABLED
