/**
 * @file AP_BattMonitor_SMBus_NeoDesign.h
 * @brief NeoDesign smart battery specific implementation for SMBus battery monitoring
 * 
 * This file implements the battery monitor backend for NeoDesign smart batteries,
 * which use a custom register layout and support individual cell voltage monitoring
 * for batteries with up to 10 cells.
 */

#pragma once

#include "AP_BattMonitor_SMBus.h"

#if AP_BATTERY_SMBUS_NEODESIGN_ENABLED

/**
 * @class AP_BattMonitor_SMBus_NeoDesign
 * @brief Battery monitor backend for NeoDesign smart batteries
 * 
 * @details This backend implements support for NeoDesign smart batteries which use
 *          a custom SMBus register layout that differs from the standard Smart Battery
 *          Data Specification. Key features include:
 *          - Support for batteries with up to 10 cells
 *          - Individual cell voltage monitoring via NeoDesign-specific registers
 *          - Custom register addresses for battery telemetry data
 *          - Extended battery health and status information
 * 
 * @note NeoDesign batteries use a proprietary register layout that is not compatible
 *       with standard SMBus Smart Battery registers. This backend handles the
 *       NeoDesign-specific protocol for reading cell voltages and other telemetry.
 */
class AP_BattMonitor_SMBus_NeoDesign : public AP_BattMonitor_SMBus
{
public:
    /**
     * @brief Constructor for NeoDesign smart battery monitor
     * 
     * @param[in] mon          Reference to the parent battery monitor instance
     * @param[in] mon_state    Reference to the battery state structure for storing measurements
     * @param[in] params       Reference to battery monitor parameters including I2C bus and address configuration
     */
    AP_BattMonitor_SMBus_NeoDesign(AP_BattMonitor &mon,
                             AP_BattMonitor::BattMonitor_State &mon_state,
                             AP_BattMonitor_Params &params);

private:

    /**
     * @brief Periodic timer callback for reading battery telemetry
     * 
     * @details Called at regular intervals to read battery data from NeoDesign-specific
     *          SMBus registers including voltage, current, temperature, remaining capacity,
     *          and individual cell voltages. This override implements the custom register
     *          access pattern required for NeoDesign batteries.
     */
    void timer(void) override;

    uint8_t _cell_count;  ///< Detected or configured number of battery cells (1-10)

    static const constexpr uint8_t max_cell_count = 10;  ///< Maximum number of cells supported by NeoDesign batteries
};

#endif  // AP_BATTERY_SMBUS_NEODESIGN_ENABLED
