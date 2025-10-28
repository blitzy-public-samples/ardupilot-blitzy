/**
 * @file AP_BattMonitor_SMBus_Solo.h
 * @brief 3DR Solo smart battery specific implementation
 * 
 * This file implements the SMBus battery monitor backend specifically
 * designed for the 3DR Solo smart battery, which includes custom register
 * extensions beyond standard SMBus specifications for enhanced telemetry.
 */

#pragma once

#include "AP_BattMonitor_SMBus.h"

#if AP_BATTERY_SMBUS_SOLO_ENABLED

/**
 * @class AP_BattMonitor_SMBus_Solo
 * @brief Battery monitor backend for 3DR Solo smart battery
 * 
 * @details This backend is specific to the 3DR Solo smart battery which
 *          implements custom register extensions beyond the standard SMBus
 *          battery specification. The Solo battery provides extended telemetry
 *          data including additional cell voltage information and temperature
 *          readings accessed through vendor-specific registers.
 *          
 *          The backend also implements button press detection, allowing users
 *          to interact with the battery's physical button to request extended
 *          telemetry data display.
 * 
 * @note Solo battery extended registers provide additional cell voltage and
 *       temperature data not available through standard SMBus commands.
 * @note Button press detection enables user interaction with the battery
 *       display for viewing detailed status information.
 */
class AP_BattMonitor_SMBus_Solo : public AP_BattMonitor_SMBus
{
public:

    /**
     * @brief Construct a new AP_BattMonitor_SMBus_Solo backend
     * 
     * @param[in] mon           Reference to the parent AP_BattMonitor object
     * @param[in] mon_state     Reference to the battery monitor state structure for this instance
     * @param[in] params        Reference to the battery monitor parameter set for this instance
     */
    AP_BattMonitor_SMBus_Solo(AP_BattMonitor &mon,
                             AP_BattMonitor::BattMonitor_State &mon_state,
                             AP_BattMonitor_Params &params);

private:

    /**
     * @brief Periodic timer callback for reading Solo battery registers
     * 
     * @details Performs periodic register reads from the Solo smart battery,
     *          including standard SMBus battery data and Solo-specific extended
     *          telemetry registers. This method reads additional cell voltage
     *          data, temperature information, and monitors button press events
     *          for user interaction with the battery display.
     */
    void timer(void) override;

    uint8_t _button_press_count;  ///< Count of battery button presses for extended data request
    bool _use_extended;           ///< True when extended Solo telemetry registers are available
};

#endif  // AP_BATTERY_SMBUS_SOLO_ENABLED
