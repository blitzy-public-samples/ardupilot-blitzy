/**
 * @file AP_BattMonitor_EFI.h
 * @brief Battery monitor backend that derives state from Electronic Fuel Injection (EFI) system
 * 
 * This file implements battery monitoring for internal combustion engine systems
 * by integrating with the EFI system to map fuel consumption to battery capacity metrics.
 * Fuel usage is treated as energy consumption for monitoring purposes.
 */

#pragma once

#include "AP_BattMonitor_Backend.h"

#if AP_BATTERY_EFI_ENABLED

/**
 * @class AP_BattMonitor_EFI
 * @brief Battery monitor backend for EFI-equipped internal combustion engines
 * 
 * @details This backend derives battery state from fuel consumption data reported by
 *          the Electronic Fuel Injection (EFI) system. It maps fuel usage to battery
 *          capacity metrics, treating fuel consumption as energy consumption for
 *          monitoring and logging purposes.
 *          
 *          The backend synthesizes current measurements from fuel flow rate data
 *          and calculates consumed energy from cumulative fuel consumption reported
 *          by the EFI controller.
 * 
 * @note This backend is specifically used for internal combustion engine monitoring
 *       where fuel consumption is treated as equivalent to battery energy consumption
 *       for system monitoring, telemetry, and failsafe purposes.
 * 
 * @see AP_EFI for Electronic Fuel Injection system implementation details
 */
class AP_BattMonitor_EFI : public AP_BattMonitor_Backend
{
public:

    /**
     * @brief Constructor inherited from AP_BattMonitor_Backend
     * 
     * Uses the base class constructor for standard battery monitor initialization.
     */
    using AP_BattMonitor_Backend::AP_BattMonitor_Backend;

    /**
     * @brief Update battery state from EFI fuel consumption data
     * 
     * @details Performs periodic update of battery monitor state by reading current
     *          fuel consumption data from the EFI system. Converts fuel flow rate
     *          to synthesized current measurement and cumulative fuel consumption
     *          to consumed energy metrics.
     *          
     *          This method is called periodically by the battery monitor framework
     *          to refresh state information used for telemetry, logging, and
     *          failsafe monitoring.
     */
    void read(void) override;

    /**
     * @brief Check if current measurement is available
     * 
     * @return true - Current is synthesized from EFI fuel flow rate data
     * 
     * @details This backend always reports current availability because it
     *          synthesizes current measurements from the fuel flow rate
     *          reported by the EFI system.
     */
    bool has_current(void) const override {
        return true;
    }

    /**
     * @brief Check if consumed energy measurement is available
     * 
     * @return true - Consumed energy is calculated from cumulative fuel consumption
     * 
     * @details This backend always reports consumed energy availability because it
     *          calculates consumed energy from the cumulative fuel consumption
     *          reported by the EFI controller.
     */
    bool has_consumed_energy(void) const override {
        return true;
    }
};
#endif // AP_BATTERY_EFI_ENABLED
