/**
 * @file AP_BattMonitor_SMBus_Maxell.h
 * @brief Maxell-specific SMBus smart battery monitor backend
 * 
 * This file implements battery monitoring support for Maxell smart batteries
 * that communicate via SMBus protocol. Maxell batteries use a vendor-specific
 * capacity reporting format that requires scaling to obtain actual milliamp-hour values.
 */

#pragma once

#include "AP_BattMonitor_SMBus_Generic.h"

#if AP_BATTERY_SMBUS_MAXELL_ENABLED

/**
 * @class AP_BattMonitor_SMBus_Maxell
 * @brief Battery monitor backend for Maxell SMBus smart batteries
 * 
 * @details This backend provides support for Maxell-branded smart batteries that
 *          communicate via the SMBus (System Management Bus) protocol. It derives
 *          from AP_BattMonitor_SMBus_Generic and customizes the capacity scaling
 *          behavior to account for Maxell's vendor-specific capacity reporting.
 *          
 *          Maxell smart batteries report capacity values in a non-standard format
 *          that requires a 2x multiplier to convert to actual milliamp-hours (mAh).
 *          This backend applies the appropriate scaling factor to ensure accurate
 *          capacity readings for battery percentage calculations, remaining capacity
 *          estimates, and flight time predictions.
 *          
 *          The backend inherits all standard SMBus smart battery functionality
 *          including voltage, current, temperature monitoring, and state of charge
 *          reporting from the generic SMBus implementation.
 * 
 * @note Maxell batteries report capacity in units that are half the actual mAh value,
 *       requiring the 2x scaler for correct capacity calculations.
 * 
 * @see AP_BattMonitor_SMBus_Generic
 * @see Smart Battery Data Specification (SBS) v1.1 for capacity register definitions
 */
class AP_BattMonitor_SMBus_Maxell : public AP_BattMonitor_SMBus_Generic
{
    /**
     * @brief Inherit constructor from parent AP_BattMonitor_SMBus_Generic class
     * 
     * Uses the parent class constructor to initialize the SMBus communication
     * and battery monitoring infrastructure.
     */
    using AP_BattMonitor_SMBus_Generic::AP_BattMonitor_SMBus_Generic;

private:

    /**
     * @brief Get the capacity scaling factor for Maxell battery capacity values
     * 
     * @details Returns the multiplier that must be applied to capacity values reported
     *          by Maxell smart batteries to convert them to actual milliamp-hours (mAh).
     *          Maxell batteries use a vendor-specific capacity reporting format where
     *          the raw capacity values must be multiplied by 2 to obtain the true
     *          battery capacity.
     *          
     *          This scaler is applied to:
     *          - Full charge capacity (FCC) register readings
     *          - Remaining capacity register readings
     *          - Design capacity values
     *          
     *          Without this scaler, the battery monitor would report capacity values
     *          that are half of the actual battery capacity, leading to incorrect
     *          state of charge calculations and premature low battery warnings.
     * 
     * @return uint16_t Capacity scaling factor (always 2 for Maxell batteries)
     * 
     * @note This peculiarity is specific to Maxell's implementation of the Smart Battery
     *       Data Specification and differs from most other SMBus smart batteries which
     *       report capacity directly in mAh (requiring a scaler of 1).
     * 
     * @see Smart Battery Data Specification v1.1, Section 5.1 (Capacity Registers)
     */
    uint16_t get_capacity_scaler() const override { return 2; }

};

#endif  // AP_BATTERY_SMBUS_MAXELL_ENABLED
