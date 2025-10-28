/**
 * @file AP_BattMonitor_SMBus_Rotoye.h
 * @brief Rotoye Batmon smart battery specific SMBus implementation
 * 
 * @details This file implements the battery monitor backend for Rotoye Batmon
 *          smart batteries. Rotoye Batmon batteries are SMBus-compatible smart
 *          batteries that provide enhanced monitoring capabilities including
 *          dual temperature sensor support for comprehensive thermal monitoring.
 * 
 * @note This backend extends the generic SMBus battery monitor with Rotoye-specific
 *       features, particularly dual temperature sensing for both internal battery
 *       core temperature and external surface temperature monitoring.
 * 
 * Source: libraries/AP_BattMonitor/AP_BattMonitor_SMBus_Rotoye.h
 */

#pragma once

#include "AP_BattMonitor_SMBus_Generic.h"

#if AP_BATTERY_SMBUS_ROTOYE_ENABLED

/**
 * @class AP_BattMonitor_SMBus_Rotoye
 * @brief Battery monitor backend for Rotoye Batmon smart batteries
 * 
 * @details This class implements the SMBus battery monitor interface specifically
 *          for Rotoye Batmon smart batteries. It derives from the generic SMBus
 *          battery monitor (AP_BattMonitor_SMBus_Generic) and extends it with
 *          Rotoye-specific functionality.
 * 
 *          Key features of Rotoye Batmon batteries:
 *          - SMBus-compatible smart battery interface
 *          - Dual temperature sensor support (internal core + external surface)
 *          - Enhanced thermal monitoring for improved safety
 *          - Standard SMBus commands for voltage, current, capacity
 * 
 *          The primary difference from generic SMBus batteries is the dual
 *          temperature sensor capability, which allows monitoring both the
 *          battery core temperature (internal sensor) and surface temperature
 *          (external sensor) for more comprehensive thermal management.
 * 
 * @note Rotoye Batmon provides two independent temperature readings, enabling
 *       better detection of thermal issues and improved battery health monitoring
 *       compared to single-sensor implementations.
 * 
 * @see AP_BattMonitor_SMBus_Generic for base SMBus battery functionality
 * @see https://rotoye.com for Rotoye Batmon specifications
 */
class AP_BattMonitor_SMBus_Rotoye : public AP_BattMonitor_SMBus_Generic
{
    /**
     * @brief Inherit constructor from AP_BattMonitor_SMBus_Generic
     * 
     * @details Uses the parent class constructor directly, as no additional
     *          initialization is required beyond the standard SMBus setup.
     *          Rotoye-specific behavior is implemented through method overrides.
     */
    using AP_BattMonitor_SMBus_Generic::AP_BattMonitor_SMBus_Generic;

private:

    /**
     * @brief Read temperature from Rotoye Batmon dual temperature sensors
     * 
     * @details This method overrides the generic SMBus temperature reading to
     *          support Rotoye Batmon's dual temperature sensor capability.
     *          
     *          Rotoye Batmon smart batteries provide two independent temperature
     *          readings via SMBus:
     *          - Internal temperature sensor: Monitors battery core temperature
     *          - External temperature sensor: Monitors battery surface temperature
     *          
     *          This dual-sensor approach provides enhanced thermal monitoring
     *          compared to standard single-sensor smart batteries. The internal
     *          sensor provides early detection of cell-level thermal issues,
     *          while the external sensor monitors environmental and surface
     *          cooling effectiveness.
     *          
     *          The temperature readings are used for:
     *          - Battery health monitoring and degradation tracking
     *          - Thermal failsafe protection
     *          - Charge/discharge rate limiting based on temperature
     *          - User feedback through telemetry and OSD
     * 
     * @note Rotoye Batmon's dual temperature sensor capability enables more
     *       accurate thermal management and early detection of thermal anomalies
     *       that might indicate battery damage or unsafe operating conditions.
     * 
     * @warning Temperature monitoring is critical for battery safety. Thermal
     *          runaway detection relies on accurate temperature readings.
     * 
     * @see AP_BattMonitor_SMBus_Generic::read_temp() for base implementation
     * @see https://rotoye.com for Rotoye Batmon temperature sensor specifications
     */
    void read_temp(void) override;

};

#endif  // AP_BATTERY_SMBUS_ROTOYE_ENABLED
