/**
 * @file AP_BattMonitor_Generator.h
 * @brief Battery monitor backends for generator electrical and fuel monitoring integration
 * 
 * @details This file defines battery monitoring backends that integrate with the AP_Generator
 *          subsystem to monitor both electrical output (voltage, current, energy) and fuel
 *          consumption (level, remaining capacity) from onboard generators. This allows
 *          unified energy management across battery and generator power sources.
 * 
 *          Two specialized backends are provided:
 *          - AP_BattMonitor_Generator_Elec: Monitors generator electrical output
 *          - AP_BattMonitor_Generator_FuelLevel: Monitors generator fuel consumption
 * 
 * @note These backends bridge the AP_Generator subsystem to the battery monitoring
 *       framework, enabling ground control stations to display generator status
 *       alongside battery information through the standard battery monitoring interface.
 * 
 * @see AP_Generator
 * @see AP_BattMonitor_Backend
 */

#pragma once

#include <AP_Generator/AP_Generator.h>

#if HAL_GENERATOR_ENABLED

#include "AP_BattMonitor.h"
#include "AP_BattMonitor_Backend.h"

/**
 * @class AP_BattMonitor_Generator_Elec
 * @brief Battery monitor backend for generator electrical output monitoring
 * 
 * @details This backend monitors the electrical output of a generator system by
 *          retrieving voltage, current, and energy consumption data from the
 *          AP_Generator subsystem. It presents generator electrical characteristics
 *          through the standard battery monitoring interface, allowing unified
 *          power system management and telemetry reporting.
 * 
 *          The backend retrieves real-time electrical measurements from the generator
 *          and implements generator-specific failsafe logic to handle electrical
 *          faults or abnormal operating conditions.
 * 
 * @note This class does not directly interface with generator hardware; it acts
 *       as a bridge between AP_Generator and the battery monitoring framework.
 * 
 * @see AP_Generator
 */
class AP_BattMonitor_Generator_Elec : public AP_BattMonitor_Backend
{
public:

    /**
     * @brief Inherit constructor from AP_BattMonitor_Backend
     * 
     * @details Uses the base class constructor for initialization with monitor
     *          index and backend state parameters.
     */
    using AP_BattMonitor_Backend::AP_BattMonitor_Backend;

    /**
     * @brief Initialize the generator electrical monitoring backend
     * 
     * @details No-op initialization as generator electrical monitoring requires
     *          no backend-specific setup. The AP_Generator subsystem handles
     *          all hardware initialization.
     */
    void init(void) override {};

    /**
     * @brief Read generator voltage and current measurements
     * 
     * @details Retrieves current electrical output measurements from the AP_Generator
     *          subsystem including voltage, current, and consumed energy. Updates the
     *          backend state with the latest values for telemetry reporting and failsafe
     *          evaluation.
     * 
     *          This method is called periodically by the battery monitoring scheduler
     *          to maintain up-to-date generator electrical status.
     * 
     * @note Called at battery monitor update rate (typically 10Hz)
     */
    void read(void) override;

    /**
     * @brief Check if generator provides current measurement
     * 
     * @return true if generator electrical current measurement is available
     * @return false if current measurement is not supported by the generator
     */
    bool has_current(void) const override;

    /**
     * @brief Check if generator provides consumed energy tracking
     * 
     * @return true if generator tracks consumed energy (mAh)
     * @return false if energy consumption tracking is not available
     */
    bool has_consumed_energy(void) const override;

    /**
     * @brief Evaluate generator-specific failsafe conditions
     * 
     * @details Implements failsafe logic specific to generator electrical systems,
     *          checking for conditions such as over-voltage, over-current, or
     *          generator electrical faults reported by the AP_Generator subsystem.
     * 
     * @return AP_BattMonitor::Failsafe enum indicating the current failsafe state
     *         (None, Low, Critical, or other generator-specific conditions)
     * 
     * @note This override allows generator electrical systems to trigger vehicle
     *       failsafe actions based on generator health and electrical parameters.
     */
    AP_BattMonitor::Failsafe update_failsafes() override;
};

/**
 * @class AP_BattMonitor_Generator_FuelLevel
 * @brief Battery monitor backend for generator fuel level and consumption monitoring
 * 
 * @details This backend monitors the fuel consumption of a generator system by
 *          retrieving fuel level and consumption data from the AP_Generator subsystem.
 *          It presents generator fuel status through the battery monitoring interface,
 *          treating fuel capacity analogously to battery capacity for unified energy
 *          management and telemetry.
 * 
 *          The backend supports:
 *          - Fuel level percentage monitoring
 *          - Fuel capacity in milliliters
 *          - Fuel consumption tracking
 *          - Manual fuel level reset for refill scenarios via MAV_CMD_BATTERY_RESET
 * 
 *          By default, the system assumes a 100% full tank at startup. The initial
 *          fuel level can be adjusted via MAVLink command to support partial fills
 *          or refueling operations.
 * 
 * @note This class bridges generator fuel monitoring to the battery monitoring
 *       framework, allowing fuel status to be displayed alongside battery status
 *       in ground control stations.
 * 
 * @see AP_Generator
 */
class AP_BattMonitor_Generator_FuelLevel : public AP_BattMonitor_Backend
{
public:

    /**
     * @brief Inherit constructor from AP_BattMonitor_Backend
     * 
     * @details Uses the base class constructor for initialization with monitor
     *          index and backend state parameters.
     */
    using AP_BattMonitor_Backend::AP_BattMonitor_Backend;

    /**
     * @brief Initialize the generator fuel level monitoring backend
     * 
     * @details Performs backend-specific initialization for fuel monitoring,
     *          setting up initial fuel level state if not provided by the
     *          generator system.
     */
    void init(void) override;

    /**
     * @brief Read generator fuel level and consumption data
     * 
     * @details Retrieves current fuel level information from the AP_Generator
     *          subsystem including remaining fuel percentage and consumed fuel
     *          volume. Updates the backend state to reflect current fuel status
     *          for telemetry reporting and low-fuel failsafe evaluation.
     * 
     * @note Called at battery monitor update rate (typically 10Hz)
     */
    void read(void) override;

    /**
     * @brief Indicate that fuel level monitoring reports current-equivalent data
     * 
     * @details Returns true to enable fuel level reporting through the battery
     *          monitoring interface. This allows the battery monitor to treat
     *          fuel consumption analogously to electrical current consumption.
     * 
     * @return true to enable fuel level remaining calculations
     * @return false if fuel level should not be reported
     */
    bool has_current(void) const override;

    /**
     * @brief Indicate that fuel consumption tracking is available
     * 
     * @details Returns true to enable consumed fuel reporting through the battery
     *          monitoring interface. This allows tracking of total fuel consumed
     *          analogously to battery energy consumption (mAh).
     * 
     * @return true to enable fuel consumption tracking
     * @return false if fuel consumption should not be tracked
     */
    bool has_consumed_energy(void) const override;

    /**
     * @brief Reset fuel level to specified percentage
     * 
     * @details Allows manual adjustment of the fuel level, typically used after
     *          refueling operations. This is invoked by MAV_CMD_BATTERY_RESET
     *          MAVLink command to set a new fuel level baseline.
     * 
     * @param[in] percentage Fuel level percentage to set (0-100)
     * 
     * @return true if fuel level was successfully reset
     * @return false if reset failed or percentage is invalid
     * 
     * @note Used to support tank refills or initial fuel levels different from 100%
     */
    bool reset_remaining(float percentage) override;

    /**
     * @brief Get remaining fuel capacity in milliliters
     * 
     * @details Calculates the remaining fuel volume in milliliters based on
     *          current fuel level percentage and total tank capacity reported
     *          by the generator system.
     * 
     * @param[out] capacity_ml Remaining fuel capacity in milliliters (mL)
     * 
     * @return true if fuel capacity is valid and capacity_ml was updated
     * @return false if fuel capacity cannot be determined
     * 
     * @note Fuel volume calculation requires the generator to provide tank
     *       capacity information
     */
    bool capacity_remaining_ml(float &capacity_ml) const;

    /**
     * @brief Get remaining fuel capacity as percentage
     * 
     * @details Retrieves the current fuel level as a percentage of total tank
     *          capacity from the AP_Generator subsystem. If the generator does
     *          not provide fuel percentage directly, uses the _initial_fuel_pct
     *          value adjusted for consumed fuel.
     * 
     * @param[out] percentage Remaining fuel percentage (0-100)
     * 
     * @return true if fuel percentage is valid and percentage was updated
     * @return false if fuel percentage cannot be determined
     */
    bool capacity_remaining_pct(uint8_t &percentage) const override;

    /**
     * @brief Evaluate generator fuel-level-specific failsafe conditions
     * 
     * @details Implements failsafe logic for generator fuel systems, checking
     *          for low fuel conditions and triggering appropriate failsafe actions
     *          (low fuel warning, critical fuel level, fuel system faults).
     * 
     * @return AP_BattMonitor::Failsafe enum indicating the current failsafe state
     *         (None, Low, Critical based on remaining fuel percentage)
     * 
     * @note Allows fuel depletion to trigger vehicle failsafe actions such as
     *       return-to-launch or landing procedures
     */
    AP_BattMonitor::Failsafe update_failsafes() override;
    
    /**
     * @brief Default initial fuel level percentage assumption
     * 
     * @details By default, the system assumes a 100% full tank at startup. This value
     *          can be overridden using the MAV_CMD_BATTERY_RESET MAVLink command to
     *          specify a different initial fuel level. This variable is only used when
     *          the generator does not provide its own fuel percentage measurement.
     * 
     *          This variable is essential for supporting:
     *          - Tank refills to less than 100% capacity
     *          - Initial startup with partial fuel load
     *          - Manual fuel level calibration after refueling
     * 
     * @note Value range: 0-100 representing fuel percentage
     * @see MAV_CMD_BATTERY_RESET
     */
    uint8_t _initial_fuel_pct = 100.0f;
};
#endif
