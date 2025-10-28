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
 * @file AP_BattMonitor_ESC.h
 * @brief ESC telemetry aggregation backend for battery monitoring
 * 
 * This file implements battery monitoring by aggregating telemetry data
 * from multiple Electronic Speed Controllers (ESCs). It provides voltage,
 * current, temperature, and mAh consumption measurements by combining
 * data from all ESCs specified by the mask parameter.
 */

#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_ESC_Telem/AP_ESC_Telem.h>
#include "AP_BattMonitor_Backend.h"

#if AP_BATTERY_ESC_ENABLED

/**
 * @class AP_BattMonitor_ESC
 * @brief Battery monitor backend that aggregates telemetry from multiple ESCs
 * 
 * @details This backend aggregates voltage, current, temperature, and mAh consumption
 *          data from ESC telemetry across multiple ESCs specified by a bitmask parameter.
 *          It subscribes to ESC telemetry data and combines measurements from all selected
 *          ESCs to provide comprehensive battery monitoring information.
 *          
 *          The aggregation process sums currents and consumed mAh from all ESCs, averages
 *          voltages, and reports the highest temperature detected across all monitored ESCs.
 *          
 *          Telemetry is typically read at approximately 10Hz, with data freshness validated
 *          to ensure accurate real-time monitoring.
 * 
 * @note ESC telemetry is typically available from BLHeli_32/BLHeli_S ESCs or DroneCAN ESCs.
 *       Not all ESC firmware versions provide complete telemetry (voltage, current, temperature, consumed mAh).
 * 
 * @warning ESC telemetry quality varies by ESC hardware and firmware. Current and consumed mAh
 *          measurements require proper ESC calibration for accuracy. Temperature readings may
 *          lag actual ESC temperatures due to sensor placement and update rates.
 */
class AP_BattMonitor_ESC :public AP_BattMonitor_Backend
{
public:
    /**
     * @brief Constructor that initializes the ESC telemetry battery monitor
     * 
     * Initializes the battery monitor backend with ESC mask configuration.
     * This incorporates initialization of telemetry subscription and state tracking.
     * 
     * @param[in] mon          Reference to parent AP_BattMonitor object
     * @param[in] mon_state    Reference to battery monitor state structure
     * @param[in] params       Reference to battery monitor parameters including ESC mask
     */
    AP_BattMonitor_ESC(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state, AP_BattMonitor_Params &params);

    /**
     * @brief Destructor for ESC telemetry battery monitor
     * 
     * Cleans up ESC telemetry subscription and releases resources.
     */
    virtual ~AP_BattMonitor_ESC(void) {};

    /**
     * @brief Initialize ESC telemetry subscription for battery monitoring
     * 
     * Sets up subscription to ESC telemetry data based on configured ESC mask.
     * Prepares internal state tracking for voltage, current, temperature, and
     * consumed mAh aggregation from selected ESCs.
     */
    void init() override;

    /**
     * @brief Read and aggregate latest battery data from ESC telemetry
     * 
     * @details Aggregates ESC telemetry data from all ESCs specified by the mask parameter.
     *          This method is typically called at approximately 10Hz by the battery monitor
     *          scheduler. It performs the following aggregation operations:
     *          
     *          - Voltage: Averages voltage readings from all reporting ESCs
     *          - Current: Sums current draw from all ESCs to get total system current
     *          - Temperature: Reports the maximum temperature across all ESCs
     *          - Consumed mAh: Accumulates incremental mAh consumption since last read
     *          
     *          Only processes telemetry data that is fresh (updated within timeout period).
     *          Updates internal state with aggregated values for access by flight controller.
     */
    void read() override;

    /**
     * @brief Check if current measurement is available from ESC telemetry
     * 
     * @return true if at least one ESC in the configured mask reports current telemetry
     * @return false if no ESCs report current data
     */
    bool has_current() const override { return have_current; };

    /**
     * @brief Check if temperature measurement is available from ESC telemetry
     * 
     * @return true if at least one ESC in the configured mask reports temperature telemetry
     * @return false if no ESCs report temperature data
     */
    bool has_temperature() const override { return have_temperature; };

    /**
     * @brief Reset consumed mAh tracking to match specified remaining percentage
     * 
     * @details Resets the consumed mAh counter based on the specified remaining battery
     *          percentage. This requires reconciliation across all monitored ESCs since
     *          individual ESCs may reset their consumed mAh counters independently.
     *          
     *          The backend calculates the new consumed mAh value based on battery capacity
     *          and the desired remaining percentage, then adjusts internal tracking to
     *          maintain consistency across ESC telemetry resets.
     * 
     * @param[in] percentage  Desired remaining battery percentage (0.0 to 100.0)
     * 
     * @return true if reset was successful
     * @return false if reset failed (e.g., no ESC telemetry available, invalid percentage)
     * 
     * @note ESC telemetry counters may reset independently on ESC power cycles or firmware
     *       resets. This method helps maintain accurate consumed mAh tracking across such events.
     */
    virtual bool reset_remaining(float percentage) override;

    static const struct AP_Param::GroupInfo var_info[];

private:

    AP_Int32  _mask;  ///< Bitmask selecting which ESCs to monitor (bit 0 = ESC 0, bit 1 = ESC 1, etc.)

    bool have_current;        ///< True if at least one monitored ESC reports current telemetry
    bool have_consumed_mah;   ///< True if cumulative consumed mAh data is available from ESC telemetry
    bool have_temperature;    ///< True if at least one monitored ESC reports temperature telemetry
    float delta_mah;          ///< Incremental consumed mAh since last read, in milliamp-hours

    uint32_t last_read_us;    ///< Timestamp of last telemetry read in microseconds
};

#endif  // AP_BATTERY_ESC_ENABLED
