/**
 * @file AP_BattMonitor_Torqeedo.h
 * @brief Battery monitor backend for Torqeedo electric outboard motors
 * 
 * This file implements battery monitoring integration with Torqeedo electric
 * motors for marine applications. It retrieves battery state information
 * (voltage, current, temperature, capacity) from Torqeedo electric outboard
 * motors through the AP_Torqeedo library, bridging motor battery telemetry
 * to the ArduPilot battery monitoring subsystem.
 */

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
#pragma once

#include "AP_BattMonitor_config.h"

#if AP_BATTERY_TORQEEDO_ENABLED

#include <AP_Torqeedo/AP_Torqeedo.h>

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_BattMonitor_Backend.h"

/**
 * @class AP_BattMonitor_Torqeedo
 * @brief Battery monitor backend for Torqeedo electric outboard motors
 * 
 * @details This backend retrieves battery state from Torqeedo electric outboard
 *          motors via the AP_Torqeedo library, bridging motor battery telemetry
 *          to the ArduPilot battery monitoring subsystem. It provides voltage,
 *          current, temperature, and capacity information from the Torqeedo
 *          motor's integrated battery management system.
 *          
 *          The Torqeedo integration is particularly useful for marine applications
 *          where the electric motor's battery system is the primary power source
 *          and the motor provides comprehensive battery telemetry.
 * 
 * @note This backend is specifically designed for integration with Torqeedo
 *       electric motors used in marine applications (boats, underwater vehicles)
 * @see AP_Torqeedo library for motor communication details
 * @warning Battery percentage accuracy depends on the Torqeedo battery management
 *          system calibration. Ensure the Torqeedo system is properly calibrated
 *          for accurate state-of-charge reporting.
 */
class AP_BattMonitor_Torqeedo: public AP_BattMonitor_Backend
{
public:

    /**
     * @brief Inherit constructor from AP_BattMonitor_Backend
     * 
     * Uses the base class constructor for initialization
     */
    using AP_BattMonitor_Backend::AP_BattMonitor_Backend;

    /**
     * @brief Read the latest battery state from Torqeedo motor
     * 
     * @details Retrieves periodic battery state information from the Torqeedo
     *          motor communication interface via the AP_Torqeedo library. This
     *          includes voltage, current, temperature, and capacity data from
     *          the motor's battery management system. The method is called
     *          periodically by the battery monitoring subsystem to update
     *          battery state.
     */
    void read() override;

    /**
     * @brief Check if current measurement is available
     * 
     * @return true if Torqeedo has provided battery information including current,
     *         false if no battery telemetry received yet
     */
    bool has_current() const override { return have_info; };

    /**
     * @brief Check if temperature measurement is available
     * 
     * @return true if Torqeedo has provided battery information including temperature,
     *         false if no battery telemetry received yet
     */
    bool has_temperature() const override { return have_info; };

    /**
     * @brief Get remaining battery capacity as a percentage
     * 
     * @param[out] percentage Battery remaining capacity (0-100%)
     * 
     * @return true if battery capacity percentage is available and has been written
     *         to the percentage parameter, false if capacity data not yet received
     *         from Torqeedo
     * 
     * @warning The WARN_IF_UNUSED attribute ensures return value is checked for
     *          safety-critical battery monitoring. Battery percentage accuracy
     *          depends on Torqeedo battery management system calibration.
     */
    bool capacity_remaining_pct(uint8_t &percentage) const override WARN_IF_UNUSED;

private:

    bool have_info;             ///< true after first successful Torqeedo battery data reception
    bool have_capacity;         ///< true once capacity value has been received from Torqeedo
    uint8_t remaining_pct;      ///< battery remaining capacity as percentage (0-100)
};

#endif
