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
 * @file AP_BattMonitor_Bebop.h
 * @brief Battery monitoring backend for Parrot Bebop and Disco platforms
 * 
 * This backend provides battery monitoring capabilities specifically designed
 * for Parrot Bebop and Disco platforms by deriving battery state information
 * from BLDC motor telemetry and RPM data. Unlike traditional battery monitors
 * that directly measure voltage and current, this implementation leverages
 * the motor controller's telemetry to estimate battery parameters.
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_BattMonitor_Backend.h"

/**
 * @class AP_BattMonitor_Bebop
 * @brief Platform-specific battery monitor for Parrot Bebop and Disco
 * 
 * @details This backend is specifically designed for Parrot Bebop and Disco platforms,
 *          where battery state information is derived from BLDC motor telemetry rather
 *          than direct voltage/current measurement. The implementation compensates for
 *          voltage sag under load by using motor RPM data to provide more accurate
 *          battery state estimation.
 *          
 *          Key features:
 *          - Voltage reading from BLDC motor telemetry at approximately 10Hz
 *          - Voltage compensation based on motor RPM to account for load-induced sag
 *          - Low-pass filtering for stable voltage readings
 *          - Battery percentage estimation from compensated voltage
 *          - Current information available from motor telemetry
 *          
 *          Platform dependencies:
 *          - Requires BLDC motor controller telemetry support
 *          - Specific to Parrot hardware architecture
 *          
 * @note This backend uses motor RPM data to compensate for voltage sag under load,
 *       providing more accurate state-of-charge estimation than raw voltage readings
 */
class AP_BattMonitor_Bebop :public AP_BattMonitor_Backend
{
public:
    /**
     * @brief Construct a new AP_BattMonitor_Bebop backend instance
     * 
     * Constructor initializes the Bebop-specific battery monitor backend with
     * inline initialization of voltage filtering state variables. All voltage
     * tracking variables are initialized to 0.0f for clean startup state.
     * 
     * @param[in] mon         Reference to parent AP_BattMonitor instance
     * @param[in] mon_state   Reference to battery monitor state structure
     * @param[in] params      Reference to battery monitor parameters
     */
    AP_BattMonitor_Bebop(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state, AP_BattMonitor_Params &params):
        AP_BattMonitor_Backend(mon, mon_state, params),
        _prev_vbat_raw(0.0f),
        _prev_vbat(0.0f),
        _battery_voltage_max(0.0f)
    {};

    /**
     * @brief Destructor for AP_BattMonitor_Bebop backend
     * 
     * Virtual destructor to ensure proper cleanup through base class pointer.
     * No platform-specific resources require explicit cleanup.
     */
    virtual ~AP_BattMonitor_Bebop(void) {};

    /**
     * @brief Initialize the Bebop battery monitor backend
     * 
     * @details Performs platform-specific initialization for Bebop/Disco battery
     *          monitoring. Sets up access to BLDC motor telemetry interfaces and
     *          initializes voltage filtering parameters for the specific platform.
     *          Must be called before read() operations begin.
     */
    void init() override;

    /**
     * @brief Read the latest battery voltage and state from motor telemetry
     * 
     * @details Retrieves battery voltage information from BLDC motor telemetry at
     *          approximately 10Hz update rate. The reading process includes:
     *          1. Obtaining raw voltage from motor controller telemetry
     *          2. Applying RPM-based compensation for load-induced voltage sag
     *          3. Low-pass filtering for stable readings
     *          4. Computing battery percentage from compensated voltage
     *          5. Updating battery monitor state with current information
     *          
     *          This method should be called regularly at the battery monitor update
     *          rate to maintain current battery state information.
     * 
     * @note Update rate is approximately 10Hz, limited by motor telemetry rate
     */
    void read() override;

    /**
     * @brief Check if current measurement is available
     * 
     * @details The Bebop platform provides current information through the BLDC
     *          motor telemetry system, allowing for both voltage and current
     *          monitoring capabilities.
     * 
     * @return true - Current information is available from motor telemetry
     */
    bool has_current() const override { return true; };

    /**
     * @brief Attempt to reset remaining battery capacity
     * 
     * @details Remaining capacity reset is not supported on the Bebop platform
     *          because battery state is derived from motor telemetry rather than
     *          direct fuel gauge integration. The battery percentage is computed
     *          from voltage readings and cannot be manually overridden.
     * 
     * @param[in] percentage Requested remaining capacity percentage (ignored)
     * 
     * @return false - Reset operation is not supported on Bebop platform
     * 
     * @warning Do not rely on this method for Bebop battery management. Use
     *          voltage-based state estimation provided by the backend instead.
     */
    bool reset_remaining(float percentage) override { return false; }

private:
    float _prev_vbat_raw;        ///< Previous raw voltage reading in volts (before filtering)
    float _prev_vbat;            ///< Previous filtered voltage in volts (after low-pass filter)
    float _battery_voltage_max;  ///< Maximum battery voltage in volts (fully charged state)

    /**
     * @brief Compute voltage compensation based on motor RPM
     * 
     * @details Calculates voltage compensation factor from motor RPM data to account
     *          for load-induced voltage sag. Higher RPM typically indicates higher
     *          current draw, which causes battery voltage to drop below its true
     *          state-of-charge voltage. This method estimates the compensation needed
     *          to recover the actual battery voltage from the measured sagged voltage.
     * 
     * @param[in] rpm       Pointer to motor RPM values from motor controllers
     * @param[in] vbat_raw  Raw battery voltage reading in volts
     * 
     * @return Compensated voltage in volts accounting for load-induced sag
     */
    float _compute_compensation(const uint16_t *rpm, float vbat_raw);

    /**
     * @brief Apply low-pass filter to voltage reading
     * 
     * @details Implements a low-pass filter on raw voltage readings to reduce noise
     *          and provide stable voltage measurements. The filter smooths out
     *          transient voltage fluctuations while maintaining responsiveness to
     *          actual battery discharge.
     * 
     * @param[in] vbat_raw  Raw voltage reading in volts
     * 
     * @return Filtered voltage in volts
     */
    float _filter_voltage(float vbat_raw);

    /**
     * @brief Estimate battery percentage from voltage
     * 
     * @details Computes remaining battery capacity as a percentage based on the
     *          filtered and compensated voltage reading. Uses battery voltage
     *          discharge curve to map voltage to approximate state-of-charge.
     *          The estimation accounts for the non-linear voltage-to-capacity
     *          relationship typical of lithium polymer batteries used in Bebop.
     * 
     * @param[in] vbat  Filtered and compensated battery voltage in volts
     * 
     * @return Estimated battery percentage (0.0 to 100.0)
     */
    float _compute_battery_percentage(float vbat);
};
