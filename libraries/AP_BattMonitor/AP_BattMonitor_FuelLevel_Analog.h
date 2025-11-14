/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Code by Charlie Johnson
 */

/**
 * @file AP_BattMonitor_FuelLevel_Analog.h
 * @brief Analog fuel level sensor integration with polynomial curve fitting
 * 
 * @details This file implements a battery monitor backend that reads analog
 *          fuel level sensors to estimate remaining fuel/energy capacity.
 *          The implementation uses ADC sampling with low-pass filtering for
 *          noise reduction and polynomial curve fitting to compensate for
 *          non-linear tank geometry.
 */

#pragma once

#include "AP_BattMonitor_Backend.h"

#if AP_BATTERY_FUELLEVEL_ANALOG_ENABLED

#include <Filter/LowPassFilter.h>
#include "AP_BattMonitor.h"
#include <Filter/Filter.h>             // Filter library

/**
 * @class AP_BattMonitor_FuelLevel_Analog
 * @brief Battery monitor backend for analog fuel level sensors
 * 
 * @details This backend reads analog fuel level sensors via ADC and converts
 *          the voltage reading into battery capacity metrics. Key features:
 *          - ADC sampling with low-pass filtering for noise reduction
 *          - Polynomial curve fitting to compensate for non-linear tank geometry
 *          - Maps fuel level to battery capacity (consumed energy)
 *          - Provides synthetic current estimate from fuel level rate of change
 *          - Configurable voltage range and polynomial coefficients
 * 
 * @note The polynomial curve fitting allows accurate fuel measurement in tanks
 *       with non-uniform cross-sections by using up to 3rd-order polynomial
 *       coefficients to map voltage to fuel level percentage.
 * 
 * @warning Proper sensor calibration is critical for accurate fuel measurement.
 *          Empty and full tank voltages must be configured correctly, and
 *          polynomial coefficients must be derived from the specific tank
 *          geometry to ensure accurate fuel level tracking.
 */
class AP_BattMonitor_FuelLevel_Analog : public AP_BattMonitor_Backend
{
public:

    /**
     * @brief Constructor for analog fuel level sensor backend
     * 
     * @param[in] mon         Reference to parent AP_BattMonitor instance
     * @param[in] mon_state   Reference to battery monitor state structure
     * @param[in] params      Reference to battery monitor parameters
     */
    AP_BattMonitor_FuelLevel_Analog(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state, AP_BattMonitor_Params &params);

    static const struct AP_Param::GroupInfo var_info[];

    /**
     * @brief Read analog fuel level sensor and update battery state
     * 
     * @details This function performs the following operations:
     *          1. Samples the analog fuel level sensor via ADC (~10Hz rate)
     *          2. Applies low-pass filtering to reduce electrical noise
     *          3. Converts filtered voltage to fuel level percentage using
     *             polynomial curve fitting (compensates for tank geometry)
     *          4. Updates battery monitor state with fuel level as capacity
     *          5. Calculates synthetic current from rate of fuel level change
     * 
     * @note Should be called at approximately 10Hz for optimal filtering
     *       and fuel level tracking accuracy.
     */
    void read() override;

    /**
     * @brief Check if this monitor provides consumed energy information
     * 
     * @return true - fuel level tracking provides consumed energy metrics
     * 
     * @details Fuel level sensors inherently track consumed energy by
     *          measuring remaining fuel, which is converted to consumed
     *          energy based on total tank capacity.
     */
    bool has_consumed_energy() const override { return true; }

    /**
     * @brief Check if this monitor provides current information
     * 
     * @return true - provides synthetic current from fuel level rate of change
     * 
     * @details While fuel level sensors don't directly measure current, this
     *          backend calculates a synthetic current value by measuring the
     *          rate of change of fuel level over time.
     */
    bool has_current() const override { return true; }

    /**
     * @brief Initialize the fuel level sensor backend
     * 
     * @details Initialization is handled by the base class and parameter
     *          system. No additional initialization is required for this
     *          backend as the analog source is configured dynamically.
     */
    void init(void) override {}

private:

    AP_Float _fuel_level_empty_voltage;        ///< Empty tank voltage in volts - ADC reading when fuel tank is empty
    AP_Float _fuel_level_voltage_mult;         ///< Voltage multiplier in volts per unit - scaling factor for ADC conversion
    AP_Float _fuel_level_filter_frequency;     ///< Low-pass filter cutoff frequency in Hz - reduces sensor noise
    AP_Int8  _pin;                             ///< ADC pin number for analog fuel level sensor input
    AP_Float _fuel_fit_first_order_coeff;      ///< First-order polynomial coefficient for tank geometry curve fitting
    AP_Float _fuel_fit_second_order_coeff;     ///< Second-order polynomial coefficient for tank geometry curve fitting
    AP_Float _fuel_fit_third_order_coeff;      ///< Third-order polynomial coefficient for tank geometry curve fitting
    AP_Float _fuel_fit_offset;                 ///< Polynomial offset coefficient for tank geometry curve fitting
    AP_HAL::AnalogSource *_analog_source;      ///< HAL analog input interface for reading ADC voltage from fuel sensor

    LowPassFilterFloat _voltage_filter;        ///< Low-pass filter for noise reduction on analog fuel level voltage reading

};

#endif  // AP_BATTERY_FUELLEVEL_ANALOG_ENABLED
