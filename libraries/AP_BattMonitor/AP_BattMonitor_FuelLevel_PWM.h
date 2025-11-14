/**
 * @file AP_BattMonitor_FuelLevel_PWM.h
 * @brief PWM-based fuel level sensor integration for battery monitoring
 * 
 * This file defines the AP_BattMonitor_FuelLevel_PWM backend for reading fuel
 * level from PWM-based sensors commonly used in fuel tanks. The backend combines
 * analog voltage measurement (via AP_BattMonitor_Analog) with PWM duty cycle or
 * frequency decoding to determine fuel level percentage.
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

#include "AP_BattMonitor_Analog.h"

#if AP_BATTERY_FUELLEVEL_PWM_ENABLED

#include "AP_BattMonitor.h"

/**
 * @class AP_BattMonitor_FuelLevel_PWM
 * @brief Battery monitor backend for PWM-based fuel level sensors
 * 
 * @details This backend reads fuel level from PWM sensors such as fuel tank level
 *          senders with PWM output. It derives from AP_BattMonitor_Analog to provide
 *          voltage measurement capabilities while adding PWM duty cycle or frequency
 *          decoding to determine fuel level percentage.
 *          
 *          The implementation decodes PWM signals (typically at ~10Hz) to extract
 *          fuel level information, which is then used to calculate consumed energy
 *          and remaining capacity. This is commonly used with automotive-style fuel
 *          level senders that provide PWM output proportional to float position.
 * 
 * @note Common PWM fuel level sensor protocols:
 *       - Frequency-based: 10-180Hz range (e.g., 10Hz = empty, 180Hz = full)
 *       - Duty cycle-based: 10-90% duty cycle proportional to fuel level
 *       - Update rate: Typically 10Hz sampling of PWM input signal
 * 
 * @warning PWM signal timing requirements:
 *          - Ensure PWM signal meets voltage level requirements (3.3V or 5V logic)
 *          - Sensor must be properly calibrated for accurate fuel level reading
 *          - PWM input pin must support hardware PWM capture for reliable decoding
 *          - Signal noise or timing jitter can cause inaccurate fuel level readings
 * 
 * @see AP_BattMonitor_Analog for base voltage measurement functionality
 * @see AP_HAL::PWMSource for PWM input interface details
 */
class AP_BattMonitor_FuelLevel_PWM : public AP_BattMonitor_Analog
{
public:

    /**
     * @brief Constructor for PWM fuel level battery monitor backend
     * 
     * @param[in] mon Reference to parent AP_BattMonitor instance
     * @param[in] mon_state Reference to battery monitor state structure for storing measurements
     * @param[in] params Reference to battery monitor parameters for configuration
     */
    AP_BattMonitor_FuelLevel_PWM(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state, AP_BattMonitor_Params &params);

    /**
     * @brief Read the battery voltage and fuel level from PWM sensor
     * 
     * @details This method performs the following operations:
     *          1. Calls parent AP_BattMonitor_Analog::read() for voltage measurement
     *          2. Decodes PWM signal (duty cycle or frequency) from pwm_source
     *          3. Converts PWM reading to fuel level percentage (0-100%)
     *          4. Updates consumed energy based on fuel level change
     *          5. Calculates remaining capacity from current fuel level
     *          
     *          The PWM signal is typically sampled at ~10Hz and decoded to determine
     *          fuel tank level. The decoding method (frequency vs duty cycle) depends
     *          on sensor type and is configured via parameters.
     * 
     * @note This method should be called at 10Hz for optimal PWM signal decoding
     *       and fuel level tracking accuracy
     */
    void read() override;

    /**
     * @brief Check if this battery monitor provides consumed energy information
     * 
     * @details This backend calculates consumed energy based on fuel level changes
     *          measured from the PWM sensor. As fuel is consumed, the change in
     *          fuel level percentage is converted to energy consumption.
     * 
     * @return true Always returns true as fuel level provides consumed energy data
     */
    bool has_consumed_energy() const override { return true; }

    /**
     * @brief Check if this battery monitor provides current measurement
     * 
     * @details This backend derives current information from the rate of fuel
     *          consumption measured via PWM signal changes. Current is estimated
     *          based on fuel level change rate over time.
     * 
     * @return true Always returns true as current is derived from fuel consumption rate
     */
    bool has_current() const override { return true; }

    /**
     * @brief Initialize the PWM fuel level battery monitor
     * 
     * @details This method performs backend-specific initialization. For the PWM
     *          fuel level monitor, initialization is handled by the parent class
     *          and the PWM source is configured during read() operation.
     */
    void init(void) override {}

private:

    /**
     * @brief HAL PWM input interface for reading fuel level sensor signal
     * 
     * This PWMSource object provides hardware-level PWM signal capture capabilities
     * for measuring duty cycle and frequency from the fuel level sensor. The decoded
     * PWM signal is converted to fuel level percentage during read() operation.
     */
    AP_HAL::PWMSource pwm_source;
};

#endif  // AP_BATTERY_FUELLEVEL_PWM_ENABLED
