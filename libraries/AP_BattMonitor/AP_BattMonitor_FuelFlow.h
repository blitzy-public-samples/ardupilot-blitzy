/**
 * @file AP_BattMonitor_FuelFlow.h
 * @brief Fuel flow sensor integration via PWM pulse counting for battery/fuel monitoring
 * 
 * This file implements battery monitoring through fuel flow sensors that output
 * PWM pulse trains, where pulse frequency is proportional to fuel flow rate.
 * The backend converts fuel consumption into battery capacity metrics for
 * integration with the ArduPilot battery monitoring system.
 */

#pragma once

#include "AP_BattMonitor_Analog.h"

#if AP_BATTERY_FUELFLOW_ENABLED

#include "AP_BattMonitor.h"

/**
 * @class AP_BattMonitor_FuelFlow
 * @brief Battery monitor backend that measures fuel flow rate via GPIO pin interrupts
 * 
 * @details This backend monitors fuel consumption by counting PWM pulses from a fuel
 *          flow sensor connected to a GPIO pin. Each pulse represents a fixed volume
 *          of fuel (typically specified in pulses-per-liter). The backend converts
 *          cumulative fuel consumption to battery capacity metrics and instantaneous
 *          flow rate to current draw, allowing fuel-powered systems to integrate with
 *          ArduPilot's standard battery monitoring and failsafe systems.
 *          
 *          Pulse counting is performed in an interrupt handler for accurate timing,
 *          and consumption/flow calculations are performed at ~10Hz in the read() method.
 * 
 * @note Typical fuel flow sensors provide calibration in pulses-per-liter (e.g., 5600 pulses/L).
 *       Configure via BATT_FL_MULTIPLIER parameter.
 * 
 * @warning Interrupt timing constraints require careful pin configuration. Ensure the
 *          GPIO pin supports interrupt capability and is not shared with other
 *          time-critical functions. High pulse frequencies may impact system performance.
 */
class AP_BattMonitor_FuelFlow : public AP_BattMonitor_Analog
{
public:

    /**
     * @brief Constructor for fuel flow battery monitor backend
     * 
     * @param[in] mon Reference to parent AP_BattMonitor instance
     * @param[in] mon_state Reference to battery monitor state structure for this instance
     * @param[in] params Reference to battery monitor parameters for this instance
     */
    AP_BattMonitor_FuelFlow(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state, AP_BattMonitor_Params &params);

    /**
     * @brief Read fuel flow data and update battery state at ~10Hz
     * 
     * @details This method calculates instantaneous fuel flow rate from the PWM pulse
     *          frequency captured by the interrupt handler. It converts pulse count
     *          and timing data into:
     *          - Fuel flow rate (volume/time) mapped to battery current
     *          - Cumulative fuel consumption mapped to consumed energy/mAh
     *          - Remaining fuel capacity mapped to battery percentage
     *          
     *          The method reads interrupt state atomically, calculates flow from
     *          pulse frequency using the configured pulses-per-liter multiplier,
     *          and updates the battery monitor state structure.
     *          
     *          Should be called at approximately 10Hz for accurate flow rate calculation
     *          and smooth current reporting.
     * 
     * @note Inherits voltage reading from AP_BattMonitor_Analog parent class
     */
    void read() override;

    /**
     * @brief Indicates if consumed energy information is available
     * 
     * @details This backend calculates consumed energy from cumulative fuel consumption
     *          measured via pulse counting. Total fuel volume consumed is converted to
     *          energy/mAh metrics for integration with battery monitoring.
     * 
     * @return true - consumed energy is calculated from cumulative fuel consumption
     */
    bool has_consumed_energy() const override { return true; }

    /**
     * @brief Indicates if current information is available
     * 
     * @details This backend synthesizes current from instantaneous fuel flow rate.
     *          The pulse frequency from the fuel flow sensor is converted to volume/time,
     *          then mapped to battery current (mA) for display and failsafe triggering.
     * 
     * @return true - current is synthesized from instantaneous fuel flow rate
     */
    bool has_current() const override { return true; }

    /**
     * @brief Initialize fuel flow monitor backend
     * 
     * @details Initialization is handled by parent class and interrupt registration.
     *          This override is provided for interface compliance.
     */
    void init(void) override {}

private:
    /**
     * @brief Interrupt handler for fuel flow sensor PWM pulse counting
     * 
     * @details Called by GPIO interrupt system on each pulse edge from the fuel flow sensor.
     *          Atomically updates pulse count and timing information used by read() to
     *          calculate flow rate. Executes in interrupt context with strict timing
     *          constraints - must complete quickly to avoid impacting system performance.
     * 
     * @param[in] pin GPIO pin number that triggered the interrupt
     * @param[in] pin_state Current state of the pin (HIGH/LOW)
     * @param[in] timestamp Microsecond timestamp of the interrupt event
     */
    void irq_handler(uint8_t pin, bool pin_state, uint32_t timestamp);

    /**
     * @brief Interrupt state structure for atomic pulse data capture
     */
    struct IrqState {
        uint32_t pulse_count;     ///< Cumulative pulse count since initialization
        uint32_t total_us;        ///< Total duration in microseconds for pulse timing calculations
        uint32_t last_pulse_us;   ///< Timestamp of last pulse in microseconds for frequency calculation
    } irq_state;

    int8_t last_pin = -1;  ///< GPIO pin number for fuel flow sensor input tracking, -1 if not configured
};

#endif  // AP_BATTERY_FUELFLOW_ENABLED
