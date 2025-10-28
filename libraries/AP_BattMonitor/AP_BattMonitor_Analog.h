/**
 * @file AP_BattMonitor_Analog.h
 * @brief Analog battery monitor implementation using HAL analog input pins
 * 
 * This file implements analog voltage and current sensing for battery monitoring.
 * Measurements are taken from hardware analog pins defined in board configuration
 * (hwdef.dat on ChibiOS platforms) and converted to battery voltage and current
 * using configurable scaling factors (voltage divider and current sensor sensitivity).
 * 
 * The analog implementation supports:
 * - Direct voltage measurement via voltage divider
 * - Current measurement via Hall effect or shunt-based current sensors
 * - Configurable calibration offsets for sensor zeroing
 * - Board-specific default pin assignments and scaling factors
 * 
 * @note This is the most common battery monitoring method, used by power modules
 *       like the 3DR Power Module, AttoPilot sensors, and board-integrated sensing.
 */
#pragma once

#include "AP_BattMonitor_Backend.h"

#if AP_BATTERY_ANALOG_ENABLED

#include "AP_BattMonitor.h"

// default pins and dividers
#if defined(HAL_BATT_VOLT_PIN)
 // pins defined in board config (hwdef.dat on ChibiOS)
 # define AP_BATT_VOLT_PIN                  HAL_BATT_VOLT_PIN
 # define AP_BATT_CURR_PIN                  HAL_BATT_CURR_PIN
 # define AP_BATT_VOLTDIVIDER_DEFAULT       HAL_BATT_VOLT_SCALE
 # define AP_BATT_CURR_AMP_PERVOLT_DEFAULT  HAL_BATT_CURR_SCALE
#elif CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
 # define AP_BATT_VOLT_PIN                  4
 # define AP_BATT_CURR_PIN                  3
 # define AP_BATT_VOLTDIVIDER_DEFAULT       10.1f
 # define AP_BATT_CURR_AMP_PERVOLT_DEFAULT  17.0f
#else
 # define AP_BATT_VOLT_PIN                  -1
 # define AP_BATT_CURR_PIN                  -1
 # define AP_BATT_VOLTDIVIDER_DEFAULT       10.1f
 # define AP_BATT_CURR_AMP_PERVOLT_DEFAULT  17.0f
#endif

// This is 0 for the majority of the power modules.
#ifndef AP_BATT_CURR_AMP_OFFSET_DEFAULT
 #define AP_BATT_CURR_AMP_OFFSET_DEFAULT 0.0f
#endif

// Other values normally set directly by mission planner
// # define AP_BATT_VOLTDIVIDER_DEFAULT 15.70   // Volt divider for AttoPilot 50V/90A sensor
// # define AP_BATT_VOLTDIVIDER_DEFAULT 4.127   // Volt divider for AttoPilot 13.6V/45A sensor
// # define AP_BATT_CURR_AMP_PERVOLT_DEFAULT 27.32  // Amp/Volt for AttoPilot 50V/90A sensor
// # define AP_BATT_CURR_AMP_PERVOLT_DEFAULT 13.66  // Amp/Volt for AttoPilot 13.6V/45A sensor

/**
 * @class AP_BattMonitor_Analog
 * @brief Battery monitor backend for analog voltage and current sensing via HAL analog pins
 * 
 * @details This class implements battery monitoring using analog voltage and current measurements
 *          from hardware analog input pins. It reads raw analog values from the Hardware Abstraction
 *          Layer (HAL) and converts them to battery voltage and current using calibrated scaling factors.
 * 
 *          Hardware Interface:
 *          - Voltage sensing: Analog pin measuring battery voltage through a resistor divider network
 *          - Current sensing: Analog pin measuring voltage from current sensor (Hall effect or shunt-based)
 *          
 *          Measurement Pipeline:
 *          1. Read raw analog voltage from HAL AnalogSource (typically 0-3.3V or 0-5V range)
 *          2. Apply offset correction (subtract _volt_offset or _curr_amp_offset)
 *          3. Apply scaling factor (_volt_multiplier or _curr_amp_per_volt)
 *          4. Result: Battery voltage in volts, current in amperes
 * 
 *          Configuration:
 *          - Pin assignments: Set via parameters or hwdef.dat board configuration
 *          - Voltage multiplier: Accounts for voltage divider ratio (e.g., 10.1 for 10:1 divider)
 *          - Current scaling: Sensor-specific (e.g., 17.0 A/V for 3DR Power Module, 27.32 A/V for AttoPilot 50V/90A)
 *          - Offsets: Zero-point calibration for sensors (typically 0V = 0A for current sensors)
 * 
 *          Typical Power Modules:
 *          - 3DR Power Module: 10.1 V/V voltage divider, 17.0 A/V current sensor
 *          - AttoPilot 50V/90A: 15.70 V/V voltage divider, 27.32 A/V current sensor
 *          - AttoPilot 13.6V/45A: 4.127 V/V voltage divider, 13.66 A/V current sensor
 * 
 * @note Board-specific defaults are defined in hwdef.dat via HAL_BATT_VOLT_PIN, HAL_BATT_CURR_PIN,
 *       HAL_BATT_VOLT_SCALE, and HAL_BATT_CURR_SCALE definitions.
 * 
 * @warning Incorrect voltage multiplier or current scaling can lead to inaccurate battery monitoring,
 *          potentially causing unexpected failsafes or battery damage from over-discharge.
 * 
 * @see AP_BattMonitor_Backend for base class interface
 * @see AP_HAL::AnalogSource for analog input abstraction
 */
class AP_BattMonitor_Analog : public AP_BattMonitor_Backend
{
public:

    /**
     * @brief Constructor for analog battery monitor backend
     * 
     * @param[in] mon         Reference to parent AP_BattMonitor instance managing all battery monitors
     * @param[in] mon_state   Reference to state structure for this battery monitor instance
     * @param[in] params      Reference to parameter structure containing user-configurable settings
     */
    AP_BattMonitor_Analog(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state, AP_BattMonitor_Params &params);

    /**
     * @brief Read the battery voltage and current from analog pins
     * 
     * @details This method samples the analog voltage and current pins, applies calibration
     *          scaling factors and offsets, and updates the battery monitor state with the
     *          measured voltage (in volts) and current (in amperes).
     *          
     *          Measurement process:
     *          1. Read raw analog voltage from _volt_pin_analog_source (0-3.3V or 0-5V)
     *          2. Apply voltage offset: (raw_voltage - _volt_offset)
     *          3. Apply voltage multiplier to account for divider: voltage = corrected * _volt_multiplier
     *          4. Read raw analog voltage from _curr_pin_analog_source
     *          5. Apply current offset: (raw_voltage - _curr_amp_offset)
     *          6. Apply current scaling: current = corrected * _curr_amp_per_volt
     *          7. Update mon_state with measured values
     *          
     *          The voltage multiplier compensates for the voltage divider network that scales
     *          battery voltage down to the ADC input range. For example, a 10:1 divider with
     *          0.1V offset would have _volt_multiplier=10.1.
     *          
     *          The current scaling factor converts the current sensor output voltage to amperes.
     *          Hall effect and shunt-based sensors typically output a voltage proportional to
     *          current (e.g., 3DR Power Module outputs ~0.05V per ampere, giving 17.0 A/V scaling).
     * 
     * @note This method should be called at 10Hz to provide stable battery monitoring.
     *       Higher rates may add unnecessary CPU load; lower rates may miss transient events.
     * 
     * @note Method has no return value; results are stored in inherited mon_state structure
     *       (mon_state.voltage_V and mon_state.current_A members).
     */
    virtual void read() override;

    /**
     * @brief Check if this battery monitor can provide consumed energy information
     * 
     * @details Consumed energy (mAh) is calculated by integrating current over time.
     *          This is only possible if current measurement is available.
     * 
     * @return true if current sensing is available (and thus energy consumption can be calculated)
     * @return false if current pin is not configured or current sensing is unavailable
     */
    virtual bool has_consumed_energy() const override { return has_current(); }

    /**
     * @brief Check if this battery monitor can provide current measurement
     * 
     * @details Current measurement is available if a valid current sensing pin is configured
     *          (i.e., _curr_pin is set to a valid analog input pin number, not -1).
     * 
     * @return true if current pin is configured and current measurement is available
     * @return false if current pin is disabled (_curr_pin == -1) or unavailable
     */
    virtual bool has_current() const override;

    /**
     * @brief Initialize the analog battery monitor (empty implementation)
     * 
     * @details The analog backend performs initialization in its constructor, so this
     *          method has no additional initialization to perform.
     */
    virtual void init(void) override {}

    static const struct AP_Param::GroupInfo var_info[];

protected:

    /// HAL analog input source for voltage measurement - provides raw ADC reading from voltage divider
    AP_HAL::AnalogSource *_volt_pin_analog_source;
    
    /// HAL analog input source for current measurement - provides raw ADC reading from current sensor
    AP_HAL::AnalogSource *_curr_pin_analog_source;

    // ========== Calibration Parameters (user-configurable via GCS) ==========
    
    /**
     * @brief Voltage multiplier to convert analog pin voltage to battery voltage
     * 
     * This parameter accounts for the voltage divider network that scales battery voltage
     * down to the ADC input range (typically 0-3.3V or 0-5V).
     * 
     * Calculation: battery_voltage = (analog_pin_voltage - _volt_offset) * _volt_multiplier
     * 
     * Example: For a 10:1 voltage divider, _volt_multiplier = 10.1
     * Example: For AttoPilot 50V/90A sensor, _volt_multiplier = 15.70
     * 
     * Units: Unitless ratio (V_battery / V_adc)
     * Typical range: 4.0 to 20.0 for common power modules
     * 
     * @note This value must be calibrated for accurate voltage readings. Measure actual
     *       battery voltage with a multimeter and adjust parameter until displayed voltage matches.
     */
    AP_Float _volt_multiplier;
    
    /**
     * @brief Current sensor scaling factor to convert analog pin voltage to current
     * 
     * This parameter converts the current sensor output voltage to amperes. Different
     * current sensors have different sensitivities (output voltage per ampere of current).
     * 
     * Calculation: battery_current = (analog_pin_voltage - _curr_amp_offset) * _curr_amp_per_volt
     * 
     * Example: 3DR Power Module outputs ~0.0588V/A, so _curr_amp_per_volt = 17.0 A/V
     * Example: AttoPilot 50V/90A sensor: _curr_amp_per_volt = 27.32 A/V
     * Example: AttoPilot 13.6V/45A sensor: _curr_amp_per_volt = 13.66 A/V
     * 
     * Units: Amperes per volt (A/V)
     * Typical range: 10.0 to 30.0 A/V for common sensors
     * 
     * @note Calibrate by measuring actual current with a DC clamp meter and adjusting
     *       parameter until displayed current matches under load.
     */
    AP_Float _curr_amp_per_volt;
    
    /**
     * @brief Current sensor zero-point offset voltage
     * 
     * Voltage output from current sensor when zero current is flowing. This offset is
     * subtracted before applying the scaling factor to get actual current.
     * 
     * Calculation: battery_current = (analog_pin_voltage - _curr_amp_offset) * _curr_amp_per_volt
     * 
     * Units: Volts (V)
     * Typical value: 0.0V for most Hall effect sensors (zero current = zero voltage)
     * Range: -0.5V to +0.5V for typical offset calibration
     * 
     * @note Most power modules output 0V at 0A, so this is typically 0.0. Adjust if current
     *       reading shows non-zero value when no current is flowing (battery disconnected).
     */
    AP_Float _curr_amp_offset;
    
    /**
     * @brief Voltage measurement zero-point offset
     * 
     * Voltage offset subtracted from analog pin reading before applying multiplier.
     * Compensates for ADC offset or voltage divider bias voltage.
     * 
     * Calculation: battery_voltage = (analog_pin_voltage - _volt_offset) * _volt_multiplier
     * 
     * Units: Volts (V)
     * Typical value: 0.0V (no offset correction needed)
     * Range: -0.5V to +0.5V for typical offset calibration
     * 
     * @note Rarely needs adjustment. Only modify if voltage reading is consistently offset
     *       by a fixed amount across all battery voltage levels.
     */
    AP_Float _volt_offset;
    
    /**
     * @brief Analog pin number for voltage measurement
     * 
     * Board-specific analog input pin connected to battery voltage divider output.
     * Pin numbers are defined in board hwdef.dat configuration.
     * 
     * Value: Board-specific pin number (e.g., 0-15 for typical ADC channels)
     * Special value: -1 disables voltage sensing
     * 
     * @note Default value is set from HAL_BATT_VOLT_PIN in hwdef.dat if defined,
     *       otherwise defaults to pin 4 on ChibiOS or -1 (disabled) on other platforms.
     */
    AP_Int8  _volt_pin;
    
    /**
     * @brief Analog pin number for current measurement
     * 
     * Board-specific analog input pin connected to current sensor output.
     * Pin numbers are defined in board hwdef.dat configuration.
     * 
     * Value: Board-specific pin number (e.g., 0-15 for typical ADC channels)
     * Special value: -1 disables current sensing
     * 
     * @note Default value is set from HAL_BATT_CURR_PIN in hwdef.dat if defined,
     *       otherwise defaults to pin 3 on ChibiOS or -1 (disabled) on other platforms.
     * @note If set to -1, has_current() and has_consumed_energy() will return false.
     */
    AP_Int8  _curr_pin;
};

#endif  // AP_BATTERY_ANALOG_ENABLED
