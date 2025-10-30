/**
 * @file AP_Motors_Thrust_Linearization.h
 * @brief Thrust linearization and battery voltage compensation for motor control
 * 
 * @details This file implements thrust curve linearization to compensate for the
 *          non-linear relationship between ESC throttle input and actual motor thrust output.
 *          Additionally provides battery voltage compensation to maintain consistent
 *          thrust as battery voltage sags during flight.
 *          
 *          The thrust curve linearization converts between desired thrust (0-1 normalized)
 *          and ESC actuator output (0-1 PWM range) using an exponential mapping that
 *          accounts for the typical quadratic relationship between motor RPM and thrust.
 *          
 *          Battery voltage compensation scales motor outputs to maintain thrust levels
 *          as battery voltage decreases, preventing altitude loss during discharge.
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 * 
 * Source: libraries/AP_Motors/AP_Motors_Thrust_Linearization.h
 */

#pragma once

#include <AP_Param/AP_Param.h>
#include <Filter/LowPassFilter.h>

class AP_Motors;

/**
 * @class Thrust_Linearization
 * @brief Linearizes ESC/motor/propeller thrust curve and compensates for battery voltage sag
 * 
 * @details **Purpose**: Provides thrust linearization and battery voltage compensation
 *          for multicopter and helicopter motor control systems.
 *          
 *          **Problem**: The relationship between ESC throttle input (PWM) and actual thrust
 *          output is highly non-linear due to:
 *          - Motor torque vs current characteristics
 *          - Propeller thrust scaling with RPM squared (thrust ∝ RPM²)
 *          - ESC response curves
 *          - Battery voltage affecting motor speed at a given throttle
 *          
 *          Without compensation, attitude and position controllers receive non-linear response
 *          to their outputs, degrading stability and requiring conservative tuning.
 *          
 *          **Solution**: This class provides bidirectional mapping between desired thrust
 *          (0-1 normalized, used by controllers) and actuator output (0-1, sent to ESCs):
 *          
 *          1. **Thrust Curve Linearization**: Uses exponential curve to map thrust to throttle:
 *             - Forward: actuator = ((thrust^(1/expo)) - spin_min) / (1 - spin_min)
 *             - Inverse: thrust = (actuator * (1 - spin_min) + spin_min)^expo
 *             - Expo parameter: 0 = linear, 0.5 = square root, 1 = quadratic
 *             - Typical value: 0.65 provides good approximation for most motor/prop combinations
 *          
 *          2. **Battery Voltage Compensation**: Scales outputs to maintain thrust as voltage drops:
 *             - lift_max = (battery_voltage - volt_min) / (volt_max - volt_min)
 *             - Compensates for reduced motor speed at lower voltages
 *             - Filtered with 5-second time constant to prevent oscillations
 *             - Optional air density correction for altitude/temperature effects
 *          
 *          3. **Usable Throttle Range**: spin_min and spin_max define safe operating range:
 *             - spin_min: Minimum throttle where motors spin stably (default 0.15)
 *             - spin_max: Maximum safe throttle before saturation (default 0.95)
 *             - Controllers work in 0-1 normalized space; this class maps to spin_min-spin_max
 *          
 *          **Integration**: Used by AP_MotorsMulticopter and AP_MotorsHeli_Single classes.
 *          Controllers output desired thrust; this class converts to ESC commands.
 *          update_lift_max_from_batt_voltage() must be called periodically (~10Hz) to
 *          update voltage compensation.
 * 
 * @note Thrust curve expo affects controller response. Incorrect values cause instability or sluggishness.
 * @note Battery compensation requires accurate battery voltage sensor (AP_BattMonitor).
 * @note Changes to expo or spin parameters require re-tuning attitude controllers.
 * 
 * @warning Incorrect expo tuning can cause oscillations or poor throttle response
 * @warning Voltage compensation requires volt_min > ESC low-voltage cutoff to prevent shutdowns
 * @warning spin_min too high wastes throttle range; too low causes motor instability
 * 
 * @see AP_MotorsMulticopter
 * @see AP_MotorsHeli_Single
 */
class Thrust_Linearization {
friend class AP_MotorsMulticopter;
friend class AP_MotorsMulticopter_test;
friend class AP_MotorsHeli_Single;

public:
    /**
     * @brief Constructor for Thrust_Linearization
     * 
     * @param[in] _motors Reference to parent AP_Motors object for parameter access
     */
    Thrust_Linearization(AP_Motors& _motors);

    /**
     * @brief Apply thrust curve linearization and battery voltage scaling
     * 
     * @details Converts normalized desired thrust to throttle output with both
     *          exponential curve correction and battery voltage compensation applied.
     *          This is the primary forward conversion used to generate ESC commands.
     *          
     *          Algorithm:
     *          1. Apply exponential thrust curve: actuator = thrust^(1/expo)
     *          2. Map to spin_min to spin_max range
     *          3. Scale by voltage compensation factor (lift_max)
     *          
     *          Equation: throttle = (thrust^(1/expo) - spin_min) / (1 - spin_min) * lift_max
     *          
     * @param[in] thrust Desired thrust in range 0.0 to 1.0 (normalized, 0=no thrust, 1=max thrust)
     * 
     * @return Throttle output in range 0.0 to 1.0 with voltage compensation applied
     * 
     * @note Output is voltage-compensated; will be higher than input when battery is low
     * @note Returns values clamped to 0.0-1.0 range
     * 
     * @see remove_thrust_curve_and_volt_scaling() for inverse operation
     * @see thrust_to_actuator() for curve-only conversion without voltage scaling
     */
    float apply_thrust_curve_and_volt_scaling(float thrust) const;

    /**
     * @brief Remove thrust curve linearization and battery voltage scaling (inverse operation)
     * 
     * @details Converts throttle output back to normalized thrust value.
     *          This is the inverse of apply_thrust_curve_and_volt_scaling().
     *          Used by controllers that need to work backwards from actuator commands.
     *          
     *          Algorithm:
     *          1. Remove voltage compensation: actuator = throttle / lift_max
     *          2. Unmap from spin_min-spin_max range
     *          3. Apply inverse exponential curve: thrust = actuator^expo
     *          
     *          Equation: thrust = ((throttle / lift_max) * (1 - spin_min) + spin_min)^expo
     * 
     * @param[in] throttle Throttle output in range 0.0 to 1.0 (as sent to ESCs)
     * 
     * @return Desired thrust in range 0.0 to 1.0 (normalized thrust)
     * 
     * @note Input should be voltage-compensated throttle value
     * @note Returns values clamped to 0.0-1.0 range
     * 
     * @see apply_thrust_curve_and_volt_scaling() for forward operation
     * @see actuator_to_thrust() for curve-only inverse without voltage scaling
     */
    float remove_thrust_curve_and_volt_scaling(float throttle) const;

    /**
     * @brief Convert desired thrust to linearized actuator output (curve only, no voltage scaling)
     * 
     * @details Applies only the exponential thrust curve mapping without battery voltage
     *          compensation. Maps normalized thrust (0-1) to actuator output in the
     *          spin_min to spin_max range.
     *          
     *          This is used when voltage compensation is not desired or is applied
     *          separately. The thrust curve compensates for the non-linear relationship
     *          between motor PWM input and thrust output.
     *          
     *          Mathematical relationship:
     *          - actuator = ((thrust^(1/expo)) - spin_min) / (1 - spin_min)
     *          - For typical expo=0.65: actuator ≈ (thrust^1.54 - 0.15) / 0.85
     *          
     *          Thrust curve behavior by expo value:
     *          - expo = 0.0: Linear mapping (actuator = thrust)
     *          - expo = 0.5: Square root curve (more responsive at low thrust)
     *          - expo = 0.65: Typical value, balances response across range
     *          - expo = 1.0: Quadratic curve (matches thrust ∝ RPM² relationship)
     * 
     * @param[in] thrust_in Normalized desired thrust in range 0.0 to 1.0
     *                      (0.0 = minimum stable thrust, 1.0 = maximum thrust)
     * 
     * @return Actuator output in range 0.0 to 1.0 (maps to spin_min-spin_max PWM range)
     * 
     * @note Does NOT include voltage compensation - use apply_thrust_curve_and_volt_scaling() for full compensation
     * @note Output maps to spin_min at thrust=0, spin_max at thrust=1
     * @note Returns values clamped to 0.0-1.0 range
     * 
     * @see actuator_to_thrust() for inverse operation
     * @see apply_thrust_curve_and_volt_scaling() for version with voltage compensation
     */
    float thrust_to_actuator(float thrust_in) const;

    /**
     * @brief Convert actuator output to desired thrust (inverse, curve only, no voltage scaling)
     * 
     * @details Inverse of thrust_to_actuator(). Converts actuator output back to
     *          normalized thrust using inverse exponential curve. Used by controllers
     *          that need to interpret actuator commands as thrust values.
     *          
     *          Mathematical relationship:
     *          - thrust = (actuator * (1 - spin_min) + spin_min)^expo
     *          - For typical expo=0.65: thrust ≈ (actuator * 0.85 + 0.15)^0.65
     *          
     *          This inverse operation allows controllers to work in normalized thrust
     *          space (0-1) even when examining ESC actuator outputs.
     * 
     * @param[in] actuator Actuator output in range 0.0 to 1.0 (from ESC output)
     * 
     * @return Normalized thrust in range 0.0 to 1.0
     * 
     * @note Does NOT include voltage compensation removal
     * @note Input actuator=0 maps to thrust at spin_min, actuator=1 maps to thrust at spin_max
     * @note Returns values clamped to 0.0-1.0 range
     * 
     * @see thrust_to_actuator() for forward operation
     * @see remove_thrust_curve_and_volt_scaling() for version with voltage decompensation
     */
    float actuator_to_thrust(float actuator) const;

    /**
     * @brief Update battery voltage compensation factor
     * 
     * @details Reads current battery voltage and calculates lift_max compensation factor
     *          to maintain thrust output as battery voltage sags. Should be called
     *          periodically at approximately 10Hz (every 100ms) from the main scheduler.
     *          
     *          Battery voltage is filtered with a 5-second time constant low-pass filter
     *          to prevent rapid throttle changes that could cause oscillations or instability.
     *          
     *          Calculation:
     *          1. Read battery voltage from AP_BattMonitor (battery instance batt_idx)
     *          2. Apply low-pass filter with 5-second time constant
     *          3. Calculate: lift_max = (V_filtered - V_min) / (V_max - V_min)
     *          4. Clamp lift_max to reasonable range (typically 0.5 to 1.2)
     *          
     *          The lift_max factor directly scales throttle outputs. As voltage drops,
     *          lift_max decreases below 1.0, causing thrust_to_actuator() to output
     *          higher throttle values to maintain the same thrust.
     *          
     *          Optional air density compensation (altitude/temperature) can also be
     *          applied if configured, accounting for reduced air density at altitude.
     * 
     * @note Call at approximately 10Hz from main scheduler loop
     * @note Requires valid battery voltage sensor (AP_BattMonitor configured)
     * @note 5-second filter time constant prevents rapid throttle changes
     * @note Voltage below batt_voltage_min results in maximum compensation
     * 
     * @warning If battery voltage sensor fails, compensation may become invalid
     * @warning Ensure batt_voltage_min > ESC low-voltage cutoff to prevent motor shutdowns
     * 
     * @see get_compensation_gain() to retrieve current compensation value
     * @see get_lift_max() to get filtered lift_max factor
     */
    void update_lift_max_from_batt_voltage();

    /**
     * @brief Get combined voltage and air density compensation gain
     * 
     * @details Returns the gain multiplier used for thrust compensation based on
     *          battery voltage and optionally air density (altitude/temperature effects).
     *          
     *          This gain represents how much additional throttle is needed to maintain
     *          thrust compared to nominal conditions (fresh battery at sea level):
     *          - gain = 1.0: Nominal conditions, no compensation needed
     *          - gain < 1.0: Better than nominal (fresh battery, cold/dense air)
     *          - gain > 1.0: Compensation needed (low battery, hot/thin air)
     *          
     *          Gain is based on filtered battery voltage (lift_max) and air density
     *          correction if enabled. Controllers can use this for gain scheduling
     *          to adjust PID response as conditions change.
     * 
     * @return Compensation gain multiplier (dimensionless, typically 0.5 to 1.2)
     * 
     * @note Includes both battery voltage and air density compensation
     * @note Updated by update_lift_max_from_batt_voltage()
     * @note Can be used for gain scheduling in attitude/position controllers
     * 
     * @see update_lift_max_from_batt_voltage()
     * @see get_lift_max()
     */
    float get_compensation_gain() const;

    /**
     * @brief Get minimum stable throttle parameter value
     * 
     * @details Returns the spin_min parameter which defines the minimum throttle
     *          output where motors spin stably. Below this value, motors may not
     *          spin reliably or may exhibit erratic behavior.
     *          
     *          Typical values: 0.10 to 0.20 (10-20% throttle)
     *          Default: 0.15 (15% throttle)
     *          
     *          This value is the lower bound of the usable throttle range. Controllers
     *          output 0.0-1.0 normalized thrust, which gets mapped to spin_min-spin_max
     *          throttle range.
     * 
     * @return Minimum stable throttle in range 0.0 to 1.0 (typically 0.10-0.20)
     * 
     * @note Value from MOT_SPIN_MIN parameter
     * @note Too low: motors may not start reliably or may stutter
     * @note Too high: wastes available throttle range, reduces maximum thrust
     * 
     * @see get_spin_max()
     */
    float get_spin_min() const { return spin_min.get(); }

    /**
     * @brief Get maximum throttle parameter value
     * 
     * @details Returns the spin_max parameter which defines the maximum throttle
     *          output sent to ESCs. This is typically set below 1.0 to prevent
     *          ESC saturation and maintain control authority.
     *          
     *          Typical values: 0.90 to 0.98 (90-98% throttle)
     *          Default: 0.95 (95% throttle)
     *          
     *          This value is the upper bound of the usable throttle range. Limiting
     *          to less than 100% ensures ESCs don't saturate and controllers maintain
     *          authority for attitude corrections.
     * 
     * @return Maximum throttle in range 0.0 to 1.0 (typically 0.90-0.98)
     * 
     * @note Value from MOT_SPIN_MAX parameter
     * @note Setting to 1.0 may cause ESC saturation and loss of control authority
     * @note Reducing value provides more headroom but limits maximum thrust
     * 
     * @see get_spin_min()
     */
    float get_spin_max() const { return spin_max.get(); }

    /**
     * @brief Get battery instance index used for voltage compensation
     * 
     * @details Returns which battery instance (from AP_BattMonitor) is used for
     *          voltage compensation calculations. Useful for multi-battery systems.
     * 
     * @return Battery instance index (0-based, typically 0 for first battery)
     * 
     * @note Value from MOT_BAT_IDX parameter
     * @note Must be a valid battery instance with voltage sensing
     * 
     * @see update_lift_max_from_batt_voltage()
     */
    int8_t get_battery_index() const { return batt_idx.get(); }

    /**
     * @brief Get minimum battery voltage parameter for compensation scaling
     * 
     * @details Returns the minimum battery voltage used for compensation calculations.
     *          When battery voltage reaches this value, maximum compensation is applied.
     *          
     *          Should be set above ESC low-voltage cutoff to prevent motor shutdowns
     *          when compensation saturates.
     * 
     * @return Minimum battery voltage in volts (e.g., 14.0V for 4S LiPo)
     * 
     * @note Value from MOT_BAT_VOLT_MIN parameter
     * @note Must be > ESC low-voltage cutoff to prevent shutdowns
     * @note Typical: ~3.5V per cell (14.0V for 4S, 21.0V for 6S)
     * 
     * @warning Setting too low risks ESC low-voltage cutoff during compensation
     * 
     * @see get_battery_max_voltage()
     * @see update_lift_max_from_batt_voltage()
     */
    float get_battery_min_voltage() const { return batt_voltage_min.get(); }

    /**
     * @brief Get current lift_max compensation factor
     * 
     * @details Returns the current lift_max factor calculated by
     *          update_lift_max_from_batt_voltage(). This is the filtered
     *          battery voltage expressed as a scaling factor.
     *          
     *          Value interpretation:
     *          - lift_max = 1.0: Battery at maximum voltage (nominal conditions)
     *          - lift_max < 1.0: Battery voltage below maximum (compensation active)
     *          - lift_max = 0.5: Battery at minimum voltage (maximum compensation)
     * 
     * @return Current lift_max factor (typically 0.5 to 1.0)
     * 
     * @note Updated by update_lift_max_from_batt_voltage()
     * @note Filtered with 5-second time constant
     * @note Used internally for voltage compensation scaling
     * 
     * @see update_lift_max_from_batt_voltage()
     * @see get_compensation_gain()
     */
    float get_lift_max() const { return lift_max; }

    /**
     * @brief Parameter table for ArduPilot parameter system
     * 
     * @details Defines parameters exposed to ground control stations and parameter
     *          management system. Includes thrust curve expo, spin limits, and
     *          battery compensation settings.
     */
    static const struct AP_Param::GroupInfo var_info[];

protected:
    /**
     * @brief Thrust curve exponent parameter (MOT_THST_EXPO)
     * 
     * @details Exponential curve used to linearize PWM to thrust conversion.
     *          Compensates for the non-linear relationship between motor throttle
     *          input and actual thrust output.
     *          
     *          Parameter range: 0.0 to 1.0
     *          - 0.0 = Linear mapping (no compensation)
     *          - 0.5 = Square root curve (thrust^0.5)
     *          - 0.65 = Default/typical value (balanced response)
     *          - 1.0 = Quadratic curve (matches thrust ∝ RPM² relationship)
     *          
     *          Tuning guidance:
     *          - Hover throttle < 40%: Increase expo (makes low throttle more responsive)
     *          - Hover throttle > 60%: Decrease expo (makes high throttle more linear)
     *          - Test in stabilize mode at mid-stick position
     *          
     *          Units: Dimensionless (0.0 to 1.0)
     *          Default: 0.65
     * 
     * @warning Incorrect expo causes poor throttle response or instability
     * @warning Changes require re-tuning attitude controllers
     * 
     * @note Typical range: 0.55 to 0.75 depending on motor/prop combination
     * @note Affects feel of throttle stick and altitude hold performance
     */
    AP_Float curve_expo;

    /**
     * @brief Minimum stable throttle parameter (MOT_SPIN_MIN)
     * 
     * @details Throttle output ratio which produces minimum stable thrust.
     *          Below this value, motors may not spin reliably or consistently.
     *          Defines the lower bound of the usable throttle range.
     *          
     *          Parameter range: 0.0 to 1.0 (typically 0.10 to 0.20)
     *          Units: Ratio of full throttle range (0-1)
     *          Default: 0.15 (15% throttle)
     *          
     *          Tuning guidance:
     *          - Set to lowest value where all motors spin reliably and evenly
     *          - Too low: Motors may stutter, fail to start, or spin unevenly
     *          - Too high: Wastes throttle range, reduces maximum thrust available
     *          - Test by commanding zero thrust and observing motor behavior
     * 
     * @warning Too low causes motor instability and unreliable startup
     * @warning Too high wastes throttle range and limits maximum thrust
     * 
     * @note Vehicle-specific, depends on ESC/motor/prop combination
     * @note Should be verified on bench before flight testing
     */
    AP_Float spin_min;

    /**
     * @brief Maximum throttle parameter (MOT_SPIN_MAX)
     * 
     * @details Throttle output ratio which produces maximum thrust.
     *          Defines the upper bound of the usable throttle range.
     *          Typically set below 1.0 to prevent ESC saturation and maintain
     *          control authority for attitude corrections.
     *          
     *          Parameter range: 0.0 to 1.0 (typically 0.90 to 0.98)
     *          Units: Ratio of full throttle range (0-1)
     *          Default: 0.95 (95% throttle)
     *          
     *          Tuning guidance:
     *          - Start with 0.95 and reduce if ESCs saturate during aggressive maneuvers
     *          - Setting to 1.0 may cause loss of control authority at full throttle
     *          - Lower values provide more headroom but limit maximum climb rate
     * 
     * @warning Setting to 1.0 may cause ESC saturation and loss of control
     * @warning Too low limits maximum thrust and climb performance
     * 
     * @note Leave 5-10% headroom below 1.0 for control authority
     * @note Critical for maintaining stability during full-throttle climbs
     */
    AP_Float spin_max;

    /**
     * @brief Battery instance index parameter (MOT_BAT_IDX)
     * 
     * @details Specifies which battery instance from AP_BattMonitor to use for
     *          voltage compensation calculations. Useful for multi-battery systems.
     *          
     *          Parameter range: 0 to (number of batteries - 1)
     *          Units: Integer index (0-based)
     *          Default: 0 (first battery)
     * 
     * @note Must correspond to a valid battery instance with voltage sensing
     * @note For single-battery systems, always use 0
     */
    AP_Int8 batt_idx;

    /**
     * @brief Maximum battery voltage parameter (MOT_BAT_VOLT_MAX)
     * 
     * @details Maximum (nominal full charge) battery voltage used as reference
     *          for voltage compensation scaling. Represents fully charged battery voltage.
     *          
     *          When battery is at this voltage, no compensation is applied (lift_max = 1.0).
     *          As voltage drops below this, compensation increases to maintain thrust.
     *          
     *          Parameter range: Depends on battery chemistry and cell count
     *          Units: Volts (V)
     *          
     *          Typical values:
     *          - 4S LiPo (4 cells): 16.8V (4.2V per cell)
     *          - 6S LiPo (6 cells): 25.2V (4.2V per cell)
     *          - 4S Li-ion: 16.8V
     *          
     *          Default: Depends on detected battery configuration
     * 
     * @note Should match fully charged battery voltage for your battery type
     * @note Affects voltage compensation scaling range
     */
    AP_Float batt_voltage_max;

    /**
     * @brief Minimum battery voltage parameter (MOT_BAT_VOLT_MIN)
     * 
     * @details Minimum battery voltage used for voltage compensation scaling.
     *          When battery reaches this voltage, maximum compensation is applied.
     *          
     *          Should be set above ESC low-voltage cutoff to prevent motor shutdowns
     *          when maximum compensation is reached.
     *          
     *          Parameter range: Depends on battery chemistry and cell count
     *          Units: Volts (V)
     *          
     *          Typical values:
     *          - 4S LiPo (4 cells): 14.0V (3.5V per cell)
     *          - 6S LiPo (6 cells): 21.0V (3.5V per cell)
     *          
     *          Default: Depends on detected battery configuration
     * 
     * @warning Must be above ESC low-voltage cutoff to prevent motor shutdowns
     * @warning Setting too low risks sudden motor cutoff during compensation
     * 
     * @note Recommended: ~3.5V per cell for LiPo batteries
     * @note Should trigger battery failsafe before reaching this voltage
     */
    AP_Float batt_voltage_min;

private:
    /**
     * @brief Current maximum lift ratio from battery voltage compensation
     * 
     * @details Calculated lift capacity factor based on filtered battery voltage.
     *          Used to scale throttle outputs to maintain thrust as voltage drops.
     *          
     *          Calculation: lift_max = (V_batt - V_min) / (V_max - V_min)
     *          
     *          Value range: Typically 0.5 to 1.0
     *          - 1.0 = Battery at maximum voltage (no compensation)
     *          - 0.5 = Battery at minimum voltage (maximum compensation)
     *          
     *          Updated by update_lift_max_from_batt_voltage() at ~10Hz.
     *          Filtered with 5-second time constant to prevent rapid changes.
     */
    float lift_max;

    /**
     * @brief Throttle limit ratio between hover and maximum
     * 
     * @details Ratio used for throttle limiting calculations.
     *          Represents the relationship between hover throttle and maximum
     *          available throttle, used by motor mixing algorithms.
     */
    float throttle_limit;

    /**
     * @brief Low-pass filtered battery voltage
     * 
     * @details Battery voltage expressed as a percentage (0.0 to 1.0) of the
     *          difference between batt_voltage_max and batt_voltage_min.
     *          
     *          Filtered with 5-second time constant low-pass filter to prevent
     *          rapid throttle changes that could cause oscillations or instability.
     *          
     *          Filter prevents:
     *          - Oscillations from rapid voltage fluctuations under load
     *          - Sudden throttle jumps during current spikes
     *          - Instability from noisy voltage measurements
     *          
     *          5-second time constant chosen as compromise between responsiveness
     *          to actual voltage sag and stability.
     * 
     * @note 5-second time constant prevents rapid throttle changes
     * @note Helps stabilize voltage compensation during load transients
     */
    LowPassFilterFloat batt_voltage_filt;

    /**
     * @brief Reference to parent AP_Motors object
     * 
     * @details Provides access to motor parameters and system state.
     *          Used for retrieving battery voltage and other system information.
     */
    AP_Motors& motors;
};
