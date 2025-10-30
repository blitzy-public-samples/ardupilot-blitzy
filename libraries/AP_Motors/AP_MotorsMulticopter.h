/**
 * @file AP_MotorsMulticopter.h
 * @brief Base class for multicopter motor control with mixing, thrust linearization, and battery compensation
 * 
 * @details This file defines the AP_MotorsMulticopter base class which provides common functionality
 *          for all multicopter motor configurations including Matrix (custom mixing), Single, Coax,
 *          Tri, Quad, Hexa, Octa, and other multirotor frames. The class manages:
 *          - Spool state machine for safe motor startup and shutdown
 *          - Thrust linearization to compensate for non-linear ESC/motor/propeller response
 *          - Battery voltage and current compensation for consistent flight characteristics
 *          - PWM output range management and throttle filtering
 *          - Motor spin limits (min/max/arm) for stable control and safety margins
 *          - Yaw headroom reservation to ensure yaw authority
 * 
 * Source: libraries/AP_Motors/AP_MotorsMulticopter.h:1-211
 */
#pragma once

#include "AP_Motors_Class.h"
#include "AP_Motors_Thrust_Linearization.h"

#define AP_MOTORS_YAW_HEADROOM_DEFAULT  200
#define AP_MOTORS_THST_EXPO_DEFAULT     0.65f   // set to 0 for linear and 1 for second order approximation
#define AP_MOTORS_THST_HOVER_DEFAULT    0.35f   // the estimated hover throttle, 0 ~ 1
#define AP_MOTORS_THST_HOVER_TC         10.0f   // time constant used to update estimated hover throttle, 0 ~ 1
#define AP_MOTORS_THST_HOVER_MIN        0.125f  // minimum possible hover throttle
#define AP_MOTORS_THST_HOVER_MAX        0.6875f // maximum possible hover throttle
#define AP_MOTORS_SPIN_MIN_DEFAULT      0.15f   // throttle out ratio which produces the minimum thrust.  (i.e. 0 ~ 1 ) of the full throttle range
#define AP_MOTORS_SPIN_MAX_DEFAULT      0.95f   // throttle out ratio which produces the maximum thrust.  (i.e. 0 ~ 1 ) of the full throttle range
#define AP_MOTORS_SPIN_ARM_DEFAULT      0.10f   // throttle out ratio which produces the armed spin rate.  (i.e. 0 ~ 1 ) of the full throttle range
#define AP_MOTORS_BAT_VOLT_MAX_DEFAULT  0.0f    // voltage limiting max default
#define AP_MOTORS_BAT_VOLT_MIN_DEFAULT  0.0f    // voltage limiting min default (voltage dropping below this level will have no effect)
#define AP_MOTORS_BAT_CURR_MAX_DEFAULT  0.0f    // current limiting max default
#define AP_MOTORS_BAT_CURR_TC_DEFAULT   5.0f    // Time constant used to limit the maximum current
#define AP_MOTORS_SLEW_TIME_DEFAULT     0.0f    // slew rate limit for thrust output
#define AP_MOTORS_SAFE_TIME_DEFAULT     1.0f    // Time for the esc when transitioning between zero pwm to minimum

// spool definition
#define AP_MOTORS_SPOOL_UP_TIME_DEFAULT 0.5f    // time (in seconds) for throttle to increase from zero to min throttle, and min throttle to full throttle.

/**
 * @class AP_MotorsMulticopter
 * @brief Base class for all multicopter motor mixing implementations
 * 
 * @details AP_MotorsMulticopter provides the foundation for all multirotor motor control in ArduPilot.
 *          Derived classes (AP_MotorsMatrix, AP_MotorsSingle, AP_MotorsCoax, AP_MotorsTri, etc.)
 *          implement frame-specific motor mixing while this base class handles common multicopter features.
 * 
 * ## Spool State Machine
 * 
 * The spool state machine ensures safe motor transitions during arming, takeoff, and landing:
 * 
 * - **SHUT_DOWN**: Motors stopped, outputs at minimum PWM. Vehicle is disarmed or emergency stopped.
 * - **GROUND_IDLE**: Motors spinning at MOT_SPIN_ARM speed (default 10% throttle range). 
 *                    Vehicle is armed on ground, preparing for takeoff. Provides stable control for pre-flight checks.
 * - **SPOOLING_UP**: Ramping from ground idle to flight speed over MOT_SPOOL_TIME (default 0.5s).
 *                    Gradual acceleration protects ESCs and mechanical components from sudden loads.
 * - **THROTTLE_UNLIMITED**: Normal flight state with full throttle authority from MOT_SPIN_MIN to MOT_SPIN_MAX.
 *                           All control loops operate normally with complete thrust range.
 * - **SPOOLING_DOWN**: Ramping down from flight speed to ground idle during landing.
 *                      Gradual deceleration provides smooth touchdown and protects mechanics.
 * 
 * Transitions between states are managed automatically based on pilot input, altitude, and flight mode.
 * 
 * ## Thrust Linearization
 * 
 * ESCs, motors, and propellers have a non-linear thrust response to PWM input. The thrust linearization
 * system (via Thrust_Linearization member thr_lin) compensates for this to provide linear control:
 * 
 * - thrust_to_actuator(): Converts desired thrust (0-1) to ESC actuator output using curve_expo
 * - actuator_to_thrust(): Inverse function for control loops to compute required actuator values
 * - MOT_THST_EXPO parameter: Controls linearization curve (0=linear, 0.65=default quadratic approximation, 1=full quadratic)
 * 
 * Proper thrust linearization is critical for:
 * - Predictable attitude control across throttle range
 * - Consistent auto-tune results
 * - Stable hover in varying flight conditions
 * 
 * ## Battery Compensation
 * 
 * Battery voltage sag and current limiting affect available thrust. Compensation systems include:
 * 
 * - **Voltage Compensation**: Adjusts _throttle_thrust_max based on battery voltage relative to MOT_BAT_VOLT_MAX.
 *                             As voltage drops, maximum available thrust is reduced proportionally to maintain
 *                             consistent control authority. Prevents over-demanding ESCs at low voltage.
 * 
 * - **Current Limiting**: If battery current exceeds MOT_BAT_CURR_MAX, thrust is limited to stay below the limit.
 *                         Prevents battery damage and voltage collapse. Uses MOT_BAT_CURR_TC time constant for filtering.
 * 
 * - **Lift Max Scaling**: The lift_max value (inherited from AP_Motors) represents maximum available thrust after
 *                         applying voltage and current compensation. Control loops use this to scale demands.
 * 
 * ## Motor Spin Limits
 * 
 * Motor spin parameters define the usable throttle range for stable control:
 * 
 * - **MOT_SPIN_MIN** (default 0.15): Minimum motor speed for reliable attitude control. Below this speed,
 *                                     motors may not respond predictably to small thrust changes.
 * - **MOT_SPIN_MAX** (default 0.95): Maximum motor speed, leaving headroom for control mixing and yaw authority.
 * - **MOT_SPIN_ARM** (default 0.10): Armed ground idle speed. High enough to verify motor rotation, low enough for safety.
 * 
 * Values are ratios (0-1) of the full throttle range from MOT_PWM_MIN to MOT_PWM_MAX.
 * 
 * ## Yaw Headroom
 * 
 * MOT_YAW_HEADROOM (default 200µs) reserves PWM range exclusively for yaw control. When motors are near
 * maximum thrust, this headroom ensures yaw commands can still be executed by reducing collective thrust
 * and increasing differential thrust. Critical for maintaining yaw authority during aggressive climbs.
 * 
 * ## Throttle Hover Learning
 * 
 * The system learns hover throttle (MOT_THST_HOVER) during flight when in altitude-holding modes:
 * - Learns from actual thrust required to maintain altitude
 * - Filtered with MOT_THST_HOVER_TC time constant (10 seconds default)
 * - Used by position controller for feedforward thrust estimation
 * - Can be saved to parameters on disarm with MOT_HOVER_LEARN=2
 * 
 * @note This is an abstract base class. Derived classes must implement output_to_motors() for frame-specific mixing.
 * @note All thrust values in the API use 0-1 range. PWM values are in microseconds.
 * @note Coordinate system: Body frame with X forward, Y right, Z down (NED convention).
 * 
 * @warning Incorrect MOT_SPIN_MIN/MAX values can cause loss of control or ESC damage.
 * @warning MOT_THST_EXPO must be tuned for your specific motor/ESC/propeller combination.
 * @warning Battery compensation parameters are critical for safe operation - do not disable without testing.
 * 
 * @see AP_MotorsMatrix for custom motor mixing
 * @see AP_MotorsQuad for standard quadcopter configurations
 * @see AP_MotorsHexa for hexacopter configurations
 * @see AP_MotorsTri for tricopter with yaw servo
 * 
 * Source: libraries/AP_Motors/AP_MotorsMulticopter.h:28-210
 */
class AP_MotorsMulticopter : public AP_Motors {
public:

    /**
     * @brief Constructor for multicopter motor control base class
     * 
     * @param[in] speed_hz PWM output frequency in Hertz (default: AP_MOTORS_SPEED_DEFAULT = 400Hz)
     * 
     * @note Higher PWM frequencies provide faster ESC response but may cause EMI or ESC overheating.
     * @note Typical values: 400Hz (standard), 490Hz (racing), 50Hz (traditional servos).
     */
    AP_MotorsMulticopter(uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT);

    /**
     * @brief Main motor output method - runs spool logic and sends commands to motors
     * 
     * @details This is the primary entry point called by vehicle code each control loop iteration.
     *          The method performs the following sequence:
     *          1. Runs output_logic() to update spool state machine
     *          2. Applies thrust compensation callback if registered (for tiltrotors)
     *          3. Calls output_to_motors() implemented by derived classes for frame-specific mixing
     *          4. Applies PWM output with slew rate limiting
     * 
     *          The spool state machine manages transitions between SHUT_DOWN, GROUND_IDLE, SPOOLING_UP,
     *          THROTTLE_UNLIMITED, and SPOOLING_DOWN states based on arming status, throttle input,
     *          and flight conditions.
     * 
     * @note Called at main loop rate (typically 400Hz for copters).
     * @note Overrides AP_Motors::output() base class method.
     * 
     * @warning Do not call this method directly from multiple threads - not thread-safe.
     * 
     * @see output_to_motors() for frame-specific motor mixing implementation
     * @see output_logic() for spool state machine details
     * 
     * Source: libraries/AP_Motors/AP_MotorsMulticopter.cpp
     */
    virtual void        output() override;

    /**
     * @brief Sends minimum safe PWM values to all motors
     * 
     * @details Outputs MOT_PWM_MIN to all enabled motors. Used during:
     *          - Initial arming sequence before transitioning to GROUND_IDLE
     *          - Emergency stop conditions
     *          - Pre-arm safety checks
     *          - ESC initialization sequences
     * 
     *          This ensures motors are in a known safe state without rotation.
     * 
     * @note Overrides AP_Motors::output_min() base class method.
     * @note Does not run spool logic - outputs minimum PWM directly.
     * 
     * @see get_pwm_output_min() for the actual minimum PWM value
     * 
     * Source: libraries/AP_Motors/AP_MotorsMulticopter.cpp
     */
    void                output_min() override;

    /**
     * @brief Set yaw headroom - reserves PWM range exclusively for yaw control
     * 
     * @param[in] pwm Yaw headroom in microseconds (typical range: 100-300µs, default: 200µs)
     * 
     * @details Yaw headroom ensures yaw commands can be executed even when motors are near maximum thrust.
     *          When collective thrust approaches the limit, the mixer reduces overall thrust and increases
     *          differential thrust to maintain yaw authority. Higher headroom provides more yaw authority
     *          at high throttle but reduces maximum climb rate.
     * 
     * @note This directly sets the MOT_YAW_HEADROOM parameter.
     * @note Typical values: 100µs (racing/acro), 200µs (default), 300µs (heavy payloads requiring strong yaw).
     * 
     * @see get_yaw_headroom() to retrieve current headroom value
     */
    void                set_yaw_headroom(int16_t pwm) { _yaw_headroom.set(pwm); }

    /**
     * @brief Update throttle PWM output range based on battery voltage compensation
     * 
     * @details Recalculates the usable PWM range and thrust limits based on:
     *          - Battery voltage relative to MOT_BAT_VOLT_MAX and MOT_BAT_VOLT_MIN
     *          - Battery current relative to MOT_BAT_CURR_MAX
     *          - Configured MOT_SPIN_MIN and MOT_SPIN_MAX limits
     * 
     *          As battery voltage drops, _throttle_thrust_max is reduced proportionally to prevent
     *          over-demanding ESCs and maintain consistent control. Updates _throttle_limit which
     *          is used by the mixer to scale motor outputs.
     * 
     * @note Called automatically by output() each loop iteration.
     * @note Battery compensation is critical for safe flight at low battery voltages.
     * 
     * @see get_throttle_thrust_max() to query current maximum thrust limit
     * 
     * Source: libraries/AP_Motors/AP_MotorsMulticopter.cpp
     */
    void                update_throttle_range();

    /**
     * @brief Update estimated hover throttle by learning from flight data
     * 
     * @param[in] dt Time delta since last update in seconds
     * 
     * @details Learns the throttle required to hover by observing actual thrust in altitude-hold modes.
     *          Uses a first-order low-pass filter with time constant MOT_THST_HOVER_TC (default 10 seconds)
     *          to gradually update MOT_THST_HOVER. Learning only occurs when:
     *          - Vehicle is in an altitude-holding mode (AltHold, Loiter, PosHold, etc.)
     *          - Vertical velocity is low (near hover condition)
     *          - MOT_HOVER_LEARN is not DISABLED (value 0)
     * 
     *          The learned hover throttle is used by the position controller for feedforward thrust estimation,
     *          improving altitude hold performance and reducing oscillations.
     * 
     * @note Constrained to range [AP_MOTORS_THST_HOVER_MIN, AP_MOTORS_THST_HOVER_MAX] = [0.125, 0.6875]
     * @note If MOT_HOVER_LEARN=2 (HOVER_LEARN_AND_SAVE), value is saved to EEPROM on disarm.
     * 
     * @see get_throttle_hover() to retrieve current learned hover throttle
     * 
     * Source: libraries/AP_Motors/AP_MotorsMulticopter.cpp
     */
    void                update_throttle_hover(float dt);
    
    /**
     * @brief Get the learned hover throttle value
     * 
     * @return Hover throttle in range 0-1 (0=no thrust, 1=maximum thrust)
     * 
     * @details Returns the current MOT_THST_HOVER parameter value, constrained to safe limits.
     *          This value represents the throttle fraction required to maintain hover in calm conditions.
     * 
     * @note Overrides AP_Motors::get_throttle_hover() base class method.
     * @note Return value is always in range [0.125, 0.6875] regardless of parameter setting.
     */
    virtual float       get_throttle_hover() const override { return constrain_float(_throttle_hover, AP_MOTORS_THST_HOVER_MIN, AP_MOTORS_THST_HOVER_MAX); }

    /**
     * @brief Pass throttle directly to all motors for ESC calibration - BYPASSES ALL SAFETY LIMITS
     * 
     * @param[in] throttle_input Throttle in range 0-1 where 0 sends MOT_PWM_MIN and 1 sends MOT_PWM_MAX
     * 
     * @details ESC calibration mode directly passes pilot throttle input to all motors simultaneously,
     *          bypassing the spool state machine, thrust linearization, battery compensation, and all
     *          safety limits. This is used during ESC setup to teach ESCs the PWM range.
     * 
     *          Typical ESC calibration procedure:
     *          1. Connect battery with throttle at maximum (throttle_input = 1.0)
     *          2. ESCs detect maximum PWM and store it
     *          3. Reduce throttle to minimum (throttle_input = 0.0)
     *          4. ESCs detect minimum PWM and complete calibration
     * 
     * @warning EXTREMELY DANGEROUS - motors will spin at high speed with full battery power
     * @warning Remove propellers before entering ESC calibration mode
     * @warning Vehicle will be completely uncontrollable in this mode
     * @warning No safety checks are performed - immediate full thrust is possible
     * @warning Exit calibration mode immediately after ESC programming completes
     * 
     * @note Only use this for ESC programming, never for flight.
     * @note All motors receive identical throttle output regardless of frame configuration.
     * 
     * Source: libraries/AP_Motors/AP_MotorsMulticopter.cpp
     */
    void                set_throttle_passthrough_for_esc_calibration(float throttle_input);

    /**
     * @brief Get maximum available thrust after battery compensation
     * 
     * @return Maximum thrust in range 0-1 (0=no thrust, 1=full configured thrust)
     * 
     * @details Returns the current maximum thrust limit after applying battery voltage and current
     *          compensation. This value decreases as battery voltage drops or current limiting engages.
     *          Control loops use this to scale thrust demands and avoid integrator windup.
     * 
     * @note Value is typically 1.0 with fresh battery, decreasing to 0.6-0.8 as battery depletes.
     * @note Updated by update_throttle_range() which is called each loop iteration.
     * 
     * @see update_throttle_range() for compensation algorithm details
     */
    float               get_throttle_thrust_max() const { return _throttle_thrust_max; }

    /**
     * @brief Check if spool-up is complete and vehicle is ready for full throttle authority
     * 
     * @return true if in THROTTLE_UNLIMITED state, false otherwise
     * 
     * @details During takeoff, motors gradually spool up from GROUND_IDLE to flight speed over
     *          MOT_SPOOL_TIME (default 0.5 seconds). This method returns true only when the
     *          spool-up is complete and motors are ready for unlimited throttle commands.
     * 
     *          Flight modes and control loops should check this before aggressive maneuvers to
     *          avoid demanding thrust that motors cannot yet provide.
     * 
     * @note Returns false in states: SHUT_DOWN, GROUND_IDLE, SPOOLING_UP, SPOOLING_DOWN
     * @note Returns true only in: THROTTLE_UNLIMITED
     * 
     * @see output_logic() for spool state machine implementation
     */
    bool spool_up_complete() const { return _spool_state == SpoolState::THROTTLE_UNLIMITED; }

    /**
     * @brief Output thrust to motors matching a specific bitmask - used for tiltrotor forward flight
     * 
     * @param[in] thrust Desired thrust in range 0-1 (0=no thrust, 1=maximum thrust)
     * @param[in] mask Bitmask of motors to control (bit 0 = motor 0, bit 1 = motor 1, etc.)
     * @param[in] rudder_dt Rudder differential thrust for yaw control (-1 to +1)
     * 
     * @details Provides direct thrust control to a subset of motors, primarily used by tiltrotor
     *          quadplanes in forward flight where some motors transition to forward thrust while
     *          others remain in hover configuration. The mask parameter selects which motors to
     *          command, and rudder_dt provides yaw authority through differential thrust.
     * 
     * @note Virtual method - can be overridden by derived classes for frame-specific behavior.
     * @note Default implementation outputs thrust to all motors matching the mask.
     * @note Bypasses normal mixer for masked motors but respects spool state and safety limits.
     * 
     * @see get_motor_mask() to query which motors are configured
     * 
     * Source: libraries/AP_Motors/AP_MotorsMulticopter.cpp
     */
    virtual void        output_motor_mask(float thrust, uint32_t mask, float rudder_dt);

    /**
     * @brief Get bitmask of output channels used for motors
     * 
     * @return Bitmask where bit N is set if motor N is configured (bit 0 = motor 0, etc.)
     * 
     * @details Returns a bitmask indicating which PWM output channels are used for motor control.
     *          This allows other subsystems (servo outputs, auxiliary functions) to avoid conflicts
     *          with motor outputs. Only enabled motors are included in the mask.
     * 
     * @note Overrides AP_Motors::get_motor_mask() base class method.
     * @note Maximum 32 motors supported (uint32_t bitmask limitation).
     * 
     * @see is_motor_enabled() to check if a specific motor is enabled
     * 
     * Source: libraries/AP_Motors/AP_MotorsMulticopter.cpp
     */
    virtual uint32_t    get_motor_mask() override;

    /**
     * @brief Get minimum PWM value that can be output to motors
     * 
     * @return Minimum PWM in microseconds (typically 1000-1100µs)
     * 
     * @details Returns MOT_PWM_MIN parameter value. If MOT_PWM_MIN is 0, the vehicle's throttle
     *          input channel minimum PWM is used instead. This value represents the lowest PWM
     *          that will ever be sent to ESCs, corresponding to zero thrust.
     * 
     * @note ESCs must be calibrated to recognize this value as minimum throttle.
     * @note Typical range: 1000µs (standard ESCs) to 1100µs (some racing ESCs).
     */
    int16_t             get_pwm_output_min() const { return _pwm_min; }
    
    /**
     * @brief Get maximum PWM value that can be output to motors
     * 
     * @return Maximum PWM in microseconds (typically 1900-2000µs)
     * 
     * @details Returns MOT_PWM_MAX parameter value. If MOT_PWM_MAX is 0, the vehicle's throttle
     *          input channel maximum PWM is used instead. This value represents the highest PWM
     *          that will ever be sent to ESCs, corresponding to maximum thrust.
     * 
     * @note ESCs must be calibrated to recognize this value as maximum throttle.
     * @note Typical range: 1900µs (some ESCs) to 2000µs (standard ESCs).
     */
    int16_t             get_pwm_output_max() const { return _pwm_max; }
    
    /**
     * @brief Validate MOT_PWM_MIN and MOT_PWM_MAX parameters are reasonable
     * 
     * @return true if parameters are valid, false if problematic values detected
     * 
     * @details Checks that:
     *          - MOT_PWM_MIN < MOT_PWM_MAX (proper range)
     *          - Values are within typical ESC PWM range (800-2200µs)
     *          - Range is sufficient for control (minimum ~100µs separation)
     * 
     *          Used during arming checks to prevent flight with misconfigured PWM limits.
     * 
     * @note Called by arming_checks() before allowing vehicle to arm.
     * @note Invalid PWM parameters will prevent arming with appropriate error message.
     * 
     * @see arming_checks() for complete pre-arm validation
     * 
     * Source: libraries/AP_Motors/AP_MotorsMulticopter.cpp
     */
    bool check_mot_pwm_params() const;

    /**
     * @brief Set callback function for thrust compensation - used by tiltrotors and tiltwings
     * 
     * @param[in] callback Function pointer to thrust compensation callback
     * 
     * @details Registers a vehicle-supplied callback that modifies motor thrust values before output.
     *          Used primarily by tiltrotor and tiltwing vehicles to compensate for:
     *          - Motor tilt angle effects on vertical thrust
     *          - Transition between hover and forward flight
     *          - Differential tilt for control
     * 
     *          The callback receives the actuator array and motor count, allowing direct modification
     *          of thrust values before PWM conversion.
     * 
     * @note Callback signature: void callback(float* actuator_array, uint8_t motor_count)
     * @note Callback is invoked by output() before output_to_motors() each loop iteration.
     * @note Only one callback can be registered - subsequent calls replace previous callback.
     * 
     * @see thrust_compensation() for internal compensation hook
     */
    FUNCTOR_TYPEDEF(thrust_compensation_fn_t, void, float *, uint8_t);
    void                set_thrust_compensation_callback(thrust_compensation_fn_t callback) {
        _thrust_compensation_callback = callback;
    }
    
    /**
     * @brief Disable motor torque for yaw control - for vehicles with external yaw mechanisms
     * 
     * @details Disables differential motor thrust for yaw control. Used when vehicle has an
     *          alternative yaw control mechanism such as:
     *          - Thrust vectoring (tilting motors for yaw)
     *          - Tail rotor (traditional helicopter)
     *          - Differential tilt (tiltrotor configurations)
     * 
     *          After calling this, yaw commands will not affect motor mixing, allowing the
     *          external mechanism to provide yaw authority without interference.
     * 
     * @note Virtual method - derived classes can override for frame-specific behavior.
     * @note Default implementation does nothing (base multicopters use motor torque for yaw).
     * @note AP_MotorsTri overrides this to disable motor-based yaw when using tail servo.
     */
    virtual void        disable_yaw_torque(void) {}

    /**
     * @brief Check if a specific motor is enabled in the configuration
     * 
     * @param[in] i Motor number (0-based index)
     * 
     * @return true if motor is enabled, false if disabled or invalid index
     * 
     * @details Returns the enabled state for a specific motor. Motors can be disabled in
     *          frame configuration for custom configurations or when outputs are used for
     *          other purposes (servos, auxiliary functions).
     * 
     * @note Overrides AP_Motors::is_motor_enabled() base class method.
     * @note Index must be < AP_MOTORS_MAX_NUM_MOTORS (typically 12).
     */
    bool                is_motor_enabled(uint8_t i) override { return motor_enabled[i]; }

    /**
     * @brief Convert MOT_PWM_MIN/MAX from radio input values if not explicitly configured
     * 
     * @param[in] radio_min Minimum PWM from throttle input channel (typically 1000µs)
     * @param[in] radio_max Maximum PWM from throttle input channel (typically 2000µs)
     * 
     * @details If MOT_PWM_MIN or MOT_PWM_MAX are set to 0 (not configured), this method
     *          initializes them from the vehicle's throttle input channel range. This provides
     *          sensible defaults matching the pilot's radio calibration.
     * 
     * @note Only updates parameters that are set to 0 (unconfigured).
     * @note Called during motor initialization before first use.
     * 
     * Source: libraries/AP_Motors/AP_MotorsMulticopter.cpp
     */
    void                convert_pwm_min_max_param(int16_t radio_min, int16_t radio_max);

    /**
     * @brief Get thrust value for a specific motor after mixing
     * 
     * @param[in]  motor_num Motor number (0-based index)
     * @param[out] thr_out   Thrust value in range 0-1 (0=no thrust, 1=maximum thrust)
     * 
     * @return true if motor_num is valid and enabled, false otherwise
     * 
     * @details Returns the commanded thrust for a specific motor after frame mixing but before
     *          thrust linearization and PWM conversion. Useful for telemetry, logging, and
     *          external control systems that need to monitor individual motor commands.
     * 
     * @note Overrides AP_Motors::get_thrust() base class method.
     * @note Returns false if motor_num >= AP_MOTORS_MAX_NUM_MOTORS or motor is disabled.
     * @note Thrust value includes battery compensation and spool state scaling.
     * 
     * Source: libraries/AP_Motors/AP_MotorsMulticopter.cpp
     */
    bool                get_thrust(uint8_t motor_num, float& thr_out) const override;

    /**
     * @brief Get raw actuator output for a specific motor before thrust linearization
     * 
     * @param[in]  motor_num Motor number (0-based index)
     * @param[out] thr_out   Raw actuator value in range 0-1
     * 
     * @return true if motor_num is valid and enabled, false otherwise
     * 
     * @details Returns the raw actuator output for a specific motor from the _actuator array.
     *          This is the value before thrust linearization (thrust_to_actuator conversion)
     *          and PWM mapping. Used primarily for debugging and advanced tuning.
     * 
     * @note Overrides AP_Motors::get_raw_motor_throttle() base class method.
     * @note Returns false if motor_num >= AP_MOTORS_MAX_NUM_MOTORS or motor is disabled.
     * 
     * Source: libraries/AP_Motors/AP_MotorsMulticopter.cpp
     */
    bool                get_raw_motor_throttle(uint8_t motor_num, float& thr_out) const override;

#if HAL_LOGGING_ENABLED
    /**
     * @brief Log voltage scaling and maximum thrust to dataflash
     * 
     * @details Writes motor control telemetry to dataflash logs at 10Hz including:
     *          - Battery voltage compensation factor
     *          - Current thrust limit (_throttle_thrust_max)
     *          - Spool state
     *          - Learned hover throttle
     * 
     *          Used for post-flight analysis of motor performance, battery compensation
     *          effectiveness, and thrust limiting behavior.
     * 
     * @note Overrides AP_Motors::Log_Write() base class method.
     * @note Only compiled if HAL_LOGGING_ENABLED is defined.
     * @note Called automatically by vehicle code at 10Hz.
     * 
     * Source: libraries/AP_Motors/AP_MotorsMulticopter.cpp
     */
    void                Log_Write() override;
#endif

    /**
     * @brief Perform pre-arm safety checks for motor configuration
     * 
     * @param[in]  buflen Size of error message buffer in bytes
     * @param[out] buffer Buffer to write error message if check fails
     * 
     * @return true if all checks pass, false if any check fails (buffer contains error message)
     * 
     * @details Validates motor configuration before allowing vehicle to arm. Checks include:
     *          - MOT_PWM_MIN and MOT_PWM_MAX are valid (via check_mot_pwm_params)
     *          - Motor mixing matrix is valid for frame type
     *          - At least minimum number of motors are enabled
     *          - Thrust linearization parameters are reasonable
     *          - Battery compensation parameters are safe
     * 
     *          If any check fails, a descriptive error message is written to buffer and the
     *          vehicle will refuse to arm until the issue is corrected.
     * 
     * @note Overrides AP_Motors::arming_checks() base class method.
     * @note Called by vehicle arming logic before each arming attempt.
     * @note Buffer must be at least 50 characters to hold typical error messages.
     * 
     * @warning Failed arming checks indicate potentially unsafe configuration - do not bypass.
     * 
     * Source: libraries/AP_Motors/AP_MotorsMulticopter.cpp
     */
    bool arming_checks(size_t buflen, char *buffer) const override;

    /**
     * @brief Get average maximum throttle across all enabled motors - for AP_Motors example only
     * 
     * @return Average maximum throttle value in range 0-1
     * 
     * @note This method is for the AP_Motors example program and is not used by vehicle code.
     * @note Provided for testing and demonstration purposes.
     */
    float get_throttle_avg_max() const;
    
    /**
     * @brief Get current yaw headroom value - for AP_Motors example only
     * 
     * @return Yaw headroom in microseconds
     * 
     * @note This method is for the AP_Motors example program and is not used by vehicle code.
     * @note Provided for testing and demonstration purposes.
     * 
     * @see set_yaw_headroom() to modify yaw headroom
     */
    int16_t get_yaw_headroom() const;

    /**
     * @brief Thrust linearization handler - compensates for non-linear motor/ESC/prop response
     * 
     * @details The Thrust_Linearization object handles conversion between desired thrust (linear)
     *          and actuator output (non-linear ESC response). Key methods:
     *          - thrust_to_actuator(thrust): Converts desired thrust to ESC command using MOT_THST_EXPO curve
     *          - actuator_to_thrust(actuator): Inverse conversion for control loops
     * 
     *          MOT_THST_EXPO parameter controls the linearization curve:
     *          - 0.0: Linear (no compensation) - rarely correct for real systems
     *          - 0.65: Default second-order approximation - works for most systems
     *          - 1.0: Full quadratic compensation - for very non-linear systems
     * 
     * @note Public member accessible to derived classes for motor mixing calculations.
     * @note Battery voltage compensation is integrated into thrust calculations.
     * 
     * @see AP_Motors_Thrust_Linearization class for implementation details
     */
    Thrust_Linearization thr_lin {*this};

    /**
     * @brief Parameter table for motor configuration parameters
     * 
     * @details Defines all configurable parameters for multicopter motors including:
     *          - MOT_PWM_MIN/MAX: PWM output range (microseconds)
     *          - MOT_SPIN_MIN/MAX/ARM: Motor speed limits (0-1 ratio)
     *          - MOT_YAW_HEADROOM: Reserved PWM for yaw (microseconds)
     *          - MOT_THST_HOVER: Learned hover throttle (0-1)
     *          - MOT_THST_EXPO: Thrust linearization curve (0-1)
     *          - MOT_BAT_VOLT_MAX/MIN: Battery voltage compensation limits (volts)
     *          - MOT_BAT_CURR_MAX: Battery current limit (amperes)
     *          - MOT_SPOOL_TIME: Motor spool up/down time (seconds)
     *          - MOT_SLEW_UP_TIME/DOWN_TIME: Thrust slew rate limiting (seconds)
     * 
     * @note These parameters are exposed to ground control stations for configuration.
     * @note Parameter changes take effect immediately without reboot.
     */
    static const struct AP_Param::GroupInfo        var_info[];

protected:

    /**
     * @brief Run spool state machine and update motor spin-up/spin-down logic
     * 
     * @details Implements the spool state machine that manages motor transitions:
     *          - SHUT_DOWN → GROUND_IDLE: When arming
     *          - GROUND_IDLE → SPOOLING_UP: When throttle increases above idle
     *          - SPOOLING_UP → THROTTLE_UNLIMITED: After spool time elapses
     *          - THROTTLE_UNLIMITED → SPOOLING_DOWN: When throttle decreases to idle
     *          - SPOOLING_DOWN → GROUND_IDLE: After spool down time elapses
     *          - Any state → SHUT_DOWN: When disarming or emergency stop
     * 
     *          Updates _spin_up_ratio which scales motor outputs during transitions to provide
     *          smooth acceleration/deceleration and protect mechanical components.
     * 
     * @note Called by output() each loop iteration before motor mixing.
     * @note Spool timing controlled by MOT_SPOOL_TIME parameter (default 0.5s).
     * 
     * Source: libraries/AP_Motors/AP_MotorsMulticopter.cpp
     */
    void                output_logic();

    /**
     * @brief Frame-specific motor mixing implementation - MUST be implemented by derived classes
     * 
     * @details Pure virtual method that derived classes implement to perform frame-specific
     *          motor mixing. Takes roll/pitch/yaw/throttle commands from vehicle and converts
     *          them to individual motor thrust values based on frame geometry.
     * 
     *          Typical implementation sequence:
     *          1. Read roll/pitch/yaw/throttle from base class
     *          2. Apply frame-specific mixing matrix
     *          3. Normalize motor outputs if any exceed limits
     *          4. Store results in _actuator[] array
     * 
     * @note Called by output() after output_logic() updates spool state.
     * @note Must handle motor_enabled[] mask to skip disabled motors.
     * @note Must respect _throttle_thrust_max for battery compensation.
     * 
     * @see AP_MotorsMatrix::output_to_motors() for custom mixing implementation
     * @see AP_MotorsQuad::output_to_motors() for quadcopter mixing
     */
    virtual void        output_to_motors() = 0;

    /**
     * @brief Update throttle input filter to smooth pilot commands
     * 
     * @details Applies low-pass filtering to pilot throttle input to reduce control jitter
     *          and provide smoother motor response. Filter cutoff frequency is configurable
     *          and affects the trade-off between responsiveness and stability.
     * 
     * @note Overrides AP_Motors::update_throttle_filter() base class method.
     * @note Called automatically by output() each loop iteration.
     * @note Filter state is maintained across calls for continuous filtering.
     * 
     * Source: libraries/AP_Motors/AP_MotorsMulticopter.cpp
     */
    virtual void        update_throttle_filter() override;

    /**
     * @brief Calculate maximum throttle limit based on battery current limiting
     * 
     * @return Maximum throttle in range 0-1 after current limiting
     * 
     * @details Reduces maximum available throttle when battery current exceeds MOT_BAT_CURR_MAX
     *          to protect the battery from over-current damage. Uses MOT_BAT_CURR_TC time constant
     *          to filter current measurements and provide smooth limiting.
     * 
     *          Current limiting is essential for:
     *          - Preventing battery damage and voltage collapse
     *          - Extending battery life during aggressive maneuvers
     *          - Maintaining stable voltage for flight controller and sensors
     * 
     * @note Returns 1.0 if current limiting is disabled (MOT_BAT_CURR_MAX = 0).
     * @note Virtual method - can be overridden by derived classes for custom limiting.
     * 
     * Source: libraries/AP_Motors/AP_MotorsMulticopter.cpp
     */
    virtual float       get_current_limit_max_throttle();

    /**
     * @brief Convert actuator output (0-1) to PWM value (microseconds)
     * 
     * @param[in] _actuator_output Actuator value in range 0-1 (0=min thrust, 1=max thrust)
     * 
     * @return PWM value in microseconds (typically 1000-2000µs range)
     * 
     * @details Converts normalized actuator output through the following stages:
     *          1. Apply thrust linearization via thr_lin.thrust_to_actuator()
     *          2. Scale from 0-1 range to MOT_PWM_MIN to MOT_PWM_MAX range
     *          3. Constrain to valid PWM limits
     * 
     *          This is the final conversion before sending values to ESCs via HAL.
     * 
     * @note Actuator range 0-1 maps to PWM range MOT_PWM_MIN to MOT_PWM_MAX.
     * @note Thrust linearization compensation is applied inside this method.
     * 
     * Source: libraries/AP_Motors/AP_MotorsMulticopter.cpp
     */
    int16_t             output_to_pwm(float _actuator_output);

    /**
     * @brief Apply slew rate limiting to actuator output for mechanical protection
     * 
     * @param[in,out] actuator_output Current actuator value, modified to apply slew limit
     * @param[in]     input           Desired actuator value before slew limiting
     * 
     * @details Limits the rate of change of actuator output based on MOT_SLEW_UP_TIME and
     *          MOT_SLEW_DOWN_TIME parameters. Slew rate limiting:
     *          - Protects mechanical components (motors, ESCs, propellers) from sudden loads
     *          - Reduces stress on drivetrain during aggressive maneuvers
     *          - Can smooth out control oscillations
     * 
     *          Slew limiting is only applied when MOT_SLEW_TIME > 0 and not in SHUT_DOWN state.
     * 
     * @note actuator_output parameter is both input (previous value) and output (slew-limited value).
     * @note Different slew rates can be configured for increasing (up) and decreasing (down) thrust.
     * 
     * @warning Excessive slew limiting can reduce control authority and affect stability.
     * 
     * Source: libraries/AP_Motors/AP_MotorsMulticopter.cpp
     */
    void                set_actuator_with_slew(float& actuator_output, float input);

    /**
     * @brief Calculate actuator output for gradual spin-up to ground idle
     * 
     * @return Actuator value in range 0-1 during ground idle spin-up
     * 
     * @details Returns the appropriate actuator output during the transition from SHUT_DOWN to
     *          GROUND_IDLE state. Uses _spin_up_ratio to gradually increase from zero to
     *          MOT_SPIN_ARM over MOT_SAFE_TIME (default 1.0 second).
     * 
     *          This gradual spin-up:
     *          - Provides time for ESCs to initialize properly
     *          - Prevents sudden mechanical stress during arming
     *          - Allows pilot to verify motor rotation direction safely
     * 
     * @note Called by output_to_motors() implementations during GROUND_IDLE state.
     * @note Returns value based on MOT_SPIN_ARM parameter (default 0.10 = 10% throttle range).
     * 
     * Source: libraries/AP_Motors/AP_MotorsMulticopter.cpp
     */
    float               actuator_spin_up_to_ground_idle() const;

    /**
     * @brief Apply frame-specific thrust compensation - hook for derived classes
     * 
     * @details Virtual method that derived classes can override to apply frame-specific thrust
     *          compensation that isn't handled by the vehicle callback. Examples:
     *          - Compensation for motor tilt angles in tilted multicopters
     *          - Frame-specific aerodynamic interactions
     *          - Center of gravity offset corrections
     * 
     * @note Virtual method with empty default implementation.
     * @note Called by output() before output_to_motors() if no vehicle callback is registered.
     * @note Most frames don't need this - vehicle callback is preferred for complex compensation.
     */
    virtual void        thrust_compensation(void) {}

    /**
     * @brief Output additional thrust to booster motor if configured
     * 
     * @details Some custom frame configurations include a vertical booster motor for extra lift.
     *          This method outputs appropriate thrust to the booster based on collective thrust
     *          and the MOT_BOOST_SCALE parameter.
     * 
     * @note Virtual method - can be overridden by derived classes.
     * @note Default implementation handles basic booster motor configurations.
     * @note Booster motor output is independent of normal motor mixing.
     * 
     * Source: libraries/AP_Motors/AP_MotorsMulticopter.cpp
     */
    virtual void        output_boost_throttle(void);

    /**
     * @brief Output roll/pitch/yaw/thrust commands - alternative output method
     * 
     * @details Alternative to output_to_motors() that directly outputs roll, pitch, yaw, and
     *          thrust values without frame-specific mixing. Used by some external control
     *          systems or flight modes that provide direct motor commands.
     * 
     * @note Virtual method - can be overridden by derived classes.
     * @note Not commonly used - most control goes through output_to_motors() mixing path.
     * 
     * Source: libraries/AP_Motors/AP_MotorsMulticopter.cpp
     */
    virtual void        output_rpyt(void);

    /**
     * @brief Save motor parameters to EEPROM during disarm sequence
     * 
     * @details Called when vehicle disarms to save learned parameters to persistent storage:
     *          - MOT_THST_HOVER if MOT_HOVER_LEARN=2 (HOVER_LEARN_AND_SAVE)
     *          - Any other auto-learned motor parameters
     * 
     *          Saving on disarm rather than continuously prevents excessive EEPROM wear while
     *          still preserving learned values between flights.
     * 
     * @note Overrides AP_Motors::save_params_on_disarm() base class method.
     * @note Only saves if parameters have changed to minimize EEPROM writes.
     * 
     * Source: libraries/AP_Motors/AP_MotorsMulticopter.cpp
     */
    void                save_params_on_disarm() override;

    /**
     * @brief Update thrust limits from Lua scripting external control
     * 
     * @details Checks for and applies external thrust limits set by Lua scripts via the
     *          scripting interface. Allows scripts to temporarily limit motor thrust for:
     *          - Custom flight modes or maneuvers
     *          - Dynamic payload management
     *          - Advanced failsafe behaviors
     *          - Research and development experiments
     * 
     * @note Called by output_logic() each loop iteration.
     * @note External limits are transient - reset when script stops providing updates.
     * @note Scripting must be enabled (AP_SCRIPTING_ENABLED) for this to function.
     * 
     * Source: libraries/AP_Motors/AP_MotorsMulticopter.cpp
     */
    void                update_external_limits();

    /**
     * @enum HoverLearn
     * @brief Hover throttle learning mode configuration
     * 
     * @details Controls whether and how the system learns hover throttle (MOT_THST_HOVER) during flight.
     *          Hover learning improves altitude hold performance by adapting to vehicle weight, battery
     *          state, atmospheric conditions, and payload changes.
     */
    enum HoverLearn {
        HOVER_LEARN_DISABLED = 0,     ///< Hover learning disabled - use fixed MOT_THST_HOVER value
        HOVER_LEARN_ONLY = 1,         ///< Learn during flight but don't save to EEPROM (temporary learning)
        HOVER_LEARN_AND_SAVE = 2      ///< Learn during flight and save to EEPROM on disarm (persistent learning)
    };

    // ========== Motor Configuration Parameters (exposed via AP_Param) ==========
    
    AP_Int16            _yaw_headroom;          ///< MOT_YAW_HEADROOM: PWM reserved for yaw control (µs, default 200)
    AP_Float            _slew_up_time;          ///< MOT_SLEW_UP_TIME: Throttle increase slew rate limit (seconds, 0=disabled)
    AP_Float            _slew_dn_time;          ///< MOT_SLEW_DN_TIME: Throttle decrease slew rate limit (seconds, 0=disabled)
    AP_Float            _safe_time;             ///< MOT_SAFE_TIME: ESC initialization time from zero to minimum PWM (seconds, default 1.0)
    AP_Float            _spin_arm;              ///< MOT_SPIN_ARM: Armed ground idle throttle ratio 0-1 (default 0.10 = 10%)
    AP_Float            _batt_current_max;      ///< MOT_BAT_CURR_MAX: Current limit for thrust reduction (amperes, 0=disabled)
    AP_Float            _batt_current_time_constant;  ///< MOT_BAT_CURR_TC: Current measurement filter time constant (seconds, default 5.0)
    AP_Int16            _pwm_min;               ///< MOT_PWM_MIN: Minimum PWM output to ESCs (µs, 0=use RC input min)
    AP_Int16            _pwm_max;               ///< MOT_PWM_MAX: Maximum PWM output to ESCs (µs, 0=use RC input max)
    AP_Float            _throttle_hover;        ///< MOT_THST_HOVER: Learned hover throttle 0-1 (default 0.35 = 35%)
    AP_Int8             _throttle_hover_learn;  ///< MOT_HOVER_LEARN: Enable hover learning (0=off, 1=learn, 2=learn+save)
    AP_Int8             _disarm_disable_pwm;    ///< MOT_PWM_OFF: Disable PWM output when disarmed (0=output min PWM, 1=no output)

    // Tricopter-specific parameter
    AP_Float            _yaw_servo_angle_max_deg;  ///< MOT_YAW_SV_ANGLE: Maximum yaw servo lean angle (degrees, tricopter only)

    // Spool timing parameters
    AP_Float            _spool_up_time;       ///< MOT_SPOOL_TIME: Time to spool from ground idle to flight speed (seconds, default 0.5)
    AP_Float            _spool_down_time;     ///< MOT_SPOOL_DN_TIME: Time to spool down from flight to ground idle (seconds)

    // Booster motor parameter
    AP_Float            _boost_scale;         ///< MOT_BOOST_SCALE: Throttle scaling for booster motor (0-1, 0=disabled)

    // ========== Motor Configuration State (not parameters) ==========
    
    bool                motor_enabled[AP_MOTORS_MAX_NUM_MOTORS];  ///< Motor enable mask - true if motor is enabled in frame config

    // ========== Spool State Machine Variables ==========
    
    float               _spin_up_ratio;       ///< Spool progress ratio 0-1: 0=shut down, 1=full flight speed

    // ========== Battery Compensation State ==========
    
    float               _throttle_limit;      ///< Voltage-based thrust limit ratio between hover and maximum (0-1)
    float               _throttle_thrust_max; ///< Maximum available thrust after battery compensation (0-1)
    float               _disarm_safe_timer;   ///< Timer for ESC safe initialization period (seconds remaining)

    // ========== Thrust Compensation ==========
    
    thrust_compensation_fn_t _thrust_compensation_callback;  ///< Vehicle callback for tiltrotor/tiltwing thrust compensation

    // ========== Motor Output Array ==========
    
    float _actuator[AP_MOTORS_MAX_NUM_MOTORS];  ///< Final actuator outputs 0-1 before PWM conversion, indexed by motor number

    /**
     * @brief Check if motor is enabled considering tiltrotor override mask
     * 
     * @param[in] i Motor number (0-based index)
     * 
     * @return true if motor is enabled and not overridden, false otherwise
     * 
     * @details Combines motor_enabled[] array with _motor_mask_override to determine if a motor
     *          should receive normal multicopter mixing commands. Used by tilt quadplanes where
     *          some motors may be temporarily controlled by fixed-wing flight code during transition.
     * 
     * @note _motor_mask_override is only set for tilt quadplane configurations.
     * @note Normal multicopters always have _motor_mask_override = 0, so this reduces to motor_enabled[i].
     * 
     * @see output_motor_mask() for tiltrotor motor control
     */
    bool motor_enabled_mask(uint8_t i) const {
        return motor_enabled[i] && (_motor_mask_override & (1U << i)) == 0;
    }

    /**
     * @brief Bitmask of motors overridden by tiltrotor forward flight control
     * 
     * @details When non-zero, indicates which motors are currently controlled by external systems
     *          (typically fixed-wing forward flight controller in tilt quadplanes) rather than
     *          normal multicopter mixing. These motors receive thrust commands via output_motor_mask()
     *          instead of output_to_motors().
     * 
     * @note Only used by tilt quadplane configurations - always 0 for standard multicopters.
     * @note Bit N set means motor N is overridden.
     */
    uint32_t _motor_mask_override;
};
