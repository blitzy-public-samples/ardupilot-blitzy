/**
 * @file AP_MotorsHeli_Single.h
 * @brief Motor control class for single main rotor helicopters with swashplate and tail rotor
 * 
 * @details This file defines the AP_MotorsHeli_Single class which provides motor control
 *          for traditional single main rotor helicopters. The implementation handles:
 *          - Swashplate control for main rotor cyclic and collective pitch
 *          - Tail rotor control for anti-torque and yaw authority
 *          - Rotor Speed Control (RSC) for main rotor governor/throttle
 *          - Multiple tail rotor configurations (servo, external gyro, direct drive)
 *          - Flybar and flybarless configurations
 *          - Thrust linearization for improved throttle response
 * 
 *          The class extends AP_MotorsHeli base class and integrates with:
 *          - AP_MotorsHeli_Swash for swashplate mixing and servo control
 *          - AP_MotorsHeli_RSC for rotor speed control
 *          - SRV_Channel for servo output management
 * 
 * @note This is flight-critical code - changes affect vehicle stability and control
 */
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>            // ArduPilot Mega Vector/Matrix math Library
#include <SRV_Channel/SRV_Channel.h>
#include "AP_MotorsHeli.h"
#include "AP_MotorsHeli_RSC.h"
#include "AP_MotorsHeli_Swash.h"
#include "AP_Motors_Thrust_Linearization.h"

// rsc and extgyro function output channels.
#define AP_MOTORS_HELI_SINGLE_EXTGYRO                          CH_7
#define AP_MOTORS_HELI_SINGLE_TAILRSC                          CH_7

// direct-drive variable pitch defaults
#define AP_MOTORS_HELI_SINGLE_DDVP_SPEED_DEFAULT               50

// default external gyro gain
#define AP_MOTORS_HELI_SINGLE_EXT_GYRO_GAIN                    350

// COLYAW parameter min and max values
#define AP_MOTORS_HELI_SINGLE_COLYAW_RANGE                     5.0f

// maximum number of swashplate servos
#define AP_MOTORS_HELI_SINGLE_NUM_SWASHPLATE_SERVOS            3

/**
 * @class AP_MotorsHeli_Single
 * @brief Motor control for single main rotor helicopters with swashplate and tail rotor
 * 
 * @details This class provides comprehensive motor control for traditional single main rotor
 *          helicopters, handling the complete control chain from attitude commands to servo outputs.
 * 
 *          **Architecture and Components:**
 *          - **Main Rotor Swashplate**: Controlled via AP_MotorsHeli_Swash object, converts roll/pitch/collective
 *            commands into cyclic and collective pitch angles for the main rotor blades
 *          - **Tail Rotor**: Provides anti-torque force and yaw control, supports multiple configurations:
 *            * Servo-controlled variable pitch tail rotor
 *            * Servo with external gyro for enhanced stability
 *            * Direct-drive variable pitch (DDVP) with electronic speed control
 *            * Direct-drive fixed pitch (DDFP) clockwise or counter-clockwise
 *          - **Rotor Speed Control (RSC)**: Manages main rotor RPM via throttle/governor control,
 *            implemented in AP_MotorsHeli_RSC
 * 
 *          **Control Mixing:**
 *          The class performs the following control transformations:
 *          1. Receives desired roll, pitch, yaw rates and collective from attitude controller
 *          2. Applies collective-to-yaw coupling compensation (H_COLYAW parameter)
 *          3. Converts roll/pitch commands to swashplate cyclic pitch via mixing matrix
 *          4. Converts collective command to swashplate collective pitch
 *          5. Converts yaw command to tail rotor pitch or thrust
 *          6. Outputs servo positions to swashplate servos (typically 3) and tail rotor servo/ESC
 * 
 *          **Integration with AP_MotorsHeli Base Class:**
 *          - Extends AP_MotorsHeli, inheriting common helicopter motor control functionality
 *          - Implements vehicle-specific output_to_motors() for single rotor configuration
 *          - Overrides rotor speed control methods for main and tail rotor coordination
 *          - Provides arming checks specific to single rotor configurations
 * 
 *          **Configuration Parameters:**
 *          Key parameters controlling behavior (see var_info for complete list):
 *          - H_COL_MIN, H_COL_MAX, H_COL_MID: Collective pitch range and midpoint
 *          - H_CYC_MAX: Maximum cyclic pitch angle
 *          - H_SV_MAN: Manual servo mode for setup
 *          - H_TAIL_TYPE: Tail rotor configuration type (servo, external gyro, direct drive)
 *          - H_COLYAW: Collective-to-yaw coupling compensation scale
 *          - H_GYR_GAIN: External gyro gain (if using external gyro tail configuration)
 * 
 * @note Collective pitch range typically spans -2000 to +2000 in PWM microseconds, representing
 *       full negative to full positive blade pitch
 * 
 * @note Tail rotor automatically compensates for main rotor torque variations with collective pitch
 *       changes via the collective-to-yaw scale parameter
 * 
 * @note External gyro mode (SERVO_EXTGYRO) provides superior tail rotor stabilization by delegating
 *       rate damping to a dedicated external gyro, with the flight controller providing only
 *       heading hold commands
 * 
 * @note Throttle curve uses cubic spline interpolation for smooth power delivery across the
 *       collective pitch range, configured in the RSC subsystem
 * 
 * @warning Incorrect collective pitch ranges (H_COL_MIN/MAX) can cause control saturation,
 *          limiting cyclic authority and potentially causing loss of control
 * 
 * @warning Swashplate servo reversals or incorrect phasing will cause unstable flight - always
 *          verify correct swashplate operation on the ground before flight
 * 
 * @see AP_MotorsHeli - Base class for all helicopter motor control
 * @see AP_MotorsHeli_Swash - Swashplate mixing and servo control
 * @see AP_MotorsHeli_RSC - Rotor speed control (governor/throttle)
 */
class AP_MotorsHeli_Single : public AP_MotorsHeli {
public:
    /**
     * @brief Constructor for single main rotor helicopter motor control
     * 
     * @details Initializes the motor control system for a single main rotor helicopter,
     *          setting up the tail rotor RSC and swashplate servo control objects.
     *          The constructor configures:
     *          - Tail rotor with RSC channel and output channel assignment
     *          - Swashplate with servo channel assignments for 3 servos plus auxiliary
     *          - Default parameter values via AP_Param system
     * 
     * @param[in] speed_hz Servo update rate in Hertz, defaults to AP_MOTORS_HELI_SPEED_DEFAULT (125Hz)
     *                     Higher rates provide better servo response but may cause servo heating
     * 
     * @note Constructor initializes _tail_rotor with k_heli_tail_rsc function and CH_7 output
     * @note Constructor initializes _swashplate with servo channels MOT_1, MOT_2, MOT_3, and MOT_5
     */
    AP_MotorsHeli_Single(uint16_t speed_hz = AP_MOTORS_HELI_SPEED_DEFAULT) :
        AP_MotorsHeli(speed_hz),
        _tail_rotor(SRV_Channel::k_heli_tail_rsc, AP_MOTORS_HELI_SINGLE_TAILRSC, 1U),
        _swashplate(AP_MOTORS_MOT_1, AP_MOTORS_MOT_2, AP_MOTORS_MOT_3, AP_MOTORS_MOT_5, 1U)
    {
        AP_Param::setup_object_defaults(this, var_info);
    };

    /**
     * @brief Set servo output update rate for swashplate and tail rotor servos
     * 
     * @details Configures the PWM update rate for all helicopter servos including swashplate
     *          cyclic/collective servos and tail rotor servo. Higher update rates provide better
     *          servo response and reduced lag but may cause servo heating or excessive power consumption.
     *          Typical rates: 50Hz (analog servos), 125Hz (digital servos), 333Hz (high-speed digital).
     * 
     * @param[in] speed_hz Update rate in Hertz (typically 50-400 Hz)
     * 
     * @note This overrides the base class to set update rates for both swashplate and tail servos
     * @note Update rate must be compatible with servo specifications to avoid damage
     * @note Called during initialization and when parameters are changed
     */
    void set_update_rate(uint16_t speed_hz) override;

    /**
     * @brief Main output method - sends commands to swashplate servos, tail rotor, and ESC
     * 
     * @details This is the primary output method called at high frequency (typically 400Hz) to send
     *          control commands to all helicopter actuators. The method:
     *          1. Applies servo test overrides if servo_test() is active
     *          2. Sends cyclic and collective commands to swashplate servos
     *          3. Sends yaw command to tail rotor servo or ESC
     *          4. Manages rotor speed controller outputs for main and tail rotors
     *          5. Handles arming/disarming states and safety interlocks
     * 
     *          The method translates high-level roll/pitch/yaw/collective commands (set by attitude
     *          controller) into individual servo positions through the swashplate mixing matrix and
     *          tail rotor control laws.
     * 
     * @note Called at main loop rate, typically 400Hz for stable control
     * @note This method directly controls flight-critical actuators
     * 
     * @warning This method directly controls swashplate and tail rotor - incorrect mixing or
     *          servo assignments can cause loss of control or unstable flight
     * @warning Ensure swashplate phasing and servo reversals are correct before flight
     * 
     * @see move_actuators() - Internal method that performs the actual mixing calculations
     * @see AP_MotorsHeli_Swash::output() - Swashplate servo output implementation
     */
    void output_to_motors() override;

    /**
     * @brief Set target main rotor speed
     * 
     * @details Sets the desired main rotor RPM as a normalized value for the rotor speed controller (RSC).
     *          The RSC translates this normalized value into throttle or governor commands based on
     *          the configured throttle curve and RSC mode (throttle curve, external governor, or throttle passthrough).
     *          The actual rotor acceleration is limited by the H_RSC_RAMP_TIME and H_RSC_RUNUP_TIME parameters
     *          to prevent damage from rapid power changes.
     * 
     * @param[in] desired_speed Target rotor speed, normalized range 0.0 to 1.0
     *                          - 0.0 = rotor stopped (motor at idle or off)
     *                          - 1.0 = full rotor speed (nominal head speed)
     *                          - Values are clamped to [0.0, 1.0] range
     * 
     * @note The RSC system ramps speed changes gradually to protect mechanical components
     * @note Actual rotor RPM depends on throttle curve configuration and governor tuning
     * @note Called by flight mode logic and altitude controller
     * 
     * @see AP_MotorsHeli_RSC::set_desired_speed() - RSC implementation
     */
    void set_desired_rotor_speed(float desired_speed) override;

    /**
     * @brief Recalculate control scaling factors and mixing parameters
     * 
     * @details Recalculates internal scaling factors used for control mixing based on current
     *          parameter values. This includes:
     *          - Collective pitch range scaling (H_COL_MIN, H_COL_MAX, H_COL_MID)
     *          - Cyclic pitch limits (H_CYC_MAX)
     *          - Collective-to-yaw coupling scale (H_COLYAW)
     *          - Swashplate geometry and mixing matrix updates
     * 
     *          Called when parameters are modified or during initialization to ensure all
     *          internal calculations use current parameter values.
     * 
     * @note Called during parameter updates and initialization
     * @note Should not be called at high frequency - only when parameters change
     */
    void calculate_scalars() override;

    /**
     * @brief Recalculate scalars that are allowed to change while vehicle is armed
     * 
     * @details Recalculates a subset of control scaling factors that can safely be updated
     *          during flight without causing control discontinuities. These typically include
     *          parameters that affect control feel but not fundamental mixing, such as:
     *          - External gyro gains
     *          - Yaw trim adjustments
     *          - RSC throttle curve points
     * 
     * @note Called periodically even when armed to allow in-flight parameter tuning
     * @note Only updates parameters that won't cause control jumps or instability
     */
    void calculate_armed_scalars() override;

    /**
     * @brief Get bitmask of PWM outputs used by helicopter motor control
     * 
     * @details Returns a bitmask indicating which PWM output channels are currently being used
     *          for helicopter motor/servo control (swashplate servos, tail rotor, RSC). Each bit
     *          position corresponds to an output channel number, with bit=1 indicating that channel
     *          is in use by the motor control system.
     * 
     *          This information is used to:
     *          - Prevent auxiliary functions (camera trigger, gripper, etc.) from using motor channels
     *          - Configure the servo output library's channel allocation
     *          - Display active channels in ground station and logs
     * 
     *          For single rotor helicopters, typically uses:
     *          - 3 channels for swashplate servos (MOT_1, MOT_2, MOT_3)
     *          - 1 channel for tail rotor (MOT_4 or MOT_7 depending on configuration)
     *          - 1 channel for main rotor RSC (MOT_8)
     *          - Optional: 1 channel for tail rotor RSC (CH_7) in direct-drive configurations
     * 
     * @return Bitmask with bit N set if PWM output channel N is used for motor control
     * 
     * @note Mask changes based on tail rotor type (servo, external gyro, direct drive)
     * @note Used by SRV_Channel library to prevent output conflicts
     */
    uint32_t get_motor_mask() override;

    /**
     * @brief Set external tail gyro gain value
     * 
     * @details Sets the PWM value sent to an external tail rotor gyro (when H_TAIL_TYPE = SERVO_EXTGYRO).
     *          External gyros provide superior yaw rate damping by using a dedicated rate sensor and
     *          control loop. The gain parameter is sent as a PWM value on channel 7 to configure the
     *          gyro's sensitivity. Higher values increase gyro authority and damping.
     * 
     * @param[in] gain External gyro gain setting, range 0-1000 PWM microseconds
     *                 - Typical values: 200-500
     *                 - Higher = more aggressive damping
     *                 - Lower = softer damping, more pilot authority
     *                 - Values outside [0, 1000] are ignored (parameter unchanged)
     * 
     * @note Only effective when tail type is configured as SERVO_EXTGYRO
     * @note Gain value is in PWM microseconds, not a percentage
     * @note Default gain is 350 (AP_MOTORS_HELI_SINGLE_EXT_GYRO_GAIN)
     */
    void ext_gyro_gain(float gain)  override { if (gain >= 0 && gain <= 1000) { _ext_gyro_gain_std.set(gain); }}

    /**
     * @brief Check if helicopter has a mechanical flybar
     * 
     * @details Returns true if this helicopter is configured with a mechanical flybar stabilization
     *          system (Bell/Hiller bar). Flybar presence affects which attitude controller is used:
     *          - Flybar helicopters use rate-based ACRO control (no angle stabilization)
     *          - Flybarless helicopters can use full attitude stabilization
     * 
     * @return true if flybar is present (H_FLYBAR_MODE = 1), false for flybarless (H_FLYBAR_MODE = 0)
     * 
     * @note Flybar mode parameter should match physical helicopter configuration
     * @note Using incorrect flybar setting can cause control oscillations or poor handling
     */
    bool has_flybar() const  override { return _flybar_mode; }

    /**
     * @brief Check if yaw passthrough mode is supported
     * 
     * @details Returns true if the current tail rotor configuration supports yaw passthrough mode,
     *          where pilot yaw stick input is passed directly to the tail without rate control.
     *          This is only available with external gyro tail configuration, where the external
     *          gyro provides rate damping while the flight controller provides heading commands.
     * 
     * @return true if tail type is SERVO_EXTGYRO, false otherwise
     * 
     * @note Yaw passthrough provides improved tail response with external gyros
     * @note Not available with direct-drive or basic servo tail configurations
     */
    bool supports_yaw_passthrough() const override { return get_tail_type() == TAIL_TYPE::SERVO_EXTGYRO; }

    /**
     * @brief Enable/disable ACRO tail mode for different gyro gains
     * 
     * @details Sets flag to use ACRO-specific external gyro gain (_ext_gyro_gain_acro) instead of
     *          standard gain (_ext_gyro_gain_std). ACRO mode typically uses lower gyro gain to
     *          give pilot more direct authority for aerobatic maneuvers.
     * 
     * @param[in] set true to use ACRO gyro gain, false to use standard gyro gain
     * 
     * @note Only affects behavior with external gyro tail configuration
     * @note Called by flight mode code when entering/exiting ACRO mode
     */
    void set_acro_tail(bool set) override { _acro_tail = set; }

    /**
     * @brief Run pre-arm safety checks specific to single rotor helicopters
     * 
     * @details Performs comprehensive safety validation before allowing vehicle arming.
     *          Checks include:
     *          - Collective pitch range parameters (H_COL_MIN < H_COL_MAX)
     *          - Cyclic pitch limit is reasonable (H_CYC_MAX > 0)
     *          - Swashplate servo configuration is valid
     *          - Tail rotor configuration is appropriate for selected type
     *          - RSC (Rotor Speed Control) parameters are valid
     *          - External gyro settings are appropriate if using SERVO_EXTGYRO
     * 
     *          If any check fails, the method returns false and writes a human-readable
     *          error message to the provided buffer for display on ground station.
     * 
     * @param[in]  buflen Size of error message buffer in bytes
     * @param[out] buffer Buffer to write error message if check fails
     * 
     * @return true if all checks pass (safe to arm), false if any check fails
     * 
     * @note Called by arming system before allowing vehicle to arm
     * @note Prevents arming with invalid configurations that could cause loss of control
     * 
     * @warning Do not bypass these checks - invalid parameters can cause crashes
     * 
     * @see AP_Arming::pre_arm_checks() - Main arming check coordinator
     */
    bool arming_checks(size_t buflen, char *buffer) const override;

    /**
     * @brief Perform parameter format conversions for legacy compatibility
     * 
     * @details Handles automatic conversion of old parameter formats to new formats during
     *          firmware updates. This ensures backward compatibility when parameter definitions
     *          change between firmware versions. Conversions may include:
     *          - Renaming parameters that changed names
     *          - Scaling parameters that changed units or ranges
     *          - Migrating from old collective/cyclic representations to new formats
     *          - Converting old tail type enums to new tail type system
     * 
     * @note Called once during initialization
     * @note Conversions are one-way and permanent (old parameter is deleted after conversion)
     * @note Allows seamless upgrades without requiring manual parameter reconfiguration
     */
    void heli_motors_param_conversions(void) override;

    /**
     * @brief Thrust linearization object for improved altitude control
     * 
     * @details Provides thrust linearization to improve altitude controller performance by
     *          compensating for non-linear relationship between collective pitch and thrust.
     *          The linearization learns the actual thrust curve through flight data and applies
     *          corrections to make throttle response more linear and predictable.
     * 
     * @note Improves altitude hold accuracy and reduces overshoot/oscillation
     * @see AP_Motors_Thrust_Linearization for implementation details
     */
    Thrust_Linearization thr_lin {*this};

#if HAL_LOGGING_ENABLED
    /**
     * @brief Write helicopter-specific telemetry to dataflash log
     * 
     * @details Logs detailed helicopter state information for post-flight analysis and debugging.
     *          Logged data includes:
     *          - Current blade collective and cyclic angles
     *          - Tail rotor position/thrust
     *          - Main rotor speed (RPM)
     *          - Swashplate servo positions
     *          - RSC state and throttle output
     * 
     * @note Called at 10 Hz to balance detail with log storage space
     * @note Critical for diagnosing control issues and tuning problems
     * @note Log messages use HELI message type in dataflash
     */
    void Log_Write(void) override;
#endif

    /**
     * @brief Parameter table definition for AP_Param system
     * 
     * @details Defines all configurable parameters for single rotor helicopter control.
     *          Parameters are organized into groups and include:
     * 
     *          **Collective Control (H_COL_*):**
     *          - H_COL_MIN: Minimum collective pitch (typically -2000)
     *          - H_COL_MAX: Maximum collective pitch (typically +2000)
     *          - H_COL_MID: Mid-point collective for hover (typically 0)
     * 
     *          **Cyclic Control (H_CYC_*):**
     *          - H_CYC_MAX: Maximum cyclic pitch angle limit
     * 
     *          **Tail Configuration (H_TAIL_*, H_GYR_*):**
     *          - H_TAIL_TYPE: Tail rotor configuration type (enum TAIL_TYPE)
     *          - H_GYR_GAIN: External gyro gain (standard mode)
     *          - H_GYR_GAIN_ACRO: External gyro gain (ACRO mode)
     * 
     *          **Coupling Compensation (H_COLYAW):**
     *          - H_COLYAW: Collective-to-yaw coupling scale (-5.0 to +5.0)
     * 
     *          **Other:**
     *          - H_FLYBAR_MODE: Flybar present (0/1)
     *          - H_SV_MAN: Manual servo mode for setup
     *          - H_DDVP_SPEED: Direct-drive tail speed setting
     * 
     * @note Parameters are persistent across reboots (stored in EEPROM/flash)
     * @note Changing parameters requires calculate_scalars() call to take effect
     * 
     * @warning Incorrect collective pitch ranges can cause control saturation and loss of control
     */
    static const struct AP_Param::GroupInfo var_info[];

protected:

    /**
     * @brief Initialize servo/PWM output ranges and endpoints
     * 
     * @details Configures all servo outputs for helicopter control including:
     *          - Swashplate servo PWM ranges (min, max, trim)
     *          - Tail rotor servo/ESC PWM range
     *          - Main rotor RSC throttle output range
     *          - Servo update rates for all channels
     * 
     *          Sets up the SRV_Channel library with appropriate ranges and functions
     *          for each servo, ensuring outputs stay within safe mechanical limits.
     * 
     * @note Called once during motor initialization
     * @note Reads servo range parameters (SERVOn_MIN, SERVOn_MAX, SERVOn_TRIM)
     * @note Must be called before first output_to_motors()
     */
    void init_outputs() override;

    /**
     * @brief Send commands to main and tail rotor speed controllers
     * 
     * @details Updates the rotor speed controller (RSC) outputs based on current control state.
     *          Manages rotor spool-up, spool-down, and steady-state speed control. Coordinates
     *          main rotor RSC with tail rotor RSC (in direct-drive configurations).
     * 
     * @param[in] state Current rotor control state (STOPPED, IDLE, ACTIVE, etc.)
     * 
     * @note Called from output_to_motors() at main loop rate
     * @see AP_MotorsHeli_RSC::RotorControlState for state definitions
     */
    void update_motor_control(AP_MotorsHeli_RSC::RotorControlState state) override;

    /**
     * @brief Perform swashplate and tail rotor mixing and output
     * 
     * @details Core mixing function that converts high-level roll/pitch/yaw/collective commands
     *          into individual servo positions. Process flow:
     *          1. Apply collective-to-yaw coupling compensation
     *          2. Send roll/pitch/collective to swashplate mixer
     *          3. Swashplate calculates cyclic and collective blade angles
     *          4. Swashplate mixer converts to 3 servo positions
     *          5. Send yaw command to tail rotor control
     * 
     * @param[in] roll_out  Roll output from attitude controller (-1.0 to +1.0)
     * @param[in] pitch_out Pitch output from attitude controller (-1.0 to +1.0)
     * @param[in] coll_in   Collective input from throttle/altitude controller (0.0 to 1.0)
     * @param[in] yaw_out   Yaw output from attitude controller (-1.0 to +1.0)
     * 
     * @note Called from output_to_motors() after motor control state update
     * @note All inputs are normalized values, scaled internally based on parameters
     * 
     * @warning Incorrect swashplate configuration will result in incorrect mixing and unstable flight
     */
    void move_actuators(float roll_out, float pitch_out, float coll_in, float yaw_out) override;

    /**
     * @brief Calculate and output tail rotor control
     * 
     * @details Converts yaw command to tail rotor servo position or ESC throttle, applying:
     *          - Collective-to-yaw coupling compensation
     *          - Yaw trim offset
     *          - Servo/ESC range scaling
     *          - External gyro gain (if SERVO_EXTGYRO mode)
     * 
     * @param[in] yaw_out Yaw command from attitude controller (-1.0 to +1.0)
     * 
     * @note Called from move_actuators()
     * @note Behavior depends on tail type configuration (servo, external gyro, direct drive)
     */
    void move_yaw(float yaw_out);

    /**
     * @brief Calculate yaw offset to compensate for main rotor torque
     * 
     * @details Computes the tail rotor offset required to cancel steady-state main rotor torque
     *          at a given collective pitch. As collective increases, main rotor power and torque
     *          increase, requiring more tail rotor thrust. This function applies the collective-to-yaw
     *          scale parameter (H_COLYAW) to automatically add tail rotor compensation.
     * 
     * @param[in] collective Current collective pitch command (0.0 to 1.0 normalized)
     * 
     * @return Yaw offset value to add to yaw command (-1.0 to +1.0)
     * 
     * @note Scale can be positive or negative depending on rotor rotation direction
     * @note Provides feedforward compensation to reduce yaw integrator buildup
     */
    float get_yaw_offset(float collective);

    /**
     * @brief Handle direct-drive fixed-pitch tail output with limit flags
     * 
     * @details Sends throttle command to direct-drive fixed-pitch tail rotor ESC while managing
     *          output limit flags. Direct-drive fixed-pitch tails use throttle variation for
     *          yaw control rather than collective pitch variation.
     * 
     * @param[in] throttle Throttle command for tail ESC (0.0 to 1.0)
     * 
     * @note Only used when tail type is DIRECTDRIVE_FIXEDPITCH_CW or DIRECTDRIVE_FIXEDPITCH_CCW
     * @note Updates output limit flags for altitude controller awareness
     */
    void output_to_ddfp_tail(float throttle);

    /**
     * @brief Move servos through full range for ground testing
     * 
     * @details Executes automated servo test sequence that cycles all servos through their full
     *          range of motion for visual verification of:
     *          - Correct servo assignment and phasing
     *          - Swashplate geometry and mixing
     *          - Servo direction and trim
     *          - Full mechanical range without binding
     * 
     *          Test sequence oscillates cyclic inputs while slowly varying collective from
     *          min to max, allowing inspection of servo motion throughout the entire control envelope.
     * 
     * @note Called when H_SV_MAN servo test mode is enabled
     * @note Overrides normal control inputs during test
     * 
     * @warning Requires all rotor blades and propellers removed before use
     * @warning Servos will move to full deflection - ensure no obstructions
     * 
     * @see _servo_test_cycle_time, _oscillate_angle for test state tracking
     */
    void servo_test() override;

    /**
     * @enum TAIL_TYPE
     * @brief Tail rotor configuration types for single rotor helicopters
     * 
     * @details Defines the supported tail rotor control configurations, each with different
     *          hardware requirements and control characteristics:
     * 
     *          **SERVO (0)**: Traditional servo-controlled variable pitch tail rotor
     *          - Standard mechanical tail rotor with servo controlling collective pitch
     *          - Flight controller provides full yaw rate control
     *          - Simplest configuration, good for basic helicopters
     * 
     *          **SERVO_EXTGYRO (1)**: Servo-controlled tail with external gyro stabilization
     *          - Servo-controlled variable pitch with dedicated external rate gyro
     *          - External gyro provides superior rate damping and disturbance rejection
     *          - Flight controller sends heading commands, gyro provides rate feedback
     *          - Recommended for best yaw performance with mechanical tail
     *          - Requires external gyro (e.g., Spartan, CGY750) on CH7 gain input
     * 
     *          **DIRECTDRIVE_VARPITCH (2)**: Direct-drive variable pitch tail rotor
     *          - Electric motor directly drives variable pitch tail rotor
     *          - ESC throttle controls motor speed (anti-torque thrust)
     *          - Collective pitch adjusted via servo for additional control authority
     *          - Used on some electric helicopter designs
     * 
     *          **DIRECTDRIVE_FIXEDPITCH_CW (3)**: Direct-drive fixed pitch clockwise tail rotor
     *          - Electric motor with fixed-pitch propeller rotating clockwise (viewed from behind)
     *          - Yaw control via ESC throttle variation only
     *          - Simple, lightweight, good for small electric helicopters
     *          - Motor rotation must match main rotor torque direction
     * 
     *          **DIRECTDRIVE_FIXEDPITCH_CCW (4)**: Direct-drive fixed pitch counter-clockwise tail rotor
     *          - Same as CW but propeller rotates counter-clockwise
     *          - Select based on main rotor rotation direction
     * 
     *          **DIRECTDRIVE_VARPIT_EXT_GOV (5)**: Direct-drive variable pitch with external governor
     *          - Variable pitch tail with motor speed managed by external governor
     *          - Governor maintains constant tail rotor RPM
     *          - Yaw control via collective pitch adjustment only
     *          - Used on high-performance setups with dedicated tail governor
     * 
     * @note Tail type must match physical hardware configuration
     * @note Incorrect tail type will cause poor yaw performance or no yaw control
     * 
     * @see H_TAIL_TYPE parameter for configuration
     */
    enum class TAIL_TYPE {
        SERVO = 0,                          ///< Servo-controlled variable pitch tail
        SERVO_EXTGYRO = 1,                  ///< Servo tail with external gyro
        DIRECTDRIVE_VARPITCH = 2,           ///< Direct-drive variable pitch tail
        DIRECTDRIVE_FIXEDPITCH_CW = 3,      ///< Direct-drive fixed pitch clockwise
        DIRECTDRIVE_FIXEDPITCH_CCW = 4,     ///< Direct-drive fixed pitch counter-clockwise
        DIRECTDRIVE_VARPIT_EXT_GOV = 5      ///< Direct-drive variable pitch with external governor
    };

    /**
     * @brief Get currently configured tail rotor type
     * 
     * @return Current tail type from H_TAIL_TYPE parameter
     * 
     * @note Returns strongly-typed enum value for type-safe comparisons
     */
    TAIL_TYPE get_tail_type() const { return TAIL_TYPE(_tail_type.get()); }

    /**
     * @brief Check if tail is direct-drive fixed-pitch configuration
     * 
     * @details Returns true if tail rotor is configured as direct-drive fixed pitch,
     *          either clockwise or counter-clockwise rotation. Used to select appropriate
     *          control algorithms and output methods for DDFP tails.
     * 
     * @return true if tail type is DIRECTDRIVE_FIXEDPITCH_CW or DIRECTDRIVE_FIXEDPITCH_CCW
     * 
     * @note DDFP tails use throttle control rather than collective pitch control
     */
    bool have_DDFP_tail() const;

    /**
     * @brief Check if tail rotor RSC (Rotor Speed Control) should be used
     * 
     * @details Returns true if the tail rotor configuration requires a dedicated RSC output
     *          for motor speed control. This is true for direct-drive tail configurations
     *          where the tail motor speed must be actively controlled.
     * 
     * @return true if tail configuration uses RSC (direct-drive types)
     * 
     * @note Servo-controlled tails do not use RSC (mechanical tail RPM follows main rotor)
     * @note RSC output typically uses CH7 for tail motor ESC
     */
    bool use_tail_RSC() const;

    /**
     * @brief Tail rotor RSC (Rotor Speed Control) object
     * 
     * @details Manages tail rotor motor speed control for direct-drive configurations.
     *          Handles throttle output, governor interface, and speed ramping for tail motor.
     *          Initialized with k_heli_tail_rsc function and CH_7 output channel.
     * 
     * @note Only actively used in direct-drive tail configurations
     * @note Servo-controlled tails do not use dedicated RSC (tail spins with main rotor)
     * 
     * @see AP_MotorsHeli_RSC for rotor speed control implementation
     */
    AP_MotorsHeli_RSC   _tail_rotor;

    /**
     * @brief Swashplate control and mixing object
     * 
     * @details Handles all swashplate-related functions including:
     *          - Swashplate geometry configuration (H120, H140, H135, etc.)
     *          - Cyclic and collective mixing to individual servo positions
     *          - Servo phasing and mechanical correction
     *          - Servo output to swashplate servos (typically MOT_1, MOT_2, MOT_3)
     * 
     *          The swashplate object translates roll/pitch/collective commands into the
     *          three servo positions required to achieve desired blade angles.
     * 
     * @note Swashplate type and geometry configured via H_SW_TYPE and related parameters
     * @note Correct swashplate configuration is critical for stable flight
     * 
     * @see AP_MotorsHeli_Swash for swashplate mixing implementation
     */
    AP_MotorsHeli_Swash _swashplate;

    /**
     * @name Servo Test State Variables
     * @brief Internal state tracking for automated servo test sequence
     * @{
     */
    float _oscillate_angle = 0.0f;              ///< Current cyclic oscillation angle in servo test (radians)
    float _servo_test_cycle_time = 0.0f;        ///< Elapsed time in current servo test cycle (seconds)
    float _collective_test = 0.0f;              ///< Collective override during servo test (-1.0 to +1.0)
    float _roll_test = 0.0f;                    ///< Roll override during servo test (-1.0 to +1.0)
    float _pitch_test = 0.0f;                   ///< Pitch override during servo test (-1.0 to +1.0)
    float _yaw_test = 0.0f;                     ///< Yaw override during servo test (-1.0 to +1.0)
    /** @} */

    /**
     * @brief Tail rotor servo/ESC output value
     * 
     * @details Stores the calculated output value sent to tail rotor servo or ESC.
     *          Value is in normalized range and gets converted to PWM microseconds
     *          by the servo output library.
     * 
     * @note Updated by move_yaw() and output_to_ddfp_tail()
     * @note Range depends on tail type: servo uses ±1.0, ESC uses 0.0-1.0
     */
    float _servo4_out = 0.0f;

    /**
     * @name Configuration Parameters
     * @brief Persistent configuration parameters stored in EEPROM/flash
     * 
     * @details These parameters control helicopter motor mixing and control behavior.
     *          All parameters are exposed via ground station for configuration and tuning.
     *          Parameter name prefix is typically H_ (e.g., H_TAIL_TYPE, H_COLYAW).
     * @{
     */

    /**
     * @brief Tail rotor configuration type (H_TAIL_TYPE)
     * 
     * @details Selects tail rotor control method from TAIL_TYPE enum:
     *          - 0 = SERVO: Traditional servo-controlled variable pitch
     *          - 1 = SERVO_EXTGYRO: Servo with external gyro stabilization
     *          - 2 = DIRECTDRIVE_VARPITCH: Electric direct-drive variable pitch
     *          - 3 = DIRECTDRIVE_FIXEDPITCH_CW: Fixed pitch clockwise
     *          - 4 = DIRECTDRIVE_FIXEDPITCH_CCW: Fixed pitch counter-clockwise
     *          - 5 = DIRECTDRIVE_VARPIT_EXT_GOV: Variable pitch with external governor
     * 
     * @note Must match physical tail configuration
     * @note Changing requires reboot to take full effect
     */
    AP_Int16        _tail_type;

    /**
     * @brief External gyro gain for standard flight modes (H_GYR_GAIN)
     * 
     * @details PWM value (0-1000) sent to external gyro on channel 7 to configure
     *          gyro sensitivity in stabilize, altitude hold, and autonomous modes.
     *          Higher values increase gyro authority and damping.
     * 
     * @note Only effective with SERVO_EXTGYRO tail type
     * @note Typical range: 200-500, default 350
     * @note Units are PWM microseconds offset, not percentage
     */
    AP_Int16        _ext_gyro_gain_std;

    /**
     * @brief External gyro gain for ACRO mode (H_GYR_GAIN_ACRO)
     * 
     * @details PWM value (0-1000) sent to external gyro on channel 7 during ACRO flight mode.
     *          Typically set lower than standard gain to provide more direct pilot authority
     *          for aerobatic maneuvers while still maintaining some rate damping.
     * 
     * @note Only effective with SERVO_EXTGYRO tail type
     * @note Usually 50-100 less than H_GYR_GAIN for improved aerobatic response
     */
    AP_Int16        _ext_gyro_gain_acro;

    /**
     * @brief Flybar mode flag (H_FLYBAR_MODE)
     * 
     * @details Indicates presence of mechanical flybar (Bell/Hiller stabilizer bar):
     *          - 0 = Flybarless: Full attitude stabilization available
     *          - 1 = Flybar: Uses rate-based ACRO control only
     * 
     *          Flybar helicopters have mechanical rate damping from the flybar, so the
     *          flight controller uses simpler rate control. Flybarless helicopters require
     *          the flight controller to provide all stabilization.
     * 
     * @note Must match physical helicopter configuration
     * @note Incorrect setting causes poor control response or oscillations
     */
    AP_Int8         _flybar_mode;

    /**
     * @brief Direct-drive tail motor speed (H_DDVP_SPEED)
     * 
     * @details ESC throttle setting for direct-drive variable pitch tail rotor, range 0-1000.
     *          Sets the baseline motor speed; yaw control achieved by varying blade pitch
     *          rather than motor speed.
     * 
     * @note Only used with DIRECTDRIVE_VARPITCH tail type
     * @note Default: 50 (AP_MOTORS_HELI_SINGLE_DDVP_SPEED_DEFAULT)
     * @note Higher speeds provide more control authority but increase power consumption
     */
    AP_Int16        _direct_drive_tailspeed;

    /**
     * @brief Collective-to-yaw coupling compensation scale (H_COLYAW)
     * 
     * @details Feedforward scale factor that automatically adds tail rotor thrust as collective
     *          pitch increases to compensate for increased main rotor torque. This reduces the
     *          need for the yaw integrator to wind up during climbs or descents.
     * 
     *          - Positive values: Add tail rotor thrust as collective increases
     *          - Negative values: Reduce tail rotor thrust as collective increases
     *          - Zero: No coupling compensation
     * 
     *          Sign depends on main rotor rotation direction (CW vs CCW) and tail rotor
     *          configuration. Typical magnitude: 0.5 to 2.0.
     * 
     * @note Range: -5.0 to +5.0 (AP_MOTORS_HELI_SINGLE_COLYAW_RANGE)
     * @note Correct tuning improves yaw response and reduces integrator windup
     * @note Too much coupling can cause yaw oscillations during altitude changes
     * 
     * @warning Incorrect sign will make yaw control worse instead of better
     */
    AP_Float        _collective_yaw_scale;

    /**
     * @brief Yaw output trim offset (H_YAW_TRIM)
     * 
     * @details Fixed offset added to tail rotor output to compensate for consistent yaw bias,
     *          allowing the yaw integrator to stay near zero during hover. This reduces integrator
     *          windup and improves response to yaw commands.
     * 
     *          Adjust this value so that the yaw integrator (visible in logs as PID.I) stays
     *          near zero during stable hover.
     * 
     * @note Range typically -1.0 to +1.0 (normalized)
     * @note Tune after H_COLYAW is properly set
     * @note Small adjustments (0.01-0.05) usually sufficient
     */
    AP_Float        _yaw_trim;

    /** @} */  // End of Configuration Parameters group

    /**
     * @brief ACRO tail mode active flag (runtime state, not a parameter)
     * 
     * @details Runtime flag indicating whether ACRO-specific tail control is active.
     *          When true, uses _ext_gyro_gain_acro instead of _ext_gyro_gain_std for
     *          external gyro configurations. Set by flight mode code via set_acro_tail().
     * 
     * @note Not a persistent parameter, reset to false on boot
     * @note Only affects behavior with SERVO_EXTGYRO tail type
     */
    bool            _acro_tail = false;
};
