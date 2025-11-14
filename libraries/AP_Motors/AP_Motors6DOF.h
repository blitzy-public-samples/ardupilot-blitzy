/**
 * @file AP_Motors6DOF.h
 * @brief Motor control class for vehicles requiring 6 Degrees of Freedom (6DOF) control
 * 
 * @details This file implements motor control for vehicles that require full 6DOF control
 *          capabilities, providing independent control over all three translational axes
 *          (X, Y, Z) and all three rotational axes (roll, pitch, yaw). This is primarily
 *          designed for underwater ROVs (Remotely Operated Vehicles) and submarines, but
 *          can also support advanced vectored-thrust aircraft.
 * 
 *          6DOF control enables:
 *          - Translation: Forward/backward (X), Left/right (Y), Up/down (Z)
 *          - Rotation: Roll, Pitch, Yaw
 * 
 *          Motor mixing calculates individual motor outputs based on arbitrary motor
 *          orientations and thrust directions, allowing vehicles to achieve motion in
 *          all six degrees of freedom simultaneously.
 * 
 * @note Supports reversible ESCs for bidirectional thrust capability
 * @note Includes battery current limiting for underwater vehicle safety
 * @note Forward-vertical coupling compensation for hydrodynamic effects
 */

#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <RC_Channel/RC_Channel.h>     // RC Channel Library
#include "AP_MotorsMatrix.h"

/**
 * @class AP_Motors6DOF
 * @brief Motor control for vehicles requiring full 6 Degrees of Freedom control
 * 
 * @details This class extends AP_MotorsMatrix to provide comprehensive 6DOF motor control
 *          for underwater ROVs, submarines, and advanced vectored-thrust vehicles.
 * 
 *          Purpose:
 *          - Control vehicles requiring independent motion in all 6 axes simultaneously
 *          - Translate desired 6-axis control inputs into individual motor thrust commands
 *          - Support arbitrary motor orientations and thrust directions
 * 
 *          Use Cases:
 *          - Underwater ROVs: Station-keeping, precise maneuvering, complex underwater tasks
 *          - Submarines: Full 3D navigation in underwater environments
 *          - Advanced vectored-thrust aircraft: VTOL with omnidirectional control
 * 
 *          Vectoring Approach:
 *          - Each motor contributes to multiple control axes based on its position and orientation
 *          - Motor factors define contribution to: forward, lateral, vertical (throttle), roll, pitch, yaw
 *          - Mixing matrix calculates required thrust per motor to achieve desired 6-axis motion
 * 
 *          Frame Types:
 *          - SUB_FRAME_VECTORED: Horizontal and vertical thrusters in fixed orientations
 *          - SUB_FRAME_VECTORED_6DOF: Full omnidirectional capability with angled thrusters
 *          - SUB_FRAME_VECTORED_6DOF_90DEG: 90-degree oriented thruster configuration
 *          - SUB_FRAME_SIMPLEROV_*: Simplified configurations with 3, 4, or 5 thrusters
 *          - SUB_FRAME_BLUEROV1: BlueROV1 frame configuration
 *          - SUB_FRAME_CUSTOM: User-defined motor positions and orientations
 * 
 * @note Motor factors per axis: forward, lateral, throttle (vertical), roll, pitch, yaw
 * @note Supports reversible motors with bidirectional ESCs for thrust in both directions
 * @note Normalization ensures control authority is maintained during motor saturation
 * @note Battery current limiting integrated to protect power systems underwater
 * @note Forward-vertical coupling factors compensate for hydrodynamic interaction effects
 * 
 * @warning Underwater use: Ensure all motors and ESCs are properly waterproofed
 * @warning Reversible ESCs required for bidirectional thrust - standard ESCs will not work
 * @warning Incorrect motor factor configuration causes loss of control in affected axes
 * @warning Test thoroughly in controlled environment (pool/tank) before open water deployment
 * @warning Current limiting is critical for battery protection in underwater applications
 */
class AP_Motors6DOF : public AP_MotorsMatrix {
public:

    /**
     * @brief Constructor for 6DOF motor controller
     * 
     * @details Initializes the 6DOF motor controller with specified PWM output frequency.
     *          Sets up parameter defaults for motor reversing and coupling factors.
     * 
     * @param[in] speed_hz PWM output frequency in Hz (default: AP_MOTORS_SPEED_DEFAULT)
     *                     Typical values: 50Hz for standard servos, 400Hz for ESCs
     * 
     * @note Calls AP_Param::setup_object_defaults to initialize all parameters
     */
    AP_Motors6DOF(uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT) :
        AP_MotorsMatrix(speed_hz) {
        AP_Param::setup_object_defaults(this, var_info);
    };

    /**
     * @brief Supported 6DOF frame types for underwater vehicles
     * 
     * @details Defines pre-configured frame geometries for various ROV configurations.
     *          Each frame type has specific motor positions and thrust vector orientations.
     */
    typedef enum {
        SUB_FRAME_BLUEROV1,           ///< BlueROV1 configuration: 6 thrusters (4 horizontal, 2 vertical)
        SUB_FRAME_VECTORED,           ///< Vectored frame: horizontal + vertical thrusters in standard orientation
        SUB_FRAME_VECTORED_6DOF,      ///< Full 6DOF: angled thrusters providing omnidirectional control
        SUB_FRAME_VECTORED_6DOF_90DEG,///< 6DOF with 90-degree thruster orientation
        SUB_FRAME_SIMPLEROV_3,        ///< Simplified 3-thruster configuration (limited DOF)
        SUB_FRAME_SIMPLEROV_4,        ///< Simplified 4-thruster configuration (limited DOF)
        SUB_FRAME_SIMPLEROV_5,        ///< Simplified 5-thruster configuration (limited DOF)
        SUB_FRAME_CUSTOM              ///< User-defined motor positions and thrust vectors
    } sub_frame_t;

    /**
     * @brief Initialize and configure 6DOF motor arrangement
     * 
     * @details Configures motor positions, thrust vector orientations, and contribution
     *          factors for each motor based on the selected frame type. Sets up the mixing
     *          matrix that maps 6-axis control inputs to individual motor outputs.
     * 
     *          This method:
     *          - Validates frame_class and frame_type compatibility
     *          - Configures motor output channels
     *          - Sets motor position relative to center of gravity
     *          - Defines thrust vector directions for each motor
     *          - Calculates motor factors (forward, lateral, vertical, roll, pitch, yaw)
     * 
     * @param[in] frame_class Motor frame class (e.g., MOTOR_FRAME_TYPE_PLUS, MOTOR_FRAME_TYPE_X)
     * @param[in] frame_type Specific 6DOF frame type from sub_frame_t enumeration
     * 
     * @note Must be called during vehicle initialization before motors are armed
     * @note For SUB_FRAME_CUSTOM, motor factors must be configured via parameters
     * 
     * @warning Incorrect frame configuration will result in unpredictable vehicle behavior
     */
    void setup_motors(motor_frame_class frame_class, motor_frame_type frame_type) override;

    /**
     * @brief Output minimum PWM values to all motors
     * 
     * @details Sends minimum PWM signals (typically 1100μs) to all configured motors.
     *          Used during initialization, disarming, and emergency stop conditions.
     *          For bidirectional ESCs, this represents zero thrust (motor stopped).
     * 
     * @note Override of AP_MotorsMatrix::output_min()
     * @note For reversible ESCs, minimum PWM = neutral point (1500μs typically)
     */
    void output_min() override;

    /**
     * @brief Convert normalized thrust to PWM output value
     * 
     * @details Maps thrust input in range -1.0 to +1.0 to PWM output range (typically 1100-1900μs).
     *          For bidirectional ESCs:
     *          - thrust = -1.0 → full reverse (1100μs)
     *          - thrust =  0.0 → neutral/stopped (1500μs)
     *          - thrust = +1.0 → full forward (1900μs)
     * 
     * @param[in] thrust_in Normalized thrust value from -1.0 (full reverse) to +1.0 (full forward)
     * 
     * @return PWM value in microseconds, typically 1100-1900μs
     * 
     * @note Handles bidirectional ESC mapping with neutral point at center
     * @note Thrust values outside [-1.0, 1.0] are clamped to valid range
     */
    int16_t calc_thrust_to_pwm(float thrust_in) const;

    /**
     * @brief Send calculated thrust values to physical motor outputs
     * 
     * @details Converts normalized thrust values (stored internally after mixing) to PWM signals
     *          and sends them to the corresponding motor output channels. Applies:
     *          - Thrust to PWM conversion (calc_thrust_to_pwm)
     *          - Motor reversing (if configured via _motor_reverse parameters)
     *          - Output limiting and scaling
     *          - Current limiting (via _output_limited factor)
     * 
     * @note Override of AP_MotorsMatrix::output_to_motors()
     * @note Called at motor output rate (typically 50-400Hz depending on configuration)
     * @note Respects motor enable/disable states
     * 
     * @warning Ensure motors are properly configured before calling
     */
    void output_to_motors() override;

    /**
     * @brief Set maximum throttle limit
     * 
     * @details Limits the maximum vertical thrust output (throttle axis). Used to restrict
     *          vehicle ascent rate or comply with power limitations. Does not affect
     *          horizontal thrust (forward/lateral) or rotational control (roll/pitch/yaw).
     * 
     * @param[in] max_throttle Maximum throttle as fraction 0.0-1.0 (1.0 = full throttle)
     * 
     * @note Only affects vertical (Z-axis) thrust, not horizontal motion
     * @note Useful for limiting ascent rate in underwater vehicles
     */
    void set_max_throttle(float max_throttle) { _max_throttle = max_throttle; }

    /**
     * @brief Get rotational (angular) contribution factors for a specific motor
     * 
     * @details Returns a vector containing the motor's contribution to roll, pitch, and yaw
     *          control. These factors define how much torque this motor produces about each
     *          rotational axis when thrust is applied.
     * 
     *          Factor interpretation:
     *          - Positive roll factor: motor thrust causes right roll
     *          - Positive pitch factor: motor thrust causes nose-up pitch
     *          - Positive yaw factor: motor thrust causes clockwise yaw (viewed from above)
     * 
     * @param[in] motor_number Motor index (0 to AP_MOTORS_MAX_NUM_MOTORS-1)
     * 
     * @return Vector3f with components (roll_factor, pitch_factor, yaw_factor)
     * 
     * @note Factors are normalized based on motor position and thrust vector orientation
     * @note Used for control allocation and thrust balancing calculations
     * 
     * @warning Returns zero vector if motor_number is invalid or motor is not configured
     */
    Vector3f get_motor_angular_factors(int motor_number);

    /**
     * @brief Check if a specific motor is enabled and active
     * 
     * @details Determines if the specified motor is configured and actively participating
     *          in motor mixing calculations. Disabled motors are ignored during output.
     * 
     * @param[in] motor_number Motor index (0 to AP_MOTORS_MAX_NUM_MOTORS-1)
     * 
     * @return true if motor is enabled and configured, false otherwise
     * 
     * @note A motor may be disabled due to:
     *       - Not configured in current frame type
     *       - Hardware failure detected
     *       - Manually disabled via parameters
     */
    bool motor_is_enabled(int motor_number);

    /**
     * @brief Set motor direction reversal flag
     * 
     * @details Configures whether a motor's thrust direction should be reversed.
     *          Used to correct for motors installed backwards or to invert thrust direction
     *          for specific motor outputs without rewiring.
     * 
     *          When reversed:
     *          - Positive thrust command → negative (reverse) thrust output
     *          - Negative thrust command → positive (forward) thrust output
     * 
     * @param[in] motor_number Motor index (0 to AP_MOTORS_MAX_NUM_MOTORS-1)
     * @param[in] reversed true to reverse motor direction, false for normal direction
     * 
     * @return true if motor reversal was successfully set, false if motor_number invalid
     * 
     * @note Changes are saved to _motor_reverse parameter array
     * @note Commonly used during initial vehicle setup and testing
     * 
     * @warning Incorrect reversal settings cause vehicle control issues - test in safe environment
     */
    bool set_reversed(int motor_number, bool reversed);

    /**
     * @brief Parameter information table for AP_Param system
     * 
     * @details Defines all user-configurable parameters for 6DOF motor control:
     *          - Motor reversal flags (_motor_reverse): Direction reversal per motor
     *          - Forward-vertical coupling factor (_forwardVerticalCouplingFactor):
     *            Hydrodynamic compensation for thrust interaction
     * 
     * @note Parameters are stored in EEPROM and accessible via ground control station
     * @note See AP_Param documentation for parameter system details
     */
    static const struct AP_Param::GroupInfo        var_info[];

protected:
    /**
     * @brief Calculate maximum throttle based on battery current limiting
     * 
     * @details Computes a throttle scaling factor (0.0-1.0) to limit motor output when
     *          battery current approaches or exceeds safe limits. Prevents battery damage
     *          and voltage sag in high-current conditions common in underwater vehicles.
     * 
     *          Algorithm:
     *          - Monitors battery current draw
     *          - Applies gradual reduction when current exceeds threshold
     *          - Returns scaling factor applied to all motor outputs
     * 
     * @return Current limit factor from 0.0 (full limiting) to 1.0 (no limiting)
     * 
     * @note Override of AP_MotorsMatrix::get_current_limit_max_throttle()
     * @note Essential for battery protection in high-power underwater applications
     * @note Stored in _output_limited for use by output_to_motors()
     * 
     * @warning Aggressive current limiting may reduce control authority in critical situations
     */
    float               get_current_limit_max_throttle() override;

    /**
     * @brief Add a motor to the 6DOF mixing matrix with full axis factors
     * 
     * @details Configures a motor's contribution to all six control axes. Each factor defines
     *          how much the motor contributes to motion along that axis when thrust is applied.
     * 
     *          Factor conventions (positive values):
     *          - roll_fac: Contributes to right roll (positive = rolls right)
     *          - pitch_fac: Contributes to nose-up pitch (positive = pitches up)
     *          - yaw_fac: Contributes to clockwise yaw (positive = yaws right, viewed from above)
     *          - climb_fac: Contributes to upward motion (positive = ascends)
     *          - forward_fac: Contributes to forward motion (positive = moves forward)
     *          - lat_fac: Contributes to rightward motion (positive = moves right)
     * 
     * @param[in] motor_num Motor index (0 to AP_MOTORS_MAX_NUM_MOTORS-1)
     * @param[in] roll_fac Roll axis contribution factor
     * @param[in] pitch_fac Pitch axis contribution factor
     * @param[in] yaw_fac Yaw axis contribution factor
     * @param[in] climb_fac Vertical (Z-axis) contribution factor
     * @param[in] forward_fac Forward (X-axis) contribution factor
     * @param[in] lat_fac Lateral (Y-axis) contribution factor
     * @param[in] testing_order Motor test sequence order (for motor identification tests)
     * 
     * @note Override of AP_MotorsMatrix method, extended for 6DOF with forward/lateral axes
     * @note Factors are typically normalized but can be scaled for thrust asymmetry
     * @note Called during setup_motors() to configure frame geometry
     * 
     * @warning Incorrect factors cause unpredictable vehicle motion - verify carefully
     */
    void add_motor_raw_6dof(int8_t motor_num, float roll_fac, float pitch_fac, float yaw_fac, float climb_fac, float forward_fac, float lat_fac, uint8_t testing_order);

    /**
     * @brief Main motor mixing function - distributes 6-axis control to individual motors
     * 
     * @details Primary motor output method called when vehicle is armed and stabilizing.
     *          Selects appropriate mixing algorithm based on frame type:
     *          - Standard frames: Direct motor mixing
     *          - Vectored frames: Calls output_armed_stabilizing_vectored()
     *          - Full 6DOF frames: Calls output_armed_stabilizing_vectored_6dof()
     * 
     *          Mixing process:
     *          1. Read desired motion from attitude/position controllers (roll, pitch, yaw, throttle, forward, lateral)
     *          2. Multiply control inputs by motor factor matrix
     *          3. Calculate individual motor thrust commands
     *          4. Apply normalization to prevent motor saturation
     *          5. Apply current limiting if necessary
     *          6. Output to motors via output_to_motors()
     * 
     * @note Override of AP_MotorsMatrix::output_armed_stabilizing()
     * @note Called at main loop rate (typically 50-400Hz)
     * @note Applies forward-vertical coupling compensation for hydrodynamic effects
     * 
     * @warning Critical safety function - errors cause loss of vehicle control
     */
    void output_armed_stabilizing() override;

    /**
     * @brief Motor mixing for vectored thrust frames (horizontal + vertical thrusters)
     * 
     * @details Specialized mixing for frames with separate horizontal and vertical thrusters
     *          in fixed orientations. Applies thrust normalization to maintain control
     *          authority when motors approach saturation.
     * 
     *          Mixing strategy:
     *          - Horizontal thrusters: Handle forward, lateral, and yaw control
     *          - Vertical thrusters: Handle throttle (up/down), roll, and pitch control
     *          - Normalization ensures proportional control when thrust limits reached
     * 
     * @note Used for SUB_FRAME_VECTORED frame type
     * @note Maintains control authority during aggressive maneuvers
     */
    void output_armed_stabilizing_vectored();

    /**
     * @brief Motor mixing for full 6DOF omnidirectional frames
     * 
     * @details Advanced mixing for frames with angled thrusters providing true omnidirectional
     *          control. Each motor contributes to multiple axes simultaneously.
     * 
     *          Enhanced features:
     *          - Full 6-axis independent control
     *          - Advanced normalization preserving control ratios
     *          - Optimal thrust distribution for complex maneuvers
     *          - Handles highly coupled motor interactions
     * 
     * @note Used for SUB_FRAME_VECTORED_6DOF and SUB_FRAME_VECTORED_6DOF_90DEG frame types
     * @note Most computationally intensive mixing algorithm
     * @note Enables station-keeping and precise positioning in all axes
     */
    void output_armed_stabilizing_vectored_6dof();

    // ========== Parameters (stored in EEPROM, user-configurable) ==========

    /**
     * @brief Motor direction reversal flags (per motor)
     * 
     * @details Array of flags indicating if each motor's thrust direction should be reversed.
     *          Values: 0 = normal direction, 1 = reversed direction
     * 
     * @note Configurable via ground control station parameters (MOT_n_REVERSE)
     * @note Used to correct for motors installed backwards without rewiring
     */
    AP_Int8             _motor_reverse[AP_MOTORS_MAX_NUM_MOTORS];

    /**
     * @brief Forward-vertical thrust coupling compensation factor
     * 
     * @details Compensates for hydrodynamic coupling between forward and vertical thrust.
     *          When ROV moves forward, horizontal thrusters can create vertical force
     *          components due to hull geometry and water flow. This factor compensates
     *          by adjusting vertical thrust based on forward thrust demand.
     * 
     * @note Typical values: -0.5 to +0.5 (frame-dependent, determined experimentally)
     * @note Configurable via ground control station (MOT_FV_CPLNG_K parameter)
     * @note Essential for maintaining depth during horizontal translation
     */
    AP_Float            _forwardVerticalCouplingFactor;

    // ========== Motor Mixing Factors (configured during setup_motors) ==========

    /**
     * @brief Forward/backward contribution factor for each motor
     * 
     * @details Defines how much each motor contributes to forward (positive) or
     *          backward (negative) motion when thrust is applied.
     * 
     * @note Calculated during setup_motors() based on motor position and orientation
     * @note Range typically -1.0 to +1.0, but can be scaled for thrust asymmetry
     */
    float               _forward_factor[AP_MOTORS_MAX_NUM_MOTORS];

    /**
     * @brief Lateral (left/right) contribution factor for each motor
     * 
     * @details Defines how much each motor contributes to rightward (positive) or
     *          leftward (negative) motion when thrust is applied.
     * 
     * @note Calculated during setup_motors() based on motor position and orientation
     * @note Range typically -1.0 to +1.0, but can be scaled for thrust asymmetry
     */
    float               _lateral_factor[AP_MOTORS_MAX_NUM_MOTORS];

    // ========== Runtime State Variables ==========

    /**
     * @brief Maximum throttle limit (0.0 - 1.0)
     * 
     * @details User-configured limit on vertical thrust output. Used to restrict
     *          ascent rate or comply with power limitations.
     * 
     * @note Does not affect horizontal motion or rotational control
     * @note Set via set_max_throttle() method
     */
    float _max_throttle = 1.0f;

    /**
     * @brief Current limiting scale factor (0.0 - 1.0)
     * 
     * @details Calculated throttle reduction factor to limit battery current draw.
     *          Applied to all motor outputs when battery current exceeds safe limits.
     *          1.0 = no limiting, <1.0 = outputs scaled down to protect battery.
     * 
     * @note Updated by get_current_limit_max_throttle()
     * @note Critical for battery protection in high-power underwater operations
     */
    float _output_limited = 1.0f;

    /**
     * @brief Previous battery current reading (Amperes)
     * 
     * @details Stores last measured battery current for filtering and rate limiting
     *          calculations in current limiter algorithm.
     * 
     * @note Used to smooth current limit transitions
     * @note Updated each time get_current_limit_max_throttle() is called
     */
    float _batt_current_last = 0.0f;
};
