/**
 * @file AP_MotorsTri.h
 * @brief Motor control class for Tricopters with tilting tail servo for yaw control
 * 
 * @details This file implements motor mixing and control for tricopter Y-frame configurations.
 *          Tricopters use three motors arranged in a Y pattern with a pivoting tail motor
 *          that provides yaw control through thrust vectoring rather than motor speed differential.
 * 
 *          Configuration:
 *          - Two front motors (left and right) provide thrust and roll/pitch control
 *          - One rear motor with attached tilt servo provides thrust and yaw control
 *          - Tail servo angle determines yaw thrust vector direction
 * 
 *          Advantages:
 *          - Simpler mechanical design than quadcopters
 *          - More efficient than single-rotor helicopters with tail rotors
 *          - Direct yaw control without relying on motor torque differential
 * 
 * Source: libraries/AP_Motors/AP_MotorsTri.h
 */
#pragma once

#include "AP_Motors_config.h"

#if AP_MOTORS_TRI_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include "AP_MotorsMulticopter.h"

/**
 * @def AP_MOTORS_CH_TRI_YAW
 * @brief Output channel assignment for tricopter tail servo
 * 
 * @details The tail servo for yaw control is assigned to channel 7 (CH_7).
 *          This channel controls the tail motor tilt mechanism that provides
 *          yaw authority through thrust vectoring.
 * 
 * @note This is a fixed assignment for tricopter configurations
 * @note Ensure no other servos or outputs conflict with channel 7
 */
#define AP_MOTORS_CH_TRI_YAW    CH_7

/**
 * @def AP_MOTORS_TRI_SERVO_RANGE_DEG_MIN
 * @brief Minimum allowable tail servo angular range
 * 
 * @details Minimum servo deflection angle in degrees for tail servo configuration.
 *          Values below this minimum provide insufficient yaw authority.
 * 
 * Units: degrees
 * Value: 5°
 * 
 * @note If YAW_SV_ANGLE parameter is set below this value, it will be limited to this minimum
 */
#define AP_MOTORS_TRI_SERVO_RANGE_DEG_MIN   5

/**
 * @def AP_MOTORS_TRI_SERVO_RANGE_DEG_MAX
 * @brief Maximum allowable tail servo angular range
 * 
 * @details Maximum servo deflection angle in degrees for tail servo configuration.
 *          Values above this maximum may cause mechanical binding or excessive
 *          servo strain. This limit prevents damage to tail servo mechanism.
 * 
 * Units: degrees
 * Value: 80°
 * 
 * @note If YAW_SV_ANGLE parameter exceeds this value, it will be limited to this maximum
 * @warning Exceeding mechanical servo limits can cause binding and loss of yaw control
 */
#define AP_MOTORS_TRI_SERVO_RANGE_DEG_MAX   80

/**
 * @class AP_MotorsTri
 * @brief Motor control for tricopter Y-frame with pivoting tail motor
 * 
 * @details This class implements motor mixing and control algorithms for tricopter configurations.
 *          
 *          Motor Configuration:
 *          - Motor 1: Front left motor (standard multicopter motor)
 *          - Motor 2: Front right motor (standard multicopter motor)
 *          - Motor 3: Rear motor with attached tilt servo for yaw control
 *          
 *          Yaw Control Mechanism:
 *          The tail motor is mounted on a servo-controlled pivot that can tilt the thrust
 *          vector left or right (typically ±45° or configurable). This vectored thrust provides
 *          yaw authority without requiring motor differential torque like quadcopters.
 *          
 *          Control Allocation:
 *          - Roll: Differential thrust between front left and front right motors
 *          - Pitch: Front motors vs rear motor differential thrust
 *          - Yaw: Tail servo angle tilts rear motor thrust vector
 *          - Throttle: Collective thrust from all three motors
 *          
 *          Efficiency Benefits:
 *          - Simpler than quadcopters (one less motor and ESC)
 *          - More efficient than single-rotor helicopters (no tail rotor thrust loss)
 *          - Direct yaw control provides excellent yaw authority
 *          
 *          Special Features:
 *          - Supports pitch-reverse mode for inverted flight capability
 *          - Configurable tail servo angle limits via parameters
 *          - Reversible servo direction for different mechanical mounting orientations
 * 
 * @note Tail servo typically uses channel 7 (CH_7) for output
 * @note Tail servo angle range is configurable, typically ±45° but can be adjusted via YAW_SV_ANGLE parameter
 * @note Can reverse motor direction for inverted thrust in pitch-reverse mode
 * 
 * @warning Tail servo binding or mechanical failure causes complete loss of yaw control
 * @warning Tail servo must have adequate throw and speed for responsive yaw control
 * @warning Tail motor failure results in loss of both yaw and pitch authority
 * @warning Ensure tail servo is mechanically constrained to safe range to prevent binding
 * @warning Incorrect servo direction (YAW_SV_REV) causes reversed yaw response - verify direction before flight
 */
class AP_MotorsTri : public AP_MotorsMulticopter {
public:

    /// Constructor
    AP_MotorsTri(uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT) :
        AP_MotorsMulticopter(speed_hz)
    {
    };

    /**
     * @brief Initialize tricopter motor and servo outputs
     * 
     * @details Initializes three motor outputs and the tail servo output channel.
     *          Sets up motor-to-output mapping and configures the tail servo on CH_7.
     *          Establishes motor mixing parameters based on frame configuration.
     * 
     * @param[in] frame_class Motor frame class (should be MOTOR_FRAME_TRI for tricopters)
     * @param[in] frame_type Motor frame type variant (currently only standard Y-frame supported)
     * 
     * @note Must be called before any motor output operations
     * @note Tail servo is automatically assigned to channel 7 (AP_MOTORS_CH_TRI_YAW)
     */
    void                init(motor_frame_class frame_class, motor_frame_type frame_type) override;

    /**
     * @brief Set frame class and type for tricopter configuration
     * 
     * @details Updates the frame class and type, reconfiguring motor mixing if needed.
     *          For tricopters, frame_class should be MOTOR_FRAME_TRI. Frame type
     *          determines specific Y-frame variant configuration.
     * 
     * @param[in] frame_class Motor frame class (MOTOR_FRAME_TRI for tricopters)
     * @param[in] frame_type Motor frame type variant (standard Y-frame configuration)
     * 
     * @note Changing frame class/type at runtime will recalculate motor mixing parameters
     */
    void set_frame_class_and_type(motor_frame_class frame_class, motor_frame_type frame_type) override;

    /**
     * @brief Set update rate for motor and servo outputs
     * 
     * @details Sets the PWM update frequency for the three motor outputs and the tail
     *          servo output. Higher rates provide better control response but may not
     *          be supported by all ESCs. The tail servo typically uses standard servo
     *          rates (50-490 Hz).
     * 
     * @param[in] speed_hz Update rate in Hertz (typical values: 50, 400, 490)
     * 
     * @note Motor update rate should match ESC capabilities (typically 400-490 Hz for multicopter ESCs)
     * @note Tail servo update rate is usually lower than motor rate (50-490 Hz depending on servo type)
     * @warning Setting update rate too high for ESC capabilities may cause erratic motor behavior
     */
    void                set_update_rate( uint16_t speed_hz ) override;

    /**
     * @brief Output calculated motor speeds and tail servo angle to hardware
     * 
     * @details Sends the computed motor throttle values and tail servo angle to the
     *          physical motor outputs and servo output. This is the final stage of the
     *          control pipeline, converting desired commands to PWM outputs.
     * 
     * @note Called by the main output loop after motor mixing calculations
     * @note Tail servo angle output is combined with motor thrust outputs in this call
     * @note Respects arming state - outputs minimum values when disarmed
     */
    virtual void        output_to_motors() override;

    /**
     * @brief Get bitmask of outputs used for motors and tail servo
     * 
     * @details Returns a bitmask indicating which PWM output channels are being used
     *          by the tricopter motors and tail servo. Bit positions correspond to
     *          output channel numbers. This prevents conflicts with auxiliary servo outputs.
     * 
     * @return Bitmask where bit N set to 1 means output channel N is used (motors 1-3 and CH_7 for tail servo)
     * 
     * @note For tricopter: bits 0-2 for motors, bit 6 for tail servo (CH_7)
     * @note Used by servo output manager to avoid PWM channel conflicts
     */
    uint32_t            get_motor_mask() override;

    /**
     * @brief Output thrust to motors matching a bitmask with differential thrust for yaw
     * 
     * @details Outputs a specified thrust level to motors selected by the bitmask.
     *          Used for tiltrotor configurations in forward flight mode where motors
     *          provide forward thrust. Differential thrust (rudder_dt) provides yaw control.
     * 
     * @param[in] thrust Base thrust level for selected motors (0.0 to 1.0)
     * @param[in] mask Bitmask selecting which motors to control (bit N = motor N)
     * @param[in] rudder_dt Differential thrust for yaw control (0.0 to 1.0, applied asymmetrically)
     * 
     * @note This method is primarily used for hybrid VTOL aircraft configurations
     * @note For standard tricopter flight, use normal mixing via output_armed_stabilizing()
     */
    void                output_motor_mask(float thrust, uint32_t mask, float rudder_dt) override;

    /**
     * @brief Get roll control contribution factor for a specific motor
     * 
     * @details Returns the roll factor for motor i, indicating how much that motor
     *          contributes to roll control. Used for tiltrotor and tailsitter aircraft
     *          that use multicopter motors for forward flight control.
     * 
     * @param[in] i Motor index (0 = front left, 1 = front right, 2 = rear)
     * 
     * @return Roll factor in range -1.0 to 1.0 (negative = rolls left, positive = rolls right, 0 = no roll contribution)
     * 
     * @note Front left motor: negative roll factor (rolls left when increased)
     * @note Front right motor: positive roll factor (rolls right when increased)
     * @note Rear motor: zero roll factor (does not contribute to roll)
     */
    float               get_roll_factor(uint8_t i) override;

    /**
     * @brief Get pitch control contribution factor for a specific motor
     * 
     * @details Returns the pitch factor for motor i, indicating how much that motor
     *          contributes to pitch control. Used by AP_Motors_test for motor testing
     *          and JSON output of motor configuration.
     * 
     * @param[in] i Motor index (0 = front left, 1 = front right, 2 = rear)
     * 
     * @return Pitch factor in range -1.0 to 1.0 (negative = nose down, positive = nose up, 0 = no pitch contribution)
     * 
     * @note Front motors: positive pitch factor (nose up when increased)
     * @note Rear motor: negative pitch factor (nose down when increased, or positive if pitch-reversed)
     */
    float               get_pitch_factor_json(uint8_t i);

    /**
     * @brief Run pre-arm safety checks specific to tricopter configuration
     * 
     * @details Performs tricopter-specific pre-arm safety checks including tail servo
     *          configuration validation, servo range checks, and motor output verification.
     *          Checks ensure tail servo is properly configured and responsive before flight.
     * 
     * @param[in] buflen Length of the error message buffer
     * @param[out] buffer Buffer to store error message if checks fail
     * 
     * @return true if all arming checks pass, false if any check fails (error message in buffer)
     * 
     * @note Validates tail servo angle range is within safe limits (MIN to MAX degrees)
     * @note Verifies tail servo output channel is properly assigned
     * @warning Failed arming checks indicate unsafe flight conditions - do not override without addressing the issue
     */
    bool arming_checks(size_t buflen, char *buffer) const override;

    /**
     * @brief Get motor testing sequence order
     * 
     * @details Returns the testing order sequence for motors, used by AP_Motors_test
     *          to test motors in a logical sequence (typically: rear, front-right, front-left).
     * 
     * @param[in] i Test sequence index (0, 1, or 2)
     * 
     * @return Motor number to test at sequence position i
     * 
     * @note Testing order helps identify motor positions during setup and calibration
     * @note Typical sequence: 2 (rear), 1 (front-right), 0 (front-left)
     */
    uint8_t get_motor_test_order(uint8_t i);

protected:
    /**
     * @brief Compute motor mixing and tail servo angle for armed stabilizing flight
     * 
     * @details Core motor mixing algorithm for tricopter. Calculates individual motor
     *          thrust values from pilot inputs (roll, pitch, yaw, throttle) and tail
     *          servo angle for yaw control. Implements the tricopter mixing matrix:
     *          - Front left/right motors: throttle + pitch ± roll
     *          - Rear motor: throttle - pitch
     *          - Tail servo: yaw input scaled to servo angle limits
     * 
     * @note Called at main loop rate (typically 400 Hz) when armed and stabilizing
     * @note Tail servo provides yaw authority without motor differential torque
     * @note Respects motor and servo output limits to prevent saturation
     * 
     * @warning This is a safety-critical function - any modifications must be thoroughly tested
     */
    void                output_armed_stabilizing() override;

    /**
     * @brief Apply vehicle-supplied thrust compensation corrections
     * 
     * @details Calls the vehicle-specific thrust compensation callback if configured.
     *          Thrust compensation adjusts motor outputs based on battery voltage,
     *          altitude, or other factors to maintain consistent thrust characteristics.
     * 
     * @note Compensation helps maintain consistent flight characteristics as battery voltage drops
     * @note Vehicle can register custom compensation function via callback
     */
    void                thrust_compensation(void) override;

    const char* _get_frame_string() const override { return "TRI"; }
    const char*  get_type_string() const override { return _pitch_reversed ? "pitch-reversed" : ""; }

    /**
     * @brief Output test PWM signal to a specific motor in test sequence
     * 
     * @details Outputs a specified PWM value to a motor identified by test sequence number.
     *          Used during motor testing to verify motor operation, direction, and position.
     *          Bypasses normal motor mixing and directly outputs raw PWM values.
     * 
     * @param[in] motor_seq Motor sequence number in test order (1 to 3)
     * @param[in] pwm PWM value in microseconds (typically 1000-2000 μs)
     * 
     * @note motor_seq is 1-indexed (1, 2, 3) not 0-indexed
     * @note Test sequence order is: 1=rear, 2=front-right, 3=front-left (or as defined by get_motor_test_order)
     * @note Standard PWM range: 1000 μs = minimum, 2000 μs = maximum
     * 
     * @warning Only call this function during controlled motor testing - not during normal flight
     * @warning Ensure propellers are removed or vehicle is secured before motor testing
     */
    virtual void _output_test_seq(uint8_t motor_seq, int16_t pwm) override;

    /**
     * @name Tricopter Configuration Parameters
     * @{
     */
    
    /**
     * @brief Current tail servo pivot angle for yaw control
     * 
     * @details Calculated tail servo deflection angle in degrees. This angle determines
     *          the direction of the rear motor's thrust vector for yaw control.
     *          Positive values tilt right, negative values tilt left.
     * 
     * Units: degrees
     * Range: -YAW_SV_ANGLE to +YAW_SV_ANGLE (typically ±45° or ±80° max)
     * 
     * @note Updated every control loop based on pilot yaw input
     * @note Can be reversed via YAW_SV_REV parameter if servo mechanically mounted reversed
     */
    float           _pivot_angle;
    
    /**
     * @brief Calculated thrust for front right motor
     * 
     * @details Thrust value for the front right motor after mixing calculations.
     *          Combines throttle, pitch, and roll inputs.
     * 
     * Units: 0.0 to 1.0 (0 = minimum thrust, 1 = maximum thrust)
     */
    float           _thrust_right;
    
    /**
     * @brief Calculated thrust for rear motor
     * 
     * @details Thrust value for the rear motor after mixing calculations.
     *          Combines throttle and pitch inputs. Roll and yaw do not affect
     *          rear motor thrust (yaw is controlled by tail servo angle).
     * 
     * Units: 0.0 to 1.0 (0 = minimum thrust, 1 = maximum thrust)
     */
    float           _thrust_rear;
    
    /**
     * @brief Calculated thrust for front left motor
     * 
     * @details Thrust value for the front left motor after mixing calculations.
     *          Combines throttle, pitch, and roll inputs.
     * 
     * Units: 0.0 to 1.0 (0 = minimum thrust, 1 = maximum thrust)
     */
    float           _thrust_left;

    /**
     * @brief Pitch-reverse mode flag for inverted flight capability
     * 
     * @details When true, pitch control is reversed to support inverted (upside-down)
     *          flight. Motors spin in reverse and pitch inputs are inverted.
     *          Enables aerobatic maneuvers and inverted hover.
     * 
     * @note Configured via PITCH_REV_EXPO parameter
     * @note Requires motors capable of bidirectional rotation
     * 
     * @warning Switching pitch-reverse mode in flight requires careful pilot technique
     */
    bool _pitch_reversed;
    
    /**
     * @brief Tail servo availability flag
     * 
     * @details Indicates whether a tail servo is configured and available for yaw control.
     *          If false, tricopter cannot control yaw (unsafe flight condition).
     * 
     * @warning If _have_tail_servo is false, vehicle should not arm
     * @warning Loss of tail servo during flight results in loss of yaw control
     */
    bool _have_tail_servo;
    
    /** @} */ // end of Tricopter Configuration Parameters
};

#endif  // AP_MOTORS_TRI_ENABLED
