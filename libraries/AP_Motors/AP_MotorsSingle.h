/**
 * @file AP_MotorsSingle.h
 * @brief Motor and Servo control class for single/coaxial motor configurations with control surfaces
 * 
 * @details This file implements motor control for unconventional multirotor configurations that use
 *          a single motor (or coaxial counter-rotating motor pair) for thrust combined with four
 *          airplane-style control surfaces (flaps/elevons) for attitude control.
 * 
 *          Configuration:
 *          - One or two motors (coaxial counter-rotating) provide vertical thrust only
 *          - Four servo-controlled surfaces provide pitch, roll, and yaw moments
 *          - Mixing algorithm separates thrust (motors) from attitude control (surfaces)
 * 
 *          Use Cases:
 *          - Simple VTOL platforms with airplane-like control
 *          - Educational multirotor platforms
 *          - Low-cost multirotor designs with minimal motor count
 *          - Experimental hybrid aircraft configurations
 * 
 *          Design Trade-offs:
 *          - Advantages: Simplified motor configuration, reduced cost, efficient forward flight
 *          - Disadvantages: No motor redundancy, limited attitude authority compared to multi-motor designs
 * 
 * @note This is a specialized configuration not suitable for most multirotor applications
 * @warning Single point of failure for thrust generation - motor failure results in immediate loss of lift
 */
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include "AP_MotorsMulticopter.h"

/**
 * @defgroup SingleMotorConstants Configuration constants for single/coaxial motor frames
 * @{
 */

/**
 * @brief Positive feedback direction for control surface mixing
 * @note Used to define servo deflection direction in mixing calculations
 */
#define AP_MOTORS_SING_POSITIVE      1

/**
 * @brief Negative feedback direction for control surface mixing
 * @note Used to reverse servo deflection direction in mixing calculations
 */
#define AP_MOTORS_SING_NEGATIVE     -1

/**
 * @brief Number of control surface actuators
 * 
 * @details Defines the number of control surfaces used for attitude control.
 *          Four surfaces provide independent control of roll, pitch, and yaw moments.
 * 
 * @note Fixed at 4 for standard single/coaxial configurations
 * @note Each actuator is a servo-controlled surface (flap, elevon, etc.)
 */
#define NUM_ACTUATORS 4

/**
 * @brief Default update rate for digital servos in Hertz
 * 
 * @details Digital servos can handle higher update rates, providing better response and
 *          control precision for attitude control surfaces.
 * 
 * @note Units: Hertz (updates per second)
 * @warning Do not use this rate with analog servos - may cause damage or excessive current draw
 */
#define AP_MOTORS_SINGLE_SPEED_DIGITAL_SERVOS 250

/**
 * @brief Default update rate for analog servos in Hertz
 * 
 * @details Analog servos are typically limited to lower update rates. This conservative
 *          rate ensures compatibility with most analog servo types.
 * 
 * @note Units: Hertz (updates per second)
 * @note Safe for all analog servo types
 */
#define AP_MOTORS_SINGLE_SPEED_ANALOG_SERVOS 125

/**
 * @brief Control surface servo input range in centidegrees
 * 
 * @details Defines the maximum control input range for servo deflection. An input of
 *          -4500 centidegrees (-45°) moves servo to minimum endpoint (radio_min), while
 *          +4500 centidegrees (+45°) moves servo to maximum endpoint (radio_max).
 * 
 *          This range provides:
 *          - Full authority over control surface deflection
 *          - Symmetric positive and negative deflection
 *          - Compatibility with standard servo endpoint configuration
 * 
 * @note Units: centidegrees (1 degree = 100 centidegrees)
 * @note Range: ±4500 centidegrees = ±45 degrees
 * @warning Ensure servo endpoints are configured to prevent mechanical binding at limits
 */
#define AP_MOTORS_SINGLE_SERVO_INPUT_RANGE      4500

/** @} */ // end of SingleMotorConstants group

/**
 * @class AP_MotorsSingle
 * @brief Motor control implementation for single/coaxial thrust with four control surfaces
 * 
 * @details This class implements motor mixing and output for multirotor configurations that use
 *          one or two motors for thrust combined with airplane-style control surfaces for attitude.
 * 
 *          Motor Configuration:
 *          - Single motor: One motor provides all vertical thrust
 *          - Coaxial: Two counter-rotating motors share thrust and cancel torque
 * 
 *          Control Surface Configuration:
 *          - Four servo-controlled surfaces (flaps, elevons, or similar)
 *          - Surfaces deflect to generate pitch, roll, and yaw moments
 *          - Surface deflection range: ±4500 centidegrees (±45 degrees)
 * 
 *          Mixing Strategy:
 *          - Thrust component: Directed entirely to motor(s)
 *          - Roll/Pitch/Yaw: Directed to control surface deflections
 *          - Similar to airplane control mixing but for VTOL multirotor flight
 * 
 *          Attitude Authority:
 *          - Limited compared to multi-motor designs (no differential thrust)
 *          - Dependent on airspeed for aerodynamic effectiveness
 *          - Requires properly configured and calibrated control surfaces
 * 
 * @note NUM_ACTUATORS = 4 for the four control surfaces
 * @note Motor(s) provide only thrust; surfaces provide all attitude moments
 * @note Can use single motor or coaxial counter-rotating pair for yaw control
 * @note Servo ranges: ±4500 centidegrees for full deflection
 * 
 * @warning Requires properly configured control surface throws to avoid saturation
 * @warning Limited attitude authority compared to multi-motor multirotor designs
 * @warning Motor failure results in complete loss of thrust with no redundancy
 * @warning Control surface failure can cause loss of control authority in affected axis
 * @warning Not suitable for aggressive multirotor flight or high wind conditions
 * 
 * @see AP_MotorsMulticopter Base class for multirotor motor control
 * @see AP_MotorsMatrix Standard multirotor motor mixing
 */
class AP_MotorsSingle : public AP_MotorsMulticopter {
public:

    /// Constructor
    AP_MotorsSingle(uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT) :
        AP_MotorsMulticopter(speed_hz)
    {
    };

    /**
     * @brief Initialize motor and servo outputs for single/coaxial configuration
     * 
     * @details Initializes the motor output channel(s) and four servo output channels for control
     *          surfaces. Sets up output ranges, default parameters, and frame-specific configuration.
     *          Must be called during vehicle setup before motors can be used.
     * 
     *          Initialization sequence:
     *          1. Configure motor output channel(s) for thrust
     *          2. Configure four servo channels for control surfaces
     *          3. Set servo output ranges (±4500 centidegrees)
     *          4. Apply frame-specific mixing parameters
     *          5. Initialize safety features and output limits
     * 
     * @param[in] frame_class Motor frame class (should be MOTOR_FRAME_SINGLE)
     * @param[in] frame_type Motor frame type variant (single motor vs coaxial)
     * 
     * @note Called once during vehicle initialization
     * @note Must complete successfully before output_to_motors() can be used
     * @warning Failure to initialize properly can result in incorrect control surface mixing
     * 
     * @see set_frame_class_and_type()
     * @see AP_MotorsMulticopter::init()
     */
    void                init(motor_frame_class frame_class, motor_frame_type frame_type) override;

    /**
     * @brief Set frame class and type for single/coaxial motor configuration
     * 
     * @details Updates the frame configuration, allowing dynamic reconfiguration of motor and
     *          control surface mixing. For single motor configurations, this determines whether
     *          to use a single motor or coaxial counter-rotating pair.
     * 
     *          Frame type variations:
     *          - Single motor: One motor for thrust, yaw from control surfaces only
     *          - Coaxial: Two counter-rotating motors, yaw from differential motor speed
     * 
     * @param[in] frame_class Motor frame class (should be MOTOR_FRAME_SINGLE)
     * @param[in] frame_type Motor frame type variant (single vs coaxial configuration)
     * 
     * @note Can be called after initialization to reconfigure frame
     * @note Changes take effect on next output_to_motors() call
     * @warning Changing frame type during flight is not recommended
     * 
     * @see init()
     */
    void set_frame_class_and_type(motor_frame_class frame_class, motor_frame_type frame_type) override;

    /**
     * @brief Set output update rate for motors and servos
     * 
     * @details Configures the PWM update frequency for motor ESC(s) and control surface servos.
     *          Different update rates are appropriate for different servo types and applications.
     * 
     *          Typical update rates:
     *          - Digital servos: 250 Hz (AP_MOTORS_SINGLE_SPEED_DIGITAL_SERVOS)
     *          - Analog servos: 125 Hz (AP_MOTORS_SINGLE_SPEED_ANALOG_SERVOS)
     *          - Motors: 50-490 Hz depending on ESC type
     * 
     *          Trade-offs:
     *          - Higher rates: Better servo response, more CPU overhead, higher current draw
     *          - Lower rates: Adequate for analog servos, reduced system load
     * 
     * @param[in] speed_hz Update rate in Hertz (typical range: 50-490 Hz)
     * 
     * @note Update rate applies to both motor and servo outputs
     * @note Digital servos can handle higher rates (up to 250 Hz)
     * @note Analog servos typically limited to 50-125 Hz
     * @warning Excessive update rates can damage analog servos or cause servo jitter
     * @warning Update rate must be set before outputs are enabled
     * 
     * @see AP_MOTORS_SINGLE_SPEED_DIGITAL_SERVOS
     * @see AP_MOTORS_SINGLE_SPEED_ANALOG_SERVOS
     */
    void                set_update_rate( uint16_t speed_hz ) override;

    /**
     * @brief Output thrust commands to motor(s) and deflection commands to control surfaces
     * 
     * @details Sends calculated PWM signals to motor ESC(s) for thrust and to servos for control
     *          surface deflections. This is the final stage of the control pipeline that converts
     *          mixing outputs to hardware commands.
     * 
     *          Output sequence:
     *          1. Apply output limits and safety constraints
     *          2. Scale thrust commands to motor PWM range
     *          3. Scale control surface deflections to servo PWM range (±4500 centidegrees)
     *          4. Send PWM signals to motor and servo output channels
     *          5. Update telemetry and logging
     * 
     *          When disarmed or in failsafe:
     *          - Motors output minimum throttle (typically motor stop)
     *          - Control surfaces may be centered or hold last position depending on configuration
     * 
     * @note Called at main loop rate (typically 400 Hz for multirotors)
     * @note Motor output is thrust only; all attitude control via surfaces
     * @note Control surface range: ±4500 centidegrees (±45 degrees)
     * @warning Must be called regularly to maintain control; gaps cause servo jitter
     * @warning Ensure proper servo endpoint configuration to prevent binding or damage
     * 
     * @see output_armed_stabilizing()
     * @see get_motor_mask()
     */
    virtual void        output_to_motors() override;

    /**
     * @brief Get bitmask of output channels used for motors and control surface servos
     * 
     * @details Returns a bitmask indicating which PWM output channels are allocated to this motor
     *          controller. Bit position corresponds to output channel number (bit 0 = channel 1).
     *          A bit value of 1 indicates the channel is in use.
     * 
     *          For single/coaxial configurations:
     *          - 1-2 bits set for motor channel(s) (single motor = 1 bit, coaxial = 2 bits)
     *          - 4 bits set for control surface servo channels
     *          - Total: 5-6 output channels allocated
     * 
     *          Usage:
     *          This mask prevents conflicts when other vehicle systems (camera gimbals, grippers,
     *          auxiliary servos) attempt to use PWM outputs. The vehicle code checks this mask
     *          before assigning additional servo functions.
     * 
     * @return Bitmask of output channels in use (bit N = 1 means channel N+1 is used)
     * 
     * @note Bitmask is zero-indexed (bit 0 = output channel 1)
     * @note Mask includes both motor and control surface servo outputs
     * @note Mask is constant after initialization for a given frame configuration
     * 
     * @see SRV_Channel Servo output channel management
     */
    uint32_t            get_motor_mask() override;

    /**
     * @brief Run pre-arm safety checks for motor and servo configuration
     * 
     * @details Performs safety validation before allowing vehicle to arm. Inherits checks from
     *          base AP_Motors class which validates motor configuration, output ranges, and
     *          safety features. For single/coaxial configurations, this is particularly important
     *          due to lack of motor redundancy.
     * 
     * @param[in] buflen Size of error message buffer
     * @param[out] buffer Buffer to store error messages if checks fail
     * 
     * @return true if all checks pass and vehicle is safe to arm, false otherwise
     * 
     * @note Called by arming logic before allowing vehicle to arm
     * @warning Single/coaxial configurations have no thrust redundancy - extra vigilance required
     * 
     * @see AP_Motors::arming_checks()
     */
    bool arming_checks(size_t buflen, char *buffer) const override { return AP_Motors::arming_checks(buflen, buffer); }

protected:
    /**
     * @brief Output mixing for armed stabilizing flight
     * 
     * @details Implements the core mixing algorithm that separates thrust and attitude control.
     *          Called when vehicle is armed and stabilization is active.
     * 
     *          Mixing algorithm:
     *          1. Extract thrust command → send to motor(s)
     *          2. Extract roll command → mix to control surfaces (aileron-like)
     *          3. Extract pitch command → mix to control surfaces (elevator-like)
     *          4. Extract yaw command → mix to control surfaces (rudder-like) or differential motor speed
     *          5. Limit surface deflections to ±4500 centidegrees
     *          6. Apply outputs via output_to_motors()
     * 
     *          Control mixing:
     *          - Thrust: 100% to motor(s), 0% to surfaces
     *          - Roll/Pitch/Yaw: 0% to motor thrust, 100% to surface deflections
     *          - Similar to airplane control but for vertical flight regime
     * 
     * @note Called at main loop rate during armed stabilizing flight
     * @note Motor(s) provide only vertical thrust; surfaces provide all moments
     * @warning Limited attitude authority compared to multi-motor multirotors
     * @warning Control surface saturation can occur during aggressive maneuvers
     * 
     * @see output_to_motors()
     */
    void                output_armed_stabilizing() override;

    /**
     * @brief Get human-readable frame type string for logging and display
     * 
     * @return String identifier "SINGLE" for telemetry and log messages
     * 
     * @note Used in log files and ground station display
     */
    const char* _get_frame_string() const override { return "SINGLE"; }

    /**
     * @brief Output test sequence for motor and servo testing
     * 
     * @details Outputs a specified PWM value to a specific motor or servo for ground testing.
     *          Used during pre-flight checks, motor ordering verification, and hardware validation.
     *          For single/coaxial configs, this tests both thrust motor(s) and control surfaces.
     * 
     *          Test sequence numbering:
     *          - motor_seq 1-2: Thrust motor(s) (1 for single, 1-2 for coaxial)
     *          - motor_seq 3-6: Control surface servos (4 surfaces)
     * 
     * @param[in] motor_seq Motor/servo sequence number (1 to total number of outputs)
     * @param[in] pwm PWM value to output in microseconds (typically 1000-2000 µs)
     * 
     * @note Used by motor test function in ground control station
     * @note Bypasses normal mixing - outputs raw PWM directly
     * @warning Only use on ground with propellers removed or proper safety measures
     * @warning Verify correct motor/servo wiring before running test sequence
     * @warning Test with low PWM values first to prevent unexpected movement
     * 
     * @see AP_Motors::output_test()
     */
    virtual void        _output_test_seq(uint8_t motor_seq, int16_t pwm) override;

    /**
     * @brief Total throttle PWM output value
     * 
     * @details Final calculated PWM value for motor throttle output, summed onto throttle channel
     *          minimum. Typically ranges from ~1100 µs (motor stop/idle) to ~1900 µs (full throttle).
     * 
     * @note Units: microseconds (PWM pulse width)
     * @note Range: Approximately 1000-2000 µs depending on ESC configuration
     */
    int16_t             _throttle_radio_output;
    
    /**
     * @brief Combined actuator outputs for control surfaces
     * 
     * @details Array of four control surface deflection outputs after mixing roll, pitch, and yaw
     *          commands. Each value represents desired surface position in normalized range.
     * 
     * @note Array size: NUM_ACTUATORS (4 control surfaces)
     * @note Range: 0.0 to 1.0 normalized output (converted to ±4500 centidegrees for servos)
     * @note Index mapping depends on frame configuration and surface layout
     */
    float               _actuator_out[NUM_ACTUATORS];
    
    /**
     * @brief Normalized thrust output command
     * 
     * @details Thrust component extracted from attitude controller, sent to motor(s).
     *          Represents desired vertical thrust in normalized range.
     * 
     * @note Range: 0.0 (motor stop) to 1.0 (full thrust)
     * @note Does not include attitude components (roll/pitch/yaw)
     * @note Converted to PWM via _throttle_radio_output
     */
    float               _thrust_out;
};
