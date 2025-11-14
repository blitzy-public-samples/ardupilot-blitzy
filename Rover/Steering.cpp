/**
 * @file Steering.cpp
 * @brief Implementation of rover steering control outputting steering and throttle commands to motors
 * 
 * @details This file implements the main servo output function for Rover vehicles,
 *          which sends steering and throttle commands to the AR_Motors library.
 *          The function serves as the interface between high-level control algorithms
 *          and low-level motor/servo outputs.
 *          
 *          The steering control outputs body-frame commands where:
 *          - Steering: Body-frame lateral control (positive = turn right)
 *          - Throttle: Body-frame longitudinal control (positive = forward)
 *          
 *          Integration with AR_Motors (g2.motors):
 *          - Handles steering angle to servo PWM conversion
 *          - Performs throttle scaling and limiting based on vehicle configuration
 *          - Manages different motor configurations (skid-steering, Ackermann, etc.)
 *          - Applies safety limits and arming checks
 * 
 * Source: Rover/Steering.cpp
 */

#include "Rover.h"

/**
 * @brief Set the steering and throttle servo outputs based on current calculated control values
 * 
 * @details This function outputs the final steering and throttle commands to the motors.
 *          It serves as the main interface between the attitude controller and motor outputs.
 *          
 *          The function operates in two modes:
 *          1. Motor test mode: Outputs test signals via motor_test_output()
 *          2. Normal operation: Outputs steering/throttle via g2.motors.output()
 *          
 *          In normal operation:
 *          - Retrieves current ground speed from attitude controller
 *          - Passes arming state, speed, and time delta to motor output function
 *          - g2.motors.output() handles:
 *            * Conversion of steering angle to servo PWM values
 *            * Throttle scaling based on speed and configured limits
 *            * Motor mixing for different vehicle configurations
 *            * Safety limiting and arming interlocks
 *          
 *          Coordinate Frame: All commands are in body frame
 *          - Forward speed: Positive = forward motion
 *          - Steering: Controlled via g2.attitude_control, applied in body frame
 * 
 * @note This function integrates with the AR_Motors library (g2.motors) which
 *       performs the actual motor mixing, PWM conversion, and hardware output.
 *       See libraries/AR_Motors for detailed motor control implementation.
 * 
 * @note Called at the main loop rate (typically 50Hz for rovers)
 * 
 * @warning This is a safety-critical function. The motor output must respect
 *          arming state to prevent unexpected vehicle motion.
 * 
 * @see AR_Motors::output() for detailed motor output implementation
 * @see Rover::motor_test_output() for motor testing functionality
 * @see AC_AttitudeControl_Rover::get_forward_speed() for speed retrieval
 */
void Rover::set_servos(void)
{
    // Check if motor test mode is active (commanded via MAVLink or ground station)
    if (motor_test) {
        // Motor test mode: Output test signals to individual motors/servos
        // Used for pre-flight checks and motor configuration validation
        motor_test_output();
    } else {
        // Normal operation mode: Output steering and throttle based on control algorithms
        
        // Get current forward ground speed from attitude controller (m/s)
        // This is used by the motor library for speed-dependent throttle scaling
        // and to implement features like pivot turns at low speeds
        float speed = 0.0f;
        g2.attitude_control.get_forward_speed(speed);

        // Output steering and throttle commands to motors via AR_Motors library
        // 
        // Parameters passed to g2.motors.output():
        // - arming.is_armed(): Safety interlock - motors only move when armed
        // - speed: Current forward speed (m/s) for speed-dependent control
        // - G_Dt: Time delta since last update (seconds) for rate calculations
        //
        // The g2.motors.output() function performs:
        // 1. Retrieves desired steering and throttle from attitude controller
        // 2. Converts steering angle (radians) to servo PWM values (typically 1000-2000 μs)
        // 3. Applies throttle scaling based on speed, configured limits, and arming state
        // 4. Performs motor mixing for vehicle type (skid-steer, Ackermann, omni, etc.)
        // 5. Applies slew rate limiting for smooth acceleration/deceleration
        // 6. Outputs final PWM values to SRV_Channel for hardware output
        //
        // Motor output respects:
        // - MOT_THR_MIN/MAX: Throttle output limits
        // - MOT_STR_THR_*: Steering-to-throttle mixing parameters
        // - Vehicle configuration: Number of motors, steering type, etc.
        // - Arming state: No motor movement when disarmed (safety)
        //
        // @note The actual PWM conversion and hardware output is handled by
        //       the SRV_Channel library, which manages servo output channels
        g2.motors.output(arming.is_armed(), speed, G_Dt);
    }
}
