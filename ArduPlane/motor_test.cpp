/**
 * @file motor_test.cpp
 * @brief QuadPlane motor test functionality for MAV_CMD_DO_MOTOR_TEST
 * 
 * @details This file implements the MAVLink motor test command (MAV_CMD_DO_MOTOR_TEST)
 *          which allows pilots to test individual QuadPlane motors to verify proper
 *          wiring, rotation direction, and motor response. The motor test functionality
 *          enables safe ground testing of VTOL motors before flight operations.
 *          
 *          Key features:
 *          - Individual motor testing by sequence number
 *          - Multiple throttle control modes (percent, PWM, pilot input)
 *          - Configurable test duration and timeout protection
 *          - Sequential multi-motor testing capability
 *          - Safety checks requiring disarmed state before test initiation
 *          - Visual notification via ESC calibration LEDs
 *          
 *          The motor test runs in a time-limited mode with automatic termination
 *          to prevent unintended prolonged motor operation. All tests require the
 *          vehicle to be disarmed initially and perform comprehensive safety checks
 *          before enabling motor output.
 *          
 * @note This functionality is only compiled when HAL_QUADPLANE_ENABLED is defined.
 *       It is specific to QuadPlane configurations and not applicable to pure
 *       fixed-wing aircraft.
 * 
 * @warning Motor tests should only be performed with propellers removed or with
 *          the vehicle securely restrained. Unexpected motor activation can cause
 *          injury or damage.
 * 
 * @see QuadPlane::motor_test_output()
 * @see QuadPlane::mavlink_motor_test_start()
 * @see QuadPlane::motor_test_stop()
 * 
 * Source: ArduPlane/motor_test.cpp
 */

#include "Plane.h"

/**
 * @def MOTOR_TEST_TIMEOUT_MS_MAX
 * @brief Maximum allowed duration for a single motor test in milliseconds
 * 
 * @details This defines the upper limit for motor test duration to prevent
 *          prolonged motor operation that could cause overheating or battery
 *          drain. Any test duration requested beyond this limit will be
 *          clamped to this maximum value.
 */
#define MOTOR_TEST_TIMEOUT_MS_MAX       30000   // max timeout is 30 seconds

/**
 * @brief Main motor test execution function called periodically during active tests
 * 
 * @details This function manages the ongoing motor test operation, handling timeout
 *          checks, throttle value calculation, and motor output commands. It is called
 *          repeatedly during the scheduler loop while a motor test is active.
 *          
 *          Execution sequence:
 *          1. Check if motor test is running (exit early if not)
 *          2. Verify test timeout hasn't been exceeded
 *          3. Handle multi-motor sequential testing with inter-motor delays
 *          4. Calculate PWM output based on throttle type (percent, PWM, or pilot)
 *          5. Validate PWM values are within safe limits
 *          6. Send output command to motors subsystem
 *          
 *          Throttle type handling:
 *          - MOTOR_TEST_THROTTLE_PERCENT: Scales throttle_value (0-100%) to PWM range
 *          - MOTOR_TEST_THROTTLE_PWM: Uses throttle_value directly as PWM microseconds
 *          - MOTOR_TEST_THROTTLE_PILOT: Uses current pilot throttle stick input
 *          
 *          Multi-motor testing:
 *          When motor_count > 1, the test automatically sequences through motors
 *          with a 0.5 second zero-output delay between motors to clearly distinguish
 *          individual motor responses.
 * 
 * @note This function is called at the main scheduler loop rate (typically 50Hz)
 *       and must execute quickly to avoid timing jitter.
 * 
 * @note PWM values are constrained to RC_Channel::RC_MIN_LIMIT_PWM and
 *       RC_Channel::RC_MAX_LIMIT_PWM to prevent invalid servo outputs.
 * 
 * @warning This function temporarily enables motor output while the vehicle is
 *          technically disarmed. Ensure proper safety precautions are in place.
 * 
 * @see QuadPlane::mavlink_motor_test_start()
 * @see QuadPlane::motor_test_stop()
 * @see AP_Motors::output_test_seq()
 */
#if HAL_QUADPLANE_ENABLED
void QuadPlane::motor_test_output()
{
    // exit immediately if the motor test is not running
    if (!motor_test.running) {
        return;
    }

    // check for test timeout
    uint32_t now = AP_HAL::millis();
    if ((now - motor_test.start_ms) >= motor_test.timeout_ms) {
        if (motor_test.motor_count > 1) {
            if (now - motor_test.start_ms < motor_test.timeout_ms*1.5) {
                // output zero for 0.5s
                motors->output_min();
            } else {
                // move onto next motor
                motor_test.seq++;
                motor_test.motor_count--;
                motor_test.start_ms = now;
            }
            return;
        }
        // stop motor test
        motor_test_stop();
        return;
    }
            
    int16_t pwm = 0;   // pwm that will be output to the motors

    // calculate pwm based on throttle type
    const int16_t thr_min_pwm = motors->get_pwm_output_min();
    const int16_t thr_max_pwm = motors->get_pwm_output_max();

    switch (motor_test.throttle_type) {
    case MOTOR_TEST_THROTTLE_PERCENT:
        // sanity check motor_test.throttle value
        if (motor_test.throttle_value <= 100) {
            pwm = thr_min_pwm + (thr_max_pwm - thr_min_pwm) * (float)motor_test.throttle_value*0.01f;
        }
        break;

    case MOTOR_TEST_THROTTLE_PWM:
        pwm = motor_test.throttle_value;
        break;

    case MOTOR_TEST_THROTTLE_PILOT:
        pwm = thr_min_pwm + (thr_max_pwm - thr_min_pwm) * plane.get_throttle_input()*0.01f;
        break;

    default:
        motor_test_stop();
        return;
    }

    // sanity check throttle values
    if (pwm < RC_Channel::RC_MIN_LIMIT_PWM || pwm > RC_Channel::RC_MAX_LIMIT_PWM) {
        motor_test_stop();
        return;
    }

    // turn on motor to specified pwm value
    if (!motors->output_test_seq(motor_test.seq, pwm)) {
        gcs().send_text(MAV_SEVERITY_INFO, "Motor Test: cancelled");
        motor_test_stop();
    }
}

/**
 * @brief Start a motor test sequence via MAVLink MAV_CMD_DO_MOTOR_TEST command
 * 
 * @details This function initiates a motor test by validating safety conditions,
 *          configuring test parameters, and enabling motor output. It implements
 *          the MAVLink MAV_CMD_DO_MOTOR_TEST command for QuadPlane configurations.
 *          
 *          Safety check sequence:
 *          1. Verify QuadPlane is available and motors subsystem initialized
 *          2. Confirm vehicle is disarmed (required for initial test start)
 *          3. Execute motor subsystem safety checks via motors->motor_test_checks()
 *          4. Only after all checks pass, enable motor output and arm motors
 *          
 *          Test initialization:
 *          - Arms the motors subsystem temporarily for test duration
 *          - Enables ESC calibration notification LEDs for visual indication
 *          - Stores test parameters (sequence, throttle type/value, timeout)
 *          - Supports sequential multi-motor testing (up to 8 motors)
 *          
 *          Subsequent calls during an active test:
 *          - Allowed to modify test parameters without requiring disarm
 *          - Resets timeout to allow extended testing sessions
 *          - Updates throttle values and motor sequence
 * 
 * @param[in] chan           MAVLink channel for response messages
 * @param[in] motor_seq      Motor sequence number to test (0-based index)
 * @param[in] throttle_type  Throttle interpretation mode:
 *                           - MOTOR_TEST_THROTTLE_PERCENT: throttle_value is 0-100%
 *                           - MOTOR_TEST_THROTTLE_PWM: throttle_value is PWM μs
 *                           - MOTOR_TEST_THROTTLE_PILOT: use pilot throttle input
 * @param[in] throttle_value Throttle setting (interpretation per throttle_type)
 * @param[in] timeout_sec    Test duration in seconds (clamped to MOTOR_TEST_TIMEOUT_MS_MAX)
 * @param[in] motor_count    Number of motors to test sequentially (1-8, for multi-motor tests)
 * 
 * @return MAV_RESULT_ACCEPTED if test started successfully
 * @return MAV_RESULT_FAILED if safety checks fail or QuadPlane unavailable
 * 
 * @note The vehicle must be disarmed before the initial motor test can start.
 *       This is a critical safety requirement to prevent unintended motor activation.
 * 
 * @note During a test, the motors are temporarily armed but the vehicle remains
 *       in a disarmed state from the perspective of flight control.
 * 
 * @warning Propellers should be removed before conducting motor tests. The test
 *          will generate actual motor thrust and rotation.
 * 
 * @warning Motor test will fail if:
 *          - Vehicle is armed (not in motor test mode)
 *          - QuadPlane not configured or motors subsystem unavailable
 *          - Motor subsystem safety checks fail (battery voltage, etc.)
 * 
 * @see QuadPlane::motor_test_output()
 * @see QuadPlane::motor_test_stop()
 * @see AP_Motors::motor_test_checks()
 */
MAV_RESULT QuadPlane::mavlink_motor_test_start(mavlink_channel_t chan, uint8_t motor_seq, uint8_t throttle_type,
                                            uint16_t throttle_value, float timeout_sec, uint8_t motor_count)
{
    if (!available() || motors == nullptr) {
        return MAV_RESULT_FAILED;
    }

    // Must be disarmed to start a motor test but allow changes during a test
    if (!motor_test.running && motors->armed()) {
        gcs().send_text(MAV_SEVERITY_INFO, "Must be disarmed for motor test");
        return MAV_RESULT_FAILED;
    }

    // Check Motor test is allowed
    char failure_msg[50] {};
    if (!motors->motor_test_checks(ARRAY_SIZE(failure_msg), failure_msg)) {
        gcs().send_text(MAV_SEVERITY_CRITICAL,"Motor Test: %s", failure_msg);
        return MAV_RESULT_FAILED;
    }

    // if test has not started try to start it
    if (!motor_test.running) {
        // start test
        motor_test.running = true;

        // enable and arm motors
        set_armed(true);
        
        // turn on notify leds
        AP_Notify::flags.esc_calibration = true;
    }

    // set timeout
    motor_test.start_ms = AP_HAL::millis();
    motor_test.timeout_ms = MIN(timeout_sec * 1000, MOTOR_TEST_TIMEOUT_MS_MAX);

    // store required output
    motor_test.seq = motor_seq;
    motor_test.throttle_type = throttle_type;
    motor_test.throttle_value = throttle_value;
    motor_test.motor_count = MIN(motor_count, 8);

    // return success
    return MAV_RESULT_ACCEPTED;
}

/**
 * @brief Stop the currently running motor test and restore safe state
 * 
 * @details This function terminates an active motor test, disarms the motors,
 *          and resets all motor test state variables to their safe defaults.
 *          It is called automatically when a test timeout expires, when an
 *          error condition is detected, or when explicitly commanded to stop.
 *          
 *          Shutdown sequence:
 *          1. Verify a test is actually running (exit early if not)
 *          2. Set motor_test.running flag to false to halt output
 *          3. Disarm motors subsystem via set_armed(false)
 *          4. Clear timeout tracking variables
 *          5. Disable ESC calibration notification LEDs
 *          
 *          This function ensures the vehicle returns to a safe, fully disarmed
 *          state after motor testing is complete. All motors will stop receiving
 *          output commands and return to their unpowered state.
 * 
 * @note This function is safe to call multiple times or when no test is running.
 *       It will simply return early if motor_test.running is already false.
 * 
 * @note The disarm operation via set_armed(false) ensures proper cleanup of
 *       motor output channels and safety interlocks.
 * 
 * @see QuadPlane::motor_test_output()
 * @see QuadPlane::mavlink_motor_test_start()
 * @see QuadPlane::set_armed()
 */
void QuadPlane::motor_test_stop()
{
    // exit immediately if the test is not running
    if (!motor_test.running) {
        return;
    }

    // flag test is complete
    motor_test.running = false;

    // disarm motors
    set_armed(false);

    // reset timeout
    motor_test.start_ms = 0;
    motor_test.timeout_ms = 0;

    // turn off notify leds
    AP_Notify::flags.esc_calibration = false;
}

/**
 * @note Conditional Compilation: All motor test functionality in this file is
 *       conditionally compiled based on HAL_QUADPLANE_ENABLED. When this define
 *       is not set (pure fixed-wing configurations), this entire file compiles
 *       to an empty object file, adding no code size overhead. This is because
 *       motor testing is only applicable to QuadPlane VTOL configurations that
 *       have multirotor motors requiring individual testing.
 */
#endif  // HAL_QUADPLANE_ENABLED
