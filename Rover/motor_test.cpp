#include "Rover.h"

/**
 * @file motor_test.cpp
 * @brief Implementation of MAVLink motor test command (MAV_CMD_DO_MOTOR_TEST) for Rover
 * 
 * @details This module implements the MAVLink MAV_CMD_DO_MOTOR_TEST command, allowing
 *          ground control stations (GCS) and pilots to individually test motors, servos,
 *          and other actuators to verify proper wiring, rotation direction, and operation
 *          before field deployment.
 *          
 *          Motor testing is a critical safety procedure that allows validation of:
 *          - Correct motor/servo wiring and connections
 *          - Proper rotation direction for steering and throttle
 *          - ESC configuration and response
 *          - Actuator range of motion and calibration
 *          
 *          The system supports three test modes:
 *          1. Throttle Percentage (0-100%): Normalized output for relative testing
 *          2. PWM Direct (microseconds): Direct PWM output for precise servo/ESC testing
 *          3. Pilot Pass-through: Real-time RC input pass-through for manual testing
 *          
 * @warning Motor testing bypasses normal safety systems including arming checks and
 *          failsafes. Tests should ONLY be performed with vehicle safely restrained
 *          and clear of obstacles. Vehicle will move unexpectedly during testing.
 *          
 * @warning Motor tests temporarily disable critical failsafe systems:
 *          - GCS failsafe (fs_gcs_enabled)
 *          - Throttle failsafe (fs_throttle_enabled)  
 *          - Crash detection (fs_crash_check)
 *          These are automatically re-enabled when test completes or times out.
 *          
 * @note Motor test has a maximum duration of 30 seconds (MOTOR_TEST_TIMEOUT_MS_MAX)
 *       to prevent prolonged unsafe operation if test is not properly terminated.
 *       
 * @note The ESC calibration notify flag is set during testing to provide visual
 *       feedback that the vehicle is in a special test mode.
 * 
 * @see AP_MotorsUGV for motor output implementation
 * @see MAV_CMD_DO_MOTOR_TEST in MAVLink common.xml message set
 */

// Motor test configuration limits
/// Maximum PWM value accepted for direct PWM motor testing (microseconds)
/// Prevents excessively high PWM values that could damage ESCs or servos
static const int16_t MOTOR_TEST_PWM_MAX = 2200;

/// Maximum motor test duration in milliseconds (30 seconds)
/// Safety limit to prevent prolonged operation in test mode with disabled failsafes
static const int16_t MOTOR_TEST_TIMEOUT_MS_MAX = 30000;

// Motor test state variables (maintained between motor_test_output calls)
/// System time (milliseconds) when the current motor test sequence began
static uint32_t motor_test_start_ms = 0;

/// Test duration in milliseconds - test will automatically stop after this period
/// Calculated from requested timeout, capped at MOTOR_TEST_TIMEOUT_MS_MAX
static uint32_t motor_test_timeout_ms = 0;

/// Motor/servo instance being tested (steering, throttle left, throttle right)
/// @see AP_MotorsUGV::motor_test_order for valid motor instance identifiers
static AP_MotorsUGV::motor_test_order motor_test_instance;

/// Motor test throttle type - determines how motor_test_throttle_value is interpreted
/// 0 = MOTOR_TEST_THROTTLE_PERCENT: Value is percentage (0-100)
/// 1 = MOTOR_TEST_THROTTLE_PWM: Value is direct PWM output in microseconds
/// 2 = MOTOR_TEST_THROTTLE_PILOT: Value ignored, uses live pilot RC input
static uint8_t motor_test_throttle_type = 0;

/// Throttle/servo value to output during test
/// Interpretation depends on motor_test_throttle_type:
/// - Percentage mode: 0-100 representing output percentage
/// - PWM mode: 800-2200 microseconds for direct servo/ESC control
/// - Pilot mode: unused, pilot RC input controls output directly
static int16_t motor_test_throttle_value = 0;

/**
 * @brief Motor test output handler - manages ongoing motor test execution and timeout
 * 
 * @details This function is called repeatedly from the main loop during an active motor test
 *          to output the requested test signal to the selected motor/servo. It handles:
 *          - Timeout detection and automatic test termination
 *          - Mode-specific output generation (percentage, PWM, or pilot pass-through)
 *          - Error detection and safe test termination on output failure
 *          
 *          The function maintains motor test state between calls and ensures tests
 *          cannot run indefinitely by enforcing the configured timeout period.
 *          
 *          **Test Mode Handling:**
 *          
 *          1. **Throttle Percentage Mode (MOTOR_TEST_THROTTLE_PERCENT)**:
 *             - Outputs normalized percentage (0-100) to selected motor
 *             - Useful for relative testing and comparing motor responses
 *             - Motor mixing and scaling applied by AP_MotorsUGV
 *          
 *          2. **PWM Direct Mode (MOTOR_TEST_THROTTLE_PWM)**:
 *             - Outputs direct PWM signal in microseconds to motor/servo
 *             - Bypasses normal scaling for precise ESC/servo testing
 *             - Typical range: 1000-2000μs, max allowed: 2200μs
 *             - Critical for verifying exact servo endpoints and ESC calibration
 *          
 *          3. **Pilot Pass-through Mode (MOTOR_TEST_THROTTLE_PILOT)**:
 *             - Routes live pilot RC input directly to selected motor/servo
 *             - Steering test: Uses normalized steering input with deadzone
 *             - Throttle test: Uses throttle channel control input
 *             - Allows manual real-time testing with RC transmitter
 *          
 * @warning This function executes with FAILSAFES DISABLED. The following safety
 *          systems are temporarily deactivated during motor testing:
 *          - GCS communication loss failsafe
 *          - Throttle loss failsafe  
 *          - Crash detection
 *          Vehicle must be safely restrained during all motor tests.
 *          
 * @warning Motor test bypasses normal arming checks. Motors can spin even if
 *          pre-arm checks would normally fail. Only use in controlled test environment.
 *          
 * @note Called at main loop rate (typically 50Hz) when motor_test flag is active
 * @note Automatically stops test on timeout or if motor output fails
 * @note ESC calibration notify LED active during test for visual feedback
 * 
 * @see motor_test_stop() for test termination and failsafe re-enable
 * @see mavlink_motor_test_start() for test initialization
 * @see AP_MotorsUGV::output_test_pct() for percentage mode output
 * @see AP_MotorsUGV::output_test_pwm() for direct PWM mode output
 */
void Rover::motor_test_output()
{
    // Exit immediately if motor test is not active
    // The motor_test flag is set by mavlink_motor_test_start() and cleared by motor_test_stop()
    if (!motor_test) {
        return;
    }

    // Check for test timeout - enforces maximum test duration safety limit
    // Timeout calculated as: (current_time - start_time) >= requested_duration
    // Maximum duration capped at MOTOR_TEST_TIMEOUT_MS_MAX (30 seconds)
    if ((AP_HAL::millis() - motor_test_start_ms) >= motor_test_timeout_ms) {
        // Timeout reached - stop test and re-enable failsafes
        motor_test_stop();
    } else {
        // Test still active - generate appropriate motor output based on test mode
        bool test_result = false;
        
        // Select output method based on throttle type requested by MAVLink command
        // Each mode provides different testing capabilities for various scenarios
        switch (motor_test_throttle_type) {
            case MOTOR_TEST_THROTTLE_PERCENT:
                // Percentage mode: Output normalized 0-100% value to motor
                // Uses standard motor mixing and scaling - good for relative comparisons
                test_result = g2.motors.output_test_pct(motor_test_instance, motor_test_throttle_value);
                break;

            case MOTOR_TEST_THROTTLE_PWM:
                // PWM mode: Output direct PWM value in microseconds to motor/servo
                // Bypasses normal scaling - precise control for ESC calibration and servo endpoints
                // Value range typically 1000-2000μs, validated against MOTOR_TEST_PWM_MAX (2200μs)
                test_result = g2.motors.output_test_pwm(motor_test_instance, motor_test_throttle_value);
                break;

            case MOTOR_TEST_THROTTLE_PILOT:
                // Pilot pass-through mode: Route live RC input directly to selected motor
                // Allows manual real-time testing with transmitter sticks
                if (motor_test_instance == AP_MotorsUGV::MOTOR_TEST_STEERING) {
                    // Steering test: Use steering channel with deadzone applied
                    // norm_input_dz() returns -1.0 to +1.0, convert to percentage -100 to +100
                    test_result = g2.motors.output_test_pct(motor_test_instance, channel_steer->norm_input_dz() * 100.0f);
                } else {
                    // Throttle test: Use throttle channel control input (already scaled)
                    // get_control_in() returns percentage value suitable for motor output
                    test_result = g2.motors.output_test_pct(motor_test_instance, channel_throttle->get_control_in());
                }
                break;

            default:
                // Invalid throttle type - should never occur if mavlink_motor_test_check() validated input
                // Exit without output to prevent undefined behavior
                return;
        }
        
        // Check if motor output failed (could indicate hardware issue or invalid motor instance)
        // Stop test immediately on failure to prevent continued operation in unknown state
        if (!test_result) {
            motor_test_stop();
        }
    }
}

/**
 * @brief Perform safety validation checks before motor test can begin
 * 
 * @details Validates vehicle state and test parameters to ensure motor testing can
 *          proceed safely. This function enforces multiple safety checks to prevent
 *          motor tests from running in unsafe conditions that could damage the vehicle
 *          or cause injury.
 *          
 *          Safety checks performed:
 *          1. Board initialization complete (sensors, calibration loaded)
 *          2. RC calibration valid (if using percentage/pilot modes)
 *          3. Safety switch pushed (if equipped - hardware interlock)
 *          4. Motor instance valid for vehicle configuration
 *          5. Throttle type is supported (percentage, PWM, or pilot)
 *          6. Throttle value within safe limits for selected type
 *          
 *          RC calibration check can be bypassed for direct PWM mode since raw
 *          microsecond values don't depend on RC calibration scaling.
 * 
 * @param[in] gcs_chan     MAVLink channel for sending error messages to GCS
 * @param[in] check_rc     true to enforce RC calibration check, false to skip
 *                         (false for PWM mode which doesn't need RC calibration)
 * @param[in] motor_seq    Motor instance to test (steering, throttle_left, throttle_right)
 * @param[in] throttle_type Test mode (0=percentage, 1=PWM, 2=pilot pass-through)
 * @param[in] throttle_value Output value to test (interpretation depends on throttle_type)
 * 
 * @return true if all safety checks pass and test can proceed
 * @return false if any check fails - error message sent to GCS with failure reason
 * 
 * @warning This function does NOT prevent motor movement - it only validates pre-conditions.
 *          Vehicle must still be physically restrained before calling motor test functions.
 * 
 * @note Maximum PWM value limited to MOTOR_TEST_PWM_MAX (2200μs) to protect ESCs
 * @note Maximum percentage value limited to 100% to prevent invalid scaling
 * @note Called by mavlink_motor_test_start() before initiating test sequence
 * 
 * @see mavlink_motor_test_start() for test initialization after checks pass
 * @see AP_MotorsUGV::motor_test_order for valid motor instance values
 */
bool Rover::mavlink_motor_test_check(const GCS_MAVLINK &gcs_chan, bool check_rc, AP_MotorsUGV::motor_test_order motor_seq, uint8_t throttle_type, int16_t throttle_value)
{
    // check board has initialised
    if (!initialised) {
        gcs_chan.send_text(MAV_SEVERITY_CRITICAL, "Motor Test: Board initialising");
        return false;
    }

    // check rc has been calibrated
    if (check_rc && !arming.rc_calibration_checks(true)) {
        gcs_chan.send_text(MAV_SEVERITY_CRITICAL, "Motor Test: RC not calibrated");
        return false;
    }

    // check if safety switch has been pushed
    if (hal.util->safety_switch_state() == AP_HAL::Util::SAFETY_DISARMED) {
        gcs_chan.send_text(MAV_SEVERITY_CRITICAL, "Motor Test: Safety switch");
        return false;
    }

    // check motor_seq
    if (motor_seq > AP_MotorsUGV::MOTOR_TEST_THROTTLE_RIGHT) {
        gcs_chan.send_text(MAV_SEVERITY_CRITICAL, "Motor Test: invalid motor (%d)", (int)motor_seq);
        return false;
    }

    // check throttle type
    if (throttle_type > MOTOR_TEST_THROTTLE_PILOT) {
        gcs_chan.send_text(MAV_SEVERITY_CRITICAL, "Motor Test: invalid throttle type: %d", (int)throttle_type);
        return false;
    }

    // check throttle value
    if (throttle_type == MOTOR_TEST_THROTTLE_PWM && throttle_value > MOTOR_TEST_PWM_MAX) {
        gcs_chan.send_text(MAV_SEVERITY_CRITICAL, "Motor Test: pwm (%d) too high", (int)throttle_value);
        return false;
    }
    if (throttle_type == MOTOR_TEST_THROTTLE_PERCENT && throttle_value > 100) {
        gcs_chan.send_text(MAV_SEVERITY_CRITICAL, "Motor Test: percentage (%d) too high", (int)throttle_value);
        return false;
    }

    // if we got this far the check was successful and the motor test can continue
    return true;
}

/**
 * @brief Start motor test sequence - initialize test mode and output to specified motor
 * 
 * @details Initiates a motor test sequence in response to MAV_CMD_DO_MOTOR_TEST MAVLink command.
 *          This function orchestrates the motor test startup procedure:
 *          
 *          **Initialization Sequence:**
 *          1. Perform safety validation checks (if test not already running)
 *          2. Arm vehicle using special MOTORTEST arming method (bypasses pre-arm checks)
 *          3. Temporarily disable failsafe systems for test duration
 *          4. Configure test parameters (motor, throttle type, value, timeout)
 *          5. Activate visual notification (ESC calibration LED pattern)
 *          
 *          **Failsafe Suppression:**
 *          During motor testing, the following failsafes are temporarily disabled:
 *          - GCS failsafe (fs_gcs_enabled = 0): Prevents RTL if GCS link drops
 *          - Throttle failsafe (fs_throttle_enabled = 0): Prevents failsafe if RC lost
 *          - Crash detection (fs_crash_check = 0): Prevents false crash detection
 *          
 *          These are automatically restored when test completes or times out.
 *          
 *          **Arming Bypass:**
 *          Motor test uses AP_Arming::Method::MOTORTEST which bypasses normal pre-arm
 *          safety checks. This allows testing motors even if GPS, compass, or other
 *          sensors are not ready. Vehicle MUST be physically restrained.
 *          
 *          If a test is already running, this function updates the test parameters
 *          (motor instance, throttle value, timeout) without re-initializing.
 * 
 * @param[in] gcs_chan       MAVLink channel for sending status messages
 * @param[in] motor_instance Motor to test (steering, throttle_left, throttle_right)
 * @param[in] throttle_type  Test mode: 0=percentage (0-100), 1=PWM (μs), 2=pilot pass-through
 * @param[in] throttle_value Output value (interpretation depends on throttle_type)
 * @param[in] timeout_sec    Test duration in seconds (capped at 30 seconds max)
 * 
 * @return MAV_RESULT_ACCEPTED if test started successfully or parameters updated
 * @return MAV_RESULT_FAILED if safety checks fail or arming fails
 * 
 * @warning THIS FUNCTION DISABLES CRITICAL FAILSAFE SYSTEMS. Motor test should ONLY
 *          be performed with vehicle safely restrained on a test bench or with wheels
 *          off the ground. Vehicle will move unexpectedly and failsafes will not protect it.
 *          
 * @warning Motor test bypasses arming checks including GPS lock, compass calibration,
 *          RC calibration, and pre-arm safety checks. This is intentional to allow
 *          hardware testing before full system is operational, but requires extreme caution.
 *          
 * @warning Timeout is capped at MOTOR_TEST_TIMEOUT_MS_MAX (30 seconds) to prevent
 *          indefinite operation with disabled failsafes if test stop command is not received.
 * 
 * @note Visual feedback provided via ESC calibration LED pattern during test
 * @note Test automatically terminates on timeout via motor_test_output()
 * @note Failsafes restored and vehicle disarmed when test completes
 * 
 * @see mavlink_motor_test_check() for safety validation performed before test start
 * @see motor_test_output() for ongoing test execution and timeout handling  
 * @see motor_test_stop() for test termination and failsafe restoration
 * @see AP_Arming::Method::MOTORTEST for special motor test arming mode
 */
MAV_RESULT Rover::mavlink_motor_test_start(const GCS_MAVLINK &gcs_chan, AP_MotorsUGV::motor_test_order motor_instance, uint8_t throttle_type, int16_t throttle_value, float timeout_sec)
{
    // if test has not started try to start it
    if (!motor_test) {
        /* perform checks that it is ok to start test
           The RC calibrated check can be skipped if direct pwm is
           suppliedo
        */
        if (!mavlink_motor_test_check(gcs_chan, throttle_type != 1, motor_instance, throttle_type, throttle_value)) {
            return MAV_RESULT_FAILED;
        } else {
            // start test
            motor_test = true;

            // arm motors
            if (!arming.is_armed()) {
                if (!arming.arm(AP_Arming::Method::MOTORTEST)) {
                    return MAV_RESULT_FAILED;
                }
            }

            // disable failsafes
            g.fs_gcs_enabled.set(0);
            g.fs_throttle_enabled.set(0);
            g.fs_crash_check.set(0);

            // turn on notify leds
            AP_Notify::flags.esc_calibration = true;
        }
    }

    // set timeout
    motor_test_start_ms = AP_HAL::millis();
    motor_test_timeout_ms = MIN(timeout_sec * 1000, MOTOR_TEST_TIMEOUT_MS_MAX);

    // store required output
    motor_test_instance = motor_instance;
    motor_test_throttle_type = throttle_type;
    motor_test_throttle_value = throttle_value;

    // return success
    return MAV_RESULT_ACCEPTED;
}

/**
 * @brief Stop motor test and restore normal vehicle safety systems
 * 
 * @details Terminates an active motor test sequence and restores the vehicle to normal
 *          operational state. This function performs the complete shutdown procedure:
 *          
 *          **Shutdown Sequence:**
 *          1. Disarm vehicle using MOTORTEST disarm method (stops motor output)
 *          2. Clear test timing state (timeout and start time)
 *          3. Re-enable failsafe systems that were disabled during test
 *          4. Deactivate visual notification (ESC calibration LED)
 *          5. Clear motor test active flag
 *          
 *          **Failsafe Restoration:**
 *          All failsafe systems disabled during test are restored to their configured
 *          values by reloading from parameter storage:
 *          - GCS failsafe (fs_gcs_enabled): Restored from FS_GCS_ENABL parameter
 *          - Throttle failsafe (fs_throttle_enabled): Restored from FS_THR_ENABLE parameter
 *          - Crash detection (fs_crash_check): Restored from FS_CRASH_CHECK parameter
 *          
 *          This ensures vehicle returns to user-configured safety state after test.
 *          
 *          **Automatic Invocation:**
 *          This function is automatically called by motor_test_output() when:
 *          - Test timeout period expires
 *          - Motor output fails (returns false)
 *          - Manual stop command received via MAVLink
 *          
 *          **Safe Idempotence:**
 *          Multiple calls to this function are safe - it checks motor_test flag and
 *          returns immediately if test is not active. This prevents issues with
 *          redundant stop commands.
 * 
 * @warning After this function returns, vehicle is DISARMED but failsafes are active.
 *          Normal arming procedures and pre-arm checks will be required before flight.
 *          
 * @note Called automatically on test timeout (30 second maximum)
 * @note Called automatically if motor output fails during test
 * @note Can be called explicitly via MAVLink command to abort test early
 * @note ESC calibration LED turns off to indicate test completion
 * 
 * @see motor_test_output() for timeout detection and automatic stop
 * @see mavlink_motor_test_start() for test initialization and failsafe disable
 * @see AP_Arming::Method::MOTORTEST for special motor test disarm method
 */
void Rover::motor_test_stop()
{
    // Exit immediately if motor test is not active
    // Safe to call multiple times - prevents redundant disarm/parameter operations
    if (!motor_test) {
        return;
    }

    // Disarm motors using MOTORTEST disarm method
    // This stops all motor output immediately and safely
    AP::arming().disarm(AP_Arming::Method::MOTORTEST);

    // Clear test timing state - prepares for potential next test
    motor_test_start_ms = 0;
    motor_test_timeout_ms = 0;

    // Re-enable failsafe systems by reloading from parameter storage
    // This restores user-configured failsafe settings that were disabled for testing
    g.fs_gcs_enabled.load();        // Restore GCS communication loss failsafe
    g.fs_throttle_enabled.load();   // Restore throttle/RC loss failsafe
    g.fs_crash_check.load();        // Restore crash detection failsafe

    // Deactivate ESC calibration LED notification
    // Returns LED patterns to normal operation mode
    AP_Notify::flags.esc_calibration = false;

    // Clear motor test active flag - allows new test to be started
    motor_test = false;
}
