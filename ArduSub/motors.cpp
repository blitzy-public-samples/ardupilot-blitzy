/**
 * @file motors.cpp
 * @brief Motor and thruster control functions for ArduSub ROV/submarine
 * 
 * @details This file implements motor/thruster control for underwater vehicles with 6-DOF
 *          (6 Degrees of Freedom) control capability. ArduSub uses vectored thrust from
 *          multiple thrusters to achieve independent control of:
 *          - Translation: forward/back, left/right, up/down
 *          - Rotation: roll, pitch, yaw
 *          
 *          The motor library (AP_Motors6DOF) performs motor mixing, which transforms
 *          desired forces and moments into individual thruster outputs using a motor
 *          matrix. The mixing algorithm accounts for thruster positions, orientations,
 *          and frame-specific configurations (vectored, BlueROV2, etc.).
 *          
 *          Key Functions:
 *          - Motor output and initialization
 *          - Motor testing and calibration
 *          - Translation of navigation outputs to thruster commands
 *          - 6-DOF control: roll, pitch, yaw, throttle, forward, lateral
 *          
 * @note Motor/thruster calibration should be performed before flight using the
 *       motor test feature. ESCs must be configured for bidirectional operation
 *       in underwater applications.
 *       
 * @warning Incorrect thruster configuration or motor matrix can result in loss of
 *          vehicle control. Always verify thruster directions in a controlled
 *          environment before deployment.
 * 
 * @see AP_Motors6DOF for motor mixing implementation
 * @see mode.cpp for flight mode usage of motor control
 */
#include "Sub.h"

/**
 * @brief Enable motor output and set all motors to minimum output values
 * 
 * @details Enables the motor subsystem and commands all motors/thrusters to their
 *          minimum PWM values (typically 1500μs for bidirectional ESCs in neutral).
 *          This is used during initialization and when entering armed state to ensure
 *          motors start from a known safe state.
 *          
 *          Called during:
 *          - Vehicle arming sequence
 *          - Motor test initialization
 *          - Failsafe recovery
 * 
 * @note Minimum output values are defined in AP_Motors6DOF and depend on ESC
 *       configuration. For bidirectional ESCs, minimum typically means neutral/stopped.
 * 
 * @see motors_output() for main motor output function
 * @see init_motor_test() for motor test usage
 */
void Sub::enable_motor_output()
{
    motors.output_min();
}

/**
 * @brief Main motor output function - sends commands to motors library for ESC output
 * 
 * @details This function is called at the main loop rate (typically 50-400 Hz) to update
 *          motor/thruster outputs. It coordinates the motor mixing and servo output pipeline:
 *          
 *          1. Check for special modes (MOTOR_DETECT, motor test)
 *          2. Enable motor interlock (safety mechanism)
 *          3. Cork servo outputs (batch updates for efficiency)
 *          4. Calculate PWM values from desired forces/moments
 *          5. Output to all servo channels
 *          6. Perform motor mixing and output
 *          7. Push corked outputs (send batched commands to hardware)
 *          
 *          Motor Mixing Process:
 *          The motors library (AP_Motors6DOF) transforms 6-DOF control inputs
 *          (roll, pitch, yaw, throttle, forward, lateral) into individual thruster
 *          PWM outputs using the motor matrix. The matrix accounts for:
 *          - Thruster position relative to vehicle center of mass
 *          - Thruster orientation (angle and direction)
 *          - Vehicle frame type (BlueROV2, vectored, custom, etc.)
 *          
 * @note Motor interlock must be enabled for motors to produce thrust. This is a
 *       safety feature that prevents motor output during disarmed or unsafe conditions.
 *       
 * @note The cork/push mechanism batches servo outputs to minimize communication
 *       overhead with hardware (especially important for CAN/DroneCAN ESCs).
 *       
 * @warning This function must be called regularly to maintain vehicle control.
 *          Missing calls will result in command timeouts and failsafe activation.
 *          
 * @warning In MOTOR_DETECT mode, this function returns without updating motors,
 *          allowing direct thruster control for configuration purposes.
 * 
 * @see AP_Motors6DOF::output() for motor mixing implementation
 * @see SRV_Channels for servo output management
 * @see Sub::update() for main loop integration
 */
void Sub::motors_output()
{
    // Motor detection mode controls the thrusters directly
    if (control_mode == Mode::Number::MOTOR_DETECT){
        return;
    }
    // check if we are performing the motor test
    if (ap.motor_test) {
        verify_motor_test();
    } else {
        motors.set_interlock(true);
        auto &srv = AP::srv();
        srv.cork();
        SRV_Channels::calc_pwm();
        SRV_Channels::output_ch_all();
        motors.output();
        srv.push();
    }
}

/**
 * @brief Initialize motor test mode with safety checks
 * 
 * @details Initializes the motor test feature which allows individual thruster control
 *          via MAVLink commands (MAV_CMD_DO_MOTOR_TEST). This is used for:
 *          - Verifying thruster directions and positions
 *          - ESC calibration procedures
 *          - Thruster performance testing
 *          - Motor matrix validation
 *          
 *          Safety Checks Performed:
 *          1. 10-second cooldown after previous test failure
 *          2. Hardware safety switch must be armed
 *          3. Vehicle must be armed (motors enabled)
 *          
 *          Once initialized, motor test mode requires continuous MAVLink commands
 *          (at least 2 Hz) or the test will timeout for safety.
 * 
 * @return true if motor test successfully initialized, false if safety checks failed
 * 
 * @note Motor test requires vehicle to be armed but NOT flying. Perform tests in
 *       a controlled environment with vehicle secured (e.g., out of water on bench).
 *       
 * @note ESC calibration: For bidirectional ESCs, follow manufacturer's calibration
 *       procedure. Most require: full throttle → arm → min throttle → disarm sequence.
 *       
 * @warning Motor test bypasses normal flight control. Vehicle will not maintain
 *          position or attitude during motor test. Ensure vehicle is secured.
 *          
 * @warning The 10-second cooldown after failure prevents rapid retry attempts that
 *          could cause hardware damage or unsafe conditions.
 * 
 * @see verify_motor_test() for timeout verification
 * @see handle_do_motor_test() for MAVLink command handling
 */
bool Sub::init_motor_test()
{
    uint32_t tnow = AP_HAL::millis();

    // Ten second cooldown period required with no do_set_motor requests required
    // after failure.
    if (tnow < last_do_motor_test_fail_ms + 10000 && last_do_motor_test_fail_ms > 0) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "10 second cooldown required after motor test");
        return false;
    }

    // check if safety switch has been pushed
    if (hal.util->safety_switch_state() == AP_HAL::Util::SAFETY_DISARMED) {
        gcs().send_text(MAV_SEVERITY_CRITICAL,"Disarm hardware safety switch before testing motors.");
        return false;
    }

    // Make sure we are on the ground
    if (!motors.armed()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Arm motors before testing motors.");
        return false;
    }

    enable_motor_output(); // set all motor outputs to zero
    ap.motor_test = true;

    return true;
}

/**
 * @brief Verify motor test is still active with command timeout check
 * 
 * @details Called every loop iteration when motor test is active to verify that
 *          MAVLink motor test commands are still being received. If commands stop
 *          arriving (timeout > 500ms), the motor test is automatically terminated
 *          for safety.
 *          
 *          Timeout Safety Mechanism:
 *          - Requires at least 2 Hz command rate (500ms max interval)
 *          - On timeout: disarms vehicle, ends motor test, sets cooldown timer
 *          - Prevents runaway motors if ground station connection is lost
 *          
 *          This implements a dead-man's switch: operator must continuously send
 *          commands or the test automatically terminates.
 * 
 * @return true if motor test is still valid, false if timed out
 * 
 * @note The 2 Hz (500ms) timeout is conservative to account for network latency
 *       while still providing reasonably fast safety response.
 *       
 * @warning On timeout, the vehicle is automatically disarmed. This will stop all
 *          thrusters immediately. Ensure vehicle is secured before testing.
 * 
 * @see init_motor_test() for motor test initialization
 * @see handle_do_motor_test() for command reception
 */
bool Sub::verify_motor_test()
{
    bool pass = true;

    // Require at least 2 Hz incoming do_set_motor requests
    if (AP_HAL::millis() > last_do_motor_test_ms + 500) {
        gcs().send_text(MAV_SEVERITY_INFO, "Motor test timed out!");
        pass = false;
    }

    if (!pass) {
        ap.motor_test = false;
        AP::arming().disarm(AP_Arming::Method::MOTORTEST);
        last_do_motor_test_fail_ms = AP_HAL::millis();
        return false;
    }

    return true;
}

/**
 * @brief Handle MAVLink MAV_CMD_DO_MOTOR_TEST command for individual motor control
 * 
 * @details Processes MAVLink motor test commands to control individual thrusters.
 *          Supports two throttle modes:
 *          - PWM: Direct PWM output in microseconds (1100-1900μs typical range)
 *          - Percent: Throttle as percentage (0-100%), mapped to configured PWM range
 *          
 *          Command Parameters (mavlink_command_int_t):
 *          - param1: Motor number (1-N, where N is number of motors in frame)
 *          - param2: Throttle type (PWM or PERCENT)
 *          - param3: Throttle value (PWM in μs, or 0-100 for percent)
 *          - param4: Timeout (not used - timeout handled by verify_motor_test)
 *          - param5: Motor count (not used)
 *          - y: Test type (must be MOTOR_TEST_ORDER_BOARD)
 *          
 *          Initialization Throttling:
 *          To prevent error message spam, initialization attempts are rate-limited
 *          to one attempt per 2 seconds on failure.
 * 
 * @param[in] command MAVLink command structure containing motor test parameters
 * 
 * @return true if motor output successfully set, false on error or unsupported parameters
 * 
 * @note Supported test type: MOTOR_TEST_ORDER_BOARD only (board motor order)
 * @note Unsupported throttle type: MOTOR_TEST_THROTTLE_PILOT (pilot throttle curve)
 *       
 * @note Motor numbering follows the frame configuration. Use MOTOR_DETECT mode
 *       or documentation to identify which physical thruster corresponds to each number.
 *       
 * @note For bidirectional ESCs (typical in ROVs), PWM 1500μs is neutral/stopped,
 *       values below 1500 are reverse thrust, above 1500 are forward thrust.
 * 
 * @warning Each command updates the timeout watchdog. Commands must continue at
 *          2+ Hz or motor test will automatically terminate.
 *          
 * @warning Direct PWM control bypasses safety limits. Use conservative values
 *          during initial testing to prevent damage or loss of control.
 * 
 * @see init_motor_test() for motor test initialization
 * @see verify_motor_test() for timeout handling
 * @see AP_Motors6DOF::output_test_num() for motor output implementation
 */
bool Sub::handle_do_motor_test(mavlink_command_int_t command) {
    last_do_motor_test_ms = AP_HAL::millis();

    // If we are not already testing motors, initialize test
    static uint32_t tLastInitializationFailed = 0;
    if(!ap.motor_test) {
        // Do not allow initializations attempt under 2 seconds
        // If one fails, we need to give the user time to fix the issue
        // instead of spamming error messages
        if (AP_HAL::millis() > (tLastInitializationFailed + 2000)) {
            if (!init_motor_test()) {
                gcs().send_text(MAV_SEVERITY_WARNING, "motor test initialization failed!");
                tLastInitializationFailed = AP_HAL::millis();
                return false; // init fail
            }
        } else {
            return false;
        }
    }

    float motor_number = command.param1;
    float throttle_type = command.param2;
    float throttle = command.param3;
    // float timeout_s = command.param4; // not used
    // float motor_count = command.param5; // not used
    const uint32_t test_type = command.y;

    if (test_type != MOTOR_TEST_ORDER_BOARD) {
        gcs().send_text(MAV_SEVERITY_WARNING, "bad test type %0.2f", (double)test_type);
        return false; // test type not supported here
    }

    if (is_equal(throttle_type, (float)MOTOR_TEST_THROTTLE_PILOT)) {
        gcs().send_text(MAV_SEVERITY_WARNING, "bad throttle type %0.2f", (double)throttle_type);

        return false; // throttle type not supported here
    }

    if (is_equal(throttle_type, (float)MOTOR_TEST_THROTTLE_PWM)) {
        return motors.output_test_num(motor_number, throttle); // true if motor output is set
    }

    if (is_equal(throttle_type, (float)MOTOR_TEST_THROTTLE_PERCENT)) {
        throttle = constrain_float(throttle, 0.0f, 100.0f);
        throttle = channel_throttle->get_radio_min() + throttle * 0.01f * (channel_throttle->get_radio_max() - channel_throttle->get_radio_min());
        return motors.output_test_num(motor_number, throttle); // true if motor output is set
    }

    return false;
}


/**
 * @brief Translate waypoint navigation roll/pitch outputs to lateral/forward motion
 * 
 * @details Converts attitude control outputs (roll/pitch in centidegrees) from the
 *          waypoint navigation controller into normalized lateral/forward translation
 *          commands for underwater vehicles. This adaptation is necessary because
 *          ROVs/submarines use thrusters for direct translation rather than banking
 *          like aircraft.
 *          
 *          Translation Process:
 *          1. Get roll (lateral) and pitch (forward) targets from wp_nav in centidegrees
 *          2. Reverse pitch output (navigation convention vs thruster convention)
 *          3. Constrain to configured angle limits (aparm.angle_max)
 *          4. Normalize to -1.0 to +1.0 range for motor mixing
 *          
 *          Coordinate Transformation:
 *          - Roll → Lateral (right positive)
 *          - Pitch → Forward (forward positive, output reversed)
 *          
 *          The normalized outputs are then fed to the motor mixer which combines
 *          them with vertical throttle and rotational commands to compute individual
 *          thruster outputs via the motor matrix.
 * 
 * @param[out] lateral_out Normalized lateral motion command (-1.0 to +1.0)
 *                         Positive = right, Negative = left
 * @param[out] forward_out Normalized forward motion command (-1.0 to +1.0)
 *                         Positive = forward, Negative = backward
 * 
 * @note Roll/pitch outputs from wp_nav should already be constrained, but additional
 *       constraint is applied here for safety.
 *       
 * @note The pitch output is negated due to coordinate frame conventions. This ensures
 *       positive pitch command results in forward motion.
 *       
 * @note Units: Input in centidegrees (1/100 degree), output normalized to ±1.0
 * 
 * @see translate_pos_control_rp() for position controller translation
 * @see translate_circle_nav_rp() for circle navigation translation
 * @see AP_Motors6DOF for motor mixing that uses these outputs
 */
void Sub::translate_wpnav_rp(float &lateral_out, float &forward_out)
{
    // get roll and pitch targets in centidegrees
    int32_t lateral = wp_nav.get_roll();
    int32_t forward = -wp_nav.get_pitch(); // output is reversed

    // constrain target forward/lateral values
    // The outputs of wp_nav.get_roll and get_pitch should already be constrained to these values
    lateral = constrain_int16(lateral, -aparm.angle_max, aparm.angle_max);
    forward = constrain_int16(forward, -aparm.angle_max, aparm.angle_max);

    // Normalize
    lateral_out = (float)lateral/(float)aparm.angle_max;
    forward_out = (float)forward/(float)aparm.angle_max;
}

/**
 * @brief Translate circle navigation roll/pitch outputs to lateral/forward motion
 * 
 * @details Converts attitude control outputs (roll/pitch in centidegrees) from the
 *          circle navigation controller into normalized lateral/forward translation
 *          commands. Circle mode makes the vehicle orbit around a point at a specified
 *          radius and rate.
 *          
 *          Translation Process:
 *          1. Get roll (lateral) and pitch (forward) targets from circle_nav in centidegrees
 *          2. Reverse pitch output (navigation convention vs thruster convention)
 *          3. Constrain to configured angle limits (aparm.angle_max)
 *          4. Normalize to -1.0 to +1.0 range for motor mixing
 *          
 *          Circle Navigation Behavior:
 *          The circle controller outputs roll/pitch commands that create a circular
 *          path. These are translated to lateral/forward thrust to follow the circle
 *          without banking the vehicle (as would happen in an aircraft).
 * 
 * @param[out] lateral_out Normalized lateral motion command (-1.0 to +1.0)
 *                         Positive = right, Negative = left
 * @param[out] forward_out Normalized forward motion command (-1.0 to +1.0)
 *                         Positive = forward, Negative = backward
 * 
 * @note This function is structurally identical to translate_wpnav_rp() but operates
 *       on circle_nav outputs instead of wp_nav outputs.
 *       
 * @note The pitch output is negated due to coordinate frame conventions.
 *       
 * @note Units: Input in centidegrees (1/100 degree), output normalized to ±1.0
 * 
 * @see translate_wpnav_rp() for waypoint navigation translation
 * @see translate_pos_control_rp() for position controller translation  
 * @see AC_Circle for circle navigation implementation
 */
void Sub::translate_circle_nav_rp(float &lateral_out, float &forward_out)
{
    // get roll and pitch targets in centidegrees
    int32_t lateral = circle_nav.get_roll_cd();
    int32_t forward = -circle_nav.get_pitch_cd(); // output is reversed

    // constrain target forward/lateral values
    lateral = constrain_int16(lateral, -aparm.angle_max, aparm.angle_max);
    forward = constrain_int16(forward, -aparm.angle_max, aparm.angle_max);

    // Normalize
    lateral_out = (float)lateral/(float)aparm.angle_max;
    forward_out = (float)forward/(float)aparm.angle_max;
}

/**
 * @brief Translate position controller roll/pitch outputs to lateral/forward motion
 * 
 * @details Converts attitude control outputs (roll/pitch in centidegrees) from the
 *          position controller into normalized lateral/forward translation commands.
 *          The position controller is used in modes like POSHOLD, GUIDED, and AUTO
 *          to maintain or track position targets.
 *          
 *          Translation Process:
 *          1. Get roll (lateral) and pitch (forward) targets from pos_control in centidegrees
 *          2. Reverse pitch output (navigation convention vs thruster convention)
 *          3. Constrain to configured angle limits (aparm.angle_max)
 *          4. Normalize to -1.0 to +1.0 range for motor mixing
 *          
 *          Position Control Architecture:
 *          The position controller implements cascaded PID loops:
 *          - Position error → Velocity command
 *          - Velocity error → Acceleration command
 *          - Acceleration → Attitude (roll/pitch) command
 *          
 *          This function performs the final step: attitude → thruster translation.
 * 
 * @param[out] lateral_out Normalized lateral motion command (-1.0 to +1.0)
 *                         Positive = right, Negative = left
 * @param[out] forward_out Normalized forward motion command (-1.0 to +1.0)
 *                         Positive = forward, Negative = backward
 * 
 * @note This function is structurally identical to translate_wpnav_rp() but operates
 *       on pos_control outputs, which are used in different flight modes.
 *       
 * @note The pitch output is negated due to coordinate frame conventions.
 *       
 * @note Units: Input in centidegrees (1/100 degree), output normalized to ±1.0
 *       
 * @note The position controller's roll/pitch outputs represent desired accelerations
 *       translated through the attitude control system, not literal vehicle tilts.
 * 
 * @see translate_wpnav_rp() for waypoint navigation translation
 * @see translate_circle_nav_rp() for circle navigation translation
 * @see AC_PosControl for position controller implementation
 */
void Sub::translate_pos_control_rp(float &lateral_out, float &forward_out)
{
    // get roll and pitch targets in centidegrees
    int32_t lateral = pos_control.get_roll_cd();
    int32_t forward = -pos_control.get_pitch_cd(); // output is reversed

    // constrain target forward/lateral values
    lateral = constrain_int16(lateral, -aparm.angle_max, aparm.angle_max);
    forward = constrain_int16(forward, -aparm.angle_max, aparm.angle_max);

    // Normalize
    lateral_out = (float)lateral/(float)aparm.angle_max;
    forward_out = (float)forward/(float)aparm.angle_max;
}
