/**
 * @file mode_turtle.cpp
 * @brief Turtle mode implementation for crash recovery of inverted multicopters
 * 
 * @details Turtle mode is a specialized flight mode designed for self-recovery when a
 *          racing quadcopter crashes and lands upside-down. This mode reverses motor
 *          directions using bi-directional DShot ESC protocols to flip the vehicle
 *          right-side-up without manual intervention.
 *          
 *          Key Features:
 *          - Detects inverted orientation using IMU data
 *          - Reverses motor spin direction via DShot commands
 *          - Automated flip sequence controlled by pilot stick inputs
 *          - Safety checks ensure vehicle is disarmed and on ground before entry
 *          - Requires bi-directional DShot ESC capability (DShot150/300/600)
 *          
 *          Intended Use: Racing quadcopters and acrobatic multirotors with motor
 *          reversal capability. NOT for use with conventional ESCs or large/heavy vehicles.
 *          
 *          Safety Considerations:
 *          - Mode entry only allowed when disarmed and on ground
 *          - Temporarily disables failsafes during motor reversal
 *          - Requires centered sticks and zero throttle for mode entry
 *          - Motors arm automatically when throttle raised in this mode
 * 
 * @warning This is a potentially dangerous mode that reverses motors. Only use with
 *          appropriate hardware and in controlled environments.
 * 
 * @see ModeTurtle class in Copter.h
 * @see AP_HAL::RCOutput for DShot command interface
 */
#include "Copter.h"

#if MODE_TURTLE_ENABLED

// Exponential curve factor for stick input shaping (35% linear, 65% cubic response)
#define CRASH_FLIP_EXPO 35.0f
// Minimum stick deflection threshold to activate flip (15% of full range)
#define CRASH_FLIP_STICK_MINF 0.15f
// Macro for cubic power calculation used in expo curve
#define power3(x) ((x) * (x) * (x))

/**
 * @brief Initialize Turtle mode for crash recovery
 * 
 * @details Performs comprehensive safety checks before allowing entry into Turtle mode.
 *          This mode can only be entered when the vehicle is disarmed, on the ground,
 *          and using DShot ESCs that support bi-directional motor control.
 *          
 *          Safety Check Sequence:
 *          1. Verify vehicle is disarmed (prevents mode entry during flight)
 *          2. Verify DShot ESCs are configured (required for motor reversal)
 *          3. Execute MAVLink motor control permission checks
 *          4. Verify all control sticks are centered (pitch, roll, yaw)
 *          5. Verify throttle is at zero position
 *          
 *          Mode Entry Requirements:
 *          - Vehicle must be disarmed
 *          - DShot ESC type must be configured (DShot150/300/600)
 *          - All control inputs must be in neutral position
 *          - MAVLink motor control must be authorized
 *          
 * @param[in] ignore_checks Currently unused; checks are always performed for safety
 * 
 * @return true if mode successfully initialized and ready for motor arming
 * @return false if any safety check fails, preventing mode entry
 * 
 * @note This mode bypasses normal arming checks - use with extreme caution
 * @warning Vehicle will arm automatically when throttle is raised in this mode
 * 
 * @see arm_motors() for motor arming sequence with direction reversal
 * @see SRV_Channels::get_dshot_esc_type() for ESC capability detection
 */
bool ModeTurtle::init(bool ignore_checks)
{
    WITH_SEMAPHORE(msem);

    // Safety check: do not enter the mode when already armed or when flying
    // Also verify DShot ESC support (required for motor direction reversal)
    if (motors->armed() || SRV_Channels::get_dshot_esc_type() == 0) {
        return false;
    }

    // Perform minimal arming checks to ensure MAVLink motor control is permitted
    if (!copter.mavlink_motor_control_check(*gcs().chan(0), true, "Turtle Mode")) {
        return false;
    }

    // Safety check: do not enter the mode if sticks are not centered or throttle is not at zero
    // This prevents accidental motor activation during mode entry
    if (!is_zero(channel_pitch->norm_input_dz())
        || !is_zero(channel_roll->norm_input_dz())
        || !is_zero(channel_yaw->norm_input_dz())
        || !is_zero(channel_throttle->norm_input_dz())) {
        return false;
    }

    // Clear shutdown flag to allow motor arming
    shutdown = false;

    return true;
}

/**
 * @brief Arm motors with reversed direction for turtle mode flip recovery
 * 
 * @details Executes a specialized arming sequence that reverses motor spin direction
 *          using DShot commands before arming. This allows the inverted vehicle to
 *          generate thrust in the correct direction to flip itself right-side-up.
 *          
 *          Arming Sequence:
 *          1. Check if already armed (skip if armed to prevent re-reversal)
 *          2. Disable motor spoolup blocking
 *          3. Disable channel mask updates during motor reversal
 *          4. Send DShot commands to reverse all motor directions
 *          5. Temporarily disable failsafes (throttle, GCS, EKF)
 *          6. Notify arming library of turtle mode arming
 *          7. Set armed state and enable motor outputs
 *          
 *          Safety Implications:
 *          - Failsafes are disabled to prevent interference during flip
 *          - Normal throttle limits and checks are bypassed
 *          - EKF failsafe reduced to report-only to prevent disarm during flip
 *          - Motor direction reversal requires bi-directional DShot ESCs
 * 
 * @note This function is called automatically when throttle is raised in turtle mode
 * @note Failsafes are re-enabled when motors are disarmed via disarm_motors()
 * 
 * @warning Disables multiple failsafes - use only in controlled environments
 * @warning Motors will spin in reverse direction after this call
 * 
 * @see change_motor_direction() for DShot motor reversal implementation
 * @see disarm_motors() for reverse operation and failsafe re-enable
 * @see output_to_motors() for automatic arming trigger
 */
void ModeTurtle::arm_motors()
{
    WITH_SEMAPHORE(msem);

    // Prevent re-arming if already armed
    if (hal.util->get_soft_armed()) {
        return;
    }

    // Disable spoolup block to allow immediate motor control
    motors->set_spoolup_block(false);

    // Reverse the motor directions using DShot commands
    // Disable channel mask updates during reversal to ensure atomic operation
    hal.rcout->disable_channel_mask_updates();
    change_motor_direction(true);

    // Temporarily disable failsafes during turtle mode operation
    // These will be restored when motors are disarmed
    g.failsafe_throttle.set(FS_THR_DISABLED);
    g.failsafe_gcs.set(FS_GCS_DISABLED);
    g.fs_ekf_action.set(FS_EKF_ACTION_REPORT_ONLY);

    // Notify arming library of turtle mode arming
    // Even if it fails we don't want to prevent people getting into turtle mode
    AP::arming().AP_Arming::arm(AP_Arming::Method::TURTLE_MODE, false);
    
    // Set armed state to enable motor outputs
    motors->armed(true);
    hal.util->set_soft_armed(true);
}

/**
 * @brief Check if arming is allowed in turtle mode
 * 
 * @details Turtle mode uses a specialized arming mechanism that bypasses normal
 *          pre-arm checks. This function always returns true to allow the custom
 *          turtle mode arming sequence to proceed.
 * 
 * @param[in] method Arming method being used (typically TURTLE_MODE)
 * 
 * @return true Always allows arming (safety checks performed in init())
 * 
 * @note Normal arming checks are performed in init() before mode entry
 * @see init() for turtle mode entry safety checks
 * @see arm_motors() for actual arming sequence with motor reversal
 */
bool ModeTurtle::allows_arming(AP_Arming::Method method) const
{
    return true;
}

/**
 * @brief Exit turtle mode and restore normal vehicle configuration
 * 
 * @details Cleanly exits turtle mode by disarming motors, restoring normal motor
 *          direction, re-enabling failsafes, and clearing notification indicators.
 *          This function is called automatically when the pilot exits the mode.
 *          
 *          Exit Sequence:
 *          1. Set shutdown flag to trigger disarm
 *          2. Disarm motors and restore normal direction
 *          3. Re-enable all failsafes to normal values
 *          4. Clear ESC calibration notification LEDs
 * 
 * @note All changes made during arm_motors() are reversed
 * @see disarm_motors() for motor shutdown and direction restoration
 */
void ModeTurtle::exit()
{
    // Signal motor output system to shut down
    shutdown = true;

    // Disarm motors and restore normal configuration
    disarm_motors();

    // Turn off ESC calibration notification LEDs
    AP_Notify::flags.esc_calibration = false;
}

/**
 * @brief Disarm motors and restore normal motor direction and failsafes
 * 
 * @details Reverses the arming sequence performed by arm_motors(), restoring the
 *          vehicle to normal configuration. This includes un-reversing motor
 *          directions via DShot commands and re-enabling all failsafes.
 *          
 *          Disarm Sequence:
 *          1. Check if armed (skip if already disarmed)
 *          2. Notify arming library of turtle mode disarm
 *          3. Set motors to disarmed state
 *          4. Send DShot commands to restore normal motor direction
 *          5. Re-enable channel mask updates
 *          6. Reload and re-enable all failsafes (throttle, GCS, EKF)
 *          7. Clear soft armed state
 *          
 *          Safety Restoration:
 *          - Motors return to normal spin direction
 *          - Throttle failsafe restored to configured value
 *          - GCS failsafe restored to configured value
 *          - EKF failsafe restored to configured action
 * 
 * @note Called automatically when throttle is lowered to zero or mode is exited
 * @note Restores all failsafe settings modified by arm_motors()
 * 
 * @see arm_motors() for the reverse operation
 * @see change_motor_direction() for DShot direction restoration
 */
void ModeTurtle::disarm_motors()
{
    WITH_SEMAPHORE(msem);

    // Prevent disarming if already disarmed
    if (!hal.util->get_soft_armed()) {
        return;
    }

    // Notify arming library of turtle mode disarm
    AP::arming().AP_Arming::disarm(AP_Arming::Method::TURTLE_MODE, false);

    // Set motors to disarmed state
    motors->armed(false);

    // Restore normal motor spin direction using DShot commands
    change_motor_direction(false);
    hal.rcout->enable_channel_mask_updates();

    // Re-enable failsafes by loading saved configuration values
    g.failsafe_throttle.load();
    g.failsafe_gcs.load();
    g.fs_ekf_action.load();

    // Clear soft armed state
    hal.util->set_soft_armed(false);
}

/**
 * @brief Change motor spin direction using DShot commands
 * 
 * @details Sends DShot protocol commands to ESCs to reverse or restore normal motor
 *          spin direction. This is the core functionality that enables turtle mode,
 *          allowing an inverted vehicle to generate upward thrust by spinning motors
 *          in reverse.
 *          
 *          Implementation Handles Two Scenarios:
 *          
 *          1. No Pre-Reversed Motors (normal configuration):
 *             - Sends single command to all channels simultaneously
 *             - All motors receive same direction command
 *             
 *          2. Pre-Reversed Motors (some motors already reversed in configuration):
 *             - Iterates through each motor individually
 *             - Checks reversed mask to determine current motor direction
 *             - Applies opposite command to pre-reversed motors
 *             - Ensures all motors end up spinning in desired direction
 *          
 *          DShot Command Details:
 *          - DSHOT_REVERSE: Command to reverse motor direction
 *          - DSHOT_NORMAL: Command to restore normal motor direction
 *          - Commands sent with 10 retries to ensure reception
 *          - Wait for ESC response enabled (true parameter)
 * 
 * @param[in] reverse true to reverse motor direction, false to restore normal
 * 
 * @note Requires ESCs with bi-directional DShot support (DShot150/300/600)
 * @note Some motor configurations have pre-reversed motors (e.g., X-frame patterns)
 * @warning Motor direction change takes effect immediately on next throttle command
 * 
 * @see AP_HAL::RCOutput::send_dshot_command() for DShot protocol implementation
 * @see arm_motors() for usage during arming with direction=true
 * @see disarm_motors() for usage during disarm with direction=false
 */
void ModeTurtle::change_motor_direction(bool reverse)
{
    // Determine DShot commands based on desired direction
    AP_HAL::RCOutput::BLHeliDshotCommand direction = reverse ? AP_HAL::RCOutput::DSHOT_REVERSE : AP_HAL::RCOutput::DSHOT_NORMAL;
    AP_HAL::RCOutput::BLHeliDshotCommand inverse_direction = reverse ? AP_HAL::RCOutput::DSHOT_NORMAL : AP_HAL::RCOutput::DSHOT_REVERSE;

    // Check if any motors are pre-configured as reversed
    if (!hal.rcout->get_reversed_mask()) {
        // Simple case: no pre-reversed motors, send same command to all channels
        hal.rcout->send_dshot_command(direction, AP_HAL::RCOutput::ALL_CHANNELS, 0, 10, true);
    } else {
        // Complex case: some motors are pre-reversed, handle each motor individually
        for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; ++i) {
            // Skip disabled motors
            if (!motors->is_motor_enabled(i)) {
                continue;
            }

            // Check if this motor is pre-reversed in configuration
            if ((hal.rcout->get_reversed_mask() & (1U << i)) == 0) {
                // Normal motor: apply standard direction command
                hal.rcout->send_dshot_command(direction, i, 0, 10, true);
            } else {
                // Pre-reversed motor: apply inverse command to achieve same final direction
                hal.rcout->send_dshot_command(inverse_direction, i, 0, 10, true);
            }
        }
    }
}

/**
 * @brief Main control loop for turtle mode flip recovery
 * 
 * @details Processes pilot stick inputs and calculates motor commands to flip an inverted
 *          vehicle right-side-up. Uses exponential curve shaping, directional locking,
 *          and motor selection logic to provide intuitive and powerful flip control.
 *          
 *          Control Algorithm Overview:
 *          
 *          1. Input Processing:
 *             - Read pitch, roll, yaw stick positions (normalized -1.0 to 1.0)
 *             - Handle RC signal loss (default to zero input)
 *             - Apply deadzone filtering
 *          
 *          2. Exponential Curve Shaping:
 *             - Apply 35% linear + 65% cubic response curve
 *             - Provides gentle control near center, aggressive at extremes
 *             - Calculated per-axis before combining
 *          
 *          3. Directional Locking Logic:
 *             - Yaw Dominance: If yaw > pitch/roll, disable pitch/roll (pure rotation)
 *             - Diagonal Detection: If input within 30° of axis, lock to single axis
 *             - Prevents confusing multi-axis flips, ensures predictable motion
 *          
 *          4. Power Calculation:
 *             - Apply minimum stick deflection threshold (15%)
 *             - Scale remaining range to 0..1 motor power
 *             - Convert to actuator output using thrust curve
 *          
 *          5. Motor Direction Selection:
 *             - Calculate normalized flip direction vector
 *             - Scale to ±0.5 range for motor matching
 *             - output_to_motors() selects motors aligned with flip direction
 *          
 *          Coordinate Frame:
 *          - Pitch: Forward (positive) / Backward (negative) flip
 *          - Roll: Right (positive) / Left (negative) flip  
 *          - Yaw: Rotation about vertical axis when vehicle inverted
 * 
 * @note Called at main loop rate (typically 400Hz) when in turtle mode
 * @note Motor selection and output performed by output_to_motors()
 * @note Bypasses normal attitude control - direct motor power control
 * 
 * @see output_to_motors() for motor selection and PWM output
 * @see motors_input for calculated flip direction vector
 * @see motors_output for calculated motor power level
 */
void ModeTurtle::run()
{
    // Calculate linear component of expo curve (35% linear, 65% cubic)
    const float flip_power_factor = 1.0f - CRASH_FLIP_EXPO * 0.01f;
    
    // Read stick inputs with RC loss protection
    const bool norc = !rc().has_valid_input();
    const float stick_deflection_pitch = norc ? 0.0f : channel_pitch->norm_input_dz();
    const float stick_deflection_roll = norc ? 0.0f : channel_roll->norm_input_dz();
    const float stick_deflection_yaw = norc ? 0.0f : channel_yaw->norm_input_dz();

    // Calculate absolute stick deflections for processing
    const float stick_deflection_pitch_abs = fabsf(stick_deflection_pitch);
    const float stick_deflection_roll_abs = fabsf(stick_deflection_roll);
    const float stick_deflection_yaw_abs = fabsf(stick_deflection_yaw);

    // Apply exponential curve to each axis: expo = linear_factor * x + cubic_factor * x^3
    const float stick_deflection_pitch_expo = flip_power_factor * stick_deflection_pitch_abs + power3(stick_deflection_pitch_abs) * (1 - flip_power_factor);
    const float stick_deflection_roll_expo = flip_power_factor * stick_deflection_roll_abs + power3(stick_deflection_roll_abs) * (1 - flip_power_factor);
    const float stick_deflection_yaw_expo = flip_power_factor * stick_deflection_yaw_abs + power3(stick_deflection_yaw_abs) * (1 - flip_power_factor);

    // Extract direction signs (note: roll sign inverted for correct motor mapping)
    float sign_pitch = stick_deflection_pitch < 0 ? -1 : 1;
    float sign_roll = stick_deflection_roll < 0 ? 1 : -1;

    // Calculate combined pitch/roll magnitude (Euclidean distance)
    float stick_deflection_length = sqrtf(sq(stick_deflection_pitch_abs) + sq(stick_deflection_roll_abs));
    float stick_deflection_expo_length = sqrtf(sq(stick_deflection_pitch_expo) + sq(stick_deflection_roll_expo));

    // Directional locking: if yaw is dominant, disable pitch and roll
    if (stick_deflection_yaw_abs > MAX(stick_deflection_pitch_abs, stick_deflection_roll_abs)) {
        // Yaw-only mode for pure rotation flips
        stick_deflection_length = stick_deflection_yaw_abs;
        stick_deflection_expo_length = stick_deflection_yaw_expo;
        sign_roll = 0;
        sign_pitch = 0;
    }

    // Calculate angle between input vector and nearest axis
    // cos_phi measures how close input is to 45° diagonal
    const float cos_phi = (stick_deflection_length > 0) ? (stick_deflection_pitch_abs + stick_deflection_roll_abs) / (sqrtf(2.0f) * stick_deflection_length) : 0;
    const float cos_threshold = sqrtf(3.0f) / 2.0f; // cos(30°) = 0.866

    // Axis locking: if input is more than 30° from diagonal, lock to single axis
    if (cos_phi < cos_threshold) {
        // Enforce either roll or pitch exclusively, not diagonal
        if (stick_deflection_roll_abs > stick_deflection_pitch_abs) {
            sign_pitch = 0;  // Pure roll flip
        } else {
            sign_roll = 0;   // Pure pitch flip
        }
    }

    // Apply minimum stick deflection threshold with expo curve
    const float crash_flip_stick_min_expo = flip_power_factor * CRASH_FLIP_STICK_MINF + power3(CRASH_FLIP_STICK_MINF) * (1 - flip_power_factor);
    const float flip_stick_range = 1.0f - crash_flip_stick_min_expo;
    
    // Calculate final flip power in range 0..1
    const float flip_power = MAX(0.0f, stick_deflection_expo_length - crash_flip_stick_min_expo) / flip_stick_range;

    // Normalize the roll and pitch input to create flip direction vector
    // Scaled to ±0.5 range to match motor factor range
    Vector2f input{sign_roll, sign_pitch};
    motors_input = input.normalized() * 0.5;
    
    // Convert flip power to motor actuator output value
    // Bypass spin_min in deadzone - motors only spin when stick moved beyond threshold
    motors_output = !is_zero(flip_power) ? motors->thr_lin.thrust_to_actuator(flip_power) : 0.0f;
}

/**
 * @brief Write motor outputs for turtle mode flip recovery
 * 
 * @details Translates pilot flip commands into individual motor PWM outputs by selecting
 *          motors aligned with the desired flip direction. This is the output stage that
 *          actually drives the motors to flip the inverted vehicle right-side-up.
 *          
 *          Output Sequence:
 *          
 *          1. Shutdown Check:
 *             - If shutdown flag set, disarm motors and exit
 *             - Clear ESC calibration LED notification
 *          
 *          2. Throttle Arming Check:
 *             - Verify throttle is raised from zero position
 *             - If throttle at zero, display warning message (5 second interval)
 *             - Disarm motors if throttle lowered (safety feature)
 *          
 *          3. Motor Arming:
 *             - Enable ESC calibration LED notification (visual indicator)
 *             - Call arm_motors() to reverse motor direction and arm
 *          
 *          4. Motor Selection and Output:
 *             - Iterate through all configured motors
 *             - Get each motor's roll/pitch contribution factors
 *             - Calculate alignment between motor vector and flip direction
 *             - Select motors aligned within 0.5 vector distance threshold
 *             - Drive selected motors with calculated power level
 *             - Disable non-aligned motors (set to minimum PWM)
 *          
 *          Motor Selection Algorithm:
 *          - Motor factors represent thrust contribution to roll/pitch
 *          - Flip direction (motors_input) is normalized vector from run()
 *          - Vector distance < 0.5 means motor contributes to desired flip
 *          - Only aligned motors receive power, others stay at minimum
 *          
 *          Example: Forward flip (pitch stick forward)
 *          - Front motors disabled (opposite direction)
 *          - Rear motors powered (aligned with flip direction)
 *          - Creates pitching moment to flip forward
 * 
 * @note Called at main loop rate (typically 400Hz) from Copter scheduler
 * @note Throttle must be raised to arm motors automatically
 * @note Uses motors_input and motors_output calculated by run()
 * 
 * @warning Motors will spin in reverse direction - ensure safe environment
 * @see run() for flip direction and power calculation
 * @see arm_motors() for motor arming with direction reversal
 * @see disarm_motors() for motor shutdown when throttle lowered
 */
void ModeTurtle::output_to_motors()
{
    // Check if mode is shutting down
    if (shutdown) {
        disarm_motors();
        // Turn off ESC calibration LED notification
        AP_Notify::flags.esc_calibration = false;
        return;
    }

    // Safety check: throttle must be raised to arm motors
    if (is_zero(channel_throttle->norm_input_dz())) {
        // Display periodic warning message to pilot
        const uint32_t now = AP_HAL::millis();
        if (now - last_throttle_warning_output_ms > 5000) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Turtle: raise throttle to arm");
            last_throttle_warning_output_ms = now;
        }

        // Disarm if throttle is lowered during operation
        disarm_motors();
        return;
    }

    // Turn on ESC calibration LED as visual indicator that turtle mode is active
    AP_Notify::flags.esc_calibration = true;

    // Arm motors with reversed direction if not already armed
    arm_motors();

    // Check if motors are allowed to spin (armed and interlock enabled)
    const bool allow_output = motors->armed() && motors->get_interlock();

    // Iterate through all motors and selectively power those aligned with flip direction
    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; ++i) {
        // Skip motors that are not configured/enabled
        if (!motors->is_motor_enabled(i)) {
            continue;
        }

        // Get this motor's roll and pitch contribution factors
        const Vector2f output{motors->get_roll_factor(i), motors->get_pitch_factor(i)};
        
        // Check if motor output aligns with desired flip input direction
        // Vector distance > 0.5 means motor is not aligned with flip direction
        if (!allow_output || (motors_input - output).length() > 0.5) {
            // Motor not aligned or output disabled - set to minimum PWM
            motors->rc_write(i, motors->get_pwm_output_min());
            continue;
        }

        // Motor is aligned - calculate PWM based on desired flip power
        int16_t pwm = motors->get_pwm_output_min() + (motors->get_pwm_output_max() - motors->get_pwm_output_min()) * motors_output;

        // Write calculated PWM value to this motor
        motors->rc_write(i, pwm);
    }
}

#endif
