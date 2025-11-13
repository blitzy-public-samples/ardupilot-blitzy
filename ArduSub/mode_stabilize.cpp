/**
 * @file mode_stabilize.cpp
 * @brief Stabilize mode implementation for ArduSub
 * 
 * @details Stabilize mode is the primary manual control mode for ArduSub, providing
 *          attitude stabilization while allowing direct pilot control of the vehicle.
 *          This mode maintains desired roll and pitch angles, provides yaw rate control
 *          with heading hold, and allows direct throttle control for depth changes along
 *          with independent forward and lateral thrust control for translation.
 *          
 *          Stabilize mode is typically the default flight mode and provides the most
 *          intuitive control for manual piloting of underwater vehicles. The attitude
 *          stabilization prevents the vehicle from tumbling while still giving the pilot
 *          full authority over vehicle motion.
 *          
 *          Control Input Processing:
 *          - Roll/Pitch: Pilot stick inputs are converted to desired lean angles
 *          - Yaw: Pilot input controls yaw rate with automatic heading hold when centered
 *          - Throttle: Direct control of vertical thrust for depth changes
 *          - Forward/Lateral: Direct control of horizontal translation thrusters
 *          
 * @note This is the recommended mode for normal manual operations and is typically
 *       set as the default mode on vehicle startup.
 * 
 * @note Stabilize mode requires a functioning AHRS (Attitude and Heading Reference System)
 *       to maintain attitude control. If AHRS fails, the vehicle should be switched to
 *       a manual passthrough mode.
 * 
 * Source: ArduSub/mode_stabilize.cpp
 */
#include "Sub.h"


/**
 * @brief Initialize Stabilize mode
 * 
 * @details Performs initialization when entering Stabilize mode. This function sets up
 *          the initial state for attitude-stabilized manual control:
 *          
 *          1. Resets desired altitude position to zero for accurate position reporting
 *          2. Captures current vehicle heading as the initial heading reference
 *          
 *          The initialization always succeeds regardless of the ignore_checks parameter
 *          since Stabilize mode has minimal requirements (only functional AHRS needed,
 *          which is verified at a higher level).
 * 
 * @param[in] ignore_checks If true, skip pre-flight checks (currently unused as
 *                          Stabilize mode has no specific entry requirements)
 * 
 * @return true Always returns true - Stabilize mode initialization always succeeds
 * 
 * @note The desired altitude is set to zero for the position controller's reference,
 *       but actual depth control in Stabilize mode is handled through direct throttle
 *       commands rather than closed-loop altitude control.
 * 
 * @note Current heading is stored to enable heading hold functionality when pilot
 *       yaw input is centered during the run() method execution.
 * 
 * Source: ArduSub/mode_stabilize.cpp:4-11
 */
bool ModeStabilize::init(bool ignore_checks) {
    // set target altitude to zero for reporting
    position_control->set_pos_desired_U_cm(0);
    sub.last_pilot_heading = ahrs.yaw_sensor;

    return true;
    return true;
}

/**
 * @brief Main control loop for Stabilize mode
 * 
 * @details Executes the primary control algorithm for Stabilize mode, called at the
 *          scheduler's main loop rate (typically 50-400 Hz depending on configuration).
 *          
 *          Algorithm Overview:
 *          1. Safety Check: Verify motors are armed, otherwise idle all outputs
 *          2. Pilot Input Processing: Convert stick inputs to desired angles and rates
 *          3. Attitude Control: Send desired attitudes to attitude controller
 *          4. Heading Hold: Implement yaw rate control with heading hold functionality
 *          5. Throttle Output: Pass pilot throttle directly to motors for depth control
 *          6. Translation Control: Set forward and lateral thrust from pilot inputs
 *          
 *          Attitude Control Integration:
 *          The mode uses the vehicle's attitude controller (AC_AttitudeControl) to
 *          maintain desired roll and pitch angles while controlling yaw rate. Roll and
 *          pitch inputs are converted to target angles (limited by angle_max parameter),
 *          while yaw input is processed as a rate command.
 *          
 *          Heading Hold Implementation:
 *          When pilot yaw input is zero (stick centered), the mode implements heading
 *          hold by commanding the attitude controller to maintain the last heading when
 *          pilot input was non-zero. A 250ms deceleration period prevents heading bounce-
 *          back after rapid yaw maneuvers due to vehicle inertia.
 *          
 *          Throttle Handling for Depth Control:
 *          Pilot throttle input is directly passed to the motors without closed-loop
 *          depth control. The throttle value is normalized from the input range to
 *          0.0-1.0 before being sent to the attitude controller's throttle output,
 *          which then distributes thrust across the vertical motors.
 *          
 *          Forward and Lateral Thrust Control:
 *          Independent control of horizontal translation is provided through direct
 *          pass-through of pilot forward and lateral stick inputs to the motor mixer.
 *          These inputs control the horizontal thrusters independently of attitude
 *          control, allowing translation while maintaining attitude.
 * 
 * @note Called at main loop rate (typically 50-400 Hz) - ensure execution time is
 *       minimal to maintain real-time performance
 * 
 * @note This mode provides direct pilot authority and is suitable for all normal
 *       manual operations including takeoff, landing, and general maneuvering
 * 
 * @warning If motors are not armed, all outputs are set to safe idle values and
 *          attitude controllers are relaxed to prevent integrator wind-up
 * 
 * @see AC_AttitudeControl::input_euler_angle_roll_pitch_euler_rate_yaw_cd()
 * @see AC_AttitudeControl::input_euler_angle_roll_pitch_yaw_cd()
 * @see Sub::get_pilot_desired_lean_angles()
 * @see Sub::get_pilot_desired_yaw_rate()
 * 
 * Source: ArduSub/mode_stabilize.cpp:13-69
 */
void ModeStabilize::run()
{
  uint32_t tnow = AP_HAL::millis();
    float target_roll, target_pitch;

    // Safety Check: If not armed, set all outputs to safe idle state and exit
    // This prevents any motor movement when the vehicle is disarmed, which is
    // critical for safety during transport, maintenance, and pre-flight checks
    if (!motors.armed()) {
        // Set motor spool state to ground idle (minimal/zero throttle)
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        
        // Set throttle output to zero with angular velocity limit disabled
        attitude_control->set_throttle_out(0,true,g.throttle_filt);
        
        // Relax attitude controllers to prevent integrator wind-up while disarmed
        // This resets PID integrators so they don't accumulate error while stationary
        attitude_control->relax_attitude_controllers();
        
        // Update heading reference to current heading to prevent sudden yaw on arming
        sub.last_pilot_heading = ahrs.yaw_sensor;
        return;
    }

    // Set motor spool state to unlimited throttle since vehicle is armed
    // This allows full motor output range for attitude control and depth changes
    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // Pilot Input Processing: Convert pilot roll and pitch stick inputs to desired lean angles
    // 
    // The stick inputs (in centidegrees) are scaled and limited by the angle_max parameter
    // to produce target roll and pitch angles. This converts the raw RC input into
    // attitude targets that the attitude controller can track.
    //
    // Input: RC stick positions from roll and pitch channels (range: -4500 to 4500 centidegrees)
    // Output: target_roll and target_pitch in centidegrees, limited by angle_max
    //
    // To-Do: convert sub.get_pilot_desired_lean_angles to return angles as floats
    // TODO2: move into mode.h
    sub.get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, sub.aparm.angle_max);

    // Yaw Rate Input Processing: Convert pilot yaw stick input to desired yaw rate
    //
    // Unlike roll and pitch which are angle targets, yaw is controlled as a rate command.
    // This provides more intuitive control for underwater vehicles where heading hold
    // is desired when the stick is centered.
    //
    // Step 1: Convert PWM input to angle with deadzone and trim compensation
    // The deadzone prevents small stick movements or drift from causing rotation
    float yaw_input = channel_yaw->pwm_to_angle_dz_trim(channel_yaw->get_dead_zone() * sub.gain, channel_yaw->get_radio_trim());
    
    // Step 2: Scale the processed input to desired yaw rate in centidegrees/second
    // This applies the yaw rate limits and input scaling for smooth control
    float target_yaw_rate = sub.get_pilot_desired_yaw_rate(yaw_input);

    // Attitude Controller Integration: Send desired attitudes and rates to the attitude controller
    //
    // The attitude controller is called with either:
    // 1. Euler angles (roll, pitch) + yaw rate - when pilot is actively rotating
    // 2. Euler angles (roll, pitch, yaw) - when holding a fixed heading
    //
    // This dual-mode approach provides intuitive control: rate control during active
    // maneuvering and heading hold when the stick is released.

    // Branch 1: Active Yaw Control - Pilot is commanding rotation
    if (!is_zero(target_yaw_rate)) { 
        // Call attitude controller with rate yaw determined by pilot input
        // Roll and pitch are controlled as angles, yaw is controlled as rate
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_cd(target_roll, target_pitch, target_yaw_rate);
        
        // Update the heading reference to current heading
        // This will be used as the target when pilot releases the yaw stick
        sub.last_pilot_heading = ahrs.yaw_sensor;
        
        // Record timestamp of last pilot yaw input for deceleration logic
        sub.last_pilot_yaw_input_ms = tnow; // time when pilot last changed heading

    } else { // Branch 2: Heading Hold - Pilot yaw stick is centered

        // Deceleration Period: Prevent heading bounce-back after fast yaw maneuvers
        //
        // When pilot releases the yaw stick after a rapid rotation, the vehicle's
        // rotational inertia causes it to continue rotating slightly. If we immediately
        // lock to a fixed heading, the attitude controller will fight this inertia,
        // causing an unnatural "bounce back" to the heading at the moment input stopped.
        //
        // Solution: After yaw input stops, give the vehicle 250ms to naturally decelerate
        // before locking to a fixed heading target.
        
        if (tnow < sub.last_pilot_yaw_input_ms + 250) { 
            // Deceleration phase: 0-250ms after pilot released yaw stick
            // Command zero yaw rate to let the vehicle naturally slow down
            target_yaw_rate = 0;  // Stop rotation on yaw axis

            // Call attitude controller with zero yaw rate to actively decelerate
            // This engages yaw damping without a fixed heading target
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_cd(target_roll, target_pitch, target_yaw_rate);
            
            // Continuously update heading reference during deceleration
            // This tracks where the vehicle naturally stops rotating
            sub.last_pilot_heading = ahrs.yaw_sensor; // update heading to hold

        } else { 
            // Heading Hold phase: More than 250ms after pilot released yaw stick
            // Now lock to the absolute bearing captured at the end of deceleration
            //
            // Call attitude controller with fixed heading target
            // The 'true' parameter indicates this is an earth-frame yaw reference
            attitude_control->input_euler_angle_roll_pitch_yaw_cd(target_roll, target_pitch, sub.last_pilot_heading, true);
        }
    }

    // Throttle Output: Direct throttle control for depth changes
    //
    // In Stabilize mode, pilot throttle input is passed directly to the motors without
    // closed-loop depth control. This provides immediate response and intuitive control.
    //
    // The throttle input is normalized from RC input range (-1.0 to +1.0) to motor
    // output range (0.0 to 1.0) where:
    //   - 0.0 = full downward thrust (sink)
    //   - 0.5 = neutral (maintain depth if properly trimmed)
    //   - 1.0 = full upward thrust (ascend)
    //
    // The transform: (norm_input + 1.0) / 2.0 converts from [-1,+1] to [0,1]
    //
    // Parameters:
    //   - throttle_in: normalized throttle value (0.0 to 1.0)
    //   - false: do not apply angle boost (not needed for underwater vehicles)
    //   - g.throttle_filt: throttle filter frequency for smoothing
    attitude_control->set_throttle_out((channel_throttle->norm_input() + 1.0f) / 2.0f, false, g.throttle_filt);

    // Forward and Lateral Thrust Control: Direct control of horizontal translation
    //
    // These inputs control the horizontal thrusters independently of attitude control,
    // allowing the vehicle to translate (move sideways or forward/back) while maintaining
    // its attitude. This is essential for precise positioning and maneuvering.
    //
    // norm_input() returns normalized values in range -1.0 to +1.0:
    //   Forward: -1.0 = full backward, 0.0 = stop, +1.0 = full forward
    //   Lateral: -1.0 = full left, 0.0 = stop, +1.0 = full right
    //
    // These values are passed directly to the motor mixer which distributes the thrust
    // commands to the appropriate horizontal thrusters based on the vehicle's motor
    // configuration.
    //
    // Note: control_in is range -1000 to 1000 (used internally)
    //       radio_in is raw PWM value (used for low-level RC input)
    motors.set_forward(channel_forward->norm_input());
    motors.set_lateral(channel_lateral->norm_input());
}
