/**
 * @file mode_poshold.cpp
 * @brief ArduSub Position Hold flight mode implementation
 * 
 * @details This file implements the Position Hold mode for underwater vehicles,
 *          which maintains the vehicle's position using positioning sensors such as
 *          Doppler Velocity Log (DVL), visual odometry, or other underwater positioning
 *          systems integrated through the inertial navigation system.
 *          
 *          Position Hold mode allows the vehicle to maintain its current position
 *          underwater while the pilot can provide velocity commands to reposition.
 *          When pilot inputs are released, the vehicle will brake and lock to the
 *          new position. This mode is particularly useful for station-keeping,
 *          inspection tasks, and precise maneuvering in underwater environments.
 *          
 *          Key Features:
 *          - Automatic position hold using inertial navigation and positioning sensors
 *          - Pilot inputs create velocity requests in the body frame
 *          - Automatic brake and position lock when inputs are released
 *          - Integrated depth control with horizontal position hold
 *          - Yaw hold with manual override capability
 *          
 * @note Position Hold mode requires a valid position estimate from the inertial
 *       navigation system. This typically requires DVL (Doppler Velocity Log),
 *       visual odometry, or other underwater positioning sensors. GPS does not
 *       work underwater and cannot be used for this mode in ArduSub.
 *       
 * @warning This is a safety-critical flight mode. Position hold accuracy depends
 *          entirely on the quality and availability of positioning sensor data.
 *          Loss of positioning during operation will cause the mode to fall back
 *          to manual velocity control if pilot_speed > 0.
 * 
 * @author Jacob Walser August 2016
 * @copyright Copyright (c) 2016-2025 ArduPilot.org
 * 
 * Source: ArduSub/mode_poshold.cpp
 */

#include "Sub.h"

#if POSHOLD_ENABLED

/**
 * @brief Initialize Position Hold mode controller
 * 
 * @details This function initializes the Position Hold mode by setting up the
 *          position and attitude controllers with appropriate limits and initializing
 *          the current position as the hold target. The initialization sequence:
 *          
 *          1. Verify position estimate is available (position_ok())
 *          2. Configure NE (North-East) controller speed and acceleration limits
 *          3. Configure U (Up/Down) controller for depth control
 *          4. Initialize position controller with current stopping point
 *          5. Stop all thrusters and relax attitude controllers
 *          6. Store current heading for yaw hold
 *          
 *          The position controller uses the inertial navigation system's position
 *          estimate, which is derived from sensor fusion of DVL, visual odometry,
 *          pressure sensors, and IMU data.
 * 
 * @param[in] ignore_checks If true, skip pre-flight checks (currently unused)
 * 
 * @return true if initialization successful and mode can be entered
 * @return false if position estimate unavailable (mode cannot be entered)
 * 
 * @note Requires valid position estimate from inertial navigation system.
 *       Position estimate typically comes from DVL (Doppler Velocity Log),
 *       visual odometry (camera-based), or other underwater positioning sensors.
 *       
 * @note The NE (North-East) frame refers to horizontal position in earth frame,
 *       while U refers to the up/down axis for depth control.
 *       
 * @warning Initialization will fail if sub.position_ok() returns false, which
 *          indicates insufficient sensor data for position estimation. This is
 *          a safety feature to prevent entering position hold without valid
 *          position feedback.
 * 
 * @see AC_PosControl::init_NE_controller_stopping_point()
 * @see AC_PosControl::init_U_controller()
 * @see Sub::position_ok()
 */
bool ModePoshold::init(bool ignore_checks)
{
    // Fail to initialize PosHold mode if no valid position estimate
    // Note: Despite legacy comment, this is NOT GPS-based (GPS doesn't work underwater)
    // Position estimate comes from DVL, visual odometry, or other underwater sensors
    if (!sub.position_ok()) {
        return false;
    }

    // Configure position controller speed and acceleration limits
    // NE (North-East / horizontal) controller uses pilot_speed and pilot_accel_z parameters
    position_control->set_max_speed_accel_NE_cm(g.pilot_speed, g.pilot_accel_z);
    position_control->set_correction_speed_accel_NE_cm(g.pilot_speed, g.pilot_accel_z);
    
    // U (Up/Down / vertical) controller uses asymmetric speed limits for depth control
    // Down speed is typically limited more than up speed for safety
    position_control->set_max_speed_accel_U_cm(-sub.get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    position_control->set_correction_speed_accel_U_cmss(-sub.get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // Initialize position controller with current position as target
    // NE controller uses stopping point algorithm to account for current velocity
    position_control->init_NE_controller_stopping_point();
    position_control->init_U_controller();

    // Stop all thrusters and relax controllers for smooth mode entry
    // Set throttle to neutral (0.5) to prevent sudden depth changes
    attitude_control->set_throttle_out(0.5f ,true, g.throttle_filt);
    attitude_control->relax_attitude_controllers();
    position_control->relax_U_controller(0.5f);

    // Store current heading as initial heading hold target
    sub.last_pilot_heading = ahrs.yaw_sensor;

    return true;
}

/**
 * @brief Main Position Hold mode control loop
 * 
 * @details This function executes the complete Position Hold control algorithm,
 *          integrating attitude control, depth control, and horizontal position
 *          control. The control loop implements the following sequence:
 *          
 *          1. **Disarmed Handling**: When motors are not armed, all controllers
 *             are relaxed and thrusters set to idle to prevent unexpected motion.
 *          
 *          2. **Attitude Control**: Processes pilot yaw input with intelligent
 *             heading hold behavior:
 *             - Active yaw input: Vehicle rotates at pilot-commanded rate
 *             - No yaw input: Vehicle holds last commanded heading
 *             - Yaw deceleration: 250ms settle time to prevent overshoot
 *             
 *          3. **Depth Control**: Maintains or adjusts depth based on pilot input
 *             via control_depth() helper function
 *             
 *          4. **Horizontal Position Control**: Maintains horizontal position or
 *             responds to pilot velocity commands via control_horizontal() helper
 *          
 *          **Position Hold Algorithm**:
 *          The position controller uses a cascaded control structure:
 *          - Outer loop: Position error → desired velocity (earth frame)
 *          - Inner loop: Velocity error → desired attitude (roll/pitch)
 *          - Attitude controller converts desired attitude to motor commands
 *          
 *          **Pilot Input Integration**:
 *          - Pilot stick inputs are converted to velocity requests in body frame
 *          - Position controller integrates velocity requests to update target position
 *          - When inputs are released, controller brakes and locks new position
 *          - Yaw control is independent with heading hold functionality
 * 
 * @note This function should be called at 100Hz or higher for stable control.
 *       Lower rates may result in degraded position hold performance or instability.
 *       
 * @note The 250ms yaw deceleration delay (line 79) prevents heading "bounce back"
 *       after rapid yaw maneuvers, accounting for vehicle rotational inertia.
 *       
 * @note Position hold accuracy depends on the quality of the inertial navigation
 *       position estimate, which is derived from DVL, visual odometry, or other
 *       underwater positioning sensors fused with IMU and depth data.
 *       
 * @warning Motors must be armed for active position hold. When disarmed, all
 *          control outputs are set to safe idle states.
 *          
 * @warning Loss of position estimate during operation (position_ok() becomes false)
 *          will cause control_horizontal() to fall back to velocity control mode,
 *          allowing manual repositioning but losing position hold capability.
 * 
 * @see ModePoshold::control_horizontal()
 * @see ModePoshold::control_depth()
 * @see AC_AttitudeControl::input_euler_angle_roll_pitch_euler_rate_yaw_cd()
 * @see AC_PosControl::update_NE_controller()
 */
void ModePoshold::run()
{
    uint32_t tnow = AP_HAL::millis();
    
    // When unarmed, disable motors and stabilization
    // This is a safety feature to prevent unexpected motion when vehicle is not armed
    if (!motors.armed()) {
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        
        // Sub vehicles do not stabilize roll/pitch/yaw when not armed
        // Set throttle to neutral (0.5) and relax all controllers
        attitude_control->set_throttle_out(0.5f ,true, g.throttle_filt);
        attitude_control->relax_attitude_controllers();
        
        // Reset position controller to current location to prevent jump when re-arming
        position_control->init_NE_controller_stopping_point();
        position_control->relax_U_controller(0.5f);
        
        // Store current heading so vehicle doesn't spin when armed
        sub.last_pilot_heading = ahrs.yaw_sensor;
        return;
    }

    // set motors to full range
    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    /////////////////////
    // Update Attitude //
    /////////////////////

    // Get pilot's desired yaw rate from rudder/yaw stick input
    // Applies deadzone and trim settings for smooth control
    float yaw_input = channel_yaw->pwm_to_angle_dz_trim(channel_yaw->get_dead_zone() * sub.gain, channel_yaw->get_radio_trim());
    float target_yaw_rate = sub.get_pilot_desired_yaw_rate(yaw_input);

    // Convert pilot roll/pitch stick inputs to target lean angles
    // These will be used by position controller for horizontal movement
    // @todo Convert get_pilot_desired_lean_angles to return angles as floats
    float target_roll, target_pitch;
    sub.get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, sub.aparm.angle_max);

    // Yaw Control with Heading Hold
    // Implements intelligent yaw behavior: rate control during input, heading hold when released
    if (!is_zero(target_yaw_rate)) {
        // Active yaw input: Control yaw rate while holding target roll/pitch
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_cd(target_roll, target_pitch, target_yaw_rate);
        
        // Update heading reference to current heading
        sub.last_pilot_heading = ahrs.yaw_sensor;
        
        // Record time of last yaw input for deceleration delay logic
        sub.last_pilot_yaw_input_ms = tnow;

    } else {
        // No yaw input: Hold heading with deceleration delay
        
        // Check if we're still within deceleration period after yaw input released
        // This prevents heading "bounce back" due to vehicle rotational inertia
        // The vehicle may continue rotating briefly after input stops
        if (tnow < sub.last_pilot_yaw_input_ms + 250) {
            // Deceleration period (250ms): Command zero yaw rate to brake rotation
            target_yaw_rate = 0;
            
            // Call attitude controller with zero yaw rate to actively stop rotation
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_cd(target_roll, target_pitch, target_yaw_rate);
            
            // Continue updating heading reference during deceleration
            sub.last_pilot_heading = ahrs.yaw_sensor;

        } else {
            // Deceleration complete: Hold absolute heading
            // Switch to heading hold mode using last commanded heading
            attitude_control->input_euler_angle_roll_pitch_yaw_cd(target_roll, target_pitch, sub.last_pilot_heading, true);
        }
    }

    ////////////////////////
    // Update Depth Control //
    ////////////////////////
    
    // Control depth (z-axis / U-axis) based on pilot throttle input
    // Maintains current depth when throttle centered, climbs/descends with input
    control_depth();

    ////////////////////////////
    // Update Horizontal Control //
    ////////////////////////////
    
    // Control horizontal position (NE plane / xy axes)
    // Note: Call this AFTER control_depth() so throttle deadzone is properly calculated
    // Position controller uses pilot roll/pitch inputs as velocity commands
    control_horizontal();
}

/**
 * @brief Control horizontal position hold using pilot velocity inputs
 * 
 * @details This function implements the horizontal (NE plane) position hold control
 *          algorithm, which maintains the vehicle's horizontal position while allowing
 *          pilot inputs to create velocity commands for repositioning. The control
 *          flow implements two distinct modes:
 *          
 *          **Position Hold Mode** (when position_ok() is true):
 *          1. Read pilot inputs for forward/lateral velocity commands (body frame)
 *          2. Transform body frame velocities to earth frame (NE)
 *          3. Feed velocity commands to position controller
 *          4. Position controller generates desired roll/pitch attitudes
 *          5. Convert attitude commands back to forward/lateral thrust outputs
 *          6. Update position controller for next iteration
 *          
 *          **Manual Velocity Mode** (fallback when position_ok() is false):
 *          1. Read pilot velocity inputs in body frame
 *          2. Scale inputs by configured pilot_speed parameter
 *          3. Output directly as forward/lateral thrust commands
 *          4. No position hold - pure velocity control
 *          
 *          **Brake and Position Lock Mechanism**:
 *          When pilot inputs return to neutral (zero velocity command), the position
 *          controller automatically brakes the vehicle and locks to the new position.
 *          The init_NE_controller_stopping_point() function calculates the optimal
 *          stopping position based on current velocity and deceleration limits.
 *          
 *          **Controller Timeout Handling**:
 *          The position controller has an internal timeout mechanism. If the controller
 *          becomes inactive (is_active_NE() returns false), it is re-initialized to
 *          the current stopping point to prevent control discontinuities.
 *          
 * @note Body frame rates are in cm/s: x = forward, y = lateral (right positive)
 * @note Earth frame rates are in cm/s: N = north, E = east (NE frame)
 * @note Output range is normalized [-1.0, 1.0] for motor mixer
 * 
 * @note The position controller uses a cascaded PID control structure:
 *       Position Error → Velocity Command → Acceleration Command → Attitude Command
 *       
 * @note Position hold requires continuous position updates from the inertial
 *       navigation system. The system fuses DVL (Doppler Velocity Log), visual
 *       odometry, depth sensors, and IMU data to maintain accurate position estimate.
 *       
 * @warning If position estimate is lost (position_ok() returns false), the mode
 *          falls back to manual velocity control. The vehicle will NOT hold position
 *          in this fallback mode and will drift with currents.
 *          
 * @warning Controller re-initialization may cause small transients if the controller
 *          times out during operation. This is a safety feature to prevent runaway
 *          control in case of temporary position estimate loss.
 * 
 * @see AC_PosControl::input_vel_accel_NE_cm()
 * @see AC_PosControl::update_NE_controller()
 * @see AC_PosControl::init_NE_controller_stopping_point()
 * @see Sub::translate_pos_control_rp()
 * @see Sub::position_ok()
 */
void ModePoshold::control_horizontal() {
    float lateral_out = 0;
    float forward_out = 0;

    // Get desired velocity rates in the body frame from pilot stick inputs
    // Forward/back on pitch stick, left/right on roll stick
    // Returns velocity in cm/s based on configured pilot_speed parameter
    Vector2f body_rates_cm_s = {
        sub.get_pilot_desired_horizontal_rate(channel_forward),
        sub.get_pilot_desired_horizontal_rate(channel_lateral)
    };

    if (sub.position_ok()) {
        // Position Hold Mode: Active position control with valid position estimate
        
        if (!position_control->is_active_NE()) {
            // The NE (horizontal) controller timed out or was not running
            // Re-initialize to current stopping point to prevent control discontinuity
            // This calculates optimal brake position based on current velocity
            position_control->init_NE_controller_stopping_point();
        }

        // Transform pilot velocity commands from body frame to earth frame (NE)
        // This allows position controller to work in a fixed earth reference frame
        auto earth_rates_cm_s = ahrs.body_to_earth2D(body_rates_cm_s);
        
        // Feed velocity commands to position controller
        // Zero acceleration request allows controller to determine optimal acceleration
        position_control->input_vel_accel_NE_cm(earth_rates_cm_s, {0, 0});

        // Position controller outputs desired roll/pitch angles for horizontal control
        // Translate these back to forward/lateral thrust outputs for the motor mixer
        sub.translate_pos_control_rp(lateral_out, forward_out);

        // Update the NE position controller to calculate next control cycle outputs
        // This executes the cascaded position → velocity → acceleration → attitude control
        position_control->update_NE_controller();
        
    } else if (g.pilot_speed > 0) {
        // Fallback Mode: Manual velocity control when position estimate unavailable
        // Normalize pilot inputs to [-1.0, 1.0] range for direct thrust control
        // Vehicle will drift with currents - no position hold active
        forward_out = body_rates_cm_s.x / (float)g.pilot_speed;
        lateral_out = body_rates_cm_s.y / (float)g.pilot_speed;
    }

    // Send computed forward and lateral outputs to motor mixer
    // Range: [-1.0, 1.0] normalized thrust commands
    motors.set_forward(forward_out);
    motors.set_lateral(lateral_out);
}

#endif  // POSHOLD_ENABLED
