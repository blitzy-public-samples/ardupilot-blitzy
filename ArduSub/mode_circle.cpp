#include "Sub.h"

/**
 * @file mode_circle.cpp
 * @brief Circle flight mode implementation for ArduSub
 * 
 * @details This file implements the Circle flight mode for underwater vehicles,
 *          enabling autonomous circular loiter patterns at a specified depth.
 *          Circle mode uses the AC_Circle navigation controller to maintain
 *          a circular flight path around a center point, useful for:
 *          - Inspection missions requiring circular observation patterns
 *          - Surveying cylindrical structures (pipes, pillars, etc.)
 *          - Creating circular search patterns
 *          - Maintaining station keeping with orbital motion
 *          
 *          The circle center is initialized based on the vehicle's current
 *          position and velocity vector. The vehicle maintains depth control
 *          independently while following the circular horizontal path.
 *          
 *          Key Components:
 *          - AC_Circle (circle_nav): Circular path navigation controller
 *          - AC_PosControl_Sub (position_control): Horizontal position and vertical depth control
 *          - AC_AttitudeControl_Sub (attitude_control): Vehicle attitude stabilization
 *          
 * @note Circle mode requires a valid position estimate to initialize
 * @note The pilot can override yaw control during circular navigation
 * @see AC_Circle for circular navigation algorithm details
 * @see AC_PosControl_Sub for position control implementation
 */

/**
 * @brief Initialize Circle mode and configure navigation parameters
 * 
 * @details Sets up the Circle mode by configuring speed and acceleration limits
 *          for both horizontal (North-East) and vertical (Up) movement, then
 *          initializes the circular navigation controller. The circle center
 *          is automatically determined based on the vehicle's current position
 *          and velocity vector.
 *          
 *          Initialization sequence:
 *          1. Verify position estimate is valid (GPS, visual odometry, etc.)
 *          2. Reset pilot yaw override flag
 *          3. Configure horizontal speed/acceleration from waypoint navigation parameters
 *          4. Configure vertical speed/acceleration from pilot control parameters
 *          5. Initialize circle_nav controller (sets center point and initial heading)
 *          
 *          Speed and acceleration parameters are derived from:
 *          - Horizontal: wp_nav default speed (WPNAV_SPEED) and acceleration (WPNAV_ACCEL)
 *          - Vertical: pilot speed up/down (PILOT_SPEED_UP, PILOT_SPEED_DN) and accel (PILOT_ACCEL_Z)
 * 
 * @param[in] ignore_checks Currently unused in ArduSub Circle mode implementation
 * 
 * @return true if initialization successful and mode can be entered
 * @return false if position estimate is not valid, preventing mode entry
 * 
 * @note This function is called when the pilot switches to Circle mode
 * @note Requires valid position estimate from EKF (GPS, DVL, visual odometry, etc.)
 * @warning Mode will fail to initialize if underwater navigation sensors are not available
 * 
 * @see sub.circle_nav.init() for circle controller initialization
 * @see sub.position_ok() for position estimate validation
 */
bool ModeCircle::init(bool ignore_checks)
{
    // Verify we have a valid position estimate from navigation sensors
    // (GPS, DVL, visual odometry, or other underwater positioning)
    if (!sub.position_ok()) {
        return false;
    }

    // Reset yaw override flag - initially follow circle path heading
    // Pilot can override yaw with stick input during flight
    sub.circle_pilot_yaw_override = false;

    // Configure horizontal (North-East plane) speed and acceleration limits
    // Uses waypoint navigation parameters for consistent autonomous behavior
    // Speed in cm/s, acceleration in cm/s/s
    position_control->set_max_speed_accel_NE_cm(sub.wp_nav.get_default_speed_NE_cms(), sub.wp_nav.get_wp_acceleration_cmss());
    position_control->set_correction_speed_accel_NE_cm(sub.wp_nav.get_default_speed_NE_cms(), sub.wp_nav.get_wp_acceleration_cmss());
    
    // Configure vertical (Up axis) speed and acceleration limits
    // Uses pilot speed parameters: negative for down, positive for up
    // Allows pilot to control depth during circular pattern
    position_control->set_max_speed_accel_U_cm(-sub.get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    position_control->set_correction_speed_accel_U_cmss(-sub.get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // Initialize circular navigation controller
    // Sets circle center based on current position and velocity vector
    // Circle radius and rate are determined by CIRCLE_RADIUS and CIRCLE_RATE parameters
    sub.circle_nav.init();

    return true;
}

/**
 * @brief Execute Circle mode control loop
 * 
 * @details Runs the main Circle mode control loop, managing circular horizontal
 *          navigation while maintaining depth control and attitude stabilization.
 *          This function integrates multiple control systems:
 *          
 *          Control Loop Architecture:
 *          1. Process pilot inputs (yaw rate, climb rate)
 *          2. Update circle navigation controller (maintains circular path)
 *          3. Translate circular navigation outputs to vehicle forward/lateral commands
 *          4. Control vehicle attitude (roll, pitch, yaw)
 *          5. Control vertical position (depth holding with pilot climb rate input)
 *          
 *          Pilot Control Integration:
 *          - Throttle stick: Controls climb/descent rate (overrides depth hold)
 *          - Yaw stick: Overrides automatic yaw-to-circle-path behavior
 *          - Roll/Pitch sticks: Passed to attitude controller (typically centered)
 *          
 *          Circle Navigation:
 *          - Radius: Defined by CIRCLE_RADIUS parameter (cm)
 *          - Rate: Defined by CIRCLE_RATE parameter (deg/s, positive=clockwise)
 *          - Center: Set during init() based on initial position/velocity
 *          - Path following uses AC_Circle navigation controller
 *          
 *          Depth Control:
 *          - Maintains current depth when throttle stick centered
 *          - Responds to pilot climb rate commands
 *          - Integrates with terrain failsafe system
 * 
 * @note This function must be called at 100Hz or higher for stable control
 * @note Speed and acceleration parameters can be changed at runtime
 * @warning Disarmed state immediately disables motors and relaxes attitude control
 * 
 * @see AC_Circle::update_cms() for circular path navigation
 * @see AC_PosControl_Sub::update_U_controller() for depth control
 * @see AC_AttitudeControl_Sub for attitude stabilization
 */
void ModeCircle::run()
{
    float target_yaw_rate = 0;      // Desired yaw rotation rate from pilot (deg/s)
    float target_climb_rate = 0;    // Desired vertical velocity from pilot (cm/s, positive=up)

    // Update speed and acceleration parameters to allow runtime adjustment
    // This enables parameter tuning without mode exit/re-entry
    // Horizontal (NE plane) parameters control circle path following smoothness
    position_control->set_max_speed_accel_NE_cm(sub.wp_nav.get_default_speed_NE_cms(), sub.wp_nav.get_wp_acceleration_cmss());
    // Vertical (U axis) parameters control depth change responsiveness
    position_control->set_max_speed_accel_U_cm(-sub.get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // Safety check: if vehicle is disarmed, disable all control outputs
    if (!motors.armed()) {
        // Set motors to ground idle state (minimal power, not spinning)
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        // Disable attitude stabilization when disarmed (ArduSub convention)
        attitude_control->set_throttle_out(0,true,g.throttle_filt);
        attitude_control->relax_attitude_controllers();
        // Reset circle navigation to re-initialize on next arming
        sub.circle_nav.init();
        return;
    }

    ///////////////////////////
    // Process Pilot Inputs //
    ///////////////////////////
    
    // Get pilot's desired yaw rate from yaw stick input
    // Stick deflection maps to rotation rate in deg/s based on ACRO_YAW_P parameter
    target_yaw_rate = sub.get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
    
    // If pilot provides yaw input, override automatic yaw-to-circle behavior
    // This allows pilot to look around while maintaining circular path
    if (!is_zero(target_yaw_rate)) {
        sub.circle_pilot_yaw_override = true;
    }

    // Get pilot's desired climb rate from throttle stick input
    // Stick deflection maps to vertical velocity based on PILOT_SPEED_UP/DN parameters
    // Centered stick = hold current depth, up = ascend, down = descend
    target_climb_rate = sub.get_pilot_desired_climb_rate(channel_throttle->get_control_in());

    // Enable full motor output range for active flight
    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    ////////////////////////////////////////////
    // Update Circular Navigation Controller //
    ////////////////////////////////////////////
    
    // Execute circle navigation update - calculates desired position on circular path
    // Uses CIRCLE_RADIUS (cm) and CIRCLE_RATE (deg/s) parameters
    // Returns true if navigation update successful, false on terrain failsafe trigger
    // Updates internal circle state: angle, target position, velocity
    sub.failsafe_terrain_set_status(sub.circle_nav.update_cms());

    ////////////////////////////////////////////////////
    // Translate Circle Navigation to Motor Commands //
    ////////////////////////////////////////////////////
    
    // Convert circle navigation desired velocities (North-East frame)
    // to vehicle body frame lateral and forward commands
    // Accounts for vehicle yaw angle and frame transformation
    float lateral_out, forward_out;
    sub.translate_circle_nav_rp(lateral_out, forward_out);

    // Apply horizontal velocity commands to lateral and forward thrusters
    // These outputs maintain the circular path in the horizontal plane
    motors.set_lateral(lateral_out);
    motors.set_forward(forward_out);

    /////////////////////////////////////
    // Attitude Control (Yaw Handling) //
    /////////////////////////////////////
    
    // Control vehicle attitude based on pilot yaw input mode
    if (sub.circle_pilot_yaw_override) {
        // Pilot is controlling yaw rate - use rate controller
        // Allows manual yaw rotation while following circle path
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_cd(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_yaw_rate);
    } else {
        // Automatic yaw pointing - vehicle faces tangent to circle path
        // get_yaw_cd() returns target heading to maintain circular motion orientation
        attitude_control->input_euler_angle_roll_pitch_yaw_cd(channel_roll->get_control_in(), channel_pitch->get_control_in(), sub.circle_nav.get_yaw_cd(), true);
    }

    /////////////////////////////////////////////////////////////
    // Depth Control (Vertical Position Hold with Pilot Input) //
    /////////////////////////////////////////////////////////////
    
    // Update vertical position target based on pilot climb rate input
    // If climb rate is zero (stick centered), maintains current depth
    // If climb rate is non-zero, adjusts depth at commanded rate
    position_control->set_pos_target_U_from_climb_rate_cm(target_climb_rate);
    
    // Execute vertical position controller to maintain/adjust depth
    // Computes throttle commands to achieve target vertical velocity
    // Integrates with EKF vertical position estimate and barometer/depth sensor
    position_control->update_U_controller();
}
