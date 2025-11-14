/**
 * @file mode_follow.cpp
 * @brief Implementation of Follow mode for Rover
 * 
 * @details Follow mode enables the rover to autonomously track and maintain position
 *          relative to another vehicle (lead vehicle) or beacon. The rover receives
 *          position updates from the target via MAVLink GLOBAL_POSITION_INT messages
 *          or beacon positioning systems, then uses proportional control to maintain
 *          a configured distance and bearing offset from the target.
 *          
 *          The follow algorithm integrates with the AP_Follow library which handles:
 *          - Target position tracking and velocity estimation
 *          - Configurable position offsets (distance and bearing)
 *          - MAVLink message parsing for target updates
 *          - Support for beacon-based positioning systems
 *          
 *          Control Strategy:
 *          - Proportional position controller generates desired velocity vector
 *          - Desired velocity = target velocity + position error * kP
 *          - Speed limiting applied to respect configured maximum follow speed
 *          - Heading calculated from desired velocity direction
 *          - Standard steering and throttle controllers track desired heading/speed
 *          
 *          Follow Parameters (configured via AP_Follow library):
 *          - FOLL_ENABLE: Enable/disable follow mode
 *          - FOLL_SYSID: MAVLink system ID of target vehicle
 *          - FOLL_DIST_MAX: Maximum distance to target (meters)
 *          - FOLL_OFS_X: Offset north of target (meters)
 *          - FOLL_OFS_Y: Offset east of target (meters)
 *          - FOLL_OFS_TYPE: Offset type (relative to target heading or absolute)
 *          - FOLL_POS_P: Proportional gain for position tracking
 *          
 * @note This mode is conditionally compiled based on MODE_FOLLOW_ENABLED feature flag.
 *       The mode will fail to enter if AP_Follow is not enabled via FOLL_ENABLE parameter.
 *       
 * @warning Follow mode requires continuous position updates from the target. Loss of
 *          target position data will cause the rover to stop until updates resume.
 *          
 * @see libraries/AP_Follow for target tracking implementation
 * @see Rover/mode.h for base Mode class interface
 * 
 * Source: Rover/mode_follow.cpp
 */

#include "Rover.h"

#if MODE_FOLLOW_ENABLED

/**
 * @brief Initialize Follow mode
 * 
 * @details Called when entering Follow mode. Verifies that the AP_Follow library
 *          is enabled via FOLL_ENABLE parameter before allowing mode entry.
 *          Initializes desired speed to the default waypoint navigation speed.
 *          
 * @return true if mode successfully entered (AP_Follow enabled), false otherwise
 * 
 * @note If AP_Follow is not enabled, this will return false and prevent mode entry.
 */
bool ModeFollow::_enter()
{
    if (!g2.follow.enabled()) {
        return false;
    }

    // initialise speed to waypoint speed
    _desired_speed = g2.wp_nav.get_default_speed();

    return true;
}

/**
 * @brief Exit Follow mode
 * 
 * @details Called when exiting Follow mode. Clears any temporary position offsets
 *          that may have been set in the AP_Follow library if offset clearing is
 *          configured (based on FOLL_OFS_TYPE parameter).
 *          
 * @note This ensures the AP_Follow library returns to its default configuration
 *       when Follow mode is not active.
 */
void ModeFollow::_exit()
{
    g2.follow.clear_offsets_if_required();
}

/**
 * @brief Main update loop for Follow mode
 * 
 * @details This function is called at the main loop rate (typically 50Hz for Rover).
 *          It implements the follow algorithm using proportional control to track
 *          the target vehicle while maintaining configured distance and bearing offsets.
 *          
 *          Algorithm Flow:
 *          1. Verify speed estimate available (required for control)
 *          2. Get target position, velocity from AP_Follow library
 *          3. Calculate desired velocity using proportional control:
 *             desired_vel = target_vel + position_error * kP
 *          4. Apply speed limiting to respect maximum follow speed
 *          5. Calculate desired heading from velocity vector
 *          6. Run steering and throttle controllers to track desired heading/speed
 *          
 *          Target Position Sources (via AP_Follow library):
 *          - MAVLink GLOBAL_POSITION_INT messages from lead vehicle
 *          - Beacon positioning systems
 *          - External vision/motion capture systems
 *          
 * @note This function will stop the vehicle if:
 *       - Speed estimation is unavailable
 *       - Target position data is unavailable
 *       - Calculated desired velocity is zero
 *       
 * @warning Loss of target position updates will cause the rover to stop. Ensure
 *          reliable communication with the lead vehicle or beacon system.
 */
void ModeFollow::update()
{
    // Verify forward speed estimation is available before attempting control
    // Speed estimate is required for accurate throttle and steering control
    float speed;
    if (!attitude_control.get_forward_speed(speed)) {
        // No valid speed estimate - cannot safely control vehicle, so stop
        g2.motors.set_throttle(0.0f);
        g2.motors.set_steering(0.0f);
        return;
    }

    // Position and velocity vectors in NED (North-East-Down) earth frame
    Vector3f dist_vec;      // Vector from rover to lead vehicle (meters)
    Vector3f dist_vec_offs; // Vector from rover to desired offset position relative to lead vehicle (meters)
    Vector3f vel_of_target; // Velocity of lead vehicle (m/s in NED frame)

    // Retrieve target position and velocity from AP_Follow library
    // The AP_Follow library tracks the target vehicle using:
    // - MAVLink GLOBAL_POSITION_INT messages (most common)
    // - Beacon positioning system measurements
    // - External positioning system data
    // Returns false if target data is unavailable or too old
    if (!g2.follow.get_target_dist_and_vel_NED_m(dist_vec, dist_vec_offs, vel_of_target)) {
        // No valid target position available - stop vehicle and wait for updates
        _reached_destination = true;
        stop_vehicle();
        return;
    }

    // Calculate desired velocity vector using proportional control
    // Algorithm: desired_velocity = target_velocity + position_error * kP
    // This creates a "follow" behavior where the rover:
    // - Matches the target's velocity when at the desired offset position
    // - Accelerates to catch up when behind the desired position
    // - Decelerates when ahead of the desired position
    Vector2f desired_velocity_ne;  // Desired velocity in North-East components (m/s)
    const float kp = g2.follow.get_pos_p().kP();  // Proportional gain from FOLL_POS_P parameter
    desired_velocity_ne.x = vel_of_target.x + (dist_vec_offs.x * kp);  // North component
    desired_velocity_ne.y = vel_of_target.y + (dist_vec_offs.y * kp);  // East component

    // Check if desired velocity is zero (rover at correct position relative to stationary target)
    if (is_zero(desired_velocity_ne.x) && is_zero(desired_velocity_ne.y)) {
        // Desired position achieved with zero velocity - stop and hold position
        _reached_destination = true;
        stop_vehicle();
        return;
    }

    // Rover has non-zero desired velocity, so destination not yet reached
    _reached_destination = false;

    // Apply speed limiting to respect maximum follow speed
    // This prevents the rover from exceeding configured speed limits while following
    // The speed limit is set by set_desired_speed() or defaults to waypoint navigation speed
    float desired_speed = safe_sqrt(sq(desired_velocity_ne.x) + sq(desired_velocity_ne.y));  // Magnitude of velocity vector (m/s)
    if (!is_zero(desired_speed) && (desired_speed > _desired_speed)) {
        // Scale velocity vector down to maximum allowed speed while preserving direction
        const float scalar_xy = _desired_speed / desired_speed;
        desired_velocity_ne *= scalar_xy;
        desired_speed = _desired_speed;
    }

    // Calculate desired heading from velocity vector direction
    // Heading is calculated from the desired velocity direction in the NE plane
    // atan2(y, x) gives angle from east axis, which corresponds to rover heading
    // Result is wrapped to ±180° and converted to centidegrees
    const float desired_yaw_cd = wrap_180_cd(rad_to_cd(atan2f(desired_velocity_ne.y, desired_velocity_ne.x)));

    // Execute steering and throttle control to track desired heading and speed
    // calc_steering_to_heading: L1 or steering rate controller drives heading to desired_yaw_cd
    // calc_throttle: Speed controller adjusts throttle to achieve desired_speed
    // The true parameter enables active throttle control (not coasting)
    calc_steering_to_heading(desired_yaw_cd);
    calc_throttle(desired_speed, true);
}

/**
 * @brief Return desired heading to target for ground station reporting
 * 
 * @details Returns the bearing from the rover's current position to the target
 *          vehicle or beacon. This value is reported to the ground control station
 *          in the NAV_CONTROLLER_OUTPUT MAVLink message to display follow mode
 *          navigation information.
 *          
 * @return Bearing to target in degrees (0-360°, where 0° is north)
 * 
 * @note This is a reporting function only and does not affect control behavior.
 *       The actual desired heading used for control is calculated in update()
 *       based on the desired velocity vector.
 */
float ModeFollow::wp_bearing() const
{
    return g2.follow.get_bearing_to_target_deg();
}

/**
 * @brief Return distance to target for ground station reporting
 * 
 * @details Returns the horizontal distance from the rover's current position to
 *          the target vehicle or beacon. This value is reported to the ground
 *          control station for monitoring follow mode progress and is displayed
 *          as "WP Dist" or "Distance to Destination" in ground station software.
 *          
 * @return Distance to target in meters
 * 
 * @note This reports distance to the actual target position, not to the desired
 *       offset position. The rover aims to maintain the configured offset distance
 *       (FOLL_OFS_X, FOLL_OFS_Y) from this target position.
 */
float ModeFollow::get_distance_to_destination() const
{
    return g2.follow.get_distance_to_target_m();
}

/**
 * @brief Set maximum follow speed
 * 
 * @details Sets the maximum speed the rover will use while following the target.
 *          This speed limit is enforced in the update() function by scaling down
 *          the desired velocity vector if it exceeds this limit. This can be used
 *          to limit follow speed below the target's speed for safety or to prevent
 *          aggressive acceleration when catching up to a fast-moving target.
 *          
 * @param[in] speed Desired maximum speed in m/s (must be non-negative)
 * 
 * @return true if speed successfully set, false if speed is negative
 * 
 * @note The default speed on mode entry is the waypoint navigation speed
 *       (WP_SPEED parameter). This function allows dynamic speed changes
 *       during follow mode operation via MAVLink commands or scripts.
 *       
 * @warning Setting speed to zero will cause the rover to stop and remain
 *          stopped until a positive speed is set. Negative speeds are rejected.
 */
bool ModeFollow::set_desired_speed(float speed)
{
    if (is_negative(speed)) {
        return false;
    }
    _desired_speed = speed;
    return true;
}

#endif // MODE_FOLLOW_ENABLED
