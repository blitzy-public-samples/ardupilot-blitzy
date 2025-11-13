/**
 * @file AP_ExternalControl_Rover.cpp
 * @brief Implementation of external control library for ArduRover
 * 
 * @details This file implements the rover-specific interface for external control systems
 *          (e.g., offboard computers, companion systems) to command the vehicle. It translates
 *          external velocity/position commands into rover control setpoints and integrates with
 *          the rover guided mode control system.
 * 
 *          Key responsibilities:
 *          - Accept velocity commands in earth frame (NED) and convert to body frame (FRD)
 *          - Accept global position targets for waypoint navigation
 *          - Enforce safety checks (guided mode, armed state) before accepting commands
 *          - Bridge external systems with rover's internal guided mode controller
 * 
 * @note External control is only active when vehicle is in guided mode and armed
 */


#include "AP_ExternalControl_Rover.h"
#if AP_EXTERNAL_CONTROL_ENABLED

#include "Rover.h"

/**
 * @brief Set linear velocity and yaw rate for external control
 * 
 * @details This function accepts velocity commands from external control sources and translates
 *          them into rover control setpoints. The implementation performs coordinate frame
 *          transformation from earth NED frame to body FRD frame and converts yaw rate from
 *          radians/second to centidegrees/second for internal control system compatibility.
 * 
 *          Processing steps:
 *          1. Verify vehicle is ready (guided mode + armed)
 *          2. Transform velocity from earth frame (NED) to body frame (FRD) using AHRS
 *          3. Extract forward velocity (body.x) - lateral velocity ignored for standard rovers
 *          4. Convert yaw rate: rad/s → deg/s → centideg/s (factor of 100)
 *          5. Send commands to guided mode controller
 * 
 * @param[in] linear_velocity Desired velocity in earth frame (NED), m/s
 *                             - North component: linear_velocity.x
 *                             - East component: linear_velocity.y
 *                             - Down component: linear_velocity.z (typically ignored for ground vehicles)
 * @param[in] yaw_rate_rads   Desired yaw rate in rad/s (positive = clockwise when viewed from above)
 *                             Pass NaN to not control yaw (maintains current heading)
 * 
 * @return true if command accepted and applied, false if vehicle not ready for external control
 * 
 * @note Rover is commanded in body-frame using FRD (Forward-Right-Down) convention
 * @note Only forward velocity (body.x) is used; lateral velocity is ignored for standard rovers
 * @note Uses rover.mode_guided.set_desired_turn_rate_and_speed() for actual control
 * @note Yaw rate conversion: rad/s * (180/π) * 100 = centidegrees/s
 * @warning Vehicle must be in guided mode and armed for commands to be accepted
 * 
 * @see ready_for_external_control()
 * @see Rover::Mode::Guided::set_desired_turn_rate_and_speed()
 */
bool AP_ExternalControl_Rover::set_linear_velocity_and_yaw_rate(const Vector3f &linear_velocity, float yaw_rate_rads)
{
    // Safety check: only accept commands when in guided mode and armed
    if (!ready_for_external_control()) {
        return false;
    }

    // Rover is commanded in body-frame using FRD convention
    // Transform velocity from earth frame (NED) to body frame (FRD)
    auto &ahrs = AP::ahrs();
    Vector3f linear_velocity_body = ahrs.earth_to_body(linear_velocity);

    // Extract forward velocity (body-frame x-axis). Lateral velocity ignored for standard rovers
    const float target_speed = linear_velocity_body.x;  // m/s
    
    // Convert yaw rate from rad/s to centidegrees/s (internal unit)
    // NaN handling: if yaw_rate_rads is NaN, use 0 (no turn command - maintain heading)
    const float turn_rate_cds = isnan(yaw_rate_rads)? 0: degrees(yaw_rate_rads)*100;  // centidegrees/s

    // Send velocity and turn rate commands to guided mode controller
    rover.mode_guided.set_desired_turn_rate_and_speed(turn_rate_cds, target_speed);
    return true;
}

/**
 * @brief Set global position target for external control
 * 
 * @details This function accepts a global position target (latitude, longitude, altitude) from
 *          external control sources and delegates to the rover's waypoint navigation system.
 *          The function internally checks if the rover is in guided mode before accepting
 *          the position target.
 * 
 *          The rover will navigate to the specified location using its standard waypoint
 *          following algorithm, which includes path planning, obstacle avoidance (if configured),
 *          and speed management.
 * 
 * @param[in] loc Global position target
 *                - loc.lat: Target latitude in degrees * 1e7
 *                - loc.lng: Target longitude in degrees * 1e7
 *                - loc.alt: Target altitude in cm (may be ignored for ground vehicles)
 * 
 * @return true if target accepted, false if vehicle not in guided mode
 * 
 * @note Altitude component may be ignored for ground vehicles following terrain
 * @note This function delegates to rover.set_target_location() which performs the guided mode check
 * @note The rover will use its configured navigation parameters (speed, turn rate limits) when navigating to target
 * @warning Vehicle must be in guided mode for position target to be accepted
 * 
 * @see Rover::set_target_location()
 * @see ready_for_external_control()
 */
bool AP_ExternalControl_Rover::set_global_position(const Location& loc)
{
    // Delegate to rover's set_target_location which checks if rover is in guided mode
    // and sets up waypoint navigation to the specified global position
    return rover.set_target_location(loc);
}

/**
 * @brief Check if vehicle is ready to accept external control commands
 * 
 * @details This function verifies that the vehicle is in the appropriate state to safely
 *          accept external control commands. Two conditions must be met:
 *          1. Vehicle must be in guided mode (prevents external control in manual modes)
 *          2. Vehicle must be armed (prevents commands when vehicle is not ready to move)
 * 
 *          This safety check is called at the beginning of all external control command
 *          functions to ensure commands are only processed when the vehicle is in the
 *          correct operational state.
 * 
 * @return true if vehicle is in guided mode AND armed, false otherwise
 * 
 * @note This is a safety check to prevent external control in inappropriate modes
 * @note External control commands will be rejected if either condition is not met
 * @warning Do not bypass this check - it prevents unsafe operation
 * 
 * @see set_linear_velocity_and_yaw_rate()
 * @see set_global_position()
 */
bool AP_ExternalControl_Rover::ready_for_external_control()
{
    // Check both required conditions:
    // 1. Vehicle must be in guided mode (external control only works in guided)
    // 2. Vehicle must be armed (safety check - prevents commands when disarmed)
    return rover.control_mode->in_guided_mode() && rover.arming.is_armed();
}

#endif // AP_EXTERNAL_CONTROL_ENABLED
