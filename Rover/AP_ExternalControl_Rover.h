/**
 * @file AP_ExternalControl_Rover.h
 * @brief External control library interface for ArduRover
 * 
 * @details This file defines the rover-specific implementation of the external
 *          control interface, which enables offboard control systems (such as
 *          companion computers, ROS2 integration, or external autopilots) to
 *          command velocity and position setpoints to the rover.
 * 
 *          The external control interface allows external systems to:
 *          - Command linear velocity and yaw rate in earth frame
 *          - Set global position targets for loiter/hold
 *          - Query vehicle readiness for external control
 * 
 * @note This functionality is only available when AP_EXTERNAL_CONTROL_ENABLED
 *       is defined during compilation.
 * 
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */
#pragma once

#include <AP_ExternalControl/AP_ExternalControl.h>

#if AP_EXTERNAL_CONTROL_ENABLED

/**
 * @class AP_ExternalControl_Rover
 * @brief Rover-specific implementation of external control interface
 * 
 * @details This class provides the rover-specific implementation for external
 *          control capabilities, allowing offboard systems to command the rover's
 *          motion. It extends the AP_ExternalControl base class with rover-specific
 *          behavior and constraints.
 * 
 *          The external control system enables companion computers, ROS2 nodes,
 *          or other external autopilots to:
 *          - Set velocity commands (linear velocity and yaw rate) in earth frame
 *          - Command global position targets for autonomous navigation
 *          - Query vehicle state to ensure commands are accepted
 * 
 *          **Coordinate Frames and Units**:
 *          - Velocity: Earth frame NED (North-East-Down) in m/s
 *          - Yaw rate: Earth frame NED in rad/s
 *          - Position: Global coordinates (latitude, longitude, altitude)
 * 
 *          **Mode Requirements**:
 *          External control is only functional in certain flight modes, typically
 *          Guided mode. The ready_for_external_control() method validates mode
 *          and arm state before accepting commands.
 * 
 *          **Thread Safety**:
 *          All methods must be called from the main vehicle thread. This class
 *          does not implement internal thread synchronization.
 * 
 *          **Safety Considerations**:
 *          External control bypasses normal pilot RC input. External systems
 *          should implement:
 *          - Command validation before sending
 *          - Timeout/watchdog mechanisms
 *          - Failsafe handling for lost communication
 *          - Smooth command transitions to avoid abrupt vehicle motion
 * 
 * @warning External control commands bypass normal pilot input. Loss of external
 *          control link should trigger appropriate failsafe behavior in the
 *          external system.
 * 
 * @note Inherits from AP_ExternalControl base class
 * @see AP_ExternalControl for base interface definition
 */
class AP_ExternalControl_Rover : public AP_ExternalControl
{
public:
    /**
     * @brief Set desired linear velocity and yaw rate from external source
     * 
     * @details Commands the rover to achieve the specified linear velocity vector
     *          and yaw rate. This method allows external control systems to directly
     *          command vehicle motion in the earth frame.
     * 
     *          The command is applied if the vehicle is ready for external control
     *          (typically requires Guided mode and armed state). If any velocity
     *          component or the yaw rate should not be controlled, pass NaN for
     *          that parameter.
     * 
     *          **Coordinate Frame**: Earth frame NED (North-East-Down)
     *          - linear_velocity.x: North velocity [m/s]
     *          - linear_velocity.y: East velocity [m/s]  
     *          - linear_velocity.z: Down velocity [m/s] (typically 0 or NaN for ground vehicles)
     *          - yaw_rate_rads: Yaw rate positive clockwise viewed from above [rad/s]
     * 
     *          **Selective Control**: Pass NaN for components that should not be controlled:
     *          - NaN in linear_velocity.x: Do not control north velocity
     *          - NaN in linear_velocity.y: Do not control east velocity
     *          - NaN in linear_velocity.z: Do not control down velocity (typical for rovers)
     *          - NaN in yaw_rate_rads: Do not control yaw rate
     * 
     * @param[in] linear_velocity Desired velocity vector in earth frame NED [m/s].
     *                            Pass NaN for any component to not control that axis.
     * @param[in] yaw_rate_rads   Desired yaw rate in earth frame NED [rad/s].
     *                            Pass NaN to not control yaw rate.
     * 
     * @return true if command accepted and applied to vehicle control system,
     *         false if rejected (vehicle not ready or invalid values)
     * 
     * @note Units: velocity [m/s], yaw rate [rad/s]
     * @note Coordinate frame: Earth NED (North-East-Down)
     * @note Update rate: Should be called at regular intervals (typically 10-50 Hz)
     *       for smooth control
     * 
     * @warning Requires vehicle in appropriate mode (typically Guided) and armed state.
     *          Commands will be rejected if ready_for_external_control() returns false.
     * @warning External system must implement timeout/watchdog. If commands stop,
     *          vehicle behavior depends on failsafe configuration.
     * 
     * @see ready_for_external_control() to check if vehicle will accept commands
     */
    bool set_linear_velocity_and_yaw_rate(const Vector3f &linear_velocity, float yaw_rate_rads)override WARN_IF_UNUSED; // override: implements base class virtual method; WARN_IF_UNUSED: ensures caller checks return value (safety requirement)

    /**
     * @brief Set global position target for loiter/hold
     * 
     * @details Commands the rover to navigate to and loiter at the specified
     *          global position. This method allows external control systems to
     *          set waypoint-like targets for the rover.
     * 
     *          The rover will navigate to the target location using its configured
     *          navigation algorithms and remain in the vicinity of that position
     *          (loiter behavior). The exact loiter radius and behavior depend on
     *          vehicle configuration parameters.
     * 
     *          **Coordinate System**: Global coordinates
     *          - Latitude: Degrees (WGS84 datum)
     *          - Longitude: Degrees (WGS84 datum)
     *          - Altitude: Meters above mean sea level (may be ignored for ground vehicles)
     * 
     *          The command is applied if the vehicle is ready for external control
     *          (typically requires Guided mode and armed state).
     * 
     * @param[in] loc Target location in global coordinates (latitude, longitude, altitude)
     * 
     * @return true if position target accepted and applied to vehicle navigation system,
     *         false if rejected (vehicle not ready or invalid location)
     * 
     * @note Altitude component may be ignored for ground vehicles depending on configuration
     * @note The rover will navigate to this position using configured navigation parameters
     *       (turn rate limits, speed limits, obstacle avoidance if enabled)
     * @note This is a "set and forget" command - rover maintains loiter at this position
     *       until a new command is received or mode changes
     * 
     * @warning Requires appropriate mode (typically Guided) and armed state.
     *          Commands will be rejected if ready_for_external_control() returns false.
     * @warning Ensure target location is reachable and safe (no obstacles, within geofence)
     * 
     * @see ready_for_external_control() to check if vehicle will accept commands
     * @see set_linear_velocity_and_yaw_rate() for direct velocity control alternative
     */
    bool set_global_position(const Location& loc) override WARN_IF_UNUSED; // override: implements base class virtual method; WARN_IF_UNUSED: ensures caller checks return value (safety requirement)

private:
    /**
     * @brief Check if rover is ready to accept external control commands
     * 
     * @details Validates that the vehicle is in an appropriate state to accept
     *          and execute external control commands. This method checks:
     *          - Vehicle mode (typically must be in Guided mode)
     *          - Arm state (typically must be armed)
     *          - Any other vehicle-specific conditions
     * 
     *          This is called internally before processing external control commands
     *          to ensure the vehicle can safely execute the requested actions.
     * 
     *          **Typical Requirements**:
     *          - Mode: GUIDED (external control not available in most other modes)
     *          - Armed: true (vehicle must be armed to accept motion commands)
     *          - No critical failsafes active
     * 
     * @return true if ready for external control (commands will be accepted),
     *         false otherwise (commands will be rejected)
     * 
     * @note This method is called internally before processing each external control
     *       command to validate vehicle state
     * @note Typically requires Guided mode and armed state, but exact requirements
     *       may vary based on vehicle configuration
     * 
     * @warning If this returns false, all external control commands
     *          (set_linear_velocity_and_yaw_rate, set_global_position) will be rejected
     * @warning External control systems should monitor this state and handle
     *          transitions gracefully (e.g., stop sending commands when not ready)
     * 
     * @see set_linear_velocity_and_yaw_rate()
     * @see set_global_position()
     */
    bool ready_for_external_control() WARN_IF_UNUSED; // WARN_IF_UNUSED: ensures caller checks return value before proceeding with external control
};

#endif // AP_EXTERNAL_CONTROL_ENABLED - Entire external control system only compiled if enabled in build configuration
