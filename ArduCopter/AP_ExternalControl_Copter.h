/**
 * @file AP_ExternalControl_Copter.h
 * @brief External control interface for ArduCopter multirotor vehicles
 * 
 * @details This file defines the vehicle-specific implementation of the external control
 *          interface for multicopter platforms. The external control API allows offboard
 *          control systems such as companion computers, vision-based navigation systems,
 *          and external autonomy modules to command the vehicle's position, velocity, and
 *          attitude.
 * 
 *          Key features and safety considerations:
 *          - **Safety Gates**: External control commands are only accepted when the vehicle
 *            is armed and in Guided mode, ensuring pilot authority and preventing unwanted
 *            autonomous behavior.
 *          - **Coordinate Frames**: All velocity commands use the NED (North-East-Down) earth
 *            frame convention, consistent with ArduPilot's internal navigation system.
 *          - **Unit Conventions**: Linear velocities are specified in meters per second [m/s],
 *            yaw rates in radians per second [rad/s]. Internal conversions to centimeters and
 *            centidegrees are handled automatically.
 *          - **Yaw Control Flexibility**: Commands support yaw-free operation by passing NaN
 *            for yaw rate, allowing position/velocity control without yaw constraints.
 * 
 *          The external control interface is designed for integration with:
 *          - Vision-based navigation systems (visual-inertial odometry, optical flow)
 *          - Companion computer path planning and obstacle avoidance
 *          - External localization systems (motion capture, beacon positioning)
 *          - Custom autopilot behaviors and research algorithms
 * 
 * @note External control is only available when AP_EXTERNAL_CONTROL_ENABLED is defined.
 *       This is typically enabled in vehicle-specific configuration files.
 * 
 * @warning External control commands bypass normal flight mode logic and directly command
 *          the vehicle. Ensure external control systems have proper failsafe handling,
 *          timeout detection, and coordinate frame validation to prevent vehicle crashes.
 * 
 * @see AP_ExternalControl Base class defining the external control interface
 * @see Copter::Mode::Guided Guided mode implementation accepting external commands
 * 
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */
#pragma once

#include <AP_ExternalControl/AP_ExternalControl.h>

#if AP_EXTERNAL_CONTROL_ENABLED

/**
 * @class AP_ExternalControl_Copter
 * @brief Copter-specific implementation of external control interface
 * 
 * @details This class provides the vehicle-specific implementation of the AP_ExternalControl
 *          interface for multicopter platforms. It handles the translation of external control
 *          commands into Copter-specific guided mode targets while enforcing safety constraints.
 * 
 *          **Architecture and Integration**:
 *          - Inherits from AP_ExternalControl base class which defines the generic interface
 *          - Integrates with Copter::Mode::Guided to set position and velocity targets
 *          - Uses Copter's arming system to enforce safety gates
 *          - Leverages Copter's coordinate frame transformation utilities
 * 
 *          **Safety Model**:
 *          The external control interface implements a conservative safety model:
 *          1. Commands accepted ONLY in Guided mode (pilot must explicitly enable)
 *          2. Vehicle MUST be armed (propellers spinning, ready for flight)
 *          3. Timeouts enforced - commands must be continuously sent or vehicle will hold position
 *          4. Mode transitions immediately disable external control
 * 
 *          **Coordinate Frame Transformations**:
 *          External control uses NED (North-East-Down) earth frame:
 *          - North: Positive X velocity moves vehicle north
 *          - East: Positive Y velocity moves vehicle east
 *          - Down: Positive Z velocity moves vehicle down (negative is up)
 *          This matches MAVLink conventions and ArduPilot's internal navigation frames.
 * 
 *          **Unit Conversions**:
 *          External API uses SI units (meters, radians) for ease of integration:
 *          - Input velocities [m/s] → Internal velocities [cm/s] (multiply by 100)
 *          - Input yaw rates [rad/s] → Internal rates [deg/s] (multiply by 57.2958)
 *          - Position targets handled by Location class with automatic conversions
 * 
 *          **NaN Yaw Semantics**:
 *          Passing NaN (Not-a-Number) for yaw_rate parameter enables yaw-free control:
 *          - Vehicle maintains current heading while following position/velocity commands
 *          - Useful for visual servoing and position tracking without yaw constraints
 *          - Implementation uses isnan() check to detect NaN and skip yaw updates
 * 
 * @note This class is instantiated as a singleton through the Copter vehicle object
 *       and accessed via MAVLink DO_SET_POSITION_TARGET_* commands or companion computer APIs.
 * 
 * @warning External control commands have NO collision avoidance, fence checking, or terrain
 *          following by default. External control systems MUST implement their own safety layers.
 */
class AP_ExternalControl_Copter : public AP_ExternalControl
{
public:
    /**
     * @brief Set linear velocity and yaw rate commands for external control
     * 
     * @details Provides velocity-based control of the multicopter through external systems.
     *          This method sends velocity targets to the Copter guided mode controller which
     *          executes them using the position control system. The command is continuously
     *          updated by the external system (typically 10-50 Hz) to track desired trajectories.
     * 
     *          **Control Implementation**:
     *          - Velocity targets sent to AC_PosControl velocity controller
     *          - Internal conversion: input [m/s] → target [cm/s]
     *          - Yaw rate sent to attitude controller if not NaN
     *          - Commands expire after timeout (typically 1 second) if not refreshed
     * 
     *          **Coordinate Frame Convention**:
     *          NED (North-East-Down) earth frame used for all velocity components:
     *          - linear_velocity.x: North velocity (positive = move north)
     *          - linear_velocity.y: East velocity (positive = move east)
     *          - linear_velocity.z: Down velocity (positive = descend, negative = climb)
     * 
     *          **Yaw Control Options**:
     *          - Normal operation: Specify yaw_rate_rads for coordinated turns
     *          - Yaw-free operation: Pass NaN to maintain current heading
     *          - Zero yaw rate: Pass 0.0 to actively hold current heading
     * 
     *          **Safety Enforcement**:
     *          Command rejected unless:
     *          1. Vehicle in Guided mode (pilot authorization required)
     *          2. Vehicle armed (motors spinning)
     *          3. EKF position estimate healthy
     * 
     * @param[in] linear_velocity  Desired velocity in NED earth frame [m/s].
     *                              Valid range: typically ±10 m/s depending on vehicle configuration.
     *                              Components: (north, east, down)
     * @param[in] yaw_rate_rads    Desired yaw rate in earth frame [rad/s].
     *                              Pass NaN to not control yaw (maintain current heading).
     *                              Valid range: typically ±1.0 rad/s depending on configuration.
     * 
     * @return true if command accepted and sent to guided mode controller, false if rejected
     *         due to safety checks (wrong mode, disarmed, unhealthy navigation)
     * 
     * @note This method should be called continuously (10+ Hz recommended) to maintain smooth
     *       velocity tracking. Commands timeout after ~1 second causing vehicle to hold position.
     * 
     * @note Velocity limits are enforced by the velocity controller using WPNAV_SPEED parameter
     *       for horizontal motion and PILOT_SPEED_DN/PILOT_SPEED_UP for vertical motion.
     * 
     * @warning Rapid velocity changes or excessive rates can cause aggressive vehicle response.
     *          External control systems should implement acceleration limiting and smooth
     *          trajectory generation.
     * 
     * @warning This command has NO obstacle avoidance, terrain following, or fence checking
     *          integrated by default. External control system is responsible for collision
     *          avoidance and safety constraints.
     * 
     * @see AC_PosControl::set_vel_desired() Internal velocity controller
     * @see Copter::Mode::Guided::set_velocity() Guided mode velocity command handler
     */
    bool set_linear_velocity_and_yaw_rate(const Vector3f &linear_velocity, float yaw_rate_rads) override WARN_IF_UNUSED;

    /**
     * @brief Set global position target for external control
     * 
     * @details Commands the multicopter to fly to and loiter at a specified global position.
     *          This method provides position-based control as an alternative to velocity control,
     *          allowing external systems to specify waypoint targets that the vehicle will
     *          navigate to autonomously using its internal position controller.
     * 
     *          **Control Implementation**:
     *          - Position target sent to Copter guided mode as a loiter point
     *          - Vehicle uses waypoint navigation (AC_WPNav) to reach target
     *          - Automatic speed profiling and deceleration on approach
     *          - Loiter controller maintains position once target reached
     * 
     *          **Location Format**:
     *          The Location parameter specifies global position using:
     *          - Latitude: WGS84 coordinate in degrees (or 1e-7 degree units)
     *          - Longitude: WGS84 coordinate in degrees (or 1e-7 degree units)
     *          - Altitude: Can be specified as:
     *            * Absolute altitude (AMSL - Above Mean Sea Level)
     *            * Relative altitude (above home position)
     *            * Terrain altitude (above ground level using terrain database)
     *          - Altitude frame specified by Location.flags.relative_alt and terrain_alt
     * 
     *          **Navigation Behavior**:
     *          - Vehicle flies to horizontal position at configured navigation speed (WPNAV_SPEED)
     *          - Altitude changes at configured climb/descent rates
     *          - S-curve acceleration/deceleration for smooth motion
     *          - Loiter radius typically 5-10cm once target reached
     * 
     *          **Safety Enforcement**:
     *          Command rejected unless:
     *          1. Vehicle in Guided mode (pilot authorization required)
     *          2. Vehicle armed (motors spinning)
     *          3. GPS lock available for global position navigation
     *          4. Target position within geofence if enabled
     * 
     * @param[in] loc  Target global position as Location structure.
     *                 Latitude and longitude in WGS84 datum.
     *                 Altitude frame determined by Location flags.
     *                 Typical range: Within mission planner operational area (geofence).
     * 
     * @return true if command accepted and position target set in guided mode, false if rejected
     *         due to safety checks (wrong mode, disarmed, no GPS, invalid location)
     * 
     * @note Unlike velocity commands, position targets do not require continuous updates.
     *       Vehicle will fly to target and hold position until new command received.
     * 
     * @note Position accuracy depends on GPS precision (typically 1-5m horizontally) and
     *       altitude source (GPS altitude typically ±5-10m, barometer relative altitude ±1m).
     * 
     * @note This method can be used for waypoint-based autonomous missions where external
     *       system computes waypoints but delegates trajectory generation to ArduPilot.
     * 
     * @warning GPS glitches or loss of position lock during navigation can cause unpredictable
     *          behavior. External control systems should monitor GPS health (EKF status).
     * 
     * @warning Target positions outside geofence boundaries will be rejected if geofence
     *          is enabled and configured to prevent breaches.
     * 
     * @warning Altitude frame mismatches can cause dangerous altitude commands. Ensure
     *          external system correctly sets Location flags for intended altitude reference.
     * 
     * @see Location ArduPilot location class for coordinate representation
     * @see Copter::Mode::Guided::set_destination() Guided mode position command handler
     * @see AC_WPNav Waypoint navigation controller implementation
     */
    bool set_global_position(const Location& loc) override WARN_IF_UNUSED;
private:
    /**
     * @brief Check if vehicle is ready to accept external control commands
     * 
     * @details Performs safety validation before accepting any external control command.
     *          This method implements the safety gate logic that ensures external control
     *          only operates when the vehicle is in an appropriate state and the pilot
     *          has explicitly authorized external control operation.
     * 
     *          **Safety Checks Performed**:
     *          1. Mode Check: Vehicle must be in Guided mode
     *             - Guided mode indicates pilot has authorized autonomous operation
     *             - Other modes (Stabilize, AltHold, etc.) do not accept external commands
     *             - Mode transition immediately invalidates external control authority
     * 
     *          2. Arming Check: Vehicle must be armed
     *             - Motors must be spinning for control commands to take effect
     *             - Disarmed state prevents any motor output regardless of commands
     *             - Prevents unwanted motor start from external commands
     * 
     *          **Rationale for Safety Model**:
     *          - Pilot authority: Guided mode requirement ensures pilot explicitly enables
     *          - Controlled engagement: Arming requirement prevents surprise motor activation
     *          - Clear disengagement: Mode switch immediately stops external control
     * 
     *          **Future Extensions**:
     *          Additional checks that may be added (currently not implemented):
     *          - EKF health validation for position/velocity commands requiring navigation
     *          - GPS lock verification for global position commands
     *          - Battery failsafe status
     *          - Geofence breach status
     * 
     * @return true if vehicle is in Guided mode AND armed, allowing external control commands;
     *         false otherwise, causing external control commands to be rejected
     * 
     * @note This method is called by all external control command handlers before processing
     *       commands, providing centralized safety gate enforcement.
     * 
     * @note The check is stateless and called on every command - there is no persistent
     *       "external control session" that needs to be established.
     * 
     * @warning Do not bypass this safety check when implementing new external control commands.
     *          The mode and arming requirements are essential for safe operation.
     * 
     * @see Copter::control_mode Current vehicle control mode
     * @see AP_Arming::is_armed() Vehicle arming state check
     */
    bool ready_for_external_control() WARN_IF_UNUSED;
};

#endif // AP_EXTERNAL_CONTROL_ENABLED
