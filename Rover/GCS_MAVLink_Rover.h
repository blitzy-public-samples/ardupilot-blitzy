/**
 * @file GCS_MAVLink_Rover.h
 * @brief Rover-specific MAVLink message handlers and protocol implementation
 * 
 * @details This file defines the GCS_MAVLINK_Rover class which extends the base
 *          GCS_MAVLINK class to provide Rover-specific implementations of MAVLink
 *          message handlers. It implements custom behavior for ground vehicle
 *          commands, telemetry streaming, and guided mode operations specific
 *          to the Rover vehicle type.
 * 
 *          Key responsibilities:
 *          - Handle rover-specific MAVLink commands (guided steering, set yaw speed)
 *          - Process position target messages for guided mode navigation
 *          - Stream rover-specific telemetry (servo outputs, navigation data)
 *          - Manage attitude and position control via external commands
 *          - Support rangefinder/water depth telemetry for aquatic rovers
 * 
 * @note Some features are conditionally compiled based on configuration flags
 *       such as AP_MAVLINK_MAV_CMD_NAV_SET_YAW_SPEED_ENABLED, HAL_LOGGING_ENABLED,
 *       AP_RANGEFINDER_ENABLED, and HAL_HIGH_LATENCY2_ENABLED
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

#include <GCS_MAVLink/GCS.h>

  // set 0 in 4.6, remove feature in 4.7:
#ifndef AP_MAVLINK_MAV_CMD_NAV_SET_YAW_SPEED_ENABLED
#define AP_MAVLINK_MAV_CMD_NAV_SET_YAW_SPEED_ENABLED 0
#endif

#include "defines.h"

/**
 * @class GCS_MAVLINK_Rover
 * @brief Rover-specific MAVLink communication channel handler
 * 
 * @details This class extends GCS_MAVLINK to provide Rover-specific implementations
 *          of MAVLink message handling. It processes commands and telemetry requests
 *          specific to ground vehicles including steering commands, position targets,
 *          and rover navigation data.
 * 
 *          The class handles:
 *          - MAV_CMD_NAV_SET_YAW_SPEED for heading and speed control
 *          - DO_REPOSITION commands for guided mode waypoint navigation
 *          - SET_POSITION_TARGET_* messages for external position control
 *          - SET_ATTITUDE_TARGET for external attitude control
 *          - Manual control input processing for joystick/RC override
 *          - Rover-specific telemetry streaming (servo outputs, PID tuning)
 * 
 *          Vehicle capabilities are exposed via the capabilities() method to inform
 *          ground control stations of supported features.
 * 
 * @note Thread Safety: Methods are called from the main scheduler thread and
 *       MAVLink receive thread. Shared state access uses appropriate locking.
 * 
 * @see GCS_MAVLINK base class for common MAVLink functionality
 * @see Rover main vehicle class for mode and control integration
 */
class GCS_MAVLINK_Rover : public GCS_MAVLINK
{
public:

    using GCS_MAVLINK::GCS_MAVLINK;

protected:

    /**
     * @brief Handle preflight calibration commands specific to Rover
     * 
     * @details Processes MAV_CMD_PREFLIGHT_CALIBRATION commands for sensor
     *          calibration including gyro, accelerometer, and compass calibration.
     *          Overrides base implementation to add rover-specific calibration logic.
     * 
     * @param[in] packet MAVLink COMMAND_INT packet containing calibration parameters
     * @param[in] msg    Complete MAVLink message wrapper
     * 
     * @return MAV_RESULT indicating command acceptance and execution status:
     *         - MAV_RESULT_ACCEPTED: Calibration started successfully
     *         - MAV_RESULT_DENIED: Vehicle not in appropriate state
     *         - MAV_RESULT_UNSUPPORTED: Calibration type not supported
     *         - MAV_RESULT_TEMPORARILY_REJECTED: System busy, retry later
     */
    MAV_RESULT _handle_command_preflight_calibration(const mavlink_command_int_t &packet, const mavlink_message_t &msg) override;
    
    /**
     * @brief Handle MAVLink COMMAND_INT messages for Rover
     * 
     * @details Primary handler for position-based commands sent via MAV_CMD_*
     *          in COMMAND_INT format. Routes commands to appropriate handlers
     *          including DO_REPOSITION, NAV_SET_YAW_SPEED, and other rover-specific
     *          navigation commands. Uses integer coordinates for precise positioning.
     * 
     * @param[in] packet COMMAND_INT packet with target position in WGS84 coordinates
     * @param[in] msg    Complete MAVLink message for source system identification
     * 
     * @return MAV_RESULT command execution status:
     *         - MAV_RESULT_ACCEPTED: Command executed successfully
     *         - MAV_RESULT_DENIED: Command rejected (e.g., not in guided mode)
     *         - MAV_RESULT_UNSUPPORTED: Command not implemented for rovers
     *         - MAV_RESULT_FAILED: Execution failed (e.g., invalid parameters)
     * 
     * @note This is called at MAVLink receive rate when COMMAND_INT messages arrive
     * @see handle_command_int_do_reposition() for DO_REPOSITION implementation
     * @see handle_command_nav_set_yaw_speed() for yaw/speed command handling
     */
    MAV_RESULT handle_command_int_packet(const mavlink_command_int_t &packet, const mavlink_message_t &msg) override;
    
    /**
     * @brief Handle DO_REPOSITION command to move rover to new location
     * 
     * @details Processes MAV_CMD_DO_REPOSITION to command the rover to navigate
     *          to a new position in guided mode. Sets target location and speed,
     *          initiating autonomous navigation to the specified coordinates.
     * 
     * @param[in] packet COMMAND_INT with target latitude/longitude/altitude and speed
     *                   - param1: Ground speed in m/s (-1 to use default)
     *                   - param4: Yaw in degrees (NaN to use default)
     *                   - x: Target latitude in degrees * 1E7
     *                   - y: Target longitude in degrees * 1E7
     *                   - z: Target altitude (not used by rover, reserved)
     * 
     * @return MAV_RESULT_ACCEPTED if target set successfully,
     *         MAV_RESULT_DENIED if not in guided mode or parameters invalid
     * 
     * @note Requires vehicle to be in GUIDED mode
     * @see Mode::set_desired_location() for target position handling
     */
    MAV_RESULT handle_command_int_do_reposition(const mavlink_command_int_t &packet);
    
    /**
     * @brief Handle NAV_SET_YAW_SPEED command for heading and speed control
     * 
     * @details Processes MAV_CMD_NAV_SET_YAW_SPEED to command the rover to drive
     *          at a specified speed while maintaining or changing heading. Supports
     *          both absolute and relative heading commands.
     * 
     * @param[in] packet COMMAND_INT containing yaw and speed parameters:
     *                   - param1: Yaw angle in degrees (0-360)
     *                   - param2: Speed in m/s
     *                   - param3: Direction: -1=CCW, 0=shortest, 1=CW
     *                   - param4: 0=absolute angle, 1=relative to current heading
     * @param[in] msg    Complete MAVLink message wrapper
     * 
     * @return MAV_RESULT_ACCEPTED if command accepted,
     *         MAV_RESULT_DENIED if not in guided mode,
     *         MAV_RESULT_UNSUPPORTED if feature disabled at compile time
     * 
     * @note Conditionally compiled based on AP_MAVLINK_MAV_CMD_NAV_SET_YAW_SPEED_ENABLED
     * @warning Feature deprecated, set to 0 in 4.6, will be removed in 4.7
     */
    MAV_RESULT handle_command_nav_set_yaw_speed(const mavlink_command_int_t &packet, const mavlink_message_t &msg);

    /**
     * @brief Send current position target to ground control station
     * 
     * @details Streams POSITION_TARGET_GLOBAL_INT message containing the rover's
     *          current navigation target in guided mode. Includes target position,
     *          velocity, and control flags. Used by GCS to display intended path
     *          and monitor autonomous navigation progress.
     * 
     *          Message includes:
     *          - Target latitude/longitude (WGS84, degrees * 1E7)
     *          - Target velocity in NED frame (m/s)
     *          - Yaw angle (radians)
     *          - Type mask indicating which fields are valid
     * 
     * @note Called at telemetry stream rate when position target is active
     * @note Only sends meaningful data when in GUIDED mode with active target
     * @see POSITION_TARGET_GLOBAL_INT MAVLink message definition
     */
    void send_position_target_global_int() override;

    /**
     * @brief Report vehicle capabilities to ground control station
     * 
     * @details Returns bitmask of MAV_PROTOCOL_CAPABILITY flags indicating
     *          features supported by Rover. Used by GCS to enable/disable
     *          UI elements and validate command compatibility.
     * 
     * @return uint64_t bitmask of capabilities including:
     *         - MISSION_FLOAT: Supports mission items with float parameters
     *         - PARAM_FLOAT: Supports float parameters
     *         - COMMAND_INT: Supports COMMAND_INT messages
     *         - MAVLINK2: Supports MAVLink 2.0 protocol
     *         - SET_ATTITUDE_TARGET: Supports external attitude control
     *         - SET_POSITION_TARGET_*: Supports external position control
     * 
     * @note Capabilities are static for vehicle type, may vary by build configuration
     */
    uint64_t capabilities() const override;

    /**
     * @brief Send navigation controller output telemetry
     * 
     * @details Streams NAV_CONTROLLER_OUTPUT message with current navigation
     *          status including cross-track error, bearing to target, distance
     *          remaining, and steering output. Critical for monitoring autonomous
     *          navigation performance and tuning control loops.
     * 
     * @note Called at telemetry stream rate (typically 1-10 Hz)
     * @see NAV_CONTROLLER_OUTPUT MAVLink message for field definitions
     */
    void send_nav_controller_output() const override;
    
    /**
     * @brief Send PID tuning data for selected controller
     * 
     * @details Streams PID_TUNING message with real-time PID controller data
     *          including desired value, achieved value, P/I/D components, and
     *          FF term. Used for tuning steering and speed controllers.
     * 
     * @note Streams data for controller selected via TUNE_PARAM parameter
     * @note Called at high rate when tuning is active
     * @see PID_TUNING MAVLink message definition
     */
    void send_pid_tuning() override;

#if HAL_LOGGING_ENABLED
    /**
     * @brief Get logging bitmask for radio/RC input data
     * 
     * @details Returns the log bitmask bit that controls logging of radio
     *          (RC input) data for Rover. Used by logging system to determine
     *          if RC_IN messages should be written to dataflash logs.
     * 
     * @return uint32_t MASK_LOG_PM bitmask value for performance monitoring logs
     * 
     * @note Conditionally compiled when HAL_LOGGING_ENABLED is true
     */
    uint32_t log_radio_bit() const override { return MASK_LOG_PM; }
#endif

    /**
     * @brief Send available flight mode information via AVAILABLE_MODES message
     * 
     * @details Iterates through rover flight modes and sends information about
     *          the mode at the given index. Used by GCS to populate mode selection
     *          menus with rover-specific modes (MANUAL, ACRO, STEERING, HOLD, 
     *          LOITER, FOLLOW, SIMPLE, AUTO, RTL, SMART_RTL, GUIDED, INITIALISING).
     * 
     * @param[in] index Mode index to send (1-based indexing, not mode number)
     * 
     * @return uint8_t Total number of available modes for rover
     * 
     * @note Index parameter starts at 1, not 0
     * @note Called repeatedly by GCS to enumerate all available modes
     * @see Mode class for mode number to name mapping
     */
    uint8_t send_available_mode(uint8_t index) const override;

private:

    /**
     * @brief Process received MAVLink messages with rover-specific handling
     * 
     * @details Main message dispatcher for rover-specific MAVLink messages.
     *          Routes messages to appropriate handlers including SET_ATTITUDE_TARGET,
     *          SET_POSITION_TARGET_*, RADIO, LANDING_TARGET, and other rover messages.
     *          Falls back to base class for common messages.
     * 
     * @param[in] msg Received MAVLink message to process
     * 
     * @note Called from MAVLink receive thread at message arrival rate
     * @note Override of base GCS_MAVLINK::handle_message()
     */
    void handle_message(const mavlink_message_t &msg) override;
    
    /**
     * @brief Handle guided mode mission command requests
     * 
     * @details Processes mission commands sent by GCS for execution in guided mode.
     *          Validates command parameters and initiates guided mode navigation
     *          or control actions. Used for waypoint navigation and control commands.
     * 
     * @param[in,out] cmd Mission command structure to process and potentially modify
     * 
     * @return true if command accepted and executed, false if rejected
     * 
     * @note Requires vehicle to be in GUIDED mode for most commands
     */
    bool handle_guided_request(AP_Mission::Mission_Command &cmd) override;
    
    /**
     * @brief Attempt to send queued telemetry message
     * 
     * @details Tries to send a telemetry message from the prioritized queue.
     *          Returns false if message cannot be sent due to bandwidth limits
     *          or buffer full conditions. Implements rover-specific message handling.
     * 
     * @param[in] id Enum identifier of message type to send (ap_message enum)
     * 
     * @return true if message sent successfully, false if deferred or skipped
     * 
     * @note Called from scheduler at telemetry stream rate
     * @note Bandwidth management handled automatically by MAVLink stream system
     */
    bool try_send_message(enum ap_message id) override;

    /**
     * @brief Process manual control axis inputs from joystick
     * 
     * @details Handles MANUAL_CONTROL message providing joystick axis inputs
     *          for rover steering, throttle, and auxiliary functions. Converts
     *          normalized joystick values to rover control inputs in guided mode.
     * 
     * @param[in] packet MANUAL_CONTROL packet with axis values (-1000 to 1000)
     *                   - x: Pitch axis (forward/backward throttle)
     *                   - y: Roll axis (left/right steering)
     *                   - z: Throttle axis (alternative throttle input)
     *                   - r: Yaw axis (alternative steering input)
     * @param[in] tnow   Current system time in milliseconds
     * 
     * @note Override of base implementation for rover-specific axis mapping
     */
    void handle_manual_control_axes(const mavlink_manual_control_t &packet, const uint32_t tnow) override;
    
    /**
     * @brief Handle SET_ATTITUDE_TARGET message for external attitude control
     * 
     * @details Processes SET_ATTITUDE_TARGET message to control rover attitude
     *          (primarily yaw) and throttle from external controller. Used for
     *          advanced control applications and companion computer integration.
     * 
     * @param[in] msg MAVLink message containing target attitude quaternion,
     *                body frame angular rates, and thrust
     * 
     * @note Requires GUIDED mode for attitude control
     * @note Rover primarily uses yaw component, roll/pitch ignored
     */
    void handle_set_attitude_target(const mavlink_message_t &msg);
    
    /**
     * @brief Handle SET_POSITION_TARGET_LOCAL_NED for local position control
     * 
     * @details Processes position/velocity targets in local NED (North-East-Down)
     *          coordinate frame. Allows external control of rover position and
     *          velocity for precision navigation applications.
     * 
     * @param[in] msg Message with target position (meters), velocity (m/s),
     *                acceleration (m/s²), and type mask indicating valid fields
     * 
     * @note Local frame origin is typically home position or EKF origin
     * @warning Acceleration targets are ignored, inform user if not masked
     */
    void handle_set_position_target_local_ned(const mavlink_message_t &msg);
    
    /**
     * @brief Handle SET_POSITION_TARGET_GLOBAL_INT for global position control
     * 
     * @details Processes position/velocity targets in global WGS84 coordinates
     *          (latitude/longitude). Enables external navigation control using
     *          GPS-based position targets.
     * 
     * @param[in] msg Message with target lat/lon (degrees * 1E7), altitude,
     *                velocity (m/s in NED frame), and type mask
     * 
     * @note Altitude component ignored for ground vehicles
     * @warning Acceleration targets not supported, user must mask them out
     */
    void handle_set_position_target_global_int(const mavlink_message_t &msg);
    
    /**
     * @brief Handle RADIO message with RC receiver signal strength
     * 
     * @details Processes RADIO message containing RC link quality metrics
     *          (RSSI, noise, etc.) from telemetry radios or RC receivers.
     *          Used for monitoring link quality and triggering failsafes.
     * 
     * @param[in] msg Message with radio status information
     */
    void handle_radio(const mavlink_message_t &msg);
    
    /**
     * @brief Handle LANDING_TARGET message for precision navigation
     * 
     * @details Processes detected landing target or visual marker information.
     *          For rovers, can be used for precision docking or follow-target
     *          applications using vision systems.
     * 
     * @param[in] msg           LANDING_TARGET message with target position/size
     * @param[in] timestamp_ms  System timestamp in milliseconds for sensor fusion
     * 
     * @note Requires AP_PRECLAND enabled and appropriate sensor backend
     */
    void handle_landing_target(const mavlink_landing_target_t &msg, uint32_t timestamp_ms) override;

    /**
     * @brief Send servo output values to ground control station
     * 
     * @details Streams SERVO_OUTPUT_RAW message containing PWM values for
     *          all servo channels (steering, throttle, auxiliary functions).
     *          Used by GCS to monitor output commands and diagnose control issues.
     * 
     * @note Called at telemetry stream rate
     * @note Includes timestamp for output synchronization
     */
    void send_servo_out();

    /**
     * @brief Send warning message about acceleration field masking requirement
     * 
     * @details If we receive a SET_POSITION_TARGET_* message where the user has
     *          not masked out acceleration fields (which rovers don't support),
     *          send a STATUSTEXT message informing them that acceleration must
     *          be masked. Prevents repeated processing of unsupported fields.
     * 
     * @param[in] msgname Name of the message type for error message context
     *                    (e.g., "SET_POSITION_TARGET_LOCAL_NED")
     * 
     * @note Message sent with MAV_SEVERITY_WARNING level
     * @note Only sent once per message type to avoid spamming
     */
    void send_acc_ignore_must_be_set_message(const char *msgname);
    
    /**
     * @brief Get base mode flags for HEARTBEAT message
     * 
     * @details Returns MAV_MODE_FLAG bitmask indicating current vehicle state
     *          including armed status, guided mode, stabilization, manual input,
     *          and safety switch state. Used in HEARTBEAT message.
     * 
     * @return uint8_t Bitmask of MAV_MODE_FLAG values
     * 
     * @note Called at HEARTBEAT rate (typically 1 Hz)
     * @see MAV_MODE_FLAG enumeration in MAVLink common definitions
     */
    uint8_t base_mode() const override;
    
    /**
     * @brief Get vehicle system status for HEARTBEAT message
     * 
     * @details Returns MAV_STATE indicating current operational state:
     *          UNINIT, BOOT, CALIBRATING, STANDBY, ACTIVE, CRITICAL, EMERGENCY.
     *          Reflects initialization progress, arming state, and error conditions.
     * 
     * @return MAV_STATE Current vehicle state enumeration value
     * 
     * @note Called at HEARTBEAT rate (typically 1 Hz)
     * @see MAV_STATE enumeration in MAVLink common definitions
     */
    MAV_STATE vehicle_system_status() const override;

    /**
     * @brief Get throttle percentage for VFR_HUD message
     * 
     * @details Returns current throttle output as percentage (0-100) for
     *          display in GCS HUD (Heads-Up Display). Represents actual
     *          throttle servo output, not pilot input.
     * 
     * @return int16_t Throttle percentage (0-100), or 0 if disarmed
     * 
     * @note Called when streaming VFR_HUD message
     * @see VFR_HUD MAVLink message definition
     */
    int16_t vfr_hud_throttle() const override;

#if AP_MAVLINK_MSG_RANGEFINDER_SENDING_ENABLED
    /**
     * @brief Send rangefinder distance measurements to GCS
     * 
     * @details Streams RANGEFINDER message containing distance measurements
     *          from all configured rangefinder sensors. Used for obstacle
     *          detection, terrain following, and object avoidance visualization.
     * 
     * @note Conditionally compiled when AP_MAVLINK_MSG_RANGEFINDER_SENDING_ENABLED
     * @note Called at telemetry stream rate for rangefinder data
     * @see AP_RangeFinder library for sensor backend implementations
     */
    void send_rangefinder() const override;
#endif  // AP_MAVLINK_MSG_RANGEFINDER_SENDING_ENABLED

#if AP_RANGEFINDER_ENABLED
    /**
     * @brief Send water depth measurement for aquatic rovers
     * 
     * @details Sends WATER_DEPTH message containing depth below water surface
     *          and water temperature from downward-facing rangefinder. Used for
     *          bathymetric surveys and underwater navigation. Cycles through
     *          multiple rangefinder backends to manage bandwidth consumption.
     * 
     * @note Conditionally compiled when AP_RANGEFINDER_ENABLED
     * @note Cycles through rangefinder backends using last_WATER_DEPTH_index
     * @note Depth in meters, temperature in degrees Celsius
     * @see AP_RangeFinder for sensor configuration and calibration
     */
    void send_water_depth();
    
    /**
     * @brief State variable tracking last rangefinder sensor for WATER_DEPTH
     * 
     * @details Index of the last rangefinder backend for which we sent a
     *          WATER_DEPTH message. Used to round-robin through multiple
     *          rangefinders to limit telemetry bandwidth while providing
     *          data from all sensors over time.
     */
    uint8_t last_WATER_DEPTH_index;
#endif

#if HAL_HIGH_LATENCY2_ENABLED
    /**
     * @brief Get target heading for high latency telemetry
     * 
     * @details Returns current navigation target heading in degrees (0-360)
     *          for HIGH_LATENCY2 message. Provides compressed heading data
     *          for bandwidth-constrained satellite or long-range links.
     * 
     * @return uint8_t Target heading in degrees scaled to uint8 (0-255 = 0-360°)
     * 
     * @note Conditionally compiled when HAL_HIGH_LATENCY2_ENABLED
     */
    uint8_t high_latency_tgt_heading() const override;
    
    /**
     * @brief Get distance to target for high latency telemetry
     * 
     * @details Returns distance to current navigation target in meters for
     *          HIGH_LATENCY2 message. Distance is scaled to fit uint16 range.
     * 
     * @return uint16_t Distance to target in meters (0-65535m range)
     * 
     * @note Conditionally compiled when HAL_HIGH_LATENCY2_ENABLED
     */
    uint16_t high_latency_tgt_dist() const override;
    
    /**
     * @brief Get target ground speed for high latency telemetry
     * 
     * @details Returns current target ground speed in m/s for HIGH_LATENCY2
     *          message. For rovers, this is the commanded navigation speed.
     * 
     * @return uint8_t Target speed in m/s scaled to uint8 (0-255 m/s)
     * 
     * @note Conditionally compiled when HAL_HIGH_LATENCY2_ENABLED
     * @note Named "airspeed" in base class for consistency, represents ground speed for rovers
     */
    uint8_t high_latency_tgt_airspeed() const override;
    
    /**
     * @brief Get wind speed estimate for high latency telemetry
     * 
     * @details Returns estimated wind speed for HIGH_LATENCY2 message.
     *          For rovers, may return zero or estimate based on navigation
     *          performance degradation.
     * 
     * @return uint8_t Wind speed estimate in m/s scaled to uint8 (0-255 m/s)
     * 
     * @note Conditionally compiled when HAL_HIGH_LATENCY2_ENABLED
     * @note Wind estimation less critical for ground vehicles than aircraft
     */
    uint8_t high_latency_wind_speed() const override;
    
    /**
     * @brief Get wind direction estimate for high latency telemetry
     * 
     * @details Returns estimated wind direction in degrees for HIGH_LATENCY2
     *          message. For rovers, may return zero or estimate if available.
     * 
     * @return uint8_t Wind direction in degrees scaled to uint8 (0-255 = 0-360°)
     * 
     * @note Conditionally compiled when HAL_HIGH_LATENCY2_ENABLED
     */
    uint8_t high_latency_wind_direction() const override;
#endif // HAL_HIGH_LATENCY2_ENABLED
};
