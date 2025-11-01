/**
 * @file GCS_MAVLink_Plane.h
 * @brief MAVLink communication interface for ArduPlane
 * 
 * @details This file defines the ArduPlane-specific MAVLink channel implementation,
 *          extending the base GCS_MAVLink class with Plane-specific telemetry,
 *          command handling, and status reporting. This interface handles all
 *          MAVLink protocol communication between the autopilot and ground control
 *          stations, including mission management, parameter updates, telemetry
 *          streaming, and real-time command execution.
 *          
 *          The implementation supports:
 *          - Fixed-wing specific telemetry (airspeed, angle of attack, sideslip)
 *          - VTOL/QuadPlane state reporting and command handling
 *          - Plane-specific guided mode commands (altitude changes, repositioning)
 *          - Flight mode enumeration and switching
 *          - High-latency telemetry compression
 *          - Custom PID tuning message generation
 * 
 * @note This class is instantiated per MAVLink channel (typically 2-4 channels)
 * @see GCS_MAVLink base class in libraries/GCS_MAVLink/GCS.h
 * 
 * Source: ArduPlane/GCS_MAVLink_Plane.h
 */

#pragma once

#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Airspeed/AP_Airspeed_config.h>
#include "quadplane.h"
#include "defines.h"

/**
 * @class GCS_MAVLINK_Plane
 * @brief ArduPlane-specific MAVLink communication channel implementation
 * 
 * @details This class extends the base GCS_MAVLink interface to provide
 *          ArduPlane-specific MAVLink protocol handling. Each instance represents
 *          a single MAVLink communication channel (serial port, telemetry radio,
 *          USB, or network connection) and manages:
 *          
 *          - **Telemetry Streaming**: Fixed-wing specific sensor data, flight
 *            parameters, and status information sent to ground control stations
 *          - **Command Processing**: Handles MAV_CMD_* commands specific to
 *            fixed-wing and VTOL operation (takeoff, landing, altitude changes)
 *          - **Mission Management**: Waypoint navigation and mission item handling
 *          - **Guided Mode Control**: Position/attitude target processing for
 *            real-time flight control from GCS
 *          - **Mode Enumeration**: Reports available flight modes to GCS
 *          - **Capability Reporting**: Advertises vehicle capabilities to GCS
 *          
 *          The class handles both standard fixed-wing operations and QuadPlane
 *          VTOL-specific functionality when HAL_QUADPLANE_ENABLED is defined.
 * 
 * @note Inherits streaming rate configuration, message routing, and base
 *       MAVLink protocol handling from GCS_MAVLINK parent class
 * 
 * @warning This class is instantiated per channel at boot time and methods
 *          are called at varying rates (1-50Hz) by the main scheduler. Ensure
 *          all operations are non-blocking and timing-aware.
 * 
 * @see GCS_MAVLink base class for protocol implementation details
 * @see plane.gcs().send_text() for status message output
 * 
 * Source: ArduPlane/GCS_MAVLink_Plane.h:9-106
 */
class GCS_MAVLINK_Plane : public GCS_MAVLINK
{

public:

    /**
     * @brief Use inherited constructor from GCS_MAVLINK base class
     * @details Inherits channel initialization, buffer allocation, and
     *          MAVLink protocol setup from parent class
     */
    using GCS_MAVLINK::GCS_MAVLINK;

protected:

#if HAL_LOGGING_ENABLED
    /**
     * @brief Return logging mask bit for radio (RC input) messages
     * @return MASK_LOG_PM bit indicating performance monitoring log mask
     * @details Used by logging system to determine if radio/RC data should
     *          be logged. MASK_LOG_PM enables performance monitoring logs
     *          which include RC input data.
     */
    uint32_t log_radio_bit() const override { return MASK_LOG_PM; }
#endif

#if AP_MAVLINK_MISSION_SET_CURRENT_ENABLED
    /**
     * @brief Handle MISSION_SET_CURRENT MAVLink message
     * @param[in,out] mission Reference to AP_Mission object managing waypoints
     * @param[in] msg MAVLink message containing mission index to set as current
     * @details Processes request to change current active mission item. Validates
     *          requested index and updates mission state machine. Used by GCS to
     *          skip waypoints or restart missions.
     */
    void handle_mission_set_current(AP_Mission &mission, const mavlink_message_t &msg) override;
#endif

    /**
     * @brief Handle MAV_CMD_PREFLIGHT_CALIBRATION command
     * @param[in] packet COMMAND_INT packet containing calibration parameters
     * @param[in] msg Complete MAVLink message for context
     * @return MAV_RESULT indicating success, failure, or acceptance status
     * @details Processes pre-flight sensor calibration commands including:
     *          - Gyroscope calibration (param1)
     *          - Magnetometer calibration (param2)
     *          - Barometer/ground pressure calibration (param3)
     *          - Radio trim calibration (param4)
     *          - Accelerometer calibration (param5)
     *          - Airspeed sensor calibration (param7)
     * @warning Vehicle must be stationary for accurate calibration
     * @note Calibration procedures can take 5-60 seconds depending on sensor type
     */
    MAV_RESULT handle_command_preflight_calibration(const mavlink_command_int_t &packet, const mavlink_message_t &msg) override;
    
    /**
     * @brief Main entry point for COMMAND_INT MAVLink message processing
     * @param[in] packet COMMAND_INT packet containing command ID and parameters
     * @param[in] msg Complete MAVLink message for context
     * @return MAV_RESULT indicating command acceptance, success, or failure
     * @details Routes COMMAND_INT messages to appropriate Plane-specific handlers.
     *          Handles commands not processed by base class, including:
     *          - DO_REPOSITION for guided mode repositioning
     *          - DO_CHANGE_ALTITUDE for altitude adjustments
     *          - NAV_TAKEOFF for VTOL/QuadPlane takeoffs
     *          - VTOL transition commands
     *          - Motor test commands
     * @note Called by MAVLink message parser when COMMAND_INT received
     */
    MAV_RESULT handle_command_int_packet(const mavlink_command_int_t &packet, const mavlink_message_t &msg) override;
    
    /**
     * @brief Handle MAV_CMD_DO_SET_MISSION_CURRENT command
     * @param[in] packet COMMAND_INT packet with mission index in param1
     * @return MAV_RESULT_ACCEPTED on success, MAV_RESULT_FAILED on invalid index
     * @details Changes the current active mission waypoint to specified index.
     *          Validates index is within mission bounds before updating.
     */
    MAV_RESULT handle_command_do_set_mission_current(const mavlink_command_int_t &packet) override;

    /**
     * @brief Send POSITION_TARGET_GLOBAL_INT message with current target position
     * @details Transmits the current guided mode position/velocity/acceleration
     *          targets in global coordinates (lat/lon/alt). Used by GCS to display
     *          where the vehicle is trying to fly in guided mode.
     * @note Only sends valid data when vehicle is in guided mode
     * @see handle_set_position_target_global_int() for target setting
     */
    void send_position_target_global_int() override;

    /**
     * @brief Send AOA_SSA message with angle of attack and sideslip angle
     * @details Transmits current aerodynamic angles:
     *          - Angle of Attack (AOA): Angle between wing chord and airflow
     *          - Sideslip Angle (SSA): Lateral airflow angle
     *          Values are calculated from airspeed vector and vehicle attitude.
     *          Used for aerodynamic monitoring and stall warning.
     * @note Requires valid airspeed and attitude data
     * @note Message only sent if airspeed is above minimum threshold
     */
    void send_aoa_ssa();
    
    /**
     * @brief Send ATTITUDE message with current vehicle orientation
     * @details Transmits current vehicle attitude (roll, pitch, yaw) and
     *          rotation rates from AHRS system. Sent at STREAM_EXTRA1 rate
     *          (typically 4-10Hz depending on telemetry configuration).
     * @note Attitude values in radians, rates in rad/s
     */
    void send_attitude() const override;
    
    /**
     * @brief Send ATTITUDE_TARGET message with desired attitude setpoint
     * @details Transmits the current attitude controller target (desired
     *          roll, pitch, yaw) and thrust. Used by GCS to visualize what
     *          the autopilot is commanding vs actual attitude.
     * @note Only sends meaningful data when attitude control is active
     */
    void send_attitude_target() override;
    
    /**
     * @brief Send WIND message with estimated wind velocity
     * @details Transmits current wind speed and direction estimate from
     *          EKF or AHRS wind estimation. Wind is reported in NED frame:
     *          - Direction: degrees from north (0-360)
     *          - Speed: m/s
     *          - Vertical component: m/s (positive = updraft)
     * @note Requires valid airspeed sensor for accurate wind estimation
     */
    void send_wind() const;

    /**
     * @brief Indicate that stream rate configuration should persist to storage
     * @return true - stream rates are saved to EEPROM/flash
     * @details Plane saves configured MAVLink stream rates to non-volatile
     *          storage so they persist across reboots. This allows GCS to
     *          configure telemetry rates once rather than on every connection.
     */
    bool persist_streamrates() const override { return true; }

    /**
     * @brief Report vehicle capabilities to GCS via AUTOPILOT_VERSION message
     * @return Bitmask of MAV_PROTOCOL_CAPABILITY flags
     * @details Returns capability flags indicating supported features:
     *          - MISSION_FLOAT: Mission items with floating point values
     *          - PARAM_FLOAT: Floating point parameters
     *          - MISSION_INT: Mission items with integer coordinates
     *          - COMMAND_INT: Integer command support
     *          - FTP: MAVLink file transfer protocol
     *          - SET_ATTITUDE_TARGET: Attitude control via MAVLink
     *          - SET_POSITION_TARGET: Position control via MAVLink
     *          - TERRAIN: Terrain following support
     *          Plus QuadPlane-specific capabilities when VTOL enabled
     * @note Capabilities vary based on compilation options (HAL_QUADPLANE_ENABLED, etc.)
     */
    uint64_t capabilities() const override;

    /**
     * @brief Send NAV_CONTROLLER_OUTPUT message with navigation status
     * @details Transmits current navigation controller state including:
     *          - nav_roll: Desired bank angle (centidegrees)
     *          - nav_pitch: Desired pitch angle (centidegrees)
     *          - nav_bearing: Target bearing from vehicle (centidegrees)
     *          - target_bearing: Absolute target bearing (centidegrees)
     *          - wp_dist: Distance to active waypoint (meters)
     *          - alt_error: Altitude error (meters)
     *          - aspd_error: Airspeed error (m/s)
     *          - xtrack_error: Cross-track error (meters)
     * @note Sent at STREAM_POSITION rate (typically 2-5Hz)
     */
    void send_nav_controller_output() const override;
    
    /**
     * @brief Send PID_TUNING messages for active controller
     * @details Transmits real-time PID controller state for tuning purposes.
     *          Sends data for roll, pitch, yaw, steering, or throttle controller
     *          depending on which axis is selected for tuning via parameter.
     *          Includes desired value, achieved value, and P/I/D/FF terms.
     * @note Requires PID tuning to be enabled via TUNE_PARAM parameter
     * @note High bandwidth - typically sent at 10-25Hz when enabled
     */
    void send_pid_tuning() override;

    /**
     * @brief Process MANUAL_CONTROL message for joystick/RC override input
     * @param[in] packet MANUAL_CONTROL message with axis values (-1000 to 1000)
     * @param[in] tnow Current system time in milliseconds
     * @details Handles manual control input from GCS joystick, allowing direct
     *          control of vehicle from ground station. Processes pitch, roll,
     *          yaw, and throttle axes with appropriate scaling and mode logic.
     * @warning Overrides RC input when active - ensure safe operation
     * @note Input range: -1000 to 1000 per axis, scaled to servo/motor outputs
     */
    void handle_manual_control_axes(const mavlink_manual_control_t &packet, const uint32_t tnow) override;
    
    /**
     * @brief Process LANDING_TARGET message for precision landing
     * @param[in] packet LANDING_TARGET message with target position/size
     * @param[in] timestamp_ms Message timestamp in milliseconds
     * @details Handles visual landing target position updates from companion
     *          computer or ground-based beacon detector. Target information
     *          is passed to precision landing library (AC_PrecLand) to guide
     *          final approach and touchdown.
     * @note Requires precision landing to be enabled and calibrated
     * @note Target angles in radians, distance in meters
     */
    void handle_landing_target(const mavlink_landing_target_t &packet, uint32_t timestamp_ms) override;

    /**
     * @brief Send available flight mode information to GCS
     * @param[in] index Mode index to report (1-based, not mode number)
     * @return Total number of available modes
     * @details Reports flight mode at given index for GCS mode enumeration.
     *          Used by modern GCS to build dynamic mode selection UI showing
     *          only modes available on this vehicle. Index 1 = first mode,
     *          not mode number. Returns total mode count regardless of index.
     * @note Index starts at 1, not 0 (index 0 is invalid)
     * @note Mode availability depends on configuration (VTOL, autoland, etc.)
     */
    uint8_t send_available_mode(uint8_t index) const override;

private:

    /**
     * @brief Send PID_TUNING message for specified controller axis
     * @param[in] pid_info Pointer to PID controller state (P/I/D/FF values)
     * @param[in] axis Axis identifier (MAV_PID_TUNING_* constant)
     * @param[in] achieved Actual achieved value for this control axis
     * @details Helper function to format and transmit PID tuning telemetry.
     *          Packs PID gains, desired value, achieved value, and individual
     *          term contributions into PID_TUNING message.
     * @note Called by send_pid_tuning() for active tuning axis
     */
    void send_pid_info(const struct AP_PIDInfo *pid_info, const uint8_t axis, const float achieved);

    /**
     * @brief Main MAVLink message dispatcher for Plane-specific messages
     * @param[in] msg Received MAVLink message to process
     * @details Routes incoming MAVLink messages to appropriate Plane-specific
     *          handlers. Processes messages not handled by base GCS_MAVLink class:
     *          - SET_POSITION_TARGET_GLOBAL_INT (guided mode positioning)
     *          - SET_POSITION_TARGET_LOCAL_NED (local frame positioning)
     *          - SET_ATTITUDE_TARGET (attitude control)
     *          Falls through to base class handler for standard messages.
     * @note Called at message receive rate (varies by telemetry link speed)
     */
    void handle_message(const mavlink_message_t &msg) override;
    
    /**
     * @brief Process guided mode mission command from GCS
     * @param[in,out] cmd Mission command structure to validate and execute
     * @return true if command accepted and executed, false if rejected
     * @details Validates and executes mission-style commands sent in guided mode,
     *          including navigation commands (waypoint, loiter, circle) and
     *          conditional commands. Checks parameter validity and vehicle state.
     * @note Only processes commands when vehicle is in guided mode
     */
    bool handle_guided_request(AP_Mission::Mission_Command &cmd) override;
    
    /**
     * @brief Handle altitude change request in guided mode
     * @param[in,out] location Target location with new altitude, updated with current lat/lon
     * @details Processes altitude change commands in guided mode. Preserves
     *          current horizontal position (lat/lon) while updating target
     *          altitude. Location is modified to include current position.
     * @note Altitude frame preserved (AGL, AMSL, or terrain relative)
     */
    void handle_change_alt_request(Location &location) override;
    /**
     * @brief Handle MAV_CMD_DO_REPOSITION command
     * @param[in] packet COMMAND_INT with target lat/lon/alt and speed parameters
     * @return MAV_RESULT_ACCEPTED on success, error code on failure
     * @details Repositions vehicle to new location in guided mode. Parameters:
     *          - param1: Ground speed (m/s, -1 = default)
     *          - param2: Bitmask (bit 0: hold at destination)
     *          - param3: Loiter radius (meters, 0 = default)
     *          - param4: Yaw heading (degrees, NaN = use current)
     *          - x: Target latitude (degrees * 1e7)
     *          - y: Target longitude (degrees * 1e7)
     *          - z: Target altitude (meters)
     * @note Switches vehicle to guided mode if not already in guided
     */
    MAV_RESULT handle_command_int_do_reposition(const mavlink_command_int_t &packet);
    
    /**
     * @brief Handle MAV_CMD_DO_CHANGE_ALTITUDE command
     * @param[in] packet COMMAND_INT with target altitude in param1, frame in param2
     * @return MAV_RESULT_ACCEPTED on success, error code on failure
     * @details Changes target altitude while maintaining current horizontal position.
     *          - param1: Target altitude (meters)
     *          - param2: Frame (0=AMSL, 1=AGL)
     * @note Only works in guided mode or auto mode
     */
    MAV_RESULT handle_command_int_DO_CHANGE_ALTITUDE(const mavlink_command_int_t &packet);
    
    /**
     * @brief Handle guided mode slew rate commands (MAV_CMD_GUIDED_CHANGE_*)
     * @param[in] packet COMMAND_INT with slew rate parameters
     * @return MAV_RESULT_ACCEPTED on success, error code on failure
     * @details Processes gradual rate-limited changes to guided targets:
     *          - MAV_CMD_GUIDED_CHANGE_SPEED: Change speed with specified slew rate
     *          - MAV_CMD_GUIDED_CHANGE_ALTITUDE: Change altitude with rate limit
     *          - MAV_CMD_GUIDED_CHANGE_HEADING: Change heading with rate limit
     * @note Provides smoother transitions than immediate target changes
     */
    MAV_RESULT handle_command_int_guided_slew_commands(const mavlink_command_int_t &packet);
    
    /**
     * @brief Handle MAV_CMD_DO_AUTOTUNE_ENABLE command
     * @param[in] packet COMMAND_INT with enable flag in param1, axis in param2
     * @return MAV_RESULT_ACCEPTED on success, error code on failure
     * @details Enables or disables automatic PID tuning for specified axis.
     *          - param1: 1=enable, 0=disable, 2=toggle
     *          - param2: Axis bitmask (1=roll, 2=pitch, 4=yaw)
     * @warning Autotune maneuvers vehicle aggressively - ensure safe altitude
     * @note Requires AUTOTUNE flight mode to be available
     */
    MAV_RESULT handle_MAV_CMD_DO_AUTOTUNE_ENABLE(const mavlink_command_int_t &packet);
    
    /**
     * @brief Handle MAV_CMD_DO_CHANGE_SPEED command
     * @param[in] packet COMMAND_INT with speed type and target speed
     * @return MAV_RESULT_ACCEPTED on success, error code on failure
     * @details Changes target speed for current flight mode. Parameters:
     *          - param1: Speed type (0=Airspeed, 1=Ground speed)
     *          - param2: Target speed (m/s, -1=no change)
     *          - param3: Throttle percentage (-1=no change)
     * @note Changes persist until mode change or next speed command
     */
    MAV_RESULT handle_command_DO_CHANGE_SPEED(const mavlink_command_int_t &packet);
    
    /**
     * @brief Handle MAV_CMD_DO_MOTOR_TEST command
     * @param[in] packet COMMAND_INT with motor number and test parameters
     * @return MAV_RESULT_ACCEPTED on success, error code on failure
     * @details Commands individual motor/servo test for diagnostic purposes.
     *          - param1: Motor sequence number (1-8)
     *          - param2: Throttle type (0=percent, 1=PWM, 2=pilot)
     *          - param3: Throttle/PWM value
     *          - param4: Test duration (seconds)
     *          - param5: Motor count (0=all motors)
     * @warning Vehicle must be disarmed or armed with safety switch enabled
     * @warning Propellers may spin - ensure safe conditions
     */
    MAV_RESULT handle_MAV_CMD_DO_MOTOR_TEST(const mavlink_command_int_t &packet);
    
    /**
     * @brief Handle MAV_CMD_DO_PARACHUTE command
     * @param[in] packet COMMAND_INT with parachute action in param1
     * @return MAV_RESULT_ACCEPTED on success, error code on failure
     * @details Controls parachute deployment system. Actions:
     *          - 0: Disable
     *          - 1: Enable
     *          - 2: Release/deploy parachute
     * @warning Release action is irreversible - parachute will deploy
     * @note Requires parachute hardware to be configured
     */
    MAV_RESULT handle_MAV_CMD_DO_PARACHUTE(const mavlink_command_int_t &packet);
    
    /**
     * @brief Handle MAV_CMD_DO_VTOL_TRANSITION command
     * @param[in] packet COMMAND_INT with transition state in param1
     * @return MAV_RESULT_ACCEPTED on success, error code on failure
     * @details Commands VTOL transition for QuadPlane vehicles:
     *          - param1: 3=Fixed-wing mode, 4=Multicopter mode
     * @note Only available when HAL_QUADPLANE_ENABLED is defined
     * @note Transition may take several seconds and requires safe altitude
     */
    MAV_RESULT handle_command_DO_VTOL_TRANSITION(const mavlink_command_int_t &packet);

    /**
     * @brief Handle SET_POSITION_TARGET_GLOBAL_INT message for guided mode
     * @param[in] msg MAVLink message with global position/velocity targets
     * @details Processes position/velocity/acceleration targets in global
     *          coordinates (lat/lon/alt). Target can specify position,
     *          velocity, or acceleration in any combination using type_mask.
     *          Switches vehicle to guided mode if necessary.
     * @note Position in degrees*1e7, velocity in m/s, acceleration in m/s²
     * @note Coordinate frame specified in coordinate_frame field
     */
    void handle_set_position_target_global_int(const mavlink_message_t &msg);
    
    /**
     * @brief Handle SET_POSITION_TARGET_LOCAL_NED message for guided mode
     * @param[in] msg MAVLink message with local NED position/velocity targets
     * @details Processes position/velocity/acceleration targets in local
     *          NED frame (North-East-Down) relative to home position.
     *          Type_mask field selects which components are active targets.
     * @note All units in meters and m/s in NED frame
     * @note Origin is vehicle home position or current location if no home set
     */
    void handle_set_position_target_local_ned(const mavlink_message_t &msg);
    
    /**
     * @brief Handle SET_ATTITUDE_TARGET message for direct attitude control
     * @param[in] msg MAVLink message with attitude quaternion and rates
     * @details Processes direct attitude control commands with thrust.
     *          Allows GCS or companion computer to command specific attitudes
     *          and rotation rates. Used for advanced control modes and research.
     * @note Quaternion attitude, rates in rad/s, thrust 0.0-1.0
     * @warning Bypasses normal flight mode logic - use with caution
     * @note Requires vehicle to be in guided mode
     */
    void handle_set_attitude_target(const mavlink_message_t &msg);

#if HAL_QUADPLANE_ENABLED
#if AP_MAVLINK_COMMAND_LONG_ENABLED
    /**
     * @brief Convert MAV_CMD_NAV_TAKEOFF from COMMAND_LONG to COMMAND_INT
     * @param[in] in COMMAND_LONG format takeoff command
     * @param[out] out COMMAND_INT format takeoff command with converted coordinates
     * @details Converts legacy COMMAND_LONG takeoff command to modern COMMAND_INT
     *          format. Handles coordinate conversion from float degrees to
     *          integer degrees*1e7 for lat/lon fields. QuadPlane-specific version
     *          handles VTOL takeoff parameters.
     * @note Required for backward compatibility with older GCS software
     */
    void convert_MAV_CMD_NAV_TAKEOFF_to_COMMAND_INT(const mavlink_command_long_t &in, mavlink_command_int_t &out);
    
    /**
     * @brief Convert generic COMMAND_LONG to COMMAND_INT format
     * @param[in] in COMMAND_LONG format command
     * @param[out] out COMMAND_INT format command with converted coordinates
     * @param[in] frame Coordinate frame to use (default: MAV_FRAME_GLOBAL_RELATIVE_ALT)
     * @details General conversion from legacy float-based COMMAND_LONG to
     *          integer-based COMMAND_INT. Converts lat/lon from degrees to
     *          degrees*1e7. QuadPlane override handles VTOL-specific commands.
     */
    void convert_COMMAND_LONG_to_COMMAND_INT(const mavlink_command_long_t &in, mavlink_command_int_t &out, MAV_FRAME frame = MAV_FRAME_GLOBAL_RELATIVE_ALT) override;
#endif
    /**
     * @brief Handle MAV_CMD_NAV_TAKEOFF command for QuadPlane VTOL takeoff
     * @param[in] packet COMMAND_INT with takeoff parameters
     * @return MAV_RESULT_ACCEPTED on success, error code on failure
     * @details Executes QuadPlane vertical takeoff sequence. Parameters:
     *          - param1: Minimum pitch (degrees, for fixed-wing takeoff)
     *          - param2: Takeoff mode (0=default, 1=VTOL)
     *          - x: Target latitude after takeoff (degrees*1e7)
     *          - y: Target longitude after takeoff (degrees*1e7)
     *          - z: Target altitude AGL (meters)
     * @note QuadPlane will transition to fixed-wing after reaching altitude
     * @warning Requires sufficient space for vertical takeoff and transition
     */
    MAV_RESULT handle_command_MAV_CMD_NAV_TAKEOFF(const mavlink_command_int_t &packet);
#endif

    /**
     * @brief Attempt to send a queued telemetry message
     * @param[in] id Message identifier from ap_message enum
     * @return true if message sent successfully, false if deferred
     * @details Called by scheduler to send queued telemetry messages based
     *          on stream rates. Handles Plane-specific messages (AOA_SSA, wind)
     *          and falls through to base class for standard messages.
     * @note Message send rate controlled by stream rate configuration
     * @note Returns false if insufficient bandwidth to send now
     */
    bool try_send_message(enum ap_message id) override;
    
    /**
     * @brief Notification that a complete MAVLink packet was received
     * @param[in] status MAVLink parser status (sequence numbers, errors)
     * @param[in] msg Complete parsed MAVLink message
     * @details Called for every successfully received MAVLink packet.
     *          Used for link quality monitoring, sequence tracking, and
     *          statistics. Updates telemetry radio RSSI if available.
     * @note Called before handle_message() processes message content
     */
    void packetReceived(const mavlink_status_t &status, const mavlink_message_t &msg) override;

    /**
     * @brief Get base mode flags for HEARTBEAT message
     * @return MAV_MODE_FLAG bitmask indicating vehicle state
     * @details Returns base mode flags:
     *          - MAV_MODE_FLAG_CUSTOM_MODE_ENABLED: Always set (use custom mode)
     *          - MAV_MODE_FLAG_AUTO_ENABLED: Set in auto modes
     *          - MAV_MODE_FLAG_GUIDED_ENABLED: Set in guided mode
     *          - MAV_MODE_FLAG_STABILIZE_ENABLED: Set in stabilized modes
     *          - MAV_MODE_FLAG_MANUAL_ENABLED: Set in manual mode
     *          - MAV_MODE_FLAG_SAFETY_ARMED: Set when vehicle armed
     * @note Sent in HEARTBEAT at 1Hz
     */
    uint8_t base_mode() const override;
    
    /**
     * @brief Get vehicle system status for HEARTBEAT message
     * @return MAV_STATE indicating current system state
     * @details Returns one of:
     *          - MAV_STATE_UNINIT: System initializing
     *          - MAV_STATE_BOOT: Bootloader active
     *          - MAV_STATE_CALIBRATING: Sensor calibration in progress
     *          - MAV_STATE_STANDBY: Ready to arm but not armed
     *          - MAV_STATE_ACTIVE: Armed and operational
     *          - MAV_STATE_CRITICAL: Critical failure requiring immediate landing
     *          - MAV_STATE_EMERGENCY: Emergency state, may lose control
     * @note Sent in HEARTBEAT at 1Hz
     */
    MAV_STATE vehicle_system_status() const override;

    /**
     * @brief Get airspeed for VFR_HUD message
     * @return Current airspeed in m/s
     * @details Returns indicated airspeed from airspeed sensor, or estimated
     *          airspeed from GPS if sensor unavailable. Used in VFR_HUD for
     *          primary flight display in GCS.
     * @note Returns groundspeed if no airspeed sensor and no wind estimate
     */
    float vfr_hud_airspeed() const override;
    
    /**
     * @brief Get throttle percentage for VFR_HUD message
     * @return Throttle percentage 0-100
     * @details Returns current throttle output as percentage of maximum.
     *          For fixed-wing, this is forward motor throttle. For QuadPlane
     *          in multicopter mode, returns average motor output.
     * @note Value may exceed 100 if throttle limits not enforced
     */
    int16_t vfr_hud_throttle() const override;
    
    /**
     * @brief Get climb rate for VFR_HUD message
     * @return Vertical velocity in m/s (positive = climbing)
     * @details Returns current climb rate from inertial navigation system.
     *          Positive values indicate climbing, negative descending.
     *          Filtered for smoother GCS display.
     * @note Units: m/s in NED frame (positive = down, sign inverted for display)
     */
    float vfr_hud_climbrate() const override;
    
#if HAL_HIGH_LATENCY2_ENABLED
    /**
     * @brief Get target altitude for HIGH_LATENCY2 message
     * @return Target altitude in meters AMSL
     * @details Returns current navigation target altitude for compressed
     *          telemetry over low-bandwidth links (satellite, HF radio).
     *          Value is quantized to reduce bandwidth requirements.
     * @note HIGH_LATENCY2 designed for links <1KB/s
     */
    int16_t high_latency_target_altitude() const override;
    
    /**
     * @brief Get target heading for HIGH_LATENCY2 message
     * @return Target heading in degrees (0-359) scaled to uint8_t
     * @details Returns navigation target heading compressed to single byte.
     *          Resolution: ~1.4 degrees (360/255).
     */
    uint8_t high_latency_tgt_heading() const override;
    
    /**
     * @brief Get distance to target for HIGH_LATENCY2 message
     * @return Distance to active waypoint in meters
     * @details Returns distance to current navigation target compressed
     *          for low-bandwidth telemetry. Range limited to ~65km.
     */
    uint16_t high_latency_tgt_dist() const override;
    
    /**
     * @brief Get target airspeed for HIGH_LATENCY2 message
     * @return Target airspeed in m/s scaled to uint8_t
     * @details Returns commanded airspeed for navigation, compressed to
     *          single byte. Range: 0-255 m/s.
     */
    uint8_t high_latency_tgt_airspeed() const override;
    
    /**
     * @brief Get wind speed for HIGH_LATENCY2 message
     * @return Wind speed in m/s scaled to uint8_t
     * @details Returns estimated wind speed compressed to single byte.
     *          Range: 0-255 m/s. Requires airspeed sensor for accuracy.
     */
    uint8_t high_latency_wind_speed() const override;
    
    /**
     * @brief Get wind direction for HIGH_LATENCY2 message
     * @return Wind from direction in degrees (0-359) scaled to uint8_t
     * @details Returns wind direction (where wind is coming FROM) compressed
     *          to single byte. Resolution: ~1.4 degrees. Relative to north.
     */
    uint8_t high_latency_wind_direction() const override;
#endif // HAL_HIGH_LATENCY2_ENABLED

#if AP_AIRSPEED_HYGROMETER_ENABLE
    /**
     * @brief Send HYGROMETER_SENSOR message with humidity/temperature data
     * @details Transmits atmospheric humidity and temperature from hygrometer
     *          sensor (if installed). Used for high-altitude weather monitoring
     *          and icing condition detection. Cycles through multiple sensors
     *          if available.
     * @note Requires hygrometer-equipped airspeed sensor (e.g., MS4525DO with humidity)
     */
    void send_hygrometer();
    
    /**
     * @brief Index of last hygrometer sensor sent
     * @details Tracks which sensor was sent last to cycle through multiple
     *          hygrometers in round-robin fashion. Allows monitoring of
     *          multiple humidity sensors at different locations on airframe.
     */
    uint8_t last_hygrometer_send_idx;
#endif

    /**
     * @brief Get VTOL state for EXTENDED_SYS_STATE message
     * @return MAV_VTOL_STATE indicating current VTOL mode
     * @details Returns VTOL transition state for QuadPlane:
     *          - MAV_VTOL_STATE_UNDEFINED: Not a VTOL vehicle
     *          - MAV_VTOL_STATE_TRANSITION_TO_FW: Transitioning to fixed-wing
     *          - MAV_VTOL_STATE_TRANSITION_TO_MC: Transitioning to multicopter
     *          - MAV_VTOL_STATE_MC: In multicopter mode
     *          - MAV_VTOL_STATE_FW: In fixed-wing mode
     * @note Returns MAV_VTOL_STATE_UNDEFINED if not a QuadPlane
     */
    MAV_VTOL_STATE vtol_state() const override;
    
    /**
     * @brief Get landed state for EXTENDED_SYS_STATE message
     * @return MAV_LANDED_STATE indicating current landed/flying status
     * @details Returns one of:
     *          - MAV_LANDED_STATE_UNDEFINED: State unknown
     *          - MAV_LANDED_STATE_ON_GROUND: Vehicle on ground, not moving
     *          - MAV_LANDED_STATE_IN_AIR: Vehicle airborne
     *          - MAV_LANDED_STATE_TAKEOFF: Taking off
     *          - MAV_LANDED_STATE_LANDING: Landing in progress
     * @note Used by GCS for UI display and automated procedures
     * @note Detection based on airspeed, GPS velocity, and altitude change
     */
    MAV_LANDED_STATE landed_state() const override;

};
