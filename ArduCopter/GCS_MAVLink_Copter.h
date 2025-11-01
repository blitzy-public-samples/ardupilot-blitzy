/**
 * @file GCS_MAVLink_Copter.h
 * @brief Per-link MAVLink message handler for ArduCopter multirotor vehicles
 * 
 * @details This file defines the GCS_MAVLINK_Copter class which handles MAVLink
 *          protocol communication for individual telemetry links in ArduCopter.
 *          
 *          Architecture Overview:
 *          - One GCS_MAVLINK_Copter instance exists per active communication link
 *            (e.g., separate instances for USB, telemetry radio 1, telemetry radio 2, WiFi)
 *          - Each instance processes incoming MAVLink messages from its specific link
 *          - Each instance manages outgoing telemetry stream rates and priorities
 *          - Inherits from GCS_MAVLINK base class and overrides copter-specific behaviors
 *          
 *          Primary Responsibilities:
 *          - Command Processing: Handle copter-specific MAVLink commands (takeoff, land,
 *            position control, motor test, parachute deployment, etc.)
 *          - Message Handling: Process incoming SET messages for attitude/position targets,
 *            manual control, landing target detection
 *          - Telemetry Streaming: Send copter-specific telemetry (attitude targets,
 *            position targets, navigation state, PID tuning data)
 *          - Vehicle Capabilities: Report copter-specific capabilities to ground stations
 *          - Mode Management: Handle mode changes and report available flight modes
 *          
 *          MAVLink Communication Flow:
 *          1. Incoming: MAVLink message received on link → packetReceived() → handle_message()
 *             → specific message handler (e.g., handle_message_set_position_target_local_ned)
 *          2. Outgoing: Scheduler triggers telemetry → try_send_message() → specific sender
 *             (e.g., send_position_target_local_ned) → MAVLink library → link transmission
 *          
 *          Integration Points:
 *          - GCS_MAVLink: Base class providing core MAVLink protocol handling
 *          - Copter: Main vehicle object for accessing flight modes and vehicle state
 *          - AP_Mission: Mission command execution and guided mode requests
 *          - Mode classes: Flight mode command execution (Auto, Guided, RTL, etc.)
 *          
 * @note Multiple instances of this class run concurrently, one per communication link.
 *       Thread safety is handled by the HAL UART/network drivers and GCS base class.
 * 
 * @warning This class processes safety-critical commands including flight termination,
 *          motor control, and autonomous navigation. All command handlers must validate
 *          inputs and check vehicle state before execution.
 * 
 * @see GCS_MAVLink Base MAVLink handler class
 * @see GCS_Copter.cpp Vehicle-level GCS interface
 * @see Copter Main vehicle class
 */

#pragma once

#include <GCS_MAVLink/GCS.h>
#include <AP_Winch/AP_Winch_config.h>
#include "defines.h"

#ifndef AC_MAVLINK_SOLO_BUTTON_COMMAND_HANDLING_ENABLED
#define AC_MAVLINK_SOLO_BUTTON_COMMAND_HANDLING_ENABLED 1
#endif

/**
 * @class GCS_MAVLINK_Copter
 * @brief Per-link MAVLink protocol handler for ArduCopter multirotor vehicles
 * 
 * @details This class provides copter-specific implementation of the MAVLink communication
 *          protocol for a single telemetry link. The GCS (Ground Control System) creates
 *          one instance of this class for each active communication channel (USB, radio,
 *          network), allowing simultaneous connections from multiple ground stations or
 *          companion computers.
 *          
 *          Key Design Concepts:
 *          
 *          Per-Link Architecture:
 *          - Each communication channel (USB serial, telemetry radio, WiFi/Ethernet) has
 *            its own GCS_MAVLINK_Copter instance with independent message queues and
 *            streaming rates
 *          - This allows different ground stations to request different telemetry rates
 *            and control the vehicle simultaneously (with priority/override logic)
 *          - Link-specific configuration includes stream rates, message priorities, and
 *            protocol options (MAVLink 1 vs 2, signing, forwarding)
 *          
 *          Command Handling Flow:
 *          - Commands arrive as COMMAND_INT or COMMAND_LONG MAVLink messages
 *          - Base class routes to appropriate handler based on MAV_CMD id
 *          - Copter-specific handlers validate command parameters and vehicle state
 *          - Commands may be executed immediately or passed to flight mode for sequencing
 *          - Command result (ACCEPTED/DENIED/etc.) sent back via COMMAND_ACK message
 *          
 *          Message Handling:
 *          - SET_POSITION_TARGET and SET_ATTITUDE_TARGET messages enable external control
 *          - MANUAL_CONTROL messages provide joystick/RC-over-MAVLink input
 *          - LANDING_TARGET messages support precision landing with IR beacons
 *          - Messages update vehicle state or guided mode targets
 *          
 *          Telemetry Streaming:
 *          - Scheduler calls try_send_message() at configured rates for each message type
 *          - Copter-specific telemetry includes attitude/position targets, nav state,
 *            PID tuning data, wind estimates, motor status
 *          - High Latency 2 protocol support for bandwidth-constrained links
 *          - VFR_HUD message provides synthetic airspeed, altitude, and throttle for GCS
 *          
 *          Vehicle Capabilities:
 *          - Reports copter capabilities to ground stations (guided mode, terrain following,
 *            mission commands supported, etc.)
 *          - VTOL state always reports MAV_VTOL_STATE_MC (multicopter)
 *          - Landed state detection based on copter landing detector
 *          
 *          Safety Features:
 *          - Flight termination command support for emergency motor shutoff
 *          - Sanity checking on position/velocity commands to prevent invalid targets
 *          - Pre-arm and preflight calibration command handling
 *          - Parameter system integration for safe configuration
 *          
 *          Copter-Specific Commands Handled:
 *          - NAV_TAKEOFF: Automated takeoff to specified altitude
 *          - CONDITION_YAW: Yaw to absolute heading or relative angle
 *          - DO_CHANGE_SPEED: Modify auto mode speed parameters
 *          - DO_SET_ROI: Point camera/vehicle at region of interest
 *          - DO_REPOSITION: Change position in guided mode
 *          - DO_PAUSE_CONTINUE: Pause/resume mission execution
 *          - DO_MOTOR_TEST: Individual motor testing (pre-arm only)
 *          - DO_PARACHUTE: Manual parachute deployment
 *          - DO_WINCH: Winch control for cargo delivery
 *          - SOLO_BTN_*: 3DR Solo smart shot button commands (if enabled)
 *          
 *          Thread Safety:
 *          - Called from main vehicle thread during scheduler execution
 *          - Message reception triggered by UART/network interrupts via HAL
 *          - No explicit locking required as single-threaded at vehicle level
 * 
 * @note Creating this class does not establish a connection; the link must be opened
 *       and MAVLink messages must be actively received/sent through the HAL UART/network
 *       interface associated with this instance.
 * 
 * @warning Command handlers modify vehicle state and control. Improper handling could
 *          result in unsafe vehicle behavior. All handlers must validate inputs and
 *          verify vehicle is in appropriate state before executing commands.
 * 
 * @see GCS_MAVLINK Base class providing core MAVLink protocol implementation
 * @see GCS Common ground control system interface across all vehicle types
 * @see AP_Mission Mission command execution system
 * @see Copter Main vehicle class with flight modes and control
 */
class GCS_MAVLINK_Copter : public GCS_MAVLINK
{

public:

    using GCS_MAVLINK::GCS_MAVLINK;

protected:

    // ========================================================================
    // Command Handlers - Process MAVLink commands from ground station
    // ========================================================================

    /**
     * @brief Handle MAV_CMD_DO_FLIGHTTERMINATION command for emergency motor shutoff
     * 
     * @details Processes flight termination request which immediately disarms motors
     *          and prevents re-arming. This is a safety-critical command typically
     *          used as last resort when vehicle is out of control or in unsafe situation.
     *          
     *          Command behavior:
     *          - Immediately disarms all motors (emergency stop)
     *          - Sets termination flag preventing re-arm without reboot
     *          - Logs termination event for post-flight analysis
     *          - Triggers termination notification on all GCS links
     * 
     * @param[in] packet MAVLink COMMAND_INT message containing termination parameters
     * 
     * @return MAV_RESULT_ACCEPTED if termination executed successfully
     * @return MAV_RESULT_FAILED if termination could not be executed
     * 
     * @warning This command causes immediate loss of control and vehicle will fall.
     *          Only use when vehicle safety is already compromised.
     * 
     * @see Copter::disarm_motors()
     */
    MAV_RESULT handle_flight_termination(const mavlink_command_int_t &packet) override;

    /**
     * @brief Check if vehicle parameters are loaded and ready for operation
     * 
     * @details Called by base class to determine if vehicle is ready to process
     *          commands that depend on parameter values. Parameters are loaded
     *          from persistent storage during boot sequence.
     * 
     * @return true if all parameters loaded successfully and ready
     * @return false if parameter system still initializing
     * 
     * @note Most commands will be rejected if parameters not ready
     */
    bool params_ready() const override;

    /**
     * @brief Send vehicle identification banner message on link connection
     * 
     * @details Sends vehicle type, firmware version, and capabilities to ground
     *          station when link first establishes communication. This allows
     *          GCS to identify vehicle type and adjust UI appropriately.
     * 
     * @note Called automatically by base class when new GCS connects
     */
    void send_banner() override;

    /**
     * @brief Handle MAV_CMD_PREFLIGHT_CALIBRATION for sensor calibration
     * 
     * @details Processes preflight calibration commands for gyros, accelerometers,
     *          compass, barometer, and other sensors. Calibration must be performed
     *          with vehicle stationary on level surface.
     *          
     *          Supported calibrations:
     *          - Gyro calibration: Zero-rate offset calculation
     *          - Accelerometer calibration: 6-point or 1-point calibration
     *          - Compass calibration: Start/stop compass learn
     *          - Barometer: Ground level pressure reference
     *          - Airspeed: Zero-pressure offset (copter has no airspeed sensor)
     * 
     * @param[in] packet COMMAND_INT with calibration type in parameters
     * @param[in] msg   Original MAVLink message for additional context
     * 
     * @return MAV_RESULT_ACCEPTED if calibration started successfully
     * @return MAV_RESULT_DENIED if vehicle not in valid state for calibration
     * @return MAV_RESULT_UNSUPPORTED if calibration type not supported
     * 
     * @warning Vehicle must be disarmed and stationary for accurate calibration
     */
    MAV_RESULT _handle_command_preflight_calibration(const mavlink_command_int_t &packet, const mavlink_message_t &msg) override;

    // ========================================================================
    // Telemetry Senders - Stream vehicle state to ground station
    // ========================================================================

    /**
     * @brief Send current attitude control target to ground station
     * 
     * @details Streams ATTITUDE_TARGET message containing desired roll, pitch, yaw
     *          rates and thrust being commanded by attitude controller. Useful for
     *          tuning and debugging attitude control performance.
     * 
     * @note Only sent when vehicle in mode using attitude controller
     * @note Streaming rate controlled by SR0_EXTRA1 parameter
     */
    void send_attitude_target() override;

    /**
     * @brief Send current position target in global coordinates (lat/lon/alt)
     * 
     * @details Streams POSITION_TARGET_GLOBAL_INT message with current guided mode
     *          or auto mode position target in WGS84 coordinates. Includes target
     *          velocities and accelerations if available.
     * 
     * @note Only sent when vehicle in guided or auto mode with position target
     * @note Streaming rate controlled by SR0_POSITION parameter
     */
    void send_position_target_global_int() override;

    /**
     * @brief Send current position target in local NED coordinates
     * 
     * @details Streams POSITION_TARGET_LOCAL_NED message with current guided mode
     *          target position relative to home location. NED frame: North, East, Down
     *          with origin at home position.
     * 
     * @note Only sent when vehicle in guided mode with local position target
     * @note Streaming rate controlled by SR0_POSITION parameter
     */
    void send_position_target_local_ned() override;

    // ========================================================================
    // Additional Command Handlers
    // ========================================================================

    /**
     * @brief Handle MAV_CMD_DO_SET_ROI command to point camera/vehicle at target
     * 
     * @details Configures Region Of Interest (ROI) for camera gimbal or vehicle yaw.
     *          In copter, ROI typically controls yaw to point vehicle at location
     *          while gimbal (if present) points camera at target.
     * 
     * @param[in] roi_loc Target location in global coordinates (lat/lon/alt)
     * 
     * @return MAV_RESULT_ACCEPTED if ROI set successfully
     * @return MAV_RESULT_DENIED if vehicle not in mode supporting ROI
     * 
     * @note Supported in Auto, Guided, and Loiter modes
     */
    MAV_RESULT handle_command_do_set_roi(const Location &roi_loc) override;

    /**
     * @brief Handle MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN for board reboot
     * 
     * @details Safely reboots flight controller or shuts down (if supported).
     *          Ensures vehicle is disarmed before allowing reboot.
     * 
     * @param[in] packet COMMAND_INT with reboot type in parameters
     * @param[in] msg   Original MAVLink message
     * 
     * @return MAV_RESULT_ACCEPTED if reboot initiated
     * @return MAV_RESULT_DENIED if vehicle armed or conditions unsafe
     */
    MAV_RESULT handle_preflight_reboot(const mavlink_command_int_t &packet, const mavlink_message_t &msg) override;

#if HAL_MOUNT_ENABLED
    /**
     * @brief Handle MAV_CMD_DO_MOUNT_CONTROL and related mount commands
     * 
     * @details Processes commands for camera gimbal/mount control including
     *          pointing angles, tracking modes, and stabilization settings.
     * 
     * @param[in] packet COMMAND_INT with mount parameters
     * @param[in] msg   Original MAVLink message
     * 
     * @return MAV_RESULT based on mount driver response
     * 
     * @note Only compiled if HAL_MOUNT_ENABLED (gimbal support)
     */
    MAV_RESULT handle_command_mount(const mavlink_command_int_t &packet, const mavlink_message_t &msg) override;
#endif

    /**
     * @brief Main router for COMMAND_INT messages to copter-specific handlers
     * 
     * @details Routes incoming COMMAND_INT messages to appropriate handler based
     *          on command ID (MAV_CMD). Handles copter-specific commands and
     *          passes others to base class for common handling.
     * 
     * @param[in] packet COMMAND_INT message with command parameters
     * @param[in] msg   Original MAVLink message for logging/context
     * 
     * @return MAV_RESULT from specific command handler
     * 
     * @note This is the main entry point for all MAVLink commands
     */
    MAV_RESULT handle_command_int_packet(const mavlink_command_int_t &packet, const mavlink_message_t &msg) override;

    /**
     * @brief Handle MAV_CMD_DO_REPOSITION to change position in guided mode
     * 
     * @details Commands vehicle to fly to new position while in guided mode.
     *          Can specify position in global (lat/lon/alt) coordinates with
     *          optional ground speed and heading.
     * 
     * @param[in] packet COMMAND_INT with target position and optional velocity/heading
     * 
     * @return MAV_RESULT_ACCEPTED if reposition command accepted
     * @return MAV_RESULT_DENIED if not in guided mode or invalid parameters
     * 
     * @note Only works in Guided mode
     */
    MAV_RESULT handle_command_int_do_reposition(const mavlink_command_int_t &packet);

    /**
     * @brief Handle MAV_CMD_DO_PAUSE_CONTINUE for mission pause/resume
     * 
     * @details Pauses or resumes mission execution in Auto mode. When paused,
     *          vehicle holds position. When resumed, continues to next waypoint.
     * 
     * @param[in] packet COMMAND_INT with pause (0) or continue (1) parameter
     * 
     * @return MAV_RESULT_ACCEPTED if pause/continue executed
     * @return MAV_RESULT_DENIED if not in Auto mode
     * 
     * @note Only functional in Auto mode during mission execution
     */
    MAV_RESULT handle_command_pause_continue(const mavlink_command_int_t &packet);

    // ========================================================================
    // Message Handlers - Process SET messages for external control
    // ========================================================================

#if HAL_MOUNT_ENABLED
    /**
     * @brief Handle MOUNT_CONFIGURE and MOUNT_CONTROL messages
     * 
     * @details Processes mount/gimbal control messages for camera pointing.
     *          Alternative to command-based mount control using streaming messages.
     * 
     * @param[in] msg MAVLink message containing mount control data
     * 
     * @note Only compiled if HAL_MOUNT_ENABLED
     */
    void handle_mount_message(const mavlink_message_t &msg) override;
#endif

    /**
     * @brief Handle SET_ATTITUDE_TARGET message for external attitude control
     * 
     * @details Processes attitude target messages from external controller (companion
     *          computer, ground station). Sets desired roll, pitch, yaw rates and
     *          thrust in guided mode. Enables offboard attitude control for advanced
     *          applications.
     * 
     * @param[in] msg MAVLink SET_ATTITUDE_TARGET message
     * 
     * @warning Must be called at >2Hz to maintain control, otherwise vehicle
     *          will revert to position hold for safety
     * 
     * @note Only effective in Guided mode
     */
    void handle_message_set_attitude_target(const mavlink_message_t &msg);

    /**
     * @brief Handle SET_POSITION_TARGET_GLOBAL_INT for global position control
     * 
     * @details Processes position target in global coordinates (lat/lon/alt).
     *          Used for offboard position control from companion computer or GCS.
     *          Can specify position, velocity, and/or acceleration targets.
     * 
     * @param[in] msg MAVLink SET_POSITION_TARGET_GLOBAL_INT message
     * 
     * @warning Must be called at >2Hz to maintain control authority
     * 
     * @note Only effective in Guided mode
     * @note Coordinates in WGS84 geodetic frame
     */
    void handle_message_set_position_target_global_int(const mavlink_message_t &msg);

    /**
     * @brief Handle SET_POSITION_TARGET_LOCAL_NED for local position control
     * 
     * @details Processes position target in local NED coordinates (North, East, Down)
     *          relative to home position. Primary method for companion computer
     *          position control using local frame.
     * 
     * @param[in] msg MAVLink SET_POSITION_TARGET_LOCAL_NED message
     * 
     * @warning Must be called at >2Hz to maintain control. Velocity/acceleration
     *          vectors must pass sanity checks (no NaN, magnitude < 1000 m/s)
     * 
     * @note Only effective in Guided mode
     * @note Home position is NED frame origin
     */
    void handle_message_set_position_target_local_ned(const mavlink_message_t &msg);

    /**
     * @brief Handle LANDING_TARGET message for precision landing
     * 
     * @details Processes landing target position from IR beacon or visual tracking
     *          system. Updates precision landing subsystem with target location
     *          relative to vehicle, enabling accurate landing on moving platforms.
     * 
     * @param[in] packet  Landing target data (bearing, distance, position)
     * @param[in] timestamp_ms Timestamp of target detection in milliseconds
     * 
     * @note Requires precision landing to be enabled (PLND_ENABLED=1)
     * @note Target position can be in body frame or global frame
     */
    void handle_landing_target(const mavlink_landing_target_t &packet, uint32_t timestamp_ms) override;

    // ========================================================================
    // Navigation and Status Telemetry
    // ========================================================================

    /**
     * @brief Send NAV_CONTROLLER_OUTPUT message with navigation state
     * 
     * @details Streams current navigation controller outputs including desired
     *          bearing, distance to waypoint, altitude error, and cross-track error.
     *          Useful for monitoring navigation performance in Auto and Guided modes.
     * 
     * @note Streaming rate controlled by SR0_POSITION parameter
     */
    void send_nav_controller_output() const override;

    /**
     * @brief Report vehicle capabilities to ground station
     * 
     * @details Returns bitmask of MAV_PROTOCOL_CAPABILITY flags indicating which
     *          features and protocols this vehicle supports. Includes guided mode,
     *          mission items, parameter protocol, terrain following, etc.
     * 
     * @return Bitmask of supported capabilities
     * 
     * @note Sent in AUTOPILOT_VERSION message during handshake
     */
    uint64_t capabilities() const override;

    /**
     * @brief Return VTOL state (always multicopter for ArduCopter)
     * 
     * @details Reports vehicle VTOL state for ground station display. ArduCopter
     *          is always in multicopter state (no transition modes).
     * 
     * @return MAV_VTOL_STATE_MC (multicopter) always
     * 
     * @note Used by GCS to determine UI and available commands
     */
    virtual MAV_VTOL_STATE vtol_state() const override { return MAV_VTOL_STATE_MC; };

    /**
     * @brief Return current landed state based on landing detector
     * 
     * @details Reports whether vehicle is on ground, in air, taking off, or landing.
     *          Uses copter landing detector which monitors throttle, vertical
     *          velocity, and rangefinder to determine state.
     * 
     * @return MAV_LANDED_STATE enum value (ON_GROUND, IN_AIR, TAKEOFF, LANDING)
     * 
     * @note Reported in EXTENDED_SYS_STATE message
     */
    virtual MAV_LANDED_STATE landed_state() const override;

    /**
     * @brief Handle MANUAL_CONTROL message for joystick/RC-over-MAVLink input
     * 
     * @details Processes manual control inputs from ground station joystick or
     *          game controller sent over MAVLink. Provides RC-over-MAVLink
     *          functionality when no physical RC receiver connected.
     * 
     * @param[in] packet Manual control axes data (x, y, z, r, buttons)
     * @param[in] tnow   Current time in milliseconds
     * 
     * @warning Manual control must be sent at >2Hz to prevent failsafe timeout
     * 
     * @note Requires RC_OVERRIDE_TIME parameter configured appropriately
     */
    void handle_manual_control_axes(const mavlink_manual_control_t &packet, const uint32_t tnow) override;

#if HAL_LOGGING_ENABLED
    /**
     * @brief Return logging bitmask for radio status messages
     * 
     * @details Specifies which log bitmask enables radio RSSI and link quality
     *          logging for this vehicle type. Copter uses PM (performance monitoring)
     *          log bitmask for radio telemetry.
     * 
     * @return MASK_LOG_PM bitmask for performance monitoring logs
     * 
     * @note Only compiled when logging enabled (HAL_LOGGING_ENABLED)
     */
    uint32_t log_radio_bit() const override { return MASK_LOG_PM; }
#endif

    /**
     * @brief Send available flight mode information to ground station
     * 
     * @details Streams AVAILABLE_MODES message containing flight mode details for
     *          GCS display. Called iteratively with increasing index to send all
     *          available modes. Allows GCS to populate mode selection UI with
     *          valid modes for this vehicle configuration.
     * 
     * @param[in] index Mode index to send (1-based, not mode number)
     * 
     * @return Total number of available flight modes
     * 
     * @note Index starts at 1. Ground station calls repeatedly until all modes sent.
     * @note Only modes available for current vehicle configuration are reported
     */
    uint8_t send_available_mode(uint8_t index) const override;

private:

    // ========================================================================
    // Private Helper Functions
    // ========================================================================

    /**
     * @brief Sanity check velocity or acceleration vector for valid values
     * 
     * @details Validates that vector components are finite numbers (not NaN or infinity)
     *          and have reasonable magnitude (< 1000 m/s or m/s²). Prevents invalid
     *          external control inputs from causing undefined vehicle behavior.
     *          
     *          Checks performed:
     *          - No NaN (Not a Number) components
     *          - No infinite values
     *          - Magnitude of each component < 1000 m/s or m/s²
     * 
     * @param[in] vec Velocity vector (m/s) or acceleration vector (m/s²) to validate
     * 
     * @return true if vector passes all sanity checks
     * @return false if vector contains NaN, infinity, or excessive magnitude
     * 
     * @note Used to validate SET_POSITION_TARGET messages before applying to vehicle
     * @warning Accepting invalid vectors could cause flyaways or crashes
     */
    bool sane_vel_or_acc_vector(const Vector3f &vec) const;

    /**
     * @brief Determine mission execution state for MISSION_CURRENT message
     * 
     * @details Examines mission object to determine current state (inactive, active,
     *          paused, complete) for reporting to ground station via MISSION_CURRENT.
     * 
     * @param[in] mission Mission object to query for state
     * 
     * @return MISSION_STATE enum indicating current mission execution state
     * 
     * @note Called periodically when mission telemetry enabled
     */
    MISSION_STATE mission_state(const class AP_Mission &mission) const override;

    // ========================================================================
    // Core Message Processing
    // ========================================================================

    /**
     * @brief Main message router for incoming MAVLink messages
     * 
     * @details Routes incoming MAVLink messages to appropriate handler based on
     *          message ID. Handles copter-specific messages and passes others to
     *          base class for common processing. This is the main entry point for
     *          all received MAVLink messages on this link.
     *          
     *          Copter-specific messages handled:
     *          - SET_ATTITUDE_TARGET (external attitude control)
     *          - SET_POSITION_TARGET_LOCAL_NED (local position control)
     *          - SET_POSITION_TARGET_GLOBAL_INT (global position control)
     * 
     * @param[in] msg Received MAVLink message to process
     * 
     * @note Called from packetReceived() after basic message validation
     * @note Runs in main vehicle thread context
     */
    void handle_message(const mavlink_message_t &msg) override;

    /**
     * @brief Handle COMMAND_ACK messages from other vehicle components
     * 
     * @details Processes command acknowledgements sent by other subsystems
     *          (companion computers, gimbal controllers) and forwards or handles
     *          appropriately.
     * 
     * @param[in] msg COMMAND_ACK message received
     * 
     * @note Used for multi-system command coordination
     */
    void handle_command_ack(const mavlink_message_t &msg) override;

    /**
     * @brief Handle mission command requests in guided mode
     * 
     * @details Called when mission system wants to execute command in guided mode
     *          (typically from GUIDED_ENABLE mission item). Validates command is
     *          appropriate for copter and current vehicle state.
     * 
     * @param[in,out] cmd Mission command to validate and potentially modify
     * 
     * @return true if command accepted for execution
     * @return false if command rejected or not supported in current state
     * 
     * @note Provides integration between mission system and guided mode
     */
    bool handle_guided_request(AP_Mission::Mission_Command &cmd) override;

    /**
     * @brief Attempt to send queued telemetry message
     * 
     * @details Called by scheduler to send next telemetry message in queue based
     *          on stream rates and priorities. Routes message ID to appropriate
     *          sender function.
     * 
     * @param[in] id Message type identifier (ap_message enum)
     * 
     * @return true if message sent successfully or not applicable
     * @return false if message send failed (will retry)
     * 
     * @note Message streaming rates controlled by SRx_* parameters
     * @note Bandwidth management handled by base class
     */
    bool try_send_message(enum ap_message id) override;

    /**
     * @brief Low-level packet reception callback from MAVLink parser
     * 
     * @details Called by MAVLink library when complete valid packet received on
     *          this link. Performs initial packet validation, updates link statistics,
     *          and routes to handle_message() for processing.
     * 
     * @param[in] status MAVLink parser status with link statistics
     * @param[in] msg    Received and validated MAVLink message
     * 
     * @note Called from UART/network interrupt context via HAL
     * @note Packet signature verification performed before this callback
     */
    void packetReceived(const mavlink_status_t &status,
                        const mavlink_message_t &msg) override;

    // ========================================================================
    // Vehicle State Reporting
    // ========================================================================

    /**
     * @brief Determine base mode flags for HEARTBEAT message
     * 
     * @details Computes MAV_MODE_FLAG bitmask indicating vehicle state including
     *          armed/disarmed, manual/auto mode, guided enabled, stabilization enabled,
     *          hardware-in-the-loop simulation, and safety armed.
     * 
     * @return Bitmask of MAV_MODE_FLAG values representing current vehicle state
     * 
     * @note Sent in HEARTBEAT message at 1Hz
     * @note GCS uses this to enable/disable UI elements
     */
    uint8_t base_mode() const override;

    /**
     * @brief Determine vehicle system status for HEARTBEAT message
     * 
     * @details Returns MAV_STATE enum indicating vehicle health and operational
     *          status (uninit, boot, standby, active, critical, emergency, poweroff).
     * 
     * @return MAV_STATE enum representing current vehicle operational state
     * 
     * @note Sent in HEARTBEAT message at 1Hz
     * @note Critical/emergency states trigger GCS warnings
     */
    MAV_STATE vehicle_system_status() const override;

    // ========================================================================
    // VFR_HUD Message Components (Visual Flight Rules Heads Up Display)
    // ========================================================================

    /**
     * @brief Calculate synthetic airspeed for VFR_HUD message
     * 
     * @details Copters don't have airspeed sensors, so this returns groundspeed
     *          from inertial navigation as airspeed substitute for GCS display.
     * 
     * @return Ground speed in m/s
     * 
     * @note VFR_HUD is legacy message but still used by many ground stations
     */
    float vfr_hud_airspeed() const override;

    /**
     * @brief Calculate throttle percentage for VFR_HUD message
     * 
     * @details Returns current throttle output as percentage (0-100) for GCS
     *          throttle gauge display.
     * 
     * @return Throttle percentage (0-100)
     * 
     * @note Based on motor thrust output, not RC input
     */
    int16_t vfr_hud_throttle() const override;

    /**
     * @brief Calculate altitude for VFR_HUD message
     * 
     * @details Returns barometric altitude above mean sea level (MSL) for
     *          GCS altitude display.
     * 
     * @return Altitude in meters above MSL
     * 
     * @note Uses barometer-based altitude, not GPS altitude
     */
    float vfr_hud_alt() const override;

    // ========================================================================
    // Tuning and Diagnostic Telemetry
    // ========================================================================

    /**
     * @brief Send PID tuning data for real-time tuning and analysis
     * 
     * @details Streams PID_TUNING messages containing PID controller inputs,
     *          outputs, and internal states for the currently selected tuning
     *          axis (roll, pitch, yaw, altitude, etc.). Used by ground station
     *          tuning tools for live PID tuning and analysis.
     * 
     * @note Tuning axis selection controlled by TUNE_SELECT parameter
     * @note Streaming rate controlled by SR0_EXTRA3 parameter
     */
    void send_pid_tuning() override;

#if AP_WINCH_ENABLED
    /**
     * @brief Send winch status telemetry for cargo delivery applications
     * 
     * @details Streams WINCH_STATUS message containing winch line length, speed,
     *          tension, and operational state for cargo delivery or rescue operations.
     * 
     * @note Only compiled when winch support enabled (AP_WINCH_ENABLED)
     * @note Requires winch device configured and connected
     */
    void send_winch_status() const override;
#endif

    /**
     * @brief Send wind speed and direction estimate
     * 
     * @details Streams WIND message containing estimated wind velocity vector
     *          derived from vehicle velocity error and IMU measurements. Useful
     *          for flight planning and performance analysis.
     * 
     * @note Wind estimation only valid when vehicle is moving
     * @note Streaming rate controlled by SR0_EXTRA3 parameter
     */
    void send_wind() const;

    // ========================================================================
    // High Latency 2 Protocol Support (Bandwidth-Constrained Links)
    // ========================================================================

#if HAL_HIGH_LATENCY2_ENABLED
    /**
     * @brief Get target altitude for HIGH_LATENCY2 message
     * 
     * @details Returns current navigation target altitude for compressed telemetry
     *          on bandwidth-constrained links (Iridium satellite, etc.)
     * 
     * @return Target altitude in meters above MSL
     * 
     * @note Part of HIGH_LATENCY2 compressed telemetry format
     */
    int16_t high_latency_target_altitude() const override;

    /**
     * @brief Get target heading for HIGH_LATENCY2 message
     * 
     * @details Returns current navigation target heading for compressed telemetry.
     * 
     * @return Target heading in degrees (0-359)
     */
    uint8_t high_latency_tgt_heading() const override;

    /**
     * @brief Get distance to target for HIGH_LATENCY2 message
     * 
     * @details Returns distance to current navigation target (waypoint, home, etc.)
     *          for compressed telemetry format.
     * 
     * @return Distance to target in meters
     */
    uint16_t high_latency_tgt_dist() const override;

    /**
     * @brief Get target airspeed for HIGH_LATENCY2 message
     * 
     * @details Returns target groundspeed (copters don't have airspeed) for
     *          compressed telemetry. Returns 0 if not applicable.
     * 
     * @return Target speed in m/s
     */
    uint8_t high_latency_tgt_airspeed() const override;

    /**
     * @brief Get wind speed for HIGH_LATENCY2 message
     * 
     * @details Returns estimated wind speed magnitude for compressed telemetry.
     * 
     * @return Wind speed in m/s
     */
    uint8_t high_latency_wind_speed() const override;

    /**
     * @brief Get wind direction for HIGH_LATENCY2 message
     * 
     * @details Returns estimated wind direction for compressed telemetry.
     * 
     * @return Wind direction in degrees (0-359, direction wind is coming FROM)
     */
    uint8_t high_latency_wind_direction() const override;
#endif // HAL_HIGH_LATENCY2_ENABLED

    // ========================================================================
    // Copter-Specific MAVLink Command Handlers
    // ========================================================================

    /**
     * @brief Handle MAV_CMD_CONDITION_YAW to set vehicle heading
     * 
     * @details Commands vehicle to yaw to specified heading. Can specify absolute
     *          or relative angle, rotation direction, and rotation rate. Typically
     *          used in Auto mode missions or guided mode.
     *          
     *          Parameters:
     *          - param1: Target angle in degrees (0-360 for absolute, +/- for relative)
     *          - param2: Angular speed in deg/s (0 for default)
     *          - param3: Direction (-1=CCW, 1=CW, 0=shortest path)
     *          - param4: 0=absolute angle, 1=relative to current heading
     * 
     * @param[in] packet COMMAND_INT with yaw parameters
     * 
     * @return MAV_RESULT_ACCEPTED if yaw command accepted
     * @return MAV_RESULT_DENIED if not in appropriate mode
     * 
     * @note Only functional in Auto and Guided modes
     */
    MAV_RESULT handle_MAV_CMD_CONDITION_YAW(const mavlink_command_int_t &packet);

    /**
     * @brief Handle MAV_CMD_DO_CHANGE_SPEED to modify auto mode speeds
     * 
     * @details Changes speed parameters during mission or guided mode flight.
     *          Can modify horizontal speed, vertical speed (climb/descent rate),
     *          or throttle percentage.
     *          
     *          Parameters:
     *          - param1: Speed type (0=airspeed, 1=groundspeed, 2=climb speed, 3=descent speed)
     *          - param2: Target speed in m/s (or throttle % if param1=1)
     *          - param3: Throttle % (0-100, -1 for no change)
     * 
     * @param[in] packet COMMAND_INT with speed parameters
     * 
     * @return MAV_RESULT_ACCEPTED if speed changed successfully
     * @return MAV_RESULT_DENIED if not in appropriate mode
     * 
     * @note Speed changes persist until next DO_CHANGE_SPEED command or mode change
     */
    MAV_RESULT handle_MAV_CMD_DO_CHANGE_SPEED(const mavlink_command_int_t &packet);

    /**
     * @brief Handle MAV_CMD_DO_MOTOR_TEST for individual motor testing
     * 
     * @details Commands individual motor to spin at specified throttle for testing
     *          motor/ESC functionality and verifying motor order. Critical for
     *          pre-flight verification.
     *          
     *          Parameters:
     *          - param1: Motor sequence number (1-N)
     *          - param2: Throttle type (0=%, 1=PWM, 2=pilot throttle channel %)
     *          - param3: Throttle value
     *          - param4: Duration in seconds
     *          - param5: Motor count (0 for all motors, otherwise test specific number)
     *          - param6: Test order (0=defined order, 1=board order, 2=descending)
     * 
     * @param[in] packet COMMAND_INT with motor test parameters
     * 
     * @return MAV_RESULT_ACCEPTED if motor test started
     * @return MAV_RESULT_DENIED if vehicle armed or unsafe conditions
     * 
     * @warning Vehicle must be DISARMED and secured before motor testing
     * @warning Propellers should be removed for safety during testing
     * 
     * @note Only allowed when disarmed
     */
    MAV_RESULT handle_MAV_CMD_DO_MOTOR_TEST(const mavlink_command_int_t &packet);

    /**
     * @brief Handle MAV_CMD_DO_PARACHUTE for parachute deployment
     * 
     * @details Commands parachute deployment, disablement, or release. Used for
     *          emergency recovery systems or cargo drop applications.
     *          
     *          Parameters:
     *          - param1: Action (0=disable, 1=enable, 2=release/deploy)
     * 
     * @param[in] packet COMMAND_INT with parachute action
     * 
     * @return MAV_RESULT_ACCEPTED if parachute action executed
     * @return MAV_RESULT_DENIED if parachute not configured or action invalid
     * 
     * @warning Parachute deployment may be irreversible depending on mechanism
     * 
     * @note Requires parachute configured (CHUTE_ENABLED=1)
     */
    MAV_RESULT handle_MAV_CMD_DO_PARACHUTE(const mavlink_command_int_t &packet);

    // ========================================================================
    // 3DR Solo Smart Shot Button Commands (Optional)
    // ========================================================================

#if AC_MAVLINK_SOLO_BUTTON_COMMAND_HANDLING_ENABLED
    /**
     * @brief Handle Solo Fly button click for automated takeoff
     * 
     * @details Processes 3DR Solo controller Fly button click to initiate
     *          automated takeoff to default altitude. Part of Solo smart shot
     *          system for simplified operation.
     * 
     * @param[in] packet COMMAND_INT (parameters typically unused)
     * 
     * @return MAV_RESULT_ACCEPTED if takeoff initiated
     * @return MAV_RESULT_DENIED if vehicle not ready for takeoff
     * 
     * @note Only compiled if AC_MAVLINK_SOLO_BUTTON_COMMAND_HANDLING_ENABLED
     * @note Specific to 3DR Solo controller integration
     */
    MAV_RESULT handle_MAV_CMD_SOLO_BTN_FLY_CLICK(const mavlink_command_int_t &packet);

    /**
     * @brief Handle Solo Fly button hold for automated landing
     * 
     * @details Processes 3DR Solo controller Fly button hold to initiate
     *          automated landing at current position.
     * 
     * @param[in] packet COMMAND_INT (parameters typically unused)
     * 
     * @return MAV_RESULT_ACCEPTED if landing initiated
     * @return MAV_RESULT_DENIED if vehicle not in air or mode doesn't support landing
     * 
     * @note Only compiled if AC_MAVLINK_SOLO_BUTTON_COMMAND_HANDLING_ENABLED
     */
    MAV_RESULT handle_MAV_CMD_SOLO_BTN_FLY_HOLD(const mavlink_command_int_t &packet);

    /**
     * @brief Handle Solo Pause button click for mission pause
     * 
     * @details Processes 3DR Solo controller Pause button to pause mission
     *          execution or smart shot and hold position.
     * 
     * @param[in] packet COMMAND_INT (parameters typically unused)
     * 
     * @return MAV_RESULT_ACCEPTED if vehicle paused successfully
     * @return MAV_RESULT_DENIED if not in pausable mode
     * 
     * @note Only compiled if AC_MAVLINK_SOLO_BUTTON_COMMAND_HANDLING_ENABLED
     */
    MAV_RESULT handle_MAV_CMD_SOLO_BTN_PAUSE_CLICK(const mavlink_command_int_t &packet);
#endif

    // ========================================================================
    // Command Format Conversion
    // ========================================================================

#if AP_MAVLINK_COMMAND_LONG_ENABLED
    /**
     * @brief Determine appropriate MAV_FRAME for COMMAND_LONG conversion
     * 
     * @details When COMMAND_LONG (legacy integer command format) is received,
     *          this determines the coordinate frame to use when converting to
     *          internal command representation. Copter uses MAV_FRAME_GLOBAL_RELATIVE_ALT
     *          for most position commands.
     * 
     * @param[out] frame       Reference to store determined coordinate frame
     * @param[in]  packet_command MAV_CMD being processed
     * 
     * @return true if frame determined successfully
     * @return false if command doesn't require frame or frame invalid
     * 
     * @note COMMAND_INT is preferred over COMMAND_LONG in MAVLink 2
     * @note Only compiled if COMMAND_LONG support enabled
     */
    bool mav_frame_for_command_long(MAV_FRAME &frame, MAV_CMD packet_command) const override;
#endif

    // ========================================================================
    // Mission and Navigation Commands
    // ========================================================================

    /**
     * @brief Handle MAV_CMD_MISSION_START to begin mission execution
     * 
     * @details Commands vehicle to switch to Auto mode and start executing
     *          loaded mission from specified waypoint. Alternative to mode
     *          switch + mission navigation commands.
     *          
     *          Parameters:
     *          - param1: First mission item to execute (0 for current)
     *          - param2: Last mission item to execute (0 for last)
     * 
     * @param[in] packet COMMAND_INT with mission start parameters
     * 
     * @return MAV_RESULT_ACCEPTED if mission started
     * @return MAV_RESULT_DENIED if no mission loaded or vehicle not ready
     * 
     * @note Switches vehicle to Auto mode automatically
     */
    MAV_RESULT handle_MAV_CMD_MISSION_START(const mavlink_command_int_t &packet);

    /**
     * @brief Handle MAV_CMD_NAV_TAKEOFF for commanded takeoff
     * 
     * @details Commands vehicle to take off to specified altitude. Can be used
     *          to initiate takeoff in Guided mode or as part of mission.
     *          
     *          Parameters:
     *          - param1: Minimum pitch (ignored for copter)
     *          - param2-param4: Empty
     *          - param5: Latitude (0 for current location)
     *          - param6: Longitude (0 for current location)
     *          - param7: Altitude in meters (absolute or relative based on frame)
     * 
     * @param[in] packet COMMAND_INT with takeoff altitude and position
     * 
     * @return MAV_RESULT_ACCEPTED if takeoff initiated
     * @return MAV_RESULT_DENIED if vehicle not on ground or pre-arm checks failed
     * 
     * @note Vehicle must be armed or will auto-arm if parameters permit
     * @note Altitude is relative to home unless MAV_FRAME_GLOBAL specified
     */
    MAV_RESULT handle_MAV_CMD_NAV_TAKEOFF(const mavlink_command_int_t &packet);

    // ========================================================================
    // Cargo Delivery / Winch Commands
    // ========================================================================

#if AP_WINCH_ENABLED
    /**
     * @brief Handle MAV_CMD_DO_WINCH for cargo winch control
     * 
     * @details Commands winch operations for cargo delivery including line
     *          extension, retraction, locking, and load management.
     *          
     *          Parameters:
     *          - param1: Winch ID (0 for default winch)
     *          - param2: Action (0=relax, 1=relative length, 2=rate, 3=lock, 4=deliver, 5=hold, 6=retract, 7=load_line, 8=abandon_line)
     *          - param3: Length in meters (for relative length action)
     *          - param4: Rate in m/s (for rate action)
     * 
     * @param[in] packet COMMAND_INT with winch control parameters
     * 
     * @return MAV_RESULT_ACCEPTED if winch command executed
     * @return MAV_RESULT_DENIED if winch not configured or parameters invalid
     * 
     * @note Only compiled when winch support enabled (AP_WINCH_ENABLED)
     * @note Requires winch hardware configured and connected
     */
    MAV_RESULT handle_MAV_CMD_DO_WINCH(const mavlink_command_int_t &packet);
#endif

};
