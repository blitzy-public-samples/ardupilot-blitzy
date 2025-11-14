/**
 * @file GCS_MAVLink_Tracker.h
 * @brief MAVLink backend class declaration for antenna tracker
 * 
 * @details Declares GCS_MAVLINK_Tracker class handling tracker-specific MAVLink messages.
 *          This class extends the base GCS_MAVLINK implementation to provide antenna
 *          tracker vehicle-specific message handling, telemetry streaming, and command
 *          processing for tracking operations.
 * 
 * @note One instance of this class exists per MAVLink telemetry port, typically 2-3
 *       instances for serial, USB, and radio telemetry links.
 */

#pragma once

#include <GCS_MAVLink/GCS.h>

/**
 * @class GCS_MAVLINK_Tracker
 * @brief MAVLink protocol handler for single telemetry link
 * 
 * @details Extends GCS_MAVLINK base class to implement antenna tracker-specific functionality:
 *          - Tracker-specific message handlers for target position updates
 *          - Telemetry stream functions for tracking status and position
 *          - Mode and status reporting tailored to tracker capabilities
 *          - Command processing for tracker control and configuration
 * 
 * The class overrides base methods to provide tracker-appropriate responses to MAVLink
 * commands and queries, handles incoming target position messages, and streams tracker
 * telemetry data to ground control stations.
 * 
 * @note One instance per MAVLink telemetry port (typically 2-3 instances total)
 * @warning This class handles real-time tracking control - message processing must
 *          complete within scheduler timing constraints
 */
class GCS_MAVLINK_Tracker : public GCS_MAVLINK
{

public:

    using GCS_MAVLINK::GCS_MAVLINK;

protected:

    /**
     * @brief Handle MAV_CMD_COMPONENT_ARM_DISARM command for tracker arming
     * 
     * @details Processes arm/disarm commands for the antenna tracker. Validates arming
     *          conditions and transitions tracker to armed or disarmed state.
     * 
     * @param[in] packet Command packet structure containing arm/disarm parameters
     * 
     * @return MAV_RESULT indicating command acceptance:
     *         - MAV_RESULT_ACCEPTED: Successfully armed or disarmed
     *         - MAV_RESULT_FAILED: Arming checks failed or invalid state transition
     *         - MAV_RESULT_TEMPORARILY_REJECTED: Pre-arm checks not passing
     * 
     * @note Arming is required before tracker servo outputs are enabled
     */
    MAV_RESULT handle_command_component_arm_disarm(const mavlink_command_int_t &packet) override;
    
    /**
     * @brief Handle barometer calibration command
     * 
     * @details Processes MAV_CMD_PREFLIGHT_CALIBRATION for barometer sensor calibration.
     *          Initiates barometer calibration sequence if requested.
     * 
     * @param[in] msg MAVLink message containing calibration command parameters
     * 
     * @return MAV_RESULT indicating command status:
     *         - MAV_RESULT_ACCEPTED: Calibration started successfully
     *         - MAV_RESULT_FAILED: Calibration could not be initiated
     * 
     * @note Barometer calibration sets current pressure as reference altitude
     */
    MAV_RESULT _handle_command_preflight_calibration_baro(const mavlink_message_t &msg) override;
    
    /**
     * @brief Route MAVLink command packets to appropriate handlers
     * 
     * @details Main command dispatcher for tracker-specific MAVLink commands. Routes
     *          incoming command packets to specialized handler methods based on command ID.
     * 
     * @param[in] packet Command packet structure with command ID and parameters
     * @param[in] msg Original MAVLink message for context
     * 
     * @return MAV_RESULT from the specific command handler
     * 
     * @note This is called at message receive rate, not scheduler rate
     */
    MAV_RESULT handle_command_int_packet(const mavlink_command_int_t &packet, const mavlink_message_t &msg) override;

    /**
     * @brief Get relative altitude for GLOBAL_POSITION_INT message
     * 
     * @details Returns relative altitude in millimeters above home position. For antenna
     *          tracker this is always zero since tracker is assumed to remain at home
     *          location on the ground.
     * 
     * @return 0 - tracker has no relative altitude (stationary ground device)
     * 
     * @note Comment "what if we have been picked up and carried somewhere?" acknowledges
     *       limitation that tracker cannot detect if physically moved
     */
    int32_t global_position_int_relative_alt() const override {
        return 0; // what if we have been picked up and carried somewhere?
    }

    /**
     * @brief Send tracking target information via NAV_CONTROLLER_OUTPUT message
     * 
     * @details Transmits current tracking target position and pointing angles to ground
     *          control station. Includes target bearing, distance, and servo positions.
     *          
     * @note Called at telemetry stream rate (typically 1-10 Hz)
     * @warning Override must not modify vehicle state (const method)
     */
    void send_nav_controller_output() const override;
    
    /**
     * @brief Send PID controller tuning telemetry
     * 
     * @details Transmits PID controller gains and measured values for real-time tuning
     *          and monitoring. Includes pitch and yaw controller telemetry for antenna
     *          pointing servos.
     * 
     * @note Called when PID_TUNING message is requested in telemetry stream
     * @note Useful for tuning servo response and debugging tracking oscillations
     */
    void send_pid_tuning() override;

    /**
     * @brief Send available flight mode information for mode enumeration
     * 
     * @details Transmits AVAILABLE_MODE message for a specific mode index, allowing ground
     *          control stations to enumerate all supported tracker modes (MANUAL, AUTO, etc.).
     *          Called iteratively with increasing index to send complete mode list.
     * 
     * @param[in] index Mode index to report (1-based, not mode number)
     * 
     * @return Total number of available modes (allows GCS to know when enumeration complete)
     * 
     * @note Index starts at 1, not 0
     * @note This is mode display order, not the internal mode number
     */
    // Send the mode with the given index (not mode number!) return the total number of modes
    // Index starts at 1
    uint8_t send_available_mode(uint8_t index) const override;

    /**
     * @brief Attempt to send a queued message to ground control station
     * 
     * @details Tries to send a message from the telemetry queue. Returns success/failure
     *          to allow queue management and bandwidth throttling.
     * 
     * @param[in] id Message type identifier from ap_message enum
     * 
     * @return true if message sent successfully, false if unable to send (buffer full, etc.)
     * 
     * @note Called by telemetry scheduler to process message queue
     * @note Bandwidth-limited by telemetry stream rates
     */
    bool try_send_message(enum ap_message id) override;

private:

    /**
     * @brief Main message router for tracker-specific MAVLink messages
     * 
     * @details Called when complete MAVLink packet is received. Routes message to
     *          appropriate handler based on message ID. Performs target system ID
     *          validation before processing.
     * 
     * @param[in] status MAVLink channel status (sequence numbers, packet drops, etc.)
     * @param[in] msg Received MAVLink message structure
     * 
     * @note Called at message receive rate (can be 50+ Hz on high-bandwidth links)
     * @warning Must complete quickly to avoid blocking message receive processing
     */
    void packetReceived(const mavlink_status_t &status, const mavlink_message_t &msg) override;
    
    /**
     * @brief Validate message target system and component IDs
     * 
     * @details Checks if received message is addressed to this tracker. Filters messages
     *          intended for other systems on the MAVLink network.
     * 
     * @param[in] msg MAVLink message to validate
     * 
     * @note Only processes messages targeted to this tracker or broadcast messages
     */
    void mavlink_check_target(const mavlink_message_t &msg);
    
    /**
     * @brief Route incoming messages to specialized handlers
     * 
     * @details Secondary message dispatcher that routes messages to type-specific handler
     *          methods after target validation. Handles tracker-specific message types.
     * 
     * @param[in] msg MAVLink message to process
     * 
     * @note Called after target validation by packetReceived
     */
    void handle_message(const mavlink_message_t &msg) override;
    
    /**
     * @brief Handle mission upload initialization message
     * 
     * @details Processes MISSION_WRITE_PARTIAL_LIST message to prepare for receiving
     *          mission waypoint items. Validates mission count and allocates storage.
     * 
     * @param[in] msg MAVLink message containing mission count and start/end indices
     * 
     * @note Tracker missions are typically single-item (home position)
     */
    void handle_message_mission_write_partial_list(const mavlink_message_t &msg);
    
    /**
     * @brief Handle individual mission item upload
     * 
     * @details Processes MISSION_ITEM message containing waypoint data. Stores mission
     *          item and requests next item or completes mission upload.
     * 
     * @param[in] msg MAVLink message with mission item parameters
     * 
     * @note Tracker uses mission items primarily for home position definition
     */
    void handle_message_mission_item(const mavlink_message_t &msg);
    
    /**
     * @brief Handle manual control input from joystick/gamepad
     * 
     * @details Processes MANUAL_CONTROL message for direct servo control in MANUAL mode.
     *          Translates joystick axes to pitch/yaw servo positions.
     * 
     * @param[in] msg MAVLink message with joystick axes values (-1000 to 1000)
     * 
     * @note Only active when tracker is in MANUAL mode
     */
    void handle_message_manual_control(const mavlink_message_t &msg);
    
    /**
     * @brief Handle target vehicle position update
     * 
     * @details Processes GLOBAL_POSITION_INT message from target vehicle. Updates tracking
     *          target position for autonomous pointing in AUTO mode. This is the primary
     *          input for tracking moving vehicles.
     * 
     * @param[in] msg MAVLink message with target GPS position, altitude, and velocity
     * 
     * @note Target position is used to calculate pointing angles in AUTO mode
     * @warning Message rate affects tracking smoothness - typically requires 1-10 Hz
     */
    void handle_message_global_position_int(const mavlink_message_t &msg);
    
    /**
     * @brief Handle barometric pressure sensor data
     * 
     * @details Processes SCALED_PRESSURE message for external barometer integration.
     *          Updates pressure/altitude readings if external barometer is configured.
     * 
     * @param[in] msg MAVLink message with pressure sensor readings
     */
    void handle_message_scaled_pressure(const mavlink_message_t &msg);
    
    /**
     * @brief Handle attitude target commands for GUIDED mode
     * 
     * @details Processes SET_ATTITUDE_TARGET message to command specific pointing angles.
     *          Allows external systems to directly control tracker orientation.
     * 
     * @param[in] msg MAVLink message with target attitude quaternion or Euler angles
     * 
     * @note Used for programmatic control of tracker pointing
     */
    void handle_set_attitude_target(const mavlink_message_t &msg);

    /**
     * @brief Send tracker GPS position via GLOBAL_POSITION_INT message
     * 
     * @details Transmits current tracker GPS position, altitude, and heading to ground
     *          control station. Reports tracker's own position (not target position).
     * 
     * @note Called at position telemetry stream rate (typically 1-5 Hz)
     * @note Tracker position is typically static (home location)
     */
    void send_global_position_int() override;

    /**
     * @brief Construct MAV_MODE flags from current tracker state
     * 
     * @details Builds MAV_MODE_FLAG bitmask for HEARTBEAT message based on:
     *          - Armed state (MAV_MODE_FLAG_SAFETY_ARMED)
     *          - Current flight mode
     *          - Manual/auto control mode
     * 
     * @return uint8_t MAV_MODE flags bitmask for HEARTBEAT
     * 
     * @note Called at HEARTBEAT rate (typically 1 Hz)
     */
    uint8_t base_mode() const override;
    
    /**
     * @brief Get system state for HEARTBEAT message
     * 
     * @details Returns current MAV_STATE indicating tracker operational status:
     *          - MAV_STATE_BOOT: Initializing
     *          - MAV_STATE_STANDBY: Ready but disarmed
     *          - MAV_STATE_ACTIVE: Armed and operating
     * 
     * @return MAV_STATE enum value for HEARTBEAT message
     * 
     * @note Called at HEARTBEAT rate (typically 1 Hz)
     */
    MAV_STATE vehicle_system_status() const override;

    /**
     * @brief Mission waypoint upload in progress flag
     * 
     * @details Set to true when MISSION_WRITE_PARTIAL_LIST received, cleared when
     *          mission upload completes or times out. Prevents conflicting mission
     *          operations during upload.
     */
    bool waypoint_receiving;
};
