/**
 * @file AP_Frsky_MAVliteMsgHandler.h
 * @brief Transport-agnostic MAVlite command processor for FrSky telemetry
 * 
 * This file implements a MAVlite message handler that processes commands received
 * from FrSky transmitters. The handler is transport-agnostic and processes
 * MAVLink-like commands embedded in MAVlite messages including:
 * - Parameter access (read/write)
 * - Flight mode changes
 * - Calibration commands (barometer)
 * - Geofence control (enable/disable)
 * - System reboot commands
 * 
 * The handler enforces safety checks such as armed state validation and
 * parameter validation, and sends acknowledgment responses via callback.
 * 
 * @note Only compiled when HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL is enabled
 * @see AP_Frsky_MAVlite_Message.h for message structure details
 * @see AP_Param for parameter system integration
 */

#pragma once

#include "AP_Frsky_MAVlite.h"
#include "AP_Frsky_Telem.h"
#include "AP_Frsky_MAVlite_Message.h"

#if HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL

/**
 * @class AP_Frsky_MAVliteMsgHandler
 * @brief Processes received MAVlite messages containing MAVLink-like commands
 * 
 * @details This class handles bidirectional communication with FrSky transmitters
 *          by processing incoming MAVlite messages and generating appropriate responses.
 *          
 *          Key responsibilities:
 *          - Decode and dispatch MAVlite messages to appropriate handlers
 *          - Process parameter read/write requests via AP_Param system
 *          - Execute flight mode changes via vehicle interface
 *          - Handle calibration commands (barometer)
 *          - Control geofence enable/disable
 *          - Process system reboot commands
 *          - Enforce safety checks (armed state validation)
 *          - Send acknowledgment responses via callback
 *          
 *          Message flow:
 *          1. Receive MAVlite message via process_message()
 *          2. Dispatch to handler based on msgid
 *          3. Execute command with safety validation
 *          4. Send response via _send_fn callback
 *          
 * @note Only compiled when HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL is enabled
 * @warning Not designed for concurrent access - should be called from single thread
 * @note Supported commands: PARAM_REQUEST_READ, PARAM_SET, COMMAND_LONG
 *       (mode change, fence enable/disable, baro calibration, reboot)
 * 
 * @see AP_Frsky_MAVlite_Message.h for message structure
 * @see AP_Param for parameter system
 * @see AP::vehicle() for mode changes
 */
class AP_Frsky_MAVliteMsgHandler {
public:

    /**
     * @brief Callback function type for transmitting MAVlite response messages
     * 
     * @param[in] message The MAVlite message to send (const AP_Frsky_MAVlite_Message&)
     * @return true if message was successfully queued for transmission, false otherwise
     * 
     * @note This callback is invoked to send responses back to the FrSky transmitter
     * @note Implementation must handle message serialization and transport
     */
    FUNCTOR_TYPEDEF(send_mavlite_fn_t, bool, const AP_Frsky_MAVlite_Message &);
    
    /**
     * @brief Initialize MAVlite message handler with transmission callback
     * 
     * @param[in] send_fn Function to call for transmitting response messages
     * 
     * @note The send_fn callback will be invoked whenever a response needs to be sent
     * @note Callback must remain valid for the lifetime of this handler
     */
    AP_Frsky_MAVliteMsgHandler(send_mavlite_fn_t send_fn) :
        _send_fn(send_fn) {}

    /**
     * @brief Process received MAVlite message and execute command
     * 
     * @param[in] rxmsg The received MAVlite message to handle
     * 
     * @details This is the main entry point for message processing. The method
     *          dispatches to appropriate handlers based on the message ID (msgid):
     *          - MAVLINK_MSG_ID_PARAM_REQUEST_READ → handle_param_request_read()
     *          - MAVLINK_MSG_ID_PARAM_SET → handle_param_set()
     *          - MAVLINK_MSG_ID_COMMAND_LONG → handle_command_long()
     *          
     *          Each handler enforces appropriate safety checks before executing
     *          the requested operation and sends responses via the callback.
     * 
     * @note Dispatches to appropriate handler based on msgid field
     * @warning Enforces safety checks - rejects commands if vehicle is armed
     *          (checked via hal.util->get_soft_armed())
     * 
     * @see handle_param_request_read()
     * @see handle_param_set()
     * @see handle_command_long()
     */
    void process_message(const AP_Frsky_MAVlite_Message &rxmsg);

private:
    /**
     * @brief Send MAVlite message via registered callback
     * 
     * @param[in,out] txmsg The message to transmit
     * @return true if message was successfully queued for transmission
     * 
     * @note Calls _send_fn callback to perform actual transmission
     */
    bool send_message(AP_Frsky_MAVlite_Message &txmsg);

    /**
     * @brief Read parameter by name or index and send response
     * 
     * @param[in] rxmsg The received PARAM_REQUEST_READ message
     * 
     * @details Looks up parameter using AP_Param::find() by either:
     *          - Parameter name (if param_id is valid string)
     *          - Parameter index (if param_index is valid)
     *          
     *          Sends PARAM_VALUE response with parameter name, value, type, count,
     *          and index via send_message().
     * 
     * @note Uses AP_Param::find() for parameter lookup
     * @see AP_Param for parameter system details
     */
    void handle_param_request_read(const AP_Frsky_MAVlite_Message &rxmsg);
    
    /**
     * @brief Write parameter value and send acknowledgment
     * 
     * @param[in] rxmsg The received PARAM_SET message containing parameter name and value
     * 
     * @details Validates and writes parameter value via AP_Param system.
     *          Sends PARAM_VALUE response confirming the new value.
     *          
     *          Safety checks:
     *          - Validates value is not NaN or Inf
     *          - Rejects parameter changes if vehicle is armed
     *          - Verifies parameter exists before attempting to set
     * 
     * @warning Validates for NaN/Inf values before writing
     * @warning Rejects parameter changes if vehicle is armed (safety check)
     * @note Uses AP_Param for parameter access and modification
     */
    void handle_param_set(const AP_Frsky_MAVlite_Message &rxmsg);

    /**
     * @brief Process MAV_CMD_COMMAND_LONG message
     * 
     * @param[in] rxmsg The received COMMAND_LONG message
     * 
     * @details Extracts the mavlink_command_long_t payload and dispatches to
     *          handle_command() for routing to specific command handlers.
     *          Sends COMMAND_ACK response with result code.
     * 
     * @note Dispatches to handle_command() which routes to specific command handlers
     * @see handle_command()
     */
    void handle_command_long(const AP_Frsky_MAVlite_Message &rxmsg);

    /**
     * @brief Route command to appropriate handler based on command ID
     * 
     * @param[in] mav_command_long The command structure containing command ID and parameters
     * @return MAV_RESULT status code (ACCEPTED, DENIED, UNSUPPORTED, FAILED, etc.)
     * 
     * @details Dispatches commands to specialized handlers:
     *          - MAV_CMD_PREFLIGHT_CALIBRATION → handle_command_preflight_calibration_baro()
     *          - MAV_CMD_DO_SET_MODE → handle_command_do_set_mode()
     *          - MAV_CMD_DO_FENCE_ENABLE → handle_command_do_fence_enable()
     *          - MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN → handle_command_preflight_reboot()
     * 
     * @return MAV_RESULT_UNSUPPORTED if command ID is not recognized
     */
    MAV_RESULT handle_command(const mavlink_command_long_t &mav_command_long);
    
    /**
     * @brief Execute barometer calibration command
     * 
     * @param[in] mav_command_long Command parameters (unused for baro calibration)
     * @return MAV_RESULT_ACCEPTED if calibration started successfully
     * @return MAV_RESULT_FAILED if calibration could not be initiated
     * 
     * @details Initiates barometer ground pressure calibration to establish
     *          altitude reference. Command is part of MAV_CMD_PREFLIGHT_CALIBRATION.
     * 
     * @note Calibration typically requires vehicle to be stationary on ground
     */
    MAV_RESULT handle_command_preflight_calibration_baro(const mavlink_command_long_t &mav_command_long);
    
    /**
     * @brief Change flight mode via vehicle interface
     * 
     * @param[in] mav_command_long Command parameters with mode in param1
     * @return MAV_RESULT_ACCEPTED if mode change succeeded
     * @return MAV_RESULT_DENIED if mode change was rejected
     * @return MAV_RESULT_FAILED if mode is invalid or change failed
     * 
     * @details Requests flight mode change via AP::vehicle()->set_mode() with
     *          mode reason set to ModeReason::FRSKY_COMMAND for logging.
     *          
     *          Mode parameter (param1) contains the desired mode number which
     *          is vehicle-specific (e.g., Copter modes differ from Plane modes).
     * 
     * @note Uses AP::vehicle()->set_mode(mode, ModeReason::FRSKY_COMMAND)
     * @note Mode numbers are vehicle-specific
     * @see Mode definitions in vehicle-specific code
     */
    MAV_RESULT handle_command_do_set_mode(const mavlink_command_long_t &mav_command_long);
    
    /**
     * @brief Enable or disable geofence system
     * 
     * @param[in] mav_command_long Command parameters with enable flag in param1
     * @return MAV_RESULT_ACCEPTED if fence state changed successfully
     * @return MAV_RESULT_FAILED if fence control failed
     * 
     * @details Controls the geofence system enable/disable state.
     *          param1 = 0: Disable fence
     *          param1 = 1: Enable fence
     * 
     * @see AC_Fence for geofencing system details
     */
    MAV_RESULT handle_command_do_fence_enable(const mavlink_command_long_t &mav_command_long);
    
    /**
     * @brief Reboot autopilot system
     * 
     * @param[in] mav_command_long Command parameters (unused)
     * @return MAV_RESULT_ACCEPTED if reboot will be initiated
     * @return MAV_RESULT_DENIED if vehicle is armed (safety check)
     * 
     * @details Triggers system reboot via hal.scheduler->reboot().
     *          Reboot is only allowed when vehicle is disarmed for safety.
     *          
     *          Safety enforcement:
     *          - Command is rejected if hal.util->get_soft_armed() returns true
     *          - ACK is sent before reboot is triggered
     * 
     * @warning Requires vehicle to be disarmed - command rejected if armed
     * @warning System will reboot after sending acknowledgment
     * @note Uses hal.scheduler->reboot() for system restart
     */
    MAV_RESULT handle_command_preflight_reboot(const mavlink_command_long_t &mav_command_long);

    /**
     * @brief Send command acknowledgment response to transmitter
     * 
     * @param[in] mav_result The result code indicating success or failure
     *                       (MAV_RESULT_ACCEPTED, MAV_RESULT_DENIED, MAV_RESULT_FAILED, etc.)
     * @param[in] cmdid The command ID being acknowledged
     * 
     * @details Constructs and sends a COMMAND_ACK message containing the result
     *          of the command execution. This provides feedback to the FrSky
     *          transmitter operator about whether the command succeeded.
     * 
     * @note Sends COMMAND_ACK MAVLink message via send_message()
     */
    void send_command_ack(const MAV_RESULT mav_result, const uint16_t cmdid);

    /**
     * @brief Callback functor for transmitting response messages
     * 
     * @details This is the registered send_mavlite_fn_t callback that is invoked
     *          whenever a response message needs to be transmitted back to the
     *          FrSky transmitter. The callback handles message serialization and
     *          transport-layer transmission.
     * 
     * @note Set during construction via constructor parameter
     * @note Must remain valid for the lifetime of this handler instance
     */
    send_mavlite_fn_t _send_fn;
};

#endif
