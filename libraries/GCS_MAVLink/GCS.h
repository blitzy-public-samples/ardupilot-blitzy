/**
 * @file GCS.h
 * @brief Primary interface for Ground Control System MAVLink protocols
 * 
 * @details This file defines the core MAVLink communication infrastructure for ArduPilot,
 *          providing the primary interface between ground control stations and the autopilot.
 *          
 *          Architecture Overview:
 *          - GCS singleton manages multiple GCS_MAVLINK channel instances (one per telemetry port)
 *          - Message handling pipeline: receive → parse → dispatch to handle_* methods → send acknowledgments
 *          - Stream system provides rate-controlled telemetry broadcasting
 *          - Mission/parameter protocols handle waypoint and configuration management
 *          - Command dispatcher routes MAV_CMD commands to appropriate vehicle-specific handlers
 *          
 *          Key Classes:
 *          - GCS: Singleton manager coordinating all MAVLink channels and providing broadcast messaging
 *          - GCS_MAVLINK: Per-channel transport class handling message framing, parsing, and routing
 *          - GCS_MAVLINK_InProgress: Tracks long-running commands requiring multiple ACK/IN_PROGRESS messages
 *          
 *          Thread Safety:
 *          - Message sending is thread-safe via comm_send_lock/unlock semaphores
 *          - Message handling runs in main thread during update_receive() calls
 *          - Stream scheduling runs in update_send() at configured rates
 *          
 * @note Vehicle-specific subclasses (e.g., GCS_Copter) extend this for vehicle-specific message handling
 * @warning Message handlers must complete quickly to avoid blocking the receive loop
 * 
 * Source: libraries/GCS_MAVLink/GCS.h
 */
#pragma once

#include "GCS_config.h"

#if HAL_GCS_ENABLED

#include <AP_AdvancedFailsafe/AP_AdvancedFailsafe_config.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include "GCS_MAVLink.h"
#include <AP_Mission/AP_Mission.h>
#include <stdint.h>
#include "MAVLink_routing.h"
#include <AP_RTC/JitterCorrection.h>
#include <AP_Common/Bitmask.h>
#include <AP_LTM_Telem/AP_LTM_Telem.h>
#include <AP_Devo_Telem/AP_Devo_Telem.h>
#include <AP_Filesystem/AP_Filesystem_config.h>
#include <AP_Frsky_Telem/AP_Frsky_config.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Mount/AP_Mount_config.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_RangeFinder/AP_RangeFinder_config.h>
#include <AP_Winch/AP_Winch_config.h>
#include <AP_AHRS/AP_AHRS_config.h>
#include <AP_Arming/AP_Arming_config.h>
#include <AP_Airspeed/AP_Airspeed_config.h>
#include <AP_Follow/AP_Follow.h>

#include "ap_message.h"

#define GCS_DEBUG_SEND_MESSAGE_TIMINGS 0

#ifndef HAL_GCS_ALLOW_PARAM_SET_DEFAULT
#define HAL_GCS_ALLOW_PARAM_SET_DEFAULT 1
#endif  // HAL_GCS_IGNORE_PARAM_SET_DEFAULT

/**
 * @brief Helper function to track transmit buffer space exhaustion
 * @param chan MAVLink channel that ran out of space
 * @note Increments statistics counter for monitoring channel congestion
 */
void gcs_out_of_space_to_send(mavlink_channel_t chan);

/**
 * @brief Check if a message of specified length will fit in channel buffer
 * @param chan MAVLink channel to check
 * @param max_payload_len Maximum message payload length in bytes
 * @return true if message will fit, false if insufficient space
 */
bool check_payload_size(mavlink_channel_t chan, uint16_t max_payload_len);

/**
 * @def PAYLOAD_SIZE
 * @brief Calculate total space required to send a MAVLink message including channel overhead
 * 
 * @details Calculates message size including:
 *          - MAVLink message payload length (MAVLINK_MSG_ID_##id##_LEN)
 *          - Channel protocol overhead (MAVLink 1 vs MAVLink 2 differ)
 *          - Framing bytes, CRC, and signature if enabled
 *          
 * @param chan MAVLink channel ID
 * @param id MAVLink message ID (without MAVLINK_MSG_ID_ prefix)
 * @return unsigned Total bytes required in transmit buffer
 * 
 * @note Despite the name "PAYLOAD_SIZE", this includes ALL overhead, not just payload
 * @warning MAVLink 2 has higher overhead than MAVLink 1 due to extended headers
 */
#define PAYLOAD_SIZE(chan, id) (unsigned(GCS_MAVLINK::packet_overhead_chan(chan)+MAVLINK_MSG_ID_ ## id ## _LEN))

/**
 * @def HAVE_PAYLOAD_SPACE
 * @brief Check if specified MAVLink message will fit in current channel transmit buffer
 * 
 * @details Evaluates to boolean expression checking:
 *          1. Current transmit buffer space via comm_get_txspace()
 *          2. Required space via PAYLOAD_SIZE()
 *          3. If insufficient space, calls gcs_out_of_space_to_send() to increment counter
 *          
 * @param _chan MAVLink channel to check
 * @param id MAVLink message ID (without MAVLINK_MSG_ID_ prefix)
 * @return true if message will fit, false if insufficient space
 * 
 * @note Uses comma operator to increment statistics counter on failure
 * @note Can be used anywhere in code, not just within GCS_MAVLINK methods
 */
#define HAVE_PAYLOAD_SPACE(_chan, id) (comm_get_txspace(_chan) >= PAYLOAD_SIZE(_chan, id) ? true : (gcs_out_of_space_to_send(_chan), false))

/**
 * @def CHECK_PAYLOAD_SIZE
 * @brief Early-return macro that exits function if insufficient space for message
 * 
 * @details Inserts code:
 *          if (!check_payload_size(MAVLINK_MSG_ID_##id##_LEN)) return false
 *          
 *          Use within GCS_MAVLINK methods to guard message sending.
 *          
 * @param id MAVLink message ID (without MAVLINK_MSG_ID_ prefix)
 * @return Implicitly returns false from calling function if no space
 * 
 * @note Only use in GCS_MAVLINK methods where 'this' context provides channel
 * @note Calling function must return bool type
 */
#define CHECK_PAYLOAD_SIZE(id) if (!check_payload_size(MAVLINK_MSG_ID_ ## id ## _LEN)) return false

/**
 * @def CHECK_PAYLOAD_SIZE2
 * @brief Early-return macro that exits function if insufficient space (requires 'chan' variable)
 * 
 * @details Inserts code:
 *          if (!HAVE_PAYLOAD_SPACE(chan, id)) return false
 *          
 *          Use in any function with 'chan' variable in scope.
 *          
 * @param id MAVLink message ID (without MAVLINK_MSG_ID_ prefix)
 * @return Implicitly returns false from calling function if no space
 * 
 * @note Requires mavlink_channel_t chan variable in scope
 * @note Calling function must return bool type
 */
#define CHECK_PAYLOAD_SIZE2(id) if (!HAVE_PAYLOAD_SPACE(chan, id)) return false

/**
 * @def CHECK_PAYLOAD_SIZE2_VOID
 * @brief Early-return macro for void functions that exits if insufficient space
 * 
 * @details Inserts code:
 *          if (!HAVE_PAYLOAD_SPACE(chan, id)) return
 *          
 *          Use in void functions with 'chan' variable in scope.
 *          
 * @param chan MAVLink channel to check
 * @param id MAVLink message ID (without MAVLINK_MSG_ID_ prefix)
 * @return Implicitly returns (void) from calling function if no space
 * 
 * @note Calling function must have void return type
 */
#define CHECK_PAYLOAD_SIZE2_VOID(chan, id) if (!HAVE_PAYLOAD_SPACE(chan, id)) return

/**
 * @def GCS_MAVLINK_CHAN_METHOD_DEFINITIONS
 * @brief Code generation macro for vehicle-specific channel accessor methods
 * 
 * @details Generates two type-safe channel accessor methods:
 *          - Non-const version returning subclass_name*
 *          - Const version returning const subclass_name*
 *          
 *          Use in vehicle-specific GCS subclass (e.g., GCS_Copter) to provide
 *          properly typed access to vehicle-specific GCS_MAVLINK backends.
 *          
 * @param subclass_name Vehicle-specific GCS_MAVLINK type (e.g., GCS_MAVLINK_Copter)
 * 
 * @note Performs bounds checking on channel offset
 * @note Returns nullptr if offset exceeds _num_gcs
 * 
 * Example usage in GCS_Copter:
 * @code
 * class GCS_Copter : public GCS {
 *     GCS_MAVLINK_CHAN_METHOD_DEFINITIONS(GCS_MAVLINK_Copter)
 *     // ... other vehicle-specific methods
 * };
 * @endcode
 */
#define GCS_MAVLINK_CHAN_METHOD_DEFINITIONS(subclass_name) \
    subclass_name *chan(const uint8_t ofs) override {                   \
        if (ofs >= _num_gcs) {                                           \
            return nullptr;                                             \
        }                                                               \
        return (subclass_name *)_chan[ofs];                        \
    }                                                                   \
                                                                        \
    const subclass_name *chan(const uint8_t ofs) const override { \
        if (ofs >= _num_gcs) {                                           \
            return nullptr;                                             \
        }                                                               \
        return (subclass_name *)_chan[ofs];                        \
    }


#if HAL_MAVLINK_INTERVALS_FROM_FILES_ENABLED
/**
 * @class DefaultIntervalsFromFiles
 * @brief Manages file-based default message interval configuration
 * 
 * @details Loads message interval configuration from files (SD card or ROMFS),
 *          allowing customization of telemetry rates without recompilation.
 *          Configuration files specify default intervals for ap_message IDs
 *          that override compiled-in defaults.
 *          
 *          Typical configuration file format:
 *          - One message per line: message_id,interval_ms
 *          - Loaded during channel initialization
 *          
 *          Storage is dynamically allocated based on max_num capacity.
 *          
 * @note Only available when HAL_MAVLINK_INTERVALS_FROM_FILES_ENABLED is defined
 * @note Configuration files typically located at: @MAVLink/message_intervals.txt
 */
class DefaultIntervalsFromFiles
{

public:

    /**
     * @brief Constructor allocating storage for interval mappings
     * @param max_num Maximum number of message interval entries to support
     * @note Allocates heap memory for _intervals array
     */
    DefaultIntervalsFromFiles(uint16_t max_num);
    
    /**
     * @brief Destructor freeing allocated storage
     */
    ~DefaultIntervalsFromFiles();

    /**
     * @brief Set default interval for a specific ap_message
     * @param id ap_message identifier
     * @param interval Transmission interval in milliseconds
     * @note Adds new entry or updates existing entry
     */
    void set(ap_message id, uint16_t interval);
    
    /**
     * @brief Get number of configured message intervals
     * @return Count of stored interval mappings
     */
    uint16_t num_intervals() const {
        return _num_intervals;
    }
    
    /**
     * @brief Retrieve interval for specified ap_message
     * @param id ap_message identifier to look up
     * @param[out] interval Reference to store found interval (milliseconds)
     * @return true if interval found, false if message not in configuration
     */
    bool get_interval_for_ap_message_id(ap_message id, uint16_t &interval) const;
    
    /**
     * @brief Get ap_message ID at specified index
     * @param ofs Index into stored intervals (0 to num_intervals()-1)
     * @return ap_message at specified index
     * @warning No bounds checking - caller must ensure ofs < num_intervals()
     */
    ap_message id_at(uint8_t ofs) const;
    
    /**
     * @brief Get interval value at specified index
     * @param ofs Index into stored intervals (0 to num_intervals()-1)
     * @return Interval in milliseconds at specified index
     * @warning No bounds checking - caller must ensure ofs < num_intervals()
     */
    uint16_t interval_at(uint8_t ofs) const;

private:

    /**
     * @brief Storage structure for message ID and interval pair
     */
    struct from_file_default_interval {
        ap_message id;          ///< Message identifier
        uint16_t interval;      ///< Transmission interval in milliseconds
    };

    from_file_default_interval *_intervals;  ///< Dynamically allocated array of intervals

    uint16_t _num_intervals;   ///< Current number of stored intervals
    uint16_t _max_intervals;   ///< Maximum capacity of _intervals array
};
#endif

/**
 * @class GCS_MAVLINK_InProgress
 * @brief Tracks long-running commands that require periodic IN_PROGRESS acknowledgments
 * 
 * @details Some MAV_CMD commands cannot complete in a single handler call and require
 *          extended processing time. This class manages:
 *          - Periodic MAV_RESULT_IN_PROGRESS status messages to prevent GCS timeout
 *          - Final COMMAND_ACK with success/failure result
 *          - Timeout detection for abandoned tasks
 *          
 *          Typical usage:
 *          1. Command handler calls get_task() to allocate tracker
 *          2. Handler initiates long-running operation (e.g., airspeed calibration)
 *          3. Tracker sends periodic IN_PROGRESS ACKs via send_in_progress()
 *          4. Operation completion calls conclude() with final result
 *          
 * @note Maximum 1 concurrent in-progress task supported (in_progress_tasks[1])
 * @warning Tasks must complete or be aborted to free tracker for subsequent commands
 */
class GCS_MAVLINK_InProgress
{
public:
    /**
     * @enum Type
     * @brief Identifier for long-running command types
     */
    enum class Type {
        NONE,           ///< No task in progress
        AIRSPEED_CAL,   ///< Airspeed sensor calibration (MAV_CMD_PREFLIGHT_CALIBRATION)
        SD_FORMAT,      ///< SD card format operation (MAV_CMD_STORAGE_FORMAT)
    };

    /**
     * @brief Send final COMMAND_ACK with result and free task tracker
     * @param result MAV_RESULT code (MAV_RESULT_ACCEPTED, MAV_RESULT_FAILED, etc.)
     * @return true if ACK sent successfully, false if insufficient channel space
     * @note Sets task = Type::NONE to release tracker
     * @note May fail if transmit buffer full; caller should retry on false
     */
    bool conclude(MAV_RESULT result);
    
    /**
     * @brief Send COMMAND_ACK with MAV_RESULT_IN_PROGRESS status
     * @return true if IN_PROGRESS ACK sent successfully, false if insufficient channel space
     * @note Call periodically during long operation to prevent GCS timeout
     * @note Typical interval: 1-2 seconds
     */
    bool send_in_progress();
    
    /**
     * @brief Abort task without sending any further acknowledgments
     * @details Sets task = Type::NONE immediately, freeing the tracker.
     *          Use when operation is cancelled or when final ACK already sent by other means.
     */
    void abort() { task = Type::NONE; }

    Type task;        ///< Current task type (NONE if tracker available)
    MAV_CMD mav_cmd;  ///< MAV_CMD being tracked

    /**
     * @brief Allocate a task tracker for long-running command
     * @param cmd MAV_CMD command ID to track
     * @param t Task type identifier
     * @param sysid System ID of requesting GCS
     * @param compid Component ID of requesting GCS
     * @param chan MAVLink channel where command was received
     * @return Pointer to allocated tracker, or nullptr if no tracker available
     * @note Currently supports max 1 concurrent task
     * @note Caller must call conclude() or abort() when done to free tracker
     */
    static class GCS_MAVLINK_InProgress *get_task(MAV_CMD cmd, Type t, uint8_t sysid, uint8_t compid, mavlink_channel_t chan);

    /**
     * @brief Check all task trackers for timeout and cleanup
     * @details Called periodically from main loop to:
     *          - Detect tasks exceeding timeout threshold
     *          - Send final failure ACK for timed-out tasks
     *          - Free abandoned trackers
     * @note Typical call frequency: 1Hz
     */
    static void check_tasks();

private:

    uint8_t requesting_sysid;     ///< System ID of GCS that sent command
    uint8_t requesting_compid;    ///< Component ID of GCS that sent command
    mavlink_channel_t chan;       ///< Channel where command was received and ACKs sent

    /**
     * @brief Internal method to send COMMAND_ACK message
     * @param result MAV_RESULT code to send
     * @return true if ACK sent successfully, false if insufficient channel space
     */
    bool send_ack(MAV_RESULT result);

    static GCS_MAVLINK_InProgress in_progress_tasks[1];  ///< Static array of task trackers (currently 1)

    static uint32_t last_check_ms;  ///< Timestamp of last check_tasks() call for timeout detection
};

/**
 * @class GCS_MAVLINK
 * @brief Per-channel MAVLink transport and message handling
 * 
 * @details One instance per telemetry port (UART/UDP/TCP). Provides complete MAVLink
 *          communication services:
 *          
 *          Core Responsibilities:
 *          - Message framing and parsing (MAVLink 1 and MAVLink 2 protocol support)
 *          - Routing and dispatching received messages to handle_* methods
 *          - Stream scheduling for rate-controlled telemetry broadcasting
 *          - Mission/parameter protocol implementation (upload/download/request)
 *          - Command dispatching (MAV_CMD routing to vehicle-specific handlers)
 *          - Channel flow control and transmit buffer management
 *          
 *          Threading Model:
 *          - Message parsing runs in main thread via update_receive()
 *          - Message sending runs in main thread via update_send()
 *          - Sending is thread-safe via semaphore (comm_send_lock/unlock)
 *          - Handlers must be fast to avoid blocking receive loop
 *          
 *          Stream System:
 *          - Messages grouped into streams (RAW_SENSORS, EXTENDED_STATUS, etc.)
 *          - Each stream has configurable rate via SRx_* parameters
 *          - Bucket-based scheduling ensures even distribution
 *          
 *          Vehicle Integration:
 *          - Vehicle-specific subclass (e.g., GCS_MAVLINK_Copter) extends this
 *          - Override handle_command_int_packet() for custom commands
 *          - Override try_send_message() for custom telemetry
 *          
 * @note Each GCS_MAVLINK instance is tied to one mavlink_channel_t
 * @note Maximum channels defined by MAVLINK_COMM_NUM_BUFFERS (typically 6)
 * @warning Message handlers must complete quickly (<1ms typical) to avoid receive overruns
 */
class GCS_MAVLINK
{
public:
    friend class GCS;

    /**
     * @brief Constructor initializing channel with UART driver
     * @param uart UART driver instance for this MAVLink channel
     * @note Does not start communication; call init() to activate channel
     */
    GCS_MAVLINK(AP_HAL::UARTDriver &uart);
    
    /**
     * @brief Virtual destructor for proper cleanup
     */
    virtual ~GCS_MAVLINK() {}

    static const struct AP_Param::GroupInfo        var_info[];  ///< Parameter metadata for stream rates

    /**
     * @brief Get pointer to channel message buffer for parsing
     * @return Pointer to mavlink_message_t buffer used by parser
     * @note Used internally by MAVLink library during message decode
     */
    mavlink_message_t *channel_buffer() { return &_channel_buffer; }
    
    /**
     * @brief Get pointer to channel status structure for parsing
     * @return Pointer to mavlink_status_t tracking parse state
     * @note Used internally by MAVLink library during message decode
     */
    mavlink_status_t *channel_status() { return &_channel_status; }

    /**
     * @brief Process incoming bytes from channel and dispatch complete messages
     * @param max_time_us Maximum microseconds to spend processing (default 1000us = 1ms)
     * @details Reads available bytes from UART, feeds to MAVLink parser, and dispatches
     *          successfully decoded messages to packetReceived() and handle_message().
     *          Time-limited to prevent starving other system functions.
     * @note Call periodically from main loop (typically 400Hz)
     * @note Will process multiple messages if available within time limit
     */
    void        update_receive(uint32_t max_time_us=1000);
    
    /**
     * @brief Send queued messages and periodic stream updates
     * @details Performs transmission tasks:
     *          - Sends deferred messages (HEARTBEAT, parameters)
     *          - Processes stream buckets for scheduled telemetry
     *          - Sends pushed messages (spontaneous events)
     *          - Respects transmit buffer space limits
     * @note Call periodically from main loop (typically 400Hz)
     * @note Sending is rate-limited by stream intervals and buffer availability
     */
    void        update_send();
    
    /**
     * @brief Initialize channel with instance number
     * @param instance Channel instance ID (0 to MAVLINK_COMM_NUM_BUFFERS-1)
     * @return true if initialization successful, false on failure
     * @note Sets up channel ID, initializes stream rates, loads signing keys
     * @note Must be called before channel can send/receive messages
     */
    bool        init(uint8_t instance);
    
    /**
     * @brief Queue ap_message for transmission
     * @param id ap_message identifier to send
     * @note Message sent during next update_send() call if buffer space available
     * @note Some messages (e.g., HEARTBEAT) have dedicated queues; others use stream buckets
     */
    void        send_message(enum ap_message id);
    
    /**
     * @brief Send formatted text message to GCS
     * @param severity MAV_SEVERITY level (INFO, WARNING, ERROR, etc.)
     * @param fmt printf-style format string
     * @note Text queued as STATUSTEXT message
     * @note Thread-safe; can be called from any thread
     */
    void        send_text(MAV_SEVERITY severity, const char *fmt, ...) const FMT_PRINTF(3, 4);
    
    /**
     * @brief Send next queued parameter value to GCS
     * @details Called from update_send() to handle PARAM_REQUEST_LIST protocol.
     *          Sends parameters one at a time to avoid overwhelming channel.
     * @note Automatically throttled to prevent blocking other messages
     */
    void        queued_param_send();
    
    /**
     * @brief Send next queued mission item to GCS
     * @details Called from update_send() to handle MISSION_REQUEST protocol.
     *          Sends mission items one at a time during upload/download.
     * @note Part of mission protocol state machine
     */
    void        queued_mission_request_send();

    /**
     * @brief Check if channel is using MAVLink 1 protocol
     * @return true if MAVLink 1, false if MAVLink 2
     * @note MAVLink 2 has larger message IDs and signing support
     * @note Protocol auto-negotiated based on received heartbeat
     */
    bool sending_mavlink1() const;

    /**
     * @brief Check if mission upload/download is in progress
     * @return true if actively requesting mission items from GCS, false otherwise
     * @note Used to prioritize mission messages during transfer
     */
    bool requesting_mission_items() const;

    /**
     * @brief Get available transmit buffer space
     * @return Bytes available in transmit buffer (0 if locked, max 8192)
     * @details Returns minimum of:
     *          - Actual UART transmit buffer space
     *          - 8192 bytes (cap to prevent over-committing in single loop)
     *          Returns 0 if channel locked (e.g., during SERIAL_CONTROL passthrough)
     * @note Used by PAYLOAD_SIZE macros to determine if message will fit
     * @note Capped at 8192 to prevent sending too much in one update cycle
     */
    uint16_t txspace() const {
        if (_locked) {
            return 0;
        }
        // there were concerns over return a too-large value for
        // txspace (in case we tried to do too much with the space in
        // a single loop):
        return MIN(_port->txspace(), 8192U);
    }

    /**
     * @brief Check if message of specified length will fit in buffer
     * @param max_payload_len Maximum message payload length in bytes
     * @return true if message will fit, false if insufficient space
     * @note Includes protocol overhead in calculation (not just payload)
     * @note Calls out_of_space_to_send() if insufficient space for statistics
     */
    bool check_payload_size(uint16_t max_payload_len);

    /**
     * @brief Track instance when insufficient buffer space for message
     * @note Increments out_of_space_to_send_count for telemetry monitoring
     * @note Called automatically by HAVE_PAYLOAD_SPACE and check_payload_size
     */
    void out_of_space_to_send() { out_of_space_to_send_count++; }

    /**
     * @brief Send MISSION_ACK message to GCS
     * @param msg Original mission message being acknowledged
     * @param mission_type Type of mission (MISSION, FENCE, RALLY)
     * @param result MAV_MISSION_RESULT code (ACCEPTED, ERROR, etc.)
     * @note Part of mission protocol - acknowledges mission operations
     * @note Inline function that checks buffer space before sending
     */
    void send_mission_ack(const mavlink_message_t &msg,
                          MAV_MISSION_TYPE mission_type,
                          MAV_MISSION_RESULT result) const {
        CHECK_PAYLOAD_SIZE2_VOID(chan, MISSION_ACK);
        mavlink_msg_mission_ack_send(chan,
                                     msg.sysid,
                                     msg.compid,
                                     result,
                                     mission_type);
    }

    /**
     * @brief Callback invoked on successful MAVLink message decode
     * @param status MAVLink parser status structure
     * @param msg Decoded MAVLink message
     * @details Called by update_receive() after successful message parsing.
     *          Performs:
     *          - Routing table updates
     *          - Signing validation
     *          - Message acceptance checks
     *          - Dispatch to handle_message()
     * @warning Must be fast - runs in receive path at parse rate
     * @note Virtual to allow vehicle-specific packet handling
     */
    virtual void packetReceived(const mavlink_status_t &status,
                                const mavlink_message_t &msg);

    /**
     * @brief Send pre-packed MAVLink message by ID
     * @param msgid MAVLink message ID
     * @param pkt Pointer to packed message buffer
     * @details Looks up message entry in MAVLink message database,
     *          checks buffer space, and transmits with proper framing.
     * @note Automatically handles MAVLink 1 vs 2 protocol differences
     * @note Silently drops message if msgid not found in message database
     */
    void send_message(uint32_t msgid, const char *pkt) {
        const mavlink_msg_entry_t *entry = mavlink_get_msg_entry(msgid);
        if (entry == nullptr) {
            return;
        }
        send_message(pkt, entry);
    }
    
    /**
     * @brief Send pre-packed MAVLink message with entry metadata
     * @param pkt Pointer to packed message buffer
     * @param entry MAVLink message entry with ID, length, CRC metadata
     * @details Checks buffer space and transmits message with:
     *          - Proper sequence numbering
     *          - CRC calculation (including crc_extra)
     *          - Signing if enabled
     *          - MAVLink 1 or 2 framing based on channel state
     * @note Returns silently if insufficient buffer space
     */
    void send_message(const char *pkt, const mavlink_msg_entry_t *entry) {
        if (!check_payload_size(entry->max_msg_len)) {
            return;
        }
        _mav_finalize_message_chan_send(chan,
                                        entry->msgid,
                                        pkt,
                                        entry->min_msg_len,
                                        entry->max_msg_len,
                                        entry->crc_extra);
    }

    /**
     * @brief Get UART driver for this channel
     * @return Pointer to AP_HAL::UARTDriver instance
     * @note Used for advanced port control and passthrough operations
     */
    AP_HAL::UARTDriver *get_uart() { return _port; }

    /**
     * @brief Cap message interval to prevent exceeding scheduler rate
     * @param interval_ms Requested message interval in milliseconds
     * @return Capped interval (minimum based on SCHED_LOOP_RATE)
     * @details Prevents messages from being scheduled faster than 0.8 * SCHED_LOOP_RATE
     *          to ensure main loop isn't overwhelmed by telemetry.
     * @note Typical SCHED_LOOP_RATE = 400Hz → minimum interval ~3ms
     */
    uint16_t cap_message_interval(uint16_t interval_ms) const;

    /**
     * @brief Send PARAM_VALUE message to GCS
     * @param param_name Parameter name (must point to 17-byte buffer: 16 chars + null)
     * @param param_type AP_Param type (AP_PARAM_FLOAT, AP_PARAM_INT32, etc.)
     * @param param_value Parameter value as float
     * @warning param_name MUST point to a buffer of at least AP_MAX_NAME_SIZE+1 (17) bytes
     * @note Part of parameter protocol for PARAM_REQUEST_READ/PARAM_REQUEST_LIST
     */
    void send_parameter_value(const char *param_name,
                              ap_var_type param_type,
                              float param_value);

    /**
     * @enum streams
     * @brief Telemetry stream identifiers for grouping related messages
     * 
     * @details Each stream contains multiple related MAVLink messages sent at the same rate.
     *          Stream rates configured via SRx_* parameters (SR0_RAW_SENS, SR0_EXT_STAT, etc.)
     *          where x is the channel number (0-3).
     *          
     *          Stream contents defined in all_stream_entries[] in vehicle-specific code.
     *          
     * @warning Order MUST match:
     *          - AP_Int16 streamRates[] array
     *          - default_rates[] in GCS_MAVLINK_Parameters.cpp
     *          - Stream rate parameter definitions
     * 
     * @note Typical stream contents:
     *       - RAW_SENSORS: IMU, baro, mag, GPS raw data
     *       - EXTENDED_STATUS: System status, mode, battery, fences
     *       - RC_CHANNELS: RC input values
     *       - RAW_CONTROLLER: PID outputs, attitude targets
     *       - POSITION: Global/local position, velocities
     *       - EXTRA1: Attitude quaternion, attitude rates
     *       - EXTRA2: VFR_HUD, simstate
     *       - EXTRA3: AHRS, wind, terrain data
     */
    enum streams : uint8_t {
        STREAM_RAW_SENSORS,      ///< Raw sensor data (IMU, baro, mag, GPS)
        STREAM_EXTENDED_STATUS,  ///< System status, mode, battery state
        STREAM_RC_CHANNELS,      ///< RC input channels
        STREAM_RAW_CONTROLLER,   ///< Controller outputs and targets
        STREAM_POSITION,         ///< Position and velocity estimates
        STREAM_EXTRA1,           ///< Attitude quaternion, rates
        STREAM_EXTRA2,           ///< VFR HUD, simulation state
        STREAM_EXTRA3,           ///< AHRS, wind, terrain
        STREAM_PARAMS,           ///< Parameter values (special handling)
        STREAM_ADSB,             ///< ADS-B traffic data
        NUM_STREAMS              ///< Total number of streams (sentinel)
    };

    /**
     * @brief Check if this is the primary high-bandwidth channel
     * @return true if this is MAVLINK_COMM_0 (typically USB or high-speed link)
     * @note High-bandwidth channels may send additional data like full rate sensors
     */
    bool is_high_bandwidth() { return chan == MAVLINK_COMM_0; }
    
    /**
     * @brief Check if this channel has hardware flow control
     * @return true if UART has RTS/CTS flow control enabled
     * @note Channels with flow control can safely use larger buffers without overruns
     */
    bool have_flow_control();

    /**
     * @brief Check if this channel has seen recent traffic
     * @return true if channel is active (has sent/received data recently)
     * @details Channel becomes active after first heartbeat received from GCS
     * @note Used to avoid sending to inactive channels
     */
    bool is_active() const {
        return GCS_MAVLINK::active_channel_mask() & (1 << (chan-MAVLINK_COMM_0));
    }
    
    /**
     * @brief Check if this channel is actively sending stream messages
     * @return true if currently processing deferred message buckets
     * @note Streaming state indicates channel is past initialization and sending telemetry
     */
    bool is_streaming() const {
        return sending_bucket_id != no_bucket_to_send;
    }

    /**
     * @brief Get MAVLink channel ID for this instance
     * @return Channel ID (MAVLINK_COMM_0 through MAVLINK_COMM_N)
     */
    mavlink_channel_t get_chan() const { return chan; }
    
    /**
     * @brief Get timestamp of last received heartbeat from GCS
     * @return Milliseconds since boot when last HEARTBEAT received
     * @note Used for GCS timeout detection and failsafe triggering
     */
    uint32_t get_last_heartbeat_time() const { return last_heartbeat_time; };

    uint32_t        last_heartbeat_time; // milliseconds

    /**
     * @brief Record timestamp when valid traffic seen from our designated GCS
     * @param[in] seen_time_ms Milliseconds since boot when traffic was seen
     * @details Updates both per-channel and global GCS last-seen timestamps
     * @note Used for GCS connection monitoring and failsafe logic
     * @note "Valid traffic" includes heartbeats and some manual control messages
     */
    void sysid_mygcs_seen(uint32_t seen_time_ms);

    /**
     * @brief Get timestamp of last RADIO_STATUS message with valid remote RSSI
     * @return Milliseconds since boot when last valid remrssi received
     * @note Static accessor for shared radio status data across all channels
     * @note Returns 0 if no RADIO_STATUS with remrssi ever received
     */
    static uint32_t last_radio_status_remrssi_ms() {
        return last_radio_status.remrssi_ms;
    }
    
    /**
     * @brief Get telemetry radio RSSI as normalized float
     * @return RSSI value 0.0 (no signal) to 1.0 (full signal)
     * @note Based on most recent RADIO_STATUS message
     * @note Returns 0.0 if no radio status received or RSSI unavailable
     */
    static float telemetry_radio_rssi(); // 0==no signal, 1==full signal
    
    /**
     * @brief Check if last reported radio txbuf exceeds threshold
     * @param[in] txbuf_limit Threshold value (0-100 percent)
     * @return true if last RADIO_STATUS txbuf > txbuf_limit
     * @note Used to detect radio congestion and throttle telemetry
     * @note txbuf represents percentage of transmit buffer free
     */
    static bool last_txbuf_is_greater(uint8_t txbuf_limit);

    /**
     * @brief Mission item index to report as reached to GCS
     * @details Set to mission index when waypoint is reached, then sent via MISSION_ITEM_REACHED
     * @note AP_MISSION_CMD_INDEX_NONE indicates no pending reached notification
     */
    uint16_t mission_item_reached_index = AP_MISSION_CMD_INDEX_NONE;

    /**
     * @brief Generate MISSION_STATE enumeration for current mission progress
     * @param mission Reference to AP_Mission object
     * @return MISSION_STATE value (MISSION_STATE_ACTIVE, MISSION_STATE_PAUSED, etc.)
     * @note Vehicle-specific subclasses may override for custom mission states
     */
    virtual MISSION_STATE mission_state(const class AP_Mission &mission) const;
    
    /**
     * @brief Send MISSION_CURRENT message to GCS
     * @param mission Reference to AP_Mission object
     * @param seq Mission sequence number (waypoint index)
     * @note Sent when active mission item changes
     */
    void send_mission_current(const class AP_Mission &mission, uint16_t seq);

    /**
     * @brief Send HEARTBEAT message to GCS
     * @details Sends vehicle type, autopilot type, base mode, custom mode, system status
     * @note Called at 1Hz typically, critical for GCS connection monitoring
     */
    void send_heartbeat(void) const;
    
    /**
     * @brief Send MEMINFO message with memory usage statistics
     * @note Provides heap free/used, stack usage for debugging memory issues
     */
    void send_meminfo(void);
    
    /**
     * @brief Send FENCE_STATUS message with geofence state
     * @note Reports fence breaches, enabled status, breach count
     */
    void send_fence_status() const;
    
    /**
     * @brief Send POWER_STATUS message with power supply information
     * @note Includes Vcc, servo rail voltage, USB power status, brick valid flags
     */
    void send_power_status(void);
    
#if HAL_WITH_MCU_MONITORING
    /**
     * @brief Send MCU_STATUS message with MCU temperature and voltage
     * @note Only available on boards with MCU monitoring capability
     */
    void send_mcu_status(void);
#endif
    
    /**
     * @brief Send BATTERY_STATUS message for specific battery instance
     * @param instance Battery instance number (0-based)
     * @note Sends detailed battery info: voltages per cell, current, remaining capacity
     */
    void send_battery_status(const uint8_t instance) const;
    
    /**
     * @brief Send BATTERY_STATUS for next battery instance in rotation
     * @return true if message sent, false if no payload space
     * @note Rotates through all battery instances to report each one
     */
    bool send_battery_status();
    
    /**
     * @brief Send DISTANCE_SENSOR messages for all rangefinders
     * @note Sends one message per rangefinder, reports distance and orientation
     */
    void send_distance_sensor();
    /**
     * @brief Send RANGEFINDER message with downward-facing rangefinder data
     * @note Default implementation sends only downward-facing rangefinder
     * @note Rover overrides to send forward-facing rangefinder
     */
#if AP_MAVLINK_MSG_RANGEFINDER_SENDING_ENABLED
    virtual void send_rangefinder() const;
#endif
    
    /**
     * @brief Send OBSTACLE_DISTANCE message with proximity sensor data
     * @note Sends 360-degree obstacle distance array from proximity sensors
     */
    void send_proximity();
    
    /**
     * @brief Send NAV_CONTROLLER_OUTPUT message with navigation controller state
     * @note Pure virtual - vehicle-specific implementation required
     * @note Reports nav bearing, target bearing, crosstrack error, altitude error, airspeed error
     */
    virtual void send_nav_controller_output() const = 0;
    
    /**
     * @brief Send PID_TUNING message with PID controller values
     * @note Pure virtual - vehicle-specific implementation required
     * @note Reports P/I/D values, desired/achieved, and FF for selected axis
     */
    virtual void send_pid_tuning() = 0;
    
    /**
     * @brief Send AHRS2 message with secondary AHRS solution
     * @note Provides altitude, lat/lon, and roll/pitch/yaw from backup AHRS
     */
    void send_ahrs2();
    
    /**
     * @brief Send SYSTEM_TIME message with system and GPS time
     * @note Reports time_unix_usec (Unix epoch) and time_boot_ms (since boot)
     */
    void send_system_time() const;
    
    /**
     * @brief Send RC_CHANNELS message with all RC input values
     * @note Reports up to 18 RC channels plus RSSI
     */
    void send_rc_channels() const;
    
    /**
     * @brief Send RC_CHANNELS_RAW message with raw RC input (legacy)
     * @note Legacy message, RC_CHANNELS preferred for new implementations
     */
    void send_rc_channels_raw() const;
    
    /**
     * @brief Send RAW_IMU message with raw IMU sensor data
     * @note Sends accelerometer, gyroscope, and magnetometer raw values
     */
    void send_raw_imu();
    
    /**
     * @brief Send HIGHRES_IMU message with high-resolution IMU data
     * @note Includes temperature, pressure, and higher precision than RAW_IMU
     */
    void send_highres_imu();

    /**
     * @brief Send SCALED_PRESSURE message for specific barometer instance
     * @param instance Barometer instance number
     * @param send_fn Function pointer to mavlink_msg_scaled_pressure*_send function
     * @note Helper function called by send_scaled_pressure(), send_scaled_pressure2(), etc.
     */
    void send_scaled_pressure_instance(uint8_t instance, void (*send_fn)(mavlink_channel_t chan, uint32_t time_boot_ms, float press_abs, float press_diff, int16_t temperature, int16_t temperature_press_diff));
    
    /**
     * @brief Send SCALED_PRESSURE message (primary barometer)
     * @note Reports absolute pressure, differential pressure (airspeed), and temperature
     */
    void send_scaled_pressure();
    
    /**
     * @brief Send SCALED_PRESSURE2 message (secondary barometer)
     * @note Reports second barometer instance data
     */
    void send_scaled_pressure2();
    
    /**
     * @brief Send SCALED_PRESSURE3 message (tertiary barometer)
     * @note Virtual to allow Sub to override for depth sensor
     */
    virtual void send_scaled_pressure3();
    
#if AP_AIRSPEED_ENABLED
    /**
     * @brief Send AIRSPEED message rotating through airspeed sensor instances
     * @note Rotates through all airspeed sensors to report each one
     */
    void send_airspeed();
    uint8_t last_airspeed_idx; ///< Last airspeed instance sent (for rotation)
#endif
    
    /**
     * @brief Send SIMSTATE message with SITL simulation state
     * @note Only available in SITL builds, reports simulator internal state
     */
    void send_simstate() const;
    
    /**
     * @brief Send SIM_STATE message with simulation state (legacy)
     * @note Legacy version of SIMSTATE message
     */
    void send_sim_state() const;
    
    /**
     * @brief Send AHRS message with primary AHRS solution
     * @note Reports attitude (roll/pitch/yaw), position, velocity from AHRS
     */
    void send_ahrs();
    
    /**
     * @brief Send OPTICAL_FLOW message with optical flow sensor data
     * @note Reports flow rate in x/y, quality, ground distance
     */
    void send_opticalflow();
    
    /**
     * @brief Send ATTITUDE message with vehicle attitude
     * @note Virtual to allow vehicle-specific overrides, reports roll/pitch/yaw and rates
     */
    virtual void send_attitude() const;
    
    /**
     * @brief Send ATTITUDE_QUATERNION message with attitude as quaternion
     * @note Virtual to allow vehicle-specific overrides, quaternion representation preferred
     */
    virtual void send_attitude_quaternion() const;
    
    /**
     * @brief Send AUTOPILOT_VERSION message with firmware version and capabilities
     * @note Reports git hash, capabilities bitmask, flight SW version, middleware/OS version
     */
    void send_autopilot_version() const;
    
    /**
     * @brief Send EXTENDED_SYS_STATE message with extended system state
     * @note Reports VTOL state, landed state
     */
    void send_extended_sys_state() const;
    
    /**
     * @brief Send LOCAL_POSITION_NED message with local position and velocity
     * @note Position relative to home in NED frame (meters), velocity in m/s
     */
    void send_local_position() const;
    
    /**
     * @brief Send VFR_HUD message with HUD display values
     * @note Reports airspeed, groundspeed, heading, throttle, altitude, climb rate
     * @note Legacy message for simplified HUD displays
     */
    void send_vfr_hud();
    
    /**
     * @brief Send VIBRATION message with vibration levels
     * @note Reports RMS vibration on x/y/z axes and clipping counts
     */
    void send_vibration() const;
    
    /**
     * @brief Send GIMBAL_DEVICE_ATTITUDE_STATUS message
     * @note Reports gimbal device attitude and status flags
     */
    void send_gimbal_device_attitude_status() const;
    
    /**
     * @brief Send GIMBAL_MANAGER_INFORMATION message
     * @note Reports gimbal manager capabilities and limits
     */
    void send_gimbal_manager_information() const;
    
    /**
     * @brief Send GIMBAL_MANAGER_STATUS message
     * @note Reports current gimbal manager state and mode
     */
    void send_gimbal_manager_status() const;
    
    /**
     * @brief Send NAMED_VALUE_FLOAT message with named float value
     * @param name Value name (up to 10 characters)
     * @param value Float value to send
     * @note Useful for debugging and custom telemetry values
     */
    void send_named_float(const char *name, float value) const;
    
    /**
     * @brief Send HOME_POSITION message with home location
     * @note Reports home position in lat/lon/alt and local frame
     */
    void send_home_position() const;
    
    /**
     * @brief Send GPS_GLOBAL_ORIGIN message with global coordinate system origin
     * @note Reports the origin of the local coordinate frame (typically home position)
     */
    void send_gps_global_origin() const;
    
    /**
     * @brief Send ATTITUDE_TARGET message with attitude controller targets
     * @note Virtual - default implementation does nothing, vehicles override as needed
     */
    virtual void send_attitude_target() {};
    
    /**
     * @brief Send POSITION_TARGET_GLOBAL_INT message with global position targets
     * @note Virtual - default implementation does nothing, vehicles override as needed
     */
    virtual void send_position_target_global_int() { };
    
    /**
     * @brief Send POSITION_TARGET_LOCAL_NED message with local position targets
     * @note Virtual - default implementation does nothing, vehicles override as needed
     */
    virtual void send_position_target_local_ned() { };
    
    /**
     * @brief Send SERVO_OUTPUT_RAW message with raw servo outputs
     * @note Reports PWM values for up to 16 servo outputs
     */
    void send_servo_output_raw();
    
    /**
     * @brief Send COMMAND_ACK for accelerometer calibration vehicle position
     * @param position Vehicle position bitmask for calibration step
     * @note Used during multi-position accelerometer calibration sequence
     */
    void send_accelcal_vehicle_position(uint32_t position);
    
    /**
     * @brief Send SCALED_IMU message for specific IMU instance
     * @param instance IMU instance number
     * @param send_fn Function pointer to mavlink_msg_scaled_imu*_send function
     * @note Helper function for sending SCALED_IMU, SCALED_IMU2, SCALED_IMU3
     */
    void send_scaled_imu(uint8_t instance, void (*send_fn)(mavlink_channel_t chan, uint32_t time_ms, int16_t xacc, int16_t yacc, int16_t zacc, int16_t xgyro, int16_t ygyro, int16_t zgyro, int16_t xmag, int16_t ymag, int16_t zmag, int16_t temperature));
    
    /**
     * @brief Send SYS_STATUS message with system status
     * @note Reports battery, sensors present/enabled/health, CPU load, communication errors
     * @note Critical for GCS health monitoring
     */
    void send_sys_status();
    
    /**
     * @brief Send SET_POSITION_TARGET_GLOBAL_INT message to external system
     * @param target_system Target system ID
     * @param target_component Target component ID
     * @param loc Target location
     * @note Used for commanding other vehicles or companion computers
     */
    void send_set_position_target_global_int(uint8_t target_system, uint8_t target_component, const Location& loc);
    
    /**
     * @brief Send RPM message with RPM sensor data
     * @note Reports RPM values from connected RPM sensors
     */
    void send_rpm() const;
    
    /**
     * @brief Send GENERATOR_STATUS message
     * @note Reports generator status, battery current, RPM, etc.
     */
    void send_generator_status() const;
#if AP_WINCH_ENABLED
    /**
     * @brief Send WINCH_STATUS message with winch state
     * @note Virtual - default implementation does nothing, vehicles with winch override
     */
    virtual void send_winch_status() const {};
#endif
    
    /**
     * @brief Calculate battery remaining percentage for display
     * @param instance Battery instance number
     * @return Battery remaining as percentage (0-100), or -1 if unavailable
     * @note Accounts for failsafe reserves and capacity estimation
     */
    int8_t battery_remaining_pct(const uint8_t instance) const;

#if HAL_HIGH_LATENCY2_ENABLED
    /**
     * @brief Send HIGH_LATENCY2 message with compressed telemetry
     * @note Compact message for high-latency/low-bandwidth links (satellite, LoRa)
     * @note Contains essential telemetry in ~48 bytes vs ~200 bytes for full stream
     */
    void send_high_latency2() const;
#endif // HAL_HIGH_LATENCY2_ENABLED
    
    /**
     * @brief Send UAVIONIX_ADSB_OUT_STATUS message
     * @note Reports status of uAvionix ADS-B OUT transponder
     */
    void send_uavionix_adsb_out_status() const;
    
    /**
     * @brief Send AUTOPILOT_STATE_FOR_GIMBAL_DEVICE message
     * @note Provides autopilot state information to gimbal for stabilization
     */
    void send_autopilot_state_for_gimbal_device() const;

    /**
     * @brief Send AVAILABLE_MODES message entry for mode at given index
     * @param index Mode index (1-based, not mode number!)
     * @return Total number of modes available
     * @note Pure virtual - vehicle-specific implementation required
     * @note Called iteratively to send all available flight modes to GCS
     */
    virtual uint8_t send_available_mode(uint8_t index) const = 0;

#if AP_MAVLINK_MSG_FLIGHT_INFORMATION_ENABLED
    /**
     * @brief Flight information state for FLIGHT_INFORMATION message
     */
    struct {
        MAV_LANDED_STATE last_landed_state; ///< Last reported landed state
        uint64_t takeoff_time_us;           ///< Takeoff time in microseconds (Unix epoch)
    } flight_info;

    /**
     * @brief Send FLIGHT_INFORMATION message with flight time and state
     * @note Reports time since takeoff, flight time, and landed state
     */
    void send_flight_information();
#endif

    /**
     * @brief Lock/unlock channel to prevent MAVLink usage
     * @param _lock true to lock channel, false to unlock
     * @note Used for SERIAL_CONTROL passthrough and other exclusive port access
     * @warning Locked channels cannot send/receive MAVLink messages
     */
    void lock(bool _lock) {
        _locked = _lock;
    }
    
    /**
     * @brief Check if channel is locked (unavailable for MAVLink)
     * @return true if channel is locked
     * @note Locked channels typically used for SERIAL_CONTROL or passthrough
     */
    bool locked() const {
        return _locked;
    }

    /**
     * @brief Get bitmask of active MAVLink channels
     * @return Bitmask where bit N = 1 if MAVLINK_COMM_N is active
     * @note Used by libraries to broadcast to all active channels
     * @note Active = has seen recent traffic
     */
    static uint8_t active_channel_mask(void) { return mavlink_active; }

    /**
     * @brief Get bitmask of streaming MAVLink channels
     * @return Bitmask where bit N = 1 if MAVLINK_COMM_N is streaming
     * @note Streaming = actively sending scheduled messages
     */
    static uint8_t streaming_channel_mask(void) { return chan_is_streaming; }

    /**
     * @brief Get bitmask of private MAVLink channels
     * @return Bitmask where bit N = 1 if MAVLINK_COMM_N is private
     * @note Private channels receive heartbeats but not broadcasts/forwards
     */
    static uint8_t private_channel_mask(void) { return mavlink_private; }

    /**
     * @brief Set a channel as private
     * @param chan MAVLink channel to make private
     * @note Private channels get heartbeats but not broadcast/forwarded packets
     * @note Useful for direct vehicle-to-vehicle links or companion computers
     */
    static void set_channel_private(mavlink_channel_t chan);

    /**
     * @brief Check if specific channel is private (static version)
     * @param _chan MAVLink channel to check
     * @return true if channel is private
     */
    static bool is_private(mavlink_channel_t _chan) {
        return (mavlink_private & (1U<<(unsigned)_chan)) != 0;
    }
    
    /**
     * @brief Check if this channel instance is private
     * @return true if this channel is private
     */
    bool is_private(void) const { return is_private(chan); }

#if HAL_HIGH_LATENCY2_ENABLED
    /**
     * @brief Flag indicating this is a high-latency link
     * @note High-latency links use compressed HIGH_LATENCY2 messages
     * @note Typical for satellite or LoRa connections
     */
    bool is_high_latency_link;
#endif

    /**
     * @brief Send MAVLink message to all components with this vehicle's system ID
     * @param msgid MAVLink message ID
     * @param pkt Pointer to message packet data
     * @param pkt_len Packet length in bytes
     * @note No-op if no component routes have been learned
     * @note Used for broadcasting to companion computers, gimbals, etc.
     */
    static void send_to_components(uint32_t msgid, const char *pkt, uint8_t pkt_len) { routing.send_to_components(msgid, pkt, pkt_len); }

    /**
     * @brief Disable packet forwarding and heartbeats on specific channel
     * @param chan MAVLink channel to disable routing for
     * @note Used to reduce traffic when components don't need broadcasts
     */
    static void disable_channel_routing(mavlink_channel_t chan) { routing.no_route_mask |= (1U<<(chan-MAVLINK_COMM_0)); }
    
    /**
     * @brief Find component in routing table by MAVLink type
     * @param mav_type MAV_TYPE to search for (e.g., MAV_TYPE_GIMBAL)
     * @param[out] sysid System ID of found component
     * @param[out] compid Component ID of found component
     * @param[out] channel Channel where component was seen
     * @return true if matching component found
     * @note Uses learned routing table from received messages
     */
    static bool find_by_mavtype(uint8_t mav_type, uint8_t &sysid, uint8_t &compid, mavlink_channel_t &channel) { return routing.find_by_mavtype(mav_type, sysid, compid, channel); }

    /**
     * @brief Find component by MAVLink type and component ID
     * @param mav_type MAV_TYPE to search for
     * @param compid Component ID to match
     * @param[out] sysid System ID of found component
     * @param[out] channel Channel where component was seen
     * @return true if match found
     * @note More specific than find_by_mavtype when multiple components of same type exist
     */
    static bool find_by_mavtype_and_compid(uint8_t mav_type, uint8_t compid, uint8_t &sysid, mavlink_channel_t &channel) { return routing.find_by_mavtype_and_compid(mav_type, compid, sysid, channel); }
    
    /**
     * @brief Find component by type and compid, return GCS_MAVLINK object
     * @param mav_type MAV_TYPE to search for
     * @param compid Component ID to match
     * @param[out] sysid System ID of found component
     * @return Pointer to GCS_MAVLINK object for the channel, or nullptr if not found
     * @note Convenience method that returns channel object instead of just channel enum
     */
    static GCS_MAVLINK *find_by_mavtype_and_compid(uint8_t mav_type, uint8_t compid, uint8_t &sysid);

    /**
     * @brief Update MAVLink signing timestamp when GPS lock acquired
     * @param timestamp_usec GPS timestamp in microseconds (Unix epoch)
     * @note Ensures signed messages have accurate timestamps
     */
    static void update_signing_timestamp(uint64_t timestamp_usec);

    /**
     * @brief Get packet overhead bytes for specific channel
     * @param chan MAVLink channel
     * @return Overhead bytes (varies by MAVLink version and signing)
     * @note MAVLink 1: 8 bytes, MAVLink 2: 12 bytes, +13 if signing enabled
     */
    static uint8_t packet_overhead_chan(mavlink_channel_t chan);

    /**
     * @brief Function pointer type for alternative protocol handlers
     * @note Allows switching channel to non-MAVLink protocols (e.g., DroneCAN, custom)
     * @note Handler receives: protocol_id, UART driver pointer
     * @return true if handler consumed the byte stream
     */
    FUNCTOR_TYPEDEF(protocol_handler_fn_t, bool, uint8_t, AP_HAL::UARTDriver *);

    /**
     * @struct stream_entries
     * @brief Maps stream IDs to their constituent ap_message IDs
     * @note Used to configure which messages belong to each stream
     */
    struct stream_entries {
        const streams stream_id;          ///< Stream identifier (e.g., STREAM_RAW_SENSORS)
        const ap_message *ap_message_ids; ///< Array of message IDs in this stream
        const uint8_t num_ap_message_ids; ///< Number of messages in ap_message_ids array
    };
    
    /**
     * @brief Vehicle-specific stream configuration table
     * @note Defined in vehicle subclass .cpp files (e.g., GCS_Copter.cpp)
     * @note Maps each stream enum to its message list
     */
    static const struct stream_entries all_stream_entries[];

    /**
     * @brief Get autopilot capability flags
     * @return Bitmask of MAV_PROTOCOL_CAPABILITY flags
     * @note Reports features like mission protocol, param float, etc.
     * @note Vehicle subclasses may override to add vehicle-specific capabilities
     */
    virtual uint64_t capabilities() const;
    
    /**
     * @brief Get stream slowdown factor in milliseconds
     * @return Extra delay added to stream intervals (0 = no slowdown)
     * @note Used to throttle telemetry on low-bandwidth links
     */
    uint16_t get_stream_slowdown_ms() const { return stream_slowdown_ms; }

    /**
     * @brief Set message transmission interval via MAVLink message ID
     * @param msg_id MAVLink message ID (e.g., MAVLINK_MSG_ID_ATTITUDE)
     * @param interval_us Interval in microseconds (-1 to disable, 0 for maximum rate)
     * @return MAV_RESULT_ACCEPTED on success
     * @note Implements MAV_CMD_SET_MESSAGE_INTERVAL functionality
     */
    MAV_RESULT set_message_interval(uint32_t msg_id, int32_t interval_us);

protected:

    /**
     * @brief Convert MAVLink coordinate frame to ArduPilot altitude frame
     * @param coordinate_frame MAVLink MAV_FRAME enum value
     * @param[out] frame ArduPilot Location::AltFrame to populate
     * @return true if conversion successful
     * @note Handles GLOBAL, GLOBAL_RELATIVE_ALT, GLOBAL_TERRAIN_ALT, etc.
     */
    bool mavlink_coordinate_frame_to_location_alt_frame(MAV_FRAME coordinate_frame,
                                                        Location::AltFrame &frame);

    /**
     * @brief Check if packet should be accepted from sender
     * @param status MAVLink parser status
     * @param msg Received MAVLink message
     * @return true if packet should be processed
     * @note Overridable for GCS sysid enforcement (MAV_GCS_SYSID parameter)
     * @note Can reject packets from unauthorized system IDs
     */
    bool accept_packet(const mavlink_status_t &status, const mavlink_message_t &msg) const;
    
    /**
     * @brief Set EKF origin to specified location
     * @param loc Origin location (lat, lon, alt)
     * @return MAV_RESULT_ACCEPTED on success, error code otherwise
     * @note Origin used for local coordinate frame transformations
     * @warning Should only be set once during initialization
     */
    MAV_RESULT set_ekf_origin(const Location& loc);

    /**
     * @brief Get vehicle base mode flags (pure virtual)
     * @return MAV_MODE_FLAG bitmask (ARMED, CUSTOM_MODE_ENABLED, etc.)
     * @note Vehicle subclasses must implement
     */
    virtual uint8_t base_mode() const = 0;
    
    /**
     * @brief Get overall system status
     * @return MAV_STATE enum (STANDBY, ACTIVE, CRITICAL, etc.)
     * @note Combines vehicle-specific status with common checks
     */
    MAV_STATE system_status() const;
    
    /**
     * @brief Get vehicle-specific system status (pure virtual)
     * @return MAV_STATE enum
     * @note Vehicle subclasses implement based on their state
     */
    virtual MAV_STATE vehicle_system_status() const = 0;

    /**
     * @brief Get VTOL state (for quadplanes)
     * @return MAV_VTOL_STATE enum
     * @note Default returns UNDEFINED, Plane overrides for QuadPlane support
     */
    virtual MAV_VTOL_STATE vtol_state() const { return MAV_VTOL_STATE_UNDEFINED; }
    
    /**
     * @brief Get landed state
     * @return MAV_LANDED_STATE enum (ON_GROUND, IN_AIR, etc.)
     * @note Default returns UNDEFINED, vehicles override with actual detection
     */
    virtual MAV_LANDED_STATE landed_state() const { return MAV_LANDED_STATE_UNDEFINED; }

    /**
     * @brief Convert AP_Param type to MAVLink parameter type
     * @param t AP_Param type enum
     * @return MAV_PARAM_TYPE enum
     * @note Maps AP_PARAM_INT8 → MAV_PARAM_TYPE_INT8, etc.
     */
    static MAV_PARAM_TYPE mav_param_type(enum ap_var_type t);

    /**
     * @brief Pointer to next parameter to send in queue
     * @note Used by parameter streaming protocol to track position
     */
    AP_Param *                  _queued_parameter;
    
    /**
     * @brief MAVLink channel ID for this instance
     * @note MAVLINK_COMM_0 = primary telemetry, MAVLINK_COMM_1 = secondary, etc.
     */
    mavlink_channel_t           chan;
    
    /**
     * @brief Get packet overhead for this channel
     * @return Overhead bytes (8 for MAVLink1, 12+ for MAVLink2)
     * @note Convenience wrapper around packet_overhead_chan(chan)
     */
    uint8_t packet_overhead(void) const { return packet_overhead_chan(chan); }

    /**
     * @brief Stream rate parameters for this channel
     * @note Array indexed by streams enum (STREAM_RAW_SENSORS, etc.)
     * @note Values are Hz * 10 (e.g., 50 = 5Hz, 10 = 1Hz)
     * @note Persisted as SR0_*, SR1_*, SR2_*, SR3_* parameters
     */
    AP_Int16        streamRates[NUM_STREAMS];

    /**
     * @brief Handle HEARTBEAT message from GCS
     * @param msg HEARTBEAT message
     * @note Updates last_heartbeat_time and processes GCS presence
     * @note Tracks GCS system ID for routing and acknowledgment
     */
    void handle_heartbeat(const mavlink_message_t &msg);

    /**
     * @brief Check if stream rates should be persisted
     * @return true if stream rates are saved to parameters
     * @note Default false, some vehicles override to save rates
     */
    virtual bool persist_streamrates() const { return false; }
    
    /**
     * @brief Handle REQUEST_DATA_STREAM message (legacy)
     * @param msg REQUEST_DATA_STREAM message
     * @note Configures stream rates (SR*_* parameters)
     * @note Can be disabled with NOSTREAMOVERRIDE option
     */
    void handle_request_data_stream(const mavlink_message_t &msg);

    /**
     * @brief Channel options parameter
     * @note Bitmask of Option enum values
     */
    AP_Int16 options;
    
    /**
     * @enum Option
     * @brief Channel behavior options
     */
    enum class Option : uint16_t {
        MAVLINK2_SIGNING_DISABLED = (1U << 0), ///< Disable MAVLink2 signing on this channel
        NO_FORWARD                = (1U << 1), ///< Don't forward MAVLink data to/from this channel
        NOSTREAMOVERRIDE          = (1U << 2), ///< Ignore REQUEST_DATA_STREAM messages
    };
    
    /**
     * @brief Check if option is enabled
     * @param option Option to check
     * @return true if option bit is set
     */
    bool option_enabled(Option option) const {
        return options & static_cast<uint16_t>(option);
    }
    
    /**
     * @brief Enable option and save to parameters
     * @param option Option to enable
     */
    void enable_option(Option option) {
        options.set_and_save(static_cast<uint16_t>(options) | static_cast<uint16_t>(option));
    }
    
    /**
     * @brief Disable option and save to parameters
     * @param option Option to disable
     */
    void disable_option(Option option) {
        options.set_and_save(static_cast<uint16_t>(options) & (~ static_cast<uint16_t>(option)));
    }
    
    /**
     * @brief Migration flag for options parameter format conversion
     * @note Used during parameter system upgrades
     */
    AP_Int8 options_were_converted;

    /**
     * @brief Handle COMMAND_ACK message from GCS/companion
     * @param msg COMMAND_ACK message
     * @note Processes acknowledgment of commands sent by this vehicle
     * @note Vehicle subclasses may override for custom ACK handling
     */
    virtual void handle_command_ack(const mavlink_message_t &msg);
    
    /**
     * @brief Handle SET_MODE message (deprecated)
     * @param msg SET_MODE message
     * @note Legacy message, use MAV_CMD_DO_SET_MODE command instead
     * @note Sets vehicle base_mode and custom_mode
     */
    void handle_set_mode(const mavlink_message_t &msg);
    
    /**
     * @brief Handle COMMAND_INT message dispatcher
     * @param msg COMMAND_INT message containing MAV_CMD
     * @note Extracts command parameters and calls handle_command_int_packet()
     * @note COMMAND_INT preferred over COMMAND_LONG for position commands (precise lat/lon)
     */
    void handle_command_int(const mavlink_message_t &msg);

    /**
     * @brief Handle MAV_CMD_DO_FOLLOW command
     * @param packet COMMAND_INT parameters
     * @param msg Original COMMAND_INT message
     * @return MAV_RESULT_ACCEPTED on success, error code otherwise
     * @note Enables follow mode for target vehicle
     * @note Requires AP_Follow library
     */
    MAV_RESULT handle_command_do_follow(const mavlink_command_int_t &packet, const mavlink_message_t &msg);
    
    /**
     * @brief Virtual dispatcher for COMMAND_INT commands
     * @param packet COMMAND_INT parameters (includes lat/lon as integers)
     * @param msg Original message for metadata
     * @return MAV_RESULT_* code
     * @note Vehicle subclasses override to handle vehicle-specific commands
     * @note Common commands handled in base class implementation
     */
    virtual MAV_RESULT handle_command_int_packet(const mavlink_command_int_t &packet, const mavlink_message_t &msg);
    
    /**
     * @brief Handle MAV_CMD_EXTERNAL_POSITION_ESTIMATE command
     * @param packet COMMAND_INT parameters containing position estimate
     * @return MAV_RESULT_ACCEPTED on success
     * @note Provides external position from vision/mocap systems
     * @note Integrates with EKF for position fusion
     */
    MAV_RESULT handle_command_int_external_position_estimate(const mavlink_command_int_t &packet);
    
    /**
     * @brief Handle MAV_CMD_EXTERNAL_WIND_ESTIMATE command
     * @param packet COMMAND_INT parameters containing wind vector
     * @return MAV_RESULT_ACCEPTED on success
     * @note Provides external wind speed/direction estimate
     * @note Used by airspeed estimation and wind compensation
     */
    MAV_RESULT handle_command_int_external_wind_estimate(const mavlink_command_int_t &packet);

#if AP_HOME_ENABLED
    /**
     * @brief Handle MAV_CMD_DO_SET_HOME command
     * @param packet COMMAND_INT parameters
     * @return MAV_RESULT_ACCEPTED on success
     * @note param1=1: use current location, param1=0: use specified lat/lon/alt
     * @warning Changes home position used for RTL and failsafe
     */
    MAV_RESULT handle_command_do_set_home(const mavlink_command_int_t &packet);
    
    /**
     * @brief Set home to current vehicle position
     * @param lock true to lock home position (prevent changes)
     * @return true if successful
     * @note Requires valid position estimate from EKF
     */
    bool set_home_to_current_location(bool lock);
    
    /**
     * @brief Set home to specified location
     * @param loc Location to set as home
     * @param lock true to lock home position
     * @return true if successful
     * @warning Changes failsafe behavior and RTL destination
     */
    bool set_home(const Location& loc, bool lock);
#endif

#if AP_ARMING_ENABLED
    /**
     * @brief Handle MAV_CMD_COMPONENT_ARM_DISARM command
     * @param packet COMMAND_INT parameters (param1: 1=arm, 0=disarm, param2: force flag)
     * @return MAV_RESULT_ACCEPTED if arm/disarm successful
     * @warning SAFETY CRITICAL - affects motor arming state
     * @note Checks arming preconditions via AP_Arming
     * @note param2=21196 forces disarm without checks (emergency use)
     */
    virtual MAV_RESULT handle_command_component_arm_disarm(const mavlink_command_int_t &packet);
#endif
    
    /**
     * @brief Handle MAV_CMD_DO_AUX_FUNCTION command
     * @param packet COMMAND_INT parameters (param1: function ID, param2: value)
     * @return MAV_RESULT_ACCEPTED on success
     * @note Triggers auxiliary functions by ID (e.g., camera trigger, RC overrides)
     * @see RC_Channel::AUX_FUNC for function ID enumeration
     */
    MAV_RESULT handle_command_do_aux_function(const mavlink_command_int_t &packet);
    
    /**
     * @brief Handle MAV_CMD_STORAGE_FORMAT command
     * @param packet COMMAND_INT parameters (param1: storage ID, param2: format type)
     * @param msg Original message for long-running task tracking
     * @return MAV_RESULT_IN_PROGRESS while formatting, MAV_RESULT_ACCEPTED on completion
     * @warning ERASES SD card or flash storage - all data lost
     * @note Uses GCS_MAVLINK_InProgress for task tracking
     */
    MAV_RESULT handle_command_storage_format(const mavlink_command_int_t &packet, const mavlink_message_t &msg);
    /**
     * @brief Handle MISSION_REQUEST_LIST message
     * @param msg MISSION_REQUEST_LIST message
     * @note GCS requesting download of all mission items
     * @note Sends MISSION_COUNT with total number of items
     */
    void handle_mission_request_list(const mavlink_message_t &msg);
    
#if AP_MAVLINK_MSG_MISSION_REQUEST_ENABLED
    /**
     * @brief Handle MISSION_REQUEST message (legacy)
     * @param msg MISSION_REQUEST message
     * @note GCS requesting specific mission item (deprecated)
     * @note Use MISSION_REQUEST_INT instead for better precision
     */
    void handle_mission_request(const mavlink_message_t &msg);
#endif
    
    /**
     * @brief Handle MISSION_REQUEST_INT message
     * @param msg MISSION_REQUEST_INT message with item sequence number
     * @note GCS requesting specific mission item with integer coordinates
     * @note Sends MISSION_ITEM_INT in response
     */
    void handle_mission_request_int(const mavlink_message_t &msg);
    
    /**
     * @brief Handle MISSION_CLEAR_ALL message
     * @param msg MISSION_CLEAR_ALL message
     * @note Deletes all mission items of specified type (mission/fence/rally)
     * @note Sends MISSION_ACK with result
     */
    void handle_mission_clear_all(const mavlink_message_t &msg);

#if AP_MAVLINK_MISSION_SET_CURRENT_ENABLED
    /**
     * @brief Handle MISSION_SET_CURRENT message (deprecated)
     * @param mission Mission object to update
     * @param msg MISSION_SET_CURRENT message with sequence number
     * @note Sets current mission item index (deprecated)
     * @note Use MAV_CMD_DO_SET_MISSION_CURRENT command instead (provides ACK)
     * @note GCS relies on receiving back identical sequence number as confirmation
     */
    virtual void handle_mission_set_current(AP_Mission &mission, const mavlink_message_t &msg);
#endif

    /**
     * @brief Handle MISSION_COUNT message
     * @param msg MISSION_COUNT message with total item count
     * @note GCS starting mission upload with specified number of items
     * @note Initiates mission item receive sequence
     */
    void handle_mission_count(const mavlink_message_t &msg);
    
    /**
     * @brief Handle MISSION_WRITE_PARTIAL_LIST message
     * @param msg MISSION_WRITE_PARTIAL_LIST with start/end indices
     * @note GCS uploading partial mission list
     * @note Used for updating specific mission items without full upload
     */
    void handle_mission_write_partial_list(const mavlink_message_t &msg);
    
    /**
     * @brief Handle MISSION_ITEM message
     * @param msg MISSION_ITEM message containing mission command
     * @note GCS sending single mission item (deprecated, use MISSION_ITEM_INT)
     * @note Stores item and requests next via MISSION_REQUEST
     */
    void handle_mission_item(const mavlink_message_t &msg);

    /**
     * @brief Handle DISTANCE_SENSOR message
     * @param msg DISTANCE_SENSOR message from external rangefinder
     * @note Integrates external rangefinder data into AP_RangeFinder
     * @note Used for companion computer rangefinders
     */
    void handle_distance_sensor(const mavlink_message_t &msg);
    
    /**
     * @brief Handle OBSTACLE_DISTANCE message
     * @param msg OBSTACLE_DISTANCE message with proximity data
     * @note Provides 360-degree obstacle distance array
     * @note Integrates with AP_Proximity for avoidance
     */
    void handle_obstacle_distance(const mavlink_message_t &msg);
    
    /**
     * @brief Handle OBSTACLE_DISTANCE_3D message
     * @param msg OBSTACLE_DISTANCE_3D message with 3D obstacle points
     * @note Provides individual 3D obstacle positions
     * @note More flexible than OBSTACLE_DISTANCE for complex environments
     */
    void handle_obstacle_distance_3d(const mavlink_message_t &msg);

    /**
     * @brief Handle ADSB_VEHICLE message
     * @param msg ADSB_VEHICLE message with aircraft data
     * @note Processes ADS-B traffic from external receiver
     * @note Integrates with AP_ADSB for collision avoidance
     */
    void handle_adsb_message(const mavlink_message_t &msg);

    /**
     * @brief Handle OSD_PARAM_CONFIG message
     * @param msg OSD_PARAM_CONFIG message for OSD parameter setup
     * @note Configures on-screen display parameters
     * @note Used by MSP OSD and MAVLink OSD devices
     */
    void handle_osd_param_config(const mavlink_message_t &msg) const;

    /**
     * @brief Dispatcher for parameter-related messages
     * @param msg Parameter message (PARAM_SET, PARAM_REQUEST_LIST, PARAM_REQUEST_READ)
     * @note Routes to specific parameter handler based on message ID
     */
    void handle_common_param_message(const mavlink_message_t &msg);
    
    /**
     * @brief Handle PARAM_SET message
     * @param msg PARAM_SET message with parameter name and value
     * @note Sets parameter value if allowed (see get_allow_param_set())
     * @note Sends PARAM_VALUE with result
     * @warning Parameter changes can affect flight behavior
     */
    void handle_param_set(const mavlink_message_t &msg);
    
    /**
     * @brief Handle PARAM_REQUEST_LIST message
     * @param msg PARAM_REQUEST_LIST message
     * @note GCS requesting download of all parameters
     * @note Queues all parameters for transmission
     */
    void handle_param_request_list(const mavlink_message_t &msg);
    
    /**
     * @brief Handle PARAM_REQUEST_READ message
     * @param msg PARAM_REQUEST_READ with parameter name or index
     * @note GCS requesting single parameter value
     * @note Sends PARAM_VALUE in response
     */
    void handle_param_request_read(const mavlink_message_t &msg);
    
    /**
     * @brief Check if parameters are ready for access
     * @return true if parameter system initialized
     * @note Some vehicles override to delay parameter access during boot
     */
    virtual bool params_ready() const { return true; }
    /**
     * @brief Handle RC_CHANNELS_OVERRIDE message
     * @param msg RC_CHANNELS_OVERRIDE with RC input overrides
     * @note Allows GCS to override RC inputs for manual control
     * @note Timeout after no override messages received
     * @warning Overrides pilot RC inputs - use with caution
     */
    void handle_rc_channels_override(const mavlink_message_t &msg);
    
    /**
     * @brief Handle SYSTEM_TIME message
     * @param msg SYSTEM_TIME message with Unix time and boot time
     * @note Synchronizes system time with GCS/GPS time
     * @note Used for time-based mission commands
     */
    void handle_system_time_message(const mavlink_message_t &msg);
    
    /**
     * @brief Dispatcher for rally point messages
     * @param msg Rally point message (RALLY_FETCH_POINT, RALLY_POINT)
     * @note Routes to specific rally handler based on message ID
     */
    void handle_common_rally_message(const mavlink_message_t &msg);
    
    /**
     * @brief Handle RALLY_FETCH_POINT message
     * @param msg RALLY_FETCH_POINT with rally point index
     * @note GCS requesting specific rally point
     * @note Sends RALLY_POINT in response
     */
    void handle_rally_fetch_point(const mavlink_message_t &msg);
    
    /**
     * @brief Handle RALLY_POINT message
     * @param msg RALLY_POINT message with rally point data
     * @note GCS uploading rally point
     * @note Stores rally point in AP_Rally
     */
    void handle_rally_point(const mavlink_message_t &msg) const;
    
#if HAL_MOUNT_ENABLED
    /**
     * @brief Handle gimbal/mount control messages
     * @param msg Mount message (MOUNT_CONFIGURE, MOUNT_CONTROL, etc.)
     * @note Dispatcher for all mount-related MAVLink messages
     * @note Requires AP_Mount library
     * @see AP_Mount for supported message types
     */
    virtual void handle_mount_message(const mavlink_message_t &msg);
#endif
    
    /**
     * @brief Dispatcher for geofence messages
     * @param msg Fence message (FENCE_FETCH_POINT, FENCE_POINT)
     * @note Routes to appropriate fence handler
     * @note Manages AC_Fence polygon and circular fence points
     */
    void handle_fence_message(const mavlink_message_t &msg);
    
    /**
     * @brief Handle PARAM_VALUE message
     * @param msg PARAM_VALUE from another MAVLink system
     * @note Processes parameter values from companion computers
     * @note Used for parameter synchronization
     */
    void handle_param_value(const mavlink_message_t &msg);
    
#if HAL_LOGGING_ENABLED
    /**
     * @brief Get radio status logging bit for this vehicle
     * @return Bitmask bit for radio status logging
     * @note Vehicle subclasses override to specify logging bit
     */
    virtual uint32_t log_radio_bit() const { return 0; }
#endif
    
    /**
     * @brief Handle RADIO_STATUS message
     * @param msg RADIO_STATUS with telemetry radio statistics
     * @note Updates link quality metrics (RSSI, noise, txbuf)
     * @note Used for flow control and link quality reporting
     */
    void handle_radio_status(const mavlink_message_t &msg);
    
    /**
     * @brief Handle SERIAL_CONTROL message
     * @param msg SERIAL_CONTROL for UART passthrough
     * @note Allows GCS to access serial ports (GPS, gimbal, etc.)
     * @note Locks MAVLink channel during passthrough session
     * @warning Blocks normal MAVLink communication on this channel
     */
    void handle_serial_control(const mavlink_message_t &msg);
    
    /**
     * @brief Handle VISION_POSITION_DELTA message
     * @param msg VISION_POSITION_DELTA with incremental position
     * @note Provides vision-based position deltas for dead reckoning
     * @note Integrates with EKF for visual odometry
     */
    void handle_vision_position_delta(const mavlink_message_t &msg);

    /**
     * @brief Virtual dispatcher for all incoming MAVLink messages
     * @param msg Received MAVLink message
     * @note Vehicle subclasses override to add custom message handling
     * @note Called after common message handling
     * @see packetReceived for initial message dispatch
     */
    virtual void handle_message(const mavlink_message_t &msg);
    
#if AP_MAVLINK_SET_GPS_GLOBAL_ORIGIN_MESSAGE_ENABLED
    /**
     * @brief Handle SET_GPS_GLOBAL_ORIGIN message
     * @param msg SET_GPS_GLOBAL_ORIGIN with origin coordinates
     * @note Sets global coordinate system origin
     * @note Used for local/global coordinate transformations
     */
    void handle_set_gps_global_origin(const mavlink_message_t &msg);
#endif  // AP_MAVLINK_SET_GPS_GLOBAL_ORIGIN_MESSAGE_ENABLED
    
    /**
     * @brief Handle SETUP_SIGNING message
     * @param msg SETUP_SIGNING with signing keys
     * @note Configures MAVLink2 message signing
     * @note Establishes secure communication with GCS
     * @warning Changes signature keys - security sensitive
     */
    void handle_setup_signing(const mavlink_message_t &msg) const;
    
    /**
     * @brief Handle MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN
     * @param packet Command parameters
     * @param msg Original COMMAND_INT message
     * @return MAV_RESULT_ACCEPTED if reboot initiated
     * @note Reboots autopilot or companion computer
     * @warning Causes immediate system restart
     */
    virtual MAV_RESULT handle_preflight_reboot(const mavlink_command_int_t &packet, const mavlink_message_t &msg);
    
#if AP_MAVLINK_FAILURE_CREATION_ENABLED
    /**
     * @brief Semaphore for testing deadlock scenarios
     * @note Only available with AP_MAVLINK_FAILURE_CREATION_ENABLED
     * @warning Testing infrastructure - not for production use
     */
    struct {
        HAL_Semaphore sem;
        bool taken;
    } _deadlock_sem;
    
    /**
     * @brief Create deadlock for testing
     * @note Testing infrastructure - causes intentional deadlock
     * @warning Only for development/testing
     */
    void deadlock_sem(void);
#endif

    /**
     * @brief Handle MAV_CMD_DO_SET_SAFETY_SWITCH_STATE
     * @param packet Command parameters (1.0=disarm safety, 0.0=arm safety)
     * @param msg Original COMMAND_INT message
     * @return MAV_RESULT_ACCEPTED if safety state changed
     * @warning Bypasses hardware safety switch - safety critical
     */
    MAV_RESULT handle_do_set_safety_switch_state(const mavlink_command_int_t &packet, const mavlink_message_t &msg);

    /**
     * @brief Handle MAV_CMD_SET_MESSAGE_INTERVAL
     * @param packet Command parameters with message ID and interval
     * @return MAV_RESULT_ACCEPTED if interval set successfully
     * @note Sets message streaming interval in microseconds
     * @note interval_us=0 disables message, -1 uses default rate
     */
    MAV_RESULT handle_command_set_message_interval(const mavlink_command_int_t &packet);
    
    /**
     * @brief Handle MAV_CMD_GET_MESSAGE_INTERVAL
     * @param packet Command parameters with message ID
     * @return MAV_RESULT_ACCEPTED if interval retrieved
     * @note Returns current message interval via MESSAGE_INTERVAL response
     */
    MAV_RESULT handle_command_get_message_interval(const mavlink_command_int_t &packet);
    
    /**
     * @brief Get current interval for ap_message
     * @param id ap_message identifier
     * @param[out] interval_ms Interval in milliseconds
     * @return true if interval found
     * @note Helper for get_message_interval implementation
     */
    bool get_ap_message_interval(ap_message id, uint16_t &interval_ms) const;
    
    /**
     * @brief Handle MAV_CMD_REQUEST_MESSAGE
     * @param packet Command parameters with message ID
     * @return MAV_RESULT_ACCEPTED if message sent
     * @note Requests single instance of specific message
     * @note Message sent immediately if possible
     */
    MAV_RESULT handle_command_request_message(const mavlink_command_int_t &packet);

    /**
     * @brief Handle MAV_CMD_START_RX_PAIR
     * @param packet Command parameters for RC receiver binding
     * @return MAV_RESULT_ACCEPTED if binding initiated
     * @note Initiates RC receiver bind/pair mode
     * @note Requires supported RC protocol (CRSF, ELRS, etc.)
     */
    MAV_RESULT handle_START_RX_PAIR(const mavlink_command_int_t &packet);

    /**
     * @brief Handle MAV_CMD_DO_FLIGHTTERMINATION
     * @param packet Command parameters (1.0=terminate, 0.0=resume)
     * @return MAV_RESULT_ACCEPTED if termination state changed
     * @warning SAFETY CRITICAL - immediately disarms motors
     * @note Triggers emergency termination procedures
     */
    virtual MAV_RESULT handle_flight_termination(const mavlink_command_int_t &packet);

#if AP_MAVLINK_AUTOPILOT_VERSION_REQUEST_ENABLED
    /**
     * @brief Handle REQUEST_AUTOPILOT_VERSION message
     * @param msg REQUEST_AUTOPILOT_VERSION message
     * @note Sends AUTOPILOT_VERSION with capabilities
     * @note Legacy message, use MAV_CMD_REQUEST_MESSAGE instead
     */
    void handle_send_autopilot_version(const mavlink_message_t &msg);
#endif

#if AP_MAVLINK_MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES_ENABLED
    /**
     * @brief Handle MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES
     * @param packet Command parameters
     * @return MAV_RESULT_ACCEPTED and sends AUTOPILOT_VERSION
     * @note Legacy command, use MAV_CMD_REQUEST_MESSAGE instead
     */
    MAV_RESULT handle_command_request_autopilot_capabilities(const mavlink_command_int_t &packet);
#endif

    /**
     * @brief Send startup banner text to GCS
     * @note Vehicle subclasses override to send custom banner
     * @note Includes firmware version and board type
     */
    virtual void send_banner();

    /**
     * @brief Send deprecation warning for received message
     * @param message Description of deprecated message/feature
     * @note Rate-limited to avoid spam
     * @note Warns users about legacy message usage
     */
    void send_received_message_deprecation_warning(const char *message);

    /**
     * @brief Handle DEVICE_OP_READ message
     * @param msg DEVICE_OP_READ for device register read
     * @note Reads I2C/SPI device registers for debugging
     * @note Responds with DEVICE_OP_READ_REPLY
     */
    void handle_device_op_read(const mavlink_message_t &msg);
    
    /**
     * @brief Handle DEVICE_OP_WRITE message
     * @param msg DEVICE_OP_WRITE for device register write
     * @note Writes I2C/SPI device registers for debugging
     * @note Responds with DEVICE_OP_WRITE_REPLY
     * @warning Can modify sensor/peripheral behavior - use with caution
     */
    void handle_device_op_write(const mavlink_message_t &msg);

    /**
     * @brief Send TIMESYNC message for time synchronization
     * @note Sends periodic TIMESYNC requests to GCS
     * @note Used to estimate round-trip time and clock offset
     */
    void send_timesync();
    
    /**
     * @brief Get timestamp when TIMESYNC was likely received
     * @return Receive timestamp in nanoseconds
     * @note Accounts for processing delays
     */
    uint64_t timesync_receive_timestamp_ns() const;
    
    /**
     * @brief Get current timestamp for TIMESYNC ts1 field
     * @return Current system time in nanoseconds
     * @note Monotonic timestamp suitable for synchronization
     */
    uint64_t timesync_timestamp_ns() const;
    
    /**
     * @brief Handle TIMESYNC message
     * @param msg TIMESYNC message from GCS
     * @note Processes time synchronization response
     * @note Updates time offset estimation
     */
    void handle_timesync(const mavlink_message_t &msg);
    
    /**
     * @brief TIMESYNC request state
     * @note Tracks periodic TIMESYNC request timing
     */
    struct {
        int64_t sent_ts1;           ///< Last sent ts1 value
        uint32_t last_sent_ms;      ///< Time of last TIMESYNC send
        const uint16_t interval_ms = 10000; ///< TIMESYNC interval (10 seconds)
    }  _timesync_request;

    /**
     * @brief Handle STATUSTEXT message from other systems
     * @param msg STATUSTEXT message
     * @note Processes status text from companion computers
     * @note Can be logged or re-broadcast
     */
    void handle_statustext(const mavlink_message_t &msg) const;
    
    /**
     * @brief Handle NAMED_VALUE_FLOAT or NAMED_VALUE_INT
     * @param msg NAMED_VALUE message
     * @note Processes named debug values from other systems
     * @note Used for telemetry and debugging
     */
    void handle_named_value(const mavlink_message_t &msg) const;

    /**
     * @brief Check if telemetry is intentionally delayed
     * @return true if in delayed start mode
     * @note Used during system initialization
     */
    bool telemetry_delayed() const;

    /**
     * @brief Handle MAV_CMD_RUN_PREARM_CHECKS
     * @param packet Command parameters
     * @return MAV_RESULT_ACCEPTED if checks pass, MAV_RESULT_FAILED otherwise
     * @note Manually triggers pre-arm safety checks
     * @note Results sent via STATUSTEXT messages
     */
    MAV_RESULT handle_command_run_prearm_checks(const mavlink_command_int_t &packet);
    
    /**
     * @brief Handle MAV_CMD_FLASH_BOOTLOADER
     * @param packet Command parameters with bootloader path
     * @return MAV_RESULT_ACCEPTED if flash initiated
     * @note Updates bootloader firmware
     * @warning System enters bootloader mode - requires reset
     */
    MAV_RESULT handle_command_flash_bootloader(const mavlink_command_int_t &packet);

    /**
     * @brief Handle MAV_CMD_PREFLIGHT_CALIBRATION
     * @param packet Command parameters specifying calibration type
     * @param msg Original COMMAND_INT message
     * @return MAV_RESULT_ACCEPTED, IN_PROGRESS, or error code
     * @note Vehicle wrapper for calibration dispatcher
     * @note Plane overrides to prevent failsafe during calibration
     * @warning Vehicle must be stationary during calibration
     */
    virtual MAV_RESULT handle_command_preflight_calibration(const mavlink_command_int_t &packet, const mavlink_message_t &msg);

    /**
     * @brief Virtual calibration dispatcher
     * @param packet Command parameters (gyro, mag, accel, pressure, etc.)
     * @param msg Original COMMAND_INT message
     * @return MAV_RESULT_* code indicating calibration result
     * @note Vehicle subclasses override to add sensor calibration support
     * @note Parameters: param1=gyro, param2=mag, param3=pressure, param4=radio, param5=accel, param6=compass_mot, param7=esc
     */
    virtual MAV_RESULT _handle_command_preflight_calibration(const mavlink_command_int_t &packet, const mavlink_message_t &msg);
    
    /**
     * @brief Handle barometer calibration
     * @param msg Original message for progress reporting
     * @return MAV_RESULT_ACCEPTED if calibration successful
     * @note Sets ground pressure reference
     * @note Requires stable temperature and altitude
     */
    virtual MAV_RESULT _handle_command_preflight_calibration_baro(const mavlink_message_t &msg);

#if AP_MISSION_ENABLED
    /**
     * @brief Handle MAV_CMD_DO_SET_MISSION_CURRENT
     * @param packet Command parameters with mission sequence number
     * @return MAV_RESULT_ACCEPTED if mission index changed
     * @note Jumps to specified mission item
     * @note Sends MISSION_CURRENT confirmation
     */
    virtual MAV_RESULT handle_command_do_set_mission_current(const mavlink_command_int_t &packet);
    
    /**
     * @brief Handle MAV_CMD_DO_JUMP_TAG
     * @param packet Command parameters with tag to jump to
     * @return MAV_RESULT_ACCEPTED if tag found and jump successful
     * @note Jumps to mission item with matching tag
     * @note More flexible than DO_SET_MISSION_CURRENT
     */
    MAV_RESULT handle_command_do_jump_tag(const mavlink_command_int_t &packet);
#endif

    /**
     * @brief Handle MAV_CMD_BATTERY_RESET
     * @param packet Command parameters with battery instance and reset type
     * @return MAV_RESULT_ACCEPTED if battery reset successful
     * @note Resets battery capacity estimation
     * @note Used after battery replacement or capacity recalibration
     */
    MAV_RESULT handle_command_battery_reset(const mavlink_command_int_t &packet);
    
    /**
     * @brief Handle COMMAND_LONG message (legacy)
     * @param msg COMMAND_LONG message
     * @note Converts COMMAND_LONG to COMMAND_INT internally
     * @note COMMAND_INT is preferred for location-based commands
     */
    void handle_command_long(const mavlink_message_t &msg);
    
    /**
     * @brief Handle MAV_CMD_ACCELCAL_VEHICLE_POS
     * @param packet Command parameters with vehicle orientation
     * @return MAV_RESULT_ACCEPTED if position recorded
     * @note Part of multi-step accelerometer calibration
     * @note Vehicle must be held in specified orientation
     */
    MAV_RESULT handle_command_accelcal_vehicle_pos(const mavlink_command_int_t &packet);

#if HAL_MOUNT_ENABLED
    /**
     * @brief Handle gimbal/mount MAV_CMD commands
     * @param packet Command parameters for mount control
     * @param msg Original COMMAND_INT message
     * @return MAV_RESULT_* code
     * @note Handles MAV_CMD_DO_MOUNT_CONTROL, MAV_CMD_DO_MOUNT_CONFIGURE, etc.
     * @note Requires AP_Mount library
     * @see AP_Mount for supported mount types and commands
     */
    virtual MAV_RESULT handle_command_mount(const mavlink_command_int_t &packet, const mavlink_message_t &msg);
#endif

    /**
     * @brief Handle MAV_CMD_DO_START_MAG_CAL or MAV_CMD_DO_ACCEPT_MAG_CAL
     * @param packet Command parameters (autosave, retry, etc.)
     * @return MAV_RESULT_ACCEPTED if calibration started/accepted
     * @note Starts compass calibration or accepts calibration results
     * @note Requires vehicle rotation during calibration
     */
    MAV_RESULT handle_command_mag_cal(const mavlink_command_int_t &packet);
    
    /**
     * @brief Handle MAV_CMD_FIXED_MAG_CAL_YAW
     * @param packet Command parameters with known yaw angle
     * @return MAV_RESULT_ACCEPTED if calibration set
     * @note Sets compass offsets using known heading
     * @note Alternative to rotation-based calibration
     */
    MAV_RESULT handle_command_fixed_mag_cal_yaw(const mavlink_command_int_t &packet);

    /**
     * @brief Handle camera control commands
     * @param packet Camera command parameters
     * @return MAV_RESULT_* code
     * @note Handles MAV_CMD_DO_DIGICAM_CONFIGURE, MAV_CMD_DO_DIGICAM_CONTROL, etc.
     * @note Requires AP_Camera library
     */
    MAV_RESULT handle_command_camera(const mavlink_command_int_t &packet);
    
    /**
     * @brief Handle MAV_CMD_DO_SET_ROI (Region of Interest)
     * @param packet Command with ROI location or mode
     * @return MAV_RESULT_ACCEPTED if ROI set
     * @note Points camera/mount at specified location
     * @note Calls virtual overload with Location parameter
     */
    MAV_RESULT handle_command_do_set_roi(const mavlink_command_int_t &packet);
    
    /**
     * @brief Virtual handler for ROI with Location
     * @param roi_loc Target location for camera/mount
     * @return MAV_RESULT_* code
     * @note Vehicle subclasses override for ROI behavior
     */
    virtual MAV_RESULT handle_command_do_set_roi(const Location &roi_loc);
    
    /**
     * @brief Handle MAV_CMD_DO_GRIPPER
     * @param packet Command parameters (gripper num, action)
     * @return MAV_RESULT_ACCEPTED if gripper commanded
     * @note Controls servo gripper (grab/release)
     * @note Requires AP_Gripper library
     */
    MAV_RESULT handle_command_do_gripper(const mavlink_command_int_t &packet);
    
    /**
     * @brief Handle MAV_CMD_DO_SPRAYER
     * @param packet Command parameters (enable/disable)
     * @return MAV_RESULT_ACCEPTED if sprayer state changed
     * @note Controls agricultural sprayer
     * @note Requires AP_Sprayer library
     */
    MAV_RESULT handle_command_do_sprayer(const mavlink_command_int_t &packet);
    
    /**
     * @brief Handle MAV_CMD_DO_SET_MODE
     * @param packet Command with custom_mode and base_mode
     * @return MAV_RESULT_ACCEPTED if mode change successful
     * @note Changes vehicle flight mode
     * @note Validates mode availability and arming state
     */
    MAV_RESULT handle_command_do_set_mode(const mavlink_command_int_t &packet);
    
    /**
     * @brief Handle MAV_CMD_GET_HOME_POSITION
     * @param packet Command parameters (unused)
     * @return MAV_RESULT_ACCEPTED and sends HOME_POSITION
     * @note Returns current home location
     */
    MAV_RESULT handle_command_get_home_position(const mavlink_command_int_t &packet);
    
    /**
     * @brief Handle MAV_CMD_DO_FENCE_ENABLE
     * @param packet Command parameters (1=enable, 0=disable)
     * @return MAV_RESULT_ACCEPTED if fence state changed
     * @note Enables or disables geofence
     * @note Requires AC_Fence library
     */
    MAV_RESULT handle_command_do_fence_enable(const mavlink_command_int_t &packet);
    
    /**
     * @brief Handle MAV_CMD_DEBUG_TRAP
     * @param packet Command parameters
     * @return MAV_RESULT_ACCEPTED (triggers trap)
     * @note Development command to trigger debug trap
     * @warning Causes intentional crash for debugging
     */
    MAV_RESULT handle_command_debug_trap(const mavlink_command_int_t &packet);
    
    /**
     * @brief Handle MAV_CMD_SET_EKF_SOURCE_SET
     * @param packet Command with source set number
     * @return MAV_RESULT_ACCEPTED if source set changed
     * @note Switches between EKF sensor source sets
     * @note Used for GPS/non-GPS source switching
     */
    MAV_RESULT handle_command_set_ekf_source_set(const mavlink_command_int_t &packet);
    
    /**
     * @brief Handle MAV_CMD_AIRFRAME_CONFIGURATION
     * @param packet Command with airframe configuration
     * @return MAV_RESULT_ACCEPTED if configuration applied
     * @note Configures vehicle-specific airframe parameters
     * @note Vehicle-dependent implementation
     */
    MAV_RESULT handle_command_airframe_configuration(const mavlink_command_int_t &packet);

    /**
     * @brief CAN frame callback for CAN over MAVLink
     * @param bus CAN bus number
     * @param[in] CANFrame to be forwarded
     * @note Called by CAN driver to forward frames via MAVLink
     * @note Sends CAN_FRAME message to GCS
     */
    void can_frame_callback(uint8_t bus, const AP_HAL::CANFrame &);
    
#if HAL_CANMANAGER_ENABLED
    /**
     * @brief Handle MAV_CMD_CAN_FORWARD
     * @param packet Command parameters (bus, enable/disable)
     * @param msg Original COMMAND_INT message
     * @return MAV_RESULT_ACCEPTED if CAN forwarding configured
     * @note Enables/disables CAN frame forwarding over MAVLink
     * @note Allows remote CAN bus access
     */
    MAV_RESULT handle_can_forward(const mavlink_command_int_t &packet, const mavlink_message_t &msg);
#endif

    /**
     * @brief Handle CAN_FRAME message
     * @param msg CAN_FRAME message with frame data
     * @note Receives CAN frames from GCS and writes to CAN bus
     * @note Used for remote CAN device configuration
     */
    void handle_can_frame(const mavlink_message_t &msg) const;

    /**
     * @brief Handle OPTICAL_FLOW message
     * @param msg OPTICAL_FLOW message from external sensor
     * @note Integrates external optical flow data into EKF
     * @note Provides velocity estimation for GPS-denied navigation
     */
    void handle_optical_flow(const mavlink_message_t &msg);

    /**
     * @brief Handle MANUAL_CONTROL message
     * @param msg MANUAL_CONTROL message with joystick inputs
     * @note Processes joystick/gamepad control inputs
     * @note Calls virtual handle_manual_control_axes for vehicle-specific handling
     * @see handle_manual_control_axes
     */
    void handle_manual_control(const mavlink_message_t &msg);
    
    /**
     * @brief Handle RC_CHANNELS message (radio_rc_channels)
     * @param msg RC_CHANNELS message from companion
     * @note Receives RC inputs via MAVLink instead of hardware receiver
     * @note Used for companion computer RC input injection
     */
    void handle_radio_rc_channels(const mavlink_message_t &msg);

    /**
     * @brief Virtual handler for LANDING_TARGET message
     * @param packet LANDING_TARGET data
     * @param timestamp_ms Local timestamp in milliseconds
     * @note Default implementation is empty (no-op)
     * @note Vehicle subclasses override for precision landing support
     */
    virtual void handle_landing_target(const mavlink_landing_target_t &packet, uint32_t timestamp_ms) { }
    
    /**
     * @brief Vehicle-overridable message send function
     * @param id AP message ID to send
     * @return true if message sent or queued successfully
     * @note Vehicle subclasses override to add vehicle-specific messages
     * @note Called by message scheduler for each stream message
     */
    virtual bool try_send_message(enum ap_message id);
    
    /**
     * @brief Send GLOBAL_POSITION_INT message
     * @note Reports global position, altitude, and velocity
     * @note Virtual to allow vehicle-specific altitude reporting
     */
    virtual void send_global_position_int();

    /**
     * @brief Try to send mission-related messages
     * @param id AP message ID (mission-specific)
     * @return true if message sent successfully
     * @note Handles mission upload/download protocol messages
     * @note Sends MISSION_ITEM_INT, MISSION_REQUEST, MISSION_ACK, etc.
     */
    bool try_send_mission_message(enum ap_message id);
    
    /**
     * @brief Send hardware status message
     * @note Sends HWSTATUS with I2C error count and voltage
     * @note Used for diagnostics
     */
    void send_hwstatus();
    
    /**
     * @brief Handle DATA16/DATA32/DATA64/DATA96 messages
     * @param msg Data packet message
     * @note Generic data transport mechanism
     * @note Used by some companion computer applications
     */
    void handle_data_packet(const mavlink_message_t &msg);

    /**
     * @brief Get absolute altitude for GLOBAL_POSITION_INT
     * @return Altitude in millimeters (AMSL)
     * @note Virtual to allow vehicle-specific altitude source
     * @note Called when sending GLOBAL_POSITION_INT message
     */
    virtual int32_t global_position_int_alt() const;
    
    /**
     * @brief Get relative altitude for GLOBAL_POSITION_INT
     * @return Altitude in millimeters relative to home
     * @note Virtual to allow vehicle-specific relative altitude
     * @note Used in GLOBAL_POSITION_INT message
     */
    virtual int32_t global_position_int_relative_alt() const;

    /**
     * @brief Get climb rate for VFR_HUD
     * @return Climb rate in m/s
     * @note Virtual to allow vehicle-specific climb rate source
     * @note Used in VFR_HUD message
     */
    virtual float vfr_hud_climbrate() const;
    
    /**
     * @brief Get airspeed for VFR_HUD
     * @return Airspeed in m/s
     * @note Virtual to allow vehicle-specific airspeed source
     * @note Used in VFR_HUD message
     */
    virtual float vfr_hud_airspeed() const;
    
    /**
     * @brief Get throttle percentage for VFR_HUD
     * @return Throttle as percentage (0-100)
     * @note Virtual to allow vehicle-specific throttle value
     * @note Default implementation returns 0
     */
    virtual int16_t vfr_hud_throttle() const { return 0; }
    
#if AP_AHRS_ENABLED
    /**
     * @brief Get altitude for VFR_HUD
     * @return Altitude in meters
     * @note Virtual to allow vehicle-specific altitude source
     * @note Used in VFR_HUD message
     */
    virtual float vfr_hud_alt() const;
    
    /**
     * @brief Handle MAV_CMD_DO_SET_GPS_GLOBAL_ORIGIN
     * @param packet Command with global origin location
     * @return MAV_RESULT_ACCEPTED if origin set
     * @note Sets EKF origin to specified lat/lon/alt
     * @note Used for initializing EKF in GPS-denied environments
     */
    MAV_RESULT handle_command_do_set_global_origin(const mavlink_command_int_t &packet);
#endif

#if HAL_HIGH_LATENCY2_ENABLED
    /**
     * @brief Get target altitude for HIGH_LATENCY2
     * @return Target altitude in meters
     * @note Virtual - vehicle subclasses override
     * @note Default returns 0
     */
    virtual int16_t high_latency_target_altitude() const { return 0; }
    
    /**
     * @brief Get target heading for HIGH_LATENCY2
     * @return Target heading in degrees (0-255 scaled)
     * @note Virtual - vehicle subclasses override
     * @note Default returns 0
     */
    virtual uint8_t high_latency_tgt_heading() const { return 0; }
    
    /**
     * @brief Get target distance for HIGH_LATENCY2
     * @return Distance to target in decameters
     * @note Virtual - vehicle subclasses override
     * @note Default returns 0
     */
    virtual uint16_t high_latency_tgt_dist() const { return 0; }
    
    /**
     * @brief Get target airspeed for HIGH_LATENCY2
     * @return Target airspeed in m/s
     * @note Virtual - vehicle subclasses override
     * @note Default returns 0
     */
    virtual uint8_t high_latency_tgt_airspeed() const { return 0; }
    
    /**
     * @brief Get wind speed for HIGH_LATENCY2
     * @return Wind speed in m/s (0-255)
     * @note Virtual - vehicle subclasses override
     * @note Default returns 0
     */
    virtual uint8_t high_latency_wind_speed() const { return 0; }
    
    /**
     * @brief Get wind direction for HIGH_LATENCY2
     * @return Wind direction in degrees (0-255 scaled)
     * @note Virtual - vehicle subclasses override
     * @note Default returns 0
     */
    virtual uint8_t high_latency_wind_direction() const { return 0; }
    
    /**
     * @brief Get air temperature for HIGH_LATENCY2
     * @return Temperature in degrees Celsius
     * @note Used in HIGH_LATENCY2 message for satellite links
     */
    int8_t high_latency_air_temperature() const;

    /**
     * @brief Handle MAV_CMD_CONTROL_HIGH_LATENCY
     * @param packet Command parameters (enable/disable)
     * @return MAV_RESULT_ACCEPTED if high latency mode changed
     * @note Switches between normal and high-latency telemetry modes
     * @note Reduces bandwidth for satellite links
     */
    MAV_RESULT handle_control_high_latency(const mavlink_command_int_t &packet);

#endif // HAL_HIGH_LATENCY2_ENABLED
    
    /**
     * @brief Magic parameter value for force arming via MAV_CMD_COMPONENT_ARM_DISARM
     * @details Special value in param2 that bypasses pre-arm checks
     * @warning SAFETY CRITICAL - Use only in controlled test environments
     * @note Value: 2989.0f
     */
    static constexpr const float magic_force_arm_value = 2989.0f;
    
    /**
     * @brief Magic parameter value for force arm/disarm sequence
     * @details Combined magic value for advanced force operations
     * @warning SAFETY CRITICAL - Bypasses safety checks
     * @note Value: 21196.0f
     */
    static constexpr const float magic_force_arm_disarm_value = 21196.0f;

    /**
     * @brief Apply manual RC override from GCS to RC channel
     * @param[in,out] c RC channel to override
     * @param[in] value_in Input value from GCS (typically PWM range)
     * @param[in] offset PWM offset for normalization
     * @param[in] scaler Scaling factor for value conversion
     * @param[in] tnow Current timestamp in milliseconds
     * @param[in] reversed True if channel should be reversed
     * @note Processes MANUAL_CONTROL or RC_CHANNELS_OVERRIDE messages
     * @note Thread-safe with appropriate channel locking
     */
    void manual_override(class RC_Channel *c, int16_t value_in, uint16_t offset, float scaler, const uint32_t tnow, bool reversed = false);

    /**
     * @brief Get receiver RSSI (Received Signal Strength Indicator)
     * @return RSSI value 0-255 (0=no signal, 255=full signal)
     * @note Returns RC receiver signal strength if available
     * @note Used in RADIO_STATUS and RC_CHANNELS messages
     */
    uint8_t receiver_rssi() const;

    /**
     * @brief Correct offboard timestamp to local system time
     * @param[in] offboard_usec Timestamp from external system in microseconds
     * @param[in] payload_size Size of the message payload for lag correction
     * @return Corrected local timestamp in milliseconds since boot
     * @details Converts external timestamps (e.g., from vision systems) to local time,
     *          accounting for transmission delays and time synchronization offsets.
     * @note Uses JitterCorrection for smoothing timestamp drift
     * @note Payload size helps estimate transmission delay
     */
    uint32_t correct_offboard_timestamp_usec_to_ms(uint64_t offboard_usec, uint16_t payload_size);

#if AP_MAVLINK_COMMAND_LONG_ENABLED
    /**
     * @brief Convert COMMAND_LONG message to COMMAND_INT format
     * @param[in] in Source COMMAND_LONG packet
     * @param[out] out Destination COMMAND_INT packet (fully initialized)
     * @param[in] frame Coordinate frame for location parameters (default: MAV_FRAME_GLOBAL_RELATIVE_ALT)
     * @details COMMAND_INT is preferred over COMMAND_LONG as it provides precise lat/lon as integers
     *          rather than floats, avoiding floating-point precision issues for coordinates.
     * @note If location is not present in command, frame parameter is not used
     * @note Vehicle-specific subclasses may override for custom conversion logic
     */
    virtual void convert_COMMAND_LONG_to_COMMAND_INT(const mavlink_command_long_t &in, mavlink_command_int_t &out, MAV_FRAME frame = MAV_FRAME_GLOBAL_RELATIVE_ALT);
    
    /**
     * @brief Determine appropriate MAVLink frame for COMMAND_LONG packet
     * @param[out] frame Determined MAV_FRAME for the command
     * @param[in] packet_command MAV_CMD command identifier
     * @return true if frame was determined successfully
     * @note Vehicle-specific subclasses may override for command-specific frame logic
     */
    virtual bool mav_frame_for_command_long(MAV_FRAME &fame, MAV_CMD packet_command) const;
    
    /**
     * @brief Attempt to execute COMMAND_LONG by converting to COMMAND_INT
     * @param[in] packet COMMAND_LONG packet to execute
     * @param[in] msg Original MAVLink message
     * @return MAV_RESULT_* status code
     * @details Converts legacy COMMAND_LONG to COMMAND_INT and dispatches to command handler
     * @note Provides backward compatibility for GCS using COMMAND_LONG
     */
    MAV_RESULT try_command_long_as_command_int(const mavlink_command_long_t &packet, const mavlink_message_t &msg);
#endif

    /**
     * @brief Extract Location object from COMMAND_INT parameters
     * @param[in] in COMMAND_INT packet containing location data
     * @param[out] out Extracted Location object
     * @return true if location was successfully extracted
     * @details Converts MAVLink coordinate format (lat/lon as int32_t, alt as float) to ArduPilot Location
     * @note Handles coordinate frame conversion (MAV_FRAME to Location::AltFrame)
     */
    bool location_from_command_t(const mavlink_command_int_t &in, Location &out);

private:

    // define the two objects used for parsing incoming messages:
    mavlink_message_t _channel_buffer;
    mavlink_status_t _channel_status;

    const AP_SerialManager::UARTState *uartstate;

    // last time we got a non-zero RSSI from RADIO_STATUS
    static struct LastRadioStatus {
        uint32_t remrssi_ms;
        uint8_t rssi;
        uint32_t received_ms; // time RADIO_STATUS received
        uint8_t txbuf = 100;
    } last_radio_status;

    enum class Flags {
        USING_SIGNING = (1<<0),
        ACTIVE = (1<<1),
        STREAMING = (1<<2),
        PRIVATE = (1<<3),
        LOCKED = (1<<4),
    };
    void log_mavlink_stats();

    MAV_RESULT _set_mode_common(const uint8_t base_mode, const uint32_t custom_mode);

    // send a (textual) message to the GCS that a received message has
    // been deprecated
    uint32_t last_deprecation_warning_send_time_ms;
    const char *last_deprecation_message;

    // time we last saw traffic from our GCS.  Note that there is an
    // identically named field in GCS:: which is the most recent of
    // each of the GCS_MAVLINK backends
    uint32_t _sysid_gcs_last_seen_time_ms;

    void service_statustext(void);

    MAV_RESULT handle_servorelay_message(const mavlink_command_int_t &packet);
    bool send_relay_status() const;

    static bool command_long_stores_location(const MAV_CMD command);

    bool calibrate_gyros();

    /// The stream we are communicating over
    AP_HAL::UARTDriver *_port;

    /// Perform queued sending operations
    ///
    enum ap_var_type            _queued_parameter_type; ///< type of the next
                                                        // parameter
    AP_Param::ParamToken        _queued_parameter_token; ///AP_Param token for
                                                         // next() call
    uint16_t                    _queued_parameter_index; ///< next queued
                                                         // parameter's index
    uint16_t                    _queued_parameter_count; ///< saved count of
                                                         // parameters for
                                                         // queued send
    uint32_t                    _queued_parameter_send_time_ms;

    // number of extra ms to add to slow things down for the radio
    uint16_t         stream_slowdown_ms;

    // outbound ("deferred message") queue.

    // "special" messages such as heartbeat, next_param etc are stored
    // separately to stream-rated messages like AHRS2 etc.  If these
    // were to be stored in buckets then they would be slowed down
    // based on stream_slowdown, which we have not traditionally done.
    struct deferred_message_t {
        const ap_message id;
        uint16_t interval_ms;
        uint16_t last_sent_ms; // from AP_HAL::millis16()
    } deferred_message[3] = {
        { MSG_HEARTBEAT, },
        { MSG_NEXT_PARAM, },
#if HAL_HIGH_LATENCY2_ENABLED
        { MSG_HIGH_LATENCY2, },
#endif
    };
    // returns index of id in deferred_message[] or -1 if not present
    int8_t get_deferred_message_index(const ap_message id) const;
    // returns index of a message in deferred_message[] which should
    // be sent (or -1 if none to send at the moment)
    int8_t deferred_message_to_send_index(uint16_t now16_ms);
    // cache of which deferred message should be sent next:
    int8_t next_deferred_message_to_send_cache = -1;

    struct deferred_message_bucket_t {
        Bitmask<MSG_LAST> ap_message_ids;
        uint16_t interval_ms;
        uint16_t last_sent_ms; // from AP_HAL::millis16()
    };
    deferred_message_bucket_t deferred_message_bucket[10];
    static const uint8_t no_bucket_to_send = -1;
    static const ap_message no_message_to_send = (ap_message)-1;
    uint8_t sending_bucket_id = no_bucket_to_send;
    Bitmask<MSG_LAST> bucket_message_ids_to_send;

    ap_message next_deferred_bucket_message_to_send(uint16_t now16_ms);
    void find_next_bucket_to_send(uint16_t now16_ms);
    void remove_message_from_bucket(int8_t bucket, ap_message id);

    // bitmask of IDs the code has spontaneously decided it wants to
    // send out.  Examples include HEARTBEAT (gcs_send_heartbeat)
    Bitmask<MSG_LAST> pushed_ap_message_ids;

    // returns true if it is OK to send a message while we are in
    // delay callback.  In particular, when we are doing sensor init
    // we still send heartbeats.
    bool should_send_message_in_delay_callback(const ap_message id) const;

    // if true is returned, interval will contain the default interval for id
    bool get_default_interval_for_ap_message(const ap_message id, uint16_t &interval) const;
    //  if true is returned, interval will contain the default interval for id
    // returns an interval in milliseconds for any ap_message in stream id
    uint16_t get_interval_for_stream(GCS_MAVLINK::streams id) const;
    // set an inverval for a specific mavlink message.  Returns false
    // on failure (typically because there is no mapping from that
    // mavlink ID to an ap_message)
    bool set_mavlink_message_id_interval(const uint32_t mavlink_id,
                                         const uint16_t interval_ms);
    // map a mavlink ID to an ap_message which, if passed to
    // try_send_message, will cause a mavlink message with that id to
    // be emitted.  Returns MSG_LAST if no such mapping exists.
    ap_message mavlink_id_to_ap_message_id(const uint32_t mavlink_id) const;
    // set the interval at which an ap_message should be emitted (in ms)
    bool set_ap_message_interval(enum ap_message id, uint16_t interval_ms);
    // call set_ap_message_interval for each entry in a stream,
    // the interval being based on the stream's rate
    void initialise_message_intervals_for_stream(GCS_MAVLINK::streams id);
    // call initialise_message_intervals_for_stream on every stream:
    void initialise_message_intervals_from_streamrates();
    // boolean that indicated that message intervals have been set
    // from streamrates:
    bool deferred_messages_initialised;
#if HAL_MAVLINK_INTERVALS_FROM_FILES_ENABLED
    // read configuration files from (e.g.) SD and ROMFS, set
    // intervals from same
    void initialise_message_intervals_from_config_files();
    // read file, set message intervals from it:
    void get_intervals_from_filepath(const char *path, DefaultIntervalsFromFiles &);
#endif
    // return interval deferred message bucket should be sent after.
    // When sending parameters and waypoints this may be longer than
    // the interval specified in "deferred"
    uint16_t get_reschedule_interval_ms(const deferred_message_bucket_t &deferred) const;

    bool do_try_send_message(const ap_message id);

    // time when we missed sending a parameter for GCS
    static uint32_t reserve_param_space_start_ms;
    
    // bitmask of what mavlink channels are active
    static uint8_t mavlink_active;

    // bitmask of what mavlink channels are private
    static uint8_t mavlink_private;

    // bitmask of what mavlink channels are streaming
    static uint8_t chan_is_streaming;

    // mavlink routing object
    static MAVLink_routing routing;

    struct pending_param_request {
        mavlink_channel_t chan;
        int16_t param_index;
        char param_name[AP_MAX_NAME_SIZE+1];
    };

    struct pending_param_reply {
        mavlink_channel_t chan;        
        float value;
        enum ap_var_type p_type;
        int16_t param_index;
        uint16_t count;
        char param_name[AP_MAX_NAME_SIZE+1];
    };

    // queue of pending parameter requests and replies
    static ObjectBuffer<pending_param_request> param_requests;
    static ObjectBuffer<pending_param_reply> param_replies;

    // have we registered the IO timer callback?
    static bool param_timer_registered;

    // IO timer callback for parameters
    void param_io_timer(void);

    uint8_t send_parameter_async_replies();

#if AP_MAVLINK_FTP_ENABLED
    enum class FTP_OP : uint8_t {
        None = 0,
        TerminateSession = 1,
        ResetSessions = 2,
        ListDirectory = 3,
        OpenFileRO = 4,
        ReadFile = 5,
        CreateFile = 6,
        WriteFile = 7,
        RemoveFile = 8,
        CreateDirectory = 9,
        RemoveDirectory = 10,
        OpenFileWO = 11,
        TruncateFile = 12,
        Rename = 13,
        CalcFileCRC32 = 14,
        BurstReadFile = 15,
        Ack = 128,
        Nack = 129,
    };

    enum class FTP_ERROR : uint8_t {
        None = 0,
        Fail = 1,
        FailErrno = 2,
        InvalidDataSize = 3,
        InvalidSession = 4,
        NoSessionsAvailable = 5,
        EndOfFile = 6,
        UnknownCommand = 7,
        FileExists = 8,
        FileProtected = 9,
        FileNotFound = 10,
    };

    struct pending_ftp {
        uint32_t offset;
        mavlink_channel_t chan;        
        uint16_t seq_number;
        FTP_OP opcode;
        FTP_OP req_opcode;
        bool  burst_complete;
        uint8_t size;
        uint8_t session;
        uint8_t sysid;
        uint8_t compid;
        uint8_t data[239];
    };

    enum class FTP_FILE_MODE {
        Read,
        Write,
    };

    struct ftp_state {
        ObjectBuffer<pending_ftp> *requests;

        // session specific info, currently only support a single session over all links
        int fd = -1;
        FTP_FILE_MODE mode; // work around AP_Filesystem not supporting file modes
        int16_t current_session;
        uint32_t last_send_ms;
        uint8_t need_banner_send_mask;
    };
    static struct ftp_state ftp;

    static void ftp_error(struct pending_ftp &response, FTP_ERROR error); // FTP helper method for packing a NAK
    static bool ftp_check_name_len(const struct pending_ftp &request);
    static int gen_dir_entry(char *dest, size_t space, const char * path, const struct dirent * entry); // FTP helper for emitting a dir response
    static void ftp_list_dir(struct pending_ftp &request, struct pending_ftp &response);

    bool ftp_init(void);
    void handle_file_transfer_protocol(const mavlink_message_t &msg);
    bool send_ftp_reply(const pending_ftp &reply);
    void ftp_worker(void);
    void ftp_push_replies(pending_ftp &reply);
#endif  // AP_MAVLINK_FTP_ENABLED

    void send_distance_sensor(const class AP_RangeFinder_Backend *sensor, const uint8_t instance) const;

    virtual bool handle_guided_request(AP_Mission::Mission_Command &cmd) { return false; };
    virtual void handle_change_alt_request(Location &location) {};
    void handle_common_mission_message(const mavlink_message_t &msg);

    virtual void handle_manual_control_axes(const mavlink_manual_control_t &packet, const uint32_t tnow) {};

    void handle_vicon_position_estimate(const mavlink_message_t &msg);
    void handle_vision_position_estimate(const mavlink_message_t &msg);
    void handle_global_vision_position_estimate(const mavlink_message_t &msg);
    void handle_att_pos_mocap(const mavlink_message_t &msg);
    void handle_odometry(const mavlink_message_t &msg);
    void handle_common_vision_position_estimate_data(const uint64_t usec,
                                                     const float x,
                                                     const float y,
                                                     const float z,
                                                     const float roll,
                                                     const float pitch,
                                                     const float yaw,
                                                     const float covariance[21],
                                                     const uint8_t reset_counter,
                                                     const uint16_t payload_size);
    void handle_vision_speed_estimate(const mavlink_message_t &msg);
    void handle_landing_target(const mavlink_message_t &msg);
    void handle_generator_message(const mavlink_message_t &msg);

    void lock_channel(const mavlink_channel_t chan, bool lock);

    mavlink_signing_t signing;
    static mavlink_signing_streams_t signing_streams;
    static uint32_t last_signing_save_ms;

    static StorageAccess _signing_storage;
    static bool signing_key_save(const struct SigningKey &key);
    static bool signing_key_load(struct SigningKey &key);
    void load_signing_key(void);
    bool signing_enabled(void) const;
    static void save_signing_timestamp(bool force_save_now);

#if HAL_MAVLINK_INTERVALS_FROM_FILES_ENABLED
    // structure containing default intervals read from files for this
    // link:
    DefaultIntervalsFromFiles *default_intervals_from_files;
#endif

    // alternative protocol handler support
    struct {
        GCS_MAVLINK::protocol_handler_fn_t handler;
        uint32_t last_mavlink_ms;
        uint32_t last_alternate_ms;
        bool active;
    } alternative;

    JitterCorrection lag_correction;
    
    // we cache the current location and send it even if the AHRS has
    // no idea where we are:
    Location global_position_current_loc;

    uint8_t last_tx_seq;
    uint16_t send_packet_count;
    uint16_t out_of_space_to_send_count; // number of times HAVE_PAYLOAD_SPACE and friends have returned false

#if GCS_DEBUG_SEND_MESSAGE_TIMINGS
    struct {
        uint32_t longest_time_us;
        ap_message longest_id;
        uint32_t no_space_for_message;
        uint16_t statustext_last_sent_ms;
        uint32_t behind;
        uint32_t out_of_time;
        uint16_t fnbts_maxtime;
        uint32_t max_retry_deferred_body_us;
        uint8_t max_retry_deferred_body_type;
    } try_send_message_stats;
    uint16_t max_slowdown_ms;
#endif

    uint32_t last_mavlink_stats_logged;

    uint8_t last_battery_status_idx;

    // if we've ever sent a DISTANCE_SENSOR message out of an
    // orientation we continue to send it out, even if it is not
    // longer valid.
    uint8_t proximity_ever_valid_bitmask;

    // true if we should NOT do MAVLink on this port (usually because
    // someone's doing SERIAL_CONTROL over mavlink)
    bool _locked;

    // Handling of AVAILABLE_MODES
    struct {
        bool should_send;
        // Note these start at 1
        uint8_t requested_index;
        uint8_t next_index;
    } available_modes;
    bool send_available_modes();
    bool send_available_mode_monitor();

};

/// @class GCS
/// @brief global GCS object
class GCS
{

public:

    GCS() {
        if (_singleton  == nullptr) {
            _singleton = this;
        } else {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
            // this is a serious problem, but we don't need to kill a
            // real vehicle
            AP_HAL::panic("GCS must be singleton");
#endif
        }

        AP_Param::setup_object_defaults(this, var_info);
    };

    static class GCS *get_singleton() {
        return _singleton;
    }

    static const struct AP_Param::GroupInfo        var_info[];

    virtual uint32_t custom_mode() const = 0;
    virtual MAV_TYPE frame_type() const = 0;
    virtual const char* frame_string() const { return nullptr; }

    struct statustext_t {
        mavlink_statustext_t    msg;
        uint16_t                entry_created_ms;
        uint8_t                 bitmask;
    };
    class StatusTextQueue : public ObjectArray<statustext_t> {
    public:
        using ObjectArray::ObjectArray;
        HAL_Semaphore &semaphore() { return _sem; }
        void prune();
    private:
        // a lock for the statustext queue, to make it safe to use send_text()
        // from multiple threads
        HAL_Semaphore _sem;

        uint32_t last_prune_ms;
    };

    StatusTextQueue &statustext_queue() {
        return _statustext_queue;
    }

    uint8_t sysid_gcs() const { return uint8_t(mav_gcs_sysid); }

    // last time traffic was seen from my designated GCS.  traffic
    // includes heartbeats and some manual control messages.
    uint32_t sysid_mygcs_last_seen_time_ms() const {
        return _sysid_gcs_last_seen_time_ms;
    }
    // called when valid traffic has been seen from our GCS.  This is
    // usually only called from GCS_MAVLINK::sysid_mygcs_seen(..)!
    void sysid_mygcs_seen(uint32_t seen_time_ms) {
        _sysid_gcs_last_seen_time_ms = seen_time_ms;
    }

    void send_to_active_channels(uint32_t msgid, const char *pkt);

    void send_text(MAV_SEVERITY severity, const char *fmt, ...) FMT_PRINTF(3, 4);
    void send_textv(MAV_SEVERITY severity, const char *fmt, va_list arg_list);
    virtual void send_textv(MAV_SEVERITY severity, const char *fmt, va_list arg_list, uint8_t mask);
    uint8_t statustext_send_channel_mask() const;

    virtual GCS_MAVLINK *chan(const uint8_t ofs) = 0;
    virtual const GCS_MAVLINK *chan(const uint8_t ofs) const = 0;
    // return the number of valid GCS objects
    uint8_t num_gcs() const { return _num_gcs; };
    void send_message(enum ap_message id);
    void send_mission_item_reached_message(uint16_t mission_index);
    void send_named_float(const char *name, float value) const;

    void send_parameter_value(const char *param_name,
                              ap_var_type param_type,
                              float param_value);

    // an array of objects used to handle each of the different
    // protocol types we support.  This is indexed by the enumeration
    // MAV_MISSION_TYPE, taking advantage of the fact that fence,
    // mission and rally have values 0, 1 and 2.  Indexing should be via
    // get_prot_for_mission_type to do bounds checking.
    static class MissionItemProtocol *missionitemprotocols[3];
    class MissionItemProtocol *get_prot_for_mission_type(const MAV_MISSION_TYPE mission_type) const;
    void try_send_queued_message_for_type(MAV_MISSION_TYPE type) const;

    void update_send();
    void update_receive();

    // minimum amount of time (in microseconds) that must remain in
    // the main scheduler loop before we are allowed to send any
    // mavlink messages.  We want to prioritise the main flight
    // control loop over communications
    virtual uint16_t min_loop_time_remaining_for_message_send_us() const {
        return 200;
    }

    void init();
    void setup_console();
    void setup_uarts();

    enum class Option {
      GCS_SYSID_ENFORCE = (1U << 0),
    };
    bool option_is_enabled(Option option) const {
        return (mav_options & (uint16_t)option) != 0;
    }

    // returns true if attempts to set parameters via PARAM_SET or via
    // file upload in mavftp should be honoured:
    bool get_allow_param_set() const {
        return allow_param_set;
    }
    // can be used to force sets via PARAM_SET or via mavftp file
    // upload to be ignored by the GCS library:
    void set_allow_param_set(bool new_allowed) {
        allow_param_set = new_allowed;
    }

    bool out_of_time() const;

#if AP_FRSKY_TELEM_ENABLED
    // frsky backend
    class AP_Frsky_Telem *frsky;
#endif

#if AP_LTM_TELEM_ENABLED
    // LTM backend
    AP_LTM_Telem ltm_telemetry;
#endif

#if AP_DEVO_TELEM_ENABLED
    // Devo backend
    AP_DEVO_Telem devo_telemetry;
#endif

    // install an alternative protocol handler
    bool install_alternative_protocol(mavlink_channel_t chan, GCS_MAVLINK::protocol_handler_fn_t handler);

    // get the VFR_HUD throttle
    int16_t get_hud_throttle(void) const {
        const GCS_MAVLINK *link = chan(0);
        if (link == nullptr) {
            return 0;
        }
        return link->vfr_hud_throttle();
    }

    // update uart pass-thru
    void update_passthru();

    void get_sensor_status_flags(uint32_t &present, uint32_t &enabled, uint32_t &health);
    virtual bool vehicle_initialised() const { return true; }

    virtual bool simple_input_active() const { return false; }
    virtual bool supersimple_input_active() const { return false; }

    // set message interval for a given serial port and message id
    // this function is for use by lua scripts, most consumers should use the channel level function
    MAV_RESULT set_message_interval(uint8_t port_num, uint32_t msg_id, int32_t interval_us);

    uint8_t get_channel_from_port_number(uint8_t port_num);

#if HAL_HIGH_LATENCY2_ENABLED
    bool high_latency_link_enabled;
    void enable_high_latency_connections(bool enabled);
    bool get_high_latency_status();
#endif // HAL_HIGH_LATENCY2_ENABLED

    uint8_t sysid_this_mav() const { return sysid; }
    uint32_t telem_delay() const { return mav_telem_delay; }

#if AP_SCRIPTING_ENABLED
    // lua access to command_int
    MAV_RESULT lua_command_int_packet(const mavlink_command_int_t &packet);
#endif

    // Sequence number should be incremented when available modes changes
    // Sent in AVAILABLE_MODES_MONITOR msg
    uint8_t get_available_modes_sequence() const { return available_modes_sequence; }
    void available_modes_changed() { available_modes_sequence += 1; }

protected:

    virtual GCS_MAVLINK *new_gcs_mavlink_backend(AP_HAL::UARTDriver &uart) = 0;

    HAL_Semaphore control_sensors_sem; // protects the three bitmasks
    uint32_t control_sensors_present;
    uint32_t control_sensors_enabled;
    uint32_t control_sensors_health;
    virtual void update_vehicle_sensor_status_flags() {}

    static const struct AP_Param::GroupInfo *_chan_var_info[MAVLINK_COMM_NUM_BUFFERS];
    uint8_t _num_gcs;
    GCS_MAVLINK *_chan[MAVLINK_COMM_NUM_BUFFERS];

    // parameters
    AP_Int16                 sysid;
    AP_Int16                 mav_gcs_sysid;
    AP_Enum16<Option>        mav_options;
    AP_Int8                  mav_telem_delay;

private:

    static GCS *_singleton;

    void create_gcs_mavlink_backend(AP_HAL::UARTDriver &uart);

    char statustext_printf_buffer[256+1];

#if AP_GPS_ENABLED
    virtual AP_GPS::GPS_Status min_status_for_gps_healthy() const {
        // NO_FIX simply excludes NO_GPS
        return AP_GPS::GPS_Status::NO_FIX;
    }
#endif

    void update_sensor_status_flags();

    // time we last saw traffic from our GCS.  Note that there is an
    // identically named field in GCS_MAVLINK:: which is the most
    // recent time that backend saw traffic from MAV_GCS_SYSID
    uint32_t _sysid_gcs_last_seen_time_ms;

    void service_statustext(void);
#if HAL_MEM_CLASS <= HAL_MEM_CLASS_192 || CONFIG_HAL_BOARD == HAL_BOARD_SITL
    static const uint8_t _status_capacity = 7;
#else
    static const uint8_t _status_capacity = 30;
#endif

    // ephemeral state indicating whether the GCS (including via
    // PARAM_SET and upload of param values via FTP) should be allowed
    // to change parameter values:
    bool allow_param_set = HAL_GCS_ALLOW_PARAM_SET_DEFAULT;

    // queue of outgoing statustext messages.  Each entry consumes 58
    // bytes of RAM on stm32
    StatusTextQueue _statustext_queue{_status_capacity};

    // true if we have already allocated protocol objects:
    bool initialised_missionitemprotocol_objects;

    // true if update_send has ever been called:
    bool update_send_has_been_called;

    // handle passthru between two UARTs
    struct {
        bool enabled;
        bool timer_installed;
        AP_HAL::UARTDriver *port1;
        AP_HAL::UARTDriver *port2;
        uint32_t start_ms;
        uint32_t last_ms;
        uint32_t last_port1_data_ms;
        uint32_t baud1;
        uint32_t baud2;
        uint8_t parity1;
        uint8_t parity2;
        uint8_t timeout_s;
        HAL_Semaphore sem;
    } _passthru;

    // timer called to implement pass-thru
    void passthru_timer();

    // this contains the index of the GCS_MAVLINK backend we will
    // first call update_send on.  It is incremented each time
    // GCS::update_send is called so we don't starve later links of
    // time in which they are permitted to send messages.
    uint8_t first_backend_to_send;

    // Sequence number should be incremented when available modes changes
    // Sent in AVAILABLE_MODES_MONITOR msg
    uint8_t available_modes_sequence;
};

GCS &gcs();

// send text when we do have a GCS
#if !defined(HAL_BUILD_AP_PERIPH)
#define GCS_SEND_TEXT(severity, format, args...) gcs().send_text(severity, format, ##args)
#define AP_HAVE_GCS_SEND_TEXT 1
#else
extern "C" {
    void can_printf_severity(uint8_t severity, const char *fmt, ...);
}
#define GCS_SEND_TEXT(severity, format, args...) can_printf_severity(severity, format, ##args)
#define AP_HAVE_GCS_SEND_TEXT 1
#endif

#define GCS_SEND_MESSAGE(msg) gcs().send_message(msg)

#elif defined(HAL_BUILD_AP_PERIPH) && !defined(STM32F1)

// map send text to can_printf() on larger AP_Periph boards
extern "C" {
    void can_printf_severity(uint8_t severity, const char *fmt, ...);
}
#define GCS_SEND_TEXT(severity, format, args...) can_printf_severity(severity, format, ##args)
#define GCS_SEND_MESSAGE(msg)
#define AP_HAVE_GCS_SEND_TEXT 1

/*
  we need a severity enum for the can_printf_severity function with no GCS present
 */
#ifndef HAVE_ENUM_MAV_SEVERITY
enum MAV_SEVERITY
{
    MAV_SEVERITY_EMERGENCY=0,
    MAV_SEVERITY_ALERT=1,
    MAV_SEVERITY_CRITICAL=2,
    MAV_SEVERITY_ERROR=3,
    MAV_SEVERITY_WARNING=4,
    MAV_SEVERITY_NOTICE=5,
    MAV_SEVERITY_INFO=6,
    MAV_SEVERITY_DEBUG=7,
    MAV_SEVERITY_ENUM_END=8,
};
#define HAVE_ENUM_MAV_SEVERITY
#endif

#else // HAL_GCS_ENABLED
// empty send text when we have no GCS
#define GCS_SEND_TEXT(severity, format, args...)
#define GCS_SEND_MESSAGE(msg)
#define AP_HAVE_GCS_SEND_TEXT 0

#endif // HAL_GCS_ENABLED
