/**
 * @file GCS_MAVLink.h
 * @brief Central header for MAVLink integration layer in ArduPilot
 * 
 * @details This header provides the central compile-time mapping between MAVLink 
 *          generated code and ArduPilot's runtime MAVLink communication system.
 *          
 *          Key responsibilities:
 *          - Configuration of MAVLink communication channels and buffer sizing
 *          - Low-level send/receive buffer management
 *          - Thread-safety primitives (locking/unlocking) for multi-threaded MAVLink transmission
 *          - Mapping of MAVLink generated code callbacks to ArduPilot communication functions
 *          - Channel validation and status management
 *          
 *          This header must be included before any MAVLink generated headers to properly
 *          configure the MAVLink library's buffer allocation and callback mechanisms.
 *          
 *          Threading Model: MAVLink send operations support multi-threaded access through
 *          explicit lock/unlock pairs (comm_send_lock/comm_send_unlock) that reserve
 *          transmit buffer space and protect concurrent access.
 *          
 * @note This file configures MAVLink v2.0 protocol support
 * @note MAVLink separate helpers are disabled for Arduino GUI build compatibility
 */
#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_Networking/AP_Networking_Config.h>

// we have separate helpers disabled to make it possible
// to select MAVLink 1.0 in the arduino GUI build
#define MAVLINK_SEPARATE_HELPERS
#define MAVLINK_NO_CONVERSION_HELPERS

/**
 * @brief Maps MAVLink send operation to ArduPilot's comm_send_buffer()
 * @details This macro is called by MAVLink generated code to transmit byte buffers.
 *          It redirects to ArduPilot's communication layer for actual transmission.
 * @param chan MAVLink channel number
 * @param buf Pointer to byte buffer to transmit
 * @param len Number of bytes to transmit
 */
#define MAVLINK_SEND_UART_BYTES(chan, buf, len) comm_send_buffer(chan, buf, len)

/**
 * @brief Maps MAVLink transmission start to ArduPilot's locking mechanism
 * @details Acquires channel lock and reserves transmit buffer space before MAVLink send.
 *          Must be paired with MAVLINK_END_UART_SEND for proper lock release.
 * @param chan MAVLink channel number
 * @param size Number of bytes to reserve in transmit buffer
 */
#define MAVLINK_START_UART_SEND(chan, size) comm_send_lock(chan, size)

/**
 * @brief Maps MAVLink transmission end to ArduPilot's unlock mechanism
 * @details Releases channel lock after MAVLink send completes.
 *          Must be paired with MAVLINK_START_UART_SEND.
 * @param chan MAVLink channel number
 * @param size Unused parameter (kept for MAVLink API compatibility)
 */
#define MAVLINK_END_UART_SEND(chan, size) comm_send_unlock(chan)

/**
 * @brief Maximum number of simultaneous MAVLink communication channels
 * @details Defines the compile-time limit for MAVLink channel buffers.
 *          - Large programs (>1024KB): 8 channels for extra networking or CAN ports
 *          - Small programs (≤1024KB): 5 channels to conserve memory
 *          
 *          Each channel corresponds to a potential telemetry link (UART, UDP, TCP, CAN).
 *          Increasing this value increases RAM usage proportionally.
 * @note This constant determines array sizing for mavlink_comm_port[] and related structures
 */
#if HAL_PROGRAM_SIZE_LIMIT_KB > 1024
// allow 8 telemetry ports, allowing for extra networking or CAN ports
#define MAVLINK_COMM_NUM_BUFFERS 8
#else
// allow five telemetry ports
#define MAVLINK_COMM_NUM_BUFFERS 5
#endif

/**
 * @brief Enables MAVLink channel buffer accessor function
 * @details When defined to 1, enables mavlink_get_channel_buffer() function
 *          that allows MAVLink generated code to access per-channel message buffers.
 */
#define MAVLINK_GET_CHANNEL_BUFFER 1

/**
 * @brief Enables MAVLink channel status accessor function
 * @details When defined to 1, enables mavlink_get_channel_status() function
 *          that allows MAVLink generated code to access per-channel status structures
 *          containing packet statistics and parse state.
 */
#define MAVLINK_GET_CHANNEL_STATUS 1

/**
 * @warning Alignment cast warnings suppressed for MAVLink generated code
 * @details The MAVLink protocol code generator performs its own alignment management
 *          for efficient wire protocol packing. Cast alignment warnings (-Wcast-align)
 *          and address-of-packed-member warnings are safely ignored in this context
 *          as MAVLink ensures proper alignment handling internally.
 */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-align"

#if defined(__GNUC__) && __GNUC__ >= 9
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
#endif

#include "include/mavlink/v2.0/all/version.h"

/**
 * @brief Maximum MAVLink message payload length in bytes
 * @details Defines the maximum size of the payload field in a MAVLink message (255 bytes).
 *          This limit is specified by the MAVLink protocol and determines buffer sizing
 *          for message construction and parsing. Total message size includes additional
 *          overhead for header, checksum, and signature fields.
 * @note MAVLink v2.0 protocol specification limit
 */
#define MAVLINK_MAX_PAYLOAD_LEN 255

#include "include/mavlink/v2.0/mavlink_types.h"

/**
 * @brief Array of UART driver pointers for each MAVLink communication channel
 * @details Maps MAVLink channel numbers to their corresponding UART/serial port drivers.
 *          Each element can be nullptr if the channel is not configured or a pointer
 *          to the AP_HAL::UARTDriver instance handling that telemetry link.
 *          
 *          Indexed by mavlink_channel_t (0 to MAVLINK_COMM_NUM_BUFFERS-1).
 *          Used by comm_send_buffer() and other low-level transmission functions.
 * @note Array size determined by MAVLINK_COMM_NUM_BUFFERS (5 or 8)
 */
extern AP_HAL::UARTDriver	*mavlink_comm_port[MAVLINK_COMM_NUM_BUFFERS];

/**
 * @brief Flags indicating alternative telemetry protocol active on each channel
 * @details Boolean array tracking whether each MAVLink channel is currently using
 *          an alternative telemetry protocol (e.g., FrSky, LTM) instead of standard MAVLink.
 *          When true for a channel, MAVLink message transmission may be suspended
 *          to allow the alternative protocol to use the communication link.
 *          
 *          Indexed by mavlink_channel_t (0 to MAVLINK_COMM_NUM_BUFFERS-1).
 * @note Allows protocol multiplexing on a single serial port
 */
extern bool gcs_alternative_active[MAVLINK_COMM_NUM_BUFFERS];

/**
 * @brief MAVLink system identification structure
 * @details Contains the system ID and component ID used to identify this vehicle
 *          in MAVLink communications. Structure fields:
 *          - sysid: System ID (1-255), typically unique per vehicle
 *          - compid: Component ID (1-255), typically MAV_COMP_ID_AUTOPILOT1 (1)
 *          
 *          These IDs are included in all outgoing MAVLink messages and used by
 *          ground control stations to route messages to the correct vehicle/component.
 * @note Configured via SYSID_THISMAV and SYSID_MYGCS parameters
 */
extern mavlink_system_t mavlink_system;

/**
 * @brief Validates that a MAVLink channel number is within configured bounds
 * @details Sanity check to verify channel number is less than MAVLINK_COMM_NUM_BUFFERS.
 *          Used throughout the MAVLink subsystem to prevent array overruns when
 *          accessing channel-indexed arrays (buffers, status structures, port pointers).
 * 
 * @param[in] chan MAVLink channel number to validate
 * @return true if channel number is valid (0 to MAVLINK_COMM_NUM_BUFFERS-1), false otherwise
 * @note This is a compile-time bounds check, not a runtime channel availability check
 */
static inline bool valid_channel(mavlink_channel_t chan)
{
    return static_cast<int>(chan) < MAVLINK_COMM_NUM_BUFFERS;
}

/**
 * @brief Retrieves the message buffer for a specified MAVLink channel
 * @details Returns a pointer to the mavlink_message_t buffer allocated for the given channel.
 *          This buffer is used by MAVLink generated code during message packing operations.
 *          Each channel maintains its own independent message buffer to support concurrent
 *          message construction on multiple channels.
 * 
 * @param[in] chan Channel number (0 to MAVLINK_COMM_NUM_BUFFERS-1)
 * @return Pointer to mavlink_message_t structure for this channel, or nullptr if invalid channel
 * @note Called by MAVLink generated code when MAVLINK_GET_CHANNEL_BUFFER is defined
 */
mavlink_message_t* mavlink_get_channel_buffer(uint8_t chan);

/**
 * @brief Retrieves the status structure for a specified MAVLink channel
 * @details Returns a pointer to the mavlink_status_t structure containing parse state
 *          and packet statistics for the given channel. The status structure tracks:
 *          - Parse state machine position
 *          - Packet sequence numbers
 *          - Packet drop counters
 *          - Buffer overflow indicators
 * 
 * @param[in] chan Channel number (0 to MAVLINK_COMM_NUM_BUFFERS-1)
 * @return Pointer to mavlink_status_t structure for this channel, or nullptr if invalid channel
 * @note Called by MAVLink generated code when MAVLINK_GET_CHANNEL_STATUS is defined
 */
mavlink_status_t* mavlink_get_channel_status(uint8_t chan);

/**
 * @brief Sends a byte buffer to the specified MAVLink channel
 * @details Transmits a buffer of bytes through the UART/serial driver associated with
 *          the given MAVLink channel. This is the low-level transmission function called
 *          by MAVLink generated code (via MAVLINK_SEND_UART_BYTES macro) to send
 *          serialized MAVLink message bytes.
 *          
 *          The function writes to mavlink_comm_port[chan] if configured, handling
 *          the actual hardware transmission through the HAL UART driver.
 * 
 * @param[in] chan MAVLink channel number to transmit on
 * @param[in] buf Pointer to byte buffer containing data to transmit
 * @param[in] len Number of bytes to transmit from buffer
 * @note Channel must be locked (via comm_send_lock) before calling in multi-threaded context
 */
void comm_send_buffer(mavlink_channel_t chan, const uint8_t *buf, uint8_t len);

/// Check for available transmit space on the nominated MAVLink channel
///
/// @param chan		Channel to check
/// @returns		Number of bytes available
uint16_t comm_get_txspace(mavlink_channel_t chan);

#define MAVLINK_USE_CONVENIENCE_FUNCTIONS
#include "include/mavlink/v2.0/all/mavlink.h"

/**
 * @brief Acquires channel lock for multi-threaded MAVLink transmission
 * @details Locks the specified MAVLink channel and reserves transmit buffer space
 *          before sending MAVLink messages. This prevents concurrent threads from
 *          interleaving message bytes in the output stream.
 *          
 *          Must be paired with comm_send_unlock() after transmission completes.
 *          Called automatically by MAVLink generated code via MAVLINK_START_UART_SEND macro.
 *          
 *          Typical usage pattern:
 *          1. comm_send_lock(chan, message_size) - Reserve space and acquire lock
 *          2. mavlink_msg_xxx_send() - Send message(s)
 *          3. comm_send_unlock(chan) - Release lock
 * 
 * @param[in] chan MAVLink channel number to lock
 * @param[in] size Number of bytes to reserve in transmit buffer
 * @note Blocks if another thread holds the lock for this channel
 * @warning Must call comm_send_unlock() to release lock, even if transmission fails
 */
void comm_send_lock(mavlink_channel_t chan, uint16_t size);

/**
 * @brief Releases channel lock after MAVLink transmission
 * @details Unlocks the specified MAVLink channel after message transmission completes,
 *          allowing other threads to send messages on this channel.
 *          
 *          Must be paired with comm_send_lock() called earlier.
 *          Called automatically by MAVLink generated code via MAVLINK_END_UART_SEND macro.
 * 
 * @param[in] chan MAVLink channel number to unlock
 * @note Must be called from the same thread that called comm_send_lock()
 * @warning Failing to unlock after locking will deadlock other threads trying to send
 */
void comm_send_unlock(mavlink_channel_t chan);

/**
 * @brief Returns reference to the semaphore protecting a MAVLink channel
 * @details Provides direct access to the HAL_Semaphore used for synchronizing access
 *          to the specified MAVLink channel. Allows manual lock management using
 *          WITH_SEMAPHORE or explicit take()/give() calls when more complex locking
 *          patterns are needed beyond the simple lock/unlock pair.
 * 
 * @param[in] chan MAVLink channel number
 * @return Reference to HAL_Semaphore for this channel
 * @note For advanced use cases; prefer comm_send_lock()/unlock() for typical sending
 */
HAL_Semaphore &comm_chan_lock(mavlink_channel_t chan);

#pragma GCC diagnostic pop
