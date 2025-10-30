/**
 * @file msp.h
 * @brief Core MSP (MultiWii Serial Protocol) framing, parsing, and encoding implementation
 * 
 * @details This file implements the MSP protocol based on Betaflight's MSP serial protocol.
 *          MSP is a lightweight binary protocol used for communication between flight controllers
 *          and ground control stations, OSD devices, and other peripherals.
 *          
 *          Supports three protocol variants:
 *          - MSP V1: Original protocol with XOR checksum
 *          - MSP V2 over V1: V2 messages encapsulated in V1 frames
 *          - MSP V2 Native: Native V2 framing with CRC8 DVB-S2 checksums
 *          
 *          Key features:
 *          - Byte-at-a-time finite state machine parser for all protocol versions
 *          - Jumbo frame support for large messages (up to 255 bytes)
 *          - DisplayPort protocol support for OSD text rendering
 *          - Bidirectional request/response message flow
 *          
 * @warning MSP v1 uses XOR checksum while MSP v2 uses CRC8 DVB-S2. Frame formats
 *          and header structures differ significantly between versions.
 * 
 * @note Based on Betaflight MSP implementation:
 *       - betaflight/src/main/msp/msp.h
 *       - betaflight/src/main/msp/msp_serial.c
 *       - inav/src/main/msp/msp_protocol_v2_sensor.h
 * 
 * @author ArduPilot Development Team
 */

#pragma once

#include "AP_MSP_config.h"

#if HAL_MSP_ENABLED

#include <AP_HAL/UARTDriver.h>

#include "msp_osd.h"
#include "msp_protocol.h"
#include "msp_sbuf.h"
#include "msp_version.h"
#include "msp_sensors.h"

// betaflight/src/main/common/utils.h
/// @brief Helper macro to get pointer to end of array
#define MSP_ARRAYEND(x) (&(x)[ARRAY_SIZE(x)])
/// @brief Helper macro to mark parameter as intentionally unused (suppresses compiler warnings)
#define MSP_UNUSED(x) (void)(x)

// betaflight/src/main/msp/msp_serial.c
/**
 * @def JUMBO_FRAME_SIZE_LIMIT
 * @brief Maximum frame size for jumbo frames in bytes
 * @details MSP jumbo frames allow payloads larger than standard 255 byte limit.
 *          This defines the maximum size for these extended frames.
 */
#define JUMBO_FRAME_SIZE_LIMIT 255

// betaflight/src/main/msp/msp.h
/// @brief Frame ID marker for MSP V2 native protocol (0xFF)
#define MSP_V2_FRAME_ID 255
/// @brief Version magic bytes for MSP protocol identification {'M', 'M', 'X'}
#define MSP_VERSION_MAGIC_INITIALIZER { 'M', 'M', 'X' }

/**
 * @def MSP_PORT_INBUF_SIZE
 * @brief Input buffer size in bytes for receiving MSP messages
 * @details Buffer must be large enough to hold the largest expected MSP message
 *          including header, payload, and checksum/CRC bytes.
 */
#define MSP_PORT_INBUF_SIZE 192

/**
 * @def MSP_PORT_OUTBUF_SIZE
 * @brief Output buffer size in bytes for transmitting MSP messages
 * @details Output buffer is larger than input to handle encoding overhead
 *          and allow buffering of multiple outgoing messages.
 */
#define MSP_PORT_OUTBUF_SIZE 512

/**
 * @def MSP_MAX_HEADER_SIZE
 * @brief Maximum header size in bytes for MSP v2 frames
 * @details MSP v2 native headers can be up to 9 bytes:
 *          - '$' (1 byte)
 *          - 'X' (1 byte)
 *          - '<' or '>' (1 byte)
 *          - flags (1 byte)
 *          - cmd (2 bytes)
 *          - size (2 bytes)
 *          - checksum (1 byte)
 */
#define MSP_MAX_HEADER_SIZE     9

// inav/src/main/msp/msp_protocol_v2_sensor.h
/**
 * @def MSP2_IS_SENSOR_MESSAGE
 * @brief Check if MSP command ID is a sensor message (0x1F00-0x1FFF range)
 * @param x MSP command ID to check
 * @return true if command is in sensor message range, false otherwise
 * @details Sensor messages are a special category in MSP v2 protocol used
 *          for high-frequency telemetry data from sensors (IMU, GPS, etc.)
 */
#define MSP2_IS_SENSOR_MESSAGE(x)   ((x) >= 0x1F00U && (x) <= 0x1FFFU)

// betaflight/src/main/io/displayport_msp.h
/**
 * @name DisplayPort MSP Attribute Flags
 * @brief Bit flags for DisplayPort attribute byte controlling text rendering
 * @{
 */
/**
 * @def DISPLAYPORT_MSP_ATTR_VERSION
 * @brief Format indicator bit (bit 7) - must be zero for V2 (and V1) format
 */
#define DISPLAYPORT_MSP_ATTR_VERSION 1U<<7

/**
 * @def DISPLAYPORT_MSP_ATTR_BLINK
 * @brief Device local blink attribute (bit 6) - causes text to blink on display
 */
#define DISPLAYPORT_MSP_ATTR_BLINK   1U<<6

/**
 * @def DISPLAYPORT_MSP_ATTR_MASK
 * @brief Mask for extracting display attributes (excludes VERSION and BLINK bits)
 */
#define DISPLAYPORT_MSP_ATTR_MASK    (~(DISPLAYPORT_MSP_ATTR_VERSION|DISPLAYPORT_MSP_ATTR_BLINK))
/** @} */

// betaflight/src/main/io/displayport_msp.c
/**
 * @def OSD_MSP_DISPLAYPORT_MAX_STRING_LENGTH
 * @brief Maximum string length in characters for DisplayPort text rendering
 * @details Limits the maximum string that can be sent in a single
 *          MSP_DISPLAYPORT_WRITE_STRING command to prevent buffer overflows.
 */
#define OSD_MSP_DISPLAYPORT_MAX_STRING_LENGTH 30

class AP_MSP_Telem_Backend;

namespace MSP
{
/**
 * @enum msp_version_e
 * @brief MSP protocol version enumeration
 * @details Identifies which variant of the MSP protocol is in use for framing and encoding.
 *          Different versions have different frame structures, checksums, and capabilities.
 */
typedef enum {
    MSP_V1          = 0,  ///< Original MSP protocol version 1 with XOR checksum
    MSP_V2_OVER_V1  = 1,  ///< MSP v2 messages encapsulated in v1 frame format for compatibility
    MSP_V2_NATIVE   = 2,  ///< Native MSP v2 framing with CRC8 DVB-S2 checksum and extended addressing
    MSP_VERSION_COUNT     ///< Total number of protocol versions (not a valid version)
} msp_version_e;

/**
 * @enum MSPCommandResult
 * @brief MSP command processing result codes
 * @details Return values for MSP command handlers indicating success, failure, or no-reply status.
 *          Positive values indicate acknowledgment, negative indicate errors, zero means no reply needed.
 */
typedef enum {
    MSP_RESULT_ACK = 1,        ///< Command processed successfully (positive ACK)
    MSP_RESULT_ERROR = -1,     ///< Command processing failed (negative error code)
    MSP_RESULT_NO_REPLY = 0    ///< Command processed but no reply should be sent
} MSPCommandResult;

/**
 * @struct msp_packet_s
 * @brief MSP packet structure containing command and payload data
 * @details Represents a complete MSP message with command ID, control flags,
 *          processing result, and payload buffer. Used for both encoding
 *          outgoing messages and storing parsed incoming messages.
 */
typedef struct msp_packet_s {
    sbuf_t buf;      ///< Smart buffer (sbuf) containing packet payload data
    int16_t cmd;     ///< MSP command ID identifying the message type
    uint8_t flags;   ///< Control flags from msp_flags_e (e.g., MSP_FLAG_DONT_REPLY)
    int16_t result;  ///< Command processing result from MSPCommandResult enum
} msp_packet_t;

/**
 * @enum msp_flags_e
 * @brief MSP packet control flags
 * @details Bit flags that control MSP message handling behavior
 */
typedef enum {
    MSP_FLAG_DONT_REPLY = (1 << 0),  ///< Suppress reply transmission (one-way message)
} msp_flags_e;

/**
 * @enum msp_state_e
 * @brief MSP parser finite state machine states
 * @details Defines all states for the byte-at-a-time MSP protocol parser.
 *          The parser implements a state machine that processes incoming bytes
 *          and transitions through states based on protocol framing rules.
 *          Supports all three MSP protocol versions (V1, V2_OVER_V1, V2_NATIVE).
 * 
 * @note Parser state machine flow:
 *       IDLE -> HEADER_START -> HEADER_M or HEADER_X -> version-specific states -> COMMAND_RECEIVED
 */
typedef enum {
    MSP_IDLE,                    ///< Idle state waiting for '$' start character
    MSP_HEADER_START,            ///< Received '$', waiting for 'M' or 'X'
    MSP_HEADER_M,                ///< Received '$M', waiting for '<' or '>' direction indicator
    MSP_HEADER_X,                ///< Received '$X', indicates MSP v2 native frame

    MSP_HEADER_V1,               ///< Parsing MSP v1 header (size, cmd)
    MSP_PAYLOAD_V1,              ///< Receiving MSP v1 payload bytes
    MSP_CHECKSUM_V1,             ///< Receiving MSP v1 XOR checksum byte

    MSP_HEADER_V2_OVER_V1,       ///< Parsing MSP v2 header encapsulated in v1 frame
    MSP_PAYLOAD_V2_OVER_V1,      ///< Receiving MSP v2 over v1 payload bytes
    MSP_CHECKSUM_V2_OVER_V1,     ///< Receiving MSP v2 over v1 CRC8 checksum byte

    MSP_HEADER_V2_NATIVE,        ///< Parsing native MSP v2 header (flags, cmd, size)
    MSP_PAYLOAD_V2_NATIVE,       ///< Receiving native MSP v2 payload bytes
    MSP_CHECKSUM_V2_NATIVE,      ///< Receiving native MSP v2 CRC8 checksum byte

    MSP_COMMAND_RECEIVED         ///< Complete valid packet received, ready for processing
} msp_state_e;

/**
 * @enum msp_displayport_subcmd_e
 * @brief MSP DisplayPort subcommand codes for OSD text rendering
 * @details DisplayPort protocol allows rendering text on OSD devices via MSP messages.
 *          These subcommands control screen clearing, text positioning, and display updates.
 *          Used within MSP_DISPLAYPORT command payload to specify the operation type.
 */
typedef enum : uint8_t {
    MSP_DISPLAYPORT_HEARTBEAT = 0,      ///< Keepalive message to maintain DisplayPort connection
    MSP_DISPLAYPORT_RELEASE = 1,        ///< Release DisplayPort control (return to normal OSD)
    MSP_DISPLAYPORT_CLEAR_SCREEN = 2,   ///< Clear all text from screen buffer
    MSP_DISPLAYPORT_WRITE_STRING = 3,   ///< Write text string at specified row/column position
    MSP_DISPLAYPORT_DRAW_SCREEN = 4,    ///< Commit buffer to display (render accumulated changes)
    MSP_DISPLAYPORT_SET_OPTIONS = 5,    ///< Configure DisplayPort options (font, video mode, etc.)
} msp_displayport_subcmd_e;

/**
 * @struct msp_header_v1_t
 * @brief MSP v1 protocol frame header structure (PACKED)
 * @details Standard MSP v1 header containing payload size and command ID.
 *          Follows the '$M<' or '$M>' direction indicator bytes.
 *          Limited to 255 byte payload size (single byte size field).
 * 
 * @note Frame format: '$' 'M' '<|>' [size] [cmd] [payload...] [checksum]
 */
typedef struct PACKED {
    uint8_t size;   ///< Payload size in bytes (0-255)
    uint8_t cmd;    ///< MSP command ID (8-bit address space in v1)
} msp_header_v1_t;

/**
 * @struct msp_header_jumbo_t
 * @brief MSP jumbo frame extended header structure (PACKED)
 * @details Extended header for jumbo frames that exceed 255 byte payload limit.
 *          Used when standard v1 size field is insufficient for large messages.
 *          Follows the standard v1 header when jumbo frame is indicated.
 */
typedef struct PACKED {
    uint16_t size;  ///< Extended payload size in bytes (16-bit for jumbo frames)
} msp_header_jumbo_t;

/**
 * @struct msp_header_v2_t
 * @brief MSP v2 protocol frame header structure (PACKED)
 * @details Native MSP v2 header with extended features:
 *          - 16-bit command address space (vs 8-bit in v1)
 *          - 16-bit payload size field (native jumbo frame support)
 *          - Flags byte for protocol extensions
 *          - CRC8 DVB-S2 checksum (vs XOR in v1)
 * 
 * @note Frame format: '$' 'X' '<|>' [flags] [cmd_lo] [cmd_hi] [size_lo] [size_hi] [payload...] [crc8]
 */
typedef struct PACKED {
    uint8_t  flags;  ///< Control flags (currently unused, reserved for future extensions)
    uint16_t cmd;    ///< MSP command ID (16-bit address space in v2)
    uint16_t size;   ///< Payload size in bytes (16-bit, native jumbo frame support)
} msp_header_v2_t;

/**
 * @struct msp_port_s
 * @brief MSP port instance containing parser state and buffers
 * @details Maintains complete state for MSP protocol parser and UART interface.
 *          Each MSP port represents one serial connection capable of sending and
 *          receiving MSP messages. Includes input buffer, parser state machine,
 *          checksum accumulators, and current message metadata.
 * 
 * @note One instance required per UART/serial port used for MSP communication
 */
typedef struct msp_port_s {
    AP_HAL::UARTDriver *uart;               ///< UART driver instance for physical serial communication
    msp_state_e c_state;                    ///< Current parser state machine state
    uint8_t in_buf[MSP_PORT_INBUF_SIZE];    ///< Input buffer for received message data
    uint_fast16_t offset;                   ///< Current read/write offset in input buffer (bytes)
    uint_fast16_t data_size;                ///< Expected payload size of current message (bytes)
    msp_version_e msp_version;              ///< Detected MSP protocol version of current message
    uint8_t cmd_flags;                      ///< Command flags from current message header
    uint16_t cmd_msp;                       ///< Command ID of current message being parsed
    uint8_t checksum1;                      ///< Checksum accumulator 1 (XOR for v1, CRC8 for v2)
    uint8_t checksum2;                      ///< Checksum accumulator 2 (for multi-byte checksums)
} msp_port_t;

/**
 * @enum battery_state_e
 * @brief Battery status enumeration from Betaflight
 * @details Standardized battery state codes used in MSP telemetry messages
 *          to indicate battery health and voltage level status.
 * 
 * @note Source: betaflight/src/main/sensors/battery.h
 */
typedef enum : uint8_t {
    MSP_BATTERY_OK = 0,          ///< Battery voltage normal, no warnings
    MSP_BATTERY_WARNING,         ///< Battery voltage at warning threshold
    MSP_BATTERY_CRITICAL,        ///< Battery voltage critically low, immediate action required
    MSP_BATTERY_NOT_PRESENT,     ///< Battery not detected or disconnected
    MSP_BATTERY_INIT             ///< Battery monitoring initializing, state unknown
} battery_state_e;

/**
 * @brief Calculate XOR checksum for MSP v1 protocol
 * 
 * @details Computes running XOR checksum over a buffer of bytes. Used for
 *          MSP v1 frame integrity verification. The checksum is calculated
 *          by XORing all bytes in the data buffer with the initial checksum value.
 * 
 * @param[in] checksum Initial checksum value (typically 0 or previous accumulated value)
 * @param[in] data Pointer to data buffer to checksum
 * @param[in] len Length of data buffer in bytes
 * 
 * @return Updated checksum value (XOR of initial checksum and all data bytes)
 * 
 * @note This is only used for MSP v1. MSP v2 uses CRC8 DVB-S2 instead.
 * @note Example: checksum = msp_serial_checksum_buf(0, data, len);
 */
uint8_t msp_serial_checksum_buf(uint8_t checksum, const uint8_t *data, uint32_t len);

/**
 * @brief Send framed MSP message via UART
 * 
 * @details Transmits a complete MSP frame by writing header, payload, and checksum/CRC
 *          to the UART in sequence. Checks available UART transmit buffer space
 *          before writing to prevent data loss.
 * 
 * @param[in] msp MSP port instance containing UART driver
 * @param[in] hdr Pointer to frame header buffer
 * @param[in] hdr_len Header buffer length in bytes
 * @param[in] data Pointer to payload data buffer (can be NULL if data_len is 0)
 * @param[in] data_len Payload data length in bytes
 * @param[in] crc Pointer to checksum/CRC buffer
 * @param[in] crc_len Checksum/CRC buffer length in bytes (1 for XOR, 1 for CRC8)
 * 
 * @return Total number of bytes successfully written to UART
 * 
 * @warning Checks UART txspace() before writing. If insufficient space, may write
 *          partial frame or no frame at all. Caller should verify return value.
 * 
 * @note Typically called by msp_serial_encode() after frame construction
 */
uint32_t msp_serial_send_frame(msp_port_t *msp, const uint8_t * hdr, uint32_t hdr_len, const uint8_t * data, uint32_t data_len, const uint8_t * crc, uint32_t crc_len);

/**
 * @brief Encode MSP packet with version-specific framing
 * 
 * @details Constructs complete MSP frame with appropriate headers, payload, and
 *          checksum/CRC based on specified protocol version. Handles differences
 *          between MSP v1 (XOR checksum) and MSP v2 (CRC8 DVB-S2) automatically.
 *          Transmits encoded frame via UART using msp_serial_send_frame().
 * 
 * @param[in] msp MSP port instance containing UART driver
 * @param[in] packet Pointer to MSP packet structure containing command and payload
 * @param[in] msp_version Target MSP protocol version (MSP_V1, MSP_V2_OVER_V1, or MSP_V2_NATIVE)
 * @param[in] is_request true for request messages ('<'), false for response messages ('>')
 * 
 * @return Total number of bytes encoded and transmitted
 * 
 * @note MSP v1 uses 8-bit XOR checksum, MSP v2 uses CRC8 DVB-S2 polynomial 0xD5
 * @note Automatically selects jumbo frame format if payload exceeds 255 bytes
 * 
 * @warning Packet buffer (packet->buf) must contain valid payload data before calling
 */
uint32_t msp_serial_encode(msp_port_t *msp, msp_packet_t *packet, msp_version_e msp_version, bool is_request=false);

/**
 * @brief Parse incoming byte stream into MSP packets
 * 
 * @details Implements byte-at-a-time finite state machine parser for all MSP protocol
 *          versions (V1, V2_OVER_V1, V2_NATIVE). Processes one received byte per call,
 *          updating parser state and accumulating message data in input buffer.
 *          Validates frame integrity using checksums and returns true when a complete
 *          valid packet has been received.
 * 
 * @param[in,out] msp MSP port instance containing parser state and input buffer
 * @param[in] c Single received byte to process
 * 
 * @return true if complete valid packet received (parser state is MSP_COMMAND_RECEIVED),
 *         false if more bytes needed or invalid frame detected
 * 
 * @note Call repeatedly for each byte received from UART to reconstruct MSP messages
 * @note Parser automatically detects protocol version from frame headers
 * @note State machine resets to MSP_IDLE on checksum/CRC failure or invalid frame
 * 
 * @warning Validates checksums (XOR for v1, CRC8 for v2) and rejects invalid frames.
 *          Caller should check return value before processing packet data.
 * @warning Input buffer overflow protection: rejects frames exceeding MSP_PORT_INBUF_SIZE
 */
bool msp_parse_received_data(msp_port_t *msp, uint8_t c);
}

#endif //HAL_MSP_ENABLED
