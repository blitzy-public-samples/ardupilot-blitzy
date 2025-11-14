/**
 * @file AP_Frsky_SPortParser.h
 * @brief Low-level SPort byte-stream parser for FrSky telemetry protocols
 * 
 * @details This file implements stateful de-stuffing and packet assembly
 *          for both SPort X and D protocols. The parser handles byte-by-byte
 *          processing of the incoming telemetry stream, performs escape
 *          sequence decoding (byte stuffing), validates checksums, and
 *          assembles complete packets for higher-level processing.
 * 
 * @note The parser supports both SPort X protocol (using 0x7E/0x7D escape sequences)
 *       and SPort D protocol (using 0x5E/0x5D escape sequences).
 */

#pragma once

#include "AP_Frsky_SPort.h"

#if AP_FRSKY_SPORT_TELEM_ENABLED

#include <stdint.h>

/**
 * @def FRAME_HEAD
 * @brief SPort X frame header and delimiter byte (0x7E)
 * 
 * @details This byte marks the start of a new SPort X protocol frame.
 *          When encountered in the byte stream, it indicates the beginning
 *          of a new packet and resets the parser state.
 */
#define FRAME_HEAD                  0x7E

/**
 * @def FRAME_DLE
 * @brief SPort X escape character for byte stuffing (0x7D)
 * 
 * @details When this byte appears in the data stream, the next byte
 *          must be XORed with FRAME_XOR (0x20) to recover the original
 *          data value. This allows transmission of reserved bytes (0x7E, 0x7D)
 *          within packet payload.
 */
#define FRAME_DLE                   0x7D

/**
 * @def FRAME_XOR
 * @brief XOR mask applied to escaped bytes in SPort X protocol (0x20)
 * 
 * @details After receiving FRAME_DLE escape character, the following byte
 *          is XORed with this mask to reconstruct the original data byte.
 *          Example: 0x7E in data becomes 0x7D 0x5E on wire (0x7E ^ 0x20 = 0x5E).
 */
#define FRAME_XOR                   0x20

/**
 * @def START_STOP_D
 * @brief SPort D frame delimiter byte (0x5E)
 * 
 * @details Used in SPort D protocol to mark frame boundaries.
 *          Functions similarly to FRAME_HEAD in SPort X protocol.
 */
#define START_STOP_D                0x5E

/**
 * @def BYTESTUFF_D
 * @brief SPort D escape character for byte stuffing (0x5D)
 * 
 * @details Escape character used in SPort D protocol, analogous to
 *          FRAME_DLE in SPort X. Following byte is XORed with STUFF_MASK.
 */
#define BYTESTUFF_D                 0x5D

/**
 * @def TELEMETRY_RX_BUFFER_SIZE
 * @brief Receive buffer size accounting for worst-case byte-stuffing expansion (19 bytes)
 * 
 * @details Buffer sized to hold one complete packet (9 bytes) plus maximum
 *          byte-stuffing overhead. In worst case, every byte could be escaped,
 *          requiring 18 bytes (9 * 2), plus one extra byte for safety margin.
 */
#define TELEMETRY_RX_BUFFER_SIZE    19U  // 9 bytes (full packet), worst case 18 bytes with byte-stuffing (+1)

/**
 * @def SPORT_PACKET_SIZE
 * @brief Final assembled SPort packet size after de-stuffing (9 bytes)
 * 
 * @details All SPort packets are exactly 9 bytes after escape sequences
 *          are processed and the frame is assembled.
 */
#define SPORT_PACKET_SIZE           9U

/**
 * @def STUFF_MASK
 * @brief Mask applied during de-stuffing operations (0x20)
 * 
 * @details XOR mask used to recover original byte values after escape
 *          sequences in both SPort X and D protocols.
 */
#define STUFF_MASK                  0x20

/**
 * @def SPORT_DATA_FRAME
 * @brief SPort data frame type identifier (0x10)
 * 
 * @details Frame type byte indicating a standard telemetry data frame
 *          containing sensor information.
 */
#define SPORT_DATA_FRAME            0x10

/**
 * @def SPORT_UPLINK_FRAME
 * @brief SPort uplink frame type identifier (0x30)
 * 
 * @details Frame type for uplink commands from transmitter to receiver.
 */
#define SPORT_UPLINK_FRAME          0x30

/**
 * @def SPORT_UPLINK_FRAME_RW
 * @brief SPort uplink read/write frame type identifier (0x31)
 * 
 * @details Frame type for uplink commands with read/write operations,
 *          typically used for parameter configuration.
 */
#define SPORT_UPLINK_FRAME_RW       0x31

/**
 * @def SPORT_DOWNLINK_FRAME
 * @brief SPort downlink frame type identifier (0x32)
 * 
 * @details Frame type for downlink data from receiver to transmitter,
 *          carrying telemetry responses and sensor data.
 */
#define SPORT_DOWNLINK_FRAME        0x32

/**
 * @class AP_Frsky_SPortParser
 * @brief Stateful byte-stream parser for FrSky SPort telemetry protocols
 * 
 * @details This class implements a state machine that processes incoming
 *          SPort telemetry bytes one at a time, handling:
 *          - Frame synchronization (detecting start/stop delimiters)
 *          - Byte de-stuffing (decoding escape sequences)
 *          - Packet assembly (accumulating bytes into complete frames)
 *          - CRC validation (verifying packet integrity)
 *          - Duplicate detection (filtering repeated packets)
 * 
 *          The parser supports both SPort X protocol (0x7E/0x7D escaping)
 *          and SPort D protocol (0x5E/0x5D escaping), automatically handling
 *          the appropriate escape sequences based on the incoming data.
 * 
 * @note Byte-stuffing protocol: Reserved bytes (frame delimiters and escape
 *       characters) are escaped when appearing in payload data. The parser
 *       reverses this encoding to reconstruct original packet contents.
 * 
 * @warning process_byte() must be called frequently enough to prevent
 *          buffer overflow. Maximum safe interval depends on baud rate
 *          and packet transmission frequency.
 * 
 * @see AP_Frsky_SPort.h for sport_packet_t structure definition
 * @see AP_Frsky_Telem.h for higher-level telemetry handling
 */
class AP_Frsky_SPortParser
{
public:

    /**
     * @brief Process a single byte from the SPort telemetry stream
     * 
     * @details Implements stateful parsing of incoming SPort byte stream.
     *          Each byte is processed according to current parser state:
     *          - IDLE: Waiting for frame delimiter
     *          - START: Frame delimiter received, expecting data
     *          - IN_FRAME: Accumulating packet bytes
     *          - XOR: Escape character received, next byte needs de-stuffing
     * 
     *          When a complete packet is assembled, CRC is validated against
     *          expected value of 0x00ff. Valid packets are checked for duplicates
     *          before being returned to caller.
     * 
     * @param[out] sport_packet Reference to packet structure to be filled when
     *                          complete valid packet is assembled
     * @param[in]  data         Input byte from telemetry stream
     * 
     * @return true if complete valid non-duplicate packet assembled in sport_packet,
     *         false if more bytes needed or packet invalid
     * 
     * @note This method must be called for every byte received from the telemetry
     *       UART at the full baud rate to avoid data loss
     * @note CRC validation ensures packet integrity - checksum must equal 0x00ff
     *       for packet to be accepted
     * @warning Calling frequency must match or exceed telemetry baud rate divided
     *          by 10 bits per byte to prevent buffer overflow
     * 
     * @see get_packet() for final packet extraction after assembly
     * @see should_process_packet() for duplicate detection logic
     */
    bool process_byte(AP_Frsky_SPort::sport_packet_t &sport_packet, const uint8_t data);

private:
    /**
     * @enum ParseState
     * @brief State machine states for SPort byte-stream parser
     * 
     * @details The parser progresses through these states as bytes are
     *          received and processed. State transitions occur based on
     *          special bytes (delimiters, escape characters) and byte count.
     */
    enum class ParseState : uint8_t {
        /**
         * @brief Waiting for frame delimiter to begin packet reception
         * 
         * @details Initial state. Parser remains here until FRAME_HEAD (0x7E)
         *          or START_STOP_D (0x5E) delimiter is received.
         */
        IDLE,
        
        /**
         * @brief Frame delimiter received, ready to accumulate packet data
         * 
         * @details Parser has detected start of frame and will begin storing
         *          incoming bytes into rx_buffer.
         */
        START,
        
        /**
         * @brief Actively receiving and accumulating packet bytes
         * 
         * @details Parser is building packet in rx_buffer. Remains in this state
         *          until SPORT_PACKET_SIZE bytes received or escape character detected.
         */
        IN_FRAME,
        
        /**
         * @brief Escape character received, next byte requires XOR de-stuffing
         * 
         * @details Parser has received FRAME_DLE (0x7D) or BYTESTUFF_D (0x5D).
         *          Next byte will be XORed with STUFF_MASK (0x20) to recover
         *          original data value before storage.
         */
        XOR,
    };

    /**
     * @struct _parse_state
     * @brief Parser state tracking structure
     * 
     * @details Maintains all state needed for incremental packet assembly:
     *          - Current position in receive buffer
     *          - Accumulated packet bytes (with byte-stuffing expansion)
     *          - Last successfully received packet (for duplicate detection)
     *          - Current parser state machine position
     */
    struct {
        /**
         * @brief Number of bytes currently accumulated in rx_buffer
         * 
         * @details Incremented as each byte (after de-stuffing) is added to
         *          rx_buffer. Reset to 0 when new frame starts. Maximum value
         *          is SPORT_PACKET_SIZE (9 bytes).
         */
        uint8_t rx_buffer_count;
        
        /**
         * @brief Receive buffer for packet assembly with byte-stuffing expansion
         * 
         * @details Holds incoming packet bytes as they are received and de-stuffed.
         *          Size accounts for worst-case where every payload byte is escaped.
         *          Buffer contents are validated and copied to output packet when
         *          SPORT_PACKET_SIZE bytes have been accumulated.
         * 
         * @note Buffer size (19 bytes) > packet size (9 bytes) to handle stuffing
         */
        uint8_t rx_buffer[TELEMETRY_RX_BUFFER_SIZE];
        
        /**
         * @brief Copy of last successfully received packet for duplicate detection
         * 
         * @details Stores previous valid packet to filter out repeated transmissions.
         *          SPort protocol may transmit same packet multiple times; parser
         *          compares incoming packets against this buffer to avoid processing
         *          duplicates.
         * 
         * @note Updated only after successful CRC validation and duplicate check
         */
        uint8_t last_packet[SPORT_PACKET_SIZE];
        
        /**
         * @brief Current parser state machine position
         * 
         * @details Tracks parser progress through packet reception cycle.
         *          State transitions drive parsing logic and determine how
         *          each incoming byte is processed.
         * 
         * @see ParseState for state definitions and transitions
         */
        ParseState state;
    } _parse_state;

    /**
     * @brief Check if packet should be processed or discarded as duplicate
     * 
     * @param[in] packet            Pointer to assembled packet data to check
     * @param[in] discard_duplicates If true, compare against last_packet and reject duplicates
     * 
     * @return true if packet is new and should be processed, false if duplicate
     */
    bool should_process_packet(const uint8_t *packet, bool discard_duplicates);
    
    /**
     * @brief Extract assembled packet and perform final validation
     * 
     * @param[out] sport_packet        Reference to output packet structure
     * @param[in]  discard_duplicates  If true, filter out duplicate packets
     * 
     * @return true if valid packet extracted, false if CRC invalid or duplicate
     * 
     * @note Validates CRC checksum must equal 0x00ff before accepting packet
     */
    bool get_packet(AP_Frsky_SPort::sport_packet_t &sport_packet, bool discard_duplicates);
};

#endif  // AP_FRSKY_SPORT_TELEM_ENABLED
