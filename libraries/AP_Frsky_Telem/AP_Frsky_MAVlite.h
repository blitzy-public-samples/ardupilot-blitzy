/**
 * @file AP_Frsky_MAVlite.h
 * @brief MAVlite protocol constants and definitions for FrSky bidirectional telemetry
 * 
 * @details MAVlite is a lightweight message framing protocol designed for
 *          bidirectional communication over FrSky SPort telemetry links.
 *          It provides a compact, efficient way to transport command and
 *          parameter messages between ground control stations and vehicles
 *          using the FrSky SmartPort protocol as the physical layer.
 * 
 *          MAVlite messages are fragmented into multiple SPort packets due
 *          to SPort's limited payload capacity. This header defines the
 *          constants and macros used for message segmentation, queue sizing,
 *          and protocol operation.
 * 
 * @see AP_Frsky_MAVlite_Message.h
 * @see AP_Frsky_MAVlite_MAVliteToSPort.h
 * @see AP_Frsky_MAVlite_SPortToMAVlite.h
 */

#pragma once
#include "AP_Frsky_Backend.h"

/**
 * @brief MAVlite Wire Protocol Specification
 * 
 * @details Several SPort packets make up a single MAVlite message.
 * 
 *          **SPort Packet Structure:**
 *          A maximum of six relevant data bytes are present in each SPort packet.
 *          Each SPort packet starts with a sequence number, starting with zero.
 * 
 *          **Packet Fragmentation Strategy:**
 *          - If the sequence number is zero, the parser is reset
 *          - **First SPort packet** contains:
 *            * Byte 0: Sequence number (0)
 *            * Byte 1: Length (payload length in bytes, excludes checksum)
 *            * Byte 2: Message ID
 *            * Bytes 3-5: First payload bytes
 *          - **Subsequent SPort packets** contain:
 *            * Byte 0: Sequence number (increments for each packet)
 *            * Bytes 1-5: Additional payload bytes
 *          - **Final packet** contains:
 *            * Remaining payload bytes (if any)
 *            * Checksum byte (may occupy a packet alone with only sequence number)
 * 
 *          When sufficient payload bytes have been received (based on "len"),
 *          a single checksum byte arrives. Sometimes this checksum byte goes
 *          into an SPort packet all on its own, sharing space only with the
 *          sequence number.
 * 
 * @note All lengths are specified in bytes
 * @note Sequence numbers range from 0-255 and wrap around
 */

/**
 * @def MAVLITE_MAX_PAYLOAD_LEN
 * @brief Maximum payload size for a MAVlite message
 * 
 * @details Maximum length of the payload portion of a MAVlite message in bytes.
 *          This size is designed to accommodate 7 float parameters (28 bytes)
 *          plus a command ID (1 byte) and options field (2 bytes).
 * 
 *          The payload does not include the message framing overhead (length
 *          byte, message ID, sequence numbers, or checksum).
 * 
 * @note Value: 31 bytes
 * @note Units: bytes
 */
#define MAVLITE_MAX_PAYLOAD_LEN                 31 // 7 float params + cmd_id + options

/**
 * @def MAVLITE_MSG_SPORT_PACKETS_COUNT(LEN)
 * @brief Calculate number of SPort packets required to transport a MAVlite message
 * 
 * @details This macro computes how many SPort packets are needed to fragment
 *          a MAVlite message with a given payload length.
 * 
 *          **Segmentation Algorithm:**
 *          - First SPort packet carries: 1 length byte + 1 msgid byte + 3 payload bytes = 5 overhead + 3 data
 *          - Subsequent packets carry: 5 payload bytes each
 *          - Formula: 1 packet + ceil((LEN-3)/5) additional packets
 *          - Simplified: 1 + ceil((LEN-2)/5.0) packets total
 * 
 *          **Example:** For a 31-byte payload:
 *          - First packet: 3 bytes of payload
 *          - Remaining: 31-3 = 28 bytes
 *          - Additional packets: ceil(28/5) = 6 packets
 *          - Total: 1 + 6 = 7 SPort packets
 * 
 * @param[in] LEN Payload length in bytes (excluding framing overhead)
 * @return Number of SPort packets required (uint8_t)
 * 
 * @note Units: LEN in bytes, returns packet count
 * @note The calculation accounts for the len/msgid bytes in the first packet
 */
#define MAVLITE_MSG_SPORT_PACKETS_COUNT(LEN)    static_cast<uint8_t>(1 + ceilf((LEN-2)/5.0f)) // number of sport packets required to transport a message with LEN payload

/**
 * @def SPORT_PACKET_QUEUE_LENGTH
 * @brief Size of the SPort packet transmission queue
 * 
 * @details Defines the queue depth for buffering SPort packets awaiting transmission.
 *          The queue must be large enough to hold multiple maximum-size MAVlite
 *          messages to prevent packet loss during high telemetry loads.
 * 
 *          **Queue Calculation:**
 *          - Maximum messages buffered: 30 messages
 *          - Packets per max-size message: MAVLITE_MSG_SPORT_PACKETS_COUNT(31) = 7 packets
 *          - Total queue depth: 30 × 7 = 210 SPort packets
 * 
 *          This sizing ensures sufficient buffering for burst telemetry transmission
 *          while maintaining bounded memory usage.
 * 
 * @note Value: 210 entries (30 messages × 7 packets per message)
 * @note Units: packet count
 */
#define SPORT_PACKET_QUEUE_LENGTH               static_cast<uint8_t>(30U*MAVLITE_MSG_SPORT_PACKETS_COUNT(MAVLITE_MAX_PAYLOAD_LEN))
