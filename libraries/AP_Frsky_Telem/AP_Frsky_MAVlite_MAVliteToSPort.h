/**
 * @file AP_Frsky_MAVlite_MAVliteToSPort.h
 * @brief Bidirectional downlink converter: fragments MAVlite messages into multiple SPort packets
 * 
 * This file provides the encoder that converts complete MAVlite messages into
 * a series of FrSky SPort packets for transmission over the bidirectional telemetry link.
 * 
 * @note Only compiled when HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL is enabled
 * @see AP_Frsky_MAVlite.h for wire protocol and fragmentation algorithm
 * @see AP_Frsky_MAVlite_SPortToMAVlite.h for inverse decoder
 */

#pragma once

#include "AP_Frsky_config.h"

#if HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL

#include "AP_Frsky_MAVlite_Message.h"
#include "AP_Frsky_SPort.h"

#include <AP_HAL/utility/RingBuffer.h>

#include <stdint.h>

/**
 * @class AP_Frsky_MAVlite_MAVliteToSPort
 * @brief Stateful encoder segmenting single MAVlite message into multiple SPort packet fragments
 * 
 * @details An instance of this class encodes a complete MAVlite message into several
 *          FrSky SPort packets and pushes them onto a supplied thread-safe queue.
 *          
 *          The encoder maintains state across multiple process_byte() calls to:
 *          - Generate sequence numbers (0-255) for packet ordering
 *          - Compute checksum across the entire message
 *          - Fragment payload data into multiple packets
 *          - Push packets into thread-safe queue (ObjectBuffer_TS)
 *          
 *          Packet structure:
 *          - First packet: sequence(1) + len(1) + msgid(1) + payload(up to 3 bytes)
 *          - Subsequent packets: sequence(1) + payload(up to 5 bytes)
 *          
 *          The process() method will return false if there is insufficient room in the
 *          queue for all SPort packets, ensuring atomic operation (no partial message enqueue).
 * 
 * @note Typical message generates 1-7 packets depending on payload length
 * @note First packet contains sequence(1)+len(1)+msgid(1)+payload(up to 3)
 * @note Subsequent packets contain sequence(1)+payload(up to 5)
 * 
 * @warning Queue must have SPORT_PACKET_QUEUE_LENGTH entries to avoid dropped messages
 * @warning Only compiled when HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL is enabled
 * 
 * @see AP_Frsky_MAVlite.h for wire protocol and fragmentation algorithm
 * @see AP_Frsky_MAVlite_SPortToMAVlite.h for inverse decoder
 * @see MAVLITE_MSG_SPORT_PACKETS_COUNT for maximum packet count calculation
 */
class AP_Frsky_MAVlite_MAVliteToSPort {
public:

    /**
     * @brief Fragment MAVlite message into SPort packets and enqueue
     * 
     * @details Converts a complete MAVlite message into a series of FrSky SPort
     *          packets and inserts them into the thread-safe queue. The message
     *          is fragmented across multiple packets with sequence numbers for
     *          proper reconstruction by the receiver.
     *          
     *          The encoder processes the message through states:
     *          WANT_LEN → WANT_MSGID → WANT_PAYLOAD → WANT_CHECKSUM → DONE
     *          
     *          Each state emits bytes into packets with automatic packet boundary
     *          management when packets reach maximum data capacity (6 bytes).
     * 
     * @param[in,out] queue Thread-safe queue to receive generated SPort packets (ObjectBuffer_TS)
     * @param[in]     msg   Complete MAVlite message to transmit
     * 
     * @return true if all packets successfully enqueued, false if insufficient queue space
     * 
     * @note Typical message generates 1-7 packets depending on payload length (see MAVLITE_MSG_SPORT_PACKETS_COUNT)
     * @note Function is atomic: returns false without partial enqueue if queue lacks space
     * 
     * @warning Returns false if queue full - no partial message fragments are enqueued
     * @warning Caller must ensure queue has sufficient space for all message packets
     * 
     * @see AP_Frsky_MAVlite_Message for message structure
     * @see ObjectBuffer_TS for thread-safe queue implementation
     */
    bool process(ObjectBuffer_TS<AP_Frsky_SPort::sport_packet_t> &queue,
                 const AP_Frsky_MAVlite_Message &msg) WARN_IF_UNUSED;

private:

    /**
     * @enum State
     * @brief Encoder state machine for MAVlite message fragmentation
     */
    enum class State : uint8_t {
        WANT_LEN,       ///< @brief Emitting length byte in first packet
        WANT_MSGID,     ///< @brief Emitting message ID byte in first packet
        WANT_PAYLOAD,   ///< @brief Emitting payload bytes across packets
        WANT_CHECKSUM,  ///< @brief Emitting final checksum byte
        DONE,           ///< @brief All packets generated
    };
    
    State state = State::WANT_LEN;  ///< @brief Current encoder state

    /**
     * @brief Reset encoder to WANT_LEN state for next message
     * 
     * @details Resets the state machine to initial state, ready to encode
     *          a new MAVlite message. Called after completing a message or
     *          on initialization.
     */
    void reset();

    /**
     * @brief Add byte to current packet, start new packet if full
     * 
     * @details Inserts a byte into the current packet being assembled. If the
     *          packet data area is full (6 bytes), pushes the completed packet
     *          onto the queue and starts a new packet with incremented sequence
     *          number.
     * 
     * @param[in]     byte  Byte to add to packet
     * @param[in,out] queue Queue to receive completed packets
     */
    void process_byte(uint8_t byte, ObjectBuffer_TS<AP_Frsky_SPort::sport_packet_t> &queue);

    AP_Frsky_SPort::sport_packet_t packet {};  ///< @brief Current SPort packet being assembled (sport_packet_t)
    uint8_t packet_offs = 0;                   ///< @brief Byte offset within current packet (0-5 for data bytes)

    uint8_t next_seq = 0;      ///< @brief Sequence number for next packet (uint8_t, 0-255)
    uint8_t payload_count = 0; ///< @brief Payload bytes emitted so far (uint8_t)
    uint8_t payload_len;       ///< @brief Total payload length from message (uint8_t, bytes)

    int16_t checksum;  ///< @brief Running checksum accumulator (int16_t)
    
    /**
     * @brief Add byte to checksum calculation
     * 
     * @details Updates the running checksum accumulator with the provided byte.
     *          The checksum is computed across the entire message (length, msgid,
     *          and payload) and sent as the final byte of the fragmented message.
     * 
     * @param[in] c Byte to add to checksum
     */
    void update_checksum(const uint8_t c);
};

#endif  // HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
