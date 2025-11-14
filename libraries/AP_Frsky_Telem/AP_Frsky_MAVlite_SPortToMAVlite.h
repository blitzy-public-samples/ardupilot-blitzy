/**
 * @file AP_Frsky_MAVlite_SPortToMAVlite.h
 * @brief Bidirectional uplink converter: reconstructs MAVlite messages from fragmented SPort packets
 * 
 * @details This file implements the SPort-to-MAVlite decoder for FrSky telemetry uplink.
 *          MAVlite messages are fragmented across multiple SPort packets for transmission
 *          and must be reassembled on the receiver side. This decoder maintains state across
 *          multiple packet arrivals to reconstruct complete messages.
 * 
 * @see AP_Frsky_MAVlite.h for wire protocol description
 * @see AP_Frsky_MAVlite_MAVliteToSPort.h for inverse converter (downlink)
 */

#pragma once

#include "AP_Frsky_MAVlite_Message.h"
#include "AP_Frsky_SPort.h"

#include <stdint.h>

#if HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
/**
 * @class AP_Frsky_MAVlite_SPortToMAVlite
 * @brief Stateful decoder that accumulates multiple SPort packets into a single MAVlite message
 * 
 * @details An instance of this class decodes a stream of SPort packets into a
 *          MAVlite message (see AP_Frsky_MAVlite_Message.h). It is expected
 *          that the same rxmsg is passed into process() multiple times, each
 *          time with a new SPort packet. If a packet is successfully decoded
 *          then process() will return true and rxmsg can be used as a MAVlite
 *          message.
 * 
 *          The decoder performs:
 *          - Sequence number tracking for packet ordering
 *          - Checksum validation for message integrity
 *          - Stateful accumulation of message fragments
 * 
 *          Usage pattern:
 *          @code
 *          AP_Frsky_MAVlite_SPortToMAVlite decoder;
 *          AP_Frsky_MAVlite_Message msg;
 *          
 *          // Call repeatedly with new packets until complete message received
 *          if (decoder.process(msg, packet)) {
 *              // Complete message assembled and validated
 *              // msg now contains valid MAVlite message
 *          }
 *          @endcode
 * 
 * @note Only compiled when HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL is enabled
 * @note Not thread-safe - must be called from a single thread
 * 
 * @see AP_Frsky_MAVlite.h for description of MAVlite message encoding in SPort packets
 * @see AP_Frsky_MAVlite_MAVliteToSPort.h for inverse converter
 */
class AP_Frsky_MAVlite_SPortToMAVlite {
public:

    /**
     * @brief Decode next SPort packet fragment into MAVlite message
     * 
     * @details Processes a single SPort packet and accumulates it into the MAVlite message
     *          being assembled. This method must be called repeatedly with the same rxmsg
     *          and successive SPort packets until it returns true, indicating a complete
     *          message has been received and validated.
     * 
     *          The decoder tracks sequence numbers to ensure packets arrive in order and
     *          validates the final checksum before marking the message as complete.
     * 
     * @param[in,out] rxmsg Message being assembled (modified as packets arrive)
     * @param[in] packet Next SPort packet in sequence
     * 
     * @return true when complete message received and validated, false otherwise
     * 
     * @note Must call repeatedly with same rxmsg until true is returned
     * @warning Returns false and resets state if sequence error or checksum mismatch detected
     */
    bool process(AP_Frsky_MAVlite_Message &rxmsg,
                 const AP_Frsky_SPort::sport_packet_t &packet) WARN_IF_UNUSED;

private:

    /**
     * @brief Reset parser to IDLE state, clear sequence expectations
     * 
     * @details Called when starting fresh or after a parse error to reset
     *          the decoder to initial state. Clears sequence tracking and
     *          resets state machine to wait for first packet.
     */
    void reset();

    /**
     * @brief Next expected sequence number (0-255, wraps)
     * 
     * @details Tracks the sequence number of the next expected packet fragment.
     *          Sequence numbers increment with each packet and wrap at 256.
     */
    uint8_t expected_seq;
    
    /**
     * @brief Current offset in payload buffer (bytes)
     * 
     * @details Index into the payload buffer indicating where the next payload
     *          byte should be written. Increments as payload bytes are accumulated.
     */
    uint8_t payload_next_byte;

    /**
     * @enum State
     * @brief Parser state machine states for message assembly
     */
    enum class State : uint8_t {
        IDLE=0,             ///< @brief Awaiting first packet (sequence 0)
        ERROR,              ///< @brief Parse error detected, waiting for reset
        WANT_LEN,           ///< @brief Expecting length byte in first packet
        WANT_MSGID,         ///< @brief Expecting message ID byte in first packet
        WANT_PAYLOAD,       ///< @brief Accumulating payload bytes
        WANT_CHECKSUM,      ///< @brief Expecting final checksum byte
        MESSAGE_RECEIVED,   ///< @brief Complete valid message assembled
    };
    
    /**
     * @brief Current parser state
     */
    State parse_state = State::IDLE;

    /**
     * @brief Internal message buffer for assembly
     * 
     * @details Holds the MAVlite message as it is being assembled from
     *          incoming SPort packet fragments.
     */
    AP_Frsky_MAVlite_Message _rxmsg;
    
    /**
     * @brief Process single byte from packet
     * 
     * @details State machine implementation that processes one byte at a time,
     *          transitioning through states as message components are received.
     * 
     * @param byte The byte to process from the current packet
     */
    void parse(const uint8_t byte);

    /**
     * @brief Running checksum accumulator (int16_t to detect overflow)
     * 
     * @details Accumulates checksum as bytes are processed. Uses int16_t to
     *          allow overflow detection. Final value is compared against the
     *          checksum byte sent at end of packet.
     */
    int16_t checksum;
    
    /**
     * @brief Add byte to checksum calculation
     * 
     * @details Updates the running checksum with the provided byte value.
     *          Called for each byte that contributes to the message checksum.
     * 
     * @param c Byte to add to checksum
     */
    void update_checksum(const uint8_t c);
};
#endif