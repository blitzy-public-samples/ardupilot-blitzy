/**
 * @file AP_Beacon_Nooploop.h
 * @brief Nooploop LinkTrack beacon positioning system integration
 * 
 * @details This file implements support for the Nooploop LinkTrack UWB (Ultra-Wideband)
 *          beacon positioning system. The LinkTrack system consists of fixed anchor nodes
 *          and mobile tag nodes that use UWB ranging to determine 3D position.
 *          
 *          The implementation processes serial data from the LinkTrack tag using a dual
 *          protocol system with two different message types and header bytes.
 * 
 * @note LinkTrack terminology: "tag" = mobile unit on vehicle, "anchor" = fixed reference beacon
 * 
 * Source: libraries/AP_Beacon/AP_Beacon_Nooploop.h
 */

#pragma once

#include "AP_Beacon_Backend.h"

#if AP_BEACON_NOOPLOOP_ENABLED

/**
 * @brief Maximum message buffer size for Nooploop protocol messages
 * 
 * @details Buffer sized to accommodate the largest expected message from the
 *          LinkTrack tag. Messages exceeding this size will be truncated.
 */
#define NOOPLOOP_MSG_BUF_MAX      256

/**
 * @class AP_Beacon_Nooploop
 * @brief Nooploop LinkTrack UWB beacon positioning system backend
 * 
 * @details This class implements the Nooploop LinkTrack protocol for beacon-based
 *          positioning using Ultra-Wideband (UWB) ranging technology. The system
 *          provides 3D position estimates by ranging to multiple fixed anchor beacons.
 *          
 *          Protocol Features:
 *          - Dual header system: 0x55 (NODE_FRAME2) for position/distance data,
 *            0x54 (SETTING_FRAME0) for anchor configuration
 *          - Robust byte-level state machine with separate parsers for each message type
 *          - CRC checksum validation for message integrity
 *          - 24-bit signed coordinate encoding in protocol
 *          - Coordinate transformation from ENU (East-North-Up) to NED (North-East-Down)
 *          
 *          Operational Flow:
 *          1. Request anchor configuration from tag via request_setting()
 *          2. Parse incoming SETTING_FRAME0 messages to learn anchor positions
 *          3. Parse NODE_FRAME2 messages for tag position and ranging distances
 *          4. Transform coordinates from tag's ENU frame to vehicle NED frame
 *          5. Provide position and distance data to EKF for sensor fusion
 *          
 * @note The tag must be configured to output both NODE_FRAME2 and SETTING_FRAME0 messages
 * @warning Accurate anchor placement and configuration is critical for position accuracy
 * 
 * Source: libraries/AP_Beacon/AP_Beacon_Nooploop.h
 */
class AP_Beacon_Nooploop : public AP_Beacon_Backend
{

public:
    /**
     * @brief Constructor - inherits from AP_Beacon_Backend
     * 
     * @details Uses the parent class constructor for initialization. The backend
     *          is registered with the AP_Beacon library and begins receiving
     *          serial data through the configured UART port.
     */
    using AP_Beacon_Backend::AP_Beacon_Backend;

    /**
     * @brief Check if beacon system is receiving valid data
     * 
     * @details Reports health status based on recent data reception from the LinkTrack tag.
     *          The sensor is considered healthy if valid messages have been received
     *          within the timeout period.
     * 
     * @return true if valid data received recently (sensor operational)
     * @return false if no recent data or communication failure
     * 
     * @note Called by the EKF to determine if beacon data should be trusted for fusion
     * @see _last_update_ms for timestamp of most recent valid message
     */
    bool healthy() override;

    /**
     * @brief Update beacon state by processing available serial data
     * 
     * @details Main update method called periodically by the scheduler. This function:
     *          1. Reads available bytes from the serial port
     *          2. Feeds each byte to the protocol parser state machine
     *          3. Processes complete messages (NODE_FRAME2 or SETTING_FRAME0)
     *          4. Requests anchor configuration if not yet received
     *          5. Throttles configuration requests to avoid flooding the tag
     *          
     *          The method handles both message types:
     *          - NODE_FRAME2 (0x55): Tag position and ranging distances
     *          - SETTING_FRAME0 (0x54): Anchor beacon positions
     * 
     * @note Called at the beacon update rate (typically 10 Hz)
     * @note Non-blocking - only processes available buffered data
     * 
     * @warning Ensure serial port is properly configured before calling
     * 
     * @see parse_byte() for state machine implementation
     * @see request_setting() for anchor configuration solicitation
     */
    void update() override;

private:
    /**
     * @enum MsgType
     * @brief Nooploop protocol message type identification
     * 
     * @details The LinkTrack protocol uses two distinct message types with different
     *          header bytes (0x55 and 0x54) and payload structures. The state machine
     *          differentiates between these types to route messages to the correct parser.
     */
    enum class MsgType : uint8_t {
        INVALID = 0,        ///< Invalid or incomplete message
        NODE_FRAME2,        ///< Position and ranging message (header: 0x55)
        SETTING_FRAME0      ///< Anchor configuration message (header: 0x54)
    };

    /**
     * @brief Process one byte from serial stream through protocol state machine
     * 
     * @details Implements a robust byte-level parser with separate state paths for the
     *          two Nooploop message types. The state machine sequence:
     *          
     *          HEADER → (detect 0x55 or 0x54) → H55_FUNCTION_MARK or H54_FUNCTION_MARK
     *               → LEN_L (low byte) → LEN_H (high byte)
     *               → NF2_PAYLOAD or SF0_PAYLOAD (collect message bytes)
     *               → Validate CRC and return message type
     *          
     *          The parser accumulates bytes in _msgbuf and validates message integrity
     *          via CRC checksum before returning a valid message type.
     * 
     * @param[in] b Single byte received from serial port
     * 
     * @return MsgType::NODE_FRAME2 if complete valid position message received
     * @return MsgType::SETTING_FRAME0 if complete valid configuration message received
     * @return MsgType::INVALID if message incomplete, corrupt, or failed CRC
     * 
     * @note Message contents stored in _msgbuf for subsequent parsing
     * @warning CRC validation is critical for preventing corrupt data from affecting position
     * 
     * @see ParseState enum for detailed state machine definition
     * @see _msgbuf for message storage buffer
     */
    MsgType parse_byte(uint8_t b);

    /**
     * @brief Request anchor configuration from LinkTrack tag
     * 
     * @details Sends a SETTING_FRAME0 request message to the tag. The tag responds with
     *          a SETTING_FRAME0 message containing the positions of all configured anchor
     *          beacons. This is typically called once at startup or when anchor positions
     *          are not yet known.
     *          
     *          Request throttling prevents flooding the tag with repeated configuration
     *          requests. Requests are rate-limited via _last_request_setting_ms timestamp.
     * 
     * @note The tag must be configured to respond to setting frame requests
     * @note Response time varies; may require multiple update cycles to receive reply
     * 
     * @see _last_request_setting_ms for request throttling timestamp
     * @see _anchor_pos_avail flag indicating if configuration has been received
     * @see parse_setting_frame0() for processing the tag's response
     */
    void request_setting();

    /**
     * @brief Parse NODE_FRAME2 message to extract tag position and ranging distances
     * 
     * @details Decodes the position and ranging message from the LinkTrack tag. This message
     *          contains:
     *          - Tag 3D position in ENU coordinates (24-bit signed integers, millimeters)
     *          - Distance measurements to each visible anchor (millimeters)
     *          - Node ID and status information
     *          
     *          Coordinate Transformation:
     *          The tag reports position in ENU (East-North-Up) frame, which is transformed
     *          to the vehicle's NED (North-East-Down) frame:
     *          - NED North = ENU North
     *          - NED East = ENU East
     *          - NED Down = -ENU Up (sign inversion)
     *          
     *          The parsed position and distances are stored in the backend's position and
     *          distance arrays for consumption by the EKF.
     * 
     * @note All coordinates reconstructed from 24-bit signed values to full precision
     * @note Units: Input in millimeters, output in meters after conversion
     * @note Called only after parse_byte() returns MsgType::NODE_FRAME2
     * 
     * @warning Coordinate transformation must be correct or vehicle navigation will fail
     * 
     * @see _msgbuf contains the complete message payload
     */
    void parse_node_frame2();

    /**
     * @brief Parse SETTING_FRAME0 message to extract anchor beacon positions
     * 
     * @details Decodes anchor configuration message from the tag. This message contains
     *          the 3D positions of all anchor beacons in the system, which are required
     *          for position calculation.
     *          
     *          Anchor Position Storage:
     *          - Anchor positions stored in backend's beacon position array
     *          - Positions in ENU frame, transformed to NED frame
     *          - Each anchor identified by its node ID
     *          
     *          The _anchor_pos_avail flag is set once valid anchor positions are received,
     *          preventing repeated configuration requests.
     * 
     * @note Anchor positions typically requested once at initialization
     * @note Units: Positions in meters after conversion from protocol units
     * @note Called only after parse_byte() returns MsgType::SETTING_FRAME0
     * 
     * @see _anchor_pos_avail flag set to true after successful parsing
     * @see request_setting() for soliciting this message from the tag
     * @see _msgbuf contains the complete message payload
     */
    void parse_setting_frame0();

    /**
     * @enum ParseState
     * @brief State machine states for Nooploop protocol byte-level parsing
     * 
     * @details The parser implements a robust state machine with separate paths for two
     *          message types (NODE_FRAME2 with 0x55 header, SETTING_FRAME0 with 0x54 header).
     *          
     *          Parse Sequence:
     *          1. HEADER: Wait for 0x55 or 0x54 header byte
     *          2a. H55_FUNCTION_MARK: Process function mark for NODE_FRAME2 (0x55 path)
     *          2b. H54_FUNCTION_MARK: Process function mark for SETTING_FRAME0 (0x54 path)
     *          3. LEN_L: Receive low byte of 16-bit message length
     *          4. LEN_H: Receive high byte of 16-bit message length
     *          5a. NF2_PAYLOAD: Collect NODE_FRAME2 message bytes
     *          5b. SF0_PAYLOAD: Collect SETTING_FRAME0 message bytes
     *          6. Validate CRC and return to HEADER
     *          
     *          State machine resets to HEADER on:
     *          - Buffer overflow (_msg_len >= NOOPLOOP_MSG_BUF_MAX)
     *          - CRC validation failure
     *          - Invalid function mark or length field
     *          - Successful message completion
     * 
     * @note Separate payload states (NF2_PAYLOAD vs SF0_PAYLOAD) enable different
     *       handling for the two message formats
     * 
     * @warning State machine must handle all error conditions to prevent deadlock
     */
    enum class ParseState : uint8_t {
        HEADER = 0,         ///< Waiting for header byte (0x55 or 0x54)
        H55_FUNCTION_MARK,  ///< Waiting for function mark (NODE_FRAME2 path)
        H54_FUNCTION_MARK,  ///< Waiting for function mark (SETTING_FRAME0 path)
        LEN_L,              ///< Waiting for low byte of 16-bit length field
        LEN_H,              ///< Waiting for high byte of 16-bit length field
        NF2_PAYLOAD,        ///< Receiving NODE_FRAME2 payload bytes
        SF0_PAYLOAD,        ///< Receiving SETTING_FRAME0 payload bytes
    } _state = ParseState::HEADER;

    /**
     * @name Protocol Parser State Variables
     * @{
     */
    
    /**
     * @brief Message buffer for accumulating received protocol bytes
     * 
     * @details Stores incoming message bytes during parsing. Buffer is sized to accommodate
     *          the largest expected Nooploop protocol message. Messages exceeding
     *          NOOPLOOP_MSG_BUF_MAX bytes will trigger a parser reset to prevent overflow.
     * 
     * @note Buffer reused for both NODE_FRAME2 and SETTING_FRAME0 messages
     * @warning Buffer overflow protection implemented in parse_byte() state machine
     */
    uint8_t _msgbuf[NOOPLOOP_MSG_BUF_MAX];
    
    /**
     * @brief Current number of bytes received in the active message
     * 
     * @details Increments as each byte is received. May exceed NOOPLOOP_MSG_BUF_MAX
     *          if the sender transmits an oversized message, in which case parsing
     *          is aborted and state machine resets.
     * 
     * @note Reset to 0 at the start of each new message (HEADER state)
     */
    uint16_t _msg_len;
    
    /**
     * @brief Message length field extracted from protocol header
     * 
     * @details The Nooploop protocol includes a 16-bit length field (LEN_L and LEN_H bytes)
     *          indicating the expected message size. This is compared against _msg_len
     *          to detect message completion.
     * 
     * @note Value in bytes, includes all payload bytes but not the header/length fields
     */
    uint16_t _frame_len;
    
    /**
     * @brief Calculated CRC checksum for message integrity validation
     * 
     * @details Computed incrementally as message bytes are received. After complete
     *          message reception, _crc_expected is compared against the CRC byte
     *          transmitted at the end of the message. Mismatch indicates corruption
     *          and the message is discarded.
     * 
     * @warning CRC validation is critical for flight safety - corrupt position data
     *          could cause vehicle navigation failures
     * 
     * @note CRC algorithm specific to Nooploop protocol
     */
    uint8_t _crc_expected;
    
    /**
     * @brief Timestamp of most recent valid message reception
     * 
     * @details Updated whenever a complete, CRC-valid message is successfully parsed.
     *          Used by healthy() to determine if the beacon system is actively
     *          providing data. Staleness threshold typically 1-2 seconds.
     * 
     * @note Units: milliseconds (system uptime from AP_HAL::millis())
     * @see healthy() for timeout threshold implementation
     */
    uint32_t _last_update_ms;
    
    /**
     * @brief Flag indicating whether anchor positions have been received
     * 
     * @details Set to true after successfully parsing a SETTING_FRAME0 message containing
     *          anchor beacon positions. Prevents repeated configuration requests once
     *          anchor data is known.
     *          
     *          Anchor positions are required before tag position measurements can be
     *          used for navigation. The EKF will not fuse beacon data until anchors
     *          are properly configured.
     * 
     * @note Initialized to false; set true by parse_setting_frame0()
     * @see request_setting() checks this flag to avoid redundant requests
     */
    bool _anchor_pos_avail;
    
    /**
     * @brief Timestamp of most recent anchor configuration request
     * 
     * @details Tracks when request_setting() last sent a configuration solicitation
     *          to the tag. Used to throttle requests and prevent flooding the tag's
     *          serial interface with repeated configuration queries.
     *          
     *          Typical throttle interval: 1-5 seconds between requests.
     * 
     * @note Units: milliseconds (system uptime from AP_HAL::millis())
     * @see request_setting() for throttling logic implementation
     */
    uint32_t _last_request_setting_ms;
    
    /** @} */ // end of Protocol Parser State Variables
};

#endif  // AP_BEACON_NOOPLOOP_ENABLED
