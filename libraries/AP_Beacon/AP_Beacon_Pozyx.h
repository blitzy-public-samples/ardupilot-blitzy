/**
 * @file AP_Beacon_Pozyx.h
 * @brief Pozyx beacon system integration for indoor positioning
 * 
 * @details This file implements support for the Pozyx indoor positioning system
 *          which uses Ultra-Wideband (UWB) ranging technology for precise indoor
 *          navigation. The Pozyx system consists of fixed anchor beacons and a
 *          tag device (mounted on the vehicle) that measures distances to multiple
 *          anchors to calculate position via trilateration.
 *          
 *          The implementation communicates with a Pozyx shield/Arduino combination
 *          via serial protocol, receiving beacon configuration, distance measurements,
 *          and computed position data.
 *          
 *          Protocol Overview:
 *          - Serial communication with Arduino/Pozyx shield
 *          - Message-based protocol with header, message ID, length, payload, and checksum
 *          - Three message types: beacon config, beacon distance, and position
 *          - XOR checksum validation for message integrity
 *          
 * @note Pozyx shields typically interface with Arduino boards that handle the
 *       UWB radio communication and relay data to ArduPilot via serial.
 * 
 * @see AP_Beacon_Backend for base beacon functionality
 * @see https://www.pozyx.io/ for Pozyx hardware information
 */

#pragma once

#include "AP_Beacon_Backend.h"

#if AP_BEACON_POZYX_ENABLED

/**
 * @brief Maximum message length for Pozyx serial protocol
 * @details Messages from Arduino/Pozyx shield are constrained to 20 bytes maximum.
 *          This includes: header (1 byte) + message ID (1 byte) + length (1 byte) +
 *          payload (up to 16 bytes) + checksum (1 byte).
 *          Buffer size constraint ensures efficient parsing without dynamic allocation.
 */
#define AP_BEACON_POZYX_MSG_LEN_MAX         20

/**
 * @brief Pozyx protocol message header byte
 * @details All valid Pozyx messages begin with 0x01 header byte.
 *          Parser uses this to synchronize with message stream and detect
 *          message boundaries. If header is not found, parser remains in
 *          WaitingForHeader state and discards bytes until valid header appears.
 */
#define AP_BEACON_POZYX_HEADER              0x01

/**
 * @brief Message ID for beacon configuration data
 * @details Message type 0x02 contains anchor beacon configuration including:
 *          - Beacon ID
 *          - Beacon position coordinates (X, Y, Z)
 *          Used during initialization to learn anchor beacon locations.
 * @note Coordinates received in Pozyx coordinate system, transformed to vehicle frame
 */
#define AP_BEACON_POZYX_MSGID_BEACON_CONFIG 0x02

/**
 * @brief Message ID for individual beacon distance measurement
 * @details Message type 0x03 contains range measurement from vehicle to single beacon:
 *          - Beacon ID
 *          - Distance in meters
 *          Multiple distance messages (one per visible beacon) used for position estimation.
 * @note Distance measurements are UWB time-of-flight based with centimeter-level precision
 */
#define AP_BEACON_POZYX_MSGID_BEACON_DIST   0x03

/**
 * @brief Message ID for computed vehicle position
 * @details Message type 0x04 contains calculated vehicle position from Pozyx system:
 *          - X, Y, Z coordinates in meters
 *          - Position computed by Pozyx firmware via trilateration
 *          ArduPilot uses this position as beacon-based navigation input to EKF.
 * @note Position is in Pozyx coordinate frame and undergoes transformation to vehicle frame
 */
#define AP_BEACON_POZYX_MSGID_POSITION      0x04

/**
 * @brief Maximum sanity check distance for beacon measurements
 * @details Beacon distances and positions exceeding 200 meters from origin are rejected
 *          as invalid. This prevents:
 *          - Corruption from causing huge position jumps
 *          - Multipath or NLOS errors producing unrealistic ranges
 *          - Communication errors propagating to navigation system
 * @warning Measurements beyond this threshold are silently discarded. Ensure beacon
 *          layout and origin are within 200m operational area.
 * @note Value of 200.0f meters (float) used for comparison with distance measurements
 */
#define AP_BEACON_DISTANCE_MAX              200.0f

/**
 * @class AP_Beacon_Pozyx
 * @brief Pozyx UWB beacon positioning system driver
 * 
 * @details This class implements the ArduPilot driver for Pozyx indoor positioning
 *          system based on Ultra-Wideband (UWB) ranging technology. It communicates
 *          with an Arduino/Pozyx shield combination via serial protocol to receive
 *          beacon configuration, distance measurements, and position estimates.
 *          
 *          Protocol Implementation:
 *          The Pozyx serial protocol is a simple packet-based format:
 *          - Byte 0: Header (0x01)
 *          - Byte 1: Message ID (0x02=config, 0x03=distance, 0x04=position)
 *          - Byte 2: Payload length (N bytes)
 *          - Bytes 3 to 3+N-1: Payload data
 *          - Byte 3+N: XOR checksum of all bytes from header to last payload byte
 *          
 *          Parse State Machine:
 *          The driver uses a four-state parser to handle byte-by-byte serial data:
 *          1. WaitingForHeader: Searching for 0x01 header byte
 *          2. WaitingForMsgId: Reading message type (0x02/0x03/0x04)
 *          3. WaitingForLen: Reading payload length byte
 *          4. WaitingForContents: Accumulating payload + checksum bytes
 *          
 *          On checksum validation, parser extracts beacon data and resets to WaitingForHeader.
 *          Invalid checksums or malformed messages cause parser reset to resynchronize.
 *          
 *          Message Processing:
 *          - BEACON_CONFIG (0x02): Stores anchor beacon positions in beacon database
 *          - BEACON_DIST (0x03): Updates range measurement for specific beacon ID
 *          - POSITION (0x04): Provides vehicle position estimate to navigation system
 *          
 *          Coordinate Transformation:
 *          Pozyx coordinate system is transformed to ArduPilot vehicle frame conventions:
 *          - Pozyx coordinates scaled appropriately for vehicle position reporting
 *          - Distance measurements remain in meters for EKF fusion
 *          
 *          Data Validation:
 *          - All distances sanity-checked against AP_BEACON_DISTANCE_MAX (200m)
 *          - Checksum validation ensures message integrity
 *          - Timeout detection marks sensor unhealthy if no updates received
 *          
 * @note Typically used with Arduino Uno or similar running Pozyx shield firmware
 *       that handles UWB radio and formats data for ArduPilot consumption.
 * 
 * @warning Distance measurements are only as accurate as beacon anchor positions.
 *          Carefully survey and configure beacon locations for reliable positioning.
 * 
 * @see AP_Beacon_Backend for base class functionality
 * @see AP_Beacon for beacon subsystem integration with EKF
 */
class AP_Beacon_Pozyx : public AP_Beacon_Backend
{

public:
    /**
     * @brief Constructor for Pozyx beacon driver
     * @details Inherits constructor from AP_Beacon_Backend base class.
     *          Initializes parse state machine to WaitingForHeader state.
     */
    using AP_Beacon_Backend::AP_Beacon_Backend;

    /**
     * @brief Check if Pozyx beacon system is healthy and providing data
     * 
     * @details Determines sensor health based on recent data reception.
     *          Sensor is considered healthy if position or distance updates
     *          have been received within the timeout threshold.
     *          
     * @return true if sensor is receiving valid data and operational
     * @return false if no recent updates or communication timeout detected
     * 
     * @note Health check is based on last_update_ms timestamp compared to current time.
     *       Typical timeout threshold is several seconds to allow for message intervals.
     */
    bool healthy() override;

    /**
     * @brief Main update function called periodically to process Pozyx data
     * 
     * @details This method is called by the beacon subsystem scheduler to:
     *          1. Read available bytes from serial port connected to Arduino/Pozyx
     *          2. Feed bytes to parse state machine (parse_buffer)
     *          3. Process complete messages as they are validated
     *          4. Update beacon database with configuration/distance/position data
     *          
     *          Byte-by-byte Processing:
     *          Each available serial byte is read and processed through the state machine.
     *          Parse buffer (linebuf) accumulates message payload until complete message
     *          with valid checksum is received.
     *          
     *          Message Handling:
     *          - Config messages: Store beacon anchor positions
     *          - Distance messages: Update ranging data for EKF fusion
     *          - Position messages: Provide position estimate to navigation system
     *          
     * @note Called at scheduler rate (typically 10-50 Hz) to ensure responsive
     *       processing of serial data without blocking.
     * 
     * @warning Do not call directly - invoked by beacon subsystem scheduler
     */
    void update() override;

private:

    /**
     * @enum ParseState
     * @brief State machine states for Pozyx serial protocol parser
     * 
     * @details The parser implements a simple four-state machine to handle
     *          byte-by-byte reception and validation of Pozyx protocol messages:
     *          
     *          State Transitions:
     *          WaitingForHeader -> WaitingForMsgId (on valid 0x01 header)
     *          WaitingForMsgId -> WaitingForLen (on valid message ID 0x02/0x03/0x04)
     *          WaitingForLen -> WaitingForContents (on length byte, validates len <= max)
     *          WaitingForContents -> WaitingForHeader (on checksum validation or error)
     *          
     *          Error Handling:
     *          Any invalid byte or checksum failure returns parser to WaitingForHeader
     *          to resynchronize with message stream.
     */
    enum ParseState{
        ParseState_WaitingForHeader = 0,    ///< Searching for 0x01 header byte to start message
        ParseState_WaitingForMsgId = 1,     ///< Reading message ID byte (0x02/0x03/0x04)
        ParseState_WaitingForLen = 2,       ///< Reading payload length byte
        ParseState_WaitingForContents = 3   ///< Accumulating payload data and checksum byte
    } parse_state;                          ///< Current parser state

    /**
     * @brief Process accumulated message buffer and validate checksum
     * 
     * @details Called when complete message has been received (payload + checksum).
     *          Performs XOR checksum validation across entire message including header.
     *          
     *          Checksum Calculation:
     *          XOR checksum computed over: header + msgID + length + payload bytes.
     *          Received checksum byte compared against computed value.
     *          
     *          Message Dispatch:
     *          On valid checksum, message is dispatched based on parse_msg_id:
     *          - 0x02: Parse beacon configuration (ID + X,Y,Z position)
     *          - 0x03: Parse distance measurement (beacon ID + distance in meters)
     *          - 0x04: Parse vehicle position (X,Y,Z coordinates in meters)
     *          
     *          Data Validation:
     *          - All distance measurements checked against AP_BEACON_DISTANCE_MAX (200m)
     *          - Position coordinates checked for sanity
     *          - Invalid data causes message rejection and parser reset
     *          
     * @note This method is private and called only from update() when state machine
     *       reaches end of message reception.
     */
    void parse_buffer();

    uint8_t parse_msg_id;       ///< Message ID of currently parsing message (0x02/0x03/0x04)
    uint8_t parse_msg_len;      ///< Expected payload length for current message (bytes)

    /**
     * @brief Message accumulation buffer for payload and checksum
     * @details Holds received message bytes during WaitingForContents state.
     *          Buffer size limited to AP_BEACON_POZYX_MSG_LEN_MAX (20 bytes).
     *          Buffer contents validated via XOR checksum before processing.
     */
    uint8_t linebuf[AP_BEACON_POZYX_MSG_LEN_MAX];
    
    /**
     * @brief Current number of bytes accumulated in linebuf
     * @details Tracks bytes received during WaitingForContents state.
     *          Reset to 0 when parser returns to WaitingForHeader state.
     *          Compared against parse_msg_len + 1 (payload + checksum) to detect
     *          message completion.
     */
    uint8_t linebuf_len = 0;
    
    /**
     * @brief Timestamp of last successful message reception
     * @details Updated when valid beacon configuration, distance, or position message
     *          is successfully parsed and validated. Used by healthy() method to
     *          determine if sensor is actively providing data or has timed out.
     * @note Time stored in milliseconds since system boot (AP_HAL::millis())
     */
    uint32_t last_update_ms = 0;
};

#endif  // AP_BEACON_POZYX_ENABLED
