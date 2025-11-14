/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Code by Andrew Tridgell and Siddharth Bharat Purohit
 */

/**
 * @file AP_RCProtocol_SUMD.h
 * @brief Graupner SUMD (Serial Uni Master Decoder) RC protocol decoder
 * 
 * @details This file implements the SUMD protocol decoder for Graupner HoTT receivers
 *          and compatible systems. SUMD is a unidirectional serial protocol used to transmit
 *          RC channel data from Graupner/HoTT receivers to flight controllers.
 *          
 *          Protocol Specifications:
 *          - Baudrate: 115200 bps, 8N1 (8 data bits, no parity, 1 stop bit)
 *          - Frame Format:
 *            * Header: 0xA8 (SUMD identifier byte)
 *            * Status: 1 byte (0x01 = valid data, 0x00 = SUMH mode, 0x81 = failsafe)
 *            * Channel Count: 1 byte (number of channels in frame)
 *            * Channel Data: 2 bytes per channel (16-bit big-endian, 12-bit effective resolution)
 *            * CRC16: 2 bytes (16-bit CRC for frame integrity validation)
 *          
 *          Channel Data Format:
 *          - 12-bit resolution (values 0-4095)
 *          - Center value: 2048 (neutral stick position)
 *          - Typical range: 1000-3000 (approximately)
 *          - Transmitted as 16-bit big-endian (high byte first)
 *          
 *          Protocol Features:
 *          - Support for up to 16 channels (some variants support up to 32)
 *          - Failsafe status indication in status byte
 *          - CRC16 checksum for robust error detection
 *          - Signal quality flags in status byte
 *          
 *          Compatible Hardware:
 *          - Graupner HoTT receivers (GR-12, GR-16, GR-24, etc.)
 *          - Graupner mx-series transmitters
 *          - Third-party HoTT-compatible receivers
 *          
 * @note SUMD protocol is unidirectional (receiver to flight controller only)
 * @note Some receivers may use SUMH protocol (header 0xA9) for telemetry mode
 * 
 * @see AP_RCProtocol_Backend for base protocol implementation
 * 
 * @author Andrew Tridgell and Siddharth Bharat Purohit
 * @copyright Copyright (c) ArduPilot Development Team
 */

#pragma once

#include "AP_RCProtocol_config.h"

#if AP_RCPROTOCOL_SUMD_ENABLED

#include "AP_RCProtocol_Backend.h"
#include "SoftSerial.h"

#define SUMD_MAX_CHANNELS	32  ///< Maximum supported channels in SUMD protocol (hardware-dependent, typically 16)
#define SUMD_FRAME_MAXLEN   40  ///< Maximum SUMD frame length in bytes (header + status + length + 32ch*2bytes + CRC16)

/**
 * @class AP_RCProtocol_SUMD
 * @brief RC protocol backend for decoding Graupner SUMD serial frames
 * 
 * @details This class implements a state machine decoder for the Graupner SUMD protocol,
 *          processing incoming serial bytes at 115200 bps and extracting RC channel data.
 *          The decoder validates frame structure, status, and CRC16 checksum before
 *          accepting channel values.
 *          
 *          Decoding State Machine:
 *          1. UNSYNCED - Waiting for 0xA8 header byte
 *          2. GOT_HEADER - Header received, expecting status byte
 *          3. GOT_STATE - Status received, expecting channel count
 *          4. GOT_LEN - Channel count received, reading channel data
 *          5. GOT_DATA - All channel data received, validating CRC16
 *          6. GOT_CRC - Frame complete and validated
 *          
 *          Frame Validation:
 *          - Header byte must be 0xA8 (SUMD) or 0xA9 (SUMH)
 *          - Channel count must be reasonable (1-32 channels)
 *          - CRC16 must match calculated checksum over entire frame
 *          - Status byte indicates failsafe condition (0x81)
 *          
 *          Failsafe Handling:
 *          - Status byte 0x01: Valid live data from transmitter
 *          - Status byte 0x00: SUMH mode (bidirectional telemetry mode)
 *          - Status byte 0x81: Failsafe mode (RC link lost)
 *          
 *          CRC16 Algorithm:
 *          - Standard CRC16-CCITT algorithm
 *          - Calculated over header, status, length, and all channel data bytes
 *          - Transmitted as 16-bit big-endian (high byte, then low byte)
 *          
 * @note This decoder can handle both SUMD (0xA8) and SUMH (0xA9) frame types
 * @note Typical processing rate: 20-50 Hz depending on channel count
 * 
 * @warning CRC16 validation is critical for flight safety - corrupted frames must be rejected
 *          to prevent erroneous control inputs reaching the flight controller
 * 
 * @see AP_RCProtocol_Backend for base protocol interface
 * @see ReceiverFcPacketHoTT for frame structure definition
 */
class AP_RCProtocol_SUMD : public AP_RCProtocol_Backend {
public:
    /**
     * @brief Constructor for SUMD protocol decoder backend
     * 
     * @param[in] _frontend Reference to the AP_RCProtocol frontend manager
     * 
     * @note Initializes decoder state machine to UNSYNCED state
     */
    AP_RCProtocol_SUMD(AP_RCProtocol &_frontend) : AP_RCProtocol_Backend(_frontend) {}
    
    /**
     * @brief Process pulse-width timing from hardware capture (not used for SUMD)
     * 
     * @details SUMD is a serial byte-based protocol and does not use pulse-width timing.
     *          This method is provided to satisfy the backend interface but performs
     *          no operation for SUMD protocol.
     * 
     * @param[in] width_s0 First pulse width in microseconds (unused)
     * @param[in] width_s1 Second pulse width in microseconds (unused)
     * 
     * @note SUMD protocol uses process_byte() instead of process_pulse()
     */
    void process_pulse(uint32_t width_s0, uint32_t width_s1) override;
    
    /**
     * @brief Process incoming serial byte for SUMD frame decoding
     * 
     * @details Feeds serial bytes into the SUMD state machine decoder. Each byte is
     *          processed according to the current decode state, progressively building
     *          a complete frame. When a valid frame with correct CRC16 is received,
     *          channel values are extracted and passed to the frontend.
     *          
     *          Processing includes:
     *          - Header detection (0xA8 for SUMD, 0xA9 for SUMH)
     *          - Status byte validation (failsafe detection)
     *          - Channel count validation (1-32 channels)
     *          - Channel data collection (2 bytes per channel, big-endian)
     *          - CRC16 checksum validation
     *          
     *          Frame Timing:
     *          - Baudrate: 115200 bps fixed (hardware requirement)
     *          - Frame rate: 20-50 Hz depending on channel count
     *          - Timeout: 3ms between frames for resynchronization
     * 
     * @param[in] byte Single byte received from UART at 115200 bps
     * @param[in] baudrate Serial baudrate in bps (must be 115200 for SUMD)
     * 
     * @note This method is called for each received byte by the RC protocol manager
     * @note Invalid or corrupted frames are silently discarded and state machine resets
     * @note Baudrate parameter should be 115200 bps for proper SUMD decoding
     * 
     * @warning Incorrect baudrate will result in garbled data and failed CRC validation
     * 
     * @see _process_byte() for internal byte processing implementation
     */
    void process_byte(uint8_t byte, uint32_t baudrate) override;

private:
    /**
     * @brief Internal byte processing with timestamp tracking
     * 
     * @details Implements the SUMD frame decoder state machine, processing each byte
     *          according to its position in the frame. Maintains running CRC16 checksum
     *          and validates frame integrity upon completion.
     *          
     *          State Machine Operation:
     *          - UNSYNCED: Search for 0xA8/0xA9 header, timeout resets after 3ms
     *          - GOT_HEADER: Store status byte, advance to GOT_STATE
     *          - GOT_STATE: Validate and store channel count, advance to GOT_LEN
     *          - GOT_LEN: Collect 2*channel_count bytes of channel data
     *          - GOT_DATA: Validate CRC16 high byte, advance to GOT_CRC
     *          - GOT_CRC: Complete CRC16 validation, extract channels if valid
     *          
     *          CRC16 Calculation:
     *          - Initialized at header byte
     *          - Updated with each status, length, and channel data byte
     *          - Compared against received CRC16 (2 bytes, big-endian)
     *          
     *          Channel Extraction:
     *          - Each channel is 2 bytes (high byte first, big-endian)
     *          - 12-bit effective resolution (0-4095)
     *          - Values converted to standard RC range for frontend
     * 
     * @param[in] timestamp_us Current timestamp in microseconds (for timeout detection)
     * @param[in] byte Current byte to process in state machine
     * 
     * @note Called internally by process_byte() after soft-serial decoding
     * @note Frame timeout of 3ms causes state machine reset to UNSYNCED
     * 
     * @warning Invalid frames (wrong CRC, bad channel count) are silently discarded
     */
    void _process_byte(uint32_t timestamp_us, uint8_t byte);

    /**
     * @struct ReceiverFcPacketHoTT
     * @brief SUMD/SUMH frame structure for Graupner HoTT receivers
     * 
     * @details Defines the binary layout of a complete SUMD frame as received from
     *          HoTT receivers. Frame structure matches Graupner protocol specification.
     *          
     *          Frame Layout (for N channels):
     *          - Byte 0: Header (0xA8=SUMD, 0xA9=SUMH) [not stored in struct, used for sync]
     *          - Byte 1: Status byte (this->status)
     *          - Byte 2: Channel count (this->length)
     *          - Bytes 3 to 3+2*N-1: Channel data (this->sumd_data[])
     *          - Bytes 3+2*N to 3+2*N+1: CRC16 (high byte stored in this->crc16_high, low byte separate)
     *          
     *          Status Byte Encoding:
     *          - 0x01: Valid live data frame (normal operation)
     *          - 0x00: SUMH mode frame (bidirectional telemetry mode)
     *          - 0x81: Failsafe mode (transmitter link lost)
     *          - Other values: Reserved or receiver-specific
     *          
     *          Channel Data Format:
     *          - 2 bytes per channel (16-bit big-endian)
     *          - High byte first, low byte second
     *          - 12-bit effective resolution (upper 12 bits typically used)
     *          - Center: 2048, Range: typically 1000-3000
     * 
     * @note Structure size is variable depending on channel count (not fixed-size)
     * @note CRC16 is calculated over all bytes from status through last channel data byte
     */
    typedef struct {
        uint8_t	status;							///< Status byte: 0x01=valid live data, 0x00=SUMH mode, 0x81=failsafe
        uint8_t	length;							///< Channel count (number of channels in frame, typically 8-16)
        uint8_t	sumd_data[(SUMD_MAX_CHANNELS+1) * 2];	///< Channel data array (2 bytes per channel, big-endian 16-bit values)
        uint8_t	crc16_high;						///< CRC16 high byte (most significant byte of 16-bit CRC)
    } ReceiverFcPacketHoTT;

    /**
     * @enum SUMD_DECODE_STATE
     * @brief State machine states for SUMD frame decoding
     * 
     * @details The decoder progresses through these states sequentially as bytes arrive:
     *          
     *          State Transitions:
     *          UNSYNCED → GOT_HEADER: When 0xA8 (SUMD) or 0xA9 (SUMH) header detected
     *          GOT_HEADER → GOT_STATE: After status byte received
     *          GOT_STATE → GOT_LEN: After channel count byte received and validated
     *          GOT_LEN → GOT_DATA: After all channel data bytes received (2 * channel_count)
     *          GOT_DATA → GOT_CRC: After CRC16 high byte received
     *          GOT_CRC → UNSYNCED: After CRC16 validated (success or failure)
     *          
     *          Any timeout (>3ms) or invalid data resets to UNSYNCED state.
     * 
     * @note State machine resets to UNSYNCED after frame completion or errors
     */
    enum SUMD_DECODE_STATE {
        SUMD_DECODE_STATE_UNSYNCED = 0,	///< Searching for frame header (0xA8 or 0xA9)
        SUMD_DECODE_STATE_GOT_HEADER,	///< Header received, expecting status byte
        SUMD_DECODE_STATE_GOT_STATE,	///< Status received, expecting channel count byte
        SUMD_DECODE_STATE_GOT_LEN,		///< Channel count received, collecting channel data
        SUMD_DECODE_STATE_GOT_DATA,		///< All channel data received, expecting CRC16 high byte
        SUMD_DECODE_STATE_GOT_CRC,		///< CRC16 complete, frame ready for validation
    };

    enum SUMD_DECODE_STATE _decode_state = SUMD_DECODE_STATE_UNSYNCED;	///< Current decoder state machine position
    uint8_t _rxlen;					///< Current byte index in sumd_data array during channel data collection
    ReceiverFcPacketHoTT _rxpacket;	///< Frame buffer for current SUMD frame being decoded
    uint16_t 	_crc16;				///< Running CRC16 checksum calculated during frame reception
    uint32_t last_packet_us;		///< Timestamp of last received byte (microseconds), used for timeout detection

    SoftSerial ss{115200, SoftSerial::SERIAL_CONFIG_8N1};	///< Software serial decoder at 115200 bps, 8 data bits, no parity, 1 stop bit
};

#endif  // AP_RCPROTOCOL_SUMD_ENABLED
