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
 * @file AP_RCProtocol_ST24.h
 * @brief Graupner HoTT ST24 RC protocol decoder for HoTT receivers
 * 
 * @details This file implements the ST24 protocol decoder for Graupner/SJ HoTT
 *          receivers. The ST24 protocol is a serial protocol used by HoTT receivers
 *          to transmit RC channel data and telemetry information.
 *          
 *          Protocol Characteristics:
 *          - Serial baudrate: 115200 bps, 8N1 (8 data bits, no parity, 1 stop bit)
 *          - Frame structure: sync byte (0x55) + sync byte (0x55) + length + type + payload + CRC8
 *          - Channel support: 12 channels (ST24_PACKET_TYPE_CHANNELDATA12) or 24 channels (ST24_PACKET_TYPE_CHANNELDATA24)
 *          - Channel data: 12-bit values (0-4095 range) scaled to PWM (1000-2000 μs)
 *          - RSSI reporting: Signal strength included in channel data frames
 *          - Frame validation: CRC8 checksum (XOR-based) and length consistency checks
 *          
 *          Frame Format:
 *          - Byte 0: Sync byte 1 (0x55)
 *          - Byte 1: Sync byte 2 (0x55)
 *          - Byte 2: Length (includes type + payload + CRC8)
 *          - Byte 3: Frame type (from ST24_PACKET_TYPE enum)
 *          - Bytes 4-N: Payload data (channel values, RSSI, packet counter)
 *          - Byte N+1: CRC8 checksum
 *          
 *          Supported Frame Types:
 *          - ST24_PACKET_TYPE_CHANNELDATA12: 12-channel RC data with RSSI
 *          - ST24_PACKET_TYPE_CHANNELDATA24: 24-channel RC data with RSSI
 *          - ST24_PACKET_TYPE_TRANSMITTERGPSDATA: Transmitter GPS data (telemetry)
 *          
 * @note The ST24 protocol uses a custom CRC8 algorithm for frame validation
 * @note Channel values are transmitted as 12-bit values packed into byte arrays
 * 
 * @warning Frame validation requires both CRC8 checksum verification and frame
 *          length consistency checks to ensure data integrity
 * 
 * @see AP_RCProtocol_Backend for base class interface
 * 
 * Source: libraries/AP_RCProtocol/AP_RCProtocol_ST24.h
 */

#pragma once

#include "AP_RCProtocol_config.h"

#if AP_RCPROTOCOL_ST24_ENABLED

#include "AP_RCProtocol_Backend.h"
#include "SoftSerial.h"

#define ST24_DATA_LEN_MAX	64
#define ST24_MAX_FRAMELEN   70
#define ST24_STX1		0x55
#define ST24_STX2		0x55

/* define range mapping here, -+100% -> 1000..2000 */
#define ST24_RANGE_MIN 0.0f
#define ST24_RANGE_MAX 4096.0f

#define ST24_TARGET_MIN 1000.0f
#define ST24_TARGET_MAX 2000.0f

/* pre-calculate the floating point stuff as far as possible at compile time */
#define ST24_SCALE_FACTOR ((ST24_TARGET_MAX - ST24_TARGET_MIN) / (ST24_RANGE_MAX - ST24_RANGE_MIN))
#define ST24_SCALE_OFFSET (int)(ST24_TARGET_MIN - (ST24_SCALE_FACTOR * ST24_RANGE_MIN + 0.5f))

/**
 * @class AP_RCProtocol_ST24
 * @brief Backend decoder for Graupner HoTT ST24 protocol frames
 * 
 * @details This class implements the ST24 RC protocol decoder for Graupner/SJ HoTT
 *          receivers. It handles the decoding of serial frames transmitted at 115200 bps
 *          containing RC channel data, RSSI information, and packet counters.
 *          
 *          The decoder implements a state machine to synchronize with the ST24 frame
 *          structure, validate frame integrity using CRC8 checksums, and extract
 *          12-bit channel values that are scaled to standard PWM ranges (1000-2000 μs).
 *          
 *          Protocol Features:
 *          - 12-channel or 24-channel support
 *          - RSSI (signal strength) reporting
 *          - Packet counter for frame age/rate tracking
 *          - Frame type indicators for different data types
 *          - CRC8 validation for data integrity
 *          
 *          State Machine:
 *          - ST24_DECODE_STATE_UNSYNCED: Waiting for first sync byte (0x55)
 *          - ST24_DECODE_STATE_GOT_STX1: First sync byte received
 *          - ST24_DECODE_STATE_GOT_STX2: Second sync byte received
 *          - ST24_DECODE_STATE_GOT_LEN: Length byte received
 *          - ST24_DECODE_STATE_GOT_TYPE: Frame type byte received
 *          - ST24_DECODE_STATE_GOT_DATA: Complete frame received, ready for validation
 *          
 *          Channel Data Format:
 *          - 12-bit values (0-4095) packed into byte arrays
 *          - Scaled to PWM range: 0 -> 1000 μs, 4095 -> 2000 μs
 *          - Linear scaling with pre-calculated constants for efficiency
 *          
 * @note This decoder is compatible with Graupner HoTT receivers using ST24 protocol
 * @note The serial baudrate is fixed at 115200 bps with 8N1 configuration
 * 
 * @warning Frame validation is critical - both CRC8 and length checks must pass
 *          before channel data is accepted to prevent control corruption
 * 
 * @see AP_RCProtocol_Backend for base class interface and lifecycle
 */
class AP_RCProtocol_ST24 : public AP_RCProtocol_Backend {
public:
    AP_RCProtocol_ST24(AP_RCProtocol &_frontend) : AP_RCProtocol_Backend(_frontend) {}
    
    /**
     * @brief Process pulse-width data (not used by ST24)
     * 
     * @details The ST24 protocol is serial-based and does not use pulse-width
     *          timing. This override is provided for interface compatibility
     *          but performs no operation.
     * 
     * @param[in] width_s0 Pulse width for signal 0 (unused)
     * @param[in] width_s1 Pulse width for signal 1 (unused)
     * 
     * @note ST24 uses serial byte processing via process_byte() instead
     */
    void process_pulse(uint32_t width_s0, uint32_t width_s1) override;
    
    /**
     * @brief Process incoming serial bytes from ST24 receiver
     * 
     * @details This method receives serial bytes from the RC receiver and delegates
     *          to the internal decoder. It verifies the baudrate matches the expected
     *          115200 bps for ST24 protocol before processing.
     *          
     *          The method serves as the public interface for byte-by-byte decoding
     *          of ST24 frames. Each byte is passed to the internal state machine
     *          which handles frame synchronization, validation, and channel extraction.
     * 
     * @param[in] byte Single byte received from serial port
     * @param[in] baudrate Serial port baudrate in bps (expected: 115200)
     * 
     * @note Called at serial receive rate (typically per byte interrupt)
     * @note Baudrate verification ensures protocol compatibility
     * 
     * @see _process_byte() for internal state machine implementation
     */
    void process_byte(uint8_t byte, uint32_t baudrate) override;
    
private:
    /**
     * @brief Internal ST24 frame decoder state machine
     * 
     * @details This method implements the core ST24 protocol decoder using a state
     *          machine to synchronize with frame boundaries, extract frame fields,
     *          and validate frame integrity.
     *          
     *          Decoding Process:
     *          1. Wait for first sync byte (0x55)
     *          2. Verify second sync byte (0x55)
     *          3. Extract length byte (payload + type + CRC size)
     *          4. Extract frame type (channel data or telemetry)
     *          5. Collect payload bytes based on length
     *          6. Validate CRC8 checksum
     *          7. Extract and scale channel values if validation passes
     *          
     *          Frame Validation:
     *          - Sync bytes must both be 0x55
     *          - Length must not exceed ST24_MAX_FRAMELEN
     *          - CRC8 must match calculated checksum
     *          - Frame type must be recognized
     *          
     *          On successful validation:
     *          - Channel values extracted and scaled to PWM (1000-2000 μs)
     *          - RSSI value extracted from frame
     *          - New frame availability signaled to frontend
     * 
     * @param[in] byte Single byte from ST24 serial stream
     * 
     * @note This is called by process_byte() after baudrate verification
     * @note Invalid frames cause state machine reset to ST24_DECODE_STATE_UNSYNCED
     * 
     * @warning Frame corruption can occur if bytes are dropped or baudrate is incorrect
     * 
     * @see st24_crc8() for checksum calculation algorithm
     * @see ST24_DECODE_STATE enum for state machine states
     */
    void _process_byte(uint8_t byte);
    
    /**
     * @brief Calculate CRC8 checksum for ST24 frame validation
     * 
     * @details Computes an 8-bit cyclic redundancy check (CRC8) for ST24 frame
     *          validation. The algorithm uses XOR-based checksum calculation
     *          across the frame length, type, and payload bytes.
     *          
     *          CRC Calculation:
     *          - Initialize CRC to 0
     *          - For each byte: XOR with current CRC
     *          - Result is 8-bit checksum value
     *          
     *          The CRC covers:
     *          - Length byte
     *          - Type byte
     *          - All payload data bytes
     *          
     *          The CRC does NOT cover:
     *          - Sync bytes (0x55, 0x55)
     *          - The CRC byte itself
     * 
     * @param[in] ptr Pointer to data buffer to checksum
     * @param[in] len Number of bytes to include in checksum
     * 
     * @return uint8_t Calculated CRC8 checksum value
     * 
     * @note This is a static method as it performs pure computation
     * @note CRC validation must pass before frame data is used
     * 
     * @warning Incorrect CRC indicates frame corruption - data must be discarded
     */
    static uint8_t st24_crc8(uint8_t *ptr, uint8_t len);
    enum ST24_PACKET_TYPE {
        ST24_PACKET_TYPE_CHANNELDATA12 = 0,
        ST24_PACKET_TYPE_CHANNELDATA24,
        ST24_PACKET_TYPE_TRANSMITTERGPSDATA
    };

#pragma pack(push, 1)
    typedef struct {
        uint8_t	header1;			///< 0x55 for a valid packet
        uint8_t	header2;			///< 0x55 for a valid packet
        uint8_t	length;				///< length includes type, data, and crc = sizeof(type)+sizeof(data[payload_len])+sizeof(crc8)
        uint8_t	type;				///< from enum ST24_PACKET_TYPE
        uint8_t	st24_data[ST24_DATA_LEN_MAX];
        uint8_t	crc8;				///< crc8 checksum, calculated by st24_common_crc8 and including fields length, type and st24_data
    } ReceiverFcPacket;

    /**
     * RC Channel data (12 channels).
     *
     * This is incoming from the ST24
     */
    typedef struct {
        uint16_t t;			///< packet counter or clock
        uint8_t	rssi;			///< signal strength
        uint8_t	packet_count;		///< Number of UART packets sent since reception of last RF frame (this tells something about age / rate)
        uint8_t	channel[18];		///< channel data, 12 channels (12 bit numbers)
    } ChannelData12;

    /**
     * RC Channel data (12 channels).
     *
     */
    typedef struct {
        uint16_t t;			///< packet counter or clock
        uint8_t	rssi;			///< signal strength
        uint8_t	packet_count;		///< Number of UART packets sent since reception of last RF frame (this tells something about age / rate)
        uint8_t	channel[36];		///< channel data, 24 channels (12 bit numbers)
    } ChannelData24;

    /**
     * Telemetry packet
     *
     * This is outgoing to the ST24
     *
     * imuStatus:
     * 8 bit total
     * bits 0-2 for status
     * - value 0 is FAILED
     * - value 1 is INITIALIZING
     * - value 2 is RUNNING
     * - values 3 through 7 are reserved
     * bits 3-7 are status for sensors (0 or 1)
     * - mpu6050
     * - accelerometer
     * - primary gyro x
     * - primary gyro y
     * - primary gyro z
     *
     * pressCompassStatus
     * 8 bit total
     * bits 0-3 for compass status
     * - value 0 is FAILED
     * - value 1 is INITIALIZING
     * - value 2 is RUNNING
     * - value 3 - 15 are reserved
     * bits 4-7 for pressure status
     * - value 0 is FAILED
     * - value 1 is INITIALIZING
     * - value 2 is RUNNING
     * - value 3 - 15 are reserved
     *
     */
    typedef struct {
        uint16_t t;			///< packet counter or clock
        int32_t	lat;			///< lattitude (degrees)	+/- 90 deg
        int32_t	lon;			///< longitude (degrees)	+/- 180 deg
        int32_t	alt;			///< 0.01m resolution, altitude (meters)
        int16_t	vx, vy, vz; 		///< velocity 0.01m res, +/-320.00 North-East- Down
        uint8_t	nsat;			///<number of satellites
        uint8_t	voltage; 		///< 25.4V	voltage = 5 + 255*0.1 = 30.5V, min=5V
        uint8_t	current; 		///< 0.5A resolution
        int16_t	roll, pitch, yaw;	///< 0.01 degree resolution
        uint8_t	motorStatus;		///< 1 bit per motor for status 1=good, 0= fail
        uint8_t	imuStatus;		///< inertial measurement unit status
        uint8_t	pressCompassStatus;	///< baro / compass status
    } TelemetryData;

#pragma pack(pop)

    enum ST24_DECODE_STATE {
        ST24_DECODE_STATE_UNSYNCED = 0,
        ST24_DECODE_STATE_GOT_STX1,
        ST24_DECODE_STATE_GOT_STX2,
        ST24_DECODE_STATE_GOT_LEN,
        ST24_DECODE_STATE_GOT_TYPE,
        ST24_DECODE_STATE_GOT_DATA
    };

    enum ST24_DECODE_STATE _decode_state = ST24_DECODE_STATE_UNSYNCED;
    uint8_t _rxlen;

    ReceiverFcPacket _rxpacket;

    SoftSerial ss{115200, SoftSerial::SERIAL_CONFIG_8N1};
};

#endif  // AP_RCPROTOCOL_ST24_ENABLED
