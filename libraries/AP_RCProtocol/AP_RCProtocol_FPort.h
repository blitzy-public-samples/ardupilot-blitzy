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
 * @file AP_RCProtocol_FPort.h
 * @brief FrSky FPort RC protocol decoder with bidirectional telemetry support
 * 
 * @details This file implements the FrSky FPort protocol decoder, which provides
 *          16-channel RC control with integrated S.Port telemetry downlink capability.
 *          FPort combines RC control frames and telemetry frames in a single bidirectional
 *          serial stream.
 * 
 *          Protocol Characteristics:
 *          - Serial Configuration: 115200 baud, 8N1, inverted UART (8N1I)
 *          - Frame Format: Byte-stuffed with 0x7D escape sequences, 0x7E frame boundaries
 *          - RC Channels: 16 channels, 11-bit resolution (172-1811 range, 992 center)
 *          - Telemetry: Integrated S.Port telemetry downlink for sensor data
 *          - RSSI Reporting: Link quality indicator included in control frames
 *          - Frame Size: 29 bytes for control frames (after byte unstuffing)
 *          - CRC Validation: 8-bit CRC checksum for frame integrity
 * 
 *          Frame Types:
 *          - Control Frame (0x00): Contains RC channel data and RSSI
 *          - Downlink Frame (0x10): Request for telemetry data from receiver (R-XSR)
 * 
 *          Byte-Stuffing Algorithm:
 *          FPort uses byte-stuffing to distinguish frame boundaries and data:
 *          - 0x7E: Frame boundary marker (start/end of frame)
 *          - 0x7D: Escape sequence - next byte is XOR'd with 0x20
 *          - Example: Data byte 0x7E is transmitted as 0x7D 0x5E (0x7E ^ 0x20)
 *          - Example: Data byte 0x7D is transmitted as 0x7D 0x5D (0x7D ^ 0x20)
 * 
 *          Timing Requirements:
 *          - Receiver-driven frame rate: Some receivers (R-XSR) send 0x10 byte when ready
 *          - Maximum consecutive telemetry frames limited to prevent control frame starvation
 *          - Frame synchronization critical: Lost bytes require resynchronization to 0x7E boundary
 * 
 * @note FPort requires inverted UART signal - ensure serial port supports 8N1I configuration
 * @warning Frame decoding is timing-critical - process bytes immediately at 115200 baud
 * @warning Incorrect serial configuration (non-inverted) will result in garbled data
 * 
 * @see AP_RCProtocol_Backend
 * @see AP_Frsky_SPort
 * 
 * Source: libraries/AP_RCProtocol/AP_RCProtocol_FPort.h
 */

#pragma once

#include "AP_RCProtocol_config.h"

#if AP_RCPROTOCOL_FPORT_ENABLED

#include "AP_RCProtocol.h"
#include "SoftSerial.h"
#include <AP_Frsky_Telem/AP_Frsky_SPort.h>

#define FPORT_CONTROL_FRAME_SIZE 29

struct FPort_Frame;

/**
 * @class AP_RCProtocol_FPort
 * @brief FrSky FPort protocol decoder backend for RC control and telemetry
 * 
 * @details This class implements the FrSky FPort protocol decoder, handling both
 *          incoming RC control frames and outgoing S.Port telemetry frames over
 *          a single inverted serial connection at 115200 baud.
 * 
 *          Protocol Features:
 *          - 16-channel RC control with 11-bit resolution (172-1811 range)
 *          - Integrated S.Port telemetry downlink for sensor data transmission
 *          - RSSI (Received Signal Strength Indicator) reporting
 *          - Byte-stuffed frame format with CRC validation
 *          - Bidirectional communication (control uplink + telemetry downlink)
 * 
 *          Frame Processing:
 *          - Decodes byte-stuffed serial stream (0x7D escape, 0x7E boundary)
 *          - Validates frames using 8-bit CRC checksum
 *          - Extracts 16 RC channels from control frames (type 0x00)
 *          - Handles telemetry request frames from receiver (type 0x10)
 *          - Manages telemetry frame scheduling to prevent control starvation
 * 
 *          Hardware Requirements:
 *          - Inverted UART serial port (8N1I configuration)
 *          - 115200 baud rate
 *          - Connected to FrSky receiver SPORT/FBUS port
 * 
 *          Usage Pattern:
 *          1. Instantiate with frontend reference and inverted flag
 *          2. Call process_byte() for each received byte at 115200 baud
 *          3. Backend decodes frames and updates RC channel values
 *          4. Backend constructs and sends S.Port telemetry packets
 * 
 * @note Some receivers (R-XSR) send 0x10 byte to request telemetry (receiver-driven)
 * @note Consecutive telemetry frames are rate-limited to ensure control frame delivery
 * @warning Requires inverted UART - standard UART will not work with FPort receivers
 * @warning Frame synchronization lost if bytes are dropped - requires resync to 0x7E
 */
class AP_RCProtocol_FPort : public AP_RCProtocol_Backend {
public:
    /**
     * @brief Construct FPort protocol decoder
     * 
     * @param[in] _frontend Reference to AP_RCProtocol frontend for channel updates
     * @param[in] inverted  True if UART is inverted (normal for FPort), false otherwise
     * 
     * @note FPort typically requires inverted=true for proper operation
     */
    AP_RCProtocol_FPort(AP_RCProtocol &_frontend, bool inverted);
    
    /**
     * @brief Process pulse-width timing for software serial decoding
     * 
     * @details This method implements software serial decoding from pulse widths when
     *          hardware UART is not available. It reconstructs serial bytes from the
     *          timing of signal transitions, handling both inverted and non-inverted
     *          UART signals.
     * 
     * @param[in] width_s0 Pulse width in microseconds for state 0 (low state duration)
     * @param[in] width_s1 Pulse width in microseconds for state 1 (high state duration)
     * 
     * @note This is used for software serial decoding at 115200 baud
     * @note Each pulse represents one bit time (~8.68 microseconds at 115200 baud)
     * @warning Timing jitter can cause bit errors - use hardware UART when available
     */
    void process_pulse(uint32_t width_s0, uint32_t width_s1) override;
    
    /**
     * @brief Process incoming byte from FPort serial stream
     * 
     * @details This is the primary entry point for FPort frame decoding. Each byte
     *          received at 115200 baud is processed for:
     *          - Frame boundary detection (0x7E markers)
     *          - Byte unstuffing (0x7D escape sequence handling)
     *          - Frame buffer accumulation
     *          - CRC validation and frame decoding when complete
     *          - RC channel extraction and telemetry request handling
     * 
     *          Byte-Stuffing Decoding:
     *          - 0x7E: Frame boundary - reset buffer and prepare for new frame
     *          - 0x7D: Escape sequence - XOR next byte with 0x20 before storing
     *          - Other: Regular data byte - store in frame buffer
     * 
     *          Frame Types Decoded:
     *          - Control Frame (0x00): 16 RC channels + RSSI → update channel values
     *          - Downlink Frame (0x10): Telemetry request → schedule telemetry response
     * 
     * @param[in] byte     Received byte from serial stream (after UART decoding)
     * @param[in] baudrate Expected baud rate in bits per second (should be 115200 for FPort)
     * 
     * @note Called at interrupt level or high frequency - keep processing fast
     * @note Frame timeout detection: gap >2ms between bytes triggers resynchronization
     * @warning Baudrate mismatch will cause decoding failures - FPort requires 115200 bps
     * @warning Must be called for every received byte to maintain frame synchronization
     */
    void process_byte(uint8_t byte, uint32_t baudrate) override;

private:
    /**
     * @brief Decode RC control frame and update channel values
     * 
     * @details Extracts 16 RC channels from FPort control frame (type 0x00).
     *          Channels are packed as 11-bit values with range 172-1811 (center 992).
     *          Also extracts RSSI value and frame-lost indicator.
     * 
     * @param[in] frame Reference to validated FPort_Frame structure containing control data
     * 
     * @note Control frames are 29 bytes: [type][24-byte-channel-data][flags][rssi][crc]
     * @note Channel packing: 16 channels × 11 bits = 176 bits = 22 bytes (+ 2 bytes flags/spare)
     */
    void decode_control(const FPort_Frame &frame);
    
    /**
     * @brief Decode telemetry request frame from receiver
     * 
     * @details Handles downlink frame (type 0x10) sent by receiver when ready to
     *          receive telemetry data. Sets receiver-driven frame rate flag and
     *          triggers S.Port telemetry packet transmission.
     * 
     * @param[in] frame Reference to validated FPort_Frame structure containing downlink request
     * 
     * @note R-XSR and similar receivers send 0x10 byte to request telemetry
     * @note Enables receiver-driven telemetry timing for optimal frame scheduling
     */
    void decode_downlink(const FPort_Frame &frame);
    
    /**
     * @brief Validate frame CRC checksum
     * 
     * @details Calculates 8-bit CRC over frame data and compares with received CRC.
     *          Uses FPort CRC algorithm compatible with FrSky protocol.
     * 
     * @return true if CRC is valid, false if frame is corrupted
     * 
     * @note CRC failures indicate transmission errors or loss of synchronization
     * @note Bad CRC triggers frame discard and resynchronization to next 0x7E boundary
     */
    bool check_checksum(void);

    /**
     * @brief Internal byte processing with timestamp tracking
     * 
     * @details Processes individual bytes with microsecond timestamp for timeout detection.
     *          Implements byte-stuffing decode state machine and frame boundary detection.
     * 
     * @param[in] timestamp_us Microsecond timestamp of byte reception
     * @param[in] byte         Byte to process
     * 
     * @note Timeout detection: >2ms gap between bytes triggers frame reset
     */
    void _process_byte(uint32_t timestamp_us, uint8_t byte);
    
    /// Software serial decoder for pulse-width based byte reconstruction (115200 baud, 8N1)
    SoftSerial ss{115200, SoftSerial::SERIAL_CONFIG_8N1};
    
    /// Saved pulse width for software serial bit timing reconstruction
    uint32_t saved_width;

    /**
     * @brief Byte input state machine for frame assembly
     * 
     * @details Maintains state for byte-stuffed frame decoding including buffer,
     *          current offset, timeout detection, and escape sequence tracking.
     */
    struct {
        uint8_t buf[FPORT_CONTROL_FRAME_SIZE]; ///< Frame buffer (29 bytes for control frame)
        uint8_t ofs;                           ///< Current write offset in buffer (0-28)
        uint32_t last_byte_us;                 ///< Timestamp of last received byte (microseconds) for timeout detection
        bool got_DLE;                          ///< Escape sequence flag: true after receiving 0x7D, next byte needs XOR 0x20
    } byte_input;

    /// UART inversion flag: true for inverted signal (standard FPort), false for normal UART
    const bool inverted;

#if AP_FRSKY_SPORT_TELEM_ENABLED
    /**
     * @brief S.Port telemetry packet buffer for downlink transmission
     * 
     * @details Stores outgoing S.Port telemetry packet ready for transmission
     *          when receiver requests telemetry data via downlink frame.
     */
    struct {
        bool available = false;                   ///< True when telemetry packet is ready to send
        AP_Frsky_SPort::sport_packet_t packet;    ///< S.Port telemetry packet structure (sensor data)
    } telem_data;
#endif // AP_FRSKY_SPORT_TELEM_ENABLED

    /**
     * @brief Receiver-driven frame rate flag
     * 
     * @details True when receiver (e.g., R-XSR) sends 0x10 byte to explicitly request
     *          telemetry frames. When true, telemetry is sent only on request rather
     *          than at fixed intervals.
     * 
     * @note Improves telemetry timing by synchronizing with receiver's frame schedule
     */
    bool rx_driven_frame_rate = false;

    /**
     * @brief Consecutive telemetry frame counter for rate limiting
     * 
     * @details Tracks number of consecutive telemetry frames sent without intervening
     *          control frames. Limited to prevent control frame starvation when receiver
     *          is not controlling frame rate.
     * 
     * @note Maximum consecutive frames prevents telemetry from blocking RC control updates
     * @warning Excessive telemetry can delay RC control updates - rate limiting is critical
     */
    uint8_t consecutive_telemetry_frame_count;
};

#endif  // AP_RCPROTOCOL_FPORT_ENABLED
