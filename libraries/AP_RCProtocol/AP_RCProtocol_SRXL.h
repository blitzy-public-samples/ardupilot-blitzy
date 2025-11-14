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
 * @file AP_RCProtocol_SRXL.h
 * @brief Spektrum SRXL (Spektrum Remote Receiver Link) protocol decoder
 * 
 * @details This file implements the SRXL v1 protocol decoder for Spektrum remote receivers.
 *          SRXL is a serial protocol used by Spektrum satellite receivers and compatible
 *          equipment to transmit RC channel data over a single serial connection.
 *          
 *          SRXL v1 Protocol Characteristics:
 *          - Baudrate: 115200 bps, 8N1 (8 data bits, no parity, 1 stop bit)
 *          - Variable-length frames: 18-35 bytes depending on variant (v1/v2/v5)
 *          - Frame structure: Header byte + Channel count + Channel data + CRC16
 *          - Channel data: 16-bit values per channel (0-65535 range)
 *          - Maximum channels: 12 channels for SRXL v1
 *          - CRC validation: CRC16-CCITT algorithm for frame integrity
 *          - Frame synchronization: Minimum 8000μs space between frames
 *          
 *          Supported SRXL Variants:
 *          - SRXL v1 (0xA1 header): Multiplex SRXLv1 / XBUS Mode B - 27 byte frames
 *          - SRXL v2 (0xA2 header): Multiplex SRXLv2 - 35 byte frames
 *          - SRXL v5 (0xA5 header): Spektrum AR7700 and similar - 18 byte frames
 *          
 *          Input Methods:
 *          - Serial byte stream: Direct UART connection at 115200 baud
 *          - Pulse-based input: Uses SoftSerial to decode pulse widths into serial data
 *          
 *          Relationship to SRXL2:
 *          SRXL2 is a newer Spektrum protocol with enhanced features including
 *          bidirectional communication, telemetry support, and extended channel counts.
 *          This implementation focuses on the original SRXL v1/v2/v5 variants for
 *          compatibility with legacy Spektrum satellite receivers.
 * 
 * @note Spektrum satellite receivers typically output SRXL on a dedicated serial port
 * @note Frame timing is critical - receiver must provide minimum 8ms gap between frames
 * 
 * @warning Timing Requirements: This decoder requires strict 115200 baud rate and proper
 *          frame synchronization. Incorrect baud rate or missing frame gaps will cause
 *          decode failures and lost RC data.
 * 
 * @warning CRC Validation: All frames must pass CRC16-CCITT validation. Corrupted frames
 *          are discarded to prevent invalid RC commands from reaching the flight controller.
 */

#pragma once

#include "AP_RCProtocol_config.h"

#if AP_RCPROTOCOL_SRXL_ENABLED

#include "AP_RCProtocol.h"
#include "SoftSerial.h"

#define SRXL_MIN_FRAMESPACE_US 8000U    /* Minumum space between srxl frames in us (applies to all variants)  */
#define SRXL_MAX_CHANNELS 20U           /* Maximum number of channels from srxl datastream  */

/* Variant specific SRXL datastream characteristics */
/* Framelength in byte */
#define SRXL_FRAMELEN_V1    27U      /* Framelength with header in byte for:  Mpx SRXLv1 or XBUS Mode B */
#define SRXL_FRAMELEN_V2    35U      /* Framelength with header in byte for:  Mpx SRXLv2 */
#define SRXL_FRAMELEN_V5    18U      /* Framelength with header in byte for  Spk AR7700 etc. */
#define SRXL_FRAMELEN_MAX   35U      /* maximum possible framelengh  */

/* Headerbyte */
#define SRXL_HEADER_V1          0xA1U    /* Headerbyte for:  Mpx SRXLv1 or XBUS Mode B */
#define SRXL_HEADER_V2          0xA2U    /* Headerbyte for:  Mpx SRXLv2  */
#define SRXL_HEADER_V5          0xA5U    /* Headerbyte for:  Spk AR7700 etc. */
#define SRXL_HEADER_NOT_IMPL    0xFFU    /* Headerbyte for non impemented srxl header*/

/**
 * @class AP_RCProtocol_SRXL
 * @brief Backend decoder for Spektrum SRXL (Spektrum Remote Receiver Link) v1 protocol
 * 
 * @details This class implements a complete decoder for the SRXL serial protocol used by
 *          Spektrum satellite receivers and compatible equipment. It handles frame
 *          synchronization, byte collection, CRC validation, and channel data extraction.
 *          
 *          Frame Processing Algorithm:
 *          1. STATE_IDLE: Wait for minimum frame gap (8000μs) to detect frame boundary
 *          2. STATE_NEW: Receive header byte, identify variant (v1/v2/v5), initialize CRC
 *          3. STATE_COLLECT: Accumulate frame bytes, calculate CRC concurrently
 *          4. Validation: Compare calculated CRC with received CRC at frame end
 *          5. Extraction: Parse channel data from validated frame into PWM values
 *          
 *          SRXL v1 Frame Structure (27 bytes for variant 0xA1):
 *          - Byte 0: Header byte (0xA1 for v1, 0xA2 for v2, 0xA5 for v5)
 *          - Byte 1-24: Channel data (12 channels × 2 bytes per channel, 16-bit values)
 *          - Byte 25-26: CRC16-CCITT checksum (big-endian)
 *          
 *          Channel Data Format:
 *          - Each channel: 16-bit unsigned value (0-65535)
 *          - Typical servo range: 1000-2000μs represented in 16-bit space
 *          - Values scaled from 16-bit to microseconds for ArduPilot consumption
 *          
 *          CRC16-CCITT Algorithm:
 *          - Polynomial: 0x1021 (x^16 + x^12 + x^5 + 1)
 *          - Initial value: 0x0000
 *          - Calculated over all bytes from header through channel data
 *          - Transmitted as 16-bit big-endian value at end of frame
 *          
 *          SoftSerial Pulse Decoding:
 *          When receiving pulse-based input (rather than direct serial), this class uses
 *          SoftSerial configured for 115200 baud, 8N1 to decode pulse widths into serial
 *          bytes. Pulse widths represent bit timings for software UART implementation.
 *          
 *          Spektrum Satellite Receiver Support:
 *          Compatible with Spektrum satellite receivers that output SRXL including:
 *          - SPM4648 (DSMX Remote Receiver)
 *          - AR7700 and similar Spektrum receivers
 *          - Multiplex SRXL-compatible receivers
 *          
 *          Timing Requirements:
 *          - Baudrate: 115200 bps (critical for proper byte synchronization)
 *          - Frame gap: Minimum 8000μs between frames for reliable synchronization
 *          - Frame rate: Typically 11ms (90Hz) or 22ms (45Hz) depending on receiver mode
 * 
 * @note This implementation is designed for unidirectional communication (receiver to FC)
 * @note SRXL v1/v2/v5 are legacy protocols; newer Spektrum equipment may use SRXL2
 * 
 * @warning Frame synchronization depends on proper timing gaps - continuous data stream
 *          without gaps will cause decoder to lose synchronization and drop frames
 * 
 * @warning CRC failures indicate RF interference, wiring issues, or baud rate mismatch
 * 
 * @see AP_RCProtocol_Backend for base class interface
 * @see SoftSerial for pulse-to-serial conversion implementation
 */
class AP_RCProtocol_SRXL : public AP_RCProtocol_Backend {
public:
    AP_RCProtocol_SRXL(AP_RCProtocol &_frontend) : AP_RCProtocol_Backend(_frontend) {}
    
    /**
     * @brief Process pulse-width encoded SRXL data from two-wire pulse input
     * 
     * @details This method receives pulse widths representing serial data bits and uses
     *          SoftSerial to decode them into serial bytes for SRXL frame processing.
     *          This is used when SRXL data is received as pulse-width modulated signals
     *          rather than direct UART serial connection.
     *          
     *          The SoftSerial decoder converts pulse timings into 8N1 serial format at
     *          115200 baud, then forwards decoded bytes to the SRXL frame parser.
     * 
     * @param[in] width_s0 Pulse width on signal line 0 in microseconds (μs)
     * @param[in] width_s1 Pulse width on signal line 1 in microseconds (μs)
     * 
     * @note Pulse widths must represent valid 115200 baud 8N1 bit timings
     * @note This method is called for each pulse edge detected on the input pins
     * 
     * @see SoftSerial::process_pulse() for pulse decoding implementation
     */
    void process_pulse(uint32_t width_s0, uint32_t width_s1) override;
    
    /**
     * @brief Process a single byte from SRXL serial stream
     * 
     * @details This method implements the SRXL frame decoder state machine. Each received
     *          byte advances the decoder through states: IDLE → NEW → COLLECT → validation.
     *          
     *          State Machine Operation:
     *          - IDLE: Wait for frame gap (8000μs), then transition to NEW on next byte
     *          - NEW: Identify frame type from header byte (0xA1/0xA2/0xA5), set frame
     *            length, initialize CRC calculation, transition to COLLECT
     *          - COLLECT: Accumulate bytes into buffer, update CRC, check for frame complete
     *          - Frame complete: Validate CRC16, extract channel data if valid, return to IDLE
     *          
     *          CRC Calculation:
     *          CRC16-CCITT is calculated incrementally as each byte arrives. The final two
     *          bytes of the frame contain the transmitted CRC for comparison.
     *          
     *          Frame Validation:
     *          - Header byte must match known variant (0xA1, 0xA2, or 0xA5)
     *          - Frame length must match expected length for variant
     *          - Calculated CRC must match transmitted CRC
     *          - Frame gap timing must be present before header
     *          
     *          On successful validation, channel data is extracted and converted from
     *          16-bit values to microsecond pulse widths for consumption by ArduPilot.
     * 
     * @param[in] byte     Single byte from SRXL serial stream
     * @param[in] baudrate Serial baudrate in bits per second (bps) - must be 115200 for SRXL
     * 
     * @note This method is called for each byte received on the UART at 115200 baud
     * @note Invalid frames are silently discarded; valid frames update RC channel values
     * 
     * @warning Baudrate parameter must be 115200 bps for proper SRXL decoding. Other
     *          baud rates will cause frame synchronization failures and data loss.
     * 
     * @warning This method must be called at consistent timing intervals to maintain
     *          accurate frame gap detection for synchronization
     * 
     * @see _process_byte() for internal byte processing implementation
     * @see srxl_channels_get_v1v2() for v1/v2 channel extraction
     * @see srxl_channels_get_v5() for v5 channel extraction
     */
    void process_byte(uint8_t byte, uint32_t baudrate) override;
private:
    void _process_byte(uint32_t timestamp_us, uint8_t byte);
    int srxl_channels_get_v1v2(uint16_t max_values, uint8_t *num_values, uint16_t *values, bool *failsafe_state);
    int srxl_channels_get_v5(uint16_t max_values, uint8_t *num_values, uint16_t *values, bool *failsafe_state);
    uint8_t buffer[SRXL_FRAMELEN_MAX];       /* buffer for raw srxl frame data in correct order --> buffer[0]=byte0  buffer[1]=byte1  */
    uint8_t buflen;                          /* length in number of bytes of received srxl dataframe in buffer  */
    uint32_t last_data_us;                   /* timespan since last received data in us   */
    uint16_t channels[SRXL_MAX_CHANNELS] = {0};    /* buffer for extracted RC channel data as pulsewidth in microseconds */
    uint16_t max_channels = 0;
    enum {
        STATE_IDLE,                          /* do nothing */
        STATE_NEW,                           /* get header of frame + prepare for frame reception + begin new crc cycle   */
        STATE_COLLECT                        /* collect RC channel data from frame + concurrently calc crc over payload data + extract channel information */
    };
    uint8_t frame_header = 0U;                   /* Frame header from SRXL datastream    */
    uint8_t frame_len_full = 0U;                 /* Length in number of bytes of full srxl datastream */
    uint8_t decode_state = STATE_IDLE;           /* Current state of SRXL frame decoding */
    uint8_t decode_state_next = STATE_IDLE;      /* State of frame decoding thatwill be applied when the next byte from dataframe drops in  */
    uint16_t crc_fmu = 0U;                       /* CRC calculated over payload from srxl datastream on this machine */
    uint16_t crc_receiver = 0U;                  /* CRC extracted from srxl datastream  */

    SoftSerial ss{115200, SoftSerial::SERIAL_CONFIG_8N1};
};

#endif  // AP_RCPROTOCOL_SRXL_ENABLED
