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
 */

/**
 * @file AP_RCProtocol_IBUS.h
 * @brief FlySky iBus RC protocol decoder for FlySky receivers
 * 
 * @details This file implements the FlySky iBus protocol decoder, which handles
 *          RC input from FlySky receivers using the iBus serial protocol.
 *          
 *          iBus Protocol Specification:
 *          - Frame size: 32 bytes fixed
 *          - Baudrate: 115200 bps, 8N1 configuration
 *          - Frame structure:
 *            * Byte 0: Header (0x20 for standard frames, 0x40 for sensor frames)
 *            * Byte 1: Command/flags byte
 *            * Bytes 2-29: Channel data (14 channels × 2 bytes each, little-endian)
 *            * Bytes 30-31: Checksum (16-bit, little-endian)
 *          
 *          Channel Format:
 *          - 14 channels supported (IBUS_INPUT_CHANNELS)
 *          - 11-bit resolution per channel (0-2047 raw values before scaling)
 *          - Channels transmitted as 16-bit little-endian values
 *          
 *          Failsafe Indication:
 *          - Header byte 0x20: Standard RC data frames
 *          - Header byte 0x40: Sensor telemetry frames
 *          - Failsafe flags encoded in header byte nibbles
 *          
 *          The decoder supports both UART byte-based input (process_byte) and
 *          pulse-based input via SoftSerial (process_pulse) for flexible integration.
 * 
 * @note FlySky iBus is commonly used with FlySky FS-i6, FS-i6S, and compatible receivers
 * @see AP_RCProtocol_Backend for base protocol interface
 */

#pragma once

#include "AP_RCProtocol_config.h"

#if AP_RCPROTOCOL_IBUS_ENABLED

/// iBus frame size in bytes (fixed at 32 bytes per frame)
#define IBUS_FRAME_SIZE		32
/// Number of RC channels supported by iBus protocol (14 channels maximum)
#define IBUS_INPUT_CHANNELS	14

#include "AP_RCProtocol_Backend.h"

#include "SoftSerial.h"

/**
 * @class AP_RCProtocol_IBUS
 * @brief FlySky iBus RC protocol backend for decoding iBus frames
 * 
 * @details This backend implements the FlySky iBus protocol decoder for processing
 *          RC input from FlySky receivers. It supports both UART byte-stream input
 *          and pulse-based input via software serial emulation.
 *          
 *          Frame Processing:
 *          The decoder accumulates incoming bytes into a 32-byte frame buffer and
 *          validates each complete frame using checksum verification. Valid frames
 *          are decoded to extract 14 RC channel values and failsafe status.
 *          
 *          Checksum Algorithm:
 *          iBus uses a 16-bit checksum calculated such that the sum of all 32 bytes
 *          in the frame (including the 2-byte checksum itself) equals 0xFFFF. This
 *          is achieved by setting checksum = 0xFFFF - sum(bytes[0..29]).
 *          
 *          Input Methods:
 *          - process_byte(): Direct UART byte input at 115200 bps
 *          - process_pulse(): Pulse-width input decoded via SoftSerial (8N1 config)
 *          
 *          Channel Scaling:
 *          Raw 11-bit channel values (0-2047) are scaled to standard PWM microsecond
 *          values (typically 1000-2000 μs) by the base protocol layer.
 *          
 *          Timing:
 *          Frame timeout detection uses inter-byte timing to identify frame boundaries.
 *          Gaps > 3ms between bytes indicate start of new frame.
 * 
 * @note This implementation uses SoftSerial for pulse-based input to handle receivers
 *       that output iBus protocol using pulse-width encoding rather than UART levels
 * 
 * @warning The checksum must be validated before trusting channel data. Invalid checksums
 *          indicate corrupted frames that must be discarded to prevent erratic behavior.
 * 
 * @see AP_RCProtocol_Backend
 * @see SoftSerial
 */
class AP_RCProtocol_IBUS : public AP_RCProtocol_Backend
{
public:
    using AP_RCProtocol_Backend::AP_RCProtocol_Backend;

    /**
     * @brief Process pulse-width input for iBus protocol via SoftSerial
     * 
     * @details This method handles pulse-based input from receivers that encode
     *          iBus frames as pulse widths rather than UART voltage levels.
     *          The pulses are decoded by the SoftSerial software UART implementation
     *          configured for 115200 bps, 8N1 (8 data bits, no parity, 1 stop bit).
     *          
     *          The SoftSerial decoder converts pulse timings to bytes which are then
     *          passed to _process_byte() for frame assembly and validation.
     *          
     *          This input method is useful for:
     *          - Receivers with inverted serial output
     *          - GPIO pins without hardware UART support
     *          - Flight controllers requiring pulse input compatibility
     * 
     * @param[in] width_s0 Pulse width of signal 0 in microseconds (state 0 duration)
     * @param[in] width_s1 Pulse width of signal 1 in microseconds (state 1 duration)
     * 
     * @note Called by RC input processing at pulse edge events
     * @note SoftSerial is configured for 115200 bps baudrate to match iBus specification
     * 
     * @see SoftSerial::process_pulse()
     * @see _process_byte()
     */
    void process_pulse(uint32_t width_s0, uint32_t width_s1) override;

    /**
     * @brief Process incoming byte from UART for iBus protocol decoding
     * 
     * @details This method handles direct UART byte input from FlySky receivers
     *          transmitting iBus protocol at 115200 bps. Each byte is passed to
     *          _process_byte() with timing information for frame assembly.
     *          
     *          The baudrate parameter is validated to ensure it matches the expected
     *          115200 bps rate for iBus protocol. Bytes received at other baudrates
     *          may be rejected or cause protocol detection to fail.
     *          
     *          This is the primary input method for:
     *          - Hardware UART ports connected to FlySky receivers
     *          - Standard non-inverted serial connections
     *          - Direct receiver UART output
     * 
     * @param[in] byte Received byte from UART (8-bit data)
     * @param[in] baudrate UART baudrate in bits per second (expected: 115200 bps)
     * 
     * @note Called by RC protocol manager for each received UART byte
     * @note Frame timeout and synchronization handled internally by _process_byte()
     * 
     * @see _process_byte()
     */
    void process_byte(uint8_t byte, uint32_t baudrate) override;
private:
    /**
     * @brief Internal byte processing for iBus frame assembly and decoding
     * 
     * @details This method assembles incoming bytes into complete 32-byte iBus frames,
     *          performs frame synchronization, and validates/decodes complete frames.
     *          
     *          Frame Synchronization:
     *          - Detects frame boundaries using inter-byte timing (>3ms gap = new frame)
     *          - Validates header byte (0x20 or 0x40) to confirm frame start
     *          - Resets buffer offset on timeout or invalid frame
     *          
     *          Frame Processing:
     *          When 32 bytes are accumulated, the frame is validated and decoded:
     *          1. Checksum verification (sum of all 32 bytes must equal 0xFFFF)
     *          2. Channel data extraction (14 channels, 11-bit values)
     *          3. Failsafe status determination from header byte
     *          4. Update of RC channel values via add_input()
     *          
     *          Timing Constraints:
     *          - Inter-byte timeout: 3000 μs (3 ms)
     *          - Expected byte interval: ~87 μs at 115200 bps
     *          - Frame interval: typically 7-14 ms (50-140 Hz update rate)
     * 
     * @param[in] timestamp_us Current timestamp in microseconds for timeout detection
     * @param[in] byte Received byte to add to frame buffer
     * 
     * @note This is the core frame assembly logic called by both process_byte() and
     *       process_pulse() after pulse-to-byte conversion
     * 
     * @see ibus_decode()
     * @see add_input()
     */
    void _process_byte(uint32_t timestamp_us, uint8_t byte);

    /**
     * @brief Decode and validate a complete iBus frame
     * 
     * @details This static helper function validates the checksum of a complete 32-byte
     *          iBus frame and extracts channel values and failsafe status.
     *          
     *          Checksum Validation Algorithm:
     *          The iBus checksum is calculated such that summing all 32 bytes (including
     *          the 2-byte checksum at bytes 30-31) must equal 0xFFFF. This is equivalent
     *          to: checksum = 0xFFFF - sum(bytes[0..29])
     *          
     *          The validation adds all 32 bytes as 16-bit values and checks for 0xFFFF:
     *          sum = Σ(frame[i]) for i=0 to 31, treating each pair as little-endian uint16
     *          Valid if: (sum & 0xFFFF) == 0xFFFF
     *          
     *          Channel Extraction:
     *          - 14 channels stored in bytes 2-29 (2 bytes per channel, little-endian)
     *          - Each channel is 11-bit value (0-2047) stored in lower 11 bits of 16-bit word
     *          - Upper 5 bits unused (should be 0)
     *          
     *          Failsafe Detection:
     *          - Determined from header byte (frame[0]) and flags (frame[1])
     *          - Header 0x20: Normal operation
     *          - Header 0x40: Sensor frame (non-RC data)
     *          - Specific bit patterns indicate failsafe condition
     * 
     * @param[in]  frame Complete 32-byte iBus frame buffer
     * @param[out] values Array to receive 14 decoded channel values (11-bit, 0-2047 range)
     * @param[out] ibus_failsafe Pointer to bool set to true if failsafe detected, false otherwise
     * 
     * @return true if frame checksum is valid and channels decoded successfully
     * @return false if checksum validation failed (corrupted frame - discard data)
     * 
     * @warning Checksum validation is critical for flight safety. Invalid checksums indicate
     *          data corruption and decoded channel values MUST NOT be used. Using corrupted
     *          data could cause unpredictable vehicle behavior or loss of control.
     * 
     * @note This function is static and can be called independently for testing or
     *       offline frame analysis
     * 
     * @see _process_byte()
     */
    bool ibus_decode(const uint8_t frame[IBUS_FRAME_SIZE], uint16_t *values, bool *ibus_failsafe);

    /// SoftSerial decoder for pulse-based input, configured for 115200 bps, 8N1
    SoftSerial ss{115200, SoftSerial::SERIAL_CONFIG_8N1};

    /**
     * @brief Byte input state tracking for frame assembly
     * 
     * @details This structure maintains the state for accumulating bytes into complete
     *          iBus frames, including the frame buffer, current write position, and
     *          timing information for frame synchronization.
     */
    struct {
        /// Frame buffer accumulating 32-byte iBus frames
        uint8_t buf[IBUS_FRAME_SIZE];
        /// Current write offset in frame buffer (0-31, wraps to 0 after complete frame)
        uint8_t ofs;
        /// Timestamp of last received byte in microseconds (for timeout detection)
        uint32_t last_byte_us;
    } byte_input;
};

#endif  // AP_RCPROTOCOL_IBUS_ENABLED
