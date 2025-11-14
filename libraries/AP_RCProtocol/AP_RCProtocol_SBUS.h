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
 * @file AP_RCProtocol_SBUS.h
 * @brief FrSky S.Bus protocol decoder for digital RC receivers
 * 
 * @details This file implements the S.Bus (Serial Bus) protocol decoder for ArduPilot's
 *          RC input system. S.Bus is a digital RC protocol developed by FrSky/Futaba
 *          for high-speed, low-latency transmission of RC channel data.
 * 
 *          S.Bus Protocol Specifications:
 *          - 25-byte frame structure (1 header + 22 data + 1 flags + 1 footer)
 *          - 16 proportional channels at 11-bit resolution (0-2047 raw, typically 172-1811μs)
 *          - Serial configuration: 100000 baud, 8E2I (8 data bits, even parity, 2 stop bits, inverted)
 *          - Frame rate: ~7ms (140Hz) or ~14ms (70Hz) depending on transmitter
 *          - Inverted UART signal (logic level inversion required on most hardware)
 *          - Frame synchronization via inter-frame gaps (minimum HAL_SBUS_FRAME_GAP typically 2ms)
 * 
 *          Frame Format (25 bytes):
 *          - Byte 0: Header (0x0F)
 *          - Bytes 1-22: Channel data (16 channels × 11 bits packed into 22 bytes)
 *          - Byte 23: Flags (bit 0: channel 17, bit 1: channel 18, bit 2: frame lost, bit 3: failsafe)
 *          - Byte 24: Footer (0x00)
 * 
 *          This implementation supports both inverted (standard S.Bus) and non-inverted
 *          (SBUS_NI variant) signal formats. Non-inverted S.Bus is used by certain
 *          receivers that pre-invert the signal or connect via inverting hardware.
 * 
 * @note The decoder requires accurate frame gap detection for synchronization. Without
 *       proper gap detection, the decoder may lose sync and produce invalid channel data.
 * 
 * @warning Timing-critical implementation: S.Bus frames must be processed with proper
 *          timing to detect frame boundaries. Missed frame gaps can cause channel value
 *          corruption or complete loss of RC input.
 * 
 * @see AP_RCProtocol_Backend for the base protocol interface
 * @see SoftSerial for pulse-width to serial byte conversion
 */

#pragma once

#include "AP_RCProtocol_config.h"

#if AP_RCPROTOCOL_SBUS_ENABLED

#include "AP_RCProtocol.h"
#include "SoftSerial.h"

/**
 * @class AP_RCProtocol_SBUS
 * @brief S.Bus protocol decoder backend for RC input processing
 * 
 * @details This class implements the FrSky/Futaba S.Bus digital RC protocol decoder.
 *          S.Bus transmits 16 proportional channels plus 2 digital channels over a
 *          single serial connection at 100000 baud with inverted signal levels.
 * 
 *          The decoder operates in two modes:
 *          1. Byte-based: Direct processing of serial bytes from UART hardware
 *          2. Pulse-based: Conversion of pulse-width measurements to serial bytes via SoftSerial
 * 
 *          Protocol Features:
 *          - 16 proportional channels with 11-bit resolution (2048 discrete values)
 *          - Channel value range: 0-2047 raw (typically mapped to 172-1811μs PWM equivalent)
 *          - 2 digital channels (channels 17-18) for binary on/off control
 *          - Failsafe flag indicating transmitter signal loss
 *          - Frame-lost flag indicating receiver signal quality issues
 *          - Even parity error detection on each byte
 * 
 *          Timing Requirements:
 *          - Baudrate: 100000 bps (100 kbaud)
 *          - Frame period: ~7ms (140Hz) or ~14ms (70Hz)
 *          - Inter-frame gap: Minimum HAL_SBUS_FRAME_GAP (typically 2000μs)
 *          - Byte timeout: Used to detect frame boundaries
 * 
 *          Hardware Configuration:
 *          - Standard S.Bus: Inverted UART signal (requires hardware inverter or software inversion)
 *          - Non-inverted S.Bus (SBUS_NI): Direct UART connection for pre-inverted signals
 *          - Serial format: 8 data bits, even parity, 2 stop bits (8E2)
 * 
 *          Thread Safety: This decoder is called from the RC input thread/interrupt context.
 *          Frame synchronization relies on detecting gaps between frames, so timing accuracy
 *          is critical for reliable operation.
 * 
 * @note Some receivers output non-inverted S.Bus (SBUS_NI), which requires the inverted
 *       flag to be set to false during initialization.
 * 
 * @warning Frame synchronization depends on accurate byte timing. Processing delays or
 *          missed interrupts can cause loss of synchronization and corrupt channel values.
 *          Always verify S.Bus operation with oscilloscope when debugging signal issues.
 */
class AP_RCProtocol_SBUS : public AP_RCProtocol_Backend {
public:
    /**
     * @brief Construct S.Bus protocol decoder
     * 
     * @details Initializes the S.Bus decoder with specified signal inversion setting.
     *          The inverted parameter determines whether the decoder expects standard
     *          inverted S.Bus signals or non-inverted SBUS_NI variant.
     * 
     * @param[in] _frontend      Reference to the AP_RCProtocol frontend manager
     * @param[in] inverted       True for standard inverted S.Bus, false for SBUS_NI (non-inverted)
     * @param[in] configured_baud Configured baud rate for the protocol (typically 100000)
     * 
     * @note Standard S.Bus uses inverted UART signaling (idle low instead of idle high).
     *       Set inverted=true for normal S.Bus receivers, inverted=false for receivers
     *       with built-in signal inversion or when using external inverting hardware.
     */
    AP_RCProtocol_SBUS(AP_RCProtocol &_frontend, bool inverted, uint32_t configured_baud);
    /**
     * @brief Process pulse-width measurements for S.Bus signal reconstruction
     * 
     * @details Converts pulse-width measurements into serial byte stream for S.Bus decoding.
     *          This method is used when S.Bus input is captured as pulse widths rather than
     *          direct serial bytes (e.g., on hardware without spare UART or with GPIO input).
     * 
     *          The SoftSerial decoder reconstructs the serial bit stream from pulse timing:
     *          - Measures high and low pulse durations
     *          - Reconstructs start bit, data bits, parity bit, and stop bits
     *          - Generates serial bytes for processing by _process_byte()
     * 
     *          S.Bus timing at 100000 baud:
     *          - Bit period: 10μs per bit
     *          - Byte period: ~120μs (1 start + 8 data + 1 parity + 2 stop = 12 bits)
     *          - Frame period: ~3000μs (25 bytes × 120μs)
     * 
     * @param[in] width_s0 Duration of first pulse state in microseconds (μs)
     * @param[in] width_s1 Duration of second pulse state in microseconds (μs)
     * 
     * @note This method uses SoftSerial to perform UART protocol decoding in software.
     *       Pulse timing accuracy directly affects decoding reliability. Jitter or
     *       measurement errors can cause byte framing errors and lost frames.
     * 
     * @warning Requires accurate microsecond-resolution timing. Interrupt latency or
     *          timer wraparound issues can corrupt the decoded byte stream.
     * 
     * @see SoftSerial for pulse-to-byte conversion implementation
     */
    void process_pulse(uint32_t width_s0, uint32_t width_s1) override;
    
    /**
     * @brief Process incoming serial byte from S.Bus receiver
     * 
     * @details Processes a single byte received from the S.Bus serial stream. Bytes are
     *          accumulated into 25-byte frames, synchronized using inter-frame gaps.
     *          Complete frames are validated and decoded to extract channel values.
     * 
     *          Frame Synchronization:
     *          - Detects frame boundaries by measuring time since last byte
     *          - Resets frame buffer when gap exceeds HAL_SBUS_FRAME_GAP (typically 2000μs)
     *          - Validates frame header (0x0F) and footer (0x00) bytes
     *          - Checks even parity on each byte (handled by UART hardware)
     * 
     *          Decoding Process:
     *          1. Accumulate bytes into 25-byte buffer
     *          2. Detect frame completion via gap or buffer full
     *          3. Validate header (0x0F) and footer (0x00)
     *          4. Extract 16 channels from packed 11-bit data
     *          5. Extract failsafe and frame-lost flags from byte 23
     *          6. Update RC channel values if frame valid
     * 
     * @param[in] byte     Received serial byte from S.Bus UART (0x00-0xFF)
     * @param[in] baudrate Current baud rate in bits per second (should be 100000 for S.Bus)
     * 
     * @note S.Bus requires exactly 100000 baud. Other baud rates will not decode properly.
     *       The decoder relies on byte timing gaps for frame synchronization, so consistent
     *       timing is essential for reliable operation.
     * 
     * @warning Frame synchronization will fail if bytes arrive with irregular timing or if
     *          the inter-frame gap is not properly detected. This can occur with UART FIFO
     *          buffering, interrupt latency, or scheduling delays. Ensure the HAL provides
     *          accurate byte timestamps for proper gap detection.
     * 
     * @see _process_byte() for internal byte processing with timestamp
     */
    void process_byte(uint8_t byte, uint32_t baudrate) override;

    /**
     * @brief Decode complete 25-byte S.Bus frame into channel values
     * 
     * @details Static helper function that decodes a validated S.Bus frame into individual
     *          channel values and status flags. This function performs the actual unpacking
     *          of the 11-bit channel data from the frame's packed byte representation.
     * 
     *          Frame Structure (25 bytes):
     *          - Byte 0: Header 0x0F (frame start marker)
     *          - Bytes 1-22: Channel data (16 channels × 11 bits = 176 bits = 22 bytes)
     *          - Byte 23: Flags byte
     *                     bit 0: Channel 17 (digital)
     *                     bit 1: Channel 18 (digital)
     *                     bit 2: Frame lost flag (poor signal quality)
     *                     bit 3: Failsafe flag (signal loss from transmitter)
     *                     bits 4-7: Reserved
     *          - Byte 24: Footer 0x00 (frame end marker)
     * 
     *          Channel Data Encoding:
     *          - 16 channels packed into 22 bytes as 11-bit values
     *          - Channel resolution: 0-2047 (11 bits)
     *          - Typical mapping: 0-2047 raw → 172-1811μs PWM equivalent
     *          - Channels packed LSB-first across byte boundaries
     * 
     *          Decoding Algorithm:
     *          1. Validate header byte == 0x0F
     *          2. Extract 16 channels using decode_11bit_channels() helper
     *          3. Extract digital channels 17-18 from flags byte
     *          4. Extract failsafe flag (indicates TX signal loss)
     *          5. Validate footer byte == 0x00
     * 
     * @param[in]  frame          Pointer to 25-byte S.Bus frame buffer
     * @param[out] values         Array to store decoded channel values (0-2047 per channel)
     * @param[out] num_values     Number of channels successfully decoded (typically 16 or 18)
     * @param[out] sbus_failsafe  Failsafe flag status (true = transmitter signal lost)
     * @param[in]  max_values     Maximum number of channels to decode (array size of values)
     * 
     * @return true if frame decoded successfully (valid header/footer), false if frame invalid
     * 
     * @note This is a static method that can be called independently for testing or from
     *       other S.Bus-related code. It performs pure frame decoding without state management.
     * 
     * @note Channel values are raw 11-bit values (0-2047). The frontend is responsible for
     *       scaling these to PWM microsecond values. Standard S.Bus mapping uses:
     *       - Min: 172-200μs (raw value ~0-200)
     *       - Center: 992μs (raw value ~1024)
     *       - Max: 1792-1811μs (raw value ~1800-2047)
     * 
     * @warning Caller must ensure frame buffer contains exactly 25 bytes and values array
     *          has space for at least max_values entries. Buffer overruns will cause memory
     *          corruption. Always validate max_values >= 16 for standard S.Bus operation.
     * 
     * @see decode_11bit_channels() for the bit-unpacking implementation
     */
    static bool sbus_decode(const uint8_t frame[25], uint16_t *values, uint16_t *num_values,
                            bool &sbus_failsafe, uint16_t max_values);
    
private:
    /**
     * @brief Internal byte processing with timestamp for frame synchronization
     * 
     * @details Processes incoming S.Bus bytes with accurate timing information for
     *          frame boundary detection. Accumulates bytes into the 25-byte frame
     *          buffer and detects frame completion via inter-frame gaps.
     * 
     *          Frame Synchronization Algorithm:
     *          1. Check time since last byte (timestamp_us - byte_input.last_byte_us)
     *          2. If gap > HAL_SBUS_FRAME_GAP (typically 2000μs), reset buffer (new frame start)
     *          3. Store byte in buffer at current offset
     *          4. Increment offset
     *          5. When buffer reaches 25 bytes, attempt decode
     *          6. Reset buffer for next frame
     * 
     * @param[in] timestamp_us Current timestamp in microseconds (μs) for gap detection
     * @param[in] byte         Received S.Bus serial byte (0x00-0xFF)
     * 
     * @note This method is called internally by process_byte() and by SoftSerial decoder
     *       when reconstructing bytes from pulse-width measurements. Accurate timestamps
     *       are essential for proper frame synchronization.
     */
    void _process_byte(uint32_t timestamp_us, uint8_t byte);

    /**
     * @brief Signal inversion flag
     * 
     * @details True for standard inverted S.Bus (idle low), false for non-inverted
     *          SBUS_NI variant (idle high). Standard S.Bus uses inverted serial signaling
     *          where the UART idle state is logic low instead of the normal logic high.
     *          
     *          This requires either hardware signal inversion or software bit inversion
     *          depending on the HAL implementation. Some receivers output non-inverted
     *          S.Bus (SBUS_NI) which requires this flag to be false.
     */
    bool inverted;
    
    /**
     * @brief Software serial decoder for pulse-based S.Bus input
     * 
     * @details SoftSerial instance used to reconstruct serial byte stream from pulse-width
     *          measurements. This enables S.Bus decoding on hardware without dedicated UART
     *          or when S.Bus signal is captured via GPIO with pulse timing measurements.
     *          
     *          The SoftSerial decoder:
     *          - Measures pulse widths to reconstruct bit timing
     *          - Detects start bit, data bits, parity bit, and stop bits
     *          - Handles 100000 baud timing (10μs per bit)
     *          - Generates decoded bytes for frame processing
     * 
     * @note Only used in pulse-based decoding mode via process_pulse(). When process_byte()
     *       is called directly from UART hardware, SoftSerial is not involved.
     */
    SoftSerial ss;
    
    /**
     * @brief Saved pulse width for SoftSerial processing
     * 
     * @details Stores pulse width state between process_pulse() calls for the SoftSerial
     *          decoder. This allows the decoder to track pulse timing across multiple
     *          invocations as pulse measurements arrive.
     * 
     * @note Units: microseconds (μs)
     */
    uint32_t saved_width;

    /**
     * @brief S.Bus frame reception state
     * 
     * @details Structure maintaining the current state of S.Bus frame reception, including
     *          the frame buffer, current write position, and timing information for frame
     *          gap detection.
     * 
     *          Frame Buffer Management:
     *          - buf[25]: Accumulates incoming bytes for one complete S.Bus frame
     *          - ofs: Current write position (0-24), reset to 0 after frame completion
     *          - last_byte_us: Timestamp of last received byte for gap detection
     * 
     *          The frame gap (time between last_byte_us and current byte) is used to
     *          detect frame boundaries. When gap exceeds HAL_SBUS_FRAME_GAP (typically
     *          2000μs), the decoder resets the buffer and starts accumulating a new frame.
     */
    struct {
        /**
         * @brief 25-byte S.Bus frame buffer
         * 
         * @details Accumulates incoming S.Bus bytes for frame decoding.
         *          Frame format: [0x0F][22 data bytes][flags][0x00]
         */
        uint8_t buf[25];
        
        /**
         * @brief Current buffer write offset (0-24)
         * 
         * @details Index of next byte to write in frame buffer. Reset to 0 when frame
         *          is complete or when frame gap is detected. Valid range: 0-24.
         */
        uint8_t ofs;
        
        /**
         * @brief Timestamp of last received byte in microseconds (μs)
         * 
         * @details Used to calculate inter-byte gap for frame synchronization.
         *          When (current_time - last_byte_us) > HAL_SBUS_FRAME_GAP, a new
         *          frame is detected and the buffer is reset.
         */
        uint32_t last_byte_us;
    } byte_input;
};

#endif  // AP_RCPROTOCOL_SBUS_ENABLED
