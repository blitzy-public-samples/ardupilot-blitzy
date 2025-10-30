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
 * @file AP_RCProtocol_DSM.h
 * @brief DSM/DSM2/DSMX RC protocol decoder for Spektrum receivers
 * 
 * @details This file implements decoding for the Spektrum DSM family of RC protocols
 *          used by Spektrum RC receivers including satellite receivers. The protocol
 *          family includes multiple variants with different channel resolutions:
 *          
 *          - DSM2: 10-bit channel resolution (1024 steps), 22ms frame rate
 *          - DSMX: 11-bit channel resolution (2048 steps), 11ms frame rate
 *          
 *          Frame Format:
 *          - 16 bytes per frame transmitted serially at 115200 baud
 *          - First 2 bytes: Header containing fade count and frame loss indicator
 *          - Remaining 14 bytes: 7 channel pairs (2 bytes per channel)
 *          - Each channel encoded as channel number (4 bits) + channel value (10 or 11 bits)
 *          
 *          The decoder automatically detects the protocol variant by analyzing received
 *          frames and determining whether channel values exceed 10-bit range. It supports
 *          multiple Spektrum satellite receivers and aggregates channels from all receivers.
 *          
 *          Receiver Binding:
 *          The bind procedure sends specific pulse patterns to put receivers into bind mode.
 *          This allows pairing new transmitters with receivers.
 *          
 * @note Frame synchronization is critical - decoder uses inter-frame gaps to detect frame boundaries
 * @warning Timing requirements are strict: frame gaps must be detected accurately for proper decoding
 * 
 * @author Andrew Tridgell and Siddharth Bharat Purohit
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

#include "AP_RCProtocol_config.h"

#if AP_RCPROTOCOL_DSM_ENABLED

#include "AP_RCProtocol_Backend.h"

#include "SoftSerial.h"

#define AP_DSM_MAX_CHANNELS 12  ///< Maximum number of channels supported by DSM protocol

/**
 * @class AP_RCProtocol_DSM
 * @brief Spektrum DSM/DSM2/DSMX RC protocol decoder backend
 * 
 * @details This class implements decoding for Spektrum DSM family protocols including
 *          DSM2 and DSMX variants. The decoder handles both serial byte streams and
 *          pulse-based input from satellite receivers.
 *          
 *          Protocol Variants:
 *          - DSM2: 10-bit channel resolution (values 0-1023), 22ms frame rate
 *          - DSMX: 11-bit channel resolution (values 0-2047), 11ms frame rate, frequency hopping
 *          
 *          The decoder automatically detects which variant is in use by analyzing the
 *          received channel values. If channel values exceed 1023, the decoder switches
 *          to 11-bit mode (DSMX).
 *          
 *          Frame Structure:
 *          Each 16-byte frame contains:
 *          - Byte 0-1: Header with fade count and frame loss flag
 *          - Byte 2-15: Seven 2-byte channel values (channel ID + value)
 *          
 *          Synchronization:
 *          Frame boundaries are detected using inter-frame gaps (>5ms typical).
 *          The decoder maintains sync state and resyncs if timing violations occur.
 *          
 *          Multiple Receivers:
 *          Supports Spektrum satellite receivers connected to different serial ports.
 *          Channels from multiple receivers are aggregated into a single channel set.
 *          
 * @note Inherits from AP_RCProtocol_Backend for integration with RC protocol framework
 * @warning Frame timing is critical - ensure serial port can maintain 115200 baud without gaps
 * 
 * Source: libraries/AP_RCProtocol/AP_RCProtocol_DSM.h
 */
class AP_RCProtocol_DSM : public AP_RCProtocol_Backend {
public:
    /**
     * @brief Construct a new DSM protocol decoder
     * @param[in] _frontend Reference to parent AP_RCProtocol frontend
     */
    AP_RCProtocol_DSM(AP_RCProtocol &_frontend) : AP_RCProtocol_Backend(_frontend) {}
    
    /**
     * @brief Process pulse-width input from DSM satellite receiver
     * 
     * @details Decodes DSM protocol from pulse timing on two pins (s0 and s1).
     *          This method is used when DSM receiver outputs are connected to
     *          GPIO pins configured for pulse width measurement rather than serial input.
     *          
     *          The pulse widths encode the serial data stream - each bit is represented
     *          by specific pulse timings following the DSM serial format (115200 baud, 8N1).
     * 
     * @param[in] width_s0 Pulse width on signal 0 pin in microseconds
     * @param[in] width_s1 Pulse width on signal 1 pin in microseconds
     * 
     * @note This method is called at high frequency when pulse capture is active
     * @warning Pulse timing must be accurate to microsecond level for reliable decoding
     */
    void process_pulse(uint32_t width_s0, uint32_t width_s1) override;
    
    /**
     * @brief Process a single byte from DSM serial stream
     * 
     * @details Accumulates bytes into 16-byte frames and decodes complete frames.
     *          Frame synchronization is achieved by detecting inter-frame gaps in
     *          the byte stream. When a gap exceeding the threshold is detected,
     *          the decoder resets to start of a new frame.
     *          
     *          Each complete 16-byte frame is parsed to extract channel values:
     *          - Bytes 0-1: Frame header (fade count, frame loss flag)
     *          - Bytes 2-15: Seven channel value pairs
     *          
     *          The decoder automatically determines whether channels use 10-bit
     *          (DSM2) or 11-bit (DSMX) resolution based on observed values.
     * 
     * @param[in] byte Single byte received from DSM serial stream
     * @param[in] baudrate Serial baudrate (typically 115200 for DSM)
     * 
     * @note Called for each byte received on the DSM serial input
     * @note Frame synchronization requires accurate inter-byte timing
     * @warning Baudrate must match DSM specification (115200) for proper decoding
     */
    void process_byte(uint8_t byte, uint32_t baudrate) override;
    
    /**
     * @brief Initiate DSM receiver bind sequence
     * 
     * @details Starts the bind procedure to pair a DSM transmitter with receivers.
     *          The bind sequence sends specific pulse patterns on the DSM output line
     *          that put Spektrum receivers into bind mode. The exact pulse sequence
     *          depends on the desired bind mode (DSM2 vs DSMX, number of channels).
     *          
     *          Bind Procedure:
     *          1. Send bind pulses at specific timing
     *          2. Power cycle receiver while bind signal active
     *          3. Receiver enters bind mode (indicated by LED)
     *          4. Transmitter initiates binding
     *          5. Binding completes when receiver accepts transmitter
     *          
     *          The bind state machine (bind_state) tracks progress through the sequence.
     * 
     * @note Bind procedure requires physical access to receiver and transmitter
     * @note User must power cycle the receiver during bind sequence
     * @warning Incorrect bind timing may fail to enter bind mode
     * @warning Bind pulses must be sent before receiver powers up
     */
    void start_bind(void) override;
    
    /**
     * @brief Periodic update for DSM decoder state
     * 
     * @details Called periodically to handle time-based state updates including:
     *          - Bind state machine progression
     *          - Timeout detection for frame reception
     *          - Failsafe detection when frames stop arriving
     *          - Format detection state updates
     *          
     *          If no frames are received for an extended period, the decoder
     *          may reset to desync state to allow resynchronization.
     * 
     * @note Called from main RC protocol update loop
     * @note Update rate affects timeout detection granularity
     */
    void update(void) override;

private:
    /**
     * @brief Internal byte processing with timestamp
     * 
     * @details Processes a single byte with associated timestamp for frame synchronization.
     *          Uses timestamp to detect inter-frame gaps and maintain frame alignment.
     *          Accumulates bytes into 16-byte frame buffer and triggers decoding when
     *          a complete frame is received.
     * 
     * @param[in] timestamp_ms Timestamp of byte reception in milliseconds
     * @param[in] byte Received byte from DSM serial stream
     * 
     * @note Gap detection threshold typically 5-10ms between frames
     */
    void _process_byte(uint32_t timestamp_ms, uint8_t byte);
    
    /**
     * @brief Decode accumulated DSM frame buffer
     * 
     * @details Decodes the current 16-byte frame buffer into channel values.
     *          Validates frame structure, extracts channel data, and updates
     *          the output channel array. Handles both 10-bit and 11-bit formats.
     * 
     * @note Called when a complete 16-byte frame has been accumulated
     * @note Uses channel_shift to determine 10-bit vs 11-bit decoding
     */
    void dsm_decode();
    
    /**
     * @brief Decode a single channel value from raw 16-bit word
     * 
     * @details Extracts channel number and channel value from a 2-byte channel word.
     *          The format is: [channel_id:4bits][channel_value:10 or 11 bits]
     *          The shift parameter determines whether to use 10-bit or 11-bit extraction.
     * 
     * @param[in]  raw      Raw 16-bit channel word from DSM frame
     * @param[in]  shift    Bit shift for channel value (10 or 11)
     * @param[out] channel  Decoded channel number (0-11)
     * @param[out] value    Decoded channel value (0-1023 for 10-bit, 0-2047 for 11-bit)
     * 
     * @return true if channel decoded successfully, false if channel number invalid
     * 
     * @note Channel numbers exceeding AP_DSM_MAX_CHANNELS are rejected
     */
    bool dsm_decode_channel(uint16_t raw, unsigned shift, unsigned *channel, unsigned *value);
    
    /**
     * @brief Automatically determine DSM protocol format (10-bit vs 11-bit)
     * 
     * @details Analyzes received frames to determine if channels are using 10-bit (DSM2)
     *          or 11-bit (DSMX) encoding. The detection works by attempting to decode
     *          channels with both formats and checking which produces valid channel numbers
     *          and reasonable channel values.
     *          
     *          Detection Algorithm:
     *          - Try decoding with 10-bit shift (DSM2 format)
     *          - Try decoding with 11-bit shift (DSMX format)
     *          - Accumulate confidence scores (cs10, cs11) for each format
     *          - Select format with highest confidence after sufficient samples
     *          
     *          Once format is determined, channel_shift is set accordingly.
     * 
     * @param[in] reset          Reset format detection and start over
     * @param[in] dsm_frame      16-byte DSM frame to analyze
     * @param[in] frame_channels Number of channels detected in frame
     * 
     * @note Requires multiple frames to reliably determine format
     * @note Format detection completes after ~4 frames typically
     */
    void dsm_guess_format(bool reset, const uint8_t dsm_frame[16], unsigned frame_channels);
    
    /**
     * @brief Parse single byte into DSM frame buffer
     * 
     * @details Accumulates bytes into frame buffer with frame synchronization.
     *          Detects frame boundaries using timing gaps and triggers frame
     *          decoding when complete frame received.
     * 
     * @param[in]  frame_time_ms Timestamp of current byte in milliseconds
     * @param[in]  b             Byte to parse
     * @param[out] values        Array to store decoded channel values
     * @param[out] num_values    Number of channels decoded
     * @param[in]  max_channels  Maximum channels to decode
     * 
     * @return true if a complete frame was decoded, false otherwise
     * 
     * @note Returns true only when a full 16-byte frame is decoded
     */
    bool dsm_parse_byte(uint32_t frame_time_ms, uint8_t b, uint16_t *values,
                        uint16_t *num_values, uint16_t max_channels);
    
    /**
     * @brief Decode a complete 16-byte DSM frame
     * 
     * @details Decodes a complete DSM frame into individual channel values.
     *          Parses frame header to check for frame loss indicator, then
     *          extracts all channel values from the frame. Automatically handles
     *          both 10-bit (DSM2) and 11-bit (DSMX) channel formats.
     *          
     *          Frame Structure:
     *          - Bytes 0-1: Header (fade count in byte 0, frame loss flag in byte 1)
     *          - Bytes 2-15: Seven 2-byte channel values
     *          
     *          Frame Loss Detection:
     *          Byte 1 bit patterns indicate signal quality:
     *          - 0x00: Normal frame, good signal
     *          - Non-zero: Fade count or frame loss indicator
     * 
     * @param[in]  frame_time_ms Timestamp when frame completed in milliseconds
     * @param[in]  dsm_frame     16-byte DSM frame buffer
     * @param[out] values        Array to receive decoded channel values (0-1023 or 0-2047)
     * @param[out] num_values    Number of channels successfully decoded
     * @param[in]  max_values    Maximum size of values array
     * 
     * @return true if frame decoded successfully, false if frame invalid
     * 
     * @note Frame loss flag in header can indicate signal quality issues
     * @note Not all channels may be present in every frame (depends on transmitter config)
     * @warning Invalid frames should be discarded to prevent using corrupted channel data
     */
    bool dsm_decode(uint32_t frame_time_ms, const uint8_t dsm_frame[16],
                    uint16_t *values, uint16_t *num_values, uint16_t max_values);

    /**
     * @brief Channel resolution indicator
     * @details Bit shift value determining channel value resolution:
     *          - 0: Unknown/not yet determined
     *          - 10: 10-bit channels (DSM2 format, values 0-1023)
     *          - 11: 11-bit channels (DSMX format, values 0-2047)
     *          
     *          Set automatically by dsm_guess_format() after analyzing received frames.
     */
    uint8_t channel_shift;

    // Format guessing state - used to auto-detect 10-bit vs 11-bit protocol
    uint32_t	cs10;      ///< Confidence score for 10-bit (DSM2) format
    uint32_t	cs11;      ///< Confidence score for 11-bit (DSMX) format
    uint32_t samples;  ///< Number of frames analyzed for format detection

    /**
     * @brief Bind state machine states
     * @details State machine for DSM receiver bind procedure:
     *          - BIND_STATE_NONE: Not binding, normal operation
     *          - BIND_STATE1-4: Sequential bind pulse states
     *          
     *          The bind sequence sends specific pulse patterns to receiver
     *          to initiate pairing mode. Each state represents a phase in
     *          the pulse timing sequence.
     */
    enum {
        BIND_STATE_NONE,   ///< Normal operation, not in bind mode
        BIND_STATE1,       ///< Bind sequence state 1
        BIND_STATE2,       ///< Bind sequence state 2
        BIND_STATE3,       ///< Bind sequence state 3
        BIND_STATE4,       ///< Bind sequence state 4 (final)
    } bind_state;
    
    uint32_t bind_last_ms;      ///< Timestamp of last bind state change in milliseconds
    uint32_t bind_mode_saved;   ///< Saved bind mode configuration

    /**
     * @brief Previous frame's channel values
     * @details Stores channel values from previous frame for comparison and failsafe.
     *          Used to detect channel value changes and to hold last known good
     *          values if frames are lost. Array sized for maximum DSM channels (12).
     *          Values are in raw DSM format (0-1023 for 10-bit, 0-2047 for 11-bit).
     */
    uint16_t last_values[AP_DSM_MAX_CHANNELS];

    /**
     * @brief Byte input accumulation buffer
     * @details Accumulates received bytes into complete 16-byte DSM frames.
     *          - buf: 16-byte frame buffer
     *          - ofs: Current write offset in buffer (0-15)
     *          
     *          Buffer resets to offset 0 when frame gap detected or after
     *          complete frame is processed.
     */
    struct {
        uint8_t buf[16];   ///< 16-byte DSM frame accumulation buffer
        uint8_t ofs;       ///< Current buffer write offset (0-15)
    } byte_input;

    /**
     * @brief DSM frame synchronization state
     * @details Tracks whether decoder is synchronized to frame boundaries:
     *          - DSM_DECODE_STATE_DESYNC: Not synchronized, searching for frame start
     *          - DSM_DECODE_STATE_SYNC: Synchronized, accumulating frame bytes
     *          
     *          Decoder enters SYNC state when frame gap detected (indicates frame boundary).
     *          Decoder may return to DESYNC if timing violations or corrupt frames detected.
     */
    enum DSM_DECODE_STATE {
        DSM_DECODE_STATE_DESYNC = 0,  ///< Searching for frame synchronization
        DSM_DECODE_STATE_SYNC         ///< Synchronized to frame boundaries
    } dsm_decode_state;

    uint32_t last_frame_time_ms;  ///< Timestamp of last complete frame in milliseconds
    uint32_t last_rx_time_ms;     ///< Timestamp of last byte received in milliseconds
    uint16_t chan_count;          ///< Number of channels decoded in last frame

    /**
     * @brief Software serial interface for DSM output (bind pulses)
     * @details Software-based serial port configured for DSM communication parameters:
     *          - Baudrate: 115200 bps (standard for DSM protocol)
     *          - Format: 8 data bits, no parity, 1 stop bit (8N1)
     *          
     *          Used primarily for sending bind pulses to receivers. During normal
     *          reception, hardware serial is typically used for better timing accuracy.
     *          
     * @note Software serial provides flexibility for bind pulse generation on any GPIO
     * @warning Software serial timing may be less accurate than hardware UART
     */
    SoftSerial ss{115200, SoftSerial::SERIAL_CONFIG_8N1};
};

#endif  // AP_RCPROTOCOL_DSM_ENABLED
