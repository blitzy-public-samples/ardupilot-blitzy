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
 * @file AP_RCProtocol_FPort2.h
 * @brief FrSky FPort2 RC protocol decoder with bidirectional telemetry support
 * 
 * @details This file implements the FrSky FPort2 protocol, which is the successor
 *          to the original FPort protocol. FPort2 provides enhanced features including:
 *          - Improved telemetry scheduling with better timing control
 *          - Additional frame types for expanded functionality
 *          - Enhanced bidirectional communication between receiver and flight controller
 *          - Backward compatibility considerations with FPort receivers
 * 
 *          Protocol Characteristics:
 *          - Baudrate: Typically 115200 bps (standard for FPort2)
 *          - Channel encoding: 11-bit channel values (range 0-2047)
 *          - Frame format: Similar byte-stuffing mechanism to FPort
 *          - Control frame size: 38 bytes (FPORT2_CONTROL_FRAME_SIZE)
 *          - Telemetry: Bidirectional with improved packet scheduling
 * 
 *          FPort2 Enhancements Over FPort:
 *          - Improved telemetry timing: More predictable telemetry slot allocation
 *          - Enhanced packet types: Support for additional telemetry frame formats
 *          - Better scheduling: Reduced telemetry conflicts and improved throughput
 *          - Downlink support: Enhanced receiver-to-FC communication
 * 
 *          Frame Structure:
 *          - Uses byte-stuffing for data transparency (0x7E frame delimiter, 0x7D escape)
 *          - Control frames: Contain 16 RC channels plus additional control data
 *          - Downlink frames: Receiver status and telemetry pass-through data
 *          - Checksum: CRC-based integrity verification for all frames
 * 
 * @note FPort2 uses inverted or non-inverted serial depending on receiver configuration
 * @warning FPort2 requires specific receiver firmware versions for full compatibility.
 *          Older FrSky receivers may require firmware updates to support FPort2 features.
 *          Not all FPort receivers support FPort2 protocol enhancements.
 * 
 * @see AP_RCProtocol_Backend Base class for RC protocol decoders
 * @see AP_Frsky_SPort FrSky SPort telemetry protocol implementation
 */

#pragma once

#include "AP_RCProtocol_config.h"

#if AP_RCPROTOCOL_FPORT2_ENABLED

#include "AP_RCProtocol.h"
#include "SoftSerial.h"
#include <AP_Frsky_Telem/AP_Frsky_SPort.h>

/** @brief FPort2 control frame size in bytes (includes header, channels, flags, checksum) */
#define FPORT2_CONTROL_FRAME_SIZE 38

/** @brief Forward declaration of FPort2 frame structure */
struct FPort2_Frame;

/**
 * @class AP_RCProtocol_FPort2
 * @brief FrSky FPort2 RC protocol backend decoder with enhanced telemetry
 * 
 * @details This class implements decoding of the FrSky FPort2 protocol, which is
 *          an evolution of the original FPort protocol with improvements in telemetry
 *          scheduling and frame handling. FPort2 provides bidirectional communication
 *          between FrSky receivers and the flight controller.
 * 
 *          Key Features:
 *          - 16-channel RC input decoding with 11-bit resolution per channel
 *          - Bidirectional telemetry using SPort protocol encapsulation
 *          - Improved telemetry timing compared to original FPort
 *          - Support for both inverted and non-inverted UART signals
 *          - Software serial decoding for hardware compatibility
 *          - Control frame and downlink frame processing
 * 
 *          Protocol Operation:
 *          1. Receives serial data at 115200 bps (standard FPort2 baudrate)
 *          2. Processes incoming bytes with byte-stuffing decode
 *          3. Validates frames using CRC checksum
 *          4. Decodes control frames to extract RC channel values
 *          5. Processes downlink frames for receiver status
 *          6. Schedules telemetry responses during allowed windows
 * 
 *          Frame Types:
 *          - Control frames: RC channel data from transmitter to receiver to FC
 *          - Downlink frames: Receiver status and configuration data
 *          - Telemetry frames: Flight controller data back to transmitter
 * 
 *          Timing Characteristics:
 *          - Control frame interval: Typically 7-9 ms depending on radio mode
 *          - Telemetry slot timing: Improved over FPort with better scheduling (milliseconds)
 *          - Byte timeout: Used to detect frame boundaries and synchronization loss
 * 
 *          Hardware Requirements:
 *          - UART capable of 115200 bps
 *          - Support for inverted or non-inverted serial (receiver-dependent)
 *          - Software serial implementation handles signal inversion
 * 
 * @note This backend can operate in inverted or non-inverted mode based on receiver wiring
 * @warning FPort2 protocol requires receivers with FPort2-compatible firmware. Standard
 *          FPort receivers may not support enhanced features. Check receiver firmware version
 *          and update if necessary for full FPort2 compatibility.
 * 
 * @see AP_RCProtocol_Backend Base class providing RC protocol interface
 * @see AP_Frsky_SPort SPort telemetry protocol for bidirectional communication
 */
class AP_RCProtocol_FPort2 : public AP_RCProtocol_Backend {
public:
    /**
     * @brief Construct FPort2 protocol decoder backend
     * 
     * @details Initializes the FPort2 decoder with specified signal inversion mode.
     *          The inverted parameter determines whether the UART signal needs to be
     *          inverted, which depends on the receiver hardware and wiring configuration.
     * 
     *          Initialization includes:
     *          - Setting up software serial decoder at 115200 bps, 8N1 configuration
     *          - Configuring signal inversion mode for hardware compatibility
     *          - Initializing frame reception state machine
     *          - Setting up telemetry data structures
     * 
     * @param[in] _frontend Reference to the AP_RCProtocol frontend manager
     * @param[in] inverted  Signal inversion flag:
     *                      - true: UART signal is inverted (requires software inversion)
     *                      - false: UART signal is non-inverted (standard logic levels)
     * 
     * @note The inverted parameter is typically determined by receiver hardware design.
     *       Some FrSky receivers output inverted serial, others use standard levels.
     */
    AP_RCProtocol_FPort2(AP_RCProtocol &_frontend, bool inverted);
    
    /**
     * @brief Process pulse width input for software serial decoding
     * 
     * @details Processes raw pulse width measurements to decode FPort2 serial data
     *          when connected to a GPIO pin rather than a hardware UART. This method
     *          implements software serial decoding to reconstruct the byte stream
     *          from pulse timing measurements.
     * 
     *          Software serial operation:
     *          1. Measures pulse widths of high and low periods
     *          2. Reconstructs bit values from timing (115200 bps bit period ~8.68 μs)
     *          3. Assembles bits into bytes (8N1 format: 1 start, 8 data, 1 stop)
     *          4. Passes decoded bytes to process_byte() for protocol handling
     * 
     * @param[in] width_s0 Width of state 0 (low) pulse in microseconds
     * @param[in] width_s1 Width of state 1 (high) pulse in microseconds
     * 
     * @note This is called by the HAL when FPort2 is connected to a GPIO pin
     *       instead of a hardware UART. Requires precise timing measurements.
     * @note Pulse widths are in microseconds (μs) for 115200 bps serial timing
     * 
     * @see SoftSerial Software serial decoder implementation
     */
    void process_pulse(uint32_t width_s0, uint32_t width_s1) override;
    
    /**
     * @brief Process incoming FPort2 protocol byte
     * 
     * @details Main entry point for FPort2 protocol decoding. Processes each received
     *          byte through the FPort2 frame decoder state machine. Handles byte-stuffing,
     *          frame synchronization, checksum validation, and frame type dispatch.
     * 
     *          Processing stages:
     *          1. Byte-stuffing decode: Handle 0x7D escape sequences and 0x7E delimiters
     *          2. Frame synchronization: Detect frame boundaries and recover from errors
     *          3. Buffer management: Accumulate bytes into frame buffer
     *          4. Frame validation: Verify checksum when complete frame received
     *          5. Frame dispatch: Route to control or downlink handler based on frame type
     *          6. Timeout handling: Reset state on inter-byte timeout (frame boundary)
     * 
     *          Frame Format (byte-stuffed):
     *          - 0x7E: Frame delimiter (start/end marker)
     *          - 0x7D: Escape character (next byte XORed with 0x20)
     *          - Control frames: 38 bytes containing 16 channels (11-bit each) plus flags
     *          - Downlink frames: Receiver status and telemetry pass-through
     * 
     * @param[in] byte     Received byte from UART or software serial decoder
     * @param[in] baudrate Serial baudrate in bits per second (typically 115200 for FPort2)
     * 
     * @note This method is called at the rate bytes are received (115200 bps = ~11520 bytes/sec)
     * @note Baudrate parameter used for timing calculations and protocol validation
     * @note Method handles both control frames (RC channels) and downlink frames (receiver status)
     * 
     * @see decode_control() Decodes control frames to extract RC channel values
     * @see decode_downlink() Processes downlink frames for receiver status
     * @see check_checksum() Validates frame integrity
     */
    void process_byte(uint8_t byte, uint32_t baudrate) override;

private:
    /**
     * @brief Decode FPort2 control frame to extract RC channel values
     * 
     * @details Processes a validated control frame to extract 16 RC channel values
     *          encoded as 11-bit values. Control frames contain:
     *          - Frame type identifier (control frame marker)
     *          - 16 channels × 11 bits = 176 bits of channel data (packed into 22 bytes)
     *          - RSSI value (signal strength indicator)
     *          - Flags byte (failsafe, frame lost, etc.)
     * 
     *          Channel Value Encoding:
     *          - 11-bit values per channel: Range 0-2047 (0x000 to 0x7FF)
     *          - Center value typically 1024 (0x400)
     *          - Packed bit layout: Channels stored with bit-level packing for efficiency
     * 
     *          Extracted data is passed to frontend via add_input() for RC channel processing.
     * 
     * @param[in] frame Reference to validated FPort2_Frame containing control data
     * 
     * @note Control frames are received at radio frame rate (typically 7-9 ms intervals)
     * @note Method updates internal chan_count to reflect number of channels decoded
     * 
     * @see add_input() Frontend method to process decoded channel values
     */
    void decode_control(const FPort2_Frame &frame);
    
    /**
     * @brief Decode FPort2 downlink frame for receiver status and telemetry
     * 
     * @details Processes downlink frames sent from the receiver to the flight controller.
     *          Downlink frames can contain:
     *          - Receiver status information (battery, signal quality)
     *          - Telemetry pass-through data from receiver sensors
     *          - Configuration acknowledgments
     *          - Diagnostic information
     * 
     *          FPort2 Enhancement: Improved downlink scheduling compared to original FPort,
     *          providing more reliable receiver status updates and better telemetry throughput.
     * 
     * @param[in] frame Reference to validated FPort2_Frame containing downlink data
     * 
     * @note Downlink frames may contain SPort telemetry packets to be queued for transmission
     * @note Updates telem_data structure if telemetry packet received
     */
    void decode_downlink(const FPort2_Frame &frame);
    
    /**
     * @brief Validate FPort2 frame checksum for data integrity
     * 
     * @details Verifies the CRC checksum of the received frame to ensure data integrity.
     *          FPort2 uses CRC-based error detection to identify corrupted frames.
     * 
     *          Checksum Calculation:
     *          - Computed over all frame data bytes (excluding delimiter and checksum itself)
     *          - Algorithm matches FrSky FPort2 specification
     *          - Failed checksum causes frame to be discarded
     * 
     * @return true if checksum is valid, false if frame is corrupted
     * 
     * @note Invalid frames are silently discarded - no error reporting to prevent log spam
     * @note Checksum failures may indicate electrical noise, baudrate mismatch, or signal issues
     */
    bool check_checksum(void);

    /**
     * @brief Internal byte processing with timestamp tracking
     * 
     * @details Lower-level byte processing that includes microsecond timestamp for
     *          timeout detection and frame boundary identification. Used internally
     *          by process_byte() and process_pulse() methods.
     * 
     * @param[in] timestamp_us Current timestamp in microseconds (from AP_HAL::micros())
     * @param[in] byte         Received byte to process
     * 
     * @note Timestamps used to detect inter-frame gaps (timeouts indicate frame boundaries)
     */
    void _process_byte(uint32_t timestamp_us, uint8_t byte);
    
    /** @brief Software serial decoder configured for 115200 bps, 8 data bits, no parity, 1 stop bit */
    SoftSerial ss{115200, SoftSerial::SERIAL_CONFIG_8N1};
    
    /** @brief Saved pulse width for software serial state tracking (microseconds) */
    uint32_t saved_width;

    /**
     * @brief Frame reception state machine and buffer
     * 
     * @details Maintains the current state of frame reception including:
     *          - buf[]: Buffer for accumulating frame bytes (38 bytes for control frames)
     *          - ofs: Current write offset in buffer (index of next byte position)
     *          - last_byte_us: Timestamp of last received byte (microseconds) for timeout detection
     *          - control_len: Expected length of current frame
     *          - is_downlink: Flag indicating if current frame is downlink type (vs control)
     * 
     * @note Buffer reset on timeout or frame completion
     * @note Timeout detection: Gap > ~1ms indicates frame boundary at 115200 bps
     */
    struct {
        uint8_t buf[FPORT2_CONTROL_FRAME_SIZE];  ///< Frame buffer (38 bytes max)
        uint8_t ofs;                              ///< Current buffer write position
        uint32_t last_byte_us;                    ///< Timestamp of last byte (μs) for timeout
        uint8_t control_len;                      ///< Expected frame length in bytes
        bool is_downlink;                         ///< True if frame is downlink type
    } byte_input;

    /** @brief Number of RC channels decoded from last valid control frame (typically 16) */
    uint8_t chan_count;

    /** 
     * @brief Signal inversion flag (const - set at construction)
     * 
     * @details Indicates whether UART signal requires inversion for this receiver.
     *          FPort2 receivers may use inverted or non-inverted serial depending on
     *          hardware design. This flag is immutable after construction.
     * 
     * @note Inverted mode: Logic 0 = high voltage, logic 1 = low voltage
     * @note Non-inverted mode: Standard UART logic levels
     */
    const bool inverted;

    /**
     * @brief Telemetry data buffer for outgoing SPort packets
     * 
     * @details Stores telemetry packet ready for transmission during next telemetry window.
     *          FPort2 telemetry uses SPort protocol encapsulation for compatibility with
     *          FrSky telemetry infrastructure.
     * 
     *          Structure members:
     *          - available: True when packet is queued and ready for transmission
     *          - packet: SPort telemetry packet data (sensor ID, data ID, value)
     * 
     *          FPort2 Enhancement: Improved telemetry scheduling provides more predictable
     *          timing for telemetry packet transmission compared to original FPort, reducing
     *          packet collisions and improving telemetry update rates (timing in milliseconds).
     * 
     * @note Telemetry packets transmitted during designated telemetry slots in protocol
     * @see AP_Frsky_SPort::sport_packet_t SPort telemetry packet structure
     */
    struct {
        bool available;                           ///< Telemetry packet available flag
        AP_Frsky_SPort::sport_packet_t packet;   ///< SPort telemetry packet data
    } telem_data;
};

#endif  // AP_RCPROTOCOL_FPORT2_ENABLED
