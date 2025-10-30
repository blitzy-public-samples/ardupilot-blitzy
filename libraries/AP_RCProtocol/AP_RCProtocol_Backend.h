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
 * @file AP_RCProtocol_Backend.h
 * @brief RC protocol backend base class architecture
 * 
 * @details This file defines the abstract base class for all RC protocol decoder implementations
 *          in ArduPilot. The backend architecture allows ArduPilot to support multiple RC receiver
 *          protocols (SBUS, PPM, DSM, CRSF, SRXL, etc.) through a common interface.
 *          
 *          Each protocol implementation inherits from AP_RCProtocol_Backend and implements
 *          protocol-specific decoding logic while the frontend (AP_RCProtocol) handles protocol
 *          detection, backend management, and output channel management.
 *          
 *          Backend Lifecycle:
 *          1. Construction: Backend created and registered with frontend
 *          2. Detection: Frontend feeds data via process_byte() or process_pulse()
 *          3. Decoding: Backend parses protocol frames and calls add_input() with channel data
 *          4. Output: Frontend provides decoded channels to vehicle code
 *          
 *          Thread Safety: Some backends may be called from ISR context, particularly add_input()
 *          which must be ISR-safe for pulse-based protocols.
 * 
 * @author Andrew Tridgell and Siddharth Bharat Purohit
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

#include "AP_RCProtocol.h"

#if AP_RCPROTOCOL_ENABLED

#include <AP_HAL/utility/sparse-endian.h>
#include <AP_VideoTX/AP_VideoTX_config.h>

/**
 * @class AP_RCProtocol_Backend
 * @brief Abstract base class for RC protocol decoder implementations
 * 
 * @details This class defines the common interface that all RC protocol backends must implement.
 *          Backends are responsible for decoding protocol-specific frame formats and converting
 *          them to normalized PWM channel values (typically 1000-2000μs).
 *          
 *          The backend architecture supports:
 *          - Byte-based protocols (SBUS, CRSF, DSM, etc.) via process_byte()
 *          - Pulse-based protocols (PPM, PWM) via process_pulse()
 *          - Handshake protocols (some serial protocols) via process_handshake()
 *          - Protocol-initiated binding via start_bind()
 *          - Regular polling for stateful protocols via update()
 *          
 *          Decoded channel data is passed to the frontend via add_input() which handles:
 *          - Frame counting (total frames and non-failsafe frames)
 *          - Failsafe state management
 *          - RSSI and link quality tracking
 *          - Protocol selection and switching
 *          
 * @note All virtual methods have default empty implementations, allowing backends to implement
 *       only the methods relevant to their protocol type.
 * 
 * @warning add_input() may be called from ISR context in some backend implementations,
 *          particularly pulse-based protocols. Ensure thread-safe operation.
 */
class AP_RCProtocol_Backend {
    friend class AP_RCProtcol;

public:
    /**
     * @brief Construct RC protocol backend
     * 
     * @param[in] _frontend Reference to frontend manager that owns this backend
     * 
     * @details Backends are constructed by AP_RCProtocol during initialization.
     *          The frontend reference provides access to configuration, protocol detection
     *          state, and inter-backend coordination.
     */
    AP_RCProtocol_Backend(AP_RCProtocol &_frontend);
    
    /**
     * @brief Virtual destructor for proper cleanup of derived classes
     */
    virtual ~AP_RCProtocol_Backend() {}
    
    /**
     * @brief Process a pulse width input for pulse-based protocols (PPM, PWM)
     * 
     * @param[in] width_s0 Width of first pulse in microseconds (μs)
     * @param[in] width_s1 Width of second pulse in microseconds (μs)
     * 
     * @details Called by the frontend for pulse-based RC protocols. Backends implementing
     *          pulse-based protocols (PPM, PWM) override this method to decode pulse timing
     *          into channel values. Pulse widths are typically in the range 1000-2000μs for
     *          valid channel data, with sync pulses having distinct timing characteristics.
     *          
     *          Default implementation is empty - only pulse-based backends override this.
     * 
     * @note May be called from interrupt context - keep implementation ISR-safe
     */
    virtual void process_pulse(uint32_t width_s0, uint32_t width_s1) {}
    
    /**
     * @brief Process a single byte for byte-based serial protocols (SBUS, CRSF, DSM, etc.)
     * 
     * @param[in] byte Single byte received from UART
     * @param[in] baudrate Current UART baudrate in bits per second
     * 
     * @details Called by the frontend for each byte received on the RC input UART.
     *          Backends implementing serial protocols override this method to accumulate
     *          bytes into frames, validate checksums, and decode channel data.
     *          
     *          The baudrate parameter allows backends to detect baudrate changes and adjust
     *          protocol detection or frame synchronization accordingly.
     *          
     *          Default implementation is empty - only byte-based backends override this.
     * 
     * @note Called at UART byte reception rate (typically 100-400kbaud = ~10-40kB/s)
     */
    virtual void process_byte(uint8_t byte, uint32_t baudrate) {}
    
    /**
     * @brief Process handshake for protocols requiring initialization handshake
     * 
     * @param[in] baudrate Current UART baudrate in bits per second
     * 
     * @details Some RC protocols require a handshake or initialization sequence before
     *          normal operation. Backends implementing such protocols override this method
     *          to send initialization commands or responses.
     *          
     *          Default implementation is empty - only protocols with handshake requirements
     *          override this method.
     */
    virtual void process_handshake(uint32_t baudrate) {}
    
    /**
     * @brief Read a single channel value
     * 
     * @param[in] chan Channel number (0-based, 0 = channel 1)
     * 
     * @return Channel PWM value in microseconds (typically 1000-2000μs), or 0 if invalid
     * 
     * @details Returns the most recently decoded value for the specified channel.
     *          Channel values are normalized PWM microseconds regardless of the underlying
     *          protocol's native format.
     *          
     *          Standard PWM range: 1000μs = minimum, 1500μs = center, 2000μs = maximum
     */
    uint16_t read(uint8_t chan);
    
    /**
     * @brief Read multiple channel values into an array
     * 
     * @param[out] pwm Array to receive channel values (PWM microseconds 1000-2000μs)
     * @param[in]  n   Number of channels to read
     * 
     * @details Efficiently reads multiple channels into the provided array. Each element
     *          receives the PWM value in microseconds for the corresponding channel.
     *          Array must be allocated by caller with at least n elements.
     *          
     *          Channels beyond num_channels() will return 0.
     */
    void read(uint16_t *pwm, uint8_t n);
    
    /**
     * @brief Check if new input has been received since last check
     * 
     * @return true if new frame decoded since last call, false otherwise
     * 
     * @details Returns true once per decoded frame, then clears the flag. Used by vehicle
     *          code to detect when fresh RC input is available for processing.
     *          
     *          This flag is set by add_input() when a complete frame is successfully decoded.
     */
    bool new_input();
    
    /**
     * @brief Get number of channels in most recent frame
     * 
     * @return Number of channels (typically 4-16, depending on protocol)
     * 
     * @details Returns the channel count from the most recently decoded frame.
     *          Different protocols support different channel counts:
     *          - PPM: typically 8 channels
     *          - SBUS: 16 channels
     *          - CRSF: up to 16 channels
     *          - DSM: 6-12 channels depending on variant
     */
    uint8_t num_channels() const;

    /**
     * @brief Initiate binding procedure for receivers with flight controller initiated bind
     * 
     * @details Some RC receivers support binding initiation from the flight controller
     *          rather than requiring a bind button on the receiver. Backends supporting
     *          this feature override this method to send the appropriate bind command.
     *          
     *          Typically invoked via MAVLink command or parameter.
     *          
     *          Default implementation is empty - only backends with FC-initiated bind
     *          support override this method.
     * 
     * @note Binding procedures are protocol-specific and may require specific timing
     */
    virtual void start_bind() {}

    /**
     * @brief Regular update for backends requiring periodic processing
     * 
     * @details Called regularly (typically at scheduler rate, ~400Hz) to allow backends
     *          to perform periodic tasks such as:
     *          - Timeout detection and failsafe triggering
     *          - Telemetry transmission
     *          - Protocol state machine updates
     *          - Heartbeat or keepalive transmission
     *          
     *          Default implementation is empty - only stateful backends override this.
     * 
     * @note Called from main scheduler thread, not from ISR context
     */
    virtual void update(void) {}

    /**
     * @brief Update RC channels from MAVLink radio_rc_channels message
     * 
     * @param[in] packet Pointer to MAVLink radio_rc_channels message
     * 
     * @details Allows RC input to be provided via MAVLink messages, enabling use cases such as:
     *          - RC-over-telemetry for long-range applications
     *          - Companion computer providing virtual RC input
     *          - Ground station control without physical transmitter
     *          
     *          Default implementation is empty - only MAVLink RC backend overrides this.
     * 
     * @note Only available when AP_RCPROTOCOL_MAVLINK_RADIO_ENABLED
     */
#if AP_RCPROTOCOL_MAVLINK_RADIO_ENABLED
    virtual void update_radio_rc_channels(const mavlink_radio_rc_channels_t* packet) {}
#endif

    /**
     * @brief Get total number of frames received (including failsafe frames)
     * 
     * @return Total frame count since last reset
     * 
     * @details Returns the total number of protocol frames decoded, regardless of
     *          failsafe status. This count includes both valid frames and frames marked
     *          as failsafe by the protocol.
     *          
     *          Used for protocol detection confidence and receiver diagnostics.
     * 
     * @see get_rc_input_count() for count excluding failsafe frames
     * @see reset_rc_frame_count() to reset counter
     */
    uint32_t get_rc_frame_count(void) const {
        return rc_frame_count;
    }

    /**
     * @brief Reset the frame counter to zero
     * 
     * @details Resets rc_frame_count to 0, typically called when switching protocols
     *          or reinitializing the RC system.
     */
    void reset_rc_frame_count(void) {
        rc_frame_count = 0;
    }

    /**
     * @brief Get number of valid input frames (excluding failsafe)
     * 
     * @return Count of non-failsafe frames since last reset
     * 
     * @details Returns the count of frames that contained valid RC data (not in failsafe).
     *          This count excludes frames where the protocol indicated failsafe condition.
     *          
     *          Used by vehicle code to determine RC input reliability and trigger failsafe
     *          actions when count stops incrementing.
     * 
     * @see get_rc_frame_count() for total count including failsafe frames
     */
    uint32_t get_rc_input_count(void) const {
        return rc_input_count;
    }

    /**
     * @brief Get bitmask of enabled protocols
     * 
     * @return Bitmask with bit set for each enabled protocol
     * 
     * @details Returns the frontend's protocol enable mask. Each bit corresponds to a
     *          protocol enum value. Used by backends to check if their protocol is enabled.
     */
    uint32_t get_rc_protocols_mask(void) const {
        return frontend.rc_protocols_mask;
    }

    /**
     * @brief Check if a specific protocol is enabled
     * 
     * @param[in] protocol Protocol enum to check
     * 
     * @return true if protocol is enabled, false otherwise
     * 
     * @details Queries the frontend to determine if the specified protocol is enabled
     *          in the configuration. Protocols can be selectively enabled/disabled via
     *          parameters to reduce CPU usage and improve detection time.
     */
    bool protocol_enabled(enum AP_RCProtocol::rcprotocol_t protocol) const {
        return frontend.protocol_enabled(protocol);
    }

    /**
     * @brief Get receiver signal strength indication
     * 
     * @return RSSI value: typically dBm (negative, e.g., -70) or percentage (0-100), -1 if unavailable
     * 
     * @details Returns the Received Signal Strength Indication if provided by the protocol.
     *          RSSI format is protocol-dependent:
     *          - Some protocols report in dBm (typically -120 to -30 dBm)
     *          - Others report as percentage (0-100%)
     *          - Returns -1 if protocol does not provide RSSI
     *          
     *          Used for link quality monitoring and failsafe decisions.
     * 
     * @note RSSI scale and meaning varies by protocol - consult protocol-specific documentation
     */
    int16_t get_RSSI(void) const {
        return rssi;
    }
    
    /**
     * @brief Get receiver link quality metric
     * 
     * @return Link quality: typically 0-100%, -1 if unavailable
     * 
     * @details Returns a link quality metric if provided by the protocol. Link quality (LQ)
     *          typically represents packet success rate or signal-to-noise ratio:
     *          - 100% = perfect link, all packets received
     *          - 0% = no valid packets
     *          - -1 = protocol does not provide link quality
     *          
     *          More reliable than RSSI for assessing link health as it reflects actual
     *          packet reception rate rather than just signal strength.
     * 
     * @see get_RSSI() for signal strength
     */
    int16_t get_rx_link_quality(void) const {
        return rx_link_quality;
    }
    /**
     * @brief Get UART for active RC input protocol
     * 
     * @return Pointer to UART driver if this backend is providing active RC input via UART,
     *         nullptr otherwise
     * 
     * @details Returns the UART driver only if:
     *          1. A UART is available for RC input
     *          2. This backend's protocol was detected via byte stream (not pulses)
     *          3. This backend is currently providing the active RC input
     *          
     *          Used by backends that need bidirectional UART communication (e.g., for telemetry)
     *          to ensure they only use the UART when they're the active protocol.
     * 
     * @see get_available_UART() to get UART regardless of detection status
     */
    AP_HAL::UARTDriver *get_UART(void) const {
        return frontend._detected_with_bytes?frontend.added.uart:nullptr;
    }

    /**
     * @brief Get available UART regardless of protocol detection
     * 
     * @return Pointer to UART driver if available, nullptr otherwise
     * 
     * @details Returns the UART driver assigned to RC input, regardless of whether
     *          a protocol has been detected or if this backend is active.
     *          
     *          Use with caution - multiple backends may attempt to use the UART simultaneously
     *          during protocol detection.
     * 
     * @see get_UART() for UART access restricted to active backend
     */
    AP_HAL::UARTDriver *get_available_UART(void) const {
        return frontend.added.uart;
    }

    /**
     * @brief Check if UART is available for protocol handling
     * 
     * @return true if UART available, false otherwise
     * 
     * @details Returns true if a UART has been assigned to RC input, regardless of
     *          protocol detection state. Used to determine if byte-based protocol
     *          detection is possible.
     */
    bool have_UART(void) const {
        return frontend.added.uart != nullptr;
    }

    /**
     * @brief Check if receiver is actively receiving data
     * 
     * @return true if receiver is active, false if idle or powered off
     * 
     * @details Used to detect receiver power loss, cable disconnection, or baudrate changes.
     *          Backends override this to implement protocol-specific activity detection:
     *          - Serial protocols: Check if bytes are being received
     *          - Pulse protocols: Check if pulses are being received
     *          
     *          Default implementation returns true (always active).
     * 
     * @note Called regularly to monitor receiver health
     */
    virtual bool is_rx_active() const {
        return true;
    }

    /**
     * @brief Check if this backend's protocol is currently detected and active
     * 
     * @return true if this backend is the detected protocol, false otherwise
     * 
     * @details Returns true when:
     *          1. Frontend has detected a specific protocol (not NONE)
     *          2. This backend is the one providing that detected protocol
     *          
     *          Used by backends to determine if they should be actively processing input
     *          and performing protocol-specific operations.
     */
    bool is_detected() const {
        return frontend._detected_protocol != AP_RCProtocol::NONE && frontend.backend[frontend._detected_protocol] == this;
    }

#if AP_VIDEOTX_ENABLED
    /**
     * @brief Configure video transmitter settings
     * 
     * @param[in] band     VTX frequency band (typically 1-5 for different frequency ranges)
     * @param[in] channel  Channel within band (typically 1-8)
     * @param[in] power    Transmit power level (protocol-specific encoding)
     * @param[in] pitmode  Pit mode enable (0=off, 1=on - low power mode for safety)
     * 
     * @details Static method called by backends to configure video transmitter settings
     *          when VTX configuration data is received via RC protocol (e.g., CRSF VTX frames).
     *          
     *          Settings are forwarded to the AP_VideoTX library for application to the VTX hardware.
     *          
     *          Pit mode is a low-power mode used during setup to prevent interference with
     *          other pilots' video receivers.
     * 
     * @note Only available when AP_VIDEOTX_ENABLED
     * @warning VTX configuration must comply with local regulations for frequency and power
     */
    static void configure_vtx(uint8_t band, uint8_t channel, uint8_t power, uint8_t pitmode);
#endif

protected:

    /**
     * @struct Channels11Bit_8Chan
     * @brief Packed structure for efficient decoding of 11-bit channel data
     * 
     * @details Many RC protocols (SBUS, CRSF, SRXL2) pack channel data into 11 bits per channel
     *          rather than using full 16-bit values. This structure provides efficient access
     *          to 8 channels packed into 11 bytes (88 bits = 8 × 11 bits).
     *          
     *          Memory layout (little-endian):
     *          - ch0: bits 0-10
     *          - ch1: bits 11-21
     *          - ch2: bits 22-32
     *          - ... (each channel occupies exactly 11 bits)
     *          
     *          11-bit values (0-2047) are typically scaled to PWM microseconds:
     *          - 0 → 1000μs (minimum)
     *          - 1024 → 1500μs (center)
     *          - 2047 → 2000μs (maximum)
     * 
     * @note Only supported on little-endian architectures (enforced at compile time)
     * @note PACKED attribute ensures no padding bytes between bitfields
     * 
     * @see decode_11bit_channels() for protocol-agnostic 11-bit decoding
     */
    struct Channels11Bit_8Chan {
#if __BYTE_ORDER != __LITTLE_ENDIAN
#error "Only supported on little-endian architectures"
#endif
        uint32_t ch0 : 11;
        uint32_t ch1 : 11;
        uint32_t ch2 : 11;
        uint32_t ch3 : 11;
        uint32_t ch4 : 11;
        uint32_t ch5 : 11;
        uint32_t ch6 : 11;
        uint32_t ch7 : 11;
    } PACKED;

    /**
     * @brief Add decoded input channels to frontend
     * 
     * @param[in] num_channels   Number of channels in values array
     * @param[in] values         Array of PWM values in microseconds (typically 1000-2000μs)
     * @param[in] in_failsafe    true if protocol indicates failsafe condition, false for valid data
     * @param[in] rssi           Optional RSSI value (dBm or percentage, -1 if not available)
     * @param[in] rx_link_quality Optional link quality 0-100%, -1 if not available
     * 
     * @details Called by backends when a complete frame has been decoded. This method:
     *          1. Increments rc_frame_count (total frames including failsafe)
     *          2. Increments rc_input_count (only if !in_failsafe)
     *          3. Calls frontend.set_failsafe_active() to propagate failsafe state
     *          4. Stores channel values for read() methods
     *          5. Updates RSSI and link quality if provided
     *          6. Sets new_input flag for consumption by vehicle code
     *          
     *          The frontend uses frame counts and failsafe state for protocol detection,
     *          protocol switching, and vehicle failsafe triggering.
     * 
     * @warning May be called from ISR context in pulse-based protocols - must be ISR-safe.
     *          Avoid blocking operations, memory allocation, or excessive processing time.
     * 
     * @note Channel values should be normalized PWM microseconds regardless of protocol's
     *       native format (11-bit, percentage, etc.)
     */
    void add_input(uint8_t num_channels, uint16_t *values, bool in_failsafe, int16_t rssi=-1, int16_t rx_link_quality=-1);
    
    /**
     * @brief Reference to frontend manager
     * 
     * @details Provides access to frontend configuration, protocol detection state,
     *          and coordination between backends. Used throughout backend implementation
     *          to query configuration and report status.
     */
    AP_RCProtocol &frontend;

    /**
     * @brief Log raw protocol frame data for debugging
     * 
     * @param[in] prot      Protocol type identifier
     * @param[in] timestamp Timestamp in microseconds
     * @param[in] data      Pointer to raw frame data
     * @param[in] len       Length of frame data in bytes
     * 
     * @details Streams raw protocol frames to the logging system when HAL_LOGGING_ENABLED.
     *          Used for protocol debugging, reverse engineering, and issue diagnosis.
     *          
     *          Logged data includes:
     *          - Protocol type (for filtering in log analysis)
     *          - Timestamp (for timing analysis)
     *          - Raw frame bytes (for protocol analysis)
     *          
     *          Backends call this method after receiving complete frames to capture
     *          protocol-specific data that may not be visible in decoded channel values.
     * 
     * @note Only logs when HAL_LOGGING_ENABLED to avoid performance impact
     * @note Log data can be large - use selectively to avoid overwhelming log storage
     */
    void log_data(AP_RCProtocol::rcprotocol_t prot, uint32_t timestamp, const uint8_t *data, uint8_t len) const;

    /**
     * @brief Decode channels from standard 11-bit packed format
     * 
     * @param[in]  data      Pointer to packed 11-bit channel data
     * @param[in]  nchannels Number of channels to decode
     * @param[out] values    Output array for PWM values (must have space for nchannels elements)
     * @param[in]  mult      Multiplier for scaling (numerator)
     * @param[in]  div       Divisor for scaling (denominator)
     * @param[in]  offset    Offset to add after scaling (in microseconds)
     * 
     * @details Static helper method to decode 11-bit packed channel data used by multiple
     *          protocols (SBUS, CRSF, SRXL2). Converts packed 11-bit values (0-2047) to
     *          PWM microseconds using the formula:
     *          
     *          pwm_us = (11bit_value * mult / div) + offset
     *          
     *          Common usage:
     *          - SBUS: mult=1000, div=1639, offset=880 → maps 0-2047 to 880-2128μs
     *          - CRSF: mult=1000, div=1639, offset=880 → maps 0-2047 to 880-2128μs
     *          
     *          The 11-bit format provides 2048 discrete values (0-2047), offering higher
     *          resolution than traditional 10-bit (1024 values) while remaining compact.
     * 
     * @note Static method can be called without backend instance
     * @note Scaling parameters allow protocol-specific PWM range mapping
     */
    static void decode_11bit_channels(const uint8_t* data, uint8_t nchannels, uint16_t *values, uint16_t mult, uint16_t div, uint16_t offset);

private:
    /**
     * @brief Count of non-failsafe frames received
     * 
     * @details Incremented by add_input() only when in_failsafe=false. Used by vehicle code
     *          to detect when valid RC input stops arriving (failsafe condition).
     */
    uint32_t rc_input_count;
    
    /**
     * @brief Previous value of rc_input_count for change detection
     * 
     * @details Cached value used internally to detect when rc_input_count has changed,
     *          indicating new valid input has arrived.
     */
    uint32_t last_rc_input_count;
    
    /**
     * @brief Total count of all frames received (including failsafe)
     * 
     * @details Incremented by add_input() for every frame, regardless of failsafe state.
     *          Used for protocol detection confidence and receiver diagnostics.
     */
    uint32_t rc_frame_count;

    /**
     * @brief Storage for decoded channel PWM values
     * 
     * @details Array holds the most recently decoded PWM values in microseconds (1000-2000μs)
     *          for up to MAX_RCIN_CHANNELS channels. Updated by add_input() and read by
     *          read() methods.
     */
    uint16_t _pwm_values[MAX_RCIN_CHANNELS];
    
    /**
     * @brief Number of channels in most recent frame
     * 
     * @details Updated by add_input() to reflect channel count in most recent decoded frame.
     *          Returned by num_channels() method.
     */
    uint8_t  _num_channels;
    
    /**
     * @brief Received Signal Strength Indication
     * 
     * @details RSSI value from protocol if available, -1 if not provided.
     *          Format is protocol-dependent (dBm or percentage).
     *          Updated by add_input() when provided.
     */
    int16_t rssi = -1;
    
    /**
     * @brief Receiver link quality metric
     * 
     * @details Link quality percentage (0-100%) from protocol if available, -1 if not provided.
     *          Typically represents packet success rate. Updated by add_input() when provided.
     */
    int16_t rx_link_quality = -1;
};

#endif  // AP_RCPROTOCOL_ENABLED
