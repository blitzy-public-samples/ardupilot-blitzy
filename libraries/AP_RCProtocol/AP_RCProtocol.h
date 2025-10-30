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
 * @file AP_RCProtocol.h
 * @brief RC (Radio Control) protocol manager coordinating all protocol backends with auto-detection
 * 
 * @details This file defines the AP_RCProtocol class, which serves as the central
 *          RC input subsystem for ArduPilot. It manages multiple protocol backends
 *          (PPMSUM, IBUS, SBUS, DSM, SUMD, SRXL, SRXL2, CRSF, GHST, FPORT, etc.)
 *          and performs automatic protocol detection.
 *          
 *          The system accepts RC input as either:
 *          - Pulse trains (PPM-style timing)
 *          - Byte streams (serial protocols like SBUS, CRSF)
 *          
 *          Protocol Auto-Detection Algorithm:
 *          - Sequentially probes all enabled protocols when no protocol is locked
 *          - For protocols with weak CRCs: requires 3 consecutive valid frames
 *          - For protocols with strong CRCs: locks on first valid frame
 *          - Once detected, protocol remains locked unless signal lost for extended period
 *          
 *          Channel Data Access:
 *          - Provides up to MAX_RCIN_CHANNELS (18) RC channels
 *          - Channel values in PWM microseconds (typically 1000-2000)
 *          - Supports failsafe detection and reporting
 *          - Provides RSSI and link quality metrics when available
 *          
 *          UART Configuration:
 *          - Supports optional additional UART for serial protocols
 *          - Automatically rotates through different baudrate/parity configurations
 *          - Handles inverted serial protocols (e.g., SBUS)
 * 
 * @author Andrew Tridgell and Siddharth Bharat Purohit
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */
#pragma once

#include "AP_RCProtocol_config.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#if AP_RCPROTOCOL_MAVLINK_RADIO_ENABLED
#include <GCS_MAVLink/GCS_MAVLink.h>
#endif

/**
 * @brief Maximum number of RC input channels supported by the protocol system
 * @details Most RC receivers provide 8-16 channels. This limit accommodates
 *          extended channel protocols like CRSF and SRXL2 which can provide
 *          up to 18 channels for advanced control applications.
 */
#define MAX_RCIN_CHANNELS 18

/**
 * @brief Minimum number of RC input channels required for basic vehicle control
 * @details Most vehicles require at least 4 channels (throttle, roll, pitch, yaw)
 *          plus a mode switch, totaling 5 minimum channels for safe operation.
 */
#define MIN_RCIN_CHANNELS  5

class AP_RCProtocol_Backend;

/**
 * @class AP_RCProtocol
 * @brief Central RC input subsystem managing multi-protocol RC receiver support
 * 
 * @details AP_RCProtocol coordinates multiple RC protocol backends and performs
 *          automatic protocol detection from incoming data. The system handles:
 *          
 *          **Protocol Management:**
 *          - Maintains backend instances for all enabled protocols
 *          - Implements sequential protocol probing on unidentified signals
 *          - Locks onto detected protocol until signal loss
 *          - Supports runtime protocol enable/disable via bitmask
 *          
 *          **Input Processing:**
 *          - Pulse train processing (PPM, PWM inputs)
 *          - Serial byte stream processing (SBUS, CRSF, DSM, etc.)
 *          - UART configuration management with auto-baudrate detection
 *          - Handshake processing for negotiation-based protocols
 *          
 *          **Data Management:**
 *          - Channel value storage (PWM microseconds, typically 1000-2000)
 *          - Failsafe state tracking and reporting
 *          - RSSI (signal strength) and link quality metrics
 *          - New input detection for scheduler efficiency
 *          
 *          **Protocol Auto-Detection:**
 *          - Protocols with weak CRCs (SBUS, DSM, PPMSUM, FPORT, CRSF, GHST):
 *            Require 3 consecutive valid frames before locking (prevents false detection)
 *          - Protocols with strong CRCs/checksums (IBUS, SUMD, SRXL, SRXL2, ST24):
 *            Lock on first valid frame (checksums provide confidence)
 *          
 *          **Thread Safety:**
 *          - Called from main thread and potentially UART interrupt contexts
 *          - Backend add_input() callbacks may be called from interrupt handlers
 *          - Channel reads are thread-safe via atomic operations in backends
 *          
 *          **Singleton Access:**
 *          - Global instance accessed via AP::RC() for vehicle code integration
 *          
 * @warning Once a protocol is detected and locked, it remains active even if that
 *          protocol's signal degrades. Protocol will only unlock after extended signal
 *          loss timeout, then re-detection occurs. This prevents spurious protocol
 *          switching during brief signal dropouts which could cause control glitches.
 * 
 * @note Typical main loop usage: Call update() at regular intervals (usually 1kHz)
 *       to process buffered UART data and check for protocol detection changes.
 * 
 * @see AP_RCProtocol_Backend for individual protocol implementations
 * @see AP::RC() for singleton access pattern
 */
class AP_RCProtocol {
public:

    /**
     * @enum rcprotocol_t
     * @brief Enumeration of all supported RC input protocols
     * 
     * @details Defines protocol identifiers for all RC receiver protocols supported
     *          by ArduPilot. Each protocol has unique characteristics:
     *          
     *          **Pulse Protocols:**
     *          - PPMSUM: Pulse Position Modulation (legacy, 8 channels typical)
     *          
     *          **Serial Protocols (UART-based):**
     *          - IBUS: FlySky IBus protocol (14 channels, checksum, 115200 baud)
     *          - SBUS/SBUS_NI: Futaba SBUS (16 channels, weak parity, 100000 baud inverted/non-inverted)
     *          - DSM: Spektrum DSM2/DSMX satellite protocol (up to 12 channels, 115200 baud)
     *          - SUMD: Graupner HoTT SUMD (16 channels, CRC16, 115200 baud)
     *          - SRXL: Multiplex SRXL v1 (12 channels, CRC16, 115200 baud)
     *          - SRXL2: Multiplex SRXL v2 (16 channels, CRC16, 115200/400000 baud)
     *          - CRSF: TBS Crossfire (16 channels, CRC8, 416666 baud, telemetry bidirectional)
     *          - GHST: ImmersionRC Ghost (12 channels, CRC8, 420000 baud, low latency)
     *          - FPORT/FPORT2: FrSky F.Port (16 channels, 115200 baud, telemetry bidirectional)
     *          - FASTSBUS: Fast SBUS variant
     *          - ST24: Graupner ST24 protocol (9 channels)
     *          
     *          **System Integration Protocols:**
     *          - DRONECAN: DroneCAN/UAVCAN RC input from CAN bus
     *          - MAVLINK_RADIO: RC over MAVLink radio_rc_channels message
     *          - IOMCU: RC from IO microcontroller (PX4 architecture)
     *          - RADIO: Integrated 2.4GHz radio (Cypress radio)
     *          - EMLID_RCIO: Emlid Edge RC IO protocol
     *          
     *          **Simulation Protocols (SITL only):**
     *          - JOYSTICK_SFML: Joystick input via SFML library
     *          - UDP: RC input via UDP packets
     *          - FDM: RC from Flight Dynamics Model
     *          
     *          **Protocol Selection:**
     *          - Configured via RC_PROTOCOLS bitmask parameter
     *          - Auto-detection attempts all enabled protocols
     *          - Conditional compilation via AP_RCPROTOCOL_*_ENABLED defines
     * 
     * @note NONE must always be the last enum value for iteration logic
     * @note Not all protocols compiled into all firmware variants due to flash constraints
     */
    enum rcprotocol_t {
#if AP_RCPROTOCOL_PPMSUM_ENABLED
        PPMSUM     =  0,
#endif
#if AP_RCPROTOCOL_IBUS_ENABLED
        IBUS       =  1,
#endif
#if AP_RCPROTOCOL_SBUS_ENABLED
        SBUS       =  2,
#endif
#if AP_RCPROTOCOL_SBUS_NI_ENABLED
        SBUS_NI    =  3,
#endif
#if AP_RCPROTOCOL_DSM_ENABLED
        DSM        =  4,
#endif
#if AP_RCPROTOCOL_SUMD_ENABLED
        SUMD       =  5,
#endif
#if AP_RCPROTOCOL_SRXL_ENABLED
        SRXL       =  6,
#endif
#if AP_RCPROTOCOL_SRXL2_ENABLED
        SRXL2      =  7,
#endif
#if AP_RCPROTOCOL_CRSF_ENABLED
        CRSF       =  8,
#endif
#if AP_RCPROTOCOL_ST24_ENABLED
        ST24       =  9,
#endif
#if AP_RCPROTOCOL_FPORT_ENABLED
        FPORT      = 10,
#endif
#if AP_RCPROTOCOL_FPORT2_ENABLED
        FPORT2     = 11,
#endif
#if AP_RCPROTOCOL_FASTSBUS_ENABLED
        FASTSBUS   = 12,
#endif
#if AP_RCPROTOCOL_DRONECAN_ENABLED
        DRONECAN   = 13,
#endif
#if AP_RCPROTOCOL_GHST_ENABLED
        GHST       = 14,
#endif
#if AP_RCPROTOCOL_MAVLINK_RADIO_ENABLED
        MAVLINK_RADIO = 15,
#endif
#if AP_RCPROTOCOL_JOYSTICK_SFML_ENABLED
        JOYSTICK_SFML = 16,
#endif
#if AP_RCPROTOCOL_UDP_ENABLED
        UDP = 17,
#endif
#if AP_RCPROTOCOL_FDM_ENABLED
        FDM = 18,
#endif
#if AP_RCPROTOCOL_RADIO_ENABLED
        RADIO = 19,
#endif
#if AP_RCPROTOCOL_IOMCU_ENABLED
        IOMCU = 20,
#endif  // AP_RCPROTOCOL_IOMCU_ENABLED
#if AP_RCPROTOCOL_EMLID_RCIO_ENABLED
        EMLID_RCIO = 21,
#endif
        NONE    //last enum always is None
    };

    /**
     * @brief Convert protocol enum to human-readable protocol name string
     * 
     * @param[in] protocol Protocol enum value to convert
     * 
     * @return Pointer to static string containing protocol name (e.g., "SBUS", "CRSF", "DSM")
     *         Returns "None" if protocol is NONE, "Unknown" if protocol value is invalid
     * 
     * @note Returned string pointer is to static memory, safe to use without copying
     * @note Useful for logging and GCS display of detected RC protocol
     */
    static const char *protocol_name_from_protocol(rcprotocol_t protocol);

#if AP_RCPROTOCOL_ENABLED

    /**
     * @brief Default constructor
     * @note Backend instances are created in init()
     */
    AP_RCProtocol() {}
    
    /**
     * @brief Destructor - cleans up all backend instances
     */
    ~AP_RCProtocol();
    
    /**
     * @brief Allow backend classes to access private members for add_input() callbacks
     */
    friend class AP_RCProtocol_Backend;

    /**
     * @brief Initialize the RC protocol system and create enabled backend instances
     * 
     * @details Allocates and initializes backend objects for all protocols enabled
     *          via the rc_protocols_mask configuration. Must be called before any
     *          RC input processing. Typically called once during vehicle startup.
     * 
     * @note Backends are allocated dynamically based on enabled protocols to save memory
     * @see set_rc_protocols() to configure which protocols are enabled
     */
    void init();
    
    /**
     * @brief Check if a valid serial protocol has been detected
     * 
     * @return true if current protocol is a serial byte-stream protocol (not pulse-based)
     * 
     * @note Used to determine if UART configuration is relevant
     */
    bool valid_serial_prot() const
    {
        return _valid_serial_prot;
    }
    
    /**
     * @brief Determine if protocol auto-detection search should be active
     * 
     * @param[in] now_ms Current system time in milliseconds
     * 
     * @return true if system should probe for new protocols (no locked protocol or signal lost)
     * 
     * @note Returns false when a protocol is locked and receiving valid data
     */
    bool should_search(uint32_t now_ms) const;
    
    /**
     * @brief Process a pulse train input with two pulse widths (PPM-style decoding)
     * 
     * @param[in] width_s0 First pulse width in microseconds
     * @param[in] width_s1 Second pulse width in microseconds
     * 
     * @details Used for pulse-based protocols like PPMSUM. Feeds pulse timing data to
     *          pulse-capable backends for decoding. Each backend interprets pulses
     *          according to its protocol specification.
     * 
     * @note Called at interrupt rate from HAL RCInput drivers
     * @note Only processes backends not disabled via disable_for_pulses()
     */
    void process_pulse(uint32_t width_s0, uint32_t width_s1);
    
    /**
     * @brief Process a list of pulse widths (batch pulse processing)
     * 
     * @param[in] widths     Array of pulse widths in microseconds
     * @param[in] n          Number of pulses in array
     * @param[in] need_swap  True if pulse order needs byte swapping
     * 
     * @details Optimized batch processing for hardware that captures multiple pulses.
     *          Primarily used by PPMSUM decoder to process complete frames at once.
     * 
     * @note More efficient than individual process_pulse() calls for bulk data
     */
    void process_pulse_list(const uint32_t *widths, uint16_t n, bool need_swap);
    
    /**
     * @brief Process a single byte of serial RC protocol data
     * 
     * @param[in] byte      Data byte to process
     * @param[in] baudrate  Current UART baudrate in bits per second
     * 
     * @return true if byte was consumed by a backend (used for protocol detection)
     * 
     * @details Feeds serial data to all enabled serial protocol backends. Each backend
     *          maintains its own state machine and calls add_input() callback when a
     *          complete valid frame is decoded.
     *          
     *          Auto-detection: When no protocol is locked, all backends attempt to
     *          decode the byte stream. First backend to produce valid frames wins.
     * 
     * @note Called at high rate from UART interrupt or main loop, minimize processing time
     * @note Baudrate parameter allows backends to validate protocol timing expectations
     */
    bool process_byte(uint8_t byte, uint32_t baudrate);
    
    /**
     * @brief Process protocol handshake for negotiation-based protocols
     * 
     * @param[in] baudrate  UART baudrate in bits per second for handshake
     * 
     * @details Some protocols (e.g., SRXL2) require a handshake sequence before normal
     *          operation. This method allows backends to transmit handshake data on the
     *          UART to establish communication with the receiver.
     * 
     * @note Only applicable to bidirectional protocols with initialization sequences
     */
    void process_handshake(uint32_t baudrate);
    
    /**
     * @brief Main update function - process buffered data and manage protocol detection
     * 
     * @details Called periodically (typically 1kHz from main loop) to:
     *          - Check for newly added UART and configure it
     *          - Detect and announce protocol changes
     *          - Process any asynchronous protocol backends (DRONECAN, MAVLINK, etc.)
     *          - Manage protocol search state
     * 
     * @note Must be called regularly for proper protocol detection and UART management
     * @note Does NOT process incoming bytes (that happens in process_byte/process_pulse)
     */
    void update(void);

    /**
     * @brief Check if RC failsafe is currently active
     * 
     * @return true if RC signal lost or failsafe condition triggered
     * 
     * @details Failsafe is activated when:
     *          - No valid RC frames received for timeout period (typically 1 second)
     *          - Receiver explicitly signals failsafe condition
     *          - Channel values enter failsafe range (protocol-specific)
     * 
     * @note Failsafe state is managed by vehicle code, not automatically by this class
     * @see set_failsafe_active() to set failsafe state
     */
    bool failsafe_active() const {
        return _failsafe_active;
    }
    
    /**
     * @brief Set the RC failsafe state
     * 
     * @param[in] active  true to activate failsafe, false to clear
     * 
     * @details Vehicle code calls this when RC timeout expires or signal quality
     *          degrades below safe thresholds. Does not affect channel values,
     *          only sets flag for query by vehicle control logic.
     * 
     * @note Typically set by RCInput driver when no new_input() for timeout period
     */
    void set_failsafe_active(bool active) {
        _failsafe_active = active;
    }

    /**
     * @brief Disable a specific protocol from processing pulse inputs
     * 
     * @param[in] protocol  Protocol to disable for pulse train processing
     * 
     * @details Prevents specified protocol from receiving process_pulse() or
     *          process_pulse_list() calls. Used when protocol is known to be
     *          serial-only and pulse processing would waste CPU cycles or cause
     *          false detections.
     *          
     *          Typical usage: SBUS, CRSF, and other serial-only protocols are
     *          disabled for pulses since they will never decode pulse trains.
     * 
     * @note Does not affect process_byte() - serial processing still active
     * @note Bitmask is cumulative - multiple protocols can be disabled
     */
    void disable_for_pulses(enum rcprotocol_t protocol) {
        _disabled_for_pulses |= (1U<<(uint8_t)protocol);
    }

#if !defined(__clang__)
// in the case we've disabled most backends then the "return true" in
// the following method can never be reached, and the compiler gets
// annoyed at that.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wswitch-unreachable"
#endif

    /**
     * @brief Determine if a protocol requires 3 consecutive valid frames before locking
     * 
     * @param[in] p  Protocol to check
     * 
     * @return true if protocol needs 3-frame validation, false if single frame sufficient
     * 
     * @details Protocols are categorized by CRC/checksum strength:
     *          
     *          **Weak CRC Protocols (require 3 frames):**
     *          - DSM: Simple checksum, prone to false positives on random data
     *          - FASTSBUS: Parity check only, weak error detection
     *          - SBUS/SBUS_NI: Even parity only, easily satisfied by noise
     *          - PPMSUM: No CRC, relies purely on timing validation
     *          - FPORT/FPORT2: CRC8, relatively weak for noisy environments
     *          - CRSF: CRC8, can occasionally false-trigger on similar protocols
     *          - GHST: CRC8, similar to CRSF
     *          
     *          **Strong CRC Protocols (single frame sufficient):**
     *          - IBUS: Checksum algorithm provides good error detection
     *          - SUMD: CRC16, very low false positive rate
     *          - SRXL/SRXL2: CRC16, robust error detection
     *          - ST24: Strong checksum validation
     *          - DRONECAN: CAN bus CRC, hardware-level error detection
     *          - MAVLINK_RADIO: MAVLink CRC, protocol-level validation
     *          
     *          **Rationale:**
     *          The 3-frame requirement prevents false protocol detection when probing
     *          multiple protocols simultaneously. Random noise or data from other protocols
     *          might occasionally satisfy weak CRC checks, but is very unlikely to produce
     *          3 consecutive valid frames.
     *          
     *          This prevents spurious protocol switching that could cause control glitches
     *          or loss of RC control during brief interference.
     * 
     * @note This is a compile-time classification - protocol behavior does not change at runtime
     * @warning False protocol detection could cause control loss; 3-frame rule is safety-critical
     * 
     * @see AP_RCProtocol_Backend::add_input() for frame validation counting logic
     */
    // for protocols without strong CRCs we require 3 good frames to lock on
    bool requires_3_frames(enum rcprotocol_t p) {
        switch (p) {
#if AP_RCPROTOCOL_DSM_ENABLED
        case DSM:
#endif
#if AP_RCPROTOCOL_FASTSBUS_ENABLED
        case FASTSBUS:
#endif
#if AP_RCPROTOCOL_SBUS_ENABLED
        case SBUS:
#endif
#if AP_RCPROTOCOL_SBUS_NI_ENABLED
        case SBUS_NI:
#endif
#if AP_RCPROTOCOL_PPMSUM_ENABLED
        case PPMSUM:
#endif
#if AP_RCPROTOCOL_FPORT_ENABLED
        case FPORT:
#endif
#if AP_RCPROTOCOL_FPORT2_ENABLED
        case FPORT2:
#endif
#if AP_RCPROTOCOL_CRSF_ENABLED
        case CRSF:
#endif
#if AP_RCPROTOCOL_GHST_ENABLED
        case GHST:
#endif
            return true;
#if AP_RCPROTOCOL_IBUS_ENABLED
        case IBUS:
#endif
#if AP_RCPROTOCOL_SUMD_ENABLED
        case SUMD:
#endif
#if AP_RCPROTOCOL_SRXL_ENABLED
        case SRXL:
#endif
#if AP_RCPROTOCOL_SRXL2_ENABLED
        case SRXL2:
#endif
#if AP_RCPROTOCOL_ST24_ENABLED
        case ST24:
#endif
#if AP_RCPROTOCOL_DRONECAN_ENABLED
        case DRONECAN:
#endif
#if AP_RCPROTOCOL_MAVLINK_RADIO_ENABLED
        case MAVLINK_RADIO:
#endif
#if AP_RCPROTOCOL_JOYSTICK_SFML_ENABLED
        case JOYSTICK_SFML:
#endif
#if AP_RCPROTOCOL_UDP_ENABLED
        case UDP:
#endif
#if AP_RCPROTOCOL_FDM_ENABLED
        case FDM:
#endif
#if AP_RCPROTOCOL_RADIO_ENABLED
        case RADIO:
#endif
#if AP_RCPROTOCOL_IOMCU_ENABLED
        case IOMCU:
#endif  // AP_RCPROTOCOL_IOMCU_ENABLED
#if AP_RCPROTOCOL_EMLID_RCIO_ENABLED
        case EMLID_RCIO:
#endif
        case NONE:
            return false;
        }
        return false;
    }
#if !defined(__clang__)
#pragma GCC diagnostic pop
#endif

    /**
     * @brief Get the number of RC channels provided by the detected protocol
     * 
     * @return Number of channels available (0 if no protocol detected, typically 4-18)
     * 
     * @details Different protocols provide different channel counts:
     *          - PPMSUM: typically 8 channels
     *          - SBUS: 16 channels
     *          - CRSF/GHST: 16 channels
     *          - DSM: 8-12 channels depending on variant
     *          - SRXL2: up to 16 channels
     *          
     *          Maximum is limited by MAX_RCIN_CHANNELS (18).
     * 
     * @note Channel count may change during protocol auto-detection
     * @note Returns 0 if no valid protocol detected yet
     */
    uint8_t num_channels();
    
    /**
     * @brief Read a single RC channel value
     * 
     * @param[in] chan  Channel number (0-based, 0 = channel 1)
     * 
     * @return Channel value in PWM microseconds (typically 1000-2000, 1500 = center)
     *         Returns 0 if channel is invalid or out of range
     * 
     * @details Standard PWM ranges:
     *          - 1000μs = minimum (stick full down/left)
     *          - 1500μs = center (stick centered)
     *          - 2000μs = maximum (stick full up/right)
     *          
     *          Some protocols may provide extended ranges (e.g., 800-2200μs).
     * 
     * @note Channel numbering is 0-based: chan=0 reads channel 1, chan=1 reads channel 2
     * @see num_channels() to determine valid channel range
     */
    uint16_t read(uint8_t chan);
    
    /**
     * @brief Read multiple RC channel values efficiently (batch read)
     * 
     * @param[out] pwm  Array to store channel values (PWM microseconds)
     * @param[in]  n    Number of channels to read
     * 
     * @details Copies up to n channel values into the provided array. More efficient
     *          than multiple individual read() calls when reading many channels.
     *          
     *          If n exceeds num_channels(), only valid channels are copied, remainder
     *          of array may contain undefined values.
     * 
     * @note Caller must ensure pwm array has space for n uint16_t values
     */
    void read(uint16_t *pwm, uint8_t n);
    
    /**
     * @brief Check if new RC input has been received since last check
     * 
     * @return true if new valid frame received since last call
     * 
     * @details Auto-clears flag when called (one-shot notification).
     *          Used by vehicle code to:
     *          - Update control loops only when fresh data available
     *          - Detect signal loss (no new_input for timeout period)
     *          - Optimize CPU usage (skip processing if no new data)
     * 
     * @note Flag is set by backend add_input() callback when valid frame decoded
     * @note Calling this method clears the flag atomically
     */
    bool new_input();
    
    /**
     * @brief Initiate receiver bind mode for pairing with transmitter
     * 
     * @details Sends bind command to receiver (protocol-dependent). Used during
     *          initial receiver setup to pair/bind with a specific transmitter.
     *          
     *          Supported protocols:
     *          - DSM: Enters bind mode, receiver flashes LED
     *          - DRONECAN: Sends bind command over CAN bus
     *          
     *          Not all protocols support bind commands (most require hardware button).
     * 
     * @note Only works if protocol supports bind and is currently active
     * @note Typically invoked via MAVLink command or GCS button
     */
    void start_bind();
    
    /**
     * @brief Get Received Signal Strength Indicator (RSSI) from receiver
     * 
     * @return RSSI value: 0 = no signal, 255 = maximum signal strength
     *         Returns -1 if RSSI not available from current protocol
     * 
     * @details RSSI interpretation varies by protocol:
     *          - CRSF: dBm converted to 0-255 scale
     *          - SBUS: Channel 16 flag-based RSSI
     *          - FrSky: RSSI embedded in protocol
     *          
     *          Higher values indicate stronger signal. Can be used to:
     *          - Warn pilot of weak signal before failsafe
     *          - Log signal strength for post-flight analysis
     *          - Trigger pre-emptive RTL if signal weakening
     * 
     * @note Not all protocols provide RSSI data
     * @note RSSI alone is not a reliable failsafe trigger (use link quality instead)
     */
    int16_t get_RSSI(void) const;
    
    /**
     * @brief Get receiver link quality percentage
     * 
     * @return Link quality: 0 = no link, 100 = perfect link
     *         Returns -1 if link quality not available from current protocol
     * 
     * @details Link quality represents packet success rate (not signal strength):
     *          - 100 = all packets received successfully
     *          - 50 = half of packets lost or corrupted
     *          - 0 = no valid packets
     *          
     *          More reliable than RSSI for failsafe decisions since it directly
     *          measures communication success rather than signal strength.
     *          
     *          Supported protocols:
     *          - CRSF: Provides Link Quality (LQ) metric
     *          - GHST: Similar LQ implementation
     *          - SRXL2: Link quality reporting
     * 
     * @note Preferred over RSSI for failsafe triggering
     * @note Not all protocols support link quality reporting
     */
    int16_t get_rx_link_quality(void) const;

    /**
     * @brief Get human-readable name of the actually detected protocol
     * 
     * @return Pointer to static string with protocol name (e.g., "SBUS", "CRSF")
     *         Returns "None" if no protocol detected
     * 
     * @details In cases where backend provides sub-protocol information, this returns
     *          the actual decoded protocol name rather than the backend name. Example:
     *          - Backend: "IOMCU" (IO microcontroller)
     *          - Detected protocol: "SBUS" (what IOMCU is actually decoding)
     *          
     *          This provides more useful information for pilots and logs about what
     *          RC protocol is actually in use.
     * 
     * @note String pointer is to static memory, safe to use without copying
     * @see protocol_detected() to get protocol enum value
     * @see protocol_name_from_protocol() to convert any enum to name
     */
    // return detected protocol.  In the case that backend can provide
    // information on what *it* is decoding that will be returned by
    // this method.  As opposed to "protocol_name" which will be the
    // backend name e.g. "IOMCU".
    const char *detected_protocol_name() const;

    /**
     * @brief Get the currently detected and locked protocol
     * 
     * @return Protocol enum value (NONE if no protocol detected)
     * 
     * @details Returns the protocol that is currently locked and processing input.
     *          Once locked, this remains constant until signal loss timeout expires
     *          and re-detection occurs.
     * 
     * @note NONE indicates no protocol detected yet or signal lost for extended period
     * @see detected_protocol_name() for human-readable name
     */
    // return detected protocol
    enum rcprotocol_t protocol_detected(void) const {
        return _detected_protocol;
    }

    /**
     * @brief Register an additional UART for RC input processing
     * 
     * @param[in] uart  Pointer to UART driver instance for RC serial protocols
     * 
     * @details Adds a UART that will be probed for serial RC protocols. System
     *          automatically rotates through different configurations:
     *          - Multiple baudrates (9600, 38400, 57600, 100000, 115200, 230400, 420000, etc.)
     *          - Parity settings (none, even, odd)
     *          - Stop bits (1, 2)
     *          - Inverted RX (for SBUS which uses inverted serial)
     *          
     *          Configuration cycling occurs automatically every few seconds until
     *          a protocol locks successfully.
     *          
     *          Typical usage: HAL RCInput driver calls this during initialization
     *          to provide dedicated RC UART.
     * 
     * @note Only one additional UART supported (beyond any built-in RC inputs)
     * @note UART configuration changes automatically during protocol auto-detection
     * @see SerialConfig for configuration details
     * @see has_uart() to check if UART is registered
     */
    // add a UART for RCIN
    void add_uart(AP_HAL::UARTDriver* uart);
    
    /**
     * @brief Check if additional UART has been registered for RC input
     * 
     * @return true if add_uart() has been called with valid UART pointer
     * 
     * @note Used to determine if serial protocol auto-detection should be active
     */
    bool has_uart() const { return added.uart != nullptr; }

    /**
     * @brief Configure which RC protocols are enabled via bitmask
     * 
     * @param[in] mask  Bitmask of enabled protocols (bit position = rcprotocol_t value)
     *                  Bit 0 (LSB) set = enable all protocols
     *                  Specific bits set = enable only those protocols
     * 
     * @details Controls which protocol backends are instantiated and probed:
     *          - Bit 0 = 1: All protocols enabled (default, 0xFFFFFFFF mask)
     *          - Bit N = 1: Protocol with enum value N is enabled
     *          - Bit N = 0: Protocol with enum value N is disabled
     *          
     *          Example masks:
     *          - 0xFFFFFFFF or bit 0 set: All protocols enabled
     *          - (1<<SBUS) | (1<<CRSF): Only SBUS and CRSF enabled
     *          - (1<<DSM): Only DSM enabled
     *          
     *          Disabling unused protocols:
     *          - Reduces RAM usage (fewer backend instances)
     *          - Speeds up auto-detection (fewer protocols to probe)
     *          - Prevents false detections from similar protocols
     *          
     *          Typically configured via RC_PROTOCOLS parameter.
     * 
     * @note Must be called before init() to affect backend instantiation
     * @note Changing mask after init() has no effect (backends already created)
     * @warning Disabling all protocols will prevent RC input completely
     */
    // set allowed RC protocols
    void set_rc_protocols(uint32_t mask) {
        rc_protocols_mask = mask;
    }

    /**
     * @class SerialConfig
     * @brief UART configuration parameters for serial RC protocol auto-detection
     * 
     * @details Encapsulates complete serial port configuration used during protocol
     *          probing. System automatically cycles through multiple SerialConfig
     *          combinations when searching for compatible protocol.
     *          
     *          Configuration rotation strategy:
     *          1. Try common baudrates first (115200, 100000 for SBUS)
     *          2. Try with/without RX inversion (SBUS requires inversion)
     *          3. Try different parity settings (some protocols need even parity)
     *          4. Cycle every 2-3 seconds if no protocol locks
     *          
     *          Common configurations:
     *          - SBUS: 100000 baud, even parity, 2 stop bits, inverted RX
     *          - CRSF: 420000 baud, no parity, 1 stop bit, normal RX
     *          - DSM: 115200 baud, no parity, 1 stop bit, normal RX
     *          - IBUS: 115200 baud, no parity, 1 stop bit, normal RX
     */
    class SerialConfig {
    public:
        /**
         * @brief Apply this configuration to a UART
         * 
         * @param[in] uart  UART to configure
         * 
         * @details Sets baudrate, parity, stop bits, and RX inversion on the UART
         */
        void apply_to_uart(AP_HAL::UARTDriver *uart) const;

        uint32_t baud;       ///< Baudrate in bits per second (e.g., 115200, 420000)
        uint8_t parity;      ///< Parity: 0=none, 1=odd, 2=even
        uint8_t stop_bits;   ///< Stop bits: 1 or 2
        bool invert_rx;      ///< True for inverted UART RX (SBUS requirement)
    };

    /**
     * @brief Check if current protocol uses byte stream instead of pulses
     * 
     * @return true if detected protocol is serial byte-stream based (SBUS, CRSF, etc.)
     *         false if pulse-based (PPMSUM) or no protocol detected
     * 
     * @details Used to determine:
     *          - Whether UART configuration is relevant
     *          - Which type of input processing is active
     *          - Logging and diagnostic information
     * 
     * @note Most modern protocols are byte-stream based
     * @see valid_serial_prot() for similar check
     */
    // return true if we are decoding a byte stream, instead of pulses
    bool using_uart(void) const {
        return _detected_with_bytes;
    }

    /**
     * @brief Handle RC channels received via MAVLink radio_rc_channels message
     * 
     * @param[in] packet  Pointer to MAVLink radio_rc_channels message structure
     * 
     * @details Processes RC input transmitted over MAVLink (typically from companion
     *          computer or radio modem with integrated RC capability). Extracts
     *          channel values and updates internal state as if decoded from receiver.
     *          
     *          Use cases:
     *          - RC over telemetry radio (backup to direct RC link)
     *          - Companion computer control (ROS-based control)
     *          - Long-range control via LTE/cellular
     * 
     * @note Only available if AP_RCPROTOCOL_MAVLINK_RADIO_ENABLED
     * @warning MAVLink RC has higher latency than direct receiver - use for backup only
     */
    // handle mavlink radio
#if AP_RCPROTOCOL_MAVLINK_RADIO_ENABLED
    void handle_radio_rc_channels(const mavlink_radio_rc_channels_t* packet);
#endif

private:
    /**
     * @brief Check and configure added UART for protocol detection
     * @details Rotates through SerialConfig combinations, called from update()
     */
    void check_added_uart(void);

    /**
     * @brief Check if a specific protocol is enabled in the protocol mask
     * @param[in] protocol  Protocol to check
     * @return true if protocol is enabled via rc_protocols_mask
     */
    // return true if a specific protocol is enabled
    bool protocol_enabled(enum rcprotocol_t protocol) const;

    /**
     * @brief Probe asynchronous protocols that don't use byte/pulse stream
     * @param[in] protocol  Protocol to probe (e.g., DRONECAN, MAVLINK_RADIO)
     * @return true if protocol has new data available
     * @details For protocols that pull data rather than being fed data
     */
    // explicitly investigate a backend for data, as opposed to
    // feeding the backend a byte (or pulse-train) at a time and
    // having them make an "add_input" callback):
    bool detect_async_protocol(rcprotocol_t protocol);

    enum rcprotocol_t _detected_protocol = NONE;  ///< Currently locked protocol
    uint16_t _disabled_for_pulses;                ///< Bitmask of protocols disabled for pulse processing
    bool _detected_with_bytes;                    ///< True if current protocol is byte-stream based
    AP_RCProtocol_Backend *backend[NONE];         ///< Array of backend instances (one per protocol)
    bool _new_input;                              ///< Flag: new data available since last check
    uint32_t _last_input_ms;                      ///< Timestamp of last valid input (milliseconds)
    bool _failsafe_active;                        ///< Current failsafe state
    bool _valid_serial_prot;                      ///< True if serial protocol is active

    /**
     * @brief Optional additional UART configuration and state
     */
    // optional additional uart
    struct {
        AP_HAL::UARTDriver *uart;      ///< UART driver pointer
        bool opened;                    ///< True if UART has been opened
        uint32_t last_config_change_ms; ///< Timestamp of last configuration change
        uint8_t config_num;             ///< Current configuration index
    } added;

    uint32_t rc_protocols_mask;  ///< Bitmask of enabled protocols (bit 0 = all)

    rcprotocol_t _last_detected_protocol;  ///< Previous detected protocol for change detection
    bool _last_detected_using_uart;        ///< Previous byte vs pulse mode for change detection
    
    /**
     * @brief Announce protocol detection via logging/GCS
     * @details Called when protocol changes or initial detection
     */
    void announce_detected();

#endif  // AP_RCPROTCOL_ENABLED

};

#if AP_RCPROTOCOL_ENABLED
/**
 * @namespace AP
 * @brief ArduPilot singleton accessor namespace
 */
namespace AP {
    /**
     * @brief Get reference to the global RC protocol instance
     * 
     * @return Reference to singleton AP_RCProtocol instance
     * 
     * @details Provides global access to the RC protocol system for vehicle code.
     *          This follows ArduPilot's singleton pattern for core subsystems.
     *          
     *          Typical usage:
     *          @code
     *          uint16_t throttle = AP::RC().read(2);  // Read channel 3 (0-indexed)
     *          if (AP::RC().failsafe_active()) {
     *              // Handle RC failsafe
     *          }
     *          @endcode
     * 
     * @note Instance is instantiated in AP_RCProtocol.cpp
     * @note Access via this function, not direct global variable access
     */
    AP_RCProtocol &RC();
};

#include "AP_RCProtocol_Backend.h"
#endif  // AP_RCProtocol_enabled
