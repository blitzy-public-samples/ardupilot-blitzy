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
 */

/**
 * @file AP_RCProtocol_SRXL2.h
 * @brief Spektrum SRXL2 RC protocol decoder with bidirectional communication support
 * 
 * @details This file implements the Spektrum SRXL2 (Spektrum Remote Receiver Link v2) protocol,
 *          an enhanced bidirectional communication protocol for Spektrum RC receivers.
 *          
 *          SRXL2 Protocol Features:
 *          - Bidirectional handshake with baud rate negotiation (typically 115200 or 400000 bps)
 *          - Multi-channel support (up to 32 channels with 16-bit resolution)
 *          - VTX (video transmitter) control: band, channel, and power configuration
 *          - Telemetry downlink: Flight data sent back to transmitter
 *          - Device discovery and capability exchange
 *          - Bind procedure integration
 *          
 *          The protocol operates through a state machine that processes incoming bytes,
 *          performs handshake negotiation, and maintains bidirectional communication with
 *          Spektrum SRXL2-compatible receivers.
 *          
 *          Integration with spm_srxl Library:
 *          This implementation integrates with the external spm_srxl engine library which
 *          provides the following callback interfaces:
 *          - srxlOnReceive(): Called when valid SRXL2 frame is decoded
 *          - srxlOnBind(): Handles bind procedure requests
 *          - srxlOnVtx(): Processes VTX configuration commands
 *          - srxlFillTelemetry(): Constructs telemetry packets for downlink
 * 
 * @note Requires SRXL2-compatible Spektrum receiver hardware
 * @warning Handshake must complete successfully via process_handshake() before normal
 *          operation begins. This negotiates baud rate and exchanges device capabilities.
 */

#pragma once

#include "AP_RCProtocol_config.h"

#if AP_RCPROTOCOL_SRXL2_ENABLED

#include "AP_RCProtocol.h"
#include "SoftSerial.h"

/// Maximum number of RC channels supported by SRXL2 datastream
#define SRXL2_MAX_CHANNELS 32U
/// Maximum possible SRXL2 frame length in bytes (includes header, payload, and CRC)
#define SRXL2_FRAMELEN_MAX   80U
/// SRXL2 frame header length in bytes (contains sync byte, length, and packet type)
#define SRXL2_HEADER_LEN 3U

/**
 * @class AP_RCProtocol_SRXL2
 * @brief Spektrum SRXL2 protocol backend decoder with bidirectional communication
 * 
 * @details This backend implements the Spektrum SRXL2 protocol, providing enhanced features
 *          beyond standard RC input including bidirectional communication, VTX control,
 *          and telemetry downlink capabilities.
 *          
 *          The implementation integrates with the spm_srxl library engine which handles
 *          low-level protocol encoding/decoding and provides callback interfaces for:
 *          - Receiving RC channel data (srxlOnReceive callback)
 *          - Handling bind requests (srxlOnBind callback)
 *          - Processing VTX commands (srxlOnVtx callback)
 *          - Constructing telemetry packets (srxlFillTelemetry callback)
 *          
 *          Protocol Operation:
 *          1. Bootstrap Phase: Device discovery and baud rate negotiation via handshake
 *          2. Normal Operation: Continuous frame reception and telemetry transmission
 *          3. State Machine: Processes incoming bytes (IDLE → NEW → COLLECT states)
 *          
 *          Handshake Procedure:
 *          The SRXL2 protocol requires a bidirectional handshake on startup to negotiate
 *          communication parameters (baud rate typically 115200 or 400000 bps) and exchange
 *          device capabilities. This is handled by process_handshake().
 *          
 *          VTX Control:
 *          When connected to compatible video transmitters, SRXL2 can configure:
 *          - VTX band selection
 *          - Channel/frequency selection
 *          - Transmit power levels
 *          
 *          Telemetry:
 *          Flight data is packaged and transmitted back to the transmitter for display,
 *          including battery voltage, altitude, GPS data, and other sensor information.
 * 
 * @note This protocol is only available with Spektrum SRXL2-compatible receivers
 * @warning Requires successful handshake completion before normal RC input will function
 * @see AP_RCProtocol_Backend for base protocol interface
 */
class AP_RCProtocol_SRXL2 : public AP_RCProtocol_Backend {
public:
    /**
     * @brief Constructor for SRXL2 protocol backend
     * @param[in] _frontend Reference to parent AP_RCProtocol frontend
     */
    AP_RCProtocol_SRXL2(AP_RCProtocol &_frontend);
    
    /**
     * @brief Destructor for SRXL2 protocol backend
     */
    virtual ~AP_RCProtocol_SRXL2();
    
    /**
     * @brief Process a single byte from the SRXL2 datastream
     * 
     * @details This method is called for each incoming byte from the RC receiver UART.
     *          It feeds bytes into the SRXL2 state machine which decodes frames containing
     *          RC channel data, VTX commands, and other protocol messages.
     *          
     *          The state machine progresses through:
     *          - STATE_IDLE: Waiting for frame start
     *          - STATE_NEW: Processing frame header (sync, length, type)
     *          - STATE_COLLECT: Collecting frame payload and validating CRC
     *          
     *          Once a complete valid frame is received, the spm_srxl engine invokes
     *          the appropriate callback (srxlOnReceive, srxlOnVtx, etc.) to process
     *          the decoded data.
     * 
     * @param[in] byte Single byte received from RC receiver UART
     * @param[in] baudrate Current UART baud rate in bits per second (typically 115200 or 400000 bps)
     * 
     * @note Called at UART reception rate, typically hundreds of times per second
     * @see process_handshake() for initial baud rate negotiation
     */
    void process_byte(uint8_t byte, uint32_t baudrate) override;
    
    /**
     * @brief Perform SRXL2 bidirectional handshake for baud rate negotiation
     * 
     * @details The SRXL2 protocol requires a bidirectional handshake sequence on startup
     *          to negotiate communication parameters and exchange device capabilities.
     *          
     *          Handshake Sequence:
     *          1. Flight controller sends handshake request with supported baud rates
     *          2. Receiver responds with selected baud rate and capabilities
     *          3. Both devices switch to negotiated baud rate (typically 115200 or 400000 bps)
     *          4. Device IDs and feature flags are exchanged
     *          5. Bootstrap complete, normal operation begins
     *          
     *          The handshake must complete successfully before RC channel data will be
     *          processed. Handshake is retried periodically until successful.
     *          
     *          Timing Constraints:
     *          - Handshake timeout: typically 100-500 milliseconds
     *          - Retry interval: 1000 milliseconds if handshake fails
     *          - Must complete within a few seconds of receiver power-on
     * 
     * @param[in] baudrate Initial UART baud rate in bits per second for handshake attempt
     * 
     * @warning RC input will not function until handshake completes successfully
     * @note This is called periodically by the RC protocol manager until handshake succeeds
     * @see is_bootstrapped() to check if handshake has completed
     */
    void process_handshake(uint32_t baudrate) override;
    
    /**
     * @brief Initiate bind procedure for pairing transmitter and receiver
     * 
     * @details Triggers the SRXL2 bind sequence which allows a Spektrum transmitter
     *          to pair with the receiver. This invokes the srxlOnBind() callback in
     *          the spm_srxl library to execute the bind protocol.
     *          
     *          Bind Procedure:
     *          1. Receiver enters bind mode (typically requires specific power-on sequence)
     *          2. Transmitter is placed in bind mode
     *          3. SRXL2 bind packets are exchanged
     *          4. Receiver stores transmitter GUID and configuration
     *          5. Binding complete, receiver ready for normal operation
     *          
     *          The bind procedure is hardware-specific and may require the receiver
     *          to be powered on in a specific sequence (e.g., with bind button pressed).
     * 
     * @note Typically called in response to user command via ground control station
     * @warning Vehicle should be disarmed and in a safe state before initiating bind
     * @see srxlOnBind() callback for bind protocol implementation
     */
    void start_bind(void) override;
    
    /**
     * @brief Periodic update for SRXL2 protocol state machine and telemetry transmission
     * 
     * @details Called periodically (typically at 50Hz) to maintain SRXL2 protocol state
     *          and send telemetry data back to the transmitter.
     *          
     *          Update Operations:
     *          - Check for handshake timeout and retry if needed
     *          - Manage telemetry packet transmission timing
     *          - Monitor failsafe conditions
     *          - Update RSSI (signal strength) values
     *          - Call srxlFillTelemetry() to construct outgoing telemetry packets
     *          
     *          Telemetry Downlink:
     *          Flight data including battery voltage, GPS position, altitude, attitude,
     *          and other sensor information is packaged into SRXL2 telemetry frames
     *          and transmitted back to the receiver for relay to the transmitter display.
     *          
     *          Timing Constraints:
     *          - Called at main loop rate (typically 50Hz)
     *          - Telemetry packets sent at configured rate (typically 5-10Hz)
     *          - Must not block for extended periods
     * 
     * @note Called by RC protocol manager at regular intervals
     * @see send_on_uart() for actual UART transmission
     */
    void update(void) override;
    
    /**
     * @brief Get singleton instance of SRXL2 protocol backend
     * 
     * @details Returns the singleton instance used for callback integration with
     *          the spm_srxl library. The library callbacks need to access the
     *          backend instance to process received data and send responses.
     * 
     * @return Pointer to singleton AP_RCProtocol_SRXL2 instance, or nullptr if not created
     */
    static AP_RCProtocol_SRXL2* get_singleton() {
        return _singleton;
    }

    /**
     * @brief Capture scaled RC channel input values from SRXL2 frame
     * 
     * @details Called by spm_srxl engine (via srxlOnReceive callback) when a valid
     *          SRXL2 control frame is decoded. Extracts RC channel data and failsafe
     *          status, then forwards to the RC input system.
     *          
     *          Channel values are 16-bit with typical range:
     *          - Center: 32768 (0x8000)
     *          - Min: 0 (0x0000)
     *          - Max: 65535 (0xFFFF)
     *          
     *          These are converted to microsecond PWM values (typically 1000-2000µs)
     *          for compatibility with ArduPilot's RC input processing.
     * 
     * @param[in] values_p Pointer to array of 16-bit channel values (up to SRXL2_MAX_CHANNELS)
     * @param[in] in_failsafe true if receiver is in failsafe mode (signal lost), false for normal operation
     * @param[in] rssi Received signal strength indication, -1 if not available, 0-100 for valid RSSI percentage
     * 
     * @note Called from spm_srxl callback context, must not block
     * @see SRXL2_MAX_CHANNELS for maximum channel count
     */
    void capture_scaled_input(const uint8_t *values_p, bool in_failsafe, int16_t rssi);
    
    /**
     * @brief Send data buffer to RC receiver via UART
     * 
     * @details Transmits a data packet (telemetry, handshake, or control) to the SRXL2
     *          receiver via the configured UART interface. Used for bidirectional
     *          communication including telemetry downlink and handshake negotiation.
     *          
     *          Transmission Timing:
     *          During normal operation, UART transmission is constrained to occur only
     *          within the receiver's expected timing windows to avoid collisions with
     *          incoming data. During bootstrap/failsafe, timing constraints are relaxed.
     * 
     * @param[in] pBuffer Pointer to byte buffer containing data to transmit
     * @param[in] length Number of bytes to transmit from buffer (must be ≤ SRXL2_FRAMELEN_MAX)
     * 
     * @note Transmission timing controlled by _in_bootstrap_or_failsafe flag
     * @warning Buffer must remain valid for duration of transmission
     */
    void send_on_uart(uint8_t* pBuffer, uint8_t length);
    
    /**
     * @brief Change UART baud rate for SRXL2 communication
     * 
     * @details Updates the UART baud rate following successful handshake negotiation.
     *          The SRXL2 handshake determines the optimal baud rate based on receiver
     *          capabilities and connection quality.
     *          
     *          Typical baud rates:
     *          - 115200 bps: Standard rate, good compatibility
     *          - 400000 bps: High-speed rate for reduced latency (requires good signal quality)
     *          
     *          Baud rate change sequence:
     *          1. Complete current transmission
     *          2. Switch UART hardware to new baud rate
     *          3. Resume communication at new rate
     * 
     * @param[in] baudrate New UART baud rate in bits per second (typically 115200 or 400000 bps)
     * 
     * @note Called during handshake after baud rate negotiation completes
     * @warning Both receiver and flight controller must switch rates simultaneously
     */
    void change_baud_rate(uint32_t baudrate);

private:

    /// Singleton instance pointer for callback access from spm_srxl library
    static AP_RCProtocol_SRXL2* _singleton;

    /**
     * @brief Internal byte processing with timestamp for SRXL2 state machine
     * 
     * @param[in] timestamp_us Timestamp in microseconds when byte was received
     * @param[in] byte Data byte to process
     */
    void _process_byte(uint32_t timestamp_us, uint8_t byte);
    
    /**
     * @brief Complete bootstrap/handshake process with receiver device ID
     * 
     * @details Finalizes the handshake by recording the receiver's device ID,
     *          marking the protocol as ready for normal operation.
     * 
     * @param[in] device_id SRXL2 device identifier from receiver (non-zero)
     */
    void _bootstrap(uint8_t device_id);
    
    /**
     * @brief Check if handshake/bootstrap has completed successfully
     * 
     * @return true if device is bootstrapped and ready for normal operation, false if still in handshake
     */
    bool is_bootstrapped() const { return _device_id != 0; }

    /// Receive buffer for raw SRXL2 frame data (header + payload + CRC, up to 80 bytes)
    uint8_t _buffer[SRXL2_FRAMELEN_MAX];
    
    /// Current number of bytes received in _buffer for the frame being decoded
    uint8_t _buflen;
    
    /// Timestamp in milliseconds of last state machine execution (for timeout detection)
    uint32_t _last_run_ms;
    
    /// Decoded RC channel data buffer as PWM pulse widths in microseconds (typically 1000-2000µs)
    uint16_t _channels[SRXL2_MAX_CHANNELS < MAX_RCIN_CHANNELS ? SRXL2_MAX_CHANNELS : MAX_RCIN_CHANNELS];
    
    /// Flag controlling UART transmission timing: true during bootstrap/failsafe allows relaxed timing, false requires strict timing
    bool _in_bootstrap_or_failsafe;
    
    /// SRXL2 device ID from receiver (0 = not bootstrapped, non-zero = bootstrapped with device ID)
    uint8_t _device_id;

    /**
     * @brief SRXL2 frame decoder state machine states
     */
    enum {
        STATE_IDLE,      ///< Idle state: waiting for frame start sync byte
        STATE_NEW,       ///< New frame state: processing header (length, type) and preparing for payload
        STATE_COLLECT    ///< Collect state: accumulating frame payload bytes and validating CRC
    };
    
    /// Expected total frame length in bytes (extracted from frame header)
    uint8_t _frame_len_full;
    
    /// Current decoder state (STATE_IDLE, STATE_NEW, or STATE_COLLECT)
    uint8_t _decode_state;
    
    /// Next decoder state to transition to when next byte arrives
    uint8_t _decode_state_next;
    
    /// Current failsafe status: true if receiver signal lost, false for normal operation
    bool _in_failsafe = false;
    
    /// Most recent RSSI (signal strength) value: -1 if unavailable, 0-100 for valid percentage
    int16_t _new_rssi = -1;
    
    /// Timestamp in milliseconds of last handshake attempt (for retry timing)
    uint32_t _last_handshake_ms;
    
    /// Timestamp in milliseconds when current handshake sequence started (for timeout detection)
    uint32_t _handshake_start_ms;
};

#endif  // AP_RCPROTOCOL_SRXL2_ENABLED
