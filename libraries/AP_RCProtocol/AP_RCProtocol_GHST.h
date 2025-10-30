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
 * @file AP_RCProtocol_GHST.h
 * @brief ImmersionRC Ghost (GHST) RC protocol decoder with bidirectional telemetry
 * 
 * @details This file implements the ImmersionRC Ghost (GHST) protocol, a long-range
 *          RC communication protocol designed for FPV racing and long-range applications.
 *          GHST provides low-latency RC control combined with bidirectional telemetry
 *          and supports multiple RF modes from 19Hz to 500Hz update rates.
 * 
 *          Protocol Features:
 *          - Up to 16 RC channels (4 primary 12-bit channels + 12 auxiliary channels)
 *          - Bidirectional telemetry (battery, GPS, sensors, MSP)
 *          - Link quality (LQ) and RSSI reporting
 *          - Multiple RF modes: Normal (55Hz), Race (160Hz), PureRace (250Hz), 
 *            Race250 (250Hz), Race500 (500Hz), Solid150 (150Hz), Solid250 (250Hz), LR (19Hz)
 *          - CRC-8 frame validation
 *          - Frame synchronization with device addressing
 * 
 *          Protocol Specifications:
 *          - Baudrate: 420000 bps (fixed high-speed serial)
 *          - Frame format: [Device Address][Length][Type][Payload...][CRC]
 *          - Channel resolution: 11-bit or 12-bit depending on frame type
 *          - Maximum frame length: 14 bytes
 *          - Receiver timeout: 150ms (detects RX power loss)
 *          - Transmitter timeout: 500ms (matches copter failsafe)
 * 
 * @note GHST is proprietary to ImmersionRC and used primarily with Ghost RC systems
 * @warning High-speed serial required: Must be configured for 420000 baud, 8N1
 * @warning Frame timing critical: Improper baudrate will cause frame desynchronization
 * 
 * @see AP_RCProtocol_Backend
 * @see AP_GHST_Telem for telemetry transmission implementation
 */
#pragma once

#include "AP_RCProtocol_config.h"

#if AP_RCPROTOCOL_GHST_ENABLED

#include "AP_RCProtocol.h"
#include <AP_Math/AP_Math.h>
#include <RC_Channel/RC_Channel.h>
#include "SoftSerial.h"

// GHST Protocol Constants

/// Maximum number of RC channels supported by GHST protocol (4 primary + 12 auxiliary)
#define GHST_MAX_CHANNELS   16U

/// Maximum possible frame length in bytes (including header, payload, and CRC)
#define GHST_FRAMELEN_MAX   14U

/// Header length in bytes (device address + length field)
#define GHST_HEADER_LEN     2U

/// Maximum payload size in bytes (frame length minus header)
#define GHST_FRAME_PAYLOAD_MAX (GHST_FRAMELEN_MAX - GHST_HEADER_LEN)

/// GHST protocol baudrate: 420000 bps (high-speed serial, 8N1)
#define GHST_BAUDRATE      420000U

/// Transmitter timeout in microseconds (500ms - matches copter failsafe period)
/// After this period without valid TX frames, transmitter is considered disconnected
#define GHST_TX_TIMEOUT    500000U

/// Receiver timeout in microseconds (150ms - exceeds typical ping frequency)
/// After this period without valid RX frames, receiver is considered disconnected
#define GHST_RX_TIMEOUT    150000U

/**
 * @class AP_RCProtocol_GHST
 * @brief ImmersionRC Ghost (GHST) protocol decoder backend with telemetry support
 * 
 * @details This class implements decoding of GHST RC frames and encoding of telemetry
 *          frames for bidirectional communication with ImmersionRC Ghost receivers.
 *          
 *          Frame Processing:
 *          - Receives and validates GHST frames with CRC-8 checking
 *          - Decodes RC channel data (11-bit or 12-bit resolution)
 *          - Extracts link statistics (RSSI, link quality, RF mode)
 *          - Processes telemetry requests and VTX control commands
 *          
 *          Protocol State Management:
 *          - Tracks transmitter and receiver activity timeouts
 *          - Monitors link quality and signal strength
 *          - Reports RF mode (update rate) for link status
 *          - Provides singleton access for telemetry transmission
 *          
 *          Channel Mapping:
 *          - Primary channels (1-4): 12-bit resolution, low latency
 *          - Auxiliary channels (5-16): 8-bit resolution, transmitted in groups
 *          - Frame types determine which channel groups are present
 *          
 *          Telemetry Features:
 *          - Battery status (voltage, current, capacity)
 *          - GPS data (position, velocity, satellites)
 *          - Sensor data (magnetometer, barometer, variometer)
 *          - MSP protocol support for configuration
 *          - VTX control for video transmitter management
 * 
 * @note Singleton pattern used for telemetry access from AP_GHST_Telem
 * @note Thread-safe link status access via volatile struct
 * 
 * @warning Must be configured for 420000 baud serial communication
 * @warning Frame synchronization requires consistent timing and no serial errors
 * @warning Link quality derived from uplink LQ percentage (0-100)
 * 
 * @see AP_RCProtocol_Backend for base protocol interface
 * @see AP_GHST_Telem for telemetry transmission implementation
 */
class AP_RCProtocol_GHST : public AP_RCProtocol_Backend {
public:
    /**
     * @brief Construct GHST protocol decoder backend
     * 
     * @details Initializes GHST frame decoder, sets up singleton instance,
     *          and prepares internal buffers for frame reception and telemetry.
     * 
     * @param[in] _frontend Reference to the parent AP_RCProtocol frontend
     */
    AP_RCProtocol_GHST(AP_RCProtocol &_frontend);
    
    /**
     * @brief Destructor - cleans up GHST protocol instance
     */
    virtual ~AP_RCProtocol_GHST();
    
    /**
     * @brief Process incoming byte from GHST receiver
     * 
     * @details Implements GHST frame decoding state machine:
     *          1. Frame synchronization on device address
     *          2. Frame length validation
     *          3. Payload accumulation
     *          4. CRC-8 validation
     *          5. Frame type dispatch (RC channels, link stats, telemetry, MSP)
     *          
     *          Frame Format: [Device Address][Length][Type][Payload...][CRC]
     *          - Device Address: GHST_ADDRESS_* (flight controller, goggles, receiver)
     *          - Length: Total frame length including header and CRC
     *          - Type: FrameType enum defining payload structure
     *          - Payload: 0-11 bytes depending on frame type
     *          - CRC: 8-bit CRC over entire frame except CRC byte
     * 
     * @param[in] byte      Incoming byte from serial port
     * @param[in] baudrate  Serial baudrate in bps (must be 420000 for GHST)
     * 
     * @note Called at serial interrupt rate - keep processing minimal
     * @warning Baudrate must be exactly 420000 bps or frames will not synchronize
     * @warning Invalid CRC causes frame to be discarded silently
     */
    void process_byte(uint8_t byte, uint32_t baudrate) override;
    
    /**
     * @brief Process protocol handshake for baudrate detection
     * 
     * @details GHST uses fixed 420000 baud, so this validates baudrate
     *          is correctly configured for GHST protocol.
     * 
     * @param[in] baudrate Serial baudrate to validate (bps)
     */
    void process_handshake(uint32_t baudrate) override;
    
    /**
     * @brief Update GHST protocol state and process telemetry
     * 
     * @details Called periodically from main thread to:
     *          - Process outgoing telemetry frames
     *          - Update link status indicators
     *          - Handle timeout detection
     *          - Manage telemetry scheduling
     * 
     * @note Called at scheduler rate (typically 50-400Hz depending on vehicle)
     */
    void update(void) override;

    /**
     * @brief Check if GHST receiver is active
     * 
     * @details Determines receiver connectivity by checking if valid frames
     *          have been received within the timeout period (150ms).
     *          Used to detect receiver power loss or baudrate mismatches.
     * 
     * @return true if receiver frames received within GHST_RX_TIMEOUT (150ms)
     * @return false if receiver timeout exceeded (disconnected or powered off)
     * 
     * @note Timeout period exceeds typical ping frequency to avoid false positives
     */
    bool is_rx_active() const override {
        return AP_HAL::micros() < _last_rx_frame_time_us + GHST_RX_TIMEOUT;
    }

    /**
     * @brief Check if GHST transmitter is active
     * 
     * @details Determines transmitter connectivity by checking if valid RC frames
     *          have been received within the timeout period (500ms).
     *          Used to trigger failsafe when transmitter is turned off or out of range.
     * 
     * @return true if transmitter frames received within GHST_TX_TIMEOUT (500ms)
     * @return false if transmitter timeout exceeded (failsafe condition)
     * 
     * @note Timeout matches ArduPilot Copter failsafe period (500ms)
     * @warning Failsafe triggered when this returns false - critical for safety
     */
    bool is_tx_active() const {
        // this is the same as the Copter failsafe timeout
        return AP_HAL::micros() < _last_tx_frame_time_us + GHST_TX_TIMEOUT;
    }

    /**
     * @brief Get singleton instance of GHST protocol handler
     * 
     * @details Provides access to the single GHST protocol instance for
     *          telemetry transmission from AP_GHST_Telem class.
     * 
     * @return Pointer to singleton instance, or nullptr if not instantiated
     * 
     * @note Used by AP_GHST_Telem to access link status and transmit telemetry
     * @see AP_GHST_Telem
     */
    static AP_RCProtocol_GHST* get_singleton() {
        return _singleton;
    }

    /**
     * @enum FrameType
     * @brief GHST protocol frame type identifiers
     * 
     * @details Defines all supported GHST frame types for uplink (UL) and downlink (DL):
     * 
     *          Uplink Frames (Transmitter to Receiver/FC):
     *          - RC Channel Frames: Contain 4 primary 12-bit channels + 4 auxiliary 8-bit channels
     *          - RSSI Frames: Include link statistics (RSSI, LQ, RF mode)
     *          - Control Frames: VTX channel control, setup commands
     *          - MSP Frames: MSP protocol requests and writes
     * 
     *          Downlink Frames (Receiver/FC to Transmitter):
     *          - Telemetry: Battery status, GPS data, sensors
     *          - MSP Responses: MSP protocol responses
     * 
     *          Channel Resolution:
     *          - 0x1x frames: 11-bit primary channels (0-2047)
     *          - 0x3x frames: 12-bit primary channels (0-4095)
     * 
     * @note Frame type is third byte in GHST frame after device address and length
     */
    enum FrameType {
        // Uplink RC Frames - 11-bit primary channels (0x10-0x1F)
        GHST_UL_RC_CHANS_HS4_5TO8 = 0x10,    ///< RC frame: 4 primary 11-bit channels + auxiliary CH5-8 (8-bit)
        GHST_UL_RC_CHANS_HS4_9TO12 = 0x11,   ///< RC frame: 4 primary 11-bit channels + auxiliary CH9-12 (8-bit)
        GHST_UL_RC_CHANS_HS4_13TO16 = 0x12,  ///< RC frame: 4 primary 11-bit channels + auxiliary CH13-16 (8-bit)
        GHST_UL_RC_CHANS_RSSI = 0x13,        ///< RC frame with link statistics: RSSI (dBm), LQ (0-100%), RF mode, telemetry status
        GHST_UL_RC_VTX_CTRL = 0x14,          ///< VTX control frame: Video transmitter channel/power management from goggles/FC
        // 0x15 -> 0x1F reserved for future uplink frame types
        
        // Uplink Control/Configuration Frames (0x20-0x2F)
        GHST_UL_VTX_SETUP = 0x20,            ///< VTX setup frame without primary channels (GECO goggles only)
        GHST_UL_MSP_REQ = 0x21,              ///< MSP request frame: Read configuration/status via MSP protocol
        GHST_UL_MSP_WRITE = 0x22,            ///< MSP write frame: Modify configuration via MSP protocol

        // Downlink Telemetry Frames (0x23-0x2F)
        GHST_DL_PACK_STAT = 0x23,            ///< Battery telemetry: Voltage (V), current (A), consumed capacity (mAh)
        GHST_DL_GPS_PRIMARY = 0x25,          ///< Primary GPS telemetry: Latitude, longitude, altitude, satellites
        GHST_DL_GPS_SECONDARY = 0x26,        ///< Secondary GPS telemetry: Groundspeed, heading, HDOP
        GHST_DL_MAGBARO = 0x27,              ///< Sensor telemetry: Magnetometer heading, barometric altitude, variometer
        GHST_DL_MSP_RESP = 0x28,             ///< MSP response frame: Configuration/status data response

        // Uplink RC Frames - 12-bit primary channels (0x30-0x3F)
        GHST_UL_RC_CHANS_HS4_12_5TO8 = 0x30,   ///< RC frame: 4 primary 12-bit channels + auxiliary CH5-8 (8-bit)
        GHST_UL_RC_CHANS_HS4_12_9TO12 = 0x31,  ///< RC frame: 4 primary 12-bit channels + auxiliary CH9-12 (8-bit)
        GHST_UL_RC_CHANS_HS4_12_13TO16 = 0x32, ///< RC frame: 4 primary 12-bit channels + auxiliary CH13-16 (8-bit)
        GHST_UL_RC_CHANS_12_RSSI = 0x33,       ///< RC frame with link statistics: 12-bit channels, RSSI, LQ, RF mode
        // 0x34 -> 0x3F reserved for future 12-bit frame types
    };

    /**
     * @enum DeviceAddress
     * @brief GHST bus device address identifiers
     * 
     * @details Device addresses used for GHST frame routing and identification.
     *          First byte of every GHST frame identifies source or destination device.
     *          Used for frame filtering and multi-device bus arbitration.
     * 
     * @note Addresses allow multiple devices on same serial bus (future expansion)
     */
    enum DeviceAddress {
        GHST_ADDRESS_FLIGHT_CONTROLLER = 0x82,  ///< Flight controller (autopilot) address
        GHST_ADDRESS_GOGGLES = 0x83,            ///< FPV goggles/display device address
        GHST_ADDRESS_GHST_RECEIVER = 0x89,      ///< Ghost RC receiver address
    };

    /**
     * @struct Frame
     * @brief GHST protocol frame structure
     * 
     * @details Complete GHST frame layout for all frame types:
     *          [Device Address][Length][Type][Payload 0-11 bytes][CRC]
     *          
     *          Frame Fields:
     *          - device_address: Source/destination device (DeviceAddress enum)
     *          - length: Total frame length including header and CRC (2-14 bytes)
     *          - type: Frame type identifier (FrameType enum)
     *          - payload: Variable length payload (0-11 bytes)
     *          - CRC: 8-bit CRC calculated over entire frame (not in struct, appended after payload)
     * 
     * @note CRC byte transmitted after payload, not included in this struct
     * @note Payload interpretation depends on frame type
     * @note Frame synchronization achieved via device address matching
     */
    struct Frame {
        uint8_t device_address;  ///< Source/destination device address (DeviceAddress enum)
        uint8_t length;          ///< Total frame length including device_address, length, type, payload, and CRC
        uint8_t type;            ///< Frame type identifier (FrameType enum)
        uint8_t payload[GHST_FRAME_PAYLOAD_MAX - 1]; ///< Variable payload (type byte accounted for separately)
    } PACKED;

    /**
     * @struct Channels12Bit_4Chan
     * @brief Packed 12-bit channel data structure
     * 
     * @details Stores 4 channels with 12-bit resolution (0-4095) in 48 bits (6 bytes).
     *          Used for primary RC channels (roll, pitch, yaw, throttle) which require
     *          highest resolution and lowest latency.
     * 
     * @note Little-endian architecture required for bit field packing
     * @warning Compilation will fail on big-endian systems
     */
    struct Channels12Bit_4Chan {
#if __BYTE_ORDER != __LITTLE_ENDIAN
#error "Only supported on little-endian architectures"
#endif
        uint32_t ch0 : 12;  ///< Channel 0 value (0-4095, typically roll)
        uint32_t ch1 : 12;  ///< Channel 1 value (0-4095, typically pitch)
        uint32_t ch2 : 12;  ///< Channel 2 value (0-4095, typically throttle)
        uint32_t ch3 : 12;  ///< Channel 3 value (0-4095, typically yaw)
    } PACKED;

    /**
     * @struct RadioFrame
     * @brief Standard RC control frame payload
     * 
     * @details Contains 4 high-resolution primary channels (12-bit) and
     *          4 low-resolution auxiliary channels (8-bit, 0-255).
     *          Used for frame types 0x10-0x12 (11-bit primary) and 0x30-0x32 (12-bit primary).
     * 
     * @note Primary channels: Roll, pitch, throttle, yaw (high resolution)
     * @note Auxiliary channels: Mode switches, flight options (lower resolution sufficient)
     */
    struct RadioFrame {
#if __BYTE_ORDER != __LITTLE_ENDIAN
#error "Only supported on little-endian architectures"
#endif
        Channels12Bit_4Chan channels;   ///< Primary channels (12-bit resolution): roll, pitch, throttle, yaw
        uint8_t lowres_channels[4];     ///< Auxiliary channels (8-bit resolution): switches, modes, etc.
    } PACKED;

    /**
     * @struct LinkStatisticsFrame
     * @brief Link quality and signal strength telemetry frame
     * 
     * @details Provides comprehensive link status information including:
     *          - Signal strength (RSSI in dBm)
     *          - Link quality percentage (0-100%)
     *          - RF mode (update rate configuration)
     *          - Telemetry status and configuration flags
     *          - Transmitter power level
     * 
     *          Used by frame types GHST_UL_RC_CHANS_RSSI (0x13) and
     *          GHST_UL_RC_CHANS_12_RSSI (0x33).
     * 
     * @note RSSI reported as positive value (multiply by -1 to get actual dBm)
     * @note Link quality represents uplink packet success rate (0-100%)
     */
    struct LinkStatisticsFrame {
#if __BYTE_ORDER != __LITTLE_ENDIAN
#error "Only supported on little-endian architectures"
#endif
        Channels12Bit_4Chan channels;  ///< Primary RC channels (same as RadioFrame)
        uint8_t link_quality;          ///< Uplink packet success rate: 0-100 (percentage)
        uint8_t rssi_dbm;              ///< Signal strength: dBm * -1 (positive value, negate to get actual dBm)
        uint8_t protocol : 5;          ///< RF mode identifier (RFMode enum): determines update rate
        uint8_t telemetry : 1;         ///< Telemetry enabled flag: 1=enabled, 0=disabled
        uint8_t alt_scale : 1;         ///< Altitude scaling flag for telemetry display
        uint8_t reserved : 1;          ///< Reserved bit for future use
        int8_t tx_power;               ///< Transmitter power level in dBm (signed)
    } PACKED;

    /**
     * @enum RFMode
     * @brief GHST RF mode identifiers defining link update rates
     * 
     * @details RF modes determine the balance between range, latency, and reliability:
     *          - Lower rates (LR, Normal): Longer range, higher latency
     *          - Higher rates (Race250, Race500): Lower latency, shorter range
     *          
     *          Mode Selection:
     *          - LR Mode: Maximum range for long-range flight
     *          - Normal Mode: Balanced for general flying
     *          - Race Modes: Minimum latency for racing
     * 
     * @note RF mode reported in LinkStatisticsFrame protocol field
     * @note Actual update rate may vary based on link conditions
     */
    enum RFMode {
        GHST_RF_MODE_NORMAL = 5,     ///< Normal mode: 55Hz update rate, balanced range/latency
        GHST_RF_MODE_RACE = 6,       ///< Race mode: 160Hz update rate, reduced range
        GHST_RF_MODE_PURERACE = 7,   ///< Pure race mode: 250Hz update rate, minimum latency
        GHST_RF_MODE_LR = 8,         ///< Long range mode: 19Hz update rate, maximum range
        GHST_RF_MODE_RACE250 = 10,   ///< Race 250 mode: 250Hz update rate
        GHST_RF_MODE_RACE500 = 11,   ///< Race 500 mode: 500Hz update rate, shortest latency
        GHTS_RF_MODE_SOLID150 = 12,  ///< Solid 150 mode: 150Hz update rate, stable mid-range
        GHST_RF_MODE_SOLID250 = 13,  ///< Solid 250 mode: 250Hz update rate, stable high-speed
        RF_MODE_MAX_MODES,           ///< Maximum number of RF modes (for array sizing)
        RF_MODE_UNKNOWN,             ///< Unknown or invalid RF mode
    };

    /**
     * @struct LinkStatus
     * @brief Current link quality status information
     * 
     * @details Aggregated link status extracted from LinkStatisticsFrame and
     *          provided to telemetry system for display and logging.
     *          Values updated on receipt of RSSI frames.
     * 
     * @note Thread-safe access via volatile qualifier
     * @note RSSI initialized to -1 to indicate no data received yet
     * @see get_link_status() for access from telemetry thread
     */
    struct LinkStatus {
        int16_t rssi = -1;           ///< Signal strength in dBm (-1 = no data yet, typical range -30 to -100 dBm)
        int16_t link_quality = -1;   ///< Uplink success rate 0-100% (-1 = no data yet)
        uint8_t rf_mode;             ///< Current RF mode (RFMode enum value)
    };

    /**
     * @brief Check if GHST telemetry transmission is supported
     * 
     * @details Determines if bidirectional telemetry is available based on
     *          receiver capabilities and configuration.
     * 
     * @return true if telemetry can be transmitted to receiver/transmitter
     * @return false if telemetry not available (receiver limitation or disabled)
     */
    bool is_telemetry_supported() const;

    /**
     * @brief Get current link status for telemetry reporting
     * 
     * @details Provides access to live link quality metrics from AP_GHST_Telem
     *          thread. Uses volatile qualifier for thread-safe access without
     *          explicit synchronization (reads are atomic for these data types).
     * 
     * @return Reference to current link status (RSSI, LQ, RF mode)
     * 
     * @note Called from AP_GHST_Telem to populate telemetry frames
     * @note Thread-safe via volatile qualifier on LinkStatus struct
     * @see AP_GHST_Telem
     */
    const volatile LinkStatus& get_link_status() const {
        return _link_status;
    }

    /**
     * @brief Get current RF link update rate
     * 
     * @details Returns the update rate (Hz) corresponding to current RF mode.
     *          Used for telemetry display and link performance monitoring.
     * 
     * @return Link update rate in Hz (19, 55, 150, 160, 250, or 500 Hz)
     * @return 0 if RF mode unknown or invalid
     * 
     * @note Update rate determines control latency and maximum range
     */
    uint16_t get_link_rate()  const;

    /**
     * @brief Get human-readable RF protocol mode string
     * 
     * @details Returns descriptive string for current RF mode for display
     *          in ground station and OSD.
     * 
     * @return String describing RF mode (e.g., "Normal", "Race500", "LR")
     * @return "Unknown" if RF mode invalid
     * 
     * @note Used for OSD and telemetry display
     */
    const char* get_protocol_string() const;

private:
    // Internal frame processing and telemetry management
    
    struct Frame _frame;              ///< Current incoming frame buffer (receive state machine)
    struct Frame _telemetry_frame;    ///< Outgoing telemetry frame buffer (transmit queue)
    uint8_t _frame_ofs;               ///< Current byte offset in frame during reception (0-13)
    uint8_t _frame_crc;               ///< Running CRC-8 calculation for current frame

    /// Maximum channels to decode (limited by ArduPilot MAX_RCIN_CHANNELS)
    const uint8_t MAX_CHANNELS = MIN((uint8_t)GHST_MAX_CHANNELS, (uint8_t)MAX_RCIN_CHANNELS);

    static AP_RCProtocol_GHST* _singleton;  ///< Singleton instance for telemetry access

    /**
     * @brief Internal byte processor with timestamp tracking
     * @param[in] timestamp_us Microsecond timestamp of byte reception
     * @param[in] byte Incoming byte to process
     */
    void _process_byte(uint32_t timestamp_us, uint8_t byte);
    
    /**
     * @brief Decode complete GHST frame and extract RC channels
     * @return true if frame decoded successfully and channels updated
     */
    bool decode_ghost_packet();
    
    /**
     * @brief Process outgoing telemetry frame transmission
     * @param[in] check_constraint Whether to check telemetry rate limiting
     * @return true if telemetry frame transmitted successfully
     */
    bool process_telemetry(bool check_constraint = true);
    
    /**
     * @brief Process received link statistics frame and update link status
     * @param[in] data Pointer to LinkStatisticsFrame payload
     */
    void process_link_stats_frame(const void* data);
    
    /**
     * @brief Transmit GHST frame with CRC calculation
     * @param[in] frame Pointer to frame to transmit
     * @return true if frame transmitted successfully
     */
    bool write_frame(Frame* frame);
    
    /**
     * @brief Get current UART for frame transmission
     * @return Pointer to UART driver
     */
    AP_HAL::UARTDriver* get_current_UART() { return get_available_UART(); }

    /// RC channel values buffer: PWM microseconds (typically 1000-2000 µs)
    uint16_t _channels[GHST_MAX_CHANNELS];

    /**
     * @brief Add byte to frame buffer at specified index
     * @param[in] index Byte position in frame (0-13)
     * @param[in] b Byte value to store
     */
    void add_to_buffer(uint8_t index, uint8_t b) { ((uint8_t*)&_frame)[index] = b; }

    uint32_t _last_frame_time_us;      ///< Timestamp of last frame received (any type), microseconds
    uint32_t _last_tx_frame_time_us;   ///< Timestamp of last transmitter RC frame, microseconds (for failsafe)
    uint32_t _last_rx_frame_time_us;   ///< Timestamp of last receiver frame, microseconds (for power loss detection)
    uint32_t _start_frame_time_us;     ///< Timestamp when current frame reception started, microseconds
    bool telem_available;              ///< True if telemetry data ready to transmit
    bool _use_lq_for_rssi;             ///< Use link quality for RSSI value if true (fallback mode)
    
    /**
     * @brief Convert raw link quality to scaled RSSI value
     * @param[in] uplink_lq Raw uplink link quality (0-100%)
     * @return Scaled LQ value for RSSI display
     */
    int16_t derive_scaled_lq_value(uint8_t uplink_lq);

    volatile struct LinkStatus _link_status;  ///< Current link status (volatile for thread-safe access)

    /// Lookup table: RF mode enum to update rate in Hz
    static const uint16_t RF_MODE_RATES[RFMode::RF_MODE_MAX_MODES];
};

/**
 * @namespace AP
 * @brief ArduPilot global namespace for singleton accessors
 */
namespace AP {
    /**
     * @brief Get GHST protocol singleton instance
     * 
     * @details Provides global access to GHST protocol handler for
     *          telemetry and status queries.
     * 
     * @return Pointer to GHST protocol singleton, or nullptr if not initialized
     * 
     * @note Convenience accessor equivalent to AP_RCProtocol_GHST::get_singleton()
     */
    AP_RCProtocol_GHST* ghost();
};

#endif  // AP_RCPROTOCOL_GHST_ENABLED
