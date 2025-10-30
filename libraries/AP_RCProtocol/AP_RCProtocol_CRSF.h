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

/*
 * Crossfire constants provided by Team Black Sheep under terms of the 2-Clause BSD License
 */

/**
 * @file AP_RCProtocol_CRSF.h
 * @brief Crossfire (CRSF) and ExpressLRS (ELRS) RC protocol implementation with bidirectional telemetry
 * 
 * @details This file implements the TBS Crossfire protocol decoder and ExpressLRS variant support for
 *          RC channel data reception and bidirectional telemetry transmission. The protocol supports:
 *          
 *          - Up to 24 RC channels with 11-bit resolution (172-1811 range)
 *          - Multiple frame formats: standard packed channels, subset channels, 11-bit channels
 *          - Bidirectional telemetry for GPS, battery, attitude, flight mode, link statistics
 *          - VTX (video transmitter) control and configuration
 *          - Link quality and RSSI reporting for OSD display
 *          - CRSF v3 with dynamic baudrate negotiation
 *          - ExpressLRS variant with 420kbaud bootstrap option
 *          
 *          Protocol Specifications:
 *          - Standard baudrate: 416666 bps (CRSF) or 420000 bps (ELRS)
 *          - Frame format: [Device Address][Length][Type][Payload...][CRC-8]
 *          - CRC-8: DVB-S2 polynomial for error detection
 *          - Frame synchronization timeout: 150ms (receiver), 500ms (transmitter)
 *          
 * @warning Frame processing is timing-critical and must maintain synchronization at high baudrates.
 *          Lost synchronization requires frame boundary detection via CRC validation.
 * 
 * @note Crossfire constants provided by Team Black Sheep under 2-Clause BSD License
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */
#pragma once

#include "AP_RCProtocol_config.h"
#include <AP_RCTelemetry/AP_RCTelemetry_config.h>

#if AP_RCPROTOCOL_CRSF_ENABLED

#include "AP_RCProtocol.h"
#include <AP_Math/AP_Math.h>
#include <RC_Channel/RC_Channel.h>
#include "SoftSerial.h"
#include <AP_OSD/AP_OSD_config.h>

/**
 * @name CRSF Protocol Constants
 * @{
 */

/** @brief Maximum number of RC channels supported by CRSF protocol */
#define CRSF_MAX_CHANNELS   24U

/** @brief Maximum CRSF frame length including all fields (bytes) */
#define CRSF_FRAMELEN_MAX   64U

/** @brief CRSF frame header length: device_address + length (bytes) */
#define CRSF_HEADER_LEN     2U

/** @brief Maximum CRSF frame payload size: max_frame - header (bytes) */
#define CRSF_FRAME_PAYLOAD_MAX (CRSF_FRAMELEN_MAX - CRSF_HEADER_LEN)

/** @brief Minimum valid frame length field value */
#define CRSF_FRAME_LENGTH_MIN 2

/**
 * @brief Standard Crossfire baudrate (bits per second)
 * @details 416666 bps = 416.666 kbaud, provides ~40kHz byte rate
 */
#define CRSF_BAUDRATE      416666U

/**
 * @brief ExpressLRS bootstrap baudrate (bits per second)
 * @details 420000 bps = 420 kbaud, slightly higher than CRSF standard
 */
#define ELRS_BAUDRATE      420000U

/**
 * @brief Transmitter timeout period (microseconds)
 * @details 500ms timeout matches ArduCopter radio failsafe timeout.
 *          Transmitter considered disconnected if no RC channels received within this period.
 */
#define CRSF_TX_TIMEOUT    500000U

/**
 * @brief Receiver timeout period (microseconds)
 * @details 150ms timeout exceeds typical link statistics frame period (~200ms).
 *          Receiver considered disconnected if no frames received within this period.
 */
#define CRSF_RX_TIMEOUT    150000U

/** @} */ // end of CRSF Protocol Constants

/**
 * @class AP_RCProtocol_CRSF
 * @brief Crossfire/ExpressLRS RC protocol decoder with bidirectional telemetry support
 * 
 * @details This class implements the TBS Crossfire and ExpressLRS RC protocols, providing:
 *          
 *          Channel Decoding:
 *          - Processes incoming serial data at 416666 bps (CRSF) or 420000 bps (ELRS)
 *          - Extracts up to 24 RC channels from packed frame formats
 *          - Supports multiple encoding formats: standard 11-bit, subset channels, variable resolution
 *          - Channel values decoded to 172-1811 range (11-bit resolution)
 *          
 *          Frame Processing:
 *          - Byte-by-byte frame synchronization using header detection and CRC-8 validation
 *          - Frame format: [Device Address (1)][Length (1)][Type (1)][Payload (0-61)][CRC-8 (1)]
 *          - CRC-8 DVB-S2 polynomial provides error detection for noisy RF environments
 *          - Automatic frame boundary recovery on CRC failures
 *          
 *          Bidirectional Telemetry:
 *          - Transmits GPS, battery, attitude, flight mode to TX module
 *          - Receives link statistics (RSSI, LQ, SNR, TX power) for OSD display
 *          - Processes VTX control commands from transmitter
 *          - Handles device configuration and parameter read/write
 *          
 *          Protocol Variants:
 *          - CRSF: TBS Crossfire standard protocol
 *          - ELRS: ExpressLRS variant with higher RF packet rates
 *          - CRSF v3: Dynamic baudrate negotiation for improved latency
 *          
 *          Link Quality Monitoring:
 *          - Tracks receiver activity with 150ms timeout (CRSF_RX_TIMEOUT)
 *          - Tracks transmitter activity with 500ms timeout (CRSF_TX_TIMEOUT, matches copter failsafe)
 *          - Provides link statistics for OSD and telemetry logging
 *          
 * @note Frame processing runs in RC input thread context at high priority
 * 
 * @warning Timing-critical operations: frame synchronization requires consistent processing at
 *          416666/420000 bps with minimal jitter. Delays can cause frame desynchronization.
 */
class AP_RCProtocol_CRSF : public AP_RCProtocol_Backend {
public:
    /**
     * @brief Constructor for CRSF protocol handler
     * 
     * @param[in] _frontend Reference to parent AP_RCProtocol instance for channel output
     */
    AP_RCProtocol_CRSF(AP_RCProtocol &_frontend);
    
    /**
     * @brief Destructor - cleans up UART resources
     */
    virtual ~AP_RCProtocol_CRSF();
    
    /**
     * @brief Process a single byte of incoming CRSF data
     * 
     * @details Implements byte-by-byte frame synchronization and decoding:
     *          1. Detects frame header (device address + length fields)
     *          2. Accumulates frame payload bytes
     *          3. Validates complete frame with CRC-8 check
     *          4. Decodes RC channels or processes telemetry/command frames
     *          
     *          Frame synchronization uses length field validation and CRC to detect boundaries.
     *          On CRC failure, parser skips to next potential frame start.
     * 
     * @param[in] byte    Single byte from UART receive buffer
     * @param[in] baudrate Current UART baudrate in bits per second (416666 or 420000)
     * 
     * @note Called from RC input thread at UART reception rate (~40kHz at 416666 baud)
     * 
     * @warning Must maintain synchronization - any delays can cause frame loss
     */
    void process_byte(uint8_t byte, uint32_t baudrate) override;
    
    /**
     * @brief Process initial handshake and protocol detection
     * 
     * @details Handles CRSF v3 baudrate negotiation and initial protocol detection.
     *          Called during RC protocol auto-detection phase to identify CRSF frames.
     * 
     * @param[in] baudrate Initial baudrate to attempt in bps (typically 416666 or 420000)
     */
    void process_handshake(uint32_t baudrate) override;
    
    /**
     * @brief Periodic update for telemetry transmission and link monitoring
     * 
     * @details Called periodically (typically at 50-100Hz) to:
     *          - Send queued telemetry frames to TX module
     *          - Check link timeouts and update link status
     *          - Process pending VTX commands
     *          - Handle CRSF v3 baudrate changes
     * 
     * @note Must be called regularly to maintain bidirectional telemetry flow
     */
    void update(void) override;
    
#if HAL_CRSF_TELEM_ENABLED
    /**
     * @brief Initiate receiver bind mode
     * 
     * @details Sends CRSF_COMMAND_RX_BIND command to receiver to enter binding mode
     *          for pairing with a new transmitter. Binding typically lasts 60 seconds
     *          or until successful pairing occurs.
     * 
     * @note Requires HAL_CRSF_TELEM_ENABLED for bidirectional communication
     */
    void start_bind(void) override;
#endif
    
    /**
     * @brief Change UART baudrate for CRSF v3 protocol
     * 
     * @details CRSF v3 supports dynamic baudrate negotiation to reduce latency.
     *          This method reconfigures the UART to the new baudrate after receiving
     *          a CRSF_COMMAND_GENERAL_CRSF_SPEED_PROPOSAL command.
     * 
     * @param[in] baudrate New baudrate in bits per second
     * 
     * @return true if baudrate change successful, false if invalid baudrate or UART unavailable
     * 
     * @warning Baudrate change must be coordinated with transmitter to avoid losing connection
     */
    bool change_baud_rate(uint32_t baudrate);
    /**
     * @brief Get bootstrap baudrate for protocol detection
     * 
     * @details Returns the initial baudrate to use when detecting CRSF/ELRS protocol:
     *          - 416666 bps for standard Crossfire (CRSF_BAUDRATE)
     *          - 420000 bps for ExpressLRS if ELRS_420KBAUD option enabled
     *          
     *          The bootstrap baudrate is used during RC protocol auto-detection to identify
     *          CRSF frames on the serial port.
     * 
     * @return Bootstrap baudrate in bits per second (416666 or 420000)
     */
    uint32_t get_bootstrap_baud_rate() const {
#if AP_RC_CHANNEL_ENABLED
        return rc().option_is_enabled(RC_Channels::Option::ELRS_420KBAUD) ? ELRS_BAUDRATE : CRSF_BAUDRATE;
#else
        return CRSF_BAUDRATE;
#endif
    }

    /**
     * @brief Check if receiver is active (receiving frames)
     * 
     * @details Determines receiver activity by checking time since last received frame:
     *          - Receiver considered active if frame received within last 150ms (CRSF_RX_TIMEOUT)
     *          - Used to detect receiver power loss or baudrate mismatches
     *          - CRSF v3 sends link rate frames every 200ms even without RC input
     * 
     * @return true if receiver has sent frame within timeout period, false if timed out or never connected
     * 
     * @note Timeout value matches link statistics frame period for CRSF v3
     */
    bool is_rx_active() const override {
        // later versions of CRSFv3 will send link rate frames every 200ms
        // but only before an initial failsafe
        return _last_rx_frame_time_us != 0 && AP_HAL::micros() - _last_rx_frame_time_us < CRSF_RX_TIMEOUT;
    }

    /**
     * @brief Check if transmitter is active (pilot has control)
     * 
     * @details Determines transmitter activity by checking time since last RC channels frame:
     *          - Transmitter considered active if RC frame received within last 500ms (CRSF_TX_TIMEOUT)
     *          - Used to adjust telemetry data and trigger failsafe conditions
     *          - Timeout matches ArduCopter radio failsafe timeout for consistency
     * 
     * @return true if transmitter RC channels received within timeout, false if timed out or never connected
     * 
     * @note This timeout determines when RC failsafe is triggered in the flight controller
     */
    bool is_tx_active() const {
        // this is the same as the Copter failsafe timeout
        return _last_tx_frame_time_us != 0 && AP_HAL::micros() - _last_tx_frame_time_us < CRSF_TX_TIMEOUT;
    }

    /**
     * @brief Get singleton instance of CRSF protocol handler
     * 
     * @details Returns pointer to the singleton instance for access from telemetry subsystem.
     *          Only one CRSF instance exists per system.
     * 
     * @return Pointer to singleton instance, nullptr if not initialized
     */
    static AP_RCProtocol_CRSF* get_singleton() {
        return _singleton;
    }

    /**
     * @enum FrameType
     * @brief CRSF frame type identifiers for different message types
     * 
     * @details Frame types identify the payload format and purpose of each CRSF frame.
     *          The protocol supports multiple categories:
     *          
     *          Telemetry Frames (0x02-0x21): Flight data sent from FC to TX
     *          - GPS, battery, attitude, flight mode, link statistics
     *          
     *          RC Channel Frames (0x16-0x18): RC input data from TX to FC
     *          - Standard 16-channel packed format (11-bit resolution, 172-1811 range)
     *          - Subset channel format for reduced bandwidth
     *          - 11-bit format for maximum precision
     *          
     *          Extended Frames (0x28-0x96): Configuration and control
     *          - Parameter read/write for device configuration
     *          - Command frames for bind, VTX control, etc.
     *          
     *          Custom Frames (0x7F-0x80): ArduPilot-specific telemetry extensions
     */
    enum FrameType {
        CRSF_FRAMETYPE_GPS = 0x02,                      ///< GPS position, velocity, satellites (TX → FC telemetry)
        CRSF_FRAMETYPE_VARIO = 0x07,                    ///< Variometer data (vertical speed)
        CRSF_FRAMETYPE_BATTERY_SENSOR = 0x08,           ///< Battery voltage, current, capacity
        CRSF_FRAMETYPE_BARO_VARIO = 0x09,               ///< Barometric altitude and vertical speed
        CRSF_FRAMETYPE_HEARTBEAT = 0x0B,                ///< Heartbeat frame for connection monitoring
        CRSF_FRAMETYPE_VTX = 0x0F,                      ///< VTX configuration (frequency, power, pitmode)
        CRSF_FRAMETYPE_VTX_TELEM = 0x10,                ///< VTX telemetry status
        CRSF_FRAMETYPE_LINK_STATISTICS = 0x14,          ///< Link quality statistics (RSSI, LQ, SNR, TX power)
        CRSF_FRAMETYPE_RC_CHANNELS_PACKED = 0x16,       ///< Standard 16 RC channels, 11-bit packed (172-1811 range)
        CRSF_FRAMETYPE_SUBSET_RC_CHANNELS_PACKED = 0x17,///< Subset of channels with variable resolution
        CRSF_FRAMETYPE_RC_CHANNELS_PACKED_11BIT = 0x18, ///< Up to 24 channels, 11-bit resolution
        CRSF_FRAMETYPE_LINK_STATISTICS_RX = 0x1C,       ///< Receiver link statistics (RSSI, LQ, SNR)
        CRSF_FRAMETYPE_LINK_STATISTICS_TX = 0x1D,       ///< Transmitter link statistics with FPS
        CRSF_FRAMETYPE_ATTITUDE = 0x1E,                 ///< Vehicle attitude (roll, pitch, yaw)
        CRSF_FRAMETYPE_FLIGHT_MODE = 0x21,              ///< Current flight mode string
        // Extended Header Frames, range: 0x28 to 0x96
        CRSF_FRAMETYPE_PARAM_DEVICE_PING = 0x28,        ///< Device discovery ping
        CRSF_FRAMETYPE_PARAM_DEVICE_INFO = 0x29,        ///< Device information response
        CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY = 0x2B, ///< Parameter definition/metadata
        CRSF_FRAMETYPE_PARAMETER_READ = 0x2C,           ///< Read parameter value request
        CRSF_FRAMETYPE_PARAMETER_WRITE = 0x2D,          ///< Write parameter value command
        CRSF_FRAMETYPE_COMMAND = 0x32,                  ///< General command frame (bind, VTX control, etc.)
        // Custom Telemetry Frames 0x7F,0x80
        CRSF_FRAMETYPE_AP_CUSTOM_TELEM_LEGACY = 0x7F,   ///< ArduPilot custom telemetry (fw < 4.06)
        CRSF_FRAMETYPE_AP_CUSTOM_TELEM = 0x80,          ///< ArduPilot custom telemetry (fw >= 4.06, reserved by TBS)
    };

    /**
     * @enum CommandID
     * @brief Command destination identifiers for CRSF_FRAMETYPE_COMMAND frames
     * 
     * @details Specifies the target device/subsystem for command frames.
     *          Each command ID has associated sub-commands defined in separate enums.
     */
    enum CommandID {
        CRSF_COMMAND_FC = 0x01,         ///< Flight controller commands (disarm, channel scaling)
        CRSF_COMMAND_BLUETOOTH = 0x03,  ///< Bluetooth module commands (reset, enable)
        CRSF_COMMAND_OSD = 0x05,        ///< OSD commands (send display data)
        CRSF_COMMAND_VTX = 0x08,        ///< VTX commands (channel, power, pitmode)
        CRSF_COMMAND_LED = 0x09,        ///< LED commands (color, patterns)
        CRSF_COMMAND_GENERAL = 0x0A,    ///< General commands (baudrate negotiation, firmware update)
        CRSF_COMMAND_RX = 0x10,         ///< Receiver commands (bind, cancel bind)
    };

    /**
     * @enum CommandFC
     * @brief Flight controller specific commands (CRSF_COMMAND_FC)
     */
    enum CommandFC {
        CRSF_COMMAND_FC_DISARM = 0x01,      ///< Emergency disarm command from transmitter
        CRSF_COMMAND_SCALE_CHANNEL = 0x02,  ///< Scale RC channel output range
    };

    /**
     * @enum CommandBluetooth
     * @brief Bluetooth module commands (CRSF_COMMAND_BLUETOOTH)
     */
    enum CommandBluetooth {
        CRSF_COMMAND_BLUETOOTH_RESET = 0x01,  ///< Reset Bluetooth module
        CRSF_COMMAND_BLUETOOTH_ENABLE = 0x02, ///< Enable/disable Bluetooth
        CRSF_COMMAND_BLUETOOTH_ECHO = 0x64,   ///< Bluetooth echo test
    };

    /**
     * @enum CommandOSD
     * @brief On-screen display commands (CRSF_COMMAND_OSD)
     */
    enum CommandOSD {
        CRSF_COMMAND_OSD_SEND = 0x01,  ///< Send OSD display data
    };

    /**
     * @enum CommandVTX
     * @brief Video transmitter control commands (CRSF_COMMAND_VTX)
     * 
     * @details Controls VTX frequency, power, and pitmode for FPV video transmission.
     *          Allows transmitter to configure VTX without dedicated VTX menu.
     */
    enum CommandVTX {
        CRSF_COMMAND_VTX_CHANNEL = 0x01,       ///< Set VTX channel (band + channel number)
        CRSF_COMMAND_VTX_FREQ = 0x02,          ///< Set VTX frequency directly in MHz
        CRSF_COMMAND_VTX_POWER = 0x03,         ///< Set VTX power level index
        CRSF_COMMAND_VTX_PITMODE = 0x04,       ///< Enable/disable VTX pitmode (low power)
        CRSF_COMMAND_VTX_PITMODE_POWERUP = 0x05, ///< Pitmode power-up behavior
        CRSF_COMMAND_VTX_POWER_DBM = 0x08,     ///< Set VTX power in dBm
    };

    /**
     * @enum CommandLED
     * @brief LED control commands (CRSF_COMMAND_LED)
     */
    enum CommandLED {
        CRSF_COMMAND_LED_SET_DEFAULT = 0x01,  ///< Restore default LED behavior
        CRSF_COMMAND_LED_COLOR = 0x02,        ///< Set LED color (RGB)
        CRSF_COMMAND_LED_PULSE = 0x03,        ///< LED pulse pattern
        CRSF_COMMAND_LED_BLINK = 0x04,        ///< LED blink pattern
        CRSF_COMMAND_LED_SHIFT = 0x05,        ///< LED color shift animation
    };

    /**
     * @enum CommandRX
     * @brief Receiver control commands (CRSF_COMMAND_RX)
     */
    enum CommandRX {
        CRSF_COMMAND_RX_BIND = 0x01,         ///< Enter bind mode (pair with new transmitter)
        CRSF_COMMAND_RX_CANCEL_BIND = 0x02,  ///< Cancel bind mode
        CRSF_COMMAND_RX_SET_BIND_ID = 0x03,  ///< Set receiver bind ID
    };

    /**
     * @enum CommandGeneral
     * @brief General system commands (CRSF_COMMAND_GENERAL)
     * 
     * @details Includes firmware update, device discovery, and CRSF v3 baudrate negotiation.
     */
    enum CommandGeneral {
        CRSF_COMMAND_GENERAL_CHILD_DEVICE_REQUEST = 0x04,       ///< Request child device enumeration
        CRSF_COMMAND_GENERAL_CHILD_DEVICE_FRAME = 0x05,         ///< Child device data frame
        CRSF_COMMAND_GENERAL_FIRMWARE_UPDATE_BOOTLOADER = 0x0A, ///< Enter bootloader for firmware update
        CRSF_COMMAND_GENERAL_FIRMWARE_UPDATE_ERASE = 0x0B,      ///< Erase flash for firmware update
        CRSF_COMMAND_GENERAL_WRITE_SERIAL_NUMBER = 0x13,        ///< Write device serial number
        CRSF_COMMAND_GENERAL_USER_ID = 0x15,                    ///< Set user ID
        CRSF_COMMAND_GENERAL_SOFTWARE_PRODUCT_KEY = 0x60,       ///< Software product key validation
        CRSF_COMMAND_GENERAL_CRSF_SPEED_PROPOSAL = 0x70,        ///< CRSF v3 baudrate proposal
        CRSF_COMMAND_GENERAL_CRSF_SPEED_RESPONSE = 0x71,        ///< CRSF v3 baudrate response
    };

    /**
     * @enum CustomTelemSubTypeID
     * @brief ArduPilot custom telemetry sub-types (CRSF_FRAMETYPE_AP_CUSTOM_TELEM)
     * 
     * @details ArduPilot-specific telemetry extensions for MAVLink passthrough and status text.
     *          Frame type 0x80 reserved by Team Black Sheep for ArduPilot use.
     */
    enum CustomTelemSubTypeID : uint8_t {
        CRSF_AP_CUSTOM_TELEM_SINGLE_PACKET_PASSTHROUGH = 0xF0, ///< Single MAVLink packet passthrough
        CRSF_AP_CUSTOM_TELEM_STATUS_TEXT = 0xF1,               ///< Status text message
        CRSF_AP_CUSTOM_TELEM_MULTI_PACKET_PASSTHROUGH = 0xF2,  ///< Multi-packet MAVLink passthrough
    };

    /**
     * @enum DeviceAddress
     * @brief CRSF device addresses for frame routing
     * 
     * @details Device addresses appear in frame header to identify sender and receiver.
     *          Enables multi-device communication on shared CRSF bus (FC, VTX, RX, TX, peripherals).
     */
    enum DeviceAddress {
        CRSF_ADDRESS_BROADCAST = 0x00,              ///< Broadcast to all devices
        CRSF_ADDRESS_USB = 0x10,                    ///< USB interface
        CRSF_ADDRESS_TBS_CORE_PNP_PRO = 0x80,       ///< TBS Core PNP Pro module
        CRSF_ADDRESS_RESERVED1 = 0x8A,              ///< Reserved address
        CRSF_ADDRESS_PNP_PRO_CURRENT_SENSOR = 0xC0, ///< Current sensor peripheral
        CRSF_ADDRESS_PNP_PRO_GPS = 0xC2,            ///< GPS peripheral
        CRSF_ADDRESS_TBS_BLACKBOX = 0xC4,           ///< Blackbox logger
        CRSF_ADDRESS_FLIGHT_CONTROLLER = 0xC8,      ///< Flight controller (ArduPilot)
        CRSF_ADDRESS_RESERVED2 = 0xCA,              ///< Reserved address
        CRSF_ADDRESS_RACE_TAG = 0xCC,               ///< Race tag transponder
        CRSF_ADDRESS_VTX = 0xCE,                    ///< Video transmitter
        CRSF_ADDRESS_RADIO_TRANSMITTER = 0xEA,      ///< Radio transmitter (pilot's controller)
        CRSF_ADDRESS_CRSF_RECEIVER = 0xEC,          ///< CRSF receiver
        CRSF_ADDRESS_CRSF_TRANSMITTER = 0xEE        ///< CRSF transmitter module
    };

    /**
     * @enum ExtendedFrameOffset
     * @brief Byte offsets for extended frame format fields
     * 
     * @details Extended frames (0x28-0x96) include source/destination addressing:
     *          [Device Address][Length][Type][Destination][Origin][Payload...][CRC-8]
     */
    enum ExtendedFrameOffset {
        CRSF_EXTENDED_FRAME_LENGTH_OFFSET = 1,      ///< Frame length field offset
        CRSF_EXTENDED_FRAME_TYPE_OFFSET = 2,        ///< Frame type field offset
        CRSF_EXTENDED_FRAME_DESTINATION_OFFSET = 3, ///< Destination address offset
        CRSF_EXTENDED_FRAME_ORIGIN_OFFSET = 4,      ///< Origin address offset
        CRSF_EXTENDED_FRAME_PAYLOAD_OFFSET = 5,     ///< Payload start offset
    };

    /**
     * @struct Frame
     * @brief Generic CRSF frame structure
     * 
     * @details Standard CRSF frame format:
     *          [Device Address (1 byte)][Length (1 byte)][Type (1 byte)][Payload (0-60 bytes)][CRC-8 (1 byte)]
     *          
     *          - device_address: Sender device address (see DeviceAddress enum)
     *          - length: Frame length from type to CRC (inclusive), range 2-63
     *          - type: Frame type identifier (see FrameType enum)
     *          - payload: Frame payload, length determined by (length - 2)
     *          - CRC-8: DVB-S2 polynomial over length, type, and payload bytes
     */
    struct Frame {
        uint8_t device_address;                     ///< Frame source device address
        uint8_t length;                             ///< Frame length (type to CRC, inclusive)
        uint8_t type;                               ///< Frame type (FrameType enum)
        uint8_t payload[CRSF_FRAME_PAYLOAD_MAX - 1]; ///< Frame payload (max 60 bytes, type excluded)
    } PACKED;

    /**
     * @struct LinkStatisticsFrame
     * @brief Link statistics payload (CRSF_FRAMETYPE_LINK_STATISTICS)
     * 
     * @details Provides comprehensive RF link quality metrics for both uplink (TX→RX) and downlink (RX→TX):
     *          - RSSI: Received signal strength in -dBm (negated, so 50 = -50dBm)
     *          - Link quality: Packet success rate percentage (0-100%)
     *          - SNR: Signal-to-noise ratio in dB
     *          - TX power: Transmit power level index
     *          
     *          Used by OSD to display real-time link quality and for failsafe detection.
     */
    struct LinkStatisticsFrame {
        uint8_t uplink_rssi_ant1;   ///< Uplink RSSI antenna 1 (dBm * -1), range 0-255 for 0 to -255 dBm
        uint8_t uplink_rssi_ant2;   ///< Uplink RSSI antenna 2 (dBm * -1)
        uint8_t uplink_status;      ///< Uplink packet success rate / Link quality (%), range 0-100
        int8_t uplink_snr;          ///< Uplink signal-to-noise ratio (dB), range -128 to 127
        uint8_t active_antenna;     ///< Active diversity antenna (0 = ant1, 1 = ant2)
        uint8_t rf_mode;            ///< RF packet rate mode (see RFMode enum)
        uint8_t uplink_tx_power;    ///< TX power level index (see tx_powers array)
        uint8_t downlink_rssi;      ///< Downlink RSSI (dBm * -1)
        uint8_t downlink_status;    ///< Downlink packet success rate (%)
        int8_t downlink_dnr;        ///< Downlink SNR (dB)
    } PACKED;

    /**
     * @struct LinkStatisticsRXFrame
     * @brief Receiver link statistics (CRSF_FRAMETYPE_LINK_STATISTICS_RX)
     * 
     * @details Simplified link statistics from receiver perspective (CRSF v3).
     *          Provides essential link quality metrics with reduced bandwidth.
     */
    struct LinkStatisticsRXFrame {
        uint8_t rssi_db;        ///< RSSI (dBm * -1), range 0-255
        uint8_t rssi_percent;   ///< RSSI percentage (0-100%)
        uint8_t link_quality;   ///< Packet success rate (0-100%)
        int8_t snr;             ///< Signal-to-noise ratio (dB)
        uint8_t rf_power_db;    ///< RF power (dBm)
    } PACKED;

    /**
     * @struct LinkStatisticsTXFrame
     * @brief Transmitter link statistics (CRSF_FRAMETYPE_LINK_STATISTICS_TX)
     * 
     * @details Transmitter link statistics with RF frame rate information.
     *          Adds FPS metric compared to RX statistics.
     */
    struct LinkStatisticsTXFrame {
        uint8_t rssi_db;        ///< RSSI (dBm * -1)
        uint8_t rssi_percent;   ///< RSSI percentage (0-100%)
        uint8_t link_quality;   ///< Packet success rate (0-100%)
        int8_t snr;             ///< Signal-to-noise ratio (dB)
        uint8_t rf_power_db;    ///< RF power (dBm)
        uint8_t fps;            ///< RF packet rate (fps / 10), so 150 = 15.0 fps
    } PACKED;

    /**
     * @struct SubsetChannelsFrame
     * @brief Subset RC channels frame payload (CRSF_FRAMETYPE_SUBSET_RC_CHANNELS_PACKED)
     * 
     * @details Reduced bandwidth channel encoding that transmits only a subset of channels
     *          with variable resolution (10-13 bits). Allows higher packet rates by sending
     *          fewer channels per frame.
     *          
     *          Configuration byte bitfields:
     *          - starting_channel: First channel number in this frame (0-23)
     *          - res_configuration: Channel resolution (0=10-bit, 1=11-bit, 2=12-bit, 3=13-bit)
     *          - digital_switch_flag: Whether frame includes digital switch channel
     *          
     * @warning Only supported on little-endian architectures
     */
    struct SubsetChannelsFrame {
#if __BYTE_ORDER != __LITTLE_ENDIAN
#error "Only supported on little-endian architectures"
#endif
        uint8_t starting_channel:5;     ///< First channel number (0-31)
        uint8_t res_configuration:2;    ///< Resolution config (0=10bit, 1=11bit, 2=12bit, 3=13bit)
        uint8_t digital_switch_flag:1;  ///< Digital switch channel present flag
        uint8_t channels[CRSF_FRAME_PAYLOAD_MAX - 2]; ///< Packed channel data
        // uint16_t channel[]:res;      // Variable number of channels with variable resolution
        // uint16_t digital_switch_channel[]:10; // Optional 10-bit digital switch channel
    } PACKED;

    /**
     * @enum ProtocolType
     * @brief CRSF protocol variant identification
     * 
     * @details Distinguishes between different protocol implementations:
     *          - CRSF: Standard TBS Crossfire protocol
     *          - TRACER: TBS Tracer variant (long-range)
     *          - ELRS: ExpressLRS open-source protocol (higher packet rates)
     */
    enum class ProtocolType {
        PROTOCOL_CRSF,      ///< TBS Crossfire standard protocol
        PROTOCOL_TRACER,    ///< TBS Tracer protocol variant
        PROTOCOL_ELRS       ///< ExpressLRS protocol variant
    };

    /**
     * @enum RFMode
     * @brief RF packet rate modes for CRSF and ExpressLRS
     * 
     * @details RF mode determines the over-the-air packet rate, affecting both latency and range:
     *          - Lower rates (4Hz, 25Hz, 50Hz): Maximum range, higher latency
     *          - Higher rates (250Hz, 500Hz, 1000Hz): Lower latency, reduced range
     *          
     *          CRSF modes (0-3): 4Hz, 50Hz, 150Hz, 250Hz
     *          ELRS modes (4+): Extended rate options including 333Hz, 500Hz, 1000Hz
     *          
     *          Mode suffix meanings:
     *          - Standard: Normal telemetry rate
     *          - FULL: Full telemetry ratio (more telemetry bandwidth)
     *          - D: Dynamic power mode
     *          - F: FLRC modulation (reduced range, lower latency)
     *          
     * @note ExpressLRS RF mode reference: https://www.expresslrs.org/info/signal-health/#rf-mode-indexes-rfmd
     */
    enum RFMode {
        CRSF_RF_MODE_4HZ = 0,       ///< CRSF 4 Hz mode (maximum range)
        CRSF_RF_MODE_50HZ,          ///< CRSF 50 Hz mode
        CRSF_RF_MODE_150HZ,         ///< CRSF 150 Hz mode
        CRSF_RF_MODE_250HZ,         ///< CRSF 250 Hz mode
        CRSF_RF_MAX_MODES = 4,      ///< Number of CRSF RF modes
        ELRS_RF_MODE_4HZ = 4,       ///< ELRS 4 Hz mode
        ELRS_RF_MODE_25HZ,          ///< ELRS 25 Hz mode
        ELRS_RF_MODE_50HZ,          ///< ELRS 50 Hz mode
        ELRS_RF_MODE_100HZ,         ///< ELRS 100 Hz mode
        ELRS_RF_MODE_100HZ_FULL,    ///< ELRS 100 Hz full telemetry
        ELRS_RF_MODE_150HZ,         ///< ELRS 150 Hz mode
        ELRS_RF_MODE_200HZ,         ///< ELRS 200 Hz mode
        ELRS_RF_MODE_250HZ,         ///< ELRS 250 Hz mode
        ELRS_RF_MODE_333HZ_FULL,    ///< ELRS 333 Hz full telemetry
        ELRS_RF_MODE_500HZ,         ///< ELRS 500 Hz mode
        ELRS_RF_MODE_D250HZ,        ///< ELRS 250 Hz dynamic power
        ELRS_RF_MODE_D500HZ,        ///< ELRS 500 Hz dynamic power
        ELRS_RF_MODE_F500HZ,        ///< ELRS 500 Hz FLRC
        ELRS_RF_MODE_F1000HZ,       ///< ELRS 1000 Hz FLRC (minimum latency)
        ELRS_RF_MODE_D50HZ,         ///< ELRS 50 Hz dynamic power
        RF_MODE_MAX_MODES,          ///< Total number of RF modes
        RF_MODE_UNKNOWN,            ///< Unknown/unidentified RF mode
    };

#if AP_OSD_LINK_STATS_EXTENSIONS_ENABLED
    /**
     * @brief TX power level lookup table (milliwatts)
     * 
     * @details Maps TX power level index to actual power in milliwatts.
     *          Valid for both Crossfire and ExpressLRS systems.
     *          Index comes from link statistics frame uplink_tx_power field.
     *          
     *          Power levels: 0mW, 10mW, 25mW, 100mW, 500mW, 1000mW, 2000mW, 250mW, 50mW
     */
    static constexpr uint16_t tx_powers[] = { 0, 10, 25, 100, 500, 1000, 2000, 250, 50 };    
#endif

    /**
     * @struct LinkStatus
     * @brief Aggregated link quality status for OSD and telemetry
     * 
     * @details Consolidated link statistics from latest link statistics frame.
     *          Updated by process_link_stats_frame() and accessed by telemetry subsystem.
     *          
     *          Values of -1 indicate uninitialized/unavailable metrics.
     *          
     * @note Marked volatile for safe access from telemetry thread
     */
    struct LinkStatus {
        int16_t rssi = -1;          ///< RSSI value for display (scaled), -1 if unavailable
        int16_t link_quality = -1;  ///< Link quality percentage (0-100%), -1 if unavailable
        uint8_t rf_mode;            ///< Current RF packet rate mode (RFMode enum)
#if AP_OSD_LINK_STATS_EXTENSIONS_ENABLED
        // Extended statistics for advanced OSD panels
        int16_t tx_power = -1;      ///< TX power in mW, -1 if unavailable
        int8_t rssi_dbm = -1;       ///< RSSI in dBm (-128 to 0), -1 if unavailable
        int8_t snr = INT8_MIN;      ///< Signal-to-noise ratio in dB, INT8_MIN if unavailable
        int8_t active_antenna = -1; ///< Active diversity antenna (0 or 1), -1 if unavailable
#endif
    };


    /**
     * @brief Get current link status for OSD display and telemetry
     * 
     * @details Returns reference to aggregated link quality metrics updated from
     *          CRSF link statistics frames. Safe to call from telemetry thread as
     *          both threads run within AP_RCProtocol context.
     * 
     * @return const volatile reference to current link status
     * 
     * @note Volatile qualifier allows telemetry thread to safely read link status
     */
    const volatile LinkStatus& get_link_status() const {
        return _link_status;
    }

    /**
     * @brief Get RF packet rate for given protocol and RF mode
     * 
     * @details Converts RF mode enum to actual packet rate in Hz for display.
     *          Different protocols (CRSF, ELRS) have different mode mappings.
     * 
     * @param[in] protocol Protocol type (CRSF, TRACER, or ELRS)
     * 
     * @return RF packet rate in Hz (e.g., 150 for 150Hz mode), 0 if unknown mode
     */
    uint16_t get_link_rate(ProtocolType protocol) const;

    /**
     * @brief Get protocol name string for display
     * 
     * @details Returns human-readable protocol name for OSD and logging.
     * 
     * @param[in] protocol Protocol type enum value
     * 
     * @return Protocol name string ("CRSF", "Tracer", or "ELRS")
     */
    const char* get_protocol_string(ProtocolType protocol) const;

private:
    struct Frame _frame;
    uint8_t *_frame_bytes = (uint8_t*)&_frame;
    struct Frame _telemetry_frame;
    uint8_t _frame_ofs;

    const uint8_t MAX_CHANNELS = MIN((uint8_t)CRSF_MAX_CHANNELS, (uint8_t)MAX_RCIN_CHANNELS);

    static AP_RCProtocol_CRSF* _singleton;

    void _process_byte(uint8_t byte);
    bool check_frame(uint32_t timestamp_us);
    void skip_to_next_frame(uint32_t timestamp_us);
    bool decode_crsf_packet();
    bool process_telemetry(bool check_constraint = true);
    void process_link_stats_frame(const void* data);
    void process_link_stats_rx_frame(const void* data);
    void process_link_stats_tx_frame(const void* data);
    // crsf v3 decoding
    void decode_variable_bit_channels(const uint8_t* data, uint8_t frame_length, uint8_t nchannels, uint16_t *values);

    void write_frame(Frame* frame);
    void start_uart();
    AP_HAL::UARTDriver* get_current_UART() { return (_uart ? _uart : get_available_UART()); }

    uint16_t _channels[CRSF_MAX_CHANNELS];    /* buffer for extracted RC channel data as pulsewidth in microseconds */

    uint32_t _last_frame_time_us;
    uint32_t _last_tx_frame_time_us;
    uint32_t _last_uart_start_time_ms;
    uint32_t _last_rx_frame_time_us;
    uint32_t _start_frame_time_us;
    bool telem_available;
    uint32_t _new_baud_rate;
    bool _crsf_v3_active;

    bool _use_lq_for_rssi;
    int16_t derive_scaled_lq_value(uint8_t uplink_lq);

    volatile struct LinkStatus _link_status;

    static const uint16_t RF_MODE_RATES[RFMode::RF_MODE_MAX_MODES];

    AP_HAL::UARTDriver *_uart;
};

/**
 * @namespace AP
 * @brief ArduPilot singleton accessor namespace
 */
namespace AP {
    /**
     * @brief Get CRSF protocol singleton instance
     * 
     * @details Provides global access to CRSF protocol handler for telemetry subsystem.
     *          Returns nullptr if CRSF protocol is not active.
     * 
     * @return Pointer to CRSF protocol instance, nullptr if not initialized
     */
    AP_RCProtocol_CRSF* crsf();
};

#endif  // AP_RCPROTOCOL_CRSF_ENABLED
