/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/**
 * @file AP_BLHeli.h
 * @brief BLHeli ESC pass-through protocol implementation for configuration and firmware updates
 * 
 * @details This file implements the MSP (MultiWii Serial Protocol) and BLHeli 4-way
 *          pass-through protocols for direct ESC communication. These protocols enable:
 *          - ESC calibration without removing propellers (with motor disable)
 *          - ESC firmware flashing via BLHeliSuite/Configurator
 *          - ESC telemetry collection (RPM, voltage, current, temperature)
 *          - Configuration of ESC parameters (timing, braking, direction)
 * 
 *          The implementation provides MAVLink-based pass-through from ground control
 *          stations to ESC bootloaders, allowing configuration and firmware updates
 *          without direct serial connection to individual ESCs.
 * 
 *          Protocol Support:
 *          - MSP: Command protocol for ESC detection and mode selection
 *          - BLHeli 4-way: Bootloader protocol for reading/writing ESC flash memory
 *          - Bidirectional DShot: High-speed ESC telemetry on DShot output signal
 * 
 * @note Implementation based on betaflight reference implementation
 * @warning ALWAYS REMOVE PROPELLERS before ESC configuration or firmware flashing
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>

#define HAVE_AP_BLHELI_SUPPORT HAL_SUPPORT_RCOUT_SERIAL

#if HAL_SUPPORT_RCOUT_SERIAL

#include <AP_ESC_Telem/AP_ESC_Telem_Backend.h>

#include <AP_Param/AP_Param.h>
#include <Filter/LowPassFilter.h>
#include <AP_MSP/msp_protocol.h>
#include "blheli_4way_protocol.h"

#define AP_BLHELI_MAX_ESCS 8

/**
 * @class AP_BLHeli
 * @brief BLHeli ESC pass-through support for configuration and firmware flashing
 * 
 * @details This class provides pass-through protocol support for BLHeli ESCs, enabling
 *          direct communication with ESC bootloaders for configuration and firmware updates.
 *          Derives from AP_ESC_Telem_Backend to integrate ESC telemetry into the autopilot's
 *          telemetry system.
 * 
 *          Architecture:
 *          - Implements MSP protocol state machine for command processing
 *          - Implements BLHeli 4-way protocol for ESC flash memory access
 *          - Manages UART locking to prevent protocol corruption during pass-through
 *          - Collects and processes ESC telemetry (RPM, voltage, current, temperature)
 *          - Supports bidirectional DShot for high-speed telemetry on motor control lines
 * 
 *          Protocol Handling:
 *          - MSP: Processes commands from BLHeliSuite/Configurator via MAVLink
 *          - BLHeli 4-way: Translates flash read/write/erase commands to ESC bootloader
 *          - Telemetry: Converts eRPM to RPM using motor pole count, logs data
 * 
 *          Safety Features:
 *          - Motor disable during configuration to prevent propeller rotation
 *          - Exclusive UART locking prevents protocol corruption
 *          - Timeout mechanisms for protocol state machines
 *          - Connection tracking per ESC to manage multi-ESC operations
 * 
 * @note Integrates with BLHeliSuite and BLHeli Configurator ground control tools
 * @warning ESC configuration and firmware flashing REQUIRES PROPELLER REMOVAL
 * @see AP_ESC_Telem_Backend for telemetry integration
 */
class AP_BLHeli : public AP_ESC_Telem_Backend {

public:
    AP_BLHeli();
    
    /**
     * @brief Periodic update function for BLHeli pass-through and telemetry
     * 
     * @details Called regularly from main loop to:
     *          - Process ESC telemetry data and update telemetry backend
     *          - Check for protocol timeout and reset state machines if needed
     *          - Manage telemetry request scheduling based on configured rate
     *          - Handle connection timeout and restore normal motor operation
     * 
     * @note Should be called at main loop rate (typically 50-400 Hz)
     */
    void update(void);
    
    /**
     * @brief Initialize BLHeli pass-through support with motor configuration
     * 
     * @param[in] motor_mask Bitmask of motors to enable for BLHeli protocol
     * @param[in] mode RCOutput mode (e.g., PWM_TYPE_DSHOT300 for DShot)
     * 
     * @details Sets up the BLHeli pass-through system:
     *          - Configures UART for pass-through communication
     *          - Maps motor numbers to physical output channels
     *          - Initializes protocol state machines
     *          - Configures output mode for motor control
     *          - Sets up telemetry collection if bidirectional DShot enabled
     * 
     * @note Must be called before any pass-through operations
     */
    void init(uint32_t motor_mask, AP_HAL::RCOutput::output_mode mode);
    
    /**
     * @brief Update ESC telemetry from bidirectional DShot or serial telemetry
     * 
     * @details Collects telemetry data from ESCs and processes:
     *          - eRPM (electrical RPM) conversion to mechanical RPM using motor_poles parameter
     *          - Voltage, current, and temperature measurements
     *          - Frame error counters for DShot communication quality
     *          - Logs telemetry data at configured rate
     * 
     * @note Called by update() at telemetry request rate
     * @see get_motor_poles() for RPM conversion factor
     */
    void update_telemetry(void);
    
    /**
     * @brief Process incoming byte for MSP or BLHeli 4-way protocol
     * 
     * @param[in] b Input byte from MAVLink pass-through or serial port
     * 
     * @return true if byte was consumed by protocol handler, false otherwise
     * 
     * @details Implements protocol state machine switching:
     *          - Auto-detects MSP vs BLHeli 4-way protocol from byte patterns
     *          - Routes bytes to appropriate protocol handler (msp_process_byte or blheli_4way_process_byte)
     *          - Maintains protocol state across multiple packet bytes
     *          - Handles protocol synchronization and error recovery
     * 
     * @note Called for each byte received from ground control station
     */
    bool process_input(uint8_t b);

    static const struct AP_Param::GroupInfo var_info[];

    /**
     * @brief Check if bidirectional DShot is enabled for a specific ESC
     * 
     * @param[in] esc_index ESC index (0 to AP_BLHELI_MAX_ESCS-1)
     * 
     * @return true if bidirectional DShot enabled for this ESC, false otherwise
     * 
     * @details Bidirectional DShot sends telemetry data (eRPM) back from ESC to
     *          autopilot on the same signal wire used for motor control commands.
     *          This eliminates need for separate telemetry wiring.
     * 
     * @note Uses motor_map to convert ESC index to physical channel number
     */
    bool has_bidir_dshot(uint8_t esc_index) const {
        return channel_bidir_dshot_mask.get() & (1U << motor_map[esc_index]);
    }

    /**
     * @brief Get bitmask of channels with bidirectional DShot enabled
     * 
     * @return 32-bit bitmask where bit N indicates channel N has bidir DShot
     * 
     * @details Each bit position corresponds to a physical output channel.
     *          Used for configuring DShot output mode and telemetry collection.
     */
    uint32_t get_bidir_dshot_mask() const { return channel_bidir_dshot_mask.get(); }
    
    /**
     * @brief Get motor pole count for RPM calculation
     * 
     * @return Number of motor poles (typically 12-14 for multicopter motors)
     * 
     * @details ESCs report eRPM (electrical RPM = magnetic pole pairs per minute).
     *          Mechanical RPM = eRPM / (motor_poles / 2)
     *          Example: 14-pole motor at 10000 eRPM = 10000 / 7 = 1428 mechanical RPM
     * 
     * @note This parameter must match actual motor specification for accurate RPM
     */
    uint8_t get_motor_poles() const { return motor_poles.get(); }
    
    /**
     * @brief Get configured telemetry request rate
     * 
     * @return Telemetry request rate in Hz
     * 
     * @details Controls how frequently ESC telemetry is requested and logged.
     *          Higher rates provide more data but increase CPU and bandwidth usage.
     *          Typical values: 10 Hz for logging, 100+ Hz for dynamic notch filtering.
     */
    uint16_t get_telemetry_rate() const { return telem_rate.get(); }

    /**
     * @brief Get singleton instance of AP_BLHeli
     * 
     * @return Pointer to singleton instance, nullptr if not created
     * 
     * @details Provides global access to BLHeli pass-through functionality.
     *          Used by MAVLink handlers and ESC telemetry consumers.
     */
    static AP_BLHeli *get_singleton(void) {
        return _singleton;
    }
    
private:
    static AP_BLHeli *_singleton;     ///< Singleton instance pointer
    
    // Configuration parameters (stored in EEPROM via AP_Param)
    AP_Int32 channel_mask;            ///< Bitmask of channels enabled for BLHeli pass-through
    AP_Int32 channel_reversible_mask; ///< Bitmask of channels with reversible ESCs (3D mode)
    AP_Int32 channel_reversed_mask;   ///< Bitmask of channels with reversed motor direction
    AP_Int8 channel_auto;             ///< Enable automatic channel detection
    AP_Int8 run_test;                 ///< Run connection test on specified channel
    AP_Int16 timeout_sec;             ///< Pass-through timeout in seconds (0 = no timeout)
    AP_Int16 telem_rate;              ///< ESC telemetry request rate in Hz
    AP_Int8 debug_level;              ///< Debug output verbosity level (0-4)
    AP_Int8 output_type;              ///< Motor PWM output type (see motorPwmProtocol enum)
    AP_Int8 control_port;             ///< Serial port number for pass-through control (-1 = auto)
    AP_Int8 motor_poles;              ///< Motor pole count for eRPM to RPM conversion (typically 12-14)
    AP_Int32 channel_bidir_dshot_mask; ///< Bitmask of channels with bidirectional DShot enabled
    
    /**
     * @enum mspState
     * @brief MSP protocol state machine states for packet parsing
     * 
     * @details MSP packets have format: $M<direction><size><cmd><data><checksum>
     *          State machine advances through header bytes to validate and parse commands.
     */
    enum mspState {
        MSP_IDLE=0,              ///< Waiting for packet start character '$'
        MSP_HEADER_START,        ///< Received '$', expecting 'M'
        MSP_HEADER_M,            ///< Received 'M', expecting direction '<' or '>'
        MSP_HEADER_ARROW,        ///< Received direction, expecting size byte
        MSP_HEADER_SIZE,         ///< Received size, expecting command byte
        MSP_HEADER_CMD,          ///< Received command, expecting data bytes
        MSP_COMMAND_RECEIVED     ///< Complete command received, ready to process
    };

    /**
     * @enum mspPacketType
     * @brief MSP packet direction indicator
     */
    enum mspPacketType {
        MSP_PACKET_COMMAND,      ///< Command from ground station to autopilot (direction '<')
        MSP_PACKET_REPLY         ///< Reply from autopilot to ground station (direction '>')
    };

    /**
     * @enum escProtocol
     * @brief ESC bootloader protocol types supported by pass-through
     * 
     * @details Different ESC firmware families use different bootloader protocols
     *          for flash memory access. Protocol selection determines communication
     *          timing, command format, and memory layout.
     */
    enum escProtocol {
        PROTOCOL_SIMONK = 0,     ///< SimonK bootloader protocol (legacy open-source ESC firmware)
        PROTOCOL_BLHELI = 1,     ///< BLHeli bootloader protocol (most common)
        PROTOCOL_KISS = 2,       ///< KISS ESC bootloader protocol (single ESC)
        PROTOCOL_KISSALL = 3,    ///< KISS ESC bootloader protocol (all ESCs simultaneously)
        PROTOCOL_CASTLE = 4,     ///< Castle Creations ESC protocol
        PROTOCOL_MAX = 5,        ///< Maximum valid protocol value
        PROTOCOL_NONE = 0xfe,    ///< No protocol selected
        PROTOCOL_4WAY = 0xff     ///< BLHeli 4-way protocol mode (current protocol)
    };

    /**
     * @enum motorPwmProtocol
     * @brief Motor PWM output protocol types
     * 
     * @details Defines the ESC communication protocol used for motor control.
     *          DShot protocols are digital and immune to timing jitter, while
     *          analog protocols (PWM, OneShot) are susceptible to electrical noise.
     */
    enum motorPwmProtocol {
        PWM_TYPE_STANDARD = 0,   ///< Standard PWM (1000-2000µs pulses at 50Hz)
        PWM_TYPE_ONESHOT125,     ///< OneShot125 (125-250µs pulses, 8x faster than PWM)
        PWM_TYPE_ONESHOT42,      ///< OneShot42 (42-84µs pulses, 24x faster than PWM)
        PWM_TYPE_MULTISHOT,      ///< Multishot (5-25µs pulses, high-speed analog)
        PWM_TYPE_BRUSHED,        ///< Brushed motor control (DC motor driver)
        PWM_TYPE_DSHOT150,       ///< DShot150 (150kbit/s digital protocol)
        PWM_TYPE_DSHOT300,       ///< DShot300 (300kbit/s digital protocol, most common)
        PWM_TYPE_DSHOT600,       ///< DShot600 (600kbit/s digital protocol)
        PWM_TYPE_DSHOT1200,      ///< DShot1200 (1200kbit/s digital protocol, requires fast MCU)
        PWM_TYPE_PROSHOT1000,    ///< ProShot1000 (1000kbit/s digital protocol)
    };

    /**
     * @enum MSPFeatures
     * @brief MSP feature flags bitmask for capability reporting
     * 
     * @details Used in MSP_FC_VARIANT and MSP_FC_VERSION responses to indicate
     *          which features are enabled in the flight controller. BLHeliSuite
     *          uses these flags to adjust UI and available commands.
     * 
     * @note Derived from Betaflight/Cleanflight MSP feature definitions
     */
    enum MSPFeatures {
        FEATURE_RX_PPM = 1 << 0,              ///< PPM receiver input
        FEATURE_INFLIGHT_ACC_CAL = 1 << 2,    ///< In-flight accelerometer calibration
        FEATURE_RX_SERIAL = 1 << 3,           ///< Serial receiver (SBUS, etc.)
        FEATURE_MOTOR_STOP = 1 << 4,          ///< Motor stop when disarmed
        FEATURE_SERVO_TILT = 1 << 5,          ///< Servo gimbal tilt
        FEATURE_SOFTSERIAL = 1 << 6,          ///< Software serial ports
        FEATURE_GPS = 1 << 7,                 ///< GPS navigation
        FEATURE_RANGEFINDER = 1 << 9,         ///< Distance sensor
        FEATURE_TELEMETRY = 1 << 10,          ///< Telemetry output (FrSky, etc.)
        FEATURE_3D = 1 << 12,                 ///< 3D mode (reversible motors)
        FEATURE_RX_PARALLEL_PWM = 1 << 13,    ///< Parallel PWM receiver
        FEATURE_RX_MSP = 1 << 14,             ///< MSP receiver input
        FEATURE_RSSI_ADC = 1 << 15,           ///< RSSI via ADC
        FEATURE_LED_STRIP = 1 << 16,          ///< Addressable LED strip
        FEATURE_DASHBOARD = 1 << 17,          ///< Dashboard display
        FEATURE_OSD = 1 << 18,                ///< On-screen display
        FEATURE_CHANNEL_FORWARDING = 1 << 20, ///< Channel forwarding to servos
        FEATURE_TRANSPONDER = 1 << 21,        ///< Race transponder
        FEATURE_AIRMODE = 1 << 22,            ///< Airmode (keep stabilization at zero throttle)
        FEATURE_RX_SPI = 1 << 25,             ///< SPI receiver
        FEATURE_SOFTSPI = 1 << 26,            ///< Software SPI
        FEATURE_ESC_SENSOR = 1 << 27,         ///< ESC telemetry sensor
        FEATURE_ANTI_GRAVITY = 1 << 28,       ///< Anti-gravity mode
        FEATURE_DYNAMIC_FILTER = 1 << 29,     ///< Dynamic notch filter
    };


    /**
     * @struct msp
     * @brief MSP command processing state
     * 
     * @details Maintains state for MSP protocol packet parsing and command execution.
     *          MSP packets consist of header, size, command, data payload, and checksum.
     *          This structure tracks parsing progress and stores received data.
     */
    struct {
        enum mspState state;          ///< Current MSP parser state machine position
        enum mspPacketType packetType; ///< Direction: command or reply
        uint8_t offset;               ///< Current offset in data buffer during parsing
        uint8_t dataSize;             ///< Expected data payload size in bytes
        uint8_t checksum;             ///< Running XOR checksum for packet validation
        uint8_t buf[192];             ///< Data buffer for MSP packet payload
        uint8_t cmdMSP;               ///< MSP command code being processed
        enum escProtocol escMode;     ///< Selected ESC bootloader protocol type
        uint8_t portIndex;            ///< ESC/motor index for current operation
    } msp;

    /**
     * @enum blheliState
     * @brief BLHeli 4-way protocol state machine states for packet parsing
     * 
     * @details BLHeli 4-way packets have format: <cmd><addr_low><addr_high><len><data><crc16>
     *          State machine advances through header bytes to validate and parse commands
     *          for ESC flash memory read/write/erase operations.
     */
    enum blheliState {
        BLHELI_IDLE=0,           ///< Waiting for command byte
        BLHELI_HEADER_START,     ///< Received command, processing header
        BLHELI_HEADER_CMD,       ///< Command byte received, expecting address low byte
        BLHELI_HEADER_ADDR_LOW,  ///< Address low byte received, expecting address high byte
        BLHELI_HEADER_ADDR_HIGH, ///< Address high byte received, expecting length byte
        BLHELI_HEADER_LEN,       ///< Length received, expecting data bytes
        BLHELI_CRC1,             ///< Data received, expecting CRC16 first byte
        BLHELI_CRC2,             ///< CRC16 first byte received, expecting second byte
        BLHELI_COMMAND_RECEIVED  ///< Complete command received with valid CRC, ready to process
    };

    /**
     * @struct blheli
     * @brief BLHeli 4-way protocol command processing state
     * 
     * @details Maintains state for BLHeli 4-way protocol packet parsing and ESC
     *          bootloader command execution. Tracks connection status for multiple
     *          ESCs to support multi-ESC operations like simultaneous firmware flashing.
     */
    struct {
        enum blheliState state;                  ///< Current BLHeli protocol parser state
        uint8_t command;                         ///< BLHeli command code being processed
        uint16_t address;                        ///< ESC flash memory address for read/write operations
        uint16_t param_len;                      ///< Parameter/data length in bytes
        uint16_t offset;                         ///< Current offset in data buffer during parsing
        uint8_t buf[256+3+8];                    ///< Data buffer (256 bytes data + overhead)
        uint8_t crc1;                            ///< First byte of CRC16 checksum
        uint16_t crc;                            ///< Complete CRC16 checksum for packet validation
        bool connected[AP_BLHELI_MAX_ESCS];      ///< Per-ESC bootloader connection status
        uint8_t interface_mode[AP_BLHELI_MAX_ESCS]; ///< Per-ESC interface mode (SiLabs C2, SiLabs BLB, Atmel, ARM)
        uint8_t deviceInfo[AP_BLHELI_MAX_ESCS][4];  ///< Per-ESC device signature and info
        uint8_t chan;                            ///< Current ESC channel being accessed
        uint8_t ack;                             ///< Acknowledgment byte from ESC bootloader
    } blheli;

    const uint16_t esc_status_addr = 0xEB00; ///< BLHeli ESC status structure memory address
    
    /**
     * @enum esc_protocol
     * @brief Protocol type reported by ESC in status structure
     * 
     * @details Read from ESC flash memory at address 0xEB00 to determine
     *          currently configured motor control protocol in ESC firmware.
     */
    enum esc_protocol {
        ESC_PROTOCOL_NONE=0,       ///< No protocol configured (ESC not initialized)
        ESC_PROTOCOL_NORMAL=1,     ///< Standard PWM protocol
        ESC_PROTOCOL_ONESHOT125=2, ///< OneShot125 protocol
        ESC_PROTOCOL_DSHOT=5,      ///< DShot digital protocol
    };

    /**
     * @struct esc_status
     * @brief ESC status structure at flash memory address 0xEB00
     * 
     * @details BLHeli ESCs maintain runtime statistics in flash memory at fixed
     *          address 0xEB00. This structure contains protocol configuration and
     *          communication quality metrics.
     * 
     * @note Structure is PACKED to match ESC memory layout exactly
     */
    struct PACKED esc_status {
        uint8_t unknown[3];             ///< Reserved/unknown bytes (padding)
        enum esc_protocol protocol;     ///< Currently configured motor protocol
        uint32_t good_frames;           ///< Count of valid DShot frames received
        uint32_t bad_frames;            ///< Count of invalid/corrupted DShot frames
        uint32_t unknown2;              ///< Reserved/unknown field
    };
    
    
    AP_HAL::UARTDriver *uart;       ///< UART for pass-through communication with ESC bootloaders
    AP_HAL::UARTDriver *debug_uart; ///< UART for debug output (if enabled)
    AP_HAL::UARTDriver *telem_uart; ///< UART for ESC telemetry reception

    static const uint8_t max_motors = AP_BLHELI_MAX_ESCS; ///< Maximum number of ESCs supported
    uint8_t num_motors;             ///< Number of motors currently configured

    /// Last log output timestamp per motor to avoid beat frequencies in logging
    uint32_t last_log_ms[max_motors];

    bool initialised;               ///< Flag indicating BLHeli interface initialization complete

    uint32_t last_valid_ms;         ///< Timestamp of last valid protocol packet received (milliseconds)

    uint32_t serial_start_ms;       ///< Timestamp when serial ESC output was started (milliseconds)

    bool motors_disabled;           ///< Flag indicating motors currently disabled for pass-through
    uint32_t motors_disabled_mask;  ///< Bitmask of channels that should remain disabled

    bool uart_locked;               ///< Flag indicating UART is exclusively locked for pass-through

    bool mixed_type;                ///< True if vehicle has mix of reversible and normal ESCs

    uint8_t motor_map[max_motors];  ///< Mapping from BLHeli motor numbers to physical RC output channels
    uint32_t motor_mask;            ///< Bitmask of motors enabled for BLHeli protocol

    /// Offset for converting between servo number and FMU channel number for ESC telemetry
    uint8_t chan_offset;

    // when did we last request telemetry?
    uint32_t last_telem_request_us;             ///< Timestamp of last telemetry request (microseconds)
    uint8_t last_telem_esc;                     ///< Last ESC index that telemetry was requested from
    static const uint8_t telem_packet_size = 10; ///< ESC telemetry packet size in bytes (eRPM, voltage, current, temp)
    bool telem_uart_started;                    ///< Flag indicating telemetry UART initialization complete
    uint32_t last_telem_byte_read_us;           ///< Timestamp of last telemetry byte received (microseconds)
    int8_t last_control_port;                   ///< Last control port used for pass-through

    /**
     * @brief Release exclusive UART lock and restore normal operation
     * 
     * @details Releases UART locked by BLHeli pass-through protocol, allowing
     *          other systems to use the UART. Restores motor outputs if they
     *          were disabled during pass-through operation.
     * 
     * @note Called on protocol timeout or completion
     */
    void serial_end();
    
    /**
     * @brief Process one byte through MSP protocol state machine
     * 
     * @param[in] c Input byte to process
     * 
     * @return true if byte was consumed by MSP parser, false otherwise
     * 
     * @details Implements MSP packet parsing state machine, advancing through
     *          header, command, data, and checksum states. When complete packet
     *          received, calls msp_process_command() to execute the command.
     */
    bool msp_process_byte(uint8_t c);
    
    /**
     * @brief Update running CRC16 checksum for BLHeli protocol
     * 
     * @param[in] c Byte to add to CRC calculation
     * 
     * @details BLHeli 4-way protocol uses CRC16 for packet integrity validation.
     *          Updates blheli.crc with new byte using CRC16-CCITT polynomial.
     */
    void blheli_crc_update(uint8_t c);
    
    /**
     * @brief Process one byte through BLHeli 4-way protocol state machine
     * 
     * @param[in] c Input byte to process
     * 
     * @return true if byte was consumed by BLHeli parser, false otherwise
     * 
     * @details Implements BLHeli 4-way packet parsing state machine, advancing
     *          through command, address, length, data, and CRC states. When complete
     *          packet received with valid CRC, calls blheli_process_command().
     */
    bool blheli_4way_process_byte(uint8_t c);
    
    /**
     * @brief Convert BLHeli motor number to physical output channel
     * 
     * @param[in] motor BLHeli motor number (0 to num_motors-1)
     * 
     * @return Physical RC output channel number
     * 
     * @details Maps logical motor numbers used in BLHeli protocol to actual
     *          hardware output channels using motor_map array.
     */
    uint8_t blheli_chan_to_output_chan(uint8_t motor);
    
    /**
     * @brief Send MSP acknowledgment for received command
     * 
     * @param[in] cmd MSP command code to acknowledge
     * 
     * @details Sends MSP reply packet with no data payload, indicating
     *          command was received and processed successfully.
     */
    void msp_send_ack(uint8_t cmd);
    
    /**
     * @brief Send MSP reply packet with data payload
     * 
     * @param[in] cmd MSP command code for reply
     * @param[in] buf Data payload buffer
     * @param[in] len Length of data payload in bytes
     * 
     * @details Constructs and sends MSP reply packet with header, data, and checksum.
     */
    void msp_send_reply(uint8_t cmd, const uint8_t *buf, uint8_t len);
    /**
     * @brief Write 16-bit unsigned integer to buffer in little-endian format
     * 
     * @param[out] b Buffer to write to (must have at least 2 bytes)
     * @param[in] v Value to write
     */
    void putU16(uint8_t *b, uint16_t v);
    
    /**
     * @brief Read 16-bit unsigned integer from buffer in little-endian format
     * 
     * @param[in] b Buffer to read from (must have at least 2 bytes)
     * 
     * @return 16-bit value read from buffer
     */
    uint16_t getU16(const uint8_t *b);
    
    /**
     * @brief Write 32-bit unsigned integer to buffer in little-endian format
     * 
     * @param[out] b Buffer to write to (must have at least 4 bytes)
     * @param[in] v Value to write
     */
    void putU32(uint8_t *b, uint32_t v);
    
    /**
     * @brief Write 16-bit unsigned integer to buffer in big-endian format
     * 
     * @param[out] b Buffer to write to (must have at least 2 bytes)
     * @param[in] v Value to write
     * 
     * @note Used for BLHeli protocol which requires big-endian addresses
     */
    void putU16_BE(uint8_t *b, uint16_t v);
    
    /**
     * @brief Process complete MSP command packet
     * 
     * @details Decodes MSP command from msp.cmdMSP and executes appropriate
     *          handler. Commands include: connect/disconnect ESC, read/write
     *          ESC settings, set protocol mode, etc.
     * 
     * @note Called when MSP state machine reaches MSP_COMMAND_RECEIVED
     */
    void msp_process_command(void);
    
    /**
     * @brief Send BLHeli 4-way protocol reply packet
     * 
     * @param[in] buf Data buffer to send
     * @param[in] len Length of data in bytes
     * 
     * @details Constructs BLHeli reply packet with command, address, length,
     *          data, and CRC16 checksum, then sends to UART.
     */
    void blheli_send_reply(const uint8_t *buf, uint16_t len);
    
    /**
     * @brief Calculate CRC16-CCITT checksum for buffer
     * 
     * @param[in] buf Buffer to calculate CRC for
     * @param[in] len Length of buffer in bytes
     * 
     * @return 16-bit CRC value
     * 
     * @details Uses CRC16-CCITT polynomial for BLHeli protocol packet validation.
     */
    uint16_t BL_CRC(const uint8_t *buf, uint16_t len);
    
    /**
     * @brief Check if any ESC bootloader is connected
     * 
     * @return true if at least one ESC is connected, false otherwise
     * 
     * @details Checks blheli.connected[] array for any active connections.
     */
    bool isMcuConnected(void);
    
    /**
     * @brief Disconnect all ESC bootloaders and reset state
     * 
     * @details Clears all ESC connection flags and resets protocol state.
     *          Called on timeout or explicit disconnect command.
     */
    void setDisconnected(void);
    /**
     * @brief Send buffer to ESC bootloader via UART
     * 
     * @param[in] buf Buffer to send
     * @param[in] len Number of bytes to send
     * 
     * @return true if all bytes sent successfully, false on timeout
     * 
     * @details Writes bytes to locked UART with timeout protection.
     */
    bool BL_SendBuf(const uint8_t *buf, uint16_t len);
    
    /**
     * @brief Read buffer from ESC bootloader via UART
     * 
     * @param[out] buf Buffer to store received bytes
     * @param[in] len Number of bytes to read
     * 
     * @return true if all bytes received, false on timeout
     * 
     * @details Reads bytes from locked UART with timeout protection.
     */
    bool BL_ReadBuf(uint8_t *buf, uint16_t len);
    
    /**
     * @brief Wait for ACK byte from ESC bootloader
     * 
     * @param[in] timeout_ms Timeout in milliseconds (default 2ms)
     * 
     * @return ACK byte received from bootloader, or 0 on timeout
     * 
     * @details ESC bootloader sends single-byte acknowledgment after commands.
     */
    uint8_t BL_GetACK(uint16_t timeout_ms=2);
    
    /**
     * @brief Send "Set Address" command to ESC bootloader
     * 
     * @return true if command acknowledged, false otherwise
     * 
     * @details Sets target flash memory address in bootloader for subsequent
     *          read/write operations using blheli.address value.
     */
    bool BL_SendCMDSetAddress();
    
    /**
     * @brief Read data from ESC flash memory at current address
     * 
     * @param[in] cmd Bootloader read command code
     * @param[out] buf Buffer to store read data
     * @param[in] n Number of bytes to read
     * 
     * @return true if read successful, false otherwise
     * 
     * @details Sends read command and receives data from ESC flash memory.
     */
    bool BL_ReadA(uint8_t cmd, uint8_t *buf, uint16_t n);
    
    /**
     * @brief Connect to ESC bootloader
     * 
     * @return true if connection established, false otherwise
     * 
     * @details Sends bootloader handshake sequence and retrieves ESC device
     *          signature. Must be called before flash operations.
     * 
     * @note Motors are disabled during bootloader connection
     */
    bool BL_ConnectEx(void);
    
    /**
     * @brief Send keep-alive command to ESC bootloader
     * 
     * @return true if acknowledged, false otherwise
     * 
     * @details Prevents bootloader timeout during long operations.
     *          Bootloaders exit to application after ~5 seconds without commands.
     */
    bool BL_SendCMDKeepAlive(void);
    
    /**
     * @brief Erase flash memory page at current address
     * 
     * @return true if erase successful, false otherwise
     * 
     * @details Erases one page of ESC flash memory (typically 256-1024 bytes).
     *          Page must be erased before writing new data.
     * 
     * @warning PROPELLER REMOVAL REQUIRED - erasing flash disables motor control
     * @note Flash write operations can take 10-50ms per page
     */
    bool BL_PageErase(void);
    
    /**
     * @brief Exit bootloader and restart ESC application firmware
     * 
     * @details Sends command to exit bootloader mode and run main ESC firmware.
     *          ESC will reboot and resume normal motor control operation.
     * 
     * @note Does not wait for acknowledgment - ESC immediately resets
     */
    void BL_SendCMDRunRestartBootloader(void);
    
    /**
     * @brief Set bootloader write buffer with data
     * 
     * @param[in] buf Data to write
     * @param[in] nbytes Number of bytes (must be <= 256)
     * 
     * @return ACK byte from bootloader
     * 
     * @details Loads data into bootloader's write buffer before flash programming.
     *          Data is not written to flash until BL_WriteFlash() is called.
     */
    uint8_t BL_SendCMDSetBuffer(const uint8_t *buf, uint16_t nbytes);
    
    /**
     * @brief Write data to ESC flash memory
     * 
     * @param[in] cmd Bootloader write command code
     * @param[in] buf Data buffer to write
     * @param[in] nbytes Number of bytes to write
     * @param[in] timeout Timeout in milliseconds for write operation
     * 
     * @return true if write successful, false otherwise
     * 
     * @details Sends write command with data and waits for completion acknowledgment.
     */
    bool BL_WriteA(uint8_t cmd, const uint8_t *buf, uint16_t nbytes, uint32_t timeout);
    
    /**
     * @brief Write and verify data to ESC flash memory
     * 
     * @param[in] buf Data to write
     * @param[in] n Number of bytes to write
     * 
     * @return true if write and verify successful, false otherwise
     * 
     * @details Writes data to flash at current address, then reads back and
     *          verifies. Ensures data integrity for firmware flashing.
     * 
     * @warning PROPELLER REMOVAL REQUIRED before firmware flashing
     * @note Flash write operations modify ESC firmware - use with caution
     */
    bool BL_WriteFlash(const uint8_t *buf, uint16_t n);
    
    /**
     * @brief Verify ESC flash memory contents match buffer
     * 
     * @param[in] buf Expected data
     * @param[in] n Number of bytes to verify
     * 
     * @return true if flash contents match buffer, false otherwise
     * 
     * @details Reads flash memory and compares with provided buffer.
     *          Used to verify firmware flashing completed successfully.
     */
    bool BL_VerifyFlash(const uint8_t *buf, uint16_t n);
    
    /**
     * @brief Process complete BLHeli 4-way command packet
     * 
     * @details Decodes BLHeli command from blheli.command and executes
     *          appropriate bootloader operation (connect, read, write, erase, etc.).
     * 
     * @note Called when BLHeli state machine reaches BLHELI_COMMAND_RECEIVED
     * @warning Commands can modify ESC flash - ensure propellers removed
     */
    void blheli_process_command(void);
    
    /**
     * @brief Run ESC connection test on specified channel
     * 
     * @param[in] chan Motor channel to test
     * 
     * @details Attempts to connect to ESC bootloader and retrieve device info.
     *          Used for troubleshooting pass-through connectivity issues.
     */
    void run_connection_test(uint8_t chan);
    
    /**
     * @brief Read telemetry packet from ESC via serial or bidirectional DShot
     * 
     * @details Reads 10-byte telemetry packet containing:
     *          - eRPM (electrical RPM)
     *          - Voltage (V)
     *          - Current (A)
     *          - Temperature (°C)
     * 
     * @note Telemetry format varies by ESC firmware version
     */
    void read_telemetry_packet(void);
    
    /**
     * @brief Log bidirectional DShot telemetry data
     * 
     * @details Processes and logs ESC telemetry:
     *          - Converts eRPM to mechanical RPM using motor_poles
     *          - Logs voltage, current, temperature
     *          - Updates AP_ESC_Telem backend for system-wide telemetry
     * 
     * @note RPM conversion: mechanical_RPM = eRPM / (motor_poles / 2)
     */
    void log_bidir_telemetry(void);

    /**
     * @brief UART protocol handler hook for pass-through integration
     * 
     * @param[in] Unused parameter (protocol identifier)
     * @param[in] Pointer to UART driver for pass-through communication
     * 
     * @return true if protocol handling active, false otherwise
     * 
     * @details Registered as UART protocol handler to intercept bytes during
     *          BLHeli pass-through operation. Provides exclusive UART access
     *          for ESC bootloader communication while preventing interference
     *          from other systems (MAVLink, telemetry, etc.).
     * 
     * @note UART lock key prevents accidental protocol corruption
     */
    bool protocol_handler(uint8_t , AP_HAL::UARTDriver *);
};

#endif // HAL_SUPPORT_RCOUT_SERIAL
