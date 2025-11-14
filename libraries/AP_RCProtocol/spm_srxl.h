/*
MIT License

Copyright (c) 2019 Horizon Hobby, LLC

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

/**
 * @file spm_srxl.h
 * @brief Spektrum SRXL2 Protocol Engine with Bidirectional Communication
 * 
 * @details This file defines the complete SRXL2 (Spektrum Remote Receiver eXpress Link version 2)
 *          protocol stack supporting bidirectional communication between RC receivers, flight
 *          controllers, ESCs, servos, and other peripherals.
 * 
 *          SRXL2 Protocol Overview:
 *          - Device discovery and handshake negotiation
 *          - RC channel data transmission (up to 32 channels)
 *          - Telemetry data transmission from sensors to receiver
 *          - VTX (video transmitter) control and configuration
 *          - Bind mode entry and configuration
 *          - Forward programming support
 *          - Signal quality (RSSI) reporting
 *          - Configurable baud rates: 115200 bps or 400000 bps
 * 
 *          Protocol Phases:
 *          1. Discovery/Handshake: Devices negotiate capabilities, baud rates, and priorities
 *          2. Channel Data Reception: Receiver sends RC channel values (typically 11-22ms intervals)
 *          3. Telemetry Transmission: Devices send sensor data when polled by receiver
 *          4. Bind/VTX Control: Special commands for configuration and video TX control
 * 
 *          Integrator Requirements:
 *          Applications using this library MUST implement the following callback functions:
 *          - srxlChangeBaudRate(): Change UART baud rate (typically 115200 or 400000 bps)
 *          - srxlSendOnUart(): Transmit data buffer on UART
 *          - srxlFillTelemetry(): Populate telemetry packet with sensor data
 *          - srxlReceivedChannelData(): Process received RC channel values (in microseconds)
 *          - srxlOnBind(): Handle bind mode entry/exit
 *          - srxlOnVtx(): Handle VTX control commands
 * 
 * @warning TIMING-CRITICAL IMPLEMENTATION REQUIRED
 *          The SRXL2 handshake phase requires precise UART timing and immediate callback execution.
 *          Delays in responding to handshake packets (>50ms) will cause initialization failure.
 *          Callback functions must execute quickly without blocking operations.
 *          Baud rate changes must complete within the protocol timeout window.
 * 
 * @note CRC Configuration:
 *       CRC computation mode is selected in spm_srxl_config.h via SRXL_CRC_OPTIMIZE_MODE:
 *       - SRXL_CRC_OPTIMIZE_EXTERNAL: Use external CRC function
 *       - SRXL_CRC_OPTIMIZE_SPEED: Table lookup (fast, uses 512 bytes)
 *       - SRXL_CRC_OPTIMIZE_SIZE: Bitwise operations (slow, minimal memory)
 *       - SRXL_CRC_OPTIMIZE_STM_HW: STM32F30x hardware acceleration
 *       - SRXL_CRC_OPTIMIZE_STM_HAL: STM32 HAL driver acceleration
 * 
 * @note VTX Control Parameters:
 *       - Band: 0=Fatshark, 1=Raceband, 2=E-Band, 3=B-Band, 4=A-Band
 *       - Channel: 0-7 (8 channels per band)
 *       - Power: 0=Off, 1=1-14mW, 2=15-25mW, 3=26-99mW, 4=100-299mW, 5=300-600mW, 6=601+mW, 7=Manual
 *       - Pit Mode: 0=Race (normal power), 1=Pit (reduced power)
 * 
 * @see Spektrum SRXL2 Protocol Specification
 * @author Horizon Hobby, LLC
 * @copyright MIT License (c) 2019 Horizon Hobby, LLC
 */

#ifndef __SRXL_H__
#define __SRXL_H__

#ifdef __cplusplus
extern "C"
{
#endif

// Standard C Libraries
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>


/**
 * @defgroup SRXL_Protocol_Constants SRXL2 Protocol Constants
 * @{
 */

/** @brief SRXL2 protocol identifier - always 0xA6 for SRXL version 2 packets */
#define SPEKTRUM_SRXL_ID        (0xA6)

/** @brief Maximum size of SRXL packet buffer in bytes */
#define SRXL_MAX_BUFFER_SIZE    (80)

/** @brief Maximum number of SRXL devices that can be discovered on a single bus */
#define SRXL_MAX_DEVICES        (16)

/** @} */ // end of SRXL_Protocol_Constants

/**
 * @enum SrxlDevType
 * @brief SRXL2 Device Type Identifiers
 * 
 * @details Device type is encoded in the upper nibble of the SRXL device ID.
 *          Each device on the bus must have a unique device ID combining type and instance.
 *          The device type determines the device's role in the SRXL network and affects
 *          telemetry priority and message routing.
 */
typedef enum
{
    SrxlDevType_None                = 0,    /**< No device / uninitialized */
    SrxlDevType_RemoteReceiver      = 1,    /**< Remote receiver (satellite/secondary receiver) */
    SrxlDevType_Receiver            = 2,    /**< Primary RF receiver */
    SrxlDevType_FlightController    = 3,    /**< Flight controller / autopilot */
    SrxlDevType_ESC                 = 4,    /**< Electronic Speed Controller */
    SrxlDevType_SRXLServo1          = 6,    /**< SRXL-enabled servo (type 1) */
    SrxlDevType_SRXLServo2          = 7,    /**< SRXL-enabled servo (type 2) */
    SrxlDevType_VTX                 = 8,    /**< Video transmitter */
    SrxlDevType_Broadcast           = 15    /**< Broadcast address (all devices) */
} SrxlDevType;

/**
 * @defgroup SRXL_CRC_Modes CRC Computation Optimization Modes
 * @brief Configurable CRC-16 computation methods for SRXL2 packets
 * 
 * @details Set SRXL_CRC_OPTIMIZE_MODE in spm_srxl_config.h to select the CRC computation method.
 *          Trade-offs between speed, code size, and hardware requirements:
 *          - EXTERNAL: Integrator provides CRC function (most flexible)
 *          - SPEED: Fast table lookup (best performance, uses 512 bytes constant data)
 *          - SIZE: Bitwise computation (minimal memory, slower execution)
 *          - STM_HW: Hardware CRC unit (STM32F30x only, fastest)
 *          - STM_HAL: STM32 HAL hardware CRC (STM32F3/F7, requires HAL configuration)
 * @{
 */

/** @brief Use external CRC function defined by SRXL_CRC_EXTERNAL_FN macro */
#define SRXL_CRC_OPTIMIZE_EXTERNAL  (0)

/** @brief Use table lookup for CRC (fast, requires 512 bytes for CRC table) */
#define SRXL_CRC_OPTIMIZE_SPEED     (1)

/** @brief Use bitwise operations for CRC (slow, minimal memory footprint) */
#define SRXL_CRC_OPTIMIZE_SIZE      (2)

/** @brief Use STM32 register-level hardware CRC acceleration (STM32F30x only) */
#define SRXL_CRC_OPTIMIZE_STM_HW    (3)

/** @brief Use STM32Cube HAL driver for hardware CRC (STM32F3/F7, requires HAL setup) */
#define SRXL_CRC_OPTIMIZE_STM_HAL   (4)

/** @} */ // end of SRXL_CRC_Modes

/**
 * @defgroup SRXL_STM_Targets STM32 Target Family Identifiers
 * @brief Set SRXL_STM_TARGET_FAMILY when using STM hardware-accelerated CRC modes
 * @{
 */

/** @brief STM32F3 family target identifier */
#define SRXL_STM_TARGET_F3          (3)

/** @brief STM32F7 family target identifier */
#define SRXL_STM_TARGET_F7          (7)

/** @} */ // end of SRXL_STM_Targets

/**
 * @defgroup SRXL_Handshake Handshake Protocol Definitions
 * @brief Handshake packet identifiers and baud rate negotiation
 * 
 * @details During initialization, devices exchange handshake packets to:
 *          - Discover other devices on the bus
 *          - Negotiate optimal baud rate (115200 or 400000 bps)
 *          - Exchange device capabilities and telemetry priority
 *          - Assign unique device IDs to prevent conflicts
 * 
 * @warning Handshake timing is critical - responses must occur within 50ms
 * @{
 */

/** @brief Handshake packet type identifier */
#define SRXL_HANDSHAKE_ID       (0x21)

/** @brief Default baud rate: 115200 bps (bitmask value 0x00) */
#define SRXL_BAUD_115200        (0x00)

/** @brief High-speed baud rate: 400000 bps (bitmask value 0x01) */
#define SRXL_BAUD_400000        (0x01)

/** @} */ // end of SRXL_Handshake

/**
 * @defgroup SRXL_DeviceInfo Device Information Capability Flags
 * @brief Capability flags sent in handshake packet info field
 * 
 * @details These bit flags indicate device capabilities and configuration.
 *          Multiple flags can be combined using bitwise OR.
 * @{
 */

/** @brief Non-RF device (no wireless telemetry capability) */
#define SRXL_DEVINFO_NO_RF              (0x00)

/** @brief Device is configured to transmit telemetry over RF link */
#define SRXL_DEVINFO_TELEM_TX_ENABLED   (0x01)

/** @brief Device can send full-range telemetry over RF (extended distance) */
#define SRXL_DEVINFO_TELEM_FULL_RANGE   (0x02)

/** @brief Device supports Forward Programming via RF or SRXL bus */
#define SRXL_DEVINFO_FWD_PROG_SUPPORT   (0x04)

/** @} */ // end of SRXL_DeviceInfo

/**
 * @defgroup SRXL_Bind Bind Mode Protocol Definitions
 * @brief Bind procedure control and status reporting
 * 
 * @details Binding establishes the RF link between transmitter and receiver.
 *          The bind process involves:
 *          1. Flight controller sends SRXL_BIND_REQ_ENTER to receiver
 *          2. Receiver enters bind mode and awaits RF signal from transmitter
 *          3. User initiates bind on transmitter (typically via button press)
 *          4. Receiver captures transmitter GUID and reports via SRXL_BIND_REQ_BOUND_DATA
 *          5. Flight controller confirms with SRXL_BIND_REQ_SET_BIND
 * 
 * @note Bind procedure timing: Allow 10-60 seconds for user to initiate transmitter bind.
 *       UI should provide clear feedback when receiver is in bind mode.
 * @{
 */

/** @brief Bind packet type identifier */
#define SRXL_BIND_ID                    (0x41)

/** @brief Request receiver to enter bind mode */
#define SRXL_BIND_REQ_ENTER             (0xEB)

/** @brief Request current bind status from receiver */
#define SRXL_BIND_REQ_STATUS            (0xB5)

/** @brief Request bound data (GUID) from receiver after successful bind */
#define SRXL_BIND_REQ_BOUND_DATA        (0xDB)

/** @brief Set bind information (confirm bind with GUID) */
#define SRXL_BIND_REQ_SET_BIND          (0x5B)

/** @brief No bind options selected */
#define SRXL_BIND_OPT_NONE              (0x00)

/** @brief Enable this device to transmit telemetry over RF */
#define SRXL_BIND_OPT_TELEM_TX_ENABLE   (0x01)

/** @brief Enable this device to respond to bind requests with RF discovery packet */
#define SRXL_BIND_OPT_BIND_TX_ENABLE    (0x02)

/** @brief Request US transmit power levels instead of EU power limits */
#define SRXL_BIND_OPT_US_POWER          (0x04)

/** @} */ // end of SRXL_Bind

/**
 * @enum BIND_STATUS
 * @brief Spektrum DSM/DSMX Bind Status and Protocol Types
 * 
 * @details Indicates current bind status and the specific DSM protocol variant in use.
 *          DSM2 and DSMX are 2.4GHz protocols with different channel hopping strategies.
 *          Air types are for aircraft, surface types are for ground vehicles.
 *          Frame rates: 22ms (45Hz), 11ms (90Hz), 5.5ms (180Hz) affect latency and smoothness.
 */
typedef enum
{
    NOT_BOUND           = 0x00,     /**< Receiver not bound to any transmitter */
    // Air protocol types (for aircraft)
    DSM2_1024_22MS      = 0x01,     /**< DSM2 protocol, 1024 resolution, 22ms frame rate */
    DSM2_1024_MC24      = 0x02,     /**< DSM2 protocol, 1024 resolution, MC24 mode */
    DSM2_2048_11MS      = 0x12,     /**< DSM2 protocol, 2048 resolution, 11ms frame rate */
    DSMX_22MS           = 0xA2,     /**< DSMX protocol, 22ms frame rate (standard latency) */
    DSMX_11MS           = 0xB2,     /**< DSMX protocol, 11ms frame rate (low latency) */
    // Surface protocol types (for ground vehicles)
    SURFACE_DSM1        = 0x40,     /**< Surface DSM1 protocol */
    SURFACE_DSM2_16p5MS = 0x63,     /**< Surface DSM2 protocol, 16.5ms frame rate */
    DSMR_11MS_22MS      = 0xE2,     /**< DSMR protocol, dual rate 11/22ms */
    DSMR_5p5MS          = 0xE4,     /**< DSMR protocol, 5.5ms frame rate (ultra-low latency) */
} BIND_STATUS;

//      7.4 Parameter Configuration
#define SRXL_PARAM_ID           (0x50)
#define SRXL_PARAM_REQ_QUERY    (0x50)
#define SRXL_PARAM_REQ_WRITE    (0x57)

//      7.5 Signal Quality Packet
#define SRXL_RSSI_ID            (0x55)
#define SRXL_RSSI_REQ_REQUEST   (0x52)
#define SRXL_RSSI_REQ_SEND      (0x53)

//      7.6 Telemetry Sensor Data Packet
#define SRXL_TELEM_ID           (0x80)

//      7.7 Control Data Packet
#define SRXL_CTRL_ID                (0xCD)
#define SRXL_CTRL_BASE_LENGTH       (3 + 2 + 2) // header + cmd/replyID + crc
#define SRXL_CTRL_CMD_CHANNEL       (0x00)
#define SRXL_CTRL_CMD_CHANNEL_FS    (0x01)
#define SRXL_CTRL_CMD_VTX           (0x02)
#define SRXL_CTRL_CMD_FWDPGM        (0x03)

/**
 * @enum SRXL_CMD
 * @brief SRXL2 Command Types
 * 
 * @details Command identifiers used throughout the SRXL protocol for different
 *          packet types and operations. These are used internally by the protocol
 *          engine to route and process different message types.
 */
typedef enum
{
    SRXL_CMD_NONE,          /**< No command / idle state */
    SRXL_CMD_CHANNEL,       /**< RC channel data packet */
    SRXL_CMD_CHANNEL_FS,    /**< Failsafe channel data packet */
    SRXL_CMD_VTX,           /**< Video transmitter control command */
    SRXL_CMD_FWDPGM,        /**< Forward programming data */
    SRXL_CMD_RSSI,          /**< Signal quality / RSSI report */
    SRXL_CMD_HANDSHAKE,     /**< Device handshake / discovery */
    SRXL_CMD_TELEMETRY,     /**< Telemetry data packet */
    SRXL_CMD_ENTER_BIND,    /**< Enter bind mode command */
    SRXL_CMD_REQ_BIND,      /**< Request bind status */
    SRXL_CMD_SET_BIND,      /**< Set bind configuration */
    SRXL_CMD_BIND_INFO,     /**< Bind information report */
} SRXL_CMD;

/**
 * @defgroup SRXL_VTX Video Transmitter Control Parameters
 * @brief VTX configuration values for frequency, power, and operating mode
 * 
 * @details Video transmitter control allows the flight controller to configure
 *          FPV video TX settings via the RC receiver. Parameters include:
 *          - Band and channel selection (determines RF frequency)
 *          - Power level (from off to 600mW+)
 *          - Pit mode (reduced power for bench testing)
 *          - Regional power limits (US vs EU regulations)
 * 
 * @note VTX commands are sent when srxlSetVtxData() is called and processed
 *       by the receiver, which forwards them to the VTX device.
 * @{
 */

/** @brief Fatshark band (5740-5905 MHz in 8 channels) */
#define VTX_BAND_FATSHARK   (0)

/** @brief Raceband (5658-5917 MHz in 8 channels, popular for racing) */
#define VTX_BAND_RACEBAND   (1)

/** @brief E-Band (5705-5885 MHz in 8 channels) */
#define VTX_BAND_E_BAND     (2)

/** @brief B-Band (Boscam B, 5733-5885 MHz in 8 channels) */
#define VTX_BAND_B_BAND     (3)

/** @brief A-Band (Boscam A, 5865-5905 MHz in 8 channels) */
#define VTX_BAND_A_BAND     (4)

/** @brief Race mode - normal transmit power */
#define VTX_MODE_RACE   (0)

/** @brief Pit mode - reduced power for bench testing (typically <25mW) */
#define VTX_MODE_PIT    (1)

/** @brief VTX off (no transmission) */
#define VTX_POWER_OFF           (0)

/** @brief Power range: 1mW to 14mW */
#define VTX_POWER_1MW_14MW      (1)

/** @brief Power range: 15mW to 25mW */
#define VTX_POWER_15MW_25MW     (2)

/** @brief Power range: 26mW to 99mW */
#define VTX_POWER_26MW_99MW     (3)

/** @brief Power range: 100mW to 299mW */
#define VTX_POWER_100MW_299MW   (4)

/** @brief Power range: 300mW to 600mW */
#define VTX_POWER_300MW_600MW   (5)

/** @brief Power: 601mW and above (high power, check local regulations) */
#define VTX_POWER_601_PLUS      (6)

/** @brief Manual power control (VTX-specific) */
#define VTX_POWER_MANUAL        (7)

/** @brief US region power limits (typically higher power allowed) */
#define VTX_REGION_US   (0)

/** @brief EU region power limits (typically 25mW limit) */
#define VTX_REGION_EU   (1)

/** @} */ // end of SRXL_VTX

// Forward Programming Pass-Thru
#define FWD_PGM_MAX_DATA_SIZE   (64)


// Enable byte packing for all structs defined here!
#ifdef PACKED
#define SRXL_EXTERNAL_PACKED
#elif defined(__GNUC__)
#define PACKED __attribute__((packed))
#else
#pragma pack(push, 1)
#define PACKED
#endif

/**
 * @struct SrxlHeader
 * @brief SRXL2 Packet Header
 * 
 * @details Every SRXL2 packet begins with this 3-byte header identifying
 *          the protocol version, packet type, and total packet length.
 */
typedef struct SrxlHeader
{
    uint8_t srxlID;         /**< Protocol ID - always 0xA6 for SRXL version 2 */
    uint8_t packetType;     /**< Packet type identifier (handshake, control, telemetry, etc.) */
    uint8_t length;         /**< Total packet length in bytes including header and CRC */
} PACKED SrxlHeader;

/**
 * @struct SrxlHandshakeData
 * @brief Handshake Packet Payload
 * 
 * @details Exchanged during device discovery phase. Devices negotiate baud rates,
 *          exchange capabilities, and establish telemetry priorities.
 * 
 * @note The uid field should be randomly generated or derived from device serial number
 *       to enable detection of address conflicts (two devices with same deviceID).
 */
typedef struct SrxlHandshakeData
{
    uint8_t     srcDevID;       /**< Source device ID (type in upper nibble, instance in lower) */
    uint8_t     destDevID;      /**< Destination device ID (0xFF for broadcast during discovery) */
    uint8_t     priority;       /**< Telemetry priority (0-99, lower is higher priority) */
    uint8_t     baudSupported;  /**< Supported baud rates bitmask: 0x00=115200 bps, 0x01=400000 bps */
    uint8_t     info;           /**< Capability flags - see SRXL_DEVINFO_xxx definitions */
    uint32_t    uid;            /**< Unique device identifier for conflict detection */
} PACKED SrxlHandshakeData;

/**
 * @struct SrxlHandshakePacket
 * @brief Complete Handshake Packet
 * 
 * @details Full handshake packet including header, payload, and CRC-16.
 *          Sent during initialization to discover devices and negotiate bus parameters.
 * 
 * @warning Must be sent within 50ms of bus initialization or brown-out recovery.
 *          Delays will cause handshake timeout and initialization failure.
 */
typedef struct SrxlHandshakePacket
{
    SrxlHeader          hdr;        /**< Packet header with SRXL_HANDSHAKE_ID */
    SrxlHandshakeData   payload;    /**< Handshake data payload */
    uint16_t            crc;        /**< CRC-16 checksum of packet (header + payload) */
} PACKED SrxlHandshakePacket;

/**
 * @struct SrxlBindData
 * @brief Bind Configuration Data
 * 
 * @details Contains bind type (DSM protocol variant), options, transmitter GUID,
 *          and receiver UID for establishing RF link between TX and RX.
 */
typedef struct SrxlBindData
{
    uint8_t     type;       /**< Bind type - see BIND_STATUS enum for protocol types */
    uint8_t     options;    /**< Bind options - see SRXL_BIND_OPT_xxx flags */
    uint64_t    guid;       /**< Transmitter GUID (globally unique identifier) */
    uint32_t    uid;        /**< Receiver UID for identification */
} PACKED SrxlBindData;

/**
 * @struct SrxlBindPacket
 * @brief Complete Bind Packet
 * 
 * @details Used for bind mode entry, status requests, and bind configuration.
 *          Flight controller sends bind requests, receiver responds with status.
 * 
 * @note Bind procedure requires user interaction (button press on transmitter).
 *       Allow 10-60 seconds for bind completion and provide UI feedback.
 */
typedef struct SrxlBindPacket
{
    SrxlHeader      hdr;        /**< Packet header with SRXL_BIND_ID */
    uint8_t         request;    /**< Request type - see SRXL_BIND_REQ_xxx definitions */
    uint8_t         deviceID;   /**< Target device ID for bind command */
    SrxlBindData    data;       /**< Bind configuration data */
    uint16_t        crc;        /**< CRC-16 checksum */
} PACKED SrxlBindPacket;

/**
 * @struct SrxlTelemetryData
 * @brief Telemetry Sensor Data Payload
 * 
 * @details Contains sensor data in Spektrum telemetry format. The 16-byte payload
 *          includes sensor ID, secondary ID, and 14 bytes of sensor-specific data.
 *          Sensor data format varies by sensor type (GPS, voltage, current, etc.).
 * 
 * @note Integrators must implement srxlFillTelemetry() callback to populate this
 *       structure with sensor data when telemetry is requested by the receiver.
 */
typedef struct SrxlTelemetryData
{
    union
    {
        struct
        {
            uint8_t sensorID;       /**< Spektrum sensor ID (determines data format) */
            uint8_t secondaryID;    /**< Secondary sensor identifier (instance number) */
            uint8_t data[14];       /**< Sensor-specific data payload (14 bytes) */
        };
        uint8_t raw[16];            /**< Raw 16-byte telemetry payload */
    };
} PACKED SrxlTelemetryData;

/**
 * @struct SrxlTelemetryPacket
 * @brief Complete Telemetry Packet
 * 
 * @details Flight controller or other device sends telemetry when polled by receiver.
 *          Telemetry is transmitted back to the transmitter over the RF link.
 * 
 * @note Telemetry packets are sent in response to receiver polling based on
 *       the telemetry priority negotiated during handshake.
 */
typedef struct SrxlTelemetryPacket
{
    SrxlHeader          hdr;        /**< Packet header with SRXL_TELEM_ID */
    uint8_t             destDevID;  /**< Destination device ID (typically receiver) */
    SrxlTelemetryData   payload;    /**< Telemetry sensor data */
    uint16_t            crc;        /**< CRC-16 checksum */
} PACKED SrxlTelemetryPacket;

// Signal Quality
typedef struct SrxlRssiPacket
{
    SrxlHeader  hdr;
    uint8_t     request;
    int8_t      antennaA;
    int8_t      antennaB;
    int8_t      antennaC;
    int8_t      antennaD;
    uint16_t    crc;
} PACKED SrxlRssiPacket;

// Parameter Config
typedef struct SrxlParamPacket
{
    SrxlHeader  hdr;
    uint8_t     request;
    uint8_t     destDevID;
    uint32_t    paramID;
    uint32_t    paramVal;
    uint16_t    crc;
} PACKED SrxlParamPacket;

/**
 * @struct SrxlVtxData
 * @brief Video Transmitter Configuration Data
 * 
 * @details Complete VTX configuration including frequency band/channel, power level,
 *          pit mode, and regional power limits. Sent to receiver which forwards to VTX.
 * 
 * @note Band and channel determine RF frequency: freq_MHz = base_freq + (channel * spacing)
 *       Power level affects range and battery consumption. Pit mode is for bench testing.
 * 
 * @warning Check local regulations for maximum allowed transmit power in your region.
 *          EU typically limits to 25mW, US allows higher power levels.
 */
typedef struct SrxlVtxData
{
    uint8_t band;           /**< VTX band: 0-4 (Fatshark/Raceband/E/B/A) */
    uint8_t channel;        /**< VTX channel: 0-7 (8 channels per band) */
    uint8_t pit;            /**< Pit mode: 0=Race (normal power), 1=Pit (reduced power <25mW) */
    uint8_t power;          /**< Power level: 0-7 (see VTX_POWER_xxx definitions) */
    uint16_t powerDec;      /**< Power in milliwatts (1mW units, for precise control) */
    uint8_t region;         /**< Region: 0=US (higher power), 1=EU (25mW limit) */
} PACKED SrxlVtxData;

/**
 * @struct SrxlFwdPgmData
 * @brief Forward Programming Passthrough Data
 * 
 * @details Used to pass configuration data through the SRXL bus to a device,
 *          typically for firmware updates or parameter configuration.
 */
typedef struct SrxlFwdPgmData
{
    uint8_t rfu[3];                         /**< Reserved for future use (padding for alignment) */
    uint8_t data[FWD_PGM_MAX_DATA_SIZE];    /**< Forward programming data payload (64 bytes max) */
} PACKED SrxlFwdPgmData;

/**
 * @struct SrxlChannelData
 * @brief RC Channel Data Packet Payload
 * 
 * @details Contains up to 32 RC channel values with signal quality information.
 *          Channel values are 16-bit with 32768 as center position.
 *          The mask field indicates which channels are present in the packet.
 * 
 * @note Channel values are in 16-bit range: 0-65535, center=32768
 *       To convert to microseconds: us = 988 + (value >> 1) / 1.6384
 *       Typical range: 1000-2000 microseconds with 1500 as center
 * 
 * @warning Integrators must implement srxlReceivedChannelData() callback to
 *          process channel values when received from receiver.
 */
typedef struct SrxlChannelData
{
    int8_t    rssi;             /**< Signal strength: RSSI in dBm (negative value) or percentage */
    uint16_t  frameLosses;      /**< Total frame losses since power-on or fade count */
    uint32_t  mask;             /**< Channel presence bitmask: bit N set = channel N present */
    uint16_t  values[32];       /**< Channel values 0-65535 (32768=center, lower 2 bits reserved) */
} PACKED SrxlChannelData;

// Control Data
typedef struct SrxlControlData
{
    uint8_t cmd;
    uint8_t replyID;
    union
    {
        SrxlChannelData channelData;    // Used for Channel Data and Failsafe Channel Data commands
        SrxlVtxData     vtxData;        // Used for VTX commands
        SrxlFwdPgmData  fpData;         // Used to pass forward programming data to an SRXL device
    };
} PACKED SrxlControlData;

typedef struct SrxlControlPacket
{
    SrxlHeader      hdr;
    SrxlControlData payload;
//  uint16_t        crc;    // NOTE: Since this packet is variable-length, we can't use this value anyway
} PACKED SrxlControlPacket;

// SRXL Packets
typedef union
{
    SrxlHeader          header;
    SrxlBindPacket      bind;
    SrxlHandshakePacket handshake;
    SrxlTelemetryPacket telemetry;
    SrxlRssiPacket      rssi;
    SrxlParamPacket     parameter;
    SrxlControlPacket   control;
    uint8_t             raw[SRXL_MAX_BUFFER_SIZE];
} SrxlPacket;

// SRXL full device identifier -- SRXL Device ID with bus number
typedef union
{
    struct
    {
        uint8_t deviceID;
        uint8_t busIndex;
    };
    uint16_t word;
} PACKED SrxlFullID;

// Restore packing back to default
#ifndef SRXL_EXTERNAL_PACKED
#undef PACKED
#ifndef __GNUC__
#pragma pack(pop)
#endif
#endif

// Global vars
extern SrxlChannelData srxlChData;
extern SrxlTelemetryData srxlTelemData;
extern SrxlVtxData srxlVtxData;

// Include config here, after all typedefs that might be needed within it
#include "spm_srxl_config.h"

#ifndef FALLTHROUGH
#define FALLTHROUGH
#endif

#if !defined(SRXL_NUM_OF_BUSES)
#error "SRXL_NUM_OF_BUSES must be defined in spm_srxl_config.h!"
#elif SRXL_NUM_OF_BUSES <= 0
#error "SRXL_NUM_OF_BUSES must be defined in spm_srxl_config.h!"
#elif SRXL_NUM_OF_BUSES > 1
#define SRXL_IS_HUB
#endif
#define SRXL_ALL_BUSES          ((1u << SRXL_NUM_OF_BUSES) - 1)
#define SRXL_MAX_RCVRS          (2 * SRXL_NUM_OF_BUSES)
#ifndef SRXL_CRC_OPTIMIZE_MODE  // NOTE: This should be set in spm_srxl_config.h
#define SRXL_CRC_OPTIMIZE_MODE  SRXL_CRC_OPTIMIZE_SPEED
#endif

#define RSSI_RCVD_NONE  (0)
#define RSSI_RCVD_DBM   (1)
#define RSSI_RCVD_PCT   (2)
#define RSSI_RCVD_BOTH  (3)

/**
 * @enum SrxlState
 * @brief SRXL2 Protocol State Machine States
 * 
 * @details The SRXL protocol engine operates as a state machine progressing through
 *          initialization, discovery, and operational states. States control timing
 *          of handshake exchanges, telemetry polling, and special operations.
 * 
 * @note State transitions are driven by srxlRun() which must be called periodically
 *       (typically every 1ms) to advance the protocol state machine.
 * 
 * @warning Timing of state transitions is critical for proper handshake completion.
 *          Handshake phase must complete within 200ms of bus initialization.
 */
typedef enum
{
    SrxlState_Disabled,             /**< Bus disabled (before init or after shutdown) */
    SrxlState_ListenOnStartup,      /**< Listen 50ms to detect existing traffic (brown-out recovery) */
    SrxlState_SendHandshake,        /**< Send handshake packets every 50ms during discovery */
    SrxlState_ListenForHandshake,   /**< Listen 150ms for handshake responses from other devices */
    SrxlState_Running,              /**< Normal operational state (processing channel/telemetry data) */
    SrxlState_SendTelemetry,        /**< Transmit telemetry response when polled by receiver */
    SrxlState_SendVTX,              /**< Transmit VTX configuration packet */
    SrxlState_SendEnterBind,        /**< Transmit bind entry command */
    SrxlState_SendBoundDataReport,  /**< Transmit bound data report after successful bind */
    SrxlState_SendSetBindInfo,      /**< Transmit bind confirmation with GUID */
} SrxlState;

//#ifdef SRXL_IS_HUB
typedef struct SrxlRcvrEntry
{
    uint8_t     deviceID;       // SRXL device ID of the receiver
    uint8_t     busBits;        // Supports 8 buses, with each bit corresponding to busIndex (bit 0 = bus 0, bit 7 = bus 7)
    uint8_t     info;           // Info bits reported during handshake - See SRXL_DEVINFO_XXX mask bits in header
    uint8_t     rssiRcvd;       // 0 = none, 1 = dBm, 2 = percent, 3 = both dBm and percent
    int8_t      rssi_dBm;       // Latest RSSI dBm value reported by receiver (negative, varies with receiver type)
    int8_t      rssi_Pct;       // Latest RSSI percent range estimate reported by receiver (0-100)
    uint16_t    fades;          // Latest number of fades reported for a given receiver
    uint32_t    channelMask;    // Latest channel mask for channels provided in channel data packet (0 during fade)
} SrxlRcvrEntry;

typedef struct SrxlReceiverInfo
{
    SrxlRcvrEntry   rcvr[SRXL_MAX_RCVRS];       // Stats for each receiver, filled when ch data is received
    SrxlRcvrEntry*  rcvrSorted[SRXL_MAX_RCVRS]; // Pointers to receiver entries sorted in telemetry range order
    uint8_t         rcvrSortInsert;             // Index into rcvrSorted where full-range telem rcvrs should be inserted
    uint8_t         rcvrCount;                  // Number of entries in rcvr[] and rcvrSorted[]
    uint8_t         rxBusBits;
    int8_t          bestRssi_dBm;
    int8_t          bestRssi_Pct;
    uint8_t         lossCountdown;  // Reset to lossHoldCount when frame is good, and decrement for each consecutive
                                    // frame loss -- when we get to 0, convert lossHoldCount frame losses to a hold
    uint8_t         lossHoldCount;  // Consecutive frame losses required to count as hold
    uint16_t        frameLosses;    // Increment each time all receivers are in frame loss -- if 45
                                    // consecutive, subtract those and increment holds
    uint16_t        holds;          // Increment each time 45 or more consecutive frames are lost (but don't keep
                                    // incrementing once in that state)
    SrxlRcvrEntry*  pTelemRcvr;     // Pointer to current assigned telemetry receiver (used for checking
                                    // for fade to know when to switch)
    SrxlRcvrEntry*  pBindRcvr;      // Pointer to receiver that we told to Enter Bind Mode (used to
                                    // process Bound Data Report and send Set Bind Info)
} SrxlReceiverStats;
//#endif

typedef struct SrxlDevEntry
{
    uint8_t deviceID;
    uint8_t priority;   // Requested telemetry priority of this device
    uint8_t info;       // Refer to SRXL_DEVINFO_XXX mask bits in header
    uint8_t rfu;
} SrxlDevEntry;

typedef struct SrxlTxFlags
{
    unsigned int enterBind : 1;
    unsigned int getBindInfo : 1;
    unsigned int setBindInfo : 1;
    unsigned int broadcastBindInfo : 1;
    unsigned int reportBindInfo : 1;
    unsigned int sendVtxData : 1;
    unsigned int sendFwdPgmData : 1;
} SrxlTxFlags;

typedef struct SrxlBus
{
    SrxlPacket      srxlOut;            // Transmit packet buffer
    SrxlPacket      srxlIn;             // Receive packet buffer

    SrxlState       state;              // Current state of SRXL state machine
    SrxlFullID      fullID;             // Device ID and Bus Index of this device, set during init
    uint8_t         rxDevCount;         // Number of other SRXL devices discovered via handshake
    SrxlDevEntry    rxDev[SRXL_MAX_DEVICES];    // Device entries for tracking SRXL telemetry priorities
#ifdef SRXL_INCLUDE_MASTER_CODE
    uint16_t        rxDevAge[SRXL_MAX_DEVICES]; // Telemetry age value for the associated device
#endif
    uint16_t        rxDevPrioritySum;   // Sum of priorities requested for each discovered SRXL device
    uint16_t        timeoutCount_ms;    // Milliseconds since SRXL packet was received (incremented in srxlRun)
    uint8_t         requestID;          // Device ID to poll
    uint8_t         baudSupported;      // Baud rates this device can do: 0 = 115200, 1 = 400000
    uint8_t         baudRate;           // Current baud rate: 0 = 115200, 1 = 400000
    uint8_t         frameErrCount;      // Number of consecutive missed frames
    SrxlTxFlags     txFlags;            // Pending outgoing packet types
    uint8_t         uart;               // Index number of UART tied to this SRXL bus
    SrxlRcvrEntry*  pMasterRcvr;        // Receiver entry for the bus master, if one exists
    bool            master;             // True if this device is the bus master on this bus
    bool            initialized;        // True when this SRXL bus is initialized
} SrxlBus;

typedef struct SrxlDevice
{
    SrxlDevEntry    devEntry;   // Device info for this local device, shared across all buses.
    uint32_t        uid;        // ID statistically likely to be unique (Random, hash of serial, etc.)
    SrxlRcvrEntry*  pRcvr;      // Pointer to our receiver entry, if we're a receiver (don't set for
                                // flight controller acting as hub -- only true receiver)
    bool vtxProxy;              // Set true if this device can and should respond to VTX commands
} SrxlDevice;


/**
 * @defgroup SRXL_API SRXL2 Library API Functions
 * @brief Public API for SRXL2 protocol initialization and operation
 * @{
 */

/**
 * @brief Initialize SRXL device parameters
 * 
 * @details Sets device type, telemetry priority, capabilities, and unique ID.
 *          Must be called once before initializing any buses.
 * 
 * @param[in] deviceID Device ID combining type (upper nibble) and instance (lower nibble)
 * @param[in] priority Telemetry priority 0-99 (lower value = higher priority, 0 = highest)
 * @param[in] info Device capability flags - see SRXL_DEVINFO_xxx definitions
 * @param[in] uid Unique device identifier (random or hash of serial number)
 * 
 * @return true if initialization successful, false on error
 * 
 * @note Call this before srxlInitBus(). Device parameters are shared across all buses.
 */
bool srxlInitDevice(uint8_t deviceID, uint8_t priority, uint8_t info, uint32_t uid);

/**
 * @brief Initialize an SRXL bus
 * 
 * @details Initializes state machine, assigns UART, and sets supported baud rates.
 *          After initialization, call srxlRun() periodically to advance the protocol.
 * 
 * @param[in] busIndex Bus index 0 to SRXL_NUM_OF_BUSES-1
 * @param[in] uart UART hardware index for this bus (platform-specific)
 * @param[in] baudSupported Bitmask of supported baud rates (SRXL_BAUD_115200 | SRXL_BAUD_400000)
 * 
 * @return true if initialization successful, false on error
 * 
 * @warning After calling, immediately start calling srxlRun() at 1ms intervals.
 *          Handshake phase begins and timing is critical for discovery.
 * 
 * @note Must call srxlInitDevice() first to set device parameters.
 */
bool srxlInitBus(uint8_t busIndex, uint8_t uart, uint8_t baudSupported);

/**
 * @brief Check if this device is the bus master on specified bus
 * 
 * @param[in] busIndex Bus index to query
 * @return true if this device is bus master, false otherwise
 * 
 * @note Bus master is determined during handshake based on device priorities.
 */
bool srxlIsBusMaster(uint8_t busIndex);

/**
 * @brief Get time since last valid SRXL packet received
 * 
 * @param[in] busIndex Bus index to query
 * @return Timeout count in milliseconds
 * 
 * @note Use to detect loss of communication with receiver. Typical frame interval is 11-22ms.
 */
uint16_t srxlGetTimeoutCount_ms(uint8_t busIndex);

/**
 * @brief Get device ID for this device on specified bus
 * 
 * @param[in] busIndex Bus index to query
 * @return Device ID (type in upper nibble, instance in lower nibble)
 */
uint8_t srxlGetDeviceID(uint8_t busIndex);

/**
 * @brief Parse received SRXL packet
 * 
 * @details Call this function when bytes are received on UART. Parses packet,
 *          validates CRC, and triggers appropriate callbacks (channel data, VTX, bind, etc.).
 * 
 * @param[in] busIndex Bus index receiving the packet
 * @param[in] packet Pointer to received packet buffer
 * @param[in] length Length of received packet in bytes
 * 
 * @return true if packet valid and processed, false if CRC error or invalid packet
 * 
 * @warning Must be called promptly when UART data is received to maintain timing.
 *          Delays in packet processing can cause handshake or telemetry failures.
 * 
 * @note This function will invoke integrator callbacks:
 *       - srxlReceivedChannelData() for channel data packets
 *       - srxlOnVtx() for VTX control packets
 *       - srxlOnBind() for bind-related packets
 */
bool srxlParsePacket(uint8_t busIndex, uint8_t *packet, uint8_t length);

/**
 * @brief Run SRXL protocol state machine
 * 
 * @details Advances protocol state machine, handles timing, sends periodic packets.
 *          MUST be called periodically (every 1ms recommended) to maintain protocol timing.
 * 
 * @param[in] busIndex Bus index to service
 * @param[in] timeoutDelta_ms Milliseconds elapsed since last call (typically 1)
 * 
 * @warning TIMING-CRITICAL: Must be called at regular intervals without large gaps.
 *          Irregular calling or delays >50ms during handshake will cause init failure.
 *          During normal operation, delays affect telemetry timing and failsafe detection.
 * 
 * @note This function may invoke srxlSendOnUart() callback to transmit packets.
 */
void srxlRun(uint8_t busIndex, int16_t timeoutDelta_ms);

/**
 * @brief Request receiver to enter bind mode
 * 
 * @details Sends bind entry command to receiver. User must then initiate bind on transmitter.
 * 
 * @param[in] bindType Desired bind type (see BIND_STATUS enum for protocol options)
 * @param[in] broadcast true to broadcast to all receivers, false for specific receiver
 * 
 * @return true if bind request sent successfully, false on error
 * 
 * @note After calling, allow 10-60 seconds for user to initiate transmitter bind.
 *       Provide clear UI feedback indicating bind mode is active.
 *       Monitor for srxlOnBind() callback indicating bind completion or failure.
 */
bool srxlEnterBind(uint8_t bindType, bool broadcast);

/**
 * @brief Set bind information after successful bind
 * 
 * @details Confirms bind with transmitter GUID and receiver UID.
 * 
 * @param[in] bindType Bind type (DSM protocol variant)
 * @param[in] guid Transmitter GUID from bind process
 * @param[in] uid Receiver UID
 * 
 * @return true if bind info set successfully, false on error
 */
bool srxlSetBindInfo(uint8_t bindType, uint64_t guid, uint32_t uid);

/**
 * @brief Notify protocol of frame error or loss
 * 
 * @param[in] busIndex Bus index experiencing frame error
 * 
 * @note Call when expected frame is not received within timeout period.
 *       Used for failsafe detection and signal quality tracking.
 */
void srxlOnFrameError(uint8_t busIndex);

/**
 * @brief Get current telemetry endpoint (receiver requesting telemetry)
 * 
 * @return Full device ID of telemetry endpoint (device ID + bus index)
 * 
 * @note Use to determine which receiver is currently polling for telemetry.
 */
SrxlFullID srxlGetTelemetryEndpoint(void);

/**
 * @brief Set VTX configuration data to be transmitted
 * 
 * @details Queues VTX configuration for transmission to receiver/VTX.
 *          Packet will be sent on next protocol cycle via srxlRun().
 * 
 * @param[in] pVtxData Pointer to VTX configuration data
 * 
 * @return true if VTX data queued successfully, false on error
 * 
 * @note VTX parameters: band (0-4), channel (0-7), power (0-7), pit mode (0-1)
 *       See VTX_BAND_xxx, VTX_POWER_xxx, VTX_MODE_xxx definitions for valid values.
 * 
 * @warning Check local regulations for maximum transmit power limits before setting power level.
 */
bool srxlSetVtxData(SrxlVtxData *pVtxData);

/**
 * @brief Pass forward programming data through SRXL bus
 * 
 * @param[in] pData Pointer to forward programming data
 * @param[in] length Length of data in bytes (max FWD_PGM_MAX_DATA_SIZE = 64)
 * 
 * @return true if data queued successfully, false on error
 */
bool srxlPassThruFwdPgm(uint8_t *pData, uint8_t length);

/**
 * @brief Set frame loss threshold for hold detection
 * 
 * @param[in] countdownReset Number of consecutive frame losses required for hold state
 * 
 * @note Default is typically 45 consecutive frames (1 second at 22ms frame rate).
 */
void srxlSetHoldThreshold(uint8_t countdownReset);

/**
 * @brief Clear communication statistics (frame losses, holds, RSSI)
 */
void srxlClearCommStats(void);

/**
 * @brief Update communication statistics with current frame status
 * 
 * @param[in] isFade true if current frame is lost/faded, false if received OK
 * @return true if statistics updated, false on error
 */
bool srxlUpdateCommStats(bool isFade);

/** @} */ // end of SRXL_API

/**
 * @defgroup SRXL_Integrator_Callbacks Required Integrator Callback Functions
 * @brief Functions that MUST be implemented by the integrator application
 * 
 * @details These callback functions are invoked by the SRXL library and must be provided
 *          by the integrator to handle platform-specific operations and application logic.
 * 
 * @warning ALL callback functions listed here MUST be implemented. Missing callbacks
 *          will cause link errors. Callbacks must execute quickly without blocking.
 * 
 * @{
 */

/**
 * @brief Change UART baud rate (REQUIRED CALLBACK)
 * 
 * @details Integrator must implement this function to change the UART baud rate.
 *          Called during handshake negotiation when higher baud rate is available.
 * 
 * @param[in] busIndex SRXL bus index (maps to UART instance)
 * @param[in] baudRate Baud rate: 0=115200 bps, 1=400000 bps
 * 
 * @return true if baud rate change successful, false on error
 * 
 * @warning TIMING-CRITICAL: Baud rate change must complete within 50ms.
 *          During baud rate transition, no packets should be sent/received.
 *          Flush UART buffers before and after baud rate change.
 * 
 * @note Typical implementation:
 *       1. Wait for TX buffer to empty
 *       2. Change UART baud rate
 *       3. Flush RX buffer
 *       4. Return true if successful
 * 
 * @code
 * bool srxlChangeBaudRate(uint8_t busIndex, uint8_t baudRate) {
 *     uint32_t baud = (baudRate == SRXL_BAUD_400000) ? 400000 : 115200;
 *     uart_set_baudrate(busIndex, baud);
 *     uart_flush_rx(busIndex);
 *     return true;
 * }
 * @endcode
 */
// bool srxlChangeBaudRate(uint8_t busIndex, uint8_t baudRate);

/**
 * @brief Transmit data on UART (REQUIRED CALLBACK)
 * 
 * @details Integrator must implement this function to send SRXL packets on UART.
 *          Called by protocol engine when packets need to be transmitted.
 * 
 * @param[in] busIndex SRXL bus index (maps to UART instance)
 * @param[in] pBuffer Pointer to data buffer to transmit
 * @param[in] length Number of bytes to transmit
 * 
 * @return true if transmission initiated successfully, false on error
 * 
 * @warning Must execute quickly without blocking. Use interrupt-driven or DMA transmission.
 *          Ensure previous transmission is complete before starting new transmission.
 * 
 * @note Typical implementation uses UART TX interrupt or DMA:
 * 
 * @code
 * bool srxlSendOnUart(uint8_t busIndex, uint8_t *pBuffer, uint8_t length) {
 *     return uart_transmit_dma(busIndex, pBuffer, length);
 * }
 * @endcode
 */
// bool srxlSendOnUart(uint8_t busIndex, uint8_t *pBuffer, uint8_t length);

/**
 * @brief Fill telemetry packet with sensor data (REQUIRED CALLBACK)
 * 
 * @details Integrator must implement this function to populate telemetry packet with
 *          current sensor data. Called when receiver polls for telemetry.
 * 
 * @param[in]  destDevID Destination device ID (receiver requesting telemetry)
 * @param[out] pTelemetryData Pointer to telemetry data structure to fill
 * 
 * @return true if telemetry data provided, false if no data available
 * 
 * @note Telemetry data format follows Spektrum telemetry specification.
 *       Common sensor IDs: GPS, voltage, current, altitude, temperature, etc.
 *       Fill sensorID, secondaryID, and 14-byte data payload.
 * 
 * @warning Must execute quickly (<1ms). Do not perform sensor reads in this callback;
 *          use cached sensor data that is updated in background tasks.
 * 
 * @code
 * bool srxlFillTelemetry(uint8_t destDevID, SrxlTelemetryData *pTelemetryData) {
 *     pTelemetryData->sensorID = TELEM_SENSOR_VOLTAGE;
 *     pTelemetryData->secondaryID = 0;
 *     // Fill data[0-13] with sensor-specific format
 *     uint16_t voltage_mv = get_cached_battery_voltage();
 *     pTelemetryData->data[0] = voltage_mv >> 8;
 *     pTelemetryData->data[1] = voltage_mv & 0xFF;
 *     return true;
 * }
 * @endcode
 */
// bool srxlFillTelemetry(uint8_t destDevID, SrxlTelemetryData *pTelemetryData);

/**
 * @brief Process received RC channel data (REQUIRED CALLBACK)
 * 
 * @details Integrator must implement this function to handle RC channel values.
 *          Called when channel data packet is received from receiver.
 * 
 * @param[in] pChannelData Pointer to channel data structure with values and metadata
 * @param[in] isFailsafe true if this is failsafe channel data, false for normal data
 * 
 * @return true if channel data processed, false on error
 * 
 * @note Channel values are 16-bit: 0-65535, with 32768 as center position.
 *       Convert to microseconds: us = 988 + (value / 1.6384) or use right-shift approximation.
 *       Typical servo range: 1000-2000 µs with 1500 µs center.
 * 
 * @warning Must execute quickly (<1ms). Pass channel data to flight control tasks;
 *          do not perform heavy processing in this callback.
 * 
 * @code
 * bool srxlReceivedChannelData(SrxlChannelData *pChannelData, bool isFailsafe) {
 *     for (int i = 0; i < 16; i++) {
 *         if (pChannelData->mask & (1 << i)) {
 *             uint16_t value_us = 988 + (pChannelData->values[i] >> 1) / 1.6384;
 *             rc_channel_update(i, value_us);
 *         }
 *     }
 *     return true;
 * }
 * @endcode
 */
// bool srxlReceivedChannelData(SrxlChannelData *pChannelData, bool isFailsafe);

/**
 * @brief Handle bind mode events (REQUIRED CALLBACK)
 * 
 * @details Integrator must implement this function to handle bind-related events.
 *          Called when bind mode is entered, bind completes, or bind info is received.
 * 
 * @param[in] cmd Bind command type (SRXL_CMD_ENTER_BIND, SRXL_CMD_SET_BIND, etc.)
 * @param[in] pBindData Pointer to bind data (may be NULL for some commands)
 * 
 * @return true if bind event handled, false on error
 * 
 * @note Provide user feedback when entering bind mode (LED blinking, UI message, etc.).
 *       Bind process typically takes 10-60 seconds for user to activate transmitter bind.
 * 
 * @code
 * bool srxlOnBind(SRXL_CMD cmd, SrxlBindData *pBindData) {
 *     switch (cmd) {
 *         case SRXL_CMD_ENTER_BIND:
 *             led_blink_fast();  // Visual feedback
 *             ui_display_message("Bind Mode Active");
 *             break;
 *         case SRXL_CMD_SET_BIND:
 *             led_solid();
 *             ui_display_message("Bind Complete");
 *             save_bind_data(pBindData);
 *             break;
 *     }
 *     return true;
 * }
 * @endcode
 */
// bool srxlOnBind(SRXL_CMD cmd, SrxlBindData *pBindData);

/**
 * @brief Handle VTX control commands (REQUIRED CALLBACK)
 * 
 * @details Integrator must implement this function to handle VTX configuration commands.
 *          Called when VTX control packet is received (typically forwarded by receiver).
 * 
 * @param[in] pVtxData Pointer to VTX configuration data
 * 
 * @return true if VTX command processed, false on error
 * 
 * @note If device is VTX proxy (vtxProxy flag set), forward command to actual VTX hardware.
 *       VTX configuration includes band, channel, power, pit mode, and region.
 * 
 * @warning Verify power level complies with local regulations before applying.
 *          EU typically limits to 25mW, US allows higher power.
 * 
 * @code
 * bool srxlOnVtx(SrxlVtxData *pVtxData) {
 *     if (device_is_vtx_proxy) {
 *         vtx_set_band(pVtxData->band);
 *         vtx_set_channel(pVtxData->channel);
 *         vtx_set_power(pVtxData->powerDec);
 *         vtx_set_pit_mode(pVtxData->pit);
 *         return true;
 *     }
 *     return false;
 * }
 * @endcode
 */
// bool srxlOnVtx(SrxlVtxData *pVtxData);

/** @} */ // end of SRXL_Integrator_Callbacks

#ifdef __cplusplus
} // extern "C"
#endif

#ifdef SRXL_INCLUDE_MASTER_CODE
// NOTE: Most user applications should not be an SRXL2 bus master, so master-specific code is not open.
// If your application requires this functionality, please inquire about this from Spektrum RC.
#include "spm_srxl_master.h"
#endif

#endif //__SRXL_H__
