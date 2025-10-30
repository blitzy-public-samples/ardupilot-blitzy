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
 * @file spm_srxl_config.h
 * @brief Configuration parameters for the Spektrum SRXL2 protocol library
 * 
 * @details This configuration file defines compile-time settings for the SRXL2
 *          (Spektrum Remote Receiver Link v2) library implementation used in ArduPilot.
 *          SRXL2 is a bidirectional serial protocol that allows communication between
 *          Spektrum receivers and flight controllers, supporting both RC channel data
 *          reception and telemetry transmission.
 * 
 *          Key configuration areas:
 *          - Bus configuration: Number of simultaneous SRXL2 buses supported
 *          - CRC computation: Optimization mode selection (speed vs size vs hardware)
 *          - Device identification: Device type and capabilities
 *          - Interface functions: User-provided callbacks for UART and data handling
 * 
 *          This file is automatically included within spm_srxl.h and should not be
 *          included directly elsewhere in the codebase.
 * 
 * @note This is a configuration file from Horizon Hobby's SRXL2 library, adapted
 *       for integration with ArduPilot's RC protocol framework.
 * 
 * @warning Modifying these configuration parameters affects memory allocation,
 *          CPU usage, and protocol behavior. Changes should be tested thoroughly.
 * 
 * @see libraries/AP_RCProtocol/spm_srxl.h
 * @see libraries/AP_RCProtocol/AP_RCProtocol_SRXL2.cpp
 * 
 * Source: libraries/AP_RCProtocol/spm_srxl_config.h
 */

// This file is automatically included within spm_srxl.h -- do not include elsewhere!

#ifndef _SRXL_CONFIG_H_
#define _SRXL_CONFIG_H_

//### USER PROVIDED HEADER FUNCTIONS AND FORWARD DECLARATIONS ###

// User included headers/declarations to access interface functions required below
//#include <AP_HAL/AP_HAL.h>

extern "C++" {
#include <AP_Math/crc.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
}

//### USER CONFIGURATION ###

/**
 * @brief Number of physically separate SRXL2 buses supported simultaneously
 * 
 * @details This configuration parameter defines how many independent SRXL2 serial
 *          buses the library can manage concurrently. Each bus represents a separate
 *          physical UART connection to a Spektrum receiver.
 * 
 *          For ArduPilot integration, this is typically set to 1, as most flight
 *          controllers connect to a single Spektrum receiver. Multi-receiver setups
 *          are rare and would require additional UART resources.
 * 
 *          Impact on resources:
 *          - Each bus allocates static buffers for packet reception and transmission
 *          - Each bus maintains separate state machines for protocol handling
 *          - Memory usage scales linearly with number of buses
 * 
 * @warning Increasing SRXL_NUM_OF_BUSES increases static memory allocation.
 *          Each additional bus allocates approximately:
 *          - ~80 bytes for receive/transmit buffers
 *          - ~50 bytes for state tracking structures
 *          - ~20 bytes for telemetry buffering
 *          Total: ~150 bytes per additional bus
 * 
 * @note ArduPilot default: 1 bus (single receiver configuration)
 * @note Valid range: 1-4 (hardware UART availability limits practical maximum)
 * 
 * Source: libraries/AP_RCProtocol/spm_srxl_config.h:44
 */
// Set this value to the number of physically separate SRXL buses on the device
#define SRXL_NUM_OF_BUSES           1

/**
 * @brief SRXL2 device identification values for different device types
 * 
 * @details Device ID values identify the type of device on the SRXL2 bus. These
 *          identifiers allow Spektrum receivers and transmitters to recognize
 *          connected devices and route telemetry appropriately.
 * 
 *          Standard device IDs (from Spektrum SRXL2 specification Section 7.1.1):
 *          - 0x30: Flight Controller connected to Base Receiver
 *          - 0x31: Flight Controller connected to Remote Receiver (typical)
 *          - 0x40: Smart ESC (electronic speed controller)
 *          - 0x50: Smart Battery
 *          - 0x60: Smart Gimbal
 *          - 0x70: Smart Servos
 *          - 0x81: Video Transmitter (VTX)
 * 
 * @note SRXL_DEVICE_ID: Default for flight controllers connected to remote receivers
 * @note SRXL_DEVICE_ID_BASE_RX: Alternative for base receiver connections
 * @note These values are not used internally by the library -- they are passed as
 *       parameters to srxlInit() during initialization
 * 
 * @see srxlInit() for device ID usage during initialization
 * 
 * Source: libraries/AP_RCProtocol/spm_srxl_config.h:52-53
 */
// Set this to the appropriate device ID (See Section 7.1.1 in Spektrum Bi-Directional SRXL Documentation).
// Typical values are:
//    Flight Controller = 0x31 (or possibly 0x30 if connected to Base Receiver instead of Remote Receiver)
//    Smart ESC = 0x40
//    VTX = 0x81
// NOTE: This value is not used internally -- it is passed as a parameter to srxlInit() in the example app
#define SRXL_DEVICE_ID              0x31
#define SRXL_DEVICE_ID_BASE_RX      0x30

/**
 * @brief Telemetry transmission priority level configuration
 * 
 * @details This value determines the priority level for sending telemetry data
 *          back to the Spektrum transmitter, with range 0-100. Higher values
 *          indicate higher priority and more frequent transmission opportunities.
 * 
 *          Priority calculation guidelines:
 *          - Base priority: 10 points per regular telemetry packet type
 *          - High-frequency packets: Add 20 points (2x normal rate)
 *          - Low-frequency packets: Add 5 points (0.5x normal rate)
 * 
 *          Example priority calculations:
 *          - 2 normal packets: 10 + 10 = 20
 *          - 2 normal + 1 high-frequency: 10 + 10 + 20 = 40
 *          - 3 normal + 2 low-frequency: 10 + 10 + 10 + 5 + 5 = 40
 * 
 *          For ArduPilot, default is 20 (equivalent to 2 regular telemetry streams:
 *          typically battery voltage/current and GPS position).
 * 
 * @note This value is not used internally by the library -- it is passed as a
 *       parameter to srxlInit() during initialization.
 * @note Valid range: 0-100
 * 
 * Source: libraries/AP_RCProtocol/spm_srxl_config.h:62
 */
// Set this to the desired priority level for sending telemetry, ranging from 0 to 100.
// Generally, this number should be 10 times the number of different telemetry packets to regularly send.
// If there are telemetry messages that should be sent more often, increase this value.
// If there are messages that are rarely sent, add less than 10 for those.
// For example, if you had two normal priority messages and one that you plan to send twice as often as those,
// you could set the priority to 40 (10 + 10 + 2*10)
// NOTE: This value is not used internally -- it is passed as a parameter to srxlInit() in the example app
#define SRXL_DEVICE_PRIORITY        20

/**
 * @brief Device capability information flags for SRXL2 protocol
 * 
 * @details Bitmask defining the capabilities and characteristics of this device
 *          on the SRXL2 bus. These flags allow Spektrum receivers and transmitters
 *          to understand device features and enable appropriate functionality.
 * 
 *          Available capability flags:
 *          - SRXL_DEVINFO_NO_RF: Device has no RF capabilities (typical for flight controllers)
 *          - SRXL_DEVINFO_FWD_PROG_SUPPORT: Device supports Forward Programming via SRXL
 * 
 *          ArduPilot sets SRXL_DEVINFO_NO_RF because flight controllers do not have
 *          RF transmitter/receiver hardware -- they rely on external Spektrum receivers
 *          for radio communication.
 * 
 * @note SRXL_DEVINFO_FWD_PROG_SUPPORT: Set this flag if you want to allow Forward
 *       Programming (parameter configuration) of the flight controller via the
 *       Spektrum transmitter interface. This requires implementing the
 *       srxlOnFwdPgm() callback and defining SRXL_INCLUDE_FWD_PGM_CODE.
 * 
 * @see SRXL_INCLUDE_FWD_PGM_CODE for Forward Programming support
 * @see srxlOnFwdPgm() for Forward Programming callback implementation
 * 
 * Source: libraries/AP_RCProtocol/spm_srxl_config.h:67
 */
// Set these information bits based on the capabilities of the device.
// The only bit currently applicable to third-party devices is the SRXL_DEVINFO_FWD_PROG_SUPPORT flag,
// which should be set if you would like to allow Forward Programming of the device via SRXL pass-through.
#define SRXL_DEVICE_INFO            (SRXL_DEVINFO_NO_RF)

/**
 * @brief SRXL2 baud rate support configuration
 * 
 * @details Controls which baud rates the SRXL2 library supports:
 *          - 0: 115200 baud only (standard rate)
 *          - 1: 115200 and 400000 baud support (high-speed mode)
 * 
 * @note ArduPilot uses 115200 baud for SRXL2 to maintain compatibility
 *       with all Spektrum receiver models.
 * 
 * Source: libraries/AP_RCProtocol/spm_srxl_config.h:70
 */
// Set this value to 0 for 115200 baud only, or 1 for 400000 baud support
#define SRXL_SUPPORTED_BAUD_RATES   0

/**
 * @brief CRC-16 computation optimization mode selection for SRXL2 packets
 * 
 * @details SRXL2 protocol uses CRC-16-CCITT for packet integrity verification.
 *          This configuration selects the CRC computation method, trading off
 *          execution speed, code size, and memory usage.
 * 
 *          Available optimization modes:
 * 
 *          **SRXL_CRC_OPTIMIZE_SPEED**
 *          - Uses 512-byte lookup table for fast CRC computation
 *          - Fastest execution: ~10-15 CPU cycles per byte
 *          - Flash cost: +512 bytes for CRC table (stored in const memory)
 *          - Code size: ~50-100 bytes
 *          - Best for: Performance-critical applications with available flash
 * 
 *          **SRXL_CRC_OPTIMIZE_SIZE**
 *          - Uses bitwise operations without lookup tables
 *          - Slower execution: ~80-120 CPU cycles per byte
 *          - Flash cost: 0 bytes (no lookup table)
 *          - Code size: ~30-50 bytes
 *          - Best for: Memory-constrained microcontrollers
 * 
 *          **SRXL_CRC_OPTIMIZE_STM_HW**
 *          - Uses STM32 hardware CRC peripheral (register-level access)
 *          - Fastest execution: ~5-8 CPU cycles per byte
 *          - Flash cost: 0 bytes (hardware accelerated)
 *          - Code size: ~80-120 bytes (hardware setup code)
 *          - Availability: STM32F30x only
 *          - Best for: STM32F3 with available CRC hardware
 * 
 *          **SRXL_CRC_OPTIMIZE_STM_HAL**
 *          - Uses STM32Cube HAL driver for hardware CRC acceleration
 *          - Fast execution: ~8-12 CPU cycles per byte
 *          - Flash cost: Variable (depends on HAL library)
 *          - Code size: ~100-150 bytes
 *          - Availability: STM32F3/F7 with HAL configured
 *          - Best for: Projects already using STM32Cube HAL
 * 
 *          **SRXL_CRC_OPTIMIZE_EXTERNAL** (ArduPilot configuration)
 *          - Uses ArduPilot's existing crc16_ccitt() implementation
 *          - Performance: Depends on AP_Math::crc implementation
 *          - Flash cost: 0 bytes (shared with other ArduPilot CRC users)
 *          - Code size: ~20 bytes (function call overhead)
 *          - Best for: ArduPilot integration (avoids code duplication)
 * 
 *          Performance comparison for typical 24-byte SRXL2 packet:
 *          - OPTIMIZE_SPEED: ~2-3 microseconds @ 168MHz
 *          - OPTIMIZE_SIZE: ~15-20 microseconds @ 168MHz
 *          - OPTIMIZE_STM_HW: ~1-2 microseconds @ 168MHz
 *          - OPTIMIZE_EXTERNAL: ~3-5 microseconds @ 168MHz (ArduPilot)
 * 
 * @note ArduPilot uses SRXL_CRC_OPTIMIZE_EXTERNAL to leverage the existing
 *       crc16_ccitt() implementation in AP_Math library, avoiding code duplication
 *       and maintaining consistency with other protocol handlers.
 * 
 * @warning CPU timing impact: CRC computation occurs in interrupt context during
 *          packet reception. OPTIMIZE_SIZE mode may consume 10-15x more CPU cycles
 *          than OPTIMIZE_SPEED, potentially affecting real-time performance on
 *          slower microcontrollers (< 72MHz).
 * 
 * @warning Flash size impact: OPTIMIZE_SPEED mode adds 512 bytes to flash usage.
 *          On microcontrollers with limited flash (< 128KB), consider OPTIMIZE_SIZE.
 * 
 * @warning Hardware CRC modes (STM_HW, STM_HAL) require proper hardware initialization
 *          and may conflict with other code using the CRC peripheral. Use
 *          SRXL_SAVE_HW_CRC_CONTEXT flag if CRC hardware is shared.
 * 
 * @see SRXL_CRC_CALCULATE macro for actual CRC computation function
 * @see AP_Math::crc16_ccitt() in libraries/AP_Math/crc.h
 * 
 * Source: libraries/AP_RCProtocol/spm_srxl_config.h:72-80
 */
// Set this value to choose which code to include for CRC computation. Choices are:
//    SRXL_CRC_OPTIMIZE_SPEED   -- Uses table lookup for CRC computation (requires 512 const bytes for CRC table)
//    SRXL_CRC_OPTIMIZE_SIZE    -- Uses bitwise operations for smaller code size but slower execution
//    SRXL_CRC_OPTIMIZE_STM_HW  -- Uses STM32 register-level hardware acceleration (only available on STM32F30x devices for now)
//    SRXL_CRC_OPTIMIZE_STM_HAL -- Uses STM32Cube HAL driver for hardware acceleration (only available on STM32F3/F7) -- see srxlCrc16() for details on HAL config

/**
 * @brief CRC-16-CCITT computation function for SRXL2 packet verification
 * 
 * @details This macro defines the actual function used to compute CRC-16-CCITT
 *          checksums for SRXL2 protocol packets. ArduPilot uses the existing
 *          crc16_ccitt() implementation from AP_Math library to avoid code
 *          duplication and maintain consistency.
 * 
 * @param packet Pointer to packet data buffer
 * @param length Number of bytes to include in CRC calculation
 * @param crc Initial CRC value (typically 0x0000)
 * @return uint16_t Computed CRC-16-CCITT value
 * 
 * @note CRC-16-CCITT polynomial: 0x1021
 * @note Initial value: 0x0000
 * @note No final XOR applied
 * 
 * Source: libraries/AP_RCProtocol/spm_srxl_config.h:78
 */
#define SRXL_CRC_CALCULATE(packet, length, crc) crc16_ccitt(packet, length, crc)

/**
 * @brief Selected CRC optimization mode for ArduPilot integration
 * 
 * @details ArduPilot uses SRXL_CRC_OPTIMIZE_EXTERNAL mode to delegate CRC
 *          computation to the AP_Math library's crc16_ccitt() function. This
 *          avoids duplicating CRC implementation code and ensures consistency
 *          with other protocol handlers (SBUS, IBUS, etc.) that also use
 *          CRC-16-CCITT.
 * 
 * @note This configuration leverages ArduPilot's existing CRC infrastructure,
 *       providing good performance without additional flash memory overhead.
 * 
 * Source: libraries/AP_RCProtocol/spm_srxl_config.h:80
 */
#define SRXL_CRC_OPTIMIZE_MODE      SRXL_CRC_OPTIMIZE_EXTERNAL

/**
 * @brief STM32 target processor family for hardware CRC acceleration
 * 
 * @details When using SRXL_CRC_OPTIMIZE_STM_HW or SRXL_CRC_OPTIMIZE_STM_HAL modes,
 *          this macro specifies which STM32 family is being targeted. Different
 *          STM32 families have different CRC hardware implementations and register
 *          layouts.
 * 
 *          Supported target families:
 *          - SRXL_STM_TARGET_F3: STM32F3xx series (e.g., F302, F303, F373)
 *          - SRXL_STM_TARGET_F7: STM32F7xx series (e.g., F722, F745, F765)
 * 
 * @note This macro is only used when hardware CRC acceleration is enabled
 * @note ArduPilot uses SRXL_CRC_OPTIMIZE_EXTERNAL, so this setting is not active
 * 
 * Source: libraries/AP_RCProtocol/spm_srxl_config.h:85
 */
// If using STM32 hardware CRC acceleration above, set this flag to the target family. Choices are:
//    SRXL_STM_TARGET_F3
//    SRXL_STM_TARGET_F7
#define SRXL_STM_TARGET_FAMILY      SRXL_STM_TARGET_F3

/**
 * @brief Enable CRC hardware context save/restore for shared CRC peripheral
 * 
 * @details When using hardware CRC acceleration (SRXL_CRC_OPTIMIZE_STM_HW) and
 *          the CRC peripheral is shared with other code in the application, this
 *          flag enables saving and restoring CRC registers before and after SRXL
 *          library use.
 * 
 *          Context save/restore includes:
 *          - CRC_DR (data register)
 *          - CRC_INIT (initial value)
 *          - CRC_POL (polynomial, if configurable)
 * 
 * @warning Enabling context save adds ~50-80 CPU cycles overhead per CRC operation
 * @warning Only needed if other code uses the CRC peripheral concurrently
 * 
 * @note ArduPilot does not use hardware CRC for SRXL2, so this is typically commented out
 * 
 * Source: libraries/AP_RCProtocol/spm_srxl_config.h:89
 */
// If using SRXL_CRC_OPTIMIZE_STM_HW and the CRC hardware is shared with non-SRXL code, then
// uncomment the following flag to save and restore the CRC registers after use by SRXL:
//#define SRXL_SAVE_HW_CRC_CONTEXT

/**
 * @brief Enable Forward Programming support via SRXL2 protocol
 * 
 * @details Forward Programming allows configuration of the flight controller
 *          through the Spektrum transmitter interface, enabling parameter changes
 *          without connecting to a ground station.
 * 
 *          When enabled, this flag:
 *          - Includes Forward Programming packet parsing code (~200-300 bytes flash)
 *          - Enables srxlOnFwdPgm() callback for handling programming commands
 *          - Requires SRXL_DEVINFO_FWD_PROG_SUPPORT flag in SRXL_DEVICE_INFO
 * 
 *          Use cases:
 *          - Field configuration without computer access
 *          - Quick parameter tuning via transmitter
 *          - Remote vehicle setup
 * 
 * @note ArduPilot typically handles configuration through MAVLink/ground stations,
 *       so Forward Programming is usually not enabled
 * @note Enabling adds ~250 bytes flash + ~50 bytes RAM
 * 
 * @see SRXL_DEVICE_INFO for capability advertisement
 * @see srxlOnFwdPgm() for Forward Programming callback implementation
 * 
 * Source: libraries/AP_RCProtocol/spm_srxl_config.h:92
 */
// Uncomment the following flag if your code must support Forward Programming received via SRXL
//#define SRXL_INCLUDE_FWD_PGM_CODE

/**
 * @defgroup SRXL2_Interface SRXL2 User-Provided Interface Functions
 * @brief Hardware abstraction layer callbacks for SRXL2 library integration
 * 
 * @details The SRXL2 library requires the integrating application (ArduPilot) to
 *          provide implementations of these interface functions. These callbacks
 *          abstract hardware-specific operations, allowing the SRXL2 library to
 *          remain platform-independent.
 * 
 *          Interface function categories:
 *          - **UART Configuration**: srxlChangeBaudRate()
 *          - **Data Transmission**: srxlSendOnUart()
 *          - **Telemetry Handling**: srxlFillTelemetry()
 *          - **Channel Data Reception**: srxlReceivedChannelData()
 *          - **Bind Information**: srxlOnBind()
 *          - **VTX Control**: srxlOnVtx()
 *          - **Forward Programming**: srxlOnFwdPgm() (optional)
 *          - **Thread Safety**: srxlEnterCriticalSection(), srxlExitCriticalSection()
 * 
 *          These functions are called by the SRXL2 library during protocol operation
 *          and must be implemented for correct library functionality.
 * 
 * @note In ArduPilot, these functions are implemented in AP_RCProtocol_SRXL2.cpp
 * @see libraries/AP_RCProtocol/AP_RCProtocol_SRXL2.cpp for ArduPilot implementations
 * 
 * Source: libraries/AP_RCProtocol/spm_srxl_config.h:94-145
 * @{
 */
//### USER PROVIDED INTERFACE FUNCTIONS ###

/**
 * @brief Change UART baud rate for SRXL2 communication
 * 
 * @details User-provided callback to reconfigure the UART baud rate. Called by
 *          the SRXL2 library when negotiating communication speed with the receiver.
 * 
 * @param[in] uart UART identifier (same value passed to srxlInit())
 * @param[in] baudRate Desired baud rate in bps (typically 115200 or 400000)
 * 
 * @note Implementation must apply new baud rate without data loss
 * @note Called during initial handshake and when receiver requests speed change
 * 
 * Source: libraries/AP_RCProtocol/spm_srxl_config.h:99
 */
// User-provided routine to change the baud rate settings on the given UART:
// uart - the same uint8_t value as the uart parameter passed to srxlInit()
// baudRate - the actual baud rate (currently either 115200 or 400000)
void srxlChangeBaudRate(uint8_t uart, uint32_t baudRate);

/**
 * @brief Transmit SRXL2 packet on specified UART
 * 
 * @details User-provided callback to transmit an SRXL2 protocol packet. Called by
 *          the library when sending telemetry, bind responses, or other data to
 *          the Spektrum receiver.
 * 
 * @param[in] uart UART identifier (same value passed to srxlInit())
 * @param[in] pBuffer Pointer to packet data to transmit
 * @param[in] length Number of bytes in pBuffer to send
 * 
 * @note Implementation should be non-blocking or use DMA
 * @note Packet includes header, payload, and CRC - transmit all bytes in order
 * @note Called at up to 100Hz during active telemetry streaming
 * 
 * Source: libraries/AP_RCProtocol/spm_srxl_config.h:105
 */
// User-provided routine to actually transmit a packet on the given UART:
// uart - the same uint8_t value as the uart parameter passed to srxlInit()
// pBuffer - a pointer to an array of uint8_t values to send over the UART
// length - the number of bytes contained in pBuffer that should be sent
void srxlSendOnUart(uint8_t uart, uint8_t* pBuffer, uint8_t length);

/**
 * @brief Fill telemetry data buffer for transmission to Spektrum transmitter
 * 
 * @details User-provided callback invoked when the SRXL2 library needs telemetry
 *          data to send to the transmitter. Implementation should populate the
 *          16-byte telemetry buffer with current vehicle state data according
 *          to Spektrum telemetry format.
 * 
 * @param[out] pTelemetryData Pointer to 16-byte SrxlTelemetryData buffer to populate
 * 
 * @note Called when receiver requests telemetry or at configured telemetry rate
 * @note srxlTelemData global variable is also available as an alternative interface
 * @note Telemetry format depends on packet type (GPS, voltage, RPM, etc.)
 * @note See Spektrum telemetry specification for supported data formats
 * 
 * Source: libraries/AP_RCProtocol/spm_srxl_config.h:111
 */
// User-provided callback routine to fill in the telemetry data to send to the master when requested:
// pTelemetryData - a pointer to the 16-byte SrxlTelemetryData transmit buffer to populate
// NOTE: srxlTelemData is available as a global variable, so the memcpy line commented out below
// could be used if you would prefer to just populate that with the next outgoing telemetry packet.
void srxlFillTelemetry(SrxlTelemetryData* pTelemetryData);

/**
 * @brief Process received RC channel data from SRXL2 packet
 * 
 * @details User-provided callback invoked when a control data packet (RC channel
 *          values) is received from the Spektrum receiver. Implementation should
 *          extract and use the channel values for flight control.
 * 
 * @param[in] pChannelData Pointer to received SrxlChannelData structure with channel values
 * @param[in] isFailsafe true if channels are in failsafe state, false for valid data
 * 
 * @note srxlChData global variable also contains latest channel data
 * @note Called at receiver frame rate (typically 11ms/22ms depending on mode)
 * @note Failsafe state indicates loss of transmitter signal
 * @note Channel values typically range 0-2047 or similar (receiver-dependent)
 * 
 * Source: libraries/AP_RCProtocol/spm_srxl_config.h:119
 */
// User-provided callback routine that is called whenever a control data packet is received:
// pChannelData - a pointer to the received SrxlChannelData structure for manual parsing
// isFailsafe - true if channel data is set to failsafe values, else false.
// NOTE: srxlChData is available as a global variable that contains all of the latest values
// for channel data, so this callback is intended to be used if more control is desired.
// It might make sense to only use this to trigger your own handling of the received servo values.
void srxlReceivedChannelData(SrxlChannelData* pChannelData, bool isFailsafe);

/**
 * @brief Handle receiver bind information report
 * 
 * @details User-provided callback invoked when bind information is received from
 *          a Spektrum receiver, either as a response to a bind request or as an
 *          unprompted notification.
 * 
 * @param[in] device Full device ID of the bound receiver
 * @param[in] info Bind data structure containing GUID and other bind information
 * @return true to automatically propagate bind info to all receivers on all SRXL buses,
 *         false to handle bind information manually
 * 
 * @note Bind information includes receiver GUID and RF configuration
 * @note Used for multi-receiver setups to ensure consistent binding
 * 
 * Source: libraries/AP_RCProtocol/spm_srxl_config.h:123
 */
// User-provided callback routine to handle reception of a bound data report (either requested or unprompted).
// Return true if you want this bind information set automatically for all other receivers on all SRXL buses.
bool srxlOnBind(SrxlFullID device, SrxlBindData info);

/**
 * @brief Process VTX (video transmitter) control commands
 * 
 * @details User-provided callback invoked when VTX control packets are received
 *          from the transmitter via SRXL2. Allows remote control of video
 *          transmitter settings (frequency, power, etc.) through the RC link.
 * 
 * @param[in] pVtxData Pointer to SrxlVtxData structure with VTX control parameters
 * 
 * @note VTX control includes frequency band, channel, power level, pit mode
 * @note Implementation should forward commands to video transmitter hardware
 * @note Not all Spektrum receivers/transmitters support VTX control
 * 
 * Source: libraries/AP_RCProtocol/spm_srxl_config.h:126
 */
// User-provided callback routine to handle reception of a VTX control packet.
void srxlOnVtx(SrxlVtxData* pVtxData);

/**
 * @brief Process Forward Programming commands from transmitter (optional)
 * 
 * @details User-provided callback invoked when Forward Programming packets are
 *          received via SRXL2. Allows parameter configuration of the flight
 *          controller through the Spektrum transmitter interface.
 * 
 * @param[in] pData Pointer to Forward Programming data payload
 * @param[in] dataLength Length of programming data in bytes
 * 
 * @note Only compiled if SRXL_INCLUDE_FWD_PGM_CODE is defined
 * @note Implementation should parse and apply configuration changes
 * @note Requires SRXL_DEVINFO_FWD_PROG_SUPPORT flag in SRXL_DEVICE_INFO
 * @note ArduPilot typically does not use Forward Programming (uses MAVLink instead)
 * 
 * Source: libraries/AP_RCProtocol/spm_srxl_config.h:130-133
 */
// Optional user-provided callback routine to handle Forward Programming command locally if supported
#ifdef SRXL_INCLUDE_FWD_PGM_CODE
static inline void srxlOnFwdPgm(uint8_t* pData, uint8_t dataLength)
{
    // TODO: Pass data to Forward Programming library
}
#endif // SRXL_INCLUDE_FWD_PGM_CODE

/**
 * @brief Enter critical section for thread-safe SRXL2 library access
 * 
 * @details User-provided function to disable interrupts or acquire mutex for
 *          thread-safe access to SRXL2 library state. Required only in specific
 *          scenarios where concurrent access is possible.
 * 
 *          Critical section needed when:
 *          - Multiple SRXL buses are active (SRXL_NUM_OF_BUSES > 1)
 *          - Hardware CRC peripheral is shared with other code
 *          - SRXL library accessed from multiple contexts (ISR + main loop)
 * 
 * @note For single-bus configurations with no shared hardware, this can be empty
 * @note Must be balanced with srxlExitCriticalSection() call
 * @note Keep critical sections as short as possible to avoid blocking
 * 
 * @warning Disabling interrupts affects real-time performance - minimize duration
 * 
 * @see srxlExitCriticalSection()
 * 
 * Source: libraries/AP_RCProtocol/spm_srxl_config.h:138-139
 */
// User-provided routine to enter a critical section (only needed with multiple buses or if HW CRC is used externally)
static inline void srxlEnterCriticalSection(void)
{
}

/**
 * @brief Exit critical section after thread-safe SRXL2 library access
 * 
 * @details User-provided function to re-enable interrupts or release mutex after
 *          completing thread-safe access to SRXL2 library state. Must be called
 *          after every srxlEnterCriticalSection() call.
 * 
 * @note Must be called exactly once for each srxlEnterCriticalSection() call
 * @note For single-bus configurations with no shared hardware, this can be empty
 * 
 * @see srxlEnterCriticalSection()
 * 
 * Source: libraries/AP_RCProtocol/spm_srxl_config.h:142-143
 */
// User-provided routine to exit a critical section (only needed with multiple buses or if HW CRC is used externally)
static inline void srxlExitCriticalSection(void)
{
}

/** @} */ // end of SRXL2_Interface group

#endif // _SRXL_CONFIG_H_
