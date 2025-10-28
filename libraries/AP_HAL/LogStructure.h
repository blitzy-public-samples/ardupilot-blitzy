/**
 * @file LogStructure.h
 * @brief Binary log message structure definitions for Hardware Abstraction Layer
 * 
 * @details This file defines log message structures used by the HAL to record
 *          hardware-level events and statistics. These structures are part of
 *          ArduPilot's binary logging system, which records flight data to
 *          non-volatile storage for post-flight analysis and debugging.
 * 
 * Binary Log Format Overview:
 * ---------------------------
 * ArduPilot uses a compact binary format for efficient logging on embedded systems.
 * Each log message consists of:
 * - Message header (LOG_PACKET_HEADER): message ID and timestamp
 * - Fixed-size data fields: defined by the structure
 * 
 * The binary format is described by three parallel definitions:
 * 1. Structure definition (e.g., log_UART): C struct with actual data fields
 * 2. Format string: defines field types for parsing ("QBfff")
 * 3. Field labels: human-readable names ("TimeUS,I,Tx,Rx,RxDp")
 * 
 * Field Type Codes:
 * -----------------
 * Common type codes used in format strings:
 * - 'Q' = uint64_t (typically microseconds timestamp)
 * - 'I' = uint32_t (unsigned integer)
 * - 'i' = int32_t (signed integer)
 * - 'H' = uint16_t (unsigned short)
 * - 'h' = int16_t (signed short)
 * - 'B' = uint8_t (unsigned byte)
 * - 'b' = int8_t (signed byte)
 * - 'f' = float (32-bit floating point)
 * - 'd' = double (64-bit floating point)
 * - 'n' = char[4] (4-character string)
 * - 'N' = char[16] (16-character string)
 * - 'Z' = char[64] (64-character string)
 * - 'L' = int32_t latitude/longitude (degrees * 1e7)
 * - 'e' = int32_t latitude/longitude (degrees * 1e7, deprecated)
 * - 'E' = uint32_t (deprecated)
 * - 'M' = uint8_t flight mode
 * - 'c' = int16_t centi-degrees (0.01 degree)
 * - 'C' = uint16_t centi-degrees (0.01 degree)
 * 
 * Format Multiplier and Unit Codes:
 * ----------------------------------
 * Each field has associated unit and multiplier codes in the LOG_STRUCTURE macro:
 * - Units (4th parameter): "s#BBB" where:
 *   - 's' = seconds
 *   - '#' = instance/index (dimensionless)
 *   - 'B' = bytes or bytes/second
 *   - Other common: 'm' (meters), 'd' (degrees), 'A' (amperes), 'V' (volts)
 * - Multipliers (5th parameter): "F----" where:
 *   - 'F' = timestamp field (special handling)
 *   - '-' = no multiplier (1:1)
 *   - '0' = multiply by 10^0 = 1
 *   - '2' = multiply by 10^2 = 100
 *   - '?' = lat/lon (special handling)
 * 
 * LOG_PACKET_HEADER Macro:
 * ------------------------
 * Defined in AP_Logger/LogStructure.h, provides:
 * - uint8_t msg_type: Message ID (LOG_*_MSG constant)
 * - Timestamp handling varies by implementation
 * 
 * Relationship to DataFlash Format:
 * ----------------------------------
 * This format evolved from the DataFlash logging system and maintains
 * compatibility with existing log analysis tools (MAVExplorer, UAVLogViewer).
 * The format is designed for:
 * - Minimal storage overhead (binary packing)
 * - Fast writing on embedded systems (no serialization)
 * - Efficient reading with memory-mapped access
 * - Cross-platform compatibility (little-endian assumed)
 * 
 * Log Replay Compatibility:
 * -------------------------
 * Structures defined here MUST maintain binary compatibility for log replay:
 * - DO NOT change field order in existing structures
 * - DO NOT change field types in existing structures
 * - DO NOT remove fields from existing structures
 * - New fields can be added at the end with version increment
 * - New message types can be added with new message IDs
 * 
 * The AP_DAL (Data Abstraction Layer) uses these structures for deterministic
 * replay of logged data to reproduce flight behavior for debugging.
 * 
 * @note This file contains only HAL-level log structure definitions
 * @note Vehicle-specific and library-specific log structures are defined in
 *       their respective LogStructure.h files (e.g., ArduCopter/LogStructure.h)
 * @note See AP_Logger/LogStructure.h for the base logging infrastructure
 * @note See AP_Logger/README.md for comprehensive logging system documentation
 * 
 * @warning Modifying existing log structures breaks log compatibility and replay
 * @warning Always use PACKED attribute to ensure cross-platform binary compatibility
 * 
 * @see AP_Logger for the logging subsystem implementation
 * @see AP_DAL for the Data Abstraction Layer used in log replay
 * 
 * Source: libraries/AP_HAL/LogStructure.h
 */

#pragma once

#include <AP_Logger/LogStructure.h>
#include "UARTDriver.h"

/**
 * @brief Log message IDs exported by the Hardware Abstraction Layer
 * 
 * @details This macro defines the list of log message IDs that are provided
 *          by the HAL subsystem. It is used by the main logging system to
 *          enumerate all available message types.
 * 
 *          Message IDs are unique identifiers (uint8_t) assigned to each log
 *          structure type. The ID is stored in the message header and used
 *          by log parsers to identify the structure format.
 * 
 * @note This macro is included in the master LOG_BASE_STRUCTURES list in
 *       AP_Logger/LogStructure.h to register HAL message types
 * @note Each message ID constant (LOG_UART_MSG, etc.) must be unique across
 *       the entire system to avoid parsing conflicts
 * 
 * @see LOG_STRUCTURE_FROM_HAL for the corresponding structure definitions
 */
#define LOG_IDS_FROM_HAL \
    LOG_UART_MSG

/**
 * @struct log_UART
 * @brief Binary log structure for UART hardware statistics
 * 
 * @details Records UART (serial port) performance metrics including data rates
 *          and dropped data. This information is critical for diagnosing
 *          communication issues, buffer overruns, and telemetry link quality.
 * 
 *          Logged periodically (typically 1-5 Hz) for each active UART instance.
 *          High rx_drop_rate values indicate the driver cannot keep up with
 *          incoming data, suggesting CPU overload or misconfigured baud rates.
 * 
 * Binary Format: LOG_UART_MSG (message ID defined in AP_Logger/LogStructure.h)
 * Format String: "QBfff" = [uint64_t, uint8_t, float, float, float]
 * Field Labels: "TimeUS,I,Tx,Rx,RxDp"
 * Units: "s#BBB" = [seconds, instance, bytes/sec, bytes/sec, bytes/sec]
 * Multipliers: "F----" = [timestamp, none, none, none, none]
 * 
 * @note PACKED attribute ensures binary compatibility across platforms
 * @note Only logged when HAL_UART_STATS_ENABLED is true (enabled by default)
 * @note See UARTDriver::log_stats() for logging implementation
 * 
 * @warning Do not modify field order or types - breaks log replay compatibility
 * 
 * @see AP_HAL::UARTDriver for UART statistics collection
 * @see AP_Logger for logging subsystem
 */
// @LoggerMessage: UART
// @Description: UART stats
// @Field: TimeUS: Time since system startup
// @Field: I: instance
// @Field: Tx: transmitted data rate bytes per second
// @Field: Rx: received data rate bytes per second, this is all incoming data, it may not all be processed by the driver using this port.
// @Field: RxDp: Data rate of dropped received bytes, ideally should be 0. This is the difference between the received data rate and the processed data rate.
struct PACKED log_UART {
    LOG_PACKET_HEADER;          ///< Standard log message header with msg_type and timestamp
    uint64_t time_us;           ///< Timestamp in microseconds since system boot
    uint8_t instance;           ///< UART instance number (0=Serial0, 1=Serial1, etc.)
    float tx_rate;              ///< Transmit data rate in bytes per second (outgoing data)
    float rx_rate;              ///< Receive data rate in bytes per second (all incoming data)
    float rx_drop_rate;         ///< Dropped receive data rate in bytes per second (should be 0)
};

/**
 * @brief Log structure definitions exported by the Hardware Abstraction Layer
 * 
 * @details This macro provides the complete log structure metadata required by
 *          the logging system to write and parse log messages. Each entry is
 *          a LogStructure initializer containing:
 * 
 *          1. Message ID (LOG_UART_MSG): Unique identifier for this message type
 *          2. Structure size (sizeof(log_UART)): Total bytes for validation
 *          3. Message name ("UART"): Human-readable identifier for log viewers
 *          4. Format string ("QBfff"): Field types for binary parsing
 *          5. Field labels ("TimeUS,I,Tx,Rx,RxDp"): Column names for display
 *          6. Unit codes ("s#BBB"): Physical units for each field
 *          7. Multiplier codes ("F----"): Scaling factors for each field
 * 
 *          The structure metadata is compiled into the LogStructure table
 *          in AP_Logger and is also embedded in log files for self-description.
 * 
 * Conditional Compilation:
 * ------------------------
 * When HAL_UART_STATS_ENABLED is disabled, this macro expands to empty,
 * removing UART statistics logging to save flash space and CPU time on
 * resource-constrained boards.
 * 
 * Format String Details:
 * ----------------------
 * "QBfff" breaks down as:
 * - Q: uint64_t time_us (timestamp in microseconds)
 * - B: uint8_t instance (UART instance number 0-255)
 * - f: float tx_rate (transmit bytes per second)
 * - f: float rx_rate (receive bytes per second)
 * - f: float rx_drop_rate (dropped bytes per second)
 * 
 * Unit String Details:
 * --------------------
 * "s#BBB" indicates:
 * - s: seconds (time_us displayed as seconds with microsecond precision)
 * - #: dimensionless instance number
 * - B: bytes per second for tx_rate
 * - B: bytes per second for rx_rate
 * - B: bytes per second for rx_drop_rate
 * 
 * Multiplier String Details:
 * --------------------------
 * "F----" indicates:
 * - F: timestamp field (special formatting in log viewers)
 * - -: no multiplier for instance (raw value)
 * - -: no multiplier for tx_rate (raw value)
 * - -: no multiplier for rx_rate (raw value)
 * - -: no multiplier for rx_drop_rate (raw value)
 * 
 * @note This macro is included in the master log structure table in
 *       AP_Logger/LogStructure.h via LOG_COMMON_STRUCTURES
 * @note Conditional compilation allows disabling features on memory-constrained boards
 * @note Structure size is validated at runtime to catch packing issues
 * 
 * @warning Modifying format, labels, units, or multipliers breaks compatibility
 *          with existing log analysis tools
 * @warning Format string length must match number of fields in structure
 * @warning Label count must match format string character count
 * 
 * @see LogStructure in AP_Logger/LogStructure.h for the base structure type
 * @see AP_Logger::Log_Write() for message writing implementation
 */
#if HAL_UART_STATS_ENABLED
#define LOG_STRUCTURE_FROM_HAL                          \
    { LOG_UART_MSG, sizeof(log_UART),                   \
      "UART","QBfff","TimeUS,I,Tx,Rx,RxDp", "s#BBB", "F----" },
#else
#define LOG_STRUCTURE_FROM_HAL
#endif
