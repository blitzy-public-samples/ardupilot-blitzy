/**
 * @file LogStructure.h
 * @brief Central repository for all binary log message format definitions, units, and multipliers used across ArduPilot
 * 
 * @details This header file defines the complete binary logging infrastructure for ArduPilot, including:
 *          - Format character mappings that specify C types for binary log encoding
 *          - Unit definitions (UnitStructure and log_Units[]) for physical quantities
 *          - Multiplier definitions (MultiplierStructure and log_Multipliers[]) for value scaling
 *          - Log message structure definitions for all common vehicle types
 *          - LOG_PACKET_HEADER macros for binary message framing
 *          
 *          This file serves as the authoritative reference for the binary log format used by
 *          AP_Logger across all vehicle types (Copter, Plane, Rover, Sub, Blimp, AntennaTracker).
 *          
 *          Binary log messages consist of:
 *          1. 3-byte header (HEAD_BYTE1, HEAD_BYTE2, msgid)
 *          2. Message payload as defined by LogStructure
 *          3. Format determined by format character string
 *          
 *          Ground control stations and log analysis tools rely on the format definitions in this
 *          file to correctly decode binary log data.
 * 
 * @note All log structures are conditionally PACKED on ChibiOS platforms to save flash space
 * @warning Maintaining backward compatibility is critical when modifying this file. Changes to
 *          format characters, units, or multipliers can break log decoding in existing tools.
 *          Always increment log format version when making incompatible changes.
 * 
 * @see libraries/AP_Logger/README.md for usage examples and complete format documentation
 * @see Tools/scripts/build_log_message_documentation.sh for auto-generated documentation
 * 
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

#include <AP_Common/AP_Common.h>

// if you add any new types, units or multipliers, please update README.md

/**
 * @brief Format character mappings for binary log message encoding
 * 
 * @details Format characters define the C storage type for each field in a binary log message.
 *          These characters appear in the LogStructure.format string and determine how data
 *          is serialized to the binary log file and deserialized by ground control stations.
 *          
 *          Each format character maps directly to a C type with specific byte width:
 *          
 *          Basic Integer Types (signed):
 *            b : int8_t           - 1 byte signed integer (-128 to 127)
 *            h : int16_t          - 2 byte signed integer (-32768 to 32767)
 *            i : int32_t          - 4 byte signed integer
 *            q : int64_t          - 8 byte signed integer
 *          
 *          Basic Integer Types (unsigned):
 *            B : uint8_t          - 1 byte unsigned integer (0 to 255)
 *            H : uint16_t         - 2 byte unsigned integer (0 to 65535)
 *            I : uint32_t         - 4 byte unsigned integer
 *            Q : uint64_t         - 8 byte unsigned integer (typically timestamps)
 *          
 *          Floating Point Types:
 *            f : float            - 4 byte IEEE 754 single precision
 *            d : double           - 8 byte IEEE 754 double precision
 *          
 *          String/Array Types:
 *            n : char[4]          - 4 character fixed-length string
 *            N : char[16]         - 16 character fixed-length string
 *            Z : char[64]         - 64 character fixed-length string
 *            a : int16_t[32]      - Array of 32 signed 16-bit integers
 *          
 *          Scaled Integer Types (automatically scaled by 100x for precision):
 *            c : int16_t * 100    - Centiscaled signed 16-bit (e.g., centidegrees)
 *            C : uint16_t * 100   - Centiscaled unsigned 16-bit
 *            e : int32_t * 100    - Centiscaled signed 32-bit
 *            E : uint32_t * 100   - Centiscaled unsigned 32-bit
 *          
 *          Special Purpose Types:
 *            L : int32_t          - Latitude/longitude in degrees * 1e7 (WGS84)
 *            M : uint8_t          - Vehicle-specific flight mode enumeration
 *          
 * @note The 'c', 'C', 'e', 'E' types automatically imply 100x scaling for storage efficiency
 *       while maintaining decimal precision. For example, a value of 45.67 degrees is stored
 *       as 4567 using type 'c'.
 * 
 * @note Type 'L' is specifically for GPS coordinates using the standard 1e7 scaling factor
 *       (e.g., latitude 37.7749° is stored as 377749000).
 * 
 * @note Type 'M' values are vehicle-specific. The same numeric value may represent different
 *       modes in Copter vs Plane vs Rover.
 * 
 * @warning Format characters define binary layout. Changing a format character for an existing
 *          log message type will break backward compatibility with all existing log files.
 *          Never modify existing format strings; instead, create a new message type.
 * 
 * @see LogStructure struct for how format strings are used in message definitions
 */

/**
 * @struct UnitStructure
 * @brief Maps a single character identifier to a physical unit string
 * 
 * @details This structure defines the relationship between a character code (ID) and its
 *          corresponding physical unit label. Unit characters appear in LogStructure.units
 *          strings, with each character corresponding to a field in the log message.
 *          
 *          Ground control stations use this mapping to display logged values with correct
 *          units and perform unit conversions when presenting data to users.
 *          
 *          Units are base SI units where possible, with common exceptions for user convenience
 *          (e.g., degrees instead of radians, Gauss instead of Tesla).
 * 
 * @note Unit definitions must remain consistent with Tools/autotest/param_metadata/param.py
 *       to ensure parameter documentation and log documentation use matching units.
 * 
 * @see log_Units[] for the complete array of unit definitions
 * @see LogStructure.units for how unit characters are assigned to message fields
 */
struct UnitStructure {
    const char ID;          ///< Single character unit identifier (e.g., 'm' for meters, 's' for seconds)
    const char *unit;       ///< Human-readable unit string (e.g., "m", "m/s", "deg", "A")
};

/**
 * @struct MultiplierStructure
 * @brief Maps a single character identifier to a numeric multiplier for value scaling
 * 
 * @details This structure defines the relationship between a character code (ID) and a numeric
 *          scaling factor. Multiplier characters appear in LogStructure.multipliers strings,
 *          with each character corresponding to a field in the log message.
 *          
 *          Multipliers are applied to the raw integer value stored in the log to obtain the
 *          actual floating-point value. This encoding technique saves space in binary logs
 *          while preserving precision.
 *          
 *          CRITICAL: The multiplier applies to the raw binary value ONLY. Any scaling implied
 *          by the format character itself (e.g., the *100 in 'c' for centidegrees) is handled
 *          separately and must NOT be inferred from the multiplier. The format character
 *          defines the C type, while the multiplier scales that value to physical units.
 * 
 * @note A multiplier of 0 (ID '-') indicates a non-numeric field such as a string
 * @note A multiplier of 1 (ID '0' or '?') indicates no scaling is applied
 * 
 * @warning Multipliers are independent of format field scaling. For example, format 'h'
 *          (int16_t) and format 'c' (int16_t*100) are both just int16_t types for multiplier
 *          purposes. Ground stations must NOT assume any relationship between the format
 *          character scaling and the multiplier value.
 * 
 * @see log_Multipliers[] for the complete array of multiplier definitions
 * @see LogStructure.multipliers for how multiplier characters are assigned to message fields
 */
struct MultiplierStructure {
    const char ID;              ///< Single character multiplier identifier (e.g., '0' for 1.0, 'B' for 1e-2)
    const double multiplier;    ///< Numeric scaling factor to apply to raw logged value
};

/**
 * @brief Complete array of physical unit definitions for binary log messages
 * 
 * @details This array maps single-character unit identifiers to human-readable unit strings.
 *          Each entry defines one physical unit that can be referenced in LogStructure.units
 *          to specify what physical quantity a log field represents.
 *          
 *          Design Philosophy:
 *          - Use base SI units where practical (meters, seconds, amperes, etc.)
 *          - Use derived units for user convenience (degrees instead of radians, Gauss instead of Tesla)
 *          - Compound units use explicit notation (e.g., "m/s" not "mps", "A.s" not "As")
 *          - Battery capacity is "A.s" (ampere-seconds) not "Ah" to maintain base unit consistency
 *          
 *          Unit Consistency Requirements:
 *          1. Unit names must match those in Tools/autotest/param_metadata/param.py to ensure
 *             parameter documentation and log documentation use identical unit terminology
 *          2. Once defined, unit IDs should never be changed as this breaks existing log analysis tools
 *          3. New units should be added at the end of appropriate sections to maintain stability
 *          
 *          Special Characters:
 *          '-' : No units (dimensionless quantities like Pi, ratios, or string fields)
 *          '?' : Unknown units (placeholder for fields not yet analyzed)
 *          '#' : Instance number (e.g., sensor instance 0, 1, 2...)
 *          '%' : Percentage (0-100)
 * 
 * @note Some units are not strict SI (degrees, Gauss, degC, rpm, ppm) but are retained for
 *       user-friendliness. Conversions: 1 Tesla = 10000 Gauss, 0°C = 273.15K, 1 rpm = 1/60 Hz
 * 
 * @warning Never change existing unit IDs or their meanings. This will break all existing ground
 *          control stations and log analysis tools. Only add new units; never modify existing ones.
 * 
 * @see UnitStructure for the structure definition
 * @see LogStructure.units for how these units are applied to log message fields
 * @see Tools/autotest/param_metadata/param.py for parameter metadata unit definitions
 */
// all units here should be base units
// This does mean battery capacity is here as "amp*second"
// Please keep the names consistent with Tools/autotest/param_metadata/param.py:33
const struct UnitStructure log_Units[] = {
    { '-', "" },              // no units e.g. Pi, or a string
    { '?', "UNKNOWN" },       // Units which haven't been worked out yet....
    { 'A', "A" },             // Ampere
    { 'a', "Ah" },            // Ampere hours
    { 'd', "deg" },           // of the angular variety, -180 to 180
    { 'b', "B" },             // bytes
    { 'B', "B/s" },           // bytes per second
    { 'k', "deg/s" },         // degrees per second. Degrees are NOT SI, but is some situations more user-friendly than radians
    { 'D', "deglatitude" },   // degrees of latitude
    { 'e', "deg/s/s" },       // degrees per second per second. Degrees are NOT SI, but is some situations more user-friendly than radians
    { 'E', "rad/s" },         // radians per second
    { 'G', "Gauss" },         // Gauss is not an SI unit, but 1 tesla = 10000 gauss so a simple replacement is not possible here
    { 'h', "degheading" },    // 0.? to 359.?
    { 'i', "A.s" },           // Ampere second
    { 'J', "W.s" },           // Joule (Watt second)
    { 'l', "l" },             // litres
    { 'L', "rad/s/s" },       // radians per second per second
    { 'm', "m" },             // metres
    { 'n', "m/s" },           // metres per second
    // { 'N', "N" },          // Newton
    { 'o', "m/s/s" },         // metres per second per second
    { 'O', "degC" },          // degrees Celsius. Not SI, but Kelvin is too cumbersome for most users
    { '%', "%" },             // percent
    { 'S', "satellites" },    // number of satellites
    { 's', "s" },             // seconds
    { 't', "N.m" },           // Newton meters, torque
    { 'q', "rpm" },           // rounds per minute. Not SI, but sometimes more intuitive than Hertz
    { 'r', "rad" },           // radians
    { 'U', "deglongitude" },  // degrees of longitude
    { 'u', "ppm" },           // pulses per minute
    { 'v', "V" },             // Volt
    { 'P', "Pa" },            // Pascal
    { 'w', "Ohm" },           // Ohm
    { 'W', "Watt" },          // Watt
    { 'X', "W.h" },           // Watt hour
    { 'y', "l/s" },           // litres per second
    { 'Y', "us" },            // pulse width modulation in microseconds
    { 'z', "Hz" },            // Hertz
    { '#', "instance" }       // (e.g.)Sensor instance number
};

/**
 * @brief Complete array of numeric multiplier definitions for value scaling in binary logs
 * 
 * @details This array maps single-character multiplier identifiers to numeric scaling factors.
 *          Each entry defines a multiplier that can be referenced in LogStructure.multipliers
 *          to specify how to scale raw binary values to their true physical values.
 *          
 *          CRITICAL CONCEPT - Format vs Multiplier Independence:
 *          =====================================================
 *          The multiplier applies ONLY to the raw value present in the binary log file.
 *          Any scaling implied by the format character itself is COMPLETELY IGNORED for
 *          multiplier purposes.
 *          
 *          Example: Format 'c' means "int16_t * 100 (centiscaled)" and format 'h' means
 *          "int16_t". However, for multiplier calculation purposes, BOTH are treated as
 *          plain int16_t. The format character defines the C storage type; the multiplier
 *          scales that stored value to physical units.
 *          
 *          Ground Control Station Decoding Algorithm:
 *          1. Read raw bytes from log according to format character → get raw_value
 *          2. Apply multiplier: physical_value = raw_value * multiplier
 *          3. Ignore any textual hints in unit names (e.g., "centidegrees")
 *          4. The format 'c' vs 'h' distinction is ONLY about what bytes to read, not scaling
 *          
 *          Why This Matters:
 *          A GCS must NOT attempt to infer scaling from unit names like "centidegrees" or
 *          from format character annotations. The multiplier is the ONLY authoritative source
 *          for value scaling. This prevents ambiguity and ensures consistent decoding across
 *          all ground control stations and log analysis tools.
 *          
 *          Multiplier Categories:
 *          - Power-of-10 scaling: '2'=1e2, '1'=1e1, '0'=1e0, 'A'=1e-1, 'B'=1e-2, ... 'I'=1e-9
 *          - Special conversions: '!'=3.6 (for mAh and km/h), '/'=3600 (for Ah)
 *          - No scaling: '-'=0 (strings), '?'=1 (unknown/unscaled)
 * 
 * @note Multiplier '0' (ID '0') represents 1.0, meaning no scaling applied
 * @note Multiplier '-' (ID '-') represents 0, used for non-numeric fields like strings
 * @note Multiplier '?' (ID '?') represents 1.0, placeholder for fields not yet analyzed
 * 
 * @warning The format character and multiplier are orthogonal concepts. Never attempt to
 *          derive one from the other. They serve different purposes: format defines binary
 *          layout, multiplier defines mathematical scaling.
 * 
 * @warning Never change existing multiplier IDs or their values. This will break decoding
 *          of all existing log files in ground control stations and analysis tools.
 * 
 * @see MultiplierStructure for the structure definition
 * @see LogStructure.multipliers for how these multipliers are applied to log message fields
 */
// this multiplier information applies to the raw value present in the
// log.  Any adjustment implied by the format field (e.g. the "centi"
// in "centidegrees" is *IGNORED* for the purposes of scaling.
// Essentially "format" simply tells you the C-type, and format-type h
// (int16_t) is equivalent to format-type c (int16_t*100)
// tl;dr a GCS shouldn't/mustn't infer any scaling from the unit name

const struct MultiplierStructure log_Multipliers[] = {
    { '-', 0 },       // no multiplier e.g. a string
    { '?', 1 },       // multipliers which haven't been worked out yet....
// <leave a gap here, just in case....>
    { '2', 1e2 },
    { '1', 1e1 },
    { '0', 1e0 },
    { 'A', 1e-1 },
    { 'B', 1e-2 },
    { 'C', 1e-3 },
    { 'D', 1e-4 },
    { 'E', 1e-5 },
    { 'F', 1e-6 },
    { 'G', 1e-7 },
    { 'I', 1e-9 },
// <leave a gap here, just in case....>
    { '!', 3.6 }, // (ampere*second => milliampere*hour) and (km/h => m/s)
    { '/', 3600 }, // (ampere*second => ampere*hour)
};

/**
 * @brief Macros defining the standard 3-byte header present in all binary log messages
 * 
 * @details Every binary log message begins with a 3-byte header consisting of two magic bytes
 *          for message synchronization and one message ID byte for message type identification.
 *          
 *          Header Structure (3 bytes total):
 *          Byte 0: head1  - First magic byte (HEAD_BYTE1 = 0xA3 = 163 decimal)
 *          Byte 1: head2  - Second magic byte (HEAD_BYTE2 = 0x95 = 149 decimal)
 *          Byte 2: msgid  - Message type identifier (0-255, see LogMessages enum)
 *          
 *          The magic bytes (0xA3, 0x95) serve two purposes:
 *          1. Message synchronization: Allow recovery from corrupted log data by searching
 *             for the magic byte sequence
 *          2. Format validation: Confirm log file is ArduPilot binary format
 *          
 *          LOG_PACKET_HEADER Macro Usage:
 *          Used as the first member in every log message structure to ensure consistent
 *          binary layout. Example:
 *          
 *          @code
 *          struct PACKED log_Example {
 *              LOG_PACKET_HEADER;    // Expands to: uint8_t head1, head2, msgid;
 *              uint64_t time_us;
 *              float value;
 *          };
 *          @endcode
 *          
 *          LOG_PACKET_HEADER_INIT Macro Usage:
 *          Used in message initialization to set header bytes. Example:
 *          
 *          @code
 *          log_Example msg = {
 *              LOG_PACKET_HEADER_INIT(LOG_EXAMPLE_MSG),  // Sets head1, head2, msgid
 *              .time_us = AP_HAL::micros64(),
 *              .value = 1.23f
 *          };
 *          @endcode
 * 
 * @note These are implemented as macros (not inline functions or structs) due to limitations
 *       in g++ named member structure initialization. The macro expansion is required for
 *       correct aggregate initialization syntax.
 * 
 * @note LOG_PACKET_HEADER_LEN defines the header size for log reading/writing algorithms
 *       that need to know how many bytes to skip when seeking to message payloads.
 * 
 * @warning The magic bytes (HEAD_BYTE1, HEAD_BYTE2) must NEVER be changed. All existing
 *          ArduPilot logs, ground control stations, and analysis tools depend on these
 *          specific values for log format identification.
 * 
 * @see HEAD_BYTE1 for first magic byte definition
 * @see HEAD_BYTE2 for second magic byte definition
 * @see LogMessages enum for message ID assignments
 */
/*
  unfortunately these need to be macros because of a limitation of
  named member structure initialisation in g++
 */
#define LOG_PACKET_HEADER	       uint8_t head1, head2, msgid;
#define LOG_PACKET_HEADER_INIT(id) head1 : HEAD_BYTE1, head2 : HEAD_BYTE2, msgid : id
#define LOG_PACKET_HEADER_LEN 3 // bytes required for LOG_PACKET_HEADER

/**
 * @brief Magic byte values for binary log message synchronization and identification
 * 
 * @details HEAD_BYTE1 and HEAD_BYTE2 are two-byte magic number sequence that identifies
 *          ArduPilot binary log messages. These bytes appear at the start of every log
 *          message in the binary log file.
 *          
 *          Magic Byte Sequence: 0xA3 0x95 (decimal 163 149)
 *          
 *          Purpose and Usage:
 *          1. **Log Format Identification**: Allows tools to distinguish ArduPilot binary
 *             logs from other file formats by checking for this signature
 *          
 *          2. **Message Synchronization**: When reading corrupted or partially damaged log
 *             files, parsers can scan for this byte sequence to re-synchronize and resume
 *             decoding from the next valid message
 *          
 *          3. **Corruption Detection**: Unexpected values in head1/head2 indicate log
 *             corruption or seek to invalid file position
 *          
 *          Selection Rationale:
 *          The values 0xA3 and 0x95 were chosen to:
 *          - Have low probability of appearing in random data or other file formats
 *          - Be distinguishable from common ASCII text characters
 *          - Not conflict with other binary protocols used in ArduPilot systems
 *          
 *          Ground Control Station Usage:
 *          When reading binary logs, GCS software:
 *          1. Seeks to a position in the log file
 *          2. Reads two bytes and verifies they match HEAD_BYTE1 and HEAD_BYTE2
 *          3. If match: Reads msgid byte and continues decoding message
 *          4. If no match: Advances byte-by-byte searching for magic sequence
 * 
 * @note These values have been stable since early ArduPilot development and are deeply
 *       embedded in the entire logging ecosystem
 * 
 * @warning NEVER change these values under any circumstances. Changing them would:
 *          - Break all existing ground control stations (Mission Planner, QGroundControl, etc.)
 *          - Make all historical log files unreadable
 *          - Break all log analysis tools and scripts
 *          - Invalidate years of flight log archives
 *          This is one of the most critical backward-compatibility requirements in ArduPilot.
 * 
 * @see LOG_PACKET_HEADER macros for how these bytes are incorporated into message structures
 * @see AP_Logger::Log_Write() for where these bytes are written to log files
 */
// once the logging code is all converted we will remove these from
// this header
#define HEAD_BYTE1  0xA3    // Decimal 163
#define HEAD_BYTE2  0x95    // Decimal 149

#include <AP_Beacon/LogStructure.h>
#include <AP_DAL/LogStructure.h>
#include <AP_NavEKF2/LogStructure.h>
#include <AP_NavEKF3/LogStructure.h>
#include <AP_GPS/LogStructure.h>
#include <AP_NavEKF/LogStructure.h>
#include <AP_BattMonitor/LogStructure.h>
#include <AP_InertialSensor/LogStructure.h>
#include <AP_AHRS/LogStructure.h>
#include <AP_Camera/LogStructure.h>
#include <AP_Mount/LogStructure.h>
#include <AP_Baro/LogStructure.h>
#include <AP_CANManager/LogStructure.h>
#include <AP_VisualOdom/LogStructure.h>
#include <AC_PrecLand/LogStructure.h>
#include <AP_Proximity/LogStructure.h>
#include <AC_Avoidance/LogStructure.h>
#include <AP_ESC_Telem/LogStructure.h>
#include <AP_AIS/LogStructure.h>
#include <AP_HAL_ChibiOS/LogStructure.h>
#include <AP_RPM/LogStructure.h>
#include <AC_Fence/LogStructure.h>
#include <AP_Landing/LogStructure.h>
#include <AC_AttitudeControl/LogStructure.h>
#include <AP_HAL/LogStructure.h>
#include <AP_Mission/LogStructure.h>
#include <AP_Servo_Telem/LogStructure.h>

/**
 * @struct LogStructure
 * @brief Defines the complete format specification for a binary log message type
 * 
 * @details LogStructure is the master definition for each log message type in ArduPilot.
 *          It contains all metadata needed for ground control stations to decode binary
 *          log messages, including field types, names, units, and scaling factors.
 *          
 *          Each vehicle type and library defines an array of LogStructure entries describing
 *          all log messages it can produce. These arrays are typically defined using the
 *          LOG_COMMON_STRUCTURES macro and vehicle-specific structure definitions.
 *          
 *          LogStructure Lifecycle:
 *          1. **Compile Time**: Static arrays of LogStructure defined in code
 *          2. **Initialization**: AP_Logger reads structure arrays during startup
 *          3. **Log File**: FMT, FMTU, UNIT, MULT messages written to log describing formats
 *          4. **GCS Decoding**: Ground stations read format messages to decode subsequent data
 *          
 *          Field Correspondence:
 *          The format, labels, units, and multipliers strings must have matching lengths,
 *          with each character position corresponding to one field in the log message:
 *          
 *          @code
 *          // Example: BARO message with 3 data fields (after TimeUS)
 *          format:      "QBfff"     - Q (uint64_t TimeUS), B (uint8_t Instance), 
 *                                      f (float Alt), f (float Press), f (float Temp)
 *          labels:      "TimeUS,I,Alt,Press,Temp"  - Field names
 *          units:       "s#mPO"     - seconds, instance, meters, Pascals, degC
 *          multipliers: "F-000"     - No scaling except TimeUS gets 1e-6 multiplier
 *          @endcode
 * 
 * @note On ChibiOS platforms (ARM-based flight controllers), this structure is PACKED to
 *       save precious flash memory space. On SITL and other platforms, it remains unpacked
 *       for better performance and compatibility (especially Apple M1 CPUs which have
 *       alignment issues with packed structures).
 * 
 * @note The streaming flag indicates whether a message can be rate-limited during high-bandwidth
 *       operations like DataFlash-over-MAVLink. Critical messages (mode changes, events) have
 *       streaming=false to ensure they're never dropped.
 * 
 * @warning All four metadata strings (format, labels, units, multipliers) must have exactly
 *          matching field counts. A mismatch will cause log decoding errors in ground stations.
 * 
 * @warning msg_len must exactly match the binary size of the corresponding log message structure
 *          including the 3-byte LOG_PACKET_HEADER. Incorrect length causes log corruption.
 * 
 * @see LOG_COMMON_STRUCTURES for macro defining common message structures
 * @see log_Format struct for the FMT message that communicates this structure to GCS
 * @see AP_Logger::WriteBlock() for how structures are written to binary logs
 */
// structure used to define logging format
// It is packed on ChibiOS to save flash space; however, this causes problems
// when building the SITL on an Apple M1 CPU (and is also slower) so we do not
// pack it by default
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
struct PACKED LogStructure {
#else
struct LogStructure {
#endif
    uint8_t msg_type;           ///< Unique message type identifier (0-255, see LogMessages enum)
    uint8_t msg_len;            ///< Total message size in bytes including 3-byte header
    const char *name;           ///< 4-character message name (e.g., "BARO", "GPS", "IMU")
    const char *format;         ///< Format character string defining field C types
    const char *labels;         ///< Comma-separated field names for display in GCS
    const char *units;          ///< Unit character string (one char per field, see log_Units[])
    const char *multipliers;    ///< Multiplier character string (one char per field, see log_Multipliers[])
    bool streaming;             ///< true = can be rate limited; false = always sent immediately (critical messages)
};

/**
 * @brief Maximum string lengths for LogStructure fields, including null terminators
 * 
 * @details These constants define the buffer sizes required for storing LogStructure
 *          metadata strings. They represent the maximum allowable length for each field
 *          to prevent buffer overflows and ensure compatibility across the logging system.
 *          
 *          Size Rationale:
 *          - LS_NAME_SIZE (5): 4 characters for name + 1 null terminator
 *          - LS_FORMAT_SIZE (17): 16 format characters + 1 null terminator
 *          - LS_LABELS_SIZE (65): ~64 characters for comma-separated labels + 1 null terminator
 *          - LS_UNITS_SIZE (17): 16 unit characters + 1 null terminator
 *          - LS_MULTIPLIERS_SIZE (17): 16 multiplier characters + 1 null terminator
 *          
 *          The 16-character limit for format/units/multipliers corresponds to supporting
 *          log messages with up to 16 fields. Most messages have fewer fields, but complex
 *          messages (like PID controllers or sensor arrays) may approach this limit.
 *          
 *          Usage Example:
 *          @code
 *          char name_buffer[LS_NAME_SIZE];
 *          strncpy(name_buffer, log_structure.name, LS_NAME_SIZE - 1);
 *          name_buffer[LS_NAME_SIZE - 1] = '\0';  // Ensure null termination
 *          @endcode
 * 
 * @note These sizes include space for null terminators, so usable string length is SIZE - 1
 * @note Labels can be longer (65 chars) because they include comma separators and full field names
 * @note These limits are enforced during compile-time structure definition validation
 * 
 * @warning Exceeding these limits will cause buffer overflows in log reading/writing code.
 *          Always validate string lengths before creating new LogStructure definitions.
 * 
 * @see LogStructure for the structure these limits apply to
 */
// maximum lengths of fields in LogStructure, including trailing nulls
static const uint8_t LS_NAME_SIZE = 5;
static const uint8_t LS_FORMAT_SIZE = 17;
static const uint8_t LS_LABELS_SIZE = 65;
static const uint8_t LS_UNITS_SIZE = 17;
static const uint8_t LS_MULTIPLIERS_SIZE = 17;

/**
 * @brief Log structures common to all vehicle types
 * 
 * @details The following structures define binary log message formats that are written
 *          to DataFlash/SD card logs. These messages communicate metadata and data to
 *          ground control stations for flight analysis and debugging.
 */

/**
 * @struct log_Format
 * @brief FMT message: Defines the format of other log messages in the binary log file
 * 
 * @details The FMT (Format) message is written to the log file to describe the structure
 *          of every other message type that appears in the log. Ground control stations
 *          read these FMT messages first to understand how to decode subsequent data messages.
 *          
 *          FMT Message Lifecycle:
 *          1. **Log Start**: AP_Logger writes FMT messages for all message types at log initialization
 *          2. **Runtime**: Additional FMT messages may be written if new message types are registered
 *          3. **Log Reading**: GCS reads FMT messages to build decoding tables
 *          4. **Data Decoding**: GCS uses FMT definitions to parse all other messages
 *          
 *          Format String Encoding:
 *          The format field contains single characters representing C data types:
 *          - 'b' = int8_t, 'B' = uint8_t
 *          - 'h' = int16_t, 'H' = uint16_t
 *          - 'i' = int32_t, 'I' = uint32_t
 *          - 'q' = int64_t, 'Q' = uint64_t
 *          - 'f' = float, 'd' = double
 *          - 'n' = char[4], 'N' = char[16], 'Z' = char[64]
 *          - 'c' = int16_t*100, 'C' = uint16_t*100
 *          - 'e' = int32_t*100, 'E' = uint32_t*100
 *          - 'L' = int32_t lat/lon, 'M' = uint8_t flight mode
 *          
 *          Example FMT Message:
 *          @code
 *          // Describes a GPS message with format "QBIHBcLLeeee"
 *          log_Format fmt = {
 *              .head1 = HEAD_BYTE1,
 *              .head2 = HEAD_BYTE2,
 *              .msgid = LOG_FORMAT_MSG,
 *              .type = LOG_GPS_MSG,
 *              .length = sizeof(log_GPS),
 *              .name = "GPS",
 *              .format = "QBIHBcLLeeee",
 *              .labels = "TimeUS,Status,GMS,GWk,NSats,HDop,Lat,Lng,Alt,Spd,GCrs,VZ"
 *          };
 *          @endcode
 * 
 * @note This structure is PACKED to ensure binary layout matches on all platforms
 * @note Fixed array sizes (name[4], format[16], labels[64]) are sized for common use cases
 * @note First message in every ArduPilot log file is the FMT message describing itself
 * 
 * @warning Changing this structure breaks compatibility with existing log analysis tools
 * @warning name[] and format[] arrays are NOT null-terminated if fully used (legacy limitation)
 * 
 * @see LogStructure for the compile-time definition that generates this message
 * @see LOG_FORMAT_MSG enum value for this message's type identifier
 */
struct PACKED log_Format {
    LOG_PACKET_HEADER;          ///< Standard 3-byte header: head1, head2, msgid
    uint8_t type;               ///< Message type identifier being described (0-255)
    uint8_t length;             ///< Total size in bytes of the described message type
    char name[4];               ///< 4-character message name (not null-terminated if full)
    char format[16];            ///< Format string: one character per field (see format chars above)
    char labels[64];            ///< Comma-separated field names for ground station display
};

/**
 * @struct log_Unit
 * @brief UNIT message: Maps unit character identifiers to human-readable SI unit strings
 * 
 * @details The UNIT message communicates the mapping between single-character unit identifiers
 *          (used in FMTU messages) and their full SI unit representations. These messages are
 *          written at log initialization to establish the unit vocabulary for the entire log.
 *          
 *          UNIT Message Purpose:
 *          - Provides human-readable unit strings for ground station display
 *          - Maps characters like 'm', 's', 'A' to "meters", "seconds", "Amperes"
 *          - Written once at log start for all units defined in log_Units[] array
 *          - Enables GCS to show proper units on graphs and displays
 *          
 *          Message Flow:
 *          1. **Log Initialization**: UNIT messages written for each entry in log_Units[]
 *          2. **FMT Messages**: Format definitions reference data types
 *          3. **FMTU Messages**: Associate unit characters with FMT message fields
 *          4. **Data Messages**: GCS knows units for each field via UNIT→FMTU→FMT linkage
 *          
 *          Example:
 *          @code
 *          // Unit message defining meters
 *          log_Unit unit_meters = {
 *              .head1 = HEAD_BYTE1,
 *              .head2 = HEAD_BYTE2,
 *              .msgid = LOG_UNIT_MSG,
 *              .time_us = AP_HAL::micros64(),
 *              .type = 'm',
 *              .unit = "m"  // SI unit string for meters
 *          };
 *          @endcode
 * 
 * @note unit[] array is 64 bytes to accommodate long unit names like "degrees/second/second"
 * @note Most unit strings are short (1-10 chars) but buffer provides flexibility
 * @note The comment "you know, this might be overkill..." is original developer humor
 * 
 * @see log_Units[] array for all defined unit mappings
 * @see log_Format_Units for FMTU messages that associate units with message fields
 * @see UnitStructure for compile-time unit definitions
 */
struct PACKED log_Unit {
    LOG_PACKET_HEADER;          ///< Standard 3-byte header: head1, head2, msgid
    uint64_t time_us;           ///< Timestamp in microseconds since system boot
    char type;                  ///< Single character unit identifier (matches log_Units[].ID)
    char unit[64];              ///< Human-readable SI unit string (null-terminated)
};

/**
 * @struct log_Format_Multiplier
 * @brief MULT message: Maps multiplier character identifiers to numeric scaling factors
 * 
 * @details The MULT message communicates the mapping between single-character multiplier
 *          identifiers and their numeric values. These define how to scale raw integer
 *          values in log messages to obtain real-world floating-point values.
 *          
 *          MULT Message Purpose:
 *          - Defines scaling factors for efficient integer storage with float semantics
 *          - Maps characters like '0', 'C', 'F' to 1.0, 1e-3, 1e-6 respectively
 *          - Enables compact storage: int16_t with multiplier 0.001 represents ±32.767 range
 *          - Written once at log start for all multipliers in log_Multipliers[] array
 *          
 *          Critical Distinction:
 *          Multipliers apply to RAW LOG VALUES, independent of format character semantics:
 *          - Format 'h' (int16_t) with multiplier 'C' (1e-3): raw_value * 0.001 = actual_value
 *          - Format 'c' (int16_t*100) with multiplier 'C' (1e-3): raw_value * 0.001 = actual_value
 *          
 *          The format character's implied scaling (e.g., *100 in 'c') is ignored for multiplier
 *          purposes. Both 'h' and 'c' are treated as int16_t containers for multiplier application.
 *          
 *          Common Multipliers:
 *          - '0' = 1e0 (1.0)     : No scaling
 *          - 'A' = 1e-1 (0.1)    : Tenths
 *          - 'B' = 1e-2 (0.01)   : Hundredths (centimeters to meters)
 *          - 'C' = 1e-3 (0.001)  : Thousandths (milliseconds to seconds)
 *          - 'F' = 1e-6 (0.000001): Millionths (microseconds to seconds)
 *          
 *          Example:
 *          @code
 *          // MULT message defining 1e-6 multiplier (microseconds to seconds)
 *          log_Format_Multiplier mult_micro = {
 *              .head1 = HEAD_BYTE1,
 *              .head2 = HEAD_BYTE2,
 *              .msgid = LOG_MULT_MSG,
 *              .time_us = AP_HAL::micros64(),
 *              .type = 'F',
 *              .multiplier = 1e-6
 *          };
 *          @endcode
 * 
 * @note GCS must NOT infer scaling from unit names - multiplier is the authoritative source
 * @note Special multipliers: '!' = 3.6 (m/s to km/h), '/' = 3600 (A·s to A·h)
 * @note Multiplier '-' (zero) indicates no numeric scaling (used for strings)
 * 
 * @warning Multiplier application is independent of format character type scaling
 * @warning GCS implementations that infer scaling from format chars will decode incorrectly
 * 
 * @see log_Multipliers[] array for all defined multiplier mappings
 * @see log_Format_Units for FMTU messages that associate multipliers with message fields
 * @see MultiplierStructure for compile-time multiplier definitions
 */
struct PACKED log_Format_Multiplier {
    LOG_PACKET_HEADER;          ///< Standard 3-byte header: head1, head2, msgid
    uint64_t time_us;           ///< Timestamp in microseconds since system boot
    char type;                  ///< Single character multiplier identifier (matches log_Multipliers[].ID)
    double multiplier;          ///< Numeric scaling factor to apply to raw log values
};

/**
 * @struct log_Format_Units
 * @brief FMTU message: Associates units and multipliers with fields of a specific message type
 * 
 * @details The FMTU (Format Units) message links unit and multiplier characters to the fields
 *          of a specific log message type defined by a prior FMT message. This creates the
 *          complete decoding specification: FMT defines types, FMTU defines units/scaling.
 *          
 *          FMTU Message Purpose:
 *          - Associates each field in a message with its unit and multiplier
 *          - Enables GCS to display values with correct units and scaling
 *          - Provides metadata for automated analysis and plotting
 *          - Written after corresponding FMT message during log initialization
 *          
 *          Message Relationships:
 *          @code
 *          FMT message:  "GPS" format="QBffLLff" → defines 8 fields with types
 *          FMTU message: units="s#--DU--" multipliers="F-00GG00" → defines units/scaling for each field
 *          UNIT messages: Map 's'→"seconds", '#'→"instance", 'D'→"deglatitude", etc.
 *          MULT messages: Map 'F'→1e-6, '0'→1.0, 'G'→1e-7, etc.
 *          
 *          Result: Field 0 (TimeUS) is in seconds with 1e-6 multiplier (microseconds)
 *                  Field 3 (Lat) is in degrees-latitude with 1e-7 multiplier
 *          @endcode
 *          
 *          Character Position Mapping:
 *          Each character in units[] and multipliers[] corresponds to a format character
 *          at the same position in the FMT message's format string:
 *          - units[0] and multipliers[0] apply to format[0]
 *          - units[1] and multipliers[1] apply to format[1]
 *          - And so on for all fields
 *          
 *          Example:
 *          @code
 *          // FMTU message for GPS (format type LOG_GPS_MSG)
 *          log_Format_Units fmtu_gps = {
 *              .head1 = HEAD_BYTE1,
 *              .head2 = HEAD_BYTE2,
 *              .msgid = LOG_FORMAT_UNITS_MSG,
 *              .time_us = AP_HAL::micros64(),
 *              .format_type = LOG_GPS_MSG,
 *              .units = "s#--DU--",        // One character per field
 *              .multipliers = "F-00GG00"    // One character per field
 *          };
 *          @endcode
 * 
 * @note units[] and multipliers[] length must match the format string length in the FMT message
 * @note Character '-' means "no unit" or "no multiplier" (dimensionless or unscaled)
 * @note Arrays are NOT null-terminated if fully used (16 characters = 16-field message maximum)
 * 
 * @warning Mismatched lengths between FMT format and FMTU units/multipliers causes decoding errors
 * @warning format_type must reference a valid message type defined by a prior FMT message
 * 
 * @see log_Format for FMT messages that define field types
 * @see log_Unit for UNIT messages that define unit character mappings
 * @see log_Format_Multiplier for MULT messages that define multiplier character mappings
 */
struct PACKED log_Format_Units {
    LOG_PACKET_HEADER;          ///< Standard 3-byte header: head1, head2, msgid
    uint64_t time_us;           ///< Timestamp in microseconds since system boot
    uint8_t format_type;        ///< Message type ID this FMTU applies to (matches FMT.type)
    char units[16];             ///< Unit characters, one per field (references UNIT messages)
    char multipliers[16];       ///< Multiplier characters, one per field (references MULT messages)
};

/**
 * @struct log_Parameter
 * @brief PARM message: Records parameter values for flight analysis and configuration tracking
 * 
 * @details The PARM message logs parameter values at strategic times during flight, enabling
 *          ground stations and analysts to understand vehicle configuration and track parameter
 *          changes. Parameters are the user-configurable settings that control vehicle behavior.
 *          
 *          PARM Message Timing:
 *          - **Log Start**: All parameters written at log initialization (configuration snapshot)
 *          - **Parameter Change**: Written whenever a parameter is modified during flight
 *          - **Periodic**: May be written periodically for critical flight parameters
 *          - **Requested**: Written in response to MAVLink parameter read requests
 *          
 *          Use Cases:
 *          - **Configuration Tracking**: Know exact parameter values during any flight segment
 *          - **Tuning Analysis**: Compare parameter changes against flight performance
 *          - **Troubleshooting**: Identify if incorrect parameters caused issues
 *          - **Compliance**: Document configuration for flight testing and certification
 *          - **Default Comparison**: default_value shows if parameter differs from firmware default
 *          
 *          Parameter Naming:
 *          ArduPilot parameters follow hierarchical naming convention:
 *          - COMPONENT_PROPERTY (e.g., "COMPASS_ORIENT", "ATC_RAT_RLL_P")
 *          - Maximum 16 characters including null terminator
 *          - Case-sensitive (though typically uppercase)
 *          
 *          Example:
 *          @code
 *          // Log the roll rate P gain parameter
 *          log_Parameter parm = {
 *              .head1 = HEAD_BYTE1,
 *              .head2 = HEAD_BYTE2,
 *              .msgid = LOG_PARAMETER_MSG,
 *              .time_us = AP_HAL::micros64(),
 *              .name = "ATC_RAT_RLL_P",
 *              .value = 0.135,           // Current configured value
 *              .default_value = 0.150    // Firmware default value
 *          };
 *          @endcode
 * 
 * @note name[] is null-terminated if less than 16 characters
 * @note default_value enables detecting non-default configurations without firmware access
 * @note Parameter values are always stored as float regardless of underlying type
 * @note Integer parameters are converted to float for storage in this message
 * 
 * @warning name[] is NOT null-terminated if exactly 16 characters long (legacy limitation)
 * @warning Ground stations must compare values with epsilon tolerance due to float precision
 * 
 * @see AP_Param for the parameter storage and management system
 * @see MAVLink PARAM_VALUE for real-time parameter query protocol
 */
struct PACKED log_Parameter {
    LOG_PACKET_HEADER;          ///< Standard 3-byte header: head1, head2, msgid
    uint64_t time_us;           ///< Timestamp in microseconds since system boot
    char name[16];              ///< Parameter name (null-terminated if <16 chars)
    float value;                ///< Current parameter value
    float default_value;        ///< Factory default value from firmware
};

/**
 * @struct log_DSF
 * @brief DSF message: DataFlash statistics tracking logging performance and buffer health
 * 
 * @details The DSF (DataFlash File) message records statistics about the onboard logging
 *          system's performance, particularly buffer utilization and write throughput. This
 *          data is critical for diagnosing logging issues and optimizing log message rates.
 *          
 *          DSF Message Purpose:
 *          - **Performance Monitoring**: Track logging system health during flight
 *          - **Buffer Analysis**: Detect buffer overflow conditions causing data loss
 *          - **Rate Optimization**: Identify if logging rates are too aggressive
 *          - **Storage Health**: Monitor write performance to SD card or flash
 *          - **Troubleshooting**: Diagnose intermittent logging failures
 *          
 *          Buffer Space Metrics:
 *          ArduPilot uses ring buffers to decouple log message generation from storage writes:
 *          - Messages written to buffer at task rate (fast, non-blocking)
 *          - Background thread writes buffer to storage (slower, blocking I/O)
 *          - Buffer full = dropped messages = data loss
 *          
 *          Metric Interpretation:
 *          - buf_space_min approaching 0: Buffer nearly overflowed (data loss risk)
 *          - buf_space_avg < 50%: Logging rate close to maximum sustainable rate
 *          - dropped > 0: Data loss occurred, reduce logging rates
 *          - blocks incrementing steadily: Normal operation, data being written
 *          - bytes tracking cumulative data: Log file size estimate
 *          
 *          Common Issues:
 *          - **Slow SD Card**: buf_space_min very low, storage writes slow
 *          - **High Log Rates**: Too many messages per second, overwhelm buffer
 *          - **Task Overruns**: Scheduler overruns delay buffer writes
 *          - **Corrupted Storage**: Write errors cause buffer backup
 *          
 *          Example Analysis:
 *          @code
 *          // Good logging health
 *          DSF: dropped=0, buf_space_min=8192, buf_space_avg=12000
 *          
 *          // Warning: Near overflow
 *          DSF: dropped=0, buf_space_min=512, buf_space_avg=4000
 *          
 *          // Critical: Data loss
 *          DSF: dropped=15, buf_space_min=0, buf_space_avg=1024
 *          @endcode
 * 
 * @note DSF messages written periodically (typically 1-5 Hz) to track trends
 * @note Buffer space measured in bytes available in ring buffer
 * @note Statistics tracked over recent time window (typically 1 second)
 * 
 * @warning dropped > 0 indicates data loss; reduce log rates or improve storage speed
 * @warning buf_space_min = 0 may indicate sustained overflow condition
 * 
 * @see AP_Logger for logging system implementation
 * @see log_DMS for DataFlash-over-MAVLink statistics
 */
struct PACKED log_DSF {
    LOG_PACKET_HEADER;          ///< Standard 3-byte header: head1, head2, msgid
    uint64_t time_us;           ///< Timestamp in microseconds since system boot
    uint32_t dropped;           ///< Number of log messages dropped due to buffer overflow
    uint16_t blocks;            ///< Current block number being written to storage
    uint32_t bytes;             ///< Total bytes written to log file
    uint32_t buf_space_min;     ///< Minimum free buffer space in measurement period (bytes)
    uint32_t buf_space_max;     ///< Maximum free buffer space in measurement period (bytes)
    uint32_t buf_space_avg;     ///< Average free buffer space in measurement period (bytes)
};

/**
 * @struct log_Event
 * @brief EV message: Coded event markers for significant flight milestones and state changes
 * 
 * @details The EV message logs specifically coded events that mark important transitions,
 *          milestones, or noteworthy conditions during flight. Events are compact markers
 *          (single byte ID) that provide waypoints in the log timeline for analysis.
 *          
 *          Event System Purpose:
 *          - **Flight Milestones**: Mark takeoff, landing, mode changes, mission waypoints
 *          - **State Transitions**: Log important system state changes
 *          - **Timeline Markers**: Provide reference points for log analysis
 *          - **Compact Logging**: Efficient representation of discrete occurrences
 *          - **Automated Analysis**: Enable scripted log parsing for event sequences
 *          
 *          Event Categories (examples):
 *          - **Flight Phase**: Takeoff, landing, mission start/end
 *          - **Autonomy**: Auto mode engaged, guided mode target reached
 *          - **Safety**: Geofence breach, EKF failsafe, battery failsafe
 *          - **Navigation**: Waypoint reached, loiter started, return to launch
 *          - **System**: Logging started, parameters loaded, calibration complete
 *          
 *          Event vs Error/Message:
 *          - **Event (EV)**: Normal operational milestones, expected occurrences
 *          - **Error (ERR)**: Abnormal conditions requiring attention
 *          - **Message (MSG)**: Human-readable text for debugging
 *          
 *          Typical Event IDs (see LogEvent enum):
 *          - FLIGHT_MODE_CHANGE: Vehicle changed flight mode
 *          - MISSION_ITEM_REACHED: Waypoint reached in AUTO mode
 *          - FENCE_BREACH: Geofence boundary crossed
 *          - EKF_FAILSAFE: EKF switched to backup or degraded mode
 *          - LAND_COMPLETE: Landing detected/completed
 *          
 *          Usage Example:
 *          @code
 *          // Log waypoint reached event
 *          log_Event ev = {
 *              .head1 = HEAD_BYTE1,
 *              .head2 = HEAD_BYTE2,
 *              .msgid = LOG_EVENT_MSG,
 *              .time_us = AP_HAL::micros64(),
 *              .id = static_cast<uint8_t>(LogEvent::MISSION_WAYPOINT_REACHED)
 *          };
 *          @endcode
 *          
 *          Analysis Applications:
 *          - Plot events on timeline graphs
 *          - Calculate time between events (e.g., mission duration)
 *          - Detect event sequences (e.g., failsafe cascade)
 *          - Correlate events with sensor data anomalies
 * 
 * @note Event IDs defined in LogEvent enum (values documented with @LoggerMessage: EV)
 * @note Multiple EV messages may be written per second during dynamic operations
 * @note Events are timestamped precisely at occurrence for timeline accuracy
 * 
 * @warning Do not confuse with ERR messages which indicate error conditions
 * 
 * @see LogEvent enum for complete event ID definitions
 * @see log_Error for error condition logging
 * @see log_Message for free-form text logging
 */
struct PACKED log_Event {
    LOG_PACKET_HEADER;          ///< Standard 3-byte header: head1, head2, msgid
    uint64_t time_us;           ///< Timestamp in microseconds since system boot
    uint8_t id;                 ///< Event identifier from LogEvent enum
};

/**
 * @struct log_Error
 * @brief ERR message: Coded error conditions by subsystem for fault diagnosis and analysis
 * 
 * @details The ERR message logs specifically coded error conditions detected by various
 *          vehicle subsystems. Each error is identified by a subsystem ID and subsystem-specific
 *          error code, enabling precise fault diagnosis and troubleshooting.
 *          
 *          Error System Architecture:
 *          - **Two-Level Hierarchy**: Subsystem identifies component, error code identifies issue
 *          - **Compact Representation**: Two bytes encode error condition efficiently
 *          - **Subsystem Isolation**: Each subsystem maintains its own error code space
 *          - **Precise Timing**: Timestamp captures exact moment of error detection
 *          - **Multiple Instances**: Same error may log repeatedly if condition persists
 *          
 *          Error vs Event vs Message:
 *          - **ERR**: Abnormal conditions, faults, failures requiring attention
 *          - **EV**: Normal operational milestones and expected state transitions
 *          - **MSG**: Human-readable text for debugging (less structured)
 *          
 *          Common Subsystems (LogErrorSubsystem enum):
 *          - MAIN: Core autopilot errors
 *          - RADIO: RC receiver and telemetry radio errors
 *          - COMPASS: Magnetometer calibration and health errors
 *          - OPTFLOW: Optical flow sensor errors
 *          - FAILSAFE: Failsafe activation conditions
 *          - GPS: GPS receiver errors and glitches
 *          - CRASH: Crash detection false positives or issues
 *          - FLIP: Flip mode errors
 *          - EKF: Extended Kalman Filter errors
 *          - TERRAIN: Terrain database errors
 *          
 *          Error Code Interpretation:
 *          Error codes are subsystem-specific; consult subsystem documentation:
 *          - Compass: 0=calibration failed, 1=inconsistent, 2=unhealthy
 *          - GPS: 0=glitch, 1=fix lost
 *          - EKF: 0=bad variance, 1=excessive innovation
 *          
 *          Example Scenarios:
 *          @code
 *          // Compass calibration failure
 *          ERR: subsys=COMPASS(2), error_code=0  // Calibration failed
 *          
 *          // GPS glitch detected
 *          ERR: subsys=GPS(5), error_code=0  // GPS glitch
 *          
 *          // EKF failsafe activation
 *          ERR: subsys=EKF(8), error_code=2  // EKF failsafe triggered
 *          @endcode
 *          
 *          Troubleshooting Workflow:
 *          1. Identify ERR messages in log
 *          2. Note subsystem ID and look up subsystem name
 *          3. Note error code and consult subsystem documentation
 *          4. Correlate error timestamp with sensor data and events
 *          5. Determine root cause and corrective action
 *          
 *          Safety Analysis:
 *          - ERR messages may precede failsafe activations
 *          - Multiple ERRs in sequence may indicate cascading failures
 *          - Repeated ERRs suggest persistent configuration or hardware issues
 * 
 * @note Subsystem IDs defined in LogErrorSubsystem enum
 * @note Error codes are subsystem-specific; see subsystem source code for definitions
 * @note Same error may log multiple times if condition persists or recurs
 * @note Critical errors often followed by failsafe activation (check for MODE changes)
 * 
 * @warning ERR messages indicate abnormal conditions requiring investigation
 * @warning Some errors are informational (e.g., EKF glitch), others are critical
 * 
 * @see LogErrorSubsystem enum for subsystem ID definitions
 * @see log_Event for normal operational events
 * @see log_Message for human-readable debug messages
 */
struct PACKED log_Error {
  LOG_PACKET_HEADER;          ///< Standard 3-byte header: head1, head2, msgid
  uint64_t time_us;           ///< Timestamp in microseconds since system boot
  uint8_t sub_system;         ///< Subsystem that detected the error (LogErrorSubsystem enum)
  uint8_t error_code;         ///< Subsystem-specific error code identifying the fault
};


/**
 * @struct log_Message
 * @brief MSG message: Human-readable text messages for debugging and status information
 * 
 * @details The MSG message logs free-form text strings providing human-readable information
 *          about system state, warnings, and debugging output. Unlike ERR and EV messages
 *          which use coded IDs, MSG contains natural language for developer interpretation.
 *          
 *          MSG Message Categories:
 *          - **Initialization**: Subsystem startup messages ("GPS: UBLOX init")
 *          - **Warnings**: Non-critical issues ("Compass cal poor", "EKF variance high")
 *          - **Status**: Operational state changes ("Fence enabled", "Mission loaded")
 *          - **Debug**: Developer troubleshooting output (when enabled)
 *          - **Configuration**: Parameter-related messages ("Loaded 247 params")
 *          
 *          MSG vs ERR vs EV:
 *          - **MSG**: Human-readable text, flexible, requires manual parsing
 *          - **ERR**: Coded errors, structured, machine-parseable
 *          - **EV**: Coded events, structured, machine-parseable
 *          
 *          Message Content Guidelines:
 *          - Concise: Maximum 64 characters including null terminator
 *          - Informative: Clear description of condition or state
 *          - Context: Include relevant identifiers (sensor instance, mode name)
 *          - Severity: Implicit from content (no explicit severity field)
 *          
 *          Common Message Patterns:
 *          @code
 *          "PreArm: GPS not healthy"
 *          "EKF2 IMU0 switching to IMU1"
 *          "Fence breach: ALT"
 *          "Mission: 15 waypoints loaded"
 *          "Compass cal failed: field too weak"
 *          "Throttle failsafe: no PWM"
 *          @endcode
 *          
 *          Ground Station Display:
 *          - GCS typically displays MSG content in status/message console
 *          - Messages may trigger audible alerts if keywords detected
 *          - Log analysis tools can filter/search message text
 *          
 *          Logging Considerations:
 *          - MSG messages consume more log space than coded ERR/EV
 *          - Rate-limited to prevent log buffer overflow
 *          - Critical messages prioritized over debug messages
 *          - Same message may repeat if condition persists
 *          
 *          Analysis Applications:
 *          - Text search for specific subsystem names or keywords
 *          - Timeline of initialization sequence
 *          - Warning/error correlation with sensor data
 *          - Failure mode analysis from message sequence
 *          
 *          Example Usage:
 *          @code
 *          // Log initialization message
 *          log_Message msg;
 *          msg.head1 = HEAD_BYTE1;
 *          msg.head2 = HEAD_BYTE2;
 *          msg.msgid = LOG_MESSAGE_MSG;
 *          msg.time_us = AP_HAL::micros64();
 *          strncpy(msg.msg, "EKF3: GPS 0 selected", sizeof(msg.msg));
 *          msg.msg[sizeof(msg.msg)-1] = '\0';  // Ensure null termination
 *          @endcode
 * 
 * @note msg[] buffer is 64 bytes; always null-terminate to prevent overruns
 * @note If message exceeds 63 characters, it will be truncated
 * @note MSG not null-terminated if exactly 64 characters (rare edge case)
 * @note Message text should be concise due to size limit
 * 
 * @warning Rate limiting may drop MSG messages during high-frequency logging
 * @warning Do not rely on MSG for critical error reporting; use ERR messages
 * 
 * @see log_Error for coded error conditions
 * @see log_Event for coded milestone events
 * @see GCS_MAVLINK::send_statustext() for real-time message transmission
 */
struct PACKED log_Message {
    LOG_PACKET_HEADER;          ///< Standard 3-byte header: head1, head2, msgid
    uint64_t time_us;           ///< Timestamp in microseconds since system boot
    char msg[64];               ///< Message text (null-terminated if <64 chars)
};

/**
 * @struct log_RCIN
 * @brief RCIN message: RC input PWM values for channels 1-14 from pilot transmitter
 * 
 * @details The RCIN message logs the raw PWM (Pulse Width Modulation) input values from
 *          the RC receiver for the first 14 channels. These values represent the pilot's
 *          stick, switch, and dial positions, captured before any processing, failsafe
 *          overrides, or mapping.
 *          
 *          RCIN Message Purpose:
 *          - **Input Validation**: Verify pilot commands are being received correctly
 *          - **Failsafe Analysis**: Detect RC signal loss or corruption
 *          - **Control Calibration**: Verify RC input ranges and centering
 *          - **Flight Review**: Understand pilot inputs during maneuvers
 *          - **Tuning Correlation**: Compare pilot inputs with vehicle response
 *          
 *          PWM Value Ranges:
 *          - **Standard Range**: 1000-2000 microseconds (most common)
 *          - **Center/Neutral**: Typically 1500 microseconds
 *          - **Full Deflection**: 1000 (low/left/back) to 2000 (high/right/forward)
 *          - **Extended Range**: Some systems support 800-2200 microseconds
 *          - **Invalid/No Signal**: Often reads 0 or anomalous value
 *          
 *          Standard Channel Assignments (Mode 2 transmitter):
 *          - chan1: Roll (aileron) - typically 1000-2000, neutral 1500
 *          - chan2: Pitch (elevator) - typically 1000-2000, neutral 1500
 *          - chan3: Throttle - typically 1000-2000 (low to high)
 *          - chan4: Yaw (rudder) - typically 1000-2000, neutral 1500
 *          - chan5-8: Auxiliary switches/dials (mode selection, camera control, etc.)
 *          - chan9-14: Additional aux channels (optional features)
 *          
 *          Note: Channel assignments are configurable via RCx_OPTION parameters
 *          
 *          RC Protocols Supported:
 *          - PPM (Pulse Position Modulation)
 *          - SBUS (Futaba)
 *          - DSM/DSM2/DSMX (Spektrum)
 *          - CRSF (Crossfire/ELRS)
 *          - IBUS (FlySky)
 *          - SRXL (Multiplex)
 *          
 *          Failsafe Detection:
 *          - Channel values = 0: Total RC signal loss
 *          - Channel values frozen: RC receiver in failsafe mode
 *          - Erratic values: RF interference or weak signal
 *          - Check RCI2 message for override flags and failsafe status
 *          
 *          Analysis Applications:
 *          @code
 *          // Normal flight - all channels in valid range
 *          RCIN: C1=1500, C2=1500, C3=1200, C4=1500, C5=1800, ...
 *          
 *          // RC failsafe - channels drop to 0 or failsafe values
 *          RCIN: C1=0, C2=0, C3=0, C4=0, C5=0, ...
 *          
 *          // Aggressive maneuver - stick inputs at limits
 *          RCIN: C1=1950, C2=1100, C3=1800, C4=1600, ...
 *          @endcode
 *          
 *          Troubleshooting:
 *          - If RCIN all zeros: Check RC receiver connection and power
 *          - If RCIN frozen: Check for RC failsafe condition
 *          - If RCIN noisy: Check for RF interference or weak signal
 *          - If RCIN ranges wrong: Calibrate RC transmitter and ArduPilot
 * 
 * @note RCIN logged at high rate (typically 50-400Hz) for accurate pilot input capture
 * @note Values are raw PWM in microseconds before any scaling or failsafe processing
 * @note Channel count depends on RC protocol; unused channels typically read 0
 * @note Channels 15-16 logged in separate RCI2 message
 * 
 * @warning chanX = 0 often indicates no signal on that channel
 * @warning Sudden changes in all channels may indicate RC failsafe activation
 * 
 * @see log_RCI2 for channels 15-16 and override status
 * @see log_RCOUT for servo/motor outputs (after processing)
 * @see RC_Channel for RC input processing and mapping
 */
struct PACKED log_RCIN {
    LOG_PACKET_HEADER;          ///< Standard 3-byte header: head1, head2, msgid
    uint64_t time_us;           ///< Timestamp in microseconds since system boot
    uint16_t chan1;             ///< RC input channel 1 PWM value (microseconds)
    uint16_t chan2;             ///< RC input channel 2 PWM value (microseconds)
    uint16_t chan3;             ///< RC input channel 3 PWM value (microseconds)
    uint16_t chan4;             ///< RC input channel 4 PWM value (microseconds)
    uint16_t chan5;             ///< RC input channel 5 PWM value (microseconds)
    uint16_t chan6;             ///< RC input channel 6 PWM value (microseconds)
    uint16_t chan7;             ///< RC input channel 7 PWM value (microseconds)
    uint16_t chan8;             ///< RC input channel 8 PWM value (microseconds)
    uint16_t chan9;             ///< RC input channel 9 PWM value (microseconds)
    uint16_t chan10;            ///< RC input channel 10 PWM value (microseconds)
    uint16_t chan11;            ///< RC input channel 11 PWM value (microseconds)
    uint16_t chan12;            ///< RC input channel 12 PWM value (microseconds)
    uint16_t chan13;            ///< RC input channel 13 PWM value (microseconds)
    uint16_t chan14;            ///< RC input channel 14 PWM value (microseconds)
};

/**
 * @struct log_RCI2
 * @brief RCI2 message: Additional RC input channels 15-16 plus override and status flags
 * 
 * @details The RCI2 message is a companion to RCIN, providing the remaining RC input channels
 *          (15-16) along with critical status information about RC overrides and receiver
 *          health. This message was split from RCIN to accommodate more channels without
 *          making RCIN excessively large.
 *          
 *          RCI2 Message Purpose:
 *          - **Extended Channels**: Support systems with more than 14 RC channels
 *          - **Override Detection**: Track MAVLink RC override commands from GCS
 *          - **Receiver Health**: Monitor RC receiver signal quality and failsafe state
 *          - **Control Authority**: Identify who has control (pilot vs GCS override)
 *          
 *          Override Mask Interpretation:
 *          The override_mask is a bitmask where each bit represents one RC channel:
 *          - Bit 0 (0x0001): Channel 1 overridden
 *          - Bit 1 (0x0002): Channel 2 overridden
 *          - ...
 *          - Bit 15 (0x8000): Channel 16 overridden
 *          
 *          When a bit is set, the corresponding channel is being controlled by MAVLink
 *          RC_CHANNELS_OVERRIDE messages from a ground control station rather than the
 *          physical RC receiver. This is common during:
 *          - Ground station control in guided mode
 *          - Click-to-fly operations
 *          - Automated testing via MAVProxy/DroneKit
 *          - RC signal lost but GCS link maintained
 *          
 *          Flags Field (AP_Logger::RCLoggingFlags enum):
 *          Bitmask indicating RC receiver status:
 *          - Bit 0: RC failsafe active
 *          - Bit 1: RC signal quality low
 *          - Bit 2: RC input frozen (not updating)
 *          - Other bits: Protocol-specific status information
 *          
 *          Analysis Scenarios:
 *          @code
 *          // Normal operation - no overrides, all channels from pilot
 *          RCI2: C15=1500, C16=1500, OMask=0x0000, Flags=0x00
 *          
 *          // GCS controlling channels 1-4 (roll, pitch, throttle, yaw)
 *          RCI2: C15=1500, C16=1500, OMask=0x000F, Flags=0x00
 *          
 *          // RC failsafe active
 *          RCI2: C15=0, C16=0, OMask=0x0000, Flags=0x01
 *          @endcode
 *          
 *          Troubleshooting Applications:
 *          - **Unexpected Vehicle Behavior**: Check if OMask != 0 (GCS override active)
 *          - **Control Authority Confusion**: Override mask shows who controls each channel
 *          - **RC Failsafe**: flags & 0x01 indicates RC signal lost
 *          - **Intermittent Control**: Check flags for signal quality issues
 *          
 *          Override Safety Considerations:
 *          - Overrides bypass physical RC transmitter safety (dead-man switch)
 *          - If GCS link fails during override, throttle failsafe should engage
 *          - Verify override timeouts configured appropriately
 *          - Physical RC should be able to regain control (depends on FS_OPTIONS)
 * 
 * @note RCI2 logged at same rate as RCIN (typically 50-400Hz)
 * @note chan15/chan16 = 0 if receiver doesn't support that many channels
 * @note override_mask only reflects MAVLink overrides, not other control modes
 * @note flags definition depends on RC protocol; check AP_Logger::RCLoggingFlags
 * 
 * @warning override_mask != 0 indicates external control; verify intentional
 * @warning flags indicate receiver health issues; may precede RC failsafe
 * 
 * @see log_RCIN for channels 1-14
 * @see AP_Logger::RCLoggingFlags for flags field bit definitions
 * @see RC_CHANNELS_OVERRIDE MAVLink message
 */
struct PACKED log_RCI2 {
    LOG_PACKET_HEADER;          ///< Standard 3-byte header: head1, head2, msgid
    uint64_t time_us;           ///< Timestamp in microseconds since system boot
    uint16_t chan15;            ///< RC input channel 15 PWM value (microseconds)
    uint16_t chan16;            ///< RC input channel 16 PWM value (microseconds)
    uint16_t override_mask;     ///< Bitmask of channels overridden by MAVLink (bit 0=ch1, etc.)
    uint8_t flags;              ///< RC receiver status flags (AP_Logger::RCLoggingFlags)
};

/**
 * @struct log_RCOUT
 * @brief RCOU message: Servo and motor output PWM values for channels 1-14
 * 
 * @details The RCOU message logs the final PWM output values sent to servos, ESCs, and other
 *          actuators on channels 1-14. These values represent the vehicle's control commands
 *          AFTER all processing: pilot input mapping, flight mode logic, stabilization,
 *          failsafe overrides, and output limiting.
 *          
 *          RCOUT Message Purpose:
 *          - **Control Verification**: Confirm flight controller is commanding expected outputs
 *          - **Tuning Analysis**: Correlate control outputs with vehicle response
 *          - **Mixer Validation**: Verify motor/servo mixing is working correctly
 *          - **Failsafe Testing**: Verify failsafe outputs are as configured
 *          - **Actuator Diagnosis**: Detect servo flutter, ESC arming issues, output limits
 *          
 *          PWM Output Interpretation:
 *          - **Motor ESCs**: Typically 1000-2000 μs (1000=stopped, 2000=full throttle)
 *          - **Servos**: Typically 1000-2000 μs (center often 1500 μs)
 *          - **Digital Protocols**: PWM value may encode protocol data (DShot, OneShot)
 *          - **Zero Output**: May indicate disabled channel or initialization state
 *          
 *          Common Channel Assignments (Multicopter):
 *          - chan1-4/6/8: Motor outputs to ESCs (exact assignment depends on frame type)
 *          - Remaining: Auxiliary functions (camera gimbal, landing gear, etc.)
 *          
 *          Common Channel Assignments (Fixed-Wing):
 *          - chan1: Aileron (or elevon left)
 *          - chan2: Elevator (or elevon right)
 *          - chan3: Throttle
 *          - chan4: Rudder
 *          - chan5-14: Flaps, airbrakes, camera, etc.
 *          
 *          Note: Actual assignments configured via SERVOx_FUNCTION parameters
 *          
 *          Output Processing Pipeline:
 *          RCIN (pilot) → Mode Logic → Stabilization → Mixer → Output Limits → RCOUT
 *          
 *          Analysis Applications:
 *          @code
 *          // Multicopter hover - motors at mid-throttle, balanced
 *          RCOU: C1=1450, C2=1455, C3=1448, C4=1452, C5=1500, ...
 *          
 *          // Multicopter full throttle climb - all motors high
 *          RCOU: C1=1950, C2=1950, C3=1950, C4=1950, C5=1500, ...
 *          
 *          // Fixed-wing turn - aileron deflection, rudder coordination
 *          RCOU: C1=1700, C2=1500, C3=1600, C4=1600, ...
 *          
 *          // Failsafe - outputs at configured failsafe values
 *          RCOU: C1=1000, C2=1000, C3=1000, C4=1000, ...
 *          @endcode
 *          
 *          Troubleshooting:
 *          - **Output Saturation**: All motors at max (2000) = insufficient power
 *          - **Asymmetric Outputs**: Large differences suggest control fighting trim issues
 *          - **Oscillating Outputs**: Rapid changes indicate instability or vibration
 *          - **Stuck Outputs**: Frozen values suggest ESC/servo disconnection
 *          - **Zero Outputs**: Disarmed state or output disabled
 *          
 *          Motor Order Verification:
 *          Compare RCOUT during test hover to expected motor layout:
 *          - Apply roll input → specific motors should increase/decrease
 *          - Apply pitch input → fore/aft motors should change
 *          - Apply yaw input → diagonal motors should differ
 *          
 *          Tuning Insights:
 *          - High-frequency oscillations: P or D gain too high
 *          - Slow wandering: I gain too low or too high
 *          - Limited authority: Output hitting limits (check ThLimit in MOTB)
 *          - Response lag: Filter delays or rate limits
 * 
 * @note RCOU logged at main loop rate (typically 100-400Hz) for detailed control analysis
 * @note Compare with RCIN to see effect of flight controller processing
 * @note ESC protocols (DShot, OneShot) encode commands differently than standard PWM
 * @note Some channels may be disabled (output 0) if SERVOx_FUNCTION = 0
 * @note Channels 15-18 logged in separate RCO2 message
 * @note Channels 19-32 logged in separate RCO3 message
 * 
 * @warning Output saturation (all motors at max) indicates control authority exhaustion
 * @warning Rapid oscillations may indicate dangerous flight instability
 * 
 * @see log_RCOUT2 for channels 15-18
 * @see log_RCOUT3 for channels 19-32
 * @see log_RCIN for comparison with pilot inputs
 * @see SRV_Channel for output channel management
 * @see AP_Motors for motor mixing (multicopter)
 */
struct PACKED log_RCOUT {
    LOG_PACKET_HEADER;          ///< Standard 3-byte header: head1, head2, msgid
    uint64_t time_us;           ///< Timestamp in microseconds since system boot
    uint16_t chan1;             ///< Output channel 1 PWM value (microseconds)
    uint16_t chan2;             ///< Output channel 2 PWM value (microseconds)
    uint16_t chan3;             ///< Output channel 3 PWM value (microseconds)
    uint16_t chan4;             ///< Output channel 4 PWM value (microseconds)
    uint16_t chan5;             ///< Output channel 5 PWM value (microseconds)
    uint16_t chan6;             ///< Output channel 6 PWM value (microseconds)
    uint16_t chan7;             ///< Output channel 7 PWM value (microseconds)
    uint16_t chan8;             ///< Output channel 8 PWM value (microseconds)
    uint16_t chan9;             ///< Output channel 9 PWM value (microseconds)
    uint16_t chan10;            ///< Output channel 10 PWM value (microseconds)
    uint16_t chan11;            ///< Output channel 11 PWM value (microseconds)
    uint16_t chan12;            ///< Output channel 12 PWM value (microseconds)
    uint16_t chan13;            ///< Output channel 13 PWM value (microseconds)
    uint16_t chan14;            ///< Output channel 14 PWM value (microseconds)
};

/**
 * @struct log_RCOUT2
 * @brief RCO2 message: Servo and motor output PWM values for channels 15-18
 * 
 * @details The RCO2 message is a companion to RCOU, providing output channels 15-18.
 *          These channels are typically used for auxiliary functions on complex vehicles
 *          with many actuators (large multirotors, VTOL aircraft, rovers with multiple
 *          servos, etc.).
 *          
 *          RCO2 Message Purpose:
 *          - **Extended Outputs**: Support vehicles with more than 14 output channels
 *          - **Auxiliary Functions**: Camera gimbals, grippers, additional control surfaces
 *          - **Redundancy**: Backup servos or ESCs on safety-critical systems
 *          - **Complex Vehicles**: VTOL with separate lift/cruise motors, octocopters+
 *          
 *          Common Uses for Channels 15-18:
 *          - **Hexacopter+**: Additional motor outputs (hexa=6, octo=8+)
 *          - **VTOL Aircraft**: Separate lift motors, tilt servos, elevons
 *          - **Camera Systems**: Multi-axis gimbal servos (pan, tilt, roll)
 *          - **Payload Delivery**: Gripper, parachute, or drop mechanism
 *          - **Complex Fixed-Wing**: Additional flaps, slats, spoilers, airbrakes
 *          - **Underwater Vehicles**: Thrusters in multiple axes (surge, sway, heave)
 *          
 *          Channel Assignment Configuration:
 *          Each channel's function is set via SERVOx_FUNCTION parameter:
 *          - SERVO15_FUNCTION, SERVO16_FUNCTION, SERVO17_FUNCTION, SERVO18_FUNCTION
 *          - Function 0 = Disabled (output will be 0)
 *          - Function 1-99 = Specific vehicle functions (motor, aileron, camera, etc.)
 *          - See SRV_Channel::Aux_servo_function_t enum for complete function list
 *          
 *          Analysis Scenarios:
 *          @code
 *          // Octocopter with 8 motors (motors 5-8 in RCO2)
 *          RCOU: C1=1450, C2=1450, C3=1450, C4=1450, ...
 *          RCO2: C15=1450, C16=1450, C17=1450, C18=1450
 *          
 *          // Fixed-wing with camera gimbal on channels 15-16
 *          RCO2: C15=1750, C16=1300, C17=0, C18=0
 *                (pan servo, tilt servo, disabled, disabled)
 *          
 *          // VTOL with tilt servos on channels 15-16
 *          RCO2: C15=1000, C16=1000, C17=0, C18=0
 *                (vertical mode - tilt servos at 0 degrees)
 *          @endcode
 *          
 *          Troubleshooting:
 *          - **Zero Outputs**: Check SERVOx_FUNCTION; 0 = disabled
 *          - **Missing Channels**: Verify board has sufficient output channels
 *          - **Unexpected Values**: Verify correct function assigned to channel
 *          - **No Movement**: Check servo reversing (SERVOx_REVERSED)
 * 
 * @note RCO2 logged at same rate as RCOU
 * @note chan15-18 = 0 if corresponding SERVO_FUNCTION = 0 (disabled)
 * @note Not all flight controllers support 18+ output channels (check hardware)
 * @note Some output protocols (DShot, OneShot) may not be available on all channels
 * 
 * @see log_RCOUT for channels 1-14
 * @see log_RCOUT3 for channels 19-32
 * @see SRV_Channel for output channel management
 * @see SERVOx_FUNCTION parameters for channel assignment
 */
struct PACKED log_RCOUT2 {
    LOG_PACKET_HEADER;          ///< Standard 3-byte header: head1, head2, msgid
    uint64_t time_us;           ///< Timestamp in microseconds since system boot
    uint16_t chan15;            ///< Output channel 15 PWM value (microseconds)
    uint16_t chan16;            ///< Output channel 16 PWM value (microseconds)
    uint16_t chan17;            ///< Output channel 17 PWM value (microseconds)
    uint16_t chan18;            ///< Output channel 18 PWM value (microseconds)
};

/**
 * @struct log_MAV
 * @brief MAV message: MAVLink ground control station link statistics and health
 * 
 * @details The MAV message logs detailed statistics for each active MAVLink communication
 *          channel between the autopilot and ground control stations (GCS), telemetry radios,
 *          or companion computers. This data is critical for diagnosing telemetry link quality,
 *          bandwidth issues, and communication reliability problems.
 *          
 *          MAV Message Purpose:
 *          - **Link Quality Monitoring**: Track packet loss, dropped messages, buffer health
 *          - **Bandwidth Management**: Identify when telemetry exceeds link capacity
 *          - **Troubleshooting**: Diagnose intermittent connection issues
 *          - **Performance Tuning**: Optimize stream rates for available bandwidth
 *          - **Redundancy Verification**: Monitor health of multiple telemetry links
 *          
 *          MAVLink Channel Types:
 *          ArduPilot supports multiple simultaneous MAVLink channels:
 *          - **Channel 0**: Typically primary telemetry radio (e.g., 915MHz or 433MHz)
 *          - **Channel 1**: Typically secondary telemetry or USB connection
 *          - **Channel 2+**: Additional GCS links, companion computers, or OSD systems
 *          - **Channel N**: Each USB, UART, or network connection is a separate channel
 *          
 *          Packet Statistics Interpretation:
 *          - **packet_tx_count**: Total MAVLink packets sent on this channel since boot
 *          - **packet_rx_success_count**: Total valid packets received successfully
 *          - **packet_rx_drop_count**: Packets dropped due to buffer overflow or corruption
 *          - Packet loss ratio: (packet_rx_drop_count / total_attempted) gives link quality
 *          
 *          Flags Field (GCS_MAVLINK::Flags bitmask):
 *          Compact status indicators for channel state:
 *          - Bit 0: Channel active/connected
 *          - Bit 1: Streaming enabled
 *          - Bit 2: High-priority message pending
 *          - Bit 3: Mission download in progress
 *          - Other bits: Protocol-specific state flags
 *          
 *          Stream Slowdown:
 *          stream_slowdown_ms indicates milliseconds being added to each message interval
 *          to fit telemetry within available bandwidth:
 *          - **0 ms**: No slowdown, operating at configured rates
 *          - **>0 ms**: Link congested, messages delayed to prevent buffer overflow
 *          - **High values (>100ms)**: Severe bandwidth limitation, consider reducing rates
 *          
 *          Buffer Health:
 *          times_full counts how many times the transmit buffer was completely full when
 *          trying to send a message:
 *          - **0**: Buffer keeping up with message generation
 *          - **>0**: Messages being delayed or dropped; reduce stream rates or increase baud
 *          - **Rapidly increasing**: Active link degradation; immediate tuning needed
 *          
 *          GCS Activity:
 *          GCS_SYSID_last_seen_ms tracks when autopilot last received a message from the
 *          active ground station (typically heartbeat or manual control):
 *          - **Recent (<1000ms)**: Active GCS connection
 *          - **Old (>3000ms)**: GCS disconnected or unresponsive
 *          - Used for GCS failsafe triggering
 *          
 *          Analysis Scenarios:
 *          @code
 *          // Healthy telemetry link - no packet loss, no slowdown
 *          MAV: chan=0, txp=1234, rxp=456, rxdp=0, flags=0x03, ss=0, tf=0, mgs=50
 *          
 *          // Congested link - slowdown active, buffer occasionally full
 *          MAV: chan=0, txp=5678, rxp=890, rxdp=5, flags=0x03, ss=50, tf=12, mgs=100
 *          
 *          // Failing link - significant packet loss, severe slowdown
 *          MAV: chan=0, txp=9999, rxp=1234, rxdp=234, flags=0x03, ss=500, tf=456, mgs=200
 *          
 *          // Disconnected GCS - no recent messages
 *          MAV: chan=0, txp=1000, rxp=100, rxdp=0, flags=0x01, ss=0, tf=0, mgs=5000
 *          @endcode
 *          
 *          Troubleshooting Guide:
 *          - **High rxdp**: Reduce SR_x stream rates or increase baud rate
 *          - **High ss**: Link saturated; disable unnecessary message streams
 *          - **High tf**: Buffer overflows; reduce message frequency immediately
 *          - **Old mgs**: GCS connection lost; check radio power and range
 *          - **Increasing txp but stable rxp**: One-way link failure (uplink working)
 *          
 *          Bandwidth Optimization:
 *          Use MAV data to optimize SRx_* parameters (stream rates):
 *          1. Start with conservative (slow) rates
 *          2. Monitor ss (slowdown) and tf (times full)
 *          3. Increase rates until ss > 0 or tf starts incrementing
 *          4. Back off 10-20% for margin
 * 
 * @note MAV message logged periodically (typically 1-5 Hz) for each active channel
 * @note chan corresponds to HAL UART channel number (not MAVLink system ID)
 * @note Packet counts are cumulative since boot (will wrap at 65535)
 * @note GCS_SYSID_last_seen_ms is milliseconds since boot (can wrap at 49.7 days)
 * 
 * @warning stream_slowdown_ms > 100 indicates severe bandwidth limitation
 * @warning times_full > 0 indicates message drops; reduce stream rates
 * @warning GCS_SYSID_last_seen_ms > 3000 may trigger GCS failsafe
 * 
 * @see GCS_MAVLink for MAVLink communication implementation
 * @see SR_x parameters for stream rate configuration (x = port number)
 * @see SERIAL_PROTOCOL parameters for channel configuration
 */
struct PACKED log_MAV {
    LOG_PACKET_HEADER;          ///< Standard 3-byte header: head1, head2, msgid
    uint64_t time_us;           ///< Timestamp in microseconds since system boot
    uint8_t chan;               ///< MAVLink channel number (UART index)
    uint16_t packet_tx_count;   ///< Total packets transmitted since boot
    uint16_t packet_rx_success_count;  ///< Total packets successfully received
    uint16_t packet_rx_drop_count;     ///< Packets dropped due to errors or overflow
    uint8_t flags;              ///< Channel status flags (GCS_MAVLINK::Flags bitmask)
    uint16_t stream_slowdown_ms;///< Milliseconds added to message intervals due to bandwidth limits
    uint16_t times_full;        ///< Count of times transmit buffer was full
    uint32_t GCS_SYSID_last_seen_ms;  ///< Milliseconds since boot when last GCS message received
};

/**
 * @struct log_RSSI
 * @brief RSSI message: RC receiver signal strength indicator and link quality
 * 
 * @details The RSSI message logs signal strength and link quality metrics from the RC receiver,
 *          providing real-time monitoring of the pilot control link health. This data is critical
 *          for identifying range limitations, interference issues, and predicting potential
 *          RC failsafe conditions.
 *          
 *          RSSI Message Purpose:
 *          - **Range Testing**: Verify sufficient signal strength at operational distances
 *          - **Interference Detection**: Identify RF interference or multipath fading
 *          - **Failsafe Prediction**: Monitor degrading link quality before failsafe trigger
 *          - **Antenna Performance**: Verify proper antenna placement and orientation
 *          - **Link Margin Analysis**: Ensure adequate signal margin for safety
 *          
 *          RXRSSI (Received Signal Strength Indicator):
 *          Signal strength of the received RC signal, units and range depend on protocol:
 *          - **SBUS/FPort**: 0-100 (arbitrary units, higher is better)
 *          - **CRSF (TBS Crossfire)**: 0-100 (represents dBm mapped to percentage)
 *          - **ExpressLRS**: 0-100 (represents RSSI percentage)
 *          - **DSM**: -100 to 0 dBm (actual received power)
 *          - **Analog PPM**: Often not available (value 0 or invalid)
 *          
 *          Typical RXRSSI Interpretation:
 *          - **>80**: Excellent signal, full control authority
 *          - **60-80**: Good signal, normal operation
 *          - **40-60**: Marginal signal, approaching max range
 *          - **<40**: Weak signal, RC failsafe imminent
 *          - **0**: No signal or not supported by receiver
 *          
 *          RXLQ (Received Link Quality):
 *          Percentage of successfully received packets (packet success rate):
 *          - **100%**: Perfect link, no packet loss
 *          - **90-100%**: Excellent link quality
 *          - **70-90%**: Acceptable quality, some packet loss
 *          - **<70%**: Poor quality, frequent packet loss
 *          - **<50%**: Severe degradation, RC failsafe likely
 *          
 *          Link Quality vs RSSI:
 *          - **RSSI**: Raw signal strength; affected by distance, obstacles, interference
 *          - **LQ**: Actual data integrity; better indicator of usable link
 *          - **High RSSI + Low LQ**: Strong signal but corrupted (interference)
 *          - **Low RSSI + High LQ**: Weak but clean signal (at range limit)
 *          
 *          RC Protocol Differences:
 *          Different RC protocols provide different quality of RSSI/LQ data:
 *          - **CRSF/ExpressLRS**: Excellent telemetry, accurate RSSI and LQ
 *          - **FrSky**: Good RSSI reporting via telemetry
 *          - **SBUS**: Basic RSSI if receiver supports it
 *          - **DSM2/DSMX**: RSSI in dBm units
 *          - **PPM**: No RSSI available (legacy protocol)
 *          
 *          Analysis Scenarios:
 *          @code
 *          // Excellent link - close range, clear line of sight
 *          RSSI: RXRSSI=95.0, RXLQ=100.0
 *          
 *          // Good link - mid-range operation
 *          RSSI: RXRSSI=75.0, RXLQ=98.5
 *          
 *          // Marginal link - approaching max range
 *          RSSI: RXRSSI=45.0, RXLQ=85.0
 *          
 *          // Failing link - imminent RC failsafe
 *          RSSI: RXRSSI=20.0, RXLQ=50.0
 *          
 *          // Interference - strong signal but data corrupted
 *          RSSI: RXRSSI=80.0, RXLQ=60.0
 *          @endcode
 *          
 *          Troubleshooting:
 *          - **RXRSSI declining with distance**: Normal; verify sufficient margin
 *          - **RXRSSI fluctuating rapidly**: Multipath fading; change antenna or position
 *          - **RXLQ < 90% at close range**: Interference or receiver issue
 *          - **Both metrics zero**: Receiver not reporting telemetry or not supported
 *          - **Sudden drops**: Obstacle blocking signal or orientation issue
 *          
 *          Range Testing Procedure:
 *          1. Log RSSI during controlled range test
 *          2. Note RXRSSI and RXLQ at max intended operational distance
 *          3. Ensure RXRSSI > 40 and RXLQ > 80% at max distance
 *          4. Verify graceful degradation with distance (not sudden dropouts)
 *          5. Test in multiple orientations (aircraft pitch/roll/yaw)
 *          
 *          Antenna Optimization:
 *          Use RSSI logs to optimize antenna placement:
 *          - **Vertical diversity**: Two antennas at 90° angle improves reliability
 *          - **Avoid carbon fiber**: Carbon blocks RF; mount antennas externally
 *          - **Ground plane**: Proper antenna ground plane improves pattern
 *          - **Polarization**: Match Tx and Rx antenna orientation
 * 
 * @note RSSI logged at RC input rate (typically 50-100 Hz)
 * @note RXRSSI units depend on RC protocol; not standardized across receivers
 * @note RXLQ is generally more reliable indicator than RXRSSI for link quality
 * @note Zero values may indicate receiver doesn't support telemetry
 * @note Some receivers only report RSSI, not LQ (LQ will be 0)
 * 
 * @warning RXLQ < 50% indicates imminent RC failsafe risk
 * @warning Rapid fluctuations in RXRSSI indicate problematic RF environment
 * 
 * @see log_RCIN for pilot input values
 * @see RSSI_TYPE parameter for configuring RSSI source
 * @see FS_THR_ENABLE for RC failsafe configuration
 */
struct PACKED log_RSSI {
    LOG_PACKET_HEADER;          ///< Standard 3-byte header: head1, head2, msgid
    uint64_t time_us;           ///< Timestamp in microseconds since system boot
    float RXRSSI;               ///< Receiver signal strength (protocol-dependent units, typically 0-100)
    float RXLQ;                 ///< Receiver link quality (packet success rate percentage, 0-100)
};

/**
 * @struct log_Optflow
 * @brief OF message: Optical flow sensor measurements for velocity estimation
 * 
 * @details The OF message logs data from optical flow sensors, which measure apparent motion
 *          of the ground surface below the vehicle. Optical flow is used to estimate horizontal
 *          velocity when GPS is unavailable or unreliable (indoor flight, GPS-denied environments).
 *          Combined with a rangefinder for altitude, optical flow enables GPS-independent position hold.
 *          
 *          OF Message Purpose:
 *          - **GPS-Independent Navigation**: Enable position hold without GPS
 *          - **Indoor Flight**: Precision hover in GPS-denied environments
 *          - **Sensor Fusion**: Provide velocity reference to EKF for improved accuracy
 *          - **Surface Tracking**: Detect ground movement below vehicle
 *          - **Drift Monitoring**: Identify unwanted horizontal drift in hover
 *          
 *          Optical Flow Principle:
 *          Optical flow sensors work like an optical mouse, tracking surface features:
 *          1. Camera captures sequential images of ground below
 *          2. Image processing detects feature movement between frames
 *          3. Movement scaled by altitude gives horizontal velocity
 *          4. Works best over textured surfaces (grass, carpet, patterned floors)
 *          
 *          Surface Quality:
 *          surface_quality indicates how well the sensor can track surface features:
 *          - **255**: Excellent surface with high-contrast features
 *          - **200-254**: Good tracking, adequate features
 *          - **100-199**: Marginal tracking, low-contrast surface
 *          - **<100**: Poor tracking, bland or featureless surface
 *          - **0**: No tracking possible or sensor error
 *          
 *          Quality affects measurement confidence:
 *          - EKF weights flow data based on surface_quality
 *          - Low quality → reduced influence on position estimate
 *          - High quality → increased confidence in flow measurements
 *          
 *          Flow Measurements (flow_x, flow_y):
 *          Raw flow rates in sensor's native units (typically radians/second or pixels/frame):
 *          - **flow_x**: Flow rate in sensor X-axis (typically forward/back)
 *          - **flow_y**: Flow rate in sensor Y-axis (typically left/right)
 *          - Positive/negative conventions depend on sensor orientation
 *          - Units are sensor-specific; converted to velocity using altitude
 *          
 *          Derived Body Velocities (body_x, body_y):
 *          Velocity estimates in vehicle body frame (meters/second):
 *          - **body_x**: Forward velocity (positive = forward motion)
 *          - **body_y**: Rightward velocity (positive = right motion)
 *          - Calculated from: flow_rate * altitude_AGL * calibration_factor
 *          - Compensated for vehicle attitude (pitch/roll)
 *          
 *          Body Frame Orientation:
 *          - +X: Forward (nose direction)
 *          - +Y: Right (right wing direction)
 *          - +Z: Down (toward ground)
 *          
 *          Supported Optical Flow Sensors:
 *          - **PX4Flow**: Smart camera with onboard processing
 *          - **Cheerson CX-OF**: Lightweight optical flow module
 *          - **HereFlow**: CAN-based optical flow and rangefinder
 *          - **MSP optical flow**: Via MSP protocol from external processor
 *          
 *          Analysis Scenarios:
 *          @code
 *          // Perfect hover over textured surface
 *          OF: Qual=250, flowX=0.01, flowY=-0.02, bodyX=0.0, bodyY=0.0
 *          
 *          // Forward flight at 2 m/s over good surface
 *          OF: Qual=240, flowX=15.0, flowY=0.5, bodyX=2.0, bodyY=0.0
 *          
 *          // Hover over bland surface - poor quality
 *          OF: Qual=50, flowX=5.0, flowY=5.0, bodyX=0.5, bodyY=0.5
 *              (low quality → unreliable velocity estimates)
 *          
 *          // Sensor obstructed or failed
 *          OF: Qual=0, flowX=0.0, flowY=0.0, bodyX=0.0, bodyY=0.0
 *          @endcode
 *          
 *          Optimal Operating Conditions:
 *          - **Altitude**: 0.5-3m AGL (altitude above ground level)
 *          - **Surface**: High-contrast texture (grass, patterned floor)
 *          - **Lighting**: Adequate illumination (500+ lux)
 *          - **Motion**: Moderate speeds (<5 m/s for best accuracy)
 *          
 *          Challenging Conditions:
 *          - **Bland surfaces**: Smooth concrete, plain carpet, water
 *          - **Low light**: Twilight, shadows, indoor dim lighting
 *          - **High altitude**: >5m AGL reduces accuracy
 *          - **Fast motion**: >10 m/s exceeds sensor tracking rate
 *          - **Reflective surfaces**: Wet floors, glossy finishes
 *          
 *          Integration with EKF:
 *          Optical flow data fused into Extended Kalman Filter for velocity estimate:
 *          1. Flow measurements converted to velocity using rangefinder altitude
 *          2. EKF compares flow velocity with IMU-derived velocity
 *          3. Innovation (difference) used to correct position estimate
 *          4. surface_quality modulates measurement confidence
 *          
 *          Troubleshooting:
 *          - **Qual always low**: Poor surface, insufficient lighting, or sensor misaligned
 *          - **body_x/y don't match actual velocity**: Sensor calibration needed (FLOW_FXSCALER/FLOW_FYSCALER)
 *          - **Erratic velocities**: Sensor vibration or altitude changes
 *          - **Zero flow during motion**: Sensor failed, obstructed, or disconnected
 *          - **Flow opposes motion**: Sensor mounted backwards (check FLOW_ORIENT_YAW)
 * 
 * @note OF logged at sensor update rate (typically 10-20 Hz)
 * @note Optical flow requires rangefinder to convert flow to velocity
 * @note EKF3 with AHRS_EKF_TYPE=3 required for flow fusion
 * @note Enable with EK3_SRC1_VELXY = 6 (optical flow)
 * @note Sensor orientation configured via FLOW_ORIENT_YAW parameter
 * 
 * @warning Optical flow fails over featureless surfaces (water, smooth floors)
 * @warning Flow measurements degrade rapidly above 3-5m altitude
 * @warning Sensor must be mounted looking straight down (perpendicular to ground)
 * 
 * @see AP_OpticalFlow for optical flow driver implementation
 * @see AP_NavEKF3 for optical flow fusion into state estimation
 * @see FLOW_TYPE parameter for sensor selection
 * @see EK3_SRC1_VELXY parameter to enable flow as velocity source
 */
struct PACKED log_Optflow {
    LOG_PACKET_HEADER;          ///< Standard 3-byte header: head1, head2, msgid
    uint64_t time_us;           ///< Timestamp in microseconds since system boot
    uint8_t surface_quality;    ///< Surface feature quality (0-255, higher=better tracking)
    float flow_x;               ///< Raw flow rate in sensor X-axis (sensor-specific units)
    float flow_y;               ///< Raw flow rate in sensor Y-axis (sensor-specific units)
    float body_x;               ///< Forward velocity in body frame (m/s, positive forward)
    float body_y;               ///< Rightward velocity in body frame (m/s, positive right)
};

/**
 * @struct log_POWR
 * @brief POWR message: System power monitoring including voltages, flags, and safety status
 * 
 * @details The POWR message logs critical power system health data including flight controller
 *          supply voltage, servo rail voltage, power fault flags, and safety switch status.
 *          This information is essential for diagnosing power-related failures, brownouts,
 *          and hardware safety mechanisms.
 *          
 *          POWR Message Purpose:
 *          - **Power System Health**: Monitor voltage stability and detect brownouts
 *          - **Fault Detection**: Identify power supply issues before they cause failure
 *          - **Servo Power Monitoring**: Verify adequate power for high-current servos/ESCs
 *          - **Safety Verification**: Confirm hardware safety switch state
 *          - **Post-Flight Analysis**: Diagnose power-related anomalies or crashes
 *          
 *          Vcc (Flight Controller Supply Voltage):
 *          Main supply voltage to the flight controller processor and peripherals:
 *          - **Typical Range**: 4.5V - 5.5V (depends on board design)
 *          - **Nominal**: 5.0V ± 0.3V
 *          - **Warning**: <4.8V or >5.3V indicates power supply stress
 *          - **Critical**: <4.5V may cause brownout reset or malfunction
 *          
 *          Vcc Sources:
 *          - USB power (when connected to computer)
 *          - Power module (BEC output)
 *          - Servo rail (if powered and jumpered)
 *          - Dedicated 5V BEC
 *          
 *          Vservo (Servo Rail Voltage):
 *          Voltage on the servo output rail powering servos and peripherals:
 *          - **Unpowered**: 0V (servo rail not connected or safety engaged)
 *          - **Standard Servos**: 4.8V - 6.0V
 *          - **High-Voltage Servos**: 7.4V - 8.4V
 *          - **ESC Power**: Varies by ESC BEC design
 *          
 *          Servo Rail Configurations:
 *          - **Powered Servo Rail**: External BEC provides power via servo connector
 *          - **Unpowered Servo Rail**: Signal only, servos/ESCs have their own power
 *          - **USB Safety**: Some boards isolate servo rail when on USB power
 *          
 *          Flags (Power Status Flags Bitmask):
 *          Real-time power fault indicators (AP_HAL::AnalogIn::PowerStatusFlag):
 *          - **Bit 0 (0x01)**: VCC power fault (flight controller undervoltage)
 *          - **Bit 1 (0x02)**: VSERVO power fault (servo rail undervoltage/overvoltage)
 *          - **Bit 2 (0x04)**: VCC overcurrent (excessive current draw)
 *          - **Bit 3 (0x08)**: VSERVO overcurrent (servo rail overloaded)
 *          - **Bit 4 (0x10)**: Brick valid (power module connected and valid)
 *          - **Bit 5 (0x20)**: Brick in use (system powered from power module)
 *          - **Bit 6 (0x40)**: Servo rail valid
 *          - **Bit 7 (0x80)**: Peripheral overcurrent
 *          
 *          Accumulated_Flags:
 *          Cumulative record of all power faults that have occurred since boot:
 *          - Once a bit is set in accumulated_flags, it remains set
 *          - Used to detect transient power events that may have been missed
 *          - Critical for post-flight analysis of intermittent power issues
 *          
 *          Safety_and_Arm:
 *          Combined safety switch and arming state (board-specific encoding):
 *          - **Bit 0**: Armed state (1=armed, 0=disarmed)
 *          - **Bit 1**: Safety switch state (1=safety off, 0=safety on)
 *          - **Bit 2-7**: Reserved for future use
 *          
 *          Hardware Safety Switch:
 *          Physical button on some flight controllers to prevent accidental arming:
 *          - **Safety ON**: Motors/servos disabled regardless of arm state
 *          - **Safety OFF**: Normal operation, arming allowed
 *          - **Not Present**: Some boards lack hardware safety switch
 *          
 *          Analysis Scenarios:
 *          @code
 *          // Normal operation - healthy power
 *          POWR: Vcc=5.1, Vservo=5.0, flags=0x30, AccFlags=0x30, Safety=3
 *                (brick in use, servo rail valid, armed, safety off)
 *          
 *          // Low Vcc - power supply struggling
 *          POWR: Vcc=4.6, Vservo=5.0, flags=0x31, AccFlags=0x31, Safety=3
 *                (VCC fault bit set)
 *          
 *          // Servo rail unpowered (signal-only configuration)
 *          POWR: Vcc=5.1, Vservo=0.0, flags=0x20, AccFlags=0x20, Safety=3
 *                (brick in use, no servo power)
 *          
 *          // Disarmed with safety on
 *          POWR: Vcc=5.0, Vservo=0.0, flags=0x20, AccFlags=0x20, Safety=0
 *                (safety engaged, disarmed)
 *          
 *          // Power fault during flight
 *          POWR: Vcc=4.2, Vservo=4.5, flags=0x33, AccFlags=0x33, Safety=3
 *                (VCC fault, VSERVO fault - critical!)
 *          @endcode
 *          
 *          Common Power Issues:
 *          - **Vcc Drops During Motor Spin-Up**: Insufficient power module capacity
 *          - **Vservo = 0 When Expected**: BEC failure, connector issue, or no external power
 *          - **Flags != 0**: Active power fault requiring immediate attention
 *          - **Accumulated_Flags Growing**: Intermittent power problems
 *          - **Vcc Noise**: Rapid fluctuations indicate poor filtering or loose connection
 *          
 *          Troubleshooting Guide:
 *          - **Low Vcc (<4.8V)**: Upgrade power module, check wiring, reduce peripheral load
 *          - **Vcc Spikes (>5.3V)**: Check BEC regulation, verify power module compatibility
 *          - **Vservo = 0**: Verify external BEC connected and powered
 *          - **Power Faults**: flags bit set indicates immediate hardware issue
 *          - **Accumulated Faults**: Review full log for transient events
 *          
 *          Power Module Selection:
 *          - Match power module current rating to total system draw
 *          - Multicopter: 50-60A per kg of AUW (all-up weight)
 *          - Fixed-wing: 20-40A per kg of AUW
 *          - Include 30% margin for transients and motor startup
 *          
 *          Safety Switch Operation:
 *          - Present on Pixhawk series and some other boards
 *          - Red LED blinks fast = safety on (motors disabled)
 *          - Red LED solid = safety off (arming allowed)
 *          - Can be disabled via BRD_SAFETYENABLE=0
 * 
 * @note POWR logged periodically (typically 1-10 Hz)
 * @note Voltage readings accuracy depends on board design and calibration
 * @note Not all boards support all voltage monitoring features
 * @note Some boards lack hardware safety switch (safety_and_arm will only show arm state)
 * 
 * @warning Vcc < 4.5V may cause brownout and unpredictable behavior
 * @warning flags != 0 indicates active power fault requiring immediate investigation
 * @warning accumulated_flags != flags indicates past transient power events
 * 
 * @see AP_HAL::AnalogIn::PowerStatusFlag for flags bit definitions
 * @see BRD_VBUS_MIN, BRD_VSERVO_MIN parameters for voltage thresholds
 * @see BRD_SAFETYENABLE parameter to enable/disable safety switch
 */
struct PACKED log_POWR {
    LOG_PACKET_HEADER;          ///< Standard 3-byte header: head1, head2, msgid
    uint64_t time_us;           ///< Timestamp in microseconds since system boot
    float Vcc;                  ///< Flight controller supply voltage (V)
    float Vservo;               ///< Servo rail voltage (V, 0 if unpowered or isolated)
    uint16_t flags;             ///< Current power status flags (AP_HAL::AnalogIn::PowerStatusFlag bitmask)
    uint16_t accumulated_flags; ///< Cumulative power flags since boot (all faults ever seen)
    uint8_t safety_and_arm;     ///< Combined safety switch and arming state
};

/**
 * @struct log_MCU
 * @brief MCU message: Microcontroller temperature and internal voltage monitoring
 * 
 * @details The MCU message logs internal microcontroller health metrics including die temperature
 *          and internal reference voltage. These parameters help diagnose thermal issues, voltage
 *          regulation problems, and provide early warning of hardware stress conditions.
 *          
 *          MCU Message Purpose:
 *          - **Thermal Monitoring**: Track processor temperature to detect overheating
 *          - **Voltage Stability**: Monitor internal MCU voltage regulation
 *          - **Hardware Health**: Identify stress conditions before failure occurs
 *          - **Environmental Limits**: Verify operation within temperature specifications
 *          - **Crash Analysis**: Determine if thermal/voltage issues contributed to failure
 *          
 *          MCU_temp (Microcontroller Die Temperature):
 *          Internal temperature of the microcontroller silicon die:
 *          - **Typical Range**: -40°C to +85°C (industrial temperature range)
 *          - **Normal Operation**: 30°C - 60°C (ambient + self-heating)
 *          - **Warning**: >70°C indicates poor cooling or high ambient temperature
 *          - **Critical**: >85°C approaching absolute maximum rating
 *          - **Extended Range**: Some MCUs rated to +125°C (automotive grade)
 *          
 *          Temperature Measurement:
 *          - Read from internal temperature sensor in MCU
 *          - Accuracy typically ±3-5°C
 *          - Not calibrated (absolute accuracy limited)
 *          - Useful for trend monitoring rather than absolute values
 *          
 *          Self-Heating:
 *          Processor generates heat proportional to computational load:
 *          - Idle/Low Load: +5-10°C above ambient
 *          - Normal Flight: +15-25°C above ambient
 *          - Heavy Load (logging, scripting, complex tasks): +25-35°C above ambient
 *          - Depends on clock speed, peripherals active, and enclosure ventilation
 *          
 *          MCU_voltage (Current Internal Voltage):
 *          Internal MCU core voltage regulation:
 *          - **STM32F4/F7**: Typically 1.2V core, 3.3V I/O
 *          - **STM32H7**: Typically 1.2V core, 3.3V I/O
 *          - **Nominal**: ±5% of specification
 *          - **Out of Range**: Indicates voltage regulator or power supply issue
 *          
 *          MCU_voltage_min (Minimum Voltage Since Boot):
 *          Lowest internal voltage observed since system startup:
 *          - Captures transient voltage sags during high-current events
 *          - Useful for detecting momentary brownouts
 *          - Resets on boot, accumulates during flight
 *          
 *          MCU_voltage_max (Maximum Voltage Since Boot):
 *          Highest internal voltage observed since system startup:
 *          - Captures voltage spikes from power supply transients
 *          - May indicate overvoltage conditions
 *          - Resets on boot, accumulates during flight
 *          
 *          Analysis Scenarios:
 *          @code
 *          // Normal operation - healthy MCU
 *          MCU: temp=45.0, volt=1.21, vmin=1.20, vmax=1.22
 *               (moderate temperature, stable voltage)
 *          
 *          // Hot environment or poor cooling
 *          MCU: temp=75.0, volt=1.19, vmin=1.18, vmax=1.22
 *               (high temperature, slight voltage droop under heat)
 *          
 *          // Cold start
 *          MCU: temp=-10.0, volt=1.22, vmin=1.20, vmax=1.24
 *               (cold operation, voltage slightly higher when cold)
 *          
 *          // Voltage instability detected
 *          MCU: temp=50.0, volt=1.20, vmin=1.10, vmax=1.25
 *               (large min/max spread indicates power supply noise)
 *          
 *          // Thermal emergency
 *          MCU: temp=88.0, volt=1.17, vmin=1.15, vmax=1.21
 *               (approaching temperature limit, voltage dropping)
 *          @endcode
 *          
 *          Temperature Factors:
 *          - **Ambient Temperature**: Base temperature of environment
 *          - **Airflow**: Cooling from propwash, forward flight, or fans
 *          - **Enclosure**: Sealed cases trap heat, vented cases improve cooling
 *          - **Direct Sunlight**: Can add 20-40°C to board temperature
 *          - **Computational Load**: Logging, scripting, image processing increase heat
 *          
 *          Thermal Management:
 *          - **Ventilation**: Ensure airflow around flight controller
 *          - **Mounting**: Avoid mounting on hot surfaces (ESCs, motors, battery)
 *          - **Enclosure Design**: Use vented cases or heat-conductive mounting
 *          - **Load Reduction**: Disable unnecessary features if overheating occurs
 *          - **Environmental**: Avoid operating in extreme temperatures
 *          
 *          Voltage Stability Indicators:
 *          - **Small min/max spread (<0.05V)**: Clean, stable power supply
 *          - **Large min/max spread (>0.10V)**: Noisy or marginal power supply
 *          - **Trending down**: Power supply degradation or overload
 *          - **Trending up**: Possible voltage regulator failure
 *          
 *          Common Issues:
 *          - **High Temperature**: Poor ventilation, hot environment, or direct sun exposure
 *          - **Temperature Climbing**: Thermal runaway from inadequate cooling
 *          - **Low MCU_voltage_min**: Brownout events, possibly causing resets or errors
 *          - **High MCU_voltage_max**: Power supply spikes, overvoltage risk
 *          - **Voltage Fluctuations**: Large min/max spread indicates power quality issues
 *          
 *          Troubleshooting:
 *          - **Temp > 70°C**: Improve cooling, relocate away from heat sources
 *          - **Temp > 85°C**: Immediate action required - land and investigate
 *          - **MCU_voltage_min < 1.10V**: Check power supply, reduce peripheral load
 *          - **MCU_voltage_max > 1.30V**: Check power supply regulation
 *          - **Unstable voltage**: Replace power supply, check connections
 *          
 *          Board-Specific Considerations:
 *          - Not all MCUs support temperature sensing (may show constant value)
 *          - STM32 MCUs have internal temperature sensor
 *          - SITL (simulation) may show dummy values
 *          - Linux-based boards may report different voltage rails
 * 
 * @note MCU logged periodically (typically 1-10 Hz)
 * @note Temperature accuracy ±3-5°C, useful for trends not absolute values
 * @note Not all boards/MCUs support internal temperature sensing
 * @note Voltage readings are internal MCU core voltage, not board supply
 * @note Min/max values reset on boot, track extremes during flight
 * 
 * @warning MCU_temp > 85°C indicates operation beyond safe limits
 * @warning Large voltage min/max spread suggests power supply instability
 * @warning Continuous temperature increase may lead to thermal shutdown
 * 
 * @see AP_HAL for MCU hardware abstraction
 * @see BRD_TYPE parameter identifies flight controller board
 */
struct PACKED log_MCU {
    LOG_PACKET_HEADER;          ///< Standard 3-byte header: head1, head2, msgid
    uint64_t time_us;           ///< Timestamp in microseconds since system boot
    float MCU_temp;             ///< Microcontroller die temperature (°C)
    float MCU_voltage;          ///< Current internal MCU core voltage (V)
    float MCU_voltage_min;      ///< Minimum MCU voltage observed since boot (V)
    float MCU_voltage_max;      ///< Maximum MCU voltage observed since boot (V)
};

/**
 * @struct log_MAVLink_Command
 * @brief MAVC message: Record of MAVLink command received and executed with result
 * 
 * @details The MAVC message logs every MAVLink command (COMMAND_LONG or COMMAND_INT) received
 *          by the autopilot, including all command parameters, routing information, and the
 *          result of command execution. This provides complete audit trail of all commands
 *          sent from ground control stations, companion computers, or other MAVLink nodes.
 *          
 *          MAVC Message Purpose:
 *          - **Command Audit Trail**: Record every command received for post-flight analysis
 *          - **Debugging**: Diagnose command failures and incorrect parameter usage
 *          - **Security**: Track which systems sent which commands
 *          - **Mission Analysis**: Review automated mission command execution
 *          - **Integration Testing**: Verify companion computer commanding behavior
 *          
 *          MAVLink Command Protocol:
 *          Commands are discrete actions or queries sent to the autopilot:
 *          - **COMMAND_LONG**: 7 float parameters (P1-P7), no position
 *          - **COMMAND_INT**: 4 float parameters + integer position (X, Y, Z)
 *          - **Acknowledgment**: Autopilot sends COMMAND_ACK with result code
 *          - **Retry Logic**: GCS may retry failed commands
 *          
 *          System/Component Addressing:
 *          MAVLink uses two-level addressing for routing:
 *          - **System ID**: Vehicle or node identifier (1-255)
 *          - **Component ID**: Component within system (autopilot=1, camera=100, etc.)
 *          - **Broadcast**: sysid=0 or compid=0 sends to all matching recipients
 *          
 *          target_system / target_component:
 *          Intended recipient of the command:
 *          - **target_system=1**: Commands for vehicle 1 (most common)
 *          - **target_component=1**: Autopilot component (MAV_COMP_ID_AUTOPILOT1)
 *          - Commands only executed if target matches this autopilot's IDs
 *          
 *          source_system / source_component:
 *          Origin of the command:
 *          - **source_system**: GCS system ID (often 255) or companion computer ID
 *          - **source_component**: Sending component (GCS=190, onboard computer=191)
 *          - Used for routing ACK response back to sender
 *          
 *          frame (MAV_FRAME):
 *          Coordinate frame for position-based commands (X, Y, Z):
 *          - **0 (GLOBAL)**: WGS84 global coordinate frame (lat/lon/alt MSL)
 *          - **3 (GLOBAL_RELATIVE_ALT)**: Global frame, altitude relative to home
 *          - **5 (LOCAL_ENU)**: Local East-North-Up tangent plane
 *          - **6 (LOCAL_NED)**: Local North-East-Down tangent plane
 *          - **10 (MISSION)**: Frame specified by mission item frame
 *          - Only relevant for COMMAND_INT with position parameters
 *          
 *          command (MAV_CMD):
 *          Command identifier from MAVLink MAV_CMD enum:
 *          - **16 (NAV_WAYPOINT)**: Navigate to waypoint
 *          - **20 (NAV_RETURN_TO_LAUNCH)**: Return to launch point
 *          - **21 (NAV_LAND)**: Land at current position
 *          - **22 (NAV_TAKEOFF)**: Takeoff to specified altitude
 *          - **176 (DO_SET_MODE)**: Change flight mode
 *          - **400 (COMPONENT_ARM_DISARM)**: Arm or disarm motors
 *          - **500 (NAV_GUIDED_ENABLE)**: Enable/disable guided mode
 *          - **2500 (REQUEST_AUTOPILOT_CAPABILITIES)**: Request capabilities
 *          - **2502 (REQUEST_STORAGE_INFORMATION)**: Request storage info
 *          - 300+ standard commands defined in MAVLink
 *          
 *          param1 - param4:
 *          Command-specific parameters (meaning varies by command):
 *          @code
 *          // MAV_CMD_COMPONENT_ARM_DISARM (400)
 *          param1: 1=arm, 0=disarm
 *          param2: 21196=force arming (bypass safety checks)
 *          
 *          // MAV_CMD_DO_SET_MODE (176)
 *          param1: MAV_MODE base mode flags
 *          param2: custom_mode (ArduPilot-specific mode number)
 *          
 *          // MAV_CMD_NAV_TAKEOFF (22)
 *          param1: Pitch angle in degrees
 *          param4: Yaw angle in degrees
 *          param7 (z): Altitude to takeoff to
 *          @endcode
 *          
 *          x, y, z (Position Parameters):
 *          Integer and float position for COMMAND_INT:
 *          - **x (int32)**: Latitude in degE7 (degrees * 1E7) OR local X in meters
 *          - **y (int32)**: Longitude in degE7 (degrees * 1E7) OR local Y in meters
 *          - **z (float)**: Altitude in meters (frame-dependent)
 *          - For COMMAND_LONG, these contain param5, param6, param7 as floats
 *          
 *          Example Position Encoding:
 *          @code
 *          // Latitude 47.641468° → x = 476414680 (degE7)
 *          // Longitude -122.140165° → y = -1221401650 (degE7)
 *          // Altitude 100m MSL → z = 100.0
 *          @endcode
 *          
 *          result (MAV_RESULT):
 *          Command execution result returned to sender:
 *          - **0 (MAV_RESULT_ACCEPTED)**: Command executed successfully
 *          - **1 (MAV_RESULT_TEMPORARILY_REJECTED)**: Cannot execute now, retry later
 *          - **2 (MAV_RESULT_DENIED)**: Command rejected, don't retry
 *          - **3 (MAV_RESULT_UNSUPPORTED)**: Command not implemented
 *          - **4 (MAV_RESULT_FAILED)**: Execution failed
 *          - **5 (MAV_RESULT_IN_PROGRESS)**: Command being executed
 *          
 *          was_command_long:
 *          Identifies which MAVLink message type carried this command:
 *          - **true**: Received as COMMAND_LONG (message ID 76)
 *          - **false**: Received as COMMAND_INT (message ID 75)
 *          - COMMAND_INT preferred for position-based commands (integer lat/lon)
 *          
 *          Analysis Scenarios:
 *          @code
 *          // Successful arm command from GCS
 *          MAVC: TS=1, TC=1, SS=255, SC=190, Fr=0, Cmd=400, P1=1.0, P2=0, Result=0, WL=1
 *                (Arm command accepted)
 *          
 *          // Mode change to Auto
 *          MAVC: TS=1, TC=1, SS=255, SC=190, Fr=0, Cmd=176, P1=4, P2=3, Result=0, WL=1
 *                (Set mode to Auto accepted)
 *          
 *          // Takeoff command rejected (not armed)
 *          MAVC: TS=1, TC=1, SS=255, SC=190, Fr=0, Cmd=22, P4=0, Z=50.0, Result=2, WL=1
 *                (Takeoff command denied)
 *          
 *          // Companion computer sending guided waypoint
 *          MAVC: TS=1, TC=1, SS=2, SC=191, Fr=3, Cmd=16, X=476414680, Y=-1221401650, Z=100, Result=0, WL=0
 *                (Guided waypoint from companion computer, COMMAND_INT)
 *          @endcode
 *          
 *          Common Commands:
 *          - **400 (ARM_DISARM)**: Most common command, arms/disarms motors
 *          - **176 (SET_MODE)**: Mode changes (Auto, Guided, RTL, etc.)
 *          - **20 (RTL)**: Return to launch
 *          - **21 (LAND)**: Land now
 *          - **84 (NAV_GUIDED_ENABLE)**: Enter guided mode
 *          - **3000 (SET_HOME)**: Set home position
 *          - **2800 (PANORAMA_CREATE)**: Trigger camera panorama
 *          
 *          Troubleshooting:
 *          - **Result=2 (DENIED)**: Check arming state, flight mode, safety checks
 *          - **Result=3 (UNSUPPORTED)**: Command not available on this vehicle type
 *          - **Result=4 (FAILED)**: Execution started but failed (check other logs)
 *          - **Multiple retries**: GCS not receiving ACK, check telemetry link
 *          - **Unexpected source_system**: Unknown device sending commands
 * 
 * @note MAVC logged for every command received, regardless of result
 * @note Commands may be logged multiple times if GCS retries
 * @note Not all parameters used by all commands (unused params typically 0)
 * @note Position parameters (x, y, z) only meaningful for position-based commands
 * 
 * @warning Unexpected commands from unknown source_system may indicate security issue
 * @warning result != 0 indicates command not executed as intended
 * 
 * @see MAV_CMD enum in MAVLink common.xml for complete command list
 * @see MAV_RESULT enum for result code definitions
 * @see MAV_FRAME enum for coordinate frame definitions
 * @see GCS_MAVLink::handle_command_long() and handle_command_int()
 */
struct PACKED log_MAVLink_Command {
    LOG_PACKET_HEADER;          ///< Standard 3-byte header: head1, head2, msgid
    uint64_t time_us;           ///< Timestamp in microseconds since system boot
    uint8_t target_system;      ///< Target system ID (recipient of command)
    uint8_t target_component;   ///< Target component ID within system (1=autopilot)
    uint8_t source_system;      ///< Source system ID (sender of command, often 255=GCS)
    uint8_t source_component;   ///< Source component ID (190=GCS, 191=companion computer)
    uint8_t frame;              ///< MAV_FRAME coordinate frame for position parameters
    uint16_t command;           ///< MAV_CMD command ID (e.g., 400=ARM_DISARM, 176=SET_MODE)
    float param1;               ///< Command-specific parameter 1
    float param2;               ///< Command-specific parameter 2
    float param3;               ///< Command-specific parameter 3
    float param4;               ///< Command-specific parameter 4
    int32_t x;                  ///< Latitude in degE7 or local X in meters (command-dependent)
    int32_t y;                  ///< Longitude in degE7 or local Y in meters (command-dependent)
    float z;                    ///< Altitude in meters or param7 (frame-dependent)
    uint8_t result;             ///< MAV_RESULT execution result (0=accepted, 2=denied, 3=unsupported, 4=failed)
    bool was_command_long;      ///< true if received as COMMAND_LONG, false if COMMAND_INT
};

/**
 * @struct log_Radio
 * @brief RAD message: Telemetry radio statistics for serial telemetry links (SiK radio)
 * 
 * @details The RAD message logs detailed performance statistics from serial telemetry radios,
 *          particularly SiK-based radio modems (RFD900, 3DR Radio, HopeRF modules). This data
 *          helps diagnose link quality issues, range problems, and interference conditions.
 *          
 *          RAD Message Purpose:
 *          - **Link Quality Assessment**: Monitor radio signal strength and reliability
 *          - **Range Testing**: Verify adequate link margin for planned operations
 *          - **Interference Detection**: Identify RF noise and packet corruption
 *          - **Configuration Optimization**: Tune radio parameters for best performance
 *          - **Troubleshooting**: Diagnose link failures and dropout causes
 *          
 *          SiK Radio Overview:
 *          SiK firmware is open-source radio firmware for ISM band telemetry:
 *          - **Frequency Bands**: 433 MHz, 868 MHz, 915 MHz (region-dependent)
 *          - **Air Data Rate**: 4-250 kbps (configurable)
 *          - **Transmit Power**: 1mW - 1W (30 dBm) depending on module
 *          - **Forward Error Correction**: Reed-Solomon coding for reliability
 *          - **Frequency Hopping**: FHSS to avoid interference
 *          - **Bi-directional**: Statistics from both local and remote radio
 *          
 *          rssi (Local Received Signal Strength Indicator):
 *          Signal strength of packets received BY this radio FROM remote radio:
 *          - **Units**: dBm (decibel-milliwatts) on some radios, 0-255 scale on others
 *          - **SiK Radios**: 0-255 scale (higher is better)
 *          - **Typical Good**: >180 (close range), >100 (medium range), >50 (long range)
 *          - **Warning**: <50 indicates weak signal, expect dropouts
 *          - **Critical**: <30 link unreliable, may lose connection
 *          
 *          RSSI Interpretation:
 *          @code
 *          rssi = 200-255: Excellent signal (very close, strong link)
 *          rssi = 150-199: Good signal (normal operational range)
 *          rssi = 100-149: Fair signal (acceptable with margin)
 *          rssi = 50-99:   Poor signal (marginal link, prone to errors)
 *          rssi = 0-49:    Very poor signal (unreliable, imminent loss)
 *          @endcode
 *          
 *          remrssi (Remote Received Signal Strength Indicator):
 *          Signal strength of packets received BY remote radio FROM this radio:
 *          - Same scale as rssi (0-255)
 *          - Indicates how well ground station receives vehicle transmissions
 *          - Asymmetric links common: one direction stronger than other
 *          - Different transmit powers or antenna gains create asymmetry
 *          
 *          Link Asymmetry:
 *          rssi and remrssi often differ due to:
 *          - Different transmit power settings (vehicle vs ground station)
 *          - Antenna differences (ground station often has better antenna)
 *          - Orientation (vehicle antenna may be directional)
 *          - Interference (different at each location)
 *          
 *          txbuf (Transmit Buffer Usage):
 *          Percentage of radio transmit buffer currently filled:
 *          - **Range**: 0-100 (percentage)
 *          - **Normal**: <50% (keeping up with data rate)
 *          - **Warning**: >70% (buffer filling, may cause latency)
 *          - **Critical**: >90% (buffer near full, data being dropped)
 *          
 *          High txbuf Causes:
 *          - Telemetry stream rates exceed radio air data rate
 *          - Poor link quality (retransmissions consume bandwidth)
 *          - Radio air data rate too low for data volume
 *          - MAVLink traffic spikes (mission upload, parameter download)
 *          
 *          noise (Local Noise Floor):
 *          Background RF noise level measured BY this radio:
 *          - **Units**: Same scale as RSSI (0-255)
 *          - **Ideal**: <50 (quiet RF environment)
 *          - **Typical**: 50-100 (some background noise)
 *          - **High**: >100 (significant interference)
 *          - **Very High**: >150 (severe interference, link degraded)
 *          
 *          remnoise (Remote Noise Floor):
 *          Background RF noise level measured BY remote radio:
 *          - Same scale as noise (0-255)
 *          - Indicates RF environment at remote end
 *          - Different noise levels at each end common
 *          
 *          Signal-to-Noise Ratio (SNR):
 *          Critical metric for link quality:
 *          - **SNR = rssi - noise** (higher is better)
 *          - **Excellent**: SNR > 100 (robust link)
 *          - **Good**: SNR = 50-100 (reliable link)
 *          - **Fair**: SNR = 20-50 (marginal link)
 *          - **Poor**: SNR < 20 (unreliable, high error rate)
 *          
 *          rxerrors (Receive Error Count):
 *          Number of corrupted packets received (failed CRC):
 *          - Cumulative count since radio boot
 *          - Increments when received packet fails checksum
 *          - High error rate indicates poor link or interference
 *          - Some errors normal on long-range flights
 *          
 *          fixed (Forward Error Correction Fixes):
 *          Number of packets corrected by FEC:
 *          - Cumulative count since radio boot
 *          - SiK uses Reed-Solomon error correction
 *          - Can correct small number of bit errors per packet
 *          - High fixed count indicates link near error threshold
 *          
 *          FEC Performance:
 *          - **fixed << rxerrors**: FEC working well, catching errors
 *          - **fixed ≈ rxerrors**: Link marginal, FEC at capacity
 *          - **rxerrors increasing**: Some errors uncorrectable
 *          
 *          Analysis Scenarios:
 *          @code
 *          // Excellent link quality - close range
 *          RAD: rssi=210, remrssi=205, txbuf=15, noise=40, remnoise=35, rxerrors=0, fixed=0
 *               (strong signal both directions, low noise, no errors)
 *          
 *          // Good link - normal operational range
 *          RAD: rssi=120, remrssi=130, txbuf=35, noise=60, remnoise=55, rxerrors=5, fixed=12
 *               (SNR=60/70, some FEC corrections, stable link)
 *          
 *          // Marginal link - near maximum range
 *          RAD: rssi=55, remrssi=48, txbuf=75, noise=45, remnoise=40, rxerrors=250, fixed=180
 *               (SNR=10/8, high error rate, nearing dropout)
 *          
 *          // Interference detected
 *          RAD: rssi=150, remrssi=155, txbuf=60, noise=120, remnoise=115, rxerrors=180, fixed=90
 *               (noise floor elevated, many errors despite moderate RSSI)
 *          
 *          // Buffer overflow - data rate too high
 *          RAD: rssi=180, remrssi=175, txbuf=95, noise=50, remnoise=45, rxerrors=5, fixed=2
 *               (good signal but buffer full, need to reduce telemetry rates)
 *          @endcode
 *          
 *          Common Issues:
 *          - **Low RSSI**: Excessive range, antenna issue, or transmit power too low
 *          - **High Noise**: RF interference from motors, video TX, Wi-Fi, or other radios
 *          - **High txbuf**: Telemetry stream rates exceed radio capacity
 *          - **High rxerrors**: Poor link quality or interference
 *          - **Asymmetric Link**: One direction much weaker (antenna or power issue)
 *          
 *          Troubleshooting:
 *          - **Weak Signal**: Increase transmit power, improve antennas, reduce range
 *          - **High Noise**: Relocate ground station, shield radios, change frequency
 *          - **Buffer Full**: Reduce SR* parameters, lower logging rate, increase air data rate
 *          - **High Errors**: Improve SNR, enable/increase FEC, reduce interference
 *          - **One-way Link**: Check transmit power settings, antenna connections
 *          
 *          Radio Parameter Optimization:
 *          - **Air Data Rate**: Balance speed vs range (lower = longer range)
 *          - **Transmit Power**: Increase for range, but check local regulations
 *          - **Duty Cycle**: Some regions limit transmit time percentage
 *          - **Error Correction**: Enable for marginal links (reduces throughput)
 * 
 * @note RAD message only present when using SiK-compatible telemetry radio
 * @note Statistics cumulative since radio power-on (not autopilot boot)
 * @note Not all radios support all statistics (may show 0 for unsupported fields)
 * @note RSSI/noise scales vary by radio firmware version
 * 
 * @warning rssi or remrssi < 50 indicates unreliable link
 * @warning High noise (>100) suggests strong RF interference
 * @warning txbuf > 90 indicates data being dropped
 * 
 * @see SiK radio firmware documentation for parameter tuning
 * @see SR*_* parameters to adjust telemetry stream rates
 */
struct PACKED log_Radio {
    LOG_PACKET_HEADER;          ///< Standard 3-byte header: head1, head2, msgid
    uint64_t time_us;           ///< Timestamp in microseconds since system boot
    uint8_t rssi;               ///< Local received signal strength indicator (0-255, higher=better)
    uint8_t remrssi;            ///< Remote received signal strength indicator (0-255, higher=better)
    uint8_t txbuf;              ///< Transmit buffer usage percentage (0-100%)
    uint8_t noise;              ///< Local noise floor (0-255, lower=better)
    uint8_t remnoise;           ///< Remote noise floor (0-255, lower=better)
    uint16_t rxerrors;          ///< Cumulative count of corrupted packets received
    uint16_t fixed;             ///< Cumulative count of packets corrected by forward error correction
};

/**
 * @struct log_PID
 * @brief PIDx messages: Proportional-Integral-Derivative controller performance data
 * 
 * @details The PID message structure logs detailed internal state of PID controllers used
 *          throughout ArduPilot for attitude, position, and velocity control. Multiple PID
 *          message types (PIDR, PIDP, PIDY, PIDA, PIDS, PIDN, PIDE) share this structure,
 *          each logging different control loops.
 *          
 *          PID Message Types:
 *          - **PIDR**: Roll rate controller (body frame roll angular velocity)
 *          - **PIDP**: Pitch rate controller (body frame pitch angular velocity)
 *          - **PIDY**: Yaw rate controller (body frame yaw angular velocity)
 *          - **PIDA**: Altitude/vertical acceleration controller
 *          - **PIDS**: Steering controller (ground vehicle yaw rate)
 *          - **PIDN**: North velocity controller (position control)
 *          - **PIDE**: East velocity controller (position control)
 *          
 *          PID Controller Purpose:
 *          PID controllers are the fundamental control algorithm in autopilots:
 *          - **Proportional (P)**: Responds to current error magnitude
 *          - **Integral (I)**: Responds to accumulated error over time
 *          - **Derivative (D)**: Responds to rate of error change
 *          - **Feedforward (FF)**: Anticipatory control from desired rate
 *          - Combines to produce control output (motor thrust, servo deflection)
 *          
 *          target (Desired Value):
 *          Setpoint the controller is trying to achieve:
 *          - **Rate Controllers**: Target angular velocity in deg/s or rad/s
 *          - **Velocity Controllers**: Target velocity in m/s
 *          - **Acceleration Controllers**: Target acceleration in m/s²
 *          - Source: Outer control loop or pilot input
 *          
 *          Example Targets:
 *          @code
 *          PIDR target = 45.0 deg/s   // Pilot commanding 45 deg/s roll rate
 *          PIDP target = 0.0 deg/s    // Holding level pitch
 *          PIDA target = -2.0 m/s     // Descending at 2 m/s
 *          PIDN target = 5.0 m/s      // Moving north at 5 m/s
 *          @endcode
 *          
 *          actual (Measured Value):
 *          Current measured value from sensors:
 *          - **Rate Controllers**: Measured angular velocity from gyroscopes
 *          - **Velocity Controllers**: Estimated velocity from EKF
 *          - **Acceleration Controllers**: Measured/estimated acceleration
 *          - Source: Sensor fusion (EKF) or direct sensor measurement
 *          
 *          error (Control Error):
 *          Tracking error driving the controller:
 *          - **Calculation**: error = target - actual
 *          - **Positive Error**: Actual below target (need more control output)
 *          - **Negative Error**: Actual above target (need less control output)
 *          - **Zero Error**: Perfect tracking (ideal but rare)
 *          
 *          P (Proportional Term):
 *          Control contribution proportional to current error:
 *          - **Calculation**: P = Kp * error
 *          - **Kp Gain**: Tuning parameter (e.g., ATC_RAT_RLL_P)
 *          - **Effect**: Immediate response to error
 *          - **Too High**: Oscillation, overshoot
 *          - **Too Low**: Slow response, tracking lag
 *          
 *          I (Integral Term):
 *          Control contribution from accumulated error over time:
 *          - **Calculation**: I += Ki * error * dt
 *          - **Ki Gain**: Tuning parameter (e.g., ATC_RAT_RLL_I)
 *          - **Purpose**: Eliminate steady-state error, handle disturbances
 *          - **Wind-up Protection**: Clamped to prevent excessive buildup
 *          - **Too High**: Overshoot, slow oscillation
 *          - **Too Low**: Residual steady-state error
 *          
 *          I-Term Behavior:
 *          - Builds up when error persists (e.g., wind disturbance)
 *          - Reduced when error changes sign
 *          - Clamped to prevent "integrator wind-up"
 *          - Reset on large errors or mode changes (implementation-dependent)
 *          
 *          D (Derivative Term):
 *          Control contribution from rate of error change:
 *          - **Calculation**: D = Kd * (error_rate)
 *          - **Kd Gain**: Tuning parameter (e.g., ATC_RAT_RLL_D)
 *          - **Purpose**: Damping, reduce overshoot
 *          - **Filtering**: Heavily filtered to reduce noise sensitivity
 *          - **Too High**: Amplifies noise, causes jitter
 *          - **Too Low**: Overshoot, oscillation
 *          
 *          FF (Feedforward Term):
 *          Anticipatory control based on target rate:
 *          - **Calculation**: FF = Kff * target
 *          - **Kff Gain**: Tuning parameter (e.g., ATC_RAT_RLL_FF)
 *          - **Purpose**: Reduce tracking lag, improve response
 *          - **Ideal**: FF alone achieves target (PID only corrects errors)
 *          - **Typical**: FF provides most of control, PID corrects disturbances
 *          
 *          Feedforward Benefit:
 *          Without FF: Controller relies entirely on error to generate output
 *          With FF: Controller anticipates needed output, PID only corrects
 *          Result: Faster response, less lag, better tracking
 *          
 *          DFF (Derivative Feedforward):
 *          Anticipatory control based on target rate change:
 *          - **Calculation**: DFF = Kdff * (target_rate_change)
 *          - **Purpose**: Further reduce lag during rapid setpoint changes
 *          - **Advanced**: Not all controllers implement this
 *          - **Benefit**: Improved response to aggressive pilot input
 *          
 *          Dmod (D-Term Modifier):
 *          Scaling factor applied to D term to reduce limit cycling:
 *          - **Range**: 0.0 to 1.0
 *          - **Purpose**: Reduce D gain when at control limits
 *          - **Effect**: Prevents D term from fighting saturation
 *          - **1.0**: Full D term applied (normal)
 *          - **<1.0**: D term reduced (approaching limits)
 *          
 *          slew_rate (Output Slew Limiter):
 *          Rate limit applied to controller output:
 *          - **Units**: Output units per second
 *          - **Purpose**: Prevent abrupt control changes
 *          - **0**: No slew limiting active
 *          - **>0**: Output rate limited to this value
 *          - **Use Case**: Smooth mode transitions, reduce mechanical stress
 *          
 *          flags (Controller State Flags):
 *          Bitmask indicating controller operational state:
 *          - **Bit 0**: Integrator reset this cycle
 *          - **Bit 1**: Integrator at positive limit (saturated)
 *          - **Bit 2**: Integrator at negative limit (saturated)
 *          - **Bit 3**: Controller output limited
 *          - Additional bits controller-specific
 *          
 *          PID Tuning Analysis:
 *          @code
 *          // Well-tuned controller - good tracking
 *          PIDR: Tar=30, Act=29.5, Err=0.5, P=5, I=2, D=-1, FF=25
 *                (small error, FF doing most work, PID correcting)
 *          
 *          // Under-tuned P gain - slow response
 *          PIDR: Tar=30, Act=22, Err=8, P=8, I=15, D=-2, FF=25
 *                (large error, I-term building up to compensate)
 *          
 *          // Over-tuned D gain - noise amplification
 *          PIDR: Tar=30, Act=30.5, Err=-0.5, P=-5, I=2, D=35, FF=25
 *                (tiny error but huge D-term, sign of noise)
 *          
 *          // Integrator wind-up - stuck at limit
 *          PIDR: Tar=0, Act=5, Err=-5, P=-50, I=100, D=10, FF=0, Flags=0x02
 *                (I-term saturated, can't correct error)
 *          @endcode
 *          
 *          Tuning Guidelines:
 *          1. **Start with P**: Increase until oscillation, then reduce 30%
 *          2. **Add D**: Increase to dampen oscillation, reduce overshoot
 *          3. **Add I**: Increase to eliminate steady-state error
 *          4. **Add FF**: Set to 0.5-1.0 of expected needed output
 *          5. **Iterate**: Fine-tune all terms for best tracking
 *          
 *          Common Tuning Problems:
 *          - **Oscillation**: P or D too high, reduce P first
 *          - **Overshoot**: P too high or D too low, increase D
 *          - **Slow Response**: P too low or D too high, increase P
 *          - **Steady Error**: I too low, increase I
 *          - **Wind-up**: I too high or FF inadequate, reduce I or increase FF
 *          - **Jitter/Noise**: D too high, reduce D or increase filtering
 * 
 * @note Total control output = P + I + D + FF + DFF
 * @note Units vary by controller type (deg/s, m/s, m/s², etc.)
 * @note Log analysis tools can plot these values over time for tuning
 * @note Some controllers may not use all terms (e.g., no DFF)
 * 
 * @warning Large persistent error indicates tuning or mechanical problem
 * @warning I-term saturation (flags bit 1 or 2) indicates insufficient control authority
 * @warning Noisy D-term suggests need for increased filtering or reduced D gain
 * 
 * @see ATC_RAT_*_P/I/D/FF parameters for multicopter rate controller tuning
 * @see PSC_VEL*_P/I/D parameters for position controller tuning
 * @see AC_PID class implementation for algorithm details
 */
struct PACKED log_PID {
    LOG_PACKET_HEADER;          ///< Standard 3-byte header: head1, head2, msgid
    uint64_t time_us;           ///< Timestamp in microseconds since system boot
    float   target;             ///< Desired value (setpoint) for this controller
    float   actual;             ///< Measured value from sensors or estimator
    float   error;              ///< Control error: target - actual
    float   P;                  ///< Proportional term contribution (Kp * error)
    float   I;                  ///< Integral term contribution (accumulated error correction)
    float   D;                  ///< Derivative term contribution (damping based on error rate)
    float   FF;                 ///< Feedforward term contribution (anticipatory control)
    float   DFF;                ///< Derivative feedforward contribution (advanced anticipatory control)
    float   Dmod;               ///< D-term modifier for limit cycle reduction (0.0-1.0)
    float   slew_rate;          ///< Output slew rate limit being applied (0=none)
    uint8_t flags;              ///< Controller state flags (bit 0=reset, 1/2=I saturated, 3=output limited)
};

/**
 * @struct log_WheelEncoder
 * @brief WENC message: Wheel encoder odometry measurements for ground vehicles
 * 
 * @details The WENC message logs wheel encoder measurements used for dead-reckoning navigation
 *          on ground vehicles. Wheel encoders provide wheel rotation data that is converted to
 *          distance traveled, which is then fused with GPS and IMU data in the EKF for improved
 *          position estimation.
 *          
 *          WENC Message Purpose:
 *          - **Ground Vehicle Navigation**: Maintain position estimate when GPS unavailable
 *          - **EKF Integration**: Wheel odometry fused with IMU and GPS in EKF
 *          - **Sensor Health Monitoring**: Track encoder quality and detect failures
 *          - **Calibration Validation**: Verify wheel radius and scaling factor accuracy
 *          - **Slip Detection**: Compare wheel-based and GPS-based distance for slip estimation
 *          
 *          Wheel Encoder Overview:
 *          Wheel encoders measure wheel rotation and convert to distance:
 *          - **Encoder Types**: Quadrature encoders, Hall effect sensors, optical encoders
 *          - **Resolution**: Pulses per revolution (PPR) determines distance resolution
 *          - **Measurement**: Count pulses and multiply by wheel circumference / PPR
 *          - **Differential**: Two encoders (left/right) enable heading estimation
 *          - **Limitations**: Affected by wheel slip, tire pressure, load
 *          
 *          distance_0 (Wheel 0 Distance):
 *          Cumulative distance traveled by first wheel encoder:
 *          - **Units**: Meters
 *          - **Cumulative**: Total distance since boot or reset
 *          - **Positive**: Forward wheel rotation (normal driving)
 *          - **Negative**: Reverse wheel rotation (backing up)
 *          - **Calculation**: (pulse_count / PPR) * wheel_circumference
 *          
 *          Distance Calculation:
 *          @code
 *          // Example encoder configuration
 *          PPR = 1000;                    // 1000 pulses per wheel revolution
 *          wheel_diameter = 0.5;          // 0.5 meter wheel diameter
 *          wheel_circumference = π * 0.5 = 1.571 meters
 *          
 *          // After 5000 pulses
 *          distance = (5000 / 1000) * 1.571 = 7.855 meters
 *          @endcode
 *          
 *          quality_0 (Wheel 0 Quality):
 *          Health/quality metric for first wheel encoder:
 *          - **Range**: 0-100 (percentage)
 *          - **100**: Encoder functioning perfectly
 *          - **50-99**: Encoder working but with some issues
 *          - **0**: Encoder failed or disconnected
 *          
 *          Quality Factors:
 *          - **Signal Strength**: Weak encoder signals reduce quality
 *          - **Missed Pulses**: Dropped counts due to noise or mechanical issues
 *          - **Consistency**: Erratic readings reduce quality
 *          - **Age**: Time since last valid reading
 *          
 *          distance_1 (Wheel 1 Distance):
 *          Cumulative distance traveled by second wheel encoder:
 *          - **Units**: Meters
 *          - **Purpose**: Same as distance_0 but for opposite side wheel
 *          - **Differential**: Difference between wheels indicates turning
 *          - **Redundancy**: Two encoders improve reliability
 *          
 *          Differential Odometry:
 *          Two wheel encoders enable heading estimation:
 *          @code
 *          // Heading change from wheel difference
 *          distance_diff = distance_1 - distance_0;
 *          wheel_base = 1.5;  // meters between wheels
 *          heading_change = distance_diff / wheel_base;  // radians
 *          @endcode
 *          
 *          quality_1 (Wheel 1 Quality):
 *          Health/quality metric for second wheel encoder:
 *          - **Range**: 0-100 (percentage)
 *          - Same interpretation as quality_0
 *          - Independent quality assessment for redundancy
 *          
 *          EKF Integration:
 *          Wheel encoder data fused into Extended Kalman Filter:
 *          - **Velocity Estimate**: Rate of distance change provides velocity
 *          - **Position Update**: Integrated velocity updates position estimate
 *          - **Heading Constraint**: Differential odometry constrains heading
 *          - **Innovation Check**: EKF compares wheel data to GPS/IMU for consistency
 *          - **Adaptive Weighting**: Poor quality reduces EKF trust in encoder data
 *          
 *          Analysis Scenarios:
 *          @code
 *          // Normal operation - both encoders healthy
 *          WENC: Dist0=150.5, Qual0=100, Dist1=150.3, Qual1=100
 *                (straight driving, minor difference due to uneven terrain)
 *          
 *          // Turning right - left wheel travels farther
 *          WENC: Dist0=200.8, Qual0=100, Dist1=195.2, Qual1=100
 *                (5.6m difference indicates right turn)
 *          
 *          // Encoder failure
 *          WENC: Dist0=100.0, Qual0=100, Dist1=100.0, Qual1=0
 *                (right encoder failed, stuck reading)
 *          
 *          // Wheel slip detected
 *          WENC: Dist0=250.0, Qual0=100, Dist1=250.0, Qual1=100
 *          GPS indicates only 200m traveled
 *                (50m discrepancy suggests wheel slip or wrong calibration)
 *          @endcode
 *          
 *          Configuration Parameters:
 *          - **WENC_TYPE_0/1**: Encoder type (quadrature, single pulse, etc.)
 *          - **WENC_CPR_0/1**: Counts per revolution (pulses per wheel rotation)
 *          - **WENC_RADIUS_0/1**: Wheel radius in meters
 *          - **WENC_POS_X/Y/Z_0/1**: Encoder position relative to vehicle center
 *          
 *          Common Issues:
 *          - **Zero Quality**: Encoder disconnected or failed
 *          - **Stuck Distance**: Encoder not counting or broken wire
 *          - **Erratic Distance**: Electrical noise, loose connection, or mechanical issue
 *          - **GPS Mismatch**: Wrong wheel radius, tire pressure, or excessive slip
 *          - **Asymmetric Distance**: Tire pressure difference or drivetrain issue
 *          
 *          Troubleshooting:
 *          - **No Distance Change**: Check wiring, power, and encoder mounting
 *          - **Wrong Distance**: Calibrate WENC_RADIUS parameters
 *          - **Poor Quality**: Check signal integrity, shielding, pull-up resistors
 *          - **Slip**: Compare to GPS over known distance, adjust trust in EKF
 *          
 *          Calibration Procedure:
 *          1. Measure actual wheel diameter accurately
 *          2. Set WENC_RADIUS to measured value
 *          3. Drive straight for known distance with GPS
 *          4. Compare WENC distance to GPS distance
 *          5. Adjust WENC_RADIUS if discrepancy exists
 *          6. Repeat until WENC matches GPS within 2%
 * 
 * @note Wheel encoders most useful in GPS-denied environments (tunnels, under bridges)
 * @note EKF automatically detects and compensates for some wheel slip
 * @note Distance is cumulative - analyze rate of change for velocity
 * @note Two encoders required for full odometry (position and heading)
 * 
 * @warning quality = 0 indicates encoder failure
 * @warning Large GPS/encoder mismatch indicates calibration error or excessive slip
 * @warning Asymmetric encoder readings may indicate mechanical problem
 * 
 * @see WENC_* parameters for encoder configuration
 * @see EKF3_SRC_OPTIONS for enabling/disabling wheel odometry
 * @see AP_WheelEncoder library for encoder driver implementation
 */
struct PACKED log_WheelEncoder {
    LOG_PACKET_HEADER;          ///< Standard 3-byte header: head1, head2, msgid
    uint64_t time_us;           ///< Timestamp in microseconds since system boot
    float distance_0;           ///< Cumulative distance traveled by wheel encoder 0 in meters
    uint8_t quality_0;          ///< Quality/health of encoder 0 (0-100%, 0=failed, 100=perfect)
    float distance_1;           ///< Cumulative distance traveled by wheel encoder 1 in meters
    uint8_t quality_1;          ///< Quality/health of encoder 1 (0-100%, 0=failed, 100=perfect)
};

/**
 * @struct log_ADSB
 * @brief ADSB message: Automatic Dependent Surveillance-Broadcast detected vehicle information
 * 
 * @details The ADSB message logs information about other aircraft detected via ADS-B receivers.
 *          ADS-B is a surveillance technology where aircraft broadcast their position, velocity,
 *          and identification, enabling traffic awareness and collision avoidance without radar.
 *          
 *          ADSB Message Purpose:
 *          - **Collision Avoidance**: Detect nearby aircraft and take evasive action
 *          - **Traffic Awareness**: Maintain situational awareness of surrounding traffic
 *          - **Airspace Integration**: Enable safe operation in controlled airspace
 *          - **Incident Analysis**: Post-flight review of traffic encounters
 *          - **ADSB_VEHICLE_LIST**: Track multiple aircraft simultaneously
 *          
 *          ADS-B Overview:
 *          Automatic Dependent Surveillance-Broadcast is a cooperative surveillance technology:
 *          - **1090 MHz**: Most common frequency for ADS-B (also Mode S transponder)
 *          - **978 MHz**: UAT (Universal Access Transceiver) used in US
 *          - **Automatic**: No pilot or controller input required
 *          - **Dependent**: Relies on aircraft navigation systems (GPS)
 *          - **Surveillance**: Provides position and velocity information
 *          - **Broadcast**: One-way transmission (not request/response)
 *          
 *          Supported Receivers:
 *          - **Ping ADS-B Receivers**: uAvionix Ping1090, Ping2020
 *          - **SageTech**: SageTech MX series transponders
 *          - **MAVLink**: ADS-B data forwarded via MAVLink from GCS or companion computer
 *          - **ADSB_LOCATION messages**: External ADS-B sources
 *          
 *          ICAO_address (Aircraft Address):
 *          Unique 24-bit identifier assigned to each aircraft:
 *          - **Format**: Hexadecimal number (e.g., 0xA1B2C3)
 *          - **Assignment**: ICAO assigns address blocks to countries
 *          - **Permanent**: Aircraft keeps same address for life
 *          - **Tracking**: Use to distinguish and track individual aircraft
 *          - **Database Lookup**: Can resolve to registration number (tail number)
 *          
 *          ICAO Address Structure:
 *          @code
 *          // Example ICAO addresses
 *          0xA00000 - 0xAFFFFF: United States
 *          0x400000 - 0x43FFFF: United Kingdom
 *          0x700000 - 0x700FFF: Australia
 *          0xC00000 - 0xC3FFFF: Canada
 *          
 *          // Lookup example
 *          ICAO_address = 0xA12345
 *          → US registered aircraft
 *          → Database: N12345 (tail number)
 *          @endcode
 *          
 *          lat (Latitude):
 *          Aircraft latitude in degrees * 1E7 (degE7 format):
 *          - **Format**: Signed 32-bit integer
 *          - **Range**: -900000000 to +900000000 (-90° to +90°)
 *          - **Resolution**: ~1.1 cm precision
 *          - **Conversion**: latitude_deg = lat / 10000000.0
 *          
 *          lng (Longitude):
 *          Aircraft longitude in degrees * 1E7 (degE7 format):
 *          - **Format**: Signed 32-bit integer
 *          - **Range**: -1800000000 to +1800000000 (-180° to +180°)
 *          - **Resolution**: ~1.1 cm precision at equator
 *          - **Conversion**: longitude_deg = lng / 10000000.0
 *          
 *          Position Example:
 *          @code
 *          // San Francisco International Airport
 *          lat = 376169722    // 37.6169722° N
 *          lng = -1222370278  // -122.2370278° W
 *          
 *          // Convert to degrees
 *          lat_deg = 376169722 / 10000000.0 = 37.6169722°
 *          lng_deg = -1222370278 / 10000000.0 = -122.2370278°
 *          @endcode
 *          
 *          alt (Altitude):
 *          Aircraft altitude above mean sea level (MSL):
 *          - **Units**: Millimeters
 *          - **Range**: -1000m to 50000m (typical)
 *          - **Source**: GPS or barometric altimeter
 *          - **Conversion**: altitude_m = alt / 1000.0
 *          
 *          heading (Aircraft Heading):
 *          Direction aircraft is pointing (true heading):
 *          - **Units**: Centidegrees (degrees * 100)
 *          - **Range**: 0 to 36000 (0° to 360°)
 *          - **Reference**: True north (not magnetic)
 *          - **Conversion**: heading_deg = heading / 100.0
 *          
 *          Heading vs Track:
 *          - **Heading**: Direction aircraft nose points
 *          - **Track**: Direction aircraft actually moves (ground track)
 *          - **Difference**: Wind causes drift (track ≠ heading)
 *          
 *          hor_velocity (Horizontal Velocity):
 *          Aircraft ground speed (magnitude of velocity over ground):
 *          - **Units**: Centimeters per second (cm/s)
 *          - **Range**: 0 to 65535 cm/s (0 to 655 m/s)
 *          - **Conversion**: speed_m_s = hor_velocity / 100.0
 *          - **Typical**: 12000 cm/s (120 m/s or 233 knots) for airliners
 *          
 *          ver_velocity (Vertical Velocity):
 *          Aircraft climb or descent rate:
 *          - **Units**: Centimeters per second (cm/s)
 *          - **Range**: -32768 to +32767 cm/s (-327 to +327 m/s)
 *          - **Positive**: Climbing
 *          - **Negative**: Descending
 *          - **Zero**: Level flight
 *          - **Conversion**: climb_m_s = ver_velocity / 100.0
 *          
 *          squawk (Transponder Code):
 *          Four-digit octal code set by pilot or ATC:
 *          - **Format**: Octal number (0000 to 7777)
 *          - **Purpose**: Identify aircraft to air traffic control
 *          - **Standard Codes**:
 *            - 1200: VFR (Visual Flight Rules) in US
 *            - 7500: Hijacking
 *            - 7600: Radio failure
 *            - 7700: Emergency
 *          - **Assignment**: ATC assigns discrete codes in controlled airspace
 *          
 *          Squawk Code Examples:
 *          @code
 *          squawk = 1200  // VFR aircraft
 *          squawk = 7700  // Aircraft declaring emergency
 *          squawk = 1234  // Discrete code assigned by ATC
 *          @endcode
 *          
 *          Collision Avoidance:
 *          ADSB data enables autonomous collision avoidance:
 *          - **Threat Assessment**: Calculate closest point of approach (CPA)
 *          - **Evasion**: Automatically change course or altitude
 *          - **Alerting**: Warn pilot of traffic conflicts
 *          - **Geofencing**: Stay clear of detected traffic
 *          
 *          ADSB_* Parameters:
 *          - **ADSB_ENABLE**: Enable ADS-B receiver
 *          - **ADSB_BEHAVIORS**: Collision avoidance behavior (warn, evade, etc.)
 *          - **ADSB_LIST_MAX**: Maximum number of tracked aircraft
 *          - **ADSB_LIST_RADIUS**: Radius to track aircraft (meters)
 *          - **ADSB_ICAO_ID**: This vehicle's ICAO address (for ADS-B out)
 *          
 *          Analysis Scenarios:
 *          @code
 *          // Nearby aircraft detected
 *          ADSB: ICAO=0xA12345, Lat=376169722, Lng=-1222370278, Alt=3000000, 
 *                Hdg=9000, HVel=12000, VVel=0, Sq=1234
 *                (Aircraft at 3000m MSL, heading 90° at 120 m/s level flight)
 *          
 *          // Emergency aircraft
 *          ADSB: ICAO=0xA99999, Lat=375000000, Lng=-1220000000, Alt=1500000,
 *                Hdg=18000, HVel=8000, VVel=-500, Sq=7700
 *                (Emergency squawk, descending at 5 m/s)
 *          
 *          // Multiple aircraft logged as sequence
 *          ADSB: ICAO=0xA11111, Lat=..., Lng=..., Alt=2000000, ...
 *          ADSB: ICAO=0xA22222, Lat=..., Lng=..., Alt=4000000, ...
 *          ADSB: ICAO=0xA33333, Lat=..., Lng=..., Alt=6000000, ...
 *          @endcode
 *          
 *          Common Issues:
 *          - **No ADSB Messages**: Receiver not connected or no traffic nearby
 *          - **Intermittent Detection**: Aircraft out of range or poor antenna
 *          - **Stale Position**: Aircraft stopped transmitting or receiver issue
 *          - **Invalid Altitude**: Aircraft not transmitting GPS altitude
 *          
 *          Troubleshooting:
 *          - **Check Receiver**: Verify ADSB_ENABLE = 1 and receiver connected
 *          - **Antenna**: Ensure proper antenna installation and orientation
 *          - **Range**: 1090 MHz has ~100+ nm range line-of-sight
 *          - **Ground Testing**: May not detect aircraft if on ground (blocked)
 * 
 * @note One ADSB message logged per detected aircraft per update
 * @note Not all aircraft equipped with ADS-B (older aircraft, military)
 * @note Position accuracy depends on aircraft GPS quality
 * @note Ground vehicles and some drones also use ADS-B
 * 
 * @warning squawk=7700 indicates aircraft emergency
 * @warning Close CPA with high closure rate requires immediate action
 * @warning Missing ADSB data does not guarantee no traffic nearby
 * 
 * @see ADSB_* parameters for receiver configuration
 * @see ADSB_VEHICLE_THREAT message for threat assessment
 * @see AP_ADSB library for ADS-B processing
 */
struct PACKED log_ADSB {
    LOG_PACKET_HEADER;          ///< Standard 3-byte header: head1, head2, msgid
    uint64_t time_us;           ///< Timestamp in microseconds since system boot
    uint32_t ICAO_address;      ///< 24-bit ICAO aircraft address (unique identifier)
    int32_t lat;                ///< Latitude in degrees * 1E7 (degE7 format)
    int32_t lng;                ///< Longitude in degrees * 1E7 (degE7 format)
    int32_t alt;                ///< Altitude above MSL in millimeters
    uint16_t heading;           ///< True heading in centidegrees (0-36000 for 0-360°)
    uint16_t hor_velocity;      ///< Horizontal ground speed in cm/s
    int16_t ver_velocity;       ///< Vertical speed in cm/s (positive=climb, negative=descend)
    uint16_t squawk;            ///< Transponder squawk code (octal, e.g., 1200, 7700)
};

/**
 * @struct log_MAG
 * @brief MAG message: Magnetometer (compass) sensor data
 * 
 * @details The MAG message logs raw magnetometer measurements and calibration offsets for compass
 *          sensors. Magnetometers measure Earth's magnetic field to determine heading, which is
 *          critical for navigation when GPS heading is unavailable or unreliable (low speed).
 *          
 *          MAG Message Purpose:
 *          - **Heading Estimation**: Provide magnetic heading to AHRS/EKF
 *          - **Calibration Validation**: Verify compass calibration quality
 *          - **Interference Detection**: Identify magnetic interference from motors/ESCs
 *          - **Sensor Health Monitoring**: Track compass reliability and consistency
 *          - **Multi-Compass Fusion**: Compare multiple magnetometers for redundancy
 *          
 *          Magnetometer Overview:
 *          Magnetometers measure magnetic field strength in three axes:
 *          - **Technology**: Hall effect, magnetoresistive (AMR, TMR, GMR), fluxgate
 *          - **Common Sensors**: HMC5843, HMC5883L, LSM303D, QMC5883L, IST8310, AK8963
 *          - **External vs Internal**: External compasses farther from interference
 *          - **I2C/SPI**: Connected via I2C or SPI bus
 *          - **Update Rate**: Typically 10-100 Hz
 *          
 *          Magnetic Field and Heading:
 *          @code
 *          // Earth's magnetic field vector in body frame
 *          mag_field = [mag_x, mag_y, mag_z]
 *          
 *          // After removing hard iron offsets and motor interference
 *          corrected = mag_field - offsets - motor_offsets
 *          
 *          // Rotate to horizontal plane using roll/pitch from IMU
 *          mag_horizontal = rotate(corrected, roll, pitch)
 *          
 *          // Calculate heading
 *          heading = atan2(-mag_horizontal.y, mag_horizontal.x)
 *          @endcode
 *          
 *          instance (Compass Instance):
 *          Magnetometer sensor instance number:
 *          - **Range**: 0-2 (typically 0-1)
 *          - **0**: Primary compass
 *          - **1**: Secondary compass (if equipped)
 *          - **2**: Tertiary compass (rare)
 *          - **Multiple Compasses**: Improve reliability through voting/fusion
 *          
 *          mag_x, mag_y, mag_z (Raw Magnetic Field):
 *          Magnetic field measurements in body frame after offset correction:
 *          - **Units**: Milligauss (mGauss)
 *          - **Body Frame**: +X forward, +Y right, +Z down
 *          - **Range**: Typically -1000 to +1000 mGauss
 *          - **Earth's Field**: ~500 mGauss total magnitude (varies by location)
 *          - **Sign**: Depends on vehicle orientation and magnetic declination
 *          
 *          Magnetic Field Components:
 *          @code
 *          // Vehicle pointing north, level
 *          mag_x ≈ +500 mGauss  (pointing toward magnetic north)
 *          mag_y ≈ 0 mGauss
 *          mag_z ≈ -400 mGauss  (downward component, varies by latitude)
 *          
 *          // Vehicle pointing east, level
 *          mag_x ≈ 0 mGauss
 *          mag_y ≈ +500 mGauss  (pointing toward magnetic east)
 *          mag_z ≈ -400 mGauss
 *          
 *          // Vehicle pointing north, pitched up 45°
 *          mag_x ≈ +350 mGauss
 *          mag_y ≈ 0 mGauss
 *          mag_z ≈ -640 mGauss  (z-component increases with pitch)
 *          @endcode
 *          
 *          offset_x, offset_y, offset_z (Hard Iron Offsets):
 *          Calibration offsets to compensate for hard iron magnetic interference:
 *          - **Units**: Milligauss (mGauss)
 *          - **Hard Iron**: Permanent magnetic fields from ferrous materials
 *          - **Examples**: Steel screws, motors, magnets, batteries
 *          - **Constant**: Does not change with orientation
 *          - **Calibration**: Determined by rotating vehicle through all orientations
 *          
 *          Hard Iron Offset Calculation:
 *          During calibration, offsets are computed to center the magnetic field sphere:
 *          @code
 *          // Without offsets, magnetic field forms offset sphere
 *          // Example uncalibrated readings spinning 360°:
 *          mag_x: -200 to +800 mGauss  (should be -500 to +500)
 *          mag_y: -150 to +850 mGauss  (should be -500 to +500)
 *          mag_z: -600 to +400 mGauss  (should be -500 to +500)
 *          
 *          // Offsets to center sphere:
 *          offset_x = (max_x + min_x) / 2 = (+800 + -200) / 2 = +300
 *          offset_y = (max_y + min_y) / 2 = (+850 + -150) / 2 = +350
 *          offset_z = (max_z + min_z) / 2 = (+400 + -600) / 2 = -100
 *          
 *          // Corrected readings:
 *          mag_x_corrected = mag_x_raw - offset_x
 *          @endcode
 *          
 *          motor_offset_x, motor_offset_y, motor_offset_z (Motor Interference):
 *          Dynamic magnetic field offsets caused by motor/ESC currents:
 *          - **Units**: Milligauss (mGauss)
 *          - **Dynamic**: Changes with throttle/current
 *          - **Sources**: Motor magnetic fields, ESC switching noise, battery currents
 *          - **Compensation**: Scaled by throttle or battery current
 *          - **Calibration**: Computed during motor calibration procedure
 *          
 *          Motor Interference Compensation:
 *          @code
 *          // Motor offsets proportional to throttle or current
 *          throttle_factor = current_throttle / throttle_used_during_cal;
 *          
 *          dynamic_offset_x = motor_offset_x * throttle_factor;
 *          dynamic_offset_y = motor_offset_y * throttle_factor;
 *          dynamic_offset_z = motor_offset_z * throttle_factor;
 *          
 *          // Total correction
 *          mag_x_final = mag_x_raw - offset_x - dynamic_offset_x;
 *          @endcode
 *          
 *          Typical Motor Offsets:
 *          - **Magnitude**: 50-300 mGauss at full throttle
 *          - **Direction**: Usually in Z-axis (vertical) for multicopters
 *          - **Problem Sign**: Offsets > 500 mGauss indicate poor compass placement
 *          
 *          health (Compass Health):
 *          Boolean health status:
 *          - **1**: Compass healthy and reliable
 *          - **0**: Compass unhealthy, failed, or inconsistent
 *          
 *          Health Criteria:
 *          - **Magnitude Check**: Total field magnitude reasonable (~300-700 mGauss)
 *          - **Consistency**: Agrees with other compasses
 *          - **Innovation**: Matches expected heading from GPS velocity
 *          - **Communication**: I2C/SPI communication successful
 *          
 *          SUS (Sample Time):
 *          Time in microseconds when this measurement was taken:
 *          - **Units**: Microseconds since system boot
 *          - **Purpose**: Precise timestamping for sensor fusion
 *          - **Synchronization**: Align magnetometer data with IMU in EKF
 *          - **Delay Compensation**: Account for sensor read delays
 *          
 *          Compass Calibration:
 *          Two calibration procedures determine offsets:
 *          
 *          1. **Hard Iron Calibration** (compass calibration dance):
 *             - Rotate vehicle through all orientations
 *             - Collect min/max readings in each axis
 *             - Compute offsets to center sphere
 *             - Store in offset_x/y/z parameters
 *          
 *          2. **Motor Calibration** (motor interference):
 *             - Arm vehicle and spin motors at various throttles
 *             - Measure magnetic field change vs throttle
 *             - Compute motor_offset_x/y/z coefficients
 *             - Applied dynamically based on throttle
 *          
 *          Analysis Scenarios:
 *          @code
 *          // Well-calibrated external compass
 *          MAG: I=0, MagX=520, MagY=-50, MagZ=-420, OfsX=150, OfsY=-80, OfsZ=200,
 *               MOX=80, MOY=20, MOZ=150, Health=1, SUS=12345678
 *               (Good field magnitude, small motor offsets, healthy)
 *          
 *          // Internal compass with high motor interference
 *          MAG: I=0, MagX=480, MagY=100, MagZ=-350, OfsX=250, OfsY=180, OfsZ=300,
 *               MOX=420, MOY=350, MOZ=650, Health=1, SUS=12345789
 *               (High motor offsets indicate poor placement)
 *          
 *          // Failed compass
 *          MAG: I=0, MagX=0, MagY=0, MagZ=0, OfsX=150, OfsY=-80, OfsZ=200,
 *               MOX=80, MOY=20, MOZ=150, Health=0, SUS=12345890
 *               (Zero readings, health=0 indicates failure)
 *          
 *          // Magnetic anomaly or interference
 *          MAG: I=0, MagX=1500, MagY=800, MagZ=-1200, OfsX=150, OfsY=-80, OfsZ=200,
 *               MOX=80, MOY=20, MOZ=150, Health=0, SUS=12345901
 *               (Excessive field magnitude, health=0, likely near metal)
 *          @endcode
 *          
 *          COMPASS_* Parameters:
 *          - **COMPASS_ENABLE**: Enable compass sensors
 *          - **COMPASS_OFS_X/Y/Z**: Hard iron offset calibration values
 *          - **COMPASS_MOT_X/Y/Z**: Motor interference compensation
 *          - **COMPASS_EXTERNAL**: Designate as external compass
 *          - **COMPASS_USE**: Enable this compass instance
 *          - **COMPASS_PRIMARY**: Select primary compass
 *          
 *          Common Issues:
 *          - **Poor Calibration**: Offsets wrong, erratic heading
 *          - **Motor Interference**: Heading wanders with throttle
 *          - **Magnetic Anomaly**: Near metal structures, power lines
 *          - **Failed Sensor**: health=0, zero readings
 *          - **Wrong Orientation**: Compass mounted incorrectly
 *          
 *          Troubleshooting:
 *          - **Recalibrate**: Perform compass calibration if heading inaccurate
 *          - **Motor Cal**: Run motor compensation if heading changes with throttle
 *          - **External Compass**: Use external compass on GPS mast
 *          - **Check Mounting**: Verify compass orientation parameters
 *          - **Avoid Metal**: Keep compass away from ferrous materials
 * 
 * @note Multiple MAG messages logged if multiple compasses configured
 * @note Compass critical for heading at low speeds when GPS heading unavailable
 * @note External compasses strongly recommended for multirotors
 * @note Compass not used when EKF has sufficient GPS velocity for heading
 * 
 * @warning health=0 indicates compass failure or excessive interference
 * @warning Motor offsets > 500 mGauss indicate compass too close to motors
 * @warning Total field magnitude outside 300-700 mGauss indicates calibration issue
 * 
 * @see COMPASS_* parameters for compass configuration
 * @see AP_Compass library for magnetometer drivers
 * @see Compass calibration documentation in ArduPilot wiki
 */
struct PACKED log_MAG {
    LOG_PACKET_HEADER;          ///< Standard 3-byte header: head1, head2, msgid
    uint64_t time_us;           ///< Timestamp in microseconds since system boot
    uint8_t  instance;          ///< Compass instance number (0=primary, 1=secondary, 2=tertiary)
    int16_t  mag_x;             ///< Magnetic field X-axis (body frame forward) in milligauss
    int16_t  mag_y;             ///< Magnetic field Y-axis (body frame right) in milligauss
    int16_t  mag_z;             ///< Magnetic field Z-axis (body frame down) in milligauss
    int16_t  offset_x;          ///< Hard iron offset X-axis in milligauss
    int16_t  offset_y;          ///< Hard iron offset Y-axis in milligauss
    int16_t  offset_z;          ///< Hard iron offset Z-axis in milligauss
    int16_t  motor_offset_x;    ///< Motor interference offset X-axis in milligauss
    int16_t  motor_offset_y;    ///< Motor interference offset Y-axis in milligauss
    int16_t  motor_offset_z;    ///< Motor interference offset Z-axis in milligauss
    uint8_t  health;            ///< Compass health status (1=healthy, 0=unhealthy/failed)
    uint32_t SUS;               ///< Sample timestamp in microseconds (when measurement taken)
};

/**
 * @struct log_Mode
 * @brief MODE message: Vehicle control mode information and transitions
 * 
 * @details The MODE message logs vehicle flight mode changes, recording when modes are entered
 *          and why the transition occurred. This is critical for understanding vehicle behavior,
 *          diagnosing mode transition issues, and analyzing autonomous mission execution.
 *          
 *          MODE Message Purpose:
 *          - **Mode History**: Track sequence of mode changes during flight
 *          - **Transition Analysis**: Understand why autonomous modes exited
 *          - **Failsafe Diagnosis**: Identify which failsafe triggered mode change
 *          - **Mission Debugging**: Verify correct mode transitions during waypoint missions
 *          - **Incident Analysis**: Reconstruct mode sequence leading to incidents
 *          
 *          Flight Mode Overview:
 *          ArduPilot supports multiple flight modes per vehicle type:
 *          
 *          **Copter Modes** (examples):
 *          - 0: STABILIZE - Manual control with self-leveling
 *          - 1: ACRO - Manual control, no self-leveling
 *          - 2: ALT_HOLD - Hold altitude automatically
 *          - 3: AUTO - Execute waypoint mission
 *          - 4: GUIDED - External control (e.g., companion computer)
 *          - 5: LOITER - Hold position and altitude
 *          - 6: RTL - Return to launch
 *          - 9: LAND - Autonomous landing
 *          - 17: BRAKE - Rapid stop
 *          
 *          **Plane Modes** (examples):
 *          - 0: MANUAL - Direct servo control
 *          - 2: STABILIZE - Self-leveling
 *          - 3: TRAINING - Stabilized with limits
 *          - 10: AUTO - Waypoint mission
 *          - 11: RTL - Return to launch
 *          - 15: GUIDED - External control
 *          
 *          **Rover Modes** (examples):
 *          - 0: MANUAL - Direct throttle/steering control
 *          - 3: STEERING - Heading hold with manual throttle
 *          - 4: HOLD - Stop and hold position
 *          - 10: AUTO - Waypoint mission
 *          - 11: RTL - Return to launch
 *          - 15: GUIDED - External control
 *          
 *          mode (Mode Number):
 *          Current flight mode enum value:
 *          - **Type**: Vehicle-specific mode enumeration
 *          - **Range**: 0-255
 *          - **Varies**: Each vehicle type has different mode numbers
 *          - **Lookup**: Refer to vehicle-specific mode definitions
 *          
 *          mode_num (Mode Number Alias):
 *          Duplicate of mode field for compatibility:
 *          - **Purpose**: Historical field kept for log compatibility
 *          - **Value**: Always same as mode field
 *          - **Usage**: Use mode field; mode_num is redundant
 *          
 *          mode_reason (Mode Change Reason):
 *          Enumerated reason why mode changed:
 *          - **Type**: ModeReason enum value
 *          - **Purpose**: Explain what triggered the mode change
 *          - **Critical**: Essential for diagnosing unexpected transitions
 *          
 *          ModeReason Enum Values:
 *          @code
 *          enum class ModeReason : uint8_t {
 *              UNKNOWN                 = 0,  // Mode change reason unknown
 *              RC_COMMAND              = 1,  // Pilot changed mode via RC switch
 *              GCS_COMMAND             = 2,  // Ground station commanded mode change
 *              RADIO_FAILSAFE          = 3,  // RC signal lost (radio failsafe)
 *              BATTERY_FAILSAFE        = 4,  // Battery voltage/capacity low
 *              GCS_FAILSAFE            = 5,  // GCS telemetry lost
 *              EKF_FAILSAFE            = 6,  // Navigation estimate failed
 *              GPS_GLITCH              = 7,  // GPS position jump detected
 *              MISSION_END             = 8,  // Waypoint mission completed
 *              THROTTLE_LAND_ESCAPE    = 9,  // Throttle raised during landing
 *              FENCE_BREACHED          = 10, // Geofence boundary crossed
 *              TERRAIN_FAILSAFE        = 11, // Terrain data unavailable
 *              BRAKE_TIMEOUT           = 12, // Brake mode timeout
 *              FLIP_COMPLETE           = 13, // Flip maneuver completed
 *              AVOIDANCE               = 14, // Object avoidance triggered
 *              AVOIDANCE_RECOVERY      = 15, // Avoidance recovery complete
 *              THROW_COMPLETE          = 16, // Throw mode launch detected
 *              AUTO_LAND_FINISHED      = 17, // Auto landing sequence complete
 *              CRASH_FAILSAFE          = 18, // Crash detected
 *              SOARING_FBW_B_WITH_MOTOR_RUNNING = 19, // Soaring: motor restart
 *              SOARING_THERMAL_DETECTED = 20, // Soaring: thermal found
 *              SOARING_THERMAL_ESTIMATE_DETERIORATED = 21, // Soaring: thermal lost
 *              VTOL_FAILED_TRANSITION  = 22, // QuadPlane transition failed
 *              VTOL_FAILED_TAKEOFF     = 23, // QuadPlane takeoff failed
 *              FAILSAFE                = 24, // Generic failsafe (multiple triggers)
 *              INITIALISED             = 25, // Mode set during initialization
 *              SURFACE_COMPLETE        = 26, // Sub surfacing complete
 *              BAD_DEPTH               = 27, // Sub depth estimate invalid
 *              LEAK_FAILSAFE           = 28, // Sub leak detected
 *              SERVOTEST               = 29, // Servo test mode activated
 *              STARTUP                 = 30, // Mode set at startup
 *              // ... more reasons added over time
 *          };
 *          @endcode
 *          
 *          Mode Change Scenarios:
 *          @code
 *          // Pilot switches to AUTO mode via RC
 *          MODE: Mode=3, ModeNum=3, Rsn=1
 *                (AUTO mode, RC_COMMAND - pilot intentional)
 *          
 *          // Radio failsafe triggers RTL
 *          MODE: Mode=6, ModeNum=6, Rsn=3
 *                (RTL mode, RADIO_FAILSAFE - RC signal lost)
 *          
 *          // Battery failsafe forces LAND
 *          MODE: Mode=9, ModeNum=9, Rsn=4
 *                (LAND mode, BATTERY_FAILSAFE - low battery)
 *          
 *          // Mission completes, switches to RTL
 *          MODE: Mode=6, ModeNum=6, Rsn=8
 *                (RTL mode, MISSION_END - waypoints complete)
 *          
 *          // Geofence breach triggers RTL
 *          MODE: Mode=6, ModeNum=6, Rsn=10
 *                (RTL mode, FENCE_BREACHED - flew outside boundary)
 *          
 *          // GCS commands GUIDED mode
 *          MODE: Mode=4, ModeNum=4, Rsn=2
 *                (GUIDED mode, GCS_COMMAND - ground station control)
 *          @endcode
 *          
 *          Typical Mode Sequences:
 *          @code
 *          // Normal autonomous mission flight:
 *          MODE: Mode=0, Rsn=30  // STABILIZE at startup
 *          MODE: Mode=3, Rsn=1   // AUTO via RC switch
 *          MODE: Mode=6, Rsn=8   // RTL when mission ends
 *          MODE: Mode=9, Rsn=17  // LAND at home
 *          
 *          // Failsafe sequence:
 *          MODE: Mode=5, Rsn=1   // LOITER via RC
 *          MODE: Mode=6, Rsn=3   // RTL - radio failsafe
 *          MODE: Mode=9, Rsn=17  // LAND when RTL completes
 *          
 *          // Multiple failsafes:
 *          MODE: Mode=3, Rsn=1   // AUTO mission
 *          MODE: Mode=6, Rsn=4   // RTL - battery failsafe
 *          MODE: Mode=9, Rsn=18  // LAND - crash detected during RTL
 *          @endcode
 *          
 *          Mode Change Analysis:
 *          **Reason=0 (UNKNOWN)**: Should not occur; indicates potential bug
 *          **Reason=1 (RC_COMMAND)**: Normal pilot-initiated change
 *          **Reason=2 (GCS_COMMAND)**: Normal GCS-initiated change
 *          **Reason=3 (RADIO_FAILSAFE)**: RC signal lost - check receiver/transmitter
 *          **Reason=4 (BATTERY_FAILSAFE)**: Low battery - check voltage/capacity
 *          **Reason=5 (GCS_FAILSAFE)**: Telemetry lost - check radio link
 *          **Reason=6 (EKF_FAILSAFE)**: Navigation failed - check GPS/compass
 *          **Reason=8 (MISSION_END)**: Expected at end of AUTO missions
 *          **Reason=10 (FENCE_BREACHED)**: Flew outside geofence - check fence settings
 *          **Reason=18 (CRASH_FAILSAFE)**: Crash detected - investigate landing/impact
 *          
 *          Flight Mode Parameters:
 *          - **FLTMODE1-6**: RC channel PWM to flight mode mapping
 *          - **FS_THR_ENABLE**: Throttle failsafe behavior
 *          - **FS_GCS_ENABLE**: GCS failsafe behavior
 *          - **BATT_FS_LOW_ACT**: Battery failsafe action
 *          - **FENCE_ACTION**: Geofence breach action
 *          
 *          Common Issues:
 *          - **Unexpected Mode Change**: Check reason - likely failsafe
 *          - **Cannot Enter Mode**: Pre-arm checks or mode requirements not met
 *          - **Frequent Mode Changes**: RC switch glitching or failsafe oscillation
 *          - **Mode Rejected**: Vehicle conditions prevent mode (e.g., no GPS for LOITER)
 *          
 *          Troubleshooting:
 *          - **Check Reason**: mode_reason explains why change occurred
 *          - **Verify Settings**: Review failsafe parameters if unexpected
 *          - **Pre-Arm Checks**: Some modes require GPS, compass, or specific sensors
 *          - **RC Mapping**: Verify FLTMODE parameters match RC switch positions
 * 
 * @note MODE message logged every time flight mode changes
 * @note mode and mode_num always identical (redundant field)
 * @note Different vehicle types have different mode number meanings
 * @note Check vehicle-specific documentation for mode number definitions
 * 
 * @warning Unexpected mode changes (Rsn ≠ 1 or 2) indicate potential problems
 * @warning RADIO_FAILSAFE (Rsn=3) requires immediate attention
 * @warning Multiple failsafes in sequence indicate serious system issues
 * 
 * @see ModeReason enum for complete list of mode change reasons
 * @see Vehicle-specific mode definitions (Copter::Mode, Plane::Mode, Rover::Mode)
 * @see FLTMODE* parameters for RC switch to mode mapping
 */
struct PACKED log_Mode {
    LOG_PACKET_HEADER;          ///< Standard 3-byte header: head1, head2, msgid
    uint64_t time_us;           ///< Timestamp in microseconds since system boot
    uint8_t mode;               ///< Flight mode number (vehicle-specific enum value)
    uint8_t mode_num;           ///< Mode number alias (duplicate of mode field for compatibility)
    uint8_t mode_reason;        ///< Reason for mode change (ModeReason enum value)
};

/**
 * @struct log_RFND
 * @brief RFND message: Rangefinder (distance sensor) measurements
 * 
 * @details The RFND message logs distance measurements from rangefinder sensors such as lidars,
 *          sonars, and radar altimeters. Rangefinders are used for terrain following, precision
 *          landing, obstacle detection, altitude hold near ground, and object avoidance.
 *          
 *          RFND Message Purpose:
 *          - **Terrain Following**: Maintain constant height above ground
 *          - **Precision Landing**: Accurate altitude during final landing approach
 *          - **Altitude Hold**: Improved altitude control near ground
 *          - **Object Detection**: Detect obstacles for collision avoidance
 *          - **Surface Tracking**: Follow ground/water surface contours
 *          
 *          Rangefinder Overview:
 *          Multiple rangefinder technologies supported:
 *          - **Lidar**: Laser-based, accurate, long range (LightWare, Benewake, Garmin)
 *          - **Sonar/Ultrasonic**: Sound-based, short range, weather-sensitive (MaxBotix, HC-SR04)
 *          - **Radar**: Microwave-based, works in fog/rain (Ainstein, NRA24)
 *          - **IR**: Infrared-based, very short range (Sharp GP2Y)
 *          - **PWM**: Analog output sensors
 *          
 *          Supported Sensors:
 *          - **LightWare**: SF10, SF11, SF20, SF30, SF40, SF45, LW20
 *          - **Benewake**: TFmini, TFmini Plus, TF02, TF03, TFLuna
 *          - **Garmin**: Lidar-Lite v3, v4
 *          - **MaxBotix**: MB1xxx series sonar
 *          - **TeraRanger**: Evo, One
 *          - **Leddar**: Leddar One
 *          - **uLanding**: Aerotenna uLanding Radar
 *          - **NMEA**: GPS-style serial rangefinders
 *          
 *          instance (Rangefinder Instance):
 *          Rangefinder sensor instance number:
 *          - **Range**: 0-9 (up to 10 rangefinders supported)
 *          - **0**: First rangefinder (typically downward-facing)
 *          - **1-9**: Additional rangefinders for different orientations
 *          - **Multiple**: Enable obstacle detection from all directions
 *          
 *          dist (Distance Measurement):
 *          Measured distance to target surface or object:
 *          - **Units**: Meters
 *          - **Range**: 0.0 to sensor maximum (typically 0.2m to 100m)
 *          - **Resolution**: Typically 1cm for lidar, 1-2cm for sonar
 *          - **Accuracy**: ±2-5cm for lidar, ±1% for sonar
 *          - **Update Rate**: 10-200 Hz depending on sensor
 *          
 *          Distance Interpretation:
 *          @code
 *          // Near ground during landing
 *          dist = 0.50  // 50cm above ground
 *          
 *          // Terrain following over varying terrain
 *          dist = 5.20  // 5.2m above ground
 *          
 *          // Out of range or no target
 *          dist = sensor_max_range  // At maximum range limit
 *          @endcode
 *          
 *          status (Sensor Status):
 *          Rangefinder operational status enum:
 *          - **RangeFinder::Status enum values**:
 *            - 0: NotConnected - Sensor not detected or disconnected
 *            - 1: NoData - Sensor connected but no valid reading
 *            - 2: OutOfRangeLow - Target too close (below min range)
 *            - 3: OutOfRangeHigh - Target too far (beyond max range)
 *            - 4: Good - Valid measurement within range
 *          
 *          Status Meanings:
 *          @code
 *          status = 0 (NotConnected)
 *              - Sensor not found on I2C/serial bus
 *              - Wrong RNGFND_TYPE parameter
 *              - Hardware failure or disconnected
 *          
 *          status = 1 (NoData)
 *              - Sensor responding but no valid reading
 *              - Target surface absorbing signal (dark, soft)
 *              - Interference or noise
 *          
 *          status = 2 (OutOfRangeLow)
 *              - dist < minimum range (e.g., < 20cm)
 *              - Sensor blind zone
 *              - Not useful for navigation
 *          
 *          status = 3 (OutOfRangeHigh)
 *              - dist > maximum range (e.g., > 50m)
 *              - No target detected
 *              - Surface too far or absorbing
 *          
 *          status = 4 (Good)
 *              - Valid measurement
 *              - Distance within min/max range
 *              - Signal quality acceptable
 *          @endcode
 *          
 *          orient (Sensor Orientation):
 *          Rangefinder mounting orientation (Rotation enum):
 *          - **0**: ROTATION_NONE / Down (most common)
 *          - **24**: ROTATION_PITCH_270 / Up
 *          - **0-5**: Front, right, back, left combinations
 *          - **See Rotation enum**: Complete orientation definitions
 *          
 *          Common Orientations:
 *          @code
 *          orient = 25 (Down)        // Altitude/terrain following
 *          orient = 24 (Up)          // Ceiling detection
 *          orient = 0  (Forward)     // Obstacle detection
 *          orient = 2  (Back)        // Rear obstacle detection
 *          orient = 1  (Right)       // Side obstacle detection
 *          orient = 3  (Left)        // Side obstacle detection
 *          @endcode
 *          
 *          quality (Signal Quality):
 *          Signal strength or confidence indicator:
 *          - **Range**: -1 to 100
 *          - **-1**: Quality invalid or not supported by sensor
 *          - **0**: No signal or very poor signal
 *          - **100**: Perfect signal, high confidence
 *          - **Interpretation**: Sensor-specific
 *          
 *          Quality Factors:
 *          - **Signal Strength**: Strong return = high quality
 *          - **Target Reflectivity**: Bright surfaces = higher quality
 *          - **Ambient Light**: Bright sunlight can reduce lidar quality
 *          - **Angle of Incidence**: Perpendicular = best quality
 *          - **Surface Type**: Smooth, hard surfaces = better quality
 *          
 *          Rangefinder Usage in Flight Control:
 *          @code
 *          // Terrain following (WPNAV_RFND_USE=1)
 *          if (status == Good && dist < 20.0) {
 *              // Use rangefinder for altitude control
 *              altitude_source = RANGEFINDER;
 *              target_altitude_above_terrain = 5.0;  // maintain 5m AGL
 *          } else {
 *              // Fall back to barometric altitude
 *              altitude_source = BAROMETER;
 *          }
 *          
 *          // Precision landing
 *          if (status == Good && dist < 5.0) {
 *              // Use rangefinder in final landing phase
 *              descent_rate = calculate_land_rate(dist);
 *          }
 *          
 *          // Object avoidance
 *          if (status == Good && dist < 3.0 && orient == Forward) {
 *              // Obstacle detected ahead - slow down or stop
 *              max_velocity_forward = 0.5;  // reduce speed
 *          }
 *          @endcode
 *          
 *          Analysis Scenarios:
 *          @code
 *          // Normal terrain following
 *          RFND: Inst=0, Dist=10.50, Stat=4, Orient=25, Qual=95
 *                (10.5m above ground, good reading, downward-facing, excellent quality)
 *          
 *          // Approaching ground during landing
 *          RFND: Inst=0, Dist=2.35, Stat=4, Orient=25, Qual=98
 *          RFND: Inst=0, Dist=1.82, Stat=4, Orient=25, Qual=99
 *          RFND: Inst=0, Dist=0.45, Stat=4, Orient=25, Qual=100
 *                (Decreasing distance, consistently good readings during landing)
 *          
 *          // Out of range over water
 *          RFND: Inst=0, Dist=50.00, Stat=3, Orient=25, Qual=0
 *                (Water absorbs signal, out of range high, no quality)
 *          
 *          // Too close - blind zone
 *          RFND: Inst=0, Dist=0.15, Stat=2, Orient=25, Qual=-1
 *                (15cm - below minimum range, out of range low)
 *          
 *          // Forward obstacle detection
 *          RFND: Inst=1, Dist=2.50, Stat=4, Orient=0, Qual=85
 *                (Obstacle 2.5m ahead, good reading, forward-facing)
 *          
 *          // Sensor failure
 *          RFND: Inst=0, Dist=0.00, Stat=0, Orient=25, Qual=-1
 *                (Not connected, sensor offline or disconnected)
 *          @endcode
 *          
 *          RNGFND_* Parameters:
 *          - **RNGFND1_TYPE**: Sensor type (Lidar, Sonar, etc.)
 *          - **RNGFND1_ORIENT**: Mounting orientation
 *          - **RNGFND1_MIN_CM**: Minimum valid distance (cm)
 *          - **RNGFND1_MAX_CM**: Maximum valid distance (cm)
 *          - **RNGFND1_GNDCLEAR**: Ground clearance in cm (sensor below vehicle center)
 *          - **RNGFND1_ADDR**: I2C address or serial port
 *          - **RNGFND_LANDING**: Use rangefinder during landing
 *          
 *          Common Issues:
 *          - **status=0**: Sensor not detected - check wiring, power, address
 *          - **status=1**: No data - check target surface, interference
 *          - **status=3**: Out of range - surface too far or absorbing (water, foam)
 *          - **Low quality**: Poor surface reflectivity or bright sunlight
 *          - **Erratic readings**: Electrical noise, loose connection
 *          
 *          Troubleshooting:
 *          - **NotConnected**: Verify RNGFND_TYPE, check I2C address/serial port
 *          - **NoData**: Test sensor over different surfaces
 *          - **OutOfRange**: Verify MIN_CM and MAX_CM parameters
 *          - **Poor Quality**: Adjust mounting angle, avoid bright sunlight
 *          - **Water**: Lidars don't work over water - use radar instead
 * 
 * @note Multiple RFND messages if multiple rangefinders configured
 * @note Rangefinder data fused with barometer in EKF for improved altitude
 * @note status=4 (Good) required before data used for navigation
 * @note Lidar ineffective over water - radar recommended for boats
 * 
 * @warning status=0 indicates sensor failure or misconfiguration
 * @warning Out of range readings (status=2 or 3) not used for control
 * @warning quality=0 may indicate unreliable measurement
 * 
 * @see RNGFND_* parameters for rangefinder configuration
 * @see RangeFinder::Status enum for status values
 * @see Rotation enum for orientation values
 * @see AP_RangeFinder library for sensor drivers
 */
struct PACKED log_RFND {
    LOG_PACKET_HEADER;          ///< Standard 3-byte header: head1, head2, msgid
    uint64_t time_us;           ///< Timestamp in microseconds since system boot
    uint8_t instance;           ///< Rangefinder instance number (0-9)
    float dist;                 ///< Measured distance to target in meters
    uint8_t status;             ///< Sensor status (0=NotConnected, 1=NoData, 2=OutOfRangeLow, 3=OutOfRangeHigh, 4=Good)
    uint8_t orient;             ///< Sensor mounting orientation (Rotation enum: 25=Down, 24=Up, 0=Forward, etc.)
    int8_t quality;             ///< Signal quality (-1=invalid, 0=poor, 100=excellent)
};

/**
 * @struct log_TERRAIN
 * @brief TERR message: Terrain database information for terrain following
 * 
 * @details The TERR message logs terrain altitude database status, enabling accurate terrain
 *          following in AUTO missions. ArduPilot can download global terrain data from ground
 *          stations or use pre-loaded terrain files to maintain constant height above ground
 *          during waypoint missions, even over varying elevation.
 *          
 *          TERR Message Purpose:
 *          - **Terrain Following**: Maintain constant AGL (Above Ground Level) in AUTO mode
 *          - **Database Status**: Monitor terrain data availability and quality
 *          - **Coverage Analysis**: Verify terrain data loaded for mission area
 *          - **Memory Usage**: Track terrain tile cache usage
 *          - **Diagnostics**: Debug terrain following issues
 *          
 *          Terrain System Overview:
 *          ArduPilot's terrain system provides global elevation data:
 *          - **Data Source**: SRTM (Shuttle Radar Topography Mission) worldwide elevation
 *          - **Resolution**: 100m grid spacing globally, 30m in some regions
 *          - **Coverage**: Latitude ±60° (most inhabited areas)
 *          - **Delivery**: Downloaded from GCS during flight or pre-loaded on SD card
 *          - **Storage**: Cached in RAM tiles (typically 28x22 grid points per tile)
 *          
 *          How Terrain Following Works:
 *          @code
 *          // Mission planning with terrain
 *          1. Set waypoint altitude to "Terrain" frame instead of "Relative" or "Absolute"
 *          2. Vehicle requests terrain data for mission area from GCS
 *          3. GCS sends terrain tiles via MAVLink TERRAIN_DATA messages
 *          4. Vehicle caches tiles in RAM (keeps nearest tiles in memory)
 *          5. During mission, vehicle looks up terrain elevation at current GPS position
 *          6. Vehicle adjusts altitude to maintain constant height above terrain
 *          
 *          // Example terrain waypoint
 *          Waypoint 5: Lat=-35.363, Lon=149.165, Alt=50m (Terrain frame)
 *              Terrain elevation = 580m AMSL
 *              Vehicle flies at 580m + 50m = 630m AMSL
 *          
 *          Waypoint 6: Lat=-35.364, Lon=149.166, Alt=50m (Terrain frame)
 *              Terrain elevation = 620m AMSL (hill)
 *              Vehicle climbs to 620m + 50m = 670m AMSL (maintains 50m AGL)
 *          @endcode
 *          
 *          status (Terrain System Status):
 *          Terrain database operational status:
 *          - **AP_Terrain::TerrainStatus enum values**:
 *            - 0: TerrainStatusDisabled - Terrain system disabled (TERRAIN_ENABLE=0)
 *            - 1: TerrainStatusUnhealthy - System enabled but data unavailable
 *            - 2: TerrainStatusOK - System operational, data available for current location
 *          
 *          Status Interpretation:
 *          @code
 *          status = 0 (Disabled)
 *              - TERRAIN_ENABLE = 0
 *              - Terrain following not available
 *              - Terrain frame waypoints rejected
 *          
 *          status = 1 (Unhealthy)
 *              - System enabled but no terrain data for current location
 *              - No GCS connection to download data
 *              - SD card missing or terrain files not present
 *              - Outside terrain database coverage (>60° latitude)
 *              - Pending tile downloads not yet complete
 *          
 *          status = 2 (OK)
 *              - Terrain data available for current GPS position
 *              - Terrain following operational
 *              - Terrain frame waypoints will work correctly
 *          @endcode
 *          
 *          lat/lng (Current Vehicle Position):
 *          Current GPS position for terrain lookup:
 *          - **Units**: Degrees * 1e7 (multiply by 0.0000001 to get degrees)
 *          - **lat**: Latitude in degE7 format (-90° to +90°)
 *          - **lng**: Longitude in degE7 format (-180° to +180°)
 *          - **Purpose**: Position where terrain elevation is being queried
 *          
 *          spacing (Terrain Grid Spacing):
 *          Resolution of terrain data grid:
 *          - **Units**: Meters
 *          - **Typical**: 100m (standard global SRTM data)
 *          - **High-res**: 30m (available in some regions)
 *          - **Meaning**: Distance between elevation data points
 *          - **Interpolation**: Vehicle interpolates between grid points
 *          
 *          terrain_height (Terrain Elevation):
 *          Ground elevation at current GPS position:
 *          - **Units**: Meters AMSL (Above Mean Sea Level)
 *          - **Source**: Interpolated from terrain database
 *          - **Range**: -500m to 9000m (sea level to highest mountains)
 *          - **Accuracy**: ±10-20m typical (SRTM accuracy)
 *          - **Use**: Combined with current altitude to calculate AGL height
 *          
 *          current_height (Height Above Terrain):
 *          Vehicle's current altitude above ground level:
 *          - **Units**: Meters
 *          - **Calculation**: current_altitude_AMSL - terrain_height
 *          - **Typical**: 50-200m for terrain following missions
 *          - **Monitoring**: Verify vehicle maintains target AGL
 *          - **Safety**: Should match waypoint terrain altitude setting
 *          
 *          pending (Pending Tile Requests):
 *          Number of terrain tiles requested but not yet received:
 *          - **Range**: 0-100+
 *          - **0**: All needed tiles already in cache
 *          - **High**: Many tiles being downloaded from GCS
 *          - **Monitoring**: Should decrease over time as tiles arrive
 *          - **Issue**: Stuck high value indicates download problem
 *          
 *          loaded (Loaded Tiles in Cache):
 *          Number of terrain tiles currently in RAM cache:
 *          - **Range**: 0 to TERRAIN_CACHE_SIZE (default 20-30 tiles)
 *          - **Management**: Least-recently-used tiles evicted when cache full
 *          - **Coverage**: Each tile covers approximately 2.8km x 2.2km (at 100m spacing)
 *          - **Typical**: 5-15 tiles for local operation
 *          
 *          reference_offset (Terrain Reference):
 *          Terrain altitude at arming location for reference:
 *          - **Units**: Meters
 *          - **Purpose**: Difference between home altitude and terrain at home
 *          - **Use**: Fallback reference for terrain calculations
 *          - **Set**: At arming time based on home position terrain
 *          
 *          Terrain Following Mission Example:
 *          @code
 *          // Before mission start (on ground, armed)
 *          TERR: Status=2, Lat=-353630000, Lon=1491650000, Spacing=100,
 *                TerrH=580.5, CHeight=0.0, Pending=0, Loaded=8, ROfs=580.5
 *                (OK status, on ground, 8 tiles loaded, terrain at 580.5m)
 *          
 *          // During mission over flat terrain
 *          TERR: Status=2, Lat=-353640000, Lon=1491660000, Spacing=100,
 *                TerrH=582.3, CHeight=50.2, Pending=0, Loaded=9, ROfs=580.5
 *                (Flying 50m AGL, terrain 582m, all tiles loaded)
 *          
 *          // Approaching hill
 *          TERR: Status=2, Lat=-353650000, Lon=1491670000, Spacing=100,
 *                TerrH=620.8, CHeight=49.8, Pending=2, Loaded=12, ROfs=580.5
 *                (Hill ahead, vehicle climbing, 2 new tiles being downloaded)
 *          
 *          // Over hill
 *          TERR: Status=2, Lat=-353660000, Lon=1491680000, Spacing=100,
 *                TerrH=638.2, CHeight=50.1, Pending=0, Loaded=14, ROfs=580.5
 *                (Maintaining 50m AGL over elevated terrain)
 *          
 *          // Terrain data unavailable
 *          TERR: Status=1, Lat=-353670000, Lon=1491690000, Spacing=100,
 *                TerrH=0.0, CHeight=-99.0, Pending=5, Loaded=10, ROfs=580.5
 *                (Unhealthy - no terrain data, 5 tiles pending but not arriving)
 *          @endcode
 *          
 *          Terrain System Parameters:
 *          - **TERRAIN_ENABLE**: Enable/disable terrain following (0=off, 1=on)
 *          - **TERRAIN_SPACING**: Grid spacing in meters (typically 100)
 *          - **TERRAIN_CACHE_SIZE**: Number of tiles to keep in RAM
 *          - **TERRAIN_FOLLOW**: Enable terrain following in AUTO mode
 *          - **WPNAV_RFND_USE**: Use rangefinder for terrain when available
 *          
 *          Mission Planning Considerations:
 *          - **Waypoint Frame**: Set to "Terrain" for constant AGL altitude
 *          - **Pre-Load**: Can upload terrain tiles to SD card before flight
 *          - **GCS Connection**: Need telemetry link to download tiles during flight
 *          - **Coverage**: Verify terrain data available for mission area (±60° lat)
 *          - **Accuracy**: SRTM ±10-20m accuracy - add safety margin
 *          
 *          Common Issues:
 *          - **Status=1**: No terrain data available
 *              - Check GCS connection for tile downloads
 *              - Verify SD card has terrain files
 *              - Check mission area within ±60° latitude
 *          - **High Pending**: Tiles not downloading
 *              - Check telemetry bandwidth (low bandwidth = slow downloads)
 *              - Verify GCS has internet connection for SRTM data
 *          - **Loaded=0**: No tiles in cache
 *              - Check TERRAIN_ENABLE parameter
 *              - Verify GPS has valid 3D fix
 *          
 *          Troubleshooting:
 *          - **Enable Debug**: Set LOG_BITMASK to include terrain logging
 *          - **Verify Status**: Should be 2 (OK) before terrain missions
 *          - **Check Coverage**: Confirm mission within terrain database range
 *          - **Pre-Load**: Download terrain files to SD card before flight
 *          - **Bandwidth**: Terrain downloads need ~1-5 kB/s bandwidth
 * 
 * @note Terrain data downloaded from GCS via MAVLink TERRAIN_DATA messages
 * @note Each tile covers approximately 2.8km x 2.2km at 100m spacing
 * @note Terrain following only works in AUTO mode with terrain-frame waypoints
 * @note Rangefinder data can supplement terrain database if available
 * 
 * @warning status=1 (Unhealthy) prevents terrain-frame waypoint execution
 * @warning Terrain accuracy ±10-20m - add safety margin for obstacle clearance
 * @warning No terrain data available outside ±60° latitude
 * @warning High pending count with no decrease indicates download failure
 * 
 * @see TERRAIN_* parameters for configuration
 * @see AP_Terrain::TerrainStatus enum for status values
 * @see WPNAV_RFND_USE to combine rangefinder with terrain data
 * @see Mission Planner or QGroundControl for terrain-frame waypoint planning
 */
struct PACKED log_TERRAIN {
    LOG_PACKET_HEADER;          ///< Standard 3-byte header: head1, head2, msgid
    uint64_t time_us;           ///< Timestamp in microseconds since system boot
    uint8_t status;             ///< Terrain system status (0=Disabled, 1=Unhealthy, 2=OK)
    int32_t lat;                ///< Current vehicle latitude in degrees * 1e7
    int32_t lng;                ///< Current vehicle longitude in degrees * 1e7
    uint16_t spacing;           ///< Terrain grid spacing in meters (typically 100m)
    float terrain_height;       ///< Ground elevation at current position in meters AMSL
    float current_height;       ///< Vehicle height above terrain in meters
    uint16_t pending;           ///< Number of terrain tiles requested but not yet received
    uint16_t loaded;            ///< Number of terrain tiles currently loaded in RAM cache
    float reference_offset;     ///< Terrain altitude at arming location in meters
};

/**
 * @struct log_ARSP
 * @brief ARSP message: Airspeed sensor data for fixed-wing aircraft
 * 
 * @details The ARSP message logs airspeed measurements from pitot tube or other airspeed sensors,
 *          critical for fixed-wing flight control. Airspeed is the velocity of the aircraft
 *          relative to the surrounding air mass, essential for maintaining proper lift, avoiding
 *          stalls, and calculating wind velocity.
 *          
 *          ARSP Message Purpose:
 *          - **Stall Prevention**: Monitor airspeed to avoid dropping below stall speed
 *          - **Performance**: Maintain optimal cruise speed for efficiency
 *          - **Wind Estimation**: Calculate wind velocity (ground speed - airspeed)
 *          - **Control Authority**: Higher airspeed = more control surface effectiveness
 *          - **TECS Energy Management**: Total Energy Control System uses airspeed/altitude
 *          
 *          Airspeed Sensor Types:
 *          - **MS4525**: Analog differential pressure sensor (most common)
 *          - **MS5525**: Analog differential pressure with integrated barometer
 *          - **DLVR**: Honeywell digital pressure sensors
 *          - **SDP3x**: Sensirion digital sensors
 *          - **Analog**: Simple analog voltage output sensors
 *          - **NMEA**: Serial NMEA-format airspeed
 *          - **ASP5033**: TE Connectivity sensor
 *          
 *          Pitot Tube Fundamentals:
 *          @code
 *          Pitot tube measures dynamic pressure:
 *          - Total Pressure Port (forward-facing): Static + Dynamic pressure
 *          - Static Pressure Port (side): Static pressure only
 *          - Differential = Total - Static = Dynamic pressure
 *          
 *          Bernoulli equation:
 *          Dynamic_Pressure = 0.5 * air_density * velocity^2
 *          
 *          Airspeed calculation:
 *          airspeed = sqrt((2 * differential_pressure) / air_density)
 *          @endcode
 *          
 *          instance (Airspeed Sensor Instance):
 *          Airspeed sensor instance number:
 *          - **Range**: 0-1 (up to 2 airspeed sensors supported)
 *          - **0**: Primary airspeed sensor
 *          - **1**: Secondary/backup airspeed sensor
 *          - **Redundancy**: Dual sensors for safety-critical applications
 *          
 *          airspeed (Calibrated Airspeed):
 *          Measured airspeed velocity:
 *          - **Units**: Meters per second (m/s)
 *          - **Range**: 0-100 m/s (typical 10-40 m/s for small aircraft)
 *          - **Resolution**: 0.01 m/s typical
 *          - **Accuracy**: ±0.5 m/s with proper calibration
 *          - **Conversion**: 1 m/s = 1.944 knots = 3.6 km/h
 *          
 *          Airspeed Ranges:
 *          @code
 *          Typical fixed-wing airspeed ranges:
 *          - **Stall Speed**: 8-15 m/s (depends on wing loading)
 *          - **Cruise Speed**: 15-25 m/s (small UAVs)
 *          - **Maximum Speed**: 30-50 m/s
 *          - **Landing Approach**: 12-18 m/s
 *          - **Takeoff**: 12-20 m/s
 *          
 *          Speed in different units:
 *          15 m/s = 29.2 knots = 54 km/h = 33.5 mph
 *          20 m/s = 38.9 knots = 72 km/h = 44.7 mph
 *          25 m/s = 48.6 knots = 90 km/h = 55.9 mph
 *          @endcode
 *          
 *          diffpressure (Differential Pressure):
 *          Raw pressure difference measured by sensor:
 *          - **Units**: Pascals (Pa)
 *          - **Range**: 0-2000 Pa typical (0-50 m/s airspeed)
 *          - **Zero Offset**: ~0 Pa at zero airspeed (after calibration)
 *          - **Relationship**: diffpressure = 0.5 * rho * V^2
 *          
 *          Differential Pressure Calculation:
 *          @code
 *          // At sea level, standard atmosphere (rho = 1.225 kg/m^3)
 *          diffpressure = 0.5 * 1.225 * airspeed^2
 *          
 *          // Examples:
 *          airspeed = 15 m/s  =>  diffpressure = 138 Pa
 *          airspeed = 20 m/s  =>  diffpressure = 245 Pa
 *          airspeed = 25 m/s  =>  diffpressure = 383 Pa
 *          @endcode
 *          
 *          temperature (Sensor Temperature):
 *          Temperature measured at airspeed sensor:
 *          - **Units**: Degrees Celsius * 100 (divide by 100 for °C)
 *          - **Range**: -4000 to +8500 (-40°C to +85°C)
 *          - **Purpose**: Temperature compensation for air density
 *          - **Typical**: 2000-4000 (20-40°C) at typical flight altitudes
 *          
 *          rawpressure (Raw Pressure Reading):
 *          Uncompensated pressure measurement:
 *          - **Units**: Pascals (Pa)
 *          - **Purpose**: Raw sensor output before offset correction
 *          - **Diagnostic**: Compare with diffpressure to verify offset
 *          - **Calibration**: rawpressure - offset = diffpressure
 *          
 *          offset (Pressure Offset/Zero Calibration):
 *          Zero-airspeed pressure offset:
 *          - **Units**: Pascals (Pa)
 *          - **Purpose**: Correct for sensor zero-point error
 *          - **Calibration**: Measured at zero airspeed (stationary on ground)
 *          - **Typical**: -50 to +50 Pa
 *          - **Auto-calibration**: Updated during pre-flight or in-flight zero detection
 *          - **Parameter**: ARSPD_OFFSET can manually set offset
 *          
 *          Offset Calibration:
 *          @code
 *          // Pre-flight offset calibration
 *          1. Vehicle stationary, airspeed should be zero
 *          2. Measure rawpressure over several seconds
 *          3. offset = average(rawpressure)
 *          4. diffpressure = rawpressure - offset
 *          
 *          // In-flight auto-calibration
 *          if (groundspeed < 3 m/s && GPS_valid) {
 *              // Assume zero airspeed (light wind)
 *              offset = exponential_average(rawpressure);
 *          }
 *          @endcode
 *          
 *          use (Sensor In Use Flag):
 *          Whether this sensor is being used for flight control:
 *          - **true**: Sensor data actively used by flight controller
 *          - **false**: Sensor online but not used (backup, unhealthy, or disabled)
 *          - **Selection**: Based on health, calibration, and ARSPD_PRIMARY
 *          
 *          healthy (Sensor Health Status):
 *          Whether sensor is functioning correctly:
 *          - **true**: Sensor passing all health checks
 *          - **false**: Sensor failed health checks, not used
 *          - **Checks**: Innovation test, range check, timeout check
 *          
 *          Health Failure Reasons:
 *          @code
 *          healthy = false when:
 *          - Sensor not responding (timeout)
 *          - Airspeed unreasonable (negative, too high)
 *          - Innovation test failure (airspeed disagrees with EKF/GPS)
 *          - Blocked pitot tube (constant reading)
 *          - Damaged sensor
 *          @endcode
 *          
 *          health_prob (Health Probability):
 *          Probability estimate that sensor is healthy (0.0-1.0):
 *          - **Range**: 0.0 (definitely unhealthy) to 1.0 (definitely healthy)
 *          - **Purpose**: Gradual health assessment, smoother than binary
 *          - **Use**: Sensor selection, warnings
 *          - **Typical**: >0.95 for good sensor
 *          
 *          test_ratio (Innovation Test Ratio):
 *          Statistical test comparing airspeed to other estimates:
 *          - **Range**: 0.0-1.0+ (typically 0.1-0.5 for good data)
 *          - **Meaning**: Ratio of innovation to innovation variance
 *          - **Purpose**: Detect sensor failure or blockage
 *          - **Threshold**: >1.0 typically indicates problem
 *          
 *          Innovation Test:
 *          @code
 *          // Compare measured airspeed to EKF predicted airspeed
 *          predicted_airspeed = sqrt(vel_N^2 + vel_E^2) - wind_estimate
 *          innovation = measured_airspeed - predicted_airspeed
 *          test_ratio = abs(innovation) / sqrt(innovation_variance)
 *          
 *          if (test_ratio > 1.0) {
 *              // Measured airspeed disagrees with EKF
 *              health_prob -= 0.1;  // reduce confidence
 *          }
 *          @endcode
 *          
 *          primary (Primary Sensor Flag):
 *          Which sensor is selected as primary:
 *          - **0**: This sensor is not primary
 *          - **1**: This sensor is currently primary
 *          - **Selection**: Based on ARSPD_PRIMARY parameter and health
 *          
 *          Typical Flight Scenario:
 *          @code
 *          // Takeoff - accelerating
 *          ARSP: I=0, Airspeed=12.5, DiffP=96, Temp=2500, RawP=98, Ofs=2, U=1, H=1, Hp=0.99, TR=0.15, Pri=1
 *                (12.5 m/s, healthy, in use, good innovation, primary sensor)
 *          
 *          // Cruise flight
 *          ARSP: I=0, Airspeed=20.3, DiffP=252, Temp=1800, RawP=254, Ofs=2, U=1, H=1, Hp=1.00, TR=0.08, Pri=1
 *                (20.3 m/s cruise, cold air at altitude, excellent health)
 *          
 *          // Approach to land - slowing down
 *          ARSP: I=0, Airspeed=15.8, DiffP=153, Temp=2200, RawP=155, Ofs=2, U=1, H=1, Hp=0.98, TR=0.12, Pri=1
 *                (15.8 m/s approach speed, still healthy)
 *          
 *          // Blocked pitot tube - sensor failure
 *          ARSP: I=0, Airspeed=0.0, DiffP=-2, Temp=2500, RawP=0, Ofs=2, U=0, H=0, Hp=0.02, TR=5.23, Pri=0
 *                (Zero airspeed but vehicle moving - blocked tube, not in use, unhealthy, high innovation)
 *          @endcode
 *          
 *          ARSPD_* Parameters:
 *          - **ARSPD_TYPE**: Sensor type (MS4525, MS5525, etc.)
 *          - **ARSPD_USE**: Enable/disable airspeed sensor
 *          - **ARSPD_OFFSET**: Manual pressure offset
 *          - **ARSPD_RATIO**: Airspeed scaling ratio (calibration)
 *          - **ARSPD_PIN**: Analog sensor pin (for analog sensors)
 *          - **ARSPD_AUTOCAL**: Enable automatic offset calibration
 *          - **ARSPD_PRIMARY**: Select primary sensor (0 or 1)
 *          
 *          Common Issues:
 *          - **healthy=0**: Sensor failure
 *              - Check pitot tube for blockage (insects, ice, dirt)
 *              - Verify sensor power and I2C connection
 *              - Check for physical damage to pitot tube
 *          - **High test_ratio**: Airspeed disagrees with GPS/EKF
 *              - Pitot tube partially blocked
 *              - Incorrect ARSPD_RATIO calibration
 *              - Pitot tube facing wrong direction
 *          - **Negative airspeed**: Offset calibration error
 *              - Perform offset calibration on ground
 *              - Check ARSPD_AUTOCAL enabled
 *          
 *          Troubleshooting:
 *          - **Zero Reading**: Check pitot tube not blocked, sensor connected
 *          - **Erratic**: Check for leaks in pneumatic connections
 *          - **Offset**: Calibrate on ground with ARSPD_AUTOCAL
 *          - **Scale Error**: Perform in-flight calibration with ARSPD_RATIO
 * 
 * @note Airspeed critical for fixed-wing flight - monitor health carefully
 * @note Pitot tube must face forward into airflow for accurate readings
 * @note Multiple sensors recommended for safety-critical applications
 * @note Temperature affects air density and thus airspeed calculation
 * 
 * @warning healthy=0 indicates sensor failure - may trigger airspeed failsafe
 * @warning Blocked pitot tube is common cause of crashes - inspect before flight
 * @warning high test_ratio indicates sensor disagreement with other data sources
 * @warning use=0 means sensor not used - flight control using estimates only
 * 
 * @see ARSPD_* parameters for airspeed sensor configuration
 * @see AP_Airspeed library for sensor drivers
 * @see TECS algorithm for airspeed/altitude energy management
 * @see ARSPD_FBW_MIN and ARSPD_FBW_MAX for speed limits
 */
struct PACKED log_ARSP {
    LOG_PACKET_HEADER;          ///< Standard 3-byte header: head1, head2, msgid
    uint64_t time_us;           ///< Timestamp in microseconds since system boot
    uint8_t instance;           ///< Airspeed sensor instance number (0-1)
    float   airspeed;           ///< Calibrated airspeed in meters per second
    float   diffpressure;       ///< Differential pressure in Pascals (after offset correction)
    int16_t temperature;        ///< Sensor temperature in degrees Celsius * 100
    float   rawpressure;        ///< Raw pressure measurement in Pascals (before offset)
    float   offset;             ///< Pressure offset/zero calibration in Pascals
    bool    use;                ///< true if sensor is being used for flight control
    bool    healthy;            ///< true if sensor passing health checks
    float   health_prob;        ///< Health probability estimate (0.0=unhealthy, 1.0=healthy)
    float   test_ratio;         ///< Innovation test ratio (>1.0 indicates disagreement)
    uint8_t primary;            ///< Primary sensor flag (1=primary, 0=backup)
};

/**
 * @struct log_DMS
 * @brief DMS message: DataFlash-Over-MAVLink streaming statistics
 * 
 * @details The DMS message logs statistics for DataFlash log streaming over MAVLink telemetry,
 *          allowing real-time log download during flight. This feature enables ground stations
 *          to receive log data without physical SD card access, useful for remote operations,
 *          continuous monitoring, and immediate post-flight analysis.
 *          
 *          DMS Message Purpose:
 *          - **Streaming Health**: Monitor real-time log download performance
 *          - **Bandwidth Usage**: Track telemetry bandwidth consumed by logging
 *          - **Packet Loss**: Identify dropped blocks and retransmission overhead
 *          - **Buffer Management**: Monitor block queue states (free, pending, sent)
 *          - **Performance Tuning**: Optimize streaming rate vs. bandwidth
 *          
 *          DataFlash-Over-MAVLink Overview:
 *          ArduPilot can stream binary logs to ground station during flight:
 *          - **Protocol**: MAVLink LOG_DATA messages (ID 120)
 *          - **Block Size**: 90 bytes per MAVLink message
 *          - **Bandwidth**: Typically 5-50 kB/s depending on telemetry link
 *          - **Reliability**: Acknowledged protocol with retry mechanism
 *          - **Storage**: Still writes to SD card simultaneously
 *          
 *          How Log Streaming Works:
 *          @code
 *          1. Ground station requests log streaming (LOG_REQUEST_DATA)
 *          2. ArduPilot allocates block buffers for streaming
 *          3. Log data written to both SD card and streaming buffers
 *          4. Blocks move: FREE → PENDING → SENT
 *          5. GCS acknowledges received blocks
 *          6. Acknowledged blocks return to FREE pool
 *          7. Un-acknowledged blocks retry after timeout
 *          @endcode
 *          
 *          Block State Machine:
 *          @code
 *          FREE blocks:
 *              - Available for new log data
 *              - Not yet filled with data
 *              - Waiting to be written
 *          
 *          PENDING blocks:
 *              - Filled with log data
 *              - Queued for transmission
 *              - Waiting for telemetry bandwidth
 *          
 *          SENT blocks:
 *              - Transmitted to GCS
 *              - Awaiting acknowledgment
 *              - Will retry if not ACKed
 *          
 *          RETRY blocks (not in this message):
 *              - Previously sent but not acknowledged
 *              - Scheduled for retransmission
 *          @endcode
 *          
 *          seqno (Sequence Number):
 *          Current block sequence number being processed:
 *          - **Range**: 0 to 2^32-1
 *          - **Increments**: Each time a new block is filled with data
 *          - **Purpose**: Track streaming progress
 *          - **Monotonic**: Always increases (wraps at 32-bit limit)
 *          - **Use**: GCS uses to detect missing blocks
 *          
 *          dropped (Dropped Block Count):
 *          Number of log blocks rejected/dropped:
 *          - **Reason**: Insufficient free blocks available
 *          - **Impact**: Some log data lost from stream (but still on SD card)
 *          - **Causes**:
 *            - Telemetry link too slow for log rate
 *            - High logging rate (many messages enabled)
 *            - Telemetry bandwidth consumed by other traffic
 *          - **Ideal**: Should remain 0 or very low
 *          
 *          Dropped Block Interpretation:
 *          @code
 *          dropped = 0
 *              - All log data successfully queued for streaming
 *              - Bandwidth sufficient for current log rate
 *          
 *          dropped > 0 and increasing
 *              - Log streaming cannot keep up
 *              - Some data being written only to SD card
 *              - Need to reduce log rate or increase bandwidth
 *          
 *          dropped = 1000+
 *              - Significant data loss in stream
 *              - Streaming effectively failing
 *              - Must use SD card for complete logs
 *          @endcode
 *          
 *          retries (Retry Count):
 *          Number of block retransmissions due to lost acknowledgments:
 *          - **Cause**: Blocks sent but ACK not received from GCS
 *          - **Timeout**: Typically 3-5 seconds before retry
 *          - **Impact**: Increased bandwidth usage, slower overall transfer
 *          - **Typical**: Low (<10%) for good telemetry links
 *          
 *          resends (Resend Count):
 *          Total number of blocks resent to GCS:
 *          - **Includes**: All retransmission attempts
 *          - **Purpose**: Track retransmission overhead
 *          - **Efficiency**: Low resends = efficient streaming
 *          - **High Resends**: Indicates poor telemetry link quality
 *          
 *          Retry vs Resend:
 *          @code
 *          // Block initially sent
 *          SENT block 100 → GCS
 *          
 *          // No ACK received after timeout
 *          retries++  // Block moved back to retry queue
 *          
 *          // Block retransmitted
 *          resends++  // Block resent
 *          SENT block 100 → GCS (again)
 *          
 *          // Multiple retries possible for same block
 *          If still no ACK: retries++, resends++, send again...
 *          @endcode
 *          
 *          state_free_avg/min/max (Free Block Statistics):
 *          Statistics on FREE block pool availability:
 *          - **avg**: Average number of free blocks during period
 *          - **min**: Minimum free blocks (lowest availability)
 *          - **max**: Maximum free blocks (highest availability)
 *          - **Range**: 0 to total allocated blocks (typically 30-50)
 *          - **Health**: Higher = better (more capacity)
 *          
 *          Free Block Interpretation:
 *          @code
 *          state_free_min = 20-30
 *              - Healthy: Plenty of free blocks available
 *              - Streaming keeping up with logging rate
 *          
 *          state_free_min = 5-10
 *              - Marginal: Occasional buffer pressure
 *              - May drop blocks during high log rate bursts
 *          
 *          state_free_min = 0-2
 *              - Critical: Running out of buffers
 *              - Definitely dropping blocks (see dropped counter)
 *              - Need more bandwidth or reduce log rate
 *          @endcode
 *          
 *          state_pending_avg/min/max (Pending Block Statistics):
 *          Statistics on PENDING block queue (waiting to send):
 *          - **avg**: Average number of blocks waiting transmission
 *          - **min**: Minimum pending (best throughput)
 *          - **max**: Maximum pending (worst backlog)
 *          - **Health**: Lower = better (catching up quickly)
 *          
 *          Pending Block Interpretation:
 *          @code
 *          state_pending_avg = 0-5
 *              - Excellent: Minimal queue backlog
 *              - Bandwidth sufficient for current rate
 *          
 *          state_pending_avg = 10-20
 *              - Moderate: Some backlog but manageable
 *              - Streaming slightly behind logging rate
 *          
 *          state_pending_avg = 30+
 *              - High backlog: Cannot catch up
 *              - Will eventually drop blocks
 *              - Need more bandwidth
 *          @endcode
 *          
 *          state_sent_avg/min/max (Sent Block Statistics):
 *          Statistics on SENT block queue (awaiting ACK):
 *          - **avg**: Average number of blocks waiting for acknowledgment
 *          - **min**: Minimum sent (fastest ACK turnaround)
 *          - **max**: Maximum sent (slowest ACK turnaround)
 *          - **Latency**: Higher values indicate longer round-trip time
 *          
 *          Sent Block Interpretation:
 *          @code
 *          state_sent_avg = 0-5
 *              - Fast ACKs: Low latency telemetry link
 *              - Quick buffer turnover
 *          
 *          state_sent_avg = 10-20
 *              - Moderate latency: Typical for long-range links
 *              - Still healthy if not growing
 *          
 *          state_sent_avg = 30+
 *              - High latency or packet loss
 *              - Many unacknowledged blocks
 *              - Risk of buffer exhaustion
 *          @endcode
 *          
 *          Typical Streaming Scenarios:
 *          @code
 *          // Healthy streaming - good bandwidth
 *          DMS: N=5234, Dp=0, RT=2, RS=2, Fa=35, Fmn=32, Fmx=38, Pa=3, Pmn=0, Pmx=8, Sa=5, Smn=3, Smx=7
 *               (No drops, minimal retries, plenty of free blocks, low pending, fast ACKs)
 *          
 *          // Marginal - limited bandwidth
 *          DMS: N=6891, Dp=15, RT=45, RS=52, Fa=15, Fmn=8, Fmx=25, Pa=18, Pmn=10, Pmx=30, Sa=12, Smn=5, Smx=20
 *               (Some drops, moderate retries, lower free blocks, higher pending)
 *          
 *          // Failing - insufficient bandwidth
 *          DMS: N=8123, Dp=456, RT=234, RS=298, Fa=3, Fmn=0, Fmx=12, Pa=35, Pmn=25, Pmx=45, Sa=25, Smn=15, Smx=35
 *               (Many drops, high retries, critically low free blocks, high pending and sent)
 *          
 *          // Excellent link - high bandwidth
 *          DMS: N=10500, Dp=0, RT=0, RS=0, Fa=42, Fmn=40, Fmx=45, Pa=1, Pmn=0, Pmx=3, Sa=2, Smn=0, Smx=4
 *               (No drops, no retries, maximum free blocks, minimal pending, instant ACKs)
 *          @endcode
 *          
 *          LOG_* Parameters:
 *          - **LOG_BACKEND_TYPE**: Include MAVLink in backend list (512 bit)
 *          - **LOG_DSMLOG**: Enable/disable DataFlash streaming
 *          - **LOG_MAV_BUFSIZE**: MAVLink log buffer size
 *          - **LOG_BITMASK**: Which messages to log (affects rate)
 *          - **LOG_DISARMED**: Log while disarmed (affects data volume)
 *          
 *          Performance Optimization:
 *          - **Reduce Log Rate**: Disable unnecessary LOG_BITMASK bits
 *          - **Increase Bandwidth**: Use higher telemetry baud rate
 *          - **Dedicated Link**: Use separate radio for log streaming
 *          - **Compression**: Future feature to reduce bandwidth
 *          - **Selective Logging**: Log only important messages
 *          
 *          Troubleshooting:
 *          - **High dropped**: Increase telemetry bandwidth or reduce LOG_BITMASK
 *          - **High retries**: Check RF link quality, reduce interference
 *          - **Low free blocks**: Increase LOG_MAV_BUFSIZE or reduce logging rate
 *          - **High pending**: Telemetry too slow for log rate
 * 
 * @note Log streaming runs concurrently with SD card logging
 * @note Streaming requires MAVLink bandwidth - competes with telemetry
 * @note Dropped blocks still saved to SD card (not lost, just not streamed)
 * @note GCS must request streaming with LOG_REQUEST_DATA command
 * 
 * @warning dropped > 0 indicates streaming cannot keep up with logging rate
 * @warning state_free_min = 0 indicates critical buffer exhaustion
 * @warning High resends waste bandwidth and slow overall transfer
 * @warning Streaming consumes 5-50 kB/s bandwidth depending on log rate
 * 
 * @see LOG_BACKEND_TYPE parameter to enable MAVLink backend
 * @see LOG_BITMASK to control which messages are logged
 * @see MAVLink LOG_DATA message (ID 120)
 * @see DSF message for local SD card logging statistics
 */
struct PACKED log_DMS {
    LOG_PACKET_HEADER;          ///< Standard 3-byte header: head1, head2, msgid
    uint64_t timestamp;         ///< Timestamp in microseconds since system boot
    uint32_t seqno;             ///< Current block sequence number being processed
    uint32_t dropped;           ///< Number of blocks dropped due to insufficient buffers
    uint32_t retries;           ///< Number of block retransmissions due to lost ACKs
    uint32_t resends;           ///< Total number of blocks resent to GCS
    uint8_t state_free_avg;     ///< Average number of free blocks available
    uint8_t state_free_min;     ///< Minimum number of free blocks (critical if 0)
    uint8_t state_free_max;     ///< Maximum number of free blocks
    uint8_t state_pending_avg;  ///< Average number of blocks queued for transmission
    uint8_t state_pending_min;  ///< Minimum number of pending blocks
    uint8_t state_pending_max;  ///< Maximum number of pending blocks
    uint8_t state_sent_avg;     ///< Average number of blocks awaiting acknowledgment
    uint8_t state_sent_min;     ///< Minimum number of sent blocks
    uint8_t state_sent_max;     ///< Maximum number of sent blocks
    // uint8_t state_retry_avg; ///< (Commented out) Average retry queue depth
    // uint8_t state_retry_min; ///< (Commented out) Minimum retry queue depth
    // uint8_t state_retry_max; ///< (Commented out) Maximum retry queue depth
};

/**
 * @struct log_Rally
 * @brief RALY message: Rally point information for safe return locations
 * 
 * @details The RALY message logs rally point definitions, which are pre-programmed safe return
 *          locations used as alternatives to the home position. Rally points provide designated
 *          landing sites or safe loiter locations, useful when the home position is unsuitable
 *          (obstacles, crowds, water) or when operating over large areas.
 *          
 *          RALY Message Purpose:
 *          - **Safe Return Locations**: Define alternative return destinations
 *          - **Landing Sites**: Pre-surveyed safe landing areas
 *          - **Mission Continuity**: Document rally points used during flight
 *          - **Site Planning**: Record approved operating areas
 *          - **Compliance**: Maintain record of designated safe zones
 *          
 *          Rally Point System Overview:
 *          Rally points are GPS coordinates where the vehicle can safely:
 *          - **Loiter**: Circle at specified altitude
 *          - **Land**: Execute landing sequence (if flags permit)
 *          - **Wait**: Pause mission execution
 *          - **Recover**: Return after failsafe event
 *          
 *          Rally vs Home Position:
 *          @code
 *          HOME position:
 *              - Set at arming location
 *              - Single point
 *              - Default RTL destination
 *              - May be unsuitable for landing
 *          
 *          RALLY points:
 *              - Pre-planned locations
 *              - Multiple points (up to 255)
 *              - Chosen for suitability
 *              - Used when closer or safer than home
 *          @endcode
 *          
 *          When Rally Points Are Used:
 *          - **RTL (Return To Launch)**: If RALLY_TOTAL > 0 and enabled
 *          - **Failsafe**: Battery, GPS, radio failsafes can use rally
 *          - **Geofence Breach**: Return to nearest rally point
 *          - **Mission RTL**: End of mission return
 *          - **Manual RTL**: Pilot commands return
 *          
 *          Rally Point Selection:
 *          @code
 *          When multiple rally points exist:
 *          1. Calculate distance to each rally point
 *          2. Select nearest rally point to current position
 *          3. If ALT_OFFSET flag set, maintain relative altitude
 *          4. If LAND_AFTER_RTL, perform landing at rally
 *          5. Otherwise, loiter at rally altitude
 *          @endcode
 *          
 *          total (Total Rally Points):
 *          Total number of rally points stored:
 *          - **Range**: 0-255 rally points
 *          - **Typical**: 1-10 rally points for most operations
 *          - **Zero**: No rally points, RTL goes to home
 *          - **Storage**: Rally points stored in EEPROM/SD card
 *          - **Limit**: Practical limit ~50 due to memory
 *          
 *          Rally Point Count Usage:
 *          @code
 *          total = 0
 *              - No rally points defined
 *              - RTL behavior: Return to home position
 *          
 *          total = 1
 *              - Single rally point (always selected)
 *              - Simple alternate landing site
 *          
 *          total = 4-6
 *              - Typical for large operating area
 *              - Multiple safe landing options
 *              - Nearest rally automatically selected
 *          
 *          total = 20+
 *              - Extensive coverage area
 *              - Competition or survey missions
 *              - Ensures rally point always nearby
 *          @endcode
 *          
 *          sequence (Rally Point Sequence Number):
 *          Index of this rally point in the list:
 *          - **Range**: 0 to (total-1)
 *          - **Zero-indexed**: First rally point is sequence 0
 *          - **Unique**: Each rally point has unique sequence
 *          - **Upload Order**: Typically uploaded in sequence order
 *          
 *          latitude (Rally Point Latitude):
 *          GPS latitude coordinate of rally point:
 *          - **Format**: Degrees * 10^7 (1e7 scaling)
 *          - **Range**: -900000000 to +900000000 (-90° to +90°)
 *          - **Resolution**: ~1.1 cm at equator
 *          - **Example**: 374531234 = 37.4531234° North
 *          
 *          Latitude Conversion:
 *          @code
 *          // Raw value to degrees
 *          latitude_degrees = latitude / 10000000.0
 *          
 *          // Examples:
 *          374531234 / 1e7 = 37.4531234° N (San Francisco area)
 *          -337121234 / 1e7 = -33.7121234° S (Sydney area)
 *          514971234 / 1e7 = 51.4971234° N (London area)
 *          @endcode
 *          
 *          longitude (Rally Point Longitude):
 *          GPS longitude coordinate of rally point:
 *          - **Format**: Degrees * 10^7 (1e7 scaling)
 *          - **Range**: -1800000000 to +1800000000 (-180° to +180°)
 *          - **Resolution**: ~1.1 cm at equator (varies with latitude)
 *          - **Example**: -1224182345 = -122.4182345° West
 *          
 *          Longitude Conversion:
 *          @code
 *          // Raw value to degrees
 *          longitude_degrees = longitude / 10000000.0
 *          
 *          // Examples:
 *          -1224182345 / 1e7 = -122.4182345° W (San Francisco)
 *          1511532345 / 1e7 = 151.1532345° E (Sydney)
 *          -1275 / 1e7 = -0.0001275° (London)
 *          @endcode
 *          
 *          altitude (Rally Point Altitude):
 *          Altitude of rally point:
 *          - **Units**: Meters (integer)
 *          - **Frame**: Relative to home or absolute MSL (see flags)
 *          - **Range**: -1000 to +30000 meters typical
 *          - **Negative**: Below home/datum (uncommon)
 *          - **Typical**: 50-150m for loiter/landing
 *          
 *          Altitude Interpretation:
 *          @code
 *          // If flags indicate ALT_OFFSET (relative to home)
 *          altitude = 100
 *              → Loiter/land 100m above home altitude
 *              → If home at 500m MSL, rally at 600m MSL
 *          
 *          // If flags indicate absolute altitude
 *          altitude = 500
 *              → Loiter/land at 500m MSL
 *              → Independent of home altitude
 *          @endcode
 *          
 *          Altitude Selection Guidelines:
 *          - **Loiter Only**: 80-150m AGL typical (safe altitude)
 *          - **Landing**: 0-50m typical (landing approach altitude)
 *          - **Terrain Following**: Consider terrain elevation
 *          - **Airspace**: Comply with altitude restrictions
 *          
 *          flags (Rally Point Flags):
 *          Bitmask of rally point options:
 *          - **Bit 0 (LAND_AFTER_RTL)**: Land at rally point (else loiter)
 *          - **Bit 1 (ALT_OFFSET)**: Altitude relative to home (else absolute MSL)
 *          - **Other bits**: Reserved for future use
 *          
 *          Flag Definitions:
 *          @code
 *          flags = 0x00 (binary: 00000000)
 *              - Loiter only (no landing)
 *              - Altitude is absolute MSL
 *          
 *          flags = 0x01 (binary: 00000001) - LAND_AFTER_RTL
 *              - Land at this rally point
 *              - Execute landing sequence
 *              - Suitable landing area required
 *          
 *          flags = 0x02 (binary: 00000010) - ALT_OFFSET
 *              - Altitude relative to home
 *              - Maintains height above terrain
 *              - Adapts to varying home elevations
 *          
 *          flags = 0x03 (binary: 00000011) - LAND + ALT_OFFSET
 *              - Land at rally point
 *              - Use altitude relative to home
 *              - Common for fixed landing sites
 *          @endcode
 *          
 *          Rally Point Configuration Examples:
 *          @code
 *          // Example 1: Safe loiter point above field
 *          RALY: Tot=3, Seq=0, Lat=374531234, Lng=-1224182345, Alt=100, Flags=2
 *                (Rally 0 of 3: 37.453°N, 122.418°W, 100m above home, loiter only)
 *          
 *          // Example 2: Designated landing site
 *          RALY: Tot=3, Seq=1, Lat=374541234, Lng=-1224192345, Alt=10, Flags=3
 *                (Rally 1 of 3: Landing site, 10m above home, will land)
 *          
 *          // Example 3: Absolute altitude rally point
 *          RALY: Tot=3, Seq=2, Lat=374551234, Lng=-1224202345, Alt=500, Flags=0
 *                (Rally 2 of 3: 500m MSL absolute, loiter only)
 *          
 *          // Example 4: Emergency landing pad
 *          RALY: Tot=1, Seq=0, Lat=374561234, Lng=-1224212345, Alt=0, Flags=1
 *                (Single rally: At home elevation, land immediately)
 *          @endcode
 *          
 *          RALLY_* Parameters:
 *          - **RALLY_TOTAL**: Number of rally points stored
 *          - **RALLY_LIMIT_KM**: Maximum distance to rally (safety limit)
 *          - **RALLY_INCL_HOME**: Include home position as rally option
 *          
 *          RTL_* Parameters (interact with rally):
 *          - **RTL_ALT**: Return altitude (before descending at rally)
 *          - **RTL_ALT_FINAL**: Final altitude at rally (for loiter)
 *          - **RTL_LOIT_TIME**: Loiter time at rally before landing
 *          - **RTL_AUTOLAND**: Automatically land after loiter time
 *          
 *          Typical Use Cases:
 *          - **Large Survey Area**: Multiple rally points for quick return
 *          - **Water Operations**: Rally on shore/boat when home on water
 *          - **Populated Areas**: Rally in open field away from crowds
 *          - **Long-Range Flights**: Rally points along route
 *          - **Competition Flying**: Rally in designated safe zones
 *          
 *          Rally Point Planning:
 *          - **Clear Area**: No obstacles, trees, power lines
 *          - **GPS Reception**: Good satellite visibility
 *          - **Legal**: Approved landing/flight area
 *          - **Accessible**: Can physically reach for recovery
 *          - **Wind**: Consider prevailing wind direction
 *          - **Terrain**: Level ground for landing
 *          
 *          Common Configurations:
 *          - **Single Rally (Alt Survey)**: One rally, loiter only, absolute altitude
 *          - **Multiple Landing Sites**: 3-5 rallies, land flag set, relative altitude
 *          - **Safety Net**: Rally points surrounding operating area
 *          - **Route Planning**: Rally points along mission path
 * 
 * @note Rally points provide safer alternatives to home position
 * @note Nearest rally point automatically selected during RTL
 * @note Rally points stored persistently in EEPROM/SD card
 * @note flags bit 0: LAND_AFTER_RTL, bit 1: ALT_OFFSET
 * 
 * @warning Ensure rally points are in safe, legal landing areas
 * @warning RALLY_LIMIT_KM prevents selecting distant rally points
 * @warning Verify altitude frame (relative vs absolute) matches intent
 * @warning Landing flag requires suitable landing area at rally location
 * 
 * @see RALLY_* parameters for rally point configuration
 * @see RTL_* parameters for return-to-launch behavior
 * @see AP_Rally library for rally point management
 * @see MAVLink RALLY_POINT message for uploading rally points
 */
struct PACKED log_Rally {
    LOG_PACKET_HEADER;      ///< Standard 3-byte header: head1, head2, msgid
    uint64_t time_us;       ///< Timestamp in microseconds since system boot
    uint8_t total;          ///< Total number of rally points stored (0-255)
    uint8_t sequence;       ///< Sequence number of this rally point (0 to total-1)
    int32_t latitude;       ///< Rally point latitude in degrees * 1e7
    int32_t longitude;      ///< Rally point longitude in degrees * 1e7
    int16_t altitude;       ///< Rally altitude in meters (relative or absolute per flags)
    uint8_t flags;          ///< Rally options: bit0=LAND_AFTER_RTL, bit1=ALT_OFFSET
};

/**
 * @struct log_Performance
 * @brief PM message: Autopilot system performance and health monitoring
 * 
 * @details The PM (Performance Monitor) message logs critical real-time performance metrics
 *          of the autopilot system, tracking scheduler execution, memory usage, processor load,
 *          and internal errors. This data is essential for diagnosing performance issues,
 *          detecting resource exhaustion, and ensuring the flight controller maintains
 *          real-time deterministic operation required for safe flight.
 *          
 *          PM Message Purpose:
 *          - **Real-Time Performance**: Monitor scheduler timing and loop rates
 *          - **Resource Monitoring**: Track memory and CPU utilization
 *          - **Problem Detection**: Identify long-running tasks and timing violations
 *          - **System Health**: Detect internal errors and failures
 *          - **Tuning**: Optimize scheduler configuration and task priorities
 *          - **Safety**: Ensure deterministic real-time operation
 *          
 *          ArduPilot Scheduler Architecture:
 *          @code
 *          Main Loop (Fast Loop):
 *              - Target rate: 400 Hz (2.5ms period) for Copter
 *              - Target rate: 50-400 Hz for Plane/Rover (configurable)
 *              - Critical tasks: IMU read, attitude control, motor output
 *              - Must complete within time budget
 *          
 *          Scheduled Tasks (Slow Loop):
 *              - Various rates: 400 Hz down to 0.2 Hz
 *              - Non-critical: GPS, compass, telemetry, logging
 *              - Run between main loops when time available
 *              - Can be delayed if main loop takes too long
 *          
 *          Timing Budget:
 *              Target Loop: 2500 µs (400 Hz)
 *              Main Loop Tasks: ~1500 µs typical
 *              Scheduled Tasks: ~800 µs typical
 *              Safety Margin: ~200 µs buffer
 *          @endcode
 *          
 *          loop_rate (Main Loop Rate):
 *          Actual main loop execution rate:
 *          - **Units**: Hertz (Hz) - loops per second
 *          - **Target**: 400 Hz for Copter, 50-400 Hz for Plane/Rover
 *          - **Ideal**: Should match SCHED_LOOP_RATE parameter
 *          - **Tolerance**: ±5 Hz acceptable
 *          - **Critical**: <350 Hz indicates performance problem (for 400Hz target)
 *          
 *          Loop Rate Interpretation:
 *          @code
 *          // For SCHED_LOOP_RATE = 400 Hz
 *          loop_rate = 400 Hz
 *              - Perfect: Main loop meeting target
 *              - System has sufficient CPU headroom
 *          
 *          loop_rate = 395-405 Hz
 *              - Excellent: Within normal variation
 *              - Small timing jitter acceptable
 *          
 *          loop_rate = 380-395 Hz
 *              - Marginal: Approaching timing limits
 *              - Check for long-running tasks
 *              - May affect control performance
 *          
 *          loop_rate = <380 Hz
 *              - Critical: Missing timing deadlines
 *              - Flight control compromised
 *              - Investigate immediately
 *          @endcode
 *          
 *          num_long_running (Long-Running Task Count):
 *          Number of tasks that exceeded their time budget:
 *          - **Ideal**: 0 (no timing violations)
 *          - **Acceptable**: <5 per second (rare overruns)
 *          - **Problem**: >20 per second (frequent violations)
 *          - **Critical**: Hundreds+ (system overloaded)
 *          
 *          Long-Running Task Detection:
 *          @code
 *          Task is "long-running" when:
 *          1. Task takes longer than allocated time slice
 *          2. Delays subsequent tasks
 *          3. May cause main loop to miss deadline
 *          
 *          Common causes:
 *          - Sensor driver I2C/SPI delays
 *          - Flash filesystem writes
 *          - Complex EKF calculations
 *          - MAVLink packet processing
 *          - SD card write stalls
 *          @endcode
 *          
 *          num_loops (Number of Loops):
 *          Main loop iterations during this PM period:
 *          - **Purpose**: Normalization for other counters
 *          - **Typical**: ~400 loops (for 1 second PM period at 400 Hz)
 *          - **Calculation**: num_loops = loop_rate * PM_period
 *          
 *          max_time (Maximum Loop Time):
 *          Longest main loop execution time during period:
 *          - **Units**: Microseconds (µs)
 *          - **Target**: <2500 µs for 400 Hz loop
 *          - **Typical**: 1800-2200 µs
 *          - **Critical**: >2500 µs indicates deadline miss
 *          
 *          Loop Time Budget:
 *          @code
 *          // For 400 Hz main loop
 *          Target Period: 2500 µs (1/400 sec)
 *          
 *          max_time = 1500 µs
 *              - Excellent: 60% utilization, 40% headroom
 *          
 *          max_time = 2000 µs
 *              - Good: 80% utilization, 20% headroom
 *          
 *          max_time = 2400 µs
 *              - Marginal: 96% utilization, minimal headroom
 *          
 *          max_time = 2600+ µs
 *              - Violation: Missed deadline, loop rate will drop
 *          @endcode
 *          
 *          mem_avail (Available Memory):
 *          Free heap memory available:
 *          - **Units**: Bytes
 *          - **Range**: 1-100 kB typical (board-dependent)
 *          - **Minimum**: Should stay above 5-10 kB
 *          - **Critical**: <2 kB indicates memory exhaustion
 *          
 *          Memory Usage by Board:
 *          @code
 *          STM32F7 (2 MB flash, 512 KB RAM):
 *              mem_avail = 50000-80000 bytes typical
 *          
 *          STM32F4 (1 MB flash, 192 KB RAM):
 *              mem_avail = 20000-40000 bytes typical
 *          
 *          STM32F1 (256 KB flash, 64 KB RAM):
 *              mem_avail = 5000-15000 bytes typical
 *          
 *          Linux boards (Raspberry Pi, etc.):
 *              mem_avail varies widely (megabytes available)
 *          @endcode
 *          
 *          load (CPU Load):
 *          Processor utilization percentage:
 *          - **Units**: Percentage * 10 (divide by 10 for %)
 *          - **Range**: 0-1000 (0-100%)
 *          - **Healthy**: 300-700 (30-70%)
 *          - **Critical**: >900 (>90% load)
 *          
 *          CPU Load Calculation:
 *          @code
 *          load = (actual_execution_time / available_time) * 1000
 *          
 *          load = 500 (50%)
 *              - Healthy: 50% headroom for disturbances
 *          
 *          load = 800 (80%)
 *              - High: Limited headroom, may miss occasional deadlines
 *          
 *          load = 950+ (95%+)
 *              - Critical: Overloaded, timing violations likely
 *          @endcode
 *          
 *          internal_error_last_line (Last Error Line Number):
 *          Source code line number of most recent internal error:
 *          - **Purpose**: Identify which AP_InternalError was triggered
 *          - **Range**: 0-65535 (line number in source file)
 *          - **Usage**: Cross-reference with source code for diagnosis
 *          - **0**: No recent error
 *          
 *          internal_errors (Internal Error Bitmask):
 *          Bitmask of internal error types detected:
 *          - **Bit field**: Each bit represents different error category
 *          - **Persistent**: Once set, bit remains set
 *          - **Categories**: See AP_InternalError::error_t enum
 *          
 *          Internal Error Types:
 *          @code
 *          Common internal error bits:
 *          - Bit 0: Memory allocation failure
 *          - Bit 1: Stack overflow detection
 *          - Bit 2: Scheduler task overrun
 *          - Bit 3: Invalid parameter access
 *          - Bit 4: Unexpected nullptr
 *          - Bit 5: Flow control error
 *          - Bit 6: Failed assertion
 *          ... (see AP_InternalError.h for complete list)
 *          @endcode
 *          
 *          internal_error_count (Total Error Count):
 *          Cumulative number of internal errors:
 *          - **Ideal**: 0 (no errors)
 *          - **Purpose**: Track error frequency
 *          - **Increments**: Each time internal error triggered
 *          - **Non-zero**: Indicates software or hardware problem
 *          
 *          spi_count (SPI Transaction Count):
 *          Number of SPI bus transactions during period:
 *          - **Purpose**: Monitor SPI bus activity
 *          - **Typical**: 400-4000+ per second
 *          - **Devices**: IMU sensors, barometers, external chips
 *          - **Performance**: High counts may impact timing
 *          
 *          i2c_count (I2C Transaction Count):
 *          Number of I2C bus transactions during period:
 *          - **Purpose**: Monitor I2C bus activity
 *          - **Typical**: 100-1000 per second
 *          - **Devices**: Compass, external sensors, peripherals
 *          - **Warning**: I2C slower than SPI, can cause delays
 *          
 *          i2c_isr_count (I2C Interrupt Count):
 *          Number of I2C interrupts serviced:
 *          - **Purpose**: Track interrupt overhead
 *          - **Relationship**: Usually >> i2c_count (many interrupts per transaction)
 *          - **High Count**: May indicate bus contention or errors
 *          
 *          extra_loop_us (Extra Loop Time):
 *          Accumulated time added to loops to maintain rate:
 *          - **Units**: Microseconds (µs)
 *          - **Purpose**: Scheduler compensation for overruns
 *          - **Mechanism**: Delays added to subsequent loops to average correct rate
 *          - **Growing**: Indicates persistent overruns
 *          
 *          rtc (Real-Time Clock):
 *          System real-time clock timestamp:
 *          - **Units**: Unix epoch time (seconds since Jan 1, 1970)
 *          - **Purpose**: Absolute time correlation with GPS time
 *          - **Availability**: Only if RTC present and set
 *          - **Zero**: RTC not available or not set
 *          
 *          Performance Monitoring Scenarios:
 *          @code
 *          // Healthy system - Copter at 400 Hz
 *          PM: LR=400, NLon=0, NL=400, MaxT=1850, Mem=45000, Load=650, InE=0, ErrL=0, ErC=0
 *              (Perfect 400Hz, no overruns, 1.85ms max loop, 45kB free, 65% load, no errors)
 *          
 *          // Marginal - occasional timing issues
 *          PM: LR=398, NLon=3, NL=398, MaxT=2480, Mem=42000, Load=780, InE=0, ErrL=0, ErC=0
 *              (Slightly low rate, 3 long tasks, approaching 2.5ms limit, 78% load)
 *          
 *          // Critical - overloaded system
 *          PM: LR=350, NLon=85, NL=350, MaxT=3200, Mem=8000, Load=950, InE=0x04, ErrL=1234, ErC=12
 *              (Severely low rate, many overruns, exceeded deadline, low memory, 95% load, internal errors!)
 *          
 *          // Memory exhaustion
 *          PM: LR=380, NLon=25, NL=380, MaxT=2650, Mem=1200, Load=850, InE=0x01, ErrL=567, ErC=5
 *              (Low memory critical, allocation failures, performance degrading)
 *          @endcode
 *          
 *          SCHED_* Parameters:
 *          - **SCHED_LOOP_RATE**: Target main loop frequency (50-400 Hz)
 *          - **SCHED_DEBUG**: Enable scheduler debugging
 *          
 *          Performance Optimization:
 *          - **Reduce LOG_BITMASK**: Disable unnecessary logging
 *          - **Limit Sensors**: Use only required sensors
 *          - **Disable Features**: Turn off unused features
 *          - **Upgrade Board**: Use faster processor if needed
 *          - **Optimize Code**: Review long-running tasks
 *          
 *          Troubleshooting:
 *          - **Low loop_rate**: Check num_long_running, identify slow tasks
 *          - **High max_time**: Use SCHED_DEBUG to profile tasks
 *          - **Low mem_avail**: Reduce features, check for memory leaks
 *          - **High load**: Reduce sensor polling rates, disable features
 *          - **internal_errors**: Investigate code at internal_error_last_line
 * 
 * @note PM messages critical for diagnosing performance problems
 * @note loop_rate should match SCHED_LOOP_RATE parameter
 * @note mem_avail should stay above 5-10 kB minimum
 * @note internal_errors indicate serious software/hardware problems
 * 
 * @warning loop_rate < 350 Hz critical for 400 Hz target - flight control compromised
 * @warning mem_avail < 2 kB indicates memory exhaustion - crashes likely
 * @warning load > 90% indicates CPU overload - timing violations imminent
 * @warning internal_error_count > 0 indicates software fault - investigate immediately
 * 
 * @see SCHED_LOOP_RATE parameter for main loop target frequency
 * @see AP_Scheduler library for task scheduling
 * @see AP_InternalError for error type definitions
 * @see SCHED_DEBUG parameter to enable scheduler profiling
 */
struct PACKED log_Performance {
    LOG_PACKET_HEADER;              ///< Standard 3-byte header: head1, head2, msgid
    uint64_t time_us;               ///< Timestamp in microseconds since system boot
    uint16_t loop_rate;             ///< Actual main loop rate in Hz (should match target)
    uint16_t num_long_running;      ///< Count of tasks exceeding time budget (0 ideal)
    uint16_t num_loops;             ///< Number of main loop iterations during PM period
    uint32_t max_time;              ///< Maximum main loop execution time in microseconds
    uint32_t mem_avail;             ///< Available heap memory in bytes
    uint16_t load;                  ///< CPU load percentage * 10 (500 = 50%)
    uint16_t internal_error_last_line; ///< Source line number of most recent internal error
    uint32_t internal_errors;       ///< Bitmask of internal error types detected
    uint32_t internal_error_count;  ///< Cumulative count of internal errors (0 ideal)
    uint32_t spi_count;             ///< Number of SPI transactions during period
    uint32_t i2c_count;             ///< Number of I2C transactions during period
    uint32_t i2c_isr_count;         ///< Number of I2C interrupts serviced
    uint32_t extra_loop_us;         ///< Accumulated delay added to maintain loop rate
    uint64_t rtc;                   ///< Real-time clock Unix timestamp (0 if unavailable)
};

/**
 * @struct log_SRTL
 * @brief SRTL message: SmartRTL (Smart Return-To-Launch) path tracking
 * 
 * @details The SRTL message logs SmartRTL system state, which records the vehicle's flight path
 *          and uses it to return along the safest route when RTL is triggered. Unlike standard RTL
 *          that flies directly home, SmartRTL retraces the path already flown, avoiding obstacles
 *          encountered during the outbound flight and providing a known-safe return route.
 *          
 *          SRTL Message Purpose:
 *          - **Path Recording**: Monitor path point accumulation
 *          - **Memory Management**: Track point buffer usage
 *          - **Return Planning**: Verify path availability for return
 *          - **Safety Analysis**: Confirm safe return path exists
 *          - **Debugging**: Diagnose SmartRTL behavior and point pruning
 *          
 *          SmartRTL System Overview:
 *          @code
 *          Normal RTL (Return-To-Launch):
 *              1. Climb to RTL_ALT
 *              2. Fly STRAIGHT line to home
 *              3. Descend and land
 *              Problem: May fly through obstacles or restricted areas
 *          
 *          SmartRTL (Smart Return-To-Launch):
 *              1. Continuously record flight path as points
 *              2. On RTL trigger, retrace path in REVERSE
 *              3. Follow exact path flown on outbound leg
 *              4. Return to launch point
 *              Advantage: Known-safe path, avoids obstacles
 *          @endcode
 *          
 *          How SmartRTL Works:
 *          @code
 *          During Flight (Path Recording):
 *              - Record position at regular intervals (SRTL_ACCURACY)
 *              - Store points in circular buffer (max SRTL_POINTS)
 *              - Simplify path when buffer fills (remove redundant points)
 *              - Maintain path from current position back to launch
 *          
 *          During Return (Path Following):
 *              - Reverse through recorded points
 *              - Fly to each waypoint in sequence
 *              - Remove points as they're reached
 *              - Continue until reaching launch point or home
 *          
 *          Path Simplification (when buffer full):
 *              - Identify near-colinear points
 *              - Remove middle points of straight segments
 *              - Preserve corners and significant deviations
 *              - Keep critical points (launch, current position)
 *          @endcode
 *          
 *          active (SmartRTL Active Status):
 *          Whether SmartRTL could be used right now:
 *          - **0 (false)**: SmartRTL not available/usable
 *          - **1 (true)**: SmartRTL ready, can be used for return
 *          - **Availability**: Requires sufficient path points recorded
 *          - **Minimum**: At least 3-4 points for usable path
 *          
 *          Active Status Conditions:
 *          @code
 *          active = 0 (SmartRTL not available):
 *              - Just armed (no path yet)
 *              - Disabled via SRTL_ENABLE parameter
 *              - GPS not good enough for position recording
 *              - Path buffer empty or insufficient points
 *              - Internal error in path manager
 *          
 *          active = 1 (SmartRTL available):
 *              - Good GPS fix (3D position available)
 *              - Path recorded with adequate points
 *              - Path connects current position to home
 *              - Ready to execute SmartRTL if commanded
 *          @endcode
 *          
 *          num_points (Current Point Count):
 *          Number of path points currently stored:
 *          - **Range**: 0 to max_points
 *          - **Growing**: During outbound flight (recording path)
 *          - **Stable**: When buffer full (old points pruned)
 *          - **Decreasing**: During SmartRTL return (consuming points)
 *          - **Minimum Useful**: 3-4 points minimum for return
 *          
 *          Point Count Interpretation:
 *          @code
 *          num_points = 0
 *              - Just armed, no path recorded yet
 *              - SmartRTL not available
 *          
 *          num_points = 5-20
 *              - Short flight, building path
 *              - SmartRTL usable but limited
 *          
 *          num_points = 50-100
 *              - Moderate flight, good path coverage
 *              - SmartRTL fully functional
 *          
 *          num_points = max_points
 *              - Buffer full, automatic pruning active
 *              - Oldest/redundant points being removed
 *              - Normal for long flights
 *          @endcode
 *          
 *          max_points (Maximum Point Capacity):
 *          Total point buffer capacity:
 *          - **Parameter**: SRTL_POINTS (default 300)
 *          - **Range**: 0-65535 (practical 50-500)
 *          - **Memory**: ~30 bytes per point
 *          - **Trade-off**: More points = longer paths but more memory
 *          
 *          Maximum Points Configuration:
 *          @code
 *          max_points = 100
 *              - Low memory boards (STM32F1)
 *              - Short flights (<5 minutes)
 *              - Limited path detail
 *          
 *          max_points = 300 (default)
 *              - Standard configuration
 *              - 10-15 minute flights typical
 *              - Good balance memory/capability
 *          
 *          max_points = 500+
 *              - High memory boards (STM32F7/H7)
 *              - Long missions (30+ minutes)
 *              - Maximum path detail
 *          @endcode
 *          
 *          action (Last Action Taken):
 *          Most recent internal SmartRTL action:
 *          - **Enum**: AP_SmartRTL::Action
 *          - **Purpose**: Debug path management operations
 *          - **Actions**: POINT_ADD, POINT_PRUNE, PATH_RESET, etc.
 *          
 *          Common Actions:
 *          @code
 *          action = 0 (NONE)
 *              - No recent action
 *              - Idle state
 *          
 *          action = 1 (POINT_ADD)
 *              - New path point added
 *              - Normal during flight
 *          
 *          action = 2 (POINT_PRUNE)
 *              - Point removed (redundant or buffer full)
 *              - Path simplification active
 *          
 *          action = 3 (POINT_REMOVE)
 *              - Point consumed during return
 *              - SmartRTL return in progress
 *          
 *          action = 4 (PATH_RESET)
 *              - Path cleared (disarm/rearm)
 *              - Starting fresh
 *          @endcode
 *          
 *          N (North Position Component):
 *          North coordinate of point associated with action:
 *          - **Units**: Meters
 *          - **Frame**: Relative to EKF origin (usually home)
 *          - **Positive**: North of origin
 *          - **Negative**: South of origin
 *          - **Purpose**: Shows which point was added/removed/reached
 *          
 *          E (East Position Component):
 *          East coordinate of point associated with action:
 *          - **Units**: Meters
 *          - **Frame**: Relative to EKF origin (usually home)
 *          - **Positive**: East of origin
 *          - **Negative**: West of origin
 *          
 *          D (Down Position Component):
 *          Down coordinate of point associated with action:
 *          - **Units**: Meters (NED frame convention)
 *          - **Frame**: Relative to EKF origin altitude
 *          - **Positive**: Below origin (down)
 *          - **Negative**: Above origin (up)
 *          - **Note**: NED convention: positive D = descending
 *          
 *          Position Interpretation:
 *          @code
 *          // Point added at location
 *          SRTL: Active=1, NumPts=45, MaxPts=300, Action=1(ADD), N=120.5, E=85.3, D=15.2
 *                (New point at 120.5m North, 85.3m East, 15.2m below origin)
 *          
 *          // Point pruned during simplification
 *          SRTL: Active=1, NumPts=299, MaxPts=300, Action=2(PRUNE), N=45.2, E=12.8, D=10.5
 *                (Removed redundant point to free buffer space)
 *          
 *          // Point consumed during return
 *          SRTL: Active=1, NumPts=78, MaxPts=300, Action=3(REMOVE), N=95.6, E=67.4, D=12.0
 *                (Reached waypoint, removing from path, continuing return)
 *          @endcode
 *          
 *          SmartRTL Flight Scenarios:
 *          @code
 *          // Takeoff and initial flight - building path
 *          SRTL: Active=0, NumPts=0, MaxPts=300, Action=0, N=0.0, E=0.0, D=0.0
 *                (Just armed, no path yet)
 *          
 *          SRTL: Active=1, NumPts=5, MaxPts=300, Action=1, N=15.2, E=8.5, D=5.0
 *                (Recording path, 5 points stored)
 *          
 *          SRTL: Active=1, NumPts=45, MaxPts=300, Action=1, N=250.8, E=180.3, D=25.5
 *                (Good path coverage, ready for SmartRTL)
 *          
 *          // Long flight - buffer management
 *          SRTL: Active=1, NumPts=300, MaxPts=300, Action=2, N=125.5, E=95.2, D=15.0
 *                (Buffer full, pruning redundant points)
 *          
 *          // SmartRTL triggered - returning
 *          SRTL: Active=1, NumPts=280, MaxPts=300, Action=3, N=320.5, E=210.8, D=30.2
 *                (Following path back, points being consumed)
 *          
 *          SRTL: Active=1, NumPts=12, MaxPts=300, Action=3, N=25.5, E=18.2, D=8.5
 *                (Near home, few points remaining)
 *          
 *          SRTL: Active=0, NumPts=0, MaxPts=300, Action=4, N=0.0, E=0.0, D=0.0
 *                (Completed return, path reset after landing)
 *          @endcode
 *          
 *          SRTL_* Parameters:
 *          - **SRTL_ENABLE**: Enable/disable SmartRTL (0=disabled, 1=enabled)
 *          - **SRTL_POINTS**: Maximum number of path points (50-500)
 *          - **SRTL_ACCURACY**: Minimum distance between points in cm
 *          - **SRTL_TIMEOUT**: Timeout for SmartRTL completion (seconds)
 *          
 *          Path Recording Parameters:
 *          @code
 *          SRTL_ACCURACY = 200 (2 meters)
 *              - Dense path: More points, more detail
 *              - Buffer fills faster
 *              - Better for complex routes
 *          
 *          SRTL_ACCURACY = 500 (5 meters)
 *              - Moderate path: Good balance
 *              - Standard setting
 *          
 *          SRTL_ACCURACY = 1000 (10 meters)
 *              - Sparse path: Fewer points
 *              - Buffer lasts longer
 *              - Suitable for simple routes
 *          @endcode
 *          
 *          When SmartRTL Falls Back to Regular RTL:
 *          - **No Path**: Insufficient points recorded
 *          - **GPS Lost**: Cannot determine position
 *          - **Timeout**: SRTL_TIMEOUT exceeded
 *          - **Error**: Path manager detects problem
 *          - **Disabled**: SRTL_ENABLE = 0
 *          
 *          Advantages of SmartRTL:
 *          - **Obstacle Avoidance**: Retraces known-safe path
 *          - **Regulatory**: Stays within surveyed flight area
 *          - **Safety**: Avoids restricted zones crossed outbound
 *          - **Predictable**: Ground observers know return route
 *          
 *          Limitations:
 *          - **Memory**: Limited path length (points = memory)
 *          - **GPS Required**: Needs good GPS for recording
 *          - **Wind**: Return may differ if wind changed
 *          - **Dynamic Obstacles**: Won't detect new obstacles
 * 
 * @note SmartRTL retraces outbound path in reverse - known safe route
 * @note active=1 required for SmartRTL to be usable
 * @note Buffer automatically prunes redundant points when full
 * @note Falls back to regular RTL if path unavailable
 * 
 * @warning num_points=0 means SmartRTL not available (will use regular RTL)
 * @warning Buffer full (num_points=max_points) triggers automatic pruning
 * @warning SmartRTL requires good GPS throughout flight
 * @warning Path may not account for wind changes on return
 * 
 * @see SRTL_ENABLE to enable SmartRTL feature
 * @see SRTL_POINTS to configure buffer size
 * @see SRTL_ACCURACY for path point spacing
 * @see AP_SmartRTL library for implementation
 */
struct PACKED log_SRTL {
    LOG_PACKET_HEADER;      ///< Standard 3-byte header: head1, head2, msgid
    uint64_t time_us;       ///< Timestamp in microseconds since system boot
    uint8_t active;         ///< SmartRTL availability: 0=not available, 1=ready to use
    uint16_t num_points;    ///< Current number of path points stored (0 to max_points)
    uint16_t max_points;    ///< Maximum point buffer capacity (SRTL_POINTS parameter)
    uint8_t action;         ///< Most recent action: ADD, PRUNE, REMOVE, RESET (see AP_SmartRTL::Action)
    float N;                ///< North position in meters of point associated with action
    float E;                ///< East position in meters of point associated with action
    float D;                ///< Down position in meters of point associated with action (NED frame)
};

/**
 * @struct log_Arm_Disarm
 * @brief ARM message: Arming and disarming status changes
 * 
 * @details The ARM message logs every arming and disarming event, recording the state transition,
 *          pre-arm check results, whether the action was forced, and the method used. This is
 *          critical safety data for understanding why the vehicle armed/disarmed and whether
 *          all safety checks passed before flight.
 *          
 *          ARM Message Purpose:
 *          - **Safety Audit**: Record all arming/disarming events
 *          - **Pre-Flight Validation**: Document which checks passed/failed
 *          - **Failure Analysis**: Understand unexpected disarms or arm failures
 *          - **Compliance**: Maintain record of safety check compliance
 *          - **Troubleshooting**: Diagnose arming problems
 *          
 *          Arming System Overview:
 *          @code
 *          DISARMED state:
 *              - Motors off, no thrust possible
 *              - Safe for handling and maintenance
 *              - Can modify parameters
 *              - Pre-arm checks continuously evaluated
 *          
 *          ARMING transition:
 *              - Pre-arm checks must all pass
 *              - Arming checks evaluated
 *              - Safety switch (if present) must be pressed
 *              - Pilot confirms ready for flight
 *          
 *          ARMED state:
 *              - Motors can spin, vehicle can fly
 *              - Safety-critical - dangerous to approach
 *              - All safety systems active
 *              - Disarm checks monitored
 *          
 *          DISARMING transition:
 *              - Pilot command or automatic (failsafe, landing)
 *              - Motors stop immediately
 *              - Return to safe state
 *          @endcode
 *          
 *          Why Arming Checks Exist:
 *          - **Safety**: Prevent flight with known problems
 *          - **Hardware**: Verify sensors functioning correctly
 *          - **Configuration**: Ensure parameters valid
 *          - **Environment**: Check GPS, compass calibration
 *          - **Battery**: Confirm sufficient power
 *          - **Failsafes**: Verify RC link and telemetry
 *          
 *          arm_state (Arming State):
 *          New arming state after this transition:
 *          - **0**: DISARMED - motors off, safe
 *          - **1**: ARMED - motors can spin, dangerous
 *          - **Log Time**: Logged at moment of transition
 *          
 *          Arming State Transitions:
 *          @code
 *          arm_state = 0 (DISARMED):
 *              - Vehicle just disarmed
 *              - Safe to approach
 *              - Can be:
 *                  * Pilot commanded disarm
 *                  * Auto-disarm after landing
 *                  * Emergency disarm (failsafe)
 *                  * Forced disarm (GCS command)
 *          
 *          arm_state = 1 (ARMED):
 *              - Vehicle just armed
 *              - DANGEROUS - motors can spin
 *              - Ready for flight
 *              - All pre-arm checks passed (unless forced)
 *          @endcode
 *          
 *          arm_checks (Arming Checks Bitmask):
 *          Bitmask of pre-arm/arming checks at time of transition:
 *          - **Type**: uint32_t bitmask
 *          - **Enum**: AP_Arming::Check
 *          - **Each bit**: Specific safety check
 *          - **Interpretation**: Bit set = check FAILED (or bypassed if forced)
 *          
 *          Common Arming Check Bits:
 *          @code
 *          Typical arming check bits (see AP_Arming::Check enum):
 *          - Bit 0: All checks passed (special bit)
 *          - Bit 1: Barometer health check
 *          - Bit 2: Compass health check  
 *          - Bit 3: GPS health check
 *          - Bit 4: INS (inertial sensor) health
 *          - Bit 5: RC channels received
 *          - Bit 6: Board voltage acceptable
 *          - Bit 7: Battery level sufficient
 *          - Bit 8: Safety switch pressed
 *          - Bit 9: Gyro calibrated
 *          - Bit 10: Compass calibrated
 *          - Bit 11: GPS accuracy acceptable
 *          ... (see AP_Arming.h for complete list)
 *          
 *          arm_checks = 0x00000000
 *              - All checks PASSED
 *              - Normal, safe arming
 *          
 *          arm_checks = 0x00000008 (bit 3 set)
 *              - GPS check FAILED
 *              - If armed: forced arm despite GPS issue
 *              - If disarmed: arm attempt rejected
 *          
 *          arm_checks = 0x00000104 (bits 2,8 set)
 *              - Compass and safety switch checks failed
 *              - Multiple issues preventing arming
 *          @endcode
 *          
 *          Arming Check Categories:
 *          - **Sensors**: IMU, compass, GPS, barometer healthy
 *          - **Calibration**: Sensors calibrated, offsets valid
 *          - **Configuration**: Parameters safe, no conflicts
 *          - **Environment**: GPS lock, compass variance acceptable
 *          - **Power**: Battery voltage/capacity sufficient
 *          - **Communication**: RC link, telemetry operational
 *          - **Safety**: Safety switch, geofence configured
 *          
 *          forced (Forced Arming Flag):
 *          Whether arm/disarm was forced (bypassing checks):
 *          - **0 (false)**: Normal arming, all checks passed
 *          - **1 (true)**: Forced, bypassed some/all checks
 *          - **Risk**: Forced arming dangerous - checks exist for safety
 *          - **Use Cases**: Emergency, testing, expert override
 *          
 *          Forced Arming Scenarios:
 *          @code
 *          forced = 0 (Normal Arming):
 *              - All pre-arm checks passed
 *              - Safe to fly
 *              - arm_checks = 0x00000000
 *              - Recommended mode
 *          
 *          forced = 1 (Forced Arming):
 *              - One or more checks failed
 *              - Pilot/GCS override safety
 *              - arm_checks shows which checks failed
 *              - DANGEROUS - fly at own risk
 *          
 *          When to force arm (expert only):
 *              - Testing in controlled environment
 *              - Known false positive from check
 *              - Emergency recovery situation
 *              - Indoor flight (GPS expected to fail)
 *          @endcode
 *          
 *          method (Arming Method):
 *          How the arm/disarm was initiated:
 *          - **Enum**: AP_Arming::Method
 *          - **Values**: See AP_Arming::Method enum definition
 *          
 *          Common Arming Methods:
 *          @code
 *          method = 0 (UNKNOWN):
 *              - Method not determined
 *              - Should not occur in normal operation
 *          
 *          method = 1 (RUDDER):
 *              - Traditional stick arming (rudder right + throttle down)
 *              - Copter: Yaw right, throttle down for 2 seconds
 *              - Plane: Rudder right for 2 seconds
 *              - Most common method
 *          
 *          method = 2 (MAVLINK):
 *              - Armed via MAVLink command from GCS
 *              - COMMAND_LONG with MAV_CMD_COMPONENT_ARM_DISARM
 *              - Can include force flag
 *          
 *          method = 3 (SWITCH):
 *              - RC switch assigned to arm/disarm
 *              - AUX function: ARM/DISARM
 *              - Convenient but risk of accidental trigger
 *          
 *          method = 4 (AUTO):
 *              - Automatic arming (if enabled)
 *              - Auto-arm on throttle up (Copter)
 *              - Auto-arm in AUTO mode (Plane)
 *          
 *          method = 5 (MOTORDETECTDONE):
 *              - Arming after motor direction detection test
 *              - BLHeli/ESC configuration feature
 *          
 *          method = 6 (SCRIPTING):
 *              - Armed via Lua scripting command
 *              - Custom flight logic
 *          @endcode
 *          
 *          Typical Arming Sequences:
 *          @code
 *          // Normal arming via RC sticks
 *          ARM: ArmState=1, ArmChecks=0x00000000, Forced=0, Method=1
 *               (Armed successfully, all checks passed, rudder stick method)
 *          
 *          // Forced arming with GPS failure
 *          ARM: ArmState=1, ArmChecks=0x00000008, Forced=1, Method=2
 *               (Armed via MAVLink, GPS check failed but forced, DANGEROUS)
 *          
 *          // Normal disarm after landing
 *          ARM: ArmState=0, ArmChecks=0x00000000, Forced=0, Method=4
 *               (Auto-disarmed after detecting landing)
 *          
 *          // Failed arming attempt (would show in events/errors, not ARM message)
 *          // ARM message only logged on successful state change
 *          
 *          // Multiple checks failed, forced arm
 *          ARM: ArmState=1, ArmChecks=0x00000124, Forced=1, Method=2
 *               (Forced via GCS, compass+GPS+barometer checks failed - VERY DANGEROUS)
 *          @endcode
 *          
 *          ARMING_* Parameters:
 *          - **ARMING_CHECK**: Bitmask of which checks to enforce (default: all)
 *          - **ARMING_ACCTHRESH**: Accelerometer threshold for calibration check
 *          - **ARMING_VOLT_MIN**: Minimum battery voltage for arming
 *          - **ARMING_VOLT2_MIN**: Minimum second battery voltage
 *          - **ARMING_RUDDER**: Enable/disable rudder stick arming
 *          - **ARMING_MIS_ITEMS**: Require mission uploaded before arming
 *          - **ARMING_REQUIRE**: Require specific safety features
 *          
 *          Disabling Arming Checks:
 *          @code
 *          ARMING_CHECK parameter bits:
 *          - Bit 0: All checks (set to 0 to disable all - DANGEROUS)
 *          - Bit 1: Barometer
 *          - Bit 2: Compass
 *          - Bit 3: GPS
 *          - Bit 4: INS (inertial sensors)
 *          - Bit 5: Parameters
 *          - Bit 6: RC channels
 *          - Bit 7: Board voltage
 *          - Bit 8: Battery
 *          ... etc.
 *          
 *          ARMING_CHECK = 1 (default)
 *              - All checks enabled (safest)
 *          
 *          ARMING_CHECK = 0
 *              - All checks disabled (VERY DANGEROUS)
 *              - Only for testing in controlled environment
 *          @endcode
 *          
 *          Safety Implications:
 *          - **forced=1**: Flying with known issues - high risk
 *          - **arm_checks≠0**: Some checks failed when forced
 *          - **Normal Operation**: forced=0, arm_checks=0x00000000
 *          - **Accident Investigation**: ARM messages crucial evidence
 *          
 *          Troubleshooting Arming Failures:
 *          - **Check arm_checks bitmask**: Identify which checks failed
 *          - **Review Pre-Arm Messages**: Text messages explain failures
 *          - **ERR Messages**: May indicate sensor issues
 *          - **Calibration**: Many failures due to uncalibrated sensors
 *          - **GPS**: Common issue - need 3D fix with good HDOP
 *          - **Compass**: Calibration and interference problems
 * 
 * @note ARM messages logged only on successful arming state transitions
 * @note arm_checks=0 means all pre-arm checks passed (normal)
 * @note forced=1 indicates dangerous override of safety checks
 * @note Check AP_Arming::Check enum for arm_checks bit definitions
 * 
 * @warning forced=1 means safety checks bypassed - fly at own risk
 * @warning arm_checks≠0 shows which checks failed when forced
 * @warning Never force arm in normal operations - checks exist for safety
 * @warning ARMING_CHECK=0 disables all safety checks - extremely dangerous
 * 
 * @see ARMING_CHECK parameter to enable/disable specific checks
 * @see AP_Arming::Check enum for arm_checks bitmask definition
 * @see AP_Arming::Method enum for method values
 * @see ERR and EV messages for arming failure details
 */
struct PACKED log_Arm_Disarm {
    LOG_PACKET_HEADER;      ///< Standard 3-byte header: head1, head2, msgid
    uint64_t time_us;       ///< Timestamp in microseconds since system boot
    uint8_t  arm_state;     ///< New arming state: 0=DISARMED, 1=ARMED
    uint32_t arm_checks;    ///< Bitmask of arming checks (see AP_Arming::Check)
    uint8_t forced;         ///< Forced flag: 0=normal, 1=forced (checks bypassed)
    uint8_t method;         ///< Arming method used (see AP_Arming::Method enum)
};

/**
 * @struct log_Winch
 * @brief WINC message: Winch control and status for payload delivery/retrieval
 * 
 * @details The WINC message logs winch system state for vehicles equipped with motorized winches
 *          for payload delivery, cargo hook operations, or tethered flight. The winch allows
 *          controlled deployment and retrieval of a cable/line, enabling operations like package
 *          delivery, search and rescue, sensor deployment, or tethered power applications.
 *          
 *          WINC Message Purpose:
 *          - **Payload Operations**: Monitor delivery/retrieval missions
 *          - **Line Management**: Track cable length and tension
 *          - **Safety**: Detect end-of-line, overload, motor issues
 *          - **Performance**: Verify winch responding to commands
 *          - **Maintenance**: Monitor motor temperature and health
 *          
 *          Winch System Overview:
 *          @code
 *          Winch Components:
 *              - Motor: DC or servo motor for line control
 *              - Spool: Cable/line storage drum
 *              - Clutch: Engage/disengage motor from spool
 *              - Line: Cable/rope (typically 10-100m)
 *              - Encoder: Measure line length deployed
 *              - Tension Sensor: Detect load on line
 *              - End Switch: Detect fully retracted position
 *          
 *          Typical Applications:
 *              - Package Delivery: Lower package to ground from hover
 *              - Cargo Hook: Lift/transport suspended loads
 *              - Sensor Deployment: Lower instruments to desired depth
 *              - Search and Rescue: Deliver supplies or retrieve items
 *              - Tethered Flight: Power/data cable management
 *          @endcode
 *          
 *          Winch Control Modes:
 *          @code
 *          Mode 0: RELAXED
 *              - Motor disengaged or passive
 *              - Line can free-spool
 *              - No active control
 *          
 *          Mode 1: POSITION_CONTROL
 *              - Motor controls line to desired_length
 *              - Holds position against load
 *              - Most common for delivery operations
 *          
 *          Mode 2: RATE_CONTROL
 *              - Motor controls deployment rate
 *              - Line extends/retracts at desired_rate
 *              - Used for smooth deployment
 *          @endcode
 *          
 *          healthy (Winch Health Status):
 *          Overall winch system health:
 *          - **0 (false)**: Winch unhealthy, problem detected
 *          - **1 (true)**: Winch functioning normally
 *          - **Checks**: Motor response, encoder, temperature, communication
 *          
 *          Health Check Failures:
 *          @code
 *          healthy = 0 (unhealthy) when:
 *              - Motor not responding to commands
 *              - Encoder not updating (broken sensor)
 *              - Over-temperature condition
 *              - Communication timeout with winch controller
 *              - Mechanical jam detected
 *              - Power supply issue
 *          
 *          healthy = 1 (healthy):
 *              - All systems functioning
 *              - Motor responding correctly
 *              - Sensors reading valid data
 *              - Safe to operate
 *          @endcode
 *          
 *          thread_end (End-of-Thread Detected):
 *          Whether line is fully retracted (at home position):
 *          - **0 (false)**: Line extended, not at end
 *          - **1 (true)**: Line fully retracted, end switch triggered
 *          - **Safety**: Prevents over-retraction
 *          - **Homing**: Used to zero length measurement
 *          
 *          Thread End Detection:
 *          @code
 *          thread_end = 0 (line extended):
 *              - Line deployed some distance
 *              - Normal operating state
 *              - Can continue retracting
 *          
 *          thread_end = 1 (at end):
 *              - Line fully retracted
 *              - End switch triggered
 *              - Motor stops retracting
 *              - Length should be zero (or near zero)
 *              - Ready for next deployment
 *          @endcode
 *          
 *          moving (Motor Movement Status):
 *          Whether motor is currently active:
 *          - **0 (false)**: Motor stopped, line stationary
 *          - **1 (true)**: Motor running, line extending/retracting
 *          - **Feedback**: Confirms motor responding to commands
 *          
 *          Moving Status:
 *          @code
 *          moving = 0 (stopped):
 *              - Motor not powered
 *              - Line stationary
 *              - Holding position or relaxed
 *          
 *          moving = 1 (active):
 *              - Motor powered
 *              - Line extending or retracting
 *              - Actively adjusting length
 *          @endcode
 *          
 *          clutch (Clutch Engagement Status):
 *          Whether clutch is engaged:
 *          - **0 (false)**: Clutch engaged, motor coupled to spool
 *          - **1 (true)**: Clutch disengaged, spool can free-wheel
 *          - **Free-Spool**: Allows rapid deployment under load
 *          - **Safety**: Disengaged for manual override
 *          
 *          Clutch Operation:
 *          @code
 *          clutch = 0 (engaged):
 *              - Motor controls spool directly
 *              - Position/rate control active
 *              - Normal operating mode
 *              - Cannot manually pull line
 *          
 *          clutch = 1 (disengaged):
 *              - Motor disconnected from spool
 *              - Line can free-spool
 *              - Useful for emergency release
 *              - Manual deployment possible
 *              - Used in cargo hook drop
 *          @endcode
 *          
 *          mode (Control Mode):
 *          Current winch operating mode:
 *          - **0**: RELAXED - No active control
 *          - **1**: POSITION_CONTROL - Maintain desired length
 *          - **2**: RATE_CONTROL - Control deployment rate
 *          
 *          Mode Detailed Description:
 *          @code
 *          mode = 0 (RELAXED):
 *              - Motor off or minimal holding
 *              - No position/rate control
 *              - May free-spool if clutch disengaged
 *              - Idle state
 *          
 *          mode = 1 (POSITION_CONTROL):
 *              - Target: desired_length
 *              - Motor adjusts to reach target
 *              - Holds position against tension
 *              - Most common for hovering delivery
 *              - Feedback from encoder
 *          
 *          mode = 2 (RATE_CONTROL):
 *              - Target: desired_rate (m/s)
 *              - Motor maintains deployment speed
 *              - Smooth controlled deployment
 *              - Used during lowering operations
 *          @endcode
 *          
 *          desired_length (Target Line Length):
 *          Commanded line length to deploy:
 *          - **Units**: Meters
 *          - **Range**: 0 to max_length (typically 10-100m)
 *          - **Mode**: Used in POSITION_CONTROL mode
 *          - **Zero**: Fully retracted position
 *          
 *          Desired Length Examples:
 *          @code
 *          desired_length = 0.0 m
 *              - Command: Fully retract
 *              - Target: Line at home position
 *          
 *          desired_length = 10.0 m
 *              - Command: Deploy 10 meters
 *              - Typical package delivery height
 *          
 *          desired_length = 50.0 m
 *              - Long deployment for deep operations
 *              - Sensor lowering, tethered operations
 *          @endcode
 *          
 *          length (Actual Line Length):
 *          Measured line length currently deployed:
 *          - **Units**: Meters
 *          - **Range**: 0 to max_length
 *          - **Source**: Encoder measurement
 *          - **Error**: length - desired_length shows control error
 *          
 *          Length Interpretation:
 *          @code
 *          length ≈ desired_length
 *              - Winch tracking target correctly
 *              - Position control working
 *          
 *          length < desired_length
 *              - Winch extending line
 *              - Motor deploying
 *          
 *          length > desired_length
 *              - Winch retracting line
 *              - Motor retrieving
 *          
 *          length = 0.0
 *              - Fully retracted
 *              - thread_end should be 1
 *          @endcode
 *          
 *          desired_rate (Target Deployment Rate):
 *          Commanded line deployment speed:
 *          - **Units**: Meters per second (m/s)
 *          - **Positive**: Deploying (extending line)
 *          - **Negative**: Retrieving (retracting line)
 *          - **Zero**: Hold position
 *          - **Mode**: Used in RATE_CONTROL mode
 *          
 *          Typical Deployment Rates:
 *          @code
 *          desired_rate = 0.0 m/s
 *              - Hold current position
 *              - No movement
 *          
 *          desired_rate = +0.5 m/s
 *              - Slow deployment
 *              - Gentle lowering
 *              - 30 meters per minute
 *          
 *          desired_rate = +1.0 m/s
 *              - Moderate deployment
 *              - Typical operation
 *          
 *          desired_rate = -0.5 m/s
 *              - Slow retraction
 *              - Careful retrieval
 *          
 *          desired_rate = -2.0 m/s
 *              - Fast retraction
 *              - Emergency retrieval
 *          @endcode
 *          
 *          tension (Line Tension/Load):
 *          Load detected on winch line:
 *          - **Units**: Application-specific (Newtons or arbitrary units)
 *          - **Range**: 0-65535
 *          - **Purpose**: Detect payload, snag, overload
 *          - **Safety**: Prevent motor overload
 *          
 *          Tension Monitoring:
 *          @code
 *          tension = 0
 *              - No load on line
 *              - Line slack or payload released
 *          
 *          tension = 100-500 (typical payload)
 *              - Normal load detected
 *              - Package attached
 *              - Safe operating range
 *          
 *          tension = 1000+ (high load)
 *              - Heavy load or snag
 *              - Check for obstruction
 *              - May trigger overload protection
 *          
 *          tension dropped suddenly
 *              - Payload released/detached
 *              - Line broke (emergency!)
 *              - Hook opened
 *          @endcode
 *          
 *          voltage (Motor Voltage):
 *          Voltage supplied to winch motor:
 *          - **Units**: Volts (V)
 *          - **Range**: 0-30V typical
 *          - **Indicates**: Motor power, battery health
 *          - **Performance**: Higher voltage = more torque
 *          
 *          Voltage Interpretation:
 *          @code
 *          voltage = 12.6 V (3S LiPo, full)
 *              - Fresh battery
 *              - Maximum motor performance
 *          
 *          voltage = 11.1 V (3S LiPo, nominal)
 *              - Normal operating voltage
 *              - Good motor performance
 *          
 *          voltage = 9.0 V (3S LiPo, low)
 *              - Low battery
 *              - Reduced motor torque
 *              - Should land soon
 *          @endcode
 *          
 *          temp (Motor Temperature):
 *          Motor temperature in degrees Celsius:
 *          - **Units**: Degrees Celsius (°C)
 *          - **Range**: -128 to +127 °C (int8_t)
 *          - **Safety**: Over-temperature protection
 *          - **Typical**: 20-60°C during operation
 *          
 *          Temperature Monitoring:
 *          @code
 *          temp = 25°C
 *              - Ambient temperature
 *              - Motor cool, just started
 *          
 *          temp = 40-60°C
 *              - Warm during operation
 *              - Normal working temperature
 *          
 *          temp = 70-80°C
 *              - Hot, approaching limits
 *              - Reduce duty cycle
 *              - May need cooling period
 *          
 *          temp = 90°C+
 *              - Critical over-temperature
 *              - Motor may shut down
 *              - Risk of damage
 *          @endcode
 *          
 *          Typical Winch Operations:
 *          @code
 *          // Package delivery - descend and deploy
 *          WINC: Heal=1, ThEnd=1, Mov=0, Clut=0, Mode=1, DLen=0.0, Len=0.0, DRate=0.0, Tens=0, Vcc=12.4, Temp=28
 *                (Starting position: healthy, at end, stopped, engaged, position mode, retracted)
 *          
 *          WINC: Heal=1, ThEnd=0, Mov=1, Clut=0, Mode=1, DLen=10.0, Len=3.5, DRate=0.0, Tens=250, Vcc=12.2, Temp=35
 *                (Deploying: line extending, 3.5m deployed of 10m target, payload tensioned)
 *          
 *          WINC: Heal=1, ThEnd=0, Mov=0, Clut=0, Mode=1, DLen=10.0, Len=10.0, DRate=0.0, Tens=280, Vcc=12.3, Temp=42
 *                (At target: holding 10m, payload supported)
 *          
 *          WINC: Heal=1, ThEnd=0, Mov=0, Clut=0, Mode=1, DLen=10.0, Len=10.0, DRate=0.0, Tens=0, Vcc=12.3, Temp=38
 *                (Release: tension dropped to zero, package released)
 *          
 *          WINC: Heal=1, ThEnd=0, Mov=1, Clut=0, Mode=1, DLen=0.0, Len=5.2, DRate=0.0, Tens=0, Vcc=12.1, Temp=45
 *                (Retracting: returning to home, 5.2m still deployed)
 *          
 *          WINC: Heal=1, ThEnd=1, Mov=0, Clut=0, Mode=0, DLen=0.0, Len=0.0, DRate=0.0, Tens=0, Vcc=12.4, Temp=32
 *                (Complete: back at home, relaxed mode, cooling down)
 *          @endcode
 *          
 *          WINCH_* Parameters:
 *          - **WINCH_ENABLE**: Enable winch feature
 *          - **WINCH_TYPE**: Winch hardware type
 *          - **WINCH_RATE_MAX**: Maximum deployment rate
 *          - **WINCH_POS_P**: Position control P gain
 *          - **WINCH_MAX_LEN**: Maximum line length
 *          
 *          Safety Considerations:
 *          - **Line Snag**: High tension with no movement
 *          - **Overload**: Excessive tension may damage motor
 *          - **Temperature**: Monitor temp to prevent burnout
 *          - **End Detection**: Prevents over-retraction
 *          - **Clutch Release**: Emergency free-spool capability
 * 
 * @note Winch operations require careful monitoring of tension and length
 * @note thread_end=1 indicates fully retracted position
 * @note tension=0 may indicate payload release or line break
 * @note Temperature monitoring critical for motor longevity
 * 
 * @warning healthy=0 indicates winch malfunction - abort operations
 * @warning High temperature (>80°C) may damage motor
 * @warning Sudden tension loss may indicate line break or payload drop
 * @warning clutch=1 allows free-spool - use with caution
 * 
 * @see WINCH_* parameters for winch configuration
 * @see AP_Winch library for winch control
 * @see Package delivery and cargo hook flight modes
 */
struct PACKED log_Winch {
    LOG_PACKET_HEADER;      ///< Standard 3-byte header: head1, head2, msgid
    uint64_t time_us;       ///< Timestamp in microseconds since system boot
    uint8_t healthy;        ///< Winch health status: 0=unhealthy, 1=healthy
    uint8_t thread_end;     ///< End-of-line detection: 0=extended, 1=fully retracted
    uint8_t moving;         ///< Motor movement: 0=stopped, 1=moving
    uint8_t clutch;         ///< Clutch status: 0=engaged, 1=disengaged (free-spool)
    uint8_t mode;           ///< Control mode: 0=RELAXED, 1=POSITION, 2=RATE
    float desired_length;   ///< Target line length in meters
    float length;           ///< Actual line length in meters (encoder measured)
    float desired_rate;     ///< Target deployment rate in m/s (+deploy, -retract)
    uint16_t tension;       ///< Line tension/load (units depend on sensor)
    float voltage;          ///< Motor supply voltage in volts
    int8_t temp;            ///< Motor temperature in degrees Celsius
};

/**
 * @struct log_STAK
 * @brief STAK message: Thread stack usage monitoring for RTOS platforms
 * 
 * @details The STAK message logs stack memory usage for each thread/task running on the
 *          autopilot's Real-Time Operating System (RTOS). Stack monitoring is critical for
 *          detecting stack overflow conditions that can cause crashes, memory corruption,
 *          and unpredictable behavior. This message is logged periodically on RTOS platforms
 *          (ChibiOS/ARM) to monitor system health.
 *          
 *          STAK Message Purpose:
 *          - **Memory Safety**: Detect stack overflow before crashes occur
 *          - **System Health**: Monitor if threads have adequate stack space
 *          - **Development**: Optimize stack sizes during development
 *          - **Debugging**: Identify threads consuming excessive stack
 *          - **Performance**: Ensure efficient memory allocation
 *          
 *          Stack Memory Concepts:
 *          @code
 *          What is a Stack?
 *              - Memory area for each thread's local variables
 *              - Function call chains consume stack
 *              - Deep recursion or large local arrays use more stack
 *              - Stack overflow = catastrophic failure
 *          
 *          Stack Allocation:
 *              - Each thread allocated fixed stack size at creation
 *              - Cannot grow dynamically (embedded systems)
 *              - Must size conservatively for worst-case
 *              - Too small = overflow, too large = wasted RAM
 *          
 *          Stack Growth:
 *              - Stack grows downward (high address to low)
 *              - stack_total = allocated size
 *              - stack_free = unused space remaining
 *              - stack_used = stack_total - stack_free
 *          
 *          Critical Stack Situations:
 *              - stack_free < 200 bytes: DANGEROUS
 *              - stack_free < 100 bytes: CRITICAL
 *              - stack_free = 0: OVERFLOW IMMINENT
 *          @endcode
 *          
 *          RTOS Thread Architecture:
 *          @code
 *          Typical ArduPilot Threads (ChibiOS):
 *              - Main thread: Primary control loop
 *              - IO thread: Sensor I/O, actuator output
 *              - Timer thread: Periodic scheduler callbacks
 *              - Storage thread: Flash/SD card operations
 *              - UART threads: Serial communication
 *              - Network threads: WiFi/Ethernet handling
 *              - Scripting thread: Lua script execution
 *          
 *          Each thread has:
 *              - Unique thread_id
 *              - Priority level (higher = more CPU time)
 *              - Allocated stack space
 *              - Name for identification
 *          @endcode
 *          
 *          thread_id (Thread Identifier):
 *          Numeric identifier for the thread:
 *          - **Type**: uint8_t
 *          - **Range**: 0-255
 *          - **Unique**: Each thread has different ID
 *          - **Persistent**: ID consistent across boots
 *          - **Platform**: RTOS assigns IDs
 *          
 *          Thread ID Assignment:
 *          @code
 *          Common Thread IDs (platform-dependent):
 *              thread_id = 0: Idle thread
 *              thread_id = 1: Main control thread
 *              thread_id = 2: IO thread
 *              thread_id = 3: Timer thread
 *              thread_id = 4: Storage thread
 *              ... (varies by platform and configuration)
 *          
 *          Use thread name for human identification
 *          @endcode
 *          
 *          priority (Thread Priority):
 *          Thread scheduling priority:
 *          - **Type**: uint8_t
 *          - **Range**: 0-255 (platform-dependent)
 *          - **Higher Number**: More CPU priority (typically)
 *          - **Scheduler**: RTOS gives CPU to highest priority ready thread
 *          - **Real-Time**: Ensures critical tasks run on time
 *          
 *          Priority Levels:
 *          @code
 *          priority = 0-50 (Low Priority):
 *              - Background tasks
 *              - Logging, storage
 *              - Non-critical processing
 *          
 *          priority = 51-100 (Normal Priority):
 *              - Most application threads
 *              - Sensor processing
 *              - Communication handlers
 *          
 *          priority = 101-180 (High Priority):
 *              - Time-critical control loops
 *              - Main flight control thread
 *              - Fast sensor sampling
 *          
 *          priority = 181-255 (Critical Priority):
 *              - Interrupt service threads
 *              - Safety-critical operations
 *              - Should execute immediately
 *          @endcode
 *          
 *          stack_total (Total Stack Size):
 *          Total allocated stack size for this thread:
 *          - **Units**: Bytes
 *          - **Type**: uint16_t
 *          - **Range**: 0-65535 bytes
 *          - **Fixed**: Set at thread creation
 *          - **Cannot Grow**: Embedded systems limitation
 *          
 *          Typical Stack Sizes:
 *          @code
 *          stack_total = 512 bytes
 *              - Very small stack
 *              - Simple threads only
 *              - No deep function calls
 *          
 *          stack_total = 1024-2048 bytes
 *              - Small to medium threads
 *              - Typical sensor handlers
 *              - Most application threads
 *          
 *          stack_total = 4096-8192 bytes
 *              - Large threads
 *              - Complex processing
 *              - Main control loops
 *          
 *          stack_total = 16384+ bytes
 *              - Very large threads
 *              - Scripting engines
 *              - Network stacks
 *          @endcode
 *          
 *          stack_free (Free Stack Space):
 *          Unused stack space remaining:
 *          - **Units**: Bytes
 *          - **Type**: uint16_t
 *          - **Range**: 0 to stack_total
 *          - **Critical**: Low values indicate danger
 *          - **Measurement**: High-water mark (minimum seen)
 *          
 *          Stack Free Interpretation:
 *          @code
 *          stack_free = stack_total (100% free)
 *              - Thread never ran or very shallow stack use
 *              - Stack size probably oversized
 *          
 *          stack_free > 1024 bytes (healthy)
 *              - Plenty of safety margin
 *              - Normal operation
 *              - Stack size appropriate
 *          
 *          stack_free = 200-500 bytes (caution)
 *              - Limited margin
 *              - Monitor closely
 *              - Consider increasing stack size
 *          
 *          stack_free < 200 bytes (WARNING)
 *              - Very low margin
 *              - Risk of overflow
 *              - Increase stack size immediately
 *          
 *          stack_free < 100 bytes (CRITICAL)
 *              - Imminent overflow danger
 *              - System may crash at any time
 *              - Emergency action required
 *          
 *          stack_free = 0 (OVERFLOW)
 *              - Stack exhausted
 *              - Overflow likely occurring
 *              - Memory corruption probable
 *              - System instability/crashes
 *          @endcode
 *          
 *          Stack Usage Calculation:
 *          @code
 *          stack_used = stack_total - stack_free
 *          
 *          Examples:
 *              stack_total=4096, stack_free=3200
 *                  → stack_used = 896 bytes (22% usage, healthy)
 *          
 *              stack_total=2048, stack_free=150
 *                  → stack_used = 1898 bytes (93% usage, DANGER)
 *          
 *              stack_total=8192, stack_free=7800
 *                  → stack_used = 392 bytes (5% usage, oversized)
 *          @endcode
 *          
 *          name (Thread Name):
 *          Human-readable thread identifier:
 *          - **Type**: char[16] (null-terminated string)
 *          - **Length**: Up to 15 characters + null
 *          - **Purpose**: Identify thread function
 *          - **Helpful**: Easier than numeric thread_id
 *          
 *          Common Thread Names:
 *          @code
 *          name = "main"
 *              - Primary flight control thread
 *              - Highest priority typically
 *              - Runs main loop at 400Hz (Copter)
 *          
 *          name = "IO"
 *              - Input/Output handling thread
 *              - Sensor reading, actuator output
 *              - High priority
 *          
 *          name = "timer"
 *              - Scheduler callback processing
 *              - Periodic tasks
 *              - Medium-high priority
 *          
 *          name = "storage"
 *              - Flash/SD card operations
 *              - Logging data writes
 *              - Lower priority (background)
 *          
 *          name = "uart1", "uart2", etc.
 *              - Serial port communication
 *              - MAVLink, GPS, telemetry
 *              - Medium priority
 *          
 *          name = "scripting"
 *              - Lua script execution
 *              - Large stack typically
 *              - Lower priority
 *          
 *          name = "idle"
 *              - RTOS idle task
 *              - Runs when nothing else ready
 *              - Lowest priority
 *          @endcode
 *          
 *          Typical Stack Monitoring Logs:
 *          @code
 *          // Healthy system - all threads have adequate stack
 *          STAK: Id=1, Pri=180, Total=8192, Free=6840, Name="main"
 *                (Main thread: 83% free, very healthy)
 *          
 *          STAK: Id=2, Pri=150, Total=4096, Free=2450, Name="IO"
 *                (IO thread: 60% free, healthy)
 *          
 *          STAK: Id=5, Pri=60, Total=2048, Free=1620, Name="storage"
 *                (Storage: 79% free, healthy)
 *          
 *          // Problem detected - scripting thread low on stack
 *          STAK: Id=8, Pri=50, Total=8192, Free=180, Name="scripting"
 *                (WARNING: Only 180 bytes free! Script using too much stack)
 *          
 *          // Critical - uart thread nearly out of stack
 *          STAK: Id=6, Pri=100, Total=2048, Free=45, Name="uart1"
 *                (CRITICAL: Only 45 bytes free! Overflow imminent!)
 *          @endcode
 *          
 *          Stack Overflow Symptoms:
 *          - **Random Crashes**: Unpredictable system resets
 *          - **Memory Corruption**: Variables change unexpectedly
 *          - **Erratic Behavior**: Strange sensor readings, control glitches
 *          - **Watchdog Resets**: System fails to respond
 *          - **Hard Faults**: ARM processor exceptions
 *          
 *          Debugging Stack Issues:
 *          @code
 *          1. Identify Problem Thread:
 *              - Find STAK messages with stack_free < 200
 *              - Note thread name and ID
 *          
 *          2. Analyze Thread Function:
 *              - What does this thread do?
 *              - Are there large local arrays?
 *              - Deep function call chains?
 *              - Recursive algorithms?
 *          
 *          3. Solutions:
 *              - Increase stack size for thread (recompile)
 *              - Reduce local variable sizes
 *              - Move large buffers to heap
 *              - Simplify function call chains
 *              - Disable features using excessive stack
 *          
 *          4. Scripting Stack Issues:
 *              - Lua scripts can use deep recursion
 *              - Complex scripts need more stack
 *              - Simplify script or increase SCRIPTING_HEAP_SIZE
 *          @endcode
 *          
 *          Stack Size Configuration:
 *          - **Source**: Defined in HAL board configuration
 *          - **Location**: libraries/AP_HAL_ChibiOS/hwdef/ files
 *          - **Modification**: Requires firmware recompilation
 *          - **Trade-off**: More stack = less RAM for other uses
 *          
 *          Platform Differences:
 *          @code
 *          ChibiOS (ARM boards):
 *              - Full RTOS with multiple threads
 *              - STAK messages logged for all threads
 *              - Critical for stability monitoring
 *          
 *          Linux HAL:
 *              - Different threading model
 *              - May not log STAK messages
 *              - Linux manages stacks differently
 *          
 *          SITL (simulation):
 *              - Host OS manages stacks
 *              - Stack monitoring less critical
 *              - Development environment
 *          @endcode
 *          
 *          Stack Monitoring Best Practices:
 *          - **Review Logs**: Check STAK messages after flights
 *          - **Safety Margin**: Keep stack_free > 25% of stack_total
 *          - **Test Thoroughly**: Exercise all features to find worst-case
 *          - **Monitor Trends**: Decreasing stack_free over time = problem
 *          - **Development**: Enable stack checking during feature development
 * 
 * @note STAK messages logged periodically on RTOS platforms (ChibiOS)
 * @note stack_free is "high-water mark" - lowest value seen, not current
 * @note Stack overflow causes memory corruption and crashes
 * @note stack_free < 200 bytes is dangerous - increase stack size
 * 
 * @warning stack_free < 100 bytes indicates imminent overflow
 * @warning Stack overflow can cause catastrophic system failure
 * @warning Symptoms: random crashes, memory corruption, erratic behavior
 * @warning Cannot be fixed at runtime - requires firmware recompilation
 * 
 * @see ChibiOS RTOS for thread management
 * @see HAL board hwdef files for stack size configuration
 * @see AP_HAL::Scheduler for thread creation
 */
struct PACKED log_STAK {
    LOG_PACKET_HEADER;      ///< Standard 3-byte header: head1, head2, msgid
    uint64_t time_us;       ///< Timestamp in microseconds since system boot
    uint8_t thread_id;      ///< Numeric thread identifier (platform-assigned)
    uint8_t priority;       ///< Thread scheduling priority (higher=more priority)
    uint16_t stack_total;   ///< Total allocated stack size in bytes
    uint16_t stack_free;    ///< Minimum free stack space seen (high-water mark) in bytes
    char name[16];          ///< Human-readable thread name (null-terminated)
};

/**
 * @struct log_File
 * @brief FILE message: File data for embedding files in binary logs
 * 
 * @details The FILE message allows arbitrary files to be embedded directly in the binary log.
 *          This is used to capture configuration files, mission files, fence definitions,
 *          parameter files, or other critical data that should be preserved with the flight
 *          log for complete record-keeping and post-flight analysis.
 *          
 *          FILE Message Purpose:
 *          - **Configuration Capture**: Embed params, missions, fences in log
 *          - **Post-Flight Analysis**: Ensure all settings available for review
 *          - **Accident Investigation**: Complete record of vehicle configuration
 *          - **Compliance**: Meet record-keeping requirements
 *          - **Reproducibility**: Recreate exact flight configuration
 *          
 *          File Embedding Mechanism:
 *          @code
 *          How Files Are Logged:
 *              1. File read from filesystem (SD card, flash)
 *              2. Split into 64-byte chunks
 *              3. Each chunk logged as separate FILE message
 *              4. offset indicates position in original file
 *              5. Sequential FILE messages reconstruct complete file
 *          
 *          Example: 150-byte file embedded:
 *              FILE: filename="mission.txt", offset=0, length=64, data=[bytes 0-63]
 *              FILE: filename="mission.txt", offset=64, length=64, data=[bytes 64-127]
 *              FILE: filename="mission.txt", offset=128, length=22, data=[bytes 128-149]
 *              (3 messages to capture 150 bytes)
 *          @endcode
 *          
 *          Common Files Embedded in Logs:
 *          - **Parameters**: Complete parameter set at flight time
 *          - **Missions**: Waypoint mission loaded in vehicle
 *          - **Fence**: Geofence boundary definitions
 *          - **Rally Points**: Rally point locations
 *          - **Scripts**: Lua scripts that were running
 *          - **Configuration**: Various vehicle config files
 *          
 *          filename (Filename):
 *          Name of the file being embedded:
 *          - **Type**: char[16] (null-terminated string)
 *          - **Length**: Up to 15 characters + null
 *          - **Path**: May include simple path (e.g., "@PARAM/params.parm")
 *          - **Purpose**: Identify file when extracting from log
 *          
 *          Common Filenames:
 *          @code
 *          filename = "@PARAM/defaults.parm"
 *              - Default parameter values
 *              - Factory settings
 *          
 *          filename = "@SYS/vehicle.parm"
 *              - Current parameter values
 *              - As configured for this flight
 *          
 *          filename = "mission.txt"
 *              - Waypoint mission file
 *              - AUTO mode flight plan
 *          
 *          filename = "fence.txt"
 *              - Geofence boundary definition
 *              - Polygon points
 *          
 *          filename = "rally.txt"
 *              - Rally point coordinates
 *              - Safe return locations
 *          
 *          filename = "script.lua"
 *              - Lua script that ran during flight
 *              - Custom vehicle behavior
 *          @endcode
 *          
 *          Special Filename Prefixes:
 *          @code
 *          @PARAM/ prefix
 *              - Parameter-related files
 *              - System-generated parameter dumps
 *          
 *          @SYS/ prefix
 *              - System configuration files
 *              - Vehicle setup data
 *          
 *          @ROMFS/ prefix
 *              - Files from read-only filesystem
 *              - Embedded in firmware
 *          
 *          No prefix
 *              - User files from SD card or flash
 *              - Missions, fences, scripts
 *          @endcode
 *          
 *          offset (File Offset):
 *          Byte position in original file:
 *          - **Type**: uint32_t
 *          - **Units**: Bytes from start of file
 *          - **Range**: 0 to file_size-1
 *          - **Purpose**: Reconstruct file by ordering chunks
 *          - **First Chunk**: offset = 0
 *          
 *          Offset Usage:
 *          @code
 *          offset = 0
 *              - First chunk of file
 *              - Beginning of data
 *          
 *          offset = 64
 *              - Second chunk (assuming 64-byte chunks)
 *              - Bytes 64-127 of original
 *          
 *          offset = 128
 *              - Third chunk
 *              - Bytes 128-191 of original
 *          
 *          Sequential offsets allow reconstruction:
 *              - Sort FILE messages by filename and offset
 *              - Concatenate data[] fields
 *              - Result: complete original file
 *          @endcode
 *          
 *          length (Data Length):
 *          Number of valid bytes in data[] field:
 *          - **Type**: uint8_t
 *          - **Units**: Bytes
 *          - **Range**: 0-64
 *          - **Typical**: 64 for full chunks
 *          - **Last Chunk**: Often < 64 bytes
 *          
 *          Length Interpretation:
 *          @code
 *          length = 64
 *              - Full chunk
 *              - All 64 bytes of data[] valid
 *              - More chunks likely follow
 *          
 *          length < 64
 *              - Partial chunk
 *              - Usually last chunk of file
 *              - Only first 'length' bytes of data[] valid
 *          
 *          length = 0
 *              - Empty file or marker
 *              - No data in this message
 *              - Unusual
 *          
 *          Example file sizes:
 *              63-byte file: 1 message, length=63
 *              64-byte file: 1 message, length=64
 *              65-byte file: 2 messages, length=64 and length=1
 *              200-byte file: 4 messages, length=64,64,64,8
 *          @endcode
 *          
 *          data (File Data Chunk):
 *          Raw file data for this chunk:
 *          - **Type**: char[64] (raw bytes, not necessarily text)
 *          - **Size**: Always 64 bytes allocated
 *          - **Valid Data**: Only first 'length' bytes are valid
 *          - **Binary Safe**: Can contain any byte values
 *          - **Not Null-Terminated**: Binary data, not strings
 *          
 *          Data Field Usage:
 *          @code
 *          Text Files:
 *              - Contains ASCII/UTF-8 text
 *              - May include newlines, control chars
 *              - Not necessarily null-terminated
 *          
 *          Binary Files:
 *              - Raw binary data
 *              - May contain 0x00 bytes
 *              - Any byte values valid
 *          
 *          Parameter Files:
 *              - Plain text format
 *              - "PARAM_NAME,value\n" lines
 *              - Human-readable
 *          
 *          Mission Files:
 *              - Waypoint format
 *              - One waypoint per line
 *              - Text-based
 *          @endcode
 *          
 *          Extracting Files from Logs:
 *          @code
 *          Algorithm to Extract Embedded File:
 *          
 *          1. Find all FILE messages for same filename:
 *              messages = find_all_FILE_messages(filename)
 *          
 *          2. Sort by offset:
 *              messages.sort(by=offset)
 *          
 *          3. Concatenate data chunks:
 *              output_file = open(filename, "wb")
 *              for msg in messages:
 *                  output_file.write(msg.data[0:msg.length])
 *              output_file.close()
 *          
 *          4. Verify integrity:
 *              - Check offsets sequential (0, 64, 128, ...)
 *              - Verify no gaps or duplicates
 *              - Check final file size matches expected
 *          @endcode
 *          
 *          Mission Planner Integration:
 *          - Mission Planner can extract embedded files from logs
 *          - "Log Browse" → "Extract Parameters" gets @PARAM files
 *          - Allows parameter comparison between flights
 *          - Mission/fence extraction for review
 *          
 *          Typical FILE Message Sequences:
 *          @code
 *          // Small parameter file (80 bytes)
 *          FILE: filename="@SYS/params.parm", offset=0, length=64, data=[PARAM1,10\nPARAM2,...]
 *          FILE: filename="@SYS/params.parm", offset=64, length=16, data=[...\nPARAM5,25\n]
 *          
 *          // Mission file embedded
 *          FILE: filename="mission.txt", offset=0, length=64, data=[QGC WPL 110\n0 1 0 ...]
 *          FILE: filename="mission.txt", offset=64, length=64, data=[... waypoint data ...]
 *          FILE: filename="mission.txt", offset=128, length=42, data=[... last waypoint\n]
 *          
 *          // Lua script
 *          FILE: filename="script.lua", offset=0, length=64, data=[-- Navigation script\nfunction update()...]
 *          FILE: filename="script.lua", offset=64, length=64, data=[... lua code ...]
 *          @endcode
 *          
 *          File Embedding Timing:
 *          - **Boot Time**: Default parameters, vehicle config
 *          - **Arming**: Current parameters, missions, fences
 *          - **On Demand**: When files modified or requested
 *          - **Scripting**: When scripts loaded/reloaded
 *          
 *          Use Cases:
 *          @code
 *          Accident Investigation:
 *              - Extract parameters to see exact config
 *              - Review mission to understand intended flight
 *              - Check fence boundaries that were active
 *              - Verify script behavior
 *          
 *          Post-Flight Analysis:
 *              - Compare parameters between flights
 *              - Verify correct mission loaded
 *              - Check if parameters changed during flight
 *              - Audit configuration compliance
 *          
 *          Bug Reporting:
 *              - Developers request "send your log"
 *              - Log contains complete configuration
 *              - No separate parameter file needed
 *              - Reproducible test case
 *          
 *          Competition/Compliance:
 *              - Prove vehicle configuration
 *              - Document mission plan
 *              - Show fence boundaries
 *              - Meet record-keeping requirements
 *          @endcode
 *          
 *          Limitations:
 *          - **64-byte chunks**: Large files create many messages
 *          - **16-char filename**: Long names truncated
 *          - **Log space**: Embedding large files uses logging bandwidth
 *          - **No compression**: Files stored uncompressed
 *          - **No checksum**: Verify integrity manually
 *          
 *          Storage Considerations:
 *          - Small files (parameters): Negligible impact
 *          - Mission files: Typically < 1 KB, minimal impact
 *          - Scripts: Can be large (10+ KB), more significant
 *          - Selective embedding: Only critical files included
 * 
 * @note FILE messages allow complete configuration capture in logs
 * @note Multiple FILE messages with same filename form complete file
 * @note Sort by offset and concatenate data[] to reconstruct file
 * @note Only first 'length' bytes of data[] are valid
 * 
 * @warning data[] contains binary data - not necessarily null-terminated
 * @warning Large embedded files consume log bandwidth
 * @warning Verify offset sequence when extracting (check for gaps)
 * @warning Filename limited to 15 characters (including path prefix)
 * 
 * @see Mission Planner log extraction tools
 * @see AP_Filesystem for file operations
 * @see Parameter, mission, fence file formats
 */
struct PACKED log_File {
    LOG_PACKET_HEADER;      ///< Standard 3-byte header: head1, head2, msgid
    char filename[16];      ///< Filename (null-terminated, may include path prefix like "@PARAM/")
    uint32_t offset;        ///< Byte offset of this chunk in the original file
    uint8_t length;         ///< Number of valid bytes in data[] field (0-64)
    char data[64];          ///< Raw file data chunk (binary safe, not null-terminated)
};

/**
 * @struct log_Scripting
 * @brief SCR message: Lua scripting runtime statistics and performance
 * 
 * @details The SCR message logs performance and memory usage statistics for Lua scripts
 *          running on the autopilot. ArduPilot's scripting engine allows users to add
 *          custom behavior via Lua scripts for missions, navigation, peripheral control,
 *          and advanced features. This message helps monitor script performance and
 *          diagnose issues like excessive CPU usage or memory leaks.
 *          
 *          SCR Message Purpose:
 *          - **Performance Monitoring**: Track script execution time
 *          - **Memory Management**: Detect memory leaks in scripts
 *          - **CPU Budget**: Ensure scripts not consuming excessive time
 *          - **Debugging**: Identify problematic scripts
 *          - **Optimization**: Find scripts needing performance improvement
 *          
 *          ArduPilot Scripting Overview:
 *          @code
 *          Lua Scripting System:
 *              - Language: Lua 5.3
 *              - Purpose: User-extensible custom behavior
 *              - Thread: Runs in dedicated scripting thread
 *              - Safety: Sandboxed, limited API access
 *              - Scheduler: Cooperative multitasking
 *          
 *          Script Execution Model:
 *              - Each script has update() function
 *              - Scheduler calls update() periodically
 *              - Scripts yield control after each update
 *              - Run time measured per update cycle
 *              - Memory tracked per script
 *          
 *          Common Script Applications:
 *              - Custom flight modes
 *              - Advanced mission logic
 *              - Peripheral device control
 *              - Data logging extensions
 *              - Automation and triggers
 *              - Failsafe customization
 *              - Sensor fusion algorithms
 *          @endcode
 *          
 *          Scripting Architecture:
 *          @code
 *          How Scripts Execute:
 *          
 *          1. Script Loading:
 *              - Scripts in scripts/ directory on SD card
 *              - Loaded at boot or when added
 *              - Compiled to Lua bytecode
 *              - Each script gets Lua VM instance
 *          
 *          2. Scheduler Loop:
 *              while (scripting_enabled) {
 *                  for each script {
 *                      start_time = now()
 *                      script.update()
 *                      run_time = now() - start_time
 *                      log_SCR(script.name, run_time, memory)
 *                  }
 *                  yield()
 *              }
 *          
 *          3. Time Budget:
 *              - Scripts limited to milliseconds per update
 *              - Exceed budget → script suspended
 *              - Prevents one script from starving system
 *          @endcode
 *          
 *          name (Script Name):
 *          Filename of the script being monitored:
 *          - **Type**: char[16] (null-terminated string)
 *          - **Length**: Up to 15 characters + null
 *          - **Format**: Filename without path (e.g., "navigation.lua")
 *          - **Purpose**: Identify which script these stats belong to
 *          
 *          Script Naming:
 *          @code
 *          name = "navigation.lua"
 *              - Custom navigation script
 *              - User-written mission logic
 *          
 *          name = "mount_ctrl.lua"
 *              - Gimbal/camera mount control
 *              - Custom pointing behavior
 *          
 *          name = "auto_land.lua"
 *              - Automated landing sequence
 *              - Custom approach logic
 *          
 *          name = "telemetry.lua"
 *              - Custom telemetry logging
 *              - Data export to external systems
 *          
 *          name = "lidar_avoid.lua"
 *              - Obstacle avoidance using lidar
 *              - Custom avoidance algorithm
 *          
 *          Truncated if > 15 chars:
 *              "very_long_script_name.lua" → "very_long_scri"
 *          @endcode
 *          
 *          run_time (Execution Time):
 *          Time taken for most recent update() call:
 *          - **Type**: uint32_t
 *          - **Units**: Microseconds (μs)
 *          - **Range**: 0 to 4,294,967,295 μs (~4295 seconds max)
 *          - **Typical**: 100-5000 μs (0.1-5 ms)
 *          - **Per Update**: Time for one update() execution
 *          
 *          Run Time Interpretation:
 *          @code
 *          run_time = 0-100 μs (< 0.1 ms)
 *              - Very fast script
 *              - Minimal processing
 *              - Excellent performance
 *              - Example: Simple status check
 *          
 *          run_time = 100-1000 μs (0.1-1 ms)
 *              - Fast script
 *              - Typical simple script
 *              - Good performance
 *              - Example: Basic control logic
 *          
 *          run_time = 1000-5000 μs (1-5 ms)
 *              - Moderate script
 *              - More complex processing
 *              - Acceptable if not every loop
 *              - Example: Sensor processing, math
 *          
 *          run_time = 5000-10000 μs (5-10 ms)
 *              - Slow script
 *              - Heavy processing
 *              - May impact system performance
 *              - Example: Complex algorithm, many API calls
 *          
 *          run_time > 10000 μs (> 10 ms)
 *              - Very slow script
 *              - Excessive CPU usage
 *              - PERFORMANCE PROBLEM
 *              - Needs optimization
 *          @endcode
 *          
 *          Script Time Budget:
 *          @code
 *          Typical Time Limits:
 *              - Per update: 1-5 ms recommended
 *              - Per scheduler cycle: 10-20 ms max for all scripts
 *              - Exceeded budget: Script may be suspended
 *          
 *          Main Loop Impact:
 *              - Main loop: 400 Hz (2.5 ms period) on Copter
 *              - Scripting runs in separate thread
 *              - But shares CPU with main loop
 *              - Heavy scripts reduce main loop rate
 *          
 *          Performance Guidelines:
 *              - Keep update() < 1 ms when possible
 *              - Spread work across multiple updates
 *              - Don't do everything every update
 *              - Use run_time to identify slow scripts
 *          @endcode
 *          
 *          total_mem (Total Memory Usage):
 *          Total Lua heap memory used by all scripts:
 *          - **Type**: int32_t (signed)
 *          - **Units**: Bytes
 *          - **Range**: -2,147,483,648 to +2,147,483,647
 *          - **Typical**: 10,000-200,000 bytes (10-200 KB)
 *          - **Global**: Same value for all script SCR messages
 *          
 *          Total Memory Interpretation:
 *          @code
 *          total_mem = 10,000-50,000 (10-50 KB)
 *              - Low memory usage
 *              - Simple scripts
 *              - Plenty of room for more
 *          
 *          total_mem = 50,000-100,000 (50-100 KB)
 *              - Moderate memory usage
 *              - Several scripts or complex ones
 *              - Healthy range
 *          
 *          total_mem = 100,000-200,000 (100-200 KB)
 *              - High memory usage
 *              - Many scripts or memory-intensive
 *              - Approaching limits
 *              - Monitor for leaks
 *          
 *          total_mem > 200,000 (> 200 KB)
 *              - Very high memory usage
 *              - May exceed configured heap
 *              - Check for memory leaks
 *              - Reduce script complexity
 *          
 *          total_mem increasing over time
 *              - MEMORY LEAK detected
 *              - Script not releasing resources
 *              - Will eventually fail
 *              - Debug script for leaked objects
 *          @endcode
 *          
 *          run_mem (Script-Specific Memory):
 *          Memory used by this specific script:
 *          - **Type**: int32_t (signed)
 *          - **Units**: Bytes
 *          - **Range**: -2,147,483,648 to +2,147,483,647
 *          - **Typical**: 1,000-50,000 bytes (1-50 KB)
 *          - **Per Script**: Varies by script complexity
 *          
 *          Script Memory Interpretation:
 *          @code
 *          run_mem = 1,000-5,000 (1-5 KB)
 *              - Small script
 *              - Minimal data structures
 *              - Efficient memory use
 *          
 *          run_mem = 5,000-20,000 (5-20 KB)
 *              - Medium script
 *              - Typical complexity
 *              - Normal memory usage
 *          
 *          run_mem = 20,000-50,000 (20-50 KB)
 *              - Large script
 *              - Complex data structures
 *              - Arrays, tables, state
 *          
 *          run_mem > 50,000 (> 50 KB)
 *              - Very large script
 *              - Heavy memory use
 *              - Consider optimization
 *          
 *          run_mem increasing each update
 *              - Memory leak in this script
 *              - Allocating without freeing
 *              - Will exhaust heap
 *          @endcode
 *          
 *          Memory Leak Detection:
 *          @code
 *          Normal Memory Pattern:
 *              SCR: Name="nav.lua", Runtime=800, Total_mem=45000, Run_mem=12000
 *              SCR: Name="nav.lua", Runtime=820, Total_mem=45000, Run_mem=12000
 *              SCR: Name="nav.lua", Runtime=790, Total_mem=45000, Run_mem=12000
 *              (Memory stable → healthy)
 *          
 *          Memory Leak Pattern:
 *              SCR: Name="bad.lua", Runtime=1200, Total_mem=50000, Run_mem=15000
 *              SCR: Name="bad.lua", Runtime=1220, Total_mem=52000, Run_mem=17000
 *              SCR: Name="bad.lua", Runtime=1180, Total_mem=54000, Run_mem=19000
 *              SCR: Name="bad.lua", Runtime=1210, Total_mem=56000, Run_mem=21000
 *              (Memory growing → LEAK!)
 *          
 *          Leak Causes:
 *              - Creating tables without garbage collection
 *              - Accumulating data in global variables
 *              - Circular references preventing GC
 *              - Not releasing large objects
 *          @endcode
 *          
 *          Typical SCR Message Sequences:
 *          @code
 *          // Multiple scripts running
 *          SCR: Name="navigation.lua", Runtime=650, Total_mem=48000, Run_mem=12000
 *          SCR: Name="mount.lua", Runtime=320, Total_mem=48000, Run_mem=8000
 *          SCR: Name="telemetry.lua", Runtime=1200, Total_mem=48000, Run_mem=18000
 *          SCR: Name="copter_gcs.lua", Runtime=480, Total_mem=48000, Run_mem=10000
 *          (total_mem same for all = total heap usage)
 *          
 *          // Performance problem - slow script
 *          SCR: Name="heavy_math.lua", Runtime=15000, Total_mem=65000, Run_mem=35000
 *          (Runtime 15ms = too slow! Optimize or split work)
 *          
 *          // Memory leak detected
 *          SCR: Name="leaky.lua", Runtime=800, Total_mem=52000, Run_mem=15000  [T=0s]
 *          SCR: Name="leaky.lua", Runtime=820, Total_mem=55000, Run_mem=18000  [T=1s]
 *          SCR: Name="leaky.lua", Runtime=810, Total_mem=58000, Run_mem=21000  [T=2s]
 *          (Memory growing 3KB/sec = leak)
 *          @endcode
 *          
 *          SCR_* Parameters:
 *          - **SCR_ENABLE**: Enable scripting (0=disabled, 1=enabled)
 *          - **SCR_HEAP_SIZE**: Lua heap size in KB (default 43 KB)
 *          - **SCR_VM_I_COUNT**: Instruction count limit per script per timeslice
 *          - **SCR_DEBUG_OPTS**: Debug options bitmask
 *          
 *          Scripting Configuration:
 *          @code
 *          SCR_ENABLE = 0
 *              - Scripting disabled
 *              - No scripts run
 *              - SCR messages not logged
 *              - Saves CPU and memory
 *          
 *          SCR_ENABLE = 1
 *              - Scripting enabled
 *              - Scripts run from scripts/ folder
 *              - SCR messages logged
 *          
 *          SCR_HEAP_SIZE = 43 (default)
 *              - 43 KB heap for all scripts
 *              - total_mem cannot exceed this
 *              - Adequate for most use cases
 *          
 *          SCR_HEAP_SIZE = 100
 *              - 100 KB heap
 *              - More room for complex scripts
 *              - Uses more RAM
 *          @endcode
 *          
 *          Performance Optimization Tips:
 *          @code
 *          Reduce run_time:
 *              - Cache results instead of recalculating
 *              - Don't run every update (use counters)
 *              - Minimize AP library API calls
 *              - Use local variables (faster than globals)
 *              - Avoid table creation in loops
 *          
 *          Reduce run_mem:
 *              - Reuse tables instead of creating new
 *              - Clear tables when done (set to nil)
 *              - Use smaller data types
 *              - Don't accumulate historical data
 *              - Trigger Lua garbage collection
 *          
 *          Example optimization:
 *              -- BAD (creates table every update)
 *              function update()
 *                  local data = {1, 2, 3, 4, 5}
 *                  process(data)
 *              end
 *          
 *              -- GOOD (reuse table)
 *              local data = {1, 2, 3, 4, 5}
 *              function update()
 *                  process(data)
 *              end
 *          @endcode
 *          
 *          Debugging Scripts:
 *          - Review SCR messages for high run_time
 *          - Plot total_mem and run_mem vs. time
 *          - Look for memory growth trends
 *          - Compare runtime across different scripts
 *          - Test scripts individually to isolate issues
 * 
 * @note SCR messages logged for each script every time it runs
 * @note total_mem is global (all scripts), run_mem is per-script
 * @note run_time is microseconds for one update() call
 * @note Memory leak shows as increasing total_mem/run_mem over time
 * 
 * @warning run_time > 10 ms indicates performance problem
 * @warning Increasing total_mem indicates memory leak
 * @warning Scripts can impact main loop performance if too slow
 * @warning total_mem exceeding SCR_HEAP_SIZE causes script failure
 * 
 * @see SCR_* parameters for scripting configuration
 * @see AP_Scripting library for Lua scripting engine
 * @see ArduPilot Lua scripting documentation
 * @see scripts/ directory for script files
 */
struct PACKED log_Scripting {
    LOG_PACKET_HEADER;      ///< Standard 3-byte header: head1, head2, msgid
    uint64_t time_us;       ///< Timestamp in microseconds since system boot
    char name[16];          ///< Script filename (null-terminated, e.g., "navigation.lua")
    uint32_t run_time;      ///< Execution time for this update() in microseconds
    int32_t total_mem;      ///< Total Lua heap memory used by all scripts in bytes
    int32_t run_mem;        ///< Memory used by this specific script in bytes
};

/**
 * @struct log_MotBatt
 * @brief MOTB message: Motor mixer and battery compensation information
 * 
 * @details The MOTB message logs multicopter motor mixer outputs and battery voltage
 *          compensation. As battery voltage drops during flight, motor efficiency decreases.
 *          The motor mixer compensates by increasing throttle outputs to maintain the same
 *          thrust. This message tracks those compensation factors and throttle limits,
 *          critical for understanding motor performance and battery health effects.
 *          
 *          MOTB Message Purpose:
 *          - **Battery Compensation**: Track voltage-based motor gain
 *          - **Throttle Limiting**: Monitor when hitting maximum throttle
 *          - **Performance Analysis**: Understand motor mixer saturation
 *          - **Battery Health**: Detect voltage sag under load
 *          - **Failure Detection**: Identify motor failures affecting control
 *          
 *          Multicopter Motor Mixing Concept:
 *          @code
 *          Motor Mixer Role:
 *              Desired Control → Motor Mixer → Individual Motor Outputs
 *              
 *              Inputs:
 *                  - Throttle (collective thrust)
 *                  - Roll command (differential thrust left/right)
 *                  - Pitch command (differential thrust front/back)
 *                  - Yaw command (differential motor speeds)
 *              
 *              Output:
 *                  - Individual PWM commands to each motor
 *                  - Constrained to [PWM_MIN, PWM_MAX]
 *                  - Scaled by battery voltage compensation
 *              
 *              Battery Compensation:
 *                  - Lower voltage → less motor thrust at same PWM
 *                  - Mixer increases PWM to maintain thrust
 *                  - Lift_max factor applied to scale outputs
 *          @endcode
 *          
 *          Battery Voltage Impact on Motors:
 *          @code
 *          Battery Voltage vs. Motor Performance:
 *          
 *          Full Battery (4.2V/cell × 4S = 16.8V):
 *              - Motors produce maximum thrust
 *              - Efficient operation
 *              - Low current draw for given thrust
 *              - No compensation needed (lift_max ≈ 1.0)
 *          
 *          Mid Battery (3.7V/cell × 4S = 14.8V):
 *              - Motors less powerful
 *              - Need higher throttle for same thrust
 *              - Moderate compensation (lift_max ≈ 1.1-1.2)
 *          
 *          Low Battery (3.5V/cell × 4S = 14.0V):
 *              - Motors significantly less powerful
 *              - Much higher throttle needed
 *              - Heavy compensation (lift_max ≈ 1.3-1.5)
 *              - May hit throttle limits
 *          
 *          Critical Battery (3.3V/cell × 4S = 13.2V):
 *              - Motors very weak
 *              - Maximum throttle insufficient
 *              - lift_max > 1.5, but can't deliver
 *              - Loss of control authority
 *              - LAND IMMEDIATELY
 *          @endcode
 *          
 *          lift_max (Maximum Lift Compensation):
 *          Motor gain factor for battery voltage compensation:
 *          - **Type**: float (dimensionless ratio)
 *          - **Units**: None (multiplier/gain)
 *          - **Range**: 0.5 to 2.0 typical
 *          - **Nominal**: 1.0 (full battery, no compensation)
 *          - **Increasing**: Battery voltage dropping
 *          
 *          Lift_max Interpretation:
 *          @code
 *          lift_max = 1.0
 *              - Full battery voltage
 *              - No compensation needed
 *              - Motors operating at rated efficiency
 *              - Ideal performance
 *          
 *          lift_max = 1.0 - 1.2
 *              - Battery voltage slightly low
 *              - Minimal compensation applied
 *              - Still good performance
 *              - Normal operating range
 *          
 *          lift_max = 1.2 - 1.4
 *              - Battery voltage moderately low
 *              - Significant compensation needed
 *              - Performance degrading
 *              - Consider landing soon
 *          
 *          lift_max = 1.4 - 1.6
 *              - Battery voltage very low
 *              - Heavy compensation required
 *              - Reduced control authority
 *              - LAND SOON - approaching limits
 *          
 *          lift_max > 1.6
 *              - Battery critically low
 *              - Maximum compensation
 *              - Throttle saturation likely
 *              - CRITICAL - land immediately
 *          @endcode
 *          
 *          Calculation of lift_max:
 *          @code
 *          lift_max Derivation:
 *          
 *          Basic relationship:
 *              Thrust ∝ Voltage² (approximately)
 *              
 *          To maintain constant thrust as voltage drops:
 *              lift_max = (V_nominal / V_current)²
 *          
 *          Example (4S LiPo):
 *              V_nominal = 14.8V (3.7V/cell)
 *              V_current = 13.2V (3.3V/cell, near empty)
 *              
 *              lift_max = (14.8 / 13.2)² = 1.26
 *              
 *              Means: Need 26% more throttle to get same thrust
 *          
 *          Mixer applies lift_max:
 *              motor_output = desired_thrust × lift_max
 *          @endcode
 *          
 *          bat_volt (Battery Voltage Ratio):
 *          Ratio of current voltage to maximum voltage:
 *          - **Type**: float (dimensionless ratio)
 *          - **Units**: None (ratio 0-1)
 *          - **Range**: 0.0 to 1.0
 *          - **Full Battery**: 1.0
 *          - **Empty Battery**: 0.0
 *          
 *          Battery Voltage Ratio:
 *          @code
 *          bat_volt = 1.0
 *              - Battery fully charged
 *              - Maximum voltage detected
 *              - No sag under load
 *          
 *          bat_volt = 0.8 - 1.0
 *              - Battery healthy
 *              - Good capacity remaining
 *              - Normal operation
 *          
 *          bat_volt = 0.6 - 0.8
 *              - Battery mid capacity
 *              - Voltage dropping
 *              - Monitor closely
 *          
 *          bat_volt = 0.4 - 0.6
 *              - Battery low
 *              - Significant voltage sag
 *              - Plan to land
 *          
 *          bat_volt < 0.4
 *              - Battery critical
 *              - Severe voltage sag
 *              - LAND IMMEDIATELY
 *          
 *          Relationship to lift_max:
 *              - As bat_volt decreases, lift_max increases
 *              - Inverse relationship
 *              - bat_volt low + lift_max high = battery problem
 *          @endcode
 *          
 *          th_limit (Throttle Limit):
 *          Maximum throttle available after current/voltage limiting:
 *          - **Type**: float (normalized 0-1)
 *          - **Units**: None (0 = no throttle, 1 = full throttle)
 *          - **Range**: 0.0 to 1.0
 *          - **Typical**: 1.0 (no limiting)
 *          - **Limited**: < 1.0 (current or voltage protection active)
 *          
 *          Throttle Limit Interpretation:
 *          @code
 *          th_limit = 1.0
 *              - No throttle limiting
 *              - Battery current/voltage OK
 *              - Full throttle available
 *              - Normal operation
 *          
 *          th_limit = 0.8 - 1.0
 *              - Slight throttle limit
 *              - Approaching current limits
 *              - Minor restriction
 *              - Still flyable
 *          
 *          th_limit = 0.6 - 0.8
 *              - Moderate throttle limit
 *              - Current limit active
 *              - Reduced performance
 *              - Battery struggling
 *          
 *          th_limit = 0.4 - 0.6
 *              - Severe throttle limit
 *              - Heavy current limiting
 *              - Significantly reduced thrust
 *              - LAND SOON
 *          
 *          th_limit < 0.4
 *              - Critical throttle limit
 *              - Battery cannot deliver current
 *              - Altitude loss likely
 *              - EMERGENCY LAND
 *          
 *          Causes of Throttle Limiting:
 *              - Excessive current draw (BATT_I_MAX)
 *              - Battery voltage sag (BATT_WATT_MAX)
 *              - Internal resistance too high
 *              - Battery cell imbalance
 *              - Battery end of life
 *          @endcode
 *          
 *          th_average_max (Average Maximum Throttle):
 *          Maximum average throttle that can maintain attitude control:
 *          - **Type**: float (normalized 0-1)
 *          - **Units**: None (0 = no throttle, 1 = full throttle)
 *          - **Range**: 0.0 to 1.0
 *          - **Purpose**: Headroom for attitude control
 *          - **Derived From**: MOT_THR_MIX_* parameters
 *          
 *          Average Maximum Throttle:
 *          @code
 *          th_average_max Concept:
 *              - Motors need throttle margin for control
 *              - If average at 100%, no room to increase some motors
 *              - th_average_max enforces headroom
 *              - Allows differential thrust for attitude
 *          
 *          th_average_max = 1.0
 *              - No restriction on average throttle
 *              - Can use full motor range
 *              - May lose control authority at high throttle
 *          
 *          th_average_max = 0.8
 *              - Average throttle limited to 80%
 *              - Leaves 20% margin for attitude control
 *              - Individual motors can go 80%±20%
 *              - Good control authority maintained
 *          
 *          th_average_max = 0.6
 *              - Average limited to 60%
 *              - Large margin for control
 *              - Reduced maximum climb rate
 *              - Excellent attitude control
 *          
 *          Example:
 *              th_average_max = 0.8 (80% max average)
 *              
 *              If commanding 80% collective:
 *                  - Some motors: 80% + 20% roll/pitch = 100%
 *                  - Other motors: 80% - 20% roll/pitch = 60%
 *                  - All within 0-100% range
 *                  - Attitude control maintained
 *              
 *              If th_average_max = 1.0 and commanding 90%:
 *                  - Some motors: 90% + 20% = 110% (SATURATED!)
 *                  - Saturation limits attitude control
 *                  - Loss of roll/pitch authority
 *          @endcode
 *          
 *          th_out (Actual Throttle Output):
 *          Current average throttle being output to motors:
 *          - **Type**: float (normalized 0-1)
 *          - **Units**: None (0 = minimum, 1 = maximum)
 *          - **Range**: 0.0 to 1.0
 *          - **Typical Hover**: 0.4-0.6
 *          - **Climb**: > 0.6
 *          - **Descent**: < 0.4
 *          
 *          Throttle Output Analysis:
 *          @code
 *          th_out = 0.0
 *              - Motors at minimum
 *              - Vehicle descending rapidly or on ground
 *              - Disarmed
 *          
 *          th_out = 0.4 - 0.6
 *              - Typical hover throttle
 *              - Balanced flight
 *              - Good battery/weight
 *          
 *          th_out = 0.6 - 0.8
 *              - Climbing or heavy load
 *              - More power needed
 *              - Or: battery voltage low (higher throttle for same thrust)
 *          
 *          th_out = 0.8 - 1.0
 *              - Near maximum throttle
 *              - Aggressive climb or heavy vehicle
 *              - Or: battery very weak
 *              - Limited control authority
 *          
 *          th_out = 1.0
 *              - Maximum throttle
 *              - Cannot climb more
 *              - May be descending despite full throttle
 *              - CRITICAL situation
 *          
 *          Comparing th_out to th_limit:
 *              If th_out == th_limit < 1.0:
 *                  - Hitting throttle limit
 *                  - Current limiting active
 *                  - Cannot provide more thrust
 *                  - Altitude loss possible
 *          @endcode
 *          
 *          mot_fail_flags (Motor Failure Flags):
 *          Bitmask indicating motor mixer status and failures:
 *          - **Type**: uint8_t (8-bit bitmask)
 *          - **Bit 0**: Motor failed
 *          - **Bit 1**: Motors balanced (should be set in normal flight)
 *          - **Other bits**: Reserved for future use
 *          
 *          Motor Failure Flags:
 *          @code
 *          mot_fail_flags bit definitions:
 *          
 *          Bit 0 (0x01): Motor Failure Detected
 *              - One or more motors not responding
 *              - ESC failure or motor damage
 *              - CRITICAL - vehicle may crash
 *          
 *          Bit 1 (0x02): Motors Balanced
 *              - Mixer producing balanced outputs
 *              - Normal flight operation
 *              - SHOULD BE SET (value = 2) in flight
 *          
 *          mot_fail_flags = 0 (0b00000000)
 *              - No failure
 *              - Motors NOT balanced (unusual)
 *              - Mixer saturated or on ground
 *          
 *          mot_fail_flags = 2 (0b00000010)
 *              - No failure
 *              - Motors balanced
 *              - NORMAL FLIGHT (expected value)
 *          
 *          mot_fail_flags = 1 (0b00000001)
 *              - Motor failure detected
 *              - Motors not balanced
 *              - EMERGENCY
 *          
 *          mot_fail_flags = 3 (0b00000011)
 *              - Motor failure AND balanced (contradictory?)
 *              - Check vehicle logs
 *          @endcode
 *          
 *          Typical MOTB Message Sequences:
 *          @code
 *          // Normal hover - fresh battery
 *          MOTB: LiftMax=1.00, BatVolt=0.95, ThLimit=1.00, ThrAvMx=0.80, ThrOut=0.50, FailFlags=2
 *          (Good: low lift_max, high bat_volt, no limiting, hover ~50%, balanced)
 *          
 *          // Normal hover - battery getting low
 *          MOTB: LiftMax=1.25, BatVolt=0.70, ThLimit=1.00, ThrAvMx=0.80, ThrOut=0.60, FailFlags=2
 *          (Battery low: higher lift_max, lower bat_volt, higher hover throttle needed)
 *          
 *          // Aggressive climb - fresh battery
 *          MOTB: LiftMax=1.05, BatVolt=0.90, ThLimit=1.00, ThrAvMx=0.80, ThrOut=0.75, FailFlags=2
 *          (Climbing: high throttle but battery OK)
 *          
 *          // Battery critical - current limiting active
 *          MOTB: LiftMax=1.50, BatVolt=0.50, ThLimit=0.70, ThrAvMx=0.80, ThrOut=0.70, FailFlags=2
 *          (CRITICAL: high lift_max, low bat_volt, hitting throttle limit)
 *          
 *          // Motor failure detected
 *          MOTB: LiftMax=1.10, BatVolt=0.85, ThLimit=1.00, ThrAvMx=0.80, ThrOut=0.65, FailFlags=1
 *          (FAILURE: mot_fail_flags shows motor problem)
 *          @endcode
 *          
 *          Diagnosing Issues with MOTB:
 *          @code
 *          Problem: Vehicle losing altitude
 *              Check: lift_max > 1.4, th_out approaching 1.0
 *              Cause: Battery cannot deliver enough power
 *              Action: Land immediately
 *          
 *          Problem: Poor attitude control at high throttle
 *              Check: th_out near th_average_max
 *              Cause: No throttle headroom for control
 *              Action: Reduce aggressive climb, tune MOT_THR_MIX
 *          
 *          Problem: Current spikes, throttle limiting
 *              Check: th_limit < 1.0 during high throttle
 *              Cause: BATT_I_MAX too low, or battery weak
 *              Action: Check BATT_I_MAX parameter, battery health
 *          
 *          Problem: High hover throttle
 *              Check: th_out during hover > 0.7
 *              Cause: Vehicle overweight, battery low, or props damaged
 *              Action: Check vehicle weight, battery, propellers
 *          @endcode
 *          
 *          Related Parameters:
 *          - **MOT_THST_EXPO**: Thrust curve linearization
 *          - **MOT_SPIN_MIN**: Minimum throttle for motor spin
 *          - **MOT_SPIN_MAX**: Maximum throttle
 *          - **MOT_BAT_VOLT_MAX**: Maximum battery voltage (for compensation calculation)
 *          - **MOT_BAT_VOLT_MIN**: Minimum battery voltage
 *          - **MOT_THR_MIX_MAN**: Throttle vs. attitude priority in manual modes
 *          - **MOT_THR_MIX_MAX**: Maximum attitude correction priority
 *          - **MOT_THR_MIX_MIN**: Minimum attitude correction priority
 * 
 * @note MOTB logged at high rate during flight (typically 10-25 Hz)
 * @note lift_max increasing = battery voltage dropping
 * @note th_limit < 1.0 = current/voltage limiting active
 * @note mot_fail_flags should be 2 (balanced) in normal flight
 * 
 * @warning lift_max > 1.5 indicates critically low battery
 * @warning th_limit < 0.8 indicates severe current limiting - land soon
 * @warning mot_fail_flags & 0x01 indicates motor failure - emergency
 * @warning th_out sustained at 1.0 = cannot climb, may descend
 * 
 * @see AP_Motors library for motor mixing and output
 * @see AP_BattMonitor for battery monitoring and limiting
 * @see MOT_* parameters for motor configuration
 */
struct PACKED log_MotBatt {
    LOG_PACKET_HEADER;      ///< Standard 3-byte header: head1, head2, msgid
    uint64_t time_us;       ///< Timestamp in microseconds since system boot
    float   lift_max;       ///< Motor gain compensation factor for battery voltage (1.0 = nominal)
    float   bat_volt;       ///< Battery voltage ratio (0.0 = empty, 1.0 = full)
    float   th_limit;       ///< Maximum throttle available after current/voltage limiting (0-1)
    float th_average_max;   ///< Maximum average throttle to maintain attitude control authority (0-1)
    float th_out;           ///< Actual average throttle output to motors (0-1)
    uint8_t mot_fail_flags; ///< Motor failure status: bit 0 = failure, bit 1 = balanced (2 = normal)
};

/**
 * @struct log_VER
 * @brief VER message: ArduPilot firmware version and board identification
 * 
 * @details The VER message records complete firmware version information, board hardware
 *          details, and build configuration. This message is critical for:
 *          - **Support**: Identifying exact firmware version in logs
 *          - **Debugging**: Reproducing issues with specific builds
 *          - **Compatibility**: Verifying firmware matches hardware
 *          - **Development**: Tracking which git commits generated logs
 *          - **Analysis**: Understanding feature availability in log
 *          
 *          VER messages are logged once at boot and whenever firmware parameters change.
 *          This allows logs to be self-documenting regarding what firmware produced them.
 *          
 *          VER Message Purpose:
 *          - **Version Tracking**: Major.Minor.Patch semantic versioning
 *          - **Git Integration**: Exact git commit hash for reproducibility
 *          - **Board Identification**: Hardware platform and variant
 *          - **Build Metadata**: Debug vs. release, custom builds
 *          - **IOMCU Version**: Separate processor firmware version
 *          - **Filter Version**: Important for EKF and control tuning
 *          
 *          Firmware Versioning Scheme:
 *          @code
 *          ArduPilot Version Format:
 *              Major.Minor.Patch (e.g., 4.5.3)
 *              
 *              Major: Significant architecture changes
 *              Minor: Feature additions, improvements
 *              Patch: Bug fixes, minor updates
 *          
 *          Version Examples:
 *              4.0.0 - Major new version with breaking changes
 *              4.1.0 - Added new features to 4.x series
 *              4.1.5 - Fifth patch release of 4.1
 *              
 *          Development Builds:
 *              4.6.0-dev - Development version, not released
 *              4.6.0-beta1 - First beta test release
 *              4.6.0-rc1 - First release candidate
 *          @endcode
 * 
 * @var log_VER::time_us
 * Timestamp in microseconds since system boot
 * 
 * @var log_VER::board_type
 * HAL board type identifier
 * - **Type**: uint8_t
 * - **Values**: Enumerated HAL_BOARD values
 *   - 0: HAL_BOARD_SITL (Software-in-the-loop simulation)
 *   - 3: HAL_BOARD_LINUX (Linux-based boards: Navio, BBB, etc.)
 *   - 4: HAL_BOARD_CHIBIOS (ChibiOS/ARM-based flight controllers)
 *   - 5: HAL_BOARD_ESP32 (ESP32-based boards)
 *   - 10: HAL_BOARD_EMPTY (Stub/template implementation)
 * 
 * @var log_VER::board_subtype
 * Board-specific subtype within HAL board category
 * - **Type**: uint16_t
 * - **ChibiOS**: Specific flight controller model (Pixhawk variants)
 * - **Linux**: Linux board variant (Navio2, BeagleBone, etc.)
 * - **Purpose**: Distinguish between different hardware in same family
 * 
 * @var log_VER::major
 * Major version number (first digit in X.Y.Z)
 * - **Type**: uint8_t
 * - **Range**: 0-255 (typically 3-5 for modern ArduPilot)
 * - **Increments**: Significant architectural changes, major milestones
 * 
 * @var log_VER::minor
 * Minor version number (second digit in X.Y.Z)
 * - **Type**: uint8_t
 * - **Range**: 0-255
 * - **Increments**: New features, improvements within major version
 * 
 * @var log_VER::patch
 * Patch version number (third digit in X.Y.Z)
 * - **Type**: uint8_t
 * - **Range**: 0-255
 * - **Increments**: Bug fixes, minor updates
 * 
 * @var log_VER::fw_type
 * Firmware vehicle type
 * - **Type**: uint8_t
 * - **Values**: Enumerated APM_BUILD values
 *   - 0: APM_BUILD_DIRECTORY (unknown/misconfigured)
 *   - 1: APM_BUILD_ArduCopter (multirotor)
 *   - 2: APM_BUILD_ArduPlane (fixed-wing)
 *   - 3: APM_BUILD_APMrover2 (ground vehicle)
 *   - 7: APM_BUILD_ArduSub (underwater vehicle)
 *   - 14: APM_BUILD_Blimp (airship)
 *   - 15: APM_BUILD_AP_Periph (CAN peripheral)
 *   - Other values for special builds
 * 
 * @var log_VER::git_hash
 * Git commit hash (short form, 8 hex digits)
 * - **Type**: uint32_t
 * - **Format**: First 4 bytes of git SHA-1 hash
 * - **Purpose**: Exact source code identification
 * - **Example**: 0xABCD1234 corresponds to git commit abcd1234...
 * - **Use**: Checkout exact source with `git checkout HASH`
 * 
 * @var log_VER::fw_string
 * Human-readable firmware version string
 * - **Type**: char[64] (null-terminated)
 * - **Format**: "ArduCopter V4.5.3 (abcd1234)"
 * - **Contents**: Vehicle type, version, optional commit hash
 * - **Purpose**: User-friendly version display
 * 
 * @var log_VER::_APJ_BOARD_ID
 * APJ (ArduPilot JSON) board ID for bootloader
 * - **Type**: uint16_t
 * - **Purpose**: Match firmware to specific hardware during upload
 * - **Bootloader**: Prevents flashing wrong firmware to board
 * - **Examples**:
 *   - 9: Pixhawk 1
 *   - 50: Pixhawk 4
 *   - 1000+: Custom board IDs
 * 
 * @var log_VER::build_type
 * Build configuration type
 * - **Type**: uint8_t
 * - **Values**:
 *   - 0: Unknown/unspecified
 *   - 1: Debug build (assertions enabled, optimization reduced)
 *   - 2: Release build (optimized, assertions disabled)
 *   - 3: Beta build (testing release)
 * - **Debug builds**: Larger, slower, more error checking
 * - **Release builds**: Smaller, faster, production-ready
 * 
 * @var log_VER::filter_version
 * Digital filter library version
 * - **Type**: uint8_t
 * - **Purpose**: Track filter algorithm changes
 * - **Critical for**: EKF tuning, control loop analysis
 * - **Changes**: When filter implementations updated
 * - **Why Important**: Filter changes affect vehicle behavior and tuning
 * 
 * @var log_VER::iomcu_mcu_id
 * I/O MCU (coprocessor) unique MCU identifier
 * - **Type**: uint32_t
 * - **Purpose**: Identify specific IOMCU chip
 * - **Only on**: Boards with separate I/O coprocessor (Pixhawk series)
 * - **Zero**: If no IOMCU present
 * 
 * @var log_VER::iomcu_cpu_id
 * I/O MCU (coprocessor) CPU type identifier
 * - **Type**: uint32_t
 * - **Purpose**: Identify IOMCU processor architecture
 * - **Only on**: Boards with separate I/O coprocessor
 * - **Zero**: If no IOMCU present
 * 
 * @note VER message logged once at boot to identify firmware in log file
 * @note git_hash allows exact reproduction of build from source
 * @note fw_type distinguishes Copter, Plane, Rover, Sub logs
 * @note filter_version critical when analyzing control behavior changes
 * 
 * @see Tools/gitversion.py for git hash extraction
 * @see AP_HAL for board type definitions
 * @see version.h for semantic version information
 */
struct PACKED log_VER {
    LOG_PACKET_HEADER;      ///< Standard 3-byte header: head1, head2, msgid
    uint64_t time_us;       ///< Timestamp in microseconds since system boot
    uint8_t board_type;     ///< HAL_BOARD type (ChibiOS, Linux, ESP32, SITL, etc.)
    uint16_t board_subtype; ///< Specific board model within HAL_BOARD category
    uint8_t major;          ///< Major version number (X in X.Y.Z)
    uint8_t minor;          ///< Minor version number (Y in X.Y.Z)
    uint8_t patch;          ///< Patch version number (Z in X.Y.Z)
    uint8_t fw_type;        ///< Firmware vehicle type (Copter, Plane, Rover, Sub, etc.)
    uint32_t git_hash;      ///< Short git commit hash (first 4 bytes of SHA-1)
    char fw_string[64];     ///< Human-readable version string (e.g., "ArduCopter V4.5.3")
    uint16_t _APJ_BOARD_ID; ///< APJ board ID for bootloader firmware matching
    uint8_t build_type;     ///< Build configuration (debug, release, beta)
    uint8_t filter_version; ///< Digital filter library version (affects EKF/control)
    uint32_t iomcu_mcu_id;  ///< I/O coprocessor unique MCU identifier (0 if none)
    uint32_t iomcu_cpu_id;  ///< I/O coprocessor CPU type identifier (0 if none)
};


/**
 * @brief Log message metadata overview
 * 
 * @details ArduPilot binary logs are self-describing through metadata messages:
 *          
 *          **FMT Messages**: Define all message formats (except FMT itself)
 *          - Specify message type, length, name, format string, and field labels
 *          - Allow log parsers to understand any message type
 *          
 *          **UNIT Messages**: Define units which can be referenced by FMTU messages
 *          - Map single character IDs to SI unit strings
 *          - Example: 'm' = meters, 's' = seconds, 'A' = Amperes
 *          
 *          **FMTU Messages**: Associate units and multipliers to FMT message fields
 *          - Link format fields to their physical units and scaling
 *          - Example: A float field might be "meters" with multiplier "1e-2" (centimeters)
 *          
 *          This three-tier system makes logs fully self-documenting, allowing tools to:
 *          - Parse any log without hardcoded message definitions
 *          - Display values with correct units
 *          - Apply proper scaling to raw data
 *          - Support forward/backward compatibility
 */

/**
 * @def PID_LABELS
 * @brief Standard field labels for all PID controller log messages
 * 
 * @details Defines the comma-separated field names for PID messages.
 *          Used by PIDR, PIDP, PIDY, PIDA, PIDS, PIDN, PIDE messages.
 *          
 *          Field Definitions:
 *          - TimeUS: Timestamp in microseconds
 *          - Tar: Target/desired value (setpoint)
 *          - Act: Actual/measured value (process variable)
 *          - Err: Error (Tar - Act)
 *          - P: Proportional term output
 *          - I: Integral term output
 *          - D: Derivative term output
 *          - FF: Feed-forward term output
 *          - DFF: Derivative feed-forward term output
 *          - Dmod: Derivative modifier (D-term scale factor to reduce oscillation)
 *          - SRate: Slew rate limiter value
 *          - Flags: PID state flags bitmask
 */
#define PID_LABELS "TimeUS,Tar,Act,Err,P,I,D,FF,DFF,Dmod,SRate,Flags"

/**
 * @def PID_FMT
 * @brief Format string for PID controller log messages
 * 
 * @details Defines the binary data types for each PID message field.
 *          - Q: uint64_t (TimeUS)
 *          - f: float (all PID terms and values)
 *          - B: uint8_t (Flags)
 */
#define PID_FMT    "QffffffffffB"

/**
 * @def PID_UNITS
 * @brief Unit string for PID controller log messages
 * 
 * @details Defines the physical units for each PID field.
 *          - s: seconds (TimeUS)
 *          - -: dimensionless/unitless (all PID terms)
 *          
 *          PID terms are unitless because:
 *          - Units depend on what is being controlled (angle, rate, velocity, etc.)
 *          - Each PID instance (PIDR, PIDP, etc.) controls different quantities
 *          - Actual units specified in @LoggerMessage comments
 */
#define PID_UNITS  "s-----------"

/**
 * @def PID_MULTS
 * @brief Multiplier string for PID controller log messages
 * 
 * @details Defines the scaling multipliers for each PID field.
 *          - F: 1e-6 multiplier (TimeUS stored as microseconds)
 *          - -: no multiplier (PID terms stored as-is)
 */
#define PID_MULTS  "F-----------"

// @LoggerMessage: ADSB
// @Description: Automatic Dependent Serveillance - Broadcast detected vehicle information
// @Field: TimeUS: Time since system startup
// @Field: ICAO_address: Transponder address
// @Field: Lat: Vehicle latitude
// @Field: Lng: Vehicle longitude
// @Field: Alt: Vehicle altitude
// @Field: Heading: Vehicle heading
// @Field: Hor_vel: Vehicle horizontal velocity
// @Field: Ver_vel: Vehicle vertical velocity
// @Field: Squark: Transponder squawk code

// @LoggerMessage: ARM
// @Description: Arming status changes
// @Field: TimeUS: Time since system startup
// @Field: ArmState: true if vehicle is now armed
// @Field: ArmChecks: arming bitmask at time of arming
// @FieldBitmaskEnum: ArmChecks: AP_Arming::Check
// @Field: Forced: true if arm/disarm was forced
// @Field: Method: method used for arming
// @FieldValueEnum: Method: AP_Arming::Method

// @LoggerMessage: ARSP
// @Description: Airspeed sensor data
// @Field: TimeUS: Time since system startup
// @Field: I: Airspeed sensor instance number
// @Field: Airspeed: Current airspeed
// @Field: DiffPress: Pressure difference between static and dynamic port
// @Field: Temp: Temperature used for calculation
// @Field: RawPress: Raw pressure less offset
// @Field: Offset: Offset from parameter
// @Field: U: True if sensor is being used
// @Field: H: True if sensor is healthy
// @Field: Hp: Probability sensor is healthy
// @Field: TR: innovation test ratio
// @Field: Pri: True if sensor is the primary sensor

// @LoggerMessage: DMS
// @Description: DataFlash-Over-MAVLink statistics
// @Field: TimeUS: Time since system startup
// @Field: N: Current block number
// @Field: Dp: Number of times we rejected a write to the backend
// @Field: RT: Number of blocks sent from the retry queue
// @Field: RS: Number of resends of unacknowledged data made
// @Field: Fa: Average number of blocks on the free list
// @Field: Fmn: Minimum number of blocks on the free list
// @Field: Fmx: Maximum number of blocks on the free list
// @Field: Pa: Average number of blocks on the pending list
// @Field: Pmn: Minimum number of blocks on the pending list
// @Field: Pmx: Maximum number of blocks on the pending list
// @Field: Sa: Average number of blocks on the sent list
// @Field: Smn: Minimum number of blocks on the sent list
// @Field: Smx: Maximum number of blocks on the sent list

// @LoggerMessage: DSF
// @Description: Onboard logging statistics
// @Field: TimeUS: Time since system startup
// @Field: Dp: Number of times we rejected a write to the backend
// @Field: Blk: Current block number
// @Field: Bytes: Current write offset
// @Field: FMn: Minimum free space in write buffer in last time period
// @Field: FMx: Maximum free space in write buffer in last time period
// @Field: FAv: Average free space in write buffer in last time period

// @LoggerMessage: ERR
// @Description: Specifically coded error messages
// @Field: TimeUS: Time since system startup
// @Field: Subsys: Subsystem in which the error occurred
// @FieldValueEnum: Subsys: LogErrorSubsystem
// @Field: ECode: Subsystem-specific error code

// @LoggerMessage: EV
// @Description: Specifically coded event messages
// @Field: TimeUS: Time since system startup
// @Field: Id: Event identifier
// @FieldValueEnum: Id: LogEvent

// @LoggerMessage: FMT
// @Description: Message defining the format of messages in this file
// @URL: https://ardupilot.org/dev/docs/code-overview-adding-a-new-log-message.html
// @Field: Type: unique-to-this-log identifier for message being defined
// @Field: Length: the number of bytes taken up by this message (including all headers)
// @Field: Name: name of the message being defined
// @Field: Format: character string defining the C-storage-type of the fields in this message
// @Field: Columns: the labels of the message being defined

// @LoggerMessage: FMTU
// @Description: Message defining units and multipliers used for fields of other messages
// @Field: TimeUS: Time since system startup
// @Field: FmtType: numeric reference to associated FMT message
// @Field: UnitIds: each character refers to a UNIT message.  The unit at an offset corresponds to the field at the same offset in FMT.Format
// @Field: MultIds: each character refers to a MULT message.  The multiplier at an offset corresponds to the field at the same offset in FMT.Format

// @LoggerMessage: LGR
// @Description: Landing gear information
// @Field: TimeUS: Time since system startup
// @Field: LandingGear: Current landing gear state
// @FieldValueEnum: LandingGear: AP_LandingGear::LG_LandingGear_State
// @Field: WeightOnWheels: Weight on wheels state
// @FieldValueEnum: WeightOnWheels: AP_LandingGear::LG_WOW_State

// @LoggerMessage: MAG
// @Description: Information received from compasses
// @Field: TimeUS: Time since system startup
// @Field: I: magnetometer sensor instance number
// @Field: MagX: magnetic field strength in body frame
// @Field: MagY: magnetic field strength in body frame
// @Field: MagZ: magnetic field strength in body frame
// @Field: OfsX: magnetic field offset in body frame
// @Field: OfsY: magnetic field offset in body frame
// @Field: OfsZ: magnetic field offset in body frame
// @Field: MOX: motor interference magnetic field offset in body frame
// @Field: MOY: motor interference magnetic field offset in body frame
// @Field: MOZ: motor interference magnetic field offset in body frame
// @Field: Health: true if the compass is considered healthy
// @Field: S: time measurement was taken

// @LoggerMessage: MAV
// @Description: GCS MAVLink link statistics
// @Field: TimeUS: Time since system startup
// @Field: chan: mavlink channel number
// @Field: txp: transmitted packet count
// @Field: rxp: received packet count
// @Field: rxdp: perceived number of packets we never received
// @Field: flags: compact representation of some state of the channel
// @FieldBitmaskEnum: flags: GCS_MAVLINK::Flags
// @Field: ss: stream slowdown is the number of ms being added to each message to fit within bandwidth
// @Field: tf: times buffer was full when a message was going to be sent
// @Field: mgs: time MAV_GCS_SYSID heartbeat (or manual control) last seen

// @LoggerMessage: MAVC
// @Description: MAVLink command we have just executed
// @Field: TimeUS: Time since system startup
// @Field: TS: target system for command
// @Field: TC: target component for command
// @Field: SS: source system for command
// @Field: SC: source component for command
// @Field: Fr: command frame
// @Field: Cmd: mavlink command enum value
// @Field: P1: first parameter from mavlink packet
// @Field: P2: second parameter from mavlink packet
// @Field: P3: third parameter from mavlink packet
// @Field: P4: fourth parameter from mavlink packet
// @Field: X: X coordinate from mavlink packet
// @Field: Y: Y coordinate from mavlink packet
// @Field: Z: Z coordinate from mavlink packet
// @Field: Res: command result being returned from autopilot
// @Field: WL: true if this command arrived via a COMMAND_LONG rather than COMMAND_INT

// @LoggerMessage: MODE
// @Description: vehicle control mode information
// @Field: TimeUS: Time since system startup
// @Field: Mode: vehicle-specific mode number
// @Field: ModeNum: alias for Mode
// @Field: Rsn: reason for entering this mode; enumeration value
// @FieldValueEnum: Rsn: ModeReason

// @LoggerMessage: MSG
// @Description: Textual messages
// @Field: TimeUS: Time since system startup
// @Field: Message: message text

// @LoggerMessage: MULT
// @Description: Message mapping from single character to numeric multiplier
// @Field: TimeUS: Time since system startup
// @Field: Id: character referenced by FMTU
// @Field: Mult: numeric multiplier

// @LoggerMessage: OF
// @Description: Optical flow sensor data
// @Field: TimeUS: Time since system startup
// @Field: Qual: Estimated sensor data quality
// @Field: flowX: Sensor flow rate, X-axis
// @Field: flowY: Sensor flow rate,Y-axis
// @Field: bodyX: derived rotational velocity, X-axis
// @Field: bodyY: derived rotational velocity, Y-axis

// @LoggerMessage: PARM
// @Description: parameter value
// @Field: TimeUS: Time since system startup
// @Field: Name: parameter name
// @Field: Value: parameter value
// @Field: Default: default parameter value for this board and config

// @LoggerMessage: PIDR
// @Description: Proportional/Integral/Derivative gain values for Roll rate
// @LoggerMessage: PIDP
// @Description: Proportional/Integral/Derivative gain values for Pitch rate
// @LoggerMessage: PIDY
// @Description: Proportional/Integral/Derivative gain values for Yaw rate
// @LoggerMessage: PIDA
// @Description: Proportional/Integral/Derivative gain values for vertical acceleration
// @LoggerMessage: PIDS
// @Description: Proportional/Integral/Derivative gain values for ground steering yaw rate
// @LoggerMessage: PIDN
// @Description: Proportional/Integral/Derivative gain values for North/South velocity
// @LoggerMessage: PIDE
// @Description: Proportional/Integral/Derivative gain values for East/West velocity
// @Field: TimeUS: Time since system startup
// @Field: Tar: desired value
// @Field: Act: achieved value
// @Field: Err: error between target and achieved
// @Field: P: proportional part of PID
// @Field: I: integral part of PID
// @Field: D: derivative part of PID
// @Field: FF: controller feed-forward portion of response
// @Field: DFF: controller derivative feed-forward portion of response
// @Field: Dmod: scaler applied to D gain to reduce limit cycling
// @Field: SRate: slew rate used in slew limiter
// @Field: Flags: bitmask of PID state flags
// @FieldBitmaskEnum: Flags: log_PID_Flags

// @LoggerMessage: PM
// @Description: autopilot system performance and general data dumping ground
// @Field: TimeUS: Time since system startup
// @Field: LR: Main loop rate
// @Field: NLon: Number of long loops detected
// @Field: NL: Number of measurement loops for this message
// @Field: MaxT: Maximum loop time
// @Field: Mem: Free memory available
// @Field: Load: System processor load
// @Field: InE: Internal error mask; which internal errors have been detected
// @FieldBitmaskEnum: InE: AP_InternalError::error_t
// @Field: ErrL: Internal error line number; last line number on which a internal error was detected
// @Field: ErC: Internal error count; how many internal errors have been detected
// @Field: SPIC: Number of SPI transactions processed
// @Field: I2CC: Number of i2c transactions processed
// @Field: I2CI: Number of i2c interrupts serviced
// @Field: Ex: number of microseconds being added to each loop to address scheduler overruns
// @Field: R: RTC time, time since Unix epoch

// @LoggerMessage: POWR
// @Description: System power information
// @Field: TimeUS: Time since system startup
// @Field: Vcc: Flight board voltage
// @Field: VServo: Servo rail voltage
// @Field: Flags: System power flags
// @FieldBitmaskEnum: Flags: AP_HAL::AnalogIn::PowerStatusFlag
// @Field: AccFlags: Accumulated System power flags; all flags which have ever been set
// @FieldBitmaskEnum: AccFlags: AP_HAL::AnalogIn::PowerStatusFlag
// @Field: Safety: Hardware Safety Switch status

// @LoggerMessage: MCU
// @Description: MCU voltage and temprature monitering
// @Field: TimeUS: Time since system startup
// @Field: MTemp: Temperature
// @Field: MVolt: Voltage
// @Field: MVmin: Voltage min
// @Field: MVmax: Voltage max

// @LoggerMessage: RAD
// @Description: Telemetry radio statistics
// @Field: TimeUS: Time since system startup
// @Field: RSSI: RSSI
// @Field: RemRSSI: RSSI reported from remote radio
// @Field: TxBuf: number of bytes in radio ready to be sent
// @Field: Noise: local noise floor
// @Field: RemNoise: local noise floor reported from remote radio
// @Field: RxErrors: damaged packet count
// @Field: Fixed: fixed damaged packet count

// @LoggerMessage: RALY
// @Description: Rally point information
// @Field: TimeUS: Time since system startup
// @Field: Tot: total number of rally points onboard
// @Field: Seq: this rally point's sequence number
// @Field: Lat: latitude of rally point
// @Field: Lng: longitude of rally point
// @Field: Alt: altitude of rally point
// @Field: Flags: altitude frame flags

// @LoggerMessage: RCI2
// @Description: (More) RC input channels to vehicle
// @Field: TimeUS: Time since system startup
// @Field: C15: channel 15 input
// @Field: C16: channel 16 input
// @Field: OMask: bitmask of RC channels being overridden by mavlink input
// @Field: Flags: bitmask of RC state flags
// @FieldBitmaskEnum: Flags: AP_Logger::RCLoggingFlags

// @LoggerMessage: RCIN
// @Description: RC input channels to vehicle
// @Field: TimeUS: Time since system startup
// @Field: C1: channel 1 input
// @Field: C2: channel 2 input
// @Field: C3: channel 3 input
// @Field: C4: channel 4 input
// @Field: C5: channel 5 input
// @Field: C6: channel 6 input
// @Field: C7: channel 7 input
// @Field: C8: channel 8 input
// @Field: C9: channel 9 input
// @Field: C10: channel 10 input
// @Field: C11: channel 11 input
// @Field: C12: channel 12 input
// @Field: C13: channel 13 input
// @Field: C14: channel 14 input

// @LoggerMessage: RCOU
// @Description: Servo channel output values 1 to 14
// @Field: TimeUS: Time since system startup
// @Field: C1: channel 1 output
// @Field: C2: channel 2 output
// @Field: C3: channel 3 output
// @Field: C4: channel 4 output
// @Field: C5: channel 5 output
// @Field: C6: channel 6 output
// @Field: C7: channel 7 output
// @Field: C8: channel 8 output
// @Field: C9: channel 9 output
// @Field: C10: channel 10 output
// @Field: C11: channel 11 output
// @Field: C12: channel 12 output
// @Field: C13: channel 13 output
// @Field: C14: channel 14 output

// @LoggerMessage: RCO2
// @Description: Servo channel output values 15 to 18
// @Field: TimeUS: Time since system startup
// @Field: C15: channel 15 output
// @Field: C16: channel 16 output
// @Field: C17: channel 17 output
// @Field: C18: channel 18 output

// @LoggerMessage: RCO3
// @Description: Servo channel output values 19 to 32
// @Field: TimeUS: Time since system startup
// @Field: C19: channel 19 output
// @Field: C20: channel 20 output
// @Field: C21: channel 21 output
// @Field: C22: channel 22 output
// @Field: C23: channel 23 output
// @Field: C24: channel 24 output
// @Field: C25: channel 25 output
// @Field: C26: channel 26 output
// @Field: C27: channel 27 output
// @Field: C28: channel 28 output
// @Field: C29: channel 29 output
// @Field: C30: channel 30 output
// @Field: C31: channel 31 output
// @Field: C32: channel 32 output

// @LoggerMessage: RFND
// @Description: Rangefinder sensor information
// @Field: TimeUS: Time since system startup
// @Field: Instance: rangefinder instance number this data is from
// @Field: Dist: Reported distance from sensor
// @Field: Stat: Sensor state
// @FieldValueEnum: Stat: RangeFinder::Status
// @Field: Orient: Sensor orientation
// @FieldValueEnum: Orient: Rotation
// @Field: Quality: Signal quality. -1 means invalid, 0 is no signal, 100 is perfect signal

// @LoggerMessage: RSSI
// @Description: Received Signal Strength Indicator for RC receiver
// @Field: TimeUS: Time since system startup
// @Field: RXRSSI: RSSI
// @Field: RXLQ: RX Link Quality

// @LoggerMessage: SIM
// @Description: SITL simulator state
// @Field: TimeUS: Time since system startup
// @Field: Roll: Simulated roll
// @Field: Pitch: Simulated pitch
// @Field: Yaw: Simulated yaw
// @Field: Alt: Simulated altitude
// @Field: Lat: Simulated latitude
// @Field: Lng: Simulated longitude
// @Field: Q1: Attitude quaternion component 1
// @Field: Q2: Attitude quaternion component 2
// @Field: Q3: Attitude quaternion component 3
// @Field: Q4: Attitude quaternion component 4

// @LoggerMessage: SRTL
// @Description: SmartRTL statistics
// @Field: TimeUS: Time since system startup
// @Field: Active: true if SmartRTL could be used right now
// @Field: NumPts: number of points currently in use
// @Field: MaxPts: maximum number of points that could be used
// @Field: Action: most recent internal action taken by SRTL library
// @FieldValueEnum: Action: AP_SmartRTL::Action
// @Field: N: point associated with most recent action (North component)
// @Field: E: point associated with most recent action (East component)
// @Field: D: point associated with most recent action (Down component)

// @LoggerMessage: TERR
// @Description: Terrain database information
// @Field: TimeUS: Time since system startup
// @Field: Status: Terrain database status
// @FieldValueEnum: Status: AP_Terrain::TerrainStatus
// @Field: Lat: Current vehicle latitude
// @Field: Lng: Current vehicle longitude
// @Field: Spacing: terrain Tile spacing
// @Field: TerrH: current Terrain height
// @Field: CHeight: Vehicle height above terrain
// @Field: Pending: Number of tile requests outstanding
// @Field: Loaded: Number of tiles in memory
// @Field: ROfs: terrain reference offset for arming altitude

// @LoggerMessage: TSYN
// @Description: Time synchronisation response information
// @Field: TimeUS: Time since system startup
// @Field: SysID: system ID this data is for
// @Field: RTT: round trip time for this system

// @LoggerMessage: UNIT
// @Description: Message mapping from single character to SI unit
// @Field: TimeUS: Time since system startup
// @Field: Id: character referenced by FMTU
// @Field: Label: Unit - SI where available

// @LoggerMessage: WENC
// @Description: Wheel encoder measurements
// @Field: TimeUS: Time since system startup
// @Field: Dist0: First wheel distance travelled
// @Field: Qual0: Quality measurement of Dist0
// @Field: Dist1: Second wheel distance travelled
// @Field: Qual1: Quality measurement of Dist1

// @LoggerMessage: WINC
// @Description: Winch
// @Field: TimeUS: Time since system startup
// @Field: Heal: Healthy
// @Field: ThEnd: Reached end of thread
// @Field: Mov: Motor is moving
// @Field: Clut: Clutch is engaged (motor can move freely)
// @Field: Mode: 0 is Relaxed, 1 is Position Control, 2 is Rate Control
// @Field: DLen: Desired Length
// @Field: Len: Estimated Length
// @Field: DRate: Desired Rate
// @Field: Tens: Tension on line
// @Field: Vcc: Voltage to Motor
// @Field: Temp: Motor temperature

// @LoggerMessage: STAK
// @Description: Stack information
// @Field: TimeUS: Time since system startup
// @Field: Id: thread ID
// @Field: Pri: thread priority
// @Field: Total: total stack
// @Field: Free: free stack
// @Field: Name: thread name

// @LoggerMessage: FILE
// @Description: File data
// @Field: FileName: File name
// @Field: Offset: Offset into the file of this block
// @Field: Length: Length of this data block
// @Field: Data: File data of this block

// @LoggerMessage: SCR
// @Description: Scripting runtime stats
// @Field: TimeUS: Time since system startup
// @Field: Name: script name
// @Field: Runtime: run time
// @Field: Total_mem: total memory usage of all scripts
// @Field: Run_mem: run memory usage

// @LoggerMessage: VER
// @Description: Ardupilot version
// @Field: TimeUS: Time since system startup
// @Field: BT: Board type
// @FieldValueEnum: BT: HAL_BOARD
// @Field: BST: Board subtype
// @FieldValueEnum: BST: HAL_BOARD_SUBTYPE
// @Field: Maj: Major version number
// @Field: Min: Minor version number
// @Field: Pat: Patch number
// @Field: FWT: Firmware type
// @Field: GH: Github commit
// @Field: FWS: Firmware version string
// @Field: APJ: Board ID
// @Field: BU: Build vehicle type
// @FieldValueEnum: BU: APM_BUILD
// @Field: FV: Filter version
// @Field: IMI: IOMCU MCU ID
// @Field: ICI: IOMCU CPU ID

// @LoggerMessage: MOTB
// @Description: Motor mixer information
// @Field: TimeUS: Time since system startup
// @Field: LiftMax: Maximum motor compensation gain
// @Field: BatVolt: Ratio between detected battery voltage and maximum battery voltage
// @Field: ThLimit: Throttle limit set due to battery current limitations
// @Field: ThrAvMx: Maximum average throttle that can be used to maintain attitude control, derived from throttle mix params
// @Field: ThrOut: Throttle output
// @Field: FailFlags: bit 0 motor failed, bit 1 motors balanced, should be 2 in normal flight

/**
 * @def LOG_COMMON_STRUCTURES
 * @brief Master macro defining all common log message structure definitions
 * 
 * @details This macro expands to a complete initialization list for the LogStructure array,
 *          defining all log messages common to every ArduPilot vehicle type (Copter, Plane,
 *          Rover, Sub, Blimp, Tracker, Periph). Vehicle-specific messages are defined in
 *          each vehicle's Log.cpp file and combined with these common structures.
 *          
 *          Purpose of LOG_COMMON_STRUCTURES:
 *          - **Centralized Definition**: All shared message formats in one place
 *          - **Consistency**: Same messages logged identically across all vehicles
 *          - **Maintenance**: Update once, applies to all vehicles
 *          - **Modularity**: Vehicle-specific messages defined separately
 *          
 *          Structure Definition Format:
 *          Each entry in the macro follows this pattern:
 *          @code
 *          { message_type, sizeof(struct), "NAME", "Format", "Labels", "Units", "Multipliers", streaming }
 *          
 *          Fields:
 *              message_type: Unique uint8_t identifier (LOG_*_MSG enum)
 *              sizeof(struct): Size in bytes of the log message struct
 *              "NAME": 4-character message name (e.g., "FMT", "GPS", "IMU")
 *              "Format": Format string specifying C types of each field
 *              "Labels": Comma-separated field names
 *              "Units": Single-character unit IDs (from log_Units[])
 *              "Multipliers": Single-character multiplier IDs (from log_Multipliers[])
 *              streaming: bool, true if message can be rate-limited
 *          @endcode
 *          
 *          Format String Characters (repeated from file header for reference):
 *          - b: int8_t
 *          - B: uint8_t
 *          - h: int16_t
 *          - H: uint16_t
 *          - i: int32_t
 *          - I: uint32_t
 *          - f: float
 *          - d: double
 *          - q: int64_t
 *          - Q: uint64_t
 *          - n: char[4]
 *          - N: char[16]
 *          - Z: char[64]
 *          - c: int16_t * 100
 *          - C: uint16_t * 100
 *          - e: int32_t * 100
 *          - E: uint32_t * 100
 *          - L: int32_t (latitude/longitude)
 *          - M: uint8_t (flight mode)
 *          
 *          Unit Character IDs (from log_Units[]):
 *          - '-': no units (dimensionless)
 *          - 's': seconds
 *          - 'm': meters
 *          - 'n': meters/second
 *          - 'o': meters/second/second
 *          - 'd': degrees
 *          - 'k': degrees/second
 *          - 'A': Amperes
 *          - 'v': Volts
 *          - 'G': Gauss
 *          - 'Y': microseconds (PWM)
 *          - See log_Units[] array for complete list
 *          
 *          Multiplier Character IDs (from log_Multipliers[]):
 *          - '-': no multiplier (raw value)
 *          - '0': 1e0 (×1)
 *          - 'A': 1e-1 (÷10)
 *          - 'B': 1e-2 (÷100, centimeters)
 *          - 'C': 1e-3 (÷1000, millimeters)
 *          - 'F': 1e-6 (÷1,000,000, microseconds)
 *          - See log_Multipliers[] array for complete list
 *          
 *          Streaming Flag:
 *          - **true**: Message can be rate-limited to save bandwidth/storage
 *          - **false**: Message always logged at full rate (critical messages)
 *          - High-rate sensor data typically streaming=true
 *          - Events and state changes typically streaming=false
 *          
 *          Macro Structure:
 *          The macro is organized in sections:
 *          1. **Metadata Messages**: FMT, UNIT, FMTU, MULT (self-describing log format)
 *          2. **System Messages**: PARM, MSG, VER (configuration and version info)
 *          3. **Input Messages**: RCIN, RCI2, RSSI (RC receiver inputs)
 *          4. **Output Messages**: RCOU, RCO2, RCO3 (servo/motor outputs)
 *          5. **Communication**: MAV, MAVC, RAD (MAVLink and telemetry radio)
 *          6. **Power**: POWR, MCU (voltage and system power monitoring)
 *          7. **Sensor Data**: Inserted via LOG_STRUCTURE_FROM_* macros
 *          8. **Navigation**: Inserted via LOG_STRUCTURE_FROM_NAVEKF*, LOG_STRUCTURE_FROM_AHRS
 *          9. **Control**: PID messages for rate and position controllers
 *          10. **Mission**: MODE, TERR, RALY, SRTL (mission and mode information)
 *          11. **Safety**: ARM, ERR, EV (arming, errors, events)
 *          12. **Peripherals**: WENC, ADSB, WINC, STAK, FILE, SCR, MOTB
 *          
 *          LOG_STRUCTURE_FROM_* Macros:
 *          External modules define their own message structures via macros:
 *          - LOG_STRUCTURE_FROM_GPS: GPS-related messages (from AP_GPS/LogStructure.h)
 *          - LOG_STRUCTURE_FROM_BARO: Barometer messages (from AP_Baro/LogStructure.h)
 *          - LOG_STRUCTURE_FROM_NAVEKF2: EKF2 messages (from AP_NavEKF2/LogStructure.h)
 *          - LOG_STRUCTURE_FROM_NAVEKF3: EKF3 messages (from AP_NavEKF3/LogStructure.h)
 *          - LOG_STRUCTURE_FROM_BATTMONITOR: Battery messages (from AP_BattMonitor/LogStructure.h)
 *          - LOG_STRUCTURE_FROM_INERTIALSENSOR: IMU messages (from AP_InertialSensor/LogStructure.h)
 *          - And many more from included modules
 *          
 *          This modular approach allows each library to define its own log messages while
 *          still contributing to the common message table.
 *          
 *          Example Entry Interpretation:
 *          @code
 *          { LOG_RCIN_MSG, sizeof(log_RCIN), 
 *            "RCIN", "QHHHHHHHHHHHHHH",
 *            "TimeUS,C1,C2,C3,C4,C5,C6,C7,C8,C9,C10,C11,C12,C13,C14",
 *            "sYYYYYYYYYYYYYY", "F--------------", true }
 *          
 *          Interpretation:
 *              - Message type: LOG_RCIN_MSG (enum value)
 *              - Size: sizeof(log_RCIN) = 3 + 8 + 14×2 = 39 bytes
 *              - Name: "RCIN" (RC Input)
 *              - Format: Q (uint64_t TimeUS), 14× H (uint16_t channels)
 *              - Labels: TimeUS and channels C1-C14
 *              - Units: 's' (seconds) for TimeUS, 'Y' (microseconds) for all channels
 *              - Multipliers: 'F' (1e-6) for TimeUS, '-' (none) for channels
 *              - Streaming: true (can be rate-limited)
 *          
 *          Result: Each RC input channel stored as uint16_t in microseconds (1000-2000),
 *          with timestamp as uint64_t divided by 1e6 to convert to seconds.
 *          @endcode
 *          
 *          Usage in Vehicle Code:
 *          @code
 *          // In each vehicle's Log.cpp:
 *          const LogStructure Copter::log_structure[] = {
 *              LOG_COMMON_STRUCTURES,        // Expand common messages
 *              // Vehicle-specific messages:
 *              { LOG_CONTROL_TUNING_MSG, ... },
 *              { LOG_ATTITUDE_MSG, ... },
 *              ...
 *          };
 *          @endcode
 * 
 * @note This macro must end with a comma so vehicle-specific entries can follow
 * @note Modifying this macro affects all vehicles - test thoroughly
 * @note Message type enums must be unique across common + vehicle-specific messages
 * @note Keep format/labels/units/multipliers strings within size limits (LS_*_SIZE)
 * 
 * @warning Adding new common messages requires enum allocation in LogMessages
 * @warning Changing existing message formats breaks log compatibility
 * @warning Total messages (common + vehicle) must not exceed 255 (uint8_t limit)
 * 
 * @see LogStructure struct definition for field descriptions
 * @see LogMessages enum for message type identifiers
 * @see Individual vehicle Log.cpp for vehicle-specific message additions
 * @see AP_Logger library for log writing and reading implementation
 */
// messages for all boards
#define LOG_COMMON_STRUCTURES \
    { LOG_FORMAT_MSG, sizeof(log_Format), \
      "FMT", "BBnNZ",      "Type,Length,Name,Format,Columns", "-b---", "-----" },    \
    { LOG_UNIT_MSG, sizeof(log_Unit), \
      "UNIT", "QbZ",      "TimeUS,Id,Label", "s--","F--" },    \
    { LOG_FORMAT_UNITS_MSG, sizeof(log_Format_Units), \
      "FMTU", "QBNN",      "TimeUS,FmtType,UnitIds,MultIds","s---", "F---" },   \
    { LOG_MULT_MSG, sizeof(log_Format_Multiplier), \
      "MULT", "Qbd",      "TimeUS,Id,Mult", "s--","F--" },   \
    { LOG_PARAMETER_MSG, sizeof(log_Parameter), \
     "PARM", "QNff",        "TimeUS,Name,Value,Default", "s---", "F---"  },       \
LOG_STRUCTURE_FROM_GPS \
    { LOG_MESSAGE_MSG, sizeof(log_Message), \
      "MSG",  "QZ",     "TimeUS,Message", "s-", "F-"}, \
    { LOG_RCIN_MSG, sizeof(log_RCIN), \
      "RCIN",  "QHHHHHHHHHHHHHH",     "TimeUS,C1,C2,C3,C4,C5,C6,C7,C8,C9,C10,C11,C12,C13,C14", "sYYYYYYYYYYYYYY", "F--------------", true }, \
    { LOG_RCI2_MSG, sizeof(log_RCI2), \
      "RCI2",  "QHHHB",     "TimeUS,C15,C16,OMask,Flags", "sYY--", "F----", true }, \
    { LOG_RCOUT_MSG, sizeof(log_RCOUT), \
      "RCOU",  "QHHHHHHHHHHHHHH",     "TimeUS,C1,C2,C3,C4,C5,C6,C7,C8,C9,C10,C11,C12,C13,C14", "sYYYYYYYYYYYYYY", "F--------------", true  }, \
    { LOG_RCOUT2_MSG, sizeof(log_RCOUT2), \
      "RCO2",  "QHHHH",     "TimeUS,C15,C16,C17,C18", "sYYYY", "F----", true  }, \
    { LOG_RCOUT3_MSG, sizeof(log_RCOUT), \
      "RCO3",  "QHHHHHHHHHHHHHH",     "TimeUS,C19,C20,C21,C22,C23,C24,C25,C26,C27,C28,C29,C30,C31,C32", "sYYYYYYYYYYYYYY", "F--------------", true  }, \
    { LOG_RSSI_MSG, sizeof(log_RSSI), \
      "RSSI",  "Qff",     "TimeUS,RXRSSI,RXLQ", "s-%", "F--", true  }, \
LOG_STRUCTURE_FROM_BARO \
LOG_STRUCTURE_FROM_CANMANAGER \
LOG_STRUCTURE_FROM_PRECLAND \
    { LOG_POWR_MSG, sizeof(log_POWR), \
      "POWR","QffHHB","TimeUS,Vcc,VServo,Flags,AccFlags,Safety", "svv---", "F00---", true }, \
    { LOG_MCU_MSG, sizeof(log_MCU), \
      "MCU","Qffff","TimeUS,MTemp,MVolt,MVmin,MVmax", "sOvvv", "F0000", true }, \
LOG_STRUCTURE_FROM_MISSION \
    { LOG_MAVLINK_COMMAND_MSG, sizeof(log_MAVLink_Command), \
      "MAVC", "QBBBBBHffffiifBB","TimeUS,TS,TC,SS,SC,Fr,Cmd,P1,P2,P3,P4,X,Y,Z,Res,WL", "s---------------", "F---------------" }, \
    { LOG_RADIO_MSG, sizeof(log_Radio), \
      "RAD", "QBBBBBHH", "TimeUS,RSSI,RemRSSI,TxBuf,Noise,RemNoise,RxErrors,Fixed", "s-------", "F-------", true }, \
LOG_STRUCTURE_FROM_CAMERA \
LOG_STRUCTURE_FROM_MOUNT \
    { LOG_ARSP_MSG, sizeof(log_ARSP), "ARSP",  "QBffcffBBffB", "TimeUS,I,Airspeed,DiffPress,Temp,RawPress,Offset,U,H,Hp,TR,Pri", "s#nPOPP-----", "F-00B00-----", true }, \
    LOG_STRUCTURE_FROM_BATTMONITOR \
    { LOG_MAG_MSG, sizeof(log_MAG), \
      "MAG", "QBhhhhhhhhhBI",    "TimeUS,I,MagX,MagY,MagZ,OfsX,OfsY,OfsZ,MOX,MOY,MOZ,Health,S", "s#GGGGGGGGG-s", "F-CCCCCCCCC-F", true }, \
    { LOG_MODE_MSG, sizeof(log_Mode), \
      "MODE", "QMBB",         "TimeUS,Mode,ModeNum,Rsn", "s---", "F---" }, \
    { LOG_RFND_MSG, sizeof(log_RFND), \
      "RFND", "QBfBBb", "TimeUS,Instance,Dist,Stat,Orient,Quality", "s#m--%", "F-0---", true }, \
    { LOG_DMS_MSG, sizeof(log_DMS), \
      "DMS", "QIIIIBBBBBBBBB",         "TimeUS,N,Dp,RT,RS,Fa,Fmn,Fmx,Pa,Pmn,Pmx,Sa,Smn,Smx", "s-------------", "F-------------" }, \
    LOG_STRUCTURE_FROM_BEACON                                       \
    LOG_STRUCTURE_FROM_PROXIMITY                                    \
    { LOG_PERFORMANCE_MSG, sizeof(log_Performance),                     \
      "PM",  "QHHHIIHHIIIIIIQ", "TimeUS,LR,NLon,NL,MaxT,Mem,Load,ErrL,InE,ErC,SPIC,I2CC,I2CI,Ex,R", "sz---b%------ss", "F----0A------FF" }, \
    { LOG_SRTL_MSG, sizeof(log_SRTL), \
      "SRTL", "QBHHBfff", "TimeUS,Active,NumPts,MaxPts,Action,N,E,D", "s----mmm", "F----000" }, \
LOG_STRUCTURE_FROM_AVOIDANCE \
    { LOG_SIMSTATE_MSG, sizeof(log_AHRS), \
      "SIM","QccCfLLffff","TimeUS,Roll,Pitch,Yaw,Alt,Lat,Lng,Q1,Q2,Q3,Q4", "sddhmDU----", "FBBB0GG0000", true }, \
    { LOG_TERRAIN_MSG, sizeof(log_TERRAIN), \
      "TERR","QBLLHffHHf","TimeUS,Status,Lat,Lng,Spacing,TerrH,CHeight,Pending,Loaded,ROfs", "s-DU-mm--m", "F-GG-00--0", true }, \
LOG_STRUCTURE_FROM_ESC_TELEM \
LOG_STRUCTURE_FROM_SERVO_TELEM \
    { LOG_PIDR_MSG, sizeof(log_PID), \
      "PIDR", PID_FMT,  PID_LABELS, PID_UNITS, PID_MULTS, true },  \
    { LOG_PIDP_MSG, sizeof(log_PID), \
      "PIDP", PID_FMT,  PID_LABELS, PID_UNITS, PID_MULTS , true }, \
    { LOG_PIDY_MSG, sizeof(log_PID), \
      "PIDY", PID_FMT,  PID_LABELS, PID_UNITS, PID_MULTS , true }, \
    { LOG_PIDA_MSG, sizeof(log_PID), \
      "PIDA", PID_FMT,  PID_LABELS, PID_UNITS, PID_MULTS , true }, \
    { LOG_PIDS_MSG, sizeof(log_PID), \
      "PIDS", PID_FMT,  PID_LABELS, PID_UNITS, PID_MULTS , true }, \
    { LOG_PIDN_MSG, sizeof(log_PID), \
      "PIDN", PID_FMT,  PID_LABELS, PID_UNITS, PID_MULTS , true }, \
    { LOG_PIDE_MSG, sizeof(log_PID), \
      "PIDE", PID_FMT,  PID_LABELS, PID_UNITS, PID_MULTS , true }, \
LOG_STRUCTURE_FROM_LANDING \
LOG_STRUCTURE_FROM_INERTIALSENSOR \
LOG_STRUCTURE_FROM_DAL \
LOG_STRUCTURE_FROM_NAVEKF2 \
LOG_STRUCTURE_FROM_NAVEKF3 \
LOG_STRUCTURE_FROM_NAVEKF \
LOG_STRUCTURE_FROM_AHRS \
LOG_STRUCTURE_FROM_HAL_CHIBIOS \
LOG_STRUCTURE_FROM_HAL \
LOG_STRUCTURE_FROM_RPM \
LOG_STRUCTURE_FROM_FENCE \
    { LOG_DF_FILE_STATS, sizeof(log_DSF), \
      "DSF", "QIHIIII", "TimeUS,Dp,Blk,Bytes,FMn,FMx,FAv", "s--b---", "F--0---" }, \
    { LOG_RALLY_MSG, sizeof(log_Rally), \
      "RALY", "QBBLLhB", "TimeUS,Tot,Seq,Lat,Lng,Alt,Flags", "s--DUm-", "F--GGB-" },  \
    { LOG_MAV_MSG, sizeof(log_MAV),   \
      "MAV", "QBHHHBHHI",   "TimeUS,chan,txp,rxp,rxdp,flags,ss,tf,mgs", "s#----s-s", "F-000-C-C" },   \
LOG_STRUCTURE_FROM_VISUALODOM \
    { LOG_OPTFLOW_MSG, sizeof(log_Optflow), \
      "OF",   "QBffff",   "TimeUS,Qual,flowX,flowY,bodyX,bodyY", "s-EEEE", "F-0000" , true }, \
    { LOG_WHEELENCODER_MSG, sizeof(log_WheelEncoder), \
      "WENC",  "Qfbfb", "TimeUS,Dist0,Qual0,Dist1,Qual1", "sm-m-", "F0-0-" , true }, \
    { LOG_ADSB_MSG, sizeof(log_ADSB), \
      "ADSB",  "QIiiiHHhH", "TimeUS,ICAO_address,Lat,Lng,Alt,Heading,Hor_vel,Ver_vel,Squark", "s-DUmhnn-", "F-GGCBCC-" }, \
    { LOG_EVENT_MSG, sizeof(log_Event), \
      "EV",   "QB",           "TimeUS,Id", "s-", "F-" }, \
    { LOG_ARM_DISARM_MSG, sizeof(log_Arm_Disarm), \
      "ARM", "QBIBB", "TimeUS,ArmState,ArmChecks,Forced,Method", "s----", "F----" }, \
    { LOG_ERROR_MSG, sizeof(log_Error), \
      "ERR",   "QBB",         "TimeUS,Subsys,ECode", "s--", "F--" }, \
    { LOG_WINCH_MSG, sizeof(log_Winch), \
      "WINC", "QBBBBBfffHfb", "TimeUS,Heal,ThEnd,Mov,Clut,Mode,DLen,Len,DRate,Tens,Vcc,Temp", "s-----mmn?vO", "F-----000000" }, \
    LOG_STRUCTURE_FROM_AC_ATTITUDECONTROL,                              \
    { LOG_STAK_MSG, sizeof(log_STAK), \
      "STAK", "QBBHHN", "TimeUS,Id,Pri,Total,Free,Name", "s#----", "F-----", true }, \
    { LOG_FILE_MSG, sizeof(log_File), \
      "FILE",   "NIBZ",       "FileName,Offset,Length,Data", "----", "----" }, \
LOG_STRUCTURE_FROM_AIS \
    { LOG_SCRIPTING_MSG, sizeof(log_Scripting), \
      "SCR",   "QNIii", "TimeUS,Name,Runtime,Total_mem,Run_mem", "s#sbb", "F-F--", true }, \
    { LOG_VER_MSG, sizeof(log_VER), \
      "VER",   "QBHBBBBIZHBBII", "TimeUS,BT,BST,Maj,Min,Pat,FWT,GH,FWS,APJ,BU,FV,IMI,ICI", "s-------------", "F-------------", false }, \
    { LOG_MOTBATT_MSG, sizeof(log_MotBatt), \
      "MOTB", "QfffffB",  "TimeUS,LiftMax,BatVolt,ThLimit,ThrAvMx,ThrOut,FailFlags", "s------", "F------" , true }

/**
 * @brief Message type ID reservation for vehicle-specific messages
 * 
 * @details Message type IDs 0-31 are reserved exclusively for vehicle-specific log messages.
 *          This reservation ensures that each vehicle type (Copter, Plane, Rover, Sub, Blimp,
 *          Tracker, Periph) can define its own unique messages without conflicts.
 *          
 *          Reserved Range: 0-31 (32 message slots)
 *          - ArduCopter: Attitude, control tuning, guided target, etc.
 *          - ArduPlane: TECS, L1 controller, AETR (aileron/elevator/throttle/rudder), etc.
 *          - Rover: Steering, throttle, wheel encoder, pivot turn, etc.
 *          - ArduSub: Depth, attitude target, control tuning, etc.
 *          - Blimp: Fins, buoyancy, wind, etc.
 *          - AntennaTracker: Tracking state, target position, etc.
 *          
 *          Each vehicle's Log.cpp defines its specific usage of IDs 0-31.
 */
// message types 0 to 31 reserved for vehicle-specific use

/**
 * @enum LogMessages
 * @brief Enumeration of all common log message type identifiers
 * 
 * @details This enum assigns unique uint8_t identifiers to every common log message type
 *          shared across all ArduPilot vehicles. Message type IDs are used as the third
 *          byte (msgid) in the LOG_PACKET_HEADER to identify the type of each message in
 *          the binary log stream.
 *          
 *          ID Allocation Strategy:
 *          - **0-31**: Reserved for vehicle-specific messages (not in this enum)
 *          - **32-127**: Common messages shared by all vehicles
 *          - **128**: LOG_FORMAT_MSG (FMT) - must remain at 128
 *          - **129-254**: Additional common messages and module-specific messages
 *          - **255**: Reserved for future expansion
 *          
 *          Why LOG_FORMAT_MSG = 128?
 *          The FMT message is fixed at ID 128 for historical reasons and log parser
 *          compatibility. Many log analysis tools hardcode this value. Changing it would
 *          break all existing log parsers and tools.
 *          
 *          Message ID Assignment Rules:
 *          1. **Stability**: Once assigned, IDs should never change
 *          2. **Uniqueness**: Each message type must have a unique ID
 *          3. **Ordering**: Order in enum affects log file organization
 *          4. **Modules**: LOG_IDS_FROM_* placeholders reserve ranges for modules
 *          
 *          LOG_IDS_FROM_* Placeholders:
 *          These enum entries reserve ID ranges for external modules:
 *          - LOG_IDS_FROM_GPS: GPS module messages (AP_GPS/LogStructure.h)
 *          - LOG_IDS_FROM_NAVEKF2: EKF2 messages (AP_NavEKF2/LogStructure.h)
 *          - LOG_IDS_FROM_NAVEKF3: EKF3 messages (AP_NavEKF3/LogStructure.h)
 *          - LOG_IDS_FROM_BATTMONITOR: Battery messages (AP_BattMonitor/LogStructure.h)
 *          - LOG_IDS_FROM_INERTIALSENSOR: IMU messages (AP_InertialSensor/LogStructure.h)
 *          - And many more...
 *          
 *          Each placeholder expands to multiple enum entries defined in the respective
 *          module's LogStructure.h file. For example:
 *          @code
 *          // In AP_GPS/LogStructure.h:
 *          #define LOG_IDS_FROM_GPS \
 *              LOG_GPS_MSG, \
 *              LOG_GPA_MSG, \
 *              LOG_GPS2_MSG
 *          @endcode
 *          
 *          Duplicate Detection:
 *          The LOG_MODE_MSG is used as a boundary marker to detect duplicate message IDs.
 *          Messages should not be added between LOG_MODE_MSG and LOG_FORMAT_MSG as this
 *          range is used for validation. The static_assert at the end verifies LOG_MODE_MSG < 128.
 *          
 *          Adding New Common Messages:
 *          To add a new common log message:
 *          1. Add message struct definition (log_NewMessage)
 *          2. Add @LoggerMessage documentation comment
 *          3. Add enum entry here (LOG_NEWMESSAGE_MSG)
 *          4. Add entry to LOG_COMMON_STRUCTURES macro
 *          5. Ensure ID doesn't conflict with existing or reserved IDs
 *          6. Test that total messages < 255
 *          
 *          Example Message Flow:
 *          @code
 *          Writing a log message:
 *          1. Application calls: logger.Write_RCIN(channels)
 *          2. Logger looks up: LOG_RCIN_MSG = 33 (example value)
 *          3. Writes packet: [0xA3, 0x95, 33, <RCIN data>]
 *          4. Message written to log file/SD card
 *          
 *          Reading a log message:
 *          1. Parser reads packet: [0xA3, 0x95, 33, <data>]
 *          2. Recognizes: head1=0xA3, head2=0x95, msgid=33
 *          3. Looks up: msgid 33 = RCIN message
 *          4. Parses data according to RCIN format
 *          5. Extracts: TimeUS, C1, C2, ..., C14
 *          @endcode
 *          
 *          Message ID Stability:
 *          Message IDs must remain stable across firmware versions for log compatibility:
 *          - **Forward Compatibility**: New firmware can read old logs
 *          - **Backward Compatibility**: Old tools can read new logs (with graceful degradation)
 *          - **Version Tracking**: VER message identifies firmware version
 *          - **Format Messages**: FMT messages describe message layouts
 *          
 *          If a message format must change:
 *          1. **Preferred**: Add new fields at end (backward compatible)
 *          2. **Acceptable**: Increment message version (e.g., GPS → GPS2)
 *          3. **Avoid**: Changing existing field meanings or order
 * 
 * @note Total message count must not exceed 255 (uint8_t limit)
 * @note LOG_FORMAT_MSG fixed at 128 for parser compatibility
 * @note LOG_MODE_MSG used as duplicate detection boundary
 * @note _LOG_LAST_MSG_ sentinel must be < 255
 * 
 * @warning Never reuse message IDs - breaks log compatibility
 * @warning Never change LOG_FORMAT_MSG from 128
 * @warning Vehicle-specific messages (0-31) not included in this enum
 * 
 * @see LOG_COMMON_STRUCTURES for message structure definitions
 * @see LOG_PACKET_HEADER for message framing
 * @see AP_Logger::Write_* methods for message writing
 */
// message types for common messages
enum LogMessages : uint8_t {
    // ========== Common Message IDs (32-127) ==========
    // Range: 32-127 reserved for frequently-used common messages
    
    LOG_PARAMETER_MSG = 32,        ///< PARM: Parameter values with defaults
    LOG_IDS_FROM_NAVEKF2,          ///< EKF2: Extended Kalman Filter 2 messages (NKF1-NKF10)
    LOG_IDS_FROM_NAVEKF3,          ///< EKF3: Extended Kalman Filter 3 messages (XKF1-XKF10)
    LOG_MESSAGE_MSG,               ///< MSG: Text messages
    LOG_RCIN_MSG,                  ///< RCIN: RC input channels 1-14
    LOG_RCI2_MSG,                  ///< RCI2: RC input channels 15-16 plus override mask and flags
    LOG_RCOUT_MSG,                 ///< RCOU: Servo/motor output channels 1-14
    LOG_RSSI_MSG,                  ///< RSSI: RC receiver signal strength
    LOG_IDS_FROM_BARO,             ///< BARO: Barometer messages (BAR1, BAR2, etc.)
    LOG_IDS_FROM_CANMANAGER,       ///< CAN messages from CAN manager
    LOG_POWR_MSG,                  ///< POWR: System power monitoring (Vcc, servo rail voltage)
    LOG_MCU_MSG,                   ///< MCU: Microcontroller temperature and voltage
    LOG_IDS_FROM_AHRS,             ///< AHRS: Attitude Heading Reference System messages
    LOG_SIMSTATE_MSG,              ///< SIM: SITL simulator state
    LOG_MAVLINK_COMMAND_MSG,       ///< MAVC: MAVLink commands executed
    LOG_RADIO_MSG,                 ///< RAD: Telemetry radio statistics
    LOG_ATRP_MSG,                  ///< ATRP: Autopilot startup messages
    LOG_IDS_FROM_CAMERA,           ///< Camera messages (CAM, TRIG)
    LOG_IDS_FROM_MOUNT,            ///< Mount/gimbal messages (MNT)
    LOG_TERRAIN_MSG,               ///< TERR: Terrain database information
    LOG_IDS_FROM_SERVO_TELEM,      ///< Servo telemetry messages
    LOG_IDS_FROM_ESC_TELEM,        ///< ESC telemetry messages
    LOG_IDS_FROM_BATTMONITOR,      ///< Battery monitoring messages (BAT, BCL, etc.)
    LOG_IDS_FROM_HAL_CHIBIOS,      ///< ChibiOS HAL-specific messages
    LOG_IDS_FROM_MISSION,          ///< Mission waypoint messages (WPT)

    LOG_IDS_FROM_GPS,              ///< GPS messages (GPS, GPA, GPS2, etc.)

    // PID controller messages for different control axes
    LOG_PIDR_MSG,                  ///< PIDR: Roll rate PID controller
    LOG_PIDP_MSG,                  ///< PIDP: Pitch rate PID controller
    LOG_PIDY_MSG,                  ///< PIDY: Yaw rate PID controller
    LOG_PIDA_MSG,                  ///< PIDA: Vertical acceleration PID controller
    LOG_PIDS_MSG,                  ///< PIDS: Steering (ground vehicle yaw rate) PID controller
    LOG_PIDN_MSG,                  ///< PIDN: North velocity PID controller
    LOG_PIDE_MSG,                  ///< PIDE: East velocity PID controller
    LOG_IDS_FROM_LANDING,          ///< Landing system messages
    LOG_MAG_MSG,                   ///< MAG: Magnetometer/compass data
    LOG_ARSP_MSG,                  ///< ARSP: Airspeed sensor data
    LOG_IDS_FROM_RPM,              ///< RPM: Engine RPM sensor data
    LOG_RFND_MSG,                  ///< RFND: Rangefinder distance sensor data
    LOG_DMS_MSG,                   ///< DMS: DataFlash-over-MAVLink statistics
    LOG_FORMAT_UNITS_MSG,          ///< FMTU: Format units and multipliers metadata
    LOG_UNIT_MSG,                  ///< UNIT: Unit definition metadata
    LOG_MULT_MSG,                  ///< MULT: Multiplier definition metadata
    LOG_RALLY_MSG,                 ///< RALY: Rally point information

    /**
     * @brief Duplicate detection boundary marker
     * 
     * @details LOG_MODE_MSG serves as a boundary in the enum to detect duplicate message IDs.
     *          The static_assert at the end of this enum verifies that LOG_MODE_MSG < 128,
     *          ensuring no common messages accidentally use IDs >= 128 before LOG_FORMAT_MSG.
     *          
     *          DO NOT ADD MESSAGES BETWEEN LOG_MODE_MSG AND LOG_FORMAT_MSG.
     *          
     *          This gap (LOG_MODE_MSG to 127) is intentionally left empty for duplicate detection
     *          and to maintain LOG_FORMAT_MSG at exactly 128.
     */
    // LOG_MODE_MSG is used as a check for duplicates. Do not add between this and LOG_FORMAT_MSG
    LOG_MODE_MSG,                  ///< MODE: Vehicle flight mode changes

    // ========== LOG_FORMAT_MSG - MUST REMAIN AT 128 ==========
    /**
     * @brief Format message type - FIXED at 128 for parser compatibility
     * 
     * @details LOG_FORMAT_MSG is hardcoded to 128 for historical reasons and must never change.
     *          The FMT message defines the structure of all other log messages, making logs
     *          self-describing. Log parsers worldwide expect FMT messages at ID 128.
     *          
     *          Changing this value would break:
     *          - All existing log analysis tools (Mission Planner, MAVExplorer, etc.)
     *          - Log conversion utilities
     *          - Custom log parsers and analysis scripts
     *          - Historical log compatibility
     *          
     *          The FMT message is always the first message written to a new log file, allowing
     *          parsers to understand the format of all subsequent messages without hardcoded
     *          message definitions.
     */
    LOG_FORMAT_MSG = 128,          ///< FMT: Message format definition (FIXED AT 128 - NEVER CHANGE)

    // ========== Additional Common Messages (129-254) ==========
    // Range: 129-254 for additional common messages and module-specific messages
    
    LOG_IDS_FROM_DAL,              ///< DAL: Data Abstraction Layer messages (for log replay)
    LOG_IDS_FROM_INERTIALSENSOR,   ///< IMU messages (IMU, IMT, IST, etc.)

    LOG_IDS_FROM_VISUALODOM,       ///< Visual odometry messages
    LOG_IDS_FROM_AVOIDANCE,        ///< Obstacle avoidance messages
    LOG_IDS_FROM_BEACON,           ///< Beacon positioning messages
    LOG_IDS_FROM_PROXIMITY,        ///< Proximity sensor messages
    LOG_DF_FILE_STATS,             ///< DSF: Onboard logging statistics
    LOG_SRTL_MSG,                  ///< SRTL: SmartRTL (Smart Return To Launch) statistics
    LOG_PERFORMANCE_MSG,           ///< PM: System performance monitoring
    LOG_OPTFLOW_MSG,               ///< OF: Optical flow sensor data
    LOG_EVENT_MSG,                 ///< EV: Coded event messages
    LOG_WHEELENCODER_MSG,          ///< WENC: Wheel encoder data (ground vehicles)
    LOG_MAV_MSG,                   ///< MAV: MAVLink link statistics per channel
    LOG_ERROR_MSG,                 ///< ERR: Coded error messages
    LOG_ADSB_MSG,                  ///< ADSB: ADS-B aircraft detection data
    LOG_ARM_DISARM_MSG,            ///< ARM: Arming/disarming events
    LOG_WINCH_MSG,                 ///< WINC: Winch system status
    LOG_IDS_FROM_AC_ATTITUDECONTROL, ///< Attitude controller messages
    LOG_IDS_FROM_PRECLAND,         ///< Precision landing messages
    LOG_IDS_FROM_AIS,              ///< AIS (maritime vessel tracking) messages
    LOG_STAK_MSG,                  ///< STAK: RTOS thread stack usage monitoring
    LOG_FILE_MSG,                  ///< FILE: Embedded file data
    LOG_SCRIPTING_MSG,             ///< SCR: Lua scripting runtime statistics
    LOG_VIDEO_STABILISATION_MSG,   ///< Video stabilization messages
    LOG_MOTBATT_MSG,               ///< MOTB: Motor mixer and battery compensation
    LOG_VER_MSG,                   ///< VER: Firmware version and board identification
    LOG_RCOUT2_MSG,                ///< RCO2: Servo/motor output channels 15-18
    LOG_RCOUT3_MSG,                ///< RCO3: Servo/motor output channels 19-32
    LOG_IDS_FROM_FENCE,            ///< Geofence messages
    LOG_IDS_FROM_HAL,              ///< HAL (Hardware Abstraction Layer) messages

    /**
     * @brief Sentinel value marking end of message ID enum
     * 
     * @details _LOG_LAST_MSG_ is not a real message type, but a sentinel value used to:
     *          - Determine the total number of defined message types
     *          - Validate that total messages < 255 (uint8_t limit)
     *          - Detect accidental enum overflows
     *          
     *          The static_assert after this enum verifies _LOG_LAST_MSG_ < 255.
     *          
     *          Usage:
     *          @code
     *          const uint8_t num_message_types = _LOG_LAST_MSG_;
     *          @endcode
     */
    _LOG_LAST_MSG_                 ///< Sentinel: Total message count (not a real message type)
};

/**
 * @brief Message ID #255 reserved for future expansion
 * 
 * @details Message type ID 255 is explicitly reserved and not assigned to any message.
 *          This reservation provides flexibility for future protocol extensions without
 *          breaking existing message numbering. Potential future uses:
 *          - Extended message format indicator (messages > 255 types)
 *          - Special control/escape sequences
 *          - Backward compatibility markers
 *          - Format version indicators
 */
// we reserve ID #255 for future expansion

/**
 * @brief Compile-time assertion: Total message count must not exceed 255
 * 
 * @details This static_assert ensures that the total number of log message types
 *          (common + vehicle-specific + module-specific) does not exceed 255, which
 *          is the maximum value representable by the uint8_t message ID field.
 *          
 *          Why This Matters:
 *          - Message IDs stored as uint8_t in LOG_PACKET_HEADER
 *          - Maximum representable value: 255
 *          - ID 255 reserved for future use
 *          - Effective limit: 254 message types
 *          
 *          If This Assertion Fails:
 *          "Too many message formats" error at compile time indicates:
 *          - Too many common messages defined
 *          - Too many module messages via LOG_IDS_FROM_* macros
 *          - Need to deprecate/remove unused message types
 *          - Or consider protocol extension for >255 message types
 *          
 *          Resolution Strategies:
 *          1. Remove obsolete/unused message types
 *          2. Combine related messages into single messages with flags
 *          3. Move vehicle-specific messages to optional modules
 *          4. Design extended format protocol (major undertaking)
 * 
 * @note This check includes all messages: common (this file) + vehicle-specific + modules
 * @warning Exceeding 255 messages requires protocol redesign
 */
static_assert(_LOG_LAST_MSG_ < 255, "Too many message formats");

/**
 * @brief Compile-time assertion: LOG_MODE_MSG must be below LOG_FORMAT_MSG
 * 
 * @details This static_assert verifies that LOG_MODE_MSG (the duplicate detection boundary)
 *          is assigned an ID less than 128, ensuring it falls in the common message range
 *          and not in the post-LOG_FORMAT_MSG range.
 *          
 *          Purpose of This Check:
 *          - Verify LOG_MODE_MSG is in correct range (32-127)
 *          - Detect if messages accidentally assigned duplicate IDs
 *          - Ensure gap between LOG_MODE_MSG and LOG_FORMAT_MSG is empty
 *          - Maintain LOG_FORMAT_MSG at exactly 128
 *          
 *          The Gap (LOG_MODE_MSG to 127):
 *          The enum range from LOG_MODE_MSG to 127 should be empty. If any messages are
 *          accidentally inserted here, it indicates:
 *          - Enum ordering problems
 *          - Duplicate ID assignments
 *          - Violation of ID allocation rules
 *          
 *          If This Assertion Fails:
 *          "Duplicate message format IDs" error indicates:
 *          - Messages added between LOG_MODE_MSG and LOG_FORMAT_MSG (forbidden)
 *          - LOG_MODE_MSG pushed past 127 (too many messages before it)
 *          - Enum value collision/duplication
 *          
 *          Resolution:
 *          1. Check for messages between LOG_MODE_MSG and LOG_FORMAT_MSG - remove them
 *          2. Verify no duplicate explicit ID assignments (e.g., two enums set to same value)
 *          3. Ensure LOG_FORMAT_MSG remains at 128
 *          4. Review LOG_IDS_FROM_* macro expansions for conflicts
 * 
 * @note This is a sanity check for enum ordering and duplicate detection
 * @warning Never add messages between LOG_MODE_MSG and LOG_FORMAT_MSG
 */
static_assert(LOG_MODE_MSG < 128, "Duplicate message format IDs");
