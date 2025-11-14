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
 * @file AP_GPS_SIRF.h
 * @brief GPS driver for SiRF chipsets using SiRF binary protocol
 * 
 * @details This driver implements the SiRF binary protocol for legacy SiRF GPS modules.
 *          The SiRF binary protocol was used in older SiRF Star II, Star III, and Star IV
 *          GPS chipsets. This driver parses the following SiRF binary messages:
 *          - Message 0x02: Measured Navigation Data (ECEF position and velocity)
 *          - Message 0x07: Clock Status Data
 *          - Message 0x29: Geodetic Navigation Data (WGS84 lat/lon/alt)
 * 
 *          The SiRF binary protocol uses a packet structure with:
 *          - Start sequence: 0xA0 0xA2
 *          - Payload length: 16-bit big-endian
 *          - Message ID: 8-bit
 *          - Payload data: variable length
 *          - Checksum: 15-bit sum over payload bytes
 *          - End sequence: 0xB0 0xB3
 * 
 *          Position data is provided in WGS84 geodetic coordinates (latitude, longitude, altitude).
 *          Velocity data is provided in ECEF (Earth-Centered Earth-Fixed) format and converted
 *          to NED (North-East-Down) frame for use by the autopilot.
 * 
 * @note The SiRF binary protocol is largely obsolete. Most modern SiRF-based receivers
 *       support NMEA protocol or proprietary high-performance protocols. This driver is
 *       maintained for compatibility with legacy hardware.
 * 
 * @note Most SiRF receivers can be configured to output either NMEA or binary protocol.
 *       The SIRF_SET_BINARY command can be used to switch the receiver to binary mode.
 * 
 * @author Michael Smith (original implementation)
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 * 
 * Source: libraries/AP_GPS/AP_GPS_SIRF.h
 */
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>

#include "AP_GPS.h"
#include "GPS_Backend.h"

#if AP_GPS_SIRF_ENABLED

/**
 * @def SIRF_SET_BINARY
 * @brief NMEA command string to configure SiRF receiver to binary protocol mode
 * 
 * @details This PSRF100 command configures the SiRF receiver to switch from NMEA output
 *          to SiRF binary protocol at 38400 baud, 8 data bits, 1 stop bit, no parity.
 *          
 *          Command format: $PSRF100,protocol,baud,databits,stopbits,parity*checksum
 *          - protocol: 0 = SiRF binary
 *          - baud: 38400 bps
 *          - databits: 8
 *          - stopbits: 1
 *          - parity: 0 (none)
 * 
 * @note This command must be sent while the receiver is in NMEA mode.
 */
#define SIRF_SET_BINARY "$PSRF100,0,38400,8,1,0*3C\r\n"

/**
 * @class AP_GPS_SIRF
 * @brief GPS driver for SiRF chipsets using SiRF binary protocol
 * 
 * @details This class implements a GPS backend for SiRF binary protocol receivers.
 *          It handles the parsing of SiRF binary messages and extracts position,
 *          velocity, timing, and satellite information.
 * 
 *          The driver implements a state machine parser that:
 *          1. Searches for start sequence (0xA0 0xA2)
 *          2. Reads payload length
 *          3. Reads message ID
 *          4. Accumulates payload bytes
 *          5. Verifies checksum
 *          6. Confirms end sequence (0xB0 0xB3)
 * 
 *          Primary message processed is Geodetic Navigation Data (0x29) which provides:
 *          - 3D position fix (WGS84 lat/lon/alt)
 *          - Ground speed and course
 *          - Climb rate
 *          - Number of satellites used
 *          - HDOP (Horizontal Dilution of Precision)
 *          - Position and velocity error estimates
 * 
 *          This is a legacy driver for older SiRF chipsets. Modern GPS receivers
 *          typically use NMEA, UBX (u-blox), or other proprietary protocols.
 * 
 * @note SiRF binary protocol does not support RTK (Real-Time Kinematic) corrections.
 *       Maximum fix status is GPS_OK_FIX_3D.
 * 
 * @see GPS_Backend for base class interface
 * @see AP_GPS for GPS subsystem architecture
 * 
 * Source: libraries/AP_GPS/AP_GPS_SIRF.h, libraries/AP_GPS/AP_GPS_SIRF.cpp
 */
class AP_GPS_SIRF : public AP_GPS_Backend {
public:
    /**
     * @brief Constructor for SiRF binary protocol GPS driver
     * 
     * @details Initializes the SiRF GPS backend with references to the GPS subsystem,
     *          configuration parameters, state structure, and UART port for communication.
     *          Sets up the state machine for binary protocol parsing.
     * 
     * @param[in] _gps Reference to main AP_GPS object
     * @param[in] _params GPS configuration parameters
     * @param[in] _state GPS state structure to populate with fix data
     * @param[in] _port UART port connected to the SiRF GPS receiver
     * 
     * @note The UART port should be configured to 38400 baud, 8N1 for SiRF binary protocol.
     */
	AP_GPS_SIRF(AP_GPS &_gps, AP_GPS::Params &_params, AP_GPS::GPS_State &_state, AP_HAL::UARTDriver *_port);

    /**
     * @brief Read and parse data from the SiRF GPS receiver
     * 
     * @details This method reads available bytes from the UART port and processes them
     *          through the SiRF binary protocol state machine. It searches for valid
     *          message frames, verifies checksums, and extracts navigation data.
     * 
     *          The state machine handles:
     *          - Start sequence detection (0xA0 0xA2)
     *          - Payload length extraction
     *          - Message ID identification
     *          - Payload accumulation with checksum calculation
     *          - End sequence verification (0xB0 0xB3)
     * 
     *          When a complete valid message is received, it updates the GPS state
     *          with new position, velocity, timing, and satellite information.
     * 
     * @return true if a valid GPS message was successfully parsed and processed
     * @return false if no complete message was processed (incomplete data or invalid message)
     * 
     * @note This method should be called regularly (typically at main loop rate) to
     *       process incoming GPS data. Multiple calls may be needed to parse a complete message.
     * 
     * @see GPS_Backend::read()
     */
    bool read() override;

    /**
     * @brief Detect SiRF binary protocol in a stream of GPS data
     * 
     * @details Static detection function used by the GPS subsystem to identify SiRF
     *          binary protocol receivers. Examines incoming bytes to detect the
     *          characteristic start sequence and message structure of SiRF binary protocol.
     * 
     *          This function is called during GPS auto-detection when the GPS type is
     *          set to AUTO. It maintains detection state across multiple calls until
     *          enough data has been examined to confirm or reject SiRF protocol.
     * 
     * @param[in,out] state Detection state structure maintaining progress through detection algorithm
     * @param[in] data Single byte of GPS data to examine
     * 
     * @return true if SiRF binary protocol is positively detected
     * @return false if detection is not yet complete or protocol is not SiRF binary
     * 
     * @note This is a static method called during GPS initialization before the driver is instantiated.
     */
	static bool _detect(struct SIRF_detect_state &state, uint8_t data);

    /**
     * @brief Get the name identifier for this GPS driver
     * 
     * @details Returns a string identifier used for logging and driver identification.
     * 
     * @return "SIRF" string constant identifying the SiRF binary protocol driver
     * 
     * @see GPS_Backend::name()
     */
    const char *name() const override { return "SIRF"; }

private:
    /**
     * @struct sirf_geonav
     * @brief SiRF binary protocol Message 0x29 - Geodetic Navigation Data structure
     * 
     * @details This structure defines the layout of the SiRF Geodetic Navigation Data message (0x29),
     *          which provides comprehensive navigation solution information including:
     *          - 3D position in WGS84 geodetic coordinates (latitude, longitude, altitude)
     *          - GPS time and date
     *          - Ground speed, course, and climb rate
     *          - Navigation accuracy estimates (HDOP, position error, velocity error)
     *          - Satellite information and fix quality indicators
     * 
     *          All multi-byte values are transmitted in big-endian (network byte order).
     *          The structure is packed to match the exact binary layout of the SiRF message.
     * 
     * @note Coordinate system: WGS84 geodetic (lat/lon in degrees * 1e7, altitude in meters * 100)
     * @note Velocity: Ground speed in m/s * 100, course in degrees * 100
     * @note Time: GPS week, time of week in milliseconds, UTC date/time
     * 
     * Source: SiRF Binary Protocol Reference Manual
     */
    struct PACKED sirf_geonav {
        uint16_t fix_invalid;              ///< Fix invalid flag: 0 = valid, non-zero = invalid
        uint16_t fix_type;                 ///< Fix type: bits 0-2 define fix mode (see sirf_fix_type enum)
        uint16_t week;                     ///< GPS week number
        uint32_t time;                     ///< GPS time of week in milliseconds
        uint16_t year;                     ///< UTC year
        uint8_t month;                     ///< UTC month (1-12)
        uint8_t day;                       ///< UTC day of month (1-31)
        uint8_t hour;                      ///< UTC hour (0-23)
        uint8_t minute;                    ///< UTC minute (0-59)
        uint16_t second;                   ///< UTC second * 1000 (milliseconds)
        uint32_t satellites_used;          ///< Satellite ID bitmap of satellites used in solution
        int32_t latitude;                  ///< Latitude in degrees * 1e7 (WGS84)
        int32_t longitude;                 ///< Longitude in degrees * 1e7 (WGS84)
        int32_t altitude_ellipsoid;        ///< Altitude above WGS84 ellipsoid in meters * 100
        int32_t altitude_msl;              ///< Altitude above mean sea level in meters * 100
        int8_t map_datum;                  ///< Map datum (typically WGS84 = 21)
        int16_t ground_speed;              ///< Ground speed in m/s * 100
        int16_t ground_course;             ///< Ground course in degrees * 100 (0-360)
        int16_t res1;                      ///< Reserved field
        int16_t climb_rate;                ///< Vertical velocity (climb rate) in m/s * 100
        uint16_t heading_rate;             ///< Heading rate (turn rate) in degrees/s * 100
        uint32_t horizontal_position_error;///< Estimated horizontal position error in meters * 100
        uint32_t vertical_position_error;  ///< Estimated vertical position error in meters * 100
        uint32_t time_error;               ///< Estimated time error in milliseconds
        int16_t horizontal_velocity_error; ///< Estimated horizontal velocity error in m/s * 100
        int32_t clock_bias;                ///< GPS receiver clock bias in meters * 100
        uint32_t clock_bias_error;         ///< Estimated clock bias error in meters * 100
        int32_t clock_drift;               ///< GPS receiver clock drift in m/s * 100
        uint32_t clock_drift_error;        ///< Estimated clock drift error in m/s * 100
        uint32_t distance;                 ///< Distance traveled since reset in meters
        uint16_t distance_error;           ///< Estimated distance error in meters
        uint16_t heading_error;            ///< Estimated heading error in degrees * 100
        uint8_t satellites;                ///< Number of satellites in view
        uint8_t hdop;                      ///< Horizontal Dilution of Precision * 5
        uint8_t mode_info;                 ///< Additional mode information flags
    };
    
    /**
     * @enum sirf_protocol_bytes
     * @brief SiRF binary protocol framing and message identifier constants
     * 
     * @details Defines the special byte sequences and message IDs used in SiRF binary protocol.
     *          Every SiRF binary message is framed with a start sequence (preamble) and
     *          end sequence (postamble) to enable message boundary detection.
     * 
     *          Message structure:
     *          [PREAMBLE1][PREAMBLE2][Length MSB][Length LSB][Message ID][Payload...][Checksum MSB][Checksum LSB][POSTAMBLE1][POSTAMBLE2]
     */
    enum sirf_protocol_bytes {
        PREAMBLE1 = 0xa0,   ///< Start sequence byte 1 (0xA0)
        PREAMBLE2 = 0xa2,   ///< Start sequence byte 2 (0xA2)
        POSTAMBLE1 = 0xb0,  ///< End sequence byte 1 (0xB0)
        POSTAMBLE2 = 0xb3,  ///< End sequence byte 2 (0xB3)
        MSG_GEONAV = 0x29   ///< Message ID for Geodetic Navigation Data (0x29 = 41 decimal)
    };
    
    /**
     * @enum sirf_fix_type
     * @brief SiRF GPS fix type indicators
     * 
     * @details Defines bit masks and values for interpreting the fix_type field in
     *          SiRF navigation messages. The lower 3 bits (masked by FIX_MASK) indicate
     *          the type of position fix:
     *          - 0x0 = No fix
     *          - 0x1 = 1 satellite solution (invalid for navigation)
     *          - 0x2 = 2 satellite solution (invalid for navigation)
     *          - 0x3 = 3 satellite solution (2D fix)
     *          - 0x4 = 4+ satellite solution (3D fix)
     *          - 0x6 = 3D fix with DGPS corrections (FIX_3D)
     * 
     * @note SiRF protocol does not support RTK corrections. Maximum fix quality is 3D DGPS.
     */
    enum sirf_fix_type {
        FIX_3D = 0x6,       ///< 3D fix value (with or without DGPS)
        FIX_MASK = 0x7      ///< Bit mask to extract fix type from fix_type field (lower 3 bits)
    };

    // =========================================================================
    // State Machine Variables
    // =========================================================================
    
    /**
     * @brief Current state of the protocol parser state machine
     * 
     * @details Tracks the parser's position within the message frame:
     *          0 = Searching for start sequence
     *          1 = Found PREAMBLE1, expecting PREAMBLE2
     *          2 = Reading payload length MSB
     *          3 = Reading payload length LSB
     *          4 = Reading message ID
     *          5 = Reading payload bytes
     *          6 = Reading checksum MSB
     *          7 = Reading checksum LSB
     *          8 = Reading POSTAMBLE1
     *          9 = Reading POSTAMBLE2
     */
    uint8_t         _step;
    
    /**
     * @brief Running checksum accumulator for current message
     * 
     * @details SiRF binary protocol uses a 15-bit checksum calculated as the sum
     *          of all payload bytes (message ID + payload data). This variable
     *          accumulates the checksum as payload bytes are received.
     */
    uint16_t        _checksum;
    
    /**
     * @brief Flag indicating whether to accumulate payload bytes into buffer
     * 
     * @details Set to true when processing messages that need their payload
     *          stored in _buffer (e.g., MSG_GEONAV). Set to false for messages
     *          that are ignored or processed inline.
     */
    bool            _gather;
    
    /**
     * @brief Expected length of the current message payload in bytes
     * 
     * @details Extracted from the 16-bit length field at the start of each
     *          SiRF binary message (after preamble). Includes message ID and
     *          all payload bytes, but not preamble, postamble, or checksum.
     */
    uint16_t        _payload_length;
    
    /**
     * @brief Number of payload bytes received for the current message
     * 
     * @details Increments as each payload byte is received. When _payload_counter
     *          reaches _payload_length, the payload is complete and the parser
     *          expects the checksum bytes.
     */
    uint16_t        _payload_counter;
    
    /**
     * @brief Message ID of the current message being parsed
     * 
     * @details Extracted from the first payload byte. Used to determine message
     *          type and whether to gather payload into buffer. Key message IDs:
     *          - 0x02: Measured Navigation Data (ECEF)
     *          - 0x07: Clock Status Data
     *          - 0x29: Geodetic Navigation Data (primary message)
     */
    uint8_t         _msg_id;

    // =========================================================================
    // Message Buffer
    // =========================================================================
    
    /**
     * @brief Union buffer for storing parsed SiRF message payloads
     * 
     * @details Provides byte-level access for message parsing and structured
     *          access for interpreting complete messages. The nav member provides
     *          structured access to Geodetic Navigation Data (message 0x29).
     * 
     * @note DEFINE_BYTE_ARRAY_METHODS macro provides byte array access methods
     *       for the union, enabling byte-by-byte payload accumulation.
     */
    union {
        DEFINE_BYTE_ARRAY_METHODS
        sirf_geonav nav;  ///< Structured access to Geodetic Navigation Data (message 0x29)
    } _buffer;

    // =========================================================================
    // Private Methods
    // =========================================================================
    
    /**
     * @brief Parse a complete SiRF GPS message and update GPS state
     * 
     * @details Called when a complete message with valid checksum has been received.
     *          Extracts navigation data from the message buffer and updates the GPS
     *          state structure with position, velocity, time, and satellite information.
     * 
     *          For Geodetic Navigation Data (0x29), this method:
     *          - Converts WGS84 lat/lon/alt to ArduPilot's internal format
     *          - Converts ground speed and course to NED velocity vector
     *          - Extracts satellite count and HDOP
     *          - Determines fix type and quality
     *          - Updates GPS time and date
     * 
     * @return true if the message was successfully parsed and state updated
     * @return false if the message could not be parsed or contained invalid data
     * 
     * @note This method performs coordinate system conversions from WGS84 geodetic
     *       to NED (North-East-Down) frame used by the autopilot.
     */
    bool        _parse_gps(void);
    
    /**
     * @brief Accumulate a payload byte into the checksum and message buffer
     * 
     * @details Adds a payload byte to the running checksum calculation and,
     *          if _gather is true, stores the byte in the message buffer at
     *          the current payload counter position.
     * 
     * @param[in] val Payload byte to accumulate
     * 
     * @note Checksum is calculated as the 15-bit sum of all payload bytes
     *       (message ID + payload data), transmitted as a 16-bit value.
     */
    void        _accumulate(uint8_t val);

    /**
     * @brief Initialization command sequence for SiRF receivers
     * 
     * @details Contains a sequence of SiRF binary commands sent to the GPS receiver
     *          during initialization to configure message rates, navigation parameters,
     *          and output format.
     * 
     * @note The initialization blob is specific to SiRF binary protocol and configures
     *       the receiver to output navigation messages at appropriate rates for autopilot use.
     */
    static const uint8_t _initialisation_blob[];
};
#endif
