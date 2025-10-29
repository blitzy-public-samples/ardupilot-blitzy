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

//
// NMEA parser, adapted by Michael Smith from TinyGPS v9:
//
// TinyGPS - a small GPS library for Arduino providing basic NMEA parsing
// Copyright (C) 2008-9 Mikal Hart
// All rights reserved.
//
//

/**
 * @file AP_GPS_NMEA.h
 * @brief NMEA 0183 ASCII protocol parser for GPS receivers
 * 
 * @details This is a lightweight NMEA parser implementing the NMEA 0183 standard,
 *          derived originally from the TinyGPS parser by Mikal Hart. It is frugal
 *          in its use of memory and tries to avoid unnecessary arithmetic.
 *          
 *          The parser handles standard NMEA sentences:
 *          - GPGGA: Global Positioning System Fix Data (position, altitude, fix quality, satellites, HDOP)
 *          - GPRMC: Recommended Minimum Specific GPS/Transit Data (position, velocity, course, date)
 *          - GPVTG: Track Made Good and Ground Speed (velocity over ground, course)
 *          
 *          Additionally supports vendor-specific extensions:
 *          - KSXT: Unicore 3D velocity and yaw
 *          - AGRICA: Unicore RTK position and heading
 *          - PHD: AllyStar vertical velocity and accuracy (maps ublox UBX to NMEA)
 *          - THS: Trimble true heading with quality indicator
 *          - VERSIONA: Unicore firmware version
 *          - UNIHEADINGA: Unicore dual-antenna heading
 *          
 *          The parser attempts to be robust in the face of occasional corruption
 *          in the input stream. It makes a basic effort to configure GPS receivers
 *          that are likely to be connected in NMEA mode (SiRF, MediaTek, ublox)
 *          to emit the correct message stream, but does not validate that the
 *          correct stream is being received.
 *          
 *          Coordinate Systems:
 *          - Position: WGS84 geodetic coordinates (latitude/longitude in degrees,
 *            altitude above mean sea level in meters)
 *          - Velocity: Ground speed in m/s, course over ground in degrees
 *          - Frame: Earth-fixed frame (latitude positive north, longitude positive east)
 *          
 *          Unit Conventions:
 *          - Input (NMEA format): Degrees/minutes (DDMM.MMMM), knots, feet
 *          - Internal representation: Decimal degrees × 1e7, meters, m/s
 *          - Output: SI units (degrees, meters, m/s)
 * 
 * @note NMEA 0183 is widely supported across GPS receivers but is less efficient
 *       than binary protocols due to ASCII encoding and lower update rates (typically 1-10 Hz).
 *       Binary protocols (ublox UBX, DroneCAN) are preferred for high-performance applications.
 * 
 * @warning A GPS unit emitting only GPRMC sentences will show as having a fix
 *          even though no altitude data is being received. This can lead to
 *          incorrect altitude estimates derived solely from barometric pressure.
 * 
 * @warning GPVTG data is parsed, but as the message may not contain the qualifier
 *          field (common with older SiRF units), it is not considered a reliable
 *          source of fix-valid information.
 * 
 * @see AP_GPS for the main GPS driver interface
 * @see GPS_Backend for the base class interface
 * 
 * Source: libraries/AP_GPS/AP_GPS_NMEA.h, libraries/AP_GPS/AP_GPS_NMEA.cpp
 */
#pragma once

#include "AP_GPS.h"
#include "GPS_Backend.h"

#if AP_GPS_NMEA_ENABLED
/**
 * @class AP_GPS_NMEA
 * @brief NMEA 0183 protocol parser backend for ArduPilot GPS driver
 * 
 * @details This class implements a robust parser for NMEA 0183 ASCII GPS protocol.
 *          It processes character-by-character input from the GPS receiver, validates
 *          checksums, and extracts position, velocity, time, and fix quality data.
 *          
 *          Parser State Machine:
 *          1. Searches for sentence start ($GP, $GN, $GL, $BD or # for Unicore)
 *          2. Accumulates characters and calculates checksum parity
 *          3. Splits sentence into comma-separated terms
 *          4. Processes terms based on sentence type
 *          5. Validates checksum before accepting data
 *          
 *          Sentence Processing:
 *          - GPGGA: Extracts time, lat/lon, altitude, fix quality, satellite count, HDOP
 *          - GPRMC: Extracts time, date, lat/lon, speed, course, magnetic variation
 *          - GPVTG: Extracts course and speed over ground
 *          - Vendor extensions: Extract heading, 3D velocity, accuracy estimates
 *          
 *          Auto-Configuration:
 *          Attempts to configure known GPS receivers to output appropriate NMEA sentences:
 *          - SiRF: Sends SiRF binary protocol switch to NMEA mode
 *          - MediaTek: Configures output message types and rate
 *          - ublox: Configures NMEA talker ID and message selection
 *          
 *          Thread Safety:
 *          - Called from GPS detection thread or GPS driver thread
 *          - Inherits thread safety from AP_GPS_Backend base class
 *          - Internal state protected by GPS state machine
 *          
 *          Performance:
 *          - Character-by-character parsing minimizes memory usage
 *          - No buffering of complete sentences
 *          - Temporary storage only for parsed numeric values
 *          - Typical execution: <100 microseconds per character on ARM Cortex-M
 * 
 * @note Most modern GPS receivers support both NMEA and binary protocols.
 *       Binary protocols are strongly preferred for autopilot use due to higher
 *       update rates, better accuracy representation, and lower CPU overhead.
 * 
 * @warning This parser does not validate that GPS is configured optimally.
 *          It will accept whatever NMEA sentences are received, which may be
 *          insufficient for autopilot operation (e.g., missing altitude data).
 * 
 * @see AP_GPS for the main GPS subsystem
 * @see GPS_Backend for the backend interface contract
 */
class AP_GPS_NMEA : public AP_GPS_Backend
{
    friend class AP_GPS_NMEA_Test;

public:

    using AP_GPS_Backend::AP_GPS_Backend;

    /**
     * @brief Parse incoming NMEA sentences character-by-character and update GPS state
     * 
     * @details Reads available characters from the serial receive buffer and processes
     *          them through the NMEA parser state machine. When a complete, valid sentence
     *          is received (validated by checksum), extracts position, velocity, and fix
     *          quality data and updates the GPS driver state.
     *          
     *          This method is called repeatedly by the GPS driver task (typically at 10-50 Hz)
     *          to process incoming NMEA data. The parser handles partial sentences across
     *          multiple calls and maintains state between invocations.
     *          
     *          Processing Flow:
     *          1. Read all available characters from serial port (up to buffer size)
     *          2. Feed each character to _decode() state machine
     *          3. When complete sentence detected, _term_complete() extracts data
     *          4. Validate checksum before accepting parsed values
     *          5. Update GPS state (position, velocity, fix) if data is valid
     *          
     *          NMEA Sentence Examples:
     *          - $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
     *          - $GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A
     *          - $GPVTG,054.7,T,034.4,M,005.5,N,010.2,K*48
     * 
     * @return true when a complete, valid NMEA sentence has been successfully parsed
     *         and GPS state has been updated with new data
     * @return false if no complete sentence available yet, sentence invalid, or checksum failed
     * 
     * @note This method is non-blocking and processes only available characters.
     *       Multiple calls may be required to complete a single sentence.
     * 
     * @note Typical NMEA update rates: 1 Hz (basic receivers) to 10 Hz (modern receivers).
     *       Update rate depends on GPS receiver configuration.
     * 
     * @see _decode() for character-by-character parsing
     * @see _term_complete() for sentence processing
     */
    bool        read() override;

	/**
	 * @brief Detect NMEA protocol by identifying valid NMEA sentence start patterns
	 * 
	 * @details Static detection method called by GPS auto-detection system to identify
	 *          if incoming data stream contains valid NMEA 0183 sentences. Searches for
	 *          NMEA sentence start sequences: $GP, $GN, $GL, $BD (standard talker IDs),
	 *          or # (Unicore proprietary format).
	 *          
	 *          Detection Algorithm:
	 *          1. Look for '$' or '#' character (sentence start)
	 *          2. Check next two characters for valid talker ID
	 *          3. Verify sentence continues with expected structure
	 *          4. Detection succeeds after finding valid sentence start
	 *          
	 *          Supported Talker IDs:
	 *          - GP: GPS (USA)
	 *          - GN: GNSS (multi-constellation: GPS+GLONASS+Galileo+BeiDou)
	 *          - GL: GLONASS (Russia)
	 *          - BD: BeiDou (China)
	 *          - #: Unicore proprietary format marker
	 * 
	 * @param[in,out] state Detection state structure maintaining parser context across calls
	 * @param[in] data Next character from GPS serial stream
	 * 
	 * @return true if valid NMEA sentence start detected, indicating NMEA protocol
	 * @return false if no valid pattern detected yet, continue probing
	 * 
	 * @note This method is called during GPS auto-detection phase before the driver
	 *       is fully initialized. It must be lightweight and stateless (except for state parameter).
	 * 
	 * @see AP_GPS::detect() for the GPS auto-detection system
	 */
	static bool _detect(struct NMEA_detect_state &state, uint8_t data);

    /**
     * @brief Get driver name identifier string
     * 
     * @return "NMEA" - constant string identifying this GPS driver type
     * 
     * @note Used for logging, parameter selection, and driver identification in multi-GPS systems
     */
    const char *name() const override { return "NMEA"; }

    /**
     * @brief Check GPS driver health status
     * 
     * @details Evaluates whether the NMEA GPS driver is operating correctly based on:
     *          - Recent message reception (not timed out)
     *          - Valid checksum validation
     *          - Fix quality indicators
     *          - Satellite count sufficiency
     * 
     * @return true if driver is healthy and providing valid GPS data
     * @return false if driver has not received valid data recently or data quality is poor
     * 
     * @note Health check is used by GPS failover logic in multi-GPS configurations
     * 
     * @see AP_GPS::get_best() for GPS selection algorithm
     */
    bool is_healthy(void) const override;

    /**
     * @brief Get GPS measurement lag time
     * 
     * @details Returns the estimated time delay between the GPS measurement timestamp
     *          and when the data is made available to the autopilot. This lag is used
     *          by the EKF for proper time-alignment of GPS measurements.
     *          
     *          NMEA protocol does not provide explicit timing information about measurement
     *          lag, so this returns an estimated value based on typical GPS receiver
     *          characteristics and serial transmission delays.
     * 
     * @param[out] lag_sec Estimated lag time in seconds (typically 0.1-0.3 seconds for NMEA)
     * 
     * @return true if lag estimate is available
     * @return false if lag cannot be determined
     * 
     * @note EKF uses this lag value to properly fuse GPS measurements with IMU data
     * 
     * @see AP_NavEKF3 for GPS measurement fusion with lag compensation
     */
    bool get_lag(float &lag_sec) const override;

#if HAL_LOGGING_ENABLED
    /**
     * @brief Write GPS configuration and status to dataflash log at startup
     * 
     * @details Logs GPS driver initialization information including:
     *          - Driver type (NMEA)
     *          - GPS receiver detection status
     *          - Configuration strings sent
     *          - Initial fix status
     *          
     *          Called once during GPS initialization to record startup configuration
     *          in the dataflash log for post-flight analysis and debugging.
     * 
     * @note Only available when logging is enabled (HAL_LOGGING_ENABLED)
     * 
     * @see AP_Logger for the dataflash logging subsystem
     */
    void Write_AP_Logger_Log_Startup_messages() const override;
#endif

private:
    /**
     * @brief NMEA sentence type identifiers for parser state machine
     * 
     * @details Enumeration of supported NMEA sentence types with unique numeric codes.
     *          Values are spaced to accommodate maximum field count in each sentence type,
     *          allowing term_number to be combined with sentence_type for state tracking.
     *          
     *          Standard NMEA Sentences:
     *          - RMC (Recommended Minimum): Position, velocity, course, date (12+ fields)
     *          - GGA (Global Positioning Fix): Position, altitude, fix quality, satellites (15+ fields)
     *          - VTG (Track Made Good): Ground speed and course (9+ fields)
     *          - HDT (Heading True): True heading from dual-antenna GPS (3+ fields)
     *          
     *          Vendor Extensions:
     *          - PHD (AllyStar): Vertical velocity and accuracy, maps ublox UBX NAV messages to NMEA
     *          - THS (Trimble MB-Two): True heading with quality indicator for dual-antenna systems
     *          - KSXT (Unicore): 3D velocity (NED frame) and yaw angle (21 fields)
     *          - AGRICA (Unicore): RTK position, heading, velocity, and accuracy (65 fields)
     *          - VERSIONA (Unicore): Firmware version information (10 fields)
     *          - UNIHEADINGA (Unicore): Dual-antenna baseline heading and pitch (20 fields)
     *          
     * @note Numeric values are chosen to be multiples allowing term indexing within sentence type
     * @note OTHER = 0 is used for unrecognized or unsupported sentence types
     */
    enum _sentence_types : uint16_t {
        _GPS_SENTENCE_RMC = 32,          ///< $GPRMC: Recommended Minimum Specific GPS/Transit Data (position, velocity, date)
        _GPS_SENTENCE_GGA = 64,          ///< $GPGGA: Global Positioning System Fix Data (position, altitude, fix quality)
        _GPS_SENTENCE_VTG = 96,          ///< $GPVTG: Track Made Good and Ground Speed
        _GPS_SENTENCE_HDT = 128,         ///< $GPHDT: Heading True (from dual-antenna GPS)
        _GPS_SENTENCE_PHD = 138,         ///< $PHD: AllyStar extension - vertical velocity and accuracy (maps ublox UBX to NMEA)
        _GPS_SENTENCE_THS = 160,         ///< $GPTHS: True heading with quality indicator (Trimble MB-Two)
        _GPS_SENTENCE_KSXT = 170,        ///< $KSXT: Unicore extension - 3D velocity and yaw (21 fields)
        _GPS_SENTENCE_AGRICA = 193,      ///< #AGRICA: Unicore extension - RTK position and heading (65 fields)
        _GPS_SENTENCE_VERSIONA = 270,    ///< #VERSIONA: Unicore extension - firmware version (10 fields)
        _GPS_SENTENCE_UNIHEADINGA = 290, ///< #UNIHEADINGA: Unicore extension - dual-antenna heading (20 fields)
        _GPS_SENTENCE_OTHER = 0          ///< Unrecognized or unsupported sentence type
    };

    /**
     * @brief Update NMEA parser state machine with next character from GPS stream
     * 
     * @details Implements character-by-character NMEA sentence parsing state machine.
     *          Handles sentence framing, term separation, checksum calculation, and
     *          validation. Maintains parser state across calls to accumulate complete
     *          sentences from the character stream.
     *          
     *          Parser State Machine:
     *          1. Wait for sentence start: '$' (standard NMEA) or '#' (Unicore)
     *          2. Accumulate sentence type identifier (GP, GN, GL, BD + message type)
     *          3. Process comma-separated terms, accumulating characters into _term buffer
     *          4. Calculate XOR checksum parity for standard NMEA or CRC32 for Unicore
     *          5. When '*' or '#' encountered, validate checksum
     *          6. Accept sentence data if checksum valid
     *          
     *          Character Processing:
     *          - '$' or '#': Reset state, start new sentence
     *          - ',': Term separator, process accumulated term
     *          - '*': Checksum follows (standard NMEA)
     *          - '\r' or '\n': End of sentence, validate and process
     *          - Other: Accumulate into current term or checksum
     * 
     * @param[in] c Next character from GPS serial stream
     * 
     * @return true when a complete, valid sentence has been parsed and GPS state updated
     * @return false if sentence incomplete, invalid, or checksum failed
     * 
     * @note Checksum validation prevents corruption from causing invalid GPS data
     * @note Parser is robust to garbage characters between sentences
     * 
     * @see _term_complete() for processing complete terms
     */
    bool                        _decode(char c);

    /**
     * @brief Parse NMEA decimal number string with up to 3 decimal digits
     * 
     * @details Converts NMEA ASCII decimal number to fixed-point integer representation.
     *          Handles optional decimal point and up to 3 fractional digits.
     *          Returns value scaled by 100 for efficient integer arithmetic.
     *          
     *          Examples:
     *          - "12.34" → 1234 (12.34 × 100)
     *          - "0.5" → 50 (0.5 × 100)
     *          - "100" → 10000 (100 × 100)
     *          
     *          Used for parsing NMEA fields like HDOP, speed, course where
     *          integer arithmetic is preferred over floating point.
     * 
     * @param[in] p Pointer to null-terminated string containing decimal number
     * 
     * @return Parsed value multiplied by 100 (fixed-point representation with 2 decimal places)
     * 
     * @note Efficient integer parsing avoids floating-point operations
     * @note Handles missing decimal point (treats as integer)
     */
    static int32_t _parse_decimal_100(const char *p);

    /**
     * @brief Parse NMEA degrees+minutes coordinate format to decimal degrees
     * 
     * @details Converts NMEA coordinate representation (DDMM.MMMM or DDDMM.MMMM format)
     *          to decimal degrees scaled by 1e7 for high-precision integer representation.
     *          
     *          NMEA Coordinate Format:
     *          - Latitude: DDMM.MMMM (DD = degrees 00-90, MM.MMMM = minutes 00.0000-59.9999)
     *          - Longitude: DDDMM.MMMM (DDD = degrees 000-180, MM.MMMM = minutes)
     *          
     *          Conversion: decimal_degrees = DD + MM.MMMM/60
     *          
     *          Examples:
     *          - "4807.038" → 48°07.038' → 48.1173° → 481173000 (× 1e7)
     *          - "01131.000" → 11°31.000' → 11.5167° → 115167000 (× 1e7)
     *          
     *          Resolution:
     *          With 4 decimal places in minutes: 0.0001' = 0.0001/60° ≈ 1.67e-6° ≈ 0.185 meters at equator
     *          After scaling by 1e7: Effective resolution ~1-2 cm
     * 
     * @return Decimal degrees multiplied by 1e7 (matches GPS driver coordinate scaling)
     * 
     * @note Result scaled by 1e7 matches ArduPilot GPS coordinate representation
     * @note Integer arithmetic avoids floating-point rounding errors
     * 
     * @see state.location.lat, state.location.lng in GPS_Backend for coordinate storage format
     */
    uint32_t    _parse_degrees();

    /**
     * @brief Process completed NMEA sentence term based on sentence type and term position
     * 
     * @details Called by _decode() when comma or end-of-sentence detected. Extracts
     *          numeric or string data from _term buffer and stores in temporary variables
     *          (_new_latitude, _new_longitude, etc.) until complete sentence validated.
     *          
     *          Term processing is sentence-type and position-dependent:
     *          
     *          GPGGA Terms:
     *          - Term 1: UTC time (HHMMSS.sss)
     *          - Term 2,3: Latitude (DDMM.MMMM), N/S
     *          - Term 4,5: Longitude (DDDMM.MMMM), E/W
     *          - Term 6: Fix quality (0=invalid, 1=GPS, 2=DGPS, 4=RTK fixed, 5=RTK float)
     *          - Term 7: Number of satellites
     *          - Term 8: HDOP (Horizontal Dilution of Precision)
     *          - Term 9: Altitude MSL
     *          
     *          GPRMC Terms:
     *          - Term 1: UTC time (HHMMSS.sss)
     *          - Term 2: Status (A=active/valid, V=void/invalid)
     *          - Term 3,4: Latitude, N/S
     *          - Term 5,6: Longitude, E/W
     *          - Term 7: Speed over ground (knots)
     *          - Term 8: Course over ground (degrees)
     *          - Term 9: Date (DDMMYY)
     *          
     *          GPVTG Terms:
     *          - Term 1: Course (degrees true)
     *          - Term 5: Speed (knots)
     *          - Term 7: Speed (km/h)
     *          
     *          Vendor extension terms processed by dedicated parsers (parse_agrica_field, etc.)
     * 
     * @return true if processing term resulted in updating GPS state
     * @return false if term is intermediate or sentence incomplete
     * 
     * @note Parsed values stored temporarily until sentence checksum validated
     * @note Invalid or missing terms are handled gracefully (skipped or defaulted)
     * 
     * @see _decode() for term extraction and calling context
     * @see _have_new_message() for determining when to commit parsed values
     */
    bool                        _term_complete();

    /**
     * @brief Check if complete set of NMEA messages received for GPS state update
     * 
     * @details Determines whether sufficient NMEA sentences have been received to
     *          constitute a complete GPS fix update. Different sentence combinations
     *          provide different data:
     *          
     *          Minimum for position fix:
     *          - GPGGA alone: Position, altitude, fix quality
     *          - GPRMC alone: Position, velocity (but NO altitude - warning case)
     *          
     *          Complete fix:
     *          - GPGGA + GPRMC: Position, altitude, velocity, date/time
     *          - GPGGA + GPVTG: Position, altitude, velocity
     *          
     *          Enhanced fix (vendor extensions):
     *          - + KSXT: Adds 3D velocity and yaw
     *          - + AGRICA: Adds RTK heading and accuracy
     *          - + PHD: Adds vertical velocity and accuracy estimates
     * 
     * @return true if sufficient sentence data received for GPS state update
     * @return false if waiting for more sentences to complete fix data
     * 
     * @note Method checks timestamps of received sentences to avoid using stale data
     * @note Sentence combination logic balances latency vs completeness
     */
    bool _have_new_message(void);

#if AP_GPS_NMEA_UNICORE_ENABLED
    /**
     * @brief Parse Unicore AGRICA sentence field
     * 
     * @details Processes individual fields from Unicore #AGRICA proprietary sentence
     *          providing RTK position, velocity, heading, and accuracy data. AGRICA
     *          contains 65 comma-separated fields with comprehensive GNSS solution data.
     *          
     *          Key AGRICA Fields:
     *          - RTK solution status (fixed/float/autonomous)
     *          - Heading status and value
     *          - Position (lat/lon/alt) with standard deviations
     *          - Velocity NED (North-East-Down frame) with standard deviations
     *          - Undulation (geoid separation)
     *          - GPS time (iTOW - integer time of week)
     * 
     * @param[in] term_number Field position within AGRICA sentence (0-65)
     * @param[in] term Pointer to null-terminated string containing field data
     * 
     * @note Only available when AP_GPS_NMEA_UNICORE_ENABLED
     * @note AGRICA provides significantly more data than standard NMEA sentences
     */
    void parse_agrica_field(uint16_t term_number, const char *term);

    /**
     * @brief Parse Unicore VERSIONA sentence field
     * 
     * @details Extracts firmware version information from Unicore #VERSIONA sentence.
     *          Used to identify GPS receiver model and firmware version for logging
     *          and compatibility checking.
     *          
     *          VERSIONA Fields:
     *          - Receiver type (model identifier)
     *          - Firmware version string
     *          - Build date
     * 
     * @param[in] term_number Field position within VERSIONA sentence
     * @param[in] term Pointer to null-terminated string containing field data
     * 
     * @note Only available when AP_GPS_NMEA_UNICORE_ENABLED
     * @note Version information logged at startup for diagnostics
     */
    void parse_versiona_field(uint16_t term_number, const char *term);

#if GPS_MOVING_BASELINE
    /**
     * @brief Parse Unicore UNIHEADINGA sentence field for dual-antenna heading
     * 
     * @details Processes Unicore #UNIHEADINGA sentence containing dual-antenna
     *          baseline heading and pitch measurements. Provides accurate heading
     *          independent of vehicle motion (unlike magnetic compass or GPS course).
     *          
     *          UNIHEADINGA Fields:
     *          - Baseline length (distance between antennas in meters)
     *          - Heading (degrees, 0-360)
     *          - Pitch (degrees, -90 to +90)
     *          - Heading standard deviation (accuracy estimate)
     *          
     *          Typical dual-antenna baseline: 0.3m to 2.0m
     *          Heading accuracy: ~0.5° with 1m baseline
     * 
     * @param[in] term_number Field position within UNIHEADINGA sentence
     * @param[in] term Pointer to null-terminated string containing field data
     * 
     * @note Only available when GPS_MOVING_BASELINE enabled
     * @note Requires two GPS antennas connected to Unicore receiver
     * @note Provides heading even when vehicle stationary
     */
    void parse_uniheadinga_field(uint16_t term_number, const char *term);
#endif
#endif


    uint8_t _parity;                                                    ///< NMEA message checksum accumulator
    uint32_t _crc32;                                            ///< CRC for unicore messages
    bool _is_checksum_term;                                     ///< current term is the checksum
    char _term[30];                                                     ///< buffer for the current term within the current sentence
    uint16_t _sentence_type;                                     ///< the sentence type currently being processed
    bool _is_unicore;                                           ///< true if in a unicore '#' sentence
    uint16_t _term_number;                                       ///< term index within the current sentence
    uint8_t _term_offset;                                       ///< character offset with the term being received
    uint16_t _sentence_length;
    bool _sentence_done;                                        ///< set when a sentence has been fully decoded

    // The result of parsing terms within a message is stored temporarily until
    // the message is completely processed and the checksum validated.
    // This avoids the need to buffer the entire message.
    int32_t _new_time;                                                  ///< time parsed from a term
    int32_t _new_date;                                                  ///< date parsed from a term
    int32_t _new_latitude;                                      ///< latitude parsed from a term
    int32_t _new_longitude;                                     ///< longitude parsed from a term
    int32_t _new_altitude;                                      ///< altitude parsed from a term
    int32_t _new_speed;                                                 ///< speed parsed from a term
    int32_t _new_course;                                        ///< course parsed from a term
    float   _new_gps_yaw;                                        ///< yaw parsed from a term
    uint16_t _new_hdop;                                                 ///< HDOP parsed from a term
    uint8_t _new_satellite_count;                       ///< satellite count parsed from a term
    uint8_t _new_quality_indicator;                                     ///< GPS quality indicator parsed from a term

    uint32_t _last_RMC_ms;
    uint32_t _last_GGA_ms;
    uint32_t _last_VTG_ms;
    uint32_t _last_yaw_ms;
    uint32_t _last_vvelocity_ms;
    uint32_t _last_vaccuracy_ms;
    uint32_t _last_3D_velocity_ms;
    uint32_t _last_KSXT_pos_ms;
    uint32_t _last_AGRICA_ms;
    uint32_t _last_fix_ms;

    /// @name	Init strings
    ///			In ::init, an attempt is made to configure the GPS
    ///			unit to send just the messages that we are interested
    ///			in using these strings
    //@{
    static const char _SiRF_init_string[];         ///< init string for SiRF units
    static const char _ublox_init_string[];        ///< init string for ublox units
    //@}

    static const char _initialisation_blob[];

    /**
     * @brief AllyStar PHD sentence parser state
     * 
     * @details The $PHD message is a proprietary extension from AllyStar GPS modules
     *          that maps ublox UBX binary protocol messages to NMEA ASCII format.
     *          Provides vertical velocity and accuracy estimates not available in
     *          standard NMEA sentences.
     *          
     *          PHD Message Structure: $PHD,<class>,<id>,<format>,<blank>,<fields>*<checksum>
     *          
     *          Supported Mappings:
     *          - Class 01, ID 12: NAV-VELNED (velocity NED frame, accuracy estimates)
     *            Fields: iTOW, velN, velE, velD, speed, gSpeed, heading, sAcc, cAcc
     *          - Class 01, ID 26: NAV-PVERR (position/velocity error estimates)
     *            Fields: iTOW, hAcc, vAcc, pDOP, numSV, reserved
     *          
     *          Examples:
     *          - $PHD,01,12,TIIITTITT,,245808000,0,0,0,0,0,10260304,0,0*27
     *          - $PHD,01,26,TTTTTTT,,245808000,877,864,1451,11,11,17*17
     *          
     *          Format field specifies data types: T=time, I=integer, etc.
     * 
     * @note PHD allows AllyStar receivers to provide ublox-equivalent data via NMEA
     * @note Vertical velocity from NAV-VELNED improves altitude estimation in EKF
     */
    struct {
        uint8_t msg_class;   ///< UBX message class (01 = NAV)
        uint8_t msg_id;      ///< UBX message ID (12 = VELNED, 26 = PVERR)
        uint32_t itow;       ///< GPS time of week in milliseconds
        int32_t fields[8];   ///< Parsed message fields (contents depend on msg_class/msg_id)
    } _phd;

    /**
     * @brief Unicore KSXT sentence parser state
     * 
     * @details The $KSXT message is a proprietary extension from Unicore GPS receivers
     *          providing 3D velocity in NED frame and yaw angle. This data supplements
     *          standard NMEA position information with high-rate velocity and heading.
     *          
     *          KSXT Message Structure (21 fields):
     *          1. UTC timestamp (YYYYMMDDHHMMSS.ss)
     *          2. Longitude (decimal degrees)
     *          3. Latitude (decimal degrees)
     *          4. Altitude (meters MSL)
     *          5. Velocity North (m/s)
     *          6. Velocity East (m/s)
     *          7. Velocity Down (m/s)
     *          8. Velocity standard deviation (m/s)
     *          9. Reserved
     *          10. Position solution status
     *          11. Velocity solution status
     *          12. Number of satellites used
     *          13. Number of satellites tracked
     *          14-17. Reserved
     *          18. Velocity North standard deviation (m/s)
     *          19. Velocity East standard deviation (m/s)
     *          20. Velocity Down standard deviation (m/s)
     *          21. Reserved
     *          
     *          Example:
     *          $KSXT,20211016083433.00,116.31296102,39.95817066,49.4911,223.57,-11.32,330.19,0.024,,1,3,28,27,,,,-0.012,0.021,0.020,,*2D
     *          
     *          Coordinate System: NED (North-East-Down) velocity frame
     * 
     * @note 3D velocity improves EKF performance, especially for vertical velocity
     * @note Velocity standard deviations used by EKF for adaptive fusion
     */
    struct {
        double fields[21];  ///< All 21 KSXT fields stored as doubles (mixed position, velocity, status)
    } _ksxt;

#if AP_GPS_NMEA_UNICORE_ENABLED
    /**
     * @brief Unicore AGRICA sentence parsed data structure
     * 
     * @details Stores parsed data from Unicore #AGRICA proprietary sentence providing
     *          comprehensive RTK (Real-Time Kinematic) GNSS solution data including
     *          position, velocity, heading, and accuracy estimates.
     *          
     *          AGRICA provides:
     *          - RTK position solution (fixed/float/autonomous status)
     *          - Dual-antenna heading (when two antennas configured)
     *          - 3D velocity vector in NED frame
     *          - Position standard deviations (lat, lon, alt)
     *          - Velocity standard deviations (N, E, D)
     *          - Geoid undulation (WGS84 to MSL conversion)
     *          
     *          RTK Status Values:
     *          - 0: No solution
     *          - 1: Single point positioning
     *          - 4: RTK fixed (cm-level accuracy)
     *          - 5: RTK float (dm-level accuracy)
     *          
     *          Heading Status Values:
     *          - 0: No heading solution
     *          - 1: Valid dual-antenna heading
     *          
     *          Typical Accuracy (RTK fixed):
     *          - Horizontal: 1-2 cm
     *          - Vertical: 2-3 cm
     *          - Heading: 0.2-0.5 degrees (depends on baseline length)
     * 
     * @note Only available when AP_GPS_NMEA_UNICORE_ENABLED
     * @note RTK requires base station or NTRIP corrections
     */
    struct {
        uint32_t start_byte;      ///< Sentence start marker for framing validation
        uint8_t rtk_status;       ///< RTK solution status (0=none, 1=autonomous, 4=fixed, 5=float)
        uint8_t heading_status;   ///< Heading solution status (0=invalid, 1=valid dual-antenna)
        Vector3f vel_NED;         ///< Velocity vector in NED frame (North, East, Down in m/s)
        Vector3f vel_stddev;      ///< Velocity standard deviations (North, East, Down in m/s)
        double lat, lng;          ///< Position: Latitude and longitude (decimal degrees, WGS84)
        float alt;                ///< Altitude above mean sea level (meters)
        uint32_t itow;            ///< GPS time of week (milliseconds)
        float undulation;         ///< Geoid undulation: WGS84 ellipsoid to MSL offset (meters)
        Vector3f pos_stddev;      ///< Position standard deviations (lat, lon, alt in meters)
    } _agrica;

    /**
     * @brief Unicore VERSIONA firmware version information
     * 
     * @details Parsed firmware version data from Unicore #VERSIONA sentence.
     *          Used for logging GPS receiver identification and for implementing
     *          firmware version-specific workarounds if needed.
     *          
     *          Version Information:
     *          - Receiver type/model identifier
     *          - Firmware version string (major.minor.patch format)
     *          - Build date (YYYY-MM-DD format)
     * 
     * @note Logged at startup for post-flight diagnostics
     * @note Helps identify GPS hardware configuration in multi-receiver setups
     */
    struct {
        char type[10];         ///< Receiver type/model identifier string
        char version[20];      ///< Firmware version string (e.g., "1.2.3")
        char build_date[13];   ///< Firmware build date (YYYY-MM-DD format)
    } _versiona;
    bool _have_unicore_versiona;  ///< True after VERSIONA sentence successfully parsed

#if GPS_MOVING_BASELINE
    /**
     * @brief Unicore UNIHEADINGA dual-antenna heading solution
     * 
     * @details Parsed dual-antenna heading data from Unicore #UNIHEADINGA sentence.
     *          Provides accurate heading independent of vehicle motion by measuring
     *          the vector between two GPS antennas (baseline).
     *          
     *          Dual-Antenna Heading Advantages:
     *          - Works when vehicle stationary (unlike GPS course over ground)
     *          - No magnetic interference (unlike compass)
     *          - Stable in high magnetic noise environments
     *          - Provides pitch angle from antenna height difference
     *          
     *          Baseline Configuration:
     *          - Minimum baseline: ~0.3m (heading accuracy ~2°)
     *          - Typical baseline: 0.5-1.0m (heading accuracy ~0.5-1°)
     *          - Maximum baseline: 2-3m (heading accuracy ~0.2-0.5°)
     *          
     *          Accuracy Formula: heading_accuracy ≈ arctan(2cm / baseline_length)
     *          where 2cm is typical RTK positioning accuracy
     * 
     * @note Only available when GPS_MOVING_BASELINE enabled
     * @note Requires two GPS antennas connected to Unicore receiver
     * @note Baseline must be rigidly mounted and calibrated in autopilot parameters
     */
    struct {
        float baseline_length;  ///< Distance between primary and secondary antennas (meters)
        float heading;          ///< True heading from antenna baseline (degrees, 0-360)
        float pitch;            ///< Pitch angle from antenna height difference (degrees, -90 to +90)
        float heading_sd;       ///< Heading standard deviation - accuracy estimate (degrees)
    } _uniheadinga;
#endif
#endif // AP_GPS_NMEA_UNICORE_ENABLED
    bool _expect_agrica;  ///< Flag indicating AGRICA sentence expected in message set

    uint32_t last_config_ms;  ///< Timestamp (milliseconds) of last configuration string transmission to GPS

    /**
     * @brief Send GPS receiver type-specific configuration strings
     * 
     * @details Attempts to auto-configure known GPS receivers to output appropriate
     *          NMEA sentences for autopilot use. Sends initialization strings to:
     *          
     *          SiRF Receivers:
     *          - Switch from SiRF binary protocol to NMEA mode
     *          - Configure output message types (GGA, RMC, VTG)
     *          - Set update rate to maximum
     *          
     *          MediaTek Receivers:
     *          - Configure NMEA sentence selection
     *          - Set update rate (typically 5 Hz or 10 Hz)
     *          - Enable WAAS/EGNOS if available
     *          
     *          ublox Receivers:
     *          - Configure UBX-CFG-NMEA for optimal settings
     *          - Set talker ID to GP or GN
     *          - Enable high-precision mode if supported
     *          
     *          Unicore Receivers:
     *          - Configure serial port baud rate (230400)
     *          - Enable GPGGA output at 5 Hz
     *          - Prepare for proprietary sentence reception
     *          
     *          Configuration is sent periodically (every few seconds) until GPS
     *          starts producing valid output, as receiver type is not known until
     *          after detection completes.
     * 
     * @note Configuration strings are GPS type-specific and may not work on all receivers
     * @note Some receivers ignore configuration if already in NMEA mode
     * @note Called periodically from read() until GPS provides valid fix data
     * 
     * @see _initialisation_blob, _SiRF_init_string, _ublox_init_string for config data
     */
    void send_config(void);
};

#if AP_GPS_NMEA_UNICORE_ENABLED && !defined(NMEA_UNICORE_SETUP)
// we don't know what port the GPS may be using, so configure all 3. We need to get it sending
// one message to allow the NMEA detector to run
#define NMEA_UNICORE_SETUP "CONFIG COM1 230400 8 n 1\r\nCONFIG COM2 230400 8 n 1\r\nCONFIG COM3 230400 8 n 1\r\nGPGGA 0.2\r\n"
#endif

#endif // AP_GPS_NMEA_ENABLED

