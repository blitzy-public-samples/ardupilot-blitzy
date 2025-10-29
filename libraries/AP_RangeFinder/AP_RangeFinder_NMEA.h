/**
 * @file AP_RangeFinder_NMEA.h
 * @brief NMEA protocol rangefinder driver for marine depth sounders and underwater distance sensors
 * 
 * @details This backend implements support for NMEA-0183 compatible depth sounders commonly used
 *          in marine and underwater robotics applications. It parses standard NMEA sentences
 *          transmitted over serial connections to extract depth/distance measurements and optional
 *          water temperature data.
 *          
 *          Primary use case: Underwater vehicles (ArduSub) using commercial depth sounders for
 *          altitude-above-bottom measurement, precision landing, and obstacle avoidance.
 *          
 *          Supported NMEA sentence types:
 *          - DBT (Depth Below Transducer): Standard depth measurement in feet, meters, fathoms
 *          - DPT (Depth): Enhanced depth with offset compensation
 *          - MTW (Mean Water Temperature): Optional temperature sensor data
 *          - HDED (Hondex custom): Vendor-specific format for Hondex depth sounders
 *          
 *          The parser implements character-by-character decoding with checksum validation
 *          following NMEA-0183 specifications. Typical sentence format:
 *          $--DBT,feet,f,meters,M,fathoms,F*checksum
 *          
 *          Integration: Configured via RNGFND_TYPE parameter. Requires serial connection
 *          to depth sounder with appropriate baud rate (typically 4800 or 9600).
 * 
 * @see AP_RangeFinder_Backend_Serial
 * @see ArduSub vehicle for primary usage
 * @see NMEA-0183 specification for protocol details
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_NMEA_ENABLED

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend_Serial.h"

/**
 * @class AP_RangeFinder_NMEA
 * @brief NMEA-0183 protocol rangefinder backend for marine depth sounders
 * 
 * @details This backend implements a serial-based rangefinder driver that parses NMEA-0183
 *          sentences commonly transmitted by marine depth sounders (also called echo sounders
 *          or fishfinders). The primary application is underwater vehicles measuring distance
 *          to the sea floor or underwater obstacles.
 *          
 *          **Architecture**:
 *          - Inherits from AP_RangeFinder_Backend_Serial for serial communication handling
 *          - Implements character-by-character NMEA sentence parsing with state machine
 *          - Validates data using NMEA checksum (*XX format)
 *          - Extracts depth measurements in meters from various sentence formats
 *          - Optionally extracts water temperature from MTW sentences
 *          
 *          **NMEA Sentence Parsing**:
 *          NMEA sentences follow format: $AABBB,field1,field2,...*CS<CR><LF>
 *          - AA: Talker ID (e.g., SD for depth sounder)
 *          - BBB: Sentence formatter (e.g., DBT for depth below transducer)
 *          - Fields: Comma-separated data values
 *          - CS: Two-character hexadecimal checksum (XOR of all bytes between $ and *)
 *          
 *          **DBT Sentence Example** (Depth Below Transducer):
 *          $SDDBT,10.5,f,3.2,M,1.75,F*3A
 *          - 10.5 feet, 3.2 meters, 1.75 fathoms
 *          - Parser extracts the meter value (field 4)
 *          
 *          **DPT Sentence Example** (Depth):
 *          $SDDPT,3.2,0.5*4D
 *          - 3.2 meters depth, 0.5 meter transducer offset
 *          
 *          **Usage Pattern**:
 *          1. Connect depth sounder NMEA output to autopilot serial port
 *          2. Configure SERIAL_PROTOCOL = 9 (RangeFinder)
 *          3. Set RNGFND_TYPE = 17 (NMEA)
 *          4. Set appropriate baud rate (typically 4800 or 9600)
 *          5. Backend automatically detects and parses supported sentences
 *          
 *          **Thread Safety**:
 *          - Called from serial I/O thread via read_timeout_ms() timer
 *          - State machine variables protected by serial backend
 *          - Distance and temperature atomically updated per sentence
 *          
 *          **Coordinate System**:
 *          - Measures distance downward from transducer (body-down axis)
 *          - Returns positive distance to detected surface/object
 *          - ArduSub typically mounts transducer on bottom pointing down
 *          
 *          **Performance**:
 *          - Update rate: 1-10 Hz depending on depth sounder
 *          - Timeout: 3000ms (read_timeout_ms())
 *          - Typical latency: 100-200ms from measurement to parsed value
 *          
 * @note This driver is primarily used in ArduSub for underwater navigation
 * @note Compatible with most NMEA-0183 depth sounders (Garmin, Lowrance, Hondex, etc.)
 * @warning Ensure correct serial port configuration - incorrect baud rate will cause parse failures
 * @warning Checksum validation is mandatory - corrupted data is rejected
 * 
 * @see AP_RangeFinder_Backend_Serial for serial communication implementation
 * @see ArduSub documentation for depth sounder integration examples
 */
class AP_RangeFinder_NMEA : public AP_RangeFinder_Backend_Serial
{

public:

    /**
     * @brief Factory method to create NMEA rangefinder backend instance
     * 
     * @details Static factory method called by rangefinder driver manager during initialization.
     *          Allocates a new AP_RangeFinder_NMEA instance on the heap using NEW_NOTHROW to
     *          handle potential memory allocation failures gracefully.
     *          
     *          Called when RNGFND_TYPE parameter is set to 17 (NMEA) during sensor detection.
     * 
     * @param[in,out] _state Reference to rangefinder state structure for storing measurements
     * @param[in]     _params Reference to rangefinder parameters (min/max range, offsets, etc.)
     * 
     * @return Pointer to newly created backend instance, or nullptr if allocation fails
     * 
     * @note Uses NEW_NOTHROW to avoid exceptions on memory allocation failure
     * @note Caller is responsible for checking nullptr return
     * 
     * @see AP_RangeFinder::detect_instance() for backend instantiation
     */
    static AP_RangeFinder_Backend_Serial *create(
        RangeFinder::RangeFinder_State &_state,
        AP_RangeFinder_Params &_params) {
        return NEW_NOTHROW AP_RangeFinder_NMEA(_state, _params);
    }

protected:

    /**
     * @brief Report sensor type to MAVLink ground control stations
     * 
     * @details Returns the MAVLink distance sensor type identifier for this rangefinder.
     *          NMEA depth sounders typically use ultrasonic transducers (sonar), so this
     *          reports MAV_DISTANCE_SENSOR_ULTRASOUND to ground stations for correct
     *          sensor identification and display.
     *          
     *          This value appears in DISTANCE_SENSOR MAVLink messages and helps GCS software
     *          correctly interpret and display the sensor data (e.g., showing sonar icon).
     * 
     * @return MAV_DISTANCE_SENSOR_ULTRASOUND (constant value 0)
     * 
     * @note Override of virtual method from AP_RangeFinder_Backend
     * @note Some depth sounders may use other technologies (laser, radar) but ultrasound is most common
     * 
     * @see MAVLink DISTANCE_SENSOR message definition
     * @see AP_RangeFinder_Backend::_get_mav_distance_sensor_type()
     */
    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_ULTRASOUND;
    }

private:

    /**
     * @brief Inherit parent class constructors
     * 
     * @details Using declaration to inherit all constructors from AP_RangeFinder_Backend_Serial.
     *          This allows the factory method to construct the backend with state and params
     *          without explicitly defining constructors in this derived class.
     */
    using AP_RangeFinder_Backend_Serial::AP_RangeFinder_Backend_Serial;

    /**
     * @enum sentence_types
     * @brief NMEA sentence type identifiers for parsing state machine
     * 
     * @details Enumeration of supported NMEA-0183 sentence formats that this driver can parse.
     *          The parser identifies the sentence type from the three-character formatter field
     *          (characters 3-5 after the $ start delimiter) and sets _sentence_type accordingly
     *          to guide field extraction logic.
     *          
     *          Each sentence type has a different field structure and meaning:
     *          
     *          **SONAR_DBT** (Depth Below Transducer - NMEA standard):
     *          - Format: $--DBT,feet,f,meters,M,fathoms,F*CS
     *          - Provides depth in three units simultaneously
     *          - Most common format for depth sounders
     *          - Example: $SDDBT,10.5,f,3.2,M,1.75,F*3A
     *          
     *          **SONAR_DPT** (Depth - NMEA standard):
     *          - Format: $--DPT,depth_meters,offset_meters*CS
     *          - Single depth value with optional transducer offset
     *          - Offset allows compensation for transducer mounting position
     *          - Example: $SDDPT,3.2,0.5*4D
     *          
     *          **SONAR_MTW** (Mean Water Temperature):
     *          - Format: $--MTW,temperature,C*CS
     *          - Water temperature in Celsius
     *          - Optional auxiliary data, not used for ranging
     *          - Stored separately and accessible via get_temp()
     *          - Example: $SDMTW,15.5,C*2F
     *          
     *          **SONAR_HDED** (Hondex custom format):
     *          - Vendor-specific format for Hondex brand depth sounders
     *          - Non-standard NMEA extension
     *          - Field layout specific to Hondex protocol
     *          - Provides compatibility with Hondex fishfinders
     *          
     *          **SONAR_UNKNOWN**:
     *          - Default/unrecognized sentence type
     *          - Parser ignores sentence if type cannot be identified
     *          - Set when sentence formatter doesn't match known types
     */
    enum sentence_types : uint8_t {
        SONAR_UNKNOWN = 0,  ///< Unrecognized or unsupported sentence type (ignored)
        SONAR_DBT,          ///< Depth Below Transducer (standard format with feet/meters/fathoms)
        SONAR_DPT,          ///< Depth with offset (enhanced depth measurement)
        SONAR_MTW,          ///< Mean Water Temperature (auxiliary sensor data)
        SONAR_HDED,         ///< Hondex custom sonar message (vendor-specific format)
    };

    /**
     * @brief Retrieve the latest validated distance measurement
     * 
     * @details Override of backend virtual method called by rangefinder driver to obtain
     *          the most recent distance reading. This method reads characters from the serial
     *          port, feeds them to the NMEA parser via decode(), and returns successfully
     *          when a complete validated sentence has been parsed.
     *          
     *          **Operation**:
     *          1. Read available bytes from serial port (inherited uart)
     *          2. Feed each character to decode() for parsing
     *          3. When decode() returns true (complete sentence validated):
     *             - Extract distance from _distance_m member
     *             - Return distance via reading_m reference parameter
     *          4. Return false if no complete sentence parsed within available data
     *          
     *          **Parser Integration**:
     *          - decode() performs character-by-character state machine parsing
     *          - Checksum validation ensures data integrity
     *          - _distance_m set to valid value (-1 if invalid)
     *          - Only sentences with valid checksums update reading_m
     *          
     *          **Update Rate**:
     *          Depends on depth sounder transmission rate (typically 1-10 Hz).
     *          If no sentence received within read_timeout_ms() (3000ms), sensor
     *          marked as unhealthy by backend framework.
     * 
     * @param[out] reading_m Distance measurement in meters (positive = detected surface below)
     * 
     * @return true if valid distance reading obtained, false if no complete sentence available
     * 
     * @note Called repeatedly by rangefinder driver at scheduler task rate
     * @note Inherited uart object provides serial data via read() method
     * @note Distance set to -1.0f by parser if no valid measurement in sentence
     * 
     * @see decode() for NMEA parsing state machine
     * @see read_timeout_ms() for timeout specification
     * @see AP_RangeFinder_Backend_Serial for serial port handling
     */
    bool get_reading(float &reading_m) override;

    /**
     * @brief Retrieve the latest water temperature measurement
     * 
     * @details Returns the most recent validated water temperature extracted from NMEA MTW
     *          (Mean Water Temperature) sentences. This is auxiliary sensor data provided
     *          by depth sounders with integrated temperature sensors.
     *          
     *          **Temperature Data Flow**:
     *          1. Parser extracts temperature from MTW sentences
     *          2. Unvalidated value stored in _temp_unvalidated
     *          3. On successful checksum validation:
     *             - _temp_unvalidated copied to _temp
     *             - _temp_readtime_ms updated to current system time
     *          4. This method returns the validated _temp value
     *          
     *          **Staleness Check**:
     *          Method checks _temp_readtime_ms to determine if temperature data is available.
     *          If _temp_readtime_ms is 0, no temperature has been received since boot.
     *          
     *          **Application**:
     *          Water temperature useful for:
     *          - Speed of sound correction in sonar calculations
     *          - Environmental monitoring in underwater vehicles
     *          - Density compensation for depth sensors
     * 
     * @param[out] temp Water temperature in degrees Celsius
     * 
     * @return true if valid temperature reading available, false if no MTW sentence received
     * 
     * @note Override of virtual method from AP_RangeFinder_Backend
     * @note Temperature updates independent of distance measurement
     * @note Not all depth sounders transmit MTW sentences
     * @note Temperature persists until new MTW sentence received (does not timeout)
     * 
     * @see _temp member for validated temperature storage
     * @see _temp_readtime_ms for last update timestamp
     * @see sentence_types::SONAR_MTW for temperature sentence format
     */
    bool get_temp(float &temp) const override;

    /**
     * @brief Specify maximum time between valid readings before sensor marked unhealthy
     * 
     * @details Returns the timeout duration in milliseconds for this sensor type. If no valid
     *          distance reading is obtained within this timeout period, the rangefinder backend
     *          framework automatically marks the sensor as unhealthy and stops using its data.
     *          
     *          **Timeout Rationale**:
     *          3000ms (3 seconds) chosen to accommodate:
     *          - Slow update rates from depth sounders (minimum ~0.3 Hz)
     *          - Deep water measurements requiring longer pulse travel time
     *          - Serial communication at slow baud rates (4800 baud)
     *          - Intermittent loss of bottom lock in mid-water
     *          
     *          **Depth Sounder Characteristics**:
     *          - Shallow water (<10m): Typically 5-10 Hz update rate
     *          - Deep water (>50m): May slow to 1-2 Hz due to longer echo time
     *          - No bottom detected: Some units stop transmitting sentences
     *          
     *          **Health Management**:
     *          Backend framework compares time since last successful get_reading() against
     *          this timeout. Exceeding timeout triggers sensor unhealthy state, preventing
     *          use of stale data for navigation.
     * 
     * @return 3000 milliseconds (3 seconds)
     * 
     * @note Override of virtual method from AP_RangeFinder_Backend_Serial
     * @note Longer than typical rangefinder timeouts due to marine sensor characteristics
     * @note Health status visible to GCS via DISTANCE_SENSOR MAVLink messages
     * 
     * @see AP_RangeFinder_Backend::update() for timeout enforcement
     */
    uint16_t read_timeout_ms() const override { return 3000; }

    /**
     * @brief Process a single character through the NMEA parsing state machine
     * 
     * @details Core parsing method that implements character-by-character NMEA-0183 sentence
     *          decoding. This method is called repeatedly from get_reading() for each character
     *          received from the serial port. It maintains parser state across calls and returns
     *          true when a complete, validated sentence has been successfully decoded.
     *          
     *          **NMEA-0183 Format**:
     *          Standard sentences follow this structure:
     *          $AABBB,field1,field2,...,fieldN*CS<CR><LF>
     *          - '$': Start delimiter
     *          - 'AA': Two-character talker ID (e.g., 'SD' for depth sounder)
     *          - 'BBB': Three-character sentence formatter (e.g., 'DBT' for depth below transducer)
     *          - ',': Field separator
     *          - '*': Checksum delimiter
     *          - 'CS': Two hex digits (XOR of all bytes between $ and *)
     *          - <CR><LF>: Carriage return + line feed terminators
     *          
     *          **State Machine Operation**:
     *          1. **Waiting for '$'**: Discard characters until sentence start
     *          2. **Accumulating term**: Build current field in _term buffer until ',' or '*'
     *          3. **Checksum calculation**: XOR each character (between $ and *) into _checksum
     *          4. **Term completion**: On ',' call decode_latest_term() to process field
     *          5. **Checksum validation**: On '*' prepare to receive 2-digit hex checksum
     *          6. **Sentence completion**: Validate checksum and extract measurements
     *          
     *          **Checksum Algorithm**:
     *          - Initialize _checksum to 0 at sentence start
     *          - XOR each character between '$' and '*' (exclusive)
     *          - Compare computed checksum with received two-digit hex value
     *          - Reject entire sentence if mismatch (data corruption detected)
     *          
     *          **Field Extraction**:
     *          - _term buffer holds current field string (max 14 chars + null terminator)
     *          - _term_number tracks position in sentence (0 = formatter, 1+ = data fields)
     *          - decode_latest_term() interprets field based on _sentence_type and _term_number
     *          - Distance value stored in _distance_m when appropriate field parsed
     *          
     *          **Return Value Semantics**:
     *          - true: Complete sentence decoded, checksum validated, distance in _distance_m
     *          - false: Sentence incomplete, or checksum failed, or no distance in sentence
     *          
     *          **Error Recovery**:
     *          - Parse errors: Reset state and wait for next '$'
     *          - Checksum failure: Discard sentence, increment error counter
     *          - Buffer overflow: Truncate term, continue parsing
     *          - Unknown sentence: Parse but don't extract data
     * 
     * @param[in] c Single character from serial stream to process
     * 
     * @return true if complete validated sentence with distance measurement decoded, false otherwise
     * 
     * @note Distance value must be retrieved from _distance_m member (-1.0f if invalid)
     * @note Called at serial I/O rate (potentially thousands of times per second)
     * @note State machine variables (_term, _term_offset, etc.) maintain state between calls
     * @note Sentences without distance (e.g., MTW temperature-only) return false
     * 
     * @warning Checksum validation is mandatory - no distance returned without valid checksum
     * @warning Buffer overflow protection limits term length to 14 characters
     * 
     * @see decode_latest_term() for field interpretation logic
     * @see _distance_m for parsed distance storage
     * @see sentence_types for supported sentence formats
     */
    bool decode(char c);

    /**
     * @brief Interpret the most recently completed term based on sentence type and position
     * 
     * @details Called by decode() each time a complete field is extracted from the NMEA sentence
     *          (on ',' delimiter or '*' checksum delimiter). This method examines _sentence_type
     *          and _term_number to determine the meaning of the field in _term buffer and extracts
     *          numeric values accordingly.
     *          
     *          **Term Processing by Sentence Type**:
     *          
     *          **DBT Sentence** ($--DBT,feet,f,meters,M,fathoms,F*CS):
     *          - Term 0: "DBT" - Identifies sentence type
     *          - Term 1: Depth in feet (not used)
     *          - Term 2: "f" - Units indicator (not used)
     *          - Term 3: Depth in meters - **EXTRACTED** to _distance_m
     *          - Term 4: "M" - Units indicator (not used)
     *          - Term 5: Depth in fathoms (not used)
     *          - Term 6: "F" - Units indicator (not used)
     *          
     *          **DPT Sentence** ($--DPT,depth,offset*CS):
     *          - Term 0: "DPT" - Identifies sentence type
     *          - Term 1: Depth in meters - **EXTRACTED** to _distance_m
     *          - Term 2: Transducer offset in meters (not currently used)
     *          
     *          **MTW Sentence** ($--MTW,temperature,C*CS):
     *          - Term 0: "MTW" - Identifies sentence type
     *          - Term 1: Temperature in Celsius - **EXTRACTED** to _temp_unvalidated
     *          - Term 2: "C" - Units indicator (not used)
     *          
     *          **HDED Sentence** (Hondex custom format):
     *          - Vendor-specific field layout
     *          - Implementation extracts depth from Hondex-specific position
     *          
     *          **Sentence Type Identification**:
     *          When _term_number is 0, the term contains the sentence formatter (e.g., "DBT").
     *          This method compares against known types and sets _sentence_type to guide
     *          subsequent field extraction.
     *          
     *          **Return Value**:
     *          Method returns true when the final term of a sentence is processed AND the
     *          sentence has been validated (checksum correct). This signals decode() to
     *          return true to get_reading(), indicating a complete measurement is available.
     *          
     *          **Data Validation**:
     *          - Numeric conversion uses atof() on _term buffer
     *          - Invalid/missing fields may parse as 0.0 or garbage
     *          - Distance values set to -1.0f if field empty or invalid
     *          - Temperature stored even if invalid (validated on checksum pass)
     * 
     * @return true if sentence is complete and validated (final term processed with good checksum)
     *         false if more terms expected or validation pending
     * 
     * @note Called internally by decode() - not invoked directly by external code
     * @note Accesses parser state: _term, _term_number, _sentence_type, _term_is_checksum
     * @note Updates _distance_m or _temp_unvalidated based on sentence type
     * @note On successful sentence completion, copies _temp_unvalidated to _temp if MTW
     * 
     * @see decode() for state machine context
     * @see sentence_types for format details
     * @see _term for current field string
     * @see _term_number for field position
     */
    bool decode_latest_term();

    /**
     * @name NMEA Parser State Machine Variables
     * @brief Internal state variables for character-by-character NMEA sentence parsing
     * @{
     */
    
    /**
     * @brief Buffer for the current field being parsed from the NMEA sentence
     * 
     * @details Accumulates characters for a single comma-delimited field until the field
     *          terminator (',' or '*') is encountered. Buffer is null-terminated to allow
     *          string processing functions (atof, strcmp, etc.).
     *          
     *          Size: 15 bytes (14 characters + null terminator)
     *          - Sufficient for typical NMEA numeric fields: "123.456789"
     *          - Sufficient for sentence formatters: "DBT", "DPT", "MTW", "HDED"
     *          - Overflow protection: decode() limits _term_offset to prevent overrun
     *          
     *          Cleared at start of each field. Populated character-by-character by decode().
     *          Interpreted by decode_latest_term() based on sentence type and field position.
     */
    char _term[15];
    
    /**
     * @brief Write position within _term buffer for next character
     * 
     * @details Index indicating where the next character should be stored in _term buffer.
     *          Incremented for each character added to the current field. Reset to 0 at the
     *          start of each new field (after ',' or '$').
     *          
     *          Range: 0-13 (limited to prevent buffer overflow beyond null terminator position)
     *          
     *          When _term_offset reaches 13, additional characters are discarded (overflow
     *          protection). Null terminator always placed at _term[_term_offset] when field
     *          completes to ensure valid C string.
     */
    uint8_t _term_offset;
    
    /**
     * @brief Sequential position of current field within the NMEA sentence
     * 
     * @details Zero-based index tracking which field is currently being parsed. Incremented
     *          each time a comma delimiter is encountered. Used by decode_latest_term() to
     *          determine the meaning of the field based on sentence format.
     *          
     *          Field numbering:
     *          - 0: Sentence formatter (e.g., "DBT", "DPT", "MTW")
     *          - 1+: Data fields (meaning depends on sentence type)
     *          
     *          Example for $SDDBT,10.5,f,3.2,M,1.75,F*3A:
     *          - Term 0: "DBT"
     *          - Term 1: "10.5" (feet)
     *          - Term 2: "f" (units)
     *          - Term 3: "3.2" (meters) <- distance extracted here
     *          - Term 4: "M" (units)
     *          - Term 5: "1.75" (fathoms)
     *          - Term 6: "F" (units)
     *          
     *          Reset to 0 at start of new sentence ('$' character).
     */
    uint8_t _term_number;
    
    /**
     * @brief Most recently parsed distance measurement in meters
     * 
     * @details Storage for the extracted distance value from NMEA depth sentences. Updated
     *          by decode_latest_term() when the appropriate field is parsed based on sentence
     *          type. Retrieved by get_reading() to return measurement to rangefinder driver.
     *          
     *          Value semantics:
     *          - Positive value: Valid distance to detected surface/object (meters)
     *          - -1.0f: No valid distance in current/last sentence (default/invalid)
     *          - 0.0: Valid zero distance (surface at transducer, rare)
     *          
     *          Updated for sentence types:
     *          - DBT: Extracted from field 3 (meter value)
     *          - DPT: Extracted from field 1 (depth in meters)
     *          - HDED: Extracted from Hondex-specific field position
     *          
     *          Not updated for:
     *          - MTW: Temperature-only sentence (distance set to -1.0f)
     *          - UNKNOWN: Unrecognized sentence type
     *          
     *          Persistence: Retains last valid value until new sentence parsed. Value
     *          remains valid across multiple get_reading() calls if no new data received.
     */
    float _distance_m = -1.0f;
    
    /**
     * @brief Temperature value extracted from MTW sentence before checksum validation
     * 
     * @details Temporary storage for temperature extracted from NMEA MTW (Mean Water Temperature)
     *          sentence. Value stored here during parsing but not considered valid until the
     *          sentence checksum is verified. On successful validation, copied to _temp.
     *          
     *          Units: Degrees Celsius (as specified by NMEA-0183 MTW sentence)
     *          
     *          Data flow:
     *          1. decode_latest_term() extracts temperature from MTW term 1
     *          2. Stores in _temp_unvalidated
     *          3. Continue parsing through checksum
     *          4. If checksum valid: Copy to _temp and update _temp_readtime_ms
     *          5. If checksum invalid: Discard, _temp unchanged
     *          
     *          This two-stage validation ensures corrupted temperature data (failed checksum)
     *          is never exposed to the application.
     */
    float _temp_unvalidated;
    
    /**
     * @brief Validated water temperature in degrees Celsius
     * 
     * @details Storage for temperature measurement that has passed NMEA checksum validation.
     *          Extracted from MTW (Mean Water Temperature) sentences transmitted by depth
     *          sounders with integrated temperature sensors.
     *          
     *          Units: Degrees Celsius (NMEA-0183 standard)
     *          
     *          Update conditions:
     *          - MTW sentence received
     *          - Temperature field parsed successfully
     *          - Checksum validation passed
     *          - Copied from _temp_unvalidated on validation success
     *          
     *          Retrieval: Accessed via get_temp() method
     *          
     *          Staleness: No timeout - retains last valid value indefinitely. Caller should
     *          check _temp_readtime_ms to determine data freshness.
     *          
     *          Applications:
     *          - Speed of sound correction for sonar ranging accuracy
     *          - Environmental monitoring in underwater vehicles
     *          - Water density calculations
     */
    float _temp;
    
    /**
     * @brief System timestamp of last validated temperature reading
     * 
     * @details Records the system time (milliseconds since boot) when the most recent valid
     *          temperature measurement was received and validated. Used by get_temp() to
     *          determine if temperature data is available.
     *          
     *          Value semantics:
     *          - 0: No temperature data received since boot (never initialized)
     *          - >0: System time in milliseconds when last MTW sentence validated
     *          
     *          Updated simultaneously with _temp when MTW sentence checksum passes.
     *          
     *          Staleness checking: Caller can compare against current system time to
     *          determine age of temperature data. Unlike distance readings, temperature
     *          does not timeout - last valid value persists indefinitely.
     *          
     *          Units: Milliseconds (from AP_HAL::millis())
     */
    uint32_t _temp_readtime_ms;
    
    /**
     * @brief Running XOR checksum accumulator for current sentence
     * 
     * @details Computes NMEA-0183 checksum by XORing all characters between the '$' start
     *          delimiter and '*' checksum delimiter (exclusive of both delimiters). Result
     *          compared against the two-digit hexadecimal checksum value transmitted after '*'.
     *          
     *          Algorithm:
     *          1. Initialize to 0 when '$' received (sentence start)
     *          2. For each character c between '$' and '*': _checksum ^= c
     *          3. After '*', parse two hex digits as received checksum
     *          4. Compare computed _checksum against received value
     *          5. Sentence valid only if match exactly
     *          
     *          Example for "$SDDBT,10.5,f,3.2,M,1.75,F*3A":
     *          - XOR of "SDDBT,10.5,f,3.2,M,1.75,F" = 0x3A
     *          - Transmitted checksum: "3A" = 0x3A
     *          - Match: Sentence valid
     *          
     *          Purpose: Detect data corruption in serial transmission. Single-bit errors will
     *          cause checksum mismatch with very high probability.
     *          
     *          Reset: Cleared to 0 at start of each sentence ('$' character)
     */
    uint8_t _checksum;
    
    /**
     * @brief Flag indicating current term is the checksum field
     * 
     * @details Set to true when the '*' checksum delimiter is encountered, indicating that
     *          the next two characters are the hexadecimal checksum rather than data fields.
     *          Parser interprets subsequent characters as hex digits and validates against
     *          computed _checksum.
     *          
     *          State transitions:
     *          - false: Parsing normal data fields (between ',' delimiters)
     *          - true: Parsing checksum (after '*', expecting two hex digits)
     *          
     *          When true:
     *          - Characters interpreted as hex digits (0-9, A-F)
     *          - First digit: High nibble of received checksum
     *          - Second digit: Low nibble of received checksum
     *          - After second digit: Compare received vs computed checksum
     *          
     *          Reset: Cleared to false at start of new sentence ('$' character)
     */
    bool _term_is_checksum;
    
    /**
     * @brief Type identifier for the sentence currently being parsed
     * 
     * @details Identifies which NMEA sentence format is being processed in the current parse
     *          cycle. Determined by examining the sentence formatter field (first term after '$')
     *          and comparing against known types: "DBT", "DPT", "MTW", "HDED".
     *          
     *          Set by: decode_latest_term() when _term_number is 0 (formatter field)
     *          Used by: decode_latest_term() for subsequent fields to determine extraction logic
     *          
     *          Value controls field interpretation:
     *          - SONAR_DBT: Extract meters from term 3
     *          - SONAR_DPT: Extract meters from term 1
     *          - SONAR_MTW: Extract temperature from term 1
     *          - SONAR_HDED: Use Hondex-specific field layout
     *          - SONAR_UNKNOWN: Parse but don't extract (unrecognized sentence)
     *          
     *          Persistence: Retains value throughout sentence parsing. Reset when new sentence
     *          begins ('$' character triggers reset to SONAR_UNKNOWN).
     *          
     *          Example: For "$SDDBT,10.5,f,3.2,M,1.75,F*3A"
     *          - Term 0 is "DBT" -> _sentence_type set to SONAR_DBT
     *          - Term 3 parsing knows to extract "3.2" as distance in meters
     */
    sentence_types _sentence_type;
    
    /**
     * @brief Flag indicating current sentence has been fully processed
     * 
     * @details Tracks whether the current sentence has completed parsing and validation.
     *          Prevents duplicate processing of the same sentence data. Set to true after
     *          successful checksum validation and data extraction.
     *          
     *          State transitions:
     *          - false: Sentence in progress (parsing fields or validating)
     *          - true: Sentence complete (data extracted, awaiting next sentence)
     *          
     *          Set to true when:
     *          - Checksum validated successfully
     *          - Data fields extracted and stored
     *          - decode() returns true to signal completion
     *          
     *          Reset to false when:
     *          - New sentence starts ('$' delimiter encountered)
     *          - Parser reset (error recovery)
     *          
     *          Purpose: Ensures decode() returns true only once per sentence even if called
     *          multiple times with no new data. Prevents get_reading() from returning the same
     *          measurement multiple times.
     */
    bool _sentence_done;
    
    /** @} */ // End of NMEA Parser State Machine Variables group
};

#endif  // AP_RANGEFINDER_NMEA_ENABLED
