/**
 * @file AP_Airspeed_NMEA.h
 * @brief NMEA 0183 serial protocol airspeed sensor backend
 * 
 * @details This backend implements an airspeed sensor using NMEA 0183
 *          serial protocol, primarily for marine/underwater vehicle applications
 *          (ArduSub). It parses NMEA sentences to extract water speed and
 *          temperature data:
 *          - VHW (Water Speed and Heading) sentence for velocity measurement
 *          - MTW (Water Temperature) sentence for temperature measurement
 *          
 *          The backend validates data integrity using XOR checksum verification
 *          and provides direct velocity measurements without requiring pressure
 *          sensor calculations.
 * 
 * @note Designed for underwater vehicles where water speed is measured directly
 *       rather than calculated from differential pressure
 * 
 * Source: libraries/AP_Airspeed/AP_Airspeed_NMEA.h
 */

#pragma once

#include "AP_Airspeed_config.h"

#if AP_AIRSPEED_NMEA_ENABLED

#include "AP_Airspeed_Backend.h"
#include <AP_HAL/AP_HAL.h>

/**
 * @class AP_Airspeed_NMEA
 * @brief NMEA 0183 protocol airspeed sensor backend for water speed measurement
 * 
 * @details This backend parses NMEA 0183 sentences received over serial UART
 *          to extract water speed and temperature measurements. The implementation
 *          handles two primary NMEA sentence types:
 *          
 *          VHW Sentence Format: $--VHW,x.x,T,x.x,M,x.x,N,x.x,K*hh
 *          - Provides water speed and heading information
 *          - Speed extracted in knots (converted to m/s internally)
 *          
 *          MTW Sentence Format: $--MTW,x.x,C*hh
 *          - Provides water temperature in Celsius
 *          
 *          The parser implements:
 *          - Comma-delimited field parsing with term buffering
 *          - XOR checksum validation for data integrity (*hh field)
 *          - 100ms data freshness timeout
 *          - Sentence type detection and routing
 *          
 *          Unlike pressure-based airspeed sensors, this backend provides
 *          direct velocity measurements from the NMEA device, bypassing
 *          pressure-to-velocity conversion calculations.
 * 
 * @note Primarily used in ArduSub for underwater vehicles with water speed sensors
 * @warning XOR checksum validation is critical to prevent corrupted serial data
 *          from affecting vehicle control decisions
 * 
 * @see AP_Airspeed_Backend for base class interface
 * 
 * Source: libraries/AP_Airspeed/AP_Airspeed_NMEA.h
 */
class AP_Airspeed_NMEA : public AP_Airspeed_Backend
{
public:

    /**
     * @brief Constructor using base class constructor
     * 
     * @details Inherits AP_Airspeed_Backend constructor to initialize
     *          the airspeed backend with frontend reference and sensor instance
     */
    using AP_Airspeed_Backend::AP_Airspeed_Backend;

    /**
     * @brief Initialize and configure NMEA sentence reception
     * 
     * @details Configures the serial UART interface for NMEA sentence reception.
     *          Sets up communication parameters and prepares the parser state
     *          machine for sentence decoding.
     * 
     * @return true if initialization successful and UART available, false otherwise
     * 
     * @note Called once during sensor initialization phase
     */
    bool init(void) override;

    /**
     * @brief Indicate that this sensor provides airspeed (water speed) measurements
     * 
     * @details Returns true to indicate this backend provides direct velocity
     *          measurements from the NMEA device, not calculated from pressure.
     * 
     * @return true - this backend always provides direct velocity data
     * 
     * @note Unlike pressure-based airspeed sensors, this bypasses pressure
     *       calculation and provides water speed directly from NMEA sentences
     */
    bool has_airspeed() override {return true;}

    /**
     * @brief Read water speed from the NMEA sensor
     * 
     * @details Returns the last successfully parsed water speed value from
     *          VHW NMEA sentences. Data is considered fresh if received within
     *          the last 100ms. The speed value is extracted directly from NMEA
     *          sentences and converted to m/s, bypassing pressure-to-velocity
     *          calculations used by differential pressure sensors.
     * 
     * @param[out] airspeed Water speed in meters per second (m/s)
     * 
     * @return true if fresh data available (received within 100ms), false if stale
     * 
     * @note This provides water speed, not air speed, for underwater applications
     * @note Freshness timeout ensures sensor failures are detected quickly
     */
    bool get_airspeed(float &airspeed) override;

    /**
     * @brief Read water temperature from the NMEA sensor
     * 
     * @details Returns the averaged water temperature from MTW NMEA sentences.
     *          Temperature values are accumulated and averaged to reduce noise
     *          from individual measurements.
     * 
     * @param[out] temperature Water temperature in degrees Celsius
     * 
     * @return true if temperature data available, false otherwise
     * 
     * @note Temperature measurement is optional and depends on MTW sentence availability
     */
    bool get_temperature(float &temperature) override;


private:
    /**
     * @brief Serial UART interface pointer for NMEA sentence reception
     * 
     * @details Pointer to the HAL UART driver used for receiving NMEA 0183
     *          sentences from the connected sensor. Configured during init()
     *          and used by periodic timer callback to read serial data.
     */
    AP_HAL::UARTDriver *_uart = nullptr; 

    /**
     * @brief Add a single character to the buffer and attempt to decode
     * 
     * @details Implements character-by-character NMEA sentence parsing state
     *          machine. Accumulates characters into term buffer, detects
     *          sentence delimiters (comma, asterisk, newline), and triggers
     *          term decoding when complete. Maintains checksum accumulator
     *          for validation.
     * 
     * @param[in] c Single character received from serial UART
     * 
     * @return true if a complete sentence was successfully decoded and validated,
     *         false if still accumulating or validation failed
     * 
     * @note Called for each character received from serial interface
     * @note Checksum calculated as XOR of all characters between $ and *
     */
    bool decode(char c);

    /**
     * @brief Decode the just-completed term and update sentence state
     * 
     * @details Parses the current term buffer based on sentence type and term
     *          position. Extracts speed and temperature values from appropriate
     *          fields in VHW and MTW sentences. Performs checksum validation
     *          when processing checksum term.
     * 
     * @return true if complete sentence has passed checksum validation,
     *         false if still processing or checksum failed
     * 
     * @note VHW sentence uses term 7 (knots) for speed extraction
     * @note MTW sentence uses term 1 (Celsius) for temperature extraction
     * @warning Invalid checksum causes entire sentence to be rejected to
     *          prevent corrupted data from affecting vehicle control
     */
    bool decode_latest_term();

    /**
     * @enum sentence_types
     * @brief Enumeration of supported NMEA sentence types
     * 
     * @details Identifies which NMEA 0183 sentence format is currently being
     *          parsed. Used to route term parsing to appropriate field extraction
     *          logic for each sentence type.
     */
    enum sentence_types : uint8_t {
        TPYE_MTW = 0,  ///< MTW - Water Temperature sentence ($--MTW,x.x,C*hh)
        TYPE_VHW,       ///< VHW - Water Speed and Heading sentence ($--VHW,x.x,T,x.x,M,x.x,N,x.x,K*hh)
    };


    // ===== NMEA Sentence Parsing State Machine Members =====
    
    /**
     * @brief Buffer for current comma-delimited term within NMEA sentence
     * 
     * @details Accumulates characters for a single field (term) in the NMEA
     *          sentence until comma, asterisk, or newline delimiter encountered.
     *          Size of 15 characters accommodates typical NMEA field lengths.
     */
    char _term[15];
    
    /**
     * @brief Offset within term buffer for next character placement
     * 
     * @details Tracks current write position in _term buffer. Reset to 0
     *          when starting new term after delimiter.
     */
    uint8_t _term_offset;
    
    /**
     * @brief Term index within the current NMEA sentence (0-based)
     * 
     * @details Incremented at each comma delimiter. Used to identify which
     *          field is being parsed (e.g., term 7 in VHW is speed in knots).
     *          Reset to 0 at sentence start ($).
     */
    uint8_t _term_number;
    
    /**
     * @brief Parsed water speed in meters per second (m/s)
     * 
     * @details Extracted from VHW sentence term 7 (knots), converted to m/s.
     *          Updated when complete VHW sentence successfully validated.
     */
    float _speed;
    
    /**
     * @brief Parsed water temperature in degrees Celsius
     * 
     * @details Extracted from MTW sentence term 1. Updated when complete MTW
     *          sentence successfully validated.
     */
    float _temp;
    
    /**
     * @brief XOR checksum accumulator for current sentence
     * 
     * @details Computed as XOR of all characters between $ and * (exclusive).
     *          Compared against 2-digit hexadecimal checksum field after *
     *          for sentence validation.
     * 
     * @note Critical for detecting serial transmission errors and corrupted data
     */
    uint8_t _checksum;
    
    /**
     * @brief Flag indicating current term is the checksum field
     * 
     * @details Set to true when asterisk (*) delimiter detected, indicating
     *          next term is the 2-digit hex checksum for validation.
     */
    bool _term_is_checksum;
    
    /**
     * @brief Flag indicating current sentence decoding is complete
     * 
     * @details Set when newline detected, triggers final validation and
     *          data extraction from parsed sentence.
     */
    bool _sentence_done;
    
    /**
     * @brief Flag indicating sentence parsing validity so far
     * 
     * @details Set to false if checksum mismatch, buffer overflow, or
     *          parse error detected. Prevents corrupted data extraction.
     */
    bool _sentence_valid;
    
    /**
     * @brief Type of NMEA sentence currently being processed
     * 
     * @details Identified from sentence header (e.g., "MTW", "VHW") and
     *          used to route term parsing to appropriate field extraction.
     */
    sentence_types _sentence_type;

    // ===== Temperature Averaging Members =====
    
    /**
     * @brief Accumulator for temperature averaging in Celsius
     * 
     * @details Accumulates temperature values from multiple MTW sentences
     *          for noise reduction through averaging.
     */
    float _temp_sum;
    
    /**
     * @brief Count of temperature samples in accumulator
     * 
     * @details Number of MTW temperature readings accumulated in _temp_sum.
     *          Used to compute average when temperature requested.
     */
    uint16_t _temp_count;

    // ===== Last Valid Measurement Storage =====
    
    /**
     * @brief Last valid water temperature reading in degrees Celsius
     * 
     * @details Cached temperature value provided to get_temperature() calls.
     *          Updated when new MTW sentence validated. Retained during
     *          periods between measurements due to slow NMEA update rate.
     */
    float _last_temp;
    
    /**
     * @brief Last valid water speed reading in meters per second (m/s)
     * 
     * @details Cached speed value provided to get_airspeed() calls. Updated
     *          when new VHW sentence validated. Retained during periods
     *          between measurements due to slow NMEA update rate.
     */
    float _last_speed;

    /**
     * @brief Timestamp of last successful NMEA message reception in milliseconds
     * 
     * @details System millisecond timestamp when last valid VHW sentence was
     *          decoded. Used for 100ms freshness timeout - data older than
     *          100ms is considered stale and get_airspeed() returns false.
     * 
     * @note 100ms timeout ensures rapid detection of sensor communication failures
     */
    uint32_t _last_update_ms;
};

#endif  // AP_AIRSPEED_NMEA_ENABLED
