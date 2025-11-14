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
 * @file AP_RangeFinder_RDS02UF.h
 * @brief Benewake RDS02UF ultrasonic rangefinder backend driver
 * 
 * @details This file implements the serial protocol backend driver for the
 *          Benewake RDS02UF ultrasonic rangefinder. The RDS02UF is an ultrasonic
 *          distance sensor with a range of 1.5m to 20m, communicating via a
 *          serial protocol with structured 21-byte packets.
 *          
 *          The driver handles:
 *          - Serial communication and packet parsing
 *          - Distance measurement reading and validation
 *          - Error detection and range limiting
 *          - Buffer management for incoming data
 *          
 *          Protocol: 115200 baud, 21-byte packets with header/footer magic bytes
 *          and CRC8 checksum validation.
 * 
 * @note This driver requires a serial port configured for the rangefinder
 * @see AP_RangeFinder_Backend_Serial for base serial rangefinder functionality
 */

#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_RDS02UF_ENABLED

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend_Serial.h"

/// Maximum buffer size for RDS02UF serial data parsing (bytes)
#define RDS02_BUFFER_SIZE           50

/// Maximum measurement distance for RDS02UF sensor (meters)
#define RDS02UF_DIST_MAX            20.00

/// Minimum measurement distance for RDS02UF sensor (meters)
#define RDS02UF_DIST_MIN            1.50

/// Length of the data payload field in RDS02UF packet (bytes)
#define RDS02UF_DATA_LEN            10

/**
 * @class AP_RangeFinder_RDS02UF
 * @brief Backend driver for Benewake RDS02UF ultrasonic rangefinder
 * 
 * @details This class implements the serial protocol driver for the Benewake RDS02UF
 *          ultrasonic distance sensor. The RDS02UF communicates via UART using a
 *          structured 21-byte packet protocol with the following characteristics:
 *          
 *          - Measurement range: 1.5m to 20m
 *          - Protocol: Serial UART at 115200 baud (8N1)
 *          - Packet structure: 21 bytes with header/footer magic values
 *          - Validation: CRC8 checksum for data integrity
 *          - Update rate: Typically 10-50 Hz depending on configuration
 *          
 *          The driver continuously reads serial data, searches for valid packet
 *          headers, validates packet structure and checksum, and extracts distance
 *          measurements. Invalid or out-of-range readings are rejected.
 *          
 *          Thread Safety: This driver assumes single-threaded access via the
 *          rangefinder backend scheduler task.
 * 
 * @note Distance measurements are reported in meters
 * @note Sensor is reported as MAV_DISTANCE_SENSOR_RADAR type to MAVLink
 * @see AP_RangeFinder_Backend_Serial for base implementation
 */
class AP_RangeFinder_RDS02UF : public AP_RangeFinder_Backend_Serial
{

public:

    /**
     * @brief Factory method to create RDS02UF rangefinder backend instance
     * 
     * @details Creates a new AP_RangeFinder_RDS02UF backend driver instance using
     *          dynamic memory allocation. This factory method is called by the
     *          rangefinder frontend when a RDS02UF sensor is configured.
     *          
     *          Memory allocation uses NEW_NOTHROW to safely handle allocation
     *          failures without exceptions (returns nullptr on failure).
     * 
     * @param[in,out] _state    Reference to rangefinder state structure for this sensor
     * @param[in]     _params   Reference to configuration parameters for this sensor
     * 
     * @return Pointer to newly created backend instance, or nullptr if allocation fails
     * 
     * @note Called during rangefinder initialization at boot time
     * @see AP_RangeFinder_Backend_Serial::AP_RangeFinder_Backend_Serial()
     */
    static AP_RangeFinder_Backend_Serial *create(
        RangeFinder::RangeFinder_State &_state,
        AP_RangeFinder_Params &_params) {
        return NEW_NOTHROW AP_RangeFinder_RDS02UF(_state, _params);
    }

protected:

    /**
     * @brief Get the MAVLink distance sensor type for this rangefinder
     * 
     * @details Returns the MAVLink sensor type identifier used in DISTANCE_SENSOR
     *          messages. The RDS02UF is reported as RADAR type rather than ULTRASOUND
     *          to maintain consistency with other serial rangefinders in the ArduPilot
     *          ecosystem.
     * 
     * @return MAV_DISTANCE_SENSOR_RADAR - MAVLink sensor type enumeration value
     * 
     * @note Despite being an ultrasonic sensor, this is reported as RADAR type
     * @see MAVLink DISTANCE_SENSOR message definition
     */
    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_RADAR;
    }

private:
    /// Inherit constructor from base class AP_RangeFinder_Backend_Serial
    using AP_RangeFinder_Backend_Serial::AP_RangeFinder_Backend_Serial;

    /**
     * @brief Find and align RDS02UF packet header in receive buffer
     * 
     * @details Searches the receive buffer for a valid RDS02UF packet header
     *          (0x55 0x55 magic bytes) starting at the specified offset. If a
     *          header is found, moves that packet (or partial packet) to the
     *          beginning of the buffer for further processing.
     *          
     *          This function implements packet synchronization by searching for
     *          the distinctive header pattern and discarding any leading garbage
     *          bytes from incomplete or corrupted transmissions.
     * 
     * @param[in] initial_offset Byte offset in buffer to begin header search
     * 
     * @note Modifies the internal buffer (u.parse_buffer) and body_length
     * @note Called repeatedly during serial data processing
     */
    void move_header_in_buffer(uint8_t initial_offset);

    /**
     * @brief Attempt to read a distance measurement from the sensor
     * 
     * @details Reads available serial data, searches for valid RDS02UF packets,
     *          validates packet structure and CRC8 checksum, and extracts the
     *          distance measurement. Multiple reads may be required to accumulate
     *          a complete 21-byte packet.
     *          
     *          Algorithm:
     *          1. Read available bytes from serial port into buffer
     *          2. Search for packet header (0x55 0x55)
     *          3. Validate packet length and structure
     *          4. Verify CRC8 checksum
     *          5. Extract distance from data payload
     *          6. Validate distance is within sensor range
     * 
     * @param[out] reading_m Distance measurement in meters if successful
     * 
     * @return true if a valid distance reading was obtained, false otherwise
     * 
     * @note Called periodically by the rangefinder scheduler task
     * @note Invalid readings (bad CRC, out of range) return false
     * @see move_header_in_buffer() for packet synchronization
     */
    bool get_reading(float &reading_m) override;

    /**
     * @brief Get the serial read timeout for this sensor
     * 
     * @details Returns the maximum time to wait for serial data before considering
     *          the sensor unhealthy. A 500ms timeout accommodates update rates from
     *          2 Hz to typical 10-50 Hz with margin for transmission delays.
     * 
     * @return 500 - Timeout in milliseconds
     * 
     * @note Sensor marked unhealthy if no valid data received within this timeout
     */
    uint16_t read_timeout_ms() const override { return 500; }

    /**
     * @brief Get the maximum valid measurement distance for this sensor
     * 
     * @details Returns the effective maximum range by taking the minimum of:
     *          - User-configured maximum distance parameter
     *          - Sensor hardware maximum capability (20.0m for RDS02UF)
     *          
     *          This ensures distance readings beyond sensor capability are properly
     *          marked as out-of-range even if user configures a larger maximum.
     * 
     * @return Maximum valid distance in meters (≤ 20.0m)
     * 
     * @note Readings exceeding this distance are marked as out-of-range
     * @see RDS02UF_DIST_MAX for hardware maximum
     */
    float max_distance() const override  {
        return MIN(AP_RangeFinder_Backend::max_distance(), RDS02UF_DIST_MAX);
    }

    /**
     * @brief Get the minimum valid measurement distance for this sensor
     * 
     * @details Returns the effective minimum range by taking the maximum of:
     *          - User-configured minimum distance parameter
     *          - Sensor hardware minimum capability (1.5m for RDS02UF)
     *          
     *          This ensures distance readings below sensor capability are properly
     *          marked as out-of-range even if user configures a smaller minimum.
     *          The RDS02UF has a significant minimum range due to ultrasonic
     *          transducer ringdown time.
     * 
     * @return Minimum valid distance in meters (≥ 1.5m)
     * 
     * @note Readings below this distance are marked as out-of-range
     * @see RDS02UF_DIST_MIN for hardware minimum
     */
    float min_distance() const override {
        return MAX(AP_RangeFinder_Backend::min_distance(), RDS02UF_DIST_MIN);
    }

    /**
     * @struct RDS02UFPacket
     * @brief Benewake RDS02UF serial protocol packet structure
     * 
     * @details Data format for Benewake RDS02UF rangefinder serial communication.
     *          Total packet size: 21 bytes with fixed header/footer magic values
     *          for packet synchronization and CRC8 checksum for validation.
     *          
     *          Packet Structure (21 bytes total):
     *          - Bytes 0-1:   Header magic (0x55 0x55) - packet start marker
     *          - Byte 2:      Address - device address on multi-drop bus
     *          - Byte 3:      Error code - sensor status/error flags
     *          - Bytes 4-5:   Function code (low/high bytes) - command/response type
     *          - Bytes 6-7:   Length (low/high bytes) - payload length indicator
     *          - Bytes 8-17:  Real data (10 bytes) - distance and auxiliary data
     *          - Byte 18:     CRC8 checksum - computed over address through data
     *          - Bytes 19-20: Footer magic (0xAA 0xAA) - packet end marker
     *          
     *          The distance measurement is typically encoded in the first bytes
     *          of the data field, with format specified by the function code.
     * 
     * @note PACKED attribute ensures no padding bytes in structure
     * @note Structure size must exactly match 21-byte wire protocol
     */
    struct PACKED RDS02UFPacket {
        uint8_t headermagic1;   ///< Header magic byte 1 (0x55)
        uint8_t headermagic2;   ///< Header magic byte 2 (0x55)
        uint8_t address;        ///< Device address (multi-drop support)
        uint8_t error_code;     ///< Sensor error/status code
        uint8_t fc_low;         ///< Function code low byte
        uint8_t fc_high;        ///< Function code high byte
        uint8_t length_l;       ///< Data length low byte
        uint8_t length_h;       ///< Data length high byte
        uint8_t data[RDS02UF_DATA_LEN];  ///< Measurement data payload (10 bytes)
        uint8_t checksum;       ///< CRC8 checksum for packet validation
        uint8_t footermagic1;   ///< Footer magic byte 1 (0xAA)
        uint8_t footermagic2;   ///< Footer magic byte 2 (0xAA)
    };

    /**
     * @union RDS02UF_Union
     * @brief Union for efficient packet parsing and byte-level access
     * 
     * @details Provides dual interpretation of the same memory region as either:
     *          - Raw byte array for serial data accumulation
     *          - Structured packet for field access after validation
     *          
     *          This allows efficient byte-by-byte reading from the serial port
     *          into parse_buffer, followed by structured access to packet fields
     *          once a complete valid packet has been identified.
     * 
     * @note Both members overlay the same 21 bytes of memory
     */
    union RDS02UF_Union {
        uint8_t parse_buffer[21];        ///< Raw byte buffer for serial data accumulation
        struct RDS02UFPacket packet;     ///< Structured packet view for field access
    };

    /// Packet parsing buffer and structured packet view
    RDS02UF_Union u;

    /// Number of valid bytes currently accumulated in the parse buffer
    uint8_t body_length;
};
#endif  // AP_RANGEFINDER_RDS02UF_ENABLED
