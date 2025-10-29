/**
 * @file AP_RangeFinder_GYUS42v2.h
 * @brief Driver for GY-US42v2 ultrasonic rangefinder using serial interface
 * 
 * @details This file implements support for the GY-US42v2 ultrasonic distance sensor.
 *          The GY-US42v2 is a compact ultrasonic rangefinder that supports both I2C 
 *          (address 0x70) and serial interfaces. This backend implementation uses the
 *          serial interface with a command/response protocol at 9600 baud.
 * 
 *          Hardware specifications:
 *          - Maximum range: 8 meters
 *          - Minimum range: 0.2 meters (typical)
 *          - Resolution: 1 cm
 *          - I2C address: 0x70 (when using I2C interface - not used by this backend)
 *          - Serial baudrate: 9600 bps
 *          - Communication: Command/response protocol via UART
 * 
 *          This implementation uses a 7-byte buffer to read distance measurements
 *          from the sensor via serial communication and includes signature byte
 *          detection for protocol synchronization.
 * 
 * @note This backend uses the serial interface; for I2C interface, a separate backend
 *       would be required.
 * 
 * @see AP_RangeFinder_Backend_Serial
 */

#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_GYUS42V2_ENABLED

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend_Serial.h"

/**
 * @class AP_RangeFinder_GYUS42v2
 * @brief Backend driver for GY-US42v2 ultrasonic rangefinder via serial interface
 * 
 * @details This class provides the serial communication interface for the GY-US42v2
 *          ultrasonic distance sensor. The sensor communicates using a command/response
 *          protocol over UART at 9600 baud. The driver reads distance measurements
 *          from a 7-byte serial data packet that includes a signature byte for
 *          synchronization.
 * 
 *          Protocol details:
 *          - Data format: 7-byte packets with signature byte
 *          - Baud rate: 9600 bps
 *          - No parity, 8 data bits, 1 stop bit (standard)
 * 
 *          Usage pattern:
 *          1. Sensor continuously transmits distance readings
 *          2. Driver searches for signature byte in incoming data
 *          3. Once synchronized, reads complete 7-byte packets
 *          4. Extracts distance measurement in meters
 * 
 *          Sensor characteristics:
 *          - Type: Ultrasonic (reported as MAV_DISTANCE_SENSOR_ULTRASOUND)
 *          - Maximum range: 8.0 meters
 *          - Update rate: Determined by sensor firmware (typically ~10Hz)
 * 
 * @note Inherits serial communication handling from AP_RangeFinder_Backend_Serial
 */
class AP_RangeFinder_GYUS42v2 : public AP_RangeFinder_Backend_Serial
{

public:

    /**
     * @brief Factory method to create an instance of the GY-US42v2 rangefinder backend
     * 
     * @details This static factory method creates a new instance of the GY-US42v2 driver
     *          on the heap using NEW_NOTHROW for safe memory allocation in the embedded
     *          environment. Called by the rangefinder library when a GY-US42v2 sensor is
     *          configured.
     * 
     * @param[in,out] _state Reference to rangefinder state structure for storing measurements
     * @param[in] _params Reference to rangefinder parameters (serial port, orientation, etc.)
     * 
     * @return Pointer to newly created AP_RangeFinder_GYUS42v2 instance, or nullptr if allocation fails
     * 
     * @note Uses NEW_NOTHROW to handle memory allocation failures gracefully
     * @see AP_RangeFinder_Backend_Serial
     */
    static AP_RangeFinder_Backend_Serial *create(
        RangeFinder::RangeFinder_State &_state,
        AP_RangeFinder_Params &_params) {
        return NEW_NOTHROW AP_RangeFinder_GYUS42v2(_state, _params);
    }

protected:

    /**
     * @brief Returns the MAVLink distance sensor type for this rangefinder
     * 
     * @details Returns MAV_DISTANCE_SENSOR_ULTRASOUND to identify this as an ultrasonic
     *          distance sensor in MAVLink telemetry messages. This information is used
     *          by ground control stations and companion computers to understand the
     *          sensor type and its characteristics.
     * 
     * @return MAV_DISTANCE_SENSOR_ULTRASOUND indicating ultrasonic sensor type
     * 
     * @note This is an override of the base class virtual method
     * @see AP_RangeFinder_Backend::_get_mav_distance_sensor_type()
     */
    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_ULTRASOUND;
    }

    /**
     * @brief Returns the initial serial port baudrate for communication with the sensor
     * 
     * @details Specifies 9600 baud as the communication speed for the GY-US42v2 sensor's
     *          serial interface. This baudrate is used by the serial backend to configure
     *          the UART port during initialization. The GY-US42v2 uses standard serial
     *          format: 9600 bps, 8 data bits, no parity, 1 stop bit (9600-8-N-1).
     * 
     * @param[in] serial_instance The serial port instance number (unused for this sensor)
     * 
     * @return 9600 indicating 9600 bits per second baudrate
     * 
     * @note This is an override of the base class virtual method
     * @see AP_RangeFinder_Backend_Serial::initial_baudrate()
     */
    uint32_t initial_baudrate(uint8_t serial_instance) const override {
        return 9600;
    }

private:

    /**
     * @brief Inherit constructors from base class AP_RangeFinder_Backend_Serial
     * 
     * @details This using declaration makes the base class constructors available for
     *          construction of AP_RangeFinder_GYUS42v2 instances. The backend uses the
     *          base class constructor which accepts state and parameter references.
     */
    using AP_RangeFinder_Backend_Serial::AP_RangeFinder_Backend_Serial;

    /**
     * @brief Read and parse distance measurement from the sensor
     * 
     * @details Reads incoming serial data from the GY-US42v2 sensor, searches for the
     *          protocol signature byte to achieve synchronization, and extracts the
     *          distance measurement from the 7-byte data packet. This method is called
     *          periodically by the rangefinder scheduler to update the distance reading.
     * 
     *          Protocol processing:
     *          1. Read available bytes from serial port into buffer
     *          2. Search for signature byte using find_signature_in_buffer()
     *          3. Wait until complete 7-byte packet is received
     *          4. Extract and validate distance measurement
     *          5. Convert to meters and return via output parameter
     * 
     * @param[out] reading_m Distance measurement in meters (0.2 to 8.0 meters typical range)
     * 
     * @return true if valid distance reading obtained, false if no valid data available
     * 
     * @note This is an override of the base class virtual method
     * @note Called at scheduler rate (typically 20Hz) by rangefinder update loop
     * @see AP_RangeFinder_Backend::get_reading()
     * @see find_signature_in_buffer()
     */
    bool get_reading(float &reading_m) override;

    /**
     * @brief Find and synchronize to the protocol signature byte in the receive buffer
     * 
     * @details Searches the receive buffer starting at the specified position for the
     *          protocol signature byte that marks the beginning of a valid 7-byte data
     *          packet. When found, shifts the signature byte and any following bytes to
     *          the start of the buffer to maintain synchronization with the packet stream.
     * 
     *          This function is essential for protocol synchronization because:
     *          - Serial data can be received at any point in the packet stream
     *          - Initial connection may occur mid-packet
     *          - Noise or errors can cause loss of synchronization
     * 
     *          Buffer management:
     *          - Scans from start position to end of used buffer region
     *          - When signature found, moves it and following bytes to buffer[0]
     *          - Updates buffer_used to reflect compacted buffer contents
     * 
     * @param[in] start Starting position in buffer to begin signature search (0 to buffer_used-1)
     * 
     * @return true if signature byte found and buffer repositioned, false if not found
     * 
     * @note Modifies buffer contents by shifting data when signature is found
     * @note Updates buffer_used member to reflect new buffer state
     * @see get_reading()
     */
    bool find_signature_in_buffer(uint8_t start);

    /**
     * @brief Receive buffer for serial data from sensor (7-byte packet size)
     * 
     * @details Stores incoming serial data bytes from the GY-US42v2 sensor. The buffer
     *          size of 7 bytes matches the sensor's packet format which includes:
     *          - Signature/header byte for synchronization
     *          - Distance data bytes (format depends on sensor firmware)
     *          - Possible checksum or terminator bytes
     * 
     * @note Buffer is filled sequentially as serial data arrives
     * @see buffer_used
     */
    uint8_t buffer[7];

    /**
     * @brief Number of valid data bytes currently stored in the buffer
     * 
     * @details Tracks how many bytes of the 7-byte buffer contain valid received data.
     *          Used to determine when a complete packet has been received and to manage
     *          buffer filling during serial reception. Value ranges from 0 (empty) to 7
     *          (complete packet).
     * 
     *          Usage:
     *          - Incremented as each byte is received from serial port
     *          - Reset to 0 after processing complete packet
     *          - Adjusted by find_signature_in_buffer() during synchronization
     * 
     * @note Valid range: 0 to 7 bytes
     * @see buffer
     */
    uint8_t buffer_used;
};

#endif  // AP_RANGEFINDER_GYUS42V2_ENABLED
