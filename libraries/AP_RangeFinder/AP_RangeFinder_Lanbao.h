/**
 * @file AP_RangeFinder_Lanbao.h
 * @brief Lanbao PSK-CM8JL65-CC5 serial LiDAR rangefinder driver
 * 
 * @details This driver implements support for the Lanbao PSK-CM8JL65-CC5 serial LiDAR
 *          rangefinder, which provides distance measurements up to 30 meters using a
 *          proprietary serial protocol at 115200 baud.
 *          
 *          Device Specifications:
 *          - Model: PSK-CM8JL65-CC5
 *          - Type: Serial LiDAR (laser-based distance sensor)
 *          - Maximum range: 30 meters
 *          - Communication: UART serial at 115200 baud (fixed)
 *          - Protocol: Proprietary 6-byte message format
 *          - Update rate: Continuous measurements
 *          
 *          The driver inherits from AP_RangeFinder_Backend_Serial to leverage
 *          common serial communication infrastructure and reading buffering.
 * 
 * @note This driver is enabled via AP_RANGEFINDER_LANBAO_ENABLED configuration flag
 * 
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_LANBAO_ENABLED

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend_Serial.h"

/**
 * @class AP_RangeFinder_Lanbao
 * @brief Driver for Lanbao PSK-CM8JL65-CC5 serial LiDAR rangefinder
 * 
 * @details Implements the serial protocol parser for Lanbao PSK-CM8JL65-CC5 LiDAR sensor.
 *          This sensor uses a proprietary 6-byte serial message format to report distance
 *          measurements. The driver continuously reads the serial port, validates incoming
 *          data packets, and extracts distance measurements.
 *          
 *          Serial Protocol:
 *          - Baud rate: 115200 (fixed, non-configurable)
 *          - Message format: 6-byte packets with checksum validation
 *          - Distance encoding: Binary format decoded in get_reading()
 *          
 *          Usage:
 *          - Set RNGFNDx_TYPE = [Lanbao type value] in parameters
 *          - Connect sensor TX to autopilot UART RX
 *          - Driver automatically handles serial initialization and reading
 *          
 *          Thread Safety:
 *          - Serial port reading occurs in sensor thread via scheduler callbacks
 *          - Distance updates protected by backend state locking mechanisms
 * 
 * @note This driver reports sensor type as MAV_DISTANCE_SENSOR_LASER to ground stations
 * @warning Ensure correct serial port assignment to avoid conflicts with other serial devices
 */
class AP_RangeFinder_Lanbao : public AP_RangeFinder_Backend_Serial
{

public:

    /**
     * @brief Factory method to create Lanbao rangefinder backend instance
     * 
     * @details Creates a new AP_RangeFinder_Lanbao driver instance using heap allocation.
     *          This static factory method is called by the rangefinder driver manager when
     *          a Lanbao sensor is configured via parameters.
     *          
     *          Memory allocation uses NEW_NOTHROW to handle potential heap exhaustion on
     *          memory-constrained flight controllers. If allocation fails, returns nullptr
     *          and the rangefinder instance will not be created.
     * 
     * @param[in,out] _state Reference to rangefinder state structure for storing measurements
     * @param[in]     _params Reference to rangefinder parameters (serial port, orientation, etc.)
     * 
     * @return Pointer to newly created backend instance, or nullptr if allocation fails
     * 
     * @note Called during rangefinder initialization, typically during vehicle startup
     * @see AP_RangeFinder_Backend_Serial for base class implementation details
     */
    static AP_RangeFinder_Backend_Serial *create(
        RangeFinder::RangeFinder_State &_state,
        AP_RangeFinder_Params &_params) {
        return NEW_NOTHROW AP_RangeFinder_Lanbao(_state, _params);
    }

    /**
     * @brief Get the fixed baud rate for Lanbao sensor communication
     * 
     * @details The Lanbao PSK-CM8JL65-CC5 sensor operates exclusively at 115200 baud
     *          and does not support baud rate negotiation or configuration. This method
     *          overrides the base class to provide the correct fixed baud rate for
     *          serial port initialization.
     *          
     *          The serial manager will configure the assigned UART to this baud rate
     *          during driver initialization.
     * 
     * @param[in] serial_instance Serial port instance number (unused, sensor always 115200)
     * 
     * @return uint32_t Fixed baud rate of 115200 bps
     * 
     * @note This baud rate is fixed by the sensor hardware and cannot be changed
     * @warning Using incorrect baud rate will result in communication failure and no distance readings
     */
    uint32_t initial_baudrate(uint8_t serial_instance) const override {
        return 115200;
    }

protected:

    /**
     * @brief Report sensor type to MAVLink ground control stations
     * 
     * @details Returns the MAVLink sensor type identifier for this rangefinder.
     *          The Lanbao PSK-CM8JL65-CC5 is a laser-based LiDAR sensor, so it
     *          reports as MAV_DISTANCE_SENSOR_LASER in telemetry messages.
     *          
     *          This information is included in DISTANCE_SENSOR MAVLink messages
     *          and helps ground stations properly interpret and display the
     *          sensor data.
     * 
     * @return MAV_DISTANCE_SENSOR MAV_DISTANCE_SENSOR_LASER (laser rangefinder type)
     * 
     * @note Used by GCS_MAVLink for DISTANCE_SENSOR message generation
     * @see MAVLink DISTANCE_SENSOR message specification
     */
    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_LASER;
    }

private:

    /**
     * @brief Inherit base class constructor for serial backend initialization
     * 
     * @details Uses the AP_RangeFinder_Backend_Serial constructor which handles
     *          common serial port setup, state management, and parameter initialization.
     */
    using AP_RangeFinder_Backend_Serial::AP_RangeFinder_Backend_Serial;

    /**
     * @brief Parse serial data and extract distance measurement
     * 
     * @details Reads bytes from the serial port, validates the 6-byte message format,
     *          performs checksum verification, and extracts the distance measurement.
     *          
     *          Protocol Implementation:
     *          - Reads bytes into buf[] until 6-byte message complete
     *          - Validates message header and checksum
     *          - Decodes distance from binary protocol format
     *          - Resets buffer for next message
     *          
     *          This method is called periodically by the scheduler through the backend
     *          update mechanism. Invalid messages are discarded and the buffer is reset
     *          to resynchronize with the data stream.
     * 
     * @param[out] reading_m Distance measurement in meters (0.0 to 30.0 meters)
     * 
     * @return true if valid reading successfully parsed and extracted
     * @return false if no complete message available or message validation failed
     * 
     * @note Called at backend update rate (typically 20-50 Hz depending on scheduler configuration)
     * @note Implementation in corresponding .cpp file handles protocol-specific parsing details
     * @warning Invalid checksums or malformed messages are rejected to prevent erroneous distance reports
     * 
     * @see AP_RangeFinder_Backend_Serial::update() for calling context
     */
    bool get_reading(float &reading_m) override;

    /**
     * @brief Serial receive buffer for 6-byte Lanbao protocol messages
     * 
     * @details Accumulates incoming serial bytes until a complete 6-byte message
     *          is received. The buffer is managed by get_reading() which fills it
     *          incrementally and processes it when full.
     *          
     *          Buffer size is exactly 6 bytes to match the Lanbao message format:
     *          - Byte 0: Message header/sync byte
     *          - Bytes 1-4: Distance data (encoding defined in .cpp implementation)
     *          - Byte 5: Checksum for message validation
     * 
     * @note Buffer is reset to empty state after successful message parsing or on validation errors
     */
    uint8_t buf[6];

    /**
     * @brief Current number of bytes accumulated in receive buffer
     * 
     * @details Tracks the fill level of buf[] as bytes arrive from the serial port.
     *          Valid range is 0-6, where 6 indicates a complete message ready for parsing.
     *          Reset to 0 after message processing or on buffer overflow.
     * 
     * @note Initialized to 0, incremented as bytes received, reset after message processing
     */
    uint8_t buf_len = 0;
};

#endif  // AP_RANGEFINDER_LANBAO_ENABLED
