/**
 * @file AP_RangeFinder_Benewake.h
 * @brief Benewake serial rangefinder base class for TF-series LiDAR sensors
 * 
 * @details This file implements the base class for Benewake TF-series serial LiDAR
 *          rangefinders. The Benewake family uses a common serial protocol with
 *          0x59 0x59 frame header for distance measurements.
 * 
 *          Supported sensor models (inherit from this base class):
 *          - TF02: Medium-range LiDAR (up to 22 meters)
 *          - TF03: Long-range LiDAR (up to 180 meters)  
 *          - TFmini: Short-range compact LiDAR (up to 12 meters)
 *          - TFminiPlus: Enhanced short-range LiDAR (up to 12 meters)
 * 
 *          Protocol Structure:
 *          Byte 0: 0x59 (header byte 1)
 *          Byte 1: 0x59 (header byte 2)
 *          Byte 2: Distance low byte (LSB)
 *          Byte 3: Distance high byte (MSB)
 *          Byte 4: Strength low byte (LSB)
 *          Byte 5: Strength high byte (MSB)
 *          Byte 6-7: Reserved or signal quality (model-specific)
 *          Byte 8: Checksum (sum of bytes 0-7, lower 8 bits)
 * 
 * @note Distance reported by sensor is in centimeters and converted to meters
 * @warning Serial baudrate varies by model: TFmini uses 115200, TF02/TF03 typically 115200
 * 
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_BENEWAKE_ENABLED

#include "AP_RangeFinder_Backend_Serial.h"
#include "AP_RangeFinder.h"

/**
 * @class AP_RangeFinder_Benewake
 * @brief Base class providing shared protocol parsing for Benewake TF-series serial LiDAR sensors
 * 
 * @details This abstract base class implements the common Benewake serial protocol used across
 *          the TF-series rangefinder family. It handles the serial frame parsing with 0x59 0x59
 *          header detection, distance extraction from low/high byte pairs, and checksum validation.
 * 
 *          Derived classes must implement model_dist_max_cm() to specify their maximum range
 *          and optionally override has_signal_byte() if the model reports signal quality.
 * 
 *          The protocol reports distance in centimeters which is automatically converted to
 *          meters for ArduPilot's internal representation.
 * 
 * @note This is an abstract base class - instantiate specific model classes (TF02, TF03, etc.)
 * @see AP_RangeFinder_Backend_Serial
 */
class AP_RangeFinder_Benewake : public AP_RangeFinder_Backend_Serial
{

public:

    using AP_RangeFinder_Backend_Serial::AP_RangeFinder_Backend_Serial;

protected:

    /**
     * @brief Get MAVLink distance sensor type identifier
     * 
     * @details Returns the MAVLink enum indicating this is a laser-based distance sensor.
     *          All Benewake TF-series sensors use laser ranging technology.
     * 
     * @return MAV_DISTANCE_SENSOR_LASER indicating laser/LiDAR sensor type
     */
    virtual MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_LASER;
    }

    /**
     * @brief Get model-specific maximum detection distance
     * 
     * @details Pure virtual method that must be implemented by derived classes to specify
     *          the maximum range capability of the specific Benewake sensor model.
     *          
     *          Typical values:
     *          - TFmini/TFminiPlus: 1200 cm (12 meters)
     *          - TF02: 2200 cm (22 meters)
     *          - TF03: 18000 cm (180 meters)
     * 
     * @return Maximum distance in centimeters that this sensor model can measure
     */
    virtual float model_dist_max_cm() const = 0;

    /**
     * @brief Check if this sensor model reports signal quality byte
     * 
     * @details Some Benewake models include signal quality/strength information in the
     *          serial data frame. Override this method to return true if the model
     *          uses the signal byte for reliability indication.
     * 
     * @return false by default; derived classes override if model reports signal quality
     */
    virtual bool has_signal_byte() const { return false; }

private:

    /**
     * @brief Parse Benewake serial protocol frame and extract distance measurement
     * 
     * @details Reads serial data to detect and parse Benewake protocol frames:
     *          1. Searches for 0x59 0x59 frame header in serial stream
     *          2. Collects frame bytes into line buffer (9 bytes total after header)
     *          3. Validates checksum (sum of bytes 0-7, lower 8 bits must match byte 8)
     *          4. Extracts distance from bytes 2-3 (low byte, high byte)
     *          5. Converts distance from centimeters to meters
     * 
     *          Frame format: [0x59][0x59][Dist_L][Dist_H][Strength_L][Strength_H][Reserved][Reserved][Checksum]
     * 
     *          The method is called by the rangefinder update loop to obtain the latest
     *          distance measurement from the sensor.
     * 
     * @param[out] reading_m Distance measurement in meters (converted from sensor's cm units)
     * 
     * @return true if valid frame received and distance extracted successfully
     * @return false if no valid frame available, checksum failed, or distance out of range
     * 
     * @note Distance is reported by sensor in centimeters, converted to meters for output
     * @note Checksum validation ensures data integrity of serial transmission
     */
    bool get_reading(float &reading_m) override;

    uint8_t linebuf[10];      ///< Serial data buffer for frame assembly (10 bytes including header)
    uint8_t linebuf_len;      ///< Current number of bytes collected in line buffer
};

#endif  // AP_RANGEFINDER_BENEWAKE_ENABLED
