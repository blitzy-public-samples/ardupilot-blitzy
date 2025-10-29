/**
 * @file AP_RangeFinder_Benewake_TFMiniPlus.h
 * @brief Driver for Benewake TFminiPlus 12-meter LiDAR rangefinder with I2C interface
 * 
 * @details This driver implements support for the Benewake TFminiPlus LiDAR distance sensor,
 *          an enhanced version of the original TFmini with improved performance characteristics.
 *          The TFminiPlus offers superior outdoor performance, extended range up to 12 meters,
 *          and better immunity to ambient light interference compared to the TFmini.
 * 
 *          Key Features:
 *          - Maximum range: 12 meters (vs 11 meters on TFmini)
 *          - Enhanced outdoor performance with better sunlight immunity
 *          - Improved accuracy and reliability
 *          - I2C communication interface
 *          - Configurable update rate
 *          - Built-in signal strength reporting
 * 
 *          The driver communicates via I2C protocol at the default address 0x10 and processes
 *          raw distance measurements with signal strength validation to ensure data quality.
 * 
 * @note This driver uses I2C interface. For serial/UART communication, use the TFmini serial driver.
 * 
 * @warning Ensure proper I2C bus configuration and address uniqueness to avoid conflicts.
 * 
 * Copyright (C) 2019  Lucas De Marchi <lucas.de.marchi@gmail.com>
 *
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_BENEWAKE_TFMINIPLUS_ENABLED

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend.h"

/**
 * @def TFMINIPLUS_ADDR_DEFAULT
 * @brief Default I2C device address for TFminiPlus sensor
 * 
 * @details The TFminiPlus uses 0x10 as its factory default I2C address.
 *          This address can be reconfigured if multiple sensors are needed on the same bus.
 */
#define TFMINIPLUS_ADDR_DEFAULT              0x10        // TFminiPlus default device id

#include <AP_HAL/utility/sparse-endian.h>
#include <AP_HAL/I2CDevice.h>

/**
 * @class AP_RangeFinder_Benewake_TFMiniPlus
 * @brief Benewake TFminiPlus 12-meter LiDAR rangefinder driver with I2C interface
 * 
 * @details This class implements the backend driver for the Benewake TFminiPlus LiDAR sensor,
 *          providing distance measurements up to 12 meters with enhanced outdoor performance.
 *          The TFminiPlus is an improved version of the TFmini with better ambient light rejection,
 *          extended range, and improved reliability in challenging environments.
 * 
 *          Protocol Details:
 *          - Communication: I2C interface at default address 0x10
 *          - Data format: Little-endian 16-bit distance and strength values
 *          - Update rate: Configurable, typically 100Hz
 *          - Data validation: Checksum verification on received packets
 *          - Signal strength: Reported with each measurement for quality assessment
 * 
 *          Performance Improvements over TFmini:
 *          - Extended maximum range: 12m vs 11m
 *          - Better outdoor performance with enhanced sunlight immunity
 *          - Improved accuracy and consistency
 *          - Lower susceptibility to ambient light interference
 *          - Enhanced reliability in varied environmental conditions
 * 
 *          Driver Operation:
 *          - Detection phase: Probes I2C bus for device presence
 *          - Initialization: Configures sensor and validates communication
 *          - Runtime: Periodic timer-based reading and accumulation of distance samples
 *          - Filtering: Accumulates multiple samples for noise reduction
 * 
 * @note Requires I2C bus with proper pullup resistors (typically 4.7kΩ)
 * 
 * @warning Ensure I2C bus speed is compatible (typically 400kHz fast mode)
 * 
 * @see AP_RangeFinder_Backend for base class interface
 * @see AP_RangeFinder for rangefinder management
 */
class AP_RangeFinder_Benewake_TFMiniPlus : public AP_RangeFinder_Backend
{

public:
    /**
     * @brief Static factory method to detect and instantiate TFminiPlus sensor
     * 
     * @details Probes the I2C bus for a TFminiPlus sensor at the specified device address.
     *          If the sensor is successfully detected and initialized, returns a pointer
     *          to a new driver instance. This method is called by the rangefinder manager
     *          during the sensor detection phase at system startup.
     * 
     *          Detection sequence:
     *          1. Verify I2C device responds at expected address
     *          2. Attempt to read sensor identification/status
     *          3. Validate response format and checksum
     *          4. If successful, create and initialize driver instance
     * 
     * @param[in,out] _state      Reference to rangefinder state structure for storing measurements
     * @param[in]     _params     Reference to rangefinder configuration parameters
     * @param[in]     dev         I2C device interface handle (ownership transferred if successful)
     * 
     * @return Pointer to new driver instance if detection successful, nullptr otherwise
     * 
     * @note This method takes ownership of the I2C device handle if successful
     * @note Called during system initialization on the main thread
     */
    static AP_RangeFinder_Backend *detect(RangeFinder::RangeFinder_State &_state,
                                          AP_RangeFinder_Params &_params,
                                          AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    /**
     * @brief Update rangefinder state with latest distance measurement
     * 
     * @details Called by the rangefinder manager to retrieve the latest distance reading
     *          from the sensor. This method processes accumulated samples from the timer
     *          callback, averages them for noise reduction, and updates the state structure
     *          with the filtered distance value.
     * 
     *          Update process:
     *          1. Check if new samples are available from timer callback
     *          2. Calculate average of accumulated distance samples
     *          3. Apply configured offset and scaling parameters
     *          4. Validate distance is within sensor range (0.1m to 12m)
     *          5. Update state with filtered distance or timeout status
     * 
     * @note Called at main loop rate (typically 50-400Hz depending on vehicle)
     * @note Thread-safe: Uses atomic operations for accumulator access
     * 
     * @see timer() for raw sample acquisition
     */
    void update(void) override;

protected:

    /**
     * @brief Get MAVLink distance sensor type identifier
     * 
     * @details Returns the MAVLink enum value identifying this as a laser-based distance sensor.
     *          This information is used by ground control stations and telemetry systems to
     *          properly interpret and display rangefinder data.
     * 
     * @return MAV_DISTANCE_SENSOR_LASER indicating laser/LiDAR sensor type
     * 
     * @note Used by MAVLink DISTANCE_SENSOR message generation
     */
    virtual MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_LASER;
    }

private:
    /**
     * @brief Private constructor for TFminiPlus driver instance
     * 
     * @details Constructs a new driver instance with the provided state, parameters, and I2C device.
     *          This constructor is private and only called by the detect() factory method after
     *          successful sensor detection. The constructor initializes internal state but does
     *          not perform I2C communication.
     * 
     * @param[in,out] _state  Reference to rangefinder state for storing measurements
     * @param[in]     _params Reference to rangefinder configuration parameters
     * @param[in]     dev     I2C device interface handle (ownership transferred)
     * 
     * @note Constructor does not perform I2C communication - init() must be called separately
     * @see detect() for public instantiation interface
     */
    AP_RangeFinder_Benewake_TFMiniPlus(RangeFinder::RangeFinder_State &_state,
                                       AP_RangeFinder_Params &_params,
                                       AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    /**
     * @brief Initialize sensor and start periodic timer callback
     * 
     * @details Performs sensor initialization including:
     *          - Registering periodic timer callback for sample acquisition
     *          - Initializing accumulator for sample averaging
     *          - Configuring I2C transfer parameters
     * 
     *          The timer is registered at 20Hz (50ms period) to read sensor data
     *          and accumulate samples for noise reduction.
     * 
     * @return true if initialization successful, false on failure
     * 
     * @note Called once after construction before sensor is used
     * @note Failure typically indicates I2C bus or configuration issues
     */
    bool init();

    /**
     * @brief Timer callback for reading raw sensor data
     * 
     * @details Periodic callback function registered with the I2C device to read raw distance
     *          and signal strength data from the TFminiPlus sensor. This function runs at the
     *          timer rate (typically 20Hz) and accumulates samples for averaging.
     * 
     *          Timer callback sequence:
     *          1. Initiate I2C read transaction to retrieve sensor data packet
     *          2. Validate packet checksum
     *          3. Extract distance and signal strength values (little-endian format)
     *          4. Process raw measurements and validate range
     *          5. Accumulate valid samples in sum/count accumulators
     * 
     * @note Executes in timer interrupt context - must be fast and non-blocking
     * @note Accumulator updated atomically for thread safety with update()
     * 
     * @warning Do not perform blocking operations in timer callback
     * @see update() for processing accumulated samples
     */
    void timer();

    /**
     * @brief Process raw distance and strength measurements from sensor
     * 
     * @details Converts raw little-endian sensor data to distance in centimeters and validates
     *          the measurement based on signal strength and sensor range limits. The TFminiPlus
     *          reports distance in millimeters and signal strength as a quality indicator.
     * 
     *          Processing steps:
     *          1. Convert little-endian distance from millimeters to centimeters
     *          2. Evaluate signal strength for measurement validity
     *          3. Check distance is within sensor operating range (10cm to 1200cm)
     *          4. Set output distance or mark as invalid based on validation
     * 
     * @param[in]  distance_raw       Raw distance value in millimeters (little-endian)
     * @param[in]  strength_raw       Signal strength indicator (little-endian)
     * @param[out] output_distance_cm Processed distance in centimeters (valid if within range)
     * 
     * @note Signal strength below threshold indicates unreliable measurement
     * @note Invalid measurements set output_distance_cm to 0
     */
    void process_raw_measure(le16_t distance_raw, le16_t strength_raw,
                             uint16_t &output_distance_cm);

    /**
     * @brief Verify packet checksum for data integrity
     * 
     * @details Validates the checksum of received I2C data packets to ensure data integrity.
     *          The TFminiPlus uses a simple additive checksum algorithm where the checksum byte
     *          equals the lower 8 bits of the sum of all preceding bytes in the packet.
     * 
     *          Checksum calculation:
     *          - Sum all bytes except the checksum byte itself
     *          - Compare lower 8 bits of sum with received checksum byte
     *          - Return true if match, false if mismatch
     * 
     * @param[in] arr     Pointer to data packet buffer including checksum
     * @param[in] pkt_len Total packet length in bytes including checksum
     * 
     * @return true if checksum valid, false if checksum mismatch indicates corrupted data
     * 
     * @note Checksum failures typically indicate I2C communication errors or EMI
     * @note Failed checksums result in measurement being discarded
     */
    bool check_checksum(uint8_t *arr, int pkt_len);

    /**
     * @brief I2C device interface handle
     * 
     * @details Owns the I2C device interface for communicating with the TFminiPlus sensor.
     *          Used for both blocking and non-blocking I2C transfers to read sensor data.
     */
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;

    /**
     * @brief Sample accumulator for noise reduction
     * 
     * @details Accumulates multiple distance samples between update() calls to reduce measurement
     *          noise through averaging. The timer callback adds samples to the sum and increments
     *          the count, while update() calculates the average and resets the accumulator.
     * 
     *          Thread safety: Modified by timer callback, read/reset by update() on main thread.
     *          Uses implicit atomic operations for sum and count.
     */
    struct {
        uint32_t sum;      ///< Accumulated sum of distance samples in centimeters
        uint32_t count;    ///< Number of samples accumulated since last update
    } accum;
};

#endif
