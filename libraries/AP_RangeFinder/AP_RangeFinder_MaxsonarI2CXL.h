/**
 * @file AP_RangeFinder_MaxsonarI2CXL.h
 * @brief MaxBotix MB12xx I2C XL ultrasonic rangefinder backend driver
 * 
 * @details This file implements the driver for MaxBotix MB12xx series I2C XL 
 *          ultrasonic rangefinders. These sensors use I2C communication protocol 
 *          and provide distance measurements up to 7 meters with a wide beam pattern,
 *          making them suitable for obstacle detection and terrain following applications.
 * 
 *          Key Sensor Characteristics:
 *          - Communication: I2C protocol at standard I2C speeds
 *          - Maximum Range: 7 meters (700 cm)
 *          - Beam Pattern: Wide conical beam for broad coverage
 *          - Sensor Type: Ultrasonic distance sensor
 *          - Resolution: 1 cm
 * 
 * @note The wide beam pattern makes these sensors good for general obstacle detection
 *       but less suitable for precision applications requiring narrow beam focus.
 * 
 * @warning Ultrasonic sensors can be affected by acoustic noise, temperature gradients,
 *          and soft/angled surfaces that may absorb or deflect the acoustic signal.
 * 
 * Source: libraries/AP_RangeFinder/AP_RangeFinder_MaxsonarI2CXL.h
 */

#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_MAXSONARI2CXL_ENABLED

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend.h"

#include <AP_HAL/I2CDevice.h>

/// @brief Default I2C address for MaxBotix MB12xx I2C XL sensors (0x70)
#define AP_RANGE_FINDER_MAXSONARI2CXL_DEFAULT_ADDR   0x70

/// @brief I2C command byte to trigger a new range measurement (0x51)
#define AP_RANGE_FINDER_MAXSONARI2CXL_COMMAND_TAKE_RANGE_READING 0x51

/**
 * @class AP_RangeFinder_MaxsonarI2CXL
 * @brief Backend driver for MaxBotix MB12xx I2C XL ultrasonic rangefinders
 * 
 * @details This class implements the rangefinder backend for MaxBotix MB12xx series
 *          I2C XL ultrasonic distance sensors. The driver handles I2C communication,
 *          sensor initialization, periodic range measurements, and data validation.
 * 
 *          Sensor Communication Protocol:
 *          1. Send command byte 0x51 to trigger measurement
 *          2. Wait for sensor to complete measurement (typically 100ms)
 *          3. Read 2-byte distance value (high byte, low byte)
 *          4. Validate and update range state
 * 
 *          The driver operates asynchronously using timer callbacks to avoid
 *          blocking the main scheduler during I2C operations and sensor
 *          measurement delays.
 * 
 *          Technical Specifications:
 *          - Maximum Range: 7 meters (700 cm)
 *          - Minimum Range: ~20 cm (sensor dependent)
 *          - Resolution: 1 cm
 *          - Beam Width: Wide cone (~60-70 degrees typical)
 *          - Update Rate: Configurable via timer callback
 *          - I2C Address: 0x70 (default, may be configurable on some models)
 * 
 * @note The wide beam pattern provides good coverage for obstacle detection
 *       but may return the distance to the nearest object within the beam,
 *       which may not be directly ahead.
 * 
 * @warning Ensure adequate I2C bus timing for sensor measurement delays.
 *          Do not poll sensor faster than its measurement rate capability.
 * 
 * Source: libraries/AP_RangeFinder/AP_RangeFinder_MaxsonarI2CXL.h:15-48
 */
class AP_RangeFinder_MaxsonarI2CXL : public AP_RangeFinder_Backend
{
public:
    /**
     * @brief Static factory method to detect and instantiate MaxBotix MB12xx I2C XL sensor
     * 
     * @details Attempts to detect a MaxBotix MB12xx I2C XL sensor on the provided I2C device.
     *          Detection involves attempting I2C communication and verifying the device responds
     *          as expected. If successful, creates and returns a new driver instance.
     * 
     * @param[in,out] _state    Reference to rangefinder state structure for this sensor
     * @param[in,out] _params   Reference to rangefinder parameters for this sensor
     * @param[in]     dev       Pointer to initialized I2C device interface
     * 
     * @return Pointer to new AP_RangeFinder_MaxsonarI2CXL instance if detection successful,
     *         nullptr if sensor not detected or initialization failed
     * 
     * @note This function is called by the rangefinder driver during sensor auto-detection
     * @note The I2C device must be properly configured with correct address before calling
     */
    static AP_RangeFinder_Backend *detect(RangeFinder::RangeFinder_State &_state,
                                          AP_RangeFinder_Params &_params,
                                          AP_HAL::I2CDevice *dev);

    /**
     * @brief Update rangefinder state with latest measurement data
     * 
     * @details Called periodically by the rangefinder manager to update the sensor state.
     *          Processes new distance measurements received from the sensor via timer callback,
     *          validates the data, and updates the rangefinder state structure with current
     *          distance and status information.
     * 
     *          This method is non-blocking and simply transfers data from the timer callback
     *          context to the main loop context.
     * 
     * @note Called at main loop rate (typically 10-50 Hz depending on vehicle type)
     * @note Actual I2C communication occurs in timer callback, not in this function
     * 
     * Source: libraries/AP_RangeFinder/AP_RangeFinder_MaxsonarI2CXL.cpp
     */
    void update(void) override;

protected:

    /**
     * @brief Get MAVLink distance sensor type identifier
     * 
     * @details Returns the MAVLink sensor type identifier for telemetry reporting.
     *          For ultrasonic sensors, this returns MAV_DISTANCE_SENSOR_ULTRASOUND
     *          to properly identify the sensor technology to ground control stations.
     * 
     * @return MAV_DISTANCE_SENSOR_ULTRASOUND - MAVLink enum indicating ultrasonic sensor
     * 
     * @note This information is used in MAVLink DISTANCE_SENSOR messages
     */
    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_ULTRASOUND;
    }

private:
    /**
     * @brief Private constructor for MaxBotix MB12xx I2C XL rangefinder backend
     * 
     * @details Constructs a new driver instance for the MaxBotix MB12xx I2C XL sensor.
     *          This constructor is private and should only be called by the detect()
     *          factory method after successful sensor detection.
     * 
     * @param[in,out] _state   Reference to rangefinder state structure
     * @param[in,out] _params  Reference to rangefinder parameters
     * @param[in]     dev      Pointer to I2C device interface (must not be nullptr)
     * 
     * @note Constructor does not perform I2C communication; initialization
     *       occurs in _init() method
     */
    AP_RangeFinder_MaxsonarI2CXL(RangeFinder::RangeFinder_State &_state,
    								AP_RangeFinder_Params &_params,
                                 AP_HAL::I2CDevice *dev);

    /**
     * @brief Initialize the sensor and register timer callback
     * 
     * @details Performs sensor initialization including:
     *          - Verifying I2C communication
     *          - Configuring sensor parameters if needed
     *          - Registering periodic timer callback for asynchronous operation
     * 
     * @return true if initialization successful, false otherwise
     * 
     * @note Called once during sensor detection and setup
     * @note Failure typically indicates I2C communication problem or sensor not present
     */
    bool _init(void);

    /**
     * @brief Timer callback for asynchronous I2C operations and measurements
     * 
     * @details Executes periodically (typically 10-20 Hz) to handle sensor communication:
     *          1. Trigger new range measurement by sending command
     *          2. Wait appropriate time for measurement completion
     *          3. Read distance value from sensor
     *          4. Store result for update() to process
     * 
     *          Runs in timer callback context to avoid blocking main scheduler during
     *          I2C operations and sensor measurement delays (typically 100ms).
     * 
     * @note Executed in timer/interrupt context - must be fast and non-blocking
     * @note Uses I2C device transfer methods that are timer-callback safe
     * 
     * Source: libraries/AP_RangeFinder/AP_RangeFinder_MaxsonarI2CXL.cpp
     */
    void _timer(void);

    /// @brief Latest distance measurement from sensor in centimeters
    uint16_t distance;

    /// @brief Flag indicating new distance data is available for update()
    bool new_distance;
    
    /**
     * @brief Initiate a new range measurement
     * 
     * @details Sends the I2C command byte (0x51) to trigger a new ultrasonic
     *          measurement cycle. The sensor will begin transmitting an ultrasonic
     *          pulse and measuring the echo return time.
     * 
     * @return true if command sent successfully, false if I2C error occurred
     * 
     * @note After calling, must wait for sensor measurement time before reading result
     * @note Typical measurement time is ~100ms
     */
    bool start_reading(void);

    /**
     * @brief Read completed distance measurement from sensor
     * 
     * @details Reads 2-byte distance value from sensor via I2C. The sensor returns
     *          distance as high byte followed by low byte, representing distance
     *          in centimeters.
     * 
     * @param[out] reading_cm  Distance reading in centimeters if successful
     * 
     * @return true if read successful and value valid, false if I2C error or invalid data
     * 
     * @note Should only be called after sufficient time for measurement completion
     * @note Invalid readings (out of range) may be indicated by specific values
     */
    bool get_reading(uint16_t &reading_cm);

    /// @brief Pointer to I2C device interface for sensor communication
    AP_HAL::I2CDevice *_dev;
};

#endif  // AP_RANGEFINDER_MAXSONARI2CXL_ENABLED
