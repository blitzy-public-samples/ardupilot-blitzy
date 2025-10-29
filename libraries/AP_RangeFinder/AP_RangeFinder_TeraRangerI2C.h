/**
 * @file AP_RangeFinder_TeraRangerI2C.h
 * @brief TeraRanger I2C laser rangefinder backend driver
 * 
 * @details This file implements support for TeraRanger infrared Time-of-Flight
 *          (ToF) distance sensors communicating over I2C bus. Supports TeraRanger
 *          One and TeraRanger Evo series sensors which provide contactless distance
 *          measurement using infrared LED technology.
 * 
 *          TeraRanger sensors are compact, low-power laser rangefinders suitable
 *          for obstacle detection, altitude hold, and terrain following applications.
 *          They communicate via I2C protocol and provide fast update rates suitable
 *          for real-time distance measurement.
 * 
 *          Key Features:
 *          - Infrared Time-of-Flight technology
 *          - I2C communication interface
 *          - Fast update rate suitable for control loops
 *          - Compact form factor
 *          - Low power consumption
 * 
 *          Supported Models:
 *          - TeraRanger One: 0.2m to 14m range
 *          - TeraRanger Evo series: Various range configurations
 * 
 * @note The driver uses periodic I2C reads with accumulation for noise reduction
 * @warning Ensure correct I2C bus configuration and address for sensor detection
 * 
 * @see AP_RangeFinder_Backend
 * @see AP_HAL::I2CDevice
 */

#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_TRI2C_ENABLED

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend.h"

#include <AP_HAL/I2CDevice.h>

/**
 * @class AP_RangeFinder_TeraRangerI2C
 * @brief Backend driver for TeraRanger I2C infrared ToF laser rangefinders
 * 
 * @details Implements the rangefinder backend interface for TeraRanger series
 *          infrared Time-of-Flight sensors. The driver handles I2C communication,
 *          measurement triggering, raw data collection, and distance calculation.
 * 
 *          The measurement cycle consists of:
 *          1. Trigger measurement via I2C command
 *          2. Wait for sensor to complete measurement
 *          3. Read raw distance data over I2C
 *          4. Process and validate raw data
 *          5. Accumulate multiple readings for stability
 *          6. Update rangefinder state with filtered distance
 * 
 *          Thread Safety: Uses HAL scheduler timer callbacks for I2C operations
 *          to avoid blocking the main control loop.
 * 
 *          Hardware Interface: Standard I2C bus communication at 400kHz (Fast Mode)
 * 
 * @note Measurements are accumulated over multiple reads to reduce noise
 * @warning Sensor detection requires proper I2C bus initialization and correct device address
 * 
 * @see AP_RangeFinder_Backend for base class interface
 */
class AP_RangeFinder_TeraRangerI2C : public AP_RangeFinder_Backend
{
public:
    /**
     * @brief Detect and initialize a TeraRanger I2C sensor
     * 
     * @details Attempts to detect a TeraRanger sensor on the provided I2C device.
     *          This static factory method probes the I2C bus to verify sensor presence,
     *          creates a new driver instance if successful, and initializes the sensor
     *          for continuous operation.
     * 
     *          Detection Process:
     *          1. Verify I2C device is valid and responsive
     *          2. Create driver instance with provided state and parameters
     *          3. Initialize sensor communication and configuration
     *          4. Register periodic timer callback for measurements
     *          5. Return driver instance or nullptr on failure
     * 
     * @param[in,out] _state      Reference to rangefinder state structure for this sensor
     * @param[in]     _params     Reference to configuration parameters for this sensor
     * @param[in]     i2c_dev     Pointer to initialized I2C device interface
     * 
     * @return Pointer to new AP_RangeFinder_TeraRangerI2C instance on success, nullptr on failure
     * 
     * @note This method is called during rangefinder initialization/probing
     * @warning Caller is responsible for I2C device lifecycle management
     * 
     * @see AP_RangeFinder_Backend::detect()
     */
    static AP_RangeFinder_Backend *detect(RangeFinder::RangeFinder_State &_state,
                                          AP_RangeFinder_Params &_params,
                                          AP_HAL::I2CDevice *i2c_dev);

    /**
     * @brief Update rangefinder state with latest distance measurement
     * 
     * @details Retrieves accumulated distance measurements from the timer callback,
     *          calculates the averaged distance, and updates the rangefinder state.
     *          Called periodically by the rangefinder library at the configured update rate.
     * 
     *          Update Process:
     *          1. Lock access to accumulated measurement data
     *          2. Calculate average distance from accumulated samples
     *          3. Validate distance is within sensor range limits
     *          4. Update state with distance and status
     *          5. Reset accumulator for next update cycle
     * 
     * @note Called at main loop rate (typically 10-50Hz) from rangefinder library
     * @note Actual I2C measurements occur in timer() callback at higher rate
     * @warning Must complete quickly to avoid delaying main loop execution
     * 
     * @see timer() for the I2C measurement callback
     * @see AP_RangeFinder_Backend::update()
     */
    void update(void) override;

protected:

    /**
     * @brief Get MAVLink distance sensor type for telemetry reporting
     * 
     * @details Returns the MAVLink distance sensor type identifier for this sensor.
     *          TeraRanger sensors use infrared LED ToF technology but are classified
     *          as LASER type for MAVLink telemetry compatibility and ground station display.
     * 
     * @return MAV_DISTANCE_SENSOR_LASER indicating laser/LED rangefinder type
     * 
     * @note Used by telemetry system to inform ground control station of sensor type
     * @see AP_RangeFinder_Backend::get_mav_distance_sensor_type()
     */
    virtual MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_LASER;
    }

private:
    /**
     * @brief Private constructor for TeraRanger I2C driver instance
     * 
     * @details Constructs a new TeraRanger I2C driver instance. Constructor is private
     *          to enforce creation through the static detect() factory method, ensuring
     *          proper sensor detection and initialization sequence.
     * 
     * @param[in,out] _state   Reference to rangefinder state structure
     * @param[in]     _params  Reference to configuration parameters
     * @param[in]     i2c_dev  Pointer to initialized I2C device interface
     * 
     * @note Only called by detect() static factory method
     * @see detect() for sensor detection and driver creation
     */
    AP_RangeFinder_TeraRangerI2C(RangeFinder::RangeFinder_State &_state,
    								AP_RangeFinder_Params &_params,
                                 AP_HAL::I2CDevice *i2c_dev);

    /**
     * @brief Trigger a new distance measurement on the sensor
     * 
     * @details Sends the measurement trigger command to the TeraRanger sensor over I2C.
     *          The sensor will begin a ToF measurement cycle, which takes several milliseconds
     *          to complete. Actual measurement data is read later via collect_raw().
     * 
     * @return true if I2C trigger command sent successfully, false on I2C error
     * 
     * @note Called from timer() callback before attempting to read data
     * @warning Does not block waiting for measurement completion
     * 
     * @see collect_raw() for reading measurement results
     */
    bool measure(void);
    
    /**
     * @brief Read raw distance measurement from sensor
     * 
     * @details Reads the raw distance measurement data from the TeraRanger sensor via I2C.
     *          The sensor returns distance data in sensor-specific units that require
     *          processing before use.
     * 
     * @param[out] raw_distance  Raw distance value read from sensor (sensor-specific units)
     * 
     * @return true if data read successfully, false on I2C read error or invalid data
     * 
     * @note Must be called after measurement trigger with appropriate delay
     * @note Raw data may include special values indicating out-of-range or error conditions
     * 
     * @see measure() for triggering measurements
     * @see process_raw_measure() for converting raw data to usable distance
     */
    bool collect_raw(uint16_t &raw_distance);
    
    /**
     * @brief Process and validate raw sensor measurement
     * 
     * @details Converts raw sensor data to distance in centimeters and validates the measurement.
     *          Handles special sensor values indicating out-of-range conditions, errors, or
     *          invalid measurements. Filters out readings that exceed sensor specifications.
     * 
     *          Processing Steps:
     *          1. Check for special error/status values in raw data
     *          2. Convert raw sensor units to centimeters
     *          3. Validate distance is within sensor min/max range
     *          4. Apply any sensor-specific scaling or calibration
     * 
     * @param[in]  raw_distance  Raw distance value from sensor
     * @param[out] distance_cm   Processed distance in centimeters
     * 
     * @return true if measurement is valid and within range, false if out of range or error
     * 
     * @note Invalid measurements are rejected to prevent accumulation of bad data
     * @see collect_raw() for obtaining raw measurements
     */
    bool process_raw_measure(uint16_t raw_distance, uint16_t &distance_cm);

    /**
     * @brief Initialize sensor communication and configuration
     * 
     * @details Performs initial sensor setup including I2C communication verification
     *          and configuration. Registers a periodic timer callback for continuous
     *          measurement operation.
     * 
     *          Initialization Steps:
     *          1. Verify I2C communication with sensor
     *          2. Configure sensor operating mode if applicable
     *          3. Register timer callback for periodic measurements
     *          4. Set initial sensor state
     * 
     * @return true if initialization successful, false on failure
     * 
     * @note Called during driver construction via detect() method
     * @warning Initialization failure prevents driver from operating
     * 
     * @see detect() for driver creation flow
     */
    bool init(void);
    
    /**
     * @brief Periodic timer callback for I2C measurement operations
     * 
     * @details Called periodically by HAL scheduler to perform I2C measurement cycle:
     *          trigger measurement, read data, process result, and accumulate valid readings.
     *          Runs at higher rate than main update() to provide multiple samples for averaging.
     * 
     *          Timer Callback Sequence:
     *          1. Trigger new sensor measurement via measure()
     *          2. Read raw measurement data via collect_raw()
     *          3. Process and validate raw data via process_raw_measure()
     *          4. Accumulate valid measurements for later averaging
     * 
     * @note Runs in HAL scheduler timer context, not main thread
     * @note Typical callback rate: 50-100Hz for smooth data accumulation
     * @warning Must complete quickly to avoid scheduler overrun
     * @warning All I2C operations must be non-blocking
     * 
     * @see update() for consuming accumulated measurements
     * @see AP_HAL::I2CDevice for I2C communication
     */
    void timer(void);
    
    /**
     * @brief Pointer to I2C device interface for sensor communication
     * 
     * @details HAL I2C device handle used for all communication with the TeraRanger sensor.
     *          Provides methods for I2C transfers including reads, writes, and register operations.
     * 
     * @note Initialized during construction, must remain valid for driver lifetime
     * @see AP_HAL::I2CDevice
     */
    AP_HAL::I2CDevice *dev;

    /**
     * @brief Measurement accumulator for noise reduction
     * 
     * @details Accumulates multiple distance measurements between update() calls to provide
     *          stable, averaged distance values. Timer callback adds measurements to accumulator,
     *          and update() calculates average and resets for next cycle.
     * 
     *          Members:
     *          - sum: Accumulated sum of valid distance measurements (centimeters)
     *          - count: Number of valid measurements accumulated
     * 
     * @note Averaging multiple samples reduces measurement noise and provides smoother data
     * @note Accumulator is reset after each update() call
     */
    struct {
        uint32_t sum;      ///< Sum of accumulated distance measurements in centimeters
        uint32_t count;    ///< Number of measurements accumulated
    } accum;
};

#endif  // AP_RANGEFINDER_TRI2C_ENABLED
