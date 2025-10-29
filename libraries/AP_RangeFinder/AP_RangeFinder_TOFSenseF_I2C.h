/**
 * @file AP_RangeFinder_TOFSenseF_I2C.h
 * @brief Driver for TOFSense-F I2C time-of-flight distance sensor
 * 
 * @details This file implements the backend driver for TOFSense-F compact
 *          time-of-flight (ToF) distance sensors using I2C communication protocol.
 *          The TOFSense-F is a compact laser rangefinder suitable for obstacle
 *          avoidance, terrain following, and precision landing applications.
 *          
 *          Communication: I2C bus with 7-bit addressing
 *          Default I2C Address: 0x08
 *          Measurement Principle: Time-of-flight laser ranging
 *          Sensor Type: Laser distance sensor (MAVLink: MAV_DISTANCE_SENSOR_LASER)
 *          
 *          The driver follows the standard ArduPilot rangefinder backend pattern:
 *          - Static detect() method for device discovery and initialization
 *          - Asynchronous timer-based reading via HAL scheduler callbacks
 *          - update() method called by frontend to retrieve latest measurements
 * 
 * @note This driver requires I2C bus support from the HAL implementation
 * @note Sensor readings are updated asynchronously via timer callbacks
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_TOFSENSEF_I2C_ENABLED

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend.h"

#include <AP_HAL/I2CDevice.h>

/// Default I2C address for TOFSense-F sensor (7-bit addressing)
#define TOFSENSEP_I2C_DEFAULT_ADDR   0x08

/**
 * @class AP_RangeFinder_TOFSenseF_I2C
 * @brief Backend driver for TOFSense-F I2C time-of-flight distance sensors
 * 
 * @details This class implements support for TOFSense-F compact laser rangefinders
 *          using I2C communication protocol. The driver handles:
 *          - Device detection and initialization on the I2C bus
 *          - Asynchronous distance measurement via timer callbacks
 *          - Distance data retrieval and state management
 *          - Signal strength and status monitoring
 *          
 *          Driver Lifecycle:
 *          1. detect() - Static factory method probes I2C bus for sensor
 *          2. Constructor - Initializes driver with I2C device handle
 *          3. init() - Configures sensor and registers timer callback
 *          4. timer() - Periodic callback requests/reads distance measurements
 *          5. update() - Frontend calls to retrieve latest distance reading
 *          
 *          Threading Model:
 *          - timer() runs in HAL scheduler context (asynchronous)
 *          - update() called from main thread (synchronous with frontend)
 *          - new_distance flag provides thread-safe handoff between contexts
 *          
 *          Measurement Process:
 *          - start_reading() initiates a distance measurement request
 *          - get_reading() retrieves measurement result, signal strength, and status
 *          - Distance stored in distance_mm until consumed by update()
 * 
 * @note Inherits from AP_RangeFinder_Backend base class
 * @note Uses HAL I2CDevice for hardware abstraction
 * @note All I2C operations are non-blocking to avoid scheduler delays
 * 
 * @see AP_RangeFinder_Backend
 * @see AP_HAL::I2CDevice
 */
class AP_RangeFinder_TOFSenseF_I2C : public AP_RangeFinder_Backend
{
public:
    /**
     * @brief Static factory method to detect and initialize TOFSense-F sensor
     * 
     * @details Attempts to detect a TOFSense-F sensor on the provided I2C device.
     *          If successful, creates and returns a new driver instance. This method
     *          is called by the rangefinder frontend during device enumeration.
     *          
     *          Detection Process:
     *          1. Creates driver instance with provided I2C device handle
     *          2. Calls init() to verify sensor presence and configure
     *          3. Returns driver instance if successful, nullptr if failed
     *          
     *          The I2C device handle is transferred to the driver if detection succeeds,
     *          or destroyed if detection fails.
     * 
     * @param[in,out] _state    Rangefinder state structure to be managed by driver
     * @param[in]     _params   Rangefinder configuration parameters
     * @param[in]     dev       I2C device handle (ownership transferred on success)
     * 
     * @return Pointer to new driver instance if sensor detected, nullptr if not detected
     * 
     * @note This method transfers ownership of the I2C device handle to the driver
     * @note Called during rangefinder initialization/enumeration phase
     * @note Detection failure does not indicate an error - sensor may not be present
     * 
     * @see init()
     * @see AP_RangeFinder_Backend
     */
    static AP_RangeFinder_Backend *detect(RangeFinder::RangeFinder_State &_state,
                                          AP_RangeFinder_Params &_params,
                                          AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    /**
     * @brief Update rangefinder state with latest distance measurement
     * 
     * @details Called by the rangefinder frontend at regular intervals (typically 10-50Hz)
     *          to retrieve the latest distance measurement. This method checks if a new
     *          measurement is available from the timer callback and updates the state
     *          structure accordingly.
     *          
     *          Update Process:
     *          1. Checks new_distance flag set by timer() callback
     *          2. If new data available, copies distance_mm to state structure
     *          3. Clears new_distance flag after consuming measurement
     *          4. Updates state status and timestamp
     *          
     *          Threading: This method runs in the main thread context and reads
     *          data written by timer() which runs in scheduler callback context.
     *          The new_distance flag provides simple handshaking between contexts.
     * 
     * @return void
     * 
     * @note Overrides AP_RangeFinder_Backend::update()
     * @note Called from main thread, reads data from timer callback context
     * @note Distance measurements are in millimeters
     * @note State structure updated only when new measurement available
     * 
     * @see timer()
     * @see AP_RangeFinder_Backend::update()
     */
    void update(void) override;

protected:

    /**
     * @brief Return MAVLink distance sensor type for this rangefinder
     * 
     * @details Returns the MAVLink sensor type identifier for telemetry and logging.
     *          TOFSense-F uses laser time-of-flight technology, so reports as
     *          MAV_DISTANCE_SENSOR_LASER type.
     *          
     *          This type is used by ground control stations and companion computers
     *          to understand the sensor technology and apply appropriate filtering
     *          or interpretation of distance data.
     * 
     * @return MAV_DISTANCE_SENSOR_LASER indicating laser-based distance measurement
     * 
     * @note Overrides AP_RangeFinder_Backend::_get_mav_distance_sensor_type()
     * @note This is a const method - does not modify object state
     * @note Used for MAVLink DISTANCE_SENSOR message generation
     * 
     * @see AP_RangeFinder_Backend::_get_mav_distance_sensor_type()
     */
    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_LASER;
    }

private:
    /**
     * @brief Private constructor for TOFSense-F I2C driver instance
     * 
     * @details Constructs a driver instance with provided state, parameters, and I2C device.
     *          Constructor is private - instances are only created via the static detect()
     *          factory method after successful device detection.
     *          
     *          Initialization Steps:
     *          1. Stores reference to rangefinder state structure
     *          2. Stores reference to configuration parameters
     *          3. Takes ownership of I2C device handle
     *          4. Initializes member variables to default values
     *          
     *          The actual sensor initialization and configuration occurs in init(),
     *          not in the constructor.
     * 
     * @param[in,out] _state    Rangefinder state structure managed by this driver
     * @param[in]     _params   Configuration parameters for this rangefinder instance
     * @param[in]     dev       I2C device handle (ownership transferred to driver)
     * 
     * @note Private constructor - use detect() factory method instead
     * @note Does not perform I2C communication - see init() for sensor setup
     * @note Ownership of I2C device handle is transferred to driver
     * 
     * @see detect()
     * @see init()
     * @see AP_RangeFinder_Backend::AP_RangeFinder_Backend()
     */
    AP_RangeFinder_TOFSenseF_I2C(RangeFinder::RangeFinder_State &_state,
    								AP_RangeFinder_Params &_params,
                                 AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    /**
     * @brief Initialize TOFSense-F sensor and register timer callback
     * 
     * @details Performs sensor initialization and configures the driver for operation.
     *          Called by detect() during sensor enumeration to verify sensor presence
     *          and configure communication.
     *          
     *          Initialization Process:
     *          1. Configures I2C bus parameters (retry count, timing)
     *          2. Verifies sensor responds to I2C communication
     *          3. Performs any sensor-specific configuration
     *          4. Registers periodic timer callback with HAL scheduler
     *          5. Initializes measurement state variables
     *          
     *          If initialization fails at any step, the method returns false and
     *          the driver instance will be destroyed by detect().
     * 
     * @return true if initialization successful, false if sensor not responding
     * 
     * @note Called only once during driver creation in detect()
     * @note Failure indicates sensor not present or not responding on I2C bus
     * @note Registers timer() as periodic callback if successful
     * @note Typical timer callback rate: 10-50Hz for distance updates
     * 
     * @see detect()
     * @see timer()
     * @see AP_HAL::I2CDevice::register_periodic_callback()
     */
    bool init(void);

    /**
     * @brief Timer callback for asynchronous distance measurement
     * 
     * @details Periodic callback registered with HAL scheduler to request and retrieve
     *          distance measurements from the sensor. Runs in scheduler callback context
     *          independently of the main thread.
     *          
     *          Timer Operation Sequence:
     *          1. Calls start_reading() to initiate distance measurement request
     *          2. Sensor performs time-of-flight measurement (takes several ms)
     *          3. Calls get_reading() to retrieve distance, signal strength, status
     *          4. Stores result in distance_mm and sets new_distance flag
     *          5. update() method in main thread consumes the measurement
     *          
     *          Error Handling:
     *          - I2C communication failures are detected and ignored
     *          - Previous valid measurement is retained on failure
     *          - State structure status updated to reflect health
     *          
     *          Threading: Runs asynchronously in HAL scheduler context, writes data
     *          consumed by update() in main thread. The new_distance flag provides
     *          simple thread-safe handshaking.
     * 
     * @return void
     * 
     * @note Called periodically by HAL scheduler (typically 10-50Hz)
     * @note Runs in scheduler callback context, not main thread
     * @note All I2C operations must be non-blocking to avoid stalling scheduler
     * @note Sets new_distance flag when measurement completes successfully
     * 
     * @warning Do not perform blocking operations in this callback
     * @warning Keep execution time minimal to avoid impacting scheduler performance
     * 
     * @see init()
     * @see update()
     * @see start_reading()
     * @see get_reading()
     */
    void timer(void);

    /// Latest distance measurement in millimeters from timer callback
    /// Written by timer(), read by update()
    uint32_t distance_mm;

    /// Flag indicating new distance measurement available
    /// Set by timer() when measurement completes, cleared by update() after consumption
    /// Provides simple thread-safe handshaking between timer callback and main thread
    bool new_distance;

    /**
     * @brief Initiate a distance measurement request to the sensor
     * 
     * @details Sends I2C command to TOFSense-F sensor to start a time-of-flight
     *          distance measurement. The sensor will perform the measurement asynchronously,
     *          and the result can be retrieved with get_reading() after a brief delay.
     *          
     *          I2C Protocol:
     *          - Sends measurement trigger command to sensor
     *          - Non-blocking operation using HAL I2C interface
     *          - Sensor begins ToF measurement process
     *          - Measurement typically completes in several milliseconds
     *          
     *          Timing: After start_reading() returns, the sensor requires time to
     *          complete the measurement. Typical measurement time is 5-50ms depending
     *          on range and accuracy settings.
     * 
     * @return true if I2C command sent successfully, false on communication error
     * 
     * @note Called from timer() callback context
     * @note Non-blocking operation - does not wait for measurement to complete
     * @note get_reading() should be called after measurement completes
     * @note I2C communication errors return false but do not assert/crash
     * 
     * @see timer()
     * @see get_reading()
     * @see AP_HAL::I2CDevice::transfer()
     */
    bool start_reading(void);

    /**
     * @brief Retrieve completed distance measurement from sensor
     * 
     * @details Reads distance measurement result, signal strength, and status from
     *          TOFSense-F sensor via I2C. Should be called after start_reading() and
     *          after allowing time for the sensor to complete the measurement.
     *          
     *          I2C Protocol:
     *          - Reads measurement result registers via I2C
     *          - Retrieves distance value in millimeters
     *          - Retrieves signal strength (quality metric)
     *          - Retrieves measurement status (valid/error codes)
     *          
     *          Measurement Validation:
     *          - Status value indicates measurement validity
     *          - Signal strength indicates measurement quality/confidence
     *          - Distance value is raw sensor reading in millimeters
     *          - Invalid measurements indicated by status bits
     *          
     *          Error Handling:
     *          - I2C communication failures return false
     *          - Invalid measurement status values return false
     *          - Output parameters unchanged on failure
     * 
     * @param[out] reading_mm       Distance measurement in millimeters
     * @param[out] signal_strength  Signal strength/quality indicator (higher is better)
     * @param[out] status           Measurement status code from sensor
     * 
     * @return true if valid measurement retrieved, false on communication or measurement error
     * 
     * @note Called from timer() callback context after start_reading()
     * @note Non-blocking I2C operation
     * @note Output parameters only valid when return value is true
     * @note Distance in millimeters - no unit conversion needed
     * 
     * @see timer()
     * @see start_reading()
     * @see AP_HAL::I2CDevice::transfer()
     */
    bool get_reading(uint32_t &reading_mm, uint16_t &signal_strength, uint16_t &status);

    /// I2C device handle for communication with TOFSense-F sensor
    /// Owned by driver, provides HAL-abstracted I2C access
    /// Configured during init() with bus parameters and callback registration
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;
};

#endif  // AP_RANGEFINDER_TOFSENSEF_I2C_ENABLED
