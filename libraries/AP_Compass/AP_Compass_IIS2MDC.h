/*
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
 *
 */

/**
 * @file AP_Compass_IIS2MDC.h
 * @brief Driver for STMicroelectronics IIS2MDC ultralow-power 3-axis magnetometer
 * 
 * @details The IIS2MDC is a high-performance 3-axis magnetometer designed for
 *          ultralow-power applications. It features a wide magnetic field range
 *          of ±50 gauss with 16-bit resolution and supports both continuous and
 *          single-shot measurement modes for power optimization.
 * 
 *          Key Features:
 *          - Ultralow power consumption ideal for battery-powered systems
 *          - ±50 gauss magnetic field measurement range
 *          - 16-bit resolution for precise compass readings
 *          - Continuous and single-shot operating modes
 *          - Self-test capability for field validation
 *          - I2C digital interface
 * 
 *          Applications:
 *          - Compass heading for navigation systems
 *          - Attitude and heading reference systems (AHRS)
 *          - Battery-powered UAVs and portable devices
 *          - External compass modules
 * 
 *          Output units: Magnetic field reported in milligauss (mG)
 * 
 * Source: libraries/AP_Compass/AP_Compass_IIS2MDC.h
 */

#pragma once

#include "AP_Compass_config.h"

#if AP_COMPASS_IIS2MDC_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>

#include "AP_Compass.h"
#include "AP_Compass_Backend.h"

/**
 * @brief Default I2C address for IIS2MDC magnetometer
 * @details Standard 7-bit I2C address 0x1E. Can be overridden in board-specific
 *          hwdef if sensor uses alternate address configuration.
 */
#ifndef HAL_COMPASS_IIS2MDC_I2C_ADDR
#define HAL_COMPASS_IIS2MDC_I2C_ADDR 0x1E
#endif

/**
 * @brief Default rotation for externally mounted IIS2MDC compass
 * @details Defines the rotation matrix applied to external compass measurements.
 *          ROTATION_NONE assumes sensor X/Y/Z axes align with vehicle forward/right/down.
 *          Override in board hwdef to match actual external compass mounting orientation.
 */
#ifndef HAL_COMPASS_IIS2MDC_ORIENTATION_EXTERNAL
#define HAL_COMPASS_IIS2MDC_ORIENTATION_EXTERNAL ROTATION_NONE
#endif

/**
 * @brief Default rotation for internally mounted IIS2MDC compass
 * @details Defines the rotation matrix applied to internal compass measurements.
 *          ROTATION_NONE assumes sensor X/Y/Z axes align with vehicle forward/right/down.
 *          Override in board hwdef to match actual internal compass mounting orientation.
 * @note Internal compasses may experience more magnetic interference from vehicle electronics.
 */
#ifndef HAL_COMPASS_IIS2MDC_ORIENTATION_INTERNAL
#define HAL_COMPASS_IIS2MDC_ORIENTATION_INTERNAL ROTATION_NONE
#endif

/**
 * @class AP_Compass_IIS2MDC
 * @brief Backend driver for STMicroelectronics IIS2MDC ultralow-power magnetometer
 * 
 * @details This driver implements the ArduPilot compass backend interface for the
 *          IIS2MDC 3-axis magnetometer. The driver handles sensor initialization,
 *          configuration, and periodic magnetic field measurements.
 * 
 *          Driver Architecture:
 *          - Factory pattern via probe() for sensor detection and instantiation
 *          - Interrupt-driven or polled reading via timer() callback
 *          - Automatic field unit conversion to milligauss
 *          - Configurable sensor orientation for external/internal mounting
 *          - Self-test capability during initialization
 * 
 *          Low-Power Features:
 *          - Supports continuous measurement mode for real-time heading
 *          - Supports single-shot mode for power-critical applications
 *          - Ultralow quiescent current consumption
 *          - Efficient I2C communication protocol
 * 
 *          Field Measurement:
 *          - Measurement range: ±50 gauss full scale
 *          - Resolution: 16-bit digital output
 *          - Output units: milligauss (mG)
 *          - Typical use: Earth's magnetic field (~250-650 mG depending on location)
 * 
 *          Integration:
 *          - Inherits from AP_Compass_Backend for standardized interface
 *          - Registers with AP_Compass singleton for multi-compass support
 *          - Supports both internal and external compass mounting
 *          - Configurable rotation matrix for sensor alignment
 * 
 * @note This driver is optimized for battery-powered systems requiring
 *       efficient power management and reliable compass operation.
 * 
 * @warning Sensor must be mounted away from magnetic interference sources
 *          (motors, power wiring, batteries) for accurate heading measurements.
 * 
 * @see AP_Compass_Backend
 * @see AP_Compass
 */
class AP_Compass_IIS2MDC : public AP_Compass_Backend
{
public:
    /**
     * @brief Factory method to detect and initialize an IIS2MDC magnetometer
     * 
     * @details Implements the probe pattern for IIS2MDC sensor detection. This method
     *          attempts to communicate with the sensor via I2C, validates the WHO_AM_I
     *          register, performs initialization and self-test, and creates a driver
     *          instance if successful.
     * 
     *          Probe Sequence:
     *          1. Attempt I2C communication with device
     *          2. Read and verify WHO_AM_I register (device identification)
     *          3. Perform sensor initialization and configuration
     *          4. Execute self-test to validate sensor functionality
     *          5. Register sensor with AP_Compass manager
     *          6. Configure power mode and measurement settings
     * 
     * @param[in] dev I2C device handle with bus and address pre-configured
     * @param[in] force_external true to force external compass classification,
     *                           false to auto-detect based on board configuration
     * @param[in] rotation Rotation matrix to align sensor axes with vehicle frame
     *                     (use ROTATION_NONE if sensor is aligned with vehicle axes)
     * 
     * @return Pointer to initialized AP_Compass_IIS2MDC backend on success,
     *         nullptr if sensor not detected or initialization failed
     * 
     * @note This is called during compass auto-detection at boot time
     * @note Only one instance should be created per physical sensor
     * 
     * @see check_whoami()
     * @see init()
     */
    static AP_Compass_Backend *probe(AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev,
                                     bool force_external,
                                     enum Rotation rotation);

    /**
     * @brief Read magnetic field measurements from the sensor
     * 
     * @details Implements the AP_Compass_Backend interface for periodic compass readings.
     *          This method retrieves the latest magnetic field data from the sensor,
     *          applies rotation corrections, converts units to milligauss, and publishes
     *          to the AP_Compass frontend.
     * 
     *          Operating Modes:
     *          - Continuous mode: Sensor automatically performs measurements at configured
     *            output data rate (ODR). Read retrieves latest buffered measurement.
     *          - Single-shot mode: Sensor performs one measurement per read() call, then
     *            returns to idle state for power savings (useful for low-power applications).
     * 
     *          Data Processing:
     *          1. Read 16-bit raw magnetic field data from sensor registers (X, Y, Z axes)
     *          2. Apply sensor-specific scaling to convert to milligauss units
     *          3. Apply rotation matrix to align with vehicle coordinate frame
     *          4. Publish corrected field vector to AP_Compass singleton
     * 
     *          Field Units: Output is in milligauss (mG)
     *          - Earth's magnetic field typically 250-650 mG depending on location
     *          - Sensor range: ±50,000 mG (±50 gauss) full scale
     * 
     * @note Called periodically by scheduler at configured compass read rate
     * @note Actual read frequency depends on sensor ODR configuration and operating mode
     * 
     * @warning Magnetic interference from motors or power systems will corrupt readings.
     *          Compass calibration required for accurate heading determination.
     * 
     * @see timer()
     * @see AP_Compass::publish()
     */
    void read() override;

    /**
     * @brief Driver identification name for logging and configuration
     */
    static constexpr const char *name = "IIS2MDC";

private:
    /**
     * @brief Private constructor for IIS2MDC driver instance
     * 
     * @details Constructor is private to enforce factory pattern via probe() method.
     *          Initializes driver state but does not configure hardware (see init()).
     * 
     * @param[in] dev I2C device handle (ownership transferred to driver)
     * @param[in] force_external true to classify as external compass
     * @param[in] rotation Sensor orientation relative to vehicle frame
     * 
     * @note Use probe() factory method instead of calling constructor directly
     * @see probe()
     */
    AP_Compass_IIS2MDC(AP_HAL::OwnPtr<AP_HAL::Device> dev,
                        bool force_external,
                        enum Rotation rotation);

    /**
     * @brief Verify sensor identity by reading WHO_AM_I register
     * 
     * @details Reads the IIS2MDC WHO_AM_I register and validates the device ID
     *          to confirm correct sensor type and successful I2C communication.
     * 
     * @return true if WHO_AM_I register matches expected IIS2MDC device ID,
     *         false if communication failed or wrong device detected
     * 
     * @note This is the first validation step during probe sequence
     * @see probe()
     */
    bool check_whoami();

    /**
     * @brief Periodic timer callback for sensor data acquisition
     * 
     * @details Registered with HAL scheduler to run at configured compass update rate.
     *          Reads magnetic field data from sensor registers, applies scaling and
     *          rotation, and updates compass state.
     * 
     * @note Runs in timer context - must be efficient and avoid blocking operations
     * @note Actual implementation may trigger sensor reads or retrieve buffered data
     *       depending on operating mode (continuous vs single-shot)
     * 
     * @see read()
     */
    void timer();

    /**
     * @brief Initialize and configure the IIS2MDC sensor hardware
     * 
     * @details Performs complete sensor initialization sequence including register
     *          configuration, self-test validation, and power mode setup.
     * 
     *          Initialization Sequence:
     *          1. Software reset to ensure known state
     *          2. Configure operating mode (continuous or single-shot)
     *          3. Set output data rate (ODR) for measurement frequency
     *          4. Configure low-pass filtering and averaging
     *          5. Perform self-test to validate sensor functionality
     *          6. Register periodic timer callback for data acquisition
     *          7. Enable sensor measurements
     * 
     *          Power Mode Configuration:
     *          - Configures sensor for optimal power efficiency while maintaining
     *            required measurement rate for compass operation
     *          - Selects continuous mode for real-time heading updates or
     *            single-shot mode for battery-critical applications
     * 
     *          Self-Test Procedure:
     *          - Applies internal test excitation field
     *          - Validates sensor response within acceptable range
     *          - Confirms proper operation of magnetic sensing elements
     * 
     * @return true if initialization and self-test successful,
     *         false if sensor configuration or self-test failed
     * 
     * @note Called during probe() sequence before driver instance is returned
     * @note Failure indicates sensor malfunction or communication issues
     * 
     * @see probe()
     * @see check_whoami()
     */
    bool init();

    /**
     * @brief I2C device handle for sensor communication
     * @details Ownership transferred from probe() method. Used for all register
     *          read/write operations with the IIS2MDC sensor.
     */
    AP_HAL::OwnPtr<AP_HAL::Device> _dev;

    /**
     * @brief Rotation matrix to align sensor axes with vehicle frame
     * @details Applied to raw magnetic field measurements to correct for sensor
     *          mounting orientation. Use ROTATION_NONE if sensor axes align with vehicle.
     */
    enum Rotation _rotation;

    /**
     * @brief Compass instance ID assigned by AP_Compass manager
     * @details Used to identify this compass in multi-compass configurations.
     *          Assigned during sensor registration.
     */
    uint8_t _instance;

    /**
     * @brief Force classification as external compass
     * @details If true, compass is treated as external regardless of board configuration.
     *          External compasses typically require different calibration and are subject
     *          to less magnetic interference from vehicle electronics.
     */
    bool _force_external;
};

#endif  // AP_COMPASS_IIS2MDC_ENABLED
