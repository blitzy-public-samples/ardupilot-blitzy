/**
 * @file AP_Compass_MAG3110.h
 * @brief Driver for Freescale/NXP MAG3110 3-axis magnetometer
 * 
 * @details This driver supports the MAG3110 digital magnetometer from Freescale
 *          (now NXP Semiconductors), a low-power 3-axis magnetometer with I2C
 *          interface commonly used in embedded navigation systems.
 * 
 *          The MAG3110 provides magnetic field measurements with automatic
 *          calibration and temperature compensation features.
 * 
 * @note The MAG3110 is found on various flight controller boards and external
 *       magnetometer modules, typically used as an external compass sensor.
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

#include "AP_Compass_config.h"

#if AP_COMPASS_MAG3110_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Device.h>
#include <AP_Math/AP_Math.h>

#include "AP_Compass.h"
#include "AP_Compass_Backend.h"


#ifndef HAL_MAG3110_I2C_ADDR 
 #define HAL_MAG3110_I2C_ADDR     0x0E
#endif

/**
 * @class AP_Compass_MAG3110
 * @brief Backend driver for Freescale/NXP MAG3110 3-axis magnetometer
 * 
 * @details This driver implements the AP_Compass_Backend interface for the
 *          MAG3110 digital magnetometer, providing magnetic field measurements
 *          for vehicle heading and navigation.
 * 
 *          Driver Lifecycle:
 *          1. probe() - Factory method detects and initializes the sensor
 *          2. init() - Configures sensor registers and continuous mode
 *          3. _hardware_init() - Performs low-level hardware initialization
 *          4. read() - Called periodically to update magnetic field data
 *          5. _read_sample() - Reads raw magnetic field values from sensor
 *          6. _update() - Processes and publishes compass data to frontend
 * 
 *          Hardware Specifications:
 *          - Measurement Range: ±1000 µT (±10 Gauss)
 *          - Resolution: 0.10 µT per LSB
 *          - Output Data Rate: Configurable, typically 80 Hz
 *          - Interface: I2C (default address 0x0E)
 *          - Operating Current: ~9 µA at 10 Hz ODR
 * 
 *          Field Units:
 *          - Internal storage: milligauss (1 milligauss = 0.1 µT)
 *          - Published to frontend: milligauss
 *          - Raw sensor data: 16-bit signed integers
 * 
 *          Initialization and Configuration:
 *          - Sensor detection via WHO_AM_I register verification
 *          - Configuration for continuous measurement mode
 *          - Automatic oversampling and noise reduction enabled
 *          - Default 80 Hz output data rate
 *          - Rotation matrix applied for board-specific mounting
 * 
 *          Common Applications:
 *          - External compass modules on flight controllers
 *          - Multi-compass configurations for redundancy
 *          - Heading reference for GPS-denied navigation
 * 
 * @note The MAG3110 does not have built-in hard-iron or soft-iron calibration.
 *       All magnetic calibration is performed by the AP_Compass frontend.
 * 
 * @warning Sensor performance can be affected by nearby magnetic fields from
 *          power cables, motors, or ferromagnetic materials. Mount externally
 *          when possible to minimize interference.
 * 
 * Source: libraries/AP_Compass/AP_Compass_MAG3110.h
 */
class AP_Compass_MAG3110 : public AP_Compass_Backend
{
public:
    /**
     * @brief Factory method to detect and initialize MAG3110 sensor
     * 
     * @details Attempts to detect a MAG3110 magnetometer on the provided I2C
     *          device and create a driver instance if successful. This method:
     *          1. Verifies sensor presence via WHO_AM_I register
     *          2. Creates driver instance if detection succeeds
     *          3. Initializes sensor configuration
     *          4. Registers compass instance with frontend
     * 
     *          This is called by the compass driver framework during sensor
     *          auto-detection at startup.
     * 
     * @param[in] dev      I2C device handle with pre-configured bus and address
     * @param[in] rotation Board-specific rotation for sensor mounting orientation
     * 
     * @return Pointer to AP_Compass_Backend instance if probe succeeds, nullptr if:
     *         - Sensor not detected (WHO_AM_I mismatch)
     *         - I2C communication failure
     *         - Initialization failure
     * 
     * @note This method transfers ownership of the device handle to the driver
     *       instance if successful, releases it if probe fails.
     * 
     * @see AP_Compass_Backend::probe() for probe protocol details
     */
    static AP_Compass_Backend *probe(AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                     enum Rotation rotation);

    /**
     * @brief Driver identification name for logging and diagnostics
     */
    static constexpr const char *name = "MAG3110";

    /**
     * @brief Read and update magnetic field measurements
     * 
     * @details Called periodically by the compass scheduler (typically at 100 Hz)
     *          to read new magnetic field data from the sensor and publish to the
     *          compass frontend. This method:
     *          1. Calls _read_sample() to read raw sensor data
     *          2. Calls _update() to process and publish measurements
     * 
     *          The sensor operates in continuous measurement mode, automatically
     *          acquiring new samples at the configured output data rate (80 Hz).
     *          This method polls for new data availability and reads when ready.
     * 
     * @note This is called at the AP_Compass update rate, typically 100 Hz.
     *       The sensor update rate (80 Hz) is slightly lower, so some calls
     *       may not have new data available.
     * 
     * @see AP_Compass_Backend::read() for scheduler integration
     */
    void read() override;

    /**
     * @brief Destructor
     * 
     * @details Cleans up driver resources. Device handle is automatically
     *          released by OwnPtr destructor.
     */
    ~AP_Compass_MAG3110() { }

private:
    /**
     * @brief Private constructor
     * 
     * @details Constructs driver instance with I2C device handle. Only called
     *          internally by probe() after successful sensor detection.
     * 
     * @param[in] dev I2C device handle (ownership transferred to driver)
     */
    AP_Compass_MAG3110(AP_HAL::OwnPtr<AP_HAL::Device> dev);

    /**
     * @brief Initialize sensor configuration and register with frontend
     * 
     * @details Performs sensor initialization sequence:
     *          1. Calls _hardware_init() to configure sensor registers
     *          2. Registers compass instance with AP_Compass frontend
     *          3. Applies rotation matrix for mounting orientation
     *          4. Sets compass ID and external/internal status
     * 
     * @param[in] rotation Board-specific sensor mounting orientation
     * 
     * @return true if initialization successful, false on failure
     */
    bool init(enum Rotation rotation);

    /**
     * @brief Read raw magnetic field sample from sensor
     * 
     * @details Reads 6 bytes of raw magnetic field data from the MAG3110
     *          registers (OUT_X_MSB through OUT_Z_LSB) and stores in
     *          _mag_x, _mag_y, _mag_z member variables.
     * 
     *          The raw 16-bit signed values represent magnetic field strength
     *          with 0.10 µT per LSB resolution.
     * 
     * @return true if read successful, false on I2C communication error
     * 
     * @note This method does not check for new data availability. The sensor
     *       continuously updates output registers at configured ODR (80 Hz).
     */
    bool _read_sample();

    /**
     * @brief Configure sensor hardware registers
     * 
     * @details Performs low-level hardware initialization:
     *          - Verifies WHO_AM_I register (0xC4 expected)
     *          - Configures CTRL_REG1 for continuous measurement mode
     *          - Enables automatic oversampling (typically 4x)
     *          - Sets output data rate to 80 Hz
     *          - Configures CTRL_REG2 for automatic magnetic sensor reset
     * 
     * @return true if hardware initialization successful, false if:
     *         - WHO_AM_I register mismatch
     *         - Register write failures
     */
    bool _hardware_init();

    /**
     * @brief Process and publish magnetic field measurements
     * 
     * @details Converts raw sensor data to calibrated magnetic field values:
     *          1. Converts raw counts to milligauss units
     *          2. Applies rotation matrix for sensor mounting orientation
     *          3. Publishes to compass frontend via accumulate() method
     * 
     *          Units: Sensor outputs 0.10 µT per LSB, converted to milligauss
     *                 (1 µT = 10 milligauss)
     * 
     * @note Called internally by read() after successful _read_sample()
     */
    void _update();

    /**
     * @brief I2C device handle for sensor communication
     * 
     * @details Provides I2C bus access for register reads/writes. Ownership
     *          transferred from probe() method. Automatically released on
     *          driver destruction.
     */
    AP_HAL::OwnPtr<AP_HAL::Device> _dev;

    /**
     * @brief Raw magnetic field X-axis measurement (milligauss)
     * 
     * @details Stores latest X-axis magnetic field reading. Positive X points
     *          forward in sensor body frame before rotation matrix is applied.
     */
    int32_t _mag_x;

    /**
     * @brief Raw magnetic field Y-axis measurement (milligauss)
     * 
     * @details Stores latest Y-axis magnetic field reading. Positive Y points
     *          right in sensor body frame before rotation matrix is applied.
     */
    int32_t _mag_y;

    /**
     * @brief Raw magnetic field Z-axis measurement (milligauss)
     * 
     * @details Stores latest Z-axis magnetic field reading. Positive Z points
     *          down in sensor body frame before rotation matrix is applied.
     */
    int32_t _mag_z;

    /**
     * @brief Compass instance ID registered with frontend
     * 
     * @details Index used to identify this compass in multi-compass systems.
     *          Assigned by AP_Compass frontend during registration in init().
     */
    uint8_t _compass_instance;

    /**
     * @brief Initialization status flag
     * 
     * @details Set to true after successful hardware initialization and
     *          frontend registration. Used to prevent multiple initialization
     *          attempts.
     */
    bool _initialised;
};

#endif  // AP_COMPASS_MAG3110_ENABLED
