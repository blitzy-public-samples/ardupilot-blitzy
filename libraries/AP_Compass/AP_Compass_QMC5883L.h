/*
 * Copyright (C) 2016  Emlid Ltd. All rights reserved.
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

/**
 * @file AP_Compass_QMC5883L.h
 * @brief Driver for QST QMC5883L 3-axis magnetometer sensor
 * 
 * @details This file implements the driver for the QMC5883L magnetometer,
 *          a pin-compatible alternative to the Honeywell HMC5883L. The QMC5883L
 *          is manufactured by QST Corporation and provides similar functionality
 *          with different register mappings and configuration.
 *          
 *          Key specifications:
 *          - Measurement range: ±8 gauss
 *          - Resolution: 16-bit ADC
 *          - Output data rate: up to 200 Hz
 *          - I2C interface (default address 0x0D)
 *          - Power consumption: 75 μA typical
 *          
 *          The driver handles auto-detection and initialization, configures
 *          continuous measurement mode, and provides calibrated magnetic field
 *          readings in milligauss.
 * 
 * @note While pin-compatible with HMC5883L, the QMC5883L has completely
 *       different register addresses and bit mappings. Auto-detection based
 *       on chip ID is used to distinguish between the two sensors.
 * 
 * Source: libraries/AP_Compass/AP_Compass_QMC5883L.h
 */
#pragma once

#include "AP_Compass_config.h"

#if AP_COMPASS_QMC5883L_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_Math/AP_Math.h>

#include "AP_Compass.h"
#include "AP_Compass_Backend.h"

#ifndef HAL_COMPASS_QMC5883L_I2C_ADDR
#define HAL_COMPASS_QMC5883L_I2C_ADDR 0x0D
#endif

/*
  setup default orientations
 */
#ifndef HAL_COMPASS_QMC5883L_ORIENTATION_EXTERNAL
#define HAL_COMPASS_QMC5883L_ORIENTATION_EXTERNAL ROTATION_ROLL_180
#endif

#ifndef HAL_COMPASS_QMC5883L_ORIENTATION_INTERNAL
#define HAL_COMPASS_QMC5883L_ORIENTATION_INTERNAL ROTATION_ROLL_180_YAW_270
#endif

/**
 * @class AP_Compass_QMC5883L
 * @brief Driver for QST QMC5883L magnetometer - pin-compatible HMC5883L alternative
 * 
 * @details This class implements the backend driver for the QMC5883L 3-axis
 *          magnetometer chip. The QMC5883L is a pin-compatible replacement for
 *          the Honeywell HMC5883L with similar electrical characteristics but
 *          completely different internal register layout.
 *          
 *          Key differences from HMC5883L:
 *          - Different register addresses (incompatible register map)
 *          - Different chip ID for auto-detection (0xFF for QMC5883L vs 'H43' for HMC5883L)
 *          - Different sensitivity values and conversion factors
 *          - Different control register bit mappings
 *          - Fixed ±8 gauss measurement range (vs configurable on HMC5883L)
 *          
 *          The driver operates in continuous measurement mode, reading all three
 *          magnetic field axes (X, Y, Z) and applying rotation matrices to convert
 *          from sensor frame to vehicle body frame. Measurements are reported in
 *          milligauss units.
 *          
 *          Initialization sequence:
 *          1. Verify chip ID via WHO_AM_I register
 *          2. Configure output data rate and oversampling
 *          3. Set continuous measurement mode
 *          4. Register periodic timer callback for data reading
 *          
 *          The driver supports both internal (board-mounted) and external
 *          (GPS-mounted or standalone) compass configurations with appropriate
 *          default orientations.
 * 
 * @note Auto-detection: This driver is probed alongside HMC5883L driver during
 *       I2C device enumeration. Chip ID verification determines which driver
 *       instance is retained.
 * 
 * @warning Magnetic interference: Magnetometers are sensitive to nearby magnetic
 *          fields from motors, ESCs, and power wiring. Proper calibration and
 *          mounting location are critical for heading accuracy.
 * 
 * Source: libraries/AP_Compass/AP_Compass_QMC5883L.h
 */
class AP_Compass_QMC5883L : public AP_Compass_Backend
{
public:
    /**
     * @brief Factory method to detect and initialize QMC5883L magnetometer
     * 
     * @details This static factory method attempts to probe and initialize a
     *          QMC5883L magnetometer on the provided I2C device. The probe
     *          sequence includes:
     *          1. Verifying device communication
     *          2. Reading and validating chip ID (WHO_AM_I register)
     *          3. Configuring sensor for continuous measurement mode
     *          4. Registering compass instance with AP_Compass frontend
     *          
     *          If successful, returns a pointer to the initialized backend driver.
     *          If probe fails (wrong chip ID, communication error, or initialization
     *          failure), returns nullptr and the I2C device is released.
     *          
     *          The rotation parameter specifies the orientation transform from
     *          sensor frame to vehicle body frame. If ROTATION_NONE is provided,
     *          default orientations are used based on force_external flag:
     *          - External: ROTATION_ROLL_180 (typical for GPS-compass modules)
     *          - Internal: ROTATION_ROLL_180_YAW_270 (board-specific default)
     * 
     * @param[in] dev            I2C device handle (ownership transferred to driver if successful)
     * @param[in] force_external true to treat as external compass (affects default orientation
     *                           and compass priority), false for internal board-mounted compass
     * @param[in] rotation       Rotation matrix to apply for sensor-to-body frame transform,
     *                           or ROTATION_NONE to use default orientation
     * 
     * @return Pointer to initialized AP_Compass_Backend instance on success, nullptr on failure
     * 
     * @note This method is called during I2C bus enumeration at boot time and during
     *       runtime compass detection. Multiple probe attempts may occur on the same
     *       I2C address with different driver types (HMC5883L vs QMC5883L).
     * 
     * @see AP_Compass::_detect_backends() for probe invocation
     * @see init() for detailed initialization sequence
     * 
     * Source: libraries/AP_Compass/AP_Compass_QMC5883L.cpp probe() implementation
     */
    static AP_Compass_Backend *probe(AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev,
									 bool force_external,
                                     enum Rotation rotation);

    /**
     * @brief Read magnetic field data from sensor in continuous measurement mode
     * 
     * @details This method is called by the AP_Compass frontend at the configured
     *          compass update rate (typically 50-100 Hz). For QMC5883L, actual
     *          data acquisition is handled by the timer() callback which runs at
     *          the sensor's configured output data rate.
     *          
     *          The read() override provides interface compatibility with the
     *          AP_Compass_Backend base class but does not perform actual hardware
     *          reads. Instead, it allows the frontend to maintain consistent
     *          timing and provides a hook for future synchronous read implementations.
     *          
     *          Actual measurement workflow:
     *          1. timer() callback reads raw X, Y, Z magnetic field values from
     *             sensor data registers (6 bytes total)
     *          2. Raw 16-bit ADC values are converted to milligauss using
     *             sensor-specific sensitivity factor
     *          3. Rotation matrix applied to transform sensor frame to body frame
     *          4. Accumulated data passed to frontend via publish_field()
     * 
     * @return void
     * 
     * @note This method executes in the main thread context, while timer()
     *       executes in scheduler callback context. Thread safety is maintained
     *       through the HAL's semaphore mechanisms.
     * 
     * @see timer() for actual hardware data acquisition
     * @see AP_Compass_Backend::publish_field() for data reporting
     * 
     * Source: libraries/AP_Compass/AP_Compass_QMC5883L.cpp read() implementation
     */
    void read() override;

    /**
     * @brief Human-readable sensor name for identification and logging
     * 
     * @details This static constexpr string identifies the sensor type in log
     *          messages, parameter descriptions, and ground control station displays.
     *          Used by the compass library to report which magnetometer types are
     *          detected and active.
     * 
     * Source: libraries/AP_Compass/AP_Compass_QMC5883L.h
     */
    static constexpr const char *name = "QMC5883L";

private:
    /**
     * @brief Private constructor called by probe() factory method
     * 
     * @param[in] dev            I2C device handle (ownership transferred)
     * @param[in] force_external true for external compass configuration
     * @param[in] rotation       Sensor-to-body frame rotation transform
     * 
     * Source: libraries/AP_Compass/AP_Compass_QMC5883L.cpp constructor implementation
     */
    AP_Compass_QMC5883L(AP_HAL::OwnPtr<AP_HAL::Device> dev,
                        bool force_external,
                        enum Rotation rotation);

    /**
     * @brief Debug utility to dump all sensor registers to console
     * @note Used for development and troubleshooting register configuration issues
     * Source: libraries/AP_Compass/AP_Compass_QMC5883L.cpp
     */
    void _dump_registers();
    
    /**
     * @brief Verify chip identity by reading WHO_AM_I register
     * @return true if chip ID matches QMC5883L (0xFF), false otherwise
     * @note This method distinguishes QMC5883L from HMC5883L during auto-detection
     * Source: libraries/AP_Compass/AP_Compass_QMC5883L.cpp
     */
    bool _check_whoami();
    
    /**
     * @brief Periodic callback to read magnetic field data from sensor
     * @details Executes at sensor output data rate, reads 6-byte measurement,
     *          converts to milligauss, applies rotation, and publishes to frontend
     * @note Runs in scheduler callback context with appropriate timing guarantees
     * Source: libraries/AP_Compass/AP_Compass_QMC5883L.cpp
     */
    void timer();
    
    /**
     * @brief Initialize sensor hardware and register with compass frontend
     * @return true if initialization successful, false on failure
     * @details Configures control registers, sets continuous mode, validates
     *          configuration, and registers periodic timer callback
     * Source: libraries/AP_Compass/AP_Compass_QMC5883L.cpp
     */
    bool init();

    /// I2C device handle for sensor communication
    AP_HAL::OwnPtr<AP_HAL::Device> _dev;

    /// Rotation transform from sensor frame to vehicle body frame
    enum Rotation _rotation;
    
    /// Compass instance ID assigned by AP_Compass frontend
    uint8_t _instance;
    
    /// Configuration flag: true if external compass, false if internal
    bool _force_external:1;
};

#endif  // AP_COMPASS_QMC5883L_ENABLED
