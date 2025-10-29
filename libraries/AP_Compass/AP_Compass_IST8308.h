/*
 * Copyright (C) 2018  Lucas De Marchi. All rights reserved.
 *
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 2 of the License, or
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
 * @file AP_Compass_IST8308.h
 * @brief Driver for the iSentek IST8308 3-axis magnetometer
 * 
 * @details This file implements the ArduPilot compass backend driver for the
 *          iSentek IST8308 digital magnetometer. The IST8308 is a 3-axis
 *          magnetic sensor with I2C interface, commonly used for heading
 *          determination in autopilot systems.
 * 
 *          Sensor Specifications:
 *          - Measurement range: ±1600 μT (±16 Gauss)
 *          - Resolution: 0.3 μT (3 milligauss) per LSB
 *          - Output data rate: Up to 200 Hz
 *          - Interface: I2C (default address 0x0C)
 *          - Operating voltage: 2.5V to 3.6V
 *          - Operating temperature: -40°C to +85°C
 * 
 *          Driver Architecture:
 *          - Implements AP_Compass_Backend interface for compass abstraction
 *          - Uses I2C HAL device for hardware communication
 *          - Periodic timer-based sampling for continuous measurement
 *          - Supports external compass mounting with rotation correction
 *          - Magnetic field reported in milligauss units
 * 
 *          Configuration:
 *          - Single measurement mode or continuous mode operation
 *          - WHOAMI verification (device ID: 0x08)
 *          - Self-test capability for sensor validation
 *          - Temperature compensation support
 * 
 * @note The IST8308 shares similar characteristics with IST8310 but has
 *       different register mappings and configuration procedures.
 * 
 * @see AP_Compass_Backend
 * @see AP_HAL::I2CDevice
 */
#pragma once

#include "AP_Compass_config.h"

#if AP_COMPASS_IST8308_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>

#include "AP_Compass_Backend.h"

#ifndef HAL_COMPASS_IST8308_I2C_ADDR
#define HAL_COMPASS_IST8308_I2C_ADDR 0x0C
#endif

/**
 * @class AP_Compass_IST8308
 * @brief Backend driver for iSentek IST8308 3-axis magnetometer
 * 
 * @details This class provides the ArduPilot compass backend implementation
 *          for the IST8308 magnetometer. The driver handles device detection,
 *          initialization, configuration, and periodic sampling of magnetic
 *          field measurements.
 * 
 *          Driver Lifecycle:
 *          1. probe() - Factory method attempts device detection via WHOAMI check
 *          2. Constructor - Initializes device handle and configuration
 *          3. init() - Configures sensor registers and measurement modes
 *          4. timer() - Periodic callback reads raw magnetic field data
 *          5. read() - Processes and publishes compass data to AP_Compass
 * 
 *          Operating Modes:
 *          - Continuous measurement mode for periodic sampling
 *          - Single measurement mode for on-demand readings
 *          - Self-test mode for sensor validation
 * 
 *          Coordinate System:
 *          - Raw sensor data in device frame (chip orientation)
 *          - Rotation matrix applied to align with vehicle frame
 *          - Magnetic field output in milligauss (mG) units
 * 
 *          Hardware Interface:
 *          - I2C communication at standard or fast mode (100kHz/400kHz)
 *          - Polling-based data acquisition via timer callback
 *          - Supports both internal and external compass mounting
 * 
 * @note This driver is instantiated via the probe() factory method during
 *       compass auto-detection at system startup.
 * 
 * @warning Ensure proper I2C bus configuration and pull-up resistors for
 *          reliable communication with the IST8308 sensor.
 */
class AP_Compass_IST8308 : public AP_Compass_Backend
{
public:
    /**
     * @brief Factory method to detect and initialize IST8308 magnetometer
     * 
     * @details Attempts to detect an IST8308 sensor on the provided I2C device
     *          by performing a WHOAMI register check (expected value: 0x08).
     *          If detection succeeds, creates and initializes a new driver
     *          instance with the specified configuration.
     * 
     *          Detection Process:
     *          1. Read WHOAMI register to verify device identity
     *          2. Validate device ID matches IST8308 (0x08)
     *          3. Construct driver instance if detection successful
     *          4. Initialize sensor configuration and register periodic callback
     *          5. Register compass instance with AP_Compass frontend
     * 
     * @param[in] dev             I2C device handle for communication (ownership transferred)
     * @param[in] force_external  If true, compass treated as external regardless of
     *                           board configuration (affects priority and sensor fusion)
     * @param[in] rotation        Rotation matrix to transform from sensor frame to
     *                           vehicle frame (corrects for mounting orientation)
     * 
     * @return Pointer to initialized AP_Compass_Backend driver instance on success,
     *         nullptr if device not detected or initialization failed
     * 
     * @note This method transfers ownership of the I2C device handle. On failure,
     *       the device handle is automatically released.
     * 
     * @note External compasses typically have higher priority in sensor fusion
     *       as they are mounted away from magnetic interference sources.
     * 
     * @see init()
     * @see AP_Compass::_add_backend()
     */
    static AP_Compass_Backend *probe(AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev,
                                     bool force_external,
                                     enum Rotation rotation);

    /**
     * @brief Process and publish latest compass measurements
     * 
     * @details Called by AP_Compass frontend to retrieve the latest magnetic
     *          field measurements. This method is called at the main compass
     *          update rate (typically 50-100 Hz) and publishes data that was
     *          acquired asynchronously by the timer() callback.
     * 
     *          The method performs no I2C communication itself; it processes
     *          data that was previously read by timer() and accumulated in
     *          internal buffers.
     * 
     * @note This is a non-blocking call that processes pre-fetched data.
     * @note Magnetic field values published in milligauss (mG) units.
     * 
     * @see timer()
     * @see AP_Compass_Backend::publish_filtered_field()
     */
    void read() override;

    /**
     * @brief Device identification string
     * 
     * @details Human-readable name used for logging, diagnostics, and
     *          ground control station display. This name appears in
     *          boot messages and compass selection logs.
     */
    static constexpr const char *name = "IST8308";

private:
    /**
     * @brief Private constructor for IST8308 driver instance
     * 
     * @details Constructs a new IST8308 compass driver with the provided
     *          configuration. This constructor is private and should only be
     *          called by the probe() factory method after successful device
     *          detection.
     * 
     * @param[in] dev             Device handle for I2C communication (ownership transferred)
     * @param[in] force_external  If true, compass treated as external mount
     * @param[in] rotation        Rotation to apply to sensor measurements
     * 
     * @note Constructor does not perform I2C communication. Actual sensor
     *       initialization happens in init() method.
     */
    AP_Compass_IST8308(AP_HAL::OwnPtr<AP_HAL::Device> dev,
                       bool force_external,
                       enum Rotation rotation);

    /**
     * @brief Periodic timer callback for sensor data acquisition
     * 
     * @details This method is called periodically by the HAL scheduler (typically
     *          at 100 Hz) to read raw magnetic field data from the IST8308 sensor
     *          via I2C. The method performs the following operations:
     * 
     *          1. Check data ready status register
     *          2. Read 6 bytes of raw magnetometer data (X, Y, Z axes)
     *          3. Convert raw ADC values to milligauss units
     *          4. Apply rotation correction for sensor mounting orientation
     *          5. Accumulate data for processing by read() method
     * 
     *          Data Format:
     *          - 16-bit signed integers per axis (little-endian)
     *          - Scale factor: 0.3 μT/LSB (3 milligauss/LSB)
     *          - Range: ±1600 μT (±16000 milligauss)
     * 
     * @note This method runs in timer interrupt context - must be fast and non-blocking
     * @note I2C transactions performed at interrupt priority level
     * 
     * @see read()
     */
    void timer();
    
    /**
     * @brief Initialize IST8308 sensor configuration
     * 
     * @details Performs initial sensor configuration after successful probe.
     *          This method configures the IST8308 registers for continuous
     *          measurement mode and registers the periodic timer callback.
     * 
     *          Initialization Sequence:
     *          1. Verify WHOAMI register (device ID = 0x08)
     *          2. Perform soft reset to restore default configuration
     *          3. Configure measurement mode (single or continuous)
     *          4. Set output data rate and averaging settings
     *          5. Enable temperature compensation if available
     *          6. Register timer() callback with HAL scheduler
     *          7. Register compass instance with AP_Compass frontend
     * 
     *          Configuration Parameters:
     *          - Measurement mode: Continuous for periodic sampling
     *          - Output data rate: Typically 100 Hz
     *          - Averaging: Minimal for low latency
     *          - Period: Timer callback registered at 10ms (100 Hz)
     * 
     * @return true if initialization successful and sensor responding correctly,
     *         false if I2C communication fails or sensor not responding
     * 
     * @note Initialization failure typically indicates hardware connection issues,
     *       incorrect I2C address, or sensor malfunction.
     * 
     * @warning If init() fails, the driver instance should not be used and will
     *          be automatically destroyed by the probe() method.
     * 
     * @see probe()
     * @see timer()
     */
    bool init();

    /**
     * @brief I2C device handle for hardware communication
     * 
     * @details Owns the I2C device interface for communicating with the IST8308
     *          sensor. Used for register reads/writes in init() and timer().
     *          Automatically released when driver instance is destroyed.
     */
    AP_HAL::OwnPtr<AP_HAL::Device> _dev;

    /**
     * @brief Rotation correction for sensor mounting orientation
     * 
     * @details Specifies the rotation matrix to transform measurements from
     *          sensor coordinate frame to vehicle coordinate frame. This
     *          accounts for non-standard sensor mounting orientations.
     *          Applied during data processing in timer().
     * 
     * @note Common rotations: ROTATION_NONE (standard orientation),
     *       ROTATION_YAW_90, ROTATION_YAW_180, ROTATION_YAW_270, etc.
     */
    enum Rotation _rotation;
    
    /**
     * @brief Compass instance identifier
     * 
     * @details Unique identifier assigned by AP_Compass frontend for this
     *          compass instance. Used when publishing measurements and for
     *          multi-compass configurations. Assigned during init().
     * 
     * @note Systems may have multiple compass sensors (internal + external)
     *       with different instance IDs for sensor fusion and failover.
     */
    uint8_t _instance;
    
    /**
     * @brief External compass mounting flag
     * 
     * @details If true, this compass is treated as externally mounted (typically
     *          on GPS mast) and given higher priority in sensor fusion due to
     *          lower magnetic interference. If false, treated as internal compass
     *          mounted on autopilot board near power electronics.
     * 
     * @note External compasses generally provide more accurate heading data
     *       as they are physically separated from vehicle magnetic interference
     *       sources (motors, ESCs, power wiring).
     */
    bool _force_external;
};

#endif  // AP_COMPASS_IST8308_ENABLED
