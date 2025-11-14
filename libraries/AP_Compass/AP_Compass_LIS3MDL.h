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
 */

/**
 * @file AP_Compass_LIS3MDL.h
 * @brief Driver for STMicroelectronics LIS3MDL 3-axis magnetometer
 * 
 * @details This driver implements support for the LIS3MDL digital magnetic sensor,
 *          a low-power 3-axis magnetometer with I2C/SPI interfaces commonly used
 *          in modern autopilot boards for heading determination and compass functionality.
 * 
 *          Device Specifications:
 *          - Magnetic field range: ±4, ±8, ±12, or ±16 gauss (driver uses ±16 gauss full scale)
 *          - Resolution: 16-bit ADC per axis
 *          - Output data rate: Configurable from 0.625 Hz to 80 Hz
 *          - Field units: Milligauss (reported to AP_Compass framework)
 * 
 *          Driver Lifecycle:
 *          1. probe() - Factory method to detect and initialize sensor
 *          2. init() - Configure sensor registers and verify WHOAMI ID
 *          3. timer() - Periodic callback registered with HAL scheduler for data acquisition
 *          4. read() - Publish accumulated samples to AP_Compass frontend
 * 
 *          The LIS3MDL is commonly found on:
 *          - Pixhawk 4, Pixhawk 6C/X
 *          - Holybro Kakute F7/H7 boards
 *          - mRo Control Zero boards
 *          - Various third-party autopilot hardware
 * 
 * Source: libraries/AP_Compass/AP_Compass_LIS3MDL.h
 */

#pragma once

#include "AP_Compass_config.h"

#if AP_COMPASS_LIS3MDL_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Device.h>
#include <AP_Math/AP_Math.h>

#include "AP_Compass.h"
#include "AP_Compass_Backend.h"

#ifndef HAL_COMPASS_LIS3MDL_I2C_ADDR
# define HAL_COMPASS_LIS3MDL_I2C_ADDR 0x1c
#endif

#ifndef HAL_COMPASS_LIS3MDL_I2C_ADDR2
# define HAL_COMPASS_LIS3MDL_I2C_ADDR2 0x1e
#endif

/**
 * @class AP_Compass_LIS3MDL
 * @brief Backend driver for STMicroelectronics LIS3MDL 3-axis magnetometer
 * 
 * @details This class implements the AP_Compass_Backend interface for the LIS3MDL
 *          magnetometer sensor, providing magnetic field measurements for heading
 *          determination and compass functionality in ArduPilot autopilot systems.
 * 
 *          Driver Architecture:
 *          - Inherits from AP_Compass_Backend base class
 *          - Uses AP_HAL::Device abstraction for I2C/SPI communication
 *          - Implements asynchronous sensor reading via HAL scheduler callbacks
 *          - Supports sensor rotation matrices for flexible mounting orientations
 *          - Handles both internal and external compass configurations
 * 
 *          Initialization Sequence:
 *          1. probe() factory method attempts device detection on I2C/SPI bus
 *          2. WHOAMI register (0x0F) verified for device ID 0x3D
 *          3. Device configured for continuous measurement mode
 *          4. Control registers set for ±16 gauss range and appropriate data rate
 *          5. Periodic timer callback registered with HAL scheduler
 *          6. Compass instance registered with AP_Compass frontend
 * 
 *          Sensor Configuration:
 *          - Full scale range: ±16 gauss (65536 LSB range)
 *          - Sensitivity: 0.58 milligauss per LSB (±16 gauss mode)
 *          - Operating mode: Continuous measurement
 *          - Temperature compensation: Enabled in hardware
 * 
 *          Data Flow:
 *          - timer() callback reads raw X,Y,Z magnetic field data
 *          - Raw ADC values converted to milligauss units
 *          - Rotation matrix applied for board mounting orientation
 *          - Samples accumulated and averaged
 *          - read() publishes averaged samples to AP_Compass frontend
 * 
 *          Thread Safety:
 *          - timer() executes in HAL scheduler context (typically 100 Hz)
 *          - read() called from main vehicle loop
 *          - No explicit locking required due to single-writer pattern
 * 
 * @note The LIS3MDL can operate on either I2C or SPI buses depending on board
 *       hardware configuration. The probe() method handles both interfaces.
 * 
 * @warning Ensure proper calibration via standard compass calibration procedures.
 *          Uncalibrated compass data can lead to navigation errors.
 * 
 * Source: libraries/AP_Compass/AP_Compass_LIS3MDL.h
 */
class AP_Compass_LIS3MDL : public AP_Compass_Backend
{
public:
    /**
     * @brief Factory method to detect and initialize LIS3MDL magnetometer
     * 
     * @details Attempts to detect a LIS3MDL sensor on the provided device interface
     *          (I2C or SPI) and initializes it for operation if found. This is the
     *          primary entry point for sensor instantiation during autopilot startup.
     * 
     *          Probe Sequence:
     *          1. Reads WHOAMI register (0x0F) and verifies device ID is 0x3D
     *          2. If WHOAMI check fails, returns nullptr (device not present/compatible)
     *          3. Creates AP_Compass_LIS3MDL instance with provided parameters
     *          4. Calls init() to configure sensor registers
     *          5. If init() fails, deletes instance and returns nullptr
     *          6. On success, returns pointer to initialized backend
     * 
     * @param[in] dev HAL device interface (I2C or SPI) for sensor communication.
     *                Ownership transferred to the driver on successful probe.
     * @param[in] force_external If true, sensor is marked as external compass
     *                           (mounted outside main board enclosure), affecting
     *                           interference compensation and priority ordering.
     * @param[in] rotation Board rotation matrix from Rotation enum to correct
     *                     sensor measurements for physical mounting orientation.
     *                     Common values: ROTATION_NONE, ROTATION_YAW_90, etc.
     * 
     * @return Pointer to initialized AP_Compass_Backend on success, nullptr on failure
     * 
     * @note This method is called by the compass driver detection system during
     *       autopilot initialization. Failed probes are expected and handled gracefully.
     * 
     * @see AP_Compass::_detect_backends() for probe invocation
     * 
     * Source: libraries/AP_Compass/AP_Compass_LIS3MDL.cpp probe() implementation
     */
    static AP_Compass_Backend *probe(AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                     bool force_external,
                                     enum Rotation rotation);

    /**
     * @brief Publish accumulated magnetometer samples to frontend
     * 
     * @details This method is called from the main vehicle loop to retrieve and
     *          publish magnetic field measurements that have been accumulated by
     *          the timer() callback running in the HAL scheduler context.
     * 
     *          Operation:
     *          - Called at main loop rate (typically 50-400 Hz depending on vehicle)
     *          - Retrieves samples accumulated since last read() call
     *          - Applies final corrections and filtering
     *          - Publishes magnetic field vector to AP_Compass frontend via
     *            publish_filtered_field() or similar frontend API
     * 
     *          Field Data:
     *          - Units: Milligauss (mGa) for each axis
     *          - Axes: X (forward), Y (right), Z (down) in body frame after rotation
     *          - Typical earth field magnitude: 250-650 mGa depending on location
     * 
     * @note This method implements the pure virtual read() from AP_Compass_Backend.
     *       It runs in main thread context, while timer() runs in scheduler context.
     * 
     * @see timer() for sample acquisition in scheduler context
     * 
     * Source: libraries/AP_Compass/AP_Compass_LIS3MDL.cpp read() implementation
     */
    void read() override;

    static constexpr const char *name = "LIS3MDL";

private:
    /**
     * @brief Private constructor for LIS3MDL driver instance
     * 
     * @details Constructs a new LIS3MDL driver instance with the provided device
     *          interface and configuration parameters. This constructor is private
     *          and only called from the probe() factory method after successful
     *          device detection.
     * 
     * @param[in] dev HAL device interface (I2C or SPI) for sensor communication.
     *                Ownership transferred to this driver instance.
     * @param[in] force_external If true, compass marked as external for priority
     *                           and interference compensation purposes.
     * @param[in] rotation Rotation matrix to correct for sensor mounting orientation.
     * 
     * @note Does not perform sensor initialization - init() must be called separately.
     * 
     * Source: libraries/AP_Compass/AP_Compass_LIS3MDL.cpp constructor implementation
     */
    AP_Compass_LIS3MDL(AP_HAL::OwnPtr<AP_HAL::Device> dev,
                       bool force_external,
                       enum Rotation rotation);

    /**
     * @brief HAL device interface for I2C/SPI communication
     * 
     * @details OwnPtr to hardware abstraction layer device providing read/write
     *          access to LIS3MDL registers via I2C (addresses 0x1C or 0x1E) or
     *          SPI depending on board configuration.
     */
    AP_HAL::OwnPtr<AP_HAL::Device> dev;
    
    /**
     * @brief Initialize sensor hardware and verify communication
     * 
     * @details Configures the LIS3MDL sensor for continuous measurement mode
     *          and registers periodic callback with HAL scheduler for data acquisition.
     * 
     *          Initialization Sequence:
     *          1. Verify WHOAMI register (0x0F) reads as 0x3D (LIS3MDL device ID)
     *          2. Configure CTRL_REG1 (0x20): Enable X/Y axes, set data rate
     *          3. Configure CTRL_REG2 (0x21): Set ±16 gauss full-scale range
     *          4. Configure CTRL_REG3 (0x22): Continuous measurement mode
     *          5. Configure CTRL_REG4 (0x23): Set Z-axis operating mode
     *          6. Enable temperature sensor compensation if supported
     *          7. Register compass instance with AP_Compass frontend
     *          8. Register timer() callback with HAL scheduler at ~100 Hz
     * 
     * @return true if initialization successful, false on communication failure
     *              or WHOAMI mismatch
     * 
     * @note Called once during probe() sequence. Failure causes probe() to return nullptr.
     * 
     * @warning Must be called before timer() callback begins executing.
     * 
     * Source: libraries/AP_Compass/AP_Compass_LIS3MDL.cpp init() implementation
     */
    bool init();
    
    /**
     * @brief Periodic callback to acquire magnetic field samples from sensor
     * 
     * @details This method is invoked periodically by the HAL scheduler (typically
     *          at 100 Hz) to read raw magnetic field data from the LIS3MDL sensor
     *          and accumulate samples for later publishing via read().
     * 
     *          Data Acquisition Process:
     *          1. Check STATUS_REG (0x27) for data ready flag (ZYXDA bit)
     *          2. Read 6 bytes from OUT_X_L through OUT_Z_H registers (0x28-0x2D)
     *          3. Convert 16-bit signed raw values to milligauss:
     *             field_mGa = raw_value * 0.58 (for ±16 gauss range)
     *          4. Apply rotation matrix for sensor mounting orientation
     *          5. Accumulate samples for averaging
     *          6. Update sample health indicators
     * 
     *          Register Reading:
     *          - OUT_X_L (0x28), OUT_X_H (0x29): X-axis magnetic field (LSB, MSB)
     *          - OUT_Y_L (0x2A), OUT_Y_H (0x2B): Y-axis magnetic field (LSB, MSB)
     *          - OUT_Z_L (0x2C), OUT_Z_H (0x2D): Z-axis magnetic field (LSB, MSB)
     *          - Data format: 16-bit two's complement, little-endian
     * 
     * @note Executes in HAL scheduler context, separate from main thread.
     *       Must be fast and non-blocking (<1ms execution time typical).
     * 
     * @see read() for sample publication to main thread
     * 
     * Source: libraries/AP_Compass/AP_Compass_LIS3MDL.cpp timer() implementation
     */
    void timer();

    /**
     * @brief Compass instance ID assigned by AP_Compass frontend
     * 
     * @details Instance identifier used when publishing magnetic field samples
     *          to the AP_Compass frontend. Assigned during init() when registering
     *          the compass with register_compass() or similar frontend API.
     */
    uint8_t compass_instance;
    
    /**
     * @brief External compass flag indicating mounting location
     * 
     * @details If true, this compass is mounted externally (outside main autopilot
     *          enclosure), affecting priority ordering and electromagnetic interference
     *          compensation strategies. External compasses typically have lower
     *          interference from power systems and motors.
     */
    bool force_external;
    
    /**
     * @brief Rotation matrix for sensor mounting orientation correction
     * 
     * @details Rotation enum value defining the transformation from sensor physical
     *          orientation to vehicle body frame. Applied to raw sensor readings to
     *          correct for non-standard mounting (e.g., sensor rotated 90°, 180°, etc.).
     *          Common values include ROTATION_NONE, ROTATION_YAW_90, ROTATION_YAW_180,
     *          ROTATION_ROLL_180, etc.
     */
    enum Rotation rotation;
};

#endif  // AP_COMPASS_LIS3MDL_ENABLED
