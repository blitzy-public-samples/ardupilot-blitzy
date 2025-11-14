/*
 * Copyright (C) 2016  Intel Corporation. All rights reserved.
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
 * @file AP_Compass_BMM150.h
 * @brief Driver for Bosch BMM150 3-axis digital geomagnetic sensor
 * 
 * @details This driver implements support for the Bosch Sensortec BMM150
 *          low-power 3-axis magnetometer. The BMM150 is commonly used in
 *          mobile devices, wearables, and IoT applications for compass
 *          heading and magnetic field measurements.
 *          
 *          Key Features:
 *          - I2C interface with configurable addresses (0x10-0x13)
 *          - 16-bit resolution for X, Y, Z axes
 *          - Hall sensor resistance (RHALL) for temperature compensation
 *          - Factory-calibrated trim parameters stored in device NVM
 *          - Field measurement range: ±1300 μT (±13 Gauss)
 *          - Output in milligauss units after compensation
 *          
 *          Driver Architecture:
 *          1. probe() - Detects device and validates chip ID
 *          2. init() - Loads trim parameters from NVM and configures sensor
 *          3. read() - Periodic callback reads raw data and applies compensation
 *          
 *          The compensation algorithm uses factory trim values to correct for:
 *          - Manufacturing process variations in sensitivity
 *          - Temperature-dependent offset drift
 *          - Cross-axis sensitivity effects
 *          
 * @note Typical applications: smartphones, tablets, fitness trackers, drones
 * 
 * Source: libraries/AP_Compass/AP_Compass_BMM150.h
 */

#pragma once

#include "AP_Compass_config.h"

#if AP_COMPASS_BMM150_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_Math/AP_Math.h>

#include "AP_Compass.h"
#include "AP_Compass_Backend.h"

#define BMM150_I2C_ADDR_MIN 0x10
#define BMM150_I2C_ADDR_MAX 0x13

/**
 * @class AP_Compass_BMM150
 * @brief Backend driver for Bosch BMM150 3-axis magnetometer
 * 
 * @details This class implements the ArduPilot compass backend interface for the
 *          BMM150 sensor. The driver follows the standard probe/init/read pattern
 *          used throughout the AP_InertialSensor and AP_Compass subsystems.
 *          
 *          Lifecycle:
 *          1. probe() - Static factory method that detects the BMM150 on I2C bus,
 *             validates chip ID, and returns an instance if successful
 *          2. init() - Reads 11 factory trim parameters from device NVM (non-volatile
 *             memory) that are unique to each sensor die
 *          3. _update() - Registered as periodic callback, reads raw magnetic field
 *             and Hall resistance (RHALL) values
 *          4. Compensation - Applies mathematical correction using trim parameters
 *             to produce accurate field measurements in milligauss
 *          
 *          Trim Parameter Loading:
 *          The BMM150 stores factory-calibrated trim values in registers 0x5D-0x71
 *          that compensate for:
 *          - Sensitivity variations from silicon manufacturing process
 *          - Temperature-dependent magnetic offset drift (via RHALL measurement)
 *          - Cross-coupling between X/Y/Z axes
 *          
 *          Field Measurement Units:
 *          - Raw ADC values: 13-bit signed integers from sensor registers
 *          - Intermediate: Floating-point μT (microtesla) after compensation
 *          - Final output: milligauss (1 gauss = 100 μT, so 1 μT = 0.01 mG)
 *          
 *          Temperature Compensation:
 *          The sensor provides RHALL (Hall resistance) measurement alongside
 *          magnetic readings. RHALL varies with temperature and is used in
 *          compensation formulas to maintain accuracy across -40°C to +85°C.
 *          
 * @note The BMM150 is commonly found in compact autopilot boards and mobile
 *       platforms where low power consumption (170 μA typical) is important.
 * 
 * @see AP_Compass_Backend
 * @see libraries/AP_Compass/AP_Compass_BMM150.cpp for compensation algorithm implementation
 */
class AP_Compass_BMM150 : public AP_Compass_Backend
{
public:
    /**
     * @brief Probe for BMM150 magnetometer on I2C bus and create driver instance
     * 
     * @details This static factory method attempts to detect and initialize a BMM150
     *          sensor on the provided I2C device. The probe sequence:
     *          1. Reads chip ID register (0x40) and verifies value is 0x32
     *          2. Performs soft reset to ensure clean state
     *          3. Loads factory trim parameters from NVM registers
     *          4. Configures sensor for normal mode operation
     *          5. Registers periodic read callback with HAL scheduler
     *          
     *          If any step fails, the method returns nullptr and the device is
     *          released back to the HAL for potential use by other drivers.
     * 
     * @param[in] dev I2C device handle with configured bus and address (0x10-0x13)
     * @param[in] force_external If true, treat compass as externally mounted regardless
     *                            of board configuration (affects orientation and priority)
     * @param[in] rotation Rotation/orientation of sensor relative to board frame
     *                     (one of ROTATION_NONE, ROTATION_YAW_90, etc.)
     * 
     * @return Pointer to AP_Compass_BMM150 instance if probe successful, nullptr if failed
     * 
     * @note This method takes ownership of the I2C device handle via OwnPtr transfer
     * @note Typical I2C addresses: 0x10 (default), 0x11, 0x12, 0x13 based on SDO pin
     * 
     * @see init() for configuration details
     * @see _load_trim_values() for NVM parameter extraction
     */
    static AP_Compass_Backend *probe(AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev, bool force_external, enum Rotation rotation);

    /**
     * @brief Read magnetic field data from sensor (periodic callback)
     * 
     * @details This method is called periodically by the AP_Compass frontend to
     *          retrieve the latest magnetometer measurements. It triggers the
     *          internal _update() method which:
     *          1. Reads 8 bytes of data (X, Y, Z, RHALL registers)
     *          2. Extracts 13-bit signed values from packed register format
     *          3. Applies compensation using factory trim parameters
     *          4. Publishes corrected field vector in milligauss to compass frontend
     *          
     *          The compensation algorithm corrects raw ADC readings for temperature
     *          drift and manufacturing variations to produce accurate measurements.
     *          
     *          Field values are rotated according to the configured sensor orientation
     *          before being published to the navigation system.
     * 
     * @note Called at compass library update rate (typically 50-100 Hz)
     * @note Overrides AP_Compass_Backend::read() pure virtual method
     * 
     * @see _update() for data acquisition implementation
     * @see _compensate_xy() and _compensate_z() for correction algorithms
     */
    void read() override;

    static constexpr const char *name = "BMM150"; ///< Driver name for identification and logging

private:
    /**
     * @brief Private constructor called by probe() after successful device detection
     * 
     * @param[in] dev Device handle (I2C or SPI) with established communication
     * @param[in] force_external External mounting flag for orientation handling
     * @param[in] rotation Sensor mounting orientation relative to board frame
     */
    AP_Compass_BMM150(AP_HAL::OwnPtr<AP_HAL::Device> dev, bool force_external, enum Rotation rotation);

    /**
     * @brief Initialize BMM150 sensor and configure for operation
     * 
     * @details Initialization sequence:
     *          1. Read and validate chip ID (expected: 0x32)
     *          2. Perform soft reset via control register
     *          3. Load 11 factory trim parameters from NVM (registers 0x5D-0x71)
     *          4. Configure power mode to normal operation
     *          5. Set output data rate and oversampling
     *          6. Register compass instance with AP_Compass frontend
     *          7. Schedule periodic _update() callback with HAL scheduler
     * 
     * @return true if initialization successful, false if any step fails
     * 
     * @note Called once during probe() before driver instance is returned
     * @see _load_trim_values() for trim parameter extraction
     */
    bool init();
    
    /**
     * @brief Read raw data from sensor and publish compensated measurements
     * 
     * @details This is the periodic callback registered with the HAL scheduler.
     *          Data acquisition process:
     *          1. Read 8 bytes from data registers (0x42-0x49)
     *          2. Extract X, Y, Z magnetic field (13-bit signed values)
     *          3. Extract RHALL (Hall resistance for temperature compensation)
     *          4. Apply compensation formulas using trim parameters
     *          5. Convert from raw ADC to milligauss units
     *          6. Apply configured rotation matrix
     *          7. Publish field vector to compass frontend
     *          
     *          The BMM150 uses a unique compensation algorithm where RHALL
     *          measurement is used to adjust sensitivity based on temperature.
     * 
     * @note Runs at scheduler rate (typically 100 Hz)
     * @note Holds device semaphore during register reads
     */
    void _update();
    
    /**
     * @brief Load factory trim/calibration parameters from device NVM
     * 
     * @details The BMM150 stores 11 trim parameters in non-volatile memory
     *          (registers 0x5D through 0x71) that are unique to each sensor die.
     *          These parameters compensate for manufacturing process variations.
     *          
     *          Trim parameters loaded into _dig structure:
     *          - dig_x1, dig_y1: X/Y axis offset coefficients
     *          - dig_x2, dig_y2: X/Y axis sensitivity coefficients
     *          - dig_z1, dig_z2, dig_z3, dig_z4: Z axis compensation coefficients
     *          - dig_xy1, dig_xy2: Cross-axis correction factors
     *          - dig_xyz1: Common sensitivity scaling factor
     *          
     *          These values are used by _compensate_xy() and _compensate_z()
     *          to convert raw ADC readings to accurate magnetic field values.
     * 
     * @return true if all trim values read successfully, false on I2C error
     * 
     * @note Trim values are read-only and set at factory calibration
     * @note Called once during init() before normal operation begins
     */
    bool _load_trim_values();
    
    /**
     * @brief Compensate raw X or Y axis magnetic field measurement
     * 
     * @details Applies Bosch compensation formula for X/Y axes that corrects for:
     *          - Temperature drift using RHALL (Hall resistance) measurement
     *          - Manufacturing sensitivity variations using dig_xy1, dig_xy2
     *          - Axis-specific offsets using dig_x1/dig_x2 or dig_y1/dig_y2
     *          - Common scaling using dig_xyz1
     *          
     *          The algorithm uses floating-point arithmetic to compute:
     *          compensated_xy = (raw_xy - offset(RHALL)) * sensitivity(RHALL, trim)
     *          
     *          Output is in units of 1/16 μT which is then scaled to milligauss.
     * 
     * @param[in] xy Raw X or Y axis ADC value (13-bit signed, range ±4096)
     * @param[in] rhall Hall resistance measurement (14-bit value, typical ~1500-2500)
     * @param[in] txy1 Axis-specific trim parameter 1 (dig_x1 or dig_y1)
     * @param[in] txy2 Axis-specific trim parameter 2 (dig_x2 or dig_y2)
     * 
     * @return Compensated magnetic field in 1/16 μT units (requires further scaling)
     * 
     * @note This method is const as it uses only input parameters and immutable _dig members
     * @note Called by _update() for both X and Y axis measurements
     */
    int16_t _compensate_xy(int16_t xy, uint32_t rhall, int32_t txy1, int32_t txy2) const;
    
    /**
     * @brief Compensate raw Z axis magnetic field measurement
     * 
     * @details Applies Bosch compensation formula for Z axis using separate
     *          coefficient set (dig_z1, dig_z2, dig_z3, dig_z4). The Z axis
     *          compensation differs from X/Y due to sensor geometry.
     *          
     *          Algorithm accounts for:
     *          - Temperature-dependent sensitivity via RHALL
     *          - Non-linear response characteristics of Z Hall sensor
     *          - Factory calibration offsets
     *          
     *          Formula involves division and multiplication with trim parameters
     *          to produce output in 1/16 μT units.
     * 
     * @param[in] z Raw Z axis ADC value (15-bit signed, range ±16384)
     * @param[in] rhall Hall resistance measurement (14-bit value)
     * 
     * @return Compensated Z field in 1/16 μT units (requires further scaling to mG)
     * 
     * @note Z axis has different bit depth and compensation than X/Y axes
     * @note Called by _update() for Z axis measurement
     */
    int16_t _compensate_z(int16_t z, uint32_t rhall) const;

    AP_HAL::OwnPtr<AP_HAL::Device> _dev; ///< I2C or SPI device handle with exclusive ownership

    uint8_t _compass_instance; ///< Frontend compass instance ID for data publishing

    /**
     * @brief Factory trim/calibration parameters loaded from device NVM
     * 
     * @details This structure holds 11 unique trim parameters that are burned
     *          into each BMM150 sensor during factory calibration. These values
     *          compensate for manufacturing process variations in Hall sensor
     *          sensitivity and offset characteristics.
     *          
     *          Parameters are used in compensation formulas to convert raw ADC
     *          readings to accurate magnetic field measurements. Values are
     *          loaded once during init() from registers 0x5D-0x71 and remain
     *          constant throughout sensor lifetime.
     *          
     *          Typical value ranges (vary per device):
     *          - x1, y1: -128 to +127 (offset trim for X/Y)
     *          - x2, y2: -128 to +127 (sensitivity trim for X/Y)
     *          - z1: 0 to 65535 (Z axis base resistance)
     *          - z2, z3, z4: signed 16-bit (Z axis compensation coefficients)
     *          - xy1, xy2: Cross-axis correction factors
     *          - xyz1: 0 to 65535 (common scaling factor, typically ~10000)
     * 
     * @note Values are read-only after initialization
     * @note Do not modify - these are factory-calibrated per sensor die
     * 
     * @see _load_trim_values() for extraction from NVM registers
     * @see _compensate_xy() and _compensate_z() for usage in compensation
     */
    struct {
        int8_t x1;      ///< X axis offset trim parameter (8-bit signed)
        int8_t y1;      ///< Y axis offset trim parameter (8-bit signed)
        int8_t x2;      ///< X axis sensitivity trim parameter (8-bit signed)
        int8_t y2;      ///< Y axis sensitivity trim parameter (8-bit signed)
        uint16_t z1;    ///< Z axis base resistance value (16-bit unsigned)
        int16_t z2;     ///< Z axis compensation coefficient 2 (16-bit signed)
        int16_t z3;     ///< Z axis compensation coefficient 3 (16-bit signed)
        int16_t z4;     ///< Z axis compensation coefficient 4 (16-bit signed)
        uint8_t xy1;    ///< XY cross-axis correction factor 1 (8-bit unsigned)
        int8_t xy2;     ///< XY cross-axis correction factor 2 (8-bit signed)
        uint16_t xyz1;  ///< Common axis scaling factor (16-bit unsigned, typically ~10000)
    } _dig;

    uint32_t _last_read_ms;     ///< Timestamp of last successful sensor read (milliseconds)
    enum Rotation _rotation;    ///< Sensor mounting orientation (ROTATION_NONE, ROTATION_YAW_90, etc.)
    bool _force_external;       ///< If true, treat as external compass regardless of board config
};

#endif  // AP_COMPASS_BMM150_ENABLED
