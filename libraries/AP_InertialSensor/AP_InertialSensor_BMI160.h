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
 * @file AP_InertialSensor_BMI160.h
 * @brief Bosch BMI160 low-power 6-axis IMU driver
 * 
 * Implements backend driver for Bosch BMI160 single-chip IMU optimized for
 * low power consumption with integrated accelerometer and gyroscope.
 */

#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/SPIDevice.h>
#include <AP_HAL/I2CDevice.h>

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"

#ifndef BMI160_DEFAULT_ROTATION
#define BMI160_DEFAULT_ROTATION ROTATION_NONE
#endif

/**
 * @class AP_InertialSensor_BMI160
 * @brief Backend driver for BMI160 integrated IMU
 * 
 * @details Single-chip 6-axis IMU driver supporting:
 *          - Unified SPI/I2C interface (single chip select)
 *          - Gyro range: ±2000 deg/s
 *          - Accel range: ±16g
 *          - Combined FIFO for accel and gyro data
 *          - Low power modes for battery-operated applications
 *          - Integrated interrupt engine
 *          
 *          Advantages over BMI055/088:
 *          - Single chip reduces board space and cost
 *          - Unified FIFO simplifies driver logic
 *          - Lower power consumption (850µA typical)
 *          - Built-in motion detection and gesture recognition
 *          
 *          FIFO handling:
 *          - Watermark interrupt triggers FIFO read
 *          - Frame headers identify accel/gyro/time packets
 *          - Parser reconstructs synchronized sensor pairs
 *          - Overflow detection via status register
 * 
 * @note Single-chip design more susceptible to PCB vibration than dual-die BMI088
 * @note FIFO frame format includes header byte identifying data type
 * @warning Must soft-reset chip and wait 50ms before configuration
 */
class AP_InertialSensor_BMI160 : public AP_InertialSensor_Backend {
public:
    /**
     * @brief Probe and initialize BMI160 sensor via SPI interface
     * 
     * @details Attempts to detect and initialize a BMI160 IMU on the provided
     *          SPI device. Performs chip ID verification, soft reset, and
     *          configuration of accelerometer, gyroscope, and FIFO.
     *          
     *          Initialization sequence:
     *          1. Read and verify chip ID (0xD1)
     *          2. Perform soft reset and wait 50ms
     *          3. Configure accelerometer (±16g range, 1600Hz ODR)
     *          4. Configure gyroscope (±2000 deg/s, 3200Hz ODR)
     *          5. Configure FIFO with watermark interrupt
     *          6. Register periodic callback for data polling
     * 
     * @param[in] imu Reference to AP_InertialSensor instance for sensor registration
     * @param[in] dev SPI device handle (ownership transferred to driver)
     * @param[in] rotation Board rotation for sensor orientation correction
     * 
     * @return Pointer to initialized backend on success, nullptr on failure
     * 
     * @note SPI communication uses mode 0 (CPOL=0, CPHA=0) at up to 10MHz
     * @note Device must be powered and stable for at least 10ms before probe
     * 
     * @see AP_InertialSensor_Backend::probe()
     */
    static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu,
                                            AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev,
                                            enum Rotation rotation=BMI160_DEFAULT_ROTATION);

    /**
     * @brief Probe and initialize BMI160 sensor via I2C interface
     * 
     * @details Attempts to detect and initialize a BMI160 IMU on the provided
     *          I2C device. Performs same initialization as SPI probe but uses
     *          I2C protocol for register access. Supports standard (100kHz)
     *          and fast mode (400kHz) I2C speeds.
     *          
     *          I2C addresses: 0x68 (SDO low) or 0x69 (SDO high)
     * 
     * @param[in] imu Reference to AP_InertialSensor instance for sensor registration
     * @param[in] dev I2C device handle (ownership transferred to driver)
     * @param[in] rotation Board rotation for sensor orientation correction
     * 
     * @return Pointer to initialized backend on success, nullptr on failure
     * 
     * @note I2C interface typically slower than SPI for high-rate FIFO reads
     * @note Device address determined by SDO pin state (see datasheet)
     * 
     * @see AP_InertialSensor_Backend::probe()
     */
    static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu,
                                            AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev,
                                            enum Rotation rotation=BMI160_DEFAULT_ROTATION);

    /**
     * @brief Configure the sensors and start reading routine
     * 
     * @details Called by AP_InertialSensor to begin continuous sensor data
     *          acquisition. Registers periodic callback with HAL scheduler
     *          to poll FIFO at configured sample rate.
     *          
     *          This method:
     *          - Verifies sensor configuration is complete
     *          - Registers _poll_data() as periodic callback (1kHz typical)
     *          - Enables FIFO watermark interrupts if available
     *          - Initializes sample timestamp synchronization
     * 
     * @note Called once during AP_InertialSensor::_startup_sensors()
     * @note Must be called after successful probe() initialization
     * 
     * @see AP_InertialSensor_Backend::start()
     */
    void start() override;

    /**
     * @brief Update sensor data and push samples to frontend
     * 
     * @details Called by AP_InertialSensor at main loop rate (typically 400Hz)
     *          to process accumulated FIFO samples and update sensor state.
     *          
     *          Processing steps:
     *          - Retrieves samples accumulated by _poll_data() callback
     *          - Applies rotation and scaling transformations
     *          - Publishes samples to AP_InertialSensor frontend
     *          - Updates sensor health indicators
     * 
     * @return true if new samples were processed, false otherwise
     * 
     * @note Actual sensor reads occur in _poll_data() at higher rate
     * @note This method primarily handles sample post-processing
     * 
     * @see AP_InertialSensor_Backend::update()
     */
    bool update() override;

private:
    AP_InertialSensor_BMI160(AP_InertialSensor &imu,
                             AP_HAL::OwnPtr<AP_HAL::Device> dev,
                             enum Rotation rotation);

    /**
     * If the macro BMI160_DEBUG is defined, check if there are errors reported
     * on the device's error register and panic if so. The implementation is
     * empty if the BMI160_DEBUG isn't defined.
     */
    void _check_err_reg();

    /**
     * Try to perform initialization of the BMI160 device.
     *
     * The device semaphore must be taken and released by the caller.
     *
     * @return true on success, false otherwise.
     */
    bool _hardware_init();

    /**
     * Try to initialize this driver.
     *
     * Do sensor and other required initializations.
     *
     * @return true on success, false otherwise.
     */
    bool _init();

    /**
     * Configure accelerometer sensor. The device semaphore must already be
     * taken before calling this function.
     *
     * @return true on success, false otherwise.
     */
    bool _configure_accel();

    /**
     * Configure gyroscope sensor. The device semaphore must already be
     * taken before calling this function.
     *
     * @return true on success, false otherwise.
     */
    bool _configure_gyro();

    /**
     * Configure INT1 pin as watermark interrupt pin at the level of one sample
     * if using fifo or data-ready pin otherwise.
     *
     * @return true on success, false otherwise.
     */
    bool _configure_int1_pin();

    /**
     * Configure FIFO.
     *
     * @return true on success, false otherwise.
     */
    bool _configure_fifo();

    /**
     * Device periodic callback to read data from the sensors.
     */
    void _poll_data();

    /**
     * Read samples from fifo.
     */
    void _read_fifo();

    AP_HAL::OwnPtr<AP_HAL::Device> _dev;
    enum Rotation _rotation;

    float _accel_scale;
    float _gyro_scale;

    AP_HAL::DigitalSource *_int1_pin;
};
