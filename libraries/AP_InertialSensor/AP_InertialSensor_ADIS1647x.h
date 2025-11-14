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
 * @file AP_InertialSensor_ADIS1647x.h
 * @brief ADIS16470/ADIS16475/ADIS16477 family tactical-grade MEMS IMU driver
 * 
 * @details Implements backend driver for Analog Devices ADIS1647x series tactical-grade MEMS IMUs.
 *          These sensors provide high-accuracy 6-axis inertial measurement with integrated
 *          temperature sensors and advanced FIFO buffering capabilities.
 *          
 *          Key characteristics of ADIS1647x sensors:
 *          - Uses 16-bit register addressing (unusual for IMU sensors)
 *          - Requires dedicated SPI bus for optimal performance
 *          - Supports burst read protocol for efficient data transfer
 *          - Provides delta angle/velocity outputs for integrated samples
 *          - Includes data counter and CRC validation for robust communication
 *          
 *          Hardware configuration notes:
 *          - Must be sole device on SPI bus for consistent timing
 *          - Requires DRDY (Data Ready) GPIO pin for synchronization
 *          - SPI clock speed limited to 2MHz maximum
 * 
 * @note This driver uses a dedicated polling thread for consistent sample timing
 * @warning Sharing the SPI bus with other devices will degrade performance
 */

#pragma once

#include <AP_HAL/AP_HAL.h>

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"

/**
 * @class AP_InertialSensor_ADIS1647x
 * @brief Backend driver for Analog Devices ADIS1647x series tactical-grade MEMS IMUs
 * 
 * @details High-performance SPI-based driver for ADIS16470/16475/16477 IMUs supporting:
 *          
 *          **Operating Modes:**
 *          - Basic: Standard 16-bit accel/gyro output
 *          - AG32: 32-bit accel/gyro output for extended range
 *          - Delta32: 32-bit delta angle/velocity integrated samples (preferred)
 *          
 *          **Communication Protocol:**
 *          - SPI burst read mode transfers 20+ bytes per transaction
 *          - CRC checksum validation on every sample for data integrity
 *          - Data counter tracks dropped samples and communication errors
 *          - Supports multiple decimation rates for optimal throughput
 *          
 *          **Sensor Specifications (typical):**
 *          - Gyro measurement range: ±2000 deg/s
 *          - Accel measurement range: ±40g
 *          - Internal sample rate: Up to 2kHz
 *          - Output sample rate: Decimated to match vehicle loop rate (typically 400Hz-1kHz)
 *          - Output format: Delta angle (radians) and delta velocity (m/s) as integrated samples
 *          - Temperature range: Integrated sensor with averaging for compensation readiness
 *          
 *          **Thread Architecture:**
 *          - Uses dedicated high-priority polling thread for sensor reads
 *          - Ensures consistent sample timing independent of main loop jitter
 *          - DRDY (Data Ready) GPIO pin provides hardware synchronization
 *          
 *          **Data Flow:**
 *          1. DRDY interrupt indicates new sample available
 *          2. Burst read command retrieves all sensor data in single transaction
 *          3. CRC validation ensures data integrity
 *          4. Data counter checked for missed samples
 *          5. Delta angle/velocity accumulated into backend buffers
 *          6. Temperature averaged for thermal compensation
 * 
 * @note Burst read requires specific command sequence per ADIS1647x datasheet
 * @note Uses 16-bit register addressing unlike most IMU sensors (8-bit)
 * @warning SPI speed must not exceed 2MHz for reliable communication
 * @warning Device should be sole sensor on SPI bus for optimal performance
 * @warning Ensure adequate SPI bus bandwidth for high sample rates
 * 
 * @see AP_InertialSensor_Backend
 */
class AP_InertialSensor_ADIS1647x : public AP_InertialSensor_Backend {
public:
    /**
     * @brief Probe and detect ADIS1647x IMU on SPI bus
     * 
     * @details Static factory method that attempts to detect and initialize an ADIS1647x
     *          series IMU on the provided SPI device. Performs the following detection sequence:
     *          1. Attempts SPI communication with device
     *          2. Reads product ID register to identify sensor model
     *          3. Validates product ID against supported ADIS16470/16475/16477 variants
     *          4. Creates driver instance if detection successful
     *          5. Performs initial sensor configuration and self-test
     *          
     *          Product ID validation ensures correct sensor variant and prevents
     *          misconfiguration with incompatible devices.
     * 
     * @param[in,out] imu           Reference to AP_InertialSensor frontend for sensor registration
     * @param[in]     dev           SPI device pointer with ownership transfer (takes ownership on success)
     * @param[in]     rotation      Sensor mounting orientation (body frame rotation matrix)
     * @param[in]     drdy_gpio     GPIO pin number for DRDY (Data Ready) hardware interrupt
     * 
     * @return Pointer to initialized backend driver instance on success, nullptr if probe fails
     * 
     * @note This method transfers ownership of dev pointer on successful probe
     * @note Probe failure returns nullptr and releases device ownership
     * @warning Must be called during hardware initialization phase before scheduler starts
     * @warning drdy_gpio must be valid and correctly configured as input with interrupt capability
     */
    static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu,
                                            AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                            enum Rotation rotation,
                                            uint8_t drdy_gpio);

    /**
     * @brief Configure the sensor and start continuous reading routine
     * 
     * @details Completes sensor initialization and launches dedicated polling thread:
     *          1. Configures sensor operating mode (Basic/AG32/Delta32)
     *          2. Sets decimation filter for desired output rate
     *          3. Enables data counter and CRC validation
     *          4. Configures DRDY interrupt output
     *          5. Performs initial sensor reads to flush pipeline
     *          6. Launches high-priority polling thread
     *          7. Registers backend with frontend for data publication
     *          
     *          After start() completes, the polling thread runs continuously,
     *          reading sensor data on DRDY interrupts and accumulating into
     *          backend buffers for frontend consumption.
     * 
     * @note Called once during sensor initialization after successful probe
     * @note Polling thread runs at higher priority than main vehicle loop
     * @warning Must complete successfully before vehicle can arm
     * @warning Failure to start will disable this IMU instance
     */
    void start() override;
    
    /**
     * @brief Update frontend with accumulated sensor data
     * 
     * @details Called periodically by AP_InertialSensor frontend to retrieve sensor data.
     *          Transfers accumulated delta angle/velocity samples from backend buffers
     *          to frontend for EKF consumption. Also updates temperature readings.
     *          
     *          This method:
     *          - Retrieves samples accumulated since last update
     *          - Applies sensor rotation matrix to transform to body frame
     *          - Publishes gyro and accel data to frontend
     *          - Updates temperature for thermal compensation
     *          - Checks for data counter discontinuities (dropped samples)
     *          
     *          Designed for lock-free data transfer from polling thread to main thread.
     * 
     * @return true if new samples available and published, false if no new data
     * 
     * @note Called at main loop rate (typically 400Hz for Copter)
     * @note Returns false if polling thread has not provided new samples
     * @warning Data counter discontinuities logged as errors but do not prevent update
     */
    bool update() override;

private:
    /**
     * @brief Private constructor for ADIS1647x driver instance
     * 
     * @details Constructs driver instance with device configuration. Called internally
     *          by probe() method after successful device detection. Initializes member
     *          variables and stores device parameters for subsequent configuration.
     * 
     * @param[in,out] imu        Reference to AP_InertialSensor frontend
     * @param[in]     dev        SPI device pointer with ownership transfer
     * @param[in]     rotation   Sensor mounting orientation matrix
     * @param[in]     drdy_gpio  Data Ready GPIO pin number
     * 
     * @note Constructor is private - instances created only via probe() factory method
     * @note Does not perform hardware initialization - see init() method
     */
    AP_InertialSensor_ADIS1647x(AP_InertialSensor &imu,
                                AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                enum Rotation rotation,
                                uint8_t drdy_gpio);

    /**
     * @brief Initialize sensor hardware and configure operating parameters
     * 
     * @details Performs complete sensor initialization sequence:
     *          1. Validate product ID and sensor model
     *          2. Perform sensor self-test if enabled
     *          3. Configure decimation filter based on backend rate
     *          4. Select operating mode (Basic/AG32/Delta32)
     *          5. Enable data counter and CRC validation
     *          6. Configure DRDY interrupt output
     *          7. Calculate scale factors for delta angle/velocity conversion
     *          8. Flush sensor data pipeline with initial reads
     * 
     * @return true if initialization successful, false on any error
     * 
     * @note Called internally by start() method
     * @warning Initialization failure will disable this IMU instance
     */
    bool init();
    
    /**
     * @brief Read sensor data in Basic 16-bit mode
     * 
     * @details Performs burst read of gyro and accel data in 16-bit format.
     *          This mode provides standard resolution output suitable for
     *          lower-rate applications. Less commonly used than 32-bit modes.
     * 
     * @note Burst read transfers all axes in single SPI transaction
     * @note Called from loop() based on selected opmode
     */
    void read_sensor16(void);
    
    /**
     * @brief Read sensor data in AG32 32-bit mode
     * 
     * @details Performs burst read of gyro and accel data in 32-bit format.
     *          Provides extended range and resolution compared to 16-bit mode.
     *          Data represents instantaneous angular rate and acceleration.
     * 
     * @note Burst read transfers 32-bit values for all 6 axes
     * @note Called from loop() when opmode == OpMode::AG32
     */
    void read_sensor32(void);
    
    /**
     * @brief Read sensor data in Delta32 mode (preferred)
     * 
     * @details Performs burst read of integrated delta angle and delta velocity in 32-bit format.
     *          This is the preferred operating mode as it provides:
     *          - Integrated samples reducing quantization noise
     *          - Automatic coning/sculling compensation
     *          - Better resolution for low-rate output
     *          
     *          Delta values represent integrated angular change (radians) and
     *          velocity change (m/s) since last sample, already scaled.
     * 
     * @note Burst read transfers delta angle (gyro integration) and delta velocity (accel integration)
     * @note Called from loop() when opmode == OpMode::Delta32 (default for high-performance)
     * @note Delta angle/velocity outputs optimal for EKF integration
     */
    void read_sensor32_delta(void);
    
    /**
     * @brief Main polling loop running in dedicated thread
     * 
     * @details Infinite loop that:
     *          1. Waits for DRDY interrupt indicating new sample
     *          2. Calls appropriate read function based on opmode
     *          3. Validates CRC and data counter
     *          4. Accumulates samples into backend buffers
     *          5. Averages temperature readings
     *          
     *          Runs continuously at sensor sample rate (up to 2kHz internal,
     *          decimated to backend rate). High thread priority ensures
     *          consistent timing regardless of main loop load.
     * 
     * @note Runs in dedicated thread created by start()
     * @note Thread priority higher than main vehicle loop
     * @warning Thread must not block or delay to maintain sample timing
     */
    void loop(void);
    
    /**
     * @brief Verify sensor product ID and model
     * 
     * @details Reads PROD_ID register to identify sensor model and validate
     *          device is a supported ADIS1647x variant (16470/16475/16477).
     *          Used during probe() to confirm correct device detected.
     * 
     * @param[out] id  Product ID value read from sensor
     * 
     * @return true if product ID read successful and matches supported models, false otherwise
     * 
     * @note Different ADIS1647x models have different product IDs
     * @note Product ID validation prevents driver misconfiguration
     */
    bool check_product_id(uint16_t &id);

    /**
     * @brief Read 16-bit register from ADIS1647x sensor
     * 
     * @details Performs SPI transaction to read a single 16-bit register.
     *          ADIS1647x uses 16-bit addressing and 16-bit data unlike most
     *          IMU sensors which use 8-bit addressing.
     * 
     * @param[in] regnum  Register number to read (16-bit address)
     * 
     * @return 16-bit register value read from sensor
     * 
     * @note ADIS1647x register reads require specific SPI timing per datasheet
     * @warning Must not be called from interrupt context
     */
    uint16_t read_reg16(uint8_t regnum) const;

    /**
     * @brief Write 16-bit register to ADIS1647x sensor
     * 
     * @details Performs SPI transaction to write a single 16-bit register.
     *          Optionally reads back register value to confirm write success.
     * 
     * @param[in] regnum   Register number to write (16-bit address)
     * @param[in] value    16-bit value to write to register
     * @param[in] confirm  If true, read back register to verify write (default: false)
     * 
     * @return true if write successful (and verification passed if enabled), false otherwise
     * 
     * @note Write confirmation adds additional SPI transaction latency
     * @note Some configuration registers require multiple writes to fully configure
     * @warning Critical configuration writes should use confirm=true for safety
     */
    bool write_reg16(uint8_t regnum, uint16_t value, bool confirm=false) const;
    
    /// SPI device interface with ownership
    AP_HAL::OwnPtr<AP_HAL::Device> dev;

    /**
     * @enum OpMode
     * @brief Sensor operating mode selection
     * 
     * @details Defines available data output formats for ADIS1647x:
     *          - Basic: 16-bit instantaneous gyro/accel outputs
     *          - AG32: 32-bit instantaneous gyro/accel outputs
     *          - Delta32: 32-bit integrated delta angle/velocity (preferred for accuracy)
     */
    enum class OpMode : uint8_t {
        Basic      =1,  ///< 16-bit mode: Standard resolution instantaneous data
        AG32       =2,  ///< 32-bit mode: Extended resolution instantaneous angular rate and acceleration
        Delta32    =3   ///< 32-bit mode: Integrated delta angle (rad) and delta velocity (m/s) - preferred
    } opmode;

    /// Sensor mounting orientation matrix for body frame transformation
    enum Rotation rotation;
    
    /// GPIO pin number for DRDY (Data Ready) hardware interrupt
    uint8_t drdy_pin;

    /// Previous sample data counter value for detecting dropped samples
    uint16_t last_counter;
    
    /// Flag indicating first sensor read completed (initialization state)
    bool done_first_read;
    
    /// Accumulated temperature readings for averaging
    float temp_sum;
    
    /// Count of temperature samples accumulated in temp_sum
    uint8_t temp_count;
    
    /// Expected sensor output sample rate in Hz (after decimation)
    float expected_sample_rate_hz;

    /// Scale factor for converting raw accel values to m/s² (AG32/Basic modes)
    float accel_scale;
    
    /// Scale factor for converting raw gyro values to rad/s (AG32/Basic modes)
    float gyro_scale;
    
    /// Scale factor for converting raw delta angle to radians (Delta32 mode)
    double dangle_scale;
    
    /// Scale factor for converting raw delta velocity to m/s (Delta32 mode)
    double dvel_scale;
};
