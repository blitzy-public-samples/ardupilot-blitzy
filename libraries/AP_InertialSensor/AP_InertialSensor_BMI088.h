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
 * @file AP_InertialSensor_BMI088.h
 * @brief Bosch BMI088 high-performance 6-axis IMU driver
 * 
 * @details Implements backend driver for Bosch BMI088 automotive-grade IMU with separate
 *          accelerometer and gyroscope die, providing enhanced vibration immunity through
 *          mechanical decoupling. The BMI088 is unusual as it has separate chip-select 
 *          for accel and gyro, which means it needs two Device pointers for SPI communication.
 * 
 * @note Read dummy byte required after accel register address for correct SPI operation
 * @note Uses SPI mode 3 (CPOL=1, CPHA=1) per BMI088 specification
 */
#pragma once

#include <AP_HAL/AP_HAL.h>

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"

/**
 * @class AP_InertialSensor_BMI088
 * @brief Backend driver for BMI088 automotive-grade 6-axis IMU
 * 
 * @details The BMI088 is an enhanced version of the BMI055 architecture with improved 
 *          specifications for demanding automotive and drone applications:
 * 
 *          **Key Specifications:**
 *          - Dual SPI device topology (independent accel/gyro chips with separate chip-select)
 *          - Gyroscope range: ±2000 deg/s with 16-bit resolution
 *          - Accelerometer range: ±24g with 16-bit resolution
 *          - Sample rates: Up to 2kHz gyro, 1.6kHz accel
 *          - Enhanced vibration rejection through mechanical decoupling of dies
 *          - Temperature sensor per axis for thermal compensation
 *          - Automotive-qualified temperature range (-40°C to +105°C)
 *          
 *          **Key Differences from BMI055:**
 *          - Separate die reduces mechanical coupling (better vibration immunity)
 *          - Wider accelerometer range (±24g vs ±16g)
 *          - Different register map and initialization sequence
 *          - Requires dummy byte read after accel register address
 *          - Improved gyroscope noise performance
 *          
 *          **Driver Lifecycle:**
 *          1. probe(): Verify WHO_AM_I registers for both accel (0x1E) and gyro (0x0F)
 *          2. init(): Configure ranges, output data rates, and bandwidth filters
 *          3. Enable FIFO streaming for continuous data acquisition
 *          4. Register periodic callbacks with scheduler for data sampling
 *          5. Periodic callbacks sample FIFO buffers and publish to InertialSensor frontend
 *          
 *          **Hardware Communication:**
 *          - Uses SPI mode 3 (CPOL=1, CPHA=1) per BMI088 datasheet requirements
 *          - Accelerometer requires dummy byte after register address (special read protocol)
 *          - Both devices must respond successfully for initialization to complete
 *          - Independent periodic handlers for accel and gyro to optimize throughput
 * 
 * @note Both accelerometer and gyroscope must respond to WHO_AM_I for successful initialization
 * @note Accelerometer uses unique SPI read protocol requiring dummy byte - see read_accel_registers()
 * @warning Incorrect SPI mode or missing dummy byte will result in corrupted accelerometer data
 * 
 * @see AP_InertialSensor_Backend
 * @see libraries/AP_InertialSensor/AP_InertialSensor_BMI088.cpp for implementation details
 */
class AP_InertialSensor_BMI088 : public AP_InertialSensor_Backend {
public:
    /**
     * @brief Probe for BMI088 IMU on SPI bus and create driver instance if detected
     * 
     * @details Static factory method that attempts to detect a BMI088 IMU by verifying
     *          WHO_AM_I registers on both accelerometer (expected 0x1E) and gyroscope
     *          (expected 0x0F) devices. If both chips respond correctly, allocates and
     *          initializes a new driver instance.
     *          
     *          The BMI088 requires two separate SPI device pointers because the accelerometer
     *          and gyroscope have independent chip-select lines, unlike typical IMUs that
     *          share a single chip-select.
     *          
     *          Probe sequence:
     *          1. Verify accelerometer WHO_AM_I register reads 0x1E
     *          2. Verify gyroscope WHO_AM_I register reads 0x0F
     *          3. If both succeed, construct driver instance and call init()
     *          4. If initialization succeeds, return driver pointer
     *          5. If any step fails, return nullptr
     * 
     * @param[in] imu Reference to AP_InertialSensor frontend for registering this sensor
     * @param[in] dev_accel OwnPtr to SPI device for accelerometer communication (transferred to driver)
     * @param[in] dev_gyro OwnPtr to SPI device for gyroscope communication (transferred to driver)
     * @param[in] rotation Rotation matrix to apply for board orientation (ROTATION_NONE, ROTATION_YAW_90, etc.)
     * 
     * @return Pointer to new AP_InertialSensor_BMI088 instance if probe successful, nullptr if failed
     * 
     * @note Both device pointers must be valid and communicate successfully
     * @note Device pointers are transferred to the driver instance (ownership transfer)
     * @note Called during board initialization from AP_InertialSensor::_detect_backends()
     * 
     * @see AP_InertialSensor_Backend::probe()
     */
    static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu,
                                            AP_HAL::OwnPtr<AP_HAL::Device> dev_accel,
                                            AP_HAL::OwnPtr<AP_HAL::Device> dev_gyro,
                                            enum Rotation rotation);

    /**
     * @brief Configure the sensors and start periodic reading routine
     * 
     * @details Registers periodic callback handlers with the HAL scheduler to continuously
     *          read FIFO data from both accelerometer and gyroscope. This method is called
     *          once during system initialization after successful probe.
     *          
     *          Start sequence:
     *          1. Register accel_periodic_handle for read_fifo_accel() callback
     *          2. Register gyro_periodic_handle for read_fifo_gyro() callback
     *          3. Callbacks run at appropriate rates (typically 1kHz) in interrupt context
     *          
     *          Once started, the driver continuously samples data via FIFO reads and
     *          publishes to the InertialSensor frontend without further intervention.
     * 
     * @note Called from AP_InertialSensor::_start_backends() during initialization
     * @note Periodic callbacks run in scheduler interrupt context
     * @warning Must be called only once per driver instance
     * 
     * @see read_fifo_accel()
     * @see read_fifo_gyro()
     */
    void start() override;

    /**
     * @brief Update sensor state and publish accumulated samples
     * 
     * @details Called periodically from the main thread to process samples accumulated
     *          by the interrupt-driven FIFO readers. This method typically has minimal
     *          work since actual data acquisition happens in read_fifo_accel() and
     *          read_fifo_gyro() callbacks.
     *          
     *          Update activities:
     *          - Check for sensor health and error conditions
     *          - Update temperature readings if available
     *          - Perform any main-thread processing required
     *          
     *          The heavy lifting (SPI transactions and sample accumulation) occurs in
     *          the periodic callback handlers registered by start().
     * 
     * @return true if update successful and sensor healthy, false if error detected
     * 
     * @note Called from main loop via AP_InertialSensor::update()
     * @note Runs in main thread context, not interrupt context
     * 
     * @see start()
     */
    bool update() override;

private:
    /**
     * @brief Private constructor for BMI088 driver instance
     * 
     * @details Constructs a new BMI088 backend driver with provided SPI device pointers
     *          and rotation. This constructor is private and only called from the static
     *          probe() method after successful device detection.
     *          
     *          Constructor responsibilities:
     *          - Store device pointers for accelerometer and gyroscope
     *          - Store rotation matrix for board orientation
     *          - Initialize member variables to default values
     *          - Call init() to complete hardware configuration
     * 
     * @param[in] imu Reference to AP_InertialSensor frontend
     * @param[in] dev_accel OwnPtr to accelerometer SPI device (ownership transferred)
     * @param[in] dev_gyro OwnPtr to gyroscope SPI device (ownership transferred)
     * @param[in] rotation Board rotation to apply to sensor measurements
     * 
     * @note Only called from probe() static method
     * @note Device pointers must be valid and successfully probed
     * 
     * @see probe()
     */
    AP_InertialSensor_BMI088(AP_InertialSensor &imu,
                             AP_HAL::OwnPtr<AP_HAL::Device> dev_accel,
                             AP_HAL::OwnPtr<AP_HAL::Device> dev_gyro,
                             enum Rotation rotation);

    /**
     * @brief Initialize accelerometer hardware configuration
     * 
     * @details Configures BMI088 accelerometer registers including range, output data rate,
     *          bandwidth, power mode, and FIFO settings. This method performs the chip-specific
     *          initialization sequence required by the BMI088 accelerometer.
     *          
     *          Initialization sequence:
     *          1. Soft reset accelerometer
     *          2. Configure range (±24g, ±12g, ±6g, or ±3g)
     *          3. Set output data rate and bandwidth filter
     *          4. Enable active mode
     *          5. Configure FIFO if enabled
     * 
     * @return true if accelerometer initialized successfully, false on communication error
     * 
     * @note Called from init() during driver initialization
     * @note Requires dummy byte read protocol for register access
     * 
     * @see setup_accel_config()
     * @see write_accel_register()
     */
    bool accel_init();

    /**
     * @brief Initialize gyroscope hardware configuration
     * 
     * @details Configures BMI088 gyroscope registers including range, output data rate,
     *          bandwidth, and FIFO settings. The gyroscope initialization is more straightforward
     *          than accelerometer as it uses standard SPI read/write protocol.
     *          
     *          Initialization sequence:
     *          1. Soft reset gyroscope
     *          2. Configure range (±2000 deg/s, ±1000 deg/s, ±500 deg/s, or ±250 deg/s)
     *          3. Set output data rate and bandwidth filter
     *          4. Configure FIFO if enabled
     * 
     * @return true if gyroscope initialized successfully, false on communication error
     * 
     * @note Called from init() during driver initialization
     * @note Uses standard SPI protocol (no dummy byte required)
     */
    bool gyro_init();

    /**
     * @brief Initialize complete BMI088 driver
     * 
     * @details Master initialization routine that configures both accelerometer and gyroscope,
     *          registers the sensors with the InertialSensor frontend, and prepares for data
     *          acquisition. Called from constructor after device detection.
     *          
     *          Initialization sequence:
     *          1. Call accel_init() to configure accelerometer
     *          2. Call gyro_init() to configure gyroscope
     *          3. Register accelerometer instance with frontend
     *          4. Register gyroscope instance with frontend
     *          5. Allocate FIFO buffers if needed
     * 
     * @return true if complete initialization successful, false if any step fails
     * 
     * @note Called from constructor during probe sequence
     * @note If init() fails, probe() returns nullptr
     * @warning Driver is unusable if init() returns false
     * 
     * @see accel_init()
     * @see gyro_init()
     */
    bool init();

    /**
     * @brief Read accelerometer data from FIFO buffer
     * 
     * @details Periodic callback that reads accumulated accelerometer samples from the
     *          BMI088 FIFO, processes them, and publishes to the InertialSensor frontend.
     *          This method runs in scheduler interrupt context at configured rate (typically 1kHz).
     *          
     *          Read sequence:
     *          1. Check FIFO level/status registers
     *          2. Read FIFO data using read_accel_registers() (handles dummy byte)
     *          3. Parse FIFO frames and extract accelerometer samples
     *          4. Apply rotation and scaling to convert to m/s²
     *          5. Publish samples via _publish_accel()
     *          6. Optionally read temperature sensor
     *          
     *          FIFO usage reduces SPI transaction overhead and ensures no samples are
     *          lost between reads.
     * 
     * @note Runs in scheduler interrupt context at high priority
     * @note Must complete quickly to avoid blocking other interrupts
     * @note Registered as periodic callback by start()
     * @warning Uses special BMI088 accel read protocol with dummy byte
     * 
     * @see read_accel_registers()
     * @see start()
     */
    void read_fifo_accel();

    /**
     * @brief Read gyroscope data from FIFO buffer
     * 
     * @details Periodic callback that reads accumulated gyroscope samples from the
     *          BMI088 FIFO, processes them, and publishes to the InertialSensor frontend.
     *          This method runs in scheduler interrupt context at configured rate (typically 1kHz).
     *          
     *          Read sequence:
     *          1. Check FIFO level/status registers
     *          2. Read FIFO data using standard SPI transaction
     *          3. Parse FIFO frames and extract gyroscope samples
     *          4. Apply rotation and scaling to convert to rad/s
     *          5. Publish samples via _publish_gyro()
     *          
     *          The gyroscope uses standard SPI protocol, unlike accelerometer which
     *          requires dummy byte handling.
     * 
     * @note Runs in scheduler interrupt context at high priority
     * @note Must complete quickly to avoid blocking other interrupts
     * @note Registered as periodic callback by start()
     * 
     * @see start()
     */
    void read_fifo_gyro();

    /**
     * @brief Read from accelerometer registers using BMI088-specific SPI protocol
     * 
     * @details The BMI088 accelerometer requires a unique SPI read protocol where a dummy
     *          byte must be read after the register address byte before valid data appears.
     *          This is different from typical SPI sensors and must be handled correctly.
     *          
     *          BMI088 accel read protocol:
     *          1. Assert chip-select
     *          2. Write register address with read bit set (0x80 | reg)
     *          3. Read and discard dummy byte (always returns 0x00 or undefined)
     *          4. Read actual register data (1 or more bytes)
     *          5. De-assert chip-select
     *          
     *          This method encapsulates the special protocol so callers don't need to
     *          handle the dummy byte manually.
     * 
     * @param[in] reg Register address to read (0x00-0x7F, read bit added automatically)
     * @param[out] data Pointer to buffer to receive register data
     * @param[in] len Number of bytes to read from registers
     * 
     * @return true if read successful, false on SPI communication error
     * 
     * @note Dummy byte handling is critical for correct operation
     * @note This method is specific to BMI088 accelerometer, gyro uses standard SPI
     * @warning Missing dummy byte read will result in corrupted data
     * 
     * @see accel_init()
     * @see read_fifo_accel()
     */
    bool read_accel_registers(uint8_t reg, uint8_t *data, uint8_t len);

    /**
     * @brief Write to accelerometer register with retry mechanism
     * 
     * @details Writes a single byte to a BMI088 accelerometer register with automatic
     *          retry logic. Verifies write by reading back register value to ensure
     *          communication reliability, which is important for safety-critical sensor
     *          configuration.
     *          
     *          Write-and-verify sequence:
     *          1. Write register value
     *          2. Delay for register update
     *          3. Read back register using read_accel_registers()
     *          4. Compare readback value with written value
     *          5. Retry up to 5 times if mismatch
     * 
     * @param[in] reg Register address to write (0x00-0x7F)
     * @param[in] v Value byte to write to register
     * 
     * @return true if write verified successfully, false if verification fails after retries
     * 
     * @note Used during initialization to ensure reliable sensor configuration
     * @note Retry mechanism handles occasional SPI communication glitches
     * 
     * @see read_accel_registers()
     * @see setup_accel_config()
     */
    bool write_accel_register(uint8_t reg, uint8_t v);

    /**
     * @brief Configure accelerometer registers for operational use
     * 
     * @details Applies complete accelerometer configuration including range, bandwidth,
     *          output data rate, and FIFO settings. This method may be called multiple
     *          times during initialization if initial configuration attempts fail.
     *          
     *          Configuration parameters set:
     *          - Accelerometer range (±3g, ±6g, ±12g, or ±24g)
     *          - Output data rate (12.5Hz to 1600Hz)
     *          - Bandwidth filter settings
     *          - FIFO watermark and mode
     *          - Interrupt configuration
     *          
     *          The driver tracks configuration state with done_accel_config flag
     *          and accel_config_count to ensure successful setup before data acquisition.
     * 
     * @return true if accelerometer configured successfully, false on error
     * 
     * @note Called from accel_init() during initialization
     * @note May retry configuration if initial attempts fail
     * @note Uses write_accel_register() with verification for reliability
     * 
     * @see accel_init()
     * @see write_accel_register()
     */
    bool setup_accel_config(void);

    AP_HAL::OwnPtr<AP_HAL::Device> dev_accel;                ///< SPI device pointer for accelerometer communication
    AP_HAL::Device::PeriodicHandle accel_periodic_handle;   ///< Handle for accelerometer periodic callback registration
    AP_HAL::OwnPtr<AP_HAL::Device> dev_gyro;                ///< SPI device pointer for gyroscope communication
    AP_HAL::Device::PeriodicHandle gyro_periodic_handle;    ///< Handle for gyroscope periodic callback registration

    enum Rotation rotation;                                  ///< Board orientation rotation matrix to apply to sensor data
    uint8_t temperature_counter;                             ///< Counter for temperature sensor reading decimation
    enum DevTypes _accel_devtype;                            ///< Device type identifier for accelerometer (DEVTYPE_BMI088)
    float accel_range;                                       ///< Current accelerometer range in g (3, 6, 12, or 24)

    bool done_accel_config;                                  ///< Flag indicating accelerometer configuration completed successfully
    uint32_t accel_config_count;                             ///< Counter for accelerometer configuration attempts (for retry logic)
};
