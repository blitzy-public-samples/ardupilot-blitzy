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
 * @file AP_InertialSensor_BMI055.h
 * @brief Bosch BMI055 6-axis IMU driver for ArduPilot
 * 
 * @details Driver implementation for the Bosch BMI055 inertial measurement unit,
 *          which uniquely features separate accelerometer and gyroscope chips
 *          requiring independent SPI chip selects and device management.
 *          
 *          Hardware Architecture:
 *          - BMI055_ACC: 3-axis accelerometer on separate SPI device
 *          - BMI055_GYR: 3-axis gyroscope on separate SPI device
 *          - Each sensor requires dedicated SPI device pointer and chip select
 *          
 *          Key Features:
 *          - FIFO streaming mode for efficient high-rate data collection
 *          - Configurable output data rates and digital filtering
 *          - Built-in temperature compensation for improved accuracy
 *          - Gyro range: ±2000 deg/s, Accel range: ±16g
 * 
 * @note The BMI055 is unusual among IMU sensors in requiring two separate
 *       SPI chip selects (one for accel, one for gyro), necessitating dual
 *       SPIDevice pointers in the driver implementation.
 * 
 * @see AP_InertialSensor_Backend
 * @see libraries/AP_InertialSensor/AP_InertialSensor_Backend.h
 */
#pragma once

#include <AP_HAL/AP_HAL.h>

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"

/**
 * @class AP_InertialSensor_BMI055
 * @brief Backend driver for Bosch BMI055 IMU with dual SPI device management
 * 
 * @details Implements AP_InertialSensor_Backend interface for the BMI055 6-axis IMU.
 *          This driver handles the unique dual-chip architecture where accelerometer
 *          and gyroscope are separate physical devices requiring independent SPI
 *          communication channels.
 *          
 *          Driver Lifecycle:
 *          1. probe() - Static factory method detects both accel and gyro via WHO_AM_I
 *          2. Constructor - Initializes device pointers and rotation matrix
 *          3. init() - Configures both sensors (range, ODR, filters, FIFO)
 *          4. start() - Registers periodic callbacks for background sampling
 *          5. update() - Processes accumulated FIFO data and publishes samples
 *          
 *          Data Acquisition:
 *          - Periodic callbacks (read_fifo_accel/read_fifo_gyro) run at sensor ODR
 *          - FIFO buffering reduces interrupt overhead and CPU load
 *          - Data rotated to vehicle frame using configured rotation matrix
 *          - Temperature compensation applied for calibration stability
 *          
 *          Hardware Configuration:
 *          - Accelerometer: ±16g range, up to 1kHz ODR, 32-sample FIFO
 *          - Gyroscope: ±2000 deg/s range, up to 2kHz ODR, 100-sample FIFO
 *          - Both sensors include configurable low-pass filters for noise reduction
 *          - Independent power management and reset capabilities
 *          
 *          Thread Safety:
 *          - FIFO read callbacks execute in scheduler context
 *          - update() called from main thread publishes samples with backend lock held
 *          - No shared state between accel and gyro paths (independent devices)
 * 
 * @note Initialization fails if either accelerometer or gyroscope is not detected,
 *       as both are required for a functional 6-axis IMU.
 * @note FIFO overflow handling uses counter wraparound detection to prevent data loss.
 * @warning Temperature sensor reads from accelerometer chip only; gyro temp not monitored.
 * @warning Ensure both SPI chip selects are correctly mapped in board hwdef file.
 * 
 * @see AP_InertialSensor_Backend for base class interface
 * @see libraries/AP_HAL/Device.h for SPI device abstraction
 */
class AP_InertialSensor_BMI055 : public AP_InertialSensor_Backend {
public:
    /**
     * @brief Static factory method to detect and initialize BMI055 IMU
     * 
     * @details Probes both accelerometer and gyroscope devices to verify hardware
     *          presence via WHO_AM_I register reads. If both sensors respond with
     *          correct identification codes, allocates and initializes a BMI055
     *          driver instance. Returns nullptr if either device is not detected
     *          or fails initialization.
     *          
     *          Detection Sequence:
     *          1. Read accelerometer WHO_AM_I register (expected: 0xFA)
     *          2. Read gyroscope WHO_AM_I register (expected: 0x0F)
     *          3. If both match, construct driver instance
     *          4. Call init() to configure sensor registers
     *          5. Return driver pointer on success, nullptr on failure
     *          
     *          This method is called during sensor auto-detection at boot or when
     *          explicitly configured in board hwdef files.
     * 
     * @param[in,out] imu Reference to AP_InertialSensor instance for sensor registration
     * @param[in] dev_accel SPI device pointer for accelerometer (ownership transferred)
     * @param[in] dev_gyro SPI device pointer for gyroscope (ownership transferred)
     * @param[in] rotation Rotation matrix to transform sensor frame to vehicle frame
     * 
     * @return Pointer to initialized AP_InertialSensor_BMI055 backend on success,
     *         nullptr if detection fails or either sensor not present
     * 
     * @note Both dev_accel and dev_gyro must be valid and respond to WHO_AM_I queries
     * @note Ownership of device pointers transfers to the backend on successful probe
     * @note Called during boot from AP_InertialSensor::_detect_backends()
     * 
     * @see AP_InertialSensor_Backend::probe() pattern
     * @see libraries/AP_HAL/Device.h for SPIDevice interface
     */
    static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu,
                                            AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev_accel,
                                            AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev_gyro,
                                            enum Rotation rotation);

    /**
     * @brief Configure sensors and start periodic sampling callbacks
     * 
     * @details Starts the BMI055 data acquisition by registering periodic callbacks
     *          with the HAL scheduler for background FIFO reading. Called once after
     *          successful initialization to begin continuous sensor sampling.
     *          
     *          Startup Sequence:
     *          1. Register read_fifo_accel() callback at accelerometer ODR (typically 1kHz)
     *          2. Register read_fifo_gyro() callback at gyroscope ODR (typically 2kHz)
     *          3. Callbacks execute in scheduler context, reading FIFO data
     *          4. update() method (main thread) processes buffered samples
     *          
     *          The periodic callbacks run independently for accel and gyro, allowing
     *          different sample rates. FIFO buffering ensures no samples are lost
     *          between update() calls even at high data rates.
     * 
     * @return void
     * 
     * @note Called automatically by AP_InertialSensor after successful probe/init
     * @note Callbacks run in scheduler interrupt context - must be fast and non-blocking
     * @note This method implements the AP_InertialSensor_Backend::start() interface
     * 
     * @see update() for sample processing
     * @see AP_HAL::Device::register_periodic_callback()
     */
    void start() override;
    
    /**
     * @brief Process accumulated FIFO samples and publish to AP_InertialSensor
     * 
     * @details Called periodically from main loop to process sensor data accumulated
     *          in FIFOs by background callbacks. Publishes samples to the inertial
     *          sensor frontend for fusion and filtering. This method executes in
     *          the main thread context, not interrupt context.
     *          
     *          Processing Steps:
     *          1. Lock backend to prevent concurrent access
     *          2. Process all accumulated accelerometer samples from FIFO reads
     *          3. Process all accumulated gyroscope samples from FIFO reads
     *          4. Apply rotation matrix transformation to vehicle frame
     *          5. Publish samples to AP_InertialSensor frontend
     *          6. Update temperature reading periodically
     *          
     *          The background callbacks (read_fifo_accel/read_fifo_gyro) accumulate
     *          samples between update() calls. This decouples high-rate sensor
     *          sampling from main loop timing.
     * 
     * @return true if new samples were processed and published, false otherwise
     * 
     * @note Called at main loop rate (typically 400Hz for copter, varies by vehicle)
     * @note Must complete quickly to avoid delaying main loop
     * @note Temperature updates occur every ~N samples to reduce overhead
     * 
     * @see start() for callback registration
     * @see AP_InertialSensor_Backend::_publish_accel()
     * @see AP_InertialSensor_Backend::_publish_gyro()
     */
    bool update() override;

private:
    /**
     * @brief Private constructor for BMI055 backend driver
     * 
     * @details Constructs BMI055 driver instance with dual SPI device pointers.
     *          Constructor is private - instances created only via probe() factory
     *          method after successful hardware detection.
     *          
     *          Initialization:
     *          - Stores device pointers for accelerometer and gyroscope
     *          - Stores rotation matrix for sensor-to-vehicle frame transformation
     *          - Registers instance with AP_InertialSensor frontend
     *          - Initializes temperature counter for periodic temp updates
     *          
     *          Device pointers must remain valid for driver lifetime as they are
     *          used by periodic callbacks and update() method.
     * 
     * @param[in,out] imu Reference to AP_InertialSensor for backend registration
     * @param[in] dev_accel SPI device pointer for accelerometer (ownership transferred)
     * @param[in] dev_gyro SPI device pointer for gyroscope (ownership transferred)
     * @param[in] rotation Rotation matrix (enum) to transform sensor frame to vehicle frame
     * 
     * @note Constructor does not perform hardware initialization - call init() separately
     * @note Only called from probe() static factory method
     * @note Device pointers must be valid and detected via WHO_AM_I before construction
     * 
     * @see probe() for public factory method
     * @see init() for hardware configuration
     */
    AP_InertialSensor_BMI055(AP_InertialSensor &imu,
                             AP_HAL::OwnPtr<AP_HAL::Device> dev_accel,
                             AP_HAL::OwnPtr<AP_HAL::Device> dev_gyro,
                             enum Rotation rotation);

    /**
     * @brief Initialize accelerometer hardware and configure registers
     * 
     * @details Configures BMI055 accelerometer chip via SPI register writes.
     *          Sets output data rate, measurement range, digital filtering,
     *          and FIFO operation mode for optimal data acquisition.
     *          
     *          Configuration Steps:
     *          1. Perform soft reset to ensure clean state
     *          2. Configure measurement range (±16g for ArduPilot)
     *          3. Set output data rate (typically 1kHz)
     *          4. Configure digital low-pass filter bandwidth
     *          5. Enable FIFO buffering if supported
     *          6. Set FIFO watermark for interrupt/polling
     *          7. Verify configuration by reading back registers
     * 
     * @return true if accelerometer initialization successful, false on failure
     * 
     * @note Called during init() sequence after WHO_AM_I verification
     * @note Failure typically indicates SPI communication error or hardware fault
     * @warning Must complete before start() registers periodic callbacks
     * 
     * @see gyro_init() for gyroscope configuration
     * @see init() for overall initialization sequence
     */
    bool accel_init();
    
    /**
     * @brief Initialize gyroscope hardware and configure registers
     * 
     * @details Configures BMI055 gyroscope chip via SPI register writes.
     *          Sets output data rate, measurement range, digital filtering,
     *          and FIFO operation mode for optimal data acquisition.
     *          
     *          Configuration Steps:
     *          1. Perform soft reset to ensure clean state
     *          2. Configure measurement range (±2000 deg/s for ArduPilot)
     *          3. Set output data rate (typically 2kHz)
     *          4. Configure digital low-pass filter bandwidth
     *          5. Enable FIFO buffering if supported
     *          6. Set FIFO watermark for interrupt/polling
     *          7. Verify configuration by reading back registers
     * 
     * @return true if gyroscope initialization successful, false on failure
     * 
     * @note Called during init() sequence after WHO_AM_I verification
     * @note Failure typically indicates SPI communication error or hardware fault
     * @note Gyroscope FIFO depth (100 samples) larger than accelerometer (32 samples)
     * @warning Must complete before start() registers periodic callbacks
     * 
     * @see accel_init() for accelerometer configuration
     * @see init() for overall initialization sequence
     */
    bool gyro_init();

    /**
     * @brief Initialize complete BMI055 driver and configure both sensors
     * 
     * @details Master initialization method that orchestrates configuration of both
     *          accelerometer and gyroscope hardware. Called from probe() after
     *          successful WHO_AM_I detection. Failure at this stage prevents the
     *          backend from being used.
     *          
     *          Initialization Sequence:
     *          1. Call accel_init() to configure accelerometer registers
     *          2. Call gyro_init() to configure gyroscope registers
     *          3. Register sensor instances with AP_InertialSensor frontend
     *          4. Configure backend scaling factors for raw-to-SI conversion
     *          5. Initialize internal state (temperature counter, FIFO pointers)
     *          6. Verify both sensors responding correctly
     *          
     *          On failure, the backend is not registered and probe() returns nullptr,
     *          allowing AP_InertialSensor to try other IMU drivers.
     * 
     * @return true if both sensors initialized successfully, false if either fails
     * 
     * @note Called only once during probe() sequence
     * @note Both accel_init() and gyro_init() must succeed for overall success
     * @note Failure reasons logged to AP_Logger for troubleshooting
     * @warning Do not call start() if init() returns false
     * 
     * @see probe() for detection and construction
     * @see accel_init() for accelerometer configuration
     * @see gyro_init() for gyroscope configuration
     * @see start() for beginning data acquisition
     */
    bool init();

    /**
     * @brief Read and process accelerometer FIFO data
     * 
     * @details Periodic callback that reads accumulated accelerometer samples from
     *          the BMI055 accelerometer FIFO buffer. Executes in scheduler interrupt
     *          context at configured sample rate (typically 1kHz).
     *          
     *          FIFO Processing:
     *          1. Read FIFO status register to determine sample count
     *          2. Handle FIFO overflow condition if detected (counter wraparound)
     *          3. Burst-read all available samples (up to 32 samples)
     *          4. Parse 6-byte accelerometer data frames (X, Y, Z axes)
     *          5. Convert raw ADC values to m/s² using range scaling
     *          6. Accumulate samples for update() to publish
     *          7. Update FIFO read pointer for next callback
     *          
     *          FIFO format: Each sample is 6 bytes (2 bytes per axis, LSB first).
     *          Overflow detection uses frame counter wraparound to identify lost data.
     * 
     * @return void
     * 
     * @note Registered as periodic callback by start() method
     * @note Executes in scheduler context - must be fast (<100µs typical)
     * @note FIFO depth: 32 samples, overflow if not read before buffer full
     * @warning Must not block or call slow operations (no logging, no delays)
     * @warning Overflow discards oldest samples - tune callback rate to prevent
     * 
     * @see read_fifo_gyro() for gyroscope equivalent
     * @see update() for sample publishing
     * @see AP_HAL::Device::register_periodic_callback()
     */
    void read_fifo_accel();
    
    /**
     * @brief Read and process gyroscope FIFO data
     * 
     * @details Periodic callback that reads accumulated gyroscope samples from
     *          the BMI055 gyroscope FIFO buffer. Executes in scheduler interrupt
     *          context at configured sample rate (typically 2kHz).
     *          
     *          FIFO Processing:
     *          1. Read FIFO status register to determine sample count
     *          2. Handle FIFO overflow condition if detected (counter wraparound)
     *          3. Burst-read all available samples (up to 100 samples)
     *          4. Parse 6-byte gyroscope data frames (X, Y, Z angular rates)
     *          5. Convert raw ADC values to rad/s using range scaling
     *          6. Accumulate samples for update() to publish
     *          7. Update FIFO read pointer for next callback
     *          
     *          FIFO format: Each sample is 6 bytes (2 bytes per axis, LSB first).
     *          Gyroscope FIFO is deeper (100 vs 32 samples) to handle higher ODR.
     * 
     * @return void
     * 
     * @note Registered as periodic callback by start() method
     * @note Executes in scheduler context - must be fast (<100µs typical)
     * @note FIFO depth: 100 samples, larger than accel to handle higher data rate
     * @note Gyro typically sampled at 2kHz (2x accel rate) for better noise performance
     * @warning Must not block or call slow operations (no logging, no delays)
     * @warning Overflow discards oldest samples - tune callback rate to prevent
     * 
     * @see read_fifo_accel() for accelerometer equivalent
     * @see update() for sample publishing
     * @see AP_HAL::Device::register_periodic_callback()
     */
    void read_fifo_gyro();

    /**
     * @brief SPI device pointer for BMI055 accelerometer chip
     * 
     * @details Owns and manages SPI communication with the BMI055_ACC device.
     *          Used by accel_init(), read_fifo_accel(), and temperature reads.
     *          Lifetime managed by OwnPtr - automatically cleaned up on destruction.
     * 
     * @note Requires separate chip select from gyroscope device
     * @note Must be valid throughout driver lifetime for periodic callbacks
     * @see dev_gyro for gyroscope device pointer
     */
    AP_HAL::OwnPtr<AP_HAL::Device> dev_accel;
    
    /**
     * @brief SPI device pointer for BMI055 gyroscope chip
     * 
     * @details Owns and manages SPI communication with the BMI055_GYR device.
     *          Used by gyro_init() and read_fifo_gyro().
     *          Lifetime managed by OwnPtr - automatically cleaned up on destruction.
     * 
     * @note Requires separate chip select from accelerometer device
     * @note Must be valid throughout driver lifetime for periodic callbacks
     * @see dev_accel for accelerometer device pointer
     */
    AP_HAL::OwnPtr<AP_HAL::Device> dev_gyro;

    /**
     * @brief Rotation matrix to transform sensor frame to vehicle frame
     * 
     * @details Enum value representing the 3D rotation needed to align the sensor's
     *          physical orientation with the vehicle's reference frame (NED).
     *          Applied to all accelerometer and gyroscope samples before publishing.
     *          
     *          Common values: ROTATION_NONE, ROTATION_YAW_90, ROTATION_ROLL_180, etc.
     *          Configured via board hwdef or runtime parameter.
     * 
     * @note Set once at construction, never modified during operation
     * @note Applied by backend publish functions, not in FIFO read callbacks
     * @see enum Rotation in AP_Compass/Compass.h
     */
    enum Rotation rotation;
    
    /**
     * @brief Counter for throttling temperature sensor reads
     * 
     * @details Temperature reading is relatively slow compared to accel/gyro sampling,
     *          so it's only updated every N samples to reduce overhead. This counter
     *          tracks update cycles to determine when to read temperature.
     *          
     *          Temperature is read from accelerometer chip only (gyro temp not used).
     *          Typical update rate: every 100 samples (~10Hz at 1kHz accel rate).
     * 
     * @note Incremented in update() method, wraps at configured threshold
     * @note Temperature compensation applied per-axis for improved accuracy
     * @see update() for temperature read logic
     */
    uint8_t temperature_counter;
};
