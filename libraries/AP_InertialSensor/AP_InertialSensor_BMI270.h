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
 * @file AP_InertialSensor_BMI270.h
 * @brief Bosch BMI270 advanced 6-axis IMU driver with AI features
 * 
 * @details Implements backend driver for Bosch BMI270, a smart inertial measurement
 *          unit featuring integrated motion processing and AI-assisted sensor fusion
 *          capabilities. The BMI270 includes an on-chip motion coprocessor that
 *          enables advanced features like gesture recognition and activity classification.
 * 
 * Key Features:
 * - 6-axis IMU (3-axis gyroscope + 3-axis accelerometer)
 * - Integrated motion coprocessor for intelligent feature detection
 * - Advanced FIFO with hardware filtering
 * - SPI and I2C interface support
 * - Configurable output data rates and measurement ranges
 * - On-chip temperature sensor
 * 
 * Hardware Characteristics:
 * - Gyroscope range: ±125, ±250, ±500, ±1000, ±2000 deg/s
 * - Accelerometer range: ±2g, ±4g, ±8g, ±16g
 * - 16-bit ADC resolution
 * - 2KB FIFO buffer
 * - Configuration file (microcode) storage: 8KB internal RAM
 * 
 * @note BMI270 requires configuration file upload during initialization
 * @see AP_InertialSensor_Backend for base class interface
 * 
 * Source: libraries/AP_InertialSensor/AP_InertialSensor_BMI270.h
 */

#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/SPIDevice.h>
#include <AP_HAL/I2CDevice.h>

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"

#ifndef BMI270_DEFAULT_ROTATION
#define BMI270_DEFAULT_ROTATION ROTATION_NONE
#endif

/**
 * @class AP_InertialSensor_BMI270
 * @brief Backend driver for Bosch BMI270 smart IMU with integrated motion processing
 * 
 * @details Advanced IMU driver implementing support for the Bosch BMI270 featuring
 *          on-chip intelligence and motion coprocessor capabilities:
 *          
 *          Hardware Capabilities:
 *          - Integrated motion coprocessor for feature detection
 *          - Advanced FIFO buffering with intelligent filtering
 *          - Gyroscope range: ±2000 deg/s (typical configuration)
 *          - Accelerometer range: ±16g (typical configuration)
 *          - On-chip gesture recognition and activity classification
 *          - Hardware-based motion detection and step counting
 *          
 *          Critical Initialization Sequence:
 *          1. Soft reset and wait for device ready status
 *          2. Upload configuration file (microcode) to internal RAM via burst writes
 *          3. Verify configuration CRC for data integrity
 *          4. Configure output data rate (ODR), measurement ranges, and filters
 *          5. Enable FIFO buffering and configure interrupts
 *          6. Register periodic callback for data acquisition
 *          
 *          Configuration File Handling:
 *          - 8KB configuration blob containing motion algorithms and filter coefficients
 *          - Loaded via sequential burst writes during initialization
 *          - Must match chip hardware version (BMI270 vs BMI270_LEGACY variants)
 *          - CRC validation ensures correct upload and prevents sensor malfunction
 *          - Contains proprietary Bosch algorithms for motion feature extraction
 *          
 *          Data Acquisition:
 *          - FIFO-based buffered reading reduces CPU overhead
 *          - Hardware timestamps for precise sensor fusion
 *          - Interleaved gyro and accel data frames in FIFO
 *          - Automatic FIFO watermark and overflow management
 *          - Temperature data acquired periodically for thermal compensation
 *          
 *          Thread Safety:
 *          - Device semaphore must be held during register access
 *          - Periodic callback runs in scheduler context at configured rate
 *          - FIFO reads are atomic operations protected by device semaphore
 * 
 * @note Config file upload takes approximately 50ms - must be performed during
 *       initialization only, not in time-critical paths
 * @note Different configuration files exist for BMI270 and BMI270_LEGACY chip variants
 * @note FIFO overflow handling: driver resets FIFO and reports data gap to EKF
 * 
 * @warning Incorrect configuration file causes complete sensor malfunction - always
 *          verify chip ID before config upload to ensure correct variant matching
 * @warning Do not perform config file operations during flight - initialization only
 * @warning FIFO reads must complete within periodic callback deadline to prevent overflow
 * 
 * @see AP_InertialSensor_Backend for base class interface and lifecycle
 * @see AP_InertialSensor for sensor manager integration
 */
class AP_InertialSensor_BMI270 : public AP_InertialSensor_Backend {
public:
    /**
     * @brief Probe for BMI270 sensor on SPI bus and create driver instance if detected
     * 
     * @details Attempts to detect and initialize a BMI270 IMU connected via SPI interface.
     *          Detection sequence:
     *          1. Reads chip ID register to verify BMI270 presence
     *          2. Performs soft reset if chip ID matches
     *          3. Uploads configuration file to internal RAM
     *          4. Verifies configuration integrity via CRC check
     *          5. Creates and returns driver instance on success
     *          
     *          This is a static factory method called during hardware detection phase
     *          at system startup. The method takes ownership of the SPIDevice pointer
     *          and either constructs a driver instance or releases the device on failure.
     * 
     * @param[in,out] imu         Reference to AP_InertialSensor manager for sensor registration
     * @param[in]     dev         Ownership pointer to SPI device interface (consumed on success)
     * @param[in]     rotation    Board-specific sensor orientation (default: ROTATION_NONE)
     * 
     * @return Pointer to initialized BMI270 backend driver instance on successful detection,
     *         nullptr if sensor not detected or initialization failed
     * 
     * @note This function performs blocking I/O and may take 50-100ms due to config upload
     * @note Called only during system initialization, not during normal operation
     * @note Device pointer is released automatically if probe fails
     * 
     * @see probe(AP_InertialSensor&, AP_HAL::OwnPtr<AP_HAL::I2CDevice>, Rotation) for I2C variant
     */
    static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu,
                                            AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev,
                                            enum Rotation rotation=BMI270_DEFAULT_ROTATION);

    /**
     * @brief Probe for BMI270 sensor on I2C bus and create driver instance if detected
     * 
     * @details Attempts to detect and initialize a BMI270 IMU connected via I2C interface.
     *          Detection and initialization sequence identical to SPI variant, but uses
     *          I2C protocol for all register access operations.
     *          
     *          I2C-specific considerations:
     *          - I2C address typically 0x68 or 0x69 (hardware configurable)
     *          - Clock stretching may occur during config file writes
     *          - Burst write operations used for efficient config upload
     *          - Bus speed should be configured for 400kHz (fast mode)
     * 
     * @param[in,out] imu         Reference to AP_InertialSensor manager for sensor registration
     * @param[in]     dev         Ownership pointer to I2C device interface (consumed on success)
     * @param[in]     rotation    Board-specific sensor orientation (default: ROTATION_NONE)
     * 
     * @return Pointer to initialized BMI270 backend driver instance on successful detection,
     *         nullptr if sensor not detected or initialization failed
     * 
     * @note This function performs blocking I/O and may take 50-100ms due to config upload
     * @note I2C bus contention during probe can delay other sensor initialization
     * @note Device pointer is released automatically if probe fails
     * 
     * @see probe(AP_InertialSensor&, AP_HAL::OwnPtr<AP_HAL::SPIDevice>, Rotation) for SPI variant
     */
    static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu,
                                            AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev,
                                            enum Rotation rotation=BMI270_DEFAULT_ROTATION);

    /**
     * @brief Configure the sensors and start periodic data acquisition
     * 
     * @details Completes sensor initialization and begins continuous data collection:
     *          1. Configures accelerometer range, ODR, and filtering
     *          2. Configures gyroscope range, ODR, and filtering  
     *          3. Enables and configures FIFO buffering
     *          4. Registers periodic callback for data reading (typically 1kHz)
     *          5. Enables sensor data ready interrupts if supported
     *          
     *          This method is called by the AP_InertialSensor manager after successful
     *          probe to transition the sensor from initialized to active state.
     *          
     *          Typical configuration:
     *          - Gyro: ±2000 deg/s, 1600Hz ODR
     *          - Accel: ±16g, 1600Hz ODR
     *          - FIFO: Enabled for both gyro and accel data
     *          - Callback rate: 1000Hz (main loop rate)
     * 
     * @note Called once during system startup after successful probe
     * @note Must not be called multiple times - behavior undefined
     * @note Periodic callback begins immediately after this method returns
     * 
     * @warning Failure in this method prevents sensor from providing data to EKF
     * 
     * @see AP_InertialSensor_Backend::start() for base class interface
     */
    void start() override;

    /**
     * @brief Update backend driver state and publish sensor data to frontend
     * 
     * @details Called by AP_InertialSensor at main loop rate to process sensor data
     *          accumulated by periodic callback. This method:
     *          - Checks for new data availability from periodic reads
     *          - Applies sensor rotation transformation based on board orientation
     *          - Publishes gyro and accel samples to frontend for EKF consumption
     *          - Updates sensor health monitoring and error statistics
     *          - Reports FIFO overflow conditions if detected
     *          
     *          Data flow:
     *          Periodic callback (1kHz) → FIFO read → Sample accumulation →
     *          update() call → Rotation applied → Frontend publication → EKF
     *          
     *          The actual sensor reading occurs in the periodic callback (read_fifo),
     *          while this method handles data processing and publishing.
     * 
     * @return true if new data was processed and published, false if no new data available
     * 
     * @note Called at vehicle main loop rate (typically 400Hz for copter, 50Hz for plane)
     * @note Thread-safe with respect to periodic callback via internal buffering
     * @note Return value indicates data freshness for sensor health monitoring
     * 
     * @see read_fifo() for actual sensor data acquisition in periodic callback
     * @see AP_InertialSensor_Backend::update() for base class interface
     */
    bool update() override;

private:
    /**
     * @brief Private constructor for BMI270 driver instance
     * 
     * @details Constructs driver instance with specified hardware interface and configuration.
     *          Constructor is private to enforce creation through probe() factory methods.
     *          Stores device interface, rotation matrix, and initializes internal state.
     *          
     *          The device pointer ownership is transferred to this object and will be
     *          automatically released when the driver instance is destroyed.
     * 
     * @param[in,out] imu         Reference to AP_InertialSensor manager
     * @param[in]     dev         Ownership pointer to SPI or I2C device interface
     * @param[in]     rotation    Board-specific sensor orientation matrix
     * 
     * @note Only called from probe() methods after successful sensor detection
     * @see probe() for public factory interface
     */
    AP_InertialSensor_BMI270(AP_InertialSensor &imu,
                             AP_HAL::OwnPtr<AP_HAL::Device> dev,
                             enum Rotation rotation);

    /**
     * @brief Read multiple consecutive registers from BMI270
     * 
     * @details Performs burst read operation to retrieve multiple register values.
     *          Acquires device semaphore, executes read transaction, and releases semaphore.
     *          Works for both SPI and I2C interfaces via HAL abstraction.
     *          
     *          For SPI: Sends read command (register | 0x80) followed by dummy bytes
     *          For I2C: Sends register address followed by read of len bytes
     * 
     * @param[in]  reg   Starting register address to read from
     * @param[out] data  Buffer to store read data (must be at least len bytes)
     * @param[in]  len   Number of consecutive registers to read
     * 
     * @return true if read operation completed successfully, false on communication failure
     * 
     * @note Device semaphore is automatically managed by this function
     * @note Used for reading sensor data, FIFO contents, and status registers
     */
    bool read_registers(uint8_t reg, uint8_t *data, uint8_t len);
    
    /**
     * @brief Write single register value to BMI270
     * 
     * @details Performs single register write operation.
     *          Acquires device semaphore, executes write transaction, and releases semaphore.
     *          Works for both SPI and I2C interfaces via HAL abstraction.
     *          
     *          For SPI: Sends write command (register & 0x7F) followed by value byte
     *          For I2C: Sends register address followed by value byte
     * 
     * @param[in] reg  Register address to write to
     * @param[in] v    8-bit value to write to register
     * 
     * @return true if write operation completed successfully, false on communication failure
     * 
     * @note Device semaphore is automatically managed by this function
     * @note Used for configuration, control commands, and FIFO management
     */
    bool write_register(uint8_t reg, uint8_t v);

    /**
     * @brief Check BMI270 error register for hardware fault conditions (debug only)
     * 
     * @details If BMI270_DEBUG macro is defined at compile time, reads the error register
     *          and triggers assertion failure if any error flags are set. This helps catch
     *          hardware communication issues, configuration errors, or sensor malfunctions
     *          during development and testing.
     *          
     *          Error conditions detected:
     *          - Fatal errors (sensor initialization failure)
     *          - Configuration errors (invalid parameter combinations)
     *          - Communication errors (SPI/I2C transaction failures)
     *          
     *          If BMI270_DEBUG is not defined, this function compiles to empty (no-op)
     *          to avoid runtime overhead in production builds.
     * 
     * @note Implementation is conditionally compiled - empty in production builds
     * @note Should be called after configuration changes during debugging
     * @warning Triggers assertion failure (system halt) if errors detected
     */
    void check_err_reg();

    /**
     * @brief Perform hardware-level initialization of BMI270 device
     *
     * @details Executes the critical configuration file upload sequence required for
     *          BMI270 operation. This is the most timing-sensitive part of initialization:
     *          
     *          Initialization sequence:
     *          1. Issue soft reset command and wait for completion (2ms delay)
     *          2. Wait for device ready status (init_status register polling)
     *          3. Disable advanced power save mode to enable register access
     *          4. Upload 8KB configuration file to internal RAM via burst writes
     *          5. Enable configuration and wait for completion
     *          6. Verify configuration CRC matches expected value
     *          7. Re-enable advanced power save if desired
     *          
     *          The configuration file contains proprietary Bosch algorithms including:
     *          - Motion detection coefficients
     *          - Feature extraction parameters  
     *          - Filter coefficients for various modes
     *          - Activity recognition state machines
     *          
     *          Different config files exist for BMI270 standard and legacy variants,
     *          stored in maximum_fifo_config_file[] array.
     *
     * @return true on successful configuration file upload and verification,
     *         false if device not ready, upload failed, or CRC check failed
     * 
     * @note Caller must hold device semaphore before calling
     * @note Takes approximately 50ms to complete due to config upload
     * @note Must be called before any sensor configuration
     * 
     * @warning Critical function - sensor non-functional if this fails
     * @warning Must use correct config file variant for chip version
     * 
     * @see maximum_fifo_config_file for configuration data
     */
    bool hardware_init();

    /**
     * @brief Initialize BMI270 driver and register with sensor manager
     *
     * @details Top-level initialization method that orchestrates the complete driver
     *          initialization sequence:
     *          
     *          1. Perform hardware initialization (config file upload)
     *          2. Verify chip ID matches expected BMI270 identifier
     *          3. Allocate instance in AP_InertialSensor manager
     *          4. Configure sensor ranges and filters
     *          5. Perform initial sensor self-test if supported
     *          6. Set up FIFO buffering
     *          
     *          This method is called from probe() after basic device detection succeeds.
     *          If initialization fails at any step, the driver instance will not be
     *          registered with the sensor manager.
     *
     * @return true if all initialization steps completed successfully,
     *         false if any initialization step failed
     * 
     * @note Called once during probe sequence
     * @note Failure here causes probe() to return nullptr
     * @note Device semaphore handling is internal to this method
     * 
     * @see hardware_init() for config file upload details
     * @see probe() for caller context
     */
    bool init();

    /**
     * @brief Configure accelerometer measurement range, ODR, and filtering
     *
     * @details Configures accelerometer subsystem for optimal performance:
     *          - Sets measurement range (typically ±16g for vehicle dynamics)
     *          - Configures output data rate (ODR) to match gyro rate (1600Hz typical)
     *          - Enables bandwidth filtering to reduce noise
     *          - Configures power mode (normal vs low power)
     *          - Sets up oversampling if supported
     *          
     *          Configuration written to ACC_CONF and ACC_RANGE registers.
     *          Settings optimized for vehicle motion with high dynamic range
     *          and sufficient bandwidth for vibration isolation.
     *
     * @note Caller must hold device semaphore before calling
     * @note Must be called after hardware_init() completes successfully
     * @note Configuration persists until device reset
     * 
     * @see configure_gyro() for gyroscope configuration
     */
    void configure_accel();

    /**
     * @brief Configure gyroscope measurement range, ODR, and filtering
     *
     * @details Configures gyroscope subsystem for optimal performance:
     *          - Sets measurement range (typically ±2000 deg/s for vehicle rates)
     *          - Configures output data rate (ODR) to 1600Hz or 3200Hz
     *          - Enables bandwidth filtering matched to control loop bandwidth
     *          - Configures power mode (normal vs low power)
     *          - Sets up noise filtering and averaging
     *          
     *          Configuration written to GYR_CONF and GYR_RANGE registers.
     *          Settings optimized for attitude control with sufficient bandwidth
     *          for fastest expected vehicle rotation rates.
     *
     * @note Caller must hold device semaphore before calling
     * @note Must be called after hardware_init() completes successfully
     * @note Gyro and accel ODR should match for synchronized sampling
     * 
     * @see configure_accel() for accelerometer configuration
     */
    void configure_gyro();

    /**
     * @brief Reset FIFO buffer and clear all buffered data
     *
     * @details Issues FIFO reset command to clear all accumulated sensor data in hardware
     *          buffer. This is necessary:
     *          - During initialization before starting data acquisition
     *          - After FIFO overflow to recover to known state
     *          - When changing FIFO configuration
     *          - To discard stale data after long delays
     *          
     *          Reset is performed by writing to FIFO command register.
     *          Hardware clears FIFO read/write pointers and resets watermark status.
     * 
     * @note Must hold device semaphore before calling
     * @note Any data in FIFO is lost - cannot be recovered
     * @note FIFO configuration (enabled sensors) is preserved
     */
    void fifo_reset();

    /**
     * @brief Configure FIFO buffering mode and enabled data sources
     *
     * @details Configures hardware FIFO to buffer both gyro and accelerometer data:
     *          - Enables FIFO for gyro and accel data frames
     *          - Configures FIFO watermark threshold for interrupt generation
     *          - Sets FIFO overflow behavior (stop-at-full vs overwrite-old)
     *          - Enables FIFO header mode for frame type identification
     *          - Configures data frame format (gyro+accel interleaved)
     *          
     *          BMI270 FIFO structure:
     *          - 2KB hardware buffer
     *          - Frame-based: each frame contains header + data
     *          - Gyro frame: 1 byte header + 6 bytes data (X,Y,Z as int16)
     *          - Accel frame: 1 byte header + 6 bytes data (X,Y,Z as int16)
     *          - Frames interleaved: gyro, accel, gyro, accel, ...
     *          
     *          FIFO reduces CPU overhead by batching samples and reduces chance
     *          of data loss if periodic callback is occasionally delayed.
     * 
     * @note Must be called after configure_accel() and configure_gyro()
     * @note FIFO should be reset before configuration
     * @note Watermark interrupt optional but recommended for efficiency
     */
    void configure_fifo();

    /**
     * @brief Read and process all available samples from FIFO buffer
     *
     * @details Periodic callback function that reads FIFO contents and extracts sensor samples:
     *          1. Read FIFO length register to determine bytes available
     *          2. Perform burst read of FIFO data buffer
     *          3. Parse frame headers to identify gyro vs accel frames
     *          4. Extract and scale raw sensor data from frames
     *          5. Accumulate samples for publishing in update() method
     *          6. Detect and handle FIFO overflow condition
     *          7. Read temperature data periodically for thermal compensation
     *          
     *          Frame parsing:
     *          - Header byte identifies frame type (gyro=0x88, accel=0x84)
     *          - Data bytes are 16-bit two's complement integers
     *          - Byte order is little-endian (LSB first)
     *          - Scaling applied based on configured range
     *          
     *          FIFO overflow handling:
     *          - Detected via overflow flag in FIFO status register
     *          - Triggers FIFO reset to recover
     *          - Logs error for EKF data gap notification
     *          
     *          This function runs in high-priority scheduler callback context at
     *          approximately 1kHz rate (1ms period).
     * 
     * @note Called automatically by scheduler periodic callback
     * @note Must complete within 1ms deadline to prevent overflow
     * @note Device semaphore automatically held during callback
     * 
     * @warning Long execution delays can cause FIFO overflow and data loss
     * @warning Overflow causes EKF reset due to lost samples
     * 
     * @see parse_accel_frame() for accelerometer data extraction
     * @see parse_gyro_frame() for gyroscope data extraction
     * @see update() for data publishing to frontend
     */
    void read_fifo();
    
    /**
     * @brief Parse and accumulate accelerometer data frame from FIFO
     * 
     * @details Extracts 3-axis accelerometer sample from raw FIFO frame data:
     *          - Combines LSB and MSB bytes into 16-bit signed integers
     *          - Converts raw ADC values to m/s² using scale factor
     *          - Applies sensor rotation matrix for board orientation
     *          - Accumulates sample for batch publishing in update()
     *          
     *          Frame format (7 bytes):
     *          - Byte 0: Header (0x84 for accel frame)
     *          - Byte 1-2: X-axis LSB, MSB (little-endian int16)
     *          - Byte 3-4: Y-axis LSB, MSB (little-endian int16)
     *          - Byte 5-6: Z-axis LSB, MSB (little-endian int16)
     *          
     *          Scale factor depends on configured accelerometer range:
     *          - ±2g:  16384 LSB/g
     *          - ±4g:   8192 LSB/g
     *          - ±8g:   4096 LSB/g
     *          - ±16g:  2048 LSB/g
     * 
     * @param[in] d  Pointer to 7-byte FIFO frame (header + 6 data bytes)
     * 
     * @note Called from read_fifo() for each accelerometer frame
     * @note Pointer must point to valid 7-byte buffer
     * @note No return value - accumulates internally
     * 
     * @see parse_gyro_frame() for gyroscope frame parsing
     */
    void parse_accel_frame(const uint8_t* d);
    
    /**
     * @brief Parse and accumulate gyroscope data frame from FIFO
     * 
     * @details Extracts 3-axis gyroscope sample from raw FIFO frame data:
     *          - Combines LSB and MSB bytes into 16-bit signed integers
     *          - Converts raw ADC values to rad/s using scale factor
     *          - Applies sensor rotation matrix for board orientation
     *          - Accumulates sample for batch publishing in update()
     *          
     *          Frame format (7 bytes):
     *          - Byte 0: Header (0x88 for gyro frame)
     *          - Byte 1-2: X-axis LSB, MSB (little-endian int16)
     *          - Byte 3-4: Y-axis LSB, MSB (little-endian int16)
     *          - Byte 5-6: Z-axis LSB, MSB (little-endian int16)
     *          
     *          Scale factor depends on configured gyroscope range:
     *          - ±125 deg/s:  262.4 LSB/(deg/s)
     *          - ±250 deg/s:  131.2 LSB/(deg/s)
     *          - ±500 deg/s:   65.6 LSB/(deg/s)
     *          - ±1000 deg/s:  32.8 LSB/(deg/s)
     *          - ±2000 deg/s:  16.4 LSB/(deg/s)
     * 
     * @param[in] d  Pointer to 7-byte FIFO frame (header + 6 data bytes)
     * 
     * @note Called from read_fifo() for each gyroscope frame
     * @note Pointer must point to valid 7-byte buffer
     * @note No return value - accumulates internally
     * 
     * @see parse_accel_frame() for accelerometer frame parsing
     */
    void parse_gyro_frame(const uint8_t* d);

    /**
     * @brief Ownership pointer to hardware device interface (SPI or I2C)
     * 
     * @details HAL abstraction providing register read/write operations and bus management.
     *          Supports both SPI and I2C BMI270 variants through polymorphic Device interface.
     *          Ownership is transferred from probe() and automatically released on destruction.
     *          Device semaphore is managed through this interface for thread-safe access.
     */
    AP_HAL::OwnPtr<AP_HAL::Device> _dev;
    
    /**
     * @brief Handle for registered periodic callback in scheduler
     * 
     * @details Identifies the periodic callback registered for read_fifo() execution.
     *          Callback runs at approximately 1kHz (1000 microsecond period) in high-priority
     *          scheduler context. Handle used to stop callback during shutdown or errors.
     *          Registered in start() method and runs until driver destruction.
     */
    AP_HAL::Device::PeriodicHandle periodic_handle;

    /**
     * @brief Sensor rotation matrix for board orientation correction
     * 
     * @details Rotation enumeration defining transformation from sensor frame to vehicle
     *          body frame. Applied to both gyro and accel samples to correct for physical
     *          sensor mounting orientation on flight controller board.
     *          
     *          Common rotations:
     *          - ROTATION_NONE: Sensor aligned with vehicle axes
     *          - ROTATION_YAW_90: Sensor rotated 90° clockwise
     *          - ROTATION_PITCH_180: Sensor mounted upside-down
     *          
     *          Set during probe() from board configuration or default value.
     */
    enum Rotation _rotation;

    /**
     * @brief Counter for periodic temperature sample acquisition
     * 
     * @details Incremented on each read_fifo() call to reduce temperature reading frequency.
     *          Temperature sampled every Nth FIFO read (typically every 100-200ms) since
     *          temperature changes slowly compared to IMU data rate.
     *          
     *          Temperature data used for:
     *          - Thermal drift compensation in EKF
     *          - Sensor health monitoring
     *          - Bias correction at high temperatures
     *          
     *          Counter wraps at configured threshold, triggering temperature register read.
     */
    uint8_t temperature_counter;

    /**
     * @brief BMI270 configuration file data (8KB microcode blob)
     * 
     * @details Static constant array containing proprietary Bosch configuration data loaded
     *          to BMI270 internal RAM during hardware_init(). Contains motion detection
     *          algorithms, filter coefficients, and feature extraction parameters.
     *          
     *          Configuration file characteristics:
     *          - Size: 8192 bytes (8KB)
     *          - Format: Binary blob uploaded via sequential burst writes
     *          - Content: Proprietary Bosch algorithms (not publicly documented)
     *          - Variants: Separate files for BMI270 standard vs legacy chips
     *          - CRC protected: Integrity verified after upload
     *          
     *          File enables advanced features:
     *          - Any-motion / no-motion detection
     *          - Step counter and detector
     *          - Activity recognition
     *          - Wrist gesture detection
     *          - Optimized FIFO filtering
     *          
     *          Without correct config file, sensor provides raw data only without
     *          intelligent processing features.
     * 
     * @note Array defined in corresponding .cpp implementation file
     * @note Must match chip variant (standard BMI270 vs BMI270_LEGACY)
     * 
     * @warning Using wrong config file causes sensor malfunction
     * @warning Do not modify - provided by Bosch Sensortec
     * 
     * @see hardware_init() for config file upload sequence
     */
    static const uint8_t maximum_fifo_config_file[];
};
