/**
 * @file AP_InertialSensor_RST.h
 * @brief Robsense RST IMU driver for dual-chip SPI configuration (Linux HAL only)
 * 
 * @details Implements backend driver for Robsense RST series IMUs using separate
 *          SPI devices for gyroscope (ST i3g4250d) and accelerometer (ST iis328dq).
 *          This driver is specific to Linux-based flight controllers where the RST
 *          IMU hardware interfaces via dual SPI buses.
 *          
 *          Hardware configuration:
 *          - Gyroscope: ST i3g4250d 3-axis MEMS gyroscope via SPI
 *          - Accelerometer: ST iis328dq 3-axis MEMS accelerometer via SPI
 *          - Dual independent SPI device communication
 *          - Configurable rotation matrices for sensor alignment
 *          
 *          This is a Linux HAL-specific driver used primarily on embedded Linux
 *          boards where the RST IMU is connected via SPI interface.
 * 
 * @note Linux HAL only (CONFIG_HAL_BOARD == HAL_BOARD_LINUX)
 * @note Requires two separate SPI device interfaces for gyro and accel
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX

#include <AP_HAL/I2CDevice.h>
#include <AP_HAL/SPIDevice.h>
#include <Filter/Filter.h>
#include <Filter/LowPassFilter2p.h>

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"

/**
 * @class AP_InertialSensor_RST
 * @brief Backend driver for Robsense RST dual-SPI IMU (Linux HAL)
 * 
 * @details This driver implements the AP_InertialSensor_Backend interface for
 *          Robsense RST series IMUs using a dual-chip configuration with separate
 *          SPI interfaces for gyroscope and accelerometer sensors.
 *          
 *          Architecture:
 *          - Gyroscope: ST i3g4250d (3-axis MEMS gyro, ±245/±500/±2000 dps)
 *          - Accelerometer: ST iis328dq (3-axis MEMS accel, ±2g/±4g/±8g)
 *          - Communication: Dual independent SPI buses
 *          - Data rate: Configurable based on sensor registers
 *          
 *          Driver lifecycle:
 *          1. probe() - Static factory method to detect and instantiate driver
 *          2. _init_sensor() - Initialize both gyro and accel via SPI
 *          3. start() - Register periodic callbacks for sensor sampling
 *          4. update() - Process accumulated samples and publish to frontend
 *          
 *          Sensor alignment:
 *          - Supports arbitrary rotation matrices for both sensors
 *          - Rotation applied to raw sensor data before publishing
 *          - Allows for flexible board mounting orientations
 *          
 *          Scaling:
 *          - Gyro scale converts raw ADC values to rad/s
 *          - Accel scale converts raw ADC values to m/s²
 *          - Scale factors determined from sensor range configuration
 *          
 *          Thread safety:
 *          - Sensor reads occur in HAL scheduler callbacks
 *          - SPI device ownership ensures exclusive access
 *          - update() called from main IMU update task
 * 
 * @note Platform-specific: Linux HAL only
 * @note Hardware-specific: Robsense RST IMU boards
 * @warning Requires correct SPI device paths configured in board definition
 * @warning Rotation matrices must match physical sensor orientation
 * 
 * @see AP_InertialSensor_Backend for base class interface
 * @see libraries/AP_InertialSensor/AP_InertialSensor.h for frontend API
 */
class AP_InertialSensor_RST : public AP_InertialSensor_Backend
{
public:
    /**
     * @brief Construct RST IMU backend with dual SPI devices
     * 
     * @details Constructor initializes the RST IMU driver with separate SPI device
     *          handles for gyroscope and accelerometer. Device ownership is transferred
     *          to this driver instance via move semantics (OwnPtr).
     *          
     *          The rotation parameters allow the driver to correct for physical sensor
     *          mounting orientation relative to the vehicle body frame. Rotations are
     *          applied to raw sensor data before publishing to the frontend.
     *          
     *          Initialization sequence:
     *          1. Store device pointers and rotation parameters
     *          2. Initialize scale factors to zero (set during _init_sensor)
     *          3. Register with frontend (handled by base class)
     * 
     * @param[in] imu Reference to AP_InertialSensor frontend manager
     * @param[in] dev_gyro OwnPtr to SPI device for i3g4250d gyroscope
     * @param[in] dev_accel OwnPtr to SPI device for iis328dq accelerometer
     * @param[in] rotation_a Rotation matrix for accelerometer alignment
     * @param[in] rotation_g Rotation matrix for gyroscope alignment
     * 
     * @note Constructor does not initialize hardware - call _init_sensor() via probe()
     * @note Ownership of SPI devices transferred to this object
     * 
     * @see probe() for typical construction pattern
     */
    AP_InertialSensor_RST(AP_InertialSensor &imu,
                              AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev_gyro,
                              AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev_accel,
                              enum Rotation rotation_a,
                              enum Rotation rotation_g);

    /**
     * @brief Destructor cleans up SPI device resources
     * 
     * @details Virtual destructor ensures proper cleanup of SPI device handles
     *          when driver instance is destroyed. OwnPtr automatically releases
     *          device resources.
     */
    virtual ~AP_InertialSensor_RST();

    /**
     * @brief Probe and initialize RST IMU on SPI buses
     * 
     * @details Static factory method that attempts to detect and initialize a
     *          Robsense RST IMU on the provided SPI device interfaces. This is
     *          the standard entry point for driver instantiation.
     *          
     *          Probe sequence:
     *          1. Create driver instance with provided parameters
     *          2. Call _init_sensor() to initialize hardware
     *          3. Return driver pointer on success, nullptr on failure
     *          
     *          Success criteria:
     *          - Both gyro and accel respond with valid WHO_AM_I registers
     *          - Device configuration registers accept writes
     *          - Initial sensor reads return valid data
     *          
     *          Failure cases:
     *          - SPI communication errors
     *          - Incorrect WHO_AM_I values (wrong sensor or no sensor)
     *          - Device configuration failures
     * 
     * @param[in] imu Reference to AP_InertialSensor frontend manager
     * @param[in] dev_gyro OwnPtr to SPI device for i3g4250d gyroscope
     * @param[in] dev_accel OwnPtr to SPI device for iis328dq accelerometer
     * @param[in] rotation_a Rotation matrix for accelerometer alignment
     * @param[in] rotation_g Rotation matrix for gyroscope alignment
     * 
     * @return Pointer to new driver instance on success, nullptr on failure
     * 
     * @note Called during sensor detection phase at system startup
     * @note Device ownership transferred on success, released on failure
     * @warning nullptr return indicates hardware not present or initialization failed
     * 
     * @see _init_sensor() for hardware initialization details
     */
    static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu,
                                              AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev_gyro,
                                              AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev_accel,
                                              enum Rotation rotation_a,
                                              enum Rotation rotation_g);

    /**
     * @brief Update frontend with accumulated sensor samples
     * 
     * @details Called by the InertialSensor frontend during its periodic update cycle
     *          to publish accumulated gyroscope and accelerometer samples. This method
     *          processes raw samples collected by the periodic callbacks, applies
     *          rotation corrections, and pushes data to the frontend for sensor fusion.
     *          
     *          Update workflow:
     *          1. Retrieve accumulated samples from internal buffers
     *          2. Apply rotation matrices (rotation_g for gyro, rotation_a for accel)
     *          3. Apply scale factors to convert to standard units
     *          4. Publish to frontend via _publish_gyro() and _publish_accel()
     *          5. Return status indicating data availability
     *          
     *          Units after scaling:
     *          - Gyroscope: rad/s (radians per second)
     *          - Accelerometer: m/s² (meters per second squared)
     *          
     *          Timing:
     *          - Called from main scheduler task
     *          - Typically runs at 50-400 Hz depending on vehicle configuration
     *          - Must complete quickly to avoid scheduler overruns
     * 
     * @return true if new data was published, false if no new samples available
     * 
     * @note Overrides AP_InertialSensor_Backend::update()
     * @note Thread safety: Called from main scheduler context
     * 
     * @see start() for periodic callback registration
     * @see gyro_measure() and accel_measure() for sample acquisition
     */
    bool update() override;

    /**
     * @brief Start periodic sensor sampling callbacks
     * 
     * @details Registers periodic timer callbacks with the HAL scheduler to sample
     *          gyroscope and accelerometer at their configured data rates. This method
     *          is called once during system initialization after successful probe.
     *          
     *          Callback registration:
     *          - Gyro callback: gyro_measure() at gyro sample rate
     *          - Accel callback: accel_measure() at accel sample rate
     *          - Callbacks run in timer interrupt context (HAL scheduler)
     *          
     *          Typical sample rates:
     *          - Gyroscope: 100-800 Hz depending on sensor configuration
     *          - Accelerometer: 50-400 Hz depending on sensor configuration
     *          
     *          After start():
     *          - Sensor data begins accumulating in internal buffers
     *          - update() must be called periodically to consume samples
     *          - Callbacks continue until driver destruction
     * 
     * @note Overrides AP_InertialSensor_Backend::start()
     * @note Called once during initialization, not repeatedly
     * @warning Must be called after successful _init_sensor()
     * @warning Callback frequency must not exceed scheduler capacity
     * 
     * @see gyro_measure() for gyro sampling callback
     * @see accel_measure() for accel sampling callback
     * @see update() for sample consumption
     */
    void start(void) override;

private:
    /**
     * @brief Initialize both gyro and accel sensors via SPI
     * 
     * @details Master initialization routine that configures both IMU sensors.
     *          Called during probe() to set up hardware for operation.
     *          
     *          Initialization sequence:
     *          1. Call _init_gyro() to configure i3g4250d gyroscope
     *          2. Call _init_accel() to configure iis328dq accelerometer
     *          3. Calculate and store scale factors for unit conversion
     *          4. Verify sensors respond correctly
     *          
     *          Configuration includes:
     *          - Sensor range selection (full scale)
     *          - Output data rate (ODR) configuration
     *          - Filter bandwidth settings
     *          - Power mode and enable settings
     *          - Interrupt configuration (if used)
     * 
     * @return true if both sensors initialized successfully, false otherwise
     * 
     * @note Called only during probe() sequence
     * @warning Failure indicates hardware problem or incorrect SPI configuration
     * 
     * @see _init_gyro() for gyroscope-specific initialization
     * @see _init_accel() for accelerometer-specific initialization
     */
    bool _init_sensor();
    
    /**
     * @brief Initialize i3g4250d gyroscope via SPI
     * 
     * @details Configures the ST i3g4250d 3-axis MEMS gyroscope for operation.
     *          Performs register writes via SPI to set up sensor parameters.
     *          
     *          Initialization steps:
     *          1. Read and verify WHO_AM_I register (device ID 0xD3)
     *          2. Configure CTRL_REG1: Power mode, ODR, bandwidth
     *          3. Configure CTRL_REG4: Full scale selection, SPI mode
     *          4. Configure CTRL_REG5: FIFO and filtering options
     *          5. Calculate scale factor from full-scale setting
     *          6. Perform initial read to clear any stale data
     *          
     *          Typical configuration:
     *          - Full scale: ±2000 dps (degrees per second)
     *          - ODR: 800 Hz output data rate
     *          - Bandwidth: 110 Hz low-pass filter
     *          - Mode: Normal mode (not power-down)
     * 
     * @return true if gyro initialized successfully, false on failure
     * 
     * @note Sets _gyro_scale for raw-to-rad/s conversion
     * @warning SPI communication errors return false
     * @warning Incorrect WHO_AM_I indicates wrong sensor or no sensor
     * 
     * @see ST i3g4250d datasheet for register details
     */
    bool _init_gyro();
    
    /**
     * @brief Initialize iis328dq accelerometer via SPI
     * 
     * @details Configures the ST iis328dq 3-axis MEMS accelerometer for operation.
     *          Performs register writes via SPI to set up sensor parameters.
     *          
     *          Initialization steps:
     *          1. Read and verify WHO_AM_I register (device ID 0x32)
     *          2. Configure CTRL_REG1: Power mode, ODR, axis enable
     *          3. Configure CTRL_REG4: Full scale selection, SPI mode
     *          4. Configure CTRL_REG5: Boot and FIFO settings
     *          5. Calculate scale factor from full-scale setting
     *          6. Perform initial read to clear any stale data
     *          
     *          Typical configuration:
     *          - Full scale: ±8g (acceleration range)
     *          - ODR: 400 Hz output data rate
     *          - Mode: Normal mode, all axes enabled
     *          - Resolution: High resolution mode
     * 
     * @return true if accel initialized successfully, false on failure
     * 
     * @note Sets _accel_scale for raw-to-m/s² conversion
     * @warning SPI communication errors return false
     * @warning Incorrect WHO_AM_I indicates wrong sensor or no sensor
     * 
     * @see ST iis328dq datasheet for register details
     */
    bool _init_accel();
    
    /**
     * @brief Periodic callback to sample gyroscope data
     * 
     * @details Timer callback registered with HAL scheduler to read gyroscope
     *          samples at the configured data rate. Runs in timer interrupt context.
     *          
     *          Sampling workflow:
     *          1. Read STATUS_REG to check data ready flag
     *          2. Read OUT_X_L through OUT_Z_H registers (6 bytes)
     *          3. Combine low/high bytes into 16-bit signed values
     *          4. Apply rotation matrix (_rotation_g)
     *          5. Apply scale factor (_gyro_scale) to convert to rad/s
     *          6. Accumulate sample in backend buffer
     *          
     *          SPI transaction:
     *          - Multi-byte read with auto-increment addressing
     *          - CS asserted for entire transaction
     *          - Typical transaction time: ~50 µs at 1 MHz SPI clock
     *          
     *          Error handling:
     *          - Skip sample if data not ready
     *          - Detect and log SPI communication errors
     *          - Continue operation on transient errors
     * 
     * @note Runs in timer interrupt context - keep execution time minimal
     * @note Called at gyro ODR frequency (typically 100-800 Hz)
     * @warning Long execution time can cause scheduler overruns
     * @warning Must not block or use delays
     * 
     * @see start() for callback registration
     * @see update() for sample publication
     */
    void gyro_measure();
    
    /**
     * @brief Periodic callback to sample accelerometer data
     * 
     * @details Timer callback registered with HAL scheduler to read accelerometer
     *          samples at the configured data rate. Runs in timer interrupt context.
     *          
     *          Sampling workflow:
     *          1. Read STATUS_REG to check data ready flag
     *          2. Read OUT_X_L through OUT_Z_H registers (6 bytes)
     *          3. Combine low/high bytes into 16-bit signed values
     *          4. Apply rotation matrix (_rotation_a)
     *          5. Apply scale factor (_accel_scale) to convert to m/s²
     *          6. Accumulate sample in backend buffer
     *          
     *          SPI transaction:
     *          - Multi-byte read with auto-increment addressing
     *          - CS asserted for entire transaction
     *          - Typical transaction time: ~50 µs at 1 MHz SPI clock
     *          
     *          Error handling:
     *          - Skip sample if data not ready
     *          - Detect and log SPI communication errors
     *          - Continue operation on transient errors
     * 
     * @note Runs in timer interrupt context - keep execution time minimal
     * @note Called at accel ODR frequency (typically 50-400 Hz)
     * @warning Long execution time can cause scheduler overruns
     * @warning Must not block or use delays
     * 
     * @see start() for callback registration
     * @see update() for sample publication
     */
    void accel_measure();

    /**
     * @brief SPI device handle for i3g4250d gyroscope
     * 
     * @details Owned pointer to HAL SPI device for exclusive gyroscope access.
     *          Provides thread-safe SPI communication via HAL abstraction.
     *          Device ownership ensures no other code can interfere with transactions.
     *          
     *          SPI configuration (typical):
     *          - Clock speed: 1-10 MHz
     *          - Mode: SPI Mode 3 (CPOL=1, CPHA=1)
     *          - Bit order: MSB first
     *          - CS: Active low, controlled by device
     */
    AP_HAL::OwnPtr<AP_HAL::SPIDevice> _dev_gyro;  ///< i3g4250d gyroscope SPI device
    
    /**
     * @brief SPI device handle for iis328dq accelerometer
     * 
     * @details Owned pointer to HAL SPI device for exclusive accelerometer access.
     *          Provides thread-safe SPI communication via HAL abstraction.
     *          Device ownership ensures no other code can interfere with transactions.
     *          
     *          SPI configuration (typical):
     *          - Clock speed: 1-10 MHz
     *          - Mode: SPI Mode 3 (CPOL=1, CPHA=1)
     *          - Bit order: MSB first
     *          - CS: Active low, controlled by device
     */
    AP_HAL::OwnPtr<AP_HAL::SPIDevice> _dev_accel;  ///< iis328dq accelerometer SPI device

    /**
     * @brief Scale factor to convert gyro raw values to rad/s
     * 
     * @details Multiplier applied to raw 16-bit gyroscope ADC values to convert
     *          to standard angular rate units (radians per second).
     *          
     *          Scale calculation:
     *          - Depends on configured full-scale range (±245/±500/±2000 dps)
     *          - raw_value * scale = angular_rate in rad/s
     *          - Example: For ±2000 dps range: scale ≈ 0.07 mrad/s per LSB
     *          
     *          Conversion: dps to rad/s requires factor of (π/180)
     * 
     * @note Set during _init_gyro() based on full-scale configuration
     * @note Units: rad/s per LSB (least significant bit)
     */
    float _gyro_scale;
    
    /**
     * @brief Scale factor to convert accel raw values to m/s²
     * 
     * @details Multiplier applied to raw 16-bit accelerometer ADC values to convert
     *          to standard acceleration units (meters per second squared).
     *          
     *          Scale calculation:
     *          - Depends on configured full-scale range (±2g/±4g/±8g)
     *          - raw_value * scale = acceleration in m/s²
     *          - Example: For ±8g range: scale ≈ 2.4 mm/s² per LSB
     *          
     *          Conversion: g to m/s² requires factor of 9.80665 m/s² per g
     * 
     * @note Set during _init_accel() based on full-scale configuration
     * @note Units: m/s² per LSB (least significant bit)
     */
    float _accel_scale;

    /**
     * @brief Rotation matrix for gyroscope sensor alignment
     * 
     * @details Rotation applied to raw gyroscope data to align sensor axes with
     *          vehicle body frame. Compensates for physical sensor mounting orientation.
     *          
     *          Common rotations:
     *          - ROTATION_NONE: No rotation (sensor aligned with body frame)
     *          - ROTATION_YAW_90: 90° rotation around Z axis
     *          - ROTATION_PITCH_180: 180° rotation around Y axis
     *          - Many other standard rotations defined in enum Rotation
     *          
     *          Applied in gyro_measure() before publishing to frontend
     * 
     * @note Set via constructor parameter
     * @note Must match physical sensor mounting
     * @warning Incorrect rotation causes invalid attitude estimation
     */
    enum Rotation _rotation_g;
    
    /**
     * @brief Rotation matrix for accelerometer sensor alignment
     * 
     * @details Rotation applied to raw accelerometer data to align sensor axes with
     *          vehicle body frame. Compensates for physical sensor mounting orientation.
     *          
     *          Common rotations:
     *          - ROTATION_NONE: No rotation (sensor aligned with body frame)
     *          - ROTATION_YAW_90: 90° rotation around Z axis
     *          - ROTATION_PITCH_180: 180° rotation around Y axis
     *          - Many other standard rotations defined in enum Rotation
     *          
     *          Applied in accel_measure() before publishing to frontend
     * 
     * @note Set via constructor parameter
     * @note Must match physical sensor mounting
     * @warning Incorrect rotation causes invalid position/velocity estimation
     */
    enum Rotation _rotation_a;
};
#endif
