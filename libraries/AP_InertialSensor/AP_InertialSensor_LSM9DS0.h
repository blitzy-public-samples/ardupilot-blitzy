/**
 * @file AP_InertialSensor_LSM9DS0.h
 * @brief STMicroelectronics LSM9DS0 9-axis IMU driver
 * 
 * @details Implements backend driver for LSM9DS0 9-DOF sensor combining gyroscope,
 *          accelerometer, and magnetometer in single package. The LSM9DS0 is a
 *          system-in-package featuring a 3D digital linear acceleration sensor,
 *          a 3D digital angular rate sensor, and a 3D digital magnetic sensor.
 *          
 *          Two-device architecture:
 *          - XM device: Accelerometer + Magnetometer (I2C address 0x1E)
 *          - G device: Gyroscope (I2C address 0x6A or 0x6B)
 *          
 *          Key specifications:
 *          - Gyro full scales: ±245, ±500, ±2000 deg/s
 *          - Accel full scales: ±2g, ±4g, ±6g, ±8g, ±16g
 *          - 16-bit ADC resolution for all sensors
 *          - Configurable data-ready interrupt pins
 *          - SPI interface support
 *          
 *          This driver handles gyroscope and accelerometer only. The magnetometer
 *          is handled by the separate AP_Compass_LSM9DS0 driver.
 * 
 * @note Legacy sensor - uncommon in modern flight controllers, replaced by newer IMUs
 * @note Magnetometer functionality accessed via AP_Compass_LSM9DS0, not this driver
 * @warning Requires both device addresses (gyro and accel) responding for successful probe
 * 
 * Source: libraries/AP_InertialSensor/AP_InertialSensor_LSM9DS0.h
 */

#pragma once

#define LSM9DS0_DEBUG 0

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/SPIDevice.h>

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"
#include <Filter/LowPassFilter2p.h>

/**
 * @class AP_InertialSensor_LSM9DS0
 * @brief Backend driver for LSM9DS0 9-axis IMU
 * 
 * @details Driver for ST LSM9DS0 9-DOF system-in-package combining gyroscope
 *          and accelerometer functionality. This driver follows the standard
 *          ArduPilot sensor backend lifecycle:
 *          
 *          Lifecycle: probe() → constructor → _init_sensor() → start() → update() loop
 *          
 *          The LSM9DS0 consists of two separate devices on different addresses:
 *          - Gyro device (G): 3-axis angular rate sensor
 *          - Accel/Mag device (XM): 3-axis accelerometer + magnetometer
 *          
 *          This driver manages dual-device communication, separate register banks,
 *          independent data-ready signals, and configurable rotation matrices for
 *          each sensor to handle board-specific mounting orientations.
 *          
 *          Data Ready Handling:
 *          - Preferred: Hardware GPIO pins for data-ready interrupts (best performance)
 *          - Fallback: Status register polling (if GPIO pins not available)
 *          
 *          Thread Safety:
 *          - Uses HAL semaphore (_spi_sem) for SPI bus arbitration
 *          - Called from main scheduler at configured sample rate
 * 
 * @note Temperature reading integrated from accel/mag device, filtered with low-pass filter
 * @note Supports different rotation matrices for L3GD20 vs L3GD20H gyro variants
 * @warning Both gyro and accel devices must be functional for driver operation
 */
class AP_InertialSensor_LSM9DS0 : public AP_InertialSensor_Backend
{
public:
    /**
     * @brief Virtual destructor
     * 
     * @details Ensures proper cleanup of derived class resources.
     *          Device cleanup handled by AP_HAL::OwnPtr smart pointers.
     */
    virtual ~AP_InertialSensor_LSM9DS0() { }
    
    /**
     * @brief Start periodic sampling of IMU data
     * 
     * @details Registers periodic callback with HAL scheduler to poll sensor data
     *          at configured sample rate. This is called after successful probe
     *          and initialization. Sets up the main data acquisition loop by
     *          registering _poll_data() as a periodic callback function.
     *          
     *          Typical sample rate: 1000 Hz (1 kHz) for gyro and accel
     * 
     * @note Called once during sensor initialization, not every sample
     * @note Must be called after successful _init_sensor() completion
     */
    void start(void) override;
    
    /**
     * @brief Update sensor readings and publish to frontend
     * 
     * @details Main update function called by AP_InertialSensor frontend to retrieve
     *          latest sensor data. Checks for new data availability, reads from both
     *          gyro and accel devices, applies scaling factors and rotation matrices,
     *          and publishes samples to the frontend for fusion.
     *          
     *          Update sequence:
     *          1. Check data-ready flags (GPIO or status register)
     *          2. Read raw sensor data if available
     *          3. Apply scale factors (deg/s for gyro, m/s² for accel)
     *          4. Apply rotation matrices for board orientation
     *          5. Publish samples to frontend via _publish_gyro() and _publish_accel()
     *          6. Update temperature reading periodically
     * 
     * @return true if new data was available and published, false otherwise
     * 
     * @note Called at main loop rate (typically 400 Hz for copter, 50-400 Hz for plane)
     * @note Actual sensor sampling rate independent of update() call frequency
     */
    bool update() override;

    /**
     * @brief Probe and initialize LSM9DS0 sensor
     * 
     * @details Static factory method that attempts to detect and initialize an LSM9DS0
     *          IMU. Verifies device presence by reading WHO_AM_I registers from both
     *          gyro and accel devices, initializes hardware configuration, and creates
     *          driver instance if successful.
     *          
     *          Probe sequence:
     *          1. Verify gyro device responds (WHO_AM_I register check)
     *          2. Verify accel device responds (WHO_AM_I register check)
     *          3. Detect gyro variant (L3GD20 vs L3GD20H) for correct rotation
     *          4. Create driver instance if both devices present
     *          5. Initialize hardware configuration (_init_sensor)
     *          6. Register instances with AP_InertialSensor frontend
     *          
     *          Expected WHO_AM_I values:
     *          - Gyro: 0xD4 (L3GD20) or 0xD7 (L3GD20H)
     *          - Accel/Mag: 0x49 (LSM9DS0)
     * 
     * @param[in,out] imu Reference to AP_InertialSensor frontend for registration
     * @param[in] dev_gyro HAL device pointer for gyro device (takes ownership)
     * @param[in] dev_accel HAL device pointer for accel device (takes ownership)
     * @param[in] rotation_a Rotation matrix for accelerometer mounting orientation
     * @param[in] rotation_g Rotation matrix for gyro mounting (L3GD20)
     * @param[in] rotation_gH Rotation matrix for gyro mounting (L3GD20H variant)
     * 
     * @return Pointer to new driver instance if probe successful, nullptr if failed
     * 
     * @note Returns nullptr if either device fails WHO_AM_I check
     * @note Takes ownership of device pointers - caller should not use after call
     * @warning Both gyro and accel devices must be functional for successful probe
     */
    static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu,
                                            AP_HAL::OwnPtr<AP_HAL::Device> dev_gyro,
                                            AP_HAL::OwnPtr<AP_HAL::Device> dev_accel,
                                            enum Rotation rotation_a,
                                            enum Rotation rotation_g,
                                            enum Rotation rotation_gH);

private:
    /**
     * @brief Private constructor for LSM9DS0 driver
     * 
     * @details Constructs driver instance with specified devices and configuration.
     *          Called internally by probe() after successful device detection.
     *          Initializes device pointers, data-ready pins, and rotation matrices.
     * 
     * @param[in,out] imu Reference to AP_InertialSensor frontend
     * @param[in] dev_gyro HAL device for gyroscope (takes ownership)
     * @param[in] dev_accel HAL device for accelerometer (takes ownership)
     * @param[in] drdy_pin_num_a GPIO pin number for accel data-ready (-1 if unused)
     * @param[in] drdy_pin_num_b GPIO pin number for gyro data-ready (-1 if unused)
     * @param[in] rotation_a Rotation matrix for accelerometer orientation
     * @param[in] rotation_g Rotation matrix for L3GD20 gyro orientation
     * @param[in] rotation_gH Rotation matrix for L3GD20H gyro orientation
     * 
     * @note Constructor is private - use probe() static factory method
     */
    AP_InertialSensor_LSM9DS0(AP_InertialSensor &imu,
                              AP_HAL::OwnPtr<AP_HAL::Device> dev_gyro,
                              AP_HAL::OwnPtr<AP_HAL::Device> dev_accel,
                              int drdy_pin_num_a, int drdy_pin_num_b,
                              enum Rotation rotation_a,
                              enum Rotation rotation_g,
                              enum Rotation rotation_gH);

    /**
     * @struct sensor_raw_data
     * @brief Raw 3-axis sensor data structure
     * 
     * @details Packed structure for reading raw 16-bit sensor values from
     *          device registers. Used for both gyroscope and accelerometer
     *          data. Values are in device-specific units before scaling.
     *          
     *          Memory layout matches LSM9DS0 register output format for
     *          efficient bulk reading of X, Y, Z axis data.
     */
    struct PACKED sensor_raw_data {
        int16_t x;  ///< X-axis raw reading (LSB)
        int16_t y;  ///< Y-axis raw reading (LSB)
        int16_t z;  ///< Z-axis raw reading (LSB)
    };

    /**
     * @enum gyro_scale
     * @brief Gyroscope full-scale range configuration
     * 
     * @details Defines available full-scale ranges for gyroscope measurements.
     *          Smaller ranges provide better resolution, larger ranges prevent
     *          saturation during high angular rate maneuvers.
     *          
     *          Scale factors (LSB/deg/s):
     *          - 245 dps: 0.00875 deg/s per LSB
     *          - 500 dps: 0.0175 deg/s per LSB
     *          - 2000 dps: 0.070 deg/s per LSB
     * 
     * @note Driver typically uses G_SCALE_2000DPS for maximum range
     */
    enum gyro_scale {
        G_SCALE_245DPS = 0,   ///< ±245 degrees per second (highest resolution)
        G_SCALE_500DPS,       ///< ±500 degrees per second
        G_SCALE_2000DPS,      ///< ±2000 degrees per second (maximum range)
    };

    /**
     * @enum accel_scale
     * @brief Accelerometer full-scale range configuration
     * 
     * @details Defines available full-scale ranges for accelerometer measurements.
     *          Scale selection trades resolution versus maximum measurable acceleration.
     *          
     *          Scale factors (mg/LSB):
     *          - 2g: 0.061 mg/LSB
     *          - 4g: 0.122 mg/LSB
     *          - 6g: 0.183 mg/LSB
     *          - 8g: 0.244 mg/LSB
     *          - 16g: 0.732 mg/LSB
     * 
     * @note Driver typically uses A_SCALE_16G to avoid saturation during aerobatics
     */
    enum accel_scale {
        A_SCALE_2G = 0,   ///< ±2g (highest resolution)
        A_SCALE_4G,       ///< ±4g
        A_SCALE_6G,       ///< ±6g
        A_SCALE_8G,       ///< ±8g
        A_SCALE_16G,      ///< ±16g (maximum range)
    };

    /**
     * @brief Check if new accelerometer data is ready
     * 
     * @details Checks data-ready status using either GPIO pin (preferred) or
     *          status register polling (fallback). GPIO method provides lower
     *          latency and reduces SPI/I2C bus traffic.
     * 
     * @return true if new accelerometer sample available, false otherwise
     * 
     * @note GPIO method: Reads digital pin state directly
     * @note Register method: Reads STATUS_REG_A and checks ZYXDA bit
     */
    bool _accel_data_ready();
    
    /**
     * @brief Check if new gyroscope data is ready
     * 
     * @details Checks data-ready status using either GPIO pin (preferred) or
     *          status register polling (fallback). GPIO pins provide best
     *          performance for high-rate sampling.
     * 
     * @return true if new gyroscope sample available, false otherwise
     * 
     * @note GPIO method: Reads digital pin state directly
     * @note Register method: Reads STATUS_REG and checks ZYXDA bit
     */
    bool _gyro_data_ready();

    /**
     * @brief Periodic data polling callback
     * 
     * @details Registered as periodic callback with HAL scheduler. Called at
     *          configured sample rate to read sensor data. Checks data-ready
     *          flags, reads raw data from both devices, and updates internal
     *          state. This is the main data acquisition function.
     *          
     *          Execution sequence:
     *          1. Check if gyro data ready
     *          2. Check if accel data ready
     *          3. Read data from ready devices
     *          4. Update temperature periodically (every ~100 samples)
     * 
     * @note Called from HAL scheduler at high priority (typically 1 kHz)
     * @note Uses semaphore for thread-safe SPI bus access
     * @warning Must execute quickly to avoid blocking scheduler
     */
    void _poll_data();

    /**
     * @brief Initialize sensor hardware configuration
     * 
     * @details Performs complete sensor initialization sequence including device
     *          identification, configuration register setup, and validation.
     *          Called after successful device probe.
     *          
     *          Initialization sequence:
     *          1. Verify gyro WHO_AM_I register
     *          2. Verify accel WHO_AM_I register
     *          3. Disable I2C interfaces (if using SPI)
     *          4. Initialize gyro configuration
     *          5. Initialize accel configuration
     *          6. Set scale factors
     *          7. Enable data output
     * 
     * @return true if initialization successful, false if any step fails
     * 
     * @note Failure typically indicates hardware problem or wrong device
     * @warning Must be called before start() and data acquisition
     */
    bool _init_sensor();
    
    /**
     * @brief Low-level hardware initialization
     * 
     * @details Wrapper for _init_sensor() providing consistent interface
     *          with other sensor backend drivers.
     * 
     * @return true if hardware init successful, false otherwise
     */
    bool _hardware_init();

    /**
     * @brief Initialize gyroscope configuration
     * 
     * @details Configures gyroscope operating parameters including output data rate,
     *          bandwidth, full-scale range, and power mode. Sets up continuous
     *          measurement mode with appropriate filtering.
     *          
     *          Typical configuration:
     *          - Output data rate: 760 Hz
     *          - Full-scale: ±2000 deg/s
     *          - High-pass filter: Disabled or normal mode
     *          - Power mode: Normal (all axes enabled)
     * 
     * @note Must be called during _init_sensor() sequence
     */
    void _gyro_init();
    
    /**
     * @brief Initialize accelerometer configuration
     * 
     * @details Configures accelerometer operating parameters including output data rate,
     *          full-scale range, anti-aliasing filter, and power mode. Sets up
     *          continuous measurement mode.
     *          
     *          Typical configuration:
     *          - Output data rate: 1600 Hz
     *          - Full-scale: ±16g
     *          - Anti-aliasing filter bandwidth: 773 Hz
     *          - Power mode: Normal (all axes enabled)
     * 
     * @note Must be called during _init_sensor() sequence
     */
    void _accel_init();

    /**
     * @brief Disable I2C interface on gyroscope device
     * 
     * @details When using SPI communication, the I2C interface must be disabled
     *          to prevent bus conflicts. Sets I2C_DIS bit in configuration register.
     * 
     * @note Only relevant when using SPI interface
     * @note Must be called before enabling SPI communication
     */
    void _gyro_disable_i2c();
    
    /**
     * @brief Disable I2C interface on accelerometer device
     * 
     * @details When using SPI communication, the I2C interface must be disabled
     *          to prevent bus conflicts. Sets I2C_DIS bit in configuration register.
     * 
     * @note Only relevant when using SPI interface
     * @note Must be called before enabling SPI communication
     */
    void _accel_disable_i2c();

    /**
     * @brief Set gyroscope full-scale range
     * 
     * @details Configures gyroscope measurement range and updates internal scale
     *          factor for raw data conversion to engineering units (deg/s).
     * 
     * @param[in] scale Desired gyroscope full-scale range
     * 
     * @note Scale factor stored in _gyro_scale for data conversion
     * @see gyro_scale enum for available ranges
     */
    void _set_gyro_scale(gyro_scale scale);
    
    /**
     * @brief Set accelerometer full-scale range
     * 
     * @details Configures accelerometer measurement range and updates internal scale
     *          factor for raw data conversion to engineering units (m/s²).
     * 
     * @param[in] scale Desired accelerometer full-scale range
     * 
     * @note Scale factor stored in _accel_scale for data conversion
     * @see accel_scale enum for available ranges
     */
    void _set_accel_scale(accel_scale scale);

    /**
     * @brief Read register from accelerometer/magnetometer device
     * 
     * @details Performs single register read from XM (accelerometer/magnetometer)
     *          device. Handles SPI/I2C protocol differences automatically.
     * 
     * @param[in] reg Register address to read
     * 
     * @return Register value (8-bit)
     * 
     * @note XM device typically at I2C address 0x1E or SPI CS line
     */
    uint8_t _register_read_xm(uint8_t reg);
    
    /**
     * @brief Read register from gyroscope device
     * 
     * @details Performs single register read from G (gyroscope) device.
     *          Handles SPI/I2C protocol differences automatically.
     * 
     * @param[in] reg Register address to read
     * 
     * @return Register value (8-bit)
     * 
     * @note G device typically at I2C address 0x6A/0x6B or separate SPI CS line
     */
    uint8_t _register_read_g(uint8_t reg);
    
    /**
     * @brief Write register to accelerometer/magnetometer device
     * 
     * @details Performs single register write to XM device. Optionally reads back
     *          register value to verify write succeeded.
     * 
     * @param[in] reg Register address to write
     * @param[in] val Value to write (8-bit)
     * @param[in] checked If true, read back register to verify write (default: false)
     * 
     * @warning checked=true increases bus traffic, use for critical config only
     */
    void _register_write_xm(uint8_t reg, uint8_t val, bool checked=false);
    
    /**
     * @brief Write register to gyroscope device
     * 
     * @details Performs single register write to G device. Optionally reads back
     *          register value to verify write succeeded.
     * 
     * @param[in] reg Register address to write
     * @param[in] val Value to write (8-bit)
     * @param[in] checked If true, read back register to verify write (default: false)
     * 
     * @warning checked=true increases bus traffic, use for critical config only
     */
    void _register_write_g(uint8_t reg, uint8_t val, bool checked=false);

    /**
     * @brief Read accelerometer data transaction
     * 
     * @details Performs bulk read of accelerometer X, Y, Z axis data registers.
     *          Reads 6 bytes (3 axes × 2 bytes each) in single transaction for
     *          efficiency and data coherency. Updates internal accel sample buffer.
     * 
     * @note Uses burst read mode for atomic multi-register access
     * @note Data stored in _accel_data for processing by update()
     */
    void _read_data_transaction_a();
    
    /**
     * @brief Read gyroscope data transaction
     * 
     * @details Performs bulk read of gyroscope X, Y, Z axis data registers.
     *          Reads 6 bytes (3 axes × 2 bytes each) in single transaction for
     *          efficiency and data coherency. Updates internal gyro sample buffer.
     * 
     * @note Uses burst read mode for atomic multi-register access
     * @note Data stored in _gyro_data for processing by update()
     */
    void _read_data_transaction_g();

#if LSM9DS0_DEBUG
    /**
     * @brief Dump all sensor registers for debugging
     * 
     * @details Debug function that reads and displays all configuration and
     *          data registers from both gyro and accel devices. Useful for
     *          hardware debugging and configuration verification.
     *          
     *          Only compiled when LSM9DS0_DEBUG=1 is defined.
     * 
     * @note Output sent to HAL console
     * @warning Generates significant console output
     */
    void        _dump_registers();
#endif

    // Hardware device interfaces
    AP_HAL::OwnPtr<AP_HAL::Device> _dev_gyro;    ///< HAL device for gyroscope (SPI/I2C)
    AP_HAL::OwnPtr<AP_HAL::Device> _dev_accel;   ///< HAL device for accelerometer (SPI/I2C)
    AP_HAL::Semaphore *_spi_sem;                 ///< Semaphore for SPI bus arbitration

    /*
     * Data-ready GPIO pins for interrupt-driven sampling.
     * 
     * If data-ready GPIO pin numbers are not defined (i.e. any negative
     * value), the fallback approach used is to check if there's new data ready
     * by reading the status register. It is *strongly* recommended to use
     * data-ready GPIO pins for performance reasons as polling status registers
     * increases bus traffic and latency.
     */
    AP_HAL::DigitalSource * _drdy_pin_a;   ///< Digital source for accel data-ready interrupt
    AP_HAL::DigitalSource * _drdy_pin_g;   ///< Digital source for gyro data-ready interrupt
    
    // Scale factors for converting raw data to engineering units
    float _gyro_scale;    ///< Gyro scale factor: LSB to deg/s conversion
    float _accel_scale;   ///< Accel scale factor: LSB to m/s² conversion
    
    // Data-ready pin configuration
    int _drdy_pin_num_a;  ///< GPIO pin number for accel data-ready (-1 if unused)
    int _drdy_pin_num_g;  ///< GPIO pin number for gyro data-ready (-1 if unused)
    
    // Temperature measurement
    float _temperature;              ///< Current temperature reading in °C
    uint8_t _temp_counter;           ///< Counter for periodic temperature updates
    LowPassFilter2pFloat _temp_filter; ///< Low-pass filter for temperature smoothing

    // Device identification
    uint8_t whoami_g;  ///< Gyro WHO_AM_I value (0xD4=L3GD20, 0xD7=L3GD20H)
    
    /*
     * Rotation matrices for sensor orientation.
     * 
     * For boards that have a separate LSM303D and L3GD20 there can be
     * different rotations for each sensor. The LSM9DS0 supports configurable
     * rotation matrices to handle various PCB mounting orientations.
     */
    enum Rotation _rotation_a;   ///< Accelerometer rotation matrix
    enum Rotation _rotation_g;   ///< Gyroscope rotation (for L3GD20 variant)
    enum Rotation _rotation_gH;  ///< Gyroscope rotation (for L3GD20H variant)
};
