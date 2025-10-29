/**
 * @file AP_InertialSensor_LSM9DS1.h
 * @brief STMicroelectronics LSM9DS1 9-axis IMU driver
 * 
 * @details Implements backend driver for LSM9DS1, successor to LSM9DS0 with improved
 *          specifications and SPI/I2C dual-interface support. This driver handles the
 *          accelerometer and gyroscope components of the LSM9DS1 9-DOF system.
 *          
 *          The LSM9DS1 features enhanced noise performance, lower power consumption,
 *          and improved temperature stability compared to its predecessor.
 */

#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/SPIDevice.h>

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"

/* enable debug to see a register dump on startup */
#define LSM9DS1_DEBUG 0

/**
 * @class AP_InertialSensor_LSM9DS1
 * @brief Backend driver for LSM9DS1 9-axis IMU
 * 
 * @details Enhanced 9-DOF IMU driver supporting:
 *          - Gyro + accel + magnetometer in single package
 *          - SPI and I2C interfaces (dual-mode)
 *          - Gyro range: ±2000 deg/s
 *          - Accel range: ±16g
 *          - FIFO buffering for burst reads
 *          - Improved noise performance vs LSM9DS0
 *          
 *          Dual-device topology:
 *          - Accel/Gyro device (AG): One SPI/I2C interface
 *          - Magnetometer device (M): Separate SPI/I2C interface
 *          - Independent control and sampling rates
 *          
 *          FIFO operation:
 *          - 32-level FIFO for gyro and accel
 *          - Watermark interrupt support
 *          - Reduces main CPU polling overhead
 *          
 *          Driver lifecycle:
 *          1. probe() - Static factory method for device detection
 *          2. Constructor - Initialize device handles and configuration
 *          3. _hardware_init() - Configure registers and scaling
 *          4. start() - Begin periodic sampling via scheduler
 *          5. update() - Called by main loop to publish sensor data
 *          6. _poll_data() - Periodic callback to read FIFO and process samples
 * 
 * @note Magnetometer handled by AP_Compass_LSM9DS1, not this driver
 * @note SPI mode preferred over I2C for lower latency and higher reliability
 * @warning FIFO must be flushed after configuration changes to avoid stale data
 * 
 * Source: libraries/AP_InertialSensor/AP_InertialSensor_LSM9DS1.h
 */
class AP_InertialSensor_LSM9DS1 : public AP_InertialSensor_Backend
{
public:
    /**
     * @brief Destructor for LSM9DS1 driver instance
     */
    virtual ~AP_InertialSensor_LSM9DS1() { }
    
    /**
     * @brief Start periodic sampling of the IMU
     * 
     * @details Registers periodic callback with scheduler to poll sensor data
     *          at configured sample rate. Called once during IMU initialization.
     */
    void start(void) override;
    
    /**
     * @brief Update sensor data for main loop
     * 
     * @details Called by main loop to publish accumulated sensor samples
     *          to the frontend. Processes data collected by _poll_data().
     * 
     * @return true if new data was published, false otherwise
     */
    bool update() override;

    /**
     * @brief Probe for LSM9DS1 device on SPI bus
     * 
     * @details Factory method that attempts to detect and initialize LSM9DS1.
     *          Reads WHO_AM_I register to verify device identity, then creates
     *          driver instance if successful.
     * 
     * @param[in] imu           Reference to AP_InertialSensor frontend
     * @param[in] dev           SPI device handle (ownership transferred)
     * @param[in] rotation      Board rotation for coordinate frame transformation
     * 
     * @return Pointer to initialized backend instance, or nullptr if probe failed
     * 
     * @note This is the primary entry point for driver instantiation
     */
    static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu,
                                            AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev,
                                            enum Rotation rotation);
private:
    /**
     * @brief Private constructor for LSM9DS1 driver
     * 
     * @details Initializes driver instance with device handles and configuration.
     *          Called by probe() after successful device detection.
     * 
     * @param[in] imu              Reference to AP_InertialSensor frontend
     * @param[in] dev              SPI device handle (ownership transferred)
     * @param[in] drdy_pin_num_xg  Data ready GPIO pin number for accel/gyro
     * @param[in] rotation         Board rotation for coordinate frame transformation
     */
    AP_InertialSensor_LSM9DS1(AP_InertialSensor &imu,
                              AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev,
                              int drdy_pin_num_xg,
                              enum Rotation rotation);

    /**
     * @brief Detect LSM9DS1 device on system
     * 
     * @param[in] imu Reference to AP_InertialSensor frontend
     * @return Pointer to initialized backend instance, or nullptr if not found
     */
    static AP_InertialSensor_Backend *detect(AP_InertialSensor &imu);

    /**
     * @struct sensor_raw_data
     * @brief Raw 16-bit sensor data structure
     * 
     * @details Packed structure for reading raw accelerometer or gyroscope data
     *          from LSM9DS1 registers. Data is in two's complement format and
     *          requires scaling to physical units.
     */
    struct PACKED sensor_raw_data {
        int16_t x;  ///< X-axis raw data
        int16_t y;  ///< Y-axis raw data
        int16_t z;  ///< Z-axis raw data
    };

    /**
     * @enum accel_scale
     * @brief Accelerometer full-scale range configuration
     * 
     * @details Defines available accelerometer measurement ranges.
     *          Larger ranges reduce sensitivity but increase maximum measurable acceleration.
     */
    enum accel_scale {
        A_SCALE_2G = 0,   ///< ±2g range (highest sensitivity)
        A_SCALE_4G,       ///< ±4g range
        A_SCALE_8G,       ///< ±8g range
        A_SCALE_16G       ///< ±16g range (lowest sensitivity, highest range)
    };

    /**
     * @brief Poll sensor data from device
     * 
     * @details Periodic callback registered with scheduler. Reads FIFO contents,
     *          applies scaling and rotation, and accumulates samples for publication.
     *          Called at configured sample rate (typically 952Hz for LSM9DS1).
     */
    void _poll_data();
    
    /**
     * @brief Reset FIFO buffer
     * 
     * @details Clears FIFO contents and resets watermark. Must be called after
     *          configuration changes to prevent reading stale data.
     * 
     * @warning Discards any pending samples in FIFO
     */
    void _fifo_reset();

    /**
     * @brief Initialize sensor hardware
     * 
     * @details Top-level initialization: verifies device ID, configures registers,
     *          sets up FIFO, and prepares for data acquisition.
     * 
     * @return true if initialization successful, false on failure
     */
    bool _init_sensor();
    
    /**
     * @brief Configure hardware registers
     * 
     * @details Low-level hardware setup: gyroscope, accelerometer, and FIFO configuration.
     * 
     * @return true if configuration successful, false on failure
     */
    bool _hardware_init();

    /**
     * @brief Initialize gyroscope subsystem
     * 
     * @details Configures gyroscope output data rate, bandwidth, and enables axes.
     */
    void _gyro_init();
    
    /**
     * @brief Initialize accelerometer subsystem
     * 
     * @details Configures accelerometer output data rate, anti-aliasing filter, and enables axes.
     */
    void _accel_init();

    /**
     * @brief Set gyroscope full-scale range
     * 
     * @details Configures gyroscope to ±2000 deg/s range and calculates scale factor
     *          for converting raw data to deg/s.
     */
    void _set_gyro_scale();
    
    /**
     * @brief Set accelerometer full-scale range
     * 
     * @details Configures accelerometer range and calculates scale factor for
     *          converting raw data to m/s².
     * 
     * @param[in] scale Desired accelerometer range (A_SCALE_2G to A_SCALE_16G)
     */
    void _set_accel_scale(accel_scale scale);

    /**
     * @brief Read single register from device
     * 
     * @param[in] reg Register address to read
     * @return Register value (8-bit)
     */
    uint8_t _register_read(uint8_t reg);
    
    /**
     * @brief Write single register to device
     * 
     * @param[in] reg     Register address to write
     * @param[in] val     Value to write (8-bit)
     * @param[in] checked If true, read back and verify written value
     */
    void _register_write(uint8_t reg, uint8_t val, bool checked=false);

    /**
     * @brief Read accelerometer data from FIFO
     * 
     * @details Performs SPI transaction to read specified number of accelerometer
     *          samples from FIFO buffer.
     * 
     * @param[in] samples Number of samples to read from FIFO
     */
    void _read_data_transaction_x(uint16_t samples);
    
    /**
     * @brief Read gyroscope data from FIFO
     * 
     * @details Performs SPI transaction to read specified number of gyroscope
     *          samples from FIFO buffer.
     * 
     * @param[in] samples Number of samples to read from FIFO
     */
    void _read_data_transaction_g(uint16_t samples);

    #if LSM9DS1_DEBUG
    /**
     * @brief Dump all device registers for debugging
     * 
     * @details Reads and prints all LSM9DS1 registers to console for
     *          hardware troubleshooting. Only available when LSM9DS1_DEBUG is enabled.
     */
    void        _dump_registers();
    #endif

    // Device handles and configuration
    AP_HAL::OwnPtr<AP_HAL::SPIDevice> _dev;     ///< SPI device handle for accel/gyro (AG device)
    AP_HAL::Semaphore *_spi_sem;                ///< SPI bus semaphore for thread-safe access
    AP_HAL::DigitalSource * _drdy_pin_xg;       ///< Data ready GPIO pin for accel/gyro
    
    // Scaling factors
    float _gyro_scale;                          ///< Gyroscope scale factor (raw to deg/s)
    float _accel_scale;                         ///< Accelerometer scale factor (raw to m/s²)
    
    // Configuration parameters
    int _drdy_pin_num_xg;                       ///< Data ready pin number for accel/gyro
    enum Rotation _rotation;                    ///< Board rotation for coordinate frame transformation
};
