/**
 * @file AP_Compass_HMC5843.h
 * @brief Driver for Honeywell HMC5843 and HMC5883L 3-axis magnetometers
 * 
 * @details This file implements support for the HMC5843 and HMC5883L digital
 *          compass sensors. These sensors communicate via I2C and provide
 *          3-axis magnetic field measurements used for heading determination.
 *          
 *          The driver supports:
 *          - Direct I2C connection via AP_HAL::Device
 *          - Auxiliary I2C bus connection through IMU (e.g., MPU6000)
 *          - Automatic sensor detection via WHO_AM_I register
 *          - Factory calibration with positive bias test
 *          - Continuous sampling at up to 75Hz
 *          - External compass mounting with rotation compensation
 *          
 *          The HMC5883L is the production successor to the HMC5843 with
 *          identical register interface but different gain settings.
 * 
 * @note Default I2C address is 0x1E
 * @warning Ensure external compasses are mounted away from magnetic
 *          interference sources (motors, ESCs, power wiring) for
 *          accurate heading measurement
 * 
 * @author Original code by Jordi Muñoz and Jose Julio, DIYDrones.com
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 * 
 * Source: libraries/AP_Compass/AP_Compass_HMC5843.h:1-163
 * Source: libraries/AP_Compass/AP_Compass_HMC5843.cpp
 */

#pragma once

#include "AP_Compass_config.h"

#if AP_COMPASS_HMC5843_ENABLED

#ifndef HAL_COMPASS_HMC5843_I2C_ADDR
#define HAL_COMPASS_HMC5843_I2C_ADDR 0x1E
#endif

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Device.h>

#include "AP_Compass_Backend.h"
#include <AP_InertialSensor/AP_InertialSensor_config.h>

class AuxiliaryBus;
class AuxiliaryBusSlave;
class AP_InertialSensor;
class AP_HMC5843_BusDriver;

/**
 * @class AP_Compass_HMC5843
 * @brief Backend driver for HMC5843/HMC5883L 3-axis magnetometer
 * 
 * @details This class implements the ArduPilot compass backend interface for
 *          Honeywell HMC5843 and HMC5883L magnetometers. The driver manages
 *          the complete sensor lifecycle including detection, calibration,
 *          continuous sampling, and data accumulation.
 *          
 *          Driver Lifecycle:
 *          1. probe() - Creates driver instance and calls init()
 *          2. init() - WHO_AM_I check → calibration → sampling setup → register compass
 *          3. _timer() - Called at 75Hz to read samples and accumulate data
 *          4. read() - Called by frontend at ~10Hz to drain accumulated samples
 *          
 *          The driver uses a bus abstraction layer (AP_HMC5843_BusDriver) to
 *          support both direct I2C and auxiliary bus connections. This allows
 *          the same driver to work with compasses connected directly to the
 *          autopilot or through an IMU's auxiliary I2C port.
 *          
 *          Measurement Range: ±8.1 Gauss (±810 milligauss)
 *          Output Rate: 75Hz (samples accumulated for frontend)
 *          Resolution: ~2 milligauss
 *          
 * @note Thread Safety: All bus operations are protected by the bus semaphore
 * @note Coordinate Frame: Raw sensor readings are in sensor frame (X forward,
 *       Y right, Z down). External compasses are rotated 90° in yaw to align
 *       with standard board orientation.
 * 
 * @warning Do not call read() or access sensor data from interrupt context
 * @warning Ensure proper power-up delay (>5ms) before attempting communication
 * 
 * @see AP_Compass_Backend for base interface requirements
 * @see AP_HMC5843_BusDriver for bus abstraction
 * 
 * Source: libraries/AP_Compass/AP_Compass_HMC5843.cpp:95-221
 */
class AP_Compass_HMC5843 : public AP_Compass_Backend
{
public:
    /**
     * @brief Factory method to probe and initialize HMC5843/HMC5883L on a HAL device
     * 
     * @details This static factory method attempts to detect and initialize a
     *          HMC5843/HMC5883L magnetometer on the provided device. It performs
     *          the following sequence:
     *          1. Creates a HAL device bus driver wrapper
     *          2. Instantiates the compass driver
     *          3. Calls init() to verify sensor and perform calibration
     *          4. Returns initialized driver or nullptr on failure
     *          
     *          The probe sequence includes WHO_AM_I verification and factory
     *          calibration with positive bias test to validate sensor operation.
     * 
     * @param[in] dev           OwnPtr to initialized I2C or SPI device at correct address
     * @param[in] force_external If true, marks compass as external regardless of detection
     * @param[in] rotation      Board orientation rotation for compass mounting
     * 
     * @return Pointer to initialized AP_Compass_Backend on success, nullptr on failure
     * 
     * @note Ownership of dev is transferred to the driver on success
     * @note On failure, dev is automatically destroyed
     * @note Typical probe time: 200-500ms due to calibration sequence
     * 
     * @see init() for detailed initialization steps
     * 
     * Source: libraries/AP_Compass/AP_Compass_HMC5843.cpp:108-127
     */
    static AP_Compass_Backend *probe(AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                     bool force_external,
                                     enum Rotation rotation);

#if AP_INERTIALSENSOR_ENABLED
    /**
     * @brief Factory method to probe HMC5843 connected to MPU6000 auxiliary I2C bus
     * 
     * @details This specialized probe method detects and initializes a HMC5843/HMC5883L
     *          connected to the auxiliary I2C port of an MPU6000/MPU6050 IMU. This
     *          configuration allows the IMU to act as an I2C master and read compass
     *          data automatically, reducing main processor I2C bus load.
     *          
     *          The compass is accessed through the InertialSensor auxiliary bus
     *          interface at the default HMC5843 I2C address (0x1E).
     * 
     * @param[in] rotation Board orientation rotation for compass mounting
     * 
     * @return Pointer to initialized AP_Compass_Backend on success, nullptr on failure
     * 
     * @note This method automatically sets force_external=false since auxiliary
     *       bus compasses are typically integrated on the same board as the IMU
     * @note Requires valid AP_InertialSensor singleton instance
     * @note MPU6000 auxiliary bus must be configured and operational
     * 
     * Source: libraries/AP_Compass/AP_Compass_HMC5843.cpp:130-149
     */
    static AP_Compass_Backend *probe_mpu6000(enum Rotation rotation);
#endif

    static constexpr const char *name = "HMC5843";

    virtual ~AP_Compass_HMC5843();

    /**
     * @brief Read accumulated magnetometer samples from the frontend
     * 
     * @details This method is called by the compass frontend (typically at ~10Hz)
     *          to retrieve accumulated magnetic field measurements. Samples are
     *          collected at 75Hz by _timer() and accumulated in the frontend
     *          buffer. This method drains the accumulated samples and applies
     *          calibration scaling.
     *          
     *          If the compass is not yet initialized, this method returns without
     *          reading to prevent mid-flight initialization issues.
     * 
     * @note This is called from the main scheduler task, not interrupt context
     * @note Bus semaphore must NOT be held when calling this method
     * @note Accumulated samples are automatically averaged by the frontend
     * 
     * @see _timer() for sample collection at 75Hz
     * @see drain_accumulated_samples() in AP_Compass_Backend
     * 
     * Source: libraries/AP_Compass/AP_Compass_HMC5843.cpp:260-270
     */
    void read() override;

private:
    /**
     * @brief Private constructor for compass driver instance
     * 
     * @details Constructs a compass driver with the specified bus, external flag,
     *          and rotation. This constructor is private - use probe() factory
     *          methods to create instances.
     * 
     * @param[in] bus            Pointer to initialized bus driver (ownership transferred)
     * @param[in] force_external Force compass to be treated as external
     * @param[in] rotation       Board rotation for compass mounting orientation
     * 
     * Source: libraries/AP_Compass/AP_Compass_HMC5843.cpp:95-101
     */
    AP_Compass_HMC5843(AP_HMC5843_BusDriver *bus,
                       bool force_external, enum Rotation rotation);

    /**
     * @brief Initialize the HMC5843/HMC5883L sensor
     * 
     * @details Performs complete sensor initialization sequence:
     *          1. Acquire bus semaphore
     *          2. Configure bus (auxiliary bus specific setup)
     *          3. Verify WHO_AM_I identification
     *          4. Perform factory calibration with positive bias test
     *          5. Configure sampling mode (75Hz, 1 sample averaging)
     *          6. Start continuous measurements
     *          7. Register compass instance with frontend
     *          8. Register periodic callback for 75Hz sampling
     *          
     *          Initialization may take 200-500ms due to calibration delays.
     * 
     * @return true if initialization successful, false on any failure
     * 
     * @note Sets high retry count (10) during init, reduces to 3 for runtime
     * @note Performs initial read() to populate first sample
     * 
     * Source: libraries/AP_Compass/AP_Compass_HMC5843.cpp:151-221
     */
    bool init();

    /**
     * @brief Verify sensor identity via WHO_AM_I register
     * 
     * @details Reads the 3-byte identification register at 0x0A and verifies
     *          the ASCII string "H43" which identifies HMC5843/HMC5883L devices.
     * 
     * @return true if correct ID detected, false if wrong ID or bus error
     * 
     * @note Bus semaphore must be held when calling this method
     * 
     * Source: libraries/AP_Compass/AP_Compass_HMC5843.cpp:330-343
     */
    bool _check_whoami();

    /**
     * @brief Perform factory calibration with positive bias test
     * 
     * @details Executes the HMC5883L positive bias self-test to verify sensor
     *          operation and determine per-axis scaling factors. The test applies
     *          a known magnetic field via internal current source and measures
     *          the sensor response.
     *          
     *          The calibration attempts up to 25 iterations and requires 5 good
     *          samples within expected range to pass. Expected field strength
     *          is approximately 700 counts per axis with 2.5 Gauss gain setting.
     *          
     *          Calibration compensates for:
     *          - Per-axis sensitivity variations
     *          - Manufacturing tolerance in sense coils
     *          - Temperature effects on sensitivity
     * 
     * @return true if calibration successful, false if sensor fails self-test
     * 
     * @note Calibration takes 50-100ms per attempt
     * @note Bus semaphore must be held when calling this method
     * @note Updates _scaling vector with per-axis correction factors
     * 
     * Source: libraries/AP_Compass/AP_Compass_HMC5843.cpp:345-436
     */
    bool _calibrate();

    /**
     * @brief Configure sensor sampling parameters
     * 
     * @details Configures the HMC5883L for continuous operation:
     *          - Sample rate: 75Hz
     *          - Sample averaging: 1 sample (no averaging)
     *          - Gain: 1.3 Gauss full scale (±810 milligauss)
     *          - Mode: Single measurement (triggered by _take_sample)
     *          - Temperature sensor: Enabled
     *          
     *          Gain scale factor: 1.0/1090 * 1000 = 0.917 milligauss/LSB
     * 
     * @return true if configuration successful, false on bus error
     * 
     * @note Bus semaphore must be held when calling this method
     * 
     * Source: libraries/AP_Compass/AP_Compass_HMC5843.cpp:272-286
     */
    bool _setup_sampling_mode();

    /**
     * @brief Periodic callback to read magnetometer samples at 75Hz
     * 
     * @details This method is registered as a periodic callback and executed at
     *          75Hz (every 13333 microseconds) by the HAL scheduler. It reads
     *          a single sample from the sensor and accumulates it in the frontend
     *          buffer for later retrieval by read().
     *          
     *          Sequence:
     *          1. Read current sample via _read_sample()
     *          2. Trigger next measurement via _take_sample()
     *          3. If read successful, scale and accumulate field vector
     *          4. Apply 90° yaw rotation for external compasses
     *          
     *          Sample accumulation allows the frontend to retrieve averaged data
     *          at lower rates (~10Hz) while maintaining high-rate measurements.
     * 
     * @note Called from scheduler context with bus semaphore already acquired
     * @note Accumulates up to 14 samples before read() must be called
     * @note Field values are in milligauss after gain_scale multiplication
     * 
     * @see _read_sample() for hardware read
     * @see _take_sample() for triggering next conversion
     * 
     * Source: libraries/AP_Compass/AP_Compass_HMC5843.cpp:228-252
     */
    void _timer();

    /**
     * @brief Read a single 3-axis magnetic field sample from sensor
     * 
     * @details Reads 6 bytes from data output registers (X_MSB, Z_MSB, Y_MSB order)
     *          and converts to signed 16-bit values. The HMC5843 returns data in
     *          XZY order which is reordered to XYZ and signs adjusted for ArduPilot
     *          coordinate frame conventions.
     *          
     *          Invalid data is detected by checking for -4096 (0xF000) which
     *          indicates data not ready or sensor overflow.
     * 
     * @return true if valid sample read, false on bus error or invalid data
     * 
     * @note Bus semaphore must be held when calling this method
     * @note Raw values are in sensor LSB units (not yet scaled to milligauss)
     * @note Values stored in _mag_x, _mag_y, _mag_z member variables
     * 
     * Source: libraries/AP_Compass/AP_Compass_HMC5843.cpp:291-318
     */
    bool _read_sample();

    /**
     * @brief Trigger a new single-shot measurement
     * 
     * @details Writes to the mode register to request a new measurement. The
     *          HMC5883L is configured in single-shot mode where each measurement
     *          must be explicitly triggered. After conversion completes (~6ms at
     *          75Hz rate), data is available in the output registers.
     * 
     * @note Bus semaphore must be held when calling this method
     * @note Measurement conversion time: ~13ms
     * 
     * Source: libraries/AP_Compass/AP_Compass_HMC5843.cpp:324-328
     */
    void _take_sample();

    AP_HMC5843_BusDriver *_bus;  ///< Bus driver for I2C or auxiliary bus communication

    Vector3f _scaling;           ///< Per-axis calibration scaling factors from factory cal
    float _gain_scale;           ///< Conversion factor from LSB to milligauss (0.917 mG/LSB)

    int16_t _mag_x;              ///< Last X-axis reading in raw LSB units
    int16_t _mag_y;              ///< Last Y-axis reading in raw LSB units  
    int16_t _mag_z;              ///< Last Z-axis reading in raw LSB units

    uint8_t _compass_instance;   ///< Frontend compass instance number

    enum Rotation _rotation;     ///< Board rotation for compass mounting orientation
    
    bool _initialised:1;         ///< True after successful init() completion
    bool _force_external:1;      ///< Force compass to be treated as external
};

/**
 * @class AP_HMC5843_BusDriver
 * @brief Abstract bus interface for HMC5843 communication
 * 
 * @details This abstract base class provides a hardware abstraction layer for
 *          HMC5843/HMC5883L communication, allowing the same compass driver to
 *          work with different bus types:
 *          
 *          - Direct I2C via AP_HAL::Device (AP_HMC5843_BusDriver_HALDevice)
 *          - IMU auxiliary I2C bus (AP_HMC5843_BusDriver_Auxiliary)
 *          
 *          The abstraction isolates bus-specific details like register access,
 *          semaphore management, and periodic callback registration from the
 *          main compass driver logic. This design pattern is common in ArduPilot
 *          for sensors that can be connected via multiple bus interfaces.
 *          
 *          Concrete implementations must handle:
 *          - Thread-safe register read/write operations
 *          - Semaphore management for bus access
 *          - Device identification (bus ID)
 *          - Periodic callback scheduling for sample collection
 * 
 * @note All bus operations must be thread-safe via semaphore protection
 * @note Implementations should minimize bus transaction overhead
 * 
 * @see AP_HMC5843_BusDriver_HALDevice for direct I2C implementation
 * @see AP_HMC5843_BusDriver_Auxiliary for auxiliary bus implementation
 */
class AP_HMC5843_BusDriver
{
public:
    virtual ~AP_HMC5843_BusDriver() { }

    /**
     * @brief Read multiple bytes from consecutive registers
     * 
     * @param[in]  reg  Starting register address
     * @param[out] buf  Buffer to store read data
     * @param[in]  size Number of bytes to read
     * 
     * @return true if read successful, false on bus error
     * 
     * @note Bus semaphore must be held before calling
     */
    virtual bool block_read(uint8_t reg, uint8_t *buf, uint32_t size) = 0;

    /**
     * @brief Read a single register
     * 
     * @param[in]  reg Starting register address
     * @param[out] val Pointer to store read value
     * 
     * @return true if read successful, false on bus error
     * 
     * @note Bus semaphore must be held before calling
     */
    virtual bool register_read(uint8_t reg, uint8_t *val) = 0;

    /**
     * @brief Write a single register
     * 
     * @param[in] reg Register address
     * @param[in] val Value to write
     * 
     * @return true if write successful, false on bus error
     * 
     * @note Bus semaphore must be held before calling
     */
    virtual bool register_write(uint8_t reg, uint8_t val) = 0;

    /**
     * @brief Get the bus semaphore for thread-safe access
     * 
     * @return Pointer to semaphore, or nullptr if bus doesn't support semaphores
     * 
     * @note Caller must acquire semaphore before bus operations
     */
    virtual AP_HAL::Semaphore *get_semaphore() = 0;

    /**
     * @brief Perform bus-specific configuration during initialization
     * 
     * @details Optional hook for bus-specific setup (e.g., auxiliary bus
     *          passthrough mode). Default implementation returns success.
     * 
     * @return true if configuration successful, false on error
     */
    virtual bool configure() { return true; }

    /**
     * @brief Start continuous measurements on the bus
     * 
     * @details Optional hook for bus-specific measurement start (e.g.,
     *          auxiliary bus sample enable). Default implementation returns success.
     * 
     * @return true if measurements started, false on error
     */
    virtual bool start_measurements() { return true; }

    /**
     * @brief Register a periodic callback for sample collection
     * 
     * @param[in] period_usec Period in microseconds between callbacks
     * @param[in] cb          Callback function to execute periodically
     * 
     * @return Handle to the registered callback
     * 
     * @note Callback is executed from scheduler context with semaphore held
     */
    virtual AP_HAL::Device::PeriodicHandle register_periodic_callback(uint32_t, AP_HAL::Device::PeriodicCb) = 0;

    /**
     * @brief Set the device type identifier
     * 
     * @param[in] devtype Device type code (e.g., DEVTYPE_HMC5883)
     * 
     * @note Used for device identification in logs and parameter system
     */
    virtual void set_device_type(uint8_t devtype) = 0;

    /**
     * @brief Get the 24-bit bus identifier
     * 
     * @details Returns a unique identifier encoding bus type, bus number,
     *          and device address. Used for compass instance identification
     *          and parameter association.
     * 
     * @return 24-bit bus ID
     */
    virtual uint32_t get_bus_id(void) const = 0;

    /**
     * @brief Set the number of bus transaction retries
     * 
     * @param[in] retries Number of retry attempts on bus errors
     * 
     * @note Default implementation does nothing (no retry support)
     * @note Typically set to 10 during init, 3 during runtime
     */
    virtual void set_retries(uint8_t retries) {}
};

/**
 * @class AP_HMC5843_BusDriver_HALDevice
 * @brief Bus driver implementation for direct I2C or SPI via AP_HAL::Device
 * 
 * @details This concrete bus driver implements the AP_HMC5843_BusDriver interface
 *          for compasses connected directly to the autopilot via I2C or SPI.
 *          It wraps an AP_HAL::Device object and delegates all bus operations
 *          to the HAL device interface.
 *          
 *          This is the most common connection method for external compasses
 *          connected to the autopilot's I2C ports. The HAL device handles
 *          platform-specific I2C/SPI implementation details and provides
 *          automatic retry logic and error handling.
 *          
 *          Typical usage: External GPS+compass modules, standalone compass boards
 * 
 * @note The wrapped device is owned by this class and destroyed on deletion
 * @note All operations use the HAL device's built-in semaphore for thread safety
 * 
 * @see AP_HAL::Device for device interface details
 * @see probe() for usage in device detection
 */
class AP_HMC5843_BusDriver_HALDevice : public AP_HMC5843_BusDriver
{
public:
    /**
     * @brief Construct HAL device bus driver
     * 
     * @param[in] dev OwnPtr to initialized HAL device (ownership transferred)
     * 
     * @note dev must be initialized and at correct I2C address or SPI chip select
     */
    AP_HMC5843_BusDriver_HALDevice(AP_HAL::OwnPtr<AP_HAL::Device> dev);

    /**
     * @brief Read multiple consecutive registers via HAL device
     * 
     * @param[in]  reg  Starting register address
     * @param[out] buf  Buffer to store read data
     * @param[in]  size Number of bytes to read
     * 
     * @return true if read successful, false on bus error
     */
    bool block_read(uint8_t reg, uint8_t *buf, uint32_t size) override;

    /**
     * @brief Read single register via HAL device
     * 
     * @param[in]  reg Starting register address
     * @param[out] val Pointer to store read value
     * 
     * @return true if read successful, false on bus error
     */
    bool register_read(uint8_t reg, uint8_t *val) override;

    /**
     * @brief Write single register via HAL device
     * 
     * @param[in] reg Register address
     * @param[in] val Value to write
     * 
     * @return true if write successful, false on bus error
     */
    bool register_write(uint8_t reg, uint8_t val) override;

    /**
     * @brief Get HAL device semaphore
     * 
     * @return Pointer to device semaphore
     */
    AP_HAL::Semaphore *get_semaphore() override;

    /**
     * @brief Register periodic callback via HAL device scheduler
     * 
     * @param[in] period_usec Period in microseconds between callbacks
     * @param[in] cb          Callback function to execute
     * 
     * @return Handle to registered callback
     */
    AP_HAL::Device::PeriodicHandle register_periodic_callback(uint32_t period_usec, AP_HAL::Device::PeriodicCb) override;

    /**
     * @brief Set device type identifier in HAL device
     * 
     * @param[in] devtype Device type code
     */
    void set_device_type(uint8_t devtype) override {
        _dev->set_device_type(devtype);
    }

    /**
     * @brief Get 24-bit bus identifier from HAL device
     * 
     * @return 24-bit bus ID encoding bus type, number, and address
     */
    uint32_t get_bus_id(void) const override {
        return _dev->get_bus_id();
    }

    /**
     * @brief Set transaction retry count in HAL device
     * 
     * @param[in] retries Number of retry attempts on bus errors
     */
    void set_retries(uint8_t retries) override {
        return _dev->set_retries(retries);
    }
    
private:
    AP_HAL::OwnPtr<AP_HAL::Device> _dev;  ///< Owned HAL device for bus communication
};

#if AP_INERTIALSENSOR_ENABLED
/**
 * @class AP_HMC5843_BusDriver_Auxiliary
 * @brief Bus driver implementation for IMU auxiliary I2C bus
 * 
 * @details This concrete bus driver implements the AP_HMC5843_BusDriver interface
 *          for compasses connected to an IMU's auxiliary I2C port. IMUs like the
 *          MPU6000/MPU6050/MPU9250 include an auxiliary I2C master that can read
 *          external sensors and buffer the data internally.
 *          
 *          Benefits of auxiliary bus connection:
 *          - Reduces main I2C bus traffic
 *          - IMU can read compass synchronously with IMU samples
 *          - Automatic sample synchronization for sensor fusion
 *          - Frees main I2C bus for other peripherals
 *          
 *          The auxiliary bus interface provides passthrough mode for initialization
 *          and configuration, then switches to automatic read mode where the IMU
 *          periodically reads compass registers and buffers the data.
 *          
 *          Typical usage: Integrated IMU+compass modules, internal compass on
 *          flight controllers with MPU6000/9250
 * 
 * @note Requires AP_InertialSensor with auxiliary bus support
 * @note Compass samples are synchronized with IMU sample rate
 * @note Some auxiliary bus operations have higher latency than direct I2C
 * 
 * @see AP_InertialSensor::get_auxiliary_bus() for obtaining auxiliary bus
 * @see AuxiliaryBus for auxiliary bus interface documentation
 * @see probe_mpu6000() for usage in device detection
 */
class AP_HMC5843_BusDriver_Auxiliary : public AP_HMC5843_BusDriver
{
public:
    /**
     * @brief Construct auxiliary bus driver for compass on IMU auxiliary port
     * 
     * @param[in] ins        Reference to InertialSensor singleton
     * @param[in] backend_id IMU backend ID (e.g., HAL_INS_MPU60XX_SPI)
     * @param[in] addr       I2C address of compass on auxiliary bus
     * 
     * @note Auxiliary bus is requested from the specified IMU backend
     */
    AP_HMC5843_BusDriver_Auxiliary(AP_InertialSensor &ins, uint8_t backend_id,
                                   uint8_t addr);

    /**
     * @brief Destructor releases auxiliary bus resources
     */
    virtual ~AP_HMC5843_BusDriver_Auxiliary();

    /**
     * @brief Read multiple consecutive registers via auxiliary bus
     * 
     * @param[in]  reg  Starting register address
     * @param[out] buf  Buffer to store read data
     * @param[in]  size Number of bytes to read
     * 
     * @return true if read successful, false on bus error
     * 
     * @note Uses passthrough mode, may be slower than direct I2C
     */
    bool block_read(uint8_t reg, uint8_t *buf, uint32_t size) override;

    /**
     * @brief Read single register via auxiliary bus
     * 
     * @param[in]  reg Starting register address
     * @param[out] val Pointer to store read value
     * 
     * @return true if read successful, false on bus error
     */
    bool register_read(uint8_t reg, uint8_t *val) override;

    /**
     * @brief Write single register via auxiliary bus
     * 
     * @param[in] reg Register address
     * @param[in] val Value to write
     * 
     * @return true if write successful, false on bus error
     */
    bool register_write(uint8_t reg, uint8_t val) override;

    /**
     * @brief Get auxiliary bus semaphore
     * 
     * @return Pointer to auxiliary bus semaphore
     * 
     * @note Semaphore is shared with IMU operations
     */
    AP_HAL::Semaphore *get_semaphore() override;

    /**
     * @brief Configure auxiliary bus for compass access
     * 
     * @details Requests auxiliary bus from IMU and configures passthrough
     *          mode for compass initialization. Must be called before any
     *          register access.
     * 
     * @return true if auxiliary bus configured successfully
     */
    bool configure() override;

    /**
     * @brief Start automatic compass measurements via auxiliary bus
     * 
     * @details Configures the IMU to automatically read compass data registers
     *          and buffer the results. After this call, compass data is available
     *          via the auxiliary bus read interface.
     * 
     * @return true if measurements started successfully
     */
    bool start_measurements() override;

    /**
     * @brief Register periodic callback via auxiliary bus
     * 
     * @param[in] period_usec Period in microseconds between callbacks
     * @param[in] cb          Callback function to execute
     * 
     * @return Handle to registered callback
     * 
     * @note Callback rate synchronized with IMU sample rate
     */
    AP_HAL::Device::PeriodicHandle register_periodic_callback(uint32_t period_usec, AP_HAL::Device::PeriodicCb) override;

    /**
     * @brief Set device type identifier for compass
     * 
     * @param[in] devtype Device type code
     */
    void set_device_type(uint8_t devtype) override;

    /**
     * @brief Get 24-bit bus identifier for auxiliary bus compass
     * 
     * @return 24-bit bus ID encoding auxiliary bus information
     */
    uint32_t get_bus_id(void) const override;
    
private:
    AuxiliaryBus *_bus;          ///< Pointer to IMU auxiliary bus interface
    AuxiliaryBusSlave *_slave;   ///< Slave device handle on auxiliary bus
    bool _started;               ///< True after start_measurements() called
};
#endif  // AP_INERTIALSENSOR_ENABLED

#endif // AP_COMPASS_HMC5843_ENABLED
