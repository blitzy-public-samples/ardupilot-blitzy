/**
 * @file AP_Compass_AK8963.h
 * @brief AK8963 3-axis magnetometer driver
 * 
 * @details This driver supports the AKM AK8963 magnetometer, commonly found as
 *          part of the MPU9250 9-axis IMU but also available as a standalone I2C device.
 *          The AK8963 provides 3-axis magnetic field sensing with 16-bit ADC resolution.
 *          
 *          The driver supports two connection modes:
 *          1. Standalone I2C connection - Direct I2C bus communication
 *          2. MPU auxiliary bus - Connected via MPU9250's auxiliary I2C master interface
 *          
 *          Key features:
 *          - Factory calibration data (ASA - Axis Sensitivity Adjustment) loaded from ROM
 *          - 16-bit ADC resolution providing 0.15 µT/LSB sensitivity
 *          - Continuous measurement mode at 100Hz
 *          - Overflow detection and validation
 *          - Output in milligauss units
 *          
 *          Sensor lifecycle: probe → init → register_periodic_callback → read (100Hz)
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 * 
 * Source: libraries/AP_Compass/AP_Compass_AK8963.h, libraries/AP_Compass/AP_Compass_AK8963.cpp
 */

#pragma once

#include "AP_Compass_config.h"

#if AP_COMPASS_AK8963_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_HAL/SPIDevice.h>
#include <AP_Math/AP_Math.h>

#include "AP_Compass.h"
#include "AP_Compass_Backend.h"

class AuxiliaryBus;
class AuxiliaryBusSlave;
class AP_InertialSensor;
class AP_AK8963_BusDriver;

/**
 * @class AP_Compass_AK8963
 * @brief Driver for AKM AK8963 3-axis magnetometer
 * 
 * @details This class implements the ArduPilot compass backend for the AK8963 magnetometer.
 *          The AK8963 is commonly integrated into the MPU9250 9-DOF IMU but can also be
 *          used as a standalone I2C device.
 *          
 *          Driver Initialization Sequence:
 *          1. probe() - Static factory method creates bus driver and sensor instance
 *          2. init() - Performs sensor initialization:
 *             - Configures I2C bus or MPU auxiliary bus
 *             - Verifies device ID (0x48)
 *             - Loads factory calibration (ASA values from FUSE ROM)
 *             - Configures continuous measurement mode 2 (100Hz, 16-bit)
 *             - Registers periodic callback for 100Hz sampling
 *          3. _update() - Called at 100Hz via periodic callback:
 *             - Reads raw magnetic field data (HXL-HZH registers + ST2 status)
 *             - Applies factory sensitivity adjustment (ASA calibration)
 *             - Applies ADC resolution scaling (0.15 µT/LSB)
 *             - Converts to milligauss and accumulates samples
 *          4. read() - Called by compass library:
 *             - Drains accumulated samples for delivery to EKF
 *          
 *          Factory Calibration (ASA):
 *          The AK8963 stores per-axis factory calibration data in FUSE ROM (registers ASAX-ASAZ).
 *          These sensitivity adjustment values compensate for manufacturing variations.
 *          Formula: Adjusted = Raw × ((ASA - 128) / 256 + 1)
 *          ASA values typically range from 0 to 255, centered around 128.
 *          
 *          Magnetic Field Conversion:
 *          Raw ADC → Factory ASA adjustment → ADC resolution (0.15 µT/LSB) → Milligauss (×10)
 *          Output range: ±4912 µT (±49.12 milligauss) with 16-bit resolution
 *          
 *          Bus Driver Abstraction:
 *          Communication is abstracted through AP_AK8963_BusDriver interface:
 *          - AP_AK8963_BusDriver_HALDevice: Direct I2C via HAL device
 *          - AP_AK8963_BusDriver_Auxiliary: Via MPU9250 auxiliary I2C master
 * 
 * @note Measurement rate is 100Hz in continuous mode 2
 * @warning When using MPU auxiliary bus, potential for I2C bus contention if MPU auxiliary
 *          master and host I2C transactions overlap. The auxiliary bus configuration must
 *          be coordinated with IMU driver to avoid conflicts.
 * 
 * Source: libraries/AP_Compass/AP_Compass_AK8963.cpp:127-180 (init), 205-231 (_update)
 */
class AP_Compass_AK8963 : public AP_Compass_Backend
{
public:
    /**
     * @brief Probe for AK8963 magnetometer on standalone I2C bus
     * 
     * @details Factory method to detect and initialize AK8963 connected directly to I2C bus.
     *          Creates HAL device bus driver, verifies device presence, and initializes sensor.
     *          
     *          Probe sequence:
     *          1. Takes ownership of I2C device
     *          2. Creates AP_AK8963_BusDriver_HALDevice wrapper
     *          3. Instantiates AP_Compass_AK8963 sensor object
     *          4. Calls init() to verify ID and configure sensor
     *          5. Returns backend pointer on success, nullptr on failure
     * 
     * @param[in] dev I2C device handle (ownership transferred to driver on success)
     * @param[in] rotation Board rotation to apply for sensor orientation correction
     * 
     * @return AP_Compass_Backend* Pointer to initialized compass backend, or nullptr if probe fails
     * 
     * @note Takes ownership of dev - caller must not use device after calling
     * @note Device is deleted automatically if probe fails
     * 
     * Source: libraries/AP_Compass/AP_Compass_AK8963.cpp:66-84
     */
    static AP_Compass_Backend *probe(AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev,
                                     enum Rotation rotation);

    /**
     * @brief Probe for AK8963 on MPU9250 auxiliary I2C bus (MPU on I2C)
     * 
     * @details Factory method for AK8963 connected via MPU9250's auxiliary I2C master interface
     *          when the MPU9250 itself is on the system I2C bus. This is the common configuration
     *          for MPU9250 modules with integrated AK8963.
     *          
     *          The MPU9250 acts as I2C master to the AK8963 on an auxiliary bus, with the host
     *          accessing AK8963 registers through MPU9250 passthrough or slave registers.
     *          
     *          This method triggers IMU backend detection to ensure MPU9250 auxiliary bus is
     *          configured before attempting AK8963 communication, then delegates to standard probe().
     * 
     * @param[in] dev I2C device handle for MPU9250 (ownership transferred)
     * @param[in] rotation Board rotation to apply for sensor orientation
     * 
     * @return AP_Compass_Backend* Pointer to initialized compass backend, or nullptr if probe fails
     * 
     * @note Requires AP_InertialSensor to detect and configure MPU9250 first
     * @warning Auxiliary bus configuration must be coordinated with MPU9250 driver to avoid
     *          I2C conflicts between host and MPU auxiliary master transactions
     * 
     * Source: libraries/AP_Compass/AP_Compass_AK8963.cpp:86-100
     */
    static AP_Compass_Backend *probe_mpu9250(AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev,
                                             enum Rotation rotation);

    /**
     * @brief Probe for AK8963 on MPU9250 auxiliary I2C bus (MPU on SPI)
     * 
     * @details Factory method for AK8963 connected via MPU9250's auxiliary I2C master when
     *          the MPU9250 itself is connected via SPI. Common on flight controllers with
     *          SPI-connected IMUs for higher data rates.
     *          
     *          Uses AP_AK8963_BusDriver_Auxiliary to communicate with AK8963 through
     *          the MPU9250's auxiliary bus slave registers accessed over SPI.
     *          
     *          Probe sequence:
     *          1. Creates auxiliary bus driver for specified MPU9250 instance
     *          2. Instantiates AP_Compass_AK8963 with auxiliary bus driver
     *          3. Calls init() to configure sensor
     *          4. Returns backend on success
     * 
     * @param[in] mpu9250_instance MPU9250 backend instance number (for multi-IMU systems)
     * @param[in] rotation Board rotation to apply for sensor orientation
     * 
     * @return AP_Compass_Backend* Pointer to initialized compass backend, or nullptr if probe fails
     * 
     * @note mpu9250_instance identifies which MPU9250 backend manages this AK8963
     * @note AK8963 I2C address is fixed at 0x0C on auxiliary bus
     * 
     * Source: libraries/AP_Compass/AP_Compass_AK8963.cpp:102-125
     */
    static AP_Compass_Backend *probe_mpu9250(uint8_t mpu9250_instance,
                                             enum Rotation rotation);

    /**
     * @brief Driver identification name
     * 
     * @details Human-readable name used for logging and identification in compass library
     */
    static constexpr const char *name = "AK8963";

    /**
     * @brief Destructor - releases bus driver resources
     * 
     * @details Deletes the bus driver instance, which in turn releases I2C device or
     *          auxiliary bus slave resources
     */
    virtual ~AP_Compass_AK8963();

    /**
     * @brief Read accumulated magnetometer samples
     * 
     * @details Called by compass library to retrieve magnetic field data for delivery to EKF.
     *          Drains samples accumulated by _update() callback running at 100Hz.
     *          
     *          This method is called by the main compass library at scheduler rate
     *          (typically 50-400Hz depending on vehicle). It does not directly read hardware;
     *          instead it retrieves samples accumulated by the periodic _update() callback.
     * 
     * @note Returns immediately if sensor not initialized
     * @note Actual hardware reads occur in _update() at 100Hz
     * 
     * Source: libraries/AP_Compass/AP_Compass_AK8963.cpp:182-189
     */
    void read() override;

    /**
     * @brief Hardware register structure for magnetic field samples
     * 
     * @details Packed structure matching AK8963 output register layout (HXL-ST2).
     *          Read in a single I2C block transaction to ensure atomic sample capture.
     *          
     *          Register mapping:
     *          - val[0] = HXL/HXH (X-axis magnetic field, 16-bit signed)
     *          - val[1] = HYL/HYH (Y-axis magnetic field, 16-bit signed)
     *          - val[2] = HZL/HZH (Z-axis magnetic field, 16-bit signed)
     *          - st2 = ST2 status register (bit 3 = overflow flag)
     *          
     *          Raw ADC values before calibration adjustments.
     * 
     * @note Must be public for bus driver access during register read operations
     * @note Structure is packed to match hardware register layout exactly
     * 
     * Source: libraries/AP_Compass/AP_Compass_AK8963.cpp:210 (block_read usage)
     */
    struct PACKED sample_regs {
        int16_t val[3];  ///< Raw ADC values for X, Y, Z axes (16-bit signed)
        uint8_t st2;     ///< Status register 2 (bit 3 indicates magnetic overflow)
    };

private:
    /**
     * @brief Private constructor - use probe() factory methods
     * 
     * @details Constructs sensor instance with specified bus driver and rotation.
     *          Private to enforce probe pattern for proper initialization.
     * 
     * @param[in] bus Bus driver instance (takes ownership)
     * @param[in] rotation Board rotation for sensor orientation correction
     */
    AP_Compass_AK8963(AP_AK8963_BusDriver *bus,
                      enum Rotation rotation);

    /**
     * @brief Initialize AK8963 sensor
     * 
     * @details Performs complete sensor initialization sequence:
     *          1. Configures bus (auxiliary bus setup if using MPU9250)
     *          2. Verifies device ID (reads WIA register, expects 0x48)
     *          3. Loads factory calibration from FUSE ROM (ASA values)
     *          4. Configures continuous measurement mode 2 with 16-bit ADC
     *          5. Starts measurements on auxiliary bus if applicable
     *          6. Registers compass instance with frontend
     *          7. Registers periodic callback for 100Hz sampling
     * 
     * @return true if initialization successful, false on any failure
     * 
     * @note Called by probe() methods during sensor detection
     * @note Initialization failures print diagnostic messages to console
     * 
     * Source: libraries/AP_Compass/AP_Compass_AK8963.cpp:127-180
     */
    bool init();

    /**
     * @brief Apply factory sensitivity adjustment (ASA calibration)
     * 
     * @details Applies per-axis factory calibration loaded from FUSE ROM during init().
     *          The AK8963 stores sensitivity adjustment values in ASAX, ASAY, ASAZ registers
     *          that compensate for manufacturing variations.
     *          
     *          Formula: field[i] *= ((ASA[i] - 128) / 256 + 1)
     *          
     *          ASA values typically range 0-255, centered at 128 (no adjustment).
     *          Adjustment typically ±10% to correct for magnetic circuit variations.
     * 
     * @param[in,out] field Magnetic field vector to adjust (modified in place)
     * 
     * @note Called during _update() after reading raw ADC values
     * @note ASA values loaded once during _calibrate() at initialization
     * 
     * Source: libraries/AP_Compass/AP_Compass_AK8963.cpp:198-203
     */
    void _make_factory_sensitivity_adjustment(Vector3f &field) const;

    /**
     * @brief Apply ADC resolution sensitivity adjustment
     * 
     * @details Converts raw 16-bit ADC values to magnetic flux density in microtesla.
     *          In 16-bit mode, the AK8963 ADC resolution is 0.15 µT per LSB.
     *          
     *          This is a fixed scaling factor independent of device calibration.
     * 
     * @param[in,out] field Magnetic field vector to scale (modified in place)
     * 
     * @note ADC resolution: 0.15 µT/LSB in 16-bit mode
     * @note Called after factory sensitivity adjustment
     * 
     * Source: libraries/AP_Compass/AP_Compass_AK8963.cpp:191-196
     */
    void _make_adc_sensitivity_adjustment(Vector3f &field) const;

    /**
     * @brief Reset AK8963 sensor
     * 
     * @details Writes soft reset command to CNTL2 register to return sensor to
     *          power-on-reset state. Clears all settings and prepares for reinitialization.
     * 
     * @return true if reset command written successfully
     * 
     * @note Not currently called in normal operation
     * @note After reset, sensor returns to power-down mode
     * 
     * Source: libraries/AP_Compass/AP_Compass_AK8963.cpp:252-255
     */
    bool _reset();

    /**
     * @brief Configure sensor measurement mode
     * 
     * @details Writes to CNTL1 register to configure:
     *          - Continuous measurement mode 2 (100Hz output rate)
     *          - 16-bit ADC resolution (0.15 µT/LSB sensitivity)
     *          
     *          Mode configuration:
     *          CNTL1 = 0x16 = AK8963_CONTINUOUS_MODE2 | AK8963_16BIT_ADC
     * 
     * @return true if mode register written successfully
     * 
     * @note Called by init() after loading calibration data
     * @note Continuous mode 2 provides 100Hz update rate vs 8Hz in mode 1
     * 
     * Source: libraries/AP_Compass/AP_Compass_AK8963.cpp:248-250
     */
    bool _setup_mode();

    /**
     * @brief Verify AK8963 device ID
     * 
     * @details Reads WIA (Who I Am) register and verifies it contains expected
     *          device ID value of 0x48. Retries up to 5 times to handle I2C
     *          startup timing issues.
     * 
     * @return true if device ID matches 0x48, false after 5 failed attempts
     * 
     * @note Called by init() to verify sensor presence before configuration
     * @note Multiple retries handle potential I2C bus initialization delays
     * 
     * Source: libraries/AP_Compass/AP_Compass_AK8963.cpp:233-246
     */
    bool _check_id();

    /**
     * @brief Load factory calibration data from FUSE ROM
     * 
     * @details Reads axis sensitivity adjustment (ASA) values from FUSE ROM registers.
     *          These per-device calibration values compensate for magnetic circuit
     *          variations during manufacturing.
     *          
     *          Calibration sequence:
     *          1. Enter FUSE access mode (write 0x1F to CNTL1)
     *          2. Read ASAX, ASAY, ASAZ registers (0x10-0x12)
     *          3. Convert raw ASA values to adjustment multipliers
     *          4. Store in _magnetometer_ASA[3] for runtime use
     *          
     *          Conversion formula: ASA_multiplier = ((raw_ASA - 128) / 256 + 1)
     * 
     * @return true (always succeeds - FUSE ROM read cannot fail once in FUSE mode)
     * 
     * @note Called by init() before configuring measurement mode
     * @note FUSE ROM values are factory-programmed and read-only
     * @note ASA values stored in _magnetometer_ASA[] for subsequent measurements
     * 
     * Source: libraries/AP_Compass/AP_Compass_AK8963.cpp:258-273
     */
    bool _calibrate();

    /**
     * @brief Periodic update callback for magnetic field sampling
     * 
     * @details Called at 100Hz by HAL scheduler to read magnetic field data from sensor.
     *          Performs complete sample acquisition and processing:
     *          
     *          1. Block read HXL-ST2 registers (7 bytes) atomically
     *          2. Check ST2 overflow flag (bit 3) - discard sample if set
     *          3. Validate non-zero reading (sensor health check)
     *          4. Apply factory sensitivity adjustment (ASA calibration)
     *          5. Apply ADC resolution scaling (0.15 µT/LSB)
     *          6. Convert to milligauss (×10 scale factor)
     *          7. Accumulate sample with 10ms delta for averaging
     *          
     *          Samples are accumulated and retrieved by read() method.
     * 
     * @note Registered during init() for 100Hz callback (10ms period)
     * @note Samples discarded on overflow or all-zero readings
     * @note Accumulated samples averaged by compass library before EKF delivery
     * 
     * Source: libraries/AP_Compass/AP_Compass_AK8963.cpp:205-231
     */
    void _update();

    /**
     * @brief Bus driver abstraction for I2C communication
     * 
     * @details Pointer to bus driver instance providing hardware abstraction.
     *          Either AP_AK8963_BusDriver_HALDevice (direct I2C) or
     *          AP_AK8963_BusDriver_Auxiliary (via MPU9250 auxiliary bus).
     *          
     *          Owned by this compass instance - deleted in destructor.
     */
    AP_AK8963_BusDriver *_bus;

    /**
     * @brief Factory calibration sensitivity adjustment values
     * 
     * @details Per-axis sensitivity adjustment (ASA) multipliers loaded from
     *          FUSE ROM during initialization. Applied to raw magnetic field
     *          readings to compensate for manufacturing variations.
     *          
     *          Values derived from ASAX/ASAY/ASAZ FUSE ROM registers using:
     *          ASA[i] = ((FUSE_ROM[i] - 128) / 256 + 1)
     *          
     *          Typical range: 0.9 to 1.1 (±10% adjustment)
     * 
     * @note Loaded once during _calibrate() at initialization
     * @note Applied to every sample in _update()
     * 
     * Source: libraries/AP_Compass/AP_Compass_AK8963.cpp:258-273 (_calibrate),
     *         198-203 (_make_factory_sensitivity_adjustment)
     */
    float _magnetometer_ASA[3] {0, 0, 0};

    /**
     * @brief Compass instance ID in frontend
     * 
     * @details Instance identifier assigned by compass frontend during registration.
     *          Used to accumulate and drain samples for this specific compass.
     */
    uint8_t _compass_instance;

    /**
     * @brief Initialization completion flag
     * 
     * @details Set to true after successful init() completion.
     *          Checked by read() to prevent operations before sensor ready.
     */
    bool _initialized;

    /**
     * @brief Board rotation for sensor orientation correction
     * 
     * @details Rotation transformation applied to compensate for physical sensor
     *          mounting orientation relative to vehicle body frame.
     *          Set during probe and registered with compass frontend.
     */
    enum Rotation _rotation;
};

/**
 * @class AP_AK8963_BusDriver
 * @brief Abstract bus driver interface for AK8963 communication
 * 
 * @details Hardware abstraction layer providing uniform interface for AK8963 communication
 *          regardless of physical connection method. Supports two connection topologies:
 *          
 *          1. Direct I2C Connection (AP_AK8963_BusDriver_HALDevice):
 *             - Standard I2C bus communication via HAL I2CDevice
 *             - Used for standalone AK8963 or MPU9250 on I2C
 *             
 *          2. MPU Auxiliary Bus (AP_AK8963_BusDriver_Auxiliary):
 *             - Communication via MPU9250's auxiliary I2C master interface
 *             - AK8963 registers accessed through MPU slave registers
 *             - Used when MPU9250 is on SPI for higher IMU bandwidth
 *          
 *          This abstraction allows the compass driver to operate identically
 *          regardless of underlying bus topology.
 * 
 * @note All methods are pure virtual except configure() and start_measurements()
 * @note Implementations must provide thread-safe bus access via semaphores
 */
class AP_AK8963_BusDriver
{
public:
    /**
     * @brief Virtual destructor for proper cleanup of derived classes
     */
    virtual ~AP_AK8963_BusDriver() { }

    /**
     * @brief Read multiple consecutive registers
     * 
     * @details Performs block read of consecutive AK8963 registers starting at specified address.
     *          Used for atomic reading of magnetic field samples (HXL-ST2, 7 bytes).
     * 
     * @param[in] reg Starting register address
     * @param[out] buf Buffer to receive read data
     * @param[in] size Number of bytes to read
     * 
     * @return true if read successful, false on I2C error
     * 
     * @note Must be atomic to ensure consistent sample capture
     */
    virtual bool block_read(uint8_t reg, uint8_t *buf, uint32_t size) = 0;

    /**
     * @brief Read single register
     * 
     * @details Reads one byte from specified AK8963 register address.
     *          Used for status and identification registers.
     * 
     * @param[in] reg Register address to read
     * @param[out] val Pointer to receive register value
     * 
     * @return true if read successful, false on I2C error
     */
    virtual bool register_read(uint8_t reg, uint8_t *val) = 0;

    /**
     * @brief Write single register
     * 
     * @details Writes one byte to specified AK8963 register address.
     *          Used for configuration and control registers.
     * 
     * @param[in] reg Register address to write
     * @param[in] val Value to write to register
     * 
     * @return true if write successful, false on I2C error
     */
    virtual bool register_write(uint8_t reg, uint8_t val) = 0;

    /**
     * @brief Get bus access semaphore
     * 
     * @details Returns semaphore for thread-safe bus access coordination.
     *          Caller must acquire semaphore before register operations.
     * 
     * @return AP_HAL::Semaphore* Pointer to bus semaphore
     * 
     * @note Critical for multi-threaded access to shared I2C bus
     */
    virtual AP_HAL::Semaphore  *get_semaphore() = 0;

    /**
     * @brief Configure bus for AK8963 communication
     * 
     * @details Performs bus-specific configuration before sensor initialization.
     *          For auxiliary bus, sets up MPU9250 auxiliary I2C master.
     *          For direct I2C, typically no configuration needed.
     * 
     * @return true if configuration successful, false on error
     * 
     * @note Default implementation returns true (no configuration needed)
     * @note Overridden by auxiliary bus driver for MPU configuration
     */
    virtual bool configure() { return true; }

    /**
     * @brief Start continuous measurement data flow
     * 
     * @details Enables measurement data acquisition on the bus.
     *          For auxiliary bus, starts MPU auxiliary I2C master sampling.
     *          For direct I2C, typically no action needed.
     * 
     * @return true if measurements started successfully
     * 
     * @note Default implementation returns true (no start needed)
     * @note Overridden by auxiliary bus driver to enable MPU sampling
     */
    virtual bool start_measurements() { return true; }

    /**
     * @brief Register periodic callback for sensor sampling
     * 
     * @details Registers callback function to be invoked periodically by HAL scheduler.
     *          Used to schedule _update() method at 100Hz for magnetic field sampling.
     * 
     * @param[in] period_usec Callback period in microseconds (10000 = 100Hz)
     * @param[in] cb Callback function to invoke
     * 
     * @return AP_HAL::Device::PeriodicHandle Handle to registered callback
     * 
     * @note Callback runs in interrupt or scheduler context - must be efficient
     */
    virtual AP_HAL::Device::PeriodicHandle register_periodic_callback(uint32_t, AP_HAL::Device::PeriodicCb) = 0;

    /**
     * @brief Set device type identifier
     * 
     * @details Configures device type ID used for compass identification and logging.
     *          Set to DEVTYPE_AK8963 during initialization.
     * 
     * @param[in] devtype Device type identifier constant
     * 
     * @note Used by compass frontend for device identification
     */
    virtual void set_device_type(uint8_t devtype) = 0;

    /**
     * @brief Get 24-bit bus identifier
     * 
     * @details Returns unique bus identifier encoding bus type, bus number, and device address.
     *          Used to uniquely identify this compass instance across reboots for parameter storage.
     * 
     * @return uint32_t 24-bit bus ID (bits 0-7: address, 8-15: bus number, 16-23: bus type)
     * 
     * @note Bus ID must remain stable across reboots for compass calibration persistence
     */
    virtual uint32_t get_bus_id(void) const = 0;
};

/**
 * @class AP_AK8963_BusDriver_HALDevice
 * @brief Direct I2C bus driver implementation using HAL I2C device
 * 
 * @details Implements AP_AK8963_BusDriver interface for direct I2C communication using
 *          ArduPilot's hardware abstraction layer. Used when AK8963 is connected directly
 *          to system I2C bus or when MPU9250 module is accessed via I2C.
 *          
 *          This is the simpler of the two bus drivers, providing straightforward
 *          pass-through to HAL I2CDevice methods.
 *          
 *          Communication characteristics:
 *          - Standard I2C protocol at configured bus speed (typically 400kHz)
 *          - Direct register access via HAL I2C primitives
 *          - No special MPU coordination required
 *          - Uses HAL device semaphore for bus arbitration
 * 
 * @note AK8963 I2C address is 0x0C
 * @note No additional configuration needed - inherits HAL device setup
 * 
 * Source: libraries/AP_Compass/AP_Compass_AK8963.cpp:275-301
 */
class AP_AK8963_BusDriver_HALDevice: public AP_AK8963_BusDriver
{
public:
    /**
     * @brief Construct HAL device bus driver
     * 
     * @details Takes ownership of I2C device and wraps it for compass driver use.
     * 
     * @param[in] dev I2C device handle (ownership transferred)
     * 
     * @note Device must be pre-configured with correct I2C address (0x0C)
     */
    AP_AK8963_BusDriver_HALDevice(AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    /**
     * @brief Read multiple consecutive registers via I2C
     * 
     * @details Delegates to HAL I2CDevice::read_registers() for block read.
     * 
     * @param[in] reg Starting register address
     * @param[out] buf Buffer to receive data
     * @param[in] size Number of bytes to read
     * 
     * @return true if I2C transaction successful
     * 
     * Source: libraries/AP_Compass/AP_Compass_AK8963.cpp:281-284
     */
    virtual bool block_read(uint8_t reg, uint8_t *buf, uint32_t size) override;

    /**
     * @brief Read single register via I2C
     * 
     * @details Delegates to HAL I2CDevice::read_registers() with size=1.
     * 
     * @param[in] reg Register address
     * @param[out] val Pointer to receive register value
     * 
     * @return true if I2C transaction successful
     * 
     * Source: libraries/AP_Compass/AP_Compass_AK8963.cpp:286-289
     */
    virtual bool register_read(uint8_t reg, uint8_t *val) override;

    /**
     * @brief Write single register via I2C
     * 
     * @details Delegates to HAL I2CDevice::write_register().
     * 
     * @param[in] reg Register address
     * @param[in] val Value to write
     * 
     * @return true if I2C transaction successful
     * 
     * Source: libraries/AP_Compass/AP_Compass_AK8963.cpp:291-294
     */
    virtual bool register_write(uint8_t reg, uint8_t val) override;

    /**
     * @brief Get I2C bus semaphore
     * 
     * @details Returns HAL I2C device semaphore for bus access arbitration.
     * 
     * @return AP_HAL::Semaphore* Pointer to I2C bus semaphore
     * 
     * Source: libraries/AP_Compass/AP_Compass_AK8963.cpp:296-299
     */
    virtual AP_HAL::Semaphore  *get_semaphore() override;

    /**
     * @brief Register periodic callback
     * 
     * @details Delegates to HAL I2CDevice to register scheduler callback.
     * 
     * @param[in] period_usec Callback period in microseconds
     * @param[in] cb Callback function
     * 
     * @return AP_HAL::Device::PeriodicHandle Callback handle
     */
    AP_HAL::Device::PeriodicHandle register_periodic_callback(uint32_t period_usec, AP_HAL::Device::PeriodicCb) override;

    /**
     * @brief Set device type identifier
     * 
     * @details Passes device type to underlying HAL I2C device.
     * 
     * @param[in] devtype Device type constant (DEVTYPE_AK8963)
     */
    void set_device_type(uint8_t devtype) override {
        _dev->set_device_type(devtype);
    }

    /**
     * @brief Get bus identifier
     * 
     * @details Returns HAL device bus ID encoding I2C bus and address.
     * 
     * @return uint32_t 24-bit bus identifier
     */
    uint32_t get_bus_id(void) const override {
        return _dev->get_bus_id();
    }
    
private:
    /**
     * @brief Owned HAL I2C device instance
     * 
     * @details Provides actual I2C communication primitives.
     *          Ownership transferred during construction, released at destruction.
     */
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;
};

/**
 * @class AP_AK8963_BusDriver_Auxiliary
 * @brief MPU9250 auxiliary I2C bus driver implementation
 * 
 * @details Implements AP_AK8963_BusDriver interface for AK8963 connected via MPU9250's
 *          auxiliary I2C master interface. Used when MPU9250 is connected via SPI for
 *          higher IMU data rates, with AK8963 remaining on I2C but accessed through
 *          MPU9250 as an I2C master.
 *          
 *          Communication Architecture:
 *          Host (SPI) ←→ MPU9250 (SPI slave / I2C master) ←→ AK8963 (I2C slave)
 *          
 *          The MPU9250 acts as I2C master, performing I2C transactions to AK8963
 *          on behalf of the host. The host reads/writes MPU9250 slave registers
 *          over SPI, which the MPU9250 forwards to/from AK8963 over I2C.
 *          
 *          Operation Modes:
 *          1. Direct register access - Host writes AK8963 register address and data
 *             to MPU slave configuration, MPU performs I2C transaction
 *          2. Sampled data mode - MPU automatically reads AK8963 at configured rate,
 *             host reads data from MPU slave data registers
 *          
 *          Initialization Sequence:
 *          1. configure() - Set up MPU auxiliary bus interface
 *          2. Compass driver performs ID check and calibration via register_read/write
 *          3. start_measurements() - Enable automatic MPU sampling of AK8963
 *          4. Periodic reads via block_read retrieve MPU-cached AK8963 data
 * 
 * @note AK8963 fixed at I2C address 0x0C on auxiliary bus
 * @warning Potential for bus contention if MPU auxiliary transactions and host
 *          register access timing overlap. Careful coordination required.
 * @warning MPU9250 auxiliary bus configuration is shared resource - changes affect
 *          all devices on auxiliary bus
 * 
 * Source: libraries/AP_Compass/AP_Compass_AK8963.cpp (auxiliary bus implementation)
 */
class AP_AK8963_BusDriver_Auxiliary : public AP_AK8963_BusDriver
{
public:
    /**
     * @brief Construct auxiliary bus driver
     * 
     * @details Creates driver instance for AK8963 on MPU9250 auxiliary I2C bus.
     *          Obtains auxiliary bus interface from InertialSensor backend.
     * 
     * @param[in] ins InertialSensor singleton for backend access
     * @param[in] backend_id IMU backend type identifier (HAL_INS_MPU9250_SPI)
     * @param[in] backend_instance MPU9250 instance number (for multi-IMU systems)
     * @param[in] addr AK8963 I2C address on auxiliary bus (always 0x0C)
     * 
     * @note Requires MPU9250 backend to be detected and initialized first
     */
    AP_AK8963_BusDriver_Auxiliary(AP_InertialSensor &ins, uint8_t backend_id,
                                  uint8_t backend_instance, uint8_t addr);

    /**
     * @brief Destructor - releases auxiliary bus slave
     * 
     * @details Cleans up auxiliary bus slave resources, returning them to IMU backend.
     */
    ~AP_AK8963_BusDriver_Auxiliary();

    /**
     * @brief Read multiple registers via MPU auxiliary bus
     * 
     * @details Reads consecutive AK8963 registers through MPU9250 auxiliary bus slave interface.
     *          Data is read from MPU slave data registers containing cached AK8963 values.
     * 
     * @param[in] reg Starting AK8963 register address
     * @param[out] buf Buffer to receive data
     * @param[in] size Number of bytes to read
     * 
     * @return true if read successful through auxiliary bus
     * 
     * @note In sampled mode, reads MPU-cached data from last automatic sample
     */
    bool block_read(uint8_t reg, uint8_t *buf, uint32_t size) override;

    /**
     * @brief Read single register via MPU auxiliary bus
     * 
     * @details Reads one AK8963 register through MPU9250 auxiliary bus interface.
     * 
     * @param[in] reg AK8963 register address
     * @param[out] val Pointer to receive register value
     * 
     * @return true if read successful
     * 
     * @note Used during initialization for ID check and calibration reads
     */
    bool register_read(uint8_t reg, uint8_t *val) override;

    /**
     * @brief Write single register via MPU auxiliary bus
     * 
     * @details Writes one byte to AK8963 register through MPU9250 auxiliary bus.
     *          MPU performs I2C write transaction on behalf of host.
     * 
     * @param[in] reg AK8963 register address
     * @param[in] val Value to write
     * 
     * @return true if write successful
     * 
     * @note Used during initialization for mode configuration
     */
    bool register_write(uint8_t reg, uint8_t val) override;

    /**
     * @brief Register periodic callback via auxiliary bus
     * 
     * @details Registers callback through auxiliary bus interface for periodic sampling.
     * 
     * @param[in] period_usec Callback period in microseconds
     * @param[in] cb Callback function
     * 
     * @return AP_HAL::Device::PeriodicHandle Callback handle
     */
    AP_HAL::Device::PeriodicHandle register_periodic_callback(uint32_t period_usec, AP_HAL::Device::PeriodicCb) override;
    
    /**
     * @brief Get auxiliary bus semaphore
     * 
     * @details Returns semaphore for MPU auxiliary bus access coordination.
     *          Shared with IMU access to MPU9250.
     * 
     * @return AP_HAL::Semaphore* Pointer to MPU bus semaphore
     * 
     * @note Semaphore coordinates access between IMU and compass drivers
     */
    AP_HAL::Semaphore  *get_semaphore() override;

    /**
     * @brief Configure MPU auxiliary bus for AK8963 access
     * 
     * @details Sets up MPU9250 auxiliary I2C master to communicate with AK8963.
     *          Configures slave address, register addresses, and read/write modes.
     * 
     * @return true if auxiliary bus configured successfully
     * 
     * @note Called during init() before device ID check
     * @note Configuration persists until MPU reset or reconfiguration
     */
    bool configure() override;

    /**
     * @brief Start automatic AK8963 sampling via MPU
     * 
     * @details Enables MPU9250 to automatically read AK8963 data registers at configured rate.
     *          MPU performs periodic I2C reads and caches results in slave data registers.
     * 
     * @return true if sampling started successfully
     * 
     * @note Called during init() after sensor configuration complete
     * @note Reduces I2C transaction overhead by having MPU cache readings
     */
    bool start_measurements() override;

    /**
     * @brief Set device type identifier
     * 
     * @details Configures device type through auxiliary bus slave interface.
     * 
     * @param[in] devtype Device type identifier (DEVTYPE_AK8963)
     */
    void set_device_type(uint8_t devtype) override;

    /**
     * @brief Get bus identifier
     * 
     * @details Returns bus ID from auxiliary bus slave, encoding MPU auxiliary bus
     *          topology and AK8963 address.
     * 
     * @return uint32_t 24-bit bus identifier
     * 
     * @note Bus ID reflects auxiliary bus structure, not direct I2C topology
     */
    uint32_t get_bus_id(void) const override;
    
private:
    /**
     * @brief MPU9250 auxiliary bus interface
     * 
     * @details Provides access to MPU auxiliary I2C master functionality.
     *          Obtained from InertialSensor backend during construction.
     */
    AuxiliaryBus *_bus;

    /**
     * @brief Auxiliary bus slave instance for AK8963
     * 
     * @details Represents this AK8963 as a slave device on MPU auxiliary bus.
     *          Manages slave configuration registers and data buffers.
     */
    AuxiliaryBusSlave *_slave;

    /**
     * @brief Measurement sampling state flag
     * 
     * @details Tracks whether automatic MPU sampling has been started.
     *          Set by start_measurements(), used to prevent duplicate starts.
     */
    bool _started;
};

#endif  // AP_COMPASS_AK8963_ENABLED
