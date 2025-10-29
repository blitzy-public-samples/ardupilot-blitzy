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
 * @file AP_Compass_AK09916.h
 * @brief AK09916 magnetometer driver for ArduPilot
 * 
 * @details This driver provides support for the Asahi Kasei Microdevices (AKM) 
 *          AK09916 3-axis electronic compass. The AK09916 is a modern magnetometer
 *          with high resolution (16-bit) and low noise characteristics.
 *          
 *          Key Features:
 *          - 16-bit ADC resolution for all three magnetic axes
 *          - Sensitivity: 0.15 µT/LSB (microTesla per least significant bit)
 *          - Measurement range: ±4900 µT (±49 gauss)
 *          - Field output in milligauss (1 gauss = 100 milligauss = 0.1 mT)
 *          - I2C and SPI bus support
 *          - Continuous measurement mode at up to 100 Hz
 *          - Built-in self-test and temperature sensor
 *          
 *          Bus Support:
 *          - Standalone I2C operation (default address 0x0C)
 *          - Auxiliary bus operation via ICM20948 IMU (I2C or SPI)
 *          
 *          Driver Lifecycle:
 *          1. Probe: Detect sensor on specified bus and verify WHOAMI
 *          2. Init: Configure measurement mode and sensitivity
 *          3. Read: Periodic sampling in continuous measurement mode
 *          
 * @note The AK09916 is commonly found integrated with ICM20948 IMU chips,
 *       accessed through the IMU's auxiliary I2C/SPI bus interface.
 * 
 * @see AP_Compass_Backend for base compass interface
 * @see AP_InertialSensor for ICM20948 auxiliary bus access
 */
#pragma once

#include "AP_Compass_config.h"

#if AP_COMPASS_AK09916_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_Math/AP_Math.h>

#include "AP_Compass.h"
#include "AP_Compass_Backend.h"

#ifndef HAL_COMPASS_AK09916_I2C_ADDR
# define HAL_COMPASS_AK09916_I2C_ADDR 0x0C
#endif


#ifndef HAL_COMPASS_ICM20948_I2C_ADDR
# define HAL_COMPASS_ICM20948_I2C_ADDR 0x69
#endif

#ifndef HAL_COMPASS_ICM20948_I2C_ADDR2
# define HAL_COMPASS_ICM20948_I2C_ADDR2 0x68
#endif

class AuxiliaryBus;
class AuxiliaryBusSlave;
class AP_InertialSensor;
class AP_AK09916_BusDriver;

/**
 * @class AP_Compass_AK09916
 * @brief Driver for AKM AK09916 3-axis magnetometer
 * 
 * @details This class implements the ArduPilot compass backend interface for the
 *          AK09916 electronic compass sensor. The driver supports multiple connection
 *          methods including standalone I2C operation and auxiliary bus access through
 *          ICM20948 IMU chips.
 *          
 *          The AK09916 provides 16-bit resolution magnetic field measurements with
 *          a sensitivity of 0.15 µT/LSB. Field values are reported in milligauss
 *          units (1 mT = 10 gauss = 1000 milligauss).
 *          
 *          Typical Usage Pattern:
 *          - Static probe() method detects sensor and creates instance
 *          - Constructor initializes bus driver and rotation parameters
 *          - init() configures sensor mode and validates calibration
 *          - read() is called periodically to sample magnetic field data
 *          
 *          Connection Methods:
 *          1. Standalone I2C: Direct connection to flight controller I2C bus
 *          2. ICM20948 Auxiliary I2C: Through ICM20948 IMU's auxiliary I2C master
 *          3. ICM20948 Auxiliary SPI: Through ICM20948 IMU connected via SPI
 *          
 *          The driver performs factory sensitivity adjustment and ADC sensitivity
 *          correction to provide calibrated magnetic field readings. Continuous
 *          measurement mode is used for efficient periodic sampling.
 *          
 * @note This driver is typically used with external compass modules or integrated
 *       within 9-axis IMU packages containing ICM20948 + AK09916.
 * 
 * @warning Field measurements must be corrected for hard-iron and soft-iron effects
 *          through the compass calibration process before use in navigation.
 * 
 * @see AP_Compass_Backend for base interface
 * @see AP_AK09916_BusDriver for bus abstraction interface
 */
class AP_Compass_AK09916 : public AP_Compass_Backend
{
public:
    /**
     * @brief Probe for AK09916 magnetometer on standalone I2C bus
     * 
     * @details Attempts to detect an AK09916 sensor on the provided I2C device.
     *          This method performs WHOAMI register verification to confirm the
     *          sensor identity. If successful, creates and initializes an instance
     *          of the driver.
     *          
     *          Probe sequence:
     *          1. Verify device communication
     *          2. Read and validate WHOAMI register (expected 0x09)
     *          3. Create driver instance if sensor detected
     *          4. Initialize sensor configuration
     * 
     * @param[in] dev           I2C device handle for the AK09916 (default address 0x0C)
     * @param[in] force_external  If true, treat compass as external even if detected
     *                           on internal bus (affects priority and calibration)
     * @param[in] rotation      Sensor orientation relative to vehicle frame (NED convention)
     *                          Specifies required rotation transform to align sensor
     *                          axes with vehicle body frame
     * 
     * @return Pointer to initialized AP_Compass_Backend instance if probe successful,
     *         nullptr if sensor not detected or initialization failed
     * 
     * @note This is typically called during compass auto-detection at startup
     * @see HAL_COMPASS_AK09916_I2C_ADDR for default I2C address
     */
    static AP_Compass_Backend *probe(AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev,
                                     bool force_external,
                                     enum Rotation rotation);

#if AP_COMPASS_ICM20948_ENABLED
    /**
     * @brief Probe for AK09916 on ICM20948 auxiliary I2C bus
     * 
     * @details Detects AK09916 magnetometer accessed through ICM20948 IMU's auxiliary
     *          I2C master interface. The ICM20948 acts as an I2C master to communicate
     *          with the AK09916 magnetometer, allowing both devices to share a single
     *          data interface to the flight controller.
     * 
     * @param[in] dev           I2C device handle for the AK09916 magnetometer
     * @param[in] dev_icm       I2C device handle for the ICM20948 IMU
     * @param[in] force_external  Treat compass as external module
     * @param[in] rotation      Sensor orientation relative to vehicle frame
     * 
     * @return Pointer to initialized driver instance, or nullptr if probe failed
     */
    static AP_Compass_Backend *probe_ICM20948(AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev,
                                             AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev_icm,
                                             bool force_external,
                                             enum Rotation rotation);

    /**
     * @brief Probe for AK09916 on ICM20948 auxiliary bus (SPI connection by default)
     * 
     * @param[in] mpu9250_instance  ICM20948 instance number (for multi-IMU systems)
     * @param[in] rotation         Sensor orientation relative to vehicle frame
     * 
     * @return Pointer to initialized driver instance, or nullptr if probe failed
     */
    static AP_Compass_Backend *probe_ICM20948(uint8_t mpu9250_instance, enum Rotation rotation);
    
    /**
     * @brief Probe for AK09916 on ICM20948 auxiliary bus with explicit SPI connection
     * 
     * @details Detects AK09916 when ICM20948 IMU is connected via SPI bus. The magnetometer
     *          communication is tunneled through the ICM20948's auxiliary I2C master.
     * 
     * @param[in] mpu9250_instance  ICM20948 instance number (0 for first IMU, 1 for second, etc.)
     * @param[in] rotation         Sensor orientation relative to vehicle frame
     * 
     * @return Pointer to initialized driver instance, or nullptr if probe failed
     */
    static AP_Compass_Backend *probe_ICM20948_SPI(uint8_t mpu9250_instance,
                                             enum Rotation rotation);

    /**
     * @brief Probe for AK09916 on ICM20948 auxiliary bus with explicit I2C connection
     * 
     * @details Detects AK09916 when ICM20948 IMU is connected via I2C bus. The magnetometer
     *          communication is tunneled through the ICM20948's auxiliary I2C master.
     * 
     * @param[in] mpu9250_instance  ICM20948 instance number (0 for first IMU, 1 for second, etc.)
     * @param[in] rotation         Sensor orientation relative to vehicle frame
     * 
     * @return Pointer to initialized driver instance, or nullptr if probe failed
     * 
     * @note Default I2C addresses: ICM20948 at 0x69 or 0x68, AK09916 at 0x0C
     */
    static AP_Compass_Backend *probe_ICM20948_I2C(uint8_t mpu9250_instance,
                                             enum Rotation rotation);
#endif

    /**
     * @brief Driver name identifier for logging and diagnostics
     */
    static constexpr const char *name = "AK09916";

    /**
     * @brief Destructor - releases bus driver resources
     */
    virtual ~AP_Compass_AK09916();

    /**
     * @brief Read magnetic field data from sensor
     * 
     * @details Performs a periodic read of the AK09916 magnetometer in continuous
     *          measurement mode. This method:
     *          1. Reads all sensor registers including status and data
     *          2. Validates data ready and overflow status bits
     *          3. Converts 16-bit ADC values to physical units (milligauss)
     *          4. Applies factory and ADC sensitivity adjustments
     *          5. Applies rotation transform to align with vehicle frame
     *          6. Publishes corrected field vector to compass library
     *          
     *          The sensor operates in continuous measurement mode (100 Hz typical),
     *          providing new data at each read cycle. Status registers are checked
     *          to ensure data validity before processing.
     * 
     * @note This method is called by the scheduler at the configured compass update rate
     * @note Field values are output in milligauss units after all corrections applied
     * 
     * @see _make_factory_sensitivity_adjustment for factory calibration correction
     * @see _make_adc_sensitivity_adjustment for ADC sensitivity correction
     */
    void read() override;

    /**
     * @brief AK09916 sensor register structure for block reads
     * 
     * @details Defines the register layout for reading magnetic field data and status.
     *          This structure matches the AK09916 register map for efficient block
     *          read operations (sequential register read in a single I2C/SPI transaction).
     *          
     *          Register contents:
     *          - st1: Status 1 (Data Ready bit 0, Data Overrun bit 1)
     *          - val[0]: X-axis magnetic field (16-bit signed, LSB first)
     *          - val[1]: Y-axis magnetic field (16-bit signed, LSB first)
     *          - val[2]: Z-axis magnetic field (16-bit signed, LSB first)
     *          - tmps: Temperature sensor output (8-bit)
     *          - st2: Status 2 (Overflow bit 3, Output bit size bit 4)
     *          
     * @note PACKED attribute ensures structure matches register layout without padding
     * @note Public visibility required for BusDriver classes to access definition
     */
    struct PACKED sample_regs {
        uint8_t st1;      ///< Status register 1 (DRDY, DOR bits)
        int16_t val[3];   ///< Magnetic field [X, Y, Z] in sensor counts
        uint8_t tmps;     ///< Temperature sensor reading
        uint8_t st2;      ///< Status register 2 (HOFL, output bit flag)
    };

private:
    /**
     * @brief Private constructor - instantiated via probe() methods
     * 
     * @param[in] bus            Bus driver abstraction (I2C or auxiliary bus)
     * @param[in] force_external Force external compass designation
     * @param[in] rotation       Sensor rotation relative to vehicle frame
     */
    AP_Compass_AK09916(AP_AK09916_BusDriver *bus, bool force_external,
                       enum Rotation rotation);

    /**
     * @brief Initialize sensor configuration and register with compass library
     * 
     * @details Performs complete sensor initialization sequence:
     *          1. Reset sensor to known state
     *          2. Verify WHOAMI register (device ID check)
     *          3. Read factory calibration data
     *          4. Configure continuous measurement mode
     *          5. Register compass instance with AP_Compass library
     * 
     * @return true if initialization successful, false on any failure
     */
    bool init();
    
    /**
     * @brief Apply factory sensitivity adjustment to magnetic field vector
     * 
     * @details Corrects raw sensor readings using factory calibration values stored
     *          in sensor ROM. These adjustments compensate for manufacturing variations
     *          in sensor sensitivity across the three axes.
     * 
     * @param[in,out] field Magnetic field vector in milligauss (modified in place)
     */
    void _make_factory_sensitivity_adjustment(Vector3f &field) const;
    
    /**
     * @brief Apply ADC sensitivity adjustment to magnetic field vector
     * 
     * @details Converts raw ADC counts to physical units (milligauss) using the
     *          AK09916 sensitivity specification of 0.15 µT/LSB. Also applies any
     *          mode-dependent scaling factors.
     * 
     * @param[in,out] field Magnetic field vector (modified in place)
     * 
     * @note Conversion: 1 µT = 10 milligauss, so 0.15 µT/LSB = 1.5 milligauss/LSB
     */
    void _make_adc_sensitivity_adjustment(Vector3f &field) const;

    /**
     * @brief Perform soft reset of AK09916 sensor
     * 
     * @details Issues soft reset command to return sensor to default power-on state.
     *          Waits for reset completion before returning.
     * 
     * @return true if reset successful, false on communication error
     */
    bool _reset();
    
    /**
     * @brief Configure sensor measurement mode
     * 
     * @details Sets the AK09916 to continuous measurement mode for periodic sampling.
     *          Mode 2 (100 Hz) is typically used for navigation-grade compass operation.
     * 
     * @return true if mode configuration successful, false on error
     */
    bool _setup_mode();
    
    /**
     * @brief Verify sensor identity via WHOAMI register
     * 
     * @details Reads the WHOAMI register and confirms it contains the expected
     *          value (0x09 for AK09916). This validates correct sensor detection
     *          and communication.
     * 
     * @return true if WHOAMI matches expected value, false otherwise
     */
    bool _check_id();
    
    /**
     * @brief Read and store factory calibration data
     * 
     * @details Retrieves factory-programmed sensitivity adjustment values from
     *          sensor ROM. These values are used for lifetime correction of sensor
     *          measurements.
     * 
     * @return true if calibration data read successfully, false on error
     */
    bool _calibrate();

    /**
     * @brief Internal update method called by periodic callback
     * 
     * @details Reads sensor data and publishes to compass library. Called at
     *          configured update rate by HAL scheduler.
     */
    void _update();

    AP_AK09916_BusDriver *_bus;           ///< Bus abstraction (I2C or auxiliary)
    bool _force_external;                 ///< Force external designation flag
    uint8_t _compass_instance;            ///< Compass instance ID in AP_Compass
    bool _initialized;                    ///< Initialization completion flag
    enum Rotation _rotation;              ///< Rotation transform to vehicle frame
    enum AP_Compass_Backend::DevTypes _devtype; ///< Device type identifier
    uint8_t no_data;                      ///< Consecutive no-data counter for health monitoring
};


/**
 * @class AP_AK09916_BusDriver
 * @brief Abstract bus interface for AK09916 communication
 * 
 * @details Provides hardware abstraction layer for AK09916 sensor access across
 *          different bus types and connection methods. This interface allows the
 *          compass driver to operate identically whether the sensor is connected
 *          via standalone I2C, auxiliary I2C through an IMU, or SPI.
 *          
 *          Concrete implementations:
 *          - AP_AK09916_BusDriver_HALDevice: Direct I2C/SPI via HAL device
 *          - AP_AK09916_BusDriver_Auxiliary: Through ICM20948 auxiliary bus
 *          
 * @note This abstraction enables unit testing and hardware independence
 */
class AP_AK09916_BusDriver
{
public:
    virtual ~AP_AK09916_BusDriver() { }

    /**
     * @brief Read multiple consecutive registers in a single transaction
     * 
     * @param[in]  reg  Starting register address
     * @param[out] buf  Buffer to receive register data
     * @param[in]  size Number of bytes to read
     * 
     * @return true if read successful, false on communication error
     */
    virtual bool block_read(uint8_t reg, uint8_t *buf, uint32_t size) = 0;
    
    /**
     * @brief Read a single register value
     * 
     * @param[in]  reg  Register address to read
     * @param[out] val  Pointer to store register value
     * 
     * @return true if read successful, false on communication error
     */
    virtual bool register_read(uint8_t reg, uint8_t *val) = 0;
    
    /**
     * @brief Write a single register value
     * 
     * @param[in] reg     Register address to write
     * @param[in] val     Value to write to register
     * @param[in] checked If true, enable register write verification
     * 
     * @return true if write successful, false on communication error
     */
    virtual bool register_write(uint8_t reg, uint8_t val, bool checked=false) = 0;

    /**
     * @brief Get bus semaphore for thread-safe access
     * 
     * @return Pointer to semaphore object for bus arbitration
     */
    virtual AP_HAL::Semaphore  *get_semaphore() = 0;

    /**
     * @brief Configure bus-specific settings
     * 
     * @details Performs any bus-specific initialization or configuration.
     *          Default implementation returns success (no configuration needed).
     * 
     * @return true if configuration successful, false on error
     */
    virtual bool configure() { return true; }
    
    /**
     * @brief Start measurement mode on the bus
     * 
     * @details Enables continuous sampling mode for auxiliary bus configurations.
     *          Default implementation returns success (no action needed).
     * 
     * @return true if measurements started, false on error
     */
    virtual bool start_measurements() { return true; }
    
    /**
     * @brief Register periodic callback for sensor updates
     * 
     * @param[in] period_usec Callback period in microseconds
     * @param[in] cb          Callback function to execute
     * 
     * @return Handle to periodic callback for later cancellation
     */
    virtual AP_HAL::Device::PeriodicHandle register_periodic_callback(uint32_t, AP_HAL::Device::PeriodicCb) = 0;

    /**
     * @brief Set device type identifier within device class
     * 
     * @param[in] devtype Device type enumeration value
     * 
     * @note Used for logging and driver identification in multi-sensor systems
     */
    virtual void set_device_type(uint8_t devtype) = 0;

    /**
     * @brief Get 24-bit bus identifier for this device
     * 
     * @return Unique bus ID encoding bus type, bus number, and device address
     * 
     * @note Format: [bus_type:4][bus_num:4][address:16] allows disambiguation
     *       of multiple sensors in logging and parameter systems
     */
    virtual uint32_t get_bus_id(void) const = 0;

    /**
     * @brief Setup automatic register value verification
     * 
     * @details Configures periodic verification that critical register values
     *          have not changed unexpectedly (e.g., due to sensor glitch or reset).
     * 
     * @param[in] num_regs Number of registers to monitor
     * 
     * @note Default implementation does nothing (no register checking)
     */
    virtual void setup_checked_registers(uint8_t num_regs) {}
    
    /**
     * @brief Check next register in rotation
     * 
     * @details Verifies one register value matches expected, called periodically.
     *          Cycles through all configured registers over time.
     * 
     * @note Default implementation does nothing (no register checking)
     */
    virtual void check_next_register(void) {}
};

/**
 * @class AP_AK09916_BusDriver_HALDevice
 * @brief Direct I2C bus driver implementation using HAL device interface
 * 
 * @details Provides AK09916 access through standalone I2C connection using the
 *          ArduPilot Hardware Abstraction Layer (HAL) device interface. This is
 *          the standard method for external compass modules directly connected
 *          to flight controller I2C buses.
 *          
 *          This implementation wraps an AP_HAL::I2CDevice and forwards all bus
 *          operations directly to the HAL layer, which handles platform-specific
 *          I2C communication, timing, and error handling.
 * 
 * @note Typical I2C bus speed is 400 kHz for AK09916 sensors
 * @see AP_HAL::I2CDevice for platform-specific I2C implementation
 */
class AP_AK09916_BusDriver_HALDevice: public AP_AK09916_BusDriver
{
public:
    /**
     * @brief Constructor taking ownership of I2C device handle
     * 
     * @param[in] dev I2C device handle (ownership transferred)
     */
    AP_AK09916_BusDriver_HALDevice(AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    virtual bool block_read(uint8_t reg, uint8_t *buf, uint32_t size) override;
    virtual bool register_read(uint8_t reg, uint8_t *val) override;
    virtual bool register_write(uint8_t reg, uint8_t val, bool checked) override;
    virtual AP_HAL::Semaphore  *get_semaphore() override;
    AP_HAL::Device::PeriodicHandle register_periodic_callback(uint32_t period_usec, AP_HAL::Device::PeriodicCb) override;

    /**
     * @brief Set device type identifier
     * @param[in] devtype Device type enumeration value
     */
    void set_device_type(uint8_t devtype) override {
        _dev->set_device_type(devtype);
    }

    /**
     * @brief Get 24-bit bus identifier
     * @return Unique bus ID from HAL device
     */
    uint32_t get_bus_id(void) const override {
        return _dev->get_bus_id();
    }

    /**
     * @brief Setup automatic register verification
     * @param[in] num_regs Number of registers to monitor for unexpected changes
     */
    void setup_checked_registers(uint8_t num_regs) override {
        _dev->setup_checked_registers(num_regs);
    }
    
    /**
     * @brief Check next register value in verification rotation
     */
    void check_next_register(void) override {
        _dev->check_next_register();
    }
    
private:
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;  ///< Owned I2C device handle
};

/**
 * @class AP_AK09916_BusDriver_Auxiliary
 * @brief Auxiliary bus driver for AK09916 accessed through ICM20948 IMU
 * 
 * @details Implements AK09916 access through the auxiliary I2C master interface
 *          of an ICM20948 IMU. The ICM20948 acts as an I2C master to communicate
 *          with the magnetometer, allowing the compass to be read through the same
 *          interface as the IMU (I2C or SPI to flight controller).
 *          
 *          This arrangement is common in integrated 9-axis sensor modules where
 *          the IMU and magnetometer share a single communication bus to the host.
 *          The auxiliary bus configuration handles the complexity of configuring
 *          the ICM20948's I2C master and mapping magnetometer registers.
 *          
 *          Operational Sequence:
 *          1. Configure ICM20948 auxiliary I2C master
 *          2. Map AK09916 registers to ICM20948 read buffers
 *          3. Enable continuous reading from auxiliary device
 *          4. Read magnetometer data from ICM20948's buffer registers
 * 
 * @note This driver depends on AP_InertialSensor's auxiliary bus interface
 * @warning Timing is controlled by ICM20948 sample rate, not directly by compass driver
 * 
 * @see AP_InertialSensor for auxiliary bus interface
 * @see AuxiliaryBus for auxiliary I2C master abstraction
 */
class AP_AK09916_BusDriver_Auxiliary : public AP_AK09916_BusDriver
{
public:
    /**
     * @brief Constructor for auxiliary bus access through InertialSensor
     * 
     * @param[in] ins              Reference to AP_InertialSensor instance managing IMU
     * @param[in] backend_id       IMU backend identifier (e.g., INS_ICM20948)
     * @param[in] backend_instance Instance number of this backend type (0, 1, 2...)
     * @param[in] addr             I2C address of AK09916 on auxiliary bus (typically 0x0C)
     */
    AP_AK09916_BusDriver_Auxiliary(AP_InertialSensor &ins, uint8_t backend_id,
                                  uint8_t backend_instance, uint8_t addr);
    
    /**
     * @brief Destructor - releases auxiliary bus resources
     */
    ~AP_AK09916_BusDriver_Auxiliary();

    bool block_read(uint8_t reg, uint8_t *buf, uint32_t size) override;
    bool register_read(uint8_t reg, uint8_t *val) override;
    bool register_write(uint8_t reg, uint8_t val, bool checked) override;
    AP_HAL::Device::PeriodicHandle register_periodic_callback(uint32_t period_usec, AP_HAL::Device::PeriodicCb) override;
    AP_HAL::Semaphore  *get_semaphore() override;

    /**
     * @brief Configure ICM20948 auxiliary I2C master for AK09916 access
     * 
     * @details Sets up the ICM20948's auxiliary I2C master interface to
     *          communicate with the AK09916 magnetometer. Configures I2C
     *          clock speed, slave address, and register mapping.
     * 
     * @return true if configuration successful, false on error
     */
    bool configure() override;
    
    /**
     * @brief Enable continuous measurement mode on auxiliary bus
     * 
     * @details Starts the ICM20948's automatic reading of AK09916 registers.
     *          After this call, magnetometer data is continuously updated in
     *          ICM20948's data buffers synchronized with IMU samples.
     * 
     * @return true if measurements started, false on error
     */
    bool start_measurements() override;

    /**
     * @brief Set device type identifier
     * @param[in] devtype Device type enumeration value
     */
    void set_device_type(uint8_t devtype) override;

    /**
     * @brief Get 24-bit bus identifier
     * @return Bus ID including auxiliary bus encoding
     */
    uint32_t get_bus_id(void) const override;
    
private:
    AuxiliaryBus *_bus;        ///< Pointer to auxiliary bus interface from IMU
    AuxiliaryBusSlave *_slave; ///< Slave device handle for this magnetometer
    bool _started;             ///< Flag indicating measurements have been started
};

#endif  // AP_COMPASS_AK09916_ENABLED
