/**
 * @file AP_Compass_LSM303D.h
 * @brief Driver for STMicroelectronics LSM303D combined accelerometer and magnetometer
 * 
 * @details This file implements the compass (magnetometer) portion of the LSM303D sensor.
 *          The LSM303D is a system-in-package featuring a 3D digital linear acceleration sensor
 *          and a 3D digital magnetic sensor. This driver handles only the magnetometer functionality,
 *          while the accelerometer portion is handled separately by the AP_InertialSensor subsystem.
 *          
 *          The sensor communicates via I2C or SPI and provides magnetic field measurements
 *          with a configurable range up to ±12 gauss with 16-bit resolution.
 * 
 * @note The accelerometer functionality of this chip is managed by AP_InertialSensor, not this driver
 * @see AP_InertialSensor for the accelerometer implementation
 */

#pragma once

#include "AP_Compass_config.h"

#if AP_COMPASS_LSM303D_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Device.h>
#include <AP_Math/AP_Math.h>

#include "AP_Compass.h"
#include "AP_Compass_Backend.h"

/**
 * @class AP_Compass_LSM303D
 * @brief Compass backend driver for LSM303D magnetometer
 * 
 * @details This class provides the ArduPilot compass backend interface for the LSM303D
 *          magnetometer. The LSM303D is a combined accelerometer/magnetometer, but this
 *          driver implements only the compass functionality. The accelerometer data from
 *          the same physical chip is handled by the AP_InertialSensor subsystem to maintain
 *          proper separation of concerns in the ArduPilot architecture.
 *          
 *          Key Features:
 *          - Magnetic field range: ±2/±4/±8/±12 gauss (typically configured for ±12 gauss)
 *          - 16-bit resolution for each axis (X, Y, Z)
 *          - Configurable output data rates up to 100 Hz
 *          - I2C and SPI interface support
 *          - Integrated temperature sensor
 *          - Built-in self-test
 *          
 *          The driver performs the following operations:
 *          1. Device detection via WHOAMI register verification
 *          2. Initialization of magnetometer-specific registers
 *          3. Configuration of magnetic field range and sample rate
 *          4. Periodic reading of magnetic field data
 *          5. Scaling and rotation of raw sensor values to vehicle frame
 *          
 *          Coordinate System:
 *          - Raw sensor data is read in sensor frame (X, Y, Z)
 *          - Data is transformed to vehicle frame using rotation parameter
 *          - Final output is in milligauss (mG)
 * 
 * @note This is a compass-only driver; accelerometer handled by AP_InertialSensor
 * @warning Ensure proper board rotation is specified during probe() to align sensor frame with vehicle frame
 */
class AP_Compass_LSM303D : public AP_Compass_Backend
{
public:
    /**
     * @brief Probe and initialize LSM303D magnetometer device
     * 
     * @details This static factory method attempts to detect and initialize an LSM303D
     *          magnetometer on the provided device interface. The probe sequence includes:
     *          1. Read and verify WHOAMI register (expected value 0x49)
     *          2. Initialize magnetometer registers for continuous measurement mode
     *          3. Configure magnetic field range (typically ±12 gauss)
     *          4. Set output data rate (typically 100 Hz)
     *          5. Apply board rotation to align sensor frame with vehicle frame
     *          6. Register compass instance with AP_Compass frontend
     *          
     *          If probe is successful, returns a new AP_Compass_LSM303D instance.
     *          If probe fails (device not detected or initialization error), returns nullptr.
     * 
     * @param[in] dev       Device interface (I2C or SPI) for communication with sensor
     * @param[in] rotation  Board rotation to transform sensor frame to vehicle frame
     *                      (e.g., ROTATION_NONE, ROTATION_YAW_90, etc.)
     * 
     * @return Pointer to new AP_Compass_LSM303D backend if successful, nullptr if probe fails
     * 
     * @note Called during sensor detection at startup by AP_Compass frontend
     * @see AP_Compass::_driver_probe() for probe calling mechanism
     */
    static AP_Compass_Backend *probe(AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                     enum Rotation rotation);

    /**
     * @brief Driver name identifier
     * @details Used for logging and debugging to identify this compass backend type
     */
    static constexpr const char *name = "LSM303D";

    /**
     * @brief Read current magnetic field sample from sensor
     * 
     * @details This method is called periodically by the AP_Compass frontend (typically at
     *          the configured sample rate, e.g., 100 Hz) to read the latest magnetic field
     *          data from the LSM303D sensor. The method performs:
     *          1. Check if new data is available (via status register or DRDY pin)
     *          2. Read 6 bytes of raw magnetometer data (X, Y, Z as 16-bit values)
     *          3. Scale raw values to milligauss using configured range scale factor
     *          4. Apply board rotation to transform to vehicle frame
     *          5. Publish scaled and rotated field values to AP_Compass frontend
     *          
     *          Magnetic Field Units:
     *          - Raw sensor values: 16-bit signed integers
     *          - Scale factor: Depends on configured range (e.g., ±12 gauss)
     *          - Output units: milligauss (mG), 1 gauss = 1000 milligauss
     *          
     *          Example scaling for ±12 gauss range:
     *          - 16-bit range: -32768 to +32767
     *          - Physical range: -12000 mG to +12000 mG
     *          - Scale factor: ~0.732 mG per LSB (12000 / 16384)
     * 
     * @note Called at main loop rate by scheduler via AP_Compass frontend
     * @note Magnetic field strength on Earth's surface: typically 250-650 mG depending on location
     * @warning Ensure sensor is properly calibrated before use in flight
     * 
     * @see AP_Compass::read() for frontend calling mechanism
     */
    void read() override;

    /**
     * @brief Destructor
     * @details Virtual destructor for proper cleanup of derived class
     */
    virtual ~AP_Compass_LSM303D() { }

private:
    /**
     * @brief Private constructor for LSM303D compass backend
     * 
     * @details Constructs a new LSM303D compass backend with the provided device interface.
     *          The constructor is private; instances are created via the static probe() method.
     * 
     * @param[in] dev Device interface (I2C or SPI) for sensor communication
     * 
     * @note Use probe() factory method to create instances
     */
    AP_Compass_LSM303D(AP_HAL::OwnPtr<AP_HAL::Device> dev);

    /**
     * @brief Initialize LSM303D magnetometer hardware and register with frontend
     * 
     * @details Performs complete initialization sequence:
     *          1. Verify device WHOAMI register (0x49 for LSM303D)
     *          2. Disable I2C interface if using SPI
     *          3. Configure magnetometer for continuous conversion mode
     *          4. Set magnetic field range (±12 gauss typical)
     *          5. Set sample rate (100 Hz typical)
     *          6. Register compass instance with AP_Compass frontend
     *          7. Set board rotation for coordinate frame transformation
     * 
     * @param[in] rotation Board rotation to align sensor frame with vehicle frame
     * 
     * @return true if initialization successful, false on error
     */
    bool init(enum Rotation rotation);
    
    /**
     * @brief Read single 8-bit register from LSM303D
     * @param[in] reg Register address to read
     * @return Register value
     */
    uint8_t _register_read(uint8_t reg);
    
    /**
     * @brief Write single 8-bit register to LSM303D
     * @param[in] reg Register address to write
     * @param[in] val Value to write to register
     */
    void _register_write(uint8_t reg, uint8_t val);
    
    /**
     * @brief Modify register using read-modify-write operation
     * @param[in] reg        Register address to modify
     * @param[in] clearbits  Bits to clear (bitwise AND with complement)
     * @param[in] setbits    Bits to set (bitwise OR)
     */
    void _register_modify(uint8_t reg, uint8_t clearbits, uint8_t setbits);
    
    /**
     * @brief Read multiple consecutive registers in single transaction
     * @param[in]  reg  Starting register address
     * @param[out] buf  Buffer to store read data
     * @param[in]  size Number of bytes to read
     * @return true if read successful, false on error
     */
    bool _block_read(uint8_t reg, uint8_t *buf, uint32_t size);

    /**
     * @brief Read and store one magnetic field sample
     * 
     * @details Reads 6 bytes of magnetometer data (X, Y, Z as 16-bit signed integers)
     *          and stores in member variables _mag_x, _mag_y, _mag_z for later processing.
     * 
     * @return true if sample read successfully, false on error
     */
    bool _read_sample();

    /**
     * @brief Check if new magnetometer data is available
     * @return true if data ready, false if no new data
     * @note Checks STATUS_REG_M register for data ready bit
     */
    bool _data_ready();
    
    /**
     * @brief Perform hardware-specific initialization
     * 
     * @details Configures LSM303D registers for magnetometer operation including:
     *          - Temperature sensor enable
     *          - Magnetometer resolution mode
     *          - Continuous conversion mode
     *          - Anti-aliasing filter bandwidth
     * 
     * @return true if hardware init successful, false on error
     */
    bool _hardware_init();
    
    /**
     * @brief Update compass reading and publish to frontend
     * 
     * @details Scales raw magnetic field values using range scale factor,
     *          converts to milligauss, and publishes to AP_Compass frontend.
     */
    void _update();
    
    /**
     * @brief Disable I2C interface when using SPI
     * @details Required when using SPI to prevent bus conflicts
     * @note Call during initialization if using SPI interface
     */
    void _disable_i2c();
    
    /**
     * @brief Configure magnetometer full-scale range
     * 
     * @details Sets the magnetic field measurement range and calculates the
     *          appropriate scale factor for converting raw values to milligauss.
     *          
     *          Available ranges: ±2, ±4, ±8, ±12 gauss
     *          ArduPilot typically uses ±12 gauss for maximum dynamic range.
     *          
     *          Scale factor calculation:
     *          - For ±12 gauss: 16-bit resolution over ±12000 mG range
     *          - Scale = 12000 / 16384 = ~0.732 mG per LSB
     * 
     * @param[in] max_ga Maximum range in gauss (2, 4, 8, or 12)
     * @return true if range set successfully, false on invalid range
     */
    bool _mag_set_range(uint8_t max_ga);
    
    /**
     * @brief Configure magnetometer output data rate
     * @param[in] frequency Sample rate in Hz (3.125 to 100 Hz)
     * @return true if sample rate set successfully, false on invalid frequency
     * @note Higher rates provide lower latency but increase CPU load
     */
    bool _mag_set_samplerate(uint16_t frequency);

    /**
     * @brief Optional data-ready pin for magnetometer
     * @details If configured, used to detect when new data is available
     *          instead of polling status register
     */
    AP_HAL::DigitalSource *_drdy_pin_m;
    
    /**
     * @brief Device interface for I2C or SPI communication
     * @details OwnPtr ensures proper cleanup when driver is destroyed
     */
    AP_HAL::OwnPtr<AP_HAL::Device> _dev;

    /**
     * @brief Scaling factor to convert raw values to milligauss
     * @details Calculated during initialization based on configured range
     *          Example: For ±12 gauss, scale ≈ 0.732 mG/LSB
     */
    float _mag_range_scale;
    
    /**
     * @brief Raw magnetometer X-axis value
     * @details 16-bit signed integer from sensor, updated by _read_sample()
     */
    int16_t _mag_x;
    
    /**
     * @brief Raw magnetometer Y-axis value
     * @details 16-bit signed integer from sensor, updated by _read_sample()
     */
    int16_t _mag_y;
    
    /**
     * @brief Raw magnetometer Z-axis value
     * @details 16-bit signed integer from sensor, updated by _read_sample()
     */
    int16_t _mag_z;

    /**
     * @brief Compass instance ID registered with AP_Compass frontend
     * @details Used to publish data to correct compass instance
     */
    uint8_t _compass_instance;
    
    /**
     * @brief Initialization success flag
     * @details Set to true after successful init(), prevents repeated initialization
     */
    bool _initialised;

    /**
     * @brief Configured magnetometer range in gauss
     * @details Stores current full-scale range setting (typically 12 for ±12 gauss)
     */
    uint8_t _mag_range_ga;
    
    /**
     * @brief Configured magnetometer sample rate in Hz
     * @details Stores current output data rate setting
     */
    uint8_t _mag_samplerate;
    
    /**
     * @brief Expected value of CTRL7 register for validation
     * @details Used to detect if register settings have been corrupted
     */
    uint8_t _reg7_expected;
};

#endif  // AP_COMPASS_LSM303D_ENABLED
