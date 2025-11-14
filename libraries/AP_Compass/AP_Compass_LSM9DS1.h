/**
 * @file AP_Compass_LSM9DS1.h
 * @brief Driver for STMicroelectronics LSM9DS1 9-axis IMU magnetometer
 * 
 * @details This driver implements magnetometer support for the LSM9DS1 9-DOF
 *          (degrees of freedom) inertial measurement unit. The LSM9DS1 combines
 *          a 3-axis accelerometer, 3-axis gyroscope, and 3-axis magnetometer.
 *          
 *          This driver handles ONLY the magnetometer portion of the LSM9DS1.
 *          The accelerometer and gyroscope are handled separately by the
 *          AP_InertialSensor subsystem.
 *          
 *          The magnetometer provides 3-axis magnetic field measurements with
 *          selectable full-scale ranges and supports both I2C and SPI interfaces.
 *          
 * @note The LSM9DS1 accelerometer/gyroscope portion is handled by AP_InertialSensor_LSM9DS1
 * 
 * Source: libraries/AP_Compass/AP_Compass_LSM9DS1.h
 */

#pragma once

#include "AP_Compass_config.h"

#if AP_COMPASS_LSM9DS1_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Device.h>
#include <AP_Math/AP_Math.h>

#include "AP_Compass_Backend.h"

/**
 * @class AP_Compass_LSM9DS1
 * @brief Magnetometer driver backend for STMicroelectronics LSM9DS1
 * 
 * @details This class implements the compass backend interface for the LSM9DS1
 *          magnetometer. The LSM9DS1 is a 9-DOF IMU, but this driver only handles
 *          the magnetometer component, with accelerometer and gyroscope handled
 *          by the AP_InertialSensor subsystem.
 *          
 *          Key Features:
 *          - 3-axis digital magnetometer
 *          - Selectable full-scale ranges: ±4, ±8, ±12, ±16 gauss
 *          - 16-bit data output
 *          - I2C and SPI interface support
 *          - Configurable output data rates
 *          
 *          Hardware Interface:
 *          - I2C address: 0x1E (default) or 0x1C
 *          - SPI mode 0 or mode 3 compatible
 *          - Maximum SPI clock: 10 MHz
 *          
 *          Measurement Range and Resolution:
 *          - Full-scale range: ±16 gauss (configurable)
 *          - Sensitivity: ~0.58 milligauss per LSB at ±16 gauss
 *          - Output data rate: Up to 80 Hz
 *          
 *          Coordinate System:
 *          - Outputs in sensor body frame
 *          - Rotation parameter applied during probe() to align with vehicle frame
 *          
 *          Field Units:
 *          - Raw sensor values converted to milligauss (mGauss)
 *          - 1 gauss = 1000 milligauss = 0.1 millitesla
 * 
 * @note This driver is part of the modular compass subsystem and registers
 *       itself with the AP_Compass frontend during initialization
 * 
 * @warning Ensure proper magnetometer calibration before flight. Uncalibrated
 *          magnetometers can cause navigation errors and unsafe flight behavior
 */
class AP_Compass_LSM9DS1 : public AP_Compass_Backend
{
public:
    /**
     * @brief Probe for LSM9DS1 magnetometer on the specified device
     * 
     * @details Attempts to detect and initialize an LSM9DS1 magnetometer on the
     *          provided device interface (I2C or SPI). This static factory method
     *          performs the following steps:
     *          
     *          1. Checks device ID to verify LSM9DS1 presence
     *          2. Creates driver instance if device detected
     *          3. Initializes and configures the magnetometer
     *          4. Registers with AP_Compass frontend if successful
     *          
     *          The rotation parameter allows the raw sensor measurements to be
     *          transformed to match the vehicle's orientation, accounting for
     *          physical mounting orientation of the sensor board.
     *          
     *          Typical usage during board initialization:
     *          - HAL probes all configured I2C/SPI buses
     *          - Calls probe() for each potential compass device
     *          - Driver registers itself if hardware found
     * 
     * @param[in] dev      HAL device interface (I2C or SPI) with ownership transferred
     * @param[in] rotation Board rotation to align sensor frame with vehicle frame
     *                     (e.g., ROTATION_YAW_90 for 90° rotation about Z-axis)
     * 
     * @return Pointer to initialized AP_Compass_Backend instance if successful,
     *         nullptr if device not found or initialization failed
     * 
     * @note Ownership of the device is transferred to the driver instance
     * @note This method is called during HAL initialization, not at runtime
     */
    static AP_Compass_Backend *probe(AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                     enum Rotation rotation);

    /// Driver name identifier for logging and diagnostics
    static constexpr const char *name = "LSM9DS1";

    /**
     * @brief Read current magnetometer values and update frontend
     * 
     * @details Reads the latest magnetic field measurements from the LSM9DS1,
     *          applies calibration and rotation transformations, and publishes
     *          the data to the AP_Compass frontend.
     *          
     *          This method is called periodically by the compass subsystem at
     *          the configured compass update rate (typically 50-100 Hz).
     *          
     *          Processing steps:
     *          1. Check data ready status register
     *          2. Read 6 bytes of magnetometer data (X, Y, Z as 16-bit values)
     *          3. Apply scaling to convert to milligauss
     *          4. Apply board rotation transformation
     *          5. Publish to frontend for sensor fusion
     *          
     *          Field Units:
     *          - Output values are in milligauss (mGauss)
     *          - Earth's magnetic field is approximately 250-650 mGauss depending on location
     * 
     * @note This method is called from the scheduler's compass update task
     * @note Override of AP_Compass_Backend::read() pure virtual method
     */
    void read() override;

    /**
     * @brief Virtual destructor
     * 
     * @details Ensures proper cleanup of derived class resources.
     *          The HAL device interface is automatically cleaned up by OwnPtr.
     */
    virtual ~AP_Compass_LSM9DS1() {}

private:
    /**
     * @brief Private constructor for LSM9DS1 magnetometer driver
     * 
     * @details Constructs driver instance with specified device and rotation.
     *          Constructor is private; instances created only via probe() method.
     *          
     *          The device ownership is transferred to this driver instance and
     *          will be automatically cleaned up on destruction.
     * 
     * @param[in] dev      HAL device interface with ownership transferred
     * @param[in] rotation Board rotation for sensor-to-vehicle frame transformation
     * 
     * @note Use probe() static factory method to create instances
     */
    AP_Compass_LSM9DS1(AP_HAL::OwnPtr<AP_HAL::Device> dev,
                       enum Rotation rotation);
    
    /**
     * @brief Initialize LSM9DS1 magnetometer
     * 
     * @details Performs complete initialization sequence for the magnetometer:
     *          1. Verify device ID matches LSM9DS1 (_check_id)
     *          2. Configure magnetometer registers (_configure)
     *          3. Set full-scale range and calculate scaling (_set_scale)
     *          4. Register compass instance with frontend
     *          5. Configure periodic read callback
     *          
     *          This method is called once during driver probe sequence.
     * 
     * @return true if initialization successful, false on any failure
     * 
     * @note Called from probe() after driver instance creation
     */
    bool init();
    
    /**
     * @brief Check device ID register to verify LSM9DS1 presence
     * 
     * @details Reads the WHO_AM_I register (0x0F) and verifies it matches the
     *          expected LSM9DS1 magnetometer ID value (0x3D).
     *          
     *          This check ensures the device on the bus is actually an LSM9DS1
     *          magnetometer and not a different sensor or an I2C/SPI fault.
     * 
     * @return true if device ID matches LSM9DS1, false otherwise
     */
    bool _check_id(void);
    
    /**
     * @brief Configure LSM9DS1 magnetometer registers
     * 
     * @details Configures the magnetometer for continuous measurement mode with
     *          optimal settings for ArduPilot:
     *          
     *          - Operating mode: Continuous conversion mode
     *          - Output data rate: 80 Hz (highest available)
     *          - Performance mode: Ultra-high performance for X and Y axes
     *          - Temperature compensation: Enabled
     *          - Low-power mode: Disabled for best accuracy
     *          
     *          Register Configuration:
     *          - CTRL_REG1_M: Data rate and performance mode
     *          - CTRL_REG2_M: Full-scale range selection
     *          - CTRL_REG3_M: Operating mode (continuous)
     *          - CTRL_REG4_M: Z-axis performance mode
     *          - CTRL_REG5_M: Block data update settings
     * 
     * @return true if configuration successful, false on communication failure
     */
    bool _configure(void);
    
    /**
     * @brief Set magnetometer full-scale range and calculate scaling factor
     * 
     * @details Configures the full-scale measurement range and calculates the
     *          scaling factor to convert raw 16-bit ADC values to milligauss.
     *          
     *          LSM9DS1 Supported Full-Scale Ranges:
     *          - ±4 gauss:  0.14 milligauss/LSB
     *          - ±8 gauss:  0.29 milligauss/LSB
     *          - ±12 gauss: 0.43 milligauss/LSB
     *          - ±16 gauss: 0.58 milligauss/LSB (typically used)
     *          
     *          The ±16 gauss range is typically selected to provide maximum
     *          dynamic range for operation near strong magnetic interference
     *          sources (motors, power systems).
     *          
     *          The _scaling member is set to convert raw values to milligauss:
     *          milligauss = raw_value * _scaling
     * 
     * @return true if scale configuration successful, false on communication failure
     * 
     * @note Field units after scaling are in milligauss (mGauss)
     */
    bool _set_scale(void);
    
    /**
     * @brief Update magnetometer reading from sensor
     * 
     * @details Internal update method called by the periodic timer callback.
     *          Reads magnetometer data from sensor and publishes to frontend:
     *          
     *          1. Read sample_regs structure (status + X,Y,Z data)
     *          2. Check status register for data ready
     *          3. Apply scaling to convert to milligauss
     *          4. Apply rotation transformation
     *          5. Publish to AP_Compass frontend
     *          
     *          This method is called at high frequency (typically 100 Hz) via
     *          the HAL timer system, asynchronously from the main thread.
     * 
     * @note Called from HAL timer interrupt context
     * @note Thread-safe through HAL device locking
     */
    void _update(void);

    /**
     * @brief Read single byte from LSM9DS1 register
     * 
     * @param[in] reg Register address to read
     * 
     * @return Register value (8-bit)
     */
    uint8_t _register_read(uint8_t reg);
    
    /**
     * @brief Write single byte to LSM9DS1 register
     * 
     * @param[in] reg Register address to write
     * @param[in] val Value to write (8-bit)
     */
    void _register_write(uint8_t reg, uint8_t val);
    
    /**
     * @brief Modify LSM9DS1 register with clear and set masks
     * 
     * @details Performs read-modify-write operation on register:
     *          new_value = (old_value & ~clearbits) | setbits
     *          
     *          Useful for changing specific bit fields without affecting others.
     * 
     * @param[in] reg       Register address
     * @param[in] clearbits Bitmask of bits to clear (set to 0)
     * @param[in] setbits   Bitmask of bits to set (set to 1)
     */
    void _register_modify(uint8_t reg, uint8_t clearbits, uint8_t setbits);
    
    /**
     * @brief Read multiple sequential bytes from LSM9DS1 registers
     * 
     * @details Performs burst read of multiple registers starting at specified
     *          address. Used for efficiently reading magnetometer X,Y,Z values
     *          and status in a single transaction.
     * 
     * @param[in]  reg  Starting register address
     * @param[out] buf  Buffer to store read data
     * @param[in]  size Number of bytes to read
     * 
     * @return true if read successful, false on communication failure
     */
    bool _block_read(uint8_t reg, uint8_t *buf, uint32_t size);
    
    /**
     * @brief Dump all LSM9DS1 magnetometer registers for debugging
     * 
     * @details Reads and prints all magnetometer configuration and data registers
     *          for diagnostic purposes. Used during development and troubleshooting.
     * 
     * @note Output sent to debug console
     */
    void _dump_registers();

    /// HAL device interface (I2C or SPI) with exclusive ownership
    AP_HAL::OwnPtr<AP_HAL::Device> _dev;
    
    /// Compass instance ID assigned by AP_Compass frontend during registration
    uint8_t _compass_instance;
    
    /// Scaling factor to convert raw 16-bit ADC values to milligauss
    /// Value depends on configured full-scale range (typically 0.58 mGauss/LSB for ±16 gauss)
    float _scaling;
    
    /// Board rotation to transform sensor body frame to vehicle frame
    /// Applied to raw measurements before publishing to frontend
    enum Rotation _rotation;

    /**
     * @brief Packed register structure for efficient burst reading
     * 
     * @details Memory layout matches LSM9DS1 magnetometer data registers:
     *          - Offset 0: STATUS_REG_M (0x27) - Data ready status
     *          - Offset 1-2: OUT_X_L_M, OUT_X_H_M - X-axis magnetic field (little-endian)
     *          - Offset 3-4: OUT_Y_L_M, OUT_Y_H_M - Y-axis magnetic field (little-endian)
     *          - Offset 5-6: OUT_Z_L_M, OUT_Z_H_M - Z-axis magnetic field (little-endian)
     *          
     *          The PACKED attribute ensures no padding is added, allowing direct
     *          memory mapping to hardware register layout.
     *          
     *          Status register bits:
     *          - Bit 3 (ZYXDA): X, Y, Z-axis data available
     *          - Bit 7 (ZYXOR): X, Y, Z-axis data overrun
     *          
     *          Value fields are signed 16-bit integers in sensor body frame,
     *          requiring scaling and rotation before use.
     */
    struct PACKED sample_regs {
        uint8_t status;    ///< STATUS_REG_M register value
        int16_t val[3];    ///< Magnetometer X, Y, Z axis raw values (signed 16-bit)
    };
};

#endif
