/**
 * @file AP_InertialSensor_L3G4200D.h
 * @brief STMicroelectronics L3G4200D gyroscope driver with separate accelerometer (Linux HAL only)
 * 
 * Implements backend driver for L3G4200D 3-axis MEMS gyroscope paired with a separate
 * accelerometer to form a complete 6-axis IMU. Legacy sensor combination used in early
 * Linux-based flight controllers (BeagleBone, Navio v1, Erle-Brain).
 * 
 * @note Legacy driver - modern platforms use integrated 6-axis IMUs (MPU-6000, ICM-20xxx, BMI088)
 * @warning Linux-only driver - uses Linux I2C sysfs interface, not available on ChibiOS/SITL
 * 
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX

#include <AP_HAL/I2CDevice.h>
#include <Filter/Filter.h>
#include <Filter/LowPassFilter2p.h>

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"

/**
 * @class AP_InertialSensor_L3G4200D
 * @brief Backend driver for L3G4200D gyroscope with separate accelerometer
 * 
 * @details Linux-specific I2C driver implementing 6-axis IMU from separate gyro and accel sensors:
 * 
 *          Gyroscope: STMicroelectronics L3G4200D
 *          - 3-axis MEMS gyroscope only
 *          - Selectable range: ±250, ±500, or ±2000 deg/s (configured at init)
 *          - 16-bit output resolution
 *          - I2C interface at up to 400kHz
 *          - Output data rate: Up to 800Hz
 *          - Typical configuration: ±2000 deg/s range
 * 
 *          Accelerometer: Separate sensor (model varies by board)
 *          - Board-specific accelerometer (ADXL345 or similar)
 *          - Paired with gyro to form complete 6-axis IMU
 *          - Separate I2C device handle
 * 
 *          Driver Architecture:
 *          - Dual I2C device management (gyro + accel)
 *          - Scheduler-driven periodic sampling via callbacks
 *          - Low-pass filtering on both gyro and accel data
 *          - Accumulation buffers for consistent update rates
 *          - No FIFO buffering (sensor limitation)
 * 
 *          Limitations:
 *          - Linux HAL only (uses Linux I2C sysfs interfaces)
 *          - Separate device initialization and error handling required
 *          - Higher CPU overhead vs integrated 6-axis IMUs with hardware FIFO
 *          - Legacy platform - limited testing on modern ArduPilot
 * 
 *          Typical Usage (board-specific probe):
 *          1. probe() called during HAL initialization with I2C device handles
 *          2. _init_sensor() configures both gyro and accel registers
 *          3. start() registers periodic callbacks for data accumulation
 *          4. _accumulate_gyro() and _accumulate_accel() sample at scheduler rate
 *          5. update() called by main loop to publish accumulated samples
 * 
 * @note Legacy sensor combination - newer platforms use MPU-6000/9250 or BMI088 integrated IMUs
 * @note Requires separate accelerometer for full 6-axis IMU functionality
 * @warning Linux-only driver - not available on ChibiOS, SITL, or other HAL platforms
 * @warning Limited testing on modern ArduPilot - use on legacy boards only
 * 
 * @see AP_InertialSensor_Backend
 * @see libraries/AP_InertialSensor/AP_InertialSensor_L3G4200D.cpp
 */
class AP_InertialSensor_L3G4200D : public AP_InertialSensor_Backend
{
public:
    /**
     * @brief Construct L3G4200D backend driver with separate gyro and accel I2C devices
     * 
     * @details Initializes the backend with ownership of two I2C device handles. Does not
     *          communicate with sensors - actual initialization deferred to _init_sensor()
     *          called by probe(). Stores device pointers and prepares filter objects.
     * 
     * @param[in] imu          Reference to main AP_InertialSensor instance
     * @param[in] dev_gyro     Owned I2C device handle for L3G4200D gyroscope
     * @param[in] dev_accel    Owned I2C device handle for companion accelerometer
     * 
     * @note Called only by probe() after successful sensor detection
     * @note Takes ownership of I2C device pointers (uses OwnPtr smart pointer)
     * 
     * @see probe()
     * @see _init_sensor()
     */
    AP_InertialSensor_L3G4200D(AP_InertialSensor &imu,
                                            AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev_gyro,
                                            AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev_accel);
    
    
    
    /**
     * @brief Destroy L3G4200D backend and release I2C device handles
     * 
     * @details Virtual destructor ensures proper cleanup of I2C device resources.
     *          OwnPtr automatically releases I2C device handles when destroyed.
     * 
     * @note Scheduler callbacks must be unregistered before destruction
     */
    virtual ~AP_InertialSensor_L3G4200D();

    /**
     * @brief Probe for L3G4200D gyroscope and companion accelerometer on I2C bus
     * 
     * @details Static factory method that attempts to detect and initialize L3G4200D
     *          gyroscope and separate accelerometer on provided I2C bus addresses.
     *          
     *          Detection sequence:
     *          1. Attempt to read WHO_AM_I register from gyro (expected value: 0xD3)
     *          2. Attempt to detect accelerometer on accel I2C device
     *          3. Construct backend instance if both sensors detected
     *          4. Call _init_sensor() to configure registers
     *          5. Return backend pointer on success, nullptr on failure
     *          
     *          Called during HAL initialization for Linux boards with L3G4200D configuration.
     * 
     * @param[in] imu          Reference to main AP_InertialSensor instance
     * @param[in] dev_gyro     Owned I2C device handle for gyroscope (address 0x68 or 0x69)
     * @param[in] dev_accel    Owned I2C device handle for accelerometer (board-specific)
     * 
     * @return Pointer to new backend instance on success, nullptr if sensor not detected
     * 
     * @note Static method - does not require existing instance
     * @note Takes ownership of I2C device pointers even on failure (will auto-release)
     * @note Linux HAL only - uses I2C sysfs device handles
     * 
     * @see AP_InertialSensor_Backend::probe_first()
     * @see _init_sensor()
     */
    static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu,
                                            AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev_gyro,
                                            AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev_accel);

   
   
    /**
     * @brief Publish accumulated gyro and accel samples to AP_InertialSensor frontend
     * 
     * @details Called by main loop scheduler to retrieve accumulated sensor samples
     *          and publish to the frontend for use by AHRS/EKF. Processes data from
     *          accumulation buffers filled by periodic _accumulate callbacks.
     *          
     *          Update sequence:
     *          1. Retrieve accumulated gyro samples (raw sensor units)
     *          2. Retrieve accumulated accel samples (raw sensor units)
     *          3. Apply calibration offsets and scale factors
     *          4. Apply low-pass filtering (_gyro_filter, _accel_filter)
     *          5. Publish to frontend via _publish_gyro() and _publish_accel()
     *          6. Reset accumulation counters for next cycle
     * 
     * @return true if samples published successfully, false if no new samples available
     * 
     * @note Called at main loop rate (typically 400Hz for Copter, 50-400Hz for other vehicles)
     * @note Accumulation callbacks (_accumulate_gyro, _accumulate_accel) run at higher rate
     * @note Filtering reduces noise while maintaining acceptable phase lag
     * 
     * @see start()
     * @see _accumulate_gyro()
     * @see _accumulate_accel()
     */
    bool update() override;

    /**
     * @brief Start periodic sampling by registering scheduler callbacks
     * 
     * @details Registers periodic timer callbacks with the HAL scheduler for both
     *          gyro and accel sampling. Called once during IMU initialization after
     *          successful probe and configuration.
     *          
     *          Callback registration:
     *          - _accumulate_gyro() registered at gyro sampling rate (typically 1kHz)
     *          - _accumulate_accel() registered at accel sampling rate (typically 1kHz)
     *          - Both callbacks accumulate samples into buffers
     *          - Main loop update() publishes accumulated samples to frontend
     * 
     * @note Called once during AP_InertialSensor::_detect_backends()
     * @note Callbacks continue until backend destroyed
     * @note Sampling rates determined by sensor register configuration in _init_sensor()
     * 
     * @see update()
     * @see _accumulate_gyro()
     * @see _accumulate_accel()
     */
    void start(void) override;

    
private:
    /**
     * @brief Initialize companion accelerometer sensor registers
     * 
     * @details Configures accelerometer for continuous sampling. Exact register
     *          configuration depends on accelerometer model (ADXL345 or similar).
     *          Typically sets range, sample rate, and enables measurement mode.
     * 
     * @return true if accelerometer configured successfully, false on I2C error
     * 
     * @note Called once during _init_sensor() after successful probe
     * @note Board-specific implementation varies by accelerometer model
     */
    bool _accel_init();
    
    /**
     * @brief Initialize L3G4200D gyroscope sensor registers
     * 
     * @details Configures L3G4200D register settings:
     *          - Power mode: Normal mode (CTRL_REG1)
     *          - Output data rate: 800Hz (CTRL_REG1)
     *          - Full scale: ±2000 deg/s (CTRL_REG4)
     *          - Block data update: Enable for atomic reads (CTRL_REG4)
     *          - FIFO mode: Bypass (sensor has no FIFO)
     * 
     * @return true if gyro configured successfully, false on I2C communication error
     * 
     * @note Called once during _init_sensor() after successful WHO_AM_I verification
     * @note I2C errors during init will cause probe() to fail and return nullptr
     */
    bool _gyro_init();
    
    /**
     * @brief Initialize both gyro and accelerometer sensors
     * 
     * @details Master initialization function that calls _gyro_init() and _accel_init()
     *          in sequence. Verifies both sensors are responding before registering backend.
     * 
     * @return true if both sensors initialized successfully, false if either fails
     * 
     * @note Called by probe() after successful sensor detection
     * @note Failure causes probe() to return nullptr (backend not registered)
     * 
     * @see _gyro_init()
     * @see _accel_init()
     */
    bool _init_sensor();
    
    /**
     * @brief Periodic callback to read and accumulate gyroscope samples
     * 
     * @details Scheduler callback that reads 6 bytes (X, Y, Z axes) from L3G4200D
     *          output registers and accumulates into sample buffer. Runs at gyro
     *          sampling rate (typically 1kHz).
     *          
     *          Read sequence:
     *          1. Read STATUS_REG to check data ready flag
     *          2. Read 6 bytes from OUT_X_L through OUT_Z_H (auto-increment)
     *          3. Convert 16-bit two's complement to signed values
     *          4. Accumulate into buffer for update() processing
     * 
     * @note Called at scheduler rate (higher than main loop rate)
     * @note Multiple samples accumulated between update() calls
     * @note I2C errors logged but don't halt sampling
     * 
     * @see start()
     * @see update()
     */
    void _accumulate_gyro();
    
    /**
     * @brief Periodic callback to read and accumulate accelerometer samples
     * 
     * @details Scheduler callback that reads accelerometer data and accumulates
     *          into sample buffer. Runs at accel sampling rate (typically 1kHz).
     *          Register read sequence depends on accelerometer model.
     * 
     * @note Called at scheduler rate (higher than main loop rate)
     * @note Multiple samples accumulated between update() calls
     * @note I2C errors logged but don't halt sampling
     * 
     * @see start()
     * @see update()
     */
    void _accumulate_accel();
    
    /**
     * @brief I2C device handle for L3G4200D gyroscope
     * 
     * @details Owned pointer to I2C device on Linux sysfs. Typical addresses:
     *          - 0x68 (SDO/SA0 pin pulled low)
     *          - 0x69 (SDO/SA0 pin pulled high)
     *          
     *          Auto-released when backend destroyed.
     */
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev_gyro;
    
    /**
     * @brief I2C device handle for companion accelerometer
     * 
     * @details Owned pointer to I2C device for separate accelerometer.
     *          Address and device type board-specific. Auto-released when backend destroyed.
     */
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev_accel;

    /**
     * @brief Configure low-pass filter cutoff frequency for both gyro and accel
     * 
     * @details Sets 2-pole low-pass filter cutoff frequency to reduce sensor noise
     *          while maintaining acceptable phase lag. Applied to both gyro and accel
     *          data streams. Typical values: 20-100Hz depending on vehicle dynamics.
     * 
     * @param[in] filter_hz    Low-pass filter cutoff frequency in Hz
     * 
     * @note Called during initialization with default filter frequency
     * @note Higher frequency = less filtering, more noise, less lag
     * @note Lower frequency = more filtering, less noise, more lag
     * 
     * @see LowPassFilter2pVector3f::set_cutoff_frequency()
     */
    void _set_filter_frequency(uint8_t filter_hz);

    /**
     * @brief 2-pole low-pass filter for accelerometer data (3-axis)
     * 
     * @details Digital biquad filter reducing accelerometer noise. Applied in update()
     *          after calibration but before publishing to frontend. Reduces vibration
     *          noise from motors and airframe.
     */
    LowPassFilter2pVector3f _accel_filter;
    
    /**
     * @brief 2-pole low-pass filter for gyroscope data (3-axis)
     * 
     * @details Digital biquad filter reducing gyroscope noise. Applied in update()
     *          after calibration but before publishing to frontend. Reduces high-frequency
     *          sensor noise while preserving dynamic response.
     */
    LowPassFilter2pVector3f _gyro_filter;
};
#endif // __AP_INERTIAL_SENSOR_L3G4200D2_H__
