/**
 * @file AP_Baro_ICM20789.h
 * @brief InvenSense ICM-20789 6-axis IMU with integrated barometer driver
 * 
 * @details This driver supports the ICM-20789, a combination 6-axis IMU 
 *          (gyro + accelerometer) and barometric pressure sensor in a single package.
 *          The barometer shares the same physical package and communication interface
 *          with the IMU, requiring coordinated access through the IMU device interface.
 * 
 *          Key characteristics:
 *          - Integrated IMU + barometer in single package
 *          - Barometer accessed via IMU bridge/pass-through mechanism
 *          - OTP (One-Time Programmable) calibration data stored in device
 *          - LUT (Look-Up Table) based pressure/temperature conversion
 *          - Pressure range: 30kPa to 110kPa (≈300m below sea level to ≈9000m altitude)
 *          - Pressure resolution: 0.13 Pa
 * 
 * @note The barometer portion of the ICM-20789 is not standalone accessible;
 *       it must be accessed through the IMU device interface (I2C or SPI).
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

#include "AP_Baro_Backend.h"

#if AP_BARO_ICM20789_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Semaphores.h>
#include <AP_HAL/Device.h>

#ifndef HAL_BARO_ICM20789_I2C_ADDR
/**
 * @brief Default I2C address for ICM-20789 barometer portion
 * 
 * @details The barometer is on a separate I2C address (0x63) from the IMU
 *          portion (typically 0x68 or 0x69), even though they are in the same
 *          physical package. Access is coordinated through the IMU bridge.
 */
#define HAL_BARO_ICM20789_I2C_ADDR 0x63
#endif

/**
 * @class AP_Baro_ICM20789
 * @brief Driver for InvenSense ICM-20789 integrated IMU/barometer sensor
 * 
 * @details The ICM-20789 combines a 6-axis IMU (3-axis gyroscope + 3-axis accelerometer)
 *          with a barometric pressure sensor in a single package. This driver handles
 *          only the barometer portion; the IMU portion is handled by AP_InertialSensor.
 * 
 *          Architecture:
 *          - Barometer accessed through IMU device interface (I2C or SPI)
 *          - Requires coordination with IMU driver for shared device access
 *          - Uses OTP (One-Time Programmable) calibration data stored in sensor
 *          - Implements LUT (Look-Up Table) based conversion for pressure/temperature
 * 
 *          Measurement Process:
 *          1. Read OTP calibration constants via IMU bridge at initialization
 *          2. Send measurement commands through IMU register interface
 *          3. Read raw pressure and temperature values
 *          4. Apply LUT-based conversion using calibration constants
 *          5. Output pressure in Pascals and temperature in Celsius
 * 
 *          Data Flow:
 *          ArduPilot → AP_Baro_ICM20789 → IMU Device Interface → 
 *          ICM-20789 IMU Bridge → Barometer Registers
 * 
 * @note This driver requires the IMU portion to be initialized first, as the
 *       barometer is accessed through the IMU's device interface.
 * 
 * @note Pressure output is in Pascals (Pa), temperature in degrees Celsius (°C)
 * 
 * @warning Both IMU and barometer drivers must coordinate access to the shared
 *          device interface to avoid bus conflicts.
 */
class AP_Baro_ICM20789 : public AP_Baro_Backend
{
public:
    /**
     * @brief Update barometer readings from accumulated measurements
     * 
     * @details Called by the main AP_Baro scheduler to publish new pressure and
     *          temperature readings. Retrieves accumulated samples from the timer
     *          callback, averages them, and updates the barometer instance with
     *          the latest data.
     * 
     *          Process:
     *          1. Lock accumulation semaphore
     *          2. Average accumulated pressure and temperature samples
     *          3. Reset accumulation counters
     *          4. Publish averaged data to AP_Baro frontend
     * 
     * @note This is called at the AP_Baro update rate (typically 10-50 Hz),
     *       while actual sensor reads occur faster in the timer callback.
     */
    void update() override;

    /**
     * @brief Probe for ICM-20789 barometer and create driver instance if found
     * 
     * @details Attempts to detect and initialize an ICM-20789 barometer. This
     *          function performs device detection, reads OTP calibration data via
     *          the IMU bridge, and creates a driver instance if successful.
     * 
     *          Probe sequence:
     *          1. Initialize IMU bridge for barometer access (I2C or SPI)
     *          2. Send barometer reset command through IMU
     *          3. Read OTP calibration constants from barometer
     *          4. Verify calibration data validity
     *          5. Create and register driver instance if all checks pass
     * 
     * @param[in,out] baro       Reference to AP_Baro frontend for registration
     * @param[in]     dev        I2C device interface for barometer portion (address 0x63)
     * @param[in]     dev_imu    Device interface for IMU portion (I2C or SPI) used for bridge access
     * 
     * @return Pointer to new AP_Baro_ICM20789 instance if probe successful, nullptr if failed
     * 
     * @note The dev_imu parameter is critical - it provides the bridge to access
     *       barometer registers through the IMU device interface.
     */
    static AP_Baro_Backend *probe(AP_Baro &baro, AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev, AP_HAL::OwnPtr<AP_HAL::Device> dev_imu);
    
private:
    /**
     * @brief Constructor for ICM-20789 barometer driver
     * 
     * @param[in,out] baro    Reference to AP_Baro frontend
     * @param[in]     dev     I2C device interface for barometer
     * @param[in]     dev_imu Device interface for IMU (provides bridge access)
     */
    AP_Baro_ICM20789(AP_Baro &baro, AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev, AP_HAL::OwnPtr<AP_HAL::Device> dev_imu);

    /**
     * @brief Initialize barometer hardware and start periodic measurements
     * 
     * @details Performs complete initialization sequence:
     *          1. Configure IMU bridge for barometer access
     *          2. Reset barometer to known state
     *          3. Read and validate OTP calibration data
     *          4. Register timer callback for periodic measurements
     * 
     * @return true if initialization successful, false on failure
     */
    bool init();
    
    /**
     * @brief Send 16-bit command to barometer through IMU bridge
     * 
     * @details Commands are sent as two 8-bit bytes through the IMU device
     *          interface to the barometer's command register.
     * 
     * @param[in] cmd 16-bit command value to send
     * 
     * @return true if command sent successfully, false on I2C/SPI error
     */
    bool send_cmd16(uint16_t cmd);

    /**
     * @brief Read OTP calibration data from barometer via IMU bridge
     * 
     * @details Reads factory-programmed One-Time Programmable (OTP) calibration
     *          constants from the barometer. These constants (sensor_constants[4])
     *          are used in the LUT-based pressure/temperature conversion algorithm.
     * 
     *          OTP data includes:
     *          - Pressure sensitivity coefficients
     *          - Temperature compensation factors
     *          - Offset correction values
     * 
     * @return true if calibration data read and validated successfully, false on error
     * 
     * @note Calibration data is read once at initialization and stored for
     *       all subsequent conversions.
     */
    bool read_calibration_data(void);

    /**
     * @brief Convert raw pressure and temperature to calibrated values
     * 
     * @details Applies LUT-based conversion algorithm using OTP calibration constants
     *          to convert raw ADC values to physical units. Results are accumulated
     *          for averaging in the update() method.
     * 
     *          Conversion process:
     *          1. Apply LUT interpolation using calibration constants
     *          2. Calculate pressure in Pascals using quadratic correction
     *          3. Calculate temperature in Celsius using linear correction
     *          4. Accumulate results for later averaging
     * 
     * @param[in] Praw Raw pressure ADC value (20-bit)
     * @param[in] Traw Raw temperature ADC value (20-bit)
     * 
     * @note Output pressure is in Pascals (Pa), temperature in degrees Celsius (°C)
     */
    void convert_data(uint32_t Praw, uint32_t Traw);

    /**
     * @brief Calculate LUT conversion constants from calibration points
     * 
     * @details Computes quadratic curve fit constants (A, B, C) from three
     *          calibration points for LUT-based pressure conversion. Uses the
     *          algorithm provided by InvenSense in the ICM-20789 datasheet.
     * 
     * @param[in]  p_Pa  Array of 3 pressure calibration points in Pascals
     * @param[in]  p_LUT Array of 3 corresponding LUT values
     * @param[out] A     Quadratic coefficient (calculated)
     * @param[out] B     Linear coefficient (calculated)
     * @param[out] C     Constant offset (calculated)
     */
    void calculate_conversion_constants(const float p_Pa[3], const float p_LUT[3],
                                        float &A, float &B, float &C);
    
    /**
     * @brief Calculate calibrated pressure from raw sensor values
     * 
     * @details Applies complete LUT-based conversion algorithm using OTP calibration
     *          constants and conversion factors to produce pressure in Pascals.
     * 
     * @param[in] p_LSB Raw pressure value (least significant bits)
     * @param[in] T_LSB Raw temperature value (least significant bits, used for compensation)
     * 
     * @return Calibrated pressure in Pascals (Pa)
     */
    float get_pressure(uint32_t p_LSB, uint32_t T_LSB);

    /**
     * @brief Initialize IMU bridge for SPI-connected barometer access
     * 
     * @details Configures the IMU's internal I2C master (bridge) to access the
     *          barometer portion when the IMU is connected via SPI. Sets up
     *          pass-through mode and configures I2C master registers.
     * 
     * @return true if SPI bridge initialization successful, false on error
     * 
     * @note Required when ICM-20789 IMU is on SPI bus but barometer uses internal I2C
     */
    bool imu_spi_init(void);
    
    /**
     * @brief Initialize IMU bridge for I2C-connected barometer access
     * 
     * @details Configures the IMU's internal bridge to access the barometer
     *          portion when the IMU is connected via I2C. Simpler than SPI
     *          case as both use same physical I2C bus.
     * 
     * @return true if I2C bridge initialization successful, false on error
     */
    bool imu_i2c_init(void);
    
    /**
     * @brief Timer callback for periodic barometer measurements
     * 
     * @details Called at high frequency (typically 50-100 Hz) to read pressure
     *          and temperature data from the sensor. Sends measurement commands,
     *          reads results, converts to physical units, and accumulates for
     *          averaging in the update() method.
     * 
     *          Timer sequence:
     *          1. Send measurement start command
     *          2. Wait for conversion complete (timing-based)
     *          3. Read raw pressure and temperature registers via IMU bridge
     *          4. Convert and accumulate results
     * 
     * @note Runs in timer interrupt context - must be fast and non-blocking
     */
    void timer(void);
    
    /**
     * @brief OTP (One-Time Programmable) calibration constants from sensor
     * 
     * @details Factory-programmed calibration values read from barometer at
     *          initialization. These 4 constants are used in the LUT-based
     *          conversion algorithm to correct pressure and temperature readings.
     *          Values are sensor-specific and stored in non-volatile memory.
     */
    int16_t sensor_constants[4];

    /** @brief AP_Baro frontend instance number for this sensor */
    uint8_t instance;

    /** @brief I2C device interface for barometer portion (address 0x63) */
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev;
    
    /**
     * @brief Device interface for IMU portion (I2C or SPI)
     * 
     * @details Provides bridge access to barometer registers. All barometer
     *          communication is routed through this IMU device interface using
     *          the ICM-20789's internal I2C master/bridge mechanism.
     */
    AP_HAL::OwnPtr<AP_HAL::Device> dev_imu;

    /**
     * @brief Timestamp of last measurement command in microseconds
     * 
     * @details Used to track measurement timing and ensure adequate conversion
     *          time before reading results. ICM-20789 requires specific delays
     *          between command and data ready.
     */
    uint32_t last_measure_us;

    /**
     * @brief Accumulation structure for averaging multiple measurements
     * 
     * @details Stores sum of temperature and pressure readings from timer callback.
     *          Protected by device semaphore for thread-safe access between
     *          timer interrupt and main update() method.
     * 
     * @note Temperature sum in degrees Celsius, pressure sum in Pascals
     */
    struct {
        float tsum;      ///< Sum of temperature readings (°C)
        float psum;      ///< Sum of pressure readings (Pa)
        uint32_t count;  ///< Number of accumulated samples
    } accum;

    /**
     * @name LUT (Look-Up Table) Conversion Constants
     * @brief Constants for pressure/temperature conversion algorithm
     * 
     * @details The ICM-20789 uses a LUT-based conversion algorithm rather than
     *          simple linear scaling. These constants define the calibration
     *          points and scaling factors for the piecewise conversion.
     * 
     *          Conversion algorithm from InvenSense datasheet:
     *          - Three pressure calibration points define LUT boundaries
     *          - Quadratic interpolation between points
     *          - Temperature compensation applied
     *          - Final output in Pascals and Celsius
     * 
     * @note Algorithm and constants provided by InvenSense in datasheet
     *       with Python reference implementation
     * @{
     */
    
    /** @brief Three pressure calibration points in Pascals for LUT boundaries */
    const float p_Pa_calib[3] = {45000.0, 80000.0, 105000.0};
    
    /** @brief Lower LUT boundary value (3.5 * 2^20) for pressure mapping */
    const float LUT_lower = 3.5 * (1U<<20);
    
    /** @brief Upper LUT boundary value (11.5 * 2^20) for pressure mapping */
    const float LUT_upper = 11.5 * (1U<<20);
    
    /** @brief Quadratic scaling factor (1 / 2^24) for pressure conversion */
    const float quadr_factor = 1 / 16777216.0;
    
    /** @brief Offset scaling factor (2048.0) for temperature/pressure correction */
    const float offst_factor = 2048.0;
    
    /** @} */ // End of LUT conversion constants group
};

#endif  // AP_BARO_ICM20789_ENABLED
