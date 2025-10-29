/**
 * @file AP_Baro_ICP101XX.h
 * @brief TDK InvenSense ICP-101XX barometric pressure sensor driver
 * 
 * @details This driver supports the ICP-101XX series of low-power, high-accuracy
 *          barometric pressure sensors from TDK InvenSense. The ICP-101XX sensors
 *          provide pressure and temperature measurements with excellent stability
 *          and low noise characteristics suitable for altitude estimation in
 *          UAV applications.
 *          
 *          Key Features:
 *          - High accuracy: ±1 Pa (±0.08m altitude)
 *          - Low power consumption
 *          - I2C interface
 *          - Integrated temperature sensor for compensation
 *          - Factory-calibrated sensor constants
 *          
 *          The driver implements sensor initialization, calibration data retrieval,
 *          periodic measurements, and conversion of raw sensor data to calibrated
 *          pressure (Pascals) and temperature (Celsius) values.
 * 
 * @note Pressure measurements are in Pascals (Pa)
 * @note Temperature measurements are in degrees Celsius
 */

#pragma once

#include "AP_Baro_Backend.h"

#if AP_BARO_ICP101XX_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Semaphores.h>
#include <AP_HAL/Device.h>

/**
 * @class AP_Baro_ICP101XX
 * @brief Driver for TDK InvenSense ICP-101XX barometric pressure sensor
 * 
 * @details The ICP-101XX is a low-power, high-accuracy barometric pressure sensor
 *          with integrated temperature compensation. This driver handles I2C
 *          communication, sensor initialization, calibration data management,
 *          and conversion of raw sensor readings to calibrated pressure and
 *          temperature values.
 *          
 *          The sensor provides factory calibration constants that are used with
 *          a proprietary compensation algorithm to achieve high accuracy across
 *          the full operating temperature range.
 *          
 *          Measurement Flow:
 *          1. Initialize sensor and read calibration constants
 *          2. Start periodic measurements via timer callback
 *          3. Read raw pressure and temperature data
 *          4. Apply calibration and compensation algorithms
 *          5. Accumulate samples for averaging
 *          6. Update AP_Baro with filtered results
 *          
 * @note All pressure values are in Pascals (Pa)
 * @note All temperature values are in degrees Celsius
 * @note This driver requires I2C communication
 */
class AP_Baro_ICP101XX : public AP_Baro_Backend
{
public:
    /**
     * @brief Update the sensor readings and push to frontend
     * 
     * @details Called periodically by the AP_Baro frontend to retrieve accumulated
     *          sensor data. This method pulls pressure and temperature samples from
     *          the accumulator (populated by the timer callback), computes averages,
     *          and updates the AP_Baro frontend with the filtered results.
     *          
     *          The accumulator is protected by semaphore to ensure thread-safe
     *          access between the timer callback and this update method.
     * 
     * @note Called at the main loop rate by AP_Baro frontend
     * @note Pressure in Pascals (Pa), temperature in degrees Celsius
     */
    void update() override;

    /**
     * @brief Probe for ICP-101XX sensor on I2C bus and create driver instance
     * 
     * @details Attempts to detect and initialize an ICP-101XX barometric pressure
     *          sensor on the provided I2C device. If successful, creates a new
     *          driver instance, reads sensor calibration data, configures periodic
     *          measurements, and returns the driver instance.
     *          
     *          Probe Sequence:
     *          1. Attempt communication with sensor at I2C address
     *          2. Read and validate factory calibration constants
     *          3. Configure measurement mode and timing
     *          4. Register periodic timer callback for data collection
     *          5. Return driver instance if successful, nullptr if failed
     * 
     * @param[in,out] baro Reference to AP_Baro frontend for sensor registration
     * @param[in]     dev  I2C device handle with configured bus and address
     * 
     * @return Pointer to new AP_Baro_ICP101XX instance if probe successful, nullptr if failed
     * 
     * @note This is called during sensor auto-detection at startup
     * @note Only one instance should be created per physical sensor
     */
    static AP_Baro_Backend *probe(AP_Baro &baro, AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);
    
private:
    /**
     * @brief Private constructor for ICP-101XX driver instance
     * 
     * @details Constructs a new driver instance with the provided I2C device handle.
     *          Called internally by probe() after successful sensor detection.
     *          Initializes member variables and prepares for sensor initialization.
     * 
     * @param[in,out] baro Reference to AP_Baro frontend for sensor registration
     * @param[in]     dev  I2C device handle (ownership transferred to driver)
     * 
     * @note Constructor is private - use probe() to create instances
     */
    AP_Baro_ICP101XX(AP_Baro &baro, AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    /**
     * @brief Initialize the ICP-101XX sensor
     * 
     * @details Performs complete sensor initialization sequence including reading
     *          factory calibration data, calculating conversion constants, and
     *          registering the sensor instance with the AP_Baro frontend.
     * 
     * @return true if initialization successful, false if failed
     * 
     * @note Called once during probe() after constructor
     */
    bool init();
    
    /**
     * @brief Send a 16-bit command to the sensor via I2C
     * 
     * @param[in] cmd 16-bit command code to send
     * 
     * @return true if command sent successfully, false if I2C communication failed
     */
    bool send_cmd16(uint16_t cmd);
    
    /**
     * @brief Read measurement result data from sensor
     * 
     * @param[out] buf Pointer to buffer to store received data
     * @param[in]  len Number of bytes to read
     * 
     * @return true if read successful, false if I2C communication failed
     */
    bool read_measure_results(uint8_t *buf, uint8_t len);
    
    /**
     * @brief Send command and read response from sensor
     * 
     * @details Sends a command to the sensor and reads back the response data.
     *          Includes CRC validation of received data.
     * 
     * @param[in]  cmd 16-bit command code to send
     * @param[out] buf Pointer to buffer to store response data
     * @param[in]  len Number of bytes to read in response
     * 
     * @return true if command and response successful with valid CRC, false if failed
     */
    bool read_response(uint16_t cmd, uint8_t *buf, uint8_t len);
    
    /**
     * @brief Send a command to the sensor without reading response
     * 
     * @param[in] cmd 16-bit command code to send
     * 
     * @return true if command sent successfully, false if I2C communication failed
     */
    bool send_command(uint16_t cmd);
    
    /**
     * @brief Send a command with data payload to the sensor
     * 
     * @param[in] cmd  16-bit command code to send
     * @param[in] data Pointer to data bytes to send with command
     * @param[in] len  Number of data bytes to send
     * 
     * @return true if command and data sent successfully, false if I2C communication failed
     */
    bool send_command(uint16_t cmd, uint8_t *data, uint8_t len);
    
    /**
     * @brief Calculate CRC checksum for sensor communication
     * 
     * @details Implements the CRC algorithm specified by ICP-101XX for validating
     *          communication integrity. Used to verify received calibration data
     *          and measurement results.
     * 
     * @param[in] seed Initial CRC seed value
     * @param[in] data Data byte to include in CRC calculation
     * 
     * @return Calculated CRC value
     * 
     * @note CRC algorithm is specific to ICP-101XX sensor protocol
     */
    int8_t cal_crc(uint8_t seed, uint8_t data);
    
    /**
     * @brief Start a pressure and temperature measurement with specified mode
     * 
     * @details Initiates a measurement cycle on the sensor with the specified
     *          measurement mode, which determines resolution and conversion time.
     * 
     * @param[in] mode Measurement mode code defining resolution/speed tradeoff
     * 
     * @return true if measurement started successfully, false if command failed
     * 
     * @note Different modes trade off measurement time vs accuracy
     */
    bool start_measure(uint16_t mode);
    
    /**
     * @brief Read factory calibration constants from sensor OTP memory
     * 
     * @details Reads the four factory-programmed calibration constants from the
     *          sensor's One-Time Programmable (OTP) memory. These constants are
     *          unique to each sensor and are used in the compensation algorithms
     *          to achieve specified accuracy across temperature range.
     *          
     *          The calibration data includes CRC validation to ensure data integrity.
     * 
     * @return true if calibration data read and validated successfully, false if failed
     * 
     * @note Calibration constants are stored in sensor_constants[] array
     * @note This must be called successfully before measurements can be accurate
     */
    bool read_calibration_data(void);
    
    /**
     * @brief Convert raw sensor readings to calibrated pressure and temperature
     * 
     * @details Applies the ICP-101XX compensation algorithm to convert raw ADC
     *          values (LSB counts) to calibrated pressure in Pascals and temperature
     *          in degrees Celsius. Uses factory calibration constants and proprietary
     *          conversion equations.
     *          
     *          Results are accumulated in the thread-safe accumulator structure
     *          for later averaging in update().
     * 
     * @param[in] Praw Raw pressure ADC value from sensor (LSB counts)
     * @param[in] Traw Raw temperature ADC value from sensor (LSB counts)
     * 
     * @note Output pressure is in Pascals (Pa)
     * @note Output temperature is in degrees Celsius
     * @note Accumulates results in protected accum structure
     */
    void convert_data(uint32_t Praw, uint32_t Traw);
    
    /**
     * @brief Calculate intermediate conversion constants for pressure compensation
     * 
     * @details Computes quadratic fit coefficients (A, B, C) used in the pressure
     *          compensation algorithm. Based on the proprietary InvenSense algorithm
     *          provided in sensor datasheet sample code.
     *          
     *          These constants relate the raw sensor readings to calibrated pressure
     *          values across the measurement range.
     * 
     * @param[in]  p_Pa  Array of 3 reference pressure values in Pascals
     * @param[in]  p_LUT Array of 3 corresponding lookup table values
     * @param[out] A     Quadratic coefficient A
     * @param[out] B     Quadratic coefficient B
     * @param[out] C     Quadratic coefficient C
     * 
     * @note Algorithm derived from InvenSense datasheet Python reference code
     */
    void calculate_conversion_constants(const float p_Pa[3], const float p_LUT[3],
                                        float &A, float &B, float &C);
    
    /**
     * @brief Get compensated pressure from raw sensor values
     * 
     * @details Applies the full ICP-101XX compensation algorithm including temperature
     *          correction to compute calibrated pressure from raw ADC readings.
     *          Uses factory calibration constants and conversion equations.
     * 
     * @param[in] p_LSB Raw pressure ADC value (LSB counts)
     * @param[in] T_LSB Raw temperature ADC value (LSB counts)
     * 
     * @return Compensated pressure in Pascals (Pa)
     * 
     * @note Temperature compensation is critical for accuracy across operating range
     */
    float get_pressure(uint32_t p_LSB, uint32_t T_LSB);
    
    /**
     * @brief Timer callback for periodic sensor reading
     * 
     * @details Called periodically by the HAL scheduler to read sensor data.
     *          Reads raw pressure and temperature values, converts to calibrated
     *          units, and accumulates samples for averaging. Protected by semaphore
     *          for thread-safe accumulation.
     *          
     *          Timer Sequence:
     *          1. Check if measurement ready based on timing
     *          2. Read raw pressure and temperature data from sensor
     *          3. Start next measurement
     *          4. Convert raw data to calibrated values
     *          5. Accumulate in protected structure
     * 
     * @note Called at rate determined by measure_interval
     * @note Uses semaphore to protect accumulator from concurrent access
     * @note Runs in timer thread context, not main loop
     */
    void timer(void);
    
    /**
     * @brief Factory calibration constants from sensor OTP memory
     * 
     * @details Four 16-bit signed calibration constants unique to each sensor,
     *          programmed during factory calibration. These constants are used
     *          in the pressure and temperature compensation algorithms to achieve
     *          the sensor's specified accuracy.
     *          
     *          The constants are read from OTP (One-Time Programmable) memory
     *          during initialization and validated using CRC checksums.
     * 
     * @note Values are sensor-specific and must be read before accurate measurements
     * @note Array indices correspond to specific calibration parameters per datasheet
     */
    int16_t sensor_constants[4];

    /**
     * @brief Sensor instance number in AP_Baro frontend
     * 
     * @details Instance index assigned by AP_Baro frontend when sensor is registered.
     *          Used to update the correct sensor slot in the frontend's sensor array.
     */
    uint8_t instance;

    /**
     * @brief I2C device handle for sensor communication
     * 
     * @details Owned pointer to the I2C device interface used for all sensor
     *          communication. Manages bus access, addressing, and low-level I2C
     *          protocol handling.
     */
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev;

    /**
     * @brief Timestamp of last measurement command in microseconds
     * 
     * @details Records the system time when the last measurement was initiated.
     *          Used to determine when measurement conversion is complete and
     *          results are ready to read (sensor has minimum conversion time).
     * 
     * @note Time in microseconds from system boot (AP_HAL::micros())
     */
    uint32_t last_measure_us;

    /**
     * @brief Accumulation structure for sensor sample averaging
     * 
     * @details Thread-safe accumulator for pressure and temperature samples.
     *          The timer callback accumulates multiple samples, and update()
     *          retrieves the accumulated values for averaging. Protected by
     *          semaphore to ensure thread safety between timer and main loop.
     *          
     *          Structure members:
     *          - tsum: Sum of temperature samples in degrees Celsius
     *          - psum: Sum of pressure samples in Pascals
     *          - count: Number of accumulated samples
     * 
     * @note Protected by device semaphore (WITH_SEMAPHORE pattern)
     * @note Cleared after each update() call
     */
    struct {
        float tsum;   ///< Accumulated temperature sum in degrees Celsius
        float psum;   ///< Accumulated pressure sum in Pascals
        uint32_t count; ///< Number of accumulated samples
    } accum;

    /**
     * @brief Reference calibration pressure points in Pascals
     * 
     * @details Three reference pressure values used in the conversion algorithm
     *          to compute compensation constants. These values define the calibration
     *          points across the sensor's operating range:
     *          - 45000 Pa (~4700m altitude)
     *          - 80000 Pa (~1800m altitude)  
     *          - 105000 Pa (sea level + margin)
     * 
     * @note Values from InvenSense datasheet reference code
     * @note Used with corresponding LUT values to calculate quadratic fit
     */
    const float p_Pa_calib[3] = {45000.0, 80000.0, 105000.0};
    
    /**
     * @brief Lower lookup table value for conversion algorithm
     * 
     * @details Lower bound of sensor's internal lookup table range, used in
     *          pressure compensation calculations. Value is 3.5 * 2^20 in
     *          sensor's internal representation.
     * 
     * @note Constant from InvenSense compensation algorithm
     */
    const float LUT_lower = 3.5 * (1U<<20);
    
    /**
     * @brief Upper lookup table value for conversion algorithm
     * 
     * @details Upper bound of sensor's internal lookup table range, used in
     *          pressure compensation calculations. Value is 11.5 * 2^20 in
     *          sensor's internal representation.
     * 
     * @note Constant from InvenSense compensation algorithm
     */
    const float LUT_upper = 11.5 * (1U<<20);
    
    /**
     * @brief Quadratic scaling factor for pressure conversion
     * 
     * @details Scaling factor used in quadratic pressure compensation equation.
     *          Value is 1/2^24 to scale from 24-bit sensor representation.
     * 
     * @note Constant from InvenSense compensation algorithm
     */
    const float quadr_factor = 1 / 16777216.0;
    
    /**
     * @brief Offset scaling factor for pressure conversion
     * 
     * @details Offset factor used in pressure compensation calculations to
     *          scale and shift values in the conversion equation.
     * 
     * @note Constant from InvenSense compensation algorithm (2^11)
     */
    const float offst_factor = 2048.0;
    
    /**
     * @brief Measurement interval in microseconds between samples
     * 
     * @details Time interval between sensor measurements, determines the rate
     *          at which new pressure and temperature readings are acquired.
     *          Set during initialization based on sensor configuration.
     * 
     * @note Value in microseconds, typically set based on measurement mode
     * @note Affects sample averaging and update rate
     */
    uint32_t measure_interval = 0;
};

#endif  // AP_BARO_ICP101XX_ENABLED
