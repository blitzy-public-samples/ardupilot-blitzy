/**
 * @file AP_Baro_BMP280.h
 * @brief Bosch BMP280 digital pressure sensor driver
 * 
 * @details This file implements the driver for the Bosch BMP280 barometric
 *          pressure sensor, which supports both I2C and SPI communication
 *          interfaces. The BMP280 is a modern replacement for the BMP180
 *          with improved accuracy, lower power consumption, and enhanced
 *          noise performance.
 * 
 *          Key specifications:
 *          - Pressure range: 300-1100 hPa (9000m to -500m altitude)
 *          - Absolute accuracy: ±1.0 hPa
 *          - Relative accuracy: ±0.12 hPa (±1m altitude)
 *          - Temperature range: -40 to +85°C
 *          - Power consumption: 2.7 μA @ 1Hz sampling
 * 
 * @note The BMP280 requires calibration coefficients stored in non-volatile
 *       memory which are read during initialization and used for temperature
 *       and pressure compensation calculations.
 * 
 * @see AP_Baro_Backend for the barometer backend interface
 */

#pragma once

#include "AP_Baro_Backend.h"

#if AP_BARO_BMP280_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Device.h>
#include <AP_HAL/utility/OwnPtr.h>

// BMP280 supports two possible I2C addresses depending on SDO pin state
#ifndef HAL_BARO_BMP280_I2C_ADDR
 #define HAL_BARO_BMP280_I2C_ADDR  (0x76)  ///< I2C address when SDO pin is low
#endif
#ifndef HAL_BARO_BMP280_I2C_ADDR2
 #define HAL_BARO_BMP280_I2C_ADDR2 (0x77)  ///< I2C address when SDO pin is high
#endif

/**
 * @class AP_Baro_BMP280
 * @brief Driver implementation for Bosch BMP280 barometric pressure sensor
 * 
 * @details This class implements the AP_Baro backend interface for the BMP280
 *          sensor. The driver uses continuous sampling mode with periodic timer
 *          callbacks to read raw ADC values for pressure and temperature, applies
 *          manufacturer-specified compensation algorithms using device-specific
 *          calibration coefficients, and publishes averaged samples to the frontend.
 * 
 *          The driver implements the following workflow:
 *          1. Probe: Detect sensor on I2C/SPI bus and verify chip ID
 *          2. Init: Read calibration coefficients from device NVM
 *          3. Configure: Set oversampling and IIR filter parameters
 *          4. Sample: Periodic timer reads raw ADC values
 *          5. Compensate: Apply datasheet compensation formulas
 *          6. Accumulate: Sum samples for noise reduction
 *          7. Update: Publish averaged pressure and temperature to frontend
 * 
 *          Temperature compensation is performed first to calculate the
 *          intermediate value _t_fine, which is then used in the pressure
 *          compensation algorithm. This ensures proper temperature dependency
 *          correction for pressure measurements.
 * 
 * @note The accumulator pattern (summing samples and averaging in update())
 *       reduces sensor noise and improves altitude estimation accuracy.
 * 
 * @warning Incorrect calibration coefficient handling will result in highly
 *          inaccurate pressure readings and unsafe altitude estimates.
 */
class AP_Baro_BMP280 : public AP_Baro_Backend
{
public:
    /**
     * @brief Construct a new BMP280 barometer backend
     * 
     * @details Creates a BMP280 driver instance for the given device.
     *          The constructor stores the device handle and barometer
     *          frontend reference, but does not initialize hardware.
     *          Hardware initialization occurs in _init().
     * 
     * @param[in] baro Reference to the barometer frontend
     * @param[in] dev  Ownership pointer to the I2C/SPI device interface
     * 
     * @note This constructor is typically called by the probe() factory method
     *       after successful device detection.
     */
    AP_Baro_BMP280(AP_Baro &baro, AP_HAL::OwnPtr<AP_HAL::Device> dev);

    /**
     * @brief Update barometer measurements and publish to frontend
     * 
     * @details This method is called by the frontend at the main loop rate
     *          to publish accumulated pressure and temperature samples. The
     *          driver uses an accumulator pattern where samples are summed
     *          in _timer() callbacks and averaged here to reduce noise.
     * 
     *          Process:
     *          1. Check if new samples are available (_pressure_count > 0)
     *          2. Calculate average pressure from accumulated samples
     *          3. Publish pressure (Pascals) and temperature (Celsius) to frontend
     *          4. Reset accumulators for next averaging period
     * 
     * @note Called at main loop rate (typically 50-400 Hz depending on vehicle)
     * @note Thread-safe: Uses frontend locking mechanisms
     * 
     * @see _timer() for sample acquisition and accumulation
     */
    void update() override;

    /**
     * @brief Probe for BMP280 sensor on the given device interface
     * 
     * @details Static factory method that attempts to detect and initialize
     *          a BMP280 sensor on the provided I2C or SPI device. The probe
     *          sequence verifies the chip ID register (0xD0) contains the
     *          expected value (0x58 for BMP280) and reads calibration
     *          coefficients from non-volatile memory.
     * 
     *          Probe sequence:
     *          1. Read and verify chip ID register
     *          2. Perform soft reset
     *          3. Read calibration coefficients (registers 0x88-0xA1)
     *          4. Configure measurement settings and IIR filter
     *          5. Register periodic timer callback for sampling
     * 
     * @param[in] baro Reference to the barometer frontend
     * @param[in] dev  Ownership pointer to the I2C/SPI device interface to probe
     * 
     * @return Pointer to newly created AP_Baro_BMP280 backend on success,
     *         nullptr if probe fails (wrong chip ID, communication error,
     *         or initialization failure)
     * 
     * @note This method transfers ownership of the device pointer to the
     *       backend instance if probe succeeds
     * @note Probe failure is normal on buses where BMP280 is not present
     */
    static AP_Baro_Backend *probe(AP_Baro &baro, AP_HAL::OwnPtr<AP_HAL::Device> dev);

private:

    /**
     * @brief Initialize BMP280 hardware and read calibration data
     * 
     * @details Performs hardware initialization sequence including soft reset,
     *          reading calibration coefficients from device NVM (registers
     *          0x88-0xA1), configuring measurement parameters (oversampling
     *          rates, IIR filter), and setting up periodic timer callbacks.
     * 
     * @return true if initialization successful, false on communication error
     *         or invalid calibration data
     */
    bool _init(void);

    /**
     * @brief Periodic timer callback to read and process sensor samples
     * 
     * @details This method is called at the configured sampling rate (typically
     *          50-100 Hz) by the HAL scheduler. It reads raw ADC values for
     *          pressure and temperature from the sensor, applies the datasheet-
     *          specified compensation algorithms using calibration coefficients,
     *          and accumulates compensated values for averaging in update().
     * 
     * @note Runs in timer interrupt context - must be fast and non-blocking
     * @note Temperature compensation must be performed before pressure
     *       compensation to calculate _t_fine intermediate value
     * 
     * @see _update_temperature() for temperature compensation algorithm
     * @see _update_pressure() for pressure compensation algorithm
     */
    void _timer(void);

    /**
     * @brief Apply temperature compensation algorithm
     * 
     * @details Implements the BMP280 datasheet temperature compensation formula
     *          using calibration coefficients _t1, _t2, _t3. Calculates the
     *          compensated temperature in Celsius and the intermediate value
     *          _t_fine which is required for pressure compensation.
     * 
     * @param[in] adc_T Raw 20-bit temperature ADC value from sensor
     * 
     * @note Updates _temperature (Celsius) and _t_fine (intermediate value)
     * @note Must be called before _update_pressure() to calculate _t_fine
     * 
     * @see BMP280 datasheet section 3.11.3 for compensation formula
     */
    void _update_temperature(int32_t);

    /**
     * @brief Apply pressure compensation algorithm
     * 
     * @details Implements the BMP280 datasheet pressure compensation formula
     *          using calibration coefficients _p1-_p9 and the intermediate
     *          temperature value _t_fine. Calculates compensated pressure
     *          in Pascals and accumulates samples for averaging.
     * 
     * @param[in] adc_P Raw 20-bit pressure ADC value from sensor
     * 
     * @note Requires _t_fine to be set by prior _update_temperature() call
     * @note Accumulates result in _pressure_sum/_pressure_count for averaging
     * 
     * @see BMP280 datasheet section 3.11.3 for compensation formula
     */
    void _update_pressure(int32_t);

    AP_HAL::OwnPtr<AP_HAL::Device> _dev;  ///< I2C or SPI device interface handle

    uint8_t _instance;  ///< Barometer instance number registered with frontend

    /**
     * Intermediate temperature value used in pressure compensation.
     * Calculated by temperature compensation algorithm and used by
     * pressure compensation to correct for temperature dependency.
     */
    int32_t _t_fine;

    /**
     * Accumulator for pressure samples (Pascals).
     * Samples from _timer() callbacks are summed here and averaged in update().
     * This reduces sensor noise and improves altitude estimation accuracy.
     */
    float _pressure_sum;

    /**
     * Count of accumulated pressure samples.
     * Reset to zero after averaging in update().
     */
    uint32_t _pressure_count;

    /**
     * Most recent compensated temperature in Celsius.
     * Updated by _update_temperature() from raw ADC readings.
     */
    float _temperature;

    /**
     * Calibration coefficients read from sensor non-volatile memory.
     * These device-specific values are programmed during manufacturing
     * and stored in registers 0x88-0xA1. They are used in the datasheet-
     * specified compensation algorithms to convert raw ADC values to
     * calibrated pressure and temperature.
     * 
     * Temperature coefficients: _t1 (unsigned), _t2, _t3 (signed)
     * Pressure coefficients: _p1 (unsigned), _p2-_p9 (signed)
     * 
     * @warning Using incorrect calibration values will result in highly
     *          inaccurate pressure and temperature readings.
     * 
     * @see BMP280 datasheet section 3.11.2 for calibration register mapping
     */
    int16_t _t2, _t3, _p2, _p3, _p4, _p5, _p6, _p7, _p8, _p9;
    uint16_t _t1, _p1;
};

#endif  // AP_BARO_BMP280_ENABLED
