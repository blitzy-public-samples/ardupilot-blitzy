/**
 * @file AP_Baro_BMP085.h
 * @brief Bosch BMP085/BMP180 barometer driver for ArduPilot
 * 
 * @details This driver implements support for the legacy Bosch BMP085 and BMP180
 *          I2C pressure sensors. These sensors provide barometric pressure and
 *          temperature measurements for altitude estimation.
 *          
 *          The BMP085/BMP180 requires sequential temperature and pressure readings
 *          with conversion delays, implemented via a state machine.
 * 
 * @note This is a legacy driver for older BMP085/BMP180 sensors. For new designs,
 *       consider more modern barometer options like BMP280, BMP388, or MS5611.
 * 
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

#include "AP_Baro_Backend.h"

#if AP_BARO_BMP085_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_HAL/utility/OwnPtr.h>
#include <Filter/Filter.h>

/// Default I2C address for BMP085/BMP180 sensors
#ifndef HAL_BARO_BMP085_I2C_ADDR
#define HAL_BARO_BMP085_I2C_ADDR        (0x77)
#endif

/**
 * @class AP_Baro_BMP085
 * @brief Driver for Bosch BMP085 and BMP180 I2C barometric pressure sensors
 * 
 * @details This class implements the barometer backend for BMP085/BMP180 sensors.
 *          These sensors use a piezoresistive pressure sensor and require sequential
 *          temperature and pressure readings with conversion delays.
 *          
 *          Operation:
 *          - State machine cycles: IDLE -> Read Temp -> Read Pressure -> Calculate -> IDLE
 *          - Temperature reading (4.5ms conversion time) updates compensation values
 *          - Pressure reading (25.5ms conversion time at highest resolution) provides raw data
 *          - Compensation algorithm applies factory calibration coefficients
 *          - Pressure values filtered using 10-sample moving average
 *          
 *          Hardware interface:
 *          - I2C communication at default address 0x77
 *          - Optional EOC (End of Conversion) pin for conversion ready detection
 *          - Falls back to timing-based polling if EOC pin not available
 *          
 *          Calibration:
 *          - Factory calibration coefficients (ac1-ac6, b1, b2, mb, mc, md) stored in PROM
 *          - Coefficients read once at initialization
 *          - Applied during temperature and pressure compensation calculations
 * 
 * @note Legacy driver for older sensors; consider modern alternatives for new designs
 */
class AP_Baro_BMP085 : public AP_Baro_Backend {
public:
    /**
     * @brief Construct a new AP_Baro_BMP085 driver instance
     * 
     * @param[in] baro Reference to the main AP_Baro frontend
     * @param[in] dev  Ownership pointer to the I2C device interface
     */
    AP_Baro_BMP085(AP_Baro &baro, AP_HAL::OwnPtr<AP_HAL::Device> dev);

    /**
     * @brief Update barometer readings and publish to frontend
     * 
     * @details Called periodically by the scheduler to:
     *          1. Process state machine transitions
     *          2. Collect temperature and pressure readings
     *          3. Apply compensation and filtering
     *          4. Publish final pressure and temperature to frontend
     *          
     *          This method is non-blocking and uses a state machine to handle
     *          the sequential conversion delays required by the sensor.
     * 
     * @note Called at main loop rate (typically 50-400Hz), but sensor updates
     *       occur at lower rate due to conversion times
     */
    void update() override;

    /**
     * @brief Probe for BMP085/BMP180 sensor and create driver instance
     * 
     * @details Static factory method that attempts to detect and initialize
     *          a BMP085 or BMP180 sensor on the provided I2C device.
     *          
     *          Probe sequence:
     *          1. Attempt communication with sensor
     *          2. Read and validate factory calibration coefficients from PROM
     *          3. Verify sensor responds correctly
     *          4. Create and initialize driver instance if successful
     * 
     * @param[in] baro Reference to the main AP_Baro frontend
     * @param[in] dev  Ownership pointer to the I2C device interface to probe
     * 
     * @return Pointer to new AP_Baro_BMP085 instance on success, nullptr on failure
     * 
     * @note Takes ownership of dev parameter; do not use dev after calling
     */
    static AP_Baro_Backend *probe(AP_Baro &baro, AP_HAL::OwnPtr<AP_HAL::Device> dev);


private:
    // Initialization
    bool _init();

    // State machine command and read methods
    void _cmd_read_pressure();  ///< Issue pressure conversion command (25.5ms conversion)
    void _cmd_read_temp();       ///< Issue temperature conversion command (4.5ms conversion)
    bool _read_pressure();       ///< Read pressure conversion result
    void _read_temp();           ///< Read temperature conversion result
    void _calculate();           ///< Apply compensation algorithm and publish results
    bool _data_ready();          ///< Check if conversion is complete (via EOC pin or timing)

    // Scheduler callback
    void _timer(void);

    // PROM calibration access
    uint16_t _read_prom_word(uint8_t word);  ///< Read single calibration word from PROM
    bool     _read_prom(uint16_t *prom);     ///< Read all calibration coefficients from PROM

    /// I2C device interface - ownership passed in constructor
    AP_HAL::OwnPtr<AP_HAL::Device> _dev;
    
    /**
     * End of Conversion (EOC) GPIO pin
     * 
     * Optional hardware signal indicating conversion complete.
     * If available, provides faster response than timing-based polling.
     * If nullptr, falls back to timing-based conversion detection.
     */
    AP_HAL::DigitalSource *_eoc;

    uint8_t _instance;    ///< Barometer instance number in frontend
    bool _has_sample;     ///< Flag indicating new data ready for frontend

    /**
     * Timing fallback for boards without EOC pin
     * 
     * Stores timestamps of last command issue to calculate when
     * conversion should be complete based on datasheet timing specs.
     */
    uint32_t _last_press_read_command_time;  ///< Timestamp of last pressure conversion start (microseconds)
    uint32_t _last_temp_read_command_time;   ///< Timestamp of last temperature conversion start (microseconds)

    /**
     * State machine variable
     * 
     * Implements sequential conversion cycle:
     * - State 0 (IDLE): Start temperature conversion
     * - State 1: Wait for temperature, then read and start pressure
     * - State 2: Wait for pressure, then read and calculate
     * - Returns to State 0 to repeat cycle
     */
    uint8_t _state;

    /**
     * Factory calibration coefficients from sensor PROM
     * 
     * These coefficients are unique to each sensor and stored in
     * non-volatile memory. They are used in the compensation algorithm
     * to convert raw ADC readings into calibrated pressure and temperature.
     * 
     * Coefficient usage (from BMP085 datasheet):
     * - ac1, ac2, ac3: Pressure compensation coefficients (signed)
     * - ac4, ac5, ac6: Temperature compensation coefficients (unsigned)
     * - b1, b2: Pressure compensation coefficients (signed)
     * - mb, mc, md: Additional compensation coefficients (signed)
     * 
     * Algorithm applies these in temperature compensation first,
     * then uses compensated temperature in pressure compensation.
     */
    int16_t ac1, ac2, ac3, b1, b2, mb, mc, md;  ///< Signed calibration coefficients
    uint16_t ac4, ac5, ac6;                      ///< Unsigned calibration coefficients

    int32_t _raw_pressure;  ///< Raw pressure ADC reading (uncompensated)
    int32_t _raw_temp;      ///< Raw temperature ADC reading (uncompensated)
    int32_t _temp;          ///< Compensated temperature value (used in pressure compensation)
    
    /**
     * Pressure filtering
     * 
     * 10-sample moving average filter to reduce pressure noise.
     * Smooths out sensor noise while maintaining reasonable response time.
     * Filtered pressure published to frontend in Pascals.
     */
    AverageIntegralFilter<int32_t, int32_t, 10> _pressure_filter;

    uint8_t _vers;  ///< Sensor version information
    uint8_t _type;  ///< Sensor type identifier
};

#endif  // AP_BARO_BMP085_ENABLED
