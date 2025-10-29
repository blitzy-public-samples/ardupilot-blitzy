/**
 * @file AP_Baro_DPS280.h
 * @brief Infineon DPS280 and DPS310 barometric pressure sensor driver
 * 
 * @details This driver implements support for the Infineon DPS280 and DPS310
 *          high-precision digital barometric pressure sensors. These sensors
 *          provide both pressure and temperature measurements with configurable
 *          oversampling for noise reduction.
 *          
 *          Key features:
 *          - I2C and SPI interface support (I2C addresses: 0x76, 0x77)
 *          - 24-bit pressure and temperature measurements
 *          - Configurable oversampling (1x to 128x)
 *          - On-chip calibration coefficients (12-bit to 20-bit packed format)
 *          - Polynomial compensation for accurate readings
 *          - DPS310 variant includes temperature bug workaround
 *          
 *          The driver performs initialization with soft-reset, reads packed
 *          calibration coefficients from device registers, and applies polynomial
 *          compensation per the Infineon datasheet to produce calibrated pressure
 *          (Pascals) and temperature (Celsius) readings.
 *          
 *          Hardware quirks handled:
 *          - Boot sequence timing requirements
 *          - WHOAMI register detection (0x10 for DPS280/DPS310)
 *          - Coefficient unpacking from bit-packed registers
 *          - DPS310 temperature sensor source selection bug
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

#include "AP_Baro_Backend.h"

#if AP_BARO_DPS280_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Device.h>
#include <AP_HAL/utility/OwnPtr.h>

// Default I2C addresses for DPS280/DPS310 (SDO pin determines address selection)
#ifndef HAL_BARO_DPS280_I2C_ADDR
 #define HAL_BARO_DPS280_I2C_ADDR  0x76  ///< I2C address when SDO pulled low
#endif
#ifndef HAL_BARO_DPS280_I2C_ADDR2
 #define HAL_BARO_DPS280_I2C_ADDR2 0x77  ///< I2C address when SDO pulled high
#endif

/**
 * @class AP_Baro_DPS280
 * @brief Barometer driver for Infineon DPS280 and DPS310 sensors
 * 
 * @details This class implements the ArduPilot barometer backend interface for
 *          Infineon DPS280 and DPS310 high-precision digital pressure sensors.
 *          
 *          The driver handles the complete sensor lifecycle:
 *          1. Device detection via WHOAMI register (0x10)
 *          2. Soft-reset for reliable initialization
 *          3. Calibration coefficient reading from packed device registers
 *          4. Configuration of oversampling rates and measurement modes
 *          5. Periodic sampling via timer callback
 *          6. Polynomial compensation using calibration coefficients
 *          7. Averaging and publishing of pressure/temperature data
 *          
 *          Calibration coefficient decoding:
 *          The device stores calibration coefficients in bit-packed format
 *          (12-bit, 16-bit, and 20-bit values) across multiple registers.
 *          The driver unpacks these coefficients and applies two's complement
 *          sign extension where necessary.
 *          
 *          Compensation algorithm:
 *          Pressure and temperature are compensated using polynomial equations
 *          from the Infineon datasheet:
 *          - Temperature compensation: 2nd order polynomial
 *          - Pressure compensation: 3rd order polynomial with temperature cross-terms
 *          
 *          Oversampling configuration:
 *          Higher oversampling reduces noise but increases measurement time.
 *          The driver configures appropriate oversampling rates balancing
 *          noise performance with update frequency requirements.
 *          
 * @note The DPS310 variant (AP_Baro_DPS310 subclass) includes workarounds
 *       for a temperature sensor source selection bug in some hardware revisions.
 * 
 * @warning This driver assumes periodic timer() calls for data acquisition.
 *          Pressure and temperature readings are averaged over multiple samples
 *          before publishing via update().
 */
class AP_Baro_DPS280 : public AP_Baro_Backend {
public:
    /**
     * @brief Construct a new DPS280/DPS310 barometer driver instance
     * 
     * @param[in] baro Reference to the AP_Baro frontend manager
     * @param[in] dev  Ownership pointer to the HAL device (I2C or SPI)
     * 
     * @note Constructor does not perform hardware initialization.
     *       Initialization occurs during the probe sequence.
     */
    AP_Baro_DPS280(AP_Baro &baro, AP_HAL::OwnPtr<AP_HAL::Device> dev);

    /**
     * @brief Update barometer readings and publish to frontend
     * 
     * @details Called periodically by the AP_Baro frontend to publish new
     *          pressure and temperature readings. This method averages the
     *          samples accumulated by timer() callbacks and publishes the
     *          result to the barometer frontend for sensor fusion.
     *          
     *          Publishing includes:
     *          - Averaged pressure in Pascals
     *          - Averaged temperature in Celsius
     *          - Sample health status
     *          
     * @note This is called at main loop rate, while timer() is called at
     *       the configured sampling rate (typically faster).
     */
    void update() override;

    /**
     * @brief Factory method to probe and initialize a DPS280 sensor
     * 
     * @param[in] baro Reference to the AP_Baro frontend manager
     * @param[in] dev  Ownership pointer to the HAL device (I2C or SPI)
     * 
     * @return Pointer to initialized AP_Baro_Backend instance on success, nullptr on failure
     * 
     * @details Convenience wrapper that calls probe() with is_dps310=false to
     *          explicitly probe for DPS280 variant without temperature bug workarounds.
     */
    static AP_Baro_Backend *probe_280(AP_Baro &baro, AP_HAL::OwnPtr<AP_HAL::Device> dev) {
        return probe(baro, std::move(dev), false);
    }

    /**
     * @brief Factory method to probe and initialize a DPS280 or DPS310 sensor
     * 
     * @param[in] baro        Reference to the AP_Baro frontend manager
     * @param[in] dev         Ownership pointer to the HAL device (I2C or SPI)
     * @param[in] _is_dps310  True to enable DPS310-specific workarounds, false for DPS280
     * 
     * @return Pointer to initialized AP_Baro_Backend instance on success, nullptr on failure
     * 
     * @details Performs complete device detection and initialization sequence:
     *          1. Read and verify WHOAMI register (expected: 0x10)
     *          2. Perform soft-reset and wait for boot completion
     *          3. Read and unpack calibration coefficients from device registers
     *          4. Configure oversampling rates and measurement modes
     *          5. Register periodic timer callback for data acquisition
     *          6. Allocate barometer instance in frontend
     *          
     *          The _is_dps310 flag enables temperature sensor source selection
     *          workarounds required for some DPS310 hardware revisions.
     *          
     * @note Probe sequence is non-blocking but may take several milliseconds
     *       due to device reset and coefficient reading operations.
     * 
     * @warning Returns nullptr if WHOAMI check fails, calibration read fails,
     *          or device does not respond within expected timeouts.
     */
    static AP_Baro_Backend *probe(AP_Baro &baro, AP_HAL::OwnPtr<AP_HAL::Device> dev, bool _is_dps310=false);

protected:
    /**
     * @brief Initialize the DPS280/DPS310 sensor hardware
     * 
     * @param[in] _is_dps310 True to enable DPS310-specific workarounds
     * 
     * @return true if initialization successful, false on failure
     * 
     * @details Performs soft-reset, waits for boot, reads calibration
     *          coefficients, configures measurement parameters, and
     *          registers timer callback for periodic sampling.
     */
    bool init(bool _is_dps310);
    
    /**
     * @brief Read and unpack calibration coefficients from device registers
     * 
     * @return true if calibration read successful, false on I2C/SPI error
     * 
     * @details Reads 18 bytes of packed calibration coefficients from device
     *          registers and unpacks them into the calibration structure.
     *          Coefficients are stored in bit-packed format (12-bit, 16-bit,
     *          20-bit) and require two's complement sign extension.
     */
    bool read_calibration(void);
    
    /**
     * @brief Periodic timer callback for data acquisition
     * 
     * @details Called at high rate (typically 50-100Hz) to read pressure and
     *          temperature measurements from the device, apply polynomial
     *          compensation, and accumulate samples for averaging. Health
     *          monitoring is performed to detect sensor failures.
     *          
     * @note This runs in interrupt/timer context and must be fast.
     */
    void timer(void);
    
    /**
     * @brief Calculate compensated pressure and temperature from raw readings
     * 
     * @param[in]  UT          Raw temperature reading (24-bit signed)
     * @param[in]  UP          Raw pressure reading (24-bit signed)
     * @param[out] pressure    Compensated pressure in Pascals
     * @param[out] temperature Compensated temperature in degrees Celsius
     * 
     * @details Applies polynomial compensation equations from Infineon datasheet
     *          using calibration coefficients. Temperature compensation is 2nd
     *          order polynomial. Pressure compensation is 3rd order polynomial
     *          with temperature cross-terms.
     */
    void calculate_PT(int32_t UT, int32_t UP, float &pressure, float &temperature);

    /**
     * @brief Apply two's complement sign extension to 16-bit packed coefficient
     * 
     * @param[in,out] v    Coefficient value to fix
     * @param[in]     bits Number of significant bits (less than 16)
     * 
     * @details Calibration coefficients stored as 12-bit values in 16-bit
     *          registers need sign extension for correct interpretation.
     */
    void fix_config_bits16(int16_t &v, uint8_t bits) const;
    
    /**
     * @brief Apply two's complement sign extension to 32-bit packed coefficient
     * 
     * @param[in,out] v    Coefficient value to fix
     * @param[in]     bits Number of significant bits (less than 32)
     * 
     * @details Calibration coefficients stored as 20-bit values in 32-bit
     *          variables need sign extension for correct interpretation.
     */
    void fix_config_bits32(int32_t &v, uint8_t bits) const;
    
    /**
     * @brief Configure device measurement parameters
     * 
     * @details Sets oversampling rates for pressure and temperature, configures
     *          measurement mode (continuous vs. triggered), and sets data rates.
     *          Balances noise performance with update frequency requirements.
     */
    void set_config_registers(void);
    
    /**
     * @brief Monitor sensor health and handle error conditions
     * 
     * @details Checks for communication errors, sensor timeouts, and invalid
     *          readings. Increments error counter and may trigger soft-reset
     *          if persistent errors detected.
     */
    void check_health();

    AP_HAL::OwnPtr<AP_HAL::Device> dev;  ///< HAL device interface (I2C or SPI)

    uint8_t instance;  ///< Barometer instance number in frontend

    uint32_t count;               ///< Number of accumulated samples for averaging
    uint8_t err_count;            ///< Consecutive error count for health monitoring
    float pressure_sum;           ///< Accumulated pressure for averaging (Pascals)
    float temperature_sum;        ///< Accumulated temperature for averaging (Celsius)
    float last_temperature;       ///< Most recent temperature reading (Celsius)
    bool pending_reset;           ///< Flag indicating soft-reset in progress
    bool is_dps310;               ///< True if DPS310 variant (enables temperature bug workaround)

    /**
     * @struct dps280_cal
     * @brief Calibration coefficients for polynomial pressure/temperature compensation
     * 
     * @details The DPS280/DPS310 stores factory-calibrated compensation coefficients
     *          in bit-packed format across multiple device registers. These coefficients
     *          are used in polynomial equations to convert raw ADC readings to calibrated
     *          pressure (Pascals) and temperature (Celsius) values.
     *          
     *          Coefficient bit widths are specified by the Infineon datasheet and require
     *          two's complement sign extension during unpacking from device registers.
     *          
     *          Compensation equations (simplified):
     *          - Temperature = C0 + C1 * UT_raw (2nd order polynomial)
     *          - Pressure = C00 + C10*T + C01*UP + C11*T*UP + C20*UP² + C21*T*UP² + C30*UP³
     *          
     *          Where UT_raw and UP are raw ADC readings, T is compensated temperature,
     *          and the result is calibrated pressure.
     */
    struct dps280_cal {
        int16_t C0;   ///< Temperature coefficient 0 (12-bit, sign-extended to 16-bit)
        int16_t C1;   ///< Temperature coefficient 1 (12-bit, sign-extended to 16-bit)
        int32_t C00;  ///< Pressure coefficient 00 (20-bit, sign-extended to 32-bit)
        int32_t C10;  ///< Pressure coefficient 10 - pressure/temperature cross-term (20-bit)
        int16_t C01;  ///< Pressure coefficient 01 (16-bit)
        int16_t C11;  ///< Pressure coefficient 11 - pressure/temperature cross-term (16-bit)
        int16_t C20;  ///< Pressure coefficient 20 - 2nd order pressure term (16-bit)
        int16_t C21;  ///< Pressure coefficient 21 - 2nd order cross-term (16-bit)
        int16_t C30;  ///< Pressure coefficient 30 - 3rd order pressure term (16-bit)
        uint8_t temp_source;  ///< Temperature sensor source selection (DPS310 workaround)
    } calibration;  ///< Factory calibration coefficients read from device registers
};

/**
 * @class AP_Baro_DPS310
 * @brief DPS310-specific variant with temperature sensor bug workaround
 * 
 * @details The DPS310 is functionally similar to the DPS280 but some hardware
 *          revisions have a temperature sensor source selection bug that can
 *          cause incorrect temperature readings. This subclass inherits all
 *          DPS280 functionality and enables the temperature bug workaround by
 *          setting the is_dps310 flag during initialization.
 *          
 *          Temperature sensor source selection workaround:
 *          The DPS310 has an internal and external temperature sensor. Some
 *          hardware revisions may not correctly select the appropriate sensor
 *          source, leading to temperature measurement errors. The workaround
 *          forces selection of the correct temperature sensor source during
 *          device configuration.
 *          
 * @note This class uses constructor inheritance from AP_Baro_DPS280.
 *       Only the probe() method is overridden to set is_dps310=true.
 * 
 * @warning Always use AP_Baro_DPS310::probe() for DPS310 hardware, not
 *          AP_Baro_DPS280::probe(), to ensure workarounds are enabled.
 */
class AP_Baro_DPS310 : public AP_Baro_DPS280 {
public:
    using AP_Baro_DPS280::AP_Baro_DPS280;  ///< Inherit constructor from base class
    
    /**
     * @brief Factory method to probe and initialize a DPS310 sensor with workarounds
     * 
     * @param[in] baro Reference to the AP_Baro frontend manager
     * @param[in] dev  Ownership pointer to the HAL device (I2C or SPI)
     * 
     * @return Pointer to initialized AP_Baro_Backend instance on success, nullptr on failure
     * 
     * @details Calls the base class probe() method with is_dps310=true to enable
     *          temperature sensor source selection workaround. Otherwise identical
     *          to AP_Baro_DPS280::probe() initialization sequence.
     */
    static AP_Baro_Backend *probe(AP_Baro &baro, AP_HAL::OwnPtr<AP_HAL::Device> dev);
};


#endif  // AP_BARO_DPS280_ENABLED
