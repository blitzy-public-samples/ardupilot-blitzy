/**
 * @file AP_Baro_BMP388.h
 * @brief Driver for Bosch BMP388/BMP390 high-precision barometric pressure sensors
 * 
 * @details This driver implements support for the Bosch BMP388 and BMP390 barometric
 *          pressure sensors, which provide enhanced precision and lower noise compared
 *          to earlier BMP280/BMP085 sensors. These are high-end sensors designed for
 *          demanding applications including drones, weather stations, and altitude
 *          measurement systems.
 *          
 *          Key features:
 *          - Pressure range: 300-1250 hPa (equivalent to +9000m to -500m above/below sea level)
 *          - Pressure resolution: 0.016 Pa (13 cm at sea level)
 *          - Temperature range: -40 to +85°C
 *          - Low noise: 0.03 Pa RMS
 *          - Interfaces: I2C (addresses 0x76, 0x77) or SPI
 *          
 *          The driver uses a two-stage calibration approach: raw NVM calibration
 *          coefficients are read from the sensor at startup and scaled to floating-point
 *          values for efficient runtime compensation calculations.
 * 
 * @note BMP390 is functionally identical to BMP388 with identical register interface
 * @author ArduPilot Development Team
 */

#pragma once

#include "AP_Baro_Backend.h"

#if AP_BARO_BMP388_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Device.h>
#include <AP_HAL/utility/OwnPtr.h>

/**
 * @brief Primary I2C address for BMP388/BMP390 sensor
 * @details Default I2C address when SDO pin is pulled low (GND)
 */
#ifndef HAL_BARO_BMP388_I2C_ADDR
 #define HAL_BARO_BMP388_I2C_ADDR  (0x76)
#endif

/**
 * @brief Secondary I2C address for BMP388/BMP390 sensor
 * @details Alternative I2C address when SDO pin is pulled high (VDDIO)
 * @note Allows two BMP388 sensors on the same I2C bus
 */
#ifndef HAL_BARO_BMP388_I2C_ADDR2
 #define HAL_BARO_BMP388_I2C_ADDR2 (0x77)
#endif

/**
 * @class AP_Baro_BMP388
 * @brief Driver implementation for Bosch BMP388/BMP390 high-precision barometric pressure sensors
 * 
 * @details This class implements the backend driver for BMP388 and BMP390 sensors,
 *          providing enhanced precision and lower noise compared to earlier BMP280 sensors.
 *          The BMP388/BMP390 are high-end sensors suitable for demanding applications
 *          requiring precise altitude measurement and atmospheric pressure monitoring.
 *          
 *          Driver Architecture:
 *          - Initialization (init): Configures sensor, reads calibration coefficients
 *          - Periodic sampling (timer): Called by HAL scheduler to read raw ADC values
 *          - Compensation: Converts raw ADC to calibrated pressure/temperature
 *          - Accumulation: Averages multiple samples for noise reduction
 *          - Update cycle (update): Provides averaged values to AP_Baro frontend
 *          
 *          Calibration Process:
 *          1. Raw NVM coefficients (calib_p, calib_t) are read from sensor registers
 *          2. scale_calibration_data() converts integer NVM values to floating-point
 *          3. Runtime compensation uses scaled floating-point coefficients (calib)
 *          
 *          Compensation Algorithms:
 *          - Temperature: 3-parameter polynomial compensation (par_t1, par_t2, par_t3)
 *          - Pressure: 11-parameter compensation with temperature linearization (par_p1-par_p11, t_lin)
 *          
 *          Units:
 *          - Pressure output: Pascals (Pa)
 *          - Temperature output: Degrees Celsius (°C)
 *          - Internal calculations use sensor-specific scaling factors
 * 
 * @note Thread safety: timer() runs in scheduler context; update() runs in main thread
 * @warning Sensor requires proper power-up sequence and settling time for accurate readings
 */
class AP_Baro_BMP388 : public AP_Baro_Backend
{
public:
    /**
     * @brief Constructor for BMP388/BMP390 barometer driver
     * 
     * @param[in] baro Reference to the main AP_Baro object
     * @param[in] _dev Unique pointer to the HAL device interface (I2C or SPI)
     * 
     * @note Device ownership is transferred to this object
     */
    AP_Baro_BMP388(AP_Baro &baro, AP_HAL::OwnPtr<AP_HAL::Device> _dev);

    /* AP_Baro public interface: */
    /**
     * @brief Update barometer readings and provide averaged data to frontend
     * 
     * @details Called periodically by the AP_Baro frontend (typically at 10-20 Hz) to
     *          retrieve the latest barometer readings. This method averages the pressure
     *          samples accumulated by the timer() callback since the last update, reducing
     *          measurement noise through oversampling.
     *          
     *          Process:
     *          1. Check if new samples are available (pressure_count > 0)
     *          2. Calculate average pressure: pressure_sum / pressure_count
     *          3. Provide averaged pressure and temperature to frontend via _copy_to_frontend()
     *          4. Reset accumulators (pressure_sum = 0, pressure_count = 0)
     *          
     *          The accumulator pattern allows high-rate sampling in timer() while providing
     *          averaged, lower-rate output to the system, improving signal-to-noise ratio.
     * 
     * @note Must be called from main thread; accesses data written by timer() callback
     * @note Returns immediately if no new samples accumulated since last update
     */
    void update() override;

    /**
     * @brief Static factory method to probe and instantiate BMP388/BMP390 driver
     * 
     * @details Attempts to detect a BMP388 or BMP390 sensor on the provided device interface.
     *          This is the standard ArduPilot driver probe pattern used during sensor auto-detection.
     *          
     *          Probe sequence:
     *          1. Create temporary driver instance
     *          2. Call init() to verify sensor presence via chip ID register read
     *          3. If sensor detected and initialized successfully, return driver instance
     *          4. If detection fails, driver is destroyed and nullptr returned
     *          
     *          Chip ID detection:
     *          - BMP388: chip ID = 0x50
     *          - BMP390: chip ID = 0x60
     *          
     *          Both sensors use identical register interfaces and are handled by this driver.
     * 
     * @param[in] baro Reference to the main AP_Baro object for registration
     * @param[in] _dev Unique pointer to HAL device interface (I2C or SPI) to probe
     * 
     * @return Pointer to initialized AP_Baro_BMP388 instance if sensor detected, nullptr if not found
     * 
     * @note This method transfers device ownership to the created driver instance
     * @note Called during system initialization for each potential barometer device
     */
    static AP_Baro_Backend *probe(AP_Baro &baro, AP_HAL::OwnPtr<AP_HAL::Device> _dev);

private:
    /**
     * @brief Initialize the BMP388/BMP390 sensor
     * @return true if initialization successful, false otherwise
     * @note Verifies chip ID, reads calibration data, configures sensor settings
     */
    bool init(void);
    
    /**
     * @brief Periodic callback to read and process sensor data
     * @details Called by HAL scheduler at high rate (typically 100+ Hz) to read raw ADC values,
     *          apply compensation algorithms, and accumulate samples for averaging.
     *          Reads both pressure and temperature, computes compensated values, and updates accumulators.
     * @note Runs in scheduler/interrupt context
     */
    void timer(void);
    
    /**
     * @brief Update compensated temperature from raw ADC value
     * @param[in] raw_temp Raw 24-bit temperature ADC reading from sensor
     * @details Applies 3-parameter compensation algorithm using par_t1, par_t2, par_t3
     *          and stores linearized temperature value (t_lin) for pressure compensation
     */
    void update_temperature(uint32_t);
    
    /**
     * @brief Update compensated pressure from raw ADC value
     * @param[in] raw_press Raw 24-bit pressure ADC reading from sensor
     * @details Applies 11-parameter compensation algorithm using par_p1 through par_p11
     *          with temperature linearization (t_lin). Result accumulated in pressure_sum.
     */
    void update_pressure(uint32_t);

    /** @brief HAL device interface (I2C or SPI) - ownership held by this driver */
    AP_HAL::OwnPtr<AP_HAL::Device> dev;

    /** @brief Barometer instance number registered with AP_Baro frontend */
    uint8_t instance;
    
    /** @brief Accumulator for pressure samples (Pascals) - summed in timer(), averaged in update() */
    float pressure_sum;
    
    /** @brief Count of accumulated pressure samples - reset after each update() */
    uint32_t pressure_count;
    
    /** @brief Last compensated temperature reading (degrees Celsius) */
    float temperature;

    /**
     * @brief Raw pressure calibration coefficients from sensor NVM (non-volatile memory)
     * @details These integer calibration values are factory-programmed into the BMP388/BMP390
     *          during manufacturing and stored in internal NVM registers starting at address 0x36.
     *          Each sensor has unique calibration values determined during factory calibration.
     *          
     *          The coefficients are read once during init() and converted to floating-point
     *          by scale_calibration_data() for efficient runtime compensation calculations.
     *          
     *          This is the first stage of two-stage calibration: NVM integer values → scaled floats
     */
    struct PACKED {
        int16_t nvm_par_p1; ///< Pressure calibration coefficient P1 (NVM register 0x36-0x37)
        int16_t nvm_par_p2; ///< Pressure calibration coefficient P2
        int8_t nvm_par_p3;  ///< Pressure calibration coefficient P3
        int8_t nvm_par_p4;  ///< Pressure calibration coefficient P4
        int16_t nvm_par_p5; ///< Pressure calibration coefficient P5
        int16_t nvm_par_p6; ///< Pressure calibration coefficient P6
        int8_t nvm_par_p7;  ///< Pressure calibration coefficient P7
        int8_t nvm_par_p8;  ///< Pressure calibration coefficient P8
        int16_t nvm_par_p9; ///< Pressure calibration coefficient P9
        int8_t nvm_par_p10; ///< Pressure calibration coefficient P10
        int8_t nvm_par_p11; ///< Pressure calibration coefficient P11
    } calib_p;

    /**
     * @brief Raw temperature calibration coefficients from sensor NVM (non-volatile memory)
     * @details These integer calibration values are factory-programmed into the BMP388/BMP390
     *          during manufacturing and stored in internal NVM registers starting at address 0x31.
     *          
     *          Temperature compensation is performed first, as the linearized temperature (t_lin)
     *          is required for accurate pressure compensation. The coefficients are read once
     *          during init() and converted to floating-point by scale_calibration_data().
     */
    struct PACKED {
        uint16_t nvm_par_t1; ///< Temperature calibration coefficient T1 (NVM register 0x31-0x32)
        uint16_t nvm_par_t2; ///< Temperature calibration coefficient T2
        int8_t nvm_par_t3;   ///< Temperature calibration coefficient T3
    } calib_t;

    /**
     * @brief Scaled floating-point calibration coefficients for runtime compensation
     * @details This structure contains the calibration coefficients converted from raw NVM
     *          integer values to floating-point, scaled according to the BMP388/BMP390 datasheet
     *          compensation algorithms. The conversion is performed once during initialization
     *          by scale_calibration_data().
     *          
     *          This is the second stage of two-stage calibration: scaled floats used at runtime
     *          
     *          Temperature Compensation Algorithm:
     *          Uses 3 parameters (par_t1, par_t2, par_t3) to convert raw ADC to degrees Celsius
     *          and compute linearized temperature (t_lin) for pressure compensation:
     *            partial_data1 = (raw_temp - par_t1)
     *            partial_data2 = (partial_data1 * par_t2)
     *            t_lin = partial_data2 + (partial_data1² * par_t3)
     *            temperature = t_lin (in °C)
     *          
     *          Pressure Compensation Algorithm:
     *          Uses 11 parameters (par_p1 through par_p11) and t_lin to convert raw ADC
     *          to Pascals with temperature-compensated output. The algorithm applies polynomial
     *          corrections accounting for temperature effects on the piezo-resistive pressure sensor.
     *          
     *          Implementation follows Bosch BMP388/BMP390 datasheet compensation formulas.
     */
    struct {
        float par_t1;   ///< Scaled temperature coefficient T1 for temperature compensation
        float par_t2;   ///< Scaled temperature coefficient T2 for temperature compensation
        float par_t3;   ///< Scaled temperature coefficient T3 for temperature compensation
        float par_p1;   ///< Scaled pressure coefficient P1 for pressure compensation
        float par_p2;   ///< Scaled pressure coefficient P2 for pressure compensation
        float par_p3;   ///< Scaled pressure coefficient P3 for pressure compensation
        float par_p4;   ///< Scaled pressure coefficient P4 for pressure compensation
        float par_p5;   ///< Scaled pressure coefficient P5 for pressure compensation
        float par_p6;   ///< Scaled pressure coefficient P6 for pressure compensation
        float par_p7;   ///< Scaled pressure coefficient P7 for pressure compensation
        float par_p8;   ///< Scaled pressure coefficient P8 for pressure compensation
        float par_p9;   ///< Scaled pressure coefficient P9 for pressure compensation
        float par_p10;  ///< Scaled pressure coefficient P10 for pressure compensation
        float par_p11;  ///< Scaled pressure coefficient P11 for pressure compensation
        float t_lin;    ///< Linearized temperature value used in pressure compensation (intermediate)
    } calib;

    /**
     * @brief Convert raw NVM calibration coefficients to scaled floating-point values
     * @details Performs the two-stage calibration conversion from integer NVM values
     *          (calib_p, calib_t) to floating-point scaled coefficients (calib) according
     *          to the BMP388/BMP390 datasheet compensation algorithm specifications.
     *          
     *          Each raw coefficient is multiplied by a datasheet-specified scaling factor
     *          to produce the floating-point values used in runtime compensation calculations.
     *          This conversion is performed once during init() after reading NVM registers.
     *          
     *          Example scaling transformations (from datasheet):
     *          - par_t1 = nvm_par_t1 / 2^-8  (shift left 8 bits)
     *          - par_p1 = (nvm_par_p1 - 2^14) / 2^20
     *          - Additional sensor-specific scaling factors applied per datasheet
     * 
     * @note Must be called after reading calib_p and calib_t from sensor NVM
     * @note Scaling factors are defined in BMP388/BMP390 datasheet compensation section
     */
    void scale_calibration_data(void);
    
    /**
     * @brief Read multiple registers from the BMP388/BMP390 sensor
     * 
     * @param[in]  reg  Starting register address to read from
     * @param[out] data Buffer to store read data
     * @param[in]  len  Number of bytes to read
     * 
     * @return true if read successful, false on communication error
     * 
     * @note Abstracts I2C/SPI differences via HAL device interface
     */
    bool read_registers(uint8_t reg, uint8_t *data, uint8_t len);
};

#endif  // AP_BARO_BMP388_ENABLED
