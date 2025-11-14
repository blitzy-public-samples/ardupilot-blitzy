/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * @file AP_Airspeed_MS5525.h
 * @brief MS5525 I2C differential pressure sensor backend driver
 * 
 * @details This file implements support for the TE Connectivity MS5525DSO
 *          digital barometric pressure sensor family used for airspeed
 *          measurement via differential pressure sensing.
 * 
 *          Key features:
 *          - 24-bit ADC resolution for superior accuracy
 *          - PROM-based factory calibration with CRC4 validation
 *          - 2nd-order temperature compensation for improved accuracy
 *          - I2C interface with dual address support (0x76, 0x77)
 *          - State machine-based conversion management
 * 
 *          Sensor operates in conversion sequence:
 *          PROM read → D1 conversion (pressure) → D2 conversion (temperature) → 
 *          calculate compensated values → repeat
 * 
 *          For detailed sensor specifications and calibration formulas, refer to
 *          TE Connectivity MS5525DSO datasheet.
 * 
 * @note 2nd-order temperature compensation is critical for accurate readings
 *       at temperature extremes (below 20°C or above 45°C)
 * 
 * Source: libraries/AP_Airspeed/AP_Airspeed_MS5525.h:1-85
 * Source: libraries/AP_Airspeed/AP_Airspeed_MS5525.cpp (2nd-order compensation algorithm)
 */
#pragma once

/*
  backend driver for airspeed from I2C
 */

#include "AP_Airspeed_config.h"

#if AP_AIRSPEED_MS5525_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <utility>

#include "AP_Airspeed_Backend.h"

/**
 * @class AP_Airspeed_MS5525
 * @brief MS5525 differential pressure sensor backend driver
 * 
 * @details This class implements the backend driver for the TE Connectivity MS5525DSO
 *          I2C digital barometric pressure sensor used for airspeed measurement.
 * 
 *          The MS5525 provides high-resolution differential pressure measurements
 *          through a 24-bit ADC, enabling accurate airspeed calculations for aircraft.
 * 
 *          Advanced features:
 *          - PROM calibration coefficients (C1-C6) with CRC4 validation
 *          - 2nd-order temperature compensation for extended temperature range accuracy
 *          - Configurable oversampling ratio (OSR) for noise/speed tradeoff
 *          - State machine-based conversion management for efficient I2C usage
 * 
 *          Operation sequence:
 *          1. Initialization: Probe sensor, read and validate PROM calibration data
 *          2. Measurement cycle: Trigger D1 (pressure) conversion
 *          3. Collection: Read 24-bit D1 result
 *          4. Measurement cycle: Trigger D2 (temperature) conversion
 *          5. Collection: Read 24-bit D2 result
 *          6. Calculation: Apply PROM calibration and 2nd-order compensation
 *          7. Repeat from step 2 at ~50Hz
 * 
 *          The state machine ensures proper timing for ADC conversions (~10ms for OSR=4096)
 *          and coordinates measure/collect sequences without blocking.
 * 
 * @note Supports multiple I2C addresses (0x76, 0x77) with auto-detection capability
 * 
 * @warning CRC4 validation failure during initialization indicates corrupted calibration
 *          data - sensor cannot be used reliably in this state
 * 
 * @warning 2nd-order temperature compensation is critical for accurate measurements
 *          at temperature extremes (< 20°C or > 45°C)
 * 
 * @see AP_Airspeed_Backend for base class interface
 */
class AP_Airspeed_MS5525 : public AP_Airspeed_Backend
{
public:
    /**
     * @enum MS5525_ADDR
     * @brief I2C address selection for MS5525 sensor
     * 
     * @details The MS5525 sensor can be configured at two different I2C addresses,
     *          or the driver can automatically scan for the sensor at both addresses.
     */
    enum MS5525_ADDR {
        MS5525_ADDR_1    = 0,   ///< Use I2C address 0x76 (CSB pin grounded)
        MS5525_ADDR_2    = 1,   ///< Use I2C address 0x77 (CSB pin to VDD)
        MS5525_ADDR_AUTO = 255, ///< Auto-detect by scanning both addresses (0x76, 0x77)
    };

    /**
     * @brief Construct MS5525 airspeed sensor backend
     * 
     * @param[in] frontend  Reference to AP_Airspeed frontend object
     * @param[in] _instance Sensor instance number for multi-sensor support
     * @param[in] address   I2C address selection (MS5525_ADDR_1, MS5525_ADDR_2, or MS5525_ADDR_AUTO)
     * 
     * @details Initializes the backend with the specified I2C address configuration.
     *          If MS5525_ADDR_AUTO is specified, the driver will attempt to probe
     *          both possible I2C addresses (0x76 and 0x77) during init().
     */
    AP_Airspeed_MS5525(AP_Airspeed &frontend, uint8_t _instance, MS5525_ADDR address);
    
    /**
     * @brief Destructor for MS5525 airspeed sensor backend
     * 
     * @details Cleans up I2C device interface allocated during initialization.
     */
    ~AP_Airspeed_MS5525(void) {
        delete dev;
    }

    /**
     * @brief Probe sensor and read calibration PROM
     * 
     * @return true if sensor detected, PROM read successfully, and CRC4 validated; false otherwise
     * 
     * @details Initialization sequence:
     *          1. Probe I2C bus at configured address(es) to detect sensor
     *          2. Read 8x 16-bit calibration coefficients from sensor PROM
     *          3. Validate PROM data integrity using CRC4 checksum
     *          4. Register periodic timer callback for measurement state machine (~50Hz)
     * 
     *          The PROM contains factory calibration coefficients (C1-C6) essential
     *          for accurate pressure and temperature compensation calculations.
     * 
     * @note If MS5525_ADDR_AUTO is configured, both 0x76 and 0x77 addresses are probed
     * 
     * @warning Initialization failure typically indicates sensor not present on I2C bus
     *          or corrupted PROM calibration data (CRC4 mismatch)
     */
    bool init() override;

    /**
     * @brief Return current compensated differential pressure
     * 
     * @param[out] pressure Differential pressure in Pascals
     * @return true if fresh data available (within 100ms), false if data stale or unavailable
     * 
     * @details Returns the latest differential pressure measurement after applying:
     *          - PROM calibration coefficients
     *          - 2nd-order temperature compensation
     * 
     *          The 2nd-order compensation algorithm corrects for non-linear temperature
     *          effects, particularly important at temperature extremes (< 20°C or > 45°C).
     * 
     *          Data freshness is validated against 100ms timeout to detect sensor failures.
     * 
     * @note Pressure units are Pascals - frontend converts to airspeed using pitot equation
     * @note Typical update rate is ~50Hz when sensor operating normally
     * 
     * @warning Stale data (return false) indicates sensor communication failure or
     *          timer callback not executing
     */
    bool get_differential_pressure(float &pressure) override;

    /**
     * @brief Return current compensated temperature
     * 
     * @param[out] temperature Temperature in degrees Celsius
     * @return true if data available, false otherwise
     * 
     * @details Returns the sensor die temperature after applying PROM calibration.
     *          Temperature measurement is used both for external reporting and internally
     *          for 2nd-order pressure compensation.
     * 
     * @note Temperature units are degrees Celsius
     * @note Temperature is measured from sensor die, not ambient air temperature
     */
    bool get_temperature(float &temperature) override;

private:
    /**
     * @brief Initiate ADC conversion for pressure (D1) or temperature (D2)
     * 
     * @details Sends I2C command to start ADC conversion based on current state:
     *          - State 0: Start D1 (pressure) conversion
     *          - State 2: Start D2 (temperature) conversion
     * 
     *          Conversion time depends on OSR (oversampling ratio):
     *          - OSR=256:  0.5ms
     *          - OSR=512:  1.1ms
     *          - OSR=1024: 2.1ms
     *          - OSR=2048: 4.1ms
     *          - OSR=4096: 8.2ms (typical, best accuracy/noise tradeoff)
     * 
     * @note Records command send timestamp for timing validation in collect()
     */
    void measure();
    
    /**
     * @brief Read 24-bit ADC result from sensor
     * 
     * @details Reads the completed ADC conversion result via I2C:
     *          - State 1: Collect D1 (pressure) result
     *          - State 3: Collect D2 (temperature) result
     * 
     *          Ensures sufficient conversion time has elapsed since measure() command.
     * 
     * @note ADC results are raw 24-bit digital values requiring calibration
     */
    void collect();
    
    /**
     * @brief Periodic state machine callback for measurement sequencing
     * 
     * @details Orchestrates the complete measurement cycle at ~50Hz:
     *          State 0: Measure D1 (initiate pressure conversion)
     *          State 1: Collect D1 (read pressure result)
     *          State 2: Measure D2 (initiate temperature conversion)
     *          State 3: Collect D2 (read temperature result)
     *          State 4: Calculate (apply calibration and compensation)
     * 
     *          State machine ensures proper ADC conversion timing and efficient
     *          I2C bus usage without blocking.
     * 
     * @note Called by HAL scheduler at periodic interval registered in init()
     * @note Non-blocking design allows other sensor operations to proceed
     */
    void timer();
    
    /**
     * @brief Read 8x 16-bit calibration coefficients from sensor PROM
     * 
     * @return true if PROM read successfully and CRC4 validated, false otherwise
     * 
     * @details Reads factory calibration data from sensor's internal PROM:
     *          - prom[0]: Factory reserved (includes CRC)
     *          - prom[1]: C1 = Pressure sensitivity (SENS_T1)
     *          - prom[2]: C2 = Pressure offset (OFF_T1)
     *          - prom[3]: C3 = Temperature coefficient of pressure sensitivity (TCS)
     *          - prom[4]: C4 = Temperature coefficient of pressure offset (TCO)
     *          - prom[5]: C5 = Reference temperature (T_REF)
     *          - prom[6]: C6 = Temperature coefficient of temperature (TEMPSENS)
     *          - prom[7]: Serial code and CRC
     * 
     *          PROM integrity is validated using CRC4 checksum. Corrupted calibration
     *          data will cause initialization failure.
     * 
     * @warning CRC4 validation failure indicates sensor calibration data corruption -
     *          sensor cannot provide accurate measurements
     */
    bool read_prom(void);
    
    /**
     * @brief Compute CRC4 checksum of PROM calibration data
     * 
     * @return 4-bit CRC value
     * 
     * @details Implements CRC4 algorithm specified in MS5525 datasheet to validate
     *          PROM calibration data integrity. The CRC is stored in the lower 4 bits
     *          of prom[7] and compared against calculated value.
     * 
     * @note CRC validation is essential - corrupted calibration coefficients will
     *       produce incorrect pressure/temperature calculations
     */
    uint16_t crc4_prom(void);
    
    /**
     * @brief Read 24-bit ADC result from sensor
     * 
     * @return 24-bit ADC value (0 to 16777215), or -1 on I2C communication error
     * 
     * @details Reads three bytes from sensor and combines into 24-bit result.
     *          Used by collect() to retrieve both D1 (pressure) and D2 (temperature)
     *          ADC conversion results.
     * 
     * @note Raw ADC values require calibration using PROM coefficients
     */
    int32_t read_adc();
    
    /**
     * @brief Apply PROM calibration and 2nd-order temperature compensation
     * 
     * @details Implements MS5525 calibration algorithm:
     * 
     *          First-order calculations:
     *          1. dT = D2 - C5 * 2^8 (temperature difference from reference)
     *          2. TEMP = 2000 + dT * C6 / 2^23 (actual temperature in centidegrees)
     *          3. OFF = C2 * 2^17 + (C4 * dT) / 2^6 (offset at actual temperature)
     *          4. SENS = C1 * 2^16 + (C3 * dT) / 2^7 (sensitivity at actual temperature)
     *          5. P = (D1 * SENS / 2^21 - OFF) / 2^15 (compensated pressure)
     * 
     *          Second-order compensation (temperature extremes):
     *          - For TEMP < 20°C: Apply low-temperature corrections
     *          - For TEMP > 45°C: Apply high-temperature corrections
     *          - Corrects non-linear effects in OFF and SENS
     * 
     *          Results are accumulated for averaging and stored for retrieval by
     *          get_differential_pressure() and get_temperature().
     * 
     * @note 2nd-order compensation significantly improves accuracy at temperature
     *       extremes compared to first-order calculations alone
     * 
     * @note Formulas reference MS5525DSO datasheet calibration section
     */
    void calculate();

    float pressure;              ///< Last calculated compensated pressure in Pascals
    float temperature;           ///< Last calculated compensated temperature in degrees Celsius
    float temperature_sum;       ///< Accumulator for temperature averaging
    float pressure_sum;          ///< Accumulator for pressure averaging
    uint32_t temp_count;        ///< Number of temperature samples in accumulator
    uint32_t press_count;       ///< Number of pressure samples in accumulator
    
    uint32_t last_sample_time_ms; ///< Timestamp of last successful calculation in milliseconds (for freshness validation, 100ms timeout)

    uint16_t prom[8];           ///< PROM calibration coefficients: [0]=reserved+CRC, [1-6]=C1-C6 calibration values, [7]=serial+CRC4
    uint8_t state;              ///< State machine position: 0=measure D1, 1=collect D1, 2=measure D2, 3=collect D2, 4=calculate
    int32_t D1;                 ///< Raw 24-bit ADC value for differential pressure (0 to 16777215)
    int32_t D2;                 ///< Raw 24-bit ADC value for temperature (0 to 16777215)
    uint32_t command_send_us;   ///< Timestamp of last ADC conversion command in microseconds (for conversion timing validation)
    bool ignore_next;           ///< Flag to skip first reading after PROM initialization (allows sensor to stabilize)
    uint8_t cmd_sent;           ///< Last command sent to sensor (for state tracking and debugging)
    MS5525_ADDR _address;       ///< Configured I2C address mode (MS5525_ADDR_1, MS5525_ADDR_2, or MS5525_ADDR_AUTO)

    AP_HAL::I2CDevice *dev;     ///< HAL I2C device interface pointer for sensor communication
};

#endif  // AP_AIRSPEED_MS5525_ENABLED
