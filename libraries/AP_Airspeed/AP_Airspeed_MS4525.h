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
 * @file AP_Airspeed_MS4525.h
 * @brief MS4525 I2C differential pressure sensor backend driver
 *
 * @details This file implements the backend driver for TE Connectivity MS4525DO 
 *          digital differential pressure sensors connected via I2C bus. The MS4525DO
 *          sensor family provides high-resolution airspeed measurements for fixed-wing
 *          aircraft and other applications requiring differential pressure sensing.
 *
 *          The driver implements a periodic measurement cycle:
 *          1. Send measurement command via I2C
 *          2. Wait 10ms for conversion
 *          3. Read 4-byte response (status + 14-bit pressure + 11-bit temperature)
 *          4. Apply voltage correction and averaging
 *
 *          Sensor features:
 *          - 14-bit pressure ADC resolution
 *          - 11-bit temperature ADC resolution  
 *          - Multiple I2C addresses supported (0x28, 0x36, 0x46)
 *          - Supply voltage correction for improved accuracy
 *          - ~50Hz update rate with averaging
 *
 * @note Conversion formulas and calibration data from MS4525DO datasheet
 *
 * Source: libraries/AP_Airspeed/AP_Airspeed_MS4525.h:1-71
 * Source: libraries/AP_Airspeed/AP_Airspeed_MS4525.cpp:1-284
 */

#pragma once

#include "AP_Airspeed_config.h"

#if AP_AIRSPEED_MS4525_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <utility>

#include "AP_Airspeed_Backend.h"

/**
 * @class AP_Airspeed_MS4525
 * @brief Backend driver for MS4525DO I2C differential pressure sensor
 *
 * @details This class implements the AP_Airspeed_Backend interface for the TE Connectivity
 *          MS4525DO digital differential pressure sensor. The sensor is accessed via I2C
 *          and provides high-resolution measurements for airspeed calculation.
 *
 *          Communication Protocol:
 *          The driver uses a command-response protocol over I2C:
 *          1. Write single-byte (0x00) measurement command
 *          2. Wait minimum 10ms for analog-to-digital conversion
 *          3. Read 4-byte response containing status, pressure, and temperature
 *          4. Parse 14-bit pressure value and 11-bit temperature value
 *
 *          Data Format (4 bytes read from sensor):
 *          - Byte 0: [7:6] = Status, [5:0] = Pressure MSB
 *          - Byte 1: Pressure LSB (total 14 bits)
 *          - Byte 2: [7:3] = Temperature MSB
 *          - Byte 3: [7:5] = Temperature LSB (total 11 bits)
 *
 *          Status codes (bits 7:6 of first byte):
 *          - 00: Normal operation, valid data
 *          - 01: Reserved
 *          - 10: Stale data (not updated since last read)
 *          - 11: Fault detected
 *
 *          Measurement Cycle:
 *          The periodic timer callback (_timer) runs at ~50Hz and orchestrates the
 *          measurement sequence. It maintains a state machine alternating between
 *          starting measurements and collecting results, ensuring the required 10ms
 *          conversion time between operations.
 *
 *          Averaging and Filtering:
 *          Multiple samples are accumulated between get_differential_pressure() calls.
 *          The driver maintains running sums and counts, computing the average when
 *          pressure or temperature is requested. This reduces measurement noise and
 *          provides more stable airspeed readings.
 *
 *          Voltage Correction:
 *          The MS4525DO sensor output varies with supply voltage. The driver applies
 *          empirically-determined correction factors to compensate for voltage
 *          variations from the nominal 5V supply, improving accuracy across different
 *          power supply conditions.
 *
 *          Coordinate System:
 *          Positive differential pressure indicates dynamic port pressure exceeds
 *          static port pressure (normal forward flight). The conversion equation
 *          is inverted so bottom port = static, top port = dynamic.
 *
 * @note Typical update rate is 50Hz via HAL scheduler periodic callback
 * @note Data older than 100ms is considered stale and rejected
 * @note Supports multiple sensor variants with different pressure ranges (PSI_range parameter)
 *
 * @warning Voltage correction is critical for accurate measurements at varying supply voltages
 * @warning Sensor must complete 10ms conversion time before data collection
 *
 * @see AP_Airspeed_Backend for base class interface
 * @see TE Connectivity MS4525DO datasheet for pressure/temperature conversion equations
 *
 * Source: libraries/AP_Airspeed/AP_Airspeed_MS4525.cpp:38-101 (initialization)
 * Source: libraries/AP_Airspeed/AP_Airspeed_MS4525.cpp:142-206 (data collection)
 */
class AP_Airspeed_MS4525 : public AP_Airspeed_Backend
{
public:
    /**
     * @brief Constructor using base class constructor
     *
     * @details Inherits constructor from AP_Airspeed_Backend, which registers
     *          this instance with the frontend and initializes base parameters.
     *
     * @param[in] frontend Pointer to the airspeed frontend instance
     * @param[in] instance Sensor instance number for multi-sensor configurations
     *
     * @note Uses C++11 inheriting constructor syntax
     */
    using AP_Airspeed_Backend::AP_Airspeed_Backend;

    /**
     * @brief Destructor - cleanup I2C device handle
     *
     * @details Releases the I2C device pointer allocated during sensor probing.
     *          The HAL I2C device manager handles cleanup of the underlying
     *          I2C device resources.
     *
     * @note Automatically called when backend instance is destroyed
     */
    ~AP_Airspeed_MS4525(void) {
        delete _dev;
    }

    /**
     * @brief Probe I2C bus for MS4525DO sensor and initialize if found
     *
     * @details Attempts to detect the MS4525DO sensor on configured I2C buses
     *          at known addresses (0x28, 0x36, 0x46). Performs the following:
     *          1. If bus configured: Probe only that bus at all addresses
     *          2. If bus not configured: Probe all external buses, then internal
     *          3. For each candidate, send measurement command and verify response
     *          4. Register periodic timer callback at 50Hz (~20ms period)
     *          5. Configure device type and bus ID for logging/diagnostics
     *
     *          The probing process includes multiple retries (10 during probe,
     *          reduced to 2 for runtime) to handle bus contention and transient
     *          communication errors during boot.
     *
     * @return true if sensor found and initialized successfully, false otherwise
     *
     * @note Called once during ArduPilot startup from airspeed frontend
     * @note Sends GCS message indicating success/failure with bus and address
     * @note Sets up 50Hz timer callback for continuous measurement updates
     *
     * Source: libraries/AP_Airspeed/AP_Airspeed_MS4525.cpp:57-101
     */
    bool init() override;

    /**
     * @brief Return the current averaged differential pressure
     *
     * @details Retrieves the accumulated and averaged differential pressure
     *          since the last call. The driver continuously collects samples
     *          via the periodic timer callback and accumulates them in _press_sum.
     *          This method computes the average, resets the accumulator, and
     *          returns the result.
     *
     *          Freshness check: Data older than 100ms is considered stale and
     *          the method returns false to indicate invalid data. This timeout
     *          detects sensor communication failures or timer callback issues.
     *
     *          Thread safety: Uses semaphore to protect shared accumulators
     *          from concurrent access by timer callback.
     *
     * @param[out] pressure Averaged differential pressure in Pascals
     *
     * @return true if fresh data available (< 100ms old), false if stale or no data
     *
     * @note Called by airspeed frontend at main loop rate (typically 50Hz)
     * @note Positive pressure indicates dynamic port pressure exceeds static port
     * @note Averaging reduces noise but may introduce slight lag in dynamic conditions
     *
     * Source: libraries/AP_Airspeed/AP_Airspeed_MS4525.cpp:245-262
     */
    bool get_differential_pressure(float &pressure) override;

    /**
     * @brief Return the current averaged temperature
     *
     * @details Retrieves the accumulated and averaged temperature readings
     *          since the last call. Temperature is measured simultaneously with
     *          pressure and accumulated in _temp_sum. This method computes the
     *          average, resets the accumulator, and returns the result.
     *
     *          Temperature data is used for:
     *          - Pressure compensation in some airspeed calculations
     *          - Diagnostic purposes (detecting sensor freezing or overheating)
     *          - Voltage correction algorithm (temperature-dependent offset)
     *
     *          Freshness check: Same 100ms timeout as pressure data.
     *
     * @param[out] temperature Averaged temperature in degrees Celsius
     *
     * @return true if fresh data available (< 100ms old), false if stale or no data
     *
     * @note Called by airspeed frontend when temperature data is requested
     * @note Temperature has 11-bit resolution (~0.2°C per LSB)
     * @note Valid range approximately -50°C to +150°C per datasheet
     *
     * Source: libraries/AP_Airspeed/AP_Airspeed_MS4525.cpp:264-281
     */
    bool get_temperature(float &temperature) override;

private:
    /**
     * @brief Initiate pressure and temperature conversion on sensor
     *
     * @details Sends single-byte (0x00) measurement command to sensor via I2C.
     *          This triggers the sensor to begin analog-to-digital conversion
     *          of both pressure and temperature. The conversion requires minimum
     *          10ms to complete before valid data can be read.
     *
     *          Sets _measurement_started_ms timestamp if command successful,
     *          which is used by _timer() to enforce 10ms wait before collection.
     *
     * @note Called by _timer() to start each measurement cycle
     * @note No return value - failure is detected in subsequent _collect() call
     *
     * Source: libraries/AP_Airspeed/AP_Airspeed_MS4525.cpp:104-111
     */
    void _measure();

    /**
     * @brief Read and parse sensor data from I2C
     *
     * @details Reads 4-byte response from sensor containing status bits,
     *          14-bit pressure ADC value, and 11-bit temperature ADC value.
     *          Performs comprehensive data validation:
     *          - Status bits check (reject if stale or fault)
     *          - Range check (reject min/max values indicating bus errors)
     *          - Double-read verification (two reads must agree within 0xFF)
     *
     *          Data format (4 bytes):
     *          Byte 0: [7:6]=status, [5:0]=pressure[13:8]
     *          Byte 1: pressure[7:0]
     *          Byte 2: [7:3]=temperature[10:6]
     *          Byte 3: [7:5]=temperature[5:3]
     *
     *          Valid data is converted to engineering units, optionally voltage-
     *          corrected, and accumulated into _press_sum/_temp_sum for averaging.
     *
     * @note Called by _timer() after 10ms conversion delay
     * @note Clears _measurement_started_ms to indicate measurement complete
     * @note Updates _last_sample_time_ms for freshness checking
     *
     * Source: libraries/AP_Airspeed/AP_Airspeed_MS4525.cpp:142-206
     */
    void _collect();

    /**
     * @brief Periodic callback orchestrating measurement/collection cycle
     *
     * @details HAL scheduler invokes this method at ~50Hz (20ms period).
     *          Implements simple state machine:
     *          - If no measurement in progress: call _measure() to start one
     *          - If measurement started >10ms ago: call _collect() then _measure()
     *
     *          The 10ms delay ensures ADC conversion completes before reading
     *          data. State is tracked via _measurement_started_ms timestamp.
     *
     * @note Registered via _dev->register_periodic_callback() in init()
     * @note Called from HAL scheduler interrupt context with semaphore held
     * @note Update rate: 20000 microseconds = 50Hz
     *
     * Source: libraries/AP_Airspeed/AP_Airspeed_MS4525.cpp:232-243
     */
    void _timer();

    /**
     * @brief Apply supply voltage compensation to pressure and temperature
     *
     * @details MS4525DO sensor output has empirically-measured dependence on
     *          supply voltage. This method applies piecewise linear correction
     *          to compensate for voltage deviations from nominal 5V.
     *
     *          Correction parameters (empirically determined):
     *          - Pressure slope: 65.0 Pa per Volt
     *          - Temperature slope: 0.887 °C per Volt
     *          - Valid voltage range: 4.3V to 5.5V (clamped)
     *
     *          Correction formula:
     *          voltage_diff = board_voltage - 5.0V (clamped to ±0.7V, ±0.5V)
     *          corrected_pressure = raw_pressure - (voltage_diff * 65.0)
     *          corrected_temperature = raw_temperature - (voltage_diff * 0.887)
     *
     * @param[in,out] diff_press_pa Differential pressure in Pascals (modified in-place)
     * @param[in,out] temperature Temperature in degrees Celsius (modified in-place)
     *
     * @note Only applied if disable_voltage_correction() parameter is false
     * @note Empirical data: http://uav.tridgell.net/MS4525/MS4525-offset.png
     * @note Critical for accuracy when board voltage varies under load
     *
     * Source: libraries/AP_Airspeed/AP_Airspeed_MS4525.cpp:208-229
     */
    void _voltage_correction(float &diff_press_pa, float &temperature);

    /**
     * @brief Convert raw 14-bit ADC value to differential pressure in Pascals
     *
     * @details Applies MS4525DO transfer function from datasheet page 4.
     *          The equation inverts the sensor's output relationship:
     *          
     *          diff_press_PSI = -((dp_raw - 0.1*16383) * (P_max-P_min) / (0.8*16383) + P_min)
     *          diff_press_Pa = diff_press_PSI * 6894.757
     *
     *          Where:
     *          - dp_raw: 14-bit ADC reading (0 to 16383)
     *          - P_max: Maximum pressure range in PSI (from parameter)
     *          - P_min: Minimum pressure range = -P_max (symmetric)
     *          - 0.1*16383 and 0.8*16383: Sensor calibration points (10% and 90%)
     *          - 6894.757: Conversion factor PSI to Pascals
     *
     *          Sign convention: Result is negated so positive pressure corresponds
     *          to bottom port = static, top port = dynamic (standard pitot mounting).
     *
     * @param[in] dp_raw Raw 14-bit pressure ADC value (0-16383)
     *
     * @return Differential pressure in Pascals
     *
     * @note Resolution: ~0.03 Pa per LSB for 1 PSI sensor
     * @note Called twice per sample (double-read verification)
     *
     * Source: libraries/AP_Airspeed/AP_Airspeed_MS4525.cpp:113-130
     */
    float _get_pressure(int16_t dp_raw) const;

    /**
     * @brief Convert raw 11-bit ADC value to temperature in degrees Celsius
     *
     * @details Applies MS4525DO temperature transfer function from datasheet:
     *
     *          temperature_C = ((200.0 * dT_raw) / 2047) - 50
     *
     *          Where:
     *          - dT_raw: 11-bit ADC reading (0 to 2047)
     *          - Output range: -50°C to +150°C
     *          - Resolution: ~0.098°C per LSB
     *
     * @param[in] dT_raw Raw 11-bit temperature ADC value (0-2047)
     *
     * @return Temperature in degrees Celsius
     *
     * @note Resolution: ~0.2°C per LSB
     * @note Valid range: approximately -50°C to +150°C
     *
     * Source: libraries/AP_Airspeed/AP_Airspeed_MS4525.cpp:132-139
     */
    float _get_temperature(int16_t dT_raw) const;

    // Accumulator variables for averaging multiple samples
    
    /**
     * @brief Temperature accumulator for averaging
     * @details Running sum of temperature readings (in Celsius) collected by _collect()
     * @note Protected by semaphore for thread-safe access between timer and main thread
     */
    float _temp_sum;

    /**
     * @brief Pressure accumulator for averaging
     * @details Running sum of pressure readings (in Pascals) collected by _collect()
     * @note Protected by semaphore for thread-safe access between timer and main thread
     */
    float _press_sum;

    /**
     * @brief Number of temperature samples in accumulator
     * @details Count of samples added to _temp_sum since last get_temperature() call
     * @note Reset to zero when average is computed and returned
     */
    uint32_t _temp_count;

    /**
     * @brief Number of pressure samples in accumulator
     * @details Count of samples added to _press_sum since last get_differential_pressure() call
     * @note Reset to zero when average is computed and returned
     */
    uint32_t _press_count;

    /**
     * @brief Last computed average temperature
     * @details Cached temperature value in degrees Celsius from most recent averaging calculation
     * @note Updated by get_temperature() when _temp_count > 0
     */
    float _temperature;

    /**
     * @brief Last computed average pressure
     * @details Cached pressure value in Pascals from most recent averaging calculation
     * @note Updated by get_differential_pressure() when _press_count > 0
     */
    float _pressure;

    /**
     * @brief Timestamp of last successful sample collection
     * @details Millisecond timestamp set by _collect() when valid data is obtained.
     *          Used for freshness checking - data older than 100ms is considered stale.
     * @note Compared against AP_HAL::millis() in get_differential_pressure() and get_temperature()
     */
    uint32_t _last_sample_time_ms;

    /**
     * @brief Timestamp when measurement command was sent
     * @details Millisecond timestamp set by _measure() when I2C measurement command succeeds.
     *          Value of 0 indicates no measurement in progress. _timer() checks this to
     *          enforce minimum 10ms conversion delay before calling _collect().
     * @note Reset to 0 by _collect() to indicate measurement complete
     */
    uint32_t _measurement_started_ms;

    /**
     * @brief HAL I2C device interface pointer
     * @details Pointer to I2C device handle obtained from hal.i2c_mgr during probing.
     *          Used for all I2C transfers (measurement command and data reads).
     * @note Allocated during probe(), deleted in destructor
     * @note Provides bus synchronization via get_semaphore()
     */
    AP_HAL::I2CDevice *_dev;

    /**
     * @brief Attempt I2C communication with sensor at specific bus and address
     *
     * @details Tests for sensor presence by:
     *          1. Obtaining I2C device handle for specified bus and address
     *          2. Sending measurement command (_measure)
     *          3. Waiting 10ms for conversion
     *          4. Reading data (_collect)
     *          5. Verifying _last_sample_time_ms was updated (indicating valid response)
     *
     *          Uses high retry count (10) during probing to handle transient
     *          boot-time bus issues. Semaphore held during entire probe sequence.
     *
     * @param[in] bus I2C bus number (0-based, platform-dependent)
     * @param[in] address I2C device address (typically 0x28, 0x36, or 0x46)
     *
     * @return true if sensor responds with valid data, false otherwise
     *
     * @note Called by init() for each candidate bus/address combination
     * @note Sets _dev pointer if successful
     *
     * Source: libraries/AP_Airspeed/AP_Airspeed_MS4525.cpp:38-54
     */
    bool probe(uint8_t bus, uint8_t address);
};

#endif  // AP_AIRSPEED_MS4525_ENABLED
