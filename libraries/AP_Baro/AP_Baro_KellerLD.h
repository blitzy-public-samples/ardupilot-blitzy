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
 * @file AP_Baro_KellerLD.h
 * @brief Driver for Keller LD series industrial-grade pressure transducers
 * 
 * @details This file implements support for the Keller 4 LD through 9 LD line
 *          of high-accuracy industrial pressure transducers. These sensors
 *          communicate via I2C and are available in various form factors with
 *          measurement ranges from 0-200 bar depending on the model.
 * 
 *          Key Features:
 *          - I2C interface (default address 0x40)
 *          - Industrial-grade accuracy and reliability
 *          - Multiple sensor modes: vented gauge (PR), sealed gauge (PA), absolute (PAA)
 *          - Model-specific calibration stored in device ROM
 *          - Measurement range: 0-200 bar (model dependent)
 * 
 *          Sensor Modes:
 *          - PR_MODE (0): Vented gauge - pressure relative to ambient
 *          - PA_MODE (1): Sealed gauge - pressure relative to sealed reference
 *          - PAA_MODE (2): Absolute pressure measurement
 * 
 *          Protocol: Standard I2C communication with device-specific calibration
 *                    readout and compensated pressure/temperature measurements.
 * 
 * @note Keller LD sensors are high-accuracy industrial pressure transducers
 *       designed for demanding environmental conditions and long-term stability.
 * 
 * @see http://www.keller-druck.com/home_e/paprod_e/4ld_e.asp
 * @see AP_Baro_Backend for the barometer backend interface
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

#include "AP_Baro_Backend.h"

#if AP_BARO_KELLERLD_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Semaphores.h>
#include <AP_HAL/Device.h>

#ifndef HAL_BARO_KELLERLD_I2C_ADDR
#define HAL_BARO_KELLERLD_I2C_ADDR 0x40
#endif

/**
 * @class AP_Baro_KellerLD
 * @brief Barometer backend driver for Keller LD series industrial pressure sensors
 * 
 * @details This class implements the AP_Baro_Backend interface for Keller LD
 *          (4 LD through 9 LD) series pressure transducers. These are high-accuracy
 *          industrial-grade sensors that communicate via I2C and provide compensated
 *          pressure and temperature measurements.
 * 
 *          Driver Lifecycle:
 *          1. Probe: Detect sensor on I2C bus at configured address
 *          2. Init: Read calibration data from device ROM
 *          3. Timer: Periodic sampling via _timer() callback
 *          4. Update: Publish accumulated sensor data to AP_Baro
 * 
 *          Calibration Process:
 *          - Reads sensor mode type (PR/PA/PAA) from device ROM
 *          - Reads pressure measurement range parameters (P_min, P_max)
 *          - Reads mode-specific offset for pressure calculation
 *          - All calibration values stored in non-volatile device memory
 * 
 *          Pressure Calculation:
 *          Raw sensor values are converted to calibrated pressure using:
 *          - Model-specific min/max range (_p_min, _p_max)
 *          - Sensor mode offset (_p_mode_offset)
 *          - Temperature compensation applied internally by sensor
 * 
 *          I2C Communication:
 *          - Default address: 0x40 (configurable via HAL_BARO_KELLERLD_I2C_ADDR)
 *          - Protocol includes delays for EEPROM read operations
 *          - Periodic read of pressure and temperature values
 * 
 *          Units:
 *          - Pressure output: Pascals (Pa)
 *          - Temperature output: Celsius (°C)
 *          - Measurement range: 0-200 bar (model dependent)
 * 
 *          Thread Safety:
 *          - Sensor sampling runs in timer thread via _timer() callback
 *          - Accumulator pattern with atomic updates for thread-safe data sharing
 *          - Main thread reads accumulated samples in update()
 * 
 * @note These sensors are industrial-grade transducers designed for high accuracy
 *       and long-term stability in demanding environmental conditions.
 * 
 * @warning Sensor mode must be correctly identified during initialization for
 *          accurate pressure calculations. Incorrect mode will result in offset errors.
 */
class AP_Baro_KellerLD : public AP_Baro_Backend
{
public:
    /**
     * @brief Update barometer state with accumulated sensor readings
     * 
     * @details Called by the main AP_Baro frontend at regular intervals to publish
     *          accumulated pressure and temperature samples. This method reads the
     *          thread-safe accumulator, calculates averages, and updates the barometer
     *          instance with new pressure (Pa) and temperature (°C) values.
     * 
     *          Accumulator pattern ensures thread-safe data transfer from timer
     *          thread sampling to main thread publishing.
     */
    void update() override;

    /**
     * @brief Probe for Keller LD sensor on I2C bus and create driver instance
     * 
     * @details Attempts to detect a Keller LD pressure sensor on the provided I2C
     *          device. If successful, creates a new AP_Baro_KellerLD instance,
     *          initializes the sensor by reading calibration data from ROM, and
     *          registers a periodic timer callback for sensor sampling.
     * 
     *          Detection sequence:
     *          1. Attempt I2C communication at configured address
     *          2. Read and validate sensor mode type from ROM
     *          3. Read calibration parameters (P_min, P_max, offsets)
     *          4. Register periodic timer callback for continuous sampling
     * 
     * @param[in] baro Reference to AP_Baro frontend for sensor registration
     * @param[in] dev  OwnPtr to I2C device at potential sensor address (typically 0x40)
     * 
     * @return Pointer to new AP_Baro_KellerLD instance if probe successful, nullptr if failed
     * 
     * @note This is a static factory method called during sensor probing phase
     * @note Device ownership transfers to the created backend instance
     */
    static AP_Baro_Backend *probe(AP_Baro &baro, AP_HAL::OwnPtr<AP_HAL::Device> dev);

private:
    /**
     * @brief Private constructor for Keller LD barometer backend
     * 
     * @details Constructs AP_Baro_KellerLD instance with provided I2C device.
     *          Called by probe() method after successful sensor detection.
     * 
     * @param[in] baro Reference to AP_Baro frontend
     * @param[in] dev  OwnPtr to I2C device for sensor communication
     */
    AP_Baro_KellerLD(AP_Baro &baro, AP_HAL::OwnPtr<AP_HAL::Device> dev);

    /**
     * @brief Initialize Keller LD sensor and read calibration data
     * 
     * @details Performs sensor initialization sequence:
     *          1. Reads sensor mode type (PR/PA/PAA) from device ROM
     *          2. Reads pressure measurement range (P_min, P_max) from ROM
     *          3. Reads mode-specific offset for pressure calculation
     *          4. Registers sensor instance with AP_Baro frontend
     *          5. Schedules periodic timer callback for sampling
     * 
     * @return true if initialization and calibration read successful, false on failure
     * 
     * @note Calibration values are permanently stored in sensor ROM
     * @warning Initialization must complete successfully for accurate measurements
     */
    bool _init();

    /**
     * @brief Periodic timer callback for sensor sampling
     * 
     * @details Called at regular intervals (typically 10-20 Hz) by the HAL scheduler
     *          to read pressure and temperature from the Keller LD sensor. Raw values
     *          are accumulated in thread-safe accumulator for later processing by
     *          update() method running in main thread.
     * 
     *          Sampling sequence:
     *          1. Read raw pressure and temperature values via I2C
     *          2. Accumulate samples in _accum structure
     *          3. Wrap accumulator when max_count reached
     * 
     * @note Runs in timer thread context - must be thread-safe
     * @note Accumulator pattern prevents data races with main thread
     */
    void _timer();

    /**
     * @brief Read raw pressure and temperature from Keller LD sensor
     * 
     * @details Performs I2C transaction to read current pressure and temperature
     *          measurements from the sensor. Values are raw counts that require
     *          calibration parameters for conversion to physical units.
     * 
     * @return true if I2C read successful, false on communication error
     * 
     * @note Called from _timer() in timer thread context
     * @note Raw values accumulated before calibration applied
     */
    bool _read();

    /**
     * @brief Accumulate sensor sample and wrap when full
     * 
     * @details Adds pressure and temperature samples to running accumulator.
     *          When accumulator reaches max_count, current sum is processed
     *          and accumulator resets for next batch of samples.
     * 
     * @param[in] pressure    Raw pressure value from sensor (uncalibrated counts)
     * @param[in] temperature Raw temperature value from sensor (uncalibrated counts)
     * @param[in] max_count   Maximum samples before wrapping accumulator
     * 
     * @note Thread-safe accumulation for timer thread to main thread data transfer
     * @note Accumulator wrapping prevents overflow and provides averaging
     */
    void _update_and_wrap_accumulator(uint16_t pressure, uint16_t temperature, uint8_t max_count);

    /// I2C device handle for communication with Keller LD sensor
    AP_HAL::OwnPtr<AP_HAL::Device> _dev;

    /**
     * @brief Thread-safe accumulator for sensor samples
     * 
     * @details Shared data structure between timer thread (sampling) and main
     *          thread (publishing). Accumulates raw sensor readings for averaging.
     * 
     *          sum_pressure: Accumulated raw pressure values (counts, not Pa)
     *          sum_temperature: Accumulated raw temperature values (counts, not °C)
     *          num_samples: Number of samples in current accumulation
     * 
     * @note Raw outputs, not calibrated values - calibration applied during update()
     * @note Access pattern ensures thread safety without explicit locking
     */
    struct {
        uint32_t sum_pressure;      ///< Sum of raw pressure samples
        uint32_t sum_temperature;   ///< Sum of raw temperature samples
        uint8_t num_samples;         ///< Number of accumulated samples
    } _accum;

    /// Barometer instance number in AP_Baro frontend
    uint8_t _instance;

    /**
     * @enum SensorMode
     * @brief Keller LD sensor measurement mode types
     * 
     * @details Defines the pressure reference mode configured in the sensor.
     *          Mode is read from device ROM during initialization and determines
     *          the offset applied to pressure calculations.
     * 
     *          PR_MODE: Vented gauge pressure - measures relative to ambient pressure
     *                   through a vent path. Changes with altitude/weather.
     * 
     *          PA_MODE: Sealed gauge pressure - measures relative to a sealed
     *                   reference cavity. Fixed reference pressure at manufacturing.
     * 
     *          PAA_MODE: Absolute pressure - measures relative to perfect vacuum.
     *                    Most common mode for altimetry applications.
     * 
     * @note Sensor mode is factory-configured and stored in non-volatile ROM
     * @note Incorrect mode interpretation leads to pressure offset errors
     */
    enum class SensorMode {
        PR_MODE = 0,    ///< Vented gauge pressure (relative to ambient)
        PA_MODE = 1,    ///< Sealed gauge pressure (relative to sealed reference)
        PAA_MODE = 2,   ///< Absolute pressure (relative to vacuum)
        UNDEFINED = 3,  ///< Uninitialized or invalid mode
    };

    /// Current sensor measurement mode (read from device ROM during init)
    SensorMode _p_mode;
    
    /**
     * @brief Calibration and measurement range parameters
     * 
     * @details Model-specific calibration values permanently stored in device ROM.
     *          These parameters are read during _init() and used to convert raw
     *          sensor counts to calibrated pressure values in Pascals.
     * 
     *          _p_mode_offset: Pressure offset (bar) specific to sensor mode type.
     *                          Compensates for reference pressure in PA/PR modes.
     * 
     *          _p_min: Minimum pressure (bar) of sensor measurement range.
     *                  Defines lower bound for pressure calculation.
     * 
     *          _p_max: Maximum pressure (bar) of sensor measurement range.
     *                  Defines upper bound (0-200 bar depending on model).
     * 
     * @note All values read from factory calibration stored in sensor EEPROM
     * @note Pressure calculation: P = _p_min + (raw / 65535) * (_p_max - _p_min) + offset
     */
    float _p_mode_offset;  ///< Pressure offset for current sensor mode (bar)
    float _p_min;          ///< Minimum pressure of measurement range (bar)
    float _p_max;          ///< Maximum pressure of measurement range (bar)

    /**
     * @brief Helper methods for reading calibration data from sensor ROM
     * 
     * @details These methods implement the Keller-specific I2C protocol for
     *          reading factory calibration values stored in non-volatile memory.
     */

    /**
     * @brief Perform I2C transfer with required delays for EEPROM access
     * 
     * @details Keller LD sensors require specific timing delays when reading
     *          from internal EEPROM. This helper adds appropriate delays between
     *          write and read operations to ensure reliable calibration readout.
     * 
     * @param[in]  send    Buffer containing bytes to send
     * @param[in]  sendlen Number of bytes to send
     * @param[out] recv    Buffer to receive response bytes
     * @param[in]  recvlen Number of bytes to receive
     * 
     * @return true if I2C transfer successful, false on communication error
     * 
     * @note Required for EEPROM reads during calibration
     * @note Delays are protocol-specific and should not be modified
     */
    bool transfer_with_delays(uint8_t *send, uint8_t sendlen, uint8_t *recv, uint8_t recvlen);
    
    /**
     * @brief Read pressure measurement limit from sensor ROM
     * 
     * @details Reads a floating-point pressure limit value (P_min or P_max)
     *          from sensor EEPROM at specified addresses. Value is stored as
     *          two bytes (MSB and LSB) that are combined into a float.
     * 
     * @param[out] limit    Pointer to store the read pressure limit (bar)
     * @param[in]  msb_addr EEPROM address of most significant byte
     * @param[in]  lsb_addr EEPROM address of least significant byte
     * 
     * @return true if read successful, false on I2C error or invalid data
     * 
     * @note Called during _init() to read _p_min and _p_max
     * @note Uses transfer_with_delays() for proper EEPROM timing
     */
    bool read_measurement_limit(float *limit, uint8_t msb_addr, uint8_t lsb_addr);
    
    /**
     * @brief Read all calibration parameters from sensor ROM
     * 
     * @details Reads complete set of factory calibration data from sensor EEPROM:
     *          - Pressure measurement range (P_min, P_max)
     *          - Mode-specific offset (_p_mode_offset)
     *          All values are permanently stored during manufacturing.
     * 
     * @return true if calibration read successful, false on I2C error
     * 
     * @note Called during _init() after sensor mode determined
     * @note Calibration data is specific to each individual sensor unit
     * @warning Accurate pressure measurements depend on successful calibration read
     */
    bool read_cal();
    
    /**
     * @brief Read and validate sensor mode type from ROM
     * 
     * @details Reads the sensor mode configuration (PR/PA/PAA) from device ROM
     *          and validates it against known mode types. Sets _p_mode member
     *          variable for use in subsequent pressure calculations.
     * 
     * @return true if mode read and validated successfully, false on error
     * 
     * @note First step in _init() sequence - mode must be known before calibration
     * @note Sensor mode is factory-configured and should not change
     * @warning Invalid mode will cause incorrect pressure offset in calculations
     */
    bool read_mode_type();
};


#endif  // AP_BARO_KELLERLD_ENABLED
