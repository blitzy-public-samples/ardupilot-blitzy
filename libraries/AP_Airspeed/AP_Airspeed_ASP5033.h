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
 * @file AP_Airspeed_ASP5033.h
 * @brief Backend driver for ASP5033 I2C differential pressure sensor
 * 
 * This file implements support for the ASP5033 airspeed sensor from QIO-Tek
 * (www.qio-tek.com). The ASP5033 is a high-speed I2C differential pressure
 * sensor capable of 80Hz update rates, making it suitable for fast-response
 * airspeed measurements in autopilot systems.
 * 
 * Sensor Specifications:
 * - I2C interface at high speed (400kHz)
 * - Dual I2C addresses: 0x6C or 0x6D
 * - 80Hz update rate for low-latency pressure measurements
 * - 24-bit signed pressure data with configurable range
 * - 16-bit signed temperature data (1/256°C resolution)
 * - Continuous measurement mode
 * - Unusual identity verification handshake for sensor validation
 * 
 * Data Format:
 * - Pressure: 24-bit signed twos complement, scaled by k-factor
 * - Temperature: 16-bit signed, units of 1/256 degrees Celsius
 * - Both values read in single 5-byte block for consistency
 * 
 * @see libraries/AP_Airspeed/AP_Airspeed_ASP5033.cpp for implementation details
 * @see ASP5033 datasheet for pressure range k-factor table
 */

#pragma once

#include "AP_Airspeed_config.h"

#if AP_AIRSPEED_ASP5033_ENABLED

#include "AP_Airspeed_Backend.h"
#include <AP_HAL/I2CDevice.h>

/**
 * @class AP_Airspeed_ASP5033
 * @brief Backend driver for ASP5033 I2C differential pressure sensor
 * 
 * @details This class implements the AP_Airspeed_Backend interface for the
 * ASP5033 sensor. Key features of this implementation:
 * 
 * High-Speed Operation:
 * - Operates at 80Hz update rate for fast airspeed response
 * - Uses periodic timer callback for continuous sampling
 * - Accumulates samples and returns averages to reduce noise
 * 
 * Identity Verification:
 * - Implements unique identity check handshake
 * - Writes and verifies part_id register to confirm sensor type
 * - Prevents configuration errors from incorrect sensor detection
 * 
 * Data Acquisition:
 * - 24-bit signed pressure data (twos complement encoding)
 * - 16-bit signed temperature data
 * - Single atomic read of both values for consistency
 * - 100ms timeout for data freshness validation
 * 
 * Thread Safety:
 * - Uses semaphore protection for shared data access
 * - Safe for concurrent calls from multiple threads
 * 
 * Continuous Measurement Mode:
 * - Sensor configured for continuous operation
 * - No power-down between samples
 * - Timer triggers measurement start each cycle
 * 
 * @note The ASP5033 requires specific identity verification sequence that
 * differs from standard I2C sensor probing
 * 
 * @warning Ensure correct I2C bus configuration and pullup resistors for
 * reliable high-speed operation at 80Hz
 * 
 * Source: libraries/AP_Airspeed/AP_Airspeed_ASP5033.h
 * Source: libraries/AP_Airspeed/AP_Airspeed_ASP5033.cpp
 */
class AP_Airspeed_ASP5033 : public AP_Airspeed_Backend
{
public:
    /**
     * @brief Constructor - uses base class constructor
     * 
     * @details Inherits constructor from AP_Airspeed_Backend, which handles
     * airspeed instance configuration and parameter initialization.
     */
    using AP_Airspeed_Backend::AP_Airspeed_Backend;

    /**
     * @brief Destructor - cleanup I2C device resources
     * 
     * @details Releases the I2C device pointer allocated during init().
     * Called automatically when the backend is destroyed.
     */
    ~AP_Airspeed_ASP5033(void) {
        delete dev;
    }

    /**
     * @brief Initialize and probe the ASP5033 sensor
     * 
     * @details Initialization sequence:
     * 1. Probes sensor at possible I2C addresses (0x6C, 0x6D)
     * 2. Configures I2C bus for high-speed operation (400kHz)
     * 3. Verifies sensor identity using unique handshake
     * 4. Sets device type for identification
     * 5. Registers 80Hz periodic timer callback for continuous sampling
     * 
     * The identity verification writes to part_id register and confirms
     * the sensor responds correctly, preventing false detection.
     * 
     * @return true if sensor found and initialized successfully
     * @return false if sensor not found or initialization failed
     * 
     * @note Must be called before get_differential_pressure() or get_temperature()
     * @note Automatically tries both possible I2C addresses
     * 
     * Source: libraries/AP_Airspeed/AP_Airspeed_ASP5033.cpp:42-70
     */
    bool init() override;

    /**
     * @brief Get the current differential pressure reading
     * 
     * @details Returns the averaged differential pressure from accumulated
     * samples since last call. The sensor runs at 80Hz and accumulates
     * readings which are averaged to reduce noise. After returning the
     * average, the accumulator is reset for the next period.
     * 
     * Data is considered stale if no samples received within 100ms.
     * 
     * @param[out] _pressure Differential pressure in Pascals
     * 
     * @return true if fresh pressure data available (within 100ms)
     * @return false if no recent samples (sensor communication failure)
     * 
     * @note Thread-safe: uses semaphore protection
     * @note Returns last valid reading if no new samples accumulated
     * @note Units: Pascals (Pa)
     * 
     * Source: libraries/AP_Airspeed/AP_Airspeed_ASP5033.cpp:140-159
     */
    bool get_differential_pressure(float &_pressure) override;

    /**
     * @brief Get the current temperature reading
     * 
     * @details Returns the averaged temperature from accumulated samples
     * since last call. Temperature is measured simultaneously with pressure
     * and accumulated at 80Hz rate. After returning the average, the
     * accumulator is reset for the next period.
     * 
     * Data is considered stale if no samples received within 100ms.
     * 
     * @param[out] _temperature Temperature in degrees Celsius
     * 
     * @return true if fresh temperature data available (within 100ms)
     * @return false if no recent samples (sensor communication failure)
     * 
     * @note Thread-safe: uses semaphore protection
     * @note Returns last valid reading if no new samples accumulated
     * @note Units: Degrees Celsius (°C)
     * @note Resolution: 1/256°C from sensor
     * 
     * Source: libraries/AP_Airspeed/AP_Airspeed_ASP5033.cpp:162-179
     */
    bool get_temperature(float &_temperature) override;

private:
    /**
     * @brief Periodic timer callback for sensor reading at 80Hz
     * 
     * @details This function is called every 12.5ms (80Hz) to read sensor data.
     * 
     * Operation sequence:
     * 1. Sends measure command to trigger new conversion
     * 2. Checks sensor ready status
     * 3. Reads 5-byte data block (3 bytes pressure + 2 bytes temperature)
     * 4. Decodes 24-bit signed pressure (twos complement)
     * 5. Decodes 16-bit signed temperature (1/256°C units)
     * 6. Accumulates readings for averaging
     * 7. Updates freshness timestamp
     * 
     * Pressure Decoding:
     * - Raw data is 24-bit signed twos complement
     * - Scaled by k-factor from datasheet (k=7 for typical range)
     * - Final value = raw * (1 / 2^k) Pascals
     * 
     * Temperature Decoding:
     * - Raw data is 16-bit signed
     * - Units of 1/256 degrees Celsius
     * - Final value = raw * (1/256) °C
     * 
     * @note Called from HAL scheduler at 80Hz rate
     * @note Thread-safe: uses semaphore protection for shared data
     * @note High frequency enables fast airspeed response
     * 
     * Source: libraries/AP_Airspeed/AP_Airspeed_ASP5033.cpp:96-136
     */
    void timer();

    /**
     * @brief Verify sensor identity using unique handshake
     * 
     * @details The ASP5033 uses an unusual identity verification scheme.
     * Instead of a fixed WHO_AM_I register, it requires a specific sequence:
     * 
     * 1. Read part_id register (should be 0x00 or 0x66)
     * 2. Write 0x66 to part_id_set register
     * 3. Read part_id register again
     * 4. Verify it now reads 0x66
     * 
     * This handshake confirms the sensor responds correctly to register
     * writes and reads, preventing false positive detection of other I2C
     * devices at the same address.
     * 
     * @return true if sensor identity confirmed
     * @return false if sensor does not respond correctly to handshake
     * 
     * @note Called during init() to validate sensor before registration
     * @note This prevents configuration errors from incorrect sensor types
     * 
     * Source: libraries/AP_Airspeed/AP_Airspeed_ASP5033.cpp:77-92
     */
    bool confirm_sensor_id(void);

    /**
     * @brief Accumulator for temperature readings in degrees Celsius
     * 
     * @details Temperature samples from timer() callback are accumulated
     * in this variable. The sum is divided by temp_count to produce
     * the average temperature when get_temperature() is called.
     * 
     * Reset to zero after each get_temperature() call.
     */
    float temp_sum;

    /**
     * @brief Accumulator for pressure readings in Pascals
     * 
     * @details Pressure samples from timer() callback are accumulated
     * in this variable. The sum is divided by press_count to produce
     * the average pressure when get_differential_pressure() is called.
     * 
     * Reset to zero after each get_differential_pressure() call.
     */
    float press_sum;

    /**
     * @brief Last reported pressure value in Pascals
     * 
     * @details Stores the most recent pressure returned by
     * get_differential_pressure(). Used to return a valid value
     * when no new samples have accumulated since the last call.
     */
    float last_pressure;

    /**
     * @brief Last reported temperature value in degrees Celsius
     * 
     * @details Stores the most recent temperature returned by
     * get_temperature(). Used to return a valid value when no
     * new samples have accumulated since the last call.
     */
    float last_temperature;

    /**
     * @brief Count of accumulated pressure samples
     * 
     * @details Incremented by timer() for each pressure reading.
     * Used to calculate average pressure. Reset to zero after
     * get_differential_pressure() returns the average.
     */
    uint32_t press_count;

    /**
     * @brief Count of accumulated temperature samples
     * 
     * @details Incremented by timer() for each temperature reading.
     * Used to calculate average temperature. Reset to zero after
     * get_temperature() returns the average.
     */
    uint32_t temp_count;

    /**
     * @brief Timestamp of last successful sensor reading in milliseconds
     * 
     * @details Updated by timer() callback on each successful sensor read.
     * Used to detect stale data - if current time exceeds this timestamp
     * by more than 100ms, data is considered invalid and get functions
     * return false.
     * 
     * Units: milliseconds (from AP_HAL::millis())
     */
    uint32_t last_sample_ms;

    /**
     * @brief Pointer to HAL I2C device interface
     * 
     * @details Handle for I2C communication with the ASP5033 sensor.
     * Allocated during init() when sensor is successfully probed.
     * Freed in destructor.
     * 
     * Configured for:
     * - High-speed I2C (400kHz)
     * - 2 retries on communication failure
     * - 80Hz periodic callback
     * 
     * @note Must not be null when sensor is initialized
     */
    AP_HAL::I2CDevice *dev;
};

#endif  // AP_AIRSPEED_ASP5033_ENABLED
