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
 * @file AP_Airspeed_AUAV.h
 * @brief AUAV I2C differential pressure sensor backend driver
 * 
 * @details This file implements support for AUAV differential pressure sensors
 *          connected via I2C interface. These sensors measure airspeed by detecting
 *          differential pressure between pitot and static ports.
 * 
 *          Supported sensor ranges:
 *          - 5 inch H2O (water column inches)
 *          - 10 inch H2O
 *          - 30 inch H2O
 * 
 *          Key features:
 *          - Temperature-compensated differential pressure measurement
 *          - Extended linearity compensation using factory calibration coefficients
 *          - Staged coefficient acquisition during initialization
 *          - 20ms measurement cycle with 200ms timeout
 *          - I2C communication at address 0x26
 * 
 *          The sensor requires a staged initialization sequence to acquire factory
 *          calibration coefficients (LIN_A, LIN_B, LIN_C, LIN_D, Es, TC50H, TC50L)
 *          which are used for linearity and temperature compensation.
 * 
 *          Output units:
 *          - Pressure: Pascals (Pa)
 *          - Temperature: Degrees Celsius (°C)
 * 
 * @note This driver supports both differential and absolute pressure variants,
 *       though ArduPilot primarily uses differential configuration for airspeed.
 * 
 * @see AP_Airspeed_Backend
 * @see libraries/AP_Airspeed/AP_Airspeed_AUAV.cpp
 * 
 * Source: libraries/AP_Airspeed/AP_Airspeed_AUAV.h
 *         libraries/AP_Airspeed/AP_Airspeed_AUAV.cpp
 */

#pragma once

#include "AP_Airspeed_config.h"

#if AP_AIRSPEED_AUAV_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <utility>

#include "AP_Airspeed_Backend.h"

/**
 * @class AUAV_Pressure_sensor
 * @brief Low-level driver for AUAV I2C pressure sensor hardware
 * 
 * @details This class handles direct communication with AUAV pressure sensors,
 *          managing measurement requests, data collection, and factory calibration
 *          coefficient acquisition. It supports both differential and absolute
 *          pressure sensor variants.
 * 
 *          Sensor Operation Sequence:
 *          1. Initialization: Read factory calibration coefficients in stages
 *          2. Runtime: Periodic measure() → delay → collect() cycle
 *          3. Compensation: Apply linearity and temperature corrections
 * 
 *          Calibration Coefficients (factory-programmed):
 *          - LIN_A, LIN_B, LIN_C, LIN_D: Linearity compensation (cubic polynomial)
 *          - TC50H, TC50L: Temperature compensation above/below 25°C
 *          - Es: Error sensitivity for temperature compensation
 * 
 *          The coefficient acquisition is staged across multiple I2C transactions
 *          because each coefficient requires a request command followed by a read
 *          operation. Coefficients are stored as 32-bit values read in high/low
 *          16-bit pairs.
 * 
 *          Communication Protocol:
 *          - I2C address: 0x26
 *          - Measurement command: 0xAE (start 8-sample average)
 *          - Measurement time: 16.2ms (differential), 37ms (absolute)
 *          - Data format: 7-byte response (status + 3-byte pressure + 3-byte temp)
 * 
 * @note This class is used internally by AP_Airspeed_AUAV and is not intended
 *       for direct instantiation by other code.
 * 
 * Source: libraries/AP_Airspeed/AP_Airspeed_AUAV.cpp:34-312
 */
class AUAV_Pressure_sensor
{
public:
    /**
     * @enum Type
     * @brief Pressure sensor measurement type
     */
    enum class Type {
        Differential,  ///< Measures differential pressure (airspeed applications)
        Absolute,      ///< Measures absolute pressure (altitude applications)
    };

    /**
     * @brief Construct AUAV pressure sensor instance
     * 
     * @param[in] _dev Reference to I2C device pointer (managed externally)
     * @param[in] _type Sensor type (Differential or Absolute)
     * 
     * @note The I2C device pointer is stored by reference to allow external
     *       initialization and lifecycle management by the parent class.
     * 
     * Source: libraries/AP_Airspeed/AP_Airspeed_AUAV.cpp:34-38
     */
    AUAV_Pressure_sensor(AP_HAL::I2CDevice *&_dev, Type _type);

    /**
     * @brief Initiate a pressure measurement cycle
     * 
     * @details Sends command 0xAE to start an 8-sample averaged measurement.
     *          Measurement takes approximately 16.2ms for differential sensors
     *          or 37ms for absolute sensors. Call collect() after appropriate
     *          delay to retrieve results.
     * 
     * @return true if I2C command transmitted successfully, false on I2C failure
     * 
     * @note Must wait appropriate time before calling collect() or sensor will
     *       return busy status.
     * 
     * Source: libraries/AP_Airspeed/AP_Airspeed_AUAV.cpp:88-97
     */
    bool measure() WARN_IF_UNUSED;

    /**
     * @enum Status
     * @brief Sensor reading status codes
     */
    enum class Status {
        Normal,  ///< Valid reading acquired successfully
        Busy,    ///< Measurement in progress, retry later
        Fault,   ///< Sensor fault detected or I2C communication failure
    };

    /**
     * @brief Read and process sensor measurement data
     * 
     * @details Retrieves 7-byte sensor response containing status byte, 24-bit
     *          pressure value, and 24-bit temperature value. Applies factory
     *          calibration coefficients for linearity compensation (cubic polynomial)
     *          and temperature compensation.
     * 
     *          Compensation Algorithm:
     *          1. Linearity: Pcorr = Pnorm + (A*Pnorm³ + B*Pnorm² + C*Pnorm + D)
     *          2. Temperature: Select TC50H or TC50L based on temp relative to 25°C
     *          3. Apply temperature correction with error sensitivity Es
     * 
     * @param[out] PComp Compensated pressure value (raw counts, not scaled to Pascals)
     * @param[out] temperature Temperature in degrees Celsius (°C)
     * 
     * @return Status code:
     *         - Normal: Valid data retrieved and stored in output parameters
     *         - Busy: Measurement not complete, call again later
     *         - Fault: Sensor error or unexpected status byte
     * 
     * @note PComp output is compensated but not scaled to final pressure units.
     *       Caller must apply range-specific scaling factor.
     * 
     * @warning Returned values are only valid when Status::Normal is returned.
     * 
     * Source: libraries/AP_Airspeed/AP_Airspeed_AUAV.cpp:103-162
     */
    Status collect(float &PComp, float &temperature) WARN_IF_UNUSED;

    /**
     * @enum CoefficientStage
     * @brief State machine stages for sequential factory coefficient acquisition
     * 
     * @details Factory calibration coefficients are stored in sensor EEPROM as
     *          32-bit values accessed via sequential 16-bit reads (high/low pairs).
     *          Each stage represents one 16-bit read operation. Coefficients must
     *          be read in order: A, B, C, D, then E (which contains TC50H, TC50L, Es).
     */
    enum class CoefficientStage {
        A_high,  ///< Read high 16 bits of LIN_A coefficient
        A_low,   ///< Read low 16 bits of LIN_A coefficient
        B_high,  ///< Read high 16 bits of LIN_B coefficient
        B_low,   ///< Read low 16 bits of LIN_B coefficient
        C_high,  ///< Read high 16 bits of LIN_C coefficient
        C_low,   ///< Read low 16 bits of LIN_C coefficient
        D_high,  ///< Read high 16 bits of LIN_D coefficient
        D_low,   ///< Read low 16 bits of LIN_D coefficient
        E_high,  ///< Read TC50H and TC50L (8 bits each in 16-bit word)
        E_low,   ///< Read Es (error sensitivity, 8 bits in low byte)
        Done,    ///< All coefficients acquired, sensor ready for measurements
    } stage;  ///< Current stage in coefficient acquisition sequence

    /**
     * @brief Execute one step in staged coefficient acquisition state machine
     * 
     * @details Reads factory calibration coefficients from sensor EEPROM in stages.
     *          Each coefficient requires a two-step I2C transaction: request command
     *          followed by read operation. Must be called repeatedly until stage
     *          reaches CoefficientStage::Done.
     * 
     *          Coefficient Base Commands:
     *          - Differential sensors: 0x2B + offset
     *          - Absolute sensors: 0x2F + offset
     * 
     *          Acquired coefficients (stored in private members):
     *          - LIN_A, LIN_B, LIN_C, LIN_D: Linearity correction (float, normalized)
     *          - TC50H, TC50L: Temperature compensation above/below 25°C (float)
     *          - Es: Error sensitivity for temp compensation (float)
     * 
     * @note This method advances the 'stage' member variable on successful reads.
     *       On I2C failure, stage is not advanced and caller should retry.
     * 
     * @note Once stage reaches Done, I2C retry count is reduced from 10 to 2
     *       for runtime efficiency.
     * 
     * Source: libraries/AP_Airspeed/AP_Airspeed_AUAV.cpp:195-312
     */
    void read_coefficients();

    /**
     * @brief Read single 16-bit coefficient register via staged I2C transaction
     * 
     * @details Implements two-step read protocol required by AUAV sensor:
     *          Step 1 (request): Send command byte to initiate coefficient read
     *          Step 2 (read): Retrieve 3-byte response (status + 16-bit data)
     * 
     * @param[in]  cmd Command byte (coefficient address)
     * @param[out] result 16-bit coefficient value (only valid when returning true)
     * 
     * @return true when data successfully read in step 2, false during step 1
     *         or on I2C/status errors
     * 
     * @note Uses internal coefficient_step state to track request/read phases.
     *       Automatically advances between request and read states.
     * 
     * Source: libraries/AP_Airspeed/AP_Airspeed_AUAV.cpp:164-193
     */
    bool read_register(uint8_t cmd, uint16_t &result);

private:
    /// @brief Reference to I2C device interface pointer (managed by parent)
    AP_HAL::I2CDevice *&dev;

    /// @brief Sensor type (Differential or Absolute)
    Type type;

    /**
     * @name Factory Calibration Coefficients
     * @brief Extended compensation coefficients read from sensor EEPROM
     * 
     * @details These coefficients are factory-programmed into each sensor and
     *          provide linearity correction and temperature compensation.
     *          Values are normalized floats acquired via read_coefficients().
     * 
     * @{
     */
    float LIN_A;   ///< Cubic term coefficient for linearity compensation
    float LIN_B;   ///< Quadratic term coefficient for linearity compensation
    float LIN_C;   ///< Linear term coefficient for linearity compensation
    float LIN_D;   ///< Constant term coefficient for linearity compensation
    float Es;      ///< Error sensitivity for temperature compensation
    float TC50H;   ///< Temperature compensation coefficient above 25°C
    float TC50L;   ///< Temperature compensation coefficient below 25°C
    /** @} */

    /**
     * @enum CoefficientStep
     * @brief Two-phase I2C transaction state for coefficient reads
     */
    enum class CoefficientStep {
        request,  ///< Send command byte to initiate read
        read,     ///< Retrieve data from sensor
    } coefficient_step;  ///< Current step in read_register() state machine

};

/**
 * @class AP_Airspeed_AUAV
 * @brief ArduPilot backend driver for AUAV I2C differential pressure airspeed sensors
 * 
 * @details This class integrates AUAV pressure sensors into the ArduPilot airspeed
 *          subsystem. It manages sensor initialization, periodic measurement updates,
 *          and provides calibrated differential pressure and temperature readings.
 * 
 *          Operational Sequence:
 *          1. Initialization (init):
 *             - Probe I2C bus for sensor at address 0x26
 *             - Register 20ms periodic timer callback
 *          2. Coefficient Acquisition (automatic in _timer):
 *             - Sequentially read factory calibration coefficients
 *             - Continues until all coefficients acquired (typically 200-300ms)
 *          3. Normal Operation (automatic in _timer):
 *             - Request measurement (20ms period)
 *             - Read and compensate pressure/temperature
 *             - Cache values for get_differential_pressure() and get_temperature()
 * 
 *          Supported Sensor Ranges:
 *          - 5 inch H2O (±12.4 mbar, ±0-40 m/s typical airspeed range)
 *          - 10 inch H2O (±24.9 mbar, ±0-56 m/s typical airspeed range)
 *          - 30 inch H2O (±74.7 mbar, ±0-98 m/s typical airspeed range)
 * 
 *          The range parameter (range_inH2O) is used to scale raw sensor output
 *          to Pascals. Formula: P_pascals = raw * 248.8 * 1.25 * 2 * range_inH2O
 * 
 * @note Measurements have 200ms timeout - readings older than this are
 *       considered stale and methods return false.
 * 
 * @warning This backend requires exclusive I2C bus access during coefficient
 *          acquisition phase. Bus contention may cause initialization failure.
 * 
 * @see AP_Airspeed_Backend
 * @see AUAV_Pressure_sensor
 * 
 * Source: libraries/AP_Airspeed/AP_Airspeed_AUAV.cpp:40-360
 */
class AP_Airspeed_AUAV : public AP_Airspeed_Backend
{
public:
    /**
     * @brief Construct AUAV airspeed backend instance
     * 
     * @param[in] frontend Reference to parent AP_Airspeed frontend
     * @param[in] _instance Sensor instance number (for multi-sensor configurations)
     * @param[in] _range_inH2O Sensor pressure range in inches of water column
     *                         (5, 10, or 30 inch H2O)
     * 
     * @note Constructor does not initialize I2C device. Call init() to complete
     *       initialization.
     * 
     * Source: libraries/AP_Airspeed/AP_Airspeed_AUAV.cpp:40-44
     */
    AP_Airspeed_AUAV(AP_Airspeed &frontend, uint8_t _instance, const float _range_inH2O);
    
    /**
     * @brief Destructor - cleanup I2C device resources
     * 
     * @details Deletes the dynamically allocated I2C device pointer to free
     *          HAL resources when backend is destroyed.
     */
    ~AP_Airspeed_AUAV(void) {
        delete _dev;
    }

    /**
     * @brief Probe I2C bus and initialize AUAV sensor
     * 
     * @details Performs the following initialization sequence:
     *          1. Get I2C device handle for configured bus at address 0x26
     *          2. Probe sensor by requesting measurement and verifying response
     *          3. Set device type identifier for logging
     *          4. Register 20ms periodic timer callback (_timer)
     * 
     *          Timer callback will automatically acquire factory calibration
     *          coefficients before beginning normal measurement operation.
     * 
     * @return true if sensor detected and initialization successful,
     *         false if sensor not found or probe failed
     * 
     * @note Sends GCS text message on success or failure for pilot notification.
     * 
     * @note I2C retry count is initially set to 10 for robust probing, then
     *       reduced to 2 after coefficient acquisition completes.
     * 
     * Source: libraries/AP_Airspeed/AP_Airspeed_AUAV.cpp:68-85
     */
    bool init() override;

    /**
     * @brief Retrieve current differential pressure reading
     * 
     * @details Returns the most recent pressure measurement cached by the
     *          periodic timer callback. Pressure is fully compensated (linearity
     *          and temperature) and scaled to Pascals based on sensor range.
     * 
     * @param[out] pressure Differential pressure in Pascals (Pa)
     * 
     * @return true if measurement is fresh (less than 200ms old),
     *         false if measurement is stale or sensor not initialized
     * 
     * @note This method is thread-safe, protected by semaphore.
     * 
     * @note Returns false during coefficient acquisition phase or if sensor
     *       has stopped responding.
     * 
     * Source: libraries/AP_Airspeed/AP_Airspeed_AUAV.cpp:347-352
     */
    bool get_differential_pressure(float &pressure) override;

    /**
     * @brief Retrieve current temperature reading
     * 
     * @details Returns the temperature measured by the sensor's internal
     *          thermistor, cached from the most recent measurement cycle.
     * 
     * @param[out] temperature Temperature in degrees Celsius (°C)
     * 
     * @return true if measurement is fresh (less than 200ms old),
     *         false if measurement is stale or sensor not initialized
     * 
     * @note This method is thread-safe, protected by semaphore.
     * 
     * @note Temperature is used internally for pressure compensation and
     *       also available for airspeed temperature correction.
     * 
     * Source: libraries/AP_Airspeed/AP_Airspeed_AUAV.cpp:355-360
     */
    bool get_temperature(float &temperature) override;

private:
    /**
     * @brief Probe for sensor presence on specified I2C bus
     * 
     * @details Attempts to communicate with sensor by:
     *          1. Acquiring I2C device handle
     *          2. Starting a measurement
     *          3. Waiting 20ms for measurement completion
     *          4. Reading result and verifying normal status
     * 
     * @param[in] bus I2C bus number to probe
     * @param[in] address I2C device address (typically 0x26)
     * 
     * @return true if sensor responds correctly, false otherwise
     * 
     * @note Uses 10 I2C retries during probing for robustness.
     * 
     * @note This method is called by init() and should not be called directly.
     * 
     * Source: libraries/AP_Airspeed/AP_Airspeed_AUAV.cpp:47-65
     */
    bool probe(uint8_t bus, uint8_t address);
    
    /**
     * @brief Periodic timer callback for sensor management and data acquisition
     * 
     * @details Called at 20ms intervals (50Hz) by HAL scheduler. Implements
     *          two-phase operation:
     * 
     *          Phase 1 - Coefficient Acquisition (startup):
     *          - Calls sensor.read_coefficients() until stage reaches Done
     *          - Typically takes 200-300ms to complete all stages
     * 
     *          Phase 2 - Normal Measurement Operation:
     *          - Request measurement from sensor
     *          - Collect results from previous measurement
     *          - Apply range scaling: P = 248.8 * 1.25 * (Pcomp-8388608)/16777216 * 2 * range_inH2O
     *          - Cache pressure (Pascals) and temperature (°C)
     *          - Update last_sample_time_ms for freshness checking
     * 
     * @note Handles busy status by skipping measurement request to avoid
     *       overrunning sensor timing.
     * 
     * @warning This method runs in timer interrupt context. Keep execution
     *          time minimal to avoid blocking other scheduler tasks.
     * 
     * Source: libraries/AP_Airspeed/AP_Airspeed_AUAV.cpp:314-344
     */
    void _timer();

    /// @brief Timestamp of last valid measurement in milliseconds (for 200ms timeout check)
    uint32_t last_sample_time_ms;
    
    /// @brief Flag indicating measurement request sent to sensor
    bool measurement_requested;
    
    /// @brief Pointer to HAL I2C device interface (dynamically allocated)
    AP_HAL::I2CDevice *_dev;

    /// @brief Low-level sensor driver instance (manages I2C protocol and compensation)
    AUAV_Pressure_sensor sensor { _dev, AUAV_Pressure_sensor::Type::Differential };

    /// @brief Cached differential pressure reading in Pascals (Pa)
    float pressure;
    
    /// @brief Cached temperature reading in degrees Celsius (°C)
    float temp_C;
    
    /// @brief Sensor pressure range in inches of water column (5, 10, or 30 inch H2O)
    const float range_inH2O;
};

#endif  // AP_Airspeed_AUAV_ENABLED
