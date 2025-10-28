/**
 * @file AP_BattMonitor_LTC2946.h
 * @brief Linear Technology LTC2946 I2C power monitor driver for ArduPilot battery monitoring
 * 
 * This driver interfaces with the LTC2946 I2C power monitor integrated circuit, which provides
 * wide-range voltage, current, and power measurement capabilities for battery monitoring applications.
 * The LTC2946 supports voltage measurements up to 80V and bidirectional current sensing with
 * programmable gain for high accuracy across different current ranges.
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/I2CDevice.h>
#include "AP_BattMonitor_Backend.h"
#include <utility>

#if AP_BATTERY_LTC2946_ENABLED

/**
 * @class AP_BattMonitor_LTC2946
 * @brief Battery monitor backend for Linear Technology LTC2946 I2C power monitor
 * 
 * @details This backend interfaces with the LTC2946 I2C power monitor chip for high-accuracy
 *          battery voltage, current, and power monitoring. The LTC2946 provides:
 *          - Wide input voltage range (0 to 80V)
 *          - Bidirectional current sensing
 *          - High-accuracy 12-bit ADC measurements
 *          - I2C interface for easy integration
 *          
 *          The driver uses timer-based sample accumulation to reduce measurement noise
 *          and provide stable readings. Measurements are accumulated at high frequency
 *          and averaged in the read() method called at ~10Hz.
 *          
 *          Configuration includes setting up current and voltage sense resistors,
 *          ADC resolution, and measurement LSB (Least Significant Bit) scaling factors
 *          to convert raw ADC values to physical units.
 * 
 * @note The LTC2946 wide input voltage range makes it suitable for high-voltage battery systems
 *       including 6S to 12S LiPo batteries and other power sources up to 80V. Integration with
 *       timer-based accumulation provides noise reduction for stable telemetry readings.
 * 
 * @warning I2C bus speed must be configured appropriately (typically 100kHz or 400kHz standard).
 *          Device I2C address configuration (via address pins on LTC2946) must match the address
 *          specified in AP_BattMonitor parameters. Incorrect address will result in init failure.
 *          Current sense resistor value must be correctly specified for accurate current measurements.
 */
class AP_BattMonitor_LTC2946 : public AP_BattMonitor_Backend
{
public:
    /**
     * @brief Inherit constructor from AP_BattMonitor_Backend
     * 
     * Uses the parent class constructor to initialize the backend with monitor instance,
     * state, and parameters from the battery monitor library.
     */
    using AP_BattMonitor_Backend::AP_BattMonitor_Backend;

    /**
     * @brief Check if individual cell voltages are available
     * 
     * @return false - LTC2946 measures total battery voltage, not individual cell voltages
     */
    bool has_cell_voltages() const override { return false; }
    
    /**
     * @brief Check if temperature measurement is available
     * 
     * @return false - LTC2946 does not provide temperature sensing
     */
    bool has_temperature() const override { return false; }
    
    /**
     * @brief Check if current measurement is available
     * 
     * @return true - LTC2946 provides bidirectional current sensing
     */
    bool has_current() const override { return true; }
    
    /**
     * @brief Check if battery cycle count is available
     * 
     * @param[out] cycles Battery cycle count (not used)
     * @return false - LTC2946 does not track battery charge/discharge cycles
     */
    bool get_cycle_count(uint16_t &cycles) const override { return false; }

    /**
     * @brief Initialize LTC2946 I2C power monitor
     * 
     * @details Performs the following initialization sequence:
     *          1. Configures I2C device interface using HAL device manager
     *          2. Reads and validates LTC2946 configuration registers
     *          3. Configures ADC measurement mode and resolution
     *          4. Calculates current_LSB and voltage_LSB scaling factors
     *          5. Registers timer callback for periodic sample accumulation
     *          
     *          If initialization fails (I2C communication error or invalid device response),
     *          the backend will remain disabled and read() will not be called.
     */
    virtual void init(void) override;
    
    /**
     * @brief Read accumulated voltage and current measurements
     * 
     * @details Retrieves accumulated samples from the timer callback and computes averaged
     *          battery voltage and current measurements. Called at approximately 10Hz by the
     *          battery monitor library. Uses semaphore protection to safely access accumulated
     *          data that is being updated by the timer interrupt context.
     *          
     *          Process:
     *          1. Acquires accumulate.sem semaphore for thread-safe access
     *          2. Retrieves accumulated voltage sum, current sum, and sample count
     *          3. Resets accumulators for next measurement period
     *          4. Releases semaphore
     *          5. Calculates average voltage and current from accumulated samples
     *          6. Updates battery state with new measurements
     *          
     *          If no samples have been accumulated (count == 0), the read is skipped to
     *          avoid division by zero and maintain last valid measurement.
     */
    virtual void read() override;
    
private:
    /**
     * HAL I2C device interface pointer for communicating with LTC2946 chip
     * Initialized in init() using AP_HAL::get_HAL().i2c_mgr->get_device()
     */
    AP_HAL::I2CDevice *dev;

    /**
     * @brief Read 16-bit word from LTC2946 register
     * 
     * @param[in]  reg  Register address to read from (LTC2946 register map)
     * @param[out] data 16-bit data read from register (MSB first)
     * 
     * @return true if I2C read successful, false on I2C communication error
     * 
     * @details Performs I2C read transaction to retrieve 16-bit register value from LTC2946.
     *          The LTC2946 uses big-endian byte order (MSB transmitted first). This method
     *          handles the byte order conversion and validates I2C communication success.
     */
    bool read_word(const uint8_t reg, uint16_t& data) const;
    
    /**
     * @brief Timer callback for periodic sample accumulation
     * 
     * @details Called periodically (typically at high frequency, e.g., 100Hz) from timer
     *          interrupt context to read and accumulate voltage and current samples from
     *          LTC2946. Accumulation provides noise reduction through averaging.
     *          
     *          Process:
     *          1. Reads voltage register from LTC2946 via I2C
     *          2. Reads current register from LTC2946 via I2C
     *          3. Converts raw ADC values to volts and amperes using LSB scaling
     *          4. Acquires semaphore and adds samples to accumulate.volt_sum and accumulate.current_sum
     *          5. Increments accumulate.count
     *          6. Releases semaphore
     *          
     *          Must be fast and non-blocking since executed in timer/interrupt context.
     *          I2C reads are handled by HAL with appropriate timeout protection.
     */
    void timer(void);

    /**
     * Accumulator structure for thread-safe sample collection
     * Used to accumulate multiple high-frequency samples for averaging in read() method
     */
    struct {
        uint16_t count;         ///< Number of samples accumulated since last read() call
        float volt_sum;         ///< Accumulated voltage sum in volts for averaging
        float current_sum;      ///< Accumulated current sum in amperes for averaging
        HAL_Semaphore sem;      ///< Semaphore for thread-safe access between timer() and read() contexts
    } accumulate;
    
    /**
     * Current measurement resolution in amperes per LSB
     * Calculated during init() based on sense resistor value and LTC2946 ADC configuration
     * Used to convert raw current ADC values to amperes
     */
    float current_LSB;
    
    /**
     * Voltage measurement resolution in volts per LSB
     * Calculated during init() based on LTC2946 voltage divider configuration
     * Used to convert raw voltage ADC values to volts
     */
    float voltage_LSB;
};

#endif // AP_BATTERY_LTC2946_ENABLED
