/**
 * @file AP_BattMonitor_INA239.h
 * @brief Texas Instruments INA239 SPI-based precision current/voltage sensor integration
 * 
 * @details This file implements the battery monitor backend for the Texas Instruments
 *          INA239 high-precision power monitor IC. The INA239 provides accurate voltage
 *          and current measurement via SPI bus interface, supporting bidirectional current
 *          measurement with 16-bit ADC resolution. This implementation accumulates samples
 *          at high frequency via timer callback and provides averaged readings to the
 *          battery monitoring system.
 * 
 * @note The INA239 requires external shunt resistor configuration for current measurement
 * @see AP_BattMonitor_Backend for base battery monitor interface
 */

#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/SPIDevice.h>
#include "AP_BattMonitor_Backend.h"
#include <utility>

#if AP_BATTERY_INA239_ENABLED

/**
 * @class AP_BattMonitor_INA239
 * @brief Battery monitor backend for Texas Instruments INA239 I2C/SPI power monitor chip
 * 
 * @details This backend interfaces with the Texas Instruments INA239 precision power monitor
 *          for high-accuracy battery voltage and current measurement via SPI bus. The INA239
 *          provides 16-bit ADC resolution for both voltage and current sensing with programmable
 *          conversion times and averaging. This implementation uses timer-based accumulation
 *          to collect samples at high frequency (~100Hz) and provides averaged readings at
 *          the standard battery monitor update rate (~10Hz).
 * 
 *          Key Features:
 *          - Bidirectional current measurement (charge and discharge)
 *          - High-precision voltage measurement (up to 85V bus voltage)
 *          - Programmable shunt resistor calibration
 *          - SPI interface for low-latency communication
 *          - Hardware accumulation with semaphore protection for thread safety
 * 
 * @note Requires calibration using shunt resistor value (rShunt) and maximum expected
 *       current (max_amps) parameters for accurate current measurement
 * @warning Ensure shunt resistor power rating exceeds maximum expected power dissipation
 *          (I²R) to prevent overheating and measurement drift
 * 
 * @see AP_BattMonitor_Backend
 * @see Texas Instruments INA239 datasheet for register definitions and electrical specifications
 */
class AP_BattMonitor_INA239 : public AP_BattMonitor_Backend
{
public:
    /**
     * @brief Construct INA239 battery monitor backend
     * 
     * @param[in] mon Reference to parent AP_BattMonitor instance for parameter access
     * @param[in] mon_state Reference to battery monitor state structure for storing measurements
     * @param[in] params Reference to battery monitor parameters (voltage/current scaling, offsets)
     */
    AP_BattMonitor_INA239(AP_BattMonitor &mon,
                          AP_BattMonitor::BattMonitor_State &mon_state,
                          AP_BattMonitor_Params &params);

    /**
     * @brief Query if individual cell voltages are available
     * @return false - INA239 measures total pack voltage only, not individual cell voltages
     */
    bool has_cell_voltages() const override { return false; }
    
    /**
     * @brief Query if battery temperature measurement is available
     * @return false - INA239 does not provide temperature sensing capability
     */
    bool has_temperature() const override { return false; }
    
    /**
     * @brief Query if current measurement is available
     * @return true - INA239 provides bidirectional current measurement via shunt resistor
     */
    bool has_current() const override { return true; }
    
    /**
     * @brief Query battery charge/discharge cycle count
     * @param[out] cycles Reference to store cycle count (not modified)
     * @return false - INA239 does not track charge cycles (requires smart battery protocol)
     */
    bool get_cycle_count(uint16_t &cycles) const override { return false; }

    /**
     * @brief Initialize INA239 SPI device and configure measurement parameters
     * 
     * @details Performs the following initialization sequence:
     *          1. Obtains SPI device handle from HAL for INA239
     *          2. Calls configure() to set up INA239 registers
     *          3. Registers timer() callback for periodic sample accumulation
     *          
     *          Called once during battery monitor startup. If initialization fails,
     *          backend will be marked unhealthy and read() will not be called.
     * 
     * @note Requires valid max_amps and rShunt parameters for calibration
     * @see configure() for INA239 register configuration details
     */
    void init(void) override;
    
    /**
     * @brief Read accumulated voltage and current samples and update battery state
     * 
     * @details Retrieves samples accumulated by timer() callback at ~100Hz and computes
     *          averaged voltage and current values. Updates are performed at battery monitor
     *          rate (~10Hz). Uses semaphore protection to safely access accumulation buffer
     *          shared with timer interrupt context.
     *          
     *          Processing steps:
     *          1. Acquire accumulate.sem semaphore
     *          2. Copy accumulated voltage_sum, current_sum, and sample count
     *          3. Reset accumulators to zero
     *          4. Release semaphore
     *          5. Calculate averaged voltage and current
     *          6. Update mon_state with new measurements
     *          
     *          If no samples accumulated since last read, retains previous values.
     * 
     * @note Called at ~10Hz by battery monitor scheduler task
     * @warning Requires timer() callback to be registered during init() for sample accumulation
     * @see timer() for sample accumulation implementation
     */
    void read() override;

    static const struct AP_Param::GroupInfo var_info[];

protected:
    AP_HAL::Device *dev; ///< HAL SPI device interface for INA239 communication

    /**
     * @brief Configure INA239 registers for voltage and current measurement
     * 
     * @details Writes configuration to INA239 registers:
     *          - Sets ADC conversion times and averaging
     *          - Configures shunt and bus voltage measurement modes
     *          - Calculates and writes calibration register based on shunt resistor value
     *          - Computes current_LSB and voltage_LSB for raw ADC to engineering unit conversion
     *          
     *          Calibration formula: CAL = 0.00512 / (current_LSB * rShunt)
     *          where current_LSB = max_amps / 32768 (for 16-bit signed range)
     * 
     * @note Called by init() and retried on read failures via last_configure_ms timeout
     * @see INA239 datasheet section on calibration for register calculation details
     */
    void configure(void);
    
    /**
     * @brief Read 16-bit word from INA239 register via SPI
     * 
     * @param[in] reg Register address to read (0x00-0x0F per INA239 register map)
     * @param[out] data Reference to store 16-bit signed register value
     * @return true if SPI transaction successful, false on communication error
     * 
     * @note Increments failed_reads counter on error for health monitoring
     */
    bool read_word(const uint8_t reg, int16_t& data) const;
    
    /**
     * @brief Write 16-bit word to INA239 register via SPI
     * 
     * @param[in] reg Register address to write (0x00-0x0F per INA239 register map)
     * @param[in] data 16-bit unsigned value to write to register
     * @return true if SPI transaction successful, false on communication error
     */
    bool write_word(const uint8_t reg, const uint16_t data) const;
    
    /**
     * @brief Periodic timer callback for high-frequency sample accumulation
     * 
     * @details Called at ~100Hz by HAL scheduler to read INA239 voltage and current registers
     *          and accumulate samples for averaging. Uses semaphore-protected accumulate
     *          structure to safely share data with read() method in main thread context.
     *          
     *          Accumulation process:
     *          1. Read voltage and current registers from INA239
     *          2. Convert raw ADC values to engineering units using LSB scaling
     *          3. Acquire accumulate.sem semaphore
     *          4. Add voltage and current to running sums
     *          5. Increment sample count
     *          6. Release semaphore
     *          
     *          If read failures exceed threshold, triggers reconfiguration.
     * 
     * @note Runs in timer interrupt context - keep execution time minimal
     * @warning Must complete within timer period (~10ms) to avoid scheduler overruns
     */
    void timer(void);

    bool configured; ///< true after successful INA239 register configuration during init()
    bool callback_registered; ///< true after timer() callback successfully registered with HAL scheduler
    uint32_t failed_reads; ///< Cumulative count of failed SPI read transactions for health monitoring
    uint32_t last_configure_ms; ///< Timestamp in milliseconds of last configure() attempt for retry logic

    /**
     * @brief Accumulation buffer for high-frequency samples from timer() callback
     * 
     * @details Thread-safe accumulator shared between timer() interrupt context and
     *          read() main thread context. Semaphore protection ensures atomic access
     *          to prevent race conditions during accumulation and retrieval.
     */
    struct {
        uint16_t count; ///< Number of samples accumulated since last read() call
        float volt_sum; ///< Accumulated voltage sum in volts for averaging
        float current_sum; ///< Accumulated current sum in amperes (positive = discharge, negative = charge)
        HAL_Semaphore sem; ///< Semaphore for thread-safe access to accumulation buffer
    } accumulate;
    
    float current_LSB; ///< Current measurement resolution in amperes per LSB (calculated from max_amps)
    float voltage_LSB; ///< Voltage measurement resolution in volts per LSB (INA239 fixed at 3.125mV/LSB)

    AP_Float max_amps; ///< Maximum expected current in amperes for full-scale range configuration (parameter)
    AP_Float rShunt; ///< Shunt resistor value in ohms for current sensing (parameter, typically 0.001-0.01Ω)
};

#endif // AP_BATTERY_INA239_ENABLED
