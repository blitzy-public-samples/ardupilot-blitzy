/**
 * @file AP_BattMonitor_INA2xx.h
 * @brief Battery monitor backend for Texas Instruments INA2xx family I2C precision current/voltage/power sensors
 * 
 * @details This backend supports multiple Texas Instruments INA2xx precision power monitor devices:
 *          - INA226: 16-bit voltage/current/power monitor with alerts
 *          - INA228: 20-bit high-precision voltage/current/power/energy monitor
 *          - INA238: 16-bit voltage/current/power monitor with enhanced features
 *          - INA231: 16-bit voltage/current/power monitor (pin-compatible with INA226)
 * 
 *          These devices provide high-accuracy measurements over I2C bus with programmable shunt
 *          resistor configuration for current sensing. Device auto-detection is performed across
 *          multiple I2C addresses and device types during initialization.
 * 
 * Source: libraries/AP_BattMonitor/AP_BattMonitor_INA2xx.h
 */

#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/I2CDevice.h>
#include "AP_BattMonitor_Backend.h"
#include <AP_Param/AP_Param.h>
#include <utility>

#if AP_BATTERY_INA2XX_ENABLED

/**
 * @class AP_BattMonitor_INA2XX
 * @brief Battery monitor backend supporting Texas Instruments INA2xx I2C precision power monitors
 * 
 * @details This backend implements high-accuracy voltage, current, and power measurement using
 *          Texas Instruments INA2xx family devices. The implementation supports automatic device
 *          detection across multiple I2C addresses (0x40-0x4F) and device types (INA226/228/238/231).
 * 
 *          Key features:
 *          - High-precision 16-bit to 20-bit ADC measurements (device-dependent)
 *          - Programmable shunt resistor value for current sensing
 *          - Programmable maximum current range for optimal resolution
 *          - I2C bus and address configuration
 *          - Automatic device type detection
 *          - Temperature measurement support (INA228/238)
 *          - ~10Hz measurement sampling with accumulation
 * 
 *          Configuration parameters:
 *          - I2C bus selection
 *          - I2C address (or auto-detection across multiple addresses)
 *          - Maximum expected current (for optimal LSB calculation)
 *          - Shunt resistor value in ohms
 * 
 * @note Device auto-detection tries multiple I2C addresses sequentially if initial detection fails.
 *       This allows robust operation with devices at non-default addresses.
 * 
 * @warning Shunt resistor value must be accurately configured to match hardware. Incorrect value
 *          will result in inaccurate current measurements. I2C bus speed should be 400kHz for
 *          reliable operation with these devices.
 */
class AP_BattMonitor_INA2XX : public AP_BattMonitor_Backend
{
public:
    /**
     * @brief Construct INA2xx battery monitor backend
     * 
     * @param[in] mon Reference to parent AP_BattMonitor instance
     * @param[in] mon_state Reference to battery monitor state structure for this instance
     * @param[in] params Reference to battery monitor parameters for this instance
     */
    AP_BattMonitor_INA2XX(AP_BattMonitor &mon,
                          AP_BattMonitor::BattMonitor_State &mon_state,
                          AP_BattMonitor_Params &params);

    /**
     * @brief Check if individual cell voltages are available
     * @return false - INA2xx devices measure total battery voltage only, not individual cells
     */
    bool has_cell_voltages() const override { return false; }
    
    /**
     * @brief Check if temperature measurement is available
     * @return true if device supports temperature (INA228/238), false otherwise
     * @note INA226 and INA231 do not have temperature sensing capability
     */
    bool has_temperature() const override { return has_temp; }
    
    /**
     * @brief Check if current measurement is available
     * @return true - all INA2xx devices support current measurement via shunt resistor
     */
    bool has_current() const override { return true; }
    
    /**
     * @brief Get battery charge/discharge cycle count
     * @param[out] cycles Cycle count (not modified)
     * @return false - INA2xx devices do not track charge cycles
     */
    bool get_cycle_count(uint16_t &cycles) const override { return false; }
    
    /**
     * @brief Get current battery temperature measurement
     * @param[out] temperature Temperature in degrees Celsius (only updated if available)
     * @return true if temperature measurement available and valid, false otherwise
     * @note Only INA228 and INA238 variants support temperature measurement
     */
    bool get_temperature(float &temperature) const override;

    /**
     * @brief Initialize INA2xx device on I2C bus
     * 
     * @details Performs I2C device probe across configured addresses, detects device type,
     *          and configures device-specific settings including ADC configuration, averaging,
     *          conversion times, and shunt calibration. Registers periodic timer callback for
     *          continuous measurement accumulation.
     * 
     * @note If initial I2C address fails, automatically tries alternate addresses in probe list
     */
    void init(void) override;
    
    /**
     * @brief Read accumulated voltage and current measurements
     * 
     * @details Retrieves accumulated voltage and current samples from timer callback (~10Hz),
     *          calculates averages, updates battery monitor state with voltage, current, and
     *          power measurements. Also reads temperature if supported by device variant.
     * 
     * @note Called by main battery monitor update loop, typically at lower rate than timer callback
     */
    void read() override;

    static const struct AP_Param::GroupInfo var_info[];

private:
    AP_HAL::I2CDevice *dev; ///< HAL I2C device interface for communication with INA2xx sensor

    /**
     * @brief Supported INA2xx device type variants
     * 
     * @details Enumeration of detected device types with different capabilities:
     *          - UNKNOWN: Device not yet detected or detection failed
     *          - INA226: 16-bit, no temperature sensor
     *          - INA228: 20-bit high precision with temperature sensor
     *          - INA238: 16-bit with temperature sensor and enhanced features
     *          - INA231: 16-bit, no temperature sensor (INA226 compatible)
     */
    enum class DevType : uint8_t {
        UNKNOWN = 0,
        INA226,
        INA228,
        INA238,
        INA231,
    };

    static const uint8_t i2c_probe_addresses[]; ///< Array of I2C addresses to probe during device detection (0x40-0x4F range)
    uint8_t i2c_probe_next; ///< Index of next address to try in probe list if current address fails

    /**
     * @brief Configure INA2xx device with device-specific settings
     * 
     * @param[in] dtype Device type variant to configure (INA226/228/238/231)
     * @return true if configuration successful, false on I2C communication failure
     * 
     * @details Writes device-specific configuration registers including:
     *          - ADC conversion time and averaging settings
     *          - Operating mode (continuous voltage and current measurement)
     *          - Shunt calibration register based on rShunt and max_amps parameters
     *          - Alert/limit configurations if applicable
     */
    bool configure(DevType dtype);
    
    /**
     * @brief Read 16-bit word from device register
     * 
     * @param[in] reg Register address to read from
     * @param[out] data 16-bit signed value read from register
     * @return true if read successful, false on I2C communication failure
     */
    bool read_word16(const uint8_t reg, int16_t& data) const;
    
    /**
     * @brief Read 24-bit word from device register
     * 
     * @param[in] reg Register address to read from (20-bit or 24-bit register)
     * @param[out] data 32-bit signed value with 24-bit data (sign-extended)
     * @return true if read successful, false on I2C communication failure
     * 
     * @note Used for high-precision INA228 20-bit ADC readings
     */
    bool read_word24(const uint8_t reg, int32_t& data) const;
    
    /**
     * @brief Write 16-bit word to device register
     * 
     * @param[in] reg Register address to write to
     * @param[in] data 16-bit value to write
     * @return true if write successful, false on I2C communication failure
     */
    bool write_word(const uint8_t reg, const uint16_t data) const;
    
    /**
     * @brief Timer callback for periodic measurement accumulation
     * 
     * @details Called at ~100Hz by HAL scheduler to read voltage and current from device
     *          registers, accumulate samples for averaging, and update failure counter on
     *          I2C communication errors. Uses semaphore protection for thread-safe accumulation.
     * 
     * @note Runs in timer thread context, must be fast and non-blocking
     */
    void timer(void);
    
    /**
     * @brief Detect INA2xx device type by reading manufacturer/device ID registers
     * 
     * @return true if known device type detected and identified, false otherwise
     * 
     * @details Reads device identification registers and matches against known INA2xx
     *          manufacturer/device ID combinations to determine device variant. Sets
     *          dev_type member and has_temp flag based on detected device capabilities.
     */
    bool detect_device(void);

    DevType dev_type; ///< Detected device type variant (UNKNOWN until successful detection)
    uint32_t last_detect_ms; ///< Timestamp of last device detection attempt in milliseconds

    AP_Int8 i2c_bus; ///< I2C bus number parameter (0-based, platform-specific)
    AP_Int8 i2c_address; ///< I2C device address parameter (0x40-0x4F range, 0 for auto-detection)
    AP_Float max_amps; ///< Maximum expected current parameter in amperes (used to calculate optimal current_LSB for resolution)
    AP_Float rShunt; ///< Shunt resistor value parameter in ohms (typical values: 0.002 to 0.1 ohms)
    uint32_t failed_reads; ///< Counter of consecutive I2C read failures for health monitoring

    /**
     * @brief Accumulated voltage and current samples from timer callback
     * 
     * @details Thread-safe accumulation structure protected by semaphore:
     *          - count: Number of samples accumulated since last read()
     *          - volt_sum: Sum of voltage samples in volts
     *          - current_sum: Sum of current samples in amperes
     *          - sem: Semaphore protecting accumulation data from concurrent access
     */
    struct {
        uint16_t count;
        float volt_sum;
        float current_sum;
        HAL_Semaphore sem;
    } accumulate;
    
    float current_LSB; ///< Current measurement LSB (least significant bit) in amperes per bit, calculated from max_amps and ADC resolution
    float voltage_LSB; ///< Voltage measurement LSB in volts per bit, device-specific constant

    float temperature; ///< Last temperature reading in degrees Celsius (only valid if has_temp is true)

    bool has_temp; ///< Flag indicating if detected device variant supports temperature measurement (INA228/238 only)
};

#endif // AP_BATTERY_INA2XX_ENABLED
