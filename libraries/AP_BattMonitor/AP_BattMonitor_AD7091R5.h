/**
 * @file AP_BattMonitor_AD7091R5.h
 * @brief Battery monitor backend for AD7091R5 4-channel I2C ADC
 * 
 * This file implements battery monitoring using the AD7091R5 I2C-connected
 * 4-channel 12-bit ADC extender. The AD7091R5 allows reading multiple analog
 * voltage and current sensing channels through a single I2C interface, enabling
 * flexible battery monitoring configurations when direct analog inputs are limited.
 * 
 * @note The AD7091R5 supports simultaneous sampling of up to 4 analog channels
 * @note This implementation uses buffered reads to efficiently capture multi-channel data
 */

#pragma once

#include "AP_BattMonitor_Backend.h"

#ifndef AP_BATTERY_AD7091R5_ENABLED
#define AP_BATTERY_AD7091R5_ENABLED (HAL_PROGRAM_SIZE_LIMIT_KB > 1024)
#endif

#if AP_BATTERY_AD7091R5_ENABLED

#include <AP_HAL/I2CDevice.h>

// AD7091R5 Hardware Configuration
#define AD7091R5_NO_OF_CHANNELS  4    ///< Number of analog input channels available on AD7091R5 (4 channels: CH0-CH3)

// AD7091R5 Register Addresses and Command Codes
#define AD7091R5_CONF_CMD        0x04  ///< Configuration register command code for writing device settings
#define AD7091R5_CHAN_ALL        0x0F  ///< Channel mask to enable all 4 channels (bits 0-3 set)

// AD7091R5 Power-Down Mode Configuration Values
#define AD7091R5_CONF_PDOWN0     0x00  ///< Power-down mode 0: Normal operation (all channels active)
#define AD7091R5_CONF_PDOWN2     0x02  ///< Power-down mode 2: Power-saving mode with fast wake-up
#define AD7091R5_CONF_PDOWN3     0x03  ///< Power-down mode 3: Full power-down with slow wake-up
#define AD7091R5_CONF_PDOWN_MASK 0x03  ///< Bit mask for power-down mode bits in configuration register

/**
 * @class AP_BattMonitor_AD7091R5
 * @brief Battery monitor backend interfacing with AD7091R5 4-channel I2C ADC
 * 
 * @details This backend interfaces with the Analog Devices AD7091R5, a 12-bit 
 *          4-channel I2C ADC, to extend analog input capabilities for battery 
 *          voltage and current sensing. The AD7091R5 is particularly useful when:
 *          - Direct analog pins on the flight controller are limited or unavailable
 *          - Multiple battery parameters need monitoring (voltage, current, etc.)
 *          - I2C bus bandwidth is available and shared analog input is acceptable
 * 
 *          The driver configures the ADC for continuous sampling of all 4 channels
 *          and uses buffered reads to efficiently retrieve multi-channel data through
 *          the I2C interface. Voltage and current readings are converted using
 *          configurable multipliers and offsets to match specific sensor hardware.
 * 
 * @note Multiple instances of this backend can share the same ADC hardware
 * @note Actual sampling rate depends on I2C bus speed and scheduler timing
 * @note The AD7091R5 provides 12-bit resolution (0-4095 counts) across a 0-2.5V reference
 * 
 * @warning I2C communication failures will result in loss of battery monitoring data
 */
class AP_BattMonitor_AD7091R5 : public AP_BattMonitor_Backend
{
public:
    /**
     * @brief Construct a new AP_BattMonitor_AD7091R5 battery monitor backend
     * 
     * @param[in,out] mon          Reference to the main AP_BattMonitor instance managing all battery monitors
     * @param[in,out] mon_state    Reference to the state structure for this specific battery monitor instance
     * @param[in]     params       Reference to the parameters for this battery monitor (voltage/current pin assignments, scaling factors)
     */
    AP_BattMonitor_AD7091R5(AP_BattMonitor &mon,
                            AP_BattMonitor::BattMonitor_State &mon_state,
                            AP_BattMonitor_Params &params);

    /**
     * @brief Read battery voltage and current from the AD7091R5 ADC
     * 
     * @details This method performs a buffered multi-channel read from the AD7091R5
     *          to retrieve voltage and current sensor data. The ADC channels are read
     *          sequentially, with data conversion applied based on configured voltage
     *          multipliers and current sensing parameters. The results are stored in
     *          the battery monitor state structure for consumption by other systems.
     * 
     *          The method retrieves raw ADC counts, converts them to voltages using
     *          the ADC reference voltage (2.5V typical), then applies user-configured
     *          scaling factors to produce final voltage and current measurements.
     * 
     * @note This method should be called at 10Hz for proper battery state tracking
     * @note Multi-channel buffered reads improve I2C efficiency compared to individual channel reads
     * @note Thread-safe access to shared data is ensured through semaphore protection
     * 
     * @see _read_adc() for low-level ADC communication details
     */
    void read() override;
    
    /**
     * @brief Initialize the AD7091R5 I2C device and configure ADC settings
     * 
     * @details Performs I2C device initialization including:
     *          - I2C device handle acquisition from HAL
     *          - ADC configuration register setup (channel enables, power modes)
     *          - Initial health status check
     *          - Buffer pointer initialization for voltage and current channels
     * 
     * @note Called once during battery monitor initialization
     * @note Failure to initialize will prevent battery monitoring on this backend
     * 
     * @see _initialize() for detailed initialization sequence
     */
    void init(void) override;

    /**
     * @brief Check if this battery monitor provides consumed energy information
     * 
     * @details Consumed energy (mAh) can be calculated when current measurement
     *          is available by integrating current over time. This backend provides
     *          energy information whenever current sensing is configured.
     * 
     * @return true if consumed energy information is available, false otherwise
     * 
     * @see has_current() for current measurement capability
     */
    bool has_consumed_energy() const override
    {
        return has_current();
    }

    /**
     * @brief Check if this battery monitor provides current measurement
     * 
     * @details The AD7091R5 backend supports current sensing through one of its
     *          analog channels when configured with appropriate current sensor hardware
     *          (e.g., hall effect sensor, shunt resistor). This method always returns
     *          true as the hardware capability is present when this backend is active.
     * 
     * @return true - current measurement capability is always available with this backend
     * 
     * @note Actual current accuracy depends on sensor hardware and calibration parameters
     */
    bool has_current() const override
    {
        return true;
    }

    static const struct AP_Param::GroupInfo var_info[];

private:
    /**
     * @brief Perform low-level ADC read operation over I2C
     * 
     * @details Communicates with the AD7091R5 via I2C to retrieve raw ADC data
     *          from all configured channels. Reads are performed as multi-byte
     *          transfers with channel data extracted from the response packets.
     *          Updates the shared _analog_data array with latest ADC counts.
     * 
     * @note Uses semaphore protection for thread-safe access to shared data
     * @note Updates _health status based on I2C communication success
     */
    void _read_adc();
    
    /**
     * @brief Initialize and configure the AD7091R5 hardware
     * 
     * @details Performs hardware initialization sequence:
     *          - Acquires I2C device handle from HAL I2C manager
     *          - Configures AD7091R5 registers (channel enables, power mode)
     *          - Verifies device communication
     *          - Sets up initial state for shared data structures
     * 
     * @return true if initialization successful and device is responding, false otherwise
     * 
     * @note Called during init() to prepare the ADC for operation
     */
    bool _initialize();
    
    /**
     * @brief Convert raw ADC count to voltage
     * 
     * @details Converts a 12-bit ADC count (0-4095) to voltage using the AD7091R5
     *          reference voltage. Typical reference is 2.5V, giving resolution of
     *          approximately 0.61mV per count (2.5V / 4096 counts).
     * 
     * @param[in] data Raw ADC count value (0-4095 for 12-bit ADC)
     * 
     * @return Voltage in volts corresponding to the ADC count
     * 
     * @note This returns the voltage at the ADC input pin, before any external divider/multiplier scaling
     */
    float _data_to_volt(uint32_t data);

    /**
     * @brief Analog data structure for storing raw ADC counts
     * 
     * Holds the raw 12-bit ADC count for one channel. Multiple channels
     * are stored in the _analog_data array for buffered access.
     */
    static struct AnalogData {
        uint32_t data;  ///< Raw ADC count (0-4095 for 12-bit resolution)
    } _analog_data[AD7091R5_NO_OF_CHANNELS];  ///< Shared array holding latest ADC data for all 4 channels; accessed by all monitor instances using this ADC
    
    static bool _first;   ///< First instance flag - true for the first AP_BattMonitor_AD7091R5 instance to initialize; used to coordinate shared resource initialization
    static bool _health;  ///< Shared health status - true if I2C communication with AD7091R5 is functioning; false indicates communication failures

    HAL_Semaphore sem;  ///< Semaphore for thread-safe access to shared _analog_data array; prevents race conditions when multiple scheduler tasks read from shared ADC data
    
    AP_HAL::I2CDevice *_dev;  ///< Pointer to I2C device handle for communicating with the AD7091R5; acquired during initialization from HAL I2C manager
    
    uint8_t volt_buff_pt;  ///< ADC channel index (0-3) used for battery voltage sensing; maps to one of the 4 AD7091R5 analog input channels
    uint8_t curr_buff_pt;  ///< ADC channel index (0-3) used for battery current sensing; maps to one of the 4 AD7091R5 analog input channels

protected:

    // Configuration Parameters for Voltage and Current Sensing
    
    /**
     * @brief Voltage multiplier for battery voltage calculation
     * 
     * The ADC voltage reading is multiplied by this factor to calculate the actual
     * battery voltage. This accounts for voltage divider networks used to scale
     * battery voltage to the ADC input range (0-2.5V typical for AD7091R5).
     * 
     * Example: If a 10:1 voltage divider is used, set this to 10.0 to restore
     * the original battery voltage from the divided voltage at the ADC input.
     * 
     * Units: dimensionless (voltage ratio)
     * Typical range: 1.0 to 50.0
     */
    AP_Float _volt_multiplier;
    
    /**
     * @brief Current sensor scale factor
     * 
     * The ADC voltage reading from the current sensor is multiplied by this factor
     * to calculate battery current in amperes. This value depends on the current
     * sensor type (e.g., hall effect sensor output is typically 40mV/A, giving
     * a scale factor of 25.0 A/V; shunt resistor with amplifier may vary).
     * 
     * Units: amperes per volt (A/V)
     * Typical range: 0.1 to 100.0 A/V
     */
    AP_Float _curr_amp_per_volt;
    
    /**
     * @brief Current sensor voltage offset
     * 
     * This voltage offset is subtracted from the current sensor ADC reading before
     * scaling to current. Compensates for current sensor zero-current offset voltage
     * (e.g., hall effect sensors typically output 2.5V at zero current).
     * 
     * Units: volts (V)
     * Typical range: 0.0 to 5.0 V (often 2.5V for bipolar current sensors)
     */
    AP_Float _curr_amp_offset;
    
    /**
     * @brief Voltage measurement offset correction
     * 
     * This voltage offset is subtracted from the battery voltage ADC reading before
     * applying the voltage multiplier. Compensates for systematic measurement errors
     * or calibration offsets in the voltage sensing circuit.
     * 
     * Units: volts (V)
     * Typical range: -1.0 to 1.0 V (usually near zero)
     */
    AP_Float _volt_offset;
    
    /**
     * @brief AD7091R5 channel number for voltage sensing
     * 
     * Specifies which of the 4 AD7091R5 analog input channels (0-3) is connected
     * to the battery voltage sensing circuit. Must match hardware wiring.
     * 
     * Units: channel index (0-3)
     * Valid values: 0, 1, 2, 3
     */
    AP_Int8  _volt_pin;
    
    /**
     * @brief AD7091R5 channel number for current sensing
     * 
     * Specifies which of the 4 AD7091R5 analog input channels (0-3) is connected
     * to the battery current sensor output. Must match hardware wiring.
     * 
     * Units: channel index (0-3)
     * Valid values: 0, 1, 2, 3
     */
    AP_Int8  _curr_pin;
};

#endif // AP_BATTERY_AD7091R5_ENABLED
