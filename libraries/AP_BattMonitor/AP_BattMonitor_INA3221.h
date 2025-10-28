/**
 * @file AP_BattMonitor_INA3221.h
 * @brief Battery monitor backend for Texas Instruments INA3221 triple-channel I2C power monitor
 * 
 * @details This backend interfaces with the TI INA3221 3-channel current/voltage monitor,
 *          enabling a single INA3221 chip to monitor up to three battery instances simultaneously
 *          over I2C. The device performs sequential channel measurements in continuous conversion
 *          mode with programmable conversion times.
 * 
 *          The INA3221 takes two measurements for each channel: one for shunt voltage and one for
 *          bus voltage. Each measurement can be independently or sequentially measured, based on
 *          the mode setting (bits 2-0 in the Configuration register). When the INA3221 is in normal
 *          operating mode (that is, the MODE bits of the Configuration register are set to 111),
 *          the device continuously converts a shunt-voltage reading followed by a bus-voltage
 *          reading. This procedure converts one channel, and then continues to the shunt voltage
 *          reading of the next enabled channel, followed by the bus-voltage reading for that
 *          channel, and so on, until all enabled channels have been measured. The programmed
 *          Configuration register mode setting applies to all channels. Any channels that are not
 *          enabled are bypassed in the measurement sequence, regardless of mode setting.
 * 
 * @note Software Reset (Datasheet Section 8.3.3):
 *       The INA3221 features a software reset that reinitializes the device and register settings
 *       to default power-up values without having to cycle power to the device. Use bit 15 (RST)
 *       of the Configuration register to perform a software reset. Setting RST reinitializes all
 *       registers and settings to the default power state with the exception of the power-valid
 *       output state. If a software reset is issued, the INA3221 holds the output of the PV pin
 *       until the power-valid detection sequence completes. The Power-Valid UpperLimit and
 *       Power-Valid Lowerlimit registers return to the default state when the software reset has
 *       been issued. Therefore, any reprogrammed limit registers are reset, resulting in the
 *       original power-valid thresholds validating the power-valid conditions. This architecture
 *       prevents interruption to circuitry connected to the power-valid output during a software
 *       reset event.
 * 
 * @note Programmable Conversion Times:
 *       The INA3221 has programmable conversion times for both the shunt- and bus-voltage
 *       measurements. The selectable conversion times for these measurements range from 140μs
 *       to 8.244ms, allowing optimization of conversion speed versus measurement accuracy.
 * 
 * @see TI INA3221 Datasheet: https://www.ti.com/lit/ds/symlink/ina3221.pdf
 */

#pragma once

#include "AP_BattMonitor_config.h"

#if AP_BATTERY_INA3221_ENABLED

#include "AP_BattMonitor.h"
#include "AP_BattMonitor_Backend.h"

#ifndef HAL_BATTMON_INA3221_MAX_DEVICES
#define HAL_BATTMON_INA3221_MAX_DEVICES 1
#endif

/**
 * @class AP_BattMonitor_INA3221
 * @brief Battery monitor backend for Texas Instruments INA3221 3-channel current/voltage monitor
 * 
 * @details This backend interfaces with the TI INA3221 3-channel current and voltage monitor chip
 *          over I2C. The INA3221 can monitor up to three separate battery instances simultaneously,
 *          with each channel providing independent shunt voltage and bus voltage measurements.
 *          
 *          Multiple AP_BattMonitor_INA3221 instances can share a single physical INA3221 device by
 *          configuring different channel numbers (1-3) on the same I2C address. The AddressDriver
 *          structure manages shared access to the I2C device across multiple battery monitor instances,
 *          ensuring proper channel multiplexing and measurement sequencing.
 *          
 *          The device operates in continuous conversion mode, sequentially measuring enabled channels.
 *          Each channel measurement consists of a shunt voltage reading followed by a bus voltage
 *          reading, allowing calculation of current (from shunt voltage) and voltage (bus voltage).
 * 
 * @note One INA3221 chip can monitor up to 3 battery instances simultaneously with sequential
 *       channel measurement. Configure multiple battery monitors on the same I2C address with
 *       different channel numbers to utilize all three channels.
 * 
 * @note Programmable conversion times range from 140μs to 8.244ms per measurement, enabling
 *       trade-offs between update rate and measurement accuracy. In continuous conversion mode,
 *       the device automatically sequences through all enabled channels.
 * 
 * @warning Channel enable bits and measurement sequence coordination are critical for proper
 *          multi-channel operation. Ensure that only the channels actually configured in
 *          battery monitor instances are enabled in the device configuration register.
 */
class AP_BattMonitor_INA3221 : public AP_BattMonitor_Backend
{
public:
    /**
     * @brief Constructor for INA3221 battery monitor backend
     * 
     * @param[in,out] mon         Reference to main AP_BattMonitor instance
     * @param[in,out] mon_state   Reference to battery monitor state structure for this instance
     * @param[in]     params      Reference to battery monitor parameters for this instance
     */
    AP_BattMonitor_INA3221(AP_BattMonitor &mon,
                           AP_BattMonitor::BattMonitor_State &mon_state,
                           AP_BattMonitor_Params &params);

    /**
     * @brief Initialize I2C device and configure channel-specific settings
     * 
     * @details Sets up I2C communication with the INA3221 device and configures the specific
     *          channel (1-3) that this battery monitor instance will use. If multiple battery
     *          monitors share the same I2C address, they will share the same AddressDriver
     *          instance for coordinated device access.
     */
    void init() override;

    /**
     * @brief Read channel-specific battery voltage and current from INA3221
     * 
     * @details Reads the shunt voltage and bus voltage for this instance's configured channel.
     *          The device performs measurements sequentially across all enabled channels in
     *          continuous conversion mode. Current is calculated from shunt voltage using the
     *          configured shunt resistance.
     * 
     * @note This method should be called at approximately 10Hz for optimal battery monitoring
     *       performance and state estimation accuracy.
     */
    void read() override;

    /**
     * @brief Check if this battery monitor provides current measurement
     * 
     * @return true - INA3221 always provides current measurement capability via shunt voltage
     */
    bool has_current() const override {
        return true;
    }

    static const struct AP_Param::GroupInfo var_info[];

private:

    AP_Int8 i2c_bus;       ///< I2C bus number where INA3221 device is connected (0-based)
    AP_Int8 i2c_address;   ///< I2C device address (typically 0x40-0x43 depending on A0 pin configuration)
    AP_Int8 channel;       ///< Channel number on INA3221 chip (1-3, corresponding to hardware channels 1-3)

    /**
     * @struct AddressDriver
     * @brief Shared device driver for INA3221 to enable multi-channel access across battery instances
     * 
     * @details This structure manages shared access to a single INA3221 I2C device across multiple
     *          AP_BattMonitor_INA3221 instances. Since one INA3221 chip has three channels, up to
     *          three battery monitor instances can share the same physical device by using different
     *          channels. The AddressDriver coordinates I2C communication and maintains per-channel
     *          state information through a linked list of StateList entries.
     *          
     *          Each unique I2C bus/address combination gets its own AddressDriver instance, and all
     *          battery monitor instances on that bus/address share the driver for coordinated access.
     */
    static struct AddressDriver {
        /**
         * @brief Read a 16-bit register from the INA3221 device
         * 
         * @param[in]  addr  Register address to read (see INA3221 datasheet register map)
         * @param[out] ret   Reference to store the 16-bit register value read from device
         * 
         * @return true if register read successful, false on I2C communication error
         */
        bool read_register(uint8_t addr, uint16_t &ret);

        /**
         * @brief Write a 16-bit value to an INA3221 register
         * 
         * @param[in] addr  Register address to write (see INA3221 datasheet register map)
         * @param[in] val   16-bit value to write to the register
         * 
         * @return true if register write successful, false on I2C communication error
         */
        bool write_register(uint8_t addr, uint16_t val);

        /**
         * @brief Write configuration register to set up INA3221 operating mode and channel enables
         * 
         * @details Configures the INA3221 Configuration register (address 0x00) with appropriate
         *          settings for continuous conversion mode, enabled channels based on channel_mask,
         *          and conversion time settings. This ensures only the channels being used by
         *          configured battery monitors are enabled in the measurement sequence.
         * 
         * @return true if configuration write successful, false on I2C communication error
         */
        bool write_config(void);

        void timer(void);           ///< Periodic timer callback for reading measurements from device
        void register_timer();      ///< Register the timer callback with HAL scheduler

        AP_HAL::I2CDevice *dev;     ///< Pointer to I2C device interface for this INA3221 chip
        uint8_t bus;                ///< I2C bus number for this device
        uint8_t address;            ///< I2C device address for this device
        uint8_t channel_mask;       ///< Bitmask of channels in use by battery monitor instances (bits 0-2)
        uint8_t dev_channel_mask;   ///< Device channel enable mask written to configuration register

        /**
         * @struct StateList
         * @brief Per-channel state information for a single INA3221 channel
         * 
         * @details Each battery monitor instance using a channel on the shared INA3221 device
         *          has its own StateList entry. The AddressDriver maintains a linked list of
         *          StateList entries, one per configured channel, to store independent voltage,
         *          current, and health information for each battery being monitored.
         */
        struct StateList {
            struct StateList *next;       ///< Pointer to next StateList entry in linked list
            HAL_Semaphore sem;            ///< Semaphore for thread-safe access to state data
            uint8_t channel;              ///< Channel number (1-3) for this state entry

            bool healthy;                 ///< Health status of measurements on this channel
            float voltage;                ///< Most recent bus voltage measurement in volts
            float current_amps;           ///< Most recent current measurement in amperes
            float delta_mah;              ///< Accumulated milliamp-hours since last read
            float delta_wh;               ///< Accumulated watt-hours since last read
            uint32_t last_time_micros;    ///< Timestamp of last measurement in microseconds
        };
        StateList *statelist;   ///< Pointer to head of linked list of per-channel state entries

    } address_driver[HAL_BATTMON_INA3221_MAX_DEVICES];
    static uint8_t address_driver_count;

    AddressDriver::StateList *address_driver_state;  ///< Pointer to this instance's state in shared AddressDriver
};

#endif  // AP_BATTERY_INA3221_ENABLED
