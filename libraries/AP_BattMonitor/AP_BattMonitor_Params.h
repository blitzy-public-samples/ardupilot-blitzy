/**
 * @file AP_BattMonitor_Params.h
 * @brief Battery monitor parameter definitions and configuration structure
 * 
 * This file defines the AP_BattMonitor_Params class which encapsulates all configuration
 * parameters for battery monitoring backends. Parameters include battery capacity, voltage
 * thresholds, failsafe actions, and backend type selection. All parameters are stored via
 * AP_Param for runtime configurability and EEPROM persistence.
 */

#pragma once

#include <AP_Param/AP_Param.h>
#include "AP_BattMonitor_config.h"

/**
 * @class AP_BattMonitor_Params
 * @brief Battery monitor configuration parameters with AP_Param storage
 * 
 * This class encapsulates all battery monitor configuration parameters with AP_Param storage
 * for runtime persistence in EEPROM. Each battery monitor instance has its own parameter set
 * allowing independent configuration of multiple batteries. Parameters control backend type
 * selection, capacity and voltage thresholds for failsafes, arming checks, and optional
 * features via bitmask flags.
 * 
 * @note Parameters are stored in EEPROM via AP_Param for runtime configuration
 * @note Backend type changes require reboot to take effect
 * @warning Capacity values must account for reserve capacity that should not be used
 */
class AP_BattMonitor_Params {
public:
    static const struct AP_Param::GroupInfo var_info[];

    /**
     * @brief Constructor for battery monitor parameters
     */
    AP_BattMonitor_Params(void);

    /* Do not allow copies - parameter objects should not be copied due to AP_Param storage */
    CLASS_NO_COPY(AP_BattMonitor_Params);

    /**
     * @enum Type
     * @brief Battery monitor backend type selection
     * 
     * Defines available battery monitor backend implementations. Each type represents
     * a different hardware interface or measurement method for battery monitoring.
     * Selection determines how voltage, current, and capacity are measured and reported.
     */
    enum class Type {
        NONE                           = 0,   ///< Battery monitoring disabled
        ANALOG_VOLTAGE_ONLY            = 3,   ///< Analog voltage divider only (no current sensing)
        ANALOG_VOLTAGE_AND_CURRENT     = 4,   ///< Analog voltage divider with current sensor (voltage + current sensing)
        SOLO                           = 5,   ///< 3DR Solo smart battery with SMBus communication
        BEBOP                          = 6,   ///< Parrot Bebop/Disco smart battery
        SMBus_Generic                  = 7,   ///< Generic SMBus/I2C smart battery with auto-detection
        UAVCAN_BatteryInfo             = 8,   ///< DroneCAN (UAVCAN) battery using BatteryInfo message
        BLHeliESC                      = 9,   ///< ESC telemetry aggregation from BLHeli ESCs
        Sum                            = 10,  ///< Multi-battery aggregation (sum of multiple batteries)
        FuelFlow                       = 11,  ///< Fuel flow sensor for IC engines
        FuelLevel_PWM                  = 12,  ///< Fuel level sensor with PWM output
        SUI3                           = 13,  ///< SUI 3-cell smart battery
        SUI6                           = 14,  ///< SUI 6-cell smart battery
        NeoDesign                      = 15,  ///< NeoDesign battery management system
        MAXELL                         = 16,  ///< Maxell battery pack
        GENERATOR_ELEC                 = 17,  ///< Generator electrical output monitoring
        GENERATOR_FUEL                 = 18,  ///< Generator fuel consumption monitoring
        Rotoye                         = 19,  ///< Rotoye smart battery
        // 20 was MPPT_PacketDigital
        INA2XX                         = 21,  ///< Texas Instruments INA2XX series precision I2C power monitor
        LTC2946                        = 22,  ///< Linear Technology LTC2946 I2C power monitor
        Torqeedo                       = 23,  ///< Torqeedo electric motor battery
        FuelLevel_Analog               = 24,  ///< Fuel level sensor with analog voltage output
        Analog_Volt_Synthetic_Current  = 25,  ///< Analog voltage with synthetic current (estimated from throttle)
        INA239_SPI                     = 26,  ///< Texas Instruments INA239 SPI power monitor
        EFI                            = 27,  ///< Electronic Fuel Injection system monitoring
        AD7091R5                       = 28,  ///< Analog Devices AD7091R5 ADC for battery monitoring
        Scripting                      = 29,  ///< Lua scripting interface for custom battery backends
        INA3221                        = 30,  ///< Texas Instruments INA3221 triple-channel I2C power monitor
        ANALOG_CURRENT_ONLY            = 31,  ///< Analog current sensor only (no voltage measurement)
    };

    /**
     * @enum BattMonitor_LowVoltage_Source
     * @brief Voltage source selection for failsafe triggering
     * 
     * Determines which voltage measurement is used for low voltage failsafe detection.
     * Raw voltage drops under load due to battery internal resistance (voltage sag).
     * Sag-compensated voltage estimates the battery resting voltage by compensating
     * for the voltage drop under current load, providing more accurate state-of-charge.
     * Used for BATT_LOW_TYPE parameter.
     */
    enum BattMonitor_LowVoltage_Source {
        BattMonitor_LowVoltageSource_Raw            = 0,  ///< Use raw measured voltage (includes voltage sag under load)
        BattMonitor_LowVoltageSource_SagCompensated = 1   ///< Use sag-compensated resting voltage estimate (compensates for internal resistance)
    };
    /**
     * @enum Options
     * @brief Bitmask option flags for battery monitor behavior customization
     * 
     * These flags control optional features and behavior modifications for specific
     * battery monitor types. Multiple options can be combined using bitwise OR.
     */
    enum class Options : uint16_t {
        Ignore_UAVCAN_SoC                   = (1U<<0),  ///< Ignore DroneCAN/UAVCAN State-of-Charge (charge %) from device, use internal calculation instead. Use when device SoC is inaccurate.
        MPPT_Use_Input_Value                = (1U<<1),  ///< MPPT (solar) reports voltage/current from input (solar panel) instead of output. Use for monitoring solar panel performance.
        MPPT_Power_Off_At_Disarm            = (1U<<2),  ///< MPPT disabled when vehicle disarmed (if hardware supports). Use to conserve power when vehicle not in use.
        MPPT_Power_On_At_Arm                = (1U<<3),  ///< MPPT enabled when vehicle armed (if hardware supports). Use for power management during flight operations.
        MPPT_Power_Off_At_Boot              = (1U<<4),  ///< MPPT disabled at startup/boot (if hardware supports). Use for controlled power-up sequences.
        MPPT_Power_On_At_Boot               = (1U<<5),  ///< MPPT enabled at startup/boot (if hardware supports). If Power_Off_at_Boot also set, Power_Off takes precedence.
        GCS_Resting_Voltage                 = (1U<<6),  ///< Send resistance-compensated resting voltage to GCS via telemetry. Use for advanced battery state monitoring.
        AllowSplitAuxInfo                   = (1U<<7),  ///< Allow different DroneCAN node to provide auxiliary battery info (temperature, etc). Use with multi-node battery systems.
        InternalUseOnly                     = (1U<<8),  ///< Battery for internal ArduPilot use only, not sent via MAVLink BATTERY_STATUS. Use for hidden/debug battery monitors.
        Minimum_Voltage                     = (1U<<9),  ///< Sum monitor reports minimum voltage across batteries rather than average. Use to detect weakest cell in multi-battery setup.
        AllowDynamicNodeUpdate              = (1U<<10), ///< Allow dynamic DroneCAN node ID update during hot-swap when telemetry lost. Use for in-flight battery replacement systems.
    };

    /**
     * @brief Get configured voltage source for failsafe triggering
     * @return Voltage source type (raw or sag-compensated) used for low voltage failsafe detection
     */
    BattMonitor_LowVoltage_Source failsafe_voltage_source(void) const { return (enum BattMonitor_LowVoltage_Source)_failsafe_voltage_source.get(); }

    /**
     * @warning Capacity values (_pack_capacity, _low_capacity, _critical_capacity, _arming_minimum_capacity)
     *          must account for reserve capacity that should not be used. Never set pack capacity to the
     *          absolute maximum - always leave a safety margin to prevent over-discharge and battery damage.
     */

    AP_Int32 _pack_capacity;            ///< Battery pack capacity in milliamp-hours (mAh). Must account for reserve capacity that should not be used.
    AP_Int32 _serial_number;            ///< Battery serial number for unique identification. Automatically populated on SMBus smart batteries.
    AP_Float _low_voltage;              ///< Low battery voltage threshold in volts (V). Triggers low battery failsafe when voltage drops below this level.
    AP_Float _low_capacity;             ///< Low capacity threshold in milliamp-hours remaining (mAh). Triggers low capacity failsafe when remaining capacity falls below this value.
    AP_Float _critical_voltage;         ///< Critical battery voltage threshold in volts (V). Triggers critical failsafe (often initiates landing) when voltage drops below this level.
    AP_Float _critical_capacity;        ///< Critical capacity threshold in milliamp-hours remaining (mAh). Triggers critical capacity failsafe when remaining capacity falls below this value.
    AP_Int32 _arming_minimum_capacity;  ///< Minimum capacity required to arm in milliamp-hours (mAh). Vehicle will refuse to arm if remaining capacity is below this threshold.
    AP_Float _arming_minimum_voltage;   ///< Minimum voltage required to arm in volts (V). Vehicle will refuse to arm if battery voltage is below this threshold.
    AP_Int32 _options;                  ///< Bitmask of Options flags for behavior customization
#if AP_BATTERY_WATT_MAX_ENABLED
    AP_Int16 _watt_max;                 ///< Maximum battery power limit in watts (W). Reduces maximum throttle to limit current draw and satisfy this power constraint.
#endif
    AP_Enum<Type>  _type;                     ///< Battery monitor backend type selection. Determines hardware interface and measurement method.
    AP_Int8  _low_voltage_timeout;      ///< Timeout in seconds before low voltage failsafe triggers. Prevents false triggers from brief voltage sags.
    AP_Int8  _failsafe_voltage_source;  ///< Voltage measurement type for failsafe detection: raw measured voltage vs sag-compensated resting voltage estimate.
    AP_Int8  _failsafe_low_action;      ///< Failsafe action on low battery: none/warn/RTL/land. Defines vehicle behavior when low voltage/capacity threshold reached.
    AP_Int8  _failsafe_critical_action; ///< Failsafe action on critical battery: typically more aggressive than low action (e.g., immediate landing).
#if AP_BATTERY_ESC_TELEM_OUTBOUND_ENABLED
    AP_Int8  _esc_telem_outbound_index; ///< Bitmask of ESC indices for forwarding battery telemetry data (voltage, current, consumption, temperature) to ESCs.
#endif
};
