/**
 * @file AnalogIn.h
 * @brief Simulated analog input (ADC) implementation for SITL (Software In The Loop)
 * 
 * @details This file implements the HAL AnalogIn interface for SITL simulation,
 *          providing simulated analog-to-digital converter (ADC) channels that
 *          read voltage values from the SITL_State simulation model.
 *          
 *          The simulated ADC provides voltage inputs for:
 *          - Battery voltage and current monitoring
 *          - Airspeed sensor differential pressure
 *          - RC receiver RSSI (signal strength)
 *          - Analog rangefinders
 *          - Other analog sensors requiring ADC input
 *          
 *          Each ADC channel maps to specific voltage fields in SITL_State,
 *          allowing the physics simulation to provide realistic sensor inputs
 *          to the flight control algorithms.
 * 
 * @note This is a simulation implementation - actual hardware uses platform-specific
 *       HAL implementations (AP_HAL_ChibiOS, AP_HAL_Linux, etc.)
 * 
 * @warning Simulated ADC has perfect accuracy with no noise, quantization error,
 *          or non-linearity unless explicitly added by the simulation model
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

#include <AP_HAL/AP_HAL.h>
#include "AP_HAL_SITL_Namespace.h"

/**
 * @brief ADC resolution in bits for simulated analog inputs
 * 
 * @details Defines the bit depth of the simulated ADC converter.
 *          16-bit resolution provides 65536 discrete levels, matching
 *          or exceeding most real hardware ADC implementations.
 *          
 *          This affects SITL_ADC_MAX_PIN_VALUE calculation but does not
 *          introduce actual quantization in simulation (values remain floating-point).
 */
#define SITL_ADC_RESOLUTION 16 // bits of resolution

/**
 * @brief Maximum digital value for simulated ADC pin reading
 * 
 * @details Calculated as (2^SITL_ADC_RESOLUTION - 1), representing the maximum
 *          digital count that corresponds to full-scale voltage input.
 *          For 16-bit: 65535 (0xFFFF)
 *          
 *          Used for scaling voltage to raw ADC counts in read_average/read_latest.
 */
#define SITL_ADC_MAX_PIN_VALUE ((1<<SITL_ADC_RESOLUTION)-1)

/**
 * @brief Full-scale voltage range for simulated ADC
 * 
 * @details Defines the maximum voltage input that produces SITL_ADC_MAX_PIN_VALUE.
 *          Set to 5.0V to match common 5V logic level systems.
 *          
 *          Voltage scaling: digital_value = (voltage / 5.0) * 65535
 *          
 * @note Units: volts (V)
 */
#define SITL_ADC_FULL_SCALE_VOLTAGE 5.0f

/**
 * @brief Maximum number of simulated ADC input channels
 * 
 * @details SITL supports up to 12 independent analog input channels,
 *          matching typical microcontroller ADC channel counts.
 *          
 *          Channel assignments (typical mapping):
 *          - Channel 0: Battery voltage monitor
 *          - Channel 1: Battery current monitor  
 *          - Channel 2: Airspeed sensor (differential pressure)
 *          - Channel 3-11: Additional analog sensors (rangefinders, RSSI, etc.)
 *          
 *          Actual channel usage depends on vehicle configuration and enabled features.
 */
#define SITL_INPUT_MAX_CHANNELS 12

/**
 * @class HALSITL::ADCSource
 * @brief Simulated analog input source for a single ADC channel
 * 
 * @details Implements the AP_HAL::AnalogSource interface for SITL, representing
 *          a single analog input channel that reads voltage values from the
 *          SITL_State simulation model.
 *          
 *          Each ADCSource instance is associated with a specific pin number
 *          that maps to voltage fields in SITL_State:
 *          - Pin 0: voltage (battery voltage in volts)
 *          - Pin 1: current (battery current in amps)
 *          - Pin 2: airspeed_voltage (differential pressure sensor voltage)
 *          - Pin 3+: Additional voltage sources as configured
 *          
 *          The simulation provides instantaneous voltage values without filtering,
 *          averaging, or noise (unless added by the physics model).
 * 
 * @note Thread-Safety: Voltage reads are atomic float operations, no locking required
 * @note Accuracy: Simulated ADC has perfect linearity and zero quantization error
 * 
 * @warning Values are simulation-provided - do not reflect real ADC characteristics
 *          like noise, drift, or temperature effects unless explicitly modeled
 */
class HALSITL::ADCSource : public AP_HAL::AnalogSource {
public:
    friend class HALSITL::AnalogIn;
    
    /**
     * @brief Construct simulated ADC source for specified pin
     * 
     * @param[in] sitlState  Pointer to SITL_State containing simulated voltage values
     * @param[in] pin        ADC input channel number (0 to SITL_INPUT_MAX_CHANNELS-1)
     * 
     * @note Pin number determines which SITL_State voltage field is read
     */
    ADCSource(SITL_State *sitlState, int16_t pin);

    /**
     * @brief Read averaged ADC value in digital counts
     * 
     * @details Returns the simulated ADC reading scaled to digital counts
     *          based on SITL_ADC_MAX_PIN_VALUE and SITL_ADC_FULL_SCALE_VOLTAGE.
     *          
     *          Calculation: (voltage / 5.0) * 65535
     *          
     *          In SITL, "average" and "latest" are identical since there's no
     *          hardware sampling or averaging - returns instantaneous simulation value.
     * 
     * @return Simulated ADC digital count (0 to SITL_ADC_MAX_PIN_VALUE)
     * 
     * @note Units: dimensionless digital counts
     * @note No actual averaging occurs in simulation
     */
    float read_average() override;
    
    /**
     * @brief Read latest ADC value in digital counts
     * 
     * @details Returns the simulated ADC reading scaled to digital counts.
     *          Identical to read_average() in SITL since there's no hardware
     *          sample buffering or filtering.
     * 
     * @return Simulated ADC digital count (0 to SITL_ADC_MAX_PIN_VALUE)
     * 
     * @note Units: dimensionless digital counts
     */
    float read_latest() override;
    
    /**
     * @brief Change the ADC pin/channel being read
     * 
     * @param[in] p  New pin number (0 to SITL_INPUT_MAX_CHANNELS-1)
     * 
     * @return true if pin changed successfully, false if invalid pin number
     * 
     * @note Allows runtime reconfiguration of which voltage source is monitored
     */
    bool set_pin(uint8_t p) override;
    
    /**
     * @brief Read averaged voltage in volts
     * 
     * @details Returns the simulated voltage value directly from SITL_State
     *          for the configured pin number. This is the primary interface
     *          for reading analog sensor values in SITL.
     *          
     *          Voltage mapping by pin:
     *          - Pin 0: Battery voltage (typically 10-16V for LiPo)
     *          - Pin 1: Current sensor voltage (scaled from amps)
     *          - Pin 2: Airspeed differential pressure voltage
     *          - Pin 3+: Additional analog sensors
     * 
     * @return Voltage in volts (V)
     * 
     * @note No actual averaging - returns instantaneous simulation value
     * @note Integration: Used by AP_BattMonitor, AP_Airspeed, and other analog sensors
     */
    float voltage_average() override;
    
    /**
     * @brief Read latest voltage in volts
     * 
     * @details Returns the simulated voltage value from SITL_State.
     *          Identical to voltage_average() in SITL simulation.
     * 
     * @return Voltage in volts (V)
     */
    float voltage_latest() override;
    
    /**
     * @brief Read ratiometric voltage (normalized to board voltage)
     * 
     * @details Returns voltage_average() directly since SITL board voltage
     *          is constant at 5.0V. Ratiometric measurement compensates for
     *          supply voltage variations, but SITL has perfect voltage regulation.
     * 
     * @return Voltage in volts (V), same as voltage_average()
     * 
     * @note Real hardware uses ratiometric measurement for resistive sensors
     */
    float voltage_average_ratiometric() override {
        return voltage_average();
    }

private:
    SITL_State *_sitlState;  ///< Pointer to simulation state containing voltage values
    int16_t _pin;            ///< ADC channel number (0 to SITL_INPUT_MAX_CHANNELS-1)
};

/**
 * @class HALSITL::AnalogIn
 * @brief Simulated analog input subsystem manager for SITL
 * 
 * @details Implements the AP_HAL::AnalogIn interface for SITL, managing all
 *          simulated ADC channels and providing access to analog input sources.
 *          
 *          Responsibilities:
 *          - Initialize simulated ADC channels on demand
 *          - Provide ADCSource instances for each channel number
 *          - Report simulated board voltage
 *          - Manage channel array and lifecycle
 *          
 *          Integration with SITL_State:
 *          The simulation model (SITL_State) updates voltage fields each
 *          simulation timestep based on physics calculations:
 *          - Battery voltage calculated from discharge model
 *          - Current calculated from motor power consumption  
 *          - Airspeed voltage from simulated pitot tube pressure
 *          
 *          These simulated voltages are then read by ArduPilot sensor drivers
 *          (AP_BattMonitor, AP_Airspeed) through the ADCSource interface,
 *          creating a closed-loop simulation environment.
 * 
 * @note Channel Management: ADCSource objects created on first access and cached
 * @note Thread-Safety: Channel creation not thread-safe, should occur during init
 * @note Perfect Accuracy: No ADC noise, offset, or gain errors in simulation
 * 
 * @warning Board voltage fixed at 5.0V - does not simulate voltage regulator droop
 * 
 * @see SITL_State for voltage field definitions and physics simulation
 * @see AP_BattMonitor for battery voltage/current monitoring usage
 * @see AP_Airspeed for airspeed sensor integration
 */
class HALSITL::AnalogIn : public AP_HAL::AnalogIn {
public:
    /**
     * @brief Construct analog input manager with simulation state reference
     * 
     * @param[in] sitlState  Pointer to SITL_State providing simulated voltages
     * 
     * @note Constructor is explicit to prevent implicit conversions
     */
    explicit AnalogIn(SITL_State *sitlState): _sitlState(sitlState) {}
    
    /**
     * @brief Initialize analog input subsystem
     * 
     * @details Performs any required initialization for the analog input system.
     *          In SITL, this is typically a no-op since channels are created
     *          on-demand when first accessed.
     * 
     * @note Called during HAL initialization before vehicle code starts
     */
    void init() override;
    
    /**
     * @brief Get analog source for specified channel number
     * 
     * @details Returns an ADCSource object for the requested channel, creating
     *          it on first access if needed. The channel object persists for
     *          the lifetime of the simulation.
     *          
     *          Channel allocation is lazy - channels are only created when
     *          requested by sensor drivers or other subsystems.
     * 
     * @param[in] n  Channel number (0 to SITL_INPUT_MAX_CHANNELS-1)
     * 
     * @return Pointer to ADCSource for this channel, or nullptr if invalid channel
     * 
     * @note Returned pointer valid for lifetime of AnalogIn object
     * @note Not thread-safe on first call - initialization should be single-threaded
     * 
     * @warning Do not delete returned pointer - managed by AnalogIn
     */
    AP_HAL::AnalogSource* channel(int16_t n) override;
    
    /**
     * @brief Get simulated board voltage
     * 
     * @details Returns the simulated board power supply voltage used as
     *          ADC reference and for ratiometric measurements.
     *          
     *          Fixed at 5.0V in SITL to represent a perfectly regulated
     *          5V power supply. Real hardware may have voltage variation
     *          from 4.5V to 5.5V depending on load and battery voltage.
     * 
     * @return Board voltage in volts (always 5.0V in SITL)
     * 
     * @note Real hardware: Used for ratiometric ADC measurement correction
     * @note SITL: Constant value, no supply voltage simulation
     */
    float board_voltage(void) override {
        return 5.0f;
    }
    
private:
    /**
     * @brief Array of ADC channel sources
     * 
     * @details Static array holding pointers to ADCSource objects for each channel.
     *          Initialized to nullptr, populated on-demand when channels are requested.
     *          
     * @note Static storage allows channels to persist across AnalogIn instances
     */
    static ADCSource* _channels[SITL_INPUT_MAX_CHANNELS];
    
    /**
     * @brief Pointer to simulation state containing voltage values
     * 
     * @details Reference to SITL_State object that provides simulated sensor
     *          voltages updated by the physics model each timestep.
     */
    SITL_State *_sitlState;
};
