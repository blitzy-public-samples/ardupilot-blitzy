/**
 * @file AnalogIn.h
 * @brief QURT platform analog input implementation using ESC telemetry
 * 
 * @details This file implements analog input for the Qualcomm Hexagon DSP platform (QURT HAL).
 *          Unlike typical HAL implementations that directly read ADC hardware, the QURT analog
 *          input implementation delegates voltage and current readings to the RCOutput driver,
 *          which receives telemetry data from the ESC (Electronic Speed Controller) board.
 *          
 *          Architecture: The Hexagon DSP does not have direct ADC hardware access. Instead,
 *          analog measurements (voltage, current) are performed by the ESC controller board's
 *          ADC hardware and reported back via UART telemetry. This implementation reads those
 *          telemetry values through the RCOutput driver's esc_power_status structure.
 *          
 *          This design is unusual compared to other ArduPilot HAL platforms (ChibiOS, Linux, etc.)
 *          which typically provide direct ADC register access for analog inputs.
 * 
 * @note Platform-specific: QURT/Hexagon DSP architecture requires this delegation model
 * @note Telemetry dependency: Analog readings only available when ESC telemetry is active
 * 
 * @warning Limited channels: Only supports analog channels provided by ESC telemetry
 *          (typically 2-4 channels: voltage, current, possibly power and temperature)
 * @warning Update rate: Analog readings limited by ESC telemetry rate (typically 10-100Hz),
 *          much slower than direct ADC access which could run at kHz rates
 * @warning Accuracy: Analog reading accuracy depends on ESC controller ADC quality and calibration
 * 
 * @see RCOutput.h for ESC telemetry structure definitions (esc_power_status)
 * @see AP_BattMonitor library for typical analog input usage in battery monitoring
 * 
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

#include "AP_HAL_QURT.h"

/**
 * @class QURT::AnalogSource
 * @brief Analog input channel for voltage, current, and RSSI readings via ESC telemetry
 * 
 * @details This class implements the AP_HAL::AnalogSource interface for the QURT platform
 *          by reading analog measurements from ESC telemetry data provided by the RCOutput
 *          driver. It does not directly access ADC hardware.
 *          
 *          Delegation Model: Instead of reading ADC registers directly (as on ChibiOS, Linux),
 *          this implementation retrieves voltage and current measurements from the ESC
 *          controller board, which has its own ADC hardware and reports measurements via
 *          UART telemetry.
 *          
 *          Typical Usage:
 *          - Battery voltage monitoring: Reads board voltage from ESC power telemetry for
 *            battery failsafe triggers and capacity estimation
 *          - Current sensing: Reads current draw for power monitoring and mAh consumed
 *            calculation used by battery failsafe and remaining capacity estimation
 *          - RSSI monitoring: May read signal strength if provided by ESC telemetry
 *          
 *          Data Source: All readings come from RCOutput::esc_power_status telemetry structure,
 *          which the RCOutput driver populates from ESC UART telemetry packets. The ESC
 *          controller board performs ADC conversions at its own rate and transmits results.
 * 
 * @note Unusual design: Most HAL platforms read ADC directly; QURT uses ESC telemetry as ADC source
 * @note Averaging: read_average() typically averages 100-1000 samples to reduce noise for
 *       stable battery voltage readings critical for failsafe decisions
 * @note DSP platform: Hexagon DSP has no ADC hardware; all analog measurements done on
 *       apps processor or ESC controller
 * 
 * @warning Limited channels: Only supports voltage/current readings provided by ESC telemetry
 *          (typically 2-4 channels). Cannot read arbitrary analog pins.
 * @warning Telemetry loss: If ESC telemetry communication fails, analog readings become stale.
 *          Vehicle should detect this condition via telemetry timeout monitoring.
 * @warning No direct ADC: Cannot add new analog channels without ESC controller firmware modifications
 * @warning Safety-critical: Voltage/current readings used for battery failsafe triggers
 * 
 * @see AP_HAL::AnalogSource for the abstract interface definition
 * @see RCOutput.h for esc_power_status structure containing telemetry data
 * @see AP_BattMonitor for usage of analog voltage/current in battery monitoring
 */
class QURT::AnalogSource : public AP_HAL::AnalogSource
{
public:
    /**
     * @brief Read analog value averaged over time
     * 
     * @details Returns the analog channel value averaged over multiple samples to reduce
     *          noise. For voltage channels, returns board voltage in volts. For current
     *          channels, returns current draw in amperes. Averaging window typically
     *          includes 100-1000 samples for stable battery monitoring.
     *          
     *          Implementation: Reads from ESC telemetry data structure (esc_power_status)
     *          which is updated by RCOutput driver when telemetry packets arrive from ESC.
     * 
     * @return Averaged analog value: voltage in volts or current in amps depending on channel
     * 
     * @note Voltage readings typically 0-30V range for battery monitoring
     * @note Current readings typically 0-200A range depending on ESC rating
     * @note Returns last known value if ESC telemetry is stale
     * 
     * @warning Value may be stale if ESC telemetry communication has failed
     */
    float read_average() override;
    
    /**
     * @brief Read most recent analog value without averaging
     * 
     * @details Returns the latest analog channel value from the most recent ESC telemetry
     *          packet without time-averaging. Useful for detecting rapid changes in voltage
     *          or current, though noisier than read_average().
     *          
     *          Update Rate: Limited by ESC telemetry rate (typically 10-100Hz), much slower
     *          than direct ADC sampling which could be kHz. Use read_average() for stable
     *          readings; use this method for detecting transients.
     * 
     * @return Latest analog value: voltage in volts or current in amps depending on channel
     * 
     * @note May show more noise than read_average() due to lack of filtering
     * @note Update rate depends on ESC telemetry transmission frequency
     * 
     * @warning Single-sample reading susceptible to telemetry packet noise
     */
    float read_latest() override;
    
    /**
     * @brief Configure which analog channel to read
     * 
     * @details Sets the pin/channel number for this analog source. On QURT, pin numbers
     *          map to specific telemetry values rather than physical ADC pins:
     *          - Channel 0: Typically board voltage (V)
     *          - Channel 1: Typically current draw (A)
     *          - Channel 2-3: May be power (W), temperature (C), or RSSI depending on ESC
     *          
     *          Available channels are limited by what the ESC telemetry provides.
     * 
     * @param[in] p Channel number (maps to telemetry source, not physical pin)
     * 
     * @return true if channel is valid and supported by ESC telemetry, false otherwise
     * 
     * @note Limited pin options: Only channels 0-3 typically available (voltage, current,
     *       possibly power and temperature)
     * @note Not a physical pin: Pin number maps to telemetry data field, not GPIO/ADC pin
     * 
     * @warning Invalid channel numbers will cause reading methods to return stale or zero values
     */
    bool set_pin(uint8_t p) override;
    
    /**
     * @brief Read averaged voltage measurement
     * 
     * @details Convenience method specifically for voltage readings. Returns board voltage
     *          averaged over time for stable battery monitoring. Equivalent to read_average()
     *          when channel is configured for voltage (typically channel 0).
     *          
     *          Primary Use: Battery voltage monitoring for failsafe triggers and capacity
     *          estimation. Averaging reduces noise for reliable low-voltage detection.
     * 
     * @return Board voltage in volts (V), averaged over sampling window
     * 
     * @note Typical range: 0-30V for common LiPo battery configurations
     * @note Averaging window: 100-1000 samples for noise reduction
     * @note Safety-critical: Used for battery failsafe voltage threshold checks
     * 
     * @warning Stale value if ESC telemetry communication has failed
     */
    float voltage_average() override;
    
    /**
     * @brief Read latest voltage measurement without averaging
     * 
     * @details Returns most recent board voltage reading from latest ESC telemetry packet.
     *          Useful for detecting rapid voltage sags under load, though noisier than
     *          voltage_average(). Equivalent to read_latest() for voltage channel.
     * 
     * @return Most recent board voltage in volts (V)
     * 
     * @note More responsive to transients but noisier than voltage_average()
     * @note Update rate: 10-100Hz depending on ESC telemetry rate
     * 
     * @warning Single-sample reading may show voltage spikes/dips from measurement noise
     */
    float voltage_latest() override;
    
    /**
     * @brief Read ratiometric voltage (same as absolute voltage on QURT)
     * 
     * @details On platforms with ratiometric ADC references, this method returns voltage
     *          relative to the ADC reference voltage. On QURT, ESC telemetry provides
     *          absolute voltage measurements, so this simply returns voltage_average().
     *          
     *          Implementation: Inline method delegates to voltage_average() since ESC
     *          telemetry reports absolute voltage rather than ADC counts.
     * 
     * @return Board voltage in volts (V), same as voltage_average()
     * 
     * @note QURT-specific: No ratiometric difference since readings come from ESC telemetry
     */
    float voltage_average_ratiometric() override
    {
        return voltage_average();
    }
    
    /**
     * @brief Channel/pin number for this analog source
     * 
     * @details Stores the channel number set by set_pin(). Maps to telemetry data field:
     *          0=voltage, 1=current, 2-3=power/temp/RSSI depending on ESC capabilities.
     *          Not a physical pin number since QURT has no direct ADC access.
     */
    uint8_t pin;
};

/**
 * @class QURT::AnalogIn
 * @brief Factory class for creating analog input source channels
 * 
 * @details This class manages analog input channels on the QURT platform. It provides the
 *          channel() method to create AnalogSource instances for specific measurements
 *          (voltage, current, etc.) from ESC telemetry data.
 *          
 *          Channel Mapping: Channel numbers map to specific telemetry values:
 *          - Channel 0: Board voltage (V) - primary battery voltage
 *          - Channel 1: Current draw (A) - total current consumption
 *          - Channel 2: Power (W) or temperature (C) if available
 *          - Channel 3: RSSI or other auxiliary measurement if available
 *          
 *          Number of Available Channels: Limited by ESC telemetry capabilities, typically
 *          2-4 channels (voltage and current are standard; power, temperature, RSSI optional).
 *          Unlike platforms with ADC hardware that can support 8-16 analog inputs, QURT is
 *          constrained to ESC-provided measurements.
 *          
 *          Initialization: The init() method sets up communication with the RCOutput driver
 *          to receive ESC telemetry data containing analog measurements.
 *          
 *          Typical Usage Pattern:
 *          1. Vehicle calls AP::analogin().init() during startup
 *          2. AP_BattMonitor requests voltage channel: analogin().channel(0)
 *          3. AP_BattMonitor requests current channel: analogin().channel(1)
 *          4. Each channel reads from ESC telemetry via RCOutput driver
 * 
 * @note Limited channels: Only 2-4 channels available (voltage, current, maybe power/temp)
 * @note Telemetry source: All channels read from RCOutput::esc_power_status structure
 * @note No arbitrary pins: Cannot create channels for arbitrary GPIO/ADC pins
 * @note DSP architecture: No direct ADC hardware on Hexagon DSP platform
 * 
 * @warning Adding new channels requires ESC controller firmware modifications
 * @warning Channel availability depends on ESC telemetry protocol capabilities
 * 
 * @see QURT::AnalogSource for the analog input channel implementation
 * @see AP_HAL::AnalogIn for the abstract interface definition
 * @see RCOutput.h for ESC telemetry data structures
 */
class QURT::AnalogIn : public AP_HAL::AnalogIn
{
public:
    /**
     * @brief Constructor for QURT analog input manager
     * 
     * @details Initializes the analog input subsystem. Sets up internal state for
     *          managing analog source channels. Does not initialize hardware since
     *          QURT has no direct ADC hardware; actual initialization happens in init().
     */
    AnalogIn();
    
    /**
     * @brief Initialize analog input subsystem
     * 
     * @details Performs analog input initialization for QURT platform. Sets up connection
     *          to RCOutput driver to receive ESC telemetry data containing voltage and
     *          current measurements. Called during vehicle startup before analog channels
     *          are requested.
     *          
     *          Unlike other HAL platforms that initialize ADC hardware registers, this
     *          method establishes the telemetry data path from ESC through RCOutput.
     * 
     * @note Called once during vehicle initialization
     * @note No hardware initialization needed (no ADC on DSP)
     * @note Must complete before channel() calls
     * 
     * @warning ESC telemetry must be configured and active for analog inputs to work
     */
    void init() override;
    
    /**
     * @brief Create analog source for specified channel
     * 
     * @details Returns an AnalogSource instance configured for the requested channel number.
     *          Channel numbers map to specific ESC telemetry measurements:
     *          - n=0: Board voltage (typical: battery voltage for monitoring)
     *          - n=1: Current draw (typical: total current for power monitoring)
     *          - n=2: Power or temperature (if ESC telemetry supports)
     *          - n=3: RSSI or auxiliary (if ESC telemetry supports)
     *          
     *          The returned AnalogSource object will read from the corresponding field in
     *          the RCOutput driver's esc_power_status telemetry structure.
     *          
     *          Typical Callers: AP_BattMonitor requests channels 0 (voltage) and 1 (current)
     *          for battery monitoring. Other subsystems may request additional channels if
     *          available.
     * 
     * @param[in] n Channel number (0-3 typical range, maps to telemetry field not GPIO pin)
     * 
     * @return Pointer to AnalogSource for the requested channel, or nullptr if invalid
     * 
     * @note Channel numbers limited by ESC telemetry capabilities (typically 0-3)
     * @note Returned pointer valid for lifetime of vehicle (not dynamically freed)
     * @note Multiple calls with same channel number return different AnalogSource instances
     * 
     * @warning Invalid channel numbers may return null or non-functional AnalogSource
     * @warning Channel availability depends on ESC firmware and telemetry protocol
     * 
     * @see QURT::AnalogSource for the returned object type
     * @see AP_BattMonitor for primary usage of analog channels
     */
    AP_HAL::AnalogSource* channel(int16_t n) override;
    
    /**
     * @brief Get board voltage measurement
     * 
     * @details Convenience method to read board/battery voltage directly without creating
     *          an AnalogSource channel. Returns voltage from ESC telemetry channel 0
     *          (standard board voltage measurement). Commonly used for quick voltage checks.
     *          
     *          Equivalent to: channel(0)->voltage_average()
     *          
     *          Primary Use: Battery monitoring and low-voltage failsafe checks.
     * 
     * @return Board voltage in volts (V), averaged from ESC telemetry
     * 
     * @note Reads from ESC telemetry channel 0 (voltage)
     * @note Averaged reading for noise reduction
     * @note Safety-critical: Used for battery failsafe decisions
     * 
     * @warning Returns stale value if ESC telemetry communication has failed
     * @warning Voltage accuracy depends on ESC ADC calibration
     * 
     * @see voltage_average() in AnalogSource for detailed reading information
     */
    float board_voltage(void) override;
    
private:
    /**
     * @brief Counter for allocating analog source channels
     * 
     * @details Tracks the next available channel slot when creating AnalogSource instances.
     *          Used internally by channel() method to allocate channel resources. Increments
     *          each time a new channel is requested.
     * 
     * @note Internal implementation detail for channel allocation
     */
    uint8_t next_chan;
};
