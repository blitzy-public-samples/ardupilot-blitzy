/**
 * @file AP_Airspeed_analog.h
 * @brief Analog voltage-based airspeed sensor backend driver
 * 
 * This file implements an airspeed sensor backend for simple analog voltage
 * sensors that output a voltage proportional to differential pressure between
 * pitot and static ports. The ratiometric voltage measurement is converted
 * to differential pressure in Pascals using a fixed conversion constant.
 * 
 * Typical sensors: MPXV7002DP, MPXV5004DP, and similar analog pressure sensors
 * 
 * @note Analog sensors do not provide temperature measurement capability
 * 
 * Source: libraries/AP_Airspeed/AP_Airspeed_analog.h:1-30
 * Source: libraries/AP_Airspeed/AP_Airspeed_analog.cpp:30-53
 */

#pragma once

#include "AP_Airspeed_config.h"

#if AP_AIRSPEED_ANALOG_ENABLED

#include <AP_HAL/AP_HAL.h>

#include "AP_Airspeed_Backend.h"

/**
 * @class AP_Airspeed_Analog
 * @brief Analog voltage-based airspeed sensor backend
 * 
 * @details This backend implements support for simple analog voltage airspeed
 *          sensors where the output voltage is proportional to differential
 *          pressure between the pitot tube and static port.
 *          
 *          Operating Principle:
 *          - Analog pressure sensor outputs voltage proportional to pressure
 *          - HAL provides ratiometric voltage reading (0.0-1.0 scale referenced to supply)
 *          - Voltage is converted to pressure using VOLTS_TO_PASCAL constant (819)
 *          - Final pressure adjusted by PSI range parameter (get_psi_range())
 *          
 *          Conversion Formula:
 *          pressure = voltage_ratiometric * VOLTS_TO_PASCAL / psi_range
 *          
 *          Typical Applications:
 *          - 5V analog differential pressure sensors
 *          - Simple fixed-wing applications
 *          - Cost-sensitive installations
 *          
 *          Limitations:
 *          - No temperature measurement (returns false for get_temperature())
 *          - No self-diagnostics or health monitoring
 *          - Requires manual calibration via parameters
 *          - Susceptible to electrical noise
 *          
 * @note Uses ARSPD_PIN parameter to configure analog input pin
 * @note Ratiometric measurement compensates for supply voltage variations
 * 
 * @see AP_Airspeed_Backend for base interface
 * @see MPXV7002DP datasheet for typical sensor characteristics
 */
class AP_Airspeed_Analog : public AP_Airspeed_Backend
{
public:
    /**
     * @brief Construct analog airspeed sensor backend
     * 
     * @details Constructor initializes the analog input source using the pin
     *          configured via the ARSPD_PIN parameter. The HAL analog input
     *          interface is accessed through hal.analogin to create a ratiometric
     *          voltage source.
     * 
     * @param[in] frontend Reference to AP_Airspeed frontend managing this sensor
     * @param[in] _instance Sensor instance index (0-based) for multi-sensor support
     * 
     * @note Analog source is created but not validated until init() is called
     * @note Pin number is retrieved via get_pin() from base class parameter
     * 
     * Source: libraries/AP_Airspeed/AP_Airspeed_analog.cpp:33-37
     */
    AP_Airspeed_Analog(AP_Airspeed &frontend, uint8_t _instance);

    /**
     * @brief Probe and initialize the analog airspeed sensor
     * 
     * @details Validates that the analog input source was successfully created
     *          during construction. This is a simple check that the HAL was able
     *          to allocate an analog input channel for the configured pin.
     *          
     *          No actual sensor probing occurs since analog sensors have no
     *          digital interface for identification.
     * 
     * @return true if analog source is valid (not nullptr), false otherwise
     * 
     * @note This method is called during sensor initialization at startup
     * @note Uses ARSPD_PIN parameter to determine which analog input to use
     * @note Failure typically indicates invalid pin configuration
     * 
     * Source: libraries/AP_Airspeed/AP_Airspeed_analog.cpp:39-42
     */
    bool init(void) override;

    /**
     * @brief Read analog voltage and convert to differential pressure
     * 
     * @details Reads the ratiometric analog voltage from the pressure sensor
     *          and converts it to differential pressure in Pascals using the
     *          fixed conversion constant VOLTS_TO_PASCAL (819).
     *          
     *          Conversion Process:
     *          1. Check analog source validity and pin configuration
     *          2. Read ratiometric voltage (0.0-1.0 scale referenced to supply)
     *          3. Convert: pressure = voltage * VOLTS_TO_PASCAL / psi_range
     *          
     *          The ratiometric measurement compensates for supply voltage
     *          variations, improving accuracy across different power conditions.
     *          The PSI range parameter (get_psi_range()) allows calibration
     *          for different sensor pressure ranges.
     * 
     * @param[out] pressure Differential pressure in Pascals
     * 
     * @return true if valid reading obtained, false if source invalid or pin change failed
     * 
     * @note This method is called at the main sensor update rate (typically 10-50 Hz)
     * @note Ratiometric voltage is 0.0-1.0 scale (not absolute volts)
     * @note VOLTS_TO_PASCAL = 819 is calibrated for 3DR analog airspeed sensor
     * @note Pin can be changed dynamically via parameter update
     * 
     * @warning Analog readings are susceptible to electrical noise and require filtering
     * 
     * @see voltage_average_ratiometric() for HAL voltage measurement interface
     * 
     * Source: libraries/AP_Airspeed/AP_Airspeed_analog.cpp:45-53
     */
    bool get_differential_pressure(float &pressure) override;

    /**
     * @brief Attempt to get temperature reading (not supported)
     * 
     * @details Analog pressure sensors do not provide temperature measurement
     *          capability. This method always returns false to indicate
     *          temperature is unavailable.
     *          
     *          Temperature compensation for analog sensors must be performed
     *          using external temperature sources or empirical calibration.
     * 
     * @param[out] temperature Temperature in degrees Celsius (not modified)
     * 
     * @return false always - analog sensors do not provide temperature
     * 
     * @note Frontend handles missing temperature by using alternative sources
     * @note Digital airspeed sensors (I2C/SPI) typically do provide temperature
     */
    bool get_temperature(float &temperature) override { return false; }

private:
    /**
     * @brief Pointer to HAL analog input source for ratiometric voltage readings
     * 
     * @details This pointer references the Hardware Abstraction Layer (HAL)
     *          analog input channel configured for the airspeed sensor. The HAL
     *          provides platform-independent access to analog-to-digital converter
     *          (ADC) hardware.
     *          
     *          The analog source provides:
     *          - Ratiometric voltage measurement (0.0-1.0 scale)
     *          - Automatic averaging of multiple samples
     *          - Supply voltage compensation
     *          - Dynamic pin reconfiguration
     *          
     *          Ratiometric Measurement:
     *          Reading is referenced to supply voltage, so variations in the
     *          supply (e.g., 4.8V vs 5.2V) are automatically compensated.
     *          This improves accuracy for sensors powered from the same supply.
     * 
     * @note Initialized in constructor via hal.analogin->channel(pin)
     * @note nullptr indicates HAL could not allocate analog channel
     * @note Pin number comes from ARSPD_PIN parameter via get_pin()
     * 
     * @see AP_HAL::AnalogIn for HAL analog input interface
     * @see AP_HAL::AnalogSource for ratiometric voltage measurement interface
     */
    AP_HAL::AnalogSource *_source;
};

#endif  // AP_AIRSPEED_ANALOG_ENABLED
