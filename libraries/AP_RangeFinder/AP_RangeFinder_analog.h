/**
 * @file AP_RangeFinder_analog.h
 * @brief Analog voltage rangefinder backend driver
 * 
 * This file implements a rangefinder driver that converts analog voltage readings
 * from an ADC (Analog-to-Digital Converter) into distance measurements. The driver
 * supports configurable transfer functions to accommodate different sensor output
 * characteristics, allowing integration with a variety of analog distance sensors
 * that output voltage proportional to distance.
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_ANALOG_ENABLED

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend.h"
#include "AP_RangeFinder_Params.h"

/**
 * @class AP_RangeFinder_analog
 * @brief Rangefinder backend for analog voltage sensors
 * 
 * @details This class implements distance measurement using analog voltage sensors
 *          connected to an ADC input. The driver reads voltage from a configured
 *          analog pin and converts it to distance using one of three configurable
 *          transfer functions:
 * 
 *          **Transfer Functions:**
 *          - **LINEAR**: distance = (voltage - offset) * scaling
 *            - Used for sensors with linear voltage-to-distance relationship
 *            - Most common type for analog rangefinders
 *          
 *          - **INVERTED**: distance = (offset - voltage) * scaling
 *            - Used for sensors where voltage decreases as distance increases
 *            - Offset represents maximum voltage (minimum distance)
 *          
 *          - **HYPERBOLA**: distance = scaling / (voltage - offset)
 *            - Used for sensors with inverse relationship (e.g., infrared sensors)
 *            - Offset represents voltage at infinite distance
 *            - Automatically clamps to maximum distance to prevent overflow
 * 
 *          **Voltage Reading Modes:**
 *          - **Ratiometric**: Voltage measured relative to supply voltage (VCC)
 *            - Compensates for supply voltage variations
 *            - Recommended for sensors powered from same supply as ADC
 *          
 *          - **Absolute**: Voltage measured against fixed reference
 *            - Uses ADC's internal voltage reference
 *            - Required for sensors with independent power supply
 * 
 *          **Parameters (configured via AP_RangeFinder_Params):**
 *          - pin: ADC pin number for analog input
 *          - scaling: Scaling factor for voltage-to-distance conversion (sensor-specific)
 *          - offset: Voltage offset for transfer function (volts)
 *          - function: Transfer function type (LINEAR, INVERTED, or HYPERBOLA)
 *          - ratiometric: Enable ratiometric voltage measurement
 * 
 *          **Typical Usage Workflow:**
 *          1. Detection: detect() checks if pin is valid (pin != -1)
 *          2. Construction: Allocates analog input channel
 *          3. Update loop: update() called periodically (~10Hz) to read sensor
 *          4. Conversion: Voltage converted to distance using selected function
 *          5. Validation: Distance validated against min/max range limits
 * 
 *          **Example Sensor Configurations:**
 *          
 *          *Sharp GP2Y0A21YK (IR rangefinder, 10-80cm):*
 *          - Function: HYPERBOLA
 *          - Scaling: 0.42 (sensor-specific constant)
 *          - Offset: 0.0
 *          
 *          *MaxBotix LV-EZ (ultrasonic, analog output):*
 *          - Function: LINEAR
 *          - Scaling: 2.54 (converts mV/cm to m)
 *          - Offset: 0.0
 * 
 * @note The driver does not perform sensor-specific initialization or calibration.
 *       All configuration is performed through parameters.
 * 
 * @warning Distance measurements are only as accurate as the ADC resolution and
 *          sensor linearity. Noise filtering may be required for stable readings.
 * 
 * @see AP_RangeFinder_Backend
 * @see AP_RangeFinder_Params
 * @see RangeFinder::Function
 */
class AP_RangeFinder_analog : public AP_RangeFinder_Backend
{
public:
    /**
     * @brief Construct analog rangefinder backend and initialize ADC channel
     * 
     * @details This constructor allocates an analog input channel from the HAL
     *          using the pin number configured in parameters. The constructor is
     *          only called after detect() returns true, ensuring the pin parameter
     *          is valid.
     *          
     *          Initialization sequence:
     *          1. Calls parent constructor (AP_RangeFinder_Backend)
     *          2. Allocates analog input channel via hal.analogin->channel()
     *          3. Sets initial status based on channel allocation success
     *          
     *          If channel allocation fails (source == nullptr), the sensor
     *          is marked as NotConnected. Otherwise, status is set to NoData
     *          until the first valid reading is received.
     * 
     * @param[in,out] _state Reference to rangefinder state structure for storing
     *                       measurements (distance, voltage, status)
     * @param[in] _params Reference to parameter structure containing pin number,
     *                    scaling, offset, function type, and ratiometric setting
     * 
     * @note This constructor assumes detect() has already validated the pin number
     * @warning Constructor may set status to NotConnected if ADC channel allocation
     *          fails, which typically indicates hardware resource exhaustion
     * 
     * @see detect()
     * @see AP_HAL::AnalogIn::channel()
     * 
     * Source: libraries/AP_RangeFinder/AP_RangeFinder_analog.cpp:38-48
     */
    AP_RangeFinder_analog(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params);

    /**
     * @brief Detect if analog rangefinder is configured and available
     * 
     * @details Static detection function called during rangefinder initialization
     *          to determine if an analog rangefinder should be instantiated. The
     *          only validation performed is checking if a valid pin number has
     *          been configured.
     *          
     *          Detection logic:
     *          - If pin != -1: Sensor is assumed present (returns true)
     *          - If pin == -1: No sensor configured (returns false)
     *          
     *          This minimal detection is appropriate for analog sensors since there
     *          is no way to probe or auto-detect an analog sensor - any voltage on
     *          the pin could be valid sensor output.
     * 
     * @param[in] _params Reference to parameter structure to check pin configuration
     * 
     * @return true if pin is configured (pin != -1), false if pin is not configured
     * 
     * @note This function does not verify hardware capability or pin validity,
     *       only that a pin has been configured in parameters
     * @note If detection succeeds, the constructor will be called to create the backend
     * 
     * Source: libraries/AP_RangeFinder/AP_RangeFinder_analog.cpp:55-61
     */
    static bool detect(AP_RangeFinder_Params &_params);

    /**
     * @brief Update rangefinder state with new distance measurement
     * 
     * @details Main update function called periodically (typically ~10Hz) to read
     *          the analog sensor and update the distance measurement. This function
     *          performs the complete measurement cycle:
     *          
     *          Update sequence:
     *          1. update_voltage(): Read raw ADC voltage (ratiometric or absolute)
     *          2. Apply transfer function: Convert voltage to distance in meters
     *          3. Clamp distance: Apply minimum (0) and maximum range limits
     *          4. Update state: Store distance_m and last_reading_ms
     *          5. update_status(): Validate range and update sensor status
     *          
     *          **Transfer function application:**
     *          - LINEAR: distance = (voltage - offset) * scaling
     *          - INVERTED: distance = (offset - voltage) * scaling
     *          - HYPERBOLA: distance = scaling / (voltage - offset)
     *          
     *          Negative distances are clamped to zero. For HYPERBOLA function,
     *          distances exceeding max_distance are clamped to prevent overflow
     *          when voltage approaches offset value.
     * 
     * @note This function overrides AP_RangeFinder_Backend::update()
     * @note Called at main loop rate by AP_RangeFinder::update() for all backends
     * @note Distance is stored in meters in state.distance_m
     * @note Voltage is stored in millivolts in state.voltage_mv
     * 
     * @warning For HYPERBOLA function, if voltage <= offset, distance is set to 0
     *          to prevent division by zero or negative denominators
     * 
     * @see update_voltage()
     * @see AP_RangeFinder_Backend::update_status()
     * @see RangeFinder::Function
     * 
     * Source: libraries/AP_RangeFinder/AP_RangeFinder_analog.cpp:84-122
     */
    void update(void) override;

protected:

    /**
     * @brief Get MAVLink distance sensor type identifier
     * 
     * @details Returns the MAVLink distance sensor type for this rangefinder backend.
     *          Since analog rangefinders can be connected to many different sensor
     *          types (ultrasonic, infrared, laser, etc.), the specific sensor type
     *          cannot be determined. Therefore, this function returns UNKNOWN.
     *          
     *          This type is used in MAVLink DISTANCE_SENSOR messages to inform the
     *          ground control station about the sensor technology being used.
     * 
     * @return MAV_DISTANCE_SENSOR_UNKNOWN indicating generic analog sensor
     * 
     * @note This function overrides AP_RangeFinder_Backend::_get_mav_distance_sensor_type()
     * @note The actual sensor type (IR, ultrasonic, etc.) is not detectable from
     *       an analog voltage signal alone
     * 
     * @see AP_RangeFinder_Backend::_get_mav_distance_sensor_type()
     */
    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_UNKNOWN;
    }

private:
    /**
     * @brief Update internal voltage state from ADC reading
     * 
     * @details Reads the current voltage from the analog input channel and stores
     *          it in state.voltage_mv (in millivolts). The voltage reading method
     *          depends on the ratiometric parameter setting:
     *          
     *          **Ratiometric mode (params.ratiometric == true):**
     *          - Voltage measured relative to VCC (supply voltage)
     *          - Uses voltage_average_ratiometric() method
     *          - Compensates for variations in supply voltage
     *          - Recommended when sensor and ADC share power supply
     *          
     *          **Absolute mode (params.ratiometric == false):**
     *          - Voltage measured against internal reference
     *          - Uses voltage_average() method  
     *          - Provides absolute voltage measurement
     *          - Required when sensor has independent power supply
     *          
     *          Error handling:
     *          - If source is nullptr: Sets voltage to 0, status to NotConnected
     *          - If set_pin() fails: Sets voltage to 0, status to NotConnected
     *          
     *          The function uses averaged voltage readings to reduce noise.
     * 
     * @note Voltage is stored as uint16_t in millivolts (voltage * 1000)
     * @note Called internally by update() before distance conversion
     * @note Sets sensor status to NotConnected on any error condition
     * 
     * @warning If the analog source is null or pin configuration fails, the
     *          sensor will be marked as not connected and no distance will be reported
     * 
     * @see update()
     * @see AP_HAL::AnalogSource::voltage_average()
     * @see AP_HAL::AnalogSource::voltage_average_ratiometric()
     * 
     * Source: libraries/AP_RangeFinder/AP_RangeFinder_analog.cpp:67-79
     */
    void update_voltage(void);

    /**
     * @brief Pointer to HAL analog input source
     * 
     * @details Handle to the hardware abstraction layer's analog input channel
     *          allocated for this rangefinder. The source is obtained from
     *          hal.analogin->channel(pin) during construction and provides access
     *          to ADC voltage readings.
     *          
     *          The source pointer may be nullptr if:
     *          - ADC channel allocation failed during construction
     *          - All available ADC channels are already in use
     *          - The specified pin is not a valid analog input
     *          
     *          If nullptr, update_voltage() will set status to NotConnected.
     * 
     * @note Allocated in constructor via hal.analogin->channel()
     * @note Managed by HAL - not freed by this class
     * @note Null pointer indicates hardware resource exhaustion or invalid pin
     * 
     * @see AP_HAL::AnalogSource
     * @see AP_HAL::AnalogIn::channel()
     */
    AP_HAL::AnalogSource *source;
};

#endif  // AP_RANGEFINDER_ANALOG_ENABLED
