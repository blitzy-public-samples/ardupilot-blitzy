/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * @file AP_RangeFinder_PWM.h
 * @brief PWM-based rangefinder backend driver
 * 
 * @details This file implements support for generic PWM input rangefinder sensors
 *          where distance is encoded as PWM pulse width. The driver reads PWM signals
 *          from a GPIO pin and converts pulse width to distance measurements using
 *          configurable scaling parameters.
 *          
 *          PWM rangefinders output distance information by varying the pulse width
 *          of a periodic signal. This driver uses the HAL PWMSource interface to
 *          measure pulse widths and applies linear scaling to convert pulse width
 *          (typically in microseconds) to distance (in meters).
 *          
 *          Common applications include sonar sensors and certain laser rangefinders
 *          that provide PWM output interfaces.
 * 
 * @note This driver requires hardware PWM input capture capability on the target board
 * @note Power saving features can disable the sensor when out of range to reduce power
 * 
 * Source: libraries/AP_RangeFinder/AP_RangeFinder_PWM.h
 */

#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_PWM_ENABLED

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend.h"

/**
 * @class AP_RangeFinder_PWM
 * @brief PWM rangefinder backend driver for generic PWM input distance sensors
 * 
 * @details This class implements a rangefinder backend for sensors that encode
 *          distance measurements as PWM pulse widths. The driver:
 *          - Uses AP_HAL::PWMSource to capture PWM input on a configured GPIO pin
 *          - Converts pulse width (microseconds) to distance (meters) using linear scaling
 *          - Supports configurable scaling parameters (SCALING, MIN_CM, MAX_CM)
 *          - Implements power saving by disabling sensor when out of range
 *          - Provides terrain height estimation integration
 * 
 *          **PWM to Distance Conversion**:
 *          The conversion from PWM pulse width to distance is performed using
 *          parameters configured in AP_RangeFinder_Params:
 *          - SCALING: Scaling factor for pulse width to distance conversion
 *          - MIN_CM: Minimum valid distance in centimeters
 *          - MAX_CM: Maximum valid distance in centimeters
 *          
 *          **Power Saving**:
 *          When enabled (POWERSAVE parameter), the driver can control a stop pin
 *          to power down the sensor when readings indicate the target is beyond
 *          the power-saving range, reducing overall system power consumption.
 *          
 *          **Hardware Requirements**:
 *          - GPIO pin with PWM input capture capability
 *          - Optional GPIO pin for sensor power control (stop pin)
 *          - PWM signal typically 3.3V or 5V logic level
 * 
 * @note Inherits from AP_RangeFinder_Backend for common rangefinder functionality
 * @note Thread-safety: update() called from main scheduler loop at configured rate
 * @note MAVLink sensor type reported as MAV_DISTANCE_SENSOR_UNKNOWN (generic PWM)
 * 
 * Source: libraries/AP_RangeFinder/AP_RangeFinder_PWM.cpp
 */
class AP_RangeFinder_PWM : public AP_RangeFinder_Backend
{
public:
    /**
     * @brief Construct a PWM rangefinder backend instance
     * 
     * @details Initializes the PWM rangefinder backend with references to the
     *          rangefinder state, configuration parameters, and terrain height estimate.
     *          The constructor stores references but does not initialize hardware -
     *          actual pin configuration and PWM source setup occurs during detect()
     *          and subsequent update() calls.
     * 
     * @param[in,out] _state Reference to RangeFinder_State for storing sensor readings
     * @param[in]     _params Reference to AP_RangeFinder_Params with configuration (pin, scaling, limits)
     * @param[in,out] _estimated_terrain_height Reference to terrain height estimate for integration
     * 
     * @note Does not perform hardware initialization in constructor
     * @note All parameters must remain valid for the lifetime of this object
     * 
     * Source: libraries/AP_RangeFinder/AP_RangeFinder_PWM.cpp:20-25
     */
    AP_RangeFinder_PWM(RangeFinder::RangeFinder_State &_state,
                       AP_RangeFinder_Params &_params,
                       float &_estimated_terrain_height);

    /**
     * @brief Destructor for PWM rangefinder backend
     * 
     * @details Cleanup is handled automatically by PWMSource destructor.
     *          No explicit resource deallocation required.
     */
    ~AP_RangeFinder_PWM(void) {};

    /**
     * @brief Detect if PWM rangefinder hardware is available
     * 
     * @details Static detection function called during rangefinder initialization
     *          to determine if PWM rangefinder capability is present on the board.
     *          For PWM rangefinders, detection always returns true if the feature
     *          is compiled in, as actual pin availability is checked during runtime
     *          initialization in check_pin().
     * 
     * @return true if PWM rangefinder support is compiled and available
     * @return false if PWM rangefinder support is not available
     * 
     * @note This is a compile-time capability check, not a runtime hardware probe
     * @note Actual pin validation occurs in check_pin() during update()
     * 
     * Source: libraries/AP_RangeFinder/AP_RangeFinder_PWM.cpp:29-32
     */
    static bool detect();

    /**
     * @brief Update rangefinder state with latest PWM reading
     * 
     * @details Called periodically by the scheduler to update the rangefinder state.
     *          This method:
     *          1. Validates and initializes the configured PWM input pin (first call)
     *          2. Checks and configures the optional power control stop pin
     *          3. Reads the current PWM pulse width from the hardware
     *          4. Converts pulse width to distance using configured scaling
     *          5. Updates rangefinder state with new distance reading
     *          6. Manages power saving by controlling stop pin based on range
     *          
     *          **Update Frequency**: Called at the rate specified by the scheduler,
     *          typically 20-50Hz for rangefinders.
     *          
     *          **PWM Reading**: Uses AP_HAL::PWMSource to get the latest pulse width
     *          in microseconds, then applies linear scaling to convert to meters.
     *          
     *          **Power Saving**: If POWERSAVE parameter is enabled and the target
     *          is beyond the power-saving range, the stop pin is activated to
     *          disable the sensor and reduce power consumption.
     * 
     * @note Overrides AP_RangeFinder_Backend::update()
     * @note Called from main scheduler loop context
     * @note Pin initialization is deferred until first update() call
     * 
     * Source: libraries/AP_RangeFinder/AP_RangeFinder_PWM.cpp:81-94
     */
    void update(void) override;

protected:

    /**
     * @brief Get the latest distance reading from PWM input
     * 
     * @details Retrieves the current PWM pulse width from the hardware and converts
     *          it to a distance measurement in meters. This method:
     *          1. Calls pwm_source.get_pwm_us() to get pulse width in microseconds
     *          2. Validates the pulse width is non-zero (sensor active)
     *          3. Applies configured SCALING factor to convert to distance
     *          4. Validates result is within configured MIN_CM to MAX_CM range
     *          
     *          **PWM to Distance Conversion Formula**:
     *          distance_m = (pulse_width_us * SCALING) / 1000.0
     *          
     *          The SCALING parameter is sensor-specific and must be configured
     *          to match the sensor's PWM output characteristics.
     * 
     * @param[out] reading_m Distance reading in meters if successful
     * 
     * @return true if valid reading obtained and within configured limits
     * @return false if no PWM signal, invalid pulse width, or out of range
     * 
     * @note Returns false if pulse width is zero (no signal detected)
     * @note Readings outside MIN_CM to MAX_CM are rejected as invalid
     * 
     * Source: libraries/AP_RangeFinder/AP_RangeFinder_PWM.cpp:96-113
     */
    bool get_reading(float &reading_m);

    /**
     * @brief Get MAVLink distance sensor type identifier
     * 
     * @details Returns the MAVLink sensor type enumeration for this rangefinder.
     *          PWM rangefinders are generic and could represent various sensor types
     *          (sonar, laser, etc.), so we report MAV_DISTANCE_SENSOR_UNKNOWN to
     *          indicate the specific technology is not identified by the protocol.
     * 
     * @return MAV_DISTANCE_SENSOR_UNKNOWN indicating generic PWM sensor
     * 
     * @note Overrides AP_RangeFinder_Backend::_get_mav_distance_sensor_type()
     * @note This affects the MAVLink DISTANCE_SENSOR message sensor_type field
     */
    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_UNKNOWN;
    }

private:

    /**
     * @brief Validate and initialize the PWM input pin
     * 
     * @details Checks if the configured PWM input pin (params.pin parameter) is valid
     *          and initializes the PWMSource for reading. This method:
     *          1. Validates the pin number is not -1 (disabled)
     *          2. Calls pwm_source.set_pin() to configure the HAL PWM input
     *          3. Returns success/failure status
     *          
     *          Pin initialization is performed once during the first update() call.
     *          Subsequent calls return quickly if already initialized.
     * 
     * @return true if pin is valid and successfully configured
     * @return false if pin is -1 (disabled) or initialization fails
     * 
     * @note Pin number comes from params.pin configuration parameter
     * @note Called during first update() to defer hardware initialization
     * 
     * Source: libraries/AP_RangeFinder/AP_RangeFinder_PWM.cpp:34-44
     */
    bool check_pin();
    
    /**
     * @brief Check and configure the optional power control stop pin
     * 
     * @details Validates and configures the GPIO pin used to control sensor power
     *          for power-saving functionality. If params.stop_pin is configured
     *          (not -1) and different from the last configured pin, this method
     *          sets up the pin as a GPIO output for controlling sensor power.
     *          
     *          The stop pin is used to disable the sensor when out_of_range()
     *          returns true, reducing system power consumption.
     * 
     * @note Pin configuration occurs when params.stop_pin changes
     * @note Stop pin is optional - power saving disabled if stop_pin is -1
     * 
     * Source: libraries/AP_RangeFinder/AP_RangeFinder_PWM.cpp:46-57
     */
    void check_stop_pin();
    
    /**
     * @brief Validate and initialize both PWM input and stop pins
     * 
     * @details Convenience method that calls both check_pin() and check_stop_pin()
     *          to ensure all required pins are properly configured. Called during
     *          update() to handle pin initialization and reconfiguration.
     * 
     * @return true if PWM input pin is valid and configured successfully
     * @return false if PWM input pin initialization fails
     * 
     * @note Stop pin configuration failure does not affect return value (optional)
     * 
     * Source: libraries/AP_RangeFinder/AP_RangeFinder_PWM.cpp:59-63
     */
    bool check_pins();
    
    /**
     * @brief Last configured stop pin number
     * 
     * @details Tracks the stop pin number from the previous configuration check
     *          to detect when params.stop_pin changes and requires reconfiguration.
     *          Initialized to -1 (invalid pin) to ensure first configuration attempt.
     */
    uint8_t last_stop_pin = -1;

    /**
     * @brief HAL PWM input source for reading pulse widths
     * 
     * @details Hardware abstraction layer interface for capturing PWM input signals.
     *          Provides get_pwm_us() to read the latest pulse width in microseconds.
     *          Handles platform-specific PWM input capture implementation.
     * 
     * @note Uses interrupt-driven or DMA-based capture depending on HAL implementation
     */
    AP_HAL::PWMSource pwm_source;

    /**
     * @brief Reference to estimated terrain height
     * 
     * @details Reference to the terrain height estimate maintained by the rangefinder
     *          system. Updated by the backend to support terrain-relative navigation
     *          and altitude estimation. Passed in during construction.
     */
    float &estimated_terrain_height;

    /**
     * @brief Check if sensor reading indicates target beyond power-saving range
     * 
     * @details Determines if the current distance reading exceeds the power-saving
     *          threshold, indicating the sensor can be powered down to conserve energy.
     *          This is used to control the stop pin when POWERSAVE parameter is enabled.
     *          
     *          The power-saving range is typically set to a distance beyond which
     *          measurements are not critical for vehicle operation.
     * 
     * @return true if target is beyond power-saving range (sensor can be disabled)
     * @return false if target is within operational range (sensor must remain active)
     * 
     * @note Power saving only active if params.powersave is enabled
     * @note Exact range threshold determined by sensor configuration
     * 
     * Source: libraries/AP_RangeFinder/AP_RangeFinder_PWM.cpp:65-79
     */
    bool out_of_range(void) const;
    
    /**
     * @brief Previous out-of-range state for power transition detection
     * 
     * @details Tracks the previous out_of_range() result to detect transitions
     *          between in-range and out-of-range states. Initialized to -1 (invalid)
     *          to ensure the first state update triggers appropriate stop pin action.
     *          
     *          State transitions:
     *          - false -> true: Target moved out of range, activate stop pin
     *          - true -> false: Target in range again, deactivate stop pin
     * 
     * @note Odd initialization to -1 ensures first comparison always detects transition
     */
    bool was_out_of_range = -1; // this odd initialisation ensures we transition to new state

};

#endif  // AP_RANGEFINDER_PWM_ENABLED
