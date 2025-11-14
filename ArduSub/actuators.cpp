/**
 * @file actuators.cpp
 * @brief Auxiliary actuator management system for ArduSub underwater vehicles
 * 
 * @details This file implements the actuators subsystem which manages auxiliary
 *          actuators beyond the main thrusters. These actuators typically control
 *          peripheral devices such as grippers, lights, camera tilts, and other
 *          accessories that require PWM servo output control.
 *          
 *          The actuators system provides:
 *          - Up to ACTUATOR_CHANNELS (6) independently controlled auxiliary outputs
 *          - Normalized value storage (0.0 to 1.0 range) mapped to servo PWM ranges
 *          - Configurable increment steps for manual control adjustments
 *          - Direct control methods (increase, decrease, min, max, center, toggle)
 *          - Integration with the SRV_Channel servo output system
 *          
 *          Actuator values are maintained as normalized floats (0.0-1.0) internally
 *          and converted to PWM pulse widths (typically 1000-2000μs) based on each
 *          servo channel's configured min/max/trim parameters.
 *          
 *          This system is designed for operator control via joystick buttons or
 *          ground control station commands, providing intuitive manual adjustment
 *          of auxiliary functions during underwater operations.
 * 
 * @note Actuators are distinct from thrusters - they control accessories, not propulsion
 * @warning Actuator outputs must be properly configured to avoid damage to connected devices
 * 
 * @see actuators.h for class definition and ACTUATOR_CHANNELS constant
 * @see SRV_Channel for underlying servo output management
 * 
 * Source: ArduSub/actuators.cpp
 */

#include "Sub.h"
#include "actuators.h"


/**
 * @brief Parameter group definition for actuator configuration
 * 
 * @details Defines AP_Param parameters for configuring actuator behavior.
 *          Each actuator has an increment step parameter (*_INC) that controls
 *          how much the actuator value changes with each increase/decrease command.
 *          
 *          Parameter naming convention: ACTUATORS_N_INC where N is actuator number (1-6)
 *          
 *          The increment step is specified in microseconds and determines the PWM
 *          change per button press or command. Default is ACTUATOR_DEFAULT_INCREMENT.
 *          Smaller values provide finer control, larger values provide faster adjustment.
 */
const AP_Param::GroupInfo Actuators::var_info[] = {

    // @Param: 1_INC
    // @DisplayName: Increment step for actuator 1
    // @Description:  Initial increment step for changing the actuator's PWM
    // @Units: us
    // @User: Standard
    AP_GROUPINFO("1_INC", 1, Actuators, actuator_increment_step[0], ACTUATOR_DEFAULT_INCREMENT),

    // @Param: 2_INC
    // @DisplayName: Increment step for actuator 2
    // @Description:  Initial increment step for changing the actuator's PWM
    // @Units: us
    // @User: Standard
    AP_GROUPINFO("2_INC", 2, Actuators, actuator_increment_step[1], ACTUATOR_DEFAULT_INCREMENT),

    // @Param: 3_INC
    // @DisplayName: Increment step for actuator 3
    // @Description:  Initial increment step for changing the actuator's PWM
    // @Units: us
    // @User: Standard
    AP_GROUPINFO("3_INC", 3, Actuators, actuator_increment_step[2], ACTUATOR_DEFAULT_INCREMENT),

    // @Param: 4_INC
    // @DisplayName: Increment step for actuator 4
    // @Description:  Initial increment step for changing the actuator's PWM
    // @Units: us
    // @User: Standard
    AP_GROUPINFO("4_INC", 4, Actuators, actuator_increment_step[3], ACTUATOR_DEFAULT_INCREMENT),

    // @Param: 5_INC
    // @DisplayName: Increment step for actuator 5
    // @Description:  Initial increment step for changing the actuator's PWM
    // @Units: us
    // @User: Standard
    AP_GROUPINFO("5_INC", 5, Actuators, actuator_increment_step[4], ACTUATOR_DEFAULT_INCREMENT),

    // @Param: 6_INC
    // @DisplayName: Increment step for actuator 6
    // @Description:  Initial increment step for changing the actuator's PWM
    // @Units: us
    // @User: Standard
    AP_GROUPINFO("6_INC", 6, Actuators, Actuators::actuator_increment_step[5], ACTUATOR_DEFAULT_INCREMENT),

    AP_GROUPEND
};

/**
 * @brief Constructor for Actuators class
 * 
 * @details Initializes the Actuators object and sets up parameter defaults.
 *          Calls AP_Param::setup_object_defaults to load parameter values
 *          from storage or apply defaults if parameters are not yet configured.
 */
Actuators::Actuators() {
    AP_Param::setup_object_defaults(this, var_info);
}


/**
 * @brief Initialize all configured actuator channels to their trim positions
 * 
 * @details Performs initial setup of the actuator subsystem during vehicle initialization.
 *          For each configured actuator channel (k_actuator1 through k_actuator6):
 *          
 *          1. Checks if the actuator function is assigned to a servo output channel
 *          2. Retrieves the servo channel configuration (min, max, trim values)
 *          3. Calculates the servo output range (max - min)
 *          4. Sets the normalized actuator value to correspond to the trim position
 *          5. Calls update_actuators() to apply initial PWM outputs
 *          
 *          The trim position is converted to a normalized value (0.0-1.0) where:
 *          - 0.0 represents servo_min (typically 1000μs)
 *          - 1.0 represents servo_max (typically 2000μs)
 *          - Trim position maps to a value between 0.0-1.0 proportionally
 *          
 *          This initialization ensures actuators start in a known, safe position
 *          (typically center/neutral for most auxiliary devices).
 * 
 * @note Called during Sub::init_ardupilot() vehicle initialization sequence
 * @note Only processes actuators that have been assigned to output channels
 * @warning Must be called after SRV_Channels system is initialized
 * 
 * @see update_actuators() for PWM output application
 * @see SRV_Channel::get_trim() for trim position configuration
 */
void Actuators::initialize_actuators() {
    // Start with first auxiliary actuator function (k_actuator1)
    const SRV_Channel::Function first_aux = SRV_Channel::Function::k_actuator1;
    
    // Initialize each of the ACTUATOR_CHANNELS (typically 6) auxiliary actuators
    for (int i = 0; i < ACTUATOR_CHANNELS; i++) {
        uint8_t channel_number;
        
        // Check if this actuator function is assigned to an output channel
        // Skip this actuator if not configured (allows sparse actuator assignment)
        if (!SRV_Channels::find_channel((SRV_Channel::Function)(first_aux + i), channel_number)) {
            continue;
        }
        
        // Get pointer to the servo channel object for this actuator
        SRV_Channel* chan = SRV_Channels::srv_channel(channel_number);
        
        // Retrieve configured PWM range for this servo channel
        uint16_t servo_min = chan->get_output_min();  // Minimum PWM value (typically 1000μs)
        uint16_t servo_max = chan->get_output_max();  // Maximum PWM value (typically 2000μs)
        uint16_t servo_range = servo_max - servo_min; // Total PWM range (typically 1000μs)

        // Get the configured trim (neutral) position for this servo
        uint16_t servo_trim = chan->get_trim();
        
        // Convert trim PWM position to normalized value (0.0-1.0)
        // aux_actuator_value array stores normalized positions for all actuators
        // Formula: (trim - min) / range gives position as fraction of full range
        aux_actuator_value[i] = (servo_trim - servo_min) / static_cast<float>(servo_range);
    }
    
    // Apply initial actuator values to servo outputs
    update_actuators();
}

/**
 * @brief Update PWM outputs for all configured actuator channels
 * 
 * @details Converts normalized actuator values (0.0-1.0) to PWM pulse widths
 *          and applies them to the corresponding servo output channels.
 *          This function is called each main loop iteration to keep auxiliary
 *          actuator outputs synchronized with their commanded values.
 *          
 *          For each configured actuator:
 *          1. Looks up the assigned servo output channel
 *          2. Retrieves the channel's PWM range configuration (min/max)
 *          3. Converts normalized value to PWM: pwm = min + (range × value)
 *          4. Sets the output PWM on the servo channel
 *          
 *          The normalized value (aux_actuator_value[i]) represents position:
 *          - 0.0 → servo_min (typically 1000μs, minimum position)
 *          - 0.5 → mid-range (typically 1500μs, center position)
 *          - 1.0 → servo_max (typically 2000μs, maximum position)
 *          
 *          Actuators without assigned output channels are silently skipped,
 *          allowing partial actuator configuration.
 * 
 * @note Called at main loop rate (typically 50Hz for ArduSub)
 * @note Only updates channels that have actuator functions assigned
 * @warning Ensure aux_actuator_value elements stay within 0.0-1.0 range
 * 
 * @see initialize_actuators() for initial value setup
 * @see increase_actuator(), decrease_actuator() for value modification
 * @see SRV_Channel::set_output_pwm() for PWM output application
 */
void Actuators::update_actuators() {
    // Start with first auxiliary actuator function (k_actuator1)
    const SRV_Channel::Function first_aux = SRV_Channel::Function::k_actuator1;
    
    // Update each configured actuator channel
    for (int i = 0; i < ACTUATOR_CHANNELS; i++) {
        uint8_t channel_number;
        
        // Check if this actuator function is assigned to an output channel
        // Skip if not configured (allows sparse actuator configuration)
        if (!SRV_Channels::find_channel((SRV_Channel::Function)(first_aux + i), channel_number)) {
            continue;
        }
        
        // Get pointer to the servo channel object
        SRV_Channel* chan = SRV_Channels::srv_channel(channel_number);
        
        // Retrieve configured PWM range for this channel
        uint16_t servo_min = chan->get_output_min();  // Minimum PWM (μs)
        uint16_t servo_max = chan->get_output_max();  // Maximum PWM (μs)
        uint16_t servo_range = servo_max - servo_min; // Total range (μs)
        
        // Convert normalized value (0.0-1.0) to PWM pulse width (μs)
        // Formula: pwm = min + (range × normalized_value)
        // Example: min=1000, range=1000, value=0.75 → pwm=1750μs
        chan->set_output_pwm(servo_min + servo_range * aux_actuator_value[i]);
    }
}

/**
 * @brief Increase an actuator's position by its configured increment step
 * 
 * @details Increments the normalized actuator value by the configured step size
 *          (ACTUATORS_N_INC parameter). The result is constrained to valid range
 *          [0.0, 1.0] to prevent out-of-bounds values.
 *          
 *          This function provides incremental manual control of auxiliary actuators,
 *          typically triggered by joystick button presses or GCS commands.
 *          
 *          The increment step is configurable per-actuator via parameters, allowing
 *          fine control (small steps) or coarse control (large steps) as appropriate
 *          for each device (e.g., lights might use large steps, camera tilt small steps).
 * 
 * @param[in] actuator_num Actuator index (0 to ACTUATOR_CHANNELS-1)
 * 
 * @note Changes take effect on next update_actuators() call
 * @note Out-of-range actuator numbers are silently ignored for safety
 * @warning Actuator must be configured with assigned output channel to have effect
 * 
 * @see decrease_actuator() for opposite direction
 * @see actuator_increment_step[] parameter array for step configuration
 * @see ACTUATOR_DEFAULT_INCREMENT for default step size
 */
void Actuators::increase_actuator(uint8_t actuator_num) {
    // Validate actuator number is within valid range
    // Silently return if invalid to prevent array overflow
    if (actuator_num >= ACTUATOR_CHANNELS) {
        return;
    }
    
    // Increment normalized value by configured step, constrain to [0.0, 1.0]
    // actuator_increment_step is normalized (0.0-1.0), not PWM microseconds
    aux_actuator_value[actuator_num] = constrain_float(
        aux_actuator_value[actuator_num] + actuator_increment_step[actuator_num],
        0.0f, 1.0f
    );
}

/**
 * @brief Decrease an actuator's position by its configured increment step
 * 
 * @details Decrements the normalized actuator value by the configured step size
 *          (ACTUATORS_N_INC parameter). The result is constrained to valid range
 *          [0.0, 1.0] to prevent out-of-bounds values.
 *          
 *          This function provides incremental manual control in the reverse direction,
 *          typically triggered by joystick button presses or GCS commands.
 *          
 *          Common use cases:
 *          - Dimming lights (decrease brightness)
 *          - Tilting camera down (decrease tilt angle)
 *          - Closing gripper (decrease opening)
 *          - Retracting mechanism (decrease extension)
 * 
 * @param[in] actuator_num Actuator index (0 to ACTUATOR_CHANNELS-1)
 * 
 * @note Changes take effect on next update_actuators() call
 * @note Out-of-range actuator numbers are silently ignored for safety
 * @warning Actuator must be configured with assigned output channel to have effect
 * 
 * @see increase_actuator() for opposite direction
 * @see actuator_increment_step[] parameter array for step configuration
 */
void Actuators::decrease_actuator(uint8_t actuator_num) {
    // Validate actuator number is within valid range
    // Silently return if invalid to prevent array overflow
    if (actuator_num >= ACTUATOR_CHANNELS) {
        return;
    }
    
    // Decrement normalized value by configured step, constrain to [0.0, 1.0]
    // actuator_increment_step is normalized (0.0-1.0), not PWM microseconds
    aux_actuator_value[actuator_num] = constrain_float(
        aux_actuator_value[actuator_num] - actuator_increment_step[actuator_num],
        0.0f, 1.0f
    );
}

/**
 * @brief Set actuator to minimum position
 * 
 * @details Sets the normalized actuator value to 0.0, which corresponds to the
 *          servo channel's minimum PWM output (typically 1000μs).
 *          
 *          This provides direct "snap to minimum" control, useful for:
 *          - Turning lights fully off
 *          - Retracting mechanisms completely
 *          - Opening grippers fully
 *          - Setting camera to minimum tilt angle
 *          
 *          Unlike incremental decrease, this immediately jumps to minimum position
 *          regardless of current value.
 * 
 * @param[in] actuator_num Actuator index (0 to ACTUATOR_CHANNELS-1)
 * 
 * @note Changes take effect on next update_actuators() call
 * @note Out-of-range actuator numbers are silently ignored for safety
 * 
 * @see max_actuator() for setting to maximum position
 * @see center_actuator() for setting to center position
 * @see min_toggle_actuator() for toggle between min and center
 */
void Actuators::min_actuator(uint8_t actuator_num) {
    // Validate actuator number is within valid range
    if (actuator_num >= ACTUATOR_CHANNELS) {
        return;
    }
    
    // Set to minimum position (0.0 → servo_min PWM)
    aux_actuator_value[actuator_num] = 0;
}

/**
 * @brief Set actuator to maximum position
 * 
 * @details Sets the normalized actuator value to 1.0, which corresponds to the
 *          servo channel's maximum PWM output (typically 2000μs).
 *          
 *          This provides direct "snap to maximum" control, useful for:
 *          - Turning lights fully on
 *          - Extending mechanisms completely
 *          - Closing grippers fully
 *          - Setting camera to maximum tilt angle
 *          
 *          Unlike incremental increase, this immediately jumps to maximum position
 *          regardless of current value.
 * 
 * @param[in] actuator_num Actuator index (0 to ACTUATOR_CHANNELS-1)
 * 
 * @note Changes take effect on next update_actuators() call
 * @note Out-of-range actuator numbers are silently ignored for safety
 * 
 * @see min_actuator() for setting to minimum position
 * @see center_actuator() for setting to center position
 * @see max_toggle_actuator() for toggle between max and center
 */
void Actuators::max_actuator(uint8_t actuator_num) {
    // Validate actuator number is within valid range
    if (actuator_num >= ACTUATOR_CHANNELS) {
        return;
    }
    
    // Set to maximum position (1.0 → servo_max PWM)
    aux_actuator_value[actuator_num] = 1;
}

/**
 * @brief Toggle actuator between minimum and center positions
 * 
 * @details Provides two-state toggle functionality with minimum and center positions.
 *          The behavior depends on current actuator position:
 *          
 *          - If current value < 0.4 (near minimum): Set to 0.5 (center)
 *          - If current value >= 0.4 (at or above center): Set to 0.0 (minimum)
 *          
 *          This creates a toggle with hysteresis to prevent rapid switching.
 *          The 0.4 threshold provides a dead zone ensuring clean state transitions.
 *          
 *          Common use cases:
 *          - Lights: Off (0.0) ↔ Half brightness (0.5)
 *          - Gripper: Open (0.0) ↔ Partially closed (0.5)
 *          - Camera: Down angle (0.0) ↔ Forward (0.5)
 *          
 *          Useful when full range is not needed and operator wants simple
 *          two-position control with a single button.
 * 
 * @param[in] actuator_num Actuator index (0 to ACTUATOR_CHANNELS-1)
 * 
 * @note Changes take effect on next update_actuators() call
 * @note Threshold at 0.4 creates hysteresis for stable toggling
 * @note Out-of-range actuator numbers are silently ignored for safety
 * 
 * @see max_toggle_actuator() for toggle between center and maximum
 * @see center_actuator() for setting to center without toggle logic
 * @see min_actuator() for direct minimum positioning
 */
void Actuators::min_toggle_actuator(uint8_t actuator_num) {
    // Validate actuator number is within valid range
    if (actuator_num >= ACTUATOR_CHANNELS) {
        return;
    }
    
    // Toggle between minimum (0.0) and center (0.5) with hysteresis
    // Threshold at 0.4 ensures clean state transitions without bouncing
    if (aux_actuator_value[actuator_num] < 0.4) {
        // Currently at or near minimum → move to center
        aux_actuator_value[actuator_num] = 0.5;
    } else {
        // Currently at or above threshold → move to minimum
        aux_actuator_value[actuator_num] = 0;
    }
}

/**
 * @brief Toggle actuator between center and maximum positions
 * 
 * @details Provides two-state toggle functionality with center and maximum positions.
 *          The behavior depends on current actuator position:
 *          
 *          - If current value >= 0.6 (at or near maximum): Set to 0.5 (center)
 *          - If current value < 0.6 (below maximum): Set to 1.0 (maximum)
 *          
 *          This creates a toggle with hysteresis to prevent rapid switching.
 *          The 0.6 threshold provides a dead zone ensuring clean state transitions.
 *          
 *          Common use cases:
 *          - Lights: Half brightness (0.5) ↔ Full brightness (1.0)
 *          - Gripper: Partially closed (0.5) ↔ Fully closed (1.0)
 *          - Camera: Forward (0.5) ↔ Up angle (1.0)
 *          
 *          Useful when operator needs to toggle between neutral and maximum
 *          positions with a single button, avoiding the minimum position.
 * 
 * @param[in] actuator_num Actuator index (0 to ACTUATOR_CHANNELS-1)
 * 
 * @note Changes take effect on next update_actuators() call
 * @note Threshold at 0.6 creates hysteresis for stable toggling
 * @note Out-of-range actuator numbers are silently ignored for safety
 * 
 * @see min_toggle_actuator() for toggle between minimum and center
 * @see center_actuator() for setting to center without toggle logic
 * @see max_actuator() for direct maximum positioning
 */
void Actuators::max_toggle_actuator(uint8_t actuator_num) {
    // Validate actuator number is within valid range
    if (actuator_num >= ACTUATOR_CHANNELS) {
        return;
    }
    
    // Toggle between center (0.5) and maximum (1.0) with hysteresis
    // Threshold at 0.6 ensures clean state transitions without bouncing
    if (aux_actuator_value[actuator_num] >= 0.6) {
        // Currently at or near maximum → move to center
        aux_actuator_value[actuator_num] = 0.5;
    } else {
        // Currently below threshold → move to maximum
        aux_actuator_value[actuator_num] = 1;
    }
}

/**
 * @brief Set actuator to center (neutral) position
 * 
 * @details Sets the normalized actuator value to 0.5, which corresponds to the
 *          midpoint of the servo channel's PWM range (typically 1500μs).
 *          
 *          This provides direct "snap to center" control, useful for:
 *          - Setting lights to half brightness
 *          - Positioning camera at forward-looking angle
 *          - Setting mechanism to neutral position
 *          - Returning to default/safe position
 *          
 *          The center position (0.5) typically corresponds to the servo's trim
 *          setting, which is often the mechanically neutral or safe position
 *          for most auxiliary devices.
 *          
 *          Unlike toggle functions, this always sets to exactly 0.5 regardless
 *          of current position.
 * 
 * @param[in] actuator_num Actuator index (0 to ACTUATOR_CHANNELS-1)
 * 
 * @note Changes take effect on next update_actuators() call
 * @note Out-of-range actuator numbers are silently ignored for safety
 * @note Center position (0.5) typically matches the servo trim configuration
 * 
 * @see min_actuator() for setting to minimum position
 * @see max_actuator() for setting to maximum position
 * @see min_toggle_actuator() for toggle with center as one state
 * @see max_toggle_actuator() for toggle with center as one state
 */
void Actuators::center_actuator(uint8_t actuator_num) {
    // Validate actuator number is within valid range
    if (actuator_num >= ACTUATOR_CHANNELS) {
        return;
    }
    
    // Set to center position (0.5 → midpoint PWM, typically 1500μs)
    aux_actuator_value[actuator_num] = 0.5;
}
