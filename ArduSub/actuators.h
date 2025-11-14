/**
 * @file actuators.h
 * @brief Actuator control interface for ArduSub auxiliary outputs
 * 
 * @details This header defines the Actuators class which manages up to 6 auxiliary
 *          actuator channels for submarine/ROV operations. These actuators provide
 *          control for additional servos or outputs beyond the primary motor/thruster
 *          channels, such as grippers, lights, camera servos, or other peripherals.
 *          
 *          Each actuator can be controlled through MAVLink commands or joystick buttons
 *          and supports configurable increment steps, speed limits, and position ranges.
 * 
 * @note This is part of the ArduSub vehicle implementation
 * @warning Actuator outputs directly control hardware - incorrect configuration could
 *          damage connected devices
 */

#pragma once

/**
 * @def ACTUATOR_CHANNELS
 * @brief Maximum number of auxiliary actuator channels supported
 * 
 * Defines the array size for actuator parameters and state variables.
 * ArduSub supports up to 6 independent actuator channels that can be
 * mapped to servo outputs for ROV/submarine auxiliary functions.
 */
#define ACTUATOR_CHANNELS 6

/**
 * @class Actuators
 * @brief Manages auxiliary actuator outputs for ArduSub vehicle
 * 
 * @details The Actuators class provides an interface for controlling auxiliary servo
 *          outputs on a submarine/ROV beyond the main thruster channels. Each actuator
 *          channel maintains its own position, increment step, and speed parameters.
 *          
 *          Actuators can be controlled via:
 *          - MAVLink commands from ground control station
 *          - Joystick button mappings
 *          - Mission commands
 *          
 *          Each actuator supports multiple control modes:
 *          - Increment/decrement by configured step size
 *          - Jump to minimum or maximum position
 *          - Toggle between min/max positions
 *          - Center to neutral position
 *          
 *          Position values are internally tracked as floats and then converted to
 *          PWM output values for the servo channels.
 * 
 * @note Parameters are stored via AP_Param system for persistent configuration
 * @see SRV_Channel for underlying servo output implementation
 */
class Actuators
{
public:
    /**
     * @brief AP_Param parameter table for actuator configuration
     * 
     * Defines the parameter structure for actuator settings that are stored
     * in EEPROM and configurable via ground control station. Includes increment
     * step sizes for each actuator channel.
     */
    static const struct AP_Param::GroupInfo var_info[];

    /**
     * @brief Constructor for Actuators class
     * 
     * Initializes actuator state variables and parameter defaults.
     * Actual hardware initialization occurs in initialize_actuators().
     */
    Actuators();
    
    /**
     * @brief Initialize actuator channels and load saved positions
     * 
     * @details Performs one-time initialization of actuator subsystem:
     *          - Loads saved actuator positions from storage
     *          - Initializes servo output channels
     *          - Sets default increment steps if not configured
     *          
     *          Called during vehicle initialization before arming.
     */
    void initialize_actuators();
    
    /**
     * @brief Update actuator servo outputs based on current position values
     * 
     * @details Called periodically (typically at main loop rate) to:
     *          - Apply speed limiting to position changes
     *          - Convert internal position values to PWM outputs
     *          - Write updated values to servo channels
     *          
     *          This handles smooth transitions when actuator positions are changed.
     * 
     * @note Called from main vehicle update loop
     */
    void update_actuators();
    
    /**
     * @brief Increase actuator position by configured increment step
     * 
     * @details Increases the target position of the specified actuator by the
     *          increment step defined in parameters. Position is clamped to
     *          maximum allowed value. Actual servo movement occurs gradually
     *          in update_actuators() based on configured speed.
     * 
     * @param[in] actuator_num Actuator channel number (0 to ACTUATOR_CHANNELS-1)
     * 
     * @warning actuator_num must be valid (< ACTUATOR_CHANNELS) - no bounds checking
     */
    void increase_actuator(uint8_t actuator_num);
    
    /**
     * @brief Decrease actuator position by configured increment step
     * 
     * @details Decreases the target position of the specified actuator by the
     *          increment step defined in parameters. Position is clamped to
     *          minimum allowed value. Actual servo movement occurs gradually
     *          in update_actuators() based on configured speed.
     * 
     * @param[in] actuator_num Actuator channel number (0 to ACTUATOR_CHANNELS-1)
     * 
     * @warning actuator_num must be valid (< ACTUATOR_CHANNELS) - no bounds checking
     */
    void decrease_actuator(uint8_t actuator_num);
    
    /**
     * @brief Set actuator to minimum position
     * 
     * @details Immediately sets the target position to the minimum value defined
     *          by servo output configuration. Actual servo movement occurs gradually
     *          in update_actuators() based on configured speed.
     * 
     * @param[in] actuator_num Actuator channel number (0 to ACTUATOR_CHANNELS-1)
     * 
     * @warning actuator_num must be valid (< ACTUATOR_CHANNELS) - no bounds checking
     */
    void min_actuator(uint8_t actuator_num);
    
    /**
     * @brief Set actuator to maximum position
     * 
     * @details Immediately sets the target position to the maximum value defined
     *          by servo output configuration. Actual servo movement occurs gradually
     *          in update_actuators() based on configured speed.
     * 
     * @param[in] actuator_num Actuator channel number (0 to ACTUATOR_CHANNELS-1)
     * 
     * @warning actuator_num must be valid (< ACTUATOR_CHANNELS) - no bounds checking
     */
    void max_actuator(uint8_t actuator_num);
    
    /**
     * @brief Toggle actuator between current position and minimum
     * 
     * @details If actuator is not at minimum position, moves to minimum.
     *          If already at minimum, moves to previous position. Useful for
     *          open/close operations like grippers or doors.
     * 
     * @param[in] actuator_num Actuator channel number (0 to ACTUATOR_CHANNELS-1)
     * 
     * @warning actuator_num must be valid (< ACTUATOR_CHANNELS) - no bounds checking
     */
    void min_toggle_actuator(uint8_t actuator_num);
    
    /**
     * @brief Toggle actuator between current position and maximum
     * 
     * @details If actuator is not at maximum position, moves to maximum.
     *          If already at maximum, moves to previous position. Useful for
     *          open/close operations like grippers or doors.
     * 
     * @param[in] actuator_num Actuator channel number (0 to ACTUATOR_CHANNELS-1)
     * 
     * @warning actuator_num must be valid (< ACTUATOR_CHANNELS) - no bounds checking
     */
    void max_toggle_actuator(uint8_t actuator_num);
    
    /**
     * @brief Set actuator to center/neutral position
     * 
     * @details Sets the target position to the center point between minimum and
     *          maximum values. Useful for centering camera servos or returning
     *          control surfaces to neutral. Actual servo movement occurs gradually
     *          in update_actuators() based on configured speed.
     * 
     * @param[in] actuator_num Actuator channel number (0 to ACTUATOR_CHANNELS-1)
     * 
     * @warning actuator_num must be valid (< ACTUATOR_CHANNELS) - no bounds checking
     */
    void center_actuator(uint8_t actuator_num);

protected:
    /**
     * @brief Configurable increment step size for each actuator channel
     * 
     * Defines how much the actuator position changes per increase_actuator()
     * or decrease_actuator() call. Stored as AP_Float parameters so they can
     * be configured via ground control station and persisted in EEPROM.
     * 
     * Units: Normalized position value (typically 0.0 to 1.0 range)
     */
    AP_Float actuator_increment_step[ACTUATOR_CHANNELS];
    
    /**
     * @brief Speed limit for actuator position changes (units per second)
     * 
     * Controls how quickly actuators move to new target positions. Prevents
     * abrupt servo movements that could damage mechanisms or create jerky motion.
     * Applied in update_actuators() to smooth transitions.
     * 
     * Units: Normalized position change per second
     * 
     * @note Set to 0.0 for no speed limiting (instant movement)
     */
    float aux_actuator_change_speed[ACTUATOR_CHANNELS];
    
    /**
     * @brief Current target position value for each actuator channel
     * 
     * Internal state tracking the desired position for each actuator.
     * Values are normalized (typically 0.0 to 1.0) then mapped to PWM
     * output range by update_actuators(). Position changes are applied
     * gradually based on aux_actuator_change_speed.
     * 
     * Units: Normalized position (0.0 = minimum, 1.0 = maximum)
     * 
     * @note This is the target position; actual servo position may lag
     *       due to speed limiting
     */
    float aux_actuator_value[ACTUATOR_CHANNELS];
    
public:
   
};
