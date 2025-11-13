/**
 * @file RC_Channel_Rover.h
 * @brief Rover-specific RC channel handling and auxiliary function mapping
 * 
 * @details This file defines rover-specific extensions to the RC_Channel and RC_Channels
 *          base classes, implementing auxiliary switch functions and RC input processing
 *          tailored for ground vehicle operations including differential steering,
 *          sailboat motor control, and rover-specific flight mode switching.
 * 
 * Source: Rover/RC_Channel_Rover.h
 */

#pragma once

#include <RC_Channel/RC_Channel.h>
#include "Rover.h"
#include "mode.h"

/**
 * @class RC_Channel_Rover
 * @brief Rover-specific RC channel implementation with ground vehicle auxiliary functions
 * 
 * @details Extends the base RC_Channel class to provide rover-specific auxiliary switch
 *          functions including:
 *          - Sailboat motor control (3-position switch for sail-only, motor-assist, motor-only)
 *          - Ground vehicle mode switching (Manual, Hold, Auto, Steering, Acro)
 *          - Waypoint addition at current location
 *          - Rover-specific arming and safety functions
 * 
 *          This class processes RC input from transmitters and maps auxiliary channels
 *          to rover control functions, handling switch position changes and function triggers.
 * 
 * @note Inherits common RC functionality from RC_Channel base class
 * @see RC_Channel for base channel functionality
 * 
 * Source: Rover/RC_Channel_Rover.h
 */
class RC_Channel_Rover : public RC_Channel
{

public:

protected:

    /**
     * @brief Initialize auxiliary function for a specific channel option
     * 
     * @details Called during system initialization to set up auxiliary channel functions.
     *          Configures rover-specific auxiliary switches and their initial states.
     * 
     * @param[in] ch_option    Auxiliary function type to initialize (e.g., SaveWP, LearnCruise)
     * @param[in] switch_pos   Initial switch position (LOW, MIDDLE, HIGH)
     * 
     * @note This is called once during boot for each configured auxiliary channel
     */
    void init_aux_function(AUX_FUNC ch_option, AuxSwitchPos) override;

    /**
     * @brief Process auxiliary function trigger from RC input
     * 
     * @details Handles auxiliary switch state changes and executes corresponding rover functions.
     *          Processes functions such as mode changes, waypoint saving, sailboat motor control,
     *          and other rover-specific auxiliary operations based on switch position changes.
     * 
     * @param[in] trigger  Auxiliary function trigger containing function ID and switch position
     * 
     * @return true if auxiliary function was handled successfully, false if function not recognized
     * 
     * @note Called at RC input rate (typically 50Hz) when auxiliary switch positions change
     * @warning Some auxiliary functions affect vehicle control immediately - ensure safe switch configuration
     * 
     * @see init_aux_function() for auxiliary function initialization
     */
    bool do_aux_function(const AuxFuncTrigger &trigger) override;

    /**
     * @brief Handle flight mode switch position change
     * 
     * @details Called when the RC mode switch changes position, triggering a mode transition
     *          to the corresponding rover flight mode (Manual, Hold, Auto, Steering, Acro, etc.).
     *          Validates the mode transition and updates vehicle state accordingly.
     * 
     * @param[in] new_pos  New mode switch position (typically 6-position switch)
     * 
     * @note Called automatically by RC input processing when mode switch position changes
     * @warning Mode changes may affect vehicle control immediately - ensure safe operating conditions
     * 
     * @see Rover::set_mode() for mode transition logic
     */
    void mode_switch_changed(modeswitch_pos_t new_pos) override;

private:

    /**
     * @brief Trigger auxiliary function mode change
     * 
     * @details Helper function to change rover flight mode via auxiliary switch.
     *          Used for auxiliary channels configured to trigger specific modes
     *          (e.g., dedicated RTL switch, Hold switch).
     * 
     * @param[in] mode     Target mode to switch to
     * @param[in] ch_flag  Auxiliary switch position triggering the change
     */
    void do_aux_function_change_mode(Mode &mode,
                                     const AuxSwitchPos ch_flag);

    /**
     * @brief Add waypoint at current vehicle location
     * 
     * @details Adds a new waypoint to the current mission at the rover's current position.
     *          Used by SaveWP auxiliary function to mark locations during manual driving
     *          for later autonomous navigation.
     * 
     * @note Waypoint is added to end of current mission in memory and saved to storage
     */
    void add_waypoint_for_current_loc();

    /**
     * @brief Handle 3-position sailboat motor control switch
     * 
     * @details Controls sailboat motor assistance based on 3-position switch:
     *          - LOW: Sail only (motor disabled)
     *          - MIDDLE: Motor assist (motor supplements sail)
     *          - HIGH: Motor only (sail may be reefed, full motor power)
     * 
     * @param[in] ch_flag  Switch position (LOW, MIDDLE, HIGH)
     * 
     * @note Specific to sailboat rover configurations
     * @see Sailboat for motor control implementation
     */
    void do_aux_function_sailboat_motor_3pos(const AuxSwitchPos ch_flag);
};

/**
 * @class RC_Channels_Rover
 * @brief Container for all RC channels with rover-specific behavior
 * 
 * @details Manages the complete set of RC input channels for the rover, providing
 *          rover-specific implementations of RC failsafe detection, input validation,
 *          and channel access. Contains array of RC_Channel_Rover objects for
 *          individual channel processing.
 * 
 *          Responsibilities:
 *          - RC failsafe state detection for ground vehicles
 *          - RC input validity checking (signal presence, timeout)
 *          - Flight mode channel identification and reading
 *          - Channel access and management
 * 
 * @note Inherits common RC channel management from RC_Channels base class
 * @see RC_Channels for base channel container functionality
 * @see RC_Channel_Rover for individual channel implementation
 * 
 * Source: Rover/RC_Channel_Rover.h
 */
class RC_Channels_Rover : public RC_Channels
{

public:

    /**
     * @brief Check if rover is in RC failsafe condition
     * 
     * @details Determines if RC input has been lost or is invalid, triggering failsafe behavior.
     *          Considers factors such as:
     *          - RC signal timeout (no valid frames received)
     *          - Throttle failsafe value detection
     *          - Rover-specific failsafe criteria
     * 
     * @return true if in RC failsafe (invalid or lost RC input), false if RC input valid
     * 
     * @note Called at main loop rate to monitor RC health
     * @warning RC failsafe triggers safety actions - ensure failsafe settings are configured correctly
     * 
     * @see Rover::failsafe_trigger() for failsafe action handling
     */
    bool in_rc_failsafe() const override;

    /**
     * @brief Check if RC input is currently valid
     * 
     * @details Validates that RC receiver is providing usable input data.
     *          Checks for signal presence, recent updates, and data validity.
     * 
     * @return true if RC input is valid and current, false if invalid or stale
     * 
     * @note Used to determine if RC control is available for manual modes
     */
    bool has_valid_input() const override;

    /**
     * @brief Get the RC channel configured for arming control
     * 
     * @details Returns pointer to RC channel configured for arming/disarming the rover
     *          (typically auxiliary switch configured for ARM/DISARM function).
     * 
     * @return Pointer to arming channel if configured, nullptr if no arming channel set
     * 
     * @note Arming channel is configured via RCx_OPTION parameter
     */
    RC_Channel *get_arming_channel(void) const override;

    /**
     * @brief Array of all RC channel objects for the rover
     * 
     * @details Contains NUM_RC_CHANNELS individual RC_Channel_Rover objects,
     *          each processing one RC input channel (typically 16 channels).
     */
    RC_Channel_Rover obj_channels[NUM_RC_CHANNELS];

    /**
     * @brief Get specific RC channel by number
     * 
     * @details Provides access to individual RC channel object for reading input
     *          values, configuring options, or checking channel state.
     * 
     * @param[in] chan  Channel number (0-based index, typically 0-15)
     * 
     * @return Pointer to RC_Channel_Rover object for specified channel, nullptr if invalid
     * 
     * @note Channel 0 = RC input 1, Channel 1 = RC input 2, etc.
     */
    RC_Channel_Rover *channel(const uint8_t chan) override {
        if (chan >= NUM_RC_CHANNELS) {
            return nullptr;
        }
        return &obj_channels[chan];
    }

private:

    /**
     * @brief Get flight mode channel number from parameters
     * 
     * @details Returns the RC channel number configured for flight mode selection
     *          (configured via FLTMODE_CH parameter, typically channel 5 or 8).
     * 
     * @return Channel number (1-based, e.g., 5 for RC input 5), -1 if not configured
     * 
     * @note Used internally to identify which RC channel controls mode switching
     */
    int8_t flight_mode_channel_number() const override;
};
