#pragma once

/**
 * @file RC_Channel_Blimp.h
 * @brief RC channel handling for Blimp (lighter-than-air vehicle)
 * 
 * @details This file defines blimp-specific RC (Radio Control) channel classes
 *          that extend the base RC_Channel and RC_Channels classes. These classes
 *          handle RC input processing, auxiliary function mapping, mode switching,
 *          and failsafe detection tailored for lighter-than-air blimp vehicles.
 *          
 *          The blimp uses a unique control paradigm with vectored fins rather than
 *          traditional multicopter motors or fixed-wing control surfaces, requiring
 *          specialized RC input interpretation.
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#include <RC_Channel/RC_Channel.h>
#include "Fins.h"
#include "mode.h" //this includes Blimp.h which includes Fins.h

/**
 * @class RC_Channel_Blimp
 * @brief Blimp-specific RC channel implementation
 * 
 * @details This class extends the base RC_Channel class to provide blimp-specific
 *          handling of individual RC channels. It specializes auxiliary function
 *          processing and mode switching behavior for lighter-than-air vehicle control.
 *          
 *          Key responsibilities:
 *          - Initialize and process auxiliary function switches (arm/disarm, mode changes, etc.)
 *          - Handle mode switch position changes for flight mode selection
 *          - Manage blimp-specific auxiliary functions (air mode, stabilization options)
 *          - Provide safe mode transition logic for buoyant vehicle characteristics
 *          
 *          Thread Safety: Called from main loop thread, not thread-safe
 *          
 * @note Blimp auxiliary functions differ from multicopter/plane due to unique buoyancy
 *       and fin-based control characteristics
 */
class RC_Channel_Blimp : public RC_Channel
{

public:

protected:

    /**
     * @brief Initialize an auxiliary function on this RC channel
     * 
     * @details This method is called during RC channel initialization to set up
     *          auxiliary functions (switches, knobs, buttons) assigned to this channel.
     *          For blimp, handles special cases like air mode toggles and stabilization
     *          options that are unique to lighter-than-air vehicle control.
     * 
     * @param[in] ch_option The auxiliary function to initialize (arm, mode change, etc.)
     * @param[in] AuxSwitchPos Current switch position at initialization
     * 
     * @note Called during vehicle startup and when auxiliary function assignments change
     * @see do_aux_function()
     */
    void init_aux_function(AUX_FUNC ch_option, AuxSwitchPos) override;
    
    /**
     * @brief Execute an auxiliary function based on trigger event
     * 
     * @details Processes auxiliary function triggers (switch changes, button presses)
     *          for blimp-specific functions. Routes to specialized handlers for mode
     *          changes, air mode, and other blimp auxiliary functions. Validates that
     *          requested functions are safe for current vehicle state.
     * 
     * @param[in] trigger Auxiliary function trigger containing function ID, switch position,
     *                    and activation type (switch change, momentary button, etc.)
     * 
     * @return true if function was executed successfully, false if function invalid or unsafe
     * 
     * @note Called at main loop rate when auxiliary switch positions change
     * @warning Some functions may be rejected based on arming state or flight mode
     * @see init_aux_function(), do_aux_function_change_mode()
     */
    bool do_aux_function(const AuxFuncTrigger &trigger) override;

private:

    /**
     * @brief Handle auxiliary function request to change flight mode
     * 
     * @details Processes mode change requests from auxiliary switches. Validates that
     *          the requested mode is available and safe for blimp operation. Performs
     *          mode transition with appropriate pre-flight checks for lighter-than-air
     *          vehicle characteristics (buoyancy compensation, fin initialization, etc.).
     * 
     * @param[in] mode The flight mode number to switch to (MANUAL, LOITER, LAND, etc.)
     * @param[in] ch_flag Switch position (LOW, MIDDLE, HIGH) triggering the mode change
     * 
     * @note Mode changes may be rejected if vehicle is not armed or mode prerequisites not met
     * @warning Switching modes during flight affects buoyancy control and fin authority
     * @see mode_switch_changed(), Mode::Number
     */
    void do_aux_function_change_mode(const Mode::Number mode,
                                     const AuxSwitchPos ch_flag);
    
    /**
     * @brief Handle auxiliary function request to change air mode
     * 
     * @details Processes air mode toggle requests from auxiliary switches. Air mode
     *          affects how the blimp responds to control inputs and manages buoyancy.
     *          This is specific to lighter-than-air vehicles and controls fin mixing
     *          and stabilization aggressiveness.
     * 
     * @param[in] ch_flag Switch position (LOW=disable, MIDDLE=default, HIGH=enable)
     * 
     * @note Air mode changes affect vehicle responsiveness and should be used carefully
     * @warning Changing air mode during aggressive maneuvering may cause instability
     */
    void do_aux_function_change_air_mode(const AuxSwitchPos ch_flag);

    /**
     * @brief Handle mode switch position changes
     * 
     * @details Called when the dedicated flight mode selection switch changes position.
     *          Maps switch position to configured flight mode and initiates mode transition.
     *          For blimp, ensures mode transitions are compatible with current buoyancy
     *          state and fin configuration.
     * 
     * @param[in] new_pos New position of the mode switch (typically 6-position switch
     *                    mapped to PWM ranges)
     * 
     * @note Mode switch is typically mapped to RC channel 5 but configurable via parameters
     * @warning Mode switch changes are rejected if vehicle safety checks fail
     * @see do_aux_function_change_mode()
     */
    void mode_switch_changed(modeswitch_pos_t new_pos) override;

};

/**
 * @class RC_Channels_Blimp
 * @brief Multi-channel RC input manager for Blimp vehicle
 * 
 * @details This class extends RC_Channels to manage all RC input channels for the blimp,
 *          providing blimp-specific implementations for input validation, failsafe detection,
 *          and channel access. Coordinates between multiple RC_Channel_Blimp instances and
 *          provides vehicle-wide RC input health monitoring.
 *          
 *          Key responsibilities:
 *          - Validate RC input data is fresh and within expected ranges
 *          - Detect RC failsafe conditions (signal loss, low RSSI, corrupt data)
 *          - Provide access to individual RC channels with bounds checking
 *          - Identify arming channel and flight mode selection channel
 *          - Manage array of RC_Channel_Blimp objects (typically 16 channels)
 *          
 *          Coordinate Systems: RC inputs are typically:
 *          - Roll/Pitch: -4500 to 4500 centidegrees
 *          - Yaw: -4500 to 4500 centidegrees/s
 *          - Throttle: 0 to 1000 (scaled to buoyancy compensation)
 *          
 *          Thread Safety: Accessed from main loop thread and RC input thread with locking
 *          
 * @note Blimp RC failsafe behavior differs from multicopter due to buoyancy - vehicle
 *       may hold altitude naturally without active control
 * @warning RC failsafe triggers safety-critical behaviors - accurate detection essential
 */
class RC_Channels_Blimp : public RC_Channels
{
public:

    /**
     * @brief Check if RC input is valid and fresh
     * 
     * @details Validates that RC input data is being received, is recent (not stale),
     *          and passes basic sanity checks. For blimp, checks that critical channels
     *          (roll, pitch, yaw, throttle/buoyancy) are within expected ranges and
     *          updated within failsafe timeout period.
     * 
     * @return true if RC input is valid and can be used for control, false otherwise
     * 
     * @note Called at main loop rate to continuously monitor RC health
     * @warning If returns false, vehicle should enter RC failsafe mode
     * @see in_rc_failsafe()
     */
    bool has_valid_input() const override;
    
    /**
     * @brief Check if vehicle is currently in RC failsafe
     * 
     * @details Determines if RC failsafe condition is active due to signal loss,
     *          invalid data, or timeout. For blimp, failsafe triggers when no valid
     *          RC data received for configured timeout period (typically 1 second).
     *          
     *          RC failsafe for blimp may trigger:
     *          - RTL (Return to Launch) if position available
     *          - LAND mode for controlled descent
     *          - Hold position using buoyancy if no GPS
     * 
     * @return true if in RC failsafe, false if RC input is healthy
     * 
     * @note Failsafe state is latched and requires explicit recovery conditions
     * @warning RC failsafe is safety-critical - triggers autonomous emergency response
     * @see has_valid_input(), Blimp::failsafe_radio()
     */
    bool in_rc_failsafe() const override;

    /**
     * @brief Get the RC channel configured for arming/disarming
     * 
     * @details Returns pointer to the RC channel assigned as the arming switch
     *          (typically a 2 or 3-position switch used to arm/disarm motors/fins).
     *          For blimp, arming enables fin control - vehicle may float when disarmed.
     * 
     * @return Pointer to arming channel, or nullptr if no arming channel configured
     * 
     * @note Arming channel assignment is configurable via RC_MAP_ARM parameter
     * @see RC_Channel::AUX_FUNC::ARMDISARM
     */
    RC_Channel *get_arming_channel(void) const override;

    /**
     * @brief Array of RC channel objects for all RC inputs
     * 
     * @details Stores all RC_Channel_Blimp instances (typically 16 channels) that
     *          represent physical RC receiver channels. Indexed from 0 to NUM_RC_CHANNELS-1.
     *          
     *          Standard channel assignments for blimp:
     *          - Channel 0: Roll (right fin servo control)
     *          - Channel 1: Pitch (forward fin servo control)  
     *          - Channel 2: Throttle/Buoyancy compensation
     *          - Channel 3: Yaw (fin differential control)
     *          - Channel 4+: Auxiliary functions (mode, arm, options)
     * 
     * @note Actual channel functions are configurable via RCx_OPTION parameters
     */
    RC_Channel_Blimp obj_channels[NUM_RC_CHANNELS];
    
    /**
     * @brief Get pointer to specific RC channel by index
     * 
     * @details Returns pointer to RC channel at specified index with bounds checking.
     *          Provides safe access to individual RC_Channel_Blimp objects from the
     *          obj_channels array.
     * 
     * @param[in] chan Channel index (0 to NUM_RC_CHANNELS-1, typically 0-15)
     * 
     * @return Pointer to RC_Channel_Blimp at index, or nullptr if index out of bounds
     * 
     * @note Channel indexing is zero-based (channel 1 is index 0)
     * @see obj_channels
     */
    RC_Channel_Blimp *channel(const uint8_t chan) override
    {
        if (chan >= NUM_RC_CHANNELS) {
            return nullptr;
        }
        return &obj_channels[chan];
    }

protected:

    /**
     * @brief Get the RC channel number configured for flight mode selection
     * 
     * @details Returns the channel index (0-based) that is configured as the flight
     *          mode selection switch. For blimp, this switch typically selects between
     *          MANUAL, LOITER, LAND, and RTL modes. Returns -1 if no mode channel configured.
     * 
     * @return Channel index (0-15) of mode switch, or -1 if not configured
     * 
     * @note Mode channel is configured via FLTMODE_CH parameter (default channel 5)
     * @see RC_Channel_Blimp::mode_switch_changed()
     */
    int8_t flight_mode_channel_number() const override;

};
