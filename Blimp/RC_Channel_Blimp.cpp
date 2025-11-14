/**
 * @file RC_Channel_Blimp.cpp
 * @brief RC channel integration and input processing for Blimp lighter-than-air vehicle
 * 
 * @details This file implements blimp-specific RC input handling, including:
 *          - Flight mode switching via RC transmitter mode switch
 *          - Auxiliary function mappings for blimp-specific features
 *          - RC failsafe detection and validation
 *          - Pilot input processing with appropriate scaling for lighter-than-air dynamics
 *          - Mode change requests from RC auxiliary switches (LOITER, MANUAL modes)
 *          
 *          The RC channel system provides the interface between pilot commands from
 *          the radio transmitter and the blimp flight control system. Blimp-specific
 *          considerations include gentle input response suitable for lighter-than-air
 *          vehicles with slower dynamics compared to multirotors.
 *          
 *          Coordinate Frame: Pilot inputs are in body frame (forward/right/up/yaw)
 *          Units: RC inputs are PWM values (typically 1000-2000 μs), scaled internally
 * 
 * @note This implementation inherits from the common RC_Channel framework and
 *       specializes behavior for blimp vehicle characteristics
 * 
 * @see RC_Channel.h for base class interface
 * @see Blimp.h for vehicle state and mode definitions
 * 
 * Source: Blimp/RC_Channel_Blimp.cpp
 */

#include "Blimp.h"

#include "RC_Channel_Blimp.h"


// defining these two macros and including the RC_Channels_VarInfo header defines the parameter information common to all vehicle types
#define RC_CHANNELS_SUBCLASS RC_Channels_Blimp
#define RC_CHANNEL_SUBCLASS RC_Channel_Blimp

#include <RC_Channel/RC_Channels_VarInfo.h>

/**
 * @brief Get the RC channel number used for flight mode switching
 * 
 * @details Returns the RC channel configured for flight mode selection via
 *          the FLTMODE_CH parameter. This channel's position determines which
 *          flight mode the blimp operates in (MANUAL, LOITER, LAND, etc.).
 *          
 *          The channel position is typically divided into 6 positions, each
 *          mapped to a different flight mode via FLTMODE1-6 parameters.
 * 
 * @return int8_t RC channel number (1-16), or 0 if not configured
 * 
 * @note This is called by the RC_Channel framework to determine which channel
 *       to monitor for mode switch changes
 * 
 * Source: Blimp/RC_Channel_Blimp.cpp:12-15
 */
int8_t RC_Channels_Blimp::flight_mode_channel_number() const
{
    return blimp.g.flight_mode_chan.get();
}

/**
 * @brief Handle flight mode switch position changes from RC transmitter
 * 
 * @details Called by the RC_Channel framework when the configured flight mode
 *          channel position changes. This function:
 *          1. Validates the new switch position is within valid range
 *          2. Looks up the flight mode configured for that switch position
 *          3. Requests mode change via blimp.set_mode()
 *          4. Provides user feedback via AP_Notify (tone/LED) on success or failure
 *          
 *          The switch position is mapped to a flight mode via the FLTMODE1-6
 *          parameters, allowing the pilot to configure which modes are available
 *          on their transmitter's mode switch.
 * 
 * @param[in] new_pos New mode switch position (0-5 for 6-position switch)
 * 
 * @note User notification (tone/LED) is suppressed during autopilot initialization
 *       to avoid spurious alerts during boot
 * 
 * @warning If the mode change fails (e.g., pre-arm checks fail, mode not available),
 *          the blimp remains in its current flight mode and failure is indicated
 *          to the pilot
 * 
 * @see Blimp::set_mode() for mode change logic and validation
 * 
 * Source: Blimp/RC_Channel_Blimp.cpp:17-37
 */
void RC_Channel_Blimp::mode_switch_changed(modeswitch_pos_t new_pos)
{
    if (new_pos < 0 || (uint8_t)new_pos > blimp.num_flight_modes) {
        // should not have been called
        return;
    }

    if (!blimp.set_mode((Mode::Number)blimp.flight_modes[new_pos].get(), ModeReason::RC_COMMAND)) {
        // alert user to mode change failure
        if (blimp.ap.initialised) {
            AP_Notify::events.user_mode_change_failed = 1;
        }
        return;
    }

    // play a tone
    // alert user to mode change (except if autopilot is just starting up)
    if (blimp.ap.initialised) {
        AP_Notify::events.user_mode_change = 1;
    }
}

/**
 * @brief Check if the blimp is currently in RC failsafe condition
 * 
 * @details Returns the RC radio failsafe state. The failsafe is triggered when:
 *          - No valid RC signal received for FS_TIMEOUT seconds
 *          - RC signal quality drops below acceptable threshold
 *          - Loss of communication with RC receiver
 *          
 *          When in RC failsafe, the blimp may trigger failsafe actions such as
 *          landing or maintaining current position, depending on FS_THR_ENABLE
 *          parameter configuration.
 * 
 * @return true if in RC failsafe (no valid RC input), false if RC is healthy
 * 
 * @note This is called by the RC_Channel framework for failsafe state checks
 * 
 * @warning Loss of RC link is a critical safety condition for lighter-than-air
 *          vehicles, as they may drift with wind when not under active control
 * 
 * @see Blimp::failsafe_radio_on_event() for failsafe trigger logic
 * @see has_valid_input() for more detailed RC input validation
 * 
 * Source: Blimp/RC_Channel_Blimp.cpp:39-42
 */
bool RC_Channels_Blimp::in_rc_failsafe() const
{
    return blimp.failsafe.radio;
}

/**
 * @brief Check if RC input is currently valid and usable for control
 * 
 * @details Performs comprehensive validation of RC input health by checking:
 *          1. RC failsafe state (blimp.failsafe.radio) - false if in failsafe
 *          2. Failsafe counter (blimp.failsafe.radio_counter) - must be zero
 *          
 *          The radio_counter tracks consecutive failed RC update cycles and
 *          provides hysteresis to prevent rapid failsafe toggling due to
 *          intermittent signal issues.
 *          
 *          This is stricter than in_rc_failsafe() as it also rejects inputs
 *          during the failsafe entry/exit transition period.
 * 
 * @return true if RC input is valid and safe to use for control
 * @return false if RC is in failsafe or transitioning to/from failsafe
 * 
 * @note Called by various flight control functions to determine if pilot
 *       input should be accepted or ignored
 * 
 * @warning Control algorithms should not process RC input when this returns
 *          false, as the input may be stale, corrupted, or missing
 * 
 * @see in_rc_failsafe() for simpler failsafe-only check
 * 
 * Source: Blimp/RC_Channel_Blimp.cpp:44-53
 */
bool RC_Channels_Blimp::has_valid_input() const
{
    if (blimp.failsafe.radio) {
        return false;
    }
    if (blimp.failsafe.radio_counter != 0) {
        return false;
    }
    return true;
}

/**
 * @brief Get the RC channel used for rudder arming/disarming
 * 
 * @details Returns the yaw channel which is used for the rudder arming
 *          procedure. To arm the blimp using RC:
 *          - Hold yaw stick to the right (high position) for 2+ seconds
 *          - All pre-arm checks must pass
 *          - Vehicle must be in a mode that allows arming
 *          
 *          For blimps, the yaw channel controls rotation around the vertical
 *          axis and is appropriate for the arming gesture.
 * 
 * @return RC_Channel* Pointer to the yaw channel used for arming, never nullptr
 * 
 * @note Alternative arming methods include GCS commands and auxiliary switches
 * @note The rudder arming feature can be disabled via ARMING_RUDDER parameter
 * 
 * @see AP_Arming::is_armed() for arming state
 * @see Blimp::init_rc_in() for channel initialization
 * 
 * Source: Blimp/RC_Channel_Blimp.cpp:55-58
 */
RC_Channel * RC_Channels_Blimp::get_arming_channel(void) const
{
    return blimp.channel_yaw;
}

/**
 * @brief Initialize an auxiliary function for a configured RC channel
 * 
 * @details Called during RC channel initialization to set up auxiliary switch
 *          functions (e.g., mode changes, feature toggles) mapped to RC channels
 *          via RCx_OPTION parameters.
 *          
 *          For blimp-specific auxiliary functions:
 *          - MANUAL mode switch: No initialization needed (stateless)
 *          - All other functions: Delegated to base RC_Channel class
 *          
 *          The initialization sets the initial state of features controlled by
 *          switches, ensuring the blimp starts in the correct configuration
 *          based on switch positions at boot.
 * 
 * @param[in] ch_option The auxiliary function type (from AUX_FUNC enum)
 * @param[in] ch_flag Current switch position (LOW/MIDDLE/HIGH)
 * 
 * @note This is called once during startup for each configured auxiliary function
 * @note Some functions require initialization (e.g., enabling features if switch
 *       is HIGH at boot), while others like mode switches are stateless
 * 
 * @see do_aux_function() for runtime auxiliary function handling
 * @see RC_Channel::init_aux_function() for base class implementation
 * 
 * Source: Blimp/RC_Channel_Blimp.cpp:60-72
 */
void RC_Channel_Blimp::init_aux_function(const AUX_FUNC ch_option, const AuxSwitchPos ch_flag)
{
    // init channel options
    switch (ch_option) {
    // the following functions do not need to be initialised:
    case AUX_FUNC::MANUAL:
        break;
    default:
        RC_Channel::init_aux_function(ch_option, ch_flag);
        break;
    }
}

/**
 * @brief Change flight mode based on an auxiliary switch position
 * 
 * @details Handles flight mode changes triggered by auxiliary RC channels
 *          (configured via RCx_OPTION parameters). This provides an alternative
 *          to the main flight mode channel for accessing specific modes.
 *          
 *          Behavior by switch position:
 *          - HIGH: Engage the specified mode (if possible)
 *          - MIDDLE/LOW: If currently in this mode, return to flight mode
 *            switch's selected mode (allows temporary mode activation)
 *          
 *          Common use case: Assign LOITER or MANUAL to a momentary switch
 *          for quick mode access, returning to main mode switch when released.
 *          
 *          User feedback via tones/LEDs indicates success or failure of mode
 *          change, helping pilot understand vehicle state.
 * 
 * @param[in] mode The flight mode to change to (MANUAL, LOITER, LAND, etc.)
 * @param[in] ch_flag Current auxiliary switch position (LOW/MIDDLE/HIGH)
 * 
 * @note Mode change may fail due to pre-arm checks, mode requirements, or
 *       vehicle state - the blimp remains in current mode on failure
 * 
 * @note User notification is suppressed during autopilot initialization to
 *       avoid alerts during boot sequence
 * 
 * @warning Rapid mode switching can be disorienting for lighter-than-air
 *          vehicles with slow dynamic response
 * 
 * @see do_aux_function() for auxiliary function dispatch
 * @see Blimp::set_mode() for mode change validation and execution
 * 
 * Source: Blimp/RC_Channel_Blimp.cpp:74-99
 */
void RC_Channel_Blimp::do_aux_function_change_mode(const Mode::Number mode,
        const AuxSwitchPos ch_flag)
{
    switch (ch_flag) {
    case AuxSwitchPos::HIGH: {
        // engage mode (if not possible we remain in current flight mode)
        const bool success = blimp.set_mode(mode, ModeReason::AUX_FUNCTION);
        if (blimp.ap.initialised) {
            if (success) {
                AP_Notify::events.user_mode_change = 1;
            } else {
                AP_Notify::events.user_mode_change_failed = 1;
            }
        }
        break;
    }
    default:
        // return to flight mode switch's flight mode if we are currently
        // in this mode
        if (blimp.control_mode == mode) {
            rc().reset_mode_switch();
        }
    }
}

/**
 * @brief Execute the auxiliary function assigned to an RC channel
 * 
 * @details Central dispatcher for auxiliary switch functions on blimp. Called
 *          by the RC_Channel framework when an auxiliary channel changes state.
 *          
 *          Blimp-specific auxiliary functions:
 *          - LOITER: Switch to position hold mode (maintain lat/lon/alt)
 *          - MANUAL: Switch to manual control mode (direct pilot input)
 *          
 *          All other auxiliary functions (arming, lights, camera trigger, etc.)
 *          are handled by the base RC_Channel class, which provides common
 *          functionality shared across all vehicle types.
 *          
 *          The trigger contains both the function type (what the switch does)
 *          and the position (LOW/MIDDLE/HIGH), allowing functions to respond
 *          to switch transitions or act as momentary/toggle switches.
 * 
 * @param[in] trigger Auxiliary function trigger containing:
 *                    - func: The auxiliary function type (AUX_FUNC enum)
 *                    - pos: Current switch position (AuxSwitchPos enum)
 * 
 * @return true if function was handled successfully
 * @return false if function is not supported or execution failed
 * 
 * @note Called at RC input rate (typically 50Hz) when switch position changes
 * 
 * @note Auxiliary functions are configured via RCx_OPTION parameters, allowing
 *       users to customize their transmitter layout
 * 
 * @see init_aux_function() for auxiliary function initialization
 * @see do_aux_function_change_mode() for mode change implementation
 * @see RC_Channel::do_aux_function() for common auxiliary functions
 * 
 * Source: Blimp/RC_Channel_Blimp.cpp:101-121
 */
bool RC_Channel_Blimp::do_aux_function(const AuxFuncTrigger &trigger)
{
    const AUX_FUNC &ch_option = trigger.func;
    const AuxSwitchPos &ch_flag = trigger.pos;

    switch (ch_option) {

    case AUX_FUNC::LOITER:
        do_aux_function_change_mode(Mode::Number::LOITER, ch_flag);
        break;

    case AUX_FUNC::MANUAL:
        do_aux_function_change_mode(Mode::Number::MANUAL, ch_flag);
        break;

    default:
        return RC_Channel::do_aux_function(trigger);
    }
    return true;
}
