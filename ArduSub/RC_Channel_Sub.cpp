/**
 * @file RC_Channel_Sub.cpp
 * @brief ArduSub-specific RC_Channel and RC_Channels subclass implementation
 * 
 * @details This file implements underwater vehicle-specific RC channel handling,
 *          extending the base RC_Channel and RC_Channels classes with Sub-specific
 *          behavior for:
 *          - Flight mode switching adapted for underwater operations
 *          - Auxiliary function handling for sub-specific features
 *          - RC failsafe detection tailored to underwater environments
 *          - Input validation considering underwater communication constraints
 *          - Throttle arming checks for center-sprung/reversing throttle configurations
 * 
 *          ArduSub uses RC channels for manual control of thrusters, cameras, lights,
 *          and mode switching. The implementation accounts for underwater-specific
 *          requirements such as bidirectional thrust control and joystick-style
 *          input handling.
 * 
 * @note RC failsafe behavior in ArduSub is adapted for underwater operations where
 *       loss of RC signal may indicate loss of tether or surface communication.
 * 
 * @warning Throttle configuration (thr_arming_position) must be properly set for
 *          the specific control input device (joystick vs traditional RC transmitter)
 *          to ensure correct arming behavior.
 * 
 * Source: ArduSub/RC_Channel_Sub.cpp:1-78
 */

#include "Sub.h"

#include "RC_Channel_Sub.h"
#include "config.h"

// defining these two macros and including the RC_Channels_VarInfo
// header defines the parameter information common to all vehicle
// types
#define RC_CHANNELS_SUBCLASS RC_Channels_Sub
#define RC_CHANNEL_SUBCLASS RC_Channel_Sub

#include <RC_Channel/RC_Channels_VarInfo.h>


#if AP_SUB_RC_ENABLED
/**
 * @brief Get the RC channel number used for flight mode selection
 * 
 * @return int8_t Channel number (1-based) configured for flight mode switching
 * 
 * @note This is used by the RC_Channels framework to identify which channel
 *       monitors mode switch position changes
 */
int8_t RC_Channels_Sub::flight_mode_channel_number() const
{
    return sub.g.flight_mode_chan.get();
}

/**
 * @brief Handle flight mode switch position changes for underwater vehicle
 * 
 * @details Called when the configured flight mode RC channel changes position.
 *          Validates the switch position and attempts to transition to the
 *          corresponding flight mode configured for that position.
 * 
 *          ArduSub supports up to 6 flight mode positions (0-5), each mapped
 *          to a specific mode (Manual, Stabilize, Depth Hold, etc.) via the
 *          FLTMODE1-FLTMODE6 parameters.
 * 
 * @param[in] new_pos New switch position (0-5 for valid positions, 6 positions total)
 * 
 * @note If mode transition fails (e.g., pre-arm checks not met), the vehicle
 *       remains in its current mode without notification
 * 
 * @see Sub::set_mode()
 * @see RC_Channel::mode_switch_changed()
 * 
 * Source: ArduSub/RC_Channel_Sub.cpp:21-31
 */
void RC_Channel_Sub::mode_switch_changed(modeswitch_pos_t new_pos)
{
    // Validate switch position is within expected range (0-5 for 6-position switch)
    if (new_pos < 0 || new_pos > 6) {
        // should not have been called
        return;
    }

    // Attempt to set the mode corresponding to this switch position
    // sub.flight_modes[] array maps switch positions to Mode::Number values
    if (!sub.set_mode((Mode::Number)sub.flight_modes[new_pos].get(), ModeReason::RC_COMMAND)) {
        return;
    }
}

/**
 * @brief Initialize auxiliary switch function for Sub-specific features
 * 
 * @details Sets up the initial state of an auxiliary function mapped to an RC channel.
 *          This is called during system initialization for each configured auxiliary
 *          function to establish the correct starting state based on the current
 *          switch position.
 * 
 *          ArduSub supports auxiliary functions for features such as:
 *          - Camera tilt control
 *          - Lights on/off and brightness
 *          - Gain adjustments
 *          - Relay control for manipulators
 *          - Servo outputs for grippers or tools
 * 
 * @param[in] ch_option The auxiliary function type (e.g., LIGHTS1, CAMERA_TILT)
 * @param[in] ch_flag Current switch position (LOW, MIDDLE, HIGH) at initialization
 * 
 * @note Currently delegates to base RC_Channel implementation as Sub does not
 *       require special initialization beyond the standard behavior
 * 
 * @see RC_Channel::init_aux_function()
 * @see do_aux_function()
 * 
 * Source: ArduSub/RC_Channel_Sub.cpp:34-37
 */
void RC_Channel_Sub::init_aux_function(const AUX_FUNC ch_option, const AuxSwitchPos ch_flag)
{
    RC_Channel::init_aux_function(ch_option, ch_flag);
}

/**
 * @brief Check if RC failsafe condition is active for underwater vehicle
 * 
 * @details Determines if the RC receiver has lost connection with the transmitter,
 *          indicating potential loss of surface communication or tether connection.
 *          
 *          In ArduSub, RC failsafe typically indicates:
 *          - Loss of topside control (joystick or RC transmitter)
 *          - Tether disconnection (for tethered ROVs)
 *          - Communication system failure
 * 
 * @return true if RC failsafe is active (no valid RC signal)
 * @return false if RC connection is healthy
 * 
 * @note RC failsafe in underwater vehicles may be less critical than in aircraft
 *       since the vehicle typically remains stationary or surfaces rather than
 *       falling
 * 
 * @warning In tethered operations, RC failsafe usually indicates catastrophic
 *          tether failure requiring immediate attention
 * 
 * @see has_valid_input()
 * @see Sub::failsafe_radio_check()
 * 
 * Source: ArduSub/RC_Channel_Sub.cpp:39-42
 */
bool RC_Channels_Sub::in_rc_failsafe() const
{
    return sub.failsafe.radio;
}

/**
 * @brief Check if RC input is valid and usable for control
 * 
 * @details Validates RC input quality beyond basic failsafe detection, ensuring
 *          the RC data is stable and reliable for vehicle control. This method
 *          performs two checks:
 *          1. RC failsafe status - confirms signal is present
 *          2. Radio counter - ensures no recent signal interruptions
 * 
 *          The radio_counter tracks consecutive frames with valid RC data.
 *          A non-zero counter indicates recent signal loss or glitches, suggesting
 *          the input may be unreliable even if currently present.
 * 
 * @return true if RC input is valid and stable for control use
 * @return false if in failsafe or recent signal instability detected
 * 
 * @note This provides more conservative input validation than in_rc_failsafe()
 *       alone, preventing control actions during marginal signal conditions
 * 
 * @note Used by arming checks and mode transitions to ensure RC reliability
 *       before allowing state changes
 * 
 * @see in_rc_failsafe()
 * 
 * Source: ArduSub/RC_Channel_Sub.cpp:44-53
 */
bool RC_Channels_Sub::has_valid_input() const
{
    // First check: are we in RC failsafe?
    if (in_rc_failsafe()) {
        return false;
    }
    // Second check: have we had recent signal interruptions?
    // radio_counter increments during signal loss and decrements during recovery
    if (sub.failsafe.radio_counter != 0) {
        return false;
    }
    return true;
}


/**
 * @brief Execute auxiliary switch function for Sub-specific features
 * 
 * @details Called when an auxiliary switch changes state (LOW, MIDDLE, HIGH)
 *          or when activated by other triggers. Handles execution of auxiliary
 *          functions specific to underwater operations.
 * 
 *          Supported Sub auxiliary functions include:
 *          - LIGHTS1/LIGHTS2: Control LED brightness (0-100%)
 *          - CAMERA_TILT: Servo position control for camera mount
 *          - MOUNT_LOCK: Lock/unlock camera stabilization
 *          - GAIN_TOGGLE: Switch between gain presets
 *          - RELAY: Control external devices via relay outputs
 *          - SERVO functions: Direct servo position control
 *          - RANGEFINDER: Enable/disable rangefinder
 * 
 * @param[in] trigger Structure containing:
 *                    - ch_option: The auxiliary function to execute
 *                    - pos: Switch position (LOW, MIDDLE, HIGH)
 *                    - reason: What triggered the function
 * 
 * @return true if function was handled successfully
 * @return false if function is not supported or execution failed
 * 
 * @note Currently delegates to base RC_Channel implementation as Sub does not
 *       override standard auxiliary function behavior
 * 
 * @note Some functions (like lights) may have Sub-specific parameter ranges
 *       defined in the vehicle code
 * 
 * @see init_aux_function()
 * @see RC_Channel::do_aux_function()
 * 
 * Source: ArduSub/RC_Channel_Sub.cpp:57-60
 */
bool RC_Channel_Sub::do_aux_function(const AuxFuncTrigger &trigger)
{
   return RC_Channel::do_aux_function(trigger);
}
#else
/**
 * @brief Stub implementation when RC support is disabled
 * 
 * @details When AP_SUB_RC_ENABLED is not defined, RC channel functionality
 *          is disabled and this stub is provided for compatibility.
 * 
 * @return int8_t Returns 1 as placeholder (RC functionality not active)
 * 
 * @note This path is used when building ArduSub without RC receiver support,
 *       such as for autonomous-only configurations or testing
 */
int8_t RC_Channels_Sub::flight_mode_channel_number() const
{
    return 1; // sub does not have a flight mode channel when RC is disabled
}
#endif

/**
 * @brief Determine if throttle position arming checks should be enforced
 * 
 * @details Controls whether the standard minimum-throttle arming check should
 *          be applied based on Sub-specific throttle configuration. This is
 *          critical for supporting different control input types in underwater
 *          vehicles:
 * 
 *          - **Traditional RC throttle**: Requires throttle at minimum position
 *            before arming (prevents unexpected motor start)
 *          
 *          - **Joystick/center-sprung throttle** (thr_arming_position = WITHIN_THR_TRIM):
 *            Suppresses minimum-throttle check since center position is the
 *            neutral point, not minimum. ArduSub performs its own center-position
 *            check in this case.
 * 
 * @return true if standard minimum-throttle arming check should be applied
 * @return false if check should be suppressed (center-sprung throttle configured)
 * 
 * @note When thr_arming_position == WITHIN_THR_TRIM, the vehicle uses bidirectional
 *       thrust control (forward/reverse) requiring center-stick as neutral
 * 
 * @note ArduSub performs its own arming check for center-sprung throttles to
 *       verify the stick is within the trim range before arming
 * 
 * @warning Incorrect thr_arming_position configuration for your input device
 *          can prevent arming or allow arming in unsafe throttle positions
 * 
 * @see Sub::g.thr_arming_position parameter
 * @see AP_Arming_Sub::rc_calibration_checks()
 * 
 * Source: ArduSub/RC_Channel_Sub.cpp:70-77
 */
bool RC_Channels_Sub::arming_check_throttle() const {
    // Check if center-sprung/reversing throttle is configured AND base class wants to check
    if (sub.g.thr_arming_position == WITHIN_THR_TRIM && RC_Channels::arming_check_throttle()) {
        // Center sprung/reversing throttle configured - suppress AP_Arming min-throttle check
        // ArduSub validates center-stick position in its own vehicle-specific arming checks
        // This prevents false arming failures when throttle is at center (neutral) position
        return false;
    }
    // For traditional throttle configurations, use standard minimum-throttle check
    return RC_Channels::arming_check_throttle();
}
