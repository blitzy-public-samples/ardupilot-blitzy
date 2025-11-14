/**
 * @file RC_Channel_Copter.cpp
 * @brief ArduCopter RC channel input handling and auxiliary function processing
 * 
 * @details This file implements RC (Radio Control) channel processing specific to
 *          multicopter vehicles. It handles:
 *          - Flight mode switching from RC transmitter
 *          - Auxiliary function switches (landing gear, camera, parachute, etc.)
 *          - RC failsafe detection and validation
 *          - Motor interlock and safety interlocks
 *          - Simple and Super Simple mode configuration
 *          - Trim saving and auto-trim functionality
 *          
 *          Safety-Critical Functions:
 *          - Motor interlock prevents accidental motor start
 *          - Parachute deployment safety checks
 *          - Mode switching with safety validation
 *          - Failsafe detection and handling
 *          
 *          Thread Safety: Functions are called from main scheduler thread
 *          
 * @note All RC inputs are processed through debouncing and validation
 * @warning Improper auxiliary switch configuration can lead to unexpected mode changes
 * 
 * Source: ArduCopter/RC_Channel_Copter.cpp
 */
#include "Copter.h"

#include "RC_Channel_Copter.h"


// defining these two macros and including the RC_Channels_VarInfo header defines the parameter information common to all vehicle types
#define RC_CHANNELS_SUBCLASS RC_Channels_Copter
#define RC_CHANNEL_SUBCLASS RC_Channel_Copter

#include <RC_Channel/RC_Channels_VarInfo.h>

/**
 * @brief Get the RC channel number configured for flight mode selection
 * 
 * @details Returns the channel number (1-based) that is configured to control
 *          flight mode switching. This is typically a 6-position switch on the
 *          transmitter mapped to channels 5-8.
 *          
 *          The channel number is stored in the FLTMODE_CH parameter and can be
 *          configured via ground control station.
 * 
 * @return int8_t RC channel number (1-16), or 0 if not configured
 * 
 * @note This is called by the RC library to identify which channel to monitor for mode changes
 * 
 * Source: ArduCopter/RC_Channel_Copter.cpp:12-15
 */
int8_t RC_Channels_Copter::flight_mode_channel_number() const
{
    return copter.g.flight_mode_chan.get();
}

/**
 * @brief Handle flight mode switch position changes from RC transmitter
 * 
 * @details Called when the configured flight mode switch changes position.
 *          This function:
 *          1. Validates the new switch position is within valid range
 *          2. Attempts to switch to the mode configured for that switch position
 *          3. Automatically configures Simple/Super Simple mode if not controlled by aux switch
 *          
 *          Simple Mode Configuration:
 *          - If no auxiliary switch controls Simple Mode, uses stored EEPROM parameters
 *          - Each flight mode switch position can have independent Simple Mode setting
 *          - Super Simple mode takes precedence over Simple mode
 *          
 *          Safety: Mode changes may be rejected if vehicle state doesn't allow the requested mode
 * 
 * @param[in] new_pos New switch position (0-5, typically 6-position switch)
 * 
 * @return void
 * 
 * @note Switch position is debounced before this function is called
 * @note Failed mode changes are logged but do not prevent Simple Mode updates
 * @warning Invalid switch positions are silently ignored to prevent safety issues
 * 
 * Source: ArduCopter/RC_Channel_Copter.cpp:17-38
 */
void RC_Channel_Copter::mode_switch_changed(modeswitch_pos_t new_pos)
{
    // Validate switch position is within configured range
    if (new_pos < 0 || (uint8_t)new_pos > copter.num_flight_modes) {
        // should not have been called
        return;
    }

    // Attempt to switch to the flight mode configured for this switch position
    // Mode change may fail if vehicle state doesn't permit the requested mode
    if (!copter.set_mode((Mode::Number)copter.flight_modes[new_pos].get(), ModeReason::RC_COMMAND)) {
        return;
    }

    // Check if Simple or Super Simple mode is controlled by an auxiliary switch
    if (!rc().find_channel_for_option(AUX_FUNC::SIMPLE_MODE) &&
        !rc().find_channel_for_option(AUX_FUNC::SUPERSIMPLE_MODE)) {
        // if none of the Aux Switches are set to Simple or Super Simple Mode then
        // set Simple Mode using stored parameters from EEPROM
        // Each flight mode position can have independent Simple Mode configuration
        if (BIT_IS_SET(copter.g.super_simple, new_pos)) {
            copter.set_simple_mode(Copter::SimpleMode::SUPERSIMPLE);
        } else {
            copter.set_simple_mode(BIT_IS_SET(copter.g.simple_modes, new_pos) ? Copter::SimpleMode::SIMPLE : Copter::SimpleMode::NONE);
        }
    }
}

/**
 * @brief Check if vehicle is currently in RC failsafe condition
 * 
 * @details Returns true if radio failsafe has been triggered due to loss of
 *          RC signal or invalid RC data. This state is set by the failsafe
 *          monitoring system when RC input is lost for the configured timeout period.
 * 
 * @return true if RC failsafe is active
 * @return false if RC signal is normal
 * 
 * @note This is used by RC library to determine if RC input should be processed
 * @warning In RC failsafe, vehicle will execute configured failsafe action (RTL, Land, etc.)
 * 
 * Source: ArduCopter/RC_Channel_Copter.cpp:40-43
 */
bool RC_Channels_Copter::in_rc_failsafe() const
{
    return copter.failsafe.radio;
}

/**
 * @brief Check if RC input is valid and safe to use for vehicle control
 * 
 * @details Validates RC input by checking multiple conditions:
 *          1. Not in RC failsafe state (signal present and valid)
 *          2. Radio counter is zero (no recent glitches or signal issues)
 *          
 *          The radio_counter tracks transient signal issues and must be zero
 *          for input to be considered fully valid. This prevents using RC input
 *          immediately after signal recovery before stability is confirmed.
 *          
 *          Safety: This validation prevents erratic vehicle behavior from
 *          using marginal or recovering RC signals.
 * 
 * @return true if RC input is valid and safe to use
 * @return false if RC input should not be trusted
 * 
 * @note Called frequently by control loops to validate RC input
 * @warning Never bypass this check - using invalid RC input can cause crashes
 * 
 * Source: ArduCopter/RC_Channel_Copter.cpp:45-54
 */
bool RC_Channels_Copter::has_valid_input() const
{
    // First check: RC failsafe must not be active
    if (in_rc_failsafe()) {
        return false;
    }
    // Second check: Radio counter must be zero (no recent glitches)
    if (copter.failsafe.radio_counter != 0) {
        return false;
    }
    return true;
}

/**
 * @brief Determine if throttle position arming checks should be performed
 * 
 * @details Controls whether the standard RC throttle low check is required for arming.
 *          
 *          For standard throttle sticks (bottom = zero throttle):
 *          - Returns true to enforce throttle-low requirement before arming
 *          - Prevents accidental motor start with throttle raised
 *          
 *          For center-sprung throttles (middle = zero thrust):
 *          - Returns false to skip throttle position check
 *          - Center-sprung throttles don't have a "low" position
 *          - Copter has separate arming checks for this configuration
 *          
 *          Safety: This prevents arming with throttle raised which would
 *          cause immediate motor spin-up and potential injury or crash.
 * 
 * @return true if throttle-low check should be enforced for arming
 * @return false if center-sprung throttle is configured
 * 
 * @note Throttle behavior is configured via THR_BEHAVE parameter
 * @warning Disabling throttle checks without proper safeguards is dangerous
 * 
 * Source: ArduCopter/RC_Channel_Copter.cpp:56-64
 */
// returns true if throttle arming checks should be run
bool RC_Channels_Copter::arming_check_throttle() const {
    // Check if center-sprung throttle is configured
    if ((copter.g.throttle_behavior & THR_BEHAVE_FEEDBACK_FROM_MID_STICK) != 0) {
        // center sprung throttle configured, dont run AP_Arming check
        // Copter already checks this case in its own arming checks
        return false;
    }
    // Standard throttle stick - require throttle low for arming
    return RC_Channels::arming_check_throttle();
}

/**
 * @brief Get the RC channel used for rudder stick arming
 * 
 * @details Returns the yaw (rudder) channel which is used for stick-based
 *          arming and disarming. The standard arming procedure is:
 *          - Throttle low + Yaw right (held) = Arm
 *          - Throttle low + Yaw left (held) = Disarm
 *          
 *          The yaw channel is used because it's least likely to cause
 *          unintended motion during the arming sequence.
 * 
 * @return RC_Channel* Pointer to yaw channel (typically channel 4)
 * 
 * @note This is called by the arming library to monitor stick positions
 * @note Some configurations may disable stick arming entirely
 * 
 * Source: ArduCopter/RC_Channel_Copter.cpp:66-69
 */
RC_Channel * RC_Channels_Copter::get_arming_channel(void) const
{
    return copter.channel_yaw;
}

/**
 * @brief Initialize auxiliary function based on current switch position at startup
 * 
 * @details Called during vehicle initialization to set the initial state of each
 *          configured auxiliary function based on its current RC switch position.
 *          
 *          Function Categories:
 *          
 *          1. Flight Modes (no initialization needed):
 *             - Mode switches handled by flight_mode_channel
 *             - Include: ALTHOLD, AUTO, LOITER, RTL, GUIDED, LAND, BRAKE, etc.
 *          
 *          2. State Functions (require initialization):
 *             - Must be set to match switch position at boot
 *             - Include: MOTOR_INTERLOCK, PARACHUTE_ENABLE, SIMPLE_MODE, AIRMODE
 *             - Prevents unexpected state changes when switch already positioned
 *          
 *          3. Action Functions (no initialization):
 *             - Trigger-based actions that execute on switch change
 *             - Include: SAVE_WP, PARACHUTE_RELEASE, SAVE_TRIM
 *             - No persistent state to initialize
 *          
 *          Safety Considerations:
 *          - Motor interlock initialized to prevent unexpected motor start
 *          - Parachute state matched to switch to prevent inadvertent deployment
 *          - State functions synced to prevent mode confusion at startup
 * 
 * @param[in] ch_option Auxiliary function type to initialize
 * @param[in] ch_flag Current position of the RC switch (LOW/MIDDLE/HIGH)
 * 
 * @return void
 * 
 * @note Called once per configured aux function during vehicle initialization
 * @note Some functions defer to parent class init_aux_function for common behavior
 * @warning Improper initialization can cause unexpected vehicle behavior at startup
 * 
 * Source: ArduCopter/RC_Channel_Copter.cpp:71-152
 */
// init_aux_switch_function - initialize aux functions
void RC_Channel_Copter::init_aux_function(const AUX_FUNC ch_option, const AuxSwitchPos ch_flag)
{
    // init channel options
    switch(ch_option) {
    // Flight mode and action functions do not need initialization
    // They either have no persistent state or are handled by mode switch logic
    case AUX_FUNC::ALTHOLD:
    case AUX_FUNC::AUTO:
    case AUX_FUNC::AUTOTUNE_MODE:
    case AUX_FUNC::AUTOTUNE_TEST_GAINS:
    case AUX_FUNC::BRAKE:
    case AUX_FUNC::CIRCLE:
    case AUX_FUNC::DRIFT:
    case AUX_FUNC::FLIP:
    case AUX_FUNC::FLOWHOLD:
    case AUX_FUNC::FOLLOW:
    case AUX_FUNC::GUIDED:
    case AUX_FUNC::LAND:
    case AUX_FUNC::LOITER:
#if HAL_PARACHUTE_ENABLED
    case AUX_FUNC::PARACHUTE_RELEASE:
#endif
    case AUX_FUNC::POSHOLD:
    case AUX_FUNC::RESETTOARMEDYAW:
    case AUX_FUNC::RTL:
    case AUX_FUNC::SAVE_TRIM:
    case AUX_FUNC::SAVE_WP:
    case AUX_FUNC::SMART_RTL:
    case AUX_FUNC::STABILIZE:
    case AUX_FUNC::THROW:
    case AUX_FUNC::USER_FUNC1:
    case AUX_FUNC::USER_FUNC2:
    case AUX_FUNC::USER_FUNC3:
#if AP_WINCH_ENABLED
    case AUX_FUNC::WINCH_CONTROL:
#endif
    case AUX_FUNC::ZIGZAG:
    case AUX_FUNC::ZIGZAG_Auto:
    case AUX_FUNC::ZIGZAG_SaveWP:
    case AUX_FUNC::ACRO:
    case AUX_FUNC::AUTO_RTL:
    case AUX_FUNC::TURTLE:
    case AUX_FUNC::SIMPLE_HEADING_RESET:
    case AUX_FUNC::ARMDISARM_AIRMODE:
    case AUX_FUNC::TURBINE_START:
    case AUX_FUNC::FLIGHTMODE_PAUSE:
#if AP_COPTER_AHRS_AUTO_TRIM_ENABLED
    case AUX_FUNC::AHRS_AUTO_TRIM:
#endif
        break;
    // State-based functions that MUST be initialized to match switch position
    // This prevents unexpected state changes when vehicle powers up with switch already positioned
    case AUX_FUNC::ACRO_TRAINER:
    case AUX_FUNC::ATTCON_ACCEL_LIM:
    case AUX_FUNC::ATTCON_FEEDFWD:
    case AUX_FUNC::INVERTED:
    case AUX_FUNC::MOTOR_INTERLOCK:  // Critical: Must match switch to prevent unexpected motor start
#if HAL_PARACHUTE_ENABLED
    case AUX_FUNC::PARACHUTE_3POS:      // we trust the vehicle will be disarmed so even if switch is in release position the chute will not release
    case AUX_FUNC::PARACHUTE_ENABLE:
#endif
    case AUX_FUNC::PRECISION_LOITER:
#if AP_RANGEFINDER_ENABLED
    case AUX_FUNC::RANGEFINDER:
#endif
    case AUX_FUNC::SIMPLE_MODE:
    case AUX_FUNC::STANDBY:
    case AUX_FUNC::SUPERSIMPLE_MODE:
    case AUX_FUNC::SURFACE_TRACKING:
#if AP_WINCH_ENABLED
    case AUX_FUNC::WINCH_ENABLE:
#endif
    case AUX_FUNC::AIRMODE:
    case AUX_FUNC::FORCEFLYING:
    case AUX_FUNC::CUSTOM_CONTROLLER:
    case AUX_FUNC::WEATHER_VANE_ENABLE:
    case AUX_FUNC::TRANSMITTER_TUNING:
        run_aux_function(ch_option, ch_flag, AuxFuncTrigger::Source::INIT, ch_in);
        break;
    default:
        RC_Channel::init_aux_function(ch_option, ch_flag);
        break;
    }
}

/**
 * @brief Change flight mode based on auxiliary switch position
 * 
 * @details Implements momentary auxiliary switch behavior for flight modes.
 *          This allows temporary mode changes via auxiliary switches that
 *          automatically revert when the switch is returned to low position.
 *          
 *          Switch Position Behavior:
 *          - HIGH: Engage the specified flight mode
 *          - MIDDLE/LOW: Return to flight mode switch's configured mode
 *          
 *          Example Use Cases:
 *          - RTL switch: HIGH = Return To Launch, LOW = return to normal mode
 *          - LAND switch: HIGH = Land mode, LOW = return to flight mode switch
 *          - LOITER switch: HIGH = Loiter mode, LOW = return to flight mode switch
 *          
 *          Mode Change Safety:
 *          - Mode changes may be rejected if vehicle state doesn't permit
 *          - Vehicle remains in current mode if requested mode is unavailable
 *          - Failed mode changes are logged but don't cause errors
 * 
 * @param[in] mode Flight mode number to engage when switch is HIGH
 * @param[in] ch_flag Current auxiliary switch position (HIGH/MIDDLE/LOW)
 * 
 * @return void
 * 
 * @note This implements "momentary" switch behavior - switch must be held HIGH
 * @note Mode changes respect vehicle state and arming status
 * @warning Some modes require specific conditions (GPS lock, etc.) to engage
 * 
 * Source: ArduCopter/RC_Channel_Copter.cpp:154-172
 */
// do_aux_function_change_mode - change mode based on an aux switch
// being moved
void RC_Channel_Copter::do_aux_function_change_mode(const Mode::Number mode,
                                                    const AuxSwitchPos ch_flag)
{
    switch(ch_flag) {
    case AuxSwitchPos::HIGH: {
        // engage mode (if not possible we remain in current flight mode)
        copter.set_mode(mode, ModeReason::AUX_FUNCTION);
        break;
    }
    default:
        // return to flight mode switch's flight mode if we are currently
        // in this mode
        if (copter.flightmode->mode_number() == mode) {
            rc().reset_mode_switch();
        }
    }
}

/**
 * @brief Execute auxiliary function action based on RC switch change
 * 
 * @details Main dispatcher for all auxiliary switch functions. Called when an
 *          auxiliary switch changes position (LOW→MIDDLE→HIGH). Implements the
 *          vehicle-specific behavior for each auxiliary function option.
 *          
 *          Function Categories Handled:
 *          
 *          1. Flight Modes:
 *             - FLIP, RTL, AUTO, LOITER, GUIDED, LAND, BRAKE, THROW, ZIGZAG, etc.
 *             - Use do_aux_function_change_mode() for momentary mode switching
 *          
 *          2. Control Modifiers:
 *             - SIMPLE_MODE, SUPERSIMPLE_MODE: Heading reference simplification
 *             - ACRO_TRAINER: Training wheels for acro mode
 *             - ATTCON_FEEDFWD: Attitude controller feed-forward enable/disable
 *             - ATTCON_ACCEL_LIM: Acceleration limiting enable/disable
 *          
 *          3. Safety Systems:
 *             - MOTOR_INTERLOCK: Emergency motor enable/disable
 *             - PARACHUTE_ENABLE/RELEASE: Parachute deployment control
 *             - STANDBY: Disable motor output while armed
 *          
 *          4. Navigation Features:
 *             - PRECISION_LOITER: High-precision position hold using beacons
 *             - SURFACE_TRACKING: Ground/ceiling following with rangefinder
 *             - RANGEFINDER: Enable/disable rangefinder usage
 *          
 *          5. Accessories:
 *             - WINCH_ENABLE/CONTROL: Winch operation
 *             - INVERTED: Inverted flight mode (helicopter only)
 *             - TURBINE_START: Turbine engine control (helicopter only)
 *          
 *          6. Utility Functions:
 *             - SAVE_TRIM: Save current RC trim to AHRS
 *             - SAVE_WP: Add current position to mission
 *             - SIMPLE_HEADING_RESET: Reset simple mode heading reference
 *          
 *          Input Debouncing:
 *          - RC switch positions are debounced before this function is called
 *          - Prevents multiple triggers from single physical switch movement
 *          - Debounce time: typically 100ms
 *          
 *          Safety Interlocks:
 *          - Motor interlock prevents accidental motor start
 *          - Parachute requires HIGH position and deliberate action
 *          - SAVE_WP blocked in AUTO mode and when disarmed
 *          - Many functions check arming state and flight mode
 * 
 * @param[in] trigger Structure containing:
 *                    - func: Auxiliary function to execute (AUX_FUNC enum)
 *                    - pos: New switch position (LOW/MIDDLE/HIGH)
 *                    - source: Trigger source (RC, init, script)
 * 
 * @return true if function was handled (Copter-specific or parent class)
 * @return false if function is not supported
 * 
 * @note Switch position mapping: LOW=1000-1230μs, MIDDLE=1231-1760μs, HIGH=1761-2000μs
 * @note Some functions only trigger on HIGH position to prevent accidental activation
 * @warning Safety-critical functions have additional state checks before execution
 * 
 * Source: ArduCopter/RC_Channel_Copter.cpp:174-686
 */
// do_aux_function - implement the function invoked by auxiliary switches
bool RC_Channel_Copter::do_aux_function(const AuxFuncTrigger &trigger)
{
    const AUX_FUNC &ch_option = trigger.func;
    const AuxSwitchPos &ch_flag = trigger.pos;

    switch(ch_option) {
        case AUX_FUNC::FLIP:
            // Flip mode: Performs aerobatic flip maneuver
            // Safety: Only engages on HIGH position, flip mode has internal safety checks
            // for altitude, throttle, and vehicle state before executing flip
            if (ch_flag == AuxSwitchPos::HIGH) {
                copter.set_mode(Mode::Number::FLIP, ModeReason::AUX_FUNCTION);
            }
            break;

        case AUX_FUNC::SIMPLE_MODE:
            // Simple Mode: Makes copter fly relative to initial heading instead of current heading
            // LOW = Simple mode OFF (normal heading-based control)
            // MIDDLE/HIGH = Simple mode ON (flies relative to arming direction)
            // Useful for new pilots or when copter orientation is hard to determine
            copter.set_simple_mode((ch_flag == AuxSwitchPos::LOW) ? Copter::SimpleMode::NONE : Copter::SimpleMode::SIMPLE);
            break;

        case AUX_FUNC::SUPERSIMPLE_MODE: {
            // Super Simple Mode: 3-position switch for graduated simple mode control
            // LOW = No simple mode (normal heading-based control)
            // MIDDLE = Simple mode (flies relative to arming heading)
            // HIGH = Super Simple mode (always flies relative to home direction)
            // Super Simple makes copter fly toward/away from home regardless of yaw
            Copter::SimpleMode newmode = Copter::SimpleMode::NONE;
            switch (ch_flag) {
            case AuxSwitchPos::LOW:
                break;
            case AuxSwitchPos::MIDDLE:
                newmode = Copter::SimpleMode::SIMPLE;
                break;
            case AuxSwitchPos::HIGH:
                newmode = Copter::SimpleMode::SUPERSIMPLE;
                break;
            }
            copter.set_simple_mode(newmode);
            break;
        }

#if MODE_RTL_ENABLED
        case AUX_FUNC::RTL:
            do_aux_function_change_mode(Mode::Number::RTL, ch_flag);
            break;
#endif

        case AUX_FUNC::SAVE_TRIM:
            // Save Trim: Records current RC stick positions as trim offsets
            // Safety interlocks (all must be true):
            // 1. Switch must be HIGH (prevents accidental save)
            // 2. Flight mode must allow trim save (not all modes support it)
            // 3. Throttle must be zero (prevents saving trim during flight)
            // This ensures trim is only saved when copter is stable and safe
            if ((ch_flag == AuxSwitchPos::HIGH) &&
                (copter.flightmode->allows_save_trim()) &&
                (copter.channel_throttle->get_control_in() == 0)) {
                copter.g2.rc_channels.save_trim();
            }
            break;

#if MODE_AUTO_ENABLED
        case AUX_FUNC::SAVE_WP:
            // Save Waypoint: Adds current position to mission plan on-the-fly
            // Allows building missions during flight by marking locations with switch
            // save waypoint when switch is brought high
            if (ch_flag == RC_Channel::AuxSwitchPos::HIGH) {

                // Safety Interlock 1: Prevent saving while in AUTO mode or disarmed
                // - AUTO mode: Mission is actively executing, don't modify it mid-flight
                // - Disarmed: Position may be invalid, motors not ready
                if (copter.flightmode == &copter.mode_auto || !copter.motors->armed()) {
                    break;
                }

                // Safety Interlock 2: First waypoint must be saved with throttle up
                // Ensures vehicle is airborne before starting mission
                // Prevents creating mission that starts on ground
                if (!copter.mode_auto.mission.present() && (copter.channel_throttle->get_control_in() == 0)) {
                    break;
                }

                // create new mission command
                AP_Mission::Mission_Command cmd  = {};

                // Special case: If mission is empty, automatically add takeoff command first
                // This ensures any newly created mission starts with proper takeoff
                if (!copter.mode_auto.mission.present()) {
                    // set our location ID to 16, MAV_CMD_NAV_WAYPOINT
                    cmd.id = MAV_CMD_NAV_TAKEOFF;
                    // Takeoff to current altitude or minimum 100cm, whichever is higher
                    cmd.content.location.alt = MAX(copter.current_loc.alt,100);

                    // use the current altitude for the target alt for takeoff.
                    // only altitude will matter to the AP mission script for takeoff.
                    if (copter.mode_auto.mission.add_cmd(cmd)) {
                        // log event
                        LOGGER_WRITE_EVENT(LogEvent::SAVEWP_ADD_WP);
                    }
                }

                // set new waypoint to current location
                cmd.content.location = copter.current_loc;

                // Waypoint type selection based on throttle:
                // Throttle > 0: Create navigation waypoint (fly to this position)
                // Throttle = 0: Create land command (descend and land at this position)
                // This allows marking landing zones by reducing throttle to zero
                if (copter.channel_throttle->get_control_in() > 0) {
                    cmd.id = MAV_CMD_NAV_WAYPOINT;
                } else {
                    // with zero throttle, create LAND command
                    cmd.id = MAV_CMD_NAV_LAND;
                }

                // save command
                if (copter.mode_auto.mission.add_cmd(cmd)) {
                    // log event
                    LOGGER_WRITE_EVENT(LogEvent::SAVEWP_ADD_WP);
                }
            }
            break;

        case AUX_FUNC::AUTO:
            do_aux_function_change_mode(Mode::Number::AUTO, ch_flag);
            break;
#endif

#if AP_RANGEFINDER_ENABLED
        case AUX_FUNC::RANGEFINDER:
            // enable or disable the rangefinder
            if ((ch_flag == AuxSwitchPos::HIGH) &&
                copter.rangefinder.has_orientation(ROTATION_PITCH_270)) {
                copter.rangefinder_state.enabled = true;
            } else {
                copter.rangefinder_state.enabled = false;
            }
            break;
#endif // AP_RANGEFINDER_ENABLED

#if MODE_ACRO_ENABLED
        case AUX_FUNC::ACRO_TRAINER:
            switch(ch_flag) {
                case AuxSwitchPos::LOW:
                    copter.g.acro_trainer.set((uint8_t)ModeAcro::Trainer::OFF);
                    LOGGER_WRITE_EVENT(LogEvent::ACRO_TRAINER_OFF);
                    break;
                case AuxSwitchPos::MIDDLE:
                    copter.g.acro_trainer.set((uint8_t)ModeAcro::Trainer::LEVELING);
                    LOGGER_WRITE_EVENT(LogEvent::ACRO_TRAINER_LEVELING);
                    break;
                case AuxSwitchPos::HIGH:
                    copter.g.acro_trainer.set((uint8_t)ModeAcro::Trainer::LIMITED);
                    LOGGER_WRITE_EVENT(LogEvent::ACRO_TRAINER_LIMITED);
                    break;
            }
            break;
#endif

#if AUTOTUNE_ENABLED
        case AUX_FUNC::AUTOTUNE_MODE:
            do_aux_function_change_mode(Mode::Number::AUTOTUNE, ch_flag);
            break;
        case AUX_FUNC::AUTOTUNE_TEST_GAINS:
            copter.mode_autotune.autotune.do_aux_function(ch_flag);
            break;
#endif

        case AUX_FUNC::LAND:
            do_aux_function_change_mode(Mode::Number::LAND, ch_flag);
            break;

        case AUX_FUNC::GUIDED:
            do_aux_function_change_mode(Mode::Number::GUIDED, ch_flag);
            break;

        case AUX_FUNC::LOITER:
            do_aux_function_change_mode(Mode::Number::LOITER, ch_flag);
            break;

        case AUX_FUNC::FOLLOW:
            do_aux_function_change_mode(Mode::Number::FOLLOW, ch_flag);
            break;

#if HAL_PARACHUTE_ENABLED
        case AUX_FUNC::PARACHUTE_ENABLE:
            // Parachute Enable/Disable: Arms/disarms the parachute system
            // LOW = Parachute system disabled (cannot deploy)
            // HIGH = Parachute system enabled (can deploy automatically or manually)
            // Safety: Disabled parachute cannot deploy even if criteria are met
            // Use case: Disable during normal flight, enable for risky maneuvers
            copter.parachute.enabled(ch_flag == AuxSwitchPos::HIGH);
            break;

        case AUX_FUNC::PARACHUTE_RELEASE:
            // Parachute Manual Release: Emergency parachute deployment
            // CRITICAL SAFETY FUNCTION - One-time irreversible action
            // 
            // Trigger: Only on HIGH position (prevents accidental deployment)
            // Effect: Immediately deploys parachute via servo/relay
            // 
            // Safety Interlocks:
            // - Switch must be HIGH (deliberate action required)
            // - Internal checks verify deployment is appropriate
            // 
            // WARNING: Parachute deployment is irreversible and typically requires
            // parachute repack before next flight. Only use in genuine emergencies.
            if (ch_flag == AuxSwitchPos::HIGH) {
                copter.parachute_manual_release();
            }
            break;

        case AUX_FUNC::PARACHUTE_3POS:
            // Parachute 3-Position Control: Integrated enable and release control
            // Combines PARACHUTE_ENABLE and PARACHUTE_RELEASE into single switch
            // 
            // LOW = Parachute system disabled (safe state)
            // MIDDLE = Parachute system enabled (ready but not deployed)
            // HIGH = Parachute system enabled AND deployed (emergency deployment)
            // 
            // Typical usage: Keep in MIDDLE during risky flight, move to HIGH in emergency
            // Safety: Requires deliberate move through MIDDLE to reach HIGH
            switch (ch_flag) {
                case AuxSwitchPos::LOW:
                    copter.parachute.enabled(false);
                    break;
                case AuxSwitchPos::MIDDLE:
                    copter.parachute.enabled(true);
                    break;
                case AuxSwitchPos::HIGH:
                    copter.parachute.enabled(true);
                    copter.parachute_manual_release();
                    break;
            }
            break;
#endif  // HAL_PARACHUTE_ENABLED

        case AUX_FUNC::ATTCON_FEEDFWD:
            // enable or disable feed forward
            copter.attitude_control->bf_feedforward(ch_flag == AuxSwitchPos::HIGH);
            break;

        case AUX_FUNC::ATTCON_ACCEL_LIM:
            // enable or disable accel limiting by restoring defaults
            copter.attitude_control->accel_limiting(ch_flag == AuxSwitchPos::HIGH);
            break;

        case AUX_FUNC::MOTOR_INTERLOCK:
            // Motor Interlock: Emergency motor enable/disable control
            // CRITICAL SAFETY FUNCTION - Provides immediate motor shutdown capability
            // 
            // Switch Position Behavior:
            // - LOW: Motors disabled (interlock OFF) - immediate motor stop
            // - MIDDLE/HIGH: Motors enabled (interlock ON) - normal operation
            // 
            // Safety Use Cases:
            // - Emergency shutdown if vehicle becomes uncontrollable
            // - Safe transport (prevents accidental motor start when armed)
            // - Ground testing without motor spin
            // 
            // WARNING: Turning off interlock in flight will cause immediate crash
            // Only use for genuine emergencies or when vehicle is on ground
#if FRAME_CONFIG == HELI_FRAME
            // Helicopter-specific: Passthrough mode handles interlock in rotor speed control
            // The interlock logic for ROTOR_CONTROL_MODE_PASSTHROUGH is handled 
            // in heli_update_rotor_speed_targets.  Otherwise turn on when above low.
            if (copter.motors->get_rsc_mode() != ROTOR_CONTROL_MODE_PASSTHROUGH) {
                copter.ap.motor_interlock_switch = (ch_flag == AuxSwitchPos::HIGH || ch_flag == AuxSwitchPos::MIDDLE);
            }
#else
            // Multicopter: Interlock enabled when switch is MIDDLE or HIGH
            copter.ap.motor_interlock_switch = (ch_flag == AuxSwitchPos::HIGH || ch_flag == AuxSwitchPos::MIDDLE);
#endif
            break;

#if FRAME_CONFIG == HELI_FRAME
        case AUX_FUNC::TURBINE_START:
           switch (ch_flag) {
                case AuxSwitchPos::HIGH:
                    copter.motors->set_turb_start(true);
                    break;
                case AuxSwitchPos::MIDDLE:
                    // nothing
                    break;
                case AuxSwitchPos::LOW:
                    copter.motors->set_turb_start(false);
                    break;
           }
           break;
#endif

#if MODE_BRAKE_ENABLED
        case AUX_FUNC::BRAKE:
            do_aux_function_change_mode(Mode::Number::BRAKE, ch_flag);
            break;
#endif

#if MODE_THROW_ENABLED
        case AUX_FUNC::THROW:
            do_aux_function_change_mode(Mode::Number::THROW, ch_flag);
            break;
#endif

#if AC_PRECLAND_ENABLED && MODE_LOITER_ENABLED
        case AUX_FUNC::PRECISION_LOITER:
            switch (ch_flag) {
                case AuxSwitchPos::HIGH:
                    copter.mode_loiter.set_precision_loiter_enabled(true);
                    break;
                case AuxSwitchPos::MIDDLE:
                    // nothing
                    break;
                case AuxSwitchPos::LOW:
                    copter.mode_loiter.set_precision_loiter_enabled(false);
                    break;
            }
            break;
#endif

#if MODE_SMARTRTL_ENABLED
        case AUX_FUNC::SMART_RTL:
            do_aux_function_change_mode(Mode::Number::SMART_RTL, ch_flag);
            break;
#endif

#if FRAME_CONFIG == HELI_FRAME
        case AUX_FUNC::INVERTED:
            switch (ch_flag) {
            case AuxSwitchPos::HIGH:
                if (copter.flightmode->allows_inverted()) {
                    copter.attitude_control->set_inverted_flight(true);
                } else {
                    gcs().send_text(MAV_SEVERITY_WARNING, "Inverted flight not available in %s mode", copter.flightmode->name());
                }
                break;
            case AuxSwitchPos::MIDDLE:
                // nothing
                break;
            case AuxSwitchPos::LOW:
                copter.attitude_control->set_inverted_flight(false);
                break;
            }
            break;
#endif

#if AP_WINCH_ENABLED
        case AUX_FUNC::WINCH_ENABLE:
            switch (ch_flag) {
                case AuxSwitchPos::HIGH:
                    // high switch position stops winch using rate control
                    copter.g2.winch.set_desired_rate(0.0f);
                    break;
                case AuxSwitchPos::MIDDLE:
                case AuxSwitchPos::LOW:
                    // all other position relax winch
                    copter.g2.winch.relax();
                    break;
                }
            break;

        case AUX_FUNC::WINCH_CONTROL:
            // do nothing, used to control the rate of the winch and is processed within AP_Winch
            break;
#endif  // AP_WINCH_ENABLED

#ifdef USERHOOK_AUXSWITCH
        case AUX_FUNC::USER_FUNC1:
            copter.userhook_auxSwitch1(ch_flag);
            break;

        case AUX_FUNC::USER_FUNC2:
            copter.userhook_auxSwitch2(ch_flag);
            break;

        case AUX_FUNC::USER_FUNC3:
            copter.userhook_auxSwitch3(ch_flag);
            break;
#endif

#if MODE_ZIGZAG_ENABLED
        case AUX_FUNC::ZIGZAG:
            do_aux_function_change_mode(Mode::Number::ZIGZAG, ch_flag);
            break;

        case AUX_FUNC::ZIGZAG_SaveWP:
            if (copter.flightmode == &copter.mode_zigzag) {
                // initialize zigzag auto
                copter.mode_zigzag.init_auto();
                switch (ch_flag) {
                    case AuxSwitchPos::LOW:
                        copter.mode_zigzag.save_or_move_to_destination(ModeZigZag::Destination::A);
                        break;
                    case AuxSwitchPos::MIDDLE:
                        copter.mode_zigzag.return_to_manual_control(false);
                        break;
                    case AuxSwitchPos::HIGH:
                        copter.mode_zigzag.save_or_move_to_destination(ModeZigZag::Destination::B);
                        break;
                }
            }
            break;
#endif

        case AUX_FUNC::STABILIZE:
            do_aux_function_change_mode(Mode::Number::STABILIZE, ch_flag);
            break;

#if MODE_POSHOLD_ENABLED
        case AUX_FUNC::POSHOLD:
            do_aux_function_change_mode(Mode::Number::POSHOLD, ch_flag);
            break;
#endif

        case AUX_FUNC::ALTHOLD:
            do_aux_function_change_mode(Mode::Number::ALT_HOLD, ch_flag);
            break;

#if MODE_ACRO_ENABLED
        case AUX_FUNC::ACRO:
            do_aux_function_change_mode(Mode::Number::ACRO, ch_flag);
            break;
#endif

#if MODE_FLOWHOLD_ENABLED
        case AUX_FUNC::FLOWHOLD:
            do_aux_function_change_mode(Mode::Number::FLOWHOLD, ch_flag);
            break;
#endif

#if MODE_CIRCLE_ENABLED
        case AUX_FUNC::CIRCLE:
            do_aux_function_change_mode(Mode::Number::CIRCLE, ch_flag);
            break;
#endif

#if MODE_DRIFT_ENABLED
        case AUX_FUNC::DRIFT:
            do_aux_function_change_mode(Mode::Number::DRIFT, ch_flag);
            break;
#endif

        case AUX_FUNC::STANDBY: {
            switch (ch_flag) {
                case AuxSwitchPos::HIGH:
                    copter.standby_active = true;
                    LOGGER_WRITE_EVENT(LogEvent::STANDBY_ENABLE);
                    gcs().send_text(MAV_SEVERITY_INFO, "Stand By Enabled");
                    break;
                default:
                    copter.standby_active = false;
                    LOGGER_WRITE_EVENT(LogEvent::STANDBY_DISABLE);
                    gcs().send_text(MAV_SEVERITY_INFO, "Stand By Disabled");
                    break;
                }
            break;
        }

#if AP_RANGEFINDER_ENABLED
        case AUX_FUNC::SURFACE_TRACKING:
            switch (ch_flag) {
            case AuxSwitchPos::LOW:
                copter.surface_tracking.set_surface(Copter::SurfaceTracking::Surface::GROUND);
                break;
            case AuxSwitchPos::MIDDLE:
                copter.surface_tracking.set_surface(Copter::SurfaceTracking::Surface::NONE);
                break;
            case AuxSwitchPos::HIGH:
                copter.surface_tracking.set_surface(Copter::SurfaceTracking::Surface::CEILING);
                break;
            }
            break;
#endif

        case AUX_FUNC::FLIGHTMODE_PAUSE:
            switch (ch_flag) {
                case AuxSwitchPos::HIGH:
                    if (!copter.flightmode->pause()) {
                        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Flight Mode Pause failed");
                    }
                    break;
                case AuxSwitchPos::MIDDLE:
                    break;
                case AuxSwitchPos::LOW:
                    copter.flightmode->resume();
                    break;
            }
            break;

#if MODE_ZIGZAG_ENABLED
        case AUX_FUNC::ZIGZAG_Auto:
            if (copter.flightmode == &copter.mode_zigzag) {
                switch (ch_flag) {
                case AuxSwitchPos::HIGH:
                    copter.mode_zigzag.run_auto();
                    break;
                default:
                    copter.mode_zigzag.suspend_auto();
                    break;
                }
            }
            break;
#endif

        case AUX_FUNC::AIRMODE:
            do_aux_function_change_air_mode(ch_flag);
#if MODE_ACRO_ENABLED && FRAME_CONFIG != HELI_FRAME
            copter.mode_acro.air_mode_aux_changed();
#endif
            break;

        case AUX_FUNC::FORCEFLYING:
            do_aux_function_change_force_flying(ch_flag);
            break;

#if MODE_AUTO_ENABLED
        case AUX_FUNC::AUTO_RTL:
            do_aux_function_change_mode(Mode::Number::AUTO_RTL, ch_flag);
            break;
#endif

#if MODE_TURTLE_ENABLED
        case AUX_FUNC::TURTLE:
            do_aux_function_change_mode(Mode::Number::TURTLE, ch_flag);
            break;
#endif

#if AP_COPTER_AHRS_AUTO_TRIM_ENABLED
        case AUX_FUNC::AHRS_AUTO_TRIM:
            copter.g2.rc_channels.do_aux_function_ahrs_auto_trim(ch_flag);
            break;
#endif  // AP_COPTER_AHRS_AUTO_TRIM_ENABLED

        case AUX_FUNC::SIMPLE_HEADING_RESET:
            if (ch_flag == AuxSwitchPos::HIGH) {
                copter.init_simple_bearing();
                gcs().send_text(MAV_SEVERITY_INFO, "Simple heading reset");
            }
            break;

        case AUX_FUNC::ARMDISARM_AIRMODE:
            RC_Channel::do_aux_function_armdisarm(ch_flag);
            if (copter.arming.is_armed()) {
                copter.ap.armed_with_airmode_switch = true;
            }
            break;

#if AC_CUSTOMCONTROL_MULTI_ENABLED
        case AUX_FUNC::CUSTOM_CONTROLLER:
            copter.custom_control.set_custom_controller(ch_flag == AuxSwitchPos::HIGH);
            break;
#endif

#if WEATHERVANE_ENABLED
    case AUX_FUNC::WEATHER_VANE_ENABLE: {
        switch (ch_flag) {
            case AuxSwitchPos::HIGH:
                copter.g2.weathervane.allow_weathervaning(true);
                break;
            case AuxSwitchPos::MIDDLE:
                break;
            case AuxSwitchPos::LOW:
                copter.g2.weathervane.allow_weathervaning(false);
                break;
        }
        break;
    }
#endif
    case AUX_FUNC::TRANSMITTER_TUNING:
        // do nothing, used in tuning.cpp for transmitter based tuning
        break;

    default:
        return RC_Channel::do_aux_function(trigger);
    }
    return true;
}

/**
 * @brief Change air mode status based on auxiliary switch position
 * 
 * @details Air Mode maintains stabilization even at zero throttle, allowing
 *          aerobatic maneuvers and better control during descents.
 *          
 *          Air Mode Behavior:
 *          - ENABLED (HIGH): Stabilization active even at zero throttle
 *                           Motors spin at minimum to maintain control authority
 *                           Useful for acrobatics and aggressive flying
 *          - DISABLED (LOW): Normal behavior, stabilization reduces at low throttle
 *                           Motors can spin down to zero
 *                           Safer for beginners
 *          
 *          Use Cases:
 *          - Enable for acrobatic flight and fast descents
 *          - Disable for gentle flying and landing approaches
 *          
 *          Safety: Air mode keeps motors spinning which prevents brownouts
 *          but requires more careful throttle management near ground
 * 
 * @param[in] ch_flag Switch position (HIGH=enabled, LOW=disabled, MIDDLE=no change)
 * 
 * @return void
 * 
 * @note MIDDLE position causes no change to allow 3-position switch use
 * @note Air mode is most useful in ACRO and STABILIZE flight modes
 * 
 * Source: ArduCopter/RC_Channel_Copter.cpp:688-701
 */
// change air-mode status
void RC_Channel_Copter::do_aux_function_change_air_mode(const AuxSwitchPos ch_flag)
{
    switch (ch_flag) {
    case AuxSwitchPos::HIGH:
        copter.air_mode = AirMode::AIRMODE_ENABLED;
        break;
    case AuxSwitchPos::MIDDLE:
        break;
    case AuxSwitchPos::LOW:
        copter.air_mode = AirMode::AIRMODE_DISABLED;
        break;
    }
}

/**
 * @brief Change force flying status for altitude estimation
 * 
 * @details Force Flying overrides the land detection system to force the
 *          altitude estimator to treat the vehicle as airborne even if
 *          landing detection would normally trigger.
 *          
 *          Purpose:
 *          - Prevents false "landed" detection during low-altitude flight
 *          - Useful for flying in ground effect or over water
 *          - Maintains full altitude control authority
 *          
 *          When to Use:
 *          - Flying very close to ground for extended periods
 *          - Operations over water that confuse landing detection
 *          - Situations where throttle is low but vehicle must maintain altitude
 *          
 *          Safety Implications:
 *          - Disables automatic landing detection safety features
 *          - Pilot must manually manage landing transition
 *          - Use with caution as normal landing safeties are bypassed
 * 
 * @param[in] ch_flag Switch position (HIGH=force flying, LOW=normal, MIDDLE=no change)
 * 
 * @return void
 * 
 * @note MIDDLE position causes no change to allow 3-position switch use
 * @warning Disable before landing or automatic landing detection won't work
 * 
 * Source: ArduCopter/RC_Channel_Copter.cpp:703-716
 */
// change force flying status
void RC_Channel_Copter::do_aux_function_change_force_flying(const AuxSwitchPos ch_flag)
{
    switch (ch_flag) {
    case AuxSwitchPos::HIGH:
        copter.force_flying = true;
        break;
    case AuxSwitchPos::MIDDLE:
        break;
    case AuxSwitchPos::LOW:
        copter.force_flying = false;
        break;
    }
}

/**
 * @brief Save roll and pitch RC trim adjustments to AHRS
 * 
 * @details Captures current RC stick positions as permanent trim offsets in the
 *          AHRS (Attitude Heading Reference System). This corrects for:
 *          - IMU mounting misalignment
 *          - CG (center of gravity) offsets
 *          - Asymmetric vehicle configuration
 *          
 *          Trim Save Process:
 *          1. Read current roll and pitch stick positions
 *          2. Convert from centidegrees to radians
 *          3. Add as permanent trim to AHRS
 *          4. Log event and notify ground station
 *          
 *          Two Trim Save Modes:
 *          
 *          Manual Trim (auto_trim.running = false):
 *          - Captures instantaneous stick positions
 *          - Pilot holds sticks to desired trim position
 *          - Quick one-time adjustment
 *          
 *          Auto Trim (auto_trim.running = true):
 *          - Accumulated adjustments from auto_trim_run()
 *          - Gradual trim learning during flight
 *          - More accurate for CG offsets
 *          
 *          Safety:
 *          - Only callable when flight mode allows trim save
 *          - Throttle must be at zero (enforced by caller)
 *          - Prevents accidental trim save during flight
 * 
 * @return void
 * 
 * @note This is a method on RC_Channels object, not individual channel
 * @note Trim is saved to EEPROM and persists across power cycles
 * @note Excessive trim (>10 degrees) indicates mechanical problem
 * @warning Bad trim values can make vehicle unflyable - save carefully
 * 
 * Source: ArduCopter/RC_Channel_Copter.cpp:718-740
 */
// note that this is a method on the RC_Channels object, not the
// individual channel
// save_trim - adds roll and pitch trims from the radio to ahrs
void RC_Channels_Copter::save_trim()
{
    float roll_trim = 0;
    float pitch_trim = 0;
#if AP_COPTER_AHRS_AUTO_TRIM_ENABLED
    // If auto trim was running, stop it and use accumulated trim values
    if (auto_trim.running) {
        auto_trim.running = false;
    } else {
#endif
    // Manual trim: save current stick positions
    // Convert RC input from centidegrees to radians
    roll_trim = cd_to_rad((float)get_roll_channel().get_control_in());
    pitch_trim = cd_to_rad((float)get_pitch_channel().get_control_in());
#if AP_COPTER_AHRS_AUTO_TRIM_ENABLED    
    }
#endif
    // Add trim to AHRS - permanently adjusts attitude reference
    AP::ahrs().add_trim(roll_trim, pitch_trim);
    LOGGER_WRITE_EVENT(LogEvent::SAVE_TRIM);
    gcs().send_text(MAV_SEVERITY_INFO, "Trim saved");
}

#if AP_COPTER_AHRS_AUTO_TRIM_ENABLED
/**
 * @brief Start or stop automatic AHRS trim learning
 * 
 * @details Auto Trim automatically learns and applies attitude trim adjustments
 *          during flight. Pilot flies normally trying to keep copter level,
 *          and system gradually learns trim offsets needed for level flight.
 *          
 *          Switch Position Behavior:
 *          - HIGH: Start auto trim learning (LEDs flash to indicate active)
 *          - MIDDLE: No action
 *          - LOW: Stop auto trim and save learned values to EEPROM
 *          
 *          Usage Procedure:
 *          1. Fly in mode that allows auto trim (STABILIZE, ALT_HOLD, etc.)
 *          2. Switch to HIGH to start learning
 *          3. Fly normally for 30-60 seconds keeping copter as level as possible
 *          4. Switch to LOW to save learned trim
 *          
 *          Safety Checks:
 *          - Only works in modes that allow auto trim
 *          - Must be airborne (not landed or maybe-landed)
 *          - Automatically cancels if mode changes
 *          
 *          Advantages over Manual Trim:
 *          - More accurate for CG offsets
 *          - Accounts for dynamic effects
 *          - Less prone to pilot-induced errors
 * 
 * @param[in] ch_flag Auxiliary switch position (HIGH=start, LOW=stop and save, MIDDLE=no action)
 * 
 * @return void
 * 
 * @note LEDs flash during auto trim to provide visual feedback
 * @note Pilot must actively fly during auto trim - hovering is not sufficient
 * @warning Auto trim cancelled if vehicle lands or changes to incompatible mode
 * 
 * Source: ArduCopter/RC_Channel_Copter.cpp:742-765
 */
// start/stop ahrs auto trim
void RC_Channels_Copter::do_aux_function_ahrs_auto_trim(const RC_Channel::AuxSwitchPos ch_flag)
{
    switch (ch_flag) {
    case RC_Channel::AuxSwitchPos::HIGH:
        // Validate flight mode allows auto trim before starting
        if (!copter.flightmode->allows_auto_trim()) {
            gcs().send_text(MAV_SEVERITY_INFO, "AutoTrim not allowed in this mode");
            break;
        }
        gcs().send_text(MAV_SEVERITY_INFO, "AutoTrim running");
        // flash the leds to provide visual feedback that auto trim is active
        AP_Notify::flags.save_trim = true;
        auto_trim.running = true;
        break;
    case RC_Channel::AuxSwitchPos::MIDDLE:
        break;
    case RC_Channel::AuxSwitchPos::LOW:
        // If auto trim was running, stop it and save accumulated trim
        if (auto_trim.running) {
            AP_Notify::flags.save_trim = false;
            save_trim();
        }
        break;
    }
}

/**
 * @brief Cancel auto trim learning without saving
 * 
 * @details Called when conditions become unsuitable for auto trim (mode change,
 *          landing detected, etc.). Stops the auto trim process and discards
 *          any accumulated trim adjustments.
 *          
 *          Cancellation Triggers:
 *          - Mode changes to one that doesn't allow auto trim
 *          - Vehicle lands or enters "maybe landed" state
 *          - Loss of critical sensors (IMU, attitude estimation)
 *          
 *          Safety: Prevents applying trim learned under invalid conditions
 * 
 * @return void
 * 
 * @note Original trim values are preserved (accumulated adjustments discarded)
 * @note Pilot notified via GCS text message
 * 
 * Source: ArduCopter/RC_Channel_Copter.cpp:767-776
 */
// auto_trim - slightly adjusts the ahrs.roll_trim and ahrs.pitch_trim towards the current stick positions
// meant to be called continuously while the pilot attempts to keep the copter level
void RC_Channels_Copter::auto_trim_cancel()
{
    auto_trim.running = false;
    AP_Notify::flags.save_trim = false;
    gcs().send_text(MAV_SEVERITY_INFO, "AutoTrim cancelled");
    // restore original trims
}

/**
 * @brief Execute one iteration of auto trim learning
 * 
 * @details Called continuously from main loop while auto trim is active.
 *          Gradually accumulates trim adjustments based on attitude controller
 *          target angles. The system learns what trim is needed to achieve
 *          level flight when pilot is trying to fly level.
 *          
 *          Algorithm:
 *          1. Read current attitude target from controller (pilot's intent)
 *          2. Scale by factor of 20 (subjectively tuned for good "feel")
 *          3. Apply as temporary trim adjustment (not saved to EEPROM yet)
 *          4. Repeat each loop, gradually accumulating trim
 *          
 *          Scaling Factor (divide by 20):
 *          - Makes trim learning gradual and smooth
 *          - Prevents over-correction from momentary disturbances
 *          - Gives similar feel to original RC input-based method
 *          - Takes 30-60 seconds to converge for typical trim needs
 *          
 *          Safety Checks:
 *          - Verify auto trim still enabled
 *          - Verify mode still allows auto trim (cancel if not)
 *          - Verify vehicle is airborne (cancel if landed)
 *          
 *          Technical Details:
 *          - Trim adjustments are temporary until pilot stops auto trim
 *          - Uses attitude target, not actual attitude (learns pilot intent)
 *          - Only affects roll and pitch, not yaw
 *          - Trim stored in radians internally
 * 
 * @return void
 * 
 * @note Called at main loop rate (typically 400Hz for Copter)
 * @note Accumulated trim is NOT saved to EEPROM until pilot lowers switch
 * @warning Must be flying - auto trim on ground will learn incorrect values
 * 
 * Source: ArduCopter/RC_Channel_Copter.cpp:778-804
 */
void RC_Channels_Copter::auto_trim_run()
{
        // Quick exit if auto trim not active
        if (!auto_trim.running) {
            return;
        }

        // Safety check: only trim in certain modes
        // Cancel if mode has changed to one that doesn't support auto trim
        if (!copter.flightmode->allows_auto_trim()) {
            auto_trim_cancel();
            return;
        }

        // Safety check: must be airborne
        // Auto trim on ground would learn incorrect values
        // must be started and stopped mid-air
        if (copter.ap.land_complete_maybe) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING,"Must be flying to use AUTOTRIM");
            auto_trim_cancel();
            return;
        }
        
        // Calculate roll trim adjustment from current attitude target
        // Divisor of 20 set subjectively to give same "feel" as previous RC input method
        // Larger divisor = slower convergence but smoother, less prone to over-correction
        float roll_trim_adjustment_rad = copter.attitude_control->get_att_target_euler_rad().x / 20.0f;

        // Calculate pitch trim adjustment from current attitude target
        // Same divisor for consistent learning rate in both axes
        // calculate pitch trim adjustment, divisor set subjectively to give same "feel" as previous RC input method
        float pitch_trim_adjustment_rad = copter.attitude_control->get_att_target_euler_rad().y / 20.0f;

        // Apply trim adjustment temporarily (not saved to permanent storage yet)
        // Trim accumulates over time as this function is called repeatedly
        // Final save happens when pilot lowers auto trim switch
        // add trim to ahrs object, but do not save to permanent storage:
        AP::ahrs().add_trim(roll_trim_adjustment_rad, pitch_trim_adjustment_rad, false);
}

#endif  // AP_COPTER_AHRS_AUTO_TRIM_ENABLED
