/**
 * @file RC_Channel_Rover.cpp
 * @brief Implementation of rover-specific RC channel and auxiliary function handling
 * 
 * @details This file implements the Rover vehicle type specialization of the RC_Channel
 *          and RC_Channels classes, providing rover-specific behavior for:
 *          - Flight mode (driving mode) switching via RC channel
 *          - Auxiliary function initialization and handling for rover-specific features
 *          - RC failsafe detection for ground vehicles
 *          - Arming channel configuration
 * 
 *          The RC channel system allows pilots to control rover driving modes, trigger
 *          special functions (waypoint saving, cruise speed learning, sailboat controls),
 *          and switch between manual and autonomous control modes via RC transmitter.
 * 
 *          Rover-specific auxiliary functions include:
 *          - SAVE_WP: Save waypoints during manual operation
 *          - LEARN_CRUISE: Learn optimal cruise speed and throttle
 *          - Driving mode switches: MANUAL, ACRO, STEERING, HOLD, AUTO, RTL, etc.
 *          - Sailboat controls: SAILBOAT_TACK, SAILBOAT_MOTOR_3POS
 *          - TRIM_TO_CURRENT_SERVO_RC: Save steering trim for Ackermann steering
 * 
 * @note This file uses the RC_Channels framework's parameter information system
 *       via macro-based inclusion of RC_Channels_VarInfo.h
 * 
 * @see RC_Channel.h for base class documentation
 * @see Rover.h for main rover vehicle class
 * @see mode.h for rover driving mode implementations
 */

#include "Rover.h"

#include "RC_Channel_Rover.h"

// defining these two macros and including the RC_Channels_VarInfo
// header defines the parameter information common to all vehicle
// types
#define RC_CHANNELS_SUBCLASS RC_Channels_Rover
#define RC_CHANNEL_SUBCLASS RC_Channel_Rover

#include <RC_Channel/RC_Channels_VarInfo.h>

/**
 * @brief Get the RC channel number used for flight mode (driving mode) selection
 * 
 * @details Returns the RC channel configured for mode switching, allowing the pilot
 *          to select different driving modes via a switch on the RC transmitter.
 *          This channel typically uses a 6-position switch mapped to different rover modes.
 * 
 * @return int8_t RC channel number (1-based), or 0 if no mode channel configured
 * 
 * @note Channel number stored in rover.g.mode_channel parameter (MODE_CH)
 * @see mode_switch_changed for mode change handling
 */
int8_t RC_Channels_Rover::flight_mode_channel_number() const
{
    return rover.g.mode_channel;
}

/**
 * @brief Handle mode switch position changes from RC transmitter
 * 
 * @details This function is called when the RC mode selection switch changes position.
 *          It reads the switch position (typically from a 6-position switch mapped to
 *          PWM ranges) and changes the rover's driving mode accordingly.
 * 
 *          The PWM input from the RC receiver is converted to a discrete position
 *          by the base RC_Channel class (read_mode_switch/read_6pos_switch), which
 *          maps PWM ranges (typically 1000-2000μs) to position indices (0-5).
 * 
 *          Mode mapping is configured via MODEx parameters where x is the position:
 *          - Position 0 (low PWM): MODE1 parameter
 *          - Position 1: MODE2 parameter
 *          - Position 2: MODE3 parameter
 *          - Position 3: MODE4 parameter
 *          - Position 4: MODE5 parameter
 *          - Position 5 (high PWM): MODE6 parameter
 * 
 * @param[in] new_pos Switch position index (0-5 for 6-position switch)
 * 
 * @note Called automatically by RC_Channel framework when mode switch PWM changes
 * @note Mode change only occurs if new_pos is valid (0 <= new_pos <= num_modes)
 * @note Mode change reason is logged as RC_COMMAND for telemetry/logging
 * 
 * @see read_mode_switch in RC_Channel base class for PWM to position conversion
 * @see rover.set_mode for actual mode transition logic
 */
void RC_Channel_Rover::mode_switch_changed(modeswitch_pos_t new_pos)
{
    // Validate switch position is within configured mode array bounds
    if (new_pos < 0 || (uint8_t)new_pos > rover.num_modes) {
        // should not have been called
        return;
    }

    // Change to the mode configured for this switch position
    rover.set_mode((Mode::Number)rover.modes[new_pos].get(), ModeReason::RC_COMMAND);
}

/**
 * @brief Initialize rover-specific auxiliary RC channel functions
 * 
 * @details This function is called during rover initialization to set up auxiliary
 *          RC channel functions to their initial states. Most rover aux functions
 *          are stateless (mode switches, triggers) and require no initialization.
 * 
 *          The only rover aux function requiring initialization is SAILBOAT_MOTOR_3POS,
 *          which sets the initial motor usage state based on the current switch position.
 * 
 *          Auxiliary functions are assigned to RC channels via RCx_OPTION parameters,
 *          allowing pilots to map functions like mode changes, waypoint saving, and
 *          feature toggles to switches on their RC transmitter.
 * 
 * @param[in] ch_option The auxiliary function type being initialized
 * @param[in] ch_flag   Current switch position (LOW/MIDDLE/HIGH) at initialization
 * 
 * @note Called once during rover startup for each configured aux function
 * @note Functions not explicitly handled here delegate to RC_Channel base class
 * @see do_aux_function for runtime auxiliary function handling
 * @see RCx_OPTION parameter documentation for available functions
 */
void RC_Channel_Rover::init_aux_function(const AUX_FUNC ch_option, const AuxSwitchPos ch_flag)
{
    // Initialize rover-specific channel options based on function type
    switch (ch_option) {
    
    // Rover driving mode switches - no initialization needed (stateless triggers)
    case AUX_FUNC::ACRO:         // Acro mode (direct throttle/steering control)
    case AUX_FUNC::AUTO:         // Auto mode (mission execution)
    case AUX_FUNC::CIRCLE:       // Circle mode (drive in circles)
    case AUX_FUNC::FOLLOW:       // Follow mode (follow another vehicle)
    case AUX_FUNC::GUIDED:       // Guided mode (GCS-commanded navigation)
    case AUX_FUNC::HOLD:         // Hold mode (stop and hold position)
    case AUX_FUNC::LOITER:       // Loiter mode (maintain current position)
    case AUX_FUNC::MANUAL:       // Manual mode (direct pilot control)
    case AUX_FUNC::RTL:          // Return to Launch
    case AUX_FUNC::SIMPLE:       // Simple mode (simplified control)
    case AUX_FUNC::SMART_RTL:    // Smart RTL (return via recorded path)
    case AUX_FUNC::STEERING:     // Steering mode (pilot steers, autopilot controls speed)
        
    // Trigger-based functions - no initialization needed (activated on switch change)
    case AUX_FUNC::LEARN_CRUISE:           // Learn cruise speed and throttle
    case AUX_FUNC::SAVE_WP:                // Save current location as waypoint
    case AUX_FUNC::SAILBOAT_TACK:          // Trigger sailboat tack maneuver
    case AUX_FUNC::TRIM_TO_CURRENT_SERVO_RC: // Save steering trim
        
    // Sailboat-specific controls - no initialization needed
    case AUX_FUNC::MAINSAIL:               // Manual mainsail control input
    case AUX_FUNC::WIND_VANE_DIR_OFSSET:   // Wind vane direction offset
        
    // Walking robot controls - no initialization needed
    case AUX_FUNC::PITCH:                  // Manual pitch control for walkers
    case AUX_FUNC::ROLL:                   // Manual roll control for walkers
    case AUX_FUNC::WALKING_HEIGHT:         // Walking height adjustment
        break;
        
    // Sailboat 3-position motor control - requires initialization to set motor state
    case AUX_FUNC::SAILBOAT_MOTOR_3POS:
        // Set initial motor usage state based on current switch position:
        // HIGH = always use motor, MIDDLE = assist only, LOW = never use motor
        do_aux_function_sailboat_motor_3pos(ch_flag);
        break;
        
    // All other functions: delegate to base class (common functions like arming, etc.)
    default:
        RC_Channel::init_aux_function(ch_option, ch_flag);
        break;
    }
}


/**
 * @brief Check if rover is currently in RC failsafe condition
 * 
 * @details Determines if the RC receiver has lost connection with the transmitter
 *          (throttle failsafe triggered). For rovers, this checks the throttle
 *          failsafe bit in the rover's failsafe status.
 * 
 * @return true if in RC failsafe (lost connection), false if RC link healthy
 * 
 * @note Called by RC_Channels framework to determine input validity
 * @see has_valid_input for combined RC health check
 */
bool RC_Channels_Rover::in_rc_failsafe() const
{
    return rover.failsafe.bits & FAILSAFE_EVENT_THROTTLE;
}

/**
 * @brief Check if rover has valid RC input
 * 
 * @details Returns true if RC receiver is connected and providing valid input.
 *          This is used to determine if pilot commands should be accepted.
 * 
 * @return true if RC input is valid, false if in failsafe
 * 
 * @note Used by rover control loops to decide whether to accept pilot input
 */
bool RC_Channels_Rover::has_valid_input() const
{
    if (in_rc_failsafe()) {
        return false;
    }
    return true;
}

/**
 * @brief Get the RC channel used for arming/disarming stick gestures
 * 
 * @details Returns the steering channel, which is used in combination with
 *          throttle for rudder arming (holding steering right/left and throttle
 *          down to arm/disarm). For rovers, steering is the logical equivalent
 *          of copter/plane rudder.
 * 
 * @return RC_Channel* Pointer to the steering channel (typically RC channel 1)
 * 
 * @note Used by arming library to detect stick-arming gestures
 */
RC_Channel * RC_Channels_Rover::get_arming_channel(void) const
{
    return rover.channel_steer;
}

/**
 * @brief Helper to change rover mode based on auxiliary switch position
 * 
 * @details Implements standard 3-position switch behavior for mode changes:
 *          - HIGH position: Switch to specified mode
 *          - MIDDLE position: No action (allows mode to be changed by other means)
 *          - LOW position: If currently in this mode, allow mode switch to control mode
 * 
 *          This pattern allows auxiliary switches to request specific modes without
 *          completely overriding the main mode switch.
 * 
 * @param[in] mode    Reference to the mode to switch to
 * @param[in] ch_flag Current switch position (HIGH/MIDDLE/LOW)
 * 
 * @note Used internally by multiple mode-related auxiliary functions
 * @note Mode changes logged with ModeReason::AUX_FUNCTION
 */
void RC_Channel_Rover::do_aux_function_change_mode(Mode &mode,
        const AuxSwitchPos ch_flag)
{
    switch (ch_flag) {
    case AuxSwitchPos::HIGH:
        // Switch HIGH: Activate this mode
        rover.set_mode(mode, ModeReason::AUX_FUNCTION);
        break;
    case AuxSwitchPos::MIDDLE:
        // Switch MIDDLE: Do nothing, allows other mode control
        break;
    case AuxSwitchPos::LOW:
        // Switch LOW: If we're in this mode, return control to mode switch
        if (rover.control_mode == &mode) {
            rc().reset_mode_switch();
        }
    }
}

/**
 * @brief Add a waypoint at the rover's current location to the mission
 * 
 * @details Creates a new NAV_WAYPOINT mission command at the rover's current
 *          GPS position and appends it to the mission. This allows pilots to
 *          build missions on-the-fly during manual operation by saving waypoints
 *          at interesting locations.
 * 
 *          The waypoint is added to the end of the current mission list and can
 *          be executed later by switching to AUTO mode.
 * 
 * @note Called by SAVE_WP auxiliary function when triggered
 * @note Waypoint added to mission in EEPROM, persists across reboots
 * @note Success/failure message sent to GCS console
 */
void RC_Channel_Rover::add_waypoint_for_current_loc()
{
    // Create new mission command structure
    AP_Mission::Mission_Command cmd = {};

    // Set waypoint location to rover's current GPS position
    cmd.content.location = rover.current_loc;

    // Set command type to navigation waypoint (straight line to point)
    cmd.id = MAV_CMD_NAV_WAYPOINT;

    // Append waypoint to mission and report success
    if (rover.mode_auto.mission.add_cmd(cmd)) {
        hal.console->printf("Added waypoint %u", (unsigned)rover.mode_auto.mission.num_commands());
    }
}

/**
 * @brief Handle 3-position switch for sailboat motor control
 * 
 * @details Sets the sailboat motor usage policy based on switch position:
 *          - HIGH: Always use motor (motor-sailing mode, ignores wind)
 *          - MIDDLE: Use motor to assist sailing (motor helps when wind insufficient)
 *          - LOW: Never use motor (pure sailing mode, motor disabled)
 * 
 *          This allows sailboat operators to quickly change motor policy based on
 *          conditions - using motor in harbors or no-wind situations, sailing-only
 *          in good wind, or assisted mode for optimal performance.
 * 
 * @param[in] ch_flag Switch position (HIGH/MIDDLE/LOW)
 * 
 * @note Only applicable to sailboat rovers (FRAME_TYPE = sailboat)
 * @see Sailboat::UseMotor enum for motor state definitions
 */
void RC_Channel_Rover::do_aux_function_sailboat_motor_3pos(const AuxSwitchPos ch_flag)
{
    switch (ch_flag) {
    case AuxSwitchPos::HIGH:
        // HIGH: Always run motor, ignore wind (motor-sailing mode)
        rover.g2.sailboat.set_motor_state(Sailboat::UseMotor::USE_MOTOR_ALWAYS);
        break;
    case AuxSwitchPos::MIDDLE:
        // MIDDLE: Use motor to assist when wind is insufficient
        rover.g2.sailboat.set_motor_state(Sailboat::UseMotor::USE_MOTOR_ASSIST);
        break;
    case AuxSwitchPos::LOW:
        // LOW: Never use motor, pure sailing mode
        rover.g2.sailboat.set_motor_state(Sailboat::UseMotor::USE_MOTOR_NEVER);
        break;
    }
}

/**
 * @brief Execute rover-specific auxiliary RC channel functions
 * 
 * @details This function is called whenever an auxiliary RC channel changes state
 *          (switch position changes, button pressed, etc.). It handles rover-specific
 *          auxiliary functions including:
 *          
 *          - Driving mode changes (MANUAL, AUTO, HOLD, RTL, etc.)
 *          - Waypoint management (SAVE_WP to add waypoints during operation)
 *          - Cruise speed learning (LEARN_CRUISE to optimize throttle/speed)
 *          - Sailboat controls (tacking, motor control)
 *          - Steering trim adjustment
 *          - Walking robot controls (height, pitch, roll)
 * 
 *          Auxiliary functions are configured via RCx_OPTION parameters, allowing
 *          pilots to assign functions to switches/buttons on their RC transmitter.
 *          This provides flexible control customization for different rover types
 *          and operational needs.
 * 
 * @param[in] trigger Structure containing:
 *                    - func: The auxiliary function being triggered (AUX_FUNC enum)
 *                    - pos: Switch position (HIGH/MIDDLE/LOW) or trigger state
 * 
 * @return true if function was handled, false if should be ignored
 * 
 * @note Called at RC input rate (typically 50Hz) when switch position changes
 * @note Unhandled functions delegated to RC_Channel base class
 * @see init_aux_function for auxiliary function initialization
 * @see RCx_OPTION parameter documentation for available functions
 */
bool RC_Channel_Rover::do_aux_function(const AuxFuncTrigger &trigger)
{
    const AUX_FUNC &ch_option = trigger.func;
    const AuxSwitchPos &ch_flag = trigger.pos;

    switch (ch_option) {
    
    // DO_NOTHING: Placeholder function, no action taken
    case AUX_FUNC::DO_NOTHING:
        break;
    // SAVE_WP: Save current location as waypoint or clear mission
    case AUX_FUNC::SAVE_WP:
        if (ch_flag == AuxSwitchPos::HIGH) {
            // Don't save waypoints while in AUTO mode (mission already running)
            if (rover.control_mode == &rover.mode_auto) {
                break;
            }

            // Special behavior when disarmed: Clear mission and reset home location
            // This allows starting a fresh mission from the current position
            if (!rover.arming.is_armed()) {
                rover.mode_auto.mission.clear();
                GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "SaveWP: Mission cleared!");
                if (!rover.set_home_to_current_location(false)) {
                    // Home setting failed, but continue anyway
                }
                break;
            }
            
            // When armed: Add current location as new waypoint to end of mission
            // This allows building a mission by driving the desired path manually
            add_waypoint_for_current_loc();
        }
        break;

    // LEARN_CRUISE: Learn optimal cruise speed and throttle
    // Samples current speed and throttle over time to determine efficient cruise settings
    case AUX_FUNC::LEARN_CRUISE:
        if (ch_flag == AuxSwitchPos::HIGH) {
            // Start cruise speed learning - rover records speed/throttle relationship
            // Used to set CRUISE_SPEED and CRUISE_THROTTLE parameters automatically
            rover.cruise_learn_start();
        }
        break;

    // MANUAL: Set mode to Manual (direct pilot control of throttle and steering)
    case AUX_FUNC::MANUAL:
        do_aux_function_change_mode(rover.mode_manual, ch_flag);
        break;

    // ACRO: Set mode to Acro (rate-based control for advanced pilots)
    case AUX_FUNC::ACRO:
        do_aux_function_change_mode(rover.mode_acro, ch_flag);
        break;

    // STEERING: Set mode to Steering (pilot steers, autopilot controls speed)
    case AUX_FUNC::STEERING:
        do_aux_function_change_mode(rover.mode_steering, ch_flag);
        break;

    // HOLD: Set mode to Hold (stop and hold current position)
    case AUX_FUNC::HOLD:
        do_aux_function_change_mode(rover.mode_hold, ch_flag);
        break;

    // AUTO: Set mode to Auto (execute mission waypoints)
    case AUX_FUNC::AUTO:
        do_aux_function_change_mode(rover.mode_auto, ch_flag);
        break;

    // RTL: Set mode to Return to Launch (navigate back to home location)
    case AUX_FUNC::RTL:
        do_aux_function_change_mode(rover.mode_rtl, ch_flag);
        break;

    // SMART_RTL: Set mode to Smart RTL (return via recorded path, avoiding obstacles)
    case AUX_FUNC::SMART_RTL:
        do_aux_function_change_mode(rover.mode_smartrtl, ch_flag);
        break;

    // GUIDED: Set mode to Guided (GCS-commanded navigation, e.g., "go to" commands)
    case AUX_FUNC::GUIDED:
        do_aux_function_change_mode(rover.mode_guided, ch_flag);
        break;

    // LOITER: Set mode to Loiter (maintain current position using GPS)
    case AUX_FUNC::LOITER:
        do_aux_function_change_mode(rover.mode_loiter, ch_flag);
        break;

#if MODE_FOLLOW_ENABLED
    // FOLLOW: Set mode to Follow (follow another vehicle using position telemetry)
    case AUX_FUNC::FOLLOW:
        do_aux_function_change_mode(rover.mode_follow, ch_flag);
        break;
#endif

    // SIMPLE: Set mode to Simple (simplified control for beginners)
    case AUX_FUNC::SIMPLE:
        do_aux_function_change_mode(rover.mode_simple, ch_flag);
        break;

    // CIRCLE: Set mode to Circle (drive in circles at current location)
    case AUX_FUNC::CIRCLE:
        do_aux_function_change_mode(rover.g2.mode_circle, ch_flag);
        break;

    // SAILBOAT_TACK: Trigger sailboat tack maneuver
    // Initiates a tack (change direction through the wind) for sailboat rovers
    case AUX_FUNC::SAILBOAT_TACK:
        // Any switch movement interpreted as request to tack
        // Current mode handles tack execution (different behavior in AUTO vs MANUAL)
        rover.control_mode->handle_tack_request();
        break;

    // SAILBOAT_MOTOR_3POS: 3-position switch for sailboat motor control
    // HIGH = always use motor, MIDDLE = assist mode, LOW = sailing only
    case AUX_FUNC::SAILBOAT_MOTOR_3POS:
        do_aux_function_sailboat_motor_3pos(ch_flag);
        break;

    // TRIM_TO_CURRENT_SERVO_RC: Save current steering position as trim
    // Allows centering steering trim for Ackermann rovers while driving straight
    case AUX_FUNC::TRIM_TO_CURRENT_SERVO_RC:
        // Only allow trim adjustment for Ackermann steering (not skid-steer)
        // Must be armed and not in position-hold modes (to ensure vehicle is moving)
        if (!rover.g2.motors.have_skid_steering() && rover.arming.is_armed() &&
            (rover.control_mode != &rover.mode_loiter)
            && (rover.control_mode != &rover.mode_hold) && ch_flag == AuxSwitchPos::HIGH) {
            // Save current servo output as new trim point (centers steering)
            SRV_Channels::set_trim_to_servo_out_for(SRV_Channel::k_steering);
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Steering trim saved!");
        }
        break;

    // Manual input pass-through functions - no special handling needed
    // These provide direct RC input to specific outputs for manual control
    case AUX_FUNC::MAINSAIL:              // Direct mainsail servo control (sailboats)
    case AUX_FUNC::PITCH:                 // Direct pitch control (walking robots)
    case AUX_FUNC::ROLL:                  // Direct roll control (walking robots)
    case AUX_FUNC::WALKING_HEIGHT:        // Walking robot height adjustment
    case AUX_FUNC::WIND_VANE_DIR_OFSSET:  // Wind vane direction offset (sailboats)
        break;

    // All other functions: delegate to base RC_Channel class
    // Handles common functions like arming, camera trigger, etc.
    default:
        return RC_Channel::do_aux_function(trigger);

    }

    return true;
}
