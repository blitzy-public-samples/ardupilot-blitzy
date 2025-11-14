/**
 * @file events.cpp
 * @brief Event handling for failsafe conditions in fixed-wing aircraft
 * 
 * @details This file implements the ArduPlane failsafe event handling system,
 *          managing critical safety responses to control signal loss, battery
 *          depletion, and GCS communication failures. The failsafe system provides
 *          graduated responses based on failure duration and severity to ensure
 *          safe vehicle operation under all conditions.
 * 
 *          Key Failsafe Types:
 *          - Short Failsafe: Temporary RC signal loss (typically <3 seconds)
 *          - Long Failsafe: Extended RC or GCS signal loss (>3 seconds)
 *          - Battery Failsafe: Low battery voltage or capacity
 *          - GCS Failsafe: Ground Control Station communication loss
 * 
 *          The failsafe system is designed with flight safety as the primary concern,
 *          prioritizing safe landing or return-to-launch over mission completion.
 * 
 * @note All failsafe handlers are called from the main scheduler loop
 * @warning Modifications to failsafe logic can affect flight safety and should
 *          be thoroughly tested in SITL before hardware deployment
 * 
 * @see Plane::read_radio() for failsafe detection
 * @see Plane::set_mode() for mode transition implementation
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#include "Plane.h"

/**
 * @brief Check if vehicle is currently in a landing sequence
 * 
 * @details This helper function determines whether the aircraft is actively
 *          executing a landing maneuver in any form. It checks multiple sources:
 *          - Fixed-wing landing flight stage
 *          - QuadPlane VTOL landing sequence
 *          - Mission landing sequence flag
 * 
 *          This function is specifically intended for use in failsafe code to
 *          prevent interrupting a committed landing attempt, which could be
 *          more dangerous than allowing the landing to complete.
 * 
 * @return true if aircraft is in any landing sequence, false otherwise
 * 
 * @note This is a const method - does not modify vehicle state
 * @warning Do not use this for general mode checking; it's specifically for
 *          failsafe logic where landing must not be interrupted
 * 
 * @see flight_stage for fixed-wing flight phases
 * @see quadplane.in_vtol_land_sequence() for VTOL landing detection
 * @see mission.get_in_landing_sequence_flag() for mission landing status
 */
bool Plane::failsafe_in_landing_sequence() const
{
    if (flight_stage == AP_FixedWing::FlightStage::LAND) {
        return true;
    }
#if HAL_QUADPLANE_ENABLED
    if (quadplane.in_vtol_land_sequence()) {
        return true;
    }
#endif
    if (mission.get_in_landing_sequence_flag()) {
        return true;
    }
    return false;
}

/**
 * @brief Handle short-term RC signal loss failsafe event
 * 
 * @details This function implements the immediate response to a temporary loss
 *          of RC control signal, typically triggered after ~0.5 seconds of signal
 *          loss. The short failsafe provides a quick recovery mechanism for brief
 *          radio glitches while maintaining aircraft control.
 * 
 *          Response Strategy by Mode:
 *          - Manual/Stabilize/Acro/FBWA/FBWB/Cruise/Training/Autotune:
 *            * Transitions to FBWA, FBWB, or CIRCLE based on FS_SHORT_ACTN parameter
 *            * Emergency landing switch overrides to force FBWA
 *          
 *          - QuadPlane Manual Modes (QSTABILIZE/QHOVER/QLOITER/QACRO/QAUTOTUNE):
 *            * Transitions to QLAND, QRTL, or RTL based on Q_OPTIONS
 *          
 *          - Auto/Guided/Loiter/Thermal/AVOID_ADSB:
 *            * Action depends on FS_SHORT_ACTN parameter
 *            * If FS_SHORT_ACTN=0 (BESTGUESS), no mode change occurs
 *            * If in landing sequence, no action taken
 *          
 *          - Circle/Takeoff/RTL/QLAND/QRTL:
 *            * No action - these modes continue uninterrupted
 * 
 *          The saved mode is stored to enable recovery when signal returns.
 * 
 * @param[in] fstype Failsafe state type (FAILSAFE_SHORT_RC, etc.)
 * @param[in] reason Mode change reason for logging and tracking
 * 
 * @note Called from radio.cpp when RC signal loss is detected
 * @note Stores current mode in failsafe.saved_mode_number for recovery
 * @note Sends MAVLink notification to ground station
 * 
 * @warning This is a safety-critical function - all mode transitions must
 *          be safe for immediate execution without pilot input
 * @warning Emergency landing switch (plane.emergency_landing) overrides normal
 *          failsafe behavior to allow out-of-range landing attempts
 * 
 * @see failsafe_short_off_event() for recovery when signal returns
 * @see g.fs_action_short parameter for configurable failsafe action
 * @see FS_ACTION_SHORT_BESTGUESS, FS_ACTION_SHORT_FBWA, FS_ACTION_SHORT_FBWB
 */
void Plane::failsafe_short_on_event(enum failsafe_state fstype, ModeReason reason)
{
    // Update failsafe state and save current mode for potential recovery
    failsafe.state = fstype;
    failsafe.short_timer_ms = millis();
    failsafe.saved_mode_number = control_mode->mode_number();
    
    // Determine appropriate failsafe action based on current flight mode
    switch (control_mode->mode_number())
    {
    case Mode::Number::MANUAL:
    case Mode::Number::STABILIZE:
    case Mode::Number::ACRO:
    case Mode::Number::FLY_BY_WIRE_A:
    case Mode::Number::AUTOTUNE:
    case Mode::Number::FLY_BY_WIRE_B:
    case Mode::Number::CRUISE:
    case Mode::Number::TRAINING:
        // Manual and stabilized modes: Switch to stabilized autonomous mode for safety
        if(plane.emergency_landing) {
            // Emergency landing switch forces FBWA to allow pilot-initiated out-of-range landing
            set_mode(mode_fbwa, reason);
            break;
        }
        // Execute configured short failsafe action
        if(g.fs_action_short == FS_ACTION_SHORT_FBWA) {
            set_mode(mode_fbwa, reason);
        } else if (g.fs_action_short == FS_ACTION_SHORT_FBWB) {
            set_mode(mode_fbwb, reason);
        } else {
            set_mode(mode_circle, reason); // Default: circle if action = 0 (BESTGUESS) or 1 (CIRCLE)
        }
        break;

#if HAL_QUADPLANE_ENABLED
    // QuadPlane manual modes: Always take action to prevent uncontrolled descent
    case Mode::Number::QSTABILIZE:
    case Mode::Number::QLOITER:
    case Mode::Number::QHOVER:
#if QAUTOTUNE_ENABLED
    case Mode::Number::QAUTOTUNE:
#endif
    case Mode::Number::QACRO:
        // QuadPlane failsafe priority: RTL > QRTL > QLAND based on Q_OPTIONS bitmask
        if (quadplane.option_is_set(QuadPlane::OPTION::FS_RTL)) {
            set_mode(mode_rtl, reason);
        } else if (quadplane.option_is_set(QuadPlane::OPTION::FS_QRTL)) {
            set_mode(mode_qrtl, reason);
        } else {
            set_mode(mode_qland, reason);
        }
        break;
#endif // HAL_QUADPLANE_ENABLED

    // Autonomous modes: Landing sequence takes priority, otherwise configurable action
    case Mode::Number::AUTO:
#if MODE_AUTOLAND_ENABLED
    case Mode::Number::AUTOLAND:
#endif
        {
        if (failsafe_in_landing_sequence()) {
            // CRITICAL: Never interrupt a committed landing - more dangerous than signal loss
            break;
        }
        // Not landing, treat like other autonomous modes
        FALLTHROUGH;
    }
    case Mode::Number::AVOID_ADSB:
    case Mode::Number::GUIDED:
    case Mode::Number::LOITER:
    case Mode::Number::THERMAL:
        // Autonomous modes: Only take action if FS_SHORT_ACTN != 0 (BESTGUESS)
        if (g.fs_action_short != FS_ACTION_SHORT_BESTGUESS) {
            failsafe.saved_mode_number = control_mode->mode_number();
            if (g.fs_action_short == FS_ACTION_SHORT_FBWA) {
                set_mode(mode_fbwa, reason);
            } else if (g.fs_action_short == FS_ACTION_SHORT_FBWB) {
                set_mode(mode_fbwb, reason);
            } else {
                set_mode(mode_circle, reason);
            }
        }
         break;
    case Mode::Number::CIRCLE:  // these modes never take any short failsafe action and continue
    case Mode::Number::TAKEOFF:
    case Mode::Number::RTL:
#if HAL_QUADPLANE_ENABLED
    case Mode::Number::QLAND:
    case Mode::Number::QRTL:
    case Mode::Number::LOITER_ALT_QLAND:
#endif
    case Mode::Number::INITIALISING:
        break;
    }
    if (failsafe.saved_mode_number != control_mode->mode_number()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "RC Short Failsafe: switched to %s", control_mode->name());
    } else {
        gcs().send_text(MAV_SEVERITY_WARNING, "RC Short Failsafe On");
    }
}

/**
 * @brief Handle extended RC or GCS signal loss failsafe event
 * 
 * @details This function implements the response to prolonged loss of control
 *          signal (typically >3 seconds), representing a more serious failure
 *          than short failsafe. The long failsafe assumes control may not return
 *          and takes more aggressive action to ensure vehicle safety, typically
 *          initiating return-to-launch or landing procedures.
 * 
 *          Critical Actions:
 *          - Clears all GCS RC overrides to allow RC control if available
 *          - Updates failsafe state for system tracking
 *          - Executes configured long failsafe action
 * 
 *          Response Strategy by Mode:
 *          - Manual/Stabilize/Acro/FBWA/FBWB/Cruise/Training/Circle/Loiter/Thermal:
 *            * During takeoff: May defer action until clear of ground (except GLIDE/PARACHUTE)
 *            * Emergency landing: Forces FBWA mode
 *            * Normal operation: Executes FS_LONG_ACTN action:
 *              - FS_ACTION_LONG_PARACHUTE: Deploys parachute
 *              - FS_ACTION_LONG_GLIDE: Transitions to FBWA
 *              - FS_ACTION_LONG_AUTO: Returns to AUTO mission
 *              - FS_ACTION_LONG_AUTOLAND: Attempts AUTOLAND, falls back to RTL
 *              - Default: Return-to-launch (RTL)
 *          
 *          - QuadPlane Manual Modes (QSTABILIZE/QHOVER/QLOITER/QACRO/QAUTOTUNE):
 *            * Transitions to QLAND, QRTL, or RTL based on Q_OPTIONS bitmask
 *          
 *          - AUTO Mode:
 *            * No action if in landing sequence (committed landing continues)
 *            * VTOL takeoff: Transitions to QLAND
 *            * Otherwise: Executes configured FS_LONG_ACTN action
 *          
 *          - GUIDED/AVOID_ADSB:
 *            * Executes configured FS_LONG_ACTN action
 *          
 *          - RTL Mode:
 *            * May switch to AUTO or AUTOLAND based on FS_LONG_ACTN
 *          
 *          - QLAND/QRTL/AUTOLAND/INITIALISING:
 *            * No action - these modes continue
 * 
 * @param[in] fstype Failsafe state type (FAILSAFE_LONG_RC or FAILSAFE_GCS)
 * @param[in] reason Mode change reason (RADIO_FAILSAFE or GCS_FAILSAFE)
 * 
 * @note Called from radio.cpp or GCS handling code after extended signal loss
 * @note Distinguishes between RC failsafe and GCS failsafe via reason parameter
 * @note Sends MAVLink notification indicating failsafe type and resulting mode
 * @note Uses long_failsafe_pending flag to defer action during critical takeoff phase
 * 
 * @warning This is a safety-critical function affecting autonomous vehicle behavior
 * @warning Parachute deployment (FS_ACTION_LONG_PARACHUTE) is irreversible
 * @warning During takeoff initial climb, failsafe action may be deferred to prevent
 *          ground collision, except for PARACHUTE or GLIDE actions
 * @warning Emergency landing switch overrides normal action for out-of-range landings
 * 
 * @see failsafe_long_off_event() for recovery when signal returns
 * @see g.fs_action_long parameter for configurable long failsafe action
 * @see RC_Channels::clear_overrides() to restore RC control authority
 * @see FS_ACTION_LONG_RTL, FS_ACTION_LONG_AUTO, FS_ACTION_LONG_GLIDE, etc.
 */
void Plane::failsafe_long_on_event(enum failsafe_state fstype, ModeReason reason)
{
    // Clear any GCS RC overrides to allow direct RC control if available
    // This is critical if GCS has locked up - pilot can still control via RC
    RC_Channels::clear_overrides();
    
    // Update failsafe state for system tracking
    failsafe.state = fstype;
    
    // Determine appropriate long failsafe action based on current flight mode
    switch (control_mode->mode_number())
    {
    case Mode::Number::MANUAL:
    case Mode::Number::STABILIZE:
    case Mode::Number::ACRO:
    case Mode::Number::FLY_BY_WIRE_A:
    case Mode::Number::AUTOTUNE:
    case Mode::Number::FLY_BY_WIRE_B:
    case Mode::Number::CRUISE:
    case Mode::Number::TRAINING:
    case Mode::Number::CIRCLE:
    case Mode::Number::LOITER:
    case Mode::Number::THERMAL:
    case Mode::Number::TAKEOFF:
        // CRITICAL SAFETY: Defer failsafe during initial takeoff climb to prevent ground collision
        // Exception: GLIDE and PARACHUTE actions execute immediately as they handle low altitude
        if (plane.flight_stage == AP_FixedWing::FlightStage::TAKEOFF && 
            !(g.fs_action_long == FS_ACTION_LONG_GLIDE || g.fs_action_long == FS_ACTION_LONG_PARACHUTE)) {
            // Set pending flag - failsafe will be re-evaluated after clearing takeoff phase
            long_failsafe_pending = true;
            break;
        }

        if(plane.emergency_landing) {
            // Emergency landing switch forces FBWA for pilot-controlled out-of-range landing
            set_mode(mode_fbwa, reason);
            break;
        }
        if(g.fs_action_long == FS_ACTION_LONG_PARACHUTE) {
#if HAL_PARACHUTE_ENABLED
            parachute_release();
#endif
        } else if (g.fs_action_long == FS_ACTION_LONG_GLIDE) {
            set_mode(mode_fbwa, reason);
        } else if (g.fs_action_long == FS_ACTION_LONG_AUTO) {
            set_mode(mode_auto, reason);
#if MODE_AUTOLAND_ENABLED
        } else if (g.fs_action_long == FS_ACTION_LONG_AUTOLAND) {
            if (!set_mode(mode_autoland, reason)) {
               set_mode(mode_rtl, reason);
            }
#endif
        } else {
            set_mode(mode_rtl, reason);
        }
        break;

#if HAL_QUADPLANE_ENABLED
    case Mode::Number::QSTABILIZE:
    case Mode::Number::QHOVER:
    case Mode::Number::QLOITER:
    case Mode::Number::QACRO:
#if QAUTOTUNE_ENABLED
    case Mode::Number::QAUTOTUNE:
#endif
        if (quadplane.option_is_set(QuadPlane::OPTION::FS_RTL)) {
            set_mode(mode_rtl, reason);
        } else if (quadplane.option_is_set(QuadPlane::OPTION::FS_QRTL)) {
            set_mode(mode_qrtl, reason);
        } else {
            set_mode(mode_qland, reason);
        }
        break;
#endif  // HAL_QUADPLANE_ENABLED

    case Mode::Number::AUTO:
        if (failsafe_in_landing_sequence()) {
            // don't failsafe in a landing sequence
            break;
        }

#if HAL_QUADPLANE_ENABLED
        if (quadplane.in_vtol_takeoff()) {
            set_mode(mode_qland, reason);
            // QLAND if in VTOL takeoff
            break;
        }
#endif
        FALLTHROUGH;

    case Mode::Number::AVOID_ADSB:
    case Mode::Number::GUIDED:

        if(g.fs_action_long == FS_ACTION_LONG_PARACHUTE) {
#if HAL_PARACHUTE_ENABLED
            parachute_release();
#endif
        } else if (g.fs_action_long == FS_ACTION_LONG_GLIDE) {
            set_mode(mode_fbwa, reason);
        } else if (g.fs_action_long == FS_ACTION_LONG_AUTO) {
            set_mode(mode_auto, reason);
#if MODE_AUTOLAND_ENABLED
        } else if (g.fs_action_long == FS_ACTION_LONG_AUTOLAND) {
            if (!set_mode(mode_autoland, reason)) {
               set_mode(mode_rtl, reason);
            } 
#endif           
        } else if (g.fs_action_long == FS_ACTION_LONG_RTL) {
            set_mode(mode_rtl, reason);
        }
        break;
    case Mode::Number::RTL:
        if (g.fs_action_long == FS_ACTION_LONG_AUTO) {
            set_mode(mode_auto, reason);
#if MODE_AUTOLAND_ENABLED
        } else if (g.fs_action_long == FS_ACTION_LONG_AUTOLAND) {
            set_mode(mode_autoland, reason);
#endif
        }
        break;
#if HAL_QUADPLANE_ENABLED
    case Mode::Number::QLAND:
    case Mode::Number::QRTL:
    case Mode::Number::LOITER_ALT_QLAND:
#endif
    case Mode::Number::INITIALISING:
#if MODE_AUTOLAND_ENABLED
    case Mode::Number::AUTOLAND:
#endif
        break;
    }
    gcs().send_text(MAV_SEVERITY_WARNING, "%s Failsafe On: %s", (reason == ModeReason:: GCS_FAILSAFE) ? "GCS" : "RC Long", control_mode->name());
}

/**
 * @brief Handle recovery from short-term RC signal loss
 * 
 * @details This function is called when RC signal is restored after a short
 *          failsafe event. It attempts to restore the original flight mode
 *          that was active before the failsafe was triggered, providing smooth
 *          recovery for transient radio glitches.
 * 
 *          Recovery Logic:
 *          1. Clears failsafe state flag
 *          2. Checks if current mode is still due to short failsafe
 *          3. If yes, restores the saved mode from before failsafe
 *          4. Notifies pilot and GCS of recovery
 * 
 *          Mode Restoration Conditions:
 *          - Only restores if control_mode_reason == RADIO_FAILSAFE
 *          - This prevents restoration if pilot manually changed modes during failsafe
 *          - Prevents restoration if a long failsafe has since occurred
 * 
 * @param[in] reason Recovery reason (typically RADIO_FAILSAFE_RECOVERY)
 * 
 * @note Called from radio.cpp when RC signal is restored
 * @note Uses failsafe.saved_mode_number stored by failsafe_short_on_event()
 * @note Sends MAVLink notifications for pilot awareness
 * 
 * @warning Only restores mode if no manual intervention occurred during failsafe
 * @warning Does not restore if mode was changed by pilot or other failsafe
 * 
 * @see failsafe_short_on_event() which saves the original mode
 * @see set_mode_by_number() for mode restoration
 * @see control_mode_reason for tracking why current mode is active
 */
void Plane::failsafe_short_off_event(ModeReason reason)
{
    // We're back in radio contact
    gcs().send_text(MAV_SEVERITY_WARNING, "Short Failsafe Cleared");
    failsafe.state = FAILSAFE_NONE;
    // restore entry mode if desired but check that our current mode is still due to failsafe
    if (control_mode_reason == ModeReason::RADIO_FAILSAFE) { 
       set_mode_by_number(failsafe.saved_mode_number, ModeReason::RADIO_FAILSAFE_RECOVERY);
       gcs().send_text(MAV_SEVERITY_INFO,"Flight mode %s restored",control_mode->name());
    }
}

/**
 * @brief Handle recovery from extended RC or GCS signal loss
 * 
 * @details This function is called when RC or GCS signal is restored after
 *          an extended failsafe event. Unlike short failsafe recovery, long
 *          failsafe does NOT automatically restore the previous mode, as the
 *          aircraft may have already initiated return-to-launch or landing
 *          procedures that should continue to completion.
 * 
 *          Recovery Actions:
 *          1. Clears long_failsafe_pending flag (used for takeoff deferral)
 *          2. Clears failsafe state flag
 *          3. Notifies pilot/GCS of recovery
 *          4. Maintains current mode (no automatic mode restoration)
 * 
 *          Design Rationale:
 *          After a long failsafe, the aircraft may be:
 *          - Returning to launch point
 *          - In autonomous landing sequence
 *          - In emergency descent mode
 *          
 *          Automatically reverting to the pre-failsafe mode could be unsafe
 *          if the pilot has lost situational awareness or if the aircraft
 *          has moved far from its original position.
 * 
 * @param[in] reason Recovery reason (RC or GCS recovery)
 * 
 * @note Called from radio.cpp or GCS handling code when signal restored
 * @note Handles both RC and GCS failsafe recovery
 * @note Does NOT restore previous mode - pilot must manually change mode
 * @note Clears long_failsafe_pending to allow normal failsafe operation
 * 
 * @warning Pilot must manually resume mission or change mode after recovery
 * @warning This design prioritizes safety over convenience
 * 
 * @see failsafe_long_on_event() which may set long_failsafe_pending
 * @see long_failsafe_pending flag used during takeoff phase
 */
void Plane::failsafe_long_off_event(ModeReason reason)
{
    long_failsafe_pending = false;
    // We're back in radio contact with RC or GCS
    if (reason == ModeReason:: GCS_FAILSAFE) {
        gcs().send_text(MAV_SEVERITY_WARNING, "GCS Failsafe Off");
    }
    else {
        gcs().send_text(MAV_SEVERITY_WARNING, "RC Long Failsafe Cleared");
    }
    failsafe.state = FAILSAFE_NONE;
}

/**
 * @brief Handle battery capacity or voltage failsafe event
 * 
 * @details This function implements the response to battery failsafe conditions,
 *          triggered when battery voltage drops below critical threshold or
 *          remaining capacity is insufficient. The battery failsafe provides
 *          graduated responses to ensure the aircraft lands safely before
 *          complete power loss.
 * 
 *          Battery failsafe can be triggered by:
 *          - Low battery voltage (below BATT_LOW_VOLT threshold)
 *          - Critical battery voltage (below BATT_CRT_VOLT threshold)
 *          - Low remaining capacity (below BATT_LOW_MAH threshold)
 *          - Critical remaining capacity (below BATT_CRT_MAH threshold)
 * 
 *          Failsafe Actions (configured via BATT_FS_LOW_ACT or BATT_FS_CRT_ACT):
 * 
 *          1. Failsafe_Action_None:
 *             - Flags failsafe but takes no action
 *             - Sends notifications to pilot/GCS
 * 
 *          2. Failsafe_Action_Land:
 *             - QuadPlane: Attempts QLAND or LOITER_ALT_QLAND
 *             - Fixed-wing: Searches for nearest landing mission item
 *             - If best landing found, executes mission land sequence
 *             - Falls through to RTL if no suitable landing found
 * 
 *          3. Failsafe_Action_RTL / Failsafe_Action_AUTOLAND_OR_RTL:
 *             - Does not interrupt if already in landing sequence
 *             - Loads cruise throttle for efficient return
 *             - AUTOLAND_OR_RTL: Attempts AUTOLAND mode first, falls back to RTL
 *             - Standard RTL: Executes return-to-launch
 *             - If RTL_AUTOLAND=2, may continue mission if landing is closer
 * 
 *          4. Failsafe_Action_Terminate:
 *             - If Advanced Failsafe enabled: Triggers GCS termination
 *             - Otherwise: Immediately disarms aircraft
 *             - IRREVERSIBLE - use with extreme caution
 * 
 *          5. Failsafe_Action_Parachute:
 *             - Deploys parachute if available
 *             - IRREVERSIBLE - terminates controlled flight
 * 
 *          6. QuadPlane-Specific Actions:
 *             - Failsafe_Action_QLand: Initiates QuadPlane vertical landing
 *             - Failsafe_Action_Loiter_alt_QLand: Loiters to altitude then lands
 *             - Both fall through to fixed-wing actions if QuadPlane unavailable
 * 
 *          Landing Sequence Protection:
 *          - Once in a landing sequence, failsafe will not interrupt
 *          - This prevents dangerous mode changes during final approach
 *          - Applies to both fixed-wing and VTOL landing sequences
 * 
 * @param[in] type_str Battery type description for logging ("Low" or "Critical")
 * @param[in] action Failsafe action to execute (Failsafe_Action enum value)
 * 
 * @note Called from AP_BattMonitor when battery thresholds are crossed
 * @note May be called multiple times if battery continues to degrade
 * @note Uses mission.jump_to_landing_sequence() to find nearest landing
 * @note Sets ModeReason::BATTERY_FAILSAFE for all mode changes
 * 
 * @warning This is a safety-critical function - battery depletion is time-critical
 * @warning Terminate and Parachute actions are IRREVERSIBLE
 * @warning Does not interrupt committed landing sequences to prevent crash
 * @warning QuadPlane actions fall through to fixed-wing if QuadPlane unavailable
 * @warning FALLTHROUGH statements are intentional for cascading failsafe logic
 * 
 * @see AP_BattMonitor for threshold detection and failsafe triggering
 * @see BATT_FS_LOW_ACT and BATT_FS_CRT_ACT parameters
 * @see BATT_LOW_VOLT, BATT_CRT_VOLT, BATT_LOW_MAH, BATT_CRT_MAH parameters
 * @see mission.jump_to_landing_sequence() for automatic landing selection
 * @see Failsafe_Action enum for available actions
 */
void Plane::handle_battery_failsafe(const char *type_str, const int8_t action)
{
    // Execute configured battery failsafe action with cascading fallback logic
    switch ((Failsafe_Action)action) {
#if HAL_QUADPLANE_ENABLED
        case Failsafe_Action_Loiter_alt_QLand:
            // QuadPlane: Loiter to safe altitude then vertical land
            if (quadplane.available()) {
                plane.set_mode(mode_loiter_qland, ModeReason::BATTERY_FAILSAFE);
                break;
            }
            // QuadPlane not available, try standard QLAND
            FALLTHROUGH;

        case Failsafe_Action_QLand:
            // QuadPlane: Immediate vertical landing
            if (quadplane.available()) {
                plane.set_mode(mode_qland, ModeReason::BATTERY_FAILSAFE);
                break;
            }
            // QuadPlane not available, fall through to fixed-wing landing
            FALLTHROUGH;
#endif // HAL_QUADPLANE_ENABLED
        case Failsafe_Action_Land: {
            // Fixed-wing: Find and execute nearest mission landing sequence
            // Check if already in any landing mode
            bool already_landing = flight_stage == AP_FixedWing::FlightStage::LAND;
#if HAL_QUADPLANE_ENABLED
            if (control_mode == &mode_qland || control_mode == &mode_loiter_qland) {
                already_landing = true;
            }
#endif
            if (!already_landing && plane.have_position) {
                // CRITICAL: Never interrupt a committed landing sequence
                if (plane.mission.is_best_land_sequence(plane.current_loc)) {
                    // Current mission will reach landing sooner than jumping to different sequence
                    plane.mission.set_in_landing_sequence_flag(true);
                    break;
                }
                // Search mission for nearest landing sequence and jump to it
                if (plane.mission.jump_to_landing_sequence(plane.current_loc)) {
                    plane.set_mode(mode_auto, ModeReason::BATTERY_FAILSAFE);
                    break;
                }
            }
            // No suitable landing found, fall through to RTL
            FALLTHROUGH;
        }
        case Failsafe_Action_RTL:
        case Failsafe_Action_AUTOLAND_OR_RTL: {
            // Return to launch point or execute automated landing
            // Check all possible landing modes to prevent interruption
            bool already_landing = flight_stage == AP_FixedWing::FlightStage::LAND;
#if HAL_QUADPLANE_ENABLED
            if (control_mode == &mode_qland || control_mode == &mode_loiter_qland ||
                quadplane.in_vtol_land_sequence()) {
                already_landing = true;
            }
#endif
#if MODE_AUTOLAND_ENABLED
            if (control_mode == &mode_autoland) {
                already_landing = true;
            }
#endif
            if (!already_landing) {
                // CRITICAL: Never interrupt a committed landing sequence
                // Check if RTL would trigger immediate landing and if current mission is closer
                if ((g.rtl_autoland == RtlAutoland::RTL_IMMEDIATE_DO_LAND_START) && 
                    plane.have_position && 
                    plane.mission.is_best_land_sequence(plane.current_loc)) {
                    // Continue current mission - it will land sooner than RTL
                    plane.mission.set_in_landing_sequence_flag(true);
                    break;
                }
                // Load cruise throttle for efficient return flight
                aparm.throttle_cruise.load();
#if MODE_AUTOLAND_ENABLED
                // Try AUTOLAND first if action is AUTOLAND_OR_RTL
                if (((Failsafe_Action)action == Failsafe_Action_AUTOLAND_OR_RTL) && 
                    set_mode(mode_autoland, ModeReason::BATTERY_FAILSAFE)) {
                    break;
                }
#endif
                // Execute standard return-to-launch
                set_mode(mode_rtl, ModeReason::BATTERY_FAILSAFE);
            }
            break;
        }

        case Failsafe_Action_Terminate:
            // CRITICAL: IRREVERSIBLE ACTION - Immediately disarms aircraft
            // Use with extreme caution - typically only for flight termination systems
#if AP_ADVANCEDFAILSAFE_ENABLED
            // Advanced Failsafe: Trigger GCS termination protocol
            char battery_type_str[17];
            snprintf(battery_type_str, 17, "%s battery", type_str);
            afs.gcs_terminate(true, battery_type_str);
#else
            // Standard: Immediately disarm motors
            arming.disarm(AP_Arming::Method::FAILSAFE_ACTION_TERMINATE);
#endif
            break;

        case Failsafe_Action_Parachute:
            // CRITICAL: IRREVERSIBLE ACTION - Deploys parachute
            // Terminates controlled flight - use only as last resort
#if HAL_PARACHUTE_ENABLED
            parachute_release();
#endif
            break;

        case Failsafe_Action_None:
            // Log failsafe condition but take no autonomous action
            // Pilot retains full control to handle situation manually
            // Still sends notifications to ensure pilot awareness
            break;
    }
}
