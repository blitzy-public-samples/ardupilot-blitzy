/**
 * @file AP_State.cpp
 * @brief Vehicle state management for Blimp
 * 
 * @details This file implements idempotent state setter functions for managing
 *          critical vehicle states in the Blimp autopilot. It handles:
 *          - Auto-armed state management with logger event integration
 *          - Radio failsafe state with event triggering and AP_Notify synchronization
 *          - Ground Control Station (GCS) failsafe state with notification flags
 *          
 *          All setter functions are designed to be idempotent - they can be safely
 *          called repeatedly with the same value without triggering duplicate events
 *          or side effects. State changes are synchronized with the AP_Notify system
 *          for LED/buzzer feedback and with the AP_Logger for event recording.
 * 
 * @note This is part of the Blimp vehicle-specific implementation and follows
 *       the state management patterns used across all ArduPilot vehicle types.
 * 
 * Source: Blimp/AP_State.cpp
 */

#include "Blimp.h"

/**
 * @brief Set the auto-armed state for the vehicle
 * 
 * @details This function manages the auto-armed state flag which indicates whether
 *          the vehicle has automatically armed (as opposed to manual arming). The
 *          function is idempotent - it only performs actions when the state actually
 *          changes, preventing duplicate log events.
 *          
 *          When the vehicle transitions to auto-armed state (false to true), the
 *          function logs an AUTO_ARMED event to the dataflash log. This event is
 *          important for post-flight analysis and debugging.
 *          
 *          The auto-armed state is typically set during automatic arming sequences
 *          in autonomous flight modes and affects how certain mode transitions and
 *          safety checks are performed.
 * 
 * @param[in] b New auto-armed state: true = auto-armed, false = not auto-armed
 * 
 * @note This function is idempotent - calling it multiple times with the same
 *       value will only trigger the log event once on the initial transition.
 * @note Only transitions to true (becoming auto-armed) generate log events;
 *       transitions to false are silent.
 * 
 * @see LOGGER_WRITE_EVENT()
 * @see LogEvent::AUTO_ARMED
 * 
 * Source: Blimp/AP_State.cpp:4-15
 */
void Blimp::set_auto_armed(bool b)
{
    // if no change, exit immediately
    if ( ap.auto_armed == b ) {
        return;
    }

    ap.auto_armed = b;
    if (b) {
        LOGGER_WRITE_EVENT(LogEvent::AUTO_ARMED);
    }
}

/**
 * @brief Set the radio failsafe state and trigger appropriate failsafe actions
 * 
 * @details This function manages the radio failsafe state which indicates loss of
 *          RC (radio control) communication from the pilot's transmitter. The function
 *          is idempotent - it only performs actions when the state actually changes,
 *          preventing duplicate failsafe event triggers.
 *          
 *          When radio contact state changes, this function:
 *          1. Updates the failsafe.radio flag to reflect the new state
 *          2. Triggers appropriate failsafe event handlers:
 *             - failsafe_radio_off_event() when radio contact is regained
 *             - failsafe_radio_on_event() when radio contact is lost
 *          3. Updates AP_Notify flags to trigger LED/buzzer notifications
 *          
 *          The radio failsafe is a critical safety mechanism that typically triggers
 *          protective actions such as RTL (Return to Launch) or LAND when RC signal
 *          is lost. This prevents flyaways and ensures the vehicle can safely handle
 *          communication loss.
 * 
 * @param[in] b New radio failsafe state: true = radio signal lost, false = radio OK
 * 
 * @note This function is idempotent - calling it multiple times with the same
 *       value will not trigger duplicate failsafe events or notifications.
 * @warning Radio failsafe is a safety-critical function. Loss of radio contact
 *          triggers protective flight modes to prevent vehicle loss or damage.
 * @note The AP_Notify system is always updated regardless of state change to
 *       ensure notification devices reflect current failsafe status.
 * 
 * @see failsafe_radio_on_event()
 * @see failsafe_radio_off_event()
 * @see AP_Notify::flags
 * 
 * Source: Blimp/AP_State.cpp:18-41
 */
void Blimp::set_failsafe_radio(bool b)
{
    // only act on changes
    // -------------------
    if (failsafe.radio != b) {

        // store the value so we don't trip the gate twice
        // -----------------------------------------------
        failsafe.radio = b;

        if (failsafe.radio == false) {
            // We've regained radio contact
            // ----------------------------
            failsafe_radio_off_event();
        } else {
            // We've lost radio contact
            // ------------------------
            failsafe_radio_on_event();
        }

        // update AP_Notify
        AP_Notify::flags.failsafe_radio = b;
    }
}

/**
 * @brief Set the Ground Control Station (GCS) failsafe state
 * 
 * @details This function manages the GCS failsafe state which indicates loss of
 *          communication with the ground control station via MAVLink telemetry.
 *          Unlike set_failsafe_radio(), this function does not implement idempotent
 *          behavior - it updates the state and notification flags every time it is
 *          called, regardless of whether the state has changed.
 *          
 *          When called, this function:
 *          1. Updates the failsafe.gcs flag to reflect the new state
 *          2. Updates AP_Notify flags to trigger LED/buzzer notifications
 *          
 *          The GCS failsafe is typically triggered when MAVLink heartbeat messages
 *          from the ground station are not received within the configured timeout
 *          period. This is a separate failsafe from radio failsafe, as GCS telemetry
 *          and RC control are independent communication channels.
 *          
 *          GCS failsafe actions may include triggering RTL (Return to Launch), entering
 *          a holding pattern, or continuing with the current mission, depending on
 *          the configured failsafe action parameters.
 * 
 * @param[in] b New GCS failsafe state: true = GCS communication lost, false = GCS OK
 * 
 * @note Unlike set_failsafe_radio(), this function is NOT idempotent and will
 *       update the AP_Notify flags on every call.
 * @warning GCS failsafe is a safety-critical function. Loss of GCS communication
 *          may trigger protective flight modes depending on configuration.
 * @note This function does not explicitly trigger failsafe event handlers - GCS
 *       failsafe event handling is managed elsewhere in the failsafe check code.
 * 
 * @see AP_Notify::flags
 * @see set_failsafe_radio()
 * 
 * Source: Blimp/AP_State.cpp:45-51
 */
void Blimp::set_failsafe_gcs(bool b)
{
    failsafe.gcs = b;

    // update AP_Notify
    AP_Notify::flags.failsafe_gcs = b;
}
