/**
 * @file events.cpp
 * @brief Event logging and failsafe notification management for ArduCopter
 * 
 * @details This file implements the event handling system for ArduCopter, managing
 *          failsafe triggers, recovery events, and safety-critical notifications.
 *          Events are logged to both dataflash (AP_Logger) and transmitted to ground
 *          control stations (GCS) for real-time monitoring and post-flight analysis.
 *          
 *          Key Event Categories:
 *          - Radio/RC failsafe events (signal loss and recovery)
 *          - Battery failsafe events (low voltage, capacity, or current)
 *          - GCS communication failsafe events (telemetry link loss)
 *          - Terrain data failsafe events (missing terrain altitude data)
 *          - GPS glitch and dead reckoning events
 *          - Mode change events triggered by failsafes
 *          
 *          Event Notification Mechanisms:
 *          - AP_Logger (dataflash): Permanent log records for post-flight analysis
 *          - GCS MAVLink messages: Real-time text warnings to ground station
 *          - AP_Notify: LED patterns and buzzer alerts for on-vehicle indication
 *          
 *          Each failsafe follows a standard pattern:
 *          1. Detect failsafe condition (signal loss, timeout, data missing)
 *          2. Log event to dataflash with error subsystem and code
 *          3. Announce to GCS with appropriate severity level
 *          4. Determine appropriate action based on parameters and flight state
 *          5. Execute failsafe action (RTL, Land, Disarm, etc.)
 *          6. Monitor for recovery and log resolution
 *          
 *          Safety Investigation Support:
 *          - All failsafe events are timestamped in logs for correlation with telemetry
 *          - Error codes identify specific failure conditions for root cause analysis
 *          - GCS messages provide immediate feedback for operational decisions
 *          - Event sequences can be reconstructed from logs to understand cascading failures
 * 
 * @note This is safety-critical code - modifications must preserve failsafe behavior
 * @warning Improper event handling can lead to delayed failsafe responses or missed events
 * 
 * @see Copter.h for FailsafeAction enum and failsafe state tracking
 * @see AP_Logger for dataflash logging API
 * @see GCS_MAVLink for ground station communication
 * 
 * Source: ArduCopter/events.cpp
 */

#include "Copter.h"

/**
 * @brief Check if a specific failsafe option is enabled
 * 
 * @details Queries the FS_OPTIONS bitmask parameter to determine if a particular
 *          failsafe behavior modifier is enabled. Failsafe options allow users to
 *          customize failsafe behavior such as continuing landing, continuing mission,
 *          or continuing in guided mode when failsafes occur.
 *          
 *          Common failsafe options:
 *          - CONTINUE_IF_LANDING: Continue landing instead of switching modes
 *          - RC_CONTINUE_IF_AUTO: Continue AUTO mode on RC failsafe
 *          - RC_CONTINUE_IF_GUIDED: Continue GUIDED mode on RC failsafe
 *          - GCS_CONTINUE_IF_AUTO: Continue AUTO mode on GCS failsafe
 *          - GCS_CONTINUE_IF_PILOT_CONTROL: Continue pilot-controlled modes on GCS failsafe
 *          - RELEASE_GRIPPER: Release gripper on failsafe
 * 
 * @param[in] opt The failsafe option to check (from FailsafeOption enum)
 * 
 * @return true if the specified failsafe option is enabled in FS_OPTIONS parameter
 * @return false if the option is not enabled
 * 
 * @note This function is const as it only reads parameter state
 * @note FS_OPTIONS is a bitmask parameter allowing multiple options simultaneously
 * 
 * @see FailsafeOption enum in Copter.h
 * @see FS_OPTIONS parameter documentation
 */
bool Copter::failsafe_option(FailsafeOption opt) const
{
    return (g2.fs_options & (uint32_t)opt);
}

/**
 * @brief Handle radio (RC) failsafe trigger event
 * 
 * @details Called when RC receiver signal is lost or invalid PWM values are received.
 *          This is a critical safety event as loss of RC control requires immediate
 *          action to prevent flyaways or crashes. The function determines the appropriate
 *          response based on FS_THR_ENABLE parameter, current flight mode, and vehicle state.
 *          
 *          Failsafe Action Selection (based on FS_THR_ENABLE parameter):
 *          - FS_THR_DISABLED (0): No action (not recommended)
 *          - FS_THR_ENABLED_ALWAYS_RTL (1): Return to Launch
 *          - FS_THR_ENABLED_CONTINUE_MISSION (2): RTL or continue mission if in AUTO
 *          - FS_THR_ENABLED_ALWAYS_SMARTRTL_OR_RTL (3): Smart RTL with RTL fallback
 *          - FS_THR_ENABLED_ALWAYS_SMARTRTL_OR_LAND (4): Smart RTL with Land fallback
 *          - FS_THR_ENABLED_ALWAYS_LAND (5): Land immediately
 *          - FS_THR_ENABLED_AUTO_RTL_OR_RTL (6): Jump to DO_LAND_START in AUTO or RTL
 *          - FS_THR_ENABLED_BRAKE_OR_LAND (7): Brake then land
 *          
 *          Special Handling Cases (override parameter setting):
 *          1. Should disarm: If on ground with zero throttle or in arming delay, disarm immediately
 *          2. Battery + Radio: If landing with battery failsafe active, continue landing
 *          3. Continue landing: If FS_OPTIONS allows, continue existing landing
 *          4. Continue AUTO: If FS_OPTIONS allows, continue mission
 *          5. Continue GUIDED: If FS_OPTIONS allows, continue guided mode
 *          
 *          Event Logging:
 *          - Logs FAILSAFE_RADIO/FAILSAFE_OCCURRED to dataflash for post-flight analysis
 *          - Sends MAVLink text message to GCS with specific action taken
 *          - Triggers AP_Notify for LED/buzzer indication
 *          
 *          Notification Examples:
 *          - "Radio Failsafe - Disarming" (on ground)
 *          - "Radio + Battery Failsafe - Continuing Landing"
 *          - "Radio Failsafe - Continuing Auto"
 *          - "Radio Failsafe" (standard case, action implied by mode change)
 * 
 * @note Called from failsafe.cpp when RC signal is lost
 * @note This is called at the main loop rate (typically 400Hz for copter)
 * 
 * @warning This is safety-critical code - vehicle behavior changes immediately
 * @warning Do not call directly - use through failsafe state machine
 * 
 * @see failsafe_radio_off_event() for recovery handling
 * @see do_failsafe_action() for action execution
 * @see FS_THR_ENABLE parameter documentation
 * 
 * Source: ArduCopter/events.cpp:13-79
 */
void Copter::failsafe_radio_on_event()
{
    // Log radio failsafe event to dataflash for post-flight analysis
    LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_RADIO, LogErrorCode::FAILSAFE_OCCURRED);

    // Determine desired failsafe action based on FS_THR_ENABLE parameter
    // This is the user-configured behavior, but may be overridden below for safety
    FailsafeAction desired_action;
    switch (g.failsafe_throttle) {
        case FS_THR_DISABLED:
            // No action - continues in current mode (not recommended for safety)
            desired_action = FailsafeAction::NONE;
            break;
        case FS_THR_ENABLED_ALWAYS_RTL:
        case FS_THR_ENABLED_CONTINUE_MISSION:
            // Return to launch point - most common failsafe action
            desired_action = FailsafeAction::RTL;
            break;
        case FS_THR_ENABLED_ALWAYS_SMARTRTL_OR_RTL:
            // Smart RTL retraces path, falls back to regular RTL if path unavailable
            desired_action = FailsafeAction::SMARTRTL;
            break;
        case FS_THR_ENABLED_ALWAYS_SMARTRTL_OR_LAND:
            // Smart RTL with land fallback (no regular RTL fallback)
            desired_action = FailsafeAction::SMARTRTL_LAND;
            break;
        case FS_THR_ENABLED_ALWAYS_LAND:
            // Land immediately at current location
            desired_action = FailsafeAction::LAND;
            break;
        case FS_THR_ENABLED_AUTO_RTL_OR_RTL:
            // Jump to DO_LAND_START mission item if in AUTO mode
            desired_action = FailsafeAction::AUTO_DO_LAND_START;
            break;
        case FS_THR_ENABLED_BRAKE_OR_LAND:
            // Brake to stop horizontal movement, then land
            desired_action = FailsafeAction::BRAKE_LAND;
            break;
        default:
            // Invalid parameter value - default to land for safety
            desired_action = FailsafeAction::LAND;
    }

    // Special handling: Override parameter setting based on vehicle state and safety considerations
    // These conditions take precedence over FS_THR_ENABLE to prevent unsafe behavior
    if (should_disarm_on_failsafe()) {
        // Safety check: Disarm immediately if on ground with zero throttle or in arming delay
        // Prevents unnecessary mode changes when vehicle is already safely on the ground
        announce_failsafe("Radio", "Disarming");
        arming.disarm(AP_Arming::Method::RADIOFAILSAFE);
        desired_action = FailsafeAction::NONE;

    } else if (flightmode->is_landing() && ((battery.has_failsafed() && battery.get_highest_failsafe_priority() <= FAILSAFE_LAND_PRIORITY))) {
        // Cascading failsafe: If battery AND radio failsafe active during landing,
        // continue landing rather than attempting RTL (may not have enough battery)
        announce_failsafe("Radio + Battery", "Continuing Landing");
        desired_action = FailsafeAction::LAND;

    } else if (flightmode->is_landing() && failsafe_option(FailsafeOption::CONTINUE_IF_LANDING)) {
        // User option: Allow landing to complete if FS_OPTIONS bit is set
        // Useful to prevent aborted landings due to momentary RC glitches
        announce_failsafe("Radio", "Continuing Landing");
        desired_action = FailsafeAction::LAND;

    } else if (flightmode->mode_number() == Mode::Number::AUTO && failsafe_option(FailsafeOption::RC_CONTINUE_IF_AUTO)) {
        // User option: Continue autonomous mission on RC failsafe if FS_OPTIONS bit is set
        // Allows missions to complete even if RC link is lost (requires GCS monitoring)
        announce_failsafe("Radio", "Continuing Auto");
        desired_action = FailsafeAction::NONE;

    } else if ((flightmode->in_guided_mode()) && failsafe_option(FailsafeOption::RC_CONTINUE_IF_GUIDED)) {
        // User option: Continue guided mode on RC failsafe if FS_OPTIONS bit is set
        // Allows GCS-controlled flight to continue without RC (common for autonomous operations)
        announce_failsafe("Radio", "Continuing Guided Mode");
        desired_action = FailsafeAction::NONE;

    } else {
        // Standard case: Announce failsafe without specific action message
        // Action will be indicated by mode change notification
        announce_failsafe("Radio");
    }

    // Execute the determined failsafe action and log mode change reason
    // This will trigger mode change, log events, and notify pilot
    do_failsafe_action(desired_action, ModeReason::RADIO_FAILSAFE);
}

/**
 * @brief Handle radio (RC) failsafe recovery event
 * 
 * @details Called when RC receiver signal is restored after a radio failsafe.
 *          Unlike the failsafe trigger, recovery does not automatically change
 *          flight modes - the pilot must manually regain control using stick
 *          inputs or mode switch. This prevents dangerous automatic mode changes
 *          if signal is intermittent.
 *          
 *          Event Logging:
 *          - Logs FAILSAFE_RADIO/FAILSAFE_RESOLVED to dataflash
 *          - Sends "Radio Failsafe Cleared" MAVLink message to GCS
 *          
 *          Post-Recovery Behavior:
 *          - Vehicle remains in failsafe mode (RTL, Land, etc.)
 *          - Pilot can now override controls (roll, pitch, yaw, throttle)
 *          - Pilot can use mode switch to change flight modes
 *          - No automatic return to pre-failsafe mode (safety consideration)
 * 
 * @note Called from failsafe.cpp when RC signal is restored
 * @note Does not change flight mode - pilot must manually recover
 * 
 * @warning Do not automatically return to pre-failsafe mode to prevent
 *          dangerous behavior if signal is intermittent
 * 
 * @see failsafe_radio_on_event() for failsafe trigger handling
 * 
 * Source: ArduCopter/events.cpp:82-88
 */
void Copter::failsafe_radio_off_event()
{
    // Log recovery event to dataflash - pairs with failsafe trigger for analysis
    // User can now override roll, pitch, yaw and throttle and even use flight mode switch to restore previous flight mode
    LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_RADIO, LogErrorCode::FAILSAFE_RESOLVED);
    
    // Notify GCS that radio link has been restored
    gcs().send_text(MAV_SEVERITY_WARNING, "Radio Failsafe Cleared");
}

/**
 * @brief Announce failsafe event to ground control station
 * 
 * @details Sends a formatted MAVLink text message to the GCS announcing
 *          a failsafe event. This provides immediate feedback to the operator
 *          about what type of failsafe occurred and what action is being taken.
 *          Messages are sent with WARNING severity to ensure visibility in GCS.
 *          
 *          Message Formats:
 *          - With action: "<type> Failsafe - <action>" (e.g., "Radio Failsafe - Disarming")
 *          - Without action: "<type> Failsafe" (action implied by mode change)
 *          
 *          Common Failsafe Types:
 *          - "Radio" - RC signal loss
 *          - "Battery" - Low voltage/capacity
 *          - "GCS" - Telemetry link loss
 *          - "Radio + Battery" - Multiple simultaneous failsafes
 *          
 *          Common Actions:
 *          - "Disarming" - Vehicle disarmed on ground
 *          - "Continuing Landing" - Completing landing sequence
 *          - "Continuing Auto" - Mission continues
 *          - "Continuing Guided Mode" - GCS control continues
 * 
 * @param[in] type Failsafe type string (e.g., "Radio", "Battery", "GCS")
 * @param[in] action_undertaken Action description string, or nullptr if no specific action
 * 
 * @note Messages sent at MAV_SEVERITY_WARNING level for visibility
 * @note This function is called before do_failsafe_action() to provide context
 * 
 * @see gcs().send_text() for MAVLink message transmission
 * 
 * Source: ArduCopter/events.cpp:90-97
 */
void Copter::announce_failsafe(const char *type, const char *action_undertaken)
{
    if (action_undertaken != nullptr) {
        // Announce failsafe with specific action being taken
        gcs().send_text(MAV_SEVERITY_WARNING, "%s Failsafe - %s", type, action_undertaken);
    } else {
        // Announce failsafe without specific action (action will be indicated by mode change)
        gcs().send_text(MAV_SEVERITY_WARNING, "%s Failsafe", type);
    }
}

/**
 * @brief Handle battery failsafe trigger event
 * 
 * @details Called when battery voltage, remaining capacity, or current draw
 *          exceeds configured failsafe thresholds. Battery failsafe can be
 *          triggered at multiple levels (warning, low, critical) with different
 *          actions configured for each level via BATT_FS_LOW_ACT and BATT_FS_CRT_ACT.
 *          
 *          Battery Failsafe Triggers:
 *          - BATT_LOW_VOLT: Low voltage threshold (typically 3.5V/cell)
 *          - BATT_CRT_VOLT: Critical voltage threshold (typically 3.4V/cell)
 *          - BATT_LOW_MAH: Low capacity remaining (mAh)
 *          - BATT_CRT_MAH: Critical capacity remaining (mAh)
 *          
 *          Configurable Actions (via BATT_FS_LOW_ACT, BATT_FS_CRT_ACT):
 *          - NONE: No action (warning only)
 *          - LAND: Land immediately
 *          - RTL: Return to launch
 *          - SMARTRTL: Smart RTL with fallback
 *          - TERMINATE: Immediate disarm (emergency only)
 *          
 *          Special Handling:
 *          - Disarm if on ground with zero throttle (safe)
 *          - Continue landing if already landing and FS_OPTIONS allows
 *          - Battery failsafe takes precedence during landing operations
 *          
 *          Event Logging:
 *          - Logs FAILSAFE_BATT/FAILSAFE_OCCURRED to dataflash
 *          - Sends "Battery Failsafe" MAVLink message to GCS
 *          - Triggers AP_Notify battery warning LED pattern
 * 
 * @param[in] type_str Battery failsafe type description (currently unused)
 * @param[in] action Failsafe action to take (from BATT_FS_LOW_ACT or BATT_FS_CRT_ACT)
 * 
 * @note Called from AP_BattMonitor when voltage/capacity thresholds exceeded
 * @note Multiple battery failsafe levels can cascade (low → critical)
 * 
 * @warning Battery failsafe is time-critical - delayed action can result in
 *          voltage sag leading to brownout or uncontrolled descent
 * 
 * @see AP_BattMonitor for voltage/capacity monitoring
 * @see BATT_FS_LOW_ACT, BATT_FS_CRT_ACT parameters
 * @see do_failsafe_action() for action execution
 * 
 * Source: ArduCopter/events.cpp:99-123
 */
void Copter::handle_battery_failsafe(const char *type_str, const int8_t action)
{
    // Log battery failsafe event to dataflash for post-flight battery analysis
    LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_BATT, LogErrorCode::FAILSAFE_OCCURRED);

    // Convert parameter action value to FailsafeAction enum
    FailsafeAction desired_action = (FailsafeAction)action;

    // Special handling: Override configured action based on vehicle state
    if (should_disarm_on_failsafe()) {
        // Safety check: Disarm immediately if on ground with zero throttle
        // Prevents unnecessary mode changes when vehicle is safely on ground
        arming.disarm(AP_Arming::Method::BATTERYFAILSAFE);
        desired_action = FailsafeAction::NONE;
        announce_failsafe("Battery", "Disarming");

    } else if (flightmode->is_landing() && failsafe_option(FailsafeOption::CONTINUE_IF_LANDING) && desired_action != FailsafeAction::NONE) {
        // User option: Allow landing to complete if FS_OPTIONS bit is set
        // Prevents aborted landing which could waste more battery than completing descent
        desired_action = FailsafeAction::LAND;
        announce_failsafe("Battery", "Continuing Landing");
    } else {
        // Standard case: Announce battery failsafe, action implied by mode change
        announce_failsafe("Battery");
    }

    // Execute the configured or overridden failsafe action
    // Battery failsafe actions use the FailsafeAction enum directly
    do_failsafe_action(desired_action, ModeReason::BATTERY_FAILSAFE);

}

/**
 * @brief Check for ground control station (GCS) communication failsafe
 * 
 * @details Monitors the time since last MAVLink heartbeat from the primary GCS
 *          and triggers failsafe if timeout exceeds FS_GCS_TIMEOUT. This function
 *          is called periodically (typically every 100ms) to detect loss of
 *          telemetry link with the ground control station.
 *          
 *          GCS Failsafe Detection:
 *          - Monitors heartbeat from specific GCS system ID (SYSID_MYGCS parameter)
 *          - Timeout configured by FS_GCS_TIMEOUT parameter (default 5 seconds)
 *          - Only triggers if GCS was previously connected (prevents false triggers at startup)
 *          - Hysteresis: Requires timeout period for trigger AND recovery
 *          
 *          State Machine:
 *          1. Not in failsafe, link good: No action
 *          2. Not in failsafe, link timeout: Trigger failsafe_gcs_on_event()
 *          3. In failsafe, link good: Trigger failsafe_gcs_off_event()
 *          4. In failsafe, link timeout: No action (already in failsafe)
 *          
 *          Bypass Conditions:
 *          - FS_GCS_ENABLE = 0 (disabled): No GCS failsafe checking
 *          - GCS never connected: Prevents false trigger before first connection
 *          
 *          Typical Timeout Values:
 *          - FS_GCS_TIMEOUT = 5 seconds (default): Conservative for reliable links
 *          - FS_GCS_TIMEOUT = 2 seconds: Faster response for high-bandwidth links
 *          - FS_GCS_TIMEOUT = 20 seconds: Long delay for very poor links
 * 
 * @note Called from scheduler at ~10Hz (every 100ms)
 * @note Only monitors heartbeat from SYSID_MYGCS, ignores other GCS connections
 * @note Time calculated from system uptime (millis()), handles rollover
 * 
 * @warning Requires continuous GCS heartbeat to prevent failsafe
 * @warning Short timeout values can cause false triggers on poor links
 * 
 * @see failsafe_gcs_on_event() for failsafe trigger actions
 * @see failsafe_gcs_off_event() for recovery actions
 * @see FS_GCS_ENABLE, FS_GCS_TIMEOUT parameters
 * @see SYSID_MYGCS parameter for GCS system ID
 * 
 * Source: ArduCopter/events.cpp:125-160
 */
void Copter::failsafe_gcs_check()
{
    // Early exit if GCS failsafe is disabled (FS_GCS_ENABLE = 0)
    if (g.failsafe_gcs == FS_GCS_DISABLED) {
        return;
    }

    // Get timestamp of last heartbeat from primary GCS (SYSID_MYGCS)
    const uint32_t gcs_last_seen_ms = gcs().sysid_mygcs_last_seen_time_ms();
    
    // Early exit if GCS has never been connected (prevents false trigger at startup)
    if (gcs_last_seen_ms == 0) {
        return;
    }

    // Calculate time elapsed since last GCS heartbeat message
    // Note: Only monitors heartbeat from the specific system ID set by SYSID_MYGCS parameter
    const uint32_t last_gcs_update_ms = millis() - gcs_last_seen_ms;
    
    // Convert FS_GCS_TIMEOUT parameter from seconds to milliseconds with bounds checking
    const uint32_t gcs_timeout_ms = uint32_t(constrain_float(g2.fs_gcs_timeout * 1000.0f, 0.0f, UINT32_MAX));

    // State machine: Determine failsafe state transitions based on link status
    if (last_gcs_update_ms < gcs_timeout_ms && failsafe.gcs) {
        // State transition: Failsafe → Normal (link restored)
        set_failsafe_gcs(false);
        failsafe_gcs_off_event();

    } else if (last_gcs_update_ms < gcs_timeout_ms && !failsafe.gcs) {
        // State: Normal operation, link is good
        // No action required

    } else if (last_gcs_update_ms > gcs_timeout_ms && failsafe.gcs) {
        // State: Already in failsafe, link still bad
        // No action required (already handled)

    } else if (last_gcs_update_ms > gcs_timeout_ms && !failsafe.gcs) {
        // State transition: Normal → Failsafe (link timeout detected)
        set_failsafe_gcs(true);
        failsafe_gcs_on_event();
    }
}

/**
 * @brief Handle ground control station (GCS) failsafe trigger event
 * 
 * @details Called when GCS telemetry link is lost (no heartbeat for FS_GCS_TIMEOUT).
 *          This function determines the appropriate failsafe response based on
 *          FS_GCS_ENABLE parameter, current flight mode, and vehicle state. Unlike
 *          radio failsafe, GCS failsafe clears RC overrides to allow RC control.
 *          
 *          Failsafe Action Selection (based on FS_GCS_ENABLE parameter):
 *          - FS_GCS_DISABLED (0): No action
 *          - FS_GCS_ENABLED_ALWAYS_RTL (1): Return to Launch
 *          - FS_GCS_ENABLED_CONTINUE_MISSION (2): RTL or continue mission if in AUTO
 *          - FS_GCS_ENABLED_ALWAYS_SMARTRTL_OR_RTL (3): Smart RTL with RTL fallback
 *          - FS_GCS_ENABLED_ALWAYS_SMARTRTL_OR_LAND (4): Smart RTL with Land fallback
 *          - FS_GCS_ENABLED_ALWAYS_LAND (5): Land immediately
 *          - FS_GCS_ENABLED_AUTO_RTL_OR_RTL (6): Jump to DO_LAND_START in AUTO or RTL
 *          - FS_GCS_ENABLED_BRAKE_OR_LAND (7): Brake then land
 *          
 *          Special Handling Cases (override parameter setting):
 *          1. Not armed: No action required (safe on ground)
 *          2. Should disarm: If on ground with zero throttle, disarm immediately
 *          3. Battery + GCS: If landing with battery failsafe, continue landing
 *          4. Continue landing: If FS_OPTIONS allows, continue existing landing
 *          5. Continue AUTO: If FS_OPTIONS allows and in AUTO mode
 *          6. Continue pilot control: If FS_OPTIONS allows and in pilot-controlled mode
 *          
 *          RC Override Behavior:
 *          - Clears all RC channel overrides from GCS on failsafe trigger
 *          - Allows pilot to regain control via physical RC transmitter
 *          - Important for operations where GCS was commanding stick inputs
 *          
 *          Event Logging:
 *          - Logs FAILSAFE_GCS/FAILSAFE_OCCURRED to dataflash
 *          - Sends MAVLink text message to GCS (if link briefly recovers)
 *          - Triggers AP_Notify for LED/buzzer indication
 *          
 *          Notification Examples:
 *          - "GCS Failsafe" (standard case)
 *          - "GCS Failsafe - Disarming" (on ground)
 *          - "GCS + Battery Failsafe - Continuing Landing"
 *          - "GCS Failsafe - Continuing Auto Mode"
 *          - "GCS Failsafe - Continuing Pilot Control"
 * 
 * @note Called from failsafe_gcs_check() when timeout detected
 * @note Clears RC overrides unlike radio failsafe (allows RC takeover)
 * 
 * @warning This is safety-critical code - vehicle behavior changes immediately
 * @warning Do not call directly - use through failsafe_gcs_check()
 * 
 * @see failsafe_gcs_check() for timeout monitoring
 * @see failsafe_gcs_off_event() for recovery handling
 * @see do_failsafe_action() for action execution
 * @see FS_GCS_ENABLE parameter documentation
 * 
 * Source: ArduCopter/events.cpp:163-233
 */
void Copter::failsafe_gcs_on_event(void)
{
    // Log GCS failsafe event to dataflash for post-flight analysis
    LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_GCS, LogErrorCode::FAILSAFE_OCCURRED);
    
    // Clear any RC channel overrides from GCS to allow pilot RC control
    // Important: This allows RC transmitter to regain control after GCS link loss
    RC_Channels::clear_overrides();

    // Determine desired failsafe action based on FS_GCS_ENABLE parameter
    // This is the user-configured behavior, but may be overridden below for safety
    FailsafeAction desired_action;
    switch (g.failsafe_gcs) {
        case FS_GCS_DISABLED:
            // No action - should not reach here as check bypasses if disabled
            desired_action = FailsafeAction::NONE;
            break;
        case FS_GCS_ENABLED_ALWAYS_RTL:
        case FS_GCS_ENABLED_CONTINUE_MISSION:
            // Return to launch point - common for GCS-controlled operations
            desired_action = FailsafeAction::RTL;
            break;
        case FS_GCS_ENABLED_ALWAYS_SMARTRTL_OR_RTL:
            // Smart RTL retraces path, falls back to regular RTL
            desired_action = FailsafeAction::SMARTRTL;
            break;
        case FS_GCS_ENABLED_ALWAYS_SMARTRTL_OR_LAND:
            // Smart RTL with land fallback (no regular RTL fallback)
            desired_action = FailsafeAction::SMARTRTL_LAND;
            break;
        case FS_GCS_ENABLED_ALWAYS_LAND:
            // Land immediately at current location
            desired_action = FailsafeAction::LAND;
            break;
        case FS_GCS_ENABLED_AUTO_RTL_OR_RTL:
            // Jump to DO_LAND_START mission item if in AUTO mode
            desired_action = FailsafeAction::AUTO_DO_LAND_START;
            break;
        case FS_GCS_ENABLED_BRAKE_OR_LAND:
            // Brake to stop horizontal movement, then land
            desired_action = FailsafeAction::BRAKE_LAND;
            break;
        default:
            // Invalid parameter value - default to RTL for safety
            desired_action = FailsafeAction::RTL;
    }

    // Special handling: Override parameter setting based on vehicle state
    // These conditions take precedence over FS_GCS_ENABLE for safety
    if (!motors->armed()) {
        // Vehicle is not armed - no action needed, already safe
        desired_action = FailsafeAction::NONE;
        announce_failsafe("GCS");

    } else if (should_disarm_on_failsafe()) {
        // Safety check: Disarm immediately if on ground with zero throttle
        // Prevents unnecessary mode changes when vehicle is safely on ground
        arming.disarm(AP_Arming::Method::GCSFAILSAFE);
        desired_action = FailsafeAction::NONE;
        announce_failsafe("GCS", "Disarming");

    } else if (flightmode->is_landing() && ((battery.has_failsafed() && battery.get_highest_failsafe_priority() <= FAILSAFE_LAND_PRIORITY))) {
        // Cascading failsafe: If battery AND GCS failsafe active during landing,
        // continue landing rather than attempting RTL (may not have enough battery)
        announce_failsafe("GCS + Battery", "Continuing Landing");
        desired_action = FailsafeAction::LAND;

    } else if (flightmode->is_landing() && failsafe_option(FailsafeOption::CONTINUE_IF_LANDING)) {
        // User option: Allow landing to complete if FS_OPTIONS bit is set
        // Useful for operations where GCS monitoring is not critical during landing
        announce_failsafe("GCS", "Continuing Landing");
        desired_action = FailsafeAction::LAND;

    } else if (flightmode->mode_number() == Mode::Number::AUTO && failsafe_option(FailsafeOption::GCS_CONTINUE_IF_AUTO)) {
        // User option: Continue autonomous mission on GCS failsafe if FS_OPTIONS bit is set
        // Allows missions to complete without GCS oversight (requires RC monitoring)
        announce_failsafe("GCS", "Continuing Auto Mode");
        desired_action = FailsafeAction::NONE;

    } else if (failsafe_option(FailsafeOption::GCS_CONTINUE_IF_PILOT_CONTROL) && !flightmode->is_autopilot()) {
        // User option: Continue in pilot-controlled modes (Stabilize, AltHold, Loiter, etc.)
        // Allows RC-controlled flight to continue without GCS (common for FPV operations)
        announce_failsafe("GCS", "Continuing Pilot Control");
        desired_action = FailsafeAction::NONE;
    } else {
        // Standard case: Announce GCS failsafe, action implied by mode change
        announce_failsafe("GCS");
    }

    // Execute the determined failsafe action and log mode change reason
    // This will trigger mode change, log events, and notify pilot
    do_failsafe_action(desired_action, ModeReason::GCS_FAILSAFE);
}

/**
 * @brief Handle ground control station (GCS) failsafe recovery event
 * 
 * @details Called when GCS telemetry link is restored after a GCS failsafe.
 *          Similar to radio failsafe recovery, this does not automatically
 *          change flight modes - the vehicle remains in the failsafe mode
 *          until operator manually intervenes via RC or restored GCS commands.
 *          
 *          Event Logging:
 *          - Logs FAILSAFE_GCS/FAILSAFE_RESOLVED to dataflash
 *          - Sends "GCS Failsafe Cleared" MAVLink message
 *          
 *          Post-Recovery Behavior:
 *          - Vehicle remains in failsafe mode (RTL, Land, etc.)
 *          - GCS can now send commands and override RC channels again
 *          - RC pilot can use mode switch to change flight modes
 *          - No automatic return to pre-failsafe mode (safety consideration)
 * 
 * @note Called from failsafe_gcs_check() when link is restored
 * @note Does not change flight mode - operator must manually recover
 * 
 * @warning Do not automatically return to pre-failsafe mode to prevent
 *          dangerous behavior if link is intermittent
 * 
 * @see failsafe_gcs_on_event() for failsafe trigger handling
 * @see failsafe_gcs_check() for link monitoring
 * 
 * Source: ArduCopter/events.cpp:236-240
 */
void Copter::failsafe_gcs_off_event(void)
{
    // Notify GCS that telemetry link has been restored
    gcs().send_text(MAV_SEVERITY_WARNING, "GCS Failsafe Cleared");
    
    // Log recovery event to dataflash - pairs with failsafe trigger for analysis
    LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_GCS, LogErrorCode::FAILSAFE_RESOLVED);
}

/**
 * @brief Check for terrain data failsafe timeout
 * 
 * @details Monitors terrain altitude data availability and triggers failsafe if data
 *          is missing for longer than FS_TERRAIN_TIMEOUT_MS (default 20 seconds) while
 *          in a flight mode that requires terrain data (e.g., terrain following modes).
 *          
 *          Terrain Data Sources:
 *          - Downloaded terrain tiles from SD card (most accurate)
 *          - Rangefinder altitude above ground
 *          - GPS altitude with home altitude as terrain reference
 *          
 *          Failsafe Trigger Conditions:
 *          - Terrain data unavailable for > FS_TERRAIN_TIMEOUT_MS (20 seconds)
 *          - AND currently in a mode requiring terrain data
 *          - Modes requiring terrain: RTL with WPNAV_RFND_USE, Guided, Auto with terrain alt frames
 *          
 *          Timeout Tracking:
 *          - terrain_first_failure_ms: Time of initial terrain data loss
 *          - terrain_last_failure_ms: Time of most recent failure
 *          - Duration = last - first (handles intermittent failures)
 *          
 *          Recovery:
 *          - Failsafe clears when terrain data becomes available again
 *          - Requires 100ms of continuous good data (set in failsafe_terrain_set_status)
 *          - Logs ERROR_RESOLVED to dataflash
 * 
 * @note Called from scheduler at ~10Hz to monitor terrain data availability
 * @note Only triggers in modes that actually need terrain data for safe operation
 * 
 * @warning Missing terrain data in terrain-following modes can lead to ground collision
 * 
 * @see failsafe_terrain_set_status() for data status updates
 * @see failsafe_terrain_on_event() for failsafe actions
 * @see AP_Terrain for terrain data management
 * 
 * Source: ArduCopter/events.cpp:243-258
 */
void Copter::failsafe_terrain_check()
{
    // Calculate if failure duration exceeds timeout threshold
    // Duration = time between first and last failure (handles intermittent data loss)
    bool timeout = (failsafe.terrain_last_failure_ms - failsafe.terrain_first_failure_ms) > FS_TERRAIN_TIMEOUT_MS;
    
    // Only trigger failsafe if timeout occurred AND mode requires terrain data
    bool trigger_event = timeout && flightmode->requires_terrain_failsafe();

    // State machine: Check for failsafe state changes (trigger or recovery)
    if (trigger_event != failsafe.terrain) {
        if (trigger_event) {
            // State transition: Normal → Failsafe (terrain data timeout)
            failsafe_terrain_on_event();
        } else {
            // State transition: Failsafe → Normal (terrain data restored)
            LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_TERRAIN, LogErrorCode::ERROR_RESOLVED);
            failsafe.terrain = false;
        }
    }
}

/**
 * @brief Update terrain data availability status
 * 
 * @details Called by AP_Terrain to report terrain data status (available or missing).
 *          Maintains timestamps tracking the duration of terrain data failures for
 *          use by failsafe_terrain_check(). Uses hysteresis to prevent false triggers
 *          from brief data dropouts.
 *          
 *          Failure Tracking Logic:
 *          - First failure: Records terrain_first_failure_ms timestamp
 *          - Subsequent failures: Updates terrain_last_failure_ms (duration calculation)
 *          - Recovery: Clears timestamps after 100ms of continuous good data
 *          
 *          Hysteresis Behavior:
 *          - Brief data dropouts (<100ms) don't reset failure duration
 *          - Requires 100ms of continuous good data to clear failure state
 *          - Prevents rapid failsafe triggering from intermittent data
 * 
 * @param[in] data_ok true if terrain data is available, false if missing
 * 
 * @note Called by AP_Terrain at data update rate (variable, typically 1-10Hz)
 * @note Timestamps used by failsafe_terrain_check() to calculate failure duration
 * 
 * @see failsafe_terrain_check() for timeout monitoring
 * @see AP_Terrain for terrain data management
 * 
 * Source: ArduCopter/events.cpp:261-278
 */
void Copter::failsafe_terrain_set_status(bool data_ok)
{
    uint32_t now = millis();

    // Track failure duration: Record time of first and latest failures
    if (!data_ok) {
        // Update timestamp of most recent failure
        failsafe.terrain_last_failure_ms = now;
        
        // Record first failure timestamp if this is start of failure period
        if (failsafe.terrain_first_failure_ms == 0) {
            failsafe.terrain_first_failure_ms = now;
        }
    } else {
        // Hysteresis: Clear failures only after 100ms of persistent good data
        // Prevents false triggers from brief dropouts
        if (now - failsafe.terrain_last_failure_ms > 100) {
            failsafe.terrain_last_failure_ms = 0;
            failsafe.terrain_first_failure_ms = 0;
        }
    }
}

/**
 * @brief Handle terrain data failsafe trigger event
 * 
 * @details Called when terrain data has been missing for longer than the timeout
 *          period while in a mode requiring terrain data. Takes action to ensure
 *          safe vehicle operation without terrain altitude information.
 *          
 *          Failsafe Actions (priority order):
 *          1. Disarm if on ground (safest response)
 *          2. If in RTL mode: Restart RTL without terrain following
 *          3. Otherwise: Switch to RTL or Land mode
 *          
 *          Special RTL Handling:
 *          - If already in RTL mode, restarts RTL without terrain following
 *          - Prevents mode change disruption while maintaining safe return
 *          - Falls back to barometric altitude control instead of terrain
 *          
 *          Event Logging:
 *          - Sets failsafe.terrain flag for state tracking
 *          - Logs FAILSAFE_TERRAIN/FAILSAFE_OCCURRED to dataflash
 *          - Sends "Failsafe: Terrain data missing" critical message to GCS
 *          - Triggers AP_Notify for LED/buzzer indication
 * 
 * @note Called from failsafe_terrain_check() when timeout occurs
 * @note Uses CRITICAL severity for GCS message (higher priority than WARNING)
 * 
 * @warning Vehicle switches to barometric altitude control without terrain data
 * @warning May result in ground collision if flying at low altitude over rising terrain
 * 
 * @see failsafe_terrain_check() for timeout monitoring
 * @see mode_rtl.restart_without_terrain() for RTL handling
 * 
 * Source: ArduCopter/events.cpp:281-296
 */
void Copter::failsafe_terrain_on_event()
{
    // Set terrain failsafe state flag
    failsafe.terrain = true;
    
    // Notify GCS with critical severity (terrain loss is high-priority safety issue)
    gcs().send_text(MAV_SEVERITY_CRITICAL,"Failsafe: Terrain data missing");
    
    // Log terrain failsafe event to dataflash for post-flight analysis
    LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_TERRAIN, LogErrorCode::FAILSAFE_OCCURRED);

    // Determine appropriate action based on vehicle state
    if (should_disarm_on_failsafe()) {
        // Safety: Disarm immediately if on ground with zero throttle
        arming.disarm(AP_Arming::Method::TERRAINFAILSAFE);
#if MODE_RTL_ENABLED
    } else if (flightmode->mode_number() == Mode::Number::RTL) {
        // Special handling: If already in RTL, restart without terrain following
        // Maintains RTL behavior but uses barometric altitude instead of terrain
        mode_rtl.restart_without_terrain();
#endif
    } else {
        // Standard action: Switch to RTL or Land mode for safe return/descent
        set_mode_RTL_or_land_with_pause(ModeReason::TERRAIN_FAILSAFE);
    }
}

/**
 * @brief Check for GPS glitch or significant innovation failure
 * 
 * @details Monitors GPS health by checking if AHRS/EKF has detected GPS glitching.
 *          A GPS glitch occurs when GPS position innovations (difference between
 *          predicted and measured position) exceed acceptable thresholds, indicating
 *          GPS measurement errors or multipath interference. Does not trigger failsafe
 *          action (EKF handles by rejecting bad GPS data) but logs and notifies for awareness.
 *          
 *          GPS Glitch Detection (performed in AP_AHRS/EKF):
 *          - Innovation magnitude exceeds threshold (typically 5m)
 *          - Sudden position jumps that don't match IMU integration
 *          - Velocity inconsistencies between GPS and IMU
 *          - Multiple consecutive innovation failures
 *          
 *          Glitch Handling:
 *          - EKF rejects glitching GPS measurements automatically
 *          - Falls back to IMU dead reckoning temporarily
 *          - Vehicle continues in current mode (no mode change)
 *          - May eventually trigger dead reckoning failsafe if prolonged
 *          
 *          Event Logging:
 *          - Logs GPS/GPS_GLITCH when glitching starts
 *          - Logs GPS/ERROR_RESOLVED when glitch clears
 *          - Sends critical GCS messages for operator awareness
 *          - AP_Notify LED patterns updated by AP_AHRS automatically
 *          
 *          Common Causes:
 *          - Urban canyon multipath (buildings reflecting GPS signals)
 *          - EMI from high-power transmitters
 *          - Poor antenna placement or shielding
 *          - Satellite constellation geometry changes
 *          - Compass errors detected simultaneously
 * 
 * @note Called from scheduler at main loop rate (~400Hz for Copter)
 * @note Does NOT trigger mode change - EKF handles GPS glitches internally
 * @note Prolonged glitching may lead to dead reckoning failsafe
 * 
 * @warning GPS glitches can lead to position estimate drift if prolonged
 * @warning Message says "GPS Glitch or Compass error" - glitch detection may be due to compass issues
 * 
 * @see AP_AHRS::has_status() for glitch detection
 * @see failsafe_deadreckon_check() for dead reckoning timeout
 * @see AP_NavEKF for innovation checking logic
 * 
 * Source: ArduCopter/events.cpp:299-314
 */
void Copter::gpsglitch_check()
{
    // Query AHRS/EKF for GPS glitching status (innovation check failures)
    const bool gps_glitching = AP::ahrs().has_status(AP_AHRS::Status::GPS_GLITCHING);

    // State machine: Detect glitch state changes and log transitions
    // Note: AP_Notify LED/buzzer updates handled automatically within AP_AHRS
    if (ap.gps_glitching != gps_glitching) {
        // Update state flag
        ap.gps_glitching = gps_glitching;
        
        if (gps_glitching) {
            // State transition: Normal → Glitching
            // Log glitch event for post-flight analysis (correlate with position estimate errors)
            LOGGER_WRITE_ERROR(LogErrorSubsystem::GPS, LogErrorCode::GPS_GLITCH);
            
            // Notify operator - glitch may be GPS or compass error (EKF can't always distinguish)
            gcs().send_text(MAV_SEVERITY_CRITICAL,"GPS Glitch or Compass error");
        } else {
            // State transition: Glitching → Normal
            // Log recovery for analysis (glitch duration calculation)
            LOGGER_WRITE_ERROR(LogErrorSubsystem::GPS, LogErrorCode::ERROR_RESOLVED);
            
            // Notify operator that glitch has cleared
            gcs().send_text(MAV_SEVERITY_CRITICAL,"Glitch cleared");
        }
    }
}

/**
 * @brief Check for dead reckoning timeout and trigger failsafe if needed
 * 
 * @details Monitors EKF dead reckoning status and triggers failsafe if position estimate
 *          relies solely on IMU integration (no GPS, no optical flow, no external nav)
 *          for longer than FS_DR_TIMEOUT. Dead reckoning occurs when EKF loses all
 *          external position references and must integrate IMU data alone, which
 *          accumulates drift over time.
 *          
 *          Dead Reckoning Causes:
 *          - GPS signal loss (indoors, urban canyon, jamming)
 *          - GPS glitch rejection for extended period
 *          - Optical flow sensor failure or loss of visual features
 *          - External navigation source (vision, beacon) unavailable
 *          
 *          Timeout Monitoring:
 *          - Starts timer when EKF enters dead reckoning mode
 *          - Timeout configured by FS_DR_TIMEOUT parameter (default 15 seconds)
 *          - Alerts operator when dead reckoning starts and when timeout occurs
 *          - Clears timer and timeout flag when position reference restored
 *          
 *          Failsafe Action Configuration (FS_DR_ENABLE parameter):
 *          - 0: Disabled (alerts only, no mode change)
 *          - 1+: Action code (LAND, RTL, SMARTRTL, etc.) taken on timeout
 *          - Only triggers if in a mode requiring GPS (AUTO, GUIDED, LOITER, etc.)
 *          - Does not trigger in modes that don't need position (STABILIZE, ACRO, ALT_HOLD)
 *          
 *          Position Drift Characteristics:
 *          - Horizontal drift: Typically 1-5 m/s without GPS
 *          - Drift rate depends on IMU quality and calibration
 *          - Wind and acceleration increase drift rate
 *          - Longer dead reckoning = more position uncertainty
 *          
 *          Event Logging:
 *          - Logs FAILSAFE_DEADRECKON/FAILSAFE_OCCURRED on action trigger
 *          - Sends "Dead Reckoning started/stopped/timeout" critical messages
 *          - No dataflash log for start/stop (info only)
 * 
 * @note Called from scheduler at ~10Hz
 * @note Alerts sent at dead reckoning start, timeout, and stop
 * @note Only takes failsafe action if mode requires position estimate
 * 
 * @warning Dead reckoning position estimate drifts rapidly (1-5 m/s typical)
 * @warning Extended dead reckoning can lead to large position errors
 * @warning Vehicle may be far from believed position after prolonged dead reckoning
 * 
 * @see AP_AHRS::has_status() for dead reckoning detection
 * @see AP_NavEKF for position estimate fusion
 * @see FS_DR_ENABLE, FS_DR_TIMEOUT parameters
 * 
 * Source: ArduCopter/events.cpp:317-374
 */
void Copter::failsafe_deadreckon_check()
{
    // Message prefix for dead reckoning notifications
    const char* dr_prefix_str = "Dead Reckoning";

    // Query EKF for dead reckoning status (no external position references)
    const bool ekf_dead_reckoning = AP::ahrs().has_status(AP_AHRS::Status::DEAD_RECKONING);

    // Alert user and track timing for dead reckoning state changes
    const uint32_t now_ms = AP_HAL::millis();
    if (dead_reckoning.active != ekf_dead_reckoning) {
        // State transition detected
        dead_reckoning.active = ekf_dead_reckoning;
        
        if (dead_reckoning.active) {
            // State transition: Normal → Dead Reckoning
            // Start timeout timer for failsafe monitoring
            dead_reckoning.start_ms = now_ms;
            gcs().send_text(MAV_SEVERITY_CRITICAL,"%s started", dr_prefix_str);
        } else {
            // State transition: Dead Reckoning → Normal
            // Clear timer and timeout flag - position reference restored
            dead_reckoning.start_ms = 0;
            dead_reckoning.timeout = false;
            gcs().send_text(MAV_SEVERITY_CRITICAL,"%s stopped", dr_prefix_str);
        }
    }

    // Monitor for dead reckoning timeout if currently active
    if (dead_reckoning.active && !dead_reckoning.timeout) {
        // Convert FS_DR_TIMEOUT parameter from seconds to milliseconds
        const uint32_t dr_timeout_ms = uint32_t(constrain_float(g2.failsafe_dr_timeout * 1000.0f, 0.0f, UINT32_MAX));
        
        // Check if dead reckoning has exceeded timeout threshold
        if (now_ms - dead_reckoning.start_ms > dr_timeout_ms) {
            dead_reckoning.timeout = true;
            gcs().send_text(MAV_SEVERITY_CRITICAL,"%s timeout", dr_prefix_str);
        }
    }

    // Early exit if dead reckoning failsafe is disabled (FS_DR_ENABLE = 0)
    // Vehicle will alert but not take action
    if (g2.failsafe_dr_enable <= 0) {
        failsafe.deadreckon = false;
        return;
    }

    // Check for failsafe state transition and take action if needed
    if (failsafe.deadreckon != ekf_dead_reckoning) {
        failsafe.deadreckon = ekf_dead_reckoning;

        // Only trigger failsafe action in modes that require position estimate
        // Modes like STABILIZE, ACRO, ALT_HOLD don't need GPS so dead reckoning is not critical
        if (failsafe.deadreckon && copter.flightmode->requires_GPS()) {

            // Log dead reckoning failsafe event to dataflash
            LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_DEADRECKON, LogErrorCode::FAILSAFE_OCCURRED);

            // Safety: Disarm immediately if on ground with zero throttle
            if (should_disarm_on_failsafe()) {
                arming.disarm(AP_Arming::Method::DEADRECKON_FAILSAFE);
                return;
            }

            // Execute user-configured failsafe action (from FS_DR_ENABLE parameter)
            do_failsafe_action((FailsafeAction)g2.failsafe_dr_enable.get(), ModeReason::DEADRECKON_FAILSAFE);
        }
    }
}

/**
 * @brief Attempt RTL mode, fall back to Land mode if RTL unavailable
 * 
 * @details Failsafe helper function that attempts to switch to RTL (Return to Launch)
 *          mode, and if RTL is not available or fails, falls back to Land mode with
 *          a 4 second pause before descent starts. This is the most common failsafe
 *          action as it attempts to bring the vehicle home, but ensures safe landing
 *          if home return is not possible.
 *          
 *          RTL Availability Requirements:
 *          - MODE_RTL_ENABLED must be compiled in (board-dependent)
 *          - Home position must be set (from GPS)
 *          - EKF position estimate must be healthy
 *          - Battery sufficient for estimated return (if BATT_RTL_VOLT set)
 *          
 *          Land Fallback Reasons:
 *          - RTL mode not compiled for this board
 *          - Home position not set (no GPS lock at arming)
 *          - EKF position estimate unhealthy
 *          - Battery insufficient for home return
 *          - GPS signal lost during failsafe
 *          
 *          Notification Behavior:
 *          - Sets AP_Notify failsafe_mode_change flag for LED/buzzer
 *          - Triggers distinctive failsafe notification pattern
 *          - GCS receives mode change reason in HEARTBEAT and mode change events
 * 
 * @param[in] reason Mode change reason for logging (RADIO_FAILSAFE, GCS_FAILSAFE, etc.)
 * 
 * @note Always called from failsafe context, never during normal operations
 * @note Land mode has 4 second pause at current altitude before descending
 * 
 * @warning RTL requires good position estimate - may fail during GPS issues
 * 
 * @see set_mode_land_with_pause() for land fallback implementation
 * @see Mode::Number::RTL for RTL mode implementation
 * @see ModeReason enum for mode change reasons
 * 
 * Source: ArduCopter/events.cpp:376-389
 */
void Copter::set_mode_RTL_or_land_with_pause(ModeReason reason)
{
#if MODE_RTL_ENABLED
    // Attempt to switch to RTL mode (requires home position and good position estimate)
    if (set_mode(Mode::Number::RTL, reason)) {
        // RTL successful - trigger failsafe notification for LED/buzzer pattern
        AP_Notify::events.failsafe_mode_change = 1;
        return;
    }
#endif
    // RTL failed or not available - fall back to landing at current position
    // Land mode will trigger its own mode change notification to pilot
    set_mode_land_with_pause(reason);
}

/**
 * @brief Attempt Smart RTL mode, fall back to Land mode if unavailable
 * 
 * @details Failsafe helper function that attempts to switch to Smart RTL mode,
 *          which retraces the vehicle's actual flight path back to home. If Smart RTL
 *          is not available, falls back directly to Land mode with a 4 second pause.
 *          This provides the safest return path but sacrifices battery efficiency
 *          compared to direct RTL.
 *          
 *          Smart RTL Advantages:
 *          - Retraces actual flight path, avoiding obstacles encountered during flight
 *          - Returns through proven safe airspace
 *          - Automatically simplifies path to reduce waypoint count
 *          - Ideal for flights through complex terrain or around obstacles
 *          
 *          Smart RTL Availability Requirements:
 *          - MODE_SMARTRTL_ENABLED must be compiled in (board-dependent)
 *          - Path must be recorded (requires sufficient memory)
 *          - Minimum path length recorded (typically ~10m)
 *          - Path not pruned due to memory constraints
 *          - Good position estimate for path following
 *          
 *          Land Fallback Reasons:
 *          - Smart RTL mode not compiled for this board
 *          - No path recorded (armed without movement or insufficient memory)
 *          - Path too short or completely pruned
 *          - Memory full, path recording stopped
 *          - Position estimate degraded
 *          
 *          Notification Behavior:
 *          - Success: Sets failsafe_mode_change flag for LED/buzzer pattern
 *          - Failure: Sends \"SmartRTL Unavailable\" warning to GCS before landing
 * 
 * @param[in] reason Mode change reason for logging (RADIO_FAILSAFE, BATTERY_FAILSAFE, etc.)
 * 
 * @note Smart RTL uses more battery than direct RTL due to path retracing
 * @note Always called from failsafe context
 * 
 * @warning Smart RTL requires sufficient memory for path storage (limited on some boards)
 * 
 * @see set_mode_land_with_pause() for land fallback
 * @see Mode::Number::SMART_RTL for Smart RTL implementation
 * @see AP_SmartRTL for path recording and simplification
 * 
 * Source: ArduCopter/events.cpp:391-404
 */
void Copter::set_mode_SmartRTL_or_land_with_pause(ModeReason reason)
{
#if MODE_SMARTRTL_ENABLED
    // Attempt to switch to Smart RTL mode (requires recorded path)
    if (set_mode(Mode::Number::SMART_RTL, reason)) {
        // Smart RTL successful - trigger failsafe notification
        AP_Notify::events.failsafe_mode_change = 1;
        return;
    }
#endif
    // Smart RTL failed or not available - inform GCS and land at current position
    gcs().send_text(MAV_SEVERITY_WARNING, "SmartRTL Unavailable, Using Land Mode");
    set_mode_land_with_pause(reason);
}

/**
 * @brief Attempt Smart RTL, then RTL, then Land mode in sequence
 * 
 * @details Failsafe helper function that attempts failsafe actions in order of
 *          preference: Smart RTL (safest path), standard RTL (direct return), or
 *          Land (last resort). This provides the most comprehensive failsafe cascade,
 *          attempting to return home via the safest available method.
 *          
 *          Action Cascade:
 *          1. Smart RTL: Retrace actual flight path (safest, most battery)
 *          2. RTL: Direct return to home (faster, less battery)
 *          3. Land: Land at current position (last resort)
 *          
 *          Use Cases:
 *          - Battery failsafe where return home is preferred over immediate landing
 *          - Radio failsafe configured for safest return behavior
 *          - Situations where obstacle avoidance is important
 *          - Long-range missions requiring path retracing
 *          
 *          Mode Selection Logic:
 *          - Tries Smart RTL first if path available
 *          - Falls back to RTL if Smart RTL unavailable but home position set
 *          - Falls back to Land if neither RTL mode available
 *          
 *          Notification Behavior:
 *          - Success: Sets failsafe_mode_change flag for LED/buzzer
 *          - Smart RTL fail: Sends \"SmartRTL Unavailable, Trying RTL Mode\" to GCS
 *          - RTL also fail: Handled by set_mode_RTL_or_land_with_pause()
 * 
 * @param[in] reason Mode change reason for logging (typically RADIO_FAILSAFE or BATTERY_FAILSAFE)
 * 
 * @note Attempts modes in order: Smart RTL → RTL → Land
 * @note Always called from failsafe context
 * @note Prefers safety (Smart RTL path) over efficiency (direct RTL)
 * 
 * @see set_mode_RTL_or_land_with_pause() for RTL fallback handling
 * @see Mode::Number::SMART_RTL for Smart RTL implementation
 * 
 * Source: ArduCopter/events.cpp:406-420
 */
void Copter::set_mode_SmartRTL_or_RTL(ModeReason reason)
{
#if MODE_SMARTRTL_ENABLED
    // First attempt: Smart RTL to retrace flight path (safest but uses more battery)
    if (set_mode(Mode::Number::SMART_RTL, reason)) {
        // Smart RTL successful - trigger failsafe notification
        AP_Notify::events.failsafe_mode_change = 1;
        return;
    }
#endif
    // Smart RTL failed or unavailable - inform GCS and try standard RTL
    gcs().send_text(MAV_SEVERITY_WARNING, "SmartRTL Unavailable, Trying RTL Mode");
    // Cascade to RTL, then Land if RTL also fails
    set_mode_RTL_or_land_with_pause(reason);
}

/**
 * @brief Jump to DO_LAND_START mission item in AUTO mode, or fall back to RTL
 * 
 * @details Failsafe helper function that attempts to switch to AUTO mode and jump
 *          to the mission item specified by the AUTO_RTL parameter (typically a
 *          DO_LAND_START command). This allows missions to have predefined landing
 *          sequences for failsafes, such as landing at a safe designated location
 *          rather than the home position.
 *          
 *          AUTO_RTL Behavior:
 *          - Searches mission for DO_LAND_START waypoint
 *          - Jumps directly to that waypoint, skipping remaining mission items
 *          - Executes landing sequence from that point
 *          - Allows complex approach procedures (pattern, final approach, land)
 *          - Can specify alternate landing sites (not just home)
 *          
 *          Use Cases:
 *          - Missions with designated safe landing zones
 *          - Complex landing patterns (pattern entry, downwind, final)
 *          - Alternate landing sites based on mission phase
 *          - Commercial operations with specific landing procedures
 *          - Airports or helipads with approach requirements
 *          
 *          AUTO_RTL Mode Availability Requirements:
 *          - MODE_AUTO_ENABLED compiled in
 *          - Valid mission loaded
 *          - DO_LAND_START waypoint in mission (AUTO_RTL parameter)
 *          - Good position estimate for waypoint navigation
 *          
 *          RTL Fallback Reasons:
 *          - AUTO mode not compiled
 *          - No mission loaded
 *          - AUTO_RTL parameter not set or invalid
 *          - DO_LAND_START waypoint not found in mission
 *          - Position estimate degraded
 *          
 *          This function can be called from:
 *          - Failsafe events (radio, GCS, battery)
 *          - RC auxiliary function switch
 * 
 * @param[in] reason Mode change reason for logging
 * 
 * @note AUTO_RTL allows mission-specific landing procedures vs generic RTL
 * @note Can be triggered by failsafe or manual RC switch
 * 
 * @see Mode::Number::AUTO_RTL for AUTO mode DO_LAND_START implementation
 * @see AUTO_RTL parameter for waypoint configuration
 * @see set_mode_RTL_or_land_with_pause() for fallback
 * 
 * Source: ArduCopter/events.cpp:422-435
 */
void Copter::set_mode_auto_do_land_start_or_RTL(ModeReason reason)
{
#if MODE_AUTO_ENABLED
    // Attempt to switch to AUTO mode and jump to DO_LAND_START waypoint
    if (set_mode(Mode::Number::AUTO_RTL, reason)) {
        // AUTO_RTL successful - trigger failsafe notification
        AP_Notify::events.failsafe_mode_change = 1;
        return;
    }
#endif

    // AUTO_RTL failed - inform GCS and fall back to standard RTL
    gcs().send_text(MAV_SEVERITY_WARNING, "Trying RTL Mode");
    set_mode_RTL_or_land_with_pause(reason);
}

/**
 * @brief Attempt Brake mode to stop, then fall back to Land mode
 * 
 * @details Failsafe helper function that attempts to switch to Brake mode to rapidly
 *          decelerate the vehicle to a stop, or falls back to Land mode if Brake is
 *          unavailable. Brake mode aggressively arrests all horizontal movement before
 *          switching to Loiter or Land, making it useful for failsafes where stopping
 *          quickly is more important than returning home.
 *          
 *          Brake Mode Behavior:
 *          - Applies maximum braking (full opposing thrust)
 *          - Decelerates horizontal velocity to zero quickly
 *          - Maintains current altitude during braking
 *          - Automatically transitions to Loiter when stopped
 *          - If failsafe persists, will eventually transition to Land
 *          
 *          Use Cases:
 *          - Indoor flights where RTL would hit ceiling or walls
 *          - Flights near obstacles where stopping is safer than moving
 *          - Situations where current position is acceptable landing site
 *          - Quick response needed to prevent flying away
 *          - Limited space available for return maneuvers
 *          
 *          Brake Mode Availability Requirements:
 *          - MODE_BRAKE_ENABLED compiled in (board-dependent)
 *          - Vehicle has horizontal velocity capability
 *          - Good position estimate for velocity control
 *          
 *          Land Fallback Reasons:
 *          - Brake mode not compiled for this board
 *          - Already stopped (no velocity to brake)
 *          - Position estimate unavailable
 *          
 *          This function can be called from:
 *          - Failsafe events (radio, GCS)
 *          - RC auxiliary function switch
 * 
 * @param[in] reason Mode change reason for logging
 * 
 * @note Brake provides rapid deceleration before landing
 * @note Brake transitions to Loiter after stopping, then Land if failsafe continues
 * @note Useful for indoor or confined spaces where RTL is not appropriate
 * 
 * @see Mode::Number::BRAKE for Brake mode implementation
 * @see set_mode_land_with_pause() for fallback
 * 
 * Source: ArduCopter/events.cpp:437-450
 */
void Copter::set_mode_brake_or_land_with_pause(ModeReason reason)
{
#if MODE_BRAKE_ENABLED
    // Attempt to switch to Brake mode to rapidly decelerate
    if (set_mode(Mode::Number::BRAKE, reason)) {
        // Brake successful - trigger failsafe notification
        AP_Notify::events.failsafe_mode_change = 1;
        return;
    }
#endif

    // Brake failed or unavailable - fall back to landing immediately
    gcs().send_text(MAV_SEVERITY_WARNING, "Trying Land Mode");
    set_mode_land_with_pause(reason);
}

/**
 * @brief Determine if vehicle should disarm immediately on failsafe
 * 
 * @details Safety check that determines whether a failsafe should immediately disarm
 *          the vehicle rather than changing flight modes. This prevents unnecessary
 *          mode changes when the vehicle is safely on the ground, and avoids potential
 *          safety issues from mode changes during arming/disarming sequences.
 *          
 *          Decision Logic by Flight Mode:
 *          
 *          STABILIZE/ACRO:
 *          - Disarm if throttle at zero (pilot not commanding thrust)
 *          - Disarm if landed (land detector confirms on ground)
 *          - Rationale: Manual modes, if throttle zero or landed, vehicle is safe
 *          
 *          AUTO/AUTO_RTL:
 *          - Disarm if mission not started (not auto-armed) AND landed
 *          - Rationale: Allow mission to continue if already flying, but disarm if
 *            failsafe occurs before takeoff
 *          
 *          All Other Modes (AltHold, Guided, Loiter, RTL, etc.):
 *          - Disarm if landed (land detector confirms on ground)
 *          - Rationale: If on ground, disarming is safer than mode change
 *          
 *          Arming Delay Special Case:
 *          - Always disarm if in arming delay period
 *          - Prevents failsafe mode changes during the brief period between
 *            arming command and actual motor start
 *          - Safety: Vehicle should not attempt flight during arming sequence
 *          
 *          State Checks:
 *          - ap.in_arming_delay: In arming sequence (pre-spinup delay)
 *          - ap.throttle_zero: Throttle stick at minimum position
 *          - ap.land_complete: Land detector confirms on ground
 *          - ap.auto_armed: AUTO mode has started mission (takeoff complete)
 * 
 * @return true if vehicle should disarm immediately on failsafe
 * @return false if vehicle should change mode instead of disarming
 * 
 * @note Used by all failsafe handlers before executing failsafe action
 * @note Prevents unnecessary mode changes when vehicle safely on ground
 * 
 * @warning Disarming immediately is safer than mode change when on ground
 * 
 * @see ap flags in Copter.h for state tracking
 * @see do_failsafe_action() for failsafe execution
 * 
 * Source: ArduCopter/events.cpp:452-471
 */
bool Copter::should_disarm_on_failsafe() {
    // Special case: Always disarm during arming delay to prevent flight during arming sequence
    if (ap.in_arming_delay) {
        return true;
    }

    // Check landing state and throttle position based on current flight mode
    switch (flightmode->mode_number()) {
        case Mode::Number::STABILIZE:
        case Mode::Number::ACRO:
            // Manual modes: Disarm if throttle at zero OR vehicle confirmed landed
            // Pilot has no throttle input, so vehicle should be safe to disarm
            return ap.throttle_zero || ap.land_complete;
            
        case Mode::Number::AUTO:
        case Mode::Number::AUTO_RTL:
            // Autonomous modes: Only disarm if mission hasn't started AND vehicle is landed
            // Allows in-flight failsafes to trigger mode changes, but disarms pre-flight failsafes
            return !ap.auto_armed && ap.land_complete;
            
        default:
            // Altitude-holding modes (AltHold, Guided, Loiter, RTL, Circle, Drift, Sport, Flip, Autotune, PosHold):
            // Disarm if land detector confirms vehicle is on ground
            // These modes maintain altitude, so if landed, safe to disarm
            return ap.land_complete;
    }
}


/**
 * @brief Execute specified failsafe action with appropriate fallback handling
 * 
 * @details Central failsafe action dispatcher that executes the requested failsafe
 *          response. This function is called by all failsafe event handlers after
 *          they determine the appropriate action based on parameters and vehicle state.
 *          Each action includes intelligent fallback behavior if the primary action
 *          is unavailable.
 *          
 *          Failsafe Actions and Behavior:
 *          
 *          NONE: No action taken
 *          - Used when failsafe is disabled or special handling applies
 *          - Vehicle continues in current mode
 *          
 *          LAND: Land immediately at current location
 *          - Calls set_mode_land_with_pause() for 4 second hover before descent
 *          - Used for low battery or when return home not viable
 *          
 *          RTL: Return to launch, fallback to Land
 *          - Attempts direct return to home position
 *          - Falls back to Land if RTL unavailable (no home, no GPS, etc.)
 *          
 *          SMARTRTL: Smart RTL → Regular RTL → Land cascade
 *          - Retraces flight path (safest)
 *          - Falls back to direct RTL if path unavailable
 *          - Falls back to Land if RTL also unavailable
 *          
 *          SMARTRTL_LAND: Smart RTL → Land cascade (skips regular RTL)
 *          - Retraces flight path or lands immediately
 *          - No intermediate RTL fallback
 *          
 *          AUTO_DO_LAND_START: Jump to DO_LAND_START → RTL → Land
 *          - Executes mission-defined landing sequence
 *          - Falls back to RTL if mission unavailable
 *          - Falls back to Land if RTL also unavailable
 *          
 *          BRAKE_LAND: Brake to stop → Land
 *          - Rapidly decelerates to zero velocity
 *          - Falls back to Land if Brake unavailable
 *          
 *          TERMINATE: Emergency termination (advanced failsafe only)
 *          - If AP_COPTER_ADVANCED_FAILSAFE_ENABLED: Cuts motors via AFS
 *          - Otherwise: Immediate disarm
 *          - EXTREME: Vehicle will fall from sky - only for lost vehicle scenarios
 *          
 *          Additional Failsafe Options:
 *          - RELEASE_GRIPPER: If FS_OPTIONS bit set, releases gripper on any failsafe
 *            Useful for dropping payloads in emergency or preventing grip damage on landing
 *          
 *          Event Logging:
 *          - Mode change logged with specific reason (RADIO_FAILSAFE, BATTERY_FAILSAFE, etc.)
 *          - Allows post-flight analysis of failsafe cascade and outcomes
 *          - GCS receives mode change notification with reason code
 * 
 * @param[in] action Failsafe action to execute (from FailsafeAction enum)
 * @param[in] reason Mode change reason for logging (from ModeReason enum)
 * 
 * @note Called by all failsafe event handlers after determining action
 * @note Each action includes intelligent fallback if primary action unavailable
 * @note TERMINATE action is extremely dangerous - vehicle will fall
 * 
 * @warning TERMINATE action cuts motors immediately - vehicle will crash
 * @warning Only use TERMINATE for lost vehicle or extreme safety situations
 * 
 * @see FailsafeAction enum in Copter.h for action definitions
 * @see ModeReason enum for mode change reason codes
 * @see set_mode_*() helper functions for mode change implementation
 * 
 * Source: ArduCopter/events.cpp:474-513
 */
void Copter::do_failsafe_action(FailsafeAction action, ModeReason reason){

    // Execute the specified failsafe action with appropriate fallback handling
    switch (action) {
        case FailsafeAction::NONE:
            // No action - continue in current mode (e.g., failsafe disabled or special handling)
            return;
            
        case FailsafeAction::LAND:
            // Land immediately at current location with 4 second pause
            set_mode_land_with_pause(reason);
            break;
            
        case FailsafeAction::RTL:
            // Return to Launch, fall back to Land if RTL unavailable
            set_mode_RTL_or_land_with_pause(reason);
            break;
            
        case FailsafeAction::SMARTRTL:
            // Smart RTL (retrace path) → RTL (direct) → Land cascade
            set_mode_SmartRTL_or_RTL(reason);
            break;
            
        case FailsafeAction::SMARTRTL_LAND:
            // Smart RTL (retrace path) → Land cascade (skips regular RTL)
            set_mode_SmartRTL_or_land_with_pause(reason);
            break;
            
        case FailsafeAction::TERMINATE: {
            // EXTREME: Emergency termination - cuts motors immediately
            // Vehicle will fall from sky - only for lost vehicle scenarios
#if AP_COPTER_ADVANCED_FAILSAFE_ENABLED
            // Advanced Failsafe: Controlled termination via AFS module
            g2.afs.gcs_terminate(true, "Failsafe");
#else
            // Standard: Immediate disarm
            arming.disarm(AP_Arming::Method::FAILSAFE_ACTION_TERMINATE);
#endif
            break;
        }
        
        case FailsafeAction::AUTO_DO_LAND_START:
            // Jump to DO_LAND_START mission item → RTL → Land cascade
            set_mode_auto_do_land_start_or_RTL(reason);
            break;
            
        case FailsafeAction::BRAKE_LAND:
            // Brake to stop horizontal movement → Land
            set_mode_brake_or_land_with_pause(reason);
            break;
    }

    // Optional: Release gripper on failsafe if FS_OPTIONS bit is set
    // Useful for dropping payloads in emergency or preventing damage on landing
#if AP_GRIPPER_ENABLED
    if (failsafe_option(FailsafeOption::RELEASE_GRIPPER)) {
        gripper.release();
    }
#endif
}

