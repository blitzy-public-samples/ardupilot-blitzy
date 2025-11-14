/**
 * @file events.cpp
 * @brief Failsafe event handling for Blimp airship vehicle
 * 
 * @details This file implements the failsafe event handling system for the Blimp vehicle,
 *          including radio failsafe, battery failsafe, GCS (Ground Control Station) failsafe,
 *          and GPS glitch detection. The failsafe system monitors multiple critical subsystems
 *          and triggers appropriate recovery actions (land, disarm, terminate) when failures
 *          are detected.
 *          
 *          Key Failsafe Types:
 *          - Radio Failsafe: Triggered when RC transmitter signal is lost
 *          - Battery Failsafe: Triggered when battery voltage/capacity reaches critical levels
 *          - GCS Failsafe: Triggered when ground station telemetry link is lost
 *          - GPS Glitch: Detected when GPS position estimates become unreliable
 *          
 *          Failsafe Actions:
 *          - Failsafe_Action_None: Continue current operation
 *          - Failsafe_Action_Land: Initiate controlled landing
 *          - Failsafe_Action_Terminate: Immediate disarm (emergency only)
 *          
 *          Blimp-Specific Behavior:
 *          Unlike multirotors, the Blimp has inherent buoyancy which provides an additional
 *          safety margin during failsafe events. The vehicle will naturally maintain altitude
 *          when motors are disarmed, allowing safer recovery from failsafe conditions.
 *          
 * @warning All functions in this file are SAFETY-CRITICAL. Failsafe behavior directly
 *          affects vehicle safety and must be thoroughly tested before deployment.
 *          Incorrect failsafe handling can result in vehicle damage or injury.
 * 
 * @see Blimp.h for main vehicle class
 * @see AP_Arming for arming/disarming logic
 * @see mode.h for flight mode definitions
 * 
 * Source: Blimp/events.cpp
 */

#include "Blimp.h"

/*
 *       This event will be called when the failsafe changes
 *       boolean failsafe reflects the current state
 */

#include <AP_Vehicle/AP_MultiCopter.h>

/**
 * @brief Check if a specific failsafe option is enabled
 * 
 * @details Tests whether a particular failsafe behavior option is enabled in the
 *          FS_OPTIONS parameter (g2.fs_options). Failsafe options allow users to
 *          customize failsafe behavior, such as continuing landing during failsafe
 *          or GCS failsafe timeout configuration.
 * 
 * @param[in] opt The failsafe option to check (from FailsafeOption enum)
 * 
 * @return true if the specified option is enabled, false otherwise
 * 
 * @note This is a const function and does not modify vehicle state
 * @see FailsafeOption enum for available options
 * 
 * Source: Blimp/events.cpp:10-13
 */
bool Blimp::failsafe_option(FailsafeOption opt) const
{
    return (g2.fs_options & (uint32_t)opt);
}

/**
 * @brief Handle radio failsafe trigger event
 * 
 * @details Called when RC transmitter signal is lost for longer than the configured
 *          failsafe timeout. Determines appropriate failsafe action based on:
 *          1. FS_THR_ENABLE parameter configuration
 *          2. Current vehicle state (on ground, landing, airborne)
 *          3. Other active failsafes (battery failsafe interaction)
 *          4. User-configured failsafe options (FS_OPTIONS)
 *          
 *          Failsafe Action Decision Logic:
 *          - If on ground or arming delay: Immediate disarm
 *          - If landing with battery failsafe: Continue landing
 *          - If landing with CONTINUE_IF_LANDING option: Continue landing
 *          - Otherwise: Execute action from FS_THR_ENABLE parameter
 *          
 *          Blimp-Specific Consideration:
 *          Due to inherent buoyancy, the Blimp is safer than multirotors during radio
 *          loss. If disarmed in flight, the vehicle will maintain approximate altitude
 *          rather than fall, providing additional recovery time.
 * 
 * @warning SAFETY-CRITICAL: This function handles loss of pilot control. Incorrect
 *          behavior can result in vehicle damage or injury. All changes must be
 *          thoroughly tested in SITL and on actual hardware.
 * 
 * @note Triggered automatically by RC input monitoring system when signal loss detected
 * @note Logs error to dataflash and sends MAVLink message to GCS
 * @note Recovery is handled by failsafe_radio_off_event() when signal is restored
 * 
 * @see failsafe_radio_off_event() for recovery handling
 * @see do_failsafe_action() for action execution
 * @see should_disarm_on_failsafe() for ground disarm logic
 * @see FS_THR_ENABLE parameter for configuration
 * 
 * Source: Blimp/events.cpp:29-69
 */
void Blimp::failsafe_radio_on_event()
{
    LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_RADIO, LogErrorCode::FAILSAFE_OCCURRED);

    // set desired action based on FS_THR_ENABLE parameter
    Failsafe_Action desired_action;
    switch (g.failsafe_throttle) {
    case FS_THR_DISABLED:
        desired_action = Failsafe_Action_None;
        break;
    case FS_THR_ENABLED_ALWAYS_LAND:
        desired_action = Failsafe_Action_Land;
        break;
    default:
        desired_action = Failsafe_Action_Land;
    }

    // Conditions to deviate from FS_THR_ENABLE selection and send specific GCS warning
    if (should_disarm_on_failsafe()) {
        // should immediately disarm when we're on the ground
        gcs().send_text(MAV_SEVERITY_WARNING, "Radio Failsafe - Disarming");
        arming.disarm(AP_Arming::Method::RADIOFAILSAFE);
        desired_action = Failsafe_Action_None;

    } else if (flightmode->is_landing() && ((battery.has_failsafed() && battery.get_highest_failsafe_priority() <= FAILSAFE_LAND_PRIORITY))) {
        // Allow landing to continue when battery failsafe requires it (not a user option)
        gcs().send_text(MAV_SEVERITY_WARNING, "Radio + Battery Failsafe - Continuing Landing");
        desired_action = Failsafe_Action_Land;

    } else if (flightmode->is_landing() && failsafe_option(FailsafeOption::CONTINUE_IF_LANDING)) {
        // Allow landing to continue when FS_OPTIONS is set to continue landing
        gcs().send_text(MAV_SEVERITY_WARNING, "Radio Failsafe - Continuing Landing");
        desired_action = Failsafe_Action_Land;

    } else {
        gcs().send_text(MAV_SEVERITY_WARNING, "Radio Failsafe");
    }

    // Call the failsafe action handler
    do_failsafe_action(desired_action, ModeReason::RADIO_FAILSAFE);
}

/**
 * @brief Handle radio failsafe recovery event
 * 
 * @details Called when RC transmitter signal is restored after a radio failsafe.
 *          This function logs the recovery and notifies the GCS that pilot control
 *          has been restored.
 *          
 *          Recovery Behavior:
 *          - Logs failsafe resolution to dataflash
 *          - Sends MAVLink notification to GCS
 *          - Pilot immediately regains control authority over roll, pitch, yaw, throttle
 *          - Pilot can use flight mode switch to change modes if desired
 *          - No automatic mode change or other action taken
 *          
 *          Blimp-Specific Consideration:
 *          Thanks to inherent buoyancy, the Blimp typically maintains stable altitude
 *          even during radio loss, making recovery smoother than with multirotors.
 * 
 * @warning SAFETY-CRITICAL: Pilot control is restored by this event. Ensure radio
 *          signal quality is stable before resuming normal operations.
 * 
 * @note Called automatically by RC input monitoring system when valid signal detected
 * @note Does not automatically restore previous flight mode - pilot retains current mode
 * @note Paired with failsafe_radio_on_event() for complete failsafe handling
 * 
 * @see failsafe_radio_on_event() for initial failsafe trigger
 * 
 * Source: Blimp/events.cpp:86-93
 */
// failsafe_off_event - respond to radio contact being regained
void Blimp::failsafe_radio_off_event()
{
    // no need to do anything except log the error as resolved
    // user can now override roll, pitch, yaw and throttle and even use flight mode switch to restore previous flight mode
    LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_RADIO, LogErrorCode::FAILSAFE_RESOLVED);
    gcs().send_text(MAV_SEVERITY_WARNING, "Radio Failsafe Cleared");
}

/**
 * @brief Handle battery failsafe event
 * 
 * @details Called when battery voltage, capacity, or current reaches critical levels
 *          defined by BATT_LOW_VOLT, BATT_CRT_VOLT, BATT_LOW_MAH, BATT_CRT_MAH parameters.
 *          Determines appropriate failsafe action based on:
 *          1. BATT_FS_LOW_ACT or BATT_FS_CRT_ACT parameter (passed as action parameter)
 *          2. Current vehicle state (on ground vs airborne)
 *          3. Current flight mode (landing vs other modes)
 *          4. User-configured failsafe options (FS_OPTIONS)
 *          
 *          Battery Failsafe Levels:
 *          - Low Battery: Warning level, configurable action (typically RTL or Land)
 *          - Critical Battery: Emergency level, typically forces immediate landing
 *          
 *          Failsafe Action Decision Logic:
 *          - If on ground or arming delay: Immediate disarm
 *          - If already landing with CONTINUE_IF_LANDING option: Continue current landing
 *          - Otherwise: Execute configured battery failsafe action
 *          
 *          Blimp-Specific Consideration:
 *          Battery failsafe is especially important for Blimps as maintaining altitude
 *          control requires continuous motor operation. Unlike multirotors which can
 *          auto-rotate, the Blimp's buoyancy helps maintain altitude but fin control
 *          is lost without battery power.
 * 
 * @param[in] type_str String describing battery failsafe type ("Low" or "Critical")
 * @param[in] action   Desired failsafe action from BATT_FS_XXX_ACT parameter
 * 
 * @warning SAFETY-CRITICAL: Battery exhaustion can result in complete loss of control.
 *          Ensure battery monitoring is properly calibrated and failsafe actions are
 *          configured conservatively. Test thoroughly before flight.
 * 
 * @note Called by battery monitoring system when thresholds are exceeded
 * @note Logs error to dataflash and sends MAVLink message to GCS
 * @note Multiple battery failsafe levels can cascade (low → critical)
 * 
 * @see do_failsafe_action() for action execution
 * @see should_disarm_on_failsafe() for ground disarm logic
 * @see BATT_LOW_VOLT, BATT_CRT_VOLT, BATT_FS_LOW_ACT, BATT_FS_CRT_ACT parameters
 * 
 * Source: Blimp/events.cpp:102-124
 */
void Blimp::handle_battery_failsafe(const char *type_str, const int8_t action)
{
    LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_BATT, LogErrorCode::FAILSAFE_OCCURRED);

    Failsafe_Action desired_action = (Failsafe_Action)action;

    // Conditions to deviate from BATT_FS_XXX_ACT parameter setting
    if (should_disarm_on_failsafe()) {
        // should immediately disarm when we're on the ground
        arming.disarm(AP_Arming::Method::BATTERYFAILSAFE);
        desired_action = Failsafe_Action_None;
        gcs().send_text(MAV_SEVERITY_WARNING, "Battery Failsafe - Disarming");

    } else if (flightmode->is_landing() && failsafe_option(FailsafeOption::CONTINUE_IF_LANDING) && desired_action != Failsafe_Action_None) {
        // Allow landing to continue when FS_OPTIONS is set to continue when landing
        desired_action = Failsafe_Action_Land;
        gcs().send_text(MAV_SEVERITY_WARNING, "Battery Failsafe - Continuing Landing");
    } else {
        gcs().send_text(MAV_SEVERITY_WARNING, "Battery Failsafe");
    }

    // Battery FS options already use the Failsafe_Options enum. So use them directly.
    do_failsafe_action(desired_action, ModeReason::BATTERY_FAILSAFE);

}
/**
 * @brief Monitor Ground Control Station connection and handle GCS failsafe
 * 
 * @details Periodically checks the time since last GCS heartbeat message and triggers
 *          GCS failsafe if connection is lost for longer than the configured timeout.
 *          This function implements a state machine to detect both failsafe entry and
 *          recovery conditions.
 *          
 *          GCS Failsafe State Machine:
 *          1. Normal: Recent GCS heartbeat, no failsafe
 *          2. Failsafe Triggered: No GCS heartbeat for > FS_GCS_TIMEOUT seconds
 *          3. Failsafe Recovery: GCS heartbeat restored, failsafe cleared
 *          
 *          Monitoring Logic:
 *          - Tracks MAVLink heartbeat messages from primary GCS system ID
 *          - Compares time since last heartbeat against FS_GCS_TIMEOUT parameter
 *          - Triggers failsafe action (currently disarm) when timeout exceeded
 *          - Clears failsafe flag when connection restored
 *          
 *          Current Implementation Status:
 *          GCS failsafe currently triggers immediate disarm. Full failsafe_gcs_on_event()
 *          and failsafe_gcs_off_event() functions should be implemented in the future
 *          to provide more sophisticated behavior similar to radio failsafe.
 * 
 * @warning SAFETY-CRITICAL: Loss of GCS telemetry means loss of mission monitoring
 *          and situational awareness. Current implementation immediately disarms,
 *          which is conservative but may not be optimal for all flight scenarios.
 * 
 * @note Called at regular intervals from main scheduler (typically 1Hz)
 * @note Only monitors heartbeat from GCS system ID set by gcs().sysid_mygcs()
 * @note Failsafe disabled if FS_GCS_ENABLE = 0 or GCS never connected
 * @note Times are in milliseconds (ms)
 * 
 * @todo Implement full failsafe_gcs_on_event() and failsafe_gcs_off_event() functions
 *       to provide configurable GCS failsafe actions instead of always disarming
 * 
 * @see set_failsafe_gcs() for failsafe state management
 * @see FS_GCS_ENABLE parameter to enable/disable GCS failsafe
 * @see FS_GCS_TIMEOUT parameter for timeout configuration (seconds)
 * 
 * Source: Blimp/events.cpp:139-159
 */
// failsafe_gcs_check - check for ground station failsafe
void Blimp::failsafe_gcs_check()
{
    // Bypass GCS failsafe checks if disabled or GCS never connected
    if (g.failsafe_gcs == FS_GCS_DISABLED) {
        return;
    }

    const uint32_t gcs_last_seen_ms = gcs().sysid_mygcs_last_seen_time_ms();
    if (gcs_last_seen_ms == 0) {
        return;
    }

    // calc time since last gcs update
    // note: this only looks at the heartbeat from the device id set by gcs().sysid_mygcs()
    const uint32_t last_gcs_update_ms = millis() - gcs_last_seen_ms;
    const uint32_t gcs_timeout_ms = uint32_t(constrain_float(g2.fs_gcs_timeout * 1000.0f, 0.0f, UINT32_MAX));

    // Determine which event to trigger
    if (last_gcs_update_ms < gcs_timeout_ms && failsafe.gcs) {
        // Recovery from a GCS failsafe
        set_failsafe_gcs(false);
        // failsafe_gcs_off_event();

    } else if (last_gcs_update_ms < gcs_timeout_ms && !failsafe.gcs) {
        // No problem, do nothing

    } else if (last_gcs_update_ms > gcs_timeout_ms && failsafe.gcs) {
        // Already in failsafe, do nothing

    } else if (last_gcs_update_ms > gcs_timeout_ms && !failsafe.gcs) {
        // New GCS failsafe event, trigger events
        set_failsafe_gcs(true);
        arming.disarm(AP_Arming::Method::GCSFAILSAFE); // failsafe_gcs_on_event() should replace this when written
    }
}

/**
 * @brief Determine if vehicle should disarm immediately on failsafe
 * 
 * @details Checks vehicle state to determine if immediate disarm is the safest failsafe
 *          response. This prevents executing landing or other failsafe actions when the
 *          vehicle is already on the ground or not yet fully armed.
 *          
 *          Disarm Decision Logic:
 *          1. If in arming delay period → Disarm (not yet fully operational)
 *          2. If in MANUAL mode and land_complete → Disarm (on ground, safe to disarm)
 *          3. Otherwise → Don't disarm (airborne, need controlled response)
 *          
 *          Safety Rationale:
 *          When on the ground, disarming is the safest response to any failsafe as it
 *          prevents unintended motor operation. When airborne, disarming would cause
 *          loss of control, so controlled landing or other actions are preferred.
 *          
 *          Blimp-Specific Consideration:
 *          Even when airborne, the Blimp's buoyancy provides a safety margin if disarmed,
 *          but controlled descent via landing mode is still strongly preferred.
 * 
 * @return true if vehicle should disarm immediately, false if controlled failsafe action needed
 * 
 * @warning SAFETY-CRITICAL: This function determines whether to maintain motor control
 *          during failsafe. Incorrect logic could cause either uncontrolled descent or
 *          unexpected motor operation on the ground.
 * 
 * @note Called by failsafe handlers to determine appropriate response
 * @note Returns true during arming delay to prevent flying before fully ready
 * @note Checks ap.land_complete flag to detect ground contact
 * 
 * @see failsafe_radio_on_event() for usage in radio failsafe
 * @see handle_battery_failsafe() for usage in battery failsafe
 * @see ap.land_complete flag set by landing detection system
 * 
 * Source: Blimp/events.cpp:196-207
 */
bool Blimp::should_disarm_on_failsafe()
{
    if (ap.in_arming_delay) {
        return true;
    }

    switch (control_mode) {
    case Mode::Number::MANUAL:
    default:
        // if landed disarm
        return ap.land_complete;
    }
}


/**
 * @brief Execute specified failsafe action
 * 
 * @details Central dispatcher function that executes the determined failsafe action.
 *          Called by all failsafe handlers (radio, battery, GCS) after determining
 *          the appropriate response. Translates high-level failsafe actions into
 *          concrete vehicle commands.
 *          
 *          Supported Failsafe Actions:
 *          
 *          1. Failsafe_Action_None:
 *             - No action taken (failsafe acknowledged but no response needed)
 *             - Used when vehicle already on ground and disarmed
 *          
 *          2. Failsafe_Action_Land:
 *             - Initiate controlled landing sequence
 *             - Switches to LAND flight mode with failsafe flag set
 *             - Provides controlled descent to ground
 *             - Preferred action for airborne failsafes
 *          
 *          3. Failsafe_Action_Terminate:
 *             - Emergency immediate disarm
 *             - Used only in critical situations (rare)
 *             - Causes immediate loss of motor control
 *             - For Blimp: Vehicle will maintain approximate altitude due to buoyancy
 *          
 *          Blimp-Specific Behavior:
 *          Unlike multirotors, the Blimp will not fall rapidly if disarmed during
 *          Failsafe_Action_Terminate. Inherent buoyancy provides a safety margin,
 *          but controlled landing via Failsafe_Action_Land is still strongly preferred.
 * 
 * @param[in] action Desired failsafe action to execute (from Failsafe_Action enum)
 * @param[in] reason Mode change reason for logging and tracking (from ModeReason enum)
 * 
 * @warning SAFETY-CRITICAL: This function directly controls vehicle behavior during
 *          failsafe emergencies. All changes must be thoroughly tested. Incorrect
 *          action execution can result in vehicle damage or injury.
 * 
 * @note Called by failsafe_radio_on_event(), handle_battery_failsafe(), and other failsafe handlers
 * @note Mode change reason is logged for post-flight analysis
 * @note Failsafe_Action_Land uses set_mode_land_failsafe() which sets failsafe flag
 * 
 * @see Failsafe_Action enum for available actions
 * @see ModeReason enum for reason codes
 * @see set_mode_land_failsafe() for landing mode entry
 * @see AP_Arming::disarm() for disarm execution
 * 
 * Source: Blimp/events.cpp:211-226
 */
void Blimp::do_failsafe_action(Failsafe_Action action, ModeReason reason)
{

    // Execute the specified desired_action
    switch (action) {
    case Failsafe_Action_None:
        return;
    case Failsafe_Action_Land:
        set_mode_land_failsafe(reason);
        break;
    case Failsafe_Action_Terminate: {
        arming.disarm(AP_Arming::Method::FAILSAFE_ACTION_TERMINATE);
    }
    break;
    }
}

/**
 * @brief Monitor for GPS position estimation glitches
 * 
 * @details Checks for GPS glitches detected by the AHRS/EKF system and logs/reports
 *          changes in GPS health status. A GPS glitch occurs when GPS position estimates
 *          become inconsistent with inertial navigation or other sensors, indicating
 *          potential GPS multipath, interference, or other reliability issues.
 *          
 *          GPS Glitch Detection:
 *          - Performed by EKF innovation checking in AP_AHRS
 *          - Compares GPS measurements against EKF predictions
 *          - Large innovation (mismatch) triggers glitch flag
 *          - EKF may switch to dead reckoning using inertial sensors
 *          
 *          Notification and Logging:
 *          - Logs glitch start/stop events to dataflash
 *          - Sends critical severity MAVLink messages to GCS
 *          - AP_Notify (LEDs/buzzer) handled automatically by AP_AHRS
 *          - Tracks state changes to avoid repeated notifications
 *          
 *          Blimp Flight Implications:
 *          GPS glitches are particularly important for Blimps operating in GPS-based
 *          modes (Loiter, Auto, etc.). Due to slow dynamics and buoyancy, Blimps
 *          can tolerate brief GPS glitches better than multirotors, but extended
 *          GPS loss will still degrade position hold performance.
 *          
 *          Current Implementation:
 *          This function only logs and notifies about GPS glitches. It does NOT
 *          trigger automatic failsafe actions. Pilots should monitor GPS health
 *          and manually switch to non-GPS modes if glitches persist.
 * 
 * @warning SAFETY-CRITICAL: GPS glitches can cause position estimate errors leading
 *          to unexpected vehicle movement or loss of position hold. Monitor GPS health
 *          carefully, especially in areas with known GPS interference or multipath.
 * 
 * @note Called at regular intervals from main scheduler (typically 10Hz)
 * @note Does not trigger automatic failsafe - notification only
 * @note GPS glitch detection threshold configured in EKF parameters
 * @note State tracked in ap.gps_glitching flag to detect transitions
 * 
 * @todo Consider implementing automatic failsafe action for extended GPS glitches
 *       (e.g., switch to MANUAL mode after sustained GPS unreliability)
 * 
 * @see AP_AHRS::has_status() for GPS_GLITCHING flag source
 * @see AP_NavEKF for innovation checking and glitch detection logic
 * @see EKF_CHECK_THRESH parameter for detection sensitivity
 * 
 * Source: Blimp/events.cpp:262-277
 */
// check for gps glitch failsafe
void Blimp::gpsglitch_check()
{
    // get filter status
    const bool gps_glitching = AP::ahrs().has_status(AP_AHRS::Status::GPS_GLITCHING);

    // log start or stop of gps glitch.  AP_Notify update is handled from within AP_AHRS
    if (ap.gps_glitching != gps_glitching) {
        ap.gps_glitching = gps_glitching;
        if (gps_glitching) {
            LOGGER_WRITE_ERROR(LogErrorSubsystem::GPS, LogErrorCode::GPS_GLITCH);
            gcs().send_text(MAV_SEVERITY_CRITICAL,"GPS Glitch");
        } else {
            LOGGER_WRITE_ERROR(LogErrorSubsystem::GPS, LogErrorCode::ERROR_RESOLVED);
            gcs().send_text(MAV_SEVERITY_CRITICAL,"GPS Glitch cleared");
        }
    }
}
