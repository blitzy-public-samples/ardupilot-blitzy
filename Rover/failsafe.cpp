/**
 * @file failsafe.cpp
 * @brief Implementation of rover failsafe system aggregating throttle/RC, GCS, EKF, battery, and crash failsafes
 * 
 * @details This file implements the comprehensive failsafe system for ArduRover, providing
 *          safety mechanisms to handle various failure conditions including:
 *          - RC signal loss (throttle failsafe) - handled via FAILSAFE_EVENT_THROTTLE
 *          - Ground Control Station telemetry loss (GCS failsafe) - handled via FAILSAFE_EVENT_GCS
 *          - Extended Kalman Filter failures (EKF failsafe) - handled via FAILSAFE_EVENT_EKF
 *          - Battery voltage/capacity critical levels - handled via handle_battery_failsafe()
 *          - Vehicle crash detection - handled via FAILSAFE_EVENT_CRASH
 *          - Main loop lockup detection - monitored by failsafe_check() watchdog
 *          
 *          Failsafe behavior is configurable via parameters:
 *          - FS_ACTION: Default failsafe action (None, Hold, RTL, SmartRTL, Terminate, etc.)
 *          - FS_THR_ENABLE: Throttle failsafe behavior (disabled, hold, RTL, continue mission)
 *          - FS_GCS_ENABLE: GCS failsafe behavior (disabled, hold, RTL, continue mission)
 *          - FS_TIMEOUT: Delay before triggering failsafe action (seconds)
 *          - FS_OPTIONS: Bitmask of additional failsafe options
 *          
 *          The failsafe system uses a bitmask approach (failsafe.bits) to track multiple
 *          simultaneous failsafe conditions, and only clears when all conditions are resolved.
 * 
 * @author Andrew Tridgell, December 2011
 * @warning This is safety-critical code - modifications can affect vehicle safety
 * 
 * Source: Rover/failsafe.cpp
 */

#include "Rover.h"

#include <stdio.h>

/**
 * @brief Main loop lockup detection and emergency disarm watchdog
 * 
 * @details This function implements a critical safety watchdog that monitors the main loop
 *          for lockups. It is called from the core timer interrupt at 1kHz (every 1ms) to
 *          detect if the main scheduler loop has stopped running.
 *          
 *          Detection Algorithm:
 *          - Tracks scheduler tick count changes to verify main loop is running
 *          - If tick count hasn't changed for >0.2 seconds (200,000 microseconds), assumes lockup
 *          - Upon lockup detection, immediately disarms motors to prevent uncontrolled operation
 *          
 *          Lockup Scenarios Detected:
 *          - Main loop infinite loop or deadlock
 *          - Excessive processing time blocking scheduler
 *          - Critical software failures preventing loop execution
 *          
 *          False Positive Scenarios (intended):
 *          - During initialization routines (before scheduler starts)
 *          - During log erase operations (blocking operation)
 *          - In these cases, disarming is safe as vehicle should not be armed yet
 * 
 * @note Called from timer interrupt at 1kHz - keep execution time minimal
 * @note Uses static variables to maintain state between calls
 * 
 * @warning This is a last-resort safety mechanism - lockup indicates serious software failure
 * @warning 0.2 second threshold is critical - shorter may cause false positives, longer reduces safety
 * 
 * Source: Rover/failsafe.cpp:18-42
 */
void Rover::failsafe_check()
{
    // Static variables maintain state between 1kHz interrupt calls
    static uint16_t last_ticks;      // Last observed scheduler tick count
    static uint32_t last_timestamp;  // Timestamp (microseconds) of last tick change
    const uint32_t tnow = AP_HAL::micros();

    // Get current scheduler tick count (increments each main loop iteration)
    const uint16_t ticks = scheduler.ticks();
    if (ticks != last_ticks) {
        // Tick count changed - main loop is running normally, all is OK
        last_ticks = ticks;
        last_timestamp = tnow;
        return;
    }

    // Tick count hasn't changed - check if lockup duration exceeds threshold
    if (tnow - last_timestamp > 200000) {
        // Main loop hasn't run for >0.2 seconds (200,000 microseconds) - emergency condition
        // Possible causes: infinite loop, deadlock, critical failure, or initialization/log erase
        // @warning Safety-critical action: Immediately disarm to prevent uncontrolled vehicle operation
        // To-Do: log error
        if (arming.is_armed()) {
            // Emergency disarm with CPUFAILSAFE reason for logging/telemetry
            arming.disarm(AP_Arming::Method::CPUFAILSAFE);
        }
    }
}

/**
 * @brief Set or clear a failsafe event and trigger appropriate failsafe actions
 * 
 * @details This is the central failsafe management function that handles all failsafe events
 *          including throttle (RC) loss, GCS telemetry loss, EKF failure, and crash detection.
 *          
 *          Failsafe State Machine:
 *          1. Event Registration: Sets/clears bits in failsafe.bits bitmask
 *          2. Timeout Monitoring: Waits for FS_TIMEOUT seconds before triggering actions
 *          3. Action Execution: Executes configured failsafe action (Hold, RTL, SmartRTL, Terminate)
 *          4. Recovery: Clears triggered state when all failsafe conditions resolve
 *          
 *          Throttle Failsafe (FAILSAFE_EVENT_THROTTLE - RC signal loss):
 *          - Behavior controlled by FS_THR_ENABLE parameter:
 *            * FS_THR_DISABLED (0): No action taken
 *            * FS_THR_ENABLED_ALWAYS_RTL (1): Always execute FS_ACTION
 *            * FS_THR_ENABLED_CONTINUE_MISSION (2): Continue mission in Auto mode, otherwise FS_ACTION
 *          - Triggered when RC receiver loses signal or RC inputs invalid
 *          
 *          GCS Failsafe (FAILSAFE_EVENT_GCS - telemetry loss):
 *          - Behavior controlled by FS_GCS_ENABLE parameter:
 *            * FS_GCS_DISABLED (0): No action taken
 *            * FS_GCS_ENABLED_ALWAYS_RTL (1): Always execute FS_ACTION
 *            * FS_GCS_ENABLED_CONTINUE_MISSION (2): Continue mission in Auto mode, otherwise FS_ACTION
 *          - Triggered when heartbeat from ground control station times out
 *          
 *          Failsafe Actions (FS_ACTION parameter):
 *          - None: No action (only clear RC overrides and send telemetry warning)
 *          - Hold: Immediately stop vehicle
 *          - RTL: Return to launch position
 *          - SmartRTL: Return along recorded path, fallback to RTL if unavailable
 *          - SmartRTL_Hold: SmartRTL with Hold fallback
 *          - Loiter_Hold: Loiter (for boats) with Hold fallback
 *          - Terminate: Emergency disarm (use with caution)
 *          
 *          Special Conditions:
 *          - Already in RTL mode: No action taken (avoid mode switching loop)
 *          - In Hold mode: Action taken only if FS_OPTIONS Failsafe_Option_Active_In_Hold is set
 *          - RC overrides always cleared on failsafe trigger for safety
 * 
 * @param[in] failsafe_type Bitmask identifying failsafe type (FAILSAFE_EVENT_THROTTLE, 
 *                          FAILSAFE_EVENT_GCS, FAILSAFE_EVENT_EKF, FAILSAFE_EVENT_CRASH)
 * @param[in] type_str      Human-readable failsafe type string for telemetry messages (e.g., "Radio", "GCS")
 * @param[in] on            true to trigger failsafe condition, false to clear it
 * 
 * @note FS_TIMEOUT parameter adds delay before action to avoid transient failsafes
 * @note Multiple simultaneous failsafe conditions are tracked via bitmask
 * @note Failsafe only clears when ALL conditions (failsafe.bits == 0) are resolved
 * 
 * @warning Safety-critical function - controls vehicle behavior during emergency conditions
 * @warning Continue mission mode bypasses failsafe action - use with caution
 * @warning Terminate action immediately disarms - vehicle will be uncontrolled
 * 
 * Source: Rover/failsafe.cpp:77-115
 */
void Rover::failsafe_trigger(uint8_t failsafe_type, const char* type_str, bool on)
{
    // Step 1: Update failsafe bitmask to track active failsafe conditions
    uint8_t old_bits = failsafe.bits;
    if (on) {
        // Set bit for this failsafe type (multiple failsafes can be active simultaneously)
        failsafe.bits |= failsafe_type;
    } else {
        // Clear bit for this failsafe type (condition resolved)
        failsafe.bits &= ~failsafe_type;
    }
    
    // Step 2: Record start time when transitioning from no failsafes to first failsafe
    if (old_bits == 0 && failsafe.bits != 0) {
        // First failsafe event has started - record time for FS_TIMEOUT delay
        failsafe.start_time = millis();
    }
    
    // Step 3: Notify GCS when all failsafe conditions have cleared
    if (failsafe.triggered != 0 && failsafe.bits == 0) {
        // All failsafe conditions resolved - send "Cleared" message to GCS
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s Failsafe Cleared", type_str);
    }

    // Step 4: Clear triggered state for any failsafe conditions that have resolved
    // This ensures failsafe.triggered only contains bits for currently active failsafes
    failsafe.triggered &= failsafe.bits;

    // Step 5: Check if conditions met to trigger failsafe action
    // All conditions must be true:
    // - No failsafe action already triggered (failsafe.triggered == 0)
    // - At least one failsafe condition active (failsafe.bits != 0)
    // - Timeout period has elapsed (millis() - start_time > FS_TIMEOUT seconds)
    // - Not already in RTL mode (avoid mode switching loop)
    // - Not in Hold mode UNLESS FS_OPTIONS allows failsafe while in Hold
    if ((failsafe.triggered == 0) &&
        (failsafe.bits != 0) &&
        (millis() - failsafe.start_time > g.fs_timeout * 1000) &&
        (control_mode != &mode_rtl) &&
        ((control_mode != &mode_hold || (g2.fs_options & (uint32_t)Failsafe_Options::Failsafe_Option_Active_In_Hold)))) {
        
        // Mark this failsafe as triggered to prevent repeated action
        failsafe.triggered = failsafe.bits;
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "%s Failsafe", type_str);

        // @warning Safety-critical: Always clear RC overrides on failsafe for safety
        // Prevents manual control commands from interfering with failsafe recovery
        RC_Channels::clear_overrides();

        // Step 6: Check for "continue mission" behavior (throttle or GCS failsafe specific)
        // If in Auto mode and configured to continue mission, bypass failsafe action
        if ((control_mode == &mode_auto) &&
            ((failsafe_type == FAILSAFE_EVENT_THROTTLE && g.fs_throttle_enabled == FS_THR_ENABLED_CONTINUE_MISSION) ||
             (failsafe_type == FAILSAFE_EVENT_GCS && g.fs_gcs_enabled == FS_GCS_ENABLED_CONTINUE_MISSION))) {
            // @warning Continue mission mode - vehicle proceeds with autonomous mission despite failsafe
            // This assumes mission is safe to complete without RC or GCS input
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Failsafe - Continuing Auto Mode");
        } else {
            // Step 7: Execute configured failsafe action based on FS_ACTION parameter
            // Uses fallthrough pattern for graceful degradation (e.g., SmartRTL → RTL → Hold)
            switch ((FailsafeAction)g.fs_action.get()) {
            case FailsafeAction::None:
                // No mode change - only RC overrides cleared and warning sent
                break;
                
            case FailsafeAction::SmartRTL:
                // Attempt SmartRTL (return along recorded path)
                if (set_mode(mode_smartrtl, ModeReason::FAILSAFE)) {
                    break;  // SmartRTL successful
                }
                // SmartRTL unavailable (no path recorded) - fall through to RTL
                FALLTHROUGH;
                
            case FailsafeAction::RTL:
                // Attempt RTL (return to launch via direct path)
                if (set_mode(mode_rtl, ModeReason::FAILSAFE)) {
                    break;  // RTL successful
                }
                // RTL unavailable (no home position) - fall through to Hold
                FALLTHROUGH;
                
            case FailsafeAction::Hold:
                // Hold mode - immediately stop vehicle (always available)
                set_mode(mode_hold, ModeReason::FAILSAFE);
                break;
                
            case FailsafeAction::SmartRTL_Hold:
                // Attempt SmartRTL with Hold as fallback (no RTL fallback)
                if (!set_mode(mode_smartrtl, ModeReason::FAILSAFE)) {
                    set_mode(mode_hold, ModeReason::FAILSAFE);
                }
                break;
                
            case FailsafeAction::Loiter_Hold:
                // Attempt Loiter (for boats - maintain position) with Hold fallback
                if (!set_mode(mode_loiter, ModeReason::FAILSAFE)) {
                    set_mode(mode_hold, ModeReason::FAILSAFE);
                }
                break;
                
            case FailsafeAction::Terminate:
                // @warning Emergency disarm - vehicle will become uncontrolled immediately
                // Use only when continuing operation is more dangerous than stopping
                arming.disarm(AP_Arming::Method::FAILSAFE_ACTION_TERMINATE);
                break;
            }
        }
    }
}

/**
 * @brief Execute battery failsafe action when battery voltage or capacity reaches critical level
 * 
 * @details This function handles battery-specific failsafe actions triggered when battery
 *          monitoring detects critical conditions (low voltage, low capacity, or high cell voltage).
 *          Unlike the general failsafe_trigger() function, this directly executes the specified
 *          action without timeout delays, as battery failsafes are time-critical safety events.
 *          
 *          Battery Failsafe Triggers:
 *          - Low voltage failsafe: Triggered by BATT_LOW_VOLT parameter threshold
 *          - Critical voltage failsafe: Triggered by BATT_CRT_VOLT parameter threshold
 *          - Low capacity failsafe: Triggered by BATT_LOW_MAH parameter threshold
 *          - Critical capacity failsafe: Triggered by BATT_CRT_MAH parameter threshold
 *          - High voltage failsafe: Triggered by BATT_HIGH_VOLT parameter (for over-voltage protection)
 *          
 *          Failsafe Actions:
 *          - Uses same FailsafeAction enum as general failsafes
 *          - Graceful degradation: SmartRTL → RTL → Hold fallback pattern
 *          - Terminate action: Calls advanced failsafe system (AFS) if enabled, otherwise direct disarm
 *          
 *          Differences from General Failsafe:
 *          - No timeout delay - battery failsafes execute immediately
 *          - No "continue mission" option - battery state always requires action
 *          - Different disarm method for logging/telemetry distinction
 * 
 * @param[in] type_str Battery type identifier for telemetry (e.g., "low", "critical", "high")
 * @param[in] action   Failsafe action to execute (FailsafeAction enum value)
 * 
 * @note Called from battery monitoring code when threshold exceeded
 * @note Immediate execution without FS_TIMEOUT delay
 * @note Uses BATTERY_FAILSAFE reason code for mode changes and disarm
 * 
 * @warning Battery failsafes are time-critical - vehicle may lose power imminently
 * @warning Terminate action uses advanced failsafe system if available for coordinated shutdown
 * 
 * Source: Rover/failsafe.cpp:241-286
 */
void Rover::handle_battery_failsafe(const char* type_str, const int8_t action)
{
        // Execute battery-specific failsafe action immediately (no timeout delay)
        // Uses graceful degradation pattern with fallbacks
        switch ((FailsafeAction)action) {
            case FailsafeAction::None:
                // No action - only telemetry warning sent by caller
                break;
                
            case FailsafeAction::SmartRTL:
                // Attempt SmartRTL (return along recorded path to conserve battery)
                if (set_mode(mode_smartrtl, ModeReason::BATTERY_FAILSAFE)) {
                    break;  // SmartRTL successful
                }
                // SmartRTL unavailable - fall through to RTL
                FALLTHROUGH;
                
            case FailsafeAction::RTL:
                // Attempt RTL (direct return to launch)
                if (set_mode(mode_rtl, ModeReason::BATTERY_FAILSAFE)) {
                    break;  // RTL successful
                }
                // RTL unavailable (no home position) - fall through to Hold
                FALLTHROUGH;
                
            case FailsafeAction::Hold:
                // Hold mode - stop vehicle to conserve remaining battery
                set_mode(mode_hold, ModeReason::BATTERY_FAILSAFE);
                break;
                
            case FailsafeAction::Loiter_Hold:
                // Attempt Loiter (for boats) with Hold fallback
                if (!set_mode(mode_loiter, ModeReason::BATTERY_FAILSAFE)) {
                    set_mode(mode_hold, ModeReason::BATTERY_FAILSAFE);
                }
                break;
                
            case FailsafeAction::SmartRTL_Hold:
                // Attempt SmartRTL with Hold fallback (no RTL fallback)
                if (!set_mode(mode_smartrtl, ModeReason::BATTERY_FAILSAFE)) {
                    set_mode(mode_hold, ModeReason::BATTERY_FAILSAFE);
                }
                break;
                
            case FailsafeAction::Terminate:
                // @warning Emergency termination due to critical battery condition
                // Vehicle will be immediately disarmed and become uncontrolled
#if AP_ROVER_ADVANCED_FAILSAFE_ENABLED
                // Use Advanced Failsafe System (AFS) for coordinated termination
                // AFS notifies GCS and logs termination reason before disarm
                char battery_type_str[17];
                snprintf(battery_type_str, 17, "%s battery", type_str);
                g2.afs.gcs_terminate(true, battery_type_str);
#else
                // Direct disarm with BATTERYFAILSAFE reason for logging
                arming.disarm(AP_Arming::Method::BATTERYFAILSAFE);
#endif // AP_ROVER_ADVANCED_FAILSAFE_ENABLED
                break;
        }
}

#if AP_ROVER_ADVANCED_FAILSAFE_ENABLED
/**
 * @brief Perform Advanced Failsafe System (AFS) periodic checks
 * 
 * @details This function integrates with the Advanced Failsafe System (AFS) to perform
 *          additional safety monitoring beyond standard failsafes. AFS provides enhanced
 *          safety features including:
 *          - Geofence breach monitoring with coordinated termination
 *          - RC signal quality assessment and degradation detection
 *          - Operator intervention requirements for critical situations
 *          - Coordinated termination with GCS notification and logging
 *          
 *          The AFS system maintains its own internal state machine and can trigger
 *          termination independently based on configured safety rules.
 *          
 *          Integration Points:
 *          - Provides last_valid_rc_ms timestamp for RC signal monitoring
 *          - AFS can trigger coordinated termination via g2.afs.gcs_terminate()
 *          - Works in conjunction with standard failsafe system
 * 
 * @note Only compiled if AP_ROVER_ADVANCED_FAILSAFE_ENABLED is defined
 * @note Called periodically from main loop to update AFS state
 * @note AFS maintains separate state from standard failsafe system
 * 
 * @warning AFS can trigger autonomous termination based on safety rules
 * @warning Ensure AFS parameters are properly configured before enabling
 * 
 * @see AP_AdvancedFailsafe library for AFS implementation details
 * 
 * Source: Rover/failsafe.cpp:326-332
 */
void Rover::afs_fs_check(void)
{
    // Perform Advanced Failsafe System (AFS) periodic checks
    // Passes last valid RC timestamp for signal quality monitoring
    g2.afs.check(failsafe.last_valid_rc_ms);
}
#endif
