/**
 * @file mode.cpp
 * @brief ArduCopter flight mode management and mode base class implementation
 * 
 * @details This file implements the flight mode framework for ArduCopter, including:
 * - Mode registration and factory pattern (mode_from_mode_num)
 * - Mode switching with comprehensive safety checks (set_mode)
 * - Mode lifecycle management (init, run, exit)
 * - Base class default implementations for common mode operations
 * - Main loop integration where copter.flightmode->run() is called each iteration
 * 
 * Flight Mode Architecture:
 * - Each flight mode inherits from the Mode base class
 * - Mode objects are statically allocated (mode_stabilize, mode_althold, etc.)
 * - The mode_from_mode_num() factory function maps mode numbers to mode objects
 * - The current mode is accessed via copter.flightmode pointer
 * - Mode changes go through set_mode() which validates safety constraints
 * 
 * Safety-Critical Validation During Mode Changes:
 * - GPS position requirements (requires_GPS() check with position_ok())
 * - Altitude estimate requirements (ekf_alt_ok() for non-manual modes)
 * - RC failsafe state (allows_entry_in_rc_failsafe())
 * - Arming state and rotor runup completion
 * - Throttle position validation for manual modes
 * - Fence breach recovery restrictions
 * 
 * Coordinate Frames: This file works with NED (North-East-Down) frame for position
 * and velocity commands, and body frame for pilot stick inputs.
 * 
 * @note High level mode switching logic. Individual flight mode implementations
 *       are in mode_*.cpp files (mode_stabilize.cpp, mode_loiter.cpp, etc.)
 * 
 * @warning Mode changes must pass safety validation or vehicles may exhibit
 *          unexpected behavior. Always check set_mode() return value.
 * 
 * Source: ArduCopter/mode.cpp
 */

#include "Copter.h"

/**
 * @brief Mode base class constructor
 * 
 * @details Initializes Mode object by capturing references to key Copter subsystems.
 * This allows mode implementations to access vehicle state and control systems without
 * needing to explicitly reference the global copter object in every method call.
 * 
 * All mode objects are constructed at startup before main() runs, during static
 * initialization. The copter object must be fully constructed before any Mode
 * objects are created.
 * 
 * @note Constructor captures references to subsystems that are guaranteed to exist
 *       for the lifetime of the vehicle (g, g2, motors, etc.)
 */
Mode::Mode(void) :
    g(copter.g),
    g2(copter.g2),
    wp_nav(copter.wp_nav),
    loiter_nav(copter.loiter_nav),
    pos_control(copter.pos_control),
    ahrs(copter.ahrs),
    attitude_control(copter.attitude_control),
    motors(copter.motors),
    channel_roll(copter.channel_roll),
    channel_pitch(copter.channel_pitch),
    channel_throttle(copter.channel_throttle),
    channel_yaw(copter.channel_yaw),
    G_Dt(copter.G_Dt)
{ };

#if AC_PAYLOAD_PLACE_ENABLED
PayloadPlace Mode::payload_place;
#endif

/**
 * @brief Mode factory function - returns pointer to mode object for given mode number
 * 
 * @details This function implements the factory pattern for flight modes, mapping
 * Mode::Number enum values to their corresponding statically-allocated mode objects.
 * Each mode object (mode_stabilize, mode_althold, etc.) is a global static object
 * constructed at startup.
 * 
 * Mode Registration:
 * - Core modes (STABILIZE, ALT_HOLD, LAND) are always available
 * - Optional modes are conditionally compiled based on feature flags:
 *   * MODE_ACRO_ENABLED, MODE_AUTO_ENABLED, MODE_LOITER_ENABLED, etc.
 * - Some modes (FLOWHOLD, SYSTEMID) use dynamic allocation (g2.mode_*_ptr)
 * - Custom scripted modes can be registered via mode_guided_custom array
 * 
 * Mode Availability:
 * - If a mode is disabled at compile time, its case statement is not included
 * - If a mode is not found, function returns nullptr
 * - Caller must check for nullptr and handle mode-not-available case
 * 
 * Custom Mode Support (requires MODE_GUIDED_ENABLED && AP_SCRIPTING_ENABLED):
 * - Lua scripts can register up to ARRAY_SIZE(mode_guided_custom) custom modes
 * - Custom modes are checked after built-in modes
 * - Custom modes inherit from ModeGuided and have unique mode numbers
 * 
 * @param[in] mode Mode number to look up (from Mode::Number enum)
 * 
 * @return Pointer to Mode object if mode exists and is compiled in, nullptr otherwise
 * 
 * @note This function is called frequently during mode changes and in the main loop,
 *       but the switch statement is fast (O(1) with compiler optimization)
 * 
 * @warning Returning nullptr indicates invalid mode number or disabled feature.
 *          Caller must handle this case to prevent null pointer dereference.
 * 
 * @see Copter::set_mode() for mode change logic that uses this factory function
 * @see Mode::Number enum in mode.h for complete list of mode numbers
 * 
 * Source: ArduCopter/mode.cpp:32-169
 */
Mode *Copter::mode_from_mode_num(const Mode::Number mode)
{
    // Mode registration via switch statement - each case returns a pointer to a
    // statically allocated mode object. Modes disabled at compile time are excluded.
    switch (mode) {
#if MODE_ACRO_ENABLED
        case Mode::Number::ACRO:
            return &mode_acro;
#endif

        case Mode::Number::STABILIZE:
            return &mode_stabilize;

        case Mode::Number::ALT_HOLD:
            return &mode_althold;

#if MODE_AUTO_ENABLED
        case Mode::Number::AUTO:
            return &mode_auto;
#endif

#if MODE_CIRCLE_ENABLED
        case Mode::Number::CIRCLE:
            return &mode_circle;
#endif

#if MODE_LOITER_ENABLED
        case Mode::Number::LOITER:
            return &mode_loiter;
#endif

#if MODE_GUIDED_ENABLED
        case Mode::Number::GUIDED:
            return &mode_guided;
#endif

        case Mode::Number::LAND:
            return &mode_land;

#if MODE_RTL_ENABLED
        case Mode::Number::RTL:
            return &mode_rtl;
#endif

#if MODE_DRIFT_ENABLED
        case Mode::Number::DRIFT:
            return &mode_drift;
#endif

#if MODE_SPORT_ENABLED
        case Mode::Number::SPORT:
            return &mode_sport;
#endif

#if MODE_FLIP_ENABLED
        case Mode::Number::FLIP:
            return &mode_flip;
#endif

#if AUTOTUNE_ENABLED
        case Mode::Number::AUTOTUNE:
            return &mode_autotune;
#endif

#if MODE_POSHOLD_ENABLED
        case Mode::Number::POSHOLD:
            return &mode_poshold;
#endif

#if MODE_BRAKE_ENABLED
        case Mode::Number::BRAKE:
            return &mode_brake;
#endif

#if MODE_THROW_ENABLED
        case Mode::Number::THROW:
            return &mode_throw;
#endif

#if AP_ADSB_AVOIDANCE_ENABLED
        case Mode::Number::AVOID_ADSB:
            return &mode_avoid_adsb;
#endif

#if MODE_GUIDED_NOGPS_ENABLED
        case Mode::Number::GUIDED_NOGPS:
            return &mode_guided_nogps;
#endif

#if MODE_SMARTRTL_ENABLED
        case Mode::Number::SMART_RTL:
            return &mode_smartrtl;
#endif

#if MODE_FLOWHOLD_ENABLED
        case Mode::Number::FLOWHOLD:
            return (Mode *)g2.mode_flowhold_ptr;
#endif

#if MODE_FOLLOW_ENABLED
        case Mode::Number::FOLLOW:
            return &mode_follow;
#endif

#if MODE_ZIGZAG_ENABLED
        case Mode::Number::ZIGZAG:
            return &mode_zigzag;
#endif

#if MODE_SYSTEMID_ENABLED
        case Mode::Number::SYSTEMID:
            return (Mode *)g2.mode_systemid_ptr;
#endif

#if MODE_AUTOROTATE_ENABLED
        case Mode::Number::AUTOROTATE:
            return &mode_autorotate;
#endif

#if MODE_TURTLE_ENABLED
        case Mode::Number::TURTLE:
            return &mode_turtle;
#endif

        default:
            // Mode number not recognized or mode disabled at compile time
            break;
    }

#if MODE_GUIDED_ENABLED && AP_SCRIPTING_ENABLED
    // Custom mode support: Lua scripts can register custom flight modes that inherit
    // from ModeGuided. Check the mode_guided_custom array for script-registered modes.
    // This allows extending flight mode capabilities without modifying core code.
    for (uint8_t i = 0; i < ARRAY_SIZE(mode_guided_custom); i++) {
        if ((mode_guided_custom[i] != nullptr) && (mode_guided_custom[i]->mode_number() == mode)) {
            return mode_guided_custom[i];
        }
    }
#endif

    // Mode not found - either invalid mode number or feature not compiled in
    return nullptr;
}


/**
 * @brief Handle failed mode change attempts with appropriate notifications
 * 
 * @details Called when a mode change is rejected by safety checks or mode initialization.
 * Provides user feedback through multiple channels:
 * - GCS telemetry message with failure reason
 * - Dataflash log error entry for post-flight analysis
 * - Audio/visual notification (buzzer, LEDs) if vehicle initialized
 * 
 * Common Failure Reasons:
 * - "requires position" - GPS mode without valid position estimate
 * - "need alt estimate" - Altitude-holding mode without valid altitude
 * - "runup not complete" - Helicopter rotor not at flight speed
 * - "throttle too high" - Manual throttle mode with stick above safe threshold
 * - "in RC failsafe" - Mode doesn't allow entry during RC signal loss
 * - "in fence recovery" - Mode change disabled during fence breach recovery
 * - "init failed" - Mode-specific initialization rejected the change
 * - "GCS entry disabled (FLTMODE_GCSBLOCK)" - Mode blocked by parameter
 * 
 * @param[in] mode Pointer to the mode that failed to activate
 * @param[in] reason Human-readable string explaining why mode change failed
 * 
 * @note This function does not change vehicle state - it only provides notifications.
 *       The vehicle remains in its current flight mode after this call.
 * 
 * @warning Frequent mode change failures may indicate sensor issues, improper tuning,
 *          or incorrect parameter configuration that should be investigated.
 * 
 * Source: ArduCopter/mode.cpp:173-181
 */
void Copter::mode_change_failed(const Mode *mode, const char *reason)
{
    // Send warning to ground control station with mode name and failure reason
    gcs().send_text(MAV_SEVERITY_WARNING, "Mode change to %s failed: %s", mode->name(), reason);
    
    // Log error to dataflash for post-flight analysis
    LOGGER_WRITE_ERROR(LogErrorSubsystem::FLIGHT_MODE, LogErrorCode(mode->mode_number()));
    
    // Trigger notification system (buzzer, LEDs) to alert pilot
    // Only if vehicle has completed initialization to avoid spurious notifications at startup
    if (copter.ap.initialised) {
        AP_Notify::events.user_mode_change_failed = 1;
    }
}

/**
 * @brief Check if a mode can be entered via GCS command based on FLTMODE_GCSBLOCK parameter
 * 
 * @details Validates whether a mode change from ground control station is permitted
 * based on the FLTMODE_GCSBLOCK parameter bitmask. This allows operators to restrict
 * certain modes from GCS control while still allowing RC transmitter mode changes.
 * 
 * Use Cases:
 * - Prevent accidental mode changes from GCS in safety-critical operations
 * - Restrict autonomous modes when RC pilot must maintain direct control
 * - Competition or regulatory requirements for pilot-only mode authority
 * - Training scenarios where instructor limits available modes
 * 
 * Mode Blocking Mechanism:
 * - FLTMODE_GCSBLOCK parameter is a bitmask of modes to block from GCS
 * - Each bit position corresponds to a mode in the mode_list array below
 * - If mode is blocked, returns false and calls mode_change_failed()
 * - RC transmitter mode changes bypass this check (not affected by parameter)
 * 
 * @param[in] mode_num Mode number to validate for GCS access
 * 
 * @return true if mode can be entered from GCS, false if blocked by parameter
 * 
 * @note RTL and LAND are intentionally excludable to ensure pilot can always
 *       force landing via RC if GCS becomes unreliable
 * 
 * @warning This check only applies to ModeReason::GCS_COMMAND. Other mode change
 *          reasons (failsafe, RC switch, etc.) bypass this restriction.
 * 
 * @see Copter::set_mode() which calls this for GCS-commanded mode changes
 * @see block_GCS_mode_change() in AP_Vehicle for bitmask checking logic
 * 
 * Source: ArduCopter/mode.cpp:184-227
 */
bool Copter::gcs_mode_enabled(const Mode::Number mode_num)
{
    // List of modes that can be blocked from GCS entry via FLTMODE_GCSBLOCK parameter.
    // Array index corresponds to bit number in the parameter bitmask.
    // Not all modes are in this list - some modes (emergency modes) cannot be blocked.
    static const uint8_t mode_list [] {
        (uint8_t)Mode::Number::STABILIZE,
        (uint8_t)Mode::Number::ACRO,
        (uint8_t)Mode::Number::ALT_HOLD,
        (uint8_t)Mode::Number::AUTO,
        (uint8_t)Mode::Number::GUIDED,
        (uint8_t)Mode::Number::LOITER,
        (uint8_t)Mode::Number::CIRCLE,
        (uint8_t)Mode::Number::DRIFT,
        (uint8_t)Mode::Number::SPORT,
        (uint8_t)Mode::Number::FLIP,
        (uint8_t)Mode::Number::AUTOTUNE,
        (uint8_t)Mode::Number::POSHOLD,
        (uint8_t)Mode::Number::BRAKE,
        (uint8_t)Mode::Number::THROW,
        (uint8_t)Mode::Number::AVOID_ADSB,
        (uint8_t)Mode::Number::GUIDED_NOGPS,
        (uint8_t)Mode::Number::SMART_RTL,
        (uint8_t)Mode::Number::FLOWHOLD,
        (uint8_t)Mode::Number::FOLLOW,
        (uint8_t)Mode::Number::ZIGZAG,
        (uint8_t)Mode::Number::SYSTEMID,
        (uint8_t)Mode::Number::AUTOROTATE,
        (uint8_t)Mode::Number::AUTO_RTL,
        (uint8_t)Mode::Number::TURTLE
    };

    // Check if mode is blocked by FLTMODE_GCSBLOCK parameter bitmask
    if (!block_GCS_mode_change((uint8_t)mode_num, mode_list, ARRAY_SIZE(mode_list))) {
        // Mode is not blocked, allow GCS to command this mode change
        return true;
    }

    // Mode is blocked by parameter - provide feedback to user
    // Try to get mode name for better error message
    Mode *new_flightmode = mode_from_mode_num(mode_num);
    if (new_flightmode != nullptr) {
        // Mode exists, report it's blocked
        mode_change_failed(new_flightmode, "GCS entry disabled (FLTMODE_GCSBLOCK)");
    } else {
        // Mode doesn't exist or isn't compiled in
        notify_no_such_mode((uint8_t)mode_num);
    }

    return false;
}

/**
 * @brief Change flight mode with comprehensive safety validation
 * 
 * @details This is the primary mode switching function for ArduCopter, implementing
 * a multi-stage validation process before allowing mode changes. All mode changes
 * go through this function, whether commanded by pilot RC switch, GCS, failsafe,
 * or autonomous mission logic.
 * 
 * Mode Change Validation Sequence:
 * 1. Check if already in requested mode (early return if true)
 * 2. Validate GCS mode blocking (FLTMODE_GCSBLOCK parameter)
 * 3. Handle special case AUTO_RTL (synthetic mode)
 * 4. Look up mode object via mode_from_mode_num() factory
 * 5. Run safety checks (see Safety Checks section below)
 * 6. Call mode->init() to allow mode-specific initialization
 * 7. Exit previous mode (cleanup and smooth transitions)
 * 8. Update copter.flightmode pointer to new mode
 * 9. Log mode change and notify subsystems
 * 
 * Safety Checks (can be bypassed if disarmed via ignore_checks):
 * - Helicopter rotor runup completion (HELI_FRAME only)
 * - Throttle position validation for manual modes (prevents sudden jumps)
 * - GPS position requirement (requires_GPS() modes need position_ok())
 * - Altitude estimate requirement (non-manual modes need ekf_alt_ok())
 * - Fence breach recovery restrictions (may prevent mode changes)
 * - RC failsafe state (mode must allow entry during failsafe)
 * 
 * EKF Status Requirements for GPS Modes:
 * - Modes with requires_GPS()==true check position_ok() which validates:
 *   * EKF status flags indicate position estimate is healthy
 *   * GPS has 3D fix with sufficient accuracy
 *   * Position estimate hasn't timed out
 * - Modes requiring altitude check ekf_alt_ok() which validates:
 *   * Barometer data is healthy and recent
 *   * EKF altitude variance is within acceptable limits
 * 
 * Mode Change Reasons (ModeReason enum):
 * - INITIALISED: Startup default mode
 * - RC_COMMAND: Pilot mode switch
 * - GCS_COMMAND: Ground station command
 * - RADIO_FAILSAFE: RC signal lost
 * - BATTERY_FAILSAFE: Low battery
 * - GCS_FAILSAFE: Telemetry lost
 * - EKF_FAILSAFE: Navigation failure
 * - GPS_GLITCH: GPS accuracy degraded
 * - MISSION_END: AUTO mission completed
 * - THROTTLE_LAND_ESCAPE: Pilot throttle during landing
 * - FENCE_BREACHED: Geofence violation
 * - TERRAIN_FAILSAFE: Terrain data unavailable
 * - BRAKE_TIMEOUT: Brake mode timer expired
 * - FLIP_COMPLETE: Flip maneuver finished
 * - AVOIDANCE: Collision avoidance triggered
 * - AVOIDANCE_RECOVERY: Obstacle avoidance recovered
 * - THROW_COMPLETE: Throw mode sequence finished
 * - AUTO_LAND_STARTED: AUTO initiated land sequence
 * 
 * Smooth Transition Handling:
 * - Manual to automatic throttle: Integrator pre-loading prevents jumps
 * - Attitude rate time constants updated per mode requirements
 * - Previous mode exit() called for cleanup (cancel takeoffs, reset states)
 * - Fence manual recovery allowed on pilot mode changes
 * 
 * @param[in] mode Flight mode number to switch to (Mode::Number enum)
 * @param[in] reason Why mode change is being requested (logged for analysis)
 * 
 * @return true if mode change successful, false if safety checks failed or mode unavailable
 * 
 * @note Manual throttle modes (STABILIZE, ACRO, ALT_HOLD) can almost always be
 *       entered successfully. GPS and autonomous modes have stricter requirements.
 * 
 * @warning Always check return value. Failed mode change leaves vehicle in current
 *          mode, which may not be what caller expects. Handle failures appropriately.
 * 
 * @warning This function is called at high frequency during failsafes. Ensure any
 *          added validation is computationally efficient.
 * 
 * @see Mode::init() for mode-specific initialization that can reject mode change
 * @see Copter::exit_mode() for cleanup performed when leaving a mode
 * @see Copter::mode_from_mode_num() for mode object factory
 * @see Copter::position_ok() for GPS position validation
 * @see Copter::ekf_alt_ok() for altitude estimate validation
 * 
 * Source: ArduCopter/mode.cpp:233-394
 */
bool Copter::set_mode(Mode::Number mode, ModeReason reason)
{
    // Track mode change reason for analysis and logging
    const ModeReason last_reason = _last_reason;
    _last_reason = reason;

    // Early return if already in the requested mode - no validation needed
    // Still update reason and notify if reason changed (e.g., mode held through failsafe)
    if (mode == flightmode->mode_number()) {
        control_mode_reason = reason;
        
        // Special case: Set yaw rate time constant during autopilot initialization in STABILIZE
        // This ensures smooth yaw response from the first moment of flight
        if (reason == ModeReason::INITIALISED && mode == Mode::Number::STABILIZE) {
            attitude_control->set_yaw_rate_tc(g2.command_model_pilot_y.get_rate_tc());
        }
        
        // Notify user of successful mode "change" if reason changed
        // (e.g., pilot selected mode that was already active)
        if (copter.ap.initialised && (reason != last_reason)) {
            AP_Notify::events.user_mode_change = 1;
        }
        return true;
    }

    // Validate GCS mode blocking: Check FLTMODE_GCSBLOCK parameter bitmask
    // This allows restricting certain modes from GCS control while allowing RC control
    if ((reason == ModeReason::GCS_COMMAND) && !gcs_mode_enabled(mode)) {
        return false;
    }

#if MODE_AUTO_ENABLED
    // AUTO_RTL is a synthetic mode number that triggers special behavior in AUTO mode
    // It's not a separate mode object - it tells AUTO mode to return via SmartRTL path
    // or jump to landing sequence if SmartRTL path not available
    if (mode == Mode::Number::AUTO_RTL) {
        return mode_auto.return_path_or_jump_to_landing_sequence_auto_RTL(reason);
    }
#endif

    // Look up the mode object using factory function
    // Returns nullptr if mode doesn't exist or isn't compiled in
    Mode *new_flightmode = mode_from_mode_num(mode);
    if (new_flightmode == nullptr) {
        notify_no_such_mode((uint8_t)mode);
        return false;
    }

    // Safety check bypass: When disarmed, allow any mode change
    // Arming checks will validate mode is appropriate before allowing takeoff
    // This allows ground testing and prevents pilots from getting "stuck" in a mode
    bool ignore_checks = !motors->armed();

#if FRAME_CONFIG == HELI_FRAME
    // Helicopter-specific safety check: Rotor must be at flight speed before entering
    // altitude-holding or autonomous modes. Prevents loss of control if rotor stalls.
    // Manual throttle modes (STABILIZE, ACRO) can be entered anytime as pilot controls collective.
    if (!ignore_checks && !new_flightmode->has_manual_throttle() && !motors->rotor_runup_complete()) {
        mode_change_failed(new_flightmode, "runup not complete");
        return false;
    }
#endif

#if FRAME_CONFIG != HELI_FRAME
    // Multicopter throttle safety check: Prevent vehicle from leaping off the ground
    // when switching from automatic to manual throttle mode with throttle stick raised.
    // 
    // Scenario this prevents:
    // 1. Vehicle armed in GUIDED mode (automatic throttle)
    // 2. Pilot raises throttle stick to 60% (doesn't trigger auto takeoff)
    // 3. Pilot switches to STABILIZE (manual throttle)
    // 4. Without this check, vehicle would immediately jump to 60% throttle
    // 
    // This check blocks the mode change, forcing pilot to lower throttle first.
    bool user_throttle = new_flightmode->has_manual_throttle();
#if MODE_DRIFT_ENABLED
    // DRIFT mode uses automatic throttle but allows some manual input
    if (new_flightmode == &mode_drift) {
        user_throttle = true;
    }
#endif
    // Only check when:
    // - Vehicle is landed (ap.land_complete)
    // - Switching TO a manual throttle mode
    // - FROM a non-manual throttle mode
    // - Throttle stick is above safe threshold
    if (!ignore_checks &&
        ap.land_complete &&
        user_throttle &&
        !copter.flightmode->has_manual_throttle() &&
        new_flightmode->get_pilot_desired_throttle() > copter.get_non_takeoff_throttle()) {
        mode_change_failed(new_flightmode, "throttle too high");
        return false;
    }
#endif

    // GPS Position Requirement Check:
    // Modes that navigate horizontally (LOITER, AUTO, GUIDED, etc.) require valid position estimate
    // from EKF. The requires_GPS() method returns true for these modes.
    // 
    // position_ok() validates:
    // - EKF status flags indicate position estimate is healthy (EKF_POS_HORIZ_ABS or EKF_POS_HORIZ_REL)
    // - GPS has 3D fix with acceptable horizontal accuracy
    // - Position estimate hasn't timed out (updated recently)
    // - If using optical flow, flow data is valid and recent
    // 
    // Without valid position, GPS-dependent modes cannot safely control horizontal position
    // and may drift unpredictably or attempt to navigate using stale data.
    if (!ignore_checks &&
        new_flightmode->requires_GPS() &&
        !copter.position_ok()) {
        mode_change_failed(new_flightmode, "requires position");
        return false;
    }

    // Altitude Estimate Requirement Check:
    // When switching FROM manual throttle TO automatic throttle mode, verify altitude
    // estimate is valid. This prevents dangerous behavior if barometer or EKF altitude
    // estimate has failed.
    // 
    // ekf_alt_ok() validates:
    // - EKF has valid vertical position estimate (EKF_CONST_POS_MODE not set)
    // - Barometer data is recent and healthy
    // - Altitude variance is within acceptable bounds
    // 
    // Only enforced when switching from manual to automatic throttle because:
    // - Manual modes don't use altitude estimate, so existing bad estimate is not a problem
    // - Automatic modes actively control altitude using estimate
    // - If estimate is bad and we enter auto mode, vehicle may climb/descend unexpectedly
    // 
    // We don't block automatic->automatic transitions as those modes already depend on altitude.
    if (!ignore_checks &&
        !copter.ekf_alt_ok() &&
        flightmode->has_manual_throttle() &&
        !new_flightmode->has_manual_throttle()) {
        mode_change_failed(new_flightmode, "need alt estimate");
        return false;
    }

#if AP_FENCE_ENABLED
    // Fence Breach Recovery Check:
    // When fence is breached and DISABLE_MODE_CHANGE option is enabled, prevent mode changes
    // until vehicle returns within fence boundaries. This ensures fence breach triggers
    // (RTL, LAND, etc.) complete their recovery action without pilot interference.
    // 
    // This check requires ALL of the following conditions:
    // - Fence system is enabled
    // - DISABLE_MODE_CHANGE option is set (AC_Fence::OPTIONS bitmask)
    // - Fence is currently breached (active violation)
    // - Vehicle is armed and flying (not landed)
    // - Current mode was entered due to fence breach (not a subsequent manual change)
    // 
    // Purpose: Prevents pilot from accidentally switching to a mode that might
    // extend the fence violation or cause unsafe behavior during recovery.
    if (!ignore_checks &&
        fence.enabled() &&
        fence.option_enabled(AC_Fence::OPTIONS::DISABLE_MODE_CHANGE) &&
        fence.get_breaches() &&
        motors->armed() &&
        get_control_mode_reason() == ModeReason::FENCE_BREACHED &&
        !ap.land_complete) {
        mode_change_failed(new_flightmode, "in fence recovery");
        return false;
    }
#endif

    // RC Failsafe Entry Check:
    // Most modes cannot be entered when RC signal is lost (rc().in_rc_failsafe()==true).
    // Only modes that explicitly support failsafe entry (usually RTL, LAND, or BRAKE)
    // have allows_entry_in_rc_failsafe()==true.
    // 
    // Rationale: If RC is lost, vehicle should stay in its failsafe mode (usually RTL or LAND)
    // rather than allowing mode changes from other sources (GCS, companion computer) that
    // might interfere with safe recovery.
    // 
    // Exceptions: Modes like RTL and LAND return true from allows_entry_in_rc_failsafe()
    // so they can be triggered as failsafe actions or backup recovery modes.
    if (rc().in_rc_failsafe() && !new_flightmode->allows_entry_in_rc_failsafe()) {
        mode_change_failed(new_flightmode, "in RC failsafe");
        return false;
    }

    // Mode-Specific Initialization:
    // Call the new mode's init() method to perform mode-specific setup and validation.
    // The mode can reject the change by returning false from init().
    // 
    // Common reasons for init() to fail:
    // - AUTO mode: no mission loaded or mission contains invalid commands
    // - GUIDED mode: no valid target position set
    // - FOLLOW mode: no valid follow target
    // - Mode-specific parameters out of range
    // 
    // The ignore_checks flag is passed through - modes may bypass some internal checks when disarmed.
    if (!new_flightmode->init(ignore_checks)) {
        mode_change_failed(new_flightmode, "init failed");
        return false;
    }

    // Cleanup Previous Mode:
    // Perform any necessary cleanup as we exit the old mode. This includes:
    // - Canceling active takeoff sequences
    // - Calling mode-specific exit() cleanup
    // - Smooth throttle transition for manual->automatic changes
    // - Helicopter-specific state resets (flybar passthrough, etc.)
    exit_mode(flightmode, new_flightmode);

    // Mode Change Complete - Update Vehicle State:
    // At this point all safety checks have passed and both modes have accepted the change
    
    // Update the active mode pointer - this is what gets called in the main loop
    flightmode = new_flightmode;
    
    // Record why this mode change occurred (for logging and analysis)
    control_mode_reason = reason;
    
#if HAL_LOGGING_ENABLED
    // Log mode change to dataflash with timestamp, mode number, and reason
    // Critical for post-flight analysis of mode transitions and failsafe behavior
    logger.Write_Mode((uint8_t)flightmode->mode_number(), reason);
#endif
    
    // Notify ground control station immediately via MAVLink HEARTBEAT message
    // GCS uses custom_mode field in HEARTBEAT to display current flight mode
    gcs().send_message(MSG_HEARTBEAT);

#if HAL_ADSB_ENABLED
    // Notify ADSB system if we're in an autonomous navigation mode
    // ADSB uses this to determine if vehicle is under autonomous control
    adsb.set_is_auto_mode((mode == Mode::Number::AUTO) || (mode == Mode::Number::RTL) || (mode == Mode::Number::GUIDED));
#endif

#if AP_FENCE_ENABLED
    // Fence Manual Recovery:
    // Any mode change (even automatic failsafe changes) indicates pilot or system
    // is actively trying to recover from fence breach. Temporarily disable fence
    // enforcement to allow recovery maneuver to complete.
    // 
    // This prevents fence from repeatedly forcing RTL/LAND while pilot is trying
    // to manually recover. Fence will re-enable once vehicle returns within boundaries.
    if (fence.get_action() != AC_Fence::Action::REPORT_ONLY) {
        fence.manual_recovery_start();
    }
#endif

#if AP_CAMERA_ENABLED
    // Notify camera system if we're in AUTO mode for automatic camera triggering
    camera.set_is_auto_mode(flightmode->mode_number() == Mode::Number::AUTO);
#endif

    // Attitude Rate Controller Time Constant Updates:
    // Different modes use different rate controller time constants for pilot feel
    // - ACRO/SPORT: Aggressive response (shorter time constant)
    // - Other modes: Standard pilot response
    // - ACRO/DRIFT yaw: Extra aggressive yaw response for aerobatics
#if MODE_ACRO_ENABLED || MODE_SPORT_ENABLED
    attitude_control->set_roll_pitch_rate_tc(g2.command_model_acro_rp.get_rate_tc());
#endif
    attitude_control->set_yaw_rate_tc(g2.command_model_pilot_y.get_rate_tc());
#if MODE_ACRO_ENABLED || MODE_DRIFT_ENABLED
    if (mode== Mode::Number::ACRO || mode== Mode::Number::DRIFT) {
        attitude_control->set_yaw_rate_tc(g2.command_model_acro_y.get_rate_tc());
    }
#endif

    // Update notification subsystem (LEDs, buzzers) to reflect new mode
    notify_flight_mode();

    // Trigger "mode change success" notification (buzzer tone, LED pattern)
    // Only if vehicle has completed initialization (suppress during startup)
    if (copter.ap.initialised) {
        AP_Notify::events.user_mode_change = 1;
    }

    // Mode change successful
    return true;
}

/**
 * @brief Set flight mode using integer mode number (convenience overload)
 * 
 * @details Provides compatibility with external code that uses uint8_t mode numbers
 * instead of Mode::Number enum. This is commonly used by GCS interface code and
 * MAVLink message handlers which receive mode as integer from ground station.
 * 
 * Compile-time option DISALLOW_GCS_MODE_CHANGE_DURING_RC_FAILSAFE:
 * - If defined, blocks ALL GCS mode changes when RC failsafe is active
 * - More restrictive than default behavior (which allows some modes during failsafe)
 * - Useful for operations requiring RC pilot to maintain mode authority during failsafe
 * 
 * @param[in] new_mode Mode number as uint8_t (will be cast to Mode::Number)
 * @param[in] reason Why mode change is being requested
 * 
 * @return true if mode change successful, false otherwise
 * 
 * @note Static assert ensures Mode::Number and uint8_t are same size for safe casting
 * 
 * @see Copter::set_mode(Mode::Number, ModeReason) for main implementation
 * 
 * Source: ArduCopter/mode.cpp:396-406
 */
bool Copter::set_mode(const uint8_t new_mode, const ModeReason reason)
{
    static_assert(sizeof(Mode::Number) == sizeof(new_mode), "The new mode can't be mapped to the vehicles mode number");
    
#ifdef DISALLOW_GCS_MODE_CHANGE_DURING_RC_FAILSAFE
    // Optional compile-time restriction: Block ALL GCS mode changes during RC failsafe
    // This is more restrictive than the per-mode allows_entry_in_rc_failsafe() check
    if (reason == ModeReason::GCS_COMMAND && copter.failsafe.radio) {
        return false;
    }
#endif
    
    // Cast integer to enum and call main set_mode implementation
    return copter.set_mode(static_cast<Mode::Number>(new_mode), reason);
}

/**
 * @brief Main loop integration point - executes current flight mode logic
 * 
 * @details This is THE critical function that integrates the flight mode framework
 * into the main control loop. Called at the main loop rate (typically 400Hz for
 * multirotors, 50-400Hz depending on vehicle and configuration).
 * 
 * Main Loop Integration:
 * - Copter::fast_loop() calls this function every loop iteration
 * - This function calls flightmode->run() which executes current mode logic
 * - Each mode's run() method implements its specific control behavior
 * - Mode run() methods typically:
 *   1. Process pilot inputs (RC sticks)
 *   2. Update navigation (waypoints, loiter, etc.)
 *   3. Call attitude and position controllers
 *   4. Set motor outputs via attitude_control->output_to_motors()
 * 
 * Pre-Mode Processing:
 * - Surface tracking invalidated for fresh calculation each loop
 * - Attitude gains reduced when landed to prevent ground oscillation
 *   (prevents aggressive control response to IMU noise when on ground)
 * 
 * Execution Flow Example (LOITER mode):
 * 1. update_flight_mode() called from fast_loop()
 * 2. flightmode->run() calls ModeLoiter::run()
 * 3. ModeLoiter::run() processes pilot input
 * 4. Updates loiter_nav controller
 * 5. Calls attitude_control to execute desired rates
 * 6. attitude_control calls motors->output()
 * 
 * @note Called at main loop rate (100Hz minimum, typically 400Hz)
 * @note Performance critical - must complete within loop time budget
 * @note flightmode pointer is never null (always points to valid mode)
 * 
 * @warning Do not call functions with unpredictable execution time from here.
 *          Main loop timing is critical for stable flight.
 * 
 * @see Copter::fast_loop() in Copter.cpp for main loop caller
 * @see Mode::run() virtual method implemented by each flight mode
 * @see Copter::set_mode() for how flightmode pointer is updated
 * 
 * Source: ArduCopter/mode.cpp:410-418
 */
void Copter::update_flight_mode()
{
#if AP_RANGEFINDER_ENABLED
    // Invalidate surface tracking altitude for this loop iteration
    // If a mode uses surface tracking, it will update this during its run() method
    surface_tracking.invalidate_for_logging();
#endif
    
    // Reduce attitude controller gains when landed to prevent oscillation
    // Ground contact creates IMU noise that can cause aggressive control response
    attitude_control->landed_gain_reduction(copter.ap.land_complete);

    // Execute current flight mode logic - this is where mode-specific control happens
    // Each mode's run() method implements its unique behavior:
    // - STABILIZE: Rate control from pilot sticks
    // - LOITER: Position hold with pilot input
    // - AUTO: Mission execution
    // - RTL: Return to launch sequence
    // - etc.
    flightmode->run();
}

/**
 * @brief Orchestrate cleanup and smooth transitions when exiting a flight mode
 * 
 * @details Called by set_mode() after new mode successfully initializes but before
 * switching the active flightmode pointer. Handles vehicle-level coordination for
 * mode transitions that individual modes cannot handle independently.
 * 
 * Transition Smoothing Operations:
 * 
 * 1. Manual-to-Automatic Throttle Transition:
 *    - When switching from manual throttle mode (STABILIZE, ACRO) to automatic
 *      (LOITER, AUTO), pre-load altitude controller integrator
 *    - Prevents sudden altitude change when controller takes over
 *    - Uses current pilot throttle to estimate required hover throttle
 * 
 * 2. Takeoff Cancellation:
 *    - Stops any in-progress takeoff sequences
 *    - Resets takeoff state machine to idle
 *    - Prevents old mode's takeoff from interfering with new mode
 * 
 * 3. Mode-Specific Cleanup:
 *    - Calls old_flightmode->exit() for mode-specific cleanup
 *    - Examples: stop missions, cancel loiter, reset mode state
 * 
 * 4. Helicopter-Specific Handling (HELI_FRAME):
 *    - Reset flybar passthrough when exiting ACRO
 *    - Pre-load collective ramp value for smooth transition
 *    - Disable inverted flight if new mode doesn't support it
 * 
 * Mode Transition Example (STABILIZE → LOITER):
 * - exit_mode() called with old=mode_stabilize, new=mode_loiter
 * - Manual throttle detected, integrator pre-loaded
 * - Takeoff stopped (in case STABILIZE was taking off)
 * - mode_stabilize.exit() called for any STABILIZE cleanup
 * - Returns to set_mode() which updates flightmode pointer
 * 
 * @param[in] old_flightmode Reference to pointer of mode being exited
 * @param[in] new_flightmode Reference to pointer of mode being entered
 * 
 * @note Called with motors armed or disarmed - must handle both cases safely
 * @note Old mode must be valid (not nullptr) when this is called
 * 
 * @warning Improper transition handling can cause altitude jumps, unexpected
 *          motor commands, or mode state confusion
 * 
 * @see Copter::set_mode() which calls this function
 * @see Mode::exit() virtual method for mode-specific cleanup
 * @see set_accel_throttle_I_from_pilot_throttle() for integrator pre-loading
 * 
 * Source: ArduCopter/mode.cpp:421-459
 */
void Copter::exit_mode(Mode *&old_flightmode,
                       Mode *&new_flightmode)
{
    // Smooth throttle transition when switching from manual to automatic flight modes
    // Manual modes: pilot directly controls throttle (STABILIZE, ACRO, SPORT)
    // Automatic modes: altitude controller manages throttle (LOITER, AUTO, GUIDED)
    if (old_flightmode->has_manual_throttle() && !new_flightmode->has_manual_throttle() && motors->armed() && !ap.land_complete) {
        // Pre-load altitude controller integrator with current pilot throttle
        // This assumes all manual flight modes use get_pilot_desired_throttle()
        // to translate pilot input to output throttle value
        set_accel_throttle_I_from_pilot_throttle();
    }

    // Cancel any takeoff sequences in progress from the old mode
    // Prevents old mode's takeoff state from interfering with new mode
    old_flightmode->takeoff_stop();

    // Call mode-specific cleanup function
    // Each mode implements exit() to perform any necessary shutdown operations:
    // - AUTO: Stop mission execution
    // - GUIDED: Clear guided targets
    // - LOITER: Reset loiter state
    // - etc.
    old_flightmode->exit();

#if FRAME_CONFIG == HELI_FRAME
    // Helicopter-specific mode transition handling
    
    // Reset flybar passthrough when exiting ACRO mode
    // Flybar passthrough allows direct control of swashplate in ACRO for 3D aerobatics
    // Must be firmly disabled when leaving ACRO to prevent unintended passthrough in other modes
    if (old_flightmode == &mode_acro) {
        attitude_control->use_flybar_passthrough(false, false);
        motors->set_acro_tail(false);
    }

    // Pre-load collective ramp value to prevent control twitch during mode transition
    // Helicopters use collective pitch for altitude control, and the ramp smooths transitions
    // between STABILIZE (ramped collective) and ACRO (direct collective)
    // heli_stab_col_ramp interpolates between stabilized and acro collective control
    if (!old_flightmode->has_manual_throttle()){
        if (new_flightmode == &mode_stabilize){
            // Entering STABILIZE from automatic mode - set ramp to full stabilization
            input_manager.set_stab_col_ramp(1.0);
        } else if (new_flightmode == &mode_acro){
            // Entering ACRO from automatic mode - set ramp to full passthrough
            input_manager.set_stab_col_ramp(0.0);
        }
    }

    // Disable inverted flight if new mode doesn't support it
    // Some helicopter modes (3D ACRO) allow inverted flight, others do not
    // Safety measure to prevent inverted flight in modes that aren't designed for it
    if (!new_flightmode->allows_inverted()) {
        attitude_control->set_inverted_flight(false);
    }
#endif //HELI_FRAME
}

/**
 * @brief Update notification system with current flight mode status
 * 
 * @details Updates AP_Notify flags to reflect current flight mode, which drives
 * external notification devices like LEDs, buzzers, and displays. Originally
 * implemented for OreoLED RGB LED device but now used by all notify backends.
 * 
 * Notification Updates:
 * - autopilot_mode flag: Indicates if mode is autonomous (AUTO, RTL, GUIDED)
 *   vs manual (STABILIZE, ACRO, ALT_HOLD). Used to change LED color schemes.
 * - flight_mode number: Current mode as uint8_t for devices that display mode number
 * - flight_mode string: 4-character mode name for text displays (e.g., "LOIT", "AUTO")
 * 
 * Notify Device Examples:
 * - Pixhawk external LED: Blue=manual modes, Green=autonomous modes
 * - OreoLED: Custom RGB patterns per mode
 * - Buzzer: Different beep patterns for manual vs autonomous
 * - OSD: Mode name displayed on FPV overlay
 * - NeoPixel: Addressable RGB LEDs with per-mode colors
 * 
 * @note Called automatically by set_mode() after successful mode change
 * @note May also be called explicitly if mode state changes without mode switch
 * 
 * @see AP_Notify for notification device framework
 * @see Mode::is_autopilot() to determine if mode is autonomous
 * @see Mode::name4() for 4-character mode name
 * 
 * Source: ArduCopter/mode.cpp:462-466
 */
void Copter::notify_flight_mode() {
    // Set autopilot flag: true for autonomous modes (AUTO, RTL, GUIDED, etc.)
    // false for manual modes (STABILIZE, ACRO, ALT_HOLD, etc.)
    AP_Notify::flags.autopilot_mode = flightmode->is_autopilot();
    
    // Set numeric mode identifier for displays that show mode number
    AP_Notify::flags.flight_mode = (uint8_t)flightmode->mode_number();
    
    // Set 4-character mode name string for text-based displays (OSD, LCDs, etc.)
    notify.set_flight_mode_str(flightmode->name4());
}

/**
 * @brief Transform pilot's stick input into desired lean angles (Mode base class default)
 * 
 * @details Converts normalized RC stick inputs (roll and pitch) into desired vehicle
 * lean angles in radians. This is the default implementation used by most flight modes
 * for processing pilot input into attitude commands.
 * 
 * Input Processing:
 * 1. RC failsafe check: Returns zero if no valid RC input
 * 2. Read normalized stick positions from RC channels (with deadzone applied)
 * 3. Transform stick input to lean angles using expo curves and angle limits
 * 4. Output desired roll and pitch angles in radians
 * 
 * Angle Limiting:
 * - angle_max_rad: Maximum lean angle configured by pilot (ANGLE_MAX parameter)
 * - angle_limit_rad: Additional limit that may be lower (e.g., for ALT_HOLD mode)
 * - Final angle is constrained to not exceed either limit
 * - Typical values: 45° (0.785 rad) max, sometimes limited to 30° for altitude modes
 * 
 * Coordinate Frames:
 * - Input: Normalized stick deflection (-1.0 to +1.0) after deadzone
 * - Output: Desired lean angles in radians (body frame, NED convention)
 * - Roll: Positive = lean right, Negative = lean left
 * - Pitch: Positive = lean back (nose up), Negative = lean forward (nose down)
 * 
 * @param[out] roll_out_rad Desired roll angle in radians (body frame)
 * @param[out] pitch_out_rad Desired pitch angle in radians (body frame)
 * @param[in] angle_max_rad Maximum lean angle allowed (from ANGLE_MAX param)
 * @param[in] angle_limit_rad Additional angle limit (may be lower than angle_max)
 * 
 * @note Called at main loop rate (typically 400Hz) by modes that use direct
 *       angle control from pilot sticks (STABILIZE, ALT_HOLD, LOITER, etc.)
 * 
 * @note Some modes override this method for specialized behavior (e.g., SPORT mode)
 * 
 * @warning Returns zero angles if RC input invalid - modes must handle this case
 *          to prevent unexpected vehicle behavior during RC failsafe
 * 
 * @see rc_input_to_roll_pitch_rad() for expo curve and scaling implementation
 * @see Mode::get_pilot_desired_velocity() for velocity-based control alternative
 * 
 * Source: ArduCopter/mode.cpp:470-481
 */
void Mode::get_pilot_desired_lean_angles_rad(float &roll_out_rad, float &pitch_out_rad, float angle_max_rad, float angle_limit_rad) const
{
    // RC failsafe check - if no valid input, return zero angles
    // This ensures vehicle holds attitude during RC signal loss rather than
    // continuing with stale stick positions
    if (!rc().has_valid_input()) {
        roll_out_rad = 0.0;
        pitch_out_rad = 0.0;
        return;
    }

    // Transform pilot's normalized stick input into desired lean angles
    // - Reads current stick positions with deadzone applied (norm_input_dz)
    // - Applies expo curves for smoother control feel
    // - Scales to angle limits and constrains output
    // - Returns angles in radians (body frame, NED convention)
    rc_input_to_roll_pitch_rad(channel_roll->norm_input_dz(), channel_pitch->norm_input_dz(), angle_max_rad,  angle_limit_rad, roll_out_rad, pitch_out_rad);
}

/**
 * @brief Transform pilot's stick input into desired velocity (alternative to angle control)
 * 
 * @details Converts normalized RC stick inputs into desired horizontal velocity commands
 * in the NE (North-East) frame. Used by modes that control velocity directly rather than
 * through lean angles (e.g., GUIDED_NOGPS, FLOWHOLD, position repositioning in LAND).
 * 
 * Processing Pipeline:
 * 1. RC failsafe check: Returns zero velocity if no valid input
 * 2. Read normalized stick positions (roll and pitch channels)
 * 3. Map stick inputs to velocity vector in body frame:
 *    - Pitch stick → X velocity (forward/back)
 *    - Roll stick → Y velocity (right/left)
 * 4. Rotate velocity from body frame to NE (North-East) earth frame
 * 5. Apply circular scaling to convert square input range to circular output
 * 6. Scale to maximum velocity limit
 * 
 * Circular Scaling:
 * - RC sticks have square input range: [-1, +1] × [-1, +1]
 * - Velocity should have circular limit: |vel| ≤ vel_max
 * - Without scaling, diagonal inputs would exceed vel_max (1.41× for full diagonal)
 * - Scaling maps the square to a circle, ensuring consistent maximum speed in all directions
 * - Algorithm: Find vector to edge of square in current direction, scale by circle/square ratio
 * 
 * Coordinate Frames:
 * - Input: Normalized stick deflection (-1.0 to +1.0) body frame
 * - Output: Desired velocity in cm/s, NE frame (North-East-Down convention)
 * - Pitch stick forward (negative) → North velocity (positive)
 * - Roll stick right (positive) → East velocity (positive)
 * 
 * @param[in] vel_max Maximum velocity magnitude in cm/s (speed limit)
 * 
 * @return Desired velocity vector in NE frame (cm/s), zero if no valid input
 * 
 * @note Units are cm/s (centimeters per second) for consistency with pos_control
 * @note Typical vel_max values: 500-2000 cm/s (5-20 m/s) depending on vehicle size
 * 
 * @warning Returns zero velocity (not current velocity) during RC failsafe.
 *          Caller must handle this appropriately (typically stop or hold position).
 * 
 * @see Mode::get_pilot_desired_lean_angles_rad() for angle-based control alternative
 * @see copter.rotate_body_frame_to_NE() for frame rotation
 * 
 * Source: ArduCopter/mode.cpp:484-508
 */
Vector2f Mode::get_pilot_desired_velocity(float vel_max) const
{
    Vector2f vel;

    // RC failsafe check - return zero velocity if no valid RC input
    if (!rc().has_valid_input()) {
        return vel;
    }
    
    // Fetch roll and pitch stick inputs (normalized with deadzone applied)
    float roll_out = channel_roll->norm_input_dz();
    float pitch_out = channel_pitch->norm_input_dz();

    // Convert stick inputs to velocity vector in body frame
    // Pitch stick controls forward/back (X), roll stick controls left/right (Y)
    // Note: Pitch inverted (-pitch_out) because forward stick is negative but North is positive
    vel = Vector2f(-pitch_out, roll_out);
    
    // Early return if zero input (avoid unnecessary rotation and scaling)
    if (vel.is_zero()) {
        return vel;
    }
    
    // Rotate velocity vector from body frame to North-East earth frame
    // Accounts for current vehicle heading (yaw angle)
    copter.rotate_body_frame_to_NE(vel.x, vel.y);

    // Transform square input range to circular output to prevent velocity exceeding
    // vel_max on diagonal stick inputs
    
    // vel_scalar is the vector to the edge of the ±1.0 square in the direction of the current input
    // This finds how far we could go in this direction within the square
    Vector2f vel_scalar = vel / MAX(fabsf(vel.x), fabsf(vel.y));
    
    // Scale output by the ratio of distance to unit circle vs distance to square edge
    // Then multiply by vel_max to get final velocity in cm/s
    // This ensures: |vel| ≤ vel_max for all stick positions
    vel *= vel_max / vel_scalar.length();
    
    return vel;
}

/**
 * @brief Check if takeoff sequence should be triggered (TakeOff state machine helper)
 * 
 * @details Determines if conditions are met to begin a takeoff sequence. Part of the
 * Mode::_TakeOff helper class that manages takeoff state across different flight modes.
 * This function is called by altitude-holding modes to decide when to transition from
 * "landed" state to "taking off" state.
 * 
 * Takeoff Trigger Conditions (ALL must be true):
 * 1. Vehicle is currently on the ground (land_complete flag set)
 * 2. Pilot is commanding upward motion (positive climb rate)
 * 3. Motors have completed runup (reached THROTTLE_UNLIMITED spool state)
 * 
 * Motor Spool States (preventing premature takeoff):
 * - SHUT_DOWN: Motors stopped → Cannot takeoff
 * - GROUND_IDLE: Motors idling → Cannot takeoff
 * - SPOOLING_UP: Motors ramping up → Cannot takeoff (WAIT)
 * - THROTTLE_UNLIMITED: Full throttle authority → CAN TAKEOFF
 * 
 * Safety Design:
 * - Prevents vehicle from leaving ground until motors are fully spooled
 * - For helicopters: Ensures main rotor at flight RPM before liftoff
 * - For multirotors: Ensures ESCs armed and motors spinning at correct speed
 * - Prevents sudden jumps or instability during takeoff
 * 
 * Typical Usage Pattern:
 * ```cpp
 * if (takeoff.triggered(target_climb_rate_cms)) {
 *     takeoff.start(constrain_float(target_climb_rate_cms, 0, wp_nav->get_default_speed_up()));
 *     altitude_state = Takeoff;
 * }
 * ```
 * 
 * @param[in] target_climb_rate_cms Desired climb rate in cm/s (from pilot input or auto)
 * 
 * @return true if takeoff should begin, false if should remain on ground
 * 
 * @note This does not start the takeoff - it only indicates conditions are met.
 *       Caller must call takeoff.start() to actually begin takeoff sequence.
 * 
 * @warning Do not bypass these checks - premature takeoff can cause instability
 * 
 * @see Mode::_TakeOff::start() to begin takeoff sequence
 * @see Mode::_TakeOff::running() to check if takeoff is in progress
 * @see get_alt_hold_state() which uses this to determine vehicle state
 * 
 * Source: ArduCopter/mode.cpp:510-527
 */
bool Mode::_TakeOff::triggered(const float target_climb_rate_cms) const
{
    // Check 1: Must be on the ground to takeoff
    if (!copter.ap.land_complete) {
        // Already flying - cannot trigger takeoff
        return false;
    }
    
    // Check 2: Must be commanding upward motion
    if (target_climb_rate_cms <= 0.0f) {
        // Zero or negative climb rate - pilot not requesting takeoff
        return false;
    }

    // Check 3: Motors must have completed runup sequence
    if (copter.motors->get_spool_state() != AP_Motors::SpoolState::THROTTLE_UNLIMITED) {
        // Motors still spooling up - hold aircraft on the ground until ready
        // This is critical for helicopters (rotor speed) and multirotors (ESC initialization)
        return false;
    }

    // All conditions met - safe to begin takeoff sequence
    return true;
}

/**
 * @brief Check if vehicle is disarmed or safely on the ground
 * 
 * @details Helper function to determine if vehicle is in a state where it's safe to
 * perform operations that should only occur when not flying. Used by various modes
 * to decide whether to execute landing logic, disable certain features, or allow
 * configuration changes.
 * 
 * Returns true if ANY of these conditions is true:
 * 1. Motors are disarmed (not armed at all)
 * 2. Motors armed but not auto-armed (armed on ground, throttle not raised)
 * 3. Land complete flag set (vehicle detected ground contact and settled)
 * 
 * Arming State Details:
 * - motors->armed(): Main arming state, true after arming checks pass
 * - copter.ap.auto_armed: Set true when throttle raised after arming
 * - copter.ap.land_complete: Set true when landing detection confirms ground contact
 * 
 * Common Use Cases:
 * - Compass calibration: Only allow when disarmed or landed
 * - GPS initialization: Can reset when on ground
 * - Failsafe actions: Different behavior for airborne vs grounded
 * - Mode transitions: Some modes restricted when flying
 * 
 * @return true if vehicle is disarmed OR landed, false if flying
 * 
 * @note This is intentionally conservative - returns true if unsure
 * @note Some modes may use more sophisticated landing detection
 * 
 * @see copter.ap.auto_armed for throttle-based flight detection
 * @see copter.ap.land_complete for landing detector status
 * 
 * Source: ArduCopter/mode.cpp:529-535
 */
bool Mode::is_disarmed_or_landed() const
{
    // Return true if ANY of these conditions indicate vehicle is not flying:
    // - Motors disarmed, OR
    // - Motors armed but throttle not raised (not auto-armed), OR
    // - Vehicle detected as landed (land_complete flag)
    if (!motors->armed() || !copter.ap.auto_armed || copter.ap.land_complete) {
        return true;
    }
    return false;
}

/**
 * @brief Command zero throttle and relax attitude control (safe ground handling)
 * 
 * @details Configures motor and attitude controller for safe operation on the ground
 * with zero throttle output. Used when vehicle is landed but may need to maintain
 * motor spool state (e.g., helicopters that must keep rotor spinning).
 * 
 * This function:
 * 1. Sets desired motor spool state (either THROTTLE_UNLIMITED or GROUND_IDLE)
 * 2. Commands zero attitude angles and rates (relaxed attitude hold)
 * 3. Sets throttle output to zero
 * 
 * Spool State Selection:
 * - spool_up = true: THROTTLE_UNLIMITED
 *   * Keeps motors ready for immediate takeoff
 *   * For helicopters: Maintains rotor at flight RPM
 *   * For multirotors: Keeps ESCs armed and responsive
 *   * Used in AUTO/GUIDED when on ground with interlock enabled
 * 
 * - spool_up = false: GROUND_IDLE
 *   * Motors spin at minimum safe idle speed
 *   * For helicopters: Rotor spins down to ground idle RPM
 *   * For multirotors: Props spin slowly for safety/arming indication
 *   * Used when preparing to disarm or in safe landed state
 * 
 * Attitude Control Relaxation:
 * - Commanding zero angles and rates prevents aggressive attitude corrections
 * - Attitude controller still active but not fighting for specific attitude
 * - Prevents motor twitching and ground resonance when landed
 * 
 * @param[in] spool_up true = keep motors spooled (THROTTLE_UNLIMITED),
 *                     false = spool down to idle (GROUND_IDLE)
 * 
 * @note Despite requesting THROTTLE_UNLIMITED, actual throttle output is zero
 *       This keeps motor controller ready but not outputting thrust
 * 
 * @note For traditional helicopters, maintaining spool prevents rotor from stopping,
 *       which would require a full restart sequence
 * 
 * @see zero_throttle_and_hold_attitude() for alternative with rate control
 * @see make_safe_ground_handling() for more comprehensive ground safety
 * 
 * Source: ArduCopter/mode.cpp:537-546
 */
void Mode::zero_throttle_and_relax_ac(bool spool_up)
{
    // Set desired motor spool state based on whether we need motors ready for immediate flight
    if (spool_up) {
        // Keep motors fully spooled and ready (rotor at flight RPM for helis)
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    } else {
        // Spool down to ground idle (minimum safe motor speed)
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
    }
    
    // Command zero attitude angles and zero yaw rate (relaxed attitude control)
    // This prevents aggressive attitude corrections while on the ground
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_rad(0.0f, 0.0f, 0.0f);
    
    // Set throttle output to zero
    // false = do not apply boost, throttle_filt = filter time constant
    attitude_control->set_throttle_out(0.0f, false, copter.g.throttle_filt);
}

/**
 * @brief Command zero throttle while maintaining attitude using rate control
 * 
 * @details Sets throttle to zero while commanding zero body frame rates to attitude
 * controller. This maintains current vehicle attitude without providing vertical thrust.
 * Used in specific scenarios where attitude hold is needed without altitude control.
 * 
 * Difference from zero_throttle_and_relax_ac():
 * - This function: Uses rate controller (input_rate_bf_roll_pitch_yaw_rads)
 *   * Maintains current attitude by commanding zero rates
 *   * More active attitude control, will resist disturbances
 *   * Does not change motor spool state
 * 
 * - zero_throttle_and_relax_ac(): Uses angle controller (input_euler_angle...)
 *   * Commands zero angles (more relaxed)
 *   * Explicitly sets motor spool state
 *   * Better for ground operations
 * 
 * Use Cases:
 * - Emergency situations where throttle must be zero but attitude maintained
 * - Certain failsafe conditions
 * - Transitional states in some flight modes
 * 
 * Body Frame Rate Control:
 * - input_rate_bf_roll_pitch_yaw_rads: Commands rates in body frame (radians/sec)
 * - Zero rates = hold current attitude
 * - Rate controller will generate control outputs to maintain attitude
 * - But throttle forced to zero prevents vertical control
 * 
 * @note Motor spool state is NOT changed by this function - remains at current state
 * @note Attitude control remains active - will attempt to maintain current attitude
 * @note Zero throttle means no altitude control - vehicle will fall if airborne
 * 
 * @warning If called while airborne, vehicle will descend (potentially crash)
 *          This is typically used only when landed or in controlled descent
 * 
 * @see zero_throttle_and_relax_ac() for version with spool state control
 * @see make_safe_ground_handling() for comprehensive ground safety
 * 
 * Source: ArduCopter/mode.cpp:548-553
 */
void Mode::zero_throttle_and_hold_attitude()
{
    // Command zero body frame rates to maintain current attitude
    // This keeps attitude controller active but commanding no rotation
    attitude_control->input_rate_bf_roll_pitch_yaw_rads(0.0f, 0.0f, 0.0f);
    
    // Set throttle output to zero (no vertical thrust)
    // false = do not apply boost, throttle_filt = filter time constant
    attitude_control->set_throttle_out(0.0f, false, copter.g.throttle_filt);
}

/**
 * @brief Comprehensive safe handling for vehicle on the ground waiting for takeoff
 * 
 * @details Manages all control systems for safe ground operation, ensuring proper motor
 * spool state, controller resets, and zero attitude commands. This is the primary function
 * for ground safety across multiple flight modes (AUTO, GUIDED, LOITER when landed).
 * 
 * Ground Safety Operations:
 * 1. Motor Spool State Management
 * 2. Controller Integrator Reset (prevent wind-up)
 * 3. Yaw Target Reset (prevent yaw drift on ground)
 * 4. Position Controller Relaxation
 * 5. Altitude Controller Zero Command
 * 6. Attitude Controller Zero Command
 * 
 * Motor Spool Control:
 * - force_throttle_unlimited = true: Keep motors fully spooled
 *   * Critical for traditional helicopters: Rotor must stay at flight RPM
 *   * Helicopter rotor STOPS when spooled to ground idle
 *   * Required in AUTO/GUIDED to obey motor interlock when on ground
 *   * Allows immediate takeoff without runup delay
 * 
 * - force_throttle_unlimited = false: Spool to ground idle
 *   * Safe for multirotors: Props at minimum safe speed
 *   * Reduces power consumption when waiting
 *   * Provides clear visual/audio indication of armed state
 * 
 * Integrator Wind-Up Prevention:
 * - Resets attitude rate controller integrators smoothly
 * - Prevents integrator accumulation while on ground
 * - Without reset, stale integrator values cause jumps during takeoff
 * - Smooth reset prevents sudden motor output changes
 * 
 * Spool State-Dependent Behavior:
 * - SHUT_DOWN/GROUND_IDLE: Reset yaw target and rate (full reset)
 * - SPOOLING_UP/THROTTLE_UNLIMITED/SPOOLING_DOWN: Continue normal operation
 * - Ensures smooth transitions through spool state changes
 * 
 * Position and Altitude Control:
 * - Relaxes velocity controller to prevent position control accumulation
 * - Forces altitude controller throttle output to decay to zero
 * - Updates controllers to process the relaxation commands
 * - Prevents sudden movements when transitioning to flight
 * 
 * @param[in] force_throttle_unlimited true = keep motors spooled up (THROTTLE_UNLIMITED),
 *                                     false = allow spool down to ground idle
 * 
 * @note This is called repeatedly while on the ground, not just once
 * @note Helicopter requirement: force_throttle_unlimited MUST be true in AUTO/GUIDED
 *       to prevent rotor from stopping
 * 
 * @warning For helicopters, spooling to ground idle stops the main rotor completely,
 *          requiring full startup sequence to fly again
 * 
 * @see zero_throttle_and_relax_ac() for simpler version without position control
 * @see Mode::run() in various modes for typical usage patterns
 * 
 * Source: ArduCopter/mode.cpp:555-594
 */
void Mode::make_safe_ground_handling(bool force_throttle_unlimited)
{
    // Set desired motor spool state based on vehicle type and mode requirements
    if (force_throttle_unlimited) {
        // Keep rotors at full spool (critical for helicopters in AUTO/GUIDED)
        // Helicopter main rotor stops at ground idle, requiring full restart
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    } else {
        // Spool down to ground idle (safe for multirotors, saves power)
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
    }

    // Reset attitude rate controller integrators smoothly to prevent wind-up
    // Aircraft is landed, so any accumulated integrator values are stale
    // Smooth reset prevents sudden motor output changes during the reset
    attitude_control->reset_rate_controller_I_terms_smoothly();
 
    // Perform spool state-dependent operations
    switch (motors->get_spool_state()) {
    case AP_Motors::SpoolState::SHUT_DOWN:
    case AP_Motors::SpoolState::GROUND_IDLE:
        // In idle states, reset yaw targets and rates
        // Prevents yaw drift accumulation while on ground
        // Ensures vehicle yaw target matches current heading when taking off
        attitude_control->reset_yaw_target_and_rate();
        break;
        
    case AP_Motors::SpoolState::SPOOLING_UP:
    case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
    case AP_Motors::SpoolState::SPOOLING_DOWN:
        // While transitioning through active states, continue operating normally
        // Don't reset yaw during active states to prevent discontinuities
        break;
    }

    // Relax horizontal position controller to prevent accumulation
    // Ensures position controller doesn't build up error while on ground
    pos_control->relax_velocity_controller_NE();
    pos_control->update_NE_controller();
    
    // Relax altitude controller and force throttle output to decay to zero
    // 0.0f argument forces immediate decay rather than gradual reduction
    pos_control->relax_U_controller(0.0f);
    pos_control->update_U_controller();
    
    // Command zero attitude angles and zero yaw rate
    // Prevents attitude controller from trying to hold specific angles while landed
    // May need to be moved out depending on mode requirements
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_rad(0.0f, 0.0f, 0.0f);
}

/**
 * @brief Get height above ground estimate for landing and terrain following
 * 
 * @details Provides the best available altitude above ground level (AGL) estimate using
 * multiple sensor sources with intelligent fallback. Critical for safe landing operations,
 * terrain following, and ground proximity detection. Uses prioritized sensor fusion to
 * maximize accuracy and reliability.
 * 
 * Altitude Source Priority (best to worst):
 * 
 * 1. Rangefinder (Highest Priority - Direct Measurement)
 *    - Lidar, sonar, or radar rangefinder providing direct ground distance
 *    - Interpolated to smooth out sensor noise and dropouts
 *    - Most accurate for landing (cm-level precision possible)
 *    - Limited maximum range (typically 5-40m depending on sensor)
 *    - Can fail over non-reflective surfaces (water, glass, etc.)
 * 
 * 2. Terrain Database (ABOVE_TERRAIN altitude)
 *    - Uses GPS position + terrain database to calculate AGL
 *    - Requires terrain data loaded and position controller active
 *    - Good for general terrain following over wide areas
 *    - Accuracy limited by terrain database resolution (~30m typical)
 *    - Requires GPS fix and position estimate
 * 
 * 3. Current Altitude (Fallback - "Flat Earth" Assumption)
 *    - Returns absolute altitude from home (AMSL - home altitude)
 *    - Assumes takeoff location is at ground level
 *    - Used when no better source available
 *    - Can be very inaccurate if terrain slopes or flying from elevation
 * 
 * Usage in Landing Operations:
 * - Determines when to slow descent rate (approaching g2.land_alt_low)
 * - Triggers land_complete detection when close to ground
 * - Adjusts landing speed based on proximity to ground
 * - Used by precision landing to detect final touchdown
 * 
 * Position Controller Active Check:
 * - If position controller not active, skip terrain database lookup
 * - Position controller inactive usually means GPS lost or EKF not healthy
 * - Fall back to simple altitude in this case for safety
 * 
 * @return Estimated height above ground in centimeters
 * 
 * @note Return value can be negative if position estimate places vehicle below home
 * @note Rangefinder readings are preferred even if less smooth than terrain data
 * @note Final fallback assumes flat earth - can be inaccurate on slopes
 * 
 * @warning Do not use for obstacle avoidance - designed for downward-looking only
 * @warning Rangefinder can give false readings over water or reflective surfaces
 * 
 * @see land_run_vertical_control() for primary usage in landing
 * @see copter.get_rangefinder_height_interpolated_cm() for rangefinder source
 * @see Location::get_alt_cm(AltFrame::ABOVE_TERRAIN) for terrain database source
 * 
 * Source: ArduCopter/mode.cpp:595-613
 */
int32_t Mode::get_alt_above_ground_cm(void) const
{
    int32_t alt_above_ground_cm;
    
    // Priority 1: Use rangefinder if available (most accurate for landing)
    if (copter.get_rangefinder_height_interpolated_cm(alt_above_ground_cm)) {
        // Rangefinder providing valid interpolated reading - use it
        return alt_above_ground_cm;
    }
    
    // Priority 2: Check if we can use terrain database
    // Only attempt if position controller is active (implies GPS and EKF healthy)
    if (!pos_control->is_active_NE()) {
        // Position controller not active - no GPS or EKF issues
        // Fall back to simple altitude (flat earth assumption)
        return copter.current_loc.alt;
    }
    
    // Position controller active - try terrain database lookup
    if (copter.current_loc.get_alt_cm(Location::AltFrame::ABOVE_TERRAIN, alt_above_ground_cm)) {
        // Successfully got altitude above terrain from database
        return alt_above_ground_cm;
    }

    // Priority 3: Final fallback - assume the Earth is flat
    // Return current altitude (AMSL - home altitude)
    // This assumes we took off from ground level
    return copter.current_loc.alt;
}

/**
 * @brief Execute vertical control for landing with adaptive descent rates
 * 
 * @details Manages landing descent with intelligent speed control that adapts to altitude
 * above ground, precision landing requirements, and pilot pause commands. Implements a
 * two-stage landing profile with faster descent at altitude transitioning to slower final
 * approach near the ground.
 * 
 * Landing Descent Profile:
 * 
 * Stage 1: High Altitude Descent (above g2.land_alt_low)
 * - Descends at g.land_speed_high (if configured) or max_speed_down
 * - Uses sqrt controller to smoothly approach target altitude
 * - Constrained to not exceed g.land_speed (safety limit)
 * - Typical: 150-250 cm/s descent rate
 * 
 * Stage 2: Final Approach (below g2.land_alt_low or land_complete_maybe)
 * - Switches to slower g.land_speed (final descent rate)
 * - Ignores altitude limits (descends until ground contact)
 * - Typical: 50-100 cm/s descent rate
 * - Enables gentle touchdown
 * 
 * Sqrt Controller for Smooth Transitions:
 * - Computes descent rate as sqrt(altitude_error)
 * - Automatically slows as approaching target altitude
 * - Prevents abrupt speed changes
 * - Respects position controller P gain and acceleration limits
 * 
 * Precision Landing Integration:
 * - When prec land active and target acquired:
 *   * Pauses descent if horizontal error too large (ensures accuracy)
 *   * Slows descent near ground (35-200cm range) based on horizontal error
 *   * Balances landing accuracy vs descent speed
 *   * Can override normal descent profile for precision
 * 
 * Precision Landing Descent Logic:
 * - If >max_horiz_pos_error: Stop descending (cmb_rate = 0)
 * - If 35-200cm above target: Slow descent proportional to horizontal error
 *   * Perfect alignment: 50% of normal land_speed
 *   * Off target: Slower, down to precland_min_descent_speed (10cm/s)
 * - Fast descend mode: Uses normal rates (bypass slowdown)
 * 
 * @param[in] pause_descent true = hold current altitude (do not descend),
 *                          false = execute normal landing descent
 * 
 * Parameters Used:
 * - g.land_speed: Final descent rate (cm/s, negative)
 * - g.land_speed_high: Initial descent rate (cm/s, negative, 0=use max)
 * - g2.land_alt_low: Altitude to switch to slow descent (cm, min 100cm)
 * 
 * @note Descent rates are negative values (down is negative in NED frame)
 * @note ignore_descent_limit allows descending below altitude limits when close to ground
 * 
 * @warning If pause_descent true while airborne, vehicle will hold altitude indefinitely
 * @warning Precision landing can fully stop descent if target alignment lost
 * 
 * @see land_run_horizontal_control() for complementary horizontal control
 * @see get_alt_above_ground_cm() for altitude source
 * @see land_run_normal_or_precland() for typical usage pattern
 * 
 * Source: ArduCopter/mode.cpp:616-677
 */
void Mode::land_run_vertical_control(bool pause_descent)
{
    float cmb_rate = 0;  // Desired climb rate (negative for descent)
    bool ignore_descent_limit = false;
    
    if (!pause_descent) {
        // Not paused - compute normal landing descent rate

        // Determine if we should ignore altitude limits (close to ground or maybe landed)
        // Once below land_alt_low or possibly landed, descend until ground contact detected
        ignore_descent_limit = (MAX(g2.land_alt_low,100) > get_alt_above_ground_cm()) || copter.ap.land_complete_maybe;

        // Determine maximum descent velocity for current phase
        float max_land_descent_velocity;
        if (g.land_speed_high > 0) {
            // User configured high-altitude descent speed
            max_land_descent_velocity = -g.land_speed_high;
        } else {
            // Use position controller's default maximum descent speed
            max_land_descent_velocity = pos_control->get_max_speed_down_cms();
        }

        // Never descend faster than final land_speed (safety constraint)
        // This ensures we don't speed up for landing, only slow down
        max_land_descent_velocity = MIN(max_land_descent_velocity, -abs(g.land_speed));

        // Compute desired descent rate using sqrt controller
        // This creates smooth transition as we approach land_alt_low
        // Without constraints, this would cause hover at land_alt_low
        cmb_rate = sqrt_controller(MAX(g2.land_alt_low,100)-get_alt_above_ground_cm(), 
                                   pos_control->get_pos_U_p().kP(), 
                                   pos_control->get_max_accel_U_cmss(), 
                                   G_Dt);

        // Constrain descent rate between maximum and minimum speeds
        // Ensures we descend at least at land_speed but not faster than max
        cmb_rate = constrain_float(cmb_rate, max_land_descent_velocity, -abs(g.land_speed));

#if AC_PRECLAND_ENABLED
        // Modify descent rate for precision landing if active
        const bool navigating = pos_control->is_active_NE();
        bool doing_precision_landing = !copter.ap.land_repo_active && copter.precland.target_acquired() && navigating;

        if (doing_precision_landing) {
            // Precision landing is active - may need to pause or slow descent
            Vector2f target_pos;
            float target_error_cm = 0.0f;
            
            // Get horizontal distance to precision landing target
            if (copter.precland.get_target_position_cm(target_pos)) {
                const Vector2f current_pos = pos_control->get_pos_estimate_NEU_cm().xy().tofloat();
                // Calculate horizontal error (target is this many cm away)
                target_error_cm = (target_pos - current_pos).length();
            }
            
            // Check if horizontal error is too large to continue descending
            const float max_horiz_pos_error_cm = copter.precland.get_max_xy_error_before_descending_cm();
            Vector3f target_pos_meas;
            copter.precland.get_target_position_measurement_cm(target_pos_meas);
            
            if (target_error_cm > max_horiz_pos_error_cm && !is_zero(max_horiz_pos_error_cm)) {
                // Horizontal error too large - pause descent
                // Prevents landing off-target; wait for better alignment
                cmb_rate = 0.0f;
                
            } else if (target_pos_meas.z > 35.0f && target_pos_meas.z < 200.0f && !copter.precland.do_fast_descend()) {
                // Very close to ground (35-200cm) during precision landing
                // Slow down descent to ensure accurate landing on target
                
                // Precision landing parameters for final approach
                const float precland_acceptable_error_cm = 15.0f;    // Target accuracy threshold
                const float precland_min_descent_speed_cms = 10.0f;  // Minimum descent speed
                const float max_descent_speed_cms = abs(g.land_speed)*0.5f;  // Max 50% of normal
                
                // Compute slowdown based on horizontal error
                // Larger error = slower descent for more time to correct
                const float land_slowdown = MAX(0.0f, target_error_cm*(max_descent_speed_cms/precland_acceptable_error_cm));
                
                // Set descent rate: slower if off target, faster if well aligned
                cmb_rate = MIN(-precland_min_descent_speed_cms, -max_descent_speed_cms+land_slowdown);
            }
        }
#endif
    }

    // Command the position controller with computed descent rate
    // ignore_descent_limit allows descending below normal altitude limits when near ground
    pos_control->land_at_climb_rate_cm(cmb_rate, ignore_descent_limit);
    pos_control->update_U_controller();
}

/**
 * @brief Execute horizontal position control during landing with pilot repositioning support
 * 
 * @details Manages horizontal position control during landing with support for pilot override
 * repositioning, precision landing, and automatic attitude limiting near the ground. Provides
 * safe handling of position drift while allowing pilot intervention and precision landing
 * target tracking.
 * 
 * Landing Horizontal Control Modes:
 * 
 * 1. Precision Landing (Highest Priority if active)
 *    - Tracks precision landing target position and velocity
 *    - Commands position controller to follow moving or stationary target
 *    - Requires: target_acquired() && !land_repo_active
 *    - Handles target velocity for moving landing pads
 * 
 * 2. Pilot Repositioning (if pilot provides input)
 *    - Allows pilot to adjust landing position during descent
 *    - Converts stick inputs to horizontal velocity commands
 *    - Max velocity = half of waypoint acceleration (ensures fast stop)
 *    - Sets land_repo_active flag when pilot takes control
 *    - Can disable precision landing while pilot repositioning
 * 
 * 3. Loiter Hold (default if no pilot input or precision landing)
 *    - Maintains current horizontal position
 *    - Softens position hold if land_complete_maybe (reduce oscillation)
 *    - Zero velocity command to position controller
 * 
 * Pilot Throttle Abort:
 * - If THR_BEHAVE_HIGH_THROTTLE_CANCELS_LAND enabled:
 *   * Monitors filtered throttle input
 *   * If throttle > LAND_CANCEL_TRIGGER_THR: Exit to LOITER or ALT_HOLD
 *   * Provides escape from unwanted automatic landing
 * 
 * Attitude Limiting Near Ground (g2.wp_navalt_min):
 * - Progressively limits attitude angle as approaching ground
 * - Prevents commanded roll-over during landing
 * - Critical for helicopters with position drift after touchdown
 * - Interpolation: 7° at wp_navalt_min, normal max at (wp_navalt_min+1m)
 * - Scales thrust vector to respect angle limits
 * 
 * Position Controller Softening:
 * - When land_complete_maybe set (possibly landed):
 *   * Softens position controller gains
 *   * Reduces aggressive position corrections
 *   * Prevents oscillations when on ground
 * 
 * Precision Landing State Logic:
 * - prec_land_active = true when:
 *   * Precision landing target acquired AND
 *   * Pilot not repositioning (land_repo_active = false)
 * - Pilot input disables precision landing
 * - After pilot input ends, can re-enable if allow_precland_after_reposition()
 * 
 * @note Called every loop iteration during landing (typically 400Hz)
 * @note Pilot repositioning uses SIMPLE mode transforms if enabled
 * @note Maximum repositioning velocity = waypoint accel * 0.5 (ensures sub-second stop)
 * 
 * Parameters Used:
 * - g.throttle_behavior: Throttle behavior bitmask (includes cancellation option)
 * - g.land_repositioning: Enable/disable pilot repositioning during land
 * - g2.wp_navalt_min: Altitude below which to limit navigation attitude (meters)
 * 
 * @warning Attitude limiting near ground can cause position drift to prevent rollover
 * @warning Pilot repositioning disables precision landing until sticks centered
 * 
 * @see land_run_vertical_control() for complementary vertical control
 * @see land_run_normal_or_precland() for combined usage
 * @see precland_run() for precision landing state machine
 * 
 * Source: ArduCopter/mode.cpp:679-778
 */
void Mode::land_run_horizontal_control()
{
    Vector2f vel_correction;  // Pilot repositioning velocity command

    // Soften position controller if we might be landed
    // Reduces aggressive position corrections near/on ground
    if (copter.ap.land_complete_maybe) {
        pos_control->soften_for_landing_NE();
    }

    // Process pilot inputs for repositioning and landing abort
    if (rc().has_valid_input()) {
        // Check for pilot throttle abort (escape from landing)
        if ((g.throttle_behavior & THR_BEHAVE_HIGH_THROTTLE_CANCELS_LAND) != 0 && 
            copter.rc_throttle_control_in_filter.get() > LAND_CANCEL_TRIGGER_THR) {
            // Pilot raised throttle high - abort landing
            LOGGER_WRITE_EVENT(LogEvent::LAND_CANCELLED_BY_PILOT);
            
            // Exit to LOITER if available, otherwise ALT_HOLD
            if (!set_mode(Mode::Number::LOITER, ModeReason::THROTTLE_LAND_ESCAPE)) {
                set_mode(Mode::Number::ALT_HOLD, ModeReason::THROTTLE_LAND_ESCAPE);
            }
        }

        // Process pilot repositioning if enabled
        if (g.land_repositioning) {
            // Apply SIMPLE mode transformation to pilot inputs if enabled
            // Converts body-frame inputs to earth-frame based on takeoff heading
            update_simple_mode();

            // Convert pilot stick inputs to horizontal velocity command
            // Use half of maximum waypoint acceleration as max velocity
            // This ensures vehicle can stop from full speed in less than 1 second
            const float max_pilot_vel = wp_nav->get_wp_acceleration_cmss() * 0.5;
            vel_correction = get_pilot_desired_velocity(max_pilot_vel);

            // Track if pilot is actively repositioning
            if (!vel_correction.is_zero()) {
                // Pilot providing input - enable repositioning mode
                if (!copter.ap.land_repo_active) {
                    LOGGER_WRITE_EVENT(LogEvent::LAND_REPO_ACTIVE);
                }
                copter.ap.land_repo_active = true;
                
#if AC_PRECLAND_ENABLED
            } else {
                // No pilot input right now - check if we should re-enable precision landing
                if (copter.precland.allow_precland_after_reposition()) {
                    // Conditions met to resume precision landing
                    copter.ap.land_repo_active = false;
                }
#endif
            }
        }
    }

    // Determine if precision landing should be active
    // Active if target acquired AND pilot not repositioning
    copter.ap.prec_land_active = false;
    
#if AC_PRECLAND_ENABLED
    copter.ap.prec_land_active = !copter.ap.land_repo_active && copter.precland.target_acquired();
    
    if (copter.ap.prec_land_active) {
        // Precision landing active - track the landing target
        Vector2f target_pos, target_vel;
        
        // Get precision landing target position
        if (!copter.precland.get_target_position_cm(target_pos)) {
            // Target position not available - use current position as fallback
            target_pos = pos_control->get_pos_estimate_NEU_cm().xy().tofloat();
        }
        
        // Get velocity of the precision landing target
        // Handles moving landing pads (e.g., ship decks)
        // target_vel will remain zero if landing target is stationary
        copter.precland.get_target_velocity_cms(pos_control->get_vel_estimate_NEU_cms().xy(), target_vel);

        // Command position controller to track target position and velocity
        Vector2f zero;  // Zero acceleration (let controller compute)
        Vector2p landing_pos = target_pos.topostype();
        pos_control->input_pos_vel_accel_NE_cm(landing_pos, target_vel, zero);
    }
#endif

    if (!copter.ap.prec_land_active) {
        // Not doing precision landing - use loiter or pilot repositioning
        Vector2f accel;  // Zero acceleration
        pos_control->input_vel_accel_NE_cm(vel_correction, accel);
    }

    // Run horizontal position controller
    pos_control->update_NE_controller();
    Vector3f thrust_vector = pos_control->get_thrust_vector();

    // Apply attitude limiting near ground if configured
    if (g2.wp_navalt_min > 0) {
        // User has requested altitude below which navigation attitude is limited
        // This prevents commanded roll-over during landing, particularly critical
        // for helicopters which can experience position drift after touchdown
        
        // Linearly interpolate attitude limit:
        // - 7° at wp_navalt_min altitude
        // - Normal max angle at (wp_navalt_min + 1m)
        const float attitude_limit_rad = linear_interpolate(radians(7), 
                                                            attitude_control->lean_angle_max_rad(), 
                                                            get_alt_above_ground_cm(),
                                                            g2.wp_navalt_min*100U, 
                                                            (g2.wp_navalt_min+1)*100U);
        
        // Convert angle limit to maximum horizontal thrust component
        const float thrust_vector_max = sinf(attitude_limit_rad) * GRAVITY_MSS * 100.0f;
        const float thrust_vector_mag = thrust_vector.xy().length();
        
        if (thrust_vector_mag > thrust_vector_max) {
            // Thrust exceeds limit - scale down horizontal components
            float ratio = thrust_vector_max / thrust_vector_mag;
            thrust_vector.x *= ratio;
            thrust_vector.y *= ratio;

            // Inform position controller that external limit is being applied
            pos_control->set_externally_limited_NE();
        }
    }

    // Command attitude controller with (possibly limited) thrust vector
    attitude_control->input_thrust_vector_heading(thrust_vector, auto_yaw.get_heading());
}

/**
 * @brief Entry point for landing control - selects normal or precision landing
 * 
 * @details Determines whether to use simple landing control (horizontal + vertical)
 * or the full precision landing state machine based on precision landing availability,
 * enablement, and descent pause state. Provides single entry point for all landing
 * modes to access appropriate landing control logic.
 * 
 * Control Logic Selection:
 * 
 * 1. Simple Landing (land_run_horiz_and_vert_control):
 *    - When precision landing disabled (AP build or parameter)
 *    - When descent paused (pause_descent = true)
 *    - When precision landing not enabled via parameter
 *    - Uses: Loiter position hold + controlled vertical descent
 * 
 * 2. Precision Landing State Machine (precland_run):
 *    - When precision landing enabled (build + parameter)
 *    - When descent NOT paused (pause_descent = false)
 *    - Handles: Target search, tracking, retries, failsafes
 *    - State machine manages full landing procedure
 * 
 * Precision Landing Requirements (all must be true):
 * - AC_PRECLAND_ENABLED = 1 (compiled into firmware)
 * - copter.precland.enabled() = true (PLND_ENABLED parameter)
 * - pause_descent = false (ready to descend)
 * 
 * Common Usage by Flight Modes:
 * - Land mode: Calls this function every loop iteration
 * - Auto mode: Calls during DO_LAND mission command execution
 * - RTL mode: Calls during final descent phase
 * - SmartRTL mode: Calls during final descent phase
 * 
 * Pause Descent Scenarios:
 * - Initial mode entry delay (allow position stabilization)
 * - Terrain altitude acquisition pending
 * - Rangefinder data not yet valid
 * - Other mode-specific reasons to hold altitude
 * 
 * @param[in] pause_descent If true, hold current altitude while maintaining horizontal position
 *                          If false, execute normal descent profile
 * 
 * @note This is the primary landing entry point called by flight modes
 * @note Precision landing requires both compile-time and runtime enabling
 * @note When paused, vertical descent stops but horizontal control continues
 * 
 * @warning Ensure pause_descent correctly reflects readiness to land
 * @warning Do not pause indefinitely - can drain battery or trigger failsafes
 * 
 * @see land_run_horiz_and_vert_control() for simple landing implementation
 * @see precland_run() for precision landing state machine
 * @see Land::run() for primary usage example
 * 
 * Source: ArduCopter/mode.cpp:782-797
 */
void Mode::land_run_normal_or_precland(bool pause_descent)
{
#if AC_PRECLAND_ENABLED
    // Check if conditions allow precision landing state machine
    if (pause_descent || !copter.precland.enabled()) {
        // Cannot use precision landing - use simple landing control
        // Reasons: Either descent is paused OR precision landing is disabled
        land_run_horiz_and_vert_control(pause_descent);
    } else {
        // Precision landing available and descent not paused
        // Use full state machine for precision landing procedure
        precland_run();
    }
#else
    // Precision landing not compiled into firmware
    // Always use simple landing control
    land_run_horiz_and_vert_control(pause_descent);
#endif
}

#if AC_PRECLAND_ENABLED
/**
 * @brief Navigate to a retry position during precision landing attempts
 * 
 * @details Executes 3D position navigation to a retry position commanded by the precision
 * landing state machine when initial landing attempt failed. Handles pilot override
 * and throttle abort during retry maneuvers. Converts NED frame retry coordinates to
 * NEU frame position commands for the position controller.
 * 
 * Retry Position Navigation:
 * - Input position in NED frame (North-East-Down) in meters
 * - Converts to NEU frame (North-East-Up) in centimeters for pos_control
 * - Commands both horizontal (NE) and vertical (U) position simultaneously
 * - Uses default velocity (0.0 cm/s) and acceleration (1000 cm/s^2) limits
 * 
 * Pilot Override Detection:
 * - Monitors pilot stick inputs during retry maneuver
 * - If pilot provides roll or pitch input (with land_repositioning enabled):
 *   * Sets land_repo_active flag
 *   * Logs LAND_REPO_ACTIVE event
 *   * State machine will see this flag and cancel further retries
 * - Allows pilot to take manual control at any point
 * 
 * Throttle Abort Handling:
 * - If THR_BEHAVE_HIGH_THROTTLE_CANCELS_LAND configured:
 *   * Monitors filtered throttle input
 *   * If throttle > LAND_CANCEL_TRIGGER_THR:
 *     - Logs LAND_CANCELLED_BY_PILOT
 *     - Exits to LOITER mode (or ALT_HOLD if LOITER unavailable)
 *   * Provides escape from precision landing procedure
 * 
 * Coordinate Frame Transformations:
 * - Input: NED meters (North positive, East positive, Down positive)
 * - Conversion to NEU: (x, y, -z) to invert vertical axis
 * - Scale to centimeters: multiply by 100
 * - Output: NEU centimeters for position controller
 * 
 * Position Controller Usage:
 * - input_pos_NEU_cm(): Commands 3D position target
 * - update_NE_controller(): Runs horizontal position control
 * - update_U_controller(): Runs vertical position control
 * - Both controllers generate thrust vector components
 * 
 * Typical Retry Scenarios:
 * - Target temporarily lost during descent
 * - Position error exceeds acceptable threshold
 * - Need to climb and re-approach from better angle
 * - Environmental conditions caused tracking failure
 * 
 * @param[in] retry_pos Retry position in NED frame, in meters (North, East, Down)
 *                      Commanded by precision landing state machine
 * 
 * @note Called by precland_run() when state machine returns RETRYING status
 * @note Pilot override sets land_repo_active which disables further retries
 * @note Coordinate conversion: NED (input) → NEU (position controller)
 * @note Unit conversion: meters (input) → centimeters (position controller)
 * 
 * @warning Ensure retry position is within safe flight envelope
 * @warning Multiple retries can drain battery - state machine should limit attempts
 * 
 * @todo Consolidate duplicate pilot repositioning code (exists in land_run_horizontal_control too)
 * 
 * @see precland_run() for state machine that commands retry positions
 * @see land_run_horizontal_control() for similar pilot override logic
 * @see AC_PrecLand_StateMachine for retry logic and position calculation
 * 
 * Source: ArduCopter/mode.cpp:802-843
 */
void Mode::precland_retry_position(const Vector3f &retry_pos)
{
    // Check for pilot interventions
    if (rc().has_valid_input()) {
        // Check for throttle abort (pilot wants to escape landing)
        if ((g.throttle_behavior & THR_BEHAVE_HIGH_THROTTLE_CANCELS_LAND) != 0 && 
            copter.rc_throttle_control_in_filter.get() > LAND_CANCEL_TRIGGER_THR) {
            // Pilot raised throttle - abort landing procedure
            LOGGER_WRITE_EVENT(LogEvent::LAND_CANCELLED_BY_PILOT);
            
            // Exit to LOITER if available, otherwise ALT_HOLD
            if (!set_mode(Mode::Number::LOITER, ModeReason::THROTTLE_LAND_ESCAPE)) {
                set_mode(Mode::Number::ALT_HOLD, ModeReason::THROTTLE_LAND_ESCAPE);
            }
        }

        // Check for pilot repositioning override
        // @todo This code duplicated from land_run_horizontal_control() - should be consolidated
        if (g.land_repositioning) {
            float target_roll_rad = 0.0f;
            float target_pitch_rad = 0.0f;
            
            // Convert pilot stick inputs to desired lean angles
            get_pilot_desired_lean_angles_rad(target_roll_rad, target_pitch_rad, 
                                             loiter_nav->get_angle_max_rad(), 
                                             attitude_control->get_althold_lean_angle_max_rad());

            // Check if pilot is actively moving the sticks
            if (!is_zero(target_roll_rad) || !is_zero(target_pitch_rad)) {
                // Pilot taking control
                if (!copter.ap.land_repo_active) {
                    LOGGER_WRITE_EVENT(LogEvent::LAND_REPO_ACTIVE);
                }
                // Set flag - state machine will check this and cancel further retries
                copter.ap.land_repo_active = true;
            }
        }
    }

    // Convert retry position from NED meters to NEU centimeters
    // Input: NED frame (North, East, Down) in meters
    // Step 1: Invert vertical axis (Down → Up)
    Vector3p retry_pos_NEU{retry_pos.x, retry_pos.y, retry_pos.z * -1.0f};
    
    // Step 2: Convert from meters to centimeters
    retry_pos_NEU = retry_pos_NEU * 100.0f;
    
    // Command 3D position to position controller
    // Parameters: position (NEU cm), velocity limit (0 = use default), accel limit (1000 cm/s^2)
    pos_control->input_pos_NEU_cm(retry_pos_NEU, 0.0f, 1000.0f);

    // Run horizontal and vertical position controllers
    pos_control->update_NE_controller();  // North-East (horizontal)
    pos_control->update_U_controller();   // Up (vertical)

    // Command attitude controller with combined thrust vector
    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.get_heading());
}

/**
 * @brief Execute precision landing state machine with retry and failsafe handling
 * 
 * @details Runs the complete precision landing state machine which handles target tracking,
 * landing retries on failure, and failsafe actions. Provides intelligent landing behavior
 * that can recover from temporary target loss or positioning errors. Handles all aspects
 * of precision landing from initial descent through completion or failsafe.
 * 
 * State Machine States and Actions:
 * 
 * 1. DESCEND (Normal Operation):
 *    - Target visible and tracking
 *    - Executes normal landing descent
 *    - Calls land_run_horiz_and_vert_control()
 *    - Horizontal control tracks target, vertical descends
 * 
 * 2. RETRYING (Recovery from Failure):
 *    - Target temporarily lost or position error too large
 *    - State machine calculates retry position (often climb and reposition)
 *    - Calls precland_retry_position() with retry coordinates
 *    - Attempts to reacquire target from better position/angle
 * 
 * 3. FAILSAFE (Exhausted Retries or Critical Condition):
 *    - Multiple retry attempts failed OR critical condition detected
 *    - Two failsafe actions available:
 *      a. DESCEND: Land normally at current position (target definitely lost)
 *      b. HOLD_POS: Hold position and altitude (wait for pilot intervention)
 *    - Action determined by state machine based on conditions
 * 
 * 4. ERROR (Internal State Machine Error):
 *    - Should never occur - indicates software bug
 *    - Logs internal error for debugging
 *    - Falls through to normal descent (safest action)
 * 
 * Pilot Override Behavior:
 * - If land_repo_active flag set (pilot moved sticks):
 *   * State machine bypassed
 *   * Normal landing executed at current position
 *   * No retries or failsafes attempted
 *   * Pilot in full control
 * 
 * State Machine Update:
 * - update() called every loop iteration (~400Hz)
 * - Returns current state status
 * - Updates retry_pos if state is RETRYING
 * - Manages internal state transitions automatically
 * 
 * Typical Precision Landing Sequence:
 * 1. Initial DESCEND - tracking target successfully
 * 2. Target lost → RETRYING - climb and reposition
 * 3. Target reacquired → DESCEND - continue landing
 * 4. Target lost again → RETRYING - second attempt
 * 5. Still no target → FAILSAFE - land at current position or hold
 * 
 * Common Failsafe Triggers:
 * - Maximum retry attempts exceeded
 * - Target lost for extended duration
 * - Position error consistently too large
 * - Low altitude without target lock
 * - Battery critically low during retries
 * 
 * Integration with Flight Modes:
 * - Called by land_run_normal_or_precland() when precision landing enabled
 * - Used by: Land mode, Auto mode (DO_LAND), RTL mode, SmartRTL mode
 * - Provides consistent precision landing across all modes
 * 
 * @note This is the main precision landing entry point - manages complete procedure
 * @note State machine handles retry logic automatically based on target visibility
 * @note Pilot override immediately disables state machine and retries
 * @note Failsafe actions ensure vehicle safety even if landing fails
 * @note Called at main loop rate (typically 400Hz)
 * 
 * @warning State machine can execute multiple retries - monitor battery level
 * @warning HOLD_POS failsafe can hover indefinitely - may need manual intervention
 * @warning ERROR state indicates software bug - report and investigate
 * 
 * @see precland_retry_position() for retry position navigation
 * @see land_run_horiz_and_vert_control() for normal landing control
 * @see land_run_normal_or_precland() for entry point that calls this function
 * @see AC_PrecLand_StateMachine for state machine implementation
 * 
 * Source: ArduCopter/mode.cpp:847-888
 */
void Mode::precland_run()
{
    // Check if pilot has taken manual control during precision landing
    if (!copter.ap.land_repo_active) {
        // Pilot not overriding - run state machine
        
        // Retry position - populated by state machine if RETRYING state
        Vector3f retry_pos;

        // Update state machine and get current status
        switch (copter.precland_statemachine.update(retry_pos)) {
            
        case AC_PrecLand_StateMachine::Status::RETRYING:
            // State machine wants to retry landing from different position
            // retry_pos contains the target position for retry attempt
            precland_retry_position(retry_pos);
            break;

        case AC_PrecLand_StateMachine::Status::FAILSAFE: {
            // Precision landing failed after retries - execute failsafe action
            // Query state machine for appropriate failsafe response
            switch (copter.precland_statemachine.get_failsafe_actions()) {
                
            case AC_PrecLand_StateMachine::FailSafeAction::DESCEND:
                // Land at current position - target definitely not visible
                // Executes normal descent without trying to track target
                land_run_horiz_and_vert_control();
                break;
                
            case AC_PrecLand_StateMachine::FailSafeAction::HOLD_POS:
                // Hold current position and altitude - wait for pilot intervention
                // pause_descent = true stops vertical descent
                land_run_horiz_and_vert_control(true);
                break;
            }
            break;
        }
        
        case AC_PrecLand_StateMachine::Status::ERROR:
            // State machine error - should never happen (indicates software bug)
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
            // Fall through to safe default action (descend)
            FALLTHROUGH;
            
        case AC_PrecLand_StateMachine::Status::DESCEND:
            // Normal operation - descend while tracking target
            // If target visible: tracks target horizontally while descending
            // If target not visible: descends vertically at current position
            land_run_horiz_and_vert_control();
            break;
        }
        
    } else {
        // Pilot has taken manual control (land_repo_active = true)
        // Bypass state machine and execute simple landing
        // Landing may or may not be on target depending on pilot positioning
        land_run_horiz_and_vert_control();
    }
}
#endif

/**
 * @brief Get the learned hover throttle value
 * 
 * @details Returns the current hover throttle estimate learned by the motors library
 * through flight observation. This value represents the throttle output (0.0-1.0)
 * required to maintain hover at current weight and conditions. Used for throttle
 * input shaping and as reference point for manual throttle modes.
 * 
 * Hover Throttle Learning:
 * - Automatically learned during flight
 * - Tracked by AP_Motors library
 * - Adapts to vehicle weight changes (battery consumption, payload)
 * - Adjusts for altitude and air density changes
 * - Typically ranges from 0.3 to 0.7 for most multirotors
 * 
 * Usage:
 * - Manual throttle mode input shaping (makes mid-stick = hover)
 * - Reference for altitude hold modes
 * - Feed-forward term in position controller
 * - Pilot workload reduction (mid-stick maintains altitude)
 * 
 * @return Hover throttle estimate in range 0.0-1.0 (0-100% throttle)
 * 
 * @note Value automatically learned and updated during flight
 * @note Provides smooth throttle response centered on hover point
 * 
 * @see get_pilot_desired_throttle() for manual throttle input transformation
 * @see AP_Motors::get_throttle_hover() for learning algorithm
 * 
 * Source: ArduCopter/mode.cpp:2287-2290
 */
float Mode::throttle_hover() const
{
    return motors->get_throttle_hover();
}

/**
 * @brief Transform pilot's manual throttle input with hover-centered expo curve
 * 
 * @details Converts raw pilot throttle stick input (0-1000) into output throttle (0.0-1.0)
 * with exponential curve shaping that places hover throttle at mid-stick. Makes manual
 * throttle modes more intuitive by centering response around hover point regardless of
 * vehicle weight or conditions. Used exclusively by manual throttle flight modes.
 * 
 * Throttle Input Transformation Algorithm:
 * 
 * Step 1: Normalize Raw Input (0-1000 → 0.0-1.0)
 * - Input below mid-stick: throttle_in = (input / mid_stick) * 0.5
 * - Input above mid-stick: throttle_in = 0.5 + ((input - mid_stick) / (1000 - mid_stick)) * 0.5
 * - This creates symmetric scaling around mid-stick position
 * - Mid-stick parameter defaults to 500 but can be adjusted
 * 
 * Step 2: Calculate Exponential Shaping Factor
 * - expo = -(hover_throttle - 0.5) / 0.375
 * - Constrained to range [-0.5, 1.0]
 * - If hover is high (heavy vehicle): negative expo (soften low throttle)
 * - If hover is low (light vehicle): positive expo (soften high throttle)
 * - Centers response curve around actual hover point
 * 
 * Step 3: Apply Exponential Curve
 * - throttle_out = throttle_in * (1 - expo) + expo * throttle_in³
 * - Linear term: throttle_in * (1 - expo)
 * - Cubic term: expo * throttle_in³
 * - Cubic provides progressive feel near extremes
 * - Result: mid-stick produces hover throttle output
 * 
 * Example Transformations:
 * - Heavy vehicle (hover = 0.7):
 *   * expo ≈ -0.53 (clamped to -0.5)
 *   * More linear response at low throttle (easier to fly)
 *   * More progressive near full throttle
 * - Light vehicle (hover = 0.3):
 *   * expo ≈ 0.53
 *   * More progressive at low throttle
 *   * More linear near full throttle
 * - Standard vehicle (hover = 0.5):
 *   * expo = 0
 *   * Pure linear response (throttle_out = throttle_in)
 * 
 * Safety Features:
 * - Mid-stick clamped to minimum 1 to prevent divide-by-zero
 * - Raw input constrained to valid range [0, 1000]
 * - Expo factor constrained to prevent excessive non-linearity
 * - Always produces valid output in range [0.0, 1.0]
 * 
 * Flight Mode Usage:
 * - Stabilize mode: Direct throttle control
 * - Acro mode: Manual throttle with this shaping
 * - Sport mode: Manual throttle with this shaping
 * - Drift mode: Manual throttle with this shaping
 * - NOT used in altitude-holding modes (Alt Hold, Loiter, etc.)
 * 
 * Benefits:
 * - Mid-stick maintains hover regardless of vehicle weight
 * - Reduces pilot workload in manual modes
 * - Smooth transition between climb and descent
 * - Adapts automatically to conditions (altitude, battery, payload)
 * - Progressive feel at extremes (gentle at center, responsive at ends)
 * 
 * @return Transformed throttle output in range 0.0-1.0 (0-100% throttle)
 * 
 * @note Only used by manual throttle flight modes (Stabilize, Acro, Sport, Drift)
 * @note Mid-stick position produces hover throttle output automatically
 * @note Expo curve adapts to learned hover throttle
 * @note Input stick range is 0-1000, output range is 0.0-1.0
 * 
 * @warning Do not use in altitude-holding modes (handled by position controller)
 * 
 * @see throttle_hover() for learned hover throttle value
 * @see Copter::get_throttle_mid() for mid-stick parameter
 * @see Mode::has_manual_throttle() for modes that use this function
 * 
 * Source: ArduCopter/mode.cpp:2296-2321
 */
float Mode::get_pilot_desired_throttle() const
{
    // Get configured mid-stick position (default 500, range 0-1000)
    int16_t mid_stick = copter.get_throttle_mid();
    
    // Protect against divide by zero (unlikely but safety critical)
    if (mid_stick <= 0) {
        mid_stick = 500;  // Default to 50% stick position
    }

    // Get raw pilot throttle input (0-1000 range)
    int16_t throttle_control = channel_throttle->get_control_in();
    
    // Ensure input is in valid range (constrain for safety)
    throttle_control = constrain_int16(throttle_control, 0, 1000);

    // Normalize throttle input to 0.0-1.0 range with mid-stick scaling
    float throttle_in;
    if (throttle_control < mid_stick) {
        // Below mid-stick: scale 0 to mid-stick → 0.0 to 0.5
        throttle_in = ((float)throttle_control) * 0.5f / (float)mid_stick;
    } else {
        // Above mid-stick: scale mid-stick to 1000 → 0.5 to 1.0
        throttle_in = 0.5f + ((float)(throttle_control - mid_stick)) * 0.5f / (float)(1000 - mid_stick);
    }

    // Get learned hover throttle (automatically adapts to vehicle weight/conditions)
    const float thr_mid = throttle_hover();
    
    // Calculate exponential shaping factor based on hover throttle
    // Formula: expo = -(hover - 0.5) / 0.375, constrained to [-0.5, 1.0]
    // This centers the response curve around actual hover point
    const float expo = constrain_float(-(thr_mid - 0.5f) / 0.375f, -0.5f, 1.0f);
    
    // Apply exponential curve transformation
    // Linear component + cubic component for progressive feel
    float throttle_out = throttle_in * (1.0f - expo) + expo * throttle_in * throttle_in * throttle_in;
    
    return throttle_out;
}

/**
 * @brief Adjust vertical climb rate for obstacle avoidance if enabled
 * 
 * @details Modifies the requested vertical climb rate to avoid obstacles detected above
 * or below the vehicle. When avoidance is enabled, this function passes the climb rate
 * through the avoidance system which may reduce it to prevent collision. When avoidance
 * is disabled, returns the input rate unchanged.
 * 
 * Avoidance System Integration:
 * - Calls AC_Avoid::adjust_velocity_z() when AP_AVOIDANCE_ENABLED
 * - Provides position controller gains for smooth avoidance response
 * - Passes vertical P gain (kP) for proportional response
 * - Passes max vertical acceleration for rate limiting
 * - Modifies target_rate_cms in-place (pass by reference)
 * - Returns modified rate for convenience
 * 
 * Obstacle Detection:
 * - Uses upward-facing rangefinders to detect ceiling/obstacles above
 * - Uses downward-facing rangefinders for ground/obstacles below
 * - Proximity sensors may contribute to obstacle detection
 * - Fence altitude limits may also constrain vertical motion
 * 
 * Rate Adjustment Behavior:
 * - Climbing toward obstacle above: Rate reduced or zeroed
 * - Descending toward obstacle below: Rate reduced or zeroed
 * - No obstacles detected: Rate passes through unchanged
 * - Smooth ramping applied to avoid abrupt changes
 * - Respects maximum acceleration limits
 * 
 * Conditional Compilation:
 * - Only active when AP_AVOIDANCE_ENABLED defined at compile time
 * - Feature can be disabled in build to save flash space
 * - When disabled, function is pass-through (no overhead)
 * 
 * Typical Usage Pattern:
 * ```cpp
 * float desired_climb_rate_cms = 250.0f;  // Want to climb at 2.5 m/s
 * desired_climb_rate_cms = get_avoidance_adjusted_climbrate_cms(desired_climb_rate_cms);
 * // Rate may now be reduced if obstacle detected above
 * pos_control->set_vel_desired_z_cms(desired_climb_rate_cms);
 * ```
 * 
 * @param[in,out] target_rate_cms Desired vertical climb rate in cm/s
 *                                 Positive = climb, Negative = descend
 *                                 Modified in-place by avoidance system
 * 
 * @return Adjusted vertical climb rate in cm/s (same as modified parameter)
 * 
 * @note Parameter is modified in-place and also returned for convenience
 * @note Only affects vertical velocity - horizontal avoidance handled separately
 * @note When avoidance disabled, function has no effect (pass-through)
 * 
 * @warning Avoidance can override pilot commands to prevent collisions
 * 
 * @see AC_Avoid::adjust_velocity_z() for avoidance algorithm
 * @see Mode::get_pilot_desired_climb_rate() for pilot input
 * 
 * Source: ArduCopter/mode.cpp:2323-2331
 */
float Mode::get_avoidance_adjusted_climbrate_cms(float target_rate_cms)
{
#if AP_AVOIDANCE_ENABLED
    // Adjust climb rate through avoidance system
    // Parameters:
    // - kP: Vertical position controller P gain for smooth response
    // - max_accel: Maximum vertical acceleration for rate limiting
    // - target_rate_cms: Desired rate (modified in-place)
    // - G_Dt: Loop time for acceleration calculations
    AP::ac_avoid()->adjust_velocity_z(pos_control->get_pos_U_p().kP(), 
                                      pos_control->get_max_accel_U_cmss(), 
                                      target_rate_cms, 
                                      G_Dt);
    return target_rate_cms;
#else
    // Avoidance not compiled in - return rate unchanged
    return target_rate_cms;
#endif
}

/**
 * @brief Send motor control outputs to ESCs/servos
 * 
 * @details Calls the motors library to output PWM/CAN signals to motor controllers.
 * This is the final step in the control loop that translates attitude controller
 * commands into physical motor outputs. Can be overridden by derived mode classes
 * to implement custom motor output behavior.
 * 
 * Output Generation:
 * - Translates attitude controller thrust vector into individual motor commands
 * - Applies motor mixing for multirotor frame type
 * - Generates PWM signals for ESCs (typically 1000-2000 μs)
 * - Or generates CAN messages for CAN-based ESCs (DroneCAN)
 * - Handles thrust curve linearization
 * - Applies motor output limits and constraints
 * 
 * Motor Library Responsibilities:
 * - Frame-specific motor mixing (quad, hex, octo, Y6, etc.)
 * - Individual motor thrust curves
 * - Motor output limiting (prevent saturation)
 * - ESC protocol handling (PWM, OneShot, DShot, CAN)
 * - Throttle range mapping
 * - Motor spin-when-armed behavior
 * 
 * Control Flow Integration:
 * 1. Attitude controller generates thrust vector
 * 2. Motors library converts to individual motor thrusts
 * 3. This function triggers actual hardware output
 * 4. ESCs receive commands and control motors
 * 
 * Execution Frequency:
 * - Called at main loop rate (typically 400 Hz)
 * - Some ESC protocols support higher update rates
 * - Output rate matches attitude controller rate
 * 
 * Override Capability:
 * - Virtual function can be overridden by mode subclasses
 * - Allows modes to implement special motor output behavior
 * - Example: Turtle mode overrides for inverted recovery
 * - Most modes use this default implementation
 * 
 * Safety Features:
 * - Motors library enforces safety interlocks
 * - Respects arming state (no output when disarmed)
 * - Applies throttle limits from parameters
 * - Handles motor test and ESC calibration modes
 * 
 * @note Called every main loop iteration (~400 Hz)
 * @note Default implementation calls motors->output() directly
 * @note Virtual function - can be overridden by mode subclasses
 * @note Final step in control loop before hardware output
 * 
 * @see AP_Motors::output() for motor mixing and output generation
 * @see Copter::fast_loop() for main loop integration
 * @see Mode::run() for where this is typically called
 * 
 * Source: ArduCopter/mode.cpp:2334-2337
 */
void Mode::output_to_motors()
{
    motors->output();
}

/**
 * @brief Determine current altitude hold mode state machine state
 * 
 * @details Analyzes vehicle state (armed, landed, flying, taking off) and pilot inputs
 * to determine the appropriate altitude hold behavior state. Manages motor spool states
 * and coordinates transitions between landed, takeoff, and flying states. Used by
 * altitude-holding flight modes to implement appropriate control behavior for each state.
 * 
 * State Machine States:
 * 
 * 1. MotorStopped:
 *    - Motors completely stopped (disarmed state)
 *    - Spool state: SHUT_DOWN
 *    - No motor output
 *    - Waiting for arming
 * 
 * 2. Landed_Ground_Idle:
 *    - Armed but motors at minimum idle
 *    - Spool state: GROUND_IDLE
 *    - Motors spinning at low speed (safety, gyro cooling)
 *    - Either spooling down from disarm OR armed and waiting
 *    - Pilot requesting descent or using motor interlock
 * 
 * 3. Landed_Pre_Takeoff:
 *    - Armed and ready for immediate takeoff
 *    - Spool state: SPOOLING_UP or THROTTLE_UNLIMITED
 *    - Motors spooled up, can leave ground instantly
 *    - Waiting for positive climb rate command
 *    - Default state when armed on ground
 * 
 * 4. Takeoff:
 *    - Actively executing takeoff procedure
 *    - Either takeoff.running() or just triggered
 *    - Triggers when: land_complete + positive climb rate + motors ready
 *    - Special takeoff handling (thrust ramping, surface detection)
 *    - Transitions to Flying when clear of ground
 * 
 * 5. Flying:
 *    - Vehicle airborne and auto_armed
 *    - Normal flight control active
 *    - Full altitude hold control enabled
 *    - Spool state: THROTTLE_UNLIMITED
 * 
 * State Determination Logic:
 * 
 * Priority 1: Check if disarmed
 * - If !motors->armed(): Command SHUT_DOWN spool state
 * - Return state based on current spool state (MotorStopped, Landed_Ground_Idle, or Landed_Pre_Takeoff)
 * 
 * Priority 2: Check for active takeoff
 * - If takeoff.running() OR takeoff.triggered(): Return Takeoff
 * - takeoff.triggered() checks: land_complete + positive climb rate + motors spooled up
 * 
 * Priority 3: Check if landed while armed
 * - If !auto_armed OR land_complete: Armed but on ground
 * - If pilot wants descent AND not using interlock: Command GROUND_IDLE
 * - Otherwise: Command THROTTLE_UNLIMITED (ready for takeoff)
 * - Return Landed_Ground_Idle or Landed_Pre_Takeoff based on spool state
 * 
 * Priority 4: Default to flying
 * - All other cases: Return Flying
 * - Command THROTTLE_UNLIMITED spool state
 * 
 * Motor Spool State Management:
 * - Function actively commands desired spool states
 * - Motors library manages actual spool state transitions
 * - Spool state affects motor output and takeoff detection
 * - Smooth transitions prevent abrupt motor changes
 * 
 * Auto-Armed Flag:
 * - Set when vehicle takes off (throttle applied while armed)
 * - Cleared on landing
 * - Prevents accidental ground movement
 * - Key indicator of flight vs ground state
 * 
 * Land Complete Detection:
 * - Set when vehicle is confirmed on ground
 * - Uses multiple sensors (rangefinder, baro, IMU)
 * - Cleared during takeoff sequence
 * - Critical for state transitions
 * 
 * Motor Interlock:
 * - Helicopter feature for rotor control
 * - When using_interlock = false: Rotor disengaged
 * - Allows spool to ground idle even with pilot input
 * - Not used on standard multirotors
 * 
 * Usage by Flight Modes:
 * - Alt Hold: Primary user of this state machine
 * - Loiter: Uses for vertical control state
 * - PosHold: Uses for vertical control state
 * - Sport: Uses for altitude hold component
 * - Other altitude-holding modes: Similar usage pattern
 * 
 * Typical State Sequence:
 * 1. Arm on ground: MotorStopped → Landed_Pre_Takeoff
 * 2. Apply throttle: Landed_Pre_Takeoff → Takeoff
 * 3. Clear ground: Takeoff → Flying
 * 4. Land: Flying → Landed_Pre_Takeoff → Landed_Ground_Idle
 * 5. Disarm: Landed_Ground_Idle → MotorStopped
 * 
 * @param[in] target_climb_rate_cms Desired vertical climb rate in cm/s
 *                                   Used to detect takeoff trigger (positive rate)
 *                                   Used to determine ground idle vs ready state
 * 
 * @return Current altitude hold state for this iteration
 * 
 * @note Called every iteration by altitude-holding flight modes
 * @note Function actively commands motor spool states (has side effects)
 * @note State affects control behavior (different logic per state)
 * @note Smooth state transitions critical for stable flight
 * 
 * @warning State machine must be called every loop iteration
 * @warning Improper state handling can cause takeoff or landing issues
 * 
 * @see Mode::takeoff for takeoff detection and management
 * @see AP_Motors::set_desired_spool_state() for spool state control
 * @see Copter::ap.auto_armed for flight state detection
 * @see Copter::ap.land_complete for ground detection
 * 
 * Source: ArduCopter/mode.cpp:2339-2389
 */
Mode::AltHoldModeState Mode::get_alt_hold_state(float target_climb_rate_cms)
{
    // Alt Hold State Machine Determination
    
    // Priority 1: Check if vehicle is disarmed
    if (!motors->armed()) {
        // Vehicle should shut down - command motors to spool down
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);

        // Return state based on current spool state (transition smoothly)
        switch (motors->get_spool_state()) {

        case AP_Motors::SpoolState::SHUT_DOWN:
            // Motors completely stopped
            return AltHoldModeState::MotorStopped;

        case AP_Motors::SpoolState::GROUND_IDLE:
            // Still spinning down to ground idle
            return AltHoldModeState::Landed_Ground_Idle;

        default:
            // Still spooling down (other transient states)
            return AltHoldModeState::Landed_Pre_Takeoff;
        }

    // Priority 2: Check for active takeoff
    } else if (takeoff.running() || takeoff.triggered(target_climb_rate_cms)) {
        // Vehicle is on ground or taking off
        // Conditions: positive climb rate + land_complete + motors ready
        // Progress through takeoff procedure
        return AltHoldModeState::Takeoff;

    // Priority 3: Check if armed but landed
    } else if (!copter.ap.auto_armed || copter.ap.land_complete) {
        // Vehicle is armed and on ground (but not taking off)
        
        if (target_climb_rate_cms < 0.0f && !copter.ap.using_interlock) {
            // Pilot requesting descent OR interlock disengaged (heli)
            // Spool down to ground idle (save power, reduce noise)
            motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);

        } else {
            // Pilot not descending or interlock engaged
            // Keep motors spooled up, ready for immediate takeoff
            motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
        }

        // Return state based on actual spool state
        if (motors->get_spool_state() == AP_Motors::SpoolState::GROUND_IDLE) {
            // Motors at idle - not ready for immediate takeoff
            return AltHoldModeState::Landed_Ground_Idle;

        } else {
            // Motors spooled up - can takeoff immediately
            return AltHoldModeState::Landed_Pre_Takeoff;
        }

    // Priority 4: Default to flying state
    } else {
        // Vehicle is airborne (auto_armed = true, land_complete = false)
        // Normal flight - full throttle authority
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
        return AltHoldModeState::Flying;
    }
}

/**
 * @brief Transform pilot's yaw stick input into desired yaw rate
 * 
 * @details Converts normalized pilot yaw stick input (-1.0 to +1.0) into a desired yaw
 * rate command in radians per second. Applies deadzone, exponential curve, and rate
 * scaling according to configured parameters. Returns zero if RC input is invalid
 * (failsafe condition). Used by flight modes that allow pilot yaw control.
 * 
 * Input Processing Pipeline:
 * 1. Check RC failsafe - return 0 if no valid input
 * 2. Get normalized yaw input from channel_yaw (-1.0 to +1.0)
 * 3. Apply deadzone (already in norm_input_dz())
 * 4. Apply exponential curve for fine control near center
 * 5. Scale by maximum yaw rate parameter
 * 6. Convert degrees/sec to radians/sec
 * 
 * Deadzone Handling:
 * - channel_yaw->norm_input_dz() applies configured deadzone
 * - Typical deadzone: 20 (out of 500 range = 4%)
 * - Prevents drift from noisy or imperfect centering
 * - Stick must move beyond deadzone before output changes
 * 
 * Exponential Curve (Expo):
 * - Controlled by g2.command_model_pilot_y.get_expo()
 * - Typical range: 0.0 (linear) to 0.5 (exponential)
 * - Higher expo = finer control near center, more aggressive at extremes
 * - Applied via input_expo() function
 * - Formula: output = input * (1-expo) + input^3 * expo
 * - Allows precise yaw control in stabilized flight
 * 
 * Rate Scaling:
 * - Maximum rate from g2.command_model_pilot_y.get_rate()
 * - Typical value: 90 deg/s for smooth control
 * - Acro mode may use higher rates (200-400 deg/s)
 * - Rate determines maximum yaw rotation speed
 * - Full stick deflection = maximum rate
 * 
 * Command Model System:
 * - g2.command_model_pilot_y encapsulates yaw input shaping
 * - Provides get_rate() for maximum rate
 * - Provides get_expo() for exponential curve
 * - Provides get_rate_tc() for rate time constant (used elsewhere)
 * - Consistent input shaping across all modes
 * 
 * RC Failsafe Handling:
 * - Returns 0.0 when rc().has_valid_input() = false
 * - Prevents uncommanded yaw during RC loss
 * - Mode-specific failsafe actions handle overall behavior
 * - Zero yaw rate maintains current heading
 * 
 * Unit Conversion:
 * - Input: Normalized stick position (-1.0 to +1.0, unitless)
 * - Parameter: Maximum rate in degrees per second
 * - Output: Desired rate in radians per second
 * - Conversion: deg/s * PI/180 = rad/s
 * 
 * Yaw Rate Convention:
 * - Positive rate = clockwise rotation (from above, NED frame)
 * - Negative rate = counter-clockwise rotation
 * - Matches body frame yaw convention
 * - Right stick = positive rate (clockwise)
 * - Left stick = negative rate (counter-clockwise)
 * 
 * Mode Usage:
 * - Stabilize: Direct yaw rate control
 * - Alt Hold: Yaw rate control, holds altitude
 * - Loiter: Yaw rate control, holds position
 * - Guided: Yaw rate override during guided flight
 * - Auto: Yaw rate during waypoint navigation (if enabled)
 * - Not used in heading-hold modes (AUTO with DO_SET_ROI)
 * 
 * Time Constant Relationship:
 * - This function provides instantaneous rate command
 * - Rate time constant (get_rate_tc()) affects acceleration to rate
 * - TC applied in attitude controller, not here
 * - Higher TC = slower rate changes, smoother feel
 * - Lower TC = faster rate changes, more responsive
 * 
 * Typical Call Pattern:
 * ```cpp
 * // In mode run() function
 * float target_yaw_rate_rads = get_pilot_desired_yaw_rate_rads();
 * attitude_control->input_rate_bf_roll_pitch_yaw_rads(roll_rate, pitch_rate, target_yaw_rate_rads);
 * ```
 * 
 * @return Desired yaw rate in radians per second
 *         Positive = clockwise, Negative = counter-clockwise
 *         Zero if RC failsafe active or stick centered in deadzone
 * 
 * @note Returns zero during RC failsafe (maintains current heading)
 * @note Output already includes deadzone, expo, and rate scaling
 * @note Called at main loop rate (~400 Hz) by most flight modes
 * @note Unit conversion: function comment says "centi-degrees" but actually returns radians/s
 * 
 * @see channel_yaw->norm_input_dz() for input with deadzone applied
 * @see input_expo() for exponential curve application
 * @see g2.command_model_pilot_y for yaw command shaping parameters
 * @see AC_AttitudeControl::input_rate_bf_roll_pitch_yaw_rads() for rate command usage
 * 
 * Source: ArduCopter/mode.cpp:2391-2404
 */
float Mode::get_pilot_desired_yaw_rate_rads() const
{
    // Check for RC failsafe condition
    if (!rc().has_valid_input()) {
        // No valid RC input - return zero rate to maintain current heading
        return 0.0f;
    }

    // Get normalized yaw input from RC channel (-1.0 to +1.0)
    // norm_input_dz() applies configured deadzone
    const float yaw_in = channel_yaw->norm_input_dz();

    // Apply exponential curve and scale by maximum rate
    // input_expo() applies: output = input * (1-expo) + input^3 * expo
    // Then scale by maximum rate parameter (deg/s)
    // Finally convert degrees/sec to radians/sec
    return radians(g2.command_model_pilot_y.get_rate()) * input_expo(yaw_in, g2.command_model_pilot_y.get_expo());
}

/**
 * @brief Get pilot's desired climb rate from throttle stick input
 * 
 * @details Pass-through wrapper to Copter::get_pilot_desired_climb_rate(). Translates
 * pilot's throttle stick position into a desired vertical climb rate in cm/s. Applies
 * deadzone, expo curve, and scales by maximum climb/descend rates from parameters.
 * Automatically adjusts for hover throttle position.
 * 
 * Wrapper Function Note:
 * - This is a convenience wrapper to access copter singleton method
 * - Reduces code churn during mode class refactoring
 * - Candidate for moving implementation into Mode base class
 * - Currently just forwards to main Copter class
 * 
 * Functionality Provided:
 * - Reads channel_throttle RC input
 * - Centers around hover throttle (mid-stick = hold altitude)
 * - Applies deadzone to prevent drift
 * - Applies exponential curve for fine control
 * - Scales to maximum climb/descend rates
 * - Returns climb rate in cm/s (positive = up, negative = down)
 * 
 * @return Desired vertical climb rate in cm/s
 *         Positive = climb, Negative = descend, Zero = hold altitude
 * 
 * @note Pass-through wrapper to copter.get_pilot_desired_climb_rate()
 * @note Used by altitude-controlling flight modes
 * 
 * @see Copter::get_pilot_desired_climb_rate() for full implementation
 * @see Mode::get_pilot_desired_throttle() for manual throttle modes
 * 
 * Source: ArduCopter/mode.cpp:2407-2410
 */
float Mode::get_pilot_desired_climb_rate()
{
    return copter.get_pilot_desired_climb_rate();
}

/**
 * @brief Get throttle value that won't trigger auto-takeoff
 * 
 * @details Pass-through wrapper to Copter::get_non_takeoff_throttle(). Returns the
 * maximum throttle value that can be applied while on the ground without triggering
 * automatic takeoff. Used during mode transitions to prevent unexpected liftoff when
 * switching from manual to automatic throttle modes.
 * 
 * Wrapper Function Note:
 * - Convenience wrapper to access copter singleton method
 * - Part of mode class refactoring effort
 * - May be moved into Mode base class in future
 * 
 * Usage Context:
 * - Called during mode change validation in set_mode()
 * - Prevents mode switch if throttle > non-takeoff threshold
 * - Avoids vehicle jumping when switching to auto-throttle mode
 * - Example: User in GUIDED with high stick, switches to STABILIZE
 * 
 * Safety Purpose:
 * - Prevents uncommanded takeoffs during mode transitions
 * - Ensures smooth handover between manual and auto throttle
 * - Protects against surprising vehicle motion
 * 
 * @return Maximum safe throttle value (0.0 to 1.0) that won't cause takeoff
 * 
 * @note Pass-through wrapper to copter.get_non_takeoff_throttle()
 * @note Used in set_mode() safety checks
 * 
 * @see Copter::get_non_takeoff_throttle() for threshold calculation
 * @see Copter::set_mode() for usage in mode transition checks
 * 
 * Source: ArduCopter/mode.cpp:2412-2415
 */
float Mode::get_non_takeoff_throttle()
{
    return copter.get_non_takeoff_throttle();
}

/**
 * @brief Update SIMPLE mode coordinate frame transformation
 * 
 * @details Pass-through wrapper to Copter::update_simple_mode(). Updates the coordinate
 * frame transformation used by SIMPLE and SUPER_SIMPLE flight modes. SIMPLE mode allows
 * the pilot to control the vehicle relative to the direction it was armed, rather than
 * relative to the vehicle's current heading. Must be called each loop iteration.
 * 
 * Wrapper Function Note:
 * - Convenience wrapper to access copter singleton method
 * - Simplifies mode class implementation
 * - Part of ongoing refactoring effort
 * 
 * SIMPLE Mode Functionality:
 * - Transforms pilot inputs from simple frame to body frame
 * - Simple frame: Forward = direction vehicle faced at arming
 * - Makes vehicle easier to fly for beginners
 * - Pilot doesn't need to account for vehicle yaw
 * 
 * SUPER_SIMPLE Mode:
 * - Forward = direction from home to vehicle
 * - Allows flying back to home easily
 * - Forward stick always brings vehicle toward pilot
 * - Direction updates as vehicle moves
 * 
 * Update Frequency:
 * - Must be called every main loop iteration
 * - Updates transformation matrix
 * - Accounts for vehicle position changes (SUPER_SIMPLE)
 * - No effect if SIMPLE modes disabled
 * 
 * @note Pass-through wrapper to copter.update_simple_mode()
 * @note Only affects input when SIMPLE or SUPER_SIMPLE enabled
 * @note Must be called each loop for SUPER_SIMPLE to track position
 * 
 * @see Copter::update_simple_mode() for transformation logic
 * @see Mode::get_pilot_desired_lean_angles_rad() for input transformation usage
 * 
 * Source: ArduCopter/mode.cpp:2417-2419
 */
void Mode::update_simple_mode(void) {
    copter.update_simple_mode();
}

/**
 * @brief Request flight mode change
 * 
 * @details Pass-through wrapper to Copter::set_mode(). Requests a change to a new
 * flight mode with specified reason. Performs safety checks, validates prerequisites,
 * calls mode init, and handles cleanup. Returns success/failure status.
 * 
 * Wrapper Function Note:
 * - Allows modes to request mode changes
 * - Forwards to main copter set_mode implementation
 * - Provides access to mode switching from Mode class context
 * 
 * Common Usage:
 * - RTL mode on battery failsafe
 * - LAND mode on GPS failure
 * - LOITER/ALT_HOLD on high throttle during LAND
 * - Mode escalation during failsafe conditions
 * 
 * @param[in] mode New flight mode to switch to
 * @param[in] reason Reason for mode change (logged and reported)
 * 
 * @return true if mode change successful, false if failed validation or init
 * 
 * @note Pass-through wrapper to copter.set_mode()
 * @note Performs full safety checks and mode initialization
 * 
 * @see Copter::set_mode() for complete mode change logic
 * @see ModeReason enumeration for available reason codes
 * 
 * Source: ArduCopter/mode.cpp:2421-2424
 */
bool Mode::set_mode(Mode::Number mode, ModeReason reason)
{
    return copter.set_mode(mode, reason);
}

/**
 * @brief Set land complete flag
 * 
 * @details Pass-through wrapper to Copter::set_land_complete(). Sets or clears the
 * land_complete flag which indicates the vehicle has detected ground contact and
 * completed landing. Affects motor output, auto-disarm timing, and mode behavior.
 * 
 * Wrapper Function Note:
 * - Allows modes to signal landing completion
 * - Forwards to copter singleton method
 * - Part of mode class interface
 * 
 * Land Complete Flag Effects:
 * - Triggers auto-disarm timer if configured
 * - Affects motor spool state decisions
 * - Changes altitude hold state machine behavior
 * - Prevents immediate re-takeoff without throttle down
 * - Logged for post-flight analysis
 * 
 * Detection Criteria:
 * - Low throttle output sustained
 * - Minimal vertical velocity
 * - Rangefinder shows ground contact (if available)
 * - Passed sufficient time in landing descent
 * 
 * @param[in] b true to set land complete, false to clear
 * 
 * @note Pass-through wrapper to copter.set_land_complete()
 * @note Typically called by LAND mode when touchdown detected
 * @note Cleared during takeoff sequence
 * 
 * @see Copter::set_land_complete() for flag management
 * @see Copter::ap.land_complete for flag state
 * 
 * Source: ArduCopter/mode.cpp:2426-2429
 */
void Mode::set_land_complete(bool b)
{
    return copter.set_land_complete(b);
}

/**
 * @brief Get reference to Ground Control Station interface
 * 
 * @details Pass-through wrapper to Copter::gcs(). Returns reference to GCS_Copter
 * object which handles MAVLink communication with ground control stations. Used for
 * sending telemetry, status messages, and handling GCS commands.
 * 
 * Wrapper Function Note:
 * - Provides access to GCS from Mode class context
 * - Forwards to copter singleton method
 * - Simplifies mode implementation
 * 
 * GCS Interface Usage:
 * - Send status text messages (errors, warnings, info)
 * - Stream telemetry data
 * - Handle MAVLink commands
 * - Report mode changes and events
 * - Send mission status updates
 * 
 * Common Operations:
 * - gcs().send_text() - Send text message to GCS
 * - gcs().send_message() - Send specific MAVLink message
 * - Used extensively for user feedback and debugging
 * 
 * @return Reference to GCS_Copter interface object
 * 
 * @note Pass-through wrapper to copter.gcs()
 * @note Returns reference, not pointer (always valid)
 * 
 * @see GCS_Copter for MAVLink interface implementation
 * @see Copter::gcs() for accessor method
 * 
 * Source: ArduCopter/mode.cpp:2431-2434
 */
GCS_Copter &Mode::gcs()
{
    return copter.gcs();
}

/**
 * @brief Get pilot's desired descent speed from parameter
 * 
 * @details Pass-through wrapper to Copter::get_pilot_speed_dn(). Returns the configured
 * maximum descent speed for pilot-controlled descent. Used by modes that allow pilot
 * to control descent rate (LAND with repositioning, altitude-holding modes with pilot
 * input, etc.).
 * 
 * Wrapper Function Note:
 * - Convenience wrapper to access copter parameter method
 * - Part of mode class interface
 * - May be refactored in future
 * 
 * Parameter Source:
 * - Reads from PILOT_SPEED_DN or similar parameter
 * - Typical value: 150 cm/s (1.5 m/s)
 * - Limits maximum descent rate with stick input
 * - Separate from automatic descent rates
 * 
 * Usage Context:
 * - Landing with repositioning enabled
 * - Manual descent in altitude hold modes
 * - Rate limiting for safety
 * - Different from automatic landing speed (g.land_speed)
 * 
 * @return Pilot-controlled maximum descent speed in cm/s (positive value)
 * 
 * @note Pass-through wrapper to copter.get_pilot_speed_dn()
 * @note Returns positive value even though descent is negative velocity
 * @note Used for rate limiting pilot descent commands
 * 
 * @see Copter::get_pilot_speed_dn() for parameter access
 * @see PILOT_SPEED_DN parameter documentation
 * 
 * Source: ArduCopter/mode.cpp:2436-2439
 */
uint16_t Mode::get_pilot_speed_dn()
{
    return copter.get_pilot_speed_dn();
}

/**
 * @brief Calculate predicted stopping point with current velocity
 * 
 * @details Computes the 3D location where the vehicle will stop if all current velocity
 * is decelerated at the configured maximum deceleration rates. Used for predictive
 * trajectory visualization, obstacle avoidance look-ahead, and flight planning. Returns
 * stopping point as a Location with ABOVE_ORIGIN altitude frame.
 * 
 * Stopping Point Calculation:
 * - Uses current position from position controller
 * - Uses current velocity from position controller
 * - Applies maximum deceleration rates from parameters
 * - Integrates to find stopping distance
 * - Returns final position after complete stop
 * 
 * Position Controller Integration:
 * - get_stopping_point_NE_cm(): Horizontal (North-East) stopping point
 * - get_stopping_point_U_cm(): Vertical (Up) stopping point
 * - Each axis calculated independently
 * - Respects configured deceleration limits
 * 
 * Coordinate Frame:
 * - Internal calculation in NEU (North-East-Up) frame in cm
 * - NEU: Standard NED frame with Z inverted (Up positive)
 * - Returned as Location object for standard ArduPilot usage
 * - Altitude frame: ABOVE_ORIGIN (relative to EKF origin)
 * 
 * Horizontal Stopping Point:
 * - Accounts for North and East velocity components
 * - Applies maximum horizontal deceleration (WPNAV_ACCEL)
 * - Formula: stopping_distance = velocity^2 / (2 * deceleration)
 * - Smooth deceleration profile assumed
 * - Attitude limits may affect actual deceleration
 * 
 * Vertical Stopping Point:
 * - Accounts for vertical velocity (climb/descent)
 * - Applies maximum vertical deceleration
 * - Separate acceleration limits for up/down
 * - Important for altitude awareness and terrain avoidance
 * 
 * Usage Applications:
 * 
 * 1. Trajectory Visualization:
 *    - GCS displays predicted stopping point
 *    - Helps pilot understand vehicle momentum
 *    - Shows where vehicle will stop if sticks centered
 * 
 * 2. Obstacle Avoidance:
 *    - Check if stopping point intersects obstacles
 *    - Proactive avoidance of fences/boundaries
 *    - Warn pilot if cannot stop before fence
 * 
 * 3. Waypoint Navigation:
 *    - Determine if vehicle can stop before waypoint
 *    - Plan deceleration timing for precise positioning
 *    - Optimize approach trajectories
 * 
 * 4. Failsafe Planning:
 *    - Predict where vehicle will stop during failsafe
 *    - Ensure safe stopping within flight boundaries
 *    - Used in advanced fence actions
 * 
 * Limitations:
 * - Assumes constant maximum deceleration
 * - Doesn't account for wind (uses inertial velocity)
 * - Doesn't model attitude controller response time
 * - Doesn't account for motor thrust limits
 * - Ideal calculation, actual stopping may differ
 * 
 * Accuracy Factors:
 * - More accurate at low speeds (less integration error)
 * - Wind can affect actual stopping point significantly
 * - Aggressive maneuvers may exceed assumed deceleration
 * - Terrain and obstacles not considered in calculation
 * - Use as planning tool, not guaranteed stopping position
 * 
 * Return Value Details:
 * - Location object with lat, lon, alt
 * - Altitude relative to EKF origin (ABOVE_ORIGIN frame)
 * - Can be converted to other altitude frames as needed
 * - Includes full position (not just horizontal)
 * 
 * Performance Considerations:
 * - Lightweight calculation (analytical, not iterative)
 * - Can be called frequently without performance impact
 * - Position controller caches intermediate results
 * - No dynamic memory allocation
 * 
 * Typical Usage Pattern:
 * ```cpp
 * // Check if stopping point violates fence
 * Location stopping_loc = get_stopping_point();
 * if (fence.check_destination_within_fence(stopping_loc)) {
 *     // Safe to continue current trajectory
 * } else {
 *     // Need to slow down or change direction
 * }
 * ```
 * 
 * @return Location of predicted stopping point
 *         Altitude frame: ABOVE_ORIGIN (relative to EKF origin)
 *         Accounts for current velocity and maximum deceleration rates
 * 
 * @note Prediction assumes ideal deceleration at maximum rates
 * @note Wind, attitude limits, and thrust limits not modeled
 * @note More accurate for small velocities
 * @note Used for planning, not guaranteed actual stopping point
 * @note Returns Location suitable for fence checks and waypoint planning
 * 
 * @see AC_PosControl::get_stopping_point_NE_cm() for horizontal calculation
 * @see AC_PosControl::get_stopping_point_U_cm() for vertical calculation
 * @see Location class for coordinate representation
 * @see AC_Fence::check_destination_within_fence() for usage example
 * 
 * Source: ArduCopter/mode.cpp:2442-2448
 */
Location Mode::get_stopping_point() const
{
    // Create vector to hold stopping point in NEU frame (cm)
    Vector3p stopping_point_NEU;
    
    // Get horizontal stopping point (North-East components)
    // Calculates where vehicle will stop given current horizontal velocity
    copter.pos_control->get_stopping_point_NE_cm(stopping_point_NEU.xy());
    
    // Get vertical stopping point (Up component)
    // Calculates where vehicle will stop given current vertical velocity
    copter.pos_control->get_stopping_point_U_cm(stopping_point_NEU.z);
    
    // Convert NEU vector (cm) to Location object
    // tofloat() converts from cm to meters
    // ABOVE_ORIGIN altitude frame: relative to EKF origin
    return Location { stopping_point_NEU.tofloat(), Location::AltFrame::ABOVE_ORIGIN };
}
