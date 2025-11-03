/**
 * @file fence.cpp
 * @brief Geofence breach detection and enforcement for ArduPlane
 * 
 * @details This file implements integration between the AC_Fence library and ArduPlane,
 *          providing comprehensive geofence breach detection and automated response actions.
 *          
 *          Supported fence types:
 *          - Maximum altitude fence (prevents excessive climb)
 *          - Minimum altitude fence (prevents terrain impact if configured)
 *          - Horizontal circular fence (cylinder around home/rally point)
 *          - Horizontal polygon fence (complex inclusion/exclusion zones)
 *          
 *          Fence breach actions:
 *          - REPORT_ONLY: Log breach but continue current mission
 *          - RTL_AND_LAND: Return to launch and execute landing
 *          - AUTOLAND_OR_RTL: Attempt autoland if available, otherwise RTL
 *          - GUIDED: Fly to fence return point in guided mode
 *          - GUIDED_THROTTLE_PASS: Guided mode with manual throttle control
 *          
 *          The fence system operates asynchronously at 3Hz for breach detection,
 *          with the main loop handling breach responses and mode transitions.
 * 
 * @warning This is a critical safety system designed to prevent flyaways and
 *          contain the aircraft within designated operational boundaries.
 *          Fence breach handling must be thoroughly tested in SITL before
 *          flight operations.
 * 
 * @note Fence checks are disabled during the final landing stage to prevent
 *       inadvertent breach triggers during flare and touchdown.
 * 
 * @see AC_Fence library for core fence logic
 * @see ArduPlane/Plane.h for vehicle state management
 */

#include "Plane.h"

// Code to integrate AC_Fence library with main ArduPlane code

#if AP_FENCE_ENABLED

/**
 * @brief Asynchronous fence breach detection callback
 * 
 * @details This function performs geofence breach checking at 3Hz (every 333ms)
 *          in an asynchronous IO callback running at 1kHz. It checks all enabled
 *          fence types and latches breach status for the main loop to process.
 *          
 *          Fence breach detection respects the landing state - breaches are not
 *          triggered when the aircraft is landing or landed to prevent false
 *          positives during legitimate approach and touchdown operations.
 *          
 *          The function uses a double-buffering approach: breaches are detected
 *          here and latched, then consumed by fence_check() in the main loop,
 *          preventing race conditions between detection and response.
 * 
 * @note Called from IO thread at 1kHz, but performs checks at 3Hz to reduce
 *       computational load while maintaining adequate fence monitoring rate.
 * 
 * @note Will skip checks if previous breach status has not yet been processed
 *       by the main loop (fence_breaches.have_updates == true).
 * 
 * @warning Do not call fence response functions from this callback - breach
 *          handling must occur in the main loop via fence_check().
 * 
 * @see fence_check() for breach response and action triggering
 * @see AC_Fence::check() for core fence breach detection algorithm
 */
void Plane::fence_checks_async()
{
    const uint32_t now = AP_HAL::millis();

    if (!AP_HAL::timeout_expired(fence_breaches.last_check_ms, now, 333U)) { // 3Hz update rate
        return;
    }

    if (fence_breaches.have_updates) {
        return; // wait for the main loop to pick up the new breaches before checking again
    }

    fence_breaches.last_check_ms = now;
    orig_breaches = fence.get_breaches();
    const bool armed = arming.is_armed();

    uint16_t mission_id = plane.mission.get_current_nav_cmd().id;
    bool landing_or_landed = plane.flight_stage == AP_FixedWing::FlightStage::LAND
                         || !armed
#if HAL_QUADPLANE_ENABLED
                         || control_mode->mode_number() == Mode::Number::QLAND
                         || quadplane.in_vtol_land_descent()
#endif
                         || (plane.is_land_command(mission_id) && plane.mission.state() == AP_Mission::MISSION_RUNNING);

    // check for new breaches; new_breaches is bitmask of fence types breached
    fence_breaches.new_breaches = fence.check(landing_or_landed);
    fence_breaches.have_updates = true; // new breach status latched so main loop will now pick it up
}

/**
 * @brief Check for geofence violations and trigger appropriate response actions
 * 
 * @details This function processes fence breach status latched by fence_checks_async()
 *          and initiates the configured breach response action. It implements the
 *          complete fence breach state machine including:
 *          
 *          - Breach detection and action triggering
 *          - Mode transition management (RTL, GUIDED, AUTOLAND)
 *          - Fence recovery state tracking
 *          - Re-enablement after breach resolution
 *          - Landing sequence special handling
 *          
 *          Monitored fence types:
 *          - Maximum altitude fence: Prevents excessive climb above configured ceiling
 *          - Minimum altitude fence: Prevents terrain impact if min altitude configured
 *          - Horizontal circular fence: Cylindrical boundary around home/rally point
 *          - Horizontal polygon fence: Complex inclusion/exclusion zones
 *          
 *          Supported breach actions:
 *          - REPORT_ONLY: Log breach and send telemetry alert, no mode change
 *          - RTL_AND_LAND: Switch to RTL mode and execute landing sequence
 *          - AUTOLAND_OR_RTL: Attempt MODE_AUTOLAND if available, fallback to RTL
 *          - GUIDED: Switch to GUIDED mode and fly to fence return point
 *          - GUIDED_THROTTLE_PASS: GUIDED mode with pilot throttle control enabled
 *          
 *          Fence return point selection for GUIDED actions:
 *          - If rally points configured: Calculate best rally point or home
 *          - Otherwise: Use polygon fence return point if available, else home
 *          - Altitude: Use fence return altitude, or midpoint between min/max safe altitudes
 * 
 * @note This function is called from the main loop at the scheduler's main rate
 *       (typically 50-400Hz depending on vehicle configuration).
 * 
 * @note Fence checks are skipped when:
 *       - Fence is disabled via parameter or aux switch
 *       - Aircraft is disarmed (breach detection runs but no action taken)
 *       - Landing impact is expected (final flare/touchdown phase)
 *       - Already in fence recovery mode (prevents re-triggering)
 * 
 * @warning This is a critical safety system that prevents flyaways and enforces
 *          operational boundaries. Fence breach actions can override pilot commands
 *          and change flight modes automatically. All fence configurations must be
 *          tested thoroughly in SITL simulation before flight operations.
 * 
 * @warning Modifying fence parameters or breach actions during flight can affect
 *          vehicle safety and stability. Changes should be validated on the ground.
 * 
 * @see fence_checks_async() for asynchronous breach detection
 * @see in_fence_recovery() for recovery state determination
 * @see AC_Fence::get_action() for configured breach action
 * @see AC_Fence::get_breaches() for active breach bitmask
 */
void Plane::fence_check()
{
    if (!fence_breaches.have_updates) {
        return;
    }
    /*
      if we are either disarmed or we are currently not in breach and
      we are not flying then clear the state associated with the
      previous mode breach handling. This allows the fence state
      machine to reset at the end of a fence breach action such as an
      RTL and autoland
     */
    const bool armed = arming.is_armed();

    if (plane.previous_mode_reason == ModeReason::FENCE_BREACHED) {
        if (!armed || ((fence_breaches.new_breaches == 0 && orig_breaches == 0) && !plane.is_flying())) {
            plane.previous_mode_reason = ModeReason::FENCE_REENABLE;
        }
    }

    if (!fence.enabled()) {
        // Switch back to the chosen control mode if still in
        // GUIDED to the return point
        switch(fence.get_action()) {
            case AC_Fence::Action::GUIDED:
            case AC_Fence::Action::GUIDED_THROTTLE_PASS:
            case AC_Fence::Action::RTL_AND_LAND:
            case AC_Fence::Action::AUTOLAND_OR_RTL:
                if (plane.control_mode_reason == ModeReason::FENCE_BREACHED &&
                    control_mode->is_guided_mode()) {
                    set_mode(*previous_mode, ModeReason::FENCE_RETURN_PREVIOUS_MODE);
                }
                break;
            default:
                // No returning to a previous mode, unless our action allows it
                break;
        }
        /*
          clear mode reasons if they are FENCE_BREACHED to allow AUX
          switch fence disable/enable to re-enable the fence after a breach
         */
        if (plane.previous_mode_reason == ModeReason::FENCE_BREACHED) {
            plane.previous_mode_reason = ModeReason::FENCE_REENABLE;
        }
        if (plane.control_mode_reason == ModeReason::FENCE_BREACHED) {
            plane.control_mode_reason = ModeReason::FENCE_REENABLE;
        }
        goto fence_check_complete;
    }

    // we still don't do anything when disarmed, but we do check for fence breaches.
    // fence pre-arm check actually checks if any fence has been breached
    // that's not ever going to be true if we don't call check on AP_Fence while disarmed
    if (!armed) {
        goto fence_check_complete;
    }

    // Never trigger a fence breach in the final stage of landing
    if (landing.is_expecting_impact()) {
        goto fence_check_complete;
    }

    if (in_fence_recovery()) {
        // we have already triggered, don't trigger again until the
        // user disables/re-enables using the fence channel switch
        goto fence_check_complete;
    }

    if (fence_breaches.new_breaches) {
        fence.print_fence_message("breached", fence_breaches.new_breaches);

        // if the user wants some kind of response and motors are armed
        const auto fence_act = fence.get_action();
        switch (fence_act) {
        case AC_Fence::Action::REPORT_ONLY:
            // REPORT_ONLY action: Log the breach and send GCS notification,
            // but do not change flight mode or take any corrective action.
            // This allows monitoring fence breaches without automated intervention,
            // useful for testing fence boundaries or when manual recovery is preferred.
            break;

        case AC_Fence::Action::ALWAYS_LAND:
        case AC_Fence::Action::SMART_RTL:
        case AC_Fence::Action::SMART_RTL_OR_LAND:
        case AC_Fence::Action::BRAKE:
            // These fence actions are not applicable to fixed-wing aircraft.
            // ALWAYS_LAND, SMART_RTL, and BRAKE are designed for multicopters
            // that can hover and descend vertically. Fixed-wing aircraft require
            // horizontal forward flight and cannot perform these maneuvers safely.
            // Invalid enumeration value for Plane - no action taken.
            break;

        case AC_Fence::Action::AUTOLAND_OR_RTL:
        case AC_Fence::Action::RTL_AND_LAND:
            // RTL (Return To Launch) action: Switch to RTL mode and execute landing sequence.
            // This is the most common fence breach response for fixed-wing aircraft, providing
            // automated return to home or rally point followed by landing.
            //
            // Special case handling:
            // - If already in AUTO mode executing a landing sequence, do not interrupt
            //   (prevents disruption of legitimate approach and landing operations)
            // - If already in AUTOLAND mode, do not switch away (landing in progress)
            //
            // AUTOLAND_OR_RTL: Prefer MODE_AUTOLAND if compiled in and mode switch succeeds,
            // otherwise fall back to RTL mode. AUTOLAND provides more direct approach to landing.
            //
            // RTL_AND_LAND: Always use RTL mode, which returns to home/rally at RTL_ALTITUDE,
            // then executes landing sequence based on RTL_AUTOLAND parameter configuration.
            
            if (control_mode == &mode_auto &&
                mission.get_in_landing_sequence_flag() &&
                (g.rtl_autoland == RtlAutoland::RTL_THEN_DO_LAND_START ||
                    g.rtl_autoland == RtlAutoland::RTL_IMMEDIATE_DO_LAND_START)) {
                // already landing
                goto fence_check_complete;
            }
#if MODE_AUTOLAND_ENABLED
            if (control_mode == &mode_autoland) {
                // Already landing
                return;
            }
            if ((fence_act == AC_Fence::Action::AUTOLAND_OR_RTL) && set_mode(mode_autoland, ModeReason::FENCE_BREACHED)) {
                break;
            }
#endif
            set_mode(mode_rtl, ModeReason::FENCE_BREACHED);
            break;

        case AC_Fence::Action::GUIDED:
        case AC_Fence::Action::GUIDED_THROTTLE_PASS:
            // GUIDED mode action: Switch to GUIDED mode and navigate to calculated return point.
            // This provides more controlled fence breach recovery compared to RTL, allowing
            // the aircraft to return to a specific location while maintaining pilot oversight.
            //
            // GUIDED: Full autonomous control of throttle, roll, pitch, and yaw to return point
            // GUIDED_THROTTLE_PASS: Same navigation, but pilot retains manual throttle control
            //                       (useful for maintaining altitude control during recovery)
            //
            // Return point selection logic:
            // 1. If FENCE_RET_RALLY > 0: Calculate best rally point or home based on distance
            // 2. Otherwise: Use polygon fence return point with calculated altitude
            //
            // Return altitude determination (when not using rally point):
            // - If FENCE_RET_ALT > 0: Use specified return altitude (meters above home)
            // - If FENCE_ALT_MIN >= FENCE_ALT_MAX (invalid): Use RTL_ALTITUDE parameter
            // - Otherwise: Use midpoint between FENCE_ALT_MIN and FENCE_ALT_MAX (safe zone)
            //
            // Return point latitude/longitude (when not using rally point):
            // - If polygon fence has return point defined: Use that point (typically centroid)
            // - If no return point (e.g., only exclusion zones): Use home location as fallback
            
            set_mode(mode_guided, ModeReason::FENCE_BREACHED);

            Location loc;
            if (fence.get_return_rally() != 0) {
                // Use rally point system - calculate closest rally point or home
                loc = calc_best_rally_or_home_location(current_loc, get_RTL_altitude_cm());
            } else {
                // Return to fence return point, not a rally point
                
                // Calculate return altitude
                if (fence.get_return_altitude() > 0) {
                    // Fly to the return point using FENCE_RET_ALT parameter (meters above home)
                    loc.set_alt_cm(home.alt + 100.0 * fence.get_return_altitude(),
                                   Location::AltFrame::ABSOLUTE);
                } else if (fence.get_safe_alt_min() >= fence.get_safe_alt_max()) {
                    // Invalid min/max configuration, use RTL_ALTITUDE as safe default
                    loc.set_alt_cm(home.alt + g.RTL_altitude*100,
                                   Location::AltFrame::ABSOLUTE);
                } else {
                    // Fly to altitude midway between min and max safe altitudes
                    // This ensures we're well within the safe altitude fence boundaries
                    loc.set_alt_cm(home.alt + 100.0f * (fence.get_safe_alt_min() + fence.get_safe_alt_max()) / 2,
                                   Location::AltFrame::ABSOLUTE);
                }

                // Calculate return latitude/longitude
                Vector2l return_point;
                if(fence.polyfence().get_return_point(return_point)) {
                    // Use polygon fence return point (typically centroid of inclusion fence)
                    loc.lat = return_point[0];
                    loc.lng = return_point[1];
                } else {
                    // When no fence return point is found (ie. no inclusion fence uploaded, but exclusion is)
                    // we fail to obtain a valid fence return point. In this case, home is considered a safe
                    // return point.
                    loc.lat = home.lat;
                    loc.lng = home.lng;
                }
            }

            // Adjust target altitude for terrain following if enabled
            setup_terrain_target_alt(loc);
            
            // Set the guided mode waypoint to the calculated return location
            set_guided_WP(loc);

            // Enable throttle passthrough if GUIDED_THROTTLE_PASS action selected
            if (fence.get_action() == AC_Fence::Action::GUIDED_THROTTLE_PASS) {
                guided_throttle_passthru = true;
            }
            break;
        }

        LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_FENCE, LogErrorCode(fence_breaches.new_breaches));
    } else if (orig_breaches && fence.get_breaches() == 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Fence breach cleared");
        // record clearing of breach
        LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_FENCE, LogErrorCode::ERROR_RESOLVED);
    }

fence_check_complete:
    fence_breaches.have_updates = false;
}

/**
 * @brief Determine if pilot stick input should be mixed during fence recovery
 * 
 * @details During active fence breach recovery, this function controls whether
 *          pilot control inputs (aileron, elevator, rudder) are mixed with the
 *          automated fence recovery commands. Disabling stick mixing ensures
 *          the fence recovery action can complete without pilot interference.
 *          
 *          Stick mixing is disabled when ALL of these conditions are true:
 *          - Fence is enabled
 *          - One or more fence breaches are currently active
 *          - Vehicle is in fence recovery mode
 *          
 *          This prevents the pilot from inadvertently flying further outside
 *          the fence boundary while the automated recovery is attempting to
 *          return the aircraft to the safe zone.
 * 
 * @return true if pilot stick inputs should be mixed with navigation commands
 * @return false if pilot inputs should be ignored (fence recovery in progress)
 * 
 * @note When stick mixing is disabled, the pilot loses manual control authority
 *       over roll, pitch, and yaw (throttle may still be available with
 *       GUIDED_THROTTLE_PASS action).
 * 
 * @see in_fence_recovery() for recovery state determination
 * @see fence_check() for fence breach action triggering
 */
bool Plane::fence_stickmixing(void) const
{
    if (fence.enabled() &&
        fence.get_breaches() &&
        in_fence_recovery())
    {
        // don't mix in user input during active fence recovery
        return false;
    }
    // normal mixing rules apply
    return true;
}

/**
 * @brief Check if the vehicle is currently in fence breach recovery mode
 * 
 * @details Determines whether the vehicle is actively recovering from a fence breach,
 *          which affects behavior such as stick mixing, mode transition permissions,
 *          and re-breach detection. Recovery mode persists from initial breach through
 *          the complete recovery sequence including any mode transitions.
 *          
 *          Recovery state is TRUE when:
 *          - Current mode was entered due to fence breach (FENCE_BREACHED reason), OR
 *          - Previous mode was due to fence breach AND current mode is a completion
 *            transition (e.g., RTL completed and transitioning to landing mode)
 *          
 *          Recovery state is FALSE when:
 *          - In AUTO mode but not in landing sequence (user may have changed mission)
 *          - Mode was entered for reasons other than fence breach
 *          - Fence breach has been fully resolved and aircraft returned to normal ops
 *          
 *          Mode transition reasons indicating recovery completion:
 *          - RTL_COMPLETE_SWITCHING_TO_VTOL_LAND_RTL: RTL finished, entering QRTL
 *          - RTL_COMPLETE_SWITCHING_TO_FIXEDWING_AUTOLAND: RTL finished, entering AUTOLAND
 *          - QRTL_INSTEAD_OF_RTL: Quadplane using QRTL instead of conventional RTL
 *          - QLAND_INSTEAD_OF_RTL: Quadplane using QLAND instead of conventional RTL
 * 
 * @return true if vehicle is in fence breach recovery mode
 * @return false if normal operations (no active fence recovery)
 * 
 * @note This function is used to prevent re-triggering fence breaches during
 *       the recovery sequence and to disable pilot stick mixing during recovery.
 * 
 * @see fence_check() for fence breach detection and action triggering
 * @see fence_stickmixing() for stick mixing control during recovery
 */
bool Plane::in_fence_recovery() const
{
    if (control_mode == &mode_auto && !mission.get_in_landing_sequence_flag()) {
        // the user may have changed target WP to be outside the
        // landing sequence - not considered fence recovery
        return false;
    }
    const bool current_mode_breach = plane.control_mode_reason == ModeReason::FENCE_BREACHED;
    const bool previous_mode_breach = plane.previous_mode_reason ==  ModeReason::FENCE_BREACHED;
    const bool previous_mode_complete = (plane.control_mode_reason == ModeReason::RTL_COMPLETE_SWITCHING_TO_VTOL_LAND_RTL) ||
                                        (plane.control_mode_reason == ModeReason::RTL_COMPLETE_SWITCHING_TO_FIXEDWING_AUTOLAND) ||
                                        (plane.control_mode_reason == ModeReason::QRTL_INSTEAD_OF_RTL) ||
                                        (plane.control_mode_reason == ModeReason::QLAND_INSTEAD_OF_RTL);

    return current_mode_breach || (previous_mode_breach && previous_mode_complete);
}

#endif
