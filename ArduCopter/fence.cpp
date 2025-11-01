/**
 * @file fence.cpp
 * @brief Geofencing system integration for ArduCopter
 * 
 * @details This file integrates the AC_Fence library with the ArduCopter vehicle code,
 *          providing comprehensive boundary enforcement and breach response capabilities.
 *          
 *          The fence system supports multiple fence types:
 *          - **Cylindrical Fence**: Maximum altitude (AC_FENCE_TYPE_ALT_MAX) and maximum
 *            horizontal distance from home point (AC_FENCE_TYPE_CIRCLE)
 *          - **Polygon Inclusion Zones**: Geographic boundary polygons that the vehicle
 *            must remain within (AC_FENCE_TYPE_POLYGON)
 *          - **Polygon Exclusion Zones**: Geographic areas the vehicle must avoid
 *          
 *          **Fence Breach Detection**:
 *          The AC_Fence library performs the actual breach detection by comparing current
 *          vehicle position (from AHRS/EKF) against configured fence boundaries. Detection
 *          runs asynchronously at 25Hz to minimize impact on main loop performance.
 *          
 *          **Breach Actions**:
 *          When a fence is breached, the system can execute various responses:
 *          - REPORT_ONLY: Log breach but take no action
 *          - RTL_AND_LAND: Return to launch, fall back to LAND if RTL unavailable
 *          - ALWAYS_LAND: Immediately land at current location
 *          - SMART_RTL: Follow safe return path, fall back to RTL then LAND
 *          - BRAKE: Stop vehicle in place, fall back to LAND if BRAKE unavailable
 *          - SMART_RTL_OR_LAND: Smart return or immediate land
 *          
 *          **Manual Recovery**:
 *          Pilot can manually disable fence using channel option to regain control.
 *          Fence breach is automatically cleared when vehicle returns within boundaries
 *          while still airborne.
 *          
 *          **Integration with Avoidance**:
 *          The fence system works in parallel with object avoidance (AC_Avoidance).
 *          Fence enforces hard boundaries (triggers mode changes), while avoidance
 *          provides dynamic obstacle avoidance within those boundaries.
 *          
 * @note Fence checks are suspended when disarmed to allow for pre-flight testing
 * @warning Fence breach actions can override pilot control and trigger autonomous modes
 * 
 * @see AC_Fence library (libraries/AC_Fence/)
 * @see AP_Arming::fence_checks() for pre-arm validation
 */

#include "Copter.h"

// Code to integrate AC_Fence library with main ArduCopter code

#if AP_FENCE_ENABLED

/**
 * @brief Asynchronous fence breach checking callback
 * 
 * @details This function is called from the high-speed IO timer at 1kHz but rate-limits
 *          itself to perform actual fence checks at 25Hz. The asynchronous design prevents
 *          fence checking computational overhead from impacting the main loop timing.
 *          
 *          **Operation Flow**:
 *          1. Rate limit to 25Hz (40ms intervals) to balance responsiveness with CPU load
 *          2. Check if previous breach status has been processed by main loop
 *          3. Query AC_Fence library to check all enabled fence types against current position
 *          4. Detect landing/landed state to suppress spurious breaches on ground
 *          5. Latch new breach status for main loop to process
 *          6. Clear resolved breaches and send notification to ground station
 *          
 *          **Fence Types Checked**:
 *          - AC_FENCE_TYPE_ALT_MAX: Maximum altitude above home
 *          - AC_FENCE_TYPE_CIRCLE: Maximum horizontal distance from home (cylindrical)
 *          - AC_FENCE_TYPE_POLYGON: Inclusion/exclusion polygon boundaries
 *          - AC_FENCE_TYPE_ALT_MIN: Minimum altitude (terrain following fences)
 *          
 *          **Landing/Landed Handling**:
 *          Fence checks are relaxed when landing or landed to prevent false breach
 *          detections due to GPS drift or fence configuration issues on the ground.
 *          This allows the vehicle to arm even if technically outside a fence boundary
 *          at the arming location.
 *          
 *          **Breach Latching**:
 *          New breaches are latched in fence_breaches.new_breaches (bitmask) and the
 *          have_updates flag is set. The async checker then waits for fence_check()
 *          in the main loop to process and clear have_updates before checking again.
 *          This prevents race conditions and ensures breach actions complete before
 *          detecting additional breaches.
 * 
 * @note Called at 1kHz from IO timer but self-limits to 25Hz execution rate
 * @note Suspended when disarmed to allow pre-flight fence testing without triggering actions
 * @warning Do not perform mode changes or heavy processing in this async callback
 * 
 * @see fence_check() for breach response handling in main loop
 * @see AC_Fence::check() for actual breach detection algorithm
 */
void Copter::fence_checks_async()
{
    const uint32_t now = AP_HAL::millis();

    // Rate limit to 25Hz (40ms intervals) to balance breach detection responsiveness
    // with CPU overhead. 25Hz is sufficient for detecting fence breaches at typical
    // multicopter velocities while keeping computational load manageable
    if (!AP_HAL::timeout_expired(fence_breaches.last_check_ms, now, 40U)) { // 25Hz update rate
        return;
    }

    // Wait for main loop to process previous breach status before checking again.
    // This prevents race conditions where we might overwrite breach status before
    // the main loop has had a chance to initiate the breach response action
    if (fence_breaches.have_updates) {
        return; // wait for the main loop to pick up the new breaches before checking again
    }

    fence_breaches.last_check_ms = now;
    const uint8_t orig_breaches = fence.get_breaches();
    
    // Determine if we're landing, landed, or disarmed. In these states, fence checks
    // are relaxed to prevent false breach detections from GPS drift on the ground
    // or configuration issues where the fence doesn't include the landing/arming location
    bool is_landing_or_landed = flightmode->is_landing() || ap.land_complete  || !motors->armed();

    // Check all enabled fence types against current position from AHRS/EKF.
    // Returns bitmask of breached fence types:
    // - AC_FENCE_TYPE_ALT_MAX (0x01): Maximum altitude exceeded
    // - AC_FENCE_TYPE_CIRCLE (0x02): Maximum distance from home exceeded
    // - AC_FENCE_TYPE_POLYGON (0x04): Outside inclusion zone or inside exclusion zone
    // - AC_FENCE_TYPE_ALT_MIN (0x08): Minimum altitude violated (terrain following)
    fence_breaches.new_breaches = fence.check(is_landing_or_landed);

    // Detect when a previously active breach has been cleared (vehicle returned within boundaries)
    // Only notify if we're still airborne - suppress notifications on the ground
    if (!fence_breaches.new_breaches && orig_breaches && fence.get_breaches() == 0) {
        if (!copter.ap.land_complete) {
            GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Fence breach cleared");
        }
        // Log breach resolution for post-flight analysis and diagnostics
        LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_FENCE, LogErrorCode::ERROR_RESOLVED);
    }
    
    // Latch new breach status for main loop processing. The have_updates flag signals
    // fence_check() that new breach data is available and needs to be handled
    fence_breaches.have_updates = true; // new breach status latched so main loop will now pick it up
}

/**
 * @brief Main loop fence breach response handler
 * 
 * @details This function is called at 25Hz from the main loop to process fence breaches
 *          detected by fence_checks_async() and initiate appropriate response actions.
 *          
 *          **Response Decision Tree**:
 *          
 *          1. **No Action Conditions**:
 *             - No new breach updates from async checker
 *             - Vehicle is disarmed (breaches detected but not acted upon)
 *             
 *          2. **Immediate Disarm Conditions**:
 *             - Vehicle is on the ground (ap.land_complete)
 *             - Manual flight mode with zero throttle (and not a high-altitude breach)
 *             
 *          3. **Extreme Breach Response** (>100m outside fence):
 *             - Force LAND mode immediately regardless of configured action
 *             - Prevents runaway scenarios where vehicle is far outside boundaries
 *             
 *          4. **Standard Breach Actions** (based on FENCE_ACTION parameter):
 *             - **RTL_AND_LAND**: Attempt RTL, fall back to LAND if RTL fails
 *             - **ALWAYS_LAND**: Immediately enter LAND mode
 *             - **SMART_RTL**: Try SmartRTL → RTL → LAND cascade
 *             - **BRAKE**: Hold position in BRAKE mode, fall back to LAND
 *             - **SMART_RTL_OR_LAND**: Try SmartRTL → LAND cascade
 *             - **REPORT_ONLY**: Log breach but take no action
 *             
 *          **Manual Fence Disable**:
 *          Pilot can disable fence via configured RC channel option to regain manual
 *          control. This is useful when fence is causing undesired behavior or if
 *          pilot needs to intentionally breach fence for safety reasons.
 *          
 *          **Fence Recovery**:
 *          When vehicle returns within fence boundaries while still airborne, breach
 *          status is automatically cleared (handled in fence_checks_async). Vehicle
 *          remains in the breach response mode (RTL/LAND/etc) unless pilot manually
 *          switches modes.
 *          
 *          **High-Altitude Fence Special Case**:
 *          High-altitude breaches (AC_FENCE_TYPE_ALT_MAX) do not trigger immediate
 *          disarm on zero throttle, as pilot is likely intentionally descending to
 *          recover from the altitude breach.
 * 
 * @note Called at 25Hz from main loop (400Hz loop rate with 16x decimation)
 * @note Does not perform fence breach detection - only processes results from fence_checks_async()
 * 
 * @warning Fence breach actions override pilot control and can trigger autonomous modes
 * @warning Extreme breaches (>100m outside fence) force LAND regardless of configured action
 * @warning Fence disarm on ground can occur unexpectedly if GPS drifts outside fence
 * 
 * @see fence_checks_async() for breach detection
 * @see AC_Fence::get_action() for configured breach action
 * @see AC_Fence::get_breach_distance() for breach severity calculation
 */
void Copter::fence_check()
{
    // Check if there are new breach updates from the async checker.
    // If no updates are pending, no action is needed this iteration
    if (!fence_breaches.have_updates) {
        return;
    }

    // When disarmed, we detect and track fence breaches but don't execute breach actions.
    // This is important because:
    // 1. Allows fence breach detection to run continuously for pre-arm checks
    // 2. Pre-arm check validates that vehicle is not breaching fence at arm time
    // 3. Prevents unexpected motor disarm/mode changes on the ground
    // 4. Allows pilot to arm and takeoff even if initially outside fence (will breach immediately)
    if (!motors->armed()) {
        fence_breaches.have_updates = false; // fence checking can now be processed again
        return;
    }

    // Process new fence breaches detected by fence_checks_async()
    if (fence_breaches.new_breaches) {

        // Send breach notification to ground station (suppress if on ground to avoid spam)
        // Message includes which fence types were breached (altitude, circle, polygon)
        if (!copter.ap.land_complete) {
            fence.print_fence_message("breached", fence_breaches.new_breaches);
        }

        // Retrieve configured fence action from FENCE_ACTION parameter.
        // Action determines vehicle response: RTL, Land, Brake, SmartRTL, or ReportOnly
        const auto fence_act = fence.get_action();
        if (fence_act != AC_Fence::Action::REPORT_ONLY ) {

            // **Immediate Disarm Logic**:
            // Disarm immediately if we're on the ground or hovering at zero throttle in manual mode.
            // This prevents the vehicle from sitting on the ground breaching fence or attempting
            // autonomous actions when pilot has throttle at zero (indicating intent to land/disarm).
            //
            // **High-Altitude Fence Exception**:
            // Do NOT disarm on zero throttle if only the altitude fence is breached. In this case,
            // pilot has likely pulled throttle to zero to descend and recover from altitude breach.
            // Disarming would cause uncontrolled fall from high altitude.
            if (ap.land_complete || (flightmode->has_manual_throttle() && ap.throttle_zero && !failsafe.radio && ((fence.get_breaches() & AC_FENCE_TYPE_ALT_MAX)== 0))){
                arming.disarm(AP_Arming::Method::FENCEBREACH);

            } else {

                // **Extreme Breach Emergency Response**:
                // If vehicle is more than 100m outside fence boundaries (AC_FENCE_GIVE_UP_DISTANCE),
                // force LAND mode regardless of configured action. This handles runaway scenarios
                // where RTL might not be safe or effective. Vehicle will land at current position.
                if (fence.get_breach_distance(fence_breaches.new_breaches) > AC_FENCE_GIVE_UP_DISTANCE) {
                    set_mode(Mode::Number::LAND, ModeReason::FENCE_BREACHED);
                } else {
                    // **Standard Breach Action Selection**:
                    // Execute configured fence action with appropriate fallback modes.
                    // All actions ultimately fall back to LAND if preferred mode unavailable.
                    switch (fence_act) {
                    case AC_Fence::Action::RTL_AND_LAND:
                    default:
                        // **RTL_AND_LAND**: Return to launch point, land if RTL unavailable
                        // RTL might fail if: home not set, GPS lost, mode disabled
                        if (!set_mode(Mode::Number::RTL, ModeReason::FENCE_BREACHED)) {
                            set_mode(Mode::Number::LAND, ModeReason::FENCE_BREACHED);
                        }
                        break;
                    case AC_Fence::Action::ALWAYS_LAND:
                        // **ALWAYS_LAND**: Immediately land at current location
                        // Used when RTL is not desired (e.g., indoor flight, confined areas)
                        set_mode(Mode::Number::LAND, ModeReason::FENCE_BREACHED);
                        break;
                    case AC_Fence::Action::SMART_RTL:
                        // **SMART_RTL**: Follow safe recorded path back to launch
                        // SmartRTL → RTL → LAND cascade provides maximum return path options
                        // SmartRTL might fail if: path not recorded, GPS lost, insufficient memory
                        if (!set_mode(Mode::Number::SMART_RTL, ModeReason::FENCE_BREACHED)) {
                            if (!set_mode(Mode::Number::RTL, ModeReason::FENCE_BREACHED)) {
                                set_mode(Mode::Number::LAND, ModeReason::FENCE_BREACHED);
                            }
                        }
                        break;
                    case AC_Fence::Action::BRAKE:
                        // **BRAKE**: Hold position in place (useful for inspection/pilot assessment)
                        // BRAKE might fail if: position estimate invalid, mode disabled
                        // Vehicle will hold position until pilot intervenes or battery depletes
                        if (!set_mode(Mode::Number::BRAKE, ModeReason::FENCE_BREACHED)) {
                            set_mode(Mode::Number::LAND, ModeReason::FENCE_BREACHED);
                        }
                        break;
                    case AC_Fence::Action::SMART_RTL_OR_LAND:
                        // **SMART_RTL_OR_LAND**: Try safe path return, otherwise land in place
                        // Skips standard RTL if SmartRTL unavailable (shorter fallback chain)
                        if (!set_mode(Mode::Number::SMART_RTL, ModeReason::FENCE_BREACHED)) {
                            set_mode(Mode::Number::LAND, ModeReason::FENCE_BREACHED);
                        }
                        break;
                    }
                }
            }
        }

        // Log fence breach to dataflash for post-flight analysis.
        // LogErrorCode contains bitmask of breached fence types:
        // - Bit 0 (0x01): AC_FENCE_TYPE_ALT_MAX - Maximum altitude breach
        // - Bit 1 (0x02): AC_FENCE_TYPE_CIRCLE - Cylindrical horizontal distance breach
        // - Bit 2 (0x04): AC_FENCE_TYPE_POLYGON - Polygon inclusion/exclusion breach
        // - Bit 3 (0x08): AC_FENCE_TYPE_ALT_MIN - Minimum altitude breach
        // This log entry is critical for understanding fence-related incidents
        LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_FENCE, LogErrorCode(fence_breaches.new_breaches));
    }
    
    // Clear the have_updates flag to signal async checker it can detect new breaches.
    // This completes the handshake between async detection and main loop response
    fence_breaches.have_updates = false; // fence checking can now be processed again
}

#endif
