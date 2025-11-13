/**
 * @file fence.cpp
 * @brief Rover geofencing integration with AC_Fence library
 * 
 * @details This file implements the rover-specific interface to the ArduPilot
 *          geofencing system (AC_Fence library). It handles:
 *          - Asynchronous fence breach checking at 10Hz
 *          - Fence breach response actions (Hold, RTL, SmartRTL, Loiter, Terminate)
 *          - Integration with rover mode system for breach responses
 *          - Manual recovery window allowing pilot override after breach
 * 
 *          The AC_Fence library provides the core fence types:
 *          - Circular (inclusion/exclusion) fences
 *          - Polygon (inclusion/exclusion) fences  
 *          - Altitude (min/max) fences
 * 
 *          This implementation provides the vehicle-specific breach response
 *          behavior for ground vehicles.
 * 
 * @warning Geofencing is a safety-critical system. Incorrect configuration
 *          or breach handling can result in vehicle loss or damage. Always
 *          test fence configuration in a safe environment before operational use.
 * 
 * Source: Rover/fence.cpp
 */

#include "Rover.h"

#if AP_FENCE_ENABLED

/**
 * @brief Asynchronous fence breach checking callback
 * 
 * @details This function is called at 1kHz from the scheduler as an I/O callback,
 *          but performs actual fence checks at 10Hz (100ms intervals) to balance
 *          responsiveness with computational load. The asynchronous design prevents
 *          fence checking from blocking the main loop.
 * 
 *          Asynchronous fence checking workflow:
 *          1. Check if 100ms has elapsed since last check (10Hz rate limiting)
 *          2. Verify main loop has processed previous breach updates
 *          3. Poll AC_Fence library for current breach status
 *          4. Latch new breaches for main loop processing
 *          5. Log breach resolution if fence cleared
 * 
 *          The fence_breaches structure acts as a lockless communication mechanism
 *          between this async callback and the main loop fence_check() function.
 *          The have_updates flag prevents overwriting breach data before the main
 *          loop processes it, ensuring no breaches are missed.
 * 
 * @note Called from scheduler I/O thread at 1kHz, but rate-limited to 10Hz internally
 * @note Non-blocking design - returns immediately if main loop hasn't processed previous updates
 * 
 * @warning This function runs in interrupt context on some platforms. Keep execution
 *          time minimal and avoid blocking operations.
 * 
 * @see fence_check() Main loop function that responds to detected breaches
 * @see AC_Fence::check() Core fence breach detection in AC_Fence library
 */
// async fence checking io callback at 1Khz
void Rover::fence_checks_async()
{
    const uint32_t now = AP_HAL::millis();

    // Rate limit to 10Hz (100ms intervals) for fence checking
    // This balances breach detection responsiveness with CPU load
    if (!AP_HAL::timeout_expired(fence_breaches.last_check_ms, now, 100U)) { // 10Hz update rate
        return;
    }

    // Wait for main loop to consume previous breach updates before checking again
    // This prevents overwriting breach data and ensures all breaches are handled
    if (fence_breaches.have_updates) {
        return; // wait for the main loop to pick up the new breaches before checking again
    }

    fence_breaches.last_check_ms = now;
    const uint8_t orig_breaches = fence.get_breaches();
    
    // Poll AC_Fence library for current breach status
    // Returns bitmask of fence types breached (circle, polygon, altitude)
    // check for new breaches; new_breaches is bitmask of fence types breached
    fence_breaches.new_breaches = fence.check();

    // Detect fence breach resolution (vehicle returned to safe zone)
    if (!fence_breaches.new_breaches && orig_breaches && fence.get_breaches() == 0) {
        // record clearing of breach
        LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_FENCE, LogErrorCode::ERROR_RESOLVED);
    }
    
    // Latch breach updates for main loop processing
    // Main loop fence_check() will clear this flag after handling breaches
    fence_breaches.have_updates = true; // new breaches latched so main loop will now pick it up
}

/**
 * @brief Main loop fence breach handler - initiates response to fence violations
 * 
 * @details This function is called from the main loop and processes fence breach
 *          updates detected by fence_checks_async(). It implements the rover-specific
 *          response to geofence violations based on the FENCE_ACTION parameter.
 * 
 *          Fence breach response workflow:
 *          1. Check for new breach updates from async checker
 *          2. Verify vehicle is armed (no action if disarmed)
 *          3. Calculate breach distance from fence boundary
 *          4. Execute configured breach action based on distance
 *          5. Log breach event for post-flight analysis
 *          6. Clear breach update flag to allow new checks
 * 
 *          Breach distance threshold (AC_FENCE_GIVE_UP_DISTANCE = 100m):
 *          - Within 100m: Execute configured FENCE_ACTION (RTL, SmartRTL, Hold, etc.)
 *          - Beyond 100m: Force HOLD mode (vehicle too far for safe return)
 * 
 *          Manual recovery window:
 *          After a fence breach triggers a mode change, the pilot can manually
 *          switch to another mode to recover. The fence system will not force
 *          the vehicle back into breach response mode unless another breach occurs.
 *          This allows pilot intervention while maintaining safety monitoring.
 * 
 *          Integration with AC_Fence library:
 *          - fence.check(): Performs geometric breach detection (async callback)
 *          - fence.get_breaches(): Returns bitmask of active breaches
 *          - fence.get_breach_distance(): Distance to nearest fence boundary
 *          - fence.get_action(): Configured FENCE_ACTION parameter value
 * 
 * @note Called from main loop at scheduler task rate (typically 50Hz)
 * @note Only acts on armed vehicle - disarmed state clears breach flags
 * @note Breach actions use fallthrough pattern for graceful degradation
 * 
 * @warning SAFETY-CRITICAL FUNCTION: This function changes vehicle mode autonomously
 *          in response to fence breaches. Incorrect fence configuration or action
 *          selection can result in unexpected vehicle behavior, loss of control,
 *          or vehicle damage. Always test fence setup in safe environments.
 * 
 * @warning The 100m breach distance threshold is hardcoded (AC_FENCE_GIVE_UP_DISTANCE).
 *          Vehicles more than 100m outside the fence will enter HOLD mode regardless
 *          of FENCE_ACTION setting, preventing potentially unsafe long-distance returns.
 * 
 * @warning Manual recovery allows pilot override but does NOT disable fence checking.
 *          Pilot must navigate back inside fence boundaries or disable fencing to
 *          regain full control authority.
 * 
 * @see fence_checks_async() Asynchronous breach detection function
 * @see AC_Fence ArduPilot fence library providing core breach detection
 * @see set_mode() Vehicle mode change function
 */
// fence_check - ask fence library to check for breaches and initiate the response
void Rover::fence_check()
{
    // Only process if async checker has detected new breach updates
    // This flag is set by fence_checks_async() when new breaches are detected
    // only take action if there is a new breach
    if (!fence_breaches.have_updates) {
        return;
    }

    // No breach actions while disarmed - clear updates and return
    // This prevents mode changes during pre-flight setup and testing
    // return immediately if motors are not armed
    if (!arming.is_armed()) {
        fence_breaches.have_updates = false;
        return;
    }

    // Process fence breach - new_breaches is bitmask of violated fence types
    if (fence_breaches.new_breaches) {
        // Execute breach action if configured (FENCE_ACTION parameter)
        // if the user wants some kind of response and motors are armed
        if ((FailsafeAction)fence.get_action() != FailsafeAction::None) {
            
            // Distance-based breach handling:
            // Within 100m: Execute configured action with fallback chain
            // Beyond 100m: Force HOLD (too far for safe autonomous return)
            // if within 100m of the fence, it will take the action specified by the FENCE_ACTION parameter
            if (fence.get_breach_distance(fence_breaches.new_breaches) <= AC_FENCE_GIVE_UP_DISTANCE) {
                
                // Execute configured breach action with fallback chain
                // Fallthrough pattern provides graceful degradation if preferred mode unavailable
                switch ((FailsafeAction)fence.get_action()) {
                case FailsafeAction::None:
                    // No autonomous action - pilot retains full control
                    break;
                    
                case FailsafeAction::SmartRTL:
                    // Attempt SmartRTL (return via recorded path)
                    // Falls through to RTL if SmartRTL unavailable (no path recorded)
                    if (set_mode(mode_smartrtl, ModeReason::FENCE_BREACHED)) {
                        break;
                    }
                    FALLTHROUGH;
                    
                case FailsafeAction::RTL:
                    // Attempt RTL (return to launch via direct path)
                    // Falls through to HOLD if RTL unavailable (no home position)
                    if (set_mode(mode_rtl, ModeReason::FENCE_BREACHED)) {
                        break;
                    }
                    FALLTHROUGH;
                    
                case FailsafeAction::Hold:
                    // HOLD mode - stop vehicle immediately at current location
                    // This is the ultimate fallback - always succeeds
                    set_mode(mode_hold, ModeReason::FENCE_BREACHED);
                    break;
                    
                case FailsafeAction::SmartRTL_Hold:
                    // Try SmartRTL, fallback to HOLD (no RTL intermediate step)
                    if (!set_mode(mode_smartrtl, ModeReason::FENCE_BREACHED)) {
                        set_mode(mode_hold, ModeReason::FENCE_BREACHED);
                    }
                    break;
                    
                case FailsafeAction::Loiter_Hold:
                    // Try Loiter (circle at current location), fallback to HOLD
                    // Loiter maintains position using GPS/compass navigation
                    if (!set_mode(mode_loiter, ModeReason::FENCE_BREACHED)) {
                        set_mode(mode_hold, ModeReason::FENCE_BREACHED);
                    }
                    break;
                    
                case FailsafeAction::Terminate:
                    // Emergency termination - immediately disarm motors
                    // WARNING: Vehicle will coast to a stop with no active control
                    arming.disarm(AP_Arming::Method::FENCEBREACH);
                    break;
                }
            } else {
                // Vehicle is >100m outside fence boundary
                // Force HOLD mode - distance too great for safe autonomous return
                // Pilot must manually navigate back or disable fence
                // if more than 100m outside the fence just force to HOLD
                set_mode(mode_hold, ModeReason::FENCE_BREACHED);
            }
        }
        
        // Log fence breach event for post-flight analysis
        // LogErrorCode contains bitmask identifying which fence types were violated
        LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_FENCE, LogErrorCode(fence_breaches.new_breaches));
    }
    
    // Clear breach update flag to allow async checker to detect new breaches
    // Manual recovery window begins here - pilot can override mode if desired
    fence_breaches.have_updates = false;
}

#endif // AP_FENCE_ENABLED
