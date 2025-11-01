/**
 * @file avoidance_adsb.cpp
 * @brief ADS-B based collision avoidance implementation for multicopters
 * 
 * @details This file implements automatic collision avoidance for ArduCopter using
 *          Automatic Dependent Surveillance-Broadcast (ADS-B) data from nearby aircraft.
 *          The system continuously monitors ADS-B threats, assesses collision risk based
 *          on distance and closure rate, and executes evasive maneuvers when threats are
 *          detected. Avoidance actions include vertical climbs/descents, horizontal
 *          movements, and perpendicular evasion vectors.
 * 
 *          Key features:
 *          - Real-time threat assessment using distance and closure rate
 *          - Automatic mode transition to AVOID_ADSB when threat detected
 *          - Multiple evasion strategies (vertical, horizontal, perpendicular)
 *          - Automatic recovery and mode restoration after threat clears
 *          - Integration with failsafe system and GCS notifications
 * 
 * @note This implementation is safety-critical as it directly controls vehicle flight
 *       to avoid mid-air collisions. Changes must be thoroughly tested in SITL and
 *       verified on actual hardware.
 * 
 * @warning Avoidance maneuvers override pilot input during active threats. Ensure
 *          parameters are properly configured for vehicle flight envelope.
 */

#include "Copter.h"
#include <AP_Notify/AP_Notify.h>

#if HAL_ADSB_ENABLED || AP_ADSB_AVOIDANCE_ENABLED
/**
 * @brief Main update function for ADS-B avoidance system
 * 
 * @details Called at scheduler rate to update both ADS-B receiver data and avoidance
 *          logic. This function coordinates the two main components:
 *          - ADS-B receiver (adsb.update()) processes incoming transponder data
 *          - Avoidance logic (avoidance_adsb.update()) assesses threats and executes maneuvers
 * 
 * @note This function is called from the main scheduler loop and should execute quickly
 *       to avoid impacting overall loop rate (typically 400Hz main loop)
 */
void Copter::avoidance_adsb_update(void)
{
#if HAL_ADSB_ENABLED
    adsb.update();
#endif  // HAL_ADSB_ENABLED
#if AP_ADSB_AVOIDANCE_ENABLED
    avoidance_adsb.update();
#endif  // AP_ADSB_AVOIDANCE_ENABLED
}
#endif  // HAL_ADSB_ENABLED || AP_ADSB_AVOIDANCE_ENABLED

#if AP_ADSB_AVOIDANCE_ENABLED

#include <stdio.h>

/**
 * @brief Handle collision avoidance action for detected ADS-B threat
 * 
 * @details This is the primary avoidance handler that implements the collision avoidance
 *          response when an ADS-B threat is detected. The function:
 *          
 *          1. **Threat Detection & Failsafe Activation**: Activates ADSB failsafe flag
 *             when first threat detected, recording current flight mode for recovery
 *          
 *          2. **Mode-Specific Handling**: Prevents avoidance actions in certain modes
 *             (LAND, THROW, FLIP) where evasive maneuvers would be inappropriate or unsafe
 *          
 *          3. **Ground Safety**: If landed and threat detected, disarms instead of maneuvering
 *          
 *          4. **Action Execution**: Executes requested avoidance maneuver:
 *             - MAV_COLLISION_ACTION_RTL: Return to launch position
 *             - MAV_COLLISION_ACTION_HOVER: Hold position (switch to LOITER)
 *             - MAV_COLLISION_ACTION_ASCEND_OR_DESCEND: Vertical evasion
 *             - MAV_COLLISION_ACTION_MOVE_HORIZONTALLY: Horizontal evasion
 *             - MAV_COLLISION_ACTION_MOVE_PERPENDICULAR: 3D perpendicular evasion
 *          
 *          5. **GCS Notification**: Logs avoidance action to dataflash for post-flight analysis
 * 
 *          The threat assessment (performed by parent AP_Avoidance class) uses:
 *          - Distance to obstacle (typically < 200m triggers action)
 *          - Closure rate (approaching speed toward obstacle)
 *          - Time to collision calculation
 *          - Obstacle velocity vector
 * 
 * @param[in] obstacle Pointer to detected obstacle with position, velocity, and threat data
 * @param[in] requested_action Collision avoidance action requested by threat assessment algorithm
 * 
 * @return MAV_COLLISION_ACTION Actually executed action (may differ from requested if action failed)
 * 
 * @note This function is called at avoidance update rate (typically 10Hz) when threats detected
 * 
 * @warning Avoidance actions override pilot control. Function must handle all edge cases
 *          safely to prevent unintended behavior during critical flight phases
 * 
 * @see handle_recovery() for mode restoration after threat clears
 * @see handle_avoidance_vertical() for vertical evasion implementation
 * @see handle_avoidance_horizontal() for horizontal evasion implementation
 */
MAV_COLLISION_ACTION AP_Avoidance_Copter::handle_avoidance(const AP_Avoidance::Obstacle *obstacle, MAV_COLLISION_ACTION requested_action)
{
    MAV_COLLISION_ACTION actual_action = requested_action;
    bool failsafe_state_change = false;

    // THREAT DETECTION: Check for changes in failsafe state
    // When a new ADS-B threat is detected (based on distance and closure rate assessment),
    // activate the ADSB failsafe flag to indicate active collision avoidance in progress
    if (!copter.failsafe.adsb) {
        copter.failsafe.adsb = true;
        failsafe_state_change = true;
        // Record current flight mode for automatic restoration after threat clears
        // This allows seamless return to mission/mode after avoidance maneuver completes
        prev_control_mode = copter.flightmode->mode_number();
    }

    // MODE-SPECIFIC SAFETY: Take no avoidance action in certain flight modes where
    // evasive maneuvers would be dangerous or inappropriate:
    // - LAND: Vehicle is already landing, avoidance could cause ground collision
    // - THROW: Vehicle in throw mode startup sequence, not ready for maneuvers
    // - FLIP: Vehicle executing aerobatic flip, interruption would be catastrophic
    if (copter.flightmode->mode_number() == Mode::Number::LAND ||
#if MODE_THROW_ENABLED
        copter.flightmode->mode_number() == Mode::Number::THROW ||
#endif
        copter.flightmode->mode_number() == Mode::Number::FLIP) {
        actual_action = MAV_COLLISION_ACTION_NONE;
    }

    // GROUND SAFETY: If vehicle is on ground (or should disarm on failsafe) and action
    // is more than just reporting, simply disarm instead of attempting evasive maneuver
    // This prevents unnecessary motor startup or ground movements that could damage vehicle
    if ((actual_action > MAV_COLLISION_ACTION_REPORT) && copter.should_disarm_on_failsafe()) {
        copter.arming.disarm(AP_Arming::Method::ADSBCOLLISIONACTION);
        actual_action = MAV_COLLISION_ACTION_NONE;
    } else {

        // EVASIVE MANEUVER EXECUTION: Execute requested avoidance action
        // Each action type provides different evasion strategy based on threat geometry
        switch (actual_action) {

            case MAV_COLLISION_ACTION_RTL:
                // RETURN TO LAUNCH: Navigate away from threat area to home position
                // Only attempt mode change on initial failsafe activation to avoid repeated
                // mode switching. If RTL mode entry fails (e.g., no GPS position fix),
                // take no action to prevent vehicle instability
                if (failsafe_state_change) {
                    if (!copter.set_mode(Mode::Number::RTL, ModeReason::AVOIDANCE)) {
                        actual_action = MAV_COLLISION_ACTION_NONE;
                    }
                }
                break;

            case MAV_COLLISION_ACTION_HOVER:
                // HOLD POSITION: Stop vehicle movement by switching to LOITER mode
                // This is appropriate when threat is passing by and staying in place is safest
                // If LOITER fails (bad GPS, manual mode), take no action for safety
                if (failsafe_state_change) {
                    if (!copter.set_mode(Mode::Number::LOITER, ModeReason::AVOIDANCE)) {
                        actual_action = MAV_COLLISION_ACTION_NONE;
                    }
                }
                break;

            case MAV_COLLISION_ACTION_ASCEND_OR_DESCEND:
                // VERTICAL EVASION: Climb or descend to create vertical separation
                // Decision to climb vs descend based on relative altitude to obstacle
                // Transitions to AVOID_ADSB mode and commands vertical velocity
                if (!handle_avoidance_vertical(obstacle, failsafe_state_change)) {
                    actual_action = MAV_COLLISION_ACTION_NONE;
                }
                break;

            case MAV_COLLISION_ACTION_MOVE_HORIZONTALLY:
                // HORIZONTAL EVASION: Move perpendicular to threat vector in horizontal plane only
                // Calculates 2D evasion vector perpendicular to obstacle approach direction
                // Maintains current altitude while moving laterally away from threat
                if (!handle_avoidance_horizontal(obstacle, failsafe_state_change)) {
                    actual_action = MAV_COLLISION_ACTION_NONE;
                }
                break;

            case MAV_COLLISION_ACTION_MOVE_PERPENDICULAR:
                // 3D PERPENDICULAR EVASION: Move perpendicular to threat in all three dimensions
                // Calculates full 3D vector perpendicular to obstacle velocity vector
                // Provides maximum separation by utilizing vertical and horizontal movement
                if (!handle_avoidance_perpendicular(obstacle, failsafe_state_change)) {
                    actual_action = MAV_COLLISION_ACTION_NONE;
                }
                break;

            // NO ACTION CASES: Report-only or explicitly no action
            case MAV_COLLISION_ACTION_NONE:
                return actual_action;
            case MAV_COLLISION_ACTION_REPORT:
            default:
                break;
        }
    }

#if HAL_LOGGING_ENABLED
    // GCS NOTIFICATION: Log avoidance action to dataflash for pilot awareness and post-flight analysis
    // Only log on initial failsafe activation to avoid flooding logs with repeated messages
    // Error code contains the actual_action enum value for determining what maneuver was executed
    if (failsafe_state_change) {
        AP::logger().Write_Error(LogErrorSubsystem::FAILSAFE_ADSB,
                                 LogErrorCode(actual_action));
    }
#endif

    // Return the actually executed action (may differ from requested if action failed)
    // Caller uses this to determine if avoidance was successful and what action was taken
    return actual_action;
}

/**
 * @brief Handle recovery after ADS-B threat has cleared
 * 
 * @details Called when ADS-B threat is no longer present and vehicle should exit
 *          avoidance mode and return to normal operation. This function:
 *          
 *          1. Clears ADSB failsafe flag to indicate threat resolved
 *          2. Logs recovery event to dataflash
 *          3. Restores previous flight mode based on configured recovery action
 *          
 *          Recovery actions determine post-avoidance behavior:
 *          - REMAIN_IN_AVOID_ADSB: Stay in avoidance mode (loiter at current position)
 *          - RESUME_PREVIOUS_FLIGHTMODE: Return to mode active before threat detected
 *          - RTL: Navigate to home regardless of previous mode
 *          - RESUME_IF_AUTO_ELSE_LOITER: Resume AUTO mission if that was previous mode
 * 
 * @param[in] recovery_action Strategy for mode restoration after threat clears
 * 
 * @note Recovery only occurs if pilot has not manually changed mode during avoidance.
 *       This prevents unexpected mode changes if pilot has taken manual control.
 * 
 * @warning Mode restoration uses fallback logic (try requested → RTL → LAND) to ensure
 *          vehicle always ends in a safe state even if preferred mode unavailable
 * 
 * @see handle_avoidance() for initial threat response
 * @see set_mode_else_try_RTL_else_LAND() for fallback mode logic
 */
void AP_Avoidance_Copter::handle_recovery(RecoveryAction recovery_action)
{
    // THREAT CLEARED: Verify we are transitioning out of active ADSB failsafe
    if (copter.failsafe.adsb) {
        // Clear ADSB failsafe flag - threat no longer detected by avoidance system
        copter.failsafe.adsb = false;
        
        // GCS NOTIFICATION: Log threat resolution to dataflash for pilot awareness
        // This allows post-flight analysis of avoidance event duration and outcome
        LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_ADSB,
                           LogErrorCode::ERROR_RESOLVED);

        // MODE RESTORATION: Restore previous flight mode if pilot has not taken manual control
        // Only restore if current mode reason is still AVOIDANCE, indicating pilot has not
        // manually changed modes during avoidance maneuver
        if (copter.control_mode_reason == ModeReason::AVOIDANCE) {
            switch (recovery_action) {

            case RecoveryAction::REMAIN_IN_AVOID_ADSB:
                // Stay in AVOID_ADSB mode (which is guided mode behavior)
                // Vehicle will loiter at current position indefinitely
                // Useful when pilot wants to manually assess situation before resuming mission
                break;

            case RecoveryAction::RESUME_PREVIOUS_FLIGHTMODE:
                // Return to the mode that was active when threat was first detected
                // This allows seamless mission continuation after avoidance complete
                // Falls back to RTL then LAND if previous mode unavailable
                set_mode_else_try_RTL_else_LAND(prev_control_mode);
                break;

            case RecoveryAction::RTL:
                // Always return to launch regardless of previous mode
                // Conservative option that brings vehicle back to home after any threat
                set_mode_else_try_RTL_else_LAND(Mode::Number::RTL);
                break;

            case RecoveryAction::RESUME_IF_AUTO_ELSE_LOITER:
                // Resume AUTO mission only if that was previous mode, otherwise stay put
                // Prevents automatic mission resume in manual flight modes where pilot
                // may not be expecting automated mission continuation
                if (prev_control_mode == Mode::Number::AUTO) {
                    set_mode_else_try_RTL_else_LAND(Mode::Number::AUTO);
                }
                break;

            default:
                break;
            } // switch
        }
    }
}

/**
 * @brief Attempt mode change with RTL and LAND fallbacks for safety
 * 
 * @details Implements cascading fallback logic to ensure vehicle always ends in a safe
 *          state during avoidance recovery. Attempts mode changes in priority order:
 *          1. Requested mode (e.g., AUTO, LOITER, previous mode)
 *          2. RTL if requested mode fails (navigate home)
 *          3. LAND if RTL fails (land at current position)
 *          
 *          Mode changes can fail due to:
 *          - GPS position unavailable (required for RTL, AUTO)
 *          - Mode not configured or compiled in
 *          - Pre-arm checks fail
 *          - Hardware requirements not met
 * 
 * @param[in] mode Desired flight mode to enter
 * 
 * @note LAND mode should never fail as it requires minimal prerequisites and will
 *       execute even without GPS position using altitude hold + descent
 * 
 * @warning This function always succeeds in setting some flight mode. If all modes fail
 *          (extremely unlikely), vehicle remains in current mode.
 */
void AP_Avoidance_Copter::set_mode_else_try_RTL_else_LAND(Mode::Number mode)
{
    // Attempt to set requested mode with AVOIDANCE_RECOVERY reason for logging
    if (!copter.set_mode(mode, ModeReason::AVOIDANCE_RECOVERY)) {
        // FALLBACK CHAIN: If requested mode fails, try RTL (return home)
        // If RTL also fails (e.g., no GPS), fall back to LAND at current position
        if (!copter.set_mode(Mode::Number::RTL, ModeReason::AVOIDANCE_RECOVERY)) {
            copter.set_mode(Mode::Number::LAND, ModeReason::AVOIDANCE_RECOVERY);
        }
    }
}

/**
 * @brief Get minimum altitude for avoidance maneuvers
 * 
 * @details Returns the minimum altitude below which the vehicle should not descend
 *          during avoidance maneuvers. This prevents downward evasion from forcing
 *          the vehicle below safe operating altitude or into ground/obstacles.
 *          
 *          Uses RTL altitude parameter as minimum since this represents the pilot's
 *          configured safe altitude for navigation and obstacle clearance.
 * 
 * @return Minimum altitude in centimeters above home
 * 
 * @note Returns 0 if RTL mode not compiled in, allowing descent to ground level
 *       (this configuration is rare and typically only used for specialized builds)
 */
int32_t AP_Avoidance_Copter::get_altitude_minimum() const
{
#if MODE_RTL_ENABLED
    // Use RTL altitude parameter as safe minimum altitude floor
    // This ensures avoidance descents don't go below pilot's configured safe altitude
    return copter.g.rtl_altitude;
#else
    return 0;
#endif
}

/**
 * @brief Verify vehicle is in AVOID_ADSB mode for executing avoidance maneuvers
 * 
 * @details Checks if vehicle is in AVOID_ADSB flight mode, which is required for
 *          velocity-based evasion maneuvers. AVOID_ADSB mode is a guided mode that
 *          accepts velocity commands, allowing precise control of evasion vectors.
 *          
 *          If mode change is allowed and vehicle is not already in AVOID_ADSB,
 *          attempts to switch to AVOID_ADSB mode automatically.
 * 
 * @param[in] allow_mode_change If true, automatically switch to AVOID_ADSB mode if needed
 *                              If false, only check current mode without changing
 * 
 * @return true if vehicle is in AVOID_ADSB mode, false otherwise
 * 
 * @note AVOID_ADSB mode switch can fail if:
 *       - GPS position unavailable (mode requires position estimate)
 *       - EKF not healthy
 *       - Pre-arm checks fail
 */
bool AP_Avoidance_Copter::check_flightmode(bool allow_mode_change)
{
    // Attempt automatic mode transition to AVOID_ADSB if permitted
    if (allow_mode_change && copter.flightmode->mode_number() != Mode::Number::AVOID_ADSB) {
        if (!copter.set_mode(Mode::Number::AVOID_ADSB, ModeReason::AVOIDANCE)) {
            // Mode change failed - cannot execute velocity-based avoidance maneuvers
            return false;
        }
    }

    // Verify vehicle is in AVOID_ADSB mode for velocity command acceptance
    return (copter.flightmode->mode_number() == Mode::Number::AVOID_ADSB);
}

/**
 * @brief Execute vertical avoidance maneuver (climb or descend)
 * 
 * @details Implements vertical evasion by commanding upward or downward velocity to
 *          create vertical separation from obstacle. Direction selection based on
 *          relative altitude comparison:
 *          
 *          **Threat Assessment Logic:**
 *          - Compare vehicle altitude to obstacle altitude
 *          - If vehicle is higher: descend to pass below obstacle
 *          - If vehicle is lower: climb to pass above obstacle
 *          
 *          **Safety Constraints:**
 *          - Descent blocked if below minimum safe altitude (RTL_ALT parameter)
 *          - Uses configured waypoint climb/descent speeds for smooth maneuvering
 *          - Maintains horizontal position during vertical maneuver
 *          
 *          **Velocity Vector Calculation:**
 *          - X, Y components: 0 (no horizontal movement)
 *          - Z component: ±default vertical speed (NED frame: positive = down)
 * 
 * @param[in] obstacle Pointer to detected obstacle with position and velocity data
 * @param[in] allow_mode_change If true, switch to AVOID_ADSB mode if not already active
 * 
 * @return true if avoidance maneuver successfully commanded, false if mode switch failed
 * 
 * @note Called at avoidance update rate (typically 10Hz) while vertical threat persists
 * 
 * @warning Vertical maneuvers maintain horizontal position - ensure no horizontal obstacles present
 */
bool AP_Avoidance_Copter::handle_avoidance_vertical(const AP_Avoidance::Obstacle *obstacle, bool allow_mode_change)
{
    // Ensure vehicle is in AVOID_ADSB mode for velocity control
    if (!check_flightmode(allow_mode_change)) {
        return false;
    }

    // THREAT GEOMETRY ANALYSIS: Determine climb vs descend based on relative altitude
    // Default to climb if position unavailable (safer to go up in uncertainty)
    bool should_climb = false;
    Location my_loc;
    if (AP::ahrs().get_location(my_loc)) {
        // Compare altitudes: if we're higher than obstacle, descend to pass below
        should_climb = my_loc.alt > obstacle->_location.alt;
    }

    // EVASION VECTOR CALCULATION: Build velocity command in NED frame
    Vector3f velocity_neu;
    if (should_climb) {
        // CLIMB: Use configured waypoint climb speed (positive in NED = up)
        velocity_neu.z = copter.wp_nav->get_default_speed_up_cms();
    } else {
        // DESCEND: Use configured waypoint descent speed (negative in NED = down)
        velocity_neu.z = -copter.wp_nav->get_default_speed_down_cms();
        
        // ALTITUDE FLOOR: Prevent descent below minimum safe altitude
        // This protects against driving vehicle into ground or obstacles below
        if (copter.current_loc.alt < get_altitude_minimum()) {
            velocity_neu.z = 0.0f;
        }
    }

    // MANEUVER EXECUTION: Send vertical velocity command to AVOID_ADSB mode
    // Mode will continuously track this velocity until threat clears or action changes
    copter.mode_avoid_adsb.set_velocity(velocity_neu);
    return true;
}

/**
 * @brief Execute horizontal avoidance maneuver (move perpendicular to threat in 2D plane)
 * 
 * @details Implements horizontal-only evasion by calculating 2D perpendicular vector
 *          to obstacle approach direction and commanding lateral velocity. This maneuver:
 *          
 *          **Threat Vector Calculation:**
 *          - Calls get_vector_perpendicular() to compute 3D perpendicular evasion direction
 *          - Projects result onto horizontal plane by zeroing Z component
 *          - Normalizes 2D vector to unit length
 *          - Scales by configured waypoint horizontal speed
 *          
 *          **Movement Strategy:**
 *          - Creates lateral separation while maintaining altitude
 *          - Direction is perpendicular to obstacle velocity vector
 *          - Magnitude uses configured waypoint navigation speed
 *          
 *          **Safety Validation:**
 *          - Checks for zero horizontal vector (would indicate obstacle directly above/below)
 *          - Verifies valid perpendicular vector exists before commanding motion
 * 
 * @param[in] obstacle Pointer to detected obstacle with position and velocity data
 * @param[in] allow_mode_change If true, switch to AVOID_ADSB mode if not already active
 * 
 * @return true if horizontal avoidance velocity successfully commanded, false if failed
 * 
 * @note Maintains current altitude - appropriate when vertical separation not possible/desired
 * 
 * @warning Horizontal-only maneuver may be insufficient for high closure rate threats
 *          requiring immediate 3D separation
 * 
 * @see get_vector_perpendicular() in AP_Avoidance parent class for vector calculation
 */
bool AP_Avoidance_Copter::handle_avoidance_horizontal(const AP_Avoidance::Obstacle *obstacle, bool allow_mode_change)
{
    // Ensure vehicle is in AVOID_ADSB mode for velocity control
    if (!check_flightmode(allow_mode_change)) {
        return false;
    }

    // EVASION VECTOR CALCULATION: Get perpendicular direction to obstacle velocity
    Vector3f velocity_neu;
    if (get_vector_perpendicular(obstacle, velocity_neu)) {
        // PROJECT TO HORIZONTAL PLANE: Remove vertical component for 2D-only evasion
        velocity_neu.z = 0.0f;
        
        // VALIDATE HORIZONTAL VECTOR: Check for degenerate case (obstacle directly above/below)
        // If both horizontal components zero, no lateral evasion direction exists
        if (is_zero(velocity_neu.x) && is_zero(velocity_neu.y)) {
            return false;
        }
        
        // NORMALIZE: Convert to unit vector before applying speed scaling
        velocity_neu.normalize();
        
        // VELOCITY SCALING: Apply configured waypoint horizontal speed to evasion direction
        // Both North (X) and East (Y) components scaled by same speed for consistent velocity magnitude
        velocity_neu.x *= copter.wp_nav->get_default_speed_NE_cms();
        velocity_neu.y *= copter.wp_nav->get_default_speed_NE_cms();
        
        // MANEUVER EXECUTION: Command horizontal velocity while maintaining altitude
        copter.mode_avoid_adsb.set_velocity(velocity_neu);
        return true;
    }

    // FAILURE: Could not calculate valid perpendicular vector from obstacle data
    return false;
}

/**
 * @brief Execute 3D perpendicular avoidance maneuver (move perpendicular to threat in all axes)
 * 
 * @details Implements full 3D evasion by calculating perpendicular vector to obstacle's
 *          velocity vector and commanding motion in all three dimensions. This provides
 *          maximum separation by utilizing both horizontal and vertical movement:
 *          
 *          **Threat Vector Analysis:**
 *          - Calculates perpendicular vector to obstacle velocity in 3D space
 *          - Perpendicular direction provides fastest increase in separation distance
 *          - Uses cross product with relative position to determine evasion direction
 *          
 *          **Velocity Component Scaling:**
 *          - Horizontal (X, Y): Scaled by waypoint horizontal speed (NE speed)
 *          - Vertical (Z): Scaled by waypoint climb or descent speed based on direction
 *          - Different vertical speeds for up vs down to match vehicle performance
 *          
 *          **Safety Constraints:**
 *          - Prevents descent below minimum safe altitude (RTL_ALT)
 *          - Maintains proper speed limits for vehicle capabilities
 *          - Altitude floor overrides downward component if at minimum altitude
 * 
 * @param[in] obstacle Pointer to detected obstacle with position and velocity data
 * @param[in] allow_mode_change If true, switch to AVOID_ADSB mode if not already active
 * 
 * @return true if 3D avoidance velocity successfully commanded, false if failed
 * 
 * @note This is the most aggressive avoidance strategy, utilizing full 3D maneuvering
 *       capability of multicopter to maximize separation rate from threat
 * 
 * @warning Combines horizontal and vertical motion - ensure vehicle has sufficient
 *          power/performance margins to execute combined maneuver safely
 * 
 * @see get_vector_perpendicular() in AP_Avoidance parent class for 3D vector calculation
 * @see handle_avoidance_horizontal() for 2D-only alternative
 * @see handle_avoidance_vertical() for vertical-only alternative
 */
bool AP_Avoidance_Copter::handle_avoidance_perpendicular(const AP_Avoidance::Obstacle *obstacle, bool allow_mode_change)
{
    // Ensure vehicle is in AVOID_ADSB mode for 3D velocity control
    if (!check_flightmode(allow_mode_change)) {
        return false;
    }

    // EVASION VECTOR CALCULATION: Get 3D perpendicular direction to obstacle velocity
    // This vector represents the direction of maximum separation rate from threat
    Vector3f velocity_neu;
    if (get_vector_perpendicular(obstacle, velocity_neu)) {
        // HORIZONTAL VELOCITY SCALING: Apply waypoint horizontal speed to X and Y components
        // Scales normalized perpendicular vector by configured speed for lateral motion
        velocity_neu.x *= copter.wp_nav->get_default_speed_NE_cms();
        velocity_neu.y *= copter.wp_nav->get_default_speed_NE_cms();
        
        // VERTICAL VELOCITY SCALING: Apply asymmetric climb/descent speeds
        // Most multirotors climb slower than they descend, so use separate speed parameters
        if (velocity_neu.z > 0.0f) {
            // CLIMB: Positive Z in NED frame means upward motion
            velocity_neu.z *= copter.wp_nav->get_default_speed_up_cms();
        } else {
            // DESCEND: Negative Z in NED frame means downward motion
            velocity_neu.z *= copter.wp_nav->get_default_speed_down_cms();
            
            // ALTITUDE FLOOR: Prevent descent below minimum safe altitude
            // Overrides perpendicular vector downward component to ensure ground clearance
            if (copter.current_loc.alt < get_altitude_minimum()) {
                velocity_neu.z = 0.0f;
            }
        }
        
        // MANEUVER EXECUTION: Command full 3D velocity for maximum evasion effectiveness
        // Vehicle will track this velocity continuously until threat clears
        copter.mode_avoid_adsb.set_velocity(velocity_neu);
        return true;
    }

    // FAILURE: Could not calculate valid perpendicular vector from obstacle data
    // This can occur if obstacle velocity is zero or data is invalid
    return false;
}

#endif  // AP_ADSB_AVOIDANCE_ENABLED
