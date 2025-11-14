/**
 * @file avoidance_adsb.cpp
 * @brief ADS-B based collision avoidance implementation for ArduPlane
 * 
 * @details This file implements collision avoidance using ADS-B (Automatic Dependent 
 *          Surveillance-Broadcast) transponder data to detect and avoid potential 
 *          mid-air collisions with other aircraft. The system monitors ADS-B traffic,
 *          calculates threat levels, and executes avoidance maneuvers when necessary.
 * 
 *          Key capabilities:
 *          - Continuous monitoring of ADS-B traffic for potential threats
 *          - Threat assessment based on distance, altitude, and closure rate
 *          - Automated avoidance maneuvers (vertical, horizontal, or combined)
 *          - Multiple avoidance action types (RTL, loiter, guided waypoint)
 *          - Failsafe state management during avoidance
 *          - Recovery to previous flight mode after threat passes
 * 
 *          The implementation is conditionally compiled based on:
 *          - HAL_ADSB_ENABLED: Basic ADS-B receiver support
 *          - AP_ADSB_AVOIDANCE_ENABLED: Full collision avoidance logic
 * 
 *          Flight mode restrictions:
 *          - No avoidance action taken in Manual mode
 *          - No avoidance during takeoff in Auto mode
 *          - No avoidance during landing approach
 *          - No avoidance in Autotune mode
 *          - No avoidance in QLAND mode (quadplane landing)
 * 
 * @warning This is safety-critical code. Any modifications must be thoroughly tested
 *          in SITL and on hardware before deployment. Incorrect avoidance maneuvers
 *          could result in loss of aircraft control or actual collision.
 * 
 * @note The avoidance system requires a functioning ADS-B receiver and properly
 *       configured avoidance parameters (ADSB_*, AVD_* parameter groups).
 * 
 * @see AP_Avoidance library for core threat detection algorithms
 * @see libraries/AP_ADSB/ for ADS-B receiver interface
 * 
 * Source: ArduPlane/avoidance_adsb.cpp
 */

#include <stdio.h>
#include "Plane.h"

#if HAL_ADSB_ENABLED || AP_ADSB_AVOIDANCE_ENABLED
/**
 * @brief Update ADS-B receiver and collision avoidance system
 * 
 * @details This function is called periodically from the main scheduler to update
 *          the ADS-B subsystem and collision avoidance logic. It performs:
 *          
 *          1. ADS-B receiver update (if HAL_ADSB_ENABLED):
 *             - Processes incoming ADS-B messages from the transponder
 *             - Updates the vehicle database with latest traffic information
 *             - Maintains list of nearby aircraft with position/velocity data
 *          
 *          2. Avoidance logic update (if AP_ADSB_AVOIDANCE_ENABLED):
 *             - Evaluates all tracked aircraft for collision threats
 *             - Calculates threat levels based on distance, closure rate, and altitude
 *             - Triggers avoidance maneuvers when threat threshold exceeded
 *             - Monitors for threat resolution to initiate recovery
 * 
 * @note This function is called at the scheduler rate (typically 50-400Hz depending
 *       on board configuration). The actual threat calculations are rate-limited
 *       internally to reduce CPU load.
 * 
 * @note Conditional compilation: Only compiled if ADS-B hardware support or
 *       avoidance features are enabled in the build.
 * 
 * @warning Must be called regularly to ensure timely threat detection. Missing
 *          updates could delay avoidance response time.
 * 
 * @see AP_ADSB::update() for receiver update details
 * @see AP_Avoidance::update() for threat calculation details
 * 
 * Source: ArduPlane/avoidance_adsb.cpp:6-14
 */
void Plane::avoidance_adsb_update(void)
{
#if HAL_ADSB_ENABLED
    adsb.update();  // Update ADS-B receiver and vehicle database
#endif  // HAL_ADSB_ENABLED
#if AP_ADSB_AVOIDANCE_ENABLED
    avoidance_adsb.update();  // Update collision avoidance threat calculations
#endif  // AP_ADSB_AVOIDANCE_ENABLED
}
#endif  // HAL_ADSB_ENABLED || AP_ADSB_AVOIDANCE_ENABLED

#if AP_ADSB_AVOIDANCE_ENABLED
/**
 * @brief Execute collision avoidance maneuver for detected threat
 * 
 * @details This is the primary avoidance action handler called when a collision
 *          threat is detected. It implements the following decision logic:
 * 
 *          1. Failsafe State Management:
 *             - Activates ADSB failsafe on first threat detection
 *             - Records current flight mode for potential recovery
 *             - Maintains failsafe state until threat passes
 * 
 *          2. Flight Mode Restrictions:
 *             - Manual mode: No automated action (pilot maintains control)
 *             - Auto mode during takeoff: No action (focus on safe takeoff)
 *             - Landing phase: No action (complete landing safely)
 *             - Autotune mode: No action (avoid disrupting tuning)
 *             - QLAND mode: No action (quadplane landing in progress)
 * 
 *          3. Avoidance Actions:
 *             - RTL: Switch to Return-To-Launch mode
 *             - HOVER: Loiter in place (or QLOITER for flying quadplanes)
 *             - ASCEND_OR_DESCEND: Vertical avoidance maneuver
 *             - MOVE_HORIZONTALLY: Horizontal avoidance maneuver
 *             - MOVE_PERPENDICULAR: Combined vertical and horizontal avoidance
 *             - NONE/REPORT: No automated action taken
 * 
 *          4. Guided Waypoint Actions:
 *             - Vertical/horizontal actions calculate new target location
 *             - Aircraft commanded to new waypoint via set_guided_WP()
 *             - Actions fall back to NONE if waypoint calculation fails
 * 
 * @param[in] obstacle Pointer to detected obstacle with position, velocity, and
 *                     threat level information. Must not be NULL.
 * @param[in] requested_action The collision action recommended by the avoidance
 *                             library based on threat geometry and closure rate
 * 
 * @return MAV_COLLISION_ACTION The actual action taken, which may differ from
 *                               requested_action if flight mode prohibits action
 *                               or if maneuver calculation fails
 * 
 * @note This function may change the vehicle flight mode to AVOID_ADSB, RTL,
 *       LOITER, or QLOITER depending on the requested action and vehicle state.
 * 
 * @warning Safety-critical function. Actions must balance collision avoidance
 *          with maintaining safe flight:
 *          - Do not interrupt critical flight phases (takeoff, landing)
 *          - Ensure sufficient altitude before descending
 *          - Verify guided waypoints are achievable
 *          - Maintain pilot override capability in manual modes
 * 
 * @warning Avoidance maneuvers may be aggressive. Ensure airframe can handle
 *          maximum climb rates (TECS_CLMB_MAX) and turn rates.
 * 
 * @see handle_avoidance_vertical() for vertical maneuver calculation
 * @see handle_avoidance_horizontal() for horizontal maneuver calculation
 * @see handle_recovery() for threat resolution and mode restoration
 * 
 * Source: ArduPlane/avoidance_adsb.cpp:18-117
 */
MAV_COLLISION_ACTION AP_Avoidance_Plane::handle_avoidance(const AP_Avoidance::Obstacle *obstacle, MAV_COLLISION_ACTION requested_action)
{
    MAV_COLLISION_ACTION actual_action = requested_action;
    bool failsafe_state_change = false;

    // Check for failsafe activation (first detection of this threat)
    // Only transitions from false->true once per threat event
    if (!plane.failsafe.adsb) {
        plane.failsafe.adsb = true;
        failsafe_state_change = true;
        // Record current flight mode for potential restoration after threat passes
        // This allows recovery logic to return to AUTO/CRUISE/etc after avoidance
        prev_control_mode_number = plane.control_mode->mode_number();
    }

    // Determine if current flight mode prohibits automated avoidance action
    // Certain critical flight phases must not be interrupted by avoidance maneuvers
    bool flightmode_prohibits_action = false;
    if (plane.control_mode == &plane.mode_manual ||  // Pilot has full control, don't override
        (plane.control_mode == &plane.mode_auto && !plane.auto_state.takeoff_complete) ||  // Takeoff phase is critical, complete it first
        (plane.flight_stage == AP_FixedWing::FlightStage::LAND) ||  // Landing phase is critical, complete it safely
        plane.control_mode == &plane.mode_autotune) {  // Don't disrupt PID tuning process
        flightmode_prohibits_action = true;
    }
#if HAL_QUADPLANE_ENABLED
    // Quadplane landing is also a critical phase that should not be interrupted
    if (plane.control_mode == &plane.mode_qland) {
        flightmode_prohibits_action = true;
    }
#endif
    // Override requested action if flight mode prohibits automated response
    if (flightmode_prohibits_action) {
        actual_action = MAV_COLLISION_ACTION_NONE;
    }

    // Execute avoidance action based on threat geometry and requested response
    // Only change mode on initial failsafe activation to avoid repeated mode switches
    switch (actual_action) {

        case MAV_COLLISION_ACTION_RTL:
            // Return to launch - fly back to home position to avoid threat area
            // Typically used when threat is near current flight path
            if (failsafe_state_change) {
                plane.set_mode(plane.mode_rtl, ModeReason::AVOIDANCE);
            }
            break;

        case MAV_COLLISION_ACTION_HOVER:
            // Loiter in place - hold current position while threat passes
            // For quadplanes in VTOL flight, use QLOITER for hover capability
            if (failsafe_state_change) {
#if HAL_QUADPLANE_ENABLED
                if (plane.quadplane.is_flying()) {
                    plane.set_mode(plane.mode_qloiter, ModeReason::AVOIDANCE);
                    break;
                }
#endif
                // Fixed-wing loiter - circle at current location
                plane.set_mode(plane.mode_loiter, ModeReason::AVOIDANCE);
            }
            break;

        case MAV_COLLISION_ACTION_ASCEND_OR_DESCEND: {
            // Vertical avoidance - climb or descend to avoid threat altitude
            // Starts from next waypoint location and modifies altitude only
            Location loc = plane.next_WP_loc;
            if (handle_avoidance_vertical(obstacle, failsafe_state_change, loc)) {
                // Successfully calculated vertical avoidance waypoint
                plane.set_guided_WP(loc);
            } else {
                // Vertical avoidance not possible (e.g., too low to descend safely)
                actual_action = MAV_COLLISION_ACTION_NONE;
            }
            break;
        }
        case MAV_COLLISION_ACTION_MOVE_HORIZONTALLY: {
            // Horizontal avoidance - move perpendicular to threat vector
            // Calculates a waypoint offset from current path to avoid collision
            Location loc = plane.next_WP_loc;
            if (handle_avoidance_horizontal(obstacle, failsafe_state_change, loc)) {
                // Successfully calculated horizontal avoidance waypoint
                plane.set_guided_WP(loc);
            } else {
                // Horizontal avoidance calculation failed (e.g., no valid perpendicular vector)
                actual_action = MAV_COLLISION_ACTION_NONE;
            }
            break;
        }
        case MAV_COLLISION_ACTION_MOVE_PERPENDICULAR:
        {
            // Combined 3D avoidance - modify both horizontal position and altitude
            // Attempts both vertical and horizontal avoidance simultaneously
            // Success if either component succeeds (partial avoidance better than none)
            Location loc = plane.next_WP_loc;
            const bool success_vert = handle_avoidance_vertical(obstacle, failsafe_state_change, loc);
            const bool success_hor = handle_avoidance_horizontal(obstacle, failsafe_state_change, loc);
            if (success_vert || success_hor) {
                // At least one avoidance component succeeded, use the modified waypoint
                plane.set_guided_WP(loc);
            } else {
                // Both avoidance calculations failed, take no action
                actual_action = MAV_COLLISION_ACTION_NONE;
            }
        }
            break;

        // Actions that require no automated response
        case MAV_COLLISION_ACTION_NONE:
            // Explicitly no action - continue current flight plan
            return actual_action;
        case MAV_COLLISION_ACTION_REPORT:
            // Report only - log threat but don't change flight path
            // GCS message sent below for awareness
        default:
            // Unknown action type - take no automated action
            break;
    }

    // Send GCS alert message when avoidance action is first initiated
    // Provides pilot awareness of automated avoidance response
    if (failsafe_state_change) {
        gcs().send_text(MAV_SEVERITY_ALERT, "Avoid: Performing action: %d", actual_action);
    }

    // Return the actual action taken (may differ from requested if prohibited)
    return actual_action;
}

/**
 * @brief Handle recovery from collision avoidance when threat has passed
 * 
 * @details This function is called by the avoidance library when a previously
 *          detected threat has cleared and the aircraft can resume normal operations.
 *          It manages the transition from avoidance maneuver back to nominal flight.
 * 
 *          Recovery behavior depends on the configured recovery action:
 * 
 *          REMAIN_IN_AVOID_ADSB:
 *          - Stay in AVOID_ADSB mode (guided loiter at avoidance position)
 *          - Useful when expecting additional nearby threats
 * 
 *          RESUME_PREVIOUS_FLIGHTMODE:
 *          - Return to the flight mode active before avoidance was triggered
 *          - Restores AUTO mission, CRUISE, etc. to continue original plan
 *          - Most common recovery option
 * 
 *          RTL:
 *          - Switch to Return-To-Launch mode
 *          - Conservative option when unsure of threat status
 * 
 *          RESUME_IF_AUTO_ELSE_LOITER:
 *          - If previous mode was AUTO, resume the mission
 *          - Otherwise, loiter at current position (modified to use current_loc)
 *          - Prevents automatic mission resumption in manual flight modes
 * 
 * @param[in] recovery_action The recovery behavior to execute after threat clears
 * 
 * @note Recovery only occurs if the mode change reason is still AVOIDANCE,
 *       meaning the pilot has not manually changed modes during the avoidance.
 *       This preserves pilot override capability.
 * 
 * @note The ADSB failsafe flag is cleared immediately when this function is called,
 *       allowing new threats to trigger fresh avoidance if needed.
 * 
 * @warning If recovery action is invalid, the aircraft will loiter at current
 *          position rather than attempting an undefined recovery behavior.
 * 
 * @see handle_avoidance() for threat detection and initial avoidance response
 * 
 * Source: ArduPlane/avoidance_adsb.cpp:119-161
 */
void AP_Avoidance_Plane::handle_recovery(RecoveryAction recovery_action)
{
    // Verify we are actually in ADSB failsafe state before attempting recovery
    // This function should only be called when transitioning out of failsafe
    if (plane.failsafe.adsb) {
        plane.failsafe.adsb = false;  // Clear failsafe flag - threat has passed
        gcs().send_text(MAV_SEVERITY_INFO, "Avoid: Resuming with action: %u", (unsigned)recovery_action);

        // Only restore flight mode if mode change reason is still AVOIDANCE
        // If pilot manually changed modes during avoidance, respect that choice
        if (plane.control_mode_reason == ModeReason::AVOIDANCE) {
            switch (recovery_action) {

            case RecoveryAction::REMAIN_IN_AVOID_ADSB:
                // Stay in AVOID_ADSB mode (guided loiter)
                // No action needed - mode already set during avoidance
                // Aircraft will continue to loiter at avoidance position
                break;

            case RecoveryAction::RESUME_PREVIOUS_FLIGHTMODE:
                // Return to the flight mode that was active before avoidance
                // This restores AUTO missions, CRUISE navigation, etc.
                plane.set_mode_by_number(prev_control_mode_number, ModeReason::AVOIDANCE_RECOVERY);
                break;

            case RecoveryAction::RTL:
                // Conservative recovery - return to home position
                // Ensures aircraft moves to known safe location
                plane.set_mode(plane.mode_rtl, ModeReason::AVOIDANCE_RECOVERY);
                break;

            case RecoveryAction::RESUME_IF_AUTO_ELSE_LOITER:
                // Conditional recovery based on previous mode
                if (prev_control_mode_number == Mode::Number::AUTO) {
                    // Was running a mission - safe to resume it
                    plane.set_mode(plane.mode_auto, ModeReason::AVOIDANCE_RECOVERY);
                } else {
                    // Was in manual flight mode - don't auto-resume
                    // Stay in guided mode but reset loiter point to current position
                    // rather than the avoidance maneuver position
                    plane.set_guided_WP(plane.current_loc);
                }
                break;

            default:
                // Invalid recovery action specified in parameters
                // Safest action is to loiter at current position
                plane.set_guided_WP(plane.current_loc);
                break;
            } // switch
        }
    }
}

/**
 * @brief Verify aircraft is in AVOID_ADSB mode for guided avoidance maneuvers
 * 
 * @details This helper function ensures the aircraft is in the correct flight mode
 *          before executing guided waypoint avoidance maneuvers (vertical, horizontal,
 *          or perpendicular actions). The AVOID_ADSB mode is a guided mode variant
 *          that accepts waypoint commands while maintaining avoidance awareness.
 * 
 *          If allow_mode_change is true and the aircraft is not already in AVOID_ADSB
 *          mode, this function will automatically switch to it. This is typically done
 *          on the first avoidance action. Subsequent calls during the same avoidance
 *          event use allow_mode_change=false to verify mode without switching.
 * 
 * @param[in] allow_mode_change If true, automatically switch to AVOID_ADSB mode if
 *                              not already in it. If false, only check current mode.
 * 
 * @return true if aircraft is in AVOID_ADSB mode (or successfully switched to it)
 * @return false if aircraft is not in AVOID_ADSB mode and allow_mode_change is false
 * 
 * @note AVOID_ADSB mode is required for guided waypoint commands used in vertical
 *       and horizontal avoidance maneuvers. RTL and LOITER actions do not require
 *       this mode check.
 * 
 * @see handle_avoidance_vertical() for vertical avoidance requiring this mode
 * @see handle_avoidance_horizontal() for horizontal avoidance requiring this mode
 * 
 * Source: ArduPlane/avoidance_adsb.cpp:164-173
 */
bool AP_Avoidance_Plane::check_flightmode(bool allow_mode_change)
{
    // Switch to AVOID_ADSB mode if permitted and not already in it
    if (allow_mode_change && plane.control_mode != &plane.mode_avoidADSB) {
        plane.set_mode(plane.mode_avoidADSB, ModeReason::AVOIDANCE);
    }

    // Return true if now in AVOID_ADSB mode
    return (plane.control_mode == &plane.mode_avoidADSB);
}

/**
 * @brief Calculate vertical avoidance maneuver to avoid collision threat
 * 
 * @details This function calculates an altitude change to avoid a detected obstacle
 *          by moving vertically away from the threat altitude. The decision logic:
 * 
 *          If our altitude > threat altitude:
 *          - CLIMB: Set target 10m above current altitude
 *          - Climb rate limited by TECS_CLMB_MAX parameter
 *          - Increases vertical separation from threat below us
 * 
 *          If our altitude < threat altitude AND we're above RTL altitude:
 *          - DESCEND: Set target 10m below current altitude
 *          - Descend rate limited by TECS_SINK_MAX parameter
 *          - Only descend if sufficient altitude margin exists
 * 
 *          If our altitude < threat altitude AND we're below RTL altitude:
 *          - NO ACTION: Too low to safely descend
 *          - Avoidance must use horizontal maneuver instead
 * 
 * @param[in]     obstacle Pointer to detected obstacle with position and velocity data
 * @param[in]     allow_mode_change If true, switch to AVOID_ADSB mode if needed
 * @param[in,out] new_loc Location to modify with new target altitude. Input provides
 *                        starting position (typically next_WP_loc), output contains
 *                        altitude-modified target. Latitude/longitude unchanged.
 * 
 * @return true if vertical avoidance maneuver successfully calculated
 * @return false if vertical maneuver not possible (wrong mode, insufficient altitude)
 * 
 * @note Altitude reference frame is ABSOLUTE (above MSL), not relative to home
 * @note 10m vertical offset chosen as compromise between separation and achievability
 * @note Comment mentions "copter" but this is actually plane-specific code
 * 
 * @warning Always check return value. false indicates no valid vertical maneuver
 *          exists and horizontal avoidance or different action should be used.
 * 
 * @warning RTL_altitude provides minimum safe descent altitude. Do not descend below
 *          this to avoid terrain and obstacles near ground level.
 * 
 * @todo Consider using lower threshold than RTL_altitude for descent permission
 *       since RTL_altitude default (100m) is quite high for avoidance purposes
 * 
 * @see handle_avoidance_horizontal() for horizontal avoidance alternative
 * @see AP_TECS for climb/sink rate limiting details
 * 
 * Source: ArduPlane/avoidance_adsb.cpp:175-196
 */
bool AP_Avoidance_Plane::handle_avoidance_vertical(const AP_Avoidance::Obstacle *obstacle, bool allow_mode_change, Location &new_loc)
{
    // Ensure aircraft is in AVOID_ADSB mode for guided waypoint command
    if (!check_flightmode(allow_mode_change)) {
        return false;  // Mode check failed, cannot execute guided maneuver
    }

    // Calculate best vertical separation strategy based on relative altitudes
    if (plane.current_loc.alt > obstacle->_location.alt) {
        // Threat is below us - CLIMB to increase separation
        // Target 10m above current altitude (1000 centimeters)
        // Climb rate automatically limited by TECS to TECS_CLMB_MAX
        new_loc.set_alt_cm(plane.current_loc.alt + 1000, Location::AltFrame::ABSOLUTE);
        return true;

    } else if (plane.current_loc.alt > plane.g.RTL_altitude*100) {
        // Threat is above us and we have sufficient altitude margin - DESCEND
        // RTL_altitude in meters, convert to centimeters (*100)
        // Only descend if above safe minimum altitude
        // TODO: consider using a lower altitude than RTL_altitude since its default (100m) is quite high
        new_loc.set_alt_cm(plane.current_loc.alt - 1000, Location::AltFrame::ABSOLUTE);
        return true;
    }

    // Cannot safely descend (too low) and threat is above us
    // Vertical avoidance not possible - must use horizontal avoidance
    return false;
}

/**
 * @brief Calculate horizontal avoidance maneuver to avoid collision threat
 * 
 * @details This function calculates a horizontal position offset to avoid a detected
 *          obstacle by moving perpendicular to the threat vector. The algorithm:
 * 
 *          1. Calculate perpendicular vector away from obstacle path
 *             - Uses get_vector_perpendicular() from AP_Avoidance base class
 *             - Returns 3D vector in NED (North-East-Down) frame
 * 
 *          2. Remove vertical component (project to horizontal plane)
 *             - Sets Z component to zero
 *             - Avoidance is purely horizontal position change
 * 
 *          3. Normalize and scale the avoidance vector
 *             - Re-normalize after removing vertical component
 *             - Scale by 10,000 (100m offset in centimeters)
 *             - Provides sufficient separation from threat path
 * 
 *          4. Apply offset to create new target location
 *             - Offset from input location (typically next_WP_loc)
 *             - Creates guided waypoint perpendicular to threat
 * 
 * @param[in]     obstacle Pointer to detected obstacle with position, velocity, and
 *                         threat vector information
 * @param[in]     allow_mode_change If true, switch to AVOID_ADSB mode if needed
 * @param[in,out] new_loc Location to modify with horizontal offset. Input provides
 *                        starting position, output contains horizontally-offset target.
 *                        Altitude remains unchanged.
 * 
 * @return true if horizontal avoidance maneuver successfully calculated
 * @return false if perpendicular vector calculation failed or resulted in zero vector
 * 
 * @note Avoidance offset is 100m (10,000cm) perpendicular to threat vector
 * @note Altitude is preserved - only latitude/longitude are modified
 * @note Coordinate frame is NED (North-East-Down) earth frame
 * 
 * @warning Always check return value. false indicates no valid perpendicular vector
 *          could be calculated (e.g., threat directly above/below).
 * 
 * @warning 100m offset may not be sufficient for high-speed threats. Future
 *          enhancement could scale offset based on closure rate.
 * 
 * @see get_vector_perpendicular() in AP_Avoidance base class for vector calculation
 * @see handle_avoidance_vertical() for altitude-based avoidance alternative
 * 
 * Source: ArduPlane/avoidance_adsb.cpp:198-229
 */
bool AP_Avoidance_Plane::handle_avoidance_horizontal(const AP_Avoidance::Obstacle *obstacle, bool allow_mode_change, Location &new_loc)
{
    // Ensure aircraft is in AVOID_ADSB mode for guided waypoint command
    if (!check_flightmode(allow_mode_change)) {
        return false;  // Mode check failed, cannot execute guided maneuver
    }

    // Calculate perpendicular vector away from obstacle threat path
    Vector3f velocity_neu;  // Velocity in NED (North-East-Down) frame
    if (get_vector_perpendicular(obstacle, velocity_neu)) {
        // Project vector to horizontal plane by removing vertical component
        velocity_neu.z = 0.0f;

        // Check for degenerate case where horizontal components are zero
        // This can occur if threat is directly above/below with no horizontal velocity
        if (is_zero(velocity_neu.x) && is_zero(velocity_neu.y)) {
            return false;  // No horizontal avoidance vector available
        }

        // Re-normalize after removing vertical component
        // Ensures consistent magnitude regardless of original vertical component
        velocity_neu.normalize();

        // Scale to 100m avoidance offset (10,000 centimeters)
        // This provides sufficient lateral separation from threat path
        velocity_neu *= 10000;

        // Apply horizontal offset to target location
        // Modifies latitude/longitude, preserves altitude
        new_loc.offset(velocity_neu.x, velocity_neu.y);
        return true;
    }

    // Failed to calculate perpendicular vector (e.g., invalid obstacle data)
    return false;
}

#endif // AP_ADSB_AVOIDANCE_ENABLED

#endif // AP_ADSB_AVOIDANCE_ENABLED
