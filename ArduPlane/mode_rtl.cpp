/**
 * @file mode_rtl.cpp
 * @brief Implementation of RTL (Return To Launch) mode for ArduPlane
 * 
 * @details This file implements the RTL flight mode which provides automated
 *          return to the home location or rally point with automatic landing capability.
 *          
 *          RTL mode handles:
 *          - Selection of return destination (home or nearest rally point)
 *          - Climb to safe RTL altitude (RTL_ALTITUDE parameter)
 *          - Navigation back to the return point
 *          - Optional automatic landing sequence
 *          - Integration with QuadPlane VTOL landing (QRTL mode switching)
 *          
 *          For QuadPlanes, RTL can automatically transition to QRTL (QuadPlane RTL)
 *          when approaching the landing location, enabling VTOL landing capabilities.
 *          
 *          The mode supports multiple landing options via RTL_AUTOLAND parameter:
 *          - Loiter at return point
 *          - Immediate transition to landing sequence
 *          - Automatic mission landing approach
 * 
 * @note RTL is a safety-critical mode commonly used for failsafe recovery
 * @warning Incorrect RTL_ALTITUDE configuration can result in terrain collision
 * 
 * @see ModeRTL class definition in mode.h
 * @see plane.do_RTL() for RTL initialization logic
 */

#include "mode.h"
#include "Plane.h"

/**
 * @brief Initialize and enter RTL mode
 * 
 * @details This method is called when the vehicle transitions into RTL mode.
 *          It performs the following initialization sequence:
 *          
 *          1. Records current location as previous waypoint for navigation
 *          2. Calls plane.do_RTL() to determine return destination:
 *             - Selects nearest rally point if rally points are configured
 *             - Otherwise selects home location as target
 *             - Sets target altitude to RTL_ALTITUDE parameter value
 *          3. Resets climb completion flag for altitude gain management
 *          4. For QuadPlanes, evaluates whether to use QRTL mode instead:
 *             - Switches to QLAND if in guided wait takeoff state (failsafe during takeoff)
 *             - Switches to QRTL immediately if Q_RTL_MODE=2 (QRTL_ALWAYS)
 *             - May switch to QRTL if already close to home and configured for VTOL landing
 * 
 *          QuadPlane RTL behavior is controlled by Q_RTL_MODE parameter:
 *          - 0: RTL then switch to QRTL near home
 *          - 1: VTOL approach then QRTL
 *          - 2: Immediate QRTL (handled in this method)
 * 
 * @return Always returns true indicating successful mode entry
 * 
 * @note This method may trigger an immediate mode change to QLAND or QRTL
 *       for quadplanes under certain conditions
 * @warning For quadplanes, this mode entry can result in VTOL operations
 *          requiring sufficient battery and hover capability
 * 
 * @see plane.do_RTL() for return destination selection
 * @see QuadPlane::rtl_mode for VTOL RTL behavior configuration
 */
bool ModeRTL::_enter()
{
    plane.prev_WP_loc = plane.current_loc;
    plane.do_RTL(plane.get_RTL_altitude_cm());
    plane.rtl.done_climb = false;
#if HAL_QUADPLANE_ENABLED
    plane.vtol_approach_s.approach_stage = Plane::VTOLApproach::Stage::RTL;

    // Quadplane specific checks
    if (plane.quadplane.available()) {
        // treat RTL as QLAND if we are in guided wait takeoff state, to cope
        // with failsafes during GUIDED->AUTO takeoff sequence
        if (plane.quadplane.guided_wait_takeoff_on_mode_enter) {
            plane.set_mode(plane.mode_qland, ModeReason::QLAND_INSTEAD_OF_RTL);
            return true;
        }

        // if Q_RTL_MODE is QRTL always, immediately switch to QRTL mode
        if (plane.quadplane.rtl_mode == QuadPlane::RTL_MODE::QRTL_ALWAYS) {
            plane.set_mode(plane.mode_qrtl, ModeReason::QRTL_INSTEAD_OF_RTL);
            return true;
        }

        // if VTOL landing is expected and quadplane motors are active and within QRTL radius and under QRTL altitude then switch to QRTL
        const bool vtol_landing = (plane.quadplane.rtl_mode == QuadPlane::RTL_MODE::SWITCH_QRTL) || (plane.quadplane.rtl_mode == QuadPlane::RTL_MODE::VTOL_APPROACH_QRTL);
        if (vtol_landing && (quadplane.motors->get_desired_spool_state() == AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED)) {
            int32_t alt_cm;
            if ((plane.current_loc.get_distance(plane.next_WP_loc) < plane.mode_qrtl.get_VTOL_return_radius()) &&
                plane.current_loc.get_alt_cm(Location::AltFrame::ABOVE_HOME, alt_cm) && (alt_cm < plane.quadplane.qrtl_alt*100)) {
                plane.set_mode(plane.mode_qrtl, ModeReason::QRTL_INSTEAD_OF_RTL);
                return true;
            }
        }
    }
#endif

    return true;
}

/**
 * @brief Update RTL flight control outputs and manage initial climb phase
 * 
 * @details This method is called at the main control loop rate to update control
 *          outputs during RTL mode execution. It manages the initial climb phase
 *          to ensure the aircraft reaches a safe altitude before turning toward home.
 *          
 *          Control Output Calculation:
 *          - Calculates desired roll angle (nav_roll) for lateral navigation
 *          - Calculates desired pitch angle (nav_pitch) for altitude control
 *          - Calculates throttle output for airspeed and altitude maintenance
 *          
 *          Initial Climb Management (two possible behaviors):
 *          
 *          1. CLIMB_BEFORE_TURN option enabled:
 *             - Aircraft climbs straight ahead to RTL_ALTITUDE before turning
 *             - Overrides RTL_CLIMB_MIN parameter
 *             - Bank angle is heavily limited until altitude reached
 *          
 *          2. RTL_CLIMB_MIN enabled (default behavior):
 *             - Allows gentle turn while climbing
 *             - Limits roll angle to LEVEL_ROLL_LIMIT until aircraft climbs
 *               RTL_CLIMB_MIN meters above entry altitude
 *             - Provides safety margin for low-altitude RTL initiation
 *          
 *          Once the climb threshold is reached:
 *          - Updates previous waypoint to current location
 *          - Recalculates altitude slope for approach
 *          - Sets done_climb flag to allow normal navigation
 *          - Removes roll angle constraints for efficient navigation
 * 
 * @note Called at main loop rate (typically 50Hz for fixed-wing)
 * @note The climb phase prevents tight turns at low altitude which could
 *       result in stall or terrain collision
 * 
 * @warning Roll limit constraint is critical for safety during low-altitude RTL
 *          and should not be disabled without thorough testing
 * 
 * @see plane.calc_nav_roll() for lateral control calculation
 * @see plane.calc_nav_pitch() for longitudinal control calculation
 * @see plane.calc_throttle() for throttle control calculation
 * @see FlightOptions::CLIMB_BEFORE_TURN for straight climb behavior
 * @see plane.g2.rtl_climb_min (RTL_CLIMB_MIN parameter)
 * @see plane.g.level_roll_limit (LEVEL_ROLL_LIMIT parameter)
 */
void ModeRTL::update()
{
    plane.calc_nav_roll();
    plane.calc_nav_pitch();
    plane.calc_throttle();

    bool alt_threshold_reached = false;
    if (plane.flight_option_enabled(FlightOptions::CLIMB_BEFORE_TURN)) {
        // Climb to RTL_ALTITUDE before turning. This overrides RTL_CLIMB_MIN.
        alt_threshold_reached = plane.current_loc.alt > plane.next_WP_loc.alt;
    } else if (plane.g2.rtl_climb_min > 0) {
        /*
           when RTL first starts limit bank angle to LEVEL_ROLL_LIMIT
           until we have climbed by RTL_CLIMB_MIN meters
           */
        alt_threshold_reached = (plane.current_loc.alt - plane.prev_WP_loc.alt)*0.01 > plane.g2.rtl_climb_min;
    } else {
        return;
    }

    if (!plane.rtl.done_climb && alt_threshold_reached) {
        plane.prev_WP_loc = plane.current_loc;
        plane.setup_alt_slope();
        plane.rtl.done_climb = true;
    }
    if (!plane.rtl.done_climb) {
        // Constrain the roll limit as a failsafe, that way if something goes wrong the plane will
        // eventually turn back and go to RTL instead of going perfectly straight. This also leaves
        // some leeway for fighting wind.
        plane.roll_limit_cd = MIN(plane.roll_limit_cd, plane.g.level_roll_limit*100);
        plane.nav_roll_cd = constrain_int32(plane.nav_roll_cd, -plane.roll_limit_cd, plane.roll_limit_cd);
    }
}

/**
 * @brief Execute RTL navigation logic and manage mode transitions
 * 
 * @details This method handles the navigation phase of RTL mode, managing the
 *          return flight to home/rally point and potential transitions to landing
 *          sequences or VTOL landing modes.
 *          
 *          Navigation Flow:
 *          
 *          1. QuadPlane VTOL Landing Management (if HAL_QUADPLANE_ENABLED):
 *             - For Q_RTL_MODE=1 (VTOL_APPROACH_QRTL): Executes VTOL approach
 *               sequence and transitions to QRTL for final landing
 *             - For Q_RTL_MODE=0 (SWITCH_QRTL): Monitors distance to home and
 *               switches to QRTL when within switching radius
 *          
 *          2. Loiter Navigation:
 *             - Configures loiter radius from RTL_RADIUS parameter
 *             - Negative RTL_RADIUS values set counter-clockwise loiter direction
 *             - Positive values set clockwise loiter direction
 *             - Executes loiter pattern around return point
 *          
 *          3. Automatic Landing Sequence (RTL_AUTOLAND parameter):
 *             - RTL_AUTOLAND=0: Continue loiter indefinitely (manual landing)
 *             - RTL_AUTOLAND=1: Immediately search for and execute landing sequence
 *             - RTL_AUTOLAND=2: Wait until reaching loiter target and altitude,
 *               then search for landing sequence in mission
 *             - RTL_AUTOLAND=3: Return to closest point in mission and resume
 *          
 *          Landing Sequence Activation:
 *          - Searches mission for DO_LAND_START command
 *          - If found, switches to AUTO mode to execute landing
 *          - If mission switch fails, remains in RTL loiter
 *          - Uses mission force_resume to continue from landing sequence
 * 
 * @note This method can trigger mode changes to QRTL or AUTO
 * @note Landing sequence search is expensive and only runs once per RTL session
 * 
 * @warning Automatic landing requires valid mission with DO_LAND_START command
 * @warning QRTL transitions require sufficient battery for VTOL operations
 * @warning Loiter radius must be appropriate for aircraft type and wind conditions
 * 
 * @see plane.update_loiter() for loiter pattern execution
 * @see plane.mission.jump_to_landing_sequence() for landing sequence search
 * @see plane.verify_landing_vtol_approach() for VTOL approach management
 * @see switch_QRTL() for quadplane RTL to QRTL transition logic
 * @see plane.g.rtl_autoland (RTL_AUTOLAND parameter)
 * @see plane.g.rtl_radius (RTL_RADIUS parameter)
 */
void ModeRTL::navigate()
{
#if HAL_QUADPLANE_ENABLED
    if (plane.quadplane.available()) {
        if (plane.quadplane.rtl_mode == QuadPlane::RTL_MODE::VTOL_APPROACH_QRTL) {
            // VTOL approach landing
            AP_Mission::Mission_Command cmd;
            cmd.content.location = plane.next_WP_loc;
            plane.verify_landing_vtol_approach(cmd);
            if (plane.vtol_approach_s.approach_stage == Plane::VTOLApproach::Stage::VTOL_LANDING) {
                plane.set_mode(plane.mode_qrtl, ModeReason::RTL_COMPLETE_SWITCHING_TO_VTOL_LAND_RTL);
            }
            return;
        }

        if ((AP_HAL::millis() - plane.last_mode_change_ms > 1000) && switch_QRTL()) {
            return;
        }
    }
#endif

    uint16_t radius = abs(plane.g.rtl_radius);
    if (radius > 0) {
        plane.loiter.direction = (plane.g.rtl_radius < 0) ? -1 : 1;
    }

    plane.update_loiter(radius);

    if (!plane.auto_state.checked_for_autoland) {
        if ((plane.g.rtl_autoland == RtlAutoland::RTL_IMMEDIATE_DO_LAND_START) ||
            (plane.g.rtl_autoland == RtlAutoland::RTL_THEN_DO_LAND_START &&
            plane.reached_loiter_target() && 
            labs(plane.calc_altitude_error_cm()) < 1000)) {
                // we've reached the RTL point, see if we have a landing sequence
                if (plane.have_position && plane.mission.jump_to_landing_sequence(plane.current_loc)) {
                    // switch from RTL -> AUTO
                    plane.mission.set_force_resume(true);
                    if (plane.set_mode(plane.mode_auto, ModeReason::RTL_COMPLETE_SWITCHING_TO_FIXEDWING_AUTOLAND)) {
                        // return here so we don't change the radius and don't run the rtl update_loiter()
                        return;
                    }
                    // mode change failed, revert force resume flag
                    plane.mission.set_force_resume(false);
                }

                // prevent running the expensive jump_to_landing_sequence
                // on every loop
                plane.auto_state.checked_for_autoland = true;

        } else if (plane.g.rtl_autoland == RtlAutoland::DO_RETURN_PATH_START) {
            if (plane.have_position && plane.mission.jump_to_closest_mission_leg(plane.current_loc)) {
                plane.mission.set_force_resume(true);
                if (plane.set_mode(plane.mode_auto, ModeReason::RTL_COMPLETE_SWITCHING_TO_FIXEDWING_AUTOLAND)) {
                    // return here so we don't change the radius and don't run the rtl update_loiter()
                    return;
                }
                // mode change failed, revert force resume flag
                plane.mission.set_force_resume(false);
            }
            plane.auto_state.checked_for_autoland = true;
        }
    }
}

#if HAL_QUADPLANE_ENABLED
/**
 * @brief Evaluate and execute switch from RTL to QRTL mode for QuadPlanes
 * 
 * @details This method determines if the aircraft is close enough to the return
 *          point to transition from fixed-wing RTL flight to VTOL QRTL mode for
 *          vertical landing. It is only active when Q_RTL_MODE=0 (SWITCH_QRTL).
 *          
 *          Switching Logic:
 *          
 *          1. Verify Q_RTL_MODE is set to SWITCH_QRTL (mode 0)
 *          2. Determine switching radius:
 *             - Use RTL_RADIUS if configured (non-zero)
 *             - Otherwise use LOITER_RADIUS parameter
 *          3. Check if any of these conditions are met:
 *             - Navigation controller reports loiter target reached
 *             - Aircraft has passed the finish line (home/rally point)
 *             - Distance to home is less than MAX(switch_radius, stopping_distance)
 *          
 *          The stopping distance calculation accounts for:
 *          - Current ground speed
 *          - Deceleration capabilities
 *          - Wind conditions
 *          - Ensures aircraft can safely transition to hover before overshooting
 *          
 *          When switching conditions are met:
 *          - Immediately changes to QRTL mode
 *          - QRTL will handle final approach and vertical landing
 *          - Returns true to indicate mode change occurred
 * 
 * @return true if switch to QRTL mode was executed
 * @return false if not ready to switch or not configured for QRTL switching
 * 
 * @note This method is called repeatedly during RTL navigation (approximately every second)
 * @note Only applies to QuadPlane vehicles with Q_RTL_MODE=0
 * @note The larger of stopping distance or loiter radius is used to ensure
 *       safe transition without overshooting the landing point
 * 
 * @warning Switching too close to home without sufficient stopping distance
 *          can cause overshoot of the landing location
 * @warning Requires adequate battery remaining for VTOL hover and landing
 * 
 * @see QuadPlane::RTL_MODE for mode configuration options
 * @see plane.quadplane.stopping_distance() for deceleration calculation
 * @see plane.nav_controller->reached_loiter_target() for position checking
 */
bool ModeRTL::switch_QRTL()
{
    if (plane.quadplane.rtl_mode != QuadPlane::RTL_MODE::SWITCH_QRTL) {
        return false;
    }

    uint16_t qrtl_radius = abs(plane.g.rtl_radius);
    if (qrtl_radius == 0) {
        qrtl_radius = abs(plane.aparm.loiter_radius);
    }

    if (plane.nav_controller->reached_loiter_target() ||
         plane.current_loc.past_interval_finish_line(plane.prev_WP_loc, plane.next_WP_loc) ||
         plane.auto_state.wp_distance < MAX(qrtl_radius, plane.quadplane.stopping_distance())) {
        /*
          for a quadplane in RTL mode we switch to QRTL when we
          are within the maximum of the stopping distance and the
          RTL_RADIUS
         */
        plane.set_mode(plane.mode_qrtl, ModeReason::RTL_COMPLETE_SWITCHING_TO_VTOL_LAND_RTL);
        return true;
    }

    return false;
}

#endif  // HAL_QUADPLANE_ENABLED
