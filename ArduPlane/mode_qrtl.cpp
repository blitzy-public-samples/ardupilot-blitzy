/**
 * @file mode_qrtl.cpp
 * @brief QRTL (QuadPlane Return To Launch) flight mode implementation
 * 
 * @details This file implements the QRTL mode for QuadPlane vehicles, which provides
 *          intelligent return-to-launch behavior with VTOL landing capability.
 *          
 *          QRTL mode combines fixed-wing efficiency for long-distance returns with
 *          VTOL precision for landing. The mode automatically manages the transition
 *          from forward flight to VTOL and executes a controlled landing sequence.
 *          
 *          Flight Phases:
 *          1. Climb Phase (SubMode::climb): Initial climb to safe altitude cone
 *          2. Return Phase (SubMode::RTL): Navigation to home/rally point
 *          3. VTOL Transition: Automatic transition to multirotor flight
 *          4. Landing: Precision VTOL descent and touchdown
 *          
 *          The mode intelligently determines whether to climb before returning based on
 *          current altitude, distance to destination, and configured parameters. For
 *          vehicles already close to home or at sufficient altitude, the climb phase
 *          may be skipped.
 *          
 *          Key Features:
 *          - Automatic climb to safe altitude cone if needed
 *          - Rally point support (returns to closest rally point or home)
 *          - Smooth altitude profile during approach phase
 *          - Weather vane yaw control during climb
 *          - Stick mixing during approach for pilot override
 *          - Terrain following support when available
 *          
 * @note This mode is only available when HAL_QUADPLANE_ENABLED is defined
 * @warning Vehicle must have functional VTOL motors for safe QRTL operation
 * 
 * @see ModeQRTL class definition in mode.h
 * @see QuadPlane::vtol_position_controller() for position control details
 */

#include "mode.h"
#include "Plane.h"

#if HAL_QUADPLANE_ENABLED

/**
 * @brief Enter QRTL mode and determine return point strategy
 * 
 * @details This method initializes QRTL mode by analyzing the current vehicle state
 *          and determining the optimal return strategy. It decides whether the vehicle
 *          needs to climb before returning or can navigate directly to the destination.
 *          
 *          Decision Logic:
 *          1. Check for guided wait takeoff state (switches to QLAND if detected)
 *          2. Determine target return altitude (home + Q_RTL_ALT)
 *          3. Calculate best destination (rally point or home)
 *          4. Evaluate if climb is needed based on altitude cone geometry
 *          5. Configure initial submode (climb or RTL)
 *          
 *          Altitude Cone Calculation:
 *          The method calculates a safety cone with apex at home and radius defined by
 *          get_VTOL_return_radius(). If the vehicle is below this cone, it climbs to
 *          reach the cone surface before starting horizontal navigation.
 *          
 *          Minimum Climb: Q_RTL_ALT_MIN (constrained between Q_LAND_FINAL_ALT and Q_RTL_ALT)
 *          Target Climb: MAX(Q_RTL_ALT * (dist/radius), Q_RTL_ALT_MIN)
 *          
 *          Special Cases:
 *          - If VTOL motors already active and close to home: Skip climb and transition
 *          - If above the altitude cone: Return at current altitude (capped at Q_RTL_ALT)
 *          - If in guided wait takeoff: Switch to QLAND mode instead
 * 
 * @return true Always returns true indicating mode entry successful
 * 
 * @note Uses terrain data if available and terrain following is enabled
 * @warning Accurate altitude estimation is critical for safe climb calculation
 * 
 * @see calc_best_rally_or_home_location() for destination selection
 * @see relative_ground_altitude() for terrain-aware altitude calculation
 * @see get_VTOL_return_radius() for cone radius calculation
 */
bool ModeQRTL::_enter()
{
    // treat QRTL as QLAND if we are in guided wait takeoff state, to cope
    // with failsafes during GUIDED->AUTO takeoff sequence
    if (plane.quadplane.guided_wait_takeoff_on_mode_enter) {
       plane.set_mode(plane.mode_qland, ModeReason::QLAND_INSTEAD_OF_RTL);
       return true;
    }
    // Initialize to RTL submode by default (may change to climb if needed)
    submode = SubMode::RTL;
    plane.prev_WP_loc = plane.current_loc;

    // Calculate target return altitude: home altitude + Q_RTL_ALT parameter (converted to cm)
    int32_t RTL_alt_abs_cm = plane.home.alt + quadplane.qrtl_alt*100UL;
    if (quadplane.motors->get_desired_spool_state() == AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED) {
        // VTOL motors are active, either in VTOL flight or assisted flight
        // Select closest rally point or home as destination
        Location destination = plane.calc_best_rally_or_home_location(plane.current_loc, RTL_alt_abs_cm);
        const float dist = plane.current_loc.get_distance(destination);
        const float radius = get_VTOL_return_radius();

        // Calculate target climb altitude using cone geometry
        // Cone: apex at home, height = Q_RTL_ALT, base radius = VTOL_return_radius
        // If inside cone base (dist < radius): climb to cone surface
        // Formula: target_alt = Q_RTL_ALT * (dist/radius), but at least Q_RTL_ALT_MIN
        const float min_climb = constrain_float(quadplane.qrtl_alt_min, quadplane.land_final_alt, quadplane.qrtl_alt);
        const float target_alt = MAX(quadplane.qrtl_alt * (dist / MAX(radius, dist)), min_climb);


#if AP_TERRAIN_AVAILABLE
        const bool use_terrain = plane.terrain_enabled_in_mode(mode_number());
#else
        const bool use_terrain = false;
#endif

        // Calculate required climb distance relative to ground/terrain
        const float dist_to_climb = target_alt - plane.relative_ground_altitude(RangeFinderUse::CLIMB, use_terrain);
        if (is_positive(dist_to_climb)) {
            // Need to climb before returning - set climb submode
            submode = SubMode::climb;
            plane.next_WP_loc = plane.current_loc;
#if AP_TERRAIN_AVAILABLE
            int32_t curent_alt_terrain_cm;
            if (use_terrain && plane.current_loc.get_alt_cm(Location::AltFrame::ABOVE_TERRAIN, curent_alt_terrain_cm)) {
                plane.next_WP_loc.set_alt_cm(curent_alt_terrain_cm + dist_to_climb * 100UL, Location::AltFrame::ABOVE_TERRAIN);
                return true;
            }
#endif
            plane.next_WP_loc.set_alt_cm(plane.current_loc.alt + dist_to_climb * 100UL, plane.current_loc.get_alt_frame());
            return true;

        } else if (dist < radius) {
            // Above home "cone", return at current altitude if lower than QRTL alt
            int32_t current_alt_abs_cm;
            if (plane.current_loc.get_alt_cm(Location::AltFrame::ABSOLUTE, current_alt_abs_cm)) {
                RTL_alt_abs_cm = MIN(RTL_alt_abs_cm, current_alt_abs_cm);
            }

            // we're close to destination and already running VTOL motors, don't transition and don't climb
            gcs().send_text(MAV_SEVERITY_INFO,"VTOL position1 d=%.1f r=%.1f", dist, radius);
            poscontrol.set_state(QuadPlane::QPOS_POSITION1);
        }
    }

    // Setup navigation waypoint to destination using standard RTL logic
    plane.do_RTL(RTL_alt_abs_cm);
    quadplane.poscontrol_init_approach();

    // Determine if slow descent is needed (descending to destination altitude)
    // slow_descent flag enables gradual descent profile instead of direct navigation
    int32_t from_alt;
    int32_t to_alt;
    if (plane.current_loc.get_alt_cm(Location::AltFrame::ABSOLUTE,from_alt) && plane.next_WP_loc.get_alt_cm(Location::AltFrame::ABSOLUTE,to_alt)) {
        poscontrol.slow_descent = from_alt > to_alt;
        return true;
    }
    // Fallback: use relative altitude comparison if absolute altitude unavailable
    poscontrol.slow_descent = (plane.current_loc.alt > plane.next_WP_loc.alt);
    return true;
}

/**
 * @brief Update QRTL mode - delegates to QSTABILIZE update
 * 
 * @details This method is called periodically to update the QRTL mode state.
 *          It delegates to the QSTABILIZE mode update method to handle basic
 *          stabilization updates. The main flight control logic is handled in
 *          the run() method rather than update().
 *          
 *          This simple delegation allows QRTL to leverage the standard QSTABILIZE
 *          stabilization framework while implementing custom navigation logic in run().
 * 
 * @note Main QRTL logic executes in run(), not update()
 * 
 * @see run() for primary QRTL mode execution logic
 * @see ModeQStabilize::update() for stabilization updates
 */
void ModeQRTL::update()
{
    plane.mode_qstabilize.update();
}

/**
 * @brief Main QRTL mode execution - manages return flight and VTOL transition
 * 
 * @details This is the primary execution method for QRTL mode, called at the main loop
 *          rate to manage the complete return-to-launch sequence. It implements a
 *          state machine with two main phases: climb and RTL.
 *          
 *          Flight Phase State Machine:
 *          
 *          SubMode::climb:
 *          - Requests zero horizontal velocity (hold position during climb)
 *          - Runs XY position controller to maintain horizontal position
 *          - Controls attitude via position controller outputs
 *          - Climbs at full WP navigation speed (wp_nav->get_default_speed_up_cms())
 *          - Applies weather vane yaw control (no pilot yaw input)
 *          - Monitors stopping point altitude to detect climb completion
 *          - Transitions to RTL submode when target altitude reached
 *          
 *          SubMode::RTL:
 *          - Executes VTOL position controller for navigation
 *          - Progresses through position control states (QPOS_POSITION1/2, QPOS_APPROACH)
 *          - Adjusts target altitude to home altitude when past POSITION2
 *          - Initiates landing verification at POSITION2
 *          - Enables stick mixing during airbrake and approach phases
 *          
 *          Special Case - Tailsitters:
 *          During VTOL transition pull-up phase, tailsitters run fixed-wing controllers
 *          instead of VTOL controllers by calling Mode::run().
 *          
 *          Control Surface Management:
 *          Regardless of submode, fixed-wing control surfaces are stabilized on all axes
 *          (roll, pitch, yaw) to provide additional stability and control authority.
 * 
 * @note Called at main loop rate (typically 400Hz for copter attitude control)
 * @note Weather vane yaw automatically active during climb phase
 * @note Stick mixing enabled during approach to allow pilot corrections
 * 
 * @warning Position controller must be properly initialized before calling run()
 * @warning Accurate position and velocity estimates are critical for safe navigation
 * 
 * @see vtol_position_controller() for VTOL navigation logic
 * @see verify_vtol_land() for landing sequence management
 * @see get_weathervane_yaw_rate_cds() for yaw control during climb
 */
void ModeQRTL::run()
{
    const uint32_t now = AP_HAL::millis();
    if (quadplane.tailsitter.in_vtol_transition(now)) {
        // Tailsitters in FW pull up phase of VTOL transition run FW controllers
        Mode::run();
        return;
    }

    switch (submode) {
        case SubMode::climb: {
            // Request zero horizontal velocity to hold position during vertical climb
            Vector2f vel, accel;
            pos_control->input_vel_accel_NE_cm(vel, accel);
            quadplane.run_xy_controller();

            // nav roll and pitch are controller by position controller
            plane.nav_roll_cd = pos_control->get_roll_cd();
            plane.nav_pitch_cd = pos_control->get_pitch_cd();

            plane.quadplane.assign_tilt_to_fwd_thr();

            if (quadplane.transition->set_VTOL_roll_pitch_limit(plane.nav_roll_cd, plane.nav_pitch_cd)) {
                pos_control->set_externally_limited_NE();
            }
            // weathervane with no pilot input
            quadplane.disable_yaw_rate_time_constant();
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_cd(plane.nav_roll_cd,
                                                                          plane.nav_pitch_cd,
                                                                          quadplane.get_weathervane_yaw_rate_cds());

            // Climb at full WP navigation speed to reach target altitude quickly
            quadplane.set_climb_rate_cms(quadplane.wp_nav->get_default_speed_up_cms());
            quadplane.run_z_controller();

            // Check climb completion: stopping point altitude >= target altitude
            // Stopping point accounts for velocity and ensures smooth transition
            Vector3p stopping_point;
            pos_control->get_stopping_point_U_cm(stopping_point.z);
            Location stopping_loc = Location(stopping_point.tofloat(), Location::AltFrame::ABOVE_ORIGIN);

            ftype alt_diff;
            if (!stopping_loc.get_height_above(plane.next_WP_loc, alt_diff) || is_positive(alt_diff)) {
                // Climb finished or unable to calculate altitude difference - transition to RTL phase
                submode = SubMode::RTL;
                plane.prev_WP_loc = plane.current_loc;

                int32_t RTL_alt_abs_cm = plane.home.alt + quadplane.qrtl_alt*100UL;
                Location destination = plane.calc_best_rally_or_home_location(plane.current_loc, RTL_alt_abs_cm);
                const float dist = plane.current_loc.get_distance(destination);
                const float radius = get_VTOL_return_radius();
                if (dist < radius) {
                    // if close to home return at current target altitude
                    int32_t target_alt_abs_cm;
                    if (plane.next_WP_loc.get_alt_cm(Location::AltFrame::ABSOLUTE, target_alt_abs_cm)) {
                        RTL_alt_abs_cm = MIN(RTL_alt_abs_cm, target_alt_abs_cm);
                    }
                    gcs().send_text(MAV_SEVERITY_INFO,"VTOL position1 d=%.1f r=%.1f", dist, radius);
                    poscontrol.set_state(QuadPlane::QPOS_POSITION1);
                }

                plane.do_RTL(RTL_alt_abs_cm);
                quadplane.poscontrol_init_approach();
                if (plane.current_loc.get_height_above(plane.next_WP_loc, alt_diff)) {
                    poscontrol.slow_descent = is_positive(alt_diff);
                } else {
                    // default back to old method
                    poscontrol.slow_descent = (plane.current_loc.alt > plane.next_WP_loc.alt);
                }
            }
            break;
        }

        case SubMode::RTL: {
            // Execute VTOL position controller for autonomous navigation to destination
            quadplane.vtol_position_controller();
            if (poscontrol.get_state() > QuadPlane::QPOS_POSITION2) {
                // Past POSITION2: approaching home, set target altitude to home altitude for landing
                plane.next_WP_loc.set_alt_cm(plane.home.alt, Location::AltFrame::ABSOLUTE);
            }
            if (poscontrol.get_state() >= QuadPlane::QPOS_POSITION2) {
                // At or past POSITION2: initiate landing verification and descent logic
                quadplane.verify_vtol_land();
            }

            // Allow pilot stick input during airbrake and approach phases for safety
            // Pilot can make minor corrections during final approach if needed
            if (quadplane.poscontrol.get_state() == QuadPlane::QPOS_AIRBRAKE ||
                quadplane.poscontrol.get_state() == QuadPlane::QPOS_APPROACH) {
                plane.stabilize_stick_mixing_fbw();
            }
            break;
        }
    }

    // Always stabilize with fixed-wing control surfaces on all axes
    // Provides additional stability and control authority during VTOL operations
    // Particularly important during transitions and in windy conditions
    plane.stabilize_roll();
    plane.stabilize_pitch();
    plane.stabilize_yaw();
}

/**
 * @brief Update target altitude for smooth QRTL approach profile
 * 
 * @details This method manages the altitude profile during the approach phase of QRTL,
 *          creating a smooth descent from cruise altitude to VTOL landing altitude.
 *          
 *          Descent Profile Algorithm:
 *          The method calculates a linearly interpolated altitude that gradually
 *          decreases from RTL_ALT to Q_RTL_ALT as the vehicle approaches home.
 *          The descent rate is based on TECS maximum sink rate to allow time for
 *          the vehicle to slow down before VTOL transition.
 *          
 *          Calculation Parameters:
 *          - RTL_ALT_DELTA: Altitude difference to descend (RTL_ALT - Q_RTL_ALT)
 *          - SINK_TIME: Time needed to descend at 60% of max sink rate
 *          - SINK_DIST: Distance covered during descent at cruise speed
 *          - Interpolation range: 2*radius (start) to max(2*radius, min(20*radius, sink_dist))
 *          
 *          The altitude is interpolated based on distance from destination:
 *          - Far away (> sink_dist): Maintain RTL_ALT
 *          - Approaching (2*radius to sink_dist): Linear descent
 *          - Close (< 2*radius): At or near Q_RTL_ALT
 *          
 *          This profile ensures the vehicle has sufficient time and distance to
 *          decelerate from cruise speed to VTOL transition speed while descending.
 * 
 * @note Only active during RTL submode and QPOS_APPROACH state
 * @note Uses 60% of TECS max sink rate for conservative descent planning
 * 
 * @see TECS_controller.get_max_sinkrate() for sink rate limits
 */
void ModeQRTL::update_target_altitude()
{
    // Only update altitude profile during approach phase
    if ((submode != SubMode::RTL) || (plane.quadplane.poscontrol.get_state() != QuadPlane::QPOS_APPROACH)) {
        Mode::update_target_altitude();
        return;
    }

    /*
      initially approach at RTL_ALT_CM, then drop down to QRTL_ALT based on maximum sink rate from TECS,
      giving time to lose speed before we transition
     */
    const float radius = MAX(fabsf(float(plane.aparm.loiter_radius)), fabsf(float(plane.g.rtl_radius)));
    const float rtl_alt_delta = MAX(0, plane.g.RTL_altitude - plane.quadplane.qrtl_alt);
    const float sink_time = rtl_alt_delta / MAX(0.6*plane.TECS_controller.get_max_sinkrate(), 1);
    const float sink_dist = plane.aparm.airspeed_cruise * sink_time;
    const float dist = plane.auto_state.wp_distance;
    const float rad_min = 2*radius;
    const float rad_max = 20*radius;
    float alt = linear_interpolate(0, rtl_alt_delta,
                                   dist,
                                   rad_min, MAX(rad_min, MIN(rad_max, rad_min+sink_dist)));
    Location loc = plane.next_WP_loc;
    loc.alt += alt*100;
    plane.set_target_altitude_location(loc);
}

/**
 * @brief Determine if pilot throttle nudging is allowed in current QRTL phase
 * 
 * @details Throttle nudging allows the pilot to adjust target altitude using throttle
 *          stick input during autonomous flight. In QRTL, this is only permitted during
 *          the approach phase to give pilots the ability to adjust their landing altitude
 *          if needed (e.g., to avoid obstacles or compensate for terrain).
 *          
 *          Throttle nudging is disabled during:
 *          - Climb phase: Vehicle is climbing to safe altitude, pilot adjustment not needed
 *          - Early RTL phases: Vehicle is navigating at fixed altitude
 *          
 *          Throttle nudging is enabled during:
 *          - QPOS_APPROACH: Final approach to landing, where altitude adjustment most useful
 * 
 * @return true if in RTL submode and QPOS_APPROACH state, false otherwise
 * 
 * @note Provides pilot override capability during the most critical landing phase
 * @see update_target_altitude() for automatic altitude profile management
 */
bool ModeQRTL::allows_throttle_nudging() const
{
    return (submode == SubMode::RTL) && (plane.quadplane.poscontrol.get_state() == QuadPlane::QPOS_APPROACH);
}

/**
 * @brief Calculate the radius around destination for pure VTOL flight operations
 * 
 * @details This method calculates the radius from the destination (home or rally point)
 *          within which the vehicle should remain in pure VTOL mode without transitioning
 *          to fixed-wing flight. This radius defines the "VTOL zone" around the landing site.
 *          
 *          The radius is calculated as 1.5 times the larger of:
 *          - LOITER_RADIUS (parameter): Standard loiter radius
 *          - RTL_RADIUS (parameter): RTL loiter radius
 *          
 *          Purpose:
 *          - Ensures vehicle doesn't attempt FW transition when too close to destination
 *          - Defines the altitude cone base radius for climb calculations
 *          - Prevents unnecessary mode transitions near landing site
 *          - Provides safety margin around landing area
 *          
 *          The 1.5x multiplier provides margin beyond the standard loiter radius to
 *          account for position controller overshoot and wind effects.
 * 
 * @return Radius in meters defining VTOL-only flight zone around destination
 * 
 * @note Absolute value used to handle negative radius values (reverse loiter direction)
 * @note Larger of loiter_radius and rtl_radius used to be conservative
 * 
 * @see _enter() uses this radius to determine if climb is needed
 * @see run() uses this radius during climb completion check
 */
float ModeQRTL::get_VTOL_return_radius() const
{
    return MAX(fabsf(float(plane.aparm.loiter_radius)), fabsf(float(plane.g.rtl_radius))) * 1.5;
}

#endif
