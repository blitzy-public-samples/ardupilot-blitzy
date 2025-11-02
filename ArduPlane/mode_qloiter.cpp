/**
 * @file mode_qloiter.cpp
 * @brief QLOITER flight mode implementation for ArduPlane QuadPlane
 * 
 * @details This file implements the QLOITER (Quadplane Loiter) flight mode,
 *          which provides GPS-based position hold for QuadPlane vehicles in
 *          VTOL (Vertical Take-Off and Landing) mode. QLOITER maintains both
 *          horizontal position and altitude using the multicopter control system.
 *          
 *          The mode uses AC_PosControl for 3D position control and AC_Loiter
 *          for horizontal position hold. It integrates with the QuadPlane's
 *          position controller and supports precision landing target overrides,
 *          throttle wait states, and tailsitter transitions.
 *          
 *          Key Features:
 *          - GPS-based horizontal position hold using loiter navigation
 *          - Altitude hold using vertical position controller
 *          - Pilot input for roll/pitch (position adjustments) and yaw
 *          - Precision landing target override support (when enabled)
 *          - Integration with fixed-wing control surfaces for hybrid stability
 *          - Throttle wait state handling during initialization
 *          - VTOL recovery from extreme attitudes
 *          
 * @note This mode requires GPS lock and is only available on QuadPlane
 *       configurations (HAL_QUADPLANE_ENABLED must be defined).
 * 
 * @warning This is flight-critical code. Modifications must be thoroughly
 *          tested in SITL before hardware testing.
 * 
 * @see AC_PosControl, AC_Loiter, QuadPlane
 */

#include "mode.h"
#include "Plane.h"

#if HAL_QUADPLANE_ENABLED

/**
 * @brief Initialize QLOITER mode upon entry
 * 
 * @details This method is called when the vehicle enters QLOITER mode. It performs
 *          all necessary initialization for GPS-based position hold in VTOL mode:
 *          
 *          1. Loiter Navigation Initialization:
 *             - Clears any pilot-commanded acceleration inputs
 *             - Initializes loiter target to current position
 *          
 *          2. Vertical Control Configuration:
 *             - Sets maximum vertical speed limits (up and down)
 *             - Sets vertical acceleration limits
 *             - Configures position correction speeds for altitude control
 *          
 *          3. State Initialization:
 *             - Initializes throttle wait state
 *             - Records timestamp to prevent immediate position re-initialization
 *             - Clears precision landing target timestamp
 *          
 *          Speed and acceleration limits are configured in centimeters and cm/s/s
 *          from pilot parameters: pilot_velocity_z_max_dn, pilot_speed_z_max_up,
 *          and pilot_accel_z.
 * 
 * @return true Always returns true indicating successful mode entry
 * 
 * @note This method is called by the mode change logic before the first update()
 *       or run() call in QLOITER mode.
 * 
 * @see ModeQLoiter::run(), AC_Loiter::init_target(), AC_PosControl::set_max_speed_accel_U_cm()
 */
bool ModeQLoiter::_enter()
{
    // initialise loiter
    loiter_nav->clear_pilot_desired_acceleration();
    loiter_nav->init_target();

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_U_cm(-quadplane.get_pilot_velocity_z_max_dn(), quadplane.pilot_speed_z_max_up*100, quadplane.pilot_accel_z*100);
    pos_control->set_correction_speed_accel_U_cmss(-quadplane.get_pilot_velocity_z_max_dn(), quadplane.pilot_speed_z_max_up*100, quadplane.pilot_accel_z*100);

    quadplane.init_throttle_wait();

    // prevent re-init of target position
    quadplane.last_loiter_ms = AP_HAL::millis();

    // clear precland timestamp
    last_target_loc_set_ms = 0;

    return true;
}

/**
 * @brief Update method for QLOITER mode (fixed-wing surfaces only)
 * 
 * @details This method is called for fixed-wing control surface updates during
 *          the main loop. For QLOITER mode, it simply delegates to QSTABILIZE's
 *          update method, which handles the fixed-wing surface control logic
 *          (ailerons, elevator, rudder) while the vehicle is in VTOL mode.
 *          
 *          The primary QLOITER control logic (position hold, motor control) is
 *          implemented in the run() method. This update() method is only responsible
 *          for maintaining consistent fixed-wing surface behavior.
 * 
 * @note This method is called at the fixed-wing controller rate, separate from
 *       the quadplane run() method which executes at the VTOL controller rate.
 * 
 * @see ModeQLoiter::run(), ModeQStabilize::update()
 */
void ModeQLoiter::update()
{
    plane.mode_qstabilize.update();
}

/**
 * @brief Main control loop for QLOITER mode - runs the quadplane loiter controller
 * 
 * @details This method implements the complete QLOITER control logic, providing GPS-based
 *          position hold in VTOL mode using the AC_PosControl position controller and
 *          AC_Loiter horizontal navigation controller. It runs at the VTOL controller
 *          rate (typically 400Hz) and manages:
 *          
 *          Control Flow:
 *          1. VTOL Recovery Check: Hands off to QHOVER if extreme attitudes detected
 *          2. Precision Landing Override: Applies precision target position/velocity if active
 *          3. Tailsitter Transition: Delegates to fixed-wing controller during FW pull-up
 *          4. Throttle Wait State: Holds ground idle until throttle commanded
 *          5. Position Reinitialization: Resets target if >500ms since last update
 *          6. Horizontal Position Control: Processes pilot roll/pitch input via loiter_nav
 *          7. Attitude Control: Sends attitude targets to multicopter attitude controller
 *          8. Altitude Control: Manages vertical position based on mode (land/guided/normal)
 *          
 *          Horizontal Position Management:
 *          - Uses AC_Loiter navigation controller for GPS position hold
 *          - Pilot roll/pitch inputs adjust desired position (not direct attitude control)
 *          - Loiter controller calculates required roll/pitch angles
 *          - Respects angle limits from loiter_nav and attitude controller
 *          - Handles position target overrides from precision landing system
 *          
 *          Altitude Management:
 *          - Normal QLOITER: Pilot throttle stick controls climb rate
 *          - QLAND mode: Automated descent rate based on height above ground
 *          - GUIDED takeoff: Maintains current altitude (zero climb rate)
 *          - Uses AC_PosControl vertical controller (pos_control->land_at_climb_rate_cm)
 *          
 *          Motor and Surface Integration:
 *          - Sets motor spool state (GROUND_IDLE or THROTTLE_UNLIMITED)
 *          - Integrates fixed-wing control surfaces for additional stability
 *          - Stabilizes roll and pitch using fixed-wing surfaces
 *          - Centers rudder (no direct pilot rudder control in QLOITER)
 *          
 *          Special Handling:
 *          - Precision Landing (AC_PRECLAND_ENABLED): Accepts position/velocity overrides
 *          - Tailsitter: Uses fixed-wing controllers during VTOL transition pull-up phase
 *          - SystemID (AP_PLANE_SYSTEMID_ENABLED): Applies attitude offset for system identification
 *          - Soft Landing: Reduces loiter aggression when close to ground
 * 
 * @note This method is called at the VTOL controller rate (typically 400Hz) from the
 *       main scheduler. It assumes GPS lock and valid position estimates from EKF.
 * 
 * @warning Safety-critical flight control code. The loiter controller must maintain
 *          stable position hold to prevent flyaways. Loss of GPS during QLOITER will
 *          trigger failsafe actions defined in the QuadPlane failsafe configuration.
 * 
 * @warning Precision landing overrides are time-limited (250ms timeout). If precision
 *          landing updates stop, the vehicle reverts to normal loiter position hold.
 * 
 * @see AC_PosControl::set_pos_desired_NE_cm() - Horizontal position control (NED frame)
 * @see AC_Loiter::update() - Loiter navigation calculation
 * @see AC_AttitudeControl_Multi::input_euler_angle_roll_pitch_euler_rate_yaw_cd() - Attitude control
 * @see QuadPlane::run_z_controller() - Vertical position/velocity control
 */
void ModeQLoiter::run()
{
    // VTOL Recovery Check: If extreme attitude detected (bank angle or pitch beyond safe limits),
    // hand off control to QHOVER mode which provides simpler hover control while the fixed-wing
    // surfaces assist with attitude recovery. This prevents the loiter controller from fighting
    // extreme attitudes.
    if (quadplane.assist.check_VTOL_recovery()) {
        // use QHover to recover from extreme attitudes, this allows
        // for the fixed wing controller to handle the recovery
        plane.mode_qhover.run();
        return;
    }

    const uint32_t now = AP_HAL::millis();

#if AC_PRECLAND_ENABLED
    // Precision Landing Position/Velocity Override System
    // When precision landing is enabled, external systems (IR-LOCK, vision, etc.) can override
    // the loiter target position and/or velocity to track a precision landing target. This allows
    // the vehicle to follow a moving target while in QLOITER mode.
    const uint32_t precland_timeout_ms = 250;
    /*
      see if precision landing or precision loiter is active with
      an override of the target location.

    */
    const uint32_t last_pos_set_ms = last_target_loc_set_ms;
    const uint32_t last_vel_set_ms = quadplane.poscontrol.last_velocity_match_ms;

    // Position Target Override: If precision landing has provided a target position within
    // the last 250ms, override the loiter position target. The target position is converted
    // from a Location (lat/lon) to NED frame (North-East cm from EKF origin) and fed directly
    // to the position controller.
    if (last_pos_set_ms != 0 && now - last_pos_set_ms < precland_timeout_ms) {
        // we have an active landing target override
        Vector2f rel_origin;
        if (plane.next_WP_loc.get_vector_xy_from_origin_NE_cm(rel_origin)) {
            quadplane.pos_control->set_pos_desired_NE_cm(rel_origin);
            last_target_loc_set_ms = 0;
        }
    }

    // Velocity Match Override: If precision landing has provided a target velocity within
    // the last 250ms, command the position controller to match that velocity. This is used
    // when tracking a moving landing target. Velocity is converted from m/s to cm/s.
    if (last_vel_set_ms != 0 && now - last_vel_set_ms < precland_timeout_ms) {
        // we have an active landing velocity override
        Vector2f target_accel;
        Vector2f target_speed_xy_cms{quadplane.poscontrol.velocity_match.x*100, quadplane.poscontrol.velocity_match.y*100};
        quadplane.pos_control->input_vel_accel_NE_cm(target_speed_xy_cms, target_accel);
        quadplane.poscontrol.last_velocity_match_ms = 0;
    }
#endif // AC_PRECLAND_ENABLED

    // Tailsitter Transition Handling: During the fixed-wing pull-up phase of a VTOL-to-FW
    // transition on tailsitter aircraft, use the base Mode fixed-wing controller instead of
    // VTOL loiter control. This allows the aircraft to build forward airspeed while transitioning
    // from vertical to horizontal flight orientation.
    if (quadplane.tailsitter.in_vtol_transition(now)) {
        // Tailsitters in FW pull up phase of VTOL transition run FW controllers
        Mode::run();
        return;
    }

    // Throttle Wait State: During initialization or after arming, wait for pilot to advance
    // throttle before spooling up motors. This safety feature prevents unexpected motor spin-up.
    // While in throttle wait:
    // - Motors kept at GROUND_IDLE (minimum safe spool state)
    // - Zero throttle output commanded
    // - Attitude and position controllers relaxed (integrators reset)
    // - Loiter target reinitialized to prevent position jumps when exiting wait state
    // - Fixed-wing surfaces actively stabilize the aircraft
    if (quadplane.throttle_wait) {
        quadplane.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        attitude_control->set_throttle_out(0, true, 0);
        quadplane.relax_attitude_control();
        pos_control->relax_U_controller(0);
        loiter_nav->clear_pilot_desired_acceleration();
        loiter_nav->init_target();

        // Stabilize with fixed wing surfaces
        plane.stabilize_roll();
        plane.stabilize_pitch();
        return;
    }
    // Disarmed Reinitialization: If motors become disarmed while in QLOITER (e.g., after landing),
    // reinitialize the mode to reset all controllers and prepare for next arming/flight.
    if (!quadplane.motors->armed()) {
        plane.mode_qloiter._enter();
    }

    // Landing Softening: When close to the ground (as determined by should_relax()), reduce
    // loiter controller gains to provide gentler position corrections during landing touchdown.
    if (quadplane.should_relax()) {
        loiter_nav->soften_for_landing();
    }

    // Position Target Timeout: If more than 500ms has elapsed since the last loiter update,
    // reinitialize the loiter target to current position. This prevents stale position targets
    // if the controller was paused or the mode was temporarily overridden.
    if (now - quadplane.last_loiter_ms > 500) {
        loiter_nav->clear_pilot_desired_acceleration();
        loiter_nav->init_target();
    }
    quadplane.last_loiter_ms = now;

    // Motor Spool State: Set motors to THROTTLE_UNLIMITED allowing full motor output range.
    // This enables the position controller to command the full thrust range needed for
    // position hold and altitude control.
    quadplane.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // Vertical Speed and Acceleration Limits: Configure the vertical position controller
    // with maximum climb/descent rates and acceleration. Values in cm/s and cm/s/s.
    // - get_pilot_velocity_z_max_dn(): Maximum descent rate (negative value)
    // - pilot_speed_z_max_up: Maximum climb rate (converted from m/s to cm/s)
    // - pilot_accel_z: Vertical acceleration limit (converted to cm/s/s)
    pos_control->set_max_speed_accel_U_cm(-quadplane.get_pilot_velocity_z_max_dn(), quadplane.pilot_speed_z_max_up*100, quadplane.pilot_accel_z*100);

    // Process Pilot Roll/Pitch Input: In QLOITER, pilot roll and pitch stick inputs do NOT
    // directly command attitude. Instead, they command desired position changes (acceleration).
    // The loiter controller translates these position change requests into the appropriate
    // roll/pitch angles needed to achieve the desired position change.
    // 
    // Angle limits are the minimum of:
    // - loiter_nav angle limit (from LOIT_ANG_MAX parameter)
    // - attitude controller altitude hold angle limit (from Q_A_ANGLE_MAX)
    float target_roll_cd, target_pitch_cd;
    quadplane.get_pilot_desired_lean_angles(target_roll_cd, target_pitch_cd, loiter_nav->get_angle_max_cd(), attitude_control->get_althold_lean_angle_max_cd());
    loiter_nav->set_pilot_desired_acceleration_cd(target_roll_cd, target_pitch_cd);
    
    // Horizontal Position Controller Initialization and Update:
    // Ensure the position controller's horizontal (North-East) component is active and initialized.
    // Then run the loiter navigation update which:
    // 1. Calculates position error (desired position - current position)
    // 2. Runs velocity controller (position error -> desired velocity)
    // 3. Runs acceleration controller (velocity error -> desired acceleration)
    // 4. Converts desired acceleration to required roll/pitch angles
    if (!pos_control->is_active_NE()) {
        pos_control->init_NE_controller();
    }
    loiter_nav->update();

    // Extract Navigation Angles: The loiter controller has calculated the roll and pitch
    // angles needed to achieve the desired position hold. Store these in plane.nav_roll_cd
    // and plane.nav_pitch_cd (in centidegrees) for use by the attitude controller.
    plane.nav_roll_cd = loiter_nav->get_roll_cd();
    plane.nav_pitch_cd = loiter_nav->get_pitch_cd();

    // Tilt Motor Assignment: On tiltrotor QuadPlanes, assign any tilt motor output based
    // on forward throttle demand. This coordinates tilt angle with forward flight requirements.
    plane.quadplane.assign_tilt_to_fwd_thr();

    // Transition Angle Limiting: If the vehicle is in a VTOL transition state, the transition
    // controller may limit roll/pitch angles to prevent excessive attitudes during transition.
    // If limits are applied, notify the position controller so it can account for the constraint.
    if (quadplane.transition->set_VTOL_roll_pitch_limit(plane.nav_roll_cd, plane.nav_pitch_cd)) {
        pos_control->set_externally_limited_NE();
    }

    // Yaw Control Configuration: Set the yaw rate time constant for pilot yaw input processing.
    // This determines how quickly the vehicle responds to pilot yaw stick commands.
    quadplane.set_pilot_yaw_rate_time_constant();

    // Attitude Command Vector Construction: Build the target attitude command vector containing:
    // - target.x: Roll angle in degrees (from loiter controller)
    // - target.y: Pitch angle in degrees (from loiter controller)
    // - target.z: Yaw rate in deg/s (from pilot input)
    // Note: Roll and pitch are angles (position control), yaw is a rate (velocity control)
    Vector3f target { plane.nav_roll_cd*0.01, plane.nav_pitch_cd*0.01, quadplane.get_desired_yaw_rate_cds() * 0.01 };

#if AP_PLANE_SYSTEMID_ENABLED
    // System Identification Support: If system ID is active, add attitude offsets for
    // frequency sweep testing. This is used for PID tuning and dynamics characterization.
    auto &systemid = plane.g2.systemid;
    systemid.update();
    target += systemid.get_attitude_offset_deg();
#endif

    // Attitude Controller Command: Send the complete attitude target to the multicopter
    // attitude controller. This uses angle control for roll and pitch (to hold the angles
    // calculated by the loiter controller) and rate control for yaw (pilot commands yaw rate).
    // Values converted back to centidegrees for the attitude controller API.
    // The default smoothing gain (4.0f) provides stable attitude tracking.
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_cd(target.x*100,
                                                                  target.y*100,
                                                                  target.z*100);

    // Altitude Control Mode Selection: The vertical control behavior depends on the current
    // flight mode and state. Three primary cases:
    
    // Case 1: QLAND Mode - Automated Landing Descent
    // When in QLAND mode, manage the automated landing sequence with controlled descent.
    if (plane.control_mode == &plane.mode_qland) {
        // Transition to final landing phase if conditions met (low altitude, low speed)
        if (poscontrol.get_state() < QuadPlane::QPOS_LAND_FINAL && quadplane.check_land_final()) {
            poscontrol.set_state(QuadPlane::QPOS_LAND_FINAL);
            quadplane.setup_target_position();
#if AP_ICENGINE_ENABLED
            // Safety: Cut internal combustion engine during final landing phase if configured
            if (quadplane.land_icengine_cut != 0) {
                plane.g2.ice_control.engine_control(0, 0, 0, false);
            }
#endif  // AP_ICENGINE_ENABLED
        }
        
        // Calculate appropriate descent rate based on height above ground
        // Uses rangefinder or barometric altitude depending on availability and configuration
        float height_above_ground = plane.relative_ground_altitude(RangeFinderUse::TAKEOFF_LANDING);
        float descent_rate_cms = quadplane.landing_descent_rate_cms(height_above_ground);

        // Enable ground effect compensation in AHRS during final landing phase
        // This improves altitude estimation as the vehicle nears the ground and experiences
        // increased air pressure under the rotors (ground effect)
        if (poscontrol.get_state() == QuadPlane::QPOS_LAND_FINAL && !quadplane.option_is_set(QuadPlane::OPTION::DISABLE_GROUND_EFFECT_COMP)) {
            ahrs.set_touchdown_expected(true);
        }

        // Command descent rate to position controller (negative rate = descent)
        // Second parameter enables landing detection when descent rate is positive (descending)
        pos_control->land_at_climb_rate_cm(-descent_rate_cms, descent_rate_cms>0);
        quadplane.check_land_complete();
        
    // Case 2: GUIDED Mode with Takeoff - Hold Altitude
    // During GUIDED takeoff, maintain current altitude (zero climb rate) until takeoff complete
    } else if (plane.control_mode == &plane.mode_guided && quadplane.guided_takeoff) {
        quadplane.set_climb_rate_cms(0);
        
    // Case 3: Normal QLOITER - Pilot-Commanded Climb Rate
    // In standard QLOITER operation, pilot throttle stick commands vertical speed
    // Throttle stick centered = hover (zero climb rate)
    // Throttle up = climb, throttle down = descend
    } else {
        // update altitude target and call position controller
        quadplane.set_climb_rate_cms(quadplane.get_pilot_desired_climb_rate_cms());
    }
    
    // Execute Vertical Position Controller: Run the altitude/climb rate controller to
    // calculate required vertical thrust. This integrates the climb rate command with
    // current altitude and velocity to output motor thrust commands.
    quadplane.run_z_controller();

    // Fixed-Wing Surface Stabilization: Even in VTOL mode, use the fixed-wing control
    // surfaces (ailerons and elevator) to provide additional stability and damping.
    // This is especially important for:
    // - Traditional QuadPlanes: Control surfaces provide additional damping in wind
    // - Tailsitters: Control surfaces are primary control effectors
    // - Tiltrotors: Control surfaces supplement motor control
    // The stabilize functions output to ailerons and elevator to oppose roll/pitch rates.
    plane.stabilize_roll();
    plane.stabilize_pitch();

    // Rudder Control: Center the rudder (zero output) since yaw control is handled
    // entirely by differential motor thrust in QLOITER mode. The rudder is not used
    // for yaw control in VTOL hover flight.
    output_rudder_and_steering(0.0);
}

#endif
