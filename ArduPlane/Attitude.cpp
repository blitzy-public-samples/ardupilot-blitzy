/**
 * @file Attitude.cpp
 * @brief Fixed-wing attitude stabilization control loops for roll, pitch, and yaw axes
 * 
 * @details This file implements the core attitude stabilization functions for ArduPlane,
 *          providing PID-based control of roll, pitch, and yaw axes. Key features include:
 *          - Airspeed-compensated control gains via speed_scaler
 *          - Integration with L1 navigation controller for roll/pitch targets
 *          - TECS (Total Energy Control System) integration for pitch/throttle coordination
 *          - Stick mixing for pilot override in auto modes
 *          - Coordinated turn yaw control with rudder mixing
 *          - Ground steering support during takeoff/landing
 *          - Inverted flight handling
 *          - Quadplane integration for hybrid VTOL/fixed-wing flight
 * 
 *          The control loops run at the main loop rate (typically 50-400Hz depending on board)
 *          and are the final stage before servo output, translating desired attitudes from
 *          navigation/mode logic into actual control surface deflections.
 * 
 * @note All angle inputs/outputs use centidegrees (1/100th of a degree) for integer precision
 * @note Speed scaler adjusts control gains based on airspeed to maintain consistent handling
 * @warning This is safety-critical flight control code - any modifications require extensive
 *          flight testing across the full airspeed envelope
 * 
 * Source: ArduPlane/Attitude.cpp
 */
#include "Plane.h"

/**
 * @brief Calculate airspeed-based scaling factor for control surface gains
 * 
 * @details This function computes a speed scaling multiplier applied to all PID controllers
 *          to compensate for varying aerodynamic effectiveness at different airspeeds.
 *          The scaling ensures consistent handling characteristics across the flight envelope:
 *          
 *          At high airspeed: Surfaces are more effective → reduce gains (scale < 1.0)
 *          At low airspeed: Surfaces are less effective → increase gains (scale > 1.0)
 *          
 *          Algorithm:
 *          1. Get current airspeed estimate from AHRS (or estimate from throttle if unavailable)
 *          2. Calculate speed_scaler = g.scaling_speed / current_airspeed
 *          3. Constrain to safe range (typically 0.5 to 2.0)
 *          4. Special handling for VTOL modes and takeoff
 * 
 *          The base scaling_speed parameter (SCALING_SPEED) defines the airspeed at which
 *          the PID gains are exactly as tuned (scale = 1.0). At twice this speed, gains
 *          are halved; at half this speed, gains are doubled.
 * 
 * @return float Speed scaling multiplier (typically 0.5 to 2.0)
 *         - 1.0 = nominal tuned airspeed, use configured PID gains as-is
 *         - < 1.0 = high airspeed, reduce gains
 *         - > 1.0 = low airspeed, increase gains
 * 
 * @note Called frequently (every stabilization loop iteration) so must be computationally efficient
 * @note When no airspeed sensor available, estimates scaling from throttle output
 * @note In VTOL modes at very low airspeed, severely limits control surface movement to prevent instability
 * @note During takeoff without airspeed sensor, scaling may be suppressed if SURPRESS_TKOFF_SCALING enabled
 * 
 * @warning Incorrect speed scaling can cause pilot-induced oscillations or loss of control
 * @warning The integrator decay in VTOL low-speed regime (lines 36-38) prevents integrator windup
 * 
 * Source: ArduPlane/Attitude.cpp:8-58
 */
float Plane::calc_speed_scaler(void)
{
    float aspeed, speed_scaler;
    if (ahrs.airspeed_estimate(aspeed)) {
        if (aspeed > auto_state.highest_airspeed && arming.is_armed_and_safety_off()) {
            auto_state.highest_airspeed = aspeed;
        }
        // ensure we have scaling over the full configured airspeed
        const float airspeed_min = MAX(aparm.airspeed_min, MIN_AIRSPEED_MIN);
        const float scale_min = MIN(0.5, g.scaling_speed / (2.0 * aparm.airspeed_max));
        const float scale_max = MAX(2.0, g.scaling_speed / (0.7 * airspeed_min));
        if (aspeed > 0.0001f) {
            speed_scaler = g.scaling_speed / aspeed;
        } else {
            speed_scaler = scale_max;
        }
        speed_scaler = constrain_float(speed_scaler, scale_min, scale_max);

#if HAL_QUADPLANE_ENABLED
        if ((quadplane.in_vtol_mode() || quadplane.in_assisted_flight()) && arming.is_armed_and_safety_off()) {
            // when in VTOL modes limit surface movement at low speed to prevent instability
            float threshold = airspeed_min * 0.5;
            if (aspeed < threshold) {
                float new_scaler = linear_interpolate(0.001, g.scaling_speed / threshold, aspeed, 0, threshold);
                speed_scaler = MIN(speed_scaler, new_scaler);

                // we also decay the integrator to prevent an integrator from before
                // we were at low speed persistent at high speed
                rollController.decay_I();
                pitchController.decay_I();
                yawController.decay_I();
            }
        }
#endif
    } else if (arming.is_armed_and_safety_off()) {
        // scale assumed surface movement using throttle output
        float throttle_out = MAX(SRV_Channels::get_output_scaled(SRV_Channel::k_throttle), 1);
        speed_scaler = sqrtf(THROTTLE_CRUISE / throttle_out);
        // This case is constrained tighter as we don't have real speed info
        speed_scaler = constrain_float(speed_scaler, 0.6f, 1.67f);
    } else {
        // no speed estimate and not armed, use a unit scaling
        speed_scaler = 1;
    }
    if (!plane.ahrs.using_airspeed_sensor()  && 
        (plane.flight_option_enabled(FlightOptions::SURPRESS_TKOFF_SCALING)) &&
        (plane.flight_stage == AP_FixedWing::FlightStage::TAKEOFF)) { //scaling is suppressed during climb phase of automatic takeoffs with no airspeed sensor being used due to problems with inaccurate airspeed estimates
        return MIN(speed_scaler, 1.0f) ;
    }
    return speed_scaler;
}

/*
  return true if the current settings and mode should allow for stick mixing
 */
bool Plane::stick_mixing_enabled(void)
{
    if (!rc().has_valid_input()) {
        // never stick mix without valid RC
        return false;
    }
#if AP_FENCE_ENABLED
    const bool stickmixing = fence_stickmixing();
#else
    const bool stickmixing = true;
#endif
#if HAL_QUADPLANE_ENABLED
    if (control_mode == &mode_qrtl &&
        quadplane.poscontrol.get_state() >= QuadPlane::QPOS_POSITION1) {
        // user may be repositioning
        return false;
    }
    if (quadplane.in_vtol_land_poscontrol()) {
        // user may be repositioning
        return false;
    }
#endif
    if (control_mode->does_auto_throttle() && plane.control_mode->does_auto_navigation()) {
        // we're in an auto mode. Check the stick mixing flag
        if (g.stick_mixing != StickMixing::NONE &&
            g.stick_mixing != StickMixing::VTOL_YAW &&
            stickmixing) {
            return true;
        } else {
            return false;
        }
    }

    if (failsafe.rc_failsafe && g.fs_action_short == FS_ACTION_SHORT_FBWA) {
        // don't do stick mixing in FBWA glide mode
        return false;
    }

    // non-auto mode. Always do stick mixing
    return true;
}


/**
 * @brief Execute roll axis stabilization control loop
 * 
 * @details This is the main roll stabilization function that converts the desired roll angle
 *          (nav_roll_cd) into aileron servo output commands. The function implements a complete
 *          roll control pipeline:
 *          
 *          Control Flow:
 *          1. Check for inverted flight and adjust nav_roll_cd reference if needed
 *          2. Call stabilize_roll_get_roll_out() to compute PID output
 *          3. Apply airspeed-based gain scheduling via speed_scaler
 *          4. Handle quadplane transitions (uses multicopter rate controller when appropriate)
 *          5. Apply stick mixing if enabled and pilot is providing input
 *          6. Limit output to configured servo limits (SRV_Channel::k_aileron)
 *          
 *          The nav_roll_cd target comes from:
 *          - Navigation controller (L1 for waypoint tracking)
 *          - Flight mode logic (e.g., FBWA uses pilot stick input)
 *          - Autotune system during tuning maneuvers
 *          
 *          Inverted Flight Handling:
 *          When fly_inverted() returns true, adds 180 degrees (18000 centidegrees) to nav_roll_cd
 *          to maintain proper control authority while flying upside down.
 * 
 * @note Called at main loop rate (typically 50-400Hz depending on board configuration)
 * @note Uses rollController PID object configured via ROLL2SRV_* parameters
 * @note Speed scaler from get_speed_scaler() adjusts gains for different airspeeds
 * @note Output range: -4500 to +4500 (corresponding to ±45 degrees servo deflection)
 * 
 * @warning Critical flight control function - modifications require extensive flight testing
 * @warning Inverted flight handling is essential for aerobatic and unusual attitude recovery
 * 
 * Source: ArduPlane/Attitude.cpp:111-125
 */
void Plane::stabilize_roll()
{
    if (fly_inverted()) {
        // we want to fly upside down. We need to cope with wrap of
        // the roll_sensor interfering with wrap of nav_roll, which
        // would really confuse the PID code. The easiest way to
        // handle this is to ensure both go in the same direction from
        // zero
        nav_roll_cd += 18000;
        if (ahrs.roll_sensor < 0) nav_roll_cd -= 36000;
    }

    const float roll_out = stabilize_roll_get_roll_out();
    SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, roll_out);
}

float Plane::stabilize_roll_get_roll_out()
{
    const float speed_scaler = get_speed_scaler();
#if HAL_QUADPLANE_ENABLED
    if (!quadplane.use_fw_attitude_controllers()) {
        // use the VTOL rate for control, to ensure consistency
        const auto &pid_info = quadplane.attitude_control->get_rate_roll_pid().get_pid_info();

        // scale FF to angle P
        if (quadplane.option_is_set(QuadPlane::OPTION::SCALE_FF_ANGLE_P)) {
            const float mc_angR = quadplane.attitude_control->get_angle_roll_p().kP()
                * quadplane.attitude_control->get_last_angle_P_scale().x;
            if (is_positive(mc_angR)) {
                rollController.set_ff_scale(MIN(1.0, 1.0 / (mc_angR * rollController.tau())));
            }
        }

        const float roll_out = rollController.get_rate_out(degrees(pid_info.target), speed_scaler);
        /* when slaving fixed wing control to VTOL control we need to decay the integrator to prevent
           opposing integrators balancing between the two controllers
        */
        rollController.decay_I();
        return roll_out;
    }
#endif

    bool disable_integrator = false;
    if (control_mode == &mode_stabilize && channel_roll->get_control_in() != 0) {
        disable_integrator = true;
    }
    return rollController.get_servo_out(nav_roll_cd - ahrs.roll_sensor, speed_scaler, disable_integrator,
                                        ground_mode && !(plane.flight_option_enabled(FlightOptions::DISABLE_GROUND_PID_SUPPRESSION)));
}

/**
 * @brief Execute pitch axis stabilization control loop
 * 
 * @details This is the main pitch stabilization function that converts the desired pitch angle
 *          (nav_pitch_cd) into elevator servo output commands. The pitch control is more complex
 *          than roll due to integration with the TECS energy management system and special handling
 *          for takeoff and landing phases.
 *          
 *          Control Flow:
 *          1. Check for takeoff tail hold (special case: holds elevator down during ground roll)
 *          2. Call stabilize_pitch_get_pitch_out() to compute PID output
 *          3. Apply airspeed-based gain scheduling via speed_scaler
 *          4. Handle quadplane transitions (uses multicopter rate controller when appropriate)
 *          5. Add pitch trim and throttle-to-pitch feedforward (KFF_THRTTOPITCH)
 *          6. Apply forced landing pitch if flare switch enabled and conditions met
 *          7. Apply stick mixing if enabled and pilot is providing input
 *          8. Limit output to configured servo limits (SRV_Channel::k_elevator)
 *          
 *          The nav_pitch_cd target comes from:
 *          - TECS controller (for altitude/airspeed management in auto modes)
 *          - Navigation controller (for terrain following)
 *          - Flight mode logic (e.g., FBWA uses pilot stick input)
 *          - Landing flare logic (forces pitch to LAND_PITCH_DEG during flare)
 *          
 *          Takeoff Tail Hold:
 *          During the ground roll phase of automatic takeoffs, this function can hold the
 *          tail down by forcing a specific elevator deflection, improving directional control
 *          and preventing premature liftoff.
 *          
 *          Landing Flare:
 *          When flare conditions are met (flare switch high, throttle at zero, fixed-wing mode),
 *          overrides nav_pitch_cd with landing.get_pitch_cd() for proper landing attitude.
 * 
 * @note Called at main loop rate (typically 50-400Hz depending on board configuration)
 * @note Uses pitchController PID object configured via PITCH2SRV_* parameters
 * @note TECS may override pitch target for coordinated altitude/airspeed control
 * @note Speed scaler adjusts gains to maintain consistent handling across airspeeds
 * @note Output range: -4500 to +4500 (corresponding to ±45 degrees servo deflection)
 * 
 * @warning Critical flight control function - modifications require extensive flight testing
 * @warning Pitch control directly affects stall margins and must be thoroughly validated
 * @warning Tail hold timing critical - premature release can cause runway strikes
 * 
 * Source: ArduPlane/Attitude.cpp:166-232
 */
void Plane::stabilize_pitch()
{
    int8_t force_elevator = takeoff_tail_hold();
    if (force_elevator != 0) {
        // we are holding the tail down during takeoff. Just convert
        // from a percentage to a -4500..4500 centidegree angle
        SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, 45*force_elevator);
        return;
    }

    const float pitch_out = stabilize_pitch_get_pitch_out();
    SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, pitch_out);
}

float Plane::stabilize_pitch_get_pitch_out()
{
    const float speed_scaler = get_speed_scaler();
#if HAL_QUADPLANE_ENABLED
    if (!quadplane.use_fw_attitude_controllers()) {
        // use the VTOL rate for control, to ensure consistency
        const auto &pid_info = quadplane.attitude_control->get_rate_pitch_pid().get_pid_info();

        // scale FF to angle P
        if (quadplane.option_is_set(QuadPlane::OPTION::SCALE_FF_ANGLE_P)) {
            const float mc_angP = quadplane.attitude_control->get_angle_pitch_p().kP()
                * quadplane.attitude_control->get_last_angle_P_scale().y;
            if (is_positive(mc_angP)) {
                pitchController.set_ff_scale(MIN(1.0, 1.0 / (mc_angP * pitchController.tau())));
            }
        }

        const int32_t pitch_out = pitchController.get_rate_out(degrees(pid_info.target), speed_scaler);
        /* when slaving fixed wing control to VTOL control we need to decay the integrator to prevent
           opposing integrators balancing between the two controllers
        */
        pitchController.decay_I();
        return pitch_out;
    }
#endif
    // if LANDING_FLARE RCx_OPTION switch is set and in FW mode, manual throttle,throttle idle then set pitch to LAND_PITCH_DEG if flight option FORCE_FLARE_ATTITUDE is set
#if HAL_QUADPLANE_ENABLED
    const bool quadplane_in_frwd_transition = quadplane.in_frwd_transition();
#else
    const bool quadplane_in_frwd_transition = false;
#endif

    int32_t demanded_pitch = nav_pitch_cd + int32_t(g.pitch_trim * 100.0) + SRV_Channels::get_output_scaled(SRV_Channel::k_throttle) * g.kff_throttle_to_pitch;
    bool disable_integrator = false;
    if (control_mode == &mode_stabilize && channel_pitch->get_control_in() != 0) {
        disable_integrator = true;
    }
    /* force landing pitch if:
       - flare switch high
       - throttle stick at zero thrust
       - in fixed wing non auto-throttle mode
    */
    if (!quadplane_in_frwd_transition &&
        !control_mode->is_vtol_mode() &&
        !control_mode->does_auto_throttle() &&
        flare_mode == FlareMode::ENABLED_PITCH_TARGET &&
        throttle_at_zero()) {
        demanded_pitch = landing.get_pitch_cd();
    }

    return pitchController.get_servo_out(demanded_pitch - ahrs.pitch_sensor, speed_scaler, disable_integrator,
                                         ground_mode && !(plane.flight_option_enabled(FlightOptions::DISABLE_GROUND_PID_SUPPRESSION)));
}

/**
 * @brief Apply direct stick mixing of pilot inputs with stabilized control outputs
 * 
 * @details This function allows pilot stick inputs to directly mix with (override) the
 *          stabilization controller outputs in STABILIZE mode. This provides the pilot
 *          with direct control authority while maintaining stabilization assistance.
 *          
 *          Mixing Algorithm:
 *          1. Get current stabilized aileron/elevator outputs from controllers
 *          2. Call channel_roll->stick_mixing() and channel_pitch->stick_mixing()
 *             which blend pilot RC input with the stabilized output
 *          3. Write the mixed output back to servo channels
 *          
 *          The mixing is "direct" in that pilot stick position directly influences
 *          servo position, rather than commanding rates or angles that are then
 *          processed by outer control loops.
 *          
 *          This mode is active only when:
 *          - stick_mixing_enabled() returns true (valid RC, appropriate mode, etc.)
 *          - quadplane.allow_stick_mixing() returns true (for hybrid aircraft)
 *          - Currently in STABILIZE mode
 *          
 *          Mixing Amount:
 *          The actual mixing ratio is determined by the RC channel's stick_mixing()
 *          method, typically applying full pilot input when stick is moved and
 *          blending back to stabilized output as stick returns to center.
 * 
 * @note Only used in STABILIZE mode - other modes use stabilize_stick_mixing_fbw()
 * @note Future refactoring will move this to mode_stabilize.cpp for better code organization
 * @note Provides more direct "feel" compared to FBW-style stick mixing
 * 
 * @warning Aggressive stick inputs can override stabilization and lead to loss of control
 * @warning Mixing disabled automatically if RC signal lost or invalid
 * 
 * Source: ArduPlane/Attitude.cpp:238-255
 */
void ModeStabilize::stabilize_stick_mixing_direct()
{
    if (!plane.stick_mixing_enabled()) {
        return;
    }
#if HAL_QUADPLANE_ENABLED
    if (!plane.quadplane.allow_stick_mixing()) {
        return;
    }
#endif
    float aileron = SRV_Channels::get_output_scaled(SRV_Channel::k_aileron);
    aileron = plane.channel_roll->stick_mixing(aileron);
    SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, aileron);

    float elevator = SRV_Channels::get_output_scaled(SRV_Channel::k_elevator);
    elevator = plane.channel_pitch->stick_mixing(elevator);
    SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, elevator);
}

/**
 * @brief Apply FBW-style (Fly-By-Wire) stick mixing with rate-based control
 * 
 * @details This function implements a more sophisticated stick mixing approach used in
 *          automatic flight modes (AUTO, GUIDED, LOITER, etc.) that allows pilot override
 *          by modifying the nav_roll_cd and nav_pitch_cd targets rather than directly
 *          mixing with servo outputs.
 *          
 *          Roll Stick Mixing Algorithm:
 *          - Pilot stick input is treated as a bank angle command added to nav_roll_cd
 *          - Non-linear scaling: inputs > 50% are scaled 3x to allow full authority
 *          - Formula: For input > 0.5: roll_input = (3*input - 1)
 *          - Formula: For input < -0.5: roll_input = (3*input + 1)
 *          - Allows pilot to command up to 2x roll_limit_cd with full stick deflection
 *          - Final nav_roll_cd constrained to ±roll_limit_cd
 *          
 *          Pitch Stick Mixing Algorithm:
 *          - Similar non-linear scaling as roll
 *          - Respects pitch_limit_min and pitch_limit_max asymmetric limits
 *          - Automatically inverts pitch sense when flying inverted
 *          - Can be disabled via STICK_MIXING=FBW_NO_PITCH for altitude-only control
 *          - Disabled in LOITER if ENABLE_LOITER_ALT_CONTROL flight option set
 *          
 *          Active Modes:
 *          This stick mixing is NOT active in:
 *          - ACRO, FBWA, FBWB, CRUISE, TRAINING (manual/semi-manual modes)
 *          - QSTABILIZE, QHOVER, QLOITER, QLAND, QACRO, QAUTOTUNE (quadplane modes)
 *          - Any mode where stick_mixing_enabled() returns false
 *          
 *          The FBW-style approach provides smoother, more predictable pilot override
 *          compared to direct mixing, as the stabilization loops still close around
 *          the modified targets.
 * 
 * @note Called after navigation controller sets nav_roll_cd and nav_pitch_cd
 * @note Non-linear scaling ensures full control authority with stick deflection > 50%
 * @note Pitch mixing can be independently disabled via STICK_MIXING parameter
 * @note LOITER mode may disable pitch mixing if using altitude control feature
 * 
 * @warning Stick mixing bypasses navigation limits - pilot can command aggressive maneuvers
 * @warning Non-linear scaling region (>50% stick) requires pilot familiarity to avoid overcorrection
 * 
 * Source: ArduPlane/Attitude.cpp:261-319
 */
void Plane::stabilize_stick_mixing_fbw()
{
    if (!stick_mixing_enabled() ||
        control_mode == &mode_acro ||
        control_mode == &mode_fbwa ||
        control_mode == &mode_autotune ||
        control_mode == &mode_fbwb ||
        control_mode == &mode_cruise ||
#if HAL_QUADPLANE_ENABLED
        control_mode == &mode_qstabilize ||
        control_mode == &mode_qhover ||
        control_mode == &mode_qloiter ||
        control_mode == &mode_qland ||
        control_mode == &mode_qacro ||
#if QAUTOTUNE_ENABLED
        control_mode == &mode_qautotune ||
#endif
        !quadplane.allow_stick_mixing() ||
#endif  // HAL_QUADPLANE_ENABLED
        control_mode == &mode_training) {
        return;
    }
    // do FBW style roll stick mixing. We don't treat it linearly however. For
    // inputs up to half the maximum, we use linear addition to the nav_roll.
    // Above that it goes non-linear and ends up as 2x the maximum, to ensure
    // that the user can direct the plane in any direction with stick mixing.
    float roll_input = channel_roll->norm_input_dz();
    if (roll_input > 0.5f) {
        roll_input = (3*roll_input - 1);
    } else if (roll_input < -0.5f) {
        roll_input = (3*roll_input + 1);
    }
    nav_roll_cd += roll_input * roll_limit_cd;
    nav_roll_cd = constrain_int32(nav_roll_cd, -roll_limit_cd, roll_limit_cd);

    if (plane.g.stick_mixing == StickMixing::FBW_NO_PITCH) {
        return;
    }
    if ((control_mode == &mode_loiter) && (plane.flight_option_enabled(FlightOptions::ENABLE_LOITER_ALT_CONTROL))) {
        // loiter is using altitude control based on the pitch stick, don't use it again here
        return;
    }

    float pitch_input = channel_pitch->norm_input_dz();
    if (pitch_input > 0.5f) {
        pitch_input = (3*pitch_input - 1);
    } else if (pitch_input < -0.5f) {
        pitch_input = (3*pitch_input + 1);
    }
    if (fly_inverted()) {
        pitch_input = -pitch_input;
    }
    if (pitch_input > 0) {
        nav_pitch_cd += pitch_input * aparm.pitch_limit_max*100;
    } else {
        nav_pitch_cd += -(pitch_input * pitch_limit_min*100);
    }
    nav_pitch_cd = constrain_int32(nav_pitch_cd, pitch_limit_min*100, aparm.pitch_limit_max.get()*100);
}


/**
 * @brief Execute yaw axis stabilization control loop
 * 
 * @details This function manages yaw/rudder control with three distinct operational modes
 *          depending on flight phase and altitude:
 *          
 *          **Mode 1: Coordinated Turn (In-Flight)**
 *          - Uses calc_nav_yaw_coordinated() for rudder coordination
 *          - Applies yaw damping and turn coordination
 *          - Includes rudder mixing from aileron (KFF_RUDD_MIX parameter)
 *          - Active when above GROUND_STEER_ALT and roll stick has input
 *          
 *          **Mode 2: Course Hold Ground Steering**
 *          - Uses calc_nav_yaw_course() to hold specific ground heading
 *          - Active during landing flare or when below GROUND_STEER_ALT
 *          - Tracks navigation bearing error for runway tracking
 *          - Used for automatic takeoff and landing directional control
 *          
 *          **Mode 3: Rate-Based Ground Steering**
 *          - Uses calc_nav_yaw_ground() for pilot-commanded steering rate
 *          - Pilot rudder input commands turning rate in deg/s (GROUND_STEER_DPS)
 *          - Locks course when stick centered to prevent wandering
 *          - Active when below GROUND_STEER_ALT with no roll input
 *          
 *          Mode Selection Logic:
 *          - landing.is_flaring() → Course hold ground steering
 *          - Below GROUND_STEER_ALT + no roll input → Ground steering (course or rate)
 *          - Otherwise → Coordinated turn (airborne yaw control)
 *          
 *          Output Routing:
 *          - If ground steering active and steering channel assigned:
 *            * k_rudder gets coordinated rudder output
 *            * k_steering gets ground steering output
 *          - Otherwise:
 *            * Both k_rudder and k_steering get the same output
 * 
 * @note Called at main loop rate (typically 50-400Hz depending on board configuration)
 * @note Ground steering uses steerController, airborne uses yawController
 * @note GROUND_STEER_ALT parameter defines transition altitude (typically 5-10m AGL)
 * @note Yaw control is less critical than roll/pitch for most fixed-wing aircraft
 * @note Quadplane spin recovery assistance may override outputs (line 384)
 * 
 * @warning Ground steering during landing requires careful tuning to prevent runway departure
 * @warning Incorrect mode transitions can cause oscillations or loss of directional control
 * 
 * Source: ArduPlane/Attitude.cpp:329-386
 */
void Plane::stabilize_yaw()
{
    bool ground_steering = false;
    if (landing.is_flaring()) {
        // in flaring then enable ground steering
        ground_steering = true;
    } else {
        // otherwise use ground steering when no input control and we
        // are below the GROUND_STEER_ALT
        ground_steering = (channel_roll->get_control_in() == 0 && 
                                            fabsf(relative_altitude) < g.ground_steer_alt);
        if (!landing.is_ground_steering_allowed()) {
            // don't use ground steering on landing approach
            ground_steering = false;
        }
    }


    /*
      first calculate steering for a nose or tail
      wheel. We use "course hold" mode for the rudder when either performing
      a flare (when the wings are held level) or when in course hold in
      FBWA mode (when we are below GROUND_STEER_ALT)
     */
    float steering_output = 0.0;
    if (landing.is_flaring() ||
        (steer_state.hold_course_cd != -1 && ground_steering)) {
        steering_output = calc_nav_yaw_course();
    } else if (ground_steering) {
        steering_output = calc_nav_yaw_ground();
    }

    /*
      now calculate rudder for the rudder
     */
    const float rudder_output = calc_nav_yaw_coordinated();

    if (!ground_steering) {
        // Not doing ground steering, output rudder on steering channel
        SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, rudder_output);
        SRV_Channels::set_output_scaled(SRV_Channel::k_steering, rudder_output);

    } else if (!SRV_Channels::function_assigned(SRV_Channel::k_steering)) {
        // Ground steering active but no steering output configured, output steering on rudder channel
        SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, steering_output);
        SRV_Channels::set_output_scaled(SRV_Channel::k_steering, steering_output);

    } else {
        // Ground steering with both steering and rudder channels
        SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, rudder_output);
        SRV_Channels::set_output_scaled(SRV_Channel::k_steering, steering_output);
    }

#if HAL_QUADPLANE_ENABLED
    // possibly recover from a spin
    quadplane.assist.output_spin_recovery();
#endif
}

/*
  main stabilization function for all 3 axes
 */
void Plane::stabilize()
{
    uint32_t now = AP_HAL::millis();
#if HAL_QUADPLANE_ENABLED
    if (quadplane.available()) {
        quadplane.transition->set_FW_roll_pitch(nav_pitch_cd, nav_roll_cd);
    }
#endif

    if (now - last_stabilize_ms > 2000) {
        // if we haven't run the rate controllers for 2 seconds then reset
        control_mode->reset_controllers();
    }
    last_stabilize_ms = now;

    if (control_mode == &mode_training ||
            control_mode == &mode_manual) {
        plane.control_mode->run();
#if AP_SCRIPTING_ENABLED
    } else if (nav_scripting_active()) {
        // scripting is in control of roll and pitch rates and throttle
        const float speed_scaler = get_speed_scaler();
        const float aileron = rollController.get_rate_out(nav_scripting.roll_rate_dps, speed_scaler);
        const float elevator = pitchController.get_rate_out(nav_scripting.pitch_rate_dps, speed_scaler);
        SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, aileron);
        SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, elevator);
        float rudder = 0;
        if (yawController.rate_control_enabled()) {
            rudder = nav_scripting.rudder_offset_pct * 45;
            if (nav_scripting.run_yaw_rate_controller) {
                rudder += yawController.get_rate_out(nav_scripting.yaw_rate_dps, speed_scaler, false);
            } else {
                yawController.reset_I();
            }
        }
        SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, rudder);
        SRV_Channels::set_output_scaled(SRV_Channel::k_steering, rudder);
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, plane.nav_scripting.throttle_pct);
#endif
    } else {
        plane.control_mode->run();
    }

    /*
      see if we should zero the attitude controller integrators. 
     */
    if (is_zero(get_throttle_input()) &&
        fabsf(relative_altitude) < 5.0f && 
        fabsf(barometer.get_climb_rate()) < 0.5f &&
        ahrs.groundspeed() < 3) {
        // we are low, with no climb rate, and zero throttle, and very
        // low ground speed. Zero the attitude controller
        // integrators. This prevents integrator buildup pre-takeoff.
        rollController.reset_I();
        pitchController.reset_I();
        yawController.reset_I();

        // if moving very slowly also zero the steering integrator
        if (ahrs.groundspeed() < 1) {
            steerController.reset_I();            
        }
    }
}


/*
 * Set the throttle output.
 * This is called by TECS-enabled flight modes, e.g. AUTO, GUIDED, etc.
*/
void Plane::calc_throttle()
{
    if (aparm.throttle_cruise <= 1) {
        // user has asked for zero throttle - this may be done by a
        // mission which wants to turn off the engine for a parachute
        // landing
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0.0);
        return;
    }

    // Read the TECS throttle output and set it to the throttle channel.
    float commanded_throttle = TECS_controller.get_throttle_demand();
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, commanded_throttle);
}

/*****************************************
* Calculate desired roll/pitch/yaw angles (in medium freq loop)
*****************************************/

/**
 * @brief Calculate coordinated turn rudder output for in-flight yaw control
 * 
 * @details Computes rudder commands to maintain coordinated flight (minimizing sideslip)
 *          during turns and to provide yaw damping for stability. This function implements
 *          several control strategies depending on flight mode and conditions:
 *          
 *          **Standard Coordinated Turn Mode:**
 *          - Uses yawController.get_servo_out() with airspeed compensation
 *          - Automatically coordinates turns to minimize sideslip angle
 *          - Applies yaw damping to prevent Dutch roll oscillations
 *          - Adds rudder mixing from aileron input (g.kff_rudder_mix)
 *          - Adds direct pilot rudder input for manual trim
 *          
 *          **Rate Controller Mode (Autotune):**
 *          - Active when autotuning with yaw rate control enabled (ACRO_YAW_RATE > 0)
 *          - Converts pilot rudder stick to yaw rate command
 *          - Adds coordinated turn rate to ease flying during tuning
 *          - Uses yawController.get_rate_out() for rate tracking
 *          
 *          **Guided Mode Override:**
 *          - If external guidance command received within last 3 seconds
 *          - Directly uses commanded rudder from guided_state.forced_rpy_cd.z
 *          - Allows external controllers (companion computer) to command rudder
 *          
 *          Speed Compensation:
 *          The speed_scaler from get_speed_scaler() adjusts yaw damper and coordination
 *          gains based on airspeed, maintaining consistent handling across the flight envelope.
 *          
 *          Integrator Management:
 *          - Disabled in STABILIZE mode when pilot provides rudder input
 *          - Automatically reset when switching between angle and rate control
 * 
 * @return int16_t Rudder output command in centidegrees (-4500 to +4500)
 *         Positive values = right rudder (yaw right)
 *         Negative values = left rudder (yaw left)
 * 
 * @note Called from stabilize_yaw() when in airborne coordinated flight mode
 * @note Uses yawController PID object configured via YAW2SRV_* parameters
 * @note Rudder mixing coefficient KFF_RUDD_MIX typically 0.1 to 0.5
 * @note The coordination algorithm computes required yaw rate from roll angle and airspeed
 * 
 * @warning Excessive rudder mixing can cause adverse yaw and departures from coordinated flight
 * @warning Rate controller must be properly tuned before use in autotune mode
 * 
 * Source: ArduPlane/Attitude.cpp:482-524
 */
int16_t Plane::calc_nav_yaw_coordinated()
{
    const float speed_scaler = get_speed_scaler();
    bool disable_integrator = false;
    int16_t rudder_in = rudder_input();

    int16_t commanded_rudder;
    bool using_rate_controller = false;

    // Received an external msg that guides yaw in the last 3 seconds?
    if (control_mode->is_guided_mode() &&
            plane.guided_state.last_forced_rpy_ms.z > 0 &&
            millis() - plane.guided_state.last_forced_rpy_ms.z < 3000) {
        commanded_rudder = plane.guided_state.forced_rpy_cd.z;
    } else if (autotuning && g.acro_yaw_rate > 0 && yawController.rate_control_enabled()) {
        // user is doing an AUTOTUNE with yaw rate control
        const float rudd_expo = rudder_in_expo(true);
        const float yaw_rate = (rudd_expo/SERVO_MAX) * g.acro_yaw_rate;
        // add in the coordinated turn yaw rate to make it easier to fly while tuning the yaw rate controller
        const float coordination_yaw_rate = degrees(GRAVITY_MSS * tanf(cd_to_rad(nav_roll_cd))/MAX(aparm.airspeed_min,smoothed_airspeed));
        commanded_rudder = yawController.get_rate_out(yaw_rate+coordination_yaw_rate,  speed_scaler, false);
        using_rate_controller = true;
    } else {
        if (control_mode == &mode_stabilize && rudder_in != 0) {
            disable_integrator = true;
        }

        commanded_rudder = yawController.get_servo_out(speed_scaler, disable_integrator);

        // add in rudder mixing from roll
        commanded_rudder += SRV_Channels::get_output_scaled(SRV_Channel::k_aileron) * g.kff_rudder_mix;
        commanded_rudder += rudder_in;
    }

    if (!using_rate_controller) {
        /*
          When not running the yaw rate controller, we need to reset the rate
        */
        yawController.reset_rate_PID();
    }

    return constrain_int16(commanded_rudder, -4500, 4500);
}

/*
  calculate yaw control for ground steering with specific course
 */
int16_t Plane::calc_nav_yaw_course(void)
{
    // holding a specific navigation course on the ground. Used in
    // auto-takeoff and landing
    int32_t bearing_error_cd = nav_controller->bearing_error_cd();
    int16_t steering = steerController.get_steering_out_angle_error(bearing_error_cd);
    if (stick_mixing_enabled()) {
        steering = channel_rudder->stick_mixing(steering);
    }
    return constrain_int16(steering, -4500, 4500);
}

/*
  calculate yaw control for ground steering
 */
int16_t Plane::calc_nav_yaw_ground(void)
{
    if (gps.ground_speed() < 1 && 
        is_zero(get_throttle_input()) &&
        flight_stage != AP_FixedWing::FlightStage::TAKEOFF &&
        flight_stage != AP_FixedWing::FlightStage::ABORT_LANDING) {
        // manual rudder control while still
        steer_state.locked_course = false;
        steer_state.locked_course_err = 0;
        return rudder_input();
    }

    // if we haven't been steering for 1s then clear locked course
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - steer_state.last_steer_ms > 1000) {
        steer_state.locked_course = false;
    }
    steer_state.last_steer_ms = now_ms;

    float steer_rate = (rudder_input()/4500.0f) * g.ground_steer_dps;
    if (flight_stage == AP_FixedWing::FlightStage::TAKEOFF ||
        flight_stage == AP_FixedWing::FlightStage::ABORT_LANDING) {
        steer_rate = 0;
    }
    if (!is_zero(steer_rate)) {
        // pilot is giving rudder input
        steer_state.locked_course = false;        
    } else if (!steer_state.locked_course) {
        // pilot has released the rudder stick or we are still - lock the course
        steer_state.locked_course = true;
        if (flight_stage != AP_FixedWing::FlightStage::TAKEOFF &&
            flight_stage != AP_FixedWing::FlightStage::ABORT_LANDING) {
            steer_state.locked_course_err = 0;
        }
    }

    int16_t steering;
    if (!steer_state.locked_course) {
        // use a rate controller at the pilot specified rate
        steering = steerController.get_steering_out_rate(steer_rate);
    } else {
        // use a error controller on the summed error
        int32_t yaw_error_cd = -degrees(steer_state.locked_course_err)*100;
        steering = steerController.get_steering_out_angle_error(yaw_error_cd);
    }
    return constrain_int16(steering, -4500, 4500);
}


/*
  calculate a new nav_pitch_cd from the speed height controller
 */
void Plane::calc_nav_pitch()
{
    int32_t commanded_pitch = TECS_controller.get_pitch_demand();
    nav_pitch_cd = constrain_int32(commanded_pitch, pitch_limit_min*100, aparm.pitch_limit_max.get()*100);
}


/*
  calculate a new nav_roll_cd from the navigation controller
 */
void Plane::calc_nav_roll()
{
    int32_t commanded_roll = nav_controller->nav_roll_cd();
    nav_roll_cd = constrain_int32(commanded_roll, -roll_limit_cd, roll_limit_cd);
    update_load_factor();
}

/*
  adjust nav_pitch_cd for STAB_PITCH_DOWN_CD. This is used to make
  keeping up good airspeed in FBWA mode easier, as the plane will
  automatically pitch down a little when at low throttle. It makes
  FBWA landings without stalling much easier.
 */
void Plane::adjust_nav_pitch_throttle(void)
{
    int8_t throttle = throttle_percentage();
    if (throttle >= 0 && throttle < aparm.throttle_cruise && flight_stage != AP_FixedWing::FlightStage::VTOL) {
        float p = (aparm.throttle_cruise - throttle) / (float)aparm.throttle_cruise;
        nav_pitch_cd -= g.stab_pitch_down * 100.0f * p;
    }
}


/*
  calculate a new aerodynamic_load_factor and limit nav_roll_cd to
  ensure that the load factor does not take us below the sustainable
  airspeed
 */
void Plane::update_load_factor(void)
{
    float demanded_roll = fabsf(nav_roll_cd*0.01f);
    if (demanded_roll > 85) {
        // limit to 85 degrees to prevent numerical errors
        demanded_roll = 85;
    }

    // loadFactor = liftForce / gravityForce, where gravityForce = liftForce * cos(roll) on balanced horizontal turn
    aerodynamic_load_factor = 1.0f / cosf(radians(demanded_roll));

#if HAL_QUADPLANE_ENABLED
    if (quadplane.available() && quadplane.transition->set_FW_roll_limit(roll_limit_cd)) {
        nav_roll_cd = constrain_int32(nav_roll_cd, -roll_limit_cd, roll_limit_cd);
        return;
    }
#endif

    if (!aparm.stall_prevention) {
        // stall prevention is disabled
        return;
    }
    if (fly_inverted()) {
        // no roll limits when inverted
        return;
    }
#if HAL_QUADPLANE_ENABLED
    if (quadplane.tailsitter.active()) {
        // no limits while hovering
        return;
    }
#endif

    float stall_airspeed_1g = is_positive(aparm.airspeed_stall)
                                  ? aparm.airspeed_stall
                                  : aparm.airspeed_min;

    float max_load_factor =
        sq(smoothed_airspeed / MAX(stall_airspeed_1g, 1));

    if (max_load_factor <= 1) {
        // our airspeed is below the minimum airspeed. Limit roll to
        // 25 degrees
        nav_roll_cd = constrain_int32(nav_roll_cd, -2500, 2500);
        roll_limit_cd = MIN(roll_limit_cd, 2500);
    } else if (max_load_factor < aerodynamic_load_factor) {
        // the demanded nav_roll would take us past the aerodynamic
        // load limit. Limit our roll to a bank angle that will keep
        // the load within what the airframe can handle. We always
        // allow at least 25 degrees of roll however, to ensure the
        // aircraft can be manoeuvered with a bad airspeed estimate. At
        // 25 degrees the load factor is 1.1 (10%)
        int32_t roll_limit = degrees(acosf(1.0f / max_load_factor))*100;
        if (roll_limit < 2500) {
            roll_limit = 2500;
        }
        nav_roll_cd = constrain_int32(nav_roll_cd, -roll_limit, roll_limit);
        roll_limit_cd = MIN(roll_limit_cd, roll_limit);
    }
}
