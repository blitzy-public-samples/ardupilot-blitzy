/**
 * @file navigation.cpp
 * @brief Navigation algorithms for ArduPlane including L1 controller integration, waypoint tracking, and loiter patterns
 * 
 * @details This file implements core navigation functionality for fixed-wing aircraft:
 * - L1 guidance controller integration for waypoint tracking and loiter circles
 * - Navigation target management and distance calculations
 * - Loiter pattern execution with configurable radius and direction
 * - Airspeed control integration with TECS (Total Energy Control System)
 * - Ground speed undershoot compensation for minimum groundspeed requirements
 * - Altitude and speed management for FBWB/CRUISE modes
 * - Mission turn angle calculation for path following
 * 
 * The navigation system uses the L1 controller for predictive path following,
 * providing smooth turns, wind compensation, and crosstrack error correction.
 * All navigation operates in the NED (North-East-Down) coordinate frame.
 * 
 * @note This file is part of the ArduPlane vehicle code and integrates with:
 * - AP_L1_Control for guidance algorithms
 * - AP_TECS for energy management
 * - Control modes for mode-specific navigation behavior
 * - Mission library for waypoint and command execution
 * 
 * Source: ArduPlane/navigation.cpp
 */
#include "Plane.h"

/**
 * @brief Reset the total loiter angle tracking
 * 
 * @details Resets all loiter angle tracking variables to initial state:
 * - Clears accumulated loiter angle (sum_cd and total_cd)
 * - Resets target altitude achievement flags
 * - Called when starting a new loiter or when loiter parameters change
 * 
 * @note Loiter angles are tracked in centidegrees for precision
 */
void Plane::loiter_angle_reset(void)
{
    loiter.sum_cd = 0;
    loiter.total_cd = 0;
    loiter.reached_target_alt = false;
    loiter.unable_to_acheive_target_alt = false;
}

/**
 * @brief Update the total angle covered in a loiter pattern
 * 
 * @details Tracks cumulative angle traversed during loiter to support mission
 * commands that require N complete circles before proceeding. Functionality:
 * - Accumulates bearing changes from L1 controller target_bearing
 * - Handles loiter direction (clockwise vs counter-clockwise)
 * - Monitors altitude achievement during loiter
 * - Detects inability to reach target altitude after multiple laps
 * 
 * Algorithm:
 * 1. Calculate bearing change since last update (loiter_delta_cd)
 * 2. Apply direction multiplier and accumulate to sum_cd
 * 3. Check if position target reached (horizontal loiter accuracy)
 * 4. Once position reached, monitor altitude target achievement
 * 5. If altitude not reached after lap_check_interval (3 full circles),
 *    flag as unable to achieve target altitude
 * 
 * @note Called at navigation update rate from navigate()
 * @note Altitude checking supports both AMSL and terrain-relative targets
 * @note Angles in centidegrees, altitude in centimeters
 * 
 * @see reached_loiter_target()
 * @see nav_controller->target_bearing_cd()
 */
void Plane::loiter_angle_update(void)
{
    // Check altitude every 3 complete laps (1080 degrees = 3 * 360 degrees)
    static const int32_t lap_check_interval_cd = 3*36000;

    // Get current bearing to loiter center from L1 controller
    const int32_t target_bearing_cd = nav_controller->target_bearing_cd();
    int32_t loiter_delta_cd;
    const bool reached_target = reached_loiter_target();

    // Calculate bearing change since last update
    if (loiter.sum_cd == 0 && !reached_target) {
        // We don't start summing until we are doing the real loiter
        // (still approaching loiter point)
        loiter_delta_cd = 0;
    } else if (loiter.sum_cd == 0) {
        // First update after reaching loiter point - use 1 cd for initial delta
        // to bootstrap the tracking system
        loiter_delta_cd = 1;
        loiter.start_lap_alt_cm = current_loc.alt;
        loiter.next_sum_lap_cd = lap_check_interval_cd;
    } else {
        // Normal operation: calculate bearing change since last update
        loiter_delta_cd = target_bearing_cd - loiter.old_target_bearing_cd;
    }

    loiter.old_target_bearing_cd = target_bearing_cd;
    // Wrap bearing change to [-180, +180] degrees to handle heading wrap-around
    loiter_delta_cd = wrap_180_cd(loiter_delta_cd);
    // Apply loiter direction multiplier (+1 for CW, -1 for CCW) and accumulate
    loiter.sum_cd += loiter_delta_cd * loiter.direction;

    bool reached_target_alt = false;

    // Check altitude achievement once horizontal position target is reached
    if (reached_target) {
        // Once we reach the position target we start checking the altitude target
        bool terrain_status_ok = false;
#if AP_TERRAIN_AVAILABLE
        /*
         * Terrain-relative altitude checking:
         * If terrain following is enabled, compare current altitude AGL (Above Ground Level)
         * to the target terrain-relative altitude instead of AMSL (Above Mean Sea Level)
         */
        float altitude_agl = 0;
        if (target_altitude.terrain_following) {
            if (terrain.status() == AP_Terrain::TerrainStatusOK &&
                terrain.height_above_terrain(altitude_agl, true)) {
                terrain_status_ok = true;
            }
        }
        // Altitude tolerance: 5 meters for terrain-relative altitude
        if (terrain_status_ok &&
            fabsF(altitude_agl - target_altitude.terrain_alt_cm*0.01) < 5) {
            reached_target_alt = true;
        } else
#endif
        // AMSL altitude checking: tolerance of 500 cm (5 meters)
        if (!terrain_status_ok && labs(current_loc.alt - target_altitude.amsl_cm) < 500) {
            reached_target_alt = true;
        }
    }

    // Update loiter altitude achievement state
    if (reached_target_alt) {
        loiter.reached_target_alt = true;
        loiter.unable_to_acheive_target_alt = false;
        // Schedule next altitude check after another lap_check_interval
        loiter.next_sum_lap_cd = loiter.sum_cd + lap_check_interval_cd;

    } else if (!loiter.reached_target_alt && labs(loiter.sum_cd) >= loiter.next_sum_lap_cd) {
        /*
         * Check every few laps (lap_check_interval_cd) for scenario where updrafts/downdrafts
         * or configuration issues prevent reaching target altitude.
         * If altitude hasn't changed more than 5m in 3 complete circles, flag as unable to achieve.
         * This prevents infinite loitering when target altitude is unreachable.
         */
        loiter.unable_to_acheive_target_alt = labs(current_loc.alt - loiter.start_lap_alt_cm) < 500;
        loiter.start_lap_alt_cm = current_loc.alt;
        loiter.next_sum_lap_cd += lap_check_interval_cd;
    }
}

/**
 * @brief Main navigation update function - calculates desired direction and distance to target
 * 
 * @details Primary navigation function called from the main control loop. Executes current
 * navigation command based on flight mode and mission requirements. Key responsibilities:
 * 
 * - Validates position estimate availability before navigation
 * - Calculates waypoint distance and path proportion for TECS controller
 * - Updates loiter angle tracking for mission commands requiring N circles
 * - Delegates to mode-specific navigate() for control calculations
 * - Integrates with L1 controller for lateral guidance
 * - Monitors home altitude changes for accurate return-to-launch
 * 
 * Navigation Flow:
 * 1. Check position validity (GPS/EKF health)
 * 2. Verify next waypoint is initialized
 * 3. Calculate distance and path proportion to next waypoint
 * 4. Update TECS with path proportion for energy management
 * 5. Update loiter angle tracking
 * 6. Call mode-specific navigation implementation
 * 
 * @note Called at navigation update rate (typically 10-50Hz depending on vehicle configuration)
 * @note Does not execute if position estimate is unavailable (GPS loss or EKF failure)
 * @note Waypoint distances in meters, angles in centidegrees
 * 
 * @warning Navigation is disabled if have_position is false - vehicle will continue last
 * commanded attitude/throttle until position is reacquired
 * 
 * @see control_mode->navigate() for mode-specific navigation implementation
 * @see loiter_angle_update() for loiter tracking
 * @see AP_L1_Control for L1 guidance algorithm
 * @see AP_TECS for energy management integration
 */
void Plane::navigate()
{
    // Safety check: do not navigate with corrupt position data
    // Navigation requires valid GPS fix or alternative position source (optical flow, beacon, etc.)
    if (!have_position) {
        return;
    }

    // Verify next waypoint is initialized (non-zero lat/lng)
    // Uninitialized waypoints indicate no active navigation target
    if (next_WP_loc.lat == 0 && next_WP_loc.lng == 0) {
        return;
    }

    // Monitor for home altitude changes (e.g., barometer drift correction)
    // Critical for accurate RTL altitude calculation
    check_home_alt_change();

    /*
     * Calculate waypoint distance and path proportion:
     * - wp_distance: Direct distance from current position to next waypoint (meters)
     * - wp_proportion: Progress along line from prev_WP to next_WP (0.0 = at prev, 1.0 = at next, >1.0 = past next)
     * 
     * Path proportion is used by TECS for altitude management and by L1 controller
     * for determining when to start turn anticipation for the next waypoint
     */
    auto_state.wp_distance = current_loc.get_distance(next_WP_loc);
    auto_state.wp_proportion = current_loc.line_path_proportion(prev_WP_loc, next_WP_loc);
    TECS_controller.set_path_proportion(auto_state.wp_proportion);

    // Update loiter angle tracking for mission commands requiring N complete circles
    loiter_angle_update();

    /*
     * Delegate to mode-specific navigation implementation:
     * Mode-specific navigate() calls will update the L1 controller with appropriate commands:
     * - AUTO/GUIDED: nav_controller->update_waypoint() for waypoint tracking
     * - LOITER: nav_controller->update_loiter() for circular patterns
     * - RTL: Combination of waypoint and loiter depending on RTL phase
     * - CIRCLE: nav_controller->update_loiter() with configured radius
     * 
     * L1 controller integration provides:
     * - Predictive path following with smooth turns
     * - Crosstrack error correction
     * - Wind compensation
     * - Roll demand calculation for lateral control
     */
    control_mode->navigate();
}

/**
 * @brief Get target airspeed for AUTO mode in centimeters per second
 * 
 * @details Determines appropriate target airspeed for AUTO mode considering:
 * - Quadplane VTOL landing approach (if enabled)
 * - DO_CHANGE_SPEED mission command overrides
 * - Default cruise airspeed parameter
 * 
 * Airspeed selection priority:
 * 1. Quadplane landing airspeed during fixed-wing spiral or VTOL approach
 * 2. Mission command specified airspeed (new_airspeed_cm from DO_CHANGE_SPEED)
 * 3. Default cruise airspeed (AIRSPEED_CRUISE parameter)
 * 
 * @return Target airspeed in centimeters per second (cm/s)
 * 
 * @note Intended for use by calc_airspeed_errors() only
 * @note Airspeed in cm/s for consistency with ArduPlane unit conventions
 * 
 * @see calc_airspeed_errors() for airspeed error calculation
 * @see AP_TECS::get_land_airspeed() for landing airspeed management
 */
float Plane::mode_auto_target_airspeed_cm()
{
#if HAL_QUADPLANE_ENABLED
    if (quadplane.landing_with_fixed_wing_spiral_approach() &&
        ((vtol_approach_s.approach_stage == VTOLApproach::Stage::APPROACH_LINE) ||
         (vtol_approach_s.approach_stage == VTOLApproach::Stage::VTOL_LANDING))) {
        const float land_airspeed = TECS_controller.get_land_airspeed();
        if (is_positive(land_airspeed)) {
            return land_airspeed * 100;
        }
        // fallover to normal airspeed
        return aparm.airspeed_cruise*100;
    }
    if (quadplane.in_vtol_land_approach()) {
        return quadplane.get_land_airspeed() * 100;
    }
#endif

    // normal AUTO mode and new_airspeed variable was set by
    // DO_CHANGE_SPEED command while in AUTO mode
    if (new_airspeed_cm > 0) {
        return new_airspeed_cm;
    }

    // fallover to normal airspeed
    return aparm.airspeed_cruise*100;
}

/**
 * @brief Calculate airspeed errors and update target airspeed based on flight mode and conditions
 * 
 * @details Comprehensive airspeed management function that:
 * - Updates smoothed airspeed estimate using exponential filtering
 * - Determines mode-specific target airspeed (FBWB, CRUISE, AUTO, GUIDED, LAND, etc.)
 * - Applies ground speed undershoot compensation for minimum groundspeed requirements
 * - Handles throttle nudging for pilot airspeed adjustments
 * - Applies airspeed limits (min/max) and stall protection
 * - Calculates final airspeed error for TECS controller
 * 
 * Mode-specific airspeed targets:
 * - FBWB/CRUISE: Pilot throttle input mapped to airspeed range or fixed cruise speed
 * - AUTO: Mission-commanded airspeed or cruise default
 * - GUIDED: Offboard commanded airspeed with slew rate limiting (optional)
 * - LAND: Landing-specific airspeed reduction profile
 * - SOARING: Thermalling or cruising airspeed based on throttle suppression state
 * - CIRCLE/LOITER: Cruise airspeed
 * 
 * Ground speed compensation:
 * Adds airspeed to compensate for headwinds when groundspeed falls below MIN_GROUNDSPEED
 * Uses EAS (Equivalent Airspeed) calculation to account for air density
 * 
 * @note Called at main loop rate from control updates
 * @note All airspeed values in cm/s (centimeters per second)
 * @note Uses AHRS airspeed estimate which may be synthetic if sensor unavailable
 * @note Smoothed airspeed uses 20% new measurement, 80% previous (low-pass filter)
 * 
 * @see AP_TECS for energy management using airspeed error
 * @see calc_gndspeed_undershoot() for groundspeed compensation calculation
 */
void Plane::calc_airspeed_errors()
{
    /*
     * Get the airspeed estimate and update smoothed airspeed:
     * NOTE: We use the airspeed_estimate function not direct sensor reading
     *       because TECS may be using synthetic airspeed (GPS-derived) when
     *       airspeed sensor is unavailable or unhealthy
     */
    float airspeed_measured = 0.1;
    if (ahrs.airspeed_estimate(airspeed_measured)) {
        // Low-pass filter: 20% new measurement + 80% previous value
        // Minimum value of 0.1 m/s prevents division by zero
        smoothed_airspeed = MAX(0.1, smoothed_airspeed * 0.8f + airspeed_measured * 0.2f);
    }

    /*
     * Low-pass filter speed scaler for control surface effectiveness:
     * Speed scaler accounts for reduced control authority at low airspeeds
     * Used to scale control surface deflections (higher deflections at low speed)
     * Filter cutoff: 2Hz to smooth rapid airspeed variations
     * Sample rate: 10Hz (dt = 0.1s)
     */
    const float speed_scaler = calc_speed_scaler();
    const float cutoff_Hz = 2.0;
    const float dt = 0.1;
    surface_speed_scaler += calc_lowpass_alpha_dt(dt, cutoff_Hz) * (speed_scaler - surface_speed_scaler);


    // FBW_B/cruise airspeed target
    if (!failsafe.rc_failsafe && (control_mode == &mode_fbwb || control_mode == &mode_cruise)) {
        if (flight_option_enabled(FlightOptions::CRUISE_TRIM_AIRSPEED)) {
            target_airspeed_cm = aparm.airspeed_cruise*100;
        } else if (flight_option_enabled(FlightOptions::CRUISE_TRIM_THROTTLE)) {
            float control_min = 0.0f;
            float control_mid = 0.0f;
            const float control_max = channel_throttle->get_range();
            const float control_in = get_throttle_input();
            switch (channel_throttle->get_type()) {
            case RC_Channel::ControlType::ANGLE:
                    control_min = -control_max;
                    break;
            case RC_Channel::ControlType::RANGE:
                    control_mid = channel_throttle->get_control_mid();
                    break;
            }
            if (control_in <= control_mid) {
                target_airspeed_cm = linear_interpolate(aparm.airspeed_min * 100, aparm.airspeed_cruise*100,
                                                        control_in,
                                                        control_min, control_mid);
            } else {
                target_airspeed_cm = linear_interpolate(aparm.airspeed_cruise*100, aparm.airspeed_max * 100,
                                                        control_in,
                                                        control_mid, control_max);
            }
        } else {
            target_airspeed_cm = ((int32_t)(aparm.airspeed_max - aparm.airspeed_min) *
                                  get_throttle_input()) + ((int32_t)aparm.airspeed_min * 100);
        }
#if AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED
    } else if (control_mode == &mode_guided && guided_state.target_airspeed_cm >  0.0) { // if offboard guided speed change cmd not set, then this section is skipped
        // offboard airspeed demanded
        uint32_t now = AP_HAL::millis();
        float delta = 1e-3f * (now - guided_state.target_airspeed_time_ms);
        guided_state.target_airspeed_time_ms = now;
        float delta_amt = 100 * delta * guided_state.target_airspeed_accel;
        target_airspeed_cm += delta_amt;

        //target_airspeed_cm recalculated then clamped to between MIN airspeed and MAX airspeed by constrain_float
        if (is_positive(guided_state.target_airspeed_accel)) {
            target_airspeed_cm = constrain_float(MIN(guided_state.target_airspeed_cm, target_airspeed_cm), aparm.airspeed_min *100, aparm.airspeed_max *100);
        } else {
            target_airspeed_cm = constrain_float(MAX(guided_state.target_airspeed_cm, target_airspeed_cm), aparm.airspeed_min *100, aparm.airspeed_max *100);
        }

#endif // AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED

#if HAL_SOARING_ENABLED
    } else if (g2.soaring_controller.is_active() && g2.soaring_controller.get_throttle_suppressed()) {
        if (control_mode == &mode_thermal) {
            float arspd = g2.soaring_controller.get_thermalling_target_airspeed();

            if (arspd > 0) {
                target_airspeed_cm = arspd * 100;
            } else {
                target_airspeed_cm = aparm.airspeed_cruise*100;
            }
        } else if (control_mode == &mode_auto) {
            float arspd = g2.soaring_controller.get_cruising_target_airspeed();

            if (arspd > 0) {
                target_airspeed_cm = arspd * 100;
            } else {
                target_airspeed_cm = aparm.airspeed_cruise*100;
            }
        }
#endif

    } else if (flight_stage == AP_FixedWing::FlightStage::LAND) {
        // Landing airspeed target
        target_airspeed_cm = landing.get_target_airspeed_cm();
    } else if (control_mode == &mode_guided && new_airspeed_cm > 0) { //DO_CHANGE_SPEED overrides onboard guided speed commands, user would have re-enter guided mode to revert
                       target_airspeed_cm = new_airspeed_cm;
    } else if (control_mode == &mode_auto) {
        target_airspeed_cm = mode_auto_target_airspeed_cm();
#if HAL_QUADPLANE_ENABLED
    } else if (control_mode == &mode_qrtl && quadplane.in_vtol_land_approach()) {
        target_airspeed_cm = quadplane.get_land_airspeed() * 100;
#endif
    } else {
        // Normal airspeed target for all other cases
        target_airspeed_cm = aparm.airspeed_cruise*100;
    }

    // Set target to current airspeed + ground speed undershoot,
    // but only when this is faster than the target airspeed commanded
    // above.
    if (control_mode->does_auto_throttle() &&
        groundspeed_undershoot_is_valid &&
        control_mode != &mode_circle) {
        /*
          calculate how much extra airspeed we need to target to
          achieve the desired ground speed in MIN_GROUNDSPEED

          we quantise the additional airspeed and apply a hysteresis
          in order to avoid triggering an oscillation in TECS
         */
        float target_airspeed = target_airspeed_cm*0.01;
        float EAS_undershoot = (groundspeed_undershoot*0.01) / ahrs.get_EAS2TAS();
        float min_gnd_target_airspeed = airspeed_measured + EAS_undershoot;
        float airspeed_target_offset = min_gnd_target_airspeed > target_airspeed? (min_gnd_target_airspeed - target_airspeed) : 0;

        // round up to nearest m/s
        airspeed_target_offset = int(airspeed_target_offset + 0.5);

        // apply some hysteresis
        if (airspeed_target_offset < last_groundspeed_undershoot_offset &&
            last_groundspeed_undershoot_offset - airspeed_target_offset < 1.2) {
            airspeed_target_offset = last_groundspeed_undershoot_offset;
        }
        last_groundspeed_undershoot_offset = airspeed_target_offset;

        target_airspeed_cm += airspeed_target_offset * 100;
    }

    // when using the special GUIDED mode features for slew control, don't allow airspeed nudging as it doesn't play nicely.
#if AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED
    if (control_mode == &mode_guided && !is_zero(guided_state.target_airspeed_cm) && (airspeed_nudge_cm != 0)) {
        airspeed_nudge_cm = 0; //airspeed_nudge_cm forced to zero
    }
#endif

    // Bump up the target airspeed based on throttle nudging
    if (control_mode->allows_throttle_nudging() && airspeed_nudge_cm > 0) {
        target_airspeed_cm += airspeed_nudge_cm;
    }

    float airspeed_lower_bound = is_positive(aparm.airspeed_stall)
                                     ? aparm.airspeed_stall
                                     : aparm.airspeed_min;

    // Apply airspeed limit
    target_airspeed_cm = constrain_int32(target_airspeed_cm, airspeed_lower_bound*100, aparm.airspeed_max*100);

    // use the TECS view of the target airspeed for reporting, to take
    // account of the landing speed
    airspeed_error = TECS_controller.get_target_airspeed() - airspeed_measured;
}

/**
 * @brief Calculate ground speed undershoot for minimum groundspeed compensation
 * 
 * @details Computes how much groundspeed is below the minimum required groundspeed setting.
 * Uses forward component of ground velocity in the aircraft body frame to prevent
 * flyaway behavior if strong tailwinds push aircraft backwards relative to ground.
 * 
 * Algorithm:
 * 1. Get velocity in NED frame from inertial navigation
 * 2. Extract yaw component from rotation matrix (body to NED transformation)
 * 3. Project NED velocity onto forward direction (body X axis)
 * 4. Calculate undershoot: MIN_GROUNDSPEED - forward_groundspeed
 * 5. Undershoot used by calc_airspeed_errors() to increase target airspeed in headwinds
 * 
 * This prevents:
 * - Excessive drift in strong headwinds
 * - Inability to make progress toward waypoints
 * - Loss of control due to insufficient airspeed over controls
 * 
 * @note Requires valid inertial navigation (EKF) for velocity estimate
 * @note Only active when MIN_GROUNDSPEED parameter > 0
 * @note Groundspeed in cm/s for unit consistency
 * @note Uses NED coordinate frame for velocity calculations
 * 
 * @see calc_airspeed_errors() for application of undershoot compensation
 * @see AP_AHRS::get_velocity_NED() for velocity source
 */
void Plane::calc_gndspeed_undershoot()
{
    /*
     * Calculate forward component of ground speed:
     * Using only the forward component prevents flyaway if strong winds
     * push the aircraft backwards. We want to maintain minimum groundspeed
     * in the direction we're trying to fly, not total groundspeed.
     */
    Vector3f velNED;
    if (ahrs.have_inertial_nav() && ahrs.get_velocity_NED(velNED)) {
        // Extract yaw direction from rotation matrix (first two columns of row 1 and 2)
        const Matrix3f &rotMat = ahrs.get_rotation_body_to_ned();
        Vector2f yawVect = Vector2f(rotMat.a.x,rotMat.b.x);
        if (!yawVect.is_zero()) {
            yawVect.normalize();
            // Project NED velocity onto forward direction (dot product)
            float gndSpdFwd = yawVect * velNED.xy();
            groundspeed_undershoot_is_valid = aparm.min_groundspeed > 0;
            // Calculate undershoot in cm/s (positive value means we're slower than minimum)
            groundspeed_undershoot = groundspeed_undershoot_is_valid ? (aparm.min_groundspeed*100 - gndSpdFwd*100) : 0;
        }
    } else {
        // No valid velocity estimate - disable groundspeed compensation
        groundspeed_undershoot_is_valid = false;
        groundspeed_undershoot = 0;
    }
}

/**
 * @brief Update L1 navigation controller for loiter pattern
 * 
 * @details Internal helper for update_loiter() that commands the L1 controller with
 * appropriate navigation mode based on loiter state and vehicle configuration.
 * 
 * Navigation mode selection:
 * - If far from loiter point (> 3x radius) and using crosstrack: waypoint navigation
 * - If approaching quadplane QRTL transition: waypoint navigation
 * - If in quadplane VTOL loiter: delegate to quadplane guided mode
 * - Otherwise: circular loiter pattern via L1 controller
 * 
 * Waypoint approach is used when far from loiter point to leverage crosstrack
 * correction for smooth approach. Once within 3x radius, switches to circular loiter.
 * 
 * @param[in] radius Loiter radius in meters
 * 
 * @note Called by update_loiter() at navigation update rate
 * @note Integrates with quadplane for VTOL loiter transitions
 * @note Uses L1 controller for both waypoint approach and circular patterns
 * 
 * @see update_loiter() for loiter state management
 * @see AP_L1_Control::update_waypoint() for waypoint navigation
 * @see AP_L1_Control::update_loiter() for circular loiter
 */
void Plane::update_loiter_update_nav(uint16_t radius)
{
#if HAL_QUADPLANE_ENABLED
    /*
     * Quadplane VTOL loiter mode:
     * If loiter timer has started and quadplane guided mode is enabled,
     * transition to VTOL loiter instead of fixed-wing circular pattern.
     * This provides precision hovering at the loiter point.
     */
    if (loiter.start_time_ms != 0 &&
        quadplane.guided_mode_enabled()) {
        if (!auto_state.vtol_loiter) {
            auto_state.vtol_loiter = true;
            // Reset loiter start time, so we don't consider the point reached
            // until VTOL positioning is established (requires tighter tolerance)
            loiter.start_time_ms = 0;
            quadplane.guided_start();
        }
        return;
    }
#endif

#if HAL_QUADPLANE_ENABLED
    // Check if approaching quadplane QRTL transition point
    const bool quadplane_qrtl_switch = (control_mode == &mode_rtl && quadplane.available() && quadplane.rtl_mode == QuadPlane::RTL_MODE::SWITCH_QRTL);
#else
    const bool quadplane_qrtl_switch = false;
#endif

    /*
     * Navigation mode selection for loiter approach:
     * 
     * Use waypoint navigation (straight line with crosstrack correction) if:
     * 1. Haven't reached loiter point yet (loiter.start_time_ms == 0) AND
     * 2. In AUTO or GUIDED mode AND
     * 3. Crosstrack is enabled AND
     * 4. More than 3x loiter radius away from target
     * OR if approaching quadplane QRTL transition
     * 
     * This provides smooth approach to loiter point with crosstrack correction.
     * Once within 3x radius, switch to circular loiter pattern for smooth entry.
     * 
     * The 3x radius threshold prevents oscillation between waypoint and loiter modes.
     */
    if ((loiter.start_time_ms == 0 &&
         (control_mode == &mode_auto || control_mode == &mode_guided) &&
         auto_state.crosstrack &&
         current_loc.get_distance(next_WP_loc) > 3 * nav_controller->loiter_radius(radius)) ||
        quadplane_qrtl_switch) {
        // Use L1 waypoint navigation for approach to loiter point
        nav_controller->update_waypoint(prev_WP_loc, next_WP_loc);
        return;
    }
    /*
     * Command L1 controller for circular loiter pattern:
     * - next_WP_loc: Center of loiter circle
     * - radius: Loiter radius in meters
     * - loiter.direction: +1 for clockwise, -1 for counter-clockwise
     * 
     * L1 controller calculates required bank angle and heading to maintain
     * circular path with wind compensation
     */
    nav_controller->update_loiter(next_WP_loc, radius, loiter.direction);
}

/**
 * @brief Update loiter pattern execution and state management
 * 
 * @details Main loiter update function managing circular loiter patterns. Handles:
 * - Loiter radius and direction configuration
 * - Transition from waypoint approach to circular pattern
 * - Detection of loiter point arrival
 * - Timer initialization when target reached
 * - Integration with quadplane VTOL loiter
 * - Mission item reached notification to ground station
 * 
 * Loiter radius selection priority:
 * 1. If radius parameter <= 1: Use LOITER_RADIUS parameter (or default if <= 1)
 * 2. Otherwise: Use specified radius
 * 
 * Loiter direction:
 * - Follows waypoint loiter_ccw flag if set
 * - Otherwise follows LOITER_RADIUS sign (negative = counter-clockwise)
 * - Stored as +1 (clockwise) or -1 (counter-clockwise)
 * 
 * @param[in] radius Commanded loiter radius in meters (0 or 1 = use parameter)
 * 
 * @note Called at navigation update rate by mode-specific navigation
 * @note Loiter timer starts when reached_loiter_target() returns true
 * @note GCS mission item reached message sent when loiter begins
 * @note Integrates with L1 controller via update_loiter_update_nav()
 * 
 * @see update_loiter_update_nav() for L1 controller command
 * @see reached_loiter_target() for arrival detection
 * @see loiter_angle_update() for angle tracking
 */
void Plane::update_loiter(uint16_t radius)
{
    /*
     * Configure loiter radius and direction:
     * If commanded radius <= 1, use LOITER_RADIUS parameter value.
     * If LOITER_RADIUS <= 1, use default (LOITER_RADIUS_DEFAULT).
     * This allows mission commands to override parameter or use default.
     */
    if (radius <= 1) {
        // Use LOITER_RADIUS parameter (absolute value), or default if too small
        radius = (abs(aparm.loiter_radius) <= 1) ? LOITER_RADIUS_DEFAULT : abs(aparm.loiter_radius);
        
        // Determine loiter direction from waypoint flag or parameter sign
        if (next_WP_loc.loiter_ccw == 1) {
            // Waypoint specifies counter-clockwise
            loiter.direction = -1;
        } else {
            // Use LOITER_RADIUS parameter sign (negative = CCW, positive = CW)
            loiter.direction = (aparm.loiter_radius < 0) ? -1 : 1;
        }
    }

    // Store actual radius being used for access by other functions (e.g., mode checks, failsafe)
    loiter.radius = (float)radius;

    // Update L1 controller with loiter command (waypoint or circular depending on distance)
    update_loiter_update_nav(radius);

    /*
     * Loiter target arrival detection and timer start:
     * Once we've reached the loiter point (or passed it), start the loiter timer.
     * Timer is used for:
     * - Mission commands requiring loiter duration
     * - Mission commands requiring N loiter circles
     * - Mode-specific loiter completion detection
     */
    if (loiter.start_time_ms == 0) {
        // Check if we've reached loiter target or passed through the waypoint line
        if (reached_loiter_target() ||
            auto_state.wp_proportion > 1) {
            // Target reached - start the loiter timer
            loiter.start_time_ms = millis();
            
            // Notify ground station that loiter point has been reached
            if (control_mode->is_guided_mode()) {
                // In GUIDED mode, starting a loiter means we just reached the target point
                gcs().send_mission_item_reached_message(0);
            }
#if HAL_QUADPLANE_ENABLED
            // Initiate quadplane VTOL loiter if enabled
            if (quadplane.guided_mode_enabled()) {
                quadplane.guided_start();
            }
#endif
        }
    }
}

/**
 * @brief Handle speed and height control in FBWB, CRUISE, and optionally LOITER modes
 * 
 * @details Implements pilot-controlled altitude and airspeed management for manual modes
 * with stabilization. Elevator input controls climb/descent rate, throttle controls airspeed.
 * 
 * Functionality:
 * - Elevator stick position sets climb rate (FLYBYWIRE_CLIMB_RATE parameter)
 * - Climb rate integrated to calculate target altitude
 * - Elevator at center position locks current altitude
 * - TECS manages energy distribution for altitude/airspeed control
 * - Soaring mode integration for thermal exploitation (optional)
 * 
 * Control behavior:
 * - Elevator up: Climb at configured rate
 * - Elevator center: Hold current altitude
 * - Elevator down: Descend at configured rate
 * - Rate limited by TECS max climb/sink rates
 * 
 * @note Called at main loop rate from FBWB, CRUISE, and optionally LOITER modes
 * @note Climb rate in m/s, altitude in centimeters AMSL
 * @note Elevator input can be reversed with FLYBYWIRE_ELEV_REVERSE parameter
 * @note Update rate throttled to 10Hz (100ms) to prevent excessive altitude jitter
 * 
 * @see calc_throttle() for throttle/airspeed control
 * @see calc_nav_pitch() for pitch command calculation
 * @see AP_TECS for energy management
 */
void Plane::update_fbwb_speed_height(void)
{
    uint32_t now = micros();
    // Throttle update rate to 10Hz (100ms intervals)
    if (now - target_altitude.last_elev_check_us >= 100000) {
        /*
         * We don't run this on every loop because:
         * - At 400Hz on quadplanes, each update would be < 1cm altitude change
         * - Sub-centimeter changes would be quantized away (no climb or descent)
         * - 10Hz provides smooth altitude control with adequate resolution
         */
        float dt = (now - target_altitude.last_elev_check_us) * 1.0e-6;
        dt = constrain_float(dt, 0.1, 0.15);

        target_altitude.last_elev_check_us = now;

        // Scale elevator input from control_in range (-4500 to +4500) to normalized [-1, +1]
        float elevator_input = channel_pitch->get_control_in() * (1/4500.0);

        // Apply elevator reverse if configured (some aircraft have reversed elevator)
        if (g.flybywire_elev_reverse) {
            elevator_input = -elevator_input;
        }

        /*
         * Detect elevator stick returning to center (zero crossing):
         * When pilot centers the stick, lock in current altitude instead of
         * continuing to integrate altitude changes. This provides intuitive
         * "altitude hold at current altitude" behavior.
         */
        bool input_stop_climb = !is_positive(elevator_input) && is_positive(target_altitude.last_elevator_input);
        bool input_stop_descent = !is_negative(elevator_input) && is_negative(target_altitude.last_elevator_input);
        if (input_stop_climb || input_stop_descent) {
            // Elevator input crossed zero - lock current altitude as target
            set_target_altitude_current();
        }

        // Calculate desired climb rate from elevator input
        // Positive elevator = climb, negative = descend
        float climb_rate = g.flybywire_climb_rate * elevator_input;
        // Constrain to TECS performance limits (prevents stall in climb or overspeed in descent)
        climb_rate = constrain_float(climb_rate, -TECS_controller.get_max_sinkrate(), TECS_controller.get_max_climbrate());

        // Integrate climb rate to update target altitude (m/s * s * 100 = cm)
        int32_t alt_change_cm = climb_rate * dt * 100;
        change_target_altitude(alt_change_cm);

#if HAL_SOARING_ENABLED
        /*
         * Soaring mode altitude management:
         * When soaring is active, override normal altitude control:
         * - Throttle suppressed: Exploiting thermal, hold current altitude
         * - Throttle active: Climbing back to soaring altitude threshold
         *   Set target to SOAR_ALT_CUTOFF + 10m to ensure positive climb
         *   through threshold, which will trigger throttle suppression
         */
        if (g2.soaring_controller.is_active()) {
            if (g2.soaring_controller.get_throttle_suppressed()) {
                // In thermal with throttle off - maintain current altitude
                set_target_altitude_current();
            } else {
                // Motoring back to altitude - set target above cutoff to ensure
                // we positively cross threshold and re-enter gliding mode
                target_altitude.amsl_cm = 100*plane.g2.soaring_controller.get_alt_cutoff() + 1000 + AP::ahrs().get_home().alt;
            }
        }
#endif

        // Store elevator input for next update (used for zero-crossing detection)
        target_altitude.last_elevator_input = elevator_input;
    }

    // Enforce FBWB altitude limits (prevents excessive altitude in this mode)
    check_fbwb_altitude();

    // Calculate throttle command from TECS (energy management)
    calc_throttle();
    
    // Calculate pitch command from TECS (altitude/airspeed control)
    calc_nav_pitch();
}

/**
 * @brief Calculate the turn angle for the next mission leg
 * 
 * @details Computes the heading change required between current waypoint leg and
 * next waypoint leg for mission planning and turn anticipation. Used by L1 controller
 * to begin turns before reaching waypoint for smooth path following.
 * 
 * Algorithm:
 * 1. Query mission library for ground course of next leg
 * 2. Calculate ground course of current leg
 * 3. Compute wrapped angle difference [-180, +180] degrees
 * 4. Store in auto_state.next_turn_angle for turn anticipation
 * 
 * Turn anticipation enables:
 * - Earlier turn initiation for sharp corners
 * - Smoother path following through waypoints
 * - Reduced overshoot on tight turns
 * - Better energy management during turns
 * 
 * @note Called when starting new mission leg
 * @note If mission library can't determine next course, assumes 90-degree turn
 * @note Turn angle in degrees, positive = right turn, negative = left turn
 * @note Used by L1 controller for waypoint turn radius calculation
 * 
 * @see AP_Mission::get_next_ground_course_cd() for next leg course
 * @see AP_L1_Control for turn anticipation implementation
 */
void Plane::setup_turn_angle(void)
{
    // Query mission library for ground course of the leg after the current one
    int32_t next_ground_course_cd = mission.get_next_ground_course_cd(-1);
    if (next_ground_course_cd == -1) {
        /*
         * Mission library can't determine turn angle (e.g., at end of mission,
         * or next command is not a navigation command). Assume 90-degree turn
         * as a safe default for turn anticipation calculations.
         */
        auto_state.next_turn_angle = 90.0f;
    } else {
        // Calculate ground course of current leg (bearing from previous to next waypoint)
        int32_t ground_course_cd = prev_WP_loc.get_bearing_to(next_WP_loc);

        /*
         * Calculate turn angle: difference between next leg and current leg headings
         * wrap_180_cd ensures result is in range [-180, +180] degrees
         * Positive angle = right turn, negative angle = left turn
         * Convert from centidegrees to degrees
         */
        auto_state.next_turn_angle = wrap_180_cd(next_ground_course_cd - ground_course_cd) * 0.01f;
    }
}

/**
 * @brief Check if aircraft has reached the loiter target point
 * 
 * @details Determines whether the aircraft is close enough to the loiter center
 * to be considered "arrived". Uses different thresholds for fixed-wing vs VTOL:
 * - Fixed-wing: Uses L1 controller's loiter target reached logic (radius-based)
 * - VTOL (quadplane): Uses tight tolerance (3 meters) for precision positioning
 * 
 * The L1 controller considers loiter target reached when:
 * - Within configured loiter radius of center point
 * - Tracking error is acceptably small
 * - Aircraft is established on the circular path
 * 
 * @return true if loiter target reached, false if still approaching
 * 
 * @note Called by loiter update functions at navigation rate
 * @note Used to start loiter timer and angle tracking
 * @note VTOL uses tighter tolerance for hovering accuracy
 * @note Fixed-wing tolerance accounts for turn radius and wind
 * 
 * @see AP_L1_Control::reached_loiter_target() for fixed-wing logic
 * @see update_loiter() for loiter timer management
 * @see loiter_angle_update() for angle tracking initialization
 */
bool Plane::reached_loiter_target(void)
{
#if HAL_QUADPLANE_ENABLED
    /*
     * Quadplane VTOL mode uses precision positioning:
     * Consider loiter reached when within 3 meters of target.
     * Tighter tolerance than fixed-wing because VTOL can hover precisely.
     * Distance in meters from current position to next_WP_loc.
     */
    if (quadplane.in_vtol_auto()) {
        return auto_state.wp_distance < 3;
    }
#endif
    /*
     * Fixed-wing loiter target reached check:
     * Delegate to L1 controller which considers:
     * - Distance to loiter center
     * - Loiter radius
     * - Crosstrack error
     * - Path following accuracy
     * 
     * L1 controller uses more sophisticated logic accounting for
     * turn radius, airspeed, and wind conditions.
     */
    return nav_controller->reached_loiter_target();
}
