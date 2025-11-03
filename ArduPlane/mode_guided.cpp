/**
 * @file mode_guided.cpp
 * @brief GUIDED flight mode implementation for ArduPlane
 * 
 * @details GUIDED mode enables external control of the aircraft via Ground Control Station (GCS)
 *          or companion computer through MAVLink commands. This mode supports multiple control
 *          methods including:
 *          - Position control: Navigate to waypoint locations
 *          - Heading control: Fly towards a specified heading or course
 *          - Attitude override: Direct roll/pitch/yaw control from external commands
 *          - Velocity control: Airspeed and altitude rate control
 *          - Throttle passthrough: Manual throttle control (fence breach recovery)
 * 
 *          The mode integrates with QuadPlane for VTOL operations and supports both
 *          fixed-wing and VTOL loiter behaviors.
 * 
 *          Control inputs are accepted via MAVLink SET_POSITION_TARGET_GLOBAL_INT,
 *          SET_POSITION_TARGET_LOCAL_NED, SET_ATTITUDE_TARGET, and related commands.
 * 
 *          Safety considerations:
 *          - External control commands timeout after 3 seconds
 *          - Integrates with geofencing and failsafe systems
 *          - Supports terrain following when configured
 *          - Roll and pitch angles constrained to configured limits
 * 
 * @note This mode requires active MAVLink communication for continuous control
 * @warning Loss of communication will cause timeout to standard navigation after 3 seconds
 * 
 * Source: ArduPlane/mode_guided.cpp
 */

#include "mode.h"
#include "Plane.h"

/**
 * @brief Enter GUIDED flight mode and initialize guided state
 * 
 * @details Initializes the GUIDED mode by:
 *          1. Disabling throttle passthrough mode
 *          2. Setting the initial guided waypoint to the current location
 *          3. For QuadPlane: Projects target forward by stopping distance to smooth entry
 *          4. Resets the active loiter radius to default (WP_LOITER_RAD)
 * 
 *          This initialization matches the behavior of ArduCopter's GUIDED mode,
 *          providing consistent behavior across vehicle types.
 * 
 * @return true Always returns true (mode entry always succeeds)
 * 
 * @note Called automatically by the mode change logic when switching to GUIDED mode
 * @note QuadPlane stopping distance projection helps prevent aggressive braking on entry
 * 
 * Source: ArduPlane/mode_guided.cpp:4-28
 */
bool ModeGuided::_enter()
{
    plane.guided_throttle_passthru = false;
    /*
      when entering guided mode we set the target as the current
      location. This matches the behaviour of the copter code
    */
    Location loc{plane.current_loc};

#if HAL_QUADPLANE_ENABLED
    if (plane.quadplane.guided_mode_enabled()) {
        /*
          if using Q_GUIDED_MODE then project forward by the stopping distance
        */
        loc.offset_bearing(degrees(ahrs.groundspeed_vector().angle()),
                           plane.quadplane.stopping_distance());
    }
#endif

    // set guided radius to WP_LOITER_RAD on mode change.
    active_radius_m = 0;

    plane.set_guided_WP(loc);
    return true;
}

/**
 * @brief Update roll, pitch, and throttle control for GUIDED mode
 * 
 * @details This is the main control update function called at the scheduler rate (typically 50Hz).
 *          It handles multiple guided control sub-modes and external command inputs:
 * 
 *          **QuadPlane VTOL Loiter Mode:**
 *          - Delegates to quadplane.guided_update() for multirotor control
 * 
 *          **Roll Control Priority Order:**
 *          1. Forced RPY commands (if received within last 3 seconds)
 *             - Direct roll angle control from MAVLink SET_ATTITUDE_TARGET
 *             - Constrained to configured roll limits
 *          2. Heading/Course control (if AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED)
 *             - GUIDED_HEADING_HEADING: Fly towards absolute heading
 *             - GUIDED_HEADING_COURSE: Fly towards ground track direction
 *             - Uses AC_PID controller for smooth heading changes
 *             - Bank angle limited by lateral acceleration limit and roll limits
 *          3. Standard navigation: calc_nav_roll() for waypoint tracking
 * 
 *          **Pitch Control Priority Order:**
 *          1. Forced RPY commands (if received within last 3 seconds)
 *             - Direct pitch angle control from MAVLink
 *             - Constrained between pitch_limit_min and pitch_limit_max
 *          2. Standard navigation: calc_nav_pitch() for altitude/airspeed control
 * 
 *          **Throttle Control Priority Order:**
 *          1. Manual passthrough (guided_throttle_passthru = true)
 *             - Used for fence breach recovery or manual override
 *          2. Forced throttle commands (if received within last 3 seconds)
 *             - Direct throttle output from MAVLink
 *          3. TECS control: calc_throttle() for energy management
 * 
 * @note Called every scheduler loop iteration while in GUIDED mode (typically 50Hz)
 * @note External command timeout of 3 seconds provides safety fallback
 * @note Heading control requires AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED compile option
 * 
 * @warning All forced control inputs (roll/pitch/throttle) time out after 3 seconds
 *          If communication is lost, aircraft reverts to standard navigation
 * @warning Bank angle in heading mode is limited by both roll limits and lateral G forces
 * 
 * @see plane.guided_state for external command state tracking
 * @see plane.calc_nav_roll(), plane.calc_nav_pitch(), plane.calc_throttle() for standard navigation
 * @see Plane::handle_set_attitude_target() for forced RPY command processing
 * 
 * Source: ArduPlane/mode_guided.cpp:30-102
 */
void ModeGuided::update()
{
#if HAL_QUADPLANE_ENABLED
    if (plane.auto_state.vtol_loiter && plane.quadplane.available()) {
        plane.quadplane.guided_update();
        return;
    }
#endif

    // Received an external msg that guides roll in the last 3 seconds?
    if (plane.guided_state.last_forced_rpy_ms.x > 0 &&
            millis() - plane.guided_state.last_forced_rpy_ms.x < 3000) {
        plane.nav_roll_cd = constrain_int32(plane.guided_state.forced_rpy_cd.x, -plane.roll_limit_cd, plane.roll_limit_cd);
        plane.update_load_factor();

#if AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED
    // guided_state.target_heading is radians at this point between -pi and pi ( defaults to -4 )
    // This function is used in Guided and AvoidADSB, check for guided
    } else if ((plane.control_mode == &plane.mode_guided) && (plane.guided_state.target_heading_type != GUIDED_HEADING_NONE) ) {
        uint32_t tnow = AP_HAL::millis();
        float delta = (tnow - plane.guided_state.target_heading_time_ms) * 1e-3f;
        plane.guided_state.target_heading_time_ms = tnow;

        float error = 0.0f;
        if (plane.guided_state.target_heading_type == GUIDED_HEADING_HEADING) {
            error = wrap_PI(plane.guided_state.target_heading - AP::ahrs().get_yaw_rad());
        } else {
            Vector2f groundspeed = AP::ahrs().groundspeed_vector();
            error = wrap_PI(plane.guided_state.target_heading - atan2f(-groundspeed.y, -groundspeed.x) + M_PI);
        }

        float bank_limit = degrees(atanf(plane.guided_state.target_heading_accel_limit/GRAVITY_MSS)) * 1e2f;
        bank_limit = MIN(bank_limit, plane.roll_limit_cd);

        // push error into AC_PID
        const float desired = plane.g2.guidedHeading.update_error(error, delta, plane.guided_state.target_heading_limit);

        // Check for output saturation
        plane.guided_state.target_heading_limit = fabsf(desired) >= bank_limit;

        plane.nav_roll_cd = constrain_int32(desired, -bank_limit, bank_limit);
        plane.update_load_factor();

#endif // AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED
    } else {
        plane.calc_nav_roll();
    }

    if (plane.guided_state.last_forced_rpy_ms.y > 0 &&
            millis() - plane.guided_state.last_forced_rpy_ms.y < 3000) {
        plane.nav_pitch_cd = constrain_int32(plane.guided_state.forced_rpy_cd.y, plane.pitch_limit_min*100, plane.aparm.pitch_limit_max.get()*100);
    } else {
        plane.calc_nav_pitch();
    }

    // Throttle output
    if (plane.guided_throttle_passthru) {
        // manual passthrough of throttle in fence breach
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, plane.get_throttle_input(true));

    }  else if (plane.aparm.throttle_cruise > 1 &&
            plane.guided_state.last_forced_throttle_ms > 0 &&
            millis() - plane.guided_state.last_forced_throttle_ms < 3000) {
        // Received an external msg that guides throttle in the last 3 seconds?
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, plane.guided_state.forced_throttle);

    } else {
        // TECS control
        plane.calc_throttle();

    }

}

/**
 * @brief Calculate navigation parameters for waypoint tracking in GUIDED mode
 * 
 * @details Updates the L1 navigation controller to track towards the current guided waypoint
 *          using a loiter pattern. The loiter radius and direction are configured via
 *          MAVLink MISSION_ITEM or DO_REPOSITION commands.
 * 
 *          This function is called by the navigation update loop to compute the desired
 *          ground track and lateral acceleration for waypoint approach and loiter.
 * 
 * @note Called at navigation update rate (typically 10Hz) from main flight loop
 * @note Loiter radius can be set via set_radius_and_direction() or defaults to WP_LOITER_RAD
 * 
 * @see set_radius_and_direction() for configuring loiter parameters
 * @see plane.update_loiter() for L1 controller waypoint tracking
 * 
 * Source: ArduPlane/mode_guided.cpp:104-107
 */
void ModeGuided::navigate()
{
    plane.update_loiter(active_radius_m);
}

/**
 * @brief Process MAVLink command to navigate to a new guided waypoint
 * 
 * @details Handles external requests to change the guided waypoint target location.
 *          This function is called when receiving MAVLink commands such as:
 *          - MAV_CMD_NAV_WAYPOINT in GUIDED mode
 *          - MAV_CMD_DO_REPOSITION
 *          - SET_POSITION_TARGET_GLOBAL_INT with position mask
 * 
 *          Processing steps:
 *          1. Applies terrain altitude corrections if terrain following is enabled
 *          2. Converts relative altitudes to absolute frame if needed
 *          3. Updates the active guided waypoint via plane.set_guided_WP()
 * 
 * @param[in] target_loc Target location to navigate towards (lat, lon, alt)
 *                       May be in various altitude frames (ABSOLUTE, RELATIVE, TERRAIN)
 * 
 * @return true Always returns true (command acceptance always succeeds)
 * 
 * @note Altitude frame is normalized to ABSOLUTE if not terrain-relative
 * @note Terrain altitude lookup may adjust target altitude for terrain following
 * 
 * @see plane.set_guided_WP() for waypoint update and L1 controller initialization
 * @see plane.fix_terrain_WP() for terrain altitude correction
 * 
 * Source: ArduPlane/mode_guided.cpp:109-120
 */
bool ModeGuided::handle_guided_request(Location target_loc)
{
    plane.fix_terrain_WP(target_loc, __LINE__);
    // add home alt if needed
    if (!target_loc.terrain_alt) {
        target_loc.change_alt_frame(Location::AltFrame::ABSOLUTE);
    }

    plane.set_guided_WP(target_loc);

    return true;
}

#if AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED
/**
 * @brief Process external command to change target airspeed with rate limiting
 * 
 * @details Handles MAVLink commands to change the aircraft's target airspeed with
 *          controlled acceleration limits. This provides smooth airspeed transitions
 *          for external control applications.
 * 
 *          Command validation:
 *          - Rejects airspeeds outside the configured envelope (ARSPD_FBW_MIN to ARSPD_FBW_MAX)
 *          - Ignores duplicate commands with same airspeed target
 * 
 *          Acceleration handling:
 *          - acceleration = 0: Maximum acceleration (treated as 1000 m/s²)
 *          - acceleration > 0: Uses specified value for smooth transitions
 *          - Acceleration direction automatically determined by target vs current airspeed
 * 
 *          The acceleration limit is applied incrementally in update_target_altitude()
 *          to slew the airspeed smoothly over time.
 * 
 * @param[in] airspeed Target airspeed in m/s
 * @param[in] acceleration Maximum acceleration/deceleration in m/s²
 *                         Zero value requests maximum acceleration
 * 
 * @return true if airspeed change accepted
 * @return false if airspeed is outside valid range (below ARSPD_FBW_MIN or above ARSPD_FBW_MAX)
 * 
 * @note Only available when AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED is compiled in
 * @note Stored in plane.guided_state.target_airspeed_cm (centimeters/sec)
 * @note Acceleration is stored as signed value (negative for deceleration)
 * 
 * @warning Excessive acceleration values may cause altitude loss during airspeed changes
 * @warning Airspeed limits enforced for flight envelope protection
 * 
 * @see update_target_altitude() for acceleration slew rate application
 * 
 * Source: ArduPlane/mode_guided.cpp:123-152
 */
bool ModeGuided::handle_change_airspeed(const float airspeed, const float acceleration)
{
    // reject airspeeds that are outside of the tuning envelope
    if (airspeed > plane.aparm.airspeed_max || airspeed < plane.aparm.airspeed_min) {
        return false;
    }

    // no need to process any new packet/s with the
    // same airspeed any further, if we are already doing it.
    float new_target_airspeed_cm = airspeed * 100;
    if (is_equal(new_target_airspeed_cm,plane.guided_state.target_airspeed_cm)) { 
        return true;
    }
    plane.guided_state.target_airspeed_cm = new_target_airspeed_cm;
    plane.guided_state.target_airspeed_time_ms = AP_HAL::millis();

    if (is_zero(acceleration)) {
        // the user wanted /maximum acceleration, pick a large value as close enough
        plane.guided_state.target_airspeed_accel = 1000.0f;
    } else {
        plane.guided_state.target_airspeed_accel = fabsf(acceleration);
    }

    // assign an acceleration direction
    if (plane.guided_state.target_airspeed_cm < plane.target_airspeed_cm) {
        plane.guided_state.target_airspeed_accel *= -1.0f;
    }
    return true; 
}
#endif // AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED

/**
 * @brief Configure loiter radius and direction for waypoint approach
 * 
 * @details Sets the loiter pattern parameters used when the aircraft reaches the guided
 *          waypoint target. This is typically called when processing MAVLink commands that
 *          specify loiter behavior (e.g., MAV_CMD_DO_REPOSITION with loiter radius).
 * 
 *          The radius is constrained to fit in uint16_t range (0-65535 meters) as required
 *          by the update_loiter() function. Direction determines clockwise (CW) or
 *          counter-clockwise (CCW) loiter pattern.
 * 
 * @param[in] radius Loiter radius in meters (absolute value used)
 *                   Constrained to range [0, 65535]
 * @param[in] direction_is_ccw Loiter direction
 *                              - true: Counter-clockwise (left turns, direction = -1)
 *                              - false: Clockwise (right turns, direction = 1)
 * 
 * @note Radius of 0 uses the WP_LOITER_RAD parameter default
 * @note Direction follows standard aviation convention (left = CCW, right = CW)
 * @note Active radius stored in active_radius_m member variable
 * 
 * @see plane.update_loiter() for L1 controller loiter implementation
 * @see navigate() where active_radius_m is used
 * 
 * Source: ArduPlane/mode_guided.cpp:154-159
 */
void ModeGuided::set_radius_and_direction(const float radius, const bool direction_is_ccw)
{
    // constrain to (uint16_t) range for update_loiter()
    active_radius_m = constrain_int32(fabsf(radius), 0, UINT16_MAX);
    plane.loiter.direction = direction_is_ccw ? -1 : 1;
}

/**
 * @brief Update target altitude with optional rate limiting for smooth altitude changes
 * 
 * @details Manages altitude target updates with two modes of operation:
 * 
 *          **Offboard Altitude Control Mode** (AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED):
 *          When external altitude commands are active (guided_state.target_alt_time_ms != 0),
 *          implements rate-limited altitude changes:
 *          - Calculates incremental altitude change based on target_alt_rate (m/s)
 *          - Clamps change to not exceed specified rate over time delta
 *          - Supports multiple altitude frames (ABSOLUTE, RELATIVE, TERRAIN)
 *          - Creates intermediate target that steps towards final altitude
 *          - Prevents sudden altitude changes that could stress airframe
 * 
 *          Algorithm:
 *          1. Calculate time delta since last update
 *          2. Compute allowed altitude change = delta_time * target_alt_rate
 *          3. Constrain target altitude to current ± allowed change
 *          4. Update TECS with intermediate altitude target
 * 
 *          **Standard Mode** (offboard control not active):
 *          Delegates to base Mode::update_target_altitude() for normal waypoint
 *          altitude tracking without rate limiting.
 * 
 * @note Called every control loop iteration (typically 50Hz)
 * @note Rate limiting active only when external altitude commands are being processed
 * @note Altitude frame conversions handled automatically for terrain-relative targets
 * @note Rate limit defaults to -1 (disabled) until set by MAVLink command
 * 
 * @warning Altitude rate limiting requires valid current location and target location
 * @warning Frame conversion failures fall back to previous target altitude
 * 
 * @see plane.guided_state.target_alt_rate for altitude change rate in m/s
 * @see plane.guided_state.target_location for final altitude target
 * @see plane.set_target_altitude_location() for TECS altitude update
 * 
 * Source: ArduPlane/mode_guided.cpp:161-192
 */
void ModeGuided::update_target_altitude()
{
#if AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED
    // target altitude can be negative (e.g. flying below home altitude from the top of a mountain)
    if (((plane.guided_state.target_alt_time_ms != 0) || plane.guided_state.target_location.alt != -1 )) { // target_alt now defaults to -1, and _time_ms defaults to zero.
        // offboard altitude demanded
        uint32_t now = AP_HAL::millis();
        float delta = 1e-3f * (now - plane.guided_state.target_alt_time_ms);
        plane.guided_state.target_alt_time_ms = now;
        // determine delta accurately as a float
        float delta_amt_f = delta * plane.guided_state.target_alt_rate;
        // then scale x100 to match last_target_alt and convert to a signed int32_t as it may be negative
        int32_t delta_amt_i = (int32_t)(100.0 * delta_amt_f); 
        // To calculate the required velocity (up or down), we need to target and current altitudes in the target frame
        const Location::AltFrame target_frame = plane.guided_state.target_location.get_alt_frame();
        int32_t target_alt_previous_cm;
        if (plane.current_loc.initialised() && plane.guided_state.target_location.initialised() && 
            plane.current_loc.get_alt_cm(target_frame, target_alt_previous_cm)) {
            // create a new interim target location that that takes current_location and moves delta_amt_i in the right direction
            int32_t temp_alt_cm = constrain_int32(plane.guided_state.target_location.alt, target_alt_previous_cm - delta_amt_i,  target_alt_previous_cm + delta_amt_i);
            Location temp_location = plane.guided_state.target_location;
            temp_location.set_alt_cm(temp_alt_cm, target_frame);

            // incrementally step the altitude towards the target            
            plane.set_target_altitude_location(temp_location);
        }
    } else 
#endif // AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED
        {
        Mode::update_target_altitude();
    }
}
