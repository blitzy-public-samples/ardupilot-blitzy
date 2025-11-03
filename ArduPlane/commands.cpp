/**
 * @file commands.cpp
 * @brief Waypoint and home location management for ArduPlane
 * 
 * @details This file implements core waypoint navigation and home location management
 *          functionality for fixed-wing aircraft. Key responsibilities include:
 *          - Waypoint sequencing and cross-track path planning
 *          - Guided mode waypoint setup
 *          - Home location initialization and updates
 *          - Terrain altitude adjustment for waypoints
 *          - Loiter angle and altitude slope configuration
 * 
 *          The waypoint management system handles prev_WP_loc and next_WP_loc updates,
 *          ensuring smooth navigation between mission waypoints and proper handling of
 *          special cases like zero lat/lon (loiter-on-spot) and terrain following.
 * 
 * @note This file works in conjunction with commands_logic.cpp which handles
 *       mission command execution (start_command/verify_command)
 * 
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#include "Plane.h"

/**
 * @brief Set the next waypoint target location for autonomous navigation
 * 
 * @param[in] loc Target waypoint location in lat/lon/alt format
 * 
 * @details This function configures the next navigation target waypoint, updating
 *          prev_WP_loc and next_WP_loc for path planning. The function handles:
 *          
 *          **Cross-Track Navigation**:
 *          - If auto_state.next_wp_crosstrack is true, copies current next_WP_loc
 *            to prev_WP_loc and enables cross-track error correction
 *          - If false, uses current vehicle position as prev_WP_loc and disables
 *            cross-track for this leg (useful for sharp turns or return-to-path)
 *          
 *          **Special Location Handling**:
 *          - Zero lat/lon (0,0): Treated as "loiter on current spot" - replaces
 *            with current vehicle position
 *          - Zero altitude: Replaced with current altitude in ABSOLUTE frame
 *          - Terrain altitude: Adjusted via fix_terrain_WP() for terrain following
 *          - Relative altitude: Converted to ABSOLUTE frame for navigation
 *          
 *          **Path Skip Prevention**:
 *          - Checks if vehicle is already past the waypoint using
 *            past_interval_finish_line() to detect waypoint jumps
 *          - If already past, uses current location as prev_WP to avoid immediately
 *            considering waypoint complete
 *          
 *          **Navigation Setup**:
 *          - Resets loiter angle tracking for missed waypoint detection
 *          - Calculates altitude slope for glide path planning (setup_alt_slope)
 *          - Calculates turn angle for efficient waypoint approach (setup_turn_angle)
 *          - Updates target altitude immediately to avoid altitude calculation race
 *            conditions with scheduler ordering
 * 
 * @note Called during mission waypoint transitions, guided mode updates, and RTL
 * @note This function modifies global state: prev_WP_loc, next_WP_loc,
 *       auto_state.crosstrack, auto_state.next_wp_crosstrack
 * 
 * @warning Altitude frame conversions assume proper AHRS home location is set
 * @warning Terrain altitude adjustments require valid terrain database
 * 
 * @see set_guided_WP() for guided mode waypoint setup without cross-track
 * @see setup_alt_slope() for glide path altitude planning
 * @see fix_terrain_WP() for terrain altitude adjustment
 * 
 * Source: ArduPlane/commands.cpp:10-69
 */
void Plane::set_next_WP(const Location &loc)
{
    if (auto_state.next_wp_crosstrack) {
        // copy the current WP into the OldWP slot
        prev_WP_loc = next_WP_loc;
        auto_state.crosstrack = true;
    } else {
        // we should not try to cross-track for this waypoint
        prev_WP_loc = current_loc;
        // use cross-track for the next waypoint
        auto_state.next_wp_crosstrack = true;
        auto_state.crosstrack = false;
    }

    // Load the next_WP slot
    // ---------------------
    next_WP_loc = loc;

    // if lat and lon is zero, then use current lat/lon
    // this allows a mission to contain a "loiter on the spot"
    // command
    if (next_WP_loc.lat == 0 && next_WP_loc.lng == 0) {
        next_WP_loc.lat = current_loc.lat;
        next_WP_loc.lng = current_loc.lng;
        // additionally treat zero altitude as current altitude
        if (next_WP_loc.alt == 0) {
            next_WP_loc.set_alt_cm(current_loc.alt, Location::AltFrame::ABSOLUTE);
        }
    }

    fix_terrain_WP(next_WP_loc, __LINE__);

    // convert relative alt to absolute alt
    if (!next_WP_loc.terrain_alt) {
        next_WP_loc.change_alt_frame(Location::AltFrame::ABSOLUTE);
    }

    // are we already past the waypoint? This happens when we jump
    // waypoints, and it can cause us to skip a waypoint. If we are
    // past the waypoint when we start on a leg, then use the current
    // location as the previous waypoint, to prevent immediately
    // considering the waypoint complete
    if (current_loc.past_interval_finish_line(prev_WP_loc, next_WP_loc)) {
        prev_WP_loc = current_loc;
    }

    // zero out our loiter vals to watch for missed waypoints
    loiter_angle_reset();

    setup_alt_slope();
    setup_turn_angle();

    // update plane.target_altitude straight away, or if we are too
    // close to out loiter point we may decide we are at the correct
    // altitude before updating it (this is based on scheduler table
    // ordering, where we navigate() before we
    // adjust_altitude_target(), and navigate() uses values updated in
    // adjust_altitude_target()
    adjust_altitude_target();
}

/**
 * @brief Set waypoint target for guided mode navigation
 * 
 * @param[in] loc Target location for guided mode (lat/lon/alt with optional loiter_ccw flag)
 * 
 * @details Configures waypoint navigation for GUIDED flight mode, where the vehicle
 *          flies directly to a ground station commanded position without mission planning.
 *          This function differs from set_next_WP() by disabling cross-track navigation
 *          and using simplified path planning for responsive manual control.
 * 
 *          **Loiter Direction Configuration**:
 *          - Checks aparm.loiter_radius sign: negative radius = counter-clockwise
 *          - Checks loc.loiter_ccw flag: if set, overrides to counter-clockwise
 *          - Sets loiter.direction to -1 (CCW) or 1 (CW) for arrival loiter pattern
 * 
 *          **Waypoint Initialization**:
 *          - Sets prev_WP_loc to current vehicle position (no path history)
 *          - Sets next_WP_loc to commanded target location
 *          - Applies terrain altitude adjustment via fix_terrain_WP()
 *          - Sets target altitude to current altitude (set_target_altitude_current)
 *          
 *          **Path Planning Setup**:
 *          - Disables cross-track navigation (auto_state.crosstrack = false) for
 *            direct point-to-point flight responsive to GCS updates
 *          - Calculates altitude slope for smooth altitude transitions
 *          - Calculates turn angle for efficient approach geometry
 *          - Resets loiter start time for fresh loiter behavior
 *          
 *          **Mode-Specific Configuration**:
 *          - Disables VTOL loiter mode (auto_state.vtol_loiter = false) to start
 *            in normal fixed-wing flight
 *          - Resets loiter angle tracking for clean loiter entry
 *          - Cancels any pending quadplane guided takeoff
 * 
 * @note Called when entering GUIDED mode or receiving new guided target from GCS
 * @note Cross-track disabled for immediate response to GCS commands
 * @note Quadplane-specific behavior conditionally compiled with HAL_QUADPLANE_ENABLED
 * 
 * @warning Does not validate if location is reachable or safe - assumes GCS validation
 * @warning Terrain altitude adjustment requires valid terrain database
 * 
 * @see set_next_WP() for mission waypoint sequencing with cross-track
 * @see ModeGuided::update() for guided mode navigation execution
 * @see fix_terrain_WP() for terrain altitude handling
 * 
 * Source: ArduPlane/commands.cpp:71-111
 */
void Plane::set_guided_WP(const Location &loc)
{
    if (aparm.loiter_radius < 0 || loc.loiter_ccw) {
        loiter.direction = -1;
    } else {
        loiter.direction = 1;
    }

    // copy the current location into the OldWP slot
    // ---------------------------------------
    prev_WP_loc = current_loc;

    // Load the next_WP slot
    // ---------------------
    next_WP_loc = loc;

    fix_terrain_WP(next_WP_loc, __LINE__);

    // used to control FBW and limit the rate of climb
    // -----------------------------------------------
    set_target_altitude_current();

    setup_alt_slope();
    setup_turn_angle();

    // disable crosstrack, head directly to the point
    auto_state.crosstrack = false;

    // reset loiter start time.
    loiter.start_time_ms = 0;

    // start in non-VTOL mode
    auto_state.vtol_loiter = false;
    
    loiter_angle_reset();

#if HAL_QUADPLANE_ENABLED
    // cancel pending takeoff
    quadplane.guided_takeoff = false;
#endif
}

/**
 * @brief Update home location from GPS while disarmed
 * 
 * @return true if home location was changed, false if home unchanged or update prevented
 * 
 * @details Automatically updates the vehicle home location using current GPS position
 *          while disarmed and GPS has 3D fix. This allows home to track vehicle position
 *          during pre-flight setup, ensuring accurate return-to-home behavior.
 * 
 *          **Update Prevention Conditions**:
 *          - Watchdog was armed: After watchdog reset, prevents position jump
 *          - Home reset threshold disabled (g2.home_reset_threshold == -1): Manual mode
 *          - Altitude change exceeds threshold: Prevents updates after significant
 *            altitude change which indicates flight or major baro drift
 *          
 *          **Home Update Logic**:
 *          - Requires: ahrs.home_is_set() && !ahrs.home_is_locked()
 *          - Requires: GPS status >= GPS_OK_FIX_3D (3D fix with sufficient satellites)
 *          - Uses GPS altitude directly (not AHRS altitude) to avoid circular dependency
 *            where AHRS altitude depends on home altitude
 *          - Sets altitude in ABSOLUTE frame from GPS
 *          
 *          **Barometer Calibration**:
 *          - Always performs barometer update calibration when called (even if home
 *            not updated) to prevent baro drift accumulation while disarmed
 *          - Resets AHRS height datum for consistent altitude reference
 *          - Updates current_loc to reflect new altitude reference
 * 
 *          **Altitude Threshold Logic**:
 *          - Checks fabsf(barometer.get_altitude()) > g2.home_reset_threshold
 *          - Prevents home reset if barometer altitude has changed significantly
 *          - Allows slow baro drift correction but prevents updates after takeoff
 *          - Threshold typically set to a few meters to distinguish drift from flight
 * 
 * @note Called continuously while disarmed with 3D GPS fix
 * @note Home location is critical for RTL, geofencing, and altitude reference
 * @note Baro calibration performed every call to prevent disarmed drift
 * 
 * @warning Home updates stop once armed to prevent in-flight position reference changes
 * @warning Requires valid 3D GPS fix - 2D fix insufficient for safe home location
 * @warning Breaking circular altitude dependency requires using GPS alt not AHRS alt
 * 
 * @see set_home_persistently() for manual home location setting
 * @see AP_AHRS::set_home() for AHRS home location update
 * @see barometer.update_calibration() for baro drift correction
 * 
 * Source: ArduPlane/commands.cpp:113-157
 */
bool Plane::update_home()
{
    if (hal.util->was_watchdog_armed()) {
        return false;
    }
    if ((g2.home_reset_threshold == -1) ||
        ((g2.home_reset_threshold > 0) &&
         (fabsf(barometer.get_altitude()) > g2.home_reset_threshold))) {
        // don't auto-update if we have changed barometer altitude
        // significantly. This allows us to cope with slow baro drift
        // but not re-do home and the baro if we have changed height
        // significantly
        return false;
    }
    bool ret = false;
    if (ahrs.home_is_set() && !ahrs.home_is_locked() && gps.status() >= AP_GPS::GPS_OK_FIX_3D) {
        Location loc;
        if (ahrs.get_location(loc)) {
            // we take the altitude directly from the GPS as we are
            // about to reset the baro calibration. We can't use AHRS
            // altitude or we can end up perpetuating a bias in
            // altitude, as AHRS alt depends on home alt, which means
            // we would have a circular dependency
            loc.set_alt_cm(gps.location().alt,
                           Location::AltFrame::ABSOLUTE);
            ret = AP::ahrs().set_home(loc);
        }
    }

    // even if home is not updated we do a baro reset to stop baro
    // drift errors while disarmed
    barometer.update_calibration();
    ahrs.resetHeightDatum();

    update_current_loc();

    return ret;
}

/**
 * @brief Set home location persistently to specified position
 * 
 * @param[in] loc Desired home location (lat/lon/alt in specified frame)
 * 
 * @return true if home was set successfully, false if operation prevented or failed
 * 
 * @details Manually sets the vehicle home location to a specified position, typically
 *          used for:
 *          - Ground station commanded home relocation
 *          - Rally point home positioning
 *          - Scripting interface home updates
 *          - MAVLink DO_SET_HOME command processing
 * 
 *          **Safety Checks**:
 *          - Prevents home change if watchdog was armed (post-watchdog-reset state)
 *          - Returns false if AP::ahrs().set_home() fails (invalid location or
 *            home is locked)
 *          
 *          **Home Persistence**:
 *          - Function name indicates "persistently" but actual persistence depends on
 *            AHRS implementation and parameter storage
 *          - Home location used for RTL, geofencing, altitude reference frame
 *          - Critical for safe return-to-launch behavior
 * 
 *          **Typical Usage Scenarios**:
 *          - MAVLink command to set home before arming
 *          - Lua scripting setting custom launch location
 *          - Rally point system setting alternate home
 *          - Testing/simulation with specific home coordinates
 * 
 * @note Does not perform barometer calibration (unlike update_home)
 * @note Does not validate location safety or reachability
 * @note Home can be locked via AHRS to prevent changes
 * 
 * @warning No altitude sanity checking - caller must validate location
 * @warning Changing home while armed is prevented at AHRS level
 * @warning Incorrect home location can cause RTL to wrong position
 * 
 * @see update_home() for automatic GPS-based home updates
 * @see AP_AHRS::set_home() for AHRS home setting with validation
 * @see handle_command_int_do_set_home() for MAVLink command handler
 * 
 * Source: ArduPlane/commands.cpp:159-169
 */
bool Plane::set_home_persistently(const Location &loc)
{
    if (hal.util->was_watchdog_armed()) {
        return false;
    }
    if (!AP::ahrs().set_home(loc)) {
        return false;
    }

    return true;
}
