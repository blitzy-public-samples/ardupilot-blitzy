/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * @file altitude.cpp
 * @brief Altitude control and management for fixed-wing aircraft
 * 
 * @details This file implements comprehensive altitude management for ArduPlane,
 *          handling multiple altitude reference frames and control strategies:
 *          
 *          **Altitude Reference Frames:**
 *          - Absolute (AMSL): Altitude above mean sea level from barometer
 *          - Relative: Altitude relative to home position
 *          - Terrain-following: Altitude above ground level using terrain database
 *          - Rangefinder: Direct ground distance measurement for landing
 *          
 *          **Core Responsibilities:**
 *          - Target altitude calculation and tracking
 *          - Altitude error computation for TECS controller integration
 *          - Terrain following with lookahead for obstacle avoidance
 *          - Rangefinder integration for precision landing
 *          - Altitude slope management for smooth waypoint transitions
 *          - Home altitude change compensation during flight
 *          - Flight mode-specific altitude behavior
 *          
 *          **TECS Integration:**
 *          The altitude targets computed here are passed to the Total Energy Control
 *          System (TECS) which manages the coupled altitude/airspeed control by
 *          commanding throttle and pitch to maintain total energy state.
 *          
 *          **Safety Considerations:**
 *          - Geofence altitude limits are enforced
 *          - Terrain collision avoidance through lookahead
 *          - Rangefinder failsafe detection for landing
 *          - Home altitude change detection and compensation
 * 
 * @note Altitude values are typically stored in centimeters internally
 * @warning Terrain following requires terrain database and GPS lock
 * 
 * @see AP_TECS for energy management controller
 * @see AP_Terrain for terrain database interface
 * @see Plane::target_altitude for altitude target state
 * 
 * Source: ArduPlane/altitude.cpp
 */

#include "Plane.h"

/*
  altitude handling routines. These cope with both barometric control
  and terrain following control
 */

/**
 * @brief Adjust altitude target based on current flight mode
 * 
 * @details Delegates altitude target adjustment to the current flight mode's
 *          specific implementation. Each mode may handle altitude differently:
 *          - AUTO: Follows mission waypoint altitudes with slope management
 *          - FBWB/CRUISE: Manual altitude adjustment via pilot input
 *          - RTL: Climbs to RTL altitude then maintains
 *          - GUIDED: Follows GCS commanded altitude
 *          - LOITER: Maintains altitude set when entering mode
 *          
 *          The mode-specific update_target_altitude() method may:
 *          - Update target_altitude.amsl_cm (absolute altitude target)
 *          - Update target_altitude.terrain_alt_cm (terrain-relative target)
 *          - Modify terrain_following flag
 *          - Adjust altitude offsets for slope management
 *          
 *          This function is called regularly from the main control loop to
 *          ensure altitude targets stay current with mode requirements and
 *          pilot inputs.
 * 
 * @note Called at main loop rate (typically 50Hz for fixed-wing)
 * @warning Mode transitions should set initial altitude targets before
 *          relying on adjust_altitude_target()
 * 
 * @see Mode::update_target_altitude()
 * @see Plane::target_altitude
 */
void Plane::adjust_altitude_target()
{
    control_mode->update_target_altitude();
}

/**
 * @brief Detect and compensate for home altitude changes during flight
 * 
 * @details The home altitude can change during flight due to:
 *          - Barometer drift correction from GCS
 *          - GPS altitude updates when home is reset
 *          - Atmospheric pressure changes over long flights
 *          
 *          When a change is detected while armed, this function:
 *          1. Calculates the altitude change magnitude
 *          2. Fixes terrain waypoint references to maintain correct altitude
 *          3. Offsets TECS field elevation estimate to prevent altitude jumps
 *          
 *          The TECS offset prevents the aircraft from suddenly changing
 *          altitude when the home reference shifts, maintaining smooth flight.
 * 
 * @note Only processes changes when armed to avoid pre-flight adjustments
 * @warning Large home altitude changes (>100m) may indicate GPS or barometer issues
 * 
 * @see TECS_controller.offset_altitude()
 * @see fix_terrain_WP()
 * @see auto_state.last_home_alt_cm
 */
void Plane::check_home_alt_change(void)
{
    int32_t home_alt_cm = ahrs.get_home().alt;
    if (home_alt_cm != auto_state.last_home_alt_cm && hal.util->get_soft_armed()) {
        // cope with home altitude changing
        const int32_t alt_change_cm = home_alt_cm - auto_state.last_home_alt_cm;
        fix_terrain_WP(next_WP_loc, __LINE__);

        // reset TECS to force the field elevation estimate to reset
        TECS_controller.offset_altitude(alt_change_cm * 0.01f);
    }
    auto_state.last_home_alt_cm = home_alt_cm;
}

/**
 * @brief Configure gradual altitude slope transition to next waypoint
 * 
 * @details Determines whether to use gradual altitude slope or rapid altitude
 *          change when transitioning between waypoints. The decision depends on:
 *          - Current flight mode (AUTO, RTL, GUIDED, etc.)
 *          - Current altitude relative to target
 *          - Flight stage (climb, cruise, land)
 *          - FlightOptions configuration
 *          
 *          **Slope Strategy by Mode:**
 *          
 *          **RTL/GUIDED/AVOID_ADSB:**
 *          - Above target: Glide down slowly (gradual slope)
 *          - Below target: Climb rapidly (no slope, full climb rate)
 *          - Rationale: Safety priority - get to safe altitude quickly when low
 *          
 *          **AUTO Mode:**
 *          - If IMMEDIATE_CLIMB_IN_AUTO option enabled and below target: Rapid climb
 *          - If above 20m AGL or descending: Gradual slope
 *          - If below 20m and climbing: Rapid climb (obstacle avoidance)
 *          
 *          The 20m threshold prevents slow climbs at low altitude where obstacles
 *          may be present, prioritizing obstacle clearance over smooth flight.
 *          
 *          **Path Proportion:**
 *          Calculates position along the path from prev_WP to next_WP, used by
 *          TECS for path-aware altitude management.
 * 
 * @note Called when setting up new waypoint navigation
 * @warning Altitude slope is disabled during landing (flight_stage == LAND)
 * 
 * @see set_offset_altitude_location()
 * @see reset_offset_altitude()
 * @see TECS_controller.set_path_proportion()
 * @see target_altitude.offset_cm
 */
void Plane::setup_alt_slope(void)
{
    // establish the distance we are travelling to the next waypoint,
    // for calculating out rate of change of altitude
    auto_state.wp_distance = current_loc.get_distance(next_WP_loc);
    auto_state.wp_proportion = current_loc.line_path_proportion(prev_WP_loc, next_WP_loc);
    TECS_controller.set_path_proportion(auto_state.wp_proportion);
    update_flight_stage();

    /*
      work out if we will gradually change altitude, or try to get to
      the new altitude as quickly as possible.
     */
    switch (control_mode->mode_number()) {
#if MODE_AUTOLAND_ENABLED
    case Mode::Number::AUTOLAND:
#endif
    case Mode::Number::RTL:
    case Mode::Number::AVOID_ADSB:
    case Mode::Number::GUIDED:
        /* glide down slowly if above target altitude, but ascend more
           rapidly if below it. See
           https://github.com/ArduPilot/ardupilot/issues/39
        */
        if (above_location_current(next_WP_loc)) {
            set_offset_altitude_location(prev_WP_loc, next_WP_loc);
        } else {
            reset_offset_altitude();
        }
        break;

    case Mode::Number::AUTO:
        // climb without doing slope if option is enabled
        if (!above_location_current(next_WP_loc) && plane.flight_option_enabled(FlightOptions::IMMEDIATE_CLIMB_IN_AUTO)) {
            reset_offset_altitude();
            break;
        }

        // we only do glide slide handling in AUTO when above 20m or
        // when descending. The 20 meter threshold is arbitrary, and
        // is basically to prevent situations where we try to slowly
        // gain height at low altitudes, potentially hitting
        // obstacles.
        if (adjusted_relative_altitude_cm() > 2000 || above_location_current(next_WP_loc)) {
            set_offset_altitude_location(prev_WP_loc, next_WP_loc);
        } else {
            reset_offset_altitude();
        }
        break;
    default:
        reset_offset_altitude();
        break;
    }
}

/**
 * @brief Calculate return-to-launch (RTL) altitude in AMSL centimeters
 * 
 * @details Returns the target altitude for RTL mode as absolute altitude above
 *          mean sea level (AMSL) in centimeters. Behavior depends on RTL_altitude
 *          parameter configuration:
 *          
 *          - RTL_altitude < 0: Maintain current altitude (stay at present height)
 *          - RTL_altitude >= 0: Climb/descend to RTL_altitude meters above home
 *          
 *          The RTL altitude is added to home.alt to convert from relative altitude
 *          (above home) to absolute AMSL altitude for consistent altitude control
 *          regardless of home elevation.
 * 
 * @return int32_t RTL target altitude in centimeters AMSL
 * 
 * @note RTL_altitude parameter is in meters, converted to centimeters for return
 * @note Negative RTL_altitude maintains current altitude, useful for emergency RTL
 * 
 * @see g.RTL_altitude parameter
 * @see home.alt for home position altitude reference
 */
int32_t Plane::get_RTL_altitude_cm() const
{
    if (g.RTL_altitude < 0) {
        return current_loc.alt;
    }
    return g.RTL_altitude*100 + home.alt;
}

/**
 * @brief Get aircraft height above ground in meters
 * 
 * @details Returns altitude relative to ground using the best available source
 *          in priority order:
 *          
 *          1. **External HAGL** (Height Above Ground Level) from MAVLink if available
 *          2. **Rangefinder** data if in range and enabled for current operation
 *          3. **Terrain database** altitude if available and requested
 *          4. **Waypoint height** during VTOL landing descent (quadplane only)
 *          5. **Barometric relative altitude** as fallback (height above home)
 *          
 *          **Special Cases:**
 *          - Quadplane landing with rangefinder OutOfRangeLow: Returns 0.0 (on ground)
 *          - VTOL landing descent: Uses waypoint height as ground reference
 *          
 *          This prioritization ensures the most accurate ground clearance estimate
 *          for terrain following, obstacle avoidance, and landing operations.
 * 
 * @param[in] use_rangefinder Rangefinder usage mode (LANDING, CRUISE, ALL, etc.)
 * @param[in] use_terrain_if_available If true, use terrain database when available
 * 
 * @return float Altitude above ground in meters
 * 
 * @note Returns barometric relative altitude if no other source available
 * @warning Terrain following requires GPS lock and terrain database coverage
 * 
 * @see rangefinder_use()
 * @see AP_Terrain::height_above_terrain()
 * @see relative_altitude for barometric fallback
 */
float Plane::relative_ground_altitude(enum RangeFinderUse use_rangefinder, bool use_terrain_if_available)
{
#if AP_MAVLINK_MAV_CMD_SET_HAGL_ENABLED
   float height_AGL;
   // use external HAGL if available
   if (get_external_HAGL(height_AGL)) {
       return height_AGL;
   }
#endif // AP_MAVLINK_MAV_CMD_SET_HAGL_ENABLED

#if AP_RANGEFINDER_ENABLED
   if (rangefinder_use(use_rangefinder) && rangefinder_state.in_range) {
        return rangefinder_state.height_estimate;
   }
#endif

#if HAL_QUADPLANE_ENABLED && AP_RANGEFINDER_ENABLED
   if (rangefinder_use(use_rangefinder) && quadplane.in_vtol_land_final() &&
       rangefinder.status_orient(rangefinder_orientation()) == RangeFinder::Status::OutOfRangeLow) {
       // a special case for quadplane landing when rangefinder goes
       // below minimum. Consider our height above ground to be zero
       return 0;
   }
#endif

#if AP_TERRAIN_AVAILABLE
    float altitude;
    if (use_terrain_if_available &&
        terrain.status() == AP_Terrain::TerrainStatusOK &&
        terrain.height_above_terrain(altitude, true)) {
        return altitude;
    }
#endif

#if HAL_QUADPLANE_ENABLED
    if (quadplane.in_vtol_land_descent() &&
        !quadplane.landing_with_fixed_wing_spiral_approach()) {
        // when doing a VTOL landing we can use the waypoint height as
        // ground height. We can't do this if using the
        // LAND_FW_APPROACH as that uses the wp height as the approach
        // height
        return height_above_target();
    }
#endif

    return relative_altitude;
}

/**
 * @brief Determine if rangefinder should be used for specific operation
 * 
 * @details Checks the rangefinder_landing parameter bitmask to determine if
 *          the rangefinder is enabled for the requested use case. The parameter
 *          allows selective rangefinder usage for different flight phases:
 *          
 *          **RangeFinderUse Flags:**
 *          - NONE: Rangefinder disabled
 *          - TAKEOFF_LANDING: Use only during takeoff and landing
 *          - CRUISE: Use during cruise flight
 *          - ALL: Use rangefinder for all operations (overrides other bits)
 *          
 *          The ALL flag takes precedence - if set, rangefinder is enabled
 *          regardless of specific use case flags.
 * 
 * @param[in] use_rangefinder Specific use case to check (TAKEOFF_LANDING, CRUISE, etc.)
 * 
 * @return true if rangefinder should be used for the specified operation
 * @return false if rangefinder is disabled for this operation
 * 
 * @note Actual rangefinder usage also depends on sensor health and range validity
 * @see g.rangefinder_landing parameter
 * @see RangeFinderUse enum
 */
bool Plane::rangefinder_use(enum RangeFinderUse use_rangefinder) const
{
    const uint8_t use = uint8_t(g.rangefinder_landing.get());
    if (use == uint8_t(RangeFinderUse::NONE)) {
        return false;
    }
    if (use & uint8_t(RangeFinderUse::ALL)) {
        // if ALL bit is set then ignore other bits
        return true;
    }
    return (use & uint8_t(use_rangefinder)) != 0;
}

/**
 * @brief Get altitude above ground with automatic terrain following detection
 * 
 * @details Convenience wrapper that automatically uses terrain data if the vehicle
 *          is currently in terrain following mode. This simplifies calling code by
 *          automatically determining whether terrain database should be consulted
 *          based on current flight state.
 *          
 *          Delegates to relative_ground_altitude(use_rangefinder, use_terrain)
 *          with use_terrain set to current terrain_following state.
 * 
 * @param[in] use_rangefinder Rangefinder usage mode for this operation
 * 
 * @return float Altitude above ground in meters
 * 
 * @note Uses terrain database only if target_altitude.terrain_following is true
 * @see relative_ground_altitude(enum RangeFinderUse, bool)
 * @see target_altitude.terrain_following
 */
float Plane::relative_ground_altitude(enum RangeFinderUse use_rangefinder)
{
#if AP_TERRAIN_AVAILABLE
    return relative_ground_altitude(use_rangefinder, target_altitude.terrain_following);
#else
    return relative_ground_altitude(use_rangefinder, false);
#endif
}


/**
 * @brief Set target altitude to current aircraft altitude
 * 
 * @details Captures the current altitude as the new target altitude for
 *          altitude hold operations. Used when:
 *          - Entering altitude-hold modes (CRUISE, FBWB)
 *          - Pilot releases elevator stick (initiates altitude hold)
 *          - Transitioning from manual to altitude-controlled flight
 *          
 *          **Actions Performed:**
 *          1. Records current AMSL altitude as target_altitude.amsl_cm
 *          2. Resets any altitude slope offset to zero
 *          3. If terrain following enabled and terrain data available:
 *             - Enables terrain_following mode
 *             - Records current terrain altitude as target
 *          4. If terrain unavailable or disabled: Uses barometric altitude
 *          
 *          The function attempts to maintain terrain-following mode if currently
 *          active and terrain data is available. This provides smooth transitions
 *          when engaging altitude hold during terrain following flight.
 * 
 * @note Called at mode transitions and when pilot centers elevator stick
 * @warning Terrain following will be disabled if terrain data is not available
 * 
 * @see target_altitude structure
 * @see reset_offset_altitude()
 * @see terrain_enabled_in_current_mode()
 */
void Plane::set_target_altitude_current(void)
{
    // record altitude above sea level at the current time as our
    // target altitude
    target_altitude.amsl_cm = current_loc.alt;

    // reset any altitude slope offset
    reset_offset_altitude();

#if AP_TERRAIN_AVAILABLE
    // also record the terrain altitude if possible
    float terrain_altitude;
    if (terrain_enabled_in_current_mode() && terrain.height_above_terrain(terrain_altitude, true) && !terrain_disabled()) {
        target_altitude.terrain_following = true;
        target_altitude.terrain_alt_cm = terrain_altitude*100;
    } else {
        // if terrain following is disabled, or we don't know our
        // terrain altitude when we set the altitude then don't
        // terrain follow
        target_altitude.terrain_following = false;        
    }
#endif
}

/**
 * @brief Set target altitude from a Location structure
 * 
 * @details Extracts altitude from a Location and sets it as the target altitude.
 *          Handles both relative and absolute altitude frames, and terrain-based
 *          altitudes if terrain following is active.
 *          
 *          **Altitude Frame Handling:**
 *          
 *          **Relative Altitude** (loc.relative_alt == true):
 *          - Adds home.alt to convert from relative to AMSL
 *          - Common for waypoints specified relative to home
 *          
 *          **Absolute Altitude** (loc.relative_alt == false):
 *          - Uses loc.alt directly as AMSL altitude
 *          - Common for mission items with absolute altitudes
 *          
 *          **Terrain Altitude** (loc.terrain_alt == true):
 *          - If terrain data available: Enables terrain following mode
 *          - Sets terrain_alt_cm to maintain altitude above ground
 *          - If terrain unavailable: Falls back to barometric altitude
 *          
 *          **Pending Terrain Data:**
 *          If terrain_following_pending flag is set (previous terrain data
 *          unavailable), retries terrain initialization with this waypoint.
 * 
 * @param[in] loc Location containing target altitude and frame information
 * 
 * @note All altitudes stored internally in centimeters
 * @warning Terrain altitude requires terrain database coverage and GPS lock
 * 
 * @see Location structure
 * @see setup_terrain_target_alt()
 * @see target_altitude.terrain_following_pending
 */
void Plane::set_target_altitude_location(const Location &loc)
{
    target_altitude.amsl_cm = loc.alt;
    if (loc.relative_alt) {
        target_altitude.amsl_cm += home.alt;
    }
#if AP_TERRAIN_AVAILABLE
    if (target_altitude.terrain_following_pending) {
        /* we didn't get terrain data to init when we started on this
           target, retry
        */
        setup_terrain_target_alt(next_WP_loc);
    }
    /*
      if this location has the terrain_alt flag set and we know the
      terrain altitude of our current location then treat it as a
      terrain altitude
     */
    float height;
    if (loc.terrain_alt && terrain.height_above_terrain(height, true)) {
        target_altitude.terrain_following = true;
        target_altitude.terrain_alt_cm = loc.alt;
    } else {
        target_altitude.terrain_following = false;
    }
#endif
}

/**
 * @brief Calculate target altitude relative to home for TECS controller
 * 
 * @details Computes the target altitude in centimeters relative to home altitude,
 *          incorporating all altitude adjustments and corrections. This is the
 *          primary altitude reference used by the TECS (Total Energy Control System)
 *          controller for coupled altitude/airspeed management.
 *          
 *          **TECS Integration:**
 *          TECS uses this relative altitude target combined with current relative
 *          altitude to compute altitude error, then manages throttle and pitch to
 *          maintain total energy (kinetic + potential) of the aircraft.
 *          
 *          **Computation for Terrain Following Mode:**
 *          1. Convert terrain altitude to home-relative equivalent
 *          2. Add terrain lookahead adjustment (obstacle avoidance)
 *          3. Add rangefinder correction (landing precision)
 *          
 *          **Computation for Barometric Mode:**
 *          1. Start with AMSL target minus home altitude
 *          2. Add mission altitude offset (ALT_OFFSET parameter)
 *          3. Add rangefinder correction if landing
 *          
 *          **Adjustments Applied:**
 *          - **Lookahead**: Raises altitude ahead of terrain rises (terrain following)
 *          - **Mission offset**: Global altitude adjustment from ALT_OFFSET parameter
 *          - **Rangefinder correction**: Precision altitude adjustment during landing
 * 
 * @return int32_t Target altitude in centimeters relative to home
 * 
 * @note This is the primary interface between altitude management and TECS
 * @warning Terrain following requires valid terrain database and GPS lock
 * 
 * @see TECS_controller for total energy control system
 * @see lookahead_adjustment() for terrain obstacle avoidance
 * @see rangefinder_correction() for landing precision
 * @see mission_alt_offset() for global altitude offset
 */
int32_t Plane::relative_target_altitude_cm(void)
{
#if AP_TERRAIN_AVAILABLE
    float relative_home_height;
    if (target_altitude.terrain_following && 
        terrain.height_relative_home_equivalent(target_altitude.terrain_alt_cm*0.01f,
                                                relative_home_height, true)) {
        // add lookahead adjustment the target altitude
        target_altitude.lookahead = lookahead_adjustment();
        relative_home_height += target_altitude.lookahead;

#if AP_RANGEFINDER_ENABLED
        // correct for rangefinder data
        relative_home_height += rangefinder_correction();
#endif

        // we are following terrain, and have terrain data for the
        // current location. Use it.
        return relative_home_height*100;
    }
#endif
    int32_t relative_alt = target_altitude.amsl_cm - home.alt;
    relative_alt += mission_alt_offset()*100;
#if AP_RANGEFINDER_ENABLED
    relative_alt += rangefinder_correction() * 100;
#endif
    return relative_alt;
}

/**
 * @brief Adjust target altitude by specified amount
 * 
 * @details Increments or decrements the target altitude by the specified amount
 *          in centimeters. Used for:
 *          - Pilot elevator input in CRUISE mode (altitude adjustment)
 *          - Pilot elevator input in FBWB mode (altitude adjustment)
 *          - Altitude slope proportion adjustments
 *          - Parameter-based altitude offsets
 *          
 *          **Terrain Following Behavior:**
 *          When terrain following is active and not disabled, both the AMSL
 *          target and the terrain-relative target are adjusted by the same
 *          amount. This maintains consistent altitude above ground while also
 *          tracking absolute altitude changes.
 *          
 *          **Use Cases:**
 *          - Pilot pulls back on elevator: change_cm > 0 (climb)
 *          - Pilot pushes forward on elevator: change_cm < 0 (descend)
 *          - Altitude slope proportional adjustment during waypoint nav
 * 
 * @param[in] change_cm Altitude change in centimeters (positive = climb)
 * 
 * @note Affects both AMSL and terrain-relative targets when terrain following
 * @see target_altitude.amsl_cm
 * @see target_altitude.terrain_alt_cm
 * @see terrain_disabled()
 */
void Plane::change_target_altitude(int32_t change_cm)
{
    target_altitude.amsl_cm += change_cm;
#if AP_TERRAIN_AVAILABLE
    if (target_altitude.terrain_following && !terrain_disabled()) {
        target_altitude.terrain_alt_cm += change_cm;
    }
#endif
}

/**
 * @brief Set target altitude based on position along altitude slope
 * 
 * @details Calculates target altitude as a proportion along the altitude slope
 *          between two waypoints. Used for smooth altitude transitions during
 *          waypoint navigation.
 *          
 *          **Proportion Meaning:**
 *          - proportion = 0.0: At destination waypoint (next_WP)
 *          - proportion = 1.0: At starting waypoint (prev_WP)
 *          - proportion = 0.5: Halfway between waypoints
 *          
 *          **Altitude Calculation:**
 *          1. Sets target to destination waypoint altitude
 *          2. Applies proportional offset: altitude = dest_alt - (offset * proportion)
 *          3. As vehicle advances (proportion → 0), altitude approaches destination
 *          
 *          **ALT_SLOPE_MAX_HEIGHT Handling:**
 *          If vehicle is climbing and has fallen more than ALT_SLOPE_MAX_HEIGHT
 *          below the slope, the slope is rebuilt from current position. This
 *          prevents the aircraft from trying to maintain an unrealistic climb
 *          rate to catch up to a slope it has fallen behind.
 *          
 *          **Example:**
 *          If flying from 100m to 200m altitude:
 *          - proportion = 1.0: Target = 100m (start)
 *          - proportion = 0.5: Target = 150m (middle)
 *          - proportion = 0.0: Target = 200m (destination)
 * 
 * @param[in] loc Target location with destination altitude
 * @param[in] proportion Position along path (0.0 = at destination, 1.0 = at start)
 * 
 * @note Proportion is constrained to [0.0, 1.0] range
 * @warning Slope rebuilding requires ALT_SLOPE_MAX_HEIGHT > 0
 * 
 * @see target_altitude.offset_cm
 * @see g.alt_slope_max_height parameter
 * @see calc_altitude_error_cm()
 */
void Plane::set_target_altitude_proportion(const Location &loc, float proportion)
{
    set_target_altitude_location(loc);
    proportion = constrain_float(proportion, 0.0f, 1.0f);
    change_target_altitude(-target_altitude.offset_cm*proportion);

    // rebuild the altitude slope if we are above it and supposed to be climbing
    if (g.alt_slope_max_height > 0) {
        if (target_altitude.offset_cm > 0 && calc_altitude_error_cm() < -100 * g.alt_slope_max_height) {
            set_target_altitude_location(loc);
            set_offset_altitude_location(current_loc, loc);
            change_target_altitude(-target_altitude.offset_cm*proportion);
            // adjust the new target offset altitude to reflect that we are partially already done
            if (proportion > 0.0f)
                target_altitude.offset_cm = ((float)target_altitude.offset_cm)/proportion;
        }
    }
}

#if AP_TERRAIN_AVAILABLE
/*
  change target altitude along a path between two locations
  (prev_WP_loc and next_WP_loc) where the second location is a terrain
  altitude
 */
bool Plane::set_target_altitude_proportion_terrain(void)
{
    if (!next_WP_loc.terrain_alt ||
        !next_WP_loc.relative_alt) {
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        return false;
    }
    /*
      we first need to get the height of the terrain at prev_WP_loc
     */
    float prev_WP_height_terrain;
    if (!plane.prev_WP_loc.get_alt_m(Location::AltFrame::ABOVE_TERRAIN,
                                     prev_WP_height_terrain)) {
        return false;
    }
    // and next_WP_loc alt as terrain
    float next_WP_height_terrain;
    if (!plane.next_WP_loc.get_alt_m(Location::AltFrame::ABOVE_TERRAIN,
                                     next_WP_height_terrain)) {
        return false;
    }
    Location loc = next_WP_loc;
    const auto alt = linear_interpolate(prev_WP_height_terrain, next_WP_height_terrain,
                                        plane.auto_state.wp_proportion, 0, 1);

    loc.set_alt_m(alt, Location::AltFrame::ABOVE_TERRAIN);

    set_target_altitude_location(loc);

    return true;
}
#endif // AP_TERRAIN_AVAILABLE

/*
  constrain target altitude to be between two locations. Used to
  ensure we stay within two waypoints in altitude
 */
void Plane::constrain_target_altitude_location(const Location &loc1, const Location &loc2)
{
    if (loc1.alt > loc2.alt) {
        target_altitude.amsl_cm = constrain_int32(target_altitude.amsl_cm, loc2.alt, loc1.alt);
    } else {
        target_altitude.amsl_cm = constrain_int32(target_altitude.amsl_cm, loc1.alt, loc2.alt);
    }
}

/**
 * @brief Calculate altitude tracking error in centimeters
 * 
 * @details Computes the difference between target altitude and current altitude,
 *          accounting for terrain following mode and lookahead adjustments. This
 *          error is the primary input to the altitude control system (TECS).
 *          
 *          **Terrain Following Mode:**
 *          When terrain following is active and terrain data is available:
 *          - Error = (terrain_target + lookahead) - current_terrain_altitude
 *          - Lookahead raises target ahead of terrain rises (obstacle avoidance)
 *          - Uses terrain database for both target and current altitude
 *          
 *          **Barometric Mode:**
 *          When not terrain following or terrain data unavailable:
 *          - Error = target_AMSL - adjusted_current_AMSL
 *          - Uses barometric altitude with mission offset adjustments
 *          - Absolute altitude above mean sea level reference
 *          
 *          **Error Sign Convention:**
 *          - Positive error: Aircraft below target (need to climb)
 *          - Negative error: Aircraft above target (need to descend)
 *          - Zero error: At target altitude
 *          
 *          **TECS Integration:**
 *          This error is passed to TECS controller which commands throttle and
 *          pitch to achieve target altitude while maintaining airspeed and total
 *          energy management.
 * 
 * @return int32_t Altitude error in centimeters (positive = below target)
 * 
 * @note Critical function for altitude control loop - called at main loop rate
 * @warning Terrain following error calculation requires valid terrain data
 * 
 * @see target_altitude for target altitude state
 * @see adjusted_altitude_cm() for current barometric altitude
 * @see lookahead_adjustment() for terrain obstacle avoidance
 * @see TECS_controller for total energy control system
 */
int32_t Plane::calc_altitude_error_cm(void)
{
#if AP_TERRAIN_AVAILABLE
    float terrain_height;
    if (target_altitude.terrain_following && 
        terrain.height_above_terrain(terrain_height, true)) {
        return target_altitude.lookahead*100 + target_altitude.terrain_alt_cm - (terrain_height*100);
    }
#endif
    return target_altitude.amsl_cm - adjusted_altitude_cm();
}

/**
 * @brief Enforce altitude limits from geofence and cruise parameters
 * 
 * @details Constrains target altitude to respect configured minimum and maximum
 *          altitude limits from geofence and cruise altitude floor parameter.
 *          Used primarily in FBWB and CRUISE modes to prevent pilot from
 *          commanding altitudes outside safe limits.
 *          
 *          **Altitude Limit Sources:**
 *          
 *          **Geofence Limits** (if enabled):
 *          - AC_FENCE_TYPE_ALT_MIN: Minimum safe altitude with margin
 *          - AC_FENCE_TYPE_ALT_MAX: Maximum safe altitude with margin
 *          
 *          **Cruise Altitude Floor** (CRUISE_ALT_FLOOR parameter):
 *          - Minimum altitude for FBWB mode
 *          - Prevents inadvertent low-altitude flight
 *          
 *          **Terrain Following Behavior:**
 *          When terrain following is active, limits are applied to terrain-relative
 *          altitude (terrain_alt_cm) rather than AMSL altitude. This maintains
 *          ground clearance regardless of terrain elevation.
 *          
 *          **Safety Rationale:**
 *          Prevents pilot error in manual altitude control modes from violating
 *          safety boundaries such as minimum terrain clearance or maximum
 *          regulatory altitude limits.
 * 
 * @note Only enforces limits if geofence or cruise floor is configured
 * @warning Terrain following limits require valid terrain database
 * 
 * @see fence.get_safe_alt_min()
 * @see fence.get_safe_alt_max()
 * @see g.cruise_alt_floor parameter
 * @see target_altitude.terrain_alt_cm
 */
void Plane::check_fbwb_altitude(void)
{
    float max_alt_cm = 0.0;
    float min_alt_cm = 0.0;
    bool should_check_max = false;
    bool should_check_min = false;

#if AP_FENCE_ENABLED
    // taking fence max and min altitude (with margin)
    const uint8_t enabled_fences = plane.fence.get_enabled_fences();
    if ((enabled_fences & AC_FENCE_TYPE_ALT_MIN) != 0) {
        min_alt_cm = plane.fence.get_safe_alt_min()*100.0;
        should_check_min = true;
    }
    if ((enabled_fences & AC_FENCE_TYPE_ALT_MAX) != 0) {
        max_alt_cm = plane.fence.get_safe_alt_max()*100.0;
        should_check_max = true;
    }
#endif

    if (g.cruise_alt_floor > 0) {
        // FBWB min altitude exists
        min_alt_cm = MAX(min_alt_cm, plane.g.cruise_alt_floor*100.0);
        should_check_min = true;
    }

    if (!should_check_min && !should_check_max) {
        return;
    }

//check if terrain following (min and max)
#if AP_TERRAIN_AVAILABLE
    if (target_altitude.terrain_following) {
        // set our target terrain height to be at least the min set
        if (should_check_max) {
            target_altitude.terrain_alt_cm = MIN(target_altitude.terrain_alt_cm, max_alt_cm);
        }
        if (should_check_min) {
            target_altitude.terrain_alt_cm = MAX(target_altitude.terrain_alt_cm, min_alt_cm);
        }
        return;
    }
#endif

    if (should_check_max) {
        target_altitude.amsl_cm = MIN(target_altitude.amsl_cm, home.alt + max_alt_cm);
    }
    if (should_check_min) {
        target_altitude.amsl_cm = MAX(target_altitude.amsl_cm, home.alt + min_alt_cm);
    }
}

/**
 * @brief Reset altitude slope offset to zero
 * 
 * @details Clears the altitude slope offset, disabling gradual altitude
 *          transitions. After calling this, the aircraft will attempt to
 *          reach target altitude as quickly as possible rather than following
 *          a gradual slope.
 *          
 *          **Called When:**
 *          - Entering modes that don't use altitude slopes
 *          - Target altitude change requires immediate altitude adjustment
 *          - Altitude hold engaged (want to maintain current altitude)
 *          - Within ALT_SLOPE_MIN of target altitude (close enough)
 *          
 *          Setting offset to zero causes relative_target_altitude_cm() to
 *          return the destination altitude directly without proportional
 *          slope calculation.
 * 
 * @see target_altitude.offset_cm
 * @see set_offset_altitude_location()
 * @see set_target_altitude_proportion()
 */
void Plane::reset_offset_altitude(void)
{
    target_altitude.offset_cm = 0;
}


/**
 * @brief Calculate and set altitude slope offset between two locations
 * 
 * @details Computes the altitude difference between start and destination
 *          locations and stores it as the altitude slope offset. This offset
 *          is used to create gradual altitude transitions during waypoint
 *          navigation.
 *          
 *          **Offset Calculation:**
 *          - offset = destination_altitude - start_altitude (in centimeters)
 *          - Positive offset: Destination is higher (climbing)
 *          - Negative offset: Destination is lower (descending)
 *          - Zero offset: Same altitude (level flight)
 *          
 *          **Terrain Following Mode:**
 *          When terrain following and destination has terrain_alt flag:
 *          - offset = terrain_target - current_terrain_height
 *          - Maintains terrain-relative altitude slope
 *          
 *          **ALT_SLOPE_MIN Optimization:**
 *          If altitude difference is less than ALT_SLOPE_MIN parameter, the
 *          offset is set to zero. This prevents trying to follow tiny altitude
 *          slopes that don't meaningfully affect flight, reducing unnecessary
 *          altitude adjustments near waypoint turns.
 *          
 *          **Landing Exception:**
 *          During landing (flight_stage == LAND), ALT_SLOPE_MIN check is
 *          skipped to ensure precise altitude control throughout the landing
 *          sequence.
 * 
 * @param[in] start_loc Starting location for slope calculation
 * @param[in] destination_loc Destination location for slope calculation
 * 
 * @note Offset stored in centimeters in target_altitude.offset_cm
 * @warning Requires valid altitude frames in both locations
 * 
 * @see target_altitude.offset_cm
 * @see g.alt_slope_min parameter
 * @see set_target_altitude_proportion()
 */
void Plane::set_offset_altitude_location(const Location &start_loc, const Location &destination_loc)
{
    ftype alt_difference_m = 0;
    if (destination_loc.get_height_above(start_loc, alt_difference_m)) {
        target_altitude.offset_cm = alt_difference_m * 100;
    } else {
        target_altitude.offset_cm = 0;
    }

#if AP_TERRAIN_AVAILABLE
    /*
      if this location has the terrain_alt flag set and we know the
      terrain altitude of our current location then treat it as a
      terrain altitude
     */
    float height;
    if (destination_loc.terrain_alt && 
        target_altitude.terrain_following &&
        terrain.height_above_terrain(height, true)) {
        target_altitude.offset_cm = target_altitude.terrain_alt_cm - (height * 100);
    }
#endif

    if (flight_stage != AP_FixedWing::FlightStage::LAND) {
        // if we are within ALT_SLOPE_MIN meters of the target altitude then
        // reset the offset to not use an altitude slope. This allows for more
        // accurate flight of missions where the aircraft may lose or gain a bit
        // of altitude near waypoint turn points due to local terrain changes
        if (g.alt_slope_min <= 0 ||
            labs(target_altitude.offset_cm)*0.01f < g.alt_slope_min) {
            target_altitude.offset_cm = 0;
        }
    }
}

/**
 * @brief Determine if current position is above specified location
 * 
 * @details Compares current altitude with target location altitude, accounting
 *          for terrain following mode. The definition of "above" changes based
 *          on the altitude reference frame:
 *          
 *          **Barometric Mode (not terrain following):**
 *          "Above" means current barometric altitude exceeds location barometric
 *          altitude. Simple pressure altitude comparison.
 *          
 *          **Terrain Following Mode:**
 *          "Above" means current height above terrain exceeds location's specified
 *          height above terrain. This is frame-independent - the aircraft can be
 *          "above" a waypoint in terrain terms while being at lower barometric
 *          altitude if flying through a valley.
 *          
 *          **Example Scenario:**
 *          - Aircraft at 1000m AMSL over 900m terrain (100m AGL)
 *          - Waypoint at 1100m AMSL over 1050m terrain (50m AGL)
 *          - Barometric: Aircraft below waypoint (1000m < 1100m)
 *          - Terrain: Aircraft above waypoint (100m > 50m)
 *          
 *          **Use Cases:**
 *          - Determining altitude slope direction (climb vs descend)
 *          - Deciding whether to use gradual descent or rapid climb
 *          - Waypoint arrival detection based on altitude
 * 
 * @param[in] loc Location to compare against current position
 * 
 * @return true if current position is above the specified location
 * @return false if current position is at or below the specified location
 * 
 * @note Handles both relative and absolute altitude frames in location
 * @see setup_alt_slope() for usage in slope calculation
 * @see terrain.height_above_terrain()
 */
bool Plane::above_location_current(const Location &loc)
{
#if AP_TERRAIN_AVAILABLE
    float terrain_alt;
    if (loc.terrain_alt && 
        terrain.height_above_terrain(terrain_alt, true)) {
        float loc_alt = loc.alt*0.01f;
        if (!loc.relative_alt) {
            loc_alt -= home.alt*0.01f;
        }
        return terrain_alt > loc_alt;
    }
#endif

    float loc_alt_cm = loc.alt;
    if (loc.relative_alt) {
        loc_alt_cm += home.alt;
    }
    return current_loc.alt > loc_alt_cm;
}

/*
  modify a destination to be setup for terrain following if
  TERRAIN_FOLLOW is enabled
 */
void Plane::setup_terrain_target_alt(Location &loc)
{
#if AP_TERRAIN_AVAILABLE
    if (terrain_enabled_in_current_mode()) {
        if (!loc.change_alt_frame(Location::AltFrame::ABOVE_TERRAIN)) {
            target_altitude.terrain_following_pending = true;
            return;
        }
    }
    target_altitude.terrain_following_pending = false;
#endif
}

/**
 * @brief Get current altitude adjusted for mission altitude offset
 * 
 * @details Returns current barometric altitude (AMSL) adjusted by the mission
 *          altitude offset. This offset compensates for:
 *          - Barometric drift during long flights
 *          - GCS-commanded altitude adjustments (ALT_OFFSET parameter)
 *          - Landing approach adjustments after aborted steep landings
 *          
 *          **Offset Purpose:**
 *          During long missions, barometric pressure changes can cause altitude
 *          errors. The ALT_OFFSET parameter allows GCS to correct these errors
 *          without modifying waypoint altitudes in the mission.
 *          
 *          **Calculation:**
 *          adjusted_altitude = current_AMSL - (ALT_OFFSET + landing_offset)
 *          
 *          The subtraction means a positive offset lowers the perceived altitude,
 *          causing the aircraft to fly higher to reach the same target.
 * 
 * @return int32_t Current altitude in centimeters AMSL, adjusted for offsets
 * 
 * @note Used for altitude error calculations and TECS control
 * @see mission_alt_offset() for offset computation
 * @see g.alt_offset parameter (ALT_OFFSET)
 * @see landing.alt_offset for landing-specific adjustments
 */
int32_t Plane::adjusted_altitude_cm(void)
{
    return current_loc.alt - (mission_alt_offset()*100);
}

/**
 * @brief Get relative altitude adjusted for mission altitude offset
 * 
 * @details Returns altitude relative to home, adjusted by mission altitude
 *          offset. This provides a relative altitude measurement that accounts
 *          for barometric drift corrections and GCS-commanded adjustments.
 *          
 *          **Calculation:**
 *          adjusted_relative = (relative_altitude - mission_offset) * 100
 *          
 *          Where:
 *          - relative_altitude: Current barometric altitude minus home altitude (meters)
 *          - mission_offset: ALT_OFFSET + landing approach offset (meters)
 *          - Result: Adjusted relative altitude in centimeters
 *          
 *          **Use Cases:**
 *          - Minimum altitude checks (e.g., 20m threshold in setup_alt_slope)
 *          - Relative altitude displays and logging
 *          - Altitude-based mode transition decisions
 * 
 * @return int32_t Altitude relative to home in centimeters, adjusted for offsets
 * 
 * @see adjusted_altitude_cm() for absolute altitude version
 * @see mission_alt_offset()
 * @see relative_altitude member variable
 */
int32_t Plane::adjusted_relative_altitude_cm(void)
{
    return (relative_altitude - mission_alt_offset())*100;
}


/**
 * @brief Calculate total mission altitude offset in meters
 * 
 * @details Computes the combined altitude offset applied to all altitude
 *          calculations. This offset adjusts the effective flight altitude
 *          without modifying stored waypoint altitudes.
 *          
 *          **Offset Components:**
 *          
 *          **ALT_OFFSET Parameter (g.alt_offset):**
 *          - GCS-adjustable offset for barometric drift compensation
 *          - Applied globally to all flight modes
 *          - Positive offset causes aircraft to fly higher
 *          - Typical use: Compensate for pressure changes during long missions
 *          
 *          **Landing Altitude Offset (landing.alt_offset):**
 *          - Automatically added during AUTO mode landing operations
 *          - Set after aborting a landing due to excessive glide slope
 *          - Helps aircraft approach from higher altitude on retry
 *          - Only active during LAND stage or land approach
 *          
 *          **Effect on Flight:**
 *          A positive offset makes the aircraft think it's lower than actual,
 *          causing it to fly higher to reach commanded altitudes. This is
 *          the desired behavior for compensating barometric drift.
 * 
 * @return float Total altitude offset in meters
 * 
 * @note Applied to both barometric and terrain-following altitude calculations
 * @see g.alt_offset parameter (ALT_OFFSET)
 * @see landing.alt_offset for aborted landing compensation
 * @see adjusted_altitude_cm()
 */
float Plane::mission_alt_offset(void)
{
    float ret = g.alt_offset;
    if (control_mode == &mode_auto &&
            (flight_stage == AP_FixedWing::FlightStage::LAND || auto_state.wp_is_land_approach)) {
        // when landing after an aborted landing due to too high glide
        // slope we use an offset from the last landing attempt
        ret += landing.alt_offset;
    }
    return ret;
}

/*
  return the height in meters above the next_WP_loc altitude
 */
float Plane::height_above_target(void)
{
    float target_alt = next_WP_loc.alt*0.01;
    if (!next_WP_loc.relative_alt) {
        target_alt -= ahrs.get_home().alt*0.01f;
    }

#if AP_TERRAIN_AVAILABLE
    // also record the terrain altitude if possible
    float terrain_altitude;
    if (next_WP_loc.terrain_alt && 
        terrain.height_above_terrain(terrain_altitude, true)) {
        return terrain_altitude - target_alt;
    }
#endif

    return (adjusted_altitude_cm()*0.01f - ahrs.get_home().alt*0.01f) - target_alt;
}

/**
 * @brief Calculate terrain lookahead altitude adjustment for obstacle avoidance
 * 
 * @details Computes altitude increase needed to safely clear terrain ahead of
 *          the aircraft during terrain following flight. This is a critical
 *          safety feature that raises the altitude target when terrain rises
 *          ahead, providing obstacle clearance based on aircraft climb capability.
 *          
 *          **Algorithm:**
 *          1. Determine bearing to target (or current heading if loitering)
 *          2. Calculate lookahead distance (constrained by TERRAIN_LOOKAHEAD param)
 *          3. Compute achievable climb ratio (50% of max climb rate / groundspeed)
 *          4. Query terrain database for altitude change ahead
 *          5. Adjust for current descent (if descending toward waypoint)
 *          6. Constrain result to reasonable range (0-1000m)
 *          
 *          **Climb Ratio:**
 *          Uses 50% of maximum TECS climb rate to avoid sustained full throttle
 *          and provide margin for maneuvering. This ensures the aircraft can
 *          achieve the commanded altitude increase without excessive stress.
 *          
 *          **Lookahead Distance:**
 *          - In waypoint navigation: Distance to waypoint (up to TERRAIN_LOOKAHEAD)
 *          - In FBWB: Fixed TERRAIN_LOOKAHEAD distance ahead
 *          - While loitering: Zero (no lookahead adjustment needed)
 *          
 *          **Descent Compensation:**
 *          If descending (negative offset_cm), lookahead is reduced since we're
 *          actively flying lower. This prevents over-conservative altitude during
 *          descents.
 *          
 *          **Safety Margin:**
 *          The 50% climb rate provides built-in safety margin - the aircraft
 *          should be able to achieve the lookahead altitude with throttle to spare.
 * 
 * @return float Altitude increase in meters needed for safe terrain clearance
 * 
 * @note Returns 0.0 if terrain following disabled or insufficient groundspeed
 * @warning Requires valid terrain database coverage along flight path
 * 
 * @see AP_Terrain::lookahead()
 * @see TECS_controller.get_max_climbrate()
 * @see g.terrain_lookahead parameter (TERRAIN_LOOKAHEAD)
 * @see target_altitude.lookahead
 */
float Plane::lookahead_adjustment(void)
{
#if AP_TERRAIN_AVAILABLE
    int32_t bearing_cd;
    int16_t distance;
    // work out distance and bearing to target
    if (control_mode == &mode_fbwb) {
        // there is no target waypoint in FBWB, so use yaw as an approximation
        bearing_cd = ahrs.yaw_sensor;
        distance = g.terrain_lookahead;
    } else if (!reached_loiter_target()) {
        bearing_cd = nav_controller->target_bearing_cd();
        distance = constrain_float(auto_state.wp_distance, 0, g.terrain_lookahead);
    } else {
        // no lookahead when loitering
        bearing_cd = 0;
        distance = 0;
    }
    if (distance <= 0) {
        // no lookahead
        return 0;
    }

    
    float groundspeed = ahrs.groundspeed();
    if (groundspeed < 1) {
        // we're not moving
        return 0;
    }
    // we need to know the climb ratio. We use 50% of the maximum
    // climb rate so we are not constantly at 100% throttle and to
    // give a bit more margin on terrain
    float climb_ratio = 0.5f * TECS_controller.get_max_climbrate() / groundspeed;

    if (climb_ratio <= 0) {
        // lookahead makes no sense for negative climb rates
        return 0;
    }
    
    // ask the terrain code for the lookahead altitude change
    float lookahead = terrain.lookahead(bearing_cd*0.01f, distance, climb_ratio);
    
    if (target_altitude.offset_cm < 0) {
        // we are heading down to the waypoint, so we don't need to
        // climb as much
        lookahead += target_altitude.offset_cm*0.01f;
    }

    // constrain lookahead to a reasonable limit
    return constrain_float(lookahead, 0, 1000.0f);
#else
    return 0;
#endif
}


#if AP_RANGEFINDER_ENABLED
/**
 * @brief Calculate rangefinder-based altitude correction for precision landing
 * 
 * @details Returns altitude correction in meters based on rangefinder measurements
 *          during landing operations. Positive correction means aircraft should
 *          fly higher to achieve proper landing altitude.
 *          
 *          **Purpose:**
 *          Rangefinder provides direct ground distance measurement, more accurate
 *          than barometric or terrain database altitude during final landing phase.
 *          This correction adjusts TECS altitude target to achieve precise landing
 *          flare and touchdown.
 *          
 *          **Correction Calculation:**
 *          The correction is computed in rangefinder_height_update() by comparing:
 *          - Rangefinder-measured height above ground
 *          - Expected height based on waypoint altitude or terrain database
 *          
 *          Difference is low-pass filtered to reduce noise and stored in
 *          rangefinder_state.correction.
 *          
 *          **Staleness Check:**
 *          Rangefinder data is discarded if more than 5 seconds old, preventing
 *          use of stale measurements that could cause dangerous altitude errors.
 *          
 *          **Usage Constraints:**
 *          Correction is only applied when:
 *          - Rangefinder enabled for TAKEOFF_LANDING
 *          - Currently in LAND flight stage
 *          - Rangefinder data is fresh (<5 seconds old)
 * 
 * @return float Altitude correction in meters (positive = fly higher)
 * 
 * @note Returns 0.0 if rangefinder not in use or data is stale
 * @warning Critical for landing safety - stale data could cause crash
 * 
 * @see rangefinder_height_update() for correction calculation
 * @see rangefinder_state.correction
 * @see rangefinder_state.last_correction_time_ms
 */
float Plane::rangefinder_correction(void)
{
    if (millis() - rangefinder_state.last_correction_time_ms > 5000) {
        // we haven't had any rangefinder data for 5s - don't use it
        return 0;
    }

    // for now we only support the rangefinder for landing 
    bool using_rangefinder = (rangefinder_use(RangeFinderUse::TAKEOFF_LANDING) && flight_stage == AP_FixedWing::FlightStage::LAND);
    if (!using_rangefinder) {
        return 0;
    }

    return rangefinder_state.correction;
}

/**
 * @brief Adjust rangefinder height for terrain elevation differences
 * 
 * @details Corrects rangefinder-measured height to account for terrain elevation
 *          changes between current aircraft position and the landing waypoint.
 *          This is essential for accurate landing when terrain slopes between
 *          the approach path and touchdown point.
 *          
 *          **Problem:**
 *          If aircraft is on approach over terrain that is higher/lower than
 *          the landing point, raw rangefinder reading will not match expected
 *          height above the touchdown point. Without correction, the aircraft
 *          would flare too early (if over high ground) or too late (if over
 *          low ground).
 *          
 *          **Correction Algorithm:**
 *          1. Query terrain database for AMSL altitude at current location
 *          2. Query terrain database for AMSL altitude at landing waypoint
 *          3. Calculate correction = current_terrain_AMSL - landing_terrain_AMSL
 *          4. Add correction to rangefinder height measurement
 *          
 *          **Example Scenario:**
 *          - Landing point at 100m terrain elevation
 *          - Aircraft currently over 120m terrain elevation
 *          - Rangefinder reads 50m above ground
 *          - Correction: +20m (aircraft is 20m higher terrain)
 *          - Corrected height: 70m (relative to landing point)
 *          
 *          **Requirements:**
 *          - Rangefinder enabled for TAKEOFF_LANDING
 *          - Currently in LAND flight stage
 *          - Terrain following enabled in current mode
 *          - Valid terrain data at both current and landing locations
 * 
 * @param[in,out] height Rangefinder height in meters, adjusted by terrain difference
 * 
 * @note Correction stored in auto_state.terrain_correction for logging
 * @warning Requires accurate terrain database coverage along approach path
 * 
 * @see AP_Terrain::height_amsl()
 * @see auto_state.terrain_correction
 */
void Plane::rangefinder_terrain_correction(float &height)
{
#if AP_TERRAIN_AVAILABLE
    if (!rangefinder_use(RangeFinderUse::TAKEOFF_LANDING) ||
        flight_stage != AP_FixedWing::FlightStage::LAND ||
        !terrain_enabled_in_current_mode()) {
        return;
    }
    float terrain_amsl1, terrain_amsl2;
    if (!terrain.height_amsl(current_loc, terrain_amsl1) ||
        !terrain.height_amsl(next_WP_loc, terrain_amsl2)) {
        return;
    }
    float correction = (terrain_amsl1 - terrain_amsl2);
    height += correction;
    auto_state.terrain_correction = correction;
#endif
}

/**
 * @brief Process rangefinder measurements and update altitude correction
 * 
 * @details Complex rangefinder processing function that validates sensor data,
 *          manages rangefinder engagement for landing, and calculates altitude
 *          corrections. This is called regularly during flight to maintain
 *          up-to-date rangefinder state.
 *          
 *          **Processing Pipeline:**
 *          
 *          **1. Data Validation:**
 *          - Check rangefinder status is Good
 *          - Correct distance measurement for aircraft attitude (DCM rotation)
 *          - Verify sensor pointing downward (positive Z component)
 *          
 *          **2. Attitude Correction:**
 *          Rangefinder measures slant range, corrected to vertical distance using
 *          DCM (Direction Cosine Matrix) to account for roll/pitch angles.
 *          If aircraft is tilted, actual ground clearance differs from raw reading.
 *          
 *          **3. In-Range Detection:**
 *          Requires 10 consecutive good samples (0.2s at 50Hz) that show 5%
 *          variation from initial reading. This validates:
 *          - Sensor is actually responding to ground (not stuck reading)
 *          - Sensor is stable and reliable
 *          - Prevents false engagement from misconfigured sensors
 *          
 *          Large jumps (>20% of max range) reset the counter, indicating
 *          unreliable data or sensor switching between targets.
 *          
 *          **4. Rangefinder Engagement:**
 *          Once validated, rangefinder automatically engages if:
 *          - In suitable flight stage (LAND, QLAND, QRTL)
 *          - Rangefinder enabled for TAKEOFF_LANDING
 *          - Not previously engaged
 *          
 *          Engagement message sent to GCS for pilot awareness.
 *          
 *          **5. Correction Calculation:**
 *          - Compare rangefinder height with expected height (waypoint or terrain)
 *          - Low-pass filter correction (0.8 old + 0.2 new) unless >5s stale
 *          - If correction drifts >30m from initial: Disengage (faulty sensor)
 *          
 *          **6. Terrain Correction:**
 *          Apply terrain elevation adjustment between aircraft and landing point
 *          to compensate for sloping ground.
 * 
 * @note Called at main loop rate during flight
 * @warning Faulty rangefinder can cause dangerous altitude errors during landing
 * 
 * @see rangefinder_terrain_correction()
 * @see rangefinder_state structure
 * @see ahrs.get_rotation_body_to_ned() for attitude correction
 */
void Plane::rangefinder_height_update(void)
{
    const auto orientation = rangefinder_orientation();
    bool range_ok = rangefinder.status_orient(orientation) == RangeFinder::Status::Good;
    float distance = rangefinder.distance_orient(orientation);
    float corrected_distance = distance;

    /*
      correct distance for attitude
     */
    if (range_ok) {
        // correct the range for attitude
        const auto &dcm = ahrs.get_rotation_body_to_ned();

        Vector3f v{corrected_distance, 0, 0};
        v.rotate(orientation);
        v = dcm * v;

        if (!is_positive(v.z)) {
            // not pointing at the ground
            range_ok = false;
        } else {
            corrected_distance = v.z;
        }
    }

    if (range_ok && ahrs.home_is_set()) {
        if (!rangefinder_state.have_initial_reading) {
            rangefinder_state.have_initial_reading = true;
            rangefinder_state.initial_range = distance;
        }
        rangefinder_state.height_estimate = corrected_distance;

        rangefinder_terrain_correction(rangefinder_state.height_estimate);

        // we consider ourselves to be fully in range when we have 10
        // good samples (0.2s) that are different by 5% of the maximum
        // range from the initial range we see. The 5% change is to
        // catch Lidars that are giving a constant range, either due
        // to misconfiguration or a faulty sensor
        if (rangefinder_state.in_range_count < 10) {
            if (!is_equal(distance, rangefinder_state.last_distance) &&
                fabsf(rangefinder_state.initial_range - distance) > 0.05f * rangefinder.max_distance_orient(rangefinder_orientation())) {
                rangefinder_state.in_range_count++;
            }
            if (fabsf(rangefinder_state.last_distance - distance) > rangefinder.max_distance_orient(rangefinder_orientation())*0.2) {
                // changes by more than 20% of full range will reset counter
                rangefinder_state.in_range_count = 0;
            }
        } else {
            rangefinder_state.in_range = true;
            bool flightstage_good_for_rangefinder_landing = false;
            if (flight_stage == AP_FixedWing::FlightStage::LAND) {
                flightstage_good_for_rangefinder_landing = true;
            }
#if HAL_QUADPLANE_ENABLED
            if (control_mode == &mode_qland ||
                control_mode == &mode_qrtl ||
                (control_mode == &mode_auto && quadplane.is_vtol_land(plane.mission.get_current_nav_cmd().id))) {
                flightstage_good_for_rangefinder_landing = true;
            }
#endif
            if (!rangefinder_state.in_use &&
                flightstage_good_for_rangefinder_landing &&
                rangefinder_use(RangeFinderUse::TAKEOFF_LANDING)) {
                rangefinder_state.in_use = true;
                gcs().send_text(MAV_SEVERITY_INFO, "Rangefinder engaged at %.2fm", (double)rangefinder_state.height_estimate);
            }
        }
        rangefinder_state.last_distance = distance;
    } else {
        rangefinder_state.in_range_count = 0;
        rangefinder_state.in_range = false;
    }

    if (rangefinder_state.in_range) {
        // If not using terrain data, we expect zero correction when our height above target is equal to our rangefinder measurement
        float correction = height_above_target() - rangefinder_state.height_estimate;

#if AP_TERRAIN_AVAILABLE
        // if we are terrain following then correction is based on terrain data
        float terrain_altitude;
        if ((target_altitude.terrain_following || terrain_enabled_in_current_mode()) && 
            terrain.height_above_terrain(terrain_altitude, true)) {
            correction = terrain_altitude - rangefinder_state.height_estimate;
        }
#endif    

        // remember the last correction. Use a low pass filter unless
        // the old data is more than 5 seconds old
        uint32_t now = millis();
        if (now - rangefinder_state.last_correction_time_ms > 5000) {
            rangefinder_state.correction = correction;
            rangefinder_state.initial_correction = correction;
            if (rangefinder_use(RangeFinderUse::TAKEOFF_LANDING)) {
                landing.set_initial_slope();
            }
            rangefinder_state.last_correction_time_ms = now;
        } else {
            rangefinder_state.correction = 0.8f*rangefinder_state.correction + 0.2f*correction;
            rangefinder_state.last_correction_time_ms = now;
            if (fabsf(rangefinder_state.correction - rangefinder_state.initial_correction) > 30) {
                // the correction has changed by more than 30m, reset use of Lidar. We may have a bad lidar
                if (rangefinder_state.in_use) {
                    gcs().send_text(MAV_SEVERITY_INFO, "Rangefinder disengaged at %.2fm", (double)rangefinder_state.height_estimate);
                }
                memset(&rangefinder_state, 0, sizeof(rangefinder_state));
            }
        }
        
    }
}
#endif  // AP_RANGEFINDER_ENABLED

/**
 * @brief Check if terrain following is currently disabled
 * 
 * @details Determines whether terrain following should be disabled based on:
 *          - Control mode allows terrain disable (mode-specific capability)
 *          - Pilot has activated non-auto terrain disable
 *          
 *          This provides pilot override to disable terrain following in certain
 *          modes (typically manual/semi-manual modes like FBWB, CRUISE) when
 *          the pilot wants direct altitude control without terrain following.
 *          
 *          **Use Case:**
 *          Pilot may temporarily disable terrain following if:
 *          - Terrain database has inaccuracies in local area
 *          - Flying in area with no terrain data coverage
 *          - Prefers barometric altitude hold for specific maneuvers
 *          - Testing or troubleshooting terrain following system
 * 
 * @return true if terrain following should be disabled, false otherwise
 * 
 * @note Only works in modes that allow terrain disable (FBWB, CRUISE, etc.)
 * @warning AUTO mode typically does not allow terrain disable for safety
 * 
 * @see control_mode->allows_terrain_disable()
 * @see non_auto_terrain_disable
 */
bool Plane::terrain_disabled()
{
    return control_mode->allows_terrain_disable() && non_auto_terrain_disable;
}


/*
  Check if terrain following is enabled for the current mode
 */
#if AP_TERRAIN_AVAILABLE
const Plane::TerrainLookupTable Plane::Terrain_lookup[] = {
    {Mode::Number::FLY_BY_WIRE_B, terrain_bitmask::FLY_BY_WIRE_B},
    {Mode::Number::CRUISE, terrain_bitmask::CRUISE},
    {Mode::Number::AUTO, terrain_bitmask::AUTO},
    {Mode::Number::RTL, terrain_bitmask::RTL},
    {Mode::Number::AVOID_ADSB, terrain_bitmask::AVOID_ADSB},
    {Mode::Number::GUIDED, terrain_bitmask::GUIDED},
    {Mode::Number::LOITER, terrain_bitmask::LOITER},
    {Mode::Number::CIRCLE, terrain_bitmask::CIRCLE},
#if HAL_QUADPLANE_ENABLED
    {Mode::Number::QRTL, terrain_bitmask::QRTL},
    {Mode::Number::QLAND, terrain_bitmask::QLAND},
    {Mode::Number::QLOITER, terrain_bitmask::QLOITER},
#endif
#if MODE_AUTOLAND_ENABLED
    {Mode::Number::AUTOLAND, terrain_bitmask::AUTOLAND},
#endif
};

/**
 * @brief Check if terrain following is enabled in the current flight mode
 * 
 * @details Convenience wrapper that checks if terrain following is enabled
 *          for the currently active flight mode.
 * 
 * @return true if terrain following enabled in current mode, false otherwise
 * 
 * @see terrain_enabled_in_mode()
 * @see g.terrain_follow parameter (TERRAIN_FOLLOW bitmask)
 */
bool Plane::terrain_enabled_in_current_mode() const
{
    return terrain_enabled_in_mode(control_mode->mode_number());
}

/**
 * @brief Check if terrain following is enabled for a specific flight mode
 * 
 * @details Determines terrain following status based on TERRAIN_FOLLOW parameter
 *          bitmask configuration. Each bit corresponds to a specific flight mode.
 *          
 *          **Configuration Options:**
 *          - ALL bit set: Terrain following enabled in all supported modes
 *          - Specific mode bits: Terrain following enabled only in those modes
 *          
 *          **Supported Modes:**
 *          - FLY_BY_WIRE_B (FBWB)
 *          - CRUISE
 *          - AUTO
 *          - RTL (Return to Launch)
 *          - AVOID_ADSB
 *          - GUIDED
 *          - LOITER
 *          - CIRCLE
 *          - QRTL (Quadplane RTL)
 *          - QLAND (Quadplane Land)
 *          - QLOITER (Quadplane Loiter)
 *          - AUTOLAND
 *          
 *          **Lookup Table:**
 *          Terrain_lookup[] table maps mode numbers to bitmask values for
 *          efficient mode checking.
 * 
 * @param[in] num Flight mode number to check
 * 
 * @return true if terrain following enabled for specified mode, false otherwise
 * 
 * @note Global ALL bit overrides specific mode bits if set
 * @see Terrain_lookup table
 * @see g.terrain_follow parameter (TERRAIN_FOLLOW)
 * @see terrain_bitmask enum
 */
bool Plane::terrain_enabled_in_mode(Mode::Number num) const
{
    // Global enable
    if ((g.terrain_follow.get() & int32_t(terrain_bitmask::ALL)) != 0) {
        return true;
    }

    // Specific enable
    for (const struct TerrainLookupTable entry : Terrain_lookup) {
        if (entry.mode_num == num) {
            if ((g.terrain_follow.get() & int32_t(entry.bitmask)) != 0) {
                return true;
            }
            break;
        }
    }

    return false;
}
#endif

#if AP_MAVLINK_MAV_CMD_SET_HAGL_ENABLED
/**
 * @brief Process MAVLink command to set external Height Above Ground Level
 * 
 * @details Handles MAV_CMD_SET_HAGL MAVLink command from external system
 *          providing Height Above Ground Level measurements. This allows
 *          external sensors or systems (e.g., sophisticated terrain mapping,
 *          external lidar, RTK positioning) to override onboard altitude
 *          estimates for improved accuracy.
 *          
 *          **Command Parameters:**
 *          - param1: HAGL in meters (stored directly)
 *          - param2: Accuracy (currently ignored)
 *          - param3: Timeout in seconds (converted to milliseconds)
 *          
 *          **Use Cases:**
 *          - High-precision external lidar systems
 *          - Ground-based terrain mapping providing real-time data
 *          - Companion computer processing advanced sensor fusion
 *          - RTK-enabled terrain following
 *          
 *          **Timeout Mechanism:**
 *          External HAGL data automatically expires after specified timeout,
 *          reverting to onboard sensors. This ensures stale data from failed
 *          external system doesn't compromise safety.
 * 
 * @param[in] packet MAVLink command_int message containing HAGL data
 * 
 * @note Accuracy parameter (param2) is currently ignored
 * @warning External system must send updates before timeout or data discarded
 * 
 * @see get_external_HAGL()
 * @see external_hagl structure
 */
void Plane::handle_external_hagl(const mavlink_command_int_t &packet)
{
    auto &hagl = plane.external_hagl;
    hagl.hagl = packet.param1;
    hagl.last_update_ms = AP_HAL::millis();
    hagl.timeout_ms = uint32_t(packet.param3 * 1000);
}

/**
 * @brief Retrieve external Height Above Ground Level if valid and current
 * 
 * @details Checks if external HAGL data is available and has not expired.
 *          External HAGL takes priority over onboard rangefinder and terrain
 *          database when available, providing highest accuracy altitude data
 *          from external sensing systems.
 *          
 *          **Data Validity:**
 *          - Must have been set at least once (last_update_ms != 0)
 *          - Must not exceed configured timeout period
 *          - Automatically marks data stale on timeout
 *          
 *          **Priority in altitude_relative_ground():**
 *          External HAGL checked first before rangefinder or terrain database,
 *          giving it highest priority for altitude determination.
 * 
 * @param[out] height_agl External HAGL value in meters if valid
 * 
 * @return true if valid external HAGL available, false if expired or not set
 * 
 * @note Resets last_update_ms to 0 on timeout to prevent repeated checks
 * @see handle_external_hagl()
 * @see external_hagl structure
 */
bool Plane::get_external_HAGL(float &height_agl)
{
    auto &hagl = plane.external_hagl;
    if (hagl.last_update_ms != 0) {
        const uint32_t now_ms = AP_HAL::millis();
        if (now_ms - hagl.last_update_ms <= hagl.timeout_ms) {
            height_agl = hagl.hagl;
            return true;
        }
        hagl.last_update_ms = 0;
    }
    return false;
}
#endif // AP_MAVLINK_MAV_CMD_SET_HAGL_ENABLED

/**
 * @brief Get height above landing target for landing control
 * 
 * @details Provides height measurement for landing operations, selecting best
 *          available altitude source with preference for direct ground-sensing
 *          systems over computed altitude.
 *          
 *          **Source Priority (highest to lowest):**
 *          1. **External HAGL** (MAVLink MAV_CMD_SET_HAGL)
 *             - Highest priority if available and not timed out
 *             - Disables terrain correction (external system's responsibility)
 *             - Considered a rangefinder-type source
 *          
 *          2. **Rangefinder** (if enabled and in range)
 *             - Applied as correction to height_above_target()
 *             - Only during LAND stage with TAKEOFF_LANDING enabled
 *             - Provides direct ground distance measurement
 *          
 *          3. **Barometric/Terrain** (fallback)
 *             - height_above_target() using waypoint altitude
 *             - Less accurate but always available
 *          
 *          **Rangefinder Active Flag:**
 *          Set true when using rangefinder or external HAGL, false for
 *          barometric. This flag allows landing controller to adjust behavior
 *          based on altitude source accuracy/characteristics.
 * 
 * @param[out] rangefinder_active Set true if using rangefinder/external HAGL
 * 
 * @return float Height above landing target in meters
 * 
 * @note External HAGL disables terrain correction - external system handles it
 * @warning Accurate landing height critical for safe flare and touchdown
 * 
 * @see get_external_HAGL()
 * @see height_above_target()
 * @see rangefinder_correction()
 */
float Plane::get_landing_height(bool &rangefinder_active)
{
    float height;

#if AP_MAVLINK_MAV_CMD_SET_HAGL_ENABLED
    // if external HAGL is active use that
    if (get_external_HAGL(height)) {
        // ensure no terrain correction is applied - that is the job
        // of the external system if it is wanted
        auto_state.terrain_correction = 0;

        // an external HAGL is considered to be a type of rangefinder
        rangefinder_active = true;
        return height;
    }
#endif

    // get basic height above target
    height = height_above_target();
    rangefinder_active = false;

#if AP_RANGEFINDER_ENABLED
    // possibly correct with rangefinder
    height -= rangefinder_correction();
    rangefinder_active = rangefinder_use(RangeFinderUse::TAKEOFF_LANDING) && rangefinder_state.in_range;
#endif

    return height;
}

/**
 * @brief Detect and repair malformed terrain waypoint locations
 * 
 * @details **Bug Detection and Workaround Function**
 *          
 *          Terrain altitude waypoints MUST have relative_alt flag set because
 *          terrain altitudes are inherently relative to terrain. If terrain_alt
 *          is true but relative_alt is false, this indicates a software bug.
 *          
 *          **The Bug:**
 *          When relative_alt flag is missing, code elsewhere may have incorrectly
 *          added home.alt to loc.alt, treating it as an absolute AMSL altitude.
 *          This would cause aircraft to fly at completely wrong altitude.
 *          
 *          **Repair Strategy:**
 *          1. Log internal error with source line number for debugging
 *          2. Subtract home.alt to undo the incorrect addition
 *          3. Set relative_alt flag to correct value
 *          4. Sanity check: Only subtract if result not excessively negative
 *          
 *          **Sanity Check:**
 *          If (loc.alt - home.alt) would be < -500cm (-5m), something else is
 *          wrong. Don't subtract in this case - better to be wrong than fly
 *          underground or have extreme altitude error.
 *          
 *          **Why This Function Exists:**
 *          Defensive programming to handle bugs in waypoint loading, mission
 *          parsing, or other code that manipulates Location objects. Better to
 *          detect and attempt repair than crash due to bad altitude.
 * 
 * @param[in,out] loc Location to check and repair if needed
 * @param[in] linenum Source line number for error reporting (__LINE__)
 * 
 * @note Logs AP_InternalError::error_t::flow_of_control when bug detected
 * @warning This is a bug workaround - root cause should be fixed when found
 * 
 * @see Location::terrain_alt
 * @see Location::relative_alt
 * @see AP_InternalError
 */
void Plane::fix_terrain_WP(Location &loc, uint32_t linenum)
{
    if (loc.terrain_alt && !loc.relative_alt) {
        AP::internalerror().error(AP_InternalError::error_t::flow_of_control, linenum);
        /*
          we definitely have a bug, now we need to guess what was
          really meant. The lack of the relative_alt flag notionally
          means that home.alt has been added to loc.alt, so remove it,
          but only if it doesn't lead to a negative terrain altitude
         */
        if (loc.alt - home.alt > -500) {
            loc.alt -= home.alt;
        }
        loc.relative_alt = true;
    }
}
