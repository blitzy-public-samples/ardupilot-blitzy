/**
 * @file mode_loiter.cpp
 * @brief LOITER flight mode implementation for fixed-wing aircraft
 * 
 * @details LOITER mode commands the aircraft to circle at its current location
 *          (or a specified location) at a configured radius. The aircraft maintains
 *          altitude while circling and can be controlled via RC input if stick mixing
 *          is enabled with ENABLE_LOITER_ALT_CONTROL flight option.
 *          
 *          Key behaviors:
 *          - Establishes circular flight path at current position on mode entry
 *          - Uses L1 navigation controller for smooth circular tracking
 *          - Supports altitude control via stick input when configured
 *          - Integrates with terrain following if enabled
 *          - Provides intelligent exit heading alignment for transitions
 *          
 *          The mode implements sophisticated heading alignment logic to ensure
 *          smooth transitions when exiting the loiter circle toward a new waypoint,
 *          with expanding tolerance based on loiter time to handle high-wind conditions.
 * 
 * @note This mode is typically used for holding patterns, waiting for clearance,
 *       or as a fallback when mission items complete.
 * 
 * @warning Ensure adequate loiter radius (WP_LOITER_RAD) for aircraft turning
 *          performance to avoid stall conditions.
 * 
 * Source: ArduPlane/mode_loiter.cpp
 */

#include "mode.h"
#include "Plane.h"

/**
 * @brief Initialize LOITER mode by establishing loiter point and parameters
 * 
 * @details This method is called when entering LOITER mode. It performs the following
 *          initialization sequence:
 *          1. Establishes loiter center at current aircraft location
 *          2. Sets up loiter radius from WP_LOITER_RAD parameter (via do_loiter_at_location)
 *          3. Configures terrain-relative altitude target if terrain following enabled
 *          4. Initializes altitude target for stick mixing control if configured
 *          5. Resets loiter angle tracking for fresh heading alignment calculations
 *          
 *          If ENABLE_LOITER_ALT_CONTROL flight option is active and stick mixing is
 *          enabled, the mode allows pilot to control altitude via throttle stick
 *          similar to FBWB mode behavior.
 * 
 * @return true - Mode entry always succeeds for LOITER
 * 
 * @note The loiter center becomes the current aircraft position, not a pre-defined waypoint
 * @note Loiter direction (clockwise/counter-clockwise) is determined by sign of WP_LOITER_RAD
 * 
 * @see Plane::do_loiter_at_location()
 * @see Plane::setup_terrain_target_alt()
 * @see FlightOptions::ENABLE_LOITER_ALT_CONTROL
 */
bool ModeLoiter::_enter()
{
    plane.do_loiter_at_location();
    plane.setup_terrain_target_alt(plane.next_WP_loc);

    // make sure the local target altitude is the same as the nav target used for loiter nav
    // this allows us to do FBWB style stick control
    /*IGNORE_RETURN(plane.next_WP_loc.get_alt_cm(Location::AltFrame::ABSOLUTE, plane.target_altitude.amsl_cm));*/
    if (plane.stick_mixing_enabled() && (plane.flight_option_enabled(FlightOptions::ENABLE_LOITER_ALT_CONTROL))) {
        plane.set_target_altitude_current();
    }

    plane.loiter_angle_reset();

    return true;
}

/**
 * @brief Execute LOITER mode control calculations for attitude and throttle
 * 
 * @details This method is called at the main loop rate (typically 50-400Hz depending on
 *          scheduler configuration) to calculate desired control outputs for maintaining
 *          the loiter circle.
 *          
 *          Control calculation sequence:
 *          1. Calculate navigation roll command to track circular path (always executed)
 *          2. Branch based on altitude control mode:
 *             a. If ENABLE_LOITER_ALT_CONTROL active: Use FBWB-style speed/height control
 *                allowing pilot stick input for altitude and airspeed control
 *             b. Otherwise: Use standard navigation pitch and throttle calculations to
 *                maintain target altitude automatically
 *          
 *          When scripting is active (AP_SCRIPTING_ENABLED), altitude target is reset
 *          to current altitude each loop to prevent navigation drift during aerobatic
 *          tricks performed within the loiter area.
 * 
 * @note Called at main loop rate - keep computations efficient
 * @note Roll calculation always uses L1 controller for smooth circular tracking
 * 
 * @see Plane::calc_nav_roll() - L1 controller roll command
 * @see Plane::update_fbwb_speed_height() - FBWB-style altitude control
 * @see Plane::calc_nav_pitch() - Navigation pitch controller
 * @see Plane::calc_throttle() - Throttle controller (TECS)
 * @see FlightOptions::ENABLE_LOITER_ALT_CONTROL
 */
void ModeLoiter::update()
{
    plane.calc_nav_roll();
    if (plane.stick_mixing_enabled() && plane.flight_option_enabled(FlightOptions::ENABLE_LOITER_ALT_CONTROL)) {
        plane.update_fbwb_speed_height();
    } else {
        plane.calc_nav_pitch();
        plane.calc_throttle();
    }

#if AP_SCRIPTING_ENABLED
    if (plane.nav_scripting_active()) {
        // while a trick is running we reset altitude
        plane.set_target_altitude_current();
        plane.next_WP_loc.set_alt_cm(plane.target_altitude.amsl_cm, Location::AltFrame::ABSOLUTE);
    }
#endif
}

/**
 * @brief Check if aircraft heading is aligned for exit from loiter toward target location
 * 
 * @details Determines if the aircraft's current heading is sufficiently aligned to exit
 *          the loiter circle and proceed toward a target waypoint. This method implements
 *          sophisticated geometry to handle both targets outside and inside the loiter radius.
 *          
 *          Algorithm:
 *          1. Project aircraft position onto closest point of loiter circle to remove
 *             radial position error from the nav controller's circle tracking
 *          2. Calculate target position relative to loiter center
 *          3. Determine ideal exit bearing based on target location:
 *             - Target outside radius: Direct heading from projected position to target
 *             - Target inside radius: Tangent exit angle accounting for chord geometry
 *          4. Compare current heading tangent to ideal bearing with expanding tolerance
 *          
 *          Tolerance starts at ±10 degrees and increases by 10 degrees for each complete
 *          loiter circle to ensure exit in high-wind conditions (handles up to 200 deg/sec yaw).
 * 
 * @param[in] loiterCenterLoc Center point of the loiter circle (Location)
 * @param[in] targetLoc       Destination waypoint location to navigate toward
 * 
 * @return true if heading is aligned within tolerance for loiter exit, false otherwise
 * 
 * @note For targets coincident with loiter center, always returns true (no preferred heading)
 * @note Uses corrected loiter radius accounting for altitude (from L1 controller)
 * @note Updates next_WP_loc to current position if loiter_xtrack flag is set for tangent exit
 * 
 * @warning Zero or negative loiter radius will cause immediate return true (safety fallback)
 * 
 * @see isHeadingLinedUp_cd() - Final bearing comparison with expanding tolerance
 */
bool ModeLoiter::isHeadingLinedUp(const Location loiterCenterLoc, const Location targetLoc)
{
    // Return true if current heading is aligned to vector to targetLoc.
    // Tolerance is initially 10 degrees and grows at 10 degrees for each loiter circle completed.

    // Corrected radius for altitude
    const float loiter_radius = plane.nav_controller->loiter_radius(fabsf(plane.loiter.radius));
    if (!is_positive(loiter_radius)) {
        // Zero is invalid, protect against divide by zero for destination inside loiter radius case
        return true;
    }

    // Calculate relative position of the vehicle relative to loiter center projected onto the closest point of the loiter circle
    // This removes error due to radial position as the nav controller attempts to track the circle
    const Vector2f projected_pos = loiterCenterLoc.get_distance_NE(plane.current_loc).normalized() * loiter_radius;

    // Target position relative to loiter center
    const Vector2f target_pos = loiterCenterLoc.get_distance_NE(targetLoc);

    // Distance between loiter circle and target
    const float target_dist = target_pos.length();
    if (!is_positive(target_dist)) {
        // Target is coincident with loiter center, no heading will be closer than any other
        return true;
    }

    // Target bearing in centi-degrees
    int32_t target_bearing_cd;

    if (target_dist >= loiter_radius) {
        // Destination outside loiter radius, heading will always line up with destination

        // Vector from between projected vehicle position and target position
        const Vector2f pos_to_target = target_pos - projected_pos;
        target_bearing_cd = degrees(pos_to_target.angle()) * 100;

    } else {
        // Destination is inside loiter, heading will never line up with destination

        // Advance turn point by the angle of a segment with chord "a"
        // This results in turning earlier as the target point approaches the center
        // If target is on radius angle of 0 and angle of 60 deg if target is on center 
        const float a = loiter_radius - target_dist;
        const float segment_angle = 2.0 * asinf(a / (2.0 * loiter_radius));

        // Pick the intersection point that will be hit first for the current loiter direction, add 90 deg to get the tangent angle
        target_bearing_cd = degrees(wrap_PI(target_pos.angle() + (M_PI_2 - segment_angle) * plane.loiter.direction)) * 100;

    }

    // Ideal heading in centi-degrees, +- 90 to get tangent to loiter circle at closest point
    const int32_t current_heading_cd = degrees(wrap_PI(projected_pos.angle() + M_PI_2 * plane.loiter.direction)) * 100;

    return isHeadingLinedUp_cd(target_bearing_cd, current_heading_cd);
}


/**
 * @brief Check if current aircraft heading is aligned to a target bearing
 * 
 * @details Convenience overload that retrieves current aircraft heading from AHRS
 *          groundspeed vector and compares it to the desired bearing. The heading
 *          is calculated from the ground track direction (velocity vector angle).
 * 
 * @param[in] bearing_cd Target bearing in centidegrees (0-35999, where 0=North, 9000=East)
 * 
 * @return true if current heading is within expanding tolerance of target bearing
 * 
 * @note Uses groundspeed vector for heading, which represents actual ground track
 *       rather than nose pointing direction (affected by wind)
 * 
 * @see isHeadingLinedUp_cd(int32_t, int32_t) - Core comparison logic
 */
bool ModeLoiter::isHeadingLinedUp_cd(const int32_t bearing_cd) {

    // get current heading.
    const int32_t heading_cd = (wrap_360(degrees(ahrs.groundspeed_vector().angle())))*100;

    return isHeadingLinedUp_cd(bearing_cd, heading_cd);
}


/**
 * @brief Check if aircraft heading is aligned to target bearing with expanding tolerance
 * 
 * @details Core heading alignment check with tolerance that expands based on number of
 *          loiter circles completed. This expanding tolerance ensures the aircraft can
 *          exit the loiter even in high-wind conditions that may force yaw rates beyond
 *          200 deg/sec when crossing the desired exit course.
 *          
 *          Tolerance calculation:
 *          - Base tolerance: ±10 degrees (±1000 centidegrees)
 *          - Expanded tolerance: +10 degrees per complete circle (360 degrees)
 *          - Total tolerance = 1000 + (1000 * complete_circles)
 *          
 *          The expansion uses integer division of accumulated loiter angle to create
 *          discrete tolerance steps, preventing oscillation at threshold boundaries.
 *          
 *          When heading is aligned and loiter_xtrack flag is set, updates next_WP_loc
 *          to current aircraft position to enable tangent exit from the loiter circle
 *          rather than navigating back through the center point.
 * 
 * @param[in] bearing_cd Target bearing in centidegrees (0-35999)
 * @param[in] heading_cd Current aircraft heading in centidegrees (0-35999)
 * 
 * @return true if heading error is within (base + expanded) tolerance, false otherwise
 * 
 * @note Tolerance expands by 1000 centidegrees (10 degrees) for every 36000 centidegrees
 *       (360 degrees) of accumulated loiter angle in plane.loiter.sum_cd
 * @note Uses wrap_180_cd() to handle angular discontinuity at North (0/360 degrees)
 * 
 * @see plane.loiter.sum_cd - Accumulated loiter angle for tolerance expansion
 * @see plane.next_WP_loc.loiter_xtrack - Flag controlling tangent vs center-point exit
 */
bool ModeLoiter::isHeadingLinedUp_cd(const int32_t bearing_cd, const int32_t heading_cd)
{
    // Return true if current heading is aligned to bearing_cd.
    // Tolerance is initially 10 degrees and grows at 10 degrees for each loiter circle completed.
    const int32_t heading_err_cd = wrap_180_cd(bearing_cd - heading_cd);

    /*
      Check to see if the the plane is heading toward the land
      waypoint. We use 20 degrees (+/-10 deg) of margin so that
      we can handle 200 degrees/second of yaw.

      After every full circle, extend acceptance criteria to ensure
      aircraft will not loop forever in case high winds are forcing
      it beyond 200 deg/sec when passing the desired exit course
    */

    // Use integer division to get discrete steps
    const int32_t expanded_acceptance = 1000 * (labs(plane.loiter.sum_cd) / 36000);

    if (labs(heading_err_cd) <= 1000 + expanded_acceptance) {
        // Want to head in a straight line from _here_ to the next waypoint instead of center of loiter wp

        // 0 to xtrack from center of waypoint, 1 to xtrack from tangent exit location
        if (plane.next_WP_loc.loiter_xtrack) {
            plane.next_WP_loc = plane.current_loc;
        }
        return true;
    }
    return false;
}

/**
 * @brief Update navigation commands for loiter circle tracking
 * 
 * @details Called by the navigation controller to update loiter parameters and execute
 *          the L1 controller for circular path tracking. This method runs at navigation
 *          update rate (typically 10-50Hz) to command the guidance algorithms.
 *          
 *          Execution sequence:
 *          1. If ENABLE_LOITER_ALT_CONTROL active: Synchronize loiter waypoint altitude
 *             with current target altitude (which may be adjusted by pilot stick input)
 *          2. Skip navigation updates if scripting is actively running aerobatic tricks
 *          3. Command L1 controller to track loiter circle using WP_LOITER_RAD parameter
 *          
 *          The update_loiter(0) call with radius=0 indicates to use the WP_LOITER_RAD
 *          parameter value rather than a mission-specified radius.
 * 
 * @note Called at navigation update rate (lower than main loop rate for efficiency)
 * @note When scripting tricks are active, navigation is suspended to avoid interference
 * 
 * @see Plane::update_loiter() - L1 controller loiter circle tracking
 * @see FlightOptions::ENABLE_LOITER_ALT_CONTROL
 * @see plane.nav_scripting_active() - Checks if aerobatic scripting is running
 */
void ModeLoiter::navigate()
{
    if (plane.flight_option_enabled(FlightOptions::ENABLE_LOITER_ALT_CONTROL)) {
        // update the WP alt from the global target adjusted by update_fbwb_speed_height
        plane.next_WP_loc.set_alt_cm(plane.target_altitude.amsl_cm, Location::AltFrame::ABSOLUTE);
    }

#if AP_SCRIPTING_ENABLED
    if (plane.nav_scripting_active()) {
        // don't try to navigate while running trick
        return;
    }
#endif

    // Zero indicates to use WP_LOITER_RAD
    plane.update_loiter(0);
}

/**
 * @brief Update target altitude for loiter mode navigation
 * 
 * @details Conditionally updates the target altitude based on altitude control mode.
 *          If ENABLE_LOITER_ALT_CONTROL flight option is active with stick mixing,
 *          altitude updates are handled by the FBWB-style controller (update_fbwb_speed_height)
 *          and this method returns early without applying standard navigation altitude updates.
 *          
 *          Otherwise, delegates to base Mode class implementation which handles standard
 *          altitude target tracking including terrain following, glide slope, and other
 *          altitude management features.
 * 
 * @note Early return prevents conflict between FBWB altitude control and navigation altitude control
 * @note When not using ENABLE_LOITER_ALT_CONTROL, uses standard Mode altitude target logic
 * 
 * @see Mode::update_target_altitude() - Base class altitude target management
 * @see FlightOptions::ENABLE_LOITER_ALT_CONTROL
 * @see Plane::update_fbwb_speed_height() - Alternative altitude control path
 */
void ModeLoiter::update_target_altitude()
{
    if (plane.stick_mixing_enabled() && (plane.flight_option_enabled(FlightOptions::ENABLE_LOITER_ALT_CONTROL))) {
        return;
    }
    Mode::update_target_altitude();
}
