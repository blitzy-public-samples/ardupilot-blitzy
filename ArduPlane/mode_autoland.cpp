/**
 * @file mode_autoland.cpp
 * @brief AUTOLAND flight mode implementation for automated landing approach
 * 
 * @details This file implements the AUTOLAND flight mode for ArduPlane, which provides
 *          a fully automated landing sequence for fixed-wing aircraft. The mode uses
 *          the captured takeoff direction to determine the landing approach path.
 * 
 *          AUTOLAND operates in three sequential stages:
 *          1. CLIMB - Initial climb to minimum terrain clearance altitude (optional)
 *          2. LOITER - Loiter-to-altitude at base leg waypoint to align with landing path
 *          3. LANDING - Final approach and landing using AP_Landing library
 * 
 *          The mode requires that the aircraft's initial takeoff direction was captured
 *          during a previous flight phase (typically during takeoff). This direction is
 *          used to compute the landing approach path, ensuring the aircraft lands in
 *          the opposite direction from takeoff.
 * 
 *          Key features:
 *          - Automatic base leg positioning based on takeoff direction
 *          - Configurable final approach waypoint distance and altitude
 *          - Optional initial climb for terrain clearance
 *          - Integration with AP_Landing library for multiple landing types
 *          - Support for terrain following during approach
 * 
 *          Safety considerations:
 *          - Aircraft must already be flying to enter this mode
 *          - Takeoff direction must have been previously captured
 *          - Not available for quadplanes (fixed-wing only)
 *          - Requires valid home position
 * 
 * @note This mode is only compiled if MODE_AUTOLAND_ENABLED is defined
 * @warning This is a safety-critical flight mode - modifications must be thoroughly
 *          tested in SITL before hardware testing
 * 
 * @see AP_Landing library for landing sequence implementation
 * @see ModeAutoLand class definition in mode.h
 * 
 * Source: ArduPlane/mode_autoland.cpp
 */

#include "mode.h"
#include "Plane.h"
#include <GCS_MAVLink/GCS.h>

#if MODE_AUTOLAND_ENABLED

/**
 * @brief Extra altitude margin added to climb target to ensure altitude is exceeded
 * 
 * @details During the optional CLIMB stage, this altitude (in meters) is added to the
 *          computed terrain clearance altitude to account for steady-state tracking error.
 *          This ensures the aircraft actually reaches the desired minimum terrain clearance
 *          (terrain_alt_min parameter) before proceeding to the LOITER stage.
 * 
 *          Value must be larger than expected altitude tracking error to prevent premature
 *          transition to loiter while still climbing.
 * 
 * @note Units: meters
 * @note Default value: 10 meters
 */
constexpr float fast_climb_extra_alt = 10;

/**
 * @brief AUTOLAND mode parameter group definition
 * 
 * @details Defines configurable parameters for AUTOLAND mode that control landing approach
 *          geometry, direction capture, and terrain clearance behavior. These parameters
 *          are stored in persistent storage and configurable via ground station.
 * 
 *          Parameters defined:
 *          - WP_ALT: Final approach waypoint altitude above home (meters)
 *          - WP_DIST: Distance of final approach waypoint from home (meters)
 *          - DIR_OFF: Angular offset from takeoff direction for landing (degrees)
 *          - OPTIONS: Bitmask for optional behaviors (direction capture method)
 *          - CLIMB: Minimum altitude above terrain before turning (meters, 0 disables)
 * 
 *          Integration with AP_Param system:
 *          - Parameters accessible via ground station as AUTOLAND.WP_ALT, etc.
 *          - Values persist across power cycles
 *          - Parameter documentation uses @Param tags for auto-generation
 * 
 * @note Parameter defaults initialized in constructor via setup_object_defaults()
 * @see AP_Param documentation for parameter system details
 */
const AP_Param::GroupInfo ModeAutoLand::var_info[] = {
    // @Param: WP_ALT
    // @DisplayName: Final approach WP altitude
    // @Description: This is the target altitude above HOME for final approach waypoint
    // @Range: 0 200
    // @Increment: 1
    // @Units: m
    // @User: Standard
    AP_GROUPINFO("WP_ALT", 1, ModeAutoLand, final_wp_alt, 55),

    // @Param: WP_DIST
    // @DisplayName: Final approach WP distance
    // @Description: This is the distance from Home that the final approach waypoint is set. The waypoint point will be in the opposite direction of takeoff (the direction the plane is facing when the plane sets its takeoff heading)
    // @Range: 0 700
    // @Increment: 1
    // @Units: m
    // @User: Standard
    AP_GROUPINFO("WP_DIST", 2, ModeAutoLand, final_wp_dist, 400),

    // @Param: DIR_OFF
    // @DisplayName: Landing direction offset from takeoff
    // @Description: The captured takeoff direction after ground course is established in autotakeoffsis offset by this amount to create a different landing direction and approach.However,if TKOFF_OPTION bit1 is set, the takeoff(landing) direction is captured immediately via compass heading upon arming, then this offset is NOT applied.
    // @Range: -360 360
    // @Increment: 1
    // @Units: deg
    // @User: Standard
    AP_GROUPINFO("DIR_OFF", 3, ModeAutoLand, landing_dir_off, 0),

    // @Param: OPTIONS
    // @DisplayName: Autoland mode options
    // @Description: Enables optional autoland mode behaviors
    // @Bitmask: 0: When set if there is a healthy compass in use the compass heading will be captured at arming and used for the AUTOLAND mode's initial takeoff direction instead of capturing ground course in NAV_TAKEOFF or Mode TAKEOFF or other modes.
    // @User: Standard
    AP_GROUPINFO("OPTIONS", 4, ModeAutoLand, options, 0),

    // @Param: CLIMB
    // @DisplayName: Minimum altitude above terrain before turning upon entry
    // @Description: Vehicle will climb with limited turn ability (LEVEL_ROLL_LIMIT) until it is at least this altitude above the terrain at the point of entry, before proceeding to loiter-to-alt and landing legs. 0 Disables.
    // @Range: 0 100
    // @Increment: 1
    // @Units: m
    // @User: Standard
    AP_GROUPINFO("CLIMB", 5, ModeAutoLand, terrain_alt_min, 0),


    AP_GROUPEND
};

/**
 * @brief Constructor for ModeAutoLand flight mode
 * 
 * @details Initializes the AUTOLAND mode object and sets up parameter defaults from
 *          the var_info parameter table. Called once during vehicle initialization.
 * 
 * @note Calls AP_Param::setup_object_defaults() to initialize all AUTOLAND parameters
 *       (WP_ALT, WP_DIST, DIR_OFF, OPTIONS, CLIMB) to their default values
 * 
 * @see var_info[] for parameter definitions and defaults
 */
ModeAutoLand::ModeAutoLand() :
    Mode()
{
    AP_Param::setup_object_defaults(this, var_info);
}

/**
 * @brief Initialize and enter AUTOLAND flight mode
 * 
 * @details This method is called when transitioning into AUTOLAND mode. It performs
 *          comprehensive pre-flight checks and sets up the three-stage landing sequence.
 * 
 *          Entry requirements validated:
 *          - Aircraft must already be flying (prevents ground activation)
 *          - Quadplane check: AUTOLAND not supported for quadplanes
 *          - Takeoff direction must be initialized (captured during previous flight phase)
 * 
 *          Landing sequence initialization:
 *          
 *          For Deep Stall landing type (if enabled):
 *          - Creates single NAV_LAND command at home location
 *          - Sets altitude parameter for deep stall initiation
 *          - Transitions directly to LANDING stage
 * 
 *          For Glide Slope landing (standard):
 *          - Computes land_start waypoint: home offset by final_wp_dist in direction
 *            opposite to takeoff (180 degrees from takeoff heading)
 *          - Creates base leg loiter waypoint: positioned 90 degrees perpendicular to
 *            landing path at distance of corrected loiter radius
 *          - Determines turn direction based on current position relative to approach path
 *          - Optionally creates initial climb waypoint if terrain clearance needed
 *          - Sets up final NAV_LAND command at home location
 * 
 *          Coordinate frame handling:
 *          - Final approach altitude specified ABOVE_HOME or ABOVE_TERRAIN (if enabled)
 *          - All waypoints converted to ABSOLUTE frame for consistent navigation
 * 
 *          Integration with AP_Landing library:
 *          - Deep stall landings: AP_Landing manages entire approach from altitude
 *          - Glide slope landings: This mode handles approach, AP_Landing handles flare
 * 
 * @return true if mode entry successful and landing sequence initialized
 * @return false if pre-flight checks fail (not flying, no takeoff direction, etc.)
 * 
 * @note Sets plane.prev_WP_loc to current position for crosstrack navigation
 * @note Enables crosstrack navigation (plane.auto_state.next_wp_crosstrack = true)
 * @note Flight stage set to NORMAL initially, transitions to LAND during final approach
 * 
 * @warning Critical safety check: Returns false if aircraft not already airborne
 * @warning Quadplane compatibility: Always returns false for quadplanes
 * 
 * @see AutoLandStage enum for landing sequence stages
 * @see AP_Landing library for landing type implementations
 * @see plane.start_command() for waypoint initialization
 * 
 * Source: ArduPlane/mode_autoland.cpp:68-198
 */
bool ModeAutoLand::_enter()
{
    //must be flying to enter
    if (!plane.is_flying()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Must already be flying!");
        return false;
    }

    // autoland not available for quadplanes as capture of takeoff direction
    // doesn't make sense
#if HAL_QUADPLANE_ENABLED
    if (quadplane.available()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "autoland not available");
        return false;
    }
#endif

    if (!plane.takeoff_state.initial_direction.initialized) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Takeoff initial direction not set");
        return false;
    }

    plane.set_target_altitude_current();
    plane.auto_state.next_wp_crosstrack = true;

    plane.prev_WP_loc = plane.current_loc;

    // In flight stage normal for approach
    plane.set_flight_stage(AP_FixedWing::FlightStage::NORMAL);

    const Location &home = ahrs.get_home();

#ifndef HAL_LANDING_DEEPSTALL_ENABLED
    if (plane.landing.get_type() == AP_Landing::TYPE_DEEPSTALL) {
        // Deep stall landings require only a landing location, they do there own loiter to alt and approach
        cmd_land.id = MAV_CMD_NAV_LAND;
        cmd_land.content.location = home;

        // p1 gives the altitude from which to start the deep stall above the location alt
        cmd_land.p1 = final_wp_alt;
        plane.start_command(cmd_land);

        stage = AutoLandStage::LANDING;
        return true;
    }
#endif // HAL_LANDING_DEEPSTALL_ENABLED

    /*
      Glide slope landing is in 3 steps:
        1) a loitering to alt waypoint centered on base leg
        2) exiting and proceeeing to a final approach land start WP, with crosstrack
        3) the landing WP at home, with crosstrack

      the base leg point is 90 degrees off from the landing leg
     */

    /*
      first calculate the starting waypoint we will use when doing the
      NAV_LAND. This is offset by final_wp_dist from home, in a
      direction 180 degrees from takeoff direction
     */
    land_start = home;
    land_start.offset_bearing(plane.takeoff_state.initial_direction.heading, -final_wp_dist);
    land_start.set_alt_m(final_wp_alt, Location::AltFrame::ABOVE_HOME);
    land_start.change_alt_frame(Location::AltFrame::ABSOLUTE);

    /*
      now create the initial target waypoint for the loitering to alt centered on base leg waypoint. We
      choose if we will do a right or left turn onto the landing based
      on where we are when we enter the landing mode
     */
    const float bearing_to_current_deg = degrees(land_start.get_bearing(plane.current_loc));
    const float bearing_err_deg = wrap_180(plane.takeoff_state.initial_direction.heading - bearing_to_current_deg);
    const float bearing_offset_deg = (bearing_err_deg > 0) ? -90 : 90;

    // Try and minimize loiter radius by using the smaller of the waypoint loiter radius or 1/3 of the final WP distance
    const float loiter_radius = MIN(final_wp_dist * 0.333, abs(plane.aparm.loiter_radius));

    // corrected_loiter_radius is the radius the vehicle will actually fly, this gets larger as altitude increases.
    // Strictly this gets the loiter radius at the current altitude, really we want the loiter radius at final_wp_alt.
    const float corrected_loiter_radius = plane.nav_controller->loiter_radius(loiter_radius);

    cmd_loiter.id = MAV_CMD_NAV_LOITER_TO_ALT;
    cmd_loiter.p1 = loiter_radius;
    cmd_loiter.content.location = land_start;
    cmd_loiter.content.location.offset_bearing(plane.takeoff_state.initial_direction.heading + bearing_offset_deg, corrected_loiter_radius);
    cmd_loiter.content.location.loiter_ccw = bearing_err_deg>0? 1 :0;

    // May need to climb first
    bool climb_first = false;
    if (terrain_alt_min > 0) {
        // Work out the distance needed to climb above terrain
#if AP_TERRAIN_AVAILABLE
        const bool use_terrain = plane.terrain_enabled_in_current_mode();
#else
        const bool use_terrain = false;
#endif
        const float dist_to_climb = terrain_alt_min - plane.relative_ground_altitude(RangeFinderUse::CLIMB, use_terrain);
        if (is_positive(dist_to_climb)) {
            // Copy loiter and update target altitude to current altitude plus climb altitude
            cmd_climb = cmd_loiter;
            float abs_alt;
            if (plane.current_loc.get_alt_m(Location::AltFrame::ABSOLUTE, abs_alt)) {
                cmd_climb.content.location.set_alt_m(abs_alt + dist_to_climb + fast_climb_extra_alt, Location::AltFrame::ABSOLUTE);
                climb_first = true;
            }
        }
    }

#if AP_TERRAIN_AVAILABLE
    // Update loiter location to be relative terrain if enabled
    if (plane.terrain_enabled_in_current_mode()) {
        cmd_loiter.content.location.set_alt_m(final_wp_alt, Location::AltFrame::ABOVE_TERRAIN);
    };
#endif
    // land WP at home
    cmd_land.id = MAV_CMD_NAV_LAND;
    cmd_land.content.location = home;

    // start first leg toward the base leg loiter to alt point
    if (climb_first) {
        stage = AutoLandStage::CLIMB;
        plane.start_command(cmd_climb);

    } else {
        stage = AutoLandStage::LOITER;
        plane.start_command(cmd_loiter);
    }

    return true;
}

/**
 * @brief Main control update for AUTOLAND mode (called at main loop rate)
 * 
 * @details This method computes control surface outputs and throttle for the current
 *          landing stage. Called at the scheduler's main loop rate (typically 50Hz).
 * 
 *          Control sequence:
 *          1. Calculate navigation roll command (plane.calc_nav_roll())
 *          2. Apply roll limiting for CLIMB stage
 *          3. Calculate pitch command (plane.calc_nav_pitch())
 *          4. Calculate throttle output or suppress if landing complete
 * 
 *          Roll limiting in CLIMB stage:
 *          - Restricts roll angle to LEVEL_ROLL_LIMIT (typically 5-15 degrees)
 *          - Prevents aggressive maneuvering during low-altitude terrain clearance
 *          - Navigation continues but with reduced turn capability
 *          - Ensures safe climb away from terrain before proceeding to approach
 * 
 *          Throttle management:
 *          - Normal throttle calculation during approach phases
 *          - Throttle suppressed to zero when landing is complete (as determined
 *            by AP_Landing library's is_throttle_suppressed() method)
 *          - Throttle suppression prevents motor restart during ground roll
 * 
 *          Integration with control systems:
 *          - calc_nav_roll(): Computes desired bank angle based on navigation target
 *          - calc_nav_pitch(): Computes desired pitch angle (TECS or direct)
 *          - calc_throttle(): Computes throttle output (TECS for airspeed control)
 * 
 * @note This method does NOT perform navigation logic - see navigate() method
 * @note Called at main loop rate regardless of landing stage
 * @note Roll limiting only active during CLIMB stage for safety
 * 
 * @warning Roll limit must be respected during CLIMB to ensure terrain clearance
 * @warning Throttle suppression after landing is critical for ground safety
 * 
 * @see navigate() for waypoint sequencing and stage transitions
 * @see plane.calc_nav_roll() in Attitude.cpp
 * @see plane.calc_throttle() in AP_Plane/mode.cpp
 * @see AP_Landing::is_throttle_suppressed() for landing completion detection
 * 
 * Source: ArduPlane/mode_autoland.cpp:200-217
 */
void ModeAutoLand::update()
{
    plane.calc_nav_roll();

    // Apply level roll limit in climb stage
    if (stage == AutoLandStage::CLIMB) {
        plane.roll_limit_cd = MIN(plane.roll_limit_cd, plane.g.level_roll_limit*100);
        plane.nav_roll_cd = constrain_int16(plane.nav_roll_cd, -plane.roll_limit_cd, plane.roll_limit_cd);
    }

    plane.calc_nav_pitch();
    if (plane.landing.is_throttle_suppressed()) {
        // if landing is considered complete throttle is never allowed, regardless of landing type
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0.0);
    } else {
        plane.calc_throttle();
    }
}

/**
 * @brief Navigation logic and stage management for AUTOLAND landing sequence
 * 
 * @details This method implements the state machine for the three-stage automated landing
 *          approach. Called periodically by the scheduler to check stage completion
 *          conditions and transition to the next stage when appropriate.
 * 
 *          Landing Stage State Machine:
 * 
 *          CLIMB Stage (optional - only if terrain_alt_min > 0):
 *          - Updates loiter navigation around climb waypoint
 *          - Applies roll limiting (see update() method) for safe terrain clearance
 *          - Monitors altitude above terrain relative to target climb altitude
 *          - Transitions to LOITER when:
 *            a) Loiter target reached (position within waypoint radius), OR
 *            b) Altitude above terrain exceeds climb target (within fast_climb_extra_alt)
 *          - On transition: Sets next_wp_crosstrack, starts LOITER command
 * 
 *          LOITER Stage:
 *          - Executes NAV_LOITER_TO_ALT at base leg waypoint
 *          - Circles at configured radius until reaching final approach altitude
 *          - Monitors heading alignment with landing path
 *          - Transitions to LANDING when verify_loiter_to_alt() returns true:
 *            a) Target altitude reached, AND
 *            b) Heading aligned with landing direction (via landing_lined_up())
 *          - On transition: Starts LAND command, sets prev_WP_loc for crosstrack
 * 
 *          LANDING Stage:
 *          - Sets flight stage to LAND (triggers AP_Landing library control)
 *          - Executes NAV_LAND command from land_start waypoint to home
 *          - Enables crosstrack navigation for precise approach path following
 *          - AP_Landing library handles final approach, flare, and touchdown
 *          - Continues until landing is complete (AP_Landing determines completion)
 * 
 *          AP_Landing Library Integration:
 *          During LANDING stage, this method calls:
 *          - plane.set_flight_stage(LAND): Activates landing-specific control modes
 *          - plane.verify_command(cmd_land): Allows AP_Landing to manage landing phases
 *          - AP_Landing executes: approach descent → pre-flare → flare → touchdown
 * 
 *          Navigation features:
 *          - Crosstrack enabled throughout for precise path following
 *          - prev_WP_loc updated at stage transitions to establish crosstrack baseline
 *          - Loiter radius corrected for altitude (banking effects)
 * 
 * @note Called by scheduler at navigation update rate (typically 10-25Hz)
 * @note Stage transitions are irreversible - no returning to previous stages
 * @note CLIMB stage skipped if terrain_alt_min parameter is zero
 * 
 * @warning Flight stage must be set to LAND before AP_Landing takes control
 * @warning Crosstrack must be enabled for safe landing approach path following
 * 
 * @see AutoLandStage enum for stage definitions
 * @see plane.verify_loiter_to_alt() for loiter completion detection
 * @see landing_lined_up() for heading alignment check
 * @see AP_Landing for final landing phase control
 * @see plane.update_loiter() for loiter navigation control
 * 
 * Source: ArduPlane/mode_autoland.cpp:219-254
 */
void ModeAutoLand::navigate()
{
    switch (stage) {
    case AutoLandStage::CLIMB:
        // Update loiter, although roll limit is applied the vehicle will still navigate (slowly)
        plane.update_loiter(cmd_climb.p1);

        ftype dist;
        if (plane.reached_loiter_target() || !cmd_climb.content.location.get_height_above(plane.current_loc, dist) || (dist < fast_climb_extra_alt)) {
            // Reached destination or Climb is done, move onto loiter
            plane.auto_state.next_wp_crosstrack = true;
            stage = AutoLandStage::LOITER;
            plane.start_command(cmd_loiter);
            plane.prev_WP_loc = plane.current_loc;
        }
        break;

    case AutoLandStage::LOITER:
        // check if we have arrived and completed loiter at base leg waypoint
        if (plane.verify_loiter_to_alt(cmd_loiter)) {
            stage = AutoLandStage::LANDING;
            plane.start_command(cmd_land);
            // Crosstrack from the land start location
            plane.prev_WP_loc = land_start;

        }
        break;

    case AutoLandStage::LANDING:
        plane.set_flight_stage(AP_FixedWing::FlightStage::LAND);
        plane.verify_command(cmd_land);
        // make sure we line up
        plane.auto_state.crosstrack = true;
        break;
    }
}

/**
 * @brief Monitor and capture takeoff direction for AUTOLAND mode
 * 
 * @details This method is called periodically during flight to capture the aircraft's
 *          initial takeoff direction, which is later used by AUTOLAND mode to determine
 *          the landing approach path. The takeoff direction is captured only once per
 *          flight and remains constant until the next arming cycle.
 * 
 *          Capture conditions (all must be satisfied):
 *          - Current flight mode allows autoland direction capture
 *          - Aircraft is flying (is_flying() returns true)
 *          - System is armed
 *          - GPS ground speed exceeds GPS_GND_CRS_MIN_SPD threshold
 *          - Direction not already captured
 * 
 *          Direction calculation:
 *          - Uses GPS ground course (actual over-ground track)
 *          - Applies landing_dir_off parameter offset (AUTOLAND.DIR_OFF)
 *          - Result stored in plane.takeoff_state.initial_direction.heading
 * 
 *          Alternative capture method:
 *          - If AUTOLAND.OPTIONS bit 0 is set: Direction captured from compass heading
 *            at arming time via arm_check() method instead of GPS ground course
 * 
 * @note Quadplane check: Returns immediately without capture for quadplanes
 *       (AUTOLAND mode not supported for quadplanes)
 * @note Direction captured only once per flight - subsequent calls return early
 * @note Captured direction persists across mode changes within same flight
 * 
 * @warning GPS_GND_CRS_MIN_SPD threshold prevents capture during low-speed ground roll
 *          where GPS course may be unreliable
 * 
 * @see set_autoland_direction() for direction storage and GCS notification
 * @see arm_check() for alternative compass-based capture at arming
 * @see Mode::allows_autoland_direction_capture() for mode compatibility check
 * 
 * Source: ArduPlane/mode_autoland.cpp:261-280
 */
void ModeAutoLand::check_takeoff_direction()
{
#if HAL_QUADPLANE_ENABLED
    // we don't allow fixed wing autoland for quadplanes
    if (quadplane.available()) {
        return;
    }
#endif

    if (plane.takeoff_state.initial_direction.initialized) {
        return;
    }
    //set autoland direction to GPS course over ground
    if (plane.control_mode->allows_autoland_direction_capture() &&
        plane.is_flying() &&
        hal.util->get_soft_armed() &&
        plane.gps.ground_speed() > GPS_GND_CRS_MIN_SPD) {
        set_autoland_direction(plane.gps.ground_course() + landing_dir_off);
    }
}

/**
 * @brief Set the AUTOLAND landing direction heading
 * 
 * @details Stores the provided heading as the landing direction for AUTOLAND mode and
 *          marks it as initialized. Sends GCS notification with the captured direction.
 *          
 *          The landing direction is used by _enter() to compute:
 *          - Final approach waypoint location (180 degrees opposite)
 *          - Base leg loiter waypoint location (90 degrees perpendicular)
 * 
 *          Once set, this direction persists for the entire flight and determines
 *          the landing approach geometry when AUTOLAND mode is entered.
 * 
 * @param[in] heading Landing direction heading in degrees (0-360)
 *                    Typically GPS ground course + landing_dir_off parameter
 * 
 * @note Heading is wrapped to 0-360 degree range via wrap_360()
 * @note Sets plane.takeoff_state.initial_direction.initialized flag to true
 * @note Sends MAV_SEVERITY_INFO message to ground station with captured direction
 * 
 * @see check_takeoff_direction() which calls this method with GPS course
 * @see arm_check() which may call this method with compass heading
 * @see _enter() which uses stored heading to compute landing approach waypoints
 * 
 * Source: ArduPlane/mode_autoland.cpp:283-288
 */
void ModeAutoLand::set_autoland_direction(const float heading)
{
    plane.takeoff_state.initial_direction.heading = wrap_360(heading);
    plane.takeoff_state.initial_direction.initialized = true;
    gcs().send_text(MAV_SEVERITY_INFO, "Autoland direction= %u",int(plane.takeoff_state.initial_direction.heading));
}

/**
 * @brief Check if aircraft heading is aligned for landing approach
 * 
 * @details Determines whether the aircraft has achieved the correct heading during the
 *          LOITER stage to begin the final landing approach. This method is called by
 *          verify_loiter_to_alt() to determine if both altitude AND heading requirements
 *          are met before transitioning to the LANDING stage.
 * 
 *          Alignment check:
 *          - Evaluates heading relative to the line from base leg loiter center
 *            (cmd_loiter.content.location) to landing start waypoint (land_start)
 *          - Uses mode_loiter.isHeadingLinedUp() for alignment determination
 *          - Ensures aircraft exits loiter on correct heading for landing path
 * 
 *          Purpose:
 *          - Prevents premature transition to landing with incorrect heading
 *          - Ensures smooth transition from loiter to final approach
 *          - Critical for maintaining landing path alignment with crosstrack
 * 
 * @return true if aircraft heading is aligned with landing approach path
 * @return false if aircraft still needs to continue loitering to achieve alignment
 * 
 * @note Called from commands_logic verify_loiter_to_alt during LOITER stage
 * @note Both altitude target AND heading alignment must be satisfied to exit loiter
 * 
 * @see navigate() LOITER stage for usage context
 * @see ModeLoiter::isHeadingLinedUp() for alignment algorithm
 * @see plane.verify_loiter_to_alt() which combines altitude and heading checks
 * 
 * Source: ArduPlane/mode_autoland.cpp:293-298
 */
bool ModeAutoLand::landing_lined_up(void)
{
    // use the line between the center of the LOITER_TO_ALT on the base leg and the
    // start of the landing leg (land_start_WP)
    return plane.mode_loiter.isHeadingLinedUp(cmd_loiter.content.location, cmd_land.content.location);
}

/**
 * @brief Capture landing direction from compass at arming time (if option enabled)
 * 
 * @details This method is called during the arming sequence to optionally capture the
 *          landing direction from the aircraft's current compass heading rather than
 *          waiting for GPS ground course capture during flight.
 * 
 *          Activation requirements:
 *          - AUTOLAND.OPTIONS parameter bit 0 must be set (AUTOLAND_DIR_ON_ARM)
 *          - Valid compass must be available (plane.ahrs.use_compass() returns true)
 * 
 *          If both conditions met:
 *          - Captures current compass heading (plane.ahrs.get_yaw_deg())
 *          - Stores as landing direction via set_autoland_direction()
 *          - Prevents need for ground course capture during flight
 * 
 *          Use cases:
 *          - Hand launch or catapult launch where aircraft doesn't establish ground course
 *          - Launch from slopes or elevated positions
 *          - Short flights where ground course capture may not occur
 *          - Situations where immediate landing direction capture is preferred
 * 
 *          Alternative to GPS capture:
 *          - Normal method: GPS ground course capture via check_takeoff_direction()
 *          - This method: Immediate compass heading capture at arming
 *          - Only one method used per flight (compass capture takes precedence if enabled)
 * 
 * @note Called from arming sequence before takeoff
 * @note AUTOLAND.DIR_OFF offset parameter still applied to captured heading
 * @note If this method captures heading, check_takeoff_direction() will not recapture
 * 
 * @warning Requires accurate compass calibration for correct landing direction
 * @warning Aircraft must be pointing in desired landing direction at arm time
 * 
 * @see set_autoland_direction() for heading storage
 * @see check_takeoff_direction() for alternative GPS-based capture method
 * @see AutoLandOption::AUTOLAND_DIR_ON_ARM bitmask definition
 * 
 * Source: ArduPlane/mode_autoland.cpp:301-306
 */
void ModeAutoLand::arm_check(void)
{
    if (plane.ahrs.use_compass() && autoland_option_is_set(ModeAutoLand::AutoLandOption::AUTOLAND_DIR_ON_ARM)) {
        set_autoland_direction(plane.ahrs.get_yaw_deg());
    }
}

/**
 * @brief Check if aircraft is in final landing phase
 * 
 * @details Returns whether the aircraft is currently executing the final landing phase
 *          with AP_Landing library control active. This is used by other systems to
 *          determine if landing-specific behaviors should be applied.
 * 
 *          Returns true when:
 *          - AUTOLAND mode is in LANDING stage (after CLIMB and LOITER complete)
 *          - Flight stage is set to AP_FixedWing::FlightStage::LAND
 *          - AP_Landing library is actively controlling approach, flare, and touchdown
 * 
 *          Applications:
 *          - Throttle management: Determines if landing throttle limits apply
 *          - Control surface mixing: Landing-specific control allocation
 *          - Telemetry: Ground station landing status indication
 *          - Logging: Landing event marking and analysis
 * 
 * @return true if aircraft is in final landing phase (LAND flight stage)
 * @return false if still in CLIMB or LOITER stages, or not in AUTOLAND mode
 * 
 * @note Does not indicate landing completion - only that landing is in progress
 * @note Returns false during CLIMB and LOITER stages even though AUTOLAND is active
 * 
 * @see navigate() LANDING stage where flight_stage is set to LAND
 * @see AP_FixedWing::FlightStage enum for flight stage definitions
 * @see AP_Landing library for landing phase control when this returns true
 * 
 * Source: ArduPlane/mode_autoland.cpp:308-311
 */
bool ModeAutoLand::is_landing() const
{
    return (plane.flight_stage == AP_FixedWing::FlightStage::LAND);
}


#endif // MODE_AUTOLAND_ENABLED

