/**
 * @file mode_thermal.cpp
 * @brief Implementation of THERMAL flight mode for autonomous soaring
 * 
 * @details This file implements the THERMAL flight mode which enables autonomous
 *          thermal soaring for fixed-wing aircraft. The mode allows the aircraft to:
 *          - Detect and exploit atmospheric thermals (rising air masses) for unpowered flight
 *          - Automatically center within thermals to maximize altitude gain
 *          - Manage thermal exit criteria and return to previous flight mode
 *          - Enforce safety limits (altitude, drift, radius)
 *          
 *          The THERMAL mode integrates with the AP_Soaring library which handles:
 *          - Thermal strength estimation using variometer data
 *          - Optimal loiter radius calculation based on thermal characteristics
 *          - Drift monitoring to maintain position relative to mission path
 *          - McCready speed calculations for optimal cross-country soaring
 *          
 *          Mode Operation:
 *          - Entered automatically from AUTO, FBWB, or CRUISE when thermal detected
 *          - Aircraft performs circular loiter pattern centered on thermal
 *          - Continuously adjusts loiter center based on thermal position estimates
 *          - Exits when thermal weakens, altitude limits reached, or drift exceeded
 *          - Returns to previous flight mode with appropriate heading alignment
 *          
 *          Coordinate System:
 *          - Uses NED (North-East-Down) frame relative to home position
 *          - Position vectors in centimeters, converted to meters for soaring library
 *          
 * @note This mode requires HAL_SOARING_ENABLED=1 and proper soaring configuration
 * @warning Soaring operations require careful parameter tuning and suitable weather conditions
 * 
 * @see libraries/AP_Soaring/AP_SoaringController.h for soaring algorithm implementation
 * @see ArduPlane/soaring.cpp for mode triggering logic
 * 
 * Source: ArduPlane/mode_thermal.cpp
 */

#include "mode.h"
#include "Plane.h"

// THERMAL mode is conditionally compiled based on HAL_SOARING_ENABLED
// This feature requires variometer hardware and soaring library support
#if HAL_SOARING_ENABLED

/**
 * @brief Enter THERMAL mode and initialize thermal soaring
 * 
 * @details This method is called when transitioning into THERMAL mode from another
 *          flight mode (typically AUTO, FBWB, or CRUISE). It performs initialization
 *          required for thermal soaring operations:
 *          
 *          1. Verifies soaring controller is active and ready
 *          2. Initiates loiter pattern at current aircraft position
 *          3. Resets loiter angle tracking for proper exit heading alignment
 *          4. Initializes thermal tracking state in soaring controller
 *          5. Sets initial thermal target location ahead on flight path
 *          
 *          The method will reject mode entry if the soaring controller reports
 *          it is not active (e.g., conditions not suitable for soaring, disabled
 *          by parameters, or hardware issues with variometer).
 *          
 *          Initialization Sequence:
 *          - do_loiter_at_location(): Sets up circular loiter at current position
 *          - loiter_angle_reset(): Clears accumulated loiter angle for exit logic
 *          - init_thermalling(): Prepares soaring controller thermal tracking
 *          - get_target(): Obtains initial thermal center estimate
 * 
 * @return true if mode entry successful and thermal soaring initialized
 * @return false if soaring controller not active, mode entry rejected
 * 
 * @note This is called automatically by the mode switching logic, not by pilot
 * @note The aircraft must have valid position and attitude estimates
 * @warning Mode will be rejected if soaring is disabled or variometer unavailable
 * 
 * @see Plane::do_loiter_at_location()
 * @see SoaringController::init_thermalling()
 * @see SoaringController::is_active()
 */
bool ModeThermal::_enter()
{
    if (!plane.g2.soaring_controller.is_active()) {
        return false;
    }

    plane.do_loiter_at_location();
    plane.loiter_angle_reset();

    plane.g2.soaring_controller.init_thermalling();
    plane.g2.soaring_controller.get_target(plane.next_WP_loc); // ahead on flight path

    return true;
}

/**
 * @brief Update navigation control outputs for thermal soaring
 * 
 * @details This method is called at the main loop rate (typically 50-400 Hz depending
 *          on scheduler configuration) to update the aircraft control outputs while
 *          circling in a thermal. It calculates the necessary control surface deflections
 *          and throttle settings to maintain the thermal loiter pattern.
 *          
 *          Control Updates:
 *          - calc_nav_roll(): Computes desired roll angle for circular loiter
 *          - calc_nav_pitch(): Computes desired pitch angle for speed/altitude control
 *          - calc_throttle(): Calculates throttle setting (typically idle or off in thermals)
 *          
 *          The actual loiter geometry (radius, center point) is determined by the
 *          navigate() method which is called from the navigation controller. This method
 *          focuses solely on converting navigation targets into control outputs.
 *          
 *          Integration with AP_Soaring:
 *          The thermal detection, strength estimation, and exit criteria are handled
 *          by update_soaring() which runs at a fixed 50 Hz to ensure consistent
 *          thermal tracking regardless of main loop rate variations.
 * 
 * @note Called at main loop rate (50-400 Hz) from the mode update scheduler
 * @note Does not handle thermal detection - see update_soaring() for that logic
 * 
 * @see ModeThermal::navigate() for loiter pattern calculation
 * @see ModeThermal::update_soaring() for thermal tracking and mode exit logic
 * @see Plane::calc_nav_roll()
 * @see Plane::calc_nav_pitch()
 * @see Plane::calc_throttle()
 */
void ModeThermal::update()
{
    plane.calc_nav_roll();
    plane.calc_nav_pitch();
    plane.calc_throttle();
}

/**
 * @brief Update thermal detection, tracking, and mode exit logic
 * 
 * @details This is the core thermal soaring management function, called at a fixed
 *          50 Hz rate from soaring.cpp to ensure consistent thermal tracking regardless
 *          of main loop rate settings. It handles:
 *          
 *          1. Thermal Strength Estimation:
 *             - Updates thermal position and strength estimates using variometer data
 *             - Adjusts loiter center to track thermal core movement
 *             - Calculates optimal loiter radius based on thermal characteristics
 *          
 *          2. Safety Limit Enforcement:
 *             - Maximum radius check: RTL if aircraft drifts beyond SOAR_MAX_RADIUS
 *             - Altitude limits: Exit if exceeding SOAR_ALT_MAX or below SOAR_ALT_MIN
 *             - Coordinate system: NED frame relative to home (requires home set)
 *          
 *          3. Drift Management:
 *             - Tracks position relative to desired mission path (if from AUTO mode)
 *             - Uses previous and next waypoints to calculate drift perpendicular to track
 *             - Falls back to thermal entry point if waypoint data unavailable
 *             - Exits if drift exceeds SOAR_MAX_DRIFT parameter
 *          
 *          4. Thermal Quality Assessment:
 *             - Monitors climb rate against SOAR_VSPEED threshold
 *             - Checks if thermal strength justifies continued circling
 *             - Evaluates multiple criteria via check_cruise_criteria()
 *          
 *          5. Exit Heading Alignment:
 *             - Ensures aircraft heading aligned with next objective before exit
 *             - Prevents disoriented exits that waste altitude
 *             - Waits for alignment or timeout (max 20s or SOAR_THERMAL_CRUSING_TIME)
 *          
 *          Exit Conditions (with ModeReason logging):
 *          - ALT_TOO_HIGH: Reached SOAR_ALT_MAX ceiling
 *          - ALT_TOO_LOW: Descended to SOAR_ALT_MIN floor (immediate exit)
 *          - THERMAL_WEAK: Climb rate below SOAR_VSPEED threshold
 *          - DRIFT_EXCEEDED: Lateral drift beyond SOAR_MAX_DRIFT limit
 *          - EXIT_COMMANDED: Manual exit via RC switch
 *          - SOARING_DRIFT_EXCEEDED: Exceeded SOAR_MAX_RADIUS from home (RTL mode)
 *          
 *          Coordinate Conversions:
 *          - Mission waypoints: Convert from cm to meters (divide by 100)
 *          - Position vectors: NED frame in centimeters from get_relative_position_NED_home()
 * 
 * @note Called at fixed 50 Hz from soaring.cpp, independent of main loop rate
 * @note Requires valid home position for NED coordinate calculations
 * @note Uses previous_mode to determine drift calculation method and exit behavior
 * 
 * @warning Immediate exit (no heading alignment) if ALT_TOO_LOW to prevent terrain impact
 * @warning RTL triggered if outside SOAR_MAX_RADIUS and previous mode not AUTO
 * 
 * @see SoaringController::update_thermalling() for thermal estimation algorithm
 * @see SoaringController::check_cruise_criteria() for exit condition evaluation
 * @see ModeThermal::exit_heading_aligned() for heading alignment logic
 * @see ModeThermal::restore_mode() for mode restoration
 * 
 * Source: ArduPlane/mode_thermal.cpp:28-113
 */
void ModeThermal::update_soaring()
{
    // Update the thermal estimation and switching logic.
    // This is called from soaring.cpp at fixed 50Hz to avoid
    // potential issues with the main loop rate setting.

    // Update thermal estimate and check for switch back to AUTO
    plane.g2.soaring_controller.update_thermalling();  // Update thermal position/strength estimate using variometer data

    // Thermalling is done in a home-relative coordinate system, so we need home to be set.
    // Position vector is in NED (North-East-Down) frame with units in meters
    Vector3f position;
    if (!AP::ahrs().get_relative_position_NED_home(position)) {
        return;  // Cannot operate without valid home position
    }

    // Check distance to home against MAX_RADIUS to prevent excessive drift from home position
    // Uses Euclidean distance: sqrt(x² + y²) compared to SOAR_MAX_RADIUS
    // Position in NED frame: x=North, y=East (in meters)
    if (plane.g2.soaring_controller.max_radius >= 0 &&
        sq(position.x)+sq(position.y) > sq(plane.g2.soaring_controller.max_radius) &&
        plane.previous_mode->mode_number()!=Mode::Number::AUTO) {
        // Outside maximum soaring radius and previous mode wasn't AUTO
        // Trigger RTL to bring aircraft back toward home
        gcs().send_text(MAV_SEVERITY_INFO, "Soaring: Outside SOAR_MAX_RADIUS, RTL");
        plane.set_mode(plane.mode_rtl, ModeReason::SOARING_DRIFT_EXCEEDED);
        return;
    }

    // Calculate drift relative to desired mission track if coming from AUTO mode
    // If previous mode was AUTO and there was a previous NAV command, we can use previous and next wps for drift calculation
    // with respect to the desired direction of travel. This allows detecting lateral drift perpendicular to the flight path.
    // If these vectors are zero, drift will be calculated from thermal start position only, without taking account of the
    // desired direction of travel (falls back to radial drift from thermal entry point).
    Vector2f prev_wp, next_wp;

    if (plane.previous_mode == &plane.mode_auto) {
        // Retrieve mission waypoints to calculate drift perpendicular to desired track
        AP_Mission::Mission_Command current_nav_cmd = plane.mission.get_current_nav_cmd();
        AP_Mission::Mission_Command prev_nav_cmd;

        // Attempt to get previous and current waypoint positions in NE coordinates (centimeters)
        // If any step fails (no previous waypoint, invalid location, etc.), zero the vectors
        // to fall back to simple radial drift calculation from thermal entry point
        if (!(plane.mission.get_next_nav_cmd(plane.mission.get_prev_nav_cmd_with_wp_index(), prev_nav_cmd) &&
            prev_nav_cmd.content.location.get_vector_xy_from_origin_NE_cm(prev_wp) &&
            current_nav_cmd.content.location.get_vector_xy_from_origin_NE_cm(next_wp))) {
            prev_wp.zero();
            next_wp.zero();
        }
    }

    // Get the status of the soaring controller cruise checks
    // Convert waypoint positions from centimeters to meters for soaring library
    const SoaringController::LoiterStatus loiterStatus = plane.g2.soaring_controller.check_cruise_criteria(prev_wp/100, next_wp/100);

    if (loiterStatus == SoaringController::LoiterStatus::GOOD_TO_KEEP_LOITERING) {
        // Thermal is still strong enough to continue circling
        // Reset loiter angle accumulator so that the loiter exit heading criteria
        // only starts expanding when we're ready to exit (prevents premature exit)
        plane.loiter.sum_cd = 0;
        plane.soaring_mode_timer_ms = AP_HAL::millis();

        // Update the thermal center location based on latest thermal position estimate
        plane.g2.soaring_controller.get_target(plane.next_WP_loc);

        return;  // Continue thermalling
    }

    // Thermal conditions indicate we should consider exiting
    // Calculate time spent in current thermal and timeout threshold
    const uint32_t time_in_loiter_ms = AP_HAL::millis() - plane.soaring_mode_timer_ms;
    const uint32_t timeout = MIN(1000*plane.g2.soaring_controller.get_circling_time(), 20000);  // Max 20 seconds

    if (!exit_heading_aligned() && loiterStatus != SoaringController::LoiterStatus::ALT_TOO_LOW && time_in_loiter_ms < timeout) {
        // Heading not lined up with next objective, and:
        // - Not timed out yet (still within circling time limit)
        // - Not in ALT_TOO_LOW condition (which requires immediate exit)
        // Continue loitering until heading aligns or timeout/emergency occurs
        return;
    }

    // Heading lined up (or timeout/emergency), and thermal conditions indicate exit
    // Restore previous flight mode with appropriate reason for telemetry and logging
    switch (loiterStatus) {
    case SoaringController::LoiterStatus::ALT_TOO_HIGH:
        // Reached maximum soaring altitude ceiling (SOAR_ALT_MAX parameter)
        restore_mode("Reached SOAR_ALT_MAX", ModeReason::SOARING_ALT_TOO_HIGH);
        break;
    case SoaringController::LoiterStatus::ALT_TOO_LOW:
        // Descended to minimum altitude floor (SOAR_ALT_MIN parameter)
        // This condition bypasses heading alignment for immediate exit
        restore_mode("Reached SOAR_ALT_MIN", ModeReason::SOARING_ALT_TOO_LOW);
        break;
    default:
    case SoaringController::LoiterStatus::THERMAL_WEAK:
        // Thermal lift weakened below minimum useful climb rate (SOAR_VSPEED parameter)
        restore_mode("Climb below SOAR_VSPEED", ModeReason::SOARING_THERMAL_ESTIMATE_DETERIORATED);
        break;
    case SoaringController::LoiterStatus::DRIFT_EXCEEDED:
        // Drifted too far laterally from desired mission track (SOAR_MAX_DRIFT parameter)
        restore_mode("Reached SOAR_MAX_DRIFT", ModeReason::SOARING_DRIFT_EXCEEDED);
        break;
    case SoaringController::LoiterStatus::EXIT_COMMANDED:
        // Pilot manually commanded exit via RC switch or GCS command
        restore_mode("Exit via RC switch", ModeReason::RC_COMMAND);
        break;
    } // switch loiterStatus
}

/**
 * @brief Calculate thermal loiter navigation pattern
 * 
 * @details This method is called by the navigation controller to compute the loiter
 *          pattern for circling within a thermal. Unlike standard loiter mode which
 *          uses a fixed radius, thermal loiter dynamically adjusts the radius based
 *          on thermal characteristics to optimize altitude gain.
 *          
 *          Radius Calculation:
 *          The soaring controller calculates optimal loiter radius from SOAR_THML_BANK
 *          parameter and estimated thermal characteristics. Typical formula:
 *          radius = airspeed² / (g × tan(bank_angle))
 *          
 *          The radius is dynamically adjusted to:
 *          - Match thermal diameter for maximum time in lift
 *          - Maintain safe bank angles at current airspeed
 *          - Optimize climb rate vs. thermal centering
 *          
 *          Loiter Center:
 *          The center point (plane.next_WP_loc) is continuously updated by
 *          update_soaring() based on thermal position estimates from the variometer.
 *          The aircraft flies a circle around this moving center point to track
 *          the thermal core as it drifts with the wind.
 * 
 * @note Called by navigation controller at navigation update rate (typically 10-50 Hz)
 * @note Radius varies with airspeed and thermal strength, not fixed like normal loiter
 * 
 * @see SoaringController::get_thermalling_radius() for radius calculation
 * @see Plane::update_loiter() for loiter pattern implementation
 * @see SOAR_THML_BANK parameter for bank angle configuration
 * 
 * Source: ArduPlane/mode_thermal.cpp:115-121
 */
void ModeThermal::navigate()
{
    // Soaring library calculates radius from SOAR_THML_BANK.
    const float radius = plane.g2.soaring_controller.get_thermalling_radius();

    plane.update_loiter(radius);
}

/**
 * @brief Check if aircraft heading is aligned for thermal exit
 * 
 * @details When exiting THERMAL mode, it's important to leave the thermal with the
 *          correct heading to efficiently continue toward the next objective. This
 *          method checks if the current aircraft heading is aligned with the next
 *          navigation target based on the previous flight mode.
 *          
 *          The alignment check varies by previous mode:
 *          
 *          AUTO Mode:
 *          - Checks alignment with next mission waypoint after current nav command
 *          - Uses mission command location data for precise path following
 *          - Ensures smooth return to mission track with minimal course correction
 *          
 *          FLY_BY_WIRE_B Mode:
 *          - Aligns heading toward home position if home is set
 *          - Returns true if home not available (allows immediate exit)
 *          - Prevents aircraft from thermalling away from home indefinitely
 *          
 *          CRUISE Mode:
 *          - Aligns with cruise mode's target heading if locked
 *          - Returns true if heading not locked (allows immediate exit)
 *          - Maintains cruise direction continuity
 *          
 *          Other Modes:
 *          - Returns true (no heading constraint)
 *          - Allows immediate exit as no specific heading required
 *          
 *          The alignment tolerance is determined by the loiter heading alignment
 *          logic which typically allows exit when within 20-30 degrees and after
 *          completing sufficient loiter rotation.
 * 
 * @return true if heading aligned with next objective or no alignment required
 * @return false if heading not yet aligned, should continue loitering
 * 
 * @note Safe to exit immediately (returns true) if home not set or heading not locked
 * @note Uses loiter.sum_cd (accumulated loiter angle) to determine alignment
 * 
 * @see ModeLoiter::isHeadingLinedUp() for alignment tolerance
 * @see ModeLoiter::isHeadingLinedUp_cd() for heading-based alignment
 * 
 * Source: ArduPlane/mode_thermal.cpp:123-142
 */
bool ModeThermal::exit_heading_aligned() const
{
    // Return true if the current heading is aligned with the next objective.
    // If home is not set, or heading not locked, return true to avoid delaying mode change.
    switch (plane.previous_mode->mode_number()) {
    case Mode::Number::AUTO: {
        //Get the lat/lon of next Nav waypoint after this one:
        AP_Mission::Mission_Command current_nav_cmd = plane.mission.get_current_nav_cmd();
        return plane.mode_loiter.isHeadingLinedUp(plane.next_WP_loc, current_nav_cmd.content.location);
    }
    case Mode::Number::FLY_BY_WIRE_B:
        return (!AP::ahrs().home_is_set() || plane.mode_loiter.isHeadingLinedUp(plane.next_WP_loc, AP::ahrs().get_home()));
    case Mode::Number::CRUISE:
        int32_t target_heading_cd;
        return (!plane.mode_cruise.get_target_heading_cd(target_heading_cd) || plane.mode_loiter.isHeadingLinedUp_cd(target_heading_cd));
    default:
        break;
    }
    return true;
}

/**
 * @brief Restore previous flight mode after thermal exit
 * 
 * @details This method is called when exiting THERMAL mode to return the aircraft
 *          to the previous flight mode (typically AUTO, FBWB, or CRUISE). It provides
 *          telemetry feedback about the reason for exiting the thermal and ensures
 *          a smooth transition back to the previous mode.
 *          
 *          Exit Reasons Logged:
 *          - "Reached SOAR_ALT_MAX": Climbed to maximum soaring altitude
 *          - "Reached SOAR_ALT_MIN": Descended to minimum soaring altitude
 *          - "Climb below SOAR_VSPEED": Thermal weakened below useful strength
 *          - "Reached SOAR_MAX_DRIFT": Drifted too far from desired track
 *          - "Exit via RC switch": Pilot commanded exit manually
 *          
 *          The method sends an informational message to ground control stations showing:
 *          1. The specific reason for thermal exit
 *          2. The name of the mode being restored
 *          
 *          Mode Restoration:
 *          Uses plane.set_mode() with the stored previous_mode pointer and appropriate
 *          ModeReason for dataflash logging and telemetry. This allows post-flight
 *          analysis of soaring performance and exit triggers.
 * 
 * @param[in] reason Human-readable string describing why thermal was exited
 * @param[in] modereason Enumerated reason code for logging and telemetry
 * 
 * @note Always sends MAV_SEVERITY_INFO level message to GCS
 * @note Previous mode is stored when entering THERMAL mode
 * @note ModeReason is logged to dataflash for post-flight analysis
 * 
 * @see Plane::set_mode() for mode transition logic
 * @see ModeReason enumeration for reason codes
 * 
 * Source: ArduPlane/mode_thermal.cpp:144-148
 */
void ModeThermal::restore_mode(const char *reason, ModeReason modereason)
{
    gcs().send_text(MAV_SEVERITY_INFO, "Soaring: %s, restoring %s", reason, plane.previous_mode->name());
    plane.set_mode(*plane.previous_mode, modereason);
}

#endif // HAL_SOARING_ENABLED
