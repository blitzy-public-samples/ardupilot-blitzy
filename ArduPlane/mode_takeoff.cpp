/**
 * @file mode_takeoff.cpp
 * @brief Implementation of TAKEOFF flight mode for automated takeoff sequence
 * 
 * @details This file implements the TAKEOFF mode for ArduPlane, which provides
 *          automated takeoff capability for fixed-wing aircraft. The mode handles
 *          multiple takeoff scenarios including:
 *          - Conventional ground roll takeoffs with rotation and climb
 *          - Hand-launch detection and automatic transition to climb
 *          - Catapult launch support with immediate climb initiation
 *          - Already-flying detection for mode switches during flight
 * 
 *          TAKEOFF mode executes a multi-phase sequence:
 *          1. Ground Roll: Aircraft accelerates on runway while maintaining ground pitch
 *          2. Rotation: At TKOFF_ROTATE_SPD, pitch increases to climb attitude
 *          3. Climb: Aircraft climbs to TKOFF_ALT while maintaining wings level below TKOFF_LVL_ALT
 *          4. Transition: At target altitude, transitions to loiter at TKOFF_DIST ahead
 * 
 *          The mode supports compass-less takeoff by using GPS groundspeed vector
 *          for heading determination once minimum ground speed is achieved.
 * 
 * @note This is a safety-critical flight mode - correct takeoff phase management
 *       is essential for vehicle safety during the high-risk takeoff operation.
 * 
 * @warning Incorrect parameter configuration (especially TKOFF_GND_PITCH for
 *          taildraggers) can result in nose-over or premature rotation.
 * 
 * @see Plane::takeoff_calc_roll()
 * @see Plane::takeoff_calc_pitch()
 * @see Plane::takeoff_calc_throttle()
 */

#include "mode.h"
#include "Plane.h"
#include <GCS_MAVLink/GCS.h>

/**
 * @brief Parameter group definition for TAKEOFF mode configuration
 * 
 * @details These parameters control the automated takeoff sequence behavior.
 *          Parameters define target altitude, level-off altitude, pitch attitudes,
 *          and the loiter distance after takeoff completion.
 */
const AP_Param::GroupInfo ModeTakeoff::var_info[] = {
    // @Param: ALT
    // @DisplayName: Takeoff mode altitude
    // @Description: This is the target altitude for TAKEOFF mode
    // @Range: 0 200
    // @Increment: 1
    // @Units: m
    // @User: Standard
    AP_GROUPINFO("ALT", 1, ModeTakeoff, target_alt, 50),

    // @Param: LVL_ALT
    // @DisplayName: Takeoff mode altitude level altitude
    // @Description: This is the altitude below which the wings are held level for TAKEOFF and AUTO modes. Below this altitude, roll demand is restricted to LEVEL_ROLL_LIMIT. Normal-flight roll restriction resumes above TKOFF_LVL_ALT*3 or TKOFF_ALT, whichever is lower. Roll limits are scaled while between TKOFF_LVL_ALT and those altitudes for a smooth transition.
    // @Range: 0 50
    // @Increment: 1
    // @Units: m
    // @User: Standard
    AP_GROUPINFO("LVL_ALT", 2, ModeTakeoff, level_alt, 10),

    // @Param: LVL_PITCH
    // @DisplayName: Takeoff mode altitude initial pitch
    // @Description: This is the target pitch during the takeoff.
    // @Range: 0 30
    // @Increment: 1
    // @Units: deg
    // @User: Standard
    AP_GROUPINFO("LVL_PITCH", 3, ModeTakeoff, level_pitch, 15),

    // @Param: DIST
    // @DisplayName: Takeoff mode distance
    // @Description: This is the distance from the takeoff location where the plane will loiter. The loiter point will be in the direction of takeoff (the direction the plane is facing when the plane begins takeoff)
    // @Range: 0 500
    // @Increment: 1
    // @Units: m
    // @User: Standard
    AP_GROUPINFO("DIST", 4, ModeTakeoff, target_dist, 200),

    // @Param: GND_PITCH
    // @DisplayName: Takeoff run pitch demand
    // @Description: Degrees of pitch angle demanded during the takeoff run before speed reaches TKOFF_ROTATE_SPD. For taildraggers set to 3-point ground pitch angle and use TKOFF_TDRAG_ELEV to prevent nose tipover. For nose-wheel steer aircraft set to the ground pitch angle and if a reduction in nose-wheel load is required as speed rises, use a positive offset in TKOFF_GND_PITCH of up to 5 degrees above the angle on ground, starting at the measured pitch angle and incrementing in 1 degree steps whilst checking for premature rotation and takeoff with each increment. To increase nose-wheel load, use a negative TKOFF_TDRAG_ELEV and refer to notes on TKOFF_TDRAG_ELEV before making adjustments.
    // @Units: deg
    // @Range: -5.0 10.0
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("GND_PITCH", 5, ModeTakeoff, ground_pitch, 5),

    AP_GROUPEND
};

/**
 * @brief Constructor for TAKEOFF mode
 * 
 * @details Initializes the TAKEOFF mode object and sets up parameter defaults
 *          from the var_info table. The constructor chains to the base Mode
 *          class constructor to inherit common mode functionality.
 */
ModeTakeoff::ModeTakeoff() :
    Mode()
{
    AP_Param::setup_object_defaults(this, var_info);
}

/**
 * @brief Enter TAKEOFF mode and initialize mode state
 * 
 * @details Called when the pilot switches to TAKEOFF mode. This method initializes
 *          the mode state variables that control the takeoff sequence. The actual
 *          takeoff waypoint setup occurs in update() once valid position and home
 *          are available.
 * 
 *          Initialization includes:
 *          - Clearing takeoff_mode_setup flag to trigger waypoint configuration
 *          - Resetting fence auto-enable flag for post-takeoff fence activation
 * 
 *          The method does minimal initialization to allow immediate throttle
 *          response for hand-launch and catapult scenarios where the aircraft
 *          may already be airborne when mode is entered.
 * 
 * @return true Always returns true - mode entry always succeeds
 * 
 * @note This method only initializes flags; actual takeoff sequence parameters
 *       are configured in update() based on current flight state
 * 
 * @see update() for takeoff sequence execution and waypoint setup
 */
bool ModeTakeoff::_enter()
{
    takeoff_mode_setup = false;
    have_autoenabled_fences = false;

    return true;
}

/**
 * @brief Execute TAKEOFF mode control logic for current loop iteration
 * 
 * @details This method implements the complete automated takeoff sequence and is
 *          called at the main loop rate (typically 50Hz or 400Hz depending on
 *          scheduler configuration). It manages multiple takeoff scenarios and
 *          transitions through flight stages.
 * 
 *          Takeoff Scenarios Handled:
 *          1. **Ground Roll Takeoff**: Aircraft starts on ground, accelerates,
 *             rotates, and climbs to target altitude
 *          2. **Hand Launch**: Detects aircraft is already flying and transitions
 *             directly to climb phase
 *          3. **Catapult Launch**: Similar to hand launch with immediate flight detection
 *          4. **Mid-Flight Mode Entry**: If already above target altitude, enters
 *             loiter immediately
 * 
 *          Flight Stage Progression:
 *          - **Pre-Setup**: Waits for valid position and home before configuring waypoints
 *          - **Setup Phase**: Determines takeoff scenario and configures target waypoint
 *             * For already-flying: Sets climb target or immediate loiter
 *             * For ground takeoff: Waits for GPS_GND_CRS_MIN_SPD to lock heading
 *          - **TAKEOFF Stage**: Active climb with roll limits and takeoff pitch/throttle
 *             * Holds wings level below TKOFF_LVL_ALT
 *             * Updates loiter waypoint once above TKOFF_LVL_ALT using actual bearing
 *             * Completes when reaching TKOFF_ALT or TKOFF_DIST
 *          - **NORMAL Stage**: Transitions to loiter with standard navigation and fence enable
 * 
 *          Compass-Less Takeoff Support:
 *          The mode delays waypoint lock-in until groundspeed exceeds GPS_GND_CRS_MIN_SPD,
 *          using GPS velocity vector for heading instead of compass. This supports
 *          aircraft with no compass or compass interference from high-current takeoff.
 * 
 *          Altitude Target Completion:
 *          Takeoff is considered complete when:
 *          - Altitude reaches (TKOFF_ALT - 2m) to allow for altitude variance, OR
 *          - Horizontal distance from start exceeds TKOFF_DIST (prevents infinite climb)
 * 
 *          Waypoint Correction:
 *          Once above TKOFF_LVL_ALT, the loiter waypoint bearing is updated based on
 *          actual GPS track to correct for:
 *          - EKF yaw estimation errors during ground roll
 *          - Compass interference from maximum throttle
 *          - Crosswind drift during initial climb
 * 
 * @note This method must handle the case where position or home is not yet available
 *       at mode entry, outputting safe control values until initialization can occur
 * 
 * @warning Takeoff is a high-risk phase of flight. The mode includes multiple safety
 *          checks including timeout protection and already-flying detection to prevent
 *          unsafe behavior. Do not modify without thorough SITL and hardware testing.
 * 
 * @see Plane::takeoff_calc_roll() for roll control during TAKEOFF stage
 * @see Plane::takeoff_calc_pitch() for pitch control during TAKEOFF stage  
 * @see Plane::takeoff_calc_throttle() for throttle control during TAKEOFF stage
 * @see Plane::check_takeoff_timeout() for takeoff timeout protection
 * @see Plane::is_flying() for flight detection algorithm
 */
void ModeTakeoff::update()
{
    // Wait for valid position and home before configuring takeoff waypoints.
    // Without these, we cannot determine takeoff direction or set target altitude.
    // Output safe control values (zero throttle, level attitude) until ready.
    if (!(plane.current_loc.initialised() && AP::ahrs().home_is_set())) {
        plane.calc_nav_roll();
        plane.calc_nav_pitch();
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0.0);
        return;
    }

    // Store target altitude (meters) and loiter distance (meters) from parameters
    const float alt = target_alt;
    const float dist = target_dist;
    
    // One-time takeoff configuration on first update() call after mode entry.
    // This setup determines the takeoff scenario (ground roll, hand-launch, or
    // already flying) and configures appropriate target waypoints.
    if (!takeoff_mode_setup) {
        plane.auto_state.takeoff_altitude_rel_cm = alt * 100;
        const uint16_t altitude = plane.relative_ground_altitude(RangeFinderUse::NONE,true);
        const Vector2f &groundspeed2d = ahrs.groundspeed_vector();
        const float direction = wrap_360(degrees(groundspeed2d.angle()));
        const float groundspeed = groundspeed2d.length();

        // Check if aircraft is already flying (hand-launch or catapult scenario).
        // is_flying() uses accelerometer, airspeed, and GPS data to detect flight.
        // Require 10 seconds of flying and >3m/s groundspeed to avoid false positives.
        if (plane.is_flying() && (millis() - plane.started_flying_ms > 10000U) && groundspeed > 3) {
            // Already-flying branch: Skip ground roll and rotation phases
            if (altitude >= alt) {
                // Already above target altitude - go directly to loiter at current position
                gcs().send_text(MAV_SEVERITY_INFO, "Above TKOFF alt - loitering");
                plane.next_WP_loc = plane.current_loc;
                takeoff_mode_setup = true;
                plane.set_flight_stage(AP_FixedWing::FlightStage::NORMAL);
            } else {
                // Below target altitude - climb to TKOFF_ALT then loiter at TKOFF_DIST ahead
                gcs().send_text(MAV_SEVERITY_INFO, "Climbing to TKOFF alt then loitering");
                start_loc = plane.current_loc;
                plane.next_WP_loc = plane.current_loc;
                plane.next_WP_loc.alt += ((alt - altitude) *100);
                plane.next_WP_loc.offset_bearing(direction, dist);
                takeoff_mode_setup = true;
                plane.set_flight_stage(AP_FixedWing::FlightStage::TAKEOFF);
            }
        } else {
            // Ground takeoff branch: Not yet flying, so execute full takeoff sequence
            
            // Configure target waypoint for loiter at TKOFF_DIST ahead in current direction.
            // Initial direction is based on GPS groundspeed vector, not compass, to support
            // compass-less operation and avoid magnetic interference during high-current takeoff.
            start_loc = plane.current_loc;
            plane.prev_WP_loc = plane.current_loc;
            plane.next_WP_loc = plane.current_loc;
            plane.next_WP_loc.alt += alt*100.0;
            plane.next_WP_loc.offset_bearing(direction, dist);

            // Clear any pre-existing crash detection state
            plane.crash_state.is_crashed = false;

            // Set initial takeoff pitch from TKOFF_LVL_PITCH parameter (centidegrees)
            plane.auto_state.takeoff_pitch_cd = level_pitch * 100;

            // Enter TAKEOFF flight stage to activate takeoff-specific roll/pitch/throttle control
            plane.set_flight_stage(AP_FixedWing::FlightStage::TAKEOFF);

            // Compass-less takeoff support: Don't lock in the takeoff direction until
            // groundspeed exceeds GPS_GND_CRS_MIN_SPD. This allows using GPS velocity
            // for heading instead of compass, supporting aircraft with no compass or
            // severe compass interference from high takeoff currents.
            // 
            // The groundspeed threshold ensures GPS heading is reliable before committing
            // to a takeoff direction. In very strong headwinds where groundspeed remains
            // low, the is_flying() check above will eventually trigger and handle the
            // takeoff as a hand-launch scenario.
            if (!plane.throttle_suppressed &&
                groundspeed > GPS_GND_CRS_MIN_SPD) {
                gcs().send_text(MAV_SEVERITY_INFO, "Takeoff to %.0fm for %.1fm heading %.1f deg",
                                alt, dist, direction);
                plane.takeoff_state.start_time_ms = millis();
                plane.takeoff_state.level_off_start_time_ms = 0;
                plane.takeoff_state.throttle_max_timer_ms = millis();
                takeoff_mode_setup = true;
                // Set hold_course to enable takeoff_calc_roll() to maintain runway heading
                plane.steer_state.hold_course_cd = wrap_360_cd(direction*100);
            }
        }
    }
    
    // Safety check: If takeoff is taking too long (timeout configured via TKOFF_TIMEOUT),
    // abort the takeoff attempt and transition to NORMAL flight to allow pilot intervention
    if (plane.check_takeoff_timeout()) {
        plane.set_flight_stage(AP_FixedWing::FlightStage::NORMAL);
        takeoff_mode_setup = false;
    }
    
    // Waypoint bearing correction: Once above TKOFF_LVL_ALT or past the target distance,
    // update the loiter waypoint bearing based on actual GPS track from start position.
    // 
    // This correction addresses several issues during initial takeoff:
    // - EKF yaw estimation errors from EKF resets during ground roll
    // - Compass interference from maximum throttle current draw
    // - Crosswind drift during initial climb requiring track correction
    // 
    // By using actual GPS bearing from start position, we ensure the loiter waypoint
    // is correctly positioned ahead of the aircraft's actual flight path rather than
    // the initially estimated heading. The hold_course_cd == -1 check ensures this
    // correction only happens once (the value is set to a real heading after correction).
    const float altitude_cm = plane.current_loc.alt - start_loc.alt;
    if (plane.flight_stage == AP_FixedWing::FlightStage::TAKEOFF
        && plane.steer_state.hold_course_cd == -1  // Enter-once flag: -1 means not yet corrected
        && (altitude_cm >= (level_alt * 100.0f) || start_loc.get_distance(plane.current_loc) >= dist)
    ) {
        // Calculate actual bearing from takeoff start to current position
        const float direction = start_loc.get_bearing_to(plane.current_loc) * 0.01;
        plane.next_WP_loc = start_loc;
        plane.next_WP_loc.offset_bearing(direction, dist);
        plane.next_WP_loc.alt += alt*100.0;
        // Update hold_course to actual bearing and enable takeoff_calc_roll() heading hold
        plane.steer_state.hold_course_cd = wrap_360_cd(direction*100);
    }
    
    // Takeoff completion check: Transition from TAKEOFF to NORMAL flight stage when
    // the aircraft reaches the target altitude or distance. Uses (TKOFF_ALT - 2m)
    // threshold to account for altitude control variance and allow smooth transition.
    // 
    // The distance check provides a safety fallback: if the aircraft cannot climb
    // (due to excessive weight, insufficient power, or strong downdrafts), it will
    // still transition to normal flight after traveling TKOFF_DIST horizontally.
    // This prevents indefinite attempts to reach an unattainable altitude.
    if (plane.flight_stage == AP_FixedWing::FlightStage::TAKEOFF &&
        (altitude_cm >= (alt*100 - 200) ||
        start_loc.get_distance(plane.current_loc) >= dist)) {
        plane.set_flight_stage(AP_FixedWing::FlightStage::NORMAL);
    }

    // Level-off timeout check: During the final pitch level-off phase, if the aircraft
    // cannot close the last few meters to target altitude within the timeout period,
    // proceed to NORMAL flight stage rather than continuing indefinitely.
    if (plane.check_takeoff_timeout_level_off()) {
        plane.set_flight_stage(AP_FixedWing::FlightStage::NORMAL);
    }

    // Control calculation based on current flight stage
    if (plane.flight_stage == AP_FixedWing::FlightStage::TAKEOFF) {
        // TAKEOFF stage: Use specialized takeoff control algorithms
        // - takeoff_calc_roll(): Limits roll based on altitude (wings level below TKOFF_LVL_ALT)
        // - takeoff_calc_pitch(): Manages pitch from ground pitch through rotation to climb
        // - takeoff_calc_throttle(): Maximum throttle with optional limits during ground roll
        plane.takeoff_calc_roll();
        plane.takeoff_calc_pitch();
        plane.takeoff_calc_throttle();
    } else {
        // NORMAL stage: Takeoff complete, use standard navigation control
        
#if AP_FENCE_ENABLED
        // Auto-enable geofences after takeoff completion. Fences are typically disabled
        // during takeoff to prevent false breach detections during the climb phase, then
        // automatically enabled once the aircraft reaches stable flight.
        if (!have_autoenabled_fences) {
            plane.fence.auto_enable_fence_after_takeoff();
            have_autoenabled_fences = true;
        }
#endif
        
        // Standard navigation control for loiter at target waypoint
        plane.calc_nav_roll();    // Calculate roll for waypoint navigation
        plane.calc_nav_pitch();   // Calculate pitch for altitude hold
        plane.calc_throttle();    // Calculate throttle for airspeed/altitude control
        
        // Deferred failsafe handling: If a long failsafe condition occurred during the
        // TAKEOFF stage, it was deferred to avoid aborting the critical takeoff phase.
        // Now that takeoff is complete, trigger the failsafe action if still pending.
        if (plane.long_failsafe_pending) {
            plane.long_failsafe_pending = false;
            plane.failsafe_long_on_event(FAILSAFE_LONG, ModeReason::MODE_TAKEOFF_FAILSAFE);
        }
    }
}

/**
 * @brief Update navigation for TAKEOFF mode loiter target
 * 
 * @details Called by the navigation system to update the loiter navigation state.
 *          This method is invoked during the NORMAL flight stage after takeoff
 *          completion to maintain the loiter circle at the target waypoint.
 * 
 *          The method calls update_loiter() with radius parameter 0, which instructs
 *          the loiter controller to use the WP_LOITER_RAD parameter value for the
 *          loiter circle radius. This provides consistent loiter behavior with other
 *          loiter-based modes.
 * 
 * @note This method is not called during the TAKEOFF flight stage; it only becomes
 *       active after transitioning to NORMAL stage when takeoff is complete.
 * 
 * @see Plane::update_loiter() for loiter navigation implementation
 * @see update() for takeoff sequence and stage transitions
 */
void ModeTakeoff::navigate()
{
    // Zero indicates to use WP_LOITER_RAD parameter for loiter radius
    plane.update_loiter(0);
}

