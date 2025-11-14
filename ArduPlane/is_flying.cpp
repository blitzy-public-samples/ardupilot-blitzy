/**
 * @file is_flying.cpp
 * @brief Fixed-wing aircraft flight detection and crash detection logic
 * 
 * @details This file implements flight state detection for ArduPlane, determining whether
 *          the aircraft is airborne or on the ground. This information is critical for:
 *          - Mode transitions and flight stage management
 *          - Crash detection and safety mechanisms
 *          - Automatic arming/disarming decisions
 *          - Logging and telemetry status
 *          - Failsafe behavior activation
 * 
 *          The detection uses a probabilistic approach with multiple sensor inputs:
 *          - Airspeed: Threshold based on stall speed (typically 75% of minimum airspeed)
 *          - GPS ground speed: Movement confirmation when GPS lock available
 *          - IMU acceleration: Detects impacts and confirms movement
 *          - Altitude changes: Monitors climb/sink rates during landing
 *          - Flight stage context: Adjusts detection logic for takeoff, landing, cruise
 * 
 *          The probabilistic method low-pass filters a boolean flying state to create
 *          a confidence value (isFlyingProbability), allowing graceful transitions and
 *          reducing false positives during brief sensor dropouts.
 * 
 * @note Flight detection accuracy is essential for safety-critical decisions.
 *       False negatives (thinking we're grounded when airborne) can trigger
 *       inappropriate disarm or mode changes. False positives are generally safer
 *       but can delay landing detection.
 * 
 * @warning Modifications to flight detection logic must be thoroughly tested in
 *          SITL and on actual hardware across all flight stages (takeoff, cruise,
 *          landing) to prevent unsafe behavior.
 * 
 * @see Plane::is_flying() for the main query interface
 * @see Plane::crash_detection_update() for crash detection logic
 * 
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#include "Plane.h"

#include <AP_Stats/AP_Stats.h>     // statistics library

/**
 * @brief Debounce time for crash detection to avoid false positives
 * @details Time in milliseconds to wait before confirming a crash has occurred.
 *          This prevents transient events (e.g., hard gusts, turbulence) from
 *          triggering crash detection.
 */
#define CRASH_DETECTION_DELAY_MS            500

/**
 * @brief Duration to remember a detected impact event
 * @details After detecting a significant deceleration (potential ground impact),
 *          this timer controls how long we clip the flying probability to allow
 *          faster descent toward "not flying" state. Units: milliseconds.
 */
#define IS_FLYING_IMPACT_TIMER_MS           3000

/**
 * @brief Default minimum GPS ground speed to indicate flight
 * @details GPS ground speed threshold in cm/s (150 cm/s = 1.5 m/s = ~3 knots).
 *          Used when ARSPD_MIN parameter is not set. This speed indicates
 *          the aircraft is moving fast enough to likely be airborne rather
 *          than taxiing or being blown by wind while on the ground.
 */
#define GPS_IS_FLYING_SPEED_CMS             150

/**
 * @brief Update flight state detection at 5Hz using probabilistic sensor fusion
 * 
 * @details This function is the core of ArduPlane's flight detection system, called
 *          at 5Hz by the scheduler. It uses a probabilistic approach where multiple
 *          sensor inputs are combined to determine if the aircraft is airborne.
 * 
 *          Detection Criteria (varies by arming state and flight stage):
 *          
 *          When ARMED (assume flying unless proven otherwise):
 *          - Airspeed >= 75% of stall speed (ARSPD_MIN * 0.75)
 *          - GPS ground speed >= 90% of min_groundspeed parameter (or 150 cm/s default)
 *          - IMU confirms vehicle is not stationary (AP::ins().is_still())
 *          - Handles GPS dropouts by continuing with last airspeed estimate
 *          
 *          When DISARMED (assume not flying unless proven otherwise):
 *          - Requires BOTH airspeed movement AND GPS confirmed movement
 *          - Higher threshold to declare flying state
 * 
 *          Special Handling:
 *          - Quadplane mode: Delegates to quadplane.is_flying() if enabled
 *          - Auto mode with takeoff: Disables false airspeed readings on ground
 *          - Landing approach: Uses sink rate to maintain flying state
 *          - Abort landing: Uses climb rate to confirm flying
 *          - Impact detection: X-axis deceleration triggers rapid probability decay
 * 
 *          Probability Filter:
 *          The boolean flying determination is low-pass filtered with coefficient 0.15
 *          at 5Hz, giving a time constant of ~3 seconds for 90% probability change.
 *          This prevents rapid state changes during brief sensor dropouts.
 * 
 * @note Called at 5Hz by the scheduler task system. Do not call more frequently
 *       as the filter coefficients are tuned for 5Hz update rate.
 * 
 * @note Updates the following state variables:
 *       - isFlyingProbability: Filtered flying confidence (0.0 to 1.0)
 *       - auto_state.last_flying_ms: Timestamp of last confirmed flying
 *       - auto_state.started_flying_in_auto_ms: When auto mode flight began
 *       - crash_state.impact_detected: Whether ground impact detected
 *       - crash_state.is_crashed: Whether crash confirmed
 *       - previous_is_flying: Previous flying state for edge detection
 * 
 * @note Notifies multiple subsystems of flying state changes:
 *       - ADSB: For transponder status
 *       - Parachute: For deployment logic
 *       - Statistics: For flight time tracking
 *       - AP_Notify: For LED/buzzer indicators
 *       - AHRS: For likely_flying state used in EKF
 *       - Logger: Writes STATUS messages
 * 
 * @warning This is safety-critical code. Changes affect crash detection, auto-disarm,
 *          mode transitions, and failsafe behavior. Test thoroughly in SITL with:
 *          - Normal takeoffs and landings
 *          - GPS dropouts during flight
 *          - Strong wind conditions on ground
 *          - Bungee/catapult launches
 *          - Belly landings and hard landings
 * 
 * @see Plane::is_flying() for querying current flying state
 * @see Plane::crash_detection_update() for crash detection logic
 * @see quadplane.is_flying() for VTOL-specific detection
 * 
 * Source: ArduPlane/is_flying.cpp:17-184
 */
void Plane::update_is_flying_5Hz(void)
{
    float aspeed=0;
    bool is_flying_bool = false;
    uint32_t now_ms = AP_HAL::millis();

    // GPS movement threshold: Use 90% of configured min groundspeed, or 150cm/s default
    // The 90% factor provides margin to avoid ground speed noise triggering false flying state
    uint32_t ground_speed_thresh_cm = (aparm.min_groundspeed > 0) ? ((uint32_t)(aparm.min_groundspeed*(100*0.9))) : GPS_IS_FLYING_SPEED_CMS;
    bool gps_confirmed_movement = (gps.status() >= AP_GPS::GPS_OK_FIX_3D) &&
                                    (gps.ground_speed_cm() >= ground_speed_thresh_cm);

    // Airspeed threshold: 75% of stall speed indicates we're moving through the air fast enough to be flying
    // Minimum threshold of 2 m/s * 0.75 = 1.5 m/s prevents sensor noise from triggering detection
    const float airspeed_threshold = MAX(aparm.airspeed_min,2)*0.75f;
    bool airspeed_movement = ahrs.airspeed_estimate(aspeed) && (aspeed >= airspeed_threshold);

    // Handle GPS loss during flight: If we've lost GPS but still have some flying confidence,
    // continue using the last airspeed estimate. This prevents false crash detection during
    // extended GPS dropouts when dead-reckoning. The 0.3 probability threshold ensures we only
    // do this when we have reasonable confidence we were recently flying.
    if (gps.status() < AP_GPS::GPS_OK_FIX_2D && arming.is_armed() && !airspeed_movement && isFlyingProbability > 0.3) {
        // when flying with no GPS, use the last airspeed estimate to
        // determine if we think we have airspeed movement. This
        // prevents the crash detector from triggering when
        // dead-reckoning under long GPS loss
        airspeed_movement = aspeed >= airspeed_threshold;
    }

#if HAL_QUADPLANE_ENABLED
    is_flying_bool = quadplane.is_flying();
#endif
    if (is_flying_bool) {
        // no need to look further
    } else if(arming.is_armed()) {
        // when armed assuming flying and we need overwhelming evidence that we ARE NOT flying
        // short drop-outs of GPS are common during flight due to banking which points the antenna in different directions
        bool gps_lost_recently = (gps.last_fix_time_ms() > 0) && // we have locked to GPS before
                        (gps.status() < AP_GPS::GPS_OK_FIX_2D) && // and it's lost now
                        (now_ms - gps.last_fix_time_ms() < 5000); // but it wasn't that long ago (<5s)

        if ((auto_state.last_flying_ms > 0) && gps_lost_recently) {
            // we've flown before, remove GPS constraints temporarily and only use airspeed
            is_flying_bool = airspeed_movement; // moving through the air
        } else {
            // Because ahrs.airspeed_estimate can return a continued high value after landing if flying in
            // strong winds above stall speed it is necessary to include the IMU based movement check.
            is_flying_bool = (airspeed_movement && !AP::ins().is_still()) || // moving through the air
                                gps_confirmed_movement; // locked and we're moving
        }

        if (control_mode == &mode_auto) {
            /*
              make is_flying() more accurate during various auto modes
             */

            // Detect X-axis deceleration for probable ground impacts.
            // When a hard deceleration is detected (e.g., hitting ground, tree, or other obstacle),
            // clip the flying probability to 0.2 to allow faster transition to "not flying" state.
            // This helps crash detection respond more quickly after impacts.
            // Limit the max probability so it can decay faster. This
            // will not change the is_flying state, anything above 0.1
            // is "true", it just allows it to decay faster once we decide we
            // aren't flying using the normal schemes
            if (g.crash_accel_threshold == 0) {
                // Impact detection disabled via parameter
                crash_state.impact_detected = false;
            } else if (ins.get_accel_peak_hold_neg_x() < -(g.crash_accel_threshold)) {
                // Large negative X-axis deceleration detected (units: m/s²)
                // Peak hold value captures maximum deceleration since last reset
                // Negative X = forward deceleration (nose hitting something)
                crash_state.impact_detected = true;
                crash_state.impact_timer_ms = now_ms;
                if (isFlyingProbability > 0.2f) {
                    isFlyingProbability = 0.2f;  // Clip to 0.2 to accelerate decay toward "not flying"
                }
            } else if (crash_state.impact_detected &&
                (now_ms - crash_state.impact_timer_ms > IS_FLYING_IMPACT_TIMER_MS)) {
                // No impacts seen for IS_FLYING_IMPACT_TIMER_MS (3 seconds), 
                // clear the flag so we stop clipping isFlyingProbability
                crash_state.impact_detected = false;
            }

            switch (flight_stage)
            {
            case AP_FixedWing::FlightStage::TAKEOFF:
                break;

            case AP_FixedWing::FlightStage::NORMAL:
                if (in_preLaunch_flight_stage()) {
                    // while on the ground, an uncalibrated airspeed sensor can drift to 7m/s so
                    // ensure we aren't showing a false positive.
                    is_flying_bool = false;
                    crash_state.is_crashed = false;
                    auto_state.started_flying_in_auto_ms = 0;
                }
                break;

            case AP_FixedWing::FlightStage::VTOL:
                // TODO: detect ground impacts
                break;

            case AP_FixedWing::FlightStage::LAND:
                if (landing.is_on_approach() && auto_state.sink_rate > 0.2f) {
                    is_flying_bool = true;
                }
                break;

            case AP_FixedWing::FlightStage::ABORT_LANDING:
                if (auto_state.sink_rate < -0.5f) {
                    // steep climb
                    is_flying_bool = true;
                }
                break;

            default:
                break;
            } // switch
        }
    } else {
        // when disarmed assume not flying and need overwhelming evidence that we ARE flying
        is_flying_bool = airspeed_movement && gps_confirmed_movement;

        if ((flight_stage == AP_FixedWing::FlightStage::TAKEOFF) || landing.is_flaring()) {
            is_flying_bool = false;
        }
    }

    // Apply low-pass filter to flying determination to create probabilistic confidence value
    // When impact detected, only allow probability to decrease (prevents bouncing back to "flying")
    if (!crash_state.impact_detected || !is_flying_bool) {
        // Low-pass filter: y(n) = 0.85*y(n-1) + 0.15*x(n)
        // At 5Hz update rate, coefficient 0.15 gives time constant of ~3 seconds
        // Takes 3.0s to go from 100% down to 10% (or 0% up to 90%)
        // This prevents rapid state oscillations during brief sensor dropouts
        isFlyingProbability = (0.85f * isFlyingProbability) + (0.15f * (float)is_flying_bool);
    }

    /*
      update last_flying_ms so we always know how long we have not
      been flying for. This helps for crash detection and auto-disarm
     */
    bool new_is_flying = is_flying();

    // we are flying, note the time
    if (new_is_flying) {

        auto_state.last_flying_ms = now_ms;

        if (!previous_is_flying) {
            // just started flying in any mode
            started_flying_ms = now_ms;
        }

        if ((control_mode == &mode_auto) &&
            ((auto_state.started_flying_in_auto_ms == 0) || !previous_is_flying) ) {

            // We just started flying, note that time also
            auto_state.started_flying_in_auto_ms = now_ms;
        }
    }
    previous_is_flying = new_is_flying;
#if HAL_ADSB_ENABLED
    adsb.set_is_flying(new_is_flying);
#endif
#if HAL_PARACHUTE_ENABLED
    parachute.set_is_flying(new_is_flying);
#endif
#if AP_STATS_ENABLED
    AP::stats()->set_flying(new_is_flying);
#endif
    AP_Notify::flags.flying = new_is_flying;

    crash_detection_update();

#if HAL_LOGGING_ENABLED
    Log_Write_Status();
#endif

    // tell AHRS flying state
    set_likely_flying(new_is_flying);

    // conservative ground mode value for rate D suppression
    ground_mode = !is_flying() && !arming.is_armed_and_safety_off();
}

/**
 * @brief Determine if the aircraft is currently airborne
 * 
 * @details This is the main query interface for flight state, used throughout
 *          ArduPlane code to determine if the aircraft is flying. The determination
 *          is based on a probabilistic confidence value (isFlyingProbability) that
 *          is continuously updated by update_is_flying_5Hz().
 * 
 *          Thresholds vary based on arming state to reflect different assumptions:
 *          
 *          ARMED + Safety Off:
 *          - Returns true if isFlyingProbability >= 0.1 (10% confidence)
 *          - Conservative threshold: Assume flying unless proven otherwise
 *          - Prevents inadvertent disarm or mode changes during flight
 *          - Quadplane: Returns true immediately if in VTOL flying mode
 *          
 *          DISARMED or Safety On:
 *          - Returns true only if isFlyingProbability >= 0.9 (90% confidence)
 *          - High threshold: Assume grounded unless clearly flying
 *          - Prevents false flying state from bench testing or wind gusts
 * 
 *          The asymmetric thresholds (0.1 armed vs 0.9 disarmed) provide safe
 *          behavior: When armed, err on the side of "flying" to prevent unsafe
 *          actions; when disarmed, err on the side of "not flying" to avoid
 *          false alarms.
 * 
 * @return true if aircraft is determined to be airborne, false if on ground
 * 
 * @note This is a probabilistic estimate that requires careful use. Each use
 *       case should consider the implications of false positives vs false negatives.
 *       
 * @note Common uses:
 *       - Crash detection logic (requires sustained not-flying when armed)
 *       - Auto-disarm after landing
 *       - Mode transition guards
 *       - Logging flight time
 *       - Failsafe behavior selection
 * 
 * @warning Do not use for critical control decisions that require immediate response.
 *          The probabilistic filter has ~3 second time constant, so state changes
 *          are not instantaneous.
 * 
 * @see update_is_flying_5Hz() for the detection algorithm
 * @see quadplane.is_flying_vtol() for quadplane-specific detection
 * 
 * Source: ArduPlane/is_flying.cpp:191-205
 */
bool Plane::is_flying(void)
{
    if (arming.is_armed_and_safety_off()) {
#if HAL_QUADPLANE_ENABLED
        if (quadplane.is_flying_vtol()) {
            return true;
        }
#endif
        // when armed, assume we're flying unless we probably aren't
        return (isFlyingProbability >= 0.1f);
    }

    // when disarmed, assume we're not flying unless we probably are
    return (isFlyingProbability >= 0.9f);
}

/**
 * @brief Detect aircraft crashes and trigger appropriate safety actions
 * 
 * @details This function monitors for crash conditions during AUTO mode when crash
 *          detection is enabled (CRASH_DETECT parameter). It uses multiple indicators
 *          to determine if the aircraft has impacted the ground unexpectedly:
 * 
 *          Detection Methods by Flight Stage:
 *          
 *          1. TAKEOFF Stage:
 *             - Detects failed launches: throttle applied, acceleration threshold met,
 *               but aircraft still not flying after a delay
 *             - Only active if TKOFF_THR_MINACC > 0 and throttle not suppressed
 *          
 *          2. NORMAL Flight Stage:
 *             - Armed and not flying = crash (after 2.5s of auto flight)
 *             - Exception: Pre-launch stage (throttle suppressed, waiting for launch)
 *          
 *          3. LANDING Approach:
 *             - Armed and not flying while on approach = tree/obstacle strike
 *          
 *          4. LANDING Expected Impact (Flare):
 *             - Monitors attitude: Crash if roll or pitch exceeds 60 degrees
 *             - Distinguishes hard landing (within 75m of waypoint) from true crash
 *             - Only checks once to avoid false triggers when handling landed aircraft
 *          
 *          5. VTOL Mode:
 *             - Crash detection disabled (needs different detection method)
 * 
 *          Debouncing:
 *          Uses CRASH_DETECTION_DELAY_MS (500ms) debounce timer to avoid false
 *          positives from turbulence or transient sensor events.
 * 
 *          Safety Actions on Confirmed Crash:
 *          - Sets crash_state.is_crashed flag
 *          - Sends emergency or critical message to GCS:
 *            * "Hard landing detected" if within 75m of landing waypoint
 *            * "Crash detected" for true crashes
 *          - Disarms if CRASH_DETECT_ACTION_BITMASK_DISARM bit set
 * 
 *          GPS/Airspeed Dependency:
 *          Crash detection is disabled if both GPS lock is lost (< 3D fix) AND
 *          airspeed sensor is unavailable or unhealthy, as detection relies on
 *          flying state which requires these sensors.
 * 
 * @note Only active in AUTO mode and requires CRASH_DETECT parameter enabled.
 *       Other modes do not have predictable flight behavior for crash detection.
 * 
 * @note Requires at least 2.5 seconds of auto flight before detection activates
 *       (auto_state.started_flying_in_auto_ms) to avoid false positives during
 *       mode transitions or initial takeoff.
 * 
 * @warning This is safety-critical code that can automatically disarm the aircraft.
 *          False positives during flight are dangerous (unwanted disarm in air).
 *          False negatives are less critical (pilot can manually disarm).
 * 
 * @warning Test crash detection thoroughly in SITL before hardware testing:
 *          - Simulate normal landings (should not trigger)
 *          - Simulate nose-over landings (should trigger crash)
 *          - Simulate tree strikes during approach
 *          - Simulate failed catapult launches
 * 
 * @see is_flying() for the flying state used in crash detection
 * @see crash_state structure for maintained crash detection state
 * 
 * Source: ArduPlane/is_flying.cpp:210-326
 */
void Plane::crash_detection_update(void)
{
    if (control_mode != &mode_auto || !aparm.crash_detection_enable)
    {
        // crash detection is only available in AUTO mode
        crash_state.debounce_timer_ms = 0;
        crash_state.is_crashed = false;
        return;
    }

    uint32_t now_ms = AP_HAL::millis();
    bool crashed_near_land_waypoint = false;
    bool crashed = false;
    bool been_auto_flying = (auto_state.started_flying_in_auto_ms > 0) &&
                            (now_ms - auto_state.started_flying_in_auto_ms >= 2500);

    if (!is_flying() && arming.is_armed())
    {
        if (landing.is_expecting_impact()) {
            // We should be nice and level-ish in this flight stage. If not, we most
            // likely had a crazy landing. Throttle is inhibited already at the flare
            // but go ahead and notify GCS and perform any additional post-crash actions.
            // Declare a crash if we are oriented more that 60deg in pitch or roll
            if (!crash_state.checkedHardLanding && // only check once
                been_auto_flying &&
                (fabsf(ahrs.get_roll_deg()) > 60 || fabsf(ahrs.get_pitch_deg()) > 60)) {
                crashed = true;

                // did we "crash" within 75m of the landing location? Probably just a hard landing
                crashed_near_land_waypoint =
                        current_loc.get_distance(mission.get_current_nav_cmd().content.location) < 75;

                // trigger hard landing event right away, or never again. This inhibits a false hard landing
                // event when, for example, a minute after a good landing you pick the plane up and
                // this logic is still running and detects the plane is on its side as you carry it.
                crash_state.debounce_timer_ms = now_ms;
                crash_state.debounce_time_total_ms = 0; // no debounce
            }

            crash_state.checkedHardLanding = true;

        } else if (landing.is_on_approach()) {
            // when altitude gets low, we automatically flare so ground crashes
            // most likely can not be triggered from here. However,
            // a crash into a tree would be caught here.
            if (been_auto_flying) {
                crashed = true;
                crash_state.debounce_time_total_ms = CRASH_DETECTION_DELAY_MS;
            }

        } else {
            switch (flight_stage)
            {
            case AP_FixedWing::FlightStage::TAKEOFF:
                if (g.takeoff_throttle_min_accel > 0 &&
                        !throttle_suppressed) {
                    // if you have an acceleration holding back throttle, but you met the
                    // accel threshold but still not flying, then you either shook/hit the
                    // plane or it was a failed launch.
                    crashed = true;
                    crash_state.debounce_time_total_ms = CRASH_DETECTION_DELAY_MS;
                }
                // TODO: handle auto missions without NAV_TAKEOFF mission cmd
                break;

            case AP_FixedWing::FlightStage::NORMAL:
                if (!in_preLaunch_flight_stage() && been_auto_flying) {
                    crashed = true;
                    crash_state.debounce_time_total_ms = CRASH_DETECTION_DELAY_MS;
                }
                break;

            case AP_FixedWing::FlightStage::VTOL:
                // we need a totally new method for this
                crashed = false;
                break;

            default:
                break;
            } // switch
        }
    } else {
        crash_state.checkedHardLanding = false;
    }

    // if we have no GPS lock and we don't have a functional airspeed
    // sensor then don't do crash detection
    if (gps.status() < AP_GPS::GPS_OK_FIX_3D) {
#if AP_AIRSPEED_ENABLED
        if (!airspeed.use() || !airspeed.healthy()) {
            crashed = false;
        }
#else
        crashed = false;
#endif
    }

    if (!crashed) {
        // reset timer
        crash_state.debounce_timer_ms = 0;

    } else if (crash_state.debounce_timer_ms == 0) {
        // start timer
        crash_state.debounce_timer_ms = now_ms;

    } else if ((now_ms - crash_state.debounce_timer_ms >= crash_state.debounce_time_total_ms) && !crash_state.is_crashed) {
        crash_state.is_crashed = true;
        if (aparm.crash_detection_enable & CRASH_DETECT_ACTION_BITMASK_DISARM) {
            arming.disarm(AP_Arming::Method::CRASH);
        }
        if (crashed_near_land_waypoint) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Hard landing detected");
        } else {
            gcs().send_text(MAV_SEVERITY_EMERGENCY, "Crash detected");
        }
    }
}

/**
 * @brief Check if aircraft is in pre-launch phase awaiting launch trigger
 * 
 * @details Determines if the aircraft is in the pre-launch state where it's armed
 *          and ready for an automatic launch (typically catapult or bungee) but
 *          has not yet been released. In this state:
 *          - Throttle is suppressed (waiting for launch trigger)
 *          - Flight stage is NORMAL or in TAKEOFF mode
 *          - Currently executing NAV_TAKEOFF mission command
 * 
 *          Pre-launch detection prevents false "flying" state from:
 *          - Uncalibrated airspeed sensors drifting to 7+ m/s on the ground
 *          - Wind gusts causing momentary airspeed readings
 *          - Vibration or movement while being positioned for launch
 * 
 *          Launch Sequence:
 *          1. Pre-launch: Throttle suppressed, waiting for acceleration trigger
 *          2. Launch detected: Acceleration exceeds TKOFF_THR_MINACC threshold
 *          3. Throttle enabled: Motor spools up
 *          4. Flying detected: Airspeed and/or GPS speed exceed thresholds
 * 
 * @return true if in pre-launch phase (armed, waiting for launch trigger)
 *         false if not in pre-launch (either not waiting, or already launched)
 * 
 * @note Pre-launch phase is specific to automatic launches with acceleration-based
 *       triggering. Hand launches and runway takeoffs do not use this state.
 * 
 * @note For quadplane VTOL takeoffs (NAV_VTOL_TAKEOFF), pre-launch state is
 *       not applicable as VTOL takeoffs do not use throttle suppression.
 * 
 * @note This function is called by update_is_flying_5Hz() to ensure the aircraft
 *       is not incorrectly marked as flying while sitting on the launcher.
 * 
 * @see update_is_flying_5Hz() where this is used to prevent false flying state
 * @see Plane::set_throttle_suppressed() for throttle suppression control
 * @see TKOFF_THR_MINACC parameter for launch acceleration threshold
 * 
 * Source: ArduPlane/is_flying.cpp:331-345
 */
bool Plane::in_preLaunch_flight_stage(void)
{
    if (control_mode == &mode_takeoff && throttle_suppressed) {
        return true;
    }
#if HAL_QUADPLANE_ENABLED
    if (quadplane.is_vtol_takeoff(mission.get_current_nav_cmd().id)) {
        return false;
    }
#endif
    return (control_mode == &mode_auto &&
            throttle_suppressed &&
            flight_stage == AP_FixedWing::FlightStage::NORMAL &&
            mission.get_current_nav_cmd().id == MAV_CMD_NAV_TAKEOFF);
}
