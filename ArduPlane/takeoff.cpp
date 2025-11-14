/**
 * @file takeoff.cpp
 * @brief Fixed-wing aircraft takeoff sequence management and control
 * 
 * @details This file implements the complete takeoff state machine for ArduPlane,
 *          managing all phases of a fixed-wing takeoff from ground roll through
 *          climb to cruise altitude. The takeoff system handles multiple launch
 *          methods including:
 *          - Ground takeoffs with acceleration-based detection
 *          - Hand launches with attitude monitoring
 *          - Catapult launches with multi-pulse detection
 *          - Tail-dragger aircraft with elevator hold phases
 * 
 *          Takeoff State Machine Phases:
 *          1. PRE-LAUNCH: Armed and waiting for takeoff trigger
 *             - Monitor GPS lock and attitude
 *             - Wait for rudder neutral (if rudder-armed)
 *             - Detect forward acceleration threshold
 *          
 *          2. GROUND ROLL: Acceleration detected, building speed
 *             - Apply full throttle or configured takeoff throttle
 *             - Maintain runway heading with limited roll
 *             - Hold pitch attitude for tail-draggers (TKOFF_TDRAG_ELEV)
 *             - Monitor ground speed vs TKOFF_THR_MINSPD
 *          
 *          3. ROTATION: Reached rotate speed (TKOFF_ROTATE_SPD)
 *             - Transition from ground pitch to climb pitch
 *             - Gradually increase pitch as airspeed builds
 *             - Expand roll limits as altitude increases
 *             - Complete rotation when reaching cruise speed
 *          
 *          4. INITIAL CLIMB: Climbing to TKOFF_LVL_ALT
 *             - Maintain takeoff pitch angle (TKOFF_PITCH_CD)
 *             - Use full takeoff throttle (TKOFF_THR_MAX)
 *             - Limited roll angles to prevent wing strike
 *             - Monitor for stall prevention
 *          
 *          5. LEVEL-OFF: Approaching target altitude
 *             - Gradually reduce pitch to level flight
 *             - Transition throttle to cruise settings
 *             - Expand to full roll authority
 *             - Complete when reaching takeoff altitude
 * 
 *          Safety Features:
 *          - Bad attitude detection (excessive pitch/roll during launch)
 *          - Takeoff timeout monitoring (TKOFF_TIMEOUT)
 *          - Rudder neutral requirement after rudder arming
 *          - Multi-pulse acceleration detection for catapult launches
 *          - Hand-launch roll recovery (reduces pitch if roll error > 90°)
 *          - Ground speed verification before committing to takeoff
 * 
 *          Key Parameters:
 *          - TKOFF_THR_MINACC: Minimum acceleration to detect takeoff (m/s²)
 *          - TKOFF_THR_DELAY: Delay from acceleration to throttle (0.1s units)
 *          - TKOFF_THR_MINSPD: Minimum GPS speed to complete detection (m/s)
 *          - TKOFF_ROTATE_SPD: Airspeed for rotation phase (m/s)
 *          - TKOFF_GND_PITCH: Pitch angle during ground roll (degrees)
 *          - TKOFF_PITCH_CD: Target climb pitch angle (centidegrees)
 *          - TKOFF_LVL_ALT: Altitude to begin leveling off (meters)
 *          - TKOFF_THR_MAX: Maximum throttle during takeoff (0-100%)
 * 
 * @note This implementation relies on TECS 50Hz processing for acceleration measurement
 * @warning Improper takeoff configuration can result in stalls, crashes, or failed launches
 * 
 * @see Plane::auto_takeoff_check() - Main takeoff detection function
 * @see Plane::takeoff_calc_pitch() - Pitch angle calculation through takeoff phases
 * @see Plane::takeoff_calc_roll() - Roll angle limiting during takeoff
 * @see Plane::takeoff_calc_throttle() - Throttle management during takeoff
 */

#include "Plane.h"

/**
 * @brief Monitor and detect automatic takeoff trigger conditions
 * 
 * @details This function implements the core takeoff detection state machine that
 *          determines when an aircraft has successfully begun a takeoff sequence.
 *          It monitors multiple sensors and validates launch conditions before
 *          transitioning from ground roll to flight control.
 * 
 *          Detection Sequence (State Machine):
 *          1) PRE-FLIGHT CHECKS: Verify prerequisites for takeoff
 *             - Require armed state with safety off
 *             - Verify GPS 3D fix (GPS_OK_FIX_3D or better)
 *             - Check rudder neutral if rudder-armed
 *             - Reset state if process interrupted (>200ms gap)
 *          
 *          2) ACCELERATION DETECTION: Wait for forward acceleration
 *             - Monitor longitudinal acceleration from TECS (VXdot)
 *             - Compare against TKOFF_THR_MINACC threshold
 *             - Support multi-pulse detection for catapult (TKOFF_THR_SLEW)
 *             - Start launch timer when acceleration threshold met
 *          
 *          3) DELAY TIMER: Wait for configured delay period
 *             - Timer duration = TKOFF_THR_DELAY * 0.1 seconds
 *             - Allows aircraft to build speed before final check
 *             - Timeout after 12.7 seconds maximum
 *          
 *          4) SPEED AND ATTITUDE VALIDATION: Final launch confirmation
 *             - Verify GPS ground speed > TKOFF_THR_MINSPD
 *             - Check attitude within safe limits:
 *               * Pitch: -30° to +45° (prevents tail strike or stall)
 *               * Roll: ±30° (prevents wing strike)
 *             - Wait for timer to reach TKOFF_THR_DELAY
 *          
 *          5) TAKEOFF COMMITTED: All conditions satisfied
 *             - Return true to commit to takeoff
 *             - Initialize takeoff state variables
 *             - Lock course heading for initial climb
 *             - Start takeoff throttle and altitude tracking
 * 
 *          Multi-Pulse Catapult Detection:
 *          When TKOFF_THR_SLEW > 1, system requires alternating acceleration
 *          events (positive/negative) to distinguish catapult launch from
 *          taxi or wind gust. Each pulse must exceed TKOFF_THR_MINACC within
 *          500ms of previous pulse.
 * 
 *          Hand-Launch Support:
 *          - Attitude checks disabled for tailsitters via quadplane detection
 *          - Attitude checks optionally disabled via TKOFF_OPTIONS bit
 *          - Allows aggressive launch angles for hand-thrown aircraft
 * 
 *          Rudder-Arm Safety:
 *          If armed via rudder stick, system waits for rudder return to neutral
 *          before allowing takeoff. Prevents unintended takeoff with full rudder
 *          deflection. Warning message sent every 30 seconds while waiting.
 * 
 * @return true if all takeoff conditions met and takeoff should begin
 * @return false if waiting for conditions or conditions failed
 * 
 * @note Called at main loop rate (typically 50Hz) during AUTO, GUIDED, or TAKEOFF modes
 * @note Acceleration measurement provided by TECS at 50Hz update rate
 * @note State preserved across calls in takeoff_state structure
 * 
 * @warning Returning true commits aircraft to takeoff - all conditions must be validated
 * @warning Bad attitude detection will abort launch and reset state
 * @warning Timeout after 2.5 seconds will abort launch attempt
 * 
 * @see takeoff_state - Persistent state structure for takeoff detection
 * @see TECS_controller.get_VXdot() - Longitudinal acceleration measurement
 * @see handle_auto_mode() - Main AUTO mode handler that calls this function
 */
bool Plane::auto_takeoff_check(void)
{
    // This is a more advanced check that relies on TECS 50Hz acceleration measurement
    uint32_t now = millis();
    // Convert TKOFF_THR_DELAY (0.1s units) to milliseconds, capped at 12.7 seconds
    uint16_t wait_time_ms = MIN(uint16_t(g.takeoff_throttle_delay)*100,12700);

    // PHASE 1: Pre-flight validation - reset all takeoff state if disarmed
    if (!arming.is_armed_and_safety_off()) {
        memset(&takeoff_state, 0, sizeof(takeoff_state));
        auto_state.baro_takeoff_alt = barometer.get_altitude();
        return false;
    }

    // Reset states if process has been interrupted (>200ms gap between checks)
    // This detects loss of continuity in the state machine, except initial_direction.initialized if set
#if MODE_AUTOLAND_ENABLED
    bool takeoff_dir_initialized = takeoff_state.initial_direction.initialized;
    float takeoff_dir = takeoff_state.initial_direction.heading;
#endif
     if (takeoff_state.last_check_ms && (now - takeoff_state.last_check_ms) > 200) {
         memset(&takeoff_state, 0, sizeof(takeoff_state));
#if MODE_AUTOLAND_ENABLED
         takeoff_state.initial_direction.initialized = takeoff_dir_initialized; //restore dir init state
         takeoff_state.initial_direction.heading = takeoff_dir;
#endif
         return false;
     }
    takeoff_state.last_check_ms = now;
    
    // RUDDER-ARM SAFETY: Check if waiting for rudder neutral after rudder arm
    // Prevents takeoff with rudder deflection which could cause ground loop
    if (plane.arming.last_arm_method() == AP_Arming::Method::RUDDER &&
        !rc().seen_neutral_rudder()) {
        // we were armed with rudder but have not seen rudder neutral yet
        takeoff_state.waiting_for_rudder_neutral = true;
        // warn if we have been waiting a long time
        if (now - takeoff_state.rudder_takeoff_warn_ms > TAKEOFF_RUDDER_WARNING_TIMEOUT) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Takeoff waiting for rudder release");
            takeoff_state.rudder_takeoff_warn_ms = now;
        }
        // since we are still waiting, dont takeoff
        return false;
    } else {
       // we did not arm by rudder or rudder has returned to neutral
       // make sure we dont indicate we are in the waiting state with servo position indicator
       takeoff_state.waiting_for_rudder_neutral = false;
    }  

    // PHASE 2: GPS validation - require 3D fix for accurate speed and position
    if (gps.status() < AP_GPS::GPS_OK_FIX_3D) {
        // No auto takeoff without GPS lock - need accurate ground speed measurement
        return false;
    }

    // Determine if attitude checks should be performed (disabled for hand-launch or tailsitters)
    bool do_takeoff_attitude_check = !(flight_option_enabled(FlightOptions::DISABLE_TOFF_ATTITUDE_CHK));
#if HAL_QUADPLANE_ENABLED
    // Disable attitude check on tailsitters - they launch vertically with extreme attitudes
    do_takeoff_attitude_check &= !quadplane.tailsitter.enabled();
#endif

    // PHASE 3: ACCELERATION DETECTION - Monitor for forward acceleration indicating launch
    if (!takeoff_state.launchTimerStarted && !is_zero(g.takeoff_throttle_min_accel)) {
        // We are requiring a longitudinal acceleration event to detect launch start
        // Get longitudinal (forward/back) acceleration from TECS in m/s²
        float xaccel = TECS_controller.get_VXdot();
        if (g2.takeoff_throttle_accel_count <= 1) {
            // Single-pulse detection: Simple forward acceleration threshold
            if (xaccel < g.takeoff_throttle_min_accel) {
                goto no_launch;
            }
        } else {
            // Multi-pulse detection for catapult launch: Require alternating accel events
            // Reset counter if pulses are too far apart (>500ms indicates non-catapult event)
            if (now - takeoff_state.accel_event_ms > 500) {
                takeoff_state.accel_event_counter = 0;
            }
            // Alternate between looking for positive and negative acceleration pulses
            bool odd_event = ((takeoff_state.accel_event_counter & 1) != 0);
            // Odd events need negative accel (deceleration), even events need positive accel
            bool got_event = (odd_event?xaccel < -g.takeoff_throttle_min_accel : xaccel > g.takeoff_throttle_min_accel);
            if (got_event) {
                takeoff_state.accel_event_counter++;
                takeoff_state.accel_event_ms = now;
            }
            if (takeoff_state.accel_event_counter < g2.takeoff_throttle_accel_count) {
                goto no_launch;
            }
        }
    }

    // PHASE 4: DELAY TIMER - Acceleration detected, start delay timer to allow speed buildup
    if (!takeoff_state.launchTimerStarted) {
        takeoff_state.launchTimerStarted = true;
        takeoff_state.last_tkoff_arm_time = now;
        if (now - takeoff_state.last_report_ms > 2000) {
            gcs().send_text(MAV_SEVERITY_INFO, "Armed AUTO, xaccel = %.1f m/s/s, waiting %.1f sec",
                              (double)TECS_controller.get_VXdot(), (double)(wait_time_ms*0.001f));
            takeoff_state.last_report_ms = now;
        }
    }

    // Only perform velocity check if not timed out
    if ((now - takeoff_state.last_tkoff_arm_time) > wait_time_ms+100U) {
        if (now - takeoff_state.last_report_ms > 2000) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Timeout AUTO");
            takeoff_state.last_report_ms = now;
        }
        goto no_launch;
    }

    // ATTITUDE SAFETY CHECK: Verify aircraft attitude is within safe launch limits
    if (do_takeoff_attitude_check) {
        // Check aircraft attitude for bad launch (nose down, tail strike, or wing strike)
        // Pitch limits: -30° to +45° (centidegrees: -3000 to 4500)
        // Roll limits: ±30° for upright flight (centidegrees: ±3000)
        if (ahrs.pitch_sensor <= -3000 || ahrs.pitch_sensor >= 4500 ||
            (!fly_inverted() && labs(ahrs.roll_sensor) > 3000)) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Bad launch AUTO");
            takeoff_state.accel_event_counter = 0;
            goto no_launch;
        }
    }

    // PHASE 5: FINAL VALIDATION - Check ground speed and time delay before committing to takeoff
    if (((gps.ground_speed() > g.takeoff_throttle_min_speed || is_zero(g.takeoff_throttle_min_speed))) &&
        ((now - takeoff_state.last_tkoff_arm_time) >= wait_time_ms)) {
        // ALL CONDITIONS MET - COMMIT TO TAKEOFF
        gcs().send_text(MAV_SEVERITY_INFO, "Triggered AUTO. GPS speed = %.1f", (double)gps.ground_speed());
        // Reset detection state
        takeoff_state.launchTimerStarted = false;
        takeoff_state.last_tkoff_arm_time = 0;
        // Initialize takeoff phase tracking
        takeoff_state.start_time_ms = now;
        takeoff_state.level_off_start_time_ms = 0;
        takeoff_state.throttle_max_timer_ms = now;
        // Lock current heading as initial takeoff course
        steer_state.locked_course_err = 0; // use current heading without any error offset
        return true;
    }

    // Still in delay phase - timer running but conditions not yet met
    return false;

no_launch:
    // Takeoff conditions failed - reset state and return false
    takeoff_state.launchTimerStarted = false;
    takeoff_state.last_tkoff_arm_time = 0;
    return false;
}

/**
 * @brief Calculate desired bank angle during takeoff with progressive roll limit expansion
 * 
 * @details Computes the target roll angle for takeoff phases while applying
 *          progressively relaxing roll limits to prevent wing strikes during
 *          ground roll and initial climb. Roll authority is restricted during
 *          ground operations and gradually expanded as altitude increases.
 * 
 *          Roll Limit Strategy:
 *          1. GROUND ROLL (airspeed < TKOFF_ROTATE_SPD):
 *             - Roll limited to LEVEL_ROLL_LIMIT (typically 5°)
 *             - Prevents wing tip contact with runway
 *             - Maintains runway heading with minimal bank
 *          
 *          2. INITIAL CLIMB (altitude < TKOFF_LVL_ALT):
 *             - Roll limited to LEVEL_ROLL_LIMIT
 *             - Protects against wing strike during climbout
 *             - Maintains predictable climb path
 *          
 *          3. TRANSITION PHASE (TKOFF_LVL_ALT to TKOFF_LVL_ALT*3):
 *             - Roll limit linearly interpolated
 *             - From LEVEL_ROLL_LIMIT to full ROLL_LIMIT_DEG
 *             - Smooth expansion of roll authority
 *          
 *          4. ESTABLISHED CLIMB (altitude > TKOFF_LVL_ALT*3):
 *             - Full roll authority (ROLL_LIMIT_DEG)
 *             - Normal flight envelope available
 *             - Complete maneuverability restored
 * 
 *          Heading Control:
 *          - If no heading locked (steer_state.hold_course_cd == -1):
 *            Wings held level until GPS heading available
 *          - Once heading established: Standard L1 navigation with limited roll
 * 
 * @param[out] nav_roll_cd Target roll angle in centidegrees (set as side effect)
 * 
 * @note Called from flight mode update loops during takeoff phase
 * @note Roll limits scale reasonably for both small and large aircraft
 * @note The *3 altitude scheme provides smooth transition for various climb rates
 * 
 * @see calc_nav_roll() - Underlying navigation roll calculation
 * @see LEVEL_ROLL_LIMIT - Maximum roll during ground roll (parameter)
 * @see ROLL_LIMIT_DEG - Normal flight roll limit (parameter)
 */
void Plane::takeoff_calc_roll(void)
{
    // Special case: No heading established yet (insufficient GPS speed)
    if (steer_state.hold_course_cd == -1) {
        // We don't yet have a heading to hold - just level the wings
        // until we get up enough speed to get a GPS heading
        nav_roll_cd = 0;
        return;
    }

    // Calculate desired roll from L1 navigation controller
    calc_nav_roll();

    // Apply takeoff-specific roll limits to prevent wing strike
    // During takeoff use the level flight roll limit to prevent large bank angles.
    // Slowly allow for more roll as we get higher above the takeoff altitude
    int32_t takeoff_roll_limit_cd = roll_limit_cd;

    if (auto_state.highest_airspeed < g.takeoff_rotate_speed) {
        // GROUND ROLL PHASE: Before Vrotate (still on the ground)
        // Use minimal roll to prevent wingtip strike on runway
        takeoff_roll_limit_cd = g.level_roll_limit * 100;
    } else {
        // CLIMB PHASE: Progressively expand roll authority with altitude
        // lim1 - below altitude TKOFF_LVL_ALT, restrict roll to LEVEL_ROLL_LIMIT
        // lim2 - above altitude (TKOFF_LVL_ALT * 3) allow full flight envelope of ROLL_LIMIT_DEG
        // In between lim1 and lim2 use a scaled roll limit.
        // The *3 scheme should scale reasonably with both small and large aircraft
        const float lim1 = MAX(mode_takeoff.level_alt, 0);
        const float lim2 = MIN(mode_takeoff.level_alt*3, mode_takeoff.target_alt);
        const float current_baro_alt = barometer.get_altitude();

        // Linear interpolation from LEVEL_ROLL_LIMIT to full ROLL_LIMIT_DEG
        takeoff_roll_limit_cd = linear_interpolate(g.level_roll_limit*100, roll_limit_cd,
                                        current_baro_alt,
                                        auto_state.baro_takeoff_alt+lim1, auto_state.baro_takeoff_alt+lim2);
    }

    // Apply the computed roll limit (symmetric positive and negative)
    nav_roll_cd = constrain_int32(nav_roll_cd, -takeoff_roll_limit_cd, takeoff_roll_limit_cd);
}

        
/**
 * @brief Calculate desired pitch angle through takeoff rotation and climb phases
 * 
 * @details Manages pitch angle progression through the complete takeoff sequence,
 *          implementing a multi-phase rotation strategy that transitions from
 *          ground pitch through rotation to established climb pitch. This function
 *          is critical for safe takeoff performance and stall prevention.
 * 
 *          ROTATION PHASE SEQUENCE (when TKOFF_ROTATE_SPD configured):
 *          
 *          1. GROUND ROLL (airspeed < TKOFF_ROTATE_SPD):
 *             - Hold fixed pitch: TKOFF_GND_PITCH
 *             - Typically 0-3° for conventional gear, negative for tail-draggers
 *             - Prevents premature rotation and maintains ground contact
 *             - TECS pitch min/max locked to ground pitch value
 *          
 *          2. ROTATION (TKOFF_ROTATE_SPD to cruise speed):
 *             - Gradually increase pitch from ground to climb angle
 *             - Pitch scales linearly with ground speed / cruise speed
 *             - Minimum pitch floor of 5° to ensure positive climb
 *             - Prevents aggressive rotation that could cause tail strike or stall
 *             - TECS pitch limits track with transitioning pitch target
 *          
 *          3. ESTABLISHED CLIMB (ground speed > cruise speed):
 *             - Set rotation_complete flag
 *             - Transition to standard climb pitch control
 *             - TECS manages pitch for target climb rate
 *          
 *          POST-ROTATION PITCH CONTROL:
 *          
 *          With Airspeed Sensor:
 *          - Use TECS-calculated pitch for energy management
 *          - Enforce minimum pitch: get_takeoff_pitch_min_cd()
 *          - TECS balances altitude and airspeed targets
 *          
 *          Without Airspeed Sensor:
 *          - Use fixed minimum pitch angle
 *          - Pitch-only altitude control (no airspeed feedback)
 *          - More conservative to prevent stall
 *          
 *          HAND-LAUNCH ROLL RECOVERY:
 *          When STALL_PREVENTION enabled and roll error exceeds limits:
 *          - Reduce pitch demand proportional to roll error
 *          - Helps recover from off-axis hand launches
 *          - Pitch reduction = pitch * cos²(roll_error)
 *          - Prioritizes roll recovery over climb rate
 *          - Maximum roll error considered: 90° (9000 centidegrees)
 * 
 * @param[out] nav_pitch_cd Target pitch angle in centidegrees (set as side effect)
 * 
 * @note Called from flight mode update loops during takeoff phase
 * @note Rotation speed is recommended for ground takeoffs but optional for hand-launch
 * @note Roll recovery feature critical for hand-launch robustness
 * 
 * @warning Aggressive pitch during rotation can cause tail strike or departure stall
 * @warning Without airspeed sensor, stall protection is limited to fixed pitch
 * 
 * @see get_takeoff_pitch_min_cd() - Dynamic pitch minimum with level-off logic
 * @see TECS_controller.set_pitch_min() - TECS pitch limit configuration
 * @see calc_nav_pitch() - Standard TECS pitch calculation
 */
void Plane::takeoff_calc_pitch(void)
{
    // ROTATION PHASE MANAGEMENT: Implement controlled pitch progression from ground to climb
    // First see if TKOFF_ROTATE_SPD applies (rotation speed configured for ground takeoffs)
    // This will set the pitch for the first portion of the takeoff, up until cruise speed is reached.
    if (!auto_state.rotation_complete && g.takeoff_rotate_speed > 0) {
        // A non-zero rotate speed is recommended for ground takeoffs to prevent premature liftoff.
        if (auto_state.highest_airspeed < g.takeoff_rotate_speed) {
            // GROUND ROLL: We have not reached rotate speed, hold ground pitch angle
            // Use the specified takeoff target pitch angle (typically 0-3° for tricycle gear)
            nav_pitch_cd = int32_t(100.0f * mode_takeoff.ground_pitch);
            // Lock TECS to this pitch - no deviation allowed during ground roll
            TECS_controller.set_pitch_min(0.01f*nav_pitch_cd);
            TECS_controller.set_pitch_max(0.01f*nav_pitch_cd);
            return;
        } else if (gps.ground_speed() <= (float)aparm.airspeed_cruise) {
            // ROTATION: Between Vrotate and cruise speed - gradually increase pitch
            // If rotate speed applied, gradually transition from TKOFF_GND_PITCH to the climb angle.
            // This is recommended for ground takeoffs, so delay rotation until ground speed indicates adequate airspeed.
            const uint16_t min_pitch_cd = 500; // Set a minimum of 5 deg climb angle to ensure positive climb
            // Linear scaling: pitch increases proportionally with speed from rotate to cruise
            nav_pitch_cd = (gps.ground_speed() / (float)aparm.airspeed_cruise) * auto_state.takeoff_pitch_cd;
            nav_pitch_cd = constrain_int32(nav_pitch_cd, min_pitch_cd, auto_state.takeoff_pitch_cd); 
            // Lock TECS to follow our rotation schedule
            TECS_controller.set_pitch_min(0.01f*nav_pitch_cd);
            TECS_controller.set_pitch_max(0.01f*nav_pitch_cd);
            return;
        }
    }
    // Mark rotation complete - transition to normal climb pitch control
    auto_state.rotation_complete = true;

    // POST-ROTATION CLIMB: We are now past the rotation phase, in established climb
    // Initialize pitch limits for TECS energy management
    int16_t pitch_min_cd = get_takeoff_pitch_min_cd();
    bool pitch_clipped_max = false;

    // PITCH CONTROL STRATEGY: Depends on airspeed sensor availability
    if (ahrs.using_airspeed_sensor()) {
        // WITH AIRSPEED SENSOR: Use TECS for optimal energy management
        // TECS balances altitude gain and airspeed maintenance
        calc_nav_pitch();
        // At any rate, we don't want to go lower than the minimum pitch bound
        // (prevents stall if TECS requests low pitch)
        if (nav_pitch_cd < pitch_min_cd) {
            nav_pitch_cd = pitch_min_cd;
        }
    } else {
        // WITHOUT AIRSPEED SENSOR: Use fixed minimum pitch for safety
        // Cannot optimize energy - use conservative fixed pitch
        nav_pitch_cd = pitch_min_cd;

        // Indicate pitch is externally clamped (not from TECS optimization)
        pitch_clipped_max = true;
    }

    // HAND-LAUNCH ROLL RECOVERY: Reduce pitch if roll control is significantly off
    // Check if we have trouble with roll control during hand launch
    if (aparm.stall_prevention != 0) {
        // During takeoff we want to prioritise roll control over pitch.
        // Apply a reduction in pitch demand if our roll is significantly off.
        // The aim of this change is to increase the robustness of hand launches,
        // particularly in cross-winds. If we start to roll over then we reduce
        // pitch demand until the roll recovers, preventing departure stall.
        
        // Calculate roll error in radians, capped at 90° (9000 centidegrees)
        float roll_error_rad = cd_to_rad(constrain_float(labs(nav_roll_cd - ahrs.roll_sensor), 0, 9000));
        // Reduction factor: cos²(roll_error) - ranges from 1.0 (no error) to 0.0 (90° error)
        float reduction = sq(cosf(roll_error_rad));
        // Apply reduction to pitch demand - at 90° roll error, pitch goes to zero
        nav_pitch_cd *= reduction;

        // Update pitch minimum if reduction brought us below it
        if (nav_pitch_cd < pitch_min_cd) {
            pitch_min_cd = nav_pitch_cd;
        }
    }
    // Notify TECS about the external pitch setting for the next iteration
    TECS_controller.set_pitch_min(0.01f*pitch_min_cd);
    if (pitch_clipped_max) {TECS_controller.set_pitch_max(0.01f*nav_pitch_cd);}
}

/**
 * @brief Calculate throttle limits for takeoff phases with optional throttle range management
 * 
 * @details Computes minimum and maximum throttle bounds during takeoff, supporting
 *          both traditional full-throttle takeoffs and advanced throttle range
 *          management for improved TECS energy control. These limits are applied
 *          by Plane::apply_throttle_limits() to constrain TECS throttle output.
 * 
 *          TRADITIONAL TAKEOFF THROTTLE (default):
 *          - Minimum throttle = Maximum throttle (forced full power)
 *          - Simple, predictable, robust for all conditions
 *          - Recommended for most operations
 *          - Throttle set to TKOFF_THR_MAX (or THR_MAX if TKOFF_THR_MAX=0)
 *          
 *          ADVANCED THROTTLE RANGE (TKOFF_OPTIONS bit set):
 *          Enables TECS to modulate throttle during climbout:
 *          - Below TKOFF_LVL_ALT: Full throttle (traditional)
 *          - Above TKOFF_LVL_ALT with airspeed sensor: Throttle range enabled
 *            * Minimum: TKOFF_THR_MIN (or THR_CRUISE if TKOFF_THR_MIN=0)
 *            * Maximum: TKOFF_THR_MAX (or THR_MAX if TKOFF_THR_MAX=0)
 *          - Allows TECS to manage energy via throttle modulation
 *          - Can improve altitude and speed tracking
 *          - Requires properly tuned TECS for safe operation
 *          
 *          THROTTLE MAX TIMER (TKOFF_THR_MAX_T):
 *          When throttle_max_timer_ms is set (by takeoff trigger):
 *          - Forces full throttle for TKOFF_THR_MAX_T seconds
 *          - Ensures strong initial acceleration
 *          - Timer auto-resets after expiration
 *          - Overrides throttle range setting during timer period
 * 
 *          Parameter Selection:
 *          - TKOFF_THR_MAX: Maximum throttle during takeoff (0-100%, 0=use THR_MAX)
 *          - TKOFF_THR_MIN: Minimum throttle with range enabled (0-100%, 0=use THR_CRUISE)
 *          - TKOFF_THR_MAX_T: Duration to hold max throttle after trigger (seconds)
 *          - TKOFF_OPTIONS: Bit mask, includes THROTTLE_RANGE enable bit
 *          - TKOFF_LVL_ALT: Altitude threshold for throttle range activation (meters)
 * 
 * @param[out] takeoff_state.throttle_lim_min Minimum throttle limit (set as side effect)
 * @param[out] takeoff_state.throttle_lim_max Maximum throttle limit (set as side effect)
 * 
 * @note Limits stored in takeoff_state structure for use by apply_throttle_limits()
 * @note Calls calc_throttle() to compute base throttle from TECS
 * @note Throttle range feature requires airspeed sensor for safe operation
 * 
 * @warning Throttle range mode can cause underspeed stalls if TECS poorly tuned
 * @warning Always test throttle range mode at safe altitude before takeoff use
 * 
 * @see Plane::apply_throttle_limits() - Applies computed limits to throttle output
 * @see tkoff_option_is_set() - Check if THROTTLE_RANGE option enabled
 * @see calc_throttle() - TECS throttle calculation
 */
void Plane::takeoff_calc_throttle() {
    // STEP 1: Initialize the maximum throttle limit from parameters
    if (aparm.takeoff_throttle_max != 0) {
        // Use takeoff-specific throttle maximum
        takeoff_state.throttle_lim_max = aparm.takeoff_throttle_max;
    } else {
        // Fall back to general maximum throttle setting
        takeoff_state.throttle_lim_max = aparm.throttle_max;
    }

    // STEP 2: Initialize the minimum throttle limit from parameters
    // (only used if throttle range mode enabled, otherwise forced to max)
    if (aparm.takeoff_throttle_min != 0) {
        // Use takeoff-specific throttle minimum
        takeoff_state.throttle_lim_min = aparm.takeoff_throttle_min;
    } else {
        // Fall back to cruise throttle as minimum
        takeoff_state.throttle_lim_min = aparm.throttle_cruise;
    }

    // STEP 3: THROTTLE MAX TIMER - Force max throttle for TKOFF_THR_MAX_T seconds
    // Raise min to force max throttle for TKOFF_THR_MAX_T after a takeoff.
    // It only applies if the timer has been started externally (by takeoff trigger).
    if (takeoff_state.throttle_max_timer_ms != 0) {
        const uint32_t dt = AP_HAL::millis() - takeoff_state.throttle_max_timer_ms;
        if (dt*0.001 < aparm.takeoff_throttle_max_t) {
            // Still within max throttle timer period - force full power
            takeoff_state.throttle_lim_min = takeoff_state.throttle_lim_max;
        } else {
            // Timer expired - reset the timer for future use
            takeoff_state.throttle_max_timer_ms = 0;
        }
    }

    // STEP 4: THROTTLE RANGE LOGIC - Determine if TECS can modulate throttle
    // Enact the TKOFF_OPTIONS logic for throttle range control
    const float current_baro_alt = barometer.get_altitude();
    const bool below_lvl_alt = current_baro_alt < auto_state.baro_takeoff_alt + mode_takeoff.level_alt;
    // Check if throttle range option is enabled
    const bool use_throttle_range = tkoff_option_is_set(AP_FixedWing::TakeoffOption::THROTTLE_RANGE);
    if (!use_throttle_range // We don't want to employ a throttle range (traditional mode)
        || !ahrs.using_airspeed_sensor() // We don't have an airspeed sensor (unsafe for range mode)
        || below_lvl_alt // We are below TKOFF_LVL_ALT (force full throttle during initial climb)
        ) { 
        // TRADITIONAL TAKEOFF: Force full throttle by setting min = max
        takeoff_state.throttle_lim_min = takeoff_state.throttle_lim_max;
    }
    // Otherwise: Advanced mode with throttle range - limits already set above

    calc_throttle();
}

/**
 * @brief Calculate minimum pitch angle during takeoff with dynamic level-off logic
 * 
 * @details Returns the minimum allowable pitch angle during takeoff climb,
 *          implementing a dynamic level-off sequence as the aircraft approaches
 *          target altitude. The pitch minimum progressively decreases to facilitate
 *          smooth transition from climb to level flight.
 * 
 *          PITCH MINIMUM STRATEGIES:
 *          
 *          1. STANDARD CLIMB (far from target altitude):
 *             - Return full takeoff pitch: auto_state.takeoff_pitch_cd
 *             - Maintains aggressive climb to build altitude quickly
 *             - No level-off considerations yet
 *          
 *          2. LEVEL-OFF APPROACH (near target altitude with TKOFF_LVL_PITCH_DS configured):
 *             - Detect when seconds-to-target < TKOFF_LVL_PITCH_DS
 *             - Calculate based on climb rate and remaining altitude
 *             - Initialize level-off sequence at detection point
 *             - Record starting height for scaling calculations
 *          
 *          3. ACTIVE LEVEL-OFF (sequence initiated):
 *             - Progressively reduce pitch minimum toward zero
 *             - Scaling: pitch_min = takeoff_pitch * (remaining_height / initial_height)
 *             - Linear reduction provides smooth level-off
 *             - Prevents aggressive pitch-down that could overshoot altitude
 *          
 *          Level-Off Triggering Conditions:
 *          - Must be in TAKEOFF flight stage
 *          - Climb rate must be established (sink_rate < -0.1 m/s)
 *          - Altitude must be above 10 meters (avoid ground effects)
 *          - Seconds to target < TKOFF_LVL_PITCH_DS parameter
 *          
 *          Level-Off Calculation:
 *          sec_to_target = remaining_height / climb_rate
 *          If sec_to_target <= TKOFF_LVL_PITCH_DS:
 *            - Start level-off sequence
 *            - Record current remaining_height as reference
 *            - Scale pitch from that point to zero at target altitude
 * 
 * @return int16_t Minimum pitch angle in centidegrees
 * 
 * @note Called from takeoff_calc_pitch() to set TECS pitch minimum
 * @note Level-off sequence improves altitude capture accuracy
 * @note TKOFF_LVL_PITCH_DS = 0 disables dynamic level-off (use fixed pitch to altitude)
 * 
 * @see auto_state.takeoff_pitch_cd - Full climb pitch angle
 * @see auto_state.height_below_takeoff_to_level_off_cm - Level-off start height
 * @see g.takeoff_pitch_limit_reduction_sec - TKOFF_LVL_PITCH_DS parameter
 */
int16_t Plane::get_takeoff_pitch_min_cd(void)
{
    // Not in takeoff phase - return full climb pitch (no level-off logic)
    if (flight_stage != AP_FixedWing::FlightStage::TAKEOFF) {
        return auto_state.takeoff_pitch_cd;
    }

    // Calculate how much altitude remains to target
    int32_t relative_alt_cm = adjusted_relative_altitude_cm();
    int32_t remaining_height_to_target_cm = (auto_state.takeoff_altitude_rel_cm - relative_alt_cm);

    // LEVEL-OFF SEQUENCE: Gradually reduce pitch as we approach target altitude
    // Uses "seconds to target" method when TKOFF_LVL_PITCH_DS > 0
    if (g.takeoff_pitch_limit_reduction_sec > 0) {
        // ACTIVE LEVEL-OFF: If height-below-target has been initialized, apply progressive scaling
        if (auto_state.height_below_takeoff_to_level_off_cm != 0) {
            // Scale pitch from full climb to zero based on remaining height
            float scalar = remaining_height_to_target_cm / (float)auto_state.height_below_takeoff_to_level_off_cm;
            return auto_state.takeoff_pitch_cd * scalar;
        }

        // LEVEL-OFF DETECTION: Are we entering the region where we start levelling off?
        // Check if we're climbing (sink_rate negative when climbing)
        if (auto_state.sink_rate < -0.1f) {
            // Calculate seconds to target altitude at current climb rate
            float sec_to_target = (remaining_height_to_target_cm * 0.01f) / (-auto_state.sink_rate);
            if (sec_to_target > 0 &&
                relative_alt_cm >= 1000 && // Must be above 10m to avoid ground effect
                sec_to_target <= g.takeoff_pitch_limit_reduction_sec) { // Within level-off window
                // INITIATE LEVEL-OFF: Make a note of this altitude to use as start height for scaling
                gcs().send_text(MAV_SEVERITY_INFO, "Takeoff level-off starting at %dm", int(remaining_height_to_target_cm/100));
                auto_state.height_below_takeoff_to_level_off_cm = remaining_height_to_target_cm;
                takeoff_state.level_off_start_time_ms = AP_HAL::millis();
            }
        }
    }
    // Default: Return full climb pitch (not yet in level-off phase)
    return auto_state.takeoff_pitch_cd;
}

/**
 * @brief Calculate elevator hold percentage for tail-dragger takeoff ground roll
 * 
 * @details Returns elevator position to hold tail down during initial ground roll
 *          phase of tail-dragger aircraft takeoff. This prevents premature nose-up
 *          rotation and maintains positive control during acceleration. Works in
 *          both AUTO takeoff and FBWA mode with FBWA_TDRAG_CHAN enabled.
 * 
 *          TAIL-DRAGGER GROUND ROLL SEQUENCE:
 *          
 *          1. TAIL-DOWN PHASE (0 to TKOFF_TDRAG_SPD1):
 *             - Apply TKOFF_TDRAG_ELEV elevator deflection (typically negative/down)
 *             - Holds tail down, keeping tail wheel on ground
 *             - Maintains directional control via tail wheel steering
 *             - Prevents premature liftoff with inadequate airspeed
 *             - Continues until reaching TKOFF_TDRAG_SPD1 airspeed
 *          
 *          2. TAIL-RAISE TRANSITION (above TKOFF_TDRAG_SPD1):
 *             - Return 0 (release tail hold)
 *             - Allows tail to rise naturally with increasing airspeed
 *             - Transitions to level attitude for liftoff
 *             - Aircraft accelerates in level attitude until Vrotate
 *          
 *          SAFETY ABORT CONDITIONS:
 *          - Pitch increases >10° above initial: Emergency tail raise
 *            (Indicates early liftoff - raise tail to prevent stall)
 *          - Not in takeoff flight stage: Return 0
 *          - TKOFF_TDRAG_ELEV = 0: Feature disabled, return 0
 *          
 *          MODE COMPATIBILITY:
 *          - AUTO takeoff: Active when flight_stage == TAKEOFF
 *          - FBWA mode: Active when fbwa_tdrag_takeoff_mode flag set
 *          - Other modes: Inactive (returns 0)
 *          
 *          Parameter Configuration:
 *          - TKOFF_TDRAG_ELEV: Elevator deflection percentage (-100 to +100)
 *            * Typically negative (down elevator) to hold tail down
 *            * Magnitude depends on aircraft geometry and elevator authority
 *          - TKOFF_TDRAG_SPD1: Speed to raise tail (m/s, typically 10-15 m/s)
 *            * Set based on aircraft's tail-raising speed
 *            * Too low: Premature tail raise, loss of steering
 *            * Too high: Extended tail-down, excessive drag
 * 
 * @return Elevator hold percentage (-100 to +100), or 0 if tail hold not active
 * 
 * @note Called from servo_output mixers to override elevator during ground roll
 * @note Works in conjunction with TKOFF_GND_PITCH for tail-dragger configuration
 * @note FBWA tail-drag mode activated via FBWA_TDRAG_CHAN auxiliary switch
 * 
 * @warning Incorrect TKOFF_TDRAG_ELEV can cause ground loop or nose-over
 * @warning TKOFF_TDRAG_SPD1 too high can delay liftoff beyond safe speed
 * 
 * @see TKOFF_TDRAG_ELEV - Elevator deflection percentage parameter
 * @see TKOFF_TDRAG_SPD1 - Tail raise airspeed parameter  
 * @see auto_state.fbwa_tdrag_takeoff_mode - FBWA tail-drag mode flag
 */
int8_t Plane::takeoff_tail_hold(void)
{
    // Check if we're in a takeoff mode that supports tail-dragger hold
    // AUTO: Check flight stage, FBWA: Check fbwa_tdrag_takeoff_mode flag
    bool in_takeoff = ((plane.flight_stage == AP_FixedWing::FlightStage::TAKEOFF) ||
                       (control_mode == &mode_fbwa && auto_state.fbwa_tdrag_takeoff_mode));
    if (!in_takeoff) {
        // Not in takeoff flight stage - tail hold not applicable
        return 0;
    }
    
    // Check if tail-dragger feature is configured (TKOFF_TDRAG_ELEV != 0)
    if (g.takeoff_tdrag_elevator == 0) {
        // Feature disabled via parameter - no tail hold applied
        goto return_zero;
    }
    
    // PHASE 1 END CHECK: Have we reached tail-raise speed (TKOFF_TDRAG_SPD1)?
    if (auto_state.highest_airspeed >= g.takeoff_tdrag_speed1) {
        // We've exceeded TKOFF_TDRAG_SPD1 - time to raise tail for level acceleration
        // Return 0 to release tail hold and allow natural tail rise
        goto return_zero;
    }
    
    // SAFETY ABORT: Check for excessive pitch increase (>10° above initial)
    // This indicates premature nose-up rotation or early liftoff
    if (ahrs.pitch_sensor > auto_state.initial_pitch_cd + 1000) {
        // Pitch has increased by more than 10 degrees (1000 centidegrees)
        // Possible causes: Early liftoff, bad TKOFF_TDRAG_SPD1 setting, wind gust
        // Release tail hold immediately to prevent stall
        goto return_zero;
    }
    
    // PHASE 1 ACTIVE: We are in tail-down ground roll phase
    // Apply configured elevator deflection to hold tail down
    return g.takeoff_tdrag_elevator;

return_zero:
    // If exiting FBWA tail-drag mode, notify and clear flag
    if (auto_state.fbwa_tdrag_takeoff_mode) {
        gcs().send_text(MAV_SEVERITY_NOTICE, "FBWA tdrag off");
        auto_state.fbwa_tdrag_takeoff_mode = false;
    }
    return 0;
}

#if AP_LANDINGGEAR_ENABLED
/**
 * @brief Update landing gear position based on altitude during takeoff/landing
 * 
 * @details Calls the landing gear library to update gear position based on
 *          current altitude above ground. The gear library uses configured
 *          altitude thresholds to automatically retract/deploy gear during
 *          flight operations.
 * 
 *          LANDING GEAR OPERATION:
 *          
 *          During Takeoff:
 *          - Gear starts deployed (down) before takeoff
 *          - Automatically retracts after reaching LGR_DEPLOY_ALT altitude
 *          - Reduces drag and improves efficiency in cruise
 *          
 *          During Landing:
 *          - Automatically deploys when descending below LGR_DEPLOY_ALT
 *          - Ensures gear down and locked before touchdown
 *          - Manual override available via RC channel or MAVLink
 *          
 *          Altitude Source:
 *          - Uses relative_ground_altitude() with TAKEOFF_LANDING mode
 *          - Combines barometer, GPS, and rangefinder (if available)
 *          - Rangefinder used for precision near ground
 *          - Barometer used for altitude above rangefinder range
 * 
 * @note Only compiled when AP_LANDINGGEAR_ENABLED is defined
 * @note Called from main loop update functions during takeoff/landing
 * @note Landing gear library handles actual servo commands
 * 
 * @see g2.landing_gear - Landing gear library instance
 * @see relative_ground_altitude() - Altitude calculation function
 * @see LGR_DEPLOY_ALT - Altitude threshold for gear deployment
 */
void Plane::landing_gear_update(void)
{
    g2.landing_gear.update(relative_ground_altitude(RangeFinderUse::TAKEOFF_LANDING));
}
#endif

/**
 * @brief Check for takeoff timeout and abort if aircraft fails to accelerate
 * 
 * @details Monitors ground speed after takeoff sequence starts. If aircraft
 *          remains below minimum takeoff speed (4 m/s) for longer than the
 *          configured timeout period, assumes takeoff failure and automatically
 *          disarms to prevent prolonged full-throttle ground operation.
 * 
 *          TAKEOFF TIMEOUT PROTECTION:
 *          
 *          Purpose:
 *          - Prevents damage from prolonged full-throttle operation on ground
 *          - Detects failed takeoff attempts (insufficient thrust, brake engaged, etc.)
 *          - Automatically disarms to protect motor/ESC from overheating
 *          - Reduces risk of runway excursion or collision
 *          
 *          Operation Sequence:
 *          1. Timer starts when auto_takeoff_check() commits to takeoff
 *             (takeoff_state.start_time_ms set to current time)
 *          2. Monitor ground speed continuously
 *          3. If speed reaches 4 m/s: Reset timer (takeoff progressing normally)
 *          4. If speed stays below 4 m/s for TKOFF_TIMEOUT seconds: Disarm
 *          
 *          Timeout Conditions (all must be true):
 *          - Takeoff timer active (start_time_ms != 0)
 *          - TKOFF_TIMEOUT parameter > 0 (feature enabled)
 *          - Ground speed < 4 m/s (minimum takeoff speed)
 *          - Time since start > TKOFF_TIMEOUT seconds
 *          
 *          Common Causes of Timeout:
 *          - Insufficient throttle for aircraft weight
 *          - Parking brake still engaged
 *          - Motor/ESC failure or low battery
 *          - Excessive headwind or adverse conditions
 *          - Incorrect throttle calibration
 *          - Wheel chocks still in place
 * 
 * @return true if timeout occurred and aircraft was disarmed
 * @return false if takeoff progressing normally or timeout not configured
 * 
 * @note Called from main loop during AUTO and GUIDED takeoff sequences
 * @note Minimum takeoff speed threshold is fixed at 4 m/s (14.4 km/h, 9 mph)
 * @note Timer automatically resets once aircraft reaches 4 m/s ground speed
 * @note Disarm uses TAKEOFFTIMEOUT method (logged separately from manual disarm)
 * 
 * @warning Timeout disarm is NOT inhibited - occurs even with safety off
 * @warning Set TKOFF_TIMEOUT = 0 to disable if testing without propeller
 * 
 * @see g2.takeoff_timeout - TKOFF_TIMEOUT parameter (seconds)
 * @see takeoff_state.start_time_ms - Takeoff start timestamp
 * @see arming.disarm() - Disarm function with reason code
 */
bool Plane::check_takeoff_timeout(void)
{
    // Check if timeout monitoring is active (timer started and feature enabled)
    if (takeoff_state.start_time_ms != 0 && g2.takeoff_timeout > 0) {
        const float ground_speed = AP::gps().ground_speed();
        const float takeoff_min_ground_speed = 4;  // Minimum speed indicating takeoff progress (m/s)
        
        // SUCCESS CHECK: Has aircraft reached minimum takeoff speed?
        if (ground_speed >= takeoff_min_ground_speed) {
            // Aircraft is accelerating normally - reset timer and continue
            takeoff_state.start_time_ms = 0;
            return false;
        } else {
            // Aircraft still below minimum speed - check if timeout expired
            uint32_t now = AP_HAL::millis();
            if (now - takeoff_state.start_time_ms > (uint32_t)(1000U * g2.takeoff_timeout)) {
                // TIMEOUT EXPIRED: Aircraft failed to accelerate within time limit
                // Notify GCS with current speed and disarm for safety
                gcs().send_text(MAV_SEVERITY_INFO, "Takeoff timeout: %.1f m/s speed < 4m/s", ground_speed);
                arming.disarm(AP_Arming::Method::TAKEOFFTIMEOUT);
                takeoff_state.start_time_ms = 0;
                return true;  // Timeout occurred, aircraft disarmed
            }
        }
     }
     // Timeout not configured, not started, or not yet expired
     return false;
}

/**
 * @brief Check if pitch level-off phase duration has expired during takeoff climb
 * 
 * @details Monitors the duration of the pitch level-off phase that occurs near
 *          target altitude. Returns true when the configured level-off time has
 *          elapsed, indicating the aircraft should transition from level-off to
 *          normal climb/cruise behavior.
 * 
 *          PITCH LEVEL-OFF TIMEOUT:
 *          
 *          Purpose:
 *          - Determines when dynamic pitch reduction phase is complete
 *          - Prevents indefinite shallow climb near target altitude
 *          - Triggers transition to normal pitch control
 *          - Works with get_takeoff_pitch_min_cd() to smoothly reach altitude
 *          
 *          Level-Off Phase Operation:
 *          1. As aircraft approaches target altitude, level_off_start_time_ms is set
 *          2. Pitch is gradually reduced from takeoff pitch to level flight
 *          3. This function monitors elapsed time during level-off
 *          4. When TKOFF_LVL_PITCH (g.takeoff_pitch_limit_reduction_sec) expires:
 *             - Returns true to signal level-off complete
 *             - Aircraft transitions to normal altitude hold
 *             - Pitch control handed back to TECS
 *          
 *          Timer Lifecycle:
 *          - Started: When entering level-off region (set in get_takeoff_pitch_min_cd())
 *          - Active: During pitch reduction phase
 *          - Expired: After TKOFF_LVL_PITCH seconds
 *          - Reset: When takeoff completes or mode changes
 * 
 * @return true if level-off phase has timed out (duration > TKOFF_LVL_PITCH)
 * @return false if level-off not active or still within time limit
 * 
 * @note Level-off timer started by get_takeoff_pitch_min_cd() when approaching altitude
 * @note Only active when TKOFF_LVL_PITCH parameter is non-zero
 * @note Called from takeoff completion checking logic
 * 
 * @see get_takeoff_pitch_min_cd() - Function that starts level-off timer
 * @see g.takeoff_pitch_limit_reduction_sec - TKOFF_LVL_PITCH parameter (seconds)
 * @see takeoff_state.level_off_start_time_ms - Level-off start timestamp
 */
bool Plane::check_takeoff_timeout_level_off(void)
{
    // Check if level-off phase is active (timer has been started)
    if (takeoff_state.level_off_start_time_ms > 0) {
        // Level-off phase in progress - check elapsed time
        uint32_t now = AP_HAL::millis();
        if ((now - takeoff_state.level_off_start_time_ms) > (uint32_t)(1000U * g.takeoff_pitch_limit_reduction_sec)) {
            // Level-off duration exceeded TKOFF_LVL_PITCH parameter
            // Signal that level-off phase is complete
            return true;
        }
    }
    // Level-off not active or still within time limit
    return false;
}
