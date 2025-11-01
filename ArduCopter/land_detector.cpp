/**
 * @file land_detector.cpp
 * @brief Landing detection system for ArduCopter
 * 
 * @details This file implements a multi-stage landing detection algorithm that determines
 *          when the vehicle has successfully landed on the ground. The system uses multiple
 *          sensor inputs and state checks to reliably detect landing while avoiding false
 *          positives from turbulence, uneven terrain, or aggressive flight maneuvers.
 * 
 *          Landing Detection State Machine:
 *          - NOT_LANDED: Normal flight operation
 *          - LAND_COMPLETE_MAYBE: Initial landing detected, confirmation in progress
 *          - LAND_COMPLETE: Landing confirmed, may trigger motor disarm
 * 
 *          The detector uses a time-based confirmation approach where all landing criteria
 *          must remain true for a specified duration (LAND_DETECTOR_TRIGGER_SEC) before
 *          confirming landing. This prevents false positives from momentary ground contact
 *          or sensor noise.
 * 
 *          Key Design Considerations:
 *          - Sensors known to be unreliable during landing (barometer, EKF vertical velocity)
 *            are NOT used for landing detection
 *          - Multiple independent checks provide redundancy
 *          - Time-based confirmation reduces false positive rate
 *          - Integration with disarm-on-land feature for automated shutdown
 * 
 * @note This is safety-critical code - changes must be thoroughly tested in SITL and
 *       on physical hardware before deployment
 * 
 * @warning Incorrect landing detection can result in:
 *          - Premature motor shutdown during flight (safety hazard)
 *          - Failure to disarm after landing (battery drain, prop hazard)
 *          - Uncommanded takeoff if landing not detected
 * 
 * @see update_land_detector() for main detection algorithm
 * @see set_land_complete() for landing confirmation and disarm logic
 */

#include "Copter.h"

#include <AP_Stats/AP_Stats.h>              // statistics library

// Land detector configuration constants - tuned for reliable detection across vehicle types
#define LAND_CHECK_ANGLE_ERROR_DEG  30.0f       // maximum angle error (deg) to be considered landing - larger errors indicate external disturbance
#define LAND_CHECK_LARGE_ANGLE_CD   1500.0f     // maximum angle target (centidegrees, 15 deg) to be considered landing - larger requests indicate aggressive flight
#define LAND_CHECK_ACCEL_MOVING     3.0f        // maximum acceleration (m/s²) after subtracting gravity - higher values indicate falling or braking


/**
 * @brief Counter tracking consecutive passes of landing criteria
 * 
 * @details This counter increments each time all landing detection criteria are met
 *          and resets to zero when any criterion fails. Landing is confirmed only when
 *          the counter reaches a threshold (LAND_DETECTOR_TRIGGER_SEC * loop_rate).
 *          This time-based approach filters out momentary ground contact and sensor noise.
 * 
 *          The counter is also used for the "maybe landed" state at a lower threshold
 *          (LAND_DETECTOR_MAYBE_TRIGGER_SEC * loop_rate).
 */
static uint32_t land_detector_count = 0;

/**
 * @brief Run land and crash detection algorithms
 * 
 * @details This function is the top-level entry point for all landing and crash detection
 *          systems. It coordinates multiple safety-critical detection algorithms:
 *          - Landing detection (update_land_detector)
 *          - Parachute deployment monitoring (parachute_check)
 *          - Crash detection (crash_check)
 *          - Thrust loss detection (thrust_loss_check)
 *          - Yaw imbalance detection (yaw_imbalance_check)
 * 
 *          The function also maintains a 1Hz filtered acceleration in earth frame, which
 *          is used by the landing detector to determine if the vehicle is stationary.
 *          The Z-axis acceleration is adjusted by adding GRAVITY_MSS to remove the
 *          gravitational component, leaving only dynamic acceleration.
 * 
 * @note Called at MAIN_LOOP_RATE (typically 400Hz for multicopters)
 * 
 * @warning This function must execute quickly to maintain real-time performance.
 *          Excessive computation time will impact flight control loop timing.
 * 
 * @see update_land_detector() for landing detection algorithm details
 */
void Copter::update_land_and_crash_detectors()
{
    // update 1hz filtered acceleration
    Vector3f accel_ef = ahrs.get_accel_ef();
    accel_ef.z += GRAVITY_MSS;
    land_accel_ef_filter.apply(accel_ef, scheduler.get_loop_period_s());

    update_land_detector();

#if HAL_PARACHUTE_ENABLED
    // check parachute
    parachute_check();
#endif

    crash_check();
    thrust_loss_check();
    yaw_imbalance_check();
}

/**
 * @brief Main landing detection algorithm - determines if vehicle has landed
 * 
 * @details This function implements a comprehensive landing detection system that uses multiple
 *          independent sensor checks to reliably determine when the vehicle has contacted the
 *          ground and come to rest. The algorithm is designed to work across different vehicle
 *          types (standard multicopters, helicopters) and landing conditions (level surfaces,
 *          slopes, moving platforms).
 * 
 *          MULTI-STAGE DETECTION STATE MACHINE:
 *          1. NOT_LANDED (ap.land_complete = false, ap.land_complete_maybe = false)
 *             - Normal flight operation
 *             - All landing checks being monitored
 *             - land_detector_count = 0
 * 
 *          2. LAND_COMPLETE_MAYBE (ap.land_complete_maybe = true)
 *             - All landing criteria met for LAND_DETECTOR_MAYBE_TRIGGER_SEC (0.2s)
 *             - Flight modes can use this for pre-landing actions
 *             - Still monitoring for failed landing criteria
 * 
 *          3. LAND_COMPLETE (ap.land_complete = true)
 *             - All landing criteria met for LAND_DETECTOR_TRIGGER_SEC (1.0s)
 *             - Landing confirmed, motors may disarm if configured
 *             - Stats tracking records vehicle as not flying
 *             - AHRS notified of landed state
 * 
 *          SENSOR SELECTION AND RELIABILITY:
 *          The following sensors are INTENTIONALLY NOT USED because they are unreliable during landing:
 *          - Barometer altitude: Ground effect can cause errors >4m during descent
 *          - EKF vertical velocity: Poor barometer data and large ground impact accelerations corrupt estimate
 *          - Earth frame angle or angle error: Uneven surfaces force airframe to match ground angle
 *          - Gyro output: Airframe may rock back and forth after landing on uneven surfaces
 *          - Rangefinder: Problematic at very short distances (<30cm) due to signal characteristics
 *          - Input throttle: In slow land modes, input throttle may be only slightly less than hover
 * 
 *          LANDING CRITERIA USED (all must be true simultaneously):
 *          1. Motor Output: Throttle/collective at lower limit (vehicle not producing significant thrust)
 *          2. Throttle Mix: Attitude controller throttle mix at minimum (throttle authority prioritized)
 *          3. Angle Request: Pilot/autopilot not requesting large attitude angles (< 15 degrees)
 *          4. Angle Error: Actual attitude tracking desired attitude (< 30 degree error)
 *          5. Acceleration: Vehicle stationary in earth frame (< 3.0 m/s² after removing gravity)
 *          6. Descent Rate: Vertical velocity near zero (< 0.5 m/s down velocity)
 *          7. Rangefinder: If available and healthy, altitude < 2m
 *          8. Weight-on-Wheels (WoW): If available, landing gear indicates ground contact
 * 
 *          TIME-BASED CONFIRMATION:
 *          All criteria must remain true for a specified duration to confirm landing:
 *          - LAND_DETECTOR_MAYBE_TRIGGER_SEC (0.2s) for "maybe landed" state
 *          - LAND_DETECTOR_TRIGGER_SEC (1.0s) for "definitely landed" state
 *          - LAND_AIRMODE_DETECTOR_TRIGGER_SEC (4.0s) when airmode is enabled
 *          If any criterion fails, land_detector_count resets to zero and detection restarts.
 * 
 *          SPECIAL CASES:
 *          - Disarmed: Always considered landed
 *          - Already landed + high throttle: Clears landing flag (takeoff detection backup)
 *          - Standby mode: Landing detector disabled
 *          - Airmode enabled: Extended confirmation time (4s) to prevent false positives
 *          - Weight-on-Wheels sensor: Relaxes other criteria (2x tolerance) when available
 * 
 * @note Called at MAIN_LOOP_RATE (typically 400Hz for multicopters)
 * 
 * @warning This is safety-critical code. Changes must be tested in:
 *          - SITL simulation with various landing scenarios
 *          - Physical hardware with different landing surfaces
 *          - Edge cases: slopes, moving platforms, turbulent conditions
 * 
 * @see set_land_complete() for landing confirmation actions
 * @see set_land_complete_maybe() for intermediate landing state
 * 
 * Source: ArduCopter/land_detector.cpp:37-173
 */
void Copter::update_land_detector()
{
    // SENSOR EXCLUSIONS - these sensors are unreliable during landing and must not be used:
    // barometer altitude :                 ground effect can cause errors larger than 4m
    // EKF vertical velocity or altitude :  poor barometer and large acceleration from ground impact
    // earth frame angle or angle error :   landing on an uneven surface will force the airframe to match the ground angle
    // gyro output :                        on uneven surface the airframe may rock back an forth after landing
    // range finder :                       tend to be problematic at very short distances
    // input throttle :                     in slow land the input throttle may be only slightly less than hover

#if HAL_LOGGING_ENABLED
    uint16_t logging_flags = 0;
#define SET_LOG_FLAG(condition, flag) if (condition) { logging_flags |= (uint16_t)flag; }
#else
#define SET_LOG_FLAG(condition, flag)
#endif

    // CASE 1: Disarmed vehicle is always considered landed
    // Rationale: Cannot be flying without motors armed
    if (!motors->armed()) {
        set_land_complete(true);
    } 
    // CASE 2: Already landed - check for takeoff detection
    // Monitor for high throttle output indicating pilot is attempting takeoff
    else if (ap.land_complete) {
#if FRAME_CONFIG == HELI_FRAME
        // Helicopter takeoff detection: High rotor speed + takeoff collective pitch
        // Indicates pilot commanding significant lift for takeoff
        if (!flightmode->is_taking_off() && motors->get_takeoff_collective() && motors->get_spool_state() == AP_Motors::SpoolState::THROTTLE_UNLIMITED) {
#else
        // Multicopter takeoff detection: Throttle output significantly above hover
        // Combined with unlimited spool state indicates commanded takeoff
        if (!flightmode->is_taking_off() && motors->get_throttle_out() > get_non_takeoff_throttle() && motors->get_spool_state() == AP_Motors::SpoolState::THROTTLE_UNLIMITED) {
            // SAFETY CHECK: This should never execute as takeoff should be detected by flight mode
            // If we reach here, indicates missing takeoff detection logic in the flight mode
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
#endif
            set_land_complete(false);  // Clear landing flag, vehicle is taking off
        }
    } 
    // CASE 3: Standby mode - disable landing detector
    // In standby mode, vehicle may be on ground but detector should not run
    else if (standby_active) {
        land_detector_count = 0;
    } 
    // CASE 4: Armed and flying - perform full landing detection algorithm
    else {

        // Landing confirmation timeout - how long all criteria must remain true
        // Extended timeout for airmode to prevent false positives during aggressive flight
        float land_trigger_sec = LAND_DETECTOR_TRIGGER_SEC;

#if FRAME_CONFIG == HELI_FRAME
        // HELICOPTER-SPECIFIC MOTOR OUTPUT CHECK
        // Helicopters use collective pitch instead of throttle for altitude control
        // Check for both manual collective modes and altitude hold modes:
        // - Manual modes: Collective pitch below landing minimum OR throttle stick at zero
        //   (The stick zero check handles cases where stabilize zero position wasn't below collective min)
        // - Altitude hold modes: Pilot commanding descent AND collective at minimum allowed
        // Also require roll angle < 90° to ensure helicopter isn't inverted

        const bool landing = flightmode->is_landing();
        SET_LOG_FLAG(landing, LandDetectorLoggingFlag::LANDING);
        
        // Motor at lower limit check combines multiple conditions:
        // 1. Manual throttle modes with low collective or stick position + not inverted
        // 2. Autorotate mode with low collective (special landing mode)
        // 3. Altitude hold modes with throttle limiting + commanded descent (not force-flying or actively landing)
        bool motor_at_lower_limit = (flightmode->has_manual_throttle() && (motors->get_below_land_min_coll() || heli_flags.coll_stk_low) && fabsf(ahrs.get_roll_rad()) < M_PI/2.0f)
#if MODE_AUTOROTATE_ENABLED
                                    || (flightmode->mode_number() == Mode::Number::AUTOROTATE && motors->get_below_land_min_coll())
#endif
                                    || ((!get_force_flying() || landing) && motors->limit.throttle_lower && pos_control->get_vel_desired_NEU_cms().z < 0.0f);
        bool throttle_mix_at_min = true;  // Helicopters always treat this as true
#else
        // MULTICOPTER MOTOR OUTPUT CHECK
        // Check that throttle output is at lower limit (motors producing minimal thrust)
        // This indicates the vehicle is not attempting to maintain altitude
        bool motor_at_lower_limit = motors->limit.throttle_lower;
        
        // Check that attitude controller has minimized throttle mixing
        // When throttle_mix is at minimum, the controller prioritizes throttle over attitude control
        // This is appropriate for landed state where attitude control is less critical
        bool throttle_mix_at_min = attitude_control->is_throttle_mix_min();
        
        // AIRMODE SPECIAL HANDLING
        // In airmode, throttle is never actually at mix minimum during flight to maintain
        // full attitude authority. Extend the landing confirmation timeout to 4 seconds
        // to reduce false positives from aggressive maneuvers near the ground.
        if (flightmode->has_manual_throttle() && air_mode == AirMode::AIRMODE_ENABLED) {
            land_trigger_sec = LAND_AIRMODE_DETECTOR_TRIGGER_SEC;  // 4.0 seconds instead of 1.0
            throttle_mix_at_min = true;  // Override check since it's never true in airmode
        }
#endif
        SET_LOG_FLAG(motor_at_lower_limit, LandDetectorLoggingFlag::MOTOR_AT_LOWER_LIMIT);
        SET_LOG_FLAG(throttle_mix_at_min, LandDetectorLoggingFlag::THROTTLE_MIX_AT_MIN);

        // LANDING GEAR WEIGHT-ON-WHEELS (WoW) SENSOR INTEGRATION
        // If a WoW sensor is available and providing valid readings, we can relax
        // other landing criteria by 2x. The WoW sensor provides direct ground contact
        // detection which is more reliable than inferring ground contact from IMU data.
        uint8_t land_detector_scalar = 1;  // Default: strict criteria
#if AP_LANDINGGEAR_ENABLED
        if (landinggear.get_wow_state() != AP_LandingGear::LG_WOW_UNKNOWN) {
            // WoW sensor available and healthy - double the tolerances for acceleration and descent rate
            // This allows landing detection to succeed even with higher vibration or wind disturbance
            land_detector_scalar = 2;
        }
#endif

        // CRITERION 1: ANGLE REQUEST CHECK
        // Verify pilot/autopilot is not requesting aggressive attitude maneuvers
        // Large angle requests (>15°) indicate intentional flight, not landing
        // Uses vector magnitude of roll/pitch targets to catch any axis exceeding threshold
        const Vector3f& angle_target_rad = attitude_control->get_att_target_euler_rad();
        bool large_angle_request = angle_target_rad.xy().length() > cd_to_rad(LAND_CHECK_LARGE_ANGLE_CD);
        SET_LOG_FLAG(large_angle_request, LandDetectorLoggingFlag::LARGE_ANGLE_REQUEST);

        // CRITERION 2: ANGLE ERROR CHECK
        // Verify vehicle is tracking desired attitude (not being blown around)
        // Large angle errors (>30°) indicate external disturbances or control issues
        // that suggest the vehicle is not safely on the ground
        const float angle_error = attitude_control->get_att_error_angle_deg();
        bool large_angle_error = (angle_error > LAND_CHECK_ANGLE_ERROR_DEG);
        SET_LOG_FLAG(large_angle_error, LandDetectorLoggingFlag::LARGE_ANGLE_ERROR);

        // CRITERION 3: ACCELERATION CHECK
        // Verify vehicle is stationary (not falling, climbing, or braking)
        // Uses 1Hz low-pass filtered earth-frame acceleration with gravity removed
        // Threshold multiplied by land_detector_scalar (2x if WoW sensor available)
        // LAND_DETECTOR_ACCEL_MAX is typically 3.0 m/s² (0.3g)
        bool accel_stationary = (land_accel_ef_filter.get().length() <= LAND_DETECTOR_ACCEL_MAX * land_detector_scalar);
        SET_LOG_FLAG(accel_stationary, LandDetectorLoggingFlag::ACCEL_STATIONARY);

        // CRITERION 4: DESCENT RATE CHECK
        // Verify vertical velocity is near zero (not descending or ascending)
        // Gets down-velocity from AHRS, marked invalid if high vibration detected
        // Threshold typically 0.5 m/s, multiplied by land_detector_scalar if WoW available
        float vel_d_ms = 0;
        UNUSED_RESULT(AP::ahrs().get_velocity_D(vel_d_ms, copter.vibration_check.high_vibes));
        const bool descent_rate_low = fabsf(vel_d_ms) < LAND_DETECTOR_VEL_Z_MAX * land_detector_scalar;
        SET_LOG_FLAG(descent_rate_low, LandDetectorLoggingFlag::DESCENT_RATE_LOW);

        // CRITERION 5: RANGEFINDER ALTITUDE CHECK
        // If rangefinder is available and healthy, only detect landing below 2 meters
        // This prevents false landing detection at high altitude if other criteria are met
        // If rangefinder is unavailable or unhealthy, this check passes (allows landing detection)
        bool rangefinder_check = (!rangefinder_alt_ok() || rangefinder_state.alt_cm_filt.get() < LAND_RANGEFINDER_MIN_ALT_CM);
        SET_LOG_FLAG(rangefinder_check, LandDetectorLoggingFlag::RANGEFINDER_BELOW_2M);

        // CRITERION 6: WEIGHT-ON-WHEELS (WoW) SENSOR CHECK
        // If WoW sensor available, verify it indicates ground contact or unknown state
        // Never allow landing detection if WoW sensor explicitly reports no ground contact
        // If no WoW sensor, this check always passes (true)
#if AP_LANDINGGEAR_ENABLED
        const bool WoW_check = (landinggear.get_wow_state() == AP_LandingGear::LG_WOW || landinggear.get_wow_state() == AP_LandingGear::LG_WOW_UNKNOWN);
#else
        const bool WoW_check = true;  // No WoW sensor - always pass this check
#endif
        SET_LOG_FLAG(WoW_check, LandDetectorLoggingFlag::WOW);

        // LANDING CONFIRMATION LOGIC - TIME-BASED MULTI-CRITERION APPROACH
        // All criteria must be true simultaneously to increment the counter
        // This AND logic ensures comprehensive validation before confirming landing
        if (motor_at_lower_limit && throttle_mix_at_min && !large_angle_request && !large_angle_error && accel_stationary && descent_rate_low && rangefinder_check && WoW_check) {
            // ALL LANDING CRITERIA MET
            // Increment counter towards confirmation threshold
            // Don't increment beyond threshold to prevent counter overflow
            if( land_detector_count < land_trigger_sec*scheduler.get_loop_rate_hz()) {
                land_detector_count++;
            } else {
                // LANDING CONFIRMED: Counter has reached threshold
                // All criteria have been continuously true for required duration:
                // - Standard: 1.0 second (LAND_DETECTOR_TRIGGER_SEC)
                // - Airmode: 4.0 seconds (LAND_AIRMODE_DETECTOR_TRIGGER_SEC)
                set_land_complete(true);
            }
        } else {
            // AT LEAST ONE CRITERION FAILED
            // Reset counter to zero - landing detection must restart from beginning
            // This ensures we don't accumulate false positives from intermittent ground contact
            land_detector_count = 0;
        }
    }

    // UPDATE "MAYBE LANDED" STATE
    // This intermediate state activates after shorter duration (0.2s) than full landing confirmation
    // Flight modes can use land_complete_maybe for preparatory actions before full landing
    // Conditions: Either already fully landed OR counter exceeds maybe threshold
    set_land_complete_maybe(ap.land_complete || (land_detector_count >= LAND_DETECTOR_MAYBE_TRIGGER_SEC*scheduler.get_loop_rate_hz()));

#if HAL_LOGGING_ENABLED
// @LoggerMessage: LDET
// @Description: Land Detector State
// @Field: TimeUS: Time since system startup
// @Field: Flags: boolean state flags
// @FieldBitmaskEnum: Flags: Copter::LandDetectorLoggingFlag
// @Field: Count: landing_detector pass count
    SET_LOG_FLAG(ap.land_complete, LandDetectorLoggingFlag::LANDED);
    SET_LOG_FLAG(ap.land_complete_maybe, LandDetectorLoggingFlag::LANDED_MAYBE);
    SET_LOG_FLAG(standby_active, LandDetectorLoggingFlag::STANDBY_ACTIVE);
    Log_LDET(logging_flags, land_detector_count);
#undef SET_LOG_FLAG
#endif
}

#if HAL_LOGGING_ENABLED
/**
 * @brief Log landing detector state and criteria to dataflash
 * 
 * @details Writes LDET log message containing landing detector state flags and counter value.
 *          The logging is rate-limited and change-detection optimized to reduce log data:
 *          - Only logs when state changes (flags or counter differ from last log)
 *          - Maximum rate of 50Hz (minimum 20ms between log entries)
 * 
 *          LDET Log Message Fields:
 *          - TimeUS: Timestamp in microseconds
 *          - Flags: Bitmask of landing criteria and state (see LandDetectorLoggingFlag enum)
 *          - Count: Current value of land_detector_count (0 to threshold)
 * 
 *          Flag bits indicate which criteria are met and current landing state:
 *          - Motor at lower limit
 *          - Throttle mix at minimum
 *          - Large angle request (inverted - set when NOT requesting large angle)
 *          - Large angle error (inverted - set when error is small)
 *          - Acceleration stationary
 *          - Descent rate low
 *          - Rangefinder below 2m
 *          - Weight on wheels
 *          - Landed (land_complete)
 *          - Maybe landed (land_complete_maybe)
 * 
 * @param[in] logging_flags Bitmask of current landing detector state flags
 * @param[in] detector_count Current value of land_detector_count
 * 
 * @note This function is called at MAIN_LOOP_RATE but actual logging is throttled to 50Hz
 * 
 * @see LandDetectorLoggingFlag enum for flag definitions
 * @see update_land_detector() for flag generation
 */
void Copter::Log_LDET(uint16_t logging_flags, uint32_t detector_count)
{
    // Change detection optimization: Only log if state has changed
    // Avoids filling log with redundant entries when state is stable
    if (logging_flags == land_detector.last_logged_flags &&
        detector_count == land_detector.last_logged_count) {
        return;
    }
    
    // Rate limiting: Maximum 50Hz logging (20ms minimum interval)
    // Prevents excessive log data during rapid state changes
    const auto now = AP_HAL::millis();
    if (now - land_detector.last_logged_ms < 20) {
        return;
    }

    // Update last logged state for change detection
    land_detector.last_logged_count = detector_count;
    land_detector.last_logged_flags = logging_flags;
    land_detector.last_logged_ms = now;

    // Write LDET message to dataflash log
    // Format: TimeUS (uint64), Flags (uint16), Count (uint32)
    AP::logger().WriteStreaming(
        "LDET",
        "TimeUS," "Flags," "Count",
        "s"       "-"      "-",        // units: seconds, dimensionless, dimensionless
        "F"       "-"      "-",        // multipliers: none
        "Q"       "H"      "I",        // types: uint64, uint16, uint32
        AP_HAL::micros64(),
        logging_flags,
        land_detector_count
    );
}
#endif

/**
 * @brief Set land_complete flag and trigger disarm-on-land if configured
 * 
 * @details This function updates the ap.land_complete flag when landing state changes
 *          and coordinates multiple subsystems that need to know about landing state.
 *          When transitioning to landed state, it may automatically disarm the motors
 *          if disarm-on-land is configured and appropriate safety checks pass.
 * 
 *          SUBSYSTEM NOTIFICATIONS ON LANDING STATE CHANGE:
 *          - Dataflash logger: Writes LAND_COMPLETE or NOT_LANDED event
 *          - Flight statistics: Records vehicle as not flying (for flight time tracking)
 *          - AHRS: Notified of likely_flying state change (affects EKF tuning)
 *          - Motor disarm: Triggered if configured and all safety checks pass
 * 
 *          AUTOMATIC DISARM-ON-LAND LOGIC:
 *          When landing is detected (b = true), the function evaluates whether to
 *          automatically disarm the motors. This prevents accidental prop strikes
 *          and conserves battery. Disarm proceeds only if ALL conditions are met:
 * 
 *          1. THR_BEHAVE_DISARM_ON_LAND_DETECT parameter flag is set
 *          2. Motors are currently armed
 *          3. Flight mode does NOT have manual throttle (e.g., not STABILIZE)
 *          4. Flight mode allows disarm via LANDING method
 * 
 *          Manual throttle modes (STABILIZE, ACRO, etc.) are excluded because the
 *          pilot has direct throttle control and should manually disarm. This prevents
 *          unexpected disarm if the landing detector triggers during intended hovering.
 * 
 * @param[in] b New landing state (true = landed, false = not landed)
 * 
 * @note This function is idempotent - calling with unchanged state has no effect
 * 
 * @warning Automatic disarm is a safety-critical feature. Premature disarm during
 *          flight would cause a crash. The multi-check approach prevents this:
 *          - Time-based landing confirmation (1s+ of criteria)
 *          - Flight mode validation
 *          - Motor state verification
 * 
 * @see update_land_detector() for landing detection algorithm
 * @see AP_Arming::disarm() for disarm execution
 * 
 * Source: ArduCopter/land_detector.cpp:207-263
 */
void Copter::set_land_complete(bool b)
{
    // Early exit optimization: No action needed if state unchanged
    if( ap.land_complete == b )
        return;

    // Reset landing detector counter when state changes
    // Ensures clean state for next detection cycle
    land_detector_count = 0;

#if HAL_LOGGING_ENABLED
    // Log landing state change event for post-flight analysis
    if(b){
        AP::logger().Write_Event(LogEvent::LAND_COMPLETE);
    } else {
        AP::logger().Write_Event(LogEvent::NOT_LANDED);
    }
#endif
    
    // Update global landing state flag
    ap.land_complete = b;

#if AP_STATS_ENABLED
    // Update flight statistics - vehicle is not flying when landed
    // Used for tracking total flight time and flight cycle counts
    AP::stats()->set_flying(!b);
#endif

    // Notify AHRS of flying state change
    // AHRS uses this to adjust EKF tuning (different noise characteristics on ground vs flight)
    set_likely_flying(!b);

    // If transitioning to NOT landed, no further action needed
    if (!b) {
        return;
    }

    // === LANDED STATE - EVALUATE AUTOMATIC DISARM ===
    
    // CHECK 1: Disarm-on-land feature must be enabled
    // THR_BEHAVE_DISARM_ON_LAND_DETECT is a bitmask flag in g.throttle_behavior parameter
    if ((g.throttle_behavior & THR_BEHAVE_DISARM_ON_LAND_DETECT) == 0) {
        return;  // Feature disabled, use normal disarm delay logic
    }

    // CHECK 2: Motors must be currently armed
    // If already disarmed, no action needed
    if (!motors->armed()) {
        return;
    }

    // CHECK 3: Flight mode must NOT have manual throttle
    // Manual throttle modes (STABILIZE, ACRO, etc.) excluded because:
    // - Pilot has direct control and should manually disarm
    // - Landing detector might trigger during low hover
    // - Normal DISARM_DELAY logic should apply instead
    if (flightmode->has_manual_throttle()) {
        return;
    }

    // CHECK 4: Flight mode must allow disarm via landing detection
    // Some modes (e.g., specific autonomous modes) may prohibit automatic disarm
    // The LAND mode explicitly checks ap.land_complete before allowing disarm
    if (!flightmode->allows_arming(AP_Arming::Method::LANDING)) {
        return;
    }

    // ALL SAFETY CHECKS PASSED - Disarm the vehicle
    // AP_Arming::Method::LANDED indicates this is an automatic landing-triggered disarm
    // This is logged and can be distinguished from pilot-commanded disarm
    arming.disarm(AP_Arming::Method::LANDED);
}

/**
 * @brief Set land_complete_maybe intermediate landing state flag
 * 
 * @details This function manages the "maybe landed" state, which is an intermediate
 *          state between flying and definitely landed. This two-stage approach allows
 *          flight modes to take preparatory actions before full landing confirmation.
 * 
 *          LANDING STATE PROGRESSION:
 *          1. NOT_LANDED (both flags false)
 *          2. LAND_COMPLETE_MAYBE (after 0.2s of criteria) ← This function
 *          3. LAND_COMPLETE (after 1.0s of criteria)
 * 
 *          The "maybe landed" state is triggered when:
 *          - All landing criteria met for LAND_DETECTOR_MAYBE_TRIGGER_SEC (0.2s), OR
 *          - Already in LAND_COMPLETE state
 * 
 *          USAGE BY FLIGHT MODES:
 *          Flight modes can query ap.land_complete_maybe to:
 *          - Begin landing sequence wind-down
 *          - Prepare for motor disarm
 *          - Reduce control authority gradually
 *          - Start landing gear deployment
 *          - Initiate post-landing procedures
 * 
 *          This provides smoother transitions than abrupt changes at full landing confirmation.
 * 
 * @param[in] b New maybe-landed state (true = maybe landed, false = definitely not landed)
 * 
 * @note This function is called every loop iteration (MAIN_LOOP_RATE)
 * @note Only logs when transitioning TO maybe-landed state (not when clearing)
 * 
 * @see update_land_detector() for state calculation
 * @see set_land_complete() for full landing confirmation
 * 
 * Source: ArduCopter/land_detector.cpp:266-276
 */
void Copter::set_land_complete_maybe(bool b)
{
    // Early exit optimization: No action needed if state unchanged
    if (ap.land_complete_maybe == b)
        return;

    // Log transition to maybe-landed state for post-flight analysis
    // Note: Only logs when transitioning TO maybe-landed, not when clearing
    if (b) {
        LOGGER_WRITE_EVENT(LogEvent::LAND_COMPLETE_MAYBE);
    }
    
    // Update global maybe-landed state flag
    ap.land_complete_maybe = b;
}

/**
 * @brief Update throttle mix based on vehicle state and flight conditions
 * 
 * @details The throttle mix parameter controls the balance between attitude control
 *          authority and throttle/altitude control in the motor mixer. This function
 *          dynamically adjusts the mix based on vehicle state and pilot commands to
 *          optimize control in different flight regimes.
 * 
 *          THROTTLE MIX CONCEPT:
 *          - Low throttle mix: Prioritizes throttle/altitude control over attitude
 *          - High throttle mix: Prioritizes attitude control over throttle
 *          - Only affects control when total throttle is below hover throttle
 * 
 *          THROTTLE MIX STATES:
 *          
 *          MIN (prioritize throttle):
 *          - Disarmed or landed: Attitude control less critical
 *          - Manual throttle at zero (non-airmode): Pilot wants minimal thrust
 *          - Autopilot descent without aggressive maneuvers: Smooth landing approach
 * 
 *          MAN (manual balanced mix):
 *          - Manual throttle above zero: Pilot actively controlling
 *          - Airmode enabled: Always maintain full attitude authority
 * 
 *          MAX (prioritize attitude):
 *          - Large angle requests: Aggressive maneuvering requires attitude authority
 *          - Large angle errors: Fighting external disturbances
 *          - High acceleration: Rapid flight changes
 *          - Not commanding descent: Maintaining or gaining altitude
 *          - Force flying flag set: Pilot overriding landing detection
 * 
 *          SPECIAL CONSIDERATIONS:
 *          - Helicopter frame: Not applicable, function returns early
 *          - Airmode: Always maintains high attitude authority during flight
 *          - Landing mode: Gradually reduces attitude authority for smooth touchdown
 * 
 * @note Called at MAIN_LOOP_RATE (typically 400Hz)
 * @note Only applicable to multicopter frames, not helicopters
 * 
 * @see AC_AttitudeControl::set_throttle_mix_min()
 * @see AC_AttitudeControl::set_throttle_mix_man()
 * @see AC_AttitudeControl::set_throttle_mix_max()
 * 
 * Source: ArduCopter/land_detector.cpp:281-324
 */
void Copter::update_throttle_mix()
{
#if FRAME_CONFIG != HELI_FRAME
    // CASE 1: Disarmed or landed - minimize attitude control authority
    // On ground, attitude control is less critical than preventing motor output
    // Prioritize throttle to ensure motors stay at minimum
    if (!motors->armed() || ap.land_complete) {
        attitude_control->set_throttle_mix_min();
        return;
    }

    // CASE 2: Manual throttle modes (STABILIZE, ACRO, etc.)
    if (flightmode->has_manual_throttle()) {
        // Check pilot throttle stick position
        if (channel_throttle->get_control_in() <= 0 && air_mode != AirMode::AIRMODE_ENABLED) {
            // Throttle stick at zero and not in airmode - minimize attitude authority
            // Allows clean landing without attitude controller fighting for control
            attitude_control->set_throttle_mix_min();
        } else {
            // Throttle stick above zero OR airmode enabled - balanced manual mix
            // Maintains good attitude control while respecting pilot throttle input
            attitude_control->set_throttle_mix_man();
        }
    } 
    // CASE 3: Autopilot controlled throttle (LOITER, AUTO, GUIDED, etc.)
    else {
        // Evaluate flight conditions to determine appropriate throttle mix
        
        // Check for aggressive flight maneuvers - large angle requests indicate
        // need for strong attitude control authority
        const Vector3f& angle_target_rad = attitude_control->get_att_target_euler_rad();
        bool large_angle_request = angle_target_rad.xy().length() > cd_to_rad(LAND_CHECK_LARGE_ANGLE_CD);

        // Check for large attitude tracking errors - indicates external disturbances
        // or control difficulties requiring maximum attitude authority
        const float angle_error = attitude_control->get_att_error_angle_deg();
        bool large_angle_error = (angle_error > LAND_CHECK_ANGLE_ERROR_DEG);

        // Check for high acceleration - indicates rapid flight changes, falling,
        // or high turbulence requiring strong attitude control
        const bool accel_moving = (land_accel_ef_filter.get().length() > LAND_CHECK_ACCEL_MOVING);

        // Check if autopilot is commanding descent (negative Z velocity in NED frame)
        // If not descending, need altitude authority so maximize attitude control
        bool descent_not_demanded = pos_control->get_vel_desired_NEU_cms().z >= 0.0f;

        // Check if flight mode indicates active landing sequence
        const bool landing = flightmode->is_landing();

        // THROTTLE MIX DECISION LOGIC
        // Maximize attitude control (throttle mix max) if ANY of these conditions:
        // - Large angle request (unless actively landing)
        // - Force flying flag set (unless actively landing) - pilot override
        // - Large angle error (fighting disturbances)
        // - High acceleration (rapid flight changes)
        // - Not commanding descent (need altitude authority)
        if (((large_angle_request || get_force_flying()) && !landing) || large_angle_error || accel_moving || descent_not_demanded) {
            // Need maximum attitude authority - prioritize attitude control
            // Pass velocity control ratio to scale authority appropriately
            attitude_control->set_throttle_mix_max(pos_control->get_vel_U_control_ratio());
        } else {
            // Smooth descent or landing - minimize attitude authority
            // Allows throttle to dominate for smooth altitude changes
            attitude_control->set_throttle_mix_min();
        }
    }
#endif  // FRAME_CONFIG != HELI_FRAME
}

/**
 * @brief Get force flying flag state with frame-specific overrides
 * 
 * @details This helper function returns the effective "force flying" state, which
 *          indicates the pilot or flight mode is explicitly commanding the vehicle
 *          to treat itself as flying, overriding normal landing detection.
 * 
 *          USAGE:
 *          The force_flying flag is used to:
 *          - Prevent landing detection during intentional low hover
 *          - Maintain flight state during proximity to ground
 *          - Override landing detection when pilot wants full control
 * 
 *          FRAME-SPECIFIC OVERRIDES:
 *          - Helicopters: Automatically set force_flying during inverted flight
 *            (inverted flight requires full attitude control and should never
 *            trigger landing detection even if near ground)
 *          - Other frames: Return base force_flying flag value
 * 
 * @return true if vehicle should be considered flying regardless of landing criteria
 * @return false if vehicle can use normal landing detection
 * 
 * @note This is a const function - does not modify vehicle state
 * 
 * @see update_land_detector() where this flag affects landing criteria
 * @see update_throttle_mix() where this flag affects control mixing
 * 
 * Source: ArduCopter/land_detector.cpp:327-335
 */
bool Copter::get_force_flying() const
{
#if FRAME_CONFIG == HELI_FRAME
    // Helicopters in inverted flight must be considered flying
    // Landing detection should never trigger during inverted maneuvers
    if (attitude_control->get_inverted_flight()) {
        return true;
    }
#endif
    // Return base force_flying flag (set by flight modes or pilot commands)
    return force_flying;
}
