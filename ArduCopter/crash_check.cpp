/**
 * @file crash_check.cpp
 * @brief Crash detection and emergency response systems for ArduCopter
 * 
 * @details This file implements safety-critical crash detection algorithms and emergency
 *          response mechanisms including:
 *          - Crash detection via angle error and acceleration monitoring
 *          - Thrust loss detection and automatic compensation
 *          - Yaw imbalance detection (motor/ESC issues)
 *          - Parachute deployment for loss of control
 *          - Vibration-based impact detection
 * 
 *          The crash detection system distinguishes between:
 *          - **Crash**: Disarms motors immediately to prevent damage
 *          - **Parachute deployment**: Keeps motors running while deploying parachute
 * 
 *          All detection algorithms include false-positive prevention for aggressive
 *          maneuvers and incorporate multiple sensor inputs for robustness.
 * 
 * @warning These are safety-critical systems. Modifications must be thoroughly tested
 *          in SITL and on physical hardware. Incorrect thresholds can cause:
 *          - False positives (nuisance disarms during aggressive flight)
 *          - False negatives (failure to detect actual crashes)
 * 
 * @note All check functions are called at MAIN_LOOP_RATE (typically 400Hz)
 * 
 * @see Copter::crash_check()
 * @see Copter::parachute_check()
 * @see Copter::thrust_loss_check()
 */

#include "Copter.h"

// Crash Detection Thresholds
// These constants define the criteria for detecting a vehicle crash

/**
 * @brief Duration threshold for crash confirmation
 * @details Vehicle must meet all crash criteria continuously for this duration
 *          before motors are disarmed. This prevents false positives during
 *          aggressive maneuvers or momentary control loss.
 * @warning Increasing this value delays crash detection; decreasing may cause
 *          false positives during aggressive flight
 */
#define CRASH_CHECK_TRIGGER_SEC         2       // 2 seconds inverted indicates a crash

/**
 * @brief Maximum angle error before considering vehicle out of control
 * @details Angle error is the difference between commanded attitude and actual
 *          attitude (lean angle). 30 degrees indicates severe loss of control.
 *          Combined with lean angle check to distinguish crashes from level flight.
 * @warning Lower values may trigger on windy conditions or aggressive maneuvers
 */
#define CRASH_CHECK_ANGLE_DEVIATION_DEG 30.0f   // 30 degrees beyond target angle is signal we are out of control

/**
 * @brief Minimum lean angle required to trigger crash detection
 * @details Prevents false crash detection when vehicle is nearly level.
 *          A crashed vehicle on the ground typically leans significantly.
 * @note Crashes at very shallow angles (<15°) will not trigger crash detection
 */
#define CRASH_CHECK_ANGLE_MIN_DEG       15.0f   // vehicle must be leaning at least 15deg to trigger crash check

/**
 * @brief Maximum horizontal velocity for crash detection
 * @details A vehicle moving faster than 10 m/s is likely flying, not crashed.
 *          This prevents false positives during high-speed flight with momentary
 *          attitude deviations.
 * @warning Velocity estimate requires GPS or optical flow. Check may not work
 *          indoors without optical flow sensor.
 */
#define CRASH_CHECK_SPEED_MAX           10.0f   // vehicle must be moving at less than 10m/s to trigger crash check

/**
 * @brief Maximum acceleration threshold for crash detection
 * @details 1Hz filtered acceleration (with 1G on Z-axis subtracted). Values below
 *          3 m/s² indicate vehicle is nearly stationary - consistent with a crash.
 *          High acceleration indicates active flight or impact (ruled out by other checks).
 * @note Filter frequency prevents vibration from affecting this check
 */
#define CRASH_CHECK_ACCEL_MAX           3.0f    // vehicle must be accelerating less than 3m/s/s to be considered crashed

// Thrust Loss Detection Thresholds
// These constants detect motor/ESC failures causing insufficient thrust

/**
 * @brief Duration threshold for thrust loss confirmation
 * @details Vehicle must be descending with high throttle and level attitude for
 *          this duration before thrust loss is declared and boost mode activated.
 * @warning Thrust boost increases motor output; false positives can stress ESCs
 */
#define THRUST_LOSS_CHECK_TRIGGER_SEC         1     // 1 second descent while level and high throttle indicates thrust loss

/**
 * @brief Maximum angle deviation for thrust loss detection
 * @details Thrust loss can only be reliably detected when vehicle is nearly level
 *          (angle target < 15°). At higher angles, reduced vertical thrust is expected.
 * @note Value in centidegrees (1500 cd = 15°)
 */
#define THRUST_LOSS_CHECK_ANGLE_DEVIATION_CD  1500  // we can't expect to maintain altitude beyond 15 degrees on all aircraft

/**
 * @brief Minimum throttle level for thrust loss detection
 * @details Vehicle must be commanding >90% throttle or hitting throttle saturation.
 *          Below this, descent may be intentional or due to low throttle command.
 * @warning Setting too low may cause false positives during normal descent
 */
#define THRUST_LOSS_CHECK_MINIMUM_THROTTLE    0.9f  // we can expect to maintain altitude above 90 % throttle

// Yaw Imbalance Detection Thresholds
// These constants detect motor/ESC calibration issues or misaligned motors

/**
 * @brief Yaw I-term threshold as fraction of I-max
 * @details When low-pass filtered yaw I-term exceeds 75% of configured I-max,
 *          indicates persistent yaw control effort suggesting motor imbalance.
 * @warning High I-term is expected during yaw maneuvers; filter prevents false warnings
 */
#define YAW_IMBALANCE_IMAX_THRESHOLD 0.75f

/**
 * @brief Minimum interval between yaw imbalance warnings (milliseconds)
 * @details Prevents spamming GCS with warnings; allows time for pilot to address issue
 */
#define YAW_IMBALANCE_WARN_MS 10000

/**
 * @brief Detect vehicle crash and disarm motors for safety
 * 
 * @details This function implements multi-criteria crash detection to automatically
 *          disarm the vehicle when a crash is detected, preventing propeller damage
 *          and potential injury. A crash is identified by the simultaneous occurrence
 *          of multiple conditions over a sustained period:
 * 
 *          **Primary Detection Algorithm**:
 *          1. **Angle Error**: Actual lean angle differs from commanded angle by >30°
 *             - Indicates loss of attitude control
 *             - Distinguishes crashes from intentional aggressive maneuvers
 *          
 *          2. **Minimum Lean Angle**: Vehicle leaning >15° from level
 *             - Crashed vehicles typically lean significantly
 *             - Prevents false positives during level flight
 *          
 *          3. **Low Acceleration**: Filtered acceleration <3 m/s² (1Hz filter, 1G subtracted)
 *             - Indicates vehicle is nearly stationary (consistent with ground impact)
 *             - Active flight shows higher accelerations
 *             - Filter prevents vibration spikes from triggering detection
 *          
 *          4. **Low Velocity**: Horizontal velocity <10 m/s (if GPS/flow available)
 *             - Flying vehicles typically move faster
 *             - Prevents false positives during high-speed flight
 *          
 *          5. **Duration**: All criteria must be met continuously for 2 seconds
 *             - Filters out momentary control loss or sensor glitches
 *             - Prevents nuisance disarms during aggressive maneuvers
 * 
 *          **False Positive Prevention**:
 *          - Disabled during landing (ap.land_complete)
 *          - Disabled during force_flying state (except when landing)
 *          - Disabled in flight modes without attitude stabilization
 *          - Disabled during acrobatic maneuvers (flips, rolls)
 *          - Disabled in autorotation mode (helicopter emergency descent)
 *          - High acceleration resets counter (indicates active flight)
 *          - High velocity resets counter (indicates controlled flight)
 * 
 *          **Crash vs Parachute Deployment**:
 *          - Crash detection: Disarms motors immediately (prevents damage)
 *          - Parachute system: Keeps motors running (stabilize during descent)
 *          - Different trigger criteria and response actions
 * 
 * @return void
 * 
 * @note Called at MAIN_LOOP_RATE (typically 400Hz) from main flight loop
 * @note Requires g.fs_crash_check != 0 to be enabled (parameter FS_CRASH_CHECK)
 * @note Crash counter is static - persists between calls to track duration
 * 
 * @warning Disarming during flight is extremely dangerous. All conditions must
 *          be met to minimize false positives. Test thoroughly in SITL before
 *          modifying thresholds.
 * @warning GPS velocity check may not function indoors without optical flow sensor
 * @warning Modifying CRASH_CHECK_TRIGGER_SEC affects crash detection latency vs
 *          false positive rate trade-off
 * 
 * @see Copter::parachute_check() for alternate emergency response
 * @see CRASH_CHECK_TRIGGER_SEC, CRASH_CHECK_ANGLE_DEVIATION_DEG
 * @see CRASH_CHECK_ANGLE_MIN_DEG, CRASH_CHECK_SPEED_MAX, CRASH_CHECK_ACCEL_MAX
 */
void Copter::crash_check()
{
    static uint16_t crash_counter;  // number of iterations vehicle may have been crashed

    // Return immediately if disarmed, landed, or crash checking disabled via parameter
    // Crash detection only applies during armed flight
    if (!motors->armed() || ap.land_complete || g.fs_crash_check == 0) {
        crash_counter = 0;
        return;
    }

    // Exit immediately if in standby (motors armed but not spinning)
    // Standby mode is intentional low-power state, not a crash
    if (standby_active) {
        crash_counter = 0;
        return;
    }

    // Exit if force_flying is active (manual override indicating vehicle is flying)
    // Exception: Allow crash detection during landing even with force_flying
    if (get_force_flying() && !flightmode->is_landing()) {
        crash_counter = 0;
        return;
    }

    // Return if flight mode doesn't support crash detection
    // Modes like FLIP, ACRO_TRAINER intentionally exceed normal angle limits
    // flightmode->crash_check_enabled() returns false for these modes
    if (!flightmode->crash_check_enabled()) {
        crash_counter = 0;
        return;
    }

#if MODE_AUTOROTATE_ENABLED
    // Return immediately if in autorotation mode (helicopter emergency descent)
    // Autorotation intentionally descends with unpowered rotors - appears like crash
    if (flightmode->mode_number() == Mode::Number::AUTOROTATE) {
        crash_counter = 0;
        return;
    }
#endif

    // Check acceleration criterion: vehicle must be nearly stationary
    // land_accel_ef_filter is 1Hz filtered earth-frame acceleration (1G Z-axis subtracted)
    // High acceleration indicates active flight, impacts, or vibration from spinning motors
    // Crashed vehicle on ground shows low filtered acceleration
    const float filtered_acc = land_accel_ef_filter.get().length();
    if (filtered_acc >= CRASH_CHECK_ACCEL_MAX) {
        crash_counter = 0;
        return;
    }

    // Check lean angle criterion: vehicle must be leaning significantly
    // Calculate total lean angle from roll and pitch using: angle = acos(cos_roll * cos_pitch)
    // Crashed vehicles typically lean >15°; level flight or shallow banks reset counter
    const float lean_angle_deg = degrees(acosf(ahrs.cos_roll()*ahrs.cos_pitch()));
    if (lean_angle_deg <= CRASH_CHECK_ANGLE_MIN_DEG) {
        crash_counter = 0;
        return;
    }

    // Check angle error criterion: attitude control must have failed
    // Angle error = |commanded_attitude - actual_attitude|
    // Large angle error (>30°) indicates severe loss of control
    // Small angle error means vehicle is tracking commanded attitude (intentional maneuver)
    const float angle_error = attitude_control->get_att_error_angle_deg();
    if (angle_error <= CRASH_CHECK_ANGLE_DEVIATION_DEG) {
        crash_counter = 0;
        return;
    }

    // Check velocity criterion: vehicle must be moving slowly
    // Velocity check requires GPS or optical flow
    // High velocity indicates controlled flight; crashes occur at low speeds
    // NED frame: North-East-Down velocity vector
    Vector3f vel_ned;
    if (ahrs.get_velocity_NED(vel_ned) && (vel_ned.length() >= CRASH_CHECK_SPEED_MAX)) {
        crash_counter = 0;
        return;
    }

    // All crash criteria are met - increment counter
    // Counter tracks consecutive iterations meeting crash conditions
    crash_counter++;

    // Check if crash conditions persisted for required duration
    // Duration = CRASH_CHECK_TRIGGER_SEC * loop_rate (e.g., 2 sec * 400 Hz = 800 iterations)
    // This filters out momentary control loss and sensor glitches
    if (crash_counter >= (CRASH_CHECK_TRIGGER_SEC * scheduler.get_loop_rate_hz())) {
        LOGGER_WRITE_ERROR(LogErrorSubsystem::CRASH_CHECK, LogErrorCode::CRASH_CHECK_CRASH);
        // Send detailed crash message to ground station with triggering values
        gcs().send_text(MAV_SEVERITY_EMERGENCY,"Crash: Disarming: AngErr=%.0f>%.0f, Accel=%.1f<%.1f", angle_error, CRASH_CHECK_ANGLE_DEVIATION_DEG, filtered_acc, CRASH_CHECK_ACCEL_MAX);
        // Disarm motors immediately to prevent propeller damage and injury
        // AP_Arming::Method::CRASH provides specific disarm reason for logging
        copter.arming.disarm(AP_Arming::Method::CRASH);
    }
}

/**
 * @brief Detect loss of thrust and trigger automatic thrust boost compensation
 * 
 * @details This function detects motor or ESC failures that cause insufficient thrust,
 *          then triggers the motors library thrust boost feature to compensate by
 *          increasing output to remaining motors. This is distinct from crash detection:
 *          - Thrust loss: Motors continue running with boost to maintain control
 *          - Crash: Motors disarm immediately
 * 
 *          **Detection Algorithm**:
 *          Thrust loss is identified when ALL conditions persist for 1 second:
 * 
 *          1. **Level Attitude**: Commanded angle <15° from horizontal
 *             - At steep angles, reduced vertical thrust is normal
 *             - Level flight should maintain altitude with available thrust
 *          
 *          2. **High Throttle**: Throttle >90% or throttle saturation
 *             - Pilot/controller demanding maximum available thrust
 *             - Throttle >25% minimum to avoid false positives from low commanded throttle
 *          
 *          3. **Descending**: Positive vertical velocity (down)
 *             - Despite high throttle, vehicle unable to maintain altitude
 *             - Velocity estimate must be available and reliable (not high-vibes)
 *          
 *          4. **Good Attitude Control**: Angle error <30°
 *             - Vehicle still has attitude control (not crashed)
 *             - Distinguishes thrust loss from complete loss of control
 * 
 *          **Response Action**:
 *          - Enables thrust boost in motors library (motors->set_thrust_boost(true))
 *          - Motor mixer redistributes thrust to compensate for failed motor
 *          - Optionally releases gripper if configured (FlightOption::RELEASE_GRIPPER_ON_THRUST_LOSS)
 *          - Boost automatically disabled by motors library when no longer needed
 * 
 *          **False Positive Prevention**:
 *          - Exits if already in thrust boost mode
 *          - Checks minimum throttle (25%) to avoid false detection during descent
 *          - Requires reliable velocity estimate (rejects high-vibration conditions)
 *          - Requires good attitude control (not in crash condition)
 *          - Can be disabled via FlightOption::DISABLE_THRUST_LOSS_CHECK
 * 
 * @return void
 * 
 * @note Called at MAIN_LOOP_RATE (typically 400Hz) from main flight loop
 * @note Thrust boost is automatically disabled by motors library when conditions resolve
 * @note Detection requires valid vertical velocity estimate from AHRS
 * 
 * @warning Thrust boost increases motor output to maximum available. In extreme cases
 *          (multiple motor failures) may not provide sufficient control authority.
 * @warning False positives can stress ESCs and drain battery rapidly
 * @warning Requires accurate velocity estimation; may not work reliably in high-vibration
 * 
 * @see AP_Motors::set_thrust_boost()
 * @see AP_Motors::get_lost_motor()
 * @see THRUST_LOSS_CHECK_TRIGGER_SEC, THRUST_LOSS_CHECK_ANGLE_DEVIATION_CD
 * @see THRUST_LOSS_CHECK_MINIMUM_THROTTLE
 */
void Copter::thrust_loss_check()
{
    static uint16_t thrust_loss_counter;  // number of iterations vehicle may have lost thrust

    // Exit if thrust loss check is disabled via flight option parameter
    // Allows disabling this feature if causing false positives on specific platforms
    if (copter.option_is_enabled(FlightOption::DISABLE_THRUST_LOSS_CHECK)) {
        return;
    }

    // Exit immediately if thrust boost is already engaged
    // Prevents re-triggering and allows motors library to manage boost state
    if (motors->get_thrust_boost()) {
        return;
    }

    // Return immediately if disarmed or landed
    // Thrust loss only relevant during armed flight
    if (!motors->armed() || ap.land_complete) {
        thrust_loss_counter = 0;
        return;
    }

    // Exit immediately if in standby mode
    // Standby intentionally reduces motor output
    if (standby_active) {
        return;
    }

    // Check attitude target criterion: vehicle must be commanded nearly level
    // angle_target_rad.xy() gives roll and pitch targets (yaw not relevant)
    // At steep angles (>15°), vertical thrust component is reduced (cosine losses)
    // Thrust loss can only be reliably detected when flying level
    // todo: add thrust angle to AC_AttitudeControl for more accurate check
    const Vector3f& angle_target_rad = attitude_control->get_att_target_euler_rad();
    if (angle_target_rad.xy().length_squared() > sq(cd_to_rad(THRUST_LOSS_CHECK_ANGLE_DEVIATION_CD))) {
        thrust_loss_counter = 0;
        return;
    }

    // Check high throttle criterion: throttle >90% or hitting upper throttle limit
    // motors->limit.throttle_upper indicates saturation (requesting more than available)
    // Both conditions indicate pilot/controller demanding maximum thrust
    // If throttle is low, descent is likely intentional
    if ((attitude_control->get_throttle_in() < THRUST_LOSS_CHECK_MINIMUM_THROTTLE) && (!motors->limit.throttle_upper)) {
        thrust_loss_counter = 0;
        return;
    }

    // Check minimum throttle to prevent false positives
    // At very low throttle (<25%), vehicle may descend due to insufficient commanded thrust
    // This is different from thrust loss (where commanded thrust is not achieved)
    // Prevents thrust boost from engaging during intentional slow descents
    if ((attitude_control->get_throttle_in() < 0.25f)) {
        thrust_loss_counter = 0;
        return;
    }

    // Check descent criterion: vehicle must be descending (positive D velocity in NED frame)
    // get_velocity_D returns vertical velocity in m/s (positive = down)
    // Rejects velocity estimate if high vibrations detected (unreliable)
    // Descent despite high throttle and level attitude indicates insufficient thrust
    float vel_d_ms = 0;
    if (!AP::ahrs().get_velocity_D(vel_d_ms, vibration_check.high_vibes) || !is_positive(vel_d_ms)) {
        // we have no vertical velocity estimate and/or we are not descending
        thrust_loss_counter = 0;
        return;
    }

    // Check attitude control criterion: angle error must be low (<30°)
    // This distinguishes thrust loss (good attitude control, insufficient thrust)
    // from complete loss of control (crash condition, would trigger crash_check instead)
    // Vehicle with thrust loss can still control attitude, just not altitude
    const float angle_error = attitude_control->get_att_error_angle_deg();
    if (angle_error >= CRASH_CHECK_ANGLE_DEVIATION_DEG) {
        thrust_loss_counter = 0;
        return;
    }

    // All thrust loss criteria are met - increment counter
    // Vehicle is: level, high throttle, descending, good attitude control
    // This pattern strongly suggests motor/ESC failure reducing available thrust
    thrust_loss_counter++;

    // Check if thrust loss conditions persisted for required duration
    // Duration = THRUST_LOSS_CHECK_TRIGGER_SEC * loop_rate (e.g., 1 sec * 400 Hz = 400 iterations)
    // Filters out momentary conditions like brief wind gusts
    if (thrust_loss_counter >= (THRUST_LOSS_CHECK_TRIGGER_SEC * scheduler.get_loop_rate_hz())) {
        // Reset counter to prevent repeated triggers
        thrust_loss_counter = 0;
        LOGGER_WRITE_ERROR(LogErrorSubsystem::THRUST_LOSS_CHECK, LogErrorCode::FAILSAFE_OCCURRED);
        // Send notification to ground station with suspected motor number
        // motors->get_lost_motor() returns index of suspected failed motor (0-based)
        gcs().send_text(MAV_SEVERITY_EMERGENCY, "Potential Thrust Loss (%d)", (int)motors->get_lost_motor() + 1);
        // Enable thrust boost mode in motors library
        // Motor mixer will compensate by increasing output to remaining motors
        motors->set_thrust_boost(true);
        // The motors library automatically disables thrust boost when it is no longer
        // needed to achieve the commanded output (conditions resolve or vehicle lands)

#if AP_GRIPPER_ENABLED
        // Optionally release gripper payload to reduce weight and improve control
        // Configured via FlightOption::RELEASE_GRIPPER_ON_THRUST_LOSS parameter
        if (copter.option_is_enabled(FlightOption::RELEASE_GRIPPER_ON_THRUST_LOSS)) {
            gripper.release();
        }
#endif
    }
}

/**
 * @brief Detect and warn about yaw control imbalance indicating motor/ESC issues
 * 
 * @details This function monitors the yaw rate controller's integral term to detect
 *          persistent yaw control effort, which indicates motor imbalance problems:
 *          - Badly calibrated ESCs (inconsistent thrust across motors)
 *          - Physically misaligned motors (thrust not perpendicular to arms)
 *          - Damaged propellers (asymmetric thrust)
 *          - Motor timing issues (reduced efficiency on some motors)
 * 
 *          **Detection Algorithm**:
 *          Yaw imbalance is identified when:
 * 
 *          1. **High I-term**: Low-pass filtered yaw I-term >75% of configured I-max
 *             - I-term accumulates when persistent yaw error exists
 *             - High I-term indicates continuous corrective effort to counter imbalance
 *             - Filter prevents false warnings during intentional yaw maneuvers
 *          
 *          2. **I-term at Saturation**: Unfiltered I-term equals I-max
 *             - Indicates I-term has hit limit (extreme imbalance)
 *             - Immediate warning even if filter hasn't caught up
 *          
 *          3. **I Enabled**: Yaw rate PID has non-zero I gain
 *             - If I is disabled, problem is obvious from vehicle behavior
 *             - Check only meaningful when integral control is active
 * 
 *          **Response Action**:
 *          - Sends warning message to ground station every 10 seconds
 *          - Message shows percentage of available yaw control used for imbalance
 *          - No automatic corrective action (requires physical repair)
 * 
 *          **False Positive Prevention**:
 *          - Low-pass filter on I-term (smooths transients from maneuvers)
 *          - Minimum interval between warnings (10 seconds)
 *          - Disabled during thrust loss (yaw issues expected)
 *          - Disabled when disarmed or landed
 *          - Filter reset to zero if filtered value exceeds unfiltered
 * 
 * @return void
 * 
 * @note Called at MAIN_LOOP_RATE (typically 400Hz) from main flight loop
 * @note Warning interval controlled by YAW_IMBALANCE_WARN_MS (10 seconds)
 * @note Does not affect flight control, only provides diagnostic information
 * 
 * @warning High yaw imbalance reduces control authority and can make vehicle
 *          difficult to control, especially in windy conditions
 * @warning Requires physical diagnosis and repair (ESC calibration, motor alignment)
 * @warning Can be disabled via FlightOption::DISABLE_YAW_IMBALANCE_WARNING if causing
 *          nuisance warnings on asymmetric frame designs
 * 
 * @see AC_PID::get_pid_info()
 * @see YAW_IMBALANCE_IMAX_THRESHOLD, YAW_IMBALANCE_WARN_MS
 */
void Copter::yaw_imbalance_check()
{
    // Exit if yaw imbalance warning is disabled via flight option parameter
    // Can be disabled if causing nuisance warnings on asymmetric frame designs
    if (copter.option_is_enabled(FlightOption::DISABLE_YAW_IMBALANCE_WARNING)) {
        return;
    }

    // If yaw rate I gain is disabled, imbalance problems are obvious from behavior
    // Check only useful when integral control is active and subtly compensating
    // kI() returns the integral gain from yaw rate PID controller
    if (!is_positive(attitude_control->get_rate_yaw_pid().kI())) {
        return;
    }

    // Exit if thrust loss detected - yaw control issues are expected in this condition
    // Missing thrust from one motor causes torque imbalance requiring yaw compensation
    // Reset filter to prevent false warnings when thrust boost mode ends
    if (motors->get_thrust_boost()) {
        yaw_I_filt.reset(0.0f);
        return;
    }

    // Return immediately if disarmed or landed
    // Yaw imbalance only relevant during flight
    // Reset filter for clean start on next flight
    if (!motors->armed() || ap.land_complete) {
        yaw_I_filt.reset(0.0f);
        return;
    }

    // Exit immediately if in standby mode
    // Standby is low-power state, not representative of normal flight
    if (standby_active) {
        yaw_I_filt.reset(0.0f);
        return;
    }

    // Get current I-term from yaw rate PID controller
    // I_term accumulates when yaw rate error persists (integral of error over time)
    // Large persistent I-term indicates continuous corrective effort
    const float I_term = attitude_control->get_rate_yaw_pid().get_pid_info().I;
    
    // Apply low-pass filter to I-term to smooth out transients
    // Filter prevents false warnings during intentional yaw maneuvers
    // fabsf() takes absolute value (magnitude regardless of direction)
    // G_Dt is the loop time step for filter update
    const float I = fabsf(yaw_I_filt.apply(attitude_control->get_rate_yaw_pid().get_pid_info().I,G_Dt));
    
    // Clamp filtered value to not exceed unfiltered magnitude
    // Prevents filter from amplifying signal or lagging too far behind reality
    if (I > fabsf(I_term)) {
        yaw_I_filt.reset(I_term);
    }

    // Get maximum I-term value configured for yaw rate controller
    // I_max limits how much integral term can contribute to control output
    const float I_max = attitude_control->get_rate_yaw_pid().imax();
    
    // Check if I-term indicates significant yaw imbalance
    // Two trigger conditions (either one triggers warning):
    // 1. Filtered I-term >75% of I_max (persistent high integral effort)
    // 2. Unfiltered I-term equals I_max (I-term saturated at limit)
    if ((is_positive(I_max) && ((I > YAW_IMBALANCE_IMAX_THRESHOLD * I_max) || (is_equal(I_term,I_max))))) {
        // I-term is using large portion of available yaw control authority
        // This means significant yaw compensation is required to maintain heading
        // Indicates motor imbalance that should be diagnosed and repaired
        const uint32_t now = millis();
        
        // Rate-limit warnings to once per YAW_IMBALANCE_WARN_MS (10 seconds)
        // Prevents spamming ground station with repeated warnings
        if (now - last_yaw_warn_ms > YAW_IMBALANCE_WARN_MS) {
            last_yaw_warn_ms = now;
            // Send warning with percentage of total yaw control used for imbalance compensation
            // High percentages (>75%) indicate vehicle is using most of its yaw authority
            // just to maintain straight flight, leaving little margin for intentional yaw
            gcs().send_text(MAV_SEVERITY_EMERGENCY, "Yaw Imbalance %0.0f%%", I *100);
        }
    }
}

#if HAL_PARACHUTE_ENABLED

// Parachute Deployment Thresholds
// These constants define criteria for automatic parachute deployment

/**
 * @brief Duration threshold for parachute deployment due to loss of control
 * @details Vehicle must have angle error >30° continuously for this duration,
 *          AND be falling (descending in altitude) before parachute deploys.
 * @warning Shorter duration increases false positive risk; longer duration may
 *          deploy parachute too late to be effective
 */
#define PARACHUTE_CHECK_TRIGGER_SEC         1       // 1 second of loss of control triggers the parachute

/**
 * @brief Maximum angle error before considering loss of control for parachute
 * @details Angle error >30° indicates severe attitude control failure.
 *          This is the same threshold as crash detection but with different response:
 *          - Crash detection: Disarms motors (vehicle on ground)
 *          - Parachute: Keeps motors running (vehicle airborne, attempting to slow descent)
 * @note Parachute also checks vehicle is falling via barometric altitude
 */
#define PARACHUTE_CHECK_ANGLE_DEVIATION_DEG 30.0f   // 30 degrees off from target indicates a loss of control

/**
 * @brief Detect loss of control and trigger parachute deployment for emergency descent
 * 
 * @details This function monitors for severe loss of attitude control and automatically
 *          deploys the parachute when recovery is unlikely. This is DISTINCT from crash
 *          detection in critical ways:
 * 
 *          **Parachute vs Crash Detection**:
 *          - **Parachute**: Vehicle airborne, falling, motors KEEP RUNNING to slow descent
 *          - **Crash**: Vehicle on ground, motors DISARM immediately
 *          - **Purpose**: Parachute provides emergency descent system for mid-air failures
 * 
 *          **Detection Algorithm**:
 *          Loss of control requiring parachute deployment identified when:
 * 
 *          1. **Large Angle Error**: Attitude error >30° from commanded attitude
 *             - Indicates severe loss of attitude control
 *             - Control loss count increments each loop iteration
 *             - Count decrements if angle error returns to acceptable range
 *          
 *          2. **Vehicle is Falling**: Barometric altitude decreasing
 *             - Altitude check starts when control loss first detected
 *             - If altitude increases, control loss counter resets (vehicle recovering)
 *             - Prevents deployment when control lost but not falling
 *          
 *          3. **Above Minimum Altitude**: First control loss event must occur above
 *             configured minimum altitude (CHUTE_ALT_MIN parameter)
 *             - Prevents deployment too close to ground (parachute needs altitude to deploy)
 *             - Subsequent checks can continue below minimum altitude
 *          
 *          4. **Duration**: Conditions persist for 1 second (PARACHUTE_CHECK_TRIGGER_SEC)
 *             - Filters out momentary control loss
 *             - Shorter than crash detection (1s vs 2s) - airborne failures require faster response
 *          
 *          5. **Sink Rate Check**: Independent check via parachute.check_sink_rate()
 *             - Can trigger parachute based on excessive descent rate alone
 *             - Provides alternate trigger path independent of attitude
 * 
 *          **Additional Integration**:
 *          - Continuously updates parachute library with flight state (is_flying, sink_rate)
 *          - Parachute library manages servo/relay position
 *          - Once deployed, deployment cannot be cancelled
 * 
 *          **False Positive Prevention**:
 *          - Only active when parachute enabled (CHUTE_ENABLED parameter)
 *          - Only active during flight (not when landed)
 *          - Disabled in standby mode
 *          - Requires flight mode with attitude stabilization enabled
 *          - Decrements counter if angle error improves (allows recovery)
 *          - Checks altitude is decreasing (not just attitude error)
 * 
 * @return void
 * 
 * @note Called at MAIN_LOOP_RATE (typically 400Hz) from main flight loop
 * @note Parachute deployment is one-way - cannot be retracted after release
 * @note Landing gear deploys automatically with parachute (if equipped)
 * @note Motors continue running after deployment (attempting to slow descent)
 * 
 * @warning Parachute deployment is irreversible. Vehicle will descend under parachute.
 * @warning Requires sufficient altitude for parachute to deploy and inflate effectively
 * @warning False deployments can result in vehicle damage and mission failure
 * @warning Deployment does NOT disarm motors - they continue providing control/slowing
 * 
 * @see Copter::parachute_release()
 * @see Copter::parachute_manual_release()
 * @see Copter::crash_check() for ground impact detection
 * @see PARACHUTE_CHECK_TRIGGER_SEC, PARACHUTE_CHECK_ANGLE_DEVIATION_DEG
 */
void Copter::parachute_check()
{
    static uint16_t control_loss_count; // number of iterations we have been out of control
    static int32_t baro_alt_start;       // barometric altitude when control loss first detected

    // Exit immediately if parachute system is not enabled
    // Controlled by CHUTE_ENABLED parameter
    if (!parachute.enabled()) {
        return;
    }

    // Update parachute library with current flight state
    // is_flying flag allows parachute library to know if vehicle is airborne
    // Used by parachute library for sink rate checks and deployment decisions
    parachute.set_is_flying(!ap.land_complete);

    // Update parachute library with current descent rate
    // vel_d_ms is vertical velocity in NED frame (positive = down, m/s)
    // Rejects velocity if high vibrations detected (unreliable estimate)
    // Parachute library uses sink rate for alternative deployment trigger
    float vel_d_ms = 0;
    UNUSED_RESULT(AP::ahrs().get_velocity_D(vel_d_ms, vibration_check.high_vibes));
    parachute.set_sink_rate(vel_d_ms);

    // Exit immediately if in standby mode
    // Standby is intentional low-power state, not loss of control
    if (standby_active) {
        return;
    }

    // Call parachute update to service servo/relay positioning
    // Allows parachute mechanism to return to ready position if needed
    parachute.update();

    // Return immediately if motors are not armed
    // Parachute only relevant during armed flight
    // Reset counter for clean state on next arming
    if (!motors->armed()) {
        control_loss_count = 0;
        return;
    }

    // Exit if parachute release already initiated
    // Prevents redundant triggers once deployment sequence started
    // Deployment is irreversible
    if (parachute.release_initiated()) {
        return;
    }

    // Return if flight mode doesn't support crash checking
    // Modes like FLIP intentionally lose attitude control temporarily
    // flightmode->crash_check_enabled() returns false for acrobatic modes
    if (!flightmode->crash_check_enabled()) {
        control_loss_count = 0;
        return;
    }

    // Ensure we are flying (not landed)
    // Parachute deployment only relevant when airborne
    // Landed vehicle with attitude error should not deploy parachute
    if (ap.land_complete) {
        control_loss_count = 0;
        return;
    }

    // Ensure first control loss event occurs above minimum configured altitude
    // CHUTE_ALT_MIN parameter sets minimum altitude for parachute effectiveness
    // Parachute needs altitude to deploy and inflate before ground impact
    // current_loc.alt is in centimeters, parachute.alt_min() is in meters
    // Check only applies to first detection (control_loss_count == 0)
    // Once counting starts, can continue below minimum (vehicle already falling)
    if (control_loss_count == 0 && parachute.alt_min() != 0 && (current_loc.alt < (int32_t)parachute.alt_min() * 100)) {
        return;
    }

    // Check if sink rate alone exceeds threshold for deployment
    // Independent trigger path from attitude-based detection
    // Parachute library compares sink_rate against configured threshold
    // Can deploy parachute even if attitude control is good but descent excessive
    parachute.check_sink_rate();

    // Check angle error criterion: difference between commanded and actual attitude
    // angle_error >30° indicates loss of attitude control
    // If angle error acceptable, decrement counter (allowing recovery)
    const float angle_error = attitude_control->get_att_error_angle_deg();
    if (angle_error <= PARACHUTE_CHECK_ANGLE_DEVIATION_DEG) {
        // Angle error returned to acceptable range - vehicle may be recovering
        // Decrement counter (gradually forget control loss if recovering)
        // Prevents deployment if vehicle regains control before timeout
        if (control_loss_count > 0) {
            control_loss_count--;
        }
        return;
    }

    // Angle error exceeds threshold - increment control loss counter
    // Cap counter at trigger threshold to prevent overflow
    // Counter tracks consecutive iterations with excessive angle error
    if (control_loss_count < (PARACHUTE_CHECK_TRIGGER_SEC*scheduler.get_loop_rate_hz())) {
        control_loss_count++;
    }

    // Check if this is the first iteration of control loss
    // Record starting barometric altitude to verify vehicle is actually falling
    // Altitude check distinguishes falling from stationary control loss
    if (control_loss_count == 1) {
        baro_alt_start = baro_alt;

    // Check if barometric altitude indicates we are falling
    // baro_alt in NED-like convention where down is positive change
    // If altitude not decreasing (baro_alt >= start), we're not falling
    // Reset counter if not falling - control loss without descent may be recoverable
    } else if (baro_alt >= baro_alt_start) {
        control_loss_count = 0;
        return;

    // To-Do: add check that the vehicle is actually falling
    // Current implementation uses baro altitude; could enhance with:
    // - Vertical velocity (already available in vel_d_ms)
    // - Accelerometer vertical component
    // - GPS altitude rate of change

    // Check if loss of control persisted for required duration
    // Duration = PARACHUTE_CHECK_TRIGGER_SEC * loop_rate (e.g., 1 sec * 400 Hz = 400 iterations)
    // Shorter than crash detection (1s vs 2s) because airborne failures require faster response
    } else if (control_loss_count >= (PARACHUTE_CHECK_TRIGGER_SEC*scheduler.get_loop_rate_hz())) {
        // Reset control loss counter to prevent repeated triggers
        control_loss_count = 0;
        LOGGER_WRITE_ERROR(LogErrorSubsystem::CRASH_CHECK, LogErrorCode::CRASH_CHECK_LOSS_OF_CONTROL);
        // Deploy parachute - initiates irreversible emergency descent procedure
        // Motors continue running to provide additional control and slow descent
        parachute_release();
    }
}

/**
 * @brief Trigger parachute deployment and prepare vehicle for emergency descent
 * 
 * @details This function initiates the parachute deployment sequence. Unlike the function
 *          name comment suggests, this does NOT disarm motors - motors continue running
 *          to provide additional control authority and slow the descent rate.
 * 
 *          **Actions Performed**:
 *          1. Command parachute release (servo/relay actuation via parachute library)
 *          2. Deploy landing gear (if equipped) to prepare for ground impact
 * 
 *          **Motor Behavior**:
 *          - Motors remain ARMED and continue running
 *          - Attitude control continues attempting to stabilize vehicle
 *          - Motor thrust provides additional descent rate control
 *          - This is intentional: parachute + motors = slower, more controlled descent
 * 
 *          **Parachute Mechanism**:
 *          - Parachute library controls physical deployment (servo or relay)
 *          - Deployment is irreversible (one-way action)
 *          - Parachute library manages timing and actuation
 * 
 * @return void
 * 
 * @note Called by parachute_check() when automatic deployment criteria met
 * @note Also called by parachute_manual_release() for pilot-commanded deployment
 * @note Landing gear deployment is automatic (prepares for ground impact)
 * @note Motors do NOT disarm - they continue providing control
 * 
 * @warning Parachute deployment is irreversible and typically ends the flight mission
 * @warning Vehicle will descend under parachute; ensure safe landing area below
 * @warning Motors continue running - keep clear of propellers during descent
 * 
 * @see Copter::parachute_check() for automatic deployment trigger
 * @see Copter::parachute_manual_release() for pilot-initiated deployment
 * @see AP_Parachute::release()
 */
void Copter::parachute_release()
{
    // Trigger physical parachute release mechanism
    // Parachute library actuates configured servo or relay
    // Deployment sequence managed by parachute library
    parachute.release();

#if AP_LANDINGGEAR_ENABLED
    // Automatically deploy landing gear to prepare for ground impact
    // Extended landing gear provides better impact absorption
    // May reduce tip-over risk on landing
    landinggear.set_position(AP_LandingGear::LandingGear_Deploy);
#endif
}

/**
 * @brief Trigger pilot-commanded parachute deployment with safety checks
 * 
 * @details This function handles manual parachute deployment initiated by pilot via
 *          RC switch, GCS command, or other pilot input. Unlike automatic deployment
 *          (parachute_check), this includes safety checks to prevent pilot error:
 * 
 *          **Safety Checks Performed**:
 *          1. **Parachute Enabled**: CHUTE_ENABLED parameter must be set
 *          2. **Not Landed**: Prevents deployment on ground (ap.land_complete check)
 *          3. **Above Minimum Altitude**: Altitude must exceed CHUTE_ALT_MIN parameter
 * 
 *          **Rationale for Safety Checks**:
 *          - Deploying on ground wastes parachute, may cause damage
 *          - Deploying below minimum altitude gives insufficient time for parachute
 *            to inflate and slow descent effectively
 *          - Prevents accidental deployment due to pilot switch error
 * 
 *          **Pilot Error Prevention**:
 *          - Clear feedback messages sent to ground station on rejection
 *          - Error logging for post-flight analysis
 *          - Messages indicate specific reason for rejection (landed vs too low)
 * 
 *          **When Checks Pass**:
 *          - Calls parachute_release() to perform actual deployment
 *          - Same mechanism as automatic deployment
 *          - Motors continue running, landing gear deploys
 * 
 * @return void
 * 
 * @note Typically called in response to pilot RC switch or GCS MAVLink command
 * @note Safety checks can prevent deployment even with pilot command
 * @note Minimum altitude check uses CHUTE_ALT_MIN parameter
 * @note Ground station receives feedback message if deployment rejected
 * 
 * @warning Pilot should verify altitude and airborne status before commanding deployment
 * @warning Once checks pass and deployment initiates, action is irreversible
 * @warning Rejected deployments are logged and reported to ground station
 * 
 * @see Copter::parachute_release() for actual deployment actions
 * @see Copter::parachute_check() for automatic deployment
 * @see AP_Parachute::enabled()
 * @see CHUTE_ENABLED, CHUTE_ALT_MIN parameters
 */
void Copter::parachute_manual_release()
{
    // Exit immediately if parachute system is not enabled
    // CHUTE_ENABLED parameter must be non-zero
    // Prevents accidental deployment if parachute hardware not installed
    if (!parachute.enabled()) {
        return;
    }

    // Do not release if vehicle is landed (on ground)
    // Deploying parachute on ground wastes one-time deployment
    // May also cause parachute to wrap around vehicle causing damage
    // ap.land_complete flag set by landing detector
    if (ap.land_complete) {
        // Warn pilot of rejection reason via ground station
        gcs().send_text(MAV_SEVERITY_INFO,"Parachute: Landed");
        LOGGER_WRITE_ERROR(LogErrorSubsystem::PARACHUTES, LogErrorCode::PARACHUTE_LANDED);
        return;
    }

    // Do not release if below minimum configured altitude
    // CHUTE_ALT_MIN parameter defines minimum altitude for effective deployment
    // Parachute needs altitude to inflate and slow descent before ground impact
    // current_loc.alt is in centimeters, parachute.alt_min() is in meters
    // If CHUTE_ALT_MIN is 0, check is disabled (no minimum altitude)
    if ((parachute.alt_min() != 0 && (current_loc.alt < (int32_t)parachute.alt_min() * 100))) {
        // Warn pilot of rejection reason via ground station
        gcs().send_text(MAV_SEVERITY_ALERT,"Parachute: Too low");
        LOGGER_WRITE_ERROR(LogErrorSubsystem::PARACHUTES, LogErrorCode::PARACHUTE_TOO_LOW);
        return;
    }

    // All safety checks passed - proceed with parachute deployment
    // Calls same release function as automatic deployment
    // Triggers servo/relay, deploys landing gear, keeps motors armed
    parachute_release();
}

#endif  // HAL_PARACHUTE_ENABLED
