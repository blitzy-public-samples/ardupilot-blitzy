/**
 * @file ekf_check.cpp
 * @brief EKF (Extended Kalman Filter) health monitoring and failsafe system for Blimp
 * 
 * @details This file implements comprehensive EKF health monitoring for lighter-than-air vehicles,
 *          specifically designed for the buoyant flight characteristics of blimps. The system
 *          continuously monitors EKF variance and innovation metrics to detect navigation
 *          degradation and trigger appropriate failsafe responses.
 * 
 *          Key monitoring capabilities:
 *          - Position, velocity, and magnetometer variance threshold checking
 *          - Innovation monitoring for vibration-induced navigation errors
 *          - EKF error count tracking with configurable thresholds
 *          - Yaw reset request mechanism when approaching failsafe threshold
 *          - EKF lane switching for multi-EKF configurations
 *          - Failsafe event triggering for safety-critical navigation loss
 *          - Vibration compensation through EKF gain adjustment
 * 
 *          The monitoring runs at 10Hz during armed flight and implements a multi-stage
 *          response strategy: first attempting yaw reset, then lane switching, and finally
 *          triggering failsafe landing if variance remains excessive.
 * 
 * @note Blimp-specific considerations: Lighter-than-air vehicles have unique navigation
 *       challenges including wind sensitivity and slow response dynamics. The EKF monitoring
 *       parameters and thresholds should be tuned for the slower motion profiles typical
 *       of blimps compared to heavier-than-air multicopters.
 * 
 * @warning This is SAFETY-CRITICAL code. EKF health directly affects navigation accuracy
 *          and vehicle safety. Modifications to variance thresholds, failsafe logic, or
 *          timing parameters can compromise flight safety.
 * 
 * Source: Blimp/ekf_check.cpp
 */

#include "Blimp.h"

/**
 * @brief Maximum iterations of bad EKF variance before failsafe trigger
 * 
 * @details Defines the threshold for declaring EKF failure. At 10Hz check rate,
 *          10 iterations equals 1 second of continuous bad variance before
 *          triggering failsafe response. This provides time for temporary
 *          variance spikes to clear while responding quickly to sustained problems.
 * 
 * @note Must be at least 7 iterations to allow for yaw reset (at iter-2) and
 *       lane switching (at iter-1) recovery attempts before failsafe trigger.
 */
#ifndef EKF_CHECK_ITERATIONS_MAX
# define EKF_CHECK_ITERATIONS_MAX          10      // 1 second (ie. 10 iterations at 10hz) of bad variances signals a failure
#endif

/**
 * @brief Minimum time between EKF warning messages sent to ground station
 * 
 * @details Throttles "EKF variance" warning messages to once per 30 seconds
 *          to avoid flooding the ground control station with redundant alerts
 *          during sustained EKF problems. Value in milliseconds.
 */
#ifndef EKF_CHECK_WARNING_TIME
# define EKF_CHECK_WARNING_TIME            (30*1000)   // warning text messages are sent to ground no more than every 30 seconds
#endif

////////////////////////////////////////////////////////////////////////////////
// EKF_check structure
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief EKF health check state tracking structure
 * 
 * @details Maintains persistent state for EKF variance monitoring across
 *          multiple check iterations. Tracks failure count progression,
 *          variance status flags, and warning message timing to implement
 *          the multi-stage failsafe response strategy.
 * 
 * @note Static storage provides state persistence between ekf_check() calls
 *       at 10Hz update rate throughout armed flight.
 */
static struct {
    uint8_t fail_count;         ///< Number of consecutive iterations with EKF variance over threshold (0 to EKF_CHECK_ITERATIONS_MAX)
    uint8_t bad_variance : 1;   ///< Flag indicating EKF is untrusted - set when fail_count exceeds EKF_CHECK_ITERATIONS_MAX
    uint32_t last_warn_time;    ///< System time (milliseconds) of last "EKF variance" warning message - used to throttle GCS warnings to EKF_CHECK_WARNING_TIME interval
} ekf_check_state;

/**
 * @brief Main EKF health monitoring function - detects variance threshold violations and triggers progressive failsafe response
 * 
 * @details Implements comprehensive EKF health checking at 10Hz update rate during armed flight.
 *          Monitors compass (magnetometer), velocity, and position variance against configured
 *          threshold (g.fs_ekf_thresh) and navigation status. Employs a multi-stage response
 *          strategy to recover from temporary variance spikes before resorting to failsafe:
 * 
 *          Stage 1 (fail_count reaches EKF_CHECK_ITERATIONS_MAX-2): Request yaw reset from AHRS
 *          Stage 2 (fail_count reaches EKF_CHECK_ITERATIONS_MAX-1): Request EKF lane switch
 *          Stage 3 (fail_count reaches EKF_CHECK_ITERATIONS_MAX): Trigger failsafe_ekf_event()
 * 
 *          The fail_count increments each iteration when variance exceeds threshold and
 *          decrements when variance is acceptable, implementing hysteresis to avoid
 *          oscillation between good/bad states.
 * 
 *          Checks are bypassed when:
 *          - Motors are disarmed (not flying)
 *          - EKF has no origin set (initial startup)
 *          - g.fs_ekf_thresh <= 0.0 (feature disabled)
 * 
 * @note Called at 10Hz from main scheduler loop. Timing is critical for accurate
 *       fail_count progression and recovery attempt sequencing.
 * 
 * @warning SAFETY-CRITICAL: This function directly affects navigation safety and failsafe
 *          triggering. The multi-stage response with yaw reset and lane switching attempts
 *          provides recovery opportunities before landing, but timing must be precise.
 *          For blimps, consider that slow dynamics may require adjusted thresholds compared
 *          to faster multicopters.
 * 
 * @see ekf_over_threshold() for variance threshold logic
 * @see failsafe_ekf_event() for failsafe action implementation
 * @see failsafe_ekf_off_event() for failsafe clearing
 */
void Blimp::ekf_check()
{
    // ensure EKF_CHECK_ITERATIONS_MAX is at least 7
    static_assert(EKF_CHECK_ITERATIONS_MAX >= 7, "EKF_CHECK_ITERATIONS_MAX must be at least 7");

    // exit immediately if ekf has no origin yet - this assumes the origin can never become unset
    Location temp_loc;
    if (!ahrs.get_origin(temp_loc)) {
        return;
    }

    // return immediately if motors are not armed, or ekf check is disabled
    if (!motors->armed() || (g.fs_ekf_thresh <= 0.0f)) {
        ekf_check_state.fail_count = 0;
        ekf_check_state.bad_variance = false;
        AP_Notify::flags.ekf_bad = ekf_check_state.bad_variance;
        failsafe_ekf_off_event();   // clear failsafe
        return;
    }

    // compare compass and velocity variance vs threshold and also check
    // if we are still navigating
    bool is_navigating = ekf_has_relative_position() || ekf_has_absolute_position();
    if (ekf_over_threshold() || !is_navigating) {
        // if compass is not yet flagged as bad
        if (!ekf_check_state.bad_variance) {
            // increase counter
            ekf_check_state.fail_count++;
            if (ekf_check_state.fail_count == (EKF_CHECK_ITERATIONS_MAX-2) && ekf_over_threshold()) {
                // we are two iterations away from declaring an EKF failsafe, ask the EKF if we can reset
                // yaw to resolve the issue
                ahrs.request_yaw_reset();
            }
            if (ekf_check_state.fail_count == (EKF_CHECK_ITERATIONS_MAX-1)) {
                // we are just about to declare a EKF failsafe, ask the EKF if we can
                // change lanes to resolve the issue
                ahrs.check_lane_switch();
            }
            // if counter above max then trigger failsafe
            if (ekf_check_state.fail_count >= EKF_CHECK_ITERATIONS_MAX) {
                // limit count from climbing too high
                ekf_check_state.fail_count = EKF_CHECK_ITERATIONS_MAX;
                ekf_check_state.bad_variance = true;
                LOGGER_WRITE_ERROR(LogErrorSubsystem::EKFCHECK, LogErrorCode::EKFCHECK_BAD_VARIANCE);
                // send message to gcs
                if ((AP_HAL::millis() - ekf_check_state.last_warn_time) > EKF_CHECK_WARNING_TIME) {
                    gcs().send_text(MAV_SEVERITY_CRITICAL,"EKF variance");
                    ekf_check_state.last_warn_time = AP_HAL::millis();
                }
                failsafe_ekf_event();
            }
        }
    } else {
        // reduce counter
        if (ekf_check_state.fail_count > 0) {
            ekf_check_state.fail_count--;

            // if compass is flagged as bad and the counter reaches zero then clear flag
            if (ekf_check_state.bad_variance && ekf_check_state.fail_count == 0) {
                ekf_check_state.bad_variance = false;
                LOGGER_WRITE_ERROR(LogErrorSubsystem::EKFCHECK, LogErrorCode::EKFCHECK_VARIANCE_CLEARED);
                // clear failsafe
                failsafe_ekf_off_event();
            }
        }
    }

    // set AP_Notify flags
    AP_Notify::flags.ekf_bad = ekf_check_state.bad_variance;

    // To-Do: add ekf variances to extended status
}

/**
 * @brief Determines if EKF variance metrics exceed configured safety thresholds
 * 
 * @details Evaluates multiple EKF variance sources against the configured threshold
 *          (g.fs_ekf_thresh parameter) using a multi-criteria approach to reduce
 *          false positives. Checks compass (magnetometer), velocity, and position
 *          variance from AHRS EKF implementation.
 * 
 *          Threshold logic returns true (variance excessive) when:
 *          - Two or more of {compass, velocity, position} exceed threshold, OR
 *          - Velocity variance exceeds 2x threshold (regardless of others), OR
 *          - Position variance exceeds threshold AND any other metric exceeds threshold
 * 
 *          Special handling for optical flow:
 *          - If optical flow is NOT healthy, velocity variance threshold is doubled
 *            to account for increased uncertainty in velocity estimation
 *          - Currently optflow_healthy hardcoded to false (may be vehicle limitation)
 * 
 *          Variance sources from AHRS:
 *          - mag_variance: 3-axis magnetometer variance (X, Y, Z) - uses maximum component
 *          - vel_variance: Velocity estimation variance
 *          - position_variance: Position estimation variance
 *          - height_variance: Altitude variance (retrieved but not used in threshold logic)
 *          - tas_variance: True airspeed variance (retrieved but not used)
 * 
 * @return true if EKF variance exceeds safety thresholds (failsafe condition)
 * @return false if variance within acceptable limits OR feature disabled (g.fs_ekf_thresh <= 0)
 * 
 * @note Called at 10Hz from ekf_check(). Variance values are continuously updated by
 *       the EKF prediction and update cycles in the AHRS subsystem.
 * 
 * @warning Threshold parameter (g.fs_ekf_thresh) is vehicle-specific and must be tuned
 *          for the expected dynamics and sensor noise characteristics. Too-sensitive
 *          thresholds cause nuisance failsafes; too-relaxed thresholds compromise safety.
 * 
 * @see ekf_check() for fail_count progression and recovery logic
 */
bool Blimp::ekf_over_threshold()
{
    // return false immediately if disabled
    if (g.fs_ekf_thresh <= 0.0f) {
        return false;
    }

    // use EKF to get variance
    float position_variance, vel_variance, height_variance, tas_variance;
    Vector3f mag_variance;
    ahrs.get_variances(vel_variance, position_variance, height_variance, mag_variance, tas_variance);

    const float mag_max = fmaxf(fmaxf(mag_variance.x,mag_variance.y),mag_variance.z);

    // return true if two of compass, velocity and position variances are over the threshold OR velocity variance is twice the threshold
    uint8_t over_thresh_count = 0;
    if (mag_max >= g.fs_ekf_thresh) {
        over_thresh_count++;
    }

    bool optflow_healthy = false;
    if (!optflow_healthy && (vel_variance >= (2.0f * g.fs_ekf_thresh))) {
        over_thresh_count += 2;
    } else if (vel_variance >= g.fs_ekf_thresh) {
        over_thresh_count++;
    }

    if ((position_variance >= g.fs_ekf_thresh && over_thresh_count >= 1) || over_thresh_count >= 2) {
        return true;
    }

    return false;
}


/**
 * @brief Triggers EKF failsafe response when navigation becomes unreliable
 * 
 * @details Executes safety response when EKF variance monitoring (ekf_check) determines
 *          that navigation accuracy has degraded beyond safe limits. This function is
 *          called after recovery attempts (yaw reset, lane switching) have failed to
 *          restore acceptable variance levels.
 * 
 *          Failsafe behavior depends on current flight mode and configuration:
 *          - If current mode does NOT require GPS AND fs_ekf_action != LAND_EVEN_MANUAL:
 *            No action taken (manual modes can continue without GPS)
 *          - Otherwise: Action determined by g.fs_ekf_action parameter
 * 
 *          Failsafe actions (g.fs_ekf_action):
 *          - FS_EKF_ACTION_LAND: Force landing mode (only if mode requires GPS)
 *          - FS_EKF_ACTION_LAND_EVEN_MANUAL: Force landing regardless of mode requirements
 * 
 *          Implements once-only triggering - if failsafe.ekf already true, returns
 *          immediately to avoid redundant mode changes and log entries.
 * 
 * @note For blimps, landing with degraded navigation requires careful consideration
 *       of wind conditions and available landing space. The slow descent rate of
 *       lighter-than-air vehicles may allow drift during EKF-degraded landing.
 * 
 * @warning SAFETY-CRITICAL: This function initiates autonomous landing when navigation
 *          is unreliable. Landing site selection and obstacle avoidance may be compromised
 *          due to degraded position estimation. Ensure g.fs_ekf_thresh is conservatively
 *          tuned to trigger before position error becomes dangerous.
 * 
 * @warning Once triggered, the failsafe persists until variance clears and
 *          failsafe_ekf_off_event() is called. Pilot may need to manually switch
 *          modes after landing to regain control.
 * 
 * @see ekf_check() for variance monitoring and fail_count progression
 * @see failsafe_ekf_off_event() for failsafe clearing logic
 * @see set_mode_land_failsafe() for landing mode transition
 */
void Blimp::failsafe_ekf_event()
{
    // return immediately if ekf failsafe already triggered
    if (failsafe.ekf) {
        return;
    }

    // EKF failsafe event has occurred
    failsafe.ekf = true;
    LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_EKFINAV, LogErrorCode::FAILSAFE_OCCURRED);

    // does this mode require position?
    if (!blimp.flightmode->requires_GPS() && (g.fs_ekf_action != FS_EKF_ACTION_LAND_EVEN_MANUAL)) {
        return;
    }

    // take action based on fs_ekf_action parameter
    switch (g.fs_ekf_action) {
    case FS_EKF_ACTION_LAND:
    case FS_EKF_ACTION_LAND_EVEN_MANUAL:
    default:
        set_mode_land_failsafe(ModeReason::EKF_FAILSAFE);
        break;
    }
}

/**
 * @brief Clears EKF failsafe condition when navigation accuracy is restored
 * 
 * @details Called from ekf_check() when EKF variance has returned to acceptable
 *          levels (fail_count decremented to zero after being in bad_variance state).
 *          Resets the failsafe.ekf flag and logs the failsafe resolution event.
 * 
 *          This function only clears the internal failsafe flag - it does NOT
 *          automatically change flight modes. If failsafe_ekf_event() forced a
 *          mode change to LAND, the vehicle will continue landing unless the
 *          pilot manually selects a different mode.
 * 
 *          Implements once-only clearing - if failsafe.ekf already false, returns
 *          immediately to avoid redundant log entries.
 * 
 * @note Clearing the failsafe flag allows the vehicle to operate normally in
 *       modes that require GPS/navigation. However, mode transitions must be
 *       initiated separately (typically by pilot input).
 * 
 * @note For blimps that triggered failsafe landing, clearing the EKF failsafe
 *       after touchdown allows safe re-arming and subsequent flights once
 *       navigation is restored.
 * 
 * @see ekf_check() for fail_count decrement logic
 * @see failsafe_ekf_event() for failsafe triggering
 */
void Blimp::failsafe_ekf_off_event(void)
{
    // return immediately if not in ekf failsafe
    if (!failsafe.ekf) {
        return;
    }

    failsafe.ekf = false;
    LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_EKFINAV, LogErrorCode::FAILSAFE_RESOLVED);
}

/**
 * @brief Monitors for EKF yaw resets and primary core changes, logging events for analysis
 * 
 * @details Detects and responds to two types of EKF state discontinuities that can
 *          affect navigation and control:
 * 
 *          1. Yaw Reset Detection:
 *             Monitors for AHRS yaw angle resets (typically triggered when compass/GPS
 *             disagreement is resolved or when requested via ahrs.request_yaw_reset()).
 *             When detected, logs EKF_YAW_RESET event for post-flight analysis.
 *             The reset timestamp (ekfYawReset_ms) is tracked to detect new resets.
 * 
 *          2. Primary EKF Core Change (Lane Switching):
 *             In multi-EKF configurations (typically EKF2/EKF3 with multiple IMU lanes),
 *             monitors for changes in which EKF core is primary. Lane switching occurs
 *             when the EKF selector determines another lane has better health.
 *             Logs EKF_PRIMARY error with new core index and sends GCS warning message.
 * 
 *          Both events indicate the EKF made significant corrections to state estimates.
 *          While these corrections often improve accuracy, they can cause discontinuities
 *          in navigation that attitude and position controllers must accommodate.
 * 
 * @note Yaw resets can be requested by ekf_check() as a recovery mechanism when
 *       variance approaches failsafe threshold (at fail_count = EKF_CHECK_ITERATIONS_MAX-2).
 * 
 * @note AC_PosControl automatically handles position discontinuities during primary
 *       core changes. Attitude target adjustments for yaw resets are handled implicitly
 *       through the AHRS interface.
 * 
 * @note For blimps, yaw resets may be more common due to large magnetic disturbances
 *       from motors and structure. The slow yaw response of blimps means resets cause
 *       less abrupt behavior than in faster aircraft.
 * 
 * @see ekf_check() for yaw reset request logic (ahrs.request_yaw_reset())
 * @see check_lane_switch() for explicit lane switching requests
 */
void Blimp::check_ekf_reset()
{
    // check for yaw reset
    float yaw_angle_change_rad;
    uint32_t new_ekfYawReset_ms = ahrs.getLastYawResetAngle(yaw_angle_change_rad);
    if (new_ekfYawReset_ms != ekfYawReset_ms) {
        ekfYawReset_ms = new_ekfYawReset_ms;
        LOGGER_WRITE_EVENT(LogEvent::EKF_YAW_RESET);
    }

    // check for change in primary EKF, reset attitude target and log.  AC_PosControl handles position target adjustment
    if ((ahrs.get_primary_core_index() != ekf_primary_core) && (ahrs.get_primary_core_index() != -1)) {
        ekf_primary_core = ahrs.get_primary_core_index();
        LOGGER_WRITE_ERROR(LogErrorSubsystem::EKF_PRIMARY, LogErrorCode(ekf_primary_core));
        gcs().send_text(MAV_SEVERITY_WARNING, "EKF primary changed:%d", (unsigned)ekf_primary_core);
    }
}

/**
 * @brief Detects vibration-induced EKF altitude errors and activates compensation
 * 
 * @details Monitors EKF innovation (measurement prediction error) and variance metrics
 *          to detect characteristic patterns of vibration affecting vertical position
 *          estimation. When high vibrations corrupt accelerometer data, the EKF shows
 *          specific signatures: positive vertical velocity AND position innovations
 *          combined with high vertical velocity variance.
 * 
 *          Detection criteria (all must be true for 1 second):
 *          - g2.fs_vibe_enabled != 0 (feature enabled)
 *          - Motors armed (flying)
 *          - Vertical velocity innovation (vel_innovation.z / NKF3.IVD) is positive
 *          - Vertical position innovation (pos_innovation.z / NKF3.IPD) is positive
 *          - Vertical velocity variance (vel_variance / NK4.SV) >= 1.0
 * 
 *          Response stages:
 *          1. High vibration detected: Switches EKF to use resistant gains (reduces
 *             sensitivity to accelerometer data), sends "Vibration compensation ON"
 *             to GCS, logs FAILSAFE_VIBE occurrence
 *          2. Vibration clears: After 15 seconds of normal operation, restores
 *             standard EKF gains, sends "Vibration compensation OFF", logs resolution
 * 
 *          The 15-second clearing delay prevents oscillation between compensation
 *          states during intermittent vibration.
 * 
 * @note This is NOT a true failsafe (does not force mode changes) but rather an
 *       adaptive EKF gain adjustment to maintain altitude accuracy during vibration.
 * 
 * @note For blimps, vibration from motors/props may be lower magnitude than multicopters
 *       due to larger mass and slower prop speeds, but flexible structures can introduce
 *       different vibration modes. Monitor NK4.SV and NKF3.IVD/IPD logs to tune detection.
 * 
 * @note Innovation values are measurement residuals: positive z-axis innovations indicate
 *       the EKF predicted lower altitude/climb rate than sensors measured (typical of
 *       vibration corrupting accelerometers to indicate false upward acceleration).
 * 
 * @warning Vibration compensation degrades EKF response to real altitude changes.
 *          Prolonged compensation indicates mechanical problem requiring attention.
 *          Check motor mounts, prop balance, and structural resonances.
 * 
 * @see ahrs.get_innovations() for EKF innovation vector retrieval (NKF3 log messages)
 * @see ahrs.get_variances() for EKF variance metrics (NK4 log messages)
 */
void Blimp::check_vibration()
{
    uint32_t now = AP_HAL::millis();

    // assume checks will succeed
    bool checks_succeeded = true;

    // check if vertical velocity and position innovations are positive (NKF3.IVD & NKF3.IPD are both positive)
    Vector3f vel_innovation;
    Vector3f pos_innovation;
    Vector3f mag_innovation;
    float tas_innovation;
    float yaw_innovation;
    if (!ahrs.get_innovations(vel_innovation, pos_innovation, mag_innovation, tas_innovation, yaw_innovation)) {
        checks_succeeded = false;
    }
    const bool innov_velD_posD_positive = is_positive(vel_innovation.z) && is_positive(pos_innovation.z);

    // check if vertical velocity variance is at least 1 (NK4.SV >= 1.0)
    float position_variance, vel_variance, height_variance, tas_variance;
    Vector3f mag_variance;
    if (!ahrs.get_variances(vel_variance, position_variance, height_variance, mag_variance, tas_variance)) {
        checks_succeeded = false;
    }

    // if no failure
    if ((g2.fs_vibe_enabled == 0) || !checks_succeeded || !motors->armed() || !innov_velD_posD_positive || (vel_variance < 1.0f)) {
        if (vibration_check.high_vibes) {
            // start clear time
            if (vibration_check.clear_ms == 0) {
                vibration_check.clear_ms = now;
                return;
            }
            // turn off vibration compensation after 15 seconds
            if (now - vibration_check.clear_ms > 15000) {
                // restore ekf gains, reset timers and update user
                vibration_check.high_vibes = false;
                vibration_check.clear_ms = 0;
                LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_VIBE, LogErrorCode::FAILSAFE_RESOLVED);
                gcs().send_text(MAV_SEVERITY_CRITICAL, "Vibration compensation OFF");
            }
        }
        vibration_check.start_ms = 0;
        return;
    }

    // start timer
    if (vibration_check.start_ms == 0) {
        vibration_check.start_ms = now;
        vibration_check.clear_ms = 0;
        return;
    }

    // check if failure has persisted for at least 1 second
    if (now - vibration_check.start_ms > 1000) {
        if (!vibration_check.high_vibes) {
            // switch ekf to use resistant gains
            vibration_check.high_vibes = true;
            LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_VIBE, LogErrorCode::FAILSAFE_OCCURRED);
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Vibration compensation ON");
        }
    }
}
