#include "Plane.h"

/**
 * @file ekf_check.cpp
 * @brief EKF (Extended Kalman Filter) health monitoring and failsafe management for ArduPlane
 * 
 * @details This file implements continuous monitoring of the EKF state estimation health
 *          by checking variance levels for key navigation parameters. The monitoring system:
 *          - Tracks innovation consistency (measurement vs prediction differences)
 *          - Monitors velocity, position, height, and magnetometer variances
 *          - Implements hysteresis to prevent false triggering
 *          - Triggers automatic recovery actions when variance exceeds safety thresholds
 *          - Attempts EKF yaw reset and lane switching before declaring failsafe
 *          - Switches to safe flight modes when EKF becomes unreliable
 * 
 *          The EKF provides the primary navigation solution by fusing data from GPS,
 *          IMU, barometer, magnetometer, and airspeed sensors. When the EKF variance
 *          indicates unreliable state estimates, continuing autonomous flight could
 *          result in loss of control or crashes.
 * 
 * @warning This is safety-critical code. EKF failures can cause loss of navigation
 *          leading to flyaways, crashes, or controlled flight into terrain. All changes
 *          must be thoroughly tested in SITL and on actual hardware.
 * 
 * @note The monitoring runs at 10Hz from the scheduler. Threshold configuration is
 *       via FS_EKF_THRESH parameter (0 disables monitoring).
 * 
 * Source: ArduPlane/ekf_check.cpp
 */

/**
 * @def EKF_CHECK_ITERATIONS_MAX
 * @brief Maximum consecutive iterations of high EKF variance before declaring failsafe
 * 
 * @details Defines the number of consecutive 10Hz iterations (0.1s each) where EKF variance
 *          must exceed thresholds before triggering failsafe. Default of 10 iterations = 1 second.
 *          This hysteresis prevents false failsafe triggers from transient sensor disturbances.
 *          Must be at least 7 to allow time for yaw reset (iteration -2) and lane switch (iteration -1).
 */
#ifndef EKF_CHECK_ITERATIONS_MAX
 # define EKF_CHECK_ITERATIONS_MAX          10      // 1 second (ie. 10 iterations at 10hz) of bad variances signals a failure
#endif

/**
 * @def EKF_CHECK_WARNING_TIME
 * @brief Minimum time in milliseconds between EKF failsafe warning messages to GCS
 * 
 * @details Throttles "EKF variance" warning messages to avoid flooding the ground control station
 *          telemetry link. Default 30000ms (30 seconds) allows pilot to see warnings without
 *          overwhelming other telemetry data during an ongoing EKF failure.
 */
#ifndef EKF_CHECK_WARNING_TIME
 # define EKF_CHECK_WARNING_TIME            (30*1000)   // warning text messages are sent to ground no more than every 30 seconds
#endif

/**
 * @struct ekf_check_state
 * @brief State tracking structure for EKF health monitoring and failsafe management
 * 
 * @details This static structure maintains the state of the EKF monitoring system across
 *          multiple scheduler iterations. It implements a counting mechanism with hysteresis
 *          to prevent false failsafe triggers while responding quickly to genuine EKF failures.
 * 
 *          The fail_count increments when variance exceeds thresholds and decrements when
 *          variance returns to normal, providing smooth recovery without oscillation.
 * 
 * @note This structure persists between function calls and is initialized to zero at startup
 */
static struct {
    uint8_t fail_count;         ///< Number of consecutive iterations EKF variance exceeded thresholds (0 to EKF_CHECK_ITERATIONS_MAX)
    bool bad_variance;          ///< True when EKF should be considered untrusted (fail_count reached EKF_CHECK_ITERATIONS_MAX)
    uint32_t last_warn_time;    ///< System time in milliseconds of last "EKF variance" warning sent to GCS (throttles warning messages)
    bool failsafe_on;           ///< True when EKF failsafe mode is active (QLAND/QHOVER engaged for quadplane VTOL modes)
} ekf_check_state;

/**
 * @brief Monitor EKF health and trigger recovery actions if variance exceeds safety thresholds
 * 
 * @details This function performs continuous health monitoring of the Extended Kalman Filter,
 *          which provides the primary navigation solution by fusing sensor data. The monitoring
 *          checks multiple variance indicators to detect when the EKF state estimates become
 *          unreliable and may cause unsafe navigation commands.
 * 
 *          Monitoring Algorithm:
 *          1. Check EKF has valid origin (required for position estimates)
 *          2. Skip monitoring if disarmed or in non-position modes (monitoring not needed)
 *          3. Call ekf_over_threshold() to check variance levels against FS_EKF_THRESH parameter
 *          4. If variance high: Increment fail_count with hysteresis
 *             - At iteration MAX-2: Request EKF yaw reset (may resolve magnetic interference)
 *             - At iteration MAX-1: Request EKF lane switch (switch to healthier EKF lane if available)
 *             - At iteration MAX: Trigger failsafe_ekf_event() - switch to safe manual modes
 *          5. If variance normal: Decrement fail_count, clear failsafe when reaching zero
 *          6. Update AP_Notify flags for LED/buzzer indicators
 * 
 *          The hysteresis mechanism (requiring EKF_CHECK_ITERATIONS_MAX consecutive bad readings)
 *          prevents false triggers from transient sensor glitches while responding quickly enough
 *          to prevent crashes from genuine EKF failures.
 * 
 *          Monitored Variance Parameters:
 *          - Velocity variance: Indicates GPS velocity or accelerometer integration issues
 *          - Position variance: Indicates GPS position quality problems
 *          - Height variance: Indicates barometer or GPS altitude issues  
 *          - Magnetometer variance: Indicates compass/magnetic interference issues
 * 
 * @return void
 * 
 * @note Called at 10Hz from the scheduler task list. Must execute quickly to avoid
 *       delaying other time-critical tasks.
 * 
 * @warning SAFETY CRITICAL: This function protects against loss of navigation which could
 *          result in flyaways, crashes, or controlled flight into terrain. EKF failures
 *          in autonomous modes are a leading cause of ArduPilot vehicle losses.
 *          - Do not disable EKF monitoring (FS_EKF_THRESH > 0) without understanding risks
 *          - Test threshold changes thoroughly in SITL before hardware deployment
 *          - Monitor telemetry logs for EKF variance trends during flight testing
 * 
 * @see ekf_over_threshold() for variance threshold checking logic
 * @see failsafe_ekf_event() for failsafe mode switching actions
 * @see failsafe_ekf_off_event() for failsafe recovery/clearing
 * @see AP_AHRS::get_variances() for EKF variance data source
 * @see AP_AHRS::request_yaw_reset() for yaw reset recovery mechanism
 * @see AP_AHRS::check_lane_switch() for EKF lane switching recovery
 */
void Plane::ekf_check()
{
    // ensure EKF_CHECK_ITERATIONS_MAX is at least 7
    static_assert(EKF_CHECK_ITERATIONS_MAX >= 7, "EKF_CHECK_ITERATIONS_MAX must be at least 7");

    // exit immediately if ekf has no origin yet - this assumes the origin can never become unset
    Location temp_loc;
    if (!ahrs.get_origin(temp_loc)) {
        return;
    }

    // return immediately if motors are not armed, or ekf check is disabled
    bool ekf_check_disabled = !plane.arming.is_armed() || (g2.fs_ekf_thresh <= 0.0f);
#if HAL_QUADPLANE_ENABLED
    if (!quadplane.in_vtol_posvel_mode()) {
        ekf_check_disabled = true;
    }
#endif
    if (ekf_check_disabled) {
        ekf_check_state.fail_count = 0;
        ekf_check_state.bad_variance = false;
        AP_Notify::flags.ekf_bad = ekf_check_state.bad_variance;
        failsafe_ekf_off_event();   // clear failsafe
        return;
    }

    // compare compass and velocity variance vs threshold
    if (ekf_over_threshold()) {
        // if compass is not yet flagged as bad
        if (!ekf_check_state.bad_variance) {
            // increase counter
            ekf_check_state.fail_count++;
            
            // RECOVERY ACTION 1: Yaw Reset Attempt
            // At 2 iterations before failsafe, request EKF yaw reset. This can resolve
            // issues caused by magnetic interference or compass calibration errors by
            // reinitializing the yaw estimate using GPS velocity (if moving) or other
            // sensors. Success rate is highest when vehicle is in motion.
            if (ekf_check_state.fail_count == (EKF_CHECK_ITERATIONS_MAX-2)) {
                // we are two iterations away from declaring an EKF failsafe, ask the EKF if we can reset
                // yaw to resolve the issue
                ahrs.request_yaw_reset();
            }
            
            // RECOVERY ACTION 2: Lane Switch Attempt
            // At 1 iteration before failsafe, attempt EKF lane switching. Modern EKF3 runs
            // multiple parallel filters ("lanes") with different parameters. If one lane has
            // low variance while current lane has high variance, switching lanes may restore
            // healthy navigation without entering failsafe mode.
            if (ekf_check_state.fail_count == (EKF_CHECK_ITERATIONS_MAX-1)) {
                // we are just about to declare a EKF failsafe, ask the EKF if we can
                // change lanes to resolve the issue
                ahrs.check_lane_switch();
            }
            
            // RECOVERY ACTION 3: Failsafe Trigger
            // If variance remains high despite yaw reset and lane switch attempts, trigger
            // failsafe to prevent continued autonomous flight with unreliable navigation.
            // For quadplane VTOL modes, this switches to QLAND (if autonomous) or QHOVER
            // (if pilot-controlled) to maintain altitude while pilot regains manual control.
            // if counter above max then trigger failsafe
            if (ekf_check_state.fail_count >= EKF_CHECK_ITERATIONS_MAX) {
                // limit count from climbing too high
                ekf_check_state.fail_count = EKF_CHECK_ITERATIONS_MAX;
                ekf_check_state.bad_variance = true;
                
                // Log the EKF failure event for post-flight analysis
                LOGGER_WRITE_ERROR(LogErrorSubsystem::EKFCHECK, LogErrorCode::EKFCHECK_BAD_VARIANCE);
                
                // Notify ground control station - throttled to avoid telemetry link saturation
                // send message to gcs
                if ((AP_HAL::millis() - ekf_check_state.last_warn_time) > EKF_CHECK_WARNING_TIME) {
                    gcs().send_text(MAV_SEVERITY_CRITICAL,"EKF variance");
                    ekf_check_state.last_warn_time = AP_HAL::millis();
                }
                
                // Execute failsafe mode changes (QLAND/QHOVER for quadplane)
                failsafe_ekf_event();
            }
        }
    } else {
        // VARIANCE NORMAL: Recovery and Failsafe Clearing
        // When EKF variance returns below threshold, gradually reduce fail_count.
        // This hysteresis prevents rapid oscillation between failsafe and normal states.
        // reduce counter
        if (ekf_check_state.fail_count > 0) {
            ekf_check_state.fail_count--;

            // FAILSAFE CLEAR: When fail_count reaches zero, EKF has been healthy for
            // sufficient time to trust navigation again. Clear the bad_variance flag
            // and log the recovery event. Note: mode is NOT automatically changed -
            // pilot must manually exit QLAND/QHOVER if desired.
            // if compass is flagged as bad and the counter reaches zero then clear flag
            if (ekf_check_state.bad_variance && ekf_check_state.fail_count == 0) {
                ekf_check_state.bad_variance = false;
                
                // Log the recovery for post-flight analysis
                LOGGER_WRITE_ERROR(LogErrorSubsystem::EKFCHECK, LogErrorCode::EKFCHECK_VARIANCE_CLEARED);
                
                // Clear the failsafe state (but does not change flight mode)
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
 * @brief Check if EKF variance exceeds configured safety threshold
 * 
 * @details This function retrieves current EKF variance values for multiple sensor types
 *          and applies a weighted scoring system to determine if the navigation solution
 *          has degraded to unsafe levels. The threshold system uses:
 * 
 *          Variance Sources Checked:
 *          - velocity_variance: GPS velocity measurement consistency
 *          - position_variance: GPS position measurement consistency  
 *          - height_variance: Barometer/GPS altitude consistency
 *          - mag_variance: Magnetometer (compass) measurement consistency (3-axis vector)
 *          - tas_variance: True airspeed measurement consistency
 * 
 *          Scoring Algorithm (weighted by importance):
 *          - Magnetometer: +1 point if any axis >= threshold
 *          - Velocity: +2 points if >= 2x threshold, +1 point if >= threshold
 *          - Position: Triggers failsafe if >= threshold AND other sensors score >= 1
 *          - OR: Triggers failsafe if total score >= 2 (even without position failure)
 * 
 *          Position variance is weighted most heavily because horizontal position errors
 *          directly affect waypoint tracking, obstacle avoidance, and geofencing. Velocity
 *          is second in importance as it affects navigation prediction and control response.
 * 
 *          The scoring system prevents single-sensor failures from triggering failsafe
 *          (requires multiple concurrent issues) while ensuring critical position failures
 *          trigger quickly even with minor issues in other sensors.
 * 
 * @return true if EKF variance exceeds safety threshold and failsafe should trigger
 * @return false if EKF variance is within acceptable limits or monitoring is disabled
 * 
 * @note Threshold configured via FS_EKF_THRESH parameter:
 *       - 0 = monitoring disabled (always returns false)
 *       - 0.8 = default threshold (recommended for most vehicles)
 *       - Higher values = less sensitive (more tolerance for variance)
 *       - Lower values = more sensitive (earlier failsafe triggering)
 * 
 * @warning Setting threshold too low may cause false failsafe triggers from minor
 *          sensor disturbances. Setting too high may delay failsafe until navigation
 *          has already degraded significantly. Test any threshold changes thoroughly.
 * 
 * @see AP_AHRS::get_variances() for variance data retrieval
 * @see ekf_check() for the monitoring loop that calls this function
 */
bool Plane::ekf_over_threshold()
{
    // return false immediately if disabled
    if (g2.fs_ekf_thresh <= 0.0f) {
        return false;
    }

    // Get EKF innovations normalised wrt the innovation test limits.
    // A value above 1.0 means the EKF has rejected that sensor data
    float position_variance, vel_variance, height_variance, tas_variance;
    Vector3f mag_variance;
    if (!ahrs.get_variances(vel_variance, position_variance, height_variance, mag_variance, tas_variance)) {
        return false;
    };

    // The EKF rejects all magnetometer axes if any single axis exceeds limits
    // so take the maximum of all axes
    const float mag_max = fmaxf(fmaxf(mag_variance.x,mag_variance.y),mag_variance.z);

    // Assign a score to each over threshold based on severity
    uint8_t over_thresh_count = 0;
    if (mag_max >= g2.fs_ekf_thresh) {
        over_thresh_count++;
    }

    if (vel_variance >= (2.0f * g2.fs_ekf_thresh)) {
        over_thresh_count += 2;
    } else if (vel_variance >= g2.fs_ekf_thresh) {
        over_thresh_count++;
    }

    // Position is the most important so accept a lower score from other sensors if position failed
    if ((position_variance >= g2.fs_ekf_thresh && over_thresh_count >= 1) || over_thresh_count >= 2) {
        return true;
    }

    return false;
}


/**
 * @brief Execute EKF failsafe actions when navigation solution becomes unreliable
 * 
 * @details This function is called when EKF variance has remained above safety thresholds
 *          for EKF_CHECK_ITERATIONS_MAX consecutive iterations despite automatic recovery
 *          attempts (yaw reset and lane switching). The failsafe actions switch the vehicle
 *          to safe flight modes that do not require accurate position/velocity estimates.
 * 
 *          Failsafe Actions (for Quadplane VTOL modes only):
 *          - If in VTOL autonomous mode (QLOITER, AUTO with VTOL, etc.):
 *            Switch to QLAND mode - controlled descent to landing without position hold
 *          - If in VTOL manual mode (QHOVER, QSTABILIZE, etc.):
 *            Switch to QHOVER mode - maintain altitude with manual horizontal control
 *          
 *          For fixed-wing modes, no mode change occurs because fixed-wing flight is
 *          less dependent on precise position estimates and can continue safely with
 *          attitude-only control (pilot can manually navigate).
 * 
 *          The failsafe_on flag prevents repeated mode switches if variance remains high.
 *          Recovery to normal operation requires manual mode selection by the pilot after
 *          EKF health is restored (indicated by failsafe_ekf_off_event() being called).
 * 
 * @return void
 * 
 * @note This function logs the failsafe event with code FAILSAFE_EKFINAV for post-flight
 *       analysis. Review these logs to identify root causes (GPS issues, magnetic
 *       interference, vibration affecting IMU, etc.)
 * 
 * @warning The mode changes are automatic and immediate - no pilot confirmation is
 *          requested. Ensure FS_EKF_THRESH is properly tuned to avoid nuisance failsafes
 *          during normal flight. A QLAND activation during a mission will result in the
 *          vehicle landing at its current location, potentially far from the intended site.
 * 
 * @see ekf_check() for the monitoring loop that triggers this function
 * @see failsafe_ekf_off_event() for failsafe recovery when EKF health is restored
 * @see Plane::set_mode() for flight mode switching implementation
 */
void Plane::failsafe_ekf_event()
{
    // return immediately if ekf failsafe already triggered
    if (ekf_check_state.failsafe_on) {
        return;
    }

    // EKF failsafe event has occurred
    ekf_check_state.failsafe_on = true;
    LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_EKFINAV, LogErrorCode::FAILSAFE_OCCURRED);

    // if not in a VTOL mode requiring position, then nothing needs to be done
#if HAL_QUADPLANE_ENABLED
    if (!quadplane.in_vtol_posvel_mode()) {
        return;
    }

    if (quadplane.in_vtol_auto()) {
        // the pilot is not controlling via sticks so switch to QLAND
        plane.set_mode(mode_qland, ModeReason::EKF_FAILSAFE);
    } else {
        // the pilot is controlling via sticks so fallback to QHOVER
        plane.set_mode(mode_qhover, ModeReason::EKF_FAILSAFE);
    }
#endif
}

/**
 * @brief Clear EKF failsafe state when navigation solution health is restored
 * 
 * @details This function is called when EKF variance has returned below safety thresholds
 *          and remained healthy for sufficient iterations (fail_count decremented to zero).
 *          It clears the internal failsafe_on flag and logs the recovery event.
 * 
 *          IMPORTANT: This function does NOT automatically change the flight mode back to
 *          the pre-failsafe mode. The vehicle will remain in the failsafe mode (QLAND or
 *          QHOVER) until the pilot manually selects a different mode. This design ensures
 *          the pilot is aware of the EKF failure event and has assessed the situation
 *          before resuming autonomous flight.
 * 
 *          Typical Recovery Procedure:
 *          1. EKF variance returns to normal (sensor issue resolved)
 *          2. This function clears failsafe_on flag and logs FAILSAFE_RESOLVED
 *          3. Pilot reviews telemetry to confirm EKF health is stable
 *          4. Pilot manually switches to desired mode (QLOITER, AUTO, etc.)
 *          5. If EKF remains healthy, normal flight operations resume
 * 
 * @return void
 * 
 * @note The FAILSAFE_RESOLVED log entry is important for post-flight analysis to
 *       understand the duration and frequency of EKF issues. Recurring EKF failsafes
 *       may indicate hardware problems (vibration, magnetic interference, GPS antenna
 *       placement) that require physical correction.
 * 
 * @warning If EKF variance increases again shortly after clearing, the monitoring
 *          system will re-enter failsafe. Ensure root cause is resolved before
 *          continuing flight operations. Repeated EKF failures indicate unsafe
 *          flight conditions.
 * 
 * @see ekf_check() for the monitoring loop that calls this function during recovery
 * @see failsafe_ekf_event() for the failsafe trigger that this function reverses
 */
void Plane::failsafe_ekf_off_event(void)
{
    // return immediately if not in ekf failsafe
    if (!ekf_check_state.failsafe_on) {
        return;
    }

    ekf_check_state.failsafe_on = false;
    LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_EKFINAV, LogErrorCode::FAILSAFE_RESOLVED);
}
