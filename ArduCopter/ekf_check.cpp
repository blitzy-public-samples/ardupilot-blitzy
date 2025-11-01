#include "Copter.h"

/**
 * @file ekf_check.cpp
 * @brief Extended Kalman Filter (EKF) health monitoring and failsafe system
 * 
 * @details This file implements comprehensive monitoring of the EKF state estimation
 *          health through variance analysis and innovation checking. It provides
 *          critical failsafe protection for multicopter flight by detecting when
 *          the EKF's position, velocity, or compass estimates become unreliable.
 * 
 *          The monitoring system uses a counter-based approach that requires sustained
 *          bad EKF health (typically 1 second) before triggering failsafe actions,
 *          preventing false positives from transient sensor glitches. It also implements
 *          recovery mechanisms including EKF lane switching and yaw reset attempts
 *          before declaring a full failsafe.
 * 
 *          Key Safety Features:
 *          - Multi-variance monitoring (position, velocity, compass)
 *          - Sustained failure detection to prevent false triggers
 *          - Automatic EKF lane switching and yaw reset recovery
 *          - Configurable failsafe actions (report, AltHold, Land)
 *          - Vibration compensation for altitude control
 * 
 *          This is SAFETY-CRITICAL code that directly affects vehicle controllability.
 *          EKF failures can lead to loss of position control and vehicle crashes if
 *          not properly detected and handled.
 * 
 * @warning Any modifications to this file must be thoroughly tested in SITL and
 *          on actual hardware. Incorrect EKF failsafe logic can cause unnecessary
 *          landings or, worse, fail to detect actual navigation failures.
 * 
 * @note This system operates at 10Hz (called from the main scheduler)
 * 
 * @see libraries/AP_NavEKF3/AP_NavEKF3.h for EKF implementation details
 * @see libraries/AP_AHRS/AP_AHRS.h for AHRS interface
 */

/**
 * @def EKF_CHECK_ITERATIONS_MAX
 * @brief Maximum consecutive bad EKF variance iterations before triggering failsafe
 * 
 * @details This threshold represents 1 second of sustained bad EKF health at the 10Hz
 *          check rate. Requiring multiple consecutive failures prevents false positives
 *          from transient sensor glitches or brief GPS dropouts.
 * 
 * @note Value must be at least 7 to allow time for lane switching and yaw reset recovery
 */
#ifndef EKF_CHECK_ITERATIONS_MAX
 # define EKF_CHECK_ITERATIONS_MAX          10      // 1 second (ie. 10 iterations at 10hz) of bad variances signals a failure
#endif

/**
 * @def EKF_CHECK_WARNING_TIME
 * @brief Minimum interval between EKF variance warning messages to GCS
 * 
 * @details Warning messages are throttled to avoid flooding the telemetry link and
 *          ground station display during sustained EKF problems.
 */
#ifndef EKF_CHECK_WARNING_TIME
 # define EKF_CHECK_WARNING_TIME            (30*1000)   // warning text messages are sent to ground no more than every 30 seconds
#endif

/**
 * @brief EKF health monitoring state structure
 * 
 * @details Maintains the state machine for EKF variance monitoring and failsafe triggering.
 *          Uses a counter-based approach to require sustained failures before taking action.
 * 
 *          State Machine Flow:
 *          1. Checks pass → fail_count decrements, bad_variance clears when reaching 0
 *          2. Checks fail → fail_count increments
 *          3. At (MAX-2): Request yaw reset from EKF as recovery attempt
 *          4. At (MAX-1): Request lane switch from EKF as recovery attempt  
 *          5. At MAX: Set bad_variance flag and trigger failsafe_ekf_event()
 * 
 * @note This structure is static to this file - state persists across function calls
 */
static struct {
    uint8_t fail_count;         ///< Number of consecutive iterations EKF variances exceeded threshold (0 to EKF_CHECK_ITERATIONS_MAX)
    bool bad_variance;          ///< True when fail_count reaches MAX - triggers failsafe actions and AP_Notify warning
    bool has_ever_passed;       ///< True after EKF checks first pass - prevents failsafe before EKF fully initializes
    uint32_t last_warn_time;    ///< System time (milliseconds) of last warning message - used to throttle GCS text warnings
} ekf_check_state;

/**
 * @brief Monitor EKF health through variance analysis and trigger failsafe if necessary
 * 
 * @details This is the main EKF health monitoring function that runs continuously at 10Hz.
 *          It monitors position, velocity, and compass variance estimates from the EKF
 *          to detect navigation system failures. Uses a counter-based state machine to
 *          require sustained failures (typically 1 second) before triggering failsafe.
 * 
 *          Algorithm Flow:
 *          1. Check EKF has valid origin (exit if not)
 *          2. Check if monitoring is enabled via g.fs_ekf_thresh parameter (exit if disabled)
 *          3. Evaluate variance thresholds via ekf_over_threshold()
 *          4. Check for position estimate availability
 *          5. Increment/decrement fail_count based on health status
 *          6. At thresholds (MAX-2, MAX-1, MAX), attempt recovery or trigger failsafe
 *          7. Update AP_Notify flags for ground station and LED indicators
 * 
 *          Recovery Mechanisms (attempted before failsafe):
 *          - At fail_count = MAX-2: Request EKF yaw reset (may resolve compass issues)
 *          - At fail_count = MAX-1: Request EKF lane switch (switch to backup EKF core)
 *          - At fail_count = MAX: Trigger full EKF failsafe
 * 
 * @note Called at 10Hz from the main scheduler
 * @note Early returns if EKF origin not set or monitoring disabled
 * @note Requires has_ever_passed flag to prevent false positives during EKF initialization
 * 
 * @warning SAFETY-CRITICAL: This function determines when the vehicle's navigation system
 *          has failed. Incorrect logic can cause unnecessary landings or fail to detect
 *          real navigation failures leading to flyaways or crashes.
 * 
 * @see ekf_over_threshold() for variance threshold checking algorithm
 * @see failsafe_ekf_event() for actions taken when failsafe triggers
 * @see failsafe_ekf_off_event() for recovery actions when variances clear
 * @see libraries/AP_AHRS/AP_AHRS.h for ahrs.get_origin(), ahrs.request_yaw_reset(), ahrs.check_lane_switch()
 */
void Copter::ekf_check()
{
    // Compile-time check: Ensure we have enough iterations for recovery mechanisms
    // Need at least 7 to allow time for yaw reset (at MAX-2) and lane switching (at MAX-1)
    static_assert(EKF_CHECK_ITERATIONS_MAX >= 7, "EKF_CHECK_ITERATIONS_MAX must be at least 7");

    // Early exit: EKF must have a valid origin before we can evaluate health
    // The origin is set during EKF initialization and should never become unset
    // If origin isn't set, EKF isn't ready for operation yet
    Location temp_loc;
    if (!ahrs.get_origin(temp_loc)) {
        return;
    }

    // Early exit: Check if EKF monitoring is disabled via parameter
    // g.fs_ekf_thresh <= 0.0 disables EKF variance checking entirely
    // When disabled, clear all failsafe state and return
    if (g.fs_ekf_thresh <= 0.0f) {
        ekf_check_state.fail_count = 0;
        ekf_check_state.bad_variance = false;
        AP_Notify::flags.ekf_bad = ekf_check_state.bad_variance;
        failsafe_ekf_off_event();   // Ensure failsafe is cleared if it was active
        return;
    }

    // Evaluate current EKF health status
    // over_threshold: true if position/velocity/compass variances exceed configured threshold
    // has_position: true if EKF can provide either relative (e.g., optical flow) or absolute (GPS) position
    // checks_passed: true only if variances are acceptable AND we have position estimate
    const bool over_threshold = ekf_over_threshold();
    const bool has_position = ekf_has_relative_position() || ekf_has_absolute_position();
    const bool checks_passed = !over_threshold && has_position;

    // Track if checks have ever passed - prevents false failsafe during EKF initialization
    // During startup, EKF variances may be temporarily high while the filter converges
    // Don't evaluate failures until the EKF has demonstrated it can achieve good health
    ekf_check_state.has_ever_passed |= checks_passed;
    if (!ekf_check_state.has_ever_passed) {
        return;
    }

    // Main state machine: Increment/decrement failure counter and take action at thresholds
    // This counter-based approach requires sustained failures before declaring EKF unhealthy
    if (!checks_passed) {
        // EKF health checks are failing - variances too high or no position estimate
        
        if (!ekf_check_state.bad_variance) {
            // Variances not yet flagged as bad - we're in the "warning" period
            // Increment failure counter toward the failsafe threshold
            ekf_check_state.fail_count++;
            
            // RECOVERY ATTEMPT 1: At MAX-2 iterations, try EKF yaw reset
            // Yaw reset can resolve compass calibration issues or magnetic interference
            // Only attempt if the problem is actually high variances (not just missing position)
            if (ekf_check_state.fail_count == (EKF_CHECK_ITERATIONS_MAX-2) && over_threshold) {
                ahrs.request_yaw_reset();
            }
            
            // RECOVERY ATTEMPT 2: At MAX-1 iterations, try EKF lane switch
            // Lane switching changes to a backup EKF core that may have better health
            // This is the last-ditch recovery attempt before declaring failsafe
            if (ekf_check_state.fail_count == (EKF_CHECK_ITERATIONS_MAX-1)) {
                ahrs.check_lane_switch();
            }
            
            // FAILSAFE TRIGGER: Counter has reached maximum - declare EKF unhealthy
            if (ekf_check_state.fail_count >= EKF_CHECK_ITERATIONS_MAX) {
                // Cap counter to prevent overflow and maintain clear "bad" state
                ekf_check_state.fail_count = EKF_CHECK_ITERATIONS_MAX;
                ekf_check_state.bad_variance = true;
                
                // Log the EKF variance failure for post-flight analysis
                LOGGER_WRITE_ERROR(LogErrorSubsystem::EKFCHECK, LogErrorCode::EKFCHECK_BAD_VARIANCE);
                
                // Notify ground station - throttled to avoid telemetry flooding
                // Only send message if sufficient time has passed since last warning
                if ((AP_HAL::millis() - ekf_check_state.last_warn_time) > EKF_CHECK_WARNING_TIME) {
                    gcs().send_text(MAV_SEVERITY_CRITICAL,"EKF variance");
                    ekf_check_state.last_warn_time = AP_HAL::millis();
                }
                
                // Trigger the failsafe response (mode change, landing, etc.)
                failsafe_ekf_event();
            }
        }
    } else {
        // EKF health checks are passing - variances acceptable and position available
        
        // Decrement failure counter - allows recovery from transient issues
        // Counter decrements at same rate it increments for symmetric hysteresis
        if (ekf_check_state.fail_count > 0) {
            ekf_check_state.fail_count--;

            // If variances were flagged as bad and counter has fully cleared, declare recovery
            // Requires sustained good health (counter reaching 0) before clearing failsafe
            if (ekf_check_state.bad_variance && ekf_check_state.fail_count == 0) {
                ekf_check_state.bad_variance = false;
                LOGGER_WRITE_ERROR(LogErrorSubsystem::EKFCHECK, LogErrorCode::EKFCHECK_VARIANCE_CLEARED);
                
                // Clear the failsafe state and notify systems
                failsafe_ekf_off_event();
            }
        }
    }

    // set AP_Notify flags
    AP_Notify::flags.ekf_bad = ekf_check_state.bad_variance;

    // To-Do: add ekf variances to extended status
}

/**
 * @brief Evaluate if EKF variances exceed configured thresholds
 * 
 * @details Checks position, velocity, and compass variances from the EKF against the
 *          configured threshold (g.fs_ekf_thresh). Uses filtered variance values to
 *          reduce sensitivity to momentary spikes. Implements a multi-sensor voting
 *          scheme where failsafe triggers if multiple variance sources exceed threshold.
 * 
 *          Variance Threshold Logic:
 *          - Returns true if 2 or more of {compass, velocity, position} exceed threshold
 *          - OR if velocity variance exceeds 2x threshold (critical for flight safety)
 *          - Position variance requires at least 1 other variance also high
 *          - Special handling: velocity threshold doubled if optical flow is healthy
 * 
 *          The multi-variance approach prevents false positives from single sensor
 *          issues while detecting true navigation failures where multiple estimates
 *          degrade simultaneously (typical of real EKF convergence problems).
 * 
 * @return true if EKF variances exceed threshold and should be considered unhealthy
 * @return false if variances are within acceptable limits or monitoring is disabled
 * 
 * @note Updates filtered variance values (pos_variance_filt, vel_variance_filt) on every call
 * @note Velocity variance threshold is relaxed (2x) when optical flow is providing velocity estimates
 * @note Returns false if variances not valid or monitoring disabled (g.fs_ekf_thresh <= 0)
 * 
 * @warning SAFETY-CRITICAL: This threshold logic determines when to declare navigation
 *          system failure. Too sensitive causes nuisance failsafes; too lenient allows
 *          flight with bad state estimates leading to loss of control.
 * 
 * @see ekf_check() for how this is used in the failsafe state machine
 * @see libraries/AP_AHRS/AP_AHRS.h for ahrs.get_variances() interface
 */
bool Copter::ekf_over_threshold()
{
    // Retrieve current variance estimates from the EKF
    // Variances represent the EKF's confidence in its state estimates:
    // - Low variance = high confidence, good estimate
    // - High variance = low confidence, uncertain estimate
    float position_var, vel_var, height_var, tas_variance;
    Vector3f mag_variance;
    variances_valid = ahrs.get_variances(vel_var, position_var, height_var, mag_variance, tas_variance);

    // Early exit if variances not available (EKF not initialized or in error state)
    if (!variances_valid) {
        return false;
    }

    // Calculate time step for variance filtering
    uint32_t now_us = AP_HAL::micros();
    float dt = (now_us - last_ekf_check_us) * 1e-6f;

    // Apply low-pass filtering to variance estimates to reduce noise sensitivity
    // Filtering prevents momentary variance spikes from triggering false positives
    // Note: Filtered values are also used by check_vibration() for vibration detection
    position_var = pos_variance_filt.apply(position_var, dt);
    vel_var = vel_variance_filt.apply(vel_var, dt);

    last_ekf_check_us = now_us;

    // Check if monitoring is disabled via parameter (should have been caught earlier, but double-check)
    if (g.fs_ekf_thresh <= 0.0f) {
        return false;
    }

    // Extract maximum compass variance across all three axes (X, Y, Z)
    // Using max ensures we detect problems in any magnetic axis
    const float mag_max = fmaxf(fmaxf(mag_variance.x,mag_variance.y),mag_variance.z);

    // Multi-sensor voting scheme for threshold detection
    // Count how many variance sources exceed threshold:
    // Returns true if 2+ exceed threshold OR velocity alone exceeds 2x threshold
    // This approach avoids false positives from single sensor issues while catching
    // true navigation failures where multiple estimates degrade together
    uint8_t over_thresh_count = 0;
    
    // Check compass variance
    if (mag_max >= g.fs_ekf_thresh) {
        over_thresh_count++;
    }

    // Check velocity variance with special handling for optical flow
    // Optical flow provides velocity estimates independent of GPS/IMU integration,
    // so if it's healthy we can be more lenient with velocity variance threshold
    bool optflow_healthy = false;
#if AP_OPTICALFLOW_ENABLED
    optflow_healthy = optflow.healthy();
#endif
    if (!optflow_healthy && (vel_var >= (2.0f * g.fs_ekf_thresh))) {
        // Velocity variance is critically high (2x threshold) and we don't have optical flow backup
        // This is very serious - count as 2 votes to trigger immediate failsafe
        over_thresh_count += 2;
    } else if (vel_var >= g.fs_ekf_thresh) {
        // Velocity variance over threshold but not critical, or optical flow is backing it up
        over_thresh_count++;
    }

    // Final threshold logic:
    // Trigger if position variance high AND at least one other variance high (over_thresh_count >= 1)
    // OR if at least two variances are high regardless of position (over_thresh_count >= 2)
    if ((position_var >= g.fs_ekf_thresh && over_thresh_count >= 1) || over_thresh_count >= 2) {
        return true;
    }

    return false;
}


/**
 * @brief Execute EKF failsafe response when navigation system failure detected
 * 
 * @details Called when EKF variances have remained above threshold for the configured
 *          duration (typically 1 second), indicating the navigation system can no longer
 *          be trusted. Takes action based on the configured failsafe behavior parameter
 *          (g.fs_ekf_action) to safely handle the degraded navigation state.
 * 
 *          Failsafe Actions (configurable via FS_EKF_ACTION parameter):
 *          - REPORT_ONLY: Alert pilot but take no autonomous action
 *          - ALT_HOLD: Switch to altitude hold mode (maintains altitude, pilot controls horizontal)
 *          - LAND: Initiate autonomous landing in current location
 *          - LAND_EVEN_STABILIZE: Land even if in manual flight mode
 * 
 *          Special Handling:
 *          - No action taken if vehicle is disarmed (failsafe only matters in flight)
 *          - If already landing with GPS, switches to non-GPS land to avoid using bad position
 *          - If current mode doesn't require GPS, only reports the failure (no mode change)
 *          - Falls back to land mode if mode change fails
 * 
 * @note Sets failsafe.ekf flag and AP_Notify::flags.failsafe_ekf for ground station display
 * @note Logs the failsafe event for post-flight analysis
 * @note Called from ekf_check() when fail_count reaches EKF_CHECK_ITERATIONS_MAX
 * 
 * @warning SAFETY-CRITICAL: This function determines the vehicle's response to navigation
 *          system failure. The chosen action must balance safety (landing to avoid flyaway)
 *          against operational needs (allowing pilot recovery in some situations).
 * 
 * @see failsafe_ekf_off_event() for failsafe recovery when EKF health returns
 * @see failsafe_ekf_recheck() for re-evaluation when mode changes
 * @see ekf_check() for the detection logic that triggers this function
 */
void Copter::failsafe_ekf_event()
{
    // Mark that EKF failsafe condition has been triggered
    // This flag is checked by other systems to modify behavior during EKF failure
    failsafe.ekf = true;
    
    // Log the failsafe occurrence for post-flight analysis
    LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_EKFINAV, LogErrorCode::FAILSAFE_OCCURRED);

    // Early exit: No action needed if vehicle is disarmed
    // EKF failsafe only matters in flight - on the ground, pilot can handle EKF issues
    if (!motors->armed()) {
        return;
    }

    // Set notification flag for ground station display and LED indicators
    // This alerts the pilot that EKF failsafe is active
    AP_Notify::flags.failsafe_ekf = true;

    // Check if configured to report only (no autonomous action)
    const bool report_only = g.fs_ekf_action == FS_EKF_ACTION_REPORT_ONLY;

    // Special case: If already landing with GPS-based navigation, switch to non-GPS land
    // This prevents the land mode from using the bad position estimate to navigate
    // We want to land vertically at current location, not attempt to reach a target
    const bool landing_with_position = landing_with_GPS();
    if (landing_with_position && !report_only) {
        mode_land.do_not_use_GPS();
    }

    // Check if current mode requires position/GPS
    // If we're in a mode that doesn't need position (e.g., Stabilize, AltHold), 
    // no mode change is needed unless LAND_EVEN_STABILIZE is configured
    const bool no_action_in_current_mode = !copter.flightmode->requires_GPS() && 
                                            (g.fs_ekf_action != FS_EKF_ACTION_LAND_EVEN_STABILIZE);

    // Early exit cases: Just report the failure, don't change modes
    // - Report-only mode configured
    // - Already landing (and we've switched it to non-GPS mode above)
    // - In a manual mode that doesn't rely on position
    if (report_only || landing_with_position || no_action_in_current_mode) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "EKF Failsafe");
        return;
    }

    // Take autonomous action based on configured failsafe behavior
    // The action taken balances safety against operational flexibility
    switch (g.fs_ekf_action) {
        case FS_EKF_ACTION_REPORT_ONLY:
            // This case should have been handled by early return above
            // Include for completeness
            break;
            
        case FS_EKF_ACTION_ALTHOLD:
            // Switch to altitude hold mode - maintains current altitude while pilot regains control
            // This is less aggressive than landing, allowing pilot recovery if they can fix the issue
            // Fallback to land if radio failsafe also active or mode change fails
            if (failsafe.radio || !set_mode(Mode::Number::ALT_HOLD, ModeReason::EKF_FAILSAFE)) {
                set_mode_land_with_pause(ModeReason::EKF_FAILSAFE);
            }
            break;
            
        case FS_EKF_ACTION_LAND:
        case FS_EKF_ACTION_LAND_EVEN_STABILIZE:
        default:
            // Initiate landing - safest response to navigation system failure
            // Lands at current location using non-GPS methods (barometer for altitude)
            set_mode_land_with_pause(ModeReason::EKF_FAILSAFE);
            break;
    }

    // Notify ground station of the mode change
    gcs().send_text(MAV_SEVERITY_CRITICAL, "EKF Failsafe: changed to %s Mode", flightmode->name());
}

/**
 * @brief Clear EKF failsafe condition when navigation system health returns
 * 
 * @details Called when EKF variances have returned to acceptable levels and remained
 *          there long enough for the fail_count to decrement to zero. This indicates
 *          the navigation system has recovered and can be trusted again. Clears the
 *          failsafe flags and notifies the ground station.
 * 
 *          Recovery Actions:
 *          - Clear failsafe.ekf flag (allows position-based modes to function normally)
 *          - Clear AP_Notify::flags.failsafe_ekf (removes warning indicators)
 *          - Send notification to ground station
 *          - Log the recovery event
 * 
 *          Note: This function does NOT automatically change flight modes. If the failsafe
 *          caused a mode change (e.g., to Land or AltHold), the vehicle remains in that
 *          mode. The pilot must manually return to the desired mode after EKF recovery.
 *          This is intentional - automatic mode switching during recovery could be
 *          surprising to the pilot and potentially unsafe.
 * 
 * @note Called from ekf_check() when fail_count reaches 0 after bad_variance was set
 * @note Early returns if EKF failsafe is not currently active
 * @note Does not change flight mode - pilot must manually recover
 * 
 * @see failsafe_ekf_event() for actions taken when failsafe triggers
 * @see ekf_check() for the monitoring logic that calls this function
 */
void Copter::failsafe_ekf_off_event(void)
{
    // Early exit: Only take action if EKF failsafe is currently active
    // If failsafe.ekf is false, there's nothing to clear
    if (!failsafe.ekf) {
        return;
    }

    // Clear the internal failsafe flag
    // This allows position-based flight modes to function normally again
    failsafe.ekf = false;
    
    // Clear the notification flag and inform ground station
    // Only send message if the notification was actually active
    if (AP_Notify::flags.failsafe_ekf) {
        AP_Notify::flags.failsafe_ekf = false;
        gcs().send_text(MAV_SEVERITY_CRITICAL, "EKF Failsafe Cleared");
    }
    
    // Log the failsafe resolution for post-flight analysis
    // This helps correlate EKF recovery with flight events
    LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_EKFINAV, LogErrorCode::FAILSAFE_RESOLVED);
}

/**
 * @brief Re-evaluate EKF failsafe when flight mode changes to require position
 * 
 * @details Called by flight modes when they transition from a submode that doesn't
 *          require position to one that does. If EKF failsafe is still active, this
 *          re-triggers the failsafe response to prevent the new position-dependent
 *          submode from using bad EKF estimates.
 * 
 *          Example Usage:
 *          - Guided mode switching from velocity control to position control
 *          - Auto mode beginning a waypoint mission
 *          - Loiter mode being engaged
 * 
 *          This prevents a dangerous scenario where:
 *          1. EKF failsafe triggered while in manual mode (no action taken)
 *          2. Pilot switches to position-dependent mode
 *          3. Mode would use bad EKF estimates causing vehicle to fly uncontrolled
 * 
 * @note Called by flight mode code when transitioning to position-dependent submodes
 * @note Early returns if EKF failsafe is not active (no action needed)
 * @note Re-triggers full failsafe_ekf_event() logic with current mode evaluation
 * 
 * @warning Flight mode developers must call this when switching to position-dependent
 *          submodes to maintain safety during EKF failure conditions
 * 
 * @see failsafe_ekf_event() for the re-triggered failsafe response
 */
void Copter::failsafe_ekf_recheck()
{
    // Early exit: Only re-check if EKF failsafe is currently active
    // If failsafe.ekf is false, the EKF is healthy and no action is needed
    if (!failsafe.ekf) {
        return;
    }

    // Re-trigger EKF failsafe evaluation
    // This will re-evaluate the current flight mode's position requirements
    // and take appropriate action (mode change, landing, etc.) if the new
    // submode requires position but EKF is still unhealthy
    failsafe_ekf_event();
}

/**
 * @brief Monitor for EKF resets and primary core changes, adjust control targets accordingly
 * 
 * @details The EKF can perform two types of resets that affect vehicle control:
 *          1. Yaw Reset: Adjusts heading estimate due to compass recalibration or magnetic interference
 *          2. Primary Core Change: Switches to a different EKF core (lane switch) for better health
 * 
 *          Both reset types cause discontinuities in the state estimate that controllers must
 *          handle to avoid sudden vehicle movements. This function detects these resets and
 *          notifies the attitude controller to adjust its reference frames accordingly.
 * 
 *          Yaw Reset Handling:
 *          - Detects when EKF has reset its yaw estimate
 *          - Resets attitude controller's inertial frame reference to prevent sudden rotation commands
 *          - Logs the reset event for analysis
 * 
 *          Primary Core Change Handling:
 *          - Detects when EKF switches to a different core (lane switch)
 *          - Resets attitude controller and position controller reference frames
 *          - Logs the core change and notifies ground station
 *          - Position controller handles its own reset via AC_PosControl
 * 
 * @note Called from main loop (typically at 400Hz) to detect resets as soon as possible
 * @note Yaw resets are tracked via timestamp (ekfYawReset_ms) to detect changes
 * @note Primary core tracked via index (ekf_primary_core) comparison
 * @note Core index of -1 indicates invalid/not initialized - ignored to prevent false detection
 * 
 * @see libraries/AP_NavEKF3/AP_NavEKF3.h for EKF reset mechanisms
 * @see libraries/AC_AttitudeControl/AC_AttitudeControl.h for inertial_frame_reset()
 * @see libraries/AC_PosControl/AC_PosControl.h for position target reset handling
 */
void Copter::check_ekf_reset()
{
    // Check for EKF yaw reset (heading adjustment)
    // The EKF may reset yaw estimate due to compass recalibration, magnetic interference
    // resolution, or GPS heading validation. This causes a discontinuity in the heading
    // reference that controllers must account for.
    float yaw_angle_change_rad;
    uint32_t new_ekfYawReset_ms = ahrs.getLastYawResetAngle(yaw_angle_change_rad);
    
    // Detect yaw reset by comparing timestamp - if timestamp changed, a reset occurred
    if (new_ekfYawReset_ms != ekfYawReset_ms) {
        // Reset the attitude controller's inertial reference frame
        // This prevents the controller from trying to "correct" the yaw change,
        // which would cause the vehicle to rotate unexpectedly
        attitude_control->inertial_frame_reset();
        
        // Update our stored timestamp for next comparison
        ekfYawReset_ms = new_ekfYawReset_ms;
        
        // Log the event for post-flight analysis
        // Frequent yaw resets may indicate compass problems or magnetic interference
        LOGGER_WRITE_EVENT(LogEvent::EKF_YAW_RESET);
    }

    // Check for primary EKF core change (lane switch)
    // The EKF runs multiple parallel cores and can switch between them if one becomes
    // healthier than the current primary. This is a recovery mechanism for EKF issues.
    // Core index of -1 means no valid core, so we ignore it to avoid false detection
    if ((ahrs.get_primary_core_index() != ekf_primary_core) && (ahrs.get_primary_core_index() != -1)) {
        // Reset attitude controller's inertial reference frame
        // Position controller (AC_PosControl) handles its own position target reset
        attitude_control->inertial_frame_reset();
        
        // Update stored primary core index
        ekf_primary_core = ahrs.get_primary_core_index();
        
        // Log the core change - includes the new core index in the error code
        // Multiple lane switches may indicate EKF instability or sensor problems
        LOGGER_WRITE_ERROR(LogErrorSubsystem::EKF_PRIMARY, LogErrorCode(ekf_primary_core));
        
        // Notify ground station of the lane switch
        // This alerts the pilot that the EKF had to switch cores (potential issue)
        gcs().send_text(MAV_SEVERITY_WARNING, "EKF primary changed:%d", (unsigned)ekf_primary_core);
    }
}

/**
 * @brief Detect high vibrations affecting altitude control and enable compensation
 * 
 * @details Monitors EKF innovations and velocity variance to detect when excessive
 *          vibrations are degrading altitude and position control performance. When
 *          sustained high vibrations are detected, activates vibration-resistant gains
 *          in the position controller to maintain better altitude hold.
 * 
 *          Detection Algorithm:
 *          - Checks vertical velocity and position innovations (both positive indicates upward bias)
 *          - Checks vertical velocity variance exceeds 1.0 (indicates poor estimate quality)
 *          - Checks AHRS vibration-affected flag
 *          - Requires sustained detection (1 second) before enabling compensation
 *          - Requires sustained good conditions (15 seconds) before disabling compensation
 * 
 *          Vibration Effects on Flight:
 *          - IMU accelerometer measurements corrupted by high-frequency vibration
 *          - EKF vertical velocity estimate becomes noisy and biased
 *          - Altitude hold performance degrades (oscillations, drift)
 *          - Position controller may become unstable
 * 
 *          Compensation Mechanism:
 *          - Switches position controller to vibration-resistant gain schedule
 *          - Reduces aggressive position corrections that amplify vibration effects
 *          - Maintains safer altitude control at cost of some performance
 * 
 * @note Called at 10Hz from main scheduler (same rate as ekf_check)
 * @note Only activates when armed and in modes with automatic throttle control
 * @note Filtered variances updated by ekf_check() which runs at same rate
 * @note Asymmetric timing: 1s to enable, 15s to disable (prevents oscillation)
 * 
 * @warning High vibrations indicate mechanical issues that should be corrected
 *          Compensation helps safety but is not a substitute for fixing vibration sources
 * 
 * @see ekf_check() for variance filtering that this function uses
 * @see libraries/AC_PosControl/AC_PosControl.h for set_vibe_comp() implementation
 */
void Copter::check_vibration()
{
    uint32_t now = AP_HAL::millis();

    // Assume innovation checks will be valid unless proven otherwise
    // This flag tracks whether we have valid data for vibration detection
    bool innovation_checks_valid = true;

    // Retrieve EKF innovations (difference between predicted and measured values)
    // Innovation analysis reveals how well the EKF's predictions match sensor measurements
    // Positive vertical velocity AND position innovations indicate sustained upward bias
    // typical of vibration corruption causing false upward acceleration detection
    Vector3f vel_innovation;
    Vector3f pos_innovation;
    Vector3f mag_innovation;
    float tas_innovation;
    float yaw_innovation;
    if (!ahrs.get_innovations(vel_innovation, pos_innovation, mag_innovation, tas_innovation, yaw_innovation)) {
        innovation_checks_valid = false;
    }
    
    // Check if both vertical velocity and position innovations are positive
    // NED frame: .z is down, so positive innovation means EKF thinks we're rising
    // Both positive together is a strong indicator of vibration-induced bias
    const bool innov_velD_posD_positive = is_positive(vel_innovation.z) && is_positive(pos_innovation.z);

    // Check if vertical velocity variance is high (>= 1.0)
    // High variance indicates the EKF has low confidence in its velocity estimate
    // This is typical when accelerometer data is noisy from vibrations
    // Note: filtered variances are updated in ekf_check() at same 10Hz rate
    if (!variances_valid) {
        innovation_checks_valid = false;
    }
    
    // Check AHRS vibration detection flag
    // AHRS has its own vibration detection based on accelerometer clipping and consistency
    const bool is_vibration_affected = ahrs.is_vibration_affected();
    
    // Combine all vibration indicators
    // Bad vibration detected if:
    // - Innovation checks valid AND innovations biased AND variance high
    // - OR AHRS reports vibration-affected state
    const bool bad_vibe_detected = (innovation_checks_valid && innov_velD_posD_positive && 
                                    (vel_variance_filt.get() > 1.0f)) || is_vibration_affected;
    
    // Determine if we should take action
    // Only enable compensation when:
    // - Feature is enabled (g2.fs_vibe_enabled)
    // - Bad vibration detected
    // - Motors armed (only matters in flight)
    // - In automatic throttle mode (manual throttle modes don't use position controller)
    const bool do_bad_vibe_actions = (g2.fs_vibe_enabled == 1) && bad_vibe_detected && 
                                     motors->armed() && !flightmode->has_manual_throttle();

    // State machine for vibration compensation
    if (!vibration_check.high_vibes) {
        // Currently NOT in vibration compensation mode
        
        // Track how long bad vibration has been detected
        // Reset timer if conditions are good
        if (!do_bad_vibe_actions) {
            vibration_check.start_ms = now;
        }
        
        // Check if bad vibration has persisted for required duration (1 second)
        // This sustained detection prevents false positives from transient events
        if (now - vibration_check.start_ms > 1000) {
            // Enable vibration compensation in position controller
            // This switches to more conservative gains that are less sensitive to
            // noisy velocity estimates, maintaining safer altitude control
            vibration_check.clear_ms = 0;
            vibration_check.high_vibes = true;
            pos_control->set_vibe_comp(true);
            
            // Log the event for post-flight analysis
            LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_VIBE, LogErrorCode::FAILSAFE_OCCURRED);
            
            // Notify ground station - pilot should investigate vibration source
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Vibration compensation ON");
        }
    } else {
        // Currently IN vibration compensation mode
        
        // Track how long conditions have been good
        // Reset timer if bad vibrations detected
        if (do_bad_vibe_actions) {
            vibration_check.clear_ms = now;
        }
        
        // Turn off vibration compensation after sustained good conditions (15 seconds)
        // Longer duration than enable (15s vs 1s) prevents oscillating in/out of compensation
        // and ensures vibrations have truly subsided before returning to normal gains
        if (now - vibration_check.clear_ms > 15000) {
            // Restore normal position controller gains
            vibration_check.start_ms = 0;
            vibration_check.high_vibes = false;
            pos_control->set_vibe_comp(false);
            vibration_check.clear_ms = 0;
            
            // Log the recovery
            LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_VIBE, LogErrorCode::FAILSAFE_RESOLVED);
            
            // Notify ground station
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Vibration compensation OFF");
        }
    }

    return;
}
