/**
 * @file ekf_check.cpp
 * @brief Implementation of EKF health monitoring and failsafe triggering for Rover
 * 
 * @details This module continuously monitors the Extended Kalman Filter (EKF) health
 *          by checking variance levels for position, velocity, and compass estimates.
 *          When variances exceed configured thresholds for a sustained period, the
 *          system triggers an EKF failsafe to protect against unreliable navigation.
 *          
 *          The EKF check runs at 10Hz and implements a counter-based hysteresis
 *          mechanism to prevent spurious failsafe triggers from momentary variance
 *          spikes while still responding quickly to genuine EKF failures.
 *          
 *          Safety-Critical Component: This code is essential for detecting navigation
 *          system failures before they cause vehicle loss of control or crashes.
 * 
 * @note This module is called from the main Rover scheduler at 10Hz
 * @warning Modifications to variance thresholds or timing parameters can affect
 *          vehicle safety and should only be made after thorough testing
 * 
 * Source: Rover/ekf_check.cpp
 */

#include "Rover.h"

/**
 * @def EKF_CHECK_ITERATIONS_MAX
 * @brief Maximum number of consecutive bad EKF variance checks before triggering failsafe
 * 
 * @details At 10Hz update rate, 10 iterations equals 1 second of sustained bad variances.
 *          This provides a balance between quick failsafe response and avoiding false
 *          triggers from momentary variance spikes during maneuvers or GPS glitches.
 * 
 * Value: 10 iterations (1 second at 10Hz)
 */
#ifndef EKF_CHECK_ITERATIONS_MAX
 # define EKF_CHECK_ITERATIONS_MAX          10      // 1 second (ie. 10 iterations at 10hz) of bad variances signals a failure
#endif

/**
 * @def EKF_CHECK_WARNING_TIME
 * @brief Minimum time between EKF variance warning messages to ground station
 * 
 * @details Throttles warning messages to prevent flooding the telemetry link and
 *          ground station console with repeated warnings during sustained EKF issues.
 *          Warnings are still logged continuously to dataflash.
 * 
 * Value: 30000 milliseconds (30 seconds)
 */
#ifndef EKF_CHECK_WARNING_TIME
 # define EKF_CHECK_WARNING_TIME            (30*1000)   // warning text messages are sent to ground no more than every 30 seconds
#endif


/**
 * @struct ekf_check_state
 * @brief State tracking structure for EKF health monitoring
 * 
 * @details This static structure maintains the state of EKF variance monitoring
 *          across multiple update cycles, implementing a counter-based hysteresis
 *          mechanism to avoid spurious failsafe triggers.
 *          
 *          The fail_count increments when variances exceed threshold and decrements
 *          when they return to normal, with the bad_variance flag set only after
 *          sustained failures (fail_count >= EKF_CHECK_ITERATIONS_MAX).
 */
static struct {
    uint8_t fail_count;         ///< Number of consecutive iterations with EKF variance exceeding threshold (0 to EKF_CHECK_ITERATIONS_MAX)
    uint8_t bad_variance : 1;   ///< Flag: true when EKF should be considered untrusted (fail_count >= EKF_CHECK_ITERATIONS_MAX)
    uint32_t last_warn_time;    ///< System time of last warning message in milliseconds (used to throttle GCS text messages)
} ekf_check_state;

/**
 * @brief Main EKF health monitoring function - detects variance threshold violations and triggers failsafe
 * 
 * @details This function implements the primary EKF health monitoring logic:
 *          1. Validates EKF has origin set (required for position estimates)
 *          2. Bypasses checks when disarmed or EKF failsafe disabled
 *          3. Checks if EKF variances exceed threshold using ekf_over_threshold()
 *          4. Implements counter-based hysteresis (fail_count)
 *          5. Triggers failsafe after EKF_CHECK_ITERATIONS_MAX consecutive failures
 *          6. Clears failsafe when variances return to acceptable levels
 *          
 *          Counter Behavior:
 *          - Increments by 1 each iteration when variances are bad
 *          - Decrements by 1 each iteration when variances are good
 *          - Caps at EKF_CHECK_ITERATIONS_MAX (10 iterations = 1 second)
 *          - Failsafe triggered when counter reaches maximum
 *          - Failsafe cleared when counter returns to zero
 * 
 * @note Called at 10Hz from main Rover scheduler
 * @warning Safety-Critical: This function protects against navigation failures that
 *          could cause vehicle loss of control. Any modifications require extensive
 *          testing in SITL and on actual hardware.
 * 
 * @see ekf_over_threshold() for variance threshold checking logic
 * @see failsafe_ekf_event() for failsafe trigger actions
 * @see failsafe_ekf_off_event() for failsafe recovery actions
 */
void Rover::ekf_check()
{
    // Exit immediately if EKF has no origin yet - this assumes the origin can never become unset
    // Origin is required for position estimates; without it, EKF cannot provide navigation data
    Location temp_loc;
    if (!ahrs.get_origin(temp_loc)) {
        return;
    }

    // Return immediately if motors are not armed, or EKF check is disabled (FS_EKF_THRESH <= 0)
    // When disarmed, EKF variance is expected to be higher and does not pose safety risk
    // When disabled (threshold <= 0), user has explicitly disabled EKF monitoring
    if (!arming.is_armed() || (g.fs_ekf_thresh <= 0.0f)) {
        ekf_check_state.fail_count = 0;
        ekf_check_state.bad_variance = false;
        AP_Notify::flags.ekf_bad = ekf_check_state.bad_variance;
        failsafe_ekf_off_event();   // Clear any existing failsafe state
        return;
    }

    // Check if EKF variances (compass, velocity, position) exceed configured threshold
    // ekf_over_threshold() returns true if 2+ variance types exceed FS_EKF_THRESH parameter
    if (ekf_over_threshold()) {
        // Variances are bad - increment failure counter unless already in failsafe
        if (!ekf_check_state.bad_variance) {
            // Increment failure counter - each iteration at 10Hz adds 0.1 seconds
            ekf_check_state.fail_count++;
            
            // Check if sustained failure threshold reached (default 10 iterations = 1 second)
            if (ekf_check_state.fail_count >= EKF_CHECK_ITERATIONS_MAX) {
                // Limit counter from climbing beyond max to prevent overflow
                ekf_check_state.fail_count = EKF_CHECK_ITERATIONS_MAX;
                ekf_check_state.bad_variance = true;

                // Log the EKF variance failure to dataflash for post-flight analysis
                LOGGER_WRITE_ERROR(LogErrorSubsystem::EKFCHECK,
                                         LogErrorCode::EKFCHECK_BAD_VARIANCE);
                
                // Send warning message to ground station (throttled to once per 30 seconds)
                // Throttling prevents telemetry link saturation during sustained EKF issues
                if ((AP_HAL::millis() - ekf_check_state.last_warn_time) > EKF_CHECK_WARNING_TIME) {
                    GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,"EKF variance");
                    ekf_check_state.last_warn_time = AP_HAL::millis();
                }
                
                // Trigger EKF failsafe action (mode change based on FS_EKF_ACTION parameter)
                failsafe_ekf_event();
            }
        }
    } else {
        // Variances are good - decrement failure counter with hysteresis
        if (ekf_check_state.fail_count > 0) {
            // Decrement counter by 1 per iteration (0.1 seconds at 10Hz)
            // This provides symmetric recovery timing: 1 second bad triggers failsafe,
            // 1 second good clears it (assuming started from failsafe threshold)
            ekf_check_state.fail_count--;

            // If variance flag is set and counter has fully recovered to zero, clear the flag
            if (ekf_check_state.bad_variance && ekf_check_state.fail_count == 0) {
                ekf_check_state.bad_variance = false;
                
                // Log variance recovery to dataflash
                LOGGER_WRITE_ERROR(LogErrorSubsystem::EKFCHECK,
                                         LogErrorCode::EKFCHECK_VARIANCE_CLEARED);
                
                // Clear EKF failsafe state and restore normal operation
                failsafe_ekf_off_event();
            }
        }
    }

    // Update AP_Notify LED/buzzer flags to indicate EKF health status to pilot
    // This provides visual/audio feedback of EKF issues at all times
    AP_Notify::flags.ekf_bad = ekf_check_state.bad_variance;
}

/**
 * @brief Check if EKF variance values exceed configured threshold
 * 
 * @details This function retrieves current EKF variance estimates for position,
 *          velocity, and compass (magnetometer) from the AHRS/EKF system and
 *          compares them against the FS_EKF_THRESH parameter.
 *          
 *          Threshold Logic:
 *          - Compass variance checked as 3D magnitude of mag_variance vector
 *          - Velocity variance has special handling for optical flow scenarios
 *          - Position variance checked directly against threshold
 *          - Returns true if 2 or more variance types exceed threshold
 *          - Also returns true if ekf_position_ok() reports position estimate invalid
 *          
 *          Optical Flow Special Case:
 *          - Without optical flow: velocity variance counts double if >= 2*threshold
 *          - With optical flow: standard threshold checking
 *          - This compensates for reduced velocity estimate quality without flow
 * 
 * @return true if EKF variances indicate unreliable navigation estimates
 * @return false if EKF variances are within acceptable tolerance
 * 
 * @note FS_EKF_THRESH parameter sets the variance threshold (typically 0.8)
 * @warning Setting threshold too low causes false failsafes; too high delays detection
 * 
 * @see ekf_position_ok() for additional position validity checks
 */
bool Rover::ekf_over_threshold()
{
    // Return false immediately if EKF failsafe is disabled (threshold <= 0)
    if (g.fs_ekf_thresh <= 0.0f) {
        return false;
    }

    // Retrieve current variance estimates from EKF via AHRS interface
    // These represent the EKF's confidence in its state estimates
    float position_variance, vel_variance, height_variance, tas_variance;
    Vector3f mag_variance;  // 3D magnetometer variance vector
    ahrs.get_variances(vel_variance, position_variance, height_variance, mag_variance, tas_variance);

    // Count how many variance types exceed threshold (need 2+ for failsafe trigger)
    // This multi-variance approach reduces false positives from single sensor issues
    uint8_t over_thresh_count = 0;
    
    // Check compass (magnetometer) variance as 3D vector magnitude
    // High mag variance indicates magnetic interference or compass calibration issues
    if (mag_variance.length() >= g.fs_ekf_thresh) {
        over_thresh_count++;
    }
    
    // Check velocity variance
    // High velocity variance indicates GPS velocity quality issues or IMU problems
    if (vel_variance >= g.fs_ekf_thresh) {
        over_thresh_count++;
    }
    
    // Check horizontal position variance
    // High position variance indicates GPS position quality issues or lack of position aiding
    if (position_variance >= g.fs_ekf_thresh) {
        over_thresh_count++;
    }

    // Check if optical flow sensor is available and healthy
    bool optflow_healthy = false;
#if AP_OPTICALFLOW_ENABLED
    optflow_healthy = optflow.healthy();
#endif
    
    // Special velocity variance handling based on optical flow availability
    // Without optical flow, velocity estimates rely solely on GPS, so high variance
    // is more critical - count it twice if >= 2*threshold to trigger faster
    if (!optflow_healthy && (vel_variance >= (2.0f * g.fs_ekf_thresh))) {
        over_thresh_count += 2;  // Count velocity twice when very bad without flow backup
    } else if (vel_variance >= g.fs_ekf_thresh) {
        over_thresh_count++;  // Normal velocity variance check
    }
    
    // Trigger if 2 or more variance types exceed threshold
    // This provides robustness against single sensor failures while still detecting EKF issues
    if (over_thresh_count >= 2) {
        return true;
    }

    // Final check: even if variances look OK, verify position estimate is actually valid
    // Returns true (over threshold) if position is not OK
    return !ekf_position_ok();
}

/**
 * @brief Verify EKF horizontal position estimate validity
 * 
 * @details This function checks the EKF filter status flags to determine if the
 *          horizontal position estimate is reliable enough for navigation.
 *          
 *          Position Validity Criteria:
 *          - Must have inertial navigation (EKF running, not DCM-only mode)
 *          - When disarmed: Accepts predicted or actual absolute/relative position
 *          - When armed: Requires actual (not predicted) absolute or relative position
 *                       AND must not be in constant position mode
 *          
 *          Constant Position Mode:
 *          When EKF is in const_pos_mode, it has lost all position aiding sources
 *          (GPS, optical flow, etc.) and is holding last known position. This is
 *          unreliable for navigation and must trigger failsafe.
 *          
 *          Position Types:
 *          - horiz_pos_abs: Absolute position (GPS) available
 *          - horiz_pos_rel: Relative position (optical flow, wheel encoders) available
 *          - pred_*: Predicted position (dead reckoning, less reliable)
 * 
 * @return true if horizontal position estimate is valid for current vehicle state
 * @return false if position estimate is unreliable or unavailable
 * 
 * @note More lenient when disarmed (accepts predicted position) since no safety risk
 * @warning When armed, const_pos_mode indicates complete loss of position aiding
 *          and should trigger immediate failsafe action
 */
bool Rover::ekf_position_ok()
{
    // Require EKF-based inertial navigation system
    // DCM (Direction Cosine Matrix) does not provide position estimates
    if (!ahrs.have_inertial_nav()) {
        // Do not allow navigation with DCM attitude estimation only
        return false;
    }

    // Retrieve current EKF filter status flags
    nav_filter_status filt_status;
    rover.ahrs.get_filter_status(filt_status);

    // When disarmed, accept predicted or actual position (absolute or relative)
    // Predicted positions are acceptable when disarmed since vehicle is stationary
    // and there is no safety risk from less accurate position estimates
    if (!arming.is_armed()) {
        return (filt_status.flags.horiz_pos_abs || filt_status.flags.pred_horiz_pos_abs || 
                filt_status.flags.horiz_pos_rel || filt_status.flags.pred_horiz_pos_rel);
    } else {
        // When armed, require actual (non-predicted) absolute or relative position
        // AND verify EKF is not in constant position mode (which indicates total
        // loss of position aiding from GPS, optical flow, or other sources)
        // Constant position mode means EKF is dead reckoning with last known position
        return ((filt_status.flags.horiz_pos_abs || filt_status.flags.horiz_pos_rel) && 
                !filt_status.flags.const_pos_mode);
    }
}

/**
 * @brief Execute EKF failsafe action when navigation system becomes unreliable
 * 
 * @details This function is called when EKF variances have exceeded threshold for
 *          a sustained period (typically 1 second), indicating the navigation system
 *          cannot be trusted. The action taken depends on the FS_EKF_ACTION parameter.
 *          
 *          Failsafe Actions (FS_EKF_ACTION parameter):
 *          - FS_EKF_DISABLE (0): No action, only report issue
 *          - FS_EKF_REPORT_ONLY (1): Log and report but don't change mode
 *          - FS_EKF_HOLD (2): Switch to HOLD mode to stop vehicle (default, safest)
 *          
 *          Mode Requirement Check:
 *          If current mode doesn't require position (e.g., MANUAL, ACRO), failsafe
 *          is logged but no mode change occurs since non-position modes are not
 *          affected by EKF health.
 *          
 *          Safety Philosophy:
 *          Default action (HOLD) stops the vehicle to prevent navigation with
 *          unreliable position estimates, which could cause crashes, boundary
 *          violations, or loss of vehicle control.
 * 
 * @note This function is idempotent - multiple calls have no additional effect
 *       once failsafe flag is set
 * 
 * @warning Safety-Critical: This function prevents vehicle operation with
 *          unreliable navigation. Modifications must preserve safety behavior.
 *          Default HOLD action should only be changed after risk analysis.
 * 
 * @see failsafe_ekf_off_event() for failsafe recovery
 * @see ekf_check() for failsafe trigger conditions
 */
void Rover::failsafe_ekf_event()
{
    // Return immediately if EKF failsafe already triggered
    // Prevents redundant logging and mode changes
    if (failsafe.ekf) {
        return;
    }

    // Set EKF failsafe flag to indicate navigation system is untrusted
    failsafe.ekf = true;
    
    // Log failsafe occurrence to dataflash for post-flight analysis
    LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_EKFINAV,
                             LogErrorCode::FAILSAFE_OCCURRED);

    // Check if current control mode requires position estimates
    // Modes like MANUAL, ACRO, STEERING don't use position, so EKF failure doesn't affect them
    // Only take failsafe action for position-dependent modes (AUTO, GUIDED, RTL, etc.)
    if (!control_mode->requires_position()) {
        return;
    }

    // Execute failsafe action based on FS_EKF_ACTION parameter configuration
    switch ((enum fs_ekf_action)g.fs_ekf_action.get()) {
        case FS_EKF_DISABLE:
            // Disabled: No mode change, only logging
            // Use with caution - vehicle will continue current mode with bad navigation
            return;
            
        case FS_EKF_REPORT_ONLY:
            // Report only: Log and alert but don't change mode
            // Allows experienced operators to maintain control while aware of EKF issues
            break;
            
        case FS_EKF_HOLD:
        default:
            // Hold: Switch to HOLD mode to stop vehicle (safest default action)
            // Prevents vehicle from continuing mission or commanded actions with
            // unreliable position estimates that could cause crashes or loss
            set_mode(mode_hold, ModeReason::EKF_FAILSAFE);
            break;
    }

    // Send critical warning to ground station
    // Operator needs immediate notification of navigation system failure
    GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,"EKF failsafe");
}

/**
 * @brief Clear EKF failsafe when navigation system health is restored
 * 
 * @details This function is called when EKF variances have returned to acceptable
 *          levels (below FS_EKF_THRESH) and the fail_count has decremented back
 *          to zero, indicating sustained recovery of navigation system health.
 *          
 *          Recovery Process:
 *          - Clears the failsafe.ekf flag
 *          - Logs the failsafe resolution to dataflash
 *          - Notifies ground station operator
 *          - Does NOT automatically change vehicle mode
 *          
 *          Mode Behavior After Recovery:
 *          The vehicle remains in whatever mode it was in when failsafe cleared
 *          (typically HOLD if FS_EKF_ACTION was HOLD). The operator must manually
 *          command a new mode. This prevents unexpected behavior if EKF briefly
 *          recovers then fails again.
 *          
 *          Timing:
 *          With default settings, recovery requires 1 second (10 iterations at 10Hz)
 *          of good variances, providing symmetric hysteresis with the trigger timing.
 * 
 * @note This function is idempotent - safe to call multiple times
 * @note Vehicle does not automatically resume previous mode - operator intervention required
 * 
 * @see failsafe_ekf_event() for failsafe trigger
 * @see ekf_check() for recovery detection logic
 */
void Rover::failsafe_ekf_off_event(void)
{
    // Return immediately if not currently in EKF failsafe state
    // Prevents spurious logging and messages
    if (!failsafe.ekf) {
        return;
    }

    // Clear EKF failsafe flag - navigation system is now trusted again
    failsafe.ekf = false;
    
    // Log failsafe resolution to dataflash for flight analysis
    LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_EKFINAV,
                             LogErrorCode::FAILSAFE_RESOLVED);
    
    // Notify ground station that EKF health has been restored
    // Operator can now manually resume normal operations if desired
    GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,"EKF failsafe cleared");
}
