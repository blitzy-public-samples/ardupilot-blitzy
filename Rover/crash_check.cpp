/**
 * @file crash_check.cpp
 * @brief Implementation of rover crash and stall detection using velocity, acceleration, and motor output monitoring
 * 
 * @details This module implements safety-critical crash detection for ground vehicles by monitoring
 *          vehicle velocity, angular rates, and motor output to detect crash or stall conditions.
 *          The detection algorithm uses a time-delayed trigger mechanism to avoid false positives
 *          from momentary stops while ensuring rapid response to actual crash events.
 *          
 *          Detection Methods:
 *          - Velocity-based: Monitors groundspeed and yaw rate for near-zero motion
 *          - Attitude-based: Detects excessive pitch/roll angles indicating rollover
 *          - Throttle correlation: Confirms high motor output despite low velocity (stall condition)
 *          - Time filtering: Requires sustained stall condition to trigger crash detection
 *          
 *          Recovery Actions:
 *          - Balance bots: Immediate disarm (to prevent tip-over damage)
 *          - Standard rovers: Switch to HOLD mode, optional disarm based on FS_CRASH_CHECK parameter
 *          
 * @warning This is SAFETY-CRITICAL code. Modifications can affect vehicle crash response,
 *          potentially causing injury or property damage. All changes must be thoroughly
 *          tested in SITL and on physical hardware before deployment.
 * 
 * @note Called at 10Hz by the main scheduler
 * @note Only active when vehicle is armed and crash checking is enabled
 * 
 * Source: Rover/crash_check.cpp
 */

#include "Rover.h"

/**
 * @brief Time threshold for crash detection trigger
 * @details Vehicle must remain in stall condition for this duration before crash is declared.
 *          2 seconds provides sufficient filtering to avoid false positives from intentional
 *          stops or momentary obstacles while responding quickly to actual crashes.
 * @note Value in seconds
 */
static const uint16_t CRASH_CHECK_TRIGGER_SEC = 2;

/**
 * @brief Minimum throttle threshold for crash detection
 * @details Vehicle must have throttle output above this percentage to be considered for crash
 *          detection. This ensures we don't trigger crash detection when vehicle is intentionally
 *          stopped or idling. Value represents percentage (5.0 = 5% throttle).
 * @note Value in percent (0-100)
 */
static const float CRASH_CHECK_THROTTLE_MIN = 5.0f;

/**
 * @brief Minimum velocity threshold for crash detection
 * @details Vehicle velocity (linear or angular) must be below this threshold to be considered
 *          potentially crashed. Applies to both groundspeed (m/s) and yaw rate (rad/s).
 *          Low value ensures detection of actual stalls while allowing very slow crawling.
 * @note Value in m/s for groundspeed, rad/s for yaw rate
 */
static const float CRASH_CHECK_VEL_MIN = 0.08f;

/**
 * @brief Detect crash or stall condition and initiate safety response
 * 
 * @details This function implements multi-criteria crash detection for ground vehicles using
 *          velocity monitoring, attitude monitoring, and motor output correlation. The detection
 *          algorithm uses time-delayed triggering to filter false positives while maintaining
 *          rapid response to actual crash conditions.
 *          
 *          Detection Algorithm:
 *          1. Attitude Check: Immediate trigger if pitch/roll exceeds CRASH_ANGLE parameter
 *          2. Velocity Check: Monitor groundspeed and yaw rate for near-zero motion
 *          3. Throttle Correlation: Confirm high throttle output (>5%) despite low velocity
 *          4. Time Filter: Require sustained stall for CRASH_CHECK_TRIGGER_SEC (2 seconds)
 *          5. Crash Declaration: Execute recovery action based on vehicle type
 *          
 *          Recovery Actions:
 *          - Balance Bots: Immediate disarm to prevent tip-over damage
 *          - Standard Rovers: Switch to HOLD mode, optional disarm per FS_CRASH_CHECK parameter
 *          
 *          The function maintains a crash_counter that increments at 10Hz when stall conditions
 *          are detected. When counter reaches threshold (CRASH_CHECK_TRIGGER_SEC * 10), crash
 *          is declared and recovery action initiated.
 * 
 * @note Called at 10Hz by the main scheduler (100ms period)
 * @note Function returns early if vehicle is disarmed, crash checking disabled, or in manual modes
 * @note Balance bots have different detection criteria and recovery actions than standard rovers
 * 
 * @warning SAFETY-CRITICAL: This function can disarm the vehicle or change flight mode.
 *          False positives can cause mission abort. False negatives can result in continued
 *          operation during actual crash, potentially causing damage or injury.
 * 
 * @warning Crash detection thresholds are tuned for typical ground vehicles. Custom vehicles
 *          (especially balance bots) may require parameter adjustment via CRASH_ANGLE and
 *          FS_CRASH_CHECK parameters.
 * 
 * @see FS_CRASH_CHECK parameter (0=Disabled, 1=Hold, 2=HoldAndDisarm)
 * @see CRASH_ANGLE parameter (0=Disabled, >0=trigger angle in degrees)
 * @see ModeHold for recovery mode behavior
 * 
 * Source: Rover/crash_check.cpp:11-65
 */
void Rover::crash_check()
{
    // Crash detection state - counts consecutive iterations where stall condition detected
    static uint16_t crash_counter;  // number of iterations vehicle may have been crashed
    
    // Current crash state - true if crash condition met
    bool crashed = false;

    // Early exit conditions - reset counter and return if crash detection not applicable
    // Crash detection only active when:
    // 1. Vehicle is armed (disarmed vehicles can't crash)
    // 2. Crash checking is enabled via FS_CRASH_CHECK parameter
    // 3. Vehicle is in autopilot mode OR is a balance bot (manual modes exempt except balance bots)
    // Rationale: Manual mode pilots can handle crashes manually; balance bots always need protection
    if (!arming.is_armed() || g.fs_crash_check == FS_CRASH_DISABLE || ((!control_mode->is_autopilot_mode()) && (!is_balancebot()))) {
        crash_counter = 0;  // Reset counter to avoid stale state when re-enabling
        return;
    }

    // Attitude-based crash detection: Immediate trigger on excessive pitch/roll
    // Detects rollovers or severe tip conditions that indicate crash regardless of velocity
    // CRASH_ANGLE parameter in degrees (0 = disabled, typical values 30-45 degrees)
    // Uses absolute values to detect excessive angle in any direction
    // @warning Setting CRASH_ANGLE too low causes false positives on rough terrain
    if ((g2.crash_angle != 0) && ((fabsf(ahrs.get_pitch_rad()) > radians(g2.crash_angle)) || (fabsf(ahrs.get_roll_rad()) > radians(g2.crash_angle)))) {
        crashed = true;  // Immediate crash declaration, bypasses time filter
    }

    // @todo Future enhancement: Calculate dynamic minimum velocity based on cruise parameters
    //       Proposed formula: min_vel = (CRASH_CHECK_THROTTLE_MIN * g.speed_cruise) / g.throttle_cruise
    //       Would allow crash detection threshold to scale with vehicle size/speed capabilities

    // Velocity-based crash detection (standard rovers only, not balance bots)
    // Balance bots use attitude-only detection due to different dynamics
    if (!is_balancebot()) {
        // Check if vehicle is NOT in stall condition - any of these conditions indicate normal operation:
        // 1. Groundspeed above minimum threshold (0.08 m/s) - vehicle is moving
        // 2. Yaw rate above minimum threshold (0.08 rad/s) - vehicle is turning in place
        // 3. Throttle below minimum threshold (5%) - vehicle is intentionally stopped/idle
        // If any condition true, vehicle is operating normally - reset counter and exit
        if (!crashed && ((ahrs.groundspeed() >= CRASH_CHECK_VEL_MIN) ||        // Linear motion detected
            (fabsf(ahrs.get_gyro().z) >= CRASH_CHECK_VEL_MIN) ||  // Rotational motion detected
            (fabsf(g2.motors.get_throttle()) < CRASH_CHECK_THROTTLE_MIN))) {   // Low/no throttle (intentional stop)
            crash_counter = 0;  // Reset counter - vehicle operating normally
            return;
        }

        // Stall condition detected: Low velocity + low turn rate + high throttle
        // This indicates motor output without corresponding motion (stall/crash)
        // Increment counter to track duration of stall condition
        crash_counter++;

        // Time filter: Require sustained stall for CRASH_CHECK_TRIGGER_SEC seconds
        // Called at 10Hz, so multiply seconds by 10 to get iteration count
        // 2 seconds * 10 Hz = 20 iterations required to trigger crash
        // @warning Reducing this threshold increases false positive risk from momentary obstacles
        if (crash_counter >= (CRASH_CHECK_TRIGGER_SEC * 10)) {
            crashed = true;  // Sustained stall confirmed - declare crash
        }
    }

    // Crash recovery actions - execute safety response based on vehicle type
    // @warning SAFETY-CRITICAL: This section can disarm motors or change flight mode
    if (crashed) {
        // Log crash event for post-flight analysis
        // Logged as ERROR severity in CRASH_CHECK subsystem for easy identification in logs
        LOGGER_WRITE_ERROR(LogErrorSubsystem::CRASH_CHECK,
                           LogErrorCode::CRASH_CHECK_CRASH);

        // Recovery action varies by vehicle type due to different failure modes
        if (is_balancebot()) {
            // Balance Bot Recovery: IMMEDIATE DISARM
            // Rationale: Balance bots become unstable when crashed/tipped
            // Continued motor output during tip-over can cause violent oscillations and damage
            // Immediate disarm is safest response to prevent further damage or injury
            // @warning Balance bots ALWAYS disarm on crash, cannot be configured otherwise
            GCS_SEND_TEXT(MAV_SEVERITY_EMERGENCY, "Crash: Disarming");
            arming.disarm(AP_Arming::Method::CRASH);  // Immediate disarm, logged as crash-triggered
        } else {
            // Standard Rover Recovery: HOLD MODE + OPTIONAL DISARM
            // Rationale: Standard rovers remain stable when stopped, disarm not always necessary
            // HOLD mode stops vehicle safely while maintaining system power
            // Allows operator to assess situation and potentially recover without full restart
            GCS_SEND_TEXT(MAV_SEVERITY_EMERGENCY, "Crash: Going to HOLD");
            
            // Switch to HOLD mode - stops all motion, maintains position
            // ModeReason::CRASH_FAILSAFE logged for post-flight analysis
            set_mode(mode_hold, ModeReason::CRASH_FAILSAFE);
            
            // Optional disarm based on FS_CRASH_CHECK parameter
            // FS_CRASH_CHECK = 1 (HOLD only) - vehicle remains armed in HOLD mode
            // FS_CRASH_CHECK = 2 (HOLD_AND_DISARM) - vehicle disarms after entering HOLD
            // @note Disarm option allows complete motor shutdown for maximum safety
            if (g.fs_crash_check == FS_CRASH_HOLD_AND_DISARM) {
                arming.disarm(AP_Arming::Method::CRASH);  // Disarm after mode change
            }
        }
    }
}
