/**
 * @file afs_rover.cpp
 * @brief Rover-specific implementation of Advanced Failsafe (AFS) system
 * 
 * @details This file implements the rover-specific behavior for the Advanced Failsafe
 *          system, which provides hardware-level failsafe capabilities for vehicle
 *          termination in critical failure scenarios. The AFS system operates
 *          independently of the main autopilot logic and can forcibly terminate
 *          vehicle operation when certain failure conditions are detected.
 * 
 *          Key responsibilities:
 *          - Vehicle termination sequence (disarm and enter HOLD mode)
 *          - Control mode classification for AFS monitoring
 *          - Automatic mode entry on datalink loss
 * 
 *          The Advanced Failsafe is typically used in scenarios where regulatory
 *          requirements or safety considerations mandate a hardware-level kill switch
 *          or termination capability.
 * 
 * @warning This module implements SAFETY-CRITICAL termination logic. Any modifications
 *          must be thoroughly tested and reviewed for safety implications.
 * 
 * @see AP_AdvancedFailsafe
 * @see Rover::set_mode()
 * 
 * Source: Rover/afs_rover.cpp
 */

#include "Rover.h"

#if AP_ROVER_ADVANCED_FAILSAFE_ENABLED

/**
 * @brief Execute Advanced Failsafe vehicle termination sequence
 * 
 * @details This method implements the rover-specific termination behavior when
 *          Advanced Failsafe determines that the vehicle must be immediately stopped.
 *          The termination sequence performs two critical actions:
 *          
 *          1. Disarm the vehicle - Immediately cuts power to motors and prevents
 *             any further throttle output. Uses AP_Arming::Method::AFS to indicate
 *             the disarm was triggered by the Advanced Failsafe system.
 *          
 *          2. Enter HOLD mode - Forces the vehicle into HOLD mode which stops all
 *             autonomous navigation and holds the current position. Uses
 *             ModeReason::CRASH_FAILSAFE to indicate this is a safety-critical
 *             mode change.
 *          
 *          This termination sequence differs from normal failsafe actions by being
 *          more aggressive - it immediately disarms rather than attempting a controlled
 *          stop or return-to-launch sequence.
 * 
 * @warning SAFETY-CRITICAL FUNCTION: This function immediately terminates vehicle
 *          operation and disarms motors. It is called by the Advanced Failsafe system
 *          when critical failure conditions are detected. Vehicle will stop moving
 *          and may become unresponsive until manually rearmed.
 * 
 * @warning Once terminated, the vehicle cannot be rearmed until the AFS conditions
 *          are cleared and the operator manually rearms the system.
 * 
 * @note This function is only active when AP_ROVER_ADVANCED_FAILSAFE_ENABLED is defined
 * @note Termination is non-recoverable without operator intervention
 * 
 * @see AP_Arming::disarm()
 * @see Rover::set_mode()
 * @see AP_AdvancedFailsafe::check()
 * 
 * Source: Rover/afs_rover.cpp:12-19
 */
void AP_AdvancedFailsafe_Rover::terminate_vehicle(void)
{
    // SAFETY-CRITICAL: Immediately disarm the vehicle to stop all motor output
    // This uses the AFS method indicator to record that termination was triggered
    // by Advanced Failsafe rather than pilot command or normal failsafe
    AP::arming().disarm(AP_Arming::Method::AFS);

    // SAFETY-CRITICAL: Force vehicle into HOLD mode to stop all autonomous navigation
    // CRASH_FAILSAFE reason ensures this mode change is logged as a safety event
    // and prevents automatic mode changes until the failsafe is cleared
    rover.set_mode(rover.mode_hold, ModeReason::CRASH_FAILSAFE);
}

/**
 * @brief Classify current rover control mode for Advanced Failsafe monitoring
 * 
 * @details The Advanced Failsafe system needs to know whether the vehicle is in
 *          an autonomous mode or a manual/stabilized mode to determine appropriate
 *          failsafe behavior. This method maps rover-specific control modes to
 *          the generic AFS mode classifications.
 *          
 *          Mode Classification:
 *          - AFS_AUTO: Returned when the rover is in any autopilot-controlled mode
 *            (AUTO, GUIDED, RTL, SMART_RTL, etc.). In these modes, the vehicle is
 *            navigating autonomously and the AFS system monitors for loss of GPS,
 *            datalink, or other critical sensors.
 *          
 *          - AFS_STABILIZED: Returned when the rover is in a manual or semi-manual
 *            mode (MANUAL, ACRO, STEERING, HOLD). In these modes, the pilot has
 *            direct control and different AFS thresholds may apply.
 *          
 *          The AFS system uses this information to determine which failure conditions
 *          should trigger termination. For example, GPS loss in AFS_AUTO may trigger
 *          termination, while GPS loss in AFS_STABILIZED may be tolerated.
 * 
 * @return AP_AdvancedFailsafe::control_mode Classification of current mode
 *         - AFS_AUTO: Vehicle is in autonomous navigation mode
 *         - AFS_STABILIZED: Vehicle is in manual or semi-manual control mode
 * 
 * @note This function is called frequently by the AFS monitoring loop to track
 *       mode changes and adjust failsafe behavior accordingly
 * 
 * @see Mode::is_autopilot_mode()
 * @see AP_AdvancedFailsafe::check()
 * 
 * Source: Rover/afs_rover.cpp:24-30
 */
AP_AdvancedFailsafe::control_mode AP_AdvancedFailsafe_Rover::afs_mode(void)
{
    // Check if current mode is an autopilot-controlled mode (AUTO, GUIDED, RTL, etc.)
    if (rover.control_mode->is_autopilot_mode()) {
        return AP_AdvancedFailsafe::AFS_AUTO;
    }
    // Manual or semi-manual modes (MANUAL, ACRO, STEERING, HOLD)
    return AP_AdvancedFailsafe::AFS_STABILIZED;
}

/**
 * @brief Force vehicle into AUTO mode when datalink is lost
 * 
 * @details This method is called by the Advanced Failsafe system when datalink loss
 *          is detected and the configured AFS action is to continue the mission in
 *          AUTO mode. This allows the vehicle to complete a pre-programmed mission
 *          even if communication with the ground control station is lost.
 *          
 *          The mode change uses ModeReason::GCS_FAILSAFE to indicate that AUTO mode
 *          was entered due to a communication failure rather than pilot command.
 *          This ensures proper logging and allows the system to differentiate between
 *          intentional and failsafe-triggered mode changes.
 *          
 *          Typical use case: Vehicle is executing a waypoint mission when datalink
 *          is lost. Rather than immediately terminating or returning to launch, the
 *          AFS system forces the vehicle into AUTO mode to continue the mission
 *          autonomously. If datalink is restored, the operator can resume control.
 * 
 * @warning This function forces a mode change without pilot input. Ensure that
 *          AUTO mode behavior is properly configured and safe for autonomous
 *          operation without ground station communication.
 * 
 * @note This is an AFS-specific mode change that bypasses normal mode change checks
 * @note The vehicle will continue executing the current mission or return to the
 *       mission start if no mission is loaded
 * 
 * @see Rover::set_mode()
 * @see AP_AdvancedFailsafe::check()
 * @see ModeAuto
 * 
 * Source: Rover/afs_rover.cpp:33-36
 */
void AP_AdvancedFailsafe_Rover::set_mode_auto(void)
{
    // Force entry into AUTO mode with GCS_FAILSAFE reason to indicate
    // this is a datalink loss recovery action, not a pilot command
    rover.set_mode(rover.mode_auto, ModeReason::GCS_FAILSAFE);
}
#endif  // AP_ROVER_ADVANCED_FAILSAFE_ENABLED
