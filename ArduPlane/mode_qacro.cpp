/**
 * @file mode_qacro.cpp
 * @brief QACRO (QuadPlane Acrobatic) flight mode implementation
 * 
 * @details QACRO mode provides rate-controlled VTOL flight for aggressive 
 *          aerobatic maneuvers on QuadPlane aircraft. This mode is similar to 
 *          multicopter ACRO mode but adapted for QuadPlane hybrid aircraft with
 *          both VTOL and fixed-wing capabilities.
 *          
 *          In QACRO mode:
 *          - Pilot stick inputs directly command body frame angular rates (roll, pitch, yaw)
 *          - No automatic attitude stabilization or leveling
 *          - Throttle directly controls motor output without altitude hold
 *          - Fixed-wing control surfaces provide additional stabilization
 *          - Allows aggressive VTOL maneuvers like flips and rolls
 *          
 *          This mode requires an experienced pilot as there is no automatic
 *          attitude correction. The aircraft will not self-level.
 *          
 *          Special considerations:
 *          - Tailsitter aircraft have different stick-to-rate mappings due to 
 *            90-degree frame rotation in VTOL mode
 *          - Optional acro_locking parameter affects rate controller behavior
 *          - Fixed-wing stabilization runs concurrently for hybrid control
 * 
 * @note This mode is only available when HAL_QUADPLANE_ENABLED is defined
 * @warning QACRO mode provides no automatic attitude stabilization - intended 
 *          for experienced pilots only. Vehicle will not self-level.
 * 
 * @see ModeAcro (multicopter ACRO mode - similar concept)
 * @see AC_AttitudeControl (for rate controller implementation)
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#include "mode.h"
#include "Plane.h"

#if HAL_QUADPLANE_ENABLED

/**
 * @brief Enter QACRO mode and initialize control state
 * 
 * @details This method is called when transitioning into QACRO mode from any 
 *          other flight mode. It performs the following initialization:
 *          
 *          1. Disables throttle wait state to allow immediate motor control
 *          2. Forces any pending VTOL/fixed-wing transition to complete
 *          3. Relaxes attitude controllers to prevent sudden control jumps
 *          4. Disables yaw rate time constant for traditional acro behavior
 *          5. Captures current vehicle attitude quaternion for acro state
 *          
 *          The yaw rate time constant is disabled to maintain legacy ACRO mode
 *          behavior where yaw stick input provides direct rate control without
 *          time-based rate ramping.
 * 
 * @return true - Mode entry always succeeds
 * 
 * @note This is called automatically by the mode switching logic and should
 *       not be called directly by user code
 * @note The captured quaternion initializes the acro state to prevent attitude
 *       jumps when entering the mode
 * 
 * @see ModeQAcro::run() - Main control loop after mode entry
 * @see AC_AttitudeControl::relax_attitude_controllers()
 */
bool ModeQAcro::_enter()
{
    quadplane.throttle_wait = false;
    quadplane.transition->force_transition_complete();
    attitude_control->relax_attitude_controllers();

    // disable yaw rate time constant to maintain old behaviour
    quadplane.disable_yaw_rate_time_constant();

    IGNORE_RETURN(ahrs.get_quaternion(plane.mode_acro.acro_state.q));

    return true;
}

/**
 * @brief Update navigation targets from attitude controller
 * 
 * @details This method is called at the navigation update rate to synchronize
 *          the plane's navigation targets (nav_roll and nav_pitch) with the
 *          current attitude controller targets from the multicopter/QuadPlane
 *          attitude control system.
 *          
 *          In QACRO mode, the attitude controller maintains internal attitude
 *          targets based on rate commands from pilot input. This method extracts
 *          those targets and publishes them to the main plane navigation variables
 *          for use by other systems (e.g., ground station display, logging).
 *          
 *          The attitude targets are retrieved as Euler angles in centidegrees:
 *          - att_target.x = roll (centidegrees)
 *          - att_target.y = pitch (centidegrees)
 *          - att_target.z = yaw (centidegrees) - not used in this context
 * 
 * @note Called at navigation update rate (typically 50Hz or based on scheduler)
 * @note Units are centidegrees (1/100th of a degree)
 * @note This synchronization allows nav variables to reflect current attitude 
 *       targets for telemetry and logging purposes
 * 
 * @see ModeQAcro::run() - Main control loop that generates attitude targets
 * @see AC_AttitudeControl::get_att_target_euler_cd()
 */
void ModeQAcro::update()
{
    // get nav_roll and nav_pitch from multicopter attitude controller
    Vector3f att_target = plane.quadplane.attitude_control->get_att_target_euler_cd();
    plane.nav_pitch_cd = att_target.y;
    plane.nav_roll_cd = att_target.x;
    return;
}

/**
 * @brief Main control loop for QACRO mode - rate-controlled VTOL aerobatic flight
 * 
 * @details This method implements the primary control logic for QACRO mode, providing
 *          direct rate control of the QuadPlane in VTOL configuration for aggressive
 *          aerobatic maneuvers. This is similar to multicopter ACRO mode but adapted
 *          for QuadPlane hybrid aircraft.
 *          
 *          Control Flow:
 *          1. Check for tailsitter VTOL transition (delegates to fixed-wing if active)
 *          2. If throttle_wait active: Hold motors at ground idle with relaxed attitude
 *          3. If flying: Convert pilot stick inputs to body-frame angular rate targets
 *          4. Apply rate targets to attitude controller
 *          5. Set throttle output directly from pilot input (no altitude hold)
 *          6. Run fixed-wing acro stabilization concurrently
 *          
 *          Pilot Input Mapping:
 *          - Standard configuration:
 *            * Roll stick -> Roll rate (deg/s, scaled by Q_ACRO_ROLL_RATE)
 *            * Pitch stick -> Pitch rate (deg/s, scaled by Q_ACRO_PITCH_RATE)
 *            * Rudder stick -> Yaw rate (deg/s, scaled by Q_ACRO_YAW_RATE)
 *            * Throttle stick -> Direct motor throttle output
 *          
 *          - Tailsitter configuration:
 *            * Roll stick -> Yaw rate (due to 90° frame rotation in VTOL)
 *            * Pitch stick -> Pitch rate (unchanged)
 *            * Rudder stick -> Roll rate (due to 90° frame rotation in VTOL)
 *            * Throttle stick -> Direct motor throttle output
 *          
 *          Rate Controller Behavior:
 *          - If acro_locking enabled (plane.g.acro_locking):
 *            Uses input_rate_bf_roll_pitch_yaw_3 for "acro+" behavior with attitude
 *            locking when sticks are centered
 *          - If acro_locking disabled:
 *            Uses input_rate_bf_roll_pitch_yaw_2 for pure rate control
 *          
 *          Coordinate Frame: Body frame rates in centidegrees per second (cd/s)
 *          Units: Angular rates are in centidegrees/second (1/100 deg/s)
 *          
 *          Safety Considerations:
 *          - No automatic attitude stabilization or self-leveling
 *          - No altitude hold - throttle directly controls motor output
 *          - Angle boost is disabled on throttle to prevent unexpected behavior
 *          - Fixed-wing surfaces provide supplemental stabilization
 *          - Tailsitter transition phase uses fixed-wing controllers for safety
 * 
 * @note Called at main loop rate (typically 400Hz for QuadPlane control)
 * @note This mode requires an experienced pilot - vehicle will not self-level
 * @note Throttle is directly output without angle compensation for predictable control
 * @note Fixed-wing control surfaces remain active for hybrid stabilization
 * 
 * @warning No automatic attitude correction - pilot must manually control all axes
 * @warning Aggressive maneuvers possible - ensure adequate altitude and clearance
 * @warning Different stick mapping for tailsitters due to frame rotation
 * 
 * @see ModeQAcro::_enter() - Mode initialization
 * @see ModeQAcro::update() - Navigation target updates
 * @see AC_AttitudeControl::input_rate_bf_roll_pitch_yaw_2_cds() - Pure rate control
 * @see AC_AttitudeControl::input_rate_bf_roll_pitch_yaw_3_cds() - Rate control with locking
 * @see ModeAcro::run() - Fixed-wing acro stabilization
 */
void ModeQAcro::run()
{
    const uint32_t now = AP_HAL::millis();
    if (quadplane.tailsitter.in_vtol_transition(now)) {
        // Tailsitters in FW pull up phase of VTOL transition run FW controllers
        Mode::run();
        return;
    }

    if (quadplane.throttle_wait) {
        // Throttle wait state: motors at ground idle with no attitude control
        // Used during arming sequence or when pilot hasn't raised throttle yet
        quadplane.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        attitude_control->set_throttle_out(0, true, 0);
        quadplane.relax_attitude_control();
    } else {
        // Active flight: full throttle range available for aerobatic maneuvers
        quadplane.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // Convert pilot stick inputs to desired body frame angular rates (centidegrees/second)
        // Pilot inputs are normalized (-1.0 to +1.0), then scaled by configured rate limits
        // and converted to centidegrees/second for the attitude controller
        float target_roll = 0;
        float target_pitch = plane.channel_pitch->norm_input() * quadplane.acro_pitch_rate * 100.0f;
        float target_yaw = 0;
        if (quadplane.tailsitter.enabled()) {
            // Tailsitter frame transformation: 90 degree Y rotation in VTOL mode swaps body-frame roll and yaw
            // Roll stick commands yaw rate, rudder commands roll rate (reversed for body-frame convention)
            target_roll =  plane.channel_rudder->norm_input() * quadplane.acro_yaw_rate * 100.0f;
            target_yaw  = -plane.channel_roll->norm_input() * quadplane.acro_roll_rate * 100.0f;
        } else {
            // Standard QuadPlane: conventional stick-to-axis mapping
            target_roll = plane.channel_roll->norm_input() * quadplane.acro_roll_rate * 100.0f;
            target_yaw  = plane.channel_rudder->norm_input() * quadplane.acro_yaw_rate * 100.0;
        }

        // Get pilot throttle input directly (no altitude hold or throttle curve)
        float throttle_out = quadplane.get_pilot_throttle();

        // Run attitude rate controller with computed body-frame rate targets
        if (plane.g.acro_locking) {
            // Acro+ mode: locks attitude when sticks centered (rate target = 0)
            attitude_control->input_rate_bf_roll_pitch_yaw_3_cds(target_roll, target_pitch, target_yaw);
        } else {
            // Pure rate mode: continuous rate control without attitude locking
            attitude_control->input_rate_bf_roll_pitch_yaw_2_cds(target_roll, target_pitch, target_yaw);
        }

        // Output pilot's throttle directly without angle boost compensation
        // false = no angle boost, 10.0f = throttle slew rate filter
        attitude_control->set_throttle_out(throttle_out, false, 10.0f);
    }

    // Run fixed-wing acro stabilization concurrently for hybrid control
    // Fixed-wing surfaces provide additional stabilization during VTOL maneuvers
    plane.mode_acro.run();
}

#endif
