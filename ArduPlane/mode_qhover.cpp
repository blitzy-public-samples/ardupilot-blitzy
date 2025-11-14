/**
 * @file mode_qhover.cpp
 * @brief Implements QHOVER mode for QuadPlane VTOL hover with altitude hold using multicopter controls
 * 
 * @details QHOVER mode provides altitude-holding hover capability for QuadPlane aircraft using
 *          multicopter-style controls. The aircraft maintains position using VTOL motors while
 *          the pilot controls horizontal position with stick inputs. Fixed-wing control surfaces
 *          remain active to provide additional stability and control authority during hover.
 */

#include "mode.h"
#include "Plane.h"

// HAL_QUADPLANE_ENABLED guards this entire file as QHOVER only exists on QuadPlane-equipped aircraft
#if HAL_QUADPLANE_ENABLED

/**
 * @brief Initialize QHOVER mode on entry
 * 
 * @details Sets up vertical velocity and acceleration limits for position controller using configured parameters:
 *          - Max descent speed from Q_WP_SPEED_DN (negative, in cm/s)
 *          - Max climb speed from PILOT_SPEED_Z_MAX_UP (converted to cm/s)
 *          - Acceleration from PILOT_ACCEL_Z (converted to cm/s²)
 *          
 *          Initializes climb rate to 0 for smooth entry without sudden vertical movement.
 *          Starts throttle wait state to prevent immediate motor spool-up, allowing pilot
 *          to stabilize controls before motors engage.
 * 
 * @return Always returns true (entry always succeeds)
 * 
 * @note Position controller limits are set in cm/s and cm/s² units
 * @note Throttle wait state must be cleared by pilot input before motors engage
 */
bool ModeQHover::_enter()
{
    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_U_cm(-quadplane.get_pilot_velocity_z_max_dn(), quadplane.pilot_speed_z_max_up*100, quadplane.pilot_accel_z*100);
    pos_control->set_correction_speed_accel_U_cmss(-quadplane.get_pilot_velocity_z_max_dn(), quadplane.pilot_speed_z_max_up*100, quadplane.pilot_accel_z*100);
    quadplane.set_climb_rate_cms(0);

    quadplane.init_throttle_wait();
    return true;
}

/**
 * @brief Update navigation targets for QHOVER mode
 * 
 * @details Delegates to QSTABILIZE update() as hover uses the same target calculation
 *          approach. Navigation updates handle position and velocity target adjustments
 *          based on pilot input and external navigation commands.
 * 
 * @note Called at navigation rate (typically 10-50Hz)
 */
void ModeQHover::update()
{
    plane.mode_qstabilize.update();
}

/**
 * @brief Main control loop execution for QHOVER mode
 * 
 * @details Executes VTOL hover control sequence:
 *          1. check_VTOL_recovery() - Monitor for need to engage VTOL assist if aircraft
 *             is losing altitude or entering dangerous attitude
 *          2. Handle tailsitter FW pull-up phase during transitions (uses FW controllers)
 *             to smoothly transition from fixed-wing to VTOL flight
 *          3. If throttle_wait active: Hold motors at ground idle, relax all controllers
 *             to prevent uncommanded motion until pilot clears wait state
 *          4. Otherwise: Execute hold_hover() with pilot climb rate input, assign tilt servos
 *             for thrust vectoring, maintain position and altitude
 *          5. Stabilize fixed-wing surfaces (roll/pitch) for aerodynamic stability
 *          6. Center rudder output to prevent yaw coupling
 *          7. Apply spin recovery if needed to prevent uncontrolled rotation
 * 
 * @warning This is a safety-critical VTOL control mode - do not modify without thorough
 *          testing in SITL and on physical hardware. Incorrect control sequencing can
 *          lead to loss of control or crash.
 * 
 * @note Fixed-wing surfaces remain active in hover to provide stability and control authority.
 *       This hybrid approach improves hover stability in windy conditions.
 * @note Called at main loop rate (typically 400Hz)
 */
void ModeQHover::run()
{
    quadplane.assist.check_VTOL_recovery();

    const uint32_t now = AP_HAL::millis();
    if (quadplane.tailsitter.in_vtol_transition(now)) {
        // Tailsitters in FW pull up phase of VTOL transition run FW controllers
        Mode::run();
        return;
    }

    if (quadplane.throttle_wait) {
        quadplane.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        attitude_control->set_throttle_out(0, true, 0);
        quadplane.relax_attitude_control();
        pos_control->relax_U_controller(0);
    } else {
        plane.quadplane.assign_tilt_to_fwd_thr();
        quadplane.hold_hover(quadplane.get_pilot_desired_climb_rate_cms());
    }

    // Stabilize with fixed wing surfaces
    plane.stabilize_roll();
    plane.stabilize_pitch();

    // Center rudder
    output_rudder_and_steering(0.0);

    // possibly apply spin recovery
    quadplane.assist.output_spin_recovery();
}

#endif
