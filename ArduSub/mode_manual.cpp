/**
 * @file mode_manual.cpp
 * @brief Manual mode implementation for ArduSub
 * 
 * @details Manual mode provides direct thruster control without any attitude stabilization
 *          or assistance from the autopilot. Pilot inputs are passed directly through to
 *          the motor mixer, providing raw 6-DOF (6 degrees of freedom) control for
 *          underwater vehicles.
 *          
 *          This mode is the most basic control mode and provides no stability augmentation.
 *          Unlike Stabilize mode which maintains level attitude, Manual mode gives the pilot
 *          complete direct control over all thrusters with no autopilot intervention.
 *          
 *          Typical use cases:
 *          - Highly skilled pilots who want maximum control authority
 *          - Testing thruster configuration and motor output
 *          - Emergency situations where stabilization is not desired
 *          - Scenarios requiring very precise manual thruster control
 * 
 * @warning Manual mode requires significant pilot skill and experience. Without attitude
 *          stabilization, the vehicle can easily become unstable or disoriented. This mode
 *          is NOT recommended for novice pilots or normal operations.
 * 
 * @note Manual mode is fundamentally different from Stabilize mode:
 *       - Manual: Direct thruster control, no attitude stabilization
 *       - Stabilize: Attitude-controlled with automatic leveling
 * 
 * Source: ArduSub/mode_manual.cpp
 */

#include "Sub.h"


/**
 * @brief Initialize Manual mode
 * 
 * @details Performs mode initialization when switching into Manual mode:
 *          - Resets position controller altitude target to zero for reporting purposes
 *          - Sets all control inputs to neutral to prevent sudden movements during mode transition
 *          - Prepares the vehicle for direct thruster control without stabilization
 *          
 *          The neutral controls setting is critical for safety - it prevents chaotic behavior
 *          that could occur if the vehicle had non-neutral roll/pitch inputs from a previous
 *          mode when switching to Manual mode's pass-through control.
 * 
 * @param[in] ignore_checks Whether to ignore pre-arm checks (unused in Manual mode as
 *                          no specific checks are required for this basic mode)
 * 
 * @return true Always returns true as Manual mode initialization cannot fail
 * 
 * @note This function is called automatically by the mode switching logic when entering
 *       Manual mode from any other flight mode
 * 
 * @warning Transitioning to Manual mode removes all attitude stabilization. Ensure the
 *          vehicle is in a safe state before switching to this mode.
 * 
 * @see ModeManual::run()
 * @see Sub::set_neutral_controls()
 */
bool ModeManual::init(bool ignore_checks) {
    // set target altitude to zero for reporting
    position_control->set_pos_desired_U_cm(0);

    // attitude hold inputs become thrust inputs in manual mode
    // set to neutral to prevent chaotic behavior (esp. roll/pitch)
    sub.set_neutral_controls();

    return true;
}

/**
 * @brief Run Manual mode controller (passthrough control)
 * 
 * @details Executes the Manual mode control logic at each iteration of the main loop.
 *          This function implements pure passthrough control where pilot inputs are
 *          directly mapped to motor/thruster outputs without any attitude stabilization,
 *          rate limiting, or autopilot assistance.
 *          
 *          Control flow:
 *          1. Safety check: If not armed, set motors to ground idle and exit
 *          2. If armed, enable unlimited throttle authority
 *          3. Direct mapping of pilot inputs to 6-DOF thruster control:
 *             - Roll: Direct pass-through of roll channel input
 *             - Pitch: Direct pass-through of pitch channel input
 *             - Yaw: Pass-through with acro yaw scaling factor
 *             - Throttle (vertical): Normalized to 0.0-1.0 range
 *             - Forward: Direct pass-through of forward channel input
 *             - Lateral: Direct pass-through of lateral channel input
 *          
 *          Unlike Stabilize mode which uses attitude_control to maintain level flight,
 *          Manual mode bypasses all stabilization and directly commands the motor mixer.
 *          This provides maximum control authority but requires constant pilot input to
 *          maintain any desired attitude or position.
 *          
 *          Input normalization: Channel inputs are normalized to -1.0 to +1.0 range
 *          (except throttle which is 0.0 to 1.0 after normalization). These values
 *          are passed directly to the motor library which handles frame-specific mixing.
 * 
 * @note This function must be called at 100Hz or higher for smooth control response.
 *       It is invoked by the main vehicle scheduler at the fast loop rate (typically 400Hz).
 * 
 * @warning Manual mode provides NO attitude stabilization or safety limits:
 *          - The vehicle will not automatically level or hold position
 *          - Excessive inputs can cause instability or loss of control
 *          - Water currents and buoyancy changes require constant pilot correction
 *          - Disorientation is common, especially without visual references
 *          - Recommended only for experienced pilots in controlled environments
 * 
 * @warning When disarmed, all motor outputs are forced to ground idle regardless of
 *          pilot inputs. Always verify arm status before expecting thruster response.
 * 
 * @see ModeManual::init()
 * @see AP_Motors::set_roll()
 * @see AP_Motors::set_pitch()
 * @see AP_Motors::set_yaw()
 * @see AP_Motors::set_throttle()
 * @see AP_Motors::set_forward()
 * @see AP_Motors::set_lateral()
 */
void ModeManual::run()
{
    // if not armed set throttle to zero and exit immediately
    if (!sub.motors.armed()) {
        sub.motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        attitude_control->set_throttle_out(0,true,g.throttle_filt);
        attitude_control->relax_attitude_controllers();
        return;
    }

    sub.motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    sub.motors.set_roll(channel_roll->norm_input());
    sub.motors.set_pitch(channel_pitch->norm_input());
    sub.motors.set_yaw(channel_yaw->norm_input() * g.acro_yaw_p / ACRO_YAW_P);
    sub.motors.set_throttle((channel_throttle->norm_input() + 1.0f) / 2.0f);
    sub.motors.set_forward(channel_forward->norm_input());
    sub.motors.set_lateral(channel_lateral->norm_input());
}
