/**
 * @file mode_acro.cpp
 * @brief Acro (acrobatic) flight mode implementation for ArduSub
 * 
 * @details This file implements the Acro flight mode for underwater ROVs.
 *          In Acro mode, the pilot has direct rate control over roll, pitch, and yaw axes,
 *          along with manual control of vertical thrust (depth), forward thrust, and lateral thrust.
 *          
 *          Unlike multicopter Acro mode which only controls attitude rates, ArduSub Acro mode
 *          provides 6-DOF (six degrees of freedom) control appropriate for underwater vehicles:
 *          - Roll/Pitch/Yaw: Rate control (degrees per second)
 *          - Vertical: Direct thrust control (replaces altitude hold)
 *          - Forward/Lateral: Direct thrust control (unique to ROV operations)
 *          
 *          This mode provides maximum control authority and is typically used for:
 *          - Precise maneuvering in confined spaces
 *          - Operations requiring direct thrust control
 *          - Advanced piloting when automatic stabilization is not desired
 * 
 * @note This mode requires the vehicle to be armed and motors to be enabled
 * @warning Acro mode provides no automatic stabilization - pilot must actively control all axes
 * 
 * Source: ArduSub/mode_acro.cpp:1-43
 */

#include "Sub.h"

/**
 * @brief Initialize Acro mode
 * 
 * @details Performs initialization when entering Acro mode:
 *          - Resets the desired depth position to zero for accurate depth reporting
 *          - Sets all control inputs to neutral to prevent erratic behavior during mode transition
 *          
 *          Setting neutral controls is critical for underwater vehicles because attitude hold
 *          inputs from previous modes (like Stabilize) become direct thrust inputs in Acro mode.
 *          Without neutralization, residual control inputs could cause unexpected roll, pitch,
 *          or translation movements immediately upon entering Acro mode.
 * 
 * @param[in] ignore_checks If true, skips pre-arm safety checks (typically false for normal operation)
 * 
 * @return true Always returns true - Acro mode can always be entered regardless of vehicle state
 * 
 * @note The ignore_checks parameter is part of the mode interface but not used in Acro mode initialization
 */
bool ModeAcro::init(bool ignore_checks) {
    // set target altitude to zero for reporting
    position_control->set_pos_desired_U_cm(0);

    // attitude hold inputs become thrust inputs in acro mode
    // set to neutral to prevent chaotic behavior (esp. roll/pitch)
    sub.set_neutral_controls();

    return true;
}

/**
 * @brief Main execution loop for Acro mode
 * 
 * @details Executes the Acro mode control logic at the main loop rate (typically 50Hz for ArduSub).
 *          Implements full 6-DOF rate control for underwater ROV operations:
 *          
 *          Control Flow:
 *          1. Safety check: If motors not armed, set to ground idle and exit
 *          2. Enable full motor output authority (THROTTLE_UNLIMITED)
 *          3. Convert pilot stick inputs to desired body-frame rotation rates
 *          4. Execute attitude rate controller for roll, pitch, and yaw
 *          5. Apply vertical thrust directly from throttle input (no altitude hold)
 *          6. Apply forward and lateral thrust directly from pilot inputs
 *          
 *          Key Differences from Multicopter Acro Mode:
 *          - Forward thrust: Direct control via dedicated channel (not available in copters)
 *          - Lateral thrust: Direct control via dedicated channel (not available in copters)
 *          - Vertical thrust: Direct throttle control without angle boost compensation
 *          - 6-DOF control: Full independent control of all translation and rotation axes
 *          
 *          This implementation provides the low-level control necessary for precise ROV
 *          maneuvering in underwater environments where traditional aircraft-style
 *          control modes are not applicable.
 * 
 * @note This function is called every scheduler loop iteration while in Acro mode
 * @note Motors must be armed for any control output - safety feature to prevent accidental activation
 * 
 * @warning All control axes require active pilot input - no automatic stabilization or position hold
 * @warning Forward and lateral thrust can cause unexpected vehicle motion if pilot is not prepared
 */
void ModeAcro::run()
{
    float target_roll, target_pitch, target_yaw;

    // Safety check: if not armed, disable all motor output and exit immediately
    // This prevents any motor movement when the vehicle is disarmed
    if (!motors.armed()) {
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        attitude_control->set_throttle_out(0,true,g.throttle_filt);
        attitude_control->relax_attitude_controllers();
        return;
    }

    // Enable full motor output range - no throttle limiting in Acro mode
    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // Convert pilot stick inputs to desired body-frame rotation rates (centidegrees/second)
    // Applies configured rate scaling and expo curves from parameters
    get_pilot_desired_angle_rates(channel_roll->get_control_in(), channel_pitch->get_control_in(), channel_yaw->get_control_in(), target_roll, target_pitch, target_yaw);

    // Execute attitude rate controller to achieve desired roll, pitch, and yaw rates
    // Controller outputs motor commands to achieve target rates in body frame
    attitude_control->input_rate_bf_roll_pitch_yaw_cds(target_roll, target_pitch, target_yaw);

    // Apply vertical thrust directly from throttle input without angle compensation
    // Second parameter 'false' disables angle boost (not applicable for underwater vehicles)
    attitude_control->set_throttle_out(channel_throttle->norm_input(), false, g.throttle_filt);

    // ROV-specific 6-DOF control: Apply forward and lateral thrust independently
    // These thrust channels are unique to ArduSub and not present in multicopter Acro mode
    // norm_input() returns normalized value in range -1.0 to 1.0 from pilot input
    // Forward thrust: Positive = forward motion, Negative = reverse motion
    motors.set_forward(channel_forward->norm_input());
    // Lateral thrust: Positive = right motion, Negative = left motion
    motors.set_lateral(channel_lateral->norm_input());
}
