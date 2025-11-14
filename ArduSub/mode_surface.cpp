/**
 * @file mode_surface.cpp
 * @brief Surface mode implementation for controlled ascent to the water surface
 * 
 * @details Surface mode provides a controlled ascent from depth to the water surface
 *          with the following characteristics:
 *          - Automatic vertical ascent at a safe, controlled rate
 *          - Pilot retains control over roll, pitch, yaw, and horizontal positioning
 *          - Attitude stabilization during ascent to maintain vehicle orientation
 *          - Surface breach detection using barometer (depth sensor)
 *          - Automatic transition to ALT_HOLD mode upon reaching surface
 *          - Fallback thrust mode when barometer is unavailable
 * 
 *          The mode is designed for safe surfacing operations, protecting against
 *          rapid uncontrolled ascents that could cause vehicle damage or pose safety
 *          risks to surface traffic.
 * 
 * @note This mode requires a functioning barometer for depth-based ascent control.
 *       Without a barometer, the mode falls back to fixed-thrust operation.
 * 
 * @warning Surface operations present unique hazards:
 *          - Boat traffic and surface obstacles
 *          - Wave action and surface turbulence
 *          - Loss of underwater communication
 *          - Potential for rapid depth changes near surface
 *          Always ensure the surface area is clear before ascending.
 * 
 * Source: ArduSub/mode_surface.cpp
 */

#include "Sub.h"

/**
 * @brief Initialize Surface mode for controlled ascent to water surface
 * 
 * @details Initializes the vertical position controller and configures ascent parameters
 *          for safe surfacing operations. The initialization:
 *          1. Checks barometer availability and sets fallback mode if needed
 *          2. Configures vertical speed limits for safe ascent rate
 *          3. Initializes position controller for vertical motion control
 *          4. Sets acceleration limits to prevent abrupt depth changes
 * 
 *          The mode uses the pilot-configured speed parameters (pilot_speed_up,
 *          pilot_speed_dn, pilot_accel_z) to ensure ascent rates match operator
 *          preferences and vehicle capabilities.
 * 
 * @param[in] ignore_checks If true, skip pre-flight checks (currently unused)
 * 
 * @return true Always returns true as Surface mode has no blocking init conditions
 * 
 * @note Sets nobaro_mode flag if barometer check fails, triggering thrust-based
 *       fallback control instead of depth-based ascent
 * 
 * @warning Without a functioning barometer, ascent rate cannot be precisely
 *          controlled and relies on fixed thrust output which may vary with
 *          vehicle loading and water conditions
 * 
 * @see ModeSurface::run()
 * @see AC_PosControl::set_max_speed_accel_U_cm()
 * @see AC_PosControl::init_U_controller()
 */
bool ModeSurface::init(bool ignore_checks)
{
    nobaro_mode = !sub.control_check_barometer();

    // initialize vertical speeds and acceleration
    position_control->set_max_speed_accel_U_cm(-sub.get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    position_control->set_correction_speed_accel_U_cmss(-sub.get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // initialise position and desired velocity
    position_control->init_U_controller();

    return true;

}

/**
 * @brief Execute Surface mode control loop for ascending to water surface
 * 
 * @details Implements the main control loop for Surface mode, executing at the
 *          main loop rate (typically 50Hz). The function handles two operational modes:
 * 
 *          **Barometer Available (Normal Operation):**
 *          1. Monitors surface detection flag (sub.ap.at_surface)
 *          2. Transitions to ALT_HOLD mode automatically upon surface breach
 *          3. Processes pilot input for roll, pitch, and yaw control
 *          4. Maintains attitude stabilization during ascent
 *          5. Commands vertical ascent at configured safe rate
 *          6. Allows pilot control of forward/lateral positioning
 * 
 *          **No Barometer (Fallback Mode):**
 *          - Uses fixed thrust output based on SURFACE_NOBARO_THRUST parameter
 *          - Thrust maps from parameter range [-100, 100] to output [0, 1]
 *          - No automatic surface detection or mode transition
 *          - Pilot must manually exit mode
 * 
 *          **Ascent Algorithm:**
 *          The ascent rate is controlled by the position controller using the
 *          waypoint navigation default speed (wp_nav.get_default_speed_up_cms()).
 *          This rate is constrained to the configured maximum vertical speed to
 *          prevent unsafe rapid ascents. The rate limiting ensures:
 *          - Controlled buoyancy compensation
 *          - Time for pilot reaction to obstacles
 *          - Reduced risk of broaching (breaking surface at excessive speed)
 *          - Protection against pressure-related damage from rapid depth changes
 * 
 *          **Attitude Stabilization:**
 *          During ascent, the attitude controller maintains vehicle orientation
 *          based on pilot input, preventing uncontrolled rolling or pitching
 *          that could occur due to uneven buoyancy or current effects.
 * 
 *          **Surface Breach Detection:**
 *          The sub.ap.at_surface flag is set by the barometer/depth sensor when
 *          the vehicle reaches near-zero depth. Upon detection, the mode
 *          automatically transitions to ALT_HOLD to prevent the vehicle from
 *          continuing to climb and potentially breaching the surface excessively.
 * 
 * @note Called at main loop rate (typically 50Hz). Pilot retains full control
 *       over horizontal positioning (forward/lateral) and orientation (roll/pitch/yaw)
 *       throughout the ascent.
 * 
 * @note The climb rate is constrained with a minimum of 1 cm/s to ensure positive
 *       ascent even with low speed configurations.
 * 
 * @warning Surface operations are safety-critical:
 *          - Always verify surface area is clear of boat traffic and obstacles
 *          - Be prepared for wave action and turbulence near surface
 *          - Monitor for loss of position hold in currents
 *          - Excessive ascent rates can cause vehicle damage or loss of control
 *          - Surface breach may result in GPS acquisition and potential mode changes
 * 
 * @warning In nobaro_mode, ascent rate is uncontrolled and depends on vehicle
 *          buoyancy, loading, and SURFACE_NOBARO_THRUST parameter. Operator must
 *          carefully tune this parameter and manually monitor ascent.
 * 
 * @see ModeSurface::init()
 * @see AC_AttitudeControl::input_euler_angle_roll_pitch_euler_rate_yaw_cd()
 * @see AC_PosControl::set_pos_target_U_from_climb_rate_cm()
 * @see AC_PosControl::update_U_controller()
 */
void ModeSurface::run()
{
    float target_roll, target_pitch;

    // if not armed set throttle to zero and exit immediately
    if (!motors.armed()) {
        motors.output_min();
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        attitude_control->set_throttle_out(0,true,g.throttle_filt);
        attitude_control->relax_attitude_controllers();
        position_control->init_U_controller();
        return;
    }

    // If no barometer is available, use the surface_nobaro_thrust parameter to set the throttle output
    if (nobaro_mode) {
        float thrust_output = 0.5f + g2.surface_nobaro_thrust * 0.005f; // map -100, 100 to 0, 1
        attitude_control->set_throttle_out(thrust_output, true, g.throttle_filt);
    } else {
        // Already at surface, hold depth at surface
        if (sub.ap.at_surface) {
            set_mode(Mode::Number::ALT_HOLD, ModeReason::SURFACE_COMPLETE);
        }

        // convert pilot input to lean angles
        // To-Do: convert sub.get_pilot_desired_lean_angles to return angles as floats
        sub.get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, sub.aparm.angle_max);

        // get pilot's desired yaw rate
        float target_yaw_rate = sub.get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_cd(target_roll, target_pitch, target_yaw_rate);

        // set target climb rate
        float cmb_rate_cms = constrain_float(fabsf(sub.wp_nav.get_default_speed_up_cms()), 1, position_control->get_max_speed_up_cms());

        // update altitude target and call position controller
        position_control->set_pos_target_U_from_climb_rate_cm(cmb_rate_cms);
        position_control->update_U_controller();
    }
    // pilot has control for repositioning
    motors.set_forward(channel_forward->norm_input());
    motors.set_lateral(channel_lateral->norm_input());
}
