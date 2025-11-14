/**
 * @file mode_sport.cpp
 * @brief Sport flight mode implementation for ArduCopter
 * 
 * @details Sport mode is an intermediate flight mode between Stabilize and Acro,
 *          designed for aggressive manual flying with auto-leveling. It is popular
 *          for FPV racing and sport flying.
 *          
 *          Key characteristics:
 *          - Pilot inputs command lean angles (like Stabilize mode)
 *          - Much faster maximum lean rate than Stabilize (configurable via ACRO_RP_RATE)
 *          - Higher angle limits for aggressive maneuvering
 *          - Auto-leveling prevents excessive angles beyond configured limits
 *          - Altitude hold using vertical position controller
 *          - Trainer mode feedback provides auto-leveling when approaching angle limits
 *          
 *          Sport mode uses rate-limited angle control:
 *          - Pilot stick deflection commands a rate of change in lean angle
 *          - Square root controller smoothly limits angles at configured maximum
 *          - Provides responsive feel while preventing loss of control
 *          
 *          This mode provides the agility of Acro mode while maintaining the safety
 *          of automatic angle limiting, making it ideal for sport flying and FPV racing.
 * 
 * Source: ArduCopter/mode_sport.cpp
 */

#include "Copter.h"

#if MODE_SPORT_ENABLED

/**
 * @brief Initialize Sport flight mode controller
 * 
 * @details Initializes the Sport mode by configuring the vertical position controller
 *          with pilot-commanded speed and acceleration limits. This setup enables
 *          altitude hold functionality while the pilot controls roll/pitch attitudes
 *          aggressively.
 *          
 *          Initialization sequence:
 *          1. Configure vertical speed limits (up/down) from pilot parameters
 *          2. Set vertical acceleration limit for smooth altitude changes
 *          3. Initialize vertical position controller if not already active
 *          
 *          Sport mode uses the same altitude controller as AltHold mode, providing
 *          stable altitude hold while permitting aggressive attitude maneuvers.
 * 
 * @param[in] ignore_checks Not used in Sport mode - included for interface consistency
 * 
 * @return true Always returns true as Sport mode has no pre-flight checks that could fail
 * 
 * @note Called once when switching into Sport mode from another flight mode
 * @note Vertical position controller may already be active from previous mode
 * 
 * @see ModeSport::run()
 * @see AC_PosControl::set_max_speed_accel_U_cm()
 * @see AC_PosControl::init_U_controller()
 */
bool ModeSport::init(bool ignore_checks)
{
    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_U_cm(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    pos_control->set_correction_speed_accel_U_cmss(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // initialise the vertical position controller
    if (!pos_control->is_active_U()) {
        pos_control->init_U_controller();
    }

    return true;
}

/**
 * @brief Main Sport mode controller - executes the Sport flight mode logic
 * 
 * @details Implements Sport mode control loop combining aggressive rate-commanded
 *          attitude control with altitude hold. This function is the heart of Sport
 *          mode, processing pilot inputs and commanding aircraft attitude and altitude.
 *          
 *          Sport Mode Control Algorithm:
 *          
 *          1. ATTITUDE CONTROL (Roll/Pitch):
 *             - Pilot stick inputs command rates of angle change (like Acro mode)
 *             - Rate is configurable via ACRO_RP_RATE parameter (default higher than Stabilize)
 *             - Trainer mode provides auto-leveling feedback when near configured angle limits
 *             - Square root controller smoothly limits angles at ANGLE_MAX without hard stops
 *          
 *          2. TRAINER MODE AUTO-LEVELING:
 *             - Applies automatic leveling force proportional to current lean angle
 *             - Strength controlled by ACRO_BAL_ROLL and ACRO_BAL_PITCH parameters
 *             - Prevents excessive angles while allowing aggressive maneuvering
 *             - Uses ACRO_LEVEL_MAX_ANGLE to define trainer mode activation threshold
 *          
 *          3. ANGLE LIMITING:
 *             - If lean angle exceeds ANGLE_MAX, square root controller engages
 *             - Controller smoothly reduces rate to prevent angle overshoot
 *             - Provides soft limiting rather than hard angle clipping
 *             - Pilot retains control feel while being protected from extreme angles
 *          
 *          4. YAW CONTROL:
 *             - Direct yaw rate control from pilot input
 *             - No auto-heading hold (pilot must actively control yaw)
 *          
 *          5. ALTITUDE CONTROL:
 *             - Altitude hold mode using vertical position controller
 *             - Pilot throttle commands climb/descent rate
 *             - Supports surface tracking with rangefinder (if enabled)
 *             - State machine handles takeoff, landing, and flying states
 *          
 *          Control Flow:
 *          1. Update vertical speed/acceleration limits
 *          2. Apply SIMPLE mode transformation if enabled
 *          3. Convert pilot inputs to target rates (roll, pitch, yaw)
 *          4. Apply trainer mode auto-leveling feedback
 *          5. Apply square root controller angle limiting if needed
 *          6. Process altitude state machine (MotorStopped/Landed/Takeoff/Flying)
 *          7. Send rate commands to attitude controller
 *          8. Update vertical position controller for altitude hold
 *          
 *          Rate-Limited Angle Control Details:
 *          - Pilot input → rate of angle change (deg/s or rad/s)
 *          - Integration over time → lean angle
 *          - Trainer mode feedback → auto-leveling when angle approaches limits
 *          - Square root controller → soft limiting at maximum configured angle
 *          - Result: Responsive control with safety limiting
 * 
 * @note This function MUST be called at 100Hz or faster for proper controller performance
 * @note Higher call rates improve attitude control response and angle limiting smoothness
 * 
 * @warning Modifying the angle limiting algorithm can affect vehicle stability and safety
 * @warning Disabling angle limits (setting ANGLE_MAX too high) can lead to loss of control
 * 
 * @see ModeSport::init()
 * @see AC_AttitudeControl::input_euler_rate_roll_pitch_yaw_rads()
 * @see AC_PosControl::set_pos_target_U_from_climb_rate_cm()
 * @see sqrt_controller() - Square root controller for smooth angle limiting
 * @see get_alt_hold_state() - Altitude state machine determination
 */
void ModeSport::run()
{
    // Set vertical speed and acceleration limits for altitude controller
    // These limits determine how fast the vehicle can climb/descend in response to throttle input
    pos_control->set_max_speed_accel_U_cm(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // Apply SIMPLE mode transform if enabled
    // SIMPLE mode rotates pilot inputs relative to initial heading for easier orientation
    update_simple_mode();

    // Convert pilot stick inputs to target angular rates for Sport mode
    // Sport mode uses ACRO_RP_RATE parameter which is typically much higher than Stabilize mode
    // Normalized input (-1.0 to 1.0) is scaled by configured rate (deg/s) and converted to radians
    // This provides the aggressive, rate-commanded control characteristic of Sport mode
    float target_roll_rads = channel_roll->norm_input_dz() * radians(g2.command_model_acro_rp.get_rate());
    float target_pitch_rads = channel_pitch->norm_input_dz() * radians(g2.command_model_acro_rp.get_rate());

    // get pilot's desired yaw rate
    float target_yaw_rads = get_pilot_desired_yaw_rate_rads();

    // Get current attitude target to implement trainer mode auto-leveling
    const Vector3f att_target_euler_rad = attitude_control->get_att_target_euler_rad();

    // TRAINER MODE AUTO-LEVELING FOR ROLL AXIS
    // Calculate automatic leveling feedback proportional to current lean angle
    // This provides gentle auto-leveling that increases as angle increases
    // ACRO_BAL_ROLL parameter controls the strength of auto-leveling (0=off, 1=max)
    // Angle is constrained to +/- ACRO_LEVEL_MAX_ANGLE_RAD to limit trainer mode range
    float roll_angle_rad = wrap_PI(att_target_euler_rad.x);
    target_roll_rads -= constrain_float(roll_angle_rad, -ACRO_LEVEL_MAX_ANGLE_RAD, ACRO_LEVEL_MAX_ANGLE_RAD) * g.acro_balance_roll;

    // TRAINER MODE AUTO-LEVELING FOR PITCH AXIS
    // Same auto-leveling algorithm applied to pitch axis
    // Helps prevent nose-down dives or excessive climb angles
    float pitch_angle_rad = wrap_PI(att_target_euler_rad.y);
    target_pitch_rads -= constrain_float(pitch_angle_rad, -ACRO_LEVEL_MAX_ANGLE_RAD, ACRO_LEVEL_MAX_ANGLE_RAD) * g.acro_balance_pitch;

    // Get configured maximum lean angle for this vehicle (ANGLE_MAX parameter)
    const float angle_max_rad = attitude_control->lean_angle_max_rad();

    // SQUARE ROOT CONTROLLER ANGLE LIMITING FOR ROLL AXIS
    // When lean angle exceeds ANGLE_MAX, apply corrective rate to bring angle back within limits
    // Square root controller provides smooth, progressive limiting without hard stops
    // This maintains control feel while preventing dangerous angles
    //
    // Controller inputs:
    // - Error: How far beyond limit (angle_max_rad - current_angle)
    // - Rate limit: Maximum rate command / allowed overshoot tolerance
    // - Acceleration: Maximum roll acceleration for smooth limiting
    // - dt: Loop time step (G_Dt)
    //
    // Positive roll angles (right lean) - add negative rate to reduce angle
    if (roll_angle_rad > angle_max_rad){
        target_roll_rads +=  sqrt_controller(angle_max_rad - roll_angle_rad, radians(g2.command_model_acro_rp.get_rate()) / ACRO_LEVEL_MAX_OVERSHOOT_RAD, attitude_control->get_accel_roll_max_radss(), G_Dt);
    }else if (roll_angle_rad < -angle_max_rad) {
        // Negative roll angles (left lean) - add positive rate to reduce angle
        target_roll_rads +=  sqrt_controller(-angle_max_rad - roll_angle_rad, radians(g2.command_model_acro_rp.get_rate()) / ACRO_LEVEL_MAX_OVERSHOOT_RAD, attitude_control->get_accel_roll_max_radss(), G_Dt);
    }

    // SQUARE ROOT CONTROLLER ANGLE LIMITING FOR PITCH AXIS
    // Same smooth angle limiting applied to pitch axis
    // Prevents excessive nose-up or nose-down attitudes beyond ANGLE_MAX
    //
    // Positive pitch angles (nose up) - add negative rate to reduce angle
    if (pitch_angle_rad > angle_max_rad){
        target_pitch_rads +=  sqrt_controller(angle_max_rad - pitch_angle_rad, radians(g2.command_model_acro_rp.get_rate()) / ACRO_LEVEL_MAX_OVERSHOOT_RAD, attitude_control->get_accel_pitch_max_radss(), G_Dt);
    }else if (pitch_angle_rad < -angle_max_rad) {
        // Negative pitch angles (nose down) - add positive rate to reduce angle
        target_pitch_rads +=  sqrt_controller(-angle_max_rad - pitch_angle_rad, radians(g2.command_model_acro_rp.get_rate()) / ACRO_LEVEL_MAX_OVERSHOOT_RAD, attitude_control->get_accel_pitch_max_radss(), G_Dt);
    }

    // ALTITUDE CONTROL - Convert pilot throttle input to climb rate command
    // Throttle stick position maps to desired vertical velocity (cm/s)
    // Center stick = altitude hold, up = climb, down = descend
    float target_climb_rate_cms = get_pilot_desired_climb_rate();
    target_climb_rate_cms = constrain_float(target_climb_rate_cms, -get_pilot_speed_dn(), g.pilot_speed_up);

    // ALTITUDE STATE MACHINE - Determine current flight state based on climb rate and vehicle status
    // States: MotorStopped, Landed_Ground_Idle, Landed_Pre_Takeoff, Takeoff, Flying
    // State machine handles different controller behaviors for ground, takeoff, and flight
    AltHoldModeState sport_state = get_alt_hold_state(target_climb_rate_cms);

    // STATE MACHINE EXECUTION - Handle each altitude state appropriately
    switch (sport_state) {

    case AltHoldModeState::MotorStopped:
        // MOTORS STOPPED STATE - Vehicle is disarmed or motors are stopped
        // Reset all controllers to safe initial state
        attitude_control->reset_rate_controller_I_terms();  // Clear integrator wind-up
        attitude_control->reset_yaw_target_and_rate(false); // Reset yaw to current heading
        pos_control->relax_U_controller(0.0f);              // Force throttle to zero
        break;

    case AltHoldModeState::Landed_Ground_Idle:
        // LANDED GROUND IDLE STATE - Vehicle is on ground, motors at idle
        // Reset yaw to prevent unexpected rotation on takeoff
        attitude_control->reset_yaw_target_and_rate();
        FALLTHROUGH;

    case AltHoldModeState::Landed_Pre_Takeoff:
        // LANDED PRE-TAKEOFF STATE - Vehicle armed and ready for takeoff
        // Smoothly reset rate controller integrators to prevent takeoff jump
        // Relax altitude controller with zero throttle until pilot commands takeoff
        attitude_control->reset_rate_controller_I_terms_smoothly();
        pos_control->relax_U_controller(0.0f);   // forces throttle output to decay to zero
        break;

    case AltHoldModeState::Takeoff:
        // TAKEOFF STATE - Vehicle is executing automatic takeoff sequence
        // Initiate takeoff to configured altitude (PILOT_TKOFF_ALT parameter)
        if (!takeoff.running()) {
            takeoff.start(constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f));
        }

        // Apply obstacle avoidance adjustments to climb rate if avoidance is active
        target_climb_rate_cms = get_avoidance_adjusted_climbrate_cms(target_climb_rate_cms);

        // Execute takeoff with pilot-adjustable climb rate
        // Pilot can speed up or slow down takeoff with throttle input
        takeoff.do_pilot_takeoff(target_climb_rate_cms);
        break;

    case AltHoldModeState::Flying:
        // FLYING STATE - Normal flight, full altitude control active
        // Enable full throttle range for climb and descent
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // Apply obstacle avoidance adjustments to climb rate if avoidance is active
        target_climb_rate_cms = get_avoidance_adjusted_climbrate_cms(target_climb_rate_cms);

#if AP_RANGEFINDER_ENABLED
        // SURFACE TRACKING - Update altitude reference based on rangefinder measurements
        // Allows terrain following in Sport mode if rangefinder is available
        copter.surface_tracking.update_surface_offset();
#endif

        // Send commanded climb rate to vertical position controller
        // Controller maintains altitude or climbs/descends at commanded rate
        pos_control->set_pos_target_U_from_climb_rate_cm(target_climb_rate_cms);
        break;
    }

    // ATTITUDE CONTROLLER - Send computed rate commands to attitude controller
    // Rates include: pilot input + trainer mode leveling + angle limiting
    // Attitude controller converts rate commands to motor outputs via PID loops
    // This is where Sport mode's rate-commanded control is actually executed
    attitude_control->input_euler_rate_roll_pitch_yaw_rads(target_roll_rads, target_pitch_rads, target_yaw_rads);

    // VERTICAL POSITION CONTROLLER - Update altitude controller and set throttle output
    // Converts desired climb rate or altitude target to motor thrust commands
    // Integrates with attitude controller output to produce final motor commands
    // Controller runs at loop rate (typically 400Hz) for smooth altitude hold
    pos_control->update_U_controller();
}

#endif
