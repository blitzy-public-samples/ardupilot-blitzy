/**
 * @file mode_acro_heli.cpp
 * @brief Acro flight mode implementation for traditional helicopters
 * 
 * @details This file implements the Acro (rate-controlled) flight mode specifically
 *          adapted for traditional helicopters with single main rotor and tail rotor
 *          configurations. Unlike multirotor acro mode, this implementation handles:
 *          
 *          - Collective pitch control instead of throttle-based thrust control
 *          - Tail rotor yaw control with compensation for main rotor torque
 *          - Rotor speed governor integration for maintaining consistent rotor RPM
 *          - Flybar passthrough for mechanically stabilized helicopters
 *          - Virtual flybar simulation for flybarless helicopters
 *          - Negative collective support for inverted flight maneuvers
 *          - Autorotation capability (engine-off descent) with appropriate control authority
 *          
 *          Traditional helicopters differ fundamentally from multirotors:
 *          - Main rotor collective pitch controls vertical thrust (not motor speed)
 *          - Cyclic pitch controls roll/pitch attitude through rotor disk tilt
 *          - Tail rotor provides yaw authority and compensates for main rotor torque
 *          - Rotor speed governor maintains constant RPM independent of collective
 *          
 *          Safety considerations:
 *          - Maintains servo authority even when disarmed for pre-flight checks
 *          - Supports autorotation flight with motors not running
 *          - Preserves control during rotor spool-up and spool-down phases
 * 
 * Source: ArduCopter/mode_acro_heli.cpp
 * 
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#include "Copter.h"

#if MODE_ACRO_ENABLED

#if FRAME_CONFIG == HELI_FRAME

/**
 * @brief Initialize traditional helicopter acro flight mode controller
 * 
 * @details Configures the attitude controller and motor output for traditional helicopter
 *          acro mode operation. This initialization handles helicopter-specific setup:
 *          
 *          - Flybar detection: If helicopter has mechanical flybar stabilization,
 *            enables passthrough mode to send pilot inputs directly to servos without
 *            electronic stabilization (mechanical flybar provides stability)
 *          
 *          - Acro tail mode: Enables acro-specific tail rotor control that provides
 *            yaw rate control with appropriate torque compensation. Unlike stabilize
 *            mode which holds heading, acro tail allows continuous rotation.
 *          
 *          - Full collective range: Disables stabilize collective limiting to allow
 *            full collective pitch range from maximum negative (inverted flight) to
 *            maximum positive (maximum climb). This is critical for aerobatic maneuvers
 *            including loops, rolls, and inverted flight where negative collective is required.
 * 
 * @param[in] ignore_checks If true, skip pre-flight checks (currently unused as acro
 *                          always allows entry for helicopters - even on ground for servo checks)
 * 
 * @return true Always returns true - helicopters can always enter acro mode
 * 
 * @note Called once when pilot selects acro mode via mode switch
 * @note Helicopters always allow acro mode entry (no pre-arm checks) to permit
 *       ground-based servo operational checks before flight
 * 
 * @warning Acro mode provides NO attitude stabilization - pilot must maintain control.
 *          Unlike multirotors, helicopters can enter complex aerobatic states requiring
 *          skilled pilot intervention.
 * 
 * Source: ArduCopter/mode_acro_heli.cpp:11-23
 */
bool ModeAcro_Heli::init(bool ignore_checks)
{
    // if heli is equipped with a flybar, then tell the attitude controller to pass through controls directly to servos
    attitude_control->use_flybar_passthrough(motors->has_flybar(), motors->supports_yaw_passthrough());

    motors->set_acro_tail(true);
    
    // set stab collective false to use full collective pitch range
    copter.input_manager.set_use_stab_col(false);

    // always successfully enter acro
    return true;
}

/**
 * @brief Main acro mode controller for traditional helicopters
 * 
 * @details Executes the acro flight mode control loop for traditional helicopters,
 *          converting pilot stick inputs to rate commands and collective pitch output.
 *          This function handles the complete control flow from pilot input to servo output.
 *          
 *          Traditional helicopter acro differs from multirotor acro in several key ways:
 *          
 *          1. Collective Pitch Control:
 *             - Throttle stick controls collective pitch angle, not motor speed
 *             - Full collective range enabled: negative for inverted, positive for upright
 *             - Rotor speed maintained by governor, independent of collective
 *             - Direct mapping without angle boost (no need for tilt compensation)
 *          
 *          2. Tail Rotor Control:
 *             - Yaw control through tail rotor collective pitch
 *             - Automatic compensation for main rotor torque reaction
 *             - Supports external gyro passthrough for flybar helicopters
 *             - Acro tail mode allows continuous yaw rotation
 *          
 *          3. Autorotation Support:
 *             - Maintains full control authority when motors not running
 *             - Does NOT reset attitude targets during autorotation (motor runup check bypassed)
 *             - Allows realistic servo movement when disarmed for operational checks
 *             - Critical for emergency engine-off landing capability
 *          
 *          4. Flybar Handling:
 *             - Mechanical flybar: Direct passthrough of pilot inputs to servos
 *             - Flybarless: Virtual flybar simulation with configurable leak rates
 *             - Virtual flybar provides self-leveling tendency for easier control
 *          
 *          5. Spool State Management:
 *             - SHUT_DOWN: Motors stopped, reset attitude targets
 *             - GROUND_IDLE: On ground or autorotation, conditional reset
 *             - SPOOLING_UP/DOWN: Transitioning, maintain current state
 *             - THROTTLE_UNLIMITED: Full authority, ready for flight
 *          
 *          Control flow:
 *          1. Determine desired motor spool state based on arm status
 *          2. Handle attitude reset logic based on spool state and landing status
 *          3. Convert pilot inputs to desired body-frame rates
 *          4. Apply virtual flybar (flybarless) or passthrough (flybar)
 *          5. Execute attitude controller to generate servo commands
 *          6. Set collective pitch from pilot throttle input
 * 
 * @note Called at main loop rate (typically 400Hz for helicopters)
 * @note Unlike multirotors, does NOT zero collective when disarmed (maintains servo position)
 * @note Supports flight with motors not running (autorotation) - unique to helicopters
 * 
 * @warning In acro mode, pilot is fully responsible for attitude control.
 *          Helicopters can enter unusual attitudes requiring immediate corrective input.
 * @warning Negative collective in inverted flight requires skilled pilot - incorrect
 *          input can result in rapid altitude loss or loss of control.
 * @warning Leaky integrator behavior (when enabled) differs from traditional integrator:
 *          automatically bleeds off accumulated error over time.
 * 
 * Source: ArduCopter/mode_acro_heli.cpp:25-126
 */
void ModeAcro_Heli::run()
{
    float target_roll_rads, target_pitch_rads, target_yaw_rads;
    float pilot_throttle_scaled;

    // Traditional helicopter flight characteristics differ from multirotors:
    // - Autorotation capability: helicopter can fly with engine off, descending in controlled manner
    //   using rotor windmilling to maintain rotor speed. Must NOT reset attitude targets during
    //   autorotation or pilot will lose control authority.
    // - Pre-flight servo checks: servos must move realistically when disarmed so pilot can verify
    //   mechanical linkages, servo response, and control direction before flight.
    // - Collective pitch control: Unlike multirotors which zero throttle when disarmed, helicopters
    //   maintain collective pitch servo position to allow swashplate movement during ground checks.

    if (!motors->armed()) {
        // Motors should be Stopped
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
    } else {
        // heli will not let the spool state progress to THROTTLE_UNLIMITED until motor interlock is enabled
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    }

    // Spool state machine determines motor/rotor state and appropriate control authority:
    // SHUT_DOWN: Engine off, rotor stopped or windmilling slowly - reset all targets
    // GROUND_IDLE: Low power ground operations or autorotation practice - conditional reset
    // SPOOLING_UP/DOWN: Rotor speed transitioning - maintain current state
    // THROTTLE_UNLIMITED: Full power available, rotor at operating RPM - full control authority
    switch (motors->get_spool_state()) {
    case AP_Motors::SpoolState::SHUT_DOWN:
        // Motors completely stopped - reset attitude targets and integrators for clean startup
        // This is the only state where we aggressively reset everything
        attitude_control->reset_target_and_rate(false);
        attitude_control->reset_rate_controller_I_terms();
        break;
    case AP_Motors::SpoolState::GROUND_IDLE:
        // At ground idle: could be landed on ground OR practicing autorotation in flight
        // Leaky integrator: automatically bleeds off I-term, reset on arming
        // Standard integrator: only reset when confirmed landed to avoid disrupting autorotation
        if ((motors->init_targets_on_arming() && motors->using_leaky_integrator()) || (copter.ap.land_complete && !motors->using_leaky_integrator())) {
            attitude_control->reset_target_and_rate(false);
            attitude_control->reset_rate_controller_I_terms_smoothly();
        }
        break;
    case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
        // Full power available, rotor at governed speed
        // Only reset integrators if landed (prevents I-term windup on ground)
        // Standard integrator only - leaky integrator self-manages
        if (copter.ap.land_complete && !motors->using_leaky_integrator()) {
            attitude_control->reset_rate_controller_I_terms_smoothly();
        }
        break;
    case AP_Motors::SpoolState::SPOOLING_UP:
    case AP_Motors::SpoolState::SPOOLING_DOWN:
        // Rotor transitioning between speeds - maintain current control state
        // Governor is ramping rotor RPM, don't disrupt attitude controller
        break;
    }

    // Flybarless helicopters: Electronic stabilization with optional virtual flybar
    // Flybar helicopters: Mechanical stabilization with direct servo passthrough
    if (!motors->has_flybar()){
        // FLYBARLESS CONTROL PATH:
        // Convert pilot stick inputs to desired body-frame rotation rates
        // Deadzone applied to prevent drift from stick center position
        get_pilot_desired_rates_rads(channel_roll->norm_input_dz(), channel_pitch->norm_input_dz(), channel_yaw->norm_input_dz(), target_roll_rads, target_pitch_rads, target_yaw_rads);
        
        // Virtual flybar simulation: adds self-leveling tendency like mechanical flybar
        // Makes helicopter easier to fly by automatically reducing attitude error over time
        // Only active when trainer mode is OFF (trainer mode provides different assistance)
        if ((Trainer)g.acro_trainer.get() == Trainer::OFF) {
            // On ground: aggressive self-leveling (3.0 leak rate) for safety and easier ground handling
            if (copter.ap.land_complete) {
                virtual_flybar(target_roll_rads, target_pitch_rads, target_yaw_rads, 3.0f, 3.0f);
            // In flight: pilot-configurable self-leveling (acro_balance parameters)
            // Lower values = more aggressive leveling, higher values = more rate-like response
            } else {
                virtual_flybar(target_roll_rads, target_pitch_rads, target_yaw_rads, g.acro_balance_pitch, g.acro_balance_roll);
            }
        }
        // Tail rotor control: external gyro passthrough if supported
        // External gyro (often on flybar helis) provides its own yaw stabilization
        if (motors->supports_yaw_passthrough()) {
            // External gyro mode: pass pilot yaw input directly to tail rotor servo
            // No deadzone, no rate processing - gyro handles stabilization
            // This allows gyro to provide heading hold or rate mode based on gyro settings
            target_yaw_rads = cd_to_rad(channel_yaw->get_control_in_zero_dz());
        }

        // Execute attitude controller to convert rate targets to servo commands
        // Two modes available via ACRO_OPTIONS parameter:
        if (g2.acro_options.get() & uint8_t(AcroOptions::RATE_LOOP_ONLY)) {
            // Rate loop only: pure rate control without attitude feedback
            // Provides most direct response, requires skilled pilot
            attitude_control->input_rate_bf_roll_pitch_yaw_2_rads(target_roll_rads, target_pitch_rads, target_yaw_rads);
        } else {
            // Rate loop with attitude feedback: slight self-leveling tendency
            // Easier to fly, good for learning acro maneuvers
            attitude_control->input_rate_bf_roll_pitch_yaw_rads(target_roll_rads, target_pitch_rads, target_yaw_rads);
        }
    } else {
        // FLYBAR CONTROL PATH (mechanical stabilization):
        // Mechanical flybar provides gyroscopic stabilization - no electronic stabilization needed
        // Pass pilot inputs directly through to servos with minimal processing
        // Zero deadzone gives true 1:1 control feel that flybar pilots expect
        // Flybar system itself provides rate damping and attitude stability
        float roll_in_cds = channel_roll->get_control_in_zero_dz();
        float pitch_in_cds = channel_pitch->get_control_in_zero_dz();
        float yaw_in_cds;
        
        if (motors->supports_yaw_passthrough()) {
            // External tail gyro: pass yaw input directly through
            // Gyro provides its own stabilization (heading hold or rate mode)
            yaw_in_cds = channel_yaw->get_control_in_zero_dz();
        } else {
            // No external gyro: apply ACRO_YAW_P gain to yaw input
            // Provides rate control scaling with normal deadzone
            // Less common configuration for flybar helicopters
            yaw_in_cds = rad_to_cd(get_pilot_desired_yaw_rate_rads());
        }

        // Pass control inputs directly to servos via attitude controller
        // Attitude controller handles servo mixing for swashplate but no stabilization
        attitude_control->passthrough_bf_roll_pitch_rate_yaw_cds(roll_in_cds, pitch_in_cds, yaw_in_cds);
    }

    // COLLECTIVE PITCH CONTROL (traditional helicopter "throttle"):
    // Unlike multirotors where throttle controls motor speed, helicopter throttle stick
    // controls main rotor collective pitch angle:
    // - Center/mid-stick: zero collective (rotor producing minimal thrust)
    // - Above center: positive collective (climbing thrust)
    // - Below center: negative collective (descending thrust, used for inverted flight)
    // Rotor RPM is maintained constant by governor, independent of collective pitch
    pilot_throttle_scaled = copter.input_manager.get_pilot_desired_collective(channel_throttle->get_control_in());

    // Output collective pitch command to swashplate servos
    // angle_boost=false: No compensation for attitude tilt (unlike multirotors)
    // Helicopters maintain constant rotor speed, thrust direction is simply tilted
    // Filtering applied via g.throttle_filt to smooth collective commands
    attitude_control->set_throttle_out(pilot_throttle_scaled, false, g.throttle_filt);
}


/**
 * @brief Simulate mechanical flybar behavior by gradually reducing attitude error
 * 
 * @details Virtual flybar provides self-leveling characteristics similar to a mechanical
 *          flybar for flybarless helicopter systems. A mechanical flybar is a weighted
 *          bar attached to the rotor head that provides gyroscopic stabilization through
 *          mechanical coupling. This function simulates that behavior electronically.
 *          
 *          Purpose: Makes flybarless helicopters easier to fly in acro mode by adding
 *          a gentle self-leveling tendency that "leaks" attitude error back to level.
 *          
 *          Algorithm:
 *          1. Calculate attitude error (difference between target and current attitude)
 *          2. Convert attitude error to earth-frame rate corrections (leak rates)
 *          3. Transform earth-frame corrections to body-frame rates
 *          4. Add body-frame corrections to commanded rates
 *          
 *          Effect: When pilot releases sticks, helicopter gradually returns to level
 *          rather than maintaining current attitude (pure rate mode) or immediately
 *          snapping level (full stabilization). The leak rate parameters control how
 *          quickly this leveling occurs:
 *          - Low leak rate (0.5-1.0): Slow leveling, more rate-like behavior
 *          - Medium leak rate (2.0-3.0): Moderate leveling, balanced feel
 *          - High leak rate (5.0+): Fast leveling, more stabilized behavior
 *          
 *          Typical usage:
 *          - On ground: High leak rate (3.0) for stable ground handling
 *          - In flight: Lower leak rate (g.acro_balance_*) for more aerobatic response
 *          - Advanced pilots: Very low/zero leak rate for pure rate control
 * 
 * @param[in,out] roll_out_rads  Desired roll rate (rad/s) - modified by adding leak rate
 * @param[in,out] pitch_out_rads Desired pitch rate (rad/s) - modified by adding leak rate
 * @param[in,out] yaw_out_rads   Desired yaw rate (rad/s) - currently not modified (no yaw leak)
 * @param[in]     pitch_leak     Pitch leak rate coefficient (0-10, typically 0.5-3.0)
 *                               Higher values = faster return to level pitch
 * @param[in]     roll_leak      Roll leak rate coefficient (0-10, typically 0.5-3.0)
 *                               Higher values = faster return to level roll
 * 
 * @note Called every loop cycle (400Hz) when virtual flybar is active (flybarless mode)
 * @note Yaw is not leaked - yaw rate is maintained as commanded (prevents unwanted heading drift)
 * @note Leak rate acts as proportional gain on attitude error converted to rate correction
 * 
 * @warning Setting leak rates too high can cause oscillation or overly aggressive leveling
 * @warning Setting leak rates to zero disables virtual flybar (pure rate mode - expert only)
 * 
 * Source: ArduCopter/mode_acro_heli.cpp:129-154
 */
void ModeAcro_Heli::virtual_flybar( float &roll_out_rads, float &pitch_out_rads, float &yaw_out_rads, float pitch_leak, float roll_leak)
{
    Vector3f rate_ef_level_rads, rate_bf_level_rads;

    // Get current attitude targets from attitude controller (where we're trying to get to)
    const Vector3f& att_target_rad = attitude_control->get_att_target_euler_rad();

    // STEP 1: Calculate attitude error and convert to earth-frame rate corrections
    // Roll error: difference between target roll and current roll (wrapped to ±π)
    // Multiply by leak coefficient to get correction rate: large error = large correction
    // Negative sign: error correction opposes the error direction
    rate_ef_level_rads.x = -wrap_PI(att_target_rad.x - ahrs.get_roll_rad()) * roll_leak;

    // Pitch error: same algorithm as roll
    rate_ef_level_rads.y = -wrap_PI(att_target_rad.y - ahrs.get_pitch_rad()) * pitch_leak;

    // Yaw: no self-leveling on yaw axis (no "level" yaw reference)
    // Prevents unwanted heading drift - pilot controls yaw rate directly
    rate_ef_level_rads.z = 0;

    // STEP 2: Convert earth-frame rate corrections to body-frame rates
    // Earth-frame rates are relative to horizon, body-frame rates are relative to helicopter
    // Transformation depends on current attitude (quaternion)
    attitude_control->euler_rate_to_ang_vel(attitude_control->get_attitude_target_quat(), rate_ef_level_rads, rate_bf_level_rads);

    // STEP 3: Add self-leveling corrections to pilot-commanded rates
    // Result: pilot commands + automatic leveling = smoother, more forgiving control
    roll_out_rads += rate_bf_level_rads.x;
    pitch_out_rads += rate_bf_level_rads.y;
    yaw_out_rads += rate_bf_level_rads.z;  // Currently zero, but included for completeness

}
#endif  //HELI_FRAME
#endif  //MODE_ACRO_ENABLED
