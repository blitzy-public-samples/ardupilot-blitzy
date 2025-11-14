/**
 * @file mode_stabilize_heli.cpp
 * @brief Stabilize flight mode implementation for traditional helicopters
 * 
 * @details This file implements the Stabilize flight mode specifically adapted for
 *          traditional helicopters (HELI_FRAME configuration). Unlike multicopter
 *          stabilize mode which controls motor thrust, this variant manages:
 *          
 *          - **Collective Pitch Control**: Throttle input controls collective pitch
 *            angle of main rotor blades rather than motor speed, providing vertical
 *            thrust control through blade angle of attack changes
 *          
 *          - **Swashplate Mixing**: Cyclic (roll/pitch) and collective commands are
 *            mixed through the swashplate mechanism (via AC_AttitudeControl_Heli)
 *            to control individual blade pitch throughout the rotor rotation cycle
 *          
 *          - **Tail Rotor Heading Hold**: Yaw rate commands drive tail rotor collective
 *            to counteract main rotor torque and maintain heading, with special handling
 *            for autorotation scenarios where tail rotor authority is limited
 *          
 *          - **Rotor Speed Governor**: Motor interlock and spool state management
 *            coordinates with external governor (ESC or separate controller) to
 *            maintain constant head speed independent of collective pitch changes
 *          
 *          - **Autorotation Compatibility**: Unlike multicopters, does not reset
 *            attitude targets when motors are not spooled up while flying, allowing
 *            continued attitude control during autorotation descent
 *          
 *          The stabilize mode provides rate-stabilized control where pilot stick inputs
 *          command lean angles (roll/pitch) and yaw rate, while the attitude controller
 *          maintains those targets using helicopter-specific control mixing.
 * 
 * @see AC_AttitudeControl_Heli - Helicopter-specific attitude control with swashplate mixing
 * @see AP_MotorsHeli - Traditional helicopter motor/servo control
 * @see Copter::InputManagerHeli - Helicopter collective pitch scaling
 * 
 * @note This file is only compiled when FRAME_CONFIG == HELI_FRAME
 * @warning Helicopter control differs fundamentally from multicopter control; parameters
 *          and tuning are not interchangeable between frame types
 */

#include "Copter.h"

#if FRAME_CONFIG == HELI_FRAME

/**
 * @brief Initialize stabilize flight mode for traditional helicopters
 * 
 * @details Configures the helicopter control system for stabilize mode operation.
 *          This initialization is called when the pilot switches into stabilize mode,
 *          NOT at vehicle startup. Key initialization:
 *          
 *          - **Collective Pitch Scaling**: Enables stabilize-specific collective pitch
 *            scaling (stab_col mode) which uses a restricted collective range optimized
 *            for manual stabilized flight, typically centered around hover collective
 *            with symmetric up/down authority
 *          
 *          Unlike multicopter stabilize initialization, this does NOT:
 *          - Reset motor outputs (servos maintain position for pre-flight checks)
 *          - Initialize throttle curves (governor manages rotor speed independently)
 *          - Set motor mixing (swashplate mixing is always active in helicopter)
 *          
 *          The lightweight initialization reflects that traditional helicopters maintain
 *          servo positions and rotor speed management across mode transitions, with only
 *          collective scaling and control response characteristics changing between modes.
 * 
 * @param[in] ignore_checks If true, skip pre-arm and safety checks (used for forced
 *                          mode changes during failsafe or emergency situations)
 * 
 * @return true Always returns true for stabilize mode as it has no prerequisites
 *              (stabilize is the fallback mode and must always be enterable)
 * 
 * @note This function is called during mode transitions, not at vehicle boot
 * @note Stabilize mode is the default safe mode for helicopters and must succeed
 * @warning Do not add complex initialization logic here as it runs during mode changes
 *          which may occur during time-critical flight situations
 * 
 * @see Copter::InputManagerHeli::set_use_stab_col() - Collective pitch scaling
 * @see ModeStabilize_Heli::run() - Main stabilize mode execution
 */
bool ModeStabilize_Heli::init(bool ignore_checks)
{
    // be aware that when adding code to this function that it is *NOT
    // RUN* at vehicle startup!

    // Set stabilize collective mode to use stabilize-specific collective pitch scaling.
    // This configures the input manager to scale throttle stick to collective pitch
    // using the stabilize collective range (typically ±500 from hover collective),
    // providing intuitive collective control centered around hover power.
    copter.input_manager.set_use_stab_col(true);

    return true;
}

/**
 * @brief Execute stabilize flight mode control loop for traditional helicopters
 * 
 * @details Runs the main stabilize controller implementing rate-stabilized attitude control
 *          adapted for traditional helicopter control mechanics. This function processes
 *          pilot inputs and commands the helicopter attitude control system using
 *          helicopter-specific control allocation.
 *          
 *          **Control Flow**:
 *          1. Apply SIMPLE mode coordinate transformation to pilot inputs if enabled
 *          2. Convert pilot cyclic stick inputs to target lean angles (roll/pitch)
 *          3. Convert pilot pedal input to target yaw rate
 *          4. Scale pilot collective stick input to collective pitch angle
 *          5. Manage rotor spool state and motor interlock based on arm status
 *          6. Handle attitude controller state based on spool conditions
 *          7. Command attitude controller with helicopter-specific mixing
 *          8. Output collective pitch command to swashplate
 *          
 *          **Helicopter-Specific Control Characteristics**:
 *          
 *          - **Swashplate Mixing**: Target roll/pitch/collective are mixed through
 *            AC_AttitudeControl_Heli which implements CCPM (Cyclic/Collective Pitch Mixing)
 *            or mechanical mixing to translate control commands into individual servo
 *            positions for the swashplate mechanism
 *          
 *          - **Collective Pitch Control**: Unlike multicopter throttle which directly
 *            controls motor speed, collective pitch changes the angle of attack of all
 *            main rotor blades simultaneously while rotor speed is maintained constant
 *            by a separate governor (typically in the ESC)
 *          
 *          - **Tail Rotor Heading Hold**: Yaw rate commands control tail rotor collective
 *            pitch to generate anti-torque thrust. The tail rotor counteracts main rotor
 *            torque reaction and provides directional control
 *          
 *          - **Autorotation Handling**: Servos continue to move even when motors are not
 *            at full speed (disarmed or autorotation) to allow realistic servo movement
 *            during ground checks and maintain attitude control authority during engine-off
 *            descent. This differs from multicopters which zero outputs when disarmed
 *          
 *          - **Rotor Speed Governor**: Motor interlock enables the external governor to
 *            spool up and maintain rotor speed. Spool state progresses through GROUND_IDLE
 *            to THROTTLE_UNLIMITED only after motor interlock is enabled, coordinating
 *            with the governor's speed control loop
 *          
 *          - **Integrator Management**: Rate controller integrators are reset or smoothed
 *            based on spool state to prevent integrator windup during ground operations
 *            or spoolup, with special handling for leaky integrator configurations that
 *            allow continued integration during autorotation practice
 *          
 *          **Spool State Handling**:
 *          - SHUT_DOWN: Motors stopped, reset yaw and all integrators
 *          - GROUND_IDLE: On ground or autorotation practice, reset yaw and smooth integrators
 *          - SPOOLING_UP/DOWN: Transient states, maintain current targets
 *          - THROTTLE_UNLIMITED: Full flight authority, smooth integrators if landed
 * 
 * @return void
 * 
 * @note Must be called at 100Hz or higher for stable control (typically 400Hz main loop)
 * @note Cyclic inputs are scaled by lean_angle_max parameter (typically 45 degrees)
 * @note Collective scaling uses stabilize-specific range set during init()
 * 
 * @warning Servo outputs remain active when disarmed for ground checks; ensure rotor
 *          is not installed or area is clear during bench testing
 * @warning Autorotation mode allows attitude control without motor power; do not assume
 *          motors stopped means servos are inactive
 * @warning Motor interlock must be enabled separately from arming for rotor spoolup;
 *          arming alone will not start the rotor
 * 
 * @see AC_AttitudeControl_Heli::input_euler_angle_roll_pitch_euler_rate_yaw_rad() - Helicopter attitude control
 * @see AC_AttitudeControl_Heli::set_throttle_out() - Collective pitch output with swashplate mixing
 * @see AP_MotorsHeli::set_desired_spool_state() - Rotor speed state management
 * @see Copter::InputManagerHeli::get_pilot_desired_collective() - Collective pitch scaling
 */
void ModeStabilize_Heli::run()
{
    float target_roll_rad, target_pitch_rad;
    float pilot_throttle_scaled;

    // Apply SIMPLE mode coordinate transformation to pilot inputs if enabled.
    // SIMPLE mode rotates pilot stick inputs to be relative to initial takeoff heading
    // rather than current vehicle heading, simplifying orientation for novice pilots.
    update_simple_mode();

    // Convert pilot cyclic stick inputs to target lean angles (roll and pitch).
    // These represent the desired attitude of the helicopter's fuselage in radians.
    // Maximum lean angle is limited by attitude_control->lean_angle_max_rad() parameter
    // (typically 45 degrees = 0.785 rad) to prevent excessive attitudes.
    get_pilot_desired_lean_angles_rad(target_roll_rad, target_pitch_rad, attitude_control->lean_angle_max_rad(), attitude_control->lean_angle_max_rad());

    // Get pilot's desired yaw rate from pedal input in radians per second.
    // This commands the rate of heading change, which is achieved by varying tail rotor
    // collective pitch to generate more or less anti-torque thrust. Positive yaw rate
    // corresponds to nose-right rotation (clockwise viewed from above).
    float target_yaw_rate_rads = get_pilot_desired_yaw_rate_rads();

    // Get pilot's desired collective pitch from throttle stick input.
    // The input_manager scales the raw throttle channel value to collective pitch using
    // the stabilize collective range (H_COL_MIN/MAX and H_COL_MID parameters), typically
    // providing symmetric collective authority above and below hover collective.
    // This differs from multicopter throttle which directly controls motor thrust.
    pilot_throttle_scaled = copter.input_manager.get_pilot_desired_collective(channel_throttle->get_control_in());

    // **CRITICAL HELICOPTER BEHAVIOR**: Traditional helicopters maintain servo control authority
    // even when motors are not spooled up, for two important reasons:
    //
    // 1. **Autorotation Flight**: During engine-off autorotation descent (emergency procedure),
    //    the pilot must maintain full cyclic (roll/pitch) and pedal (yaw) authority to control
    //    the unpowered descent and landing. Resetting attitude targets would be catastrophic.
    //
    // 2. **Ground Operational Checks**: Servos must move realistically while disarmed to allow
    //    pilots to verify swashplate movement, servo direction, and control throw during
    //    pre-flight inspections without spinning the rotor.
    //
    // Unlike multicopters which zero all outputs when disarmed, helicopters keep swashplate
    // servos and tail rotor servo active, moving in response to stick inputs. Only motor output
    // (rotor speed) is stopped when disarmed. This allows collective pitch to remain non-zero,
    // which is essential for realistic servo movement and autorotation capability.

    // Command desired rotor spool state based on arm status.
    // Helicopter spool state management coordinates with external governor (typically in ESC)
    // to control main rotor speed through motor throttle output.
    if (!motors->armed()) {
        // Vehicle is disarmed: Command rotor shutdown (motor stopped, zero throttle to ESC).
        // Swashplate servos remain active for ground checks, but rotor will not spin.
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
    } else {
        // Vehicle is armed: Request full rotor speed authority (THROTTLE_UNLIMITED).
        // IMPORTANT: Actual progression to THROTTLE_UNLIMITED requires motor interlock to be
        // enabled separately. Motor interlock acts as a safety gate that must be explicitly
        // activated (typically via RC switch or auto-takeoff sequence) before the governor
        // will spool up the rotor. This prevents accidental rotor starts from arming alone.
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    }

    // Handle attitude controller state based on current rotor spool state.
    // Different spool states require different integrator management to prevent windup
    // and ensure smooth transitions between ground and flight operations.
    switch (motors->get_spool_state()) {
    case AP_Motors::SpoolState::SHUT_DOWN:
        // Rotor is stopped (motor ESC at zero throttle, not spinning).
        // Reset yaw target to current heading to prevent sudden yaw movement on spoolup.
        // Clear all rate controller integrators to zero (hard reset) since accumulated
        // integral terms are not valid when rotor is stopped.
        attitude_control->reset_yaw_target_and_rate(false);
        attitude_control->reset_rate_controller_I_terms();
        break;
        
    case AP_Motors::SpoolState::GROUND_IDLE:
        // Rotor at low speed (governor maintaining idle RPM but not flight speed).
        // This state occurs during initial spoolup or during practice autorotation at ground idle.
        //
        // Reset integrators only if BOTH conditions indicate we're on the ground:
        // 1. During initial arming sequence with leaky integrator enabled, OR
        // 2. Detected as landed (land_complete) without leaky integrator
        //
        // The leaky integrator option allows integrators to decay naturally during
        // autorotation practice rather than hard reset, maintaining some control history.
        if ((motors->init_targets_on_arming() && motors->using_leaky_integrator()) || (copter.ap.land_complete && !motors->using_leaky_integrator())) {
            attitude_control->reset_yaw_target_and_rate(false);
            attitude_control->reset_rate_controller_I_terms_smoothly();
        }
        break;
        
    case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
        // Rotor at full governed speed with unlimited control authority.
        // This is normal flight state where governor maintains constant rotor RPM and
        // collective pitch provides vertical control.
        //
        // If detected as landed without leaky integrator, smoothly decay integrators
        // to prevent large integral buildup while sitting on ground at flight RPM.
        // This prevents sudden control jumps on takeoff.
        if (copter.ap.land_complete && !motors->using_leaky_integrator()) {
            attitude_control->reset_rate_controller_I_terms_smoothly();
        }
        break;
        
    case AP_Motors::SpoolState::SPOOLING_UP:
    case AP_Motors::SpoolState::SPOOLING_DOWN:
        // Transient states: rotor RPM is actively changing under governor control.
        // Maintain current attitude targets and integrator values during transition.
        // These states typically last 3-5 seconds depending on rotor inertia and governor
        // acceleration/deceleration ramps. Control authority increases/decreases with RPM.
        break;
    }

    // Command the helicopter attitude controller with target roll angle, pitch angle, and yaw rate.
    // AC_AttitudeControl_Heli (helicopter-specific variant) converts these body-frame attitude
    // targets into rate commands, runs PID rate controllers, and performs swashplate mixing to
    // generate individual servo commands for the CCPM (Cyclic/Collective Pitch Mixing) system.
    //
    // The attitude controller output is:
    // - Cyclic pitch servo positions (fore/aft tilt of swashplate for pitch control)
    // - Cyclic roll servo positions (left/right tilt of swashplate for roll control)  
    // - Tail rotor collective (tail rotor blade pitch for yaw/anti-torque control)
    //
    // The mixing accounts for swashplate geometry (120° or 140° spacing), servo reversing,
    // mechanical advantage, and collective-to-cyclic coupling compensation (CCPM).
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_rad(target_roll_rad, target_pitch_rad, target_yaw_rate_rads);

    // Output collective pitch command to the attitude controller.
    // For helicopters, this scaled collective value (0.0 to 1.0 range) is converted to
    // collective pitch angle and mixed into the swashplate servo positions along with
    // the cyclic commands above. The second parameter (true) enables angle boost which
    // automatically increases collective as cyclic increases to maintain altitude during
    // aggressive maneuvering. The g.throttle_filt parameter applies filtering to smooth
    // collective stick inputs and reduce abrupt rotor thrust changes.
    //
    // Note: Despite the function name "throttle_out", for helicopters this controls blade
    // pitch angle, not motor speed. Motor speed is independently maintained by the governor.
    attitude_control->set_throttle_out(pilot_throttle_scaled, true, g.throttle_filt);
}

#endif  //HELI_FRAME
