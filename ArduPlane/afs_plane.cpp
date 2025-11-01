/**
 * @file afs_plane.cpp
 * @brief ArduPlane-specific implementation of the Advanced Failsafe System (AFS)
 * 
 * @details This file implements the AP_AdvancedFailsafe_Plane class, which provides
 *          plane-specific failsafe behaviors for critical system failures and geofence
 *          breaches. The Advanced Failsafe System is designed to comply with Outback
 *          Challenge (OBC) rules and provides enhanced safety mechanisms beyond standard
 *          failsafe behavior.
 *          
 *          Key Responsibilities:
 *          - Vehicle termination with aerodynamic or landing-based approaches
 *          - IO failsafe configuration for safe servo positions
 *          - Flight mode classification for AFS state machine
 *          - Automatic mode transitions on datalink loss
 *          
 *          OBC Compliance:
 *          The termination logic implements OBC-compliant failsafe behavior including:
 *          - Immediate control surface deflection to maximum safe positions
 *          - Throttle management (reverse thrust support or minimum throttle)
 *          - Quadplane-aware termination with VTOL landing capability
 *          - Hardware disarm to cut ignition
 *          
 *          Termination Strategies:
 *          1. TERMINATE_ACTION_LAND: Controlled landing (VTOL for quadplanes, normal for planes)
 *          2. Aerodynamic termination (default): Maximum control deflection to force rapid descent
 *          
 *          Safety-Critical System:
 *          This module controls vehicle termination behavior in emergency situations.
 *          All modifications must be thoroughly tested and reviewed for safety implications.
 *          Incorrect termination behavior could result in uncontrolled flight or crash.
 *          
 * @note Entire module conditionally compiled with AP_ADVANCEDFAILSAFE_ENABLED
 * @warning This is safety-critical code - modifications require extensive testing
 * 
 * @see AP_AdvancedFailsafe (base class in libraries/AP_AdvancedFailsafe/)
 * @see ArduPlane/Plane.h for vehicle integration
 * 
 * Source: ArduPlane/afs_plane.cpp
 */

#include "Plane.h"

#if AP_ADVANCEDFAILSAFE_ENABLED

/**
 * @brief Execute immediate vehicle termination sequence
 * 
 * @details This method implements the emergency vehicle termination sequence required
 *          for Advanced Failsafe compliance. It immediately configures all control
 *          surfaces and motor outputs to predefined termination values designed to
 *          bring the vehicle down in a controlled manner.
 *          
 *          Termination Behavior by Action Type:
 *          
 *          TERMINATE_ACTION_LAND:
 *          - Quadplane: Switches to QLAND mode for vertical landing
 *          - Fixed-wing: Executes controlled landing termination sequence
 *          
 *          Aerodynamic Termination (default):
 *          - Flaps: Set to 100% (maximum deployment, slew limiting removed)
 *          - Ailerons: Set to SERVO_MAX for maximum roll authority
 *          - Rudder: Set to SERVO_MAX for maximum yaw deflection
 *          - Elevator: Set to SERVO_MAX for maximum pitch deflection
 *          - Throttle: Set to TRIM (reverse thrust) or MIN (normal throttle)
 *          
 *          The aerodynamic termination strategy creates maximum drag and disrupts
 *          aerodynamic stability to force a rapid but somewhat controlled descent.
 *          This is the OBC-compliant approach to emergency termination.
 *          
 *          Additional Actions:
 *          - Disables RC passthrough to prevent pilot override
 *          - Calls plane.servos_output() to apply termination values immediately
 *          - Triggers quadplane termination sequence if applicable
 *          - Disarms the vehicle to cut ignition (safety requirement)
 *          
 * @note This is a one-way operation - once called, vehicle is in terminal state
 * @note Called automatically by AFS when termination conditions are met
 * 
 * @warning SAFETY-CRITICAL: This function causes immediate loss of normal flight control
 * @warning Vehicle will descend rapidly after termination - ensure safe termination area
 * @warning Do not call this function unless genuine emergency termination is required
 * @warning Disarm at end of sequence cuts ignition - vehicle cannot be restarted in flight
 * 
 * @see AP_AdvancedFailsafe::check() for termination trigger conditions
 * @see plane.landing.terminate() for fixed-wing landing termination
 * @see plane.mode_qland for quadplane vertical landing
 * 
 * Source: ArduPlane/afs_plane.cpp:12-60
 */
void AP_AdvancedFailsafe_Plane::terminate_vehicle(void)
{
#if HAL_QUADPLANE_ENABLED
    if (plane.quadplane.available() && _terminate_action == TERMINATE_ACTION_LAND) {
        // perform a VTOL landing
        plane.set_mode(plane.mode_qland, ModeReason::FENCE_BREACHED);
        return;
    }
#endif

    plane.g2.servo_channels.disable_passthrough(true);
    
    if (_terminate_action == TERMINATE_ACTION_LAND) {
        plane.landing.terminate();
    } else {
        // remove flap slew limiting
        SRV_Channels::set_slew_rate(SRV_Channel::k_flap_auto, 0.0, 100, plane.G_Dt);
        SRV_Channels::set_slew_rate(SRV_Channel::k_flap, 0.0, 100, plane.G_Dt);

        // aerodynamic termination is the default approach to termination
        SRV_Channels::set_output_scaled(SRV_Channel::k_flap_auto, 100.0);
        SRV_Channels::set_output_scaled(SRV_Channel::k_flap, 100.0);
        SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, SERVO_MAX);
        SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, SERVO_MAX);
        SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, SERVO_MAX);
        if (plane.have_reverse_thrust()) {
            // configured for reverse thrust, use TRIM
            SRV_Channels::set_output_limit(SRV_Channel::k_throttle, SRV_Channel::Limit::TRIM);
            SRV_Channels::set_output_limit(SRV_Channel::k_throttleLeft, SRV_Channel::Limit::TRIM);
            SRV_Channels::set_output_limit(SRV_Channel::k_throttleRight, SRV_Channel::Limit::TRIM);
        } else {
            // use MIN
            SRV_Channels::set_output_limit(SRV_Channel::k_throttle, SRV_Channel::Limit::MIN);
            SRV_Channels::set_output_limit(SRV_Channel::k_throttleLeft, SRV_Channel::Limit::MIN);
            SRV_Channels::set_output_limit(SRV_Channel::k_throttleRight, SRV_Channel::Limit::MIN);
        }
        SRV_Channels::set_output_limit(SRV_Channel::k_manual, SRV_Channel::Limit::TRIM);
        SRV_Channels::set_output_limit(SRV_Channel::k_none, SRV_Channel::Limit::TRIM);
    }

    plane.servos_output();

#if HAL_QUADPLANE_ENABLED
    plane.quadplane.afs_terminate();
#endif

    // also disarm to ensure that ignition is cut
    plane.arming.disarm(AP_Arming::Method::AFS);
}

/**
 * @brief Configure IO board failsafe limits for all servo channels
 * 
 * @details This method configures the hardware-level failsafe positions for all servo
 *          channels used by the plane. These failsafe limits are applied by the IO
 *          processor if communication with the main processor is lost, providing a
 *          hardware-level safety backup independent of software execution.
 *          
 *          The configured failsafe positions are similar to the termination values
 *          but are stored in the IO board's failsafe configuration. This ensures that
 *          even if the main CPU fails or the connection to the IO board is severed,
 *          the aircraft will enter a safe configuration.
 *          
 *          Configured Failsafe Limits:
 *          - Flaps (auto and manual): MAX (full deployment for drag)
 *          - Ailerons: MIN (one-sided deflection)
 *          - Rudder: MAX (maximum yaw deflection)
 *          - Elevator: MAX (maximum pitch deflection)
 *          - Throttle: TRIM (reverse thrust aircraft) or MIN (normal aircraft)
 *          - Manual/None channels: TRIM (neutral position)
 *          
 *          Quadplane Considerations:
 *          For quadplanes, this method also configures motor failsafe values to the
 *          minimum PWM output, ensuring motors stop spinning if IO board failsafe
 *          is triggered.
 *          
 *          Hardware Failsafe vs. Software Failsafe:
 *          This is distinct from software failsafe behavior - these values are burned
 *          into the IO processor and activate only on IO board communication loss.
 *          Software-level failsafes are handled elsewhere in the AFS system.
 *          
 * @note Called during AFS initialization to prepare hardware failsafe configuration
 * @note Failsafe limits stored in IO board persist across reboots until reconfigured
 * @note Different from software termination - this is hardware-level protection
 * 
 * @warning Hardware failsafe is last-resort protection for processor/communication failure
 * @warning Configured positions will be applied immediately if IO link is lost
 * @warning Test hardware failsafe only in safe conditions (e.g., simulation, on ground)
 * 
 * @see SRV_Channels::set_failsafe_limit() for servo failsafe configuration API
 * @see AP_Motors::get_pwm_output_min() for quadplane motor failsafe values
 * 
 * Source: ArduPlane/afs_plane.cpp:62-91
 */
void AP_AdvancedFailsafe_Plane::setup_IO_failsafe(void)
{
    // all aux channels
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_flap_auto, SRV_Channel::Limit::MAX);
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_flap, SRV_Channel::Limit::MAX);
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_aileron, SRV_Channel::Limit::MIN);
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_rudder, SRV_Channel::Limit::MAX);
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_elevator, SRV_Channel::Limit::MAX);
    if (plane.have_reverse_thrust()) {
        // configured for reverse thrust, use TRIM
        SRV_Channels::set_failsafe_limit(SRV_Channel::k_throttle, SRV_Channel::Limit::TRIM);
        SRV_Channels::set_failsafe_limit(SRV_Channel::k_throttleLeft, SRV_Channel::Limit::TRIM);
        SRV_Channels::set_failsafe_limit(SRV_Channel::k_throttleRight, SRV_Channel::Limit::TRIM);
    } else {
        // normal throttle, use MIN
        SRV_Channels::set_failsafe_limit(SRV_Channel::k_throttle, SRV_Channel::Limit::MIN);
        SRV_Channels::set_failsafe_limit(SRV_Channel::k_throttleLeft, SRV_Channel::Limit::MIN);
        SRV_Channels::set_failsafe_limit(SRV_Channel::k_throttleRight, SRV_Channel::Limit::MIN);
    }
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_manual, SRV_Channel::Limit::TRIM);
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_none, SRV_Channel::Limit::TRIM);

#if HAL_QUADPLANE_ENABLED
    if (plane.quadplane.available()) {
        // setup AP_Motors outputs for failsafe
        uint32_t mask = plane.quadplane.motors->get_motor_mask();
        hal.rcout->set_failsafe_pwm(mask, plane.quadplane.motors->get_pwm_output_min());
    }
#endif
}

/**
 * @brief Classify current flight mode into AFS control mode category
 * 
 * @details This method maps the plane's current flight mode into one of the AFS
 *          control mode categories used by the Advanced Failsafe state machine.
 *          The AFS system uses these categories to determine appropriate failsafe
 *          behavior based on the level of autonomy in the current flight mode.
 *          
 *          AFS Control Mode Classification:
 *          
 *          AFS_AUTO:
 *          - Any mode with automatic throttle control (does_auto_throttle() == true)
 *          - Examples: AUTO, GUIDED, RTL, LOITER, CRUISE, FBWB with auto throttle
 *          - Indicates vehicle has autonomous navigation capability
 *          
 *          AFS_MANUAL:
 *          - Manual mode only (direct pilot control with no stabilization)
 *          - No computer assistance beyond RC mixing
 *          - Pilot has full responsibility for stability
 *          
 *          AFS_STABILIZED:
 *          - All other modes with computer-assisted stabilization
 *          - Examples: STABILIZE, FBWA, ACRO, TRAINING, etc.
 *          - Computer assists with stability but pilot controls navigation
 *          
 *          The AFS system uses these categories to determine:
 *          - Whether autonomous failsafe actions are appropriate
 *          - What level of intervention is needed for datalink loss
 *          - Whether mode changes should be forced during failsafe
 *          
 * @return AP_AdvancedFailsafe::control_mode Current AFS control mode category
 *         - AFS_AUTO: Autonomous navigation active
 *         - AFS_MANUAL: Direct manual control (no stabilization)
 *         - AFS_STABILIZED: Computer-stabilized manual control
 * 
 * @note Called periodically by AFS state machine to monitor current mode
 * @note Mode classification affects AFS failsafe behavior and state transitions
 * 
 * @see AP_AdvancedFailsafe::control_mode enum definition
 * @see Mode::does_auto_throttle() for auto mode detection
 * 
 * Source: ArduPlane/afs_plane.cpp:96-105
 */
AP_AdvancedFailsafe::control_mode AP_AdvancedFailsafe_Plane::afs_mode(void)
{
    if (plane.control_mode->does_auto_throttle()) {
        return AP_AdvancedFailsafe::AFS_AUTO;
    }
    if (plane.control_mode == &plane.mode_manual) {
        return AP_AdvancedFailsafe::AFS_MANUAL;
    }
    return AP_AdvancedFailsafe::AFS_STABILIZED;
}

/**
 * @brief Force vehicle into AUTO mode in response to datalink loss
 * 
 * @details This method forces the plane into AUTO mode when the Advanced Failsafe
 *          system determines that a datalink loss has occurred and autonomous flight
 *          is required. This is a key component of the AFS datalink loss response
 *          strategy.
 *          
 *          Datalink Loss Response Strategy:
 *          When datalink is lost for longer than the AFS-configured timeout, the
 *          AFS system may force the vehicle into AUTO mode to:
 *          - Continue mission execution autonomously
 *          - Enable autonomous navigation to safe landing area
 *          - Maintain aircraft control without ground station input
 *          - Execute pre-programmed recovery procedures
 *          
 *          The mode change is logged with ModeReason::GCS_FAILSAFE to indicate
 *          that this was an automated failsafe action rather than a commanded
 *          mode change.
 *          
 *          Preconditions:
 *          - A valid AUTO mission must be loaded for this to be effective
 *          - Vehicle must have valid position estimate (GPS fix)
 *          - AUTO mode must be available and not disabled
 *          
 *          OBC Compliance:
 *          This behavior supports OBC requirements for autonomous operation
 *          continuation when ground station communication is lost, enabling the
 *          vehicle to complete its mission or navigate to a safe recovery area
 *          without ground operator intervention.
 *          
 * @note Called automatically by AFS state machine when datalink loss timeout expires
 * @note Mode change is permanent until pilot/GCS changes mode or AFS is reset
 * @note If AUTO mode cannot be entered, standard failsafe behavior applies
 * 
 * @warning Requires valid mission loaded - behavior undefined if mission is empty
 * @warning Vehicle must have GPS fix - mode change may fail without valid position
 * 
 * @see plane.set_mode() for mode change implementation
 * @see ModeReason::GCS_FAILSAFE for failsafe logging
 * @see Mode_Auto for autonomous mission execution
 * 
 * Source: ArduPlane/afs_plane.cpp:108-111
 */
void AP_AdvancedFailsafe_Plane::set_mode_auto(void)
{
    plane.set_mode(plane.mode_auto,ModeReason::GCS_FAILSAFE);
}

#endif // AP_ADVANCEDFAILSAFE_ENABLED
