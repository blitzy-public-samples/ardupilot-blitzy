/**
 * @file radio.cpp
 * @brief RC (Radio Control) input processing and failsafe management for ArduPlane
 * 
 * @details This file implements the core RC input handling for fixed-wing aircraft,
 *          including:
 *          - Reading and processing RC channel inputs from the receiver
 *          - Mapping RC channels to flight control functions (roll, pitch, yaw, throttle)
 *          - Applying expo curves for smoother stick response in manual modes
 *          - Managing RC failsafe detection and response
 *          - Trim learning and servo trim configuration
 *          - Throttle nudge functionality for in-flight speed adjustments
 *          
 *          The RC input system is safety-critical and includes multiple layers of
 *          failsafe protection to ensure safe behavior when RC signal is lost.
 *          
 *          Key Features:
 *          - Multi-mode failsafe detection (signal loss, throttle threshold)
 *          - Rudder-only mode support for specialized control configurations
 *          - Reverse thrust support for specialized ESC configurations
 *          - Integration with quadplane VTOL control (when enabled)
 *          - Real-time trim adjustment and learning
 *          
 * @note This is called at main loop rate (typically 50Hz for fixed-wing)
 * @warning RC failsafe behavior is flight-critical; modifications require thorough testing
 * 
 * @see RC_Channel for individual channel processing
 * @see SRV_Channels for servo output management
 * 
 * Source: ArduPlane/radio.cpp
 */

#include "Plane.h"

//Function that will read the radio data, limit servos and trigger a failsafe
// ----------------------------------------------------------------------------

/**
 * @brief Map RC input channels to flight control functions and configure channel ranges
 * 
 * @details This function establishes the mapping between physical RC channels
 *          (as received from the transmitter) and logical control functions
 *          (roll, pitch, yaw, throttle). It handles special configurations including:
 *          
 *          - Rudder-only mode: Uses yaw channel for both roll and rudder control
 *          - Reverse thrust: Configures throttle range for bidirectional motor control
 *          - Auxiliary functions: Maps flap and airbrake channels
 *          - Quadplane integration: Sets up forward throttle channel for VTOL transitions
 *          - ESC scaling: Configures proper PWM scaling for various ESC types (UAVCAN, etc.)
 *          
 *          Channel Range Configuration:
 *          - Roll/Pitch/Rudder: Set to SERVO_MAX angle range (±4500 centidegrees = ±45°)
 *          - Throttle: Set to 0-100% range (or ±100 for reverse thrust)
 *          
 *          This function is called at startup and whenever channel configuration changes
 *          at runtime (e.g., through parameter updates).
 * 
 * @note The RC library guarantees that channel pointers are non-null
 * @note Dead zones are configured separately in init_rc_in()
 * 
 * @warning Incorrect throttle configuration can cause motors to start unexpectedly
 * 
 * @see init_rc_in() for dead zone configuration
 * @see RC_Channel::set_angle() for channel range configuration
 * @see SRV_Channels::set_esc_scaling_for() for ESC output scaling
 * 
 * Source: ArduPlane/radio.cpp:9-65
 */
void Plane::set_control_channels(void)
{
    // the library guarantees that these are non-nullptr:
    if (g.rudder_only) {
        // in rudder only mode the roll and rudder channels are the
        // same.
        channel_roll = &rc().get_yaw_channel();
    } else {
        channel_roll = &rc().get_roll_channel();
    }
    channel_pitch    = &rc().get_pitch_channel();
    channel_throttle = &rc().get_throttle_channel();
    channel_rudder   = &rc().get_yaw_channel();

    // set rc channel ranges
    channel_roll->set_angle(SERVO_MAX);
    channel_pitch->set_angle(SERVO_MAX);
    channel_rudder->set_angle(SERVO_MAX);
    if (!have_reverse_thrust()) {
        // normal operation
        channel_throttle->set_range(100);
    } else {
        // reverse thrust
        if (have_reverse_throttle_rc_option) {
            // when we have a reverse throttle RC option setup we use throttle
            // as a range, and rely on the RC switch to get reverse thrust
            channel_throttle->set_range(100);
        } else {
            channel_throttle->set_angle(100);
        }
        SRV_Channels::set_angle(SRV_Channel::k_throttle, 100);
        SRV_Channels::set_angle(SRV_Channel::k_throttleLeft, 100);
        SRV_Channels::set_angle(SRV_Channel::k_throttleRight, 100);
    }

    // update flap and airbrake channel assignment
    channel_flap     = rc().find_channel_for_option(RC_Channel::AUX_FUNC::FLAP);
    channel_airbrake = rc().find_channel_for_option(RC_Channel::AUX_FUNC::AIRBRAKE);

#if HAL_QUADPLANE_ENABLED
    // update manual forward throttle channel assignment
    quadplane.rc_fwd_thr_ch = rc().find_channel_for_option(RC_Channel::AUX_FUNC::FWD_THR);
#endif

    bool set_throttle_esc_scaling = true;
#if HAL_QUADPLANE_ENABLED
    set_throttle_esc_scaling = !quadplane.enable;
#endif
    if (set_throttle_esc_scaling) {
        // setup correct scaling for ESCs like the UAVCAN ESCs which
        // take a proportion of speed. For quadplanes we use AP_Motors
        // scaling
        g2.servo_channels.set_esc_scaling_for(SRV_Channel::k_throttleLeft);
        g2.servo_channels.set_esc_scaling_for(SRV_Channel::k_throttleRight);
        g2.servo_channels.set_esc_scaling_for(SRV_Channel::k_throttle);
    }
}

/**
 * @brief Initialize RC input channel dead zones
 * 
 * @details Sets the default dead zone values for primary flight control channels.
 *          Dead zones prevent small electrical noise or transmitter centering
 *          imperfections from causing unwanted control inputs when sticks are
 *          at neutral position.
 *          
 *          Dead Zone Configuration:
 *          - Roll: 30 (±0.66% of range)
 *          - Pitch: 30 (±0.66% of range)
 *          - Rudder: 30 (±0.66% of range)
 *          - Throttle: 30 (±0.66% of range)
 *          
 *          The dead zone value of 30 is in PWM units, creating a ±30μs
 *          dead zone around the trim/center point. Inputs within this range
 *          are treated as zero.
 *          
 *          This function is called once during system initialization after
 *          set_control_channels() has mapped the channel pointers.
 * 
 * @note Channel range mapping is set separately in set_control_channels()
 * @note Dead zone only applies to control_in values, not raw PWM
 * 
 * @see set_control_channels() for channel mapping and range configuration
 * @see RC_Channel::set_default_dead_zone() for dead zone implementation
 * 
 * Source: ArduPlane/radio.cpp:70-77
 */
void Plane::init_rc_in()
{
    // set rc dead zones
    channel_roll->set_default_dead_zone(30);
    channel_pitch->set_default_dead_zone(30);
    channel_rudder->set_default_dead_zone(30);
    channel_throttle->set_default_dead_zone(30);
}

/**
 * @brief Initialize RC output for main flight control channels
 * 
 * @details Configures output trim and failsafe values for primary flight control
 *          servos. This function is called early in the initialization sequence
 *          to allow servo control before full system initialization, supporting
 *          BRD_SAFETY_DEFLT=0 (safety switch disabled) configurations.
 *          
 *          Throttle Trim Configuration:
 *          For normal (non-reversed) throttle, sets trim to minimum to prevent
 *          motors from starting unexpectedly on power-up. This is a critical
 *          safety feature that protects against CH3_TRIM being incorrectly set
 *          to a high value.
 *          
 *          Failsafe Limit Configuration:
 *          Sets all primary control channels to output their trim values when
 *          RC failsafe or servo failsafe occurs:
 *          - Aileron: Trim (wings level)
 *          - Elevator: Trim (level pitch)
 *          - Throttle: Trim (idle)
 *          - Rudder: Trim (no yaw)
 *          
 *          This ensures the aircraft maintains a safe configuration if the
 *          flight controller loses power or communication with servos.
 *          
 *          Early Initialization:
 *          Called before full system initialization to enable early servo
 *          control for ground testing and preflight checks without requiring
 *          safety switch engagement.
 * 
 * @note Throttle trim is NOT set to minimum for reverse thrust configurations
 * @note Failsafe limits apply to both RC failsafe and servo power loss
 * 
 * @warning Incorrect throttle trim can cause motors to start on power-up
 * 
 * @see init_rc_out_aux() for auxiliary channel initialization
 * @see SRV_Channels::set_trim_to_min_for() for trim configuration
 * @see SRV_Channels::set_failsafe_limit() for failsafe behavior
 * 
 * Source: ArduPlane/radio.cpp:83-103
 */
void Plane::init_rc_out_main()
{
    /*
      change throttle trim to minimum throttle. This prevents a
      configuration error where the user sets CH3_TRIM incorrectly and
      the motor may start on power up
     */
    if (!have_reverse_thrust()) {
        SRV_Channels::set_trim_to_min_for(SRV_Channel::k_throttle);
        SRV_Channels::set_trim_to_min_for(SRV_Channel::k_throttleLeft);
        SRV_Channels::set_trim_to_min_for(SRV_Channel::k_throttleRight);
    }

    SRV_Channels::set_failsafe_limit(SRV_Channel::k_aileron, SRV_Channel::Limit::TRIM);
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_elevator, SRV_Channel::Limit::TRIM);
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_throttle, SRV_Channel::Limit::TRIM);
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_throttleLeft, SRV_Channel::Limit::TRIM);
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_throttleRight, SRV_Channel::Limit::TRIM);
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_rudder, SRV_Channel::Limit::TRIM);

}

/**
 * @brief Initialize RC output for auxiliary channels
 * 
 * @details Enables and configures auxiliary servo channels (beyond the main
 *          flight controls) and sets up their failsafe behavior. This includes
 *          channels for flaps, landing gear, camera gimbals, and other accessories.
 *          
 *          Initialization Sequence:
 *          1. Enable all configured auxiliary servo outputs
 *          2. Call servos_output() to update all servo positions
 *          3. Set up failsafe trim values for all non-motor channels
 *          
 *          Failsafe Configuration:
 *          Non-motor channels are set to output their trim values if the FMU
 *          (flight management unit) firmware crashes or loses power. This is
 *          particularly important for VTOL motors which should shut off rather
 *          than continue at their last commanded value.
 *          
 *          Motor vs Non-Motor Distinction:
 *          - Motor channels: Failsafe to minimum (motors off)
 *          - Servo channels: Failsafe to trim (safe position)
 *          - Camera/Gimbal: Failsafe to trim (neutral position)
 *          
 *          This function is called after init_rc_out_main() during the later
 *          stages of initialization when the full servo library is ready.
 * 
 * @note Called after main channel initialization
 * @note Handles both traditional servos and smart servos (CAN, serial)
 * 
 * @see init_rc_out_main() for main channel initialization
 * @see SRV_Channels::setup_failsafe_trim_all_non_motors() for failsafe setup
 * @see servos_output() for servo position updates
 * 
 * Source: ArduPlane/radio.cpp:108-117
 */
void Plane::init_rc_out_aux()
{
    AP::srv().enable_aux_servos();

    servos_output();
    
    // setup PWM values to send if the FMU firmware dies
    // allows any VTOL motors to shut off
    SRV_Channels::setup_failsafe_trim_all_non_motors();
}

/**
 * @brief Read RC input channels and process pilot commands
 * 
 * @details This is the main RC input processing function called at main loop rate
 *          (typically 50Hz for fixed-wing aircraft). It performs the following operations:
 *          
 *          1. Reads raw RC input from receiver via RC library
 *          2. Updates failsafe timing counters with valid input timestamps
 *          3. Triggers failsafe response if RC signal is lost
 *          4. Calculates throttle/airspeed nudge values for in-flight adjustments
 *          5. Handles tailsitter input transformations (quadplane)
 *          6. Processes transmitter tuning inputs if enabled
 *          
 *          Throttle Nudge Feature:
 *          When enabled (g.throttle_nudge), allows pilot to request higher cruise
 *          speed/throttle by moving throttle stick above 50%. The nudge is calculated
 *          as a linear function of stick position:
 *          - Airspeed mode: Adds to cruise airspeed (up to max airspeed)
 *          - Throttle mode: Adds to cruise throttle (up to max throttle)
 *          
 *          Failsafe Behavior:
 *          If RC input read fails or signal is lost, triggers control_failsafe()
 *          which sets all control inputs to trim/safe values and may trigger
 *          autonomous failsafe mode (RTL, etc.) depending on configuration.
 * 
 * @note Called at main loop rate (typically 50Hz)
 * @note Throttle nudge only applies when fence stick mixing is allowed
 * @note Updates failsafe.AFS_last_valid_rc_ms and failsafe.last_valid_rc_ms timestamps
 * 
 * @warning This function is flight-critical; RC signal loss must be detected reliably
 * 
 * @see control_failsafe() for failsafe response handling
 * @see rc_failsafe_active() for failsafe detection logic
 * @see RC_Channels::read_input() for low-level RC frame decoding
 * 
 * Source: ArduPlane/radio.cpp:119-164
 */
void Plane::read_radio()
{
    if (!rc().read_input()) {
        control_failsafe();
        return;
    }

    if (!failsafe.rc_failsafe)
    {
        failsafe.AFS_last_valid_rc_ms = millis();
    }

    if (rc_throttle_value_ok()) {
        failsafe.last_valid_rc_ms = millis();
    }

    control_failsafe();

#if AP_FENCE_ENABLED
    const bool stickmixing = fence_stickmixing();
#else
    const bool stickmixing = true;
#endif
    airspeed_nudge_cm = 0;
    throttle_nudge = 0;
    if (g.throttle_nudge
        && channel_throttle->get_control_in() > 50
        && stickmixing) {
        float nudge = (channel_throttle->get_control_in() - 50) * 0.02f;
        if (ahrs.using_airspeed_sensor()) {
            airspeed_nudge_cm = (aparm.airspeed_max - aparm.airspeed_cruise) * nudge * 100;
        } else {
            throttle_nudge = (aparm.throttle_max - aparm.throttle_cruise) * nudge;
        }
    }

#if HAL_QUADPLANE_ENABLED
    // potentially swap inputs for tailsitters
    quadplane.tailsitter.check_input();
#endif

#if AP_TUNING_ENABLED
    // check for transmitter tuning changes
    tuning.check_input(control_mode->mode_number());
#endif
}

/**
 * @brief Get rudder input value considering special mode configurations
 * 
 * @details Returns the appropriate rudder input value based on current flight
 *          mode and configuration options. This function handles several special
 *          cases where rudder input should be ignored or modified:
 *          
 *          Rudder-Only Mode (g.rudder_only != 0):
 *          Returns 0 because in this mode the yaw channel is used for roll
 *          control instead of rudder. The aircraft steers using differential
 *          throttle or other mechanisms.
 *          
 *          Direct Rudder Only Option:
 *          When DIRECT_RUDDER_ONLY flight option is enabled, rudder input is
 *          only passed through in MANUAL, STABILIZE, and ACRO modes. In all
 *          other modes (AUTO, CRUISE, etc.), returns 0 to prevent pilot rudder
 *          input from interfering with autonomous navigation.
 *          
 *          Stick Mixing:
 *          If stick mixing is disabled for the current mode, returns 0 to
 *          prevent pilot input from affecting autonomous flight. When enabled,
 *          returns the full rudder channel input value.
 *          
 *          Return Values:
 *          - Normal operation: channel_rudder->get_control_in() (range ±4500)
 *          - Rudder suppressed: 0
 * 
 * @return int16_t Rudder input in range ±4500 (±45°), or 0 if suppressed
 * 
 * @note Return value is in centidegrees (±4500 = ±45°)
 * @note Used by flight modes to determine pilot rudder command
 * 
 * @see stick_mixing_enabled() for mode-specific stick mixing check
 * @see flight_option_enabled(FlightOptions::DIRECT_RUDDER_ONLY) for option check
 * 
 * Source: ArduPlane/radio.cpp:166-186
 */
int16_t Plane::rudder_input(void)
{
    if (g.rudder_only != 0) {
        // in rudder only mode we discard rudder input and get target
        // attitude from the roll channel.
        return 0;
    }

    if ((flight_option_enabled(FlightOptions::DIRECT_RUDDER_ONLY)) &&
        !(control_mode == &mode_manual || control_mode == &mode_stabilize || control_mode == &mode_acro)) {
        // the user does not want any input except in these modes
        return 0;
    }

    if (stick_mixing_enabled()) {
        return channel_rudder->get_control_in();
    }

    return 0;
    
}

/**
 * @brief Detect RC failsafe condition and set safe control inputs
 * 
 * @details This function implements RC failsafe detection and response, providing
 *          multiple layers of protection when radio signal is lost. It performs
 *          two primary functions:
 *          
 *          1. Sets control inputs to safe values when in failsafe
 *          2. Detects failsafe entry/exit with hysteresis to prevent oscillation
 *          
 *          Failsafe Response Actions:
 *          When RC failsafe is active, all control inputs are set to safe values:
 *          - Roll/Pitch/Rudder: Set to trim (level flight)
 *          - Throttle: Set to minimum (0%) for most modes
 *          - Quadplane VTOL modes: Set to 50% throttle to maintain altitude
 *          - Nudge values: Reset to zero
 *          
 *          Failsafe Detection Algorithm:
 *          Uses counter-based hysteresis to prevent chattering:
 *          - Entry: Requires 10 consecutive bad frames (200ms at 50Hz)
 *          - Exit: Requires counter to count down to 0 (prevents rapid toggling)
 *          - Counter is capped at maximum 10 to prevent overflow
 *          
 *          Failsafe Bypass Conditions:
 *          Failsafe checking is bypassed when:
 *          - Throttle failsafe is disabled (g.throttle_fs_enabled)
 *          - Aircraft is disarmed and not flying
 *          - RC receiver has never been detected (prevents false trigger at boot)
 *          
 *          Detection Methods:
 *          RC failsafe is triggered by:
 *          - Throttle value below configured threshold (g.throttle_fs_value)
 *          - No valid RC frames received for 1000ms
 *          - RC library reports read failure
 *          
 *          Notification:
 *          - Sets failsafe.rc_failsafe flag
 *          - Sets AP_Notify::flags.failsafe_radio for LED/buzzer indication
 *          - Sends "Throttle failsafe on/off" GCS message
 * 
 * @note Called at main loop rate (typically 50Hz) from read_radio()
 * @note Uses 10-frame hysteresis (200ms) to prevent false triggers from brief signal loss
 * @note Quadplane VTOL modes use 50% throttle in failsafe to avoid rapid descent
 * 
 * @warning This function is safety-critical; must reliably detect signal loss
 * @warning Throttle trim is intentionally NOT set during failsafe to allow throttle
 *          failsafe to trigger independently from RC failsafe
 * 
 * @see rc_failsafe_active() for failsafe condition checking
 * @see rc_throttle_value_ok() for throttle threshold validation
 * @see read_radio() for main RC processing loop
 * 
 * Source: ArduPlane/radio.cpp:188-264
 */
void Plane::control_failsafe()
{
    if (rc_failsafe_active()) {
        // we do not have valid RC input. Set all primary channel
        // control inputs to the trim value and throttle to min
        channel_roll->set_radio_in(channel_roll->get_radio_trim());
        channel_pitch->set_radio_in(channel_pitch->get_radio_trim());
        channel_rudder->set_radio_in(channel_rudder->get_radio_trim());

        // note that we don't set channel_throttle->radio_in to radio_trim,
        // as that would cause throttle failsafe to not activate
        channel_roll->set_control_in(0);
        channel_pitch->set_control_in(0);
        channel_rudder->set_control_in(0);

        airspeed_nudge_cm = 0;
        throttle_nudge = 0;

        switch (control_mode->mode_number()) {
#if HAL_QUADPLANE_ENABLED
            case Mode::Number::QSTABILIZE:
            case Mode::Number::QHOVER:
            case Mode::Number::QLOITER:
            case Mode::Number::QLAND: // throttle is ignored, but reset anyways
            case Mode::Number::QRTL:  // throttle is ignored, but reset anyways
            case Mode::Number::QACRO:
#if QAUTOTUNE_ENABLED
            case Mode::Number::QAUTOTUNE:
#endif
                if (quadplane.available() && quadplane.motors->get_desired_spool_state() > AP_Motors::DesiredSpoolState::GROUND_IDLE) {
                    // set half throttle to avoid descending at maximum rate, still has a slight descent due to throttle deadzone
                    channel_throttle->set_control_in(channel_throttle->get_range() / 2);
                    break;
                }
                FALLTHROUGH;
#endif
            default:
                channel_throttle->set_control_in(0);
                break;
        }
    }

    const bool allow_failsafe_bypass = !arming.is_armed() && !is_flying() && (rc().enabled_protocols() != 0);
    const bool has_had_input = rc().has_had_rc_receiver() || rc().has_had_rc_override();
    if ((g.throttle_fs_enabled != ThrFailsafe::Enabled && !failsafe.rc_failsafe) || (allow_failsafe_bypass && !has_had_input)) {
        // If throttle fs not enabled and not in failsafe, or 
        // not flying and disarmed, don't trigger failsafe check until RC has been received for the fist time  
        return;
    }

    if (rc_failsafe_active()) {
        // we detect a failsafe from radio
        // throttle has dropped below the mark
        failsafe.throttle_counter++;
        if (failsafe.throttle_counter == 10) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Throttle failsafe %s", "on");
            failsafe.rc_failsafe = true;
            AP_Notify::flags.failsafe_radio = true;
        }
        if (failsafe.throttle_counter > 10) {
            failsafe.throttle_counter = 10;
        }
    } else if(failsafe.throttle_counter > 0) {
        // we are no longer in failsafe condition
        // but we need to recover quickly
        failsafe.throttle_counter--;
        if (failsafe.throttle_counter > 3) {
            failsafe.throttle_counter = 3;
        }
        if (failsafe.throttle_counter == 1) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Throttle failsafe %s", "off");
        } else if(failsafe.throttle_counter == 0) {
            failsafe.rc_failsafe = false;
            AP_Notify::flags.failsafe_radio = false;
        }
    }
}

/**
 * @brief Learn and save RC trim values for level flight
 * 
 * @details This function captures the current servo output positions and saves them
 *          as trim values for all control surfaces. It implements the radio trim
 *          learning feature that allows pilots to "teach" the autopilot what stick
 *          positions and servo outputs correspond to level flight.
 *          
 *          Trim Learning Process:
 *          1. Verify aircraft is in MANUAL mode (required for safety)
 *          2. Check that control sticks are near center (within 20% of range)
 *          3. Verify aircraft is not rotating rapidly (< 30°/s gyro rate)
 *          4. Capture current servo output positions for all control surfaces
 *          5. Set RC input trim values to current stick positions
 *          6. Save trim values to persistent storage
 *          
 *          Surfaces Trimmed:
 *          - Primary: Aileron, elevator, rudder
 *          - Elevons: Left and right elevon surfaces
 *          - V-tail: Left and right V-tail surfaces
 *          - Differential spoilers: Only if no rudder input present
 *          - Flaperons: Only if no flap input present
 *          
 *          Safety Checks:
 *          - Must be in MANUAL mode (pilot has direct control)
 *          - Sticks must be centered (within 20% deadband)
 *          - Aircraft must not be rotating (< 30°/s gyro reading)
 *          - RC signal must be valid (no failsafe active)
 *          
 *          This function is typically triggered by a transmitter switch or
 *          ground station command during initial aircraft setup or after
 *          control surface adjustments.
 * 
 * @note Trim values are saved to EEPROM and persist across power cycles
 * @note Function will abort with error message if safety checks fail
 * 
 * @warning Incorrect trim can cause loss of control; only trim in calm conditions
 * @warning Do not trim while aircraft is moving or experiencing turbulence
 * 
 * @see SRV_Channels::set_trim_to_servo_out_for() for servo trim capture
 * @see RC_Channel::set_and_save_trim() for RC input trim storage
 * 
 * Source: ArduPlane/radio.cpp:266-328
 */
void Plane::trim_radio()
{
    if (failsafe.rc_failsafe) {
        // can't trim if we don't have valid input
        return;
    }

    if (plane.control_mode != &mode_manual) {
        gcs().send_text(MAV_SEVERITY_ERROR, "trim failed, not in manual mode");
        return;
    }

    if (labs(channel_roll->get_control_in()) > (channel_roll->get_range() * 0.2) ||
            labs(channel_pitch->get_control_in()) > (channel_pitch->get_range() * 0.2)) {
        // don't trim for extreme values - if we attempt to trim
        // more than 20 percent range left then assume the
        // sticks are not properly centered. This also prevents
        // problems with starting APM with the TX off
        gcs().send_text(MAV_SEVERITY_ERROR, "trim failed, large roll and pitch input");
        return;
    }

    if (degrees(ahrs.get_gyro().length()) > 30.0) {
        // rotating more than 30 deg/second
        gcs().send_text(MAV_SEVERITY_ERROR, "trim failed, large movement");
        return;
    }

    // trim main surfaces
    SRV_Channels::set_trim_to_servo_out_for(SRV_Channel::k_aileron);
    SRV_Channels::set_trim_to_servo_out_for(SRV_Channel::k_elevator);
    SRV_Channels::set_trim_to_servo_out_for(SRV_Channel::k_rudder);

    // trim elevons
    SRV_Channels::set_trim_to_servo_out_for(SRV_Channel::k_elevon_left);
    SRV_Channels::set_trim_to_servo_out_for(SRV_Channel::k_elevon_right);

    // trim vtail
    SRV_Channels::set_trim_to_servo_out_for(SRV_Channel::k_vtail_left);
    SRV_Channels::set_trim_to_servo_out_for(SRV_Channel::k_vtail_right);
    
    if (is_zero(SRV_Channels::get_output_scaled(SRV_Channel::k_rudder))) {
        // trim differential spoilers if no rudder input
        SRV_Channels::set_trim_to_servo_out_for(SRV_Channel::k_dspoilerLeft1);
        SRV_Channels::set_trim_to_servo_out_for(SRV_Channel::k_dspoilerLeft2);
        SRV_Channels::set_trim_to_servo_out_for(SRV_Channel::k_dspoilerRight1);
        SRV_Channels::set_trim_to_servo_out_for(SRV_Channel::k_dspoilerRight2);
    }

    if (is_zero(SRV_Channels::get_slew_limited_output_scaled(SRV_Channel::k_flap_auto)) &&
        is_zero(SRV_Channels::get_slew_limited_output_scaled(SRV_Channel::k_flap))) {
        // trim flaperons if no flap input
        SRV_Channels::set_trim_to_servo_out_for(SRV_Channel::k_flaperon_left);
        SRV_Channels::set_trim_to_servo_out_for(SRV_Channel::k_flaperon_right);
    }

    // now save input trims, as these have been moved to the outputs
    channel_roll->set_and_save_trim();
    channel_pitch->set_and_save_trim();
    channel_rudder->set_and_save_trim();

    gcs().send_text(MAV_SEVERITY_NOTICE, "trim complete");
}

/**
 * @brief Check if throttle PWM value is within acceptable range (not in failsafe)
 * 
 * @details This function validates that the received throttle PWM value is above
 *          (or below for reversed throttle) the configured failsafe threshold.
 *          It's one of two primary methods for detecting RC signal loss.
 *          
 *          Throttle Failsafe Threshold:
 *          The THR_FS_VALUE parameter defines the PWM value below which (or above
 *          for reversed throttle) the signal is considered lost. Typical values:
 *          - Normal throttle: THR_FS_VALUE = 950 (below normal min of ~1000)
 *          - Reversed throttle: THR_FS_VALUE above max
 *          
 *          When throttle failsafe is disabled (THR_FAILSAFE = 0), this function
 *          always returns true, relying instead on time-based failsafe detection.
 *          
 *          Reverse Throttle Handling:
 *          For reversed throttle channels (used with some ESCs), the comparison
 *          is inverted - signal is valid when PWM is below threshold instead of above.
 * 
 * @return true if throttle value is acceptable (not in failsafe), false if in failsafe range
 * 
 * @note Returns true immediately if throttle failsafe is disabled
 * @note Comparison direction depends on channel_throttle->get_reverse()
 * 
 * @see g.throttle_fs_enabled parameter to enable/disable throttle failsafe
 * @see g.throttle_fs_value parameter for PWM threshold
 * @see rc_failsafe_active() for combined failsafe detection
 * 
 * Source: ArduPlane/radio.cpp:333-342
 */
bool Plane::rc_throttle_value_ok(void) const
{
    if (g.throttle_fs_enabled == ThrFailsafe::Disabled) {
        return true;
    }
    if (channel_throttle->get_reverse()) {
        return channel_throttle->get_radio_in() < g.throttle_fs_value;
    }
    return channel_throttle->get_radio_in() > g.throttle_fs_value;
}

/**
 * @brief Check if RC failsafe condition is currently active
 * 
 * @details This function determines if the aircraft is currently in an RC failsafe
 *          condition using two independent detection methods:
 *          
 *          1. Throttle Value Check:
 *             Validates that received throttle PWM is above the configured failsafe
 *             threshold (THR_FS_VALUE parameter). This detects receiver-based
 *             failsafe where the receiver outputs a specific low PWM value when
 *             signal is lost.
 *          
 *          2. Time-Based Check:
 *             Ensures valid RC frames have been received within the last 1000ms.
 *             This catches cases where the receiver stops outputting valid data
 *             but doesn't necessarily drop to the failsafe PWM value.
 *          
 *          Either condition triggers failsafe status. This dual-method approach
 *          provides robust detection across different receiver types and failure modes.
 *          
 *          Timing Details:
 *          - Timestamp updated in read_radio() when valid frames received
 *          - 1000ms timeout provides margin for brief signal interruptions
 *          - Allows 20-50 missed frames before triggering (at 50Hz update rate)
 * 
 * @return true if in RC failsafe (signal lost or throttle below threshold), false if RC valid
 * 
 * @note This function only checks current status; it does not trigger failsafe actions
 * @note failsafe.last_valid_rc_ms is updated by read_radio() on valid frames
 * 
 * @see rc_throttle_value_ok() for throttle threshold checking
 * @see control_failsafe() for failsafe response and hysteresis
 * @see read_radio() for timestamp updates
 * 
 * Source: ArduPlane/radio.cpp:348-358
 */
bool Plane::rc_failsafe_active(void) const
{
    if (!rc_throttle_value_ok()) {
        return true;
    }
    if (millis() - failsafe.last_valid_rc_ms > 1000) {
        // we haven't had a valid RC frame for 1 seconds
        return true;
    }
    return false;
}

/**
 * @brief Apply exponential curve to RC channel input for smoother control response
 * 
 * @details This function applies an exponential (expo) curve to RC stick input,
 *          providing finer control around center stick while maintaining full
 *          deflection at the extremes. Expo curves are commonly used in manual
 *          flight modes (MANUAL, ACRO, TRAINING) to reduce sensitivity near
 *          neutral and make aircraft easier to fly smoothly.
 *          
 *          Expo Curve Characteristics:
 *          - Expo = 0: Linear response (no curve applied)
 *          - Expo = 50: Moderate reduction in center sensitivity
 *          - Expo = 100: Maximum reduction in center sensitivity
 *          - Full stick deflection always produces full output (±SERVO_MAX)
 *          
 *          The expo curve is calculated using the expo_curve() function which
 *          implements: output = sign(input) * |input|^(1+expo)
 *          This provides progressively less output for small inputs while
 *          preserving full range at the extremes.
 *          
 *          Dead Zone Handling:
 *          The use_dz parameter determines whether the channel's configured
 *          dead zone is applied before the expo curve:
 *          - use_dz = true: Apply dead zone (get_control_in())
 *          - use_dz = false: No dead zone (get_control_in_zero_dz())
 * 
 * @param[in] chan      RC channel to process (roll/pitch/rudder)
 * @param[in] expo      Expo amount in range 0-100 (percentage)
 * @param[in] use_dz    true to apply dead zone, false for zero dead zone
 * 
 * @return float Expo-curved output in range ±SERVO_MAX (±4500)
 * 
 * @note Returns 0 if channel pointer is null
 * @note Expo parameter is scaled to 0.0-1.0 range internally
 * @note Output is always scaled to ±SERVO_MAX regardless of input range
 * 
 * @see expo_curve() in AP_Math for curve calculation
 * @see roll_in_expo(), pitch_in_expo(), rudder_in_expo() for axis-specific wrappers
 * 
 * Source: ArduPlane/radio.cpp:363-370
 */
static float channel_expo(RC_Channel *chan, int8_t expo, bool use_dz)
{
    if (chan == nullptr) {
        return 0;
    }
    float rin = use_dz? chan->get_control_in() : chan->get_control_in_zero_dz();
    return SERVO_MAX * expo_curve(constrain_float(expo*0.01, 0, 1), rin/SERVO_MAX);
}

/**
 * @brief Get roll input with expo curve applied
 * 
 * @details Applies the configured manual roll expo curve (MAN_EXPO_ROLL parameter)
 *          to the roll channel input. Used in MANUAL, ACRO, and TRAINING modes
 *          to provide smoother roll control response.
 * 
 * @param[in] use_dz  true to apply dead zone, false for zero dead zone
 * 
 * @return float Roll input with expo curve in range ±SERVO_MAX (±4500)
 * 
 * @see channel_expo() for expo curve implementation
 * @see g2.man_expo_roll parameter for expo configuration
 * 
 * Source: ArduPlane/radio.cpp:372-375
 */
float Plane::roll_in_expo(bool use_dz) const
{
    return channel_expo(channel_roll, g2.man_expo_roll, use_dz);
}

/**
 * @brief Get pitch input with expo curve applied
 * 
 * @details Applies the configured manual pitch expo curve (MAN_EXPO_PITCH parameter)
 *          to the pitch channel input. Used in MANUAL, ACRO, and TRAINING modes
 *          to provide smoother pitch control response.
 * 
 * @param[in] use_dz  true to apply dead zone, false for zero dead zone
 * 
 * @return float Pitch input with expo curve in range ±SERVO_MAX (±4500)
 * 
 * @see channel_expo() for expo curve implementation
 * @see g2.man_expo_pitch parameter for expo configuration
 * 
 * Source: ArduPlane/radio.cpp:377-380
 */
float Plane::pitch_in_expo(bool use_dz) const
{
    return channel_expo(channel_pitch, g2.man_expo_pitch, use_dz);
}

/**
 * @brief Get rudder input with expo curve applied
 * 
 * @details Applies the configured manual rudder expo curve (MAN_EXPO_RUDDER parameter)
 *          to the rudder channel input. Used in MANUAL, ACRO, and TRAINING modes
 *          to provide smoother rudder control response.
 * 
 * @param[in] use_dz  true to apply dead zone, false for zero dead zone
 * 
 * @return float Rudder input with expo curve in range ±SERVO_MAX (±4500)
 * 
 * @see channel_expo() for expo curve implementation
 * @see g2.man_expo_rudder parameter for expo configuration
 * 
 * Source: ArduPlane/radio.cpp:382-385
 */
float Plane::rudder_in_expo(bool use_dz) const
{
    return channel_expo(channel_rudder, g2.man_expo_rudder, use_dz);
}

/**
 * @brief Check if throttle stick is at idle position
 * 
 * @details Determines if the throttle stick is in the idle/zero position,
 *          handling both standard throttle and center-trim throttle configurations.
 *          
 *          Standard Throttle Mode:
 *          Checks if throttle is at minimum position (bottom of stick travel).
 *          Uses in_min_dz() which checks if stick is in the minimum dead zone.
 *          
 *          Center-Trim Throttle Mode (CENTER_THROTTLE_TRIM option):
 *          For spring-loaded throttle sticks that return to center, trim is
 *          moved to the center position. Idle is then defined as the trim
 *          position. Uses in_trim_dz() which checks if stick is in the trim
 *          dead zone (center position).
 *          
 *          This function is used to:
 *          - Prevent arming when throttle is not at idle
 *          - Detect landing condition in some modes
 *          - Enable/disable certain features based on throttle position
 *          
 *          Dead Zone Checking:
 *          Both checks use dead zone testing (_dz suffix) to account for
 *          small variations in stick position, preventing false negatives
 *          from electrical noise or imperfect centering.
 * 
 * @return true if throttle stick is at idle position, false otherwise
 * 
 * @note Behavior depends on CENTER_THROTTLE_TRIM flight option
 * @note Uses dead zone checks to handle stick position variations
 * 
 * @see flight_option_enabled(FlightOptions::CENTER_THROTTLE_TRIM) for mode check
 * @see RC_Channel::in_trim_dz() for center position checking
 * @see RC_Channel::in_min_dz() for minimum position checking
 * 
 * Source: ArduPlane/radio.cpp:387-401
 */
bool Plane::throttle_at_zero(void) const
{
    /*
      true if throttle stick is at idle position...if throttle trim has been moved
       to center stick area in conjunction with sprung throttle, cannot use in_trim, must use rc_min
    */
    const bool center_trim = flight_option_enabled(FlightOptions::CENTER_THROTTLE_TRIM);
    if (center_trim && channel_throttle->in_trim_dz()) {
        return true;
    }
    if (!center_trim && channel_throttle->in_min_dz()) {
        return true;
    }
    return false;
}
