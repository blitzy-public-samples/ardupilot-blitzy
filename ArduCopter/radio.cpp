/**
 * @file radio.cpp
 * @brief Radio control input processing and failsafe management for ArduCopter
 * 
 * @details This file handles all radio control (RC) input processing for the copter,
 *          including reading RC frames, managing throttle input, detecting throttle-zero
 *          conditions, and triggering radio failsafe when RC signal is lost or invalid.
 *          
 *          Key responsibilities:
 *          - RC channel initialization and configuration
 *          - Radio frame reading and validation
 *          - Throttle failsafe detection and debouncing
 *          - Throttle-zero flag management for motor shutdown detection
 *          - RC input passthrough to motor library
 *          
 *          Safety-critical: This module implements radio failsafe, a critical safety
 *          feature that protects the vehicle when RC control is lost.
 * 
 * @note Radio input processing runs at the main loop rate (typically 400Hz)
 * @warning Modifications to failsafe logic can affect vehicle safety during RC loss
 * 
 * Source: ArduCopter/radio.cpp
 */

#include "Copter.h"


// Function that will read the radio data, limit servos and trigger a failsafe
// ----------------------------------------------------------------------------

/**
 * @brief Set default dead zones for primary control channels
 * 
 * @details Configures the dead zone (center stick region with no response) for
 *          roll, pitch, yaw, and throttle channels. Dead zones prevent control
 *          jitter from stick centering imperfections and allow precise neutral
 *          stick positioning.
 *          
 *          Dead zone values are in PWM microseconds around the stick center point.
 *          Helicopter frames use smaller dead zones for more responsive control.
 *          
 *          Standard frames:
 *          - Roll/Pitch: 20μs dead zone
 *          - Throttle: 30μs dead zone  
 *          - Yaw: 20μs dead zone
 *          
 *          Helicopter frames:
 *          - Roll/Pitch: 20μs dead zone
 *          - Throttle: 10μs dead zone (smaller for collective pitch precision)
 *          - Yaw: 15μs dead zone
 * 
 * @note Called during RC input initialization
 * @note Dead zones can be overridden by user parameters
 * 
 * @see init_rc_in()
 * @see RC_Channel::set_default_dead_zone()
 */
void Copter::default_dead_zones()
{
    channel_roll->set_default_dead_zone(20);
    channel_pitch->set_default_dead_zone(20);
#if FRAME_CONFIG == HELI_FRAME
    channel_throttle->set_default_dead_zone(10);
    channel_yaw->set_default_dead_zone(15);
#else
    channel_throttle->set_default_dead_zone(30);
    channel_yaw->set_default_dead_zone(20);
#endif
}

/**
 * @brief Initialize radio control input channels and configuration
 * 
 * @details Sets up the primary RC input channels (roll, pitch, throttle, yaw) by
 *          retrieving references from the RC_Channel library and configuring their
 *          input ranges and dead zones.
 *          
 *          Initialization sequence:
 *          1. Obtain channel references from RC library (guaranteed non-null)
 *          2. Configure angle channels (roll/pitch/yaw) with ±ROLL_PITCH_YAW_INPUT_MAX range
 *          3. Configure throttle channel with 0-1000 range
 *          4. Find transmitter tuning auxiliary channel (if configured)
 *          5. Apply default dead zones for all primary channels
 *          6. Initialize throttle_zero flag to true (motors off state)
 *          
 *          Channel ranges:
 *          - Roll/Pitch/Yaw: ±4500 (centidegrees) for angle control modes
 *          - Throttle: 0-1000 for motor throttle control
 *          
 *          The RC_Channel library handles the low-level protocol decoding (SBUS, PPM,
 *          DSM, CRSF, etc.) and provides normalized/scaled channel values to the
 *          vehicle code.
 * 
 * @note Called once during vehicle initialization (setup phase)
 * @note Channel pointers are stored in Copter class members for fast access
 * @note Throttle_zero flag starts true, indicating motors should be off until armed
 * 
 * @see default_dead_zones()
 * @see RC_Channel::set_angle()
 * @see RC_Channel::set_range()
 */
void Copter::init_rc_in()
{
    // the library guarantees that these are non-nullptr:
    channel_roll     = &rc().get_roll_channel();
    channel_pitch    = &rc().get_pitch_channel();
    channel_throttle = &rc().get_throttle_channel();
    channel_yaw      = &rc().get_yaw_channel();

    // set rc channel ranges
    channel_roll->set_angle(ROLL_PITCH_YAW_INPUT_MAX);
    channel_pitch->set_angle(ROLL_PITCH_YAW_INPUT_MAX);
    channel_yaw->set_angle(ROLL_PITCH_YAW_INPUT_MAX);
    channel_throttle->set_range(1000);

    rc_tuning = rc().find_channel_for_option(RC_Channel::AUX_FUNC::TRANSMITTER_TUNING);

    // set default dead zones
    default_dead_zones();

    // initialise throttle_zero flag
    ap.throttle_zero = true;
}

/**
 * @brief Initialize motor outputs and RC output configuration
 * 
 * @details Configures the motor library with frame type/class and sets up PWM output
 *          parameters based on RC throttle channel configuration. This function bridges
 *          RC input configuration to motor output generation.
 *          
 *          Initialization sequence:
 *          1. Initialize motors with configured frame class and type
 *          2. Enable auxiliary servo outputs for multi-channel-per-motor support
 *          3. Set motor update rate from RC_SPEED parameter
 *          4. Configure motor PWM min/max from throttle channel or defaults
 *          5. Update throttle range scaling in motor library
 *          6. Refresh auxiliary servo to function mapping
 *          7. Set safety ignore mask for non-motor channels (e.g., gimbals)
 *          
 *          For non-helicopter frames:
 *          - If throttle channel configured: Use actual RC min/max for motor PWM range
 *          - If throttle not configured: Force standard 1000-2000μs range as default
 *          - Safety mask allows servos to operate while safety switch is on
 *          
 *          For helicopter frames:
 *          - ESC scaling configured directly from throttle channel range
 *          - Supports proportion-based ESCs (e.g., UAVCAN)
 * 
 * @note Called once during vehicle initialization (setup phase)
 * @note Must be called after init_rc_in() to ensure channels are configured
 * @note Motor update rate typically 400Hz, may be higher for certain applications
 * @note Safety ignore mask prevents accidental gimbal movement during safety checks
 * 
 * @warning Frame type changes require reboot to take effect properly
 * 
 * @see init_rc_in()
 * @see AP_Motors::init()
 * @see AP_Motors::set_update_rate()
 */
void Copter::init_rc_out()
{
    motors->init((AP_Motors::motor_frame_class)g2.frame_class.get(), (AP_Motors::motor_frame_type)g.frame_type.get());

    // enable aux servos to cope with multiple output channels per motor
    AP::srv().enable_aux_servos();

    // update rate must be set after motors->init() to allow for motor mapping
    motors->set_update_rate(g.rc_speed);

#if FRAME_CONFIG != HELI_FRAME
    if (channel_throttle->configured()) {
        // throttle inputs setup, use those to set motor PWM min and max if not already configured
        motors->convert_pwm_min_max_param(channel_throttle->get_radio_min(), channel_throttle->get_radio_max());
    } else {
        // throttle inputs default, force set motor PWM min and max to defaults so they will not be over-written by a future change in RC min / max
        motors->convert_pwm_min_max_param(1000, 2000);
    }
    motors->update_throttle_range();
#else
    // setup correct scaling for ESCs like the UAVCAN ESCs which
    // take a proportion of speed.
    hal.rcout->set_esc_scaling(channel_throttle->get_radio_min(), channel_throttle->get_radio_max());
#endif

    // refresh auxiliary channel to function map
    SRV_Channels::update_aux_servo_function();

#if FRAME_CONFIG != HELI_FRAME
    /*
      setup a default safety ignore mask, so that servo gimbals can be active while safety is on
     */
    uint16_t safety_ignore_mask = (~copter.motors->get_motor_mask()) & 0x3FFF;
    BoardConfig.set_default_safety_ignore_mask(safety_ignore_mask);
#endif
}


/**
 * @brief Read radio control input and manage radio failsafe state
 * 
 * @details Primary function for processing RC input frames and detecting radio signal loss.
 *          Called at main loop rate (typically 400Hz) to continuously monitor RC health
 *          and update control inputs.
 *          
 *          Normal operation (RC frame received):
 *          1. Set new_radio_frame flag to indicate fresh data available
 *          2. Process throttle input and check for throttle-based failsafe
 *          3. Update throttle_zero flag for motor shutdown detection
 *          4. Pass pilot inputs to motors (enables servo wiggling while disarmed)
 *          5. Apply low-pass filter to throttle control input
 *          6. Update timestamp of last successful radio frame
 *          
 *          Failsafe triggering (no RC frame received):
 *          - Monitors time since last valid radio frame
 *          - Triggers failsafe if timeout exceeds FS_TIMEOUT (default 1 second)
 *          - Only triggers if throttle failsafe enabled AND (armed OR has seen RC before)
 *          - Logs RADIO_LATE_FRAME error before entering failsafe
 *          
 *          Failsafe conditions checked:
 *          1. Time since last frame > FS_TIMEOUT (typically 1000ms)
 *          2. Throttle failsafe parameter enabled (FS_THR > 0)
 *          3. Vehicle is armed OR RC has been seen before (prevents failsafe on bench)
 *          
 *          The RC library (RC_Channels) handles low-level protocol decoding and provides
 *          boolean success indicator via read_input(). This function operates at the
 *          vehicle level to determine appropriate failsafe response.
 * 
 * @note Called every main loop iteration (400Hz nominal)
 * @note Uses millis() for timing, so maximum timeout is ~49 days (uint32_t rollover)
 * @note new_radio_frame flag consumed by other systems to detect fresh input
 * @note Throttle control input filtered with time-based low-pass filter
 * 
 * @warning This is safety-critical code - radio failsafe protects vehicle during RC loss
 * @warning Failsafe timeout must be long enough to avoid false triggers from temporary
 *          signal dropouts but short enough to respond to genuine RC failure
 * @warning Bench testing should be done with vehicle disarmed to prevent failsafe triggers
 * 
 * @see set_throttle_and_failsafe()
 * @see set_throttle_zero_flag()
 * @see set_failsafe_radio()
 * @see RC_Channels::read_input()
 * 
 * Source: ArduCopter/radio.cpp:82-125
 */
void Copter::read_radio()
{
    const uint32_t tnow_ms = millis();

    // Attempt to read new RC frame from receiver via RC_Channels library
    if (rc().read_input()) {
        // SUCCESS: Valid RC frame received
        ap.new_radio_frame = true;

        // Process throttle input for failsafe detection (checks for low throttle)
        set_throttle_and_failsafe(channel_throttle->get_radio_in());
        
        // Update throttle-zero flag with debouncing (detects pilot intent to shut down motors)
        set_throttle_zero_flag(channel_throttle->get_control_in());

        // pass pilot input through to motors (used to allow wiggling servos while disarmed on heli, single, coax copters)
        radio_passthrough_to_motors();

        // Apply time-based low-pass filter to throttle control input for smooth response
        const float dt = (tnow_ms - last_radio_update_ms)*1.0e-3f;
        rc_throttle_control_in_filter.apply(channel_throttle->get_control_in(), dt);
        last_radio_update_ms = tnow_ms;
        return;
    }

    // No radio input this time - check if we need to trigger failsafe
    if (failsafe.radio) {
        // already in failsafe!
        return;
    }

    // Calculate time elapsed since last successful RC frame
    // trigger failsafe if no update from the RC Radio for RC_FS_TIMEOUT seconds
    const uint32_t elapsed_ms = tnow_ms - last_radio_update_ms;
    if (elapsed_ms < rc().get_fs_timeout_ms()) {
        // not timed out yet - still within acceptable gap between frames
        return;
    }
    if (!g.failsafe_throttle) {
        // throttle failsafe not enabled - user has disabled radio failsafe protection
        return;
    }
    if (!rc().has_ever_seen_rc_input() && !motors->armed()) {
        // we only failsafe if we are armed OR we have ever seen an RC receiver
        // This prevents failsafe on the bench before RC is bound
        return;
    }

    // All failsafe conditions met - RC signal lost for too long
    // Log an error and enter failsafe.
    LOGGER_WRITE_ERROR(LogErrorSubsystem::RADIO, LogErrorCode::RADIO_LATE_FRAME);
    set_failsafe_radio(true);
}

/**
 * @brief Debounce counter for throttle-based radio failsafe
 * 
 * @details Radio failsafe requires 3 consecutive low throttle readings before triggering
 *          to prevent false failsafes from momentary signal glitches. Similarly, 3
 *          consecutive good throttle readings are required to exit failsafe.
 */
#define FS_COUNTER 3        // radio failsafe kicks in after 3 consecutive throttle values below failsafe_throttle_value

/**
 * @brief Process throttle input and manage throttle-based radio failsafe with debouncing
 * 
 * @details Implements throttle-value-based failsafe detection as an additional safety
 *          mechanism beyond the timing-based failsafe in read_radio(). This catches
 *          cases where the RC receiver is still sending frames but with invalid/failsafe
 *          throttle values (common behavior when RC transmitter is turned off).
 *          
 *          Debouncing algorithm (prevents false triggers from signal noise):
 *          - Increments counter each time throttle below failsafe threshold
 *          - Decrements counter each time throttle above failsafe threshold
 *          - Triggers failsafe when counter reaches FS_COUNTER (3)
 *          - Exits failsafe when counter returns to 0
 *          - Counter clamped to [0, FS_COUNTER] to prevent overflow/underflow
 *          
 *          Failsafe triggering conditions:
 *          1. Throttle failsafe enabled (FS_THR parameter > 0)
 *          2. Throttle PWM < FS_THR_VALUE (typically 975μs)
 *          3. Not already in failsafe state
 *          4. Vehicle armed OR RC has been seen before
 *          5. Three consecutive low throttle readings (debounced)
 *          
 *          Failsafe exit conditions:
 *          1. Currently in failsafe state
 *          2. Throttle PWM > FS_THR_VALUE
 *          3. Three consecutive good throttle readings (debounced)
 *          
 *          Common RC receiver failsafe behaviors:
 *          - "No signal": Receiver outputs fixed low throttle (e.g., 900μs)
 *          - "Hold last": Receiver holds last valid values (not detected by this function)
 *          - "Cut output": Receiver stops outputting PPM/SBUS (detected by timing failsafe)
 * 
 * @param[in] throttle_pwm Current throttle channel PWM value in microseconds (typically 1000-2000μs)
 * 
 * @note Called every time a new RC frame is received (typically 50-100Hz for most RC protocols)
 * @note Complements the timing-based failsafe in read_radio() - both mechanisms protect the vehicle
 * @note Debounce counter provides hysteresis to prevent oscillation at threshold boundary
 * @note Does not trigger failsafe on bench before RC is bound (requires armed or seen RC)
 * 
 * @warning This is safety-critical code - throttle failsafe protects vehicle when RC transmitter
 *          is turned off or loses connection
 * @warning Failsafe throttle value (FS_THR_VALUE) must be below normal flight minimum to avoid
 *          false triggers, but above receiver noise floor
 * @warning Setting FS_THR to 0 disables throttle failsafe, relying only on timing failsafe
 * 
 * @see read_radio()
 * @see set_failsafe_radio()
 * 
 * Source: ArduCopter/radio.cpp:128-164
 */
void Copter::set_throttle_and_failsafe(uint16_t throttle_pwm)
{
    // if failsafe not enabled pass through throttle and exit
    if(g.failsafe_throttle == FS_THR_DISABLED) {
        set_failsafe_radio(false);
        return;
    }

    // check for low throttle value (below failsafe threshold)
    if (throttle_pwm < (uint16_t)g.failsafe_throttle_value) {

        // if we are already in failsafe or motors not armed pass through throttle and exit
        if (failsafe.radio || !(rc().has_ever_seen_rc_input() || motors->armed())) {
            return;
        }

        // Increment debounce counter for low throttle detection
        // check for 3 low throttle values
        // Note: we do not pass through the low throttle until 3 low throttle values are received
        failsafe.radio_counter++;
        if( failsafe.radio_counter >= FS_COUNTER ) {
            failsafe.radio_counter = FS_COUNTER;  // check to ensure we don't overflow the counter
            // Three consecutive low throttle values confirmed - trigger failsafe
            set_failsafe_radio(true);
        }
    }else{
        // Throttle above failsafe threshold - decrement debounce counter
        // we have a good throttle so reduce failsafe counter
        failsafe.radio_counter--;
        if( failsafe.radio_counter <= 0 ) {
            failsafe.radio_counter = 0;   // check to ensure we don't underflow the counter

            // disengage failsafe after three (nearly) consecutive valid throttle values
            if (failsafe.radio) {
                set_failsafe_radio(false);
            }
        }
        // pass through throttle
    }
}

/**
 * @brief Debounce time for throttle-zero detection
 * 
 * @details Throttle must remain at zero for 400ms before throttle_zero flag is set.
 *          This prevents false zero detection from momentary pilot stick inputs and
 *          provides stable motor shutdown detection.
 */
#define THROTTLE_ZERO_DEBOUNCE_TIME_MS 400

/**
 * @brief Set throttle_zero flag from debounced throttle control input
 * 
 * @details Determines if the pilot intends to shut down the motors based on throttle
 *          position and special control modes. The throttle_zero flag signals when
 *          motors should be off - either on the ground or during controlled shutdown.
 *          
 *          This is NOT the same as motor arming - it indicates pilot's immediate
 *          intention for motor output. We are "not flying" when throttle_zero is true.
 *          
 *          Conditions that indicate motors SHOULD RUN (throttle_zero = false):
 *          1. Standard mode (no interlock): throttle > 0 AND not emergency stopped
 *          2. Motor interlock mode: interlock switch enabled (regardless of throttle)
 *          3. Airmode armed via switch: maintains stabilization at zero throttle
 *          4. Airmode enabled globally: maintains stabilization at zero throttle
 *          
 *          Conditions that indicate motors SHOULD STOP (throttle_zero = true):
 *          - Throttle at zero for > 400ms (debounced)
 *          - Emergency stop active
 *          - Motor interlock disabled (for helicopters)
 *          
 *          Debouncing prevents false zero detection from:
 *          - Brief pilot stick movements through zero
 *          - RC signal glitches
 *          - Rapid throttle changes during aggressive flying
 *          
 *          Airmode handling:
 *          - Airmode allows full attitude control at zero throttle for aerobatics
 *          - When airmode active, throttle_zero stays false even at zero throttle
 *          - Enables inverted flight, flips, and other advanced maneuvers
 *          
 *          Safety note: Throttle-zero is used by various subsystems:
 *          - Motor output: Determines if motors should spin
 *          - Landing detection: Helps determine if copter is on ground
 *          - Mode transitions: Some modes check throttle-zero before allowing entry
 *          - Disarm logic: May allow faster disarm when throttle-zero
 * 
 * @param[in] throttle_control Pilot throttle control input (0-1000 scale, after dead zone)
 * 
 * @note Called every time a new RC frame is received (typically 50-100Hz)
 * @note Uses static variable to maintain state between calls
 * @note Responds immediately when throttle becomes non-zero (no delay)
 * @note 400ms delay only applies when transitioning TO zero (prevents false positives)
 * 
 * @warning Throttle-zero affects motor output and vehicle behavior - ensure debounce
 *          time is long enough to prevent false triggers but short enough for safety
 * @warning Emergency stop overrides all other logic and forces throttle-zero immediately
 * 
 * @see read_radio()
 * @see AP_Motors::get_interlock()
 * @see SRV_Channels::get_emergency_stop()
 * 
 * Source: ArduCopter/radio.cpp:171-187
 */
void Copter::set_throttle_zero_flag(int16_t throttle_control)
{
    static uint32_t last_nonzero_throttle_ms = 0;
    uint32_t tnow_ms = millis();

    // Check if motors should be running based on multiple control modes:
    // if not using throttle interlock and non-zero throttle and not E-stopped,
    // or using motor interlock and it's enabled, then motors are running, 
    // and we are flying. Immediately set as non-zero
    if ((!ap.using_interlock && (throttle_control > 0) && !SRV_Channels::get_emergency_stop()) ||
        (ap.using_interlock && motors->get_interlock()) ||
        ap.armed_with_airmode_switch || air_mode == AirMode::AIRMODE_ENABLED) {
        // Motors should be running - update timestamp and clear throttle-zero flag immediately
        last_nonzero_throttle_ms = tnow_ms;
        ap.throttle_zero = false;
    } else if (tnow_ms - last_nonzero_throttle_ms > THROTTLE_ZERO_DEBOUNCE_TIME_MS) {
        // Throttle has been at zero for debounce period - set throttle-zero flag
        ap.throttle_zero = true;
    }
    // Note: If within debounce window, throttle_zero retains previous state
}

/**
 * @brief Pass pilot RC inputs directly to motor library for disarmed servo control
 * 
 * @details Provides raw pilot inputs to the motor library to enable servo movement
 *          while the vehicle is disarmed. This is primarily used for helicopters,
 *          single-rotor, and coaxial copters to allow ground testing and setup
 *          of swashplate servos, tail servos, and other control surfaces without
 *          arming the vehicle.
 *          
 *          Input scaling:
 *          - Roll/Pitch/Yaw: Normalized to ±1.0 range via norm_input()
 *          - Throttle: Control input (0-1000) scaled to 0.0-1.0 range
 *          
 *          The motor library decides if and how to use these passthrough values:
 *          - When disarmed: May pass through to servos (frame-type dependent)
 *          - When armed: Normal flight control takes precedence
 *          - Multirotors: Typically ignores passthrough (no servos)
 *          - Helicopters: Uses for swashplate and tail rotor servos
 *          - Single/Coax: Uses for control servos
 *          
 *          This enables:
 *          - Pre-flight servo range testing
 *          - Swashplate geometry verification
 *          - Tail rotor function checks
 *          - Control surface movement verification
 *          
 *          Safety: Passthrough only affects servos, not motors (ESCs) when disarmed.
 *          Motor outputs remain at minimum while vehicle is disarmed regardless of
 *          throttle input.
 * 
 * @note Called every time a new RC frame is received (typically 50-100Hz)
 * @note Actual usage of passthrough inputs is frame-type dependent
 * @note Standard multirotors (quad, hex, octo) typically don't use passthrough
 * @note Helicopters and single-rotor frames use this for servo control
 * @note Motor library enforces safety by not spinning motors when disarmed
 * 
 * @see AP_Motors::set_radio_passthrough()
 * @see RC_Channel::norm_input()
 * @see RC_Channel::get_control_in_zero_dz()
 * 
 * Source: ArduCopter/radio.cpp:190-196
 */
void Copter::radio_passthrough_to_motors()
{
    // Pass normalized control inputs to motor library:
    // - Roll, pitch, yaw: ±1.0 normalized input
    // - Throttle: 0.0-1.0 scaled from control_in (0-1000)
    motors->set_radio_passthrough(channel_roll->norm_input(),
                                  channel_pitch->norm_input(),
                                  channel_throttle->get_control_in_zero_dz()*0.001f,
                                  channel_yaw->norm_input());
}

/**
 * @brief Return throttle mid-stick value in control-in units
 * 
 * @details Returns the throttle channel value corresponding to mid-stick position,
 *          which is used as the hover throttle reference point for altitude control
 *          modes (Loiter, PosHold, Auto, etc.). This value represents where the
 *          pilot should hold the stick for neutral vertical velocity.
 *          
 *          The mid-stick value accounts for:
 *          - RC_THR_MID parameter (typically 500 = 50% stick)
 *          - Transmitter trim settings
 *          - Non-linear throttle curves (if configured)
 *          - Toy mode override (simplified control for beginners)
 *          
 *          In altitude hold modes, the flight controller:
 *          - Maintains altitude when throttle = mid-stick
 *          - Climbs when throttle > mid-stick
 *          - Descends when throttle < mid-stick
 *          
 *          Toy mode special handling:
 *          - When toy mode enabled, uses toy mode's simplified throttle mid
 *          - Provides easier altitude control for beginner pilots
 *          - Typically uses a more forgiving mid-point calculation
 *          
 *          Throttle learn feature:
 *          - Some modes can learn hover throttle during flight
 *          - Learned value may differ from RC mid-stick setting
 *          - This function always returns configured mid-stick, not learned hover
 * 
 * @return Throttle mid-stick value in control-in units (typically 500 for 50% stick)
 * 
 * @note Control-in range is 0-1000, where 500 typically represents mid-stick
 * @note Actual hover throttle may differ from mid-stick due to vehicle weight/battery
 * @note Used by altitude controllers to determine pilot's desired vertical velocity
 * @note RC_THR_MID parameter allows adjustment of mid-stick position
 * 
 * @see RC_Channel::get_control_mid()
 * @see ToyMode::get_throttle_mid()
 * 
 * Source: ArduCopter/radio.cpp:201-209
 */
int16_t Copter::get_throttle_mid(void)
{
#if TOY_MODE_ENABLED
    // Toy mode provides simplified throttle handling for beginners
    if (g2.toy_mode.enabled()) {
        return g2.toy_mode.get_throttle_mid();
    }
#endif
    // Standard mode: return configured RC channel mid-stick value
    return channel_throttle->get_control_mid();
}
