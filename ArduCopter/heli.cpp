/**
 * @file heli.cpp
 * @brief Traditional helicopter (single main rotor + tail rotor) specific control functions
 * 
 * @details This file contains ArduCopter functions specific to traditional helicopter
 *          configurations, which differ significantly from multirotor control in several ways:
 *          
 *          Key Differences from Multirotor Control:
 *          - Uses collective pitch control instead of motor speed variation for thrust
 *          - Requires swashplate mixing to convert attitude commands to cyclic pitch
 *          - Employs dedicated tail rotor for yaw control (not differential thrust)
 *          - Implements rotor speed governor to maintain constant head speed
 *          - Supports autorotation capability (unpowered descent using rotor inertia)
 *          - Uses leaky integrators to prevent integrator wind-up during dynamic flight
 *          - Requires special handling for landing to prevent collective droop
 *          
 *          Traditional Helicopter Control Overview:
 *          - Main rotor provides lift via collective pitch and directional control via cyclic pitch
 *          - Tail rotor compensates for main rotor torque and provides yaw authority
 *          - Engine/motor governor maintains constant rotor RPM across flight conditions
 *          - Swashplate translates pilot inputs into main rotor blade pitch changes
 *          
 *          Safety-Critical Considerations:
 *          - Rotor speed management is critical for maintaining control authority
 *          - Collective limits must be enforced during landing to prevent ground contact
 *          - Motor interlock prevents inadvertent rotor engagement
 *          - Autorotation detection enables emergency unpowered landings
 * 
 * @note All helicopter-specific code is conditionally compiled with FRAME_CONFIG == HELI_FRAME
 * @warning Incorrect configuration of helicopter parameters can result in loss of control
 * 
 * @see libraries/AP_Motors/AP_MotorsHeli.h for motor/rotor control implementation
 * @see libraries/AC_AttitudeControl for attitude control integration
 */

#include "Copter.h"

// Traditional helicopter variables and functions

#if FRAME_CONFIG == HELI_FRAME

#ifndef HELI_DYNAMIC_FLIGHT_SPEED_MIN
#define HELI_DYNAMIC_FLIGHT_SPEED_MIN      250     // we are in "dynamic flight" when the speed is over 2.5m/s for 2 seconds
#endif

/**
 * @brief Counter to control dynamic flight profile detection
 * 
 * @details Increments when helicopter is moving, decrements when stationary.
 *          Provides hysteresis for dynamic_flight flag (requires 2 seconds to change state).
 *          Called at 50Hz, so 100 counts = 2 seconds.
 */
static int8_t heli_dynamic_flight_counter;

/**
 * @brief Perform special initialization required for traditional helicopters
 * 
 * @details Initializes helicopter-specific control parameters at vehicle startup.
 *          
 *          Traditional helicopters require stabilized collective (stab col) to be
 *          enabled from boot because the vehicle initializes in Stabilize mode but
 *          the stabilize_init() function is not called during startup sequence.
 *          
 *          Stabilized Collective Overview:
 *          - Stab col provides automatic collective adjustment to maintain altitude
 *          - Works in conjunction with attitude control to coordinate cyclic and collective
 *          - Ramp value of 1.0 indicates full stabilization is active immediately
 *          - Differs from multirotor "throttle hover" as it adjusts blade pitch, not motor speed
 *          
 *          Initialization Sequence:
 *          1. Enable stabilized collective control (use_stab_col = true)
 *          2. Set collective ramp to full (1.0 = no ramping, immediate response)
 *          3. Motors and swashplate initialization handled separately in AP_MotorsHeli
 * 
 * @note This function is called once during Copter::init_ardupilot()
 * @note Does not initialize rotor speed governor - that is handled in AP_MotorsHeli
 * 
 * @see Copter::init_ardupilot() for boot sequence
 * @see AP_MotorsHeli_Single::init() for motor-specific initialization
 * @see Mode::Stabilize for stabilize mode implementation
 */
void Copter::heli_init()
{
    // pre-load stab col values as mode is initialized as Stabilize, but stabilize_init() function is not run on start-up.
    input_manager.set_use_stab_col(true);
    input_manager.set_stab_col_ramp(1.0);
}

/**
 * @brief Updates the dynamic_flight flag based on helicopter horizontal velocity and flight state
 * 
 * @details Determines if the helicopter is in "dynamic flight" (actively translating) versus
 *          hovering or stationary. This distinction is critical for traditional helicopters
 *          because different control strategies are needed:
 *          
 *          Dynamic Flight Characteristics:
 *          - Helicopter is translating horizontally at speed > 2.5 m/s
 *          - Requires aggressive control response and full swashplate authority
 *          - Uses standard PID integrators (not leaky integrators)
 *          - Full collective range available for maneuvering
 *          
 *          Hover/Low Speed Characteristics:
 *          - Helicopter is stationary or moving slowly (< 2.5 m/s)
 *          - Requires damped control response to prevent oscillations
 *          - Uses leaky integrators to prevent wind-up during hover
 *          - May limit collective range to prevent ground strikes during landing
 *          
 *          Detection Algorithm with 2-Second Hysteresis:
 *          1. Check preconditions (motor spooled up, not landing)
 *          2. Determine if moving using best available sensor:
 *             - Primary: GPS/INS velocity (most accurate)
 *             - Fallback: Throttle > 80% OR pitch < -15° (no GPS)
 *             - Tertiary: Rangefinder > 2m altitude (airborne check)
 *          3. Increment/decrement counter over 2 seconds (100 cycles at 50Hz)
 *          4. Update dynamic_flight flag when counter reaches threshold
 *          
 *          Why Hysteresis Matters:
 *          - Prevents rapid toggling between control modes during speed transitions
 *          - Avoids abrupt changes in integrator behavior that could cause oscillations
 *          - Provides stable control authority during approach and departure from hover
 * 
 * @note Must be called at 50Hz for correct timing (2 seconds = 100 calls)
 * @note Counter range: 0 (stationary) to 100 (dynamic flight confirmed)
 * 
 * @warning Calling at incorrect rate will affect hysteresis timing and control transitions
 * @warning Dynamic flight detection affects integrator behavior and swashplate limits
 * 
 * @see update_heli_control_dynamics() for how this flag affects control parameters
 * @see should_use_landing_swash() for collective range limiting based on this flag
 * @see AC_AttitudeControl::use_leaky_i() for integrator behavior changes
 */
void Copter::check_dynamic_flight(void)
{
    // Ensure motors are fully spooled and not in landing sequence
    // If either condition fails, immediately clear dynamic flight state
    if (motors->get_spool_state() != AP_Motors::SpoolState::THROTTLE_UNLIMITED ||
        flightmode->is_landing()) {
        heli_dynamic_flight_counter = 0;
        heli_flags.dynamic_flight = false;
        return;
    }

    bool moving = false;

    // Primary detection: Use GPS/INS velocity for accurate ground speed measurement
    // Velocity is in NED (North-East-Down) frame in m/s
    Vector3f vel_ned_ms;
    if (AP::ahrs().get_velocity_NED(vel_ned_ms)) {
        // Calculate horizontal speed (North-East plane) and convert to cm/s
        // HELI_DYNAMIC_FLIGHT_SPEED_MIN is 250 cm/s (2.5 m/s)
        moving = (vel_ned_ms.xy().length() * 100.0 >= HELI_DYNAMIC_FLIGHT_SPEED_MIN);
    } else {
        // Fallback detection without GPS: Use throttle and pitch attitude as proxies
        // High throttle (>80%) OR significant forward pitch (<-15°) indicates dynamic flight
        // Note: Pitch is negative when nose-down in NED frame
        moving = (motors->get_throttle() > 0.8f || ahrs.get_pitch_deg() < -15);
    }

#if AP_RANGEFINDER_ENABLED
    // Tertiary detection: Use rangefinder to detect airborne state
    // If previous methods indicate stationary but we're >2m above ground, we're likely
    // in dynamic flight (climbing/descending or drifting without GPS)
    if (!moving && rangefinder_state.enabled && rangefinder.status_orient(ROTATION_PITCH_270) == RangeFinder::Status::Good) {
        // ROTATION_PITCH_270 = downward-facing rangefinder
        // when we are more than 2m from the ground with good
        // rangefinder lock consider it to be dynamic flight
        moving = (rangefinder.distance_orient(ROTATION_PITCH_270) > 2);
    }
#endif

    // Apply 2-second hysteresis to prevent rapid state changes
    if (moving) {
        // if moving for 2 seconds (100 cycles at 50Hz), set the dynamic flight flag
        if (!heli_flags.dynamic_flight) {
            heli_dynamic_flight_counter++;
            if (heli_dynamic_flight_counter >= 100) {
                heli_flags.dynamic_flight = true;
                heli_dynamic_flight_counter = 100;  // Clamp to prevent overflow
            }
        }
    } else {
        // if not moving for 2 seconds (100 cycles at 50Hz), clear the dynamic flight flag
        if (heli_flags.dynamic_flight) {
            if (heli_dynamic_flight_counter > 0) {
                heli_dynamic_flight_counter--;
            } else {
                heli_flags.dynamic_flight = false;
            }
        }
    }
}

/**
 * @brief Updates helicopter-specific control dynamics parameters
 * 
 * @details Configures attitude control parameters based on current flight state and pushes
 *          critical state information to the motor control layer (AP_MotorsHeli).
 *          
 *          This function bridges the gap between high-level flight control and low-level
 *          motor/swashplate actuation, adapting control behavior for traditional helicopter
 *          characteristics that differ from multirotors.
 *          
 *          Key Control Adaptations for Traditional Helicopters:
 *          
 *          1. Leaky Integrator Management:
 *             - Multirotors: Use standard PID integrators (maintain error accumulation)
 *             - Helicopters: Switch between standard and leaky integrators based on flight state
 *             - Leaky integrators slowly decay accumulated error to prevent wind-up
 *             - Used during hover/low-speed to prevent oscillations from persistent errors
 *             - Disabled during dynamic flight for aggressive tracking performance
 *          
 *          2. Landing State Communication:
 *             - Informs motor layer of landing status for collective limit enforcement
 *             - Enables special handling of swashplate during ground contact
 *             - Prevents collective droop that could cause blade strikes
 *          
 *          3. Hover Roll Trim Compensation:
 *             - Helicopters naturally roll during hover due to tail rotor thrust
 *             - Trim scalar gradually enables automatic roll compensation after takeoff
 *             - Ramps from 0 to 1 over 1 second to prevent abrupt control changes
 *             - Disabled on ground to prevent uncommanded swashplate deflection
 *          
 *          Control Flow:
 *          1. Determine integrator mode (standard vs leaky) based on configuration and flight state
 *          2. Communicate landing state to motor control for collective limiting
 *          3. Update hover roll trim scalar with 1-second ramp for smooth engagement
 *          
 *          Timing Requirements:
 *          - Must be called at main loop rate (typically 400Hz)
 *          - Must execute between rate controller and servo output updates
 *          - Parameters take effect on next control iteration
 * 
 * @note Call this after attitude_control->rate_controller_run() and before motors->output()
 * @note Hover roll trim ramps over scheduler.get_loop_rate_hz() cycles (1 second)
 * 
 * @warning Incorrect call ordering can cause one-cycle delay in control parameter updates
 * @warning Do not call before rate controller or after servo outputs are calculated
 * 
 * @see check_dynamic_flight() for dynamic_flight flag computation
 * @see AC_AttitudeControl::use_leaky_i() for integrator mode control
 * @see AP_MotorsHeli::set_land_complete() for collective limit enforcement
 */
void Copter::update_heli_control_dynamics(void)
{
    // Configure leaky integrator usage based on motor library configuration
    // and current flight state (dynamic vs hover)
    if (!motors->using_leaky_integrator()) {
        // Motor library not configured for leaky integrators - use standard integrators always
        // This mode is selected when operator wants consistent integrator behavior
        attitude_control->use_leaky_i(false);
        
        // Communicate landing state to motor layer for collective management
        if (ap.land_complete || ap.land_complete_maybe) {
            motors->set_land_complete(true);
        } else {
            motors->set_land_complete(false);
        }
    } else {
        // Motor library configured for leaky integrators - adapt based on flight state
        // Use leaky integrators during hover/low-speed (!dynamic_flight) to prevent wind-up
        // Use standard integrators during dynamic flight for aggressive tracking
        attitude_control->use_leaky_i(!heli_flags.dynamic_flight);
        
        // Always indicate not landed when using dynamic integrator switching
        motors->set_land_complete(false);
    }

    // Update hover roll trim engagement scalar with 1-second ramp
    // Traditional helicopters require roll compensation during hover due to tail rotor thrust
    // Scalar ramps from 0 (disabled) to 1 (full trim) over 1 second after takeoff
    if (ap.land_complete || (is_zero(motors->get_desired_rotor_speed()))) {
        // if we are landed or there is no rotor power demanded, decrement slew scalar
        // Gradually disable trim to prevent uncommanded swashplate movement on ground
        hover_roll_trim_scalar_slew--;
    } else {
        // if we are not landed and motor power is demanded, increment slew scalar
        // Gradually enable trim after takeoff for smooth engagement
        hover_roll_trim_scalar_slew++;
    }
    // Constrain scalar to valid range [0, loop_rate_hz]
    // At loop_rate_hz cycles, scalar reaches 1.0 (full trim enabled)
    hover_roll_trim_scalar_slew = constrain_int16(hover_roll_trim_scalar_slew, 0, scheduler.get_loop_rate_hz());

    // Convert slewed counter to normalized scalar (0.0 to 1.0) and apply to attitude control
    // Formula: scalar = counter / loop_rate_hz
    // At 400Hz loop rate: 400 cycles = 1 second = full trim engagement
    attitude_control->set_hover_roll_trim_scalar((float) hover_roll_trim_scalar_slew/(float) scheduler.get_loop_rate_hz());
}

/**
 * @brief Determines if landing swashplate limits should be applied
 * 
 * @details Traditional helicopters require special collective pitch limiting during landing
 *          and low-speed flight to prevent blade strikes with the ground. This function
 *          determines when to engage these protective limits based on flight mode and state.
 *          
 *          Swashplate Range Limiting Rationale:
 *          - Full collective range: -12° to +12° blade pitch (typical)
 *          - Limited collective range: 0° to +12° blade pitch (landing mode)
 *          - Limiting prevents negative collective that could cause blade/tail strikes
 *          - Only applied in automated flight modes, not manual pilot control
 *          
 *          Traditional Helicopter Landing Considerations:
 *          - Unlike multirotors that simply reduce motor speed, helicopters must carefully
 *            manage collective pitch during landing to maintain rotor RPM
 *          - Too much negative collective during landing can cause:
 *            * Main rotor blade strikes with ground or tail boom
 *            * Tail rotor strikes with ground during tail-low attitudes
 *            * Loss of rotor speed and subsequent loss of control authority
 *          - Full swashplate authority needed for manual flight and aerobatics
 *          
 *          Decision Logic:
 *          - FULL range allowed: Manual modes, drift mode, inverted flight
 *          - LIMITED range enforced:
 *            * During landing sequence (active descent)
 *            * When on ground (land_complete)
 *            * Before takeoff (not auto_armed)
 *            * During hover/low-speed flight (!dynamic_flight)
 *          
 *          The conservative approach of limiting during hover ensures safety even if
 *          flight mode detection is delayed or incorrect.
 * 
 * @return true if landing swashplate limits should be applied (restricted collective range)
 * @return false if full swashplate range should be allowed (manual control or dynamic flight)
 * 
 * @note Manual throttle modes always get full range for pilot authority
 * @note Inverted flight requires full negative collective, always gets full range
 * 
 * @see heli_update_landing_swash() for application of this limit
 * @see AP_MotorsHeli::set_collective_for_landing() for limit implementation
 */
bool Copter::should_use_landing_swash() const
{
    if (flightmode->has_manual_throttle() ||
        flightmode->mode_number() == Mode::Number::DRIFT ||
        attitude_control->get_inverted_flight()) {
        // manual modes or modes using inverted flight uses full swash range
        // Pilot has direct control and may need full collective authority
        // Inverted flight requires negative collective, cannot limit range
        return false;
    }
    if (flightmode->is_landing()) {
        // landing with non-manual throttle mode always uses limit swash range
        // Automated landing sequences enforce collective limits for safety
        return true;
    }
    if (ap.land_complete) {
        // when landed in non-manual throttle mode limit swash range
        // Prevent uncommanded negative collective while on ground
        return true;
    }
    if (!ap.auto_armed) {
        // when waiting to takeoff in non-manual throttle mode limit swash range
        // Before auto-takeoff, restrict collective to prevent blade strikes
        return true;
    }
    if (!heli_flags.dynamic_flight) {
        // Just in case we are unsure of being in non-manual throttle
        // mode, limit swash range in low speed and hovering flight.
        // This will catch any non-manual throttle mode attempting a
        // landing and driving the collective too low before the land
        // complete flag is set.
        // Conservative approach: limit collective during hover for safety
        return true;
    }
    return false;
}

/**
 * @brief Updates swashplate collective limits based on landing state
 * 
 * @details Applies or removes collective pitch limits to prevent ground strikes during
 *          landing and low-speed flight. Also updates the collective stick position flag
 *          for monitoring pilot input.
 *          
 *          This function serves as the application point for landing-related collective
 *          restrictions determined by should_use_landing_swash().
 *          
 *          Traditional Helicopter Landing Swashplate Management:
 *          - Swashplate controls main rotor blade pitch via three or more servos
 *          - Collective: All servos move together to change overall blade pitch (thrust)
 *          - Cyclic: Servos move differentially to tilt rotor disk (directional control)
 *          - Landing mode limits minimum collective to prevent negative pitch
 *          
 *          Function Operations:
 *          1. Apply collective limits to motor control layer based on flight state
 *          2. Monitor throttle/collective stick position for pilot intent detection
 *          
 *          Timing Requirements:
 *          - Must be called after update_land_detector() to have current landing state
 *          - Should be called before motor output updates for current-cycle enforcement
 *          - Typically called in main loop at 400Hz
 * 
 * @note Call soon after update_land_detector() in main code for correct state
 * @note Collective limits are enforced in AP_MotorsHeli during servo output calculation
 * 
 * @see should_use_landing_swash() for limit decision logic
 * @see AP_MotorsHeli::set_collective_for_landing() for limit enforcement
 * @see update_collective_low_flag() for stick position monitoring
 */
void Copter::heli_update_landing_swash()
{
    // Apply or remove collective limits based on current flight state
    motors->set_collective_for_landing(should_use_landing_swash());
    
    // Update flag indicating if collective stick is at minimum position
    // Used for detecting pilot intent and autorotation conditions
    update_collective_low_flag(channel_throttle->get_control_in());
}

/**
 * @brief Reads pilot's desired rotor speed from motor interlock RC channel
 * 
 * @details Converts the motor interlock auxiliary switch position to a normalized rotor
 *          speed command (0.0 to 1.0). This provides direct pilot control over rotor RPM
 *          when using ROTOR_CONTROL_MODE_PASSTHROUGH governor mode.
 *          
 *          Traditional Helicopter Motor Interlock System:
 *          - Motor interlock is a safety mechanism that controls rotor engagement
 *          - Similar to a clutch - allows engine/motor to run without spinning main rotor
 *          - Essential safety feature: prevents inadvertent rotor engagement during startup
 *          - Different from multirotor arm/disarm: motor can run with rotor disengaged
 *          
 *          Rotor Speed Control in Traditional Helicopters:
 *          - Unlike multirotors that vary motor speed for control, helicopters maintain
 *            constant rotor RPM and vary blade pitch for thrust changes
 *          - Governor maintains target RPM by adjusting engine throttle or motor power
 *          - Pilot can directly control rotor speed via motor interlock channel
 *          - Typical operation: 0% = rotor off, 100% = full governed RPM
 *          
 *          RC Channel Interpretation:
 *          - Channel range set to 1000 (PWM units: 1000-2000 typical)
 *          - get_control_in() returns 0-1000 for the configured channel range
 *          - Conversion: PWM value / 1000 = normalized speed (0.0 to 1.0)
 *          - 0.0 = rotor off (interlock disengaged)
 *          - 1.0 = full rotor speed (interlock fully engaged)
 * 
 * @return Normalized rotor speed command (0.0 to 1.0)
 * @return 0.0 if motor interlock channel not configured
 * 
 * @note Only used in ROTOR_CONTROL_MODE_PASSTHROUGH mode
 * @note Does not directly control motor output - feeds into governor system
 * 
 * @see heli_update_rotor_speed_targets() for governor mode handling
 * @see AP_MotorsHeli::set_desired_rotor_speed() for speed command application
 */
float Copter::get_pilot_desired_rotor_speed() const
{
    // Find RC channel configured for motor interlock auxiliary function
    RC_Channel *rc_ptr = rc().find_channel_for_option(RC_Channel::AUX_FUNC::MOTOR_INTERLOCK);
    if (rc_ptr != nullptr) {
        // Set channel range to 1000 for normalized conversion
        rc_ptr->set_range(1000);
        // Convert channel input (0-1000) to normalized rotor speed (0.0-1.0)
        return (float)rc_ptr->get_control_in() * 0.001f;
    }
    // Return 0.0 if motor interlock channel not configured
    return 0.0f;
}

/**
 * @brief Updates rotor speed governor targets based on pilot input and control mode
 * 
 * @details Reads pilot input and flight controller state to determine desired main rotor
 *          speed, then passes the target to the motor control layer for governor execution.
 *          This is the primary rotor speed management function for traditional helicopters.
 *          
 *          Traditional Helicopter Rotor Speed Governor System:
 *          
 *          Unlike multirotors where motor speed varies continuously for control, traditional
 *          helicopters maintain constant main rotor RPM using a governor system. The governor
 *          automatically adjusts engine throttle or motor power to maintain target RPM despite:
 *          - Changing collective pitch loads (more pitch = more drag)
 *          - Varying flight conditions (wind, maneuvers, autorotation)
 *          - Altitude and temperature effects on engine power
 *          
 *          Why Constant Rotor RPM Matters:
 *          - Control authority depends on rotor speed (low RPM = sluggish response)
 *          - Blade lift is proportional to RPM squared (small RPM drop = large lift loss)
 *          - Gyroscopic stability requires consistent rotor momentum
 *          - Swashplate effectiveness diminishes at reduced RPM
 *          
 *          Rotor Speed Control (RSC) Modes:
 *          
 *          1. PASSTHROUGH Mode:
 *             - Pilot directly controls rotor speed via motor interlock channel
 *             - No automatic governor compensation
 *             - Used for: Manual rotor spool-up, basic flight, training
 *             - Pilot responsible for maintaining RPM with throttle adjustments
 *          
 *          2. SETPOINT Mode:
 *             - Fixed RPM target from parameter (H_RSC_SETPOINT)
 *             - Governor maintains this RPM automatically
 *             - Used for: Standard flight operations with consistent RPM
 *             - Most common mode for everyday flying
 *          
 *          3. THROTTLECURVE Mode:
 *             - RPM varies with collective pitch using configured curve
 *             - Anticipates load changes for improved governor response
 *             - Used for: Aerobatic flight, aggressive maneuvering
 *             - Curve typically: low collective = low RPM, high collective = high RPM
 *          
 *          4. AUTOTHROTTLE Mode:
 *             - Closed-loop governor using RPM sensor feedback
 *             - Most aggressive compensation for load changes
 *             - Used for: Precision flight, heavy lift, 3D aerobatics
 *             - Requires RPM sensor connected to flight controller
 *          
 *          Motor Interlock Safety:
 *          - Interlock must be engaged for rotor to spin (safety mechanism)
 *          - Prevents inadvertent rotor engagement during ground operations
 *          - When disengaged: rotor commanded to 0 RPM regardless of mode
 *          
 *          Governor Implementation:
 *          - Desired RPM set here, actual governor PID runs in AP_MotorsHeli
 *          - Governor adjusts motor output to compensate for RPM deviations
 *          - Smooth ramp-up/down prevents mechanical stress and control issues
 * 
 * @note Called at main loop rate (typically 400Hz) for responsive governor control
 * @note Governor PID execution happens in AP_MotorsHeli::output() after this function
 * 
 * @warning Rotor speed changes affect control authority - governor must maintain stable RPM
 * @warning Motor interlock state critically affects safety - verify switch configuration
 * 
 * @see get_pilot_desired_rotor_speed() for manual rotor speed input
 * @see AP_MotorsHeli::set_desired_rotor_speed() for governor target application
 * @see AP_MotorsHeli_Single::output() for governor PID implementation
 * @see AP_MotorsHeli::get_rsc_mode() for mode configuration
 */
void Copter::heli_update_rotor_speed_targets()
{
    // get rotor control method (RSC mode determines governor behavior)
    uint8_t rsc_control_mode = motors->get_rsc_mode();

    switch (rsc_control_mode) {
    case ROTOR_CONTROL_MODE_PASSTHROUGH:
        // PASSTHROUGH: Pilot directly controls rotor speed via motor interlock RC channel
        // No automatic governor - pilot manually maintains RPM
        if (get_pilot_desired_rotor_speed() > 0.01) {
            // Pilot commanding rotor engagement (interlock switch active)
            ap.motor_interlock_switch = true;
            motors->set_desired_rotor_speed(get_pilot_desired_rotor_speed());
        } else {
            // Pilot commanding rotor disengagement (interlock switch off)
            ap.motor_interlock_switch = false;
            motors->set_desired_rotor_speed(0.0f);  // Command rotor to stop
        }
        break;
        
    case ROTOR_CONTROL_MODE_SETPOINT:
    case ROTOR_CONTROL_MODE_THROTTLECURVE:
    case ROTOR_CONTROL_MODE_AUTOTHROTTLE:
        // AUTOMATED GOVERNOR MODES: Use configured setpoint or throttle curve
        // Governor automatically maintains target RPM
        if (motors->get_interlock()) {
            // Motor interlock engaged - use configured RSC setpoint
            // Setpoint determined by mode: fixed value, throttle curve, or closed-loop target
            motors->set_desired_rotor_speed(motors->get_rsc_setpoint());
        } else {
            // Motor interlock disengaged - command rotor to stop for safety
            motors->set_desired_rotor_speed(0.0f);
        }
        break;
    }

}


/**
 * @brief Detects and manages autorotation state for traditional helicopters
 * 
 * @details Determines if the helicopter is in autorotation (unpowered descent using rotor
 *          inertia) and configures motor control accordingly. Autorotation is a critical
 *          emergency procedure unique to traditional helicopters.
 *          
 *          Traditional Helicopter Autorotation Overview:
 *          
 *          Autorotation is an unpowered flight mode where the helicopter descends with
 *          rotor spinning due to upward airflow, similar to a falling maple seed. This
 *          provides a survivable emergency landing option when engine/motor fails.
 *          
 *          Physics of Autorotation:
 *          - Motor interlock disengaged = no power to main rotor
 *          - Descent causes upward airflow through rotor disk
 *          - Upward airflow drives rotor rotation (windmilling)
 *          - Pilot uses collective to manage rotor RPM and descent rate
 *          - Just before touchdown: increase collective to convert rotor energy to lift
 *          
 *          Why Autorotation is Unique to Traditional Helicopters:
 *          - Multirotors: Cannot autorotate, unpowered descent is uncontrolled fall
 *          - Traditional helicopters: Can control descent and landing without power
 *          - Main rotor acts as parachute/wing during descent
 *          - Requires specific blade design and rotor inertia
 *          
 *          Autorotation Detection Criteria:
 *          - Motor interlock MUST be disengaged (no power applied)
 *          - Must be airborne (not on ground or landing)
 *          - AND one of:
 *            * Manual throttle mode (pilot managing autorotation)
 *            * Dedicated autorotation flight mode active
 *          
 *          Control Adaptations During Autorotation:
 *          - Disable governor (would fight the windmilling)
 *          - Allow full negative collective for RPM management
 *          - Modify control laws for unpowered flight dynamics
 *          - Monitor rotor RPM to ensure sufficient energy for flare
 *          
 *          Autorotation vs Bailout:
 *          - Autorotation: Controlled descent without power (set_autorotation_active)
 *          - Bailout: Recovery from autorotation by re-engaging motor interlock
 *          - Smooth transition between states prevents control discontinuities
 * 
 * @note Called at main loop rate to continuously monitor autorotation conditions
 * @note Autorotation mode (MODE_AUTOROTATE) may not be compiled in all builds
 * 
 * @warning Incorrect autorotation detection can disable motor when power is needed
 * @warning Autorotation requires pilot training - not automatic landing capability
 * 
 * @see AP_MotorsHeli::set_autorotation_active() for motor control changes
 * @see Mode::Autorotate for dedicated autorotation flight mode implementation
 * @see AC_Autorotation library for assisted autorotation control
 */
void Copter::heli_update_autorotation()
{
    bool in_autorotation_mode = false;
#if MODE_AUTOROTATE_ENABLED
    // Check if dedicated autorotation flight mode is active
    in_autorotation_mode = flightmode == &mode_autorotate;
#endif

    // Safety check: Disable autorotation if on ground or about to land
    // Autorotation only applies to airborne flight - prevents mode confusion on ground
    if (ap.land_complete || ap.land_complete_maybe) {
        motors->force_deactivate_autorotation();
        return;
    }

    // if we got this far we are flying, check for conditions to set autorotation state
    // Autorotation requires: motor interlock OFF + (manual mode OR autorotate mode)
    if (!motors->get_interlock() && (flightmode->has_manual_throttle() || in_autorotation_mode)) {
        // Motor interlock disengaged and appropriate flight mode active
        // set state in motors to facilitate manual and assisted autorotations
        // Motor layer will disable governor and allow full collective range
        motors->set_autorotation_active(true);
    } else {
        // Not in autorotation conditions (motor engaged or wrong flight mode)
        // deactivate the autorotation state via the bailout case
        // Smooth transition back to powered flight if motor re-engaged
        motors->set_autorotation_active(false);
    }
}

/**
 * @brief Updates the collective stick low position flag with debouncing
 * 
 * @details Monitors the collective (throttle) stick position and sets a flag when the
 *          stick has been at minimum position for at least 400 milliseconds. This flag
 *          is used for detecting pilot intent during landing and autorotation.
 *          
 *          Traditional Helicopter Collective Stick Usage:
 *          - Collective stick controls main rotor blade pitch (thrust)
 *          - Full down = minimum/negative pitch (descent or landing)
 *          - Full up = maximum pitch (climb or hover)
 *          - Unlike multirotor throttle, collective doesn't directly control motor speed
 *          
 *          Why Monitor Collective Low Position:
 *          - Indicates pilot intent to land or reduce power
 *          - Used in autorotation detection (low collective during descent)
 *          - Helps determine if helicopter is attempting to land
 *          - Prevents false triggers during momentary stick movements
 *          
 *          Debounce Timing Rationale:
 *          - 400ms delay prevents false positives from transient stick inputs
 *          - Long enough: Filters out turbulence corrections and brief inputs
 *          - Short enough: Detects deliberate landing or power reduction
 *          - Immediate clear when stick raised: Responsive to pilot corrections
 *          
 *          Flag Usage in Control Logic:
 *          - Landing detection: Confirms pilot reducing collective for touchdown
 *          - Autorotation entry: Low collective + interlock off = autorotation
 *          - Governor management: May adjust RPM target when collective low
 *          - Collective limiting: Can influence swashplate range restrictions
 * 
 * @param[in] throttle_control Collective stick position in RC units (0 = minimum collective)
 * 
 * @note Debounce time: 400 milliseconds to set flag, immediate clear
 * @note Uses millisecond timer (millis()) for timing accuracy
 * @note Static variable last_nonzero_collective_ms persists between calls
 * 
 * @see heli_update_landing_swash() which calls this function
 * @see heli_update_autorotation() which may use this flag for state detection
 */
void Copter::update_collective_low_flag(int16_t throttle_control)
{
    // Static variable maintains state between function calls
    static uint32_t last_nonzero_collective_ms = 0;
    uint32_t tnow_ms = millis();

    if (throttle_control > 0) {
        // Collective stick is above minimum position
        // Update timestamp and immediately clear the low flag
        last_nonzero_collective_ms = tnow_ms;
        heli_flags.coll_stk_low = false;
    } else if (tnow_ms - last_nonzero_collective_ms > 400) {
        // Collective stick has been at minimum for more than 400ms
        // Set flag to indicate sustained low collective position
        heli_flags.coll_stk_low = true;
    }
    // Note: If stick at zero but less than 400ms, flag retains previous state
}

#endif  // FRAME_CONFIG == HELI_FRAME
