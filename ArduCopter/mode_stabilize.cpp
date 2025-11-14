/**
 * @file mode_stabilize.cpp
 * @brief Stabilize flight mode implementation for multicopter vehicles
 * 
 * @details This file implements Stabilize mode, the primary manual flight mode for ArduPilot
 *          multirotors. Stabilize mode provides rate-controlled self-leveling flight where:
 *          
 *          - Pilot stick inputs command lean angles (roll/pitch) and yaw rate
 *          - The autopilot actively maintains the commanded attitude
 *          - When sticks are centered, the vehicle automatically levels itself
 *          - Throttle is directly passed through without altitude hold
 *          
 *          Stabilize is the recommended mode for manual flight and is the default mode for
 *          most multicopter configurations. It provides stable, predictable handling while
 *          still requiring active pilot throttle control.
 *          
 *          Key Characteristics:
 *          - Self-leveling: Vehicle returns to level when sticks centered
 *          - Rate-controlled: Stick deflection controls rate of attitude change
 *          - Manual throttle: No altitude hold - pilot controls climb/descent
 *          - No GPS required: Works with only IMU and barometer
 *          - Safe for beginners: Prevents excessive lean angles
 *          
 * @note This file implements the standard multicopter variant. Helicopter-specific
 *       implementation is in mode_stabilize_heli.cpp
 *       
 * @see AC_AttitudeControl for the underlying attitude control implementation
 * @see Mode base class for common flight mode functionality
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#include "Copter.h"

/**
 * @brief Initialization function for Stabilize mode
 * 
 * @details ModeStabilize does not implement a custom init() function and instead
 *          uses the base Mode class initialization (Mode::init()). This is appropriate
 *          because Stabilize mode has no special initialization requirements beyond
 *          the standard mode initialization that sets up basic state.
 *          
 *          The base Mode::init() handles:
 *          - Setting mode flags and state
 *          - Resetting mode-specific counters
 *          - Initializing common mode infrastructure
 *          
 *          Stabilize mode's simplicity (no GPS, no altitude hold, no autonomous navigation)
 *          means no additional setup is needed beyond what the base class provides.
 *          
 * @note Helicopter variant (ModeStabilize_Heli) does override init() to handle
 *       collective pitch and other helicopter-specific setup. See mode_stabilize_heli.cpp
 *       
 * @see Mode::init() Base class initialization implementation
 * @see ModeStabilize_Heli::init() Helicopter-specific initialization
 */
// Note: ModeStabilize::init() is not defined here - uses base Mode::init()

/**
 * @brief Run the Stabilize mode main control loop
 * 
 * @details This function implements the core control logic for Stabilize mode, executing at
 *          the main loop rate (typically 400Hz). It processes pilot inputs and commands the
 *          attitude controller to maintain the desired vehicle orientation.
 *          
 *          Control Flow:
 *          1. Apply simple mode transformation (if enabled) to pilot inputs
 *          2. Convert pilot stick positions to target lean angles (roll/pitch)
 *          3. Convert pilot yaw stick to target yaw rate
 *          4. Manage motor spool state based on arming and throttle
 *          5. Command attitude controller with desired angles and rates
 *          6. Pass through pilot throttle directly to motors
 *          
 *          Stick-to-Angle Mapping:
 *          - Full stick deflection = ANGLE_MAX parameter (default ±45°)
 *          - Linear mapping between stick position and lean angle
 *          - Exponential curves can be applied via RC_FEEL parameter
 *          - Maximum lean angles enforced by attitude controller
 *          
 *          Attitude Control Integration:
 *          - Roll/pitch inputs → desired lean angles → attitude quaternion
 *          - Yaw input → desired rotation rate → integrates to heading
 *          - Attitude controller runs PID loops to achieve commanded attitude
 *          - Motor mixing translates attitude commands to individual motor outputs
 *          
 *          Throttle Behavior:
 *          - Direct passthrough from pilot stick to motors
 *          - No altitude compensation or hold
 *          - Pilot has full manual control of vertical speed
 *          - Zero throttle detection for landing and disarming logic
 *          
 *          Motor Spool State Management:
 *          - SHUT_DOWN: Motors stopped, controllers reset
 *          - GROUND_IDLE: Motors at idle, preparing for takeoff or landed
 *          - SPOOLING_UP/DOWN: Transitioning between states
 *          - THROTTLE_UNLIMITED: Full flight mode, throttle control active
 *          
 * @note This function must be called at consistent high rate (100Hz minimum, 400Hz typical)
 *       for stable control performance
 *       
 * @warning Modifying the attitude controller parameters or spool state logic can affect
 *          vehicle stability and safety. Changes should be tested in SITL before flight.
 *          
 * @see ModeStabilize::init() Uses base Mode class initialization (no custom init required)
 * @see get_pilot_desired_lean_angles_rad() Converts stick input to angle commands
 * @see get_pilot_desired_yaw_rate_rads() Converts yaw stick to rate command
 * @see AC_AttitudeControl::input_euler_angle_roll_pitch_euler_rate_yaw_rad() Attitude command API
 */
void ModeStabilize::run()
{
    // ========================================================================
    // STEP 1: Process Pilot Inputs to Target Attitude Commands
    // ========================================================================
    
    // Apply simple mode transformation to pilot inputs (if enabled)
    // Simple mode rotates pilot inputs to be relative to the initial heading,
    // making the vehicle easier to control for beginners by providing
    // "headless" control where forward stick always moves forward relative
    // to the pilot's perspective regardless of vehicle yaw.
    update_simple_mode();

    // Convert pilot stick positions to target lean angles (roll and pitch)
    // This is the core of Stabilize mode's self-leveling behavior:
    // - Stick centered (0) → target angle = 0° (level flight)
    // - Stick deflection → proportional lean angle up to ANGLE_MAX (default ±45°)
    // - The attitude controller will actively maintain these commanded angles
    // - When pilot releases sticks, vehicle automatically returns to level
    float target_roll_rad, target_pitch_rad;
    get_pilot_desired_lean_angles_rad(target_roll_rad, target_pitch_rad, attitude_control->lean_angle_max_rad(), attitude_control->lean_angle_max_rad());

    // Convert pilot yaw stick position to target yaw rate (rotation rate in rad/s)
    // Unlike roll/pitch which command angles, yaw commands a rotation rate:
    // - Stick centered → 0 rad/s (hold current heading)
    // - Stick deflection → proportional rotation rate
    // - This provides intuitive heading control without GPS
    float target_yaw_rate_rads = get_pilot_desired_yaw_rate_rads();

    // ========================================================================
    // STEP 2: Determine Motor Spool State Based on Arming and Throttle
    // ========================================================================
    
    // Motor spool state manages the transition between stopped, idle, and flying states.
    // This is critical for:
    // - Safe motor startup (prevents sudden throttle application)
    // - Landing detection and safety
    // - Controller integrator management (prevents windup when not flying)
    
    if (!motors->armed()) {
        // Motors are disarmed - command complete shutdown
        // This is the safe ground state where motors will not spin
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
    } else if (copter.ap.throttle_zero
               || (copter.air_mode == AirMode::AIRMODE_ENABLED && motors->get_spool_state() == AP_Motors::SpoolState::SHUT_DOWN)) {
        // Pilot has throttle at zero (or air mode special case)
        // Command GROUND_IDLE state:
        // - Motors spin at idle speed for immediate response
        // - Indicates vehicle is on ground or attempting to land
        // - Allows smooth transition to flight when throttle applied
        // 
        // Note: In air mode, throttle_zero is never true while flying, but we still
        // allow motors to go through ground idle during the spool-up sequence for
        // smooth motor start behavior
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
    } else {
        // Normal flight condition: throttle above zero and armed
        // Command THROTTLE_UNLIMITED state:
        // - Motors respond directly to throttle and attitude commands
        // - Full attitude control authority available
        // - This is the active flying state for Stabilize mode
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    }

    // ========================================================================
    // STEP 3: Read Pilot Throttle and Manage Controller State per Spool State
    // ========================================================================
    
    // Get pilot's desired throttle (0.0 to 1.0 range)
    // In Stabilize mode, this is passed directly through to motors without
    // altitude compensation - pilot has full manual control of climb/descent rate.
    // This is what distinguishes Stabilize from altitude-holding modes like AltHold.
    float pilot_desired_throttle = get_pilot_desired_throttle();

    // Handle controller state based on current motor spool state
    // This ensures controllers are properly initialized/reset during state transitions
    // and prevents integrator windup when the vehicle is not flying
    switch (motors->get_spool_state()) {
    case AP_Motors::SpoolState::SHUT_DOWN:
        // Motors are completely stopped (disarmed or commanded off)
        // Reset all controller states for clean startup:
        // - Reset yaw target to current heading (prevents sudden yaw on takeoff)
        // - Reset PID integrators immediately (clear any accumulated errors)
        // - Force throttle to zero (safety - no motor output)
        attitude_control->reset_yaw_target_and_rate();
        attitude_control->reset_rate_controller_I_terms();
        pilot_desired_throttle = 0.0f;
        break;

    case AP_Motors::SpoolState::GROUND_IDLE:
        // Motors at idle, vehicle on ground (landed or landing)
        // Reset controllers more gently for smooth transition to flight:
        // - Reset yaw target to prevent unexpected rotation on takeoff
        // - Reset integrators smoothly (prevents control bump during takeoff)
        // - Force throttle to zero (motors at idle, not responding to pilot input yet)
        attitude_control->reset_yaw_target_and_rate();
        attitude_control->reset_rate_controller_I_terms_smoothly();
        pilot_desired_throttle = 0.0f;
        break;

    case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
        // Normal flight state - full control active
        // Clear landing detection flag if we have sufficient throttle authority
        // (motors not hitting lower limit), indicating we are definitely flying
        if (!motors->limit.throttle_lower) {
            set_land_complete(false);
        }
        break;

    case AP_Motors::SpoolState::SPOOLING_UP:
    case AP_Motors::SpoolState::SPOOLING_DOWN:
        // Transitioning between spool states
        // Do nothing - let the motor library manage the ramp up/down
        // Controller states will be handled when we reach the target state
        break;
    }

    // ========================================================================
    // STEP 4: Command Attitude Controller with Desired Orientation
    // ========================================================================
    
    // Send pilot's desired attitude to the attitude controller
    // This is where the pilot inputs (stick positions) are translated into
    // commanded vehicle orientation for the inner control loops.
    //
    // Parameters:
    // - target_roll_rad: Desired roll angle in radians (negative = right, positive = left)
    // - target_pitch_rad: Desired pitch angle in radians (negative = forward, positive = back)
    // - target_yaw_rate_rads: Desired yaw rotation rate in rad/s (not an angle!)
    //
    // The attitude controller will:
    // 1. Convert the Euler angle commands to a desired attitude quaternion
    // 2. Calculate the error between desired and actual attitude
    // 3. Run PID controllers on attitude error to generate rate targets
    // 4. Run rate PID controllers to generate motor torque commands
    // 5. Send commands to motor mixer to calculate individual motor outputs
    //
    // This multi-layer control structure provides:
    // - Stable attitude hold when sticks centered (outer loop)
    // - Disturbance rejection from wind/turbulence (integral terms)
    // - Smooth response to pilot inputs (derivative terms + feed-forward)
    // - Rate limiting to prevent aggressive maneuvers beyond vehicle limits
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_rad(target_roll_rad, target_pitch_rad, target_yaw_rate_rads);

    // ========================================================================
    // STEP 5: Output Throttle Command (Direct Passthrough)
    // ========================================================================
    
    // Send pilot's throttle directly to motors
    // This is a key characteristic of Stabilize mode: NO altitude hold
    //
    // Parameters:
    // - pilot_desired_throttle: Raw pilot input (0.0 to 1.0)
    // - true: Apply angle boost (increases throttle when leaned to compensate for thrust vector)
    // - g.throttle_filt: Low-pass filter to smooth throttle changes
    //
    // Throttle Behavior:
    // - Direct mapping from pilot stick to motor collective thrust
    // - No altitude compensation or hold - pilot must actively control height
    // - Angle boost automatically increases throttle in aggressive maneuvers to prevent altitude loss
    // - Hover throttle learning does not affect this mode (used by altitude modes only)
    //
    // Why No Altitude Hold:
    // - Simpler behavior - more predictable for pilots
    // - Works without barometer or altitude sensors
    // - Gives pilot direct feel for throttle-to-climb relationship
    // - Preferred for acrobatic flying where precise altitude control not needed
    // - Foundation mode - pilots learn throttle control before using autonomous altitude modes
    attitude_control->set_throttle_out(pilot_desired_throttle, true, g.throttle_filt);
}
