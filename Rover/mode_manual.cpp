/**
 * @file mode_manual.cpp
 * @brief Implementation of Manual mode for Rover
 * 
 * @details Manual mode provides direct pass-through of pilot RC input to steering 
 *          and throttle outputs with optional speed-based scaling. This mode requires
 *          no GPS, no position estimation, and no autonomous stabilization - the pilot
 *          has complete direct control over the vehicle.
 * 
 *          Key characteristics:
 *          - Direct RC stick-to-motor mapping
 *          - No GPS or navigation algorithms active
 *          - No autonomous corrections or stabilization
 *          - Optional steering expo for smoother control feel
 *          - Optional speed-based steering scaling via MANUAL_OPTIONS parameter
 *          - Supports special vehicle types: balance bots, walking robots, sailboats
 * 
 *          This is the safest mode for initial vehicle testing and RC range checks,
 *          as vehicle behavior is entirely predictable from pilot input.
 * 
 * @note Pilot steering behavior can be configured via PILOT_STEER_TYPE parameter:
 *       - Default: Standard steering input
 *       - Two-paddle: Separate left/right paddle controls
 *       - Direction-reversed-when-reversing: Steering direction reverses in reverse
 * 
 * @warning Manual mode provides no autonomous safety features. Geofencing, obstacle
 *          avoidance, and automated failsafes are not active in this mode.
 * 
 * Source: Rover/mode_manual.cpp
 */

#include "Rover.h"

/**
 * @brief Clean up when exiting Manual mode
 * 
 * @details Clears lateral control output when transitioning away from Manual mode
 *          to ensure clean handoff to the next mode. This prevents lateral actuator
 *          commands from persisting into modes that don't use lateral control.
 */
void ModeManual::_exit()
{
    // clear lateral when exiting manual mode
    g2.motors.set_lateral(0);
}

/**
 * @brief Main update function for Manual mode - direct RC pass-through control
 * 
 * @details This function implements Manual mode by providing direct pass-through
 *          of pilot RC inputs to motor outputs with minimal processing. Manual mode
 *          is the simplest control mode, providing no GPS-based navigation, no
 *          autonomous stabilization, and no position holding.
 * 
 *          Control Flow:
 *          1. Read pilot RC stick inputs (steering, throttle, lateral)
 *          2. Apply optional steering expo for smoother feel
 *          3. Apply vehicle-type-specific processing (balance bot, walking robot, sailboat)
 *          4. Direct mapping: RC inputs → motor outputs
 * 
 *          This function is called at the main loop rate (typically 50Hz for Rover).
 * 
 * @note Manual mode behavior:
 *       - No GPS required - works even with GPS failure
 *       - No position estimation or navigation
 *       - No autonomous corrections or stabilization
 *       - Pilot has complete direct control
 *       - Ideal for initial testing and RC system verification
 */
void ModeManual::update()
{
    // STEP 1: Read pilot RC inputs
    // Get raw pilot stick inputs for steering and throttle from RC receiver
    // Values are in ArduPilot standard ranges: -4500 to 4500 for steering (centidegrees),
    // -100 to 100 for throttle (percentage)
    float desired_steering, desired_throttle, desired_lateral;
    get_pilot_desired_steering_and_throttle(desired_steering, desired_throttle);
    get_pilot_desired_lateral(desired_lateral);

    // STEP 2: Apply steering expo for smoother control feel
    // Steering expo reduces sensitivity near center stick while maintaining full range
    // at stick extremes. This gives pilots finer control for small adjustments.
    // MANUAL_STEERING_EXPO parameter: 0.0 = linear (no expo), higher values = more expo
    desired_steering = 4500.0 * input_expo(desired_steering / 4500, g2.manual_steering_expo);

    // STEP 3: Special vehicle type processing
    
    // Balance bot vehicles (two-wheeled self-balancing robots) require continuous
    // pitch control to maintain balance. Convert throttle demand into pitch target.
    if (rover.is_balancebot()) {
        rover.balancebot_pitch_control(desired_throttle);
    }

    // Walking robots (legged vehicles) support additional degrees of freedom:
    // roll, pitch, and walking height adjustments from pilot
    float desired_roll, desired_pitch, desired_walking_height;
    get_pilot_desired_roll_and_pitch(desired_roll, desired_pitch);
    get_pilot_desired_walking_height(desired_walking_height);
    g2.motors.set_roll(desired_roll);
    g2.motors.set_pitch(desired_pitch);
    g2.motors.set_walking_height(desired_walking_height);

    // Sailboat vehicles use RC input to control mainsail angle
    g2.sailboat.set_pilot_desired_mainsail();

    // STEP 4: Direct stick-to-motor mapping - copy RC inputs to motor outputs
    // This is the core of Manual mode: minimal processing, direct control
    
    // Throttle: Direct pass-through from pilot stick to motors
    g2.motors.set_throttle(desired_throttle);
    
    // Steering: Direct pass-through with optional speed-based scaling
    // MANUAL_OPTIONS bit flag SPEED_SCALING (bit 0): When enabled, reduces steering
    // authority at higher speeds to prevent aggressive turns that could destabilize vehicle.
    // This is useful for high-speed rovers where full steering at speed could cause rollover.
    // When disabled: Full steering range available at all speeds (default behavior)
    g2.motors.set_steering(desired_steering, (g2.manual_options & ManualOptions::SPEED_SCALING));
    
    // Lateral: For vehicles with lateral thrusters (e.g., omni-directional rovers)
    // Provides direct side-to-side control from pilot
    g2.motors.set_lateral(desired_lateral);
}
