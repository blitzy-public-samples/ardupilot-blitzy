/**
 * @file balance_bot.cpp
 * @brief Implementation of balance bot (self-balancing robot) control for Rover
 * 
 * @details This file implements the control system for self-balancing robots (balance bots)
 *          that maintain upright stability using pitch angle feedback. The balance bot uses
 *          a two-wheeled inverted pendulum configuration where the vehicle must continuously
 *          adjust wheel speed to maintain balance.
 *          
 *          Control Architecture:
 *          - Pitch angle feedback from AHRS/IMU provides vehicle tilt angle
 *          - PID controller converts pitch error to throttle commands
 *          - Throttle commands drive both wheels to counteract tipping
 *          - Integration with velocity and yaw rate control for navigation
 *          
 *          The balance control runs in the main scheduler loop and works in conjunction
 *          with the steering controller to provide full 2D motion control while maintaining
 *          upright stability.
 *          
 * @note Requires frame type FRAME_BALANCEBOT to be configured via FRAME_CLASS parameter
 * @note Balance bots require constant active control - vehicle will fall if disarmed
 * 
 * @see Rover::is_balancebot()
 * @see AP_MotorsUGV::output()
 * 
 * Source: Rover/balance_bot.cpp
 */

#include<stdio.h>
#include "Rover.h"

/**
 * @brief Calculate and apply pitch angle control for balance bot stability
 * 
 * @details This function implements the core balance control loop for self-balancing robots.
 *          It converts pilot throttle input into a desired pitch angle, then uses a PID
 *          controller to generate the throttle output needed to achieve that pitch angle.
 *          
 *          Control Loop Operation:
 *          1. Throttle Input → Desired Pitch Angle
 *             - Forward throttle demands forward pitch (vehicle leans forward to accelerate)
 *             - Reverse throttle demands backward pitch (vehicle leans back to decelerate)
 *             - Scaled by BAL_PITCH_MAX parameter for maximum lean angle
 *             - Offset by BAL_PITCH_TRIM for calibration adjustments
 *          
 *          2. Pitch Error Calculation
 *             - Current pitch angle from AHRS compared to demanded pitch
 *             - Error fed into attitude PID controller
 *          
 *          3. PID Control for Pitch Stabilization
 *             - P term: Proportional response to current pitch error
 *             - I term: Eliminates steady-state pitch offset errors
 *             - D term: Damping to prevent oscillations
 *             - Output saturation aware (respects motor throttle limits)
 *          
 *          4. Throttle Output
 *             - PID output is throttle percentage needed to maintain/achieve pitch
 *             - Applied equally to both wheels (differential steering handled separately)
 *          
 *          Integration with Navigation:
 *          - Velocity control: Desired speed creates forward/backward pitch demand
 *          - Yaw rate control: Handled independently via differential steering
 *          - Position control: Generates velocity commands that become pitch demands
 *          
 *          This function is called from the main control loop at the scheduler rate
 *          (typically 50Hz) whenever the vehicle is configured as a balance bot.
 * 
 * @param[in,out] throttle Pilot throttle input (-100 to 100), replaced with balance control output
 *                         Input: Desired velocity/acceleration as throttle percentage
 *                         Output: Required throttle to maintain pitch angle for that velocity
 * 
 * @note Throttle is both input and output - input value is used to calculate desired pitch,
 *       then replaced with the control output needed to achieve that pitch
 * @note Called at main loop rate (typically 50Hz) when is_balancebot() returns true
 * @warning Balance control requires continuous execution - vehicle will fall if disabled
 * @warning Incorrect PID tuning can cause violent oscillations or instability
 * 
 * @see Rover::is_balancebot()
 * @see AC_AttitudeControl_BalanceBot::get_throttle_out_from_pitch()
 * 
 * Source: Rover/balance_bot.cpp:5-12
 */
void Rover::balancebot_pitch_control(float &throttle)
{
    // Calculate desired pitch angle from throttle input for balance control
    // Forward throttle (positive) creates negative pitch demand (lean forward to accelerate)
    // Reverse throttle (negative) creates positive pitch demand (lean back to brake/reverse)
    // Scale: throttle * 0.01 converts percentage to decimal, then multiply by BAL_PITCH_MAX (degrees)
    // Trim offset: BAL_PITCH_TRIM compensates for CG offset or sensor calibration errors
    // Result: demanded_pitch in radians represents target lean angle to achieve desired motion
    const float demanded_pitch = radians(-throttle * 0.01f * g2.bal_pitch_max) + radians(g2.bal_pitch_trim);

    // Apply PID pitch stabilization control to maintain desired pitch angle
    // PID controller compares demanded_pitch vs actual pitch from AHRS
    // Outputs throttle percentage needed to achieve/maintain target pitch angle
    // Parameters:
    //   - demanded_pitch: target pitch angle (radians) calculated above
    //   - radians(g2.bal_pitch_max): maximum allowed pitch angle for saturation limiting
    //   - motor limit flags: indicates if motors are already saturated (affects integral windup)
    //   - G_Dt: loop delta time for correct derivative/integral calculations
    // Returns: throttle output as decimal (0.0 to 1.0), converted to percentage (*100.0f)
    // This throttle value maintains the inverted pendulum balance while achieving velocity control
    throttle = g2.attitude_control.get_throttle_out_from_pitch(demanded_pitch, radians(g2.bal_pitch_max), g2.motors.limit.throttle_lower || g2.motors.limit.throttle_upper, G_Dt) * 100.0f;
}

/**
 * @brief Check if vehicle is configured as a balance bot (self-balancing robot)
 * 
 * @details This function determines whether the rover is operating in balance bot mode
 *          by checking the FRAME_CLASS parameter configuration. Balance bots are
 *          two-wheeled inverted pendulum vehicles that require active pitch control
 *          to maintain upright stability.
 *          
 *          When balance bot mode is enabled:
 *          - Pitch angle feedback control is active (balancebot_pitch_control)
 *          - Throttle output controls pitch angle for balance maintenance
 *          - Both wheels receive same base throttle for pitch control
 *          - Differential steering is applied on top of balance throttle for yaw control
 *          - Vehicle requires continuous active control to remain upright
 *          
 *          This function is called by the motor output system to determine control mode.
 *          When true, motor output uses pitch-based throttle control instead of direct
 *          velocity control. The balance control integrates with navigation by converting
 *          desired velocity into pitch angle demands.
 * 
 * @return true if FRAME_CLASS parameter is set to FRAME_BALANCEBOT
 * @return false if configured as standard rover (skid-steer, Ackermann, etc.)
 * 
 * @note Requires FRAME_CLASS parameter set to FRAME_BALANCEBOT (frame type 3)
 * @note Balance bot frame type fundamentally changes vehicle control behavior
 * @warning Switching frame types requires complete re-tuning of control parameters
 * @warning Balance bots will fall over immediately if disarmed or control disabled
 * 
 * @see Rover::balancebot_pitch_control()
 * @see AP_MotorsUGV::output()
 * @see Parameters: g2.frame_class (FRAME_CLASS)
 * 
 * Source: Rover/balance_bot.cpp:14-20
 */
bool Rover::is_balancebot() const
{
    // Check if frame class parameter matches balance bot configuration
    // FRAME_BALANCEBOT enum value indicates two-wheeled inverted pendulum vehicle
    // This determines fundamental control strategy: pitch control vs direct velocity control
    return ((enum frame_class)g2.frame_class.get() == FRAME_BALANCEBOT);
}
