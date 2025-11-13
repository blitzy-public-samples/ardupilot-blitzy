/**
 * @file mode_hold.cpp
 * @brief Implementation of Hold mode for Rover
 * 
 * @details Hold mode stops the rover and attempts to maintain its current position.
 *          This mode provides active braking and position hold capabilities when GPS
 *          is available. The rover will:
 *          - Stop all forward/reverse motion (zero throttle)
 *          - Center steering to maintain current heading
 *          - For balance bots: maintain active balance control
 *          - For sailboats: relax sails to minimize wind forces
 * 
 *          Hold mode differs from Manual mode in that it actively attempts to maintain
 *          the current position using GPS position hold when available, rather than
 *          simply releasing controls.
 * 
 * @note Common use cases for Hold mode:
 *       - Pausing during autonomous missions
 *       - Emergency stop with position maintenance
 *       - Waiting for user input or external conditions
 *       - Safe mode during failsafe conditions
 * 
 * Source: Rover/mode_hold.cpp
 */

#include "Rover.h"

/**
 * @brief Main update function for Hold mode, called at the scheduler loop rate
 * 
 * @details This function implements the hold behavior by:
 *          1. Setting throttle to zero (or calculated balance value)
 *          2. Centering steering to hold current heading
 *          3. Relaxing sails for sailboat configurations
 *          4. Commanding motors to maintain position
 * 
 *          The hold implementation is vehicle-type aware:
 *          - Regular rovers: Zero throttle with active braking if supported
 *          - Balance bots: Active pitch control to maintain balance while stationary
 *          - Sailboats: Sails relaxed to reduce wind forces
 * 
 *          Unlike Manual mode which simply stops commanding the vehicle, Hold mode
 *          actively maintains position using GPS position hold when available through
 *          the motor controller's position hold capabilities.
 * 
 * @note This function is called at the main loop rate (typically 50Hz)
 * @note Position hold accuracy depends on GPS quality and motor controller capabilities
 * 
 * @warning For balance bots, this mode maintains active balance control - the vehicle
 *          will continue to self-balance even when "holding" position
 * 
 * Source: Rover/mode_hold.cpp:3-18
 */
void ModeHold::update()
{
    // Initialize throttle to zero - this will stop forward/reverse motion
    // For balance bots, this value will be overridden by pitch control below
    float throttle = 0.0f;

    // Balance bot special handling: calculate active throttle required to maintain balance
    // Even in hold mode, balance bots need continuous throttle adjustments to stay upright
    if (rover.is_balancebot()) {
        rover.balancebot_pitch_control(throttle);
    }

    // Sailboat special handling: relax mainsail to minimize wind forces
    // This prevents the wind from pushing the boat while trying to hold position
    g2.sailboat.relax_sails();

    // Command motors to hold position with calculated throttle and centered steering
    // The motor controller may implement additional position hold logic if GPS is available
    g2.motors.set_throttle(throttle);  // Zero throttle (or balance control value)
    g2.motors.set_steering(0.0f);       // Center steering to hold current heading
}
