/**
 * @file mode_stabilize.cpp
 * @brief Implementation of STABILIZE flight mode for fixed-wing aircraft
 * 
 * @details This file implements the STABILIZE flight mode which provides attitude
 *          stabilization with pilot input mixing for fixed-wing aircraft. STABILIZE
 *          mode is the primary manual flight mode that assists the pilot by maintaining
 *          stable attitude while allowing direct control authority through stick inputs.
 *          
 *          Unlike autonomous navigation modes, STABILIZE does not follow waypoints or
 *          execute mission commands. Instead, it provides:
 *          - Roll axis stabilization with pilot input
 *          - Pitch axis stabilization with pilot input
 *          - Yaw axis stabilization
 *          - Direct throttle pass-through from pilot
 *          
 *          This mode is typically used for:
 *          - Manual flight with stabilization assistance
 *          - Initial flight testing of new aircraft
 *          - Recovery from autonomous mode failures
 *          - Training and skill development
 * 
 * @note This is one of the core flight modes and is considered safety-critical
 * @warning Proper tuning of stabilization parameters is essential for safe flight
 * 
 * @see ModeStabilize class definition in mode.h
 * @see Plane::stabilize_roll(), Plane::stabilize_pitch(), Plane::stabilize_yaw()
 */

#include "mode.h"
#include "Plane.h"

/**
 * @brief Update navigation targets for STABILIZE mode
 * 
 * @details Clears the navigation roll and pitch targets (nav_roll_cd and nav_pitch_cd)
 *          since STABILIZE mode does not use navigation waypoints or autonomous guidance.
 *          This mode only provides attitude stabilization based on direct pilot inputs,
 *          not navigation to predetermined targets.
 *          
 *          The navigation targets are set to zero to ensure that any navigation-based
 *          control loops do not interfere with the pilot's direct control authority.
 *          This is essential for proper separation between manual and autonomous control.
 * 
 * @note Called once per main loop iteration before run() is executed
 * @note nav_roll_cd and nav_pitch_cd are in centidegrees
 * 
 * @see ModeStabilize::run() for the main control execution
 */
void ModeStabilize::update()
{
    plane.nav_roll_cd = 0;
    plane.nav_pitch_cd = 0;
}

/**
 * @brief Main control loop execution for STABILIZE mode
 * 
 * @details Executes the complete attitude stabilization sequence for STABILIZE mode.
 *          This is the primary flight mode for manual flying with stabilization assistance,
 *          providing a stable and predictable aircraft response to pilot inputs.
 *          
 *          The control sequence is executed in the following order:
 *          1. stabilize_roll() - Computes roll axis stabilization with pilot input mixing
 *          2. stabilize_pitch() - Computes pitch axis stabilization with pilot input mixing
 *          3. stabilize_stick_mixing_direct() - Applies direct mixing of pilot stick inputs
 *                                               to control surfaces for immediate response
 *          4. stabilize_yaw() - Computes yaw axis stabilization (coordinated turns)
 *          5. output_pilot_throttle() - Passes through throttle command directly from pilot
 *                                        without modification or limiting
 *          
 *          The stabilization functions combine:
 *          - Pilot stick inputs (desired rates/angles)
 *          - Current aircraft attitude (from AHRS/EKF)
 *          - PID controllers tuned for the specific airframe
 *          - Control surface deflection limits
 *          
 *          This results in servo commands that maintain stable flight while responding
 *          predictably to pilot commands.
 * 
 * @note This is the primary flight mode for manual flying with stabilization assist
 * @note Called at main loop rate (typically 50-400Hz depending on board and scheduler configuration)
 * @note Throttle is passed through directly without governor or limiting in this mode
 * 
 * @warning Proper PID tuning is critical for safe flight - poorly tuned parameters can
 *          result in oscillations, instability, or loss of control
 * @warning This mode does not provide altitude hold, position hold, or waypoint navigation
 * 
 * @see Plane::stabilize_roll() for roll axis control implementation
 * @see Plane::stabilize_pitch() for pitch axis control implementation
 * @see ModeStabilize::stabilize_stick_mixing_direct() for direct stick mixing
 * @see Plane::stabilize_yaw() for yaw axis control implementation
 * @see Mode::output_pilot_throttle() for throttle pass-through implementation
 */
void ModeStabilize::run()
{
    plane.stabilize_roll();
    plane.stabilize_pitch();
    stabilize_stick_mixing_direct();
    plane.stabilize_yaw();

    output_pilot_throttle();
}
