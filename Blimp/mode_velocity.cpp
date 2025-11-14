/**
 * @file mode_velocity.cpp
 * @brief Velocity control flight mode implementation for Blimp airship vehicles
 * 
 * @details This file implements direct velocity control mode for lighter-than-air vehicles (Blimps).
 *          In this mode, pilot stick inputs are directly mapped to desired velocities rather than 
 *          positions or attitudes. This provides intuitive control for precise low-speed maneuvering
 *          of airships.
 *          
 *          Key characteristics of Velocity mode:
 *          - Direct velocity control: Pilot inputs map to target velocities in m/s
 *          - Coordinate frame handling: Supports both NED (North-East-Down) frame and body frame
 *          - Simple mode toggle: When simple_mode is enabled, inputs are in NED frame; when disabled, 
 *            inputs are in body frame and rotated to NED for control
 *          - Closed-loop control: Delegates to Loiter controller for actual velocity tracking
 *          - Suitable for: Precise positioning, low-speed maneuvering, indoor flight
 *          
 *          This mode differs from Loiter mode in that it provides velocity control rather than 
 *          position control, giving the pilot more direct control over vehicle motion.
 * 
 * @note Part of the Blimp vehicle implementation in ArduPilot
 * 
 * Source: Blimp/mode_velocity.cpp:1-25
 */

#include "Blimp.h"

#include <AP_Vehicle/AP_MultiCopter.h>

/**
 * @brief Main velocity controller for Velocity flight mode
 * 
 * @details This function implements the core velocity control algorithm for Blimp vehicles.
 *          It processes pilot inputs and commands the vehicle to track desired velocities in
 *          three translational axes (X, Y, Z) and yaw rotation.
 *          
 *          Algorithm flow:
 *          1. Get pilot input (normalized -1 to +1) via get_pilot_input()
 *          2. Scale inputs by maximum velocity parameters:
 *             - X/Y horizontal velocities scaled by max_vel_xy (typically m/s)
 *             - Z vertical velocity scaled by max_vel_z (typically m/s)
 *             - Yaw rate scaled by max_vel_yaw (typically deg/s)
 *          3. Coordinate frame transformation:
 *             - If simple_mode is disabled (g.simple_mode == 0): inputs are in body frame
 *               and must be rotated to NED (North-East-Down) frame via rotate_BF_to_NE()
 *             - If simple_mode is enabled: inputs are already in NED frame, no rotation needed
 *          4. Delegate to Loiter velocity controller for closed-loop tracking via run_vel()
 *          
 *          Coordinate frame handling:
 *          - Body frame: X=forward, Y=right, Z=down relative to vehicle heading
 *          - NED frame: X=North, Y=East, Z=Down in earth-fixed coordinates
 *          - Simple mode enables NED frame control for easier outdoor flight
 *          - Body frame control (simple_mode disabled) is more intuitive for indoor/FPV flight
 *          
 *          The Vector4b{false,false,false,false} parameter passed to run_vel() represents 
 *          reset flags for [x_reset, y_reset, z_reset, yaw_reset]. All false means we are
 *          continuously tracking velocities without resetting integrators.
 *          
 *          This mode provides velocity control as opposed to position control (Loiter mode),
 *          giving pilots more direct control over vehicle motion. The actual closed-loop
 *          velocity tracking is performed by the Loiter controller's run_vel() method,
 *          which implements PID control to achieve the commanded velocities.
 * 
 * @note Called at main loop rate (typically 50-100 Hz for Blimp)
 * @note Requires valid pilot input from RC transmitter or companion computer
 * @note Vehicle must be armed and in Velocity mode for this to execute
 * 
 * @warning Rapidly changing velocity commands can stress airship actuators and buoyancy control.
 *          Lighter-than-air vehicles have slower response dynamics than multirotors.
 * 
 * @see ModeVelocity::get_pilot_input() for input processing
 * @see Loiter::run_vel() for closed-loop velocity controller implementation
 * @see Blimp::rotate_BF_to_NE() for body-to-NED frame transformation
 */
void ModeVelocity::run()
{
    Vector3f target_vel;
    float target_vel_yaw;
    get_pilot_input(target_vel, target_vel_yaw);
    target_vel.x *= g.max_vel_xy;
    target_vel.y *= g.max_vel_xy;
    if (g.simple_mode == 0) {
        //If simple mode is disabled, input is in body-frame, thus needs to be rotated.
        blimp.rotate_BF_to_NE(target_vel.xy());
    }
    target_vel.z *= g.max_vel_z;
    target_vel_yaw *= g.max_vel_yaw;

    blimp.loiter->run_vel(target_vel, target_vel_yaw, Vector4b{false,false,false,false});
}
