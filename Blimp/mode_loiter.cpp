/**
 * @file mode_loiter.cpp
 * @brief Loiter (position hold) mode implementation for Blimp airships
 * 
 * @details This file implements the loiter flight mode for lighter-than-air vehicles,
 *          providing position-hold functionality with pilot input integration.
 *          
 *          The loiter controller maintains the vehicle's position while allowing
 *          the pilot to adjust the target position through stick inputs. Key features:
 *          - Coordinate frame handling: Body-frame inputs converted to NED frame
 *          - Lag limiting: Prevents target position from getting too far ahead of
 *            actual position, critical for airship dynamics with slow response
 *          - Smooth position updates respecting vehicle capabilities
 *          
 *          Coordinate Frame Convention: NED (North-East-Down) for position control
 * 
 * @note This is part of the Blimp vehicle implementation for ArduPilot
 */
#include "Blimp.h"

/**
 * @brief Position lag limit in seconds for loiter mode
 * 
 * @details This constant defines the maximum time (in seconds) that the target
 *          position can be ahead of the actual position during loiter mode.
 *          
 *          The lag limiting prevents the target position from "running away" when
 *          pilot inputs are applied continuously. This is critical for lighter-than-air
 *          vehicles (airships/blimps) which have:
 *          - Slow response dynamics due to buoyancy and low thrust-to-weight ratio
 *          - Difficulty catching up to rapidly moving targets
 *          - Risk of instability if commanded positions exceed achievable rates
 *          
 *          With POS_LAG = 1 second, the target position is only updated if it's
 *          within (max_rate * POS_LAG) of the current position. This creates a
 *          "moving window" that follows the vehicle while allowing position adjustments.
 * 
 * @note This value is tuned specifically for airship flight characteristics
 */
#define POS_LAG 1

/**
 * @brief Initialize loiter mode by capturing current position and yaw as targets
 * 
 * @details This function is called when the vehicle transitions into loiter mode.
 *          It initializes the loiter controller by setting:
 *          - target_pos: Current NED position from navigation system
 *          - target_yaw: Current yaw angle in radians from AHRS
 *          
 *          By capturing the current state, the vehicle will attempt to hold its
 *          position at the point where loiter mode was engaged. This provides
 *          smooth mode transitions without sudden position corrections.
 * 
 * @param[in] ignore_checks Standard mode initialization parameter (unused in loiter)
 *                          Allows mode to be entered even if pre-arm checks fail
 * 
 * @return true Always returns true - loiter mode initialization always succeeds
 * 
 * @note This captures the instantaneous position/yaw as the initial loiter target
 * @note Called automatically by the mode manager during mode transitions
 */
bool ModeLoiter::init(bool ignore_checks)
{
    target_pos = blimp.pos_ned;
    target_yaw = blimp.ahrs.get_yaw_rad();

    return true;
}

/**
 * @brief Main loiter controller executed at each control loop iteration
 * 
 * @details This function implements the core loiter (position hold) algorithm for airships.
 *          The controller maintains position while allowing pilot inputs to adjust targets.
 *          
 *          Algorithm sequence:
 *          1. Get pilot input (normalized -1 to 1 from sticks)
 *          2. Scale pilot input by maximum rates and delta time to get position deltas
 *          3. If simple_mode disabled, rotate body-frame inputs to NED frame
 *          4. Apply lag-limited position updates (only update if within lag threshold)
 *          5. Call loiter position controller with updated targets
 *          
 *          Lag Limiting Logic:
 *          For each axis (X, Y, Z, Yaw), the target is only updated if the current
 *          error (distance between target and actual position) is less than the lag
 *          threshold (max_rate * POS_LAG seconds). This prevents the target from
 *          getting unreachably far ahead for slow-responding airship dynamics.
 *          
 *          Coordinate Frames:
 *          - Pilot input: Body frame if simple_mode=0, NED frame if simple_mode=1
 *          - target_pos: NED frame (North-East-Down in meters)
 *          - target_yaw: Radians, wrapped to [-PI, PI]
 *          
 *          Units:
 *          - Position: meters (NED frame)
 *          - Rates: meters/second (g.max_pos_xy, g.max_pos_z)
 *          - Yaw: radians, yaw rate in radians/second (g.max_pos_yaw)
 *          - Time: seconds (dt = delta time)
 * 
 * @note Executed at main loop rate (typically 50-100Hz for airships)
 * @note Coordinate frame: NED (North-East-Down) convention used throughout
 * @note The Vector4b{false,false,false,false} parameter in loiter->run() disables
 *       individual axis resets - all axes track their targets continuously
 * 
 * @warning This is a flight-critical function - modifications affect position holding
 *          and could lead to loss of vehicle control if lag limiting is removed
 */
void ModeLoiter::run()
{
    // Get loop delta time for rate scaling
    const float dt = blimp.scheduler.get_last_loop_time_s();

    // Get pilot stick inputs (normalized -1 to 1) and scale by maximum rates
    Vector3f pilot;
    float pilot_yaw;
    get_pilot_input(pilot, pilot_yaw);
    
    // Convert pilot input rates to position deltas (rate * time = distance)
    pilot.x *= g.max_pos_xy * dt;  // North position delta (meters)
    pilot.y *= g.max_pos_xy * dt;  // East position delta (meters)
    pilot.z *= g.max_pos_z * dt;   // Down position delta (meters)
    pilot_yaw *= g.max_pos_yaw * dt; // Yaw delta (radians)

    // Coordinate frame transformation: Body frame → NED frame
    if (g.simple_mode == 0) {
        // If simple mode is disabled, input is in body-frame, thus needs to be rotated
        // to NED (North-East-Down) frame for position control
        blimp.rotate_BF_to_NE(pilot.xy());
    }

    // Lag-limited position update: Only update target if within catchable distance
    // This prevents target position runaway for slow airship dynamics
    
    // X-axis (North): Update target only if error < (max_rate * lag_time)
    if (fabsf(target_pos.x-blimp.pos_ned.x) < (g.max_pos_xy*POS_LAG)) {
        target_pos.x += pilot.x;
    }
    
    // Y-axis (East): Update target only if error < (max_rate * lag_time)
    if (fabsf(target_pos.y-blimp.pos_ned.y) < (g.max_pos_xy*POS_LAG)) {
        target_pos.y += pilot.y;
    }
    
    // Z-axis (Down): Update target only if error < (max_rate * lag_time)
    if (fabsf(target_pos.z-blimp.pos_ned.z) < (g.max_pos_z*POS_LAG)) {
        target_pos.z += pilot.z;
    }
    
    // Yaw: Update target only if error < (max_rate * lag_time), wrapped to [-PI, PI]
    if (fabsf(wrap_PI(target_yaw-ahrs.get_yaw_rad())) < (g.max_pos_yaw*POS_LAG)) {
        target_yaw = wrap_PI(target_yaw + pilot_yaw);
    }

    // Execute loiter position controller with updated targets
    // Vector4b{false,false,false,false} = no axis resets, all axes track targets
    blimp.loiter->run(target_pos, target_yaw, Vector4b{false,false,false,false});
}
