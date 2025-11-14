/**
 * @file Loiter.cpp
 * @brief Loiter controller implementation for blimp airship position and velocity control
 * 
 * @details This file implements the loiter control system for lighter-than-air blimp vehicles,
 *          providing both position hold and velocity control capabilities. The controller uses
 *          cascaded PID loops with adaptive output scaling to maintain stable position or
 *          track velocity commands in all axes (X, Y, Z horizontal/vertical, and yaw).
 * 
 *          The implementation features:
 *          - Dual-mode operation: position control (run) and velocity control (run_vel)
 *          - Adaptive scaler smoothing to prevent control saturation
 *          - Cascaded PID architecture: position→velocity→actuator commands
 *          - Per-axis enable/disable with Vector4b flags (x, y, z, yaw)
 *          - Coordinate frame: NED (North-East-Down) for position and velocity
 *          - Integration with blimp motor mixing and actuator outputs
 * 
 *          Control Flow:
 *          run():     target_pos → position_error → target_vel → velocity_error → actuator_cmd
 *          run_vel(): target_vel → velocity_error → actuator_cmd
 * 
 * @note Update frequency: Called at main loop rate (typically 50-100Hz for blimp)
 * @note All position/velocity calculations performed in earth-fixed NED frame
 * @note Actuator commands transformed from NED to body frame before motor output
 * 
 * Source: Blimp/Loiter.cpp
 */

#include "Blimp.h"

#include <AC_AttitudeControl/AC_PosControl.h>

/**
 * @brief Moving average coefficient for scaler smoothing (99% previous value)
 * @details Used in exponential moving average: scaler = scaler*MA + new_scaler*MO
 */
#define MA 0.99

/**
 * @brief Moving average complement coefficient (1% new value)
 * @details Complements MA to sum to 1.0 for weighted average calculation
 */
#define MO (1-MA)

/**
 * @brief Run loiter position controller to maintain target position and heading
 * 
 * @details Implements cascaded position-to-velocity-to-actuator control for blimp loiter mode.
 *          This function performs the complete control pipeline from position error calculation
 *          through velocity commands to final actuator outputs.
 * 
 *          Control Architecture (cascaded loops):
 *          1. Position Control: Calculates target velocity from position error using position PIDs
 *          2. Velocity Control: Calculates actuator commands from velocity error using velocity PIDs
 *          3. Output Scaling: Applies adaptive scalers to prevent actuator saturation
 *          4. Motor Output: Converts body-frame actuators to individual motor commands
 * 
 *          Adaptive Scaler Algorithm:
 *          - Monitors combined actuator outputs for potential saturation
 *          - XZ scaler: Limits front/down actuators when combined output > 1.0
 *          - YYaw scaler: Limits right/yaw actuators when combined output > 1.0
 *          - Exponentially smoothed (MA=0.99) to prevent rapid scaling changes
 *          - Applied to both target and actual velocities to maintain consistent error
 * 
 *          Vector4b Flag Semantics:
 *          - axes_disabled: When true, axis does not update PID or generate output (controller frozen)
 *          - zero: When true within deadzone/disarmed/disabled, axis outputs zero thrust
 *          - limit: Combination of zero and axes_disabled, used to limit PID integrators
 * 
 * @param[in,out] target_pos Target position in NED frame (meters). Updated to current position when disarmed.
 * @param[in,out] target_yaw Target heading angle (radians). Updated to current heading when disarmed.
 * @param[in] axes_disabled Per-axis disable flags {x, y, z, yaw}. True = axis disabled (no control).
 * 
 * @note Called at scheduler rate (typically 50-100Hz for blimp vehicles)
 * @note Position error checked against parameter blimp.g.pid_dz (PID deadzone in meters/radians)
 * @note Velocity limits applied: blimp.g.max_vel_xy, blimp.g.max_vel_z, blimp.g.max_vel_yaw
 * @note Coordinate frame: All position/velocity in NED (North-East-Down), converted to body frame for actuators
 * @note When disarmed: All PID integrators reset to zero, targets updated to current state
 * 
 * @warning Modifying scaler constants (MA/MO) affects control stability and saturation handling
 * @warning Axes must be properly disabled when not mechanically available to prevent integrator windup
 * 
 * @see run_vel() for direct velocity control without position loop
 * @see Blimp::pid_pos_xy, Blimp::pid_pos_z, Blimp::pid_pos_yaw for position PID controllers
 * @see Blimp::pid_vel_xy, Blimp::pid_vel_z, Blimp::pid_vel_yaw for velocity PID controllers
 * 
 * Source: Blimp/Loiter.cpp:8-132
 */
void Loiter::run(Vector3f& target_pos, float& target_yaw, Vector4b axes_disabled)
{
    const float dt = blimp.scheduler.get_last_loop_time_s();

    // Calculate adaptive output scalers to prevent actuator saturation
    // Scaler algorithm: If combined outputs exceed 1.0, scale down proportionally
    // Exponential smoothing prevents abrupt changes that could destabilize control
    
    // XZ scaler: Prevents saturation when front and down actuators combined exceed limits
    float scaler_xz_n;
    float xz_out = fabsf(blimp.motors->front_out) + fabsf(blimp.motors->down_out);
    if (xz_out > 1) {
        scaler_xz_n = 1 / xz_out;  // Scale down to keep sum at 1.0
    } else {
        scaler_xz_n = 1;  // No scaling needed
    }
    scaler_xz = scaler_xz*MA + scaler_xz_n*MO;  // Exponential moving average (99% old, 1% new)

    // YYaw scaler: Prevents saturation when right and yaw actuators combined exceed limits
    float scaler_yyaw_n;
    float yyaw_out = fabsf(blimp.motors->right_out) + fabsf(blimp.motors->yaw_out);
    if (yyaw_out > 1) {
        scaler_yyaw_n = 1 / yyaw_out;  // Scale down to keep sum at 1.0
    } else {
        scaler_yyaw_n = 1;  // No scaling needed
    }
    scaler_yyaw = scaler_yyaw*MA + scaler_yyaw_n*MO;  // Exponential moving average (99% old, 1% new)

#if HAL_LOGGING_ENABLED
    AP::logger().WriteStreaming("BSC", "TimeUS,xz,yyaw,xzn,yyawn",
                                "Qffff",
                                AP_HAL::micros64(),
                                scaler_xz, scaler_yyaw, scaler_xz_n, scaler_yyaw_n);
#endif

    // Calculate position and heading errors in NED frame
    float yaw_ef = blimp.ahrs.get_yaw_rad();  // Current heading (earth frame, radians)
    Vector3f err_xyz = target_pos - blimp.pos_ned;  // Position error in NED frame (meters)
    float err_yaw = wrap_PI(target_yaw - yaw_ef);  // Heading error wrapped to [-PI, PI] (radians)

    // Determine which axes should output zero thrust (within deadzone, disarmed, or disabled by parameter)
    // Zero thrust is output when: error < deadzone OR motors disarmed OR axis disabled via dis_mask parameter
    Vector4b zero;
    if ((fabsf(err_xyz.x) < blimp.g.pid_dz) || !blimp.motors->_armed || (blimp.g.dis_mask & (1<<(2-1)))) {
        zero.x = true;  // North axis: zero thrust when in deadzone/disarmed/disabled (dis_mask bit 2)
    }
    if ((fabsf(err_xyz.y) < blimp.g.pid_dz) || !blimp.motors->_armed || (blimp.g.dis_mask & (1<<(1-1)))) {
        zero.y = true;  // East axis: zero thrust when in deadzone/disarmed/disabled (dis_mask bit 1)
    }
    if ((fabsf(err_xyz.z) < blimp.g.pid_dz) || !blimp.motors->_armed || (blimp.g.dis_mask & (1<<(3-1)))) {
        zero.z = true;  // Down axis: zero thrust when in deadzone/disarmed/disabled (dis_mask bit 3)
    }
    if ((fabsf(err_yaw)   < blimp.g.pid_dz) || !blimp.motors->_armed || (blimp.g.dis_mask & (1<<(4-1)))) {
        zero.yaw = true;  // Yaw axis: zero thrust when in deadzone/disarmed/disabled (dis_mask bit 4)
    }

    // Disabled means "don't update PIDs or output anything at all". Zero means actually output zero thrust. I term is limited in either case.
    // Limit flag: Combination used to freeze PID integrators when axis should not be actively controlled
    Vector4b limit = zero || axes_disabled;

    // Position PID: Convert position error to target velocity (outer loop)
    // Only update PIDs for enabled axes to prevent integrator windup on disabled axes
    Vector3f target_vel_ef;
    if (!axes_disabled.x && !axes_disabled.y) target_vel_ef = {blimp.pid_pos_xy.update_all(target_pos, blimp.pos_ned, dt, {(float)limit.x, (float)limit.y, (float)limit.z}), 0};
    if (!axes_disabled.z) {
        target_vel_ef.z = blimp.pid_pos_z.update_all(target_pos.z, blimp.pos_ned.z, dt, limit.z);
    }

    // Yaw position PID: Convert heading error to target yaw rate
    float target_vel_yaw = 0;
    if (!axes_disabled.yaw) {
        target_vel_yaw = blimp.pid_pos_yaw.update_error(wrap_PI(target_yaw - yaw_ef), dt, limit.yaw);
        blimp.pid_pos_yaw.set_target_rate(target_yaw);  // For derivative term
        blimp.pid_pos_yaw.set_actual_rate(yaw_ef);  // For derivative term
    }

    Vector3f target_vel_ef_c{constrain_float(target_vel_ef.x, -blimp.g.max_vel_xy, blimp.g.max_vel_xy),
                             constrain_float(target_vel_ef.y, -blimp.g.max_vel_xy, blimp.g.max_vel_xy),
                             constrain_float(target_vel_ef.z, -blimp.g.max_vel_z, blimp.g.max_vel_z)};
    float target_vel_yaw_c = constrain_float(target_vel_yaw, -blimp.g.max_vel_yaw, blimp.g.max_vel_yaw);

    Vector2f target_vel_ef_c_scaled_xy = {target_vel_ef_c.x * scaler_xz, target_vel_ef_c.y * scaler_yyaw};
    Vector2f vel_ned_filtd_scaled_xy = {blimp.vel_ned_filtd.x * scaler_xz, blimp.vel_ned_filtd.y * scaler_yyaw};

    Vector2f actuator;
    if (!axes_disabled.x && !axes_disabled.y) actuator = blimp.pid_vel_xy.update_all(target_vel_ef_c_scaled_xy, vel_ned_filtd_scaled_xy, dt, {(float)limit.x, (float)limit.y});
    float act_down = 0;
    if (!axes_disabled.z) {
        act_down = blimp.pid_vel_z.update_all(target_vel_ef_c.z * scaler_xz, blimp.vel_ned_filtd.z * scaler_xz, dt, limit.z);
    }
    blimp.rotate_NE_to_BF(actuator);
    float act_yaw = 0;
    if (!axes_disabled.yaw) {
        act_yaw = blimp.pid_vel_yaw.update_all(target_vel_yaw_c * scaler_yyaw, blimp.vel_yaw_filtd * scaler_yyaw, dt, limit.yaw);
    }

    // When disarmed: Reset all PID integrators and update targets to current state
    // Prevents integrator windup during disarmed state and ensures smooth transition on arming
    if (!blimp.motors->armed()) {
        blimp.pid_pos_xy.set_integrator(Vector2f(0,0));
        blimp.pid_pos_z.set_integrator(0);
        blimp.pid_pos_yaw.set_integrator(0);
        blimp.pid_vel_xy.set_integrator(Vector2f(0,0));
        blimp.pid_vel_z.set_integrator(0);
        blimp.pid_vel_yaw.set_integrator(0);
        target_pos = blimp.pos_ned;  // Update target to current position
        target_yaw = blimp.ahrs.get_yaw_rad();  // Update target to current heading
    }

    // Apply motor outputs based on zero and disabled flags
    // Priority: zero flag outputs 0, disabled flag outputs nothing, otherwise apply actuator command
    if (zero.x) {
        blimp.motors->front_out = 0;  // Output zero thrust
    } else if (axes_disabled.x);  // Output nothing (maintain previous value)
    else {
        blimp.motors->front_out = actuator.x;  // Apply actuator command (body frame forward)
    }
    if (zero.y) {
        blimp.motors->right_out = 0;  // Output zero thrust
    } else if (axes_disabled.y);  // Output nothing (maintain previous value)
    else {
        blimp.motors->right_out = actuator.y;  // Apply actuator command (body frame right)
    }
    if (zero.z) {
        blimp.motors->down_out = 0;  // Output zero thrust
    } else if (axes_disabled.z);  // Output nothing (maintain previous value)
    else {
        blimp.motors->down_out = act_down;  // Apply actuator command (body frame down)
    }
    if (zero.yaw) {
        blimp.motors->yaw_out  = 0;  // Output zero thrust
    } else if (axes_disabled.yaw);  // Output nothing (maintain previous value)
    else {
        blimp.motors->yaw_out = act_yaw;  // Apply actuator command (yaw rotation)
    }

#if HAL_LOGGING_ENABLED
    AC_PosControl::Write_PSCN(0.0, target_pos.x * 100.0, blimp.pos_ned.x * 100.0, 0.0, target_vel_ef_c.x * 100.0, blimp.vel_ned_filtd.x * 100.0, 0.0, 0.0, 0.0);
    AC_PosControl::Write_PSCE(0.0, target_pos.y * 100.0, blimp.pos_ned.y * 100.0, 0.0, target_vel_ef_c.y * 100.0, blimp.vel_ned_filtd.y * 100.0, 0.0, 0.0, 0.0);
    AC_PosControl::Write_PSCD(0.0, -target_pos.z * 100.0, -blimp.pos_ned.z * 100.0, 0.0, -target_vel_ef_c.z * 100.0, -blimp.vel_ned_filtd.z * 100.0, 0.0, 0.0, 0.0);
#endif
}

/**
 * @brief Run loiter velocity controller to track target velocity and yaw rate
 * 
 * @details Implements direct velocity-to-actuator control for blimp velocity tracking mode.
 *          This function bypasses the position control loop and directly commands velocities,
 *          useful for manual velocity control or waypoint navigation with velocity commands.
 * 
 *          Control Architecture (single loop):
 *          1. Velocity Control: Calculates actuator commands from velocity error using velocity PIDs
 *          2. Output Scaling: Uses adaptive scalers from previous run() call (scalers are class members)
 *          3. Motor Output: Converts body-frame actuators to individual motor commands
 * 
 *          Differences from run():
 *          - No position PID loop (direct velocity control)
 *          - No position error or deadzone checking
 *          - No target position/yaw updates when disarmed
 *          - Uses same velocity PIDs and scaler values as run()
 *          - Typically called in velocity-controlled flight modes
 * 
 *          Vector4b Flag Semantics:
 *          - axes_disabled: When true, axis does not update PID or generate output (controller frozen)
 *          - zero: When true if disarmed/disabled, axis outputs zero thrust
 *          - limit: Combination of zero and axes_disabled, used to limit PID integrators
 * 
 * @param[in] target_vel_ef Target velocity in NED frame (m/s). Not modified (read-only in this function).
 * @param[in] target_vel_yaw Target yaw rate (rad/s). Not modified (read-only in this function).
 * @param[in] axes_disabled Per-axis disable flags {x, y, z, yaw}. True = axis disabled (no control).
 * 
 * @note Called at scheduler rate (typically 50-100Hz for blimp vehicles)
 * @note Velocity limits applied: blimp.g.max_vel_xy, blimp.g.max_vel_z, blimp.g.max_vel_yaw
 * @note Coordinate frame: All velocities in NED (North-East-Down), converted to body frame for actuators
 * @note When disarmed: Velocity PID integrators reset to zero
 * @note Scalers (scaler_xz, scaler_yyaw) must be updated by run() or initialized separately
 * 
 * @warning Modifying scaler constants (MA/MO) affects control stability and saturation handling
 * @warning Ensure scalers are properly initialized before first call to avoid undefined behavior
 * 
 * @see run() for position control with cascaded position→velocity loops
 * @see Blimp::pid_vel_xy, Blimp::pid_vel_z, Blimp::pid_vel_yaw for velocity PID controllers
 * 
 * Source: Blimp/Loiter.cpp:134-210
 */
void Loiter::run_vel(Vector3f& target_vel_ef, float& target_vel_yaw, Vector4b axes_disabled)
{
    const float dt = blimp.scheduler.get_last_loop_time_s();

    // Determine which axes should output zero thrust (disarmed or disabled by parameter)
    // Note: No deadzone check in velocity mode (no position error to check)
    Vector4b zero;
    if (!blimp.motors->_armed || (blimp.g.dis_mask & (1<<(2-1)))) {
        zero.x = true;  // North axis: zero thrust when disarmed/disabled (dis_mask bit 2)
    }
    if (!blimp.motors->_armed || (blimp.g.dis_mask & (1<<(1-1)))) {
        zero.y = true;  // East axis: zero thrust when disarmed/disabled (dis_mask bit 1)
    }
    if (!blimp.motors->_armed || (blimp.g.dis_mask & (1<<(3-1)))) {
        zero.z = true;  // Down axis: zero thrust when disarmed/disabled (dis_mask bit 3)
    }
    if (!blimp.motors->_armed || (blimp.g.dis_mask & (1<<(4-1)))) {
        zero.yaw = true;  // Yaw axis: zero thrust when disarmed/disabled (dis_mask bit 4)
    }
    // Disabled means "don't update PIDs or output anything at all". Zero means actually output zero thrust. I term is limited in either case.
    // Limit flag: Combination used to freeze PID integrators when axis should not be actively controlled
    Vector4b limit = zero || axes_disabled;

    // Apply velocity limits from parameters to prevent excessive commanded velocities
    Vector3f target_vel_ef_c{constrain_float(target_vel_ef.x, -blimp.g.max_vel_xy, blimp.g.max_vel_xy),
                             constrain_float(target_vel_ef.y, -blimp.g.max_vel_xy, blimp.g.max_vel_xy),
                             constrain_float(target_vel_ef.z, -blimp.g.max_vel_z, blimp.g.max_vel_z)};
    float target_vel_yaw_c = constrain_float(target_vel_yaw, -blimp.g.max_vel_yaw, blimp.g.max_vel_yaw);

    // Apply adaptive scalers to both target and actual velocities for consistent error calculation
    // Scalers are class members updated by run() method or must be initialized separately
    // This prevents velocity PID from seeing artificially large errors when actuators are saturated
    Vector2f target_vel_ef_c_scaled_xy = {target_vel_ef_c.x * scaler_xz, target_vel_ef_c.y * scaler_yyaw};
    Vector2f vel_ned_filtd_scaled_xy = {blimp.vel_ned_filtd.x * scaler_xz, blimp.vel_ned_filtd.y * scaler_yyaw};

    // Velocity PID: Convert velocity error to actuator commands (inner loop)
    // XY velocities processed together, Z and yaw processed independently
    Vector2f actuator;
    if (!axes_disabled.x && !axes_disabled.y) actuator = blimp.pid_vel_xy.update_all(target_vel_ef_c_scaled_xy, vel_ned_filtd_scaled_xy, dt, {(float)limit.x, (float)limit.y});
    float act_down = 0;
    if (!axes_disabled.z) {
        act_down = blimp.pid_vel_z.update_all(target_vel_ef_c.z * scaler_xz, blimp.vel_ned_filtd.z * scaler_xz, dt, limit.z);
    }
    // Convert North-East actuator commands to Body Frame (front-right)
    // Required because motors are mounted in body frame, not earth frame
    blimp.rotate_NE_to_BF(actuator);
    float act_yaw = 0;
    if (!axes_disabled.yaw) {
        act_yaw = blimp.pid_vel_yaw.update_all(target_vel_yaw_c * scaler_yyaw, blimp.vel_yaw_filtd * scaler_yyaw, dt, limit.yaw);
    }

    // When disarmed: Reset velocity PID integrators to prevent windup
    // Note: No position PID integrators in velocity control mode
    if (!blimp.motors->armed()) {
        blimp.pid_vel_xy.set_integrator(Vector2f(0,0));
        blimp.pid_vel_z.set_integrator(0);
        blimp.pid_vel_yaw.set_integrator(0);
    }

    // Apply motor outputs based on zero and disabled flags
    // Priority: zero flag outputs 0, disabled flag outputs nothing, otherwise apply actuator command
    if (zero.x) {
        blimp.motors->front_out = 0;  // Output zero thrust
    } else if (axes_disabled.x);  // Output nothing (maintain previous value)
    else {
        blimp.motors->front_out = actuator.x;  // Apply actuator command (body frame forward)
    }
    if (zero.y) {
        blimp.motors->right_out = 0;  // Output zero thrust
    } else if (axes_disabled.y);  // Output nothing (maintain previous value)
    else {
        blimp.motors->right_out = actuator.y;  // Apply actuator command (body frame right)
    }
    if (zero.z) {
        blimp.motors->down_out = 0;  // Output zero thrust
    } else if (axes_disabled.z);  // Output nothing (maintain previous value)
    else {
        blimp.motors->down_out = act_down;  // Apply actuator command (body frame down)
    }
    if (zero.yaw) {
        blimp.motors->yaw_out  = 0;  // Output zero thrust
    } else if (axes_disabled.yaw);  // Output nothing (maintain previous value)
    else {
        blimp.motors->yaw_out = act_yaw;  // Apply actuator command (yaw rotation)
    }

#if HAL_LOGGING_ENABLED
    AC_PosControl::Write_PSCN(0.0, 0.0, blimp.pos_ned.x * 100.0, 0.0, target_vel_ef_c.x * 100.0, blimp.vel_ned_filtd.x * 100.0, 0.0, 0.0, 0.0);
    AC_PosControl::Write_PSCE(0.0, 0.0, blimp.pos_ned.y * 100.0, 0.0, target_vel_ef_c.y * 100.0, blimp.vel_ned_filtd.y * 100.0, 0.0, 0.0, 0.0);
    AC_PosControl::Write_PSCD(0.0, 0.0, -blimp.pos_ned.z * 100.0, 0.0, -target_vel_ef_c.z * 100.0, -blimp.vel_ned_filtd.z * 100.0, 0.0, 0.0, 0.0);
#endif
}
