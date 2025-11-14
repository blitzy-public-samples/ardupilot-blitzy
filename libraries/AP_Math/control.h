/**
 * @file control.h
 * @brief Common control theory helper functions for position, velocity, and acceleration shaping
 * 
 * @details This file provides a comprehensive set of control system utilities used throughout
 *          ArduPilot for trajectory generation and tracking. These functions implement:
 *          - Jerk-limited motion profiles for smooth acceleration transitions
 *          - Square-root controllers for second-order limited responses
 *          - Kinematic projection with directional constraints
 *          - Input shaping and exponential curves for pilot inputs
 *          - Conversion utilities between lean angles and accelerations
 * 
 *          These functions are primarily used by:
 *          - AC_PosControl: Multicopter position and velocity control
 *          - AC_AttitudeControl: Multicopter attitude control with input shaping
 *          - AC_WPNav: Waypoint navigation with smooth trajectory generation
 * 
 * @note Coordinate frames: Position and velocity typically in NED (North-East-Down) frame,
 *       angles in body frame. Units are explicitly documented for each function.
 * 
 * @see AC_PosControl
 * @see AC_AttitudeControl
 * @see AC_WPNav
 */

#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include "vector2.h"
#include "vector3.h"

/**
 * @brief Position type: float or double based on HAL_WITH_POSTYPE_DOUBLE configuration
 * 
 * @details This typedef allows position calculations to use double precision when needed
 *          for high-accuracy navigation (e.g., surveying applications) or float for
 *          memory-constrained systems. The appropriate Vector2p/Vector3p types are also
 *          defined to match.
 */
#if HAL_WITH_POSTYPE_DOUBLE
typedef double postype_t;
typedef Vector2d Vector2p;
typedef Vector3d Vector3p;
#define topostype todouble
#else
typedef float postype_t;
typedef Vector2f Vector2p;
typedef Vector3f Vector3p;
#define topostype tofloat
#endif

/**
 * @brief Projects velocity forward in time using acceleration, constrained by directional limit
 * 
 * @details This function updates velocity using the equation: vel = vel + accel * dt, but with
 *          directional constraint logic to prevent motion that would worsen error along a limited
 *          direction. When `limit` is active, velocity is only updated if doing so would not
 *          increase the error in the limited direction. If velocity is currently opposing the
 *          limit direction, the update is clipped to avoid crossing zero.
 * 
 * @param[in,out] vel Current velocity in m/s, updated in-place
 * @param[in] accel Desired acceleration in m/s²
 * @param[in] dt Time step in seconds
 * @param[in] limit Direction constraint (non-zero = constrained direction, zero = no constraint)
 * @param[in] vel_error Direction of velocity error (sign matters, magnitude ignored)
 * 
 * @note Prevents unwanted acceleration in the direction of a constraint when it would worsen velocity error
 * @see update_pos_vel_accel() for combined position and velocity update with constraints
 */
void update_vel_accel(float& vel, float accel, float dt, float limit, float vel_error);

/**
 * @brief Projects position and velocity forward in time using acceleration, constrained by directional limit
 * 
 * @details This function performs kinematic projection with directional constraints to prevent motion
 *          that would worsen errors along a limited axis. If the update would increase position error
 *          in the constrained direction, the position update is skipped. The velocity update then
 *          proceeds with directional limit handling via update_vel_accel().
 * 
 * @param[in,out] pos Current position in meters, updated in-place
 * @param[in,out] vel Current velocity in m/s, updated in-place
 * @param[in] accel Desired acceleration in m/s²
 * @param[in] dt Time step in seconds
 * @param[in] limit Direction constraint (non-zero = constrained direction)
 * @param[in] pos_error Direction of position error (sign matters, magnitude ignored)
 * @param[in] vel_error Direction of velocity error (sign matters, magnitude ignored)
 * 
 * @note Prevents motion in a constrained direction if it would worsen the position or velocity error
 * @see update_vel_accel() for velocity-only update with constraints
 */
void update_pos_vel_accel(postype_t& pos, float& vel, float accel, float dt, float limit, float pos_error, float vel_error);

/**
 * @brief Projects velocity forward in time using acceleration, constrained by directional limits (2D XY plane)
 * 
 * @details 2D vector version of update_vel_accel(). Updates velocity using accel * dt but prevents
 *          motion that would increase error along the limit direction. When `limit` vector is non-zero,
 *          it defines a direction in which acceleration is constrained. The velocity is only updated
 *          if doing so would not increase the error in the limited direction.
 * 
 * @param[in,out] vel Current velocity vector in m/s (XY plane), updated in-place
 * @param[in] accel Desired acceleration vector in m/s² (XY plane)
 * @param[in] dt Time step in seconds
 * @param[in] limit Direction constraint vector (non-zero = constrained direction, zero = no constraint)
 * @param[in] vel_error Direction of velocity error vector (magnitude unused, direction matters)
 * 
 * @note Prevents the system from increasing velocity along the limit direction if it would worsen velocity error
 * @see update_vel_accel() for scalar version
 */
void update_vel_accel_xy(Vector2f& vel, const Vector2f& accel, float dt, const Vector2f& limit, const Vector2f& vel_error);

/**
 * @brief Projects position and velocity forward in time using acceleration, constrained by directional limits (2D XY plane)
 * 
 * @details 2D vector version of update_pos_vel_accel(). Performs kinematic projection with directional
 *          constraints to prevent motion that would worsen errors along limited axes. If a motion step
 *          would increase error along a limited axis, it is suppressed. The position is updated first,
 *          then velocity is updated via update_vel_accel_xy().
 * 
 * @param[in,out] pos Current position vector in meters (XY plane), updated in-place
 * @param[in,out] vel Current velocity vector in m/s (XY plane), updated in-place
 * @param[in] accel Desired acceleration vector in m/s² (XY plane)
 * @param[in] dt Time step in seconds
 * @param[in] limit Direction constraint vector (non-zero = constrained direction)
 * @param[in] pos_error Direction of position error vector (magnitude unused, direction matters)
 * @param[in] vel_error Direction of velocity error vector (magnitude unused, direction matters)
 * 
 * @note Avoids changes to position or velocity in the direction of `limit` if those changes would worsen errors
 * @see update_pos_vel_accel() for scalar version
 */
void update_pos_vel_accel_xy(Vector2p& pos, Vector2f& vel, const Vector2f& accel, float dt, const Vector2f& limit, const Vector2f& pos_error, const Vector2f& vel_error);

/**
 * @brief Applies jerk-limited shaping to acceleration value to gradually approach a new target
 * 
 * @details This function implements jerk limiting by constraining the rate of change of acceleration
 *          to be within ±jerk_max over the time step dt. The current acceleration value is modified
 *          in-place to smoothly transition toward accel_input. This ensures smooth transitions in
 *          thrust or lean angle command profiles, preventing abrupt changes that could destabilize
 *          the vehicle or cause excessive motor wear.
 * 
 * @param[in] accel_input Target acceleration in m/s²
 * @param[in,out] accel Current acceleration in m/s², updated in-place with jerk limiting
 * @param[in] jerk_max Maximum allowed jerk (rate of change of acceleration) in m/s³
 * @param[in] dt Time step in seconds
 * 
 * @note Used for smooth thrust transitions in multicopter position control
 * @see shape_vel_accel() for combined velocity and acceleration shaping
 */
void shape_accel(float accel_input, float& accel,
                 float jerk_max, float dt);

/**
 * @brief Applies jerk-limited shaping to a 2D acceleration vector (XY plane)
 * 
 * @details 2D vector version of shape_accel(). Constrains the rate of change of the acceleration
 *          vector to a maximum magnitude of jerk_max over time dt. The current acceleration vector
 *          is modified in-place to smoothly approach accel_input, ensuring smooth acceleration
 *          transitions in both axes simultaneously.
 * 
 * @param[in] accel_input Target acceleration vector in m/s² (XY plane)
 * @param[in,out] accel Current acceleration vector in m/s² (XY plane), updated in-place with jerk limiting
 * @param[in] jerk_max Maximum allowed jerk magnitude in m/s³
 * @param[in] dt Time step in seconds
 * 
 * @note Ensures coordinated smooth transitions in lateral acceleration
 */
void shape_accel_xy(const Vector2f& accel_input, Vector2f& accel,
                    float jerk_max, float dt);

/**
 * @brief Applies jerk-limited shaping to a 3D acceleration vector
 * 
 * @details 3D vector version of shape_accel(). Constrains the rate of change of the acceleration
 *          vector to a maximum magnitude of jerk_max over time dt in 3D space.
 * 
 * @param[in] accel_input Target acceleration vector in m/s² (3D)
 * @param[in,out] accel Current acceleration vector in m/s² (3D), updated in-place with jerk limiting
 * @param[in] jerk_max Maximum allowed jerk magnitude in m/s³
 * @param[in] dt Time step in seconds
 */
void shape_accel_xy(const Vector3f& accel_input, Vector3f& accel,
                    float jerk_max, float dt);

/**
 * @brief Shapes velocity and acceleration using jerk-limited control
 * 
 * @details This function computes a correction acceleration needed to reach vel_input from current vel
 *          using a square-root controller with max acceleration and jerk constraints. The correction
 *          is combined with feedforward accel_input. If limit_total_accel is true, the total acceleration
 *          is constrained to [accel_min, accel_max]. The result is then applied via shape_accel() to
 *          ensure jerk limiting. This provides smooth velocity tracking with bounded dynamics.
 * 
 * @param[in] vel_input Target velocity in m/s
 * @param[in] accel_input Feedforward acceleration in m/s²
 * @param[in] vel Current velocity in m/s
 * @param[in,out] accel Current acceleration in m/s², updated in-place
 * @param[in] accel_min Minimum allowed acceleration in m/s² (typically negative for deceleration)
 * @param[in] accel_max Maximum allowed acceleration in m/s²
 * @param[in] jerk_max Maximum allowed jerk in m/s³
 * @param[in] dt Time step in seconds
 * @param[in] limit_total_accel If true, constrains total acceleration; if false, constrains only correction
 * 
 * @note Uses sqrt_controller() internally for second-order limited velocity tracking
 * @see shape_accel() for jerk limiting applied to the result
 * @see sqrt_controller() for the underlying controller algorithm
 */
void shape_vel_accel(float vel_input, float accel_input,
                     float vel, float& accel,
                     float accel_min, float accel_max,
                     float jerk_max, float dt, bool limit_total_accel);

/**
 * @brief Computes a jerk-limited acceleration command in 2D to track a desired velocity input
 * 
 * @details 2D vector version of shape_vel_accel(). Uses a square-root controller to calculate
 *          correction acceleration based on velocity error. The correction is constrained to stay
 *          within accel_max (total acceleration magnitude), then added to accel_input (feedforward).
 *          If limit_total_accel is true, total acceleration is constrained after summing. This
 *          ensures velocity tracking with smooth, physically constrained motion in the XY plane.
 * 
 * @param[in] vel_input1 Target velocity vector in m/s (XY plane)
 * @param[in] accel_input Feedforward acceleration vector in m/s² (XY plane)
 * @param[in] vel Current velocity vector in m/s (XY plane)
 * @param[in,out] accel Current acceleration vector in m/s² (XY plane), updated in-place
 * @param[in] accel_max Maximum allowed acceleration magnitude in m/s²
 * @param[in] jerk_max Maximum allowed jerk magnitude in m/s³
 * @param[in] dt Time step in seconds
 * @param[in] limit_total_accel If true, constrains total acceleration; if false, constrains only correction
 * 
 * @note Used in AC_PosControl for lateral velocity control
 * @see shape_vel_accel() for scalar version
 */
void shape_vel_accel_xy(const Vector2f& vel_input1, const Vector2f& accel_input,
                        const Vector2f& vel, Vector2f& accel,
                        float accel_max, float jerk_max, float dt, bool limit_total_accel);

/**
 * @brief Shapes position, velocity, and acceleration using a jerk-limited profile (full 3-stage controller)
 * 
 * @details This function implements a complete 3-stage jerk-limited position controller. It first
 *          computes a velocity command to close position error using a square-root controller, then
 *          shapes that velocity via shape_vel_accel() to enforce acceleration and jerk limits. Limits
 *          can be applied separately to correction or to total values. This provides smooth point-to-point
 *          motion with fully constrained dynamics (velocity, acceleration, and jerk limits).
 * 
 * @param[in] pos_input Target position in meters
 * @param[in] vel_input Feedforward velocity in m/s
 * @param[in] accel_input Feedforward acceleration in m/s²
 * @param[in] pos Current position in meters
 * @param[in] vel Current velocity in m/s
 * @param[in,out] accel Current acceleration in m/s², updated in-place
 * @param[in] vel_min Minimum allowed velocity in m/s (typically negative)
 * @param[in] vel_max Maximum allowed velocity in m/s
 * @param[in] accel_min Minimum allowed acceleration in m/s² (typically negative)
 * @param[in] accel_max Maximum allowed acceleration in m/s²
 * @param[in] jerk_max Maximum allowed jerk in m/s³
 * @param[in] dt Time step in seconds
 * @param[in] limit_total If true, constrains total command; if false, constrains only correction
 * 
 * @note Primary function for smooth trajectory generation in AC_PosControl altitude control
 * @see shape_vel_accel() for velocity shaping stage
 * @see sqrt_controller() for position error to velocity conversion
 */
void shape_pos_vel_accel(const postype_t pos_input, float vel_input, float accel_input,
                         const postype_t pos, float vel, float& accel,
                         float vel_min, float vel_max,
                         float accel_min, float accel_max,
                         float jerk_max, float dt, bool limit_total);

/**
 * @brief Computes a jerk-limited acceleration profile to move toward a position and velocity target in 2D
 * 
 * @details 2D vector version of shape_pos_vel_accel(). Implements a complete 3-stage jerk-limited position
 *          controller for the XY plane. Computes a velocity correction based on position error using a
 *          square-root controller, then passes that velocity to shape_vel_accel_xy() to generate a
 *          constrained acceleration. If limit_total is true, constraints are applied to the total command
 *          (not just the correction). This provides smooth trajectory shaping for lateral motion with
 *          bounded dynamics.
 * 
 * @param[in] pos_input Target position vector in meters (XY plane)
 * @param[in] vel_input Feedforward velocity vector in m/s (XY plane)
 * @param[in] accel_input Feedforward acceleration vector in m/s² (XY plane)
 * @param[in] pos Current position vector in meters (XY plane)
 * @param[in] vel Current velocity vector in m/s (XY plane)
 * @param[in,out] accel Current acceleration vector in m/s² (XY plane), updated in-place
 * @param[in] vel_max Maximum allowed velocity magnitude in m/s
 * @param[in] accel_max Maximum allowed acceleration magnitude in m/s²
 * @param[in] jerk_max Maximum allowed jerk magnitude in m/s³
 * @param[in] dt Time step in seconds
 * @param[in] limit_total If true, constrains total command; if false, constrains only correction
 * 
 * @note Primary function for smooth lateral trajectory generation in AC_PosControl
 * @see shape_pos_vel_accel() for scalar version
 */
void shape_pos_vel_accel_xy(const Vector2p& pos_input, const Vector2f& vel_input, const Vector2f& accel_input,
                            const Vector2p& pos, const Vector2f& vel, Vector2f& accel,
                            float vel_max, float accel_max,
                            float jerk_max, float dt, bool limit_total);

/**
 * @brief Computes a jerk-limited acceleration command to follow an angular position, velocity, and acceleration target
 * 
 * @details This function applies jerk-limited shaping to angular acceleration for attitude control. It internally
 *          computes a target angular velocity using a square-root controller on the angle error, then shapes the
 *          angular acceleration toward the target using shape_vel_accel(). Velocity and acceleration are both
 *          optionally constrained: if limit_total is true, limits apply to the total (not just correction) command.
 *          Setting angle_vel_max or angle_accel_max to zero disables that respective limit.
 * 
 * @param[in] angle_input Target angle in radians
 * @param[in] angle_vel_input Feedforward angular velocity in rad/s
 * @param[in] angle_accel_input Feedforward angular acceleration in rad/s²
 * @param[in] angle Current angle in radians
 * @param[in] angle_vel Current angular velocity in rad/s
 * @param[in,out] angle_accel Current angular acceleration in rad/s², updated in-place
 * @param[in] angle_vel_max Maximum allowed angular velocity in rad/s (0 = no limit)
 * @param[in] angle_accel_max Maximum allowed angular acceleration in rad/s² (0 = no limit)
 * @param[in] angle_jerk_max Maximum allowed angular jerk in rad/s³
 * @param[in] dt Time step in seconds
 * @param[in] limit_total If true, constrains total command; if false, constrains only correction
 * 
 * @note Used in AC_AttitudeControl for smooth roll/pitch/yaw angle tracking with rate and acceleration limits
 * @see shape_vel_accel() for the underlying shaping algorithm
 */
void shape_angle_vel_accel(float angle_input, float angle_vel_input, float angle_accel_input,
                         float angle, float angle_vel, float& angle_accel,
                         float angle_vel_max, float angle_accel_max,
                         float angle_jerk_max, float dt, bool limit_total);

/**
 * @brief Limits a 2D acceleration vector to prioritize lateral (cross-track) acceleration over longitudinal (in-track) acceleration
 * 
 * @details This function constrains a 2D acceleration vector to remain within accel_max while prioritizing
 *          cross-track correction over along-track acceleration. The vel vector defines the current direction
 *          of motion, which is used to decompose accel into lateral and longitudinal components. If the full
 *          acceleration vector exceeds accel_max, it is reshaped to prioritize lateral correction (which is
 *          more important for trajectory tracking). If vel is zero, a simple magnitude limit is applied.
 * 
 * @param[in] vel Current velocity vector in m/s (defines direction of motion)
 * @param[in,out] accel Acceleration vector in m/s² (XY plane), modified in-place if limiting is needed
 * @param[in] accel_max Maximum allowed acceleration magnitude in m/s²
 * 
 * @return true if the acceleration vector was modified (limited), false if no change was needed
 * 
 * @note Used in waypoint navigation to prioritize path tracking over speed changes
 */
bool limit_accel_xy(const Vector2f& vel, Vector2f& accel, float accel_max);

/**
 * @brief Piecewise square-root + linear controller that limits second-order response (acceleration)
 * 
 * @details This controller implements a hybrid approach that behaves like a P controller near the setpoint
 *          but switches to sqrt(2·a·Δx) shaping beyond a threshold to limit acceleration. This provides
 *          time-optimal response subject to acceleration constraints. Near the target, the controller acts
 *          as a simple proportional controller. Far from the target, it uses kinematic equations to achieve
 *          maximum acceleration without overshoot.
 * 
 *          The transition point is automatically calculated based on the P gain and acceleration limit,
 *          ensuring smooth behavior across the full error range.
 * 
 * @param[in] error Position or velocity error (current - target)
 * @param[in] p Proportional gain (units depend on application: 1/s for velocity, 1/s² for position)
 * @param[in] second_ord_lim Maximum allowed second-order rate (acceleration for velocity control, jerk for position)
 * @param[in] dt Time step in seconds
 * 
 * @return Constrained correction rate (velocity correction for position error, acceleration for velocity error)
 * 
 * @warning Near zero error, numerical precision issues can occur; consider using a deadband in calling code
 * @note This is the fundamental building block for time-optimal trajectory generation in ArduPilot
 * @see inv_sqrt_controller() to invert this function
 * @see stopping_distance() which uses the inverse to calculate required stopping distance
 */
float sqrt_controller(float error, float p, float second_ord_lim, float dt);

/**
 * @brief Vector form of sqrt_controller(), applied along the direction of the input error vector
 * 
 * @details 2D vector version of sqrt_controller(). Computes the magnitude of the correction using
 *          sqrt_controller() applied to the error magnitude, then returns a correction vector in the
 *          same direction as the error vector. This preserves the direction of the error while applying
 *          acceleration-limited shaping to the correction magnitude.
 * 
 * @param[in] error Position or velocity error vector (current - target) in XY plane
 * @param[in] p Proportional gain (1/s for velocity, 1/s² for position)
 * @param[in] second_ord_lim Maximum allowed second-order rate magnitude
 * @param[in] dt Time step in seconds
 * 
 * @return Correction vector (velocity for position error, acceleration for velocity error)
 * 
 * @note Used in AC_PosControl for 2D position and velocity control with acceleration limiting
 * @see sqrt_controller() for scalar version and detailed algorithm explanation
 */
Vector2f sqrt_controller(const Vector2f& error, float p, float second_ord_lim, float dt);

/**
 * @brief Inverts the output of sqrt_controller() to recover the input error that would produce a given output
 * 
 * @details This function inverts the sqrt_controller() response to calculate what error would be required
 *          to produce a desired output rate. It handles both the linear region (near setpoint) and the
 *          square-root region (far from setpoint) of the controller response. This is useful for
 *          calculating required separation distances or for planning trajectories.
 * 
 * @param[in] output Desired output rate (velocity or acceleration depending on context)
 * @param[in] p Proportional gain used in sqrt_controller()
 * @param[in] D_max Maximum second-order rate limit (corresponds to second_ord_lim in sqrt_controller)
 * 
 * @return Required error to produce the specified output
 * 
 * @note Used internally by stopping_distance() to calculate deceleration distances
 * @see sqrt_controller() for the function being inverted
 * @see stopping_distance() for a practical application
 */
float inv_sqrt_controller(float output, float p, float D_max);

/**
 * @brief Calculates stopping distance required to reduce a velocity to zero using a square-root controller
 * 
 * @details This function uses the inverse of the sqrt_controller() response curve to calculate the distance
 *          required to smoothly decelerate from a given velocity to zero. It accounts for the P gain and
 *          maximum deceleration limit to provide the actual stopping distance that would be achieved by
 *          the sqrt_controller-based velocity control.
 * 
 * @param[in] velocity Current velocity in m/s (magnitude, always positive)
 * @param[in] p Proportional gain for position control (1/s)
 * @param[in] accel_max Maximum allowed deceleration in m/s²
 * 
 * @return Stopping distance in meters required to decelerate cleanly to zero
 * 
 * @note Critical for waypoint approach calculations and collision avoidance
 * @see inv_sqrt_controller() for the underlying inversion algorithm
 */
float stopping_distance(float velocity, float p, float accel_max);

/**
 * @brief Computes the maximum possible acceleration or velocity magnitude in a specified 3D direction
 * 
 * @details This function calculates the maximum achievable magnitude in a given 3D direction while
 *          respecting separate kinematic limits for horizontal (XY) and vertical (Z) motion. The
 *          direction vector is decomposed into horizontal and vertical components, and the limits
 *          are applied to each component independently. The function then computes the overall
 *          magnitude limit that doesn't violate any axis constraint.
 * 
 * @param[in] direction Desired direction of travel (3D unit vector or non-zero direction vector)
 * @param[in] max_xy Maximum allowed magnitude in horizontal XY plane (m/s or m/s²)
 * @param[in] max_z_pos Maximum allowed magnitude in upward Z direction (m/s or m/s²)
 * @param[in] max_z_neg Maximum allowed magnitude in downward Z direction (m/s or m/s²)
 * 
 * @return Maximum achievable magnitude in the specified direction without violating axis constraints
 * 
 * @note Used for velocity and acceleration limiting in AC_PosControl with asymmetric Z limits
 * @note Coordinate frame: NED (North-East-Down), so positive Z is downward
 */
float kinematic_limit(Vector3f direction, float max_xy, float max_z_pos, float max_z_neg);

/**
 * @brief Applies an exponential curve to a normalized input in the range [-1, 1]
 * 
 * @details This function applies an exponential response curve to pilot stick inputs to provide
 *          more precise control near center stick while maintaining full authority at the extremes.
 *          The expo parameter shapes the curve: 0 produces a linear response, values closer to 1
 *          produce more curvature (finer control near center). The expo value is clipped to < 0.95
 *          to avoid numerical issues. The output is guaranteed to be in [-1, 1] for inputs in [-1, 1].
 * 
 * @param[in] input Normalized input value in range [-1, 1] (typically from RC stick)
 * @param[in] expo Exponential shaping factor [0, 0.95] (0 = linear, higher = more expo)
 * 
 * @return Shaped output in range [-1, 1] with exponential response curve applied
 * 
 * @note Commonly used in AC_AttitudeControl for pilot roll/pitch/yaw stick input shaping
 * @note Formula: output = input * (1 - expo) + sign(input) * expo * input²
 */
float input_expo(float input, float expo);

/**
 * @brief Converts a lean angle (radians) to horizontal acceleration in m/s²
 * 
 * @details Computes the horizontal acceleration produced by a multicopter at a given lean angle.
 *          Uses the relationship: a = g * tan(θ), where g is gravitational acceleration (9.80665 m/s²).
 *          This assumes flat Earth and that the vehicle is producing enough thrust to maintain altitude.
 * 
 * @param[in] angle_rad Lean angle in radians (roll or pitch angle from horizontal)
 * 
 * @return Horizontal acceleration in m/s²
 * 
 * @note Used to convert attitude commands to expected accelerations for position control
 * @see accel_mss_to_angle_rad() for inverse conversion
 */
float angle_rad_to_accel_mss(float angle_rad);

/**
 * @brief Converts a lean angle (degrees) to horizontal acceleration in m/s²
 * 
 * @details Degree version of angle_rad_to_accel_mss(). Converts angle to radians then applies
 *          the relationship: a = g * tan(θ).
 * 
 * @param[in] angle_deg Lean angle in degrees (roll or pitch angle from horizontal)
 * 
 * @return Horizontal acceleration in m/s²
 * 
 * @see angle_rad_to_accel_mss() for radian version
 */
float angle_deg_to_accel_mss(float angle_deg);

/**
 * @brief Converts a horizontal acceleration (m/s²) to lean angle in radians
 * 
 * @details Computes the lean angle required for a multicopter to produce a given horizontal acceleration.
 *          Uses the relationship: θ = atan(a / g), where g is gravitational acceleration (9.80665 m/s²).
 *          This is the inverse of angle_rad_to_accel_mss().
 * 
 * @param[in] accel_mss Horizontal acceleration in m/s²
 * 
 * @return Required lean angle in radians
 * 
 * @note Used by position controller to convert desired acceleration to attitude target
 * @see angle_rad_to_accel_mss() for inverse conversion
 */
float accel_mss_to_angle_rad(float accel_mss);

/**
 * @brief Converts a horizontal acceleration (m/s²) to lean angle in degrees
 * 
 * @details Degree version of accel_mss_to_angle_rad(). Computes lean angle using θ = atan(a / g),
 *          then converts result to degrees.
 * 
 * @param[in] accel_mss Horizontal acceleration in m/s²
 * 
 * @return Required lean angle in degrees
 * 
 * @see accel_mss_to_angle_rad() for radian version
 */
float accel_mss_to_angle_deg(float accel_mss);

/**
 * @brief Converts pilot’s normalized roll/pitch input into target roll and pitch angles (radians)
 * 
 * @details This function converts normalized stick inputs from the pilot's transmitter into actual target
 *          roll and pitch angles. It handles two different angle limits: angle_max_rad defines the maximum
 *          stick deflection mapping, while angle_limit_rad provides a secondary constraint that limits the
 *          output while preserving the full stick input range. This allows the full stick range to be used
 *          even when angle limits are reduced (e.g., for beginner flight modes).
 * 
 *          When both roll and pitch are commanded, the function ensures the total lean angle doesn't exceed
 *          limits by proportionally scaling both outputs if necessary.
 * 
 * @param[in] roll_in_norm Pilot's normalized roll input [-1, 1] where -1 is full left, +1 is full right
 * @param[in] pitch_in_norm Pilot's normalized pitch input [-1, 1] where -1 is full forward, +1 is full back
 * @param[in] angle_max_rad Maximum angle for full stick deflection in radians (defines stick scaling)
 * @param[in] angle_limit_rad Secondary angle limit in radians (constrains output while preserving stick range)
 * @param[out] roll_out_rad Output target roll angle in radians (body frame)
 * @param[out] pitch_out_rad Output target pitch angle in radians (body frame)
 * 
 * @note Used in AC_AttitudeControl for pilot input processing in angle-controlled flight modes
 * @note Coordinate frame: Body frame Euler angles (roll right positive, pitch forward negative in NED)
 * @see input_expo() often applied to inputs before calling this function
 */
void rc_input_to_roll_pitch_rad(float roll_in_norm, float pitch_in_norm, float angle_max_rad, float angle_limit_rad, float &roll_out_rad, float &pitch_out_rad);
