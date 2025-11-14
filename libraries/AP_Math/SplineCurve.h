/**
 * @file SplineCurve.h
 * @brief Hermite cubic spline trajectory generation with kinematic constraints
 * 
 * @details This file implements smooth curved path generation between waypoints using
 *          Hermite cubic splines. The spline interpolates both position and velocity
 *          at endpoints, enabling smooth velocity transitions between trajectory segments.
 *          
 *          Key Features:
 *          - Hermite spline: position and velocity specified at both endpoints
 *          - Kinematic constraints: respects max speeds (xy/up/down) and accelerations
 *          - Time-scaling: spline parameterized 0 to 1, scaled to satisfy constraints
 *          - Coordinate frame: NEU (North-East-Up) relative to EKF origin
 *          
 *          Use Case: Curved waypoint following with velocity matching between segments
 *          for smooth multirotor flight through complex 3D paths.
 * 
 * @note All position and velocity vectors use NEU (North-East-Up) frame relative to EKF origin
 */

#pragma once

#include <AP_Common/AP_Common.h>

/**
 * @class SplineCurve
 * @brief Generates smooth curved paths between waypoints using Hermite cubic splines
 * 
 * @details SplineCurve creates smooth 3D trajectory segments that interpolate both position
 *          and velocity at the segment endpoints. This enables continuous velocity transitions
 *          when chaining multiple spline segments together for complex path following.
 *          
 *          Hermite Spline Interpolation:
 *          A Hermite cubic spline is defined by four constraints:
 *          - Position at origin (t=0)
 *          - Velocity at origin (t=0)
 *          - Position at destination (t=1)
 *          - Velocity at destination (t=1)
 *          
 *          This fully determines a cubic polynomial path that smoothly connects the endpoints
 *          while matching the specified velocities, ensuring C1 continuity (continuous position
 *          and velocity) across trajectory segments.
 *          
 *          Time Scaling and Kinematic Constraints:
 *          The spline is initially parameterized from t=0 to t=1 (unitless parameter).
 *          The actual traversal speed is determined by scaling this parameter based on:
 *          - Maximum horizontal speed (xy plane)
 *          - Maximum vertical speeds (separate up/down limits)
 *          - Maximum horizontal acceleration
 *          - Maximum vertical acceleration
 *          
 *          The time-scaling algorithm ensures the vehicle never exceeds configured limits
 *          while following the spline path as quickly as possible.
 *          
 *          Coordinate Frame:
 *          All positions are offsets from the EKF origin in NEU (North-East-Up) frame:
 *          - X axis: North
 *          - Y axis: East  
 *          - Z axis: Up
 *          
 *          All velocities are expressed in NEU frame (m/s in North, East, Up directions).
 *          
 * @note This class is typically used by position controllers for smooth waypoint navigation
 * @note The spline must be recalculated (via set_origin_and_destination) when waypoints change
 * 
 * @warning For smooth flight, origin_vel must match the vehicle's actual velocity when starting
 *          the spline segment. Velocity discontinuities will cause abrupt control changes.
 * @warning Similarly, when chaining splines, each segment's origin_vel must match the previous
 *          segment's destination_vel to maintain C1 continuity.
 */
class SplineCurve {

public:

    /**
     * @brief Set maximum speed and acceleration kinematic constraints
     * 
     * @details Configures the velocity and acceleration limits that constrain spline traversal.
     *          These limits are used during time-scaling to ensure the vehicle follows the
     *          spline path without exceeding its kinematic capabilities.
     *          
     *          The spline time-scaling algorithm will automatically slow down the trajectory
     *          in high-curvature sections to respect these acceleration limits.
     * 
     * @param[in] speed_xy    Maximum horizontal speed (XY plane) in m/s
     * @param[in] speed_up    Maximum vertical upward speed (positive Z) in m/s
     * @param[in] speed_down  Maximum vertical downward speed (negative Z) in m/s
     * @param[in] accel_xy    Maximum horizontal acceleration (XY plane) in m/s²
     * @param[in] accel_z     Maximum vertical acceleration (Z axis) in m/s²
     * 
     * @note Must be called before set_origin_and_destination() to ensure proper time-scaling
     * @note Typical multirotor values: speed_xy=5-10 m/s, accel_xy=2-5 m/s²
     */
    void set_speed_accel(float speed_xy, float speed_up, float speed_down,
                         float accel_xy, float accel_z);

    /**
     * @brief Define spline endpoints with positions and velocities
     * 
     * @details Calculates the Hermite cubic spline coefficients that define a smooth curved
     *          path from origin to destination. The spline interpolates both position and
     *          velocity at the endpoints, ensuring smooth transitions when chaining segments.
     *          
     *          Hermite Spline Calculation:
     *          The method computes a cubic polynomial P(t) where:
     *          - P(0) = origin
     *          - P(1) = destination
     *          - P'(0) = origin_vel (scaled)
     *          - P'(1) = destination_vel (scaled)
     *          
     *          The resulting spline coefficients are stored in _hermite_solution[4] for
     *          efficient evaluation during trajectory following.
     *          
     *          Velocity Matching:
     *          The origin_vel and destination_vel parameters enable C1 continuity (continuous
     *          velocity) between adjacent spline segments:
     *          - origin_vel should match the vehicle's current velocity at segment start
     *          - destination_vel should match the next segment's origin_vel
     *          
     *          This prevents velocity discontinuities that would cause jerky flight.
     * 
     * @param[in] origin          Origin position offset from EKF origin in NEU frame (meters)
     * @param[in] destination     Destination position offset from EKF origin in NEU frame (meters)
     * @param[in] origin_vel      Vehicle velocity at origin in NEU frame (m/s)
     * @param[in] destination_vel Vehicle velocity at destination in NEU frame (m/s)
     * 
     * @note All vectors use NEU (North-East-Up) coordinate frame relative to EKF origin
     * @note Call set_speed_accel() first to configure kinematic constraints
     * @note Resets spline time to 0 and clears reached_destination flag
     * 
     * @warning origin_vel must match the vehicle's actual velocity for smooth flight initiation
     * @warning Velocity discontinuity between actual and specified origin_vel causes abrupt control changes
     * 
     * @see advance_target_along_track() to step along the computed spline
     * @see update_solution() internal method that computes the spline coefficients
     */
    void set_origin_and_destination(const Vector3f &origin, const Vector3f &destination, const Vector3f &origin_vel, const Vector3f &destination_vel);

    /**
     * @brief Advance trajectory forward in time along the spline path
     * 
     * @details Steps the spline trajectory forward by the specified time increment, computing
     *          the target position and velocity while respecting kinematic constraints.
     *          
     *          Time-Scaled Trajectory Following:
     *          This method implements time-scaled spline traversal that respects the speed
     *          and acceleration limits set by set_speed_accel(). The spline parameter is
     *          advanced such that:
     *          - Speed never exceeds configured limits (speed_xy, speed_up, speed_down)
     *          - Acceleration never exceeds configured limits (accel_xy, accel_z)
     *          - Path curvature automatically reduces speed to satisfy acceleration limits
     *          
     *          The internal spline time parameter (0 to 1) is advanced based on the actual
     *          distance traveled, scaled by kinematic feasibility.
     *          
     *          Completion Detection:
     *          When the spline parameter reaches 1.0, the destination is reached and
     *          the _reached_destination flag is set. Subsequent calls continue to return
     *          the destination position and velocity.
     * 
     * @param[in]  dt         Time increment in seconds (typically main loop period, e.g., 0.0025s for 400Hz)
     * @param[out] target_pos Target position offset from EKF origin in NEU frame (meters)
     * @param[out] target_vel Target velocity in NEU frame (m/s)
     * 
     * @note Call set_origin_and_destination() first to define the spline
     * @note Call repeatedly at fixed intervals (e.g., 400Hz main loop) for smooth trajectory
     * @note target_pos and target_vel are outputs that should be passed to position controller
     * @note After reaching destination, returns destination position and velocity on subsequent calls
     * 
     * @see reached_destination() to check if trajectory is complete
     * @see calc_target_pos_vel() internal method that evaluates spline at current time
     */
    void advance_target_along_track(float dt, Vector3f &target_pos, Vector3f &target_vel);

    /**
     * @brief Check if vehicle has completed the spline trajectory
     * 
     * @details Returns true when the spline time parameter has reached 1.0, indicating
     *          the vehicle has arrived at the destination endpoint. This flag is set by
     *          advance_target_along_track() when trajectory completion is detected.
     * 
     * @return true if destination reached (spline time >= 1.0), false if still traversing
     * 
     * @note Use to transition to next waypoint segment or complete mission
     * @note Flag is reset to false when set_origin_and_destination() is called with new endpoints
     */
    bool reached_destination() const WARN_IF_UNUSED { return _reached_destination; }

    /**
     * @brief Get the unscaled destination velocity vector
     * 
     * @details Returns the destination velocity that was specified in set_origin_and_destination().
     *          This is the "unscaled" velocity vector that defines the spline endpoint derivative.
     *          
     *          Use Case:
     *          When chaining multiple spline segments, use this value as the origin_vel for the
     *          next segment to ensure C1 continuity (continuous velocity) between segments:
     *          
     *          spline1.set_origin_and_destination(p0, p1, v0, v1);
     *          spline2.set_origin_and_destination(p1, p2, spline1.get_destination_vel(), v2);
     * 
     * @return Reference to destination velocity vector in NEU frame (m/s)
     * 
     * @note Returns the velocity vector specified in set_origin_and_destination()
     * @note Use for velocity matching when transitioning to next spline segment
     */
    const Vector3f& get_destination_vel() WARN_IF_UNUSED { return _destination_vel; }

    /**
     * @brief Get maximum feasible speed at spline origin
     * 
     * @details Returns the maximum speed at which the vehicle can traverse the spline
     *          starting point while respecting acceleration constraints. This value is
     *          computed during spline initialization based on path curvature at the origin.
     *          
     *          If the spline has high initial curvature, this speed will be reduced below
     *          the configured speed_xy limit to ensure acceleration limits are respected.
     * 
     * @return Maximum speed at origin in m/s
     * 
     * @note Value is calculated in set_origin_and_destination()
     * @note May be less than configured speed_xy if initial path curvature is high
     */
    float get_origin_speed_max() const WARN_IF_UNUSED { return _origin_speed_max; }

    /**
     * @brief Get maximum feasible speed at spline destination
     * 
     * @details Returns the maximum speed at which the vehicle can reach the spline
     *          destination while respecting acceleration constraints. This value is
     *          computed based on path curvature at the destination endpoint.
     *          
     *          If the spline has high curvature near the destination (sharp turn), this
     *          speed will be reduced to ensure the vehicle can decelerate or change
     *          direction without exceeding acceleration limits.
     * 
     * @return Maximum speed at destination in m/s
     * 
     * @note Value is calculated in set_origin_and_destination()
     * @note May be less than configured speed_xy if destination path curvature is high
     * @note Use to determine appropriate origin speed for next spline segment
     */
    float get_destination_speed_max() const WARN_IF_UNUSED { return _destination_speed_max; }
    
    /**
     * @brief Limit maximum speed at destination
     * 
     * @details Applies an additional speed constraint at the destination endpoint,
     *          taking the minimum of the current limit and the specified value.
     *          
     *          Use Case:
     *          Reduce destination speed when approaching a sharp corner or when the
     *          next waypoint segment requires a low entry speed. This ensures smooth
     *          velocity transitions between segments without overshoot.
     * 
     * @param[in] destination_speed_max New speed limit at destination in m/s
     * 
     * @note Only reduces the speed limit; cannot increase it above current value
     * @note Call before set_origin_and_destination() to affect spline time-scaling
     */
    void set_destination_speed_max(float destination_speed_max) { _destination_speed_max = MIN(_destination_speed_max, destination_speed_max); }

private:

    /**
     * @brief Calculate spline time increment and kinematic limits for distance traveled
     * 
     * @details Internal method used by advance_target_along_track() to determine how much
     *          to advance the spline time parameter for a given distance increment.
     *          
     *          Computes the maximum speed and acceleration at the current spline position
     *          based on path curvature, then calculates the spline time increment needed
     *          to travel distance_delta while respecting kinematic constraints.
     * 
     * @param[in]  time            Current spline time parameter (0 to 1)
     * @param[in]  distance_delta  Distance to travel in meters
     * @param[out] spline_dt       Computed spline time increment (0 to 1)
     * @param[out] target_pos      Position at current spline time in NEU frame (meters)
     * @param[out] spline_vel_unit Unit velocity vector at current spline time
     * @param[out] speed_max       Maximum feasible speed at this position (m/s)
     * @param[out] accel_max       Maximum feasible acceleration at this position (m/s²)
     * 
     * @note Private method for internal spline time-scaling calculation
     * @note Called repeatedly by advance_target_along_track() during trajectory following
     */
    void calc_dt_speed_max(float time, float distance_delta, float &spline_dt, Vector3f &target_pos, Vector3f &spline_vel_unit, float &speed_max, float &accel_max);

    /**
     * @brief Recalculate Hermite spline coefficient matrix
     * 
     * @details Computes the four coefficient vectors that define the Hermite cubic spline
     *          polynomial connecting origin to destination with specified endpoint velocities.
     *          
     *          Hermite Spline Formula:
     *          P(t) = h0(t)*origin + h1(t)*origin_vel + h2(t)*dest + h3(t)*dest_vel
     *          
     *          where h0, h1, h2, h3 are Hermite basis functions:
     *          - h0(t) = 2t³ - 3t² + 1
     *          - h1(t) = t³ - 2t² + t
     *          - h2(t) = -2t³ + 3t²
     *          - h3(t) = t³ - t²
     *          
     *          The coefficient matrix _hermite_solution[4] is precomputed for efficient
     *          evaluation during trajectory following.
     * 
     * @param[in] origin     Origin position in NEU frame (meters)
     * @param[in] dest       Destination position in NEU frame (meters)
     * @param[in] origin_vel Velocity at origin in NEU frame (m/s)
     * @param[in] dest_vel   Velocity at destination in NEU frame (m/s)
     * 
     * @note Private method called by set_origin_and_destination()
     * @note Stores results in _hermite_solution[4] member variable
     * @note Sets _zero_length flag if origin equals destination
     */
    void update_solution(const Vector3f &origin, const Vector3f &dest, const Vector3f &origin_vel, const Vector3f &dest_vel);

    /**
     * @brief Evaluate spline position, velocity, acceleration, and jerk at given time
     * 
     * @details Evaluates the Hermite cubic spline and its derivatives at the specified
     *          spline time parameter. Uses the precomputed coefficient matrix from
     *          update_solution() for efficient calculation.
     *          
     *          Evaluation Method:
     *          - Position: P(t) = cubic polynomial evaluation
     *          - Velocity: V(t) = dP/dt (first derivative)
     *          - Acceleration: A(t) = d²P/dt² (second derivative)
     *          - Jerk: J(t) = d³P/dt³ (third derivative)
     *          
     *          All derivatives are computed in the spline parameter space (t ∈ [0,1]),
     *          so velocity magnitude needs additional scaling based on actual trajectory speed.
     * 
     * @param[in]  time         Spline time parameter (0 to 1, where 0=origin, 1=destination)
     * @param[out] position     Position at time t, offset from EKF origin in NEU frame (meters)
     * @param[out] velocity     Unscaled velocity vector at time t (spline derivative)
     * @param[out] acceleration Unscaled acceleration vector at time t (second derivative)
     * @param[out] jerk         Unscaled jerk vector at time t (third derivative)
     * 
     * @note Private method called by advance_target_along_track() and calc_dt_speed_max()
     * @note Requires update_solution() to have been called first to compute spline coefficients
     * @note Velocity/acceleration/jerk are "unscaled" derivatives in parameter space
     * @note For actual physical velocity, the velocity vector must be scaled by trajectory speed
     */
    void calc_target_pos_vel(float time, Vector3f &position, Vector3f &velocity, Vector3f &acceleration, Vector3f &jerk);

    // Spline endpoint state
    Vector3f    _origin;                ///< Origin position offset from EKF origin in NEU frame (meters)
    Vector3f    _destination;           ///< Destination position offset from EKF origin in NEU frame (meters)
    Vector3f    _origin_vel;            ///< Target velocity vector at origin in NEU frame (m/s)
    Vector3f    _destination_vel;       ///< Target velocity vector at destination in NEU frame (m/s)
    
    // Spline coefficient matrix (computed by update_solution)
    Vector3f    _hermite_solution[4];   ///< Hermite cubic spline coefficient vectors [h0, h1, h2, h3]
    
    // Trajectory state
    float       _time;                  ///< Current spline time parameter (0 to 1), where 0=origin, 1=destination
    
    // Kinematic constraints (configured by set_speed_accel)
    float       _speed_xy;              ///< Maximum horizontal speed in XY plane (m/s)
    float       _speed_up;              ///< Maximum vertical upward speed, positive Z direction (m/s)
    float       _speed_down;            ///< Maximum vertical downward speed, negative Z direction (m/s)
    float       _accel_xy;              ///< Maximum horizontal acceleration in XY plane (m/s²)
    float       _accel_z;               ///< Maximum vertical acceleration in Z direction (m/s²)
    
    // Computed kinematic limits at endpoints
    float       _origin_speed_max;      ///< Maximum feasible speed at origin based on path curvature (m/s)
    float       _destination_speed_max; ///< Maximum feasible speed at destination based on path curvature (m/s)
    
    // State flags
    bool        _reached_destination;   ///< True once spline time reaches 1.0 (destination reached)
    bool        _zero_length;           ///< True if origin equals destination (degenerate spline)
};
