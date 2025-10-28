/**
 * @file SCurve.h
 * @brief Jerk-limited S-curve trajectory generation for smooth waypoint navigation
 * 
 * This library generates kinematically constrained 3D trajectories using jerk-limited
 * S-curve profiles. The trajectories provide smooth motion between waypoints with
 * continuous position, velocity, acceleration, and jerk, avoiding step changes that
 * can excite vehicle dynamics or cause mechanical stress.
 * 
 * The S-curve implementation uses a 23-segment deterministic trajectory that respects
 * limits on snap (rate of change of jerk), jerk, acceleration, and velocity. This is
 * particularly important for multicopter waypoint navigation where smooth transitions
 * minimize attitude changes and improve tracking performance.
 * 
 * Coordinate Frame: All calculations are performed in earth frame (typically NED).
 * 
 * Unit Consistency: This library works with any consistent unit system (meters, cm, etc).
 * All parameters must use matching units (e.g., if positions are in meters, velocities
 * must be in m/s, accelerations in m/s^2, jerk in m/s^3, snap in m/s^4).
 * 
 * @note Integrates with multicopter position controllers (AC_PosControl) for waypoint navigation
 * @note Based on trigonometric S-curve profiles with raised cosine jerk transitions
 */

#pragma once

#include <AP_Common/AP_Common.h>

/**
 * @class SCurve
 * @brief Generates jerk-limited S-curve trajectories for smooth waypoint navigation
 * 
 * @details This class implements a 23-segment deterministic trajectory generator that produces
 * smooth motion profiles between waypoints. The trajectories respect kinematic limits on snap
 * (rate of change of jerk), jerk, acceleration, and velocity, ensuring continuous and smooth
 * motion without step changes.
 * 
 * Kinematic Hierarchy (each is the derivative of the previous):
 * - Position: Location in 3D space (earth frame)
 * - Velocity: Rate of change of position (m/s)
 * - Acceleration: Rate of change of velocity (m/s^2)
 * - Jerk: Rate of change of acceleration (m/s^3)
 * - Snap: Rate of change of jerk (m/s^4)
 * 
 * The 23-segment structure consists of:
 * - Segment 0: Initial conditions (position and velocity)
 * - Segments 1-7: Acceleration phase (using raised cosine jerk profile)
 * - Segments 8-14: Speed change transition
 * - Segment 15: Constant velocity cruise
 * - Segments 16-22: Deceleration phase (mirror of acceleration)
 * 
 * @note Typical Usage Workflow:
 * 1. Create three SCurve objects: prev_leg, this_leg, next_leg
 * 2. Call this_leg.calculate_track() to compute path from origin to destination
 * 3. If flying through waypoint to next destination:
 *    a) Call next_leg.calculate_track() with next segment parameters
 *    b) Set fast_waypoint = true for advance_target_along_track()
 *    If stopping at waypoint:
 *    a) Call next_leg.init() to clear
 *    b) Set fast_waypoint = false
 * 4. Call this_leg.advance_target_along_track() with dt to get target_pos, target_vel, target_accel
 *    (Note: target_pos must be initialized to segment's earth frame origin)
 * 5. Pass targets to position controller (e.g., AC_PosControl)
 * 6. Repeat steps 4-5 until finished() returns true
 * 7. Promote legs: prev_leg = this_leg, this_leg = next_leg, return to step 3
 * 
 * @note Unit Consistency: All parameters must use consistent units throughout.
 * Example: If origin/destination in meters, use m/s for velocity, m/s^2 for acceleration,
 * m/s^3 for jerk, m/s^4 for snap. If using centimeters, use cm/s, cm/s^2, cm/s^3, cm/s^4.
 * 
 * @warning Path recalculation (via set_speed_max or calculate_track) is computationally
 * expensive as it solves for all 23 segment times. Avoid calling these functions at high
 * rates (e.g., every loop iteration).
 * 
 * Additional Features:
 * - set_speed_max(): Dynamically adjust speed limits mid-path (triggers recalculation)
 * - set_origin_speed_max() / set_destination_speed_max(): Set endpoint speeds for
 *   smooth integration with spline segments or adjacent path legs
 * 
 * Terminology:
 * - track: The 3D straight-line path vector from origin to destination
 * - path: The kinematic profile (position, velocity, acceleration, jerk vs time)
 * - jerk time (tj): Duration for jerk to increase from zero to maximum using raised cosine
 * 
 * Coordinate Frame: Earth frame (NED typically used in ArduPilot)
 * 
 * @note Integrates with AC_PosControl and AC_WPNav for multicopter waypoint navigation
 * @note Uses raised cosine profiles for jerk transitions to ensure C2 continuity (continuous jerk)
 */
class SCurve {

public:

    /**
     * @brief Constructor - initializes SCurve object
     */
    SCurve();

    /**
     * @brief Initialize and clear the path, resetting all state
     * 
     * @details Resets internal time to zero, clears all segment data, and prepares
     * the object for a new trajectory calculation. Call this before calculate_track()
     * or when discarding a previously calculated path.
     */
    void init();

    /**
     * @brief Calculate segment times for the trigonometric S-curve path (static function)
     * 
     * @details Solves for the segment durations needed to achieve a kinematically constrained
     * path with the specified limits. Uses raised cosine jerk profiles for smooth transitions.
     * This is the core mathematical function that computes the 23-segment time structure.
     * 
     * This is an internal function, made static to allow testing by the test suite.
     * 
     * @param[in] Sm Maximum snap magnitude (m/s^4 if using meters, cm/s^4 if using cm)
     * @param[in] Jm Maximum jerk magnitude for raised cosine profile (m/s^3 or cm/s^3)
     * @param[in] V0 Initial velocity magnitude (m/s or cm/s)
     * @param[in] Am Maximum constant acceleration (m/s^2 or cm/s^2)
     * @param[in] Vm Maximum constant velocity (m/s or cm/s)
     * @param[in] L Total length of the path (m or cm)
     * @param[out] Jm_out Actual maximum jerk used (may be less than Jm if not needed) (m/s^3 or cm/s^3)
     * @param[out] tj_out Jerk time - duration for jerk transition using raised cosine (seconds)
     * @param[out] t2_out Duration of constant jerk segment (seconds)
     * @param[out] t4_out Duration of constant acceleration segment (seconds)
     * @param[out] t6_out Duration of constant velocity segment (seconds)
     * 
     * @note All input/output variables must use consistent units (meters OR centimeters, not mixed)
     * @note The segment durations are symmetric: acceleration phase mirrors deceleration phase
     */
    static void calculate_path(float Sm, float Jm, float V0, float Am, float Vm, float L, float &Jm_out, float &tj_out, float &t2_out, float &t4_out, float &t6_out);

    /**
     * @brief Generate 3D straight-line trajectory between two points with kinematic constraints
     * 
     * @details Calculates the complete 23-segment trajectory from origin to destination as a
     * straight line in 3D space. The trajectory respects different speed limits for horizontal
     * (XY plane) and vertical (Z axis) motion, as well as separate acceleration limits.
     * 
     * The function decomposes the 3D motion into horizontal and vertical components, applies
     * the respective limits, and generates a smooth trajectory that satisfies all constraints.
     * 
     * Call this once per path leg. For dynamic speed changes, use set_speed_max() instead.
     * 
     * @param[in] origin Starting position in earth frame (m or cm, NED convention typically)
     * @param[in] destination Ending position in earth frame (m or cm, NED convention typically)
     * @param[in] speed_xy Maximum horizontal speed (m/s or cm/s)
     * @param[in] speed_up Maximum upward (negative Z in NED) speed (m/s or cm/s)
     * @param[in] speed_down Maximum downward (positive Z in NED) speed (m/s or cm/s)
     * @param[in] accel_xy Maximum horizontal acceleration (m/s^2 or cm/s^2)
     * @param[in] accel_z Maximum vertical acceleration (m/s^2 or cm/s^2)
     * @param[in] snap_maximum Maximum snap (rate of change of jerk) (m/s^4 or cm/s^4)
     * @param[in] jerk_maximum Maximum jerk (rate of change of acceleration) (m/s^3 or cm/s^3)
     * 
     * @note All position, speed, acceleration, jerk, and snap parameters must use consistent units
     * @note Units must match throughout: if origin/destination in meters, use m/s, m/s^2, m/s^3, m/s^4
     * @warning Computationally expensive - avoid calling every loop iteration
     */
    void calculate_track(const Vector3f &origin, const Vector3f &destination,
                         float speed_xy, float speed_up, float speed_down,
                         float accel_xy, float accel_z,
                         float snap_maximum, float jerk_maximum);

    /**
     * @brief Set maximum velocity limits and recalculate the path
     * 
     * @details Dynamically updates the speed limits for an already-calculated trajectory and
     * recalculates all segment times to respect the new limits. Useful for adjusting speeds
     * mid-flight without regenerating the entire path from scratch.
     * 
     * @param[in] speed_xy New maximum horizontal speed (m/s or cm/s)
     * @param[in] speed_up New maximum upward speed (m/s or cm/s)
     * @param[in] speed_down New maximum downward speed (m/s or cm/s)
     * 
     * @note Units must match those used in calculate_track()
     * @warning Triggers path recalculation - computationally expensive, avoid high-rate calls
     */
    void set_speed_max(float speed_xy, float speed_up, float speed_down);

    /**
     * @brief Set maximum vehicle speed at the path origin
     * 
     * @details Constrains the velocity at the start of the trajectory to enable smooth
     * transitions from the previous path segment. Used when integrating with spline segments
     * or when the vehicle is already moving at path start.
     * 
     * @param[in] speed Desired speed at origin (m/s or cm/s, must match path units)
     * 
     * @return float Actual achievable speed at origin (always ≤ requested speed due to kinematic constraints)
     * 
     * @note The returned speed may be lower than requested if kinematic limits cannot achieve it
     * @note Call before or after calculate_track() to set initial velocity constraint
     */
    float set_origin_speed_max(float speed);

    /**
     * @brief Set maximum vehicle speed at the path destination
     * 
     * @details Constrains the velocity at the end of the trajectory to enable smooth
     * transitions to the next path segment. Used when flying through waypoints rather than
     * stopping, or when integrating with subsequent spline segments.
     * 
     * @param[in] speed Desired speed at destination (m/s or cm/s, must match path units)
     * 
     * @note Used in conjunction with fast_waypoint=true in advance_target_along_track()
     * @note Call after calculate_track() to set final velocity constraint
     */
    void set_destination_speed_max(float speed);

    /**
     * @brief Advance target position along the trajectory by dt seconds
     * 
     * @details Steps the trajectory forward in time and computes the target position, velocity,
     * and acceleration vectors for the position controller. This function is called at the
     * main control loop rate (typically 50-400 Hz) to generate smooth reference trajectories.
     * 
     * When approaching a waypoint with fast_waypoint=true, this function handles corner cutting
     * by blending with the next_leg trajectory, allowing smooth continuous motion through waypoints.
     * 
     * The function uses prev_leg and next_leg to compute optimal corner-cutting trajectories that
     * respect wp_radius and accel_corner constraints while maintaining smooth motion.
     * 
     * @param[in] prev_leg Previous path segment (used for corner calculations)
     * @param[in] next_leg Next path segment (used for corner calculations and blending)
     * @param[in] wp_radius Maximum distance from waypoint center at apex of turn (m or cm)
     * @param[in] accel_corner Maximum acceleration during corner maneuver (m/s^2 or cm/s^2)
     * @param[in] fast_waypoint true if flying through waypoint to next leg, false if stopping
     * @param[in] dt Time increment to advance along path (seconds, typically 0.0025 to 0.02)
     * @param[in,out] target_pos Input: must be set to segment's origin in earth frame; Output: updated position target (m or cm)
     * @param[out] target_vel Updated velocity target in earth frame (m/s or cm/s)
     * @param[out] target_accel Updated acceleration target in earth frame (m/s^2 or cm/s^2)
     * 
     * @return true if vehicle has passed the apex of the corner turn, false otherwise
     * 
     * @note Call this at main loop rate (400 Hz for copter, 50 Hz typical for other vehicles)
     * @note target_pos MUST be initialized to the segment's earth frame origin before first call
     * @note Targets are passed to position controller (AC_PosControl) after this function
     * @note Units for all position/velocity/acceleration must match those from calculate_track()
     */
    bool advance_target_along_track(SCurve &prev_leg, SCurve &next_leg, float wp_radius, float accel_corner, bool fast_waypoint, float dt, Vector3f &target_pos, Vector3f &target_vel, Vector3f &target_accel) WARN_IF_UNUSED;

    /**
     * @brief Check if trajectory has reached the end of the sequence
     * 
     * @details Returns true when internal time has reached or exceeded the final segment time,
     * indicating the vehicle has completed this path leg and should transition to the next leg
     * or stop at the destination.
     * 
     * @return true if path sequence is complete, false if still in progress
     * 
     * @note Check this after each call to advance_target_along_track()
     * @note When true, promote legs: prev_leg = this_leg, this_leg = next_leg
     */
    bool finished() const WARN_IF_UNUSED;

private:

    // increment time and return the position, velocity and acceleration vectors relative to the origin
    void move_from_pos_vel_accel(float dt, Vector3f &pos, Vector3f &vel, Vector3f &accel);

    // increment time and return the position, velocity and acceleration vectors relative to the destination
    void move_to_pos_vel_accel(float dt, Vector3f &pos, Vector3f &vel, Vector3f &accel);

    // return the position, velocity and acceleration vectors relative to the origin at a specified time along the path
    void move_from_time_pos_vel_accel(float t, Vector3f &pos, Vector3f &vel, Vector3f &accel);

    // get desired maximum speed along track
    float get_speed_along_track() const WARN_IF_UNUSED { return vel_max; }

    // get desired maximum acceleration along track
    float get_accel_along_track() const WARN_IF_UNUSED { return accel_max; }

    // get desired maximum acceleration along track
    float get_accel_z_max() const WARN_IF_UNUSED { return accel_z_max; }

    // return the change in position from origin to destination
    const Vector3f& get_track() const WARN_IF_UNUSED { return track; };

    // return the current time elapsed
    float get_time_elapsed() const WARN_IF_UNUSED { return time; }

    // time at the end of the sequence
    float time_end() const WARN_IF_UNUSED;

    // time left before sequence will complete
    float get_time_remaining() const WARN_IF_UNUSED;

    // time when acceleration section of the sequence will complete
    float get_accel_finished_time() const WARN_IF_UNUSED;

    // return true if the sequence is braking to a stop
    bool braking() const WARN_IF_UNUSED;

    // return time offset used to initiate the turn onto leg
    float time_accel_end() const WARN_IF_UNUSED;

    // return time offset used to initiate the turn from leg
    float time_decel_start() const WARN_IF_UNUSED;

    // increment the internal time
    void advance_time(float dt);

    // calculate the jerk, acceleration, velocity and position at time t
    void get_jerk_accel_vel_pos_at_time(float time_now, float &Jt_out, float &At_out, float &Vt_out, float &Pt_out) const;

    // calculate the jerk, acceleration, velocity and position at time t when running the constant jerk time segment
    void calc_javp_for_segment_const_jerk(float time_now, float J0, float A0, float V0, float P0, float &Jt, float &At, float &Vt, float &Pt) const;

    // Calculate the jerk, acceleration, velocity and position at time t when running the increasing jerk magnitude time segment based on a raised cosine profile
    void calc_javp_for_segment_incr_jerk(float time_now, float tj, float Jm, float A0, float V0, float P0, float &Jt, float &At, float &Vt, float &Pt) const;

    // Calculate the jerk, acceleration, velocity and position at time t when running the decreasing jerk magnitude time segment based on a raised cosine profile
    void calc_javp_for_segment_decr_jerk(float time_now, float tj, float Jm, float A0, float V0, float P0, float &Jt, float &At, float &Vt, float &Pt) const;

    // generate time segments for straight segment
    void add_segments(float L);

    // generate three time segments forming the jerk profile
    void add_segments_jerk(uint8_t &seg_pnt, float Jm, float tj, float Tcj);

    // generate constant jerk time segment
    void add_segment_const_jerk(uint8_t &seg_pnt, float J0, float tin);

    // generate increasing jerk magnitude time segment based on a raised cosine profile
    void add_segment_incr_jerk(uint8_t &seg_pnt, float Jm, float tj);

    // generate decreasing jerk magnitude time segment based on a raised cosine profile
    void add_segment_decr_jerk(uint8_t &seg_pnt, float Jm, float tj);

    // set speed and acceleration limits for the path
    // origin and destination are offsets from EKF origin
    // speed and acceleration parameters are given in horizontal, up and down.
    void set_kinematic_limits(const Vector3f &origin, const Vector3f &destination,
                              float speed_xy, float speed_up, float speed_down,
                              float accel_xy, float accel_z);

    // return true if the curve is valid.  Used to identify and protect against code errors
    bool valid() const WARN_IF_UNUSED;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    // debugging messages
    void debug() const;
#endif

    // segment types
    enum class SegmentType {
        CONSTANT_JERK,
        POSITIVE_JERK,
        NEGATIVE_JERK
    };

    // add single segment
    void add_segment(uint8_t &seg_pnt, float end_time, SegmentType seg_type, float jerk_ref, float end_accel, float end_vel, float end_pos);

    // members
    float snap_max;     // maximum snap magnitude
    float jerk_max;     // maximum jerk magnitude
    float accel_max;    // maximum acceleration magnitude
    float accel_z_max;    // maximum acceleration magnitude
    float vel_max;      // maximum velocity magnitude
    float time;         // time that defines position on the path
    float position_sq;  // position (squared) on the path at the last time step (used to detect finish)

    // segment 0 is the initial segment and holds the vehicle's initial position and velocity
    // segments 1 to 7 are the acceleration segments
    // segments 8 to 14 are the speed change segments
    // segment 15 is the constant velocity segment
    // segment 16 to 22 is the deceleration segment
    const static uint8_t segments_max = 23; // maximum number of time segments

    uint8_t num_segs;       // number of time segments being used
    struct {
        float jerk_ref;     // jerk reference value for time segment (the jerk at the beginning, middle or end depending upon the segment type)
        SegmentType seg_type;   // segment type (jerk is constant, increasing or decreasing)
        float end_time;     // final time value for segment
        float end_accel;    // final acceleration value for segment
        float end_vel;      // final velocity value for segment
        float end_pos;      // final position value for segment
    } segment[segments_max];

    Vector3f track;       // total change in position from origin to destination
    Vector3f delta_unit;  // reference direction vector for path
};
