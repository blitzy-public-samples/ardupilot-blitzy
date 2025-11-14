#pragma once

/**
 * @file    AP_L1_Control.h
 * @brief   L1 Control algorithm for fixed-wing lateral navigation
 * 
 * @details This file implements the L1 nonlinear guidance law for trajectory tracking
 *          in fixed-wing aircraft. L1 guidance provides robust path following with
 *          explicit control over tracking loop period and damping ratio.
 *          
 *          The L1 controller is based on the paper "A New Nonlinear Guidance Logic for
 *          Trajectory Tracking" by Sanghyuk Park, John Deyst, and Jonathan How (AIAA 2004).
 *          
 *          Modifications from the original paper include:
 *          - Explicit period and damping control for easier tuning
 *          - Support for loiter radii smaller than L1 distance
 *          - Integration with ArduPilot AHRS and navigation systems
 *          - Crosstrack error integrator for zero steady-state error
 *          
 *          This is an instance of the AP_Navigation class providing the standard
 *          navigation interface for ArduPilot fixed-wing vehicles.
 */

/*
 * Originally written by Brandon Jones 2013
 *
 *  Modified by Paul Riseborough 2013 to provide:
 *  - Explicit control over frequency and damping
 *  - Explicit control over track capture angle
 *  - Ability to use loiter radius smaller than L1 length
 */

#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>
#include <AP_Navigation/AP_Navigation.h>
#include <AP_TECS/AP_TECS.h>
#include <AP_Common/Location.h>

/**
 * @class AP_L1_Control
 * @brief L1 nonlinear guidance controller for fixed-wing lateral navigation
 * 
 * @details The L1 controller provides lateral navigation guidance for fixed-wing aircraft
 *          by computing a virtual "L1 reference point" on the desired path and generating
 *          lateral acceleration commands to track toward this point.
 *          
 *          **L1 Reference Point Concept:**
 *          The L1 reference point is a virtual waypoint placed on the desired path at a
 *          distance L1_dist ahead of the aircraft's ground track. The controller generates
 *          lateral acceleration to steer the velocity vector toward this point, resulting
 *          in smooth path tracking with predictable convergence behavior.
 *          
 *          **Coordinate Systems:**
 *          - Navigation calculations use NED (North-East-Down) frame
 *          - Position from AHRS in WGS84 lat/lon via Location class
 *          - Velocity from AHRS groundspeed vector in NED frame
 *          - Outputs (lateral acceleration) in body frame (positive to right)
 *          - Bearings in radians internally, converted to centidegrees for API
 *          
 *          **Integration with AP_Navigation:**
 *          This class inherits from AP_Navigation, providing the standard interface used
 *          by ArduPlane for all lateral navigation modes (waypoint, loiter, heading hold).
 *          
 *          **Thread Safety:**
 *          This class is designed for single-threaded sequential calls from the navigation
 *          loop (typically 10Hz). No internal locking is provided. All update methods
 *          must be called from the same thread at the navigation loop rate.
 *          
 *          **Key Algorithm Features:**
 *          - Nonlinear guidance law with guaranteed convergence
 *          - Explicit period and damping control for intuitive tuning
 *          - Crosstrack error integrator for zero steady-state error
 *          - Adaptive L1 distance based on groundspeed and desired period
 *          - Special handling for loiter circles and heading hold
 *          - Altitude compensation via EAS2TAS scaling
 *          
 * @note Based on "A New Nonlinear Guidance Logic for Trajectory Tracking"
 *       by Sanghyuk Park, John Deyst, and Jonathan How (AIAA 2004)
 * 
 * @warning L1_PERIOD is safety-critical: values too small cause aggressive turns
 *          and potential stall. Use 15-20s for typical aircraft.
 */
class AP_L1_Control : public AP_Navigation {
public:
    /**
     * @brief Construct L1 controller with required dependencies
     * 
     * @param[in] ahrs Reference to AHRS for position, velocity, and attitude data
     * @param[in] tecs Pointer to TECS controller for target airspeed integration (may be nullptr)
     * 
     * @details Constructor initializes the L1 controller and registers default parameter
     *          values via AP_Param::setup_object_defaults(). The AHRS reference is used
     *          throughout the L1 calculations to obtain current position, groundspeed,
     *          and heading. The TECS pointer is optional and used to query target airspeed
     *          for altitude scaling calculations.
     */
    AP_L1_Control(AP_AHRS &ahrs, const AP_TECS *tecs)
        : _ahrs(ahrs)
        , _tecs(tecs)
    {
        AP_Param::setup_object_defaults(this, var_info);
    }

    /* Do not allow copies */
    CLASS_NO_COPY(AP_L1_Control);

    /**
     * @brief Get desired roll angle to achieve L1 tracking
     * 
     * @return Roll angle demand in centidegrees [-9000, 9000] corresponding to ±90°
     * 
     * @details Converts lateral acceleration demand to roll angle using the relationship:
     *          roll = atan(latAccel / (g*cos(pitch)))
     *          
     *          The conversion accounts for aircraft pitch angle to maintain coordinated
     *          flight. Pitch is constrained to ±60° to avoid division by zero near
     *          vertical attitudes.
     * 
     * @note This is the primary output used by the attitude controller for navigation
     * @warning Roll output is clamped to ±9000 centidegrees (±90°) for safety
     */
    int32_t nav_roll_cd(void) const override;
    
    /**
     * @brief Get lateral acceleration needed to achieve L1 tracking
     * 
     * @return Lateral acceleration demand in m/s², positive to right
     * 
     * @details This is the raw output from the L1 guidance law before conversion to
     *          roll angle. The lateral acceleration represents the centripetal acceleration
     *          needed to curve the flight path toward the L1 reference point.
     * 
     * @note Used by attitude controller to command coordinated turns
     */
    float lateral_acceleration(void) const override;

    /**
     * @brief Get navigation bearing to L1 reference point
     * 
     * @return Bearing in centidegrees [-18000, 18000], wrapped to ±180°
     * 
     * @details Returns the bearing from the aircraft's current position to the L1
     *          reference point on the desired path. This bearing indicates the direction
     *          the aircraft should be heading to track the desired path.
     * 
     * @note Used for ground station display of navigation guidance direction
     */
    int32_t nav_bearing_cd(void) const override;

    /**
     * @brief Get bearing error relative to desired track
     * 
     * @return Bearing error in centidegrees, positive means left of track
     * 
     * @details Returns the difference between the desired track heading and the actual
     *          aircraft heading. This is the Nu angle from the L1 algorithm, representing
     *          the angle between the velocity vector and the desired track direction.
     * 
     * @note This is the Nu angle from L1 algorithm in centidegrees
     */
    int32_t bearing_error_cd(void) const override;

    /**
     * @brief Get perpendicular distance from desired track
     * 
     * @return Crosstrack error in meters, positive means right of track
     * 
     * @details Returns the distance in the X-Y plane from the aircraft position to
     *          the desired track line. This is the perpendicular distance, not the
     *          direct distance to the waypoint.
     * 
     * @note Used for telemetry and monitoring navigation performance
     */
    float crosstrack_error(void) const override { return _crosstrack_error; }
    
    /**
     * @brief Get crosstrack error integrator value
     * 
     * @return Integrator state in radians
     * 
     * @details Returns the current state of the crosstrack error integrator (_L1_xtrack_i).
     *          This integrator provides feedback to ensure crosstrack error converges to zero
     *          in the presence of steady winds or biases.
     * 
     * @note Integrator has anti-windup limiting to ±0.1 radians
     */
    float crosstrack_error_integrator(void) const override { return _L1_xtrack_i; }

    /**
     * @brief Get bearing from aircraft to next waypoint
     * 
     * @return Target bearing in centidegrees [-18000, 18000], wrapped to ±180°
     * 
     * @details Returns the direct bearing to the target waypoint, not accounting for
     *          L1 guidance. This is different from nav_bearing_cd() which points to
     *          the L1 reference point on the path.
     * 
     * @note Different from nav_bearing_cd() which points to L1 reference point
     */
    int32_t target_bearing_cd(void) const override;
    
    /**
     * @brief Calculate turn entry distance for waypoint approach
     * 
     * @param[in] wp_radius Waypoint radius in meters at sea level
     * 
     * @return Turn entry distance in meters, scaled by EAS2TAS²
     * 
     * @details Calculates the distance before a waypoint at which the aircraft should
     *          begin turning to intercept the next leg. This overload assumes a 90° turn.
     *          The distance is scaled by EAS2TAS² to account for altitude effects on
     *          turn radius.
     * 
     * @note EAS2TAS scaling accounts for altitude effects on turn radius
     * @warning Returns minimum of wp_radius*EAS2TAS² and _L1_dist to prevent excessively early turns
     */
    float turn_distance(float wp_radius) const override;
    
    /**
     * @brief Calculate turn entry distance for waypoint approach with specified turn angle
     * 
     * @param[in] wp_radius Waypoint radius in meters at sea level
     * @param[in] turn_angle Turn angle in degrees
     * 
     * @return Turn entry distance in meters, scaled by EAS2TAS² and turn angle
     * 
     * @details Calculates the distance before a waypoint at which the aircraft should
     *          begin turning. This overload scales linearly for turns less than 90°.
     *          For turns >= 90°, uses the full turn_distance calculation.
     * 
     * @note Scales linearly for turns <90° to allow earlier waypoint sequencing on gentle corners
     */
    float turn_distance(float wp_radius, float turn_angle) const override;
    
    /**
     * @brief Calculate safe loiter radius accounting for altitude and bank limits
     * 
     * @param[in] loiter_radius Requested loiter radius in meters at sea level
     * 
     * @return Safe loiter radius in meters, scaled for altitude and bank angle limits
     * 
     * @details Applies NAVL1_LIM_BANK constraint and altitude scaling via EAS2TAS².
     *          If NAVL1_LIM_BANK > 0, calculates minimum safe radius based on bank angle
     *          limit and current airspeed. If NAVL1_LIM_BANK = 0, applies only direct
     *          altitude scaling.
     * 
     * @note If NAVL1_LIM_BANK=0, uses direct altitude scaling
     * @warning Bank limit sanitized to [0°, 89°] to prevent invalid calculations
     * @warning EAS2TAS² scaling means loiter radius grows significantly with altitude
     */
    float loiter_radius (const float loiter_radius) const override;
    /**
     * @brief Update L1 guidance for waypoint navigation
     * 
     * @param[in] prev_WP Previous waypoint location
     * @param[in] next_WP Next waypoint location
     * @param[in] dist_min Minimum L1 distance in meters (default 0.0)
     * 
     * @details Computes the L1 reference point on the line between prev_WP and next_WP,
     *          then calculates lateral acceleration to track toward this point.
     *          
     *          **Algorithm Steps:**
     *          1. Calculate groundspeed and heading from AHRS
     *          2. Compute L1 distance: L1_dist = (damping * period * groundspeed) / π
     *          3. Calculate crosstrack error (perpendicular distance to AB line)
     *          4. Compute along-track distance to determine path segment
     *          5. Calculate Nu angle (velocity vector angle relative to desired track)
     *          6. Apply crosstrack integrator with anti-windup
     *          7. Compute lateral acceleration: latAccel = K_L1 * groundSpeed² / L1_dist * sin(Nu)
     *          8. Update outputs: _nav_bearing, _bearing_error, _latAccDem
     *          
     *          **Path Handling:**
     *          Handles three cases: approaching prev_WP, tracking line segment, approaching next_WP
     * 
     * @note Called at navigation loop rate, typically 10Hz
     * @note Crosstrack integrator only active when Nu < 5° (near track alignment)
     * @note Includes indecision prevention logic to avoid oscillations near waypoint
     * @warning If GPS invalid, maintains last outputs and sets _data_is_stale
     */
    void update_waypoint(const class Location &prev_WP, const class Location &next_WP, float dist_min = 0.0f) override;
    
    /**
     * @brief Update L1 guidance for loiter circle navigation
     * 
     * @param[in] center_WP Center of loiter circle
     * @param[in] radius Requested loiter radius in meters (negative for counter-clockwise)
     * @param[in] loiter_direction Direction: +1 clockwise, -1 counter-clockwise
     * 
     * @details Implements dual-mode control: capture mode to reach the circle, and
     *          circle tracking mode to maintain the circular path.
     *          
     *          **Capture Mode (outside circle):**
     *          Uses L1 law to capture toward the center point
     *          
     *          **Circle Mode (on circle):**
     *          Uses PD control for radius error plus centripetal acceleration for circular motion
     *          PD gains: Kx = omega², Kv = 2*damping*omega, where omega = 2π/period
     *          Centripetal acceleration: v²/r
     *          
     *          **Mode Switching:**
     *          Switches based on acceleration command crossover between capture and circle mode
     *          
     *          **Hysteresis:**
     *          _WPcircle flag has hysteresis to prevent false negatives from wind gusts
     * 
     * @note Called at navigation loop rate, typically 10Hz
     * @note Radius automatically scaled by loiter_radius() for altitude and bank limits
     * @note Sets _WPcircle flag when established on loiter circle
     */
    void update_loiter(const class Location &center_WP, float radius, int8_t loiter_direction) override;
    
    /**
     * @brief Update L1 guidance for heading hold mode
     * 
     * @param[in] navigation_heading_cd Desired heading in centidegrees
     * 
     * @details Maintains constant heading using L1 feedback law. L1 distance is adapted
     *          to maintain constant tracking loop frequency.
     *          
     *          **Algorithm:**
     *          1. Calculate normalized frequency: omegaA = sqrt(2)*π/period
     *          2. Adjust L1 distance: L1_dist = groundSpeed / omegaA
     *          3. Calculate heading error: Nu = desired_heading - current_heading
     *          4. Compute lateral acceleration: latAccel = 2*sin(Nu)*groundSpeed*omegaA
     * 
     * @note Called at navigation loop rate, typically 10Hz
     * @note Crosstrack error set to zero in this mode
     */
    void update_heading_hold(int32_t navigation_heading_cd) override;
    
    /**
     * @brief Update L1 guidance for level flight on current heading
     * 
     * @details Maintains current heading with zero lateral acceleration demand.
     *          All navigation outputs are set to maintain straight flight with no turn demand.
     * 
     * @note Called when no navigation guidance needed (manual throttle/pitch control)
     */
    void update_level_flight(void) override;
    
    /**
     * @brief Check if aircraft has reached loiter circle
     * 
     * @return true if established on loiter circle, false if still capturing
     * 
     * @details Returns the _WPcircle flag set by update_loiter(). "Reached" is a fuzzy
     *          concept meaning the aircraft is on the circular path, not necessarily at
     *          a specific point on the circle.
     * 
     * @note "Reached" means on circular path, not at specific point
     */
    bool reached_loiter_target(void) override;

    /**
     * @brief Set default value for NAVL1_PERIOD parameter
     * 
     * @param[in] period Default L1 period in seconds
     * 
     * @details Used during vehicle initialization to set vehicle-specific default period
     *          before parameters are loaded from EEPROM. If a saved parameter value exists,
     *          it will override this default.
     * 
     * @note Must be called before parameters loaded from EEPROM
     */
    void set_default_period(float period) {
        _L1_period.set_default(period);
    }

    /**
     * @brief Mark navigation data as stale
     * 
     * @details Called by mission controller when the waypoint changes or navigation mode
     *          changes. Indicates that current navigation outputs are invalid until the
     *          next update_*() call processes the new navigation target.
     * 
     * @note Indicates outputs invalid until next update_*() call
     */
    void set_data_is_stale(void) override {
        _data_is_stale = true;
    }
    
    /**
     * @brief Check if navigation data is stale
     * 
     * @return true if waypoint changed but not yet processed by update_*()
     * 
     * @details Allows mission controller to verify that update_*() has executed and
     *          processed the new navigation target before using navigation outputs.
     */
    bool data_is_stale(void) const override {
        return _data_is_stale;
    }

    // this supports the NAVl1_* user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

    /**
     * @brief Enable reverse flight navigation
     * 
     * @param[in] reverse true for reverse flight (tail-first navigation)
     * 
     * @details Enables tail-first waypoint tracking by wrapping yaw by 180°. Used for
     *          special mission segments requiring reverse flight or during tailsitter
     *          transition maneuvers.
     * 
     * @note Used for special mission segments or tailsitter transition
     * @warning Reverse mode (_reverse=true) is experimental; test thoroughly in SITL
     */
    void set_reverse(bool reverse) override {
        _reverse = reverse;
    }

private:
    /**
     * Reference to AHRS providing position, velocity, attitude, and yaw
     * 
     * Used throughout L1 calculations to obtain:
     * - Current position (lat/lon/alt)
     * - Groundspeed vector (NED frame in m/s)
     * - Aircraft heading/yaw (radians)
     * - Pitch angle for roll calculation
     */
    AP_AHRS &_ahrs;

    /**
     * Pointer to TECS controller for target airspeed queries
     * 
     * Used to obtain target airspeed for altitude scaling calculations via EAS2TAS.
     * May be nullptr if TECS is not available.
     */
    const AP_TECS *_tecs;

    /**
     * Lateral acceleration demand in m/s², positive to right
     * 
     * Output of L1 guidance law representing the lateral acceleration needed to
     * curve the flight path toward the L1 reference point. This value is converted
     * to roll angle by nav_roll_cd() for the attitude controller.
     * 
     * Positive values indicate rightward acceleration in the body frame.
     */
    float _latAccDem;

    /**
     * L1 tracking distance in meters, dynamically updated each cycle
     * 
     * Calculated as: L1_dist = (damping * period * groundSpeed) / π
     * 
     * This distance determines how far ahead on the desired path the L1 reference
     * point is placed. Larger values result in gentler turns and earlier path capture,
     * while smaller values result in tighter tracking but more aggressive turns.
     * 
     * Updated every cycle based on current groundspeed and tuning parameters.
     */
    float _L1_dist;

    /**
     * Status flag: true when vehicle has reached and is circling loiter waypoint
     * 
     * Set by update_loiter() when the aircraft has captured and is established on
     * the loiter circle. Used by reached_loiter_target() for mission sequencing.
     * Includes hysteresis to prevent false negatives from wind gusts.
     */
    bool _WPcircle;

    /**
     * Bearing angle in radians to L1 reference point
     * 
     * The bearing from the aircraft's current position to the L1 reference point
     * on the desired path. This bearing indicates the direction the aircraft should
     * be heading to track the desired path.
     * 
     * Converted to centidegrees by nav_bearing_cd() for external use.
     */
    float _nav_bearing;

    /**
     * Bearing error angle (Nu) in radians, positive left of track
     * 
     * The angle between the velocity vector and the desired track direction.
     * This is the Nu angle from the L1 algorithm. Positive values indicate
     * the aircraft is heading to the left of the desired track.
     * 
     * Converted to centidegrees by bearing_error_cd() for external use.
     */
    float _bearing_error;

    /**
     * Perpendicular distance from desired path in meters
     * 
     * The crosstrack error is the perpendicular distance from the aircraft position
     * to the desired track line in the X-Y plane (NED frame). Positive values mean
     * the aircraft is to the right of the desired track.
     */
    float _crosstrack_error;

    /**
     * Direct bearing to target waypoint in centidegrees
     * 
     * The bearing from the aircraft to the target waypoint, not accounting for
     * L1 guidance. This is different from _nav_bearing which points to the L1
     * reference point on the path.
     */
    int32_t _target_bearing_cd;

    /**
     * L1 tracking loop period parameter in seconds
     * 
     * Primary tuning parameter controlling the tracking loop period. Larger values
     * result in gentler turns and more relaxed path following. Smaller values result
     * in tighter tracking but more aggressive maneuvers.
     * 
     * Parameter: NAVL1_PERIOD
     * Range: 1-60 seconds
     * Default: 17 seconds
     * Units: seconds (s)
     * 
     * @warning Too small values (<10s) can cause aggressive turns and potential stall
     */
    AP_Float _L1_period;
    
    /**
     * L1 tracking loop damping ratio parameter
     * 
     * Controls overshoot behavior in path tracking. Values less than 1.0 allow
     * some overshoot for faster convergence. Values equal to 1.0 provide critical
     * damping with no overshoot. Values greater than 1.0 provide over-damping.
     * 
     * Parameter: NAVL1_DAMPING
     * Range: 0.6-1.0
     * Default: 0.75
     * Units: dimensionless damping ratio
     */
    AP_Float _L1_damping;

    /**
     * Previous cycle's Nu (bearing error) angle
     * 
     * Used by _prevent_indecision() to detect oscillating turn decisions and
     * maintain stability near waypoints. Stores the Nu angle from the previous
     * navigation update cycle.
     */
    float _last_Nu;

    /**
     * Internal helper preventing oscillating turn decisions
     * 
     * @param[in,out] Nu Current bearing error angle (modified to prevent oscillation)
     * 
     * Checks if Nu changed sign while pointing away from target. If detected,
     * maintains previous Nu value to avoid back-and-forth turn direction changes
     * that can occur near waypoints or in turbulence.
     */
    void _prevent_indecision(float &Nu);

    /**
     * Crosstrack error integrator state in radians
     * 
     * Provides integral feedback to ensure crosstrack error converges to zero in
     * the presence of steady winds or biases. Anti-windup limited to ±0.1 radians.
     * Only active when Nu < 5° (aircraft nearly aligned with track).
     * 
     * Reset to zero when integrator gain changes or after long update gaps.
     */
    float _L1_xtrack_i = 0;
    
    /**
     * Crosstrack integrator gain parameter
     * 
     * Controls the rate at which the integrator corrects persistent crosstrack errors.
     * Higher values provide faster convergence but may cause oscillation if too high.
     * Set to 0 to disable the integrator.
     * 
     * Parameter: NAVL1_XTRACK_I_GAIN
     * Range: 0-0.1
     * Default: 0.02
     */
    AP_Float _L1_xtrack_i_gain;
    
    /**
     * Previous cycle's integrator gain value
     * 
     * Used to detect changes in _L1_xtrack_i_gain parameter. When gain changes,
     * the integrator is reset to zero for clean re-convergence with new tuning.
     */
    float _L1_xtrack_i_gain_prev = 0;
    
    /**
     * Timestamp of last update_waypoint() call in microseconds
     * 
     * Used to detect long gaps between updates (e.g., mode changes) and reinitialize
     * integrators to prevent wind-up during periods when L1 is not actively controlling.
     */
    uint32_t _last_update_waypoint_us;
    
    /**
     * Flag indicating navigation data not yet updated after waypoint change
     * 
     * Set by set_data_is_stale() when waypoint or navigation mode changes.
     * Cleared by update_*() calls after processing new navigation target.
     * Indicates that current navigation outputs should not be trusted.
     */
    bool _data_is_stale = true;

    /**
     * Sea-level bank angle limit for loiter in degrees
     * 
     * Parameter: NAVL1_LIM_BANK
     * 
     * If > 0, calculates safe loiter radius based on bank angle limit and current
     * airspeed. The minimum safe radius is computed as: r_min = v² / (g * tan(bank_limit))
     * 
     * If = 0, scales loiter radius directly by EAS2TAS² for altitude compensation only.
     * 
     * Units: degrees
     * Range: 0-89°
     * Default: 0° (altitude scaling only)
     * 
     * @note Value is sanitized to [0°, 89°] to prevent invalid calculations
     */
    AP_Float _loiter_bank_limit;

    /**
     * Remembers last loiter state to detect parameter changes
     * 
     * Structure storing the previous loiter configuration to provide hysteresis
     * in reached_loiter_target() decision. Prevents false negatives from wind
     * gusts or temporary disturbances.
     * 
     * Members:
     * - reached_loiter_target_ms: Timestamp when loiter circle was reached
     * - radius: Last commanded loiter radius
     * - direction: Last commanded direction (+1 clockwise, -1 counter-clockwise)
     * - center_WP: Last commanded center point location
     */
    struct {
        uint32_t reached_loiter_target_ms;  ///< Timestamp when loiter reached (ms)
        float radius;                        ///< Last commanded radius (m)
        int8_t direction;                    ///< Last commanded direction (±1)
        Location center_WP;                  ///< Last commanded center point
    } _last_loiter;

    /**
     * Reverse flight flag for tail-first navigation
     * 
     * When true, yaw is wrapped by 180° to enable tail-first waypoint tracking.
     * Used for special mission segments requiring reverse flight or during
     * tailsitter transition maneuvers.
     * 
     * @warning Experimental feature - test thoroughly in SITL before flight use
     */
    bool _reverse = false;
    
    /**
     * Internal helper returning AHRS yaw in radians
     * 
     * @return Yaw angle in radians, wrapped by π if _reverse is true
     * 
     * Returns the aircraft yaw from AHRS. When _reverse is true, adds π radians
     * (180°) to the yaw for tail-first navigation.
     */
    float get_yaw() const;
    
    /**
     * Internal helper returning AHRS yaw in centidegrees
     * 
     * @return Yaw angle in centidegrees, wrapped by 18000 if _reverse is true
     * 
     * Returns the aircraft yaw from AHRS in centidegrees. When _reverse is true,
     * adds 18000 centidegrees (180°) to the yaw for tail-first navigation.
     */
    int32_t get_yaw_sensor() const;
    
    /**
     * @section coordinate_systems Coordinate Systems and Units Conventions
     * 
     * **Coordinate Frames:**
     * - Position/waypoints: WGS84 lat/lon via Location class
     * - Navigation vectors: NED (North-East-Down) frame in meters
     * - Velocity: NED frame in m/s from AHRS groundspeed vector
     * - Lateral acceleration: Body frame (positive to right) in m/s²
     * 
     * **Distance Units:**
     * - Distances: meters (m)
     * - L1 distance: meters (m)
     * - Crosstrack error: meters (m)
     * - Loiter radius: meters (m)
     * 
     * **Angle Units:**
     * - Bearings (internal): radians, wrapped to ±π
     * - Bearings (API): centidegrees (×100), wrapped to ±18000
     * - Roll angle: centidegrees (×100), range ±9000 (±90°)
     * - Heading: radians internally, centidegrees in API
     * 
     * **Acceleration Units:**
     * - Lateral acceleration: m/s² (positive to right in body frame)
     * 
     * **Time Units:**
     * - L1 period: seconds (s)
     * - Timestamps: microseconds (µs)
     * 
     * **Dimensionless:**
     * - Damping ratio: dimensionless [0.6-1.0]
     * - EAS2TAS: ratio (equivalent airspeed to true airspeed)
     * 
     * @section safety_considerations Safety Considerations
     * 
     * @warning **L1_PERIOD is safety-critical:** Too small values cause aggressive turns
     *          and potential stall. NAVL1_PERIOD < 10s only for highly responsive aircraft.
     *          Use 15-20s for typical planes.
     * 
     * @warning **Crosstrack integrator:** Can cause overshoot if gain too high. Start with
     *          default 0.02 and increase cautiously if needed.
     * 
     * @warning **Coordinated flight assumption:** L1 guidance assumes coordinated turns.
     *          Uncoordinated flight (significant sideslip) degrades performance.
     * 
     * @warning **Low groundspeed handling:** At very low groundspeed (<0.1 m/s), L1 uses
     *          compass heading fallback which may be less accurate.
     * 
     * @warning **Altitude scaling:** Loiter radius automatically increased at altitude via
     *          EAS2TAS² scaling. May violate geofence if radius becomes too large. EAS2TAS²
     *          scaling means loiter radius grows significantly with altitude.
     * 
     * @warning **Reverse mode:** _reverse=true is experimental. Test thoroughly in SITL
     *          before any flight use.
     * 
     * @note **Source:** libraries/AP_L1_Control/AP_L1_Control.h:1-139,
     *       libraries/AP_L1_Control/AP_L1_Control.cpp:1-548
     */
};
