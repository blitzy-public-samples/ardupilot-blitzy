/**
 * @file mode_loiter.cpp
 * @brief Implementation of Loiter mode for Rover
 * 
 * @details Loiter mode actively maintains the vehicle's position at a commanded
 *          location using GPS position control. Unlike Hold mode which simply stops
 *          the vehicle, Loiter continuously navigates to hold position at a specific
 *          loiter point, compensating for disturbances like wind or slopes.
 * 
 *          Key behaviors:
 *          - Navigates to loiter point if outside loiter radius
 *          - Actively holds position when within loiter radius
 *          - Supports sailboat-specific behaviors (tacking, wind heading)
 *          - Can reverse towards destination if configured
 * 
 *          Configuration parameters:
 *          - LOIT_RADIUS (g2.loit_radius): Radius in meters within which vehicle
 *            maintains position. Overridden by sailboat loiter radius if tacking enabled.
 *          - LOIT_SPEED_GAIN (g2.loiter_speed_gain): P controller gain converting
 *            distance error to desired speed
 *          - LOIT_TYPE (g2.loit_type): Loiter type - 0=forward/reverse, 1=forward only,
 *            2=reverse only
 * 
 * @note Difference from Hold mode: Loiter actively navigates to maintain position at
 *       a specific GPS coordinate, while Hold simply stops the vehicle in place without
 *       GPS position correction. Loiter continuously adjusts heading and throttle to
 *       counteract drift, making it suitable for holding station in challenging conditions.
 * 
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#include "Rover.h"

/**
 * @brief Enter Loiter mode - initializes loiter destination and desired state
 * 
 * @details Calculates a reasonable stopping point as the loiter destination,
 *          initializes desired speed to current speed for smooth transition,
 *          and sets desired heading to current heading.
 * 
 * @return true if mode initialization successful, false if stopping location
 *         could not be calculated
 */
bool ModeLoiter::_enter()
{
    // set _destination to reasonable stopping point
    if (!g2.wp_nav.get_stopping_location(_destination)) {
        return false;
    }

    // initialise desired speed to current speed
    if (!attitude_control.get_forward_speed(_desired_speed)) {
        _desired_speed = 0.0f;
    }

    // initialise heading to current heading
    _desired_yaw_cd = ahrs.yaw_sensor;

    return true;
}

/**
 * @brief Main update loop for Loiter mode - implements GPS position hold control
 * 
 * @details This function runs at the main loop rate (typically 50Hz) and implements
 *          the loiter navigation algorithm:
 * 
 *          Algorithm overview:
 *          1. Calculate distance to loiter destination point
 *          2. If within loiter radius: Slow to stop (or minimal speed for sailboats)
 *          3. If outside loiter radius: Navigate towards destination using P controller
 *             - Calculate desired speed based on distance error
 *             - Calculate bearing to destination
 *             - Optionally reverse if destination is behind vehicle
 *             - Reduce speed for large heading errors
 *          4. Apply sailboat-specific logic (tacking, wind avoidance)
 *          5. Execute steering and throttle control
 * 
 *          Position control uses a simple P controller: desired_speed = distance_error * gain
 *          Heading control points vehicle towards loiter destination
 *          Speed is reduced when heading error is large to improve tracking
 * 
 * @note Called at main loop rate by the mode state machine
 * @note Sailboat support: Uses tacking when enabled, points into wind when within radius
 * 
 * @see calc_steering_to_heading() for steering controller
 * @see calc_throttle() for throttle controller
 */
void ModeLoiter::update()
{
    // Calculate distance (in meters) from current position to loiter destination
    // This is the position error that drives the P controller
    _distance_to_destination = rover.current_loc.get_distance(_destination);

    // Determine loiter radius: use sailboat-specific radius if tacking, otherwise use LOIT_RADIUS parameter
    // Sailboat loiter radius is typically larger to accommodate tacking maneuvers
    const float loiter_radius = g2.sailboat.tack_enabled() ? g2.sailboat.get_loiter_radius() : g2.loit_radius;

    // WITHIN LOITER RADIUS: Slow to a stop and maintain position
    // When vehicle reaches loiter point, gradually reduce speed to zero (or minimal speed for sailboats)
    if (_distance_to_destination <= loiter_radius) {
        // Sailboats maintain minimal forward speed (0.1 m/s) to maintain steerage
        // Non-sailboats come to complete stop (0.0 m/s)
        // This prevents sailboats from stalling and losing maneuverability
        const float desired_speed_within_radius = g2.sailboat.tack_enabled() ? 0.1f : 0.0f;
        
        // Slew desired speed smoothly towards target to avoid abrupt throttle changes
        // Acceleration limiting ensures comfortable deceleration
        _desired_speed = attitude_control.get_desired_speed_accel_limited(desired_speed_within_radius, rover.G_Dt);

        // Special sailboat behavior: If sail is enabled but not actively tacking,
        // point bow into the wind to minimize drift and maintain safe weathervaning position
        // This prevents the sail from catching wind while loitering
        if (!g2.sailboat.tack_enabled() && g2.sailboat.sail_enabled()) {
            _desired_yaw_cd = degrees(g2.windvane.get_true_wind_direction_rad()) * 100.0f;
        }
    } else {
        // OUTSIDE LOITER RADIUS: Navigate towards loiter destination point
        // P (Proportional) controller: Convert distance error to desired speed
        // Formula: speed = (distance - radius) * gain, capped at maximum waypoint speed
        // The loiter_radius is subtracted so speed reaches zero at the radius boundary
        // LOIT_SPEED_GAIN parameter tunes aggressiveness of approach
        _desired_speed = MIN((_distance_to_destination - loiter_radius) * g2.loiter_speed_gain, g2.wp_nav.get_default_speed());

        // Calculate bearing from current position to loiter destination
        // This becomes the desired heading to point towards the loiter point
        _desired_yaw_cd = rover.current_loc.get_bearing_to(_destination);
        
        // Calculate heading error (difference between desired and actual heading)
        // Wrapped to ±180° range for shortest rotation direction
        float yaw_error_cd = wrap_180_cd(_desired_yaw_cd - ahrs.yaw_sensor);
        
        // REVERSING LOGIC: If destination is behind vehicle, optionally reverse towards it
        // This avoids unnecessary 180° turns for vehicles that can reverse efficiently
        // Conditions for reversing:
        // - (yaw_error > 90° AND LOIT_TYPE=0 "forward/reverse") OR
        // - LOIT_TYPE=2 "reverse only"
        // 9000 centidegrees = 90 degrees
        if ((fabsf(yaw_error_cd) > 9000 && g2.loit_type == 0) || g2.loit_type == 2) {
            // Reverse the desired heading by 180° (18000 centidegrees)
            _desired_yaw_cd = wrap_180_cd(_desired_yaw_cd + 18000);
            yaw_error_cd = wrap_180_cd(_desired_yaw_cd - ahrs.yaw_sensor);
            // Negate speed to move in reverse
            _desired_speed = -_desired_speed;
        }

        // SPEED REDUCTION for large heading errors: Slow down when not pointing at destination
        // This improves path tracking by preventing the vehicle from driving past the target
        // while turning to face it. Linear reduction from full speed at 0° error to 50% at 90° error.
        // Examples: 45° error → 75% speed, 90° error → 50% speed
        // Formula: ratio = 1.0 - (|error|/90°) * 0.5, clamped to [0.5, 1.0]
        float yaw_error_ratio = 1.0f - constrain_float(fabsf(yaw_error_cd / 9000.0f), 0.0f, 1.0f) * 0.5f;
        _desired_speed *= yaw_error_ratio;
    }

    // Initialize turn rate limit (0 = no limit, allows maximum turning performance)
    // Will be overridden to pivot rate if sailboat is tacking
    float turn_rate = 0.0;

    // SAILBOAT WIND AVOIDANCE: Prevent sailing directly into no-go zone
    // Sailboats cannot sail directly into the wind (typically ±45° of wind direction)
    // If desired heading is in the no-go zone, calculate indirect tacking route
    if (g2.sailboat.use_indirect_route(_desired_yaw_cd)) {
        // Calculate alternative heading that avoids no-go zone
        // This will be a tacking angle (typically 45° off the wind)
        _desired_yaw_cd = g2.sailboat.calc_heading(_desired_yaw_cd);
        
        // If actively tacking (transitioning through wind), limit turn rate
        // Slow pivot turns prevent sail damage and maintain control during tack
        if (g2.sailboat.tacking()) {
            turn_rate = g2.wp_nav.get_pivot_rate();
        }
    }

    // Execute low-level control using calculated desired heading and speed
    // Steering controller: Converts desired heading to steering servo output
    // Throttle controller: Converts desired speed to throttle/motor output
    // Second parameter (turn_rate) limits maximum turn rate if non-zero
    // Third parameter (true) enables braking for accurate speed control
    calc_steering_to_heading(_desired_yaw_cd, turn_rate);
    calc_throttle(_desired_speed, true);
}

/**
 * @brief Get the loiter destination location
 * 
 * @details Returns the GPS coordinate that the vehicle is attempting to hold position at.
 *          This is set during mode entry to a calculated stopping point and remains
 *          constant throughout the loiter.
 * 
 * @param[out] destination Location object to be filled with loiter destination coordinates
 * 
 * @return true (always succeeds as loiter destination is always valid once mode is entered)
 * 
 * @note Used by external systems to query where the vehicle is loitering
 * @see _enter() for how loiter destination is initially calculated
 */
bool ModeLoiter::get_desired_location(Location& destination) const
{
    destination = _destination;
    return true;
}
