/**
 * @file mode_circle.cpp
 * @brief Implementation of Circle mode for Rover
 * 
 * @details Circle mode makes the rover drive in circles around a configurable center point.
 *          The rover maintains a specified radius from the center while traveling at a
 *          configured speed in either clockwise or counter-clockwise direction.
 *          
 *          This mode implements a two-stage approach:
 *          1. Drive-to-radius: Navigate from current position to the circle's edge
 *          2. Circling: Follow the circular path maintaining constant radius
 *          
 *          The circle navigation uses geometric calculations to determine target position,
 *          velocity (tangential to circle), and acceleration (centripetal toward center).
 *          The position controller continuously updates to track these targets.
 *          
 *          Key Parameters:
 *          - Center point: Geographic location around which to circle (Location)
 *          - Radius: Distance from center to maintain in meters (config.radius)
 *          - Speed: Tangential velocity around circle in m/s (config.speed)
 *          - Direction: Clockwise (CW) or Counter-Clockwise (CCW) (config.dir)
 *          - Rate: Angular velocity calculated from speed/radius in deg/s
 *          
 *          The algorithm uses target.yaw_rad to track the current angular position around
 *          the circle, incrementing it based on the vehicle's speed and radius to maintain
 *          circular motion.
 * 
 * @note Use Cases:
 *       - System testing: Validate navigation and control systems with predictable motion
 *       - Demonstrations: Visual display of rover capabilities
 *       - Search patterns: Circular search around a point of interest
 *       - Sensor calibration: Generate consistent motion for compass or IMU calibration
 * 
 * @warning Circle mode requires valid position estimates (GPS or other positioning).
 *          Mode will stop the vehicle if position becomes unavailable.
 * 
 * Source: Rover/mode_circle.cpp
 */

#include "Rover.h"

#define AR_CIRCLE_ACCEL_DEFAULT         1.0 // default acceleration in m/s/s if not specified by user
#define AR_CIRCLE_RADIUS_MIN            0.1 // minimum radius in meters

const AP_Param::GroupInfo ModeCircle::var_info[] = {

    // @Param: _RADIUS
    // @DisplayName: Circle Radius
    // @Description: Vehicle will circle the center at this distance
    // @Units: m
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_RADIUS", 1, ModeCircle, radius, 20),

    // @Param: _SPEED
    // @DisplayName: Circle Speed
    // @Description: Vehicle will move at this speed around the circle.  If set to zero WP_SPEED will be used
    // @Units: m/s
    // @Range: 0 10
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("_SPEED", 2, ModeCircle, speed, 0),

    // @Param: _DIR
    // @DisplayName: Circle Direction
    // @Description: Circle Direction
    // @Values: 0:Clockwise, 1:Counter-Clockwise
    // @User: Standard
    AP_GROUPINFO("_DIR", 3, ModeCircle, direction, 0),

    AP_GROUPEND
};

/**
 * @brief Constructor for Circle mode
 * 
 * @details Initializes Circle mode parameters to their default values as defined
 *          in the var_info parameter table (radius=20m, speed=0, direction=CW).
 */
ModeCircle::ModeCircle() : Mode()
{
    AP_Param::setup_object_defaults(this, var_info);
}

/**
 * @brief Get the distance threshold for considering vehicle on track along the circle
 * 
 * @details Returns a distance threshold used to determine when the vehicle has reached
 *          the edge of the circle during drive-to-radius phase and for detecting when
 *          the vehicle strays too far from the desired radius during circling.
 *          
 *          The threshold scales with circle size to provide appropriate tolerances:
 *          - Larger circles (radius >= 0.5m): 0.2m tolerance
 *          - Smaller circles (radius < 0.5m): 0.1m tolerance
 * 
 * @return Distance threshold in meters
 */
float ModeCircle::get_reached_distance() const
{
    if (config.radius >= 0.5) {
        return 0.2;
    }
    return 0.1;
}

/**
 * @brief Initialize circle mode with a specific center location, radius, and direction
 * 
 * @details This function is called when Circle mode is entered from Auto mode or other
 *          modes that specify a custom circle center. It overrides the default behavior
 *          of circling around the vehicle's current position.
 *          
 *          The function converts the geographic center location to NE coordinates relative
 *          to the EKF origin, validates and stores the radius (enforcing minimum radius),
 *          and initializes the target yaw to point from center to vehicle's current position.
 * 
 * @param[in] center_loc Geographic location (Location) of circle center point
 * @param[in] radius_m   Desired circle radius in meters (will be clamped to minimum)
 * @param[in] dir_ccw    true for counter-clockwise, false for clockwise direction
 * 
 * @return true if initialization successful, false if position unavailable or _enter() fails
 * 
 * @note Replaces use of _enter() when initialized from within Auto mode with DO_CIRCLE command
 */
bool ModeCircle::set_center(const Location& center_loc, float radius_m, bool dir_ccw)
{
    Vector2f center_pos_cm;
    if (!center_loc.get_vector_xy_from_origin_NE_cm(center_pos_cm)) {
        return false;
    }
    if (!_enter()) {
        return false;
    }

    // convert center position from cm to m
    config.center_pos = center_pos_cm * 0.01;

    // set radius
    config.radius = MAX(fabsf(radius_m), AR_CIRCLE_RADIUS_MIN);
    check_config_radius();

    // set direction
    config.dir = dir_ccw ? Direction::CCW : Direction::CW;

    // set target yaw rad (target point on circle)
    init_target_yaw_rad();

    // record center as location (only used for reporting)
    config.center_loc = center_loc;

    return true;
}

/**
 * @brief Initialize circle mode using vehicle's current position as circle center
 * 
 * @details Called when Circle mode is entered directly (not from Auto mode).
 *          Uses the vehicle's current position as the circle center and applies
 *          user-configured parameters for radius, speed, and direction.
 *          
 *          Initialization sequence:
 *          1. Capture current position as circle center (NE frame relative to origin)
 *          2. Apply radius from CIRCLE_RADIUS parameter (enforcing minimum)
 *          3. Set direction from CIRCLE_DIR parameter (0=CW, 1=CCW)
 *          4. Set speed from CIRCLE_SPEED or WP_SPEED if zero
 *          5. Check speed doesn't exceed lateral acceleration limits
 *          6. Initialize position controller with speed, acceleration, and jerk limits
 *          7. Reset tracking state (angle_total_rad, reached_edge, tracking_back)
 * 
 * @return true if initialization successful, false if position estimate unavailable
 * 
 * @note This is called at main loop rate during mode entry
 */
bool ModeCircle::_enter()
{
    // capture starting point and yaw
    if (!AP::ahrs().get_relative_position_NE_origin_float(config.center_pos)) {
        return false;
    }
    config.radius = MAX(fabsf(radius), AR_CIRCLE_RADIUS_MIN);
    check_config_radius();

    config.dir = (direction == 1) ? Direction::CCW : Direction::CW;
    config.speed = is_positive(speed) ? speed : g2.wp_nav.get_default_speed();
    target.yaw_rad = AP::ahrs().get_yaw_rad();
    target.speed = 0;

    // record center as location (only used for reporting)
    config.center_loc = rover.current_loc;

    // check speed around circle does not lead to excessive lateral acceleration
    check_config_speed();

    // reset tracking_back 
    tracking_back = false;

    // calculate speed, accel and jerk limits
    // otherwise the vehicle uses wp_nav default speed limit
    float atc_accel_max = MIN(g2.attitude_control.get_accel_max(), g2.attitude_control.get_decel_max());
    if (!is_positive(atc_accel_max)) {
        atc_accel_max = AR_CIRCLE_ACCEL_DEFAULT;
    }
    const float accel_max = is_positive(g2.wp_nav.get_default_accel()) ? MIN(g2.wp_nav.get_default_accel(), atc_accel_max) : atc_accel_max;
    const float jerk_max = is_positive(g2.wp_nav.get_default_jerk()) ? g2.wp_nav.get_default_jerk() : accel_max;

    // initialise position controller
    g2.pos_control.set_limits(config.speed, accel_max, g2.attitude_control.get_turn_lat_accel_max(), jerk_max);
    g2.pos_control.init();

    // initialise angles covered and reached_edge state
    angle_total_rad = 0;
    reached_edge = false;
    dist_to_edge_m = 0;

    return true;
}

/**
 * @brief Initialize target_yaw_rad based on vehicle's position relative to circle center
 * 
 * @details Calculates the angle from circle center to the vehicle's current position
 *          to determine the starting angular position on the circle. This angle (target.yaw_rad)
 *          is used to compute the target point on the circle's edge.
 *          
 *          Algorithm:
 *          1. Get current vehicle position in NE frame
 *          2. Calculate vector from circle center to vehicle
 *          3. Compute angle of this vector (atan2) to get target.yaw_rad
 *          4. Special cases: If no position or exactly at center, use vehicle's heading
 * 
 * @note target.yaw_rad represents the angle in radians from circle center to the target
 *       point on the circle's edge, measured from North in NE frame (0=North, π/2=East)
 */
void ModeCircle::init_target_yaw_rad()
{
    // if no position estimate use vehicle yaw
    Vector2f curr_pos_NE;
    if (!AP::ahrs().get_relative_position_NE_origin_float(curr_pos_NE)) {
        target.yaw_rad = AP::ahrs().get_yaw_rad();
        return;
    }

    // calc vector from circle center to vehicle
    Vector2f center_to_veh = (curr_pos_NE - config.center_pos);
    float dist_m = center_to_veh.length();

    // if current position is exactly at the center of the circle return vehicle yaw
    if (is_zero(dist_m)) {
        target.yaw_rad = AP::ahrs().get_yaw_rad();
    } else {
        target.yaw_rad = center_to_veh.angle();
    }
}

/**
 * @brief Main update function for Circle mode - called at main loop rate
 * 
 * @details Implements the circle navigation algorithm through a state-based approach.
 *          This function is called continuously while in Circle mode (typically 50Hz).
 *          
 *          Circle Navigation Algorithm:
 *          ==========================
 *          Stage 1: Drive to Radius (reached_edge = false)
 *          - Navigate from current position to the circle's edge
 *          - Target point is on circle at initial target.yaw_rad angle
 *          - Transitions to circling when within distance threshold of edge
 *          
 *          Stage 2: Circling (reached_edge = true)
 *          - Follow circular path by incrementing target.yaw_rad based on speed/radius
 *          - Calculate target position at angle target.yaw_rad on circle's edge
 *          - Compute tangential velocity (perpendicular to radius vector)
 *          - Apply centripetal acceleration (toward center, magnitude v²/r)
 *          - Position controller tracks these targets to maintain circular motion
 *          
 *          Adaptive Speed Control:
 *          If vehicle cannot maintain radius (dist_to_edge > 2*turn_radius):
 *          - Reduce speed by 20% to help vehicle track circle
 *          - Enter "tracking_back" mode to recover to proper radius
 *          - Resume normal circling when back within tolerance
 *          
 *          The geometric calculation maintains circular path through:
 *          - Angular rate = (tangential_speed / circumference) * 2π
 *          - Target angle increment = angular_rate * dt
 *          - Centripetal acceleration = speed² / radius (directed toward center)
 * 
 * @note Called at main loop rate (typically 50Hz for Rover)
 * @warning Requires valid position estimate - stops vehicle if position unavailable
 */
void ModeCircle::update()
{
    // Get current vehicle position in NE frame (meters from EKF origin)
    Vector2f curr_pos;
    const bool position_ok = AP::ahrs().get_relative_position_NE_origin_float(curr_pos);

    // Safety check: Stop vehicle if position estimate is unavailable
    // Without position, cannot determine location relative to circle center
    if (!position_ok) {
        stop_vehicle();
        return;
    }

    // Calculate current distance metrics for state management and monitoring:
    // - Vector from circle center to current vehicle position
    const Vector2f center_to_veh = curr_pos - config.center_pos;
    // - Distance to target point on circle (for navigation reporting)
    _distance_to_destination = (target.pos.tofloat() - curr_pos).length();
    // - Distance from vehicle to circle's edge (positive = outside, negative = inside)
    dist_to_edge_m = fabsf(center_to_veh.length() - config.radius);

    // State-based update: Select navigation algorithm based on current stage
    if (!reached_edge) {
        // Stage 1: Drive to circle's edge from starting position
        update_drive_to_radius();
    } else if (dist_to_edge_m > 2 * MAX(g2.turn_radius, get_reached_distance()) && !tracking_back) {
        // Adaptive control: Vehicle is too far outside circle and cannot maintain radius
        // Reduce speed by 20% to help vehicle track back to proper radius
        config.speed = 0.8 * config.speed;
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Circle: cannot keep up, slowing to %.1fm/s", config.speed);
        tracking_back = true;
    } else if (dist_to_edge_m < MAX(g2.turn_radius, get_reached_distance()) && tracking_back) {
        // Vehicle has recovered to within tolerance - resume normal circling
        tracking_back = false;
    } else {
        // Stage 2: Normal circling - follow circular path maintaining radius
        update_circling();
    }

    // Update position controller with latest targets (runs kinematic model)
    g2.pos_control.update(rover.G_Dt);

    // Extract desired motion from position controller for low-level control
    const float desired_speed = g2.pos_control.get_desired_speed();
    const float desired_turn_rate = g2.pos_control.get_desired_turn_rate_rads();

    // Command steering and throttle controllers to achieve desired motion
    calc_steering_from_turn_rate(desired_turn_rate);
    calc_throttle(desired_speed, true);
}

/**
 * @brief Navigate from current position to the edge of the circle
 * 
 * @details First stage of circle mode operation. Drives the vehicle from its starting
 *          position to a point on the circle's edge at the initial target.yaw_rad angle.
 *          
 *          Algorithm:
 *          1. Check if vehicle is within distance threshold of circle's edge
 *          2. If reached, set reached_edge flag to transition to circling stage
 *          3. Calculate target position: center + radius * [cos(yaw), sin(yaw)]
 *          4. Command position controller to navigate to target point
 *          
 *          The distance threshold accounts for vehicle's turn radius to ensure smooth
 *          transition from linear approach to circular motion.
 * 
 * @note Once reached_edge becomes true, mode switches to update_circling()
 */
void ModeCircle::update_drive_to_radius()
{
    // Check if vehicle has reached edge of circle (within tolerance threshold)
    // Use larger of turn_radius or reached_distance to ensure smooth transition
    const float dist_thresh_m = MAX(g2.turn_radius, get_reached_distance());
    reached_edge |= dist_to_edge_m <= dist_thresh_m;

    // Calculate target point's position on circle's edge
    // Position = center + radius * unit_vector(target.yaw_rad)
    target.pos = config.center_pos.topostype();
    target.pos.offset_bearing(degrees(target.yaw_rad), config.radius);

    // Command position controller to navigate to target point (simple waypoint navigation)
    g2.pos_control.input_pos_target(target.pos, rover.G_Dt);
}

/**
 * @brief Update position controller targets while following circular path
 * 
 * @details Second stage of circle mode - maintains circular motion around the center point.
 *          Implements circular motion through geometric calculations of position, velocity,
 *          and acceleration vectors that define motion along a circle.
 *          
 *          Circular Motion Geometry:
 *          =========================
 *          
 *          Angular Position (target.yaw_rad):
 *          - Represents angle from circle center to target point on edge
 *          - Measured from North in NE frame (0 = North, π/2 = East)
 *          - Incremented each timestep based on tangential speed and radius
 *          
 *          Angular Rate Calculation:
 *          - Circumference = 2π * radius
 *          - Angular rate (rad/s) = (speed / circumference) * 2π
 *          - Simplified: angular_rate = speed / radius
 *          - Direction multiplier: CW = +1, CCW = -1
 *          - Angle increment per timestep: angular_rate * dt
 *          
 *          Position Vector:
 *          - Target point on circle's edge at angle target.yaw_rad
 *          - Position = center + radius * [cos(yaw_rad), sin(yaw_rad)]
 *          - Calculated using offset_bearing() helper function
 *          
 *          Velocity Vector (Tangential):
 *          - Perpendicular to radius vector (90° rotation)
 *          - Magnitude = target.speed (tangential speed)
 *          - Direction = target.yaw_rad + 90° (tangent to circle)
 *          - Creates motion along circle's edge
 *          
 *          Acceleration Vector (Centripetal):
 *          - Primary component: Directed toward center (centripetal acceleration)
 *          - Magnitude = speed² / radius (required for circular motion)
 *          - Secondary component: Tangential acceleration (speed changes)
 *          - Combined and rotated to align with target.yaw_rad direction
 *          
 *          The position controller uses these three vectors (position, velocity, acceleration)
 *          to compute desired heading and speed that will track the circular path.
 * 
 * @note The centripetal acceleration (v²/r) is essential for maintaining circular motion.
 *       Without it, the vehicle would tend to move in a straight line (tangent to circle).
 * 
 * @note In tracking_back mode (vehicle too far from circle), target.yaw_rad is reset
 *       to point from center to vehicle, helping it recover to proper radius.
 */
void ModeCircle::update_circling()
{

    // Gradually accelerate tangential speed up to desired configured speed
    // Limit rate of change to 50% of max acceleration to prevent abrupt changes
    const float speed_change_max = (g2.pos_control.get_accel_max() * 0.5 * rover.G_Dt);
    const float accel_fb = constrain_float(config.speed - target.speed, -speed_change_max, speed_change_max);
    target.speed += accel_fb;

    // Calculate angular rate and update target angle (angular position around circle)
    if (!tracking_back) {
        // Normal circling mode: Advance angular position based on speed and radius
        
        // Calculate circle's circumference
        const float circumference = 2.0 * M_PI * config.radius;
        
        // Calculate angular rate (rad/s) from tangential speed
        // For circular motion: angular_rate = tangential_speed / radius
        // Direction: CW = positive (increasing angle), CCW = negative (decreasing angle)
        const float angular_rate_rad = (target.speed / circumference) * M_2PI * (config.dir == Direction::CW ? 1.0 : -1.0);
        
        // Calculate angle increment for this timestep
        const float angle_dt = angular_rate_rad * rover.G_Dt;
        
        // Update target angular position and wrap to [-π, π] range
        target.yaw_rad = wrap_PI(target.yaw_rad + angle_dt);
        
        // Accumulate total angle traveled (for monitoring/reporting)
        angle_total_rad += angle_dt;
    } else {
        // Tracking back mode: Reset target angle to point from center to vehicle
        // This helps vehicle recover to proper radius when it strays too far
        init_target_yaw_rad();
    }

    // Calculate target point's position on circle's edge
    // Position vector: center + radius * unit_vector(target.yaw_rad)
    target.pos = config.center_pos.topostype();
    target.pos.offset_bearing(degrees(target.yaw_rad), config.radius);

    // Calculate velocity vector (tangential to circle)
    // Start with speed in x-direction: [speed, 0]
    // Rotate by (target.yaw_rad + 90°) to make perpendicular to radius vector
    // This creates velocity tangent to circle at target point
    target.vel = Vector2f(target.speed, 0);
    target.vel.rotate(target.yaw_rad + radians(90));

    // Calculate acceleration vector (centripetal + tangential components)
    // X-component: -speed²/radius (centripetal, toward center, hence negative)
    // Y-component: accel_fb/dt (tangential acceleration for speed changes)
    // Rotate entire vector by target.yaw_rad to align with current angular position
    target.accel = Vector2f(-sq(target.speed) / config.radius, accel_fb / rover.G_Dt);
    target.accel.rotate(target.yaw_rad);

    // Command position controller with complete target state (pos, vel, accel)
    // Controller will compute desired heading and speed to track these targets
    g2.pos_control.set_pos_vel_accel_target(target.pos, target.vel, target.accel);

}

/**
 * @brief Return desired heading for reporting to ground station
 * 
 * @details Calculates the bearing from vehicle's current position to the target point
 *          on the circle's edge. Used for NAV_CONTROLLER_OUTPUT MAVLink message to
 *          provide ground station with navigation information.
 * 
 * @return Bearing to target in degrees, or 0 if position unavailable or at target
 */
float ModeCircle::wp_bearing() const
{
    Vector2f curr_pos_NE;
    if (!AP::ahrs().get_relative_position_NE_origin_float(curr_pos_NE)) {
        return 0;
    }
    // calc vector from vehicle to target point on circle
    Vector2f veh_to_center = (target.pos.tofloat() - curr_pos_NE);
    if (veh_to_center.is_zero()) {
        return 0;
    }
    return degrees(veh_to_center.angle());
}

/**
 * @brief Return navigation bearing based on position controller error
 * 
 * @details Calculates bearing of the position error vector (from current position
 *          to desired position) as reported by the position controller. Provides
 *          the direction the vehicle needs to travel to correct position error.
 * 
 * @return Bearing of position error vector in degrees, or 0 if no error
 */
float ModeCircle::nav_bearing() const
{
    // get position error as a vector from the current position to the target position
    const Vector2p pos_error = g2.pos_control.get_pos_error();
    if (pos_error.is_zero()) {
        return 0;
    }
    return degrees(pos_error.angle());
}

/**
 * @brief Get desired lateral acceleration from position controller
 * 
 * @details Returns the lateral (perpendicular to heading) acceleration that the
 *          position controller is commanding to track the circular path. Used for
 *          reporting and monitoring control performance.
 * 
 * @return Desired lateral acceleration in m/s²
 */
float ModeCircle::get_desired_lat_accel() const
{
    return g2.pos_control.get_desired_lat_accel();
}

/**
 * @brief Set desired tangential speed around the circle
 * 
 * @details Updates the configured speed for circular motion. The speed is validated
 *          against lateral acceleration limits (v²/r must not exceed max lateral accel).
 *          If speed limit is reduced, user is notified via GCS message.
 *          
 *          The position controller speed limit is also updated if the new speed
 *          exceeds the current limit.
 * 
 * @param[in] speed_ms Desired tangential speed in m/s (must be positive)
 * 
 * @return true if speed was accepted and applied, false if speed <= 0
 * 
 * @note Speed may be automatically reduced by check_config_speed() if it would
 *       cause excessive lateral acceleration (centripetal force v²/r too large)
 */
bool ModeCircle::set_desired_speed(float speed_ms)
{
    if (is_positive(speed_ms)) {
        config.speed = speed_ms;

        // check desired speed does not lead to excessive lateral acceleration
        check_config_speed();

        // update position controller limits if required
        if (config.speed > g2.pos_control.get_speed_max()) {
            g2.pos_control.set_limits(config.speed, g2.pos_control.get_accel_max(), g2.pos_control.get_lat_accel_max(), g2.pos_control.get_jerk_max());
        }
        return true;
    }
 
    return false;
}

/**
 * @brief Get current target location on circle's edge
 * 
 * @details Calculates the geographic location (latitude/longitude) of the current
 *          target point on the circle's edge at angle target.yaw_rad from center.
 *          Used for mission reporting and ground station display.
 * 
 * @param[out] destination Location object to be populated with target coordinates
 * 
 * @return true (always successful)
 */
bool ModeCircle::get_desired_location(Location& destination) const
{
    destination = config.center_loc;
    destination.offset_bearing(degrees(target.yaw_rad), config.radius);
    return true;
}

/**
 * @brief Limit configured speed to prevent excessive lateral acceleration
 * 
 * @details Validates that the configured speed does not cause lateral acceleration
 *          (centripetal acceleration = v²/r) to exceed the vehicle's maximum lateral
 *          acceleration limit from the attitude controller.
 *          
 *          Physics constraint:
 *          - Centripetal acceleration = speed² / radius
 *          - Must satisfy: speed² / radius <= max_lat_accel
 *          - Therefore: speed <= sqrt(max_lat_accel * radius)
 *          
 *          If configured speed exceeds this limit, it is automatically reduced and
 *          a warning message is sent to the ground station.
 * 
 * @pre config.radius must be set to circle radius
 * @pre attitude controller must be initialized with max lateral acceleration
 * 
 * @note Sends MAVLink warning message to GCS if speed is reduced
 */
void ModeCircle::check_config_speed()
{
    // calculate maximum speed based on radius and max lateral acceleration
    // Derivation: v²/r <= a_max  =>  v <= sqrt(a_max * r)
    const float speed_max = MAX(safe_sqrt(g2.attitude_control.get_turn_lat_accel_max() * config.radius), 0);

    if (config.speed > speed_max) {
        config.speed = speed_max;
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Circle: max speed is %4.1f", (double)config.speed);
    }
}

/**
 * @brief Ensure configured radius is not smaller than vehicle's turn radius
 * 
 * @details Validates that the circle radius is large enough for the vehicle to
 *          physically follow. The minimum radius is constrained by the vehicle's
 *          minimum turn radius (TURN_RADIUS parameter), which is determined by
 *          vehicle geometry and steering limits.
 *          
 *          Physical constraint:
 *          - Vehicle cannot follow circles smaller than its minimum turn radius
 *          - Attempting to do so would result in the vehicle cutting inside the circle
 *          
 *          If configured radius is too small, it is automatically increased to the
 *          minimum turn radius and a warning message is sent to the ground station.
 * 
 * @pre config.radius must be set to desired circle radius
 * @pre TURN_RADIUS parameter must be configured for vehicle
 * 
 * @note Sends MAVLink warning message to GCS if radius is increased
 */
void ModeCircle::check_config_radius()
{
    // ensure radius is at least as large as vehicle's turn radius
    if (config.radius < g2.turn_radius) {
        config.radius = g2.turn_radius;
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Circle: radius increased to TURN_RADIUS (%4.1f)", (double)g2.turn_radius);
    }
}
