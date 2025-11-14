/**
 * @file mode_guided.cpp
 * @brief Implementation of Guided mode accepting real-time navigation commands from GCS or companion computer
 * 
 * @details Guided mode enables external control of the rover through MAVLink commands from a ground
 *          control station (GCS) or companion computer. This mode supports multiple control schemes:
 *          - Position target (WP): Navigate to GPS waypoint using waypoint navigation controller
 *          - Heading and speed: Maintain specified heading (degrees) and speed (m/s)
 *          - Turn rate and speed: Execute turn at specified rate (deg/s) with speed (m/s)
 *          - Steering and throttle: Direct motor control with normalized values (-1 to +1)
 *          - Loiter: Station-keeping for boats
 *          - Stop: Bring vehicle to stop
 *          
 *          Coordinate Frames:
 *          - Position targets: Global coordinates (latitude/longitude in degrees)
 *          - Velocity: Earth frame (m/s north/east/down)
 *          - Heading: Degrees from north (0-360), stored as centidegrees internally
 *          - Turn rate: Degrees per second, stored as centidegrees per second internally
 *          
 *          Safety Features:
 *          - 3-second timeout on heading/turn rate commands (reverts to stop/loiter)
 *          - 3-second timeout on steering/throttle commands (reverts to stop/loiter)
 *          - Optional horizontal distance and time limits when called from AUTO mode
 *          
 * @note Integration with external control systems via MAVLink SET_POSITION_TARGET_LOCAL_NED,
 *       SET_POSITION_TARGET_GLOBAL_INT, and SET_ATTITUDE_TARGET messages. Companion computers
 *       can use this mode for vision-based navigation, obstacle avoidance, or custom behaviors.
 * 
 * @warning Guided mode bypasses normal mission waypoints and fence checks may still apply.
 *          External control systems must implement appropriate safety monitoring.
 * 
 * Source: Rover/mode_guided.cpp
 */
#include "Rover.h"

bool ModeGuided::_enter()
{
    // initialise submode to stop or loiter
    if (rover.is_boat()) {
        if (!start_loiter()) {
            start_stop();
        }
    } else {
        start_stop();
    }

    // initialise waypoint navigation library
    g2.wp_nav.init();

    send_notification = false;

    return true;
}

/**
 * @brief Main control update for Guided mode - called at scheduler rate (typically 50Hz)
 * 
 * @details Implements guided target following with multiple control schemes based on the
 *          current submode. Handles timeouts for real-time command streams and transitions
 *          to safe stop/loiter behavior when commands cease.
 *          
 *          Control Schemes:
 *          - WP: Follow position target using waypoint navigation with path planning
 *          - HeadingAndSpeed: Maintain specified heading and speed (external command stream)
 *          - TurnRateAndSpeed: Execute turn rate and speed (external command stream)
 *          - SteeringAndThrottle: Direct motor control (external command stream)
 *          - Loiter: Station-keeping for boats
 *          - Stop: Hold position or stop
 */
void ModeGuided::update()
{
    switch (_guided_mode) {
        case SubMode::WP:
        {
            // Waypoint Navigation Mode - Navigate to target position using position controller
            // This mode uses the waypoint navigation library to follow a path to the target location
            // with optional S-curve path planning and object avoidance
            
            // check if we've reached the destination
            if (!g2.wp_nav.reached_destination()) {
                // update navigation controller - calculates desired steering and throttle
                // to follow path to waypoint while respecting speed limits and turn radius
                navigate_to_waypoint();
            } else {
                // Destination reached - transition to stop or loiter
                
                // send notification to GCS that waypoint was reached (NAV_CONTROLLER_OUTPUT)
                if (send_notification) {
                    send_notification = false;
                    rover.gcs().send_mission_item_reached_message(0);
                }

                // we have reached the destination so stay here
                // boats loiter (station-keeping), ground vehicles stop
                if (rover.is_boat()) {
                    if (!start_loiter()) {
                        stop_vehicle();
                    }
                } else {
                    stop_vehicle();
                }
                // update distance to destination for telemetry reporting
                _distance_to_destination = rover.current_loc.get_distance(g2.wp_nav.get_destination());
            }
            break;
        }

        case SubMode::HeadingAndSpeed:
        {
            // Heading and Speed Mode - Maintain specified heading (degrees) and speed (m/s)
            // Used for external control where GCS/companion sends heading commands continuously
            // Target heading is in earth frame (degrees from north), speed in m/s
            
            // Safety timeout: stop vehicle if target not updated within 3 seconds
            // This prevents runaway behavior if external control link is lost
            if (have_attitude_target && (millis() - _des_att_time_ms) > 3000) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "target not received last 3secs, stopping");
                have_attitude_target = false;
            }
            if (have_attitude_target) {
                // Execute heading and speed control
                // calc_steering_to_heading: computes steering output to achieve desired heading
                // calc_throttle: computes throttle to achieve desired speed with speed nudge for pilot input
                calc_steering_to_heading(_desired_yaw_cd);
                calc_throttle(calc_speed_nudge(_desired_speed, is_negative(_desired_speed)), true);
            } else {
                // Timeout occurred - transition to safe state
                // boats loiter (station-keeping), ground vehicles stop
                if (rover.is_boat()) {
                    if (!start_loiter()) {
                        stop_vehicle();
                    }
                } else {
                    stop_vehicle();
                }
            }
            break;
        }

        case SubMode::TurnRateAndSpeed:
        {
            // Turn Rate and Speed Mode - Execute turn at specified rate (deg/s) with speed (m/s)
            // Used for velocity control where external system sends continuous turn rate commands
            // Turn rate is in deg/s (earth frame), positive = right turn
            
            // Safety timeout: stop vehicle if target not updated within 3 seconds
            // This prevents runaway behavior if external control link is lost
            if (have_attitude_target && (millis() - _des_att_time_ms) > 3000) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "target not received last 3secs, stopping");
                have_attitude_target = false;
            }
            if (have_attitude_target) {
                // Execute turn rate and speed control
                // get_steering_out_rate: rate controller computes steering to achieve desired yaw rate
                // Converts turn rate from centidegrees/s to radians/s for attitude controller
                // Returns normalized steering (-1 to +1), scaled to centidegrees for set_steering
                float steering_out = attitude_control.get_steering_out_rate(radians(_desired_yaw_rate_cds * 0.01f),
                                                                            g2.motors.limit.steer_left,
                                                                            g2.motors.limit.steer_right,
                                                                            rover.G_Dt);
                set_steering(steering_out * 4500.0f);
                // calc_throttle: computes throttle to achieve desired speed
                calc_throttle(calc_speed_nudge(_desired_speed, is_negative(_desired_speed)), true);
            } else {
                // Timeout occurred - transition to safe state
                // boats loiter (station-keeping), ground vehicles stop
                if (rover.is_boat()) {
                    if (!start_loiter()) {
                        stop_vehicle();
                    }
                } else {
                    stop_vehicle();
                }
            }
            break;
        }

        case SubMode::Loiter:
        {
            rover.mode_loiter.update();
            break;
        }

        case SubMode::SteeringAndThrottle:
        {
            // handle timeout
            if (_have_strthr && (AP_HAL::millis() - _strthr_time_ms) > 3000) {
                _have_strthr = false;
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "target not received last 3secs, stopping");
            }
            if (_have_strthr) {
                // pass latest steering and throttle directly to motors library
                g2.motors.set_steering(_strthr_steering * 4500.0f, false);
                g2.motors.set_throttle(_strthr_throttle * 100.0f);
            } else {
                // loiter or stop vehicle
                if (rover.is_boat()) {
                    if (!start_loiter()) {
                        stop_vehicle();
                    }
                } else {
                    stop_vehicle();
                }
            }
            break;
        }

        case SubMode::Stop:
            stop_vehicle();
            break;

        default:
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Unknown GUIDED mode");
            break;
    }
}

// return heading (in degrees) and cross track error (in meters) for reporting to ground station (NAV_CONTROLLER_OUTPUT message)
float ModeGuided::wp_bearing() const
{
    switch (_guided_mode) {
    case SubMode::WP:
        return g2.wp_nav.wp_bearing_cd() * 0.01f;
    case SubMode::HeadingAndSpeed:
    case SubMode::TurnRateAndSpeed:
        return 0.0f;
    case SubMode::Loiter:
        return rover.mode_loiter.wp_bearing();
    case SubMode::SteeringAndThrottle:
    case SubMode::Stop:
        return 0.0f;
    }

    // we should never reach here but just in case, return 0
    return 0.0f;
}

float ModeGuided::nav_bearing() const
{
    switch (_guided_mode) {
    case SubMode::WP:
        return g2.wp_nav.nav_bearing_cd() * 0.01f;
    case SubMode::HeadingAndSpeed:
    case SubMode::TurnRateAndSpeed:
        return 0.0f;
    case SubMode::Loiter:
        return rover.mode_loiter.nav_bearing();
    case SubMode::SteeringAndThrottle:
    case SubMode::Stop:
        return 0.0f;
    }

    // we should never reach here but just in case, return 0
    return 0.0f;
}

float ModeGuided::crosstrack_error() const
{
    switch (_guided_mode) {
    case SubMode::WP:
        return g2.wp_nav.crosstrack_error();
    case SubMode::HeadingAndSpeed:
    case SubMode::TurnRateAndSpeed:
        return 0.0f;
    case SubMode::Loiter:
        return rover.mode_loiter.crosstrack_error();
    case SubMode::SteeringAndThrottle:
    case SubMode::Stop:
        return 0.0f;
    }

    // we should never reach here but just in case, return 0
    return 0.0f;
}

float ModeGuided::get_desired_lat_accel() const
{
    switch (_guided_mode) {
    case SubMode::WP:
        return g2.wp_nav.get_lat_accel();
    case SubMode::HeadingAndSpeed:
    case SubMode::TurnRateAndSpeed:
        return 0.0f;
    case SubMode::Loiter:
        return rover.mode_loiter.get_desired_lat_accel();
    case SubMode::SteeringAndThrottle:
    case SubMode::Stop:
        return 0.0f;
    }

    // we should never reach here but just in case, return 0
    return 0.0f;
}

// return distance (in meters) to destination
float ModeGuided::get_distance_to_destination() const
{
    switch (_guided_mode) {
    case SubMode::WP:
        return _distance_to_destination;
    case SubMode::HeadingAndSpeed:
    case SubMode::TurnRateAndSpeed:
        return 0.0f;
    case SubMode::Loiter:
        return rover.mode_loiter.get_distance_to_destination();
    case SubMode::SteeringAndThrottle:
    case SubMode::Stop:
        return 0.0f;
    }

    // we should never reach here but just in case, return 0
    return 0.0f;
}

// return true if vehicle has reached or even passed destination
bool ModeGuided::reached_destination() const
{
    switch (_guided_mode) {
    case SubMode::WP:
        return g2.wp_nav.reached_destination();
    case SubMode::HeadingAndSpeed:
    case SubMode::TurnRateAndSpeed:
    case SubMode::Loiter:
    case SubMode::SteeringAndThrottle:
    case SubMode::Stop:
        return true;
    }

    // we should never reach here but just in case, return true is the safer option
    return true;
}

// set desired speed in m/s
bool ModeGuided::set_desired_speed(float speed)
{
    switch (_guided_mode) {
    case SubMode::WP:
        return g2.wp_nav.set_speed_max(speed);
    case SubMode::HeadingAndSpeed:
    case SubMode::TurnRateAndSpeed:
        // speed is set from mavlink message
        return false;
    case SubMode::Loiter:
        return rover.mode_loiter.set_desired_speed(speed);
    case SubMode::SteeringAndThrottle:
    case SubMode::Stop:
        // no speed control
        return false;
    }
    return false;
}

// get desired location
bool ModeGuided::get_desired_location(Location& destination) const
{
    switch (_guided_mode) {
    case SubMode::WP:
        if (g2.wp_nav.is_destination_valid()) {
            destination = g2.wp_nav.get_oa_destination();
            return true;
        }
        return false;
    case SubMode::HeadingAndSpeed:
    case SubMode::TurnRateAndSpeed:
        // not supported in these submodes
        return false;
    case SubMode::Loiter:
        // get destination from loiter
        return rover.mode_loiter.get_desired_location(destination);
    case SubMode::SteeringAndThrottle:
    case SubMode::Stop:
        // no desired location in this submode
        break;
    }

    // should never get here but just in case
    return false;
}

/**
 * @brief Set target waypoint for guided navigation in global coordinates
 * 
 * @details Configures the rover to navigate to a specified GPS location using the waypoint
 *          navigation controller. Supports two navigation methods based on GUID_OPTIONS:
 *          
 *          S-Curves Navigation (default):
 *          - Uses smooth S-curve path planning from current position to target
 *          - Supports object avoidance and fence avoidance
 *          - next_destination parameter enables look-ahead for smoother cornering
 *          - Better for planned paths but cannot handle rapid target updates
 *          
 *          Position Controller Input Shaping:
 *          - Uses position controller with input shaping for responsive tracking
 *          - Allows fast updates to target position (e.g., vision-based tracking)
 *          - Does not support object avoidance or look-ahead
 *          - Better for dynamic targets that change frequently
 *          
 *          This method transitions the mode to WP submode and enables mission item
 *          reached notification when the destination is achieved.
 * 
 * @param[in] destination      Target location in global coordinates (lat/lon/alt)
 * @param[in] next_destination Next waypoint for look-ahead (used only with S-curves)
 * 
 * @return true if target was set successfully, false if navigation library rejected target
 * 
 * @note Called by GCS MAVLink commands (MAV_CMD_NAV_WAYPOINT in guided mode) or
 *       companion computer position control. Coordinate frame is global WGS84.
 * 
 * @see use_scurves_for_navigation() for navigation method selection
 */
bool ModeGuided::set_desired_location(const Location &destination, Location next_destination)
{
    // Select navigation method based on GUID_OPTIONS parameter
    // Bit 0: Use S-curves for navigation vs position controller input shaping
    if (use_scurves_for_navigation()) {
        // S-curves navigation: smooth path planning with object avoidance
        // Provides next_destination for look-ahead to smooth cornering
        // Best for pre-planned paths where target doesn't change rapidly
        if (!g2.wp_nav.set_desired_location(destination, next_destination)) {
            return false;
        }
    } else {
        // Position controller input shaping: fast target updates
        // Does not support object avoidance but allows high-rate position updates
        // Best for vision-based tracking or rapidly changing targets
        if (!g2.wp_nav.set_desired_location_expect_fast_update(destination)) {
            return false;
        }
    }

    // Transition to waypoint navigation submode
    _guided_mode = SubMode::WP;
    // Enable notification to GCS when destination is reached
    send_notification = true;
#if HAL_LOGGING_ENABLED
    // Log the new target for post-flight analysis
    // Records: target lat/lon and maximum navigation speed
    rover.Log_Write_GuidedTarget((uint8_t)_guided_mode, Vector3f(destination.lat, destination.lng, 0), Vector3f(g2.wp_nav.get_speed_max(), 0.0f, 0.0f));
#endif
    return true;
}

/**
 * @brief Set desired heading and speed for rover control
 * 
 * @details Configures the rover to maintain a specified heading (earth frame) while traveling
 *          at a target speed. This control mode is intended for continuous command streams
 *          from an external controller (GCS or companion computer).
 *          
 *          Control Method:
 *          - Heading controller maintains desired yaw angle using steering output
 *          - Speed controller adjusts throttle to achieve target speed
 *          - Both controllers run simultaneously in update() at scheduler rate
 *          
 *          Safety Behavior:
 *          - Commands must be sent at least every 3 seconds or vehicle will stop/loiter
 *          - Timeout timestamp is updated each time this method is called
 *          - This prevents runaway if external control link is lost
 *          
 *          Coordinate Frame:
 *          - Heading: Earth frame, degrees from north (0-360), stored as centidegrees
 *          - Speed: m/s in vehicle forward direction (positive = forward, negative = reverse)
 * 
 * @param[in] yaw_angle_cd Target heading in centidegrees (0-36000, 0=north, 9000=east)
 * @param[in] target_speed Target speed in m/s (positive=forward, negative=reverse)
 * 
 * @note Called by MAVLink SET_ATTITUDE_TARGET or SET_POSITION_TARGET messages from GCS
 *       or companion computer. Requires continuous updates to maintain control (3s timeout).
 *       Heading wraps at 360 degrees.
 * 
 * @warning External control system must send commands at >0.33Hz or vehicle will stop.
 *          Do not use for pre-planned paths (use set_desired_location instead).
 */
void ModeGuided::set_desired_heading_and_speed(float yaw_angle_cd, float target_speed)
{
    // Transition to heading and speed control submode
    _guided_mode = SubMode::HeadingAndSpeed;
    // Record timestamp for timeout detection (commands must be sent every 3 seconds)
    _des_att_time_ms = AP_HAL::millis();

    // Store target values for use in update() method
    _desired_yaw_cd = yaw_angle_cd;      // Heading in centidegrees (earth frame)
    _desired_speed = target_speed;        // Speed in m/s (forward/reverse)
    have_attitude_target = true;          // Flag indicating valid target

#if HAL_LOGGING_ENABLED
    // Log the new target for post-flight analysis
    // Records: desired heading (centidegrees) and desired speed (m/s)
    rover.Log_Write_GuidedTarget((uint8_t)_guided_mode, Vector3f(_desired_yaw_cd, 0.0f, 0.0f), Vector3f(_desired_speed, 0.0f, 0.0f));
#endif
}

void ModeGuided::set_desired_heading_delta_and_speed(float yaw_delta_cd, float target_speed)
{
    // handle initialisation
    if (_guided_mode != SubMode::HeadingAndSpeed) {
        _guided_mode = SubMode::HeadingAndSpeed;
        _desired_yaw_cd = ahrs.yaw_sensor;
    }
    set_desired_heading_and_speed(wrap_180_cd(_desired_yaw_cd + yaw_delta_cd), target_speed);
}

/**
 * @brief Set desired turn rate and speed for velocity-based rover control
 * 
 * @details Configures the rover to execute a turn at a specified rate while maintaining
 *          target speed. This provides velocity-based control suitable for dynamic obstacle
 *          avoidance or path following where turn commands change rapidly.
 *          
 *          Control Method:
 *          - Turn rate controller uses attitude control rate loop to achieve yaw rate
 *          - Speed controller adjusts throttle to achieve target speed
 *          - Both controllers run simultaneously in update() at scheduler rate
 *          - Rate controller respects steering limits and accounts for vehicle dynamics
 *          
 *          Safety Behavior:
 *          - Commands must be sent at least every 3 seconds or vehicle will stop/loiter
 *          - Timeout timestamp is updated each time this method is called
 *          - This prevents runaway if external control link is lost
 *          
 *          Coordinate Frame:
 *          - Turn rate: deg/s in earth frame (positive = right/clockwise, negative = left/CCW)
 *          - Speed: m/s in vehicle forward direction (positive = forward, negative = reverse)
 *          
 *          Use Cases:
 *          - Vision-based navigation sending velocity commands
 *          - Obstacle avoidance algorithms
 *          - Path following with variable curvature
 * 
 * @param[in] turn_rate_cds Target turn rate in centidegrees/second (positive=right turn)
 * @param[in] target_speed  Target speed in m/s (positive=forward, negative=reverse)
 * 
 * @note Called by MAVLink SET_POSITION_TARGET messages with velocity and yaw rate fields.
 *       Requires continuous updates to maintain control (3s timeout). Turn rate is applied
 *       in earth frame, not body frame.
 * 
 * @warning External control system must send commands at >0.33Hz or vehicle will stop.
 *          Large turn rates may cause vehicle instability on high-speed ground vehicles.
 */
void ModeGuided::set_desired_turn_rate_and_speed(float turn_rate_cds, float target_speed)
{
    // Transition to turn rate and speed control submode
    _guided_mode = SubMode::TurnRateAndSpeed;
    // Record timestamp for timeout detection (commands must be sent every 3 seconds)
    _des_att_time_ms = AP_HAL::millis();

    // Store target values for use in update() method
    _desired_yaw_rate_cds = turn_rate_cds;  // Turn rate in centidegrees/s (earth frame)
    _desired_speed = target_speed;           // Speed in m/s (forward/reverse)
    have_attitude_target = true;             // Flag indicating valid target

#if HAL_LOGGING_ENABLED
    // Log the new target for post-flight analysis
    // Records: desired turn rate (centidegrees/s) and desired speed (m/s)
    rover.Log_Write_GuidedTarget((uint8_t)_guided_mode, Vector3f(_desired_yaw_rate_cds, 0.0f, 0.0f), Vector3f(_desired_speed, 0.0f, 0.0f));
#endif
}

// set steering and throttle (both in the range -1 to +1)
void ModeGuided::set_steering_and_throttle(float steering, float throttle)
{
    _guided_mode = SubMode::SteeringAndThrottle;
    _strthr_time_ms = AP_HAL::millis();
    _strthr_steering = constrain_float(steering, -1.0f, 1.0f);
    _strthr_throttle = constrain_float(throttle, -1.0f, 1.0f);
    _have_strthr = true;
}

bool ModeGuided::start_loiter()
{
    if (rover.mode_loiter.enter()) {
        _guided_mode = SubMode::Loiter;
        return true;
    }
    return false;
}


// start stopping vehicle as quickly as possible
void ModeGuided::start_stop()
{
    _guided_mode = SubMode::Stop;
}

// set guided timeout and movement limits
void ModeGuided::limit_set(uint32_t timeout_ms, float horiz_max)
{
    limit.timeout_ms = timeout_ms;
    limit.horiz_max = horiz_max;
}

// clear/turn off guided limits
void ModeGuided::limit_clear()
{
    limit.timeout_ms = 0;
    limit.horiz_max = 0.0f;
}

// initialise guided start time and location as reference for limit checking
//  only called from AUTO mode's start_guided method
void ModeGuided::limit_init_time_and_location()
{
    limit.start_time_ms = AP_HAL::millis();
    limit.start_loc = rover.current_loc;
}

// returns true if guided mode has breached a limit
bool ModeGuided::limit_breached() const
{
    // check if we have passed the timeout
    if ((limit.timeout_ms > 0) && (millis() - limit.start_time_ms >= limit.timeout_ms)) {
        return true;
    }

    // check if we have gone beyond horizontal limit
    if (is_positive(limit.horiz_max)) {
        return (rover.current_loc.get_distance(limit.start_loc) > limit.horiz_max);
    }

    // if we got this far we must be within limits
    return false;
}

// returns true if GUID_OPTIONS bit set to use scurve navigation instead of position controller input shaping
// scurves provide path planning and object avoidance but cannot handle fast updates to the destination (for fast updates use position controller input shaping)
bool ModeGuided::use_scurves_for_navigation() const
{
    return ((g2.guided_options.get() & uint32_t(Options::SCurvesUsedForNavigation)) != 0);
}
