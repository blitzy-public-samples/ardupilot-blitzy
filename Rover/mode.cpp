/**
 * @file mode.cpp
 * @brief Implementation of Mode base class common functionality for Rover
 * 
 * @details This file implements the Mode base class which provides common
 *          functionality shared across all rover flight modes. Key responsibilities include:
 *          - Mode entry/exit validation and initialization
 *          - Pilot input decoding and processing (steering, throttle, lateral)
 *          - Throttle calculation with speed control and motor limiting
 *          - Steering calculation from various inputs (turn rate, heading, lateral acceleration)
 *          - Navigation support functions (waypoint bearing, cross-track error)
 *          - Vehicle-specific handling (skid steering, sailboats, balance bots)
 * 
 *          The Mode class serves as the foundation for all specific mode implementations
 *          (Manual, Auto, Guided, RTL, etc.) and ensures consistent behavior across modes.
 * 
 * @note This is called by all derived mode classes throughout the main loop
 * @warning Modifications to core steering/throttle calculations affect vehicle stability
 * 
 * Source: Rover/mode.cpp
 */

#include "Rover.h"

/**
 * @brief Mode constructor - initializes references to rover subsystems
 * 
 * @details Sets up reference members to avoid repeated singleton lookups.
 *          All modes share these common subsystem references for efficient access.
 */
Mode::Mode() :
    ahrs(rover.ahrs),
    g(rover.g),
    g2(rover.g2),
    channel_steer(rover.channel_steer),
    channel_throttle(rover.channel_throttle),
    channel_lateral(rover.channel_lateral),
    channel_roll(rover.channel_roll),
    channel_pitch(rover.channel_pitch),
    channel_walking_height(rover.channel_walking_height),
    attitude_control(g2.attitude_control)
{ }

/**
 * @brief Exit the current mode and perform cleanup
 * 
 * @details Called when switching away from this mode to another mode.
 *          Delegates to the mode-specific _exit() implementation for
 *          mode-specific cleanup tasks (e.g., stopping navigation,
 *          clearing waypoints, resetting controllers).
 * 
 * @note This is called before the new mode's enter() is called
 * @see enter()
 */
void Mode::exit()
{
    // Call sub-class specific exit implementation
    // Each mode overrides _exit() to perform its own cleanup
    _exit();
}

/**
 * @brief Enter this mode with validation checks
 * 
 * @details Performs mode entry validation and initialization. The entry process:
 *          1. Validates EKF state estimates if armed (position/velocity requirements)
 *          2. Calls mode-specific _enter() implementation
 *          3. Performs common initialization (reversed flag, sailboat tacking)
 * 
 *          Mode entry checks are bypassed when disarmed to allow mode switching
 *          during pre-flight configuration. When armed, modes requiring position
 *          or velocity estimates will reject entry if EKF quality is insufficient.
 * 
 * @return true if mode entry successful, false if checks fail or mode rejects entry
 * 
 * @note Mode switches are always allowed when disarmed
 * @warning Failed mode entry leaves vehicle in previous mode - ensure failsafe handling
 * 
 * @see exit(), requires_position(), requires_velocity()
 */
bool Mode::enter()
{
    // Allow switching to any mode if disarmed. When disarmed, we rely on arming checks
    // to validate vehicle state before allowing flight. This permits configuration and
    // testing of modes without requiring full sensor availability.
    const bool ignore_checks = !hal.util->get_soft_armed();
    
    if (!ignore_checks) {
        // When armed, perform EKF state validation before mode entry
        
        // Get EKF filter status containing quality flags for position/velocity estimates
        nav_filter_status filt_status;
        rover.ahrs.get_filter_status(filt_status);

        // Check position estimate quality
        // Requires: EKF origin set AND at least one horizontal position source valid AND no EKF failsafe active
        // Position sources include GPS, optical flow, visual odometry, external nav, etc.
        const bool position_ok = rover.ekf_position_ok() && !rover.failsafe.ekf;
        
        // Reject entry to modes requiring position if estimate quality insufficient
        // Prevents navigation modes (Auto, Guided, RTL) from operating without valid position
        if (requires_position() && !position_ok) {
            return false;
        }

        // Check velocity estimate (required for some modes like Loiter)
        // If we have position estimate, velocity estimate should also be available
        // horiz_vel flag indicates horizontal velocity estimate is valid
        if (requires_velocity() && !position_ok && !filt_status.flags.horiz_vel) {
            return false;
        }
    }

    // Call mode-specific entry implementation
    // Each derived mode overrides _enter() to perform its own initialization
    bool ret = _enter();

    // Perform common initialization for all modes if entry succeeded
    if (ret) {
        // Initialize reversed driving flag based on mode requirements
        // Used by modes that support backing up (Auto with DO_SET_REVERSE)
        init_reversed_flag();

        // Clear any active sailboat tacking maneuvers from previous mode
        // Ensures clean state when entering new mode
        g2.sailboat.clear_tack();
    }

    return ret;
}

/**
 * @brief Decode pilot steering and throttle inputs from RC channels
 * 
 * @details Reads RC input channels and applies configured input type processing.
 *          Handles failsafe conditions and special steering configurations.
 * 
 *          Output ranges:
 *          - steering_out: -4500 to +4500 (centidegrees, positive = clockwise/right)
 *          - throttle_out: -100 to +100 (percent, positive = forward)
 * 
 *          Pilot steering types (PILOT_STEER_TYPE parameter):
 *          - DEFAULT: Normal steering, reverses direction when backing up (tank controls)
 *          - TWO_PADDLES: Skid-steer input (left/right paddles control individual tracks)
 *          - DIR_UNCHANGED_WHEN_REVERSING: Steering direction constant regardless of direction
 * 
 * @param[out] steering_out Pilot steering command in centidegrees (-4500 to +4500)
 * @param[out] throttle_out Pilot throttle command in percent (-100 to +100)
 * 
 * @note Returns zero outputs during RC failsafe
 * @warning Steering direction reversal with throttle depends on PILOT_STEER_TYPE setting
 * 
 * @see get_pilot_desired_steering_and_throttle()
 */
void Mode::get_pilot_input(float &steering_out, float &throttle_out) const
{
    // Check for RC failsafe - return safe defaults (no movement)
    // FAILSAFE_EVENT_THROTTLE indicates loss of RC signal or invalid RC data
    if (rover.failsafe.bits & FAILSAFE_EVENT_THROTTLE) {
        steering_out = 0;
        throttle_out = 0;
        return;
    }

    // Apply RC input processing based on configured steering type
    switch ((PilotSteerType)g.pilot_steer_type.get())
    {
        case PilotSteerType::DEFAULT:
        case PilotSteerType::DIR_REVERSED_WHEN_REVERSING:
        default: {
            // Standard tank-style controls: steering reverses when backing up
            // This provides intuitive control where stick right always turns vehicle's front right
            // even when reversing (similar to car driving in reverse)
            throttle_out = rover.channel_throttle->get_control_in();
            const float steering_dir = is_negative(throttle_out) ? -1 : 1;
            steering_out = steering_dir * rover.channel_steer->get_control_in();
            break;
        }

        case PilotSteerType::TWO_PADDLES: {
            // Two-paddle skid-steer input (common on skid-steer loaders and tanks)
            // Left stick controls left track, right stick controls right track
            // Differential mixing converts to steering + throttle:
            // steering = left-paddle - right-paddle (differential between tracks)
            // throttle = average(left-paddle, right-paddle) (mean forward speed)
            const float left_paddle = rover.channel_steer->norm_input_dz();
            const float right_paddle = rover.channel_throttle->norm_input_dz();

            // Average of paddles gives forward/reverse command
            throttle_out = 0.5f * (left_paddle + right_paddle) * 100.0f;
            // Difference of paddles gives turn command (positive = right turn)
            steering_out = (left_paddle - right_paddle) * 0.5f * 4500.0f;
            break;
        }

        case PilotSteerType::DIR_UNCHANGED_WHEN_REVERSING: {
            // Direct steering mode: steering direction does not reverse when backing up
            // Useful for vehicles where driver wants absolute steering control
            // (e.g., competition robots, precise maneuvering applications)
            throttle_out = rover.channel_throttle->get_control_in();
            steering_out = rover.channel_steer->get_control_in();
            break;
        }
    }
}

// decode pilot steering and throttle inputs and return in steer_out and throttle_out arguments
// steering_out is in the range -4500 ~ +4500 with positive numbers meaning rotate clockwise
// throttle_out is in the range -100 ~ +100
void Mode::get_pilot_desired_steering_and_throttle(float &steering_out, float &throttle_out) const
{
    // do basic conversion
    get_pilot_input(steering_out, throttle_out);

    // for skid steering vehicles, if pilot commands would lead to saturation
    // we proportionally reduce steering and throttle
    if (g2.motors.have_skid_steering()) {
        const float steer_normalised = constrain_float(steering_out / 4500.0f, -1.0f, 1.0f);
        const float throttle_normalised = constrain_float(throttle_out * 0.01f, -1.0f, 1.0f);
        const float saturation_value = fabsf(steer_normalised) + fabsf(throttle_normalised);
        if (saturation_value > 1.0f) {
            steering_out /= saturation_value;
            throttle_out /= saturation_value;
        }
    }

    // check for special case of input and output throttle being in opposite directions
    float throttle_out_limited = g2.motors.get_slew_limited_throttle(throttle_out, rover.G_Dt);
    if ((is_negative(throttle_out) != is_negative(throttle_out_limited)) &&
        (g.pilot_steer_type == PilotSteerType::DEFAULT ||
         g.pilot_steer_type == PilotSteerType::DIR_REVERSED_WHEN_REVERSING)) {
        steering_out *= -1;
    }
    throttle_out = throttle_out_limited;
}

// decode pilot steering and return steering_out and speed_out (in m/s)
void Mode::get_pilot_desired_steering_and_speed(float &steering_out, float &speed_out) const
{
    float desired_throttle;
    get_pilot_input(steering_out, desired_throttle);
    speed_out = desired_throttle * 0.01f * calc_speed_max(g.speed_cruise, g.throttle_cruise * 0.01f);
    // check for special case of input and output throttle being in opposite directions
    float speed_out_limited = g2.attitude_control.get_desired_speed_accel_limited(speed_out, rover.G_Dt);
    if ((is_negative(speed_out) != is_negative(speed_out_limited)) &&
        (g.pilot_steer_type == PilotSteerType::DEFAULT ||
         g.pilot_steer_type == PilotSteerType::DIR_REVERSED_WHEN_REVERSING)) {
        steering_out *= -1;
    }
    speed_out = speed_out_limited;
}

// decode pilot lateral movement input and return in lateral_out argument
void Mode::get_pilot_desired_lateral(float &lateral_out) const
{
    // no RC input means no lateral input
    if ((rover.failsafe.bits & FAILSAFE_EVENT_THROTTLE) || (rover.channel_lateral == nullptr)) {
        lateral_out = 0;
        return;
    }

    // get pilot lateral input
    lateral_out = rover.channel_lateral->get_control_in();
}

// decode pilot's input and return heading_out (in cd) and speed_out (in m/s)
void Mode::get_pilot_desired_heading_and_speed(float &heading_out, float &speed_out) const
{
    // get steering and throttle in the -1 to +1 range
    float desired_steering = constrain_float(rover.channel_steer->norm_input_dz(), -1.0f, 1.0f);
    float desired_throttle = constrain_float(rover.channel_throttle->norm_input_dz(), -1.0f, 1.0f);

    // handle two paddle input
    if (g.pilot_steer_type == PilotSteerType::TWO_PADDLES) {
        const float left_paddle = desired_steering;
        const float right_paddle = desired_throttle;
        desired_steering = (left_paddle - right_paddle) * 0.5f;
        desired_throttle = (left_paddle + right_paddle) * 0.5f;
    }

    // calculate angle of input stick vector
    heading_out = wrap_360_cd(rad_to_cd(atan2f(desired_steering, desired_throttle)));

    // calculate throttle using magnitude of input stick vector
    const float throttle = MIN(safe_sqrt(sq(desired_throttle) + sq(desired_steering)), 1.0f);
    speed_out = throttle * calc_speed_max(g.speed_cruise, g.throttle_cruise * 0.01f);
}

// decode pilot roll and pitch inputs and return in roll_out and pitch_out arguments
// outputs are in the range -1 to +1
void Mode::get_pilot_desired_roll_and_pitch(float &roll_out, float &pitch_out) const
{
    if (channel_roll != nullptr) {
        roll_out = channel_roll->norm_input();
    } else {
        roll_out = 0.0f;
    }
    if (channel_pitch != nullptr) {
        pitch_out = channel_pitch->norm_input();
    } else {
        pitch_out = 0.0f;
    }
}

// decode pilot walking_height inputs and return in walking_height_out arguments
// outputs are in the range -1 to +1
void Mode::get_pilot_desired_walking_height(float &walking_height_out) const
{
    if (channel_walking_height != nullptr) {
        walking_height_out = channel_walking_height->norm_input();
    } else {
        walking_height_out = 0.0f;
    }
}

// return heading (in degrees) to target destination (aka waypoint)
float Mode::wp_bearing() const
{
    if (!is_autopilot_mode()) {
        return 0.0f;
    }
    return g2.wp_nav.wp_bearing_cd() * 0.01f;
}

// return short-term target heading in degrees (i.e. target heading back to line between waypoints)
float Mode::nav_bearing() const
{
    if (!is_autopilot_mode()) {
        return 0.0f;
    }
    return g2.wp_nav.nav_bearing_cd() * 0.01f;
}

// return cross track error (i.e. vehicle's distance from the line between waypoints)
float Mode::crosstrack_error() const
{
    if (!is_autopilot_mode()) {
        return 0.0f;
    }
    return g2.wp_nav.crosstrack_error();
}

// return desired lateral acceleration
float Mode::get_desired_lat_accel() const
{
    if (!is_autopilot_mode()) {
        return 0.0f;
    }
    return g2.wp_nav.get_lat_accel();
}

// set desired location
bool Mode::set_desired_location(const Location &destination, Location next_destination )
{
    if (!g2.wp_nav.set_desired_location(destination, next_destination)) {
        return false;
    }

    // initialise distance
    _distance_to_destination = g2.wp_nav.get_distance_to_destination();
    _reached_destination = false;

    return true;
}

// get default speed for this mode (held in WP_SPEED or RTL_SPEED)
float Mode::get_speed_default(bool rtl) const
{
    if (rtl && is_positive(g2.rtl_speed)) {
        return g2.rtl_speed;
    }

    return g2.wp_nav.get_default_speed();
}

// execute the mission in reverse (i.e. backing up)
void Mode::set_reversed(bool value)
{
    g2.wp_nav.set_reversed(value);
}

// handle tacking request (from auxiliary switch) in sailboats
void Mode::handle_tack_request()
{
    // autopilot modes handle tacking
    if (is_autopilot_mode()) {
        g2.sailboat.handle_tack_request_auto();
    }
}

/**
 * @brief Calculate and apply throttle to achieve target speed
 * 
 * @details This is the primary throttle control function called by most autonomous modes.
 *          The calculation pipeline:
 *          1. Apply acceleration limiting to target speed (respects ATC_ACCEL_MAX)
 *          2. Apply object avoidance speed adjustments if enabled
 *          3. Calculate throttle output using speed controller or stop controller
 *          4. Apply vehicle-specific modifications (balance bot pitch, sailboat mainsail)
 *          5. Send final throttle command to motors
 * 
 *          Throttle output is scaled to -100 to +100 range where:
 *          - Positive values: forward throttle
 *          - Negative values: reverse throttle
 *          - Zero: no throttle (vehicle may coast or brake depending on configuration)
 * 
 * @param[in] target_speed Desired speed in m/s (positive forward, negative reverse)
 * @param[in] avoidance_enabled If true, apply object avoidance adjustments to speed
 * 
 * @note Called at main loop rate (typically 50Hz for rover)
 * @warning Throttle limiting depends on motor saturation flags - ensure motors update first
 * 
 * @see calc_steering_from_turn_rate(), stop_vehicle()
 */
void Mode::calc_throttle(float target_speed, bool avoidance_enabled)
{
    // Apply acceleration limiting to target speed changes
    // Prevents sudden speed changes that could cause instability or loss of traction
    // Uses ATC_ACCEL_MAX parameter and main loop delta time (rover.G_Dt)
    target_speed = attitude_control.get_desired_speed_accel_limited(target_speed, rover.G_Dt);

#if AP_AVOIDANCE_ENABLED
    // Apply object avoidance to desired speed if enabled
    if (avoidance_enabled) {
        // Adjust speed to avoid obstacles using half of maximum deceleration
        // This provides gradual slowdown when approaching obstacles detected by proximity sensors
        // Arguments: min_speed (0 = can stop), decel rate, current heading, target speed, dt
        g2.avoid.adjust_speed(0.0f, 0.5f * attitude_control.get_decel_max(), ahrs.get_yaw_rad(), target_speed, rover.G_Dt);
        
        // Special handling for sailboats encountering avoidance limits
        if (g2.sailboat.tack_enabled() && g2.avoid.limits_active()) {
            // Sailboats cannot always slow down or stop due to wind dependence
            // When avoidance active, attempt a tack maneuver to change direction
            if (rover.control_mode != &rover.mode_acro) {
                rover.control_mode->handle_tack_request();
            }
        }
    }
#endif  // AP_AVOIDANCE_ENABLED

    // Calculate throttle controller output in -100 to +100 range
    float throttle_out = 0.0f;

    if (g2.sailboat.sail_enabled()) {
        // Sailboats use specialized controller that coordinates throttle and mainsail angle
        // Mainsail provides primary propulsion, throttle used for auxiliary motor if equipped
        g2.sailboat.get_throttle_and_set_mainsail(target_speed, throttle_out);
    } else {
        // Standard vehicles use speed controller or stop controller
        
        if (is_zero(target_speed) && !rover.is_balancebot()) {
            // Use stop controller when target speed is zero
            // Actively brakes vehicle to a stop rather than just cutting throttle
            // Motor limit flags indicate if motors are saturated (hitting physical limits)
            bool stopped;
            throttle_out = 100.0f * attitude_control.get_throttle_out_stop(
                g2.motors.limit.throttle_lower,  // Lower throttle saturation flag
                g2.motors.limit.throttle_upper,  // Upper throttle saturation flag
                g.speed_cruise,                   // Cruise speed for scaling (m/s)
                g.throttle_cruise * 0.01f,       // Cruise throttle (normalized 0-1)
                rover.G_Dt,                      // Loop delta time
                stopped);                        // Output: true when vehicle stopped
        } else {
            // Use speed controller to track target speed (forward or reverse)
            // Check for motor limiting or pitch limiting (for vehicles that can pitch)
            bool motor_lim_low = g2.motors.limit.throttle_lower || attitude_control.pitch_limited();
            bool motor_lim_high = g2.motors.limit.throttle_upper || attitude_control.pitch_limited();
            
            // Speed controller uses PID to minimize speed error
            // Limit flags used for integrator anti-windup when motors saturated
            throttle_out = 100.0f * attitude_control.get_throttle_out_speed(
                target_speed,       // Desired speed (m/s)
                motor_lim_low,      // Lower limit active flag
                motor_lim_high,     // Upper limit active flag
                g.speed_cruise,     // Cruise speed for scaling
                g.throttle_cruise * 0.01f,  // Cruise throttle
                rover.G_Dt);        // Loop delta time
        }

        // Balance bots require special pitch stabilization
        // Throttle command modified to maintain balance (inverted pendulum control)
        if (rover.is_balancebot()) {
            rover.balancebot_pitch_control(throttle_out);
        }
    }

    // Send final throttle command to motor library
    // Motor library applies final scaling, mixing (for skid steering), and outputs to servos/ESCs
    g2.motors.set_throttle(throttle_out);
}

/**
 * @brief Perform a controlled stop without turning
 * 
 * @details Brings vehicle to a complete stop using controlled deceleration while
 *          maintaining straight heading. This is safer than simply cutting throttle
 *          as it applies braking and prevents coast-down drift.
 * 
 *          Stop sequence:
 *          1. Use stop controller to generate braking throttle
 *          2. Relax sailboat sails if present
 *          3. Apply zero turn rate to maintain straight heading during stop
 *          4. Once stopped, hold position with zero throttle and zero steering
 * 
 *          Special handling:
 *          - Balance bots: Use speed controller to maintain balance while stopping
 *          - Sailboats: Relax sails to reduce propulsion
 *          - Normal vehicles: Active braking with stop controller
 * 
 * @return true when vehicle is completely stopped (speed below stop threshold),
 *         false while still slowing down
 * 
 * @note Called by Hold mode and as part of mode transitions requiring stop
 * @warning Stopping distance depends on surface friction and vehicle mass/inertia
 * 
 * @see calc_throttle()
 */
bool Mode::stop_vehicle()
{
    // Calculate throttle for controlled stop
    bool stopped = false;
    float throttle_out;

    // Balance bots require continuous pitch control to maintain balance
    // Cannot use stop controller as it would tip over - use speed controller with zero target
    if (rover.is_balancebot()) {
        throttle_out = 100.0f * attitude_control.get_throttle_out_speed(
            0,                                  // Target speed: zero (stopping)
            g2.motors.limit.throttle_lower,     // Lower throttle saturation flag
            g2.motors.limit.throttle_upper,     // Upper throttle saturation flag  
            g.speed_cruise,                     // Cruise speed for scaling
            g.throttle_cruise * 0.01f,         // Cruise throttle (normalized)
            rover.G_Dt);                        // Loop delta time
        
        // Apply pitch stabilization to prevent tipping
        rover.balancebot_pitch_control(throttle_out);
    } else {
        // Use stop controller for normal vehicles
        // Actively brakes vehicle to zero speed with controlled deceleration
        throttle_out = 100.0f * attitude_control.get_throttle_out_stop(
            g2.motors.limit.throttle_lower,     // Lower throttle saturation flag
            g2.motors.limit.throttle_upper,     // Upper throttle saturation flag
            g.speed_cruise,                     // Cruise speed for scaling
            g.throttle_cruise * 0.01f,         // Cruise throttle (normalized)
            rover.G_Dt,                         // Loop delta time
            stopped);                           // Output: true when stopped
    }

    // Relax sailboat sails to reduce wind propulsion during stop
    g2.sailboat.relax_sails();

    // Send throttle command to motors
    g2.motors.set_throttle(throttle_out);

    // Maintain straight heading while slowing down
    // Zero turn rate keeps vehicle pointed in current direction
    // Once stopped, also zero steering to prevent drift
    float steering_out = 0.0;
    if (!stopped) {
        // While still moving, use rate controller to resist turning
        steering_out = attitude_control.get_steering_out_rate(
            0.0,                            // Desired turn rate: zero (straight)
            g2.motors.limit.steer_left,     // Left steering saturation flag
            g2.motors.limit.steer_right,    // Right steering saturation flag
            rover.G_Dt);                    // Loop delta time
    }
    // steering_out is zero when stopped, or small correction while stopping
    g2.motors.set_steering(steering_out * 4500.0);

    // Return true once vehicle speed drops below stop threshold
    return stopped;
}

/**
 * @brief Estimate maximum vehicle speed from cruise parameters
 * 
 * @details Calculates the vehicle's maximum achievable speed using either:
 *          1. Explicit SPEED_MAX parameter if configured
 *          2. Projection from cruise speed and cruise throttle relationship
 * 
 *          The projection assumes linear throttle-to-speed relationship:
 *          If cruise_throttle achieves cruise_speed, then 100% throttle achieves:
 *          speed_max = cruise_speed / cruise_throttle
 * 
 *          Example: If 50% throttle (0.5) achieves 5 m/s cruise speed:
 *          speed_max = 5 / 0.5 = 10 m/s
 * 
 *          This is used by pilot input scaling to convert throttle stick position
 *          to desired speed commands.
 * 
 * @param[in] cruise_speed    Cruise speed in m/s (typically SPEED_CRUISE parameter)
 * @param[in] cruise_throttle Cruise throttle normalized 0-1 (typically THROTTLE_CRUISE / 100)
 * 
 * @return Maximum vehicle speed in m/s, constrained to 0-30 m/s (108 km/h)
 * 
 * @note Maximum capped at 30 m/s for safety and realistic rover speeds
 * @warning Inaccurate if throttle-speed relationship is non-linear (e.g., drag-limited)
 * 
 * @see get_pilot_desired_steering_and_speed()
 */
float Mode::calc_speed_max(float cruise_speed, float cruise_throttle) const
{
    float speed_max;

    // Sanity check cruise throttle parameters
    // If invalid, fall back to cruise speed as maximum
    if (cruise_throttle > 1.0f || cruise_throttle < 0.05f) {
        speed_max = cruise_speed;
    } else if (is_positive(g2.speed_max)) {
        // Use explicit maximum speed parameter if configured
        // SPEED_MAX parameter allows direct specification without calculation
        speed_max = g2.speed_max;
    } else {
        // Project vehicle's maximum speed from cruise point
        // Assumes linear relationship: speed = k * throttle
        // Solving for k: k = cruise_speed / cruise_throttle
        // Maximum speed at full throttle: speed_max = k * 1.0
        speed_max = (1.0f / cruise_throttle) * cruise_speed;
    }

    // Constrain to reasonable maximum (30 m/s = 108 km/h)
    // Prevents unrealistic speed commands from misconfigured parameters
    return constrain_float(speed_max, 0.0f, 30.0f);
}

// calculate pilot input to nudge speed up or down
//  target_speed should be in meters/sec
//  reversed should be true if the vehicle is intentionally backing up which allows the pilot to increase the backing up speed by pulling the throttle stick down
float Mode::calc_speed_nudge(float target_speed, bool reversed)
{
    // sanity checks
    if (g.throttle_cruise > 100 || g.throttle_cruise < 5) {
        return target_speed;
    }

    // convert pilot throttle input to speed
    float pilot_steering, pilot_throttle;
    get_pilot_input(pilot_steering, pilot_throttle);
    float pilot_speed = pilot_throttle * 0.01f * calc_speed_max(g.speed_cruise, g.throttle_cruise * 0.01f);

    // ignore pilot's input if in opposite direction to vehicle's desired direction of travel
    // note that the target_speed may be negative while reversed is true (or vice-versa)
    // while vehicle is transitioning between forward and backwards movement
    if ((is_positive(pilot_speed) && reversed) ||
        (is_negative(pilot_speed) && !reversed)) {
        return target_speed;
    }

    // return the larger of the pilot speed and the original target speed
    if (reversed) {
        return MIN(target_speed, pilot_speed);
    } else {
        return MAX(target_speed, pilot_speed);
    }
}

/**
 * @brief High level navigation function to drive towards waypoint
 * 
 * @details This is the primary navigation function used by autonomous modes (Auto, Guided, RTL).
 *          It coordinates the waypoint navigation controller with throttle and steering controllers
 *          to follow the path from origin to destination waypoint.
 * 
 *          Navigation pipeline:
 *          1. Apply pilot speed nudge (allows pilot to override speed while maintaining path)
 *          2. Update waypoint navigation controller (calculates turn rate and speed commands)
 *          3. Handle avoidance (sailboat tacking if needed)
 *          4. Calculate throttle from desired speed
 *          5. Calculate steering from turn rate or heading (different for sailboats vs normal vehicles)
 * 
 *          Coordinate frames used:
 *          - Waypoint positions: Earth frame (NED - North-East-Down), absolute or relative to origin
 *          - Heading commands: Earth frame (0° = North, 90° = East)
 *          - Turn rate commands: Body frame (rad/s, positive = right turn)
 * 
 * @note This function updates _distance_to_destination member variable
 * @note Called at main loop rate during autonomous navigation
 * @warning Object avoidance handled in wp_nav, do not enable in calc_throttle to avoid double-application
 * 
 * @see calc_throttle(), calc_steering_from_turn_rate(), set_desired_location()
 */
void Mode::navigate_to_waypoint()
{
    // Allow pilot to nudge speed up or down while maintaining path
    // Speed nudge input depends on whether vehicle is driving forward or backing up
    // calc_speed_nudge expects negative speed when reversing, but nudge_speed_max is always positive
    const float calc_nudge_input_speed = g2.wp_nav.get_speed_max() * (g2.wp_nav.get_reversed() ? -1.0 : 1.0);
    const float nudge_speed_max = calc_speed_nudge(calc_nudge_input_speed, g2.wp_nav.get_reversed());
    g2.wp_nav.set_nudge_speed_max(fabsf(nudge_speed_max));

    // Update waypoint navigation controller
    // Calculates desired speed, turn rate, and cross-track corrections
    // Uses L1 controller for path following with lookahead based on speed
    g2.wp_nav.update(rover.G_Dt);
    
    // Update distance to destination for mode-specific logic (e.g., waypoint reached detection)
    _distance_to_destination = g2.wp_nav.get_distance_to_destination();

#if AP_AVOIDANCE_ENABLED
    // Special avoidance handling for sailboats
    // Sailboats cannot always slow down or stop, so use tacking maneuver for avoidance
    if (g2.sailboat.tack_enabled() && g2.avoid.limits_active()) {
        // Fence or proximity sensor triggered - attempt tack to change direction
        rover.control_mode->handle_tack_request();
    }
#endif

    // Calculate throttle from waypoint controller's desired speed
    // Object avoidance disabled here because wp_nav already handles it (avoid double-application)
    calc_throttle(g2.wp_nav.get_speed(), false);

    // Calculate steering - method depends on vehicle type and sailing conditions
    // Coordinate transformation: wp_nav provides earth frame bearing, converted to steering commands
    float desired_heading_cd = g2.wp_nav.oa_wp_bearing_cd();  // Object avoidance adjusted bearing
    
    if (g2.sailboat.use_indirect_route(desired_heading_cd)) {
        // Sailboats cannot sail directly upwind - use indirect heading (tacking)
        // Heading controller used instead of turn rate controller for precise heading hold
        desired_heading_cd = g2.sailboat.calc_heading(desired_heading_cd);
        
        // Use pivot turn rate when actively tacking, otherwise normal turn rate
        const float turn_rate = g2.sailboat.tacking() ? g2.wp_nav.get_pivot_rate() : 0.0f;
        calc_steering_to_heading(desired_heading_cd, turn_rate);
    } else {
        // Normal vehicles and sailboats on direct routes use turn rate controller
        // Turn rate in body frame (rad/s) from waypoint path following controller
        float desired_turn_rate_rads = g2.wp_nav.get_turn_rate_rads();

#if AP_AVOIDANCE_ENABLED
        // Special case: if stopped or nearly stopped due to avoidance, don't turn
        // Prevents vehicle from spinning in place when blocked by obstacle
        if (g2.avoid.limits_active() && (fabsf(attitude_control.get_desired_speed()) <= attitude_control.get_stop_speed())) {
            desired_turn_rate_rads = 0.0f;
        }
#endif

        // Apply turn rate to steering controller
        calc_steering_from_turn_rate(desired_turn_rate_rads);
    }
}

/**
 * @brief Calculate steering output to achieve desired turn rate
 * 
 * @details Converts a desired turn rate to steering servo output using the turn rate
 *          controller. The controller uses vehicle kinematics and a PID loop to
 *          minimize turn rate error. This is the primary steering method used during
 *          waypoint navigation where the path controller provides turn rate commands.
 * 
 *          Steering output is scaled to -4500 to +4500 range (centidegrees) where:
 *          - Positive: turn right
 *          - Negative: turn left
 *          - Zero: straight ahead
 * 
 * @param[in] turn_rate Desired turn rate in radians/sec (positive = right turn,
 *                      negative = left turn, body frame angular velocity)
 * 
 * @note Called at main loop rate during waypoint navigation
 * @warning Turn rate commands exceeding vehicle's physical capabilities will be limited
 * 
 * @see calc_steering_to_heading(), calc_steering_from_lateral_acceleration()
 */
void Mode::calc_steering_from_turn_rate(float turn_rate)
{
    // Use turn rate controller to convert desired angular velocity to steering output
    // Controller accounts for vehicle speed and steering geometry
    // Motor limit flags used for integrator anti-windup when steering saturated
    const float steering_out = attitude_control.get_steering_out_rate(
        turn_rate,                      // Desired turn rate (rad/s, body frame)
        g2.motors.limit.steer_left,     // Left steering saturation flag
        g2.motors.limit.steer_right,    // Right steering saturation flag
        rover.G_Dt);                    // Loop delta time
    
    // Scale normalized output (-1 to +1) to centidegrees (-4500 to +4500)
    // and send to motor library via set_steering (applies stick mixing if enabled)
    set_steering(steering_out * 4500.0f);
}

/**
 * @brief Calculate steering output to achieve desired lateral acceleration
 * 
 * @details Converts desired lateral acceleration to steering servo output. This method
 *          is used for direct lateral acceleration control, typically during advanced
 *          path following where the path controller specifies centripetal acceleration.
 * 
 *          Lateral acceleration is constrained to vehicle's maximum cornering capability
 *          (ATC_TURN_MAX_G parameter) to prevent loss of traction or rollover.
 * 
 *          Coordinate frame: Lateral acceleration is in vehicle body frame where:
 *          - Positive: acceleration to the right (centripetal for right turn)
 *          - Negative: acceleration to the left (centripetal for left turn)
 * 
 * @param[in] lat_accel Desired lateral acceleration in m/s² (body frame, positive right)
 * @param[in] reversed  True if vehicle is reversing (currently unused but reserved for
 *                      future kinematic adjustments when backing up)
 * 
 * @note Lateral acceleration is limited to ATC_TURN_MAX_G (default 0.6 G = 5.88 m/s²)
 * @warning Excessive lateral acceleration requests on low-traction surfaces may cause skidding
 * 
 * @see calc_steering_from_turn_rate(), calc_steering_to_heading()
 */
void Mode::calc_steering_from_lateral_acceleration(float lat_accel, bool reversed)
{
    // Constrain lateral acceleration to vehicle's maximum cornering capability
    // Prevents commands that would exceed traction limits or cause rollover
    // Maximum typically set to 0.6 G for stability on most surfaces
    lat_accel = constrain_float(lat_accel, 
                                -attitude_control.get_turn_lat_accel_max(), 
                                 attitude_control.get_turn_lat_accel_max());

    // Calculate steering output from desired lateral acceleration
    // Uses vehicle kinematics (wheelbase, track width) to convert lateral accel to steering angle
    const float steering_out = attitude_control.get_steering_out_lat_accel(
        lat_accel,                      // Desired lateral accel (m/s², body frame)
        g2.motors.limit.steer_left,     // Left steering saturation flag
        g2.motors.limit.steer_right,    // Right steering saturation flag
        rover.G_Dt);                    // Loop delta time
    
    // Scale normalized output (-1 to +1) to centidegrees (-4500 to +4500)
    set_steering(steering_out * 4500.0f);
}

/**
 * @brief Calculate steering output to drive towards desired heading
 * 
 * @details Converts desired heading to steering output using the heading controller.
 *          The controller calculates heading error and applies rate limiting to achieve
 *          smooth turns. This method is used when the desired vehicle heading is known
 *          (e.g., heading hold mode, sailboat tacking, obstacle avoidance heading override).
 * 
 *          The heading controller wraps angles correctly across the 0/360° boundary and
 *          chooses the shortest rotation direction.
 * 
 *          Coordinate frame: Heading is in earth frame (NED - North-East-Down) where:
 *          - 0°/360°: North
 *          - 90°: East  
 *          - 180°: South
 *          - 270°: West
 * 
 * @param[in] desired_heading_cd Desired heading in centidegrees (0-36000, earth frame NED)
 * @param[in] rate_max_degs      Maximum turn rate limit in deg/s. Set to zero to use
 *                               default turn rate limit from ATC_TURN_MAX_G and current speed
 * 
 * @note Heading is in earth frame (NED), converted internally to radians for controller
 * @warning High rate limits on slippery surfaces may cause loss of heading control
 * 
 * @see calc_steering_from_turn_rate(), navigate_to_waypoint()
 */
void Mode::calc_steering_to_heading(float desired_heading_cd, float rate_max_degs)
{
    // Call heading controller with angle and rate inputs
    // Controller calculates heading error, applies rate limiting, and generates steering output
    // Angles converted from centidegrees to radians for internal calculations
    const float steering_out = attitude_control.get_steering_out_heading(
        radians(desired_heading_cd * 0.01f),  // Desired heading (radians, earth frame NED)
        radians(rate_max_degs),                // Max turn rate (rad/s, 0 = use default)
        g2.motors.limit.steer_left,            // Left steering saturation flag
        g2.motors.limit.steer_right,           // Right steering saturation flag
        rover.G_Dt);                           // Loop delta time
    
    // Scale normalized output (-1 to +1) to centidegrees (-4500 to +4500)
    set_steering(steering_out * 4500.0f);
}

/**
 * @brief Set steering output with optional pilot stick mixing
 * 
 * @details Sends steering command to motor library with optional pilot override mixing.
 *          Stick mixing allows the pilot to nudge steering in autonomous modes while
 *          maintaining autonomous path control.
 * 
 *          Steering servo output calculation in motor library:
 *          1. Scale input (-4500 to +4500) to servo PWM range (typically 1000-2000 µs)
 *          2. Apply trim and reverse settings from SERVOx parameters
 *          3. Apply expo curves if configured
 *          4. For skid steering, mix with throttle to generate left/right motor commands
 * 
 * @param[in] steering_value Steering command in centidegrees (-4500 to +4500)
 *                          Positive: turn right, Negative: turn left
 * 
 * @note Stick mixing only active if mode allows it (not in Manual/Acro) and STICK_MIXING > 0
 * @warning Excessive stick mixing can interfere with autonomous path following
 * 
 * @see calc_steering_from_turn_rate(), calc_steering_to_heading()
 */
void Mode::set_steering(float steering_value)
{
    // Apply pilot stick mixing if enabled and mode allows it
    // Stick mixing blends autonomous steering command with pilot RC input
    // Allows pilot to override or nudge autonomous steering for obstacle avoidance
    if (allows_stick_mixing() && g2.stick_mixing > 0) {
        steering_value = channel_steer->stick_mixing((int16_t)steering_value);
    }
    
    // Send final steering command to motor library
    // Motor library converts to servo PWM output and applies mixing for skid-steering
    g2.motors.set_steering(steering_value);
}

/**
 * @brief Convert mode number to mode object pointer
 * 
 * @details This function maps mode number enumerations to their corresponding
 *          mode object instances. Used during mode switching to obtain the
 *          target mode object before calling its enter() method.
 * 
 *          Mode validation occurs in the mode switching logic before this
 *          function is called. This function simply provides the mapping.
 * 
 *          Some modes are conditionally compiled based on feature flags:
 *          - FOLLOW: Requires MODE_FOLLOW_ENABLED
 *          - DOCK: Requires MODE_DOCK_ENABLED
 * 
 * @param[in] num Mode number enumeration (Mode::Number enum)
 * 
 * @return Pointer to Mode object for the requested mode, or nullptr if invalid mode number
 * 
 * @note Called during mode switches from RC input, GCS commands, or mission items
 * @warning Returns nullptr for disabled modes or invalid mode numbers - caller must check
 * 
 * @see Mode::enter(), Rover::set_mode()
 */
Mode *Rover::mode_from_mode_num(const enum Mode::Number num)
{
    Mode *ret = nullptr;
    
    // Map mode number to mode object instance
    // Each mode is a singleton object instantiated in Rover class
    switch (num) {
    case Mode::Number::MANUAL:
        ret = &mode_manual;
        break;
    case Mode::Number::ACRO:
        ret = &mode_acro;
        break;
    case Mode::Number::STEERING:
        ret = &mode_steering;
        break;
    case Mode::Number::HOLD:
        ret = &mode_hold;
        break;
    case Mode::Number::LOITER:
        ret = &mode_loiter;
        break;
#if MODE_FOLLOW_ENABLED
    case Mode::Number::FOLLOW:
        ret = &mode_follow;
        break;
#endif
    case Mode::Number::SIMPLE:
        ret = &mode_simple;
        break;
    case Mode::Number::CIRCLE:
        ret = &g2.mode_circle;
        break;
    case Mode::Number::AUTO:
        ret = &mode_auto;
        break;
    case Mode::Number::RTL:
        ret = &mode_rtl;
        break;
    case Mode::Number::SMART_RTL:
        ret = &mode_smartrtl;
        break;
    case Mode::Number::GUIDED:
        ret = &mode_guided;
        break;
    case Mode::Number::INITIALISING:
        ret = &mode_initializing;
        break;
#if MODE_DOCK_ENABLED
    case Mode::Number::DOCK:
        ret = (Mode *)g2.mode_dock_ptr;
        break;
#endif
    default:
        // Invalid mode number - return nullptr
        break;
    }
    return ret;
}
