/**
 * @file sailboat.cpp
 * @brief Implementation of sailboat-specific control algorithms for Rover
 * 
 * @details This file implements sailboat control functionality including:
 * - Sail trimming based on apparent wind angle (mainsail, wingsail, rotating mast)
 * - Tacking state machine for autonomous navigation through no-go zones
 * - Motor assist logic for low wind conditions and tacking assistance
 * - Velocity Made Good (VMG) calculation for performance monitoring
 * - Indirect route navigation when sailing upwind
 * 
 * Sailboats cannot sail directly into the wind (no-go zone), requiring special navigation
 * logic to tack back and forth when the desired heading is upwind. This file handles the
 * transition between direct navigation (when possible) and indirect tacking navigation.
 * 
 * @note Coordinate frames used:
 *       - Apparent wind: Wind direction relative to moving vehicle (body frame)
 *       - True wind: Wind direction relative to earth frame
 *       - Vehicle heading: Body frame forward direction in earth frame
 *       - All angles in radians unless specified otherwise (some in degrees or centidegrees)
 * 
 * @warning Safety-critical for sailboat operation - incorrect sail control or tacking
 *          logic can result in loss of control, especially in high wind conditions
 */

#include "Rover.h"

#define SAILBOAT_AUTO_TACKING_TIMEOUT_MS 5000   // tacks in auto mode timeout if not successfully completed within this many milliseconds
#define SAILBOAT_TACKING_ACCURACY_DEG 10        // tack is considered complete when vehicle is within this many degrees of target tack angle
#define SAILBOAT_NOGO_PAD 10                    // deg, the no go zone is padded by this much when deciding if we should use the Sailboat heading controller
#define TACK_RETRY_TIME_MS 5000                 // Can only try another auto mode tack this many milliseconds after the last is cleared (either competed or timed-out)
/*
To Do List
 - Improve tacking in light winds and bearing away in strong wings
 - consider drag vs lift sailing differences, ie upwind sail is like wing, dead down wind sail is like parachute
 - max speed parameter and controller, for mapping you may not want to go too fast
 - mavlink sailing messages
 - smart decision making, ie tack on windshifts, what to do if stuck head to wind
 - some sailing codes track waves to try and 'surf' and to allow tacking on a flat bit, not sure if there is much gain to be had here
 - add some sort of pitch monitoring to prevent nose diving in heavy weather
 - pitch PID for hydrofoils
 - more advanced sail control, ie twist
 - independent sheeting for main and jib
 - tack on depth sounder info to stop sailing into shallow water on indirect sailing routes
 - add option to do proper tacks, ie tacking on flat spot in the waves, or only try once at a certain speed, or some better method than just changing the desired heading suddenly
*/

const AP_Param::GroupInfo Sailboat::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Enable Sailboat
    // @Description: This enables Sailboat functionality
    // @Values: 0:Disable,1:Enable
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("ENABLE", 1, Sailboat, enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: ANGLE_MIN
    // @DisplayName: Sail min angle
    // @Description: Mainsheet tight, angle between centerline and boom
    // @Units: deg
    // @Range: 0 90
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ANGLE_MIN", 2, Sailboat, sail_angle_min, 0),

    // @Param: ANGLE_MAX
    // @DisplayName: Sail max angle
    // @Description: Mainsheet loose, angle between centerline and boom. For direct-control rotating masts, the rotation angle at SERVOx_MAX/_MIN; for rotating masts, this value can exceed 90 degrees if the linkages can physically rotate the mast past that angle.
    // @Units: deg
    // @Range: 0 90
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ANGLE_MAX", 3, Sailboat, sail_angle_max, 90),

    // @Param: ANGLE_IDEAL
    // @DisplayName: Sail ideal angle
    // @Description: Ideal angle between sail and apparent wind
    // @Units: deg
    // @Range: 0 90
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ANGLE_IDEAL", 4, Sailboat, sail_angle_ideal, 25),

    // @Param: HEEL_MAX
    // @DisplayName: Sailing maximum heel angle
    // @Description: When in auto sail trim modes the heel will be limited to this value using PID control
    // @Units: deg
    // @Range: 0 90
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("HEEL_MAX", 5, Sailboat, sail_heel_angle_max, 15),

    // @Param: NO_GO_ANGLE
    // @DisplayName: Sailing no go zone angle
    // @Description: The typical closest angle to the wind the vehicle will sail at. the vehicle will sail at this angle when going upwind
    // @Units: deg
    // @Range: 0 90
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("NO_GO_ANGLE", 6, Sailboat, sail_no_go, 45),

    // @Param: WNDSPD_MIN
    // @DisplayName: Sailboat minimum wind speed to sail in
    // @Description: Sailboat minimum wind speed to continue sail in, at lower wind speeds the sailboat will motor if one is fitted
    // @Units: m/s
    // @Range: 0 5
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("WNDSPD_MIN", 7, Sailboat, sail_windspeed_min, 0),

    // @Param: XTRACK_MAX
    // @DisplayName: Sailing vehicle max cross track error
    // @Description: The sail boat will tack when it reaches this cross track error, defines a corridor of 2 times this value wide, 0 disables
    // @Units: m
    // @Range: 5 25
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("XTRACK_MAX", 8, Sailboat, xtrack_max, 10),

    // @Param: LOIT_RADIUS
    // @DisplayName: Loiter radius
    // @Description: When in sailing modes the vehicle will keep moving within this loiter radius
    // @Units: m
    // @Range: 0 20
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("LOIT_RADIUS", 9, Sailboat, loit_radius, 5),

    AP_GROUPEND
};


/*
  constructor
 */
Sailboat::Sailboat()
{
    AP_Param::setup_object_defaults(this, var_info);
}

// true if sailboat navigation (aka tacking) is enabled
bool Sailboat::tack_enabled() const
{
    // tacking disabled if not a sailboat
    if (!sail_enabled()) {
        return false;
    }

    // tacking disabled if motor is always on
    if (motor_state == UseMotor::USE_MOTOR_ALWAYS) {
        return false;
    }

    // disable tacking if motor is available and wind is below cutoff
    if (motor_assist_low_wind()) {
        return false;
    }

    // otherwise tacking is enabled
    return true;
}

void Sailboat::init()
{
    // sailboat defaults
    if (sail_enabled()) {
        rover.g2.crash_angle.set_default(0);

        // sailboats without motors may travel faster than WP_SPEED so allow waypoint navigation to
        // speedup to catch the vehicle instead of asking the vehicle to slow down
        rover.g2.wp_nav.enable_overspeed(motor_state != UseMotor::USE_MOTOR_ALWAYS);
    }

    if (tack_enabled()) {
        rover.g2.loit_type.set_default(1);
    }

    // initialise motor state to USE_MOTOR_ASSIST
    // this will silently fail if there is no motor attached
    set_motor_state(UseMotor::USE_MOTOR_ASSIST, false);
}

// initialise rc input (channel_mainsail), may be called intermittently
void Sailboat::init_rc_in()
{
    // get auxiliary throttle value
    RC_Channel *rc_ptr = rc().find_channel_for_option(RC_Channel::AUX_FUNC::MAINSAIL);
    if (rc_ptr != nullptr) {
        // use aux as sail input if defined
        channel_mainsail = rc_ptr;
        channel_mainsail->set_angle(100);
        channel_mainsail->set_default_dead_zone(30);
    } else {
        // use throttle channel
        channel_mainsail = rover.channel_throttle;
    }
}

/// @brief decode pilot mainsail input in manual modes and update the various
/// sail actuator values for different sail types ready for SRV_Channel output.
void Sailboat::set_pilot_desired_mainsail()
{
    // no RC input means mainsail is moved to trim
    if ((rover.failsafe.bits & FAILSAFE_EVENT_THROTTLE) || (channel_mainsail == nullptr)) {
       relax_sails();
    } else {
       rover.g2.motors.set_mainsail(constrain_float(channel_mainsail->get_control_in(), 0.0f, 100.0f));
       rover.g2.motors.set_wingsail(constrain_float(channel_mainsail->get_control_in(), -100.0f, 100.0f));
       rover.g2.motors.set_mast_rotation(constrain_float(channel_mainsail->get_control_in(), -100.0f, 100.0f));
    }
}

/// @brief Set mainsail in auto modes
/// @param[in] desired_speed desired speed (in m/s) only used to detect desired direction
void Sailboat::set_auto_mainsail(float desired_speed)
{
    // Use PID controller to sheet out sail to prevent excessive heeling
    // get_sail_out_from_heel returns normalized value; multiply by 100 for 0-100 range
    // Larger heel angles result in larger pid_offset, sheeting sails out to reduce heeling moment
    const float pid_offset = rover.g2.attitude_control.get_sail_out_from_heel(radians(sail_heel_angle_max), rover.G_Dt) * 100.0f;

    // Get apparent wind direction in vehicle body frame
    // Apparent wind = true wind + wind created by vehicle motion (vector addition)
    // Convention: + is wind over starboard (right) side, - is wind over port (left) side
    // Range: -180 to +180 degrees, 0 degrees is directly ahead
    const float wind_dir_apparent = degrees(rover.g2.windvane.get_apparent_wind_direction_rad());
    const float wind_dir_apparent_abs = fabsf(wind_dir_apparent);
    const float wind_dir_apparent_sign = is_negative(wind_dir_apparent) ? -1.0f : 1.0f;

    //
    // Mainsail Control Logic
    //
    // Mainsail trimming algorithm: Set sail angle based on apparent wind to maximize lift
    // mainsail_out represents servo position from 0 (tight/close-hauled) to 100 (loose/running)
    float mainsail_out = 100.0f;
    
    // Main sails cannot be used to reverse - only generate forward thrust
    if (is_positive(desired_speed)) {
        // Symmetric sail control: Sails are sheeted the same angle on each tack
        // Use absolute value of apparent wind angle (port and starboard handled identically)
        
        // Calculate desired mainsail boom angle relative to centerline
        // Sail should be at ideal angle of attack relative to apparent wind for maximum lift
        // Formula: mainsail_angle = |apparent_wind_angle| - ideal_angle_of_attack
        // Constrained between min (tight) and max (loose) sail angles
        const float mainsail_angle =
            constrain_float(wind_dir_apparent_abs - sail_angle_ideal,sail_angle_min, sail_angle_max);

        // Convert desired boom angle to servo position (0-100)
        // Linear interpolation maps physical angle range to servo output range
        // sail_angle_min (e.g., 0°) → 0% (mainsheet tight)
        // sail_angle_max (e.g., 90°) → 100% (mainsheet loose)
        const float mainsail_base = linear_interpolate(0.0f, 100.0f, mainsail_angle,sail_angle_min,sail_angle_max);

        // Apply heel control offset to reduce sail power if heeling excessively
        // pid_offset increases when heel angle exceeds sail_heel_angle_max
        // Adding pid_offset sheets sail out (depowers) to reduce heeling moment
        mainsail_out = constrain_float((mainsail_base + pid_offset), 0.0f ,100.0f);
    }
    rover.g2.motors.set_mainsail(mainsail_out);
    //
    // Wingsail Control Logic
    //
    // Wingsails are rigid airfoils that auto-trim to the wind (weathervane effect)
    // Control objective: Set correct tack (port/starboard) and reduce power if heeling excessively
    // Output range: -100 (full port) to +100 (full starboard)
    
    // Wingsails can theoretically operate in reverse (though not recommended)
    // Reverse sign of output for reverse motion
    const float wing_sail_out_sign = is_negative(desired_speed) ? -1.0f : 1.0f;
    
    // Calculate wingsail servo position:
    // 1. Start with 100% power (full trim)
    // 2. Reduce power based on pid_offset (heel control) - sheets out to depower
    //    - Never reduce below 0% (don't backwind the sail to force self-righting)
    // 3. Apply sign based on apparent wind direction (determines tack)
    //    - Positive apparent wind (starboard) → positive output (wingsail to starboard)
    //    - Negative apparent wind (port) → negative output (wingsail to port)
    // 4. Apply direction sign for forward/reverse motion
    const float wingsail_out = (100.0f - MIN(pid_offset,100.0f)) * wind_dir_apparent_sign * wing_sail_out_sign;
    rover.g2.motors.set_wingsail(wingsail_out);
    //
    // Direct Mast Rotation Control Logic
    //
    // For sailboats with rotating masts (mast pivots at base, sail is rigid with mast)
    // Control objective: Rotate mast to optimal angle based on apparent wind and sailing mode
    // Output range: -100 (full port rotation) to +100 (full starboard rotation)
    //
    float mast_rotation_out = 0.0f;
    if (is_positive(desired_speed)) {
        // Rotating mast sails could be used in reverse, but not implemented in this version
        
        if (wind_dir_apparent_abs < sail_angle_ideal) {
            // In irons: Apparent wind angle is less than ideal angle of attack
            // Wind is too close to bow for effective sailing - center the sail to reduce drag
            mast_rotation_out = 0.0f;
        } else {
            float mast_rotation_angle;
            
            // Determine sailing mode based on apparent wind angle
            if (wind_dir_apparent_abs < (90.0f + sail_angle_ideal)) {
                //
                // LIFT MODE: Use sail as airfoil for upwind/reaching sailing
                // Wind is forward of beam - sail generates lift like an aircraft wing
                //
                
                // Set mast to ideal angle of attack relative to apparent wind
                // Base angle: apparent_wind - ideal_angle (positions sail optimally for lift)
                // Depower factor: Reduce angle when heeling excessively (pid_offset)
                //   - pid_offset scaled by 0.01 to match other sail type scaling (0-100 range)
                //   - Depowering reduces angle, feathering sail more into wind
                //   - MAX ensures we don't go negative (minimum angle is 0)
                mast_rotation_angle = wind_dir_apparent_abs - sail_angle_ideal * MAX(1.0f - pid_offset*0.01f,0.0f);

                // Restore sign to indicate tack (port = negative, starboard = positive)
                mast_rotation_angle *= wind_dir_apparent_sign;

            } else {
                //
                // DRAG MODE: Use sail as parachute for downwind sailing  
                // Wind is aft of beam - sail generates drag rather than lift
                //
                
                // Set mast perpendicular to apparent wind (90 degrees) for maximum drag
                mast_rotation_angle = 90.0f;
                
                if (wind_dir_apparent_abs > 135.0f) {
                    // Wind is almost directly behind (dead downwind run)
                    // Avoid oscillating sail as wind direction varies around 180/-180 boundary
                    // Solution: Keep mast on current tack to prevent violent sail flapping
                    if (is_negative(SRV_Channels::get_output_scaled(SRV_Channel::k_mast_rotation))) {
                        mast_rotation_angle *= -1.0f;
                    }
                } else {
                    // Wind is at broad reach angle (90-135 degrees)
                    // Set mast on correct tack based on apparent wind direction
                    // This allows sheeting in if wind shifts forward
                    mast_rotation_angle *= wind_dir_apparent_sign;
                }
            }
            
            // Convert desired mast rotation angle (degrees) to servo output (-100 to 100)
            // Linear interpolation maps physical rotation range to servo command range
            // -sail_angle_max → -100 (full port rotation)
            // +sail_angle_max → +100 (full starboard rotation)
            mast_rotation_out = linear_interpolate(-100.0f, 100.0f, mast_rotation_angle, -sail_angle_max, sail_angle_max);
        }
    }
    rover.g2.motors.set_mast_rotation(mast_rotation_out);
}

void Sailboat::relax_sails()
{
    rover.g2.motors.set_mainsail(100.0f);
    rover.g2.motors.set_wingsail(0.0f);
    rover.g2.motors.set_mast_rotation(0.0f);
}

/**
 * @brief Calculate motor throttle and set sail trim for desired speed
 * 
 * @details Coordinates motor and sail control based on current motor state:
 * 
 * Motor Control Conditions:
 * - USE_MOTOR_ALWAYS: Motor runs continuously at speed to achieve desired_speed
 * - USE_MOTOR_ASSIST: Motor runs only when motor_assist_tack() or motor_assist_low_wind() true
 * - USE_MOTOR_NEVER: Motor never runs (throttle_out remains 0)
 * 
 * Sail Control Conditions:  
 * - USE_MOTOR_ALWAYS: Sails relaxed (no sail trim), pure motor operation
 * - USE_MOTOR_ASSIST/NEVER: Sails actively trimmed based on apparent wind
 * 
 * @param[in] desired_speed Target speed in m/s (sign indicates direction)
 * @param[out] throttle_out Motor throttle output in percent (0-100)
 * 
 * @note Called by autonomous modes to coordinate propulsion
 * @note Speed controller uses PID to minimize speed error
 */
void Sailboat::get_throttle_and_set_mainsail(float desired_speed, float &throttle_out)
{
    throttle_out = 0.0f;
    if (!sail_enabled()) {
        relax_sails();
        return;
    }

    //
    // MOTOR CONTROL: Activate motor based on current conditions
    //
    // Run speed controller to calculate throttle if:
    // 1. Motor forced on (USE_MOTOR_ALWAYS mode - powerboat operation)
    // 2. Motor assisting stalled tack (bow stuck in irons)
    // 3. Motor assisting in low wind (insufficient wind for sailing)
    if ((motor_state == UseMotor::USE_MOTOR_ALWAYS) ||
         motor_assist_tack() ||
         motor_assist_low_wind()) {
        // Run PID speed controller to achieve desired speed
        // Same controller used by non-sailboat modes for consistent behavior
        throttle_out = 100.0f * rover.g2.attitude_control.get_throttle_out_speed(desired_speed,
                                                                        rover.g2.motors.limit.throttle_lower,
                                                                        rover.g2.motors.limit.throttle_upper,
                                                                        rover.g.speed_cruise,
                                                                        rover.g.throttle_cruise * 0.01f,
                                                                        rover.G_Dt);
    }

    //
    // SAIL CONTROL: Set sail trim or relax sails based on motor mode
    //
    if (motor_state == UseMotor::USE_MOTOR_ALWAYS) {
        // Pure motor mode: Relax sails to reduce drag and prevent interference
        relax_sails();
    } else {
        // Sailing mode: Actively trim sails based on apparent wind angle
        // Motor may be off (NEVER) or assisting (ASSIST), but sails provide primary propulsion
        set_auto_mainsail(desired_speed);
    }
}

/**
 * @brief Calculate Velocity Made Good toward navigation target
 * 
 * @details VMG is the component of vehicle velocity in the direction of the target.
 * For sailboats, actual heading often differs from target bearing due to wind constraints.
 * VMG provides a true measure of progress toward destination.
 * 
 * Calculation: VMG = speed × cos(angle_between_heading_and_bearing)
 * 
 * Coordinate Frame Calculations:
 * - Vehicle speed: Magnitude of velocity in body frame (m/s)
 * - Vehicle heading: Current yaw in earth frame (radians)
 * - Waypoint bearing: Direction to target in earth frame (centidegrees converted to radians)
 * - Angle difference: Wrapped to ±π for correct cosine calculation
 * 
 * @return VMG in m/s (positive toward target, negative away from target)
 * 
 * @note Currently used for logging and performance analysis only
 * @note Returns raw speed if not in autopilot mode (no target bearing)
 * @note Returns zero if speed estimate is unavailable
 * 
 * @see https://en.wikipedia.org/wiki/Velocity_made_good
 */
float Sailboat::get_VMG() const
{
    // Get current vehicle speed estimate from attitude controller
    float speed;
    if (!rover.g2.attitude_control.get_forward_speed(speed)) {
        // Speed estimation not available (GPS lost, EKF unhealthy, etc.)
        return 0.0f;
    }

    // In manual modes, no waypoint bearing exists - return raw speed
    if (!rover.control_mode->is_autopilot_mode()) {
        return speed;
    }

    // Calculate VMG using vector projection
    // 1. Get bearing to waypoint (earth frame, centidegrees)
    // 2. Get current heading (earth frame, radians)  
    // 3. Calculate angle between them
    // 4. Project speed vector onto bearing vector using cosine
    // Result: Speed component in direction of target
    return (speed * cosf(wrap_PI(radians(rover.g2.wp_nav.wp_bearing_cd() * 0.01f) - rover.ahrs.get_yaw_rad())));
}

// handle user initiated tack while in acro mode
void Sailboat::handle_tack_request_acro()
{
    if (!tack_enabled() || currently_tacking) {
        return;
    }
    // set tacking heading target to the current angle relative to the true wind but on the new tack
    currently_tacking = true;
    tack_heading_rad = wrap_2PI(rover.ahrs.get_yaw_rad() + 2.0f * wrap_PI((rover.g2.windvane.get_true_wind_direction_rad() - rover.ahrs.get_yaw_rad())));

    tack_request_ms = AP_HAL::millis();
}

// return target heading in radians when tacking (only used in acro)
float Sailboat::get_tack_heading_rad()
{
    if (fabsf(wrap_PI(tack_heading_rad - rover.ahrs.get_yaw_rad())) < radians(SAILBOAT_TACKING_ACCURACY_DEG) ||
       ((AP_HAL::millis() - tack_request_ms) > SAILBOAT_AUTO_TACKING_TIMEOUT_MS)) {
        clear_tack();
    }

    return tack_heading_rad;
}

// handle user initiated tack while in autonomous modes (Auto, Guided, RTL, SmartRTL, etc)
void Sailboat::handle_tack_request_auto()
{
    if (!tack_enabled() || currently_tacking) {
        return;
    }

    // record time of request for tack.  This will be processed asynchronously by sailboat_calc_heading
    tack_request_ms = AP_HAL::millis();
}

// clear tacking state variables
void Sailboat::clear_tack()
{
    currently_tacking = false;
    tack_assist = false;
    tack_request_ms = 0;
    tack_clear_ms = AP_HAL::millis();
}

// returns true if boat is currently tacking
bool Sailboat::tacking() const
{
    return tack_enabled() && currently_tacking;
}

/**
 * @brief Determine if indirect tacking navigation is required for desired heading
 * 
 * @details Sailboats cannot sail directly into the wind (no-go zone typically ±45°).
 * When the desired heading falls within this zone, the vehicle must use indirect
 * navigation by tacking back and forth to make progress toward the goal.
 * 
 * This function checks if the desired heading is achievable with direct navigation
 * or requires the sailboat-specific tacking algorithm.
 * 
 * Coordinate Frame Calculations:
 * - desired_heading: Earth frame heading toward navigation target (centidegrees)
 * - true_wind_direction: Earth frame wind direction (radians)
 * - Angle between them determines if sailing is geometrically possible
 * 
 * @param[in] desired_heading_cd Desired heading toward target in centidegrees (0-36000)
 * @return true if tacking navigation required, false if direct navigation possible
 * 
 * @note SAILBOAT_NOGO_PAD expands no-go zone slightly for controller stability
 * @note Returns true during active tack to maintain tacking state
 */
bool Sailboat::use_indirect_route(float desired_heading_cd) const
{
    if (!tack_enabled()) {
        return false;
    }

    // Always use sailboat controller (indirect route) while actively tacking
    // Prevents interruption of tack maneuver mid-execution
    if (currently_tacking) {
        return true;
    }

    // Convert desired heading from centidegrees to radians for angle calculations
    const float desired_heading_rad = radians(desired_heading_cd * 0.01f);

    // Calculate angle between desired heading and true wind direction
    // If angle is less than (no_go + pad), desired heading is in no-go zone
    // 
    // Padding the no-go zone (SAILBOAT_NOGO_PAD) provides hysteresis:
    // - Allows heading controller to be used near no-go boundary
    // - Prevents rapid switching between direct and indirect navigation
    // - Improves control stability when sailing close to the wind
    return fabsf(wrap_PI(rover.g2.windvane.get_true_wind_direction_rad() - desired_heading_rad)) <= radians(sail_no_go + SAILBOAT_NOGO_PAD);
}

/**
 * @brief Calculate best achievable heading when desired heading is in no-go zone
 * 
 * @details This function implements the sailboat tacking state machine for autonomous navigation.
 * When the desired heading is directly upwind (in the no-go zone where sailing is impossible),
 * the vehicle must tack back and forth on alternating headings to make progress toward the goal.
 * 
 * Tacking Decision Logic:
 * 1. Check if desired heading is achievable (outside no-go zone)
 * 2. Detect if heading change would require tacking vs gybing
 * 3. Monitor cross-track error to trigger tacks when corridor boundary reached
 * 4. Execute tack by setting target heading on opposite tack
 * 5. Hold tack heading until vehicle reaches target or timeout occurs
 * 
 * @param[in] desired_heading_cd Desired heading in centidegrees (0-36000)
 * @return Achievable heading in centidegrees (either desired heading or no-go boundary heading)
 * 
 * @note This function assumes caller has verified use_indirect_route() returned true
 * @note Tacking creates a corridor of width 2*xtrack_max around desired track
 */
float Sailboat::calc_heading(float desired_heading_cd)
{
    if (!tack_enabled()) {
        return desired_heading_cd;
    }
    bool should_tack = false;

    // Determine current tack based on apparent wind direction
    // TACK_PORT: Wind over port (left) side - sailing on port tack
    // TACK_STARBOARD: Wind over starboard (right) side - sailing on starboard tack
    const AP_WindVane::Sailboat_Tack current_tack = rover.g2.windvane.get_current_tack();

    // Convert desired heading from centidegrees to radians for angle calculations
    const float desired_heading_rad = radians(desired_heading_cd * 0.01f);

    //
    // TACK DETECTION: Check if desired heading is achievable or requires tacking
    //
    
    // Get true wind direction (wind direction in earth frame, independent of vehicle motion)
    const float true_wind_rad = rover.g2.windvane.get_true_wind_direction_rad();
    
    // Calculate angle between desired heading and true wind direction
    // If this angle is greater than no_go angle, heading is achievable without tacking
    // SAILBOAT_NOGO_PAD extends the achievable zone slightly to allow heading controller use
    if (fabsf(wrap_PI(true_wind_rad - desired_heading_rad)) > radians(sail_no_go) && !currently_tacking) {

        // Desired heading is outside no-go zone - potentially achievable
        // But we still need to check if reaching it requires a tack or gybe
        
        // Calculate angle of desired heading relative to true wind
        // Negative = port side, Positive = starboard side
        const float new_heading_apparent_angle = wrap_PI(true_wind_rad - desired_heading_rad);
        
        // Determine which tack the desired heading would put us on
        AP_WindVane::Sailboat_Tack new_tack;
        if (is_negative(new_heading_apparent_angle)) {
            new_tack = AP_WindVane::Sailboat_Tack::TACK_PORT;
        } else {
            new_tack = AP_WindVane::Sailboat_Tack::TACK_STARBOARD;
        }

        // If new tack differs from current tack, determine if maneuver is tack or gybe
        if (new_tack != current_tack) {
            // TACK vs GYBE determination:
            // - TACK: Bow passes through wind (slower, requires precision)
            // - GYBE: Stern passes through wind (faster, less control needed)
            //
            // Geometry test: If sum of apparent wind angle and new heading angle < 180°,
            // then bow must pass through wind (tack required)
            const float app_wind_rad = rover.g2.windvane.get_apparent_wind_direction_rad();
            if (fabsf(app_wind_rad) + fabsf(new_heading_apparent_angle) < M_PI) {
                should_tack = true;
            }
            // If >= 180°, it's a gybe - allow direct heading change without tacking logic
        }

        // If no tack needed, return desired heading as-is
        if (!should_tack) {
            return desired_heading_cd;
        }
    }

    //
    // USER-REQUESTED TACK: Check if pilot manually initiated tack
    //
    uint32_t now = AP_HAL::millis();
    if (tack_request_ms != 0 && !should_tack && !currently_tacking) {
        // User requested tack via RC switch or GCS command
        // Accept request only if made within last 500ms (debounce protection)
        // This allows pilot override of autonomous tacking logic
        should_tack = ((now - tack_request_ms) < 500);
        tack_request_ms = 0;
    }

    //
    // CROSS-TRACK ERROR TACK: Trigger tack when corridor boundary reached
    //
    // Creates a navigation corridor of width 2*xtrack_max centered on desired track
    // When vehicle reaches corridor boundary, tack to head back toward centerline
    // This implements efficient upwind progress through a series of tacks
    //
    const float cross_track_error = rover.g2.wp_nav.crosstrack_error();
    if ((fabsf(cross_track_error) >= xtrack_max) && !is_zero(xtrack_max) && !should_tack && !currently_tacking) {
        // Only tack if the new tack will reduce cross-track error
        // (Don't tack if already heading back toward centerline)
        
        // Starboard tack: Vehicle is angling left, approaching left boundary
        // Cross-track error is positive when left of track
        if (is_positive(cross_track_error) && (current_tack == AP_WindVane::Sailboat_Tack::TACK_STARBOARD)) {
            should_tack = true;
        }
        
        // Port tack: Vehicle is angling right, approaching right boundary  
        // Cross-track error is negative when right of track
        if (is_negative(cross_track_error) && (current_tack == AP_WindVane::Sailboat_Tack::TACK_PORT)) {
            should_tack = true;
        }
    }

    //
    // TACK HEADING CALCULATION: Compute target headings on port and starboard tacks
    //
    // No-go zone boundaries: sail_no_go degrees on either side of true wind
    // Left boundary (port tack heading): true_wind + no_go_angle
    // Right boundary (starboard tack heading): true_wind - no_go_angle
    const float left_no_go_heading_rad = wrap_2PI(true_wind_rad + radians(sail_no_go));
    const float right_no_go_heading_rad = wrap_2PI(true_wind_rad - radians(sail_no_go));

    //
    // TACK EXECUTION: Initiate tack maneuver if trigger conditions met
    //
    if (should_tack && (now - tack_clear_ms) > TACK_RETRY_TIME_MS) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Sailboat: Tacking");
        
        // Set target heading on opposite tack (switch to other no-go boundary)
        // From port tack → starboard tack (right boundary)
        // From starboard tack → port tack (left boundary)
        switch (current_tack) {
            case AP_WindVane::Sailboat_Tack::TACK_PORT:
                tack_heading_rad = right_no_go_heading_rad;
                break;
            case AP_WindVane::Sailboat_Tack::TACK_STARBOARD:
                tack_heading_rad = left_no_go_heading_rad;
                break;
        }
        
        // Enter tacking state - will hold this heading until complete or timeout
        currently_tacking = true;
        auto_tack_start_ms = now;
    }

    //
    // TACK COMPLETION MONITORING: Hold tack heading until vehicle reaches target
    //
    if (currently_tacking) {
        // Check if tack is complete: Vehicle heading within accuracy tolerance of target
        if (fabsf(wrap_PI(tack_heading_rad - rover.ahrs.get_yaw_rad())) <= radians(SAILBOAT_TACKING_ACCURACY_DEG)) {
            clear_tack();
        } 
        // Check for tack timeout: Taking too long (may be stuck in irons)
        else if ((now - auto_tack_start_ms) > SAILBOAT_AUTO_TACKING_TIMEOUT_MS) {
            // Tack has exceeded normal completion time
            
            if ((motor_state == UseMotor::USE_MOTOR_ASSIST) && (now - auto_tack_start_ms) < (3.0f * SAILBOAT_AUTO_TACKING_TIMEOUT_MS)) {
                // MOTOR ASSIST ACTIVATION: Use motor to complete stalled tack
                // Allow up to 3x normal timeout with motor assistance
                // Motor provides additional thrust to push bow through wind
                tack_assist = true;
            } else {
                // Tack completely failed - give up and return to normal navigation
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Sailboat: Tacking timed out");
                clear_tack();
            }
        }
        
        // Return tack target heading (converted back to centidegrees)
        // Heading controller will drive vehicle toward this heading
        return degrees(tack_heading_rad) * 100.0f;
    }

    //
    // NORMAL UPWIND SAILING: Not currently tacking, sail on current tack's no-go boundary
    //
    // Return appropriate no-go boundary heading based on current tack
    // This keeps vehicle sailing as close to desired upwind direction as physically possible
    // Port tack: Sail on left boundary (wind from port side)
    // Starboard tack: Sail on right boundary (wind from starboard side)
    if (current_tack == AP_WindVane::Sailboat_Tack::TACK_PORT) {
        return degrees(left_no_go_heading_rad) * 100.0f;
    } else {
        return degrees(right_no_go_heading_rad) * 100.0f;
    }
}

/**
 * @brief Set motor operating mode for sailboat
 * 
 * @details Configures motor usage policy for hybrid sail/motor boats:
 * - USE_MOTOR_NEVER: Motor disabled, sail-only operation
 * - USE_MOTOR_ASSIST: Motor used only when needed (low wind, tacking)
 * - USE_MOTOR_ALWAYS: Motor always on, sails relaxed (power boat mode)
 * 
 * Motor state change is only allowed if physical motor hardware exists.
 * Detection checks for:
 * - Skid steering configuration (dual motors)
 * - Throttle servo channel assignment
 * - Non-undefined frame type
 * 
 * @param[in] state Desired motor operating mode
 * @param[in] report_failure If true, send GCS warning message on failure
 * 
 * @note Motor can always be disabled regardless of hardware configuration
 * @note Failed enable attempts leave motor_state unchanged
 */
void Sailboat::set_motor_state(UseMotor state, bool report_failure)
{
    // Motor disable is always allowed (safety feature)
    // Even if motor hardware exists, pilot can always disable it
    if (state == UseMotor::USE_MOTOR_NEVER) {
        motor_state = state;
        return;
    }

    // Enable motor assist or always-on mode only if motor hardware is present
    // Check multiple indicators of motor availability:
    // 1. Skid steering configured (dual motor setup)
    // 2. Throttle servo channel assigned (single motor/prop)
    // 3. Frame type defined (motor configuration specified)
    if (rover.g2.motors.have_skid_steering() ||
        SRV_Channels::function_assigned(SRV_Channel::k_throttle) ||
        rover.get_frame_type() != rover.g2.motors.frame_type::FRAME_TYPE_UNDEFINED) {
        motor_state = state;
    } else if (report_failure) {
        // No motor hardware detected - cannot enable motor
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Sailboat: failed to enable motor");
    }
}

/**
 * @brief Check if motor should assist with tacking maneuver
 * 
 * @details Motor assistance for tacking is activated when:
 * - Motor is configured in ASSIST mode (not NEVER or ALWAYS)
 * - Tack is in progress and has exceeded normal completion time
 * - Vehicle may be stuck in irons (head-to-wind stall condition)
 * 
 * Motor provides additional thrust to push bow through wind when sail
 * drive alone is insufficient. Typically needed in light wind conditions
 * or when tacking into heavy seas.
 * 
 * @return true if motor should provide tacking assistance, false otherwise
 * 
 * @note tack_assist flag is set by tack timeout logic in calc_heading()
 * @note Motor assistance continues until tack completes or fully times out
 */
bool Sailboat::motor_assist_tack() const
{
    // Motor assist only available in ASSIST mode
    // In NEVER mode: No motor available
    // In ALWAYS mode: Motor already running continuously
    if (motor_state != UseMotor::USE_MOTOR_ASSIST) {
        return false;
    }

    // Return tack assistance flag (set when tack is taking too long)
    return tack_assist;
}

/**
 * @brief Check if motor should assist due to insufficient wind
 * 
 * @details Motor assistance for low wind is activated when:
 * - Motor is configured in ASSIST mode (not NEVER or ALWAYS)
 * - Minimum wind speed threshold is configured (sail_windspeed_min > 0)
 * - Wind speed sensor is enabled and providing valid readings
 * - Current true wind speed is below configured minimum threshold
 * 
 * This allows sailboats to maintain mobility in light wind conditions
 * where sail propulsion alone would be inadequate. Motor augments or
 * replaces sail drive until wind speed increases above threshold.
 * 
 * @return true if motor should assist due to low wind, false otherwise
 * 
 * @note Uses true wind speed (earth frame), not apparent wind speed
 * @note Motor automatically disables when wind speed rises above threshold
 * @note Helps prevent getting stuck in no-wind conditions
 */
bool Sailboat::motor_assist_low_wind() const
{
    // Motor assist only available in ASSIST mode
    // In NEVER mode: No motor available
    // In ALWAYS mode: Motor already running continuously
    if (motor_state != UseMotor::USE_MOTOR_ASSIST) {
        return false;
    }

    // Enable motor assistance if all conditions met:
    // 1. Minimum wind threshold configured (positive value set by user)
    // 2. Wind speed sensor enabled and functional
    // 3. Current wind speed below minimum sailing threshold
    return (is_positive(sail_windspeed_min) &&
            rover.g2.windvane.wind_speed_enabled() &&
            (rover.g2.windvane.get_true_wind_speed() < sail_windspeed_min));
}
