/**
 * @file Attitude.cpp
 * @brief Attitude control helper functions for ArduSub
 * 
 * @details This file contains helper functions that process pilot input and convert
 *          it into desired attitude angles and rates for the underwater vehicle.
 *          These functions integrate with the AC_AttitudeControl library to provide
 *          pilot command interpretation, input scaling, and coordinate frame transformations
 *          specific to underwater vehicle operations.
 *          
 *          Key responsibilities:
 *          - Transform pilot stick inputs to desired lean angles (angle mode)
 *          - Transform pilot stick inputs to desired body frame rates (rate mode)
 *          - Calculate desired yaw rates from pilot input
 *          - Handle throttle/climb rate conversions with deadzone processing
 *          - Provide coordinate frame transformations (body frame <-> NE frame)
 *          - Manage ROI (Region of Interest) yaw control
 *          
 *          Coordinate Frame Conventions:
 *          - Body frame: x-forward, y-right, z-down relative to vehicle
 *          - NE frame: North-East reference frame
 *          - Angles in centidegrees (1 degree = 100 centidegrees)
 *          - Rates in centidegrees per second
 * 
 * @note All angle outputs are in centidegrees for consistency with ArduPilot conventions
 * @warning These functions are called at high frequency (up to 400Hz) from the main loop
 * 
 * Source: ArduSub/Attitude.cpp:1-end
 */

#include "Sub.h"

/**
 * @brief Transform pilot's roll and pitch input into desired lean angles
 * 
 * @details This function converts pilot stick inputs into desired vehicle lean angles
 *          for angle-stabilized flight modes. It performs several critical operations:
 *          
 *          1. Input Scaling: Scales raw pilot input to the configured ANGLE_MAX parameter
 *          2. Circular Limiting: Applies vector magnitude limiting to prevent excessive
 *             combined roll/pitch angles while preserving input direction
 *          3. Tilt Compensation: Converts lateral tilt to euler roll to account for
 *             the coupling between pitch and roll in the euler angle representation
 *          
 *          Algorithm Flow:
 *          - Validate and constrain ANGLE_MAX parameter (10-80 degrees)
 *          - Scale inputs from normalized range to angle_max range
 *          - Apply circular limit: if sqrt(roll^2 + pitch^2) > angle_max, scale both
 *          - Convert lateral tilt to euler roll using: roll_euler = atan(cos(pitch) * tan(roll_lateral))
 *          
 *          The circular limiting ensures that when pilot commands maximum deflection
 *          in multiple axes, the resulting lean angle magnitude doesn't exceed angle_max,
 *          while maintaining the pilot's intended direction vector.
 * 
 * @param[in]  roll_in   Pilot roll input in normalized units (typically -4500 to 4500)
 * @param[in]  pitch_in  Pilot pitch input in normalized units (typically -4500 to 4500)
 * @param[out] roll_out  Desired roll angle in centidegrees (body frame)
 * @param[out] pitch_out Desired pitch angle in centidegrees (body frame)
 * @param[in]  angle_max Maximum lean angle in centidegrees (constrained to 1000-8000 cd)
 * 
 * @note Called from angle-stabilized flight modes (Stabilize, AltHold, PosHold, etc.)
 * @note ROLL_PITCH_INPUT_MAX defines the normalized input range (typically 4500)
 * @warning Modifying the circular limit or tilt conversion can affect vehicle stability
 * 
 * @see AC_AttitudeControl::input_euler_angle_roll_pitch_euler()
 * @see ANGLE_MAX parameter definition
 */
void Sub::get_pilot_desired_lean_angles(float roll_in, float pitch_in, float &roll_out, float &pitch_out, float angle_max)
{
    // Sanity check and constrain ANGLE_MAX parameter to safe range (10-80 degrees in centidegrees)
    // This prevents configuration errors that could lead to excessive lean angles
    aparm.angle_max.set(constrain_int16(aparm.angle_max,1000,8000));

    // Limit the requested angle_max to the configured maximum
    // Allows individual modes to request lower limits while respecting global maximum
    angle_max = constrain_float(angle_max, 1000, aparm.angle_max);

    // Input Scaling: Convert normalized pilot input to angle in centidegrees
    // ROLL_PITCH_INPUT_MAX is the normalized input range (typically 4500)
    // This scales pilot stick deflection to the configured ANGLE_MAX parameter
    float scaler = aparm.angle_max/(float)ROLL_PITCH_INPUT_MAX;
    roll_in *= scaler;
    pitch_in *= scaler;

    // Circular Limiting: Prevent combined roll/pitch magnitude from exceeding angle_max
    // Calculate the vector magnitude (Euclidean distance) of the roll/pitch command
    float total_in = norm(pitch_in, roll_in);
    if (total_in > angle_max) {
        // Scale both axes proportionally to maintain direction while limiting magnitude
        // This preserves the pilot's intended direction vector
        float ratio = angle_max / total_in;
        roll_in *= ratio;
        pitch_in *= ratio;
    }

    // Lateral Tilt to Euler Roll Conversion:
    // Convert from lateral tilt (body frame x-y plane) to euler roll angle
    // This accounts for the coupling between pitch and roll in euler angle representation
    // Formula: roll_euler = atan(cos(pitch) * tan(roll_lateral))
    // Conversion factors: 18000/M_PI converts radians to centidegrees, M_PI/18000 converts centidegrees to radians
    roll_in = (18000/M_PI) * atanf(cosf(pitch_in*(M_PI/18000))*tanf(roll_in*(M_PI/18000)));

    // Output the calculated desired angles in centidegrees (body frame)
    roll_out = roll_in;
    pitch_out = pitch_in;
}

/**
 * @brief Transform pilot's yaw input into desired yaw rate
 * 
 * @details Converts pilot yaw stick input into a desired yaw rate for rate-controlled
 *          yaw modes. This function applies a simple proportional scaling using the
 *          ACRO_YAW_P parameter, which determines yaw rate sensitivity.
 *          
 *          The scaling is linear without expo curve or deadzone - direct proportional
 *          conversion from stick input to body frame yaw rate. This gives pilot
 *          direct rate control with sensitivity determined by ACRO_YAW_P gain.
 *          
 *          Algorithm:
 *          desired_yaw_rate = stick_input × ACRO_YAW_P
 *          
 *          Typical ACRO_YAW_P values range from 1.0 to 4.5, with higher values
 *          providing more aggressive yaw response to pilot input.
 * 
 * @param[in] stick_angle Pilot yaw stick input in normalized units (typically -4500 to 4500)
 * 
 * @return Desired yaw rate in centidegrees per second (body frame z-axis rotation rate)
 * 
 * @note Called at main loop rate (typically 50-400Hz depending on vehicle configuration)
 * @note No deadzone or expo curve applied - pure linear scaling for direct rate control
 * @note Positive yaw rate corresponds to clockwise rotation when viewed from above (NED convention)
 * 
 * @see ACRO_YAW_P parameter for yaw rate sensitivity adjustment
 * @see AC_AttitudeControl::input_rate_bf_roll_pitch_yaw() for rate controller usage
 */
float Sub::get_pilot_desired_yaw_rate(int16_t stick_angle) const
{
    // Convert pilot input to desired yaw rate using proportional gain
    // stick_angle: normalized pilot input (typically -4500 to 4500)
    // g.acro_yaw_p: rate sensitivity parameter (typically 1.0 to 4.5)
    // Result: desired yaw rate in centidegrees/second
    return stick_angle * g.acro_yaw_p;
}

/**
 * @brief Check for EKF yaw reset and adjust attitude control targets
 * 
 * @details The Extended Kalman Filter (EKF) may occasionally reset its yaw estimate
 *          when it detects inconsistencies or receives new absolute heading information
 *          (e.g., from compass or GPS). When this occurs, the attitude controller's
 *          inertial frame references must be updated to prevent sudden unwanted vehicle
 *          movements.
 *          
 *          This function polls the AHRS for yaw reset events and notifies the attitude
 *          controller to reset its inertial frame references accordingly. The timestamp
 *          check ensures each reset is processed exactly once.
 *          
 *          Called at high frequency from the main loop to ensure timely detection and
 *          handling of yaw resets.
 * 
 * @note Should be called every loop iteration (typically 50-400Hz)
 * @note EKF yaw resets are relatively rare events but must be handled immediately
 * @warning Failure to handle yaw resets can cause sudden vehicle rotation commands
 * 
 * @see AP_AHRS::getLastYawResetAngle()
 * @see AC_AttitudeControl::inertial_frame_reset()
 * @see AP_NavEKF3 for EKF implementation details
 */
void Sub::check_ekf_yaw_reset()
{
    float yaw_angle_change_rad;
    // Query AHRS for the most recent yaw reset event timestamp and angle change
    uint32_t new_ekfYawReset_ms = ahrs.getLastYawResetAngle(yaw_angle_change_rad);
    
    // Check if a new yaw reset has occurred since last check (timestamp changed)
    if (new_ekfYawReset_ms != ekfYawReset_ms) {
        // Notify attitude controller to reset its inertial frame references
        // This prevents the controller from fighting the EKF's yaw correction
        attitude_control.inertial_frame_reset();
        
        // Update our stored timestamp to mark this reset as processed
        ekfYawReset_ms = new_ekfYawReset_ms;
    }
}

/*************************************************************
 * yaw controllers
 *************************************************************/

/**
 * @brief Calculate heading towards Region of Interest (ROI) waypoint
 * 
 * @details Returns the bearing from the vehicle's current position to a stored ROI
 *          waypoint location. This enables the vehicle to point its camera or sensors
 *          towards a specific location regardless of the vehicle's movement direction.
 *          
 *          The function implements rate limiting by only recalculating the bearing
 *          every 4th call, effectively reducing the update rate to ~100Hz when called
 *          at 400Hz. This optimization reduces computational load while maintaining
 *          sufficient update frequency for smooth yaw control.
 *          
 *          Algorithm:
 *          1. Increment counter on each call
 *          2. Every 4th call: recalculate bearing from current position to roi_WP
 *          3. Always return the most recently calculated bearing
 * 
 * @return Bearing to ROI waypoint in centidegrees (0-36000, 0=North, 9000=East)
 * 
 * @note Designed to be called at 400Hz but updates bearing at 100Hz for efficiency
 * @note Bearing is in NE (North-East) frame, not body frame
 * @note roi_WP location is set by MAVLink ROI commands or mission items
 * 
 * @see get_bearing_cd() for bearing calculation details
 * @see MAV_CMD_DO_SET_ROI for MAVLink ROI command
 */
float Sub::get_roi_yaw()
{
    static uint8_t roi_yaw_counter = 0;     // Counter for rate limiting (reduces updates to 100Hz)

    roi_yaw_counter++;
    if (roi_yaw_counter >= 4) {
        // Reset counter and recalculate bearing every 4th call
        roi_yaw_counter = 0;
        
        // Calculate bearing from current position (NE frame) to ROI waypoint
        // Returns bearing in centidegrees (0-36000)
        yaw_look_at_WP_bearing = get_bearing_cd(inertial_nav.get_position_xy_cm(), roi_WP.xy());
    }

    // Return the cached bearing value (updated every 4th call)
    return yaw_look_at_WP_bearing;
}

/**
 * @brief Calculate yaw angle to point vehicle in direction of travel
 * 
 * @details Automatically orients the vehicle to face its direction of motion by
 *          calculating the bearing from the velocity vector. This "look ahead" behavior
 *          is useful for maintaining camera or sensor orientation towards the direction
 *          of travel during autonomous missions.
 *          
 *          The function only updates the bearing when:
 *          1. Position estimate is valid (position_ok() returns true)
 *          2. Vehicle speed exceeds YAW_LOOK_AHEAD_MIN_SPEED threshold
 *          
 *          Below minimum speed, the previous bearing is maintained to prevent
 *          erratic yaw behavior from velocity estimate noise at low speeds.
 *          
 *          Algorithm:
 *          - Get velocity vector in NE (North-East-Up) frame
 *          - Calculate velocity magnitude in horizontal plane
 *          - If above threshold: bearing = atan2(vel_y, vel_x) converted to centidegrees
 *          - If below threshold: maintain previous bearing
 * 
 * @return Bearing aligned with velocity vector in centidegrees (0-36000, 0=North, 9000=East)
 * 
 * @note Bearing only updates when vehicle speed exceeds YAW_LOOK_AHEAD_MIN_SPEED
 * @note Uses NEU frame velocity (North-East-Up) for calculation
 * @note Returns cached value when speed is below threshold to prevent jitter
 * 
 * @see YAW_LOOK_AHEAD_MIN_SPEED constant definition
 * @see position_ok() for position estimate validity check
 */
float Sub::get_look_ahead_yaw()
{
    // Get current velocity estimate in NEU (North-East-Up) frame in cm/s
    const Vector3f& vel = inertial_nav.get_velocity_neu_cms();
    
    // Calculate horizontal speed squared (avoids sqrt for efficiency)
    const float speed_sq = vel.xy().length_squared();
    
    // Only update look-ahead bearing if position is valid and speed exceeds minimum threshold
    // This prevents erratic yaw from velocity noise at low speeds
    if (position_ok() && (speed_sq > (YAW_LOOK_AHEAD_MIN_SPEED * YAW_LOOK_AHEAD_MIN_SPEED))) {
        // Calculate bearing from velocity vector using atan2(y, x)
        // atan2 returns angle in radians, convert to degrees then to centidegrees
        // Result is bearing in NE frame (0=North, 90=East, 180=South, 270=West)
        yaw_look_ahead_bearing = degrees(atan2f(vel.y,vel.x))*100.0f;
    }
    // If speed below threshold or position invalid, return previously calculated bearing
    return yaw_look_ahead_bearing;
}

/*************************************************************
 *  throttle control
 ****************************************************************/

/**
 * @brief Transform pilot's throttle input to vertical climb rate
 * 
 * @details Converts pilot throttle stick position into a desired vertical climb/descent
 *          rate in cm/s for underwater vehicles. This function implements a deadzone
 *          around the center stick position and scales inputs above/below the deadzone
 *          to configured maximum climb and descent rates.
 *          
 *          Algorithm:
 *          1. Check failsafe status - return 0 if pilot input failsafe active
 *          2. Calculate deadzone boundaries around mid-stick position
 *          3. Constrain and validate throttle input and deadzone parameter
 *          4. Scale input based on which region it falls in:
 *             - Below deadzone: scale to descent rate (PILOT_SPEED_DN)
 *             - Above deadzone: scale to ascent rate (PILOT_SPEED_UP)
 *             - Inside deadzone: return 0 (hold current depth)
 *          
 *          The deadzone prevents drift from pilot input noise and provides a stable
 *          "hold depth" region. Scaling is linear outside the deadzone for proportional
 *          rate control.
 *          
 *          Coordinate Convention:
 *          - Positive rate = ascending (moving up in water column)
 *          - Negative rate = descending (moving down in water column)
 * 
 * @param[in] throttle_control Pilot throttle input (0-1000, center typically ~500)
 * 
 * @return Desired climb rate in cm/s (positive=up, negative=down, body frame z-axis)
 * 
 * @note Returns 0 if pilot input failsafe is active
 * @note Deadzone size controlled by THROTTLE_DEADZONE parameter
 * @note Maximum ascent rate set by PILOT_SPEED_UP parameter
 * @note Maximum descent rate set by PILOT_SPEED_DN parameter (uses PILOT_SPEED_UP if zero)
 * @warning Throttle input must be in range 0-1000 to function correctly
 * 
 * @see PILOT_SPEED_UP parameter for maximum ascent rate
 * @see PILOT_SPEED_DN parameter for maximum descent rate
 * @see THROTTLE_DEADZONE parameter for deadzone size (0-400)
 */
float Sub::get_pilot_desired_climb_rate(float throttle_control)
{
    // Throttle failsafe check - return zero rate if pilot input is lost or invalid
    if (failsafe.pilot_input) {
        return 0.0f;
    }

    // Get center stick position and calculate deadzone boundaries
    float mid_stick = channel_throttle->get_control_mid();
    float deadband_top = mid_stick + g.throttle_deadzone * gain;
    float deadband_bottom = mid_stick - g.throttle_deadzone * gain;

    // Ensure throttle value is within valid range (0-1000)
    // Constrains out-of-range inputs that could cause calculation errors
    throttle_control = constrain_float(throttle_control,0.0f,1000.0f);

    // Ensure deadzone parameter is reasonable (0-400, or 0-40% of stick range)
    // Prevents excessive deadzone that would make control difficult
    g.throttle_deadzone.set(constrain_int16(g.throttle_deadzone, 0, 400));

    // Determine which region the throttle input falls in and scale accordingly
    if (throttle_control < deadband_bottom) {
        // Below the deadband - pilot commanding descent
        // Scale linearly from 0 at deadband_bottom to full descent rate at stick bottom
        // Negative rate = descending
        return get_pilot_speed_dn() * (throttle_control-deadband_bottom) / deadband_bottom;
    } else if (throttle_control > deadband_top) {
        // Above the deadband - pilot commanding ascent
        // Scale linearly from 0 at deadband_top to full ascent rate at stick top
        // Positive rate = ascending
        return g.pilot_speed_up * (throttle_control-deadband_top) / (1000.0f-deadband_top);
    } else {
        // Inside the deadband - pilot commanding depth hold
        // Return zero rate to maintain current depth
        return 0.0f;
    }
}

/**
 * @brief Transform pilot's horizontal stick input to desired horizontal rate
 * 
 * @details Converts pilot forward/lateral stick position into a desired horizontal
 *          movement rate in cm/s. Similar to get_pilot_desired_climb_rate() but for
 *          horizontal motion (forward/back or left/right depending on which channel).
 *          
 *          Key differences from climb rate:
 *          - Uses normalized input (-1.0 to +1.0) with center trim
 *          - Symmetric scaling (same max speed forward/back or left/right)
 *          - Deadzone is symmetric around center (both positive and negative)
 *          - Single PILOT_SPEED parameter for both directions
 *          
 *          Algorithm:
 *          1. Check failsafe - return 0 if pilot input lost
 *          2. Get normalized stick input (-1.0 to +1.0, center=0)
 *          3. Calculate symmetric deadzone boundaries
 *          4. Scale input outside deadzone to PILOT_SPEED rate
 *          5. Return 0 inside deadzone for stable hold
 *          
 *          The deadzone prevents drift and provides stable "hold position" behavior
 *          when stick is near center.
 * 
 * @param[in] channel RC channel pointer (forward, lateral, or strafe stick)
 * 
 * @return Desired horizontal rate in cm/s (sign depends on stick direction and channel)
 * 
 * @note Returns 0 if pilot input failsafe is active
 * @note Used for both forward/back and left/right stick inputs
 * @note Deadzone size derived from THROTTLE_DEADZONE parameter
 * @note Maximum rate set by PILOT_SPEED parameter (symmetric for both directions)
 * @note Input is normalized (-1.0 to +1.0) unlike throttle which is 0-1000
 * 
 * @see PILOT_SPEED parameter for maximum horizontal rate
 * @see THROTTLE_DEADZONE parameter (reused for horizontal deadzone calculation)
 * @see get_pilot_desired_climb_rate() for similar vertical rate calculation
 */
float Sub::get_pilot_desired_horizontal_rate(RC_Channel *channel) const
{
    // Failsafe check - return zero rate if pilot input is lost or invalid
    if (failsafe.pilot_input) {
        return 0;
    }

    // Get normalized stick input (-1.0 to +1.0)
    // Forward and lateral sticks have center trim, unlike throttle
    auto control = channel->norm_input();

    // Calculate normalized deadzone size as fraction of total stick range
    // Converts THROTTLE_DEADZONE (0-400) to normalized units
    // Factor of 2.0 accounts for stick range being -1 to +1 (total range = 2.0)
    auto dz = (float)g.throttle_deadzone * 2.0f / (float)(channel->get_radio_max() - channel->get_radio_min());
    
    // Calculate symmetric deadzone boundaries around center (0)
    auto deadband_top = dz * gain;      // Positive deadzone boundary
    auto deadband_bottom = -dz * gain;  // Negative deadzone boundary

    // Determine region and scale to desired rate
    if (control < deadband_bottom) {
        // Below deadband - pilot commanding negative direction (back or left)
        // Scale linearly from 0 at deadband edge to full rate at maximum stick deflection
        return (float)g.pilot_speed * (control - deadband_bottom);
    } else if (control > deadband_top) {
        // Above deadband - pilot commanding positive direction (forward or right)
        // Scale linearly from 0 at deadband edge to full rate at maximum stick deflection
        return (float)g.pilot_speed * (control - deadband_top);
    } else {
        // Inside deadband - pilot commanding hold position
        // Return zero rate for stable position hold
        return 0;
    }
}

/**
 * @brief Rotate 2D vector from body frame to North-East frame
 * 
 * @details Transforms a 2D vector from the vehicle's body frame (x-forward, y-right)
 *          to the North-East (NE) inertial reference frame using the vehicle's current
 *          yaw angle. This is a standard 2D rotation matrix transformation.
 *          
 *          Coordinate Frame Definitions:
 *          - Body frame: x-axis points forward, y-axis points right (relative to vehicle)
 *          - NE frame: x-axis points North, y-axis points East (inertial reference)
 *          
 *          Rotation Matrix (2D):
 *          | x_NE |   | cos(yaw)  -sin(yaw) | | x_body |
 *          | y_NE | = | sin(yaw)   cos(yaw) | | y_body |
 *          
 *          This transformation is commonly needed when converting pilot commands
 *          (given in body frame) to position control commands (given in NE frame)
 *          or vice versa.
 *          
 *          Algorithm:
 *          1. Get current yaw angle from AHRS (cos and sin precomputed for efficiency)
 *          2. Apply 2D rotation matrix transformation
 *          3. Update input variables with rotated values
 * 
 * @param[in,out] x X-component of vector (input: body frame forward, output: North component)
 * @param[in,out] y Y-component of vector (input: body frame right, output: East component)
 * 
 * @note Input parameters are modified in-place with the rotated values
 * @note Uses AHRS yaw estimate for rotation angle
 * @note Common use: converting pilot lateral/forward commands to North/East rates
 * @warning Accuracy depends on AHRS yaw estimate quality
 * 
 * @see AP_AHRS for yaw angle estimation
 * @see Inverse transformation would use transpose of rotation matrix (or negate yaw angle)
 */
void Sub::rotate_body_frame_to_NE(float &x, float &y)
{
    // Apply 2D rotation matrix transformation using current yaw angle
    // Rotation formula: [x_NE, y_NE] = [cos(θ) -sin(θ); sin(θ) cos(θ)] * [x_body, y_body]
    float ne_x = x*ahrs.cos_yaw() - y*ahrs.sin_yaw();  // North component
    float ne_y = x*ahrs.sin_yaw() + y*ahrs.cos_yaw();  // East component
    
    // Update input variables with rotated values (in-place modification)
    x = ne_x;
    y = ne_y;
}

/**
 * @brief Get configured descent speed with fallback to ascent speed
 * 
 * @details Returns the maximum descent rate for pilot-commanded vertical motion.
 *          If PILOT_SPEED_DN parameter is configured (non-zero), that value is used.
 *          If PILOT_SPEED_DN is zero (unconfigured), falls back to using PILOT_SPEED_UP,
 *          providing symmetric ascent/descent rates.
 *          
 *          This fallback behavior allows simplified configuration where users only
 *          need to set PILOT_SPEED_UP for symmetric vertical motion, while still
 *          supporting asymmetric rates (different up/down speeds) if desired.
 *          
 *          Rationale for asymmetric rates:
 *          - Some vehicles may ascend faster than they descend (or vice versa)
 *          - Different buoyancy characteristics affect optimal rates
 *          - Safety considerations may require slower descent rates
 *          
 *          The absolute value ensures positive rate regardless of parameter sign.
 * 
 * @return Maximum descent rate in cm/s (always positive value)
 * 
 * @note Returns abs(PILOT_SPEED_DN) if configured (non-zero)
 * @note Returns abs(PILOT_SPEED_UP) if PILOT_SPEED_DN is zero
 * @note Absolute value ensures positive rate value
 * @note Rate will be negated when applied (descent = negative rate in NED convention)
 * 
 * @see PILOT_SPEED_DN parameter for configured descent rate
 * @see PILOT_SPEED_UP parameter for ascent rate and fallback descent rate
 * @see get_pilot_desired_climb_rate() for usage in rate calculation
 */
uint16_t Sub::get_pilot_speed_dn() const
{
    // If PILOT_SPEED_DN is configured (non-zero), use it
    if (g.pilot_speed_dn == 0) {
        // PILOT_SPEED_DN not configured - fall back to symmetric rate using PILOT_SPEED_UP
        // Absolute value ensures positive rate regardless of parameter sign
        return abs(g.pilot_speed_up);
    }
    // Return configured descent speed (absolute value for safety)
    return abs(g.pilot_speed_dn);
}
