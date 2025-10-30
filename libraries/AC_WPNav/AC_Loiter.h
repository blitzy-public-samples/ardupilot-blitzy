/**
 * @file AC_Loiter.h
 * @brief Loiter position controller for multicopters with predictive feed-forward control
 * 
 * @details This module implements station-keeping (loiter) control for multirotors, providing
 *          stable position hold with responsive pilot input. The controller uses predictive
 *          feed-forward to anticipate attitude response and provide smooth, natural control feel.
 *          
 *          Key Features:
 *          - Predictive feed-forward control using input_shaping_rate_predictor for anticipating
 *            attitude response before actual vehicle movement
 *          - Jerk-limited braking with configurable delay, acceleration, and jerk parameters
 *          - Fake wind resistance model for more natural stick feel during pilot input
 *          - Integration with AC_PosControl for position/velocity/acceleration targeting
 *          - Integration with AC_Avoid for object avoidance during loiter operations
 *          - Configurable maximum speed, acceleration, and lean angle limits
 *          
 *          Coordinate System:
 *          - Positions in NE (North-East) frame, cm from EKF origin
 *          - Horizontal control only (altitude managed by separate controller)
 *          
 *          Typical Usage:
 *          1. Initialize target position with init_target() or init_target_cm()
 *          2. Call update() at loop rate (~100Hz or configured rate)
 *          3. Provide pilot input via set_pilot_desired_acceleration_rad/cd()
 *          4. Extract desired roll/pitch with get_roll/pitch_rad/cd()
 *          5. Feed roll/pitch to stabilize controllers for attitude control
 * 
 * @note This controller is typically called at main loop rate (100-400Hz depending on vehicle)
 * 
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_Common/Location.h>
#include <AC_AttitudeControl/AC_PosControl.h>
#include <AC_AttitudeControl/AC_AttitudeControl.h>
#include <AC_Avoidance/AC_Avoid.h>

/**
 * @class AC_Loiter
 * @brief Loiter (station-keeping) position controller for multirotors
 * 
 * @details AC_Loiter implements horizontal position hold (loiter) with pilot stick input
 *          mapped to desired accelerations. The controller uses predictive feed-forward
 *          to anticipate attitude response, providing smooth and responsive control.
 *          
 *          Algorithm Overview:
 *          1. Pilot stick input → desired acceleration (set_pilot_desired_acceleration)
 *          2. Predictive feed-forward anticipates attitude response using
 *             input_shaping_rate_predictor to predict roll/pitch angles and rates
 *          3. Desired acceleration integrated to desired velocity with fake wind resistance
 *          4. Velocity sent to AC_PosControl for position/velocity/acceleration targeting
 *          5. AC_PosControl generates desired roll/pitch angles
 *          
 *          Braking Behavior:
 *          When pilot releases sticks, loiter initiates jerk-limited braking:
 *          - Configurable delay before braking starts (LOIT_BRK_DELAY)
 *          - Maximum braking acceleration (LOIT_BRK_ACCEL)
 *          - Maximum jerk rate during braking (LOIT_BRK_JERK) for smooth deceleration
 *          
 *          Fake Wind Resistance:
 *          Simulates aerodynamic drag proportional to velocity, providing more natural
 *          stick feel by requiring continuous input to maintain acceleration.
 *          
 *          Integration Points:
 *          - AC_PosControl: Converts desired position/velocity/acceleration to attitude
 *          - AC_AttitudeControl: Used to predict attitude response to commands
 *          - AC_Avoid: Provides object avoidance adjustments during loiter
 *          - AP_AHRS: Provides current attitude and position estimates
 *          
 *          Configurable Parameters (via var_info):
 *          - LOIT_ANG_MAX: Maximum pilot commanded lean angle
 *          - LOIT_SPEED: Maximum horizontal speed during loiter
 *          - LOIT_ACC_MAX: Maximum horizontal acceleration
 *          - LOIT_BRK_ACCEL: Maximum braking acceleration
 *          - LOIT_BRK_JERK: Maximum jerk during braking
 *          - LOIT_BRK_DELAY: Delay before braking starts
 * 
 * @warning Modifying speed and acceleration limits affects loiter response characteristics
 *          and vehicle stability. Higher limits may cause overshoot or oscillation if attitude
 *          controller gains are not properly tuned. Always test parameter changes in SITL first.
 * 
 * @note Typical update rate: 100-400Hz (main loop rate)
 * @note Position coordinates in NE frame, centimeters from EKF origin
 * @note Velocities in cm/s, accelerations in cm/s², angles in radians or centidegrees
 * 
 * @see AC_PosControl for position control implementation
 * @see AC_AttitudeControl for attitude control integration
 * @see AC_Avoid for object avoidance integration
 */
class AC_Loiter
{
public:

    /**
     * @brief Construct a new AC_Loiter controller
     * 
     * @details Initializes the loiter position controller with references to required subsystems.
     *          The controller depends on AHRS for position/attitude estimates, AC_PosControl for
     *          converting desired position/velocity to attitude commands, and AC_AttitudeControl
     *          for predicting attitude response to control inputs.
     * 
     * @param[in] ahrs              Reference to AP_AHRS_View for attitude and position estimates
     * @param[in] pos_control       Reference to AC_PosControl for position/velocity control
     * @param[in] attitude_control  Reference to AC_AttitudeControl for attitude prediction
     * 
     * @note Parameters (LOIT_*) are loaded from EEPROM via AP_Param system
     * @note Constructor does not initialize target position - call init_target() before first use
     * 
     * @see init_target() to initialize loiter target position
     * @see init_target_cm() to initialize from specific position
     */
    AC_Loiter(const AP_AHRS_View& ahrs, AC_PosControl& pos_control, const AC_AttitudeControl& attitude_control);

    /**
     * @brief Initialize loiter target to a specific position
     * 
     * @details Sets the loiter target position in horizontal (North-East) frame relative to
     *          EKF origin. Feed-forward velocity is initialized to zero. Use this method to
     *          start loiter at a specific position rather than current vehicle position.
     * 
     * @param[in] position_neu_cm  Target position in NE frame, cm from EKF origin
     * 
     * @note Position units: centimeters (cm) in North-East frame
     * @note Resets desired velocity and acceleration to zero
     * @note Resets brake timer and predicted states
     * 
     * @see init_target() to initialize from current position and velocity
     */
    void init_target_cm(const Vector2f& position_neu_cm);

    /**
     * @brief Initialize loiter target from current position and velocity
     * 
     * @details Sets the loiter target position to current vehicle position from EKF and
     *          initializes feed-forward velocity to current vehicle velocity. This provides
     *          smooth transition into loiter mode without sudden deceleration. Typical use
     *          when entering loiter mode during flight.
     * 
     * @note Queries current position and velocity from AC_PosControl/EKF
     * @note Resets desired acceleration and brake state
     * @note More common initialization method than init_target_cm()
     * 
     * @see init_target_cm() to initialize to specific position with zero velocity
     */
    void init_target();

    /**
     * @brief Reduce loiter response for landing operations
     * 
     * @details Reduces maximum speed and acceleration to gentler values suitable for landing
     *          approach. Softens control response to prevent aggressive corrections during
     *          final descent phase. Typically called when entering landing mode or final
     *          descent phase.
     * 
     * @note Reduces speed limits temporarily - not persistent across mode changes
     * @note Should be called before landing approach begins
     * @note Effect lasts until speed limits are reset via set_speed_max_NE_cms() or mode change
     * 
     * @warning Do not use during normal flight - intended only for landing phase
     * 
     * @see set_speed_max_NE_cms() to restore normal speed limits
     */
    void soften_for_landing();

    /**
     * @brief Set pilot desired acceleration from stick input (radians)
     * 
     * @details Converts pilot stick input (expressed as desired roll/pitch angles) to desired
     *          horizontal acceleration in earth frame (NE). Uses predictive feed-forward to
     *          anticipate attitude response, calculating predicted angles and rates for smooth
     *          control response. Applies input shaping and rate prediction to compensate for
     *          attitude control loop delays.
     *          
     *          Algorithm:
     *          1. Convert euler angles to desired acceleration using gravity projection
     *          2. Predict resulting attitude angles and rates using input_shaping_rate_predictor
     *          3. Store predicted values for feed-forward in update() loop
     * 
     * @param[in] euler_roll_angle_rad   Pilot commanded roll angle in radians (+ = roll right)
     * @param[in] euler_pitch_angle_rad  Pilot commanded pitch angle in radians (+ = pitch forward)
     * 
     * @note Typical input range: ±0.785 radians (±45 degrees) limited by LOIT_ANG_MAX parameter
     * @note Called at pilot input rate (typically 50-100Hz)
     * @note Internally slews acceleration changes for smooth response
     * @note Units: angles in radians, resulting acceleration in cm/s²
     * 
     * @see set_pilot_desired_acceleration_cd() for centidegree input version
     * @see clear_pilot_desired_acceleration() to zero pilot input
     */
    void set_pilot_desired_acceleration_rad(float euler_roll_angle_rad, float euler_pitch_angle_rad);

    /**
     * @brief Set pilot desired acceleration from stick input (centidegrees)
     * 
     * @details Convenience wrapper that converts centidegree angles to radians and calls
     *          set_pilot_desired_acceleration_rad(). Provides alternative interface for
     *          code using centidegree angle representation.
     * 
     * @param[in] euler_roll_angle_cd   Pilot commanded roll angle in centidegrees (+ = roll right)
     * @param[in] euler_pitch_angle_cd  Pilot commanded pitch angle in centidegrees (+ = pitch forward)
     * 
     * @note Typical input range: ±4500 centidegrees (±45 degrees) limited by LOIT_ANG_MAX
     * @note Units: angles in centidegrees (1 degree = 100 centidegrees)
     * 
     * @see set_pilot_desired_acceleration_rad() for implementation details
     */
    void set_pilot_desired_acceleration_cd(float euler_roll_angle_cd, float euler_pitch_angle_cd);

    /**
     * @brief Get pilot desired acceleration in earth frame
     * 
     * @details Returns the current pilot commanded acceleration in North-East earth frame.
     *          This is the slewed (rate-limited) acceleration value, not the raw pilot input.
     *          Useful for telemetry or coordination with other controllers.
     * 
     * @return Vector2f Desired acceleration in NE frame, cm/s² (x=North, y=East)
     * 
     * @note Returns earth frame (NE) acceleration, not body frame
     * @note Value is slewed/filtered from raw pilot input
     * @note Units: cm/s² (centimeters per second squared)
     * 
     * @see set_pilot_desired_acceleration_rad() to set this value
     */
    Vector2f get_pilot_desired_acceleration_NE_cmss() const { return Vector2f{_desired_accel_ne_cmss.x, _desired_accel_ne_cmss.y}; }

    /**
     * @brief Clear pilot desired acceleration to zero
     * 
     * @details Convenience method to zero pilot commanded acceleration. Equivalent to calling
     *          set_pilot_desired_acceleration_rad(0.0, 0.0). When pilot releases sticks, this
     *          should be called to initiate braking/deceleration sequence.
     * 
     * @note Triggers brake timer if brake delay is configured (LOIT_BRK_DELAY > 0)
     * @note After brake delay, jerk-limited deceleration begins
     * 
     * @see set_pilot_desired_acceleration_rad() for underlying implementation
     */
    void clear_pilot_desired_acceleration() {
        set_pilot_desired_acceleration_rad(0.0, 0.0);
    }

    /**
     * @brief Calculate predicted stopping point based on current position and velocity
     * 
     * @details Computes the horizontal position where vehicle will come to rest given current
     *          velocity and configured braking parameters (acceleration and jerk limits). Uses
     *          kinematic equations with jerk-limited deceleration profile. Useful for planning
     *          maneuvers and predicting loiter behavior.
     *          
     *          Calculation accounts for:
     *          - Current vehicle velocity
     *          - Maximum braking acceleration (LOIT_BRK_ACCEL)
     *          - Maximum jerk during braking (LOIT_BRK_JERK)
     *          - Brake delay time (LOIT_BRK_DELAY)
     * 
     * @param[out] stopping_point  Predicted stopping position in NE frame, cm from EKF origin
     * 
     * @note Position units: centimeters in North-East frame
     * @note Assumes maximum braking performance - actual stopping point may vary
     * @note Does not account for external disturbances (wind, etc.)
     * 
     * @see AC_PosControl::get_stopping_point_xy_cm() for underlying calculation
     */
    void get_stopping_point_NE_cm(Vector2f& stopping_point) const;

    /**
     * @brief Get horizontal distance to loiter target
     * 
     * @details Returns the current position error magnitude - distance between vehicle's current
     *          position and loiter target position in horizontal plane. Useful for monitoring
     *          loiter accuracy and triggering waypoint reached conditions.
     * 
     * @return float Distance to target in centimeters (always positive)
     * 
     * @note Units: centimeters (cm)
     * @note Returns magnitude only, not direction - use get_bearing_to_target_rad() for direction
     * @note Queries AC_PosControl for current position error
     * 
     * @see get_bearing_to_target_rad() for direction to target
     */
    float get_distance_to_target_cm() const { return _pos_control.get_pos_error_NE_cm(); }

    /**
     * @brief Get bearing to loiter target
     * 
     * @details Returns the direction from current vehicle position to loiter target position
     *          as a bearing in earth frame. Bearing is measured clockwise from North.
     *          Useful for navigation displays and monitoring loiter performance.
     * 
     * @return float Bearing to target in radians, clockwise from North (0 to 2π)
     * 
     * @note Units: radians (0 = North, π/2 = East, π = South, 3π/2 = West)
     * @note Queries AC_PosControl for bearing calculation
     * @note Undefined (may return last valid value) if distance to target is zero
     * 
     * @see get_distance_to_target_cm() for distance to target
     */
    float get_bearing_to_target_rad() const { return _pos_control.get_bearing_to_target_rad(); }

    /**
     * @brief Get maximum lean angle for loiter mode (radians)
     * 
     * @details Returns the configured maximum lean angle for pilot input during loiter.
     *          If LOIT_ANG_MAX parameter is set to zero, defaults to 2/3 of the vehicle's
     *          maximum lean angle from attitude controller. This limit constrains pilot
     *          commanded accelerations to prevent aggressive maneuvers during station-keeping.
     * 
     * @return float Maximum lean angle in radians
     * 
     * @note Units: radians (typical range 0.35-0.79 rad / 20-45 degrees)
     * @note If LOIT_ANG_MAX = 0, returns 2/3 of attitude controller's angle_max
     * @note Affects maximum achievable acceleration via gravity projection
     * 
     * @warning Lower values increase position hold accuracy but reduce responsiveness
     * 
     * @see get_angle_max_cd() for centidegree version
     */
    float get_angle_max_rad() const;

    /**
     * @brief Get maximum lean angle for loiter mode (centidegrees)
     * 
     * @details Convenience method returning maximum lean angle in centidegrees instead of radians.
     *          Functionally equivalent to get_angle_max_rad() with unit conversion.
     * 
     * @return float Maximum lean angle in centidegrees
     * 
     * @note Units: centidegrees (1 degree = 100 centidegrees, typical range 2000-4500 cd)
     * 
     * @see get_angle_max_rad() for implementation details
     */
    float get_angle_max_cd() const;

    /**
     * @brief Run the loiter position controller (main update loop)
     * 
     * @details Main loiter controller update function, called at loop rate (typically 100-400Hz).
     *          Performs the complete control sequence:
     *          
     *          1. Sanity check parameters (limits, consistency)
     *          2. Calculate desired velocity from pilot input with fake wind resistance model
     *          3. Apply object avoidance adjustments if enabled (via AC_Avoid integration)
     *          4. Update AC_PosControl with desired position/velocity/acceleration
     *          5. AC_PosControl generates desired roll/pitch angles for attitude controller
     *          
     *          Fake Wind Resistance Model:
     *          Simulates aerodynamic drag proportional to velocity magnitude, requiring continuous
     *          pilot input to maintain acceleration. Provides more natural and intuitive control feel.
     *          
     *          Braking Sequence (when pilot input is zero):
     *          - Wait for brake delay period (LOIT_BRK_DELAY)
     *          - Apply jerk-limited deceleration (LOIT_BRK_JERK limit)
     *          - Limit deceleration magnitude (LOIT_BRK_ACCEL limit)
     *          - Smoothly reduce velocity to zero
     * 
     * @param[in] avoidance_on  Enable object avoidance integration (default: true)
     *                          Set false to disable AC_Avoid adjustments for this update cycle
     * 
     * @note Call frequency: Main loop rate (typically 100-400Hz depending on vehicle)
     * @note Must be called after set_pilot_desired_acceleration to update control
     * @note Generates roll/pitch commands accessible via get_roll/pitch methods
     * @note Integrates with AC_PosControl which must be updated separately
     * 
     * @warning Skipping update calls will cause position drift and loss of control authority
     * 
     * @see set_pilot_desired_acceleration_rad() to provide pilot input
     * @see get_roll_rad() to extract desired roll angle
     * @see get_pitch_rad() to extract desired pitch angle
     * @see calc_desired_velocity() for velocity calculation implementation
     */
    void update(bool avoidance_on = true);

    /**
     * @brief Set maximum horizontal speed limit
     * 
     * @details Configures the maximum horizontal speed allowed during loiter operations.
     *          This limit is applied to the desired velocity calculation and affects how
     *          aggressively the vehicle responds to pilot input. Lower speeds increase
     *          position hold precision but reduce responsiveness to pilot commands.
     * 
     * @param[in] speed_max_NE_cms  Maximum horizontal speed in North-East plane, cm/s
     * 
     * @note Units: cm/s (centimeters per second)
     * @note Typical values: 500-2500 cm/s (5-25 m/s depending on vehicle size)
     * @note Does not affect vertical speed (managed by altitude controller)
     * @note Persistent until changed or soften_for_landing() called
     * 
     * @warning Very low speeds (<100 cm/s) may cause poor pilot control response
     * @warning Very high speeds (>3000 cm/s) may cause position overshoot and oscillation
     * 
     * @see soften_for_landing() which reduces speed limits for landing approach
     */
    void set_speed_max_NE_cms(float speed_max_NE_cms);

    /**
     * @brief Get desired roll angle for attitude controller (radians)
     * 
     * @details Returns the desired roll angle calculated by loiter position controller via
     *          AC_PosControl. This angle should be fed to the attitude controller (stabilize)
     *          to achieve the commanded horizontal position/velocity. Updated by update() call.
     * 
     * @return float Desired roll angle in radians (+ = roll right, - = roll left)
     * 
     * @note Units: radians (typical range ±0.79 rad / ±45 degrees)
     * @note Value updated by update() method - call update() before reading
     * @note Feed this value to AC_AttitudeControl::input_euler_angle_roll_pitch_euler()
     * @note Includes feed-forward prediction for improved response
     * 
     * @see get_pitch_rad() for pitch angle
     * @see get_roll_cd() for centidegree version
     * @see update() to update this value
     */
    float get_roll_rad() const { return _pos_control.get_roll_rad(); }

    /**
     * @brief Get desired pitch angle for attitude controller (radians)
     * 
     * @details Returns the desired pitch angle calculated by loiter position controller via
     *          AC_PosControl. This angle should be fed to the attitude controller (stabilize)
     *          to achieve the commanded horizontal position/velocity. Updated by update() call.
     * 
     * @return float Desired pitch angle in radians (+ = pitch forward, - = pitch back)
     * 
     * @note Units: radians (typical range ±0.79 rad / ±45 degrees)
     * @note Value updated by update() method - call update() before reading
     * @note Feed this value to AC_AttitudeControl::input_euler_angle_roll_pitch_euler()
     * @note Includes feed-forward prediction for improved response
     * 
     * @see get_roll_rad() for roll angle
     * @see get_pitch_cd() for centidegree version
     * @see update() to update this value
     */
    float get_pitch_rad() const { return _pos_control.get_pitch_rad(); }

    /**
     * @brief Get desired roll angle for attitude controller (centidegrees)
     * 
     * @details Convenience method returning desired roll angle in centidegrees instead of radians.
     *          Functionally equivalent to get_roll_rad() with unit conversion.
     * 
     * @return float Desired roll angle in centidegrees (+ = roll right, - = roll left)
     * 
     * @note Units: centidegrees (1 degree = 100 centidegrees, typical range ±4500 cd)
     * 
     * @see get_roll_rad() for implementation details
     */
    float get_roll_cd() const { return _pos_control.get_roll_cd(); }

    /**
     * @brief Get desired pitch angle for attitude controller (centidegrees)
     * 
     * @details Convenience method returning desired pitch angle in centidegrees instead of radians.
     *          Functionally equivalent to get_pitch_rad() with unit conversion.
     * 
     * @return float Desired pitch angle in centidegrees (+ = pitch forward, - = pitch back)
     * 
     * @note Units: centidegrees (1 degree = 100 centidegrees, typical range ±4500 cd)
     * 
     * @see get_pitch_rad() for implementation details
     */
    float get_pitch_cd() const { return _pos_control.get_pitch_cd(); }

    /**
     * @brief Get desired thrust vector for attitude controller
     * 
     * @details Returns the desired thrust vector (including roll, pitch, and vertical components)
     *          calculated by AC_PosControl. The thrust vector represents both attitude (roll/pitch)
     *          and throttle (vertical) commands. Alternative to using separate roll/pitch/throttle
     *          accessors, particularly useful for thrust vector control schemes.
     * 
     * @return Vector3f Desired thrust vector (body frame: x=forward, y=right, z=down)
     *                  Components normalized or in appropriate units for attitude controller
     * 
     * @note Vector in body frame following NED convention
     * @note Updated by update() method - call update() before reading
     * @note Contains both horizontal (roll/pitch) and vertical (throttle) components
     * 
     * @see get_roll_rad() for roll-only accessor
     * @see get_pitch_rad() for pitch-only accessor
     * @see AC_PosControl::get_thrust_vector() for implementation details
     */
    Vector3f get_thrust_vector() const { return _pos_control.get_thrust_vector(); }

    /**
     * @brief Parameter table for loiter controller configuration
     * 
     * @details Defines AP_Param group for loiter parameters, enabling parameter storage in
     *          EEPROM and access via ground control station. Parameters are loaded at boot
     *          and can be modified during flight.
     *          
     *          Defined Parameters:
     *          - LOIT_ANG_MAX: Maximum pilot commanded lean angle (degrees)
     *          - LOIT_SPEED: Maximum horizontal speed during loiter (cm/s)
     *          - LOIT_ACC_MAX: Maximum horizontal acceleration (cm/s²)
     *          - LOIT_BRK_ACCEL: Maximum braking acceleration (cm/s²)
     *          - LOIT_BRK_JERK: Maximum jerk during braking (cm/s³)
     *          - LOIT_BRK_DELAY: Delay before braking starts (seconds)
     * 
     * @note Parameters accessible via MAVLink parameter protocol
     * @note Changes persist across reboots when saved to EEPROM
     * 
     * @see AP_Param documentation for parameter system details
     */
    static const struct AP_Param::GroupInfo var_info[];

protected:

    /**
     * @brief Sanity check and limit loiter parameters
     * 
     * @details Validates parameter values and applies sensible limits to prevent invalid
     *          configurations. Called by update() before control calculations. Ensures
     *          speed, acceleration, and jerk limits are positive and within reasonable ranges.
     * 
     * @note Called automatically by update() - not intended for external use
     * @note Clamps parameters to safe ranges without modifying stored values
     */
    void sanity_check_params();

    /**
     * @brief Calculate desired velocity with pilot input and fake wind resistance
     * 
     * @details Updates desired velocity by integrating pilot commanded acceleration and applying
     *          fake wind resistance model. The fake wind resistance simulates aerodynamic drag
     *          proportional to velocity, requiring continuous pilot input to maintain speed.
     *          This provides more natural stick feel compared to pure acceleration control.
     *          
     *          Algorithm Steps:
     *          1. Integrate pilot desired acceleration to velocity
     *          2. Apply fake wind resistance (drag proportional to velocity)
     *          3. Limit velocity to maximum speed (LOIT_SPEED parameter)
     *          4. Apply object avoidance adjustments if enabled
     *          5. Send updated velocity to AC_PosControl
     *          
     *          Braking Logic (when pilot input is zero):
     *          - If brake delay elapsed, apply jerk-limited deceleration
     *          - Ramp brake acceleration up to LOIT_BRK_ACCEL limit
     *          - Limit jerk rate to LOIT_BRK_JERK for smooth deceleration
     * 
     * @param[in] avoidance_on  Enable object avoidance velocity adjustments (default: true)
     * 
     * @note Called by update() method - not intended for direct external use
     * @note Modifies internal velocity state and sends to AC_PosControl
     * @note Integrates with AC_Avoid for object avoidance when enabled
     */
    void calc_desired_velocity(bool avoidance_on = true);

    /**
     * @name External System References
     * @brief References to required subsystems for loiter control
     * @{
     */
    
    /** @brief Reference to AHRS for attitude and position estimates */
    const AP_AHRS_View&     _ahrs;
    
    /** @brief Reference to position controller for converting velocity to attitude commands */
    AC_PosControl&          _pos_control;
    
    /** @brief Reference to attitude controller for predicting attitude response */
    const AC_AttitudeControl& _attitude_control;
    
    /** @} */

    /**
     * @name Loiter Configuration Parameters
     * @brief AP_Param parameters for loiter controller tuning
     * @{
     */
    
    /**
     * @brief Maximum pilot commanded lean angle in degrees
     * @note If set to zero, defaults to 2/3 of attitude controller's maximum angle
     * @note Typical range: 20-45 degrees
     * @note Parameter: LOIT_ANG_MAX
     */
    AP_Float    _angle_max_deg;
    
    /**
     * @brief Maximum horizontal speed during loiter in cm/s
     * @note Typical range: 500-2500 cm/s (5-25 m/s)
     * @note Parameter: LOIT_SPEED
     */
    AP_Float    _speed_max_ne_cms;
    
    /**
     * @brief Maximum horizontal acceleration in cm/s²
     * @note Typical range: 100-1000 cm/s²
     * @note Parameter: LOIT_ACC_MAX
     */
    AP_Float    _accel_max_ne_cmss;
    
    /**
     * @brief Maximum acceleration during braking in cm/s²
     * @note Should be >= _accel_max_ne_cmss for effective braking
     * @note Typical range: 250-1500 cm/s²
     * @note Parameter: LOIT_BRK_ACCEL
     */
    AP_Float    _brake_accel_max_cmss;
    
    /**
     * @brief Maximum jerk during braking in cm/s³
     * @note Controls smoothness of braking deceleration ramp
     * @note Typical range: 500-5000 cm/s³
     * @note Parameter: LOIT_BRK_JERK
     */
    AP_Float    _brake_jerk_max_cmsss;
    
    /**
     * @brief Delay before braking begins after sticks released, in seconds
     * @note Typical range: 0-1.0 seconds
     * @note Zero delay = immediate braking
     * @note Parameter: LOIT_BRK_DELAY
     */
    AP_Float    _brake_delay_s;
    
    /** @} */

    /**
     * @name Loiter Controller Internal State
     * @brief Internal variables tracking controller state and predictions
     * @{
     */
    
    /**
     * @brief Slewed pilot desired acceleration in NE frame, cm/s²
     * @note Rate-limited from raw pilot input for smooth response
     */
    Vector2f    _desired_accel_ne_cmss;
    
    /**
     * @brief Predicted acceleration in NE frame based on pilot input, cm/s²
     * @note Used for feed-forward control to anticipate vehicle response
     */
    Vector2f    _predicted_accel_ne_cmss;
    
    /**
     * @brief Predicted roll/pitch angles in radians from input shaping
     * @note Anticipates attitude response before actual vehicle movement
     * @note x = predicted roll, y = predicted pitch
     */
    Vector2f    _predicted_euler_angle_rad;
    
    /**
     * @brief Predicted roll/pitch rates in rad/s from input shaping
     * @note Anticipates attitude rate response for feed-forward control
     * @note x = predicted roll rate, y = predicted pitch rate
     */
    Vector2f    _predicted_euler_rate;
    
    /**
     * @brief System time (milliseconds) when braking was initiated
     * @note Used to implement brake delay (LOIT_BRK_DELAY)
     * @note Zero indicates no active braking
     */
    uint32_t    _brake_timer_ms;
    
    /**
     * @brief Braking acceleration from previous iteration, cm/s²
     * @note Used for jerk limiting during braking deceleration ramp
     * @note Gradually increased up to LOIT_BRK_ACCEL limit
     */
    float       _brake_accel_cmss;
    
    /** @} */
};
