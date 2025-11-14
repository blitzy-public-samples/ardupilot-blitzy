/**
 * @file AR_AttitudeControl.h
 * @brief Attitude and speed controller for ground and marine vehicles
 * 
 * @details This file implements the attitude (steering) and speed (throttle) controller
 *          for Rover and Sub vehicles. It provides high-level control interfaces that
 *          convert desired lateral acceleration, heading, turn rate, or speed commands
 *          into normalized servo outputs for steering and throttle channels.
 *          
 *          The controller supports multiple operating modes:
 *          - Steering modes: lateral acceleration control, heading control, rate control
 *          - Throttle modes: speed control, controlled stop, pitch-based throttle
 *          - Special vehicle modes: balancebot pitch stability, sailboat heel control
 *          
 *          Internal architecture uses cascaded controllers:
 *          - Steering: AC_P for angle→rate, AC_PID for rate→output
 *          - Throttle: AC_PID for speed→throttle, special PIDs for pitch and heel
 *          
 *          Integration points:
 *          - Reads attitude and velocity from AP_AHRS
 *          - Writes servo outputs via SRV_Channels
 *          - Logs control data via AP_Logger
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

#include <AP_Common/AP_Common.h>
#include <AC_PID/AC_PID.h>
#include <AC_PID/AC_P.h>

/**
 * @class AR_AttitudeControl
 * @brief Ground and marine vehicle attitude and speed controller singleton
 * 
 * @details This class provides comprehensive steering and throttle control for ground
 *          and marine vehicles (Rover, Sub). It accepts high-level commands (desired
 *          lateral acceleration, heading, turn rate, or speed) and produces normalized
 *          servo outputs [-1.0, 1.0] for steering and throttle channels.
 *          
 *          **Steering Control Architecture:**
 *          - Lateral acceleration mode: desired_accel → turn_rate (kinematic) → steering_output
 *          - Heading mode: heading_error → turn_rate (P controller) → steering_output
 *          - Rate mode: desired_rate → steering_output (rate PID)
 *          
 *          **Throttle Control Architecture:**
 *          - Speed mode: speed_error → throttle_output (speed PID)
 *          - Stop mode: controlled deceleration to zero with stop detection
 *          - Pitch mode: pitch_error → throttle_adjustment (for slope climbing)
 *          
 *          **Internal Controllers:**
 *          - _steer_angle_p: Proportional controller for heading → turn rate
 *          - _steer_rate_pid: PID controller for turn rate → steering output
 *          - _throttle_speed_pid: PID controller for speed → throttle output
 *          - _pitch_to_throttle_pid: PID for pitch control (balancebot, slope)
 *          - _sailboat_heel_pid: PID for heel angle control (sailboat)
 *          
 *          **Special Vehicle Support:**
 *          - Balancebot: Maintains upright balance via pitch → throttle feedback
 *          - Sailboat: Controls sail release to limit heel angle during turns
 *          
 *          **Integration with ArduPilot Systems:**
 *          - AP_AHRS: Source of attitude (roll, pitch, yaw) and velocity
 *          - SRV_Channels: Target for servo output commands
 *          - AP_Logger: Logging destination for control telemetry
 *          - AP_Param: Parameter storage and retrieval
 *          
 *          **Coordinate Frames:**
 *          - Body frame: X-axis forward, Y-axis right, Z-axis down
 *          - Positive yaw rate = clockwise from above (right turn)
 *          - Earth frame: NE horizontal plane for velocity measurements
 *          
 *          **Units and Conventions:**
 *          - Steering/throttle outputs: normalized [-1.0, 1.0], can exceed for large errors
 *          - Lateral acceleration: m/s/s (positive = right turn)
 *          - Turn rates: rad/s (positive = right turn)
 *          - Speeds: m/s (negative = reverse)
 *          - Angles: radians unless otherwise noted
 *          - Acceleration limits: m/s/s
 *          
 *          **Integrator Anti-Windup:**
 *          - Integrators stop accumulating when motor_limit flags are true
 *          - Call relax_I() when switching to manual mode to zero integrators
 *          
 * @warning Tuning parameters (P gains, PID coefficients, acceleration limits) directly
 *          affect vehicle stability and handling. Improper tuning can cause oscillations,
 *          instability, or inability to follow commanded paths accurately.
 * 
 * @warning motor_limit_left, motor_limit_right, motor_limit_low, motor_limit_high flags
 *          should accurately reflect actual motor saturation state. Incorrect limit flags
 *          can cause integrator wind-up and poor tracking performance.
 * 
 * @note This is a singleton class - access via get_singleton()
 * @note Thread-safety: Methods should be called from main scheduler task only
 * @note Typical call frequency: Main loop rate (50-400 Hz depending on vehicle)
 */
class AR_AttitudeControl {
public:

    /**
     * @brief Constructor for AR_AttitudeControl
     * 
     * @details Initializes the attitude and speed controller, sets up internal PID
     *          controllers, and registers parameters. Called once during vehicle
     *          initialization.
     * 
     * @note Sets up the singleton instance pointer
     * @note Initializes all PID controllers with default gains
     */
    AR_AttitudeControl();

    // do not allow copying
    CLASS_NO_COPY(AR_AttitudeControl);

    /**
     * @brief Access the singleton instance
     * 
     * @return Pointer to the AR_AttitudeControl singleton instance
     * 
     * @note Returns nullptr if not yet constructed
     * @note Singleton created during vehicle initialization
     */
    static AR_AttitudeControl *get_singleton() { return _singleton; }

    //
    // steering controller
    //

    /**
     * @brief Calculate steering output from desired lateral acceleration
     * 
     * @details Converts desired lateral acceleration into steering servo output via:
     *          1. Kinematic conversion: lat_accel → turn_rate using current speed
     *          2. Rate control: turn_rate → steering_output using rate PID
     *          
     *          Algorithm applies acceleration limiting and integrator anti-windup based
     *          on motor limit flags. Stores desired lateral acceleration for reporting.
     *          
     *          Coordinate frame: Lateral acceleration positive = right turn
     * 
     * @param[in] desired_accel Desired lateral acceleration in m/s/s (positive = right turn)
     * @param[in] motor_limit_left True if left motor/servo has reached physical limit
     * @param[in] motor_limit_right True if right motor/servo has reached physical limit
     * @param[in] dt Time step in seconds (typically main loop period: 0.0025 to 0.02 s)
     * 
     * @return Normalized steering output in range [-1.0, 1.0] for typical operation.
     *         Can exceed ±1.0 if large tracking errors exist and limits not reached.
     * 
     * @note Call at consistent rate for proper integrator behavior
     * @note Stores desired_accel in _desired_lat_accel for get_desired_lat_accel()
     * @note Updates _steering_limit_left and _steering_limit_right flags
     * @note Lateral acceleration limited by get_turn_lat_accel_max()
     * 
     * @warning Integrator winds up if motor_limit flags incorrect
     * @warning Output can exceed ±1.0 - calling code must clamp if needed
     * 
     * @see get_steering_out_rate()
     * @see get_turn_rate_from_lat_accel()
     * @see get_turn_lat_accel_max()
     */
    float get_steering_out_lat_accel(float desired_accel, bool motor_limit_left, bool motor_limit_right, float dt);

    /**
     * @brief Calculate steering output from desired heading
     * 
     * @details Implements heading control via cascaded angle and rate controllers:
     *          1. Heading error → turn rate via _steer_angle_p (proportional controller)
     *          2. Turn rate → steering output via _steer_rate_pid (rate controller)
     *          
     *          Internally calls get_turn_rate_from_heading() then get_steering_out_rate().
     *          Applies rate limiting if rate_max_rads specified.
     * 
     * @param[in] heading_rad Desired heading in radians (earth frame, 0 = north)
     * @param[in] rate_max_rads Maximum turn rate limit in rad/s (0 = no limit)
     * @param[in] motor_limit_left True if left motor/servo saturated
     * @param[in] motor_limit_right True if right motor/servo saturated
     * @param[in] dt Time step in seconds
     * 
     * @return Normalized steering output [-1.0, 1.0] typical, can exceed for large errors
     * 
     * @note Heading error automatically wrapped to [-π, π]
     * @note Turn rate limited by rate_max_rads AND _steer_rate_max parameter
     * @note Updates _desired_turn_rate for reporting
     * 
     * @see get_turn_rate_from_heading()
     * @see get_steering_out_rate()
     */
    float get_steering_out_heading(float heading_rad, float rate_max_rads, bool motor_limit_left, bool motor_limit_right, float dt);

    /**
     * @brief Compute desired turn rate from heading error
     * 
     * @details Applies proportional control (P) to convert heading error to turn rate:
     *          turn_rate = heading_error * _steer_angle_p.kP()
     *          
     *          Heading error is wrapped to [-π, π] for shortest turn direction.
     *          Result is rate-limited if rate_max_rads > 0.
     * 
     * @param[in] heading_rad Desired heading in radians (earth frame)
     * @param[in] rate_max_rads Maximum turn rate in rad/s (0 = no limit)
     * 
     * @return Desired turn rate in rad/s (positive = right turn, negative = left turn)
     * 
     * @note Typically called internally by get_steering_out_heading()
     * @note Can be called separately for feed-forward or rate planning
     * @note Does not update internal state - pure calculation
     * 
     * @see get_steering_out_heading()
     */
    float get_turn_rate_from_heading(float heading_rad, float rate_max_rads) const;

    /**
     * @brief Calculate steering output from desired turn rate
     * 
     * @details Implements rate controller using _steer_rate_pid:
     *          - Computes rate error from current yaw rate (from AHRS)
     *          - Applies PID control: output = P*error + I*integral + D*derivative
     *          - Updates integrator with anti-windup based on motor limits
     *          - Applies acceleration/deceleration limits to rate command
     *          
     *          Sets _steering_limit_left/_steering_limit_right flags if limits reached.
     * 
     * @param[in] desired_rate Desired turn rate in rad/s (positive = right turn)
     * @param[in] motor_limit_left True if left motor saturated (prevents integrator windup)
     * @param[in] motor_limit_right True if right motor saturated
     * @param[in] dt Time step in seconds
     * 
     * @return Normalized steering output [-1.0, 1.0] typical, can exceed for large errors
     * 
     * @note This is the lowest-level steering control method
     * @note Rate limited by _steer_rate_max parameter
     * @note Acceleration limited by _steer_accel_max and _steer_decel_max
     * @note Updates _desired_turn_rate and steering limit flags
     * @note Called at main loop rate for proper derivative and integrator behavior
     * 
     * @warning Incorrect motor_limit flags cause integrator wind-up
     * @warning Derivative term sensitive to noise - ensure smooth rate signal
     * 
     * @see steering_limit_left()
     * @see steering_limit_right()
     * @see get_desired_turn_rate()
     */
    float get_steering_out_rate(float desired_rate, bool motor_limit_left, bool motor_limit_right, float dt);

    /**
     * @brief Get latest commanded turn rate
     * 
     * @details Returns the most recent desired turn rate value recorded during calls
     *          to get_steering_out_rate() or get_steering_out_heading().
     * 
     * @return Desired turn rate in rad/s (positive = right turn)
     * 
     * @note For reporting and logging purposes only
     * @note Updated by steering control methods
     */
    float get_desired_turn_rate() const;

    /**
     * @brief Get latest commanded lateral acceleration
     * 
     * @details Returns the most recent desired lateral acceleration recorded during
     *          calls to get_steering_out_lat_accel().
     * 
     * @return Desired lateral acceleration in m/s/s (positive = right turn)
     * 
     * @note For reporting and logging purposes only
     * @note Only updated by get_steering_out_lat_accel()
     */
    float get_desired_lat_accel() const;

    /**
     * @brief Get actual measured lateral acceleration
     * 
     * @details Calculates current lateral acceleration from vehicle velocity and yaw rate
     *          using kinematic relationship: lat_accel = velocity * yaw_rate
     * 
     * @param[out] lat_accel Measured lateral acceleration in m/s/s (positive = right)
     * 
     * @return true if calculation successful, false if velocity unavailable
     * 
     * @note For reporting and telemetry purposes
     * @note Requires valid velocity estimate from AHRS/EKF
     */
    bool get_lat_accel(float &lat_accel) const;

    /**
     * @brief Convert lateral acceleration to turn rate
     * 
     * @details Kinematic conversion using centripetal acceleration formula:
     *          turn_rate = lat_accel / speed
     *          
     *          Returns zero if speed is below minimum threshold to avoid division by zero.
     * 
     * @param[in] lat_accel Lateral acceleration in m/s/s
     * @param[in] speed Vehicle forward speed in m/s
     * 
     * @return Turn rate in rad/s
     * 
     * @note Pure calculation, does not modify state
     * @note Returns 0 if speed < 0.1 m/s to avoid numerical issues
     */
    float get_turn_rate_from_lat_accel(float lat_accel, float speed) const;

    /**
     * @brief Get maximum lateral acceleration limit
     * 
     * @details Returns configured lateral acceleration limit from _turn_lateral_G_max
     *          parameter, with minimum enforced value of 0.1G (≈ 1 m/s/s).
     * 
     * @return Maximum lateral acceleration in m/s/s
     * 
     * @note Minimum of 0.1G prevents excessive turn rates on low-traction surfaces
     * @note Used by lateral acceleration controller for command limiting
     */
    float get_turn_lat_accel_max() const { return MAX(_turn_lateral_G_max, 0.1f) * GRAVITY_MSS; }

    /**
     * @brief Check if steering limited on left side
     * 
     * @details Returns true if steering output has reached limits due to:
     *          - Physical servo/motor limits (motor_limit_left flag)
     *          - Turn rate limits (_steer_rate_max)
     *          - Acceleration limits (_steer_accel_max)
     * 
     * @return true if left steering limited, false otherwise
     * 
     * @note Updated by get_steering_out_rate()
     * @note Used for anti-windup and path planning
     */
    bool steering_limit_left() const { return _steering_limit_left; }

    /**
     * @brief Check if steering limited on right side
     * 
     * @details Returns true if steering output has reached limits due to:
     *          - Physical servo/motor limits (motor_limit_right flag)
     *          - Turn rate limits (_steer_rate_max)
     *          - Acceleration limits (_steer_accel_max)
     * 
     * @return true if right steering limited, false otherwise
     * 
     * @note Updated by get_steering_out_rate()
     * @note Used for anti-windup and path planning
     */
    bool steering_limit_right() const { return _steering_limit_right; }

    //
    // throttle / speed controller
    //

    /**
     * @brief Configure throttle controller acceleration limits
     * 
     * @details Sets maximum forward acceleration and braking deceleration limits used by
     *          speed controller to limit rate of speed change. These limits are applied
     *          to desired speed before PID controller processes the command.
     * 
     * @param[in] throttle_accel_max Maximum forward acceleration in m/s/s (positive)
     * @param[in] throttle_decel_max Maximum deceleration (braking) in m/s/s (positive)
     * 
     * @note Should be called during initialization or when parameters change
     * @note Limits prevent wheel slip and maintain vehicle stability
     * @note Set to 0 to disable acceleration limiting
     */
    void set_throttle_limits(float throttle_accel_max, float throttle_decel_max);

    /**
     * @brief Calculate throttle output from desired speed
     * 
     * @details Implements speed controller using _throttle_speed_pid:
     *          1. Computes speed error: desired_speed - actual_speed
     *          2. Applies PID control: throttle = P*error + I*integral + D*derivative + FF
     *          3. Feed-forward term computed from cruise_speed/cruise_throttle
     *          4. Integrator anti-windup based on motor_limit flags
     *          
     *          Algorithm records _desired_speed for reporting and applies brake control
     *          if configured via _brake_enable parameter.
     * 
     * @param[in] desired_speed Desired speed in m/s (negative = reverse, must be acceleration-limited)
     * @param[in] motor_limit_low True if motors saturated at low end (prevents integrator windup)
     * @param[in] motor_limit_high True if motors saturated at high end
     * @param[in] cruise_speed Nominal cruise speed in m/s for feed-forward calculation
     * @param[in] cruise_throttle Nominal cruise throttle [-1,1] for feed-forward
     * @param[in] dt Time step in seconds
     * 
     * @return Normalized throttle output [-1.0, 1.0] (negative = reverse)
     * 
     * @note desired_speed should be pre-filtered with get_desired_speed_accel_limited()
     * @note cruise_speed and cruise_throttle provide feed-forward for better tracking
     * @note Brake control (reverse throttle) applied if speed > desired and _brake_enable set
     * @note Updates _desired_speed and _throttle_speed_pid_info for reporting
     * @note Call at consistent rate (main loop) for proper PID behavior
     * 
     * @warning Incorrect motor_limit flags cause integrator wind-up
     * @warning Feed-forward requires accurate cruise_speed and cruise_throttle
     * 
     * @see get_desired_speed_accel_limited()
     * @see speed_control_active()
     * @see get_desired_speed()
     */
    float get_throttle_out_speed(float desired_speed, bool motor_limit_low, bool motor_limit_high, float cruise_speed, float cruise_throttle, float dt);

    /**
     * @brief Calculate throttle for controlled stop
     * 
     * @details Implements smooth deceleration to full stop with stop detection:
     *          1. Applies braking throttle to reduce speed
     *          2. Monitors actual speed against _stop_speed threshold
     *          3. Sets stopped flag and zeros throttle when stopped
     *          4. Uses speed PID controller with zero target speed
     * 
     * @param[in] motor_limit_low True if motors saturated at low end
     * @param[in] motor_limit_high True if motors saturated at high end
     * @param[in] cruise_speed Nominal cruise speed for feed-forward
     * @param[in] cruise_throttle Nominal cruise throttle for feed-forward
     * @param[in] dt Time step in seconds
     * @param[out] stopped Set to true once vehicle speed falls below _stop_speed threshold
     * 
     * @return Normalized throttle output for controlled braking (typically negative)
     * 
     * @note stopped flag set when speed < _stop_speed for at least 100ms
     * @note Once stopped, throttle returns 0 and integrators held at zero
     * @note Brake control applied if _brake_enable parameter set
     * 
     * @see get_stop_speed()
     * @see get_throttle_out_speed()
     */
    float get_throttle_out_stop(bool motor_limit_low, bool motor_limit_high, float cruise_speed, float cruise_throttle, float dt, bool &stopped);

    /**
     * @brief Calculate throttle from pitch angle for slope climbing or balancebot
     * 
     * @details Implements pitch-based throttle control for two use cases:
     *          1. Slope climbing: Adjust throttle to maintain pitch on inclines
     *          2. Balancebot: Maintain upright balance via pitch feedback
     *          
     *          Algorithm:
     *          - Computes pitch error from current pitch (AHRS)
     *          - Applies PID control via _pitch_to_throttle_pid
     *          - Adds feed-forward term: _pitch_to_throttle_ff * current_pitch
     *          - Implements pitch limit protection to prevent tipping
     *          
     *          Pitch limit protection:
     *          - Adaptively narrows pitch limits when throttle near saturation
     *          - Time constant controlled by _pitch_limit_tc
     *          - Prevents integrator windup from causing tip-over
     * 
     * @param[in] desired_pitch Desired pitch angle in radians (positive = nose up)
     * @param[in] pitch_max Maximum allowed pitch angle in radians (safety limit)
     * @param[in] motor_limit True if motors saturated (activates pitch protection)
     * @param[in] dt Time step in seconds
     * 
     * @return Normalized throttle adjustment [-1.0, 1.0]
     * 
     * @note Typically added to base throttle from speed controller
     * @note Updates _pitch_limited flag if protection active
     * @note For balancebot: This is primary stability control
     * @note For slope climbing: Compensates for gravity component
     * @note Feed-forward term improves response to pitch disturbances
     * 
     * @warning pitch_max must be set appropriately for vehicle - too large risks tip-over
     * @warning Requires well-tuned PID gains - instability can cause vehicle to fall
     * @warning Call at consistent high rate (>50Hz) for balancebot stability
     * 
     * @see pitch_limited()
     * @see get_desired_pitch()
     */
    float get_throttle_out_from_pitch(float desired_pitch, float pitch_max, bool motor_limit, float dt);

    /**
     * @brief Check if pitch control has limited pitch angle
     * 
     * @details Returns true if pitch limit protection has constrained the desired pitch
     *          to prevent vehicle tip-over. This occurs when motors near saturation and
     *          pitch error is large.
     * 
     * @return true if pitch was limited on last call to get_throttle_out_from_pitch()
     * 
     * @note For monitoring balancebot stability margin
     * @note If frequently true, increase pitch controller gains or reduce pitch_max
     */
    bool pitch_limited() const { return _pitch_limited; }

    /**
     * @brief Get latest commanded pitch angle
     * 
     * @details Returns most recent desired pitch recorded during calls to
     *          get_throttle_out_from_pitch().
     * 
     * @return Desired pitch in radians (positive = nose up)
     * 
     * @note For reporting and logging purposes
     * @note Only updated by pitch control methods
     */
    float get_desired_pitch() const;

    /**
     * @brief Calculate sail release from heel angle for sailboat
     * 
     * @details Implements heel angle controller for sailboats to limit heel during turns:
     *          - Measures current heel (roll) angle from AHRS
     *          - Computes heel error: current_heel - desired_heel
     *          - Applies PID control via _sailboat_heel_pid
     *          - Output controls sail release: larger output = more sail released
     *          
     *          Purpose: Prevent excessive heel in strong winds by releasing sail tension.
     *          The controller attempts to maintain heel angle at or below desired_heel.
     * 
     * @param[in] desired_heel Maximum desired heel angle in radians (positive = starboard down)
     * @param[in] dt Time step in seconds
     * 
     * @return Sail release amount [0.0, 1.0] where 1.0 = fully released
     * 
     * @note Sailboat-specific control mode
     * @note Heel angle positive = lean to starboard (right)
     * @note Output typically controls sail winch or sheet tension
     * @note Zero desired_heel attempts to keep vessel upright
     * 
     * @warning Requires properly configured heel PID gains
     * @warning Sudden sail release can cause rapid attitude changes
     */
    float get_sail_out_from_heel(float desired_heel, float dt);

    /**
     * @brief Access steering angle P controller
     * 
     * @details Returns reference to heading→turn_rate proportional controller for
     *          parameter tuning and logging access.
     * 
     * @return Reference to AC_P controller
     * 
     * @note For tuning interface and logging system access
     * @note Modify P gain via controller's set_kP() method
     */
    AC_P& get_steering_angle_p() { return _steer_angle_p; }

    /**
     * @brief Access steering rate PID controller
     * 
     * @details Returns reference to turn_rate→steering_output PID controller for
     *          parameter tuning and logging access.
     * 
     * @return Reference to AC_PID controller
     * 
     * @note For tuning interface and logging system access
     * @note Primary steering control PID - tune with caution
     */
    AC_PID& get_steering_rate_pid() { return _steer_rate_pid; }

    /**
     * @brief Access pitch to throttle PID controller
     * 
     * @details Returns reference to pitch→throttle PID used for balancebot and
     *          slope climbing for parameter tuning and logging.
     * 
     * @return Reference to AC_PID controller
     * 
     * @note Critical for balancebot stability
     * @note For tuning interface and logging system access
     */
    AC_PID& get_pitch_to_throttle_pid() { return _pitch_to_throttle_pid; }

    /**
     * @brief Access sailboat heel PID controller
     * 
     * @details Returns reference to heel angle→sail release PID for sailboat
     *          heel control tuning and logging.
     * 
     * @return Reference to AC_PID controller
     * 
     * @note Sailboat-specific controller
     * @note For tuning interface and logging system access
     */
    AC_PID& get_sailboat_heel_pid() { return _sailboat_heel_pid; }

    /**
     * @brief Access throttle speed PID information
     * 
     * @details Returns read-only PID info structure with P, I, D, FF terms and other
     *          diagnostic data from most recent speed controller update.
     * 
     * @return Const reference to AP_PIDInfo structure
     * 
     * @note For logging and telemetry
     * @note Updated during get_throttle_out_speed() calls
     * @note Contains target, actual, error, P/I/D/FF terms
     */
    const AP_PIDInfo& get_throttle_speed_pid_info() const { return _throttle_speed_pid_info; }

    /**
     * @brief Configure PID notch filter sample rate
     * 
     * @details Sets sample rate for notch filters in PID controllers to reject specific
     *          frequencies (motor vibrations, mechanical resonances). Propagates sample
     *          rate to all internal PID controllers.
     * 
     * @param[in] sample_rate Sample rate in Hz (typically main loop frequency)
     * 
     * @note Call during initialization or when loop rate changes
     * @note Affects notch filter center frequency calculation
     * @note Must match actual controller call frequency for proper filtering
     */
    void set_notch_sample_rate(float sample_rate);

    /**
     * @brief Get slew rates for oscillation detection
     * 
     * @details Returns rate of change (slew rate) for steering and speed outputs,
     *          useful for detecting oscillations or instability in Lua scripts or
     *          external monitoring systems.
     * 
     * @param[out] steering_srate Steering output slew rate in units/s
     * @param[out] speed_srate Speed output slew rate in units/s
     * 
     * @note For oscillation detection algorithms
     * @note High slew rates may indicate instability or poor tuning
     * @note Primarily used by Lua scripting interface
     */
    void get_srate(float &steering_srate, float &speed_srate);

    /**
     * @brief Get vehicle forward speed from navigation system
     * 
     * @details Retrieves forward speed (along vehicle X-axis) from EKF/AHRS. This is
     *          earth-frame horizontal velocity projected onto vehicle longitudinal axis.
     *          Used by speed controller as process variable.
     * 
     * @param[out] speed Forward speed in m/s (positive = forward, negative = reverse)
     * 
     * @return true if speed available from AHRS, false if no valid velocity estimate
     * 
     * @note Requires valid EKF velocity estimate
     * @note Returns body-frame forward velocity component
     * @note Used internally by all speed control methods
     */
    bool get_forward_speed(float &speed) const;

    /**
     * @brief Get maximum forward acceleration limit
     * 
     * @details Returns configured maximum forward acceleration from _throttle_accel_max
     *          parameter, used to limit rate of speed increase.
     * 
     * @return Maximum acceleration in m/s/s (always >= 0)
     * 
     * @note Also used for deceleration if _throttle_decel_max not set
     * @note Zero disables acceleration limiting
     */
    float get_accel_max() const { return MAX(_throttle_accel_max, 0.0f); }

    /**
     * @brief Get maximum deceleration (braking) limit
     * 
     * @details Returns maximum deceleration from _throttle_decel_max parameter, or
     *          falls back to _throttle_accel_max if decel not separately configured.
     * 
     * @return Maximum deceleration in m/s/s (positive value)
     * 
     * @note Separate decel limit allows asymmetric acceleration profiles
     * @note Used by controlled stop and speed reduction maneuvers
     */
    float get_decel_max() const;

    /**
     * @brief Check if speed controller is active
     * 
     * @details Returns true if speed controller has been called recently and is
     *          actively controlling throttle.
     * 
     * @return true if speed control running, false otherwise
     * 
     * @note For mode logic and failsafe systems
     * @note Based on time since last get_throttle_out_speed() call
     */
    bool speed_control_active() const;

    /**
     * @brief Get latest commanded speed
     * 
     * @details Returns most recent desired speed recorded during calls to
     *          get_throttle_out_speed().
     * 
     * @return Desired speed in m/s (negative = reverse)
     * 
     * @note For reporting and logging purposes
     * @note Updated by speed controller methods
     */
    float get_desired_speed() const;

    /**
     * @brief Apply acceleration limiting to desired speed
     * 
     * @details Limits rate of change of desired speed based on _throttle_accel_max and
     *          _throttle_decel_max parameters:
     *          - If speeding up: limit by _throttle_accel_max
     *          - If slowing down: limit by _throttle_decel_max
     *          
     *          Should be called before passing desired_speed to get_throttle_out_speed().
     * 
     * @param[in] desired_speed Target speed in m/s (negative = reverse)
     * @param[in] dt Time step in seconds
     * 
     * @return Acceleration-limited speed in m/s
     * 
     * @note Prevents wheel slip and maintains stability
     * @note Uses current _desired_speed as starting point
     * @note Returns input if acceleration limiting disabled (accel_max = 0)
     * 
     * @see set_throttle_limits()
     */
    float get_desired_speed_accel_limited(float desired_speed, float dt) const;

    /**
     * @brief Calculate stopping distance for given speed
     * 
     * @details Computes minimum distance required to stop from specified speed using
     *          maximum deceleration limit:
     *          stopping_distance = speed² / (2 * decel_max)
     * 
     * @param[in] speed Current speed in m/s (positive)
     * 
     * @return Stopping distance in meters
     * 
     * @note Assumes constant maximum deceleration
     * @note Used for obstacle avoidance and path planning
     * @note Does not account for reaction time or brake lag
     * @note Returns large value if decel_max not configured
     */
    float get_stopping_distance(float speed) const;

    /**
     * @brief Get speed threshold for "stopped" state
     * 
     * @details Returns configured speed threshold from _stop_speed parameter below which
     *          vehicle is considered fully stopped. Used by stop controller.
     * 
     * @return Stop speed threshold in m/s (always >= 0)
     * 
     * @note Typical values: 0.1 - 0.3 m/s
     * @note Prevents hunting around zero speed
     */
    float get_stop_speed() const { return MAX(_stop_speed, 0.0f); }

    /**
     * @brief Zero all integrator terms
     * 
     * @details Resets integrator terms in all PID controllers (steering rate, throttle
     *          speed, pitch, heel) to zero. Called when switching to manual mode or when
     *          control authority lost to prevent integrator wind-up.
     * 
     * @note Call when disabling attitude/speed control
     * @note Call when entering manual mode from auto modes
     * @note Call after prolonged period of saturated outputs
     * @note Does not affect P or D terms
     * 
     * @warning Do not call during active control - causes control discontinuity
     */
    void relax_I();

    /**
     * @brief Parameter table for AP_Param system
     * 
     * @details Defines all configurable parameters for attitude and speed control including:
     *          - Steering controller gains (angle P, rate PID)
     *          - Throttle controller gains (speed PID, pitch PID)
     *          - Acceleration limits (throttle_accel_max, throttle_decel_max)
     *          - Rate limits (steer_rate_max, steer_accel_max)
     *          - Safety limits (turn_lateral_G_max, stop_speed)
     *          - Special vehicle parameters (pitch feed-forward, heel PID)
     * 
     * @note Loaded from EEPROM during initialization
     * @note Modified via ground station parameter interface
     * @note Changes take effect immediately when written
     */
    static const struct AP_Param::GroupInfo var_info[];

private:

    static AR_AttitudeControl *_singleton;

    // parameters
    AC_P     _steer_angle_p;        // steering angle controller
    AC_PID   _steer_rate_pid;       // steering rate controller
    AC_PID   _throttle_speed_pid;   // throttle speed controller
    AC_PID   _pitch_to_throttle_pid;// balancebot pitch controller
    AP_Float _pitch_to_throttle_ff; // balancebot feed forward from current pitch angle
    AP_Float _pitch_limit_tc;       // balancebot pitch limit protection time constant
    AP_Float _pitch_limit_throttle_thresh;  // balancebot pitch limit throttle threshold (in the range 0 to 1.0)

    AP_Float _throttle_accel_max;   // speed/throttle control acceleration (and deceleration) maximum in m/s/s.  0 to disable limits
    AP_Float _throttle_decel_max;    // speed/throttle control deceleration maximum in m/s/s. 0 to use ATC_ACCEL_MAX for deceleration
    AP_Int8  _brake_enable;         // speed control brake enable/disable. if set to 1 a reversed output to the motors to slow the vehicle.
    AP_Float _stop_speed;           // speed control stop speed.  Motor outputs to zero once vehicle speed falls below this value
    AP_Float _steer_accel_max;      // steering angle acceleration max in deg/s/s
    AP_Float _steer_decel_max;      // steering angle deceleration max in deg/s/s
    AP_Float _steer_rate_max;       // steering rate control maximum rate in deg/s
    AP_Float _turn_lateral_G_max;   // sterring maximum lateral acceleration limit in 'G'

    // steering control
    uint32_t _steer_lat_accel_last_ms;  // system time of last call to lateral acceleration controller (i.e. get_steering_out_lat_accel)
    uint32_t _steer_turn_last_ms;   // system time of last call to steering rate controller
    float    _desired_lat_accel;    // desired lateral acceleration (in m/s/s) from latest call to get_steering_out_lat_accel (for reporting purposes)
    float    _desired_turn_rate;    // desired turn rate (in radians/sec) either from external caller or from lateral acceleration controller
    bool     _steering_limit_left;  // true when the steering control has reached its left limit (e.g. motor has reached limits or accel or turn rate limits applied)
    bool     _steering_limit_right; // true when the steering control has reached its right limit (e.g. motor has reached limits or accel or turn rate limits applied)

    // throttle control
    uint32_t _speed_last_ms;        // system time of last call to get_throttle_out_speed
    float    _desired_speed;        // last recorded desired speed
    uint32_t _stop_last_ms;         // system time the vehicle was at a complete stop
    bool     _throttle_limit_low;   // throttle output was limited from going too low (used to reduce i-term buildup)
    bool     _throttle_limit_high;  // throttle output was limited from going too high (used to reduce i-term buildup)
    AP_PIDInfo _throttle_speed_pid_info;   // local copy of throttle_speed controller's PID info to allow reporting of unusual FF

    // balancebot pitch control
    uint32_t _balance_last_ms = 0;  // system time that get_throttle_out_from_pitch was last called
    float _pitch_limit_low = 0;     // min desired pitch (in radians) used to protect against falling over
    float _pitch_limit_high = 0;    // max desired pitch (in radians) used to protect against falling over
    bool _pitch_limited = false;    // true if pitch was limited on last call to get_throttle_out_from_pitch

    // Sailboat heel control
    AC_PID   _sailboat_heel_pid;    // Sailboat heel angle pid controller
    uint32_t _heel_controller_last_ms = 0;
};
