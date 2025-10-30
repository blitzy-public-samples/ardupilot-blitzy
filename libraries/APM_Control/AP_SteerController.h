/**
 * @file AP_SteerController.h
 * @brief Steering controller for ground and marine vehicles using PID control
 * 
 * @details Implements a steering PID controller that produces servo outputs
 *          for ground and marine vehicles. The controller supports multiple
 *          input modes: lateral acceleration, yaw rate, or heading error.
 *          
 *          Key features:
 *          - Speed-adaptive gains: Controller gains are scaled by 1/speed for
 *            stability at varying velocities
 *          - Integrator anti-windup: Integrator clamping prevents wind-up
 *            during sustained errors or actuator saturation
 *          - High-speed derating: Reduces controller authority at high speeds
 *            per _deratespeed/_deratefactor parameters
 *          - Feedforward compensation: Optional feedforward path for improved
 *            tracking performance
 *          
 *          Integration:
 *          - Uses AP_Param for persistent parameter storage
 *          - Uses AP_HAL for timing
 *          - Uses AP_PIDInfo for diagnostic logging/telemetry
 *          
 *          Units and Conventions:
 *          - Servo outputs: centidegrees (×100 degrees), range [-4500, 4500] = [-45°, 45°]
 *          - Lateral acceleration: m/s/s (positive = rightward)
 *          - Yaw rates: deg/s (positive = clockwise/right turn)
 *          - Speed: m/s
 *          - Angles: centidegrees
 *          
 *          Coordinate Frames:
 *          - Body frame: positive yaw rate = clockwise rotation (right turn) viewed from above
 *          - Lateral acceleration: positive = rightward acceleration in body frame
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

#include <AP_Common/AP_Common.h>
#include <AC_PID/AP_PIDInfo.h>

/**
 * @class AP_SteerController
 * @brief Ground and marine vehicle steering PID controller
 * 
 * @details This controller produces centidegree servo outputs from desired
 *          lateral acceleration, yaw rate, or heading error inputs. The
 *          controller is designed for ground and marine vehicles where
 *          steering response varies with vehicle speed.
 *          
 *          Speed Adaptation:
 *          Gains are internally scaled by 1/speed to maintain consistent
 *          steering response across a wide speed range. At low speeds,
 *          the controller uses _minspeed to prevent excessive gain.
 *          
 *          Anti-Windup:
 *          Integrator is clamped to ±_imax (centidegrees) to prevent
 *          integrator wind-up during sustained errors or when actuator
 *          limits are reached.
 *          
 *          High-Speed Derating:
 *          Above _deratespeed, controller output is multiplied by
 *          _deratefactor to reduce steering authority and improve
 *          stability at high speeds.
 *          
 *          PID Structure:
 *          output = (K_P * error + K_I * integral + K_D * derivative + K_FF * feedforward) * speed_scaler * derate_scaler
 *          
 *          Thread Safety:
 *          This controller is not thread-safe. It should only be called
 *          from the main control loop or with appropriate external locking.
 * 
 * @note Speed-adaptive gain scaling improves stability across wide speed ranges
 * @note Integrator limited to ±_imax to prevent wind-up
 * @warning Tuning parameters _K_P, _K_I, _K_D directly affect steering stability
 *          and responsiveness. Inappropriate values can cause oscillations or loss of control.
 */
class AP_SteerController {
public:
    /**
     * @brief Constructor - initializes controller with default parameters
     * 
     * @details Sets up AP_Param object defaults from var_info table.
     *          All PID gains and configuration parameters are loaded
     *          from persistent storage or set to compile-time defaults.
     */
    AP_SteerController()
    {
        AP_Param::setup_object_defaults(this, var_info);
    }

    /* Do not allow copies */
    CLASS_NO_COPY(AP_SteerController);

    /**
     * @brief Return steering servo output from desired lateral acceleration
     * 
     * @details Converts desired lateral acceleration to steering servo output
     *          using PID control. This is the primary control mode for path
     *          following where the navigation controller computes required
     *          lateral acceleration to track a path.
     *          
     *          The controller applies speed-adaptive gain scaling and
     *          high-speed derating to maintain stability across speed ranges.
     *          
     *          Typical usage: Called at main loop rate (typically 50-400 Hz)
     *          from navigation controller during AUTO/GUIDED modes.
     * 
     * @param[in] desired_accel Desired lateral acceleration in m/s/s
     *                          Positive value = right turn (rightward acceleration)
     *                          Negative value = left turn (leftward acceleration)
     *                          Typical range: ±2 m/s/s for ground vehicles
     * 
     * @return Steering servo output in centidegrees [-4500, 4500]
     *         Positive = right turn, Negative = left turn
     *         Output is clamped to servo limits and respects _mindegree minimum
     * 
     * @note Controller internally scales gains by 1/MAX(speed, _minspeed)
     * @note Integrator is clamped to ±_imax to prevent wind-up
     * @warning Ensure vehicle speed is updated before calling this function
     * 
     * @see get_steering_out_rate() for rate-based control
     * @see get_steering_out_angle_error() for angle error control
     */
	int32_t get_steering_out_lat_accel(float desired_accel);

    /**
     * @brief Return steering servo output from desired yaw rate
     * 
     * @details Converts desired yaw rate (turn rate) to steering servo output
     *          using PID control. This mode is used for rate-based control
     *          such as ACRO mode or when commanding specific turn rates.
     *          
     *          The controller includes feedforward compensation (_K_FF) to
     *          improve tracking of commanded rates.
     * 
     * @param[in] desired_rate Desired yaw rate in deg/s
     *                         Positive = clockwise rotation (right turn) viewed from above
     *                         Negative = counter-clockwise rotation (left turn)
     *                         Typical range: ±30 deg/s for ground vehicles
     * 
     * @return Steering servo output in centidegrees [-4500, 4500]
     *         Positive = right turn, Negative = left turn
     * 
     * @note This mode bypasses lateral acceleration conversion
     * @note Speed-adaptive gain scaling still applies
     * 
     * @see get_steering_out_lat_accel() for acceleration-based control
     */
	int32_t get_steering_out_rate(float desired_rate);

    /**
     * @brief Return steering servo output from heading error
     * 
     * @details Converts heading error to steering servo output using PID
     *          control. This is used for heading hold modes where the
     *          controller maintains or achieves a target heading.
     *          
     *          The heading error is processed through the full PID loop
     *          including integral term for eliminating steady-state error.
     * 
     * @param[in] angle_err Heading error in centidegrees
     *                      Positive = vehicle pointing left of target (need to turn right)
     *                      Negative = vehicle pointing right of target (need to turn left)
     *                      Error is typically wrapped to [-18000, 18000] range
     * 
     * @return Steering servo output in centidegrees [-4500, 4500]
     *         Output commands correction in direction to eliminate error
     * 
     * @note Integrator accumulates error over time for zero steady-state error
     * @warning Large heading errors may cause integrator wind-up; consider
     *          calling reset_I() on significant heading changes
     * 
     * @see reset_I() to clear integrator
     */
	int32_t get_steering_out_angle_error(int32_t angle_err);

    /**
     * @brief Return minimum turn radius in meters
     * 
     * @details Returns the minimum turn radius the vehicle can achieve,
     *          calculated as half the proportional gain value (_K_P).
     *          This represents the tightest turn the vehicle can make
     *          at the reference speed.
     *          
     *          Turn radius is inversely related to maximum lateral
     *          acceleration capability: smaller radius = tighter turns.
     * 
     * @return Turn radius in meters
     *         Typical values: 2-10 meters for ground vehicles
     * 
     * @note This is a geometric relationship based on controller tuning
     * @note Actual achievable turn radius depends on vehicle speed,
     *       surface conditions, and mechanical limits
     */
    float get_turn_radius(void) const { return _K_P * 0.5f; }

    /**
     * @brief Reset integrator to zero
     * 
     * @details Clears the integral term accumulator. This should be called
     *          when switching to manual mode, during significant mode changes,
     *          or when large heading changes occur to prevent inappropriate
     *          integrator wind-up.
     *          
     *          Typical usage:
     *          - Transitioning from manual to automatic control
     *          - Entering steering control modes from non-steering modes
     *          - After large commanded heading changes
     *          - When vehicle has been stationary
     * 
     * @note Does not affect P or D terms, only integral accumulator
     * @note Controller will rebuild appropriate integrator value over time
     * 
     * @see get_steering_out_angle_error() which uses integrator
     */
	void reset_I();

    /**
     * @brief Parameter table for persistent storage
     * 
     * @details Defines parameters accessible via ground control station:
     *          - _tau: Time constant for derivative filter (seconds)
     *          - _K_FF: Feedforward gain (deg/s to servo output)
     *          - _K_P: Proportional gain
     *          - _K_I: Integral gain
     *          - _K_D: Derivative gain
     *          - _minspeed: Minimum speed for gain scaling (m/s)
     *          - _imax: Maximum integrator value (centidegrees)
     *          - _deratespeed: Speed above which to reduce authority (m/s)
     *          - _deratefactor: Derating multiplier at high speed (0-1)
     *          - _mindegree: Minimum servo output magnitude (centidegrees)
     */
	static const struct AP_Param::GroupInfo var_info[];

    /**
     * @brief Get PID diagnostic information
     * 
     * @details Returns reference to internal PID info structure containing
     *          current P, I, D, FF terms and other diagnostic data. This
     *          information is used for logging, telemetry, and tuning.
     *          
     *          The returned structure includes:
     *          - target: Desired value
     *          - actual: Current value (if available)
     *          - error: Control error
     *          - P: Proportional term contribution
     *          - I: Integral term contribution
     *          - D: Derivative term contribution
     *          - FF: Feedforward term contribution
     * 
     * @return Const reference to AP_PIDInfo structure for logging/telemetry
     * 
     * @note Data is updated each time a get_steering_out_* function is called
     * @note Structure is read-only to external callers
     */
    const class AP_PIDInfo& get_pid_info(void) const { return _pid_info; }

    /**
     * @brief Configure controller for reverse operation
     * 
     * @details Sets whether the steering controller should operate in reverse
     *          mode. When true, steering commands are inverted for backing up.
     *          
     *          This affects all steering output calculations but does not
     *          change the sign convention of inputs.
     * 
     * @param[in] reverse true = reverse steering polarity for backing up
     *                    false = normal forward steering operation (default)
     * 
     * @note Typically called when vehicle enters reverse gear or backup modes
     * @warning Ensure this matches actual vehicle direction to prevent
     *          steering in wrong direction during reverse operation
     */
    void set_reverse(bool reverse) {
        _reverse = reverse;
    }

    /**
     * @brief Check if controller has been run recently
     * 
     * @details Returns true if the controller has computed an output within
     *          a timeout period, indicating active use. This can be used to
     *          detect if steering control is stale or if the controller
     *          should be reset.
     *          
     *          The timeout threshold is typically 0.2-1.0 seconds depending
     *          on expected loop rates.
     * 
     * @return true if controller has run recently (within timeout period)
     *         false if controller output is stale or never run
     * 
     * @note Useful for detecting control system failures or mode transitions
     * @see reset_I() to clear integrator when restarting after period of inactivity
     */
    bool active() const;

private:
    // PID controller parameters (persistent via AP_Param)
    
    /// Time constant for derivative filter in seconds (reduces derivative noise)
    AP_Float _tau;
    
    /// Feedforward gain: converts desired yaw rate (deg/s) directly to servo output (centidegrees)
    AP_Float _K_FF;
    
    /// Proportional gain: primary responsiveness tuning parameter
    AP_Float _K_P;
    
    /// Integral gain: eliminates steady-state error, must be tuned carefully to avoid wind-up
    AP_Float _K_I;
    
    /// Derivative gain: provides damping to reduce overshoot and oscillations
    AP_Float _K_D;
    
    /// Minimum speed for gain scaling in m/s (prevents excessive gains at very low speeds)
    AP_Float _minspeed;
    
    /// Maximum integrator value in centidegrees (anti-windup limit)
    AP_Int16  _imax;
    
    /// Timestamp of last controller update in milliseconds (from AP_HAL::millis())
    uint32_t _last_t;
    
    /// Last controller output in centidegrees (used for rate limiting and diagnostics)
    float _last_out;

    /// Speed above which to reduce controller authority in m/s (high-speed stability)
    AP_Float _deratespeed;
    
    /// Derating multiplier applied above _deratespeed (0-1, where 1=no derating)
    AP_Float _deratefactor;
    
    /// Minimum servo output magnitude in centidegrees (deadband compensation)
    AP_Float _mindegree;

    /// PID diagnostic information for logging and telemetry
    AP_PIDInfo _pid_info {};

    /// Reverse steering flag: true when operating in reverse/backup mode
    bool _reverse;
};
