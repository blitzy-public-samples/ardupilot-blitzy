/**
 * @file AP_PitchController.h
 * @brief Pitch axis controller for fixed-wing aircraft
 * 
 * @details Implements angle→rate→elevator control with coordinated turn compensation
 *          for fixed-wing pitch stability and altitude control. Converts pitch angle
 *          error to elevator servo command via cascaded control loops.
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

#include "AP_FW_Controller.h"

/**
 * @class AP_PitchController
 * @brief Fixed-wing pitch controller derived from AP_FW_Controller
 * 
 * @details Converts pitch angle error to elevator servo command via cascaded control:
 *          angle_error → desired_pitch_rate (P) → servo_output (rate PID with FF/D)
 * 
 *          Control Architecture:
 *          - Angle loop: Proportional controller converts pitch angle error to desired pitch rate
 *          - Rate loop: PID controller with feedforward converts rate error to servo command
 *          - Coordinated turn compensation: Adds roll-dependent pitch rate correction to
 *            maintain level flight during coordinated turns
 * 
 *          Rate PID: Implemented in base class AP_FW_Controller using AC_PID
 * 
 *          Airspeed Scaling: Gains scaled by (equivalent_airspeed / reference_airspeed)^2
 *          for consistent response across speed range. P and I gains are scaled by scaler^2,
 *          while FF and D gains are applied without scaling.
 * 
 *          Sensor: Uses AHRS gyro_y (body frame pitch rate) for feedback
 * 
 *          Autotune Support: Can run in-flight autotune to determine optimal FF/P/I/D gains
 * 
 *          Transfer Function (simplified):
 *          - Angle loop: desired_rate = P × error + roll_ff_correction
 *          - Rate loop: servo = FF×desired_rate + P×rate_error + I×∫rate_error + D×d(rate_error)/dt
 *          - Derivative filter: first-order with time constant tau
 *          - Turn correction: roll_ff × roll_rate × airspeed_scale
 * 
 *          Control Flow:
 *          1. Angle loop: angle_err (centidegrees) × P_gain → desired_pitch_rate (deg/s)
 *          2. Coordinated turn compensation: desired_pitch_rate += _get_coordination_rate_offset()
 *          3. Rate limit: desired_pitch_rate clamped to ±rmax
 *          4. Rate loop: AP_FW_Controller::_get_rate_out() implements rate PID with feedforward
 *          5. Output: servo command in centidegrees, constrained to [-4500, 4500]
 * 
 *          Coordinate Frames:
 *          - Body frame: pitch rate positive = nose up (clockwise looking right)
 *          - Servo output: positive = elevator trailing edge down (nose up)
 * 
 *          Units and Conventions:
 *          - Angle errors in centidegrees (×100 degrees): range ±18000 (±180°)
 *          - Rates in deg/s
 *          - Servo outputs in centidegrees: range [-4500, 4500] = [-45°, 45°]
 *          - Airspeed in m/s
 *          - Time constant tau in seconds
 * 
 * @note Inherits rate_pid (AC_PID), autotune support, integrator management from AP_FW_Controller base class
 * 
 * @warning P gain (angle→rate) directly affects pitch response and phugoid damping. Too high
 *          causes pitch oscillations. Too low causes poor altitude tracking and slow response
 *          to elevator commands.
 * 
 * @warning Rate PID gains (I, D, FF) affect inner loop stability and can cause pitch oscillations
 *          or porpoising if tuned improperly. Start conservative and increase gradually.
 * 
 * @warning rmax parameter limits maximum pitch rate. Setting too high can cause stalls or
 *          high-speed dives. Setting too low limits maneuverability.
 * 
 * @warning roll_ff gain provides coordinated turn compensation. If zero, aircraft will
 *          lose/gain altitude during turns. If too high, can cause pitch oscillations during
 *          roll maneuvers.
 */
class AP_PitchController : public AP_FW_Controller
{
public:
    /**
     * @brief Constructor for pitch controller
     * 
     * @param[in] parms Reference to AP_FixedWing parameter structure containing
     *                  vehicle-specific configuration (min speed, max rate limits, etc.)
     * 
     * @details Initializes the pitch controller with references to the vehicle's
     *          fixed-wing parameters. The base class AP_FW_Controller constructor
     *          sets up the rate PID controller (AC_PID) and autotune support.
     */
    AP_PitchController(const AP_FixedWing &parms);

    /* Do not allow copies */
    CLASS_NO_COPY(AP_PitchController);

    /**
     * @brief Main control output method - computes elevator servo command from pitch angle error
     * 
     * @param[in] angle_err Pitch angle error in centidegrees (desired - actual), positive = nose up
     *                      Typical range: ±18000 centidegrees (±180°)
     * @param[in] scaler Airspeed scaling factor computed as (EAS/reference_airspeed)^2
     *                   Used to scale gains for consistent control response across speeds
     *                   Typical range: 0.5 to 2.0
     * @param[in] disable_integrator true to freeze integrator accumulation (prevents windup
     *                               during saturated conditions or mode transitions)
     * @param[in] ground_mode true when aircraft is on ground to prevent integrator wind-up
     *                        and inappropriate control surface deflections during taxi/launch
     * 
     * @return Elevator servo output in centidegrees, range [-4500, 4500] (±45°)
     *         Positive value = elevator trailing edge down (nose up)
     * 
     * @details Control algorithm sequence:
     *          1. Angle loop: Converts angle_err to desired_pitch_rate using P gain
     *          2. Adds coordinated turn compensation (roll-rate dependent correction)
     *          3. Limits desired_pitch_rate to ±rmax parameter
     *          4. Rate loop (in base class): Computes servo output using PID + feedforward
     *          5. Applies airspeed scaling to P and I gains
     *          6. Returns constrained servo command
     * 
     *          Integrator Management:
     *          - Automatically frozen when disable_integrator=true
     *          - Automatically frozen when ground_mode=true
     *          - Automatically frozen when motor limits reached (inherited from base)
     * 
     *          Coordinated Turn Logic:
     *          - Adds pitch rate offset proportional to roll rate during turns
     *          - Prevents altitude loss/gain during coordinated turns
     *          - Checks for inverted flight (roll > 90°) and adjusts sign accordingly
     * 
     * @note Rate control runs at main loop rate (typically 50Hz for fixed-wing)
     * @note Overrides pure virtual method from AP_FW_Controller base class
     * 
     * @see _get_coordination_rate_offset() for turn compensation details
     * @see AP_FW_Controller::_get_rate_out() for rate loop implementation
     */
    float get_servo_out(int32_t angle_err, float scaler, bool disable_integrator, bool ground_mode) override;

    /**
     * @brief Parameter table for Doxygen documentation
     * 
     * @details Contains PID parameters for pitch control:
     *          - P: Angle-to-rate gain (angle loop proportional gain)
     *          - I: Rate integral gain (corrects for sustained pitch rate errors)
     *          - D: Rate derivative gain (damping for rate loop)
     *          - FF: Feedforward gain (maps desired rate directly to servo)
     *          - tau: Derivative filter time constant in seconds
     *          - IMAX: Integrator limit in centidegrees (prevents excessive windup)
     *          - rmax: Maximum pitch rate limit in deg/s (safety constraint)
     *          - roll_ff: Coordinated turn feedforward gain (roll rate to pitch rate)
     * 
     * @note These parameters are stored in EEPROM and accessible via ground control station
     */
    static const struct AP_Param::GroupInfo var_info[];

    /**
     * @brief Legacy parameter conversion utility
     * 
     * @details Converts old parameter format to new parameter format when upgrading
     *          from older ArduPilot versions. Ensures parameter compatibility across
     *          firmware updates without losing user tuning settings.
     * 
     * @note Called automatically during parameter system initialization
     * @note Only performs conversion if old-format parameters are detected
     */
    void convert_pid();

private:
    /**
     * @brief Coordinated turn feedforward gain parameter
     * 
     * @details Adds pitch rate correction proportional to roll rate during turns to maintain
     *          level flight. During a coordinated turn, the lift vector is tilted, requiring
     *          additional pitch rate to maintain altitude. This gain compensates for that effect.
     * 
     *          Physical interpretation: Maps roll rate to required pitch rate adjustment
     *          
     *          Typical values: 0.5 to 2.0 depending on aircraft lift-to-drag ratio
     *          
     *          Effect:
     *          - Zero: Aircraft will lose altitude in turns (requires pilot correction)
     *          - Optimal: Aircraft maintains altitude automatically during coordinated turns
     *          - Too high: Aircraft gains altitude in turns, may cause pitch oscillations
     * 
     * @note Stored as AP_Float parameter accessible via ground control station
     * @note Applied with airspeed scaling in _get_coordination_rate_offset()
     */
    AP_Float _roll_ff;

    /**
     * @brief Compute pitch rate adjustment for coordinated turns
     * 
     * @param[in] aspeed Equivalent airspeed in m/s (used for gain scaling)
     * @param[out] inverted Set to true if aircraft is inverted (roll angle > 90°),
     *                      used by caller to adjust control logic
     * 
     * @return Pitch rate offset in deg/s to be added to desired pitch rate
     *         Positive = nose up correction needed
     * 
     * @details Coordinated turn compensation algorithm:
     *          1. Get current roll rate from AHRS (body frame)
     *          2. Check if inverted (roll angle > 90°) and set inverted flag
     *          3. Compute scaling factor based on airspeed
     *          4. Calculate offset = _roll_ff × roll_rate × airspeed_scaling
     *          5. Adjust sign if inverted
     * 
     *          Physics: During a banked turn, the lift vector is tilted. To maintain
     *          altitude, the total lift must increase, requiring increased angle of
     *          attack (pitch up). The required pitch rate is proportional to the
     *          roll rate and airspeed.
     * 
     *          Rationale: Prevents altitude loss/gain during turns without requiring
     *          pilot input or altitude controller correction (feedforward compensation)
     * 
     * @note Called from get_servo_out() after angle loop, before rate loop
     * @note Airspeed scaling maintains consistent turn coordination across speed range
     * 
     * @see get_servo_out() for integration into control loop
     */
    float _get_coordination_rate_offset(const float &aspeed, bool &inverted) const;

    /**
     * @brief Get current airspeed from AHRS
     * 
     * @return Equivalent airspeed in m/s
     * 
     * @details Retrieves current equivalent airspeed estimate from the AHRS (Attitude and
     *          Heading Reference System). Equivalent airspeed (EAS) is used instead of
     *          true airspeed to ensure consistent control response across altitude changes.
     * 
     *          EAS accounts for air density variations with altitude:
     *          EAS = TAS × sqrt(density / sea_level_density)
     * 
     * @note Overrides pure virtual method from AP_FW_Controller base class
     * @note Used for airspeed scaling of control gains and turn coordination
     * 
     * @see is_underspeed() for minimum airspeed checking
     */
    float get_airspeed() const override;

    /**
     * @brief Check if airspeed is below minimum threshold
     * 
     * @param[in] aspeed Equivalent airspeed in m/s to check
     * 
     * @return true if airspeed is below minimum groundspeed parameter, false otherwise
     * 
     * @details Used to detect slow flight or stall conditions where rate control should
     *          be disabled or modified. When underspeed is detected:
     *          - Rate PID may be disabled to prevent control surface stall aggravation
     *          - Integrator may be frozen to prevent windup during low-speed maneuvering
     *          - Control gains may be adjusted for low-speed flight characteristics
     * 
     *          Minimum speed threshold comes from vehicle min_groundspeed parameter,
     *          which is typically set to stall speed plus a safety margin.
     * 
     * @note Overrides pure virtual method from AP_FW_Controller base class
     * @note Called internally by rate controller to adjust behavior in slow flight
     * 
     * @warning Critical for stall prevention - do not disable without understanding
     *          implications for flight safety
     */
    bool is_underspeed(const float aspeed) const override;

    /**
     * @brief Get pitch rate sensor reading
     * 
     * @return Body frame pitch rate in rad/s from AHRS gyro_y sensor
     *         Positive = nose up (clockwise rotation when looking from right side)
     * 
     * @details Retrieves the current pitch rate measurement from the AHRS gyroscope
     *          (gyro_y in body frame). This is the primary feedback for the rate control loop.
     * 
     *          Sensor: Typically a MEMS gyroscope (e.g., MPU6000, ICM-20689) after:
     *          - Temperature compensation
     *          - Bias correction
     *          - Low-pass filtering
     *          - Multi-IMU blending (if multiple IMUs installed)
     * 
     *          Body Frame Convention:
     *          - X-axis: forward (nose direction)
     *          - Y-axis: right wing
     *          - Z-axis: down
     *          - gyro_y: pitch rate (rotation about Y-axis)
     * 
     * @note Overrides pure virtual method from AP_FW_Controller base class
     * @note Update rate: Typically 400Hz gyro sampling, filtered to control loop rate (50Hz)
     * @note Units: rad/s (converted internally from deg/s by rate controller)
     * 
     * @see get_servo_out() for usage in rate control loop
     */
    float get_measured_rate() const override;

};
