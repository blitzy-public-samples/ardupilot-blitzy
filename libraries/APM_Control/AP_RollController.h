/**
 * @file AP_RollController.h
 * @brief Roll axis controller for fixed-wing aircraft
 * 
 * @details Implements cascaded angle-to-rate-to-servo control for fixed-wing roll stability and maneuvering.
 *          This controller converts desired roll angle to aileron servo commands through a two-stage process:
 *          1. Angle loop: roll angle error → desired roll rate (proportional gain)
 *          2. Rate loop: desired roll rate → servo output (PID with feedforward)
 *          
 *          The rate PID controller is inherited from AP_FW_Controller base class and uses AC_PID implementation.
 *          Gains are automatically scaled by airspeed to maintain consistent control response across the flight envelope.
 *          
 *          Source: libraries/APM_Control/AP_RollController.h
 */

#pragma once

#include "AP_FW_Controller.h"

/**
 * @class AP_RollController
 * @brief Fixed-wing roll controller derived from AP_FW_Controller
 * 
 * @details Implements cascaded roll control for fixed-wing aircraft:
 * 
 * **Control Architecture:**
 * - Angle loop: Converts roll angle error (desired - actual) to desired roll rate using proportional gain
 * - Rate loop: Implements PID control with feedforward to convert desired rate to servo output
 * - Rate PID implementation inherited from base class AP_FW_Controller using AC_PID
 * 
 * **Airspeed Scaling:**
 * - All gains scaled by (equivalent_airspeed / reference_airspeed)^2
 * - Rationale: Control surface effectiveness proportional to dynamic pressure (airspeed^2)
 * - Maintains consistent response feel across speed range from slow flight to high-speed cruise
 * 
 * **Sensor Feedback:**
 * - Uses AHRS gyro_x (body frame roll rate) for rate feedback
 * - Body frame convention: positive roll rate = right wing down (clockwise looking forward)
 * 
 * **Autotune Support:**
 * - Integrates with AP_AutoTune for in-flight gain optimization
 * - Can automatically determine optimal FF, P, I, D gains through flight test maneuvers
 * 
 * **Integrator Management:**
 * - Automatic integrator freeze when on ground (prevents wind-up during takeoff)
 * - Manual integrator disable in FBWA mode (fly-by-wire A)
 * - Integrator clamping when motor limits reached
 * 
 * **Control Flow:**
 * 1. get_servo_out() receives angle error and airspeed scaler
 * 2. Angle loop: angle_err (centidegrees) × P_gain → desired_roll_rate (deg/s)
 * 3. Rate limiting: desired_roll_rate clamped to ±rmax parameter
 * 4. Rate loop: AP_FW_Controller::_get_rate_out() implements rate PID with feedforward
 * 5. Output: servo command in centidegrees, range-constrained to [-4500, 4500]
 * 
 * **Transfer Function (simplified):**
 * - Angle loop: desired_rate = P × angle_error
 * - Rate loop: servo = FF×desired_rate + P×rate_error + I×∫rate_error + D×d(rate_error)/dt
 * - Derivative filter: first-order low-pass with time constant tau
 * 
 * **Coordinate Frames:**
 * - Body frame: roll rate positive = right wing down (right-hand rule, x-axis forward)
 * - Servo output: positive = right aileron down (generates roll right)
 * 
 * **Units and Conventions:**
 * - Angle errors: centidegrees (×100 degrees), range ±18000 (±180°)
 * - Rates: deg/s
 * - Servo outputs: centidegrees, range [-4500, 4500] = [-45°, 45°]
 * - Airspeed: m/s (equivalent airspeed)
 * - Time constants: seconds
 * 
 * @note Integrator automatically frozen when disable_integrator=true (FBWA mode), 
 *       ground_mode=true, or motor limits reached
 * @note Autotune can determine optimal gains in flight via AP_AutoTune integration 
 *       (inherited from base class)
 * @note Rate control typically runs at main loop rate (50Hz for fixed-wing)
 * 
 * @warning P gain (angle→rate) directly affects roll response and spiral stability. 
 *          Too high causes roll oscillations. Too low causes sluggish response and 
 *          inability to track commanded bank angles.
 * @warning Rate PID gains (I, D, FF) affect inner loop stability. Inappropriate values 
 *          can cause rapid oscillations or servo flutter. Start with conservative values 
 *          and increase gradually during tuning.
 * @warning rmax parameter limits maximum roll rate. Setting too high can exceed 
 *          structural limits or cause loss of control in aggressive maneuvers. 
 *          Consult aircraft specifications.
 * 
 * @see AP_FW_Controller - Base class providing rate PID implementation
 * @see AC_PID - PID controller implementation used for rate loop
 * @see AP_AutoTune - Automatic gain tuning system
 */
class AP_RollController : public AP_FW_Controller
{
public:
    /**
     * @brief Constructor for fixed-wing roll controller
     * 
     * @param[in] parms Reference to AP_FixedWing parameter structure containing aircraft-specific configuration
     * 
     * @details Initializes the roll controller with reference to the main fixed-wing parameter structure.
     *          The constructor passes parameters to base class AP_FW_Controller which sets up the rate PID.
     *          Controller is ready to use after construction but requires parameter loading from EEPROM.
     * 
     * @note Constructor does not perform parameter validation or AHRS checks - those occur at runtime
     * @see AP_FW_Controller::AP_FW_Controller()
     */
    AP_RollController(const AP_FixedWing &parms);

    /* Do not allow copies */
    CLASS_NO_COPY(AP_RollController);

    /**
     * @brief Calculate aileron servo output from roll angle error
     * 
     * @param[in] angle_err Roll angle error in centidegrees (desired_angle - actual_angle)
     *                      Positive value means need to roll right (right wing down)
     *                      Typical range: ±9000 (±90°) for normal flight, can reach ±18000 (±180°)
     * @param[in] scaler Airspeed scaling factor = (equivalent_airspeed / reference_airspeed)^2
     *                   Used to scale P and I gains for consistent response across speed range
     *                   Typical range: 0.5 to 2.0 (half to double reference airspeed)
     * @param[in] disable_integrator If true, freezes integrator accumulation (used in FBWA mode)
     *                                Set true when pilot has direct rate control to prevent integrator wind-up
     * @param[in] ground_mode If true, resets integrator to prevent wind-up while on ground
     *                        Should be true when aircraft is not airborne
     * 
     * @return Aileron servo output in centidegrees, range [-4500, 4500] = [-45°, 45°]
     *         Positive output = right aileron down = roll right
     *         Output is clamped to SRV_Channel limits
     * 
     * @details This is the main control method called at loop rate (typically 50Hz). Implements two-stage control:
     * 
     * **Stage 1 - Angle Loop:**
     * - Converts angle_err to desired_roll_rate using proportional gain P
     * - desired_rate = (angle_err / 100) × P_gain (converts centidegrees to degrees)
     * - Rate limit applied: desired_rate clamped to ±rmax parameter
     * 
     * **Stage 2 - Rate Loop:**
     * - Calls inherited _get_rate_out() with desired rate and current measured rate
     * - Rate PID calculates servo output: FF×desired_rate + P×error + I×∫error + D×d(error)/dt
     * - P and I gains scaled by scaler^2 (airspeed compensation)
     * - FF and D gains not scaled (already velocity-independent)
     * 
     * **Integrator Management:**
     * - Frozen if disable_integrator == true
     * - Reset if ground_mode == true
     * - Frozen if rate_error × integrator > 0 and motor limits reached (anti-windup)
     * - Clamped to ±IMAX parameter
     * 
     * **Underspeed Protection:**
     * - If airspeed < min_groundspeed, rate PID gains reduced to prevent stall aggravation
     * - Integrator frozen during underspeed to prevent wind-up
     * 
     * @note Called every loop iteration in auto modes (AUTO, LOITER, RTL, GUIDED)
     * @note In FBWA mode, disable_integrator typically true to give pilot direct rate control
     * @note Output is further processed by SRV_Channel for trim, reverse, and PWM conversion
     * 
     * @warning Ensure angle_err is in correct sign convention (desired - actual)
     * @warning scaler must be positive and non-zero to avoid divide-by-zero in base class
     * @warning High angle errors (>90°) can cause aggressive commands - consider limiting upstream
     * 
     * @see AP_FW_Controller::_get_rate_out() - Rate loop implementation
     * @see get_measured_rate() - Roll rate sensor feedback
     * @see is_underspeed() - Underspeed condition detection
     */
    float get_servo_out(int32_t angle_err, float scaler, bool disable_integrator, bool ground_mode) override;

    /**
     * @brief Parameter table for roll controller
     * 
     * @details Defines the parameter group for roll controller configuration:
     * - P: Angle loop proportional gain (angle error → desired rate), units: deg/s per degree error
     * - I: Rate loop integral gain, units: servo centidegrees per degree-second of accumulated error
     * - D: Rate loop derivative gain, units: servo centidegrees per degree/s² of error rate change
     * - FF: Rate loop feedforward gain, units: servo centidegrees per deg/s of desired rate
     * - tau: Derivative filter time constant, units: seconds (typical: 0.1 to 0.5)
     * - IMAX: Integrator output limit, units: servo centidegrees (typical: 4500 for full range)
     * - rmax: Maximum roll rate limit, units: deg/s (typical: 60 to 180 depending on aircraft)
     * 
     * @note Parameters stored in EEPROM with prefix RLL2SRV_ (e.g., RLL2SRV_P, RLL2SRV_I)
     * @note Parameters can be modified in flight via MAVLink or ground station
     * @note Default values set for typical small fixed-wing aircraft (1-2m wingspan)
     * 
     * @see AP_Param::GroupInfo for parameter system details
     */
    static const struct AP_Param::GroupInfo var_info[];

    /**
     * @brief Convert legacy PID parameters to new format
     * 
     * @details Utility function to migrate from old parameter naming/scaling to current format.
     *          Called automatically during parameter load if old-format parameters detected.
     *          Converts parameter names and applies scaling factors for changed units.
     * 
     * @note Only relevant for upgrades from very old firmware versions (pre-2016)
     * @note Safe to call multiple times - checks if conversion already performed
     * @note Conversion is one-way and permanent once executed
     * 
     * @see AP_Param::convert_old_parameter() for conversion mechanism
     */
    void convert_pid();

private:
    /**
     * @brief Get current equivalent airspeed from AHRS
     * 
     * @return Equivalent airspeed in m/s (sea level equivalent)
     * 
     * @details Retrieves airspeed from AP_AHRS system which fuses:
     *          - Airspeed sensor (if available and healthy)
     *          - GPS groundspeed (if airspeed sensor unavailable)
     *          - Synthetic airspeed from throttle/attitude (backup)
     *          
     *          Equivalent Airspeed (EAS) is used rather than True Airspeed (TAS) because
     *          control effectiveness depends on dynamic pressure which is proportional to EAS^2.
     * 
     * @note Overrides pure virtual function from AP_FW_Controller base class
     * @note Returns smoothed/filtered value to prevent rapid gain changes
     * @note If all airspeed sources fail, returns min_groundspeed as fallback
     * 
     * @see AP_AHRS::get_EAS2TAS() for EAS/TAS conversion
     * @see AP_FixedWing::min_groundspeed parameter
     */
    float get_airspeed() const override;

    /**
     * @brief Check if aircraft is below minimum safe airspeed
     * 
     * @param[in] aspeed Current equivalent airspeed in m/s
     * 
     * @return true if airspeed is below min_groundspeed threshold, false otherwise
     * 
     * @details Used to detect potential stall conditions or slow flight where:
     *          - Control surface effectiveness is greatly reduced
     *          - Aircraft may not respond normally to control inputs
     *          - Aggressive control commands could worsen situation
     *          
     *          When underspeed detected:
     *          - Rate PID gains are reduced to prevent over-control
     *          - Integrator is frozen to prevent wind-up
     *          - More conservative control limits applied
     * 
     * @note Overrides pure virtual function from AP_FW_Controller base class
     * @note Threshold is AP_FixedWing::min_groundspeed parameter (typically 5-10 m/s)
     * @note Hysteresis may be applied to prevent rapid mode switching at threshold
     * 
     * @warning Underspeed condition indicates potential stall risk - flight mode should 
     *          prioritize altitude recovery over precise attitude control
     * 
     * @see AP_FixedWing::min_groundspeed parameter
     */
    bool is_underspeed(const float aspeed) const override;

    /**
     * @brief Get measured roll rate from AHRS gyro sensor
     * 
     * @return Body frame roll rate in rad/s (positive = right wing down)
     * 
     * @details Retrieves gyro_x (body frame x-axis angular rate) from AP_AHRS system which provides:
     *          - Calibrated gyro data from primary IMU
     *          - Multi-IMU blending if multiple IMUs available
     *          - Bias correction and temperature compensation
     *          - Notch filtering for propeller/motor vibration rejection
     *          
     *          Body frame convention:
     *          - X-axis points forward (nose)
     *          - Y-axis points right (right wing)
     *          - Z-axis points down
     *          - Positive roll rate = rotation about X-axis = right wing down (clockwise looking forward)
     * 
     * @note Overrides pure virtual function from AP_FW_Controller base class
     * @note Returns unfiltered gyro for minimal phase lag (filtering applied in rate PID if needed)
     * @note Units are rad/s - converted to deg/s internally by rate controller
     * @note Rate sensor range typically ±2000 deg/s (±35 rad/s) for standard IMUs
     * 
     * @see AP_AHRS::get_gyro() for raw gyro access
     * @see AP_InertialSensor for IMU details and filtering
     */
    float get_measured_rate() const override;
};
