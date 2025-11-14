/**
 * @file AP_YawController.h
 * @brief Yaw axis controller for fixed-wing aircraft
 * 
 * @details Implements dual-mode yaw control for fixed-wing aircraft:
 *          1. Rate-based rudder control (modern mode) - for coordinated turns
 *          2. Sideslip damping (legacy mode) - for adverse yaw compensation
 *          
 *          The controller operates in one of two mutually exclusive modes
 *          determined by the _rate_enable parameter.
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

#include <AP_Common/AP_Common.h>
#include <AC_PID/AC_PID.h>
#include "AP_AutoTune.h"

/**
 * @class AP_YawController
 * @brief Fixed-wing yaw/rudder controller with rate control and sideslip damping
 * 
 * @details This controller implements two distinct control modes for yaw axis:
 * 
 *          **RATE CONTROL MODE** (_rate_enable = 1, recommended for modern aircraft):
 *          - Converts desired yaw rate → rudder servo output
 *          - Uses rate PID controller (P, I, D, FF gains)
 *          - Supports in-flight autotune for automatic gain optimization
 *          - Provides coordinated turns and yaw rate tracking
 *          - Control law: servo = rate_pid.update(desired_rate - measured_rate)
 * 
 *          **SIDESLIP DAMPING MODE** (_rate_enable = 0, legacy mode):
 *          - Estimates sideslip from lateral acceleration and airspeed
 *          - Integrates sideslip error with _K_I gain
 *          - Applies yaw damping via high-pass filtered yaw rate (_K_D gain)
 *          - Optional feedforward from pilot rudder input (_K_FF)
 *          - Control law: servo = sideslip_integral + yaw_damping + feedforward
 * 
 *          **ARCHITECTURAL NOTES**:
 *          - Does NOT inherit from AP_FW_Controller (independent implementation
 *            predating the fixed-wing controller refactor)
 *          - Typically runs in rate control mode for modern aircraft
 *          - Sideslip mode retained for legacy tuning compatibility
 *          - Autotune available only in rate control mode
 * 
 *          **AIRSPEED SCALING**:
 *          - Sideslip mode: _K_I and _K_D scaled by airspeed
 *          - Rate control mode: P/I gains scaled by scaler² (via rate_pid.update_all)
 * 
 *          **COORDINATE FRAMES**:
 *          - Body frame: yaw rate positive = nose right (clockwise looking down)
 *          - Servo output: positive = rudder trailing edge left (causes nose right)
 * 
 *          **UNITS AND CONVENTIONS**:
 *          - Yaw rates in deg/s
 *          - Servo outputs in centidegrees: range [-4500, 4500] = [-45°, 45°]
 *          - Airspeed in m/s
 *          - Sideslip integrator in centidegrees
 *          - Time constants in seconds
 * 
 * @warning _rate_enable parameter determines control mode. Changing modes in flight
 *          will cause control transients. Set correctly on ground before flight.
 * 
 * @warning Sideslip mode (_rate_enable=0) is legacy and not recommended for new
 *          aircraft. Use rate control mode (_rate_enable=1) with properly tuned rate_pid.
 * 
 * @warning Rate PID gains (especially D) can cause rapid rudder oscillations if
 *          tuned improperly. Start with low D gain and increase gradually.
 * 
 * @warning Yaw damping gain (_K_D) in sideslip mode directly affects Dutch roll
 *          damping. Too high causes oscillations, too low allows Dutch roll.
 * 
 * @warning Sideslip integrator (_K_I, _K_A) in sideslip mode can cause slow turn
 *          coordination issues if tuned improperly.
 * 
 * @note Autotune only available in rate control mode (_rate_enable=1). Sideslip
 *       mode must be manually tuned.
 * 
 * @note High-pass filter for yaw damping uses simple first-order filter:
 *       y[n] = y[n-1] + (x[n] - x[n-1])
 * 
 * @note Rate control mode integrates with AP_AutoTune using same structure as
 *       roll/pitch controllers for consistent tuning interface.
 * 
 * @note decay_I() method is specifically for quadplane transitions to reduce
 *       integrator influence during hover modes.
 */
class AP_YawController
{
public:
    /**
     * @brief Constructor for yaw controller
     * 
     * @param[in] parms Reference to AP_FixedWing parameter structure containing
     *                  vehicle-specific configuration (airspeed limits, control limits)
     * 
     * @note Initializes rate_pid with default gains suitable for typical aircraft.
     *       These should be tuned per-vehicle or use autotune.
     */
    AP_YawController(const AP_FixedWing &parms);

    /* Do not allow copies */
    CLASS_NO_COPY(AP_YawController);

    /**
     * @brief Check if any yaw control is active
     * 
     * @details Returns true if either rate control mode is enabled OR
     *          yaw damping is active (in sideslip mode with _K_D > 0).
     *          Used to determine if yaw controller should be running.
     * 
     * @return true if rate control enabled (_rate_enable != 0) OR yaw damping
     *              gain (_K_D) is positive
     * @return false if all yaw control is disabled
     * 
     * @note This allows yaw damping to function independently in sideslip mode
     *       even if rate control is disabled.
     */
    bool enabled() const { return rate_control_enabled() || (_K_D > 0.0); } 

    /**
     * @brief Check if rate control mode is active
     * 
     * @details Rate control mode is the modern control mode where desired yaw
     *          rate is tracked using a PID controller. When enabled, use
     *          get_rate_out() instead of get_servo_out().
     * 
     * @return true if _rate_enable parameter is non-zero (rate control mode)
     * @return false if _rate_enable is zero (sideslip damping mode)
     * 
     * @note Used to determine which control method (rate vs sideslip) is active.
     */
    bool rate_control_enabled(void) const { return _rate_enable != 0; }

    /**
     * @brief Get rudder output for sideslip damping and yaw damping control
     * 
     * @details Implements legacy sideslip-based yaw control (used when _rate_enable = 0).
     *          Control algorithm:
     *          1. Estimate sideslip from lateral_accel / airspeed
     *          2. Integrate sideslip error: _integrator += _K_I * sideslip * dt
     *          3. Apply yaw damping: damping = _K_D * high_pass_filter(yaw_rate)
     *          4. Add feedforward: ff = _K_FF * pilot_rudder_input (if present)
     *          5. Combine and limit: output = _integrator + damping + ff, limited to ±_imax
     * 
     * @param[in] scaler Airspeed scaling factor for gain adjustment. Typically
     *                   computed as (airspeed / reference_airspeed). Scales _K_I
     *                   and _K_D to maintain control effectiveness across speeds.
     * @param[in] disable_integrator Set true to freeze sideslip integrator accumulation.
     *                                Used during ground operations or when integrator
     *                                should not wind up (e.g., before takeoff).
     * 
     * @return Rudder servo output in centidegrees, range [-4500, 4500] representing
     *         [-45°, 45°] servo deflection. Positive output = rudder trailing edge left
     *         (produces nose-right yaw moment).
     * 
     * @note This method is used ONLY when rate_control_enabled() returns false
     *       (i.e., _rate_enable = 0, legacy sideslip damping mode).
     * 
     * @note For modern aircraft, use get_rate_out() with _rate_enable = 1 instead.
     * 
     * @warning Sideslip estimation assumes coordinated flight. Accuracy degrades
     *          in sustained slips or skids.
     * 
     * @see get_rate_out() for the preferred rate control mode
     */
    int32_t get_servo_out(float scaler, bool disable_integrator);

    /**
     * @brief Get rudder output for rate control mode
     * 
     * @details Implements modern yaw rate tracking control (used when _rate_enable = 1).
     *          Control algorithm:
     *          1. Compute rate error: error = desired_rate - measured_yaw_rate
     *          2. Apply rate PID: output = P*error + I*∫error + D*d(error)/dt + FF*desired_rate
     *          3. Scale gains by airspeed: P and I gains scaled by scaler²
     *          4. Apply limits: output constrained by rate_pid.imax and output limits
     * 
     *          The rate_pid uses AC_PID with configurable P, I, D, FF, filter tau,
     *          integrator max, rate max, and optional notch filtering for oscillation
     *          suppression.
     * 
     * @param[in] desired_rate Desired yaw rate in deg/s. Positive = nose right.
     *                         Typically comes from navigation controller or pilot
     *                         input mapping. Range typically ±90 deg/s for normal flight.
     * @param[in] scaler Airspeed scaling factor (airspeed / reference_airspeed).
     *                   Used to scale P and I gains by scaler² to maintain consistent
     *                   control response across varying airspeeds.
     * @param[in] disable_integrator Set true to freeze rate PID integrator. Used during
     *                                mode transitions, ground operations, or when
     *                                integrator wind-up should be prevented.
     * 
     * @return Rudder servo output in centidegrees, range [-4500, 4500] representing
     *         [-45°, 45°] servo deflection. Positive output = rudder trailing edge left
     *         (produces nose-right yaw moment to achieve positive yaw rate).
     * 
     * @note This is the PRIMARY control method for modern aircraft with _rate_enable = 1.
     *       Provides better disturbance rejection and tuning than sideslip mode.
     * 
     * @note Supports in-flight autotune via autotune_start()/autotune_restore() for
     *       automatic gain optimization.
     * 
     * @note Rate PID diagnostics available via get_pid_info() for logging and telemetry.
     * 
     * @warning Ensure rate_pid gains are properly tuned. Excessive D gain can cause
     *          high-frequency rudder oscillations. Start with low gains and increase.
     * 
     * @see autotune_start() for automatic tuning
     * @see get_pid_info() for PID diagnostic information
     */
    float get_rate_out(float desired_rate, float scaler, bool disable_integrator);

    /**
     * @brief Reset sideslip integrator to zero
     * 
     * @details Clears the accumulated sideslip integral (_integrator) used in
     *          sideslip damping mode. Called during mode changes, initialization,
     *          or when integrator wind-up needs to be cleared.
     * 
     * @note This affects ONLY sideslip mode (_rate_enable = 0). Has no effect
     *       on rate control mode integrator.
     * 
     * @see reset_rate_PID() to reset rate control mode integrator
     */
    void reset_I();

    /**
     * @brief Reset rate controller integrator to zero
     * 
     * @details Clears the accumulated integral term in the rate PID controller
     *          used in rate control mode. Called during mode transitions,
     *          initialization, or when integrator wind-up needs clearing.
     * 
     * @note This affects ONLY rate control mode (_rate_enable = 1). Has no effect
     *       on sideslip mode integrator.
     * 
     * @see reset_I() to reset sideslip mode integrator
     */
    void reset_rate_PID();

    /**
     * @brief Gradually reduce integrator for quadplane hover transitions
     * 
     * @details Reduces the rate PID integrator by 95% over approximately 2 seconds.
     *          Used during quadplane transitions from forward flight to hover mode
     *          where yaw control authority changes significantly and accumulated
     *          integrator from forward flight may be inappropriate for hover.
     * 
     *          Decay rate: I_new = I_old × 0.995 per call
     *          At typical 50Hz update rate: 95% reduction in ~2 seconds
     * 
     * @note Called repeatedly during quadplane hover when scale factor is low.
     *       Does not instantly zero the integrator like reset methods.
     * 
     * @note Operates on _pid_info.I which is used for both control and logging.
     * 
     * @see reset_rate_PID() for immediate integrator reset
     */
    void decay_I()
    {
        // this reduces integrator by 95% over 2s
        _pid_info.I *= 0.995f;
    }

    /**
     * @brief Get rate PID diagnostic information
     * 
     * @details Returns const reference to AP_PIDInfo structure containing
     *          current PID controller state for logging, telemetry, and debugging.
     *          Structure includes:
     *          - target: Desired yaw rate (deg/s)
     *          - actual: Measured yaw rate (deg/s)  
     *          - error: Rate error (deg/s)
     *          - P, I, D, FF: Individual term contributions (centidegrees)
     *          - Dmod: Modified D term after filtering
     * 
     * @return Const reference to AP_PIDInfo structure with current PID state.
     *         Valid throughout object lifetime.
     * 
     * @note Updated every time get_rate_out() is called in rate control mode.
     * @note Used by logging system to record PID performance for analysis.
     * @note All term values in centidegrees except rates in deg/s.
     */
    const AP_PIDInfo& get_pid_info(void) const
    {
        return _pid_info;
    }

    /**
     * @brief Configure rate PID notch filter sample rate
     * 
     * @details Sets the sample rate for the rate PID's notch filters used to
     *          suppress resonances and oscillations. Must be called if control
     *          loop rate differs from default. Notch filters help eliminate
     *          structural resonances that could cause rudder oscillations.
     * 
     * @param[in] sample_rate Controller update rate in Hz (typically 50-400 Hz).
     *                        Should match the actual frequency at which get_rate_out()
     *                        is called.
     * 
     * @note Only affects rate control mode (_rate_enable = 1).
     * @note Must be set before flight if control loop rate is non-standard.
     * @note Notch filter configuration done in rate_pid parameters.
     */
    void set_notch_sample_rate(float sample_rate) { rate_pid.set_notch_sample_rate(sample_rate); }

    /**
     * @brief Start in-flight autotune for rate controller
     * 
     * @details Initiates automatic tuning of rate PID gains (P, I, D, FF).
     *          Allocates AP_AutoTune object and begins systematic gain optimization
     *          through controlled oscillation testing. Saves original gains for
     *          restoration if autotune is aborted or fails.
     * 
     * @note Only available in rate control mode (_rate_enable = 1).
     * @note Requires sufficient flight envelope for safe oscillation testing.
     * @note Pilot must maintain steady flight during autotune process.
     * 
     * @warning Autotune will command yaw oscillations. Ensure adequate altitude
     *          and safe flight conditions before starting.
     * 
     * @see autotune_restore() to abort and restore original gains
     */
    void autotune_start(void);
    
    /**
     * @brief Stop autotune and restore original gains
     * 
     * @details Terminates in-flight autotune, deallocates AP_AutoTune object,
     *          and restores rate PID gains to pre-autotune values. Call this
     *          to abort autotune or after successful completion to clean up.
     * 
     * @note If autotune completed successfully, tuned gains are already active.
     *       This only restores if you want to revert to original values.
     * 
     * @note Safe to call even if autotune not active (no-op).
     * 
     * @see autotune_start() to begin autotune
     */
    void autotune_restore(void);
    
    /**
     * @brief Parameter table for yaw controller configuration
     * 
     * @details AP_Param group information defining all user-configurable parameters:
     * 
     *          **SIDESLIP MODE PARAMETERS** (_rate_enable = 0):
     *          - _K_A: Sideslip integrator gain (integrator += _K_A * sideslip)
     *          - _K_I: Sideslip integral gain multiplier (scales _integrator in output)
     *          - _K_D: Yaw damping gain (Dutch roll damping via high-pass filtered yaw rate)
     *          - _K_FF: Feedforward gain from pilot rudder input
     *          - _imax: Sideslip integrator limit in centidegrees (prevents wind-up)
     * 
     *          **MODE SELECTION**:
     *          - _rate_enable: Control mode selector (0 = sideslip mode, 1 = rate control mode)
     * 
     *          **RATE CONTROL MODE PARAMETERS** (_rate_enable = 1):
     *          - rate_pid: AC_PID object containing:
     *            * P: Proportional gain (rate error to servo)
     *            * I: Integral gain (accumulated rate error to servo)
     *            * D: Derivative gain (rate of change of error to servo)
     *            * FF: Feedforward gain (desired rate to servo)
     *            * tau: D-term filter time constant (seconds)
     *            * imax: Integrator limit (centidegrees)
     *            * rmax: Slew rate limit (centidegrees/second)
     *            * filt_E_hz, filt_D_hz: Error and D-term filter frequencies
     * 
     * @note Parameters stored in EEPROM and configurable via ground station.
     * @note Changing _rate_enable in flight causes control mode switch with transients.
     * @note Rate PID parameters follow same structure as roll/pitch controllers.
     */
    static const struct AP_Param::GroupInfo var_info[];

private:
    //
    // PARAMETER STORAGE
    //
    
    /// Reference to vehicle-specific parameters (airspeed limits, control surface limits)
    const AP_FixedWing &aparm;
    
    /// Sideslip integrator gain (sideslip mode) - scales sideslip error into integrator
    AP_Float _K_A;
    
    /// Sideslip integral output gain (sideslip mode) - scales integrator in final output
    AP_Float _K_I;
    
    /// Yaw damping gain (sideslip mode) - Dutch roll damping via high-pass filtered yaw rate
    AP_Float _K_D;
    
    /// Feedforward gain (sideslip mode) - direct pilot rudder input feedthrough
    AP_Float _K_FF;
    
    /// Sideslip integrator limit in centidegrees (sideslip mode) - prevents wind-up
    AP_Int16 _imax;
    
    /// Control mode selector: 0 = sideslip damping mode, 1 = rate control mode
    AP_Int8  _rate_enable;
    
    /// Rate PID controller (rate control mode) - initialized with default gains:
    /// P=0.04, I=0.15, D=0, FF=0.15, tau=0.666s, imax=3°, rmax=0, filt_E=12Hz, filt_D=150Hz, notch=1
    AC_PID rate_pid{0.04, 0.15, 0, 0.15, 0.666, 3, 0, 12, 150, 1};

    //
    // CONTROL STATE VARIABLES
    //
    
    /// Timestamp of last controller update in microseconds (for dt calculation)
    uint32_t _last_t;
    
    /// Last servo output in centidegrees (for rate limiting and filtering)
    float _last_out;
    
    /// High-pass filter output state for yaw damping (sideslip mode) - previous filtered yaw rate
    float _last_rate_hp_out;
    
    /// High-pass filter input state for yaw damping (sideslip mode) - previous raw yaw rate
    float _last_rate_hp_in;
    
    /// Previous yaw damping gain value (for detecting parameter changes requiring reset)
    float _K_D_last;

    /// Sideslip integrator accumulator in centidegrees (sideslip mode) - ∫(sideslip error)
    float _integrator;

    //
    // AUTOTUNE SUPPORT (rate control mode only)
    //
    
    /// Saved gains structure for autotune backup/restore
    AP_AutoTune::ATGains gains;
    
    /// Pointer to autotune object (allocated during autotune_start(), nullptr when inactive)
    AP_AutoTune *autotune;
    
    /// Flag indicating autotune allocation failure (prevents repeated allocation attempts)
    bool failed_autotune_alloc;
    
    /// Rate PID diagnostic info for logging and telemetry (updated every control cycle)
    AP_PIDInfo _pid_info;
};
