#pragma once

/**
 * @file AC_PID.h
 * @brief General-purpose PID controller for ArduPilot flight control (attitude/rate controllers)
 * 
 * @details This file implements an advanced PID controller specifically designed for
 * copter attitude and rate control systems. It provides features beyond the basic PID
 * library (libraries/PID/PID.h), including:
 * - Three-stage filtering: input/target filtering (FLTT), error filtering (FLTE), 
 *   and derivative filtering (FLTD)
 * - Input/error/derivative low-pass filters to reduce noise and prevent oscillations
 * - Slew rate limiting (SMAX, SRTAU) to limit control output slew rate and reduce 
 *   high-frequency oscillations
 * - Optional notch filters for target and error signals (when AP_FILTER_ENABLED) 
 *   to reject specific disturbance frequencies
 * - Derivative feedforward (D_FF) proportional to rate of change of target for 
 *   improved tracking
 * - PD sum maximum (PDMX) to limit combined P+D output before adding integrator
 * - Directional integrator limiting (anti-windup) to prevent integrator growth 
 *   when at limits
 * - EEPROM-backed parameter storage via AP_Param for persistent gain configuration
 * - Integration with AutoTune system for automated gain tuning
 * - Comprehensive logging support via AP_PIDInfo structure
 * 
 * This controller is used extensively in AC_AttitudeControl for rate and angle control,
 * and in AC_PosControl for velocity control loops. It is typically called at the main
 * loop rate (400Hz for copter).
 * 
 * @note Square root controller functionality is NOT part of AC_PID - that is implemented
 * in AC_P_1D/AC_P_2D position controllers.
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <stdlib.h>
#include <cmath>
#include <Filter/SlewLimiter.h>
#include <Filter/NotchFilter.h>
#include <Filter/AP_Filter.h>

#define AC_PID_TFILT_HZ_DEFAULT  0.0f   // default input filter frequency
#define AC_PID_EFILT_HZ_DEFAULT  0.0f   // default input filter frequency
#define AC_PID_DFILT_HZ_DEFAULT  20.0f  // default input filter frequency
#define AC_PID_RESET_TC          0.16f  // Time constant for integrator reset decay to zero

#include "AP_PIDInfo.h"

/**
 * @class AC_PID
 * @brief Advanced PID controller for copter attitude and rate control
 * 
 * @details AC_PID implements a sophisticated PID controller with multiple advanced
 * features for flight control applications. The controller architecture includes:
 * 
 * **Filter Stages:**
 * - Target low-pass filter (FLTT): Filters the desired target/setpoint to reduce 
 *   step response overshoot
 * - Error low-pass filter (FLTE): Filters the error signal (target - measurement) 
 *   to reduce noise amplification
 * - Derivative low-pass filter (FLTD): Filters the derivative term to prevent 
 *   noise-induced high-frequency oscillations
 * - Optional notch filters: Target and error notch filters (when AP_FILTER_ENABLED) 
 *   can reject specific disturbance frequencies such as motor/propeller harmonics
 * 
 * **Slew Rate Limiter:**
 * The slew rate limiter (configured via SMAX and SRTAU parameters) limits the rate
 * of change of the controller output to reduce high-frequency oscillations and
 * prevent actuator saturation. The limiter dynamically reduces controller gain but
 * never below 10% of nominal gain to maintain control authority.
 * 
 * **Derivative Feedforward:**
 * Derivative feedforward (D_FF) adds a term proportional to the rate of change of
 * the target, improving tracking performance for time-varying setpoints.
 * 
 * **PD Sum Limiting:**
 * PD sum maximum (PDMX) limits the combined P+D output before adding the integrator
 * term, preventing excessive control output during transients while maintaining
 * integral action for steady-state tracking.
 * 
 * **Anti-Windup:**
 * Directional integrator limiting prevents integrator wind-up when the controller
 * output is saturated. When the `limit` flag is set (typically from motor mixing),
 * the integrator is allowed to shrink (reducing its contribution) but not grow.
 * 
 * **Usage Pattern:**
 * The controller is typically called at the main loop rate (400Hz for copter) from
 * AC_AttitudeControl for rate/angle control or AC_PosControl for velocity control.
 * 
 * Example usage:
 * @code
 * AC_PID rate_controller(0.15f, 0.1f, 0.004f, 0.0f, 0.5f, 20.0f, 20.0f, 20.0f);
 * float dt = 0.0025f;  // 400Hz update rate
 * float output = rate_controller.update_all(desired_rate, actual_rate, dt, limit_flag);
 * @endcode
 * 
 * @warning Modifying rate limits or filter frequencies can significantly affect
 * vehicle stability. Always test changes in SITL before flight testing.
 * 
 * @warning SMAX (slew rate maximum) should typically be set to ≤25% of actuator
 * maximum slew rate to prevent actuator saturation.
 * 
 * @warning This controller is NOT thread-safe by itself. In vehicle code, access
 * must be protected using WITH_SEMAPHORE when called from multiple threads.
 * 
 * @note Square root controller is NOT implemented in AC_PID. Square root control
 * is implemented in AC_P_1D and AC_P_2D position controllers.
 * 
 * @note Filter initialization occurs on the first call to update_all() after
 * reset_filter() is called, using the first input value.
 * 
 * @note The var_info[] parameter table is defined in AC_PID.cpp for AP_Param
 * parameter registration and EEPROM persistence.
 * 
 * @see AC_AttitudeControl Uses AC_PID for rate and angle controllers
 * @see AC_PosControl Uses AC_PID for velocity controllers
 * @see AC_AutoTune Automated tuning system that uses AC_PID
 * @see PID Basic PID library (libraries/PID/PID.h) for comparison
 * @see Filter Low-pass filter implementations
 * @see SlewLimiter Slew rate limiting implementation
 */
class AC_PID {
public:

    /**
     * @brief Default parameter values for AC_PID initialization
     * 
     * @details This structure holds default gain and filter values used during
     * controller initialization. Values are used as AP_Param defaults and can be
     * overridden from EEPROM or via parameter updates.
     */
    struct Defaults {
        float p;            ///< Proportional gain (dimensionless, typical range 0.01-1.0)
        float i;            ///< Integral gain (1/s, typical range 0.01-1.0)
        float d;            ///< Derivative gain (s, typical range 0.001-0.01)
        float ff;           ///< Feedforward gain (dimensionless, typical range 0.0-1.0)
        float imax;         ///< Maximum integrator contribution (same units as output, typical 0.1-1.0)
        float filt_T_hz;    ///< Target low-pass filter frequency (Hz, 0=disabled, typical 10-50 Hz)
        float filt_E_hz;    ///< Error low-pass filter frequency (Hz, 0=disabled, typical 10-50 Hz)
        float filt_D_hz;    ///< Derivative low-pass filter frequency (Hz, typical 10-50 Hz)
        float srmax;        ///< Slew rate maximum (output_units/s, 0=disabled, typical 25-200)
        float srtau;        ///< Slew rate time constant tau (s, typical 0.5-2.0)
        float dff;          ///< Derivative feedforward gain (dimensionless, typical 0.0-0.5)
    };

    /**
     * @brief Constructor for PID controller with explicit initial values
     * 
     * @details Constructs a PID controller with EEPROM-backed parameter storage via AP_Param.
     * Initial values are used as defaults for AP_Param initialization. Actual parameter
     * values will be loaded from EEPROM if previously saved, otherwise defaults are used.
     * 
     * @param[in] initial_p Proportional gain (dimensionless, typical 0.01-1.0)
     * @param[in] initial_i Integral gain (1/s, typical 0.01-1.0)
     * @param[in] initial_d Derivative gain (s, typical 0.001-0.01)
     * @param[in] initial_ff Feedforward gain (dimensionless, typical 0.0-1.0)
     * @param[in] initial_imax Maximum integrator value (same units as output, typical 0.1-1.0)
     * @param[in] initial_filt_T_hz Target filter cutoff frequency (Hz, 0=disabled, typical 10-50 Hz)
     * @param[in] initial_filt_E_hz Error filter cutoff frequency (Hz, 0=disabled, typical 10-50 Hz)
     * @param[in] initial_filt_D_hz Derivative filter cutoff frequency (Hz, typical 10-50 Hz)
     * @param[in] initial_srmax Slew rate maximum (output_units/s, 0=disabled, default=0, typical 25-200)
     * @param[in] initial_srtau Slew rate time constant tau (s, default=1.0, typical 0.5-2.0)
     * @param[in] initial_dff Derivative feedforward gain (dimensionless, default=0, typical 0.0-0.5)
     * 
     * @note Parameters are initialized from defaults or loaded from EEPROM at runtime.
     * @note Initial values are stored as const members for reference.
     * @note All filters are initialized to zero state; reset_filter() is called on first update.
     */
    AC_PID(float initial_p, float initial_i, float initial_d, float initial_ff, float initial_imax, float initial_filt_T_hz, float initial_filt_E_hz, float initial_filt_D_hz,
           float initial_srmax=0, float initial_srtau=1.0, float initial_dff=0);
    
    /**
     * @brief Constructor using Defaults structure
     * 
     * @details Convenience constructor that accepts a Defaults structure containing
     * all initialization parameters. Delegates to the main constructor.
     * 
     * @param[in] defaults Structure containing all PID default parameters
     * 
     * @see AC_PID::Defaults
     */
    AC_PID(const AC_PID::Defaults &defaults) :
        AC_PID(
            defaults.p,
            defaults.i,
            defaults.d,
            defaults.ff,
            defaults.imax,
            defaults.filt_T_hz,
            defaults.filt_E_hz,
            defaults.filt_D_hz,
            defaults.srmax,
            defaults.srtau,
            defaults.dff
            )
        { }

    CLASS_NO_COPY(AC_PID);

    /**
     * @brief Computes PID output from target setpoint and measurement feedback
     * 
     * @details This is the primary update method for the PID controller. It performs the
     * complete control algorithm in the following sequence:
     * 
     * 1. Filter target through low-pass and optional notch filters (FLTT, target_notch)
     * 2. Compute error = filtered_target - measurement
     * 3. Filter error through low-pass and optional notch filters (FLTE, error_notch)
     * 4. Compute derivative of filtered error
     * 5. Filter derivative through low-pass filter (FLTD)
     * 6. Update integrator with anti-windup (respects `limit` flag)
     * 7. Compute derivative feedforward from target rate of change
     * 8. Apply slew rate limiter to total output
     * 9. Clamp combined P+D output to PDMX limit
     * 10. Compute final output = (P * pd_scale) + I + (D * pd_scale) + FF + DFF
     * 
     * The method handles filter initialization on first call after reset_filter() by
     * setting all filter states to the initial input values.
     * 
     * @param[in] target Desired target value (e.g., desired rate in deg/s or rad/s, desired angle in degrees or radians)
     * @param[in] measurement Actual measured value (same units as target)
     * @param[in] dt Time step since last update (seconds, typically 0.0025s for 400Hz)
     * @param[in] limit Anti-windup flag - if true, integrator can only shrink (not grow) to prevent wind-up at saturation
     * @param[in] pd_scale Scaling factor applied to P and D terms (dimensionless, default=1.0, typical range 0.5-1.0)
     * 
     * @return Combined PID output (same units as error × gains)
     * 
     * @note This method is typically called at main loop rate (400Hz for copter)
     * @note Filter initialization occurs on first call after reset_filter()
     * @note Slew limiter reduces effective gain dynamically but never below 10% of nominal
     * @note The dt parameter is critical for correct integration and filtering - must be accurate
     * 
     * @warning Incorrect dt values will cause incorrect integration and derivative calculations
     * @warning This method is NOT thread-safe - use WITH_SEMAPHORE in calling code
     * 
     * @see update_error() Legacy error-only update method (deprecated)
     * @see reset_filter() Call before first update or when discontinuity occurs
     * @see reset_I() Call to zero integrator between mode changes
     */
    float update_all(float target, float measurement, float dt, bool limit = false, float pd_scale = 1.0f);

    /**
     * @brief Computes PID output from error signal only (legacy method)
     * 
     * @details This method computes PID output when only the error signal is available
     * (target assumed to be zero). It applies error filtering, calculates derivative,
     * and updates the integrator. This is a legacy interface maintained for backward
     * compatibility.
     * 
     * @param[in] error Error signal (target - measurement), same units as target/measurement
     * @param[in] dt Time step since last update (seconds, typically 0.0025s for 400Hz)
     * @param[in] limit Anti-windup flag - if true, integrator can only shrink (not grow)
     * 
     * @return Combined PID output (same units as error × gains)
     * 
     * @note Target and measurement values must be set separately using set_target_rate()
     * and set_actual_rate() for proper logging support.
     * 
     * @warning This method is deprecated. Use update_all() for new code.
     * 
     * @todo Remove this function when it is no longer used in vehicle code
     * 
     * @see update_all() Preferred update method with explicit target and measurement
     */
    float update_error(float error, float dt, bool limit = false);

    /**
     * @brief Get proportional term contribution from last update
     * @return Proportional term value (output units, typically same units as controller output)
     * @note Returns P term = kP × error after filtering and scaling
     */
    float get_p() const;
    
    /**
     * @brief Get integral term contribution from last update
     * @return Integral term value (output units, clamped to ±IMAX)
     * @note Returns current integrator value, accumulated over time
     */
    float get_i() const;
    
    /**
     * @brief Get derivative term contribution from last update
     * @return Derivative term value (output units)
     * @note Returns D term = kD × derivative(error) after filtering and scaling
     */
    float get_d() const;
    
    /**
     * @brief Get feedforward term contribution from last update
     * @return Feedforward term value (output units)
     * @note Returns FF term = kFF × target (unfiltered target value)
     */
    float get_ff() const;

    /**
     * @brief Reset integrator to zero
     * 
     * @details Fully zeros the integrator term. This should be called when switching
     * between flight modes or during initialization to prevent integrator wind-up from
     * carrying over between different control contexts.
     * 
     * @note Sets the _I_set flag to indicate integrator has been explicitly set
     * @note Does not affect P, D, or FF terms
     * 
     * @see set_integrator() To set integrator to a specific value
     * @see relax_integrator() To gradually adjust integrator
     */
    void reset_I();

    // Flags the input filter for reset. The next call to `update_all()` will reinitialize the filter using the next input.
    void reset_filter() {
        _flags._reset_filter = true;
    }

    /**
     * @brief Load controller parameters from EEPROM
     * 
     * @details Loads all PID gains and filter frequencies from EEPROM storage via
     * AP_Param system. This method is typically called automatically during parameter
     * system initialization.
     * 
     * @note Currently not used in vehicle code - AP_Param handles loading automatically
     * @note Included for completeness and potential future use
     */
    void load_gains();

    /**
     * @brief Save controller parameters to EEPROM
     * 
     * @details Saves all PID gains and filter frequencies to EEPROM storage via
     * AP_Param system. This method is used by AutoTune to save tuned gains after
     * successful tuning completion, and can be called manually to persist parameter
     * changes.
     * 
     * @note AutoTune calls this method to save gains before starting tuning
     * @note Writes to EEPROM - limit frequency to avoid excessive wear
     * 
     * @see AC_AutoTune Uses save_gains() to preserve gains before/after tuning
     */
    void save_gains();

    // Gain and parameter accessors - return AP_Float references for parameter system
    
    /** @brief Get proportional gain (const reference) @return kP parameter (dimensionless) */
    const AP_Float &kP() const { return _kp; }
    
    /** @brief Get proportional gain (mutable reference) @return kP parameter (dimensionless) */
    AP_Float &kP() { return _kp; }
    
    /** @brief Get integral gain @return kI parameter (1/s) */
    AP_Float &kI() { return _ki; }
    
    /** @brief Get derivative gain @return kD parameter (s) */
    AP_Float &kD() { return _kd; }
    
    /** @brief Get integrator maximum limit @return IMAX parameter (output units) */
    AP_Float &kIMAX() { return _kimax; }
    
    /** @brief Get PD sum maximum limit @return PDMAX parameter (output units) */
    AP_Float &kPDMAX() { return _kpdmax; }
    
    /** @brief Get feedforward gain @return FF parameter (dimensionless) */
    AP_Float &ff() { return _kff;}
    
    /** @brief Get target filter frequency @return FLTT parameter (Hz, 0=disabled) */
    AP_Float &filt_T_hz() { return _filt_T_hz; }
    
    /** @brief Get error filter frequency @return FLTE parameter (Hz, 0=disabled) */
    AP_Float &filt_E_hz() { return _filt_E_hz; }
    
    /** @brief Get derivative filter frequency @return FLTD parameter (Hz) */
    AP_Float &filt_D_hz() { return _filt_D_hz; }
    
    /** @brief Get slew rate maximum @return SMAX parameter (output_units/s, 0=disabled) */
    AP_Float &slew_limit() { return _slew_rate_max; }
    
    /** @brief Get derivative feedforward gain @return D_FF parameter (dimensionless) */
    AP_Float &kDff() { return _kdff; }

    /** @brief Get integrator maximum as float value @return IMAX value (output units) */
    float imax() const { return _kimax.get(); }
    
    /** @brief Get PD maximum as float value @return PDMAX value (output units) */
    float pdmax() const { return _kpdmax.get(); }

    // Returns alpha value for the target low-pass filter (based on filter frequency and dt)
    float get_filt_T_alpha(float dt) const;
    // Returns alpha value for the error low-pass filter (based on filter frequency and dt)
    float get_filt_E_alpha(float dt) const;
    // Returns alpha value for the derivative low-pass filter (based on filter frequency and dt)
    float get_filt_D_alpha(float dt) const;

    // Parameter setter methods with validation
    
    /** @brief Set proportional gain @param[in] v kP value (dimensionless, typically 0.01-1.0) */
    void set_kP(const float v) { _kp.set(v); }
    
    /** @brief Set integral gain @param[in] v kI value (1/s, typically 0.01-1.0) */
    void set_kI(const float v) { _ki.set(v); }
    
    /** @brief Set derivative gain @param[in] v kD value (s, typically 0.001-0.01) */
    void set_kD(const float v) { _kd.set(v); }
    
    /** @brief Set feedforward gain @param[in] v FF value (dimensionless, typically 0.0-1.0) */
    void set_ff(const float v) { _kff.set(v); }
    
    /** @brief Set integrator maximum limit @param[in] v IMAX value (output units, uses fabsf to ensure positive) */
    void set_imax(const float v) { _kimax.set(fabsf(v)); }
    
    /** @brief Set PD sum maximum limit @param[in] v PDMAX value (output units, uses fabsf to ensure positive) */
    void set_pdmax(const float v) { _kpdmax.set(fabsf(v)); }
    
    /**
     * @brief Set target filter cutoff frequency
     * @param[in] v FLTT frequency (Hz, 0=disabled, typically 10-50 Hz)
     * @note Triggers filter reset on next update_all() call
     */
    void set_filt_T_hz(const float v);
    
    /**
     * @brief Set error filter cutoff frequency
     * @param[in] v FLTE frequency (Hz, 0=disabled, typically 10-50 Hz)
     * @note Triggers filter reset on next update_all() call
     */
    void set_filt_E_hz(const float v);
    
    /**
     * @brief Set derivative filter cutoff frequency
     * @param[in] v FLTD frequency (Hz, typically 10-50 Hz)
     * @note Triggers filter reset on next update_all() call
     */
    void set_filt_D_hz(const float v);
    
    /**
     * @brief Set slew rate maximum
     * @param[in] v SMAX value (output_units/s, 0=disabled, typically 25-200)
     * @warning Should be ≤25% of actuator maximum slew rate to prevent saturation
     */
    void set_slew_limit(const float v);
    
    /** @brief Set derivative feedforward gain @param[in] v D_FF value (dimensionless, typically 0.0-0.5) */
    void set_kDff(const float v) { _kdff.set(v); }

    /**
     * @brief Set target value for external logging
     * @param[in] target Desired target value (for logging/telemetry only)
     * @note This is an optional method for injecting target value into logging structure
     * when using update_error() instead of update_all()
     */
    void set_target_rate(float target) { _pid_info.target = target; }
    
    /**
     * @brief Set actual measurement value for external logging
     * @param[in] actual Measured actual value (for logging/telemetry only)
     * @note This is an optional method for injecting measurement value into logging structure
     * when using update_error() instead of update_all()
     */
    void set_actual_rate(float actual) { _pid_info.actual = actual; }

    /**
     * @brief Set integrator to a specific value
     * 
     * @details Sets the integrator term directly to the specified value, clamped to ±IMAX
     * bounds to prevent excessive integral action. This is useful for smooth transitions
     * between controllers or for initializing the integrator to a known good value.
     * 
     * @param[in] i Desired integrator value (output units)
     * 
     * @note The value is clamped to [-IMAX, +IMAX]
     * @note Sets the _I_set flag to indicate integrator has been explicitly set
     * 
     * @see reset_I() To zero the integrator
     * @see relax_integrator() To gradually adjust the integrator
     */
    void set_integrator(float i);

    /**
     * @brief Gradually adjust integrator toward a desired value
     * 
     * @details Smoothly transitions the integrator from its current value toward a target
     * value using exponential decay with the specified time constant. This is typically
     * used to "relax" the integrator during dynamic maneuvers or transitions to prevent
     * sudden control output changes.
     * 
     * Algorithm: integrator += (target - integrator) × (dt / time_constant)
     * 
     * @param[in] integrator Target integrator value (output units)
     * @param[in] dt Time step (seconds, typically 0.0025s for 400Hz)
     * @param[in] time_constant Time constant for exponential approach (seconds, typical 0.1-1.0)
     * 
     * @note Shorter time constants result in faster convergence
     * @note Final value is clamped to ±IMAX bounds
     * @note Sets the _I_set flag to indicate integrator has been modified
     * 
     * @see set_integrator() For immediate integrator setting
     */
    void relax_integrator(float integrator, float dt, float time_constant);

    /**
     * @brief Set slew limiter scale factor
     * 
     * @details Sets a scale factor for the slew rate limiter, allowing dynamic adjustment
     * of slew limiting strength. Scale is applied as a percentage (0-100).
     * 
     * @param[in] scale Scale factor (int8_t, typically 0-100 representing percentage)
     * 
     * @note Used to dynamically adjust slew rate limiting during flight
     */
    void set_slew_limit_scale(int8_t scale) { _slew_limit_scale = scale; }

    /**
     * @brief Get current slew rate from limiter
     * 
     * @details Returns the instantaneous slew rate being applied by the slew limiter.
     * Useful for monitoring and debugging slew rate limiting behavior.
     * 
     * @return Current slew rate (output_units/s), or 0 if SMAX=0 (disabled)
     * 
     * @note Returns 0 when slew limiting is disabled (SMAX = 0)
     */
    float get_slew_rate(void) const { return _slew_limiter.get_slew_rate(); }

    /**
     * @brief Get PID telemetry/logging information
     * 
     * @details Returns a reference to the AP_PIDInfo structure containing all current
     * PID state information for logging and telemetry purposes. This structure includes
     * target, actual, error, P, I, D, FF, and other internal state values.
     * 
     * @return Const reference to AP_PIDInfo telemetry structure
     * 
     * @note Updated on every call to update_all() or update_error()
     * @see AP_PIDInfo Structure definition for available fields
     */
    const AP_PIDInfo& get_pid_info(void) const { return _pid_info; }

    /**
     * @brief Configure notch filter sample rate
     * 
     * @details Configures optional notch filters for target and error signals using the
     * specified sample rate. Notch filters are dynamically allocated only when
     * AP_FILTER_ENABLED is defined and filter indices are configured. Used to reject
     * specific disturbance frequencies (e.g., motor/propeller harmonics).
     * 
     * @param[in] sample_rate Sample rate for notch filter configuration (Hz, typically main loop rate)
     * 
     * @note Filters are only active when AP_FILTER_ENABLED is defined
     * @note Notch filter parameters are configured via _notch_T_filter and _notch_E_filter indices
     * @note Dynamically allocates NotchFilterFloat objects if not already allocated
     * 
     * @see AP_Filter Notch filter configuration and management
     */
    void set_notch_sample_rate(float);

    // parameter var table
    static const struct AP_Param::GroupInfo var_info[];

protected:

    /**
     * @brief Update integrator term with anti-windup
     * 
     * @details Updates the integrator based on current filtered error and time step,
     * implementing directional anti-windup. When `limit` is true (typically set by
     * motor mixing saturation detection), the integrator is only allowed to shrink
     * (move toward zero) but not grow, preventing wind-up at actuator saturation.
     * 
     * @param[in] dt Time step since last update (seconds)
     * @param[in] limit Anti-windup flag - if true, integrator can only shrink
     * 
     * @note Integrator is clamped to [-IMAX, +IMAX] bounds
     * @note Called internally by update_all() and update_error()
     */
    void update_i(float dt, bool limit);

    // AP_Param-backed controller parameters (stored in EEPROM)
    AP_Float _kp;                ///< Proportional gain (dimensionless)
    AP_Float _ki;                ///< Integral gain (1/s)
    AP_Float _kd;                ///< Derivative gain (s)
    AP_Float _kff;               ///< Feedforward gain (dimensionless)
    AP_Float _kimax;             ///< Maximum integrator value (output units)
    AP_Float _kpdmax;            ///< Maximum PD sum before adding integrator (output units)
    AP_Float _filt_T_hz;         ///< Target low-pass filter frequency (Hz, 0=disabled)
    AP_Float _filt_E_hz;         ///< Error low-pass filter frequency (Hz, 0=disabled)
    AP_Float _filt_D_hz;         ///< Derivative low-pass filter frequency (Hz)
    AP_Float _slew_rate_max;     ///< Slew rate maximum limit (output_units/s, 0=disabled)
    AP_Float _kdff;              ///< Derivative feedforward gain (dimensionless)
#if AP_FILTER_ENABLED
    AP_Int8 _notch_T_filter;     ///< Target notch filter index in AP_Filter database (0=disabled)
    AP_Int8 _notch_E_filter;     ///< Error notch filter index in AP_Filter database (0=disabled)
#endif

    AP_Float _slew_rate_tau;     ///< Slew rate time constant tau (s) - can be exposed by derived classes

    SlewLimiter _slew_limiter{_slew_rate_max, _slew_rate_tau}; ///< Slew rate limiter instance

    /**
     * @brief Controller state flags
     */
    struct ac_pid_flags {
        bool _reset_filter :1;  ///< True if filters should be reinitialized on next update_all()
        bool _I_set :1;         ///< True if integrator has been explicitly set (via set_integrator, reset_I, etc.)
    } _flags;

    // Internal controller state variables
    float _integrator;           ///< Current integrator value (output units)
    float _target;               ///< Filtered target value from previous update
    float _error;                ///< Filtered error value from previous update
    float _derivative;           ///< Filtered derivative value from previous update
    int8_t _slew_limit_scale;    ///< Slew limiter scale factor (percentage, 0-100)
    float _target_derivative;    ///< Rate of change of target (for derivative feedforward)
#if AP_FILTER_ENABLED
    NotchFilterFloat* _target_notch;  ///< Dynamically allocated target notch filter (nullptr if not used)
    NotchFilterFloat* _error_notch;   ///< Dynamically allocated error notch filter (nullptr if not used)
#endif

    AP_PIDInfo _pid_info;        ///< Telemetry structure for logging (target, actual, P, I, D, FF, etc.)

private:
    // Constructor default values (const for reference, used by AP_Param initialization)
    const float default_kp;              ///< Default proportional gain
    const float default_ki;              ///< Default integral gain
    const float default_kd;              ///< Default derivative gain
    const float default_kff;             ///< Default feedforward gain
    const float default_kdff;            ///< Default derivative feedforward gain
    const float default_kimax;           ///< Default integrator maximum
    const float default_filt_T_hz;       ///< Default target filter frequency
    const float default_filt_E_hz;       ///< Default error filter frequency
    const float default_filt_D_hz;       ///< Default derivative filter frequency
    const float default_slew_rate_max;   ///< Default slew rate maximum
};
