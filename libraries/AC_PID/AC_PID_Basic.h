#pragma once

/// @file	AC_PID_Basic.h
/// @brief	Lightweight PID controller with error and derivative filtering, integrator limit, and EEPROM gain storage.
///
/// @details This file implements AC_PID_Basic, a simplified single-axis PID controller designed for
///          resource-constrained control applications. It provides:
///          
///          - Proportional, Integral, Derivative, and Feedforward control components
///          - First-order low-pass filtering on error (E) and derivative (D) terms
///          - Integrator clamping with configurable maximum (IMAX)
///          - Directional anti-windup mechanism to prevent integrator growth when saturated
///          - EEPROM-backed parameter storage for runtime tuning persistence
///          
///          Key differences from AC_PID:
///          - Simpler implementation without slew rate limiting
///          - No notch filtering for harmonic rejection
///          - No square root controller for improved low-throttle response
///          - Lower computational overhead suitable for high-frequency control loops
///          
///          Typical use cases:
///          - Secondary control loops (yaw rate, altitude rate, etc.)
///          - High-frequency inner loops where CPU budget is constrained
///          - Simple single-axis control problems without complex dynamics
///          
///          Control equation: output = Kp*error + Ki*integrator + Kd*derivative + Kff*target
///          where error and derivative are filtered, and integrator is clamped to ±IMAX.

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include "AP_PIDInfo.h"

/// @class	AC_PID_Basic
/// @brief	Lightweight single-axis PID controller with filtered error and derivative terms
///
/// @details AC_PID_Basic implements a classical PID controller with the following architecture:
///          
///          **Control Components:**
///          - P (Proportional): Responds to current error (Kp * filtered_error)
///          - I (Integral): Accumulates error over time (Ki * integrator, clamped to ±IMAX)
///          - D (Derivative): Reacts to rate of change (Kd * filtered_derivative)
///          - FF (Feedforward): Direct response to target (Kff * target)
///          
///          **Filtering Approach:**
///          - Error filtering: First-order low-pass filter at filt_E_hz frequency
///          - Derivative filtering: First-order low-pass filter at filt_D_hz frequency
///          - Filters reduce noise sensitivity and improve stability
///          - Filter states can be reset via reset_filter() for mode transitions
///          
///          **Integrator Management:**
///          - Accumulates filtered error scaled by Ki and dt
///          - Clamped to ±IMAX to prevent excessive windup
///          - Directional anti-windup: can prevent growth when output saturates
///          - Can be reset to zero via reset_I() during mode changes
///          - Can be explicitly set via set_integrator() for bumpless transfer
///          
///          **Usage Pattern:**
///          1. Construct with initial gains (loaded from EEPROM if available)
///          2. Call update_all(target, measurement, dt) each control cycle
///          3. Apply directional limits if output is saturated
///          4. Reset filters/integrator during mode transitions as needed
///          5. Tune gains via kP(), kI(), kD(), ff() accessors or ground station
///          
///          **When to use AC_PID_Basic vs AC_PID:**
///          - Use AC_PID_Basic for simpler control loops where slew limiting and
///            notch filtering are not required
///          - Use AC_PID for primary attitude/position control loops requiring
///            advanced features like square root controller and harmonic rejection
///          
///          **Thread Safety:**
///          Not thread-safe. Must be called from a single task/thread.
///          
///          **Numeric Stability:**
///          Includes guards against NaN/Inf propagation in critical paths.
class AC_PID_Basic {
public:

    /// @brief Constructor for PID controller with EEPROM-backed parameter storage
    ///
    /// @details Initializes PID controller with default gain values. If parameters have been
    ///          previously saved to EEPROM via save_gains() or ground station configuration,
    ///          those values will override the initial values provided here during AP_Param
    ///          initialization. All filter states and integrator are initialized to zero.
    ///
    /// @param[in] initial_p      Initial proportional gain (dimensionless). Typical range: 0.1 to 10.0
    /// @param[in] initial_i      Initial integral gain (1/seconds). Typical range: 0.01 to 2.0
    /// @param[in] initial_d      Initial derivative gain (seconds). Typical range: 0.001 to 0.1
    /// @param[in] initial_ff     Initial feedforward gain (dimensionless). Typical range: 0.0 to 1.0
    /// @param[in] initial_imax   Initial integrator limit (same units as output). Typical range: 0.1 to 1.0
    /// @param[in] initial_filt_E_hz  Initial error filter cutoff frequency in Hz. Typical range: 5 to 30 Hz
    /// @param[in] initial_filt_D_hz  Initial derivative filter cutoff frequency in Hz. Typical range: 5 to 50 Hz
    ///
    /// @note All parameters are stored as AP_Float and can be modified at runtime via accessors
    /// @note Filter frequencies of 0 Hz disable filtering (no low-pass filter applied)
    AC_PID_Basic(float initial_p, float initial_i, float initial_d, float initial_ff, float initial_imax, float initial_filt_E_hz, float initial_filt_D_hz);

    /// @brief Compute complete PID output with filtering, integrator update, and optional anti-windup
    ///
    /// @details Performs full PID calculation in the following sequence:
    ///          1. Compute raw error: error_raw = target - measurement
    ///          2. Apply first-order low-pass filter to error at filt_E_hz frequency
    ///          3. Compute derivative of filtered error
    ///          4. Apply first-order low-pass filter to derivative at filt_D_hz frequency
    ///          5. Update integrator with filtered error (subject to directional limits and IMAX clamp)
    ///          6. Return total output: Kp*error + Ki*integrator + Kd*derivative + Kff*target
    ///
    ///          If filters are flagged for reset (via reset_filter()), they are initialized to
    ///          current values without filtering on this call.
    ///
    /// @param[in] target       Desired setpoint value (same units as measurement)
    /// @param[in] measurement  Current measured value (same units as target)
    /// @param[in] dt           Time step since last update in seconds. Typical: 0.0025s (400Hz) to 0.01s (100Hz)
    /// @param[in] limit        If true, integrator may shrink but not grow (directional anti-windup for saturation)
    ///
    /// @return Total PID output (same units as target/measurement, scaled by gains)
    ///
    /// @note Call this method at a consistent frequency for proper derivative calculation and filter behavior
    /// @note dt must be positive and reasonable (typically 0.001 to 0.1 seconds)
    /// @note If limit=true and output is saturated, integrator will not accumulate further error
    ///
    /// @warning Timing dt parameter is critical for stability. Inconsistent dt can cause instability.
    float update_all(float target, float measurement, float dt, bool limit = false) WARN_IF_UNUSED;
    
    /// @brief Compute complete PID output with separate negative and positive directional anti-windup limits
    ///
    /// @details Identical to update_all(target, measurement, dt, limit) but provides independent control
    ///          over integrator growth in negative and positive directions. This enables asymmetric
    ///          anti-windup for systems with different saturation characteristics in each direction.
    ///
    /// @param[in] target       Desired setpoint value (same units as measurement)
    /// @param[in] measurement  Current measured value (same units as target)
    /// @param[in] dt           Time step since last update in seconds
    /// @param[in] limit_neg    If true, integrator may only increase (prevents negative growth)
    /// @param[in] limit_pos    If true, integrator may only decrease (prevents positive growth)
    ///
    /// @return Total PID output (same units as target/measurement, scaled by gains)
    ///
    /// @note Use this variant when output saturation is directional (e.g., motor can't produce negative thrust)
    /// @note Both limit_neg and limit_pos can be true simultaneously, freezing integrator completely
    ///
    /// @warning Timing dt parameter is critical for stability. Inconsistent dt can cause instability.
    float update_all(float target, float measurement, float dt, bool limit_neg, bool limit_pos) WARN_IF_UNUSED;

    /// @brief Update integrator term with directional anti-windup control
    ///
    /// @details Accumulates the current filtered error into the integrator using the equation:
    ///          integrator += Ki * error * dt
    ///          
    ///          The integrator is then clamped to ±IMAX. Directional anti-windup prevents
    ///          integrator growth in directions where the output is saturated:
    ///          - If limit_neg is true and error would decrease integrator, no update occurs
    ///          - If limit_pos is true and error would increase integrator, no update occurs
    ///          
    ///          This method is called internally by update_all() but can also be called
    ///          independently for specialized control architectures.
    ///
    /// @param[in] dt         Time step in seconds for integrator accumulation
    /// @param[in] limit_neg  If true, prevents integrator from decreasing (anti-windup for negative saturation)
    /// @param[in] limit_pos  If true, prevents integrator from increasing (anti-windup for positive saturation)
    ///
    /// @note Uses current internal _error value (filtered error from last update_all call)
    /// @note Includes numeric guard: skips update if error or dt contains NaN or Inf
    /// @note Both limits can be true simultaneously, completely freezing the integrator
    void update_i(float dt, bool limit_neg, bool limit_pos);

    /// @brief Get proportional component of PID output
    /// @return Proportional term (Kp * filtered_error) in same units as total output
    /// @note Value is from the most recent update_all() call
    float get_p() const WARN_IF_UNUSED { return _error * _kp; }
    
    /// @brief Get integral component of PID output
    /// @return Integrator value (accumulated error scaled by Ki) in same units as total output
    /// @note Value is clamped to ±IMAX and subject to directional anti-windup
    float get_i() const WARN_IF_UNUSED { return _integrator; }
    
    /// @brief Get derivative component of PID output
    /// @return Derivative term (Kd * filtered_derivative) in same units as total output
    /// @note Derivative is of the filtered error, not raw error, reducing noise sensitivity
    float get_d() const WARN_IF_UNUSED { return _derivative * _kd; }
    
    /// @brief Get feedforward component of PID output
    /// @return Feedforward term (Kff * target) in same units as total output
    /// @note Feedforward provides direct response to target without waiting for error accumulation
    float get_ff() const WARN_IF_UNUSED { return _target * _kff; }
    
    /// @brief Get current filtered error value
    /// @return Filtered error (target - measurement after low-pass filtering) in same units as input
    /// @note This is the error value used for P term calculation
    float get_error() const WARN_IF_UNUSED { return _error; }

    /// @brief Reset integrator to zero
    ///
    /// @details Clears accumulated integral term, typically called during:
    ///          - Flight mode transitions
    ///          - Arming/disarming
    ///          - Significant setpoint changes
    ///          - Recovery from saturation conditions
    ///
    /// @note Use this to prevent integrator windup from affecting new control regime
    /// @note Filter states (error and derivative) are NOT reset by this function
    void reset_I();

    /// @brief Flag filters for reset on next update_all() call
    ///
    /// @details Sets internal flag causing error and derivative filters to initialize to
    ///          current values on the next update_all() call, bypassing filtering for one cycle.
    ///          This prevents transients when switching between control modes or after
    ///          discontinuous setpoint changes.
    ///
    /// @note Filter reset occurs automatically on next update_all(), then flag clears
    /// @note Typically called in conjunction with reset_I() during mode transitions
    /// @note Does not immediately reset filters; reset deferred until next update
    void reset_filter() { _reset_filter = true; }

    /// @brief Save current controller gains and filter parameters to EEPROM
    ///
    /// @details Persists all tunable parameters (Kp, Ki, Kd, Kff, IMAX, filt_E_hz, filt_D_hz)
    ///          to EEPROM for retention across power cycles. Parameters can be modified via
    ///          ground station or programmatically via set_* methods, then saved.
    ///
    /// @note Typically called after tuning is complete to preserve optimized gains
    /// @note EEPROM writes are limited; avoid calling this in fast loops
    /// @note Comment indicates this may not be actively used in current implementation
    void save_gains();

    /// @brief Get reference to proportional gain parameter
    /// @return Reference to Kp parameter (dimensionless) for reading or modification
    /// @note Modifying returned reference immediately affects control output
    AP_Float &kP() WARN_IF_UNUSED { return _kp; }
    
    /// @brief Get reference to integral gain parameter
    /// @return Reference to Ki parameter (1/seconds) for reading or modification
    /// @note Higher Ki increases integrator accumulation rate and steady-state error rejection
    AP_Float &kI() WARN_IF_UNUSED { return _ki; }
    
    /// @brief Get reference to derivative gain parameter
    /// @return Reference to Kd parameter (seconds) for reading or modification
    /// @note Higher Kd increases damping but also amplifies high-frequency noise
    AP_Float &kD() WARN_IF_UNUSED { return _kd; }
    
    /// @brief Get reference to feedforward gain parameter
    /// @return Reference to Kff parameter (dimensionless) for reading or modification
    /// @note Feedforward provides immediate response to setpoint changes without waiting for error
    AP_Float &ff() WARN_IF_UNUSED { return _kff;}
    
    /// @brief Get reference to error filter cutoff frequency parameter
    /// @return Reference to error filter frequency in Hz for reading or modification
    /// @note Lower frequencies provide more filtering but increase phase lag. Typical: 5-30 Hz
    AP_Float &filt_E_hz() WARN_IF_UNUSED { return _filt_E_hz; }
    
    /// @brief Get reference to derivative filter cutoff frequency parameter
    /// @return Reference to derivative filter frequency in Hz for reading or modification
    /// @note Lower frequencies reduce derivative noise but may slow damping response. Typical: 5-50 Hz
    AP_Float &filt_D_hz() WARN_IF_UNUSED { return _filt_D_hz; }
    
    /// @brief Get integrator maximum limit value
    /// @return Current IMAX value (same units as output) - maximum absolute integrator value
    /// @note Integrator is clamped to ±imax() to prevent excessive windup
    float imax() const WARN_IF_UNUSED { return _kimax.get(); }
    
    /// @brief Calculate error filter alpha coefficient for given time step
    /// @param[in] dt Time step in seconds
    /// @return Filter alpha coefficient (0 to 1) for first-order low-pass filter
    /// @note Alpha = 1 - exp(-2*pi*filt_E_hz*dt). Used internally by update_all()
    float get_filt_E_alpha(float dt) const WARN_IF_UNUSED;
    
    /// @brief Calculate derivative filter alpha coefficient for given time step
    /// @param[in] dt Time step in seconds
    /// @return Filter alpha coefficient (0 to 1) for first-order low-pass filter
    /// @note Alpha = 1 - exp(-2*pi*filt_D_hz*dt). Used internally by update_all()
    float get_filt_D_alpha(float dt) const WARN_IF_UNUSED;

    /// @brief Set proportional gain
    /// @param[in] v New Kp value (dimensionless). Typical range: 0.1 to 10.0
    /// @note Takes effect immediately on next update_all() call
    void set_kP(float v) { _kp.set(v); }
    
    /// @brief Set integral gain
    /// @param[in] v New Ki value (1/seconds). Typical range: 0.01 to 2.0
    /// @note Higher values increase steady-state error correction but may cause overshoot
    void set_kI(float v) { _ki.set(v); }
    
    /// @brief Set derivative gain
    /// @param[in] v New Kd value (seconds). Typical range: 0.001 to 0.1
    /// @note Higher values increase damping but amplify noise. Use with appropriate derivative filtering
    void set_kD(float v) { _kd.set(v); }
    
    /// @brief Set feedforward gain
    /// @param[in] v New Kff value (dimensionless). Typical range: 0.0 to 1.0
    /// @note Feedforward of 1.0 provides full direct response to target changes
    void set_ff(float v) { _kff.set(v); }
    
    /// @brief Set integrator maximum limit
    /// @param[in] v New IMAX value (same units as output). Always stored as absolute value
    /// @note Integrator is clamped to ±v. Prevents excessive windup
    void set_imax(float v) { _kimax.set(fabsf(v)); }
    
    /// @brief Set error filter cutoff frequency
    /// @param[in] hz New error filter frequency in Hz. Always stored as absolute value
    /// @note 0 Hz disables filtering. Lower frequencies increase filtering and phase lag
    void set_filt_E_hz(float hz) { _filt_E_hz.set(fabsf(hz)); }
    
    /// @brief Set derivative filter cutoff frequency
    /// @param[in] hz New derivative filter frequency in Hz. Always stored as absolute value
    /// @note 0 Hz disables filtering. Lower frequencies reduce noise but may slow damping response
    void set_filt_D_hz(float hz) { _filt_D_hz.set(fabsf(hz)); }

    /// @brief Set integrator value directly given target, measurement, and desired integrator
    ///
    /// @details Sets integrator to specified value while also updating internal target and error states.
    ///          This enables bumpless transfer when switching control modes or during initialization.
    ///          The integrator is clamped to ±IMAX after being set.
    ///
    /// @param[in] target       Current target setpoint (same units as measurement)
    /// @param[in] measurement  Current measured value (same units as target)
    /// @param[in] i            Desired integrator value (same units as output)
    ///
    /// @note Useful for initializing integrator to compensate for known steady-state offsets
    /// @note Internally computes error = target - measurement and stores for next update
    void set_integrator(float target, float measurement, float i);
    
    /// @brief Set integrator value directly given error and desired integrator
    ///
    /// @details Sets integrator to specified value and updates internal error state.
    ///          The integrator is clamped to ±IMAX after being set.
    ///
    /// @param[in] error  Current error value (target - measurement, same units as input)
    /// @param[in] i      Desired integrator value (same units as output)
    ///
    /// @note Use when error is already computed and target is not needed
    void set_integrator(float error, float i);
    
    /// @brief Set integrator value directly
    ///
    /// @details Sets integrator to specified value, clamped to ±IMAX. Does not update
    ///          target or error states. Simplest form for direct integrator manipulation.
    ///
    /// @param[in] i  Desired integrator value (same units as output)
    ///
    /// @note Most common form for simple integrator initialization or reset to non-zero value
    /// @note Integrator is automatically clamped to ±IMAX to maintain constraints
    void set_integrator(float i);

    /// @brief Get PID information structure for logging and telemetry
    ///
    /// @return Reference to AP_PIDInfo structure containing P, I, D, FF components and other diagnostic data
    ///
    /// @note Used by logging system to record PID performance for post-flight analysis
    /// @note Updated internally by update_all() each control cycle
    const AP_PIDInfo& get_pid_info(void) const WARN_IF_UNUSED { return _pid_info; }

    // parameter var table
    static const struct AP_Param::GroupInfo var_info[];

protected:

    // EEPROM-backed tunable parameters - persisted across power cycles
    AP_Float _kp;                ///< Proportional gain (dimensionless). Scales current error to output
    AP_Float _ki;                ///< Integral gain (1/seconds). Scales accumulated error to output
    AP_Float _kd;                ///< Derivative gain (seconds). Scales rate of error change to output
    AP_Float _kff;               ///< Feedforward gain (dimensionless). Scales target directly to output
    AP_Float _kimax;             ///< Integrator limit (output units). Integrator clamped to ±_kimax
    AP_Float _filt_E_hz;         ///< Error filter cutoff frequency in Hz. 0 = no filtering. Typical: 5-30 Hz
    AP_Float _filt_D_hz;         ///< Derivative filter cutoff frequency in Hz. 0 = no filtering. Typical: 5-50 Hz

    // Internal state variables - updated each control cycle by update_all()
    float _target;               ///< Last target setpoint (input units). Stored for feedforward calculation
    float _error;                ///< Filtered error value (input units). = low_pass_filter(target - measurement)
    float _derivative;           ///< Filtered derivative (input units/second). = low_pass_filter(d(error)/dt)
    float _integrator;           ///< Accumulated integral term (output units). Clamped to ±_kimax with anti-windup
    bool _reset_filter;          ///< Filter reset flag. When true, filters initialize to current values on next update_all()

    AP_PIDInfo _pid_info;        ///< PID component data structure for logging and telemetry output

private:
    const float default_kp;
    const float default_ki;
    const float default_kd;
    const float default_kff;
    const float default_kimax;
    const float default_filt_E_hz;
    const float default_filt_D_hz;
};
