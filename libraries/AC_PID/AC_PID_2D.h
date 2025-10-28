#pragma once

/**
 * @file AC_PID_2D.h
 * @brief 2D vectorized PID controller for multi-axis control applications
 * 
 * @details This file implements a 2D PID controller that operates on Vector2f/Vector3f
 *          inputs for simultaneous control of two axes (typically X-Y horizontal control).
 *          
 *          Key features:
 *          - Vector magnitude based IMAX clamping (not independent per-axis limits)
 *          - Directional anti-windup using limit vectors - integrator only grows if
 *            it doesn't push further in the direction of the limit vector
 *          - Independent input (error) and derivative filtering per axis
 *          - EEPROM-backed parameter storage for gains and filter frequencies
 *          - Separate telemetry tracking (AP_PIDInfo) for X and Y axes
 *          
 *          Typical applications: horizontal position control, velocity control in
 *          NED horizontal plane (North-East components).
 *          
 *          Source: libraries/AC_PID/AC_PID_2D.h
 */

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <stdlib.h>
#include <cmath>
#include <AC_PID/AP_PIDInfo.h>

/**
 * @class AC_PID_2D
 * @brief 2D vectorized PID controller for simultaneous multi-axis control
 * 
 * @details AC_PID_2D implements a PID controller that operates on 2D vector inputs
 *          (Vector2f/Vector3f) to provide simultaneous control of two axes. This is
 *          primarily used for horizontal position and velocity control in multirotors.
 *          
 *          Architecture:
 *          - Operates on Vector2f inputs for target, measurement, and error
 *          - Applies low-pass filters independently per axis (X and Y)
 *          - Implements directional anti-windup: the integrator only accumulates
 *            if doing so does not push further in the direction indicated by the
 *            limit vector (representing control saturation direction)
 *          - Uses vector magnitude for IMAX clamping, not independent per-axis limits
 *          
 *          Usage pattern for horizontal position control:
 *          1. Initialize with gains (kP, kI, kD, kFF) and filter frequencies
 *          2. Call update_all() with target position, actual position, dt, and limit vector
 *          3. Retrieve P, I, D, FF components or use total output for control
 *          
 *          Coordinate frames:
 *          Typically operates in NED horizontal plane (North-East components), but
 *          can work with any 2D coordinate system. Ensure target, measurement, and
 *          output are all in the same frame.
 *          
 *          Thread safety:
 *          This class is NOT thread-safe. Calls to update_all() and parameter
 *          modifications must be synchronized externally if used from multiple threads.
 *          
 * @note Vector magnitude based IMAX clamping means the integrator is limited by
 *       sqrt(Ix^2 + Iy^2) <= IMAX, not |Ix| <= IMAX and |Iy| <= IMAX independently.
 * 
 * @warning Directional anti-windup requires the limit vector to accurately represent
 *          the actual control saturation direction for correct operation.
 */
class AC_PID_2D {
public:

    /**
     * @brief Constructor for 2D PID controller with EEPROM-backed parameter storage
     * 
     * @details Initializes a 2D PID controller with default gain values. If parameters
     *          have been previously saved to EEPROM via AP_Param, those values will be
     *          loaded at runtime, overriding the defaults provided here.
     *          
     *          All internal state (integrator, filters, error) is initialized to zero.
     * 
     * @param[in] initial_kP         Proportional gain (dimensionless)
     * @param[in] initial_kI         Integral gain (1/s) - integrator accumulation rate
     * @param[in] initial_kD         Derivative gain (s) - derivative term weight
     * @param[in] initial_kFF        Feedforward gain (dimensionless)
     * @param[in] initial_imax       Maximum integrator magnitude (same units as control output)
     * @param[in] initial_filt_hz    Error filter cutoff frequency (Hz) - 0 disables filtering
     * @param[in] initial_filt_d_hz  Derivative filter cutoff frequency (Hz) - 0 disables filtering
     * 
     * @note Filter frequencies in Hz are converted to alpha coefficients internally based on dt
     * @note IMAX applies to the vector magnitude sqrt(Ix^2 + Iy^2), not per-axis
     */
    AC_PID_2D(float initial_kP, float initial_kI, float initial_kD, float initial_kFF, float initial_imax, float initial_filt_hz, float initial_filt_d_hz);

    CLASS_NO_COPY(AC_PID_2D);

    /**
     * @brief Compute complete 2D PID output from target and measurement vectors
     * 
     * @details Performs full PID calculation: P + I + D + FF terms for 2D control.
     *          
     *          Processing steps:
     *          1. Calculate error vector: target - measurement
     *          2. Apply error low-pass filter if filt_E_hz > 0
     *          3. Calculate derivative of filtered error
     *          4. Apply derivative low-pass filter if filt_D_hz > 0
     *          5. Update integrator with directional anti-windup
     *          6. Compute output: kP*error + kI*integrator + kD*derivative + kFF*target
     *          
     *          Directional anti-windup: integrator only grows if the increment does not
     *          increase the component in the direction of the limit vector. This prevents
     *          integrator buildup when control is saturated in a particular direction.
     * 
     * @param[in] target      Target value vector (e.g., desired position or velocity)
     * @param[in] measurement Current measurement vector (actual position or velocity)
     * @param[in] dt          Time step in seconds since last update
     * @param[in] limit       Control saturation direction vector for directional anti-windup
     * 
     * @return Vector2f 2D correction output (P+I+D+FF) in same units as error * gains
     * 
     * @note First call or after reset_filter() initializes filters without derivative spike
     * @note Limit vector should point in direction of control saturation, magnitude indicates severity
     * @note Typical call rate: 100-400 Hz for position control loops
     * 
     * @warning dt must be > 0; zero or negative dt will cause division by zero in derivative calculation
     * @warning Limit vector must represent actual control authority limits for correct anti-windup
     */
    Vector2f update_all(const Vector2f &target, const Vector2f &measurement, float dt, const Vector2f &limit);
    
    /**
     * @brief Compute 2D PID output using only X-Y components of 3D vectors
     * 
     * @details Convenience overload that extracts X and Y components from Vector3f inputs
     *          and calls the Vector2f version. Z components are ignored.
     *          
     *          Useful for horizontal control when working with 3D position vectors.
     * 
     * @param[in] target      Target value vector (only X,Y used)
     * @param[in] measurement Current measurement vector (only X,Y used)
     * @param[in] dt          Time step in seconds since last update
     * @param[in] limit       Control saturation direction vector (only X,Y used)
     * 
     * @return Vector2f 2D correction output (P+I+D+FF) for X-Y control
     * 
     * @see update_all(const Vector2f&, const Vector2f&, float, const Vector2f&)
     */
    Vector2f update_all(const Vector3f &target, const Vector3f &measurement, float dt, const Vector3f &limit);

    /**
     * @brief Update 2D integrator with directional anti-windup
     * 
     * @details Accumulates filtered error into the integrator using:
     *          integrator += kI * error * dt
     *          
     *          Directional anti-windup logic:
     *          - Calculate proposed integrator increment: delta = kI * error * dt
     *          - Only apply increment if (integrator + delta) · limit <= integrator · limit
     *          - This prevents integrator from growing in the direction of saturation
     *          - After update, clamp integrator magnitude to IMAX: |integrator| <= IMAX
     *          
     *          This method is called internally by update_all() and typically should
     *          not be called directly unless implementing custom control loops.
     * 
     * @param[in] dt    Time step in seconds for integration
     * @param[in] limit Control saturation direction vector for anti-windup
     * 
     * @note Integrator magnitude is clamped to IMAX as vector length, not per-axis
     * @note If kI is zero, integrator remains unchanged
     * @note Zero or near-zero limit vector disables directional anti-windup (allows normal accumulation)
     * 
     * @warning Requires _error to be set (normally by update_all) before calling
     */
    void update_i(float dt, const Vector2f &limit);

    /**
     * @brief Get proportional term output vector
     * 
     * @details Returns kP * error calculated from last update_all() call.
     * 
     * @return Vector2f Proportional term contribution (same units as error * kP)
     * 
     * @note Returns zero vector if update_all() has not been called yet
     */
    Vector2f get_p() const;
    
    /**
     * @brief Get integral term output vector
     * 
     * @details Returns current integrator value. The integrator accumulates filtered
     *          error over time with directional anti-windup applied.
     * 
     * @return const Vector2f& Reference to current integrator state (units: error * kI * time)
     * 
     * @note Integrator magnitude is always <= IMAX
     * @see reset_I() to zero the integrator
     */
    const Vector2f& get_i() const;
    
    /**
     * @brief Get derivative term output vector
     * 
     * @details Returns kD * derivative, where derivative is the filtered rate of
     *          change of error from last update_all() call.
     * 
     * @return Vector2f Derivative term contribution (units: kD * error/time)
     * 
     * @note Returns zero vector on first call after reset_filter() to prevent derivative spike
     */
    Vector2f get_d() const;
    
    /**
     * @brief Get feedforward term output vector
     * 
     * @details Returns kFF * target, providing direct feedforward from target to output.
     *          Useful for improving tracking performance when target changes.
     * 
     * @return Vector2f Feedforward term contribution (same units as target * kFF)
     * 
     * @note Feedforward is based on target, not error, so it acts even at zero error
     */
    Vector2f get_ff();
    
    /**
     * @brief Get current error vector
     * 
     * @details Returns filtered error vector (target - measurement) from last update_all() call.
     *          If error filtering is enabled (filt_E_hz > 0), this is the filtered error.
     * 
     * @return const Vector2f& Reference to current error state (same units as target/measurement)
     * 
     * @note Error is filtered if filt_E_hz > 0, raw otherwise
     */
    const Vector2f& get_error() const { return _error; }

    /**
     * @brief Reset integrator to zero
     * 
     * @details Sets the 2D integrator vector to (0, 0), clearing all accumulated integral action.
     *          
     *          Typical use cases:
     *          - Mode changes (e.g., switching from manual to automatic control)
     *          - Large target changes that would cause integrator windup
     *          - After detecting control saturation that invalidates integrator state
     * 
     * @note Does not affect filters, error, or derivative state - use reset_filter() for that
     * @note Does not reset P, D, or FF terms which are recalculated each update
     * 
     * @warning Resetting integrator causes transient response until it rebuilds
     */
    void reset_I();

    /**
     * @brief Flag filters for reset on next update_all() call
     * 
     * @details Sets internal flag to reinitialize error and derivative filters on the
     *          next update_all() call. This prevents derivative spikes when there are
     *          discontinuities in target or measurement.
     *          
     *          Filter reset behavior:
     *          - Error filter initialized to current raw error
     *          - Derivative filter initialized to zero
     *          - No derivative term output on first update after reset
     * 
     * @note Reset happens automatically on first update_all() call after construction
     * @note Call this when target or measurement has a discontinuous jump
     * @note Does not reset integrator - use reset_I() for that
     */
    void reset_filter() { _reset_filter = true; }

    /**
     * @brief Save current gains and filter parameters to EEPROM
     * 
     * @details Writes all AP_Param parameters (kP, kI, kD, kFF, IMAX, filt_E_hz, filt_D_hz)
     *          to EEPROM for persistent storage across reboots.
     *          
     *          Note: This method is typically not used directly as AP_Param handles
     *          parameter storage automatically through the ground control station interface.
     * 
     * @note EEPROM write cycles are limited; avoid calling frequently
     * @note Parameters are automatically loaded from EEPROM on next boot
     */
    void save_gains();

    /**
     * @brief Get reference to proportional gain parameter
     * @return AP_Float& Reference to kP parameter (dimensionless)
     * @note Can be used for runtime tuning or parameter setup
     */
    AP_Float &kP() { return _kp; }
    
    /**
     * @brief Get reference to integral gain parameter
     * @return AP_Float& Reference to kI parameter (1/s)
     * @note Can be used for runtime tuning or parameter setup
     */
    AP_Float &kI() { return _ki; }
    
    /**
     * @brief Get reference to derivative gain parameter
     * @return AP_Float& Reference to kD parameter (s)
     * @note Can be used for runtime tuning or parameter setup
     */
    AP_Float &kD() { return _kd; }
    
    /**
     * @brief Get reference to feedforward gain parameter
     * @return AP_Float& Reference to kFF parameter (dimensionless)
     * @note Can be used for runtime tuning or parameter setup
     */
    AP_Float &ff() { return _kff;}
    
    /**
     * @brief Get reference to error filter frequency parameter
     * @return AP_Float& Reference to error filter cutoff frequency in Hz (0 = no filtering)
     * @note Can be used for runtime tuning or parameter setup
     */
    AP_Float &filt_E_hz() { return _filt_E_hz; }
    
    /**
     * @brief Get reference to derivative filter frequency parameter
     * @return AP_Float& Reference to derivative filter cutoff frequency in Hz (0 = no filtering)
     * @note Can be used for runtime tuning or parameter setup
     */
    AP_Float &filt_D_hz() { return _filt_D_hz; }
    
    /**
     * @brief Get current integrator magnitude limit
     * @return float Maximum allowed integrator vector magnitude (same units as control output)
     * @note IMAX applies to vector magnitude sqrt(Ix^2 + Iy^2), not per-axis
     */
    float imax() const { return _kimax.get(); }
    
    /**
     * @brief Calculate error filter alpha coefficient for given time step
     * 
     * @details Converts error filter frequency (filt_E_hz) to filter alpha using:
     *          alpha = 1 - exp(-2*pi*filt_E_hz*dt)
     *          
     *          This is used internally for low-pass filtering the error signal.
     * 
     * @param[in] dt Time step in seconds
     * @return float Filter alpha coefficient [0, 1] where 0=no filtering, 1=no smoothing
     * 
     * @note Returns 1.0 (no filtering) if filt_E_hz is zero
     * @note Alpha calculation ensures consistent filter behavior across different loop rates
     */
    float get_filt_E_alpha(float dt) const;
    
    /**
     * @brief Calculate derivative filter alpha coefficient for given time step
     * 
     * @details Converts derivative filter frequency (filt_D_hz) to filter alpha using:
     *          alpha = 1 - exp(-2*pi*filt_D_hz*dt)
     *          
     *          This is used internally for low-pass filtering the derivative term.
     * 
     * @param[in] dt Time step in seconds
     * @return float Filter alpha coefficient [0, 1] where 0=no filtering, 1=no smoothing
     * 
     * @note Returns 1.0 (no filtering) if filt_D_hz is zero
     * @note Higher frequency = more responsive but noisier derivative
     */
    float get_filt_D_alpha(float dt) const;

    /**
     * @brief Set proportional gain
     * @param[in] v New kP value (dimensionless)
     * @note Takes effect immediately on next update_all() call
     */
    void set_kP(float v) { _kp.set(v); }
    
    /**
     * @brief Set integral gain
     * @param[in] v New kI value (1/s)
     * @note Takes effect immediately on next update_all() call
     * @note Does not affect existing integrator value
     */
    void set_kI(float v) { _ki.set(v); }
    
    /**
     * @brief Set derivative gain
     * @param[in] v New kD value (s)
     * @note Takes effect immediately on next update_all() call
     */
    void set_kD(float v) { _kd.set(v); }
    
    /**
     * @brief Set feedforward gain
     * @param[in] v New kFF value (dimensionless)
     * @note Takes effect immediately on next update_all() call
     */
    void set_ff(float v) { _kff.set(v); }
    
    /**
     * @brief Set integrator magnitude limit
     * @param[in] v New IMAX value (same units as control output)
     * @note Absolute value is used; negative values become positive
     * @note IMAX applies to vector magnitude sqrt(Ix^2 + Iy^2), not per-axis
     * @note Existing integrator is clamped to new IMAX on next update
     */
    void set_imax(float v) { _kimax.set(fabsf(v)); }
    
    /**
     * @brief Set error filter cutoff frequency
     * @param[in] hz New error filter frequency in Hz (0 = disable filtering)
     * @note Absolute value is used; negative values become positive
     * @note Takes effect on next update_all() call
     * @note Does not reset current filter state
     */
    void set_filt_E_hz(float hz) { _filt_E_hz.set(fabsf(hz)); }
    
    /**
     * @brief Set derivative filter cutoff frequency
     * @param[in] hz New derivative filter frequency in Hz (0 = disable filtering)
     * @note Absolute value is used; negative values become positive
     * @note Takes effect on next update_all() call
     * @note Does not reset current filter state
     */
    void set_filt_D_hz(float hz) { _filt_D_hz.set(fabsf(hz)); }

    /**
     * @brief Set integrator with target, measurement, and desired integrator value
     * 
     * @details Calculates error from target and measurement, then sets integrator.
     *          This is useful when initializing the controller with a known state.
     *          Integrator magnitude is clamped to IMAX.
     * 
     * @param[in] target      Target value vector
     * @param[in] measurement Current measurement vector
     * @param[in] i           Desired integrator value vector
     * 
     * @note Final integrator is clamped: |integrator| <= IMAX
     * @note Also updates internal _error state
     */
    void set_integrator(const Vector2f& target, const Vector2f& measurement, const Vector2f& i);
    
    /**
     * @brief Set integrator with known error and desired integrator value
     * 
     * @details Sets integrator given a pre-calculated error vector.
     *          Integrator magnitude is clamped to IMAX.
     * 
     * @param[in] error Known error vector (target - measurement)
     * @param[in] i     Desired integrator value vector
     * 
     * @note Final integrator is clamped: |integrator| <= IMAX
     * @note Also updates internal _error state
     */
    void set_integrator(const Vector2f& error, const Vector2f& i);
    
    /**
     * @brief Set integrator from Vector3f (uses X-Y components only)
     * 
     * @details Convenience overload for 3D vectors. Extracts X and Y components
     *          and discards Z component.
     * 
     * @param[in] i Desired integrator value (only X,Y used)
     * 
     * @note Z component is ignored
     * @see set_integrator(const Vector2f&)
     */
    void set_integrator(const Vector3f& i) { set_integrator(Vector2f{i.x, i.y}); }
    
    /**
     * @brief Set integrator directly
     * 
     * @details Sets the 2D integrator to the specified value, clamped to IMAX magnitude.
     *          
     *          Use cases:
     *          - Restoring integrator state after mode change
     *          - Pre-loading integrator for bumpless transfer
     *          - Implementing custom anti-windup logic
     * 
     * @param[in] i Desired integrator value vector
     * 
     * @note If |i| > IMAX, integrator is scaled to have magnitude IMAX in direction of i
     * @note IMAX clamping uses vector magnitude: sqrt(Ix^2 + Iy^2) <= IMAX
     */
    void set_integrator(const Vector2f& i);

    /**
     * @brief Get telemetry information for X-axis PID
     * 
     * @details Returns AP_PIDInfo structure containing detailed telemetry for the X-axis:
     *          - target: desired X value
     *          - actual: measured X value
     *          - error: X error
     *          - P, I, D, FF: individual term contributions for X-axis
     *          
     *          This is used for logging and ground control station display.
     * 
     * @return const AP_PIDInfo& Reference to X-axis telemetry structure
     * 
     * @note Separate telemetry for X and Y allows independent monitoring of each axis
     * @note Updated on every update_all() call
     */
    const AP_PIDInfo& get_pid_info_x(void) const { return _pid_info_x; }
    
    /**
     * @brief Get telemetry information for Y-axis PID
     * 
     * @details Returns AP_PIDInfo structure containing detailed telemetry for the Y-axis:
     *          - target: desired Y value
     *          - actual: measured Y value
     *          - error: Y error
     *          - P, I, D, FF: individual term contributions for Y-axis
     *          
     *          This is used for logging and ground control station display.
     * 
     * @return const AP_PIDInfo& Reference to Y-axis telemetry structure
     * 
     * @note Separate telemetry for X and Y allows independent monitoring of each axis
     * @note Updated on every update_all() call
     */
    const AP_PIDInfo& get_pid_info_y(void) const { return _pid_info_y; }

    // parameter var table
    static const struct AP_Param::GroupInfo        var_info[];

protected:

    // EEPROM-backed gain parameters (dimensionless or per-time units)
    AP_Float _kp;                ///< Proportional gain (dimensionless)
    AP_Float _ki;                ///< Integral gain (1/s) - integrator accumulation rate
    AP_Float _kd;                ///< Derivative gain (s) - derivative term weight
    AP_Float _kff;               ///< Feedforward gain (dimensionless)
    AP_Float _kimax;             ///< Maximum integrator vector magnitude (same units as control output)
    AP_Float _filt_E_hz;         ///< Error (input) low-pass filter cutoff frequency (Hz), 0 = no filtering
    AP_Float _filt_D_hz;         ///< Derivative low-pass filter cutoff frequency (Hz), 0 = no filtering

    // Internal 2D state variables (all in application coordinate frame, typically NED horizontal)
    Vector2f    _target;         ///< Last target vector for derivative calculation and filtering
    Vector2f    _error;          ///< Current filtered error vector (target - measurement)
    Vector2f    _derivative;     ///< Current filtered derivative of error (change per second)
    Vector2f    _integrator;     ///< Accumulated integral term, magnitude clamped to IMAX
    bool        _reset_filter;   ///< Flag: true = reset filters on next update_all() to prevent derivative spike

    // Telemetry structures for independent X and Y axis logging and monitoring
    AP_PIDInfo _pid_info_x;      ///< Telemetry data for X-axis (target, actual, error, P, I, D, FF)
    AP_PIDInfo _pid_info_y;      ///< Telemetry data for Y-axis (target, actual, error, P, I, D, FF)

private:
    // Default parameter values provided at construction, used if EEPROM values not available
    const float default_kp;            ///< Default proportional gain
    const float default_ki;            ///< Default integral gain
    const float default_kd;            ///< Default derivative gain
    const float default_kff;           ///< Default feedforward gain
    const float default_kimax;         ///< Default integrator magnitude limit
    const float default_filt_E_hz;     ///< Default error filter frequency (Hz)
    const float default_filt_D_hz;     ///< Default derivative filter frequency (Hz)
};
