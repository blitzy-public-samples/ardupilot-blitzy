/**
 * @file PID.h
 * @brief Generic PID algorithm with EEPROM-backed storage of constants
 * 
 * @details This file implements a basic PID controller for general control loops with:
 * - EEPROM-backed tunable gains via AP_Param for persistent storage
 * - Low-pass filtered derivative with 20Hz cutoff frequency
 * - Integrator anti-windup with IMAX clamping to prevent windup
 * - Integration with ArduPilot parameter system for ground station access
 * - Simpler than AC_PID (no slew limiting, no notch filters, no feedforward)
 * 
 * This is the basic/legacy PID implementation. For advanced features including
 * input filtering, slew rate limiting, notch filters, and feedforward terms,
 * see the AC_PID library (libraries/AC_PID/).
 */
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AC_PID/AC_PID.h>  // for AP_PIDInfo
#include <stdlib.h>
#include <cmath>

/**
 * @class PID
 * @brief Basic PID controller with EEPROM-backed parameter storage
 * 
 * @details This class implements a discrete-time PID controller with the following features:
 * 
 * **Control Algorithm:**
 * - output = scaler*(kp*error + kd*filtered_derivative) + integrator
 * - P term: P = kp * error (proportional to current error)
 * - I term: I += ki * error * scaler * dt (accumulated over time, clamped to ±imax)
 * - D term: D = kd * filtered_derivative, where derivative = (error - last_error)/dt
 * - Derivative filtered with first-order low-pass filter (RC = 1/(2*pi*20Hz) ≈ 8ms time constant)
 * 
 * **Implementation Details:**
 * - Discrete-time implementation with millisecond timing via hal.millis()
 * - First-order low-pass derivative filter with 20Hz cutoff frequency
 * - Integrator clamping to prevent windup (±imax limit)
 * - 1-second idle timeout that automatically resets integrator
 * - AP_Param integration for persistent EEPROM storage
 * - load_gains/save_gains methods for EEPROM operations
 * 
 * **Usage Pattern:**
 * Called periodically in control loops (typically 50-400Hz depending on application).
 * Error input should be (target - actual) for positive feedback correction.
 * 
 * **Units:**
 * Gain units depend on application:
 * - kp: dimensionless if error/output dimensionless, otherwise application-specific
 * - ki: 1/seconds (integrator accumulates error*ki*dt)
 * - kd: seconds (derivative in error/second, output in error units)
 * - imax: same units as output (absolute value clamp)
 * 
 * **Timing:**
 * - dt measured via hal.millis() in milliseconds, converted to seconds
 * - Integrator resets if dt > 1 second to prevent buildup after long pauses
 * - Timing jitter affects I and D term accuracy
 * 
 * @note This is the basic/legacy PID implementation. For advanced features
 * (input filtering, slew limiting, notch filters, feedforward) see AC_PID
 * library (libraries/AC_PID/).
 * 
 * @note Derivative filter is first-order low-pass with RC = 1/(2*pi*20Hz) ≈ 8ms
 * time constant to reject high-frequency noise.
 * 
 * @warning This is the basic PID implementation. For advanced features see
 * AC_PID library (libraries/AC_PID/).
 * 
 * @warning Integrator anti-windup: clamps to ±imax, automatically resets if dt > 1 second.
 * 
 * @see AC_PID (libraries/AC_PID/AC_PID.h) for advanced PID with additional features
 * @see Filter library for filter implementations
 */
class PID {
public:

    /**
     * @brief Construct PID controller with optional initial gains
     * 
     * @details Initializes PID controller with specified gains or defaults to zero.
     * Calls AP_Param::setup_object_defaults() to initialize AP_Param-backed members
     * (_kp, _ki, _kd, _imax) for EEPROM persistence and ground station parameter access.
     * Sets _last_derivative to NAN to suppress first derivative sample after reset,
     * avoiding transient from stale _last_error value.
     * 
     * @param[in] initial_p Proportional gain (dimensionless or application-specific units).
     *                      Positive error produces positive output. Default 0.0.
     * @param[in] initial_i Integral gain in 1/seconds. Integrator accumulates
     *                      error*ki*dt. Default 0.0.
     * @param[in] initial_d Derivative gain in seconds. Output proportional to rate
     *                      of change of error. Default 0.0.
     * @param[in] initial_imax Integrator maximum absolute value (same units as output).
     *                         Prevents windup by clamping integrator to ±imax. Default 0.
     * 
     * @note Gains are stored as AP_Param members for EEPROM persistence and ground
     * station parameter access via MAVLink parameter protocol.
     * 
     * @note Units depend on application - gains are dimensionless for unitless control,
     * or have appropriate dimensions for physical quantities (e.g., attitude control,
     * velocity control).
     */
    PID(const float &   initial_p = 0.0f,
        const float &   initial_i = 0.0f,
        const float &   initial_d = 0.0f,
        const int16_t & initial_imax = 0):
            default_kp(initial_p),
            default_ki(initial_i),
            default_kd(initial_d),
            default_kimax(initial_imax)
    {
		AP_Param::setup_object_defaults(this, var_info);

		// set _last_derivative as invalid when we startup
		_last_derivative = NAN;
    }

    /**
     * @brief Compute PID control output from current error
     * 
     * @details Calculates PID control output using discrete-time implementation:
     * 
     * **Algorithm Steps:**
     * 1. Measures dt in milliseconds via hal.millis(), converts to seconds
     * 2. Resets integrator if dt > 1 second (prevents I buildup after long pauses)
     * 3. Computes P term: P = error * kp
     * 4. Computes D term with low-pass filter: D = kd * filtered_derivative,
     *    where derivative = (error - last_error)/dt, filtered with RC = 1/(2*pi*20Hz)
     * 5. Updates integrator with anti-windup: I += ki * error * scaler * dt,
     *    clamped to ±imax
     * 6. Returns: output = scaler*(P + D) + I
     * 
     * **Control Law:**
     * - output = scaler*(kp*error + kd*filtered_derivative) + integrator
     * - P and D terms scaled by scaler parameter at output
     * - I term scaled by scaler during accumulation, not at output
     * 
     * **Timing Behavior:**
     * - Uses platform timer (hal.millis()) for dt measurement
     * - Timing jitter affects I and D term accuracy
     * - First derivative sample after reset_I() is suppressed to avoid transient
     * 
     * @param[in] error The measured error value (target - actual). Positive error
     *                  produces positive output. Units are application-specific.
     * @param[in] scaler Scaling factor applied to P and D terms at output (default 1.0).
     *                   Integrator scaled during accumulation. Dimensionless.
     * 
     * @return The combined PID control output in same units as error (scaled by gains).
     *         P and D terms scaled by scaler parameter, I term not scaled at output
     *         (already scaled during accumulation).
     * 
     * @note Must be called regularly for accurate integration and derivative calculation.
     * Typical rates: 50-400Hz depending on application (attitude control typically
     * 400Hz, position control typically 50-100Hz).
     * 
     * @note Uses platform timer (hal.millis()) for dt measurement. Timing jitter
     * affects I and D term accuracy, especially at low sample rates.
     * 
     * @warning Do not call with dt > 1 second as integrator will automatically reset.
     * For intermittent use, call reset_I() explicitly instead of relying on timeout.
     * 
     * @warning First derivative sample after reset_I() is suppressed to avoid transient
     * from stale _last_error value (derivative set to 0 when _last_derivative is NAN).
     */
    float        get_pid(float error, float scaler = 1.0);

    /**
     * @brief Reset complete PID state including integrator, derivative, and telemetry
     * 
     * @details Zeros all internal state: _pid_info (telemetry structure), _integrator,
     * and sets _last_derivative to NAN. Called during mode changes or when starting
     * fresh control (e.g., switching from manual to stabilize mode, takeoff, landing).
     * NAN sentinel in _last_derivative suppresses next derivative calculation to avoid
     * transient from undefined _last_error.
     * 
     * @note Use reset_I() if only integrator reset is needed. Full reset() also clears
     * derivative state (sets _last_derivative to NAN) and telemetry (_pid_info).
     */
    void        reset();

    /**
     * @brief Reset integrator and derivative state only
     * 
     * @details Sets _integrator to 0, _last_derivative to NAN, and _pid_info.I to 0.
     * NAN sentinel in _last_derivative suppresses next derivative calculation to avoid
     * transient from stale _last_error value. Called automatically by get_pid() if
     * dt > 1000ms to prevent integrator buildup after long pauses.
     * 
     * @note Preserves P and D state (except _last_derivative), only resets I term.
     * Useful for clearing integrator windup without full state reset (e.g., during
     * setpoint changes, mode transitions within same control regime).
     */
    void        reset_I();

    /**
     * @brief Load PID gains from EEPROM via AP_Param
     * 
     * @details Calls .load() on _kp, _ki, _kd, _imax AP_Param members to restore
     * values from persistent storage. Typically called during initialization to
     * restore previously saved or ground-station-configured gains.
     * 
     * @note Gains must have been previously saved with save_gains() or set via
     * ground station parameter interface (MAVLink parameter protocol).
     */
    void        load_gains();

    /**
     * @brief Save current PID gains to EEPROM via AP_Param
     * 
     * @details Calls .save() on _kp, _ki, _kd, _imax AP_Param members to persist
     * current values to EEPROM. Used after tuning or parameter changes to make
     * them permanent across reboots.
     * 
     * @note EEPROM has limited write cycles (~100,000 writes). Avoid calling
     * repeatedly in fast loops or on every parameter change.
     * 
     * @warning EEPROM writes take time and should not be called in time-critical
     * loops (can take several milliseconds). Call only during initialization,
     * configuration, or user-initiated save operations.
     */
    void        save_gains();

    /// @name	parameter accessors
    //@{

    /**
     * @brief Bulk set all PID gains at once
     * 
     * @details Convenient initialization pattern for setting all four PID gains
     * in a single call. Example: pid(1.0f, 0.5f, 0.01f, 100); sets kp=1.0,
     * ki=0.5, kd=0.01, imax=100 in one statement.
     * 
     * @param[in] p Proportional gain (dimensionless or application-specific)
     * @param[in] i Integral gain (1/seconds)
     * @param[in] d Derivative gain (seconds)
     * @param[in] imaxval Integrator maximum absolute value (same units as output)
     * 
     * @note Does not save to EEPROM. Call save_gains() after to persist changes.
     */
    void operator        () (const float    p,
                             const float    i,
                             const float    d,
                             const int16_t  imaxval) {
        _kp.set(p); _ki.set(i); _kd.set(d); _imax.set(imaxval);
    }

    /**
     * @brief Get proportional gain
     * @return Current kP value (dimensionless or application-specific)
     */
    float        kP() const {
        return _kp.get();
    }
    
    /**
     * @brief Get integral gain in 1/seconds
     * @return Current kI value (1/seconds)
     */
    float        kI() const {
        return _ki.get();
    }
    
    /**
     * @brief Get derivative gain in seconds
     * @return Current kD value (seconds)
     */
    float        kD() const {
        return _kd.get();
    }
    
    /**
     * @brief Get integrator maximum absolute value
     * @return Current imax value (always positive, same units as output)
     */
    int16_t        imax() const {
        return _imax.get();
    }

    /**
     * @brief Set proportional gain
     * @param[in] v New kP value (dimensionless or application-specific)
     */
    void        kP(const float v)               {
        _kp.set(v);
    }
    
    /**
     * @brief Set integral gain in 1/seconds
     * @param[in] v New kI value (1/seconds)
     */
    void        kI(const float v)               {
        _ki.set(v);
    }
    
    /**
     * @brief Set derivative gain in seconds
     * @param[in] v New kD value (seconds)
     */
    void        kD(const float v)               {
        _kd.set(v);
    }
    
    /**
     * @brief Set integrator maximum absolute value
     * 
     * @param[in] v New imax value (same units as output). Automatically applies
     *              abs() to ensure positive clamp value.
     * 
     * @note Automatically applies abs() to ensure positive clamp value, so both
     * positive and negative values result in positive imax (e.g., imax(-100) sets
     * imax to 100, clamping integrator to ±100).
     */
    void        imax(const int16_t v)   {
        _imax.set(abs(v));
    }

    /**
     * @brief Get current integrator accumulator value
     * 
     * @return Current _integrator value, same units as output, clamped to ±imax
     * 
     * @note Useful for monitoring integrator state, detecting windup conditions,
     * telemetry/logging, or debugging control performance. Integrator is clamped
     * to ±imax during accumulation, so returned value is always within bounds.
     */
    float        get_integrator() const {
        return _integrator;
    }

    /**
     * @brief AP_Param parameter definition table
     * 
     * @details Defined in PID.cpp, registers P, I, D, IMAX parameters for EEPROM
     * storage and ground station access via MAVLink parameter protocol. Used by
     * AP_Param::setup_object_defaults() in constructor to initialize AP_Param-backed
     * members with default values.
     * 
     * @note Modify var_info[] in PID.cpp to add/remove parameters (requires EEPROM
     * format change and parameter conversion for existing installations).
     */
    static const struct AP_Param::GroupInfo        var_info[];

    /**
     * @brief Get telemetry structure for logging
     * 
     * @details Returns const reference to AP_PIDInfo structure containing target,
     * error, P, I, D terms. Populated by get_pid() on each call. Used by logging
     * system (AP_Logger) to record PID performance for post-flight analysis and
     * tuning.
     * 
     * @return const reference to AP_PIDInfo structure with current PID state
     * 
     * @see AP_PIDInfo (libraries/AC_PID/AP_PIDInfo.h) for structure definition
     */
    const AP_PIDInfo& get_pid_info(void) const { return _pid_info; }

private:
    AP_Float        _kp;        ///< Proportional gain (AP_Param-backed for EEPROM storage and ground station access)
    AP_Float        _ki;        ///< Integral gain in 1/seconds (AP_Param-backed for EEPROM storage)
    AP_Float        _kd;        ///< Derivative gain in seconds (AP_Param-backed for EEPROM storage)
    AP_Int16        _imax;      ///< Integrator clamp absolute value (AP_Param-backed for EEPROM storage), same units as output

    float           _integrator;        ///< Integrator accumulator, clamped to ±_imax, same units as output
    float           _last_error;        ///< Previous error for derivative calculation (error units)
    float           _last_derivative;   ///< Filtered derivative state for low-pass filter, NAN after reset to suppress first sample
    uint32_t        _last_t;            ///< Last get_pid() call timestamp in milliseconds (from hal.millis())

    /**
     * @brief Internal PID computation with explicit dt
     * 
     * @param[in] error Current error (target - actual)
     * @param[in] dt Delta time in milliseconds
     * @param[in] scaler Scale factor for P and D terms
     * @return PID output (P, I, D combined)
     * 
     * @note Private implementation called by public get_pid() after dt calculation
     */
    float           _get_pid(float error, uint16_t dt, float scaler);

    AP_PIDInfo _pid_info {};    ///< Telemetry structure populated each get_pid() call for logging (AP_PIDInfo type)

    /**
     * @brief Low pass filter cut frequency for derivative calculation
     * 
     * @details Fixed at 20 Hz because anything over that is probably noise.
     * RC time constant = 1/(2*pi*20Hz) ≈ 8ms. First-order low-pass filter
     * applied to derivative term to reject high-frequency measurement noise
     * and prevent derivative kick.
     * 
     * @see http://en.wikipedia.org/wiki/Low-pass_filter
     */
    static const uint8_t        _fCut = 20;

    const float default_kp;     ///< Default P gain for AP_Param initialization (from constructor)
    const float default_ki;     ///< Default I gain for AP_Param initialization (from constructor)
    const float default_kd;     ///< Default D gain for AP_Param initialization (from constructor)
    const float default_kimax;  ///< Default IMAX for AP_Param initialization (from constructor)
};
