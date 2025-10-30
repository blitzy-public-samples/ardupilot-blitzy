/**
 * @file AP_FW_Controller.h
 * @brief Base class for fixed-wing rate controllers (roll, pitch)
 * 
 * @details Abstract base class providing common rate control implementation,
 *          autotune integration, and airspeed-adaptive PID control for
 *          fixed-wing axis controllers. This class implements shared rate
 *          control logic used by AP_RollController and AP_PitchController.
 *          AP_YawController has an independent implementation and does not
 *          use this base class.
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

#include <AP_Common/AP_Common.h>
#include "AP_AutoTune.h"
#include <AC_PID/AC_PID.h>

/**
 * @class AP_FW_Controller
 * @brief Abstract base class for fixed-wing rate controllers
 * 
 * @details Implements shared rate control logic for roll and pitch controllers:
 *          - AC_PID rate controller with feedforward
 *          - Airspeed-adaptive gain scaling (gains × scaler² where scaler = EAS/reference_airspeed)
 *          - Integrator management (reset, decay)
 *          - Autotune integration via AP_AutoTune
 *          - Output limiting to [-4500, 4500] centidegrees
 * 
 *          Derived classes: AP_RollController, AP_PitchController
 *          Not used by: AP_YawController (has independent implementation)
 * 
 *          Control Algorithm:
 *          - Rate PID: update_all() computes P×error + I×∫error + D×d(error)/dt
 *          - Feedforward: FF×desired_rate (applied in degrees, not scaled)
 *          - Total output: PID_out + FF_out
 *          - Constraint: output limited to [-4500, 4500] centidegrees
 * 
 *          Airspeed Scaling:
 *          - scaler = (EAS / reference_airspeed)²
 *          - P and I gains multiplied by scaler²
 *          - FF and D not scaled (FF already proportional to rate, D on error derivative)
 *          - Rationale: Control surface effectiveness ∝ dynamic_pressure ∝ airspeed²
 * 
 *          Units and Conventions:
 *          - Desired rates in deg/s (converted to rad/s internally for PID)
 *          - Measured rates from sensors in rad/s
 *          - Servo outputs in centidegrees: [-4500, 4500] = [-45°, 45°]
 *          - Airspeed in m/s (equivalent airspeed EAS)
 *          - Time constants (tau) in seconds
 *          - Angle errors in degrees (stored for autotune)
 * 
 *          Coordinate Frames:
 *          - Body frame rates: roll=gyro_x (right wing down positive), pitch=gyro_y (nose up positive)
 *          - Servo outputs: positive = control surface deflection producing positive rate
 * 
 * @warning Airspeed scaling is critical for stable flight across speed range. scaler must be
 *          computed as (EAS/reference)². Using TAS or incorrect scaling causes instability.
 * @warning P and I gains are scaled by scaler². Manual tuning should be done at reference
 *          airspeed. Gains will automatically adapt at other speeds.
 * @warning FF gain is NOT scaled by scaler. It should be tuned to produce approximately
 *          correct servo deflection for desired rate at reference airspeed.
 * @warning Integrator must be frozen on ground (ground_mode=true) to prevent wind-up
 *          during takeoff roll or landing rollout.
 * @warning Underspeed detection (is_underspeed) should return true when approaching stall
 *          to disable rate PID and prevent deep stall.
 * 
 * @note Autotune allocates AP_AutoTune dynamically. If allocation fails (low memory),
 *       failed_autotune_alloc flag is set.
 * @note EAS2TAS conversion from AP::ahrs() accounts for air density - important for
 *       high altitude flight.
 * @note Output limiting to ±4500 centidegrees prevents servo over-travel and maintains
 *       consistency across controllers.
 * @note dt from scheduler typically 0.02s (50Hz) for fixed-wing main loop
 */
class AP_FW_Controller
{
public:
    /**
     * @brief Constructor for fixed-wing rate controller base class
     * 
     * @param[in] parms     Reference to AP_FixedWing vehicle parameters
     * @param[in] defaults  AC_PID default gain values for initialization
     * @param[in] _autotune_type  Axis identifier (ROLL, PITCH, or YAW) for autotune
     * 
     * @note Initializes rate_pid with provided defaults and sets up autotune infrastructure
     */
    AP_FW_Controller(const AP_FixedWing &parms, const AC_PID::Defaults &defaults, AP_AutoTune::ATType _autotune_type);

    /* Do not allow copies */
    CLASS_NO_COPY(AP_FW_Controller);

    /**
     * @brief Non-virtual rate control wrapper for external interface
     * 
     * @param[in] desired_rate  Target rate in degrees/second
     * @param[in] scaler       Airspeed scaling factor computed as (EAS/reference_airspeed)²
     * 
     * @return Servo output in centidegrees [-4500, 4500]
     * 
     * @note Calls _get_rate_out with disable_integrator=false, ground_mode=false
     * @note This is the primary external interface for rate control
     */
    float get_rate_out(float desired_rate, float scaler);
    
    /**
     * @brief Pure virtual angle control method - must be implemented by derived classes
     * 
     * @param[in] angle_err          Angle error in centidegrees
     * @param[in] scaler            Airspeed scaling factor (EAS/reference)²
     * @param[in] disable_integrator True to freeze integrator accumulation
     * @param[in] ground_mode        True when vehicle is on ground (disables integrator)
     * 
     * @return Servo output in centidegrees [-4500, 4500]
     * 
     * @details Derived classes implement angle→rate conversion then call _get_rate_out
     *          with the computed desired rate. This allows each axis to implement
     *          vehicle-specific angle control while sharing rate control logic.
     * 
     * @note AP_RollController and AP_PitchController provide implementations
     */
    virtual float get_servo_out(int32_t angle_err, float scaler, bool disable_integrator, bool ground_mode) = 0;

    /**
     * @brief Set one-loop feedforward scaling multiplier
     * 
     * @param[in] _ff_scale  Scaling factor to apply to feedforward term (replaces previous scale)
     * 
     * @note This replaces any previous scale applied, so should only be used when only
     *       one source of scaling is needed
     * @note Used for special flight modes requiring temporary FF adjustment
     */
    void set_ff_scale(float _ff_scale) { ff_scale = _ff_scale; }

    /**
     * @brief Zero the rate PID integrator
     * 
     * @note Called when switching flight modes or resetting control state
     * @note Essential for preventing integrator wind-up from affecting new mode
     */
    void reset_I();

    /**
     * @brief Reduce integrator by 95% over 2 seconds
     * 
     * @details Used when we have a low scale factor in a quadplane hover to prevent
     *          integrator wind-up. Gradually reduces accumulated integrator value
     *          rather than abruptly zeroing it.
     * 
     * @note Specifically designed for quadplane transitions to hover mode
     * @note Prevents sudden control jumps that would occur with immediate reset
     */
    void decay_I();

    /**
     * @brief Start in-flight autotune sequence
     * 
     * @note Allocates AP_AutoTune dynamically if not already present
     * @note If allocation fails due to low memory, sets failed_autotune_alloc flag
     * @warning Only invoke when aircraft is in stable flight with sufficient altitude
     */
    void autotune_start(void);
    
    /**
     * @brief Stop autotune and restore original gain values
     * 
     * @note Deallocates AP_AutoTune and reverts to pre-autotune PID gains
     * @note Safe to call even if autotune was never started
     */
    void autotune_restore(void);

    /**
     * @brief Get rate PID diagnostic information
     * 
     * @return Const reference to AP_PIDInfo structure containing:
     *         - target: desired rate value
     *         - actual: measured rate value
     *         - error: rate error (target - actual)
     *         - P, I, D, FF: individual PID term contributions
     * 
     * @note Used for logging and telemetry to ground control station
     * @note Updated on every call to _get_rate_out
     */
    const AP_PIDInfo& get_pid_info(void) const
    {
        return _pid_info;
    }

    /**
     * @brief Configure PID notch filter sample rate
     * 
     * @param[in] sample_rate  Sample rate in Hz for notch filter calculations
     * 
     * @note Must be called if main loop rate changes from default
     * @note Notch filters used to reject specific vibration frequencies
     */
    void set_notch_sample_rate(float sample_rate) { rate_pid.set_notch_sample_rate(sample_rate); }

    /**
     * @brief Direct accessor for proportional gain parameter
     * @return Reference to rate PID proportional gain (scaled by airspeed²)
     */
    AP_Float &kP(void) { return rate_pid.kP(); }
    
    /**
     * @brief Direct accessor for integral gain parameter
     * @return Reference to rate PID integral gain (scaled by airspeed²)
     */
    AP_Float &kI(void) { return rate_pid.kI(); }
    
    /**
     * @brief Direct accessor for derivative gain parameter
     * @return Reference to rate PID derivative gain (not scaled by airspeed)
     */
    AP_Float &kD(void) { return rate_pid.kD(); }
    
    /**
     * @brief Direct accessor for feedforward gain parameter
     * @return Reference to rate feedforward gain (not scaled by airspeed)
     */
    AP_Float &kFF(void) { return rate_pid.ff(); }
    
    /**
     * @brief Direct accessor for time constant parameter
     * @return Reference to derivative filter time constant in seconds
     * @note tau controls D-term filtering to reduce noise sensitivity
     */
    AP_Float &tau(void) { return gains.tau; }

protected:
    /// Reference to vehicle fixed-wing parameters (airspeed limits, control limits, etc.)
    const AP_FixedWing &aparm;
    
    /// Autotune gain structure (tau, rmax, FF, P, I, D, IMAX)
    AP_AutoTune::ATGains gains;
    
    /// Pointer to autotune object (nullptr when not active, allocated dynamically)
    AP_AutoTune *autotune;
    
    /// Flag set to true if autotune allocation failed due to low memory
    bool failed_autotune_alloc;
    
    /// Last computed servo output in centidegrees (for continuity/smoothing)
    float _last_out;
    
    /// Rate PID controller instance (implements P, I, D, FF control law)
    AC_PID rate_pid;
    
    /// Angle error in degrees (stored for autotune analysis and logging)
    float angle_err_deg;
    
    /// Feedforward scale multiplier (default 1.0, modified by set_ff_scale)
    float ff_scale = 1.0;

    /// PID diagnostic information structure (updated each control loop)
    AP_PIDInfo _pid_info;

    /**
     * @brief Core rate control implementation with airspeed-adaptive PID
     * 
     * @param[in] desired_rate        Target rate in degrees/second
     * @param[in] scaler             Airspeed scaling factor computed as (EAS/reference_airspeed)²
     * @param[in] disable_integrator  True to freeze integrator accumulation
     * @param[in] aspeed             Equivalent airspeed in m/s (for underspeed check)
     * @param[in] ground_mode        True when vehicle is on ground (disables integrator)
     * 
     * @return Servo output in centidegrees constrained to [-4500, 4500]
     * 
     * @details Algorithm steps:
     *          1. Get dt from AP::scheduler()->get_loop_period_s()
     *          2. Get EAS2TAS from AP::ahrs().get_EAS2TAS()
     *          3. Check underspeed via is_underspeed(aspeed)
     *          4. Convert desired_rate to radians
     *          5. Scale P/I gains: rate_pid.set_ff_scale(scaler²)
     *          6. Call rate_pid.update_all(target_rad × scaler², measured_rate, dt, freeze_integrator)
     *          7. Compute FF: degrees(target_rad) × kFF × ff_scale
     *          8. Compose output: P + I + D (from PID) + FF
     *          9. Store _last_out and constrain to [-4500, 4500] centidegrees
     * 
     *          Integrator Management:
     *          - Frozen when: disable_integrator=true, ground_mode=true, is_underspeed()=true,
     *            or motor limits reached
     *          - Reset via reset_I() when switching modes
     *          - Decay via decay_I() during quadplane transitions (95% per 2s)
     * 
     * @note Called by public get_rate_out() and derived class get_servo_out() implementations
     * @note dt typically 0.02s (50Hz) for fixed-wing main loop
     * 
     * @warning P and I gains scaled by scaler² - tune at reference airspeed for correct adaptation
     * @warning FF gain NOT scaled - must be tuned for approximately correct deflection at reference speed
     * @warning Integrator must be frozen on ground to prevent wind-up during takeoff/landing
     */
    float _get_rate_out(float desired_rate, float scaler, bool disable_integrator, float aspeed, bool ground_mode);

    /**
     * @brief Check if airspeed is below minimum safe threshold
     * 
     * @param[in] aspeed  Equivalent airspeed in m/s
     * 
     * @return True if airspeed is below threshold (approaching stall)
     * 
     * @details Used to disable rate PID during slow flight or stall conditions.
     *          When true, integrator is frozen to prevent wind-up and rate
     *          control is disabled to avoid deep stall.
     * 
     * @note Implementation must compare aspeed against vehicle-specific minimum
     * @warning Critical for preventing stall/spin - must be implemented conservatively
     */
    virtual bool is_underspeed(const float aspeed) const = 0;

    /**
     * @brief Get current equivalent airspeed from AHRS
     * 
     * @return Equivalent airspeed in m/s
     * 
     * @details Typically reads from AP::ahrs() with EAS2TAS conversion.
     *          EAS (Equivalent Airspeed) accounts for air density and is
     *          used for control surface effectiveness calculations.
     * 
     * @note EAS = TAS × sqrt(ρ/ρ₀) where ρ is air density at altitude
     * @note Important for high altitude flight where TAS ≠ EAS
     */
    virtual float get_airspeed() const = 0;

    /**
     * @brief Get measured body rate for this axis from AHRS
     * 
     * @return Measured rate in radians/second
     * 
     * @details Derived classes return axis-specific gyro measurements:
     *          - Roll controller: gyro_x (body frame X-axis rate)
     *          - Pitch controller: gyro_y (body frame Y-axis rate)
     *          
     *          Body frame convention:
     *          - Roll (X): positive = right wing down
     *          - Pitch (Y): positive = nose up
     * 
     * @note Values obtained from AP::ahrs() gyro measurements
     */
    virtual float get_measured_rate() const = 0;

    /// Axis identifier for autotune (ROLL, PITCH, or YAW)
    const AP_AutoTune::ATType autotune_type;
};
