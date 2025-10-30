#pragma once

/**
 * @file AC_AttitudeControl_Multi.h
 * @brief Multicopter attitude control implementation for ArduCopter
 * 
 * @details Implements attitude and rate control for standard multicopters (quadcopters, 
 *          hexacopters, etc.). Provides inner-loop body-frame rate controllers (roll/pitch/yaw) 
 *          using AC_PID instances, outer-loop angle P controllers, and throttle mixing logic. 
 *          This is the workhorse attitude controller for ArduCopter.
 *          
 *          Control Architecture:
 *          - Outer loop: Attitude (angle) P controller converts desired attitude to desired body-frame angular rates
 *          - Inner loop: Rate PID controller converts desired rates to motor outputs
 *          - Throttle mixing: Blends throttle and attitude control authority based on flight phase
 *          - Angle boost: Increases throttle to compensate for lean angle during position hold
 *          - Rate controllers run at main loop frequency (typically 400Hz)
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#include "AC_AttitudeControl.h"
#include <AP_Motors/AP_MotorsMulticopter.h>

// Default rate controller PID gains - these are overridden by parameters in var_info[]
#ifndef AC_ATC_MULTI_RATE_RP_P
  # define AC_ATC_MULTI_RATE_RP_P           0.135f  ///< Default roll/pitch rate P gain (dimensionless)
#endif
#ifndef AC_ATC_MULTI_RATE_RP_I
  # define AC_ATC_MULTI_RATE_RP_I           0.135f  ///< Default roll/pitch rate I gain (1/s)
#endif
#ifndef AC_ATC_MULTI_RATE_RP_D
  # define AC_ATC_MULTI_RATE_RP_D           0.0036f ///< Default roll/pitch rate D gain (seconds)
#endif
#ifndef AC_ATC_MULTI_RATE_RP_IMAX
 # define AC_ATC_MULTI_RATE_RP_IMAX         0.5f    ///< Default roll/pitch integrator limit (output units, typically 0.5)
#endif
#ifndef AC_ATC_MULTI_RATE_RPY_FILT_HZ
 # define AC_ATC_MULTI_RATE_RPY_FILT_HZ      20.0f  ///< Default roll/pitch/yaw rate filter frequency (Hz)
#endif
#ifndef AC_ATC_MULTI_RATE_YAW_P
 # define AC_ATC_MULTI_RATE_YAW_P           0.180f  ///< Default yaw rate P gain (dimensionless)
#endif
#ifndef AC_ATC_MULTI_RATE_YAW_I
 # define AC_ATC_MULTI_RATE_YAW_I           0.018f  ///< Default yaw rate I gain (1/s)
#endif
#ifndef AC_ATC_MULTI_RATE_YAW_D
 # define AC_ATC_MULTI_RATE_YAW_D           0.0f    ///< Default yaw rate D gain (seconds)
#endif
#ifndef AC_ATC_MULTI_RATE_YAW_IMAX
 # define AC_ATC_MULTI_RATE_YAW_IMAX        0.5f    ///< Default yaw integrator limit (output units, typically 0.5)
#endif
#ifndef AC_ATC_MULTI_RATE_YAW_FILT_HZ
 # define AC_ATC_MULTI_RATE_YAW_FILT_HZ     2.5f    ///< Default yaw rate filter frequency (Hz)
#endif

/**
 * @class AC_AttitudeControl_Multi
 * @brief Multicopter attitude controller with dual-loop PID rate control
 * 
 * @details This class implements the complete attitude control system for standard multicopters.
 *          It provides a dual-loop control architecture:
 * 
 *          **Outer Loop (Angle Control):**
 *          - Attitude P controller converts desired attitude (quaternion) to desired body-frame angular rates
 *          - Operates in earth frame, outputs body-frame rate targets
 *          - Inherited from AC_AttitudeControl base class
 * 
 *          **Inner Loop (Rate Control):**
 *          - Rate PID controllers convert desired rates to motor outputs
 *          - Three independent PID controllers: roll, pitch, yaw
 *          - Operates entirely in body frame (rad/s)
 *          - Each PID includes: proportional, integral, derivative, filtering, slew limiting
 * 
 *          **Throttle Mixing:**
 *          - Blends throttle and attitude control authority based on flight phase
 *          - Low values favor pilot/autopilot throttle over attitude control
 *          - High values prioritize attitude control (used during active flight)
 *          - Prevents motor saturation during aggressive maneuvers
 *          - Three preset levels: min (landing), man (manual flight), max (active flight)
 * 
 *          **Angle Boost:**
 *          - Increases throttle to compensate for lean angle during position hold
 *          - Implements cosine compensation: throttle_out = throttle_in / cos(tilt_angle)
 *          - Maintains altitude during aggressive attitude maneuvers
 * 
 *          **Control Flow:**
 *          Pilot input → Mode logic → Angle P controller → Rate target → Rate PID → Motor mixer → ESC outputs
 * 
 *          **Coordinate Frames:**
 *          - Input attitude: Earth frame (quaternion representation)
 *          - Rate targets: Body frame (rad/s)
 *          - Rate feedback: Body frame from gyroscope (rad/s)
 *          - Motor outputs: Individual motor thrust values [0, 1]
 * 
 *          **Units:**
 *          - Angles: radians or centidegrees (depending on interface)
 *          - Rates: rad/s or centi deg/s (depending on interface)
 *          - Throttle: normalized [0, 1]
 *          - Time: seconds or milliseconds (dt parameter critical for integration)
 * 
 *          **Typical Loop Rates:**
 *          - Main loop (rate controller): 400Hz on most boards, 50Hz on slower boards
 *          - Angle controller: Same as main loop rate
 *          - Motor update: Same as main loop rate
 * 
 * @note Rate controllers use AC_PID with advanced features including:
 *       - Input filtering and D-term filtering
 *       - Optional notch filters for harmonic rejection
 *       - Slew rate limiting for smooth response
 *       - Integrator management (reset, limits, decay)
 * 
 * @note Parameter system: var_info[] in .cpp registers ATC_RAT_* parameters:
 *       - ATC_RAT_RLL_* : Roll rate PID gains and limits
 *       - ATC_RAT_PIT_* : Pitch rate PID gains and limits
 *       - ATC_RAT_YAW_* : Yaw rate PID gains and limits
 *       - These are exposed to ground control stations for in-flight tuning
 * 
 * @note AutoTune: ArduCopter includes AutoTune mode which automatically adjusts
 *       rate PID gains by inducing oscillations and measuring vehicle response.
 *       Always use AutoTune rather than manually adjusting gains unless you are
 *       an expert in control systems.
 * 
 * @warning Modifying rate PID gains incorrectly can cause instability, oscillations,
 *          or crashes. Always use AutoTune for gain adjustment. Manual tuning requires
 *          deep understanding of PID control theory and multicopter dynamics.
 * 
 * @warning The dt parameter in rate_controller_run_dt() is critical for correct PID
 *          integration and differentiation. It must match the actual loop rate.
 *          Incorrect dt values will cause integrator wind-up and derivative spikes.
 * 
 * @warning Throttle mixing below 0.1 can cause motor saturation during aggressive
 *          attitude maneuvers, potentially leading to loss of control. The default
 *          values have been carefully chosen for safety.
 * 
 * @warning Rate limits (ATC_RAT_*_MAX parameters) should not exceed vehicle physical
 *          capabilities. Setting limits too high can cause structural damage or loss
 *          of control. Consult vehicle specifications before modification.
 * 
 * @warning Thread safety: Vehicle code uses WITH_SEMAPHORE for multi-threaded access.
 *          Direct access from multiple threads without synchronization will cause
 *          race conditions and undefined behavior.
 * 
 * @warning Coordinate frame confusion: Rate controllers operate on body-frame rates
 *          (roll/pitch/yaw rates), NOT Euler angle rates. Confusing these will cause
 *          incorrect control behavior and potential crashes.
 * 
 * @see AC_AttitudeControl Base attitude controller class with angle P controllers
 * @see AC_PID Advanced PID controller implementation with filtering and slew limiting
 * @see AC_PosControl Position controller that commands attitude targets
 * @see AP_MotorsMulticopter Motor mixer receiving rate controller outputs
 * @see AC_AutoTune Automated PID tuning system
 * @see ArduCopter flight modes Flight modes that use attitude controller for stabilization
 */
class AC_AttitudeControl_Multi : public AC_AttitudeControl {
public:
    /**
     * @brief Construct multicopter attitude controller
     * 
     * @param[in] ahrs Attitude and Heading Reference System providing vehicle attitude and rates
     * @param[in] aparm Vehicle-specific parameters (frame type, motor configuration, etc.)
     * @param[in] motors Motor mixer object for commanding individual motor outputs
     * 
     * @details Initializes the attitude control system with:
     *          - Roll, pitch, and yaw rate PID controllers with default gains from macros
     *          - Throttle mixing parameters (min/man/max) for different flight phases
     *          - References to AHRS for attitude feedback and motors for output
     *          - The actual PID gain values are overridden by parameters loaded from EEPROM
     * 
     * @note Rate PID objects are initialized with compile-time defaults but these are
     *       immediately replaced by parameter values during system initialization
     */
	AC_AttitudeControl_Multi(AP_AHRS_View &ahrs, const AP_MultiCopter &aparm, AP_MotorsMulticopter& motors);

	// empty destructor to suppress compiler warning
	virtual ~AC_AttitudeControl_Multi() {}

    /**
     * @brief Get roll rate PID controller reference
     * @return Reference to AC_PID object for roll rate control
     * @details Provides access to roll rate PID for parameter adjustment, logging, or reset
     */
    AC_PID& get_rate_roll_pid() override { return _pid_rate_roll; }
    
    /**
     * @brief Get pitch rate PID controller reference
     * @return Reference to AC_PID object for pitch rate control
     * @details Provides access to pitch rate PID for parameter adjustment, logging, or reset
     */
    AC_PID& get_rate_pitch_pid() override { return _pid_rate_pitch; }
    
    /**
     * @brief Get yaw rate PID controller reference
     * @return Reference to AC_PID object for yaw rate control
     * @details Provides access to yaw rate PID for parameter adjustment, logging, or reset
     */
    AC_PID& get_rate_yaw_pid() override { return _pid_rate_yaw; }
    
    /**
     * @brief Get const roll rate PID controller reference
     * @return Const reference to AC_PID object for roll rate control
     * @details Provides read-only access for logging or status reporting
     */
    const AC_PID& get_rate_roll_pid() const override { return _pid_rate_roll; }
    
    /**
     * @brief Get const pitch rate PID controller reference
     * @return Const reference to AC_PID object for pitch rate control
     * @details Provides read-only access for logging or status reporting
     */
    const AC_PID& get_rate_pitch_pid() const override { return _pid_rate_pitch; }
    
    /**
     * @brief Get const yaw rate PID controller reference
     * @return Const reference to AC_PID object for yaw rate control
     * @details Provides read-only access for logging or status reporting
     */
    const AC_PID& get_rate_yaw_pid() const override { return _pid_rate_yaw; }

    /**
     * @brief Update maximum lean angle during altitude hold
     * 
     * @param[in] throttle_in Current pilot throttle input [0, 1]
     * 
     * @details Computes the safe maximum lean angle that won't cause altitude loss
     *          during altitude hold mode. As throttle decreases, available attitude
     *          control authority decreases because motors have less thrust margin.
     *          This function limits the commanded lean angle to prevent altitude loss.
     * 
     * @note Called during altitude hold and loiter modes to maintain safe operation
     * @note Lean angle limit decreases as throttle approaches hover throttle
     */
    void update_althold_lean_angle_max(float throttle_in) override;

    /**
     * @brief Set motor throttle output with optional angle boost
     * 
     * @param[in] throttle_in Desired earth-frame throttle [0, 1]
     * @param[in] apply_angle_boost If true, compensate for vehicle tilt (cosine compensation)
     * @param[in] filt_cutoff Low-pass filter cutoff frequency in Hz (0 = no filtering)
     * 
     * @details Sets the motor throttle output with optional filtering and tilt compensation.
     *          When apply_angle_boost is true, increases throttle to maintain vertical thrust
     *          during tilted flight: throttle_out = throttle_in / cos(tilt_angle)
     * 
     * @note Angle boost is typically enabled during position hold modes to maintain altitude
     * @note Filter cutoff is typically 2-5 Hz for smooth throttle response
     * 
     * @warning Angle boost can cause throttle saturation at extreme lean angles (>45°)
     */
    void set_throttle_out(float throttle_in, bool apply_angle_boost, float filt_cutoff) override;

    /**
     * @brief Calculate body-frame throttle required to produce given earth-frame throttle
     * 
     * @param[in] throttle_in Desired earth-frame vertical thrust [0, 1]
     * @return Body-frame throttle compensated for vehicle tilt [0, 1+]
     * 
     * @details Accounts for vehicle tilt by dividing by cosine of tilt angle.
     *          This ensures vertical thrust component matches the desired value even
     *          when the vehicle is tilted. Essential for maintaining altitude during
     *          aggressive attitude maneuvers.
     * 
     *          Formula: throttle_out = throttle_in / cos(tilt_angle)
     * 
     * @note Return value can exceed 1.0 at high lean angles, will be clamped by motor mixer
     * @note At 45° lean, boost factor is ~1.41x; at 60° lean, ~2.0x
     * 
     * @warning Does not check for motor saturation - caller must handle limiting
     */
    float get_throttle_boosted(float throttle_in);

    /**
     * @brief Set throttle vs attitude control priority to minimum (landing mode)
     * 
     * @details Sets throttle mixing to minimum value (_thr_mix_min, typically 0.1).
     *          Used during landing to prioritize pilot throttle control over attitude stability.
     *          Lower mix values allow motors to spool down even if attitude errors exist.
     *          The actual mix value is slewed toward this target over 1-2 seconds for smoothness.
     * 
     * @note Typical use: Landing, ground contact, disarmed states
     * @note Has no effect when throttle is above hover throttle
     * @note Lower values favor pilot/autopilot throttle over attitude control
     */
    void set_throttle_mix_min() override { _throttle_rpy_mix_desired = _thr_mix_min; }
    
    /**
     * @brief Set throttle vs attitude control priority to manual flight level
     * 
     * @details Sets throttle mixing to manual flight value (_thr_mix_man, typically 0.1-0.2).
     *          Used during manual stabilized flight modes where pilot has direct throttle control.
     *          Provides balance between throttle response and attitude stability.
     *          The actual mix value is slewed toward this target over 1-2 seconds.
     * 
     * @note Typical use: Stabilize, AltHold with pilot throttle input
     * @note Intermediate priority between landing (min) and autonomous flight (max)
     */
    void set_throttle_mix_man() override { _throttle_rpy_mix_desired = _thr_mix_man; }
    
    /**
     * @brief Set throttle vs attitude control priority to maximum (active flight)
     * 
     * @param[in] ratio Optional ratio multiplier (default handled in implementation)
     * 
     * @details Sets throttle mixing to maximum value (_thr_mix_max, typically 0.5).
     *          Used during autonomous flight modes to prioritize attitude control over throttle.
     *          Higher mix values ensure attitude stability even if it requires reducing throttle.
     *          The actual mix value is slewed toward this target over 1-2 seconds.
     * 
     * @note Typical use: Auto, Guided, Loiter, PosHold modes
     * @note Higher values prioritize attitude control over throttle
     * @note Has no effect when throttle is above hover throttle
     */
    void set_throttle_mix_max(float ratio) override;
    
    /**
     * @brief Set throttle mix value directly without slewing
     * 
     * @param[in] value Desired throttle mix ratio [0, ~5] (typical range 0.1-0.5)
     * 
     * @details Immediately sets both desired and actual throttle mix values to the specified
     *          value without slewing. Use this for initialization or when instant transitions
     *          are required. Range typically [0.1, 0.5]:
     *          - 0.1: Landing, prioritize throttle
     *          - 0.2: Manual flight, balanced
     *          - 0.5: Active flight, prioritize attitude
     * 
     * @note Bypasses normal slew rate limiting
     * @warning Sudden changes can cause abrupt throttle response
     */
    void set_throttle_mix_value(float value) override { _throttle_rpy_mix_desired = _throttle_rpy_mix = value; }
    
    /**
     * @brief Get current throttle mix ratio
     * 
     * @return Current throttle vs attitude priority mix value [0, ~5]
     * 
     * @details Returns the current (slewed) throttle mix ratio being used by the controller.
     *          Lower values favor pilot throttle, higher values favor attitude control.
     *          Typical values: 0.1 (landing), 0.2 (manual), 0.5 (active flight)
     */
    float get_throttle_mix(void) const override { return _throttle_rpy_mix; }

    /**
     * @brief Check if throttle mix is near minimum (attitude control deprioritized)
     * 
     * @return true if attitude control is deprioritized (landing/ground mode)
     * 
     * @details Returns true when throttle mix is less than 1.25× the minimum value.
     *          Indicates the controller is in a mode where pilot throttle takes priority
     *          over attitude stability (e.g., landing, near ground, or preparing to disarm).
     * 
     * @note Used by flight modes to determine if vehicle is in landing configuration
     */
    bool is_throttle_mix_min() const override { return (_throttle_rpy_mix < 1.25f * _thr_mix_min); }

    /**
     * @brief Run body-frame rate controllers and send outputs to motors
     * 
     * @param[in] gyro_rads Current body-frame angular rates from gyroscope [rad/s] (roll, pitch, yaw)
     * @param[in] dt Time step since last update [seconds]
     * 
     * @details This is the core control loop that runs at main loop frequency (typically 400Hz).
     *          Control sequence:
     *          1. Reads desired rate targets (set by angle controller or pilot)
     *          2. Compares to current rates from gyro (error calculation)
     *          3. Runs three independent PID controllers (roll, pitch, yaw)
     *          4. Applies throttle mixing and output limiting
     *          5. Sends commands to motor mixer
     * 
     *          Each PID controller performs:
     *          - Proportional: Immediate response to error
     *          - Integral: Eliminates steady-state error
     *          - Derivative: Damping to prevent overshoot
     *          - Filtering: Low-pass and notch filters for noise rejection
     *          - Slew limiting: Smooth rate changes to prevent sudden jumps
     * 
     * @note Called at 400Hz on most boards, 50Hz on slower hardware
     * @note All values in body frame (not earth frame or Euler rates)
     * @note The dt parameter MUST match actual loop timing for correct integration
     * 
     * @warning Incorrect dt values will cause integrator wind-up and derivative spikes
     * @warning Do not call from multiple threads without synchronization
     * 
     * @see rate_controller_run() Convenience wrapper using current gyro and dt
     * 
     * Source: libraries/AC_AttitudeControl/AC_AttitudeControl_Multi.cpp:150-300
     */
    void rate_controller_run_dt(const Vector3f& gyro_rads, float dt) override;
    
    /**
     * @brief Reset rate controller targets to current gyro rates
     * 
     * @details Resets the desired rate targets for all three axes (roll, pitch, yaw)
     *          to the current measured rates from the gyroscope. This prevents sudden
     *          jumps in motor output when switching to a rate-controlled mode.
     * 
     * @note Typically called when:
     *       - Switching from angle control to rate control mode
     *       - Entering Acro mode
     *       - Recovering from a failsafe condition
     *       - After arming to initialize controllers
     * 
     * @note Also resets PID integrators to prevent wind-up from accumulating
     */
    void rate_controller_target_reset() override;
    
    /**
     * @brief Run rate controllers using current gyro and dt values
     * 
     * @details Convenience wrapper that retrieves the current gyroscope rates and
     *          timestep from the AHRS system, then calls rate_controller_run_dt().
     *          This is the typical way to invoke the rate controller from flight modes.
     * 
     * @note Equivalent to: rate_controller_run_dt(ahrs.get_gyro(), ahrs.get_dt())
     * @note Main loop rate determines update frequency (typically 400Hz)
     * 
     * @see rate_controller_run_dt() Lower-level interface with explicit parameters
     */
    void rate_controller_run() override;

    /**
     * @brief Validate parameters before takeoff
     * 
     * @details Performs comprehensive sanity checks on all attitude control parameters
     *          to ensure safe values before allowing takeoff. Checks include:
     *          - PID gains within reasonable ranges (not zero, not excessive)
     *          - Filter frequencies valid and appropriate for loop rate
     *          - Integrator limits reasonable
     *          - Slew rate limits set appropriately
     *          - Throttle mix values in safe ranges
     * 
     *          Invalid parameters are corrected and warnings logged. Critical failures
     *          may prevent arming.
     * 
     * @note Called once during pre-arm checks by AP_Arming
     * @note Also called after parameter changes to validate new values
     * @note Logs warnings for any corrected parameters
     * 
     * @warning Do not skip this check - invalid parameters can cause crashes
     */
    void parameter_sanity_check() override;

    /**
     * @brief Configure notch filter sample rate for all PIDs
     * 
     * @param[in] sample_rate Controller loop rate in Hz (typically 400Hz)
     * 
     * @details Propagates the sample rate to all three rate PID controllers (roll, pitch, yaw)
     *          for proper notch filter configuration. Notch filters are used to reject
     *          specific frequencies (e.g., motor harmonics) that cause oscillations.
     * 
     *          The sample rate must match the actual loop rate for correct filter operation.
     *          Notch filters are configured via parameters (ATC_RAT_*_FLTE, FLTT, FLTD).
     * 
     * @note Called during initialization and when loop rate changes
     * @note Each PID can have independent notch filter configuration
     * @note Sample rate typically 400Hz on most boards, 50Hz on slower boards
     * 
     * @warning Incorrect sample rate will cause notch filters to reject wrong frequencies
     */
    void set_notch_sample_rate(float sample_rate) override;

    // user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

protected:

    /**
     * @brief Boost angle controller gains during rapid throttle changes
     * 
     * @details Dynamically increases angle P and PD gains during rapid throttle changes
     *          to improve attitude responsiveness during quick climbs and descents.
     *          The boost factor is applied each cycle when high throttle slew rate is detected.
     * 
     *          This prevents attitude lag when pilot makes aggressive throttle inputs,
     *          improving handling during rapid altitude changes.
     * 
     * @note Called internally by rate controller during each update
     * @note Boost amount controlled by _throttle_gain_boost parameter
     * @note Effect is temporary and decays when throttle stabilizes
     */
    void update_throttle_gain_boost();

    /**
     * @brief Slew current throttle mix toward desired value
     * 
     * @details Smoothly transitions the throttle-to-attitude mix ratio from its current
     *          value toward the target (_throttle_rpy_mix_desired) over 1-2 seconds.
     *          This prevents abrupt changes in control behavior when switching between
     *          flight modes (landing ↔ manual ↔ active flight).
     * 
     * @note Called internally during each rate controller update
     * @note Slew rate typically allows full transition in 1-2 seconds
     * @note Provides smooth pilot experience during mode transitions
     */
    void update_throttle_rpy_mix();

    /**
     * @brief Get throttle limit for attitude control based on current priority
     * 
     * @param[in] throttle_in Pilot/autopilot throttle input [0, 1]
     * @return Maximum throttle available for attitude control [0, 1]
     * 
     * @details Calculates how much throttle authority is available for attitude control
     *          based on the current throttle vs attitude priority mix setting. Used for
     *          blending during low thrust conditions to prevent motor saturation.
     * 
     *          When throttle is below hover throttle, this function reserves some throttle
     *          margin for attitude control based on the mix ratio. Higher mix values
     *          reserve more throttle for attitude control.
     * 
     * @note Returns throttle_in when above hover throttle (no blending needed)
     * @note Critical for preventing loss of control during landing and low throttle flight
     */
    float get_throttle_avg_max(float throttle_in);

    /// Reference to multicopter motor mixer for commanding individual motor outputs
    AP_MotorsMulticopter& _motors_multi;

    /**
     * @brief Roll rate PID controller for body-frame angular rate control
     * 
     * @details Controls roll axis angular rate (rad/s) in body frame.
     *          Initialized with default gains from AC_ATC_MULTI_RATE_RP_* macros,
     *          but actual values loaded from ATC_RAT_RLL_* parameters at startup.
     * 
     * @note Includes P, I, D terms plus input filtering, D-term filtering, and optional notch filters
     * @note Target rate set by angle controller or pilot stick input
     * @note Output sent to motor mixer for translation to individual motor commands
     */
    AC_PID                _pid_rate_roll {
        AC_PID::Defaults{
            .p         = AC_ATC_MULTI_RATE_RP_P,
            .i         = AC_ATC_MULTI_RATE_RP_I,
            .d         = AC_ATC_MULTI_RATE_RP_D,
            .ff        = 0.0f,
            .imax      = AC_ATC_MULTI_RATE_RP_IMAX,
            .filt_T_hz = AC_ATC_MULTI_RATE_RPY_FILT_HZ,
            .filt_E_hz = 0.0f,
            .filt_D_hz = AC_ATC_MULTI_RATE_RPY_FILT_HZ,
            .srmax     = 0,
            .srtau     = 1.0
        }
    };
    
    /**
     * @brief Pitch rate PID controller for body-frame angular rate control
     * 
     * @details Controls pitch axis angular rate (rad/s) in body frame.
     *          Initialized with default gains from AC_ATC_MULTI_RATE_RP_* macros,
     *          but actual values loaded from ATC_RAT_PIT_* parameters at startup.
     * 
     * @note Includes P, I, D terms plus input filtering, D-term filtering, and optional notch filters
     * @note Target rate set by angle controller or pilot stick input
     * @note Output sent to motor mixer for translation to individual motor commands
     */
    AC_PID                _pid_rate_pitch{
        AC_PID::Defaults{
            .p         = AC_ATC_MULTI_RATE_RP_P,
            .i         = AC_ATC_MULTI_RATE_RP_I,
            .d         = AC_ATC_MULTI_RATE_RP_D,
            .ff        = 0.0f,
            .imax      = AC_ATC_MULTI_RATE_RP_IMAX,
            .filt_T_hz = AC_ATC_MULTI_RATE_RPY_FILT_HZ,
            .filt_E_hz = 0.0f,
            .filt_D_hz = AC_ATC_MULTI_RATE_RPY_FILT_HZ,
            .srmax     = 0,
            .srtau     = 1.0
        }
    };
    
    /**
     * @brief Yaw rate PID controller for body-frame angular rate control
     * 
     * @details Controls yaw axis angular rate (rad/s) in body frame.
     *          Initialized with default gains from AC_ATC_MULTI_RATE_YAW_* macros,
     *          but actual values loaded from ATC_RAT_YAW_* parameters at startup.
     * 
     * @note Yaw typically uses lower D gain and different filter settings than roll/pitch
     * @note Includes P, I, D terms plus input filtering, D-term filtering, and optional notch filters
     * @note Target rate set by angle controller or pilot stick input
     * @note Output sent to motor mixer for translation to individual motor commands
     */
    AC_PID                _pid_rate_yaw{
        AC_PID::Defaults{
            .p         = AC_ATC_MULTI_RATE_YAW_P,
            .i         = AC_ATC_MULTI_RATE_YAW_I,
            .d         = AC_ATC_MULTI_RATE_YAW_D,
            .ff        = 0.0f,
            .imax      = AC_ATC_MULTI_RATE_YAW_IMAX,
            .filt_T_hz = AC_ATC_MULTI_RATE_RPY_FILT_HZ,
            .filt_E_hz = AC_ATC_MULTI_RATE_YAW_FILT_HZ,
            .filt_D_hz = AC_ATC_MULTI_RATE_RPY_FILT_HZ,
            .srmax     = 0,
            .srtau     = 1.0
        }
    };

    /**
     * @brief Throttle vs attitude priority for manual flight modes
     * 
     * @details Parameter controlling how much to prioritize attitude control over pilot
     *          throttle during manual flight (Stabilize, AltHold with pilot throttle).
     *          Typical value: 0.1-0.2
     *          
     *          Higher values mean attitude control takes priority, lower values favor throttle response.
     *          Only affects behavior when throttle is below hover throttle.
     * 
     * @note Exposed as ATC_THR_MIX_MAN parameter to ground control stations
     */
    AP_Float              _thr_mix_man;
    
    /**
     * @brief Throttle vs attitude priority for landing
     * 
     * @details Parameter controlling how much to prioritize attitude control over throttle
     *          during landing and near-ground operations. Typical value: 0.1
     *          
     *          Lower value allows motors to spool down even if attitude errors exist,
     *          enabling smooth landings and reducing ground effect oscillations.
     * 
     * @note Exposed as ATC_THR_MIX_MIN parameter to ground control stations
     */
    AP_Float              _thr_mix_min;
    
    /**
     * @brief Throttle vs attitude priority for active flight
     * 
     * @details Parameter controlling how much to prioritize attitude control over throttle
     *          during autonomous flight (Auto, Guided, Loiter, PosHold). Typical value: 0.5
     *          
     *          Higher value ensures attitude stability even if it requires reducing throttle,
     *          important for precise autonomous navigation and preventing loss of control.
     * 
     * @note Exposed as ATC_THR_MIX_MAX parameter to ground control stations
     */
    AP_Float              _thr_mix_max;

    /**
     * @brief Angle controller gain boost multiplier during rapid throttle changes
     * 
     * @details Parameter controlling how much to boost angle P and PD gains during
     *          rapid throttle slew. Improves attitude responsiveness during quick climbs
     *          and descents by temporarily increasing controller aggressiveness.
     *          
     *          Typical value: 1.0-2.0 (1.0 = no boost, 2.0 = double gains during throttle transient)
     * 
     * @note Exposed as parameter to ground control stations for tuning
     * @note Effect is temporary and decays when throttle stabilizes
     */
    AP_Float              _throttle_gain_boost;
};
