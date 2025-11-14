#pragma once

/**
 * @file AC_AttitudeControl_Sub.h
 * @brief ArduSub (underwater vehicle) attitude control specialization
 * 
 * @details Extends AC_AttitudeControl for underwater vehicles (ROVs and AUVs).
 *          This specialized controller adjusts default PID gains for underwater
 *          dynamics, implements sub-specific throttle mixing, and adapts lean
 *          angle limits to account for water resistance and neutral buoyancy.
 *          
 *          Key differences from aerial multirotors:
 *          - Lower default gains due to water damping
 *          - Different throttle mixing strategies for depth control
 *          - Water resistance provides natural damping (reduced D term)
 *          - Neutral buoyancy affects vertical thrust baseline
 *          - Current/surge effects similar to wind on aerial vehicles
 * 
 * @note Coordinate frame remains NED (North-East-Down) even underwater
 * @warning Underwater dynamics are fundamentally different from air - do not
 *          use aerial copter gains without significant retuning
 * 
 * @see AC_AttitudeControl Base attitude control class
 * @see ArduSub Vehicle firmware in ArduSub/ directory
 * @see AP_MotorsMulticopter Motor mixing for vectored thrusters
 */

#include "AC_AttitudeControl.h"
#include <AP_Motors/AP_MotorsMulticopter.h>

/**
 * @name Default Angle Controller PID Gains
 * @brief Sub-specific defaults for attitude angle control
 * @{
 */
/// @brief Angle controller proportional gain (deg to deg/s conversion)
/// @note Lower than aerial copters due to water damping
#define AC_ATC_SUB_ANGLE_P             6.0f

/// @brief Maximum yaw acceleration in centidegrees/s/s
/// @note Tuned for underwater yaw authority with vectored thrusters
#define AC_ATC_SUB_ACCEL_Y_MAX         110000.0f
/** @} */

/**
 * @name Default Rate Controller PID Gains
 * @brief Sub-specific defaults for body-frame angular rate control
 * @details These gains are tuned for underwater dynamics where water resistance
 *          provides significant natural damping. Values are considerably lower
 *          than aerial multirotors.
 * @{
 */
/// @brief Roll/pitch rate controller proportional gain
#define AC_ATC_SUB_RATE_RP_P           0.135f

/// @brief Roll/pitch rate controller integral gain
#define AC_ATC_SUB_RATE_RP_I           0.090f

/// @brief Roll/pitch rate controller derivative gain (low due to water damping)
#define AC_ATC_SUB_RATE_RP_D           0.0036f

/// @brief Roll/pitch rate controller integral maximum (radians/s)
#define AC_ATC_SUB_RATE_RP_IMAX        0.444f

/// @brief Roll/pitch rate controller filter cutoff frequency (Hz)
#define AC_ATC_SUB_RATE_RP_FILT_HZ     30.0f

/// @brief Yaw rate controller proportional gain
#define AC_ATC_SUB_RATE_YAW_P          0.180f

/// @brief Yaw rate controller integral gain
#define AC_ATC_SUB_RATE_YAW_I          0.018f

/// @brief Yaw rate controller derivative gain (typically zero for yaw)
#define AC_ATC_SUB_RATE_YAW_D          0.0f

/// @brief Yaw rate controller integral maximum (radians/s)
#define AC_ATC_SUB_RATE_YAW_IMAX       0.222f

/// @brief Yaw rate controller filter cutoff frequency (Hz, lower than roll/pitch)
#define AC_ATC_SUB_RATE_YAW_FILT_HZ    5.0f
/** @} */

/// @brief Maximum yaw error before triggering slew rate limiting (5 degrees in radians)
/// @details Prevents large instantaneous yaw corrections that could destabilize the vehicle
#define MAX_YAW_ERROR                  radians(5)

/**
 * @class AC_AttitudeControl_Sub
 * @brief Underwater vehicle attitude controller
 * 
 * @details Provides attitude control specifically tuned for underwater vehicles (ROVs
 *          and AUVs) which have fundamentally different dynamics than aerial multirotors.
 *          
 *          **Underwater Dynamics Considerations:**
 *          - Water resistance: ~800x denser than air, providing significant damping
 *          - Neutral buoyancy: Vehicles operate near zero-g conditions vertically
 *          - Drag forces: Velocity-dependent forces dominate over inertial effects
 *          - Thruster dynamics: Water resistance affects thruster response
 *          - Current/surge: Underwater currents affect control like wind on aircraft
 *          
 *          **Control Differences from Aerial Multirotors:**
 *          - Lower PID gains due to natural water damping
 *          - Reduced derivative terms (water provides damping)
 *          - Different throttle mixing for depth control vs altitude hold
 *          - Throttle represents vertical thrust for depth control
 *          - Slower response times acceptable due to operational environment
 *          
 *          **Throttle Interpretation:**
 *          In ArduSub, "throttle" controls vertical thrust for depth hold and
 *          vertical motion, analogous to altitude control in aircraft but operating
 *          around a neutral buoyancy point rather than fighting gravity.
 *          
 * @note Default PID gains are tuned for typical underwater vehicle dynamics
 * @note Uses same AC_PID class as aerial vehicles but with different defaults
 * @note var_info[] in .cpp registers sub-specific parameters
 * 
 * @warning Underwater dynamics are very different from air - do not use
 *          aerial copter gains without extensive retuning and testing
 * @warning Water density (1000 kg/m³ vs air 1.225 kg/m³) affects control authority
 * @warning Coordinate frame still NED (North-East-Down) even underwater
 * @warning Current/surge can significantly affect attitude control effectiveness
 * 
 * @see AC_AttitudeControl Base class for all attitude controllers
 * @see AC_AttitudeControl_Multi Multicopter base implementation
 * @see ArduSub/Attitude.cpp Sub-specific attitude control wrapper functions
 * @see AP_MotorsMulticopter Motor mixer used for vectored thrusters
 */
class AC_AttitudeControl_Sub : public AC_AttitudeControl {
public:
    /**
     * @brief Construct ArduSub attitude controller
     * 
     * @details Initializes attitude controller with sub-specific default PID gains
     *          tuned for underwater dynamics. Sets up rate controllers for roll,
     *          pitch, and yaw with appropriate filtering and limits.
     * 
     * @param[in] ahrs Reference to AHRS (Attitude Heading Reference System)
     * @param[in] aparm Reference to vehicle parameters (multicopter configuration)
     * @param[in] motors Reference to motor mixer for vectored thruster control
     * 
     * @note Constructor initializes PID controllers with underwater-tuned defaults
     * @note Throttle mixing parameters loaded from var_info[] parameter definitions
     */
    AC_AttitudeControl_Sub(AP_AHRS_View &ahrs, const AP_MultiCopter &aparm, AP_MotorsMulticopter& motors);

    /**
     * @brief Virtual destructor
     * @details Empty destructor to suppress compiler warning for virtual base class
     */
    virtual ~AC_AttitudeControl_Sub() {}

    /**
     * @name Rate PID Accessors
     * @brief Provide access to sub-tuned rate controller PID objects
     * @details Returns references to the AC_PID objects configured with
     *          underwater-appropriate gains. These are used by higher-level
     *          code for tuning, logging, and parameter access.
     * @{
     */
    
    /**
     * @brief Get roll rate PID controller (non-const)
     * @return Reference to sub-tuned roll rate AC_PID controller
     * @note Returns controller with underwater dynamics defaults
     */
    AC_PID& get_rate_roll_pid() override { return _pid_rate_roll; }
    
    /**
     * @brief Get pitch rate PID controller (non-const)
     * @return Reference to sub-tuned pitch rate AC_PID controller
     * @note Returns controller with underwater dynamics defaults
     */
    AC_PID& get_rate_pitch_pid() override { return _pid_rate_pitch; }
    
    /**
     * @brief Get yaw rate PID controller (non-const)
     * @return Reference to sub-tuned yaw rate AC_PID controller
     * @note Returns controller with underwater dynamics defaults
     */
    AC_PID& get_rate_yaw_pid() override { return _pid_rate_yaw; }
    
    /**
     * @brief Get roll rate PID controller (const)
     * @return Const reference to sub-tuned roll rate AC_PID controller
     * @note Returns controller with underwater dynamics defaults
     */
    const AC_PID& get_rate_roll_pid() const override { return _pid_rate_roll; }
    
    /**
     * @brief Get pitch rate PID controller (const)
     * @return Const reference to sub-tuned pitch rate AC_PID controller
     * @note Returns controller with underwater dynamics defaults
     */
    const AC_PID& get_rate_pitch_pid() const override { return _pid_rate_pitch; }
    
    /**
     * @brief Get yaw rate PID controller (const)
     * @return Const reference to sub-tuned yaw rate AC_PID controller
     * @note Returns controller with underwater dynamics defaults
     */
    const AC_PID& get_rate_yaw_pid() const override { return _pid_rate_yaw; }
    /** @} */

    /**
     * @brief Update altitude hold (depth hold) lean angle maximum
     * 
     * @details Computes the maximum allowable lean angle during depth hold operations
     *          considering current throttle (vertical thrust) and motor saturation.
     *          In ArduSub, this limits horizontal attitude angles during depth
     *          control to ensure sufficient vertical thrust authority remains available.
     * 
     * @param[in] throttle_in Current throttle input [0.0, 1.0] representing vertical thrust
     * 
     * @note Called during depth hold modes to dynamically adjust lean angle limits
     * @note Prevents excessive tilting that would compromise depth control authority
     * @warning Essential for maintaining depth control during maneuvering
     * 
     * @see set_throttle_out() Output throttle setting function
     */
    void update_althold_lean_angle_max(float throttle_in) override;

    /**
     * @brief Set throttle output with sub-specific mixing
     * 
     * @details Applies throttle output with underwater vehicle-specific mixing between
     *          throttle and attitude control. Implements angle boost compensation to
     *          maintain vertical thrust when vehicle is tilted. Filters throttle input
     *          to smooth depth control responses in water.
     *          
     *          Throttle mixing priority trades off between:
     *          - Attitude control authority (correcting roll/pitch/yaw errors)
     *          - Direct throttle/depth control from pilot or autopilot
     * 
     * @param[in] throttle_in Desired throttle [0.0, 1.0] for vertical thrust
     * @param[in] apply_angle_boost true to apply angle-based throttle compensation
     * @param[in] filt_cutoff Filter cutoff frequency in Hz for throttle smoothing
     * 
     * @note Throttle represents vertical thrust for depth control in ArduSub
     * @note Angle boost compensates for loss of vertical thrust when tilted
     * @warning Throttle mixing affects control authority - tune carefully for vehicle
     * 
     * @see get_throttle_boosted() Angle boost calculation
     * @see update_throttle_rpy_mix() Throttle mixing ratio update
     */
    void set_throttle_out(float throttle_in, bool apply_angle_boost, float filt_cutoff) override;

    /**
     * @brief Calculate body-frame throttle required to produce given earth-frame thrust
     * 
     * @details Accounts for vehicle tilt to maintain desired vertical (earth-frame)
     *          thrust. When the vehicle is tilted, more body-frame thrust is needed
     *          to achieve the same vertical component. This "angle boost" ensures
     *          consistent depth control regardless of attitude.
     *          
     *          Calculation: throttle_out = throttle_in / cos(tilt_angle)
     * 
     * @param[in] throttle_in Desired earth-frame vertical thrust [0.0, 1.0]
     * 
     * @return Body-frame throttle value accounting for vehicle tilt
     * 
     * @note Critical for maintaining depth hold during maneuvering
     * @note Returns higher throttle values when vehicle is tilted
     * @warning Can saturate at high tilt angles - lean angle limits prevent this
     * 
     * @see update_althold_lean_angle_max() Lean angle limit calculation
     */
	float get_throttle_boosted(float throttle_in);

    /**
     * @name Throttle vs Attitude Mixing Control
     * @brief Set desired balance between throttle authority and attitude control
     * @details Throttle mixing determines priority between direct throttle/depth control
     *          and attitude correction. The actual mix slews toward the desired value
     *          over 1-2 seconds to avoid abrupt control transitions.
     *          
     *          Mix value interpretation:
     *          - Low values (min): Favor pilot/autopilot throttle, allow attitude drift
     *          - Medium values (man): Balanced control for manual flight
     *          - High values (max): Prioritize attitude stability, may limit throttle
     *          
     *          In ArduSub context:
     *          - Minimum during landing/surfacing (depth control priority)
     *          - Manual during pilot control (balanced)
     *          - Maximum during active maneuvering (attitude stability priority)
     * 
     * @note Actual mixing ratio slews smoothly to avoid control discontinuities
     * @note Has reduced effect when throttle is near neutral buoyancy point
     * @{
     */
    
    /**
     * @brief Set throttle mix to minimum (prioritize depth control)
     * @details Used during landing, surfacing, or when precise depth control is critical
     * @note Attitude control authority is reduced to favor direct depth commands
     */
    void set_throttle_mix_min() override { _throttle_rpy_mix_desired = _thr_mix_min; }
    
    /**
     * @brief Set throttle mix to manual flight value (balanced control)
     * @details Used during normal manual piloting for balanced depth and attitude control
     * @note Provides good compromise between depth control and attitude stability
     */
    void set_throttle_mix_man() override { _throttle_rpy_mix_desired = _thr_mix_man; }
    
    /**
     * @brief Set throttle mix to maximum (prioritize attitude control)
     * @details Used during active flight and maneuvering when attitude stability is paramount
     * @param[in] ratio Not used in Sub implementation, uses _thr_mix_max parameter
     * @note Maximizes attitude control authority, may limit depth control responsiveness
     */
    void set_throttle_mix_max(float ratio) override { _throttle_rpy_mix_desired = _thr_mix_max; }

    /**
     * @brief Check if throttle mix is near minimum
     * @details Returns true if current mixing ratio indicates attitude control is
     *          deprioritized in favor of direct throttle/depth control
     * @return true if throttle mix < 1.25 * minimum threshold, false otherwise
     * @note Used to detect landing/surfacing conditions where depth control dominates
     */
    bool is_throttle_mix_min() const override { return (_throttle_rpy_mix < 1.25f*_thr_mix_min); }
    /** @} */

    /**
     * @brief Run lowest level body-frame rate controller and send outputs to motors
     * 
     * @details Executes the innermost control loop converting desired angular rates
     *          (body-frame roll/pitch/yaw rates) into motor/thruster commands.
     *          Runs the rate PID controllers and sends outputs to the motor mixer.
     *          
     *          Control flow:
     *          1. Read current angular rates from AHRS
     *          2. Calculate rate errors (desired - actual)
     *          3. Run PID controllers (roll, pitch, yaw)
     *          4. Apply throttle mixing
     *          5. Send commands to motor mixer
     * 
     * @note Called at high frequency (typically 100-400 Hz depending on platform)
     * @note This is the innermost loop - angle control calls this via rate setpoints
     * @warning Must be called at consistent rate for PID derivative term accuracy
     * 
     * @see AC_PID Rate controller implementation
     * @see AP_MotorsMulticopter Motor mixing and output
     */
    void rate_controller_run() override;

    /**
     * @brief Sanity check all attitude control parameters
     * 
     * @details Validates parameter ranges and consistency before vehicle operation.
     *          Checks PID gains, filter frequencies, angle limits, and throttle
     *          mixing parameters. Issues warnings for out-of-range or suspicious values.
     * 
     * @note Should be called once during initialization before vehicle operation
     * @note Does not modify parameters, only logs warnings for invalid values
     * @warning Operating with invalid parameters can lead to unstable control
     * 
     * @see AC_PID::sanity_check() Individual PID parameter validation
     */
    void parameter_sanity_check() override;

    /**
     * @brief Set the PID notch filter sample rate
     * 
     * @details Configures the sample rate for notch filters in the PID controllers.
     *          Notch filters are used to reject specific frequencies (e.g., thruster
     *          resonances, structural vibrations) that could destabilize control.
     * 
     * @param[in] sample_rate Sample rate in Hz for notch filter calculations
     * 
     * @note Must match the actual rate_controller_run() execution frequency
     * @note Called during initialization or when loop rate changes
     * 
     * @see AC_PID::set_notch_sample_rate() Underlying PID notch configuration
     */
    void set_notch_sample_rate(float sample_rate) override;

    /**
     * @brief Set desired attitude with yaw slew rate limiting
     * 
     * @details Sets target roll, pitch, and yaw angles with special handling for yaw.
     *          Yaw is slewed toward the target at a fixed rate until the error is
     *          within 5 degrees, preventing large instantaneous yaw corrections that
     *          could destabilize the vehicle or create excessive yaw rates.
     *          
     *          This enforces smooth, consistent heading changes particularly important
     *          for underwater vehicles where rapid yaw changes can induce coupling
     *          into roll/pitch or cause control saturation.
     * 
     * @param[in] euler_roll_angle_cd  Desired roll angle in centidegrees (NED frame)
     * @param[in] euler_pitch_angle_cd Desired pitch angle in centidegrees (NED frame)
     * @param[in] euler_yaw_angle_cd   Desired yaw angle in centidegrees (NED frame)
     * @param[in] slew_yaw_rate_cds    Maximum yaw rate in centidegrees/s for slewing
     * 
     * @note Roll and pitch are set immediately without slewing
     * @note Yaw slewing stops once error is within MAX_YAW_ERROR (5 degrees)
     * @note Angles are in NED (North-East-Down) earth frame
     * @note Centidegrees used for integer precision (1 centidegree = 0.01 degrees)
     * 
     * @warning Large instantaneous yaw errors can cause heading oscillations
     * @warning Slew rate too high can still cause instability, too low affects responsiveness
     * 
     * @see MAX_YAW_ERROR Maximum yaw error before slewing stops
     */
    void input_euler_angle_roll_pitch_slew_yaw_cd(float euler_roll_angle_cd, float euler_pitch_angle_cd, float euler_yaw_angle_cd, float slew_yaw_rate_cds);

    /**
     * @brief User-settable parameters table
     * @details Parameter group containing sub-specific tuning values including
     *          throttle mixing parameters (_thr_mix_min, _thr_mix_man, _thr_mix_max)
     *          and other underwater vehicle-specific configuration.
     * @see AC_AttitudeControl_Sub.cpp for var_info[] definition
     */
    static const struct AP_Param::GroupInfo var_info[];

protected:

    /**
     * @brief Slew throttle-to-attitude mix ratio toward desired value
     * 
     * @details Updates _throttle_rpy_mix to approach _throttle_rpy_mix_desired over
     *          time. Uses asymmetric slew rates: increases rapidly (fast attitude
     *          control engagement) and decreases more slowly (gentle disengagement)
     *          to ensure stable transitions and prevent control oscillations.
     *          
     *          This smooth transition prevents abrupt changes in control authority
     *          that could destabilize the vehicle during mode changes or throttle
     *          adjustments.
     * 
     * @note Called periodically by set_throttle_out()
     * @note Asymmetric slew rates: fast increase, slow decrease
     * @note Transition time approximately 1-2 seconds
     * 
     * @see set_throttle_mix_min() Set minimum mixing
     * @see set_throttle_mix_man() Set manual mixing
     * @see set_throttle_mix_max() Set maximum mixing
     */
    void update_throttle_rpy_mix();

    /**
     * @brief Calculate averaged throttle accounting for attitude control priority
     * 
     * @details Computes throttle value that balances direct throttle commands with
     *          attitude control requirements. As thrust approaches minimum, gracefully
     *          reduces control authority to prevent motor saturation. Includes
     *          compensation for roll/pitch angle effects on thrust vectoring.
     *          
     *          This prevents attitude control from commanding thrust that would
     *          exceed available motor authority, which could cause loss of control.
     * 
     * @param[in] throttle_in Desired throttle input [0.0, 1.0]
     * 
     * @return Averaged throttle value accounting for attitude control priority [0.0, 1.0]
     * 
     * @note Used internally by rate_controller_run() for motor mixing
     * @note Prevents control saturation at low and high throttle values
     * 
     * @see update_throttle_rpy_mix() Mix ratio update function
     */
    float get_throttle_avg_max(float throttle_in);

    /**
     * @brief Reference to motor mixer for vectored thruster control
     * @details Provides access to motor/thruster output functions for sending
     *          attitude control commands to the vehicle's propulsion system
     * @note Sub uses multicopter-style vectored thrust control
     */
    AP_MotorsMulticopter& _motors_multi;

    /**
     * @name Rate Controller PID Instances
     * @brief Sub-tuned PID controllers for body-frame angular rate control
     * @{
     */
    
    /**
     * @brief Roll and pitch rate PID default gains
     * @details Shared defaults for roll and pitch rate controllers tuned for
     *          underwater dynamics. Lower gains account for water damping and
     *          reduced control authority compared to aerial vehicles.
     * @note P, I, D values significantly lower than aerial multirotors
     * @note Feedforward (ff) set to zero - not typically used in sub control
     * @note Filter frequencies tuned for underwater response characteristics
     */
    const AC_PID::Defaults rp_defaults {
        AC_PID::Defaults{
            .p         = AC_ATC_SUB_RATE_RP_P,      ///< Proportional gain (0.135)
            .i         = AC_ATC_SUB_RATE_RP_I,      ///< Integral gain (0.090)
            .d         = AC_ATC_SUB_RATE_RP_D,      ///< Derivative gain (0.0036, low due to water damping)
            .ff        = 0.0f,                       ///< Feedforward gain (not used)
            .imax      = AC_ATC_SUB_RATE_RP_IMAX,   ///< Integral limit (0.444 rad/s)
            .filt_T_hz = AC_ATC_SUB_RATE_RP_FILT_HZ,///< Target filter cutoff (30 Hz)
            .filt_E_hz = 0.0,                        ///< Error filter cutoff (disabled)
            .filt_D_hz = AC_ATC_SUB_RATE_RP_FILT_HZ,///< Derivative filter cutoff (30 Hz)
            .srmax     = 0,                          ///< Slew rate maximum (disabled)
            .srtau     = 1.0                         ///< Slew rate time constant
        }
    };
    
    /**
     * @brief Roll rate PID controller
     * @details Controls body-frame roll rate (p) to track desired angular velocity.
     *          Initialized with underwater-tuned defaults from rp_defaults.
     * @note Accessed via get_rate_roll_pid()
     * @note Gains tuned for typical underwater vehicle dynamics
     */
    AC_PID                _pid_rate_roll { rp_defaults };
    
    /**
     * @brief Pitch rate PID controller
     * @details Controls body-frame pitch rate (q) to track desired angular velocity.
     *          Initialized with underwater-tuned defaults from rp_defaults.
     * @note Accessed via get_rate_pitch_pid()
     * @note Gains tuned for typical underwater vehicle dynamics
     */
    AC_PID                _pid_rate_pitch { rp_defaults };

    /**
     * @brief Yaw rate PID controller
     * @details Controls body-frame yaw rate (r) to track desired angular velocity.
     *          Uses separate defaults from roll/pitch due to different dynamics:
     *          - Different thruster configuration for yaw
     *          - Lower filter frequency (5 Hz vs 30 Hz)
     *          - Zero derivative term (D=0) typical for yaw control
     * @note Accessed via get_rate_yaw_pid()
     * @note Yaw dynamics differ from roll/pitch in underwater vehicles
     */
    AC_PID                _pid_rate_yaw {
        AC_PID::Defaults{
            .p         = AC_ATC_SUB_RATE_YAW_P,     ///< Proportional gain (0.180)
            .i         = AC_ATC_SUB_RATE_YAW_I,     ///< Integral gain (0.018)
            .d         = AC_ATC_SUB_RATE_YAW_D,     ///< Derivative gain (0.0, not used for yaw)
            .ff        = 0.0f,                       ///< Feedforward gain (not used)
            .imax      = AC_ATC_SUB_RATE_YAW_IMAX,  ///< Integral limit (0.222 rad/s)
            .filt_T_hz = AC_ATC_SUB_RATE_YAW_FILT_HZ,///< Target filter cutoff (5 Hz, lower than roll/pitch)
            .filt_E_hz = 0.0f,                       ///< Error filter cutoff (disabled)
            .filt_D_hz = AC_ATC_SUB_RATE_YAW_FILT_HZ,///< Derivative filter cutoff (5 Hz)
            .srmax     = 0,                          ///< Slew rate maximum (disabled)
            .srtau     = 1.0                         ///< Slew rate time constant
        }
    };
    /** @} */

    /**
     * @name Throttle Mixing Parameters
     * @brief User-configurable parameters for throttle vs attitude control priority
     * @details These AP_Float parameters control the balance between direct throttle
     *          (depth) control and attitude stabilization in different flight phases.
     *          Higher values prioritize attitude control over throttle commands.
     * @{
     */
    
    /**
     * @brief Manual flight throttle mixing parameter
     * @details Priority used during manual pilot control for balanced depth and
     *          attitude control. Typical value provides good compromise.
     * @note Configured via parameter system (var_info[])
     * @note Higher values favor attitude stability, lower values favor depth control
     */
    AP_Float              _thr_mix_man;
    
    /**
     * @brief Minimum throttle mixing parameter
     * @details Priority used during landing, surfacing, or when precise depth control
     *          is critical. Low value prioritizes direct throttle/depth commands
     *          over attitude corrections.
     * @note Used via set_throttle_mix_min()
     * @note Allows attitude drift to favor depth control
     */
    AP_Float              _thr_mix_min;
    
    /**
     * @brief Maximum throttle mixing parameter
     * @details Priority used during active maneuvering when attitude stability is
     *          paramount. High value maximizes attitude control authority, potentially
     *          limiting depth control responsiveness.
     * @note Used via set_throttle_mix_max()
     * @note Prioritizes stable attitude over direct throttle response
     */
    AP_Float              _thr_mix_max;
    /** @} */
};
