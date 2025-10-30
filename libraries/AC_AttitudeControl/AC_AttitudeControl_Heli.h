#pragma once

/**
 * @file AC_AttitudeControl_Heli.h
 * @brief Traditional helicopter attitude control specialization
 * 
 * @details Extends AC_AttitudeControl for traditional helicopters with main rotor and tail rotor.
 *          Implements helicopter-specific features including:
 *          - Flybar passthrough for mechanical flybar helicopters
 *          - Pirouette compensation to reduce yaw during aggressive cyclic inputs
 *          - Inverted flight support with reversed control outputs
 *          - Leaky integrators (AC_HELI_PID) to prevent wind-up during autorotation
 *          - CCPM swashplate mixing considerations
 *          - Gyroscopic precession compensation
 *          - Tail rotor coupling and torque reaction handling
 * 
 * @note This controller is fundamentally different from multicopter control due to
 *       unique helicopter dynamics: main rotor gyroscopic effects, tail rotor torque
 *       reaction, different cyclic vs collective mixing, and aerodynamic coupling.
 */

#include "AC_AttitudeControl.h"
#include <AP_Motors/AP_MotorsHeli.h>
#include <AC_PID/AC_HELI_PID.h>
#include <Filter/Filter.h>

// default rate controller PID gains
#define AC_ATC_HELI_RATE_RP_P                       0.024f
#define AC_ATC_HELI_RATE_RP_I                       0.15f
#define AC_ATC_HELI_RATE_RP_D                       0.001f
#define AC_ATC_HELI_RATE_RP_IMAX                    0.4f
#define AC_ATC_HELI_RATE_RP_FF                      0.15f
#define AC_ATC_HELI_RATE_RP_FILT_HZ                 20.0f
#define AC_ATC_HELI_RATE_YAW_P                      0.18f
#define AC_ATC_HELI_RATE_YAW_I                      0.12f
#define AC_ATC_HELI_RATE_YAW_D                      0.003f
#define AC_ATC_HELI_RATE_YAW_IMAX                   0.4f
#define AC_ATC_HELI_RATE_YAW_FF                     0.024f
#define AC_ATC_HELI_RATE_YAW_FILT_HZ                20.0f

#define AC_ATTITUDE_HELI_ANGLE_LIMIT_THROTTLE_MAX   0.95f    // Heli's use 95% of max collective before limiting frame angle
#define AC_ATTITUDE_HELI_RATE_INTEGRATOR_LEAK_RATE  0.02f
#define AC_ATTITUDE_HELI_RATE_RP_FF_FILTER          20.0f
#define AC_ATTITUDE_HELI_RATE_Y_FF_FILTER          20.0f
#define AC_ATTITUDE_HELI_HOVER_ROLL_TRIM_DEFAULT    300
#define AC_ATTITUDE_HELI_ACRO_OVERSHOOT_ANGLE_RAD   radians(30.0f)
#define AC_ATTITUDE_HELI_INVERTED_TRANSITION_TIME    3.0f

/**
 * @class AC_AttitudeControl_Heli
 * @brief Traditional helicopter attitude controller
 * 
 * @details Provides attitude control for single-rotor helicopters with swashplate and tail rotor.
 *          Handles unique helicopter characteristics:
 *          - Gyroscopic precession from main rotor requiring phase advance in cyclic inputs
 *          - Different cyclic vs collective mixing through CCPM swashplate
 *          - Tail rotor coupling where main rotor torque requires yaw compensation
 *          - Aerodynamic effects of rotor disc including translational lift and induced flow
 *          
 *          Supports both flybarless (rate gyro stabilization) and flybar-equipped (mechanical 
 *          stabilization) helicopters. In flybar mode, pilot inputs pass through directly to 
 *          the swashplate without electronic stabilization.
 *          
 *          Helicopter Dynamics Considerations:
 *          - Main rotor produces gyroscopic forces that resist attitude changes
 *          - Tail rotor thrust creates rolling moment that must be trimmed in hover
 *          - Collective pitch changes affect main rotor torque and require tail rotor compensation
 *          - Inverted flight (negative collective) reverses control responses
 *          
 *          Flybar vs Flybarless Control:
 *          - Flybarless: Full electronic stabilization using rate gyros (modern helicopters)
 *          - Flybar: Mechanical stabilization with pilot direct control (legacy helicopters)
 *          - Hybrid: Electronic yaw stabilization with flybar roll/pitch (common transition)
 *          
 *          Inverted Flight Handling:
 *          - Detects negative collective pitch as inverted flight indicator
 *          - Reverses cyclic control directions to maintain correct response
 *          - Maintains body-frame NED convention even when inverted
 *          - Requires proper PID tuning for stable inverted flight
 * 
 * @note Leaky integrator (AC_HELI_PID) prevents integrator buildup during autorotation
 *       when rotor is freewheeling and normal control authority is reduced.
 * 
 * @warning Helicopter control is fundamentally different from multicopter - do not mix
 *          parameters or tuning approaches between vehicle types.
 * 
 * @see AC_AttitudeControl Base attitude controller for all vehicle types
 * @see AC_HELI_PID Leaky integrator PID controller for helicopter rate control
 * @see AP_MotorsHeli Helicopter motor and swashplate mixer
 */
class AC_AttitudeControl_Heli : public AC_AttitudeControl {
public:
    /**
     * @brief Construct helicopter attitude controller
     * 
     * @param[in] ahrs AHRS reference for attitude estimation
     * @param[in] aparm Vehicle parameter reference for multi-copter parameters
     * @param[in] motors_heli Helicopter motor mixer for swashplate and tail rotor control
     * 
     * @details Initializes helicopter-specific attitude controller with leaky integrator PIDs,
     *          helicopter-tuned default gains, and references to helicopter motor mixer.
     *          Sets up roll trim compensation for tail rotor torque reaction and configures
     *          rate controller filter frequencies appropriate for helicopter rotor dynamics.
     * 
     * @note Constructor initializes AC_HELI_PID instances with helicopter-specific defaults
     *       including leaky integrator rate to prevent wind-up during autorotation.
     */
    AC_AttitudeControl_Heli( AP_AHRS_View &ahrs,
                        const AP_MultiCopter &aparm,
                        AP_MotorsHeli& motors);

    /**
     * @brief Get reference to roll rate PID controller
     * @return Reference to helicopter-tuned roll rate AC_HELI_PID controller
     * @note Returns AC_HELI_PID with leaky integrator for autorotation compatibility
     */
    AC_PID& get_rate_roll_pid() override { return _pid_rate_roll; }
    
    /**
     * @brief Get reference to pitch rate PID controller
     * @return Reference to helicopter-tuned pitch rate AC_HELI_PID controller
     * @note Returns AC_HELI_PID with leaky integrator for autorotation compatibility
     */
    AC_PID& get_rate_pitch_pid() override { return _pid_rate_pitch; }
    
    /**
     * @brief Get reference to yaw rate PID controller
     * @return Reference to helicopter-tuned yaw rate AC_HELI_PID controller
     * @note Returns AC_HELI_PID with leaky integrator for autorotation compatibility
     */
    AC_PID& get_rate_yaw_pid() override { return _pid_rate_yaw; }
    
    /**
     * @brief Get const reference to roll rate PID controller
     * @return Const reference to helicopter-tuned roll rate AC_HELI_PID controller
     */
    const AC_PID& get_rate_roll_pid() const override { return _pid_rate_roll; }
    
    /**
     * @brief Get const reference to pitch rate PID controller
     * @return Const reference to helicopter-tuned pitch rate AC_HELI_PID controller
     */
    const AC_PID& get_rate_pitch_pid() const override { return _pid_rate_pitch; }
    
    /**
     * @brief Get const reference to yaw rate PID controller
     * @return Const reference to helicopter-tuned yaw rate AC_HELI_PID controller
     */
    const AC_PID& get_rate_yaw_pid() const override { return _pid_rate_yaw; }

    /**
     * @brief Direct passthrough of pilot roll/pitch angles to swashplate with yaw rate control
     * 
     * @param[in] roll_passthrough_rads Pilot roll input in radians (directly to swashplate)
     * @param[in] pitch_passthrough_rads Pilot pitch input in radians (directly to swashplate)
     * @param[in] yaw_rate_bf_rads Body-frame yaw rate target in radians/second
     * 
     * @details Used in flybar modes where pilot directly controls swashplate cyclic without
     *          electronic stabilization. Roll and pitch inputs bypass the attitude controller
     *          and go straight to CCPM mixing, while yaw uses rate controller for tail rotor.
     *          This mode is appropriate for mechanically-stabilized flybar helicopters.
     * 
     * @note Flybar passthrough bypasses stabilization - pilot must stabilize the helicopter
     *       manually using the mechanical flybar. Only yaw axis uses electronic stabilization.
     * 
     * @warning Flybar mode disables roll/pitch stabilization - requires experienced pilot and
     *          properly configured flybar for stable flight.
     */
    void passthrough_bf_roll_pitch_rate_yaw_rads(float roll_passthrough_rads, float pitch_passthrough_rads, float yaw_rate_bf_rads) override;

    /**
     * @brief Set body-frame rate targets for all three axes
     * 
     * @param[in] roll_rate_bf_rads Body-frame roll rate target in radians/second
     * @param[in] pitch_rate_bf_rads Body-frame pitch rate target in radians/second
     * @param[in] yaw_rate_bf_rads Body-frame yaw rate target in radians/second
     * 
     * @details Overridden for flybarless helicopters to use rate controllers on all axes.
     *          Supports external gyro systems and full electronic stabilization without
     *          mechanical flybar. All three axes use AC_HELI_PID rate controllers.
     * 
     * @note Used for flybarless helicopters with full electronic stabilization on all axes.
     */
    void input_rate_bf_roll_pitch_yaw_rads(float roll_rate_bf_rads, float pitch_rate_bf_rads, float yaw_rate_bf_rads) override;

    /**
     * @brief Run helicopter body-frame rate controllers
     * 
     * @details Runs the lowest level body-frame rate controller and sends outputs directly
     *          to helicopter motors through the swashplate mixer. Implements helicopter-specific
     *          rate control including leaky integrators, pirouette compensation, and inverted
     *          flight handling. Calls rate_controller_run_dt() with current gyro and timestep.
     * 
     * @note Should be called at 100Hz or more for stable helicopter control. Typical rate is
     *       400Hz for best performance.
     * 
     * @warning Aggressive rate limits can cause loss of control during pirouettes or rapid
     *          cyclic inputs due to gyroscopic coupling with main rotor.
     */
    virtual void rate_controller_run() override;

    /**
     * @brief Update lean angle limit for helicopter altitude hold
     * 
     * @param[in] throttle_in Collective pitch input [0, 1] where 0.5 is hover collective
     * 
     * @details Updates maximum lean angle based on available collective pitch. Helicopters
     *          use 95% of max collective (AC_ATTITUDE_HELI_ANGLE_LIMIT_THROTTLE_MAX) before
     *          limiting frame angle, different from multicopter throttle-based limiting.
     *          Ensures sufficient collective authority remains for altitude control while
     *          allowing aggressive cyclic inputs.
     * 
     * @note Helicopter uses collective pitch, not throttle, but parameter named throttle_in
     *       for interface compatibility with base class.
     */
    void update_althold_lean_angle_max(float throttle_in) override;

    /**
     * @brief Enable/disable leaky integrator for rate controllers
     * 
     * @param[in] leaky_i true to enable leaky integrator, false for standard integrator
     * 
     * @details Controls whether AC_HELI_PID uses leaky integrator (slowly decays to zero)
     *          for body-frame to motor output stage. Leaky integrator prevents wind-up
     *          during autorotation when rotor is freewheeling and control authority is
     *          reduced. Standard integrator used during normal powered flight.
     * 
     * @note Leaky integrator with rate AC_ATTITUDE_HELI_RATE_INTEGRATOR_LEAK_RATE (0.02)
     *       prevents integrator buildup during autorotation practice or engine failure.
     */
    void use_leaky_i(bool leaky_i) override {  _flags_heli.leaky_i = leaky_i; }
    
    /**
     * @brief Enable/disable flybar passthrough modes
     * 
     * @param[in] passthrough true to pass roll/pitch directly to swashplate (flybar mode)
     * @param[in] tail_passthrough true to pass yaw directly to tail rotor
     * 
     * @details Controls whether pilot inputs pass through directly to swashplate cyclic and
     *          tail collective, bypassing electronic stabilization. Used for mechanically
     *          stabilized flybar helicopters where the flybar provides attitude damping.
     *          Allows hybrid configurations: flybar roll/pitch with electronic yaw, or
     *          full passthrough for completely mechanical stabilization.
     * 
     * @note Flybar passthrough disables electronic stabilization on enabled axes - requires
     *       properly configured mechanical flybar or experienced manual control.
     * 
     * @warning Enabling passthrough modes removes electronic stabilization. Only use with
     *          helicopters equipped with mechanical flybar or for advanced manual flight.
     */
    void use_flybar_passthrough(bool passthrough, bool tail_passthrough) override {  
        _flags_heli.flybar_passthrough = passthrough; 
        _flags_heli.tail_passthrough = tail_passthrough; 
    }

    /**
     * @brief Scale the hover roll trim parameter
     * 
     * @param[in] scalar Scaling factor [0.0, 1.0] to apply to hover roll trim
     * 
     * @details Scales the H_HOVER_ROLL_TRIM parameter for conditional application based on
     *          vehicle state. Used by vehicle code to reduce or eliminate roll trim during
     *          forward flight (where translational lift reduces tail rotor torque effect)
     *          or other conditions where trim compensation should be modified. Scalar of
     *          1.0 applies full trim, 0.0 disables trim.
     * 
     * @note Roll trim compensates for tail rotor torque reaction in hover. Trim requirements
     *       change with airspeed, so scalar allows dynamic adjustment.
     */
    void set_hover_roll_trim_scalar(float scalar) override {_hover_roll_trim_scalar = constrain_float(scalar, 0.0f, 1.0f);}

    /**
     * @brief Get hover roll trim angle
     * 
     * @return Trim angle in centidegrees to add to roll setpoint
     * 
     * @details Returns the angle to be added to roll commands to compensate for tail rotor
     *          torque reaction in hover. Main rotor torque causes the fuselage to roll
     *          (right for CCW main rotor, left for CW), and tail rotor thrust to counter
     *          this creates a rolling moment. Trim angle compensates for this effect.
     *          Returns H_HOVER_ROLL_TRIM scaled by _hover_roll_trim_scalar.
     * 
     * @note Roll trim is positive (trim right) for counter-clockwise main rotor (typical),
     *       negative (trim left) for clockwise main rotor. Typical values 100-500 centidegrees.
     * 
     * @note Roll trim affects flight controller setpoint, not direct tail rotor thrust.
     *       Compensation is applied through cyclic pitch to achieve level hover.
     */
    float get_roll_trim_cd() override;

    /**
     * @brief Set collective pitch output with helicopter-specific mixing
     * 
     * @param[in] throttle_in Collective pitch input [0, 1] where 0.5 is approximately hover
     * @param[in] apply_angle_boost true to boost collective for lean angle compensation
     * @param[in] filt_cutoff Low-pass filter cutoff frequency in Hz for throttle smoothing
     * 
     * @details Sets collective pitch output through helicopter motor mixer with angle boost
     *          compensation if enabled. Angle boost increases collective when helicopter is
     *          leaned over to maintain altitude (compensating for reduced vertical thrust
     *          component). Unlike multicopters, helicopter throttle represents collective
     *          pitch angle, not motor speed.
     * 
     * @note Parameter named throttle_in for interface compatibility, but represents collective
     *       pitch for helicopters. Value of 0.5 typically corresponds to hover collective.
     */
    void set_throttle_out(float throttle_in, bool apply_angle_boost, float filt_cutoff) override;

    /**
     * @brief Calculate angle-boosted collective pitch
     * 
     * @param[in] throttle_in Desired earth-frame vertical thrust [0, 1]
     * @return Body-frame collective pitch required to achieve earth-frame thrust
     * 
     * @details Calculates the body-frame collective pitch required to produce the desired
     *          earth-frame vertical thrust given current vehicle attitude. When helicopter
     *          is leaned over, more collective is required to maintain vertical thrust since
     *          rotor disc is tilted. Boost = throttle_in / cos(lean_angle).
     * 
     * @note For helicopters, "throttle" represents collective pitch. Angle boost compensates
     *       for reduced vertical thrust component when vehicle is leaned.
     */
    float get_throttle_boosted(float throttle_in);

    /**
     * @brief Set desired roll/pitch angles and yaw rate
     * 
     * @param[in] euler_roll_angle_rad Desired roll angle in radians (NED earth frame)
     * @param[in] euler_pitch_angle_rad Desired pitch angle in radians (NED earth frame)
     * @param[in] euler_yaw_rate_rads Desired yaw rate in radians/second (body frame)
     * 
     * @details Used when roll/pitch attitude stabilization is needed with manual or autonomous
     *          yaw rate control. Applies acceleration-limited input shaping for smooth
     *          transitions and computes body-frame angular velocity targets. Common for
     *          helicopter stabilize mode where pilot controls bank angle but yaw rate.
     * 
     * @note Angles are NED earth frame, yaw rate is body frame for intuitive pilot control.
     */
    void input_euler_angle_roll_pitch_euler_rate_yaw_rad(float euler_roll_angle_rad, float euler_pitch_angle_rad, float euler_yaw_rate_rads) override;

    /**
     * @brief Set desired roll, pitch, and yaw angles
     * 
     * @param[in] euler_roll_angle_rad Desired roll angle in radians (NED earth frame)
     * @param[in] euler_pitch_angle_rad Desired pitch angle in radians (NED earth frame)
     * @param[in] euler_yaw_angle_rad Desired yaw angle in radians (NED earth frame)
     * @param[in] slew_yaw true to apply yaw slew rate limiting for smooth heading changes
     * 
     * @details Used to follow an absolute attitude setpoint with all three axes controlled.
     *          Input shaping and yaw slew limits are applied for smooth transitions. Outputs
     *          are passed to rate controllers via shaped angular velocity targets. Common for
     *          autonomous navigation modes requiring precise heading control.
     * 
     * @note All angles are NED earth frame. Coordinate frame is still body-frame NED even
     *       when flying inverted.
     */
    void input_euler_angle_roll_pitch_yaw_rad(float euler_roll_angle_rad, float euler_pitch_angle_rad, float euler_yaw_angle_rad, bool slew_yaw) override;
    
    /**
     * @brief Set desired thrust vector and heading rate
     * 
     * @param[in] thrust_vector Desired thrust direction vector (NED earth frame, normalized)
     * @param[in] heading_rate_rads Desired heading rate in radians/second
     * @param[in] slew_yaw true to apply yaw slew rate limiting (default: true)
     * 
     * @details Used for tilt-based navigation with independent yaw control. The thrust vector
     *          defines the desired orientation (rotor disc pointing direction for vertical
     *          thrust component), while heading rate adjusts yaw independently. Input is
     *          shaped by acceleration and slew limits for smooth helicopter response.
     * 
     * @note Thrust vector should be normalized. Defines rotor disc tilt, not vehicle heading.
     */
    void input_thrust_vector_rate_heading_rads(const Vector3f& thrust_vector, float heading_rate_rads, bool slew_yaw = true) override;
    
    /**
     * @brief Set desired thrust vector, heading angle, and heading rate
     * 
     * @param[in] thrust_vector Desired thrust direction vector (NED earth frame, normalized)
     * @param[in] heading_angle_rad Desired heading angle in radians (NED earth frame)
     * @param[in] heading_rate_rads Desired heading rate in radians/second
     * 
     * @details Used for advanced attitude control where thrust direction (rotor disc tilt) is
     *          separated from yaw orientation. Allows helicopter to point thrust in one
     *          direction while nose points in another. Heading slew is constrained based on
     *          configured limits for smooth yaw transitions.
     * 
     * @note Thrust vector should be normalized. Enables independent control of rotor disc
     *       tilt and fuselage heading.
     */
    void input_thrust_vector_heading_rad(const Vector3f& thrust_vector, float heading_angle_rad, float heading_rate_rads) override;

    /**
     * @brief Enable/disable inverted flight mode
     * 
     * @param[in] inverted true for inverted (negative collective) flight
     * 
     * @details Reverses control outputs for inverted flight where helicopter flies with
     *          negative collective (rotor producing downward thrust). Control reversals
     *          maintain correct response: pulling cyclic aft should still pitch up from
     *          pilot perspective even though vehicle is inverted. Typically used for
     *          aerobatic maneuvers or sustained inverted flight.
     * 
     * @note Coordinate frame remains body-frame NED even in inverted flight. Control
     *       reversals applied to maintain intuitive pilot response.
     * 
     * @warning Inverted flight requires negative collective and proper PID tuning. Vehicle
     *          will descend if collective not sufficiently negative to counteract gravity.
     */
    void set_inverted_flight(bool inverted) override { _inverted_flight = inverted; }

    /**
     * @brief Get inverted flight mode status
     * 
     * @return true if helicopter is in inverted flight mode, false if normal flight
     * 
     * @details Returns current inverted flight flag state. Inverted flight is typically
     *          enabled when collective is negative (below mid-stick) and vehicle is
     *          rolled past 90 degrees.
     */
    bool get_inverted_flight() override { return _inverted_flight; }

    /**
     * @brief Configure notch filters for main rotor frequency
     * 
     * @param[in] sample_rate Notch filter sample rate in Hz (typically 400Hz)
     * 
     * @details Configures notch filter sample rates for AC_HELI_PID rate controllers to
     *          filter out main rotor vibration frequencies. Main rotor frequency creates
     *          significant vibration that affects IMU readings, and notch filters centered
     *          at rotor RPM reduce this interference. Sample rate must match actual rate
     *          controller update rate for proper filter operation.
     * 
     * @note Main rotor frequency varies with collective pitch and rotor RPM. Notch filters
     *       may need dynamic adjustment for RPM governors or constant-speed heads.
     */
    void set_notch_sample_rate(float sample_rate) override;

    // user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

private:

    /**
     * @brief Helicopter-specific control flags
     * 
     * @details Bit flags controlling helicopter-specific behaviors. These flags determine
     *          whether various helicopter features are enabled, including leaky integrators
     *          for autorotation, flybar passthrough for mechanical stabilization, and tail
     *          passthrough for direct yaw control.
     * 
     * @note To-Do: move these flags into the heli motors class for better encapsulation
     */
    struct AttControlHeliFlags {
        uint8_t leaky_i             :   1;  ///< 1 if using leaky integrator for autorotation compatibility
        uint8_t flybar_passthrough  :   1;  ///< 1 if passing roll/pitch directly to swashplate (flybar mode)
        uint8_t tail_passthrough    :   1;  ///< 1 if passing yaw directly to tail rotor (no stabilization)
    } _flags_heli;

    /**
     * @brief Inverted flight mode flag
     * 
     * @details True when helicopter is in inverted flight mode (negative collective, upside down).
     *          When true, control reversals are applied to maintain intuitive pilot response
     *          during inverted flight. Typically enabled when collective is below mid-stick
     *          and vehicle is rolled past 90 degrees.
     */
    bool _inverted_flight;

    /**
     * @brief Integrate body-frame rate errors to angle errors
     * 
     * @details Integrates vehicle rate tracking errors into attitude error vector 
     *          (_att_error_rot_vec_rad) for outer loop angle controller. Converts rate
     *          control errors (difference between desired and actual body rates) into
     *          angle corrections that feed back to the attitude setpoint. This integration
     *          ensures steady-state tracking and compensates for sustained rate errors.
     * 
     * @note This method updates _att_error_rot_vec_rad which represents euler axis-angle
     *       rotation from estimated attitude to reference attitude in body frame.
     */
    void integrate_bf_rate_error_to_angle_errors();

    /**
     * @brief Body-frame rate controller for roll and pitch
     * 
     * @param[in] rate_rads Current body-frame angular rates in radians/second (from gyro)
     * @param[in] rate_roll_target_rads Target roll rate in radians/second
     * @param[in] rate_pitch_target_rads Target pitch rate in radians/second
     * 
     * @details Calculates motor outputs to achieve target body-frame rates for roll and pitch
     *          using AC_HELI_PID rate controllers. Applies leaky integrator if enabled,
     *          handles inverted flight control reversals, and sends outputs directly to
     *          helicopter motor class for CCPM swashplate mixing. Implements pirouette
     *          compensation if enabled to reduce unwanted yaw during aggressive cyclic.
     * 
     * @note Outputs sent directly to AP_MotorsHeli for swashplate mixing and servo output.
     */
    void rate_bf_to_motor_roll_pitch(const Vector3f &rate_rads, float rate_roll_target_rads, float rate_pitch_target_rads);
    
    /**
     * @brief Body-frame rate controller for yaw
     * 
     * @param[in] rate_yaw_actual_rads Current yaw rate in radians/second (from gyro)
     * @param[in] rate_yaw_rads Target yaw rate in radians/second
     * @return Tail rotor collective output [-1, 1] for motor mixer
     * 
     * @details Calculates tail rotor collective to achieve target yaw rate using AC_HELI_PID
     *          yaw rate controller. Handles tail passthrough mode where pilot input goes
     *          directly to tail, and applies leaky integrator if enabled. Compensates for
     *          main rotor torque variations with collective pitch changes.
     * 
     * @note Returns tail collective output that is sent to AP_MotorsHeli for tail servo.
     */
    float rate_target_to_motor_yaw(float rate_yaw_actual_rads, float rate_yaw_rads);

    /**
     * @brief Roll passthrough value in centidegrees
     * 
     * @details Stores pilot roll input for direct passthrough to swashplate when flybar
     *          passthrough is enabled. Bypasses attitude controller and goes straight to
     *          CCPM mixing for mechanical flybar helicopter control.
     */
    float _passthrough_roll_cds;
    
    /**
     * @brief Pitch passthrough value in centidegrees
     * 
     * @details Stores pilot pitch input for direct passthrough to swashplate when flybar
     *          passthrough is enabled. Bypasses attitude controller and goes straight to
     *          CCPM mixing for mechanical flybar helicopter control.
     */
    float _passthrough_pitch_cds;

    /**
     * @brief Yaw passthrough value in centidegrees
     * 
     * @details Stores pilot yaw input for direct passthrough to tail rotor when tail
     *          passthrough is enabled. Bypasses yaw rate controller for direct manual
     *          tail rotor control.
     */
    float _passthrough_yaw_cds;

    /**
     * @brief Hover roll trim scalar
     * 
     * @details Scaling factor [0.0, 1.0] applied to H_HOVER_ROLL_TRIM parameter. Used to
     *          suppress or reduce roll trim compensation based on vehicle condition (e.g.,
     *          reduced trim during forward flight due to translational lift). Scalar of
     *          1.0 applies full trim, 0.0 disables trim completely.
     */
    float _hover_roll_trim_scalar = 0;

    /**
     * @brief Attitude error rotation vector
     * 
     * @details Represents an euler axis-angle rotation vector from the vehicle's estimated
     *          attitude to the reference (setpoint) attitude used in the attitude controller.
     *          Expressed in radians in the vehicle body frame of reference (NED convention).
     *          This error drives the outer loop angle controller to generate rate targets.
     * 
     * @note Body frame NED: X-forward, Y-right, Z-down, maintained even in inverted flight.
     */
    Vector3f            _att_error_rot_vec_rad;

    /**
     * @brief Pirouette compensation enable parameter (H_PIRO_COMP)
     * 
     * @details AP_Int8 parameter controlling pirouette compensation. When enabled, reduces
     *          unwanted yaw during aggressive cyclic inputs by compensating for gyroscopic
     *          coupling between cyclic pitch and yaw. Originally named for flybar presence
     *          detection, now controls pirouette compensation feature.
     * 
     * @note Parameter name is historical; affects attitude controller behavior in ACRO mode.
     */
    AP_Int8         _piro_comp_enabled;
    
    /**
     * @brief Hover roll trim parameter (H_HOVER_ROLL_TRIM)
     * 
     * @details AP_Int16 parameter storing angle in centidegrees to counter tail rotor thrust
     *          in hover. Compensates for rolling moment created by tail rotor when counteracting
     *          main rotor torque. Positive values trim right (typical for CCW main rotor),
     *          negative values trim left (for CW main rotor). Scaled by _hover_roll_trim_scalar.
     * 
     * @note Typical range: 100-500 centidegrees depending on tail rotor size and position.
     * @note Registered in var_info[] in .cpp file as H_* parameter group.
     */
    AP_Int16        _hover_roll_trim_cd;

    /**
     * @brief Roll and pitch rate PID default parameters
     * 
     * @details Default tuning values for helicopter roll and pitch rate controllers. These
     *          defaults are tuned for typical helicopter dynamics with approximately 400Hz
     *          update rate. Values differ from multicopter defaults due to different rotor
     *          response characteristics and gyroscopic effects.
     * 
     * @note P=0.024, I=0.15, D=0.001, FF=0.15, IMAX=0.4, filters at 20Hz
     */
    const AC_PID::Defaults rp_defaults {
        AC_PID::Defaults{
            .p         = AC_ATC_HELI_RATE_RP_P,           ///< Proportional gain for rate error
            .i         = AC_ATC_HELI_RATE_RP_I,           ///< Integral gain (with leaky option)
            .d         = AC_ATC_HELI_RATE_RP_D,           ///< Derivative gain for rate damping
            .ff        = AC_ATC_HELI_RATE_RP_FF,          ///< Feedforward gain for rate command
            .imax      = AC_ATC_HELI_RATE_RP_IMAX,        ///< Maximum integrator contribution
            .filt_T_hz = AC_ATTITUDE_HELI_RATE_RP_FF_FILTER,  ///< Feedforward filter frequency
            .filt_E_hz = AC_ATC_HELI_RATE_RP_FILT_HZ,    ///< Error filter frequency
            .filt_D_hz = 0.0,                              ///< D-term filter (disabled)
            .srmax     = 0,                                ///< Slew rate limit (disabled)
            .srtau     = 1.0                               ///< Slew rate time constant
        }
    };
    
    /**
     * @brief Roll rate PID controller
     * 
     * @details AC_HELI_PID controller for roll rate with leaky integrator support. Uses
     *          helicopter-tuned defaults appropriate for cyclic response characteristics.
     *          Leaky integrator prevents wind-up during autorotation when enabled.
     * 
     * @note Initialized with rp_defaults, registered in var_info[] as ATC_RAT_RLL_* parameters
     */
    AC_HELI_PID     _pid_rate_roll { rp_defaults };
    
    /**
     * @brief Pitch rate PID controller
     * 
     * @details AC_HELI_PID controller for pitch rate with leaky integrator support. Uses
     *          helicopter-tuned defaults appropriate for cyclic response characteristics.
     *          Leaky integrator prevents wind-up during autorotation when enabled.
     * 
     * @note Initialized with rp_defaults, registered in var_info[] as ATC_RAT_PIT_* parameters
     */
    AC_HELI_PID     _pid_rate_pitch { rp_defaults };

    /**
     * @brief Yaw rate PID controller
     * 
     * @details AC_HELI_PID controller for yaw rate (tail rotor) with leaky integrator support.
     *          Uses helicopter-tuned defaults appropriate for tail rotor response and main
     *          rotor torque coupling. Leaky integrator prevents wind-up during autorotation.
     * 
     * @note Default P=0.18, I=0.12, D=0.003, FF=0.024, IMAX=0.4, filters at 20Hz
     * @note Registered in var_info[] as ATC_RAT_YAW_* parameters
     */
    AC_HELI_PID     _pid_rate_yaw {
        AC_PID::Defaults{
            .p         = AC_ATC_HELI_RATE_YAW_P,          ///< Proportional gain for yaw rate error
            .i         = AC_ATC_HELI_RATE_YAW_I,          ///< Integral gain (with leaky option)
            .d         = AC_ATC_HELI_RATE_YAW_D,          ///< Derivative gain for yaw damping
            .ff        = AC_ATC_HELI_RATE_YAW_FF,         ///< Feedforward gain for yaw rate command
            .imax      = AC_ATC_HELI_RATE_YAW_IMAX,       ///< Maximum integrator contribution
            .filt_T_hz = AC_ATTITUDE_HELI_RATE_Y_FF_FILTER,   ///< Feedforward filter frequency
            .filt_E_hz = AC_ATC_HELI_RATE_YAW_FILT_HZ,   ///< Error filter frequency
            .filt_D_hz = 0.0,                              ///< D-term filter (disabled)
            .srmax     = 0,                                ///< Slew rate limit (disabled)
            .srtau     = 1.0                               ///< Slew rate time constant
        }
    };
    
};
