#pragma once

/**
 * @file AC_AttitudeControl.h
 * @brief ArduPilot attitude control abstract base class
 * 
 * @details Provides the core attitude and rate control interface for all vehicle types
 *          (multicopter, helicopter, submarine, tailsitter, airplane VTOL). Defines the
 *          outer-loop attitude controller (P control converting attitude errors to rate
 *          commands) and declares virtual interface for inner-loop rate controllers
 *          (vehicle-specific PID implementations). Includes input shaping, acceleration
 *          limiting, coordinate transformations, and extensive input method overloads.
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 * 
 * Source: libraries/AC_AttitudeControl/AC_AttitudeControl.cpp
 */

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_AHRS/AP_AHRS_View.h>
#include <AP_Motors/AP_Motors.h>
#include <AC_PID/AC_PID.h>
#include <AC_PID/AC_P.h>
#include <AP_Vehicle/AP_MultiCopter.h>
#include <AP_BoardConfig/AP_BoardConfig.h>

/// Default angle P gain for roll, pitch and yaw (converts angle error to rate, units: 1/s or Hz)
#define AC_ATTITUDE_CONTROL_ANGLE_P                     4.5f

/// Minimum body-frame acceleration limit for rate controller roll and pitch axis (rad/s²)
#define AC_ATTITUDE_ACCEL_RP_CONTROLLER_MIN_RADSS       radians(40.0f)
/// Maximum body-frame acceleration limit for rate controller roll and pitch axis (rad/s²)
#define AC_ATTITUDE_ACCEL_RP_CONTROLLER_MAX_RADSS       radians(720.0f)
/// Minimum body-frame acceleration limit for rate controller yaw axis (rad/s²)
#define AC_ATTITUDE_ACCEL_Y_CONTROLLER_MIN_RADSS        radians(10.0f)
/// Maximum body-frame acceleration limit for rate controller yaw axis (rad/s²)
#define AC_ATTITUDE_ACCEL_Y_CONTROLLER_MAX_RADSS        radians(120.0f)
/// Yaw slew rate limit (centideg/s) - constrains yaw angle error to prevent aggressive yaw during large angle errors
#define AC_ATTITUDE_CONTROL_SLEW_YAW_DEFAULT_CDS        6000
/// Default maximum acceleration for roll/pitch angle controller (centideg/s²)
#define AC_ATTITUDE_CONTROL_ACCEL_RP_MAX_DEFAULT_CDSS   110000.0f
/// Default maximum acceleration for yaw angle controller (centideg/s²)
#define AC_ATTITUDE_CONTROL_ACCEL_Y_MAX_DEFAULT_CDSS    27000.0f

/// Body-frame rate controller maximum output for roll-pitch axis [0, 1]
#define AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX          1.0f
/// Body-frame rate controller maximum output for yaw axis [0, 1]
#define AC_ATTITUDE_RATE_YAW_CONTROLLER_OUT_MAX         1.0f
/// Integrator relax time constant (seconds) - decays rate I term to 5% in ~0.5s during transitions
#define AC_ATTITUDE_RATE_RELAX_TC                       0.16f

/// Thrust error angle threshold (rad) - yaw authority reduced when tilt exceeds this angle
#define AC_ATTITUDE_THRUST_ERROR_ANGLE_RAD              radians(30.0f)
/// Maximum yaw error angle (rad) - yaw corrections limited during large tilt angles
#define AC_ATTITUDE_YAW_MAX_ERROR_ANGLE_RAD             radians(45.0f)

/// Rate feedforward enable flag (1=enabled, 0=disabled) - improves tracking of rate commands
#define AC_ATTITUDE_CONTROL_RATE_BF_FF_DEFAULT          1

/// Time constant for lean angle limiting (seconds) - prevents altitude loss from excessive tilt
#define AC_ATTITUDE_CONTROL_ANGLE_LIMIT_TC_DEFAULT      1.0f
/// Max throttle for angle limit calculation [0, 1] - determines available thrust margin
#define AC_ATTITUDE_CONTROL_ANGLE_LIMIT_THROTTLE_MAX    0.8f

/// Minimum throttle mix default [0, 1] - blends attitude control with throttle priority
#define AC_ATTITUDE_CONTROL_MIN_DEFAULT                 0.1f
/// Manual throttle mix default [0, 1] - throttle/attitude blend during manual flight
#define AC_ATTITUDE_CONTROL_MAN_DEFAULT                 0.1f
/// Maximum throttle mix default [0, 1] - max attitude authority during autonomous flight
#define AC_ATTITUDE_CONTROL_MAX_DEFAULT                 0.5f
/// Minimum throttle mix upper limit
#define AC_ATTITUDE_CONTROL_MIN_LIMIT                   0.5f
/// Manual throttle mix upper limit
#define AC_ATTITUDE_CONTROL_MAN_LIMIT                   4.0f
/// Maximum throttle mix upper limit
#define AC_ATTITUDE_CONTROL_MAX                         5.0f

/// Throttle/attitude priority ratio [0, 1] - higher favors attitude over pilot throttle input
#define AC_ATTITUDE_CONTROL_THR_MIX_DEFAULT             0.5f
/// Angle-P/PD throttle boost threshold - enables angle boost compensation
#define AC_ATTITUDE_CONTROL_THR_G_BOOST_THRESH          1.0f

/**
 * @class AC_AttitudeControl
 * @brief Abstract attitude controller base class for all vehicle types
 * 
 * @details This class implements a dual-loop control architecture:
 * 
 * **Outer Loop (Angle P Controller - implemented in this class):**
 * - Converts attitude quaternion errors to desired body rates
 * - Applies square root controller for improved transient response
 * - Implements input shaping with acceleration and slew rate limits
 * - Runs at main loop rate (typically 400Hz for copters, 50-400Hz for other vehicles)
 * 
 * **Inner Loop (Rate PID Controller - implemented by vehicle-specific subclasses):**
 * - Converts desired body rates to motor/servo outputs
 * - Vehicle-specific implementations: AC_AttitudeControl_Multi (multicopter),
 *   AC_AttitudeControl_Heli (helicopter), AC_AttitudeControl_Sub (underwater)
 * 
 * **Multiple Input Methods for Different Flight Modes:**
 * - Euler angles (roll/pitch/yaw) - standard stabilization modes
 * - Quaternions - singularity-free attitude representation
 * - Body-frame rates - acro/rate modes without attitude feedback
 * - Earth-frame rates - guidance modes with attitude tracking
 * - Thrust vectors - advanced control separating thrust direction from yaw
 * 
 * **Input Shaping Features:**
 * - Acceleration limits (configurable per axis)
 * - Slew rate limits (prevents aggressive yaw during large angle errors)
 * - Time constants for smoothing (configurable input filtering)
 * - Square root controller (non-linear P control for better transient response)
 * 
 * **Thread-Safety Considerations:**
 * - _ang_vel_body_rads is updated atomically at end of input functions
 * - Rate controller may run concurrently on different thread
 * - All input functions set _ang_vel_body_rads only after calculations complete
 * 
 * **Coordinate Frame Conventions:**
 * - Quaternions represent rotation from NED earth frame to body frame
 * - Angular rates are in body frame (rad/s)
 * - Positive yaw = clockwise rotation when viewed from above
 * - Roll right = positive roll, pitch up = positive pitch
 * 
 * **Singleton Pattern:**
 * - Single instance accessible via get_singleton()
 * - Enables easy access from vehicle code without passing references
 * 
 * @note Main loop rate typically 400Hz for copters, 50-400Hz for other vehicles
 * @note Quaternion representation avoids gimbal lock at pitch ±90°
 * @note Square root controller provides variable P gain: high for small errors, reduced for large
 * @note var_info[] defined in .cpp for AP_Param registration (ATC_* parameters)
 * 
 * @warning Incorrect dt causes integration errors - must match actual update rate
 * @warning Coordinate frame confusion (body vs earth, NED vs ENU) causes crashes
 * @warning Gimbal lock near pitch=±90° - use quaternion or tailsitter controller
 * @warning Rate controller pure virtual functions must be implemented by subclasses
 * 
 * @see AC_AttitudeControl_Multi - Multicopter specialization with rate PIDs
 * @see AC_AttitudeControl_Heli - Helicopter specialization
 * @see AC_AttitudeControl_Sub - Underwater vehicle specialization
 * @see AC_PosControl - Position controller commanding attitudes to this controller
 * @see AC_PID - Rate PID controller implementation
 * @see AC_P - Angle P controller implementation
 * @see AP_AHRS - Attitude and heading reference system providing current attitude
 * @see AP_Motors - Motor mixing and output
 * 
 * Source: libraries/AC_AttitudeControl/AC_AttitudeControl.cpp:500-1000
 */
class AC_AttitudeControl {
public:
    /**
     * @brief Construct attitude controller
     * 
     * @param[in] ahrs AHRS view providing current vehicle attitude and rates
     * @param[in] aparm Vehicle parameters including angle limits and configuration
     * @param[in] motors Motor mixer reference for output and thrust information
     * 
     * @details Initializes angle P controllers for roll/pitch/yaw with default gains,
     *          sets up singleton instance, and loads parameters from EEPROM.
     *          Square root controller enabled by default.
     * 
     * @note Called once during vehicle initialization
     * @note Registers parameter table via AP_Param::setup_object_defaults()
     */
    AC_AttitudeControl( AP_AHRS_View &ahrs,
                        const AP_MultiCopter &aparm,
                        AP_Motors& motors) :
        _p_angle_roll(AC_ATTITUDE_CONTROL_ANGLE_P),
        _p_angle_pitch(AC_ATTITUDE_CONTROL_ANGLE_P),
        _p_angle_yaw(AC_ATTITUDE_CONTROL_ANGLE_P),
        _angle_boost(0),
        _use_sqrt_controller(true),
        _throttle_rpy_mix_desired(AC_ATTITUDE_CONTROL_THR_MIX_DEFAULT),
        _throttle_rpy_mix(AC_ATTITUDE_CONTROL_THR_MIX_DEFAULT),
        _ahrs(ahrs),
        _aparm(aparm),
        _motors(motors)
        {
            _singleton = this;
            AP_Param::setup_object_defaults(this, var_info);
        }

    /**
     * @brief Get singleton instance
     * 
     * @return Pointer to singleton AC_AttitudeControl instance
     * 
     * @note Returns nullptr if no instance has been constructed
     */
    static AC_AttitudeControl *get_singleton(void) {
        return _singleton;
    }

    /**
     * @brief Virtual destructor
     * 
     * @details Empty destructor to ensure proper cleanup of derived classes
     */
    virtual ~AC_AttitudeControl() {}

    /**
     * @brief Set controller timestep
     * 
     * @param[in] dt_s Controller timestep in seconds
     * 
     * @details Sets the elapsed time since last controller update, used in rate
     *          integration, input shaping, and smoothing calculations.
     * 
     * @warning Must match actual update rate or integration errors will occur
     * @note Typically 0.0025s (400Hz) for copters, varies for other vehicles
     */
    void set_dt_s(float dt_s) { _dt_s = dt_s; }

    /**
     * @brief Get controller timestep
     * 
     * @return Controller timestep in seconds
     * 
     * @note Returns the value set by set_dt_s()
     */
    float get_dt_s() const { return _dt_s; }

    /**
     * @brief Get roll angle P controller
     * 
     * @return Reference to AC_P roll angle controller
     * 
     * @note Used for tuning and monitoring - converts angle error to rate command
     */
    AC_P& get_angle_roll_p() { return _p_angle_roll; }
    
    /**
     * @brief Get pitch angle P controller
     * 
     * @return Reference to AC_P pitch angle controller
     */
    AC_P& get_angle_pitch_p() { return _p_angle_pitch; }
    
    /**
     * @brief Get yaw angle P controller
     * 
     * @return Reference to AC_P yaw angle controller
     */
    AC_P& get_angle_yaw_p() { return _p_angle_yaw; }
    
    /**
     * @brief Get roll rate PID controller (pure virtual)
     * 
     * @return Reference to AC_PID roll rate controller
     * 
     * @note Must be implemented by vehicle-specific subclass
     */
    virtual AC_PID& get_rate_roll_pid() = 0;
    
    /**
     * @brief Get pitch rate PID controller (pure virtual)
     * 
     * @return Reference to AC_PID pitch rate controller
     */
    virtual AC_PID& get_rate_pitch_pid() = 0;
    
    /**
     * @brief Get yaw rate PID controller (pure virtual)
     * 
     * @return Reference to AC_PID yaw rate controller
     */
    virtual AC_PID& get_rate_yaw_pid() = 0;
    
    /**
     * @brief Get const roll rate PID controller (pure virtual)
     * 
     * @return Const reference to AC_PID roll rate controller
     */
    virtual const AC_PID& get_rate_roll_pid() const = 0;
    
    /**
     * @brief Get const pitch rate PID controller (pure virtual)
     * 
     * @return Const reference to AC_PID pitch rate controller
     */
    virtual const AC_PID& get_rate_pitch_pid() const = 0;
    
    /**
     * @brief Get const yaw rate PID controller (pure virtual)
     * 
     * @return Const reference to AC_PID yaw rate controller
     */
    virtual const AC_PID& get_rate_yaw_pid() const = 0;

    /**
     * @brief Get roll acceleration limit
     * 
     * @return Roll acceleration limit in centideg/s²
     * 
     * @details Controls responsiveness vs smoothness tradeoff for roll axis
     */
    float get_accel_roll_max_cdss() const { return _accel_roll_max_cdss; }
    
    /**
     * @brief Get roll acceleration limit
     * 
     * @return Roll acceleration limit in rad/s²
     */
    float get_accel_roll_max_radss() const { return cd_to_rad(_accel_roll_max_cdss); }

    /**
     * @brief Set roll acceleration limit
     * 
     * @param[in] accel_roll_max Roll acceleration limit in centideg/s²
     * 
     * @note Does not save to EEPROM - use save_accel_roll_max_cdss() to persist
     * @warning Excessive limits cause overshoot; insufficient limits cause sluggish response
     */
    void set_accel_roll_max_cdss(float accel_roll_max) { _accel_roll_max_cdss.set(accel_roll_max); }

    /**
     * @brief Set and save roll acceleration limit
     * 
     * @param[in] accel_roll_max Roll acceleration limit in centideg/s²
     * 
     * @note Saves to EEPROM for persistence across reboots
     */
    void save_accel_roll_max_cdss(float accel_roll_max) { _accel_roll_max_cdss.set_and_save(accel_roll_max); }

    /**
     * @brief Get pitch acceleration limit
     * 
     * @return Pitch acceleration limit in centideg/s²
     */
    float get_accel_pitch_max_cdss() const { return _accel_pitch_max_cdss; }
    
    /**
     * @brief Get pitch acceleration limit
     * 
     * @return Pitch acceleration limit in rad/s²
     */
    float get_accel_pitch_max_radss() const { return cd_to_rad(_accel_pitch_max_cdss); }

    /**
     * @brief Set pitch acceleration limit
     * 
     * @param[in] accel_pitch_max Pitch acceleration limit in centideg/s²
     */
    void set_accel_pitch_max_cdss(float accel_pitch_max) { _accel_pitch_max_cdss.set(accel_pitch_max); }

    /**
     * @brief Set and save pitch acceleration limit
     * 
     * @param[in] accel_pitch_max Pitch acceleration limit in centideg/s²
     */
    void save_accel_pitch_max_cdss(float accel_pitch_max) { _accel_pitch_max_cdss.set_and_save(accel_pitch_max); }

    /**
     * @brief Get yaw acceleration limit
     * 
     * @return Yaw acceleration limit in centideg/s²
     */
    float get_accel_yaw_max_cdss() const { return _accel_yaw_max_cdss; }
    
    /**
     * @brief Get yaw acceleration limit
     * 
     * @return Yaw acceleration limit in rad/s²
     */
    float get_accel_yaw_max_radss() const { return cd_to_rad(_accel_yaw_max_cdss); }

    /**
     * @brief Set yaw acceleration limit
     * 
     * @param[in] accel_yaw_max Yaw acceleration limit in centideg/s²
     */
    void set_accel_yaw_max_cdss(float accel_yaw_max) { _accel_yaw_max_cdss.set(accel_yaw_max); }

    /**
     * @brief Set and save yaw acceleration limit
     * 
     * @param[in] accel_yaw_max Yaw acceleration limit in centideg/s²
     */
    void save_accel_yaw_max_cdss(float accel_yaw_max) { _accel_yaw_max_cdss.set_and_save(accel_yaw_max); }

    /**
     * @brief Get roll angular velocity limit
     * 
     * @return Maximum roll rate in rad/s
     * 
     * @details Maximum rotation rate for roll axis - limits commanded rates
     */
    float get_ang_vel_roll_max_rads() const { return radians(_ang_vel_roll_max_degs); }
    
    /**
     * @brief Get roll angular velocity limit
     * 
     * @return Maximum roll rate in deg/s
     */
    float get_ang_vel_roll_max_degs() const { return _ang_vel_roll_max_degs; }

    /**
     * @brief Set roll angular velocity limit
     * 
     * @param[in] vel_roll_max Maximum roll rate in deg/s
     */
    void set_ang_vel_roll_max_degs(float vel_roll_max) { _ang_vel_roll_max_degs.set(vel_roll_max); }

    /**
     * @brief Get pitch angular velocity limit
     * 
     * @return Maximum pitch rate in rad/s
     */
    float get_ang_vel_pitch_max_rads() const { return radians(_ang_vel_pitch_max_degs); }
    
    /**
     * @brief Get pitch angular velocity limit
     * 
     * @return Maximum pitch rate in deg/s
     */
    float get_ang_vel_pitch_max_degs() const { return _ang_vel_pitch_max_degs; }

    /**
     * @brief Set pitch angular velocity limit
     * 
     * @param[in] vel_pitch_max Maximum pitch rate in deg/s
     */
    void set_ang_vel_pitch_max_degs(float vel_pitch_max) { _ang_vel_pitch_max_degs.set(vel_pitch_max); }

    /**
     * @brief Get yaw angular velocity limit
     * 
     * @return Maximum yaw rate in rad/s
     */
    float get_ang_vel_yaw_max_rads() const { return radians(_ang_vel_yaw_max_degs); }
    
    /**
     * @brief Get yaw angular velocity limit
     * 
     * @return Maximum yaw rate in deg/s
     */
    float get_ang_vel_yaw_max_degs() const { return _ang_vel_yaw_max_degs; }

    /**
     * @brief Set yaw angular velocity limit
     * 
     * @param[in] vel_yaw_max Maximum yaw rate in deg/s
     */
    void set_ang_vel_yaw_max_degs(float vel_yaw_max) { _ang_vel_yaw_max_degs.set(vel_yaw_max); }

    /**
     * @brief Get yaw slew rate limit
     * 
     * @return Yaw slew rate limit in deg/s
     * 
     * @details Limits yaw angle error to prevent aggressive yaw during large attitude errors
     */
    float get_slew_yaw_max_degs() const;

    /**
     * @brief Get yaw slew rate limit
     * 
     * @return Yaw slew rate limit in rad/s
     */
    float get_slew_yaw_max_rads() const;

    /**
     * @brief Get rate input smoothing time constant
     * 
     * @return Time constant in seconds [0, 1]
     * 
     * @details Smooths rate commands to reduce abrupt changes - higher values = more smoothing
     */
    float get_input_tc() const { return _input_tc; }

    /**
     * @brief Set rate input smoothing time constant
     * 
     * @param[in] input_tc Time constant in seconds [0, 1]
     * 
     * @note Automatically constrained to [0, 1] range
     */
    void set_input_tc(float input_tc) { _input_tc.set(constrain_float(input_tc, 0.0f, 1.0f)); }

    /**
     * @brief Zero attitude errors for rate mode entry
     * 
     * @details Sets attitude target to current attitude, resets rate errors to zero.
     *          Prevents control transients when switching from angle to rate control modes.
     * 
     * @note Called when entering acro/rate modes or switching from stabilized to rate control
     */
    void relax_attitude_controllers();

    /**
     * @brief Zero attitude errors with optional pitch exclusion
     * 
     * @param[in] exclude_pitch If true, don't relax pitch controller (tailsitter specific)
     * 
     * @details Virtual function overridden by AC_AttitudeControl_TS for tailsitter quadplanes
     *          Base implementation ignores parameter and calls relax_attitude_controllers()
     */
    virtual void relax_attitude_controllers(bool exclude_pitch) { relax_attitude_controllers(); }

    /**
     * @brief Reset rate PID integrators to zero
     * 
     * @details Immediately zeros I-terms in roll/pitch/yaw rate controllers.
     *          Used on mode changes or when re-enabling attitude control.
     * 
     * @warning Abrupt integrator reset can cause control discontinuity
     * @note For smooth transitions, use reset_rate_controller_I_terms_smoothly()
     */
    void reset_rate_controller_I_terms();

    /**
     * @brief Decay integrators to zero over ~0.5s
     * 
     * @details Smoothly decays I-terms using exponential decay with time constant
     *          AC_ATTITUDE_RATE_RELAX_TC. Prevents overshoot during mode transitions.
     * 
     * @note Preferred over reset_rate_controller_I_terms() for smooth transitions
     */
    void reset_rate_controller_I_terms_smoothly();

    /**
     * @brief Reduce gains while landed
     * 
     * @param[in] landed true if vehicle is on ground, false if airborne
     * 
     * @details Applies gain reduction factor to prevent ground resonance oscillations
     *          when motors are armed but vehicle is on ground. Scaling factors
     *          configured via ATC_LAND_* parameters.
     * 
     * @note Critical for preventing ground resonance in multirotors
     */
    void landed_gain_reduction(bool landed);

    /**
     * @brief Set targets to current vehicle state
     * 
     * @param[in] reset_rate If true, also reset rate targets to zero (default: true)
     * 
     * @details Sets attitude target to current attitude from AHRS. Optionally resets
     *          rate targets. Prevents control jumps when entering attitude hold modes.
     * 
     * @note Called on mode entry or when re-enabling attitude control
     */
    void reset_target_and_rate(bool reset_rate = true);

    /**
     * @brief Set yaw target to current heading
     * 
     * @param[in] reset_rate If true, also reset yaw rate target to zero (default: true)
     * 
     * @details Sets yaw angle target to current heading while preserving roll/pitch targets.
     *          Used when switching from yaw rate to yaw angle control.
     */
    void reset_yaw_target_and_rate(bool reset_rate = true);

    /**
     * @brief Handle EKF attitude reset
     * 
     * @details Updates attitude targets to compensate for discontinuous EKF attitude
     *          correction. Called when EKF performs an attitude reset due to sensor
     *          innovation or GPS/vision update.
     * 
     * @note Prevents control transient from EKF attitude discontinuity
     */
    void inertial_frame_reset();

    /**
     * @brief Set body-frame roll, euler pitch, and yaw rate (centidegrees)
     * 
     * @param[in] plane_controls If true, swap roll/yaw effects near pitch=90° (tailsitter mode)
     * @param[in] euler_roll_angle_cd Desired body-frame roll angle in centidegrees
     * @param[in] euler_pitch_angle_cd Desired euler pitch angle in centidegrees
     * @param[in] euler_yaw_rate_cds Desired yaw rate in centideg/s
     * 
     * @details Tailsitter-specific input method. Base implementation is no-op.
     *          See input_euler_rate_yaw_euler_angle_pitch_bf_roll_rad() for details.
     * 
     * @note Only implemented in AC_AttitudeControl_TS subclass
     */
    virtual void input_euler_rate_yaw_euler_angle_pitch_bf_roll_cd(bool plane_controls, float euler_roll_angle_cd, 
        float euler_pitch_angle_cd, float euler_yaw_rate_cds) {}

    /**
     * @brief Set body-frame roll, euler pitch, and yaw rate (radians)
     * 
     * @param[in] plane_controls If true, swap roll/yaw effects near pitch=90° (tailsitter mode)
     * @param[in] euler_roll_angle_rad Desired body-frame roll angle in radians
     * @param[in] euler_pitch_angle_rad Desired euler pitch angle in radians
     * @param[in] euler_yaw_rate_rads Desired yaw rate in rad/s
     * 
     * @details Used by tailsitter quadplanes to handle attitude control during transition
     *          between hover and forward flight. When plane_controls is true and pitch
     *          approaches ±90°, control axes are remapped to maintain intuitive control.
     * 
     * @note Only implemented in AC_AttitudeControl_TS subclass
     * @note Base implementation is no-op for non-tailsitter vehicles
     */
    virtual void input_euler_rate_yaw_euler_angle_pitch_bf_roll_rad(bool plane_controls, float euler_roll_angle_rad, 
        float euler_pitch_angle_rad, float euler_yaw_rate_rads) {}

    /**
     * ===== RATE UPDATE FUNCTIONS =====
     * 
     * @details All functions in this section update _ang_vel_body_rads, which is used as the
     *          rate target by the rate controller. Thread-safety is critical:
     * 
     * **Thread-Safety Pattern:**
     * - _ang_vel_body_rads can be read by rate controller running on different thread
     * - All calculations performed on local variables first
     * - _ang_vel_body_rads updated atomically at end of function
     * - Prevents rate controller from seeing intermediate/inconsistent values
     * - Locking avoided due to performance cost at 400Hz update rate
     * 
     * **Input Method Categories:**
     * - input_quaternion() - Direct quaternion + rates (singularity-free)
     * - input_euler_angle_roll_pitch_euler_rate_yaw_*() - Roll/pitch angles + yaw rate
     * - input_euler_angle_roll_pitch_yaw_*() - Full euler attitude (3 angles)
     * - input_euler_rate_*() - Earth-frame euler rate commands
     * - input_rate_bf_*() - Body-frame rate commands (acro modes)
     * - input_thrust_vector_*() - Thrust vector + heading (advanced control)
     * - input_angle_step_bf_*() - Step inputs (autotuning/testing)
     * - input_rate_step_bf_*() - Rate step inputs (system identification)
     * 
     * @warning Any new functions manipulating _ang_vel_body_rads must follow atomic update pattern
     * @note All input functions apply acceleration limiting and input shaping
     */

    /**
     * @brief Calculate body-frame angular velocities from attitude error
     * 
     * @details Core attitude controller function that implements the outer-loop P controller.
     *          Converts quaternion attitude error to desired body-frame angular velocity using
     *          angle P gains, square root controller (if enabled), and acceleration limits.
     * 
     * @note Called by most input_*() functions after setting attitude target
     * @note Implements thrust-heading decomposition for proper yaw limiting during tilt
     * 
     * Source: libraries/AC_AttitudeControl/AC_AttitudeControl.cpp:500-650
     */
    void attitude_controller_run_quat();

    /**
     * @brief Set desired attitude quaternion and body-frame angular velocity
     * 
     * @param[in] attitude_desired_quat Desired attitude quaternion (NED frame to body frame)
     * @param[in] ang_vel_body_rads Desired body-frame angular velocity (rad/s)
     * 
     * @details Most direct attitude input method - avoids gimbal lock and euler singularities.
     *          The desired quaternion is incrementally updated each timestep based on ang_vel_body_rads.
     *          Angular velocity input is shaped by acceleration limits and feedforward before being
     *          passed to rate controller.
     * 
     * @note Preferred method for guidance systems providing quaternion setpoints
     * @note No gimbal lock at pitch = ±90°
     * @note Angular velocity is in body frame, not earth frame
     * 
     * Source: libraries/AC_AttitudeControl/AC_AttitudeControl.cpp:750-850
     */
    virtual void input_quaternion(Quaternion& attitude_desired_quat, Vector3f ang_vel_body_rads);

    /**
     * @brief Set roll/pitch angles and yaw rate (centidegrees)
     * 
     * @param[in] euler_roll_angle_cd Desired roll angle in centidegrees
     * @param[in] euler_pitch_angle_cd Desired pitch angle in centidegrees
     * @param[in] euler_yaw_rate_cds Desired yaw rate in centideg/s
     * 
     * @details Centidegree wrapper for input_euler_angle_roll_pitch_euler_rate_yaw_rad().
     *          See that function for full details.
     */
    void input_euler_angle_roll_pitch_euler_rate_yaw_cd(float euler_roll_angle_cd, float euler_pitch_angle_cd, float euler_yaw_rate_cds);

    /**
     * @brief Set roll/pitch angles and yaw rate (radians)
     * 
     * @param[in] euler_roll_angle_rad Desired roll angle in radians
     * @param[in] euler_pitch_angle_rad Desired pitch angle in radians
     * @param[in] euler_yaw_rate_rads Desired yaw rate in rad/s
     * 
     * @details Common input method when roll/pitch stabilization is needed with manual or
     *          autonomous yaw rate control. Used in:
     *          - Loiter mode with pilot yaw stick input
     *          - Auto mode following curved paths (constant yaw rate around waypoints)
     *          - Any mode requiring independent yaw rate control
     * 
     *          Applies acceleration-limited input shaping for smooth transitions.
     *          Roll/pitch attitude targets are held while yaw continuously rotates.
     * 
     * @note Yaw target is integrated from yaw rate - no fixed yaw angle maintained
     * @note Roll/pitch angles are stabilized to setpoint
     * 
     * Source: libraries/AC_AttitudeControl/AC_AttitudeControl.cpp:850-950
     */
    virtual void input_euler_angle_roll_pitch_euler_rate_yaw_rad(float euler_roll_angle_rad, float euler_pitch_angle_rad, float euler_yaw_rate_rads);

    /**
     * @brief Set roll/pitch/yaw angles (centidegrees)
     * 
     * @param[in] euler_roll_angle_cd Desired roll angle in centidegrees
     * @param[in] euler_pitch_angle_cd Desired pitch angle in centidegrees
     * @param[in] euler_yaw_angle_cd Desired yaw angle in centidegrees
     * @param[in] slew_yaw If true, apply yaw slew rate limiting
     * 
     * @details Centidegree wrapper for input_euler_angle_roll_pitch_yaw_rad().
     *          See that function for full details.
     */
    void input_euler_angle_roll_pitch_yaw_cd(float euler_roll_angle_cd, float euler_pitch_angle_cd, float euler_yaw_angle_cd, bool slew_yaw);

    /**
     * @brief Set roll/pitch/yaw angles (radians)
     * 
     * @param[in] euler_roll_angle_rad Desired roll angle in radians
     * @param[in] euler_pitch_angle_rad Desired pitch angle in radians
     * @param[in] euler_yaw_angle_rad Desired yaw angle in radians
     * @param[in] slew_yaw If true, apply yaw slew rate limiting
     * 
     * @details Used to follow an absolute attitude setpoint with all three axes stabilized.
     *          Common in:
     *          - Stabilize mode with centered sticks (level flight)
     *          - Auto mode waypoint navigation (fixed heading)
     *          - Guided mode with attitude commands from GCS
     * 
     *          Input shaping and acceleration limits applied to all axes.
     *          Yaw slew limiting prevents aggressive yaw changes during large attitude errors
     *          (e.g., prevents rapid yaw when vehicle is tilted beyond thrust error threshold).
     * 
     * @note Yaw slew limiting recommended during auto flight to prevent oscillations
     * @note All three axes use angle P controller to generate rate commands
     * 
     * Source: libraries/AC_AttitudeControl/AC_AttitudeControl.cpp:950-1050
     */
    virtual void input_euler_angle_roll_pitch_yaw_rad(float euler_roll_angle_rad, float euler_pitch_angle_rad, float euler_yaw_angle_rad, bool slew_yaw);

    /**
     * @brief Set earth-frame euler rates (centidegrees/s)
     * 
     * @param[in] euler_roll_rate_cds Desired earth-frame roll rate in centideg/s
     * @param[in] euler_pitch_rate_cds Desired earth-frame pitch rate in centideg/s
     * @param[in] euler_yaw_rate_cds Desired earth-frame yaw rate in centideg/s
     * 
     * @details Centidegree wrapper for input_euler_rate_roll_pitch_yaw_rads().
     *          See that function for full details.
     */
    void input_euler_rate_roll_pitch_yaw_cds(float euler_roll_rate_cds, float euler_pitch_rate_cds, float euler_yaw_rate_cds);

    /**
     * @brief Set earth-frame euler rates (radians/s)
     * 
     * @param[in] euler_roll_rate_rads Desired earth-frame roll rate in rad/s
     * @param[in] euler_pitch_rate_rads Desired earth-frame pitch rate in rad/s
     * @param[in] euler_yaw_rate_rads Desired earth-frame yaw rate in rad/s
     * 
     * @details Applies angular rate targets in earth frame (NED frame) rather than body frame.
     *          Earth-frame rates maintain constant rates relative to ground, useful for:
     *          - Circle mode (constant turn rate relative to ground)
     *          - Orbiting waypoints
     *          - Coordinated turns in guidance modes
     * 
     *          The inputs are shaped using acceleration limits and time constants, then
     *          converted to body-frame angular velocities and passed to rate controller.
     * 
     * @note Conversion from earth to body frame depends on current attitude
     * @note Different from body-frame rates which are relative to vehicle orientation
     * 
     * Source: libraries/AC_AttitudeControl/AC_AttitudeControl.cpp:1050-1150
     */
    virtual void input_euler_rate_roll_pitch_yaw_rads(float euler_roll_rate_rads, float euler_pitch_rate_rads, float euler_yaw_rate_rads);

    /**
     * @brief Set body-frame angular rates (centidegrees/s)
     * 
     * @param[in] roll_rate_bf_cds Desired body-frame roll rate in centideg/s
     * @param[in] pitch_rate_bf_cds Desired body-frame pitch rate in centideg/s
     * @param[in] yaw_rate_bf_cds Desired body-frame yaw rate in centideg/s
     * 
     * @details Centidegree wrapper for input_rate_bf_roll_pitch_yaw_rads().
     *          See that function for full details.
     */
    void input_rate_bf_roll_pitch_yaw_cds(float roll_rate_bf_cds, float pitch_rate_bf_cds, float yaw_rate_bf_cds);

    /**
     * @brief Set body-frame angular rates with attitude stabilization (radians/s)
     * 
     * @param[in] roll_rate_bf_rads Desired body-frame roll rate in rad/s
     * @param[in] pitch_rate_bf_rads Desired body-frame pitch rate in rad/s
     * @param[in] yaw_rate_bf_rads Desired body-frame yaw rate in rad/s
     * 
     * @details Used by stabilized acro modes where pilot commands body-frame rates but
     *          attitude errors are corrected. This variant includes:
     *          - Attitude error integration (maintains attitude when stick centered)
     *          - Acceleration-limited input shaping
     *          - Rate limiting based on configured maximums
     * 
     *          Common in Copter ACRO mode with ACRO_TRAINER enabled.
     * 
     * @note Attitude feedback prevents drift - vehicle returns to level when sticks centered
     * @note Shaped by acceleration limits for smooth response
     * 
     * Source: libraries/AC_AttitudeControl/AC_AttitudeControl.cpp:1150-1250
     */
    virtual void input_rate_bf_roll_pitch_yaw_rads(float roll_rate_bf_rads, float pitch_rate_bf_rads, float yaw_rate_bf_rads);

    /**
     * @brief Set body-frame angular rates, rate-only mode (centidegrees/s)
     * 
     * @param[in] roll_rate_bf_cds Desired body-frame roll rate in centideg/s
     * @param[in] pitch_rate_bf_cds Desired body-frame pitch rate in centideg/s
     * @param[in] yaw_rate_bf_cds Desired body-frame yaw rate in centideg/s
     * 
     * @details Centidegree wrapper for input_rate_bf_roll_pitch_yaw_2_rads().
     *          See that function for full details.
     */
    void input_rate_bf_roll_pitch_yaw_2_cds(float roll_rate_bf_cds, float pitch_rate_bf_cds, float yaw_rate_bf_cds);

    /**
     * @brief Set body-frame angular rates, pure rate mode (radians/s)
     * 
     * @param[in] roll_rate_bf_rads Desired body-frame roll rate in rad/s
     * @param[in] pitch_rate_bf_rads Desired body-frame pitch rate in rad/s
     * @param[in] yaw_rate_bf_rads Desired body-frame yaw rate in rad/s
     * 
     * @details Used by Copter's pure rate acro mode (ACRO_TRAINER=0). No attitude feedback
     *          or stabilization - pilot directly commands angular rates. Smoothing applied
     *          but no attitude integration. Vehicle will drift when sticks are centered.
     * 
     *          This is the most direct control mode, preferred by experienced acro pilots.
     * 
     * @note No attitude stabilization - vehicle orientation drifts freely
     * @note Rate smoothing still applied via input time constant
     * @note Requires active pilot input to maintain orientation
     * 
     * Source: libraries/AC_AttitudeControl/AC_AttitudeControl.cpp:1250-1350
     */
    virtual void input_rate_bf_roll_pitch_yaw_2_rads(float roll_rate_bf_rads, float pitch_rate_bf_rads, float yaw_rate_bf_rads);

    /**
     * @brief Set body-frame angular rates, Plane acro variant (centidegrees/s)
     * 
     * @param[in] roll_rate_bf_cds Desired body-frame roll rate in centideg/s
     * @param[in] pitch_rate_bf_cds Desired body-frame pitch rate in centideg/s
     * @param[in] yaw_rate_bf_cds Desired body-frame yaw rate in centideg/s
     * 
     * @details Centidegree wrapper for input_rate_bf_roll_pitch_yaw_3_rads().
     *          See that function for full details.
     */
    void input_rate_bf_roll_pitch_yaw_3_cds(float roll_rate_bf_cds, float pitch_rate_bf_cds, float yaw_rate_bf_cds);

    /**
     * @brief Set body-frame angular rates with integrated error (radians/s)
     * 
     * @param[in] roll_rate_bf_rads Desired body-frame roll rate in rad/s
     * @param[in] pitch_rate_bf_rads Desired body-frame pitch rate in rad/s
     * @param[in] yaw_rate_bf_rads Desired body-frame yaw rate in rad/s
     * 
     * @details Used by Plane's acro mode with integrated rate error correction.
     *          Integrates attitude error over time to generate target angular rates,
     *          providing some attitude stabilization while maintaining direct rate feel.
     * 
     *          Hybrid between pure rate and attitude-stabilized control.
     * 
     * @note Plane-specific acro variant
     * @note Provides better attitude tracking than pure rate mode
     * 
     * Source: libraries/AC_AttitudeControl/AC_AttitudeControl.cpp:1350-1450
     */
    virtual void input_rate_bf_roll_pitch_yaw_3_rads(float roll_rate_bf_rads, float pitch_rate_bf_rads, float yaw_rate_bf_rads);

    /**
     * @brief Set body-frame angular rates without smoothing (centidegrees/s)
     * 
     * @param[in] roll_rate_bf_cds Desired body-frame roll rate in centideg/s
     * @param[in] pitch_rate_bf_cds Desired body-frame pitch rate in centideg/s
     * @param[in] yaw_rate_bf_cds Desired body-frame yaw rate in centideg/s
     * 
     * @details Centidegree wrapper for input_rate_bf_roll_pitch_yaw_no_shaping_rads().
     *          See that function for full details.
     */
    void input_rate_bf_roll_pitch_yaw_no_shaping_cds(float roll_rate_bf_cds, float pitch_rate_bf_cds, float yaw_rate_bf_cds);

    /**
     * @brief Directly set body-frame angular rates without smoothing (radians/s)
     * 
     * @param[in] roll_rate_bf_rads Desired body-frame roll rate in rad/s
     * @param[in] pitch_rate_bf_rads Desired body-frame pitch rate in rad/s
     * @param[in] yaw_rate_bf_rads Desired body-frame yaw rate in rad/s
     * 
     * @details Used when external control logic (e.g., fixed-wing controller during
     *          quadplane VTOL transition) dictates rates and has already applied its own
     *          smoothing. No additional smoothing or input shaping applied by attitude
     *          controller - rates passed directly to rate controller.
     * 
     * @warning No input shaping - ensure external controller applies proper limits
     * @note Used during VTOL transitions in quadplanes
     * 
     * Source: libraries/AC_AttitudeControl/AC_AttitudeControl.cpp:1450-1500
     */
    void input_rate_bf_roll_pitch_yaw_no_shaping_rads(float roll_rate_bf_rads, float pitch_rate_bf_rads, float yaw_rate_bf_rads);

    /**
     * @brief Apply angle step input for testing (centidegrees)
     * 
     * @param[in] roll_angle_step_bf_cd Roll angle step in centidegrees
     * @param[in] pitch_angle_step_bf_cd Pitch angle step in centidegrees
     * @param[in] yaw_angle_step_bf_cd Yaw angle step in centidegrees
     * 
     * @details Centidegree wrapper for input_angle_step_bf_roll_pitch_yaw_rad().
     *          See that function for full details.
     */
    void input_angle_step_bf_roll_pitch_yaw_cd(float roll_angle_step_bf_cd, float pitch_angle_step_bf_cd, float yaw_angle_step_bf_cd);

    /**
     * @brief Apply one-time angular offset for step response testing (radians)
     * 
     * @param[in] roll_angle_step_bf_rad Roll angle step offset in radians
     * @param[in] pitch_angle_step_bf_rad Pitch angle step offset in radians
     * @param[in] yaw_angle_step_bf_rad Yaw angle step offset in radians
     * 
     * @details Used by AutoTune and system identification to generate step responses.
     *          Applies a one-time angular offset to the current attitude target, creating
     *          a step disturbance for measuring system response characteristics.
     * 
     *          The attitude controller then generates rates to track this new target,
     *          and the response is analyzed to determine optimal PID gains.
     * 
     * @note Used exclusively for AutoTune/system identification
     * @note Step magnitude calculated by max_angle_step_bf_*() functions
     * @warning Not for normal flight operations
     * 
     * Source: libraries/AC_AttitudeControl/AC_AttitudeControl.cpp:1500-1550
     */
    virtual void input_angle_step_bf_roll_pitch_yaw_rad(float roll_angle_step_bf_rad, float pitch_angle_step_bf_rad, float yaw_angle_step_bf_rad);

    /**
     * @brief Apply rate step input for testing (centidegrees/s)
     * 
     * @param[in] roll_rate_step_bf_cds Roll rate step in centideg/s
     * @param[in] pitch_rate_step_bf_cds Pitch rate step in centideg/s
     * @param[in] yaw_rate_step_bf_cds Yaw rate step in centideg/s
     * 
     * @details Centidegree wrapper for input_rate_step_bf_roll_pitch_yaw_rads().
     *          See that function for full details.
     */
    void input_rate_step_bf_roll_pitch_yaw_cds(float roll_rate_step_bf_cds, float pitch_rate_step_bf_cds, float yaw_rate_step_bf_cds);
    
    /**
     * @brief Apply one-time angular velocity offset for rate step testing (radians/s)
     * 
     * @param[in] roll_rate_step_bf_rads Roll rate step offset in rad/s
     * @param[in] pitch_rate_step_bf_rads Pitch rate step offset in rad/s
     * @param[in] yaw_rate_step_bf_rads Yaw rate step offset in rad/s
     * 
     * @details Used to apply discrete disturbances or step inputs for system identification.
     *          Similar to angle step but directly applies rate offset, allowing measurement
     *          of rate controller response independently of attitude controller.
     * 
     *          Step magnitude calculated by max_rate_step_bf_*() functions to ensure
     *          rate PID output reaches saturation within 4 timesteps for proper tuning.
     * 
     * @note Used for rate controller tuning and identification
     * @warning Not for normal flight operations
     * 
     * Source: libraries/AC_AttitudeControl/AC_AttitudeControl.cpp:1550-1600
     */
    virtual void input_rate_step_bf_roll_pitch_yaw_rads(float roll_rate_step_bf_rads, float pitch_rate_step_bf_rads, float yaw_rate_step_bf_rads);

    /**
     * @brief Set thrust vector and heading rate (centidegrees/s)
     * 
     * @param[in] thrust_vector Desired body-frame thrust direction vector (normalized)
     * @param[in] heading_rate_cds Desired heading rate in centideg/s
     * @param[in] slew_yaw Apply yaw slew rate limiting (default: true)
     * 
     * @details Centidegree wrapper for input_thrust_vector_rate_heading_rads().
     *          See that function for full details.
     */
    void input_thrust_vector_rate_heading_cds(const Vector3f& thrust_vector, float heading_rate_cds, bool slew_yaw = true);

    /**
     * @brief Set thrust vector and heading rate (radians/s)
     * 
     * @param[in] thrust_vector Desired body-frame thrust direction vector (normalized)
     * @param[in] heading_rate_rads Desired heading rate in rad/s
     * @param[in] slew_yaw Apply yaw slew rate limiting (default: true)
     * 
     * @details Used for tilt-based navigation with independent yaw control. The thrust
     *          vector defines the desired vehicle tilt/orientation (pointing direction
     *          for vertical thrust motor), while heading rate adjusts yaw independently.
     * 
     *          Common in:
     *          - Guided mode navigation (tilt toward target)
     *          - Position controller output (tilt for acceleration)
     *          - Advanced flight modes separating tilt from heading
     * 
     *          Input shaped by acceleration limits and slew rate constraints.
     * 
     * @note Thrust vector should be normalized or will be normalized internally
     * @note Yaw and tilt controlled independently
     * @note Used by position controller (AC_PosControl)
     * 
     * Source: libraries/AC_AttitudeControl/AC_AttitudeControl.cpp:1600-1700
     */
    virtual void input_thrust_vector_rate_heading_rads(const Vector3f& thrust_vector, float heading_rate_rads, bool slew_yaw = true);

    /**
     * @brief Set thrust vector and heading angle (centidegrees)
     * 
     * @param[in] thrust_vector Desired body-frame thrust direction vector (normalized)
     * @param[in] heading_angle_cd Desired heading angle in centidegrees
     * @param[in] heading_rate_cds Feedforward heading rate in centideg/s
     * 
     * @details Centidegree wrapper for input_thrust_vector_heading_rad().
     *          See that function for full details.
     */
    void input_thrust_vector_heading_cd(const Vector3f& thrust_vector, float heading_angle_cd, float heading_rate_cds);

    /**
     * @brief Set thrust vector and heading angle with feedforward rate (radians)
     * 
     * @param[in] thrust_vector Desired body-frame thrust direction vector (normalized)
     * @param[in] heading_angle_rad Desired heading angle in radians
     * @param[in] heading_rate_rads Feedforward heading rate in rad/s
     * 
     * @details Used for advanced attitude control where thrust direction is separated
     *          from yaw orientation. Combines heading angle target with feedforward
     *          heading rate for improved tracking. Heading slew constrained by limits.
     * 
     *          Converts heading angle and rate into combined yaw control, applies
     *          thrust vector for tilt, then shapes output via acceleration limits.
     * 
     * @note Feedforward rate improves yaw tracking during curved paths
     * @note Heading slew limiting applied based on ATC_SLEW_YAW parameter
     * 
     * Source: libraries/AC_AttitudeControl/AC_AttitudeControl.cpp:1700-1800
     */
    virtual void input_thrust_vector_heading_rad(const Vector3f& thrust_vector, float heading_angle_rad, float heading_rate_rads);

    /**
     * @brief Set thrust vector and heading angle, zero rate (radians)
     * 
     * @param[in] thrust_vector Desired body-frame thrust direction vector (normalized)
     * @param[in] heading_rad Desired heading angle in radians
     * 
     * @details Convenience wrapper for input_thrust_vector_heading_rad() with zero
     *          heading rate. See that function for full details.
     */
    void input_thrust_vector_heading_rad(const Vector3f& thrust_vector, float heading_rad) {input_thrust_vector_heading_rad(thrust_vector, heading_rad, 0.0f);}

    ////// end rate update functions //////

    /**
     * @brief Convert thrust vector and heading to attitude quaternion
     * 
     * @param[in] thrust_vector Body-frame thrust direction vector
     * @param[in] heading_angle_rad Desired heading angle in radians
     * 
     * @return Quaternion representing the attitude in NED frame
     * 
     * @details Converts a thrust vector (defining vehicle tilt) and heading angle
     *          (defining yaw) into a complete attitude quaternion. Used internally
     *          to convert thrust-vector commands into attitude targets.
     * 
     * @note Thrust vector defines roll/pitch, heading defines yaw
     * 
     * Source: libraries/AC_AttitudeControl/AC_AttitudeControl.cpp:1800-1850
     */
    Quaternion attitude_from_thrust_vector(Vector3f thrust_vector, float heading_angle_rad) const;

    /**
     * @brief Run rate controller and send motor outputs (pure virtual)
     * 
     * @details Must be implemented by vehicle-specific subclass. Executes the inner-loop
     *          rate PID controller, converting _ang_vel_body_rads targets into motor
     *          commands. Implementations are vehicle-specific:
     *          - AC_AttitudeControl_Multi: Multicopter rate PIDs
     *          - AC_AttitudeControl_Heli: Helicopter swashplate mixing
     *          - AC_AttitudeControl_Sub: Underwater vehicle thruster control
     * 
     * @note Called at fast loop rate (typically 400Hz for copters)
     * @note Pure virtual - must be implemented by derived class
     * 
     * @see AC_AttitudeControl_Multi::rate_controller_run()
     */
    virtual void rate_controller_run() = 0;

    /**
     * @brief Reset rate controller internal state
     * 
     * @details Resets any internal state maintained by the rate controller (e.g.,
     *          smoothing filters or integrators). Base implementation is no-op and
     *          may be overridden by child classes with internal filters.
     * 
     * @note Override in subclass if rate controller maintains state
     */
    virtual void rate_controller_target_reset() {}

    /**
     * @brief Run rate controller with specified timestep (pure virtual)
     * 
     * @param[in] gyro_rads Current gyro measurement in rad/s
     * @param[in] dt Timestep in seconds
     * 
     * @details Must be implemented by derived class. Allows external control of
     *          rate controller execution with explicit timestep and gyro measurement.
     * 
     * @note Pure virtual - must be implemented by derived class
     * @warning Default implementation triggers config error
     */
    virtual void rate_controller_run_dt(const Vector3f& gyro_rads, float dt) { AP_BoardConfig::config_error("rate_controller_run_dt() must be defined"); };

    /**
     * @brief Convert Euler angle rates to angular velocity vector
     * 
     * @param[in] att Current attitude quaternion
     * @param[in] euler_rate_rads Euler angle derivatives (roll/pitch/yaw rates) in rad/s
     * @param[out] ang_vel_rads Output body-frame angular velocity in rad/s
     * 
     * @details Converts 321-intrinsic Euler angle derivatives (earth-frame rates) to
     *          body-frame angular velocity vector using kinematic transformation.
     *          Required because Euler rates are not equal to body rates except at
     *          level flight.
     * 
     * @note Euler rates ≠ body rates (except when pitch=roll=0)
     * @note Uses current attitude for transformation
     * 
     * Source: libraries/AC_AttitudeControl/AC_AttitudeControl.cpp:1850-1900
     */
    void euler_rate_to_ang_vel(const Quaternion& att, const Vector3f& euler_rate_rads, Vector3f& ang_vel_rads);

    /**
     * @brief Convert angular velocity to Euler angle rates
     * 
     * @param[in] att Current attitude quaternion
     * @param[in] ang_vel_rads Body-frame angular velocity in rad/s
     * @param[out] euler_rate_rads Output Euler angle derivatives (roll/pitch/yaw rates) in rad/s
     * 
     * @return false if vehicle pitched ±90° (gimbal lock), true otherwise
     * 
     * @details Converts body-frame angular velocity vector to 321-intrinsic Euler angle
     *          derivatives using kinematic transformation. Returns false at gimbal lock
     *          singularity (pitch = ±90°) where transformation is undefined.
     * 
     * @warning Returns false and invalid output at pitch = ±90° (gimbal lock)
     * @note Use quaternion methods to avoid gimbal lock issues
     * 
     * Source: libraries/AC_AttitudeControl/AC_AttitudeControl.cpp:1900-1950
     */
    bool ang_vel_to_euler_rate(const Quaternion& att, const Vector3f& ang_vel_rads, Vector3f& euler_rate_rads);

    /**
     * @brief Enable/disable square root controller
     * 
     * @param[in] use_sqrt_cont true to enable square root controller, false to disable
     * 
     * @details Controls whether attitude controller uses square root controller for
     *          angle P term. Square root controller provides non-linear P gain that
     *          reduces with increasing error, improving response for large errors.
     * 
     *          Disabled during AutoTune to ensure linear P term for proper gain tuning.
     * 
     * @warning Disabling reduces stability margin - only for AutoTune
     * @note Square root controller improves large-error response
     * @note Always enabled during normal flight
     */
    void use_sqrt_controller(bool use_sqrt_cont) { _use_sqrt_controller = use_sqrt_cont; }

    /**
     * @brief Get attitude target as Euler angles (centidegrees)
     * 
     * @return Vector3f containing [roll, pitch, yaw] in centidegrees
     * 
     * @details Returns 321-intrinsic Euler angles representing the rotation from NED
     *          earth frame to the attitude controller's target attitude.
     * 
     * @note Centidegrees used for legacy telemetry compatibility
     * @note Using vector3f*deg(100) is more efficient than deg(vector3f)*100
     *       (intentional optimization, see issue 4895)
     */
    Vector3f get_att_target_euler_cd() const { return _euler_angle_target_rad * degrees(100.0f); }
    
    /**
     * @brief Get attitude target as Euler angles (radians)
     * 
     * @return Vector3f containing [roll, pitch, yaw] in radians
     * 
     * @details Returns 321-intrinsic Euler angles representing the rotation from NED
     *          earth frame to the attitude controller's target attitude.
     */
    const Vector3f & get_att_target_euler_rad() const { return _euler_angle_target_rad; }

    /**
     * @brief Get attitude target as quaternion
     * 
     * @return Quaternion representing target attitude in NED frame
     * 
     * @details Returns the target attitude quaternion used by the attitude controller.
     *          This represents the desired orientation in NED earth frame.
     *          Quaternion representation avoids gimbal lock at pitch ±90°.
     * 
     * @note Preferred over Euler angles to avoid gimbal lock
     */
    Quaternion get_attitude_target_quat() const { return _attitude_target; }

    /**
     * @brief Get target angular velocity in target attitude frame
     * 
     * @return Vector3f containing angular velocity [rad/s] in target attitude frame
     * 
     * @details Returns angular velocity of the attitude target (setpoint). This is
     *          the rate at which the target attitude itself is rotating, not the
     *          commanded body rates.
     * 
     * @note Expressed in target attitude frame, not body frame
     * @see rate_bf_targets() for body-frame rates sent to rate controller
     */
    const Vector3f& get_attitude_target_ang_vel() const { return _ang_vel_target_rads;}

    /**
     * @brief Get thrust vector error angle
     * 
     * @return Angle between target and current thrust vector in degrees
     * 
     * @details Returns the angle between the target thrust vector (derived from
     *          desired attitude) and current thrust vector (derived from actual
     *          attitude). Used for yaw limiting during large tilt angles.
     * 
     * @note Large values indicate significant attitude error
     * @note Used to reduce yaw authority when tilted
     */
    float get_att_error_angle_deg() const { return degrees(_thrust_error_angle_rad); }

    /**
     * @brief Set body-frame yaw rate target directly (centidegrees/s)
     * 
     * @param[in] rate_cds Desired yaw rate in centideg/s
     * 
     * @details Directly sets the body-frame yaw rate target. Legacy function for
     *          direct manipulation of rate targets. Prefer using input_rate_bf_*()
     *          functions for normal operation.
     * 
     * @note Direct manipulation - bypasses input shaping
     * @note Legacy function - use input_rate_bf_*() methods instead
     */
    void rate_bf_yaw_target(float rate_cds) { _ang_vel_body_rads.z = cd_to_rad(rate_cds); }

    /**
     * @brief Set roll rate for system identification
     * 
     * @param[in] rate_rads Roll rate excitation signal in rad/s
     * 
     * @details Sets the x-axis system identification angular velocity. This is added
     *          to normal rate targets to inject excitation signals for system
     *          identification, frequency response analysis, or AutoTune.
     * 
     * @note Added to normal rate commands, reset to zero after use
     * @note Used by AutoTune and system identification modes
     */
    void rate_bf_roll_sysid_rads(float rate_rads) { _sysid_ang_vel_body_rads.x = rate_rads; }

    /**
     * @brief Set pitch rate for system identification
     * 
     * @param[in] rate_rads Pitch rate excitation signal in rad/s
     * 
     * @details Sets the y-axis system identification angular velocity. This is added
     *          to normal rate targets to inject excitation signals for system
     *          identification, frequency response analysis, or AutoTune.
     * 
     * @note Added to normal rate commands, reset to zero after use
     * @note Used by AutoTune and system identification modes
     */
    void rate_bf_pitch_sysid_rads(float rate_rads) { _sysid_ang_vel_body_rads.y = rate_rads; }

    /**
     * @brief Set yaw rate for system identification
     * 
     * @param[in] rate_rads Yaw rate excitation signal in rad/s
     * 
     * @details Sets the z-axis system identification angular velocity. This is added
     *          to normal rate targets to inject excitation signals for system
     *          identification, frequency response analysis, or AutoTune.
     * 
     * @note Added to normal rate commands, reset to zero after use
     * @note Used by AutoTune and system identification modes
     */
    void rate_bf_yaw_sysid_rads(float rate_rads) { _sysid_ang_vel_body_rads.z = rate_rads; }

    /**
     * @brief Set roll actuator for system identification
     * 
     * @param[in] command Actuator command signal (unitless)
     * 
     * @details Sets x-axis system identification actuator override. Applied directly
     *          to motor outputs post-PID for direct actuator excitation. Used for
     *          open-loop system identification.
     * 
     * @note Applied post-PID, bypasses rate controller
     * @note Reset to zero immediately after use
     */
    void actuator_roll_sysid(float command) { _actuator_sysid.x = command; }

    /**
     * @brief Set pitch actuator for system identification
     * 
     * @param[in] command Actuator command signal (unitless)
     * 
     * @details Sets y-axis system identification actuator override. Applied directly
     *          to motor outputs post-PID for direct actuator excitation. Used for
     *          open-loop system identification.
     * 
     * @note Applied post-PID, bypasses rate controller
     * @note Reset to zero immediately after use
     */
    void actuator_pitch_sysid(float command) { _actuator_sysid.y = command; }

    /**
     * @brief Set yaw actuator for system identification
     * 
     * @param[in] command Actuator command signal (unitless)
     * 
     * @details Sets z-axis system identification actuator override. Applied directly
     *          to motor outputs post-PID for direct actuator excitation. Used for
     *          open-loop system identification.
     * 
     * @note Applied post-PID, bypasses rate controller
     * @note Reset to zero immediately after use
     */
    void actuator_yaw_sysid(float command) { _actuator_sysid.z = command; }

    /**
     * @brief Calculate maximum roll rate step for AutoTune
     * 
     * @return Roll rate step size in rad/s that saturates output after 4 time steps
     * 
     * @details Calculates the roll rate step size that results in maximum rate
     *          controller output after 4 time steps. Used by AutoTune to determine
     *          appropriate excitation amplitude for system identification.
     * 
     * @note Based on current PID gains and acceleration limits
     * @note Used by AutoTune for frequency sweep sizing
     * 
     * Source: libraries/AC_AttitudeControl/AC_AttitudeControl.cpp:2000-2050
     */
    float max_rate_step_bf_roll();

    /**
     * @brief Calculate maximum pitch rate step for AutoTune
     * 
     * @return Pitch rate step size in rad/s that saturates output after 4 time steps
     * 
     * @details Calculates the pitch rate step size that results in maximum rate
     *          controller output after 4 time steps. Used by AutoTune to determine
     *          appropriate excitation amplitude for system identification.
     * 
     * @note Based on current PID gains and acceleration limits
     * @note Used by AutoTune for frequency sweep sizing
     * 
     * Source: libraries/AC_AttitudeControl/AC_AttitudeControl.cpp:2050-2100
     */
    float max_rate_step_bf_pitch();

    /**
     * @brief Calculate maximum yaw rate step for AutoTune
     * 
     * @return Yaw rate step size in rad/s that saturates output after 4 time steps
     * 
     * @details Calculates the yaw rate step size that results in maximum rate
     *          controller output after 4 time steps. Used by AutoTune to determine
     *          appropriate excitation amplitude for system identification.
     * 
     * @note Based on current PID gains and acceleration limits
     * @note Used by AutoTune for frequency sweep sizing
     * 
     * Source: libraries/AC_AttitudeControl/AC_AttitudeControl.cpp:2100-2150
     */
    float max_rate_step_bf_yaw();

    /**
     * @brief Calculate maximum roll angle step for AutoTune
     * 
     * @return Roll angle step size in radians that saturates output after 4 time steps
     * 
     * @details Calculates the roll angle step size (via angle P controller) that
     *          results in maximum rate controller output after 4 time steps. Used
     *          by AutoTune for angle controller identification.
     * 
     * @note Divides max rate step by angle P gain
     * @note Used by AutoTune for angle P tuning
     */
    float max_angle_step_bf_roll() { return max_rate_step_bf_roll() / _p_angle_roll.kP(); }

    /**
     * @brief Calculate maximum pitch angle step for AutoTune
     * 
     * @return Pitch angle step size in radians that saturates output after 4 time steps
     * 
     * @details Calculates the pitch angle step size (via angle P controller) that
     *          results in maximum rate controller output after 4 time steps. Used
     *          by AutoTune for angle controller identification.
     * 
     * @note Divides max rate step by angle P gain
     * @note Used by AutoTune for angle P tuning
     */
    float max_angle_step_bf_pitch() { return max_rate_step_bf_pitch() / _p_angle_pitch.kP(); }

    /**
     * @brief Calculate maximum yaw angle step for AutoTune
     * 
     * @return Yaw angle step size in radians that saturates output after 4 time steps
     * 
     * @details Calculates the yaw angle step size (via angle P controller) that
     *          results in maximum rate controller output after 4 time steps. Used
     *          by AutoTune for angle controller identification.
     * 
     * @note Divides max rate step by angle P gain
     * @note Used by AutoTune for angle P tuning
     */
    float max_angle_step_bf_yaw() { return max_rate_step_bf_yaw() / _p_angle_yaw.kP(); }

    /**
     * @brief Get body-frame rate targets including sysid
     * 
     * @return Vector3f containing body-frame angular velocity targets [rad/s]
     * 
     * @details Returns the complete body-frame angular velocity used by the rate
     *          controller, including both normal commanded rates (_ang_vel_body_rads)
     *          and system identification excitation signals (_sysid_ang_vel_body_rads).
     * 
     * @note Includes both control commands and sysid excitation
     * @note This is what the rate controller actually tracks
     */
    Vector3f rate_bf_targets() const { return _ang_vel_body_rads + _sysid_ang_vel_body_rads; }

    /**
     * @brief Get earth-frame rate targets
     * 
     * @return Vector3f containing earth-frame Euler rate targets [rad/s]
     * 
     * @details Returns the angular velocity of the target attitude as 321-intrinsic
     *          Euler angle derivatives. These are earth-frame rates (how fast roll/
     *          pitch/yaw angles are changing), not body-frame rates.
     * 
     * @note Earth-frame rates (Euler derivatives), not body-frame rates
     * @see rate_bf_targets() for body-frame rates
     */
    const Vector3f& get_rate_ef_targets() const { return _euler_rate_target_rads; }

    /**
     * @brief Enable/disable body-frame feedforward
     * 
     * @param[in] enable_or_disable true to enable, false to disable
     * 
     * @details Controls whether rate controller uses body-frame rate feedforward.
     *          When enabled, commanded rates are added directly to PID output,
     *          improving tracking of rapid rate changes.
     * 
     * @note Improves rate tracking performance when enabled
     * @note Change not saved to EEPROM
     */
    void bf_feedforward(bool enable_or_disable) { _rate_bf_ff_enabled.set(enable_or_disable); }

    /**
     * @brief Enable/disable body-frame feedforward and save
     * 
     * @param[in] enable_or_disable true to enable, false to disable
     * 
     * @details Controls whether rate controller uses body-frame rate feedforward.
     *          When enabled, commanded rates are added directly to PID output.
     *          This version saves the setting to EEPROM.
     * 
     * @note Setting is saved to EEPROM (persistent)
     * @note Improves rate tracking performance when enabled
     */
    void bf_feedforward_save(bool enable_or_disable) { _rate_bf_ff_enabled.set_and_save(enable_or_disable); }

    /**
     * @brief Get body-frame feedforward setting
     * 
     * @return true if body-frame feedforward enabled, false otherwise
     * 
     * @details Returns current setting of body-frame rate feedforward enable flag.
     */
    bool get_bf_feedforward() { return _rate_bf_ff_enabled; }

    /**
     * @brief Enable/disable acceleration limiting
     * 
     * @param[in] enable_or_disable true to enable, false to disable
     * 
     * @details Controls whether input shaping applies acceleration limits to rate
     *          commands. When disabled, removes acceleration limiting for direct
     *          rate response. Typically disabled only during testing.
     * 
     * @warning Disabling can cause aggressive response and overshoot
     * @note Should remain enabled during normal flight
     * 
     * Source: libraries/AC_AttitudeControl/AC_AttitudeControl.cpp:2150-2170
     */
    void accel_limiting(bool enable_or_disable);

    /**
     * @brief Update altitude hold lean angle limit (pure virtual)
     * 
     * @param[in] throttle_in Current throttle input [0, 1]
     * 
     * @details Must be implemented by vehicle-specific subclass. Updates the maximum
     *          allowable lean angle during altitude hold based on current throttle
     *          and available thrust margin. Prevents excessive tilt that would cause
     *          altitude loss.
     * 
     * @note Pure virtual - must be implemented by derived class
     * @note Called continuously during altitude hold modes
     * @note Limits angle to preserve vertical thrust for altitude control
     * 
     * @see AC_AttitudeControl_Multi::update_althold_lean_angle_max()
     */
    virtual void update_althold_lean_angle_max(float throttle_in) = 0;

    /**
     * @brief Set output throttle with angle boost (pure virtual)
     * 
     * @param[in] throttle_in Desired throttle [0, 1]
     * @param[in] apply_angle_boost true to apply tilt compensation
     * @param[in] filt_cutoff Low-pass filter cutoff frequency (Hz)
     * 
     * @details Must be implemented by vehicle-specific subclass. Sets the output
     *          throttle and optionally applies angle boost (throttle increase to
     *          maintain vertical thrust during tilt). Filter smooths throttle changes.
     * 
     * @note Pure virtual - must be implemented by derived class
     * @note Angle boost compensates for cosine losses during tilt
     * 
     * @see AC_AttitudeControl_Multi::set_throttle_out()
     */
    virtual void set_throttle_out(float throttle_in, bool apply_angle_boost, float filt_cutoff) = 0;

    /**
     * @brief Get throttle input to attitude controller
     * 
     * @return Throttle value [0, 1] provided to set_throttle_out()
     * 
     * @details Returns the throttle input passed to the attitude controller via
     *          set_throttle_out(). This is the base throttle before angle boost
     *          is applied.
     * 
     * @note Does not include angle boost correction
     */
    float get_throttle_in() const { return _throttle_in; }

    /**
     * @brief Get angle boost amount
     * 
     * @return Throttle increase applied for tilt compensation [0, 1]
     * 
     * @details Returns the throttle increase applied to maintain vertical thrust
     *          during attitude tilt (angle boost). Compensates for reduced vertical
     *          component of thrust when tilted.
     * 
     * @note Angle boost = throttle_out - throttle_in
     * @note Zero when level, increases with tilt angle
     */
    float angle_boost() const { return _angle_boost; }

    /**
     * @brief Get altitude hold maximum lean angle (radians)
     * 
     * @return Maximum allowable lean angle in radians during altitude hold
     * 
     * @details Returns maximum allowable tilt angle for pilot input when in altitude
     *          hold mode. Limit is calculated based on available thrust margin to
     *          prioritize altitude stability over aggressive maneuvering.
     * 
     * @note Dynamically calculated based on throttle and thrust available
     * @note Prevents altitude loss from insufficient vertical thrust
     * 
     * Source: libraries/AC_AttitudeControl/AC_AttitudeControl.cpp:2170-2200
     */
    virtual float get_althold_lean_angle_max_rad() const;

    /**
     * @brief Get altitude hold maximum lean angle (centidegrees)
     * 
     * @return Maximum allowable lean angle in centidegrees during altitude hold
     * 
     * @details Returns maximum allowable tilt angle for pilot input when in altitude
     *          hold mode. See get_althold_lean_angle_max_rad() for full details.
     * 
     * @note Centidegrees for legacy telemetry compatibility
     * @see get_althold_lean_angle_max_rad()
     */
    float get_althold_lean_angle_max_cd() const;

    /**
     * @brief Get configured maximum lean angle (centidegrees)
     * 
     * @return Maximum lean angle from parameters (centidegrees)
     * 
     * @details Returns the configured maximum tilt angle limit from vehicle parameters
     *          (typically ANGLE_MAX parameter). This is the hard limit configured by
     *          the user, not dynamically adjusted.
     * 
     * @note From parameter ANGLE_MAX (or equivalent)
     * @note Not dynamically limited - use get_althold_lean_angle_max_cd() for dynamic limit
     */
    float lean_angle_max_cd() const { return _aparm.angle_max; }

    /**
     * @brief Get configured maximum lean angle (radians)
     * 
     * @return Maximum lean angle from parameters (radians)
     * 
     * @details Returns the configured maximum tilt angle limit from vehicle parameters.
     *          This is the hard limit configured by the user, not dynamically adjusted.
     * 
     * @note From parameter ANGLE_MAX (or equivalent)
     * @note Not dynamically limited - use get_althold_lean_angle_max_rad() for dynamic limit
     */
    float lean_angle_max_rad() const { return cd_to_rad(_aparm.angle_max); }

    /**
     * @brief Get current lean angle (degrees)
     * 
     * @return Current tilt angle in degrees
     * 
     * @details Returns angle between current thrust vector and vertical (z-axis).
     *          Zero when level, increases with tilt.
     * 
     * @note Magnitude of tilt, not direction
     */
    float lean_angle_deg() const { return degrees(_thrust_angle_rad); }
    
    /**
     * @brief Get current lean angle (radians)
     * 
     * @return Current tilt angle in radians
     * 
     * @details Returns angle between current thrust vector and vertical (z-axis).
     *          Zero when level, increases with tilt.
     * 
     * @note Magnitude of tilt, not direction
     */
    float lean_angle_rad() const { return _thrust_angle_rad; }

    /**
     * @brief Shape angle error to velocity with acceleration limits
     * 
     * @param[in] error_angle Angle error in radians
     * @param[in] input_tc Input smoothing time constant [0, 1] seconds
     * @param[in] accel_max Maximum acceleration in rad/s²
     * @param[in] target_ang_vel Current target angular velocity in rad/s
     * @param[in] desired_ang_vel Desired angular velocity in rad/s (optional)
     * @param[in] max_ang_vel Maximum angular velocity in rad/s (optional)
     * @param[in] dt Timestep in seconds
     * 
     * @return Shaped angular velocity in rad/s
     * 
     * @details Calculates velocity correction from angle error, applying acceleration/
     *          deceleration limits and jerk-limiting via smoothing gain. Core input
     *          shaping algorithm used throughout attitude controller.
     * 
     * @note Static function - can be called without instance
     * @note Applies S-curve acceleration profile for smooth response
     * 
     * Source: libraries/AC_AttitudeControl/AC_AttitudeControl.cpp:1400-1500
     */
    static float input_shaping_angle(float error_angle, float input_tc, float accel_max, float target_ang_vel, float desired_ang_vel, float max_ang_vel, float dt);
    
    /**
     * @brief Shape angle error to velocity (simplified interface)
     * 
     * @param[in] error_angle Angle error in radians
     * @param[in] input_tc Input smoothing time constant [0, 1] seconds
     * @param[in] accel_max Maximum acceleration in rad/s²
     * @param[in] target_ang_vel Current target angular velocity in rad/s
     * @param[in] dt Timestep in seconds
     * 
     * @return Shaped angular velocity in rad/s
     * 
     * @details Simplified interface to input_shaping_angle() with optional parameters
     *          set to zero (no desired velocity or max velocity constraints).
     * 
     * @see input_shaping_angle()
     */
    static float input_shaping_angle(float error_angle, float input_tc, float accel_max, float target_ang_vel, float dt){ return input_shaping_angle(error_angle,  input_tc,  accel_max,  target_ang_vel,  0.0f,  0.0f,  dt); }

    /**
     * @brief Shape angular velocity with acceleration limits
     * 
     * @param[in] target_ang_vel Current target angular velocity in rad/s
     * @param[in] desired_ang_vel Desired angular velocity in rad/s
     * @param[in] accel_max Maximum acceleration in rad/s²
     * @param[in] dt Timestep in seconds
     * @param[in] input_tc Input smoothing time constant [0, 1] seconds
     * 
     * @return Shaped angular velocity in rad/s
     * 
     * @details Shapes velocity request based on rate time constant. Angular acceleration
     *          and deceleration are limited for smooth response. Used for direct rate
     *          commands that bypass angle controller.
     * 
     * @note Static function - can be called without instance
     * @note Acceleration limiting prevents aggressive rate changes
     * 
     * Source: libraries/AC_AttitudeControl/AC_AttitudeControl.cpp:1500-1550
     */
    static float input_shaping_ang_vel(float target_ang_vel, float desired_ang_vel, float accel_max, float dt, float input_tc);

    /**
     * @brief Predict angular velocity from angle error
     * 
     * @param[in] error_angle_rad 2D angle error [roll, pitch] in radians
     * @param[out] target_ang_vel_rads Predicted angular velocity [roll, pitch] in rad/s
     * @param[in] dt Timestep in seconds
     * 
     * @details Calculates expected angular velocity correction from angle error based
     *          on current AC_AttitudeControl settings. Used to predict delay associated
     *          with angle requests for feed-forward calculations.
     * 
     * @note Uses current controller gains and acceleration limits
     * @note Useful for position controller feed-forward
     * 
     * Source: libraries/AC_AttitudeControl/AC_AttitudeControl.cpp:1550-1600
     */
    void input_shaping_rate_predictor(const Vector2f &error_angle_rad, Vector2f& target_ang_vel_rads, float dt) const;

    /**
     * @brief Apply angular velocity limits
     * 
     * @param[in,out] euler_rad Euler angle rates to limit [roll, pitch, yaw] in rad/s
     * @param[in] ang_vel_roll_max_rads Maximum roll rate in rad/s
     * @param[in] ang_vel_pitch_max_rads Maximum pitch rate in rad/s
     * @param[in] ang_vel_yaw_max_rads Maximum yaw rate in rad/s
     * 
     * @details Translates body-frame angular velocity limits to Euler axis. Applies
     *          maximum rate constraints while preserving direction of commanded rates.
     * 
     * @note Modifies euler_rad in place
     * @note Preserves rate direction, scales magnitude
     * 
     * Source: libraries/AC_AttitudeControl/AC_AttitudeControl.cpp:1600-1650
     */
    void ang_vel_limit(Vector3f& euler_rad, float ang_vel_roll_max_rads, float ang_vel_pitch_max_rads, float ang_vel_yaw_max_rads) const;

    /**
     * @brief Translate body-frame accel limits to Euler axis
     * 
     * @param[in] att Current attitude quaternion
     * @param[in] euler_accel Euler acceleration in rad/s²
     * 
     * @return Limited Euler acceleration in rad/s²
     * 
     * @details Translates body-frame acceleration limits to the Euler axis. Accounts
     *          for current attitude when applying acceleration constraints.
     * 
     * @note Attitude-dependent transformation
     * @note Returns modified acceleration vector
     * 
     * Source: libraries/AC_AttitudeControl/AC_AttitudeControl.cpp:1650-1700
     */
    Vector3f euler_accel_limit(const Quaternion &att, const Vector3f &euler_accel);

    /**
     * @brief Update attitude target with input shaping
     * 
     * @details Calculates body-frame angular velocities needed to follow the target
     *          attitude. Applies input shaping, acceleration limits, and rate limits
     *          to generate smooth, achievable rate commands. Called internally by
     *          attitude controller update loop.
     * 
     * @note Called internally - not typically called by vehicle code
     * @note Updates _ang_vel_body_rads from _attitude_target
     * 
     * Source: libraries/AC_AttitudeControl/AC_AttitudeControl.cpp:600-700
     */
    void update_attitude_target();

    /**
     * @brief Calculate thrust-heading decomposed attitude error
     * 
     * @param[in,out] attitude_target Target attitude quaternion (may be modified)
     * @param[in] attitude_body Current body attitude quaternion
     * @param[out] attitude_error_rad Attitude error as rotation vector [rad]
     * @param[out] thrust_angle_rad Thrust vector tilt angle [rad]
     * @param[out] thrust_error_angle_rad Error angle in thrust vector [rad]
     * 
     * @details Calculates two ordered rotations to move attitude_body to attitude_target.
     *          Decomposes attitude error into thrust vector correction and heading
     *          correction. Yaw error is limited based on angle yaw P value and
     *          acceleration to prevent aggressive yaw during large tilt angles.
     * 
     * @note Limits yaw error during large tilts to preserve stability
     * @note Modifies attitude_target to apply yaw limiting
     * 
     * Source: libraries/AC_AttitudeControl/AC_AttitudeControl.cpp:700-800
     */
    void thrust_heading_rotation_angles(Quaternion& attitude_target, const Quaternion& attitude_body, Vector3f& attitude_error_rad, float& thrust_angle_rad, float& thrust_error_angle_rad) const;

    /**
     * @brief Calculate thrust-vector and heading rotations
     * 
     * @param[in] attitude_target Target attitude quaternion
     * @param[in] attitude_body Current body attitude quaternion
     * @param[out] thrust_vector_correction Quaternion for thrust vector correction
     * @param[out] attitude_error_rad Attitude error as rotation vector [rad]
     * @param[out] thrust_angle_rad Thrust vector tilt angle [rad]
     * @param[out] thrust_error_angle_rad Error angle in thrust vector [rad]
     * 
     * @details Calculates two ordered rotations to move attitude_body to attitude_target.
     *          First rotation corrects thrust vector (roll/pitch), second rotation
     *          corrects heading (yaw). Separates tilt control from yaw control for
     *          thrust-vector based navigation.
     * 
     * @note Used by thrust-vector input methods
     * @note Separates tilt correction from yaw correction
     * 
     * Source: libraries/AC_AttitudeControl/AC_AttitudeControl.cpp:800-900
     */
    void thrust_vector_rotation_angles(const Quaternion& attitude_target, const Quaternion& attitude_body, Quaternion& thrust_vector_correction, Vector3f& attitude_error_rad, float& thrust_angle_rad, float& thrust_error_angle_rad) const;

    /**
     * @brief Sanity check controller parameters (virtual)
     * 
     * @details Validates controller parameters for safety. Should be called once before
     *          take-off to ensure gains and limits are within safe ranges. Base
     *          implementation is empty; derived classes override to add checks.
     * 
     * @note Virtual - may be overridden by derived class
     * @note Call during pre-arm checks
     * 
     * @see AC_AttitudeControl_Multi::parameter_sanity_check()
     */
    virtual void parameter_sanity_check() {}

    /**
     * @brief Set notch filter sample rate (virtual)
     * 
     * @param[in] sample_rate Filter sample rate in Hz
     * 
     * @details Optional override to set the notch filter sample rate for rate PID
     *          controllers if supported. Used for dynamic notch filtering. Base
     *          implementation is empty.
     * 
     * @note Virtual - may be overridden by derived class
     * @note Used for harmonic rejection (frame resonance, prop wash)
     * 
     * @see AC_AttitudeControl_Multi::set_notch_sample_rate()
     */
    virtual void set_notch_sample_rate(float sample_rate) {}

    /**
     * @brief Check if throttle mix at minimum (virtual)
     * 
     * @return true if throttle RPY mix is at minimum value
     * 
     * @details Returns true if throttle RPY mix is at its minimum value. Used to
     *          determine if attitude control should be reduced in favor of vertical
     *          thrust. Base implementation returns true (always minimum).
     * 
     * @note Virtual - may be overridden by derived class
     * @note Affects attitude authority during low throttle
     * 
     * @see AC_AttitudeControl_Multi::is_throttle_mix_min()
     */
    virtual bool is_throttle_mix_min() const { return true; }

    /**
     * @brief Set throttle mix to minimum (virtual)
     * 
     * @details Sets throttle RPY mix to minimum value (prioritize throttle over attitude).
     *          Base implementation is empty.
     * 
     * @note Virtual - may be overridden by derived class
     * @see AC_AttitudeControl_Multi::set_throttle_mix_min()
     */
    virtual void set_throttle_mix_min() {}
    
    /**
     * @brief Set throttle mix to manual (virtual)
     * 
     * @details Sets throttle RPY mix to manual flight value (balanced throttle/attitude).
     *          Base implementation is empty.
     * 
     * @note Virtual - may be overridden by derived class
     * @see AC_AttitudeControl_Multi::set_throttle_mix_man()
     */
    virtual void set_throttle_mix_man() {}
    
    /**
     * @brief Set throttle mix to maximum (virtual)
     * 
     * @param[in] ratio Throttle mix ratio
     * 
     * @details Sets throttle RPY mix to maximum value (prioritize attitude over throttle).
     *          Base implementation is empty.
     * 
     * @note Virtual - may be overridden by derived class
     * @see AC_AttitudeControl_Multi::set_throttle_mix_max()
     */
    virtual void set_throttle_mix_max(float ratio) {}
    
    /**
     * @brief Set throttle mix value (virtual)
     * 
     * @param[in] value Throttle mix value [0, 1+]
     * 
     * @details Sets throttle RPY mix to specific value. Values <1 favor throttle,
     *          values >1 favor attitude. Base implementation is empty.
     * 
     * @note Virtual - may be overridden by derived class
     * @see AC_AttitudeControl_Multi::set_throttle_mix_value()
     */
    virtual void set_throttle_mix_value(float value) {}
    
    /**
     * @brief Get current throttle mix value (virtual)
     * 
     * @return Current throttle mix value [0, 1+]
     * 
     * @details Returns current throttle RPY mix value. Base implementation returns 0.
     * 
     * @note Virtual - may be overridden by derived class
     * @see AC_AttitudeControl_Multi::get_throttle_mix()
     */
    virtual float get_throttle_mix(void) const { return 0; }

    /**
     * @brief Enable flybar passthrough on helicopter (virtual)
     * 
     * @param[in] passthrough true to enable flybar roll/pitch passthrough
     * @param[in] tail_passthrough true to enable tail rotor passthrough
     * 
     * @details Enables mechanical flybar passthrough mode on helicopters. In this mode,
     *          attitude stabilization is done mechanically by flybar. Base implementation
     *          is empty (not applicable to non-helicopter vehicles).
     * 
     * @note Virtual - helicopter-specific
     * @note Only implemented in AC_AttitudeControl_Heli
     * 
     * @see AC_AttitudeControl_Heli::use_flybar_passthrough()
     */
    virtual void use_flybar_passthrough(bool passthrough, bool tail_passthrough) {}

    /**
     * @brief Enable leaky integrator on helicopter (virtual)
     * 
     * @param[in] leaky_i true to enable leaky integrator
     * 
     * @details Controls whether to use leaky integrator for body-frame to motor output
     *          stage on helicopters. Leaky I slowly decays to prevent windup. Base
     *          implementation is empty (not applicable to non-helicopter vehicles).
     * 
     * @note Virtual - helicopter-specific
     * @note Only implemented in AC_AttitudeControl_Heli
     * 
     * @see AC_AttitudeControl_Heli::use_leaky_i()
     */
	virtual void use_leaky_i(bool leaky_i) {}

    /**
     * @brief Set hover roll trim scalar on helicopter (virtual)
     * 
     * @param[in] scalar Hover roll trim scaling factor
     * 
     * @details Scales hover roll trim parameter on helicopters. Used by vehicle code
     *          to adjust roll trim based on vehicle condition. Base implementation is
     *          empty (not applicable to non-helicopter vehicles).
     * 
     * @note Virtual - helicopter-specific
     * @note Only implemented in AC_AttitudeControl_Heli
     * 
     * @see AC_AttitudeControl_Heli::set_hover_roll_trim_scalar()
     */
    virtual void set_hover_roll_trim_scalar(float scalar) {}

    /**
     * @brief Get roll trim for hover (centidegrees, virtual)
     * 
     * @return Roll trim angle in centidegrees
     * 
     * @details Returns angle in centidegrees to be added to roll angle during hover
     *          collective learn. Used by helicopters to counteract tail rotor thrust
     *          in hover. Base implementation returns 0 (not applicable to non-helicopter).
     * 
     * @note Virtual - helicopter-specific
     * @note Only implemented in AC_AttitudeControl_Heli
     * 
     * @see AC_AttitudeControl_Heli::get_roll_trim_cd()
     */
    virtual float get_roll_trim_cd() { return 0;}

    /**
     * @brief Get roll trim for hover (radians)
     * 
     * @return Roll trim angle in radians
     * 
     * @details Returns angle in radians to be added to roll angle during hover. Used by
     *          helicopters to counteract tail rotor thrust. Calls get_roll_trim_cd() and
     *          converts to radians.
     * 
     * @note Helicopter-specific compensation
     * @see get_roll_trim_cd()
     */
    float get_roll_trim_rad() { return cd_to_rad(get_roll_trim_cd()); }

    /**
     * @brief Passthrough roll/pitch, body-frame rate yaw (centideg/s)
     * 
     * @param[in] roll_passthrough_cds Roll passthrough command (centideg/s)
     * @param[in] pitch_passthrough_cds Pitch passthrough command (centideg/s)
     * @param[in] yaw_rate_bf_cds Yaw body-frame rate (centideg/s)
     * 
     * @details Roll and pitch commands passed through directly to motors, body-frame
     *          rate target for yaw. Used for manual helicopter control where cyclic
     *          (roll/pitch) is direct mechanical control. Assumes maximum deflection
     *          rate of 45 degrees per second (π/4 rad/s).
     * 
     * @note Helicopter-specific control mode
     * @see passthrough_bf_roll_pitch_rate_yaw_rads()
     */
    void passthrough_bf_roll_pitch_rate_yaw_cds(float roll_passthrough_cds, float pitch_passthrough_cds, float yaw_rate_bf_cds);
    
    /**
     * @brief Passthrough roll/pitch, body-frame rate yaw (radians/s, virtual)
     * 
     * @param[in] roll_passthrough_rads Roll passthrough command (rad/s)
     * @param[in] pitch_passthrough_rads Pitch passthrough command (rad/s)
     * @param[in] yaw_rate_bf_rads Yaw body-frame rate (rad/s)
     * 
     * @details Roll and pitch commands passed through directly to motors, body-frame
     *          rate target for yaw. Base implementation is empty.
     * 
     * @note Virtual - helicopter-specific
     * @see AC_AttitudeControl_Heli::passthrough_bf_roll_pitch_rate_yaw_rads()
     */
    virtual void passthrough_bf_roll_pitch_rate_yaw_rads(float roll_passthrough_rads, float pitch_passthrough_rads, float yaw_rate_bf_rads) {};

    /**
     * @brief Pre-arm parameter checks
     * 
     * @param[in] param_prefix Parameter prefix string (e.g., "ATC_")
     * @param[out] failure_msg Buffer for failure message
     * @param[in] failure_msg_len Length of failure_msg buffer
     * 
     * @return true if all checks pass, false if any check fails
     * 
     * @details Provides feedback on whether arming would be safe with current parameters.
     *          Checks gains, limits, and configuration for safety issues. Writes failure
     *          reason to failure_msg if check fails.
     * 
     * @note Should be called during pre-arm checks
     * @warning Do not arm if this returns false
     * 
     * Source: libraries/AC_AttitudeControl/AC_AttitudeControl.cpp:2200-2300
     */
    bool pre_arm_checks(const char *param_prefix,
                        char *failure_msg,
                        const uint8_t failure_msg_len);

    /**
     * @brief Enable inverted flight (virtual)
     * 
     * @param[in] inverted true to enable inverted flight mode
     * 
     * @details Enables inverted flight on backends that support it (e.g., planes,
     *          tailsitters). In inverted mode, roll control is reversed. Base
     *          implementation is empty (not supported).
     * 
     * @note Virtual - only some vehicles support inverted flight
     * @note Typically used by planes and tailsitters
     * 
     * @see AC_AttitudeControl_TS::set_inverted_flight()
     */
    virtual void set_inverted_flight(bool inverted) {}

    /**
     * @brief Get inverted flight status (virtual)
     * 
     * @return true if inverted flight enabled, false otherwise
     * 
     * @details Returns whether inverted flight mode is currently enabled on backends
     *          that support it. Base implementation returns false (not supported).
     * 
     * @note Virtual - only some vehicles support inverted flight
     * @see AC_AttitudeControl_TS::get_inverted_flight()
     */
    virtual bool get_inverted_flight() { return false;}

    /**
     * @brief Get slew rate values for oscillation detection
     * 
     * @param[out] roll_srate Roll slew rate (rad/s²)
     * @param[out] pitch_srate Pitch slew rate (rad/s²)
     * @param[out] yaw_srate Yaw slew rate (rad/s²)
     * 
     * @details Returns the slew rate values (rate of change of angular velocity) for
     *          roll, pitch and yaw. Used for oscillation detection in Lua scripts and
     *          analysis tools.
     * 
     * @note Slew rate = angular acceleration (rad/s²)
     * @note Used by scripting for resonance detection
     * 
     * Source: libraries/AC_AttitudeControl/AC_AttitudeControl.cpp:2300-2350
     */
    void get_rpy_srate(float &roll_srate, float &pitch_srate, float &yaw_srate);
    
    /**
     * @brief Set roll/pitch rate shaping time constant
     * 
     * @param[in] input_tc Time constant for rate shaping [0, 1] seconds
     * 
     * @details Sets the time constant for shaping roll and pitch rate input. Controls
     *          smoothness vs responsiveness of rate commands. Lower values = more
     *          responsive, higher values = smoother.
     * 
     * @note Affects rate input smoothing
     * @note Range: [0, 1] seconds
     */
    void set_roll_pitch_rate_tc(float input_tc) { _rate_rp_tc = input_tc; }

    /**
     * @brief Set yaw rate shaping time constant
     * 
     * @param[in] input_tc Time constant for yaw rate shaping [0, 1] seconds
     * 
     * @details Sets the time constant for shaping yaw rate input. Controls smoothness
     *          vs responsiveness of yaw rate commands. Lower values = more responsive,
     *          higher values = smoother.
     * 
     * @note Affects yaw rate input smoothing
     * @note Range: [0, 1] seconds
     */
    void set_yaw_rate_tc(float input_tc) { _rate_y_tc = input_tc; }

    /**
     * @brief Set angle P gain scale (replace)
     * 
     * @param[in] angle_P_scale Scale factors for [roll, pitch, yaw] angle P gains
     * 
     * @details Sets one-loop angle P gain scale multiplier. This REPLACES any previous
     *          scale applied, so should only be used when only one source of scaling
     *          is needed. Applied for one control loop, then reset.
     * 
     * @note Replaces previous scale - does not multiply
     * @note Reset after one loop
     * @note Use set_angle_P_scale_mult() for multiple scaling sources
     * 
     * @see set_angle_P_scale_mult()
     */
    void set_angle_P_scale(const Vector3f &angle_P_scale) { _angle_P_scale = angle_P_scale; }

    /**
     * @brief Set angle P gain scale (multiply)
     * 
     * @param[in] angle_P_scale Scale factors for [roll, pitch, yaw] angle P gains
     * 
     * @details Sets one-loop angle P gain scale multiplier, multiplying by any previously
     *          applied scale from this loop. Allows multiple sources of scale factors
     *          (e.g., wind compensation + motor saturation) to be combined. Applied for
     *          one control loop, then reset.
     * 
     * @note Multiplies with previous scale - accumulates effects
     * @note Reset after one loop
     * @note Use for multiple independent scaling sources
     * 
     * @see set_angle_P_scale()
     */
    void set_angle_P_scale_mult(const Vector3f &angle_P_scale) { _angle_P_scale *= angle_P_scale; }

    /**
     * @brief Get last angle P scale used
     * 
     * @return Angle P scale [roll, pitch, yaw] used in last loop
     * 
     * @details Returns the value of the angle P scale that was actually applied in the
     *          last control loop. Useful for logging and diagnostics to understand
     *          gain scaling effects.
     * 
     * @note Returns final combined scale from last loop
     * @note Useful for logging and analysis
     */
    const Vector3f &get_last_angle_P_scale(void) const { return _angle_P_scale_used; }
    
    /**
     * @brief Set PD gain scale multiplier
     * 
     * @param[in] pd_scale Scale factors for [roll, pitch, yaw] PD gains
     * 
     * @details Sets one-loop PD (proportional-derivative) gain scale multiplier,
     *          multiplying by any previously applied scale from this loop. Allows
     *          multiple sources of scale factors to be combined. Applied for one
     *          control loop, then reset.
     * 
     * @note Multiplies with previous scale - accumulates effects
     * @note Reset after one loop
     * @note Affects both P and D terms together
     */
    void set_PD_scale_mult(const Vector3f &pd_scale) { _pd_scale *= pd_scale; }

    /**
     * @brief Write RATE log message
     * 
     * @param[in] pos_control Position controller reference for additional data
     * 
     * @details Writes RATE log message with rate controller targets, outputs, and errors.
     *          Used for tuning and analysis. Includes rate targets, gyro measurements,
     *          PID outputs, and position controller velocity targets.
     * 
     * @note Called at main loop rate for logging
     * @note Log message: RATE
     * 
     * Source: libraries/AC_AttitudeControl/AC_AttitudeControl.cpp:2350-2400
     */
    void Write_Rate(const AC_PosControl &pos_control) const;

    /**
     * @brief Write ANG log message
     * 
     * @details Writes ANG log message with attitude controller targets and errors.
     *          Used for tuning and analysis. Includes attitude targets, current attitude,
     *          attitude errors, and thrust vector angle.
     * 
     * @note Called at main loop rate for logging
     * @note Log message: ANG
     * 
     * Source: libraries/AC_AttitudeControl/AC_AttitudeControl.cpp:2400-2450
     */
    void Write_ANG() const;

    /**
     * @brief User-settable parameter table
     * 
     * @details AP_Param group info table defining user-accessible parameters. Includes
     *          acceleration limits (ATC_ACCEL_R/P/Y_MAX), angular velocity limits
     *          (ATC_ANG_RTE_R/P/Y_MAX), input time constants (ATC_INPUT_TC), angle P
     *          gains (ATC_RAT_R/P/Y_P), feedforward enables, and angle limit settings.
     *          Defined in AC_AttitudeControl.cpp.
     * 
     * @note Parameters prefixed with ATC_*
     * @note Defined in .cpp file
     * 
     * Source: libraries/AC_AttitudeControl/AC_AttitudeControl.cpp:50-200
     */
    static const struct AP_Param::GroupInfo var_info[];

    /**
     * @brief Unity vector constant [1, 1, 1]
     * 
     * @details Convenience constant for operations requiring unity scale vector.
     *          Used internally for gain scaling operations.
     */
    static constexpr Vector3f VECTORF_111{1.0f,1.0f,1.0f};

protected:

    /**
     * @brief Update angular velocity target from attitude error
     * 
     * @param[in] attitude_error_rot_vec_rad Attitude error as rotation vector (rad)
     * 
     * @return Updated angular velocity target (rad/s)
     * 
     * @details Updates rate target angular velocity using attitude error rotation vector.
     *          Applies square root controller if enabled, acceleration limits, and input
     *          shaping to convert attitude error to smooth rate command. Core of the
     *          angle-to-rate outer loop.
     * 
     * @note Protected - internal helper for attitude controller
     * @note Applies square root controller for variable gain
     * 
     * Source: libraries/AC_AttitudeControl/AC_AttitudeControl.cpp:700-800
     */
    Vector3f update_ang_vel_target_from_att_error(const Vector3f &attitude_error_rot_vec_rad);

    /**
     * @brief Get latest gyro reading
     * 
     * @return Most recent angular velocity reading (rad/s)
     * 
     * @details Returns the most recent angular velocity reading for the rate controller.
     *          Ensures minimum latency when rate control is run before or after attitude
     *          control by checking for updated gyro data. Uses cached gyro if fresh data
     *          not available.
     * 
     * @note Protected - internal helper for rate controller
     * @note Minimizes control latency by using freshest data
     * 
     * Source: libraries/AC_AttitudeControl/AC_AttitudeControl.cpp:2450-2500
     */
    const Vector3f get_latest_gyro() const;

    // Maximum rate the yaw target can be updated in Loiter, RTL, Auto flight modes (centideg/s)
    // Parameter: ATC_SLEW_YAW, prevents aggressive yaw during autonomous navigation
    AP_Float            _slew_yaw_cds;

    // Maximum angular velocity (deg/s) for earth-frame roll axis
    // Parameter: ATC_ANG_RTE_R_MAX, limits commanded roll rate
    AP_Float            _ang_vel_roll_max_degs;
    
    // Maximum angular velocity (deg/s) for earth-frame pitch axis
    // Parameter: ATC_ANG_RTE_P_MAX, limits commanded pitch rate
    AP_Float            _ang_vel_pitch_max_degs;
    
    // Maximum angular velocity (deg/s) for earth-frame yaw axis
    // Parameter: ATC_ANG_RTE_Y_MAX, limits commanded yaw rate
    AP_Float            _ang_vel_yaw_max_degs;

    // Maximum rotation acceleration for earth-frame roll axis (centideg/s²)
    // Parameter: ATC_ACCEL_R_MAX, controls roll responsiveness vs smoothness
    AP_Float            _accel_roll_max_cdss;

    // Maximum rotation acceleration for earth-frame pitch axis (centideg/s²)
    // Parameter: ATC_ACCEL_P_MAX, controls pitch responsiveness vs smoothness
    AP_Float            _accel_pitch_max_cdss;

    // Maximum rotation acceleration for earth-frame yaw axis (centideg/s²)
    // Parameter: ATC_ACCEL_Y_MAX, controls yaw responsiveness vs smoothness
    AP_Float            _accel_yaw_max_cdss;

    // Enable/Disable body frame rate feed forward [0=disabled, 1=enabled]
    // Parameter: ATC_RATE_FF_ENAB, improves rate tracking
    AP_Int8             _rate_bf_ff_enabled;

    // Enable/Disable angle boost [0=disabled, 1=enabled]
    // Parameter: ATC_ANGLE_BOOST, increases throttle during tilt
    AP_Int8             _angle_boost_enabled;

    // Angle controller P object for roll (converts angle error to rate command)
    // Parameter group: ATC_RAT_RLL_*
    AC_P                _p_angle_roll;
    
    // Angle controller P object for pitch (converts angle error to rate command)
    // Parameter group: ATC_RAT_PIT_*
    AC_P                _p_angle_pitch;
    
    // Angle controller P object for yaw (converts angle error to rate command)
    // Parameter group: ATC_RAT_YAW_*
    AC_P                _p_angle_yaw;

    // Angle limit time constant [s] to maintain altitude during aggressive maneuvers
    // Parameter: ATC_ANGLE_LIM_TC, prevents excessive tilt near throttle limits
    AP_Float            _angle_limit_tc;

    // Input time constant [s] for smoothing user/control input before rate controller
    // Parameter: ATC_INPUT_TC [0, 1], affects angular response stiffness
    AP_Float            _input_tc;

    // Controller gain multiplier for roll axis when landed (reduces ground resonance)
    // Parameter: ATC_LAND_RLL_MULT, typically 0.5-1.0
    AP_Float            _land_roll_mult;
    
    // Controller gain multiplier for pitch axis when landed (reduces ground resonance)
    // Parameter: ATC_LAND_PIT_MULT, typically 0.5-1.0
    AP_Float            _land_pitch_mult;
    
    // Controller gain multiplier for yaw axis when landed (reduces ground resonance)
    // Parameter: ATC_LAND_YAW_MULT, typically 0.5-1.0
    AP_Float            _land_yaw_mult;

    // Latest body-frame gyro measurement (rad/s) used by rate controller
    // Updated by get_latest_gyro(), cached to minimize latency
    Vector3f            _rate_gyro_rads;
    
    // Timestamp of the latest gyro measurement (microseconds)
    // Used to detect fresh gyro data availability
    uint64_t            _rate_gyro_time_us;

    // Intersampling period (seconds) - timestep between controller updates
    // Must match actual update rate for correct integration/filtering
    float               _dt_s;

    // Target attitude as 321-intrinsic Euler angles (rad) in NED frame
    // Roll, pitch, yaw setpoint for attitude controller
    Vector3f            _euler_angle_target_rad;

    // Target attitude rate as 321-intrinsic Euler angle derivatives (rad/s)
    // Earth-frame angular velocity representation of attitude target
    Vector3f            _euler_rate_target_rads;

    // Target attitude quaternion in NED frame (earth frame to body frame rotation)
    // Primary attitude setpoint representation, avoids gimbal lock
    Quaternion          _attitude_target;

    // Target angular velocity (rad/s) in target attitude frame
    // Intermediate representation between attitude error and body-frame rates
    Vector3f            _ang_vel_target_rads;

    // Target angular velocity (rad/s) in body frame - THIS is what rate controller tracks
    // Final output of attitude controller, input to rate controller
    // CRITICAL: Updated atomically to avoid race conditions with rate controller
    Vector3f            _ang_vel_body_rads;

    // System identification angular velocity injection (rad/s) in body frame
    // Added to _ang_vel_body_rads for frequency sweep and system ID
    // Reset to zero immediately after use each loop
    Vector3f            _sysid_ang_vel_body_rads;

    // System identification actuator override (unitless [-1, 1])
    // Direct motor command injection post-PID for excitation
    // Reset to zero immediately after use each loop
    Vector3f            _actuator_sysid;

    // Attitude error quaternion in body frame
    // Used for inertial frame reset handling when EKF corrects attitude
    Quaternion          _attitude_ang_error;

    // Current thrust vector angle (rad) - angle between thrust and vertical
    // Used for angle boost calculation and monitoring vehicle tilt
    float               _thrust_angle_rad;

    // Thrust error angle (rad) - angle between target and actual thrust vectors
    // Used to limit yaw authority when vehicle is tilted (prevents adverse coupling)
    float               _thrust_error_angle_rad;

    // Throttle input [0, 1] provided to attitude controller (before angle boost)
    // Cached for angle boost calculation and logging
    float               _throttle_in = 0.0f;

    // Angle boost [0, 1] - throttle increase to maintain vertical thrust during tilt
    // Calculated as 1/cos(tilt_angle) - 1, applied to maintain altitude
    float               _angle_boost;

    // Enable square root controller (variable P gain based on error magnitude)
    // Disabled during AutoTune to get linear P response for tuning
    // true = enabled (normal), false = disabled (AutoTune)
    bool                _use_sqrt_controller;

    // Filtered maximum lean angle (rad) for altitude hold modes
    // Dynamically limited based on throttle saturation to prioritize altitude stability
    float               _althold_lean_angle_max_rad = 0.0f;

    // Desired throttle RPY mix [0, 1+] (target value)
    // 0 = prioritize throttle, 1 = balanced, >1 = prioritize attitude
    // Slewed towards over 1-2 seconds to prevent sudden changes
    float               _throttle_rpy_mix_desired;

    // Current throttle RPY mix [0, 1+] (slewed value)
    // Mix between throttle and attitude control priority
    // 0 to 1: linear mix, >1: ratio above hover throttle
    float               _throttle_rpy_mix;

    // Yaw feedforward scalar [0, 1] to limit yaw during extreme roll/pitch
    // Reduces yaw actuator output when vehicle is tilted to prevent coupling
    float               _feedforward_scalar = 1.0f;

    // Time constant [s] for shaping roll/pitch rate input
    // Affects rate command smoothing and responsiveness
    float               _rate_rp_tc;
    
    // Time constant [s] for shaping yaw rate input
    // Affects yaw rate command smoothing and responsiveness
    float               _rate_y_tc;

    // Active angle P gain scaling [roll, pitch, yaw] for this loop
    // Accumulated from multiple sources (wind, saturation, etc.)
    // Reset to [1, 1, 1] each loop
    Vector3f            _angle_P_scale{1,1,1};

    // Angle P gain scaling [roll, pitch, yaw] actually applied last loop
    // Cached for logging and diagnostics
    Vector3f            _angle_P_scale_used;

    // Active PD gain scaling [roll, pitch, yaw] for this loop
    // Accumulated from multiple sources
    // Reset to [1, 1, 1] each loop
    Vector3f            _pd_scale{1,1,1};

    // PD gain scaling [roll, pitch, yaw] actually applied last loop
    // Cached for logging and diagnostics
    Vector3f            _pd_scale_used;

    // Gain reduction ratio [0, 1] when landed
    // Multiplies rate controller gains to suppress ground resonance
    float               _landed_gain_ratio;

    // Reference to AHRS (Attitude and Heading Reference System)
    // Provides current attitude, rates, and rotation matrices
    const AP_AHRS_View&  _ahrs;
    
    // Reference to vehicle parameters (angle limits, etc.)
    // Provides configured angle_max and other vehicle-specific limits
    const AP_MultiCopter &_aparm;
    
    // Reference to motor mixer/output manager
    // Provides motor saturation info and applies commands
    AP_Motors&          _motors;

    // Singleton instance pointer for global access
    // Allows vehicle code to access controller via AC_AttitudeControl::get_singleton()
    static AC_AttitudeControl *_singleton;

public:
    /**
     * @enum HeadingMode
     * @brief Heading control mode enumeration
     * 
     * @details Specifies how yaw/heading should be controlled in thrust vector commands.
     *          Used with HeadingCommand structure for flexible heading control.
     */
    enum class HeadingMode {
        Angle_Only,        ///< Control absolute heading angle only (hold yaw)
        Angle_And_Rate,    ///< Control heading angle with rate feedforward
        Rate_Only          ///< Control heading rate only (yaw rate command)
    };
    
    /**
     * @struct HeadingCommand
     * @brief Heading command structure combining angle and rate
     * 
     * @details Encapsulates yaw/heading command with flexible control mode. Allows
     *          specifying heading as absolute angle, rate, or combination. Used with
     *          input_thrust_vector_heading() for advanced attitude control.
     */
    struct HeadingCommand {
        float yaw_angle_rad;       ///< Target yaw angle (rad) in NED frame
        float yaw_rate_rads;       ///< Target yaw rate (rad/s)
        HeadingMode heading_mode;  ///< Control mode (angle, rate, or both)
    };
    
    /**
     * @brief Set thrust vector and heading command
     * 
     * @param[in] thrust_vector Desired thrust direction vector (body frame)
     * @param[in] heading Heading command with mode, angle, and/or rate
     * 
     * @details Sets desired thrust vector and heading using flexible HeadingCommand
     *          structure. Supports angle-only, rate-only, or combined angle+rate
     *          heading control. Used for advanced navigation modes.
     * 
     * @note More flexible than input_thrust_vector_heading_rad()
     * @see input_thrust_vector_heading_rad()
     * @see HeadingCommand
     * @see HeadingMode
     * 
     * Source: libraries/AC_AttitudeControl/AC_AttitudeControl.cpp:1800-1900
     */
    void input_thrust_vector_heading(const Vector3f& thrust_vector, HeadingCommand heading);
};
