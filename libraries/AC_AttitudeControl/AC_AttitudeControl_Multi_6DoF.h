/**
 * @file AC_AttitudeControl_Multi_6DoF.h
 * @brief 6DoF (6 Degrees of Freedom) attitude control extension for thrust-vectoring multicopters
 * 
 * @details Extends AC_AttitudeControl_Multi to support vehicles with thrust-vectoring capability.
 *          Allows independent control of thrust direction in addition to vehicle attitude.
 *          Enables multicopters to translate horizontally without tilting the vehicle body,
 *          by tilting motors or thrust vectors directly. Only compiled when AP_SCRIPTING_ENABLED.
 *          
 *          The 6DoF controller separates thrust vector control from vehicle attitude control,
 *          allowing the vehicle to maintain level flight while moving in any horizontal direction.
 *          This is achieved by computing forward and lateral motor tilts that produce the desired
 *          thrust vector, while maintaining the desired vehicle attitude independently.
 *          
 *          Typical use cases:
 *          - Thrust-vectoring multicopters with tiltable motors
 *          - Vehicles requiring decoupled position and attitude control
 *          - Advanced scripting applications for custom flight behaviors
 *          
 * @note Only available when AP_SCRIPTING_ENABLED (requires scripting feature)
 * @warning Thrust-vector commands can cause instability if motor limits exceeded
 * 
 * @see AC_AttitudeControl_Multi Base multicopter attitude controller
 * @see AP_Motors Motor mixing and output
 * @see AP_Scripting Lua scripting interface for 6DoF control
 */

#pragma once
#if AP_SCRIPTING_ENABLED

#include "AC_AttitudeControl_Multi.h"

/**
 * @class AC_AttitudeControl_Multi_6DoF
 * @brief 6DoF thrust-vectoring multicopter attitude controller
 * 
 * @details Provides additional input methods for direct thrust-vector control beyond standard
 *          multicopter attitude control. Computes forward and lateral motor tilts to produce
 *          desired thrust vectors while maintaining independent attitude control. Used by
 *          scripting to implement custom thrust-vectoring behaviors.
 *          
 *          The controller maintains an internal attitude offset (roll_offset_deg, pitch_offset_deg)
 *          that represents the neutral vehicle attitude. Thrust-vector commands are computed
 *          relative to this offset, allowing the vehicle to translate without changing its
 *          perceived attitude.
 *          
 *          Coordinate frame: All thrust vectors are specified in body frame using NED convention:
 *          - Forward (X): +X axis points forward
 *          - Right (Y): +Y axis points right
 *          - Down (Z): +Z axis points down
 *          
 *          Motor tilt sign conventions:
 *          - Positive forward tilt: motors tilt forward (+X direction)
 *          - Positive lateral tilt: motors tilt right (+Y direction)
 *          
 * @note Singleton pattern enforced for scripting API access
 * @note Thrust vectors are normalized to unit vectors internally
 * @note Motor tilt commands passed directly to motor library
 * @note Forward and lateral thrust can be independently enabled/disabled
 * 
 * @warning Thrust-vector commands can cause instability if motor limits exceeded
 * @warning Thrust-vector tilts must be within motor mechanical limits
 * @warning Coordinate frame is body-frame NED (forward=+X, right=+Y, down=+Z)
 * @warning Improper thrust-vector commands can violate attitude controller assumptions
 * @warning Always validate motor tilt limits before commanding 6DoF maneuvers
 */
class AC_AttitudeControl_Multi_6DoF : public AC_AttitudeControl_Multi {
public:
    /**
     * @brief Construct 6DoF attitude controller
     * 
     * @details Initializes the 6DoF attitude controller by calling the base class constructor
     *          and registering this instance as the singleton. The singleton pattern ensures
     *          that scripting has a single global access point to the controller.
     *          
     *          Forward and lateral thrust-vectoring are enabled by default. The attitude offsets
     *          are initialized to zero, meaning the vehicle's level attitude matches the
     *          physical level orientation initially.
     * 
     * @param[in] ahrs Reference to AHRS (Attitude and Heading Reference System) for state estimation
     * @param[in] aparm Reference to multicopter parameter set
     * @param[in] motors Reference to multicopter motor mixer and output system
     * 
     * @note Panics if multiple instances are created (singleton enforcement)
     * @warning Only one AC_AttitudeControl_Multi_6DoF instance can exist per vehicle
     */
    AC_AttitudeControl_Multi_6DoF(AP_AHRS_View &ahrs, const AP_MultiCopter &aparm, AP_MotorsMulticopter& motors):
        AC_AttitudeControl_Multi(ahrs,aparm,motors) {

        if (_singleton != nullptr) {
            AP_HAL::panic("Can only be one AC_AttitudeControl_Multi_6DoF");
        }
        _singleton = this;
    }

    /**
     * @brief Get singleton instance for scripting access
     * 
     * @details Returns a pointer to the single AC_AttitudeControl_Multi_6DoF instance,
     *          allowing scripts to access the 6DoF control functionality. Returns nullptr
     *          if no instance has been created (vehicle not configured for 6DoF control).
     * 
     * @return Pointer to singleton instance, or nullptr if not instantiated
     * 
     * @note Used by AP_Scripting binding to provide Lua access to 6DoF methods
     */
    static AC_AttitudeControl_Multi_6DoF *get_singleton() {
        return _singleton;
    }

    /**
     * @brief Set desired attitude using quaternion and body-frame angular velocity
     * 
     * @details Sets a desired attitude using a quaternion and body-frame angular velocity (rad/s).
     *          The desired quaternion is incrementally updated each timestep. Angular velocity
     *          is shaped by acceleration limits and feedforward. In 6DoF mode, this method
     *          converts the desired attitude into thrust vector angles and substitutes the
     *          configured offset angles.
     *          
     *          This override applies the attitude offset (roll_offset_deg, pitch_offset_deg)
     *          to decouple the vehicle's physical attitude from its control attitude, enabling
     *          thrust-vectoring to achieve desired motion.
     * 
     * @param[in,out] attitude_desired_quat Desired vehicle attitude quaternion (will be modified for offset)
     * @param[in] ang_vel_body Desired angular velocity in body frame (rad/s)
     * 
     * @note Quaternion should be normalized before passing to this function
     * @note This method overrides input functions to convert desired angles into thrust angles
     */
    void input_quaternion(Quaternion& attitude_desired_quat, Vector3f ang_vel_body) override;

    /**
     * @brief Set desired roll and pitch angles with yaw rate
     * 
     * @details Sets desired roll and pitch angles (in radians) and yaw rate (in radians/s).
     *          Used when roll/pitch stabilization is needed with manual or autonomous yaw rate control.
     *          Applies acceleration-limited input shaping for smooth transitions and computes body-frame
     *          angular velocity targets. In 6DoF mode, applies attitude offsets to convert attitude
     *          commands into thrust-vector tilts.
     * 
     * @param[in] euler_roll_angle_rad Desired roll angle in radians
     * @param[in] euler_pitch_angle_rad Desired pitch angle in radians  
     * @param[in] euler_yaw_rate_rads Desired yaw rate in radians/s
     * 
     * @note Input shaping applies acceleration limits for smooth control
     * @note Attitude offsets are applied in 6DoF mode for thrust-vectoring
     */
    void input_euler_angle_roll_pitch_euler_rate_yaw_rad(float euler_roll_angle_rad, float euler_pitch_angle_rad, float euler_yaw_rate_rads)  override;


    /**
     * @brief Set desired roll, pitch, and yaw angles
     * 
     * @details Sets desired roll, pitch, and yaw angles (in radians). Used to follow an absolute
     *          attitude setpoint. Input shaping and yaw slew limits are applied. Outputs are passed
     *          to the rate controller via shaped angular velocity targets. In 6DoF mode, applies
     *          attitude offsets to convert attitude commands into thrust-vector tilts.
     * 
     * @param[in] euler_roll_angle_rad Desired roll angle in radians
     * @param[in] euler_pitch_angle_rad Desired pitch angle in radians
     * @param[in] euler_yaw_angle_rad Desired yaw angle in radians
     * @param[in] slew_yaw If true, apply yaw slew rate limiting for smooth transitions
     * 
     * @note Input shaping and acceleration limits are applied
     * @note Yaw slew limiting prevents rapid yaw changes when enabled
     */
    void input_euler_angle_roll_pitch_yaw_rad(float euler_roll_angle_rad, float euler_pitch_angle_rad, float euler_yaw_angle_rad, bool slew_yaw) override;

    /**
     * @brief Set desired thrust vector and heading rate
     * 
     * @details Sets desired thrust vector and heading rate (in radians/s). Used for tilt-based
     *          navigation with independent yaw control. The thrust vector defines the desired
     *          orientation (e.g., pointing direction for vertical thrust), while the heading rate
     *          adjusts yaw independently. The input is shaped by acceleration and slew limits.
     *          
     *          In 6DoF mode, this allows the vehicle to point the thrust in any direction while
     *          controlling yaw independently, enabling decoupled translational and rotational control.
     *          The thrust vector is normalized internally and converted to motor tilts.
     * 
     * @param[in] thrust_vector Desired thrust direction in body frame NED (forward=+X, right=+Y, down=+Z)
     * @param[in] heading_rate_rads Desired yaw rate in radians/s
     * @param[in] slew_yaw If true, apply yaw slew rate limiting (default: true)
     * 
     * @note Thrust vector is normalized to unit vector internally
     * @note Body frame NED convention: +X forward, +Y right, +Z down
     * @warning Thrust vector must not be zero length
     * @warning Coordinate frame is body-frame NED
     * 
     * @see input_thrust_vector_heading_rad() For heading angle control instead of rate
     */
    void input_thrust_vector_rate_heading_rads(const Vector3f& thrust_vector, float heading_rate_rads, bool slew_yaw = true) override;
    
    /**
     * @brief Set desired thrust vector and heading angle
     * 
     * @details Sets desired thrust vector and heading (in radians) with heading rate (in radians/s).
     *          Used for advanced attitude control where thrust direction is separated from yaw
     *          orientation. Heading slew is constrained based on configured limits. This enables
     *          full 6DoF control by independently commanding thrust direction and vehicle heading.
     *          
     *          The thrust vector is converted to motor tilts while maintaining the desired heading.
     *          This allows the vehicle to fly in any direction while pointing its nose in a
     *          different direction (e.g., fly sideways while facing forward).
     * 
     * @param[in] thrust_vector Desired thrust direction in body frame NED (forward=+X, right=+Y, down=+Z)
     * @param[in] heading_angle_rad Desired yaw angle in radians
     * @param[in] heading_rate_rads Desired yaw rate in radians/s (feedforward)
     * 
     * @note Thrust vector is normalized to unit vector internally
     * @note Heading slew rate limiting is applied based on configured limits
     * @warning Thrust vector must not be zero length
     * @warning Coordinate frame is body-frame NED
     * 
     * @see input_thrust_vector_rate_heading_rads() For heading rate control instead of angle
     */
    void input_thrust_vector_heading_rad(const Vector3f& thrust_vector, float heading_angle_rad, float heading_rate_rads) override;

    /**
     * @note All remaining input functions zero thrust vectoring and behave as a normal multicopter
     * @note These methods disable 6DoF thrust-vectoring to maintain compatibility with standard modes
     */

    /**
     * @brief Command yaw rate and pitch angle with body-frame roll angle (tailsitter mode)
     * 
     * @details Commands euler yaw rate and pitch angle with roll angle specified in body frame.
     *          Used only by tailsitter quadplanes. In 6DoF mode, this method zeros thrust
     *          vectoring to behave like a normal copter, ensuring compatibility with tailsitter
     *          control modes.
     * 
     * @param[in] plane_controls If true, use plane control convention
     * @param[in] euler_roll_angle_rad Roll angle in body frame (radians)
     * @param[in] euler_pitch_angle_rad Pitch angle in radians
     * @param[in] euler_yaw_rate_rads Yaw rate in radians/s
     * 
     * @note Thrust vectoring is disabled in this mode
     * @note Used exclusively by tailsitter quadplane configurations
     */
    void input_euler_rate_yaw_euler_angle_pitch_bf_roll_rad(bool plane_controls, float euler_roll_angle_rad, float euler_pitch_angle_rad, float euler_yaw_rate_rads) override;

    /**
     * @brief Command euler roll, pitch, and yaw rates with feedforward and smoothing
     * 
     * @details Commands euler roll, pitch, and yaw rates with angular velocity feedforward and
     *          smoothing. In 6DoF mode, this method zeros thrust vectoring to behave like a
     *          normal copter, ensuring rate commands are applied as pure rotational rates.
     * 
     * @param[in] euler_roll_rate_rads Desired roll rate in radians/s
     * @param[in] euler_pitch_rate_rads Desired pitch rate in radians/s
     * @param[in] euler_yaw_rate_rads Desired yaw rate in radians/s
     * 
     * @note Thrust vectoring is disabled in this mode
     * @note Angular velocity feedforward and smoothing are applied
     */
    void input_euler_rate_roll_pitch_yaw_rads(float euler_roll_rate_rads, float euler_pitch_rate_rads, float euler_yaw_rate_rads) override;

    /**
     * @brief Set desired body-frame angular rates (fully stabilized acro)
     * 
     * @details Sets desired roll, pitch, and yaw angular rates in body-frame (in radians/s).
     *          This command is used by fully stabilized acro modes. It applies angular velocity
     *          targets in the body frame, shaped using acceleration limits and passed to the rate
     *          controller. In 6DoF mode, thrust vectoring is disabled to maintain pure acro behavior.
     * 
     * @param[in] roll_rate_bf_rads Desired roll rate in body frame (radians/s)
     * @param[in] pitch_rate_bf_rads Desired pitch rate in body frame (radians/s)
     * @param[in] yaw_rate_bf_rads Desired yaw rate in body frame (radians/s)
     * 
     * @note Thrust vectoring is disabled in this mode
     * @note Acceleration limits and input shaping are applied
     * @note Used by fully stabilized acro flight modes
     */
    void input_rate_bf_roll_pitch_yaw_rads(float roll_rate_bf_rads, float pitch_rate_bf_rads, float yaw_rate_bf_rads) override;

    /**
     * @brief Set desired body-frame angular rates (Copter rate-only acro)
     * 
     * @details Sets desired roll, pitch, and yaw angular rates in body-frame (in radians/s).
     *          Used by Copter's rate-only acro mode. Applies raw angular velocity targets directly
     *          to the rate controller with smoothing and no attitude feedback or stabilization.
     *          In 6DoF mode, thrust vectoring is disabled to maintain pure rate control behavior.
     * 
     * @param[in] roll_rate_bf_rads Desired roll rate in body frame (radians/s)
     * @param[in] pitch_rate_bf_rads Desired pitch rate in body frame (radians/s)
     * @param[in] yaw_rate_bf_rads Desired yaw rate in body frame (radians/s)
     * 
     * @note Thrust vectoring is disabled in this mode
     * @note No attitude feedback - pure rate control only
     * @note Used by Copter's rate-only acro flight mode
     */
    void input_rate_bf_roll_pitch_yaw_2_rads(float roll_rate_bf_rads, float pitch_rate_bf_rads, float yaw_rate_bf_rads) override;

    /**
     * @brief Set desired body-frame angular rates (Plane acro with error integration)
     * 
     * @details Sets desired roll, pitch, and yaw angular rates in body-frame (in radians/s).
     *          Used by Plane's acro mode with rate error integration. Integrates attitude error
     *          over time to generate target angular rates. In 6DoF mode, thrust vectoring is
     *          disabled to maintain compatibility with Plane's acro mode.
     * 
     * @param[in] roll_rate_bf_rads Desired roll rate in body frame (radians/s)
     * @param[in] pitch_rate_bf_rads Desired pitch rate in body frame (radians/s)
     * @param[in] yaw_rate_bf_rads Desired yaw rate in body frame (radians/s)
     * 
     * @note Thrust vectoring is disabled in this mode
     * @note Rate error integration provides smoother control
     * @note Used by Plane's acro flight mode
     */
    void input_rate_bf_roll_pitch_yaw_3_rads(float roll_rate_bf_rads, float pitch_rate_bf_rads, float yaw_rate_bf_rads) override;

    /**
     * @brief Apply one-time angular offset step (autotune / test input)
     * 
     * @details Applies a one-time angular offset in body-frame roll/pitch/yaw angles (in radians).
     *          Used for initiating step responses during autotuning or manual test inputs. In 6DoF
     *          mode, thrust vectoring is disabled to ensure pure attitude step responses for tuning.
     * 
     * @param[in] roll_angle_step_bf_rad Roll angle step in body frame (radians)
     * @param[in] pitch_angle_step_bf_rad Pitch angle step in body frame (radians)
     * @param[in] yaw_angle_step_bf_rad Yaw angle step in body frame (radians)
     * 
     * @note Thrust vectoring is disabled in this mode
     * @note Used for autotune step response testing
     * @warning Only use for controlled testing - abrupt angle steps can destabilize flight
     */
    void input_angle_step_bf_roll_pitch_yaw_rad(float roll_angle_step_bf_rad, float pitch_angle_step_bf_rad, float yaw_angle_step_bf_rad) override;

    /**
     * @brief Run body-frame rate controller and send outputs to motors
     * 
     * @details Runs the lowest level body-frame rate controller and sends outputs to the motors.
     *          In 6DoF mode, applies the computed motor tilts for thrust-vectoring before passing
     *          control signals to the motor mixer. Called at the main loop rate (typically 400Hz).
     * 
     * @note Called at main loop rate (typically 400Hz)
     * @note Motor tilt commands are applied during this call
     * @note Thrust-vectoring motor offsets are sent to motor library
     */
    void rate_controller_run() override;

    /**
     * @brief Get maximum lean angle for altitude hold (always 90 degrees for 6DoF)
     * 
     * @details Returns maximum lean angle for altitude hold mode. Limiting lean angle based on
     *          throttle makes no sense for 6DoF vehicles, which can translate horizontally without
     *          tilting. Always returns 90 degrees (full authority) in radians.
     * 
     * @return Maximum lean angle in radians (always π/2 = 90 degrees for 6DoF)
     * 
     * @note 6DoF vehicles can translate without tilting, so lean angle limits don't apply
     * @note Standard multicopters limit lean angle based on throttle to maintain altitude
     */
    float get_althold_lean_angle_max_rad() const override { return radians(90.0f); }

    /**
     * @brief Set the attitude offset for 6DoF flight
     * 
     * @details Sets the neutral attitude (roll and pitch offsets in degrees) that will be used
     *          as the reference for 6DoF flight. This offset defines the vehicle's "level" attitude,
     *          allowing thrust-vectoring commands to be computed relative to this reference.
     *          
     *          For example, setting a 10-degree forward pitch offset means the vehicle will maintain
     *          a 10-degree nose-down attitude as its neutral position, and thrust vectors will be
     *          computed relative to this tilted reference frame.
     * 
     * @param[in] roll_deg Roll offset angle in degrees (positive = roll right)
     * @param[in] pitch_deg Pitch offset angle in degrees (positive = pitch up/nose up)
     * 
     * @note Offsets are applied to decouple physical attitude from control attitude
     * @note Allows vehicle to maintain tilted orientation while computing thrust vectors
     * @warning Large offsets may exceed motor tilt limits or violate attitude assumptions
     */
    void set_offset_roll_pitch(float roll_deg, float pitch_deg) {
        roll_offset_deg = roll_deg;
        pitch_offset_deg = pitch_deg;
    }

    /**
     * @brief Enable or disable forward thrust-vectoring
     * 
     * @details Enables or disables forward thrust-vectoring capability. When disabled, forward
     *          motor tilts are not commanded and the vehicle must pitch to move forward like a
     *          traditional multicopter. Can be dynamically changed in flight via scripting to
     *          adapt to different flight requirements or vehicle configurations.
     *          
     *          With both forward and lateral thrust disabled, the vehicle behaves like a
     *          traditional multicopter (roll and pitch to move).
     * 
     * @param[in] b True to enable forward thrust-vectoring, false to disable
     * 
     * @note Default: enabled (true)
     * @note Can be changed in flight via scripting
     * @note With both forward_enable and lateral_enable false, vehicle acts as traditional copter
     */
    void set_forward_enable(bool b) {
        forward_enable = b;
    }

    /**
     * @brief Enable or disable lateral thrust-vectoring
     * 
     * @details Enables or disables lateral thrust-vectoring capability. When disabled, lateral
     *          motor tilts are not commanded and the vehicle must roll to move laterally like a
     *          traditional multicopter. Can be dynamically changed in flight via scripting to
     *          adapt to different flight requirements or vehicle configurations.
     *          
     *          Partial 6DoF configurations are possible: enable only forward thrust for forward-only
     *          vectoring, or enable only lateral thrust for sideways-only vectoring.
     * 
     * @param[in] b True to enable lateral thrust-vectoring, false to disable
     * 
     * @note Default: enabled (true)
     * @note Can be changed in flight via scripting
     * @note Allows partial 6DoF configurations (e.g., forward-only or lateral-only vectoring)
     */
    void set_lateral_enable(bool b) {
        lateral_enable = b;
    }

private:

    /**
     * @brief Compute and set forward and lateral motor tilts from desired attitude
     * 
     * @details Internal method that converts desired euler pitch and roll angles into forward
     *          and lateral motor tilt commands. Applies the configured attitude offsets and
     *          respects the forward_enable and lateral_enable flags. The computed tilts are
     *          sent to the motor library for thrust-vector control.
     *          
     *          This is the core 6DoF conversion function that decouples vehicle attitude from
     *          thrust vector direction.
     * 
     * @param[in,out] euler_pitch_angle_rad Desired pitch angle (rad), modified by offset
     * @param[in,out] euler_roll_angle_rad Desired roll angle (rad), modified by offset
     * 
     * @note Called internally by input methods to compute motor tilts
     * @note Respects forward_enable and lateral_enable flags
     * @note Applies roll_offset_deg and pitch_offset_deg
     */
    void set_forward_lateral_rad(float &euler_pitch_angle_rad, float &euler_roll_angle_rad);

    /**
     * @brief Roll attitude offset for 6DoF neutral position (degrees)
     * 
     * @details Defines the roll component of the vehicle's neutral attitude. This offset is
     *          applied when converting attitude commands to thrust-vector tilts, allowing the
     *          vehicle to maintain a rolled orientation while computing thrust vectors.
     */
    float roll_offset_deg;

    /**
     * @brief Pitch attitude offset for 6DoF neutral position (degrees)
     * 
     * @details Defines the pitch component of the vehicle's neutral attitude. This offset is
     *          applied when converting attitude commands to thrust-vector tilts, allowing the
     *          vehicle to maintain a pitched orientation while computing thrust vectors.
     */
    float pitch_offset_deg;

    /**
     * @brief Enable flag for forward thrust-vectoring
     * 
     * @details When true, forward motor tilts are commanded for forward/backward translation.
     *          When false, forward tilts are disabled and the vehicle must pitch to move forward.
     *          Default: true (forward thrust-vectoring enabled).
     */
    bool forward_enable = true;

    /**
     * @brief Enable flag for lateral thrust-vectoring
     * 
     * @details When true, lateral motor tilts are commanded for left/right translation.
     *          When false, lateral tilts are disabled and the vehicle must roll to move laterally.
     *          Default: true (lateral thrust-vectoring enabled).
     */
    bool lateral_enable = true;

    /**
     * @brief Singleton instance pointer for scripting API access
     * 
     * @details Static pointer to the single AC_AttitudeControl_Multi_6DoF instance. Used by
     *          AP_Scripting to provide Lua bindings for 6DoF control. Only one instance is
     *          permitted per vehicle (enforced by constructor panic).
     * 
     * @note Singleton pattern enforced - only one instance allowed
     * @warning Constructor panics if multiple instances are created
     */
    static AC_AttitudeControl_Multi_6DoF *_singleton;

};

#endif // AP_SCRIPTING_ENABLED
