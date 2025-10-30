/**
 * @file AC_PosControl.h
 * @brief Position controller for multicopter navigation and waypoint following
 * 
 * @details Implements jerk-limited 3D position control with separate horizontal (NE) and vertical (U) control loops.
 *          Converts position targets to velocity targets (P control), velocity targets to acceleration targets 
 *          (PID control), and acceleration targets to attitude/throttle commands. Uses kinematic trajectory shaping 
 *          to generate smooth, dynamically-feasible paths.
 *          
 *          Control Architecture:
 *          - Position → Velocity: AC_P_2D (horizontal) and AC_P_1D (vertical) P controllers
 *          - Velocity → Acceleration: AC_PID_2D (horizontal) and AC_PID (vertical) PID controllers
 *          - Acceleration → Attitude: Converts NE acceleration to lean angles via AC_AttitudeControl
 *          - Acceleration → Throttle: Converts U acceleration to throttle via feed-forward and PID
 *          
 *          Trajectory Shaping: S-curve profiles with jerk, acceleration, and velocity limits
 *          Terrain Following: Adjusts horizontal speed to maintain vertical buffer
 *          EKF Reset Handling: Compensates for discontinuous position/velocity updates
 *          
 *          Coordinate Frame: NED (North-East-Down) earth frame for position, body frame via attitude controller
 *          Units: positions in meters or centimeters, velocities in m/s or cm/s, accelerations in m/s² or cm/s²
 * 
 * @note Position controller runs at main loop rate (typically 400Hz)
 * @warning Down is positive in NED frame (opposite intuition)
 * 
 * @see AC_AttitudeControl - Receives lean angle commands from position controller
 * @see AP_AHRS - Provides position and velocity estimates
 * @see AP_Motors - Motor output for throttle commands
 * 
 * Source: libraries/AC_AttitudeControl/AC_PosControl.cpp
 */

#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AC_PID/AC_P.h>            // P library
#include <AC_PID/AC_PID.h>          // PID library
#include <AC_PID/AC_P_1D.h>         // P library (1-axis)
#include <AC_PID/AC_P_2D.h>         // P library (2-axis)
#include <AC_PID/AC_PI_2D.h>        // PI library (2-axis)
#include <AC_PID/AC_PID_Basic.h>    // PID library (1-axis)
#include <AC_PID/AC_PID_2D.h>       // PID library (2-axis)
#include <AP_Scripting/AP_Scripting_config.h>
#include "AC_AttitudeControl.h"     // Attitude control library

#include <AP_Logger/LogStructure.h>

/**
 * Position controller default parameter definitions.
 * These values provide conservative defaults suitable for most multirotors.
 * Vehicle-specific tuning should be done via parameters, not by changing these defaults.
 */

// Horizontal (NE) trajectory shaping parameters
#define POSCONTROL_ACCEL_NE_MSS                 1.0f    ///< Default horizontal acceleration limit in m/s². Overwritten by waypoint and loiter controllers for smoother missions
#define POSCONTROL_JERK_NE_MSSS                 5.0f    ///< Default horizontal jerk limit in m/s³. Controls smoothness of acceleration changes (S-curve shaping)

// Vertical stopping distance limits
#define POSCONTROL_STOPPING_DIST_UP_MAX_M       3.0f    ///< Maximum stopping distance in meters while climbing. Used to prevent excessive altitude overshoot
#define POSCONTROL_STOPPING_DIST_DOWN_MAX_M     2.0f    ///< Maximum stopping distance in meters while descending. Tighter limit for safety near ground

// Speed limits for trajectory generation
#define POSCONTROL_SPEED_MS                     5.0f    ///< Default horizontal speed in m/s. Conservative default suitable for most multirotors
#define POSCONTROL_SPEED_DOWN_MS                -1.5f   ///< Default descent rate in m/s (negative = downward). Limited for safe landing approaches
#define POSCONTROL_SPEED_UP_MS                  2.5f    ///< Default climb rate in m/s. Balanced between performance and efficiency

// Vertical (U) trajectory shaping parameters
#define POSCONTROL_ACCEL_U_MSS                  2.5f    ///< Default vertical acceleration limit in m/s². Higher than horizontal for responsive altitude control
#define POSCONTROL_JERK_U_MSSS                  5.0f    ///< Default vertical jerk limit in m/s³. Matches horizontal for consistent feel

// PID tuning parameters
#define POSCONTROL_THROTTLE_CUTOFF_FREQ_HZ      2.0f    ///< Low-pass filter cutoff frequency for throttle acceleration error in Hz. Reduces high-frequency noise
#define POSCONTROL_OVERSPEED_GAIN_U             2.0f    ///< Gain controlling rate at which vertical speed is brought back within limits. Higher = more aggressive correction
#define POSCONTROL_RELAX_TC                     0.16f   ///< Integrator relax time constant in seconds. Decays I-term to 5% in 0.5s to prevent windup during mode transitions

/**
 * @class AC_PosControl
 * @brief Multicopter 3D position controller with jerk-limited trajectory shaping
 * 
 * @details This class implements a multi-stage cascaded control architecture for precise 3D position control:
 *          
 *          **Control Stages:**
 *          - Position → Velocity: AC_P_2D (horizontal) and AC_P_1D (vertical) P controllers
 *          - Velocity → Acceleration: AC_PID_2D (horizontal) and AC_PID (vertical) PID controllers
 *          - Acceleration → Attitude: Converts NE acceleration to lean angles via AC_AttitudeControl
 *          - Acceleration → Throttle: Converts U acceleration to throttle via feed-forward and PID
 *          
 *          **Trajectory Shaping:**
 *          S-curve profiles with configurable jerk, acceleration, and velocity limits ensure smooth,
 *          dynamically-feasible paths. Jerk limiting produces gradual acceleration changes that reduce
 *          mechanical stress and improve passenger comfort.
 *          
 *          **Coordinate Frame:**
 *          NED (North-East-Down) earth frame for position relative to EKF origin.
 *          Positive Z is DOWN (standard aviation convention).
 *          
 *          **Terrain Following:**
 *          Adjusts horizontal speed to maintain vertical terrain buffer. Slows horizontal motion
 *          when vertical clearance is insufficient.
 *          
 *          **EKF Reset Handling:**
 *          Compensates for discontinuous position/velocity updates when EKF resets due to GPS glitches
 *          or sensor failures. Prevents control transients during resets.
 *          
 *          **Unit Conventions:**
 *          - Positions: meters (m) or centimeters (cm)
 *          - Velocities: m/s or cm/s
 *          - Accelerations: m/s² or cm/s²
 *          - Angles: radians (rad) or centidegrees (cd)
 *          - Always check function name suffix for units
 *          
 *          **Typical Usage Pattern:**
 *          1. Initialize controller: init_NE_controller(), init_U_controller()
 *          2. Set limits: set_max_speed_accel_NE_m(), set_max_speed_accel_U_m()
 *          3. Set target: input_pos_NEU_m() or input_vel_accel_NE_m()
 *          4. Update estimates: update_estimates() (called every loop)
 *          5. Run controller: update_NE_controller(), update_U_controller()
 *          6. Retrieve outputs: get_roll_cd(), get_pitch_cd(), get_yaw_cd()
 *          7. Apply to attitude controller
 * 
 * @note Position controller typically runs at main loop rate (400Hz for most multirotors)
 * @note Separate NE and U controllers allow independent tuning for horizontal and vertical performance
 * @note Stopping point calculations account for current velocity and configured deceleration limits
 * @note Parameters registered via var_info[] in AC_PosControl.cpp (PSC_* parameters)
 * 
 * @warning Maximum lean angle limits altitude hold capability due to cosine loss of vertical thrust
 * @warning Jerk limits too low cause sluggish response; too high cause oscillations
 * @warning Speed limits must be achievable with configured acceleration and lean angle constraints
 * @warning Vertical speed limits affect terrain following safety margins
 * @warning Thread-safety: Vehicle code uses WITH_SEMAPHORE for multi-threaded access
 * @warning Units: Down is positive in NED frame (opposite intuition)
 * @warning dt parameter in set_dt_s() must match actual controller update rate
 * 
 * @see AC_AttitudeControl - Receives lean angle commands from position controller
 * @see AC_P_1D, AC_P_2D - Position P controllers with square root limiting
 * @see AC_PID, AC_PID_2D - Velocity PID controllers
 * @see AP_AHRS - Provides position and velocity estimates from EKF
 * @see AP_Motors - Motor output interface for throttle commands
 * @see ArduCopter navigation modes - Loiter, PosHold, Auto, Guided use position controller
 * 
 * Source: libraries/AC_AttitudeControl/AC_PosControl.cpp
 */
class AC_PosControl
{
public:

    /**
     * @brief Construct position controller
     * 
     * @param[in] ahrs Reference to attitude/heading reference system providing position and velocity estimates
     * @param[in] motors Reference to motor output interface for thrust commands
     * @param[in] attitude_control Reference to attitude controller that will receive lean angle commands
     * 
     * @details Initializes PID controllers and trajectory shaping parameters to default values.
     *          Controller must be further initialized with init_NE_controller() and init_U_controller()
     *          before use. Default parameters provide conservative tuning suitable for most multirotors.
     * 
     * @note Constructor does not activate controller. Call init methods before first use.
     * @see init_NE_controller(), init_U_controller()
     */
    AC_PosControl(AP_AHRS_View& ahrs, const class AP_Motors& motors, AC_AttitudeControl& attitude_control);

    // do not allow copying
    CLASS_NO_COPY(AC_PosControl);

    /**
     * @brief Set controller timestep in seconds
     * 
     * @param[in] dt Timestep between controller updates in seconds
     * 
     * @details Sets the time interval used for integration and differentiation in the controller.
     *          This should match the actual rate at which update_NE_controller() and update_U_controller()
     *          are called. Typical values: 0.0025s (400Hz), 0.004s (250Hz), 0.01s (100Hz).
     * 
     * @warning Must match actual update rate or control will be unstable. Too large causes sluggish
     *          response and integrator issues. Too small causes excessive gain and oscillations.
     * 
     * @note Called automatically by vehicle code at initialization
     * @see get_dt_s()
     */
    void set_dt_s(float dt) { _dt_s = dt; }

    /**
     * @brief Get controller timestep in seconds
     * 
     * @return Timestep between controller updates in seconds
     * 
     * @details Returns the configured update interval. Used for diagnostic purposes and
     *          by trajectory generators to calculate time-based profiles.
     * 
     * @see set_dt_s()
     */
    float get_dt_s() const { return _dt_s; }

    /**
     * @brief Update position and velocity estimates from AHRS
     * 
     * @param[in] high_vibes If true, force vertical-only velocity fallback due to vibration
     * 
     * @details Updates internal _pos_estimate_neu_m and _vel_estimate_neu_ms from EKF.
     *          Falls back to vertical-only data if horizontal velocity or position is invalid.
     *          Handles EKF resets transparently by detecting and compensating for discontinuities.
     *          
     *          Vibration Compensation:
     *          When high_vibes is true or vibration levels exceed threshold, disables use of
     *          horizontal velocity estimates to prevent control oscillations. Continues using
     *          position data but with degraded velocity feedback.
     * 
     * @note Must be called every control loop before running position controller
     * @note Automatically detects and handles EKF position/velocity resets
     * @warning High vibration mode degrades horizontal velocity control performance
     * 
     * @see handle_ekf_NE_reset(), handle_ekf_U_reset()
     * 
     * Source: libraries/AC_AttitudeControl/AC_PosControl.cpp:100-180
     */
    void update_estimates(bool high_vibes = false);

    // Returns the jerk limit for horizontal path shaping in cm/s³.
    // See get_shaping_jerk_NE_msss() for full details.
    float get_shaping_jerk_NE_cmsss() const { return get_shaping_jerk_NE_msss() * 100.0; }

    // Returns the jerk limit for horizontal path shaping in m/s³.
    // Used to constrain acceleration changes in trajectory generation.
    float get_shaping_jerk_NE_msss() const { return _shaping_jerk_ne_msss; }


    ///
    /// 3D position shaper
    ///

    /**
     * @brief Set 3D position target with jerk-limited trajectory (centimeters)
     * 
     * @param[in] pos_neu_cm Position target in NED frame in centimeters relative to EKF origin
     * @param[in] pos_terrain_target_alt_cm Terrain altitude at target location in centimeters
     * @param[in] terrain_buffer_cm Required vertical clearance above terrain in centimeters
     * 
     * @details Wrapper for input_pos_NEU_m() with unit conversion from centimeters to meters.
     * 
     * @see input_pos_NEU_m()
     */
    void input_pos_NEU_cm(const Vector3p& pos_neu_cm, float pos_terrain_target_alt_cm, float terrain_buffer_cm);

    /**
     * @brief Set 3D position target with jerk-limited trajectory (meters)
     * 
     * @param[in] pos_neu_m Position target in NED frame in meters relative to EKF origin
     * @param[in] pos_terrain_target_alt_m Terrain altitude at target location in meters
     * @param[in] terrain_buffer_m Required vertical clearance above terrain in meters
     * 
     * @details Computes S-curve trajectory from current position to target position respecting
     *          jerk, acceleration, and velocity limits. Updates internal position, velocity, and
     *          acceleration targets for both horizontal (NE) and vertical (U) axes.
     *          
     *          Terrain Following:
     *          Uses terrain altitude and buffer to adjust horizontal speed when vertical clearance
     *          is insufficient. Slows horizontal motion to allow vertical controller to climb and
     *          maintain safe terrain separation.
     *          
     *          Trajectory Algorithm:
     *          1. Calculate position error vector
     *          2. Generate S-curve velocity profile with jerk limiting
     *          3. Compute acceleration from kinematic constraints
     *          4. Apply terrain-following speed scaling
     *          5. Update internal targets
     * 
     * @note Position is in NED frame: North=+X, East=+Y, Down=+Z (positive down!)
     * @note Terrain altitude and buffer enable safe terrain following in Auto/Guided modes
     * @warning Target position must be achievable with current speed and acceleration limits
     * 
     * @see set_max_speed_accel_NE_m(), set_max_speed_accel_U_m()
     * @see pos_terrain_U_scaler_m()
     * 
     * Source: libraries/AC_AttitudeControl/AC_PosControl.cpp:200-350
     */
    void input_pos_NEU_m(const Vector3p& pos_neu_m, float pos_terrain_target_alt_m, float terrain_buffer_m);

    // Returns a scaling factor for horizontal velocity in cm/s to respect vertical terrain buffer.
    // See pos_terrain_U_scaler_m() for full details.
    float pos_terrain_U_scaler_cm(float pos_terrain_u_cm, float pos_terrain_u_buffer_cm) const;

    // Returns a scaling factor for horizontal velocity in m/s to ensure
    // the vertical controller maintains a safe distance above terrain.
    float pos_terrain_U_scaler_m(float pos_terrain_u_m, float pos_terrain_u_buffer_m) const;

    ///
    /// Lateral position controller
    ///

    /**
     * @brief Set horizontal speed and acceleration limits (centimeters)
     * 
     * @param[in] speed_cms Maximum horizontal speed in cm/s
     * @param[in] accel_cmss Maximum horizontal acceleration in cm/s²
     * 
     * @details Wrapper for set_max_speed_accel_NE_m() with unit conversion.
     * 
     * @see set_max_speed_accel_NE_m()
     */
    void set_max_speed_accel_NE_cm(float speed_cms, float accel_cmss);

    /**
     * @brief Set horizontal speed and acceleration limits (meters)
     * 
     * @param[in] speed_ms Maximum horizontal speed in m/s
     * @param[in] accel_mss Maximum horizontal acceleration in m/s²
     * 
     * @details Sets the velocity and acceleration limits used for trajectory shaping in the
     *          horizontal (NE) plane. These values constrain the kinematic path generated by
     *          input_pos_NEU_m() and related methods.
     *          
     *          Can be called at any time; transitions to new limits are handled smoothly through
     *          jerk limiting. Common use: reduce speed for precision maneuvers, increase speed
     *          for transit between waypoints.
     *          
     *          Relationship to Lean Angle:
     *          Maximum achievable acceleration is limited by:
     *          accel_max = g * tan(lean_angle_max)
     *          
     *          For 30° lean: accel_max ≈ 5.7 m/s²
     *          For 45° lean: accel_max ≈ 9.8 m/s²
     * 
     * @note Speed and acceleration must be positive
     * @note Limits apply to trajectory generation, not instantaneous control
     * @warning Acceleration limit must be achievable with configured lean angle
     * @warning Too-low limits cause sluggish response; too-high limits may be unachievable
     * 
     * @see set_lean_angle_max_cd()
     * @see get_max_speed_NE_ms(), get_max_accel_NE_mss()
     */
    void set_max_speed_accel_NE_m(float speed_ms, float accel_mss);

    /**
     * @brief Set horizontal PID correction limits (centimeters)
     * 
     * @param[in] speed_cms Maximum correction velocity in cm/s
     * @param[in] accel_cmss Maximum correction acceleration in cm/s²
     * 
     * @details Wrapper for set_correction_speed_accel_NE_m() with unit conversion.
     * 
     * @see set_correction_speed_accel_NE_m()
     */
    void set_correction_speed_accel_NE_cm(float speed_cms, float accel_cmss);

    /**
     * @brief Set horizontal PID correction limits (meters)
     * 
     * @param[in] speed_ms Maximum correction velocity in m/s
     * @param[in] accel_mss Maximum correction acceleration in m/s²
     * 
     * @details Sets limits on the PID controller's correction output, not the desired trajectory.
     *          Correction limits constrain how aggressively the controller responds to position
     *          and velocity errors.
     *          
     *          Distinction from Trajectory Limits:
     *          - Trajectory limits (set_max_speed_accel_NE_m): constrain desired path
     *          - Correction limits (this function): constrain error correction
     *          
     *          Lower correction limits provide smoother but slower error recovery.
     *          Higher correction limits provide faster but potentially more aggressive response.
     * 
     * @note Should only be called during initialization to avoid control discontinuities
     * @warning Changing during flight can cause abrupt control changes
     * 
     * @see set_max_speed_accel_NE_m()
     */
    void set_correction_speed_accel_NE_m(float speed_ms, float accel_mss);

    /**
     * @brief Get maximum horizontal speed (centimeters)
     * 
     * @return Maximum horizontal speed in cm/s
     * 
     * @details Wrapper for get_max_speed_NE_ms() with unit conversion to centimeters.
     * 
     * @see get_max_speed_NE_ms(), set_max_speed_accel_NE_cm()
     */
    float get_max_speed_NE_cms() const { return get_max_speed_NE_ms() * 100.0; }

    /**
     * @brief Get maximum horizontal speed (meters)
     * 
     * @return Maximum horizontal speed in m/s used for trajectory shaping
     * 
     * @details Returns the current horizontal speed limit used by the trajectory generator.
     *          This value constrains the kinematic path planning.
     * 
     * @see set_max_speed_accel_NE_m()
     */
    float get_max_speed_NE_ms() const { return _vel_max_ne_ms; }

    /**
     * @brief Get maximum horizontal acceleration (centimeters)
     * 
     * @return Maximum horizontal acceleration in cm/s²
     * 
     * @details Wrapper for get_max_accel_NE_mss() with unit conversion to centimeters.
     * 
     * @see get_max_accel_NE_mss(), set_max_speed_accel_NE_cm()
     */
    float get_max_accel_NE_cmss() const { return get_max_accel_NE_mss() * 100.0; }

    /**
     * @brief Get maximum horizontal acceleration (meters)
     * 
     * @return Maximum horizontal acceleration in m/s² used for trajectory shaping
     * 
     * @details Returns the current horizontal acceleration limit used by the trajectory generator.
     *          This value constrains the kinematic path planning and must be achievable with
     *          the configured lean angle limit.
     * 
     * @see set_max_speed_accel_NE_m(), get_lean_angle_max_rad()
     */
    float get_max_accel_NE_mss() const { return _accel_max_ne_mss; }

    /**
     * @brief Set maximum horizontal position error (centimeters)
     * 
     * @param[in] error_max_cm Maximum position error in cm
     * 
     * @details Wrapper for set_pos_error_max_NE_m() with unit conversion.
     * 
     * @see set_pos_error_max_NE_m()
     */
    void set_pos_error_max_NE_cm(float error_max_cm) { set_pos_error_max_NE_m(error_max_cm * 0.01); }

    /**
     * @brief Set maximum horizontal position error (meters)
     * 
     * @param[in] error_max_m Maximum position error in meters
     * 
     * @details Constrains the output of the horizontal position P controller by limiting
     *          how much velocity the controller can request based on position error.
     *          
     *          Limits the maximum velocity command = P_gain × position_error.
     *          
     *          Smaller values: More conservative positioning, slower approach to target
     *          Larger values: More aggressive positioning, faster approach to target
     * 
     * @note Typical values: 1-3 meters for standard operation
     * @warning Too large values can cause overshoot and oscillation
     * 
     * @see get_pos_error_max_NE_m(), _p_pos_ne_m
     */
    void set_pos_error_max_NE_m(float error_max_m) { _p_pos_ne_m.set_error_max(error_max_m); }

    /**
     * @brief Get maximum horizontal position error (centimeters)
     * 
     * @return Maximum position error in cm
     * 
     * @details Wrapper for get_pos_error_max_NE_m() with unit conversion.
     * 
     * @see get_pos_error_max_NE_m()
     */
    float get_pos_error_max_NE_cm() const { return get_pos_error_max_NE_m() * 100.0; }

    /**
     * @brief Get maximum horizontal position error (meters)
     * 
     * @return Maximum position error in meters
     * 
     * @details Returns the current position error limit that constrains the P controller output.
     * 
     * @see set_pos_error_max_NE_m()
     */
    float get_pos_error_max_NE_m() const { return _p_pos_ne_m.get_error_max(); }

    /**
     * @brief Initialize NE controller to a stationary stopping point
     * 
     * @details Initializes the horizontal position controller to a stopping point with zero
     *          velocity and acceleration. The stopping point position is calculated from
     *          current velocity and acceleration using kinematic equations.
     *          
     *          Use Case: When transitioning to position control from a velocity-controlled
     *          state and the starting target position is not yet known.
     *          
     *          The computed starting position can be retrieved with get_pos_target_NEU_m().
     * 
     * @note This calculates where the vehicle would naturally stop given current motion
     * @see init_NE_controller(), get_pos_target_NEU_m()
     */
    void init_NE_controller_stopping_point();

    /**
     * @brief Relax NE acceleration to zero over time
     * 
     * @details Smoothly decays the horizontal acceleration command to zero while maintaining
     *          current velocity and position targets. Reduces acceleration by approximately
     *          95% over 0.5 seconds using exponential decay.
     *          
     *          Purpose: Avoid abrupt transitions when switching control modes or when the
     *          vehicle should coast to maintain current velocity without acceleration.
     *          
     *          Decay time constant: POSCONTROL_RELAX_TC (0.16 seconds)
     * 
     * @note Velocity and position targets remain unchanged
     * @see POSCONTROL_RELAX_TC
     */
    void relax_velocity_controller_NE();

    /**
     * @brief Soften NE controller for landing
     * 
     * @details Modifies controller behavior for stable landing by:
     *          - Reducing position error limits to prevent aggressive corrections
     *          - Suppressing I-term windup in the velocity controller
     *          - Making horizontal drift correction less aggressive near ground
     *          
     *          Used during final descent phase to prioritize smooth touchdown over
     *          precise positioning.
     * 
     * @note Call continuously during landing descent
     * @warning Not suitable for normal flight - reduces position hold accuracy
     */
    void soften_for_landing_NE();

    /**
     * @brief Fully initialize NE controller with current state
     * 
     * @details Initializes the horizontal position controller with current position, velocity,
     *          acceleration from AHRS, and current attitude from attitude controller. This is
     *          the standard initialization used when entering position control modes.
     *          
     *          Sets:
     *          - Position target = current position
     *          - Velocity target = current velocity  
     *          - Acceleration target = current acceleration
     *          - Clears integrator terms
     *          - Resets EKF change detection
     * 
     * @note Private function shared by other NE initialization methods
     * @see init_NE_controller_stopping_point()
     */
    void init_NE_controller();

    /**
     * @brief Set desired NE acceleration with jerk limiting (centimeters)
     * 
     * @param[in] accel_neu_cmsss Desired acceleration in NEU frame (cm/s²)
     * 
     * @details Wrapper for input_accel_NE_m() with unit conversion.
     * 
     * @see input_accel_NE_m()
     */
    void input_accel_NE_cm(const Vector3f& accel_neu_cmsss);

    /**
     * @brief Set desired NE acceleration with jerk limiting (meters)
     * 
     * @param[in] accel_neu_msss Desired acceleration in NEU frame (m/s²)
     * 
     * @details Sets the desired horizontal acceleration target and computes a jerk-limited
     *          trajectory from the current kinematic state to the target acceleration.
     *          
     *          Trajectory Generation:
     *          - Smoothly transitions from current to desired acceleration
     *          - Respects maximum jerk limit (rate of change of acceleration)
     *          - Produces S-curve acceleration profile
     *          
     *          Constraints:
     *          - Maximum jerk: _shaping_jerk_ne_msss
     *          - Maximum acceleration: _accel_max_ne_mss
     * 
     * @note Typically used for external path planners providing acceleration commands
     * @see set_max_speed_accel_NE_m(), get_shaping_jerk_NE_msss()
     */
    void input_accel_NE_m(const Vector3f& accel_neu_msss);

    /**
     * @brief Set desired NE velocity and acceleration with jerk limiting (centimeters)
     * 
     * @param[in,out] vel_ne_cms Desired velocity in NE plane (cm/s), updated with shaped value
     * @param[in] accel_ne_cmss Desired acceleration in NE plane (cm/s²)
     * @param[in] limit_output If true, apply limits to total command (desired + correction)
     * 
     * @details Wrapper for input_vel_accel_NE_m() with unit conversion.
     * 
     * @see input_vel_accel_NE_m()
     */
    void input_vel_accel_NE_cm(Vector2f& vel_ne_cms, const Vector2f& accel_ne_cmss, bool limit_output = true);

    /**
     * @brief Set desired NE velocity and acceleration with jerk limiting (meters)
     * 
     * @param[in,out] vel_ne_ms Desired velocity in NE plane (m/s), updated with shaped value
     * @param[in] accel_ne_mss Desired acceleration in NE plane (m/s²)
     * @param[in] limit_output If true, apply limits to total command (desired + correction)
     * 
     * @details Sets the desired horizontal velocity and acceleration, computing a kinematic
     *          trajectory that smoothly transitions to these targets.
     *          
     *          Trajectory Shaping:
     *          - Computes jerk-limited path from current to desired velocity
     *          - Updates acceleration target to achieve smooth velocity change
     *          - Velocity parameter is updated to reflect achievable shaped velocity
     *          
     *          Limit Output Option:
     *          - true: Limits apply to combined (feedforward + feedback) command
     *          - false: Only feedforward path is shaped, feedback unlimited
     * 
     * @note vel_ne_ms is modified to contain the achievable shaped velocity
     * @see input_accel_NE_m(), update_NE_controller()
     */
    void input_vel_accel_NE_m(Vector2f& vel_ne_ms, const Vector2f& accel_ne_mss, bool limit_output = true);

    /**
     * @brief Set desired NE position, velocity, and acceleration (centimeters)
     * 
     * @param[in,out] pos_ne_cm Desired position in NE plane (cm), updated with shaped value
     * @param[in,out] vel_ne_cms Desired velocity in NE plane (cm/s), updated with shaped value
     * @param[in] accel_ne_cmss Desired acceleration in NE plane (cm/s²)
     * @param[in] limit_output If true, apply limits to total command (desired + correction)
     * 
     * @details Wrapper for input_pos_vel_accel_NE_m() with unit conversion.
     * 
     * @see input_pos_vel_accel_NE_m()
     */
    void input_pos_vel_accel_NE_cm(Vector2p& pos_ne_cm, Vector2f& vel_ne_cms, const Vector2f& accel_ne_cmss, bool limit_output = true);

    /**
     * @brief Set desired NE position, velocity, and acceleration (meters)
     * 
     * @param[in,out] pos_ne_m Desired position in NE plane (m), updated with shaped value
     * @param[in,out] vel_ne_ms Desired velocity in NE plane (m/s), updated with shaped value
     * @param[in] accel_ne_mss Desired acceleration in NE plane (m/s²)
     * @param[in] limit_output If true, apply limits to total command (desired + correction)
     * 
     * @details Sets the full desired horizontal kinematic state (position, velocity, acceleration)
     *          and computes a jerk-limited trajectory to these targets.
     *          
     *          Trajectory Shaping:
     *          - Computes S-curve path from current to desired state
     *          - Respects maximum jerk, acceleration, and velocity limits
     *          - Position and velocity parameters updated with achievable shaped values
     *          
     *          Use Case: External path planners providing full kinematic trajectory
     *          
     *          Limit Output:
     *          - true: Total output (desired + PID correction) is limited
     *          - false: Only desired trajectory is shaped, PID correction unlimited
     * 
     * @note pos_ne_m and vel_ne_ms are modified to contain achievable shaped values
     * @warning Ensure external planner provides dynamically feasible trajectories
     * @see input_vel_accel_NE_m(), update_NE_controller()
     */
    void input_pos_vel_accel_NE_m(Vector2p& pos_ne_m, Vector2f& vel_ne_ms, const Vector2f& accel_ne_mss, bool limit_output = true);

    /**
     * @brief Check if NE controller is actively running
     * 
     * @return true if controller has run in the last 5 loop cycles, false otherwise
     * 
     * @details Checks if update_NE_controller() has been called recently (within last 5 cycles).
     *          Used to detect if horizontal position control is currently active.
     *          
     *          Timeout: 5 × loop period (typically 12.5ms at 400Hz = 62.5ms total)
     * 
     * @note Returns false if controller hasn't been updated, indicating inactive control
     * @see update_NE_controller()
     */
    bool is_active_NE() const;

    /**
     * @brief Disable NE position stabilization
     * 
     * @details Disables horizontal position correction by setting the position target equal
     *          to the current estimated position. Velocity control remains active.
     *          
     *          Effect:
     *          - Position error → 0 (no position correction)
     *          - Velocity control still active (can track velocity commands)
     *          - Useful for switching from position hold to velocity control
     * 
     * @note Does not affect velocity controller
     * @see stop_vel_NE_stabilisation()
     */
    void stop_pos_NE_stabilisation();

    /**
     * @brief Disable NE position and velocity stabilization
     * 
     * @details Disables both horizontal position and velocity correction by setting targets
     *          equal to current state. Freezes all horizontal stabilization.
     *          
     *          Effect:
     *          - Position target = current position (no position correction)
     *          - Velocity target = current velocity (no velocity correction)
     *          - Only feedforward acceleration commands will be executed
     *          - Useful for manual/acro modes or when disabling stabilization
     * 
     * @note Vehicle will drift with wind/disturbances when stabilization disabled
     * @see stop_pos_NE_stabilisation()
     */
    void stop_vel_NE_stabilisation();

    /**
     * @brief Set NE control authority scale factor
     * 
     * @param[in] ne_control_scale_factor Scaling factor (0.0 to 1.0)
     * 
     * @details Applies a scalar multiplier to the horizontal control loop output.
     *          Reduces control authority proportionally.
     *          
     *          Values:
     *          - 0.0: Disables lateral control completely
     *          - 0.5: Reduces control authority by 50%
     *          - 1.0: Full control authority (normal operation)
     *          
     *          Use Cases:
     *          - Smooth transition between control modes
     *          - Blending manual and automatic control
     *          - Temporary reduction of control authority
     * 
     * @note Applied to acceleration commands in update_NE_controller()
     * @see update_NE_controller()
     */
    void set_NE_control_scale_factor(float ne_control_scale_factor) {
        _ne_control_scale_factor = ne_control_scale_factor;
    }

    /**
     * @brief Run the horizontal position controller
     * 
     * @details Executes the complete NE-axis position control cascade:
     *          
     *          Control Stages:
     *          1. Position → Velocity: P controller (_p_pos_ne_m)
     *             position_error → velocity_correction
     *          
     *          2. Velocity → Acceleration: PID controller (_pid_vel_ne_cm)
     *             velocity_error → acceleration_correction
     *          
     *          3. Combine: acceleration_desired + acceleration_correction → acceleration_target
     *          
     *          4. Acceleration → Attitude: Convert to roll/pitch lean angles
     *             Limited by maximum lean angle constraint
     *          
     *          Requirements:
     *          - Position/velocity/acceleration targets must be set via input_* methods
     *          - Typically called at 400Hz (main loop rate)
     *          - AC_AttitudeControl receives resulting roll/pitch commands
     * 
     * @note Call after setting targets with input_pos_vel_accel_NE_m() or similar
     * @warning Must be called regularly to maintain position control
     * 
     * @see input_pos_vel_accel_NE_m(), accel_NE_mss_to_lean_angles_rad()
     * 
     * Source: libraries/AC_AttitudeControl/AC_PosControl.cpp:800-950
     */
    void update_NE_controller();

    ///
    /// Vertical position controller
    ///

    /**
     * @brief Set vertical speed and acceleration limits (centimeters)
     * 
     * @param[in] speed_down_cms Maximum descent rate in cm/s (positive or negative)
     * @param[in] speed_up_cms Maximum climb rate in cm/s (positive)
     * @param[in] accel_cmss Maximum vertical acceleration in cm/s²
     * 
     * @details Wrapper for set_max_speed_accel_U_m() with unit conversion.
     *          Descent rate is always interpreted as downward regardless of sign.
     * 
     * @see set_max_speed_accel_U_m()
     */
    void set_max_speed_accel_U_cm(float speed_down_cms, float speed_up_cms, float accel_cmss);

    /**
     * @brief Set vertical speed and acceleration limits (meters)
     * 
     * @param[in] speed_down_ms Maximum descent rate in m/s (positive or negative)
     * @param[in] speed_up_ms Maximum climb rate in m/s (positive)
     * @param[in] accel_mss Maximum vertical acceleration in m/s²
     * 
     * @details Sets the vertical speed and acceleration limits used for jerk-limited kinematic
     *          trajectory shaping. These values constrain the desired vertical path.
     *          
     *          Speed Parameters:
     *          - speed_down_ms: Descent rate (positive or negative, interpreted as down)
     *          - speed_up_ms: Climb rate (positive = upward)
     *          
     *          Acceleration Constraint:
     *          - Must be achievable with available thrust margin
     *          - Affects climb/descent responsiveness
     *          
     *          Typical Values:
     *          - Descent: -1.5 m/s (POSCONTROL_SPEED_DOWN_MS)
     *          - Climb: 2.5 m/s (POSCONTROL_SPEED_UP_MS)
     *          - Accel: 2.5 m/s² (POSCONTROL_ACCEL_U_MSS)
     * 
     * @note Descent rate sign is normalized internally
     * @warning Speed limits must be achievable with configured thrust
     * 
     * @see get_max_speed_up_ms(), get_max_speed_down_ms(), get_max_accel_U_mss()
     */
    void set_max_speed_accel_U_m(float speed_down_ms, float speed_up_ms, float accel_mss);

    /**
     * @brief Set vertical PID correction limits (centimeters)
     * 
     * @param[in] speed_down_cms Maximum correction descent rate in cm/s
     * @param[in] speed_up_cms Maximum correction climb rate in cm/s
     * @param[in] accel_cmss Maximum correction acceleration in cm/s²
     * 
     * @details Wrapper for set_correction_speed_accel_U_mss() with unit conversion.
     * 
     * @note Should only be called during initialization
     * @see set_correction_speed_accel_U_mss()
     */
    void set_correction_speed_accel_U_cmss(float speed_down_cms, float speed_up_cms, float accel_cmss);

    /**
     * @brief Set vertical PID correction limits (meters)
     * 
     * @param[in] speed_down_ms Maximum correction descent rate in m/s
     * @param[in] speed_up_ms Maximum correction climb rate in m/s
     * @param[in] accel_mss Maximum correction acceleration in m/s²
     * 
     * @details Sets limits on the vertical PID controller's correction output, constraining
     *          how aggressively the controller responds to altitude and climb rate errors.
     *          
     *          Distinction from Trajectory Limits:
     *          - Trajectory limits (set_max_speed_accel_U_m): constrain desired path
     *          - Correction limits (this function): constrain PID error correction
     *          
     *          Lower correction limits:
     *          - Smoother altitude corrections
     *          - Slower error recovery
     *          - More stable near target altitude
     *          
     *          Higher correction limits:
     *          - Faster altitude corrections
     *          - More aggressive response to errors
     *          - Potential for overshoot
     * 
     * @note Should only be called during initialization to avoid control discontinuities
     * @warning Changing during flight can cause abrupt altitude changes
     * 
     * @see set_max_speed_accel_U_m()
     */
    void set_correction_speed_accel_U_mss(float speed_down_ms, float speed_up_ms, float accel_mss);

    /**
     * @brief Get maximum vertical acceleration (centimeters)
     * 
     * @return Maximum vertical acceleration in cm/s²
     * 
     * @details Wrapper for get_max_accel_U_mss() with unit conversion.
     * 
     * @see get_max_accel_U_mss(), set_max_speed_accel_U_cm()
     */
    float get_max_accel_U_cmss() const { return get_max_accel_U_mss() * 100.0; }

    /**
     * @brief Get maximum vertical acceleration (meters)
     * 
     * @return Maximum vertical acceleration in m/s² used for trajectory shaping
     * 
     * @details Returns the current vertical acceleration limit used by the trajectory generator.
     *          This value constrains climb and descent acceleration.
     * 
     * @see set_max_speed_accel_U_m()
     */
    float get_max_accel_U_mss() const { return _accel_max_u_mss; }

    /**
     * @brief Get maximum upward position error limit (centimeters)
     * 
     * @return Maximum positive (upward) position error in cm
     * 
     * @details Wrapper for get_pos_error_up_m() with unit conversion.
     * 
     * @see get_pos_error_up_m()
     */
    float get_pos_error_up_cm() const { return get_pos_error_up_m() * 100.0; }

    /**
     * @brief Get maximum upward position error limit (meters)
     * 
     * @return Maximum positive (upward) position error in meters
     * 
     * @details Returns the maximum allowed positive altitude error that constrains the
     *          vertical position P controller output. Limits climb rate command when
     *          vehicle is below target altitude.
     * 
     * @see _p_pos_u_m
     */
    float get_pos_error_up_m() const { return _p_pos_u_m.get_error_max(); }

    /**
     * @brief Get maximum downward position error limit (centimeters)
     * 
     * @return Maximum negative (downward) position error in cm
     * 
     * @details Wrapper for get_pos_error_down_m() with unit conversion.
     * 
     * @see get_pos_error_down_m()
     */
    float get_pos_error_down_cm() const { return get_pos_error_down_m() * 100.0; }

    /**
     * @brief Get maximum downward position error limit (meters)
     * 
     * @return Maximum negative (downward) position error in meters
     * 
     * @details Returns the maximum allowed negative altitude error that constrains the
     *          vertical position P controller output. Limits descent rate command when
     *          vehicle is above target altitude.
     * 
     * @see _p_pos_u_m
     */
    float get_pos_error_down_m() const { return _p_pos_u_m.get_error_min(); }

    /**
     * @brief Get maximum climb rate (centimeters)
     * 
     * @return Maximum climb rate in cm/s
     * 
     * @details Wrapper for get_max_speed_up_ms() with unit conversion.
     * 
     * @see get_max_speed_up_ms(), set_max_speed_accel_U_cm()
     */
    float get_max_speed_up_cms() const { return get_max_speed_up_ms() * 100.0; }

    /**
     * @brief Get maximum climb rate (meters)
     * 
     * @return Maximum climb rate in m/s used for trajectory shaping
     * 
     * @details Returns the current climb rate limit. Positive value indicates upward motion.
     * 
     * @see set_max_speed_accel_U_m()
     */
    float get_max_speed_up_ms() const { return _vel_max_up_ms; }

    /**
     * @brief Get maximum descent rate (centimeters)
     * 
     * @return Maximum descent rate in cm/s (typically negative)
     * 
     * @details Wrapper for get_max_speed_down_ms() with unit conversion.
     * 
     * @see get_max_speed_down_ms(), set_max_speed_accel_U_cm()
     */
    float get_max_speed_down_cms() const { return get_max_speed_down_ms() * 100.0; }

    /**
     * @brief Get maximum descent rate (meters)
     * 
     * @return Maximum descent rate in m/s (typically negative)
     * 
     * @details Returns the current descent rate limit. Negative value indicates downward motion.
     * 
     * @note In NED frame, positive Down = descending
     * @see set_max_speed_accel_U_m()
     */
    float get_max_speed_down_ms() const { return _vel_max_down_ms; }

    /**
     * @brief Initialize vertical controller without allowing descent
     * 
     * @details Initializes the U-axis (vertical) position controller to current position,
     *          velocity, and acceleration while disallowing downward motion. Used during
     *          takeoff or hold scenarios where descent must be prevented.
     *          
     *          Initialization Actions:
     *          - Sets position target to current altitude
     *          - Preserves current vertical velocity (if positive)
     *          - Prevents negative (downward) velocity commands
     *          - Zeros vertical acceleration target
     *          
     *          Typical Use Cases:
     *          - Takeoff sequence initialization
     *          - Emergency altitude hold
     *          - Transition from ground to flight
     * 
     * @note Descent prevention overrides velocity tracking
     * @warning Only call when vehicle should not descend
     * 
     * @see init_U_controller(), init_U_controller_stopping_point()
     */
    void init_U_controller_no_descent();

    /**
     * @brief Initialize vertical controller to stopping point
     * 
     * @details Initializes the U-axis controller to a stationary stopping point with zero
     *          velocity and acceleration. Calculates altitude target that allows smooth
     *          deceleration from current vertical velocity.
     *          
     *          Stopping Point Calculation:
     *          - Accounts for current vertical velocity
     *          - Applies configured deceleration limits
     *          - Generates jerk-limited stopping trajectory
     *          
     *          Result: Vehicle smoothly decelerates to hover at computed altitude.
     * 
     * @note The computed position target can be retrieved with get_pos_target_NEU_m()
     * @see get_stopping_point_U_m(), init_U_controller()
     */
    void init_U_controller_stopping_point();

    /**
     * @brief Relax vertical controller acceleration to zero
     * 
     * @param[in] throttle_setting Current throttle output (0-1) used to assess thrust margin
     * 
     * @details Smoothly decays U-axis acceleration target to zero over time while maintaining
     *          current vertical velocity. Reduces acceleration by approximately 95% every
     *          0.5 seconds (POSCONTROL_RELAX_TC time constant).
     *          
     *          Throttle Setting Usage:
     *          - Low throttle (<0.1): May preserve positive acceleration to prevent descent
     *          - Normal throttle: Full acceleration decay applied
     *          
     *          Effect:
     *          - Gradual transition to pure velocity tracking
     *          - Avoids abrupt vertical motion changes
     *          - Smooths mode transitions
     * 
     * @note Called during flight mode transitions to prevent jerky altitude changes
     * @warning Do not call during landing approach (use land-specific functions)
     * 
     * @see POSCONTROL_RELAX_TC, relax_velocity_controller_NE()
     */
    void relax_U_controller(float throttle_setting);

    /**
     * @brief Fully initialize vertical controller
     * 
     * @details Fully initializes the U-axis controller with current position, velocity,
     *          acceleration, and attitude from AHRS/EKF. Used during standard controller
     *          activation when full vehicle state is known and valid.
     *          
     *          Initialization Actions:
     *          - Sets position target to current altitude
     *          - Sets velocity target to current climb rate
     *          - Sets acceleration target to current vertical acceleration
     *          - Resets PID integrators
     *          - Initializes EKF reset tracking
     *          
     *          Prerequisites:
     *          - Valid EKF altitude estimate
     *          - Valid vertical velocity estimate
     *          - AHRS providing attitude
     * 
     * @note This is a shared initialization function called by other vertical initializers
     * @see init_U_controller_no_descent(), init_U_controller_stopping_point()
     */
    void init_U_controller();

    /**
     * @brief Set desired vertical acceleration (centimeters)
     * 
     * @param[in] accel_u_cmss Desired vertical acceleration in cm/s² (positive = upward)
     * 
     * @details Wrapper for input_accel_U_m() with unit conversion. Virtual to allow
     *          vehicle-specific overrides.
     * 
     * @see input_accel_U_m()
     */
    virtual void input_accel_U_cm(float accel_u_cmss);

    /**
     * @brief Set desired vertical acceleration (meters)
     * 
     * @param[in] accel_u_mss Desired vertical acceleration in m/s² (positive = upward)
     * 
     * @details Sets the desired vertical acceleration using jerk-limited trajectory shaping.
     *          Smoothly transitions from current acceleration to target using configured
     *          jerk limits.
     *          
     *          Jerk-Limited Shaping:
     *          - Gradual acceleration changes (S-curve profile)
     *          - Prevents abrupt thrust commands
     *          - Smoother vertical motion
     *          
     *          Constraints Applied:
     *          - Maximum acceleration: set via set_max_speed_accel_U_m()
     *          - Maximum jerk: _shaping_jerk_u_msss parameter
     *          
     *          Coordinate Frame:
     *          - NEU frame: positive = upward acceleration
     *          - Opposite sign from NED Down convention
     * 
     * @note Position and velocity targets updated via integration
     * @see set_max_speed_accel_U_m(), _shaping_jerk_u_msss
     */
    void input_accel_U_m(float accel_u_mss);

    /**
     * @brief Set desired vertical velocity and acceleration (centimeters)
     * 
     * @param[in,out] vel_u_cms Desired vertical velocity in cm/s (may be modified to respect limits)
     * @param[in] accel_u_cmss Desired vertical acceleration in cm/s²
     * @param[in] limit_output If true, apply limits to combined (desired + correction) output
     * 
     * @details Wrapper for input_vel_accel_U_m() with unit conversion.
     * 
     * @see input_vel_accel_U_m()
     */
    void input_vel_accel_U_cm(float &vel_u_cms, float accel_u_cmss, bool limit_output = true);

    /**
     * @brief Set desired vertical velocity and acceleration (meters)
     * 
     * @param[in,out] vel_u_ms Desired vertical velocity in m/s (may be modified to respect limits)
     * @param[in] accel_u_mss Desired vertical acceleration in m/s²
     * @param[in] limit_output If true, apply limits to combined (desired + correction) output
     * 
     * @details Sets desired vertical velocity and acceleration using jerk-limited trajectory
     *          shaping. Calculates required acceleration to achieve velocity while respecting
     *          kinematic constraints.
     *          
     *          Processing Steps:
     *          1. Apply jerk-limited shaping to acceleration
     *          2. Integrate acceleration to update velocity target
     *          3. Constrain velocity within configured limits
     *          4. Update position target via integration
     *          
     *          Limit Output Behavior:
     *          - limit_output = true: Limits apply to (desired + PID correction)
     *          - limit_output = false: Only desired values limited, correction unlimited
     *          
     *          Typical Usage:
     *          - Velocity-controlled climb/descent
     *          - Manual altitude rate control
     *          - Guided velocity commands
     * 
     * @note vel_u_ms may be modified if it exceeds configured limits
     * @warning Ensure velocity is achievable with available thrust margin
     * 
     * @see set_max_speed_accel_U_m(), input_accel_U_m()
     */
    void input_vel_accel_U_m(float &vel_u_ms, float accel_u_mss, bool limit_output = true);

    /**
     * @brief Set altitude target from climb rate (centimeters)
     * 
     * @param[in] vel_u_cms Desired climb rate in cm/s (positive = climb, negative = descend)
     * 
     * @details Wrapper for set_pos_target_U_from_climb_rate_m() with unit conversion.
     * 
     * @see set_pos_target_U_from_climb_rate_m()
     */
    void set_pos_target_U_from_climb_rate_cm(float vel_u_cms);

    /**
     * @brief Set altitude target from climb rate (meters)
     * 
     * @param[in] vel_u_ms Desired climb rate in m/s (positive = climb, negative = descend)
     * 
     * @details Generates a vertical trajectory by integrating the given climb rate with
     *          jerk-limited acceleration shaping. Target altitude is continuously updated
     *          based on elapsed time and climb rate.
     *          
     *          Trajectory Generation:
     *          - Altitude target = current target + (climb rate × dt)
     *          - Velocity target = climb rate (with jerk shaping)
     *          - Acceleration shaped to smoothly reach velocity
     *          
     *          Use Cases:
     *          - Pilot stick climb/descent control
     *          - Altitude hold with rate adjustment
     *          - Terrain following with vertical rate
     * 
     * @note Call this function continuously to maintain rate control
     * @see input_vel_accel_U_m()
     */
    void set_pos_target_U_from_climb_rate_m(float vel_u_ms);

    /**
     * @brief Land at specified descent rate (centimeters)
     * 
     * @param[in] vel_u_cms Desired descent rate in cm/s (negative = descend)
     * @param[in] ignore_descent_limit If true, allow descent faster than configured maximum
     * 
     * @details Wrapper for land_at_climb_rate_m() with unit conversion.
     * 
     * @see land_at_climb_rate_m()
     */
    void land_at_climb_rate_cm(float vel_u_cms, bool ignore_descent_limit);

    /**
     * @brief Land at specified descent rate (meters)
     * 
     * @param[in] vel_u_ms Desired descent rate in m/s (negative = descend)
     * @param[in] ignore_descent_limit If true, allow descent faster than configured maximum
     * 
     * @details Commands vertical descent at the specified rate using jerk-limited trajectory
     *          shaping. Used during final landing approach to ensure smooth, controlled
     *          touchdown.
     *          
     *          Landing Descent Control:
     *          - Smooth deceleration to touchdown speed
     *          - Jerk-limited for gentle ground contact
     *          - Optional override of descent rate limits
     *          
     *          Descent Limit Override:
     *          - ignore_descent_limit = false: Respects configured max descent rate
     *          - ignore_descent_limit = true: Allows faster descent (use with caution)
     *          
     *          Typical Landing Sequence:
     *          1. Initial descent at normal rate
     *          2. Reduce descent rate near ground
     *          3. Final slow descent for touchdown
     * 
     * @note Typically called repeatedly during landing sequence
     * @warning ignore_descent_limit=true can result in hard landing if misused
     * 
     * @see set_pos_target_U_from_climb_rate_m(), set_max_speed_accel_U_m()
     */
    void land_at_climb_rate_m(float vel_u_ms, bool ignore_descent_limit);

    /**
     * @brief Set vertical position, velocity, and acceleration (centimeters)
     * 
     * @param[in,out] pos_u_cm Desired altitude in cm (may be modified to respect limits)
     * @param[in,out] vel_u_cms Desired vertical velocity in cm/s (may be modified)
     * @param[in] accel_u_cmss Desired vertical acceleration in cm/s²
     * @param[in] limit_output If true, apply limits to combined (desired + correction) output
     * 
     * @details Wrapper for input_pos_vel_accel_U_m() with unit conversion.
     * 
     * @see input_pos_vel_accel_U_m()
     */
    void input_pos_vel_accel_U_cm(float &pos_u_cm, float &vel_u_cms, float accel_u_cmss, bool limit_output = true);

    /**
     * @brief Set vertical position, velocity, and acceleration (meters)
     * 
     * @param[in,out] pos_u_m Desired altitude in meters (may be modified to respect limits)
     * @param[in,out] vel_u_ms Desired vertical velocity in m/s (may be modified)
     * @param[in] accel_u_mss Desired vertical acceleration in m/s²
     * @param[in] limit_output If true, apply limits to combined (desired + correction) output
     * 
     * @details Sets complete vertical state (position, velocity, acceleration) using jerk-limited
     *          trajectory shaping. Calculates required acceleration to track both position and
     *          velocity while respecting kinematic constraints.
     *          
     *          Complete State Control:
     *          - Position tracking with P controller
     *          - Velocity tracking with PID controller
     *          - Feedforward acceleration for smooth motion
     *          - Jerk-limited transitions
     *          
     *          Processing Steps:
     *          1. Apply jerk shaping to acceleration
     *          2. Integrate to update velocity target
     *          3. Integrate to update position target
     *          4. Compute position error → velocity correction
     *          5. Compute velocity error → acceleration correction
     *          6. Combine feedforward + corrections
     *          
     *          Limit Output Behavior:
     *          - limit_output = true: Total command limited
     *          - limit_output = false: Only feedforward limited
     *          
     *          Typical Usage:
     *          - External path planning
     *          - Scripted maneuvers
     *          - Trajectory following
     * 
     * @note Both pos_u_m and vel_u_ms may be modified if they exceed limits
     * @see input_vel_accel_U_m(), update_U_controller()
     */
    void input_pos_vel_accel_U_m(float &pos_u_m, float &vel_u_ms, float accel_u_mss, bool limit_output = true);

    /**
     * @brief Set altitude target with slew rate limiting (centimeters)
     * 
     * @param[in] pos_u_cm Target altitude in cm above EKF origin
     * 
     * @details Wrapper for set_alt_target_with_slew_m() with unit conversion.
     * 
     * @see set_alt_target_with_slew_m()
     */
    void set_alt_target_with_slew_cm(float pos_u_cm);

    /**
     * @brief Set altitude target with slew rate limiting (meters)
     * 
     * @param[in] pos_u_m Target altitude in meters above EKF origin
     * 
     * @details Sets target altitude using jerk-limited trajectory shaping to gradually
     *          transition to the new position. Ensures smooth altitude changes without
     *          abrupt velocity or acceleration commands.
     *          
     *          Slew Rate Limiting:
     *          - Velocity ramped using jerk limits
     *          - Acceleration constrained to configured maximum
     *          - Smooth S-curve position profile
     *          
     *          Effect:
     *          - Altitude gradually approaches target
     *          - No sudden climbs or descents
     *          - Comfortable for occupants
     * 
     * @note Call once to set target; controller handles smooth transition
     * @see set_pos_target_U_from_climb_rate_m()
     */
    void set_alt_target_with_slew_m(float pos_u_m);

    /**
     * @brief Check if vertical controller is active
     * 
     * @return true if U-axis controller has run in the last 5 control loop cycles
     * 
     * @details Determines if the vertical position controller is actively running by
     *          checking the time since last update. Used to detect controller timeout
     *          or inactivity.
     * 
     * @note 5 cycles at 400Hz = 12.5ms timeout
     * @see is_active_NE(), update_U_controller()
     */
    bool is_active_U() const;

    /**
     * @brief Run vertical position controller
     * 
     * @details Executes the vertical (U-axis) position control loop. Implements a cascaded
     *          control architecture: position → velocity → acceleration → throttle.
     *          
     *          Control Loop Structure:
     *          1. Position Error → Velocity Command (P controller)
     *          2. Velocity Error → Acceleration Command (PID controller)
     *          3. Combine feedforward + correction accelerations
     *          4. Apply acceleration limits
     *          5. Convert to throttle via AC_AttitudeControl
     *          
     *          Prerequisites:
     *          - Position target set via input_pos_vel_accel_U_m() or similar
     *          - Velocity target set (from trajectory shaping)
     *          - Acceleration target set (from trajectory shaping)
     *          
     *          Output:
     *          - Acceleration command sent to attitude controller
     *          - Attitude controller converts to throttle
     *          
     *          Update Rate:
     *          - Typically called at 400Hz (main loop rate)
     *          - Must match dt set via set_dt_s()
     * 
     * @note Must call update_estimates() before this function
     * @warning Targets must be set before calling, or controller uses stale values
     * 
     * @see update_NE_controller(), AC_AttitudeControl, set_dt_s()
     */
    void update_U_controller();



    ///
    /// Accessors
    ///

    /**
     * @brief Set 3D position, velocity, and acceleration (centimeters)
     * 
     * @param[in] pos_neu_cm Position in NEU frame, centimeters relative to EKF origin
     * @param[in] vel_neu_cms Velocity in NEU frame, cm/s
     * @param[in] accel_neu_cmss Acceleration in NEU frame, cm/s²
     * 
     * @details Wrapper for set_pos_vel_accel_NEU_m() with unit conversion.
     * 
     * @see set_pos_vel_accel_NEU_m()
     */
    void set_pos_vel_accel_NEU_cm(const Vector3p& pos_neu_cm, const Vector3f& vel_neu_cms, const Vector3f& accel_neu_cmss);

    /**
     * @brief Set 3D position, velocity, and acceleration (meters)
     * 
     * @param[in] pos_neu_m Position in NEU frame, meters relative to EKF origin
     * @param[in] vel_neu_ms Velocity in NEU frame, m/s
     * @param[in] accel_neu_mss Acceleration in NEU frame, m/s²
     * 
     * @details Directly sets the target position, velocity, and acceleration for the full
     *          3D controller (both horizontal NE and vertical U). Used when trajectory
     *          planning or path shaping is performed externally.
     *          
     *          Use Cases:
     *          - External path planner providing complete trajectory
     *          - Scripted maneuvers with pre-computed kinematics
     *          - Following trajectories from companion computer
     *          
     *          Behavior:
     *          - Bypasses internal trajectory shaping
     *          - Controller tracks provided targets directly
     *          - No jerk limiting applied (caller must shape trajectory)
     * 
     * @note Caller is responsible for kinematic feasibility
     * @warning Ensure trajectory respects vehicle limits to avoid tracking errors
     * 
     * @see set_pos_vel_accel_NE_m(), input_pos_vel_accel_U_m()
     */
    void set_pos_vel_accel_NEU_m(const Vector3p& pos_neu_m, const Vector3f& vel_neu_ms, const Vector3f& accel_neu_mss);

    /**
     * @brief Set horizontal position, velocity, and acceleration (centimeters)
     * 
     * @param[in] pos_ne_cm Horizontal position (North-East), centimeters relative to EKF origin
     * @param[in] vel_ne_cms Horizontal velocity (North-East), cm/s
     * @param[in] accel_ne_cmss Horizontal acceleration (North-East), cm/s²
     * 
     * @details Wrapper for set_pos_vel_accel_NE_m() with unit conversion.
     * 
     * @see set_pos_vel_accel_NE_m()
     */
    void set_pos_vel_accel_NE_cm(const Vector2p& pos_ne_cm, const Vector2f& vel_ne_cms, const Vector2f& accel_ne_cmss);

    /**
     * @brief Set horizontal position, velocity, and acceleration (meters)
     * 
     * @param[in] pos_ne_m Horizontal position (North-East), meters relative to EKF origin
     * @param[in] vel_ne_ms Horizontal velocity (North-East), m/s
     * @param[in] accel_ne_mss Horizontal acceleration (North-East), m/s²
     * 
     * @details Directly sets the target position, velocity, and acceleration for the horizontal
     *          (NE) controller only. Used when horizontal path planning is done externally
     *          but vertical control uses internal trajectory shaping.
     *          
     *          Use Cases:
     *          - External waypoint navigation
     *          - Precision horizontal positioning
     *          - Following ground-based trajectories
     *          
     *          Behavior:
     *          - Bypasses internal horizontal trajectory shaping
     *          - Vertical (U) controller unaffected
     *          - No jerk limiting on horizontal axes
     * 
     * @note Vertical target unchanged; use set_pos_vel_accel_NEU_m() for full 3D control
     * @see set_pos_vel_accel_NEU_m()
     */
    void set_pos_vel_accel_NE_m(const Vector2p& pos_ne_m, const Vector2f& vel_ne_ms, const Vector2f& accel_ne_mss);


    /// Position

    /**
     * @brief Get estimated 3D position (centimeters)
     * 
     * @return Current position estimate in NEU frame, centimeters relative to EKF origin
     * 
     * @details Wrapper for get_pos_estimate_NEU_m() with unit conversion.
     * 
     * @see get_pos_estimate_NEU_m()
     */
    const Vector3p get_pos_estimate_NEU_cm() const { return get_pos_estimate_NEU_m() * 100.0; }

    /**
     * @brief Get estimated 3D position (meters)
     * 
     * @return Current position estimate in NEU frame, meters relative to EKF origin
     * 
     * @details Returns the vehicle's estimated position from AHRS/EKF. Updated by
     *          update_estimates() which is called before each controller update.
     *          
     *          Coordinate Frame:
     *          - NEU: North-East-Up relative to EKF origin
     *          - Origin typically set at arming location
     *          
     *          Data Source:
     *          - Horizontal (NE): EKF position estimate
     *          - Vertical (U): Barometric altitude or EKF altitude
     * 
     * @note Position is updated at main loop rate (typically 400Hz)
     * @see update_estimates(), _pos_estimate_neu_m
     */
    const Vector3p& get_pos_estimate_NEU_m() const { return _pos_estimate_neu_m; }

    /**
     * @brief Get 3D position target (centimeters)
     * 
     * @return Current position target in NEU frame, centimeters relative to EKF origin
     * 
     * @details Wrapper for get_pos_target_NEU_m() with unit conversion.
     * 
     * @see get_pos_target_NEU_m()
     */
    const Vector3p get_pos_target_NEU_cm() const { return get_pos_target_NEU_m() * 100.0; }

    /**
     * @brief Get 3D position target (meters)
     * 
     * @return Current position target in NEU frame, meters relative to EKF origin
     * 
     * @details Returns the current position target that the controller is tracking.
     *          This is the desired position plus any active offsets.
     *          
     *          Relationship:
     *          - target = desired + offset
     *          - Offset used for terrain following, scripting adjustments
     *          
     *          Update Frequency:
     *          - Changes with trajectory shaping
     *          - Smoothly approaches desired position
     * 
     * @see get_pos_desired_NEU_m(), get_pos_offset_NEU_m()
     */
    const Vector3p& get_pos_target_NEU_m() const { return _pos_target_neu_m; }

    /**
     * @brief Set desired horizontal position (centimeters)
     * 
     * @param[in] pos_desired_ne_cm Desired NE position, centimeters relative to EKF origin
     * 
     * @details Wrapper for set_pos_desired_NE_m() with unit conversion.
     * 
     * @see set_pos_desired_NE_m()
     */
    void set_pos_desired_NE_cm(const Vector2f& pos_desired_ne_cm) { set_pos_desired_NE_m(pos_desired_ne_cm * 0.01); }

    /**
     * @brief Set desired horizontal position (meters)
     * 
     * @param[in] pos_desired_ne_m Desired NE position, meters relative to EKF origin
     * 
     * @details Sets the desired horizontal position (excluding offsets). The controller
     *          will shape a trajectory to reach this position.
     *          
     *          Desired vs Target:
     *          - Desired: Final goal position (this setter)
     *          - Target: Intermediate position along trajectory
     * 
     * @note Use input_pos_NEU_m() for full trajectory shaping with jerk limits
     * @see get_pos_desired_NEU_m(), set_pos_desired_U_m()
     */
    void set_pos_desired_NE_m(const Vector2f& pos_desired_ne_m) { _pos_desired_neu_m.xy() = pos_desired_ne_m.topostype(); }

    /**
     * @brief Get desired 3D position (centimeters)
     * 
     * @return Desired position in NEU frame, centimeters relative to EKF origin
     * 
     * @details Wrapper for get_pos_desired_NEU_m() with unit conversion.
     * 
     * @see get_pos_desired_NEU_m()
     */
    const Vector3p get_pos_desired_NEU_cm() const { return get_pos_desired_NEU_m() * 100.0; }

    /**
     * @brief Get desired 3D position (meters)
     * 
     * @return Desired position in NEU frame, meters relative to EKF origin
     * 
     * @details Returns the final desired position (goal) without offsets. This is the
     *          ultimate position the controller is trying to reach.
     *          
     *          Desired Position:
     *          - Set by pilot commands or mission waypoints
     *          - Excludes temporary offsets (terrain, scripting)
     *          - May differ from target during trajectory transitions
     * 
     * @see get_pos_target_NEU_m(), set_pos_desired_NE_m()
     */
    const Vector3p& get_pos_desired_NEU_m() const { return _pos_desired_neu_m; }

    /**
     * @brief Get vertical position target (centimeters)
     * 
     * @return Target altitude in centimeters above EKF origin
     * 
     * @details Wrapper for get_pos_target_U_m() with unit conversion.
     * 
     * @see get_pos_target_U_m()
     */
    float get_pos_target_U_cm() const { return get_pos_target_U_m() * 100.0; }

    /**
     * @brief Get vertical position target (meters)
     * 
     * @return Target altitude in meters above EKF origin
     * 
     * @details Returns the current vertical position target (desired + offset).
     * 
     * @note Positive Up in NEU frame
     * @see get_pos_desired_U_m(), get_pos_target_NEU_m()
     */
    float get_pos_target_U_m() const { return _pos_target_neu_m.z; }

    /**
     * @brief Set desired altitude (centimeters)
     * 
     * @param[in] pos_desired_u_cm Desired altitude in centimeters above EKF origin
     * 
     * @details Wrapper for set_pos_desired_U_m() with unit conversion.
     * 
     * @see set_pos_desired_U_m()
     */
    void set_pos_desired_U_cm(float pos_desired_u_cm) { set_pos_desired_U_m(pos_desired_u_cm * 0.01); }

    /**
     * @brief Set desired altitude (meters)
     * 
     * @param[in] pos_desired_u_m Desired altitude in meters above EKF origin
     * 
     * @details Sets the desired vertical position (goal altitude, excluding offsets).
     * 
     * @note Use input_pos_vel_accel_U_m() for trajectory shaping
     * @see get_pos_desired_U_m(), set_pos_desired_NE_m()
     */
    void set_pos_desired_U_m(float pos_desired_u_m) { _pos_desired_neu_m.z = pos_desired_u_m; }

    /**
     * @brief Get desired altitude (centimeters)
     * 
     * @return Desired altitude in centimeters above EKF origin
     * 
     * @details Wrapper for get_pos_desired_U_m() with unit conversion.
     * 
     * @see get_pos_desired_U_m()
     */
    float get_pos_desired_U_cm() const { return get_pos_desired_U_m() * 100.0; }

    /**
     * @brief Get desired altitude (meters)
     * 
     * @return Desired altitude in meters above EKF origin
     * 
     * @details Returns the desired vertical position (goal altitude, excluding offsets).
     * 
     * @see get_pos_target_U_m(), set_pos_desired_U_m()
     */
    float get_pos_desired_U_m() const { return _pos_desired_neu_m.z; }


    /// Stopping Point

    /**
     * @brief Calculate horizontal stopping point (centimeters)
     * 
     * @param[out] stopping_point_neu_cm Predicted stopping position in NE plane, centimeters
     * 
     * @details Wrapper for get_stopping_point_NE_m() with unit conversion.
     * 
     * @see get_stopping_point_NE_m()
     */
    void get_stopping_point_NE_cm(Vector2p &stopping_point_neu_cm) const;

    /**
     * @brief Calculate horizontal stopping point (meters)
     * 
     * @param[out] stopping_point_neu_m Predicted stopping position in NE plane, meters
     * 
     * @details Computes the horizontal position where the vehicle would come to rest if
     *          maximum deceleration were applied immediately.
     *          
     *          Calculation:
     *          - Uses current position, velocity, and acceleration
     *          - Applies configured deceleration limits
     *          - Accounts for jerk-limited braking trajectory
     *          
     *          Use Cases:
     *          - Obstacle avoidance planning
     *          - Brake mode positioning
     *          - Geofence proximity calculations
     *          - Mission abort safety margins
     * 
     * @note Assumes maximum deceleration capability
     * @warning Does not account for wind or external disturbances
     * @see get_max_accel_NE_mss(), get_stopping_point_U_m()
     */
    void get_stopping_point_NE_m(Vector2p &stopping_point_neu_m) const;

    /**
     * @brief Calculate vertical stopping point (centimeters)
     * 
     * @param[out] stopping_point_u_cm Predicted stopping altitude, centimeters
     * 
     * @details Wrapper for get_stopping_point_U_m() with unit conversion.
     * 
     * @see get_stopping_point_U_m()
     */
    void get_stopping_point_U_cm(postype_t &stopping_point_u_cm) const;

    /**
     * @brief Calculate vertical stopping point (meters)
     * 
     * @param[out] stopping_point_u_m Predicted stopping altitude, meters
     * 
     * @details Computes the altitude where the vehicle would stop if maximum vertical
     *          deceleration were applied immediately.
     *          
     *          Calculation:
     *          - Based on current altitude, climb rate, and vertical acceleration
     *          - Uses configured vertical deceleration limit
     *          - Accounts for jerk-limited vertical braking
     *          
     *          Use Cases:
     *          - Altitude fence checks
     *          - Landing abort decisions
     *          - Vertical clearance validation
     * 
     * @note Climb vs descent deceleration may differ
     * @see get_max_accel_U_mss(), get_stopping_point_NE_m()
     */
    void get_stopping_point_U_m(postype_t &stopping_point_u_m) const;


    /// Position Error

    /**
     * @brief Get 3D position error (centimeters)
     * 
     * @return Position error vector in NEU frame, centimeters
     * 
     * @details Wrapper for get_pos_error_NEU_m() with unit conversion.
     * 
     * @see get_pos_error_NEU_m()
     */
    const Vector3f get_pos_error_NEU_cm() const { return get_pos_error_NEU_m() * 100.0; }

    /**
     * @brief Get 3D position error (meters)
     * 
     * @return Position error vector in NEU frame, meters
     * 
     * @details Returns the difference between target and estimated position in all three axes.
     *          
     *          Error Calculation:
     *          - error = target - estimate
     *          - Positive error means vehicle is behind target
     *          - Negative error means vehicle is ahead of target
     *          
     *          Components:
     *          - N, E: Horizontal position errors from _p_pos_ne_m
     *          - U: Vertical position error from _p_pos_u_m
     *          
     *          Use Cases:
     *          - Monitoring controller tracking performance
     *          - Diagnosing position hold issues
     *          - Logging and telemetry
     * 
     * @note Position P controllers use this error to generate velocity commands
     * @see get_pos_error_NE_m(), get_pos_error_U_m()
     */
    const Vector3f get_pos_error_NEU_m() const { return Vector3f(_p_pos_ne_m.get_error().x, _p_pos_ne_m.get_error().y, _p_pos_u_m.get_error()); }

    /**
     * @brief Get horizontal position error magnitude (centimeters)
     * 
     * @return Total NE-plane position error, centimeters
     * 
     * @details Wrapper for get_pos_error_NE_m() with unit conversion.
     * 
     * @see get_pos_error_NE_m()
     */
    float get_pos_error_NE_cm() const { return get_pos_error_NE_m() * 100.0; }

    /**
     * @brief Get horizontal position error magnitude (meters)
     * 
     * @return Total NE-plane position error, meters
     * 
     * @details Returns the Euclidean distance between horizontal target and estimated position.
     *          
     *          Calculation:
     *          - sqrt(error_N² + error_E²)
     *          - Always positive (magnitude only)
     *          
     *          Use Cases:
     *          - Simple horizontal tracking assessment
     *          - Waypoint arrival detection
     *          - Loiter radius verification
     * 
     * @see get_pos_error_NEU_m()
     */
    float get_pos_error_NE_m() const { return _p_pos_ne_m.get_error().length(); }

    /**
     * @brief Get vertical position error (centimeters)
     * 
     * @return Altitude error, centimeters
     * 
     * @details Wrapper for get_pos_error_U_m() with unit conversion.
     * 
     * @see get_pos_error_U_m()
     */
    float get_pos_error_U_cm() const { return get_pos_error_U_m() * 100.0; }

    /**
     * @brief Get vertical position error (meters)
     * 
     * @return Altitude error, meters (positive = below target, negative = above target)
     * 
     * @details Returns the vertical difference between target and estimated altitude.
     *          
     *          Sign Convention (NEU frame):
     *          - Positive error: Vehicle below target altitude (needs to climb)
     *          - Negative error: Vehicle above target altitude (needs to descend)
     *          
     *          Use Cases:
     *          - Altitude hold performance monitoring
     *          - Landing flare adjustments
     *          - Terrain following validation
     * 
     * @note Vertical P controller uses this to generate climb rate command
     * @see get_pos_error_NEU_m()
     */
    float get_pos_error_U_m() const { return _p_pos_u_m.get_error(); }


    /// Velocity

    /**
     * @brief Get estimated 3D velocity (centimeters)
     * 
     * @return Current velocity estimate in NEU frame, cm/s
     * 
     * @details Wrapper for get_vel_estimate_NEU_ms() with unit conversion.
     * 
     * @see get_vel_estimate_NEU_ms()
     */
    const Vector3f get_vel_estimate_NEU_cms() const { return get_vel_estimate_NEU_ms() * 100.0; }

    /**
     * @brief Get estimated 3D velocity (meters)
     * 
     * @return Current velocity estimate in NEU frame, m/s
     * 
     * @details Returns the vehicle's estimated velocity from AHRS/EKF. Updated by
     *          update_estimates() before each controller update.
     *          
     *          Data Source:
     *          - Horizontal (NE): EKF velocity estimate
     *          - Vertical (U): Barometric or EKF climb rate
     *          - Falls back to vertical-only in high vibration
     *          
     *          Coordinate Frame:
     *          - NEU: North-East-Up velocity components
     *          - Earth-fixed frame (not body frame)
     * 
     * @note Updated at main loop rate (typically 400Hz)
     * @see update_estimates(), _vel_estimate_neu_ms
     */
    const Vector3f& get_vel_estimate_NEU_ms() const { return _vel_estimate_neu_ms; }

    /**
     * @brief Set desired 3D velocity (centimeters)
     * 
     * @param[in] vel_desired_neu_cms Desired velocity in NEU frame, cm/s
     * 
     * @details Wrapper for set_vel_desired_NEU_ms() with unit conversion.
     * 
     * @see set_vel_desired_NEU_ms()
     */
    void set_vel_desired_NEU_cms(const Vector3f &vel_desired_neu_cms) { set_vel_desired_NEU_ms(vel_desired_neu_cms * 0.01); }

    /**
     * @brief Set desired 3D velocity (meters)
     * 
     * @param[in] vel_desired_neu_ms Desired velocity in NEU frame, m/s
     * 
     * @details Sets the desired velocity that the controller attempts to achieve.
     *          This is the "feedforward" velocity command.
     *          
     *          Desired vs Target:
     *          - Desired: Final goal velocity (this setter)
     *          - Target: Shaped trajectory velocity (from shaper)
     *          
     *          Typical Use:
     *          - Direct velocity commands from pilot stick
     *          - Waypoint navigation velocity profiles
     *          - Scripted velocity maneuvers
     * 
     * @see get_vel_desired_NEU_ms(), get_vel_target_NEU_ms()
     */
    void set_vel_desired_NEU_ms(const Vector3f &vel_desired_neu_ms) { _vel_desired_neu_ms = vel_desired_neu_ms; }

    /**
     * @brief Set desired horizontal velocity (centimeters)
     * 
     * @param[in] vel_desired_ne_cms Desired NE velocity, cm/s
     * 
     * @details Wrapper for set_vel_desired_NE_ms() with unit conversion.
     * 
     * @see set_vel_desired_NE_ms()
     */
    void set_vel_desired_NE_cms(const Vector2f &vel_desired_ne_cms) { set_vel_desired_NE_ms(vel_desired_ne_cms * 0.01); }

    /**
     * @brief Set desired horizontal velocity (meters)
     * 
     * @param[in] vel_desired_ne_ms Desired NE velocity, m/s
     * 
     * @details Sets desired horizontal velocity only, leaving vertical unchanged.
     * 
     * @see set_vel_desired_NEU_ms()
     */
    void set_vel_desired_NE_ms(const Vector2f &vel_desired_ne_ms) { _vel_desired_neu_ms.xy() = vel_desired_ne_ms; }

    /**
     * @brief Get desired 3D velocity (centimeters)
     * 
     * @return Desired velocity in NEU frame, cm/s
     * 
     * @details Wrapper for get_vel_desired_NEU_ms() with unit conversion.
     * 
     * @see get_vel_desired_NEU_ms()
     */
    const Vector3f get_vel_desired_NEU_cms() const { return get_vel_desired_NEU_ms() * 100.0; }

    /**
     * @brief Get desired 3D velocity (meters)
     * 
     * @return Desired velocity in NEU frame, m/s
     * 
     * @details Returns the goal velocity the controller is trying to achieve.
     * 
     * @see set_vel_desired_NEU_ms(), get_vel_target_NEU_ms()
     */
    const Vector3f& get_vel_desired_NEU_ms() const { return _vel_desired_neu_ms; }

    /**
     * @brief Get 3D velocity target (centimeters)
     * 
     * @return Velocity target in NEU frame, cm/s
     * 
     * @details Wrapper for get_vel_target_NEU_ms() with unit conversion.
     * 
     * @see get_vel_target_NEU_ms()
     */
    const Vector3f get_vel_target_NEU_cms() const { return get_vel_target_NEU_ms() * 100.0; }

    /**
     * @brief Get 3D velocity target (meters)
     * 
     * @return Velocity target in NEU frame, m/s
     * 
     * @details Returns the current velocity target from trajectory shaping.
     *          This is the intermediate velocity along the shaped path.
     *          
     *          Target Generation:
     *          - Position P controller output (position → velocity)
     *          - Jerk-limited trajectory shaping
     *          - Smoothly approaches desired velocity
     *          
     *          Controller Flow:
     *          - Velocity PID uses (target - estimate) as error
     *          - Output is acceleration command
     * 
     * @see get_vel_desired_NEU_ms(), _vel_target_neu_ms
     */
    const Vector3f& get_vel_target_NEU_ms() const { return _vel_target_neu_ms; }

    /**
     * @brief Set desired vertical velocity (centimeters)
     * 
     * @param[in] vel_desired_u_cms Desired climb rate, cm/s
     * 
     * @details Wrapper for set_vel_desired_U_ms() with unit conversion.
     * 
     * @see set_vel_desired_U_ms()
     */
    void set_vel_desired_U_cms(float vel_desired_u_cms) { set_vel_desired_U_ms(vel_desired_u_cms * 0.01); }

    /**
     * @brief Set desired vertical velocity (meters)
     * 
     * @param[in] vel_desired_u_ms Desired climb rate, m/s
     * 
     * @details Sets desired vertical velocity (climb rate) only.
     *          
     *          Sign Convention (NEU frame):
     *          - Positive: Upward (climbing)
     *          - Negative: Downward (descending)
     * 
     * @see set_vel_desired_NEU_ms()
     */
    void set_vel_desired_U_ms(float vel_desired_u_ms) { _vel_desired_neu_ms.z = vel_desired_u_ms; }

    /**
     * @brief Get vertical velocity target (centimeters)
     * 
     * @return Vertical velocity target, cm/s
     * 
     * @details Wrapper for get_vel_target_U_ms() with unit conversion.
     * 
     * @see get_vel_target_U_ms()
     */
    float get_vel_target_U_cms() const { return get_vel_target_U_ms() * 100.0; }

    /**
     * @brief Get vertical velocity target (meters)
     * 
     * @return Vertical velocity target, m/s
     * 
     * @details Returns the current vertical velocity target from trajectory shaping.
     * 
     * @note Positive = climbing, negative = descending (NEU frame)
     * @see get_vel_target_NEU_ms()
     */
    float get_vel_target_U_ms() const { return _vel_target_neu_ms.z; }


    /// Acceleration

    /**
     * @brief Set desired horizontal acceleration (centimeters)
     * 
     * @param[in] accel_desired_neu_cmss Desired NE acceleration, cm/s²
     * 
     * @details Wrapper for set_accel_desired_NE_mss() with unit conversion.
     * 
     * @see set_accel_desired_NE_mss()
     */
    void set_accel_desired_NE_cmss(const Vector2f &accel_desired_neu_cmss) { set_accel_desired_NE_mss(accel_desired_neu_cmss * 0.01); }

    /**
     * @brief Set desired horizontal acceleration (meters)
     * 
     * @param[in] accel_desired_neu_mss Desired NE acceleration, m/s²
     * 
     * @details Sets the desired horizontal acceleration (feedforward term).
     *          Used when external path planners provide acceleration profiles.
     *          
     *          Feedforward Path:
     *          - Desired acceleration bypasses position and velocity control
     *          - Added directly to PID correction acceleration
     *          - Improves tracking of known trajectories
     *          
     *          Typical Use:
     *          - Aggressive maneuvers with pre-planned acceleration
     *          - Trajectory following from companion computer
     *          - Coordinated turns with specific g-loading
     * 
     * @note Most flight modes don't set desired acceleration directly
     * @see get_accel_target_NEU_mss()
     */
    void set_accel_desired_NE_mss(const Vector2f &accel_desired_neu_mss) { _accel_desired_neu_mss.xy() = accel_desired_neu_mss; }

    /**
     * @brief Get 3D acceleration target (centimeters)
     * 
     * @return Target acceleration in NEU frame, cm/s²
     * 
     * @details Wrapper for get_accel_target_NEU_mss() with unit conversion.
     * 
     * @see get_accel_target_NEU_mss()
     */
    const Vector3f get_accel_target_NEU_cmss() const { return get_accel_target_NEU_mss() * 100.0; }

    /**
     * @brief Get 3D acceleration target (meters)
     * 
     * @return Target acceleration in NEU frame, m/s²
     * 
     * @details Returns the total commanded acceleration from all control stages.
     *          
     *          Acceleration Composition:
     *          - Feedforward: Desired acceleration from trajectory
     *          - Feedback: PID correction from position/velocity errors
     *          - Total: feedforward + feedback = target
     *          
     *          Output Path:
     *          - Horizontal (NE): Converted to lean angles via attitude controller
     *          - Vertical (U): Converted to throttle via motor mixer
     *          
     *          Constraints:
     *          - Limited by configured max acceleration
     *          - Limited by maximum lean angle (horizontal)
     *          - Limited by motor authority (vertical)
     * 
     * @note This is the final controller output (Earth frame acceleration command)
     * @see accel_NE_mss_to_lean_angles_rad(), _accel_target_neu_mss
     */
    const Vector3f& get_accel_target_NEU_mss() const { return _accel_target_neu_mss; }


    /// Terrain

    /**
     * @brief Set terrain target altitude (centimeters)
     * 
     * @param[in] pos_terrain_target_u_cm Terrain target altitude, centimeters above EKF origin
     * 
     * @details Wrapper for set_pos_terrain_target_U_m() with unit conversion.
     * 
     * @see set_pos_terrain_target_U_m()
     */
    void set_pos_terrain_target_U_cm(float pos_terrain_target_u_cm) { set_pos_terrain_target_U_m(pos_terrain_target_u_cm * 0.01); }

    /**
     * @brief Set terrain target altitude (meters)
     * 
     * @param[in] pos_terrain_target_u_m Terrain target altitude, meters above EKF origin
     * 
     * @details Sets the target terrain altitude for terrain-following mode.
     *          The internal terrain estimate (_pos_terrain_u_m) gradually moves
     *          toward this target value.
     *          
     *          Terrain Following Concept:
     *          - Vehicle maintains altitude above terrain
     *          - Terrain altitude changes as vehicle moves
     *          - Horizontal speed scaled to maintain vertical clearance buffer
     *          
     *          Update Source:
     *          - Typically from terrain database or rangefinder
     *          - Updated continuously during terrain-following flight
     * 
     * @note Terrain altitude is in NEU frame (Up = positive altitude)
     * @see get_pos_terrain_U_m(), pos_terrain_U_scaler_m()
     */
    void set_pos_terrain_target_U_m(float pos_terrain_target_u_m) { _pos_terrain_target_u_m = pos_terrain_target_u_m; }

    /**
     * @brief Initialize terrain altitude (centimeters)
     * 
     * @param[in] pos_terrain_u_cm Initial terrain altitude, centimeters above EKF origin
     * 
     * @details Wrapper for init_pos_terrain_U_m() with unit conversion.
     * 
     * @see init_pos_terrain_U_m()
     */
    void init_pos_terrain_U_cm(float pos_terrain_u_cm);

    /**
     * @brief Initialize terrain altitude (meters)
     * 
     * @param[in] pos_terrain_u_m Initial terrain altitude, meters above EKF origin
     * 
     * @details Initializes both the terrain estimate and terrain target to the same value.
     *          Used when starting terrain-following mode.
     *          
     *          Initialization:
     *          - Sets _pos_terrain_u_m = pos_terrain_u_m
     *          - Sets _pos_terrain_target_u_m = pos_terrain_u_m
     *          - Zeros terrain velocity and acceleration
     *          
     *          When to Call:
     *          - Beginning of terrain-following flight mode
     *          - After terrain data becomes available
     *          - Mode transitions to terrain-aware navigation
     * 
     * @see set_pos_terrain_target_U_m(), update_terrain()
     */
    void init_pos_terrain_U_m(float pos_terrain_u_m);

    /**
     * @brief Get current terrain altitude (centimeters)
     * 
     * @return Terrain altitude, centimeters above EKF origin
     * 
     * @details Wrapper for get_pos_terrain_U_m() with unit conversion.
     * 
     * @see get_pos_terrain_U_m()
     */
    float get_pos_terrain_U_cm() const { return get_pos_terrain_U_m() * 100.0; }

    /**
     * @brief Get current terrain altitude (meters)
     * 
     * @return Terrain altitude, meters above EKF origin
     * 
     * @details Returns the current terrain altitude estimate used by the position controller.
     *          This value is filtered/smoothed toward the terrain target.
     *          
     *          Terrain Estimate:
     *          - Gradually tracks terrain target with time constant filtering
     *          - Used to compute terrain clearance buffer
     *          - Affects horizontal speed scaling during terrain following
     *          
     *          Coordinate Frame:
     *          - NEU frame: Up = positive altitude
     *          - Relative to EKF origin (typically arming location)
     * 
     * @note Updated by update_terrain() each controller cycle
     * @see set_pos_terrain_target_U_m(), pos_terrain_U_scaler_m()
     */
    float get_pos_terrain_U_m() const { return _pos_terrain_u_m; }


    /// Offset

#if AP_SCRIPTING_ENABLED
    /**
     * @brief Sets position, velocity, and acceleration offsets for Lua scripting
     * 
     * @details Allows Lua scripts to add offsets to the controller's internal targets.
     *          These offsets are applied in addition to normal navigation commands,
     *          enabling scripted maneuvers and trajectory modifications.
     * 
     * @param[in] pos_offset_NED_m     Position offset in NED frame (meters)
     * @param[in] vel_offset_NED_ms    Velocity offset in NED frame (m/s)
     * @param[in] accel_offset_NED_mss Acceleration offset in NED frame (m/s²)
     * 
     * @return true if offsets were successfully applied
     * 
     * @note NED frame: North=+X, East=+Y, Down=+Z
     * @note Used exclusively by Lua scripting interface
     * @see get_posvelaccel_offset()
     */
    // Sets additional position, velocity, and acceleration offsets in meters (NED frame) for scripting.
    // Offsets are added to the controller’s internal target.
    // Used in LUA
    bool set_posvelaccel_offset(const Vector3f &pos_offset_NED_m, const Vector3f &vel_offset_NED_ms, const Vector3f &accel_offset_NED_mss);

    /**
     * @brief Retrieves current position, velocity, and acceleration offsets from Lua scripting
     * 
     * @details Returns the currently active offsets that were set via set_posvelaccel_offset().
     *          Useful for monitoring or adjusting scripted trajectory modifications.
     * 
     * @param[out] pos_offset_NED_m     Current position offset in NED frame (meters)
     * @param[out] vel_offset_NED_ms    Current velocity offset in NED frame (m/s)
     * @param[out] accel_offset_NED_mss Current acceleration offset in NED frame (m/s²)
     * 
     * @return true if offsets were successfully retrieved
     * 
     * @note NED frame: North=+X, East=+Y, Down=+Z
     * @note Used exclusively by Lua scripting interface
     * @see set_posvelaccel_offset()
     */
    bool get_posvelaccel_offset(Vector3f &pos_offset_NED_m, Vector3f &vel_offset_NED_ms, Vector3f &accel_offset_NED_mss);

    /**
     * @brief Retrieves current target velocity including scripted offsets for Lua
     * 
     * @details Returns the complete velocity target including both navigation commands
     *          and any scripted offsets. Represents the actual velocity the controller
     *          is attempting to achieve.
     * 
     * @param[out] vel_target_NED_ms Target velocity in NED frame (m/s)
     * 
     * @return true if target velocity was successfully retrieved
     * 
     * @note NED frame: North=+X, East=+Y, Down=+Z
     * @note Includes both desired velocity and scripted offsets
     * @note Used exclusively by Lua scripting interface
     * @see get_vel_target_NEU_ms()
     */
    bool get_vel_target(Vector3f &vel_target_NED_ms);

    /**
     * @brief Retrieves current target acceleration including scripted offsets for Lua
     * 
     * @details Returns the complete acceleration target including both navigation commands
     *          and any scripted offsets. Represents the actual acceleration the controller
     *          is attempting to achieve.
     * 
     * @param[out] accel_target_NED_mss Target acceleration in NED frame (m/s²)
     * 
     * @return true if target acceleration was successfully retrieved
     * 
     * @note NED frame: North=+X, East=+Y, Down=+Z
     * @note Includes both desired acceleration and scripted offsets
     * @note Used exclusively by Lua scripting interface
     * @see get_accel_target_NEU_mss()
     */
    bool get_accel_target(Vector3f &accel_target_NED_mss);
#endif

    // Sets NE offset targets (position [cm], velocity [cm/s], acceleration [cm/s²]) from EKF origin.
    // Offsets must be refreshed at least every 3 seconds to remain active.
    // See set_posvelaccel_offset_target_NE_m() for full details.
    void set_posvelaccel_offset_target_NE_cm(const Vector2p& pos_offset_target_ne_cm, const Vector2f& vel_offset_target_ne_cms, const Vector2f& accel_offset_target_ne_cmss);

    // Sets NE offset targets in meters, m/s, and m/s².
    void set_posvelaccel_offset_target_NE_m(const Vector2p& pos_offset_target_ne_m, const Vector2f& vel_offset_target_ne_ms, const Vector2f& accel_offset_target_ne_mss);

    // Sets vertical offset targets (cm, cm/s, cm/s²) from EKF origin.
    // See set_posvelaccel_offset_target_U_m() for full details.
    void set_posvelaccel_offset_target_U_cm(float pos_offset_target_u_cm, float vel_offset_target_u_cms, float accel_offset_target_u_cmss);

    // Sets vertical offset targets (m, m/s, m/s²) from EKF origin.
    void set_posvelaccel_offset_target_U_m(float pos_offset_target_u_m, float vel_offset_target_u_ms, float accel_offset_target_u_mss);

    // Returns current NEU position offset in cm.
    // See get_pos_offset_NEU_m() for full details.
    const Vector3p get_pos_offset_NEU_cm() const { return get_pos_offset_NEU_m() * 100.0; }

    // Returns current NEU position offset in meters.
    const Vector3p& get_pos_offset_NEU_m() const { return _pos_offset_neu_m; }

    // Returns current NEU velocity offset in cm/s.
    // See get_vel_offset_NEU_ms() for full details.
    const Vector3f get_vel_offset_NEU_cms() const { return get_vel_offset_NEU_ms() * 100.0; }

    // Returns current NEU velocity offset in m/s.
    const Vector3f& get_vel_offset_NEU_ms() const { return _vel_offset_neu_ms; }

    // Returns current NEU acceleration offset in cm/s².
    // See get_accel_offset_NEU_mss() for full details.
    const Vector3f get_accel_offset_NEU_cmss() const { return get_accel_offset_NEU_mss() * 100.0; }

    // Returns current NEU acceleration offset in m/s².
    const Vector3f& get_accel_offset_NEU_mss() const { return _accel_offset_neu_mss; }

    // Sets vertical position offset in meters above EKF origin.
    void set_pos_offset_U_m(float pos_offset_u_m) { _pos_offset_neu_m.z = pos_offset_u_m; }

    // Returns vertical position offset in cm above EKF origin.
    // See get_pos_offset_U_m() for full details.
    float get_pos_offset_U_cm() const { return get_pos_offset_U_m() * 100.0; }

    // Returns vertical position offset in meters above EKF origin.
    float get_pos_offset_U_m() const { return _pos_offset_neu_m.z; }

    // Returns vertical velocity offset in cm/s.
    // See get_vel_offset_U_ms() for full details.
    float get_vel_offset_U_cms() const { return get_vel_offset_U_ms() * 100.0; }

    // Returns vertical velocity offset in m/s.
    float get_vel_offset_U_ms() const { return _vel_offset_neu_ms.z; }

    // Returns vertical acceleration offset in cm/s².
    // See get_accel_offset_U_mss() for full details.
    float get_accel_offset_U_cmss() const { return get_accel_offset_U_mss() * 100.0; }

    // Returns vertical acceleration offset in m/s².
    float get_accel_offset_U_mss() const { return _accel_offset_neu_mss.z; }

    /// Outputs

    // Returns desired roll angle in radians for the attitude controller
    float get_roll_rad() const { return _roll_target_rad; }

    // Returns desired pitch angle in radians for the attitude controller.
    float get_pitch_rad() const { return _pitch_target_rad; }

    // Returns desired yaw angle in radians for the attitude controller.
    float get_yaw_rad() const { return _yaw_target_rad; }

    // Returns desired yaw rate in radians/second for the attitude controller.
    float get_yaw_rate_rads() const { return _yaw_rate_target_rads; }

    // Returns desired roll angle in centidegrees for the attitude controller.
    // See get_roll_rad() for full details.
    float get_roll_cd() const { return rad_to_cd(_roll_target_rad); }

    // Returns desired pitch angle in centidegrees for the attitude controller.
    // See get_pitch_rad() for full details.
    float get_pitch_cd() const { return rad_to_cd(_pitch_target_rad); }

    // Returns desired yaw angle in centidegrees for the attitude controller.
    // See get_yaw_rad() for full details.
    float get_yaw_cd() const { return rad_to_cd(_yaw_target_rad); }

    // Returns desired yaw rate in centidegrees/second for the attitude controller.
    // See get_yaw_rate_rads() for full details.
    float get_yaw_rate_cds() const { return rad_to_cd(_yaw_rate_target_rads); }

    // Returns desired thrust direction as a unit vector in the body frame.
    Vector3f get_thrust_vector() const;

    // Returns bearing from current position to position target in radians.
    // 0 = North, positive = clockwise.
    float get_bearing_to_target_rad() const;

    // Returns the maximum allowed roll/pitch angle in radians.
    float get_lean_angle_max_rad() const;

    // Overrides the maximum allowed roll/pitch angle in radians.
    // A value of 0 reverts to using the ANGLE_MAX parameter.
    void set_lean_angle_max_rad(float angle_max_rad) { _angle_max_override_rad = angle_max_rad; }

    // Overrides the maximum allowed roll/pitch angle in degrees.
    // See set_lean_angle_max_rad() for full details.
    void set_lean_angle_max_deg(const float angle_max_deg) { set_lean_angle_max_rad(radians(angle_max_deg)); }

    // Overrides the maximum allowed roll/pitch angle in centidegrees.
    // See set_lean_angle_max_rad() for full details.
    void set_lean_angle_max_cd(const float angle_max_cd) { set_lean_angle_max_rad(cd_to_rad(angle_max_cd)); }

    /// Other

    // Returns reference to the NE position P controller.
    AC_P_2D& get_pos_NE_p() { return _p_pos_ne_m; }

    // Returns reference to the U (vertical) position P controller.
    AC_P_1D& get_pos_U_p() { return _p_pos_u_m; }

    // Returns reference to the NE velocity PID controller.
    AC_PID_2D& get_vel_NE_pid() { return _pid_vel_ne_cm; }

    // Returns reference to the U (vertical) velocity PID controller.
    AC_PID_Basic& get_vel_U_pid() { return _pid_vel_u_cm; }

    // Returns reference to the U acceleration PID controller.
    AC_PID& get_accel_U_pid() { return _pid_accel_u_cm_to_kt; }

    // Marks that NE acceleration has been externally limited.
    // Prevents I-term windup by storing the current target direction.
    void set_externally_limited_NE() { _limit_vector_neu.x = _accel_target_neu_mss.x; _limit_vector_neu.y = _accel_target_neu_mss.y; }

    // Converts lean angles (rad) to NEU acceleration in cm/s².
    // See lean_angles_rad_to_accel_NEU_mss() for full details.
    Vector3f lean_angles_rad_to_accel_NEU_cmss(const Vector3f& att_target_euler_rad) const;

    // Converts lean angles (rad) to NEU acceleration in m/s².
    Vector3f lean_angles_rad_to_accel_NEU_mss(const Vector3f& att_target_euler_rad) const;

    // Writes position controller diagnostic logs (PSCN, PSCE, etc).
    void write_log();

    // Performs pre-arm checks for position control parameters and EKF readiness.
    // Returns false if failure_msg is populated.
    bool pre_arm_checks(const char *param_prefix,
                        char *failure_msg,
                        const uint8_t failure_msg_len);

    // Enables or disables vibration compensation mode.
    // When enabled, disables use of horizontal velocity estimates.
    void set_vibe_comp(bool on_off) { _vibe_comp_enabled = on_off; }

    // Returns confidence (0–1) in vertical control authority based on output usage.
    // Used to assess throttle margin and PID effectiveness.
    float get_vel_U_control_ratio() const { return constrain_float(_vel_u_control_ratio, 0.0f, 1.0f); }

    // Returns lateral distance to closest point on active trajectory in meters.
    // Used to assess horizontal deviation from path.
    float crosstrack_error() const;

    // Resets NEU position controller state to prevent transients when exiting standby.
    // Zeros I-terms and aligns targets to current position.
    void standby_NEU_reset();

    // Returns measured vertical (Up) acceleration in cm/s² (Earth frame, gravity-compensated).
    // See get_measured_accel_U_mss() for full details.
    float get_measured_accel_U_cmss() const { return get_measured_accel_U_mss() * 100.0; }

    // Returns measured vertical (Up) acceleration in m/s² (Earth frame, gravity-compensated).
    // Positive = upward acceleration.
    float get_measured_accel_U_mss() const { return -(_ahrs.get_accel_ef().z + GRAVITY_MSS); }

    // Returns true if the requested forward pitch is limited by the configured tilt constraint.
    bool get_fwd_pitch_is_limited() const;
    
    // Sets artificial NE position disturbance in centimeters.
    // See set_disturb_pos_NE_m() for full details.
    void set_disturb_pos_NE_cm(const Vector2f& disturb_pos_cm) { set_disturb_pos_NE_m(disturb_pos_cm * 0.01); }

    // Sets artificial NE position disturbance in meters.
    void set_disturb_pos_NE_m(const Vector2f& disturb_pos_m) { _disturb_pos_ne_m = disturb_pos_m; }

    // Sets artificial NE velocity disturbance in cm/s.
    // See set_disturb_vel_NE_ms() for full details.
    void set_disturb_vel_NE_cms(const Vector2f& disturb_vel_cms) { set_disturb_vel_NE_ms(disturb_vel_cms * 0.01); }

    // Sets artificial NE velocity disturbance in m/s.
    void set_disturb_vel_NE_ms(const Vector2f& disturb_vel_ms) { _disturb_vel_ne_ms = disturb_vel_ms; }

    static const struct AP_Param::GroupInfo var_info[];

    // Logs position controller state along the North axis to PSCN..
    // Logs desired, target, and actual position [m], velocity [m/s], and acceleration [m/s²].
    static void Write_PSCN(float pos_desired_m, float pos_target_m, float pos_m, float vel_desired_ms, float vel_target_ms, float vel_ms, float accel_desired_mss, float accel_target_mss, float accel_mss);

    // Logs position controller state along the East axis to PSCE.
    // Logs desired, target, and actual values for position [m], velocity [m/s], and acceleration [m/s²].
    static void Write_PSCE(float pos_desired_m, float pos_target_m, float pos_m, float vel_desired_ms, float vel_target_ms, float vel_ms, float accel_desired_mss, float accel_target_mss, float accel_mss);

    // Logs position controller state along the Down (vertical) axis to PSCD.
    // Logs desired, target, and actual values for position [m], velocity [m/s], and acceleration [m/s²].
    static void Write_PSCD(float pos_desired_m, float pos_target_m, float pos_m, float vel_desired_ms, float vel_target_ms, float vel_ms, float accel_desired_mss, float accel_target_mss, float accel_mss);

    // Logs offset tracking along the North axis to PSON.
    // Logs target and actual offset for position [m], velocity [m/s], and acceleration [m/s²].
    static void Write_PSON(float pos_target_offset_m, float pos_offset_m, float vel_target_offset_ms, float vel_offset_ms, float accel_target_offset_mss, float accel_offset_mss);

    // Logs offset tracking along the East axis to PSOE.
    // Logs target and actual offset for position [m], velocity [m/s], and acceleration [m/s²].
    static void Write_PSOE(float pos_target_offset_m, float pos_offset_m, float vel_target_offset_ms, float vel_offset_ms, float accel_target_offset_mss, float accel_offset_mss);

    // Logs offset tracking along the Down axis to PSOD.
    // Logs target and actual offset for position [m], velocity [m/s], and acceleration [m/s²].
    static void Write_PSOD(float pos_target_offset_m, float pos_offset_m, float vel_target_offset_ms, float vel_offset_ms, float accel_target_offset_mss, float accel_offset_mss);

    // Logs terrain-following offset tracking along the Down axis to PSOT.
    // Logs target and actual offset for position [m], velocity [m/s], and acceleration [m/s²].
    static void Write_PSOT(float pos_target_offset_m, float pos_offset_m, float vel_target_offset_ms, float vel_offset_ms, float accel_target_offset_mss, float accel_offset_mss);


    // Returns pointer to the global AC_PosControl singleton.
    static AC_PosControl *get_singleton(void) { return _singleton; }

protected:

    // Calculates vertical throttle using vibration-resistant feedforward estimation.
    // Returns throttle output using manual feedforward gain for vibration compensation mode.
    // Integrator is adjusted using velocity error when PID is being overridden.
    float get_throttle_with_vibration_override();

    // Converts horizontal acceleration (m/s²) to roll/pitch lean angles in radians.
    void accel_NE_mss_to_lean_angles_rad(float accel_n_mss, float accel_e_mss, float& roll_target_rad, float& pitch_target_rad) const;

    // Converts current target lean angles to NE acceleration in m/s².
    void lean_angles_to_accel_NE_mss(float& accel_n_mss, float& accel_e_mss) const;

    // Computes desired yaw and yaw rate based on the NE acceleration and velocity vectors.
    // Aligns yaw with the direction of travel if speed exceeds 5% of maximum.
    void calculate_yaw_and_rate_yaw();

    // Computes scaling factor to increase max vertical accel/jerk if vertical speed exceeds configured limits.
    float calculate_overspeed_gain();


    /// Terrain Following

    // Initializes terrain position, velocity, and acceleration to match the terrain target.
    void init_terrain();

    // Updates terrain estimate (_pos_terrain_u_m) toward target using filter time constants.
    void update_terrain();


    /// Offsets

    // Initializes NE position/velocity/acceleration offsets to match their respective targets.
    void init_offsets_NE();

    // Initializes vertical (U) offsets to match their respective targets.
    void init_offsets_U();

    // Updates NE offsets by gradually moving them toward their targets.
    void update_offsets_NE();

    // Updates vertical (U) offsets by gradually moving them toward their targets.
    void update_offsets_U();

    // Initializes tracking of NE EKF position resets.
    void init_ekf_NE_reset();

    // Handles NE position reset detection and response (e.g., clearing accumulated errors).
    void handle_ekf_NE_reset();

    // Initializes tracking of vertical (U) EKF resets.
    void init_ekf_U_reset();

    // Handles U EKF reset detection and response.
    void handle_ekf_U_reset();

    // references to inertial nav and ahrs libraries
    AP_AHRS_View&           _ahrs;              ///< Reference to AHRS providing position/velocity estimates from EKF
    const class AP_Motors&  _motors;            ///< Reference to motor library for thrust calculations and limits
    AC_AttitudeControl&     _attitude_control;  ///< Reference to attitude controller receiving roll/pitch/yaw commands

    // parameters
    AP_Float        _lean_angle_max_deg;    ///< Maximum autopilot commanded angle (degrees). Zero = use ANGLE_MAX parameter
    AP_Float        _shaping_jerk_ne_msss;  ///< Horizontal jerk limit (m/s³) for S-curve trajectory shaping
    AP_Float        _shaping_jerk_u_msss;   ///< Vertical jerk limit (m/s³) for S-curve trajectory shaping
    AC_P_2D         _p_pos_ne_m;            ///< Horizontal position P controller: position error (m) → velocity target (m/s)
    AC_P_1D         _p_pos_u_m;             ///< Vertical position P controller: altitude error (m) → climb rate target (m/s)
    AC_PID_2D       _pid_vel_ne_cm;         ///< Horizontal velocity PID controller: velocity error (cm/s) → acceleration (cm/s²)
    AC_PID_Basic    _pid_vel_u_cm;          ///< Vertical velocity PID controller: climb rate error (cm/s) → acceleration (cm/s²)
    AC_PID          _pid_accel_u_cm_to_kt;  ///< Vertical acceleration PID controller: accel error (cm/s²) → throttle (0-1000)

    // internal variables
    float       _dt_s;                     ///< Controller timestep (seconds), typically 0.0025s (400Hz)
    uint32_t    _last_update_ne_ticks;     ///< System ticks of last update_NE_controller() call for activity tracking
    uint32_t    _last_update_u_ticks;      ///< System ticks of last update_U_controller() call for activity tracking
    float       _vel_max_ne_ms;            ///< Maximum horizontal speed (m/s) for trajectory shaping
    float       _vel_max_up_ms;            ///< Maximum climb rate (m/s) for trajectory shaping
    float       _vel_max_down_ms;          ///< Maximum descent rate (m/s) for trajectory shaping (typically negative)
    float       _accel_max_ne_mss;         ///< Maximum horizontal acceleration (m/s²) for trajectory shaping
    float       _accel_max_u_mss;          ///< Maximum vertical acceleration (m/s²) for trajectory shaping
    float       _jerk_max_ne_msss;         ///< Horizontal jerk limit (m/s³) for S-curve acceleration profiles
    float       _jerk_max_u_msss;          ///< Vertical jerk limit (m/s³) for S-curve acceleration profiles
    float       _vel_u_control_ratio = 2.0f;    ///< Vertical control authority confidence (0-1), based on throttle margin
    Vector2f    _disturb_pos_ne_m;         ///< Horizontal position disturbance (m) injected by system identification mode
    Vector2f    _disturb_vel_ne_ms;        ///< Horizontal velocity disturbance (m/s) injected by system identification mode
    float       _ne_control_scale_factor = 1.0; ///< Horizontal control gain multiplier (0-1), allows external authority scaling

    // output from controller
    float       _roll_target_rad;            ///< Desired roll angle (radians) sent to attitude controller
    float       _pitch_target_rad;           ///< Desired pitch angle (radians) sent to attitude controller
    float       _yaw_target_rad;             ///< Desired yaw angle (radians) sent to attitude controller
    float       _yaw_rate_target_rads;       ///< Desired yaw rate (rad/s) sent to attitude controller

    // position controller internal variables
    Vector3p    _pos_estimate_neu_m;       ///< Current position estimate (m) in NEU frame from EKF
    Vector3p    _pos_desired_neu_m;        ///< Desired position (m) in NEU frame = target minus offsets
    Vector3p    _pos_target_neu_m;         ///< Target position (m) in NEU frame = desired plus offsets
    Vector3f    _vel_estimate_neu_ms;      ///< Current velocity estimate (m/s) in NEU frame from EKF
    Vector3f    _vel_desired_neu_ms;       ///< Desired velocity (m/s) in NEU frame from pilot/navigation
    Vector3f    _vel_target_neu_ms;        ///< Target velocity (m/s) in NEU frame from position P controller output
    Vector3f    _accel_desired_neu_mss;    ///< Desired acceleration (m/s²) in NEU frame (feedforward component)
    Vector3f    _accel_target_neu_mss;     ///< Target acceleration (m/s²) in NEU frame sent to attitude controller
    // TODO: separate the limit vector into ne and u. ne is based on acceleration while u is set ±1 based on throttle saturation. Together they don't form a direction vector because the units are different.
    Vector3f    _limit_vector_neu;         ///< Direction of controller saturation (zero when unlimited), used for anti-windup

    // terrain handling variables
    float    _pos_terrain_target_u_m;    ///< Target terrain altitude (m) in NEU frame relative to EKF origin
    float    _pos_terrain_u_m;           ///< Current terrain altitude estimate (m) in NEU frame, filtered toward target
    float    _vel_terrain_u_ms;          ///< Terrain velocity (m/s) in NEU frame for terrain-relative positioning
    float    _accel_terrain_u_mss;       ///< Terrain acceleration (m/s²) in NEU frame for smooth terrain following

    // offset handling variables
    Vector3p    _pos_offset_target_neu_m;      ///< Target position offset (m) in NEU frame for external position adjustments
    Vector3p    _pos_offset_neu_m;             ///< Current position offset (m) in NEU frame, filtered toward target
    Vector3f    _vel_offset_target_neu_ms;     ///< Target velocity offset (m/s) in NEU frame for external velocity adjustments
    Vector3f    _vel_offset_neu_ms;            ///< Current velocity offset (m/s) in NEU frame, filtered toward target
    Vector3f    _accel_offset_target_neu_mss;  ///< Target acceleration offset (m/s²) in NEU frame for external accel adjustments
    Vector3f    _accel_offset_neu_mss;         ///< Current acceleration offset (m/s²) in NEU frame
    uint32_t    _posvelaccel_offset_target_ne_ms;   ///< System time (ms) when NE offsets last set (3-second timeout)
    uint32_t    _posvelaccel_offset_target_u_ms;    ///< System time (ms) when U offset last set (3-second timeout)

    // ekf reset handling
    uint32_t    _ekf_ne_reset_ms;       ///< System time (ms) of last detected EKF horizontal position reset
    uint32_t    _ekf_u_reset_ms;        ///< System time (ms) of last detected EKF altitude reset

    // high vibration handling
    bool        _vibe_comp_enabled;     ///< True when high vibration compensation mode is active

    // angle max override, if zero then use ANGLE_MAX parameter
    float       _angle_max_override_rad;///< Maximum lean angle override (radians), zero = use ANGLE_MAX parameter

    // return true if on a real vehicle or SITL with lock-step scheduling
    bool has_good_timing(void) const;

private:
    // Internal log writer for PSCx (North, East, Down tracking).
    // Reduces duplication between Write_PSCN, PSCE, and PSCD.
    // Used for logging desired/target/actual position, velocity, and acceleration per axis.
    static void Write_PSCx(LogMessages ID, float pos_desired_m, float pos_target_m, float pos_m, 
                            float vel_desired_ms, float vel_target_ms, float vel_ms, 
                            float accel_desired_mss, float accel_target_mss, float accel_mss);

    // Internal log writer for PSOx (North, East, Down tracking).
    // Reduces duplication between Write_PSON, PSOE, and PSOD.
    // Used for logging the target/actual position, velocity, and acceleration offset per axis.
    static void Write_PSOx(LogMessages id, float pos_target_offset_m, float pos_offset_m,
                            float vel_target_offset_ms, float vel_offset_ms,
                            float accel_target_offset_mss, float accel_offset_mss);

    // singleton
    static AC_PosControl *_singleton;
};
