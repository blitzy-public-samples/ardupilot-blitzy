/**
 * @file AR_PosControl.h
 * @brief 2D position controller for ground and marine vehicles
 * 
 * @details This file implements a cascaded position→velocity→acceleration controller
 *          for Rover and marine vehicles. The controller uses input shaping to generate
 *          smooth paths that respect vehicle kinematic limits (speed, acceleration, jerk).
 *          
 *          Control Architecture:
 *          - Position loop: AC_P_2D converts position error → desired velocity
 *          - Velocity loop: AC_PID_2D converts velocity error → desired acceleration
 *          - Output stage: Converts NE acceleration to forward speed, lateral accel, turn rate
 *          
 *          The controller automatically handles EKF position resets and integrates with
 *          AR_AttitudeControl for final steering and throttle output.
 *          
 *          Coordinate Frames:
 *          - NE frame: North-East 2D frame relative to EKF origin
 *          - Positions in meters, velocities in m/s, accelerations in m/s/s
 *          - Turn rates in rad/s (positive = clockwise from above)
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 * 
 * Source: libraries/APM_Control/AR_PosControl.h
 */

#pragma once

#include <AP_Common/AP_Common.h>
#include <APM_Control/AR_AttitudeControl.h>
#include <AC_PID/AC_P_2D.h>            // P library (2-axis)
#include <AC_PID/AC_PID_2D.h>          // PID library (2-axis)

/**
 * @class AR_PosControl
 * @brief 2D position controller for ground and marine vehicles with input shaping
 * 
 * @details This class implements cascaded position/velocity control for Rover and marine vehicles:
 *          
 *          **Control Cascade:**
 *          1. Position error (P controller) → desired velocity
 *          2. Velocity error (PID controller) → desired acceleration
 *          3. Acceleration → forward speed, lateral acceleration, turn rate (for AR_AttitudeControl)
 *          
 *          **Input Shaping:**
 *          The input_* methods smoothly adjust targets to respect configured speed, acceleration,
 *          and jerk limits using S-curve velocity profiles. This ensures smooth path following
 *          without sudden jerks or overshoots.
 *          
 *          **EKF Integration:**
 *          Automatically detects and handles EKF NE position resets by adjusting internal targets.
 *          This prevents sudden jumps in desired velocity when the EKF corrects its position estimate.
 *          
 *          **Optional Avoidance:**
 *          Can integrate with AC_Avoid to adjust commanded velocity for obstacle avoidance
 *          (integration handled by vehicle code).
 *          
 *          **Outputs:**
 *          - Desired forward speed (m/s) - for throttle control
 *          - Desired lateral acceleration (m/s/s) - for steering control
 *          - Desired turn rate (rad/s) - alternative steering control
 *          
 *          These outputs are consumed by AR_AttitudeControl for final actuator commands.
 *          
 *          **Units and Conventions:**
 *          - Positions: meters (Vector2p for high precision)
 *          - Velocities: m/s (Vector2f)
 *          - Accelerations: m/s/s (Vector2f)
 *          - Turn rates: rad/s (float, positive = clockwise from above)
 *          - Lateral acceleration: m/s/s (float)
 *          
 *          **Typical Usage Pattern:**
 *          ```cpp
 *          // Initialize controller to current state
 *          pos_control.init();
 *          
 *          // Set kinematic limits
 *          pos_control.set_limits(5.0f, 2.0f, 2.0f, 1.0f); // speed, accel, lat_accel, jerk
 *          
 *          // In main loop: provide position target with input shaping
 *          Vector2p target_pos(100.0, 50.0); // 100m north, 50m east of EKF origin
 *          pos_control.input_pos_target(target_pos, dt);
 *          pos_control.update(dt);
 *          
 *          // Get outputs for attitude controller
 *          float speed = pos_control.get_desired_speed();
 *          float turn_rate = pos_control.get_desired_turn_rate_rads();
 *          ```
 * 
 * @warning Kinematic limits (speed_max, accel_max, lat_accel_max, jerk_max) must be set
 *          appropriately for vehicle characteristics. Excessive limits can cause instability
 *          or tracking errors. Insufficient limits cause sluggish response.
 * 
 * @note Input shaping ensures smooth motion by respecting acceleration and jerk limits.
 *       Use set_pos_vel_accel_target() only for externally pre-shaped paths (e.g., from
 *       a path planner that already respects kinematic constraints).
 * 
 * @note EKF position resets are handled automatically via init_ekf_xy_reset() and
 *       handle_ekf_xy_reset() called from update(). No special handling required by caller.
 * 
 * @note Integrates with AR_AttitudeControl singleton for final steering/throttle output.
 *       Ensure AR_AttitudeControl is properly initialized before using this controller.
 * 
 * @see AR_AttitudeControl
 * @see AC_P_2D
 * @see AC_PID_2D
 * 
 * Source: libraries/APM_Control/AR_PosControl.h
 */
class AR_PosControl {
public:

    /**
     * @brief Constructor for AR_PosControl
     * 
     * @details Initializes the 2D position controller with reference to the attitude controller.
     *          Sets up internal P and PID controllers for position and velocity control.
     *          Registers this instance as the singleton.
     * 
     * @param[in] atc Reference to AR_AttitudeControl instance for steering/throttle output
     * 
     * @note Typically called during vehicle initialization in Rover/mode.cpp
     * @note Only one instance should be created (singleton pattern)
     * 
     * Source: libraries/APM_Control/AR_PosControl.cpp
     */
    AR_PosControl(AR_AttitudeControl& atc);

    // do not allow copying
    CLASS_NO_COPY(AR_PosControl);

    /**
     * @brief Access the singleton instance of AR_PosControl
     * 
     * @details Provides access to the single AR_PosControl instance created by vehicle code.
     *          
     * @return Pointer to singleton AR_PosControl instance, or nullptr if not yet created
     * 
     * @note Singleton is created and owned by vehicle code during initialization
     * @note Returns nullptr if called before vehicle creates the instance
     * 
     * @see AR_PosControl::AR_PosControl()
     */
    static AR_PosControl *get_singleton() { return _singleton; }

    /**
     * @brief Main position controller update loop
     * 
     * @details This is the primary update method that must be called at the main loop rate
     *          (typically 50Hz for Rover). Performs the complete control cascade:
     *          1. Check for and handle EKF position resets
     *          2. Calculate position error from current EKF position to target
     *          3. Run position P controller: position error → desired velocity
     *          4. Run velocity PID controller: velocity error → desired acceleration
     *          5. Convert NE acceleration to forward speed, lateral accel, turn rate
     *          
     *          Updates internal state and prepares outputs for attitude controller.
     * 
     * @param[in] dt Time step in seconds since last update (typically 0.02s for 50Hz)
     * 
     * @note Must be called regularly at main loop rate for proper control operation
     * @note Automatically handles EKF position resets to prevent sudden velocity jumps
     * @note Updates _desired_speed, _desired_turn_rate_rads, _desired_lat_accel outputs
     * 
     * @warning If not called regularly, is_active() will return false and controller
     *          may be considered inactive by vehicle code
     * 
     * @see init()
     * @see get_desired_speed()
     * @see get_desired_turn_rate_rads()
     * @see get_desired_lat_accel()
     * 
     * Source: libraries/APM_Control/AR_PosControl.cpp
     */
    void update(float dt);

    /**
     * @brief Check if controller is actively running
     * 
     * @details Returns true if update() has been called recently (within last 400ms).
     *          Used by vehicle code to determine if position control is active or if
     *          manual control should take over.
     * 
     * @return true if update() called within last 400ms, false otherwise
     * 
     * @note Threshold of 400ms allows for some jitter in loop timing while still
     *       detecting loss of control updates
     * 
     * Source: libraries/APM_Control/AR_PosControl.cpp
     */
    bool is_active() const;

    /**
     * @brief Configure kinematic limits for motion planning
     * 
     * @details Sets maximum speed, acceleration, lateral acceleration, and jerk limits
     *          used by input shaping to generate smooth, achievable trajectories.
     *          These limits should match the vehicle's physical capabilities.
     *          
     *          Input shaping uses these limits to:
     *          - Limit forward/backward speed to speed_max
     *          - Limit forward/backward acceleration to accel_max
     *          - Limit lateral acceleration (cornering) to lat_accel_max
     *          - Limit jerk (rate of acceleration change) to jerk_max for smooth motion
     * 
     * @param[in] speed_max Maximum forward speed in m/s (must be > 0)
     * @param[in] accel_max Maximum forward/backward acceleration in m/s/s (must be > 0)
     * @param[in] lat_accel_max Maximum lateral acceleration in m/s/s (must be > 0)
     * @param[in] jerk_max Maximum jerk in m/s/s/s for input shaping (must be > 0)
     * 
     * @note Should be called before entering position control modes or when parameters change
     * @note Typical values: speed_max=5.0, accel_max=2.0, lat_accel_max=2.0, jerk_max=1.0
     * 
     * @warning Setting limits too high can cause instability or tracking errors
     * @warning Setting limits too low causes sluggish response and large tracking errors
     * @warning Limits must be positive and non-zero to avoid division by zero
     * 
     * Source: libraries/APM_Control/AR_PosControl.cpp
     */
    void set_limits(float speed_max, float accel_max, float lat_accel_max, float jerk_max);

    /**
     * @brief Update vehicle turn radius and pivot capability
     * 
     * @details Provides vehicle-specific turn characteristics to the position controller.
     *          Used to calculate achievable turn rates and lateral accelerations.
     *          Should be called regularly as these parameters may change with speed or mode.
     *          
     *          Turn radius affects:
     *          - Maximum achievable turn rate at a given speed
     *          - Lateral acceleration calculations
     *          - Path following performance in tight turns
     * 
     * @param[in] turn_radius Minimum turn radius in meters (must be > 0)
     * @param[in] pivot_possible True if vehicle can pivot/rotate in place (skid-steer),
     *                           false for Ackermann steering vehicles
     * 
     * @note Should be updated regularly by vehicle code (typically every loop)
     * @note For Ackermann steering: turn_radius based on wheelbase and max steering angle
     * @note For skid-steer: turn_radius can be very small, pivot_possible=true
     * 
     * @warning Incorrect turn_radius can cause unrealistic turn rate commands
     * 
     * Source: libraries/APM_Control/AR_PosControl.cpp
     */
    void set_turn_params(float turn_radius, bool pivot_possible);

    /**
     * @brief Enable reverse operation mode
     * 
     * @details When reversed is true, vehicle will move backward toward target instead
     *          of forward. Used for backing up or reverse navigation modes.
     *          Affects sign of desired speed output.
     * 
     * @param[in] reversed True to enable reverse operation, false for normal forward operation
     * 
     * @note Does not affect position/velocity control logic, only output speed sign
     * 
     * Source: libraries/APM_Control/AR_PosControl.cpp
     */
    void set_reversed(bool reversed) { _reversed = reversed; }

    /**
     * @brief Get current reverse operation state
     * 
     * @return True if vehicle is in reverse mode, false if in forward mode
     * 
     * @see set_reversed()
     */
    bool get_reversed() { return _reversed; }

    /**
     * @brief Get configured maximum forward speed limit
     * @return Maximum speed in m/s
     * @see set_limits()
     */
    float get_speed_max() const { return _speed_max; }
    
    /**
     * @brief Get configured maximum forward/backward acceleration limit
     * @return Maximum acceleration in m/s/s
     * @see set_limits()
     */
    float get_accel_max() const { return _accel_max; }
    
    /**
     * @brief Get configured maximum lateral acceleration limit
     * @return Maximum lateral acceleration in m/s/s
     * @see set_limits()
     */
    float get_lat_accel_max() const { return _lat_accel_max; }
    
    /**
     * @brief Get configured maximum jerk limit
     * @return Maximum jerk in m/s/s/s
     * @see set_limits()
     */
    float get_jerk_max() const { return _jerk_max; }

    /**
     * @brief Initialize controller to current position, velocity, and attitude
     * 
     * @details Initializes all internal targets (position, velocity, acceleration) to current
     *          vehicle state from EKF. This prevents sudden jumps when position control starts.
     *          Must be called before using input shaping methods (input_pos_target, etc.).
     *          
     *          Initialization procedure:
     *          1. Get current position from EKF (NE offset from origin)
     *          2. Get current velocity from EKF (NE frame, m/s)
     *          3. Set all targets to current state (zero error)
     *          4. Initialize EKF reset tracking
     * 
     * @return true on success, false if EKF not ready or position unavailable
     * 
     * @note Must be called after EKF has converged and position estimate is valid
     * @note Call this when entering a position control mode from a non-position mode
     * @note If init() returns false, position control should not be activated
     * 
     * @warning Do not use input shaping methods without calling init() first, as targets
     *          will be undefined and can cause sudden vehicle motion
     * 
     * @see input_pos_target()
     * @see input_pos_vel_target()
     * @see input_pos_vel_accel_target()
     * 
     * Source: libraries/APM_Control/AR_PosControl.cpp
     */
    bool init();

    /**
     * @brief Set position target with input shaping
     * 
     * @details Smoothly adjusts internal position target toward the provided position using
     *          S-curve input shaping. Respects configured speed, acceleration, and jerk limits
     *          to generate a smooth, achievable trajectory. Velocity and acceleration targets
     *          are automatically calculated by the input shaper.
     *          
     *          Input shaping ensures:
     *          - Gradual acceleration from current velocity
     *          - Velocity does not exceed speed_max
     *          - Acceleration does not exceed accel_max
     *          - Jerk (rate of accel change) does not exceed jerk_max
     *          - Smooth deceleration to reach target position with zero velocity
     * 
     * @param[in] pos Target position as NE offset from EKF origin in meters (Vector2p for precision)
     * @param[in] dt Time step in seconds since last call (typically 0.02s for 50Hz)
     * 
     * @note init() must be called before first use to initialize targets to current state
     * @note Call at regular intervals (main loop rate) for smooth control
     * @note Position is high-precision Vector2p to avoid accumulation errors over large distances
     * 
     * @warning Sudden large changes in target position may take time to reach due to
     *          acceleration and speed limits (this is intentional for smooth motion)
     * 
     * @see init()
     * @see set_limits()
     * @see update()
     * 
     * Source: libraries/APM_Control/AR_PosControl.cpp
     */
    void input_pos_target(const Vector2p &pos, float dt);
    
    /**
     * @brief Set position and velocity targets with input shaping
     * 
     * @details Smoothly adjusts internal targets toward the provided position and velocity
     *          using S-curve input shaping. Useful when following a path with desired velocity
     *          at each waypoint (e.g., higher speed on straightaways, slower in turns).
     *          Acceleration target is automatically calculated by the input shaper.
     *          
     *          Input shaping ensures smooth transition to desired velocity while respecting
     *          acceleration and jerk limits.
     * 
     * @param[in] pos Target position as NE offset from EKF origin in meters
     * @param[in] vel Desired velocity in m/s in NE frame (Vector2f)
     * @param[in] dt Time step in seconds since last call
     * 
     * @note Use this when you want specific velocity at target position (feedforward)
     * @note Velocity magnitude will be limited by speed_max if exceeded
     * @note init() must be called before first use
     * 
     * @see input_pos_target()
     * @see input_pos_vel_accel_target()
     * 
     * Source: libraries/APM_Control/AR_PosControl.cpp
     */
    void input_pos_vel_target(const Vector2p &pos, const Vector2f &vel, float dt);
    
    /**
     * @brief Set full state target (position, velocity, acceleration) with input shaping
     * 
     * @details Smoothly adjusts internal targets toward the provided full state using S-curve
     *          input shaping. Provides complete feedforward when following a pre-planned
     *          trajectory with known position, velocity, and acceleration at each point.
     *          
     *          Input shaping ensures smooth transition to desired acceleration while
     *          respecting jerk limits.
     * 
     * @param[in] pos Target position as NE offset from EKF origin in meters
     * @param[in] vel Desired velocity in m/s in NE frame
     * @param[in] accel Desired acceleration in m/s/s in NE frame (Vector2f)
     * @param[in] dt Time step in seconds since last call
     * 
     * @note Use this for trajectory following with complete feedforward control
     * @note Acceleration magnitude will be limited by accel_max if exceeded
     * @note Provides best tracking performance when following smooth trajectories
     * @note init() must be called before first use
     * 
     * @see input_pos_target()
     * @see input_pos_vel_target()
     * @see set_pos_vel_accel_target()
     * 
     * Source: libraries/APM_Control/AR_PosControl.cpp
     */
    void input_pos_vel_accel_target(const Vector2p &pos, const Vector2f &vel, const Vector2f &accel, float dt);

    /**
     * @brief Set targets directly without input shaping
     * 
     * @details Sets position, velocity, and acceleration targets directly without S-curve
     *          input shaping. Use this ONLY when targets come from an external path planner
     *          that already respects vehicle kinematic constraints and generates smooth,
     *          achievable trajectories.
     *          
     *          Unlike input_* methods, this does NOT:
     *          - Apply acceleration limiting
     *          - Apply jerk limiting  
     *          - Smooth transitions between targets
     *          
     *          The controller will attempt to track these targets directly, which can cause
     *          sudden motions if targets change abruptly or exceed vehicle capabilities.
     * 
     * @param[in] pos Target position as NE offset from EKF origin in meters
     * @param[in] vel Desired velocity in m/s in NE frame (feedforward)
     * @param[in] accel Desired acceleration in m/s/s in NE frame (feedforward)
     * 
     * @note Only use this when external path planner guarantees smooth, achievable trajectories
     * @note For manual waypoint navigation, use input_pos_target() instead for smooth motion
     * @note No dt parameter needed since no input shaping is performed
     * 
     * @warning Can cause sudden jerky motion if targets are not pre-smoothed
     * @warning Can cause tracking errors if targets exceed vehicle kinematic limits
     * 
     * @see input_pos_vel_accel_target()
     * 
     * Source: libraries/APM_Control/AR_PosControl.cpp
     */
    void set_pos_vel_accel_target(const Vector2p &pos, const Vector2f &vel, const Vector2f &accel);

    /**
     * @brief Get desired forward speed output for throttle control
     * 
     * @details Returns the commanded forward speed in m/s calculated by the position controller.
     *          This output is the primary throttle control signal for AR_AttitudeControl.
     *          Positive values indicate forward motion, negative values indicate reverse.
     *          Sign is affected by _reversed flag.
     * 
     * @return Desired forward speed in m/s (positive = forward, negative = backward)
     * 
     * @note Updated by update() method each loop
     * @note Magnitude limited by speed_max configured in set_limits()
     * @note Sign affected by set_reversed()
     * 
     * @see update()
     * @see get_desired_turn_rate_rads()
     * @see get_desired_lat_accel()
     * 
     * Source: libraries/APM_Control/AR_PosControl.cpp
     */
    float get_desired_speed() const { return _desired_speed; }
    
    /**
     * @brief Get desired turn rate output for steering control
     * 
     * @details Returns the commanded turn rate in rad/s calculated by the position controller.
     *          This output can be used by AR_AttitudeControl for heading rate control.
     *          Positive values indicate clockwise rotation (when viewed from above),
     *          negative values indicate counter-clockwise rotation.
     * 
     * @return Desired turn rate in rad/s (positive = clockwise, negative = counter-clockwise)
     * 
     * @note Updated by update() method each loop
     * @note Alternative to lateral acceleration for steering control
     * @note Convention: positive = clockwise from above (right turn)
     * 
     * @see update()
     * @see get_desired_speed()
     * @see get_desired_lat_accel()
     * 
     * Source: libraries/APM_Control/AR_PosControl.cpp
     */
    float get_desired_turn_rate_rads() const { return _desired_turn_rate_rads; }
    
    /**
     * @brief Get desired lateral acceleration output for steering control
     * 
     * @details Returns the commanded lateral acceleration in m/s/s calculated by the position
     *          controller. This output is used by AR_AttitudeControl for steering control.
     *          Lateral acceleration is the centripetal acceleration during turns.
     *          Positive values indicate acceleration to the right, negative to the left.
     * 
     * @return Desired lateral acceleration in m/s/s (positive = right, negative = left)
     * 
     * @note Updated by update() method each loop
     * @note Magnitude limited by lat_accel_max configured in set_limits()
     * @note Primary steering control signal for AR_AttitudeControl
     * @note Related to turn_rate via: lat_accel = turn_rate * forward_speed
     * 
     * @see update()
     * @see get_desired_speed()
     * @see get_desired_turn_rate_rads()
     * 
     * Source: libraries/APM_Control/AR_PosControl.cpp
     */
    float get_desired_lat_accel() const { return _desired_lat_accel; }

    /**
     * @brief Get current position target
     * 
     * @details Returns the current internal position target that the controller is tracking.
     *          This is the output of input shaping (for input_* methods) or the directly
     *          set target (for set_* methods).
     * 
     * @return Reference to position target as NE offset from EKF origin in meters (Vector2p)
     * 
     * @note Position target is high-precision Vector2p to avoid accumulation errors
     * @note Updated by input_* or set_* methods
     * @note In NE frame relative to EKF origin
     * 
     * @see input_pos_target()
     * @see set_pos_vel_accel_target()
     */
    const Vector2p& get_pos_target() const { return _pos_target; }

    /**
     * @brief Get feedforward desired velocity vector
     * 
     * @details Returns the desired velocity vector (feedforward term) in m/s in NE frame.
     *          This is the velocity target from input shaping or external path planner,
     *          used as feedforward in the velocity controller.
     *          
     *          The velocity controller combines this feedforward with feedback from
     *          velocity error to generate the desired acceleration.
     * 
     * @return Desired velocity vector in m/s in NE frame (Vector2f)
     * 
     * @note Feedforward velocity improves tracking of moving targets or curved paths
     * @note In NE frame (North component in x, East component in y)
     * @note Valid only if _vel_desired_valid is true
     * 
     * Source: libraries/APM_Control/AR_PosControl.cpp
     */
    Vector2f get_desired_velocity() const;

    /**
     * @brief Get feedforward desired acceleration vector
     * 
     * @details Returns the desired acceleration vector (feedforward term) in m/s/s in NE frame.
     *          This is the acceleration target from input shaping or external path planner,
     *          used as feedforward to improve tracking performance.
     *          
     *          The output stage combines this feedforward acceleration with feedback
     *          from the velocity PID controller.
     * 
     * @return Desired acceleration vector in m/s/s in NE frame (Vector2f)
     * 
     * @note Feedforward acceleration improves tracking of accelerating targets or trajectories
     * @note In NE frame (North component in x, East component in y)
     * @note Valid only if _accel_desired_valid is true
     * 
     * Source: libraries/APM_Control/AR_PosControl.cpp
     */
    Vector2f get_desired_accel() const;

    /**
     * @brief Get position tracking error vector
     * 
     * @details Returns the current position error as a vector from vehicle's current position
     *          (from EKF) to the target position. Used for monitoring tracking performance
     *          and by the position P controller to generate desired velocity.
     *          
     *          Error vector points from current position toward target:
     *          error = target_position - current_position
     * 
     * @return Position error vector in meters in NE frame (Vector2p)
     * 
     * @note Error magnitude indicates how far vehicle is from target position
     * @note Error direction indicates which direction to move to reach target
     * @note High-precision Vector2p used to handle large distances accurately
     * 
     * @see get_pos_target()
     * 
     * Source: libraries/APM_Control/AR_PosControl.cpp
     */
    Vector2p get_pos_error() const;

    /**
     * @brief Access position P controller for tuning or diagnostics
     * 
     * @details Provides direct access to the 2D position P controller that converts
     *          position error to desired velocity. Useful for:
     *          - Parameter tuning and autotuning
     *          - Diagnostics and performance analysis
     *          - Reading internal controller state
     *          
     *          The P controller has separate gains for X (North) and Y (East) axes.
     * 
     * @return Reference to AC_P_2D position controller instance
     * 
     * @note Direct modification of controller parameters should be done carefully
     * @note Controller parameters typically loaded from AP_Param system
     * 
     * @see AC_P_2D
     * @see get_vel_pid()
     * 
     * Source: libraries/APM_Control/AR_PosControl.h
     */
    AC_P_2D& get_pos_p() { return _p_pos; }
    
    /**
     * @brief Access velocity PID controller for tuning or diagnostics
     * 
     * @details Provides direct access to the 2D velocity PID controller that converts
     *          velocity error to desired acceleration. Useful for:
     *          - Parameter tuning and autotuning
     *          - Diagnostics and performance analysis
     *          - Reading internal controller state (P, I, D terms, saturation flags)
     *          
     *          The PID controller has separate gains for X (North) and Y (East) axes,
     *          with integrator anti-windup and derivative filtering.
     * 
     * @return Reference to AC_PID_2D velocity controller instance
     * 
     * @note Direct modification of controller parameters should be done carefully
     * @note Controller parameters typically loaded from AP_Param system
     * @note Integrator can be reset via controller methods if needed
     * 
     * @see AC_PID_2D
     * @see get_pos_p()
     * 
     * Source: libraries/APM_Control/AR_PosControl.h
     */
    AC_PID_2D& get_vel_pid() { return _pid_vel; }

    /**
     * @brief Write PSC (position control) binary log messages
     * 
     * @details Logs position controller state for post-flight analysis and tuning.
     *          PSC log messages include:
     *          - Current position and target position
     *          - Velocity error and desired velocity
     *          - Position error magnitude
     *          - Desired speed, turn rate, and lateral acceleration outputs
     *          
     *          Log data is written in binary format to dataflash/SD card for later
     *          analysis with log viewing tools.
     * 
     * @note Should be called regularly (typically once per main loop) when logging is enabled
     * @note Log message format defined in libraries/AP_Logger/LogStructure.h
     * @note PSC logs are essential for tuning position control gains
     * @note Can be viewed in Mission Planner, MAVExplorer, or other log analysis tools
     * 
     * @see AP_Logger
     * 
     * Source: libraries/APM_Control/AR_PosControl.cpp
     */
    void write_log();

    /**
     * @brief Parameter table for AP_Param system
     * 
     * @details Defines parameters that can be configured via ground station:
     *          - Position P controller gains (PSCN_POS_XY_P)
     *          - Velocity PID controller gains (PSCN_VEL_XY_P, PSCN_VEL_XY_I, PSCN_VEL_XY_D)
     *          - Velocity PID filter settings (PSCN_VEL_XY_FILT, PSCN_VEL_XY_FLTE)
     *          - Velocity PID integrator limits (PSCN_VEL_XY_IMAX)
     *          
     *          Parameters are stored in EEPROM and loaded at startup.
     * 
     * @note Parameter naming uses PSCN_ prefix (Position Single Controller Navigation)
     * @note Separate X and Y gains allow tuning for asymmetric vehicle dynamics
     * 
     * @see AP_Param
     * 
     * Source: libraries/APM_Control/AR_PosControl.cpp
     */
    static const struct AP_Param::GroupInfo var_info[];

private:

    /**
     * @brief Singleton instance pointer
     * @note Set by constructor, accessed via get_singleton()
     */
    static AR_PosControl *_singleton;

    /**
     * @brief Initialize EKF position reset tracking
     * @details Records current EKF reset timestamp to detect future resets
     * @note Called by init() and update()
     * Source: libraries/APM_Control/AR_PosControl.cpp
     */
    void init_ekf_xy_reset();
    
    /**
     * @brief Handle EKF position reset events
     * @details Detects EKF position resets and adjusts internal targets to prevent
     *          sudden velocity commands. When EKF corrects position estimate, targets
     *          are shifted by the same amount to maintain smooth control.
     * @note Called by update() each loop
     * Source: libraries/APM_Control/AR_PosControl.cpp
     */
    void handle_ekf_xy_reset();

    // ========== Private Member Variables ==========
    
    // Reference to attitude controller for final output
    AR_AttitudeControl &_atc;       ///< Rover attitude control library for steering/throttle

    // Control loops (parameters loaded from EEPROM via var_info)
    AC_P_2D   _p_pos;               ///< Position P controller: position error → desired velocity
    AC_PID_2D _pid_vel;             ///< Velocity PID controller: velocity error → desired acceleration

    // Kinematic limits configured via set_limits()
    float _speed_max;               ///< Maximum forward speed in m/s
    float _accel_max;               ///< Maximum forward/backward acceleration in m/s/s
    float _lat_accel_max;           ///< Maximum lateral acceleration in m/s/s (for turns)
    float _jerk_max;                ///< Maximum jerk in m/s/s/s (for input shaping smoothness)
    float _turn_radius;             ///< Vehicle minimum turn radius in meters

    // Position and velocity targets (set by input_* or set_* methods)
    Vector2p _pos_target;           ///< Position target as NE offset in meters from EKF origin
    Vector2f _vel_desired;          ///< Desired velocity in m/s NE frame (feedforward from input shaping)
    Vector2f _vel_target;           ///< Velocity target in m/s NE frame (for feedback control)
    Vector2f _accel_desired;        ///< Desired accel in m/s/s NE frame (feedforward from input shaping)
    Vector2f _accel_target;         ///< Accel target in m/s/s NE frame (for feedback control)
    bool _pos_target_valid;         ///< True if _pos_target has been initialized
    bool _vel_desired_valid;        ///< True if _vel_desired is valid (has feedforward)
    bool _accel_desired_valid;      ///< True if _accel_desired is valid (has feedforward)

    // Navigation state
    uint32_t _last_update_ms;       ///< System time of last update() call (for is_active check)
    bool _reversed;                 ///< True if vehicle should move in reverse toward target

    // Control outputs (calculated by update(), read by vehicle code)
    float _desired_speed;           ///< Desired forward speed in m/s (for throttle control)
    float _desired_turn_rate_rads;  ///< Desired turn rate in rad/s (positive = clockwise from above)
    float _desired_lat_accel;       ///< Desired lateral acceleration in m/s/s (for steering control)

    // EKF reset handling (prevents sudden velocity jumps when EKF corrects position)
    uint32_t _ekf_xy_reset_ms;      ///< System time of last recorded EKF XY position reset
};
